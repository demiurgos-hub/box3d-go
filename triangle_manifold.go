// Port of src/triangle_manifold.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math"

// triangleData is b3TriangleData.
type triangleData struct {
	v1, v2, v3 Vec3
	e1, e2, e3 Vec3
	plane      Plane
	flags      int32
}

// triangleFeatures is s_triangleFeatures.
// Indexed by the 3-bit vertex mask
var triangleFeatures = [8]TriangleFeature{
	FeatureNone,         // 000  (unreachable)
	FeatureVertex1,      // 001
	FeatureVertex2,      // 010
	FeatureEdge1,        // 011  v1,v2
	FeatureVertex3,      // 100
	FeatureEdge3,        // 101  v1,v3
	FeatureEdge2,        // 110  v2,v3
	FeatureTriangleFace, // 111
}

// getTriangleFeature is b3GetTriangleFeature.
func getTriangleFeature(cache *SimplexCache) TriangleFeature {
	count := int(cache.Count)
	assert(0 < count && count < 4, "0 < count && count < 4")

	// Bit i set means triangle vertex i participates in the simplex.
	mask := 0
	for i := range count {
		assert(cache.IndexA[i] < 3, "cache->indexA[i] < 3")
		mask |= 1 << cache.IndexA[i]
	}

	return triangleFeatures[mask]
}

// CollideTriangleAndSphere collides a triangle and sphere. Normal points from triangle to sphere.
// PORT: triangleA is a slice of (at least) the three triangle vertices.
func CollideTriangleAndSphere(manifold *LocalManifold, capacity int, triangleA []Vec3, sphereB *Sphere) {
	manifold.PointCount = 0

	if capacity == 0 {
		return
	}

	center := sphereB.Center
	v1, v2, v3 := triangleA[0], triangleA[1], triangleA[2]
	plane := makePlaneFromPoints(v1, v2, v3)

	offset := planeSeparation(plane, center)
	if offset < 0.0 {
		// Cull back side collision
		return
	}

	radius := sphereB.Radius

	// Closest point on triangle to sphere center
	closest := closestPointOnTriangle(v1, v2, v3, center)

	// Test separating axis
	squaredDistance := DistanceSquared(closest.point, center)
	speculativeDistance := SpeculativeDistance()
	maxDistance := radius + speculativeDistance
	if squaredDistance > maxDistance*maxDistance {
		return
	}

	distance := sqrtf(squaredDistance)

	// Normal points from triangle to sphere
	var normal Vec3
	if distance*distance > 1000.0*fltMin {
		normal = MulSV(1.0/distance, Sub(center, closest.point))
	} else {
		normal = Normalize(Cross(Sub(v2, v1), Sub(v3, v1)))
	}

	// contact point mid-way
	// p = 0.5 * (c + q) - 0.5 * r * n
	contactPoint := MulSV(0.5, Add(Sub(center, MulSV(radius, normal)), closest.point))

	manifold.Normal = normal
	manifold.PointCount = 1
	manifold.Feature = closest.feature
	manifold.SquaredDistance = squaredDistance

	mp := &manifold.Points[0]
	mp.Point = contactPoint
	mp.Separation = distance - radius
	mp.Pair = featurePairSingle
}

// clipSegmentToTriangleFace is b3ClipSegmentToTriangleFace.
func clipSegmentToTriangleFace(segment *[2]clipVertex, points []Vec3, plane Plane) bool {
	vertex1 := points[2]
	for i := range 3 {
		vertex2 := points[i]
		tangent := Normalize(Sub(vertex2, vertex1))
		binormal := Cross(tangent, plane.Normal)

		clipPlane := makePlaneFromNormalAndPoint(binormal, vertex1)

		vertexCount := 0
		p1 := segment[0]
		p2 := segment[1]

		distance1 := planeSeparation(clipPlane, p1.position)
		distance2 := planeSeparation(clipPlane, p2.position)

		// If the points are behind the plane
		if distance1 <= 0.0 {
			segment[vertexCount] = p1
			vertexCount++
		}
		if distance2 <= 0.0 {
			segment[vertexCount] = p2
			vertexCount++
		}

		// If the points are on different sides of the plane
		if (distance1 > 0.0) != (distance2 > 0.0) {
			// Find intersection point of edge and plane
			t := distance1 / (distance1 - distance2)
			segment[vertexCount].position = Lerp(p1.position, p2.position, t)
			if distance1 > 0.0 {
				segment[vertexCount].pair = p1.pair
			} else {
				segment[vertexCount].pair = p2.pair
			}
			vertexCount++
		}

		if vertexCount != 2 {
			return false
		}

		vertex1 = vertex2
	}

	return true
}

// queryTriangleFaceAndCapsule is b3QueryTriangleFaceAndCapsule.
func queryTriangleFaceAndCapsule(plane Plane, capsule *Capsule) separatingAxis {
	separation1 := planeSeparation(plane, capsule.Center1)
	separation2 := planeSeparation(plane, capsule.Center2)

	if separation1 < separation2 {
		return separatingAxis{
			normal:     plane.Normal,
			separation: separation1,
			indexA:     0,
			indexB:     0,
		}
	}

	return separatingAxis{
		normal:     plane.Normal,
		separation: separation2,
		indexA:     0,
		indexB:     1,
	}
}

// queryTriangleAndCapsuleEdges is b3QueryTriangleAndCapsuleEdges.
func queryTriangleAndCapsuleEdges(vertices []Vec3, plane Plane, capsule *Capsule) separatingAxis {
	// Work in the local space of the capsule
	p1 := capsule.Center1
	p2 := capsule.Center2
	capsuleEdge := Sub(p2, p1)

	// Find axis of minimum penetration
	maxNormal := Vec3Zero
	maxSeparation := -fltMax
	maxIndex1 := int32(NullIndex)
	maxIndex2 := int32(NullIndex)
	const squaredToleranceRoot float32 = 0.005
	squaredTolerance := squaredToleranceRoot * squaredToleranceRoot

	edgeIndex := 2
	a := Dot(capsuleEdge, plane.Normal)
	v1 := vertices[2]
	for index := range 3 {
		v2 := vertices[index]
		triangleEdge := Sub(v2, v1)
		sideNormal := Normalize(Cross(triangleEdge, plane.Normal))

		// Pretend the triangle edge embeds a zero area face with a side normal. This
		// provides a way to find an edge-edge normal that points outward from
		// the triangle.
		b := Dot(capsuleEdge, sideNormal)

		// Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
		if float32(a*a)+float32(b*b) < squaredTolerance*LengthSquared(capsuleEdge) {
			v1 = v2
			edgeIndex = index
			continue
		}

		// Similar to hull vs hull (b3QueryEdgeDirections)
		var axis Vec3
		if a*b <= 0.0 {
			t := b / (b - a)
			axis = Lerp(sideNormal, plane.Normal, t)
		} else {
			t := b / (a + b)
			axis = Lerp(sideNormal, Neg(plane.Normal), t)
		}

		validate(LengthSquared(axis) > 1000.0*fltMin, "b3LengthSquared( axis ) > 1000.0f * FLT_MIN")
		axis = Normalize(axis)
		separation := Dot(axis, Sub(p1, v1))

		if separation > maxSeparation {
			// Note: We don't exit early if we find a separating axis here since we want to
			// find the best one for caching and account for the convex radius later.
			maxNormal = axis
			maxSeparation = separation
			maxIndex1 = int32(edgeIndex)
			maxIndex2 = 0
		}

		v1 = v2
		edgeIndex = index
	}

	// Save result
	return separatingAxis{
		normal:     maxNormal,
		separation: maxSeparation,
		indexA:     maxIndex1,
		indexB:     maxIndex2,
	}
}

// buildTriangleAndCapsuleFaceContact is b3BuildTriangleAndCapsuleFaceContact.
func buildTriangleAndCapsuleFaceContact(manifold *LocalManifold, triangle []Vec3, plane Plane, capsule *Capsule) {
	assert(manifold.PointCount == 0, "manifold->pointCount == 0")

	var segment [2]clipVertex
	segment[0].position = capsule.Center1
	segment[0].separation = 0.0
	segment[0].pair = makeFeaturePair(featureShapeA, 0, featureShapeA, 0)
	segment[1].position = capsule.Center2
	segment[1].separation = 0.0
	segment[1].pair = makeFeaturePair(featureShapeA, 1, featureShapeA, 1)

	havePoints := clipSegmentToTriangleFace(&segment, triangle, plane)
	if havePoints == false {
		return
	}

	radius := capsule.Radius
	distance1 := planeSeparation(plane, segment[0].position)
	distance2 := planeSeparation(plane, segment[1].position)

	speculativeDistance := SpeculativeDistance()
	if distance1 > speculativeDistance+radius && distance2 > speculativeDistance+radius {
		return
	}

	// Average points. Half-way between capsule bottom and triangle plane.
	point1 := MulSub(segment[0].position, float32(0.5*(distance1+capsule.Radius)), plane.Normal)
	point2 := MulSub(segment[1].position, float32(0.5*(distance2+capsule.Radius)), plane.Normal)

	manifold.Normal = plane.Normal
	manifold.Feature = FeatureTriangleFace
	manifold.PointCount = 2

	pt := &manifold.Points[0]
	pt.Point = point1
	pt.Separation = distance1 - capsule.Radius
	pt.Pair = segment[0].pair

	pt = &manifold.Points[1]
	pt.Point = point2
	pt.Separation = distance2 - capsule.Radius
	pt.Pair = segment[1].pair
}

// buildTriangleAndCapsuleEdgeContact is b3BuildTriangleAndCapsuleEdgeContact.
func buildTriangleAndCapsuleEdgeContact(manifold *LocalManifold, triangle []Vec3, plane Plane,
	capsule *Capsule, query separatingAxis) {
	assert(0 <= query.indexA && query.indexA < 3, "0 <= query.indexA && query.indexA < 3")

	p1 := capsule.Center1
	p2 := capsule.Center2
	capsuleEdge := Sub(p2, p1)

	vs := triangle

	v1 := vs[query.indexA]
	v2 := vs[(query.indexA+1)%3]
	triangleEdge := Sub(v2, v1)

	sideNormal := Normalize(Cross(triangleEdge, plane.Normal))

	// Pretend the triangle edge embeds a zero area face with a side normal. This
	// provides a way to find an edge-edge normal that points outward from
	// the triangle.
	a := Dot(capsuleEdge, plane.Normal)
	b := Dot(capsuleEdge, sideNormal)

	// Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
	const squaredToleranceRoot float32 = 0.005
	squaredTolerance := squaredToleranceRoot * squaredToleranceRoot
	if float32(a*a)+float32(b*b) < squaredTolerance*LengthSquared(capsuleEdge) {
		return
	}

	// Similar to hull vs hull (b3QueryEdgeDirections)
	normal := query.normal
	result := LineDistance(v1, triangleEdge, p1, capsuleEdge)

	if result.Fraction1 < 0.0 || 1.0 < result.Fraction1 || result.Fraction2 < 0.0 || 1.0 < result.Fraction2 {
		// closest point beyond end points
		return
	}

	point := Lerp(result.Point1, MulSub(result.Point2, capsule.Radius, normal), 0.5)
	separation := Dot(normal, Sub(p1, v1))

	manifold.Normal = normal
	manifold.PointCount = 1

	edgesFeatures := [3]TriangleFeature{FeatureEdge1, FeatureEdge2, FeatureEdge3}
	manifold.Feature = edgesFeatures[query.indexA]

	pt := &manifold.Points[0]
	pt.Point = point
	pt.Separation = separation - capsule.Radius
	pt.Pair = makeFeaturePair(featureShapeA, int(query.indexA), featureShapeB, int(query.indexB))
}

// CollideTriangleAndCapsule collides a triangle and capsule. Normal points from triangle to capsule.
// PORT: triangleA is a slice of (at least) the three triangle vertices.
func CollideTriangleAndCapsule(manifold *LocalManifold, capacity int, triangleA []Vec3, capsuleB *Capsule,
	cache *SimplexCache) {
	manifold.PointCount = 0

	if capacity < 2 {
		return
	}

	v1, v2, v3 := triangleA[0], triangleA[1], triangleA[2]
	plane := makePlaneFromPoints(v1, v2, v3)
	capsuleCenter := Lerp(capsuleB.Center1, capsuleB.Center2, 0.5)

	offset := planeSeparation(plane, capsuleCenter)
	if offset < 0.0 {
		// Cull back side collision
		return
	}

	var distanceInput DistanceInput
	distanceInput.ProxyA = ShapeProxy{triangleA, 3, 0.0}
	distanceInput.ProxyB = ShapeProxy{capsulePoints(capsuleB), 2, 0.0}
	distanceInput.Transform = TransformIdentity
	distanceInput.UseRadii = false

	distanceOutput := ShapeDistance(&distanceInput, cache, nil)
	speculativeDistance := SpeculativeDistance()
	radius := capsuleB.Radius
	if distanceOutput.Distance > radius+speculativeDistance {
		// Shapes are separated, persist the cache
		return
	}

	if distanceOutput.Distance > 100.0*fltEpsilon {
		// Shallow penetration
		delta := Normalize(Sub(distanceOutput.PointB, distanceOutput.PointA))

		// Try to create two contact points if closest points difference is nearly parallel to face normal
		const kTolerance float32 = 0.2
		cosAngle := AbsFloat(Dot(plane.Normal, delta))
		if cosAngle > kTolerance {
			// Clip capsule segment against side planes of reference face
			var segment [2]clipVertex
			segment[0].position = capsuleB.Center1
			segment[0].separation = 0.0
			segment[0].pair = makeFeaturePair(featureShapeA, 0, featureShapeA, 0)
			segment[1].position = capsuleB.Center2
			segment[1].separation = 0.0
			segment[1].pair = makeFeaturePair(featureShapeA, 1, featureShapeA, 1)

			havePoints := clipSegmentToTriangleFace(&segment, triangleA, plane)

			if havePoints == true {
				distance1 := planeSeparation(plane, segment[0].position)
				distance2 := planeSeparation(plane, segment[1].position)

				normal := plane.Normal
				point1 := MulSub(segment[0].position, float32(0.5*(radius+distance1)), normal)
				point2 := MulSub(segment[1].position, float32(0.5*(radius+distance2)), normal)

				// Normal points from triangle to capsule
				manifold.Normal = normal
				manifold.Feature = FeatureTriangleFace
				manifold.PointCount = 2

				mp := &manifold.Points[0]
				mp.Point = point1
				mp.Separation = distance1 - radius
				mp.Pair = segment[0].pair

				mp = &manifold.Points[1]
				mp.Point = point2
				mp.Separation = distance2 - radius
				mp.Pair = segment[1].pair

				return
			}
		}

		// Create contact from closest points.
		point := Lerp(distanceOutput.PointA, MulSub(distanceOutput.PointB, radius, delta), 0.5)

		// Normal points from triangle to capsule.
		manifold.Normal = delta
		manifold.PointCount = 1
		manifold.Feature = getTriangleFeature(cache)

		mp := &manifold.Points[0]
		mp.Point = point
		mp.Separation = distanceOutput.Distance - radius
		mp.Pair = featurePairSingle

		return
	}

	// Deep penetration

	faceQuery := queryTriangleFaceAndCapsule(plane, capsuleB)
	if faceQuery.separation > radius {
		// Shapes are separated. Should be impossible for a reasonable capsule radius.
		return
	}

	edgeQuery := queryTriangleAndCapsuleEdges(triangleA, plane, capsuleB)
	if edgeQuery.separation > radius {
		// Shapes are separated. Should be impossible for a reasonable capsule radius.
		return
	}

	// Create face contact
	faceSeparation := faceQuery.separation - radius
	buildTriangleAndCapsuleFaceContact(manifold, triangleA, plane, capsuleB)
	validate(manifold.PointCount == 0 || manifold.PointCount == 2, "manifold->pointCount == 0 || manifold->pointCount == 2")
	if manifold.PointCount == 2 {
		// This becomes the clipped separation.
		faceSeparation = MinFloat(manifold.Points[0].Separation, manifold.Points[1].Separation)
	}

	if edgeQuery.indexA == NullIndex {
		// No valid edge contact.
		return
	}

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	linearSlop := LinearSlop()
	edgeSeparation := edgeQuery.separation - radius
	if manifold.PointCount == 0 || edgeSeparation > faceSeparation+linearSlop {
		// Edge contact
		buildTriangleAndCapsuleEdgeContact(manifold, triangleA, plane, capsuleB, edgeQuery)
	}
}

// getTriangleSupport is b3GetTriangleSupport.
func getTriangleSupport(points []Vec3, direction Vec3) int {
	index := 0
	distance := Dot(points[0], direction)

	d := Dot(points[1], direction)
	if d > distance {
		distance = d
		index = 1
	}

	d = Dot(points[2], direction)
	if d > distance {
		return 2
	}

	return index
}

// queryTriangleFace is b3QueryTriangleFace.
func queryTriangleFace(triangle *triangleData, hull *HullData) separatingAxis {
	hullPoints := GetHullPoints(hull)
	plane := triangle.plane
	normal := Neg(plane.Normal)
	vertexIndex := findHullSupportVertex(hull, normal)
	support := hullPoints[vertexIndex]
	separation := planeSeparation(plane, support)

	return separatingAxis{
		normal:     plane.Normal,
		separation: separation,
		indexA:     0,
		indexB:     int32(vertexIndex),
		typ:        FaceAxisA,
	}
}

// queryHullFace is b3QueryHullFace.
func queryHullFace(triangle *triangleData, hull *HullData) separatingAxis {
	hullPlanes := GetHullPlanes(hull)
	faceCount := int(hull.FaceCount)

	trianglePoints := [3]Vec3{triangle.v1, triangle.v2, triangle.v3}

	maxNormal := Vec3Zero
	maxFaceSeparation := float32(math.Inf(-1))
	maxFaceIndex := NullIndex
	maxVertexIndex := NullIndex

	for faceIndex := range faceCount {
		plane := hullPlanes[faceIndex]

		vertexIndex := getTriangleSupport(trianglePoints[:], Neg(plane.Normal))
		support := trianglePoints[vertexIndex]
		separation := planeSeparation(plane, support)
		if separation > maxFaceSeparation {
			maxNormal = plane.Normal
			maxFaceSeparation = separation
			maxFaceIndex = faceIndex
			maxVertexIndex = vertexIndex
		}
	}

	// Normal points from triangle to hull
	return separatingAxis{
		normal:     Neg(maxNormal),
		separation: maxFaceSeparation,
		indexA:     int32(maxVertexIndex),
		indexB:     int32(maxFaceIndex),
		typ:        FaceAxisB,
	}
}

// queryTriangleAndHullEdges is b3QueryTriangleAndHullEdges.
// A: hull, B: triangle
func queryTriangleAndHullEdges(triangle *triangleData, hull *HullData) separatingAxis {
	result := separatingAxis{
		normal:     Vec3Zero,
		separation: float32(math.Inf(-1)),
		indexA:     NullIndex,
		indexB:     NullIndex,
		typ:        EdgePairAxis,
	}

	trianglePoints := [3]Vec3{triangle.v1, triangle.v2, triangle.v3}
	triangleEdges := [3]Vec3{triangle.e1, triangle.e2, triangle.e3}
	// int edgeFlags[] = { b3_concaveEdge1, b3_concaveEdge1, b3_concaveEdge3 };

	// PORT: B3_FORCE_GHOST_COLLISIONS is not defined upstream; only the default branch is ported.
	triangleFlags := triangle.flags
	_ = triangleFlags

	triNormal := triangle.plane.Normal

	hullEdges := GetHullEdges(hull)
	hullPoints := GetHullPoints(hull)
	hullPlanes := GetHullPlanes(hull)
	edgeCount := int(hull.EdgeCount)
	const squaredToleranceRoot float32 = 0.005
	squaredTolerance := squaredToleranceRoot * squaredToleranceRoot

	for i := 0; i < edgeCount; i += 2 {
		edge := &hullEdges[i]
		twin := &hullEdges[i+1]
		assert(int(edge.Twin) == i+1 && int(twin.Twin) == i, "edge->twin == i + 1 && twin->twin == i")

		hullPoint := hullPoints[edge.Origin]
		hullEdge := Sub(hullPoints[twin.Origin], hullPoint)

		hullNormal1 := hullPlanes[edge.Face].Normal
		hullNormal2 := hullPlanes[twin.Face].Normal

		for j := range 3 {
			triEdge := triangleEdges[j]

			cab := Dot(hullNormal1, triEdge)
			dab := Dot(hullNormal2, triEdge)
			bcd := Dot(triNormal, hullEdge)
			if cab*dab >= 0.0 || cab*bcd <= 0.0 {
				continue
			}

			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if MaxFloat(cab*cab, dab*dab) < squaredTolerance*LengthSquared(triEdge) {
				continue
			}

			// Similar to hull vs hull (b3QueryEdgeDirections)
			// dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
			// Normal points out of hull by construction.
			t := cab / (cab - dab)
			axis := Lerp(hullNormal1, hullNormal2, t)
			validate(LengthSquared(axis) > 1000.0*fltMin, "b3LengthSquared( axis ) > 1000.0f * FLT_MIN")
			axis = Normalize(axis)
			separation := Dot(axis, Sub(trianglePoints[j], hullPoint))

			// if ( separation > result.separation && ( edgeFlags[j] & triangleFlags ) == 0 )
			if separation > result.separation {
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching.
				// Flip normal to point from triangle to hull.
				result.normal = Neg(axis)
				result.separation = separation
				result.indexA = int32(j)
				result.indexB = int32(i)
			}
		}
	}

	return result
}

// collideHullFace is b3CollideHullFace.
func collideHullFace(manifold *LocalManifold, pointCapacity int, triangle *triangleData,
	hull *HullData, query separatingAxis, cache *SATCache, enableSpeculative bool) float32 {
	validate(query.typ == FaceAxisB, "query.type == b3_faceAxisB")
	validate(0 <= query.indexA && query.indexA < 3, "0 <= query.indexA && query.indexA < 3")
	validate(0 <= query.indexB && query.indexB < hull.FaceCount, "0 <= query.indexB && query.indexB < hull->faceCount")

	manifold.PointCount = 0

	hullFaces := GetHullFaces(hull)
	hullEdges := GetHullEdges(hull)
	hullPlanes := GetHullPlanes(hull)
	hullPoints := GetHullPoints(hull)

	// Reference hull face
	refPlane := hullPlanes[query.indexB]

	// Build clip polygon from triangle face (the incident face)
	var buffer1, buffer2 [maxClipPoints]clipVertex

	v1 := triangle.v1
	v2 := triangle.v2
	v3 := triangle.v3
	buffer1[0].position = v1
	buffer1[0].separation = planeSeparation(refPlane, v1)
	buffer1[0].pair = makeFeaturePair(featureShapeB, 2, featureShapeB, 0)
	buffer1[1].position = v2
	buffer1[1].separation = planeSeparation(refPlane, v2)
	buffer1[1].pair = makeFeaturePair(featureShapeB, 0, featureShapeB, 1)
	buffer1[2].position = v3
	buffer1[2].separation = planeSeparation(refPlane, v3)
	buffer1[2].pair = makeFeaturePair(featureShapeB, 1, featureShapeB, 2)
	pointCount := 3

	// Clip triangle face against side planes of reference face
	input := buffer1[:]
	output := buffer2[:]

	face := &hullFaces[query.indexB]
	edgeIndex := int(face.Edge)

	for {
		edge := &hullEdges[edgeIndex]
		nextEdgeIndex := int(edge.Next)
		next := &hullEdges[nextEdgeIndex]
		vertex1 := hullPoints[edge.Origin]
		vertex2 := hullPoints[next.Origin]
		tangent := Normalize(Sub(vertex2, vertex1))
		binormal := Cross(tangent, refPlane.Normal)

		clipPlane := makePlaneFromNormalAndPoint(binormal, vertex1)

		pointCount = clipPolygon(output, input, pointCount, clipPlane, edgeIndex, refPlane)
		assert(pointCount <= maxClipPoints, "pointCount <= B3_MAX_CLIP_POINTS")

		if pointCount < 3 {
			// Using a stale cache
			*cache = SATCache{}
			return query.separation
		}

		// Swap buffers, output becomes input for the next clipping plane
		output, input = input, output
		edgeIndex = nextEdgeIndex
		if edgeIndex == int(face.Edge) {
			break
		}
	}

	pointCount = MinInt(pointCount, pointCapacity)
	minSeparation := fltMax
	finalPointCount := 0

	for i := range pointCount {
		clipPoint := &input[i]
		minSeparation = MinFloat(minSeparation, clipPoint.separation)

		if enableSpeculative == false && clipPoint.separation > 0.0 {
			continue
		}

		// Move point onto hull face improved culling
		point := MulSub(clipPoint.position, clipPoint.separation, refPlane.Normal)

		pt := &manifold.Points[finalPointCount]
		pt.Point = point
		pt.Separation = clipPoint.separation
		pt.Pair = flipPair(clipPoint.pair)

		finalPointCount += 1
	}

	speculativeDistance := float32(0.0)
	if enableSpeculative {
		speculativeDistance = SpeculativeDistance()
	}
	if minSeparation > speculativeDistance {
		// This can occur with a stale SAT cache
		manifold.PointCount = 0
		*cache = SATCache{}
		return minSeparation
	}

	manifold.PointCount = int32(finalPointCount)
	manifold.Normal = Neg(refPlane.Normal)
	manifold.Feature = FeatureHullFace

	// Save cache
	cache.Separation = minSeparation
	cache.Type = uint8(FaceAxisB)
	cache.IndexA = uint16(query.indexA)
	cache.IndexB = uint16(query.indexB)
	return minSeparation
}

// collideTriangleFace is b3CollideTriangleFace.
func collideTriangleFace(manifold *LocalManifold, pointCapacity int, triangle *triangleData,
	hull *HullData, query separatingAxis, cache *SATCache, enableSpeculative bool) float32 {
	validate(query.typ == FaceAxisA, "query.type == b3_faceAxisA")
	validate(query.indexA == 0, "query.indexA == 0")
	validate(0 <= query.indexB && query.indexB < hull.VertexCount, "0 <= query.indexB && query.indexB < hull->vertexCount")
	validate(manifold.PointCount == 0, "manifold->pointCount == 0")

	hullFaces := GetHullFaces(hull)
	hullEdges := GetHullEdges(hull)
	hullPoints := GetHullPoints(hull)

	// Find incident face
	refPlane := triangle.plane

	incFace := findIncidentFace(hull, refPlane.Normal, int(query.indexB))

	// Build clip polygon from incident face
	var buffer1, buffer2 [2 * maxClipPoints]clipVertex
	pointCount := 0
	face := &hullFaces[incFace]
	hullEdgeIndex := int(face.Edge)

	for {
		edge := &hullEdges[hullEdgeIndex]

		nextEdgeIndex := int(edge.Next)
		next := &hullEdges[nextEdgeIndex]

		hullPoint := hullPoints[next.Origin]
		buffer1[pointCount].position = hullPoint
		buffer1[pointCount].separation = planeSeparation(refPlane, hullPoint)
		buffer1[pointCount].pair = makeFeaturePair(featureShapeB, hullEdgeIndex, featureShapeB, nextEdgeIndex)

		pointCount += 1
		hullEdgeIndex = nextEdgeIndex
		if !(hullEdgeIndex != int(face.Edge) && pointCount < 2*maxClipPoints) {
			break
		}
	}

	assert(pointCount >= 3, "pointCount >= 3")

	// Clip incident face against side planes of reference face (triangle)
	input := buffer1[:]
	output := buffer2[:]

	trianglePoints := [3]Vec3{triangle.v1, triangle.v2, triangle.v3}
	triangleEdges := [3]Vec3{triangle.e1, triangle.e2, triangle.e3}

	for i := 0; i < 3 && pointCount > 0; i++ {
		sideNormal := Cross(triangleEdges[i], refPlane.Normal)
		sideNormal = Normalize(sideNormal)

		clipPlane := makePlaneFromNormalAndPoint(sideNormal, trianglePoints[i])

		pointCount = clipPolygon(output, input, pointCount, clipPlane, i, refPlane)
		assert(pointCount <= 2*maxClipPoints, "pointCount <= 2 * B3_MAX_CLIP_POINTS")

		output, input = input, output
	}

	if pointCount == 0 {
		// Triangle face clipped away. Invalidate cache.
		*cache = SATCache{}
		return fltMax
	}

	pointCount = MinInt(pointCount, pointCapacity)

	minSeparation := fltMax

	finalPointCount := 0
	for i := range pointCount {
		clipPoint := &input[i]
		minSeparation = MinFloat(minSeparation, clipPoint.separation)

		if enableSpeculative == false && clipPoint.separation > 0.0 {
			continue
		}

		// Move point onto triangle surface for improved culling
		// b3Vec3 point = b3MulSub( clipPoint->position, clipPoint->separation, refPlane.normal );
		point := clipPoint.position

		pt := &manifold.Points[finalPointCount]
		pt.Point = point
		pt.Separation = clipPoint.separation
		pt.Pair = clipPoint.pair

		finalPointCount += 1
	}

	speculativeDistance := float32(0.0)
	if enableSpeculative {
		speculativeDistance = SpeculativeDistance()
	}
	if minSeparation >= speculativeDistance {
		// This can happens if the objects move a part while re-using a cached axis
		*cache = SATCache{}
		return minSeparation
	}

	manifold.PointCount = int32(finalPointCount)
	manifold.Normal = refPlane.Normal
	manifold.Feature = FeatureTriangleFace

	// Save cache
	cache.Separation = minSeparation
	cache.Type = uint8(FaceAxisA)
	cache.IndexA = uint16(query.indexA)
	cache.IndexB = uint16(query.indexB)
	return minSeparation
}

// collideTriangleAndHullEdges is b3CollideTriangleAndHullEdges.
func collideTriangleAndHullEdges(manifold *LocalManifold, capacity int, trianglePoint, triangleEdge Vec3,
	hull *HullData, query separatingAxis, cache *SATCache) {
	validate(query.typ == EdgePairAxis, "query.type == b3_edgePairAxis")
	validate(0 <= query.indexA && query.indexA < 3, "0 <= query.indexA && query.indexA < 3")
	validate(0 <= query.indexB && query.indexB < hull.EdgeCount, "0 <= query.indexB && query.indexB < hull->edgeCount")
	validate(query.separation <= float32(2.0*SpeculativeDistance()), "query.separation <= 2.0f * B3_SPECULATIVE_DISTANCE")

	pA := trianglePoint
	eA := triangleEdge

	edgesB := GetHullEdges(hull)
	pointsB := GetHullPoints(hull)
	edgeB := &edgesB[query.indexB]
	twinB := &edgesB[edgeB.Twin]
	pB := pointsB[edgeB.Origin]
	qB := pointsB[twinB.Origin]
	eB := Sub(qB, pB)

	// Get the closest points between the infinite edge lines
	result := LineDistance(pA, eA, pB, eB)

	// Is one of the closest points outside of the associated edge segment?
	if capacity == 0 || result.Fraction1 < 0.0 || 1.0 < result.Fraction1 || result.Fraction2 < 0.0 ||
		1.0 < result.Fraction2 {
		// Invalid edge pair, no points generated
		assert(manifold.PointCount == 0, "manifold->pointCount == 0")
		*cache = SATCache{}
		return
	}

	separation := Dot(query.normal, Sub(pB, pA))
	point := MulSV(0.5, Add(result.Point1, result.Point2))

	pt := &manifold.Points[0]
	pt.Point = point
	pt.Separation = separation
	pt.Pair = makeFeaturePair(featureShapeA, int(query.indexA), featureShapeB, int(query.indexB))

	// Save cache
	cache.Separation = separation
	cache.Type = uint8(EdgePairAxis)
	cache.IndexA = uint16(query.indexA)
	cache.IndexB = uint16(query.indexB)

	manifold.Normal = query.normal
	manifold.PointCount = 1

	edgesFeatures := [3]TriangleFeature{FeatureEdge1, FeatureEdge2, FeatureEdge3}
	manifold.Feature = edgesFeatures[query.indexA]
}

// triangleConvexCalls is b3_triangleConvexCalls.
var triangleConvexCalls atomicInt

// triangleCacheHits is b3_triangleCacheHits.
var triangleCacheHits atomicInt

// CollideTriangleAndHull collides a triangle and hull. Normal points from triangle to hull.
// Triangle is in the local space of the hull for efficiency.
func CollideTriangleAndHull(manifold *LocalManifold, capacity int, v1, v2, v3 Vec3, triangleFlags int,
	hullB *HullData, cache *SATCache, enableSpeculative bool) {
	manifold.PointCount = 0
	manifold.Feature = FeatureNone

	if capacity < 4 {
		return
	}

	trianglePlane := makePlaneFromPoints(v1, v2, v3)
	linearSlop := LinearSlop()

	offset := planeSeparation(trianglePlane, hullB.Center)
	if SeparatingFeature(cache.Type) == BacksideAxis {
		// Use hysteresis to avoid jitter on wavy meshes
		if AbsFloat(cache.Separation-offset) < linearSlop {
			return
		}

		cache.Type = uint8(InvalidAxis)
	}

	if offset < -linearSlop {
		// Cull back side collision. Cache offset to add hysteresis.
		cache.Type = uint8(BacksideAxis)
		cache.Separation = offset
		return
	}

	trianglePoints := [3]Vec3{v1, v2, v3}
	triangleEdges := [3]Vec3{Sub(v2, v1), Sub(v3, v2), Sub(v1, v3)}

	triangle := triangleData{
		v1:    v1,
		v2:    v2,
		v3:    v3,
		e1:    triangleEdges[0],
		e2:    triangleEdges[1],
		e3:    triangleEdges[2],
		plane: trianglePlane,
		flags: int32(triangleFlags),
	}

	edges := GetHullEdges(hullB)
	hullPlanes := GetHullPlanes(hullB)
	hullPoints := GetHullPoints(hullB)

	speculativeDistance := float32(0.0)
	if enableSpeculative {
		speculativeDistance = SpeculativeDistance()
	}
	cache.Hit = 1

	// Attempt to use the cache to speed up collision
	switch SeparatingFeature(cache.Type) {
	case FaceAxisA:
		{
			assert(cache.IndexA == 0, "cache->indexA == 0")

			vertexIndex := findHullSupportVertex(hullB, Neg(trianglePlane.Normal))
			support := hullPoints[vertexIndex]
			separation := planeSeparation(trianglePlane, support)
			if separation > speculativeDistance {
				// Cache hit, shapes are separated
				return
			}

			var faceQuery separatingAxis
			faceQuery.normal = trianglePlane.Normal
			faceQuery.separation = separation
			faceQuery.indexA = int32(cache.IndexA)
			faceQuery.indexB = int32(vertexIndex)
			faceQuery.typ = FaceAxisA

			// Read cache but don't modify it
			localCache := *cache
			clippedSeparation :=
				collideTriangleFace(manifold, capacity, &triangle, hullB, faceQuery, &localCache, enableSpeculative)

			if manifold.PointCount > 0 && AbsFloat(cache.Separation-clippedSeparation) < linearSlop {
				// Cache hit, contact points generated
				return
			}

			// Invalidate cache and fall through
			manifold.PointCount = 0
			*cache = SATCache{}
		}

	case FaceAxisB:
		{
			assert(int32(cache.IndexB) < hullB.FaceCount, "cache->indexB < hullB->faceCount")

			plane := hullPlanes[cache.IndexB]

			// Get triangle support point
			vertexIndex := 0
			distance := -Dot(v1, plane.Normal)
			for i := 1; i < 3; i++ {
				d := -Dot(trianglePoints[i], plane.Normal)
				if d > distance {
					distance = d
					vertexIndex = i
				}
			}

			support := trianglePoints[vertexIndex]

			// Separation of triangle support point with hull plane
			separation := planeSeparation(plane, support)
			if separation > speculativeDistance {
				// Cache hit, shapes are separated
				return
			}

			// Deep overlap may lead to an invalid cache
			// todo confirm
			isDeep := separation < float32(-2.0*linearSlop)

			// Don't persist deep cache or allow separation to change too much
			if isDeep == false {
				//  Try to rebuild contact from last features
				var faceQuery separatingAxis
				faceQuery.normal = Neg(plane.Normal)
				faceQuery.separation = separation
				faceQuery.indexA = int32(vertexIndex)
				faceQuery.indexB = int32(cache.IndexB)
				faceQuery.typ = FaceAxisB

				// Read cache but don't modify it
				localCache := *cache
				clippedSeparation :=
					collideHullFace(manifold, capacity, &triangle, hullB, faceQuery, &localCache, enableSpeculative)

				// Cache reuse is only successful if it creates contact points and the clipped separation didn't change much.
				if manifold.PointCount > 0 && AbsFloat(cache.Separation-clippedSeparation) < linearSlop {
					// Cache hit, contact points generated
					return
				}
			}

			// Invalidate cache and fall through
			manifold.PointCount = 0
			*cache = SATCache{}
		}

	case EdgePairAxis:
		{
			assert(cache.IndexA < 3, "cache->indexA < 3")
			indexA := int(cache.IndexA)

			triPoint := trianglePoints[indexA]
			triEdge := triangleEdges[indexA]

			assert(int32(cache.IndexB) < hullB.EdgeCount-1, "cache->indexB < hullB->edgeCount - 1")
			indexB := int(cache.IndexB)

			edge2 := &edges[indexB]
			twin2 := &edges[indexB+1]
			assert(int(edge2.Twin) == indexB+1 && int(twin2.Twin) == indexB, "edge2->twin == indexB + 1 && twin2->twin == indexB")

			hullPoint := hullPoints[edge2.Origin]
			hullEdge := Sub(hullPoints[twin2.Origin], hullPoint)
			hullNormal1 := hullPlanes[edge2.Face].Normal
			hullNormal2 := hullPlanes[twin2.Face].Normal

			// Confirm the edge pair is still a Minkowski face.
			// See "Collision Detection of Convex Polyhedra Based on Duality Transformation"
			// Simplified for triangle versus hull.
			cab := Dot(hullNormal1, triEdge)
			dab := Dot(hullNormal2, triEdge)
			bcd := Dot(trianglePlane.Normal, hullEdge)

			if cab*dab < 0.0 && cab*bcd > 0.0 {
				const squaredToleranceRoot float32 = 0.005
				squaredTolerance := squaredToleranceRoot * squaredToleranceRoot

				// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
				if MaxFloat(cab*cab, dab*dab) >= squaredTolerance*LengthSquared(triEdge) {
					// Similar to hull vs hull (b3QueryEdgeDirections)
					// dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
					// Normal points out of hull by construction.
					t := cab / (cab - dab)
					axis := Lerp(hullNormal1, hullNormal2, t)
					validate(LengthSquared(axis) > 1000.0*fltMin, "b3LengthSquared( axis ) > 1000.0f * FLT_MIN")
					axis = Normalize(axis)
					separation := Dot(axis, Sub(triPoint, hullPoint))
					if separation > speculativeDistance {
						// Cache hit, shapes are separated
						return
					}

					if AbsFloat(cache.Separation-separation) < linearSlop {
						// Try to rebuild contact from last features
						// Flip normal to point from triangle to hull
						var edgeQuery separatingAxis
						edgeQuery.normal = Neg(axis)
						edgeQuery.indexA = int32(indexA)
						edgeQuery.indexB = int32(indexB)
						edgeQuery.separation = separation
						edgeQuery.typ = EdgePairAxis

						// Read cache but don't modify it
						localCache := *cache
						collideTriangleAndHullEdges(manifold, capacity, triPoint, triEdge, hullB, edgeQuery, &localCache)

						if manifold.PointCount > 0 {
							// Cache hit, contact point generated
							return
						}
					}
				}
			}

			// Invalidate cache and fall through
			*cache = SATCache{}
		}

		// This case is for testing
	case ManualFaceAxisA:
		{
			query := queryTriangleFace(&triangle, hullB)
			collideTriangleFace(manifold, capacity, &triangle, hullB, query, cache, enableSpeculative)
			return
		}

		// This case is for testing
	case ManualFaceAxisB:
		{
			query := queryHullFace(&triangle, hullB)
			collideHullFace(manifold, capacity, &triangle, hullB, query, cache, enableSpeculative)
			return
		}

		// This case is for testing
	case ManualEdgePairAxis:
		{
			query := queryTriangleAndHullEdges(&triangle, hullB)
			if query.indexA != NullIndex {
				trianglePoint := trianglePoints[query.indexA]
				triangleEdge := triangleEdges[query.indexA]
				collideTriangleAndHullEdges(manifold, capacity, trianglePoint, triangleEdge, hullB, query, cache)
			}
			return
		}

	default:
		assert(SeparatingFeature(cache.Type) == InvalidAxis, "cache->type == b3_invalidAxis")
	}

	// Cache miss
	cache.Hit = 0

	// Find axis of minimum penetration
	faceQueryA := queryTriangleFace(&triangle, hullB)
	if faceQueryA.separation > speculativeDistance {
		// Separating axis found
		cache.Separation = faceQueryA.separation
		cache.Type = uint8(FaceAxisA)
		cache.IndexA = uint16(faceQueryA.indexA)
		cache.IndexB = uint16(faceQueryA.indexB)
		return
	}

	faceQueryB := queryHullFace(&triangle, hullB)
	if faceQueryB.separation > speculativeDistance {
		// Separating axis found
		cache.Separation = faceQueryB.separation
		cache.Type = uint8(FaceAxisB)
		cache.IndexA = uint16(faceQueryB.indexA)
		cache.IndexB = uint16(faceQueryB.indexB)
		return
	}

	edgeQuery := queryTriangleAndHullEdges(&triangle, hullB)
	if edgeQuery.separation > speculativeDistance {
		// Separating axis found
		cache.Separation = edgeQuery.separation
		cache.Type = uint8(EdgePairAxis)
		cache.IndexA = uint16(edgeQuery.indexA)
		cache.IndexB = uint16(edgeQuery.indexB)
		return
	}

	var clipSeparation float32

	// Don't admit a hull face significantly opposed to the triangle face.
	// Need a tolerance to avoid ghost collisions.
	pushingDown := Dot(faceQueryB.normal, trianglePlane.Normal) < -0.25
	if faceQueryB.separation >= faceQueryA.separation && pushingDown == false {
		clipSeparation = collideHullFace(manifold, capacity, &triangle, hullB, faceQueryB, cache, enableSpeculative)
	} else {
		clipSeparation = collideTriangleFace(manifold, capacity, &triangle, hullB, faceQueryA, cache, enableSpeculative)
	}

	// Does an edge axis exist?
	if edgeQuery.indexA != NullIndex {
		// When axes are aligned the edge separation can be garbage.
		// If a face axis has positive separation there may be no points.
		maxFaceSeparation := MaxFloat(faceQueryA.separation, faceQueryB.separation)

		if (manifold.PointCount == 0 && edgeQuery.separation > maxFaceSeparation) ||
			(manifold.PointCount == 1 && edgeQuery.separation > clipSeparation+linearSlop) {
			assert(0 <= edgeQuery.indexA && edgeQuery.indexA < 3, "0 <= edgeQuery.indexA && edgeQuery.indexA < 3")
			trianglePoint := trianglePoints[edgeQuery.indexA]
			triangleEdge := triangleEdges[edgeQuery.indexA]
			manifold.PointCount = 0
			collideTriangleAndHullEdges(manifold, capacity, trianglePoint, triangleEdge, hullB, edgeQuery, cache)
		}
	}

	// Using the speculative distance means that sometimes there are no valid contact points from SAT.
	// In this fall back to GJK. This is important to prevent tunneling in rare cases.
	if manifold.PointCount == 0 {
		triangleB := [3]Vec3{v1, v2, v3}
		input := DistanceInput{}
		input.ProxyA = ShapeProxy{
			Points: triangleB[:],
			Count:  3,
			Radius: 0.0,
		}
		input.ProxyB = ShapeProxy{Points: hullPoints, Count: hullB.VertexCount, Radius: 0.0}
		input.Transform = TransformIdentity
		input.UseRadii = false

		simplexCache := SimplexCache{}
		output := ShapeDistance(&input, &simplexCache, nil)

		if output.Distance > 0.0 {
			assert(0 < simplexCache.Count && simplexCache.Count <= 3, "0 < simplexCache.count && simplexCache.count <= 3")

			manifold.PointCount = 1
			manifold.Feature = getTriangleFeature(&simplexCache)
			manifold.Normal = output.Normal
			manifold.Points[0].Point = output.PointB
			manifold.Points[0].Separation = output.Distance

			// This feature pair not accurate but maybe it doesn't matter
			manifold.Points[0].Pair = featurePairSingle
		}

		// No way to cache this scenario
		*cache = SATCache{}
	}
}
