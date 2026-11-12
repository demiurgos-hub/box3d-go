// Port of src/convex_manifold.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// clipSegment is b3ClipSegment.
func clipSegment(segment *[2]clipVertex, plane Plane) int {
	vertexCount := 0
	vertex1 := segment[0]
	vertex2 := segment[1]

	distance1 := planeSeparation(plane, vertex1.position)
	distance2 := planeSeparation(plane, vertex2.position)

	// If the points are behind the plane
	if distance1 <= 0.0 {
		segment[vertexCount] = vertex1
		vertexCount++
	}
	if distance2 <= 0.0 {
		segment[vertexCount] = vertex2
		vertexCount++
	}

	// If the points are on different sides of the plane
	if distance1*distance2 < 0.0 {
		// Find intersection point of edge and plane
		t := distance1 / (distance1 - distance2)
		segment[vertexCount].position = Add(MulSV(1.0-t, vertex1.position), MulSV(t, vertex2.position))
		if distance1 > 0.0 {
			segment[vertexCount].pair = vertex1.pair
		} else {
			segment[vertexCount].pair = vertex2.pair
		}
		vertexCount++
	}

	return vertexCount
}

// clipSegmentToHullFace is b3ClipSegmentToHullFace.
func clipSegmentToHullFace(segment *[2]clipVertex, hull *HullData, refFace int) int {
	faces := GetHullFaces(hull)
	planes := GetHullPlanes(hull)
	edges := GetHullEdges(hull)
	points := GetHullPoints(hull)

	refPlane := planes[refFace]

	face := &faces[refFace]

	edgeIndex := int(face.Edge)

	for {
		edge := &edges[edgeIndex]
		nextEdgeIndex := int(edge.Next)
		next := &edges[nextEdgeIndex]

		vertex1 := points[edge.Origin]
		vertex2 := points[next.Origin]
		tangent := Normalize(Sub(vertex2, vertex1))
		binormal := Cross(tangent, refPlane.Normal)

		pointCount := clipSegment(segment, makePlaneFromNormalAndPoint(binormal, vertex1))
		if pointCount < 2 {
			return 0
		}

		edgeIndex = nextEdgeIndex
		if edgeIndex == int(face.Edge) {
			break
		}
	}

	return 2
}

// queryFaceDirectionHullAndCapsule is b3QueryFaceDirectionHullAndCapsule.
func queryFaceDirectionHullAndCapsule(hull *HullData, capsule *Capsule, capsuleTransform Transform) separatingAxis {
	maxFaceIndex := -1
	maxVertexIndex := -1
	maxFaceSeparation := -fltMax
	planes := GetHullPlanes(hull)

	capsulePoints := [2]Vec3{
		TransformPoint(capsuleTransform, capsule.Center1),
		TransformPoint(capsuleTransform, capsule.Center2),
	}

	for faceIndex := 0; faceIndex < int(hull.FaceCount); faceIndex++ {
		plane := planes[faceIndex]

		vertexIndex := getPointSupport(capsulePoints[:], 2, Neg(plane.Normal))
		support := capsulePoints[vertexIndex]
		separation := planeSeparation(plane, support)
		if separation > maxFaceSeparation {
			maxVertexIndex = vertexIndex
			maxFaceIndex = faceIndex
			maxFaceSeparation = separation
		}
	}

	return separatingAxis{
		normal:     planes[maxFaceIndex].Normal,
		separation: maxFaceSeparation,
		indexA:     int32(maxFaceIndex),
		indexB:     int32(maxVertexIndex),
	}
}

// queryEdgeDirectionHullAndCapsule is b3QueryEdgeDirectionHullAndCapsule.
func queryEdgeDirectionHullAndCapsule(hull *HullData, capsule *Capsule, capsuleTransform Transform) separatingAxis {
	// Find axis of minimum penetration
	maxNormal := Vec3Zero
	maxSeparation := -fltMax
	maxIndexA := int32(NullIndex)
	maxIndexB := int32(NullIndex)

	// We perform all computations in local space of the hull
	pA := TransformPoint(capsuleTransform, capsule.Center1)
	qA := TransformPoint(capsuleTransform, capsule.Center2)
	eA := Sub(qA, pA)

	edges := GetHullEdges(hull)
	points := GetHullPoints(hull)
	planes := GetHullPlanes(hull)
	squaredTolerance := ParallelEdgeTol * ParallelEdgeTol

	for index := 0; index < int(hull.EdgeCount); index += 2 {
		edge := &edges[index]
		twin := &edges[index+1]
		assert(int(edge.Twin) == index+1 && int(twin.Twin) == index, "edge->twin == index + 1 && twin->twin == index")

		qB := points[twin.Origin]
		uB := planes[edge.Face].Normal
		vB := planes[twin.Face].Normal

		// An isolated edge (e.g. like in a capsule) defines a circle through the
		// origin on the Gauss map. So testing for overlap between this circle and
		// the arc AB simplifies to a plane test.
		cba := Dot(uB, eA)
		dba := Dot(vB, eA)

		if cba*dba < 0.0 {
			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if MaxFloat(cba*cba, dba*dba) < squaredTolerance*LengthSquared(eA) {
				continue
			}

			// The intersection of the arcs on the Gauss map is the edge pair axis. Cast the
			// arc of hull B (from uB to vB) against the plane containing the arc of hull A:
			// dot(uB + t * (vB - uB), eA) == 0
			// then
			// t = cba / (cba - dba)
			//
			// The signs of cba and dba differ (Minkowski test), so the division is safe.
			//
			// The axis generated points from B to A by construction since it lands between
			// two face normals on B. This removes the need to orient the separation axis
			// using the hull centers.
			//
			// The axis is perpendicular to both edges so I can use qA and qB as arbitrary
			// points on edgeA and edgeB to measure the separation.

			t := cba / (cba - dba)
			axis := Lerp(uB, vB, t)
			validate(LengthSquared(axis) > 1000.0*fltMin, "b3LengthSquared( axis ) > 1000.0f * FLT_MIN")
			axis = Normalize(axis)
			separation := Dot(axis, Sub(qA, qB))

			if separation > maxSeparation {
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching and account for the convex radius later.
				maxNormal = axis
				maxSeparation = separation
				maxIndexA = 0
				maxIndexB = int32(index)
			}
		}
	}

	// Save result
	return separatingAxis{
		normal:     maxNormal,
		separation: maxSeparation,
		indexA:     maxIndexA,
		indexB:     maxIndexB,
	}
}

// CollideSpheres collides two spheres.
func CollideSpheres(manifold *LocalManifold, capacity int, sphereA, sphereB *Sphere, transformBtoA Transform) {
	if capacity == 0 {
		return
	}

	// Work in shapeB coordinates
	center1 := sphereA.Center
	center2 := TransformPoint(transformBtoA, sphereB.Center)

	totalRadius := sphereA.Radius + sphereB.Radius
	maxDistance := totalRadius + SpeculativeDistance()
	offset := Sub(center2, center1)
	distanceSq := LengthSquared(offset)

	if distanceSq > maxDistance*maxDistance {
		// We found a separating axis
		return
	}

	normal := Vec3{0.0, 1.0, 0.0}
	distance := sqrtf(distanceSq)
	if distance*distance > 1000.0*fltMin {
		normal = MulSV(1.0/distance, offset)
	}

	// Contact at the midpoint
	// 0.5 * ( ((c1 + rA*n) + c2) - rB*n )
	point :=
		MulSV(0.5, MulSub(Add(MulAdd(center1, sphereA.Radius, normal), center2), sphereB.Radius, normal))

	// Manifold in frame B
	manifold.Normal = normal
	manifold.PointCount = 1

	pt := &manifold.Points[0]
	pt.Point = point
	pt.Separation = distance - totalRadius
	pt.Pair = featurePairSingle
}

// CollideCapsuleAndSphere collides a capsule and a sphere.
func CollideCapsuleAndSphere(manifold *LocalManifold, capacity int, capsuleA *Capsule, sphereB *Sphere, transformBtoA Transform) {
	manifold.PointCount = 0

	if capacity < 1 {
		return
	}

	// Work in shape B coordinates
	center := TransformPoint(transformBtoA, sphereB.Center)
	center1 := capsuleA.Center1
	center2 := capsuleA.Center2

	totalRadius := sphereB.Radius + capsuleA.Radius
	maxDistance := totalRadius + SpeculativeDistance()

	closestPoint := PointToSegmentDistance(center1, center2, center)
	offset := Sub(center, closestPoint)
	distanceSq := LengthSquared(offset)

	if distanceSq > maxDistance*maxDistance {
		// We found a separating axis
		return
	}

	normal := Vec3{0.0, 1.0, 0.0}
	distance := sqrtf(distanceSq)
	if distance*distance > 1000.0*fltMin {
		normal = MulSV(1.0/distance, offset)
	}

	// Contact at the midpoint
	// 0.5 * (((center - sB*n) + closestPoint) + cA*n)
	point :=
		MulSV(0.5, MulAdd(Add(MulSub(center, sphereB.Radius, normal), closestPoint), capsuleA.Radius, normal))

	// Manifold in frame B
	manifold.Normal = normal
	manifold.PointCount = 1

	pt := &manifold.Points[0]
	pt.Point = point
	pt.Separation = distance - totalRadius
	pt.Pair = featurePairSingle
}

// CollideHullAndSphere collides a hull and a sphere.
func CollideHullAndSphere(manifold *LocalManifold, capacity int, hullA *HullData, sphereB *Sphere, transformBtoA Transform,
	cache *SimplexCache) {
	manifold.PointCount = 0

	if capacity == 0 {
		return
	}

	center := TransformPoint(transformBtoA, sphereB.Center)

	speculativeDistance := SpeculativeDistance()

	// Work in shapeA coordinates

	var distanceInput DistanceInput
	distanceInput.ProxyA = ShapeProxy{GetHullPoints(hullA), hullA.VertexCount, 0.0}
	// PORT: &center becomes a one element slice aliasing the local.
	distanceInput.ProxyB = ShapeProxy{unsafe.Slice(&center, 1), 1, 0.0}
	distanceInput.Transform = TransformIdentity
	distanceInput.UseRadii = false

	radiusA := float32(0.0)
	radiusB := sphereB.Radius
	radius := radiusA + radiusB

	distanceOutput := ShapeDistance(&distanceInput, cache, nil)

	if distanceOutput.Distance > radius+speculativeDistance {
		// We found a separating axis
		*cache = SimplexCache{}
		return
	}

	if distanceOutput.Distance > 100.0*fltEpsilon {
		// Shallow penetration
		normal := Normalize(Sub(distanceOutput.PointB, distanceOutput.PointA))

		// cA is the projection of the sphere center onto to the hull (pointA if radiusA == 0).
		cA := MulAdd(center, radiusA-Dot(Sub(center, distanceOutput.PointA), normal), normal)

		// cB is the deepest point on the sphere with respect to the reference f
		cB := MulSub(center, radiusB, normal)

		point := Lerp(cA, cB, 0.5)

		// Manifold in frame A
		manifold.Normal = normal
		manifold.PointCount = 1

		pt := &manifold.Points[0]
		pt.Point = point
		pt.Separation = distanceOutput.Distance - radius
		pt.Pair = featurePairSingle
	} else {
		// Deep penetration
		bestIndex := -1
		bestDistance := -fltMax
		planes := GetHullPlanes(hullA)

		for index := 0; index < int(hullA.FaceCount); index++ {
			plane := planes[index]

			distance := planeSeparation(plane, center)
			if distance > bestDistance {
				bestIndex = index
				bestDistance = distance
			}
		}
		assert(bestIndex >= 0, "bestIndex >= 0")

		normal := planes[bestIndex].Normal

		// cA is the projection of the sphere center onto to the hull
		cA := MulAdd(center, radiusA-Dot(Sub(center, distanceOutput.PointA), normal), normal)

		// cB is the deepest point on the sphere with respect to the reference f
		cB := MulSub(center, radiusB, normal)

		point := Lerp(cA, cB, 0.5)

		// Manifold in frame A
		manifold.Normal = normal
		manifold.PointCount = 1

		pt := &manifold.Points[0]
		pt.Point = point
		pt.Separation = bestDistance - radius
		pt.Pair = featurePairSingle
	}
}

// CollideCapsules collides two capsules.
func CollideCapsules(manifold *LocalManifold, capacity int, capsuleA, capsuleB *Capsule, transformBtoA Transform) {
	manifold.PointCount = 0

	if capacity < 2 {
		return
	}

	// Work in shapeA coordinates
	centerA1 := capsuleA.Center1
	centerA2 := capsuleA.Center2
	centerB1 := TransformPoint(transformBtoA, capsuleB.Center1)
	centerB2 := TransformPoint(transformBtoA, capsuleB.Center2)

	radius := capsuleA.Radius + capsuleB.Radius
	maxDistance := radius + SpeculativeDistance()

	result := SegmentDistance(centerA1, centerA2, centerB1, centerB2)
	offset := Sub(result.Point2, result.Point1)
	distanceSquared := LengthSquared(offset)
	linearSlop := LinearSlop()
	minDistance := float32(0.01 * linearSlop)

	if distanceSquared > maxDistance*maxDistance || distanceSquared < minDistance*minDistance {
		// We found a separating axis
		return
	}

	segmentA := Sub(centerA2, centerA1)
	edgeA, lengthA := GetLengthAndNormalize(segmentA)
	if lengthA < MinCapsuleLength() {
		return
	}

	segmentB := Sub(centerB2, centerB1)
	edgeB, lengthB := GetLengthAndNormalize(segmentB)
	if lengthB < MinCapsuleLength() {
		return
	}

	// Parallel edges: |eA x eB| = sin(alpha)
	const alphaTol float32 = 0.05
	const alphaTolSqr = alphaTol * alphaTol
	axis := Cross(edgeA, edgeB)

	// Try to create two contact points if the capsules are nearly parallel
	if LengthSquared(axis) < alphaTolSqr {
		// Clip segment B against side planes of segment A

		// Sides planes of A
		var planesA [2]Plane
		planesA[0].Normal = Neg(edgeA)
		planesA[0].Offset = -Dot(edgeA, capsuleA.Center1)
		planesA[1].Normal = edgeA
		planesA[1].Offset = Dot(edgeA, capsuleA.Center2)

		// Clip points for B
		var verticesB [2]clipVertex
		verticesB[0].position = centerB1
		verticesB[0].separation = 0.0
		verticesB[0].pair = makeFeaturePair(featureShapeA, 0, featureShapeA, 0)
		verticesB[1].position = centerB2
		verticesB[1].separation = 0.0
		verticesB[1].pair = makeFeaturePair(featureShapeA, 1, featureShapeA, 1)

		pointCount := clipSegment(&verticesB, planesA[0])
		if pointCount == 2 {
			pointCount = clipSegment(&verticesB, planesA[1])
		}

		if pointCount == 2 {
			// Closest points on A to the clipped points on B.
			closestPoint1 := PointToSegmentDistance(centerA1, centerA2, verticesB[0].position)
			closestPoint2 := PointToSegmentDistance(centerA1, centerA2, verticesB[1].position)

			distance1 := Distance(closestPoint1, verticesB[0].position)
			distance2 := Distance(closestPoint2, verticesB[1].position)
			if distance1 <= radius && distance2 <= radius {
				if distance1 < minDistance || distance2 < minDistance {
					// Avoid divide by zero
					return
				}

				normal1 := MulSV(1.0/distance1, Sub(verticesB[0].position, closestPoint1))
				normal2 := MulSV(1.0/distance2, Sub(verticesB[1].position, closestPoint2))
				normal := Normalize(Add(normal1, normal2))
				radiusA := capsuleA.Radius
				radiusB := capsuleB.Radius

				// Contact is at the midpoint: 0.5 * (((vB.pos + rA*nK) + cP) - rB*n)
				point1 :=
					MulSV(0.5, MulSub(Add(MulAdd(verticesB[0].position, radiusA, normal1), closestPoint1), radiusB,
						normal))
				point2 :=
					MulSV(0.5, MulSub(Add(MulAdd(verticesB[1].position, radiusA, normal2), closestPoint2), radiusB,
						normal))

				// Manifold in frame A
				manifold.Normal = normal
				manifold.PointCount = 2

				pt1 := &manifold.Points[0]
				pt1.Point = point1
				pt1.Separation = distance1 - radius
				pt1.Pair = verticesB[0].pair

				pt2 := &manifold.Points[1]
				pt2.Point = point2
				pt2.Separation = distance2 - radius
				pt2.Pair = verticesB[1].pair

				return
			}
		}
	}

	normal, distance := GetLengthAndNormalize(offset)
	// Contact at the midpoint 0.5 * (((p1 + rA*n) + p2) - rB*n)
	point := MulSV(
		0.5, MulSub(Add(MulAdd(result.Point1, capsuleA.Radius, normal), result.Point2), capsuleB.Radius, normal))

	// Manifold in frame A
	manifold.Normal = normal
	manifold.PointCount = 1

	pt := &manifold.Points[0]
	pt.Point = point
	pt.Separation = distance - radius
	pt.Pair = featurePairSingle
}

// buildHullFaceAndCapsuleContact is b3BuildHullFaceAndCapsuleContact.
func buildHullFaceAndCapsuleContact(manifold *LocalManifold, hullA *HullData, capsuleB *Capsule,
	transformBtoA Transform, query separatingAxis) bool {
	// Work in shapeA coordinates
	planes := GetHullPlanes(hullA)

	// Clip the capsule edge against the side planes of the reference face
	refFace := int(query.indexA)
	refPlane := planes[refFace]

	var segmentB [2]clipVertex
	segmentB[0].position = TransformPoint(transformBtoA, capsuleB.Center1)
	segmentB[0].separation = 0.0
	segmentB[0].pair = makeFeaturePair(featureShapeA, 0, featureShapeA, 0)
	segmentB[1].position = TransformPoint(transformBtoA, capsuleB.Center2)
	segmentB[1].separation = 0.0
	segmentB[1].pair = makeFeaturePair(featureShapeA, 1, featureShapeA, 1)

	pointCount := clipSegmentToHullFace(&segmentB, hullA, refFace)
	if pointCount < 2 {
		return false
	}

	distance1 := planeSeparation(refPlane, segmentB[0].position)
	distance2 := planeSeparation(refPlane, segmentB[1].position)
	speculativeDistance := SpeculativeDistance()

	if distance1 <= speculativeDistance || distance2 <= speculativeDistance {
		normal := refPlane.Normal
		point1 := MulSub(segmentB[0].position, float32(0.5*(distance1+capsuleB.Radius)), normal)
		point2 := MulSub(segmentB[1].position, float32(0.5*(distance2+capsuleB.Radius)), normal)

		// Manifold in frame A
		manifold.Normal = normal
		manifold.PointCount = 2

		pt1 := &manifold.Points[0]
		pt1.Point = point1
		pt1.Separation = distance1 - capsuleB.Radius
		pt1.Pair = segmentB[0].pair

		pt2 := &manifold.Points[1]
		pt2.Point = point2
		pt2.Separation = distance2 - capsuleB.Radius
		pt2.Pair = segmentB[1].pair

		return true
	}

	return false
}

// buildHullAndCapsuleEdgeContact is b3BuildHullAndCapsuleEdgeContact.
func buildHullAndCapsuleEdgeContact(manifold *LocalManifold, capacity int, hullA *HullData,
	capsuleB *Capsule, transformBtoA Transform, query separatingAxis) bool {
	if capacity < 1 {
		return false
	}

	// Work in shapeA coordinates

	pc := TransformPoint(transformBtoA, capsuleB.Center1)
	qc := TransformPoint(transformBtoA, capsuleB.Center2)
	ec := Sub(qc, pc)

	edges := GetHullEdges(hullA)
	points := GetHullPoints(hullA)

	edge2 := &edges[query.indexB]
	twin2 := &edges[edge2.Twin]
	ph := points[edge2.Origin]
	qh := points[twin2.Origin]
	eh := Sub(qh, ph)

	normal := query.normal

	result := LineDistance(ph, eh, pc, ec)

	if isWithinSegments(&result) == false {
		// closest point beyond end points
		return false
	}

	point := MulSV(0.5, Add(MulSub(result.Point1, capsuleB.Radius, normal), result.Point2))
	separation := Dot(normal, Sub(result.Point2, result.Point1))
	validate(AbsFloat(separation-query.separation) < LinearSlop(), "b3AbsFloat( separation - query.separation ) < B3_LINEAR_SLOP")

	// Manifold in frame A
	manifold.Normal = normal
	manifold.PointCount = 1

	pt := &manifold.Points[0]
	pt.Point = point
	pt.Separation = separation - capsuleB.Radius
	pt.Pair = makeFeaturePair(featureShapeA, int(query.indexA), featureShapeB, int(query.indexB))
	return true
}

// CollideHullAndCapsule collides a hull and a capsule.
func CollideHullAndCapsule(manifold *LocalManifold, capacity int, hullA *HullData, capsuleB *Capsule,
	transformBtoA Transform, cache *SimplexCache) {
	manifold.PointCount = 0

	if capacity < 2 {
		return
	}

	// Work in shapeA coordinates
	var distanceInput DistanceInput
	distanceInput.ProxyA = ShapeProxy{GetHullPoints(hullA), hullA.VertexCount, 0.0}
	distanceInput.ProxyB = ShapeProxy{capsulePoints(capsuleB), 2, 0.0}
	distanceInput.Transform = transformBtoA
	distanceInput.UseRadii = false

	distanceOutput := ShapeDistance(&distanceInput, cache, nil)
	speculativeDistance := SpeculativeDistance()

	if distanceOutput.Distance > capsuleB.Radius+speculativeDistance {
		// We found a separating axis
		*cache = SimplexCache{}
		return
	}

	if distanceOutput.Distance > 100.0*fltEpsilon {
		planes := GetHullPlanes(hullA)

		// Shallow penetration
		delta := distanceOutput.Normal
		refFace := findHullSupportFace(hullA, delta)
		refPlane := planes[refFace]

		// Try to create two contact points if closest
		// points difference is nearly parallel to face normal
		const kTolerance float32 = 0.998
		if AbsFloat(Dot(refPlane.Normal, delta)) > kTolerance {
			// Clip capsule segment against side planes of reference face
			var verticesB [2]clipVertex
			verticesB[0].position = TransformPoint(transformBtoA, capsuleB.Center1)
			verticesB[0].separation = 0.0
			verticesB[0].pair = makeFeaturePair(featureShapeA, 0, featureShapeA, 0)
			verticesB[1].position = TransformPoint(transformBtoA, capsuleB.Center2)
			verticesB[1].separation = 0.0
			verticesB[1].pair = makeFeaturePair(featureShapeA, 1, featureShapeA, 1)

			pointCount := clipSegmentToHullFace(&verticesB, hullA, refFace)

			if pointCount == 2 {
				distance1 := planeSeparation(refPlane, verticesB[0].position)
				distance2 := planeSeparation(refPlane, verticesB[1].position)
				if distance1 <= capsuleB.Radius+speculativeDistance || distance2 <= capsuleB.Radius+speculativeDistance {
					normal := refPlane.Normal
					point1 := MulSub(verticesB[0].position, float32(0.5*(capsuleB.Radius+distance1)), normal)
					point2 := MulSub(verticesB[1].position, float32(0.5*(capsuleB.Radius+distance2)), normal)

					// Manifold in frame A
					manifold.Normal = normal
					manifold.PointCount = 2

					pt1 := &manifold.Points[0]
					pt1.Point = point1
					pt1.Separation = distance1 - capsuleB.Radius
					pt1.Pair = verticesB[0].pair

					pt2 := &manifold.Points[1]
					pt2.Point = point2
					pt2.Separation = distance2 - capsuleB.Radius
					pt2.Pair = verticesB[1].pair

					return
				}
			}
		}

		// Create contact from closest points
		point :=
			MulSV(0.5, Add(MulSub(distanceOutput.PointA, capsuleB.Radius, delta), distanceOutput.PointB))

		// Manifold in frame A
		manifold.Normal = delta
		manifold.PointCount = 1

		pt := &manifold.Points[0]
		pt.Point = point
		pt.Separation = distanceOutput.Distance - capsuleB.Radius
		pt.Pair = featurePairSingle
		return
	}

	// Deep penetration

	faceQuery := queryFaceDirectionHullAndCapsule(hullA, capsuleB, transformBtoA)
	if faceQuery.separation > capsuleB.Radius {
		// We found a separating axis
		return
	}

	edgeQuery := queryEdgeDirectionHullAndCapsule(hullA, capsuleB, transformBtoA)
	if edgeQuery.separation > capsuleB.Radius {
		// We found a separating axis
		return
	}

	// Create face contact
	faceSeparation := faceQuery.separation - capsuleB.Radius
	buildHullFaceAndCapsuleContact(manifold, hullA, capsuleB, transformBtoA, faceQuery)
	validate(manifold.PointCount == 0 || manifold.PointCount == 2, "manifold->pointCount == 0 || manifold->pointCount == 2")
	if manifold.PointCount == 2 {
		// This becomes the clipped separation.
		faceSeparation = MinFloat(manifold.Points[0].Separation, manifold.Points[1].Separation)
	}

	// Is there a valid edge-edge axis?
	if edgeQuery.indexA == NullIndex {
		return
	}

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	linearSlop := LinearSlop()
	edgeSeparation := edgeQuery.separation - capsuleB.Radius
	if manifold.PointCount == 0 || edgeSeparation > faceSeparation+linearSlop {
		// Edge contact
		buildHullAndCapsuleEdgeContact(manifold, capacity, hullA, capsuleB, transformBtoA, edgeQuery)
	}
}

// _Static_assert( B3_MAX_MANIFOLD_POINTS >= 4, "must be 4 or more" );
// PORT: _Static_assert becomes a constant expression that overflows unless it holds.
const _ uint = MaxManifoldPoints - 4

// PORT: only the B3_MAX_MANIFOLD_POINTS == 4 variant of b3ReduceManifoldPoints is ported.
// MaxManifoldPoints is a fixed constant (4) in Go, so the #else variant (2D hull simplification)
// is compiled out upstream as well.

// reduceManifoldPoints is b3ReduceManifoldPoints.
// Reduce the manifold points to a maximum of 4 points.
// Note: this modifies the input point array to improve performance
func reduceManifoldPoints(manifold *LocalManifold, capacity int, points []LocalManifoldPoint, count int) {
	if capacity < 4 {
		return
	}

	if count <= 4 {
		for i := range count {
			manifold.Points[i] = points[i]
		}

		manifold.PointCount = int32(count)
		return
	}

	normal := manifold.Normal
	// float linearSlop = B3_LINEAR_SLOP;
	speculativeDistance := SpeculativeDistance()
	tolSqr := speculativeDistance * speculativeDistance

	// This bias is very important for contact point consistency across time steps.
	// It creates a pecking order to avoid flickering between candidates with similar scores.
	const bias float32 = 0.95

	// Step 1: find extreme point that is touching
	bestIndex := NullIndex
	bestScore := -fltMax

	// Arbitrary tangent direction
	// b3Vec3 perp1 = b3Perp( normal );
	// b3Vec3 perp2 = b3Cross( perp1, normal );
	// b3Vec3 searchDirection = -0.4535961214255773f * perp1 + 0.8912073600614354f * perp2;
	searchDirection := arbitraryPerp(normal)
	for index := range count {
		pt := &points[index]

		if pt.Separation > speculativeDistance {
			continue
		}

		// The deeper the better
		score := -pt.Separation + Dot(searchDirection, pt.Point)
		if bias*score > bestScore {
			bestIndex = index
			bestScore = score
		}
	}

	validate(0 <= bestIndex && bestIndex < count, "0 <= bestIndex && bestIndex < count")
	if bestIndex == NullIndex {
		manifold.PointCount = 0
		return
	}

	manifold.Points[0] = points[bestIndex]
	manifold.PointCount = 1

	// Remove best point from array
	points[bestIndex] = points[count-1]
	count -= 1

	a := manifold.Points[0].Point

	// Step 2: Find farthest point in 2D
	bestScore = 0.0
	bestIndex = NullIndex

	for index := range count {
		p := points[index].Point
		d := Sub(p, a)
		v := MulSub(d, Dot(d, normal), normal)
		distanceSquared := LengthSquared(v)
		separation := MaxFloat(0.0, -points[index].Separation)
		score := distanceSquared + float32(4.0*separation*separation)
		if bias*score > bestScore {
			bestScore = score
			bestIndex = index
		}
	}

	if bestScore < tolSqr {
		return
	}

	assert(0 <= bestIndex && bestIndex < count, "0 <= bestIndex && bestIndex < count")
	manifold.Points[1] = points[bestIndex]
	manifold.PointCount = 2

	// Remove best point from array
	points[bestIndex] = points[count-1]
	count -= 1

	b := manifold.Points[1].Point

	// Step 3: Find the point with the maximum triangular area
	bestScore = tolSqr
	bestIndex = NullIndex
	bestSignedArea := float32(0.0)
	ba := Sub(b, a)
	for index := range count {
		p := points[index].Point
		signedArea := Dot(normal, Cross(ba, Sub(p, a)))
		score := AbsFloat(signedArea)
		if bias*score >= bestScore {
			bestScore = score
			bestIndex = index
			bestSignedArea = signedArea
		}
	}

	if bestIndex == NullIndex {
		return
	}

	assert(bestIndex != NullIndex, "bestIndex != B3_NULL_INDEX")

	manifold.Points[2] = points[bestIndex]
	manifold.PointCount = 3
	points[bestIndex] = points[count-1]
	count -= 1

	c := manifold.Points[2].Point

	// Step 4: get the point that adds the most area outside the current triangle
	bestScore = tolSqr
	bestIndex = NullIndex
	sign := float32(1.0)
	if bestSignedArea < 0.0 {
		sign = -1.0
	}
	for index := range count {
		p := points[index].Point
		u1 := sign * Dot(normal, Cross(Sub(p, a), ba))
		u2 := sign * Dot(normal, Cross(Sub(p, b), Sub(c, b)))
		u3 := sign * Dot(normal, Cross(Sub(p, c), Sub(a, c)))
		score := MaxFloat(u1, MaxFloat(u2, u3))

		if bias*score > bestScore {
			bestScore = score
			bestIndex = index
		}
	}

	if bestIndex != NullIndex {
		manifold.Points[manifold.PointCount] = points[bestIndex]
		manifold.PointCount += 1
	}
}

// buildPolygon is b3BuildPolygon.
func buildPolygon(out []clipVertex, transform Transform, hull *HullData, incFace int, refPlane Plane) int {
	faces := GetHullFaces(hull)
	edges := GetHullEdges(hull)
	points := GetHullPoints(hull)

	face := &faces[incFace]
	edgeIndex := int(face.Edge)
	assert(int(edges[edgeIndex].Face) == incFace, "edges[edgeIndex].face == incFace")

	outCount := 0

	matrix := MakeMatrixFromQuat(transform.Q)

	for {
		edge := &edges[edgeIndex]

		nextEdgeIndex := int(edge.Next)
		next := &edges[nextEdgeIndex]

		var vertex clipVertex
		vertex.position = Add(MulMV(matrix, points[next.Origin]), transform.P)
		vertex.separation = planeSeparation(refPlane, vertex.position)
		vertex.pair = makeFeaturePair(featureShapeB, edgeIndex, featureShapeB, nextEdgeIndex)

		out[outCount] = vertex
		outCount += 1

		edgeIndex = nextEdgeIndex
		if !(edgeIndex != int(face.Edge) && outCount < maxClipPoints) {
			break
		}
	}

	validate(!validateEnabled || validatePolygon(out, outCount), "b3ValidatePolygon( out, outCount )")

	return outCount
}

// buildFaceAContact is b3BuildFaceAContact.
func buildFaceAContact(manifold *LocalManifold, capacity int, hullA, hullB *HullData,
	transformBtoA Transform, query separatingAxis, cache *SATCache) bool {
	assert(capacity > 0, "capacity > 0")
	validate(query.typ == FaceAxisA, "query.type == b3_faceAxisA")
	validate(0 <= query.indexA && query.indexA < hullA.FaceCount, "0 <= query.indexA && query.indexA < hullA->faceCount")
	validate(0 <= query.indexB && query.indexB < hullB.VertexCount, "0 <= query.indexB && query.indexB < hullB->vertexCount")

	facesA := GetHullFaces(hullA)
	edgesA := GetHullEdges(hullA)
	planesA := GetHullPlanes(hullA)
	pointsA := GetHullPoints(hullA)

	// Reference face
	refFace := int(query.indexA)
	refPlane := planesA[refFace]

	// Find incident face
	refNormalInB := InvRotateVector(transformBtoA.Q, refPlane.Normal)
	incFace := findIncidentFace(hullB, refNormalInB, int(query.indexB))

	// Build clip polygon from incident face in frame A
	var buffer1, buffer2 [maxClipPoints]clipVertex
	pointCount := buildPolygon(buffer1[:], transformBtoA, hullB, incFace, refPlane)

	// Clip incident face against side planes of reference face
	input := buffer1[:]
	output := buffer2[:]

	face := &facesA[refFace]
	edgeIndex := int(face.Edge)

	for {
		edge := &edgesA[edgeIndex]
		nextEdgeIndex := int(edge.Next)
		next := &edgesA[nextEdgeIndex]
		vertex1 := pointsA[edge.Origin]
		vertex2 := pointsA[next.Origin]
		tangent := Normalize(Sub(vertex2, vertex1))
		binormal := Cross(tangent, refPlane.Normal)

		clipPlane := makePlaneFromNormalAndPoint(binormal, vertex1)

		pointCount = clipPolygon(output, input, pointCount, clipPlane, edgeIndex, refPlane)
		assert(pointCount <= maxClipPoints, "pointCount <= B3_MAX_CLIP_POINTS")

		output, input = input, output

		if pointCount < 3 {
			*cache = SATCache{}
			return false
		}

		edgeIndex = nextEdgeIndex
		if edgeIndex == int(face.Edge) {
			break
		}
	}

	pointCount = MinInt(pointCount, maxClipPoints)

	var points [maxClipPoints]LocalManifoldPoint
	minSeparation := fltMax

	manifold.Normal = refPlane.Normal

	for i := range pointCount {
		clipPoint := &input[i]
		pt := &points[i]
		*pt = LocalManifoldPoint{}

		// Using the half-way point keeps the points in the same position when swapping reference face from A to B.
		point := MulSub(clipPoint.position, float32(0.5*clipPoint.separation), refPlane.Normal)

		// Old way of pushing onto the reference face.
		// b3Vec3 point = clipPoint->position - clipPoint->separation * refPlane.normal;

		pt.Point = point
		pt.Separation = clipPoint.separation
		pt.Pair = clipPoint.pair

		minSeparation = MinFloat(minSeparation, clipPoint.separation)
	}

	if minSeparation >= SpeculativeDistance() {
		*cache = SATCache{}
		return false
	}

	reduceManifoldPoints(manifold, capacity, points[:], pointCount)

	// Save cache
	cache.Separation = minSeparation
	cache.Type = uint8(FaceAxisA)
	cache.IndexA = uint16(query.indexA)
	cache.IndexB = uint16(query.indexB)

	return true
}

// buildFaceBContact is b3BuildFaceBContact.
func buildFaceBContact(manifold *LocalManifold, capacity int, hullA, hullB *HullData,
	transformBtoA Transform, query separatingAxis, cache *SATCache) bool {
	validate(query.typ == FaceAxisB, "query.type == b3_faceAxisB")

	transformAtoB := InvertTransform(transformBtoA)
	flippedQuery := separatingAxis{
		normal:     Neg(query.normal),
		separation: query.separation,
		indexA:     query.indexB,
		indexB:     query.indexA,
		typ:        FaceAxisA,
	}

	touching := buildFaceAContact(manifold, capacity, hullB, hullA, transformAtoB, flippedQuery, cache)
	if touching == false {
		*cache = SATCache{}
		return false
	}

	// Results are in frame B, need to transform them into frame A
	matrix := MakeMatrixFromQuat(transformBtoA.Q)

	// Transform and flip normal so it points from A to B, even though the B has the reference face.
	manifold.Normal = Neg(MulMV(matrix, manifold.Normal))

	// Transform points from frame B to frame A.
	// Also flip the pairs to ensure correct matches.
	for i := 0; i < int(manifold.PointCount); i++ {
		pt := &manifold.Points[i]
		pt.Point = Add(MulMV(matrix, pt.Point), transformBtoA.P)
		pt.Pair = flipPair(pt.Pair)
	}

	cache.Type = uint8(FaceAxisB)
	cache.IndexA = uint16(query.indexA)
	cache.IndexB = uint16(query.indexB)

	return true
}

// buildEdgeContact is b3BuildEdgeContact.
func buildEdgeContact(manifold *LocalManifold, hullA, hullB *HullData,
	transformBtoA Transform, query separatingAxis, cache *SATCache) bool {
	validate(query.typ == EdgePairAxis, "query.type == b3_edgePairAxis")
	validate(0 <= query.indexA && query.indexA < hullA.EdgeCount, "0 <= query.indexA && query.indexA < hullA->edgeCount")
	validate(0 <= query.indexB && query.indexB < hullB.EdgeCount, "0 <= query.indexB && query.indexB < hullB->edgeCount")

	// Work in shapeA coordinates
	edgesA := GetHullEdges(hullA)
	pointsA := GetHullPoints(hullA)

	edgesB := GetHullEdges(hullB)
	pointsB := GetHullPoints(hullB)

	// B3_VALIDATE( query.separation <= 2.0f * B3_SPECULATIVE_DISTANCE );

	edgeA := &edgesA[query.indexA]
	twinA := &edgesA[edgeA.Twin]
	pA := pointsA[edgeA.Origin]
	qA := pointsA[twinA.Origin]
	eA := Sub(qA, pA)

	edgeB := &edgesB[query.indexB]
	twinB := &edgesB[edgeB.Twin]
	pB := TransformPoint(transformBtoA, pointsB[edgeB.Origin])
	qB := TransformPoint(transformBtoA, pointsB[twinB.Origin])
	eB := Sub(qB, pB)

	normal := query.normal
	result := LineDistance(pA, eA, pB, eB)

	if isWithinSegments(&result) == false {
		*cache = SATCache{}
		return false
	}

	// This can slide off the end from caching
	separation := Dot(normal, Sub(result.Point2, result.Point1))
	point := MulSV(0.5, Add(result.Point1, result.Point2))

	// Result in frame A
	manifold.Normal = normal
	manifold.PointCount = 1

	pt := &manifold.Points[0]
	pt.Point = point
	pt.Separation = separation
	pt.Pair = makeFeaturePair(featureShapeA, int(query.indexA), featureShapeB, int(query.indexB))

	// Save cache
	cache.Separation = separation
	cache.Type = uint8(EdgePairAxis)
	cache.IndexA = uint16(query.indexA)
	cache.IndexB = uint16(query.indexB)

	return true
}

// computeSeparatingAxisAtWidth is b3ComputeSeparatingAxisAtWidth.
// PORT: the width 8 kernel (b3ComputeSeparatingAxisW8) is called through the wide8 table
// (wide_kernels.go); simdWidth is 8 only when it is set.
func computeSeparatingAxisAtWidth(hullA, hullB *HullData, xfB Transform, earlyReturn bool,
	simdWidth int) axisQuery {
	if simdWidth == 8 {
		return wide8.computeSeparatingAxis(hullA, hullB, xfB, earlyReturn)
	}

	return computeSeparatingAxisW(hullA, hullB, xfB, earlyReturn)
}

// computeSeparatingAxis is b3ComputeSeparatingAxis.
func computeSeparatingAxis(hullA, hullB *HullData, xfB Transform, earlyReturn bool) axisQuery {
	return computeSeparatingAxisAtWidth(hullA, hullB, xfB, earlyReturn, GetSIMDWidth())
}

// collideHullsAtWidth is b3CollideHullsAtWidth.
func collideHullsAtWidth(manifold *LocalManifold, capacity int, hullA, hullB *HullData,
	transformBtoA Transform, cache *SATCache, simdWidth int) {
	manifold.PointCount = 0

	if capacity < 4 {
		return
	}

	// Work in shapeA coordinates
	speculativeDistance := SpeculativeDistance()

	linearSlop := LinearSlop()
	edgesA := GetHullEdges(hullA)
	planesA := GetHullPlanes(hullA)
	pointsA := GetHullPoints(hullA)

	edgesB := GetHullEdges(hullB)
	planesB := GetHullPlanes(hullB)
	pointsB := GetHullPoints(hullB)

	cache.Hit = 0

	// Attempt to use the cache to speed up collision
	switch SeparatingFeature(cache.Type) {
	case InvalidAxis:
		break

	case FaceAxisA:
		{
			assert(int32(cache.IndexA) < hullA.FaceCount, "cache->indexA < hullA->faceCount")

			// Check for separation using cached face
			plane := planesA[cache.IndexA]
			searchDirectionInB := Neg(InvRotateVector(transformBtoA.Q, plane.Normal))

			vertexIndex := findHullSupportVertex(hullB, searchDirectionInB)
			support := TransformPoint(transformBtoA, pointsB[vertexIndex])
			separation := planeSeparation(plane, support)

			if separation >= speculativeDistance {
				// Cache hit, shapes are separated
				cache.Hit = 1
				return
			}

			// Attempt face contact using cached feature
			var faceQuery separatingAxis
			faceQuery.normal = plane.Normal
			faceQuery.separation = 0.0
			faceQuery.indexA = int32(cache.IndexA)
			faceQuery.indexB = int32(vertexIndex)
			faceQuery.typ = FaceAxisA

			localCache := SATCache{}
			touching := buildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache)
			if touching == true && AbsFloat(cache.Separation-localCache.Separation) < linearSlop {
				// Cache hit, contact points generated
				cache.Hit = 1
				return
			}
		}

	case FaceAxisB:
		{
			assert(int32(cache.IndexB) < hullB.FaceCount, "cache->indexB < hullB->faceCount")

			// Check for separation using cached face
			plane := planesB[cache.IndexB]
			searchDirectionInA := Neg(RotateVector(transformBtoA.Q, plane.Normal))

			// todo use b3GetSupportWide
			vertexIndex := findHullSupportVertex(hullA, searchDirectionInA)
			support := InvTransformPoint(transformBtoA, pointsA[vertexIndex])
			separation := planeSeparation(plane, support)

			if separation >= speculativeDistance {
				// Cache hit, shapes are separated
				cache.Hit = 1
				return
			}

			// Attempt face contact using cached feature
			var faceQuery separatingAxis
			faceQuery.normal = Neg(plane.Normal)
			faceQuery.separation = 0.0
			faceQuery.indexA = int32(vertexIndex)
			faceQuery.indexB = int32(cache.IndexB)
			faceQuery.typ = FaceAxisB

			localCache := SATCache{}
			touching := buildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache)
			if touching == true && AbsFloat(cache.Separation-localCache.Separation) < linearSlop {
				// Cache hit, contact points generated
				cache.Hit = 1
				return
			}
		}

	case EdgePairAxis:
		{
			indexA := int(cache.IndexA)
			edge1 := &edgesA[indexA]
			twin1 := &edgesA[indexA+1]
			assert(int(edge1.Twin) == indexA+1 && int(twin1.Twin) == indexA, "edge1->twin == indexA + 1 && twin1->twin == indexA")

			pA := pointsA[edge1.Origin]
			qA := pointsA[twin1.Origin]
			eA := Sub(qA, pA)

			uA := planesA[edge1.Face].Normal
			vA := planesA[twin1.Face].Normal

			indexB := int(cache.IndexB)
			edge2 := &edgesB[indexB]
			twin2 := &edgesB[indexB+1]
			assert(int(edge2.Twin) == indexB+1 && int(twin2.Twin) == indexB, "edge2->twin == indexB + 1 && twin2->twin == indexB")

			pB := TransformPoint(transformBtoA, pointsB[edge2.Origin])
			qB := TransformPoint(transformBtoA, pointsB[twin2.Origin])
			eB := Sub(qB, pB)

			uB := RotateVector(transformBtoA.Q, planesB[edge2.Face].Normal)
			vB := RotateVector(transformBtoA.Q, planesB[twin2.Face].Normal)

			// flipping the signs of u2 and v2
			// cross(v2, u2) == cross(-v2, -u2)
			// so we still use -e2
			// but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
			cba := Dot(uB, eA)
			dba := Dot(vB, eA)
			adc := -Dot(uA, eB)
			bdc := -Dot(vA, eB)

			if cba*dba < 0.0 && adc*bdc < 0.0 && cba*bdc > 0.0 {
				// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
				squaredTolerance := ParallelEdgeTol * ParallelEdgeTol
				if MaxFloat(cba*cba, dba*dba) >= squaredTolerance*LengthSquared(eA) {
					// Transform reference center of the first hull into local space of the second hull
					t := cba / (cba - dba)
					axis := Lerp(uB, vB, t)
					validate(LengthSquared(axis) > 1000.0*fltMin, "b3LengthSquared( axis ) > 1000.0f * FLT_MIN")
					axis = Normalize(axis)
					separation := Dot(axis, Sub(qA, qB))

					if separation > speculativeDistance {
						// Cache hit, shapes are separated
						cache.Hit = 1
						return
					}

					// Try to rebuild contact from last features
					edgeQuery := separatingAxis{}
					edgeQuery.normal = Neg(axis)
					edgeQuery.separation = 0.0
					edgeQuery.indexA = int32(cache.IndexA)
					edgeQuery.indexB = int32(cache.IndexB)
					edgeQuery.typ = EdgePairAxis

					localCache := SATCache{}
					touching := buildEdgeContact(manifold, hullA, hullB, transformBtoA, edgeQuery, &localCache)

					// This separation tolerance may have a big impact on performance in some benchmarks.
					if touching && AbsFloat(cache.Separation-localCache.Separation) < linearSlop {
						// Cache hit, contact point generated
						cache.Hit = 1
						return
					}
				}
			}
		}

		// This case is for testing
	case ManualFaceAxisA:
		{
			axisQuery := computeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth)
			faceQuery := axisQuery.faceA
			buildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache)
			return
		}

		// This case is for testing
	case ManualFaceAxisB:
		{
			axisQuery := computeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth)
			faceQuery := axisQuery.faceB
			buildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache)
			return
		}

		// This case is for testing
	case ManualEdgePairAxis:
		{
			axisQuery := computeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth)
			edgeQuery := axisQuery.edge
			if edgeQuery.indexA != NullIndex {
				buildEdgeContact(manifold, hullA, hullB, transformBtoA, edgeQuery, cache)
			}
			return
		}

	default:
		assert(false, "false")
	}

	manifold.PointCount = 0
	*cache = SATCache{}

	axisQuery := computeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, true, simdWidth)

	if axisQuery.separatedFeature != InvalidAxis {
		// We found a separating axis
		cache.Type = uint8(axisQuery.separatedFeature)

		if axisQuery.separatedFeature == FaceAxisA {
			validate(axisQuery.faceA.separation > speculativeDistance, "axisQuery.faceA.separation > speculativeDistance")
			cache.Separation = axisQuery.faceA.separation
			cache.IndexA = uint16(axisQuery.faceA.indexA)
			cache.IndexB = uint16(axisQuery.faceA.indexB)
		} else if axisQuery.separatedFeature == FaceAxisB {
			validate(axisQuery.faceB.separation > speculativeDistance, "axisQuery.faceB.separation > speculativeDistance")
			cache.Separation = axisQuery.faceB.separation
			cache.IndexA = uint16(axisQuery.faceB.indexA)
			cache.IndexB = uint16(axisQuery.faceB.indexB)
		} else {
			assert(axisQuery.separatedFeature == EdgePairAxis, "axisQuery.separatedFeature == b3_edgePairAxis")
			validate(axisQuery.edge.separation > speculativeDistance, "axisQuery.edge.separation > speculativeDistance")
			cache.Separation = axisQuery.edge.separation
			cache.IndexA = uint16(axisQuery.edge.indexA)
			cache.IndexB = uint16(axisQuery.edge.indexB)
		}
		return
	}

	validate(axisQuery.faceA.separation <= speculativeDistance || axisQuery.faceB.separation <= speculativeDistance ||
		axisQuery.edge.separation <= speculativeDistance,
		"axisQuery.faceA.separation <= speculativeDistance || axisQuery.faceB.separation <= speculativeDistance || axisQuery.edge.separation <= speculativeDistance")

	if axisQuery.faceA.separation > axisQuery.faceB.separation {
		faceQuery := axisQuery.faceA
		validate(0 <= faceQuery.indexA && faceQuery.indexA < hullA.FaceCount, "0 <= faceQuery.indexA && faceQuery.indexA < hullA->faceCount")
		validate(0 <= faceQuery.indexB && faceQuery.indexB < hullB.VertexCount, "0 <= faceQuery.indexB && faceQuery.indexB < hullB->vertexCount")

		// Face contact A
		buildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache)

		validate(int32(cache.IndexA) < hullA.FaceCount, "cache->indexA < hullA->faceCount")
		validate(int32(cache.IndexB) < hullB.VertexCount, "cache->indexB < hullB->vertexCount")
	} else {
		faceQuery := axisQuery.faceB
		validate(0 <= faceQuery.indexA && faceQuery.indexA < hullA.VertexCount, "0 <= faceQuery.indexA && faceQuery.indexA < hullA->vertexCount")
		validate(0 <= faceQuery.indexB && faceQuery.indexB < hullB.FaceCount, "0 <= faceQuery.indexB && faceQuery.indexB < hullB->faceCount")

		// Face contact B
		buildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache)

		validate(int32(cache.IndexA) < hullA.VertexCount, "cache->indexA < hullA->vertexCount")
		validate(int32(cache.IndexB) < hullB.FaceCount, "cache->indexB < hullB->faceCount")
	}

	edgeQuery := axisQuery.edge

	if edgeQuery.indexA == NullIndex {
		// There are no valid edge pairs (all edges parallel)
		return
	}

	faceSeparation := MaxFloat(axisQuery.faceA.separation, axisQuery.faceB.separation)
	clipSeparation := cache.Separation
	edgeTol := linearSlop

	// Face contact can be empty if it is not the axis of maximum separation. It can also
	// be empty in narrow cases in the speculative region. If that case was important then
	// a GJK fallback would be used. So far it doesn't seem important.
	// Create edge contact if face contact fails or edge contact is significantly better.
	if (manifold.PointCount == 0 && edgeQuery.separation > faceSeparation) ||
		edgeQuery.separation > clipSeparation+edgeTol {
		assert(0 <= edgeQuery.indexA && edgeQuery.indexA < hullA.EdgeCount, "0 <= edgeQuery.indexA && edgeQuery.indexA < hullA->edgeCount")
		assert(0 <= edgeQuery.indexB && edgeQuery.indexB < hullB.EdgeCount, "0 <= edgeQuery.indexB && edgeQuery.indexB < hullB->edgeCount")

		// Edge contact
		// PORT: edgeManifold.points = &edgePoint becomes the caller's point buffer.
		// b3BuildEdgeContact writes the point only when it reports one, which is the only case
		// copied out below, so the face points survive a failed edge contact as upstream. A local
		// buffer would be copied into *manifold and escape to the heap.
		edgeManifold := LocalManifold{}
		edgeManifold.Points = manifold.Points

		edgeCache := SATCache{}
		buildEdgeContact(&edgeManifold, hullA, hullB, transformBtoA, edgeQuery, &edgeCache)

		// It is possible with speculation to have vertex-vertex collision that is missed by SAT,
		// so edge contact yields no points. In that case perhaps the face contact has some points.
		if edgeManifold.PointCount == 1 {
			// Copy edge manifold out, being careful to preserve manifold point buffer.
			*manifold = edgeManifold
			*cache = edgeCache
		}
	}
}

// CollideHulls collides two hulls.
func CollideHulls(manifold *LocalManifold, capacity int, hullA, hullB *HullData, transformBtoA Transform,
	cache *SATCache) {
	collideHullsAtWidth(manifold, capacity, hullA, hullB, transformBtoA, cache, GetSIMDWidth())
}
