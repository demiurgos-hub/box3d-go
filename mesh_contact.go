// Port of src/mesh_contact.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "sync/atomic"

// ---- mesh_contact.c ----

// maxPointsPerTriangle is B3_MAX_POINTS_PER_TRIANGLE.
const maxPointsPerTriangle = 32

// isSortedInts is b3IsSorted (B3_ENABLE_VALIDATION only upstream; here it is only called inside
// validate, which compiles out without the box3d_validate tag).
func isSortedInts(array []int, count int) bool {
	for i := 0; i < count-1; i++ {
		if array[i] >= array[i+1] {
			return false
		}
	}

	return true
}

// triangleQueryContext is b3TriangleQueryContext.
type triangleQueryContext struct {
	indices  []int
	capacity int
	count    int
}

// triangleQueryScratch holds the triangleQueryContext of the triangle queries.
var triangleQueryScratch scratchPool[triangleQueryContext]

// collectTriangleIndicesCallback is b3CollectTriangleIndicesCallback.
func collectTriangleIndicesCallback(a, b, c Vec3, triangleIndex int, context any) bool {
	triangleContext := context.(*triangleQueryContext)
	if triangleContext.count == triangleContext.capacity {
		return false
	}
	triangleContext.indices[triangleContext.count] = triangleIndex
	triangleContext.count += 1
	return triangleContext.count < triangleContext.capacity
}

// queryMeshTriangles is b3QueryMeshTriangles.
func queryMeshTriangles(indices []int, capacity int, mesh *Mesh, bounds AABB) int {
	// PORT: the context is handed to the query as `any`, so a local would escape to the heap; it
	// comes from a scratch pool instead (no allocation once warm).
	context := triangleQueryScratch.get()
	*context = triangleQueryContext{
		indices:  indices,
		capacity: capacity,
		count:    0,
	}

	QueryMesh(mesh, bounds, collectTriangleIndicesCallback, context)
	count := context.count
	*context = triangleQueryContext{}
	triangleQueryScratch.put(context)
	return count
}

// queryHeightFieldTriangles is b3QueryHeightFieldTriangles.
func queryHeightFieldTriangles(indices []int, capacity int, heightField *HeightFieldData, bounds AABB) int {
	// PORT: scratch pool, see queryMeshTriangles.
	context := triangleQueryScratch.get()
	*context = triangleQueryContext{
		indices:  indices,
		capacity: capacity,
		count:    0,
	}

	QueryHeightField(heightField, bounds, collectTriangleIndicesCallback, context)
	count := context.count
	*context = triangleQueryContext{}
	triangleQueryScratch.put(context)
	return count
}

// meshContactComplexOnce is the static s_once of b3RefreshCache.
// PORT: atomic so concurrent narrow phase workers do not race on it.
var meshContactComplexOnce atomic.Bool

// refreshCache is b3RefreshCache.
func refreshCache(contact *contact, shapeA *shape, xfA WorldTransform, bounds *AABB, arena arena) {
	assert(shapeA.typ == MeshShape || shapeA.typ == HeightShape, "shapeA->type == b3_meshShape || shapeA->type == b3_heightShape")

	meshContact := &contact.meshContact

	// If the dynamic body didn't move out of the cached query bounds we are done!
	if meshContact.queryBounds.Contains(*bounds) {
		if shapeA.typ == MeshShape {
			for i := 0; i < len(contact.meshContact.triangleCache); i++ {
				assert(0 <= contact.meshContact.triangleCache[i].triangleIndex &&
					contact.meshContact.triangleCache[i].triangleIndex < shapeA.mesh.Data.TriangleCount,
					"0 <= triangleIndex && triangleIndex < shapeA->mesh.data->triangleCount")
			}
		}

		return
	}

	// Enlarge to the query bounds to absorb small movement
	radius := MaxAABBMargin() + SpeculativeDistance()
	extension := Vec3{radius, radius, radius}
	meshContact.queryBounds.LowerBound = Sub(bounds.LowerBound, extension)
	meshContact.queryBounds.UpperBound = Add(bounds.UpperBound, extension)

	// Query triangles
	triangleCapacity := MaxMeshContactTriangles
	triangleIndices := bump[int](&arena, MaxMeshContactTriangles)

	// Bounds are in world space. Convert to the local mesh frame. The broadphase bounds are float,
	// so the demoted mesh transform is the matching float world frame (exact in float mode).
	meshTransform := ToRelativeTransform(xfA, PosZero)
	localBounds := meshContact.queryBounds.Transform(InvertTransform(meshTransform))
	var triangleCount int
	if shapeA.typ == MeshShape {
		triangleCount = queryMeshTriangles(triangleIndices, triangleCapacity, &shapeA.mesh, localBounds)
	} else {
		assert(shapeA.typ == HeightShape, "shapeA->type == b3_heightShape")
		triangleCount = queryHeightFieldTriangles(triangleIndices, triangleCapacity, shapeA.heightField, localBounds)
	}

	if triangleCount == triangleCapacity {
		if meshContactComplexOnce.CompareAndSwap(false, true) {
			log("WARNING: complex mesh detected, triangle buffer capacity of %d reached", triangleCapacity)
		}
	}

	// Triangle indices must be sorted to match caches.
	validate(isSortedInts(triangleIndices, triangleCount), "b3IsSorted( triangleIndices, triangleCount )")

	// Create new contact cache and match with old one
	contactCaches := bump[contactCache](&arena, triangleCount)

	index2 := 0
	for index1 := 0; index1 < triangleCount; index1++ {
		contactCaches[index1] = contactCache{}

		for index2 < len(contact.meshContact.triangleCache) &&
			int(contact.meshContact.triangleCache[index2].triangleIndex) < triangleIndices[index1] {
			index2 += 1
		}

		if index2 < len(contact.meshContact.triangleCache) &&
			int(contact.meshContact.triangleCache[index2].triangleIndex) == triangleIndices[index1] {
			contactCaches[index1] = contact.meshContact.triangleCache[index2].cache
		}
	}

	// Save new cache
	arrayResize(&contact.meshContact.triangleCache, triangleCount)
	for i := 0; i < triangleCount; i++ {
		contact.meshContact.triangleCache[i] = triangleCache{int32(triangleIndices[i]), contactCaches[i]}

		if shapeA.typ == MeshShape {
			assert(0 <= contact.meshContact.triangleCache[i].triangleIndex &&
				contact.meshContact.triangleCache[i].triangleIndex < shapeA.mesh.Data.TriangleCount,
				"0 <= triangleIndex && triangleIndex < shapeA->mesh.data->triangleCount")
		}
	}
}

// tentativeTriangle is b3TentativeTriangle.
type tentativeTriangle struct {
	squaredDistance float32
	index           int32
}

// maxEdgeCount is B3_MAX_EDGE_COUNT.
const maxEdgeCount = 64

// foundEdges is b3FoundEdges.
type foundEdges struct {
	keys  [maxEdgeCount]uint64
	count int
}

// addEdge is b3AddEdge.
func addEdge(edges *foundEdges, vertex1, vertex2 int32) bool {
	i1 := uint64(MinInt(int(vertex1), int(vertex2)))
	i2 := uint64(MaxInt(int(vertex1), int(vertex2)))
	key := i1<<32 | i2

	count := edges.count
	for i := 0; i < count; i++ {
		if edges.keys[i] == key {
			return false
		}
	}

	if count == maxEdgeCount {
		// This will lead to a potential ghost collision
		return true
	}

	edges.keys[count] = key
	edges.count += 1

	return true
}

// findEdge is b3FindEdge.
func findEdge(edges *foundEdges, vertex1, vertex2 int32) bool {
	i1 := uint64(MinInt(int(vertex1), int(vertex2)))
	i2 := uint64(MaxInt(int(vertex1), int(vertex2)))
	key := i1<<32 | i2

	count := edges.count
	for i := 0; i < count; i++ {
		if edges.keys[i] == key {
			return true
		}
	}

	return false
}

// PORT: b3TrianglesShareEdge is inside #if 0 upstream and is not ported.

// maxVertexCount is B3_MAX_VERTEX_COUNT.
const maxVertexCount = 64

// foundVertices is b3FoundVertices.
type foundVertices struct {
	keys  [maxVertexCount]int32
	count int
}

// addVertex is b3AddVertex.
func addVertex(vertices *foundVertices, vertex int32) bool {
	key := vertex

	count := vertices.count
	for i := 0; i < count; i++ {
		if vertices.keys[i] == key {
			return false
		}
	}

	if count == maxVertexCount {
		// This will lead to a potential ghost collision
		return true
	}

	vertices.keys[count] = key
	vertices.count += 1

	return true
}

// PORT: B3_MAX_MANIFOLD_POINTS is 4 (constants.go), so the #if B3_MAX_MANIFOLD_POINTS == 4 branch
// is the one ported.

// isBetterCullCandidate is b3IsBetterCullCandidate.
// Returns true if (score, separation) should replace (bestScore, bestSeparation).
func isBetterCullCandidate(score, separation, bestScore, bestSeparation, scoreTol, separationTol float32) bool {
	if score > bestScore+scoreTol {
		return true
	}
	if score < bestScore-scoreTol {
		return false
	}

	// Break the tie using separation
	return separation < bestSeparation-separationTol
}

// cullPoints is b3CullPoints.
func cullPoints(points []Point2D, count int) int {
	if count <= 1 {
		return count
	}

	tol := float32(0.25 * LinearSlop())
	tolSqr := float32(tol * tol)
	separationTol := LinearSlop()

	var finalPoints [4]Point2D
	count1 := count

	// Step 1: the two points with the largest distance, ties broken by deepest combined separation
	bestScore := float32(0.0)
	bestSeparation := float32(fltMax)
	bestIndex1 := NullIndex
	bestIndex2 := NullIndex

	for i := 0; i < count1; i++ {
		p1 := points[i].P
		for j := i + 1; j < count1; j++ {
			score := distanceSquared2(p1, points[j].P)
			// Separation sum heuristic
			separation := points[i].Separation + points[j].Separation

			if isBetterCullCandidate(score, separation, bestScore, bestSeparation, tolSqr, separationTol) {
				bestIndex1 = i
				bestIndex2 = j
				bestScore = score
				bestSeparation = separation
			}
		}
	}

	if bestScore < tolSqr {
		// Choose deepest point
		deepestIndex := 0
		for i := 1; i < count1; i++ {
			if points[i].Separation < points[deepestIndex].Separation {
				deepestIndex = i
			}
		}

		if deepestIndex != 0 {
			points[0] = points[deepestIndex]
		}
		return 1
	}

	finalPoints[0] = points[bestIndex1]
	finalPoints[1] = points[bestIndex2]

	// Cull
	points[bestIndex2] = points[count1-1]
	points[bestIndex1] = points[count1-2]
	count1 -= 2

	if count1 == 0 {
		points[0] = finalPoints[0]
		points[1] = finalPoints[1]
		return 2
	}

	// First anchor point
	a := finalPoints[0].P

	// Second anchor point
	b := finalPoints[1].P
	ba := sub2(b, a)
	// float length = b3Length2( ba );
	// float areaTol = tol * length;

	// Step 2: find the point with the maximum triangular area, ties broken by deepest separation
	bestScore = 0.0
	bestSeparation = fltMax
	bestIndex := NullIndex
	bestSignedArea := float32(0.0)
	for i := 0; i < count1; i++ {
		p := points[i].P
		signedArea := cross2(ba, sub2(p, a))
		score := AbsFloat(signedArea)

		if isBetterCullCandidate(score, points[i].Separation, bestScore, bestSeparation, tolSqr, separationTol) {
			bestSignedArea = signedArea
			bestScore = score
			bestSeparation = points[i].Separation
			bestIndex = i
		}
	}

	if bestIndex == NullIndex {
		// All points collinear
		points[0] = finalPoints[0]
		points[1] = finalPoints[1]
		return 2
	}

	// Store best point
	finalPoints[2] = points[bestIndex]

	if count1 == 1 {
		points[0] = finalPoints[0]
		points[1] = finalPoints[1]
		points[2] = finalPoints[2]
		return 3
	}

	// Cull
	points[bestIndex] = points[count1-1]
	count1 -= 1

	// Step 4: get the point that adds the most area outside the current triangle

	// Third anchor
	c := finalPoints[2].P

	// Ensure CCW ordering
	if bestSignedArea < 0.0 {
		b, c = c, b
		ba = sub2(b, a)
	}

	cb := sub2(c, b)
	ac := sub2(a, c)

	bestScore = 0.0
	bestSeparation = fltMax
	bestIndex = NullIndex
	for i := 0; i < count1; i++ {
		p := points[i].P
		u1 := cross2(sub2(p, a), ba)
		u2 := cross2(sub2(p, b), cb)
		u3 := cross2(sub2(p, c), ac)
		score := MaxFloat(u1, MaxFloat(u2, u3))

		// Use the area tolerance for collinear points and hysteresis
		if isBetterCullCandidate(score, points[i].Separation, bestScore, bestSeparation, tolSqr, separationTol) {
			bestScore = score
			bestSeparation = points[i].Separation
			bestIndex = i
		}
	}

	if bestIndex == NullIndex {
		// No additional area
		points[0] = finalPoints[0]
		points[1] = finalPoints[1]
		points[2] = finalPoints[2]
		return 3
	}

	// Store best point
	finalPoints[3] = points[bestIndex]

	// Full quad
	points[0] = finalPoints[0]
	points[1] = finalPoints[1]
	points[2] = finalPoints[2]
	points[3] = finalPoints[3]
	return 4
}

// reduceCluster is b3ReduceCluster.
func reduceCluster(points []LocalManifoldPoint, count1 int, normal Vec3, arena arena) int {
	targetCount := 1
	if count1 <= targetCount {
		return count1
	}

	pts := bump[Point2D](&arena, count1)
	u := Perp(normal)
	v := Cross(normal, u)
	origin := points[0].Point

	for i := 0; i < count1; i++ {
		d := Sub(points[i].Point, origin)
		pts[i].P = Vec2{Dot(d, u), Dot(d, v)}
		pts[i].Separation = points[i].Separation
		pts[i].OriginalIndex = int32(i)
	}

	// PORT: B3_MAX_MANIFOLD_POINTS == 4 branch (b3CullPoints); the b3Hull2D branch is not compiled.
	hull := pts
	count2 := cullPoints(pts, count1)
	assert(count2 <= MaxManifoldPoints, "count2 <= B3_MAX_MANIFOLD_POINTS")

	var finalPoints [MaxManifoldPoints]LocalManifoldPoint
	for i := 0; i < count2; i++ {
		index := int(hull[i].OriginalIndex)
		assert(0 <= index && index < count1, "0 <= index && index < count1")
		finalPoints[i] = points[index]
	}

	copy(points[:count2], finalPoints[:count2])
	return count2
}

// cluster is b3Cluster.
type cluster struct {
	manifoldNormal Vec3
	triangleNormal Vec3
	points         []LocalManifoldPoint // PORT: b3LocalManifoldPoint* into the cluster point buffer
	pointCapacity  int
	pointCount     int
}

// computeMeshManifolds is b3ComputeMeshManifolds.
// PORT: const uint16_t* materialMap is a slice; NULL is nil.
func computeMeshManifolds(world *world, workerIndex int, contact *contact, shapeA *shape, materialMap []uint16,
	xfA WorldTransform, shapeB *shape, xfB WorldTransform, isFast bool, arena arena) bool {
	return computeMeshManifoldsWithRoundedCull(world, workerIndex, contact, shapeA, materialMap, xfA, shapeB, xfB, isFast, arena, true)
}

func computeMeshManifoldsWithRoundedCull(world *world, workerIndex int, contact *contact, shapeA *shape, materialMap []uint16,
	xfA WorldTransform, shapeB *shape, xfB WorldTransform, isFast bool, arena arena, roundedCull bool) bool {
	assert(shapeA.typ == MeshShape || shapeA.typ == HeightShape, "shapeA->type == b3_meshShape || shapeA->type == b3_heightShape")

	context := arrayGet(world.taskContexts, workerIndex)

	refreshCache(contact, shapeA, xfA, &shapeB.aabb, arena)

	// Collide with triangles and build manifolds
	meshContact := &contact.meshContact
	triangleCount := len(meshContact.triangleCache)

	acceptedManifolds := bump[*LocalManifold](&arena, triangleCount)
	acceptedManifoldCount := 0
	tentativeManifolds := bump[*LocalManifold](&arena, triangleCount)
	tentativeManifoldCount := 0
	tentativeTriangles := bump[tentativeTriangle](&arena, triangleCount)
	tentativeTriangleCount := 0

	var foundEdges foundEdges
	var foundVertices foundVertices
	foundEdges.count = 0
	foundVertices.count = 0

	// This transform converts from mesh frame into the shapeB frame
	transformAtoB := InvMulWorldTransforms(xfB, xfA)
	relativeMatrix := MakeMatrixFromQuat(transformAtoB.Q)
	linearSlop := LinearSlop()

	// This should push apart shapes after a time of impact event.
	// In the past I've called this `polygon skin`, but PhysX and Unreal
	// call it `rest offset` which seems appropriate in this case.
	// It leads to a small visual gap but seems to improve the quality of mesh
	// collision, especially for hull versus mesh.
	restOffset := MeshRestOffset()
	enableSpeculative := contact.flags&enableSpeculativePoints != 0

	// Make room for clip points
	pointBufferCapacity := maxPointsPerTriangle * triangleCount

	pointBuffer := bump[LocalManifoldPoint](&arena, pointBufferCapacity)
	totalPointCount := 0

	manifoldBuffer := bump[LocalManifold](&arena, triangleCount)
	manifoldCount := 0

	triangleCaches := meshContact.triangleCache

	var hullB *HullData
	if shapeB.typ == HullShape {
		hullB = shapeB.hull
	}
	hullSpeculativeDistance := float32(0.0)
	if enableSpeculative {
		hullSpeculativeDistance = SpeculativeDistance()
	}
	hullMargin := Vec3{hullSpeculativeDistance, hullSpeculativeDistance, hullSpeculativeDistance}

	survivorIndices := bump[int](&arena, triangleCount)
	survivorVertices := bump[Vec3](&arena, 3*triangleCount)
	survivorCount := 0

	var meshTriangles []MeshTriangle
	var meshVertices []Vec3
	meshScale := Vec3One
	meshFlipped := false
	if shapeA.typ == MeshShape {
		meshTriangles = GetMeshTriangles(shapeA.mesh.Data)
		meshVertices = GetMeshVertices(shapeA.mesh.Data)
		meshScale = shapeA.mesh.Scale
		meshFlipped = float32(meshScale.X*meshScale.Y)*meshScale.Z < 0.0
	}

	cxx := splatW(relativeMatrix.Cx.X)
	cxy := splatW(relativeMatrix.Cx.Y)
	cxz := splatW(relativeMatrix.Cx.Z)
	cyx := splatW(relativeMatrix.Cy.X)
	cyy := splatW(relativeMatrix.Cy.Y)
	cyz := splatW(relativeMatrix.Cy.Z)
	czx := splatW(relativeMatrix.Cz.X)
	czy := splatW(relativeMatrix.Cz.Y)
	czz := splatW(relativeMatrix.Cz.Z)
	px := splatW(transformAtoB.P.X)
	py := splatW(transformAtoB.P.Y)
	pz := splatW(transformAtoB.P.Z)

	var hullBounds AABB
	cullBounds := hullB != nil
	if hullB != nil {
		hullBounds = hullB.AABB
	} else if roundedCull {
		hullBounds, cullBounds = meshRoundedContactBounds(shapeB)
		// Rounded kernels always use their existing speculative distance. It is
		// already included in these outward-rounded local bounds.
		hullMargin = Vec3Zero
	}
	marginW := splatW(hullMargin.X)
	hullLowerX := splatW(hullBounds.LowerBound.X)
	hullLowerY := splatW(hullBounds.LowerBound.Y)
	hullLowerZ := splatW(hullBounds.LowerBound.Z)
	hullUpperX := splatW(hullBounds.UpperBound.X)
	hullUpperY := splatW(hullBounds.UpperBound.Y)
	hullUpperZ := splatW(hullBounds.UpperBound.Z)

	// PORT: b3FloatW4 and the W4 functions are the 4-lane floatW API (laneCount is 4).
	for baseIndex := 0; baseIndex < triangleCount; baseIndex += 4 {
		laneCount := MinInt(4, triangleCount-baseIndex)

		var lanes [3][4]Vec3
		for lane := 0; lane < 4; lane++ {
			index := baseIndex + MinInt(lane, laneCount-1)
			triangleIndex := int(triangleCaches[index].triangleIndex)

			if meshVertices != nil {
				meshTriangle := meshTriangles[triangleIndex]
				lanes[0][lane] = Mul(meshScale, meshVertices[meshTriangle.Index1])
				if meshFlipped {
					lanes[1][lane] = Mul(meshScale, meshVertices[meshTriangle.Index3])
					lanes[2][lane] = Mul(meshScale, meshVertices[meshTriangle.Index2])
				} else {
					lanes[1][lane] = Mul(meshScale, meshVertices[meshTriangle.Index2])
					lanes[2][lane] = Mul(meshScale, meshVertices[meshTriangle.Index3])
				}
			} else {
				assert(shapeA.typ == HeightShape, "shapeA->type == b3_heightShape")
				triangle := getHeightFieldTriangle(shapeA.heightField, triangleIndex)
				lanes[0][lane] = triangle.vertices[0]
				lanes[1][lane] = triangle.vertices[1]
				lanes[2][lane] = triangle.vertices[2]
			}
		}

		// The scalar lane backend otherwise copies twelve four-lane values
		// through each multiply/add. Preserve the same rounding/order directly.
		if simdNone {
			survivorCount = meshScalarSurvivors(relativeMatrix, transformAtoB.P, &lanes, laneCount,
				cullBounds, hullBounds, hullMargin.X, triangleCaches, baseIndex,
				survivorIndices, survivorVertices, survivorCount)
			continue
		}

		// Transform triangle into the shape frame
		var vx, vy, vz [3]floatW
		for k := 0; k < 3; k++ {
			ax := setW(lanes[k][0].X, lanes[k][1].X, lanes[k][2].X, lanes[k][3].X)
			ay := setW(lanes[k][0].Y, lanes[k][1].Y, lanes[k][2].Y, lanes[k][3].Y)
			az := setW(lanes[k][0].Z, lanes[k][1].Z, lanes[k][2].Z, lanes[k][3].Z)

			vx[k] = addW(addW(addW(mulW(cxx, ax), mulW(cyx, ay)), mulW(czx, az)), px)
			vy[k] = addW(addW(addW(mulW(cxy, ax), mulW(cyy, ay)), mulW(czy, az)), py)
			vz[k] = addW(addW(addW(mulW(cxz, ax), mulW(cyz, ay)), mulW(czz, az)), pz)
		}

		keepMask := (uint32(1) << laneCount) - 1
		if cullBounds {
			lowerX := subW(minW(vx[0], minW(vx[1], vx[2])), marginW)
			lowerY := subW(minW(vy[0], minW(vy[1], vy[2])), marginW)
			lowerZ := subW(minW(vz[0], minW(vz[1], vz[2])), marginW)
			upperX := addW(maxW(vx[0], maxW(vx[1], vx[2])), marginW)
			upperY := addW(maxW(vy[0], maxW(vy[1], vy[2])), marginW)
			upperZ := addW(maxW(vz[0], maxW(vz[1], vz[2])), marginW)

			separated := orW(lessThanW(upperX, hullLowerX), greaterThanW(lowerX, hullUpperX))
			separated = orW(separated, orW(lessThanW(upperY, hullLowerY), greaterThanW(lowerY, hullUpperY)))
			separated = orW(separated, orW(lessThanW(upperZ, hullLowerZ), greaterThanW(lowerZ, hullUpperZ)))

			cullMask := keepMask & uint32(moveMaskW(separated))
			keepMask &= ^cullMask

			for cullMask != 0 {
				lane := ctz32(cullMask)
				cullMask &= cullMask - 1
				triangleCaches[baseIndex+int(lane)].cache.satCache = SATCache{}
			}
		}

		if keepMask == 0 {
			continue
		}

		var x, y, z [3][4]float32
		for k := 0; k < 3; k++ {
			storeW(x[k][:], vx[k])
			storeW(y[k][:], vy[k])
			storeW(z[k][:], vz[k])
		}

		for keepMask != 0 {
			lane := ctz32(keepMask)
			keepMask &= keepMask - 1

			survivor := survivorVertices[3*survivorCount:]
			survivor[0] = Vec3{x[0][lane], y[0][lane], z[0][lane]}
			survivor[1] = Vec3{x[1][lane], y[1][lane], z[1][lane]}
			survivor[2] = Vec3{x[2][lane], y[2][lane], z[2][lane]}
			survivorIndices[survivorCount] = baseIndex + int(lane)
			survivorCount += 1
		}
	}

	for survivorIndex := 0; survivorIndex < survivorCount && totalPointCount+3 < pointBufferCapacity; survivorIndex++ {
		index := survivorIndices[survivorIndex]
		triangleIndex := int(triangleCaches[index].triangleIndex)
		vertices := survivorVertices[3*survivorIndex : 3*survivorIndex+3]

		var triangle triangle
		if shapeA.typ == MeshShape {
			triangle = getMeshTriangle(&shapeA.mesh, triangleIndex)
		} else {
			assert(shapeA.typ == HeightShape, "shapeA->type == b3_heightShape")
			triangle = getHeightFieldTriangle(shapeA.heightField, triangleIndex)
		}

		cache := &triangleCaches[index].cache
		pointCapacity := pointBufferCapacity - totalPointCount
		manifold := &manifoldBuffer[manifoldCount]
		manifold.Points = pointBuffer[totalPointCount:]
		manifold.PointCount = 0
		manifold.TriangleFlags = triangle.flags
		manifold.Feature = FeatureNone

		switch shapeB.typ {
		case CapsuleShape:
			CollideTriangleAndCapsule(manifold, pointCapacity, vertices, &shapeB.capsule, &cache.simplexCache)

		case HullShape:
			// Cached edge contact is dangerous at high speed because the hull can rotate around the edge and tunnel
			// through the triangle.
			if isFast && cache.satCache.Type == uint8(EdgePairAxis) {
				cache.satCache = SATCache{}
			}

			CollideTriangleAndHull(manifold, pointCapacity, vertices[0], vertices[1], vertices[2], int(triangle.flags), hullB,
				&cache.satCache, enableSpeculative)
			context.satCallCount += 1
			context.satCacheHitCount += int32(cache.satCache.Hit)

		case SphereShape:
			CollideTriangleAndSphere(manifold, pointCapacity, vertices, &shapeB.sphere)

		default:
			assert(false, "false")
			return false
		}

		manifoldPointCount := int(manifold.PointCount)

		if manifoldPointCount > 0 {
			assert(manifold.Feature != FeatureNone, "manifold->feature != b3_featureNone")

			manifoldCount += 1
			totalPointCount += manifoldPointCount
			manifold.TriangleIndex = int32(triangleIndex)
			manifold.TriangleNormal = makeNormalFromPoints(vertices[0], vertices[1], vertices[2])
			manifold.I1 = triangle.i1
			manifold.I2 = triangle.i2
			manifold.I3 = triangle.i3

			if manifold.Feature == FeatureTriangleFace || forceGhostCollisions {
				_ = addEdge(&foundEdges, triangle.i1, triangle.i2)
				_ = addEdge(&foundEdges, triangle.i2, triangle.i3)
				_ = addEdge(&foundEdges, triangle.i3, triangle.i1)
				_ = addVertex(&foundVertices, triangle.i1)
				_ = addVertex(&foundVertices, triangle.i2)
				_ = addVertex(&foundVertices, triangle.i3)

				acceptedManifolds[acceptedManifoldCount] = manifold
				acceptedManifoldCount++
			} else if manifold.Feature == FeatureHullFace {
				cosNormalAngle := Dot(manifold.TriangleNormal, manifold.Normal)
				if cosNormalAngle > 0.5 {
					_ = addEdge(&foundEdges, triangle.i1, triangle.i2)
					_ = addEdge(&foundEdges, triangle.i2, triangle.i3)
					_ = addEdge(&foundEdges, triangle.i3, triangle.i1)
					_ = addVertex(&foundVertices, triangle.i1)
					_ = addVertex(&foundVertices, triangle.i2)
					_ = addVertex(&foundVertices, triangle.i3)

					acceptedManifolds[acceptedManifoldCount] = manifold
					acceptedManifoldCount++
				} else {
					minSeparation := manifold.Points[0].Separation
					for i := 1; i < manifoldPointCount; i++ {
						minSeparation = MinFloat(minSeparation, manifold.Points[i].Separation)
					}

					if minSeparation < -2.0*linearSlop {
						// Deep overlap
						_ = addEdge(&foundEdges, triangle.i1, triangle.i2)
						_ = addEdge(&foundEdges, triangle.i2, triangle.i3)
						_ = addEdge(&foundEdges, triangle.i3, triangle.i1)
						_ = addVertex(&foundVertices, triangle.i1)
						_ = addVertex(&foundVertices, triangle.i2)
						_ = addVertex(&foundVertices, triangle.i3)
						acceptedManifolds[acceptedManifoldCount] = manifold
						acceptedManifoldCount++
					} else {
						tentativeTriangle := tentativeTriangle{squaredDistance: manifold.SquaredDistance,
							index: int32(tentativeManifoldCount)}
						tentativeTriangles[tentativeTriangleCount] = tentativeTriangle
						tentativeTriangleCount++
						tentativeManifolds[tentativeManifoldCount] = manifold
						tentativeManifoldCount++
					}
				}
			} else {
				tentativeTriangle := tentativeTriangle{squaredDistance: manifold.SquaredDistance,
					index: int32(tentativeManifoldCount)}
				tentativeTriangles[tentativeTriangleCount] = tentativeTriangle
				tentativeTriangleCount++
				tentativeManifolds[tentativeManifoldCount] = manifold
				tentativeManifoldCount++
			}
		}
	}

	assert(acceptedManifoldCount <= triangleCount, "acceptedManifoldCount <= triangleCount")
	assert(tentativeManifoldCount <= triangleCount, "tentativeManifoldCount <= triangleCount")
	assert(tentativeTriangleCount <= triangleCount, "tentativeTriangleCount <= triangleCount")

	if shapeB.typ == SphereShape {
		// Sort triangles so the closest triangles are processed first
		{
			// PORT: QSORT with the LESS and SWAP macros as closures (qsort.go keeps the unstable
			// order of equal keys call for call).
			qsort(tentativeTriangleCount,
				func(i, j int) bool {
					return tentativeTriangles[i].squaredDistance < tentativeTriangles[j].squaredDistance
				},
				func(i, j int) {
					tmp := tentativeTriangles[i]
					tentativeTriangles[i] = tentativeTriangles[j]
					tentativeTriangles[j] = tmp
				})
		}

		// Add tentative manifolds in sorted order. Avoid adding manifolds that generate ghost collisions.
		for i := 0; i < tentativeTriangleCount; i++ {
			m := tentativeManifolds[tentativeTriangles[i].index]

			addedEdge1 := addEdge(&foundEdges, m.I1, m.I2)
			addedEdge2 := addEdge(&foundEdges, m.I2, m.I3)
			addedEdge3 := addEdge(&foundEdges, m.I3, m.I1)
			addedVertex1 := addVertex(&foundVertices, m.I1)
			addedVertex2 := addVertex(&foundVertices, m.I2)
			addedVertex3 := addVertex(&foundVertices, m.I3)

			feature := m.Feature
			shouldCollide := false
			switch feature {
			case FeatureNone, FeatureTriangleFace:
				assert(false, "false")

			case FeatureEdge1:
				shouldCollide = addedEdge1

			case FeatureEdge2:
				shouldCollide = addedEdge2

			case FeatureEdge3:
				shouldCollide = addedEdge3

			case FeatureVertex1:
				shouldCollide = addedVertex1

			case FeatureVertex2:
				shouldCollide = addedVertex2

			case FeatureVertex3:
				shouldCollide = addedVertex3

			default:
				assert(false, "false")
			}

			if shouldCollide == true {
				acceptedManifolds[acceptedManifoldCount] = m
				acceptedManifoldCount++
			}
		}
	} else {
		// Problem: hull can tunnel if time of impact is at concave edge
		// Example: flat box sliding down a ramp to a flat bottom
		// Solution: only ignore flat edges
		for i := 0; i < tentativeManifoldCount; i++ {
			m := tentativeManifolds[i]
			triangleFlags := MeshEdgeFlags(m.TriangleFlags)

			if (triangleFlags & AllFlatEdges) == AllFlatEdges {
				continue
			}

			if (triangleFlags & FlatEdge1) == FlatEdge1 {
				if findEdge(&foundEdges, m.I1, m.I2) {
					continue
				}
			}

			if (triangleFlags & FlatEdge2) == FlatEdge2 {
				if findEdge(&foundEdges, m.I2, m.I3) {
					continue
				}
			}

			if (triangleFlags & FlatEdge3) == FlatEdge3 {
				if findEdge(&foundEdges, m.I3, m.I1) {
					continue
				}
			}

			acceptedManifolds[acceptedManifoldCount] = m
			acceptedManifoldCount++
		}
	}

	assert(acceptedManifoldCount <= triangleCount, "acceptedManifoldCount <= triangleCount")

	if acceptedManifoldCount == 0 {
		if contact.manifoldCount > 0 {
			freeManifolds(world, contact.manifolds, int(contact.manifoldCount))
			contact.manifolds = nil
			contact.manifoldCount = 0
		}
		return false
	}

	clusters := bump[cluster](&arena, acceptedManifoldCount)
	clusterMemberships := bump[int](&arena, acceptedManifoldCount)

	// Cluster tolerance is tighter than the warm starting manifold matching tolerance. These
	// serve different purposes.
	const clusterThreshold float32 = 0.996
	clusterCount := 0
	clusterPointCount := 0
	for i := 0; i < acceptedManifoldCount; i++ {
		clusterMemberships[i] = NullIndex

		manifold := acceptedManifolds[i]
		clusterPointCount += int(manifold.PointCount)

		// Cluster based on the triangle normal and contact normal.
		// The first cluster found is accepted because the tolerance is tight.
		// todo consider requiring the triangles to be connect by an edge.
		// todo consider looking for the best cluster instead of the first one within tolerance
		// This bool is here to allow quick testing with and without clustering.
		allowClustering := true
		manifoldNormal := manifold.Normal
		triangleNormal := manifold.TriangleNormal
		clusterIndex := NullIndex
		for j := 0; j < clusterCount && allowClustering; j++ {
			cosManifoldAngle := Dot(clusters[j].manifoldNormal, manifoldNormal)
			cosTriangleAngle := Dot(clusters[j].triangleNormal, triangleNormal)
			if cosManifoldAngle <= clusterThreshold || cosTriangleAngle <= clusterThreshold {
				continue
			}

			// PORT: the #if 0 edge connectivity variant is not ported.

			// Found a cluster
			clusterIndex = j
			break
		}

		if clusterIndex != NullIndex {
			clusterMemberships[i] = clusterIndex
			clusters[clusterIndex].pointCapacity += int(manifold.PointCount)
		} else {
			clusters[clusterCount].manifoldNormal = manifoldNormal
			clusters[clusterCount].triangleNormal = triangleNormal
			clusters[clusterCount].pointCapacity = int(manifold.PointCount)
			clusterMemberships[i] = clusterCount
			clusterCount += 1
		}
	}

	if clusterPointCount == 0 {
		return false
	}

	// Setup clusters
	clusterPoints := bump[LocalManifoldPoint](&arena, clusterPointCount)
	pointOffset := 0

	for i := 0; i < clusterCount; i++ {
		cluster := &clusters[i]
		cluster.points = clusterPoints[pointOffset : pointOffset+cluster.pointCapacity]
		cluster.pointCount = 0
		pointOffset += cluster.pointCapacity
	}

	// Populate clusters
	for i := 0; i < acceptedManifoldCount; i++ {
		clusterIndex := clusterMemberships[i]
		if clusterIndex == NullIndex {
			continue
		}

		assert(0 <= clusterIndex && clusterIndex < clusterCount, "0 <= clusterIndex && clusterIndex < clusterCount")

		am := acceptedManifolds[i]
		cm := &clusters[clusterIndex]
		for j := 0; j < int(am.PointCount); j++ {
			assert(cm.pointCount < cm.pointCapacity, "cm->pointCount < cm->pointCapacity")
			ap := &am.Points[j]
			cp := &cm.points[cm.pointCount]

			cp.TriangleIndex = am.TriangleIndex
			cp.Point = ap.Point
			cp.Separation = ap.Separation
			cp.Pair = ap.Pair
			cm.pointCount += 1
		}
	}

	// Simplify clusters
	for i := 0; i < clusterCount; i++ {
		cm := &clusters[i]
		assert(cm.pointCount == cm.pointCapacity, "cm->pointCount == cm->pointCapacity")
		reducedCount := reduceCluster(cm.points, cm.pointCount, cm.triangleNormal, arena)
		cm.pointCount = reducedCount
	}

	// Make a temporary copy of previous manifolds
	oldManifoldCount := int(contact.manifoldCount)
	var oldManifolds []Manifold
	if oldManifoldCount > 0 {
		oldManifolds = bump[Manifold](&arena, oldManifoldCount)
		copy(oldManifolds, contact.manifolds[:oldManifoldCount])
	}

	// Resize manifolds if needed
	if oldManifoldCount != clusterCount {
		freeManifolds(world, contact.manifolds, int(contact.manifoldCount))
		contact.manifolds = allocateManifolds(world, clusterCount)
		contact.manifoldCount = int32(uint16(clusterCount))
	} else {
		// Mem zero manifolds
		clear(contact.manifolds[:contact.manifoldCount])
	}

	var consumed []bool
	if oldManifoldCount > 0 {
		consumed = bump[bool](&arena, oldManifoldCount)
		clear(consumed)
	}

	matrixB := MakeMatrixFromQuat(xfB.Q)
	offsetA := SubPos(xfB.P, xfA.P)

	const normalMatchTolerance float32 = 0.995
	for i := 0; i < clusterCount; i++ {
		cm := &clusters[i]
		pointCount := cm.pointCount
		assert(0 < pointCount && pointCount <= MaxManifoldPoints, "0 < pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS")

		manifold := &contact.manifolds[i]
		manifold.PointCount = int32(pointCount)
		manifold.Normal = MulMV(matrixB, cm.manifoldNormal)

		clusterNormal := MulMV(matrixB, cm.manifoldNormal)
		bestDot := normalMatchTolerance
		bestIndex := NullIndex

		for j := 0; j < oldManifoldCount; j++ {
			if consumed[j] == true {
				continue
			}

			dot := Dot(oldManifolds[j].Normal, clusterNormal)
			if dot > bestDot {
				bestIndex = j
				bestDot = dot
			}
		}

		var matchedManifold *Manifold
		if bestIndex != NullIndex {
			matchedManifold = &oldManifolds[bestIndex]
			manifold.FrictionImpulse = matchedManifold.FrictionImpulse
			manifold.RollingImpulse = matchedManifold.RollingImpulse
			manifold.TwistImpulse = matchedManifold.TwistImpulse
			consumed[bestIndex] = true
		}

		for j := 0; j < pointCount; j++ {
			source := &cm.points[j]
			target := &manifold.Points[j]

			// Contact points are computed in frame B
			target.AnchorB = MulMV(matrixB, source.Point)
			target.AnchorA = Add(target.AnchorB, offsetA)
			target.Separation = source.Separation - restOffset
			target.FeatureId = makeFeatureId(source.Pair)
			target.TriangleIndex = source.TriangleIndex

			// Preserve normal impulse if possible
			if matchedManifold != nil {
				oldPointCount := int(matchedManifold.PointCount)
				for k := 0; k < oldPointCount; k++ {
					oldPt := &matchedManifold.Points[k]

					if target.FeatureId == oldPt.FeatureId && target.TriangleIndex == oldPt.TriangleIndex {
						target.NormalImpulse = oldPt.NormalImpulse
						target.Persisted = true

						// claimed
						oldPt.TriangleIndex = NullIndex
						break
					}
				}
			}
		}
	}

	materialsA := getShapeMaterials(shapeA)
	materialB := &getShapeMaterials(shapeB)[0]
	tangentVelocityA := Vec3Zero

	// Update friction and restitution if the mesh has per triangle material
	if shapeA.materialCount > 0 {
		friction := float32(0.0)
		restitution := float32(0.0)
		sampleCount := float32(0.0)

		var materialIndices []uint8
		if shapeA.typ == MeshShape {
			materialIndices = GetMeshMaterialIndices(shapeA.mesh.Data)
		} else {
			materialIndices = GetHeightFieldMaterialIndices(shapeA.heightField)
		}

		for i := 0; i < clusterCount; i++ {
			manifold := &contact.manifolds[i]
			pointCount := int(manifold.PointCount)
			for j := 0; j < pointCount; j++ {
				triangleIndex := int(manifold.Points[j].TriangleIndex)
				var materialIndex int
				if shapeA.typ == MeshShape {
					materialIndex = int(materialIndices[triangleIndex])

					if materialMap != nil {
						materialIndex = int(materialMap[materialIndex])
					}
				} else {
					materialIndex = int(materialIndices[triangleIndex>>1])
				}

				materialIndex = ClampInt(materialIndex, 0, int(shapeA.materialCount)-1)
				material := materialsA[materialIndex]
				friction += world.frictionCallback(material.Friction, material.UserMaterialId, materialB.Friction,
					materialB.UserMaterialId)
				restitution += world.restitutionCallback(material.Restitution, material.UserMaterialId, materialB.Restitution,
					materialB.UserMaterialId)

				tangentVelocityA = Add(tangentVelocityA, material.TangentVelocity)

				sampleCount += 1.0
			}
		}

		if sampleCount > 0.0 {
			invCount := 1.0 / sampleCount
			contact.friction = float32(invCount * friction)
			contact.restitution = float32(invCount * restitution)
			tangentVelocityA = MulSV(invCount, tangentVelocityA)
		}

		assert(IsValidFloat(contact.friction) && contact.friction >= 0.0, "b3IsValidFloat( contact->friction ) && contact->friction >= 0.0f")
		assert(IsValidFloat(contact.restitution) && contact.restitution >= 0.0, "b3IsValidFloat( contact->restitution ) && contact->restitution >= 0.0f")
	} else {
		// Keep these updated in case the values on the shapes are modified
		contact.friction = world.frictionCallback(materialsA[0].Friction, materialsA[0].UserMaterialId, materialB.Friction,
			materialB.UserMaterialId)
		contact.restitution = world.restitutionCallback(materialsA[0].Restitution, materialsA[0].UserMaterialId,
			materialB.Restitution, materialB.UserMaterialId)
		tangentVelocityA = materialsA[0].TangentVelocity
	}

	tangentVelocityA = RotateVector(xfA.Q, tangentVelocityA)

	radiusB := float32(0.0)
	if shapeB.typ == SphereShape {
		radiusB = shapeB.sphere.Radius
	} else if shapeB.typ == CapsuleShape {
		radiusB = shapeB.capsule.Radius
	} else if shapeB.typ == HullShape {
		radiusB = shapeB.hull.InnerRadius
	}

	contact.rollingResistance = float32(materialB.RollingResistance * radiusB)

	tangentVelocityB := RotateVector(xfB.Q, materialB.TangentVelocity)
	contact.tangentVelocity = Sub(tangentVelocityA, tangentVelocityB)
	return true
}
