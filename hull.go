// Port of src/hull.h and src/hull.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.
//
// Dirk Gregorius contributed portions of this code

package box3d

import (
	"bytes"
	"unsafe"
)

// ---- hull.h ----

// hullSoaPadding is B3_HULL_SOA_PADDING. Big enough to handle AVX2.
const hullSoaPadding = 8

// getHullSoaStride is b3GetHullSoaStride.
func getHullSoaStride(count int) int {
	return (count + hullSoaPadding - 1) &^ (hullSoaPadding - 1)
}

// hullMap is b3HullMap: map keyed by hull content. The world hull database uses the value as a
// reference count, while compound baking stores uses the value as a byte offset. Implementation
// is in hull.c.
// PORT: a verstable instance; hullMapInit replaces b3HullMap_init (it binds HASH_FN/CMPR_FN).
type hullMap = verstable[*HullData, int]

// hullMapInit is b3HullMap_init.
func hullMapInit(m *hullMap) {
	m.init(hashHullData, compareHullData)
}

// ---- hull.c ----

const (
	axisX = 0 // B3_AXIS_X
	axisY = 1 // B3_AXIS_Y
	axisZ = 2 // B3_AXIS_Z

	markVisible = 0 // B3_MARK_VISIBLE
	markDelete  = 1 // B3_MARK_DELETE
)

// hullMaxCount is B3_HULL_MAX_COUNT.
// Final topology uses uint16 indices, bounded by the native hull limits.
const hullMaxCount = 2*MaxHullEdges + 1

// PORT: the quickhull builder keeps upstream's intrusive lists. A list node is the first field
// of qhVertex and qhFace, so a node pointer is converted back to its owner with unsafe exactly
// like the C casts ((b3QHVertex*)node). All builder objects live in typed slices owned by the
// builder (see hullBuilderInit), so these pointers are ordinary Go pointers.

type qhListNode struct {
	prev *qhListNode
	next *qhListNode
}

type qhVertex struct {
	// Intrusive list link. Must be first so (b3QHVertex*)nodePtr is valid.
	link qhListNode

	conflictFace *qhFace
	position     Vec3

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex int
	reachable  bool
}

type qhHalfEdge struct {
	// Edge ring (CCW) around the owning face. Not an external list.
	prev *qhHalfEdge
	next *qhHalfEdge

	origin *qhVertex
	face   *qhFace
	twin   *qhHalfEdge

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex int
}

type qhFace struct {
	// Intrusive list link. Must be first so (b3QHFace*)nodePtr is valid.
	link qhListNode

	edge *qhHalfEdge

	mark                int
	area                float32
	plane               Plane
	centroid            Vec3
	maxConflictDistance float32

	// Sentinel head for this face's conflict list of b3QHVertex.
	conflictListHead qhVertex

	// Cached farthest conflict vertex (above b3HullBuilder::minOutside).
	// NULL when no conflict above threshold; maxConflictDistance is then minOutside.
	maxConflict *qhVertex

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex int
	flipped    bool
}

// vertexOfNode is the (b3QHVertex*)node cast.
func vertexOfNode(node *qhListNode) *qhVertex {
	return (*qhVertex)(unsafe.Pointer(node))
}

// faceOfNode is the (b3QHFace*)node cast.
func faceOfNode(node *qhListNode) *qhFace {
	return (*qhFace)(unsafe.Pointer(node))
}

// nodeOfFace is the (b3QHListNode*)face cast. A nil face gives a nil node.
func nodeOfFace(face *qhFace) *qhListNode {
	return (*qhListNode)(unsafe.Pointer(face))
}

// horizonFrame is b3HorizonFrame: one frame of the iterative horizon DFS. Replaces a recursive
// call to b3HullBuilder_BuildHorizon.
type horizonFrame struct {
	face      *qhFace
	startEdge *qhHalfEdge // ring termination sentinel
	edge      *qhHalfEdge // next edge to process
	started   bool        // false until the first edge of this ring has been processed
}

// hullBuilder is b3HullBuilder: all working memory for one hull build, carved from a single b3Alloc block.
type hullBuilder struct {
	tolerance  float32
	minRadius  float32
	minOutside float32

	interiorPoint Vec3

	// List sentinels. Only the link is meaningful; other fields are unused.
	orphanedList qhVertex
	vertexList   qhVertex
	faceList     qhFace

	// Bump-allocated pools with pointer-based free lists for faces and edges.
	vertexBase     []qhVertex
	vertexCapacity int
	vertexCount    int

	edgeBase     []qhHalfEdge
	edgeCapacity int
	edgeCount    int

	// LIFO free list. Built with edge->next.
	edgeFreeHead *qhHalfEdge

	faceBase     []qhFace
	faceCapacity int
	faceCount    int

	// LIFO free list. Built with face->link.next.
	faceFreeHead *qhFace

	// Reusable scratch buffers.
	horizon         []*qhHalfEdge
	horizonCapacity int
	horizonCount    int

	cone         []*qhFace
	coneCapacity int
	coneCount    int

	mergedFaces         []*qhFace
	mergedFacesCapacity int
	mergedFacesCount    int

	// DFS stack used by the iterative b3HullBuilder_BuildHorizon. Depth bounded by live faces.
	horizonStack         []horizonFrame
	horizonStackCapacity int

	// Final counts of the constructed hull (vertexList / faceList / half-edges around faces).
	finalVertexCount   int
	finalHalfEdgeCount int
	finalFaceCount     int
}

func qhListInit(head *qhListNode) {
	head.prev = head
	head.next = head
}

// qhListEmpty is B3_LIST_EMPTY.
func qhListEmpty(a *qhListNode) bool {
	return a.next == a
}

func qhListContains(node *qhListNode) bool {
	return node.prev != nil && node.next != nil
}

// qhListInsert inserts node before `where`.
func qhListInsert(node, where *qhListNode) {
	assert(!qhListContains(node) && qhListContains(where), "!b3QHList_Contains( node ) && b3QHList_Contains( where )")

	node.prev = where.prev
	node.next = where

	node.prev.next = node
	node.next.prev = node
}

func qhListRemove(node *qhListNode) {
	assert(qhListContains(node), "b3QHList_Contains( node )")

	node.prev.next = node.next
	node.next.prev = node.prev

	node.prev = nil
	node.next = nil
}

func qhListPushBack(head, node *qhListNode) {
	qhListInsert(node, head.prev)
}

func (b *hullBuilder) newVertex(position Vec3) *qhVertex {
	assert(b.vertexCount < b.vertexCapacity, "b->vertexCount < b->vertexCapacity")
	vertex := &b.vertexBase[b.vertexCount]
	b.vertexCount++

	vertex.link.prev = nil
	vertex.link.next = nil
	vertex.conflictFace = nil
	vertex.position = position
	vertex.finalIndex = NullIndex
	vertex.reachable = false

	return vertex
}

func (b *hullBuilder) newEdge() *qhHalfEdge {
	var edge *qhHalfEdge
	if b.edgeFreeHead != nil {
		edge = b.edgeFreeHead
		b.edgeFreeHead = edge.next
	} else {
		assert(b.edgeCount < b.edgeCapacity, "b->edgeCount < b->edgeCapacity")
		edge = &b.edgeBase[b.edgeCount]
		b.edgeCount++
	}
	// All other fields (prev/next/origin/face/twin) are written by NewFace immediately after.
	edge.finalIndex = NullIndex
	return edge
}

func (b *hullBuilder) retireEdge(edge *qhHalfEdge) {
	edge.next = b.edgeFreeHead
	b.edgeFreeHead = edge
}

func (b *hullBuilder) newFace(v1, v2, v3 *qhVertex) *qhFace {
	var face *qhFace
	if b.faceFreeHead != nil {
		face = b.faceFreeHead
		// link.next was used as free-list pointer; recover next head before we clobber.
		b.faceFreeHead = faceOfNode(face.link.next)
	} else {
		assert(b.faceCount < b.faceCapacity, "b->faceCount < b->faceCapacity")
		face = &b.faceBase[b.faceCount]
		b.faceCount++
	}

	// link.prev: NULL on retired faces (cleared by Remove); NULL here so PushBack's
	// !b3QHList_Contains assert holds for fresh bump slots too.
	// link.next: was the free-list pointer on reused slots, now stale; PushBack overwrites.
	face.link.prev = nil
	face.link.next = nil
	face.maxConflict = nil
	face.maxConflictDistance = 0.0
	face.finalIndex = NullIndex

	edge1 := b.newEdge()
	edge2 := b.newEdge()
	edge3 := b.newEdge()

	p1 := v1.position
	p2 := v2.position
	p3 := v3.position

	var plane Plane
	plane.Normal = Cross(Sub(p2, p1), Sub(p3, p1))
	var length float32
	plane.Normal, length = GetLengthAndNormalize(plane.Normal)
	plane.Offset = Dot(plane.Normal, p1)

	area := float32(0.5 * length)

	face.edge = edge1
	face.mark = markVisible
	face.area = area
	face.centroid = MulSV(1.0/3.0, Add(v1.position, Add(v2.position, v3.position)))
	face.plane = plane
	face.flipped = planeSeparation(plane, b.interiorPoint) > 0.0
	qhListInit(&face.conflictListHead.link)

	edge1.prev = edge3
	edge1.next = edge2
	edge1.origin = v1
	edge1.face = face
	edge1.twin = nil

	edge2.prev = edge1
	edge2.next = edge3
	edge2.origin = v2
	edge2.face = face
	edge2.twin = nil

	edge3.prev = edge2
	edge3.next = edge1
	edge3.origin = v3
	edge3.face = face
	edge3.twin = nil

	return face
}

// retireFace removes face and add to free list.
func (b *hullBuilder) retireFace(face *qhFace) {
	// Sometimes a cone face gets merged and never added to the list.
	if qhListContains(&face.link) {
		qhListRemove(&face.link)
	}

	face.edge = nil
	// link.prev is already NULL after Remove (or was never set).
	validate(face.link.prev == nil, "face->link.prev == NULL")
	face.link.next = nodeOfFace(b.faceFreeHead)
	b.faceFreeHead = face
}

func buildBounds(vertexCount int, vertices []Vec3) AABB {
	bounds := bounds3Empty
	for i := range vertexCount {
		bounds.LowerBound = Min(bounds.LowerBound, vertices[i])
		bounds.UpperBound = Max(bounds.UpperBound, vertices[i])
	}
	return bounds
}

// findFarthestPointsAlongCardinalAxes is b3FindFarthestPointsAlongCardinalAxes.
// PORT: the index out-parameters become return values.
func findFarthestPointsAlongCardinalAxes(tolerance float32, vertexCount int, vertexBase []Vec3) (index1Out, index2Out int) {
	index1Out = NullIndex
	index2Out = NullIndex

	v0 := vertexBase[0]
	minPt := [3]Vec3{v0, v0, v0}
	maxPt := [3]Vec3{v0, v0, v0}

	minIndex := [3]int{0, 0, 0}
	maxIndex := [3]int{0, 0, 0}

	for i := 1; i < vertexCount; i++ {
		v := vertexBase[i]

		if v.X < minPt[axisX].X {
			minPt[axisX] = v
			minIndex[axisX] = i
		} else if v.X > maxPt[axisX].X {
			maxPt[axisX] = v
			maxIndex[axisX] = i
		}

		if v.Y < minPt[axisY].Y {
			minPt[axisY] = v
			minIndex[axisY] = i
		} else if v.Y > maxPt[axisY].Y {
			maxPt[axisY] = v
			maxIndex[axisY] = i
		}

		if v.Z < minPt[axisZ].Z {
			minPt[axisZ] = v
			minIndex[axisZ] = i
		} else if v.Z > maxPt[axisZ].Z {
			maxPt[axisZ] = v
			maxIndex[axisZ] = i
		}
	}

	var distance Vec3
	distance.X = maxPt[axisX].X - minPt[axisX].X
	distance.Y = maxPt[axisY].Y - minPt[axisY].Y
	distance.Z = maxPt[axisZ].Z - minPt[axisZ].Z

	distanceArray := [3]float32{distance.X, distance.Y, distance.Z}
	maxElement := maxElementIndex(distance)

	if distanceArray[maxElement] > float32(2.0*tolerance) {
		index1Out = minIndex[maxElement]
		index2Out = maxIndex[maxElement]
	}

	return index1Out, index2Out
}

func findFarthestPointFromLine(index1, index2 int, tolerance float32, vertexCount int, vertexBase []Vec3) int {
	a := vertexBase[index1]
	b := vertexBase[index2]

	// |ap x ab|^2 / |ab|^2 is the squared perpendicular distance from p to the line.
	// Compares against (2 * tolerance)^2
	ab := Sub(b, a)
	abLengthSqr := Dot(ab, ab)
	assert(abLengthSqr > 0.0, "abLengthSqr > 0.0f")

	invAbLengthSqr := 1.0 / abLengthSqr
	maxDistanceSqr := float32(4.0 * tolerance * tolerance)
	maxIndex := NullIndex

	for i := range vertexCount {
		if i == index1 || i == index2 {
			continue
		}

		ap := Sub(vertexBase[i], a)
		cross := Cross(ap, ab)
		distanceSqr := float32(Dot(cross, cross) * invAbLengthSqr)
		if distanceSqr > maxDistanceSqr {
			maxDistanceSqr = distanceSqr
			maxIndex = i
		}
	}

	return maxIndex
}

func findFarthestPointFromPlane(index1, index2, index3 int, tolerance float32, vertexCount int, vertexBase []Vec3) int {
	a := vertexBase[index1]
	b := vertexBase[index2]
	c := vertexBase[index3]

	plane := makePlaneFromPoints(a, b, c)

	maxDistance := float32(2.0 * tolerance)
	maxIndex := NullIndex

	for i := range vertexCount {
		if i == index1 || i == index2 || i == index3 {
			continue
		}

		distance := AbsFloat(planeSeparation(plane, vertexBase[i]))
		if distance > maxDistance {
			maxDistance = distance
			maxIndex = i
		}
	}

	return maxIndex
}

func isEdgeConvex(edge *qhHalfEdge, tolerance float32) bool {
	distance := planeSeparation(edge.face.plane, edge.twin.face.centroid)
	return distance < -tolerance
}

func isEdgeConcave(edge *qhHalfEdge, tolerance float32) bool {
	distance := planeSeparation(edge.face.plane, edge.twin.face.centroid)
	return distance > tolerance
}

func vertexCountOfFace(face *qhFace) int {
	count := 0
	edge := face.edge
	for {
		count++
		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	return count
}

func linkFace(face *qhFace, index int, twin *qhHalfEdge) {
	assert(face != twin.face, "face != twin->face")

	edge := face.edge
	for ; index > 0; index-- {
		assert(edge.face == face, "edge->face == face")
		edge = edge.next
	}

	assert(edge != twin, "edge != twin")
	edge.twin = twin
	twin.twin = edge
}

func linkFaces(face1 *qhFace, index1 int, face2 *qhFace, index2 int) {
	assert(face1 != face2, "face1 != face2")

	edge1 := face1.edge
	for ; index1 > 0; index1-- {
		edge1 = edge1.next
	}

	edge2 := face2.edge
	for ; index2 > 0; index2-- {
		edge2 = edge2.next
	}

	assert(edge1 != edge2, "edge1 != edge2")
	edge1.twin = edge2
	edge2.twin = edge1
}

func newellPlane(face *qhFace) {
	count := 0
	centroid := Vec3Zero
	normal := Vec3Zero

	edge := face.edge
	assert(edge.face == face, "edge->face == face")

	// Use the first vertex as the origin to reduce round-off
	origin := edge.origin.position

	for {
		twin := edge.twin
		assert(twin.twin == edge, "twin->twin == edge")

		v1 := Sub(edge.origin.position, origin)
		v2 := Sub(twin.origin.position, origin)

		count++
		centroid = Add(centroid, v1)
		normal.X += float32((v1.Y - v2.Y) * (v1.Z + v2.Z))
		normal.Y += float32((v1.Z - v2.Z) * (v1.X + v2.X))
		normal.Z += float32((v1.X - v2.X) * (v1.Y + v2.Y))

		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	assert(count > 0, "count > 0")
	centroid = MulSV(1.0/float32(count), centroid)
	centroid = Add(centroid, origin)

	length := Length(normal)
	validate(length > 0.0, "length > 0.0f")
	normal = MulSV(1.0/length, normal)

	face.centroid = centroid
	face.plane = makePlaneFromNormalAndPoint(normal, centroid)
	face.area = float32(0.5 * length)
}

// checkConsistency is b3CheckConsistency. Only called when debugBuild (B3_DEBUG).
func checkConsistency(face *qhFace) bool {
	if face.mark == markDelete {
		return false
	}

	if vertexCountOfFace(face) < 3 {
		return false
	}

	edge := face.edge

	for {
		twin := edge.twin

		if twin == nil {
			return false
		}
		if twin.face == nil {
			return false
		}
		if twin.face == face {
			return false
		}
		if twin.face.mark == markDelete {
			return false
		}
		if twin.twin != edge {
			return false
		}
		if edge.next.origin != twin.origin {
			return false
		}
		if edge.origin != twin.next.origin {
			return false
		}
		if edge.face != face {
			return false
		}

		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	return true
}

func (b *hullBuilder) computeTolerance(pointCount int, points []Vec3) {
	bounds := buildBounds(pointCount, points)
	maxAbs := Max(Abs(bounds.LowerBound), Abs(bounds.UpperBound))

	maxSum := maxAbs.X + maxAbs.Y + maxAbs.Z
	maxCoord := MaxFloat(maxAbs.X, MaxFloat(maxAbs.Y, maxAbs.Z))
	maxDistance := MinFloat(float32(sqrt3*maxCoord), maxSum)

	tolerance := float32((float32(3.0*maxDistance*1.01) + maxCoord) * fltEpsilon)

	b.tolerance = tolerance
	b.minRadius = float32(4.0 * b.tolerance)
	b.minOutside = float32(2.0 * b.minRadius)
	assert(b.minRadius < b.minOutside+3.0*fltEpsilon, "b->minRadius < b->minOutside + 3.0f * FLT_EPSILON")
}

func (b *hullBuilder) buildInitialHull(pointCount int, points []Vec3) bool {
	index1, index2 := findFarthestPointsAlongCardinalAxes(b.tolerance, pointCount, points)
	if index1 < 0 || index2 < 0 {
		return false
	}

	index3 := findFarthestPointFromLine(index1, index2, b.tolerance, pointCount, points)
	if index3 < 0 {
		return false
	}

	index4 := findFarthestPointFromPlane(index1, index2, index3, b.tolerance, pointCount, points)
	if index4 < 0 {
		return false
	}

	v1 := Sub(points[index1], points[index4])
	v2 := Sub(points[index2], points[index4])
	v3 := Sub(points[index3], points[index4])

	if scalarTripleProduct(v1, v2, v3) < 0.0 {
		temp := index2
		index2 = index3
		index3 = temp
	}

	b.interiorPoint = Vec3Zero
	b.interiorPoint = Add(b.interiorPoint, points[index1])
	b.interiorPoint = Add(b.interiorPoint, points[index2])
	b.interiorPoint = Add(b.interiorPoint, points[index3])
	b.interiorPoint = Add(b.interiorPoint, points[index4])
	b.interiorPoint = MulSV(0.25, b.interiorPoint)

	vertex1 := b.newVertex(points[index1])
	qhListPushBack(&b.vertexList.link, &vertex1.link)
	vertex2 := b.newVertex(points[index2])
	qhListPushBack(&b.vertexList.link, &vertex2.link)
	vertex3 := b.newVertex(points[index3])
	qhListPushBack(&b.vertexList.link, &vertex3.link)
	vertex4 := b.newVertex(points[index4])
	qhListPushBack(&b.vertexList.link, &vertex4.link)

	face1 := b.newFace(vertex1, vertex2, vertex3)
	qhListPushBack(&b.faceList.link, &face1.link)
	face2 := b.newFace(vertex4, vertex2, vertex1)
	qhListPushBack(&b.faceList.link, &face2.link)
	face3 := b.newFace(vertex4, vertex3, vertex2)
	qhListPushBack(&b.faceList.link, &face3.link)
	face4 := b.newFace(vertex4, vertex1, vertex3)
	qhListPushBack(&b.faceList.link, &face4.link)

	linkFaces(face1, 0, face2, 1)
	linkFaces(face1, 1, face3, 1)
	linkFaces(face1, 2, face4, 1)

	linkFaces(face2, 0, face3, 2)
	linkFaces(face3, 0, face4, 2)
	linkFaces(face4, 0, face2, 2)

	if debugBuild {
		assert(checkConsistency(face1), "b3CheckConsistency( face1 )")
		assert(checkConsistency(face2), "b3CheckConsistency( face2 )")
		assert(checkConsistency(face3), "b3CheckConsistency( face3 )")
		assert(checkConsistency(face4), "b3CheckConsistency( face4 )")
	}

	for index := range pointCount {
		if index == index1 || index == index2 || index == index3 || index == index4 {
			continue
		}

		point := points[index]

		maxDistance := b.minOutside
		var maxFace *qhFace

		for node := b.faceList.link.next; node != &b.faceList.link; node = node.next {
			face := faceOfNode(node)
			distance := planeSeparation(face.plane, point)
			if distance > maxDistance {
				maxDistance = distance
				maxFace = face
			}
		}

		if maxFace != nil {
			vertex := b.newVertex(point)
			vertex.conflictFace = maxFace
			qhListPushBack(&maxFace.conflictListHead.link, &vertex.link)
			if maxDistance > maxFace.maxConflictDistance {
				maxFace.maxConflictDistance = maxDistance
				maxFace.maxConflict = vertex
			}
		}
	}

	return true
}

// recacheConflicts recomputes the farthest-conflict cache after a face's plane changes.
// Walks the existing conflict list once; cost is bounded by that list, not the global pool.
func recacheConflicts(face *qhFace, minOutside float32) {
	var maxVertex *qhVertex
	maxDistance := minOutside

	for node := face.conflictListHead.link.next; node != &face.conflictListHead.link; node = node.next {
		vertex := vertexOfNode(node)
		distance := planeSeparation(face.plane, vertex.position)
		if distance > maxDistance {
			maxDistance = distance
			maxVertex = vertex
		}
	}

	face.maxConflict = maxVertex
	face.maxConflictDistance = maxDistance
}

func (b *hullBuilder) nextConflictVertex() *qhVertex {
	var maxVertex *qhVertex
	maxDistance := b.minOutside

	for faceNode := b.faceList.link.next; faceNode != &b.faceList.link; faceNode = faceNode.next {
		face := faceOfNode(faceNode)
		if face.maxConflict != nil && face.maxConflictDistance > maxDistance {
			maxDistance = face.maxConflictDistance
			maxVertex = face.maxConflict
		}
	}

	return maxVertex
}

// drainConflictList moves every conflict vertex of `face` onto the orphaned list and clear their conflictFace.
func (b *hullBuilder) drainConflictList(face *qhFace) {
	node := face.conflictListHead.link.next
	for node != &face.conflictListHead.link {
		orphan := vertexOfNode(node)
		node = node.next

		orphan.conflictFace = nil
		qhListRemove(&orphan.link)
		qhListPushBack(&b.orphanedList.link, &orphan.link)
	}
	assert(qhListEmpty(&face.conflictListHead.link), "B3_LIST_EMPTY( &face->conflictListHead.link )")
}

// enterHorizonFace marks a face for deletion, drain its conflict list, and populate a fresh DFS frame for it.
// `entryEdge` is the half-edge in `face` whose twin lies in the just-deleted parent face, or
// NULL for the seed. The frame skips `entryEdge` on recursive entries (it would be ignored
// anyway since the parent's mark is now DELETE, but skipping saves one iteration).
func (b *hullBuilder) enterHorizonFace(face *qhFace, entryEdge *qhHalfEdge, frameOut *horizonFrame) {
	face.mark = markDelete
	b.drainConflictList(face)

	frameOut.face = face
	frameOut.started = false
	if entryEdge != nil {
		frameOut.startEdge = entryEdge
		frameOut.edge = entryEdge.next
	} else {
		frameOut.startEdge = face.edge
		frameOut.edge = face.edge
	}
}

func (b *hullBuilder) buildHorizon(apex *qhVertex, seed *qhFace) {
	stack := b.horizonStack
	top := 0

	assert(top < b.horizonStackCapacity, "top < b->horizonStackCapacity")
	b.enterHorizonFace(seed, nil, &stack[top])
	top++

	for top > 0 {
		f := &stack[top-1]

		if f.started && f.edge == f.startEdge {
			top--
			continue
		}
		f.started = true

		edge := f.edge
		twin := edge.twin
		f.edge = edge.next

		if twin.face.mark != markVisible {
			continue
		}

		distance := planeSeparation(twin.face.plane, apex.position)
		if distance > b.minRadius {
			assert(top < b.horizonStackCapacity, "top < b->horizonStackCapacity")
			b.enterHorizonFace(twin.face, twin, &stack[top])
			top++
		} else {
			assert(b.horizonCount < b.horizonCapacity, "b->horizonCount < b->horizonCapacity")
			b.horizon[b.horizonCount] = edge
			b.horizonCount++
		}
	}
}

func (b *hullBuilder) buildCone(apex *qhVertex) {
	for i := range b.horizonCount {
		edge := b.horizon[i]
		assert(edge.twin.twin == edge, "edge->twin->twin == edge")

		face := b.newFace(apex, edge.origin, edge.twin.origin)
		assert(b.coneCount < b.coneCapacity, "b->coneCount < b->coneCapacity")
		b.cone[b.coneCount] = face
		b.coneCount++

		linkFace(face, 1, edge.twin)
	}

	face1 := b.cone[b.coneCount-1]
	for i := range b.coneCount {
		face2 := b.cone[i]
		linkFaces(face1, 2, face2, 0)
		face1 = face2
	}
}

// destroyEdges retires half-edges in the half-open ring range [begin, end) by pushing each onto edgeFreeHead.
// The caller has already detached these edges from the live face ring (rewire happens before
// destroy), so they are unreachable from the live hull.
func (b *hullBuilder) destroyEdges(begin, end *qhHalfEdge) {
	edge := begin
	for edge != end {
		next := edge.next
		b.retireEdge(edge)
		edge = next
	}
}

func (b *hullBuilder) connectEdges(prev, next *qhHalfEdge) {
	assert(prev != next, "prev != next")
	assert(prev.face == next.face, "prev->face == next->face")

	// If both shared neighbors are the same face, prev and next together would orphan that face.
	if prev.twin.face == next.twin.face {
		// next is redundant.
		if next.face.edge == next {
			next.face.edge = prev
		}

		var twin *qhHalfEdge
		if vertexCountOfFace(prev.twin.face) == 3 {
			// Capture all 3 half-edges of the dead triangle before the rewire overwrites prev->twin.
			deadEdge0 := prev.twin      // prev->twin (will be rewired below)
			deadEdge1 := next.twin      // next->twin
			deadEdge2 := next.twin.prev // third edge of the dead triangle

			twin = deadEdge2.twin
			assert(twin.face.mark != markDelete, "twin->face->mark != B3_MARK_DELETE")

			opposingFace := prev.twin.face
			opposingFace.mark = markDelete
			assert(b.mergedFacesCount < b.mergedFacesCapacity, "b->mergedFacesCount < b->mergedFacesCapacity")
			b.mergedFaces[b.mergedFacesCount] = opposingFace
			b.mergedFacesCount++

			prev.next = next.next
			prev.next.prev = prev

			prev.twin = twin
			twin.twin = prev

			// Drop the redundant vertex (slot abandoned in the bump allocator).
			qhListRemove(&next.origin.link)

			// Retire the 3 half-edges of the dead triangle now that the rewire is complete.
			b.retireEdge(deadEdge0)
			b.retireEdge(deadEdge1)
			b.retireEdge(deadEdge2)
		} else {
			twin = next.twin

			if twin.face.edge == prev.twin {
				twin.face.edge = twin
			}

			twin.next = prev.twin.next
			twin.next.prev = twin
			// prev->twin slot is retired to the edge free list.
			b.retireEdge(prev.twin)

			prev.next = next.next
			prev.next.prev = prev

			prev.twin = twin
			twin.twin = prev

			// Drop the redundant vertex (slot abandoned in the bump allocator).
			qhListRemove(&next.origin.link)
		}

		// Twin->face changed shape; recompute its plane and refresh its cached max conflict.
		newellPlane(twin.face)
		recacheConflicts(twin.face, b.minOutside)
	} else {
		prev.next = next
		next.prev = prev
	}
}

func (b *hullBuilder) absorbFaces(face *qhFace) {
	for i := range b.mergedFacesCount {
		assert(b.mergedFaces[i].mark == markDelete, "b->mergedFaces[i]->mark == B3_MARK_DELETE")
		head := &b.mergedFaces[i].conflictListHead.link

		node := head.next
		for node != head {
			vertex := vertexOfNode(node)
			node = node.next

			qhListRemove(&vertex.link)

			distance := planeSeparation(face.plane, vertex.position)
			if distance > b.minOutside {
				qhListPushBack(&face.conflictListHead.link, &vertex.link)
				vertex.conflictFace = face
				if distance > face.maxConflictDistance {
					face.maxConflictDistance = distance
					face.maxConflict = vertex
				}
			} else {
				qhListPushBack(&b.orphanedList.link, &vertex.link)
				vertex.conflictFace = nil
			}
		}

		assert(qhListEmpty(head), "B3_LIST_EMPTY( head )")

		// Conflict list is now drained. Retire this face to the free list.
		b.retireFace(b.mergedFaces[i])
	}
}

func (b *hullBuilder) connectFaces(edge *qhHalfEdge) {
	face := edge.face

	twin := edge.twin

	edgePrev := edge.prev
	edgeNext := edge.next
	twinPrev := twin.prev
	twinNext := twin.next

	for edgePrev.twin.face == twin.face {
		assert(edgePrev.twin == twinNext, "edgePrev->twin == twinNext")
		assert(twinNext.twin == edgePrev, "twinNext->twin == edgePrev")

		edgePrev = edgePrev.prev
		twinNext = twinNext.next
	}
	assert(edgePrev.face != twinNext.face, "edgePrev->face != twinNext->face")

	for edgeNext.twin.face == twin.face {
		assert(edgeNext.twin == twinPrev, "edgeNext->twin == twinPrev")
		assert(twinPrev.twin == edgeNext, "twinPrev->twin == edgeNext")

		edgeNext = edgeNext.next
		twinPrev = twinPrev.prev
	}
	assert(edgeNext.face != twinPrev.face, "edgeNext->face != twinPrev->face")

	face.edge = edgePrev

	// Discard opposing face. mergedFaces is single-buffered: ConnectFaces does not nest.
	b.mergedFacesCount = 0
	assert(b.mergedFacesCount < b.mergedFacesCapacity, "b->mergedFacesCount < b->mergedFacesCapacity")
	b.mergedFaces[b.mergedFacesCount] = twin.face
	b.mergedFacesCount++
	twin.face.mark = markDelete
	twin.face.edge = nil

	for absorbed := twinNext; absorbed != twinPrev.next; absorbed = absorbed.next {
		absorbed.face = face
	}

	b.destroyEdges(edgePrev.next, edgeNext)
	b.destroyEdges(twinPrev.next, twinNext)

	b.connectEdges(edgePrev, twinNext)
	b.connectEdges(twinPrev, edgeNext)

	newellPlane(face)
	// Existing conflicts now have stale distances under the new plane; AbsorbFaces will then
	// add more incrementally and update the cache as it goes.
	recacheConflicts(face, b.minOutside)
	if debugBuild {
		assert(checkConsistency(face), "b3CheckConsistency( face )")
	}

	b.absorbFaces(face)
}

func (b *hullBuilder) mergeConcave(face *qhFace) bool {
	edge := face.edge

	for {
		twin := edge.twin

		if isEdgeConcave(edge, b.minRadius) || isEdgeConcave(twin, b.minRadius) {
			b.connectFaces(edge)
			return true
		}

		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	return false
}

func (b *hullBuilder) mergeCoplanar(face *qhFace) bool {
	edge := face.edge

	for {
		twin := edge.twin

		if !isEdgeConvex(edge, b.minRadius) || !isEdgeConvex(twin, b.minRadius) {
			b.connectFaces(edge)
			return true
		}

		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	return false
}

func (b *hullBuilder) mergeFaces() {
	for i := range b.coneCount {
		face := b.cone[i]
		if face.mark == markVisible && face.flipped {
			face.flipped = false

			var bestArea float32 = 0
			var bestEdge *qhHalfEdge

			edge := face.edge
			for {
				twin := edge.twin
				area := twin.face.area
				if area > bestArea {
					bestArea = area
					bestEdge = edge
				}

				edge = edge.next
				if edge == face.edge {
					break
				}
			}

			assert(bestEdge != nil, "bestEdge != NULL")
			b.connectFaces(bestEdge)
		}
	}

	for i := range b.coneCount {
		face := b.cone[i]
		if face.mark == markVisible {
			for b.mergeConcave(face) {
			}
		}
	}

	for i := range b.coneCount {
		face := b.cone[i]
		if face.mark == markVisible {
			for b.mergeCoplanar(face) {
			}
		}
	}
}

func (b *hullBuilder) resolveVertices() {
	node := b.orphanedList.link.next
	for node != &b.orphanedList.link {
		vertex := vertexOfNode(node)
		node = node.next
		qhListRemove(&vertex.link)

		maxDistance := b.minOutside
		var maxFace *qhFace

		for i := range b.coneCount {
			if b.cone[i].mark == markVisible {
				distance := planeSeparation(b.cone[i].plane, vertex.position)
				if distance > maxDistance {
					maxDistance = distance
					maxFace = b.cone[i]
				}
			}
		}

		if maxFace != nil {
			assert(maxFace.mark == markVisible, "maxFace->mark == B3_MARK_VISIBLE")
			qhListPushBack(&maxFace.conflictListHead.link, &vertex.link)
			vertex.conflictFace = maxFace
			if maxDistance > maxFace.maxConflictDistance {
				maxFace.maxConflictDistance = maxDistance
				maxFace.maxConflict = vertex
			}
		}
		// Otherwise: vertex is interior to the hull. Its slot in the bump pool is abandoned.
	}

	assert(qhListEmpty(&b.orphanedList.link), "B3_LIST_EMPTY( &b->orphanedList.link )")
}

func (b *hullBuilder) resolveFaces() {
	// Splice deleted faces out of the face list. Faces already retired by AbsorbFaces are no
	// longer on faceList, so guard with b3QHList_Contains before removing.
	node := b.faceList.link.next
	for node != &b.faceList.link {
		face := faceOfNode(node)
		node = node.next

		if face.mark == markDelete && qhListContains(&face.link) {
			assert(qhListEmpty(&face.conflictListHead.link), "B3_LIST_EMPTY( &face->conflictListHead.link )")

			// Each half-edge is owned by exactly one face, so ring walks over the
			// dead region retire every interior edge exactly once. Merge deleted
			// faces are already off the face list.
			start := face.edge
			edge := start
			for {
				next := edge.next
				b.retireEdge(edge)
				edge = next
				if edge == start {
					break
				}
			}

			b.retireFace(face)
		}
	}

	for i := range b.coneCount {
		face := b.cone[i]
		if face.mark == markDelete {
			continue
		}
		qhListPushBack(&b.faceList.link, &face.link)
	}
}

func (b *hullBuilder) addVertexToHull(vertex *qhVertex) {
	face := vertex.conflictFace
	vertex.conflictFace = nil
	qhListRemove(&vertex.link)
	qhListPushBack(&b.vertexList.link, &vertex.link)

	b.horizonCount = 0
	b.buildHorizon(vertex, face)
	assert(b.horizonCount >= 3, "b->horizonCount >= 3")

	b.coneCount = 0
	b.buildCone(vertex)
	assert(b.coneCount >= 3, "b->coneCount >= 3")

	b.mergeFaces()
	b.resolveVertices()
	b.resolveFaces()
}

func (b *hullBuilder) cleanHull(origin Vec3) {
	faceCount := 0
	halfEdgeCount := 0

	for faceNode := b.faceList.link.next; faceNode != &b.faceList.link; faceNode = faceNode.next {
		face := faceOfNode(faceNode)
		edge := face.edge

		for {
			edge.origin.reachable = true
			edge = edge.next
			halfEdgeCount++
			if edge == face.edge {
				break
			}
		}

		face.plane.Offset += Dot(face.plane.Normal, origin)
		face.centroid = Add(face.centroid, origin)
		faceCount++
	}

	vertexCount := 0
	node := b.vertexList.link.next
	for node != &b.vertexList.link {
		vertex := vertexOfNode(node)
		node = node.next

		if !vertex.reachable {
			qhListRemove(&vertex.link)
		} else {
			vertex.position = Add(vertex.position, origin)
			vertexCount++
		}
	}

	b.interiorPoint = Add(b.interiorPoint, origin)

	b.finalVertexCount = vertexCount
	b.finalHalfEdgeCount = halfEdgeCount
	b.finalFaceCount = faceCount
}

// isConsistent is b3HullBuilder_IsConsistent. Only called when debugBuild (B3_DEBUG).
func (b *hullBuilder) isConsistent() bool {
	v := b.finalVertexCount
	e := b.finalHalfEdgeCount / 2
	f := b.finalFaceCount

	if v-e+f != 2 {
		return false
	}

	for faceNode := b.faceList.link.next; faceNode != &b.faceList.link; faceNode = faceNode.next {
		face := faceOfNode(faceNode)
		if face.edge.face != face {
			return false
		}

		if !checkConsistency(face) {
			return false
		}

		if planeSeparation(face.plane, b.interiorPoint) > 0 {
			return false
		}

		if face.mark != markVisible {
			return false
		}

		edge := face.edge

		for {
			if edge.next.origin != edge.twin.origin {
				return false
			}
			if edge.prev.next != edge {
				return false
			}
			if edge.next.prev != edge {
				return false
			}
			if edge.twin.twin != edge {
				return false
			}
			if edge.face != face {
				return false
			}
			if DistanceSquared(edge.origin.position, edge.twin.origin.position) < 1000.0*fltMin {
				return false
			}

			edge = edge.next
			if edge == face.edge {
				break
			}
		}
	}

	return true
}

func (b *hullBuilder) hasHull() bool {
	v := b.finalVertexCount
	e := b.finalHalfEdgeCount / 2
	f := b.finalFaceCount
	return v-e+f == 2 && f >= 4
}

// construct builds the entire hull. Returns true iff the result satisfies Euler's identity.
func (b *hullBuilder) construct(points []Vec3, pointCount, maxVertexCount int, origin Vec3, shiftedPoints []Vec3, requireComplete bool) bool {
	if pointCount < 4 {
		return false
	}

	for i := range pointCount {
		shiftedPoints[i] = Sub(points[i], origin)
	}

	b.computeTolerance(pointCount, shiftedPoints)
	haveInitialHull := b.buildInitialHull(pointCount, shiftedPoints)
	if haveInitialHull == false {
		return false
	}

	budget := ClampInt(maxVertexCount-4, 0, hullMaxCount-4)

	vertex := b.nextConflictVertex()
	for vertex != nil && budget > 0 {
		b.addVertexToHull(vertex)
		vertex = b.nextConflictVertex()
		budget -= 1
	}
	if requireComplete && vertex != nil {
		return false
	}

	b.cleanHull(origin)

	if debugBuild {
		assert(b.isConsistent(), "b3HullBuilder_IsConsistent( b )")
	}

	return b.hasHull()
}

// hullWorkSizes is b3HullWorkSizes.
type hullWorkSizes struct {
	// Input point count
	N int
	// Output point limit
	M                    int
	vertexCapacity       int
	edgeCapacity         int
	faceCapacity         int
	horizonCapacity      int
	coneCapacity         int
	mergedFacesCapacity  int
	horizonStackCapacity int
	totalBytes           int

	offsetVertex        int
	offsetEdge          int
	offsetFace          int
	offsetHorizon       int
	offsetCone          int
	offsetMergedFaces   int
	offsetHorizonStack  int
	offsetShiftedPoints int
}

// computeHullWorkSizes is b3ComputeHullWorkSizes.
// PORT: the byte offsets use the Go struct sizes. They only describe the single work block
// upstream allocates; hullBuilderInit allocates typed slices with the same capacities instead.
func computeHullWorkSizes(pointCount, clampedMaxCount int) hullWorkSizes {
	var s hullWorkSizes
	s.N = pointCount
	s.M = clampedMaxCount

	// Vertices: 4 initial hull vertices + at most one per remaining input point. No free list.
	s.vertexCapacity = pointCount + 4

	// Edges and faces use free-list recycling; capacity is proportional to live hull size.
	// edgeCapacity: peak is ~twice live edges plus cone edges. Minimum 48.
	s.edgeCapacity = MaxInt(48, 24*s.M-48)

	// faceCapacity: peak intermediate state live faces (<=2*M-4) plus full cone (<=3*M-6). Minimum 16.
	s.faceCapacity = MaxInt(16, 5*s.M-10)

	// Horizon/cone bounded by current half-edge count. Merged faces by face count.
	s.horizonCapacity = MaxInt(6, 3*s.M-6)
	s.coneCapacity = s.horizonCapacity
	s.mergedFacesCapacity = MaxInt(4, 2*s.M-4)

	// Horizon DFS depth is bounded by the number of live faces (Euler: <=2*M-4).
	s.horizonStackCapacity = MaxInt(4, 2*s.M-4)

	offset := 0

	s.offsetVertex = offset
	offset = alignUp8(offset + s.vertexCapacity*int(unsafe.Sizeof(qhVertex{})))

	s.offsetEdge = offset
	offset = alignUp8(offset + s.edgeCapacity*int(unsafe.Sizeof(qhHalfEdge{})))

	s.offsetFace = offset
	offset = alignUp8(offset + s.faceCapacity*int(unsafe.Sizeof(qhFace{})))

	s.offsetHorizon = offset
	offset = alignUp8(offset + s.horizonCapacity*int(unsafe.Sizeof((*qhHalfEdge)(nil))))

	s.offsetCone = offset
	offset = alignUp8(offset + s.coneCapacity*int(unsafe.Sizeof((*qhFace)(nil))))

	s.offsetMergedFaces = offset
	offset = alignUp8(offset + s.mergedFacesCapacity*int(unsafe.Sizeof((*qhFace)(nil))))

	s.offsetHorizonStack = offset
	offset = alignUp8(offset + s.horizonStackCapacity*int(unsafe.Sizeof(horizonFrame{})))

	s.offsetShiftedPoints = offset
	offset += pointCount * int(unsafe.Sizeof(Vec3{}))

	s.totalBytes = offset
	return s
}

// hullBuilderInit is b3HullBuilder_Init.
// PORT: instead of carving one memory block, each pool and scratch buffer is a typed slice
// with the capacity computed by computeHullWorkSizes. Returns the shifted points buffer that
// upstream carves from the same block.
func hullBuilderInit(b *hullBuilder, s *hullWorkSizes) []Vec3 {
	*b = hullBuilder{}
	qhListInit(&b.orphanedList.link)
	qhListInit(&b.vertexList.link)
	qhListInit(&b.faceList.link)

	b.vertexBase = make([]qhVertex, s.vertexCapacity)
	b.vertexCapacity = s.vertexCapacity

	b.edgeBase = make([]qhHalfEdge, s.edgeCapacity)
	b.edgeCapacity = s.edgeCapacity

	b.faceBase = make([]qhFace, s.faceCapacity)
	b.faceCapacity = s.faceCapacity

	b.horizon = make([]*qhHalfEdge, s.horizonCapacity)
	b.horizonCapacity = s.horizonCapacity

	b.cone = make([]*qhFace, s.coneCapacity)
	b.coneCapacity = s.coneCapacity

	b.mergedFaces = make([]*qhFace, s.mergedFacesCapacity)
	b.mergedFacesCapacity = s.mergedFacesCapacity

	b.horizonStack = make([]horizonFrame, s.horizonStackCapacity)
	b.horizonStackCapacity = s.horizonStackCapacity

	return make([]Vec3, s.N)
}

// The *Write accessors return writable views of the hull image. In Go the read accessors
// already return writable slices; these keep upstream's call sites.

func getHullPointsWrite(hull *HullData) []Vec3 {
	return GetHullPoints(hull)
}

func getHullPlanesWrite(hull *HullData) []Plane {
	return GetHullPlanes(hull)
}

func getHullVerticesWrite(hull *HullData) []HullVertex {
	return GetHullVertices(hull)
}

func getHullEdgesWrite(hull *HullData) []HullHalfEdge {
	return GetHullEdges(hull)
}

func getHullSoaVerticesWrite(hull *HullData) []float32 {
	return GetHullSoaVertices(hull)
}

func getHullSoaNormalsWrite(hull *HullData) []float32 {
	return GetHullSoaNormals(hull)
}

func updateHullEdgeCosines(hull *HullData) {
	edges := GetHullEdges(hull)
	planes := GetHullPlanes(hull)
	cosines := blobSlice[float32](unsafe.Pointer(hull), hull.EdgeCosineOffset, int(hull.EdgeCount/2))
	count := int(hull.EdgeCount / 2)

	for i := range count {
		cosines[i] = Dot(planes[edges[2*i].Face].Normal, planes[edges[2*i+1].Face].Normal)
	}
}

// findHullSupportVertex is b3FindHullSupportVertex.
func findHullSupportVertex(hull *HullData, direction Vec3) int {
	bestIndex := NullIndex
	bestDot := -fltMax

	vertexCount := int(hull.VertexCount)
	soa := GetHullSoaVertices(hull)

	soaVertexCount := getHullSoaStride(vertexCount)
	vx := soa
	vy := soa[soaVertexCount:]
	vz := soa[2*soaVertexCount:]

	dx := splatW(direction.X)
	dy := splatW(direction.Y)
	dz := splatW(direction.Z)
	four := splatW(4.0)
	index := setW(0.0, 1.0, 2.0, 3.0)
	bestDotW := splatW(-fltMax)
	bestIndexW := splatW(-1.0)

	// Keep this at 4 wide to avoid regressing 4 wide platforms.
	loopCount := (vertexCount + 3) &^ 3
	for i := 0; i < loopCount; i += 4 {
		dot := addW(addW(mulW(dx, loadW(vx[i:])), mulW(dy, loadW(vy[i:]))),
			mulW(dz, loadW(vz[i:])))
		mask := greaterThanW(dot, bestDotW)
		bestDotW = blendW(bestDotW, dot, mask)
		bestIndexW = blendW(bestIndexW, index, mask)
		index = addW(index, four)
	}

	var dots [4]float32
	var indices [4]float32
	storeW(dots[:], bestDotW)
	storeW(indices[:], bestIndexW)

	for lane := range 4 {
		laneIndex := int(indices[lane])
		if laneIndex >= 0 && (dots[lane] > bestDot || (dots[lane] == bestDot && laneIndex < bestIndex)) {
			bestIndex = laneIndex
			bestDot = dots[lane]
		}
	}

	assert(0 <= bestIndex && bestIndex < vertexCount, "0 <= bestIndex && bestIndex < vertexCount")
	return bestIndex
}

// findHullSupportFace is b3FindHullSupportFace.
func findHullSupportFace(hull *HullData, direction Vec3) int {
	bestIndex := NullIndex
	bestDot := -fltMax

	faceCount := int(hull.FaceCount)
	planes := GetHullPlanes(hull)

	for index := range faceCount {
		dot := Dot(planes[index].Normal, direction)
		if dot > bestDot {
			bestDot = dot
			bestIndex = index
		}
	}
	assert(bestIndex >= 0, "bestIndex >= 0")

	return bestIndex
}

// isValidHull is b3IsValidHull. Without B3_ENABLE_VALIDATION it always returns true.
func isValidHull(hull *HullData) bool {
	if !validateEnabled {
		return true
	}

	if hull.Version != HullVersion {
		return false
	}

	v := int(hull.VertexCount)
	e := int(hull.EdgeCount / 2)
	f := int(hull.FaceCount)

	if v-e+f != 2 {
		return false
	}

	vertices := GetHullVertices(hull)
	edges := GetHullEdges(hull)
	for index := range int(hull.VertexCount) {
		vertex := &vertices[index]
		edge := &edges[vertex.Edge]

		if int(edge.Origin) != index {
			return false
		}
	}

	for index := 0; index < int(hull.EdgeCount); index += 2 {
		edge := &edges[index+0]
		twin := &edges[index+1]

		if int(edge.Twin) != index+1 {
			return false
		}

		if int(twin.Twin) != index+0 {
			return false
		}
	}

	faces := GetHullFaces(hull)
	planes := GetHullPlanes(hull)
	for faceIndex := range int(hull.FaceCount) {
		face := &faces[faceIndex]

		baseEdgeIndex := int(face.Edge)

		plane := planes[faceIndex]
		if planeSeparation(plane, hull.Center) >= 0.0 {
			return false
		}

		edgeIndex := baseEdgeIndex
		for {
			edge := &edges[edgeIndex]
			next := &edges[edge.Next]
			twin := &edges[edge.Twin]

			if int(edge.Face) != faceIndex {
				return false
			}

			if int(twin.Twin) != edgeIndex {
				return false
			}

			if next.Origin != twin.Origin {
				return false
			}

			edgeIndex = int(edge.Next)
			if edgeIndex == baseEdgeIndex {
				break
			}
		}
	}

	if hull.Volume <= 0.0 {
		return false
	}

	if hull.SurfaceArea <= 0.0 {
		return false
	}

	if hull.InnerRadius <= 0.0 {
		return false
	}

	return true
}

// CreateCylinder creates a tessellated cylinder as a hull.
func CreateCylinder(height, radius, yOffset float32, sides int) *HullData {
	assert(height > 0.0, "height > 0.0f")
	assert(radius > 0.0, "radius > 0.0f")
	assert(3 <= sides && sides <= 32, "3 <= sides && sides <= 32")

	pointCount := 2 * sides
	points := make([]Vec3, pointCount)
	assert(points != nil, "points != NULL")

	var alpha float32 = 0.0
	deltaAlpha := 2.0 * Pi / float32(sides)

	for index := range sides {
		sinAlpha := Sin(alpha)
		cosAlpha := Cos(alpha)

		points[2*index+0] = Vec3{float32(radius * cosAlpha), yOffset, float32(radius * sinAlpha)}
		points[2*index+1] = Vec3{float32(radius * cosAlpha), yOffset + height, float32(radius * sinAlpha)}

		alpha += deltaAlpha
	}

	hull := CreateHull(points, pointCount, pointCount)
	assert(int(hull.VertexCount) == pointCount, "hull->vertexCount == pointCount")
	assert(int(hull.EdgeCount) == 6*sides, "hull->edgeCount == 6 * sides")
	assert(int(hull.FaceCount) == sides+2, "hull->faceCount == sides + 2")

	return hull
}

// CreateCone creates a tessellated cone as a hull.
func CreateCone(height, radius1, radius2 float32, slices int) *HullData {
	assert(height > 0.0, "height > 0.0f")
	assert(radius1 > 0.0, "radius1 > 0.0f")
	assert(radius2 > 0.0, "radius2 > 0.0f")
	assert(4 <= slices && slices <= 32, "4 <= slices && slices <= 32")

	pointCount := 2 * slices
	points := make([]Vec3, pointCount)
	assert(points != nil, "points != NULL")

	var alpha float32 = 0.0
	deltaAlpha := 2.0 * Pi / float32(slices)

	for index := range slices {
		sinAlpha := Sin(alpha)
		cosAlpha := Cos(alpha)

		points[2*index+0] = Vec3{float32(radius1 * cosAlpha), 0.0, float32(radius1 * sinAlpha)}
		points[2*index+1] = Vec3{float32(radius2 * cosAlpha), height, float32(radius2 * sinAlpha)}

		alpha += deltaAlpha
	}

	hull := CreateHull(points, pointCount, pointCount)
	assert(int(hull.VertexCount) == pointCount, "hull->vertexCount == pointCount")
	assert(int(hull.EdgeCount) == 6*slices, "hull->edgeCount == 6 * slices")
	assert(int(hull.FaceCount) == slices+2, "hull->faceCount == slices + 2")

	return hull
}

// CreateRock creates a rock shaped hull.
func CreateRock(radius float32) *HullData {
	pointCount := 10

	// Golden ratio
	phi := (1.0 + sqrtf(5.0)) / 2.0

	// Fibonacci lattice
	var points [10]Vec3

	// Azimuthal angle
	theta := 2.0 * Pi / phi

	cs := CosSin{1.0, 0.0}
	deltaCS := ComputeCosSin(theta)

	for i := range pointCount {
		// Z coordinate
		z := 1.0 - (float32(2.0*float32(i))+1.0)/float32(pointCount)
		// Radius in xy-plane
		radiusXY := sqrtf(1.0 - float32(z*z))

		points[i].X = float32(radius * radiusXY * cs.Cosine)
		points[i].Y = float32(radius * radiusXY * cs.Sine)
		points[i].Z = float32(radius * z)

		cs0 := cs
		cs.Cosine = float32(deltaCS.Cosine*cs0.Cosine) - float32(deltaCS.Sine*cs0.Sine)
		cs.Sine = float32(deltaCS.Sine*cs0.Cosine) + float32(deltaCS.Cosine*cs0.Sine)
	}

	return CreateHull(points[:], pointCount, pointCount)
}

// nextComplexHullRandom is PEEL's BasicRandom. This is just for testing.
func nextComplexHullRandom(state *uint32) uint32 {
	*state = *state*2147001325 + 715136305
	return *state
}

func complexHullRandomFloat(state *uint32) float32 {
	return float32(nextComplexHullRandom(state)&0xffff)/65535.0 - 0.5
}

func complexHullRandomDirection(state *uint32) Vec3 {
	var point Vec3
	var lengthSquared float32
	for {
		point.X = complexHullRandomFloat(state)
		point.Y = complexHullRandomFloat(state)
		point.Z = complexHullRandomFloat(state)
		lengthSquared = Dot(point, point)
		if !(lengthSquared > 0.25) {
			break
		}
	}

	return Normalize(point)
}

// CreateComplexHull is the hull of 32 pseudo random points on a sphere. From the convex pile hull in PEEL.
func CreateComplexHull(radius float32) *HullData {
	const pointCount = 32

	var points [pointCount]Vec3
	var state uint32 = 42
	for i := range pointCount {
		points[i] = MulSV(radius, complexHullRandomDirection(&state))
	}

	return CreateHull(points[:], pointCount, pointCount)
}

func updateHullBounds(hull *HullData) {
	points := GetHullPoints(hull)
	vertexCount := int(hull.VertexCount)

	assert(vertexCount > 0, "vertexCount > 0")
	var bounds AABB
	bounds.LowerBound = points[0]
	bounds.UpperBound = points[0]

	for i := 1; i < vertexCount; i++ {
		p := points[i]
		bounds.LowerBound = Min(bounds.LowerBound, p)
		bounds.UpperBound = Max(bounds.UpperBound, p)
	}

	hull.AABB = bounds
}

// sumOfProducts4 is a1*b1 + a2*b2 + a3*b3 + a4*b4 evaluated left to right with each product rounded.
func sumOfProducts4(a1, b1, a2, b2, a3, b3, a4, b4 float32) float32 {
	return float32(a1*b1) + float32(a2*b2) + float32(a3*b3) + float32(a4*b4)
}

// updateHullBulkProperties is b3UpdateHullBulkProperties.
// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
func updateHullBulkProperties(hull *HullData) bool {
	points := GetHullPoints(hull)
	faces := GetHullFaces(hull)
	edges := GetHullEdges(hull)
	planes := GetHullPlanes(hull)

	var area float32 = 0.0
	var volume float32 = 0.0
	center := Vec3Zero

	// Use the first vertex to reduce round-off errors.
	origin := points[0]

	var xx float32 = 0.0
	var xy float32 = 0.0
	var yy float32 = 0.0
	var xz float32 = 0.0
	var zz float32 = 0.0
	var yz float32 = 0.0

	faceCount := int(hull.FaceCount)

	for faceIndex := range faceCount {
		face := &faces[faceIndex]
		edge1 := &edges[face.Edge]
		edge2 := &edges[edge1.Next]
		edge3 := &edges[edge2.Next]

		assert(edge1 != edge3, "edge1 != edge3")
		assert(int32(edge1.Origin) < hull.VertexCount, "edge1->origin < hull->vertexCount")

		v1 := Sub(points[edge1.Origin], origin)

		for {
			assert(int32(edge2.Origin) < hull.VertexCount, "edge2->origin < hull->vertexCount")
			assert(int32(edge3.Origin) < hull.VertexCount, "edge3->origin < hull->vertexCount")

			v2 := Sub(points[edge2.Origin], origin)
			v3 := Sub(points[edge3.Origin], origin)

			area += Length(Cross(Sub(v2, v1), Sub(v3, v1)))

			det := scalarTripleProduct(v1, v2, v3)

			volume += det

			v4 := Add(v1, Add(v2, v3))
			center = Add(center, MulSV(det, v4))

			xx += float32(det * sumOfProducts4(v1.X, v1.X, v2.X, v2.X, v3.X, v3.X, v4.X, v4.X))
			yy += float32(det * sumOfProducts4(v1.Y, v1.Y, v2.Y, v2.Y, v3.Y, v3.Y, v4.Y, v4.Y))
			zz += float32(det * sumOfProducts4(v1.Z, v1.Z, v2.Z, v2.Z, v3.Z, v3.Z, v4.Z, v4.Z))
			xy += float32(det * sumOfProducts4(v1.X, v1.Y, v2.X, v2.Y, v3.X, v3.Y, v4.X, v4.Y))
			xz += float32(det * sumOfProducts4(v1.X, v1.Z, v2.X, v2.Z, v3.X, v3.Z, v4.X, v4.Z))
			yz += float32(det * sumOfProducts4(v1.Y, v1.Z, v2.Y, v2.Z, v3.Y, v3.Z, v4.Y, v4.Z))

			edge2 = edge3
			edge3 = &edges[edge3.Next]
			if edge1 == edge3 {
				break
			}
		}
	}

	validate(volume > 0.0, "volume > 0.0f")

	localCenter := Vec3Zero
	if volume > 0.0 {
		localCenter = MulSV(0.25/volume, center)
	}
	center = Add(localCenter, origin)

	radius := fltMax
	for faceIndex := range faceCount {
		plane := planes[faceIndex]
		distance := planeSeparation(plane, center)
		validate(distance < 0.0, "distance < 0.0f")

		radius = MinFloat(radius, -distance)
	}

	validate(0.0 < radius && radius < fltMax, "0.0f < radius && radius < FLT_MAX")

	var inertia Matrix3
	inertia.Cx.X = yy + zz
	inertia.Cy.X = -xy
	inertia.Cz.X = -xz
	inertia.Cx.Y = -xy
	inertia.Cy.Y = xx + zz
	inertia.Cz.Y = -yz
	inertia.Cx.Z = -xz
	inertia.Cy.Z = -yz
	inertia.Cz.Z = xx + yy

	mass := volume / 6.0

	centralInertia := MulSM(1.0/120.0, inertia)
	centralInertia = SubMM(centralInertia, Steiner(mass, localCenter))

	hull.Center = center
	hull.CentralInertia = centralInertia
	hull.Volume = mass
	hull.SurfaceArea = float32(0.5 * area)
	hull.InnerRadius = radius

	if mass <= 0.0 {
		return false
	}

	if volume <= 0.0 {
		return false
	}

	if area <= 0.0 {
		return false
	}

	if radius <= 0.0 {
		return false
	}

	return true
}

// CreateHull creates a generic convex hull. This can fail if MaxHullVertices, MaxHullFaces,
// or MaxHullEdges is exceeded.
// PORT: points is a slice; pointCount is kept (it may be less than len(points)).
func CreateHull(points []Vec3, pointCount, maxVertexCount int) *HullData {
	return createHull(points, pointCount, maxVertexCount, false)
}

// CreateHullUnreduced builds a convex hull without discarding outside points to
// meet a vertex budget. Interior, duplicate and coplanar points do not count
// against the output limits. The ordinary float32 hull tolerance still applies;
// nil reports invalid geometry or a hull that exceeds native topology limits.
func CreateHullUnreduced(points []Vec3, pointCount int) *HullData {
	return createHull(points, pointCount, MaxHullVertices, true)
}

func createHull(points []Vec3, pointCount, maxVertexCount int, requireComplete bool) *HullData {
	if pointCount > len(points) {
		return nil
	}
	if pointCount < 4 {
		return nil
	}

	origin := points[0]
	clampedMaxCount := ClampInt(maxVertexCount, 4, MaxHullVertices)

	// Single allocation for all working memory.
	sizes := computeHullWorkSizes(pointCount, clampedMaxCount)

	var builder hullBuilder
	shiftedPoints := hullBuilderInit(&builder, &sizes)

	ok := builder.construct(points, pointCount, clampedMaxCount, origin, shiftedPoints, requireComplete)
	if ok == false {
		return nil
	}

	if builder.finalVertexCount > MaxHullVertices {
		log("hull final vertex count of %d exceeds limit of %d", builder.finalVertexCount, MaxHullVertices)
		return nil
	}

	if builder.finalFaceCount > MaxHullFaces {
		log("hull final face count of %d exceeds limit of %d", builder.finalFaceCount, MaxHullFaces)
		return nil
	}

	maxHalfEdgeCount := 2 * MaxHullEdges
	if builder.finalHalfEdgeCount > maxHalfEdgeCount {
		log("hull final half edge count of %d exceeds limit of %d", builder.finalHalfEdgeCount, maxHalfEdgeCount)
		return nil
	}

	// Walk lists into temp arrays bounded by B3_HULL_MAX_COUNT, stamping finalIndex on each node so
	// the resolution pass below is O(E + F) instead of O(E^2 + F^2).
	var tempVertices [hullMaxCount]*qhVertex
	vertexCount := 0
	for node := builder.vertexList.link.next; node != &builder.vertexList.link; node = node.next {
		assert(vertexCount < hullMaxCount, "vertexCount < B3_HULL_MAX_COUNT")

		vertex := vertexOfNode(node)
		vertex.finalIndex = vertexCount
		tempVertices[vertexCount] = vertex
		vertexCount++
	}

	// Collect edges in twin-paired order (i, i+1) by stamping each pair as we discover it.
	var tempFaces [hullMaxCount]*qhFace
	var tempEdges [hullMaxCount]*qhHalfEdge
	faceCount := 0
	edgeCount := 0

	for faceNode := builder.faceList.link.next; faceNode != &builder.faceList.link; faceNode = faceNode.next {
		assert(faceCount < hullMaxCount, "faceCount < B3_HULL_MAX_COUNT")

		face := faceOfNode(faceNode)
		face.finalIndex = faceCount
		tempFaces[faceCount] = face
		faceCount++

		edge := face.edge
		for {
			if edge.finalIndex < 0 {
				assert(edgeCount+1 < hullMaxCount, "edgeCount + 1 < B3_HULL_MAX_COUNT")

				edge.finalIndex = edgeCount
				tempEdges[edgeCount] = edge
				edgeCount++
				edge.twin.finalIndex = edgeCount
				tempEdges[edgeCount] = edge.twin
				edgeCount++
			}
			edge = edge.next
			if edge == face.edge {
				break
			}
		}
	}

	soaVertexCount := getHullSoaStride(vertexCount)
	soaNormalCount := getHullSoaStride(faceCount)

	// Allocate the hull. Arrays hang off the end.
	byteCount := alignUp8(int(unsafe.Sizeof(HullData{})))
	vertexOffset := byteCount
	byteCount += alignUp8(vertexCount * int(unsafe.Sizeof(HullVertex{})))
	pointOffset := byteCount
	byteCount += alignUp8(vertexCount * int(unsafe.Sizeof(Vec3{})))
	edgeOffset := byteCount
	byteCount += alignUp8(edgeCount * int(unsafe.Sizeof(HullHalfEdge{})))
	planeOffset := byteCount
	byteCount += alignUp8(faceCount * int(unsafe.Sizeof(Plane{})))
	faceOffset := byteCount
	byteCount += alignUp8(faceCount * int(unsafe.Sizeof(HullFace{})))
	soaVertexOffset := byteCount
	byteCount += alignUp8(3 * soaVertexCount * 4)
	soaNormalOffset := byteCount
	byteCount += alignUp8(3 * soaNormalCount * 4)
	edgeCosineOffset := byteCount
	byteCount += alignUp8((edgeCount / 2) * 4)

	// PORT: b3Alloc + memset( 0 ) is blobAlloc (8-byte aligned, zeroed).
	hull := (*HullData)(blobAlloc(byteCount))

	hull.Version = HullVersion
	hull.VertexOffset = int32(vertexOffset)
	hull.PointOffset = int32(pointOffset)
	hull.EdgeOffset = int32(edgeOffset)
	hull.PlaneOffset = int32(planeOffset)
	hull.FaceOffset = int32(faceOffset)
	hull.SoaVertexOffset = int32(soaVertexOffset)
	hull.SoaNormalOffset = int32(soaNormalOffset)
	hull.EdgeCosineOffset = int32(edgeCosineOffset)

	hull.VertexCount = int32(vertexCount)
	hull.EdgeCount = int32(edgeCount)
	hull.FaceCount = int32(faceCount)

	hull.ByteCount = int32(byteCount)

	vertices := getHullVerticesWrite(hull)
	edges := getHullEdgesWrite(hull)
	faces := GetHullFaces(hull)
	finalPoints := getHullPointsWrite(hull)
	planes := getHullPlanesWrite(hull)
	soaVertices := getHullSoaVerticesWrite(hull)
	soaNormals := getHullSoaNormalsWrite(hull)

	vx := soaVertices
	vy := vx[soaVertexCount:]
	vz := vy[soaVertexCount:]

	for index := range vertexCount {
		vertices[index].Edge = 0
		p := tempVertices[index].position
		finalPoints[index] = p

		vx[index] = p.X
		vy[index] = p.Y
		vz[index] = p.Z
	}

	for index := vertexCount; index < soaVertexCount; index++ {
		vx[index] = vx[0]
		vy[index] = vy[0]
		vz[index] = vz[0]
	}

	for index := range edgeCount {
		edge := tempEdges[index]
		assert(0 <= edge.next.finalIndex && edge.next.finalIndex < edgeCount, "next edge in range")
		assert(0 <= edge.twin.finalIndex && edge.twin.finalIndex < edgeCount, "twin edge in range")
		assert(0 <= edge.face.finalIndex && edge.face.finalIndex < faceCount, "face in range")
		assert(0 <= edge.origin.finalIndex && edge.origin.finalIndex < vertexCount, "origin vertex in range")

		edges[index].Next = uint16(edge.next.finalIndex)
		edges[index].Twin = uint16(edge.twin.finalIndex)
		edges[index].Face = uint16(edge.face.finalIndex)
		edges[index].Origin = uint16(edge.origin.finalIndex)

		vertices[edge.origin.finalIndex].Edge = uint16(index)
	}

	nx := soaNormals
	ny := nx[soaNormalCount:]
	nz := ny[soaNormalCount:]

	for index := range faceCount {
		face := tempFaces[index]
		assert(0 <= face.edge.finalIndex && face.edge.finalIndex < edgeCount, "face edge in range")

		faces[index].Edge = uint16(face.edge.finalIndex)
		planes[index] = face.plane

		n := face.plane.Normal
		nx[index] = n.X
		ny[index] = n.Y
		nz[index] = n.Z
	}

	for index := faceCount; index < soaNormalCount; index++ {
		nx[index] = 0.0
		ny[index] = 0.0
		nz[index] = 0.0
	}

	// All builder pointers are dead from here on.

	updateHullEdgeCosines(hull)

	updateHullBounds(hull)
	success := updateHullBulkProperties(hull)
	if success == false {
		DestroyHull(hull)
		return nil
	}

	if isValidHull(hull) == false {
		DestroyHull(hull)
		return nil
	}

	hull.Hash = 0
	hull.Hash = hash64NonZero(hullBytes(hull))

	return hull
}

// CloneHull deep clones a hull.
func CloneHull(hull *HullData) *HullData {
	if hull == nil || isValidHull(hull) == false {
		return nil
	}

	clone := (*HullData)(blobAlloc(int(hull.ByteCount)))
	copy(blobBytes(unsafe.Pointer(clone), hull.ByteCount), hullBytes(hull))

	return clone
}

// hashHullData is b3HashHullData.
func hashHullData(hull *HullData) uint64 {
	return hull.Hash
}

// compareHullData is b3CompareHullData.
func compareHullData(hull1, hull2 *HullData) bool {
	if hull1 == hull2 {
		return true
	}

	if hull1.ByteCount != hull2.ByteCount {
		return false
	}

	return bytes.Equal(hullBytes(hull1), hullBytes(hull2))
}

// Hull identity covers every byte, so the structs carry explicit padding. These lock
// the layout, re-audit padding if a size changes.
// PORT: _Static_assert becomes a pair of constant expressions that overflow unless equal.
const (
	_ uintptr = unsafe.Sizeof(HullData{}) - 152
	_ uintptr = 152 - unsafe.Sizeof(HullData{})
	_ uintptr = unsafe.Sizeof(BoxHull{}) - 808
	_ uintptr = 808 - unsafe.Sizeof(BoxHull{})
)

// Implement b3HullMap: see hullMap and hullMapInit above.

// hullMapByteCount is b3HullMapByteCount: total map allocation in bytes, excluding the stored hull data
func hullMapByteCount(m *hullMap) int {
	// The map owns a combined bucket and metadata allocation, valid only when buckets exist
	byteCount := int(unsafe.Sizeof(*m))
	if m.bucketCount() > 0 {
		byteCount += m.totalAllocSize()
	}
	return byteCount
}

// CloneAndTransformHull clones and transforms a hull. Supports non-uniform and mirroring scale.
func CloneAndTransformHull(original *HullData, transform Transform, scale Vec3) *HullData {
	if original == nil || isValidHull(original) == false {
		return nil
	}

	hull := (*HullData)(blobAlloc(int(original.ByteCount)))
	copy(blobBytes(unsafe.Pointer(hull), original.ByteCount), hullBytes(original))

	safeScale := SafeScale(scale)

	edges := getHullEdgesWrite(hull)
	faces := GetHullFaces(hull)
	faceCount := int(hull.FaceCount)
	vertexCount := int(hull.VertexCount)

	if float32(safeScale.X*safeScale.Y)*safeScale.Z < 0.0 {
		// Reflected: reverse edge winding for each face.
		for i := range faceCount {
			face := &faces[i]

			startEdgeIndex := int(face.Edge)
			currentEdgeIndex := startEdgeIndex
			prevEdgeIndex := NullIndex

			for {
				edge := &edges[currentEdgeIndex]

				if int(edge.Next) == startEdgeIndex {
					prevEdgeIndex = currentEdgeIndex
					break
				}

				currentEdgeIndex = int(edge.Next)
				if currentEdgeIndex == startEdgeIndex {
					break
				}
			}

			assert(prevEdgeIndex != NullIndex, "prevEdgeIndex != B3_NULL_INDEX")

			currentEdgeIndex = startEdgeIndex

			for {
				edge := &edges[currentEdgeIndex]
				nextIndex := edge.Next
				edge.Next = uint16(prevEdgeIndex)

				if currentEdgeIndex < int(edge.Twin) {
					twin := &edges[edge.Twin]
					edge.Origin, twin.Origin = twin.Origin, edge.Origin
				}

				prevEdgeIndex = currentEdgeIndex
				currentEdgeIndex = int(nextIndex)
				if currentEdgeIndex == startEdgeIndex {
					break
				}
			}
		}

		vertices := getHullVerticesWrite(hull)

		for i := range vertexCount {
			vertex := &vertices[i]
			edge := &edges[vertex.Edge]
			vertex.Edge = edge.Twin
		}
	}

	matrix := MakeMatrixFromQuat(transform.Q)
	points := getHullPointsWrite(hull)

	soaVertexCount := getHullSoaStride(vertexCount)
	soa := getHullSoaVerticesWrite(hull)
	vx := soa
	vy := vx[soaVertexCount:]
	vz := vy[soaVertexCount:]

	for i := range vertexCount {
		p := Add(MulMV(matrix, Mul(safeScale, points[i])), transform.P)
		points[i] = p
		vx[i] = p.X
		vy[i] = p.Y
		vz[i] = p.Z
	}

	for i := vertexCount; i < soaVertexCount; i++ {
		vx[i] = vx[0]
		vy[i] = vy[0]
		vz[i] = vz[0]
	}

	planes := getHullPlanesWrite(hull)
	soaNormalCount := getHullSoaStride(faceCount)
	soaN := getHullSoaNormalsWrite(hull)
	nx := soaN
	ny := nx[soaNormalCount:]
	nz := ny[soaNormalCount:]

	for i := range faceCount {
		count := 0
		centroid := Vec3Zero
		normal := Vec3Zero

		face := &faces[i]
		startEdgeIndex := face.Edge
		currentEdgeIndex := startEdgeIndex

		startEdge := &edges[currentEdgeIndex]
		assert(int(startEdge.Face) == i, "startEdge->face == i")
		assert(int(startEdge.Origin) < vertexCount, "startEdge->origin < vertexCount")

		origin := points[startEdge.Origin]

		for {
			edge := &edges[currentEdgeIndex]
			twin := &edges[edge.Twin]
			assert(twin.Twin == currentEdgeIndex, "twin->twin == currentEdgeIndex")

			v1 := Sub(points[edge.Origin], origin)
			v2 := Sub(points[twin.Origin], origin)

			count++
			centroid = Add(centroid, v1)
			normal.X += float32((v1.Y - v2.Y) * (v1.Z + v2.Z))
			normal.Y += float32((v1.Z - v2.Z) * (v1.X + v2.X))
			normal.Z += float32((v1.X - v2.X) * (v1.Y + v2.Y))

			currentEdgeIndex = edge.Next
			if currentEdgeIndex == startEdgeIndex {
				break
			}
		}

		assert(count > 0, "count > 0")
		centroid = MulSV(1.0/float32(count), centroid)
		centroid = Add(centroid, origin)

		area := Length(normal)
		assert(area > 0.0, "area > 0.0f")
		normal = MulSV(1.0/area, normal)

		planes[i] = makePlaneFromNormalAndPoint(normal, centroid)

		nx[i] = normal.X
		ny[i] = normal.Y
		nz[i] = normal.Z
	}

	for i := faceCount; i < soaNormalCount; i++ {
		nx[i] = 0.0
		ny[i] = 0.0
		nz[i] = 0.0
	}

	updateHullEdgeCosines(hull)

	updateHullBounds(hull)
	success := updateHullBulkProperties(hull)
	if success == false {
		return nil
	}

	// Must ensure the hash is 0 so it doesn't contribute to itself.
	hull.Hash = 0
	hull.Hash = hash64NonZero(hullBytes(hull))

	validate(isValidHull(hull), "b3IsValidHull( hull )")

	return hull
}

// DestroyHull destroys a hull.
// PORT: the garbage collector frees the image; this is a no-op kept for API parity.
func DestroyHull(hull *HullData) {
	_ = hull
}

// ComputeHullMass computes mass properties of a hull.
func ComputeHullMass(shape *HullData, density float32) MassData {
	var out MassData
	out.Mass = float32(density * shape.Volume)
	out.Center = shape.Center

	// Inertia about the center of mass
	out.Inertia = MulSM(density, shape.CentralInertia)
	return out
}

// ComputeHullAABB computes the bounding box of a transformed hull.
func ComputeHullAABB(shape *HullData, transform Transform) AABB {
	return shape.AABB.Transform(transform)
}

// computeSweptHullAABB is b3ComputeSweptHullAABB.
func computeSweptHullAABB(shape *HullData, xf1, xf2 Transform) AABB {
	aabb1 := shape.AABB.Transform(xf1)
	aabb2 := shape.AABB.Transform(xf2)
	return aabb1.Union(aabb2)
}

// OverlapHull overlaps a shape versus a hull.
func OverlapHull(shape *HullData, shapeTransform Transform, proxy *ShapeProxy) bool {
	points := GetHullPoints(shape)

	var input DistanceInput
	input.ProxyA = ShapeProxy{points, shape.VertexCount, 0.0}
	input.ProxyB = *proxy
	input.Transform = InvMulTransforms(shapeTransform, TransformIdentity)
	input.UseRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil)
	return output.Distance < OverlapSlop()
}

// RayCastHull ray casts versus a hull shape in local space. A zero length ray is a point query. Initial overlap
// reports a hit at the ray origin with zero fraction and zero normal.
func RayCastHull(shape *HullData, input *RayCastInput) CastOutput {
	assert(IsValidRay(input), "b3IsValidRay( input )")
	output := CastOutput{}

	var lower float32 = 0.0
	upper := input.MaxFraction
	bestFace := NullIndex

	planes := GetHullPlanes(shape)

	for faceIndex := range int(shape.FaceCount) {
		plane := planes[faceIndex]

		distance := plane.Offset - Dot(plane.Normal, input.Origin)
		denominator := Dot(plane.Normal, input.Translation)

		if denominator == 0.0 {
			if distance < 0.0 {
				return output
			}
		} else {
			fraction := distance / denominator

			if denominator < 0.0 {
				if fraction > lower {
					bestFace = faceIndex
					lower = fraction
				}
			} else {
				if fraction < upper {
					upper = fraction
				}
			}

			if upper < lower {
				return output
			}
		}
	}

	if bestFace >= 0 {
		output.Point = Add(input.Origin, MulSV(lower, input.Translation))
		output.Normal = planes[bestFace].Normal
		output.Fraction = lower
		output.Hit = true
	} else {
		output.Point = input.Origin
		output.Hit = true
	}

	return output
}

// ShapeCastHull shape casts versus a hull. Initial overlap is treated as a miss.
func ShapeCastHull(shape *HullData, input *ShapeCastInput) CastOutput {
	points := GetHullPoints(shape)

	var pairInput ShapeCastPairInput
	pairInput.ProxyA = ShapeProxy{points, shape.VertexCount, 0.0}
	pairInput.ProxyB = input.Proxy
	pairInput.Transform = TransformIdentity
	pairInput.TranslationB = input.Translation
	pairInput.MaxFraction = input.MaxFraction
	pairInput.CanEncroach = input.CanEncroach

	output := ShapeCast(&pairInput)
	return output
}

// collideMoverAndHull is b3CollideMoverAndHull.
func collideMoverAndHull(result *PlaneResult, shape *HullData, mover *Capsule) int {
	points := GetHullPoints(shape)
	var distanceInput DistanceInput
	distanceInput.ProxyA = ShapeProxy{points, shape.VertexCount, 0.0}
	distanceInput.ProxyB = ShapeProxy{capsulePoints(mover), 2, mover.Radius}
	distanceInput.Transform = TransformIdentity
	distanceInput.UseRadii = false

	totalRadius := mover.Radius

	cache := SimplexCache{}
	distanceOutput := ShapeDistance(&distanceInput, &cache, nil)

	if distanceOutput.Distance == 0.0 {
		// I could handle deep overlap on hulls, but there is no reasonable solution for
		// deep overlap on meshes. So if someone converted a hull to a mesh there would be
		// different behavior. So I think there is not a good reason to handle deep overlap
		// on hulls.
		return 0
	}

	if distanceOutput.Distance <= totalRadius {
		plane := Plane{distanceOutput.Normal, totalRadius - distanceOutput.Distance}
		*result = PlaneResult{plane, distanceOutput.PointA, 0, 0, 0}
		return 1
	}

	return 0
}

// computeHullExtent is b3ComputeHullExtent.
func computeHullExtent(hull *HullData, origin Vec3) shapeExtent {
	points := GetHullPoints(hull)

	var extent shapeExtent
	extent.minExtent = hull.InnerRadius
	extent.maxExtent = Vec3Zero
	for index := range int(hull.VertexCount) {
		point := points[index]
		extent.maxExtent = Max(extent.maxExtent, Abs(Sub(point, origin)))
	}

	return extent
}

// computeHullProjectedArea is b3ComputeHullProjectedArea.
func computeHullProjectedArea(hull *HullData, direction Vec3) float32 {
	var area float32 = 0.0

	faceCount := int(hull.FaceCount)
	hullFaces := GetHullFaces(hull)
	hullEdges := GetHullEdges(hull)
	hullPoints := GetHullPoints(hull)

	for i := range faceCount {
		face := &hullFaces[i]

		baseEdge := int(face.Edge)
		edge := &hullEdges[baseEdge]
		p1 := hullPoints[edge.Origin]

		edgeIndex := int(edge.Next)
		edge = &hullEdges[edgeIndex]
		p2 := hullPoints[edge.Origin]

		edgeIndex = int(edge.Next)

		for {
			edge = &hullEdges[edgeIndex]
			p3 := hullPoints[edge.Origin]

			e1 := Sub(p2, p1)
			e2 := Sub(p3, p1)
			n := Cross(e1, e2)
			a := Dot(n, direction)
			area += MaxFloat(a, 0.0)

			p2 = p3
			edgeIndex = int(edge.Next)
			if edgeIndex == baseEdge {
				break
			}
		}
	}

	return float32(0.5 * area)
}

// boxHullTemplate is s_boxHull: constant template box (vertex/edge/face topology).
// b3MakeTransformedBoxHull copies and fills in the runtime-dependent fields (boxPoints, boxPlanes,
// aabb, mass properties, hash).
var boxHullTemplate = BoxHull{
	Base: HullData{
		Version:          HullVersion,
		ByteCount:        int32(unsafe.Sizeof(BoxHull{})),
		Hash:             0,
		VertexCount:      8,
		EdgeCount:        24,
		FaceCount:        6,
		VertexOffset:     int32(unsafe.Offsetof(BoxHull{}.BoxVertices)),
		PointOffset:      int32(unsafe.Offsetof(BoxHull{}.BoxPoints)),
		EdgeOffset:       int32(unsafe.Offsetof(BoxHull{}.BoxEdges)),
		PlaneOffset:      int32(unsafe.Offsetof(BoxHull{}.BoxPlanes)),
		FaceOffset:       int32(unsafe.Offsetof(BoxHull{}.BoxFaces)),
		SoaVertexOffset:  int32(unsafe.Offsetof(BoxHull{}.Vx)),
		SoaNormalOffset:  int32(unsafe.Offsetof(BoxHull{}.Nx)),
		EdgeCosineOffset: int32(unsafe.Offsetof(BoxHull{}.EdgeCosines)),
	},
	BoxVertices: [8]HullVertex{
		0: {Edge: 8},
		1: {Edge: 1},
		2: {Edge: 0},
		3: {Edge: 9},
		4: {Edge: 13},
		5: {Edge: 3},
		6: {Edge: 5},
		7: {Edge: 11},
	},
	BoxEdges: [24]HullHalfEdge{
		0: {2, 1, 2, 0}, 1: {17, 0, 1, 5}, 2: {4, 3, 1, 0}, 3: {20, 2, 5, 3},
		4: {6, 5, 5, 0}, 5: {23, 4, 6, 4}, 6: {0, 7, 6, 0}, 7: {18, 6, 2, 2},
		8: {10, 9, 0, 1}, 9: {21, 8, 3, 5}, 10: {12, 11, 3, 1}, 11: {16, 10, 7, 2},
		12: {14, 13, 7, 1}, 13: {19, 12, 4, 4}, 14: {8, 15, 4, 1}, 15: {22, 14, 0, 3},
		16: {7, 17, 3, 2}, 17: {9, 16, 2, 5}, 18: {11, 19, 6, 2}, 19: {5, 18, 7, 4},
		20: {15, 21, 1, 3}, 21: {1, 20, 0, 5}, 22: {3, 23, 4, 3}, 23: {13, 22, 5, 4},
	},
	BoxFaces: [6]HullFace{
		0: {Edge: 0},
		1: {Edge: 8},
		2: {Edge: 16},
		3: {Edge: 20},
		4: {Edge: 19},
		5: {Edge: 21},
	},
}

// MakeTransformedBoxHull makes a transformed box as a hull. Do not call DestroyHull on this.
// hx, hy, hz positive half widths
// transform local transform of box
// PORT: as in C the result is a value whose Base offsets point into the value itself. Use
// &box.Base as the *HullData; copying Base alone detaches it from its arrays.
func MakeTransformedBoxHull(hx, hy, hz float32, transform Transform) BoxHull {
	assert(IsValidTransform(transform), "b3IsValidTransform( transform )")

	boxHull := boxHullTemplate

	minH := float32(0.2 * LinearSlop())
	h := Max(Vec3{minH, minH, minH}, Vec3{hx, hy, hz})

	boxHull.Base.AABB = AABB{Neg(h), h}.Transform(transform)
	boxHull.Base.SurfaceArea = float32(8.0 * (float32(h.X*h.Y) + float32(h.X*h.Z) + float32(h.Y*h.Z)))
	boxHull.Base.Volume = float32(8.0 * h.X * h.Y * h.Z)
	boxHull.Base.InnerRadius = MinFloat(h.X, MinFloat(h.Y, h.Z))
	boxHull.Base.Center = transform.P

	boxInertia := boxInertia(boxHull.Base.Volume, Neg(h), h)
	boxHull.Base.CentralInertia = rotateInertia(transform.Q, boxInertia)

	lower := Neg(h)
	upper := h

	boxHull.BoxPlanes[0] = transformPlane(transform, makePlaneFromNormalAndPoint(Neg(Vec3AxisX), lower))
	boxHull.BoxPlanes[1] = transformPlane(transform, makePlaneFromNormalAndPoint(Vec3AxisX, upper))
	boxHull.BoxPlanes[2] = transformPlane(transform, makePlaneFromNormalAndPoint(Neg(Vec3AxisY), lower))
	boxHull.BoxPlanes[3] = transformPlane(transform, makePlaneFromNormalAndPoint(Vec3AxisY, upper))
	boxHull.BoxPlanes[4] = transformPlane(transform, makePlaneFromNormalAndPoint(Neg(Vec3AxisZ), lower))
	boxHull.BoxPlanes[5] = transformPlane(transform, makePlaneFromNormalAndPoint(Vec3AxisZ, upper))

	boxHull.BoxPoints[0] = TransformPoint(transform, Vec3{h.X, h.Y, h.Z})
	boxHull.BoxPoints[1] = TransformPoint(transform, Vec3{-h.X, h.Y, h.Z})
	boxHull.BoxPoints[2] = TransformPoint(transform, Vec3{-h.X, -h.Y, h.Z})
	boxHull.BoxPoints[3] = TransformPoint(transform, Vec3{h.X, -h.Y, h.Z})
	boxHull.BoxPoints[4] = TransformPoint(transform, Vec3{h.X, h.Y, -h.Z})
	boxHull.BoxPoints[5] = TransformPoint(transform, Vec3{-h.X, h.Y, -h.Z})
	boxHull.BoxPoints[6] = TransformPoint(transform, Vec3{-h.X, -h.Y, -h.Z})
	boxHull.BoxPoints[7] = TransformPoint(transform, Vec3{h.X, -h.Y, -h.Z})

	// SOA
	for i := range 8 {
		boxHull.Vx[i] = boxHull.BoxPoints[i].X
	}

	for i := range 8 {
		boxHull.Vy[i] = boxHull.BoxPoints[i].Y
	}

	for i := range 8 {
		boxHull.Vz[i] = boxHull.BoxPoints[i].Z
	}

	for i := range 6 {
		boxHull.Nx[i] = boxHull.BoxPlanes[i].Normal.X
	}
	boxHull.Nx[6] = 0.0
	boxHull.Nx[7] = 0.0

	for i := range 6 {
		boxHull.Ny[i] = boxHull.BoxPlanes[i].Normal.Y
	}
	boxHull.Ny[6] = 0.0
	boxHull.Ny[7] = 0.0

	for i := range 6 {
		boxHull.Nz[i] = boxHull.BoxPlanes[i].Normal.Z
	}
	boxHull.Nz[6] = 0.0
	boxHull.Nz[7] = 0.0

	// Must ensure the hash is 0 so it doesn't contribute to itself.
	boxHull.Base.Hash = 0
	boxHull.Base.Hash = hash64NonZero(hullBytes(&boxHull.Base))

	return boxHull
}

// MakeCubeHull makes a cube as a hull. Do not call DestroyHull on this.
func MakeCubeHull(halfWidth float32) BoxHull {
	return MakeBoxHull(halfWidth, halfWidth, halfWidth)
}

// MakeOffsetBoxHull makes an offset box as a hull. Do not call DestroyHull on this.
func MakeOffsetBoxHull(hx, hy, hz float32, offset Vec3) BoxHull {
	transform := Transform{P: offset, Q: QuatIdentity}
	return MakeTransformedBoxHull(hx, hy, hz, transform)
}

// MakeBoxHull makes a box as a hull. Do not call DestroyHull on this.
func MakeBoxHull(hx, hy, hz float32) BoxHull {
	return MakeTransformedBoxHull(hx, hy, hz, TransformIdentity)
}

// ScaleBox takes a box with a transform and post scale and converts it into a box with the post scale
// resolved with new half-widths and transform. This accepts non-uniform and negative scale.
// This is approximate if there is shear.
// halfWidths [in/out] the box half widths
// transform [in/out] the box transform with rotation and translation
// postScale the post scale being applied to the box after the transform
// minHalfWidth the minimum half width after scale is applied
func ScaleBox(halfWidths *Vec3, transform *Transform, postScale Vec3, minHalfWidth float32) {
	assert(IsValidFloat(minHalfWidth) && minHalfWidth > 0.0, "b3IsValidFloat( minHalfWidth ) && minHalfWidth > 0.0f")

	q := transform.Q

	if postScale.X < 0.0 || postScale.Y < 0.0 || postScale.Z < 0.0 {
		// todo this might be unnecessary if rotation is identity
		// todo compare with polar decomposition (much more expensive)
		m := MakeMatrixFromQuat(q)
		m.Cx.X *= postScale.X
		m.Cy.X *= postScale.X
		m.Cz.X *= postScale.X
		m.Cx.Y *= postScale.Y
		m.Cy.Y *= postScale.Y
		m.Cz.Y *= postScale.Y
		m.Cx.Z *= postScale.Z
		m.Cy.Z *= postScale.Z
		m.Cz.Z *= postScale.Z
		m.Cx = Normalize(m.Cx)
		m.Cy = Normalize(m.Cy)
		m.Cz = Normalize(m.Cz)
		if postScale.X < 0.0 {
			m.Cx = Neg(m.Cx)
		}
		if postScale.Y < 0.0 {
			m.Cy = Neg(m.Cy)
		}
		if postScale.Z < 0.0 {
			m.Cz = Neg(m.Cz)
		}
		q = MakeQuatFromMatrix(&m)
	}

	absScale := Abs(postScale)

	h := *halfWidths
	p1 := Mul(absScale, RotateVector(q, Neg(h)))
	p2 := Mul(absScale, RotateVector(q, h))

	localP1 := InvRotateVector(q, p1)
	localP2 := InvRotateVector(q, p2)

	lower := Min(localP1, localP2)
	upper := Max(localP1, localP2)

	scaledHalfWidth := MulSV(0.5, Sub(upper, lower))

	mLimit := Vec3{minHalfWidth, minHalfWidth, minHalfWidth}
	*halfWidths = Max(scaledHalfWidth, mLimit)
	transform.P = Mul(postScale, transform.P)
	transform.Q = q
}

// MakeScaledBoxHull makes a transformed box hull with post scaling. This is useful for boxes that are scaled in
// a level editor. Such scaling can have reflection and shear. In the case of shear the result
// may be approximate. If you need to support shear consider using CreateHull.
// Do not call DestroyHull on this.
// halfWidths positive half widths
// transform local transform of box
// postScale scale applied after the transform, may be negative
// todo use new hull scaling technique
func MakeScaledBoxHull(halfWidths Vec3, transform Transform, postScale Vec3) BoxHull {
	assert(IsValidTransform(transform), "b3IsValidTransform( transform )")

	h := halfWidths
	xf := transform
	ScaleBox(&h, &xf, postScale, float32(4.0*LinearSlop()))
	return MakeTransformedBoxHull(h.X, h.Y, h.Z, xf)
}

// 2D hull (Andrew's monotone chain)

// cross2D is b3Cross2D: cross(b - a, c - a)
func cross2D(a, b, c Vec2) float32 {
	return float32((b.X-a.X)*(c.Y-a.Y)) - float32((b.Y-a.Y)*(c.X-a.X))
}

// SimplifyHull2D simplifies a 2D hull. Used internally for contact manifold simplification. Here for testing.
// Iteratively remove the vertices contributing the least area.
// PORT: the hull pointer becomes a slice holding at least count1 points.
func SimplifyHull2D(hull []Point2D, count1, target int) int {
	assert(target >= 3, "target >= 3")

	if count1 <= 3 {
		return count1
	}

	linearSlop := LinearSlop()
	areaTol := float32(0.25 * linearSlop * linearSlop)
	count2 := count1

	for {
		minArea := fltMax
		minIndex := 0

		for i := range count2 {
			prev := (i + count2 - 1) % count2
			next := (i + 1) % count2
			area := cross2D(hull[prev].P, hull[i].P, hull[next].P)
			validate(area >= -fltEpsilon, "area >= -FLT_EPSILON")

			if area+areaTol < minArea {
				minArea = area
				minIndex = i
			}
		}

		if count2 <= 3 || (minArea > areaTol && count2 <= target) {
			break
		}

		count2 -= 1
		if minIndex < count2 {
			copy(hull[minIndex:count2], hull[minIndex+1:count2+1])
		}
	}

	return count2
}

// sort2D is b3Sort2D. Sort comparator: by x, then by y. Insertion since n is typically 20 or less.
func sort2D(pts []Point2D, count int) {
	for i := 1; i < count; i++ {
		base := pts[i]
		j := i - 1
		for j >= 0 && (pts[j].P.X > base.P.X || (pts[j].P.X == base.P.X && pts[j].P.Y > base.P.Y)) {
			pts[j+1] = pts[j]
			j -= 1
		}
		pts[j+1] = base
	}
}

// weld2D is b3Weld2D. Welds points that have been sorted O(n) time
func weld2D(pts []Point2D, count1 int) int {
	if count1 <= 1 {
		return count1
	}

	linearSlop := float32(0.25 * LinearSlop())
	tolSqr := float32(linearSlop * linearSlop)

	count2 := 1
	baseIndex := 0
	for i := 1; i < count1; i++ {
		distanceSqr := distanceSquared2(pts[i].P, pts[baseIndex].P)
		if distanceSqr < tolSqr {
			if pts[i].OriginalIndex < pts[baseIndex].OriginalIndex {
				pts[baseIndex] = pts[i]
			}
		} else {
			pts[count2] = pts[i]
			baseIndex = count2
			count2 += 1
		}
	}

	return count2
}

// Hull2D computes a 2D hull. Used internally for contact manifold simplification. Here for testing.
// Andrew's monotone chain convex hull algorithm
// hull must have space for 2 * count1 points
// PORT: pts and hull are slices; count is kept.
func Hull2D(pts []Point2D, count int, hull []Point2D) int {
	count1 := count

	if count1 <= 0 {
		return 0
	}

	if count1 == 1 {
		hull[0] = pts[0]
		return 1
	}

	sort2D(pts, count1)
	count1 = weld2D(pts, count1)

	if count1 == 1 {
		hull[0] = pts[0]
		return 1
	}

	if count1 == 2 {
		hull[0] = pts[0]
		hull[1] = pts[1]
		return 2
	}

	count2 := 0

	for i := range count1 {
		for count2 >= 2 {
			area := cross2D(hull[count2-2].P, hull[count2-1].P, pts[i].P)
			if area > 0.0 {
				break
			}

			count2 -= 1
		}

		validate(count2 < 2*count1, "count2 < 2 * count1")
		hull[count2] = pts[i]
		count2 += 1
	}

	lowerCount := count2 + 1

	for i := count1 - 2; i >= 0; i-- {
		for count2 >= lowerCount {
			area := cross2D(hull[count2-2].P, hull[count2-1].P, pts[i].P)
			if area > 0.0 {
				break
			}

			count2 -= 1
		}

		validate(count2 < 2*count1, "count2 < 2 * count1")
		hull[count2] = pts[i]
		count2 += 1
	}

	validate(hull[0].OriginalIndex == hull[count2-1].OriginalIndex, "hull[0].originalIndex == hull[count2 - 1].originalIndex")
	return count2 - 1
}
