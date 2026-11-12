// Port of src/mesh.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.
//
// Dirk Gregorius contributed portions of this code

package box3d

import (
	"math"
	"unsafe"
)

// PORT: b3DeclareArray( b3VertexNode ) etc. are not needed; b3Array( T ) is []T.

const (
	binCount                       = 8   // B3_BIN_COUNT
	desiredTrianglesPerLeaf        = 4   // B3_DESIRED_TRIANGLES_PER_LEAF
	meshLeafNode                   = 3   // B3_LEAF_NODE
	maximumTrianglesPerLeaf        = 8   // B3_MAXIMUM_TRIANGLES_PER_LEAF
	meshStackSize                  = 256 // B3_MESH_STACK_SIZE
	meshNodeLowMask         uint32 = 3   // the 2 bit axis / type field of b3MeshNode::data
)

// PORT: b3MeshNode::data is a union of two bit-field structs. The accessors below read and write
// the low 2 bits (asNode.axis / asLeaf.type) and the high 30 bits (asNode.childOffset /
// asLeaf.triangleCount) of MeshNode.Data, matching the C layout on little endian targets.

func meshNodeType(node *MeshNode) uint32 {
	return node.Data & meshNodeLowMask
}

func meshNodeAxis(node *MeshNode) int {
	return int(node.Data & meshNodeLowMask)
}

func meshNodeChildOffset(node *MeshNode) int {
	return int(node.Data >> 2)
}

func meshNodeTriangleCount(node *MeshNode) int {
	return int(node.Data >> 2)
}

func setMeshNodeLow(node *MeshNode, value int) {
	node.Data = node.Data&^meshNodeLowMask | uint32(value)&meshNodeLowMask
}

func setMeshNodeHigh(node *MeshNode, value int) {
	node.Data = node.Data&meshNodeLowMask | uint32(value)<<2
}

// PORT: node pointers (b3MeshNode*) are indices into the node slice of the mesh. Child access
// (node + 1, node + childOffset) is index arithmetic.

func meshIsLeaf(node *MeshNode) bool {
	return meshNodeType(node) == meshLeafNode
}

func getMeshNodesWrite(mesh *MeshData) []MeshNode {
	return GetMeshNodes(mesh)
}

// getLeftChildWrite is b3GetLeftChildWrite.
func meshGetLeftChildWrite(nodes []MeshNode, node int) int {
	// The left child follows its parent.
	assert(!meshIsLeaf(&nodes[node]), "!b3IsLeaf( node )")
	return node + 1
}

// getLeftChild is b3GetLeftChild.
func meshGetLeftChild(nodes []MeshNode, node int) int {
	// The left child follows its parent.
	assert(!meshIsLeaf(&nodes[node]), "!b3IsLeaf( node )")
	return node + 1
}

// getRightChildWrite is b3GetRightChildWrite.
func meshGetRightChildWrite(nodes []MeshNode, node int) int {
	// We store the offset of the right child relative to its parent
	assert(!meshIsLeaf(&nodes[node]), "!b3IsLeaf( node )")
	return node + meshNodeChildOffset(&nodes[node])
}

// getRightChild is b3GetRightChild.
func meshGetRightChild(nodes []MeshNode, node int) int {
	// We store the offset of the right child relative to its parent
	assert(!meshIsLeaf(&nodes[node]), "!b3IsLeaf( node )")
	return node + meshNodeChildOffset(&nodes[node])
}

// getRoot is b3GetRoot.
// PORT: returns the node slice and the root index 0; a nil slice is the NULL root.
func meshGetRoot(mesh *MeshData) ([]MeshNode, int) {
	// The first node is the root
	return GetMeshNodes(mesh), 0
}

// getRootWrite is b3GetRootWrite.
func meshGetRootWrite(mesh *MeshData) ([]MeshNode, int) {
	// The first node is the root
	return getMeshNodesWrite(mesh), 0
}

func getMeshTrianglesWrite(mesh *MeshData) []MeshTriangle {
	return GetMeshTriangles(mesh)
}

func getMeshVerticesWrite(mesh *MeshData) []Vec3 {
	return GetMeshVertices(mesh)
}

// static b3Vec3 b3GetVertex( b3MeshData& mesh, int vertexIndex )
//{
//	B3_ASSERT( 0 <= vertexIndex && vertexIndex < mesh.vertexCount );
//	b3Vec3* vertices = b3GetMeshVertices( &mesh );
//	return vertices[vertexIndex];
// }

func getMeshMaterialIndicesWrite(mesh *MeshData) []uint8 {
	return GetMeshMaterialIndices(mesh)
}

func getMeshFlagsWrite(mesh *MeshData) []uint8 {
	return GetMeshFlags(mesh)
}

// getNodeHeight is b3GetNodeHeight.
func meshGetNodeHeight(nodes []MeshNode, node int) int {
	if meshIsLeaf(&nodes[node]) {
		return 0
	}

	leftChild := meshGetLeftChild(nodes, node)
	leftHeight := meshGetNodeHeight(nodes, leftChild)
	rightChild := meshGetRightChild(nodes, node)
	rightHeight := meshGetNodeHeight(nodes, rightChild)

	return 1 + MaxInt(leftHeight, rightHeight)
}

// GetHeight gets the height of the mesh BVH.
func GetHeight(mesh *MeshData) int {
	nodes, root := meshGetRoot(mesh)
	if nodes == nil {
		return 0
	}

	return meshGetNodeHeight(nodes, root)
}

// PORT: the #if B3_ENABLE_VALIDATION == 1 helpers are always compiled; isValidMesh branches on
// validateEnabled and B3_VALIDATE sites skip them when validation is off.

func isDegenerate(v1, v2, v3 Vec3, minArea float32) bool {
	normal := Cross(Sub(v2, v1), Sub(v3, v1))
	lengthSq := LengthSquared(normal)
	return lengthSq < minArea*minArea
}

func isNonDegenerate(mesh *MeshData, minArea float32) bool {
	triangles := GetMeshTriangles(mesh)
	vertices := GetMeshVertices(mesh)

	// Check triangles
	for index := 0; index < int(mesh.TriangleCount); index++ {
		// Index range
		triangle := triangles[index]
		if triangle.Index1 >= mesh.VertexCount {
			return false
		}

		if triangle.Index2 >= mesh.VertexCount {
			return false
		}

		if triangle.Index3 >= mesh.VertexCount {
			return false
		}

		// Degenerate topology
		if triangle.Index1 == triangle.Index2 {
			return false
		}
		if triangle.Index1 == triangle.Index3 {
			return false
		}
		if triangle.Index2 == triangle.Index3 {
			return false
		}

		// Degenerate geometry
		vertex1 := vertices[triangle.Index1]
		vertex2 := vertices[triangle.Index2]
		vertex3 := vertices[triangle.Index3]
		if isDegenerate(vertex1, vertex2, vertex3, minArea) {
			return false
		}
	}

	return true
}

func meshGetNodeAABB(node *MeshNode) AABB {
	return AABB{
		node.LowerBound,
		node.UpperBound,
	}
}

func meshIsConsistent(mesh *MeshData) bool {
	triangles := GetMeshTriangles(mesh)
	vertices := GetMeshVertices(mesh)

	// Check nodes
	count := 0
	var stack [64]int
	nodes, root := meshGetRoot(mesh)
	stack[count] = root
	count++

	for count > 0 {
		count--
		node := stack[count]
		nodeBounds := meshGetNodeAABB(&nodes[node])

		if meshIsLeaf(&nodes[node]) == false {
			child1 := meshGetLeftChild(nodes, node)
			bounds1 := meshGetNodeAABB(&nodes[child1])
			child2 := meshGetRightChild(nodes, node)
			bounds2 := meshGetNodeAABB(&nodes[child2])

			if nodeBounds.Contains(bounds1) == false {
				return false
			}

			if nodeBounds.Contains(bounds2) == false {
				return false
			}

			stack[count] = child2
			count++
			stack[count] = child1
			count++
		} else {
			triangleBounds := bounds3Empty
			for index := 0; index < meshNodeTriangleCount(&nodes[node]); index++ {
				triangleIndex := int(nodes[node].TriangleOffset) + index
				assert(0 <= triangleIndex && triangleIndex < int(mesh.TriangleCount), "0 <= triangleIndex && triangleIndex < mesh->triangleCount")

				triangle := triangles[triangleIndex]

				vertexBounds := bounds3Empty
				vertexBounds = vertexBounds.addPoint(vertices[triangle.Index1])
				vertexBounds = vertexBounds.addPoint(vertices[triangle.Index2])
				vertexBounds = vertexBounds.addPoint(vertices[triangle.Index3])

				triangleBounds = triangleBounds.Union(vertexBounds)
			}

			if !nodeBounds.Contains(triangleBounds) {
				return false
			}
		}
	}

	return true
}

// isValidMesh is b3IsValidMesh.
func isValidMesh(meshData *MeshData) bool {
	if meshData == nil {
		return false
	}

	if meshData.Version != MeshVersion {
		return false
	}

	if meshData.ByteCount < int32(unsafe.Sizeof(MeshData{})) {
		return false
	}

	if validateEnabled {
		return meshIsConsistent(meshData)
	}

	return true
}

// getStridedVertex is b3GetStridedVertex.
// PORT: vertices is the base of the strided vertex data (see MeshDef.Stride); the element is read
// stride bytes apart through unsafe.Add.
func getStridedVertex(vertices []Vec3, index int, stride uintptr) Vec3 {
	return *(*Vec3)(unsafe.Add(unsafe.Pointer(unsafe.SliceData(vertices)), uintptr(index)*stride))
}

// vertexNode is b3VertexNode: node for a vertex linked list.
type vertexNode struct {
	vertexIndex   int32
	nextNodeIndex int32
}

// vertexMap is b3VertexMap.
// PORT: verstable instance; vtHashInteger / vtCmprInteger are HASH_FN / CMPR_FN.
type vertexMap = verstable[uint64, int]

// vtHashInteger is Verstable's vt_hash_integer (a fast integer hash).
func vtHashInteger(key uint64) uint64 {
	key ^= key >> 23
	key *= 0x2127599bf4325c37
	key ^= key >> 47
	return key
}

// vtCmprInteger is Verstable's vt_cmpr_integer.
func vtCmprInteger(key1, key2 uint64) bool {
	return key1 == key2
}

// spatialHash is b3SpatialHash.
type spatialHash struct {
	nodes       []vertexNode
	vertices    []Vec3
	vertexCount int
	stride      uintptr
	vertexMap   vertexMap
	cellSize    float32
	tolerance   float32
}

func createSpatialHash(h *spatialHash, vertices []Vec3, vertexCount int, stride uintptr, tolerance float32) {
	h.vertices = vertices
	h.vertexCount = vertexCount
	h.stride = stride
	h.tolerance = tolerance
	h.cellSize = 2.0 * tolerance
	arrayCreateN(&h.nodes, vertexCount)

	h.vertexMap.init(vtHashInteger, vtCmprInteger)
	h.vertexMap.reserve(vertexCount)

	assert(h.cellSize > 0.0, "h->cellSize > 0.0f")
}

func destroySpatialHash(h *spatialHash) {
	h.vertexMap.cleanup()
	arrayDestroy(&h.nodes)
}

// floorf is the C floorf: exact through float64.
func floorf(x float32) float32 {
	return float32(math.Floor(float64(x)))
}

// spatialHashFindDuplicate is b3SpatialHash_FindDuplicate.
// Welding works by bucketing nearby vertices into identical keys in a hash table.
// Bucketing is done manually with an array.
func spatialHashFindDuplicate(h *spatialHash, currentIndex int32) int32 {
	assert(int(currentIndex) < h.vertexCount, "currentIndex < h->vertexCount")
	stride := h.stride
	vertex := getStridedVertex(h.vertices, int(currentIndex), stride)
	cellSize := h.cellSize
	tolerance := h.tolerance

	// Get the grid coordinates for the current vertex
	baseX := int32(floorf(vertex.X / cellSize))
	baseY := int32(floorf(vertex.Y / cellSize))
	baseZ := int32(floorf(vertex.Z / cellSize))

	// Check the current cell and all 26 neighboring cells (3x3x3 - 1)
	for dx := int32(-1); dx <= 1; dx++ {
		for dy := int32(-1); dy <= 1; dy++ {
			for dz := int32(-1); dz <= 1; dz++ {
				x := baseX + dx
				y := baseY + dy
				z := baseZ + dz

				// Compute hash for this neighboring cell (this is the key in the map)
				var key uint64 = 0
				key ^= uint64(x) + 0x9e3779b9 + (key << 6) + (key >> 2)
				key ^= uint64(y) + 0x9e3779b9 + (key << 6) + (key >> 2)
				key ^= uint64(z) + 0x9e3779b9 + (key << 6) + (key >> 2)

				it := h.vertexMap.get(key)
				if h.vertexMap.isEnd(it) == false {
					// Check all vertices in this key
					nodeIndex := it.data.val

					for nodeIndex != NullIndex {
						node := h.nodes[nodeIndex]

						existingIndex := node.vertexIndex
						assert(existingIndex < currentIndex, "existingIndex < currentIndex")
						assert(int(existingIndex) < h.vertexCount, "existingIndex < h->vertexCount")

						other := getStridedVertex(h.vertices, int(existingIndex), stride)

						// IsEqual inlined: check if vertices are within tolerance
						if AbsFloat(vertex.X-other.X) <= tolerance && AbsFloat(vertex.Y-other.Y) <= tolerance &&
							AbsFloat(vertex.Z-other.Z) <= tolerance {
							// Found duplicate
							return existingIndex
						}

						nodeIndex = int(node.nextNodeIndex)
					}
				}
			}
		}
	}

	// No duplicate found, add to hash table
	var currentKey uint64 = 0
	currentKey ^= uint64(baseX) + 0x9e3779b9 + (currentKey << 6) + (currentKey >> 2)
	currentKey ^= uint64(baseY) + 0x9e3779b9 + (currentKey << 6) + (currentKey >> 2)
	currentKey ^= uint64(baseZ) + 0x9e3779b9 + (currentKey << 6) + (currentKey >> 2)

	it := h.vertexMap.get(currentKey)
	if h.vertexMap.isEnd(it) == false {
		nodeIndex := it.data.val

		node := vertexNode{
			vertexIndex:   currentIndex,
			nextNodeIndex: int32(nodeIndex),
		}

		it.data.val = len(h.nodes)
		arrayPush(&h.nodes, node)
	} else {
		node := vertexNode{
			vertexIndex:   currentIndex,
			nextNodeIndex: NullIndex,
		}

		h.vertexMap.insert(currentKey, len(h.nodes))
		arrayPush(&h.nodes, node)
	}

	// Not welded
	return NullIndex
}

// weldData is b3WeldData.
type weldData struct {
	srcVertices []Vec3
	srcIndices  []int32

	dstVertices []Vec3
	dstIndices  []int32

	vertexCount int
	indexCount  int
	stride      uintptr
}

// weldVertices is b3WeldVertices.
func weldVertices(data *weldData, tolerance float32) int {
	vertexCount := data.vertexCount
	uniqueCount := 0
	stride := data.stride

	// Create spatial hash and find duplicates
	var spatialHash spatialHash
	createSpatialHash(&spatialHash, data.srcVertices, vertexCount, stride, tolerance)
	var vertexMapping []int32
	arrayResize(&vertexMapping, vertexCount)

	for i := range vertexCount {
		duplicateIndex := spatialHashFindDuplicate(&spatialHash, int32(i))

		if duplicateIndex == NullIndex {
			// New unique vertex
			vertexMapping[i] = int32(uniqueCount)
			data.dstVertices[uniqueCount] = getStridedVertex(data.srcVertices, i, stride)
			uniqueCount += 1
		} else {
			// Found duplicate, map to existing vertex
			vertexMapping[i] = vertexMapping[duplicateIndex]
		}
	}

	// Update indices to reference the new vertex array
	indexCount := data.indexCount
	for i := range indexCount {
		srcIndex := int(data.srcIndices[i])
		assert(0 <= srcIndex && srcIndex < vertexCount, "0 <= srcIndex && srcIndex < vertexCount")
		data.dstIndices[i] = vertexMapping[srcIndex]
	}

	destroySpatialHash(&spatialHash)
	arrayDestroy(&vertexMapping)

	return uniqueCount
}

// storeLeaf is b3StoreLeaf.
func storeLeaf(node *MeshNode, aabb *AABB, triangleCount int, triangleOffset int) {
	setMeshNodeLow(node, meshLeafNode)
	setMeshNodeHigh(node, triangleCount)
	node.TriangleOffset = uint32(triangleOffset)
	node.LowerBound = aabb.LowerBound
	node.UpperBound = aabb.UpperBound
}

// primitive is b3Primitive.
type primitive struct {
	aabb          AABB
	center        Vec3
	triangleIndex int32
}

// bucket is b3Bucket.
type bucket struct {
	count  int
	bounds AABB
}

// split is b3Split.
type split struct {
	leftBounds  AABB
	rightBounds AABB
	axis        int
	index       int
}

// splitBinnedSah is b3SplitBinnedSah.
func splitBinnedSah(count int, primitives []primitive) split {
	var split split
	split.axis = -1
	split.index = -1

	// Compute bounds of primitive centroids and choose split axis
	bounds := AABB{primitives[0].center, primitives[0].center}
	for i := 1; i < count; i++ {
		bounds = bounds.addPoint(primitives[i].center)
	}

	// Compute costs for splitting after each bucket and keep track of best split
	// This is a small O(n^2) loop. This can be further optimized, but it is already
	// very fast and is kept for simplicity right now.
	bestBucket := -1
	bestCost := fltMax

	for axis := range 3 {
		extent := bounds.Extents()
		if getByIndex(extent, axis) < LinearSlop() {
			continue
		}

		// Initialize buckets
		var buckets [binCount]bucket
		for i := range binCount {
			buckets[i].count = 0
			buckets[i].bounds = bounds3Empty
		}

		// Fill buckets
		factor := binCount * (1.0 - fltEpsilon) /
			(getByIndex(bounds.UpperBound, axis) - getByIndex(bounds.LowerBound, axis))
		for i := range count {
			center := primitives[i].center
			index := int(factor * (getByIndex(center, axis) - getByIndex(bounds.LowerBound, axis)))
			assert(0 <= index && index < binCount, "0 <= index && index < B3_BIN_COUNT")

			buckets[index].count++
			buckets[index].bounds = buckets[index].bounds.Union(primitives[i].aabb)
		}

		// Evaluate splits
		for i := range binCount - 1 {
			leftCount := 0
			leftBounds := bounds3Empty
			for k := 0; k <= i; k++ {
				leftCount += buckets[k].count
				leftBounds = leftBounds.Union(buckets[k].bounds)
			}

			rightCount := 0
			rightBounds := bounds3Empty
			for k := i + 1; k < binCount; k++ {
				rightCount += buckets[k].count
				rightBounds = rightBounds.Union(buckets[k].bounds)
			}

			assert(leftCount+rightCount == count, "leftCount + rightCount == count")
			if leftCount > 0 && rightCount > 0 {
				cost := float32(float32(leftCount)*leftBounds.Area()) + float32(float32(rightCount)*rightBounds.Area())

				if cost < bestCost {
					bestBucket = i
					bestCost = cost

					split.axis = axis
					split.index = leftCount
					split.leftBounds = leftBounds
					split.rightBounds = rightBounds
				}
			}
		}
	}

	// Partition
	if bestBucket >= 0 {
		axis := split.axis
		factor := binCount * (1.0 - fltEpsilon) /
			(getByIndex(bounds.UpperBound, axis) - getByIndex(bounds.LowerBound, axis))

		splitIndex := 0
		for i := range count {
			center := primitives[i].center
			index := int(factor * (getByIndex(center, axis) - getByIndex(bounds.LowerBound, axis)))

			if index <= bestBucket {
				primitives[i], primitives[splitIndex] = primitives[splitIndex], primitives[i]
				splitIndex++
			}
		}
		assert(splitIndex == split.index, "splitIndex == split.index")
	}

	return split
}

// splitHalf is b3SplitHalf.
func splitHalf(count int, primitives []primitive) split {
	// Split in the middle
	splitIndex := count / 2

	leftBounds := bounds3Empty
	for i := range splitIndex {
		leftBounds = leftBounds.Union(primitives[i].aabb)
	}

	rightBounds := bounds3Empty
	for i := splitIndex; i < count; i++ {

		rightBounds = rightBounds.Union(primitives[i].aabb)
	}

	bounds := leftBounds.Union(rightBounds)
	axis := majorAxis(bounds.Extents())

	var split split
	split.axis = axis
	split.index = splitIndex
	split.leftBounds = leftBounds
	split.rightBounds = rightBounds

	return split
}

// splitMedian is b3SplitMedian.
func splitMedian(count int, primitives []primitive) split {
	assert(count > 2, "count > 2")

	lowerBound := primitives[0].center
	upperBound := primitives[0].center

	for i := 1; i < count; i++ {
		lowerBound = Min(lowerBound, primitives[i].center)
		upperBound = Max(upperBound, primitives[i].center)
	}

	d := Sub(upperBound, lowerBound)
	c := MulSV(0.5, Add(lowerBound, upperBound))

	split := split{}
	split.index = -1

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	i1, i2 := 0, count
	if d.X >= d.Y && d.X >= d.Z {
		split.axis = 0

		pivot := c.X

		for i1 < i2 {
			for i1 < i2 && primitives[i1].center.X < pivot {
				i1 += 1
			}

			for i1 < i2 && primitives[i2-1].center.X >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap primitives
				primitives[i1], primitives[i2-1] = primitives[i2-1], primitives[i1]

				i1 += 1
				i2 -= 1
			}
		}
	} else if d.Y >= d.Z {
		split.axis = 1

		pivot := c.Y

		for i1 < i2 {
			for i1 < i2 && primitives[i1].center.Y < pivot {
				i1 += 1
			}

			for i1 < i2 && primitives[i2-1].center.Y >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap primitives
				primitives[i1], primitives[i2-1] = primitives[i2-1], primitives[i1]

				i1 += 1
				i2 -= 1
			}
		}
	} else {
		split.axis = 2

		pivot := c.Z

		for i1 < i2 {
			for i1 < i2 && primitives[i1].center.Z < pivot {
				i1 += 1
			}

			for i1 < i2 && primitives[i2-1].center.Z >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap primitives
				primitives[i1], primitives[i2-1] = primitives[i2-1], primitives[i1]

				i1 += 1
				i2 -= 1
			}
		}
	}
	assert(i1 == i2, "i1 == i2")
	assert(0 <= i1 && i1 < count, "0 <= i1 && i1 < count")

	if i1 == 0 || i1 == count-1 {
		// failed to split
		i1 = count / 2
	}

	leftBounds := bounds3Empty
	for i := range i1 {
		leftBounds = leftBounds.Union(primitives[i].aabb)
	}

	rightBounds := bounds3Empty
	for i := i1; i < count; i++ {
		rightBounds = rightBounds.Union(primitives[i].aabb)
	}

	split.index = i1
	split.leftBounds = leftBounds
	split.rightBounds = rightBounds
	return split
}

// validateSplit is b3ValidateSplit (B3_ENABLE_VALIDATION only upstream).
func validateSplit(count int, primitives []primitive, split *split) bool {
	if split.axis < 0 {
		return false
	}

	for i := range split.index {
		if !split.leftBounds.Contains(primitives[i].aabb) {
			return false
		}
	}

	for i := split.index; i < count; i++ {
		if !split.rightBounds.Contains(primitives[i].aabb) {
			return false
		}
	}

	return true
}

// buildRecursive is b3BuildRecursive.
// PORT: primitives is the sub-slice starting at the C pointer; baseOffset is primitives - base.
func buildRecursive(nodes *[]MeshNode, count int, primitives []primitive, baseOffset int,
	useMedianSplit bool, height *int) int {
	if count > desiredTrianglesPerLeaf {
		// Try to split the input set using the SAH
		var split split
		if useMedianSplit {
			split = splitMedian(count, primitives)
		} else {
			split = splitBinnedSah(count, primitives)
		}

		if split.axis < 0 {
			if count > maximumTrianglesPerLeaf {
				// Re-split. This is a less optimal split and can create more false positives!
				split = splitHalf(count, primitives)
			} else {
				bounds := bounds3Empty
				for i := range count {
					bounds = bounds.Union(primitives[i].aabb)
				}

				// We have only a few triangles left. Create a leaf.
				index := arrayAddIndex(nodes)
				storeLeaf(&(*nodes)[index], &bounds, count, baseOffset)

				return index
			}
		}
		validate(!validateEnabled || validateSplit(count, primitives, &split), "b3ValidateSplit( count, primitives, &split )")

		// Allocate node and recurse
		index := arrayAddIndex(nodes)
		heightLeft, heightRight := 0, 0
		leftIndex := buildRecursive(nodes, split.index, primitives, baseOffset, useMedianSplit, &heightLeft)
		rightIndex := buildRecursive(nodes, count-split.index, primitives[split.index:], baseOffset+split.index,
			useMedianSplit, &heightRight)

		*height = MaxInt(heightLeft, heightRight) + 1

		assert(leftIndex-index == 1 && rightIndex-index > 1, "leftIndex - index == 1 && rightIndex - index > 1")

		aabb := split.leftBounds.Union(split.rightBounds)
		node := arrayGet(*nodes, index)
		setMeshNodeLow(node, split.axis)
		setMeshNodeHigh(node, rightIndex-index)
		node.LowerBound = aabb.LowerBound
		node.UpperBound = aabb.UpperBound

		// Zero so mesh->hash is deterministic
		node.TriangleOffset = 0

		return index
	}

	aabb := bounds3Empty
	for i := range count {
		aabb = aabb.Union(primitives[i].aabb)
	}

	index := arrayAddIndex(nodes)
	storeLeaf(&(*nodes)[index], &aabb, count, baseOffset)

	*height = 1

	return index
}

// sortMeshTriangles is b3SortMeshTriangles.
func sortMeshTriangles(mesh *MeshData) bool {
	triangles := getMeshTrianglesWrite(mesh)
	materialIndices := getMeshMaterialIndicesWrite(mesh)

	// Sort triangles in depth-first-order
	offset := 0
	var tempTriangles []MeshTriangle
	arrayCreateN(&tempTriangles, int(mesh.TriangleCount))

	var tempMaterialIndices []uint8
	arrayCreateN(&tempMaterialIndices, int(mesh.TriangleCount))

	count := 0
	var stack [meshStackSize]int
	nodes, root := meshGetRootWrite(mesh)
	stack[count] = root
	count++

	for count > 0 {
		count--
		node := stack[count]

		if meshIsLeaf(&nodes[node]) == false {
			if count >= meshStackSize-2 {
				return false
			}

			stack[count] = meshGetRightChildWrite(nodes, node)
			count++
			stack[count] = meshGetLeftChildWrite(nodes, node)
			count++
		} else {
			triangleCount := meshNodeTriangleCount(&nodes[node])
			triangleOffset := int(nodes[node].TriangleOffset)

			for triangle := range triangleCount {
				index := triangleOffset + triangle
				arrayPush(&tempTriangles, triangles[index])
				arrayPush(&tempMaterialIndices, materialIndices[index])
			}

			nodes[node].TriangleOffset = uint32(offset)
			offset += triangleCount
		}
	}

	assert(offset == len(tempTriangles), "offset == tempTriangles.count")
	assert(len(tempTriangles) == int(mesh.TriangleCount), "tempTriangles.count == mesh->triangleCount")
	assert(len(tempMaterialIndices) == int(mesh.TriangleCount), "tempMaterialIndices.count == mesh->triangleCount")

	// Copy sorted triangle array back to tree
	copy(triangles, tempTriangles[:mesh.TriangleCount])
	copy(materialIndices, tempMaterialIndices[:mesh.TriangleCount])

	arrayDestroy(&tempTriangles)
	arrayDestroy(&tempMaterialIndices)

	return true
}

// meshEdge is the anonymous b3MeshEdge struct.
type meshEdge struct {
	vertex1       int
	vertex2       int
	triangle1     int
	triangle2     int
	triangleCount uint16

	// The index of an edge within the parent triangle: 0, 1, or 2. 0xFF is unset
	triangleEdgeIndex1 uint8
	triangleEdgeIndex2 uint8
}

// edgeMap is b3EdgeMap.
type edgeMap = verstable[uint64, int]

// PORT: the #if 0 b3TestHash block is not ported.

// Results for MeshCreationBenchmark
// eastl::hash_map : 4.9703 ms and 5.1445ms with FastHash function
// std::unordered_map : 4.8755 ms and 4.4780ms with FastHash function
// verstable : 3.3968 ms with default FastHash
// no edge identification : 1.7396 ms
func identifyEdges(mesh *MeshData) {
	triangles := getMeshTrianglesWrite(mesh)
	vertices := GetMeshVertices(mesh)
	flags := getMeshFlagsWrite(mesh)

	triangleCount := int(mesh.TriangleCount)
	edgeCount := 3 * triangleCount
	edges := make([]meshEdge, edgeCount)
	normals := make([]Vec3, triangleCount)

	for i := range triangleCount {
		triangle := &triangles[i]
		i1 := int(triangle.Index1)
		i2 := int(triangle.Index2)
		i3 := int(triangle.Index3)

		edges[3*i+0].vertex1 = MinInt(i1, i2)
		edges[3*i+0].vertex2 = MaxInt(i1, i2)
		edges[3*i+0].triangle1 = i
		edges[3*i+0].triangle2 = NullIndex
		edges[3*i+0].triangleEdgeIndex1 = 0
		edges[3*i+0].triangleEdgeIndex2 = 0xFF
		edges[3*i+0].triangleCount = 1

		edges[3*i+1].vertex1 = MinInt(i2, i3)
		edges[3*i+1].vertex2 = MaxInt(i2, i3)
		edges[3*i+1].triangle1 = i
		edges[3*i+1].triangle2 = NullIndex
		edges[3*i+1].triangleEdgeIndex1 = 1
		edges[3*i+1].triangleEdgeIndex2 = 0xFF
		edges[3*i+1].triangleCount = 1

		edges[3*i+2].vertex1 = MinInt(i3, i1)
		edges[3*i+2].vertex2 = MaxInt(i3, i1)
		edges[3*i+2].triangle1 = i
		edges[3*i+2].triangle2 = NullIndex
		edges[3*i+2].triangleEdgeIndex1 = 2
		edges[3*i+2].triangleEdgeIndex2 = 0xFF
		edges[3*i+2].triangleCount = 1

		v1 := vertices[i1]
		v2 := vertices[i2]
		v3 := vertices[i3]

		e1 := Sub(v2, v1)
		e2 := Sub(v3, v1)
		n := Cross(e1, e2)

		normals[i] = Normalize(n)
	}

	var m edgeMap
	m.init(vtHashInteger, vtCmprInteger)
	m.reserve(edgeCount)

	key := uint64(edges[0].vertex1)<<32 | uint64(edges[0].vertex2)
	m.insert(key, 0)

	// Find unique edges and assign adjacency
	for i := 1; i < edgeCount; i++ {
		edge := &edges[i]
		key = uint64(edge.vertex1)<<32 | uint64(edge.vertex2)
		itr := m.get(key)

		if m.isEnd(itr) {
			m.insert(key, i)
		} else {
			otherIndex := itr.data.val
			assert(otherIndex < i, "otherIndex < i")

			base := &edges[otherIndex]
			if base.triangleCount == 1 {
				base.triangle2 = edge.triangle1
				base.triangleEdgeIndex2 = edge.triangleEdgeIndex1
			}

			base.triangleCount += 1
		}
	}

	m.cleanup()

	for i := range edgeCount {
		edge := &edges[i]
		if edge.triangleCount != 2 {
			continue
		}

		assert(edge.triangleEdgeIndex1 < 3, "edge->triangleEdgeIndex1 < 3")
		assert(edge.triangleEdgeIndex2 < 3, "edge->triangleEdgeIndex2 < 3")

		triangle1 := &triangles[edge.triangle1]
		triangle2 := &triangles[edge.triangle2]
		flag1 := &flags[edge.triangle1]
		flag2 := &flags[edge.triangle2]

		j1 := int(triangle2.Index1)
		j2 := int(triangle2.Index2)
		j3 := int(triangle2.Index3)

		opposite := NullIndex

		switch edge.triangleEdgeIndex2 {
		case 0:
			opposite = j3

		case 1:
			opposite = j1

		case 2:
			opposite = j2

		default:
			assert(false, "false")
		}

		i1 := triangle1.Index1
		i2 := triangle1.Index2
		i3 := triangle1.Index3

		v1 := vertices[i1]
		v2 := vertices[i2]
		v3 := vertices[i3]
		p := vertices[opposite]

		var cos5Deg float32 = 0.9962
		signedVolume := signedVolume(v1, v2, v3, p)
		n1 := normals[edge.triangle1]
		n2 := normals[edge.triangle2]
		cosAngle := Dot(n1, n2)
		if signedVolume > 0.0 || cosAngle > cos5Deg {
			edgeFlags := [3]MeshEdgeFlags{ConcaveEdge1, ConcaveEdge2, ConcaveEdge3}
			*flag1 |= uint8(edgeFlags[edge.triangleEdgeIndex1])
			*flag2 |= uint8(edgeFlags[edge.triangleEdgeIndex2])
		}

		if signedVolume < 0.0 || cosAngle > cos5Deg {
			edgeFlags := [3]MeshEdgeFlags{InverseConcaveEdge1, InverseConcaveEdge2, InverseConcaveEdge3}
			*flag1 |= uint8(edgeFlags[edge.triangleEdgeIndex1])
			*flag2 |= uint8(edgeFlags[edge.triangleEdgeIndex2])
		}
	}
}

// CreateGridMesh creates a grid mesh along the x and z axes.
// xCount: the number of rows in the x direction
// zCount: the number of rows in the z direction
// cellWidth: the width of each cell
// materialCount: the number of materials to generate
// identifyEdges: compute adjacency information
func CreateGridMesh(xCount, zCount int, cellWidth float32, materialCount int, identifyEdges bool) *MeshData {
	assert(0 <= materialCount && materialCount <= math.MaxUint8, "0 <= materialCount && materialCount <= UINT8_MAX")

	// Create vertices
	vertexCount := (xCount + 1) * (zCount + 1)

	var vertices []Vec3
	arrayResize(&vertices, vertexCount)
	index := 0

	xWidth := cellWidth * float32(xCount)
	zWidth := cellWidth * float32(zCount)

	x := float32(-0.5 * xWidth)
	for ix := 0; ix <= xCount; ix++ {
		z := float32(-0.5 * zWidth)
		for iz := 0; iz <= zCount; iz++ {
			vertices[index] = Vec3{x, 0.0, z}
			z += cellWidth
			index += 1
		}
		x += cellWidth
	}
	assert(index == vertexCount, "index == vertexCount")

	// Triangles
	triangleCount := 2 * xCount * zCount

	var indices []int32
	arrayResize(&indices, 3*triangleCount)

	var materialIndices []uint8
	arrayResize(&materialIndices, triangleCount)

	materialIndex := 0
	index = 0
	for ix := range xCount {
		for iz := range zCount {
			index1 := iz + (zCount+1)*ix
			index2 := index1 + 1
			index3 := index2 + (zCount + 1)
			index4 := index3 - 1

			assert(index1 < vertexCount, "index1 < vertexCount")
			assert(index2 < vertexCount, "index2 < vertexCount")
			assert(index3 < vertexCount, "index3 < vertexCount")
			assert(index4 < vertexCount, "index4 < vertexCount")

			indices[index+0] = int32(index1)
			indices[index+1] = int32(index2)
			indices[index+2] = int32(index3)

			indices[index+3] = int32(index3)
			indices[index+4] = int32(index4)
			indices[index+5] = int32(index1)

			if materialCount > 0 {
				materialIndices[2*materialIndex+0] = uint8(materialIndex % materialCount)
				materialIndices[2*materialIndex+1] = uint8(materialIndex % materialCount)
			}

			materialIndex += 1
			index += 6
		}
	}
	assert(index == 3*triangleCount, "index == 3 * triangleCount")

	def := MeshDef{}
	def.VertexCount = int32(len(vertices))
	def.Vertices = vertices
	def.TriangleCount = int32(len(indices) / 3)
	def.Indices = indices
	if materialCount > 0 {
		def.MaterialIndices = materialIndices
	}
	def.UseMedianSplit = true
	def.IdentifyEdges = identifyEdges

	meshData := CreateMesh(&def, nil)

	arrayDestroy(&indices)
	arrayDestroy(&vertices)
	arrayDestroy(&materialIndices)

	return meshData
}

// CreateWaveMesh creates a wave mesh along the x and z axes.
func CreateWaveMesh(xCount, zCount int, cellWidth, amplitude, rowFrequency, columnFrequency float32) *MeshData {
	// Create vertices
	vertexCount := (xCount + 1) * (zCount + 1)

	var vertices []Vec3
	arrayResize(&vertices, vertexCount)
	index := 0

	xWidth := cellWidth * float32(xCount)
	zWidth := cellWidth * float32(zCount)

	omegaZ := 2.0 * Pi * rowFrequency * cellWidth
	omegaX := 2.0 * Pi * columnFrequency * cellWidth

	x := float32(-0.5 * xWidth)
	cs := CosSin{}
	for ix := 0; ix <= xCount; ix++ {
		cs = ComputeCosSin(omegaX * float32(ix))
		rowHeight := cs.Sine

		z := float32(-0.5 * zWidth)
		for iz := 0; iz <= zCount; iz++ {
			cs = ComputeCosSin(omegaZ * float32(iz))
			columnHeight := cs.Sine

			y := amplitude * rowHeight * columnHeight
			vertices[index] = Vec3{x, y, z}
			z += cellWidth
			index += 1
		}
		x += cellWidth
	}
	assert(index == vertexCount, "index == vertexCount")

	// Triangles
	triangleCount := 2 * xCount * zCount

	var indices []int32
	arrayResize(&indices, 3*triangleCount)

	index = 0
	for ix := range xCount {
		for iz := range zCount {
			index1 := iz + (zCount+1)*ix
			index2 := index1 + 1
			index3 := index2 + (zCount + 1)
			index4 := index3 - 1

			assert(index1 < vertexCount, "index1 < vertexCount")
			assert(index2 < vertexCount, "index2 < vertexCount")
			assert(index3 < vertexCount, "index3 < vertexCount")
			assert(index4 < vertexCount, "index4 < vertexCount")

			indices[index+0] = int32(index1)
			indices[index+1] = int32(index2)
			indices[index+2] = int32(index3)

			indices[index+3] = int32(index3)
			indices[index+4] = int32(index4)
			indices[index+5] = int32(index1)

			index += 6
		}
	}
	assert(index == 3*triangleCount, "index == 3 * triangleCount")

	def := MeshDef{}
	def.VertexCount = int32(len(vertices))
	def.Vertices = vertices
	def.TriangleCount = int32(len(indices) / 3)
	def.Indices = indices
	def.UseMedianSplit = true
	def.IdentifyEdges = true

	meshData := CreateMesh(&def, nil)

	arrayDestroy(&indices)
	arrayDestroy(&vertices)

	return meshData
}

// CreateTorusMesh creates a torus mesh.
func CreateTorusMesh(radialResolution, tubularResolution int, radius, thickness float32) *MeshData {
	// Create vertices
	var vertices []Vec3

	for radialIndex := range radialResolution {
		for tubularIndex := range tubularResolution {
			u := float32(tubularIndex) / float32(tubularResolution) * twoPi
			v := float32(radialIndex) / float32(radialResolution) * twoPi

			x := (radius + float32(thickness*Cos(v))) * Cos(u)
			y := (radius + float32(thickness*Cos(v))) * Sin(u)
			z := thickness * Sin(v)

			vertex := Vec3{x, y, z}
			arrayPush(&vertices, vertex)
		}
	}

	// Triangles
	var indices []int32
	for radialIndex1 := range radialResolution {
		radialIndex2 := (radialIndex1 + 1) % radialResolution
		for tubularIndex1 := range tubularResolution {
			tubularIndex2 := (tubularIndex1 + 1) % tubularResolution
			index1 := radialIndex1*tubularResolution + tubularIndex1
			index2 := radialIndex1*tubularResolution + tubularIndex2
			index3 := radialIndex2*tubularResolution + tubularIndex2
			index4 := radialIndex2*tubularResolution + tubularIndex1

			arrayPush(&indices, int32(index1))
			arrayPush(&indices, int32(index2))
			arrayPush(&indices, int32(index3))

			arrayPush(&indices, int32(index3))
			arrayPush(&indices, int32(index4))
			arrayPush(&indices, int32(index1))
		}
	}

	def := MeshDef{}
	def.VertexCount = int32(len(vertices))
	def.Vertices = vertices
	def.TriangleCount = int32(len(indices) / 3)
	def.Indices = indices
	def.UseMedianSplit = false
	def.IdentifyEdges = true

	meshData := CreateMesh(&def, nil)

	arrayDestroy(&vertices)
	arrayDestroy(&indices)
	return meshData
}

// CreateBoxMesh creates a box mesh.
func CreateBoxMesh(center, extent Vec3, identifyEdges bool) *MeshData {
	x := extent.X
	y := extent.Y
	z := extent.Z
	vertices := [...]Vec3{
		{x, y, z}, {-x, y, z}, {-x, -y, z}, {x, -y, z}, {x, y, -z}, {-x, y, -z}, {-x, -y, -z}, {x, -y, -z},
	}

	for i := range 8 {
		vertices[i] = Add(vertices[i], center)
	}

	indices := [...]int32{
		0, 1, 3, 1, 2, 3, // front
		0, 4, 1, 1, 4, 5, // top
		0, 3, 7, 4, 0, 7, // right
		4, 7, 5, 6, 5, 7, // back
		1, 5, 2, 6, 2, 5, // left
		3, 2, 7, 6, 7, 2, // bottom
	}

	def := MeshDef{}
	def.VertexCount = 8
	def.Vertices = vertices[:]
	def.TriangleCount = 12
	def.Indices = indices[:]
	def.UseMedianSplit = false
	def.IdentifyEdges = identifyEdges

	return CreateMesh(&def, nil)
}

// CreateHollowBoxMesh creates a hollow box mesh.
func CreateHollowBoxMesh(center, extent Vec3) *MeshData {
	x := extent.X
	y := extent.Y
	z := extent.Z
	vertices := [...]Vec3{
		{x, y, z}, {-x, y, z}, {-x, -y, z}, {x, -y, z}, {x, y, -z}, {-x, y, -z}, {-x, -y, -z}, {x, -y, -z},
	}

	for i := range 8 {
		vertices[i] = Add(vertices[i], center)
	}

	indices := [...]int32{
		3, 1, 0, 3, 2, 1, // front
		1, 4, 0, 5, 4, 1, // top
		7, 3, 0, 7, 0, 4, // right
		5, 7, 4, 7, 5, 6, // back
		2, 5, 1, 5, 2, 6, // left
		7, 2, 3, 2, 7, 6, // bottom
	}

	def := MeshDef{}
	def.VertexCount = 8
	def.Vertices = vertices[:]
	def.TriangleCount = 12
	def.Indices = indices[:]
	def.UseMedianSplit = false
	def.IdentifyEdges = true

	return CreateMesh(&def, nil)
}

// CreatePlatformMesh creates a platform mesh. A truncated pyramid.
func CreatePlatformMesh(center Vec3, height, topWidth, bottomWidth float32) *MeshData {
	hb := float32(0.5 * bottomWidth)
	ht := float32(0.5 * topWidth)
	hy := float32(0.5 * height)
	vertices := [...]Vec3{
		{ht, hy, ht}, {-ht, hy, ht}, {-hb, -hy, hb}, {hb, -hy, hb},
		{ht, hy, -ht}, {-ht, hy, -ht}, {-hb, -hy, -hb}, {hb, -hy, -hb},
	}

	for i := range 8 {
		vertices[i] = Add(vertices[i], center)
	}

	indices := [...]int32{
		0, 1, 3, 1, 2, 3, // front
		0, 4, 1, 1, 4, 5, // top
		0, 3, 7, 4, 0, 7, // right
		4, 7, 5, 6, 5, 7, // back
		1, 5, 2, 6, 2, 5, // left
		3, 2, 7, 6, 7, 2, // bottom
	}

	def := MeshDef{}
	def.VertexCount = 8
	def.Vertices = vertices[:]
	def.TriangleCount = 12
	def.Indices = indices[:]
	def.UseMedianSplit = true
	def.IdentifyEdges = true

	return CreateMesh(&def, nil)
}

// copyVerticesWithStride is b3CopyVerticesWithStride.
func copyVerticesWithStride(dst []Vec3, src []Vec3, count int, stride uintptr) {
	if stride == unsafe.Sizeof(Vec3{}) {
		copy(dst[:count], src[:count])
		return
	}

	for i := range count {
		dst[i] = getStridedVertex(src, i, stride)
	}
}

// maxStride is B3_MAX_STRIDE.
// Stride larger than this likely indicates stride is uninitialized memory.
const maxStride = 4096

// CreateMesh creates a generic mesh.
// PORT: b3CreateMesh( def, degenerateTriangleIndices, degenerateCapacity ) takes a slice; its length
// is the capacity (nil for none).
// todo this should fail if the mesh has a height greater than B3_MESH_STACK_SIZE
func CreateMesh(def *MeshDef, degenerateTriangleIndices []int32) *MeshData {
	return createMesh(def, degenerateTriangleIndices, false)
}

// CreateMeshPreserveSmallTriangles retains input triangles with nonzero float32 area,
// including faces below the ordinary solver-slop-based cooking threshold.
// Zero-area triangles are still omitted; DegenerateCount follows the native
// convention and counts only omitted triangles with three distinct indices.
// Vertex welding, if requested, happens before this classification.
func CreateMeshPreserveSmallTriangles(def *MeshDef, degenerateTriangleIndices []int32) *MeshData {
	return createMesh(def, degenerateTriangleIndices, true)
}

func createMesh(def *MeshDef, degenerateTriangleIndices []int32, preserveSmallTriangles bool) *MeshData {
	degenerateCapacity := len(degenerateTriangleIndices)

	vec3Size := int(unsafe.Sizeof(Vec3{}))
	if def.Stride != 0 && (def.Stride < vec3Size || maxStride < def.Stride) {
		return nil
	}

	if (def.Stride & 3) != 0 {
		return nil
	}

	if def.VertexCount < 3 || def.Vertices == nil || def.TriangleCount <= 0 || def.Indices == nil {
		return nil
	}

	triangleCount := int(def.TriangleCount)
	if triangleCount == 0 {
		return nil
	}

	vertexCount := int(def.VertexCount)
	stride := uintptr(vec3Size)
	if def.Stride != 0 {
		stride = uintptr(def.Stride)
	}

	// PORT: the strided reads go through unsafe.Add, so the vertex slice must span them.
	assert(uintptr(len(def.Vertices))*uintptr(vec3Size) >= uintptr(vertexCount-1)*stride+uintptr(vec3Size),
		"def.Vertices spans vertexCount strided vertices")

	meshBounds := bounds3Empty

	// Clone indices and vertices to support welding
	var indices []int32
	arrayCreateN(&indices, 3*triangleCount)

	var vertices []Vec3
	arrayCreateN(&vertices, vertexCount)

	if def.WeldVertices && def.WeldTolerance > 0.0 {
		arrayResize(&vertices, vertexCount)
		arrayResize(&indices, 3*triangleCount)

		data := weldData{
			srcVertices: def.Vertices,
			srcIndices:  def.Indices,
			dstVertices: vertices,
			dstIndices:  indices,
			vertexCount: vertexCount,
			indexCount:  3 * triangleCount,
			stride:      stride,
		}

		vertices = vertices[:weldVertices(&data, def.WeldTolerance)]
		vertexCount = len(vertices)
		assert(vertexCount <= int(def.VertexCount), "vertexCount <= def->vertexCount")
	} else {
		vertices = vertices[:vertexCount]
		copyVerticesWithStride(vertices, def.Vertices, vertexCount, stride)
		arrayAppend(&indices, def.Indices[:3*triangleCount])
	}

	var primitives []primitive
	arrayCreateN(&primitives, triangleCount)
	degenerateCount := 0
	minArea := 0.01 * LinearSlop() * LinearSlop()
	var surfaceArea float32 = 0.0
	materialCount := 1
	clockWise := def.ClockWiseWinding

	for index := range triangleCount {
		index1 := indices[3*index+0]
		index2 := indices[3*index+1]
		index3 := indices[3*index+2]

		assert(0 <= index1 && int(index1) < vertexCount, "0 <= index1 && index1 < vertexCount")
		assert(0 <= index2 && int(index2) < vertexCount, "0 <= index2 && index2 < vertexCount")
		assert(0 <= index3 && int(index3) < vertexCount, "0 <= index3 && index3 < vertexCount")

		vertex1 := vertices[index1]
		vertex2 := vertices[index2]
		vertex3 := vertices[index3]

		normal := Cross(Sub(vertex2, vertex1), Sub(vertex3, vertex1))
		area := float32(0.5 * Length(normal))

		if area == 0 || (!preserveSmallTriangles && area < minArea) {
			// b3Log( "degenerate: %d %d %d\n", index1, index2, index3 );

			if index1 != index2 && index1 != index3 && index2 != index3 {
				if degenerateTriangleIndices != nil && degenerateCount < degenerateCapacity {
					degenerateTriangleIndices[degenerateCount] = int32(index)
				}

				degenerateCount += 1
			}

			continue
		}

		surfaceArea += area

		box := AABB{
			LowerBound: Min(vertex1, Min(vertex2, vertex3)),
			UpperBound: Max(vertex1, Max(vertex2, vertex3)),
		}

		center := box.Center()

		primitive := primitive{
			aabb:          box,
			center:        center,
			triangleIndex: int32(index),
		}
		arrayPush(&primitives, primitive)

		if def.MaterialIndices != nil {
			materialCount = MaxInt(materialCount, int(def.MaterialIndices[index])+1)
		}

		meshBounds = meshBounds.Union(box)
	}

	// Update triangle count due to degenerates being skipped
	triangleCount = len(primitives)

	if IsSaneAABB(meshBounds) == false {
		arrayDestroy(&primitives)
		arrayDestroy(&indices)
		arrayDestroy(&vertices)
		return nil
	}

	// Build the tree (this reorders the builder triangles)
	var tempNodes []MeshNode
	arrayCreateN(&tempNodes, 2*triangleCount-1)

	treeHeight := 0
	buildRecursive(&tempNodes, triangleCount, primitives, 0, def.UseMedianSplit, &treeHeight)

	// Allocate the mesh
	byteCount := alignUp8(int(unsafe.Sizeof(MeshData{})))
	nodeOffset := byteCount
	byteCount += alignUp8(len(tempNodes) * int(unsafe.Sizeof(MeshNode{})))
	vertexOffset := byteCount
	byteCount += alignUp8(vertexCount * int(unsafe.Sizeof(Vec3{})))
	triangleOffset := byteCount
	byteCount += alignUp8(triangleCount * int(unsafe.Sizeof(MeshTriangle{})))
	materialIndicesOffset := byteCount
	byteCount += alignUp8(triangleCount * 1)
	flagsOffset := byteCount
	byteCount += alignUp8(triangleCount * 1)

	// zero initialize for determinism
	mesh := (*MeshData)(blobAlloc(byteCount))

	mesh.Version = MeshVersion
	mesh.ByteCount = int32(byteCount)
	mesh.Bounds = meshBounds
	mesh.SurfaceArea = surfaceArea
	mesh.NodeCount = int32(len(tempNodes))
	mesh.TreeHeight = int32(treeHeight)
	mesh.VertexCount = int32(vertexCount)
	mesh.TriangleCount = int32(triangleCount)
	mesh.DegenerateCount = int32(degenerateCount)
	mesh.NodeOffset = int32(nodeOffset)
	mesh.VertexOffset = int32(vertexOffset)
	mesh.TriangleOffset = int32(triangleOffset)
	mesh.MaterialOffset = int32(materialIndicesOffset)
	mesh.MaterialCount = int32(materialCount)
	mesh.FlagsOffset = int32(flagsOffset)

	nodes := getMeshNodesWrite(mesh)
	triangles := getMeshTrianglesWrite(mesh)
	meshVertices := getMeshVerticesWrite(mesh)
	materialIndices := getMeshMaterialIndicesWrite(mesh)
	flags := getMeshFlagsWrite(mesh)

	copy(nodes, tempNodes)
	copy(meshVertices, vertices[:vertexCount])

	for index := range triangleCount {
		primitive := primitives[index]

		i1 := 3*int(primitive.triangleIndex) + 0
		i2 := 3*int(primitive.triangleIndex) + 1
		i3 := 3*int(primitive.triangleIndex) + 2
		if clockWise {
			i2, i3 = i3, i2
		}

		triangles[index].Index1 = indices[i1]
		triangles[index].Index2 = indices[i2]
		triangles[index].Index3 = indices[i3]
		flags[index] = 0

		// Copy material indices if they exist. Otherwise the material indices are all zeroes.
		if def.MaterialIndices != nil {
			materialIndex := def.MaterialIndices[primitive.triangleIndex]
			materialIndices[index] = materialIndex
		}
	}

	// Sort triangle in DFS order. Casts and volume queries will return sorted arrays.
	// This also sorts material indices, but not the materials.
	// This can fail if the BVH height is too large.
	success := sortMeshTriangles(mesh)
	if success == false {
		arrayDestroy(&tempNodes)
		arrayDestroy(&primitives)
		arrayDestroy(&indices)
		arrayDestroy(&vertices)
		return nil
	}

	if def.IdentifyEdges {
		identifyEdges(mesh)
	}

	validate(!validateEnabled || isNonDegenerate(mesh, minArea), "b3IsNonDegenerate( mesh, minArea )")
	validate(!validateEnabled || meshIsConsistent(mesh), "b3IsConsistent( mesh )")

	arrayDestroy(&tempNodes)
	arrayDestroy(&primitives)
	arrayDestroy(&indices)
	arrayDestroy(&vertices)

	mesh.Hash = 0
	mesh.Hash = hash64NonZero(meshBytes(mesh))

	return mesh
}

// DestroyMesh destroys a mesh.
// PORT: the Go garbage collector owns the blob; this is a no-op.
func DestroyMesh(mesh *MeshData) {
	_ = mesh
}

// OverlapMesh overlaps a shape versus a mesh.
func OverlapMesh(shape *Mesh, shapeTransform Transform, proxy *ShapeProxy) bool {
	assert(proxy.Count > 0, "proxy->count > 0")
	cache := SimplexCache{}

	var buffer [MaxShapeCastPoints]Vec3
	localProxy := makeLocalProxy(proxy, shapeTransform, buffer[:])
	aabb := computeProxyAABB(&localProxy)

	meshScale := shape.Scale

	// Scale may have reflection so min/max may become invalid when unscaled
	scale := loadV(vec3Floats(&meshScale))
	invScale := divV(oneVConst, scale)
	temp1 := mulV(invScale, loadV(vec3Floats(&aabb.LowerBound)))
	temp2 := mulV(invScale, loadV(vec3Floats(&aabb.UpperBound)))
	invScaledBoundsMin := minV(temp1, temp2)
	invScaledBoundsMax := maxV(temp1, temp2)
	invScaledBoundsCenter := mulV(halfVConst, addV(invScaledBoundsMin, invScaledBoundsMax))
	invScaledBoundsExtent := subV(invScaledBoundsMax, invScaledBoundsCenter)

	var input DistanceInput
	input.ProxyB = localProxy
	input.Transform = TransformIdentity
	input.UseRadii = true

	count := 0
	var stack [meshStackSize]int
	nodes, node := meshGetRoot(shape.Data)
	triangles := GetMeshTriangles(shape.Data)
	vertices := GetMeshVertices(shape.Data)

	for {
		// Test node overlap in unscaled space
		nodeMin := loadV(vec3Floats(&nodes[node].LowerBound))
		nodeMax := loadV(vec3Floats(&nodes[node].UpperBound))
		if testBoundsOverlap(nodeMin, nodeMax, invScaledBoundsMin, invScaledBoundsMax) {
			if meshIsLeaf(&nodes[node]) {
				triangleCount := meshNodeTriangleCount(&nodes[node])
				triangleOffset := int(nodes[node].TriangleOffset)

				for index := range triangleCount {
					triangleIndex := triangleOffset + index
					triangle := triangles[triangleIndex]

					vertex1 := vertices[triangle.Index1]
					vertex2 := vertices[triangle.Index2]
					vertex3 := vertices[triangle.Index3]
					v1 := loadV(vec3Floats(&vertex1))
					v2 := loadV(vec3Floats(&vertex2))
					v3 := loadV(vec3Floats(&vertex3))

					// Bounding box overlap test in unscaled space
					if testBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3) {
						// Shape-triangle overlap test in scaled space. Winding order doesn't matter.
						triangleVertices := [3]Vec3{Mul(meshScale, vertex1), Mul(meshScale, vertex2),
							Mul(meshScale, vertex3)}
						input.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}

						// reset the cache
						cache.Count = 0

						// get distance between triangle and query shape
						output := ShapeDistance(&input, &cache, nil)

						tolerance := 0.1 * LinearSlop()
						if output.Distance < tolerance {
							// overlap detected
							return true
						}
					}
				}
			} else {
				// Recurse
				assert(count <= meshStackSize-1, "count <= B3_MESH_STACK_SIZE - 1")
				stack[count] = meshGetRightChild(nodes, node)
				count++
				node = meshGetLeftChild(nodes, node)

				continue
			}
		}

		if count == 0 {
			break
		}
		count--
		node = stack[count]
	}

	return false
}

// ComputeMeshAABB computes the bounding box of a transformed mesh.
func ComputeMeshAABB(shape *MeshData, transform Transform, scale Vec3) AABB {
	scaledLower := Mul(scale, shape.Bounds.LowerBound)
	scaledUpper := Mul(scale, shape.Bounds.UpperBound)
	bounds := AABB{Min(scaledLower, scaledUpper), Max(scaledLower, scaledUpper)}
	return bounds.Transform(transform)
}

// RayCastMesh ray casts versus a mesh in local space.
func RayCastMesh(mesh *Mesh, input *RayCastInput) CastOutput {
	data := mesh.Data
	meshScale := mesh.Scale

	bestOutput := CastOutput{}
	bestOutput.Fraction = input.MaxFraction
	bestOutput.TriangleIndex = NullIndex

	lambda := splatV(input.MaxFraction)

	rayStart := loadV(vec3Floats(&input.Origin))
	rayDelta := loadV(vec3Floats(&input.Translation))

	scale := loadV(vec3Floats(&meshScale))
	invScale := divV(oneVConst, scale)
	ccw := meshScale.X*meshScale.Y*meshScale.Z > 0.0

	// Use the inverse scaled ray for traversal of the BVH
	invScaledRayStart := mulV(invScale, rayStart)
	invScaledRayDelta := mulV(invScale, rayDelta)
	invScaledRayEnd := addV(invScaledRayStart, mulV(lambda, invScaledRayDelta))
	invScaledRayMin := minV(invScaledRayStart, invScaledRayEnd)
	invScaledRayMax := maxV(invScaledRayStart, invScaledRayEnd)

	count := 0
	var stack [meshStackSize]int
	nodes, node := meshGetRoot(data)
	triangles := GetMeshTriangles(data)
	vertices := GetMeshVertices(data)
	materialIndices := GetMeshMaterialIndices(data)

	for {
		// Test node/ray overlap using SAT
		nodeMin := loadV(vec3Floats(&nodes[node].LowerBound))
		nodeMax := loadV(vec3Floats(&nodes[node].UpperBound))
		if testBoundsOverlap(nodeMin, nodeMax, invScaledRayMin, invScaledRayMax) &&
			testBoundsRayOverlap(nodeMin, nodeMax, invScaledRayStart, invScaledRayDelta) {
			// SAT: The node and ray overlap - process leaf node or recurse
			if meshIsLeaf(&nodes[node]) {
				triangleCount := meshNodeTriangleCount(&nodes[node])
				triangleOffset := int(nodes[node].TriangleOffset)

				for index := range triangleCount {
					triangleIndex := triangleOffset + index
					triangle := triangles[triangleIndex]

					// Collide ray with triangle in scaled space
					vertex1 := Mul(meshScale, vertices[triangle.Index1])
					var vertex2, vertex3 Vec3

					// The CPU should predict this branch
					if ccw {
						vertex2 = Mul(meshScale, vertices[triangle.Index2])
						vertex3 = Mul(meshScale, vertices[triangle.Index3])
					} else {
						vertex2 = Mul(meshScale, vertices[triangle.Index3])
						vertex3 = Mul(meshScale, vertices[triangle.Index2])
					}

					// Collide ray with triangle in scaled space
					v1 := loadV(vec3Floats(&vertex1))
					v2 := loadV(vec3Floats(&vertex2))
					v3 := loadV(vec3Floats(&vertex3))

					alpha := intersectRayTriangle(rayStart, rayDelta, v1, v2, v3)
					assert(0 <= alpha && alpha <= 1.0, "0 <= alpha && alpha <= 1.0f")

					if alpha < bestOutput.Fraction {
						edge1 := Sub(vertex2, vertex1)
						edge2 := Sub(vertex3, vertex1)
						bestOutput.Normal = Normalize(Cross(edge1, edge2))
						bestOutput.Point = Add(input.Origin, MulSV(alpha, input.Translation))
						bestOutput.Fraction = alpha
						bestOutput.TriangleIndex = int32(triangleIndex)
						bestOutput.MaterialIndex = int32(materialIndices[triangleIndex])
						bestOutput.Hit = true

						// Update ray bounds in unscaled space
						lambda = splatV(alpha)
						invScaledRayEnd = addV(invScaledRayStart, mulV(lambda, invScaledRayDelta))
						invScaledRayMin = minV(invScaledRayStart, invScaledRayEnd)
						invScaledRayMax = maxV(invScaledRayStart, invScaledRayEnd)
					}
				}
			} else {
				// Determine traversal order (front -> back) and recurse
				axis := meshNodeAxis(&nodes[node])
				if getV(invScaledRayDelta, axis) > 0.0 {
					assert(count <= meshStackSize-1, "count <= B3_MESH_STACK_SIZE - 1")
					stack[count] = meshGetRightChild(nodes, node)
					count++
					node = meshGetLeftChild(nodes, node)
				} else {
					assert(count <= meshStackSize-1, "count <= B3_MESH_STACK_SIZE - 1")
					stack[count] = meshGetLeftChild(nodes, node)
					count++
					node = meshGetRightChild(nodes, node)
				}

				continue
			}
		}

		if count == 0 {
			break
		}
		count--
		node = stack[count]
	}

	return bestOutput
}

// ShapeCastMesh shape casts versus a mesh in local space.
func ShapeCastMesh(mesh *Mesh, input *ShapeCastInput) CastOutput {
	data := mesh.Data
	meshScale := mesh.Scale

	bestOutput := CastOutput{}
	bestOutput.Fraction = input.MaxFraction
	bestOutput.TriangleIndex = NullIndex

	lambda := splatV(input.MaxFraction)

	shapeBounds := MakeAABB(input.Proxy.Points[:input.Proxy.Count], input.Proxy.Radius)
	center := shapeBounds.Center()
	extents := shapeBounds.Extents()
	shapeExtent := loadV(vec3Floats(&extents))

	rayStart := loadV(vec3Floats(&center))
	rayDelta := loadV(vec3Floats(&input.Translation))
	rayEnd := addV(rayStart, mulV(lambda, rayDelta))
	rayMin := minV(rayStart, rayEnd)
	rayMax := maxV(rayStart, rayEnd)

	scale := loadV(vec3Floats(&meshScale))
	invScale := divV(oneVConst, scale)
	absInvScale := absV(invScale)
	ccw := meshScale.X*meshScale.Y*meshScale.Z > 0.0

	// Use the inverse scaled shape cast for traversal of the BVH
	invScaledRayStart := mulV(invScale, rayStart)
	invScaledRayDelta := mulV(invScale, rayDelta)
	invScaledRayEnd := addV(invScaledRayStart, mulV(lambda, invScaledRayDelta))
	invScaledRayMin := minV(invScaledRayStart, invScaledRayEnd)
	invScaledRayMax := maxV(invScaledRayStart, invScaledRayEnd)
	invScaledShapeExtent := mulV(absInvScale, shapeExtent)

	count := 0
	var stack [meshStackSize]int
	nodes, node := meshGetRoot(data)
	triangles := GetMeshTriangles(data)
	vertices := GetMeshVertices(data)
	materialIndices := GetMeshMaterialIndices(data)

	for {
		// Test node/ray overlap using SAT in unscaled space
		nodeMin := subV(loadV(vec3Floats(&nodes[node].LowerBound)), invScaledShapeExtent)
		nodeMax := addV(loadV(vec3Floats(&nodes[node].UpperBound)), invScaledShapeExtent)

		if testBoundsOverlap(nodeMin, nodeMax, invScaledRayMin, invScaledRayMax) &&
			testBoundsRayOverlap(nodeMin, nodeMax, invScaledRayStart, invScaledRayDelta) {
			// SAT: The node and ray overlap - process leaf node or recurse
			if meshIsLeaf(&nodes[node]) {
				triangleCount := meshNodeTriangleCount(&nodes[node])
				triangleOffset := int(nodes[node].TriangleOffset)

				for index := range triangleCount {
					triangleIndex := triangleOffset + index
					triangle := triangles[triangleIndex]

					// Collide shape with triangle in scaled space
					vertex1 := Mul(meshScale, vertices[triangle.Index1])
					var vertex2, vertex3 Vec3

					// The CPU should predict this branch
					if ccw {
						vertex2 = Mul(meshScale, vertices[triangle.Index2])
						vertex3 = Mul(meshScale, vertices[triangle.Index3])
					} else {
						vertex2 = Mul(meshScale, vertices[triangle.Index3])
						vertex3 = Mul(meshScale, vertices[triangle.Index2])
					}

					v1 := loadV(vec3Floats(&vertex1))
					v2 := loadV(vec3Floats(&vertex2))
					v3 := loadV(vec3Floats(&vertex3))

					triangleMin := subV(minV(v1, minV(v2, v3)), shapeExtent)
					triangleMax := addV(maxV(v1, maxV(v2, v3)), shapeExtent)

					// Test extended triangle vs ray bounds overlap in scaled space. Skip edge tests.
					overlap := testBoundsOverlap(triangleMin, triangleMax, rayMin, rayMax)
					if overlap == false {
						continue
					}

					// This test is in scaled space. vertices and center are scaled.
					signedVolume := signedVolume(vertex1, vertex2, vertex3, center)
					if signedVolume < 0.0 {
						// Backside
						continue
					}

					// Collide shape with triangle in scaled space.
					origin := vertex1
					triangleVertices := [3]Vec3{Vec3Zero, Sub(vertex2, origin), Sub(vertex3, origin)}
					shiftedOrigin := Transform{Neg(origin), QuatIdentity}

					var pairInput ShapeCastPairInput
					pairInput.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}
					pairInput.ProxyB = input.Proxy
					pairInput.Transform = shiftedOrigin
					pairInput.MaxFraction = bestOutput.Fraction
					pairInput.TranslationB = input.Translation
					pairInput.CanEncroach = input.CanEncroach

					pairOutput := ShapeCast(&pairInput)

					if pairOutput.Hit {
						pairOutput.Point = Add(pairOutput.Point, origin)

						bestOutput = pairOutput
						bestOutput.TriangleIndex = int32(triangleIndex)
						bestOutput.MaterialIndex = int32(materialIndices[triangleIndex])

						// Update ray bounds in scaled space
						lambda = splatV(pairOutput.Fraction)
						rayEnd = addV(rayStart, mulV(lambda, rayDelta))
						rayMin = minV(rayStart, rayEnd)
						rayMax = maxV(rayStart, rayEnd)

						// Ray bounds in unscaled space
						invScaledRayEnd = addV(invScaledRayStart, mulV(lambda, invScaledRayDelta))
						invScaledRayMin = minV(invScaledRayStart, invScaledRayEnd)
						invScaledRayMax = maxV(invScaledRayStart, invScaledRayEnd)
					}
				}
			} else {
				// Determine traversal order (front -> back) and recurse
				axis := meshNodeAxis(&nodes[node])
				if getV(invScaledRayDelta, axis) > 0.0 {
					assert(count <= meshStackSize-1, "count <= B3_MESH_STACK_SIZE - 1")
					stack[count] = meshGetRightChild(nodes, node)
					count++
					node = meshGetLeftChild(nodes, node)
				} else {
					assert(count <= meshStackSize-1, "count <= B3_MESH_STACK_SIZE - 1")
					stack[count] = meshGetLeftChild(nodes, node)
					count++
					node = meshGetRightChild(nodes, node)
				}

				continue
			}
		}

		if count == 0 {
			break
		}
		count--
		node = stack[count]
	}

	return bestOutput
}

// collideMoverAndMesh is b3CollideMoverAndMesh.
// PORT: b3PlaneResult* planes with capacity becomes a slice; capacity is len(planes).
func collideMoverAndMesh(planes []PlaneResult, shape *Mesh, mover *Capsule) int {
	capacity := len(planes)
	if capacity == 0 {
		return 0
	}

	distanceInput := DistanceInput{}
	distanceInput.ProxyB = ShapeProxy{capsulePoints(mover), 2, 0.0}
	distanceInput.Transform = TransformIdentity
	distanceInput.UseRadii = false

	cache := SimplexCache{}
	radius := mover.Radius
	center := Lerp(mover.Center1, mover.Center2, 0.5)
	center1 := loadV(vec3Floats(&mover.Center1))
	center2 := loadV(vec3Floats(&mover.Center2))
	r := splatV(radius)
	boundsMin := subV(minV(center1, center2), r)
	boundsMax := addV(maxV(center1, center2), r)

	// Scale may have reflection so min/max may become invalid when unscaled
	meshScale := shape.Scale
	ccw := meshScale.X*meshScale.Y*meshScale.Z > 0.0
	scale := loadV(vec3Floats(&meshScale))
	invScale := divV(oneVConst, scale)
	temp1 := mulV(invScale, boundsMin)
	temp2 := mulV(invScale, boundsMax)
	invScaledBoundsMin := minV(temp1, temp2)
	invScaledBoundsMax := maxV(temp1, temp2)
	invScaledBoundsCenter := mulV(halfVConst, addV(invScaledBoundsMin, invScaledBoundsMax))
	invScaledBoundsExtent := subV(invScaledBoundsMax, invScaledBoundsCenter)

	count := 0
	var stack [meshStackSize]int
	nodes, node := meshGetRoot(shape.Data)
	triangles := GetMeshTriangles(shape.Data)
	vertices := GetMeshVertices(shape.Data)
	materialIndices := GetMeshMaterialIndices(shape.Data)

	planeCount := 0
	for planeCount < capacity {
		// Test node overlap in unscaled space
		nodeMin := loadV(vec3Floats(&nodes[node].LowerBound))
		nodeMax := loadV(vec3Floats(&nodes[node].UpperBound))
		if testBoundsOverlap(nodeMin, nodeMax, invScaledBoundsMin, invScaledBoundsMax) {
			if meshIsLeaf(&nodes[node]) {
				triangleCount := meshNodeTriangleCount(&nodes[node])
				triangleOffset := int(nodes[node].TriangleOffset)

				for index := range triangleCount {
					triangleIndex := triangleOffset + index
					triangle := triangles[triangleIndex]

					vertex1 := vertices[triangle.Index1]
					vertex2 := vertices[triangle.Index2]
					vertex3 := vertices[triangle.Index3]

					if ccw == false {
						vertex2, vertex3 = vertex3, vertex2
					}

					v1 := loadV(vec3Floats(&vertex1))
					v2 := loadV(vec3Floats(&vertex2))
					v3 := loadV(vec3Floats(&vertex3))

					// Test triangle bounds overlap in unscaled space
					if testBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3) {
						// Compute shape distance in scaled space.
						triangleVertices := [3]Vec3{Mul(meshScale, vertex1), Mul(meshScale, vertex2),
							Mul(meshScale, vertex3)}

						// Vertices and center are in scaled space.
						signedVolume :=
							signedVolume(triangleVertices[0], triangleVertices[1], triangleVertices[2], center)
						if signedVolume < 0.0 {
							// Backside
							continue
						}

						distanceInput.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}

						// reset the cache
						cache.Count = 0

						// get distance between triangle and query shape
						distanceOutput := ShapeDistance(&distanceInput, &cache, nil)

						if distanceOutput.Distance == 0.0 {
							// deep overlap
						} else if distanceOutput.Distance <= mover.Radius {
							plane := Plane{distanceOutput.Normal, mover.Radius - distanceOutput.Distance}
							planes[planeCount] =
								PlaneResult{plane, distanceOutput.PointA, int32(triangleIndex), 0, int32(materialIndices[triangleIndex])}
							planeCount += 1

							if planeCount == capacity {
								return planeCount
							}
						}
					}
				}
			} else {
				// Recurse
				assert(count <= meshStackSize-1, "count <= B3_MESH_STACK_SIZE - 1")
				stack[count] = meshGetRightChild(nodes, node)
				count++
				node = meshGetLeftChild(nodes, node)

				continue
			}
		}

		if count == 0 {
			break
		}
		count--
		node = stack[count]
	}

	return planeCount
}

// QueryMesh queries a mesh for triangles overlapping a local AABB. The callback receives the
// scaled triangle vertices.
func QueryMesh(mesh *Mesh, bounds AABB, fcn MeshQueryFcn, context any) {
	meshScale := mesh.Scale
	ccw := meshScale.X*meshScale.Y*meshScale.Z > 0.0

	// Scale may have reflection so min/max may become invalid when unscaled
	scale := loadV(vec3Floats(&meshScale))
	invScale := divV(oneVConst, scale)
	temp1 := mulV(invScale, loadV(vec3Floats(&bounds.LowerBound)))
	temp2 := mulV(invScale, loadV(vec3Floats(&bounds.UpperBound)))
	invScaledBoundsMin := minV(temp1, temp2)
	invScaledBoundsMax := maxV(temp1, temp2)
	invScaledBoundsCenter := mulV(halfVConst, addV(invScaledBoundsMin, invScaledBoundsMax))
	invScaledBoundsExtent := subV(invScaledBoundsMax, invScaledBoundsCenter)

	data := mesh.Data

	count := 0
	var stack [meshStackSize]int
	nodes, node := meshGetRoot(data)
	triangles := GetMeshTriangles(data)
	vertices := GetMeshVertices(data)

	// Test node overlap in unscaled space
	if testBoundsOverlap(loadV(vec3Floats(&nodes[node].LowerBound)), loadV(vec3Floats(&nodes[node].UpperBound)), invScaledBoundsMin,
		invScaledBoundsMax) == false {
		return
	}

	for {
		if meshIsLeaf(&nodes[node]) {
			triangleCount := meshNodeTriangleCount(&nodes[node])
			triangleOffset := int(nodes[node].TriangleOffset)

			for index := range triangleCount {
				triangleIndex := triangleOffset + index
				triangle := triangles[triangleIndex]

				vertex1 := vertices[triangle.Index1]
				vertex2 := vertices[triangle.Index2]
				vertex3 := vertices[triangle.Index3]
				v1 := loadV(vec3Floats(&vertex1))
				v2 := loadV(vec3Floats(&vertex2))
				v3 := loadV(vec3Floats(&vertex3))

				// Perform triangle overlap test in unscaled space.
				// todo it is possible that some margins are getting scaled
				if testBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3) {
					a := Mul(meshScale, vertex1)
					var b, c Vec3
					if ccw {
						b = Mul(meshScale, vertex2)
						c = Mul(meshScale, vertex3)
					} else {
						b = Mul(meshScale, vertex3)
						c = Mul(meshScale, vertex2)
					}

					result := fcn(a, b, c, triangleIndex, context)
					if result == false {
						return
					}
				}
			}
		} else {
			leftChild := meshGetLeftChild(nodes, node)
			rightChild := meshGetRightChild(nodes, node)

			overlapLeft := testBoundsOverlap(loadV(vec3Floats(&nodes[leftChild].LowerBound)), loadV(vec3Floats(&nodes[leftChild].UpperBound)),
				invScaledBoundsMin, invScaledBoundsMax)
			overlapRight := testBoundsOverlap(loadV(vec3Floats(&nodes[rightChild].LowerBound)),
				loadV(vec3Floats(&nodes[rightChild].UpperBound)), invScaledBoundsMin, invScaledBoundsMax)

			if overlapLeft {
				if overlapRight {
					assert(count <= meshStackSize-1, "count <= B3_MESH_STACK_SIZE - 1")
					stack[count] = rightChild
					count++
				}

				node = leftChild
				continue
			}

			if overlapRight {
				node = rightChild
				continue
			}
		}

		if count == 0 {
			break
		}
		count--
		node = stack[count]
	}
}

// Mesh blob layout. Identity covers every byte, so the structs carry explicit padding.
// PORT: _Static_assert style size locks (no upstream counterpart for these two; they pin the C
// layout the byte image depends on).
const (
	_ uintptr = unsafe.Sizeof(MeshData{}) - 96
	_ uintptr = 96 - unsafe.Sizeof(MeshData{})
	_ uintptr = unsafe.Sizeof(MeshNode{}) - 32
	_ uintptr = 32 - unsafe.Sizeof(MeshNode{})
)
