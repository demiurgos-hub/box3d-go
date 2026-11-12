// Port of src/compound.h and src/compound.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"bytes"
	"math"
	"unsafe"
)

// ---- compound.h ----

// getCompoundTreeView is b3GetCompoundTreeView.
// PORT: the node slice spans the NodeCount nodes of the image.
func getCompoundTreeView(compound *CompoundData) treeView {
	view := treeView{
		nodes:   blobSlice[TreeNode](unsafe.Pointer(compound), compound.NodeOffset, int(compound.NodeCount)),
		proxies: nil,
	}
	return view
}

// PORT: b3CompoundTimeOfImpact is declared in compound.h but its only definition in compound.c is
// inside #if 0, so it is not ported.

// makeCompoundChildSweep (b3MakeCompoundChildSweep) and collideMoverAndCompound
// (b3CollideMoverAndCompound) are defined below with compound.c.

// ---- compound.c ----

// sharedHull is b3SharedHull.
type sharedHull struct {
	hull       *HullData
	hullOffset int
}

// sharedMesh is b3SharedMesh.
type sharedMesh struct {
	meshData   *MeshData
	meshOffset int
}

// hullInstance is b3HullInstance.
type hullInstance struct {
	transform     Transform
	hullOffset    uint32
	materialIndex uint16
	padding       uint16
}

// PORT: the _Static_assert size checks become constant pairs that overflow unless equal.
const (
	_ uintptr = unsafe.Sizeof(hullInstance{}) - (unsafe.Sizeof(Transform{}) + 4 + 2*2)
	_ uintptr = (unsafe.Sizeof(Transform{}) + 4 + 2*2) - unsafe.Sizeof(hullInstance{})
)

// meshInstance is b3MeshInstance.
type meshInstance struct {
	transform      Transform
	scale          Vec3
	meshOffset     uint32
	materialOffset uint32
	materialCount  uint32
}

const (
	_ uintptr = unsafe.Sizeof(CompoundCapsule{}) - (unsafe.Sizeof(Capsule{}) + 2*2)
	_ uintptr = (unsafe.Sizeof(Capsule{}) + 2*2) - unsafe.Sizeof(CompoundCapsule{})
	_ uintptr = unsafe.Sizeof(CompoundSphere{}) - (unsafe.Sizeof(Sphere{}) + 2*2)
	_ uintptr = (unsafe.Sizeof(Sphere{}) + 2*2) - unsafe.Sizeof(CompoundSphere{})
	_ uintptr = unsafe.Sizeof(CompoundData{}) - (8 + unsafe.Sizeof(AABB{}) + 16*4)
	_ uintptr = (8 + unsafe.Sizeof(AABB{}) + 16*4) - unsafe.Sizeof(CompoundData{})

	// PORT: no upstream counterpart. The blob copies b3MeshInstance and b3TreeNode arrays verbatim,
	// so their Go layout must be the C layout.
	_ uintptr = unsafe.Sizeof(meshInstance{}) - 52
	_ uintptr = 52 - unsafe.Sizeof(meshInstance{})
	_ uintptr = unsafe.Sizeof(TreeNode{}) - 32
	_ uintptr = 32 - unsafe.Sizeof(TreeNode{})
)

// compoundBytes is the byte image of a compound: (uint8_t*)compound with compound->byteCount bytes.
func compoundBytes(compound *CompoundData) []byte {
	return blobBytes(unsafe.Pointer(compound), compound.ByteCount)
}

// GetCompoundMaterials accesses the compound material array.
// PORT: returns MaterialCount materials (nil when the offset is 0).
func GetCompoundMaterials(compound *CompoundData) []SurfaceMaterial {
	if compound.MaterialOffset == 0 {
		return nil
	}

	return blobSlice[SurfaceMaterial](unsafe.Pointer(compound), compound.MaterialOffset, int(compound.MaterialCount))
}

// GetCompoundCapsule accesses a child capsule by index.
func GetCompoundCapsule(compound *CompoundData, index int) CompoundCapsule {
	assert(0 <= index && index < int(compound.CapsuleCount) && compound.CapsuleOffset > 0,
		"0 <= index && index < compound->capsuleCount && compound->capsuleOffset > 0")

	result := CompoundCapsule{}
	if compound.CapsuleOffset == 0 {
		return result
	}

	capsules := blobSlice[CompoundCapsule](unsafe.Pointer(compound), compound.CapsuleOffset, int(compound.CapsuleCount))
	return capsules[index]
}

// GetCompoundHull accesses a child hull by index.
func GetCompoundHull(compound *CompoundData, index int) CompoundHull {
	assert(0 <= index && index < int(compound.HullCount) && compound.HullOffset > 0,
		"0 <= index && index < compound->hullCount && compound->hullOffset > 0")

	result := CompoundHull{}
	if compound.HullOffset == 0 {
		return result
	}

	hullInstances := blobSlice[hullInstance](unsafe.Pointer(compound), compound.HullOffset, int(compound.HullCount))
	hullOffset := hullInstances[index].hullOffset
	assert(uintptr(hullOffset) >= uintptr(compound.HullOffset)+uintptr(compound.HullCount)*unsafe.Sizeof(hullInstance{}),
		"hullOffset >= compound->hullOffset + compound->hullCount * sizeof( b3HullInstance )")
	result.Hull = (*HullData)(unsafe.Add(unsafe.Pointer(compound), hullOffset))
	result.Transform = hullInstances[index].transform
	result.MaterialIndex = hullInstances[index].materialIndex
	return result
}

// GetCompoundMesh accesses a child mesh by index.
func GetCompoundMesh(compound *CompoundData, index int) CompoundMesh {
	assert(0 <= index && index < int(compound.MeshCount) && compound.MeshOffset > 0,
		"0 <= index && index < compound->meshCount && compound->meshOffset > 0")

	result := CompoundMesh{}
	if compound.MeshOffset == 0 {
		return result
	}

	meshInstances := blobSlice[meshInstance](unsafe.Pointer(compound), compound.MeshOffset, int(compound.MeshCount))
	meshOffset := meshInstances[index].meshOffset
	// PORT: upstream compares against sizeof( b3HullInstance ) here; kept as is.
	assert(uintptr(meshOffset) >= uintptr(compound.MeshOffset)+uintptr(compound.MeshCount)*unsafe.Sizeof(hullInstance{}),
		"meshOffset >= compound->meshOffset + compound->meshCount * sizeof( b3HullInstance )")
	result.MeshData = (*MeshData)(unsafe.Add(unsafe.Pointer(compound), meshOffset))
	result.Transform = meshInstances[index].transform
	result.Scale = meshInstances[index].scale
	result.MaterialIndices = blobSlice[uint16](unsafe.Pointer(compound), int32(meshInstances[index].materialOffset),
		int(meshInstances[index].materialCount))
	result.MaterialCount = int32(meshInstances[index].materialCount)
	return result
}

// GetCompoundSphere accesses a child sphere by index.
func GetCompoundSphere(compound *CompoundData, index int) CompoundSphere {
	assert(0 <= index && index < int(compound.SphereCount) && compound.SphereOffset > 0,
		"0 <= index && index < compound->sphereCount && compound->sphereOffset > 0")

	result := CompoundSphere{}
	if compound.SphereOffset == 0 {
		return result
	}

	spheres := blobSlice[CompoundSphere](unsafe.Pointer(compound), compound.SphereOffset, int(compound.SphereCount))
	return spheres[index]
}

// GetCompoundChild gets a child shape of a compound.
// PORT: materialIndices aliases the image (a one element slice for convex children).
func GetCompoundChild(compound *CompoundData, childIndex int) ChildShape {
	// Capsule?
	if 0 <= childIndex && childIndex < int(compound.CapsuleCount) {
		assert(compound.CapsuleOffset > 0, "compound->capsuleOffset > 0")
		capsules := blobSlice[CompoundCapsule](unsafe.Pointer(compound), compound.CapsuleOffset, int(compound.CapsuleCount))
		return ChildShape{
			Capsule:         capsules[childIndex].Capsule,
			Transform:       TransformIdentity,
			MaterialIndices: unsafe.Slice(&capsules[childIndex].MaterialIndex, 1),
			MaterialCount:   1,
			Type:            CapsuleShape,
		}
	}
	childIndex -= int(compound.CapsuleCount)

	// Hull?
	if 0 <= childIndex && childIndex < int(compound.HullCount) {
		assert(compound.HullOffset > 0, "compound->hullOffset > 0")
		hullInstances := blobSlice[hullInstance](unsafe.Pointer(compound), compound.HullOffset, int(compound.HullCount))
		hullOffset := hullInstances[childIndex].hullOffset
		return ChildShape{
			Hull:            (*HullData)(unsafe.Add(unsafe.Pointer(compound), hullOffset)),
			Transform:       hullInstances[childIndex].transform,
			MaterialIndices: unsafe.Slice(&hullInstances[childIndex].materialIndex, 1),
			MaterialCount:   1,
			Type:            HullShape,
		}
	}
	childIndex -= int(compound.HullCount)

	// Mesh?
	if 0 <= childIndex && childIndex < int(compound.MeshCount) {
		compoundMesh := GetCompoundMesh(compound, childIndex)

		return ChildShape{
			Mesh: Mesh{
				Data:  compoundMesh.MeshData,
				Scale: compoundMesh.Scale,
			},
			Transform:       compoundMesh.Transform,
			MaterialIndices: compoundMesh.MaterialIndices,
			MaterialCount:   compoundMesh.MaterialCount,
			Type:            MeshShape,
		}
	}
	childIndex -= int(compound.MeshCount)

	assert(0 <= childIndex && childIndex < int(compound.SphereCount), "0 <= childIndex && childIndex < compound->sphereCount")

	// Sphere?
	{
		assert(compound.SphereOffset > 0, "compound->sphereOffset > 0")
		spheres := blobSlice[CompoundSphere](unsafe.Pointer(compound), compound.SphereOffset, int(compound.SphereCount))
		return ChildShape{
			Sphere:          spheres[childIndex].Sphere,
			Transform:       TransformIdentity,
			MaterialIndices: unsafe.Slice(&spheres[childIndex].MaterialIndex, 1),
			MaterialCount:   1,
			Type:            SphereShape,
		}
	}
}

// hashMesh is b3HashMesh.
// PORT: vt_wyhash is replaced by rapidhash over the same bytes. The hash only places buckets in the
// Go verstable, whose bucket order differs from Verstable anyway.
func hashMesh(mesh *MeshData) uint64 {
	return rapidhash(meshBytes(mesh))
}

// compareMeshes is b3CompareMeshes.
func compareMeshes(mesh1, mesh2 *MeshData) bool {
	if mesh1 == mesh2 {
		return true
	}

	if mesh1.ByteCount != mesh2.ByteCount {
		return false
	}

	return bytes.Equal(meshBytes(mesh1), meshBytes(mesh2))
}

// meshMap is b3MeshMap.
type meshMap = verstable[*MeshData, int]

const (
	_ uintptr = unsafe.Sizeof(SurfaceMaterial{}) - 40
	_ uintptr = 40 - unsafe.Sizeof(SurfaceMaterial{})
)

// surfaceMaterialBytes is the 40 byte image of a material (memcmp / hash input).
func surfaceMaterialBytes(material *SurfaceMaterial) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(material)), unsafe.Sizeof(*material))
}

// hashMaterial is b3HashMaterial.
// PORT: vt_wyhash is replaced by rapidhash, see hashMesh.
func hashMaterial(material *SurfaceMaterial) uint64 {
	assert(material.Padding == 0, "material->padding == 0")

	return rapidhash(surfaceMaterialBytes(material))
}

// compareMaterials is b3CompareMaterials.
func compareMaterials(mat1, mat2 *SurfaceMaterial) bool {
	assert(mat1.Padding == 0 && mat2.Padding == 0, "mat1->padding == 0 && mat2->padding == 0")

	if mat1 == mat2 {
		return true
	}

	return bytes.Equal(surfaceMaterialBytes(mat1), surfaceMaterialBytes(mat2))
}

// materialMap is b3MaterialMap.
type materialMap = verstable[*SurfaceMaterial, int]

// CreateCompound creates a compound shape. All input data in the definition is cloned into the
// resulting compound.
func CreateCompound(def *CompoundDef) *CompoundData {
	convexCount := int(def.CapsuleCount + def.HullCount + def.SphereCount)
	shapeCount := convexCount + int(def.MeshCount)

	if shapeCount >= MaxChildShapes {
		assert(false, "false")
		return nil
	}

	tree := CreateDynamicTree(shapeCount)

	childIndex := 0

	// Instances
	capsuleCount := int(def.CapsuleCount)
	capsuleInstances := make([]CompoundCapsule, capsuleCount)
	hullCount := int(def.HullCount)
	hullInstances := make([]hullInstance, hullCount)
	meshCount := int(def.MeshCount)
	meshInstances := make([]meshInstance, meshCount)
	sphereCount := int(def.SphereCount)
	sphereInstances := make([]CompoundSphere, sphereCount)

	// Determine material capacity.
	materialCapacity := convexCount
	for i := range int(def.MeshCount) {
		assert(def.Meshes[i].MaterialCount > 0, "def->meshes[i].materialCount > 0")
		materialCapacity += int(def.Meshes[i].MaterialCount)
	}

	// Material map for convex material sharing. Mesh materials are not shared for simplicity.
	var materialMap materialMap
	materialMap.init(hashMaterial, compareMaterials)
	materialMap.reserve(materialCapacity)
	materials := make([]SurfaceMaterial, materialCapacity)
	materialCount := 0

	meshMaterialTotal := materialCapacity - convexCount
	meshMaterialStaging := make([]uint16, meshMaterialTotal)
	meshMaterialCursor := 0

	for i := range int(def.CapsuleCount) {
		capsuleDef := &def.Capsules[i]
		capsuleInstances[i].Capsule = capsuleDef.Capsule

		// Look for an existing material.
		materialItr := materialMap.getOrInsert(&capsuleDef.Material, materialCount)

		// Get the shared material index.
		materialIndex := materialItr.data.val
		capsuleInstances[i].MaterialIndex = uint16(materialIndex)

		// Is this a new material?
		if materialIndex == materialCount {
			materials[materialIndex] = capsuleDef.Material
			materialCount += 1
		}

		aabb := ComputeCapsuleAABB(&capsuleDef.Capsule, TransformIdentity)
		tree.CreateProxy(aabb, ^uint64(0), uint64(childIndex))
		childIndex += 1
	}

	// Hulls
	sharedHulls := make([]sharedHull, hullCount)
	sharedHullCount := 0

	if hullCount > 0 {
		var hullMap hullMap
		hullMapInit(&hullMap)
		hullMap.reserve(hullCount)

		for i := range hullCount {
			hullDef := &def.Hulls[i]
			hull := hullDef.Hull
			aabb := ComputeHullAABB(hull, hullDef.Transform)
			tree.CreateProxy(aabb, ^uint64(0), uint64(childIndex))
			childIndex += 1

			// Look for an existing material.
			materialItr := materialMap.getOrInsert(&hullDef.Material, materialCount)

			// Get the shared material index.
			materialIndex := materialItr.data.val
			hullInstances[i].materialIndex = uint16(materialIndex)

			// Is this a new material?
			if materialIndex == materialCount {
				materials[materialIndex] = hullDef.Material
				materialCount += 1
			}

			hullInstances[i].transform = hullDef.Transform

			// Look for an existing matching hull
			itr := hullMap.getOrInsert(hull, sharedHullCount)

			// Get the unique index for this hull
			sharedHullIndex := itr.data.val

			// The offset isn't known yet, so store the index of the shared hull
			hullInstances[i].hullOffset = uint32(sharedHullIndex)

			// Is this a new hull?
			if sharedHullIndex == sharedHullCount {
				// Create a shared hull. The offset is determined below.
				sharedHulls[sharedHullIndex].hull = hull
				sharedHulls[sharedHullIndex].hullOffset = NullIndex
				sharedHullCount += 1
			}
		}

		hullMap.cleanup()
	}

	// Meshes
	sharedMeshes := make([]sharedMesh, meshCount)
	sharedMeshCount := 0

	if meshCount > 0 {
		var meshMap meshMap
		meshMap.init(hashMesh, compareMeshes)
		meshMap.reserve(meshCount)

		for i := range meshCount {
			meshDef := &def.Meshes[i]

			meshData := meshDef.MeshData
			aabb := ComputeMeshAABB(meshData, meshDef.Transform, meshDef.Scale)
			tree.CreateProxy(aabb, ^uint64(0), uint64(childIndex))
			childIndex += 1

			// No effort to share mesh materials. It would be easier to do if the number of materials was limited.
			assert(meshData.MaterialCount == meshDef.MaterialCount, "meshData->materialCount == meshDef->materialCount")

			meshMaterialStart := meshMaterialCursor

			for j := range int(meshDef.MaterialCount) {
				// Look for an existing material.
				materialItr :=
					materialMap.getOrInsert(&meshDef.Materials[j], materialCount)

				// Get the shared material index.
				materialIndex := materialItr.data.val
				meshMaterialStaging[meshMaterialCursor] = uint16(materialIndex)
				meshMaterialCursor += 1

				// Is this a new material?
				if materialIndex == materialCount {
					materials[materialIndex] = meshDef.Materials[j]
					materialCount += 1
				}
			}

			// Look for an existing matching mesh.
			itr := meshMap.getOrInsert(meshData, sharedMeshCount)

			// Get the shared mesh index.
			sharedMeshIndex := itr.data.val

			// Create mesh instance.
			meshInstances[i].transform = def.Meshes[i].Transform
			meshInstances[i].scale = def.Meshes[i].Scale

			meshInstances[i].materialOffset = uint32(meshMaterialStart)
			meshInstances[i].materialCount = uint32(meshDef.MaterialCount)

			// The offset isn't known yet, so store the index of the shared mesh.
			meshInstances[i].meshOffset = uint32(sharedMeshIndex)

			// Is this a new mesh?
			if sharedMeshIndex == sharedMeshCount {
				// Create a shared mesh. The offset is determined below.
				sharedMeshes[sharedMeshIndex].meshData = meshData
				sharedMeshes[sharedMeshIndex].meshOffset = NullIndex
				sharedMeshCount += 1
			}
		}

		meshMap.cleanup()
	}

	// Spheres
	for i := range int(def.SphereCount) {
		sphereDef := &def.Spheres[i]
		sphereInstances[i].Sphere = sphereDef.Sphere

		// Look for an existing material.
		materialItr := materialMap.getOrInsert(&sphereDef.Material, materialCount)

		// Get the shared material index.
		materialIndex := materialItr.data.val
		sphereInstances[i].MaterialIndex = uint16(materialIndex)

		// Is this a new material?
		if materialIndex == materialCount {
			materials[materialIndex] = sphereDef.Material
			materialCount += 1
		}

		aabb := ComputeSphereAABB(&sphereDef.Sphere, TransformIdentity)
		tree.CreateProxy(aabb, ^uint64(0), uint64(childIndex))
		childIndex += 1
	}

	assert(materialCount <= materialCapacity, "materialCount <= materialCapacity")
	assert(materialCount <= math.MaxUint16, "materialCount <= UINT16_MAX")
	assert(tree.ProxyCount > 0, "tree.proxyCount > 0")

	tree.Rebuild(true)

	// Tree nodes. The rebuild is dense, so only the live nodes travel.
	byteCount := alignUp8(int(unsafe.Sizeof(CompoundData{})))
	nodeOffset := byteCount
	byteCount += alignUp8(int(tree.NodeEnd) * int(unsafe.Sizeof(TreeNode{})))
	materialOffset := byteCount
	byteCount += alignUp8(materialCount * int(unsafe.Sizeof(SurfaceMaterial{})))
	capsuleOffset := byteCount
	byteCount += alignUp8(int(def.CapsuleCount) * int(unsafe.Sizeof(CompoundCapsule{})))

	// Hull data layout has another level of indirection to allow for tight data packing
	// 1. hull instance array : hull count array of b3HullInstance with individual hull transforms and offsets
	// 2. heterogeneous array of shared hull data : each shared hull can have a different byte count, so direct indexing is not
	// possible
	hullArrayOffset := byteCount

	// Array of hull instances
	byteCount += alignUp8(hullCount * int(unsafe.Sizeof(hullInstance{})))

	// Packed shared hull blobs
	for i := range sharedHullCount {
		sharedHulls[i].hullOffset = byteCount
		byteCount += alignUp8(int(sharedHulls[i].hull.ByteCount))
	}

	// Mesh data layout has another level of indirection to allow for tight data packing
	// 1. mesh instance array : mesh count array of b3MeshInstance with individual mesh transform, scale, and offset
	// 2. heterogeneous array of shared mesh data : each shared mesh can have a different byte count, so direct indexing is not
	// possible
	meshArrayOffset := byteCount

	// Array of mesh instances
	byteCount += alignUp8(meshCount * int(unsafe.Sizeof(meshInstance{})))

	meshMaterialMapOffset := byteCount
	byteCount += alignUp8(meshMaterialTotal * 2)

	// Packed shared mesh blobs
	for i := range sharedMeshCount {
		sharedMeshes[i].meshOffset = byteCount
		byteCount += alignUp8(int(sharedMeshes[i].meshData.ByteCount))
	}

	sphereOffset := byteCount
	byteCount += alignUp8(int(def.SphereCount) * int(unsafe.Sizeof(CompoundSphere{})))

	compound := (*CompoundData)(blobAlloc(byteCount))

	compound.Version = CompoundVersion
	compound.ByteCount = int32(byteCount)
	compound.NodeOffset = int32(nodeOffset)
	compound.Bounds = tree.GetRootBounds()
	compound.TreeHeight = int32(tree.GetHeight())
	compound.NodeCount = tree.NodeEnd
	compound.MaterialOffset = int32(materialOffset)
	compound.MaterialCount = int32(materialCount)
	compound.CapsuleOffset = int32(capsuleOffset)
	compound.CapsuleCount = int32(capsuleCount)
	compound.HullOffset = int32(hullArrayOffset)
	compound.HullCount = int32(hullCount)
	compound.MeshOffset = int32(meshArrayOffset)
	compound.MeshCount = int32(meshCount)
	compound.SphereOffset = int32(sphereOffset)
	compound.SphereCount = int32(sphereCount)

	base := unsafe.Pointer(compound)

	// Tree nodes
	nodes := blobSlice[TreeNode](base, int32(nodeOffset), int(tree.NodeEnd))
	copy(nodes, tree.Nodes[:tree.NodeEnd])

	// Materials
	assert(materialCount > 0, "materialCount > 0")
	destinationMaterials := blobSlice[SurfaceMaterial](base, compound.MaterialOffset, materialCount)
	if materials != nil {
		copy(destinationMaterials, materials[:materialCount])
	}

	// Capsules
	if def.CapsuleCount > 0 {
		assert(compound.CapsuleOffset > 0, "compound->capsuleOffset > 0")
		capsules := blobSlice[CompoundCapsule](base, compound.CapsuleOffset, capsuleCount)
		copy(capsules, capsuleInstances)
	}

	// Hulls
	for i := range hullCount {
		// Fix up offsets
		sharedIndex := int(hullInstances[i].hullOffset)
		assert(0 <= sharedIndex && sharedIndex < sharedHullCount, "0 <= sharedIndex && sharedIndex < sharedHullCount")
		hullInstances[i].hullOffset = uint32(sharedHulls[sharedIndex].hullOffset)
	}

	destinationHullInstances := blobSlice[hullInstance](base, int32(hullArrayOffset), hullCount)
	copy(destinationHullInstances, hullInstances)

	for i := range sharedHullCount {
		offset := sharedHulls[i].hullOffset
		destinationHull := blobBytes(unsafe.Add(base, offset), sharedHulls[i].hull.ByteCount)
		copy(destinationHull, hullBytes(sharedHulls[i].hull))
	}

	compound.SharedHullCount = int32(sharedHullCount)

	// Meshes
	for i := range meshCount {
		// Fix up offsets
		sharedIndex := int(meshInstances[i].meshOffset)
		assert(0 <= sharedIndex && sharedIndex < sharedMeshCount, "0 <= sharedIndex && sharedIndex < sharedMeshCount")
		meshInstances[i].meshOffset = uint32(sharedMeshes[sharedIndex].meshOffset)

		materialStart := meshInstances[i].materialOffset
		meshInstances[i].materialOffset = uint32(meshMaterialMapOffset) + materialStart*2
	}

	destinationMeshInstances := blobSlice[meshInstance](base, int32(meshArrayOffset), meshCount)
	copy(destinationMeshInstances, meshInstances)

	if meshMaterialTotal > 0 {
		destinationMeshMaterials := blobSlice[uint16](base, int32(meshMaterialMapOffset), meshMaterialTotal)
		copy(destinationMeshMaterials, meshMaterialStaging)
	}

	for i := range sharedMeshCount {
		offset := sharedMeshes[i].meshOffset
		destinationMesh := blobBytes(unsafe.Add(base, offset), sharedMeshes[i].meshData.ByteCount)
		copy(destinationMesh, meshBytes(sharedMeshes[i].meshData))
	}

	compound.SharedMeshCount = int32(sharedMeshCount)

	// Spheres
	if def.SphereCount > 0 {
		assert(compound.SphereOffset > 0, "compound->sphereOffset > 0")
		spheres := blobSlice[CompoundSphere](base, compound.SphereOffset, sphereCount)
		copy(spheres, sphereInstances)
	}

	materialMap.cleanup()
	tree.Destroy()

	return compound
}

// DestroyCompound destroys a compound shape.
// PORT: the Go garbage collector owns the blob; this is a no-op.
func DestroyCompound(compound *CompoundData) {
	_ = compound
}

// ValidateCompound casts serialized bytes into a compound and validates. Returns nil if invalid.
// PORT: b3ValidateCompound( const uint8_t* bytes, int byteCount ) keeps the explicit byte count
// (the tests pass a count that differs from the buffer). bytes must be 8-byte aligned and pointer
// free (any make([]byte, n) of at least 8 bytes is); the result aliases them.
func ValidateCompound(bytes []byte, byteCount int) *CompoundData {
	if bytes == nil || byteCount < int(unsafe.Sizeof(CompoundData{})) {
		return nil
	}

	compound := (*CompoundData)(unsafe.Pointer(unsafe.SliceData(bytes)))
	if compound.Version != CompoundVersion {
		return nil
	}

	if int(compound.ByteCount) != byteCount {
		return nil
	}

	if compound.NodeOffset < int32(unsafe.Sizeof(CompoundData{})) || (compound.NodeOffset&7) != 0 {
		return nil
	}

	if compound.NodeCount < 2 {
		return nil
	}

	nodeEnd := int64(compound.NodeOffset) + int64(compound.NodeCount)*int64(unsafe.Sizeof(TreeNode{}))
	if nodeEnd > int64(byteCount) {
		return nil
	}

	return compound
}

// ComputeCompoundAABB computes the bounding box of a transformed compound.
func ComputeCompoundAABB(shape *CompoundData, transform Transform) AABB {
	return shape.Bounds.Transform(transform)
}

// compoundOverlapContext is struct b3CompoundOverlapContext.
type compoundOverlapContext struct {
	compound *CompoundData
	// proxy in the compound frame
	proxy   ShapeProxy
	overlap bool
}

// compoundOverlapCallback is b3CompoundOverlapCallback.
func compoundOverlapCallback(proxyId int, userData uint64, context any) bool {
	_ = proxyId

	childIndex := int(userData)
	overlapContext := context.(*compoundOverlapContext)
	child := GetCompoundChild(overlapContext.compound, childIndex)

	overlap := false
	switch child.Type {
	case CapsuleShape:
		overlap = OverlapCapsule(&child.Capsule, child.Transform, &overlapContext.proxy)

	case HullShape:
		overlap = OverlapHull(child.Hull, child.Transform, &overlapContext.proxy)

	case MeshShape:
		overlap = OverlapMesh(&child.Mesh, child.Transform, &overlapContext.proxy)

	case SphereShape:
		overlap = OverlapSphere(&child.Sphere, child.Transform, &overlapContext.proxy)

	default:
		assert(false, "false")
	}

	if overlap {
		// Done
		overlapContext.overlap = true
		return false
	}

	// Continue the query if there is no overlap
	return true
}

// OverlapCompound overlaps a shape versus a compound.
// This is dealing with multiple transforms:
// - the compound shape transform
// - the compound child shape transforms
func OverlapCompound(shape *CompoundData, shapeTransform Transform, proxy *ShapeProxy) bool {
	assert(0 < proxy.Count && proxy.Count <= MaxShapeCastPoints, "0 < proxy->count && proxy->count <= B3_MAX_SHAPE_CAST_POINTS")

	// Use local proxy.
	var buffer [MaxShapeCastPoints]Vec3
	context := compoundOverlapContext{
		compound: shape,
		proxy:    makeLocalProxy(proxy, shapeTransform, buffer[:]),
		overlap:  false,
	}

	aabb := computeProxyAABB(&context.proxy)

	// This query must be in the compound frame.
	view := getCompoundTreeView(shape)
	_ = view.query(aabb, ^uint64(0), false, compoundOverlapCallback, &context)

	return context.overlap
}

// compoundCastContext is struct b3CompoundCastContext.
type compoundCastContext struct {
	compound *CompoundData
	output   *CastOutput
	// origin of the shape cast, the box cast callback only carries the advancing fraction
	shapeInput *ShapeCastInput
}

// compoundRayCastCallback is b3CompoundRayCastCallback.
func compoundRayCastCallback(input *RayCastInput, proxyId int, userData uint64, context any) float32 {
	_ = proxyId

	castContext := context.(*compoundCastContext)
	compound := castContext.compound

	childIndex := int(userData)

	child := GetCompoundChild(compound, childIndex)

	// Get the input in child local space.
	localInput := *input
	localInput.Origin = InvTransformPoint(child.Transform, input.Origin)
	localInput.Translation = InvRotateVector(child.Transform.Q, input.Translation)

	output := CastOutput{}

	switch child.Type {
	case CapsuleShape:
		output = RayCastCapsule(&child.Capsule, &localInput)
		output.MaterialIndex = int32(child.MaterialIndices[0])

	case HullShape:
		output = RayCastHull(child.Hull, &localInput)
		output.MaterialIndex = int32(child.MaterialIndices[0])

	case MeshShape:
		{
			output = RayCastMesh(&child.Mesh, &localInput)
			assert(0 <= output.MaterialIndex && output.MaterialIndex < child.MaterialCount,
				"0 <= output.materialIndex && output.materialIndex < child.materialCount")
			output.MaterialIndex = int32(child.MaterialIndices[output.MaterialIndex])
		}

	case SphereShape:
		output = RayCastSphere(&child.Sphere, &localInput)
		output.MaterialIndex = int32(child.MaterialIndices[0])

	default:
		assert(false, "false")
	}

	if output.Hit {
		output.Point = TransformPoint(child.Transform, output.Point)
		output.Normal = RotateVector(child.Transform.Q, output.Normal)
		output.ChildIndex = int32(childIndex)
		*castContext.output = output
		return output.Fraction
	}

	return input.MaxFraction
}

// RayCastCompound ray casts versus a compound in local space.
func RayCastCompound(shape *CompoundData, input *RayCastInput) CastOutput {
	result := CastOutput{}

	context := compoundCastContext{
		compound: shape,
		output:   &result,
	}
	view := getCompoundTreeView(shape)
	_ = view.rayCast(input, ^uint64(0), false, compoundRayCastCallback, &context)
	return result
}

// compoundShapeCastCallback is b3CompoundShapeCastCallback.
func compoundShapeCastCallback(input *BoxCastInput, proxyId int, userData uint64, context any) float32 {
	_ = proxyId

	castContext := context.(*compoundCastContext)
	compound := castContext.compound
	shapeInput := castContext.shapeInput

	childIndex := int(userData)

	child := GetCompoundChild(compound, childIndex)

	// Get the input in the child local space.
	localInput := ShapeCastInput{}
	var localPoints [MaxShapeCastPoints]Vec3
	localInput.Proxy = makeLocalProxy(&shapeInput.Proxy, child.Transform, localPoints[:])
	localInput.Translation = InvRotateVector(child.Transform.Q, shapeInput.Translation)
	localInput.MaxFraction = input.MaxFraction
	localInput.CanEncroach = shapeInput.CanEncroach

	output := CastOutput{}

	switch child.Type {
	case CapsuleShape:
		output = ShapeCastCapsule(&child.Capsule, &localInput)
		output.MaterialIndex = int32(child.MaterialIndices[0])

	case HullShape:
		output = ShapeCastHull(child.Hull, &localInput)
		output.MaterialIndex = int32(child.MaterialIndices[0])

	case MeshShape:
		{
			output = ShapeCastMesh(&child.Mesh, &localInput)
			assert(0 <= output.MaterialIndex && output.MaterialIndex < child.MaterialCount,
				"0 <= output.materialIndex && output.materialIndex < child.materialCount")
			output.MaterialIndex = int32(child.MaterialIndices[output.MaterialIndex])
		}

	case SphereShape:
		output = ShapeCastSphere(&child.Sphere, &localInput)
		output.MaterialIndex = int32(child.MaterialIndices[0])

	default:
		assert(false, "false")
	}

	if output.Hit {
		output.Point = TransformPoint(child.Transform, output.Point)
		output.Normal = RotateVector(child.Transform.Q, output.Normal)
		output.ChildIndex = int32(childIndex)
		*castContext.output = output
		return output.Fraction
	}

	return input.MaxFraction
}

// ShapeCastCompound shape casts versus a compound in local space.
func ShapeCastCompound(shape *CompoundData, input *ShapeCastInput) CastOutput {
	result := CastOutput{}

	if input.Proxy.Count == 0 {
		return result
	}

	context := compoundCastContext{
		compound:   shape,
		output:     &result,
		shapeInput: input,
	}

	// The compound tree is in the compound local frame, so the proxy box needs no origin offset
	box := MakeAABB(input.Proxy.Points[:input.Proxy.Count], input.Proxy.Radius)
	treeInput := BoxCastInput{box, input.Translation, input.MaxFraction}
	view := getCompoundTreeView(shape)
	_ = view.boxCast(&treeInput, ^uint64(0), false, compoundShapeCastCallback, &context)
	return result
}

// QueryCompound queries a compound shape for children that overlap an AABB.
func QueryCompound(compound *CompoundData, aabb AABB, fcn CompoundQueryFcn, context any) {
	view := getCompoundTreeView(compound)
	// A direct callback keeps nested visitors on the stack. Storing fcn in a
	// pooled context makes every caller's captured geometry escape to the heap.
	view.query(aabb, DefaultMaskBits, false, func(_ int, userData uint64, _ any) bool {
		return fcn(compound, int(userData), context)
	}, nil)
}

// PORT: the #if 0 block (b3CompoundImpactContext, b3CompoundTimeOfImpactFcn,
// b3CompoundTimeOfImpact) is not ported.

// makeCompoundChildSweep is b3MakeCompoundChildSweep.
// Transforms a sweep for a compound child shape
// xf = xfP * xfC
func makeCompoundChildSweep(compoundTransform, childTransform Transform) Sweep {
	xf := MulTransforms(compoundTransform, childTransform)
	return Sweep{
		LocalCenter: Vec3Zero,
		C1:          xf.P,
		C2:          xf.P,
		Q1:          xf.Q,
		Q2:          xf.Q,
	}
}

// compoundMoverContext is struct b3CompoundMoverContext.
type compoundMoverContext struct {
	compound      *CompoundData
	planes        []PlaneResult
	planeCapacity int
	planeCount    int
	mover         Capsule
}

// compoundMoverCallback is b3CompoundMoverCallback.
func compoundMoverCallback(proxyId int, userData uint64, context any) bool {
	_ = proxyId

	childIndex := int(userData)
	moverContext := context.(*compoundMoverContext)
	child := GetCompoundChild(moverContext.compound, childIndex)

	// Transform mover to child space
	var localMover Capsule
	localMover.Center1 = InvTransformPoint(child.Transform, moverContext.mover.Center1)
	localMover.Center2 = InvTransformPoint(child.Transform, moverContext.mover.Center2)
	localMover.Radius = moverContext.mover.Radius

	capacity := moverContext.planeCapacity - moverContext.planeCount
	assert(capacity > 0, "capacity > 0")

	planes := moverContext.planes[moverContext.planeCount:moverContext.planeCapacity]
	planeCount := 0

	switch child.Type {
	case CapsuleShape:
		planeCount = collideMoverAndCapsule(&planes[0], &child.Capsule, &localMover)

	case HullShape:
		planeCount = collideMoverAndHull(&planes[0], child.Hull, &localMover)

	case MeshShape:
		planeCount = collideMoverAndMesh(planes[:capacity], &child.Mesh, &localMover)

	case SphereShape:
		planeCount = collideMoverAndSphere(&planes[0], &child.Sphere, &localMover)

	default:
		assert(false, "false")
	}

	// Transform results back to shape space
	for i := range planeCount {
		planes[i].Plane.Normal = RotateVector(child.Transform.Q, planes[i].Plane.Normal)
		planes[i].Point = TransformPoint(child.Transform, planes[i].Point)

		planes[i].ChildIndex = int32(childIndex)
		assert(0 <= planes[i].MaterialIndex && planes[i].MaterialIndex < child.MaterialCount,
			"0 <= planes[i].materialIndex && planes[i].materialIndex < child.materialCount")
		planes[i].MaterialIndex = int32(child.MaterialIndices[planes[i].MaterialIndex])
	}

	moverContext.planeCount += planeCount

	// Continue query while there is room for more planes
	return moverContext.planeCount < moverContext.planeCapacity
}

// collideMoverAndCompound is b3CollideMoverAndCompound.
// PORT: b3PlaneResult* planes with capacity becomes a slice; capacity is len(planes).
func collideMoverAndCompound(planes []PlaneResult, shape *CompoundData, mover *Capsule) int {
	context := compoundMoverContext{
		compound:      shape,
		planes:        planes,
		planeCapacity: len(planes),
		planeCount:    0,
		mover:         *mover,
	}

	var aabb AABB
	aabb.LowerBound = Min(mover.Center1, mover.Center2)
	aabb.UpperBound = Max(mover.Center1, mover.Center2)
	r := Vec3{mover.Radius, mover.Radius, mover.Radius}
	aabb.LowerBound = Sub(aabb.LowerBound, r)
	aabb.UpperBound = Add(aabb.UpperBound, r)

	view := getCompoundTreeView(shape)
	_ = view.query(aabb, ^uint64(0), false, compoundMoverCallback, &context)

	return context.planeCount
}
