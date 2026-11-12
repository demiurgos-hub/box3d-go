// Port of src/shape.h and src/shape.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// ---- shape.h ----

// PORT: b3BroadPhase and b3World forward declarations are not needed in Go.

// shapeFlags is b3ShapeFlags.
type shapeFlags int32

const (
	enableSensorEvents    shapeFlags = 0x01
	enableContactEvents   shapeFlags = 0x02
	enableCustomFiltering shapeFlags = 0x04
	enableHitEvents       shapeFlags = 0x08
	enablePreSolveEvents  shapeFlags = 0x10
	// 0x20 is free, it was the enlarged AABB flag before the broad-phase marked the tree
	enableSpeculative shapeFlags = 0x40
)

// shape is b3Shape.
type shape struct {
	id             int32
	bodyId         int32
	prevShapeId    int32
	nextShapeId    int32
	sensorIndex    int32
	proxyKey       int32
	typ            ShapeType // PORT: type is a Go keyword
	density        float32
	explosionScale float32
	aabbMargin     float32

	aabb          AABB
	localCentroid Vec3

	materialCount int32
	material      SurfaceMaterial
	materials     []SurfaceMaterial // PORT: b3SurfaceMaterial* heap array; nil for a single material shape

	filter    Filter
	userData  any
	userShape any

	nameId     uint32
	generation uint16

	// b3ShapeFlags
	flags uint8

	// PORT: tagged union { capsule; sphere; hull; mesh; heightField; compound }. All members are
	// present; only the one selected by typ is meaningful.
	capsule     Capsule
	sphere      Sphere
	hull        *HullData
	mesh        Mesh
	heightField *HeightFieldData
	compound    *CompoundData
}

// getShapeMaterials is b3GetShapeMaterials.
// A single material shape keeps its material inline. Multi material meshes and compounds own a heap
// array. Reach the materials the same way for both: a single material shape presents its inline
// material as a one element array. Do not cache the pointer, the shapes array can move.
func getShapeMaterials(shape *shape) []SurfaceMaterial {
	if shape.materials != nil {
		return shape.materials
	}
	return unsafe.Slice(&shape.material, 1)
}

// PORT: the functions declared in shape.h (b3CreateShapeProxy, b3DestroyShapeProxy,
// b3DestroyShapeAllocations, b3ComputeShapeMass, b3ComputeShapeExtent, b3ComputeShapeMinExtent,
// b3ComputeShapeAABB, b3ComputeFatShapeAABB, b3ComputeSweptShapeAABB, b3GetShapeCentroid,
// b3GetShapeArea, b3GetShapeProjectedArea, b3GetShapeUserMaterialId, b3MakeShapeProxy,
// b3MakeLocalProxy, b3ComputeProxyAABB, b3RayCastShape, b3ShapeCastShape, b3OverlapShape,
// b3ShapeTimeOfImpact, b3CollideMover) are defined in the shape.c section below.
// b3ComputeSweptSphereAABB / Capsule, b3CollideMoverAndSphere / Capsule / Hull / Mesh / HeightField,
// the hull functions (b3FindHullSupportVertex, …) and the height field / mesh functions are defined
// in sphere.c, capsule.c, hull.c, mesh.c and height_field.c and are ported with those files.

// Height field

// getHeightFieldTriangleCount is b3GetHeightFieldTriangleCount.
func getHeightFieldTriangleCount(heightField *HeightFieldData) int {
	cellCount := int(heightField.RowCount-1) * int(heightField.ColumnCount-1)
	return 2 * cellCount
}

// Mesh

// getMeshTriangle is b3GetMeshTriangle.
// Making this inline made the tree benchmarks faster.
func getMeshTriangle(mesh *Mesh, triangleIndex int) triangle {
	assert(0 <= triangleIndex && triangleIndex < int(mesh.Data.TriangleCount), "0 <= triangleIndex && triangleIndex < mesh->data->triangleCount")

	triangles := GetMeshTriangles(mesh.Data)
	flags := GetMeshFlags(mesh.Data)
	vertices := GetMeshVertices(mesh.Data)

	var result triangle
	tri := triangles[triangleIndex]
	triangleFlags := flags[triangleIndex]

	scale := mesh.Scale

	result.vertices[0] = Mul(scale, vertices[tri.Index1])
	result.i1 = tri.Index1

	if scale.X*scale.Y*scale.Z < 0.0 {
		result.vertices[1] = Mul(scale, vertices[tri.Index3])
		result.vertices[2] = Mul(scale, vertices[tri.Index2])

		result.i2 = tri.Index3
		result.i3 = tri.Index2

		// mesh is inverted, so concave edges are now convex
		result.flags = 0
		if MeshEdgeFlags(triangleFlags)&InverseConcaveEdge1 != 0 {
			result.flags |= int32(ConcaveEdge1)
		}
		if MeshEdgeFlags(triangleFlags)&InverseConcaveEdge2 != 0 {
			result.flags |= int32(ConcaveEdge2)
		}
		if MeshEdgeFlags(triangleFlags)&InverseConcaveEdge3 != 0 {
			result.flags |= int32(ConcaveEdge3)
		}
	} else {
		result.vertices[1] = Mul(scale, vertices[tri.Index2])
		result.vertices[2] = Mul(scale, vertices[tri.Index3])

		result.i2 = tri.Index2
		result.i3 = tri.Index3
		result.flags = int32(triangleFlags)
	}

	return result
}

// isValidMesh (b3IsValidMesh) is implemented in mesh.go.

// shouldShapesCollide is b3ShouldShapesCollide.
func shouldShapesCollide(filterA, filterB Filter) bool {
	if filterA.GroupIndex == filterB.GroupIndex && filterA.GroupIndex != 0 {
		return filterA.GroupIndex > 0
	}

	return (filterA.MaskBits&filterB.CategoryBits) != 0 && (filterA.CategoryBits&filterB.MaskBits) != 0
}

// shouldQueryCollide is b3ShouldQueryCollide.
func shouldQueryCollide(shapeFilter *Filter, queryFilter *QueryFilter) bool {
	return (shapeFilter.CategoryBits&queryFilter.MaskBits) != 0 &&
		(queryFilter.IgnoreShapeMask || (shapeFilter.MaskBits&queryFilter.CategoryBits) != 0)
}

// isConvex is b3IsConvex.
func isConvex(typ ShapeType) bool {
	return typ == SphereShape || typ == CapsuleShape || typ == HullShape
}

// ---- shape.c ----

// getShape is b3GetShape.
func getShape(world *world, shapeId ShapeID) *shape {
	id := int(shapeId.Index1 - 1)
	shape := arrayGet(world.shapes, id)
	assert(int(shape.id) == id && shape.generation == shapeId.Generation, "shape->id == id && shape->generation == shapeId.generation")
	return shape
}

// computeShapeMargin is b3ComputeShapeMargin.
func computeShapeMargin(shape *shape) float32 {
	var margin float32 = 0.0

	switch shape.typ {
	case SphereShape:
		margin = shape.sphere.Radius

	case CapsuleShape:
		margin = mulAdd(shape.capsule.Radius, 0.5, Distance(shape.capsule.Center2, shape.capsule.Center1))

	case HullShape:
		hull := shape.hull
		points := GetHullPoints(hull)
		var maxExtentSqr float32 = 0.0
		count := int(hull.VertexCount)
		for i := range count {
			distSqr := DistanceSquared(points[i], hull.Center)
			maxExtentSqr = MaxFloat(maxExtentSqr, distSqr)
		}
		margin = sqrtf(maxExtentSqr)

	case MeshShape, HeightShape, CompoundShape:
		// Static-only shapes: broadphase uses speculative distance for static
		// proxies, so the per-shape margin is never consumed in practice.
		// Return the cap so any incidental use is generous.
		return MaxAABBMargin()

	default:
		validate(false, "false")
		return MaxAABBMargin()
	}

	return MinFloat(MaxAABBMargin(), AABBMarginFraction*margin)
}

// updateShapeAABBs is b3UpdateShapeAABBs.
func updateShapeAABBs(shape *shape, fatAABB *AABB, transform WorldTransform, proxyType BodyType) {
	// Compute a bounding box with a speculative margin
	speculativeDistance := SpeculativeDistance()
	aabbMargin := shape.aabbMargin

	aabb := computeFatShapeAABB(shape, transform, speculativeDistance)
	shape.aabb = aabb

	// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
	margin := aabbMargin
	if proxyType == StaticBody {
		margin = speculativeDistance
	}
	*fatAABB = aabb.Inflate(margin)
}

// createShapeInternal is b3CreateShapeInternal.
// PORT: const void* geometry is any holding *Sphere, *Capsule, *HullData, *MeshData,
// *HeightFieldData or *CompoundData, selected by shapeType as upstream's casts.
func createShapeInternal(world *world, body *body, bodyTransform WorldTransform, def *ShapeDef, geometry any,
	shapeType ShapeType, shapeTransform Transform, scale Vec3, haveShapeTransform bool) *shape {
	shapeId := allocId(&world.shapeIdPool)

	if shapeId == len(world.shapes) {
		arrayPush(&world.shapes, shape{})
		arrayPush(&world.fatAABBs, AABB{})
	} else {
		assert(world.shapes[shapeId].id == NullIndex, "world->shapes.data[shapeId].id == B3_NULL_INDEX")
	}

	assert(len(world.fatAABBs) == len(world.shapes), "world->fatAABBs.count == world->shapes.count")

	shape := arrayGet(world.shapes, shapeId)

	switch shapeType {
	case CapsuleShape:
		shape.capsule = *geometry.(*Capsule)

	case CompoundShape:
		// Compounds must be a static and not a sensor
		assert(body.typ == StaticBody, "body->type == b3_staticBody")
		assert(def.IsSensor == false, "def->isSensor == false")
		shape.compound = geometry.(*CompoundData)

	case SphereShape:
		shape.sphere = *geometry.(*Sphere)

	case HullShape:
		if haveShapeTransform {
			// The transform and non-uniform scale are baked into fresh data, then shared.
			baked := CloneAndTransformHull(geometry.(*HullData), shapeTransform, scale)
			if baked == nil {
				// This can fail to produce a valid hull in extreme cases
				freeId(&world.shapeIdPool, shapeId)
				shape.id = NullIndex
				return nil
			}

			shape.hull = addOwnedHullToDatabase(world, baked)
		} else {
			shape.hull = addHullToDatabase(world, geometry.(*HullData))
		}

	case MeshShape:
		shape.mesh.Data = geometry.(*MeshData)
		shape.mesh.Scale = SafeScale(scale)

	case HeightShape:
		shape.heightField = geometry.(*HeightFieldData)

	default:
		assert(false, "false")
	}

	shape.id = int32(shapeId)
	shape.bodyId = body.id
	shape.typ = shapeType
	shape.density = def.Density
	shape.explosionScale = def.ExplosionScale
	shape.filter = def.Filter
	shape.userData = def.UserData
	shape.userShape = nil
	shape.flags = 0
	if def.EnableSensorEvents {
		shape.flags |= uint8(enableSensorEvents)
	}
	if def.EnableContactEvents {
		shape.flags |= uint8(enableContactEvents)
	}
	if def.EnableCustomFiltering {
		shape.flags |= uint8(enableCustomFiltering)
	}
	if def.EnableHitEvents {
		shape.flags |= uint8(enableHitEvents)
	}
	if def.EnablePreSolveEvents {
		shape.flags |= uint8(enablePreSolveEvents)
	}
	if def.EnableSpeculativeContact {
		shape.flags |= uint8(enableSpeculative)
	}
	shape.proxyKey = NullIndex
	shape.localCentroid = getShapeCentroid(shape)
	shape.aabbMargin = computeShapeMargin(shape)
	shape.aabb = AABB{Vec3Zero, Vec3Zero}
	world.fatAABBs[shapeId] = AABB{Vec3Zero, Vec3Zero}
	shape.nameId = addName(&world.names, def.Name)
	shape.generation += 1

	if shape.typ == CompoundShape {
		world.compoundShapeCount += 1

		// Own a copy of the compound materials so every shape frees its array the same way. Compounds
		// are few, so the copy is cheap and avoids aliasing the geometry blob.
		// PORT: b3Alloc + memcpy is make + copy.
		materialCount := shape.compound.MaterialCount
		shape.materialCount = materialCount
		shape.materials = make([]SurfaceMaterial, materialCount)
		copy(shape.materials, GetCompoundMaterials(shape.compound)[:materialCount])
	} else if def.MaterialCount > 1 && def.Materials != nil {
		// Per triangle materials need a heap array.
		// PORT: b3Alloc + memcpy is make + copy.
		shape.materialCount = def.MaterialCount
		shape.materials = make([]SurfaceMaterial, def.MaterialCount)
		copy(shape.materials, def.Materials[:def.MaterialCount])
	} else {
		// The common case is one material, stored inline with no allocation.
		if def.MaterialCount == 1 && def.Materials != nil {
			shape.material = def.Materials[0]
		} else {
			shape.material = def.BaseMaterial
		}
		shape.materialCount = 1
		shape.materials = nil
	}

	if body.setIndex != disabledSet {
		proxyType := body.typ
		forcePairCreation := def.InvokeContactCreation && shape.typ != CompoundShape
		createShapeProxy(world, shape, proxyType, bodyTransform, forcePairCreation)
	}

	// Add to shape doubly linked list
	if body.headShapeId != NullIndex {
		headShape := arrayGet(world.shapes, int(body.headShapeId))
		headShape.prevShapeId = int32(shapeId)
	}

	shape.prevShapeId = NullIndex
	shape.nextShapeId = body.headShapeId
	body.headShapeId = int32(shapeId)
	body.shapeCount += 1

	if def.IsSensor {
		shape.sensorIndex = int32(len(world.sensors))
		sensor := arrayEmplace(&world.sensors)
		arrayCreateN(&sensor.hits, 4)
		arrayCreateN(&sensor.overlaps1, 16)
		arrayCreateN(&sensor.overlaps2, 16)
		sensor.shapeId = int32(shapeId)
	} else {
		shape.sensorIndex = NullIndex
	}

	validateSolverSets(world)

	return shape
}

// createShape is b3CreateShape.
// PORT: const void* geometry is any (see createShapeInternal).
func createShape(bodyId BodyID, def *ShapeDef, geometry any, shapeType ShapeType, transform Transform, scale Vec3,
	haveTransform bool) ShapeID {
	checkDef(def.internalValue)
	assert(IsValidFloat(def.Density) && def.Density >= 0.0, "b3IsValidFloat( def->density ) && def->density >= 0.0f")
	assert(IsValidFloat(def.BaseMaterial.Friction) && def.BaseMaterial.Friction >= 0.0,
		"b3IsValidFloat( def->baseMaterial.friction ) && def->baseMaterial.friction >= 0.0f")
	assert(IsValidFloat(def.BaseMaterial.Restitution) && def.BaseMaterial.Restitution >= 0.0,
		"b3IsValidFloat( def->baseMaterial.restitution ) && def->baseMaterial.restitution >= 0.0f")

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return ShapeID{}
	}

	if len(world.shapes) == MaxShapes && len(world.shapeIdPool.freeArray) == 0 {
		assert(false, "false")
		return NullShapeID
	}

	body := getBodyFullId(world, bodyId)
	if body.typ != StaticBody && (shapeType == CompoundShape || shapeType == HeightShape) {
		// Compound and height shapes must be on static bodies.
		return NullShapeID
	}

	world.locked = true

	bodyTransform := getBodyTransformQuick(world, body)

	shape := createShapeInternal(world, body, bodyTransform, def, geometry, shapeType, transform, scale, haveTransform)

	if shape == nil {
		world.locked = false
		return NullShapeID
	}

	if def.UpdateBodyMass == true {
		updateBodyMassData(world, body)
	} else if (body.flags & dirtyMass) == 0 {
		body.flags |= dirtyMass
		syncBodyFlags(world, body)
	}

	validateSolverSets(world)

	id := ShapeID{shape.id + 1, bodyId.World0, shape.generation}

	world.locked = false

	return id
}

// CreateSphereShape is b3CreateSphereShape.
// Create a sphere shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// Returns the shape id for accessing the shape
func (bodyId BodyID) CreateSphereShape(def *ShapeDef, sphere *Sphere) ShapeID {
	shapeId := createShape(bodyId, def, sphere, SphereShape, TransformIdentity, Vec3One, false)
	if shapeId.Index1 != 0 {
		world := getUnlockedWorld(int(bodyId.World0))
		if world != nil {
			// PORT: B3_REC_CREATE expanded.
			if world.recording != nil {
				recCreateArgs := recArgsCreateSphereShape{bodyId, *def, *sphere}
				recWriteRetCreateSphereShape(world.recording, &recCreateArgs, shapeId)
			}
		}
	}
	return shapeId
}

// CreateCapsuleShape is b3CreateCapsuleShape.
// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// Returns the shape id for accessing the shape
func (bodyId BodyID) CreateCapsuleShape(def *ShapeDef, capsule *Capsule) ShapeID {
	lengthSqr := DistanceSquared(capsule.Center1, capsule.Center2)
	var shapeId ShapeID
	if lengthSqr <= float32(LinearSlop()*LinearSlop()) {
		sphere := Sphere{Lerp(capsule.Center1, capsule.Center2, 0.5), capsule.Radius}
		shapeId = createShape(bodyId, def, &sphere, SphereShape, TransformIdentity, Vec3One, false)
	} else {
		shapeId = createShape(bodyId, def, capsule, CapsuleShape, TransformIdentity, Vec3One, false)
	}
	if shapeId.Index1 != 0 {
		world := getUnlockedWorld(int(bodyId.World0))
		if world != nil {
			// PORT: B3_REC_CREATE expanded.
			if world.recording != nil {
				recCreateArgs := recArgsCreateCapsuleShape{bodyId, *def, *capsule}
				recWriteRetCreateCapsuleShape(world.recording, &recCreateArgs, shapeId)
			}
		}
	}
	return shapeId
}

// CreateHullShape is b3CreateHullShape.
// Create a convex hull shape and attach it to a body. The hull data is added to the world's shared
// hull database, so the caller may free it after this call.
// Contacts are not created until the next time step.
// Returns the shape id for accessing the shape
func (bodyId BodyID) CreateHullShape(def *ShapeDef, hull *HullData) ShapeID {
	validate(isValidHull(hull), "b3IsValidHull( hull )")
	assert(hull.Version == HullVersion, "hull->version == B3_HULL_VERSION")
	assert(hull.Hash != 0, "hull->hash != 0")

	if hull.Version != HullVersion {
		return NullShapeID
	}

	shapeId := createShape(bodyId, def, hull, HullShape, TransformIdentity, Vec3One, false)
	if shapeId.Index1 != 0 {
		world := getUnlockedWorld(int(bodyId.World0))
		if world != nil && world.recording != nil {
			geometryId := recInternHull(world.recording, hull)
			createArgs := recArgsCreateHullShape{bodyId, *def, geometryId}
			recWriteRetCreateHullShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

// CreateTransformedHullShape is b3CreateTransformedHullShape.
// Create a convex hull shape with a local transform and non-uniform scale baked into the hull data.
// Contacts are not created until the next time step.
// Returns the shape id for accessing the shape
func (bodyId BodyID) CreateTransformedHullShape(def *ShapeDef, hull *HullData, transform Transform, scale Vec3) ShapeID {
	validate(isValidHull(hull), "b3IsValidHull( hull )")
	assert(hull.Version == HullVersion, "hull->version == B3_HULL_VERSION")
	assert(hull.Hash != 0, "hull->hash != 0")

	if hull.Version != HullVersion {
		return NullShapeID
	}

	shapeId := createShape(bodyId, def, hull, HullShape, transform, scale, true)
	if shapeId.Index1 != 0 {
		world := getUnlockedWorld(int(bodyId.World0))
		if world != nil && world.recording != nil {
			// The transform and scale are baked into fresh hull data at create time. Record the baked hull
			// as a plain hull shape so replay rebuilds identical geometry with no rebake, and the keyframe
			// registry, which interns the live baked hull, stays seeded.
			shape := arrayGet(world.shapes, int(shapeId.Index1-1))
			geometryId := recInternHull(world.recording, shape.hull)
			createArgs := recArgsCreateHullShape{bodyId, *def, geometryId}
			recWriteRetCreateHullShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

// CreateMeshShape is b3CreateMeshShape.
// Create a mesh shape and attach it to a body. The mesh data is not cloned and must outlive the shape.
// Contacts are not created until the next time step.
// Returns the shape id for accessing the shape
func (bodyId BodyID) CreateMeshShape(def *ShapeDef, mesh *MeshData, scale Vec3) ShapeID {
	validate(isValidMesh(mesh), "b3IsValidMesh( mesh )")
	assert(mesh.Version == MeshVersion, "mesh->version == B3_MESH_VERSION")
	assert(mesh.Hash != 0, "mesh->hash != 0")

	if mesh.Version != MeshVersion {
		return NullShapeID
	}

	shapeId := createShape(bodyId, def, mesh, MeshShape, TransformIdentity, scale, true)
	if shapeId.Index1 != 0 {
		world := getUnlockedWorld(int(bodyId.World0))
		if world != nil && world.recording != nil {
			geometryId := recInternMesh(world.recording, mesh)
			createArgs := recArgsCreateMeshShape{bodyId, *def, geometryId, scale}
			recWriteRetCreateMeshShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

// CreateHeightFieldShape is b3CreateHeightFieldShape.
// Create a height field shape and attach it to a static body. The height field data is not cloned
// and must outlive the shape.
// Returns the shape id for accessing the shape
func (bodyId BodyID) CreateHeightFieldShape(def *ShapeDef, heightField *HeightFieldData) ShapeID {
	assert(heightField.Version == HeightFieldVersion, "heightField->version == B3_HEIGHT_FIELD_VERSION")
	assert(heightField.Hash != 0, "heightField->hash != 0")

	if heightField.Version != HeightFieldVersion {
		return NullShapeID
	}

	shapeId := createShape(bodyId, def, heightField, HeightShape, TransformIdentity, Vec3One, false)
	if shapeId.Index1 != 0 {
		world := getUnlockedWorld(int(bodyId.World0))
		if world != nil && world.recording != nil {
			geometryId := recInternHeightField(world.recording, heightField)
			createArgs := recArgsCreateHeightFieldShape{bodyId, *def, geometryId}
			recWriteRetCreateHeightFieldShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

// CreateBakedCompoundShape is b3CreateBakedCompoundShape.
// Create a compound shape and attach it to a static body. The compound data is not cloned and must
// outlive the shape.
// Returns the shape id for accessing the shape
func (bodyId BodyID) CreateBakedCompoundShape(def *ShapeDef, compound *CompoundData) ShapeID {
	assert(compound.Version == CompoundVersion, "compound->version == B3_COMPOUND_VERSION")

	if compound.Version != CompoundVersion {
		return NullShapeID
	}

	shapeId := createShape(bodyId, def, compound, CompoundShape, TransformIdentity, Vec3One, false)
	if shapeId.Index1 != 0 {
		world := getUnlockedWorld(int(bodyId.World0))
		if world != nil && world.recording != nil {
			geometryId := recInternCompound(world.recording, compound)
			createArgs := recArgsCreateCompoundShape{bodyId, *def, geometryId}
			recWriteRetCreateCompoundShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

// destroyShapeInternal is b3DestroyShapeInternal.
// Destroy a shape on a body. This doesn't need to be called when destroying a body.
func destroyShapeInternal(world *world, shape *shape, body *body, wakeBodies bool) {
	shapeId := shape.id

	if shape.typ == CompoundShape {
		assert(world.compoundShapeCount > 0, "world->compoundShapeCount > 0")
		world.compoundShapeCount -= 1
	}

	// Remove the shape from the body's doubly linked list.
	if shape.prevShapeId != NullIndex {
		prevShape := arrayGet(world.shapes, int(shape.prevShapeId))
		prevShape.nextShapeId = shape.nextShapeId
	}

	if shape.nextShapeId != NullIndex {
		nextShape := arrayGet(world.shapes, int(shape.nextShapeId))
		nextShape.prevShapeId = shape.prevShapeId
	}

	if shapeId == body.headShapeId {
		body.headShapeId = shape.nextShapeId
	}

	body.shapeCount -= 1

	// Remove from broad-phase.
	destroyShapeProxy(shape, &world.broadPhase)

	// Destroy any contacts associated with the shape.
	contactKey := body.headContactKey
	for contactKey != NullIndex {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := arrayGet(world.contacts, int(contactId))
		contactKey = contact.edges[edgeIndex].nextKey

		if contact.shapeIdA == shapeId || contact.shapeIdB == shapeId {
			destroyContact(world, contact, wakeBodies)
		}
	}

	if shape.sensorIndex != NullIndex {
		sensor := arrayGet(world.sensors, int(shape.sensorIndex))
		for i := range len(sensor.overlaps2) {
			ref := &sensor.overlaps2[i]
			event := SensorEndTouchEvent{
				SensorShapeId: ShapeID{
					Index1:     shapeId + 1,
					World0:     world.worldId,
					Generation: shape.generation,
				},
				VisitorShapeId: ShapeID{
					Index1:     ref.shapeId + 1,
					World0:     world.worldId,
					Generation: ref.generation,
				},
			}

			arrayPush(&world.sensorEndEvents[world.endEventArrayIndex], event)
		}

		// Destroy sensor
		arrayDestroy(&sensor.hits)
		arrayDestroy(&sensor.overlaps1)
		arrayDestroy(&sensor.overlaps2)

		movedIndex := arrayRemoveSwap(&world.sensors, int(shape.sensorIndex))
		if movedIndex != NullIndex {
			// Fixup moved sensor
			movedSensor := arrayGet(world.sensors, int(shape.sensorIndex))
			otherSensorShape := arrayGet(world.shapes, int(movedSensor.shapeId))
			otherSensorShape.sensorIndex = shape.sensorIndex
		}
	}

	// Destroy every shape member from b3Alloc
	destroyShapeAllocations(world, shape)

	// Return shape to free list.
	freeId(&world.shapeIdPool, int(shapeId))
	shape.id = NullIndex

	validateSolverSets(world)
}

// Destroy is b3DestroyShape.
// Destroy a shape. You may defer the body mass update which can improve performance if several shapes on a
// body are destroyed at once.
// See also: BodyID.ApplyMassFromShapes
func (shapeId ShapeID) Destroy(updateBodyMass bool) {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsDestroyShape{shapeId, updateBodyMass}
		recWriteDestroyShape(world.recording, &recArgs)
	}

	world.locked = true

	shape := getShape(world, shapeId)

	// need to wake bodies because this might be a static body
	wakeBodies := true

	body := arrayGet(world.bodies, int(shape.bodyId))
	destroyShapeInternal(world, shape, body, wakeBodies)

	if updateBodyMass == true {
		updateBodyMassData(world, body)
	}

	world.locked = false
}

// computeShapeAABB is b3ComputeShapeAABB.
func computeShapeAABB(shape *shape, transform Transform) AABB {
	switch shape.typ {
	case CapsuleShape:
		return ComputeCapsuleAABB(&shape.capsule, transform)

	case CompoundShape:
		return ComputeCompoundAABB(shape.compound, transform)

	case HeightShape:
		return ComputeHeightFieldQueryAABB(shape.heightField, transform)

	case HullShape:
		return ComputeHullAABB(shape.hull, transform)

	case MeshShape:
		return ComputeMeshAABB(shape.mesh.Data, transform, shape.mesh.Scale)

	case SphereShape:
		return ComputeSphereAABB(&shape.sphere, transform)

	default:
		assert(false, "false")
		empty := AABB{transform.P, transform.P}
		return empty
	}
}

// computeFatShapeAABB is b3ComputeFatShapeAABB.
// PORT: only the single precision branch (BOX3D_DOUBLE_PRECISION is not ported). There
// b3WorldTransform is a typedef of b3Transform; here the world transform is converted exactly.
func computeFatShapeAABB(shape *shape, transform WorldTransform, extra float32) AABB {
	r := Vec3{extra, extra, extra}
	aabb := computeShapeAABB(shape, Transform{ToVec3(transform.P), transform.Q})
	aabb.LowerBound = Sub(aabb.LowerBound, r)
	aabb.UpperBound = Add(aabb.UpperBound, r)
	return aabb
}

// computeSweptShapeAABB is b3ComputeSweptShapeAABB.
func computeSweptShapeAABB(shape *shape, sweep *Sweep, time float32) AABB {
	assert(0.0 <= time && time <= 1.0, "0.0f <= time && time <= 1.0f")
	xf1 := Transform{Sub(sweep.C1, RotateVector(sweep.Q1, sweep.LocalCenter)), sweep.Q1}
	xf2 := GetSweepTransform(sweep, time)

	switch shape.typ {
	case CapsuleShape:
		return computeSweptCapsuleAABB(&shape.capsule, xf1, xf2)

	case HullShape:
		return computeSweptHullAABB(shape.hull, xf1, xf2)

	case SphereShape:
		return computeSweptSphereAABB(&shape.sphere, xf1, xf2)

	default:
		assert(false, "false")
		return AABB{xf1.P, xf1.P}
	}
}

// getShapeCentroid is b3GetShapeCentroid.
func getShapeCentroid(shape *shape) Vec3 {
	switch shape.typ {
	case CapsuleShape:
		return Lerp(shape.capsule.Center1, shape.capsule.Center2, 0.5)
	case CompoundShape:
		aabb := ComputeCompoundAABB(shape.compound, TransformIdentity)
		return aabb.Center()
	case SphereShape:
		return shape.sphere.Center
	case HullShape:
		return shape.hull.Center
	case MeshShape:
		aabb := ComputeMeshAABB(shape.mesh.Data, TransformIdentity, shape.mesh.Scale)
		return aabb.Center()
	case HeightShape:
		aabb := ComputeHeightFieldAABB(shape.heightField, TransformIdentity)
		return aabb.Center()
	default:
		return Vec3Zero
	}
}

// getShapeArea is b3GetShapeArea.
func getShapeArea(shape *shape) float32 {
	// todo_erin fix these
	switch shape.typ {
	case CapsuleShape:
		return float32(2.0*Length(Sub(shape.capsule.Center1, shape.capsule.Center2))) +
			float32(2.0*Pi*shape.capsule.Radius)

	case HullShape:
		return shape.hull.SurfaceArea

	case SphereShape:
		return 2.0 * Pi * shape.sphere.Radius

	default:
		return 0.0
	}
}

// getShapeProjectedArea is b3GetShapeProjectedArea.
// This projects the shape surface area onto a plane
func getShapeProjectedArea(shape *shape, planeNormal Vec3) float32 {
	switch shape.typ {
	case CapsuleShape:
		radius := shape.capsule.Radius
		axis := Sub(shape.capsule.Center2, shape.capsule.Center1)
		projectedLength := Length(Cross(axis, planeNormal))
		cylinderArea := float32(2.0 * radius * projectedLength)
		sphereArea := float32(Pi * radius * radius)
		return sphereArea + cylinderArea

	case HullShape:
		return computeHullProjectedArea(shape.hull, planeNormal)

	case SphereShape:
		return Pi * shape.sphere.Radius * shape.sphere.Radius

	default:
		return 0.0
	}
}

// computeShapeMass is b3ComputeShapeMass.
func computeShapeMass(shape *shape) MassData {
	switch shape.typ {
	case CapsuleShape:
		return ComputeCapsuleMass(&shape.capsule, shape.density)

	case HullShape:
		return ComputeHullMass(shape.hull, shape.density)

	case SphereShape:
		return ComputeSphereMass(&shape.sphere, shape.density)

	default:
		return MassData{}
	}
}

// computeShapeExtent is b3ComputeShapeExtent.
func computeShapeExtent(shape *shape, localCenter Vec3) shapeExtent {
	extent := shapeExtent{}

	switch shape.typ {
	case CapsuleShape:
		radius := shape.capsule.Radius
		extent.minExtent = radius
		c1 := Sub(shape.capsule.Center1, localCenter)
		c2 := Sub(shape.capsule.Center2, localCenter)
		r := Vec3{radius, radius, radius}
		extent.maxExtent = Add(Max(Abs(c1), Abs(c2)), r)

	case CompoundShape:
		// This is shouldn't be needed but here for completeness
		aabb := ComputeCompoundAABB(shape.compound, TransformIdentity)
		r1 := Length(Sub(aabb.LowerBound, localCenter))
		r2 := Length(Sub(aabb.UpperBound, localCenter))
		extent.minExtent = MinFloat(r1, r2)
		p := farthestPointOnAABB(aabb, localCenter)
		extent.maxExtent = Abs(Sub(p, localCenter))

	case SphereShape:
		radius := shape.sphere.Radius
		extent.minExtent = radius
		h := Abs(Sub(shape.sphere.Center, localCenter))
		r := Vec3{radius, radius, radius}
		extent.maxExtent = Add(h, r)

	case HullShape:
		extent = computeHullExtent(shape.hull, localCenter)

	case MeshShape:
		// This is needed for kinematic mesh sleeping
		aabb := ComputeMeshAABB(shape.mesh.Data, TransformIdentity, shape.mesh.Scale)
		r1 := Length(Sub(aabb.LowerBound, localCenter))
		r2 := Length(Sub(aabb.UpperBound, localCenter))
		extent.minExtent = MinFloat(r1, r2)
		p := farthestPointOnAABB(aabb, localCenter)
		extent.maxExtent = Abs(Sub(p, localCenter))

	default:
	}

	return extent
}

// computeShapeMinExtent is b3ComputeShapeMinExtent.
func computeShapeMinExtent(shape *shape, localCenter Vec3) float32 {
	switch shape.typ {
	case CapsuleShape:
		return shape.capsule.Radius

	case SphereShape:
		return shape.sphere.Radius

	case HullShape:
		return shape.hull.InnerRadius

	default:
		return computeShapeExtent(shape, localCenter).minExtent
	}
}

// rayCastShape is b3RayCastShape.
func rayCastShape(shape *shape, transform Transform, input *RayCastInput) CastOutput {
	localInput := *input
	localInput.Origin = InvTransformPoint(transform, input.Origin)
	localInput.Translation = InvRotateVector(transform.Q, input.Translation)

	output := CastOutput{}
	switch shape.typ {
	case CapsuleShape:
		output = RayCastCapsule(&shape.capsule, &localInput)
	case CompoundShape:
		output = RayCastCompound(shape.compound, &localInput)
	case SphereShape:
		output = RayCastSphere(&shape.sphere, &localInput)
	case HullShape:
		output = RayCastHull(shape.hull, &localInput)
	case MeshShape:
		output = RayCastMesh(&shape.mesh, &localInput)
	case HeightShape:
		output = RayCastHeightField(shape.heightField, &localInput)
	default:
		return output
	}

	output.Point = TransformPoint(transform, output.Point)
	output.Normal = RotateVector(transform.Q, output.Normal)
	return output
}

// shapeCastShape is b3ShapeCastShape.
func shapeCastShape(shape *shape, transform Transform, input *ShapeCastInput) CastOutput {
	localInput := *input
	var localPoints [MaxShapeCastPoints]Vec3

	localInput.Proxy.Count = int32(MinInt(int(input.Proxy.Count), MaxShapeCastPoints))
	for i := range int(localInput.Proxy.Count) {
		localPoints[i] = InvTransformPoint(transform, input.Proxy.Points[i])
	}

	localInput.Proxy.Points = localPoints[:]
	localInput.Translation = InvRotateVector(transform.Q, input.Translation)

	output := CastOutput{}
	switch shape.typ {
	case CapsuleShape:
		output = ShapeCastCapsule(&shape.capsule, &localInput)

	case CompoundShape:
		output = ShapeCastCompound(shape.compound, &localInput)

	case HeightShape:
		output = ShapeCastHeightField(shape.heightField, &localInput)

	case HullShape:
		output = ShapeCastHull(shape.hull, &localInput)

	case MeshShape:
		output = ShapeCastMesh(&shape.mesh, &localInput)

	case SphereShape:
		output = ShapeCastSphere(&shape.sphere, &localInput)
	default:
		return output
	}

	output.Point = TransformPoint(transform, output.Point)
	output.Normal = RotateVector(transform.Q, output.Normal)
	return output
}

// overlapShape is b3OverlapShape.
func overlapShape(shape *shape, transform Transform, proxy *ShapeProxy) bool {
	typ := shape.typ
	switch typ {
	case CapsuleShape:
		return OverlapCapsule(&shape.capsule, transform, proxy)

	case CompoundShape:
		return OverlapCompound(shape.compound, transform, proxy)

	case HeightShape:
		return OverlapHeightField(shape.heightField, transform, proxy)

	case HullShape:
		return OverlapHull(shape.hull, transform, proxy)

	case MeshShape:
		return OverlapMesh(&shape.mesh, transform, proxy)

	case SphereShape:
		return OverlapSphere(&shape.sphere, transform, proxy)

	default:
		assert(false, "false")
		return false
	}
}

// collideMover is b3CollideMover.
// PORT: planes + planeCapacity is a slice; the capacity is len(planes).
func collideMover(planes []PlaneResult, shape *shape, transform Transform, mover *Capsule) int {
	planeCapacity := len(planes)
	if planeCapacity == 0 {
		return 0
	}

	var localMover Capsule
	localMover.Center1 = InvTransformPoint(transform, mover.Center1)
	localMover.Center2 = InvTransformPoint(transform, mover.Center2)
	localMover.Radius = mover.Radius

	planeCount := 0
	switch shape.typ {
	case CapsuleShape:
		planeCount = collideMoverAndCapsule(&planes[0], &shape.capsule, &localMover)

	case CompoundShape:
		planeCount = collideMoverAndCompound(planes[:planeCapacity], shape.compound, &localMover)

	case SphereShape:
		planeCount = collideMoverAndSphere(&planes[0], &shape.sphere, &localMover)

	case HullShape:
		planeCount = collideMoverAndHull(&planes[0], shape.hull, &localMover)

	case MeshShape:
		planeCount = collideMoverAndMesh(planes[:planeCapacity], &shape.mesh, &localMover)

	case HeightShape:
		planeCount = collideMoverAndHeightField(planes[:planeCapacity], shape.heightField, &localMover)

	default:
		assert(false, "false")
	}

	for i := range planeCount {
		planes[i].Plane.Normal = RotateVector(transform.Q, planes[i].Plane.Normal)
		planes[i].Point = TransformPoint(transform, planes[i].Point)
		planes[i].MaterialIndex = int32(ClampInt(int(planes[i].MaterialIndex), 0, int(shape.materialCount)-1))
	}

	return planeCount
}

// createShapeProxy is b3CreateShapeProxy.
func createShapeProxy(world *world, shape *shape, typ BodyType, transform WorldTransform, forcePairCreation bool) {
	assert(shape.proxyKey == NullIndex, "shape->proxyKey == B3_NULL_INDEX")

	fatAABB := &world.fatAABBs[shape.id]
	updateShapeAABBs(shape, fatAABB, transform, typ)

	// Create proxies in the broad-phase.
	shape.proxyKey = int32(world.broadPhase.createProxy(typ, *fatAABB, shape.filter.CategoryBits, int(shape.id),
		forcePairCreation))
	assert(proxyType(int(shape.proxyKey)) < BodyTypeCount, "B3_PROXY_TYPE( shape->proxyKey ) < b3_bodyTypeCount")
}

// destroyShapeProxy is b3DestroyShapeProxy.
func destroyShapeProxy(shape *shape, bp *broadPhase) {
	if shape.proxyKey != NullIndex {
		bp.destroyProxy(int(shape.proxyKey))
		shape.proxyKey = NullIndex
	}
}

// destroyShapeAllocationForShapeChange is b3DestroyShapeAllocationForShapeChange.
func destroyShapeAllocationForShapeChange(world *world, shape *shape) {
	typ := shape.typ
	switch typ {
	case HullShape:
		removeHullFromDatabase(world, shape.hull)
		shape.hull = nil

	default:
	}

	if shape.userShape != nil {
		world.destroyDebugShape(shape.userShape, world.userDebugShapeContext)
		shape.userShape = nil
	}
}

// destroyShapeAllocations is b3DestroyShapeAllocations.
func destroyShapeAllocations(world *world, shape *shape) {
	destroyShapeAllocationForShapeChange(world, shape)

	if shape.materials != nil {
		assert(shape.materialCount > 0, "shape->materialCount > 0")
		// PORT: b3Free is dropping the slice.
		shape.materials = nil
		shape.materialCount = 0
	}

	// Name is stored inline. Sensor data is destroyed elsewhere
}

// makeShapeProxy is b3MakeShapeProxy.
// PORT: the capsule and sphere proxies alias the shape's centers (capsulePoints, spherePoints).
func makeShapeProxy(shape *shape) ShapeProxy {
	switch shape.typ {
	case CapsuleShape:
		return ShapeProxy{capsulePoints(&shape.capsule), 2, shape.capsule.Radius}

	case SphereShape:
		return ShapeProxy{spherePoints(&shape.sphere), 1, shape.sphere.Radius}

	case HullShape:
		hull := shape.hull
		points := GetHullPoints(hull)
		return ShapeProxy{points, hull.VertexCount, 0.0}

	default:
		assert(false, "false")
		return ShapeProxy{}
	}
}

// makeLocalProxy is b3MakeLocalProxy.
// PORT: buffer is the caller's [MaxShapeCastPoints]Vec3 array; the returned proxy points into it.
func makeLocalProxy(proxy *ShapeProxy, transform Transform, buffer []Vec3) ShapeProxy {
	invTransform := InvertTransform(transform)
	R := MakeMatrixFromQuat(invTransform.Q)

	count := MinInt(int(proxy.Count), MaxShapeCastPoints)
	for i := range count {
		buffer[i] = Add(MulMV(R, proxy.Points[i]), invTransform.P)
	}

	return ShapeProxy{
		Points: buffer,
		Count:  int32(count),
		Radius: proxy.Radius,
	}
}

// computeProxyAABB is b3ComputeProxyAABB.
func computeProxyAABB(proxy *ShapeProxy) AABB {
	points := proxy.Points
	aabb := AABB{
		LowerBound: points[0],
		UpperBound: points[0],
	}

	for i := 1; i < int(proxy.Count); i++ {
		aabb.LowerBound = Min(aabb.LowerBound, points[i])
		aabb.UpperBound = Max(aabb.UpperBound, points[i])
	}

	r := Vec3{proxy.Radius, proxy.Radius, proxy.Radius}
	aabb.LowerBound = Sub(aabb.LowerBound, r)
	aabb.UpperBound = Add(aabb.UpperBound, r)
	return aabb
}

// GetBody is b3Shape_GetBody: get the id of the body that a shape is attached to.
func (shapeId ShapeID) GetBody() BodyID {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return makeBodyId(world, int(shape.bodyId))
}

// GetWorld is b3Shape_GetWorld: get the world that owns this shape.
func (shapeId ShapeID) GetWorld() WorldID {
	world := getWorld(int(shapeId.World0))
	return WorldID{uint16(shapeId.World0 + 1), world.generation}
}

// SetUserData is b3Shape_SetUserData: set the user data for a shape.
func (shapeId ShapeID) SetUserData(userData any) {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	shape.userData = userData
}

// GetUserData is b3Shape_GetUserData: get the user data for a shape. This is useful when you get a
// shape id from an event or query.
func (shapeId ShapeID) GetUserData() any {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.userData
}

// SetName is b3Shape_SetName: set the shape name. The empty string clears it.
// PORT: const char* is a string; NULL is "".
func (shapeId ShapeID) SetName(name string) {
	world := getWorld(int(shapeId.World0))

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetName{shapeId, name}
		recWriteShapeSetName(world.recording, &recArgs)
	}

	shape := getShape(world, shapeId)
	shape.nameId = addName(&world.names, name)
}

// GetName is b3Shape_GetName: get the shape name, "" if it has none.
func (shapeId ShapeID) GetName() string {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return findNameWithDefault(&world.names, shape.nameId, "")
}

// IsSensor is b3Shape_IsSensor: returns true if the shape is a sensor.
func (shapeId ShapeID) IsSensor() bool {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.sensorIndex != NullIndex
}

// RayCast is b3Shape_RayCast: ray cast a shape directly.
// todo no tests
func (shapeId ShapeID) RayCast(origin Pos, translation Vec3) WorldCastOutput {
	assert(IsValidPosition(origin), "b3IsValidPosition( origin )")
	assert(IsValidVec3(translation), "b3IsValidVec3( translation )")

	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)

	// Re-center on the origin so the cast runs in float precision far from the world origin
	transform := ToRelativeTransform(getBodyTransform(world, int(shape.bodyId)), origin)

	// The ray starts at the origin, so its origin in the re-centered frame is zero
	input := RayCastInput{Vec3Zero, translation, 1.0}

	// Lift the re-centered float result back to a world position
	local := rayCastShape(shape, transform, &input)
	var output WorldCastOutput
	output.Normal = local.Normal
	output.Point = OffsetPos(origin, local.Point)
	output.Fraction = local.Fraction
	output.Iterations = local.Iterations
	output.TriangleIndex = local.TriangleIndex
	output.ChildIndex = local.ChildIndex
	output.MaterialIndex = local.MaterialIndex
	output.Hit = local.Hit

	return output
}

// SetDensity is b3Shape_SetDensity: set the mass density of a shape, usually in kg/m^3.
// This will optionally update the mass properties on the parent body.
func (shapeId ShapeID) SetDensity(density float32, updateBodyMass bool) {
	assert(IsValidFloat(density) && density >= 0.0, "b3IsValidFloat( density ) && density >= 0.0f")

	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetDensity{shapeId, density, updateBodyMass}
		recWriteShapeSetDensity(world.recording, &recArgs)
	}

	shape := getShape(world, shapeId)
	if density == shape.density {
		// early return to avoid expensive function
		return
	}

	shape.density = density

	if updateBodyMass == true {
		body := arrayGet(world.bodies, int(shape.bodyId))
		updateBodyMassData(world, body)
	}
}

// GetDensity is b3Shape_GetDensity: get the density of a shape, usually in kg/m^3.
func (shapeId ShapeID) GetDensity() float32 {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.density
}

// SetFriction is b3Shape_SetFriction: set the friction on a shape.
func (shapeId ShapeID) SetFriction(friction float32) {
	assert(IsValidFloat(friction) && friction >= 0.0, "b3IsValidFloat( friction ) && friction >= 0.0f")
	world := getWorld(int(shapeId.World0))
	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetFriction{shapeId, friction}
		recWriteShapeSetFriction(world.recording, &recArgs)
	}
	shape := getShape(world, shapeId)
	assert(shape.typ != CompoundShape, "shape->type != b3_compoundShape")
	getShapeMaterials(shape)[0].Friction = friction
}

// GetFriction is b3Shape_GetFriction: get the friction of a shape.
func (shapeId ShapeID) GetFriction() float32 {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return getShapeMaterials(shape)[0].Friction
}

// SetRestitution is b3Shape_SetRestitution: set the shape restitution (bounciness).
func (shapeId ShapeID) SetRestitution(restitution float32) {
	assert(IsValidFloat(restitution) && restitution >= 0.0, "b3IsValidFloat( restitution ) && restitution >= 0.0f")
	world := getWorld(int(shapeId.World0))
	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetRestitution{shapeId, restitution}
		recWriteShapeSetRestitution(world.recording, &recArgs)
	}
	shape := getShape(world, shapeId)
	assert(shape.typ != CompoundShape, "shape->type != b3_compoundShape")
	getShapeMaterials(shape)[0].Restitution = restitution
}

// GetRestitution is b3Shape_GetRestitution: get the shape restitution.
func (shapeId ShapeID) GetRestitution() float32 {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return getShapeMaterials(shape)[0].Restitution
}

// SetSurfaceMaterial is b3Shape_SetSurfaceMaterial: set the shape surface material.
func (shapeId ShapeID) SetSurfaceMaterial(surfaceMaterial SurfaceMaterial) {
	assert(IsValidFloat(surfaceMaterial.Friction) && surfaceMaterial.Friction >= 0.0,
		"b3IsValidFloat( surfaceMaterial.friction ) && surfaceMaterial.friction >= 0.0f")
	assert(IsValidFloat(surfaceMaterial.Restitution) && surfaceMaterial.Restitution >= 0.0,
		"b3IsValidFloat( surfaceMaterial.restitution ) && surfaceMaterial.restitution >= 0.0f")
	assert(IsValidFloat(surfaceMaterial.RollingResistance) && surfaceMaterial.RollingResistance >= 0.0,
		"b3IsValidFloat( surfaceMaterial.rollingResistance ) && surfaceMaterial.rollingResistance >= 0.0f")
	assert(IsValidVec3(surfaceMaterial.TangentVelocity), "b3IsValidVec3( surfaceMaterial.tangentVelocity )")

	world := getWorld(int(shapeId.World0))
	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetSurfaceMaterial{shapeId, surfaceMaterial}
		recWriteShapeSetSurfaceMaterial(world.recording, &recArgs)
	}
	shape := getShape(world, shapeId)
	assert(shape.typ != CompoundShape, "shape->type != b3_compoundShape")
	getShapeMaterials(shape)[0] = surfaceMaterial
}

// GetSurfaceMaterial is b3Shape_GetSurfaceMaterial: get the shape surface material.
func (shapeId ShapeID) GetSurfaceMaterial() SurfaceMaterial {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return getShapeMaterials(shape)[0]
}

// GetMeshMaterialCount is b3Shape_GetMeshMaterialCount: get the number of materials on a shape.
func (shapeId ShapeID) GetMeshMaterialCount() int {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return int(shape.materialCount)
}

// SetMeshMaterial is b3Shape_SetMeshMaterial: set a mesh shape material by index.
func (shapeId ShapeID) SetMeshMaterial(surfaceMaterial SurfaceMaterial, index int) {
	assert(IsValidFloat(surfaceMaterial.Friction) && surfaceMaterial.Friction >= 0.0,
		"b3IsValidFloat( surfaceMaterial.friction ) && surfaceMaterial.friction >= 0.0f")
	assert(IsValidFloat(surfaceMaterial.Restitution) && surfaceMaterial.Restitution >= 0.0,
		"b3IsValidFloat( surfaceMaterial.restitution ) && surfaceMaterial.restitution >= 0.0f")
	assert(IsValidFloat(surfaceMaterial.RollingResistance) && surfaceMaterial.RollingResistance >= 0.0,
		"b3IsValidFloat( surfaceMaterial.rollingResistance ) && surfaceMaterial.rollingResistance >= 0.0f")
	assert(IsValidVec3(surfaceMaterial.TangentVelocity), "b3IsValidVec3( surfaceMaterial.tangentVelocity )")

	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)

	assert(0 <= index && index < int(shape.materialCount), "0 <= index && index < shape->materialCount")
	assert(shape.typ != CompoundShape, "shape->type != b3_compoundShape")

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetMeshMaterial{shapeId, surfaceMaterial, int32(index)}
		recWriteShapeSetMeshMaterial(world.recording, &recArgs)
	}
	getShapeMaterials(shape)[index] = surfaceMaterial
}

// GetMeshSurfaceMaterial is b3Shape_GetMeshSurfaceMaterial: get a mesh shape material by index.
func (shapeId ShapeID) GetMeshSurfaceMaterial(index int) SurfaceMaterial {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	assert(0 <= index && index < int(shape.materialCount), "0 <= index && index < shape->materialCount")
	return getShapeMaterials(shape)[index]
}

// GetFilter is b3Shape_GetFilter: get the shape filter.
func (shapeId ShapeID) GetFilter() Filter {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.filter
}

// resetProxy is b3ResetProxy.
func resetProxy(world *world, shape *shape, wakeBodies bool, destroyProxy bool) {
	body := arrayGet(world.bodies, int(shape.bodyId))

	shapeId := shape.id

	// destroy all contacts associated with this shape
	contactKey := body.headContactKey
	for contactKey != NullIndex {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := arrayGet(world.contacts, int(contactId))
		contactKey = contact.edges[edgeIndex].nextKey

		if contact.shapeIdA == shapeId || contact.shapeIdB == shapeId {
			destroyContact(world, contact, wakeBodies)
		}
	}

	transform := getBodyTransformQuick(world, body)
	if shape.proxyKey != NullIndex {
		proxyType := proxyType(int(shape.proxyKey))
		fatAABB := &world.fatAABBs[shapeId]
		updateShapeAABBs(shape, fatAABB, transform, proxyType)

		if destroyProxy {
			world.broadPhase.destroyProxy(int(shape.proxyKey))

			forcePairCreation := true
			shape.proxyKey = int32(world.broadPhase.createProxy(proxyType, *fatAABB, shape.filter.CategoryBits,
				int(shapeId), forcePairCreation))
		} else {
			world.broadPhase.moveProxy(int(shape.proxyKey), *fatAABB)
		}
	} else {
		proxyType := body.typ
		updateShapeAABBs(shape, &world.fatAABBs[shapeId], transform, proxyType)
	}

	validateSolverSets(world)
}

// SetFilter is b3Shape_SetFilter: set the current filter. This is almost as expensive as recreating
// the shape. This may cause contacts to be immediately destroyed. However contacts are not created
// until the next world step. Sensor overlap state is also not updated until the next world step.
// invokeContacts: should this invoke contacts to be created during the next time step
func (shapeId ShapeID) SetFilter(filter Filter, invokeContacts bool) {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetFilter{shapeId, filter, invokeContacts}
		recWriteShapeSetFilter(world.recording, &recArgs)
	}

	shape := getShape(world, shapeId)
	if filter.MaskBits == shape.filter.MaskBits && filter.CategoryBits == shape.filter.CategoryBits &&
		filter.GroupIndex == shape.filter.GroupIndex {
		return
	}

	shape.filter = filter

	if invokeContacts {
		world.locked = true
		wakeBodies := true

		// If the category bits change, I need to destroy the proxy because it affects the tree sorting.
		// PORT: upstream compares after shape->filter = filter, so this is always true (ported as is,
		// see LOOSE_ENDS.md).
		destroyProxy := filter.CategoryBits == shape.filter.CategoryBits

		// need to wake bodies because a filter change may destroy contacts
		resetProxy(world, shape, wakeBodies, destroyProxy)
		world.locked = false
	}

	// note: this does not immediately update sensor overlaps. Instead sensor
	// overlaps are updated the next time step
}

// EnableSensorEvents is b3Shape_EnableSensorEvents: enable sensor events for this shape.
// See also: ShapeDef.EnableSensorEvents
func (shapeId ShapeID) EnableSensorEvents(flag bool) {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeEnableSensorEvents{shapeId, flag}
		recWriteShapeEnableSensorEvents(world.recording, &recArgs)
	}

	shape := getShape(world, shapeId)
	if flag {
		shape.flags = shape.flags | uint8(enableSensorEvents)
	} else {
		shape.flags = shape.flags &^ uint8(enableSensorEvents)
	}
}

// AreSensorEventsEnabled is b3Shape_AreSensorEventsEnabled: returns true if sensor events are enabled.
func (shapeId ShapeID) AreSensorEventsEnabled() bool {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.flags&uint8(enableSensorEvents) != 0
}

// EnableContactEvents is b3Shape_EnableContactEvents: enable contact events for this shape. Only
// applies to kinematic and dynamic bodies. Ignored for sensors.
// See also: ShapeDef.EnableContactEvents
func (shapeId ShapeID) EnableContactEvents(flag bool) {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeEnableContactEvents{shapeId, flag}
		recWriteShapeEnableContactEvents(world.recording, &recArgs)
	}

	shape := getShape(world, shapeId)
	if flag {
		shape.flags = shape.flags | uint8(enableContactEvents)
	} else {
		shape.flags = shape.flags &^ uint8(enableContactEvents)
	}
}

// AreContactEventsEnabled is b3Shape_AreContactEventsEnabled: returns true if contact events are
// enabled.
func (shapeId ShapeID) AreContactEventsEnabled() bool {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.flags&uint8(enableContactEvents) != 0
}

// EnablePreSolveEvents is b3Shape_EnablePreSolveEvents: enable pre-solve contact events for this
// shape. Only applies to dynamic bodies. These are expensive and must be carefully handled due to
// multithreading. Ignored for sensors.
// See also: PreSolveFcn
func (shapeId ShapeID) EnablePreSolveEvents(flag bool) {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeEnablePreSolveEvents{shapeId, flag}
		recWriteShapeEnablePreSolveEvents(world.recording, &recArgs)
	}

	shape := getShape(world, shapeId)
	if flag {
		shape.flags = shape.flags | uint8(enablePreSolveEvents)
	} else {
		shape.flags = shape.flags &^ uint8(enablePreSolveEvents)
	}
}

// ArePreSolveEventsEnabled is b3Shape_ArePreSolveEventsEnabled: returns true if pre-solve events
// are enabled.
func (shapeId ShapeID) ArePreSolveEventsEnabled() bool {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.flags&uint8(enablePreSolveEvents) != 0
}

// EnableHitEvents is b3Shape_EnableHitEvents: enable contact hit events for this shape. Ignored for
// sensors.
// See also: WorldDef.HitEventThreshold
func (shapeId ShapeID) EnableHitEvents(flag bool) {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeEnableHitEvents{shapeId, flag}
		recWriteShapeEnableHitEvents(world.recording, &recArgs)
	}

	shape := getShape(world, shapeId)
	if flag {
		shape.flags = shape.flags | uint8(enableHitEvents)
	} else {
		shape.flags = shape.flags &^ uint8(enableHitEvents)
	}
}

// AreHitEventsEnabled is b3Shape_AreHitEventsEnabled: returns true if hit events are enabled.
func (shapeId ShapeID) AreHitEventsEnabled() bool {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.flags&uint8(enableHitEvents) != 0
}

// GetType is b3Shape_GetType: get the type of a shape.
func (shapeId ShapeID) GetType() ShapeType {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	return shape.typ
}

// GetSphere is b3Shape_GetSphere: get a copy of the shape's sphere. Asserts the type is correct.
func (shapeId ShapeID) GetSphere() Sphere {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	assert(shape.typ == SphereShape, "shape->type == b3_sphereShape")
	return shape.sphere
}

// GetCapsule is b3Shape_GetCapsule: get a copy of the shape's capsule. Asserts the type is correct.
func (shapeId ShapeID) GetCapsule() Capsule {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	assert(shape.typ == CapsuleShape, "shape->type == b3_capsuleShape")
	return shape.capsule
}

// GetHull is b3Shape_GetHull: get the shape's convex hull. Asserts the type is correct.
func (shapeId ShapeID) GetHull() *HullData {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	assert(shape.typ == HullShape, "shape->type == b3_hullShape")
	return shape.hull
}

// GetMesh is b3Shape_GetMesh: get the shape's mesh. Asserts the type is correct.
func (shapeId ShapeID) GetMesh() Mesh {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	assert(shape.typ == MeshShape, "shape->type == b3_meshShape")
	return shape.mesh
}

// GetHeightField is b3Shape_GetHeightField: get the shape's height field. Asserts the type is
// correct.
func (shapeId ShapeID) GetHeightField() *HeightFieldData {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	assert(shape.typ == HeightShape, "shape->type == b3_heightShape")
	return shape.heightField
}

// GetCompound is b3Shape_GetCompound: get the shape's compound. Asserts the type is correct.
func (shapeId ShapeID) GetCompound() *CompoundData {
	world := getWorld(int(shapeId.World0))
	shape := getShape(world, shapeId)
	assert(shape.typ == CompoundShape, "shape->type == b3_compoundShape")
	return shape.compound
}

// SetSphere is b3Shape_SetSphere: allows you to change a shape to be a sphere or update the current
// sphere. This does not modify the mass properties.
// See also: BodyID.ApplyMassFromShapes
func (shapeId ShapeID) SetSphere(sphere *Sphere) {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetSphere{shapeId, *sphere}
		recWriteShapeSetSphere(world.recording, &recArgs)
	}

	world.locked = true

	shape := getShape(world, shapeId)

	destroyShapeAllocationForShapeChange(world, shape)

	shape.sphere = *sphere
	shape.typ = SphereShape
	shape.aabbMargin = computeShapeMargin(shape)

	// need to wake bodies so they can react to the shape change
	wakeBodies := true
	destroyProxy := true
	resetProxy(world, shape, wakeBodies, destroyProxy)

	world.locked = false
}

// SetCapsule is b3Shape_SetCapsule: allows you to change a shape to be a capsule or update the
// current capsule. This does not modify the mass properties.
// See also: BodyID.ApplyMassFromShapes
func (shapeId ShapeID) SetCapsule(capsule *Capsule) {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeSetCapsule{shapeId, *capsule}
		recWriteShapeSetCapsule(world.recording, &recArgs)
	}

	world.locked = true

	shape := getShape(world, shapeId)

	destroyShapeAllocationForShapeChange(world, shape)

	shape.capsule = *capsule
	shape.typ = CapsuleShape
	shape.aabbMargin = computeShapeMargin(shape)

	// need to wake bodies so they can react to the shape change
	wakeBodies := true
	destroyProxy := true
	resetProxy(world, shape, wakeBodies, destroyProxy)

	world.locked = false
}

// SetHull is b3Shape_SetHull: allows you to change a shape to be a convex hull or update the
// current hull. The hull is added to the world's shared hull database. This does not modify the
// mass properties.
// See also: BodyID.ApplyMassFromShapes
func (shapeId ShapeID) SetHull(hull *HullData) {
	validate(isValidHull(hull), "b3IsValidHull( hull )")
	validate(hull.Hash != 0, "hull->hash != 0")

	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	world.locked = true

	shape := getShape(world, shapeId)

	// Acquire the new hull before releasing the old so the input may safely alias
	// the shape's current shared data.
	data := addHullToDatabase(world, hull)

	// Same shared hull, avoid destroying contacts and recreating the proxy
	if shape.typ == HullShape && data == shape.hull {
		removeHullFromDatabase(world, data)
		world.locked = false
		return
	}

	if world.recording != nil {
		// Intern the shared hull.
		geometryId := recInternHull(world.recording, data)
		setArgs := recArgsShapeSetHull{shapeId, geometryId}
		recWriteShapeSetHull(world.recording, &setArgs)
	}

	destroyShapeAllocationForShapeChange(world, shape)

	shape.hull = data
	shape.typ = HullShape
	shape.aabbMargin = computeShapeMargin(shape)

	// need to wake bodies so they can react to the shape change
	wakeBodies := true
	destroyProxy := true
	resetProxy(world, shape, wakeBodies, destroyProxy)

	world.locked = false
}

// SetMesh is b3Shape_SetMesh: allows you to change a shape to be a mesh or update the current mesh.
// The mesh data is not cloned and must outlive the shape. This does not modify the mass
// properties.
// See also: BodyID.ApplyMassFromShapes
func (shapeId ShapeID) SetMesh(meshData *MeshData, scale Vec3) {
	assert(IsValidVec3(scale), "b3IsValidVec3( scale )")
	assert(meshData != nil && isValidMesh(meshData), "meshData != NULL && b3IsValidMesh( meshData )")

	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	world.locked = true

	if world.recording != nil {
		geometryId := recInternMesh(world.recording, meshData)
		setArgs := recArgsShapeSetMesh{shapeId, geometryId, scale}
		recWriteShapeSetMesh(world.recording, &setArgs)
	}

	shape := getShape(world, shapeId)

	destroyShapeAllocationForShapeChange(world, shape)

	shape.mesh.Data = meshData
	shape.mesh.Scale = SafeScale(scale)
	shape.typ = MeshShape
	shape.aabbMargin = computeShapeMargin(shape)

	// need to wake bodies so they can react to the shape change
	wakeBodies := true
	destroyProxy := true
	resetProxy(world, shape, wakeBodies, destroyProxy)

	world.locked = false
}

// GetContactCapacity is b3Shape_GetContactCapacity: get the maximum capacity required for
// retrieving all the touching contacts on a shape.
func (shapeId ShapeID) GetContactCapacity() int {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return 0
	}

	shape := getShape(world, shapeId)
	if shape.sensorIndex != NullIndex {
		return 0
	}

	body := arrayGet(world.bodies, int(shape.bodyId))

	// Conservative and fast
	return int(body.contactCount)
}

// GetContactData is b3Shape_GetContactData: get the touching contact data for a shape. The
// provided shapeId will be either shapeIdA or shapeIdB on the contact data.
// PORT: contactData + capacity is a slice; the capacity is len(contactData).
func (shapeId ShapeID) GetContactData(contactData []ContactData) int {
	capacity := len(contactData)
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return 0
	}

	shape := getShape(world, shapeId)
	if shape.sensorIndex != NullIndex {
		return 0
	}

	body := arrayGet(world.bodies, int(shape.bodyId))
	contactKey := body.headContactKey
	index := 0
	for contactKey != NullIndex && index < capacity {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := arrayGet(world.contacts, int(contactId))

		// Does contact involve this shape and is it touching?
		if (contact.shapeIdA == shapeId.Index1-1 || contact.shapeIdB == shapeId.Index1-1) &&
			(contact.flags&contactTouchingFlag) != 0 {
			shapeA := &world.shapes[contact.shapeIdA]
			shapeB := &world.shapes[contact.shapeIdB]

			contactData[index].ContactId = ContactID{contact.contactId + 1, shapeId.World0, 0, contact.generation}
			contactData[index].ShapeIdA = ShapeID{shapeA.id + 1, shapeId.World0, shapeA.generation}
			contactData[index].ShapeIdB = ShapeID{shapeB.id + 1, shapeId.World0, shapeB.generation}
			contactData[index].Manifolds = contact.manifolds
			contactData[index].ManifoldCount = contact.manifoldCount
			index += 1
		}

		contactKey = contact.edges[edgeIndex].nextKey
	}

	assert(index <= capacity, "index <= capacity")

	return index
}

// GetSensorCapacity is b3Shape_GetSensorCapacity: get the maximum capacity required for retrieving
// all the overlapped shapes on a sensor shape. This returns 0 if the provided shape is not a sensor.
func (shapeId ShapeID) GetSensorCapacity() int {
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return 0
	}

	shape := getShape(world, shapeId)
	if shape.sensorIndex == NullIndex {
		return 0
	}

	sensor := arrayGet(world.sensors, int(shape.sensorIndex))
	return len(sensor.overlaps2)
}

// GetSensorData is b3Shape_GetSensorData: get the overlapped shapes for a sensor shape. Returns the
// number of elements filled in the provided slice.
// Warning: do not ignore the return value, it specifies the valid number of elements
// PORT: visitorIds + capacity is a slice; the capacity is len(visitorIds).
func (shapeId ShapeID) GetSensorData(visitorIds []ShapeID) int {
	capacity := len(visitorIds)
	world := getUnlockedWorld(int(shapeId.World0))
	if world == nil {
		return 0
	}

	shape := getShape(world, shapeId)
	if shape.sensorIndex == NullIndex {
		return 0
	}

	sensor := arrayGet(world.sensors, int(shape.sensorIndex))

	count := MinInt(len(sensor.overlaps2), capacity)
	refs := sensor.overlaps2
	for i := range count {
		visitorId := ShapeID{
			Index1:     refs[i].shapeId + 1,
			World0:     shapeId.World0,
			Generation: refs[i].generation,
		}

		visitorIds[i] = visitorId
	}

	return count
}

// GetAABB is b3Shape_GetAABB: get the current world AABB.
func (shapeId ShapeID) GetAABB() AABB {
	world := getWorld(int(shapeId.World0))
	if world == nil {
		return AABB{}
	}

	shape := getShape(world, shapeId)
	return shape.aabb
}

// ComputeMassData is b3Shape_ComputeMassData: compute the mass data for a shape.
func (shapeId ShapeID) ComputeMassData() MassData {
	world := getWorld(int(shapeId.World0))
	if world == nil {
		return MassData{}
	}

	shape := getShape(world, shapeId)
	return computeShapeMass(shape)
}

// GetClosestPoint is b3Shape_GetClosestPoint: get the closest point on a shape to a target point.
// Target and result are in world space.
func (shapeId ShapeID) GetClosestPoint(target Vec3) Vec3 {
	world := getWorld(int(shapeId.World0))
	if world == nil {
		return Vec3Zero
	}

	shape := getShape(world, shapeId)
	body := arrayGet(world.bodies, int(shape.bodyId))
	// Low level closest point query is a documented float carve-out far from the origin
	transform := ToRelativeTransform(getBodyTransformQuick(world, body), PosZero)

	var input DistanceInput
	input.ProxyA = makeShapeProxy(shape)
	// PORT: ( b3ShapeProxy ){ &target, 1, 0.0f } aliases the parameter with unsafe.Slice.
	input.ProxyB = ShapeProxy{unsafe.Slice(&target, 1), 1, 0.0}
	input.Transform = InvMulTransforms(transform, TransformIdentity)
	input.UseRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil)

	// Witness point comes back in frame A, lift it back to the query frame
	return TransformPoint(transform, output.PointA)
}

// debugWind is B3_DEBUG_WIND.
const debugWind = false

// ApplyWind is b3Shape_ApplyWind: apply a wind force to the body for this shape using the density
// of air. This considers the projected area of the shape in the wind direction. This also
// considers the relative velocity of the shape.
// wind: the wind velocity in world space
// drag: the drag coefficient, the force that opposes the relative velocity
// lift: the lift coefficient, the force that is perpendicular to the relative velocity
// maxSpeed: the maximum relative speed used in the force computation
// wake: should this wake the body
//
// https://en.wikipedia.org/wiki/Density_of_air
// https://www.engineeringtoolbox.com/wind-load-d_1775.html
// force = 0.5 * air_density * velocity^2 * area
// https://en.wikipedia.org/wiki/Lift_(force)
func (shapeId ShapeID) ApplyWind(wind Vec3, drag, lift, maxSpeed float32, wake bool) {
	world := getWorld(int(shapeId.World0))
	if world == nil {
		return
	}

	// PORT: B3_REC expanded.
	if world.recording != nil {
		recArgs := recArgsShapeApplyWind{shapeId, wind, drag, lift, maxSpeed, wake}
		recWriteShapeApplyWind(world.recording, &recArgs)
	}

	shape := getShape(world, shapeId)

	shapeType := shape.typ
	if shapeType != SphereShape && shapeType != CapsuleShape && shapeType != HullShape {
		return
	}

	body := arrayGet(world.bodies, int(shape.bodyId))

	if body.typ != DynamicBody {
		return
	}

	if body.setIndex == disabledSet {
		return
	}

	if body.setIndex >= firstSleepingSet && wake == false {
		return
	}

	// PORT: upstream reads the sim before waking the body, so after a wake the force and torque
	// below land on the sleeping set's old sim (a dangling pointer in C, a detached copy here).
	// Ported as is, see LOOSE_ENDS.md.
	sim := getBodySim(world, body)

	if body.setIndex != awakeSet {
		// Must wake for state to exist
		wakeBodyWithLock(world, body)
	}

	assert(body.setIndex == awakeSet, "body->setIndex == b3_awakeSet")

	state := getBodyState(world, body)
	// Only the rotation is used below, so the demoted world transform is exact
	transform := ToRelativeTransform(sim.transform, PosZero)

	lengthUnits := GetLengthUnitsPerMeter()
	volumeUnits := lengthUnits * lengthUnits * lengthUnits

	airDensity := 1.2250 / (volumeUnits)

	force := Vec3{}
	torque := Vec3{}

	switch shape.typ {
	case SphereShape:
		radius := shape.sphere.Radius
		centroid := shape.localCentroid
		lever := RotateVector(transform.Q, Sub(centroid, sim.localCenter))
		shapeVelocity := Add(state.linearVelocity, Cross(state.angularVelocity, lever))
		relativeVelocity := MulSub(wind, drag, shapeVelocity)
		direction, speed := GetLengthAndNormalize(relativeVelocity)
		speed = MinFloat(speed, maxSpeed)
		projectedArea := Pi * radius * radius
		force = MulSV(0.5*airDensity*projectedArea*speed*speed, direction)
		torque = Cross(lever, force)

	case CapsuleShape:
		centroid := shape.localCentroid
		lever := RotateVector(transform.Q, Sub(centroid, sim.localCenter))
		shapeVelocity := Add(state.linearVelocity, Cross(state.angularVelocity, lever))
		relativeVelocity := MulSub(wind, drag, shapeVelocity)
		direction, speed := GetLengthAndNormalize(relativeVelocity)
		speed = MinFloat(speed, maxSpeed)

		d := Sub(shape.capsule.Center2, shape.capsule.Center1)
		d = RotateVector(transform.Q, d)

		radius := shape.capsule.Radius
		projectedArea := float32(Pi*radius*radius) + float32(2.0*radius*Length(Cross(d, direction)))

		// Normal that opposes the wind
		e := Normalize(d)
		normal := Sub(MulSV(Dot(direction, e), e), direction)

		// portion of wind that is perpendicular to surface
		liftDirection := Cross(Cross(normal, direction), direction)

		forceMagnitude := 0.5 * airDensity * projectedArea * speed * speed
		force = MulSV(forceMagnitude, MulAdd(direction, lift, liftDirection))

		edgeLever := MulAdd(lever, radius, normal)
		torque = Cross(edgeLever, force)

	case HullShape:
		matrix := MakeMatrixFromQuat(transform.Q)

		faceCount := int(shape.hull.FaceCount)
		points := GetHullPoints(shape.hull)
		faces := GetHullFaces(shape.hull)
		edges := GetHullEdges(shape.hull)
		planes := GetHullPlanes(shape.hull)

		linearVelocity := state.linearVelocity
		angularVelocity := state.angularVelocity
		localCenterOfMass := sim.localCenter

		// PORT: the half-edge pointers edge1, edge2, edge3 are indices into edges.
		for i := range faceCount {
			face := &faces[i]
			edge1 := int(face.Edge)
			edge2 := int(edges[edge1].Next)
			edge3 := int(edges[edge2].Next)

			assert(edge1 != edge3, "edge1 != edge3")
			assert(int32(edges[edge1].Origin) < shape.hull.VertexCount, "edge1->origin < shape->hull->vertexCount")
			assert(int32(edges[edge2].Origin) < shape.hull.VertexCount, "edge2->origin < shape->hull->vertexCount")

			localPoint1 := points[edges[edge1].Origin]
			localPoint2 := points[edges[edge2].Origin]
			v1 := MulMV(matrix, localPoint1)
			v2 := MulMV(matrix, localPoint2)
			normal := MulMV(matrix, planes[i].Normal)

			for {
				assert(int32(edges[edge3].Origin) < shape.hull.VertexCount, "edge3->origin < shape->hull->vertexCount")
				localPoint3 := points[edges[edge3].Origin]
				v3 := MulMV(matrix, localPoint3)

				// Triangle center
				localCenter := MulSV(0.333333, Add(localPoint1, Add(localPoint2, localPoint3)))

				// Lever arm from center of mass to triangle center in world space
				lever := MulMV(matrix, Sub(localCenter, localCenterOfMass))

				// Velocity of the triangle center in world space
				centerVelocity := Add(linearVelocity, Cross(angularVelocity, lever))

				relativeVelocity := MulSub(wind, drag, centerVelocity)
				direction, speed := GetLengthAndNormalize(relativeVelocity)

				// Check for back-side
				if Dot(normal, direction) < -fltEpsilon {
					projectedArea := -0.5 * Dot(Cross(Sub(v2, v1), Sub(v3, v1)), direction)
					validate(projectedArea >= -fltEpsilon, "projectedArea >= -FLT_EPSILON")

					liftDirection := Cross(Cross(normal, direction), direction)

					speed = MinFloat(speed, maxSpeed)

					forceMagnitude := 0.5 * airDensity * projectedArea * speed * speed
					deltaForce := MulSV(forceMagnitude, MulAdd(direction, lift, liftDirection))
					deltaTorque := Cross(lever, deltaForce)

					force = Add(force, deltaForce)
					torque = Add(torque, deltaTorque)

					if debugWind {
						lineIndex := world.taskContexts[0].lineCount
						if lineIndex < debugLineCapacity {
							line := &world.taskContexts[0].lines[lineIndex]
							line.p1 = OffsetPos(sim.transform.P, MulMV(matrix, localCenter))
							line.p2 = OffsetPos(line.p1, deltaForce)
							line.label = int32(i)
							line.color = ColorBlanchedAlmond
							world.taskContexts[0].lineCount += 1
						}
					}
				}

				edge2 = edge3
				edge3 = int(edges[edge3].Next)
				v2 = v3
				localPoint2 = localPoint3

				if edge1 == edge3 {
					break
				}
			}
		}

	default:
	}

	sim.force = Add(sim.force, force)
	sim.torque = Add(sim.torque, torque)
}

// meshImpactContext is b3MeshImpactContext.
type meshImpactContext struct {
	toiInput  TOIInput
	toiOutput TOIOutput
	// Centroid of shape in body B local space
	localCentroidB Vec3
	// Centroid of shape at beginning and end of sweep in mesh local space. Used for early out.
	meshLocalCentroidB1, meshLocalCentroidB2 Vec3
	fallbackRadius                           float32
	isSensor                                 bool

	// PORT: storage for the b3Vec3 triangle[3] local of b3MeshTimeOfImpactFcn, which proxyA
	// points at during the TOI call. Kept here so the callback does not allocate.
	triangle [3]Vec3
}

// meshTimeOfImpactFcn is b3MeshTimeOfImpactFcn.
func meshTimeOfImpactFcn(a, b, c Vec3, triangleIndex int, context any) bool {
	toiContext := context.(*meshImpactContext)

	// Early out for parallel movement
	c1 := toiContext.meshLocalCentroidB1
	c2 := toiContext.meshLocalCentroidB2

	n := Normalize(Cross(Sub(b, a), Sub(c, a)))
	offset1 := Dot(n, Sub(c1, a))
	offset2 := Dot(n, Sub(c2, a))

	if offset1 < 0.0 {
		// Started behind or finished in front
		return true
	}

	if toiContext.isSensor == false && offset1-offset2 < toiContext.fallbackRadius && offset2 > toiContext.fallbackRadius {
		// Finished in front
		return true
	}

	toiContext.triangle = [3]Vec3{a, b, c}
	toiContext.toiInput.ProxyA.Points = toiContext.triangle[:]
	toiContext.toiInput.ProxyA.Count = 3

	output := TimeOfImpact(&toiContext.toiInput)

	// It is possible for a hit at fraction == 0

	if 0.0 < output.Fraction && output.Fraction < toiContext.toiInput.MaxFraction {
		toiContext.toiOutput = output
		toiContext.toiInput.MaxFraction = output.Fraction
	} else if 0.0 == output.Fraction {
		// fallback to TOI of a small circle around the fast shape centroid
		fallbackInput := toiContext.toiInput
		// PORT: ( b3ShapeProxy ){ &toiContext->localCentroidB, 1, … } aliases the context field.
		fallbackInput.ProxyB = ShapeProxy{unsafe.Slice(&toiContext.localCentroidB, 1), 1, toiContext.fallbackRadius + LinearSlop()}
		output = TimeOfImpact(&fallbackInput)

		if 0.0 < output.Fraction && output.Fraction < toiContext.toiInput.MaxFraction {
			toiContext.toiOutput = output
			toiContext.toiInput.MaxFraction = output.Fraction
			toiContext.toiOutput.UsedFallback = true
		}
	}

	// Continue the query
	return true
}

// meshImpactScratch and compoundImpactScratch hold the impact contexts of b3ShapeTimeOfImpact
// and b3CompoundTimeOfImpactFcn (C stack locals) so that continuous collision does not allocate.
var (
	meshImpactScratch     scratchPool[meshImpactContext]
	compoundImpactScratch scratchPool[compoundImpactContext]
)

// compoundImpactContext is b3CompoundImpactContext.
type compoundImpactContext struct {
	toiInput          TOIInput
	toiOutput         TOIOutput
	compoundTransform Transform

	// Bounds local to compound
	localSweepBoundsB AABB

	// Centroid of shape in body B local space
	localCentroidB Vec3
	fallbackRadius float32

	// PORT: storage for the b3CompoundChild local of b3CompoundTimeOfImpactFcn.
	child ChildShape
}

// compoundTimeOfImpactFcn is b3CompoundTimeOfImpactFcn.
// Implements b3CompoundQueryFcn
func compoundTimeOfImpactFcn(compound *CompoundData, childIndex int, context any) bool {
	toiContext := context.(*compoundImpactContext)

	// PORT: the proxy points alias the child, so a local copy would escape to the heap; the copy
	// lives in the (pooled) context instead.
	toiContext.child = GetCompoundChild(compound, childIndex)
	child := &toiContext.child

	output := TOIOutput{}
	toiContext.toiInput.SweepA = makeCompoundChildSweep(toiContext.compoundTransform, child.Transform)

	switch child.Type {
	case CapsuleShape:
		toiContext.toiInput.ProxyA.Points = capsulePoints(&child.Capsule)
		toiContext.toiInput.ProxyA.Count = 2
		toiContext.toiInput.ProxyA.Radius = child.Capsule.Radius
		output = TimeOfImpact(&toiContext.toiInput)

	case HullShape:
		toiContext.toiInput.ProxyA.Points = GetHullPoints(child.Hull)
		toiContext.toiInput.ProxyA.Count = child.Hull.VertexCount
		toiContext.toiInput.ProxyA.Radius = 0.0
		output = TimeOfImpact(&toiContext.toiInput)

	case MeshShape:
		// PORT: the context is handed to the mesh query as `any`, so a local would escape to the
		// heap; it comes from a scratch pool instead (no allocation once warm).
		meshContext := meshImpactScratch.get()
		*meshContext = meshImpactContext{}
		meshContext.toiInput = toiContext.toiInput
		meshContext.isSensor = false
		meshContext.localCentroidB = toiContext.localCentroidB
		meshContext.fallbackRadius = toiContext.fallbackRadius

		meshWorldTransform := MulTransforms(toiContext.compoundTransform, child.Transform)

		sweepB := &toiContext.toiInput.SweepB
		xfB1 := Transform{
			P: Sub(sweepB.C1, RotateVector(sweepB.Q1, sweepB.LocalCenter)),
			Q: sweepB.Q1,
		}

		xfB2 := Transform{
			P: Sub(sweepB.C2, RotateVector(sweepB.Q2, sweepB.LocalCenter)),
			Q: sweepB.Q2,
		}

		meshContext.meshLocalCentroidB1 =
			InvTransformPoint(meshWorldTransform, TransformPoint(xfB1, meshContext.localCentroidB))
		meshContext.meshLocalCentroidB2 =
			InvTransformPoint(meshWorldTransform, TransformPoint(xfB2, meshContext.localCentroidB))

		// Bounds local to mesh
		localBounds := toiContext.localSweepBoundsB.Transform(InvertTransform(child.Transform))

		QueryMesh(&child.Mesh, localBounds, meshTimeOfImpactFcn, meshContext)

		output = meshContext.toiOutput
		meshImpactScratch.put(meshContext)

	case SphereShape:
		toiContext.toiInput.ProxyA.Points = spherePoints(&child.Sphere)
		toiContext.toiInput.ProxyA.Count = 1
		toiContext.toiInput.ProxyA.Radius = child.Sphere.Radius
		output = TimeOfImpact(&toiContext.toiInput)

	default:
		assert(false, "false")
	}

	if 0.0 < output.Fraction && output.Fraction < toiContext.toiInput.MaxFraction {
		toiContext.toiOutput = output
		toiContext.toiInput.MaxFraction = output.Fraction
	}

	// Clear this to be safe
	toiContext.toiInput.ProxyA = ShapeProxy{}

	// Continue the query
	return true
}

// shapeTimeOfImpact is b3ShapeTimeOfImpact.
func shapeTimeOfImpact(shapeA, shapeB *shape, sweepA, sweepB *Sweep, maxFraction float32) TOIOutput {
	isSensor := shapeA.sensorIndex != NullIndex

	typeA := shapeA.typ
	if typeA == CompoundShape {
		// todo implement b3CompoundTimeOfImpact
		// PORT: the context is handed to the compound query as `any`, so a local would escape to
		// the heap; it comes from a scratch pool instead (no allocation once warm).
		context := compoundImpactScratch.get()
		defer compoundImpactScratch.put(context)
		*context = compoundImpactContext{}
		context.toiInput.ProxyB = makeShapeProxy(shapeB)
		context.toiInput.SweepB = *sweepB
		context.toiInput.MaxFraction = maxFraction

		context.compoundTransform = Transform{
			P: sweepA.C1,
			Q: sweepA.Q1,
		}

		localCentroidB := getShapeCentroid(shapeB)
		context.localCentroidB = localCentroidB

		minExtent := computeShapeMinExtent(shapeB, context.localCentroidB)
		context.fallbackRadius = MaxFloat(0.75*minExtent, SpeculativeDistance())

		// Swept bounds of shapeB
		bounds := computeSweptShapeAABB(shapeB, sweepB, maxFraction)

		// Bounds local to mesh
		localBounds := bounds.Transform(InvertTransform(context.compoundTransform))
		context.localSweepBoundsB = localBounds

		QueryCompound(shapeA.compound, localBounds, compoundTimeOfImpactFcn, context)

		return context.toiOutput
	}

	if typeA == HeightShape || typeA == MeshShape {
		// todo implement b3MeshTimeOfImpact and b3HeightFieldTimeOfImpact
		// Note: assuming mesh is static

		// PORT: scratch pool, see compoundImpactContext above.
		context := meshImpactScratch.get()
		defer meshImpactScratch.put(context)
		*context = meshImpactContext{}
		context.toiInput.SweepA = *sweepA
		context.toiInput.ProxyA.Count = 3
		context.toiInput.ProxyB = makeShapeProxy(shapeB)
		context.toiInput.SweepB = *sweepB
		context.toiInput.MaxFraction = maxFraction
		context.isSensor = isSensor

		localCentroidB := getShapeCentroid(shapeB)
		context.localCentroidB = localCentroidB

		// Assume mesh is static
		xfA := Transform{
			P: Sub(sweepA.C1, RotateVector(sweepA.Q1, sweepA.LocalCenter)),
			Q: sweepA.Q1,
		}

		xfB1 := Transform{
			P: Sub(sweepB.C1, RotateVector(sweepB.Q1, sweepB.LocalCenter)),
			Q: sweepB.Q1,
		}

		xfB2 := Transform{
			P: Sub(sweepB.C2, RotateVector(sweepB.Q2, sweepB.LocalCenter)),
			Q: sweepB.Q2,
		}

		context.meshLocalCentroidB1 = InvTransformPoint(xfA, TransformPoint(xfB1, localCentroidB))
		context.meshLocalCentroidB2 = InvTransformPoint(xfA, TransformPoint(xfB2, localCentroidB))

		minExtent := computeShapeMinExtent(shapeB, context.localCentroidB)
		context.fallbackRadius = MaxFloat(0.5*minExtent, LinearSlop())

		// Swept bounds of shapeB
		// todo pass in xfA to get local bounds directly
		bounds := computeSweptShapeAABB(shapeB, sweepB, maxFraction)

		// Bounds local to mesh
		localBounds := bounds.Transform(InvertTransform(xfA))

		if typeA == MeshShape {
			QueryMesh(&shapeA.mesh, localBounds, meshTimeOfImpactFcn, context)
		} else if typeA == HeightShape {
			QueryHeightField(shapeA.heightField, localBounds, meshTimeOfImpactFcn, context)
		}

		return context.toiOutput
	}

	assert(shapeB.typ != CompoundShape && shapeB.typ != MeshShape && shapeB.typ != HeightShape,
		"shapeB->type != b3_compoundShape && shapeB->type != b3_meshShape && shapeB->type != b3_heightShape")

	var input TOIInput
	input.ProxyA = makeShapeProxy(shapeA)
	input.ProxyB = makeShapeProxy(shapeB)
	input.SweepA = *sweepA
	input.SweepB = *sweepB
	input.MaxFraction = maxFraction

	output := TimeOfImpact(&input)

	// PORT: upstream's #if 0 fallback block (convex vs convex TOI around the centroid) is not ported.

	return output
}

// getShapeUserMaterialId is b3GetShapeUserMaterialId.
// Resolve the user material id for a hit point on the given shape. Mesh/heightfield shapes
// use the manifold-point triangleIndex to pick a per-triangle material. Compound shapes use
// the contact's childIndex to find the participating child, then for a mesh child apply the
// child's materialIndices indirection on top of the per-triangle index. Convex shapes fall
// back to materials[0]. childIndex is unused for non-compound shapes.
func getShapeUserMaterialId(shape *shape, childIndex, triangleIndex int) uint64 {
	if shape.materialCount == 0 {
		return 0
	}
	return getShapeMaterials(shape)[getShapeMaterialIndex(shape, childIndex, triangleIndex)].UserMaterialId
}

func getShapeMaterialIndex(shape *shape, childIndex, triangleIndex int) int {
	materialIndex := 0
	if shape.typ == MeshShape {
		indices := GetMeshMaterialIndices(shape.mesh.Data)
		if indices != nil {
			materialIndex = int(indices[triangleIndex])
		}
	} else if shape.typ == HeightShape {
		materialIndex = getHeightFieldMaterial(shape.heightField, triangleIndex)
	} else if shape.typ == CompoundShape {
		child := GetCompoundChild(shape.compound, childIndex)
		if child.Type == MeshShape {
			indices := GetMeshMaterialIndices(child.Mesh.Data)
			meshMaterialIndex := 0
			if indices != nil {
				meshMaterialIndex = int(indices[triangleIndex])
			}
			assert(0 <= meshMaterialIndex && meshMaterialIndex < int(child.MaterialCount),
				"0 <= meshMaterialIndex && meshMaterialIndex < child.materialCount")
			materialIndex = int(child.MaterialIndices[meshMaterialIndex])
		} else {
			materialIndex = int(child.MaterialIndices[0])
		}
	}

	materialIndex = ClampInt(materialIndex, 0, int(shape.materialCount)-1)
	return materialIndex
}
