// Port of src/physics_world.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: physics_world.c is split in two Go files (spec: left to the builder). This file holds the
// world queries, upstream lines WorldQueryContext … b3World_CastMover, in upstream order. Everything
// before (world lifetime, step, debug draw, events, settings, counters) and after
// (b3World_SetCustomFilterCallback … validation) is in physics_world.go.

// worldQueryContext is WorldQueryContext.
type worldQueryContext struct {
	world       *world
	fcn         OverlapResultFcn
	filter      QueryFilter
	userContext any
}

// treeQueryCallback is TreeQueryCallback.
func treeQueryCallback(proxyId int, userData uint64, context any) bool {
	shapeId := int(userData)
	worldContext := context.(*worldQueryContext)
	world := worldContext.world

	shape := arrayGet(world.shapes, shapeId)

	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if shouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return true
	}

	id := ShapeID{int32(shapeId + 1), world.worldId, shape.generation}
	result := worldContext.fcn(id, worldContext.userContext)
	return result
}

// OverlapAABB is b3World_OverlapAABB: overlap test for all shapes that *potentially* overlap the
// provided AABB.
func (worldId WorldID) OverlapAABB(aabb AABB, filter QueryFilter, fcn OverlapResultFcn, context any) TreeStats {
	treeStats := TreeStats{}

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return treeStats
	}

	assert(IsValidAABB(aabb), "b3IsValidAABB( aabb )")

	recWriter := recQueryWriter{}
	if world.recording != nil {
		recQueryBegin(&recWriter, context, filter.Id, filter.Name)
		recWriter.overlapFcn = fcn
		recWWORLDID(&recWriter.buf, worldId)
		recWAABB(&recWriter.buf, aabb)
		recWQUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = int32(recReserveU32(&recWriter.buf))
		fcn = recOverlapTrampoline
		context = &recWriter
	}

	worldContext := worldQueryContext{world, fcn, filter, context}

	for i := 0; i < int(BodyTypeCount); i++ {
		treeResult := world.broadPhase.trees[i].Query(aabb, filter.MaskBits, false, treeQueryCallback, &worldContext)

		treeStats.NodeVisits += treeResult.NodeVisits
		treeStats.LeafVisits += treeResult.LeafVisits
	}

	if world.recording != nil {
		recPatchU32(&recWriter.buf, int(recWriter.countOffset), recWriter.hitCount)
		recWTREESTATS(&recWriter.buf, treeStats)
		recQueryCommit(world.recording, recOpQueryOverlapAABB, &recWriter)
	}

	return treeStats
}

// worldOverlapContext is WorldOverlapContext.
type worldOverlapContext struct {
	world       *world
	fcn         OverlapResultFcn
	filter      QueryFilter
	proxy       ShapeProxy
	origin      Pos
	userContext any
}

// treeOverlapCallback is b3TreeOverlapCallback.
func treeOverlapCallback(proxyId int, userData uint64, context any) bool {
	shapeId := int(userData)
	worldContext := context.(*worldOverlapContext)
	world := worldContext.world

	shape := arrayGet(world.shapes, shapeId)

	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if shouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return true
	}

	// Re-center on the query origin so the overlap test stays in float precision far from the origin
	body := arrayGet(world.bodies, int(shape.bodyId))
	transform := ToRelativeTransform(getBodyTransformQuick(world, body), worldContext.origin)

	overlapping := overlapShape(shape, transform, &worldContext.proxy)
	if overlapping == false {
		return true
	}

	id := ShapeID{shape.id + 1, world.worldId, shape.generation}
	result := worldContext.fcn(id, worldContext.userContext)
	return result
}

// OverlapShape is b3World_OverlapShape: overlap test for all shapes that overlap the provided
// shape proxy. The proxy points are relative to origin.
func (worldId WorldID) OverlapShape(origin Pos, proxy *ShapeProxy, filter QueryFilter, fcn OverlapResultFcn,
	context any) TreeStats {
	treeStats := TreeStats{}

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return treeStats
	}

	assert(IsValidPosition(origin), "b3IsValidPosition( origin )")

	recWriter := recQueryWriter{}
	if world.recording != nil {
		recQueryBegin(&recWriter, context, filter.Id, filter.Name)
		recWriter.overlapFcn = fcn
		recWWORLDID(&recWriter.buf, worldId)
		recWPOSITION(&recWriter.buf, origin)
		recWSHAPEPROXY(&recWriter.buf, *proxy)
		recWQUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = int32(recReserveU32(&recWriter.buf))
		fcn = recOverlapTrampoline
		context = &recWriter
	}

	// Bound the proxy in origin relative space then lift to a conservative world float box
	aabb := OffsetAABB(MakeAABB(proxy.Points[:proxy.Count], proxy.Radius), origin)
	worldContext := worldOverlapContext{
		world, fcn, filter, *proxy, origin, context,
	}

	for i := 0; i < int(BodyTypeCount); i++ {
		treeResult := world.broadPhase.trees[i].Query(aabb, filter.MaskBits, false, treeOverlapCallback, &worldContext)

		treeStats.NodeVisits += treeResult.NodeVisits
		treeStats.LeafVisits += treeResult.LeafVisits
	}

	if world.recording != nil {
		recPatchU32(&recWriter.buf, int(recWriter.countOffset), recWriter.hitCount)
		recWTREESTATS(&recWriter.buf, treeStats)
		recQueryCommit(world.recording, recOpQueryOverlapShape, &recWriter)
	}

	return treeStats
}

// worldMoverContext is WorldMoverContext.
type worldMoverContext struct {
	world       *world
	fcn         PlaneResultFcn
	filter      QueryFilter
	mover       Capsule
	origin      Pos
	userContext any
}

// treeCollideCallback is TreeCollideCallback.
func treeCollideCallback(proxyId int, userData uint64, context any) bool {
	shapeId := int(userData)
	worldContext := context.(*worldMoverContext)
	world := worldContext.world

	shape := arrayGet(world.shapes, shapeId)

	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if shouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return true
	}

	// Re-center on the query origin, the mover and the resulting planes are origin relative
	body := arrayGet(world.bodies, int(shape.bodyId))
	bodyTransform := getBodyTransformQuick(world, body)
	transform := ToRelativeTransform(bodyTransform, worldContext.origin)

	var buffer [64]PlaneResult
	count := collideMover(buffer[:], shape, transform, &worldContext.mover)

	if count > 0 {
		id := ShapeID{shape.id + 1, world.worldId, shape.generation}
		return worldContext.fcn(id, buffer[:count], worldContext.userContext)
	}

	return true
}

// CollideMover is b3World_CollideMover: collide a capsule mover with the world, gathering
// collision planes that can be fed to SolvePlanes. Useful for kinematic character movement. The
// mover and the resulting planes are relative to origin.
// It is tempting to use a shape proxy for the mover, but this makes handling deep overlap difficult and the generality may
// not be worth it.
func (worldId WorldID) CollideMover(origin Pos, mover *Capsule, filter QueryFilter, fcn PlaneResultFcn, context any) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	assert(IsValidPosition(origin), "b3IsValidPosition( origin )")

	recWriter := recQueryWriter{}
	if world.recording != nil {
		recQueryBegin(&recWriter, context, filter.Id, filter.Name)
		recWriter.planeFcn = fcn
		recWWORLDID(&recWriter.buf, worldId)
		recWPOSITION(&recWriter.buf, origin)
		recWCAPSULE(&recWriter.buf, *mover)
		recWQUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = int32(recReserveU32(&recWriter.buf))
		fcn = recPlaneTrampoline
		context = &recWriter
	}

	r := Vec3{mover.Radius, mover.Radius, mover.Radius}

	var relBox AABB
	relBox.LowerBound = Sub(Min(mover.Center1, mover.Center2), r)
	relBox.UpperBound = Add(Max(mover.Center1, mover.Center2), r)
	aabb := OffsetAABB(relBox, origin)

	worldContext := worldMoverContext{
		world, fcn, filter, *mover, origin, context,
	}

	for i := 0; i < int(BodyTypeCount); i++ {
		world.broadPhase.trees[i].Query(aabb, filter.MaskBits, false, treeCollideCallback, &worldContext)
	}

	if world.recording != nil {
		recPatchU32(&recWriter.buf, int(recWriter.countOffset), recWriter.hitCount)
		recQueryCommit(world.recording, recOpQueryCollideMover, &recWriter)
	}
}

// worldRayCastContext is WorldRayCastContext.
type worldRayCastContext struct {
	world       *world
	fcn         CastResultFcn
	filter      QueryFilter
	fraction    float32
	origin      Pos
	userContext any
}

// rayCastCallback is RayCastCallback.
func rayCastCallback(input *RayCastInput, proxyId int, userData uint64, context any) float32 {
	shapeId := int(userData)
	worldContext := context.(*worldRayCastContext)
	world := worldContext.world

	shape := arrayGet(world.shapes, shapeId)
	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if shouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return input.MaxFraction
	}

	body := arrayGet(world.bodies, int(shape.bodyId))
	bodyTransform := getBodyTransformQuick(world, body)
	transform := ToRelativeTransform(bodyTransform, worldContext.origin)

	localInput := *input
	localInput.Origin = Vec3Zero
	output := rayCastShapeWithBackFaces(shape, transform, &localInput, queryFilter.BackFaces)

	if output.Hit {
		assert(output.Fraction <= input.MaxFraction, "output.fraction <= input->maxFraction")

		id := ShapeID{int32(shapeId + 1), world.worldId, shape.generation}
		point := OffsetPos(worldContext.origin, output.Point)
		materialIndex := ClampInt(int(output.MaterialIndex), 0, int(shape.materialCount)-1)
		userMaterialId := getShapeMaterials(shape)[materialIndex].UserMaterialId

		triangleIndex := int(output.TriangleIndex)
		childIndex := int(output.ChildIndex)
		fraction := worldContext.fcn(id, point, output.Normal, output.Fraction, userMaterialId, triangleIndex, childIndex,
			worldContext.userContext)

		// The user may return -1 to skip this shape
		if 0.0 <= fraction && fraction <= 1.0 {
			worldContext.fraction = fraction
		}

		return fraction
	}

	return input.MaxFraction
}

// CastRay is b3World_CastRay: cast a ray into the world to collect shapes in the path of the ray.
// Your callback function controls whether you get the closest point, any point, or n-points.
// Note: The callback function may receive shapes in any order
// origin: the start point of the ray
// translation: the translation of the ray from the start point to the end point
// filter: contains bit flags to filter unwanted shapes from the results
// fcn: a user implemented callback function
// context: a user context that is passed along to the callback function
// Returns traversal performance counters.
func (worldId WorldID) CastRay(origin Pos, translation Vec3, filter QueryFilter, fcn CastResultFcn, context any) TreeStats {
	treeStats := TreeStats{}

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return treeStats
	}

	assert(IsValidPosition(origin), "b3IsValidPosition( origin )")
	assert(IsValidVec3(translation), "b3IsValidVec3( translation )")

	recWriter := recQueryWriter{}
	if world.recording != nil {
		recQueryBegin(&recWriter, context, filter.Id, filter.Name)
		recWriter.castFcn = fcn
		recWWORLDID(&recWriter.buf, worldId)
		recWPOSITION(&recWriter.buf, origin)
		recWVEC3(&recWriter.buf, translation)
		recWQUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = int32(recReserveU32(&recWriter.buf))
		fcn = recCastTrampoline
		context = &recWriter
	}

	// The tree traverses in float relative to the world origin. Each shape is then re-differenced at
	// full precision against the origin, so a hit stays accurate far from the origin.
	input := RayCastInput{ToVec3(origin), translation, 1.0}

	worldContext := worldRayCastContext{world, fcn, filter, 1.0, origin, context}

	for i := 0; i < int(BodyTypeCount); i++ {
		treeResult := world.broadPhase.trees[i].RayCast(&input, filter.MaskBits, false, rayCastCallback, &worldContext)
		treeStats.NodeVisits += treeResult.NodeVisits
		treeStats.LeafVisits += treeResult.LeafVisits

		if worldContext.fraction == 0.0 {
			break
		}

		input.MaxFraction = worldContext.fraction
	}

	if world.recording != nil {
		recPatchU32(&recWriter.buf, int(recWriter.countOffset), recWriter.hitCount)
		recWTREESTATS(&recWriter.buf, treeStats)
		recQueryCommit(world.recording, recOpQueryCastRay, &recWriter)
	}

	return treeStats
}

// rayCastClosestFcn is b3RayCastClosestFcn.
// This callback finds the closest hit. This is the most common callback used in games.
func rayCastClosestFcn(shapeId ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64,
	triangleIndex int, childIndex int, context any) float32 {
	// Ignore initial overlap
	if fraction == 0.0 {
		return -1.0
	}

	rayResult := context.(*RayResult)
	rayResult.ShapeId = shapeId
	rayResult.Point = point
	rayResult.Normal = normal
	rayResult.Fraction = fraction
	rayResult.UserMaterialId = userMaterialId
	rayResult.TriangleIndex = int32(triangleIndex)
	rayResult.ChildIndex = int32(childIndex)
	rayResult.Hit = true
	return fraction
}

// CastRayClosest is b3World_CastRayClosest: cast a ray into the world to collect the closest hit.
// This is a convenience function. Ignores initial overlap. This is less general than CastRay()
// and does not allow for custom filtering.
func (worldId WorldID) CastRayClosest(origin Pos, translation Vec3, filter QueryFilter) RayResult {
	result := RayResult{}

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return result
	}

	assert(IsValidPosition(origin), "b3IsValidPosition( origin )")
	assert(IsValidVec3(translation), "b3IsValidVec3( translation )")

	// The tree traverses in float relative to the world origin. Each shape is then re-differenced at
	// full precision against its body, so a hit stays accurate far from the origin.
	input := RayCastInput{ToVec3(origin), translation, 1.0}
	worldContext := worldRayCastContext{
		world:       world,
		fcn:         rayCastClosestFcn,
		filter:      filter,
		fraction:    1.0,
		origin:      origin,
		userContext: &result,
	}

	for i := 0; i < int(BodyTypeCount); i++ {
		treeResult := world.broadPhase.trees[i].RayCast(&input, filter.MaskBits, false, rayCastCallback, &worldContext)
		result.NodeVisits += treeResult.NodeVisits
		result.LeafVisits += treeResult.LeafVisits

		if worldContext.fraction == 0.0 {
			break
		}

		input.MaxFraction = worldContext.fraction
	}

	// Closed query, no user callback: record the inputs and the single result for the replay compare.
	if world.recording != nil {
		recWriter := recQueryWriter{}
		recQueryBegin(&recWriter, nil, filter.Id, filter.Name)
		recWWORLDID(&recWriter.buf, worldId)
		recWPOSITION(&recWriter.buf, origin)
		recWVEC3(&recWriter.buf, translation)
		recWQUERYFILTER(&recWriter.buf, filter)
		recWRAYRESULT(&recWriter.buf, result)
		recQueryCommit(world.recording, recOpQueryCastRayClosest, &recWriter)
	}

	return result
}

// worldShapeCastContext is WorldShapeCastContext.
type worldShapeCastContext struct {
	world    *world
	fcn      CastResultFcn
	filter   QueryFilter
	fraction float32
	origin   Pos
	// origin relative input
	input       ShapeCastInput
	userContext any
}

// shapeCastCallback is b3ShapeCastCallback.
func shapeCastCallback(input *BoxCastInput, proxyId int, userData uint64, context any) float32 {
	shapeId := int(userData)
	worldContext := context.(*worldShapeCastContext)
	world := worldContext.world

	shape := arrayGet(world.shapes, shapeId)
	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if shouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return input.MaxFraction
	}

	// Rebuild from the origin relative input, taking only the advancing fraction from the tree.
	// The tree box is world float and would lose the cast far from the origin.
	localInput := worldContext.input
	localInput.MaxFraction = input.MaxFraction

	// Re-center on the query origin so the per-shape cast stays in float precision far from the origin
	body := arrayGet(world.bodies, int(shape.bodyId))
	transform := ToRelativeTransform(getBodyTransformQuick(world, body), worldContext.origin)

	output := shapeCastShapeWithBackFaces(shape, transform, &localInput, queryFilter.BackFaces)

	if output.Hit {
		id := ShapeID{int32(shapeId + 1), world.worldId, shape.generation}
		materialIndex := ClampInt(int(output.MaterialIndex), 0, int(shape.materialCount)-1)
		userMaterialId := getShapeMaterials(shape)[materialIndex].UserMaterialId

		triangleIndex := int(output.TriangleIndex)
		childIndex := int(output.ChildIndex)
		fraction := worldContext.fcn(id, OffsetPos(worldContext.origin, output.Point), output.Normal, output.Fraction,
			userMaterialId, triangleIndex, childIndex, worldContext.userContext)

		// The user may return -1 to skip this shape
		if 0.0 <= fraction && fraction <= 1.0 {
			worldContext.fraction = fraction
		}

		return fraction
	}

	return input.MaxFraction
}

// CastShape is b3World_CastShape: cast a shape through the world. Similar to a cast ray except
// that a shape is cast instead of a point. The proxy points are relative to origin.
// See CastRay
func (worldId WorldID) CastShape(origin Pos, proxy *ShapeProxy, translation Vec3, filter QueryFilter,
	fcn CastResultFcn, context any) TreeStats {
	treeStats := TreeStats{}

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return treeStats
	}

	assert(IsValidPosition(origin), "b3IsValidPosition( origin )")
	assert(IsValidVec3(translation), "b3IsValidVec3( translation )")

	recWriter := recQueryWriter{}
	if world.recording != nil {
		recQueryBegin(&recWriter, context, filter.Id, filter.Name)
		recWriter.castFcn = fcn
		recWWORLDID(&recWriter.buf, worldId)
		recWPOSITION(&recWriter.buf, origin)
		recWSHAPEPROXY(&recWriter.buf, *proxy)
		recWVEC3(&recWriter.buf, translation)
		recWQUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = int32(recReserveU32(&recWriter.buf))
		fcn = recCastTrampoline
		context = &recWriter
	}

	worldContext := worldShapeCastContext{}
	worldContext.world = world
	worldContext.fcn = fcn
	worldContext.filter = filter
	worldContext.fraction = 1.0
	worldContext.origin = origin
	worldContext.input.Proxy = *proxy
	worldContext.input.Translation = translation
	worldContext.input.MaxFraction = 1.0
	worldContext.input.CanEncroach = false
	worldContext.userContext = context

	// Bound the proxy in origin relative space then lift to a conservative world float box. The tree
	// node boxes use the same directed rounding, so the swept box never clips a shape far from the
	// origin. Per shape casts re-difference at full precision against the carried origin.
	localBox := MakeAABB(proxy.Points[:proxy.Count], proxy.Radius)
	treeInput := BoxCastInput{OffsetAABB(localBox, origin), translation, 1.0}

	for i := 0; i < int(BodyTypeCount); i++ {
		treeResult := world.broadPhase.trees[i].BoxCast(&treeInput, filter.MaskBits, false, shapeCastCallback, &worldContext)
		treeStats.NodeVisits += treeResult.NodeVisits
		treeStats.LeafVisits += treeResult.LeafVisits

		if worldContext.fraction == 0.0 {
			break
		}

		treeInput.MaxFraction = worldContext.fraction
	}

	if world.recording != nil {
		recPatchU32(&recWriter.buf, int(recWriter.countOffset), recWriter.hitCount)
		recWTREESTATS(&recWriter.buf, treeStats)
		recQueryCommit(world.recording, recOpQueryCastShape, &recWriter)
	}

	return treeStats
}

// worldMoverCastContext is WorldMoverCastContext.
type worldMoverCastContext struct {
	world    *world
	fcn      MoverFilterFcn
	filter   QueryFilter
	fraction float32
	origin   Pos
	// origin relative input
	input       ShapeCastInput
	userContext any
}

// moverCastCallback is MoverCastCallback.
func moverCastCallback(input *BoxCastInput, proxyId int, userData uint64, context any) float32 {
	shapeId := int(userData)
	worldContext := context.(*worldMoverCastContext)
	world := worldContext.world

	shape := arrayGet(world.shapes, shapeId)
	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if shouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return worldContext.fraction
	}

	if worldContext.fcn != nil {
		id := ShapeID{int32(shapeId + 1), world.worldId, shape.generation}
		shouldCollide := worldContext.fcn(id, worldContext.userContext)
		if shouldCollide == false {
			return worldContext.fraction
		}
	}

	// Rebuild from the origin relative input, taking only the advancing fraction from the tree
	localInput := worldContext.input
	localInput.MaxFraction = input.MaxFraction

	// Re-center on the query origin so the per-shape cast stays in float precision far from the origin
	body := arrayGet(world.bodies, int(shape.bodyId))
	transform := ToRelativeTransform(getBodyTransformQuick(world, body), worldContext.origin)

	output := shapeCastShapeWithBackFaces(shape, transform, &localInput, queryFilter.BackFaces)
	if output.Fraction == 0.0 {
		// Ignore overlapping shapes
		return worldContext.fraction
	}

	worldContext.fraction = output.Fraction
	return output.Fraction
}

// CastMover is b3World_CastMover: cast a capsule mover through the world. This is a special shape
// cast that handles sliding along other shapes while reducing clipping. The mover is relative to
// origin. Returns the fraction of the translation that can be traveled.
func (worldId WorldID) CastMover(origin Pos, mover *Capsule, translation Vec3, filter QueryFilter,
	fcn MoverFilterFcn, context any) float32 {
	assert(IsValidPosition(origin), "b3IsValidPosition( origin )")
	assert(IsValidVec3(translation), "b3IsValidVec3( translation )")

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return 1.0
	}

	// The mover filter is a per-shape bool(shapeId, ctx) decision, the same shape as an overlap
	// callback, so the overlap trampoline captures it. Installing it even when the user passed no
	// filter records an accept-all stream that replays identically.
	recWriter := recQueryWriter{}
	if world.recording != nil {
		recQueryBegin(&recWriter, context, filter.Id, filter.Name)
		recWriter.moverFilterFcn = fcn
		recWWORLDID(&recWriter.buf, worldId)
		recWPOSITION(&recWriter.buf, origin)
		recWCAPSULE(&recWriter.buf, *mover)
		recWVEC3(&recWriter.buf, translation)
		recWQUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = int32(recReserveU32(&recWriter.buf))
		fcn = recOverlapTrampoline
		context = &recWriter
	}

	worldContext := worldMoverCastContext{
		world:       world,
		fcn:         fcn,
		filter:      filter,
		fraction:    1.0,
		origin:      origin,
		userContext: context,
	}
	// PORT: ( b3ShapeProxy ){ &mover->center1, 2, mover->radius } aliases the capsule centers.
	worldContext.input.Proxy = ShapeProxy{capsulePoints(mover), 2, mover.Radius}
	worldContext.input.Translation = translation
	worldContext.input.MaxFraction = 1.0
	worldContext.input.CanEncroach = mover.Radius > 0.0

	// Bound the capsule in origin relative space then lift to a conservative world float box
	centers := [2]Vec3{mover.Center1, mover.Center2}
	treeInput := BoxCastInput{OffsetAABB(MakeAABB(centers[:], mover.Radius), origin), translation, 1.0}

	for i := 0; i < int(BodyTypeCount); i++ {
		world.broadPhase.trees[i].BoxCast(&treeInput, filter.MaskBits, false, moverCastCallback, &worldContext)

		if worldContext.fraction == 0.0 {
			break
		}

		treeInput.MaxFraction = worldContext.fraction
	}

	if world.recording != nil {
		// The mover filter type aliases the overlap trampoline, so the user fcn lands in the same
		// union slot. Backpatch the accept count, then record the returned fraction as the tail.
		recPatchU32(&recWriter.buf, int(recWriter.countOffset), recWriter.hitCount)
		recWF32(&recWriter.buf, worldContext.fraction)
		recQueryCommit(world.recording, recOpQueryCastMover, &recWriter)
	}

	return worldContext.fraction
}
