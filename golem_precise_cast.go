// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// PreciseCastShape performs a geometric translation sweep without the solver's
// rest separation or LinearSlop bias. Tolerance is a positive distance bound;
// it does not alter ordinary CastShape, CastMover or solver CCD semantics.
// False reports invalid inputs or numerical nonconvergence, never a clear path.
// Callback fractions clip subsequent traversal; equal-fraction hits remain
// eligible. Callback order is unspecified, as with the native cast queries.
// This additive query is not supported by upstream recording format.
func (worldId WorldID) PreciseCastShape(origin Pos, proxy *ShapeProxy, translation Vec3, filter QueryFilter, tolerance float32, callback CastResultFcn, context any) (TreeStats, bool) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil || world.recording != nil || callback == nil || !validPreciseProxy(proxy) || !IsValidPosition(origin) || !IsValidVec3(translation) || !IsValidFloat(tolerance) || tolerance <= 0 {
		return TreeStats{}, false
	}
	bounds := preciseProxyBounds(proxy, translation, tolerance)
	padding := float32(4 * fltEpsilon * MaxFloat(1, MaxFloat(AbsFloat(origin.X), MaxFloat(AbsFloat(origin.Y), AbsFloat(origin.Z)))))
	localBox := preciseProxyBounds(proxy, Vec3Zero, tolerance)
	worldBox := OffsetAABB(localBox, origin)
	worldBox.LowerBound = Sub(worldBox.LowerBound, Vec3{padding, padding, padding})
	worldBox.UpperBound = Add(worldBox.UpperBound, Vec3{padding, padding, padding})
	c := preciseCastScratch.get()
	if c.visit == nil {
		c.visit = c.visitLeaf
	}
	c.origin, c.proxy, c.translation = origin, proxy, translation
	c.world = world
	c.filter, c.tolerance, c.bounds = filter, tolerance, bounds
	c.callback, c.context, c.valid, c.maxFraction = callback, context, true, 1
	defer releasePreciseCast(c)
	input := BoxCastInput{Box: worldBox, Translation: translation, MaxFraction: 1}
	var stats TreeStats
	for i := 0; i < int(BodyTypeCount); i++ {
		current := world.broadPhase.trees[i].BoxCast(&input, filter.MaskBits, false, preciseCastTreeCandidate, c)
		stats.NodeVisits += current.NodeVisits
		stats.LeafVisits += current.LeafVisits
		if !c.valid || c.stopped {
			break
		}
		input.MaxFraction = c.maxFraction
	}
	return stats, c.valid
}

type preciseCastContext struct {
	world       *world
	origin      Pos
	proxy       *ShapeProxy
	translation Vec3
	filter      QueryFilter
	tolerance   float32
	bounds      AABB
	callback    CastResultFcn
	context     any
	valid       bool
	stopped     bool
	maxFraction float32
	best        CastOutput
	child       int32
	visit       func(queryLeaf)
}

var preciseCastScratch scratchPool[preciseCastContext]

func preciseCastTreeCandidate(_ *BoxCastInput, _ int, userData uint64, context any) float32 {
	c := context.(*preciseCastContext)
	s := &c.world.shapes[int(userData)]
	if !shouldQueryCollide(&s.filter, &c.filter) {
		return c.maxFraction
	}
	id := ShapeID{s.id + 1, c.world.worldId, s.generation}
	if !preciseCastCandidate(id, c) {
		return 0
	}
	return c.maxFraction
}

func preciseCastCandidate(id ShapeID, context any) bool {
	c := context.(*preciseCastContext)
	s, transform, ok := queryShape(id, c.origin)
	if !ok {
		c.valid = false
		return false
	}
	c.best, c.child = CastOutput{Fraction: 1, TriangleIndex: -1}, -1
	bounds := c.bounds
	if c.maxFraction < 1 {
		// Clip only traversal, never the translation used by the leaf solver.
		// Padding covers prefix multiplication and transformed-bound rounding;
		// it does not change the geometric hit tolerance or reported witnesses.
		magnitude := MaxFloat(1, MaxFloat(Length(c.bounds.LowerBound), MaxFloat(Length(c.bounds.UpperBound), Length(transform.P))))
		bounds = preciseProxyBounds(c.proxy, MulSV(c.maxFraction, c.translation), c.tolerance+16*fltEpsilon*magnitude)
	}
	visitQueryGeometry(s, transform, bounds, c.visit)
	if !c.valid {
		return false
	}
	if c.best.Hit && c.best.Fraction <= c.maxFraction {
		material := getShapeUserMaterialId(s, int(c.child), int(c.best.TriangleIndex))
		fraction := c.callback(id, OffsetPos(c.origin, c.best.Point), c.best.Normal, c.best.Fraction, material, int(c.best.TriangleIndex), int(c.child), c.context)
		c.stopped = fraction == 0
		if fraction > 0 && fraction < c.maxFraction {
			c.maxFraction = fraction
		}
		return fraction != 0
	}
	return true
}

func (c *preciseCastContext) visitLeaf(leaf queryLeaf) {
	if !c.valid {
		return
	}
	out, ok := preciseLeafCast(leaf, c.proxy, c.translation, c.tolerance)
	if !ok {
		c.valid = false
		return
	}
	if !out.Hit {
		return
	}
	if leaf.triangleIndex >= 0 && !c.filter.BackFaces {
		normal := RotateVector(leaf.transform.Q, Normalize(Cross(Sub(leaf.triangle[1], leaf.triangle[0]), Sub(leaf.triangle[2], leaf.triangle[0]))))
		if Dot(normal, out.Normal) < -fltEpsilon {
			return
		}
	}
	if !c.best.Hit || out.Fraction < c.best.Fraction || out.Fraction == c.best.Fraction && (leaf.child < c.child || leaf.child == c.child && leaf.triangleIndex < c.best.TriangleIndex) {
		c.best = out
		c.best.TriangleIndex = leaf.triangleIndex
		c.child = leaf.child
	}
}

func releasePreciseCast(c *preciseCastContext) {
	visit := c.visit
	*c = preciseCastContext{visit: visit}
	preciseCastScratch.put(c)
}

func validPreciseProxy(proxy *ShapeProxy) bool {
	if proxy == nil || proxy.Count < 1 || proxy.Count > MaxShapeCastPoints || int(proxy.Count) > len(proxy.Points) || !IsValidFloat(proxy.Radius) || proxy.Radius < 0 {
		return false
	}
	for _, v := range proxy.Points[:proxy.Count] {
		if !IsValidVec3(v) {
			return false
		}
	}
	return true
}
func preciseProxyBounds(proxy *ShapeProxy, translation Vec3, padding float32) AABB {
	lower, upper := proxy.Points[0], proxy.Points[0]
	for _, v := range proxy.Points[:proxy.Count] {
		lower = Min(lower, Min(v, Add(v, translation)))
		upper = Max(upper, Max(v, Add(v, translation)))
	}
	r := proxy.Radius + padding
	return AABB{Sub(lower, Vec3{r, r, r}), Add(upper, Vec3{r, r, r})}
}
func preciseLeafCast(leaf queryLeaf, proxy *ShapeProxy, translation Vec3, tolerance float32) (CastOutput, bool) {
	if leaf.triangleIndex >= 0 && proxy.Count == 1 && proxy.Radius > 0 {
		if result, supported := preciseSphereTriangle(leaf, proxy.Points[0], proxy.Radius, translation, tolerance); supported {
			return result, true
		}
	}
	var target ShapeProxy
	var points [3]Vec3
	if leaf.triangleIndex >= 0 {
		points = leaf.triangle
		target = ShapeProxy{Points: points[:], Count: 3}
	} else {
		switch leaf.kind {
		case SphereShape:
			points[0] = leaf.sphere.Center
			target = ShapeProxy{Points: points[:1], Count: 1, Radius: leaf.sphere.Radius}
		case CapsuleShape:
			points[0] = leaf.capsule.Center1
			points[1] = leaf.capsule.Center2
			target = ShapeProxy{Points: points[:2], Count: 2, Radius: leaf.capsule.Radius}
		case HullShape:
			vertices := GetHullPoints(leaf.hull)
			target = ShapeProxy{Points: vertices, Count: int32(len(vertices))}
		default:
			return CastOutput{}, false
		}
	}
	xf := InvertTransform(leaf.transform)
	movement := InvRotateVector(leaf.transform.Q, translation)
	input := DistanceInput{ProxyA: target, ProxyB: *proxy, Transform: xf}
	radius := target.Radius + proxy.Radius
	fraction := float32(0)
	cache := SimplexCache{}
	for iteration := 0; iteration < 64; iteration++ {
		input.Transform.P = MulAdd(xf.P, fraction, movement)
		d := ShapeDistance(&input, &cache, nil)
		if !IsValidFloat(d.Distance) || !IsValidVec3(d.Normal) {
			return CastOutput{}, false
		}
		gap := d.Distance - radius
		if gap <= tolerance {
			point := MulAdd(d.PointA, target.Radius, d.Normal)
			return CastOutput{Hit: true, Fraction: fraction, Point: TransformPoint(leaf.transform, point), Normal: RotateVector(leaf.transform.Q, d.Normal), Iterations: int32(iteration + 1), TriangleIndex: leaf.triangleIndex}, true
		}
		approach := -Dot(movement, d.Normal)
		if approach <= 0 {
			return CastOutput{}, true
		}
		next := fraction + gap/approach
		if next > 1 {
			return CastOutput{}, true
		}
		if next <= fraction {
			return CastOutput{}, false
		}
		fraction = next
	}
	return CastOutput{}, false
}
