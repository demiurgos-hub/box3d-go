// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// ProxyPenetrationDeepest returns the first deepest touching/penetrating contact
// in ProxyPenetrations traversal order. Found is false for separation/backface
// culling; valid is false for the same unsupported/invalid inputs as that method.
// It retains one result regardless of triangle count, without an all-contact
// buffer or retry. Convex proxy cooking and internal traversal may allocate.
func (id ShapeID) ProxyPenetrationDeepest(origin Pos, proxy *ShapeProxy, backFaces bool) (result CapsulePenetration, found, valid bool) {
	context, transform, valid := id.prepareProxyPenetration(origin, proxy, backFaces)
	if !valid {
		return result, false, false
	}
	defer releaseProxyPenetration(context)
	context.deepest = true
	visitQueryGeometry(context.shape, transform, queryLeafBounds(context.query), context.visit)
	return context.best, context.total > 0, true
}

type proxyPenetrationContext struct {
	shape              *shape
	origin             Pos
	query, leaf        queryLeaf
	backFaces, deepest bool
	dst                []CapsulePenetration
	written, total     int
	best               CapsulePenetration
	visit              func(queryLeaf)
	emit               func(Vec3, Vec3, float32, uint32, uint32)
}

var proxyPenetrationScratch scratchPool[proxyPenetrationContext]

func (id ShapeID) prepareProxyPenetration(origin Pos, proxy *ShapeProxy, backFaces bool) (*proxyPenetrationContext, Transform, bool) {
	s, transform, valid := queryShape(id, origin)
	if !valid || !validPreciseProxy(proxy) {
		return nil, Transform{}, false
	}
	context := proxyPenetrationScratch.get()
	if context.visit == nil {
		context.visit = context.visitLeaf
		context.emit = context.emitPoint
	}
	context.shape, context.origin, context.backFaces = s, origin, backFaces
	context.query = queryLeaf{transform: TransformIdentity, child: -1, triangleIndex: -1}
	query := &context.query
	if proxy.Count <= 2 && proxy.Radius > 0 {
		if proxy.Count == 1 || proxy.Points[0] == proxy.Points[1] {
			query.kind, query.sphere = SphereShape, Sphere{proxy.Points[0], proxy.Radius}
		} else {
			query.kind, query.capsule = CapsuleShape, Capsule{proxy.Points[0], proxy.Points[1], proxy.Radius}
		}
	} else if proxy.Count >= 4 && proxy.Radius == 0 {
		query.kind = HullShape
		query.hull = CreateHull(proxy.Points[:proxy.Count], int(proxy.Count), MaxHullVertices)
		if query.hull == nil {
			releaseProxyPenetration(context)
			return nil, Transform{}, false
		}
	} else {
		releaseProxyPenetration(context)
		return nil, Transform{}, false
	}
	return context, transform, true
}

func (c *proxyPenetrationContext) visitLeaf(leaf queryLeaf) {
	c.leaf = leaf
	collideQueryLeaves(leaf, c.query, c.backFaces, c.emit)
}

func (c *proxyPenetrationContext) emitPoint(point, normal Vec3, separation float32, feature, _ uint32) {
	if separation > 0 {
		return
	}
	previous := c.total
	c.total++
	if c.deepest {
		if previous > 0 && -separation <= c.best.Depth {
			return
		}
	} else if c.written >= len(c.dst) {
		return
	}
	hit := CapsulePenetration{Point: OffsetPos(c.origin, point), Normal: normal, Depth: -separation, Feature: QueryFeature{c.leaf.child, c.leaf.triangleIndex, feature}, MaterialID: getShapeUserMaterialId(c.shape, int(c.leaf.child), int(c.leaf.triangleIndex))}
	if c.deepest {
		c.best = hit
	} else {
		c.dst[c.written] = hit
		c.written++
	}
}

func releaseProxyPenetration(c *proxyPenetrationContext) {
	if c.query.hull != nil {
		DestroyHull(c.query.hull)
	}
	visit, emit := c.visit, c.emit
	*c = proxyPenetrationContext{visit: visit, emit: emit}
	proxyPenetrationScratch.put(c)
}
