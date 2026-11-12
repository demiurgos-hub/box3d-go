// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// PairPenetrationResult is the deepest penetrating leaf contact in a shape pair.
// Normal points from A to B and Depth is positive for overlap, zero otherwise.
// PointA/PointB are geometric witnesses with PointB-PointA = -Depth*Normal.
// The deepest leaf contact is not a global MTD for a concave compound.
type PairPenetrationResult struct {
	PointA, PointB        Pos
	Normal                Vec3
	Depth                 float32
	Touching, Penetrating bool
	FeatureA, FeatureB    QueryFeature
	MaterialA, MaterialB  uint64
}

// PairPenetration finds the deepest positive geometric contact between two
// shapes in the same world. Triangle surfaces are two-sided. The bool reports
// valid/supported inputs, not overlap: inspect Depth > 0. Triangle/triangle
// leaf pairs are unsupported and return false; no partial result is returned.
// Primitives, convex hulls and compounds against mesh/terrain are supported.
// Touching is true only for exact zero separation without penetration. Positive
// solver/speculative slop is never classified as overlap. No filter applies.
// This method follows ClosestPointFeature's ownership and lifetime rules.
func (id ShapeID) PairPenetration(other ShapeID) (PairPenetrationResult, bool) {
	var result PairPenetrationResult
	if id.World0 != other.World0 {
		return result, false
	}
	a, _, valid := queryShape(id, Pos{})
	if !valid {
		return result, false
	}
	origin := id.GetBody().GetPosition()
	a, xa, valid := queryShape(id, origin)
	if !valid {
		return result, false
	}
	b, xb, valid := queryShape(other, origin)
	if !valid {
		return result, false
	}
	bounds := computeShapeAABB(b, xb)
	visitQueryGeometry(a, xa, bounds, func(la queryLeaf) {
		visitQueryGeometry(b, xb, queryLeafBounds(la), func(lb queryLeaf) {
			if !valid {
				return
			}
			valid = collideQueryLeaves(la, lb, true, func(point, normal Vec3, separation float32, fa, fb uint32) {
				if separation > 0 {
					return
				}
				if separation == 0 {
					if !result.Penetrating {
						result.Touching = true
					}
					return
				}
				featureA := QueryFeature{la.child, la.triangleIndex, fa}
				featureB := QueryFeature{lb.child, lb.triangleIndex, fb}
				depth := -separation
				if depth < result.Depth || (depth == result.Depth && !queryPairFeatureLess(featureA, featureB, result.FeatureA, result.FeatureB)) {
					return
				}
				result = PairPenetrationResult{
					PointA: OffsetPos(origin, MulAdd(point, 0.5*depth, normal)),
					PointB: OffsetPos(origin, MulSub(point, 0.5*depth, normal)),
					Normal: normal, Depth: depth, Penetrating: true, FeatureA: featureA, FeatureB: featureB,
					MaterialA: getShapeUserMaterialId(a, int(la.child), int(la.triangleIndex)),
					MaterialB: getShapeUserMaterialId(b, int(lb.child), int(lb.triangleIndex)),
				}
			})
		})
	})
	if !valid {
		return PairPenetrationResult{}, false
	}
	return result, true
}

// ProxyPenetrations writes touching/penetrating contacts between this shape and
// a proxy relative to origin. It has CapsulePenetrations' buffer/ownership rules.
// One/two points with positive radius represent a sphere/capsule. Four or more
// points with zero radius must form a full-dimensional convex hull; it is cooked
// for this call. Rounded hulls and lower-dimensional point clouds return false.
// Cooking can allocate; persistent convex queries should use PairPenetration.
func (id ShapeID) ProxyPenetrations(origin Pos, proxy *ShapeProxy, backFaces bool, dst []CapsulePenetration) (written, total int, valid bool) {
	context, transform, valid := id.prepareProxyPenetration(origin, proxy, backFaces)
	if !valid {
		return 0, 0, false
	}
	defer releaseProxyPenetration(context)
	context.dst = dst
	visitQueryGeometry(context.shape, transform, queryLeafBounds(context.query), context.visit)
	return context.written, context.total, true
}

func queryPairFeatureLess(a, b, c, d QueryFeature) bool {
	return queryFeatureLess(a, c) || (a == c && queryFeatureLess(b, d))
}

func queryLeafBounds(leaf queryLeaf) AABB {
	if leaf.triangleIndex >= 0 {
		var vertices [3]Vec3
		for i := range vertices {
			vertices[i] = TransformPoint(leaf.transform, leaf.triangle[i])
		}
		return MakeAABB(vertices[:], 0)
	}
	switch leaf.kind {
	case SphereShape:
		return ComputeSphereAABB(&leaf.sphere, leaf.transform)
	case CapsuleShape:
		return ComputeCapsuleAABB(&leaf.capsule, leaf.transform)
	default:
		return ComputeHullAABB(leaf.hull, leaf.transform)
	}
}

func queryLeafRank(leaf queryLeaf) int {
	if leaf.triangleIndex >= 0 {
		return 4
	}
	switch leaf.kind {
	case HullShape:
		return 3
	case CapsuleShape:
		return 2
	default:
		return 1
	}
}

func collideQueryLeaves(a, b queryLeaf, backFaces bool, emit func(point, normal Vec3, separation float32, fa, fb uint32)) bool {
	if a.triangleIndex >= 0 && b.triangleIndex >= 0 {
		return false
	}
	if queryLeafRank(a) < queryLeafRank(b) {
		return collideQueryLeaves(b, a, backFaces, func(point, normal Vec3, separation float32, fa, fb uint32) {
			emit(point, Neg(normal), separation, fb, fa)
		})
	}
	var points [MaxManifoldPoints]LocalManifoldPoint
	m := LocalManifold{Points: points[:]}
	var simple SimplexCache
	var sat SATCache
	xf := InvMulTransforms(a.transform, b.transform)
	frame := a.transform
	reversedTriangle := false
	if a.triangleIndex >= 0 {
		// Triangle routines work in convex B's local coordinates.
		frame = b.transform
		tx := InvMulTransforms(b.transform, a.transform)
		var tri [3]Vec3
		for i := range tri {
			tri[i] = TransformPoint(tx, a.triangle[i])
		}
		if LengthSquared(Cross(Sub(tri[1], tri[0]), Sub(tri[2], tri[0]))) == 0 {
			return true
		}
		center := b.sphere.Center
		if b.kind == CapsuleShape {
			center = Lerp(b.capsule.Center1, b.capsule.Center2, 0.5)
		} else if b.kind == HullShape {
			center = b.hull.Center
		}
		flags := a.triangleFlags
		if signedVolume(tri[0], tri[1], tri[2], center) < 0 {
			if !backFaces {
				return true
			}
			tri[1], tri[2] = tri[2], tri[1]
			reversedTriangle = true
			// Reversing the face swaps edge identities and concavity.
			flags = querySwapTriangleEdgeFlags(((flags & 0x07) << 4) | ((flags & 0x70) >> 4))
		}
		switch b.kind {
		case SphereShape:
			CollideTriangleAndSphere(&m, len(points), tri[:], &b.sphere)
		case CapsuleShape:
			CollideTriangleAndCapsule(&m, len(points), tri[:], &b.capsule, &simple)
			queryPreferFlatTriangleFace(&m, tri[:], &b.capsule, flags)
		case HullShape:
			CollideTriangleAndHull(&m, len(points), tri[0], tri[1], tri[2], int(flags), b.hull, &sat, false)
			if m.Feature == FeatureTriangleFace || m.Feature == FeatureHullFace {
				for i := range int(m.PointCount) {
					m.Points[i].Point = MulSub(m.Points[i].Point, 0.5*m.Points[i].Separation, m.Normal)
				}
			}
		}
	} else {
		switch {
		case a.kind == HullShape && b.kind == HullShape:
			CollideHulls(&m, len(points), a.hull, b.hull, xf, &sat)
		case a.kind == HullShape && b.kind == CapsuleShape:
			CollideHullAndCapsule(&m, len(points), a.hull, &b.capsule, xf, &simple)
		case a.kind == HullShape && b.kind == SphereShape:
			CollideHullAndSphere(&m, len(points), a.hull, &b.sphere, xf, &simple)
			// Upstream's deep sphere witness uses the zero-distance GJK point.
			// Recover the actual sphere surface and geometric midpoint instead.
			for i := range int(m.PointCount) {
				p := MulSub(TransformPoint(xf, b.sphere.Center), b.sphere.Radius, m.Normal)
				m.Points[i].Point = MulSub(p, 0.5*m.Points[i].Separation, m.Normal)
			}
		case a.kind == CapsuleShape && b.kind == CapsuleShape:
			CollideCapsules(&m, len(points), &a.capsule, &b.capsule, xf)
		case a.kind == CapsuleShape && b.kind == SphereShape:
			CollideCapsuleAndSphere(&m, len(points), &a.capsule, &b.sphere, xf)
		default:
			CollideSpheres(&m, len(points), &a.sphere, &b.sphere, xf)
		}
	}
	for _, point := range m.Points[:m.PointCount] {
		fa, fb := makeFeatureId(point.Pair), makeFeatureId(flipPair(point.Pair))
		if a.triangleIndex >= 0 {
			fa = uint32(m.Feature)
			if reversedTriangle {
				switch m.Feature {
				case FeatureEdge1:
					fa = uint32(FeatureEdge3)
				case FeatureEdge3:
					fa = uint32(FeatureEdge1)
				case FeatureVertex2:
					fa = uint32(FeatureVertex3)
				case FeatureVertex3:
					fa = uint32(FeatureVertex2)
				}
			}
		}
		emit(TransformPoint(frame, point.Point), RotateVector(frame.Q, m.Normal), point.Separation, fa, fb)
	}
	return true
}

func querySwapTriangleEdgeFlags(flags int32) int32 {
	edge1 := flags & int32(FlatEdge1)
	edge3 := flags & int32(FlatEdge3)
	return flags&^int32(FlatEdge1|FlatEdge3) | edge1<<2 | edge3>>2
}

func queryPreferFlatTriangleFace(m *LocalManifold, tri []Vec3, capsule *Capsule, flags int32) {
	var flat MeshEdgeFlags
	switch m.Feature {
	case FeatureEdge1:
		flat = FlatEdge1
	case FeatureEdge2:
		flat = FlatEdge2
	case FeatureEdge3:
		flat = FlatEdge3
	default:
		return
	}
	if MeshEdgeFlags(flags)&flat != flat {
		return
	}
	// A shared coplanar edge is not a physical wall. Preserve clipped face
	// contacts for deep capsule cores instead of the triangle's artificial edge.
	var points [MaxManifoldPoints]LocalManifoldPoint
	face := LocalManifold{Points: points[:]}
	buildTriangleAndCapsuleFaceContact(&face, tri, makePlaneFromPoints(tri[0], tri[1], tri[2]), capsule)
	if face.PointCount > 0 {
		copy(m.Points, face.Points[:face.PointCount])
		m.PointCount, m.Normal, m.Feature = face.PointCount, face.Normal, face.Feature
	}
}
