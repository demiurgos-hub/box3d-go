// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// QueryFeature identifies geometry within one unchanged cooked shape. Indices are
// not durable content IDs and must not be reused after geometry replacement.
// ChildIndex and TriangleIndex are -1 when that level is absent. FeatureID is an
// opaque local feature: closest-point and penetration feature IDs are distinct domains.
type QueryFeature struct {
	ChildIndex    int32
	TriangleIndex int32
	FeatureID     uint32
}

// ClosestPointFeatureResult describes the closest point on a solid primitive or
// triangle surface. Inside a solid primitive, Point is the target, Distance is
// zero and Normal is zero. Meshes/heightfields have surfaces, not an inferred interior.
type ClosestPointFeatureResult struct {
	Point      Pos
	Normal     Vec3
	Distance   float32
	Inside     bool
	Feature    QueryFeature
	MaterialID uint64
}

// CapsulePenetration is a geometric contact, without solver impulses. Normal
// points from the queried shape toward the capsule; Depth is nonnegative.
// Several contacts may belong to one triangle/child. They are constraints, not
// a guarantee that any single translation resolves an entire compound overlap.
type CapsulePenetration struct {
	Point      Pos
	Normal     Vec3
	Depth      float32
	Feature    QueryFeature
	MaterialID uint64
}

// ClosestPointFeature finds the closest point within maxDistance, inclusive.
// It supports every shape kind, including triangle meshes, terrain holes and
// compounds. Triangle surfaces are two-sided. Equal-distance ties use ascending
// child, triangle and feature IDs. maxDistance must be finite and nonnegative.
// It returns false for invalid handles/inputs, a locked world, or no nearby point.
// Like other Box3D queries it must not race world mutations or Step. It reads
// current body transforms even for disabled bodies and applies no layer filters.
func (id ShapeID) ClosestPointFeature(target Pos, maxDistance float32) (ClosestPointFeatureResult, bool) {
	var result ClosestPointFeatureResult
	s, transform, valid := queryShape(id, target)
	if !valid || !IsValidFloat(maxDistance) || maxDistance < 0 {
		return result, false
	}
	r := Vec3{maxDistance, maxDistance, maxDistance}
	bounds := AABB{Neg(r), r}
	if !IsValidAABB(bounds) {
		return result, false
	}
	found := false
	visitQueryGeometry(s, transform, bounds, func(leaf queryLeaf) {
		point, inside, feature := leafClosestPoint(leaf, InvTransformPoint(leaf.transform, Vec3Zero))
		point = TransformPoint(leaf.transform, point)
		normal, distance := GetLengthAndNormalize(Neg(point))
		if inside {
			point, normal, distance = Vec3Zero, Vec3Zero, 0
		}
		identity := QueryFeature{leaf.child, leaf.triangleIndex, feature}
		if !IsValidFloat(distance) || distance > maxDistance ||
			(found && (distance > result.Distance || (distance == result.Distance && !queryFeatureLess(identity, result.Feature)))) {
			return
		}
		result = ClosestPointFeatureResult{OffsetPos(target, point), normal, distance, inside, identity,
			getShapeUserMaterialId(s, int(leaf.child), int(leaf.triangleIndex))}
		found = true
	})
	return result, found
}

// CapsulePenetrations writes touching/penetrating contacts into dst and returns
// (written, total, valid). total counts all contacts even if dst is too short;
// callers must handle overflow before using a partial set for movement resolution.
// Capsule centers are relative to origin. A zero-length capsule is a sphere.
// Radius must be positive. When backFaces is false triangles are one-sided,
// using their authored winding and the capsule midpoint to choose the side.
// When true, the winding is oriented toward that midpoint before narrowphase.
// Contacts include deep core overlap, unlike the upstream CollideMover helper.
// Order is backend traversal order, not a stable ordering; Feature is stable for
// unchanged geometry. This method follows ClosestPointFeature's ownership rules.
func (id ShapeID) CapsulePenetrations(origin Pos, capsule *Capsule, backFaces bool, dst []CapsulePenetration) (written, total int, valid bool) {
	if capsule == nil || capsule.Radius <= 0 {
		return 0, 0, false
	}
	points := [2]Vec3{capsule.Center1, capsule.Center2}
	proxy := ShapeProxy{points[:], 2, capsule.Radius}
	return id.ProxyPenetrations(origin, &proxy, backFaces, dst)
}
func queryShape(id ShapeID, origin Pos) (*shape, Transform, bool) {
	if !id.IsValid() || !IsValidPosition(origin) {
		return nil, Transform{}, false
	}
	w := &worlds[id.World0]
	if w.locked {
		return nil, Transform{}, false
	}
	s := &w.shapes[id.Index1-1]
	b := &w.bodies[s.bodyId]
	return s, ToRelativeTransform(getBodyTransformQuick(w, b), origin), true
}

func queryFeatureLess(a, b QueryFeature) bool {
	if a.ChildIndex != b.ChildIndex {
		return a.ChildIndex < b.ChildIndex
	}
	if a.TriangleIndex != b.TriangleIndex {
		return a.TriangleIndex < b.TriangleIndex
	}
	return a.FeatureID < b.FeatureID
}

type queryLeaf struct {
	kind                 ShapeType
	sphere               Sphere
	capsule              Capsule
	hull                 *HullData
	triangle             [3]Vec3
	triangleFlags        int32
	transform            Transform
	child, triangleIndex int32
}

func visitQueryGeometry(s *shape, transform Transform, bounds AABB, visit func(queryLeaf)) {
	localBounds := bounds.Transform(InvertTransform(transform))
	if s.typ == CompoundShape {
		QueryCompound(s.compound, localBounds, func(compound *CompoundData, childIndex int, _ any) bool {
			child := GetCompoundChild(compound, childIndex)
			leaf := queryLeaf{kind: child.Type, sphere: child.Sphere, capsule: child.Capsule, hull: child.Hull,
				transform: MulTransforms(transform, child.Transform), child: int32(childIndex), triangleIndex: -1}
			if child.Type == MeshShape {
				visitQueryMesh(&child.Mesh, bounds.Transform(InvertTransform(leaf.transform)), leaf, visit)
			} else {
				visit(leaf)
			}
			return true
		}, nil)
		return
	}
	leaf := queryLeaf{kind: s.typ, sphere: s.sphere, capsule: s.capsule, hull: s.hull, transform: transform, child: -1, triangleIndex: -1}
	switch s.typ {
	case MeshShape:
		visitQueryMesh(&s.mesh, localBounds, leaf, visit)
	case HeightShape:
		// Clamp first: upstream QueryHeightField iterates out-of-range rows/columns.
		heightBounds := queryHeightFieldLocalBounds(s.heightField)
		localBounds.LowerBound = Max(localBounds.LowerBound, heightBounds.LowerBound)
		localBounds.UpperBound = Min(localBounds.UpperBound, heightBounds.UpperBound)
		if !IsValidAABB(localBounds) {
			return
		}
		QueryHeightField(s.heightField, localBounds, func(a, b, c Vec3, index int, _ any) bool {
			leaf.triangle, leaf.triangleIndex = [3]Vec3{a, b, c}, int32(index)
			leaf.triangleFlags = getHeightFieldTriangle(s.heightField, index).flags
			visit(leaf)
			return true
		}, nil)
	default:
		visit(leaf)
	}
}

// ComputeHeightFieldQueryAABB bounds the quantized heightfield surface, including
// compression and float32 rounding error. Unlike the legacy authored bounds this
// is conservative for collision/query traversal. Cooked blob bytes are unchanged.
func ComputeHeightFieldQueryAABB(field *HeightFieldData, transform Transform) AABB {
	return queryHeightFieldLocalBounds(field).Transform(transform)
}

func queryHeightFieldLocalBounds(field *HeightFieldData) AABB {
	bounds := field.AABB
	// Compression truncates by up to one step. The additional float32 error
	// bound covers normalization and decompression at the global height extrema.
	rounding := float32(4 * fltEpsilon * MaxFloat(AbsFloat(field.MinHeight), AbsFloat(field.MaxHeight)))
	error := float32(AbsFloat(field.Scale.Y) * (field.HeightScale + rounding))
	bounds.LowerBound.Y -= error
	bounds.UpperBound.Y += error
	return bounds
}

func visitQueryMesh(mesh *Mesh, bounds AABB, leaf queryLeaf, visit func(queryLeaf)) {
	QueryMesh(mesh, bounds, func(a, b, c Vec3, index int, _ any) bool {
		leaf.triangle, leaf.triangleIndex = [3]Vec3{a, b, c}, int32(index)
		leaf.triangleFlags = int32(GetMeshFlags(mesh.Data)[index])
		if mesh.Scale.X*mesh.Scale.Y*mesh.Scale.Z < 0 {
			// QueryMesh swaps the reflected triangle's second/third vertices.
			// Reflection plus winding correction preserves physical concavity.
			leaf.triangleFlags = querySwapTriangleEdgeFlags(leaf.triangleFlags)
		}
		visit(leaf)
		return true
	}, nil)
}

func leafClosestPoint(leaf queryLeaf, target Vec3) (point Vec3, inside bool, feature uint32) {
	if leaf.triangleIndex >= 0 {
		p := queryClosestTriangle(leaf.triangle[0], leaf.triangle[1], leaf.triangle[2], target)
		return p.point, false, uint32(p.feature)
	}
	switch leaf.kind {
	case SphereShape:
		return closestRoundPoint(leaf.sphere.Center, leaf.sphere.Radius, target)
	case CapsuleShape:
		return closestRoundPoint(PointToSegmentDistance(leaf.capsule.Center1, leaf.capsule.Center2, target), leaf.capsule.Radius, target)
	case HullShape:
		inside = true
		for _, plane := range GetHullPlanes(leaf.hull) {
			if planeSeparation(plane, target) > 0 {
				inside = false
				break
			}
		}
		if inside {
			return target, true, 0
		}
		vertices, edges := GetHullPoints(leaf.hull), GetHullEdges(leaf.hull)
		best := fltMax
		for index, face := range GetHullFaces(leaf.hull) {
			first := face.Edge
			e := edges[first]
			a := vertices[e.Origin]
			e = edges[e.Next]
			b := vertices[e.Origin]
			for next := e.Next; next != first; next = edges[next].Next {
				c := vertices[edges[next].Origin]
				p := queryClosestTriangle(a, b, c, target).point
				d := DistanceSquared(p, target)
				if d < best {
					point, feature, best = p, uint32(index), d
				}
				b = c
			}
		}
		return point, false, feature
	}
	return target, false, 0
}

func closestRoundPoint(center Vec3, radius float32, target Vec3) (Vec3, bool, uint32) {
	normal, distance := GetLengthAndNormalize(Sub(target, center))
	if distance <= radius {
		return target, true, 0
	}
	return MulAdd(center, radius, normal), false, 0
}

func queryClosestTriangle(a, b, c, target Vec3) trianglePoint {
	if LengthSquared(Cross(Sub(b, a), Sub(c, a))) != 0 {
		return closestPointOnTriangle(a, b, c, target)
	}
	// Degenerate cooked triangles still have a well-defined closest point.
	points := [3]Vec3{PointToSegmentDistance(a, b, target), PointToSegmentDistance(b, c, target), PointToSegmentDistance(c, a, target)}
	best := 0
	for i := 1; i < 3; i++ {
		if DistanceSquared(points[i], target) < DistanceSquared(points[best], target) {
			best = i
		}
	}
	return trianglePoint{points[best], TriangleFeature(int(FeatureEdge1) + best)}
}
