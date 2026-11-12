// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// Solver normals may be a float32 ULP short of unit length. That must not turn
// exact geometric touching into penetration for a zero-tolerance placement query.
// Recompute face distance from the authored float32 vertices in a local frame;
// this does not apply an overlap epsilon or alter upstream solver manifolds.
func queryRefineTriangleFace(m *LocalManifold, tri []Vec3, convex queryLeaf) {
	if m.PointCount == 0 || m.Feature != FeatureTriangleFace || convex.kind != SphereShape && convex.kind != CapsuleShape {
		return
	}
	a := tri[0]
	raw := Cross(Sub(tri[1], a), Sub(tri[2], a))
	scale := MaxFloat(AbsFloat(raw.X), MaxFloat(AbsFloat(raw.Y), AbsFloat(raw.Z)))
	if scale == 0 {
		return
	}
	// Divide components directly: multiplying by an approximate reciprocal can
	// leave even an axis-aligned plane short of unit length.
	scaled := Vec3{raw.X / scale, raw.Y / scale, raw.Z / scale}
	length := Length(scaled)
	normal := Vec3{scaled.X / length, scaled.Y / length, scaled.Z / length}
	distance := func(point Vec3) float32 {
		return Dot(Sub(point, a), scaled) / length
	}
	radius := convex.sphere.Radius
	segment := [2]clipVertex{{position: convex.sphere.Center, pair: featurePairSingle}}
	count := 1
	if convex.kind == CapsuleShape {
		radius = convex.capsule.Radius
		segment = [2]clipVertex{
			{position: convex.capsule.Center1, pair: makeFeaturePair(featureShapeA, 0, featureShapeA, 0)},
			{position: convex.capsule.Center2, pair: makeFeaturePair(featureShapeA, 1, featureShapeA, 1)},
		}
		if !clipSegmentToTriangleFace(&segment, tri, makePlaneFromNormalAndPoint(normal, a)) {
			return
		}
		count = 2
	}
	m.Normal, m.PointCount = normal, int32(count)
	for i := 0; i < count; i++ {
		d := distance(segment[i].position)
		m.Points[i].Separation = d - radius
		m.Points[i].Point = MulSub(segment[i].position, .5*(d+radius), normal)
		m.Points[i].Pair = segment[i].pair
	}
}
