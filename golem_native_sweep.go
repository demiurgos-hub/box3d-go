// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import "math"

// continuousShapeRadius bounds the geometry about its body's centre of mass.
func continuousShapeRadius(s *shape, center Vec3) float32 {
	switch s.typ {
	case SphereShape:
		return Distance(s.sphere.Center, center) + s.sphere.Radius
	case CapsuleShape:
		return MaxFloat(Distance(s.capsule.Center1, center), Distance(s.capsule.Center2, center)) + s.capsule.Radius
	default:
		bounds := computeShapeAABB(s, TransformIdentity)
		return Distance(farthestPointOnAABB(bounds, center), center)
	}
}

func continuousRadiusBounds(a, b Vec3, radius float32) AABB {
	// Broad-phase rounding allowance only; TOI and solver tolerances are unchanged.
	scale := MaxFloat(Length(Abs(a)), Length(Abs(b))) + radius + 1
	radius += float32(32 * fltEpsilon * scale)
	pad := Vec3{radius, radius, radius}
	return AABB{Sub(Min(a, b), pad), Add(Max(a, b), pad)}
}

func continuousBoundsRoundoff(bounds AABB) AABB {
	scale := MaxFloat(Length(Abs(bounds.LowerBound)), Length(Abs(bounds.UpperBound))) + 1
	return bounds.Inflate(float32(32 * fltEpsilon * scale))
}

func continuousOffsetBounds(bounds AABB, origin Pos) AABB {
	result := OffsetAABB(bounds, origin)
	result.LowerBound.X = math.Nextafter32(result.LowerBound.X, -math.MaxFloat32)
	result.LowerBound.Y = math.Nextafter32(result.LowerBound.Y, -math.MaxFloat32)
	result.LowerBound.Z = math.Nextafter32(result.LowerBound.Z, -math.MaxFloat32)
	result.UpperBound.X = math.Nextafter32(result.UpperBound.X, math.MaxFloat32)
	result.UpperBound.Y = math.Nextafter32(result.UpperBound.Y, math.MaxFloat32)
	result.UpperBound.Z = math.Nextafter32(result.UpperBound.Z, math.MaxFloat32)
	return result
}

// continuousShapeBounds includes intermediate rotation, not only endpoint poses.
// Every point stays within its COM radius while the COM follows a line segment.
func continuousShapeBounds(s *shape, sweep *Sweep, fraction float32) AABB {
	if sweep.Q1 == sweep.Q2 {
		switch s.typ {
		case SphereShape, CapsuleShape, HullShape:
			return continuousBoundsRoundoff(computeSweptShapeAABB(s, sweep, fraction))
		default:
			return continuousBoundsRoundoff(computeShapeAABB(s, GetSweepTransform(sweep, 0)).Union(computeShapeAABB(s, GetSweepTransform(sweep, fraction))))
		}
	}
	return continuousRadiusBounds(sweep.C1, Lerp(sweep.C1, sweep.C2, fraction), continuousShapeRadius(s, sweep.LocalCenter))
}

func continuousMeshLocalBounds(s *shape, a, b *Sweep, fraction float32) AABB {
	if a.C1 == a.C2 && a.Q1 == a.Q2 && b.Q1 == b.Q2 {
		// Preserve the original static/translation-only candidate arithmetic.
		return computeSweptShapeAABB(s, b, fraction).Transform(InvertTransform(getFinalSweepTransform(a)))
	}
	if a.Q1 == a.Q2 {
		relative := Sweep{LocalCenter: b.LocalCenter, Q1: InvMulQuat(a.Q1, b.Q1), Q2: InvMulQuat(a.Q1, b.Q2),
			C1: Add(a.LocalCenter, InvRotateVector(a.Q1, Sub(b.C1, a.C1))),
			C2: Add(a.LocalCenter, InvRotateVector(a.Q1, Sub(b.C2, a.C2)))}
		return continuousShapeBounds(s, &relative, fraction)
	}
	// Rotation of the mesh frame preserves distances. The relative COM follows
	// a line before that rotation, so its largest radius occurs at an endpoint.
	d1 := Sub(b.C1, a.C1)
	d2 := Lerp(d1, Sub(b.C2, a.C2), fraction)
	radius := MaxFloat(Length(d1), Length(d2)) + continuousShapeRadius(s, b.LocalCenter)
	return continuousRadiusBounds(a.LocalCenter, a.LocalCenter, radius)
}
