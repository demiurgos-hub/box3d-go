// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"errors"
	"fmt"
	"math"
)

// ErrMotionCastInvalid reports stale handles or invalid motion/query input.
var ErrMotionCastInvalid = errors.New("box3d: invalid motion cast")

// ErrMotionCastConvergence reports an unresolved cast, never a clear trajectory.
var ErrMotionCastConvergence = errors.New("box3d: motion cast did not converge")

// ErrMotionCastUnsupported reports geometry or sidedness not supported by this query.
var ErrMotionCastUnsupported = errors.New("box3d: unsupported motion cast")

// MotionCastResult describes the earliest leaf-pair contact within Tolerance.
// Normal points from A to B. Touching/Penetrating describe exact geometric
// contact, not the positive tolerance band. Initial means that geometric contact
// already existed at fraction zero. Depth is a leaf constraint, not a union MTD.
// Features are valid only for unchanged cooked geometry. Misses have Hit=false.
type MotionCastResult struct {
	Hit, Initial, Touching, Penetrating bool
	Fraction, Separation, Depth         float32
	PointA, PointB                      Pos
	Normal                              Vec3
	FeatureA, FeatureB                  QueryFeature
	MaterialA, MaterialB                uint64
	Iterations                          int
}

type motionPath struct {
	start, end   Transform
	angle        float32
	center, axis Vec3
}

// MotionBounds conservatively bounds a shape's complete translation/rotation
// trajectory. It includes rotational arcs absent from endpoint AABB unions.
// Poses are body-origin poses and no world mutation or filtering is performed.
func (id ShapeID) MotionBounds(start, end WorldTransform) (AABB, error) {
	if !IsValidPosition(start.P) || !IsValidPosition(end.P) || !IsValidQuat(start.Q) || !IsValidQuat(end.Q) || !IsNormalizedQuat(start.Q) || !IsNormalizedQuat(end.Q) {
		return AABB{}, ErrMotionCastInvalid
	}
	s, _, ok := queryShape(id, start.P)
	if !ok {
		return AABB{}, ErrMotionCastInvalid
	}
	m := newMotionPath(ToRelativeTransform(start, start.P), ToRelativeTransform(end, start.P))
	if m.angle == 0 {
		a := computeShapeAABB(s, m.start)
		z := computeShapeAABB(s, m.end)
		return OffsetAABB(a.Union(z), start.P), nil
	}
	return OffsetAABB(motionBounds(m, motionShapeRadius(s, Vec3Zero), 0), start.P), nil
}

func newMotionPath(a, z Transform) motionPath {
	if DotQuat(a.Q, z.Q) < 0 {
		z.Q = NegateQuat(z.Q)
	}
	delta := InvMulQuat(a.Q, z.Q)
	dx, dy, dz := float64(delta.V.X), float64(delta.V.Y), float64(delta.V.Z)
	length := math.Sqrt(float64(dx*dx) + float64(dy*dy) + float64(dz*dz))
	return motionPath{start: a, end: z, angle: float32(2 * math.Atan2(length, float64(delta.S))), axis: Normalize(delta.V)}
}

func (m motionPath) at(f float32) Transform {
	if f == 0 {
		return Transform{Sub(m.start.P, RotateVector(m.start.Q, m.center)), m.start.Q}
	}
	if f == 1 {
		return Transform{Sub(m.end.P, RotateVector(m.end.Q, m.center)), m.end.Q}
	}
	q := m.start.Q
	if m.angle > 0 {
		half := float64(m.angle*f) * 0.5
		q = NormalizeQuat(MulQuat(m.start.Q, Quat{MulSV(float32(math.Sin(half)), m.axis), float32(math.Cos(half))}))
	}
	return Transform{Sub(Lerp(m.start.P, m.end.P, f), RotateVector(q, m.center)), q}
}

// CastMotion sweeps both shapes along linear body origins and shortest-arc
// slerped rotations over [0,1]. Poses are BODY poses; local shape/compound
// transforms are retained. At most 180 degrees can be described by one segment.
// It reads geometry without moving bodies and applies no collision filters.
// It rejects locked/recording worlds and triangle/triangle leaf pairs. Rotating
// triangle sidedness currently requires backFaces=true; no unsupported result
// is reported as a miss. This is additive and does not change solver CCD.
func (id ShapeID) CastMotion(other ShapeID, aStart, aEnd, bStart, bEnd WorldTransform, tolerance float32, backFaces bool) (MotionCastResult, error) {
	if id.World0 != other.World0 || !IsValidFloat(tolerance) || tolerance <= 0 {
		return MotionCastResult{}, ErrMotionCastInvalid
	}
	for _, x := range []WorldTransform{aStart, aEnd, bStart, bEnd} {
		if !IsValidPosition(x.P) || !IsValidQuat(x.Q) || !IsNormalizedQuat(x.Q) {
			return MotionCastResult{}, ErrMotionCastInvalid
		}
	}
	a, _, ok := queryShape(id, aStart.P)
	if !ok {
		return MotionCastResult{}, ErrMotionCastInvalid
	}
	b, _, ok := queryShape(other, aStart.P)
	if !ok {
		return MotionCastResult{}, ErrMotionCastInvalid
	}
	if worlds[id.World0].recording != nil {
		return MotionCastResult{}, ErrMotionCastUnsupported
	}
	base := aStart.P
	ma := newMotionPath(ToRelativeTransform(aStart, base), ToRelativeTransform(aEnd, base))
	mb := newMotionPath(ToRelativeTransform(bStart, base), ToRelativeTransform(bEnd, base))
	return motionCastShapes(a, b, ma, mb, base, tolerance, backFaces)
}

func motionCastShapes(a, b *shape, ma, mb motionPath, base Pos, tolerance float32, backFaces bool) (MotionCastResult, error) {
	return motionVisitShapeMotion(a, b, ma, mb, base, tolerance, backFaces, nil)
}

func motionVisitShapeMotion(a, b *shape, ma, mb motionPath, base Pos, tolerance float32, backFaces bool, visit func(MotionCastResult) error) (MotionCastResult, error) {
	ra, rb := motionShapeRadius(a, ma.center), motionShapeRadius(b, mb.center)
	if !IsValidFloat(ra) || !IsValidFloat(rb) {
		return MotionCastResult{}, ErrMotionCastInvalid
	}
	if !motionBounds(ma, ra, tolerance).Overlaps(motionBounds(mb, rb, tolerance)) {
		return MotionCastResult{}, nil
	}
	var result MotionCastResult
	var failure error
	visitQueryGeometry(a, TransformIdentity, motionLocalCandidates(ma, mb, rb, tolerance), func(la queryLeaf) {
		if failure != nil {
			return
		}
		pa, ok := motionLeafProxy(la)
		if !ok {
			failure = ErrMotionCastUnsupported
			return
		}
		laRadius := motionProxyRadius(pa, la.transform, ma.center)
		visitQueryGeometry(b, TransformIdentity, motionLocalCandidates(mb, ma, laRadius, tolerance), func(lb queryLeaf) {
			if failure != nil {
				return
			}
			if la.triangleIndex >= 0 && lb.triangleIndex >= 0 {
				failure = ErrMotionCastUnsupported
				return
			}
			pb, ok := motionLeafProxy(lb)
			if !ok {
				failure = ErrMotionCastUnsupported
				return
			}
			if !backFaces && (la.triangleIndex >= 0 || lb.triangleIndex >= 0) && (ma.angle > 0 || mb.angle > 0) {
				failure = ErrMotionCastUnsupported
				return
			}
			r, err := motionCastLeaves(la, lb, pa, pb, ma, mb, laRadius, motionProxyRadius(pb, lb.transform, mb.center), tolerance, backFaces)
			if err != nil {
				failure = err
				return
			}
			if !r.Hit {
				return
			}
			if visit != nil {
				if err := visit(r); err != nil {
					failure = err
					return
				}
			}
			if !result.Hit || r.Fraction < result.Fraction || r.Fraction == result.Fraction && (r.Penetrating && !result.Penetrating || r.Penetrating == result.Penetrating && queryPairFeatureLess(r.FeatureA, r.FeatureB, result.FeatureA, result.FeatureB)) {
				result = r
			}
		})
	})
	if failure != nil {
		return MotionCastResult{}, failure
	}
	if result.Hit {
		result.PointA = OffsetPos(base, ToVec3(result.PointA))
		result.PointB = OffsetPos(base, ToVec3(result.PointB))
		result.MaterialA = getShapeUserMaterialId(a, int(result.FeatureA.ChildIndex), int(result.FeatureA.TriangleIndex))
		result.MaterialB = getShapeUserMaterialId(b, int(result.FeatureB.ChildIndex), int(result.FeatureB.TriangleIndex))
	}
	return result, nil
}

func motionLeafProxy(leaf queryLeaf) (ShapeProxy, bool) {
	if leaf.triangleIndex >= 0 {
		return ShapeProxy{Points: leaf.triangle[:], Count: 3}, true
	}
	switch leaf.kind {
	case SphereShape:
		return ShapeProxy{Points: []Vec3{leaf.sphere.Center}, Count: 1, Radius: leaf.sphere.Radius}, true
	case CapsuleShape:
		return ShapeProxy{Points: []Vec3{leaf.capsule.Center1, leaf.capsule.Center2}, Count: 2, Radius: leaf.capsule.Radius}, true
	case HullShape:
		return ShapeProxy{Points: GetHullPoints(leaf.hull), Count: leaf.hull.VertexCount}, true
	default:
		return ShapeProxy{}, false
	}
}
func motionProxyRadius(p ShapeProxy, local Transform, center Vec3) float32 {
	r := float32(0)
	for _, v := range p.Points[:p.Count] {
		r = MaxFloat(r, Length(Sub(TransformPoint(local, v), center)))
	}
	return r + p.Radius
}
func motionShapeRadius(s *shape, center Vec3) float32 {
	a := computeShapeAABB(s, TransformIdentity)
	return Length(Max(Abs(Sub(a.LowerBound, center)), Abs(Sub(a.UpperBound, center))))
}
func motionBounds(m motionPath, r, pad float32) AABB {
	d := Vec3{r + pad, r + pad, r + pad}
	return AABB{Sub(Min(m.start.P, m.end.P), d), Add(Max(m.start.P, m.end.P), d)}
}
func motionLocalCandidates(target, query motionPath, r, pad float32) AABB {
	a, z := Sub(query.start.P, target.start.P), Sub(query.end.P, target.end.P)
	if target.angle == 0 {
		a = InvRotateVector(target.start.Q, a)
		z = InvRotateVector(target.start.Q, z)
		d := Vec3{r + pad, r + pad, r + pad}
		return AABB{Add(target.center, Sub(Min(a, z), d)), Add(target.center, Add(Max(a, z), d))}
	}
	r += MaxFloat(Length(a), Length(z)) + pad
	d := Vec3{r, r, r}
	return AABB{Sub(target.center, d), Add(target.center, d)}
}

func motionCastLeaves(la, lb queryLeaf, pa, pb ShapeProxy, ma, mb motionPath, ra, rb, tolerance float32, backFaces bool) (MotionCastResult, error) {
	if !motionBounds(ma, ra, tolerance).Overlaps(motionBounds(mb, rb, tolerance)) {
		return MotionCastResult{}, nil
	}
	velocity := Sub(Sub(mb.end.P, mb.start.P), Sub(ma.end.P, ma.start.P))
	axisA, axisB := RotateVector(ma.start.Q, ma.axis), RotateVector(mb.start.Q, mb.axis)
	fraction := float32(0)
	finalProbes := 0
	for iteration := 0; iteration < 256; iteration++ {
		xa, xb := MulTransforms(ma.at(fraction), la.transform), MulTransforms(mb.at(fraction), lb.transform)
		rel := InvMulTransforms(xa, xb)
		cache := SimplexCache{}
		d := ShapeDistance(&DistanceInput{ProxyA: pa, ProxyB: pb, Transform: rel}, &cache, nil)
		if !IsValidFloat(d.Distance) || !IsValidVec3(d.Normal) || d.Iterations >= maxGJKIterations {
			return MotionCastResult{}, ErrMotionCastConvergence
		}
		gap := d.Distance - pa.Radius - pb.Radius
		normal := RotateVector(xa.Q, d.Normal)
		if gap <= tolerance {
			if !backFaces {
				if la.triangleIndex >= 0 && Dot(RotateVector(xa.Q, Normalize(Cross(Sub(la.triangle[1], la.triangle[0]), Sub(la.triangle[2], la.triangle[0])))), normal) < 0 {
					return MotionCastResult{}, nil
				}
				if lb.triangleIndex >= 0 && Dot(RotateVector(xb.Q, Normalize(Cross(Sub(lb.triangle[1], lb.triangle[0]), Sub(lb.triangle[2], lb.triangle[0])))), normal) > 0 {
					return MotionCastResult{}, nil
				}
			}
			r := MotionCastResult{Hit: true, Fraction: fraction, Separation: gap, Normal: normal, PointA: ToPos(TransformPoint(xa, MulAdd(d.PointA, pa.Radius, d.Normal))), PointB: ToPos(TransformPoint(xa, MulSub(d.PointB, pb.Radius, d.Normal))), FeatureA: QueryFeature{la.child, la.triangleIndex, 0}, FeatureB: QueryFeature{lb.child, lb.triangleIndex, 0}, Iterations: iteration + 1}
			a, b := la, lb
			a.transform = xa
			b.transform = xb
			if !collideQueryLeaves(a, b, backFaces, func(point, n Vec3, separation float32, fa, fb uint32) {
				if separation > 0 {
					return
				}
				r.Touching = r.Touching || separation == 0
				if separation < 0 && -separation >= r.Depth {
					r.Penetrating = true
					r.Depth = -separation
					r.Normal = n
					r.PointA = ToPos(MulAdd(point, 0.5*r.Depth, n))
					r.PointB = ToPos(MulSub(point, 0.5*r.Depth, n))
					r.FeatureA.FeatureID = fa
					r.FeatureB.FeatureID = fb
				}
			}) {
				return MotionCastResult{}, ErrMotionCastUnsupported
			}
			r.Touching = r.Touching && !r.Penetrating
			r.Initial = fraction == 0 && (r.Touching || r.Penetrating)
			return r, nil
		}
		if finalProbes > 0 {
			a, z := la, lb
			a.transform, z.transform = xa, xb
			contact, valid := motionManifoldResult(a, z, fraction, backFaces)
			if !valid {
				return MotionCastResult{}, ErrMotionCastUnsupported
			}
			if contact.Hit && contact.Separation <= tolerance {
				return contact, nil
			}
			return MotionCastResult{}, fmt.Errorf("%w: fraction probe %g gap=%g tolerance=%g", ErrMotionCastConvergence, fraction, gap, tolerance)
		}
		// A separating support plane gives a lower bound even if GJK stopped early.
		lower := motionSupportGap(pa, pb, rel, d.Normal)
		if lower <= 0 {
			// Near face contact, GJK's witness normal can lose enough precision
			// that it is no longer a separating plane. The existing manifold's
			// geometric normal supplies an independent candidate, never an AABB.
			a, z := la, lb
			a.transform, z.transform = xa, xb
			contact, valid := motionManifoldResult(a, z, fraction, backFaces)
			if !valid {
				return MotionCastResult{}, ErrMotionCastUnsupported
			}
			if contact.Hit {
				if contact.Separation <= tolerance {
					return contact, nil
				}
				normal = contact.Normal
				lower = motionSupportGap(pa, pb, rel, InvRotateVector(xa.Q, normal))
			}
			if lower <= 0 {
				return MotionCastResult{}, fmt.Errorf("%w: separating gap=%g lower=%g fraction=%g", ErrMotionCastConvergence, gap, lower, fraction)
			}
		}
		angular := (float32(ma.angle*(ra-pa.Radius)*Length(Cross(axisA, normal))) + float32(mb.angle*(rb-pb.Radius)*Length(Cross(axisB, normal)))) * (1 + 8*fltEpsilon)
		if ma.start == mb.start && ma.end == mb.end {
			angular = 0
		}
		rate := MaxFloat(0, -Dot(velocity, normal)) + angular
		if rate == 0 {
			return MotionCastResult{}, nil
		}
		step := (lower - tolerance) / rate
		if fraction+step > 1 {
			return MotionCastResult{}, nil
		}
		// Leave a small fraction of the geometric bound to absorb float32 roundoff.
		next := mulAdd(fraction, 0.95, step)
		if next <= fraction {
			// The boundary may fall between adjacent representable fractions.
			// A fraction ULP may not change the transformed geometry at all.
			// Probe by at most a quarter of the distance tolerance; an
			// unresolved probe must fail rather than clear the trajectory.
			probe := MinFloat(.25*tolerance, 8*fltEpsilon*MaxFloat(1, ra+rb+Length(rel.P))) / rate
			next = MinFloat(1, MaxFloat(math.Nextafter32(fraction, 1), fraction+probe))
			if next <= fraction || next > 1 {
				return MotionCastResult{}, ErrMotionCastConvergence
			}
			finalProbes = 1
		}
		fraction = next
	}
	return MotionCastResult{}, ErrMotionCastConvergence
}

func motionSupportGap(a, z ShapeProxy, transform Transform, normal Vec3) float32 {
	maxA, minB := -fltMax, fltMax
	for _, v := range a.Points[:a.Count] {
		maxA = MaxFloat(maxA, Dot(normal, v))
	}
	for _, v := range z.Points[:z.Count] {
		minB = MinFloat(minB, Dot(normal, TransformPoint(transform, v)))
	}
	return minB - maxA - a.Radius - z.Radius
}

func motionManifoldResult(a, z queryLeaf, fraction float32, backFaces bool) (MotionCastResult, bool) {
	var result MotionCastResult
	valid := collideQueryLeaves(a, z, backFaces, func(point, normal Vec3, separation float32, fa, fb uint32) {
		if result.Hit && separation >= result.Separation {
			return
		}
		result = MotionCastResult{Hit: true, Fraction: fraction, Separation: separation, Normal: normal, Depth: MaxFloat(0, -separation), Touching: separation == 0, Penetrating: separation < 0,
			Initial: fraction == 0 && separation <= 0, PointA: ToPos(MulSub(point, .5*separation, normal)), PointB: ToPos(MulAdd(point, .5*separation, normal)),
			FeatureA: QueryFeature{a.child, a.triangleIndex, fa}, FeatureB: QueryFeature{z.child, z.triangleIndex, fb}}
	})
	return result, valid
}
