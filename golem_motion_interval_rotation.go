// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// motionRotatingIntervals encloses every geometric contact in intervals whose
// uncertain boundaries are within tolerance. It certifies separated spans with
// support planes and intersecting spans with a ball inside the rounded Minkowski
// difference. Ambiguity consumes bounded work; it never becomes a clear span.
func motionRotatingIntervals(la, lb queryLeaf, pa, pb ShapeProxy, ma, mb motionPath, tolerance float32, budget *int) ([]MotionInterval, error) {
	ra, rb := motionProxyRadius(pa, la.transform, ma.center), motionProxyRadius(pb, lb.transform, mb.center)
	rate := Length(Sub(Sub(mb.end.P, mb.start.P), Sub(ma.end.P, ma.start.P))) + float32(ma.angle*(ra-pa.Radius)) + float32(mb.angle*(rb-pb.Radius))
	rate *= 1 + 16*fltEpsilon
	var nearBand func(float32, float32, int) bool
	nearBand = func(lo, hi float32, depth int) bool {
		if *budget == 0 {
			return false
		}
		*budget--
		mid := .5 * (lo + hi)
		xa, xb := MulTransforms(ma.at(mid), la.transform), MulTransforms(mb.at(mid), lb.transform)
		cache := SimplexCache{}
		d := ShapeDistance(&DistanceInput{ProxyA: pa, ProxyB: pb, Transform: InvMulTransforms(xa, xb)}, &cache, nil)
		if !IsValidFloat(d.Distance) || d.Iterations >= maxGJKIterations {
			return false
		}
		gap := d.Distance - pa.Radius - pb.Radius
		reach := float32(.5 * (hi - lo) * rate)
		if gap+reach <= tolerance {
			return true
		}
		if gap > tolerance || depth == 24 || mid <= lo || mid >= hi {
			return false
		}
		return nearBand(lo, mid, depth+1) && nearBand(mid, hi, depth+1)
	}
	var spans [][2]float32
	appendSpan := func(lo, hi float32) {
		join := false
		if len(spans) > 0 {
			previous := spans[len(spans)-1][1]
			join = previous == lo
			if !join {
				// Near a tangent, float32 distance can alternate across zero.
				// Join only when the entire intervening span is certified inside
				// the requested boundary band, never by a larger time epsilon.
				join = nearBand(previous, lo, 0)
			}
		}
		if join {
			spans[len(spans)-1][1] = hi
		} else {
			spans = append(spans, [2]float32{lo, hi})
		}
	}
	var visit func(float32, float32, int) error
	visit = func(lo, hi float32, depth int) error {
		if *budget == 0 {
			return ErrMotionIntervalLimit
		}
		*budget--
		mid := .5 * (lo + hi)
		xa, xb := MulTransforms(ma.at(mid), la.transform), MulTransforms(mb.at(mid), lb.transform)
		rel := InvMulTransforms(xa, xb)
		cache := SimplexCache{}
		d := ShapeDistance(&DistanceInput{ProxyA: pa, ProxyB: pb, Transform: rel}, &cache, nil)
		if !IsValidFloat(d.Distance) || d.Iterations >= maxGJKIterations {
			return ErrMotionCastConvergence
		}
		gap := d.Distance - pa.Radius - pb.Radius
		reach := float32(.5 * (hi - lo) * rate)
		if motionSupportGap(pa, pb, rel, d.Normal) > reach {
			return nil
		}
		inside := -gap
		if d.Distance == 0 && inside < reach {
			inside += motionMinkowskiInterior(pa, pb, rel)
		}
		if inside >= reach || reach <= .25*tolerance && gap+reach <= tolerance {
			appendSpan(lo, hi)
			return nil
		}
		if depth == 24 || mid <= lo || mid >= hi {
			return ErrMotionCastConvergence
		}
		if err := visit(lo, mid, depth+1); err != nil {
			return err
		}
		return visit(mid, hi, depth+1)
	}
	if err := visit(0, 1, 0); err != nil {
		return nil, err
	}
	if *budget == 0 {
		return nil, ErrMotionIntervalLimit
	}
	var out []MotionInterval
	sample := func(f float32) (MotionCastResult, error) {
		a, z := ma.at(f), mb.at(f)
		r, err := motionCastLeaves(la, lb, pa, pb, newMotionPath(a, a), newMotionPath(z, z), ra, rb, tolerance, true, true)
		if err != nil {
			return r, err
		}
		if !r.Hit {
			return r, ErrMotionCastConvergence
		}
		r.Fraction, r.Initial = f, f == 0 && (r.Touching || r.Penetrating)
		return r, nil
	}
	for _, span := range spans {
		enter, err := sample(span[0])
		if err != nil {
			return nil, err
		}
		exit, err := sample(span[1])
		if err != nil {
			return nil, err
		}
		out = append(out, MotionInterval{enter, exit})
	}
	return out, nil
}

// The convex hull of actual support points lies inside the Minkowski difference.
// Its origin-centered inscribed ball is therefore a conservative overlap margin,
// unlike the deepest point of a contact manifold. Failure yields no certificate.
func motionMinkowskiInterior(a, z ShapeProxy, relative Transform) float32 {
	points := make([]Vec3, 0, 26)
	scale := float32(1)
	for x := -1; x <= 1; x++ {
		for y := -1; y <= 1; y++ {
			for zAxis := -1; zAxis <= 1; zAxis++ {
				if x == 0 && y == 0 && zAxis == 0 {
					continue
				}
				d := Vec3{float32(x), float32(y), float32(zAxis)}
				ia := getProxySupport(&a, Neg(d))
				iz := getProxySupport(&z, InvRotateVector(relative.Q, d))
				point := Sub(TransformPoint(relative, z.Points[iz]), a.Points[ia])
				points = append(points, point)
				scale = MaxFloat(scale, Length(point))
			}
		}
	}
	hull := CreateHull(points, len(points), len(points))
	if hull == nil {
		return 0
	}
	defer DestroyHull(hull)
	margin := float32(fltMax)
	for _, plane := range GetHullPlanes(hull) {
		margin = MinFloat(margin, plane.Offset)
	}
	return MaxFloat(0, margin-float32(16*fltEpsilon*scale))
}
