// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// This fallback finds the first tolerance-band contact, not merely a geometric
// overlap. A geometrically separated trajectory may still enter that band.
func motionRotatingBandCast(la, lb queryLeaf, pa, pb ShapeProxy, ma, mb motionPath, ra, rb, tolerance float32, backFaces bool) (MotionCastResult, error) {
	rate := Length(Sub(Sub(mb.end.P, mb.start.P), Sub(ma.end.P, ma.start.P))) + float32(ma.angle*(ra-pa.Radius)) + float32(mb.angle*(rb-pb.Radius))
	rate *= 1 + 16*fltEpsilon
	budget := 16384
	uncertainBand := false
	sample := func(f float32) (MotionCastResult, error) {
		a, z := ma.at(f), mb.at(f)
		r, err := motionAdvanceLeaves(la, lb, pa, pb, newMotionPath(a, a), newMotionPath(z, z), ra, rb, tolerance, backFaces, true)
		r.Fraction, r.Initial = f, f == 0 && (r.Touching || r.Penetrating)
		return r, err
	}
	var visit func(float32, float32, int) (MotionCastResult, error)
	visit = func(lo, hi float32, depth int) (MotionCastResult, error) {
		if budget == 0 {
			return MotionCastResult{}, ErrMotionIntervalLimit
		}
		budget--
		first, err := sample(lo)
		if err != nil || first.Hit {
			return first, err
		}
		mid := .5 * (lo + hi)
		xa, xb := MulTransforms(ma.at(mid), la.transform), MulTransforms(mb.at(mid), lb.transform)
		rel := InvMulTransforms(xa, xb)
		cache := SimplexCache{}
		d := ShapeDistance(&DistanceInput{ProxyA: pa, ProxyB: pb, Transform: rel}, &cache, nil)
		if !IsValidFloat(d.Distance) || !IsValidVec3(d.Normal) || d.Iterations >= maxGJKIterations {
			return MotionCastResult{}, ErrMotionCastConvergence
		}
		reach := float32(.5 * (hi - lo) * rate)
		lower := motionSupportGap(pa, pb, rel, d.Normal)
		if lower > tolerance+reach {
			return MotionCastResult{}, nil
		}
		if 2*reach <= .25*tolerance {
			for _, f := range []float32{mid, hi} {
				hit, err := sample(f)
				if err != nil || hit.Hit {
					return hit, err
				}
			}
			if lower > reach {
				// Float32 may not resolve the exact edge of the positive band.
				// This span is geometrically clear, so continue toward a valid
				// band witness, but never return a clear result on this evidence.
				uncertainBand = true
				return MotionCastResult{}, nil
			}
		}
		if depth == 24 || mid <= lo || mid >= hi {
			return MotionCastResult{}, ErrMotionCastConvergence
		}
		left, err := visit(lo, mid, depth+1)
		if err != nil || left.Hit {
			return left, err
		}
		return visit(mid, hi, depth+1)
	}
	result, err := visit(0, 1, 0)
	result.Iterations += 16384 - budget
	if err == nil && !result.Hit && uncertainBand {
		return result, ErrMotionCastConvergence
	}
	return result, err
}
