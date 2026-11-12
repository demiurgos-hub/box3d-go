// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import "fmt"

func motionSegment(path motionPath, lo, hi float32) motionPath {
	a, z := path.at(lo), path.at(hi)
	return motionPath{start: Transform{Lerp(path.start.P, path.end.P, lo), a.Q}, end: Transform{Lerp(path.start.P, path.end.P, hi), z.Q}, center: path.center, axis: path.axis, angle: path.angle * (hi - lo)}
}

func motionCertifyLeafPath(a, z queryLeaf, originalA, originalB motionPath, lo, hi, tolerance, limit float32, budget *int, depth int) error {
	if budget == nil || *budget == 0 {
		return ErrMotionStepLimit
	}
	*budget -= 1
	ma, mb := motionSegment(originalA, lo, hi), motionSegment(originalB, lo, hi)
	var pointsA, pointsB [3]Vec3
	pa, ok := motionLeafProxy(a, &pointsA)
	if !ok {
		return ErrMotionCastUnsupported
	}
	pb, ok := motionLeafProxy(z, &pointsB)
	if !ok {
		return ErrMotionCastUnsupported
	}
	ra, rb := motionProxyRadius(pa, a.transform, ma.center), motionProxyRadius(pb, z.transform, mb.center)
	r, err := motionCastLeaves(a, z, pa, pb, ma, mb, ra, rb, tolerance, true, false)
	if err != nil || !r.Hit {
		return err
	}
	if r.Initial && r.Depth > limit+MotionPenetrationRoundoff {
		return fmt.Errorf("%w: refined fraction=%g depth=%g bound=%g", ErrMotionStepPenetration, lo, r.Depth, limit)
	}
	velocity := Sub(Sub(mb.end.P, mb.start.P), Sub(ma.end.P, ma.start.P))
	angular := float32(ma.angle*(ra-pa.Radius)*Length(Cross(RotateVector(ma.start.Q, ma.axis), r.Normal))) + float32(mb.angle*(rb-pb.Radius)*Length(Cross(RotateVector(mb.start.Q, mb.axis), r.Normal)))
	closing := MaxFloat(0, -Dot(velocity, r.Normal)) + angular
	if mulAdd(r.Depth, 1-r.Fraction, closing) <= limit+MotionPenetrationRoundoff {
		return nil
	}
	mid := .5 * (lo + hi)
	if depth == 20 || mid <= lo || mid >= hi {
		return ErrMotionStepUnsafePath
	}
	if err = motionCertifyLeafPath(a, z, originalA, originalB, lo, mid, tolerance, limit, budget, depth+1); err != nil {
		return err
	}
	return motionCertifyLeafPath(a, z, originalA, originalB, mid, hi, tolerance, limit, budget, depth+1)
}
