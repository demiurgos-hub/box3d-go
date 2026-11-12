// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"errors"
	"sort"
)

// ErrMotionIntervalLimit reports exhausted interval work, never a clear path.
var ErrMotionIntervalLimit = errors.New("box3d: motion interval capacity exceeded")

// MotionInterval encloses a connected contact interval within distance tolerance. Enter and
// Exit retain exact leaf identities; they need not belong to the same child.
// Touching at one instant has equal fractions. Depth is never a union MTD.
type MotionInterval struct {
	Enter, Exit MotionCastResult
}

// MotionIntervalResult distinguishes a complete interval set from output
// truncation. Any error also sets Incomplete; partial intervals are not clearance.
type MotionIntervalResult struct {
	Count, Total, LeafPairs, Refinements int
	Incomplete                           bool
}

// MotionIntervals enumerates and unions every convex-leaf contact interval for
// relative translation/rotation, including compound/mesh exit and re-entry.
// Results use BODY trajectories, two-sided triangle surfaces when requested,
// and stable ascending entry fractions. Geometry is not mutated or filtered.
// Rotational boundary brackets conservatively retain geometric contacts within
// the supplied distance tolerance. Work is bounded at 16,384 retained leaf
// intervals/refinements independently of dst capacity; exhaustion is an error.
func (id ShapeID) MotionIntervals(other ShapeID, aStart, aEnd, bStart, bEnd WorldTransform, tolerance float32, backFaces bool, dst []MotionInterval) (MotionIntervalResult, error) {
	failed := func(err error) (MotionIntervalResult, error) {
		return MotionIntervalResult{Incomplete: true}, err
	}
	if id.World0 != other.World0 || !IsValidFloat(tolerance) || tolerance <= 0 {
		return failed(ErrMotionCastInvalid)
	}
	for _, x := range []WorldTransform{aStart, aEnd, bStart, bEnd} {
		if !IsValidPosition(x.P) || !IsValidQuat(x.Q) || !IsNormalizedQuat(x.Q) {
			return failed(ErrMotionCastInvalid)
		}
	}
	a, _, ok := queryShape(id, aStart.P)
	if !ok {
		return failed(ErrMotionCastInvalid)
	}
	z, _, ok := queryShape(other, aStart.P)
	if !ok || worlds[id.World0].recording != nil {
		return failed(ErrMotionCastInvalid)
	}
	base := aStart.P
	ma := newMotionPath(ToRelativeTransform(aStart, base), ToRelativeTransform(aEnd, base))
	mb := newMotionPath(ToRelativeTransform(bStart, base), ToRelativeTransform(bEnd, base))
	var intervals []MotionInterval
	stats := MotionIntervalResult{}
	budget := 16384
	_, err := motionVisitShapeMotion(a, z, ma, mb, base, tolerance, backFaces, true, func(_ MotionCastResult, la, lb queryLeaf) error {
		stats.LeafPairs++
		pa, _ := motionLeafProxy(la)
		pb, _ := motionLeafProxy(lb)
		ca, cb := motionIntervalPath(ma, la, pa), motionIntervalPath(mb, lb, pb)
		var leafIntervals []MotionInterval
		if ca.angle != 0 || cb.angle != 0 {
			if !backFaces && (la.triangleIndex >= 0 || lb.triangleIndex >= 0) {
				return ErrMotionCastUnsupported
			}
			var err error
			leafIntervals, err = motionRotatingIntervals(la, lb, pa, pb, ca, cb, tolerance, &budget)
			if err != nil {
				return err
			}
		} else {
			ra, rb := motionProxyRadius(pa, la.transform, ca.center), motionProxyRadius(pb, lb.transform, cb.center)
			entry, err := motionCastLeaves(la, lb, pa, pb, ca, cb, ra, rb, tolerance, backFaces, true)
			if err != nil || !entry.Hit {
				return err
			}
			reverseA, reverseB := newMotionPath(ca.end, ca.start), newMotionPath(cb.end, cb.start)
			exit, err := motionCastLeaves(la, lb, pa, pb, reverseA, reverseB, ra, rb, tolerance, backFaces, true)
			if err != nil {
				return err
			}
			if !exit.Hit {
				return ErrMotionCastConvergence
			}
			exit.Fraction = 1 - exit.Fraction
			if exit.Fraction < entry.Fraction {
				return ErrMotionCastConvergence
			}
			leafIntervals = append(leafIntervals, MotionInterval{entry, exit})
		}
		for _, interval := range leafIntervals {
			for _, hit := range []*MotionCastResult{&interval.Enter, &interval.Exit} {
				hit.PointA = OffsetPos(base, ToVec3(hit.PointA))
				hit.PointB = OffsetPos(base, ToVec3(hit.PointB))
				hit.MaterialA = getShapeUserMaterialId(a, int(hit.FeatureA.ChildIndex), int(hit.FeatureA.TriangleIndex))
				hit.MaterialB = getShapeUserMaterialId(z, int(hit.FeatureB.ChildIndex), int(hit.FeatureB.TriangleIndex))
			}
			if len(intervals) == 16384 {
				return ErrMotionIntervalLimit
			}
			intervals = append(intervals, interval)
		}
		return nil
	})
	if err != nil {
		stats.Incomplete = true
		stats.Refinements = 16384 - budget
		return stats, err
	}
	sort.Slice(intervals, func(i, j int) bool {
		a, b := intervals[i].Enter, intervals[j].Enter
		return a.Fraction < b.Fraction || a.Fraction == b.Fraction && queryPairFeatureLess(a.FeatureA, a.FeatureB, b.FeatureA, b.FeatureB)
	})
	var merged []MotionInterval
	for _, interval := range intervals {
		if len(merged) == 0 || interval.Enter.Fraction > merged[len(merged)-1].Exit.Fraction {
			merged = append(merged, interval)
		} else if interval.Exit.Fraction > merged[len(merged)-1].Exit.Fraction {
			merged[len(merged)-1].Exit = interval.Exit
		}
	}
	stats.Total = len(merged)
	stats.Refinements = 16384 - budget
	stats.Count = copy(dst, merged)
	stats.Incomplete = stats.Count < stats.Total
	return stats, nil
}

func motionIntervalPath(path motionPath, leaf queryLeaf, proxy ShapeProxy) motionPath {
	if leaf.kind == SphereShape && proxy.Count == 1 && TransformPoint(leaf.transform, proxy.Points[0]) == path.center {
		path.end.Q, path.angle = path.start.Q, 0
	}
	return path
}
