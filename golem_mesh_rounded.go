// Golem broadphase specialization of native mesh contact candidate culling.
// SPDX-License-Identifier: MIT
package box3d

import "math"

// Bounds use the same speculative reach as the native rounded-shape kernels.
// Round outward at each addition/subtraction, then retain the existing float32
// broadphase roundoff allowance. No narrowphase/contact threshold is changed.
func meshRoundedContactBounds(s *shape) (AABB, bool) {
	var lower, upper Vec3
	var radius float32
	switch s.typ {
	case SphereShape:
		lower, upper, radius = s.sphere.Center, s.sphere.Center, s.sphere.Radius
	case CapsuleShape:
		lower, upper, radius = Min(s.capsule.Center1, s.capsule.Center2), Max(s.capsule.Center1, s.capsule.Center2), s.capsule.Radius
	default:
		return AABB{}, false
	}
	reach := math.Nextafter32(radius+SpeculativeDistance(), math.MaxFloat32)
	bounds := AABB{
		Vec3{math.Nextafter32(lower.X-reach, -math.MaxFloat32), math.Nextafter32(lower.Y-reach, -math.MaxFloat32), math.Nextafter32(lower.Z-reach, -math.MaxFloat32)},
		Vec3{math.Nextafter32(upper.X+reach, math.MaxFloat32), math.Nextafter32(upper.Y+reach, math.MaxFloat32), math.Nextafter32(upper.Z+reach, math.MaxFloat32)},
	}
	return continuousBoundsRoundoff(bounds), true
}
