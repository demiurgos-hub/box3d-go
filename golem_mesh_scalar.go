// Scalar specialization of the transform/cull operations in mesh_contact.c.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors; Golem scalar specialization.

package box3d

// meshScalarSurvivors preserves the wide kernel's multiply/add parentheses,
// min/max nesting, cache invalidation and survivor ordering. It changes no
// contact threshold, candidate triangle set, feature or manifold policy.
func meshScalarSurvivors(matrix Matrix3, translation Vec3, lanes *[3][4]Vec3, laneCount int,
	cull bool, bounds AABB, margin float32, caches []triangleCache, baseIndex int,
	indices []int, vertices []Vec3, count int) int {
	for lane := 0; lane < laneCount; lane++ {
		a := Add(MulMV(matrix, lanes[0][lane]), translation)
		b := Add(MulMV(matrix, lanes[1][lane]), translation)
		c := Add(MulMV(matrix, lanes[2][lane]), translation)
		if cull {
			lower := Vec3{MinFloat(a.X, MinFloat(b.X, c.X)) - margin, MinFloat(a.Y, MinFloat(b.Y, c.Y)) - margin, MinFloat(a.Z, MinFloat(b.Z, c.Z)) - margin}
			upper := Vec3{MaxFloat(a.X, MaxFloat(b.X, c.X)) + margin, MaxFloat(a.Y, MaxFloat(b.Y, c.Y)) + margin, MaxFloat(a.Z, MaxFloat(b.Z, c.Z)) + margin}
			if upper.X < bounds.LowerBound.X || lower.X > bounds.UpperBound.X || upper.Y < bounds.LowerBound.Y || lower.Y > bounds.UpperBound.Y || upper.Z < bounds.LowerBound.Z || lower.Z > bounds.UpperBound.Z {
				caches[baseIndex+lane].cache.satCache = SATCache{}
				continue
			}
		}
		vertices[3*count], vertices[3*count+1], vertices[3*count+2] = a, b, c
		indices[count] = baseIndex + lane
		count++
	}
	return count
}
