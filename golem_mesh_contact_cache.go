// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

func motionInvalidateMovingMeshCaches(w *world) {
	for i := range w.contacts {
		c := &w.contacts[i]
		if c.contactId == NullIndex || c.flags&simMeshContact == 0 || w.bodies[c.edges[0].bodyId].typ == StaticBody {
			continue
		}
		// Upstream stores the triangle query region in WORLD coordinates and
		// otherwise reuses it while only B stays inside. A moving mesh changes
		// which triangles occupy that region even if a passive B did not move.
		c.meshContact.queryBounds = AABB{Vec3{fltMax, fltMax, fltMax}, Vec3{-fltMax, -fltMax, -fltMax}}
		c.flags &^= relativeTransformValid
	}
}
