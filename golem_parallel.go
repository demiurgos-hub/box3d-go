// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// This runtime policy deliberately stays outside the upstream serialized layout.
// Golem's portable checkpoint recreates it when constructing the restored world.
var consistentParallelWarmStart [MaxWorlds]bool

// SetConsistentParallelWarmStart enables matching hemisphere selection in parallel
// joint preparation, warm starting, solving and force reporting. It prevents free
// twist through pi from reversing cached angular impulses. Upstream defaults stay
// unchanged for existing recordings and C-reference fixtures. Set this before
// creating joints. Raw native recordings do not capture this runtime policy.
func (id WorldID) SetConsistentParallelWarmStart(enabled bool) {
	w := getUnlockedWorldFromId(id)
	if w != nil {
		consistentParallelWarmStart[w.worldId] = enabled
	}
}
