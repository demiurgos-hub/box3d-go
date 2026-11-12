// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// Local indices address context.states. Static endpoints return NullIndex;
// newly awakened local participants must have a real index and finite mass.
func jointBodyForContext(context *stepContext, bodyID int32) (*bodySim, int32) {
	if context.localJointBodies != nil {
		sim, index := context.localJointBodies(bodyID)
		assert(sim != nil && (index == NullIndex || index >= 0 && int(index) < len(context.states)), "valid local joint body")
		return sim, index
	}
	body := arrayGet(context.world.bodies, int(bodyID))
	sim := getBodySim(context.world, body)
	index := int32(NullIndex)
	if body.setIndex == awakeSet {
		index = body.localIndex
	}
	return sim, index
}

// Joint drives retain the original interval's impulse caps and spring equation.
// Continuation changes geometry and remaining-time limit/constraint bias only;
// it must not grant a fresh motor/spring budget at each impact.
func jointIntervalH(context *stepContext) float32 {
	if context.jointBudgetH > 0 {
		return context.jointBudgetH
	}
	return context.h
}
