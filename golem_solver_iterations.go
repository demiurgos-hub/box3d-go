// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

type motionSolverIterations struct{ velocity, relax int }

var motionIterationSettings [MaxWorlds]motionSolverIterations

func motionIterations(w *world) (int, int) {
	s := motionIterationSettings[w.worldId]
	if s.velocity == 0 {
		return solverIterations, solverRelaxIterations
	}
	return s.velocity, s.relax
}
