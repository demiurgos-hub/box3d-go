// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import "errors"

// ErrObservedStepInvalid reports invalid observed-step settings or world state.
var ErrObservedStepInvalid = errors.New("box3d: invalid observed step")

// ObservedStepOptions selects constraint passes without changing elapsed time.
type ObservedStepOptions struct {
	VelocityIterations, RelaxIterations int
}

// DefaultObservedStepOptions returns the native solver's constraint pass counts.
func DefaultObservedStepOptions() ObservedStepOptions {
	return ObservedStepOptions{VelocityIterations: solverIterations, RelaxIterations: solverRelaxIterations}
}

// StepObserved performs exactly one native Step(dt, 1), then reports owned body,
// contact and event samples. Observation adds no candidate search, time
// subdivision or penetration policy of its own. Enabled native CCD policies,
// including SetSpeculativeCCD, may cast candidate pairs during the native step.
//
// The observer runs with the world locked and must not mutate it. Its error is
// returned after simulation has advanced; it does not roll back bodies. Callers
// with transactional state must reject their whole frame on failure. Recorder
// worlds are rejected because the observation settings are not in that format.
func (id WorldID) StepObserved(dt float32, options ObservedStepOptions, observe func(MotionStepSample) error) error {
	w := getUnlockedWorldFromId(id)
	if w == nil || w.recording != nil || !IsValidFloat(dt) || dt <= 0 || options.VelocityIterations < 1 || options.VelocityIterations > 16 || options.RelaxIterations < 1 || options.RelaxIterations > 16 {
		return ErrObservedStepInvalid
	}
	previous := motionIterationSettings[w.worldId]
	motionIterationSettings[w.worldId] = motionSolverIterations{options.VelocityIterations, options.RelaxIterations}
	defer func() { motionIterationSettings[w.worldId] = previous }()

	sample := MotionStepSample{Delta: dt, EndFraction: 1}
	if observe != nil {
		for i := range w.bodies {
			body := &w.bodies[i]
			if body.id == NullIndex || body.setIndex == disabledSet || body.typ == StaticBody {
				continue
			}
			bodyID := BodyID{body.id + 1, w.worldId, body.generation}
			sample.Bodies = append(sample.Bodies, MotionBodySample{Body: bodyID, Start: bodyID.GetTransform()})
		}
	}
	if !w.speculativeCCD.enabled {
		motionInvalidateMovingMeshCaches(w)
	}
	id.Step(dt, 1)
	sample.SpeculativeCCD = id.GetSpeculativeCCDStats()
	stepError := id.SpeculativeCCDError()
	if observe == nil {
		return stepError
	}
	for i := range sample.Bodies {
		body := &sample.Bodies[i]
		body.End = body.Body.GetTransform()
		body.LinearVelocity = body.Body.GetLinearVelocity()
		body.AngularVelocity = body.Body.GetAngularVelocity()
	}
	var scratch observedContactScratch
	motionCopyEvents(id, &sample, &scratch)
	return errors.Join(stepError, motionObserve(w, sample, observe))
}
