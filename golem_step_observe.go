// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"errors"
	"slices"
)

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
// SetTOIContinuation adds owned local impact samples and delegates swept sensor
// crossings to the caller's trajectory replay; native discrete sensors remain active.
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
	previousObservation := w.toiContinuation.observing
	w.toiContinuation.observing = observe != nil
	defer func() { w.toiContinuation.observing = previousObservation }()
	previous := motionIterationSettings[w.worldId]
	motionIterationSettings[w.worldId] = motionSolverIterations{options.VelocityIterations, options.RelaxIterations}
	defer func() { motionIterationSettings[w.worldId] = previous }()

	sample := MotionStepSample{Delta: dt, ImpulseDuration: dt, EndFraction: 1}
	if observe != nil {
		count := 0
		for i := range w.solverSets {
			if i != staticSet && i != disabledSet {
				count += len(w.solverSets[i].bodySims)
			}
		}
		sample.Bodies = make([]MotionBodySample, 0, count)
		for setID := range w.solverSets {
			if setID == staticSet || setID == disabledSet {
				continue
			}
			for i := range w.solverSets[setID].bodySims {
				sim := &w.solverSets[setID].bodySims[i]
				body := &w.bodies[sim.bodyId]
				sample.Bodies = append(sample.Bodies, MotionBodySample{Body: BodyID{sim.bodyId + 1, w.worldId, body.generation}, Start: sim.transform})
			}
		}
		slices.SortFunc(sample.Bodies, func(a, b MotionBodySample) int { return int(a.Body.Index1) - int(b.Body.Index1) })
	}

	if !w.speculativeCCD.enabled {
		motionInvalidateMovingMeshCaches(w)
	}
	id.Step(dt, 1)
	sample.SpeculativeCCD = id.GetSpeculativeCCDStats()
	sample.TOIContinuation = id.GetTOIContinuationStats()
	stepError := errors.Join(id.SpeculativeCCDError(), id.TOIContinuationError())
	if observe == nil {
		return stepError
	}
	if len(w.toiContinuation.samples) != 0 {
		var events MotionStepSample
		var scratch observedContactScratch
		motionCopyEvents(id, &events, &scratch)
		samples := w.toiContinuation.samples
		samples[0].SpeculativeCCD = sample.SpeculativeCCD
		samples[0].TOIContinuation = sample.TOIContinuation
		last := &samples[len(samples)-1]
		last.ContactEvents, last.SensorEvents, last.JointEvents = events.ContactEvents, events.SensorEvents, events.JointEvents
		for _, owned := range samples {
			if err := motionObserve(w, owned, observe); err != nil {
				return errors.Join(stepError, err)
			}
		}
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
