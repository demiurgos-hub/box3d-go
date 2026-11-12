// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"errors"
	"slices"
)

type toiImpulseWitness struct {
	manifolds        []Manifold
	centerA, centerB Vec3
}

func (e *toiEngine) ordinaryImpulses() map[toiFeature]float32 {
	out := map[toiFeature]float32{}
	e.impulseWitnesses = map[toiContactKey]toiImpulseWitness{}
	for i := range e.w.contacts {
		c := &e.w.contacts[i]
		if c.contactId == NullIndex || c.setIndex == disabledSet || c.manifoldCount == 0 {
			continue
		}
		key := toiKey(c)
		e.known[key] = true
		if !observedContactSolved(e.w, c) {
			continue
		}
		a, b := e.body(c.edges[0].bodyId), e.body(c.edges[1].bodyId)
		e.impulseWitnesses[key] = toiImpulseWitness{c.manifolds, ToVec3(a.initial.center), ToVec3(b.initial.center)}
		for _, m := range c.manifolds {
			for _, p := range m.Points[:m.PointCount] {
				out[toiFeature{key, p.FeatureId, p.TriangleIndex}] += p.TotalNormalImpulse
			}
		}
	}
	return out
}
func (e *toiEngine) sample(f float32, impulses map[toiFeature]float32) error {
	if f < e.lastSample {
		return errors.New("observed TOI sample time regression")
	}
	w := e.w
	if len(w.toiContinuation.samples) >= toiSampleLimit {
		return errors.New("TOI owned sample capacity")
	}
	bodyCount := 0
	for i := range w.solverSets {
		if i != staticSet && i != disabledSet {
			bodyCount += len(w.solverSets[i].bodySims)
		}
	}
	if e.ownedBodies+bodyCount > toiOwnedBodyLimit {
		return errors.New("TOI owned body sample capacity")
	}
	e.ownedBodies += bodyCount
	defer func() { e.impulseWitnesses = nil }()
	consumed := map[toiFeature]bool{}
	sample := MotionStepSample{Index: len(w.toiContinuation.samples), Delta: (f - e.lastSample) * e.ctx.dt, ImpulseDuration: e.ctx.dt, StartFraction: e.lastSample, EndFraction: f}
	ids := make([]int32, 0, len(e.bodies))
	for setID := range w.solverSets {
		if setID == staticSet || setID == disabledSet {
			continue
		}
		for _, sim := range w.solverSets[setID].bodySims {
			ids = append(ids, sim.bodyId)
		}
	}
	slices.Sort(ids)
	for _, id := range ids {
		body := &w.bodies[id]
		b := e.body(id)
		start := WorldTransform{P: b.initial.transform.P, Q: b.initial.transform.Q}
		if len(w.toiContinuation.samples) > 0 {
			previous := &w.toiContinuation.samples[len(w.toiContinuation.samples)-1]
			i, ok := slices.BinarySearchFunc(previous.Bodies, id, func(v MotionBodySample, id int32) int {
				if v.Body.Index1-1 < id {
					return -1
				}
				if v.Body.Index1-1 > id {
					return 1
				}
				return 0
			})
			if ok {
				start = previous.Bodies[i].End
			}
		}
		sample.Bodies = append(sample.Bodies, MotionBodySample{Body: BodyID{id + 1, w.worldId, body.generation}, Start: start, End: b.poseAt(f), LinearVelocity: b.state.linearVelocity, AngularVelocity: b.state.angularVelocity})
	}
	keys := make([]toiContactKey, 0, len(e.known))
	for key := range e.known {
		keys = append(keys, key)
	}
	slices.SortFunc(keys, toiCompareKey)
	for _, key := range keys {
		cs, err := e.build(key.toiPair, f, true)
		if err != nil {
			return err
		}
		for _, c := range cs {
			if c.childIndex != key.child {
				continue
			}
			owned, err := e.copyContact(key, c.manifolds)
			if err != nil {
				return err
			}
			for mi := range owned.Manifolds {
				m := &owned.Manifolds[mi]
				for pi := range m.Points[:m.PointCount] {
					m.Points[pi].NormalImpulse = 0
					m.Points[pi].TotalNormalImpulse = 0
				}
			}
			sample.Contacts = append(sample.Contacts, owned)
		}
		witness, ok := e.impulseWitnesses[key]
		if !ok {
			continue
		}
		a, b := e.body(w.shapes[key.a].bodyId), e.body(w.shapes[key.b].bodyId)
		offsetA, offsetB := Sub(witness.centerA, a.sweepAt(f).C1), Sub(witness.centerB, b.sweepAt(f).C1)
		var rows []Manifold
		for _, m := range witness.manifolds {
			out := m
			out.PointCount = 0
			for _, p := range m.Points[:m.PointCount] {
				token := toiFeature{key, p.FeatureId, p.TriangleIndex}
				value := impulses[token]
				if value == 0 || consumed[token] {
					continue
				}
				p.NormalImpulse = 0
				p.TotalNormalImpulse = value
				p.AnchorA = Add(p.AnchorA, offsetA)
				p.AnchorB = Add(p.AnchorB, offsetB)
				out.Points[out.PointCount] = p
				out.PointCount++
				consumed[token] = true
			}
			if out.PointCount > 0 {
				rows = append(rows, out)
			}
		}
		if len(rows) > 0 {
			owned, err := e.copyContact(key, rows)
			if err != nil {
				return err
			}
			sample.ImpulseContacts = append(sample.ImpulseContacts, owned)
		}
	}
	for token, value := range impulses {
		if value != 0 && !consumed[token] {
			return errors.New("TOI impulse witness was not retained")
		}
	}
	w.toiContinuation.samples = append(w.toiContinuation.samples, sample)
	e.lastSample = f
	return nil
}

func (e *toiEngine) copyContact(key toiContactKey, source []Manifold) (ContactData, error) {
	rows := 0
	for _, m := range source {
		rows += int(m.PointCount)
	}
	if e.ownedRows+rows > toiOwnedRowLimit {
		return ContactData{}, errors.New("TOI owned contact row capacity")
	}
	e.ownedRows += rows
	a, b := &e.w.shapes[key.a], &e.w.shapes[key.b]
	return ContactData{ShapeIdA: ShapeID{a.id + 1, e.w.worldId, a.generation}, ShapeIdB: ShapeID{b.id + 1, e.w.worldId, b.generation}, Manifolds: append([]Manifold(nil), source...), ManifoldCount: int32(len(source))}, nil
}
