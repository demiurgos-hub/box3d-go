// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// MotionBodySample is an independently owned body-origin pose/velocity sample.
type MotionBodySample struct {
	Body                            BodyID
	Start, End                      WorldTransform
	LinearVelocity, AngularVelocity Vec3
}

// MotionStepSample owns one observed native step's paths, manifolds and events.
// Contact handles remain transient; later steps do not change the copied data.
// Observers run with the world locked and must not mutate it. Returning an error
// rejects observation after simulation has advanced. Bodies contains enabled
// nonstatic bodies only.
type MotionStepSample struct {
	SpeculativeCCD  SpeculativeCCDStats
	TOIContinuation TOIContinuationStats
	// ImpulseDuration is the full native interval. Copied manifold total impulses
	// are deltas reported once, even when Delta is zero at a local impact.
	ImpulseDuration                   float32
	Index                             int
	Delta, StartFraction, EndFraction float32
	Bodies                            []MotionBodySample
	Contacts                          []ContactData
	// ImpulseContacts contains solver-response witnesses only. These rows do not
	// establish geometric or sensor overlap. TotalNormalImpulse is reported once
	// and anchors are relative to this sample's body COM. Continuation Contacts
	// carry geometry with zero impulses; ordinary fallback Contacts retain their
	// existing combined geometry/impulse semantics.
	ImpulseContacts []ContactData
	ContactEvents   ContactEvents
	SensorEvents    SensorEvents
	JointEvents     JointEvents
}

// Scratch is private to contact collection; samples never reference its storage.
type observedContactScratch struct {
	contacts     []int32
	contactMarks []uint32
	contactEpoch uint32
}

func motionObserve(w *world, sample MotionStepSample, observe func(MotionStepSample) error) error {
	w.locked = true
	defer func() { w.locked = false }()
	return observe(sample)
}

func motionCopyEvents(id WorldID, sample *MotionStepSample, scratch *observedContactScratch) {
	c := id.GetContactEvents()
	c.BeginEvents = append([]ContactBeginTouchEvent(nil), c.BeginEvents...)
	c.EndEvents = append([]ContactEndTouchEvent(nil), c.EndEvents...)
	c.HitEvents = append([]ContactHitEvent(nil), c.HitEvents...)
	sample.ContactEvents = c
	s := id.GetSensorEvents()
	s.BeginEvents = append([]SensorBeginTouchEvent(nil), s.BeginEvents...)
	s.EndEvents = append([]SensorEndTouchEvent(nil), s.EndEvents...)
	sample.SensorEvents = s
	j := id.GetJointEvents()
	j.JointEvents = append([]JointEvent(nil), j.JointEvents...)
	sample.JointEvents = j
	w := getUnlockedWorldFromId(id)
	scratch.contacts = scratch.contacts[:0]
	if len(scratch.contactMarks) < len(w.contacts) {
		scratch.contactMarks = append(scratch.contactMarks, make([]uint32, len(w.contacts)-len(scratch.contactMarks))...)
	}
	scratch.contactEpoch++
	if scratch.contactEpoch == 0 {
		clear(scratch.contactMarks)
		scratch.contactEpoch = 1
	}
	manifoldCount := 0
	for _, body := range sample.Bodies {
		for key := w.bodies[body.Body.Index1-1].headContactKey; key != NullIndex; {
			index, edge := key>>1, key&1
			contact := &w.contacts[index]
			if contact.flags&contactTouchingFlag != 0 && scratch.contactMarks[index] != scratch.contactEpoch {
				scratch.contactMarks[index] = scratch.contactEpoch
				scratch.contacts = append(scratch.contacts, index)
				manifoldCount += len(contact.manifolds)
			}
			key = contact.edges[edge].nextKey
		}
	}
	if len(scratch.contacts) == 0 {
		return
	}
	sample.Contacts = make([]ContactData, len(scratch.contacts))
	manifolds := make([]Manifold, manifoldCount)
	offset := 0
	for i, index := range scratch.contacts {
		contact := &w.contacts[index]
		a, b := &w.shapes[contact.shapeIdA], &w.shapes[contact.shapeIdB]
		end := offset + len(contact.manifolds)
		copy(manifolds[offset:end], contact.manifolds)
		if !observedContactSolved(w, contact) {
			for mi := offset; mi < end; mi++ {
				for pi := range manifolds[mi].Points[:manifolds[mi].PointCount] {
					manifolds[mi].Points[pi].TotalNormalImpulse = 0
				}
			}
		}
		// Limiting capacity also keeps a caller's append from altering its next contact.
		sample.Contacts[i] = ContactData{
			ContactId: ContactID{contact.contactId + 1, w.worldId, 0, contact.generation},
			ShapeIdA:  ShapeID{a.id + 1, w.worldId, a.generation},
			ShapeIdB:  ShapeID{b.id + 1, w.worldId, b.generation},
			Manifolds: manifolds[offset:end:end], ManifoldCount: contact.manifoldCount,
		}
		offset = end
	}
}

// Capture membership before sleeping/waking can move contacts between sets.
func captureObservedSolvedContacts(w *world) {
	if !w.toiContinuation.enabled && !w.toiContinuation.observing {
		return
	}
	if w.toiContinuation.solvedContacts == nil {
		w.toiContinuation.solvedContacts = map[int32]uint32{}
	}
	for i := range w.constraintGraph.colors {
		color := &w.constraintGraph.colors[i]
		for _, spec := range color.contacts {
			c := &w.contacts[spec.contactId]
			w.toiContinuation.solvedContacts[c.contactId] = c.generation
		}
		for _, spec := range color.convexContacts {
			c := &w.contacts[spec]
			w.toiContinuation.solvedContacts[c.contactId] = c.generation
		}
	}
}
func observedContactSolved(w *world, c *contact) bool {
	g, ok := w.toiContinuation.solvedContacts[c.contactId]
	return ok && g == c.generation
}
