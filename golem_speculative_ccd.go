// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"errors"
	"fmt"
	"slices"
)

// ErrSpeculativeCCD reports an unresolved predictive contact. Transactional
// callers must reject the whole frame; the native step may already have run.
var ErrSpeculativeCCD = errors.New("box3d: unresolved speculative CCD contact")

const buildingPredictiveContact = 0x02000000

type predictiveShape struct {
	id     int32
	sweep  Sweep
	bounds AABB
	fast   bool
}

type predictiveContact struct {
	a, b      predictiveShape
	fraction  float32
	contactID int32
	filtered  bool
	origin    Pos
	err       error
}

type speculativeCCDScratch struct {
	enabled       bool
	stats         SpeculativeCCDStats
	shapes        []predictiveShape
	contacts      map[uint64]*predictiveContact
	keys          []uint64
	kinematics    map[int32]kinematicCCDTarget
	kinematicTree DynamicTree
	kinematicIDs  []int32
	bodyIDs       []int32
}

// Kinematic poses are independent of contact impulses. Retaining their sweeps
// and shape metadata before finalization lets solver-accelerated bodies query
// them without racing the native parallel update of transforms and AABBs.
type kinematicCCDTarget struct {
	shape  shape
	sweep  Sweep
	bounds AABB
}

// SpeculativeCCDStats reports work performed by the latest native step.
// Contacts counts predictive manifolds, not geometric touching or gameplay hits.
// These counters exclude the built-in post-solve continuous sweeps.
type SpeculativeCCDStats struct {
	Candidates, PairTests, Contacts int
}

// GetSpeculativeCCDStats returns the latest step's copied counters, or zero for
// an invalid or locked world. StepObserved includes them in its owned sample.
func (id WorldID) GetSpeculativeCCDStats() SpeculativeCCDStats {
	if w := getUnlockedWorldFromId(id); w != nil {
		return w.speculativeCCD.stats
	}
	return SpeculativeCCDStats{}
}

// SetSpeculativeCCD enables motion-predicted contacts for fast moving
// dynamic/kinematic pairs. Existing native TOI supplies the hit; the ordinary
// contact solver supplies all response. It neither subdivides simulation time
// nor certifies all resulting solver paths. Solver-induced acceleration remains
// subject to the native post-solve CCD coverage. Sensors use separate queries.
// Enabled worlds retain immutable kinematic sweeps for all fast dynamic bodies,
// including bodies accelerated by constraints after the initial prediction.
// This extension is off by default and is not supported by upstream recording.
func (id WorldID) SetSpeculativeCCD(enabled bool) error {
	w := getUnlockedWorldFromId(id)
	if w == nil || w.recording != nil {
		return ErrObservedStepInvalid
	}
	w.speculativeCCD.enabled = enabled
	return nil
}

// SpeculativeCCDError returns the first unresolved contact from the last step.
// StepObserved returns this error itself; ordinary Step callers must check it
// before publishing state when speculative CCD is enabled.
func (id WorldID) SpeculativeCCDError() error {
	w := getUnlockedWorldFromId(id)
	if w == nil {
		return ErrObservedStepInvalid
	}
	for _, key := range w.speculativeCCD.keys {
		p := w.speculativeCCD.contacts[key]
		if p.err != nil {
			return fmt.Errorf("%w: shapes %d/%d: %v", ErrSpeculativeCCD, p.a.id, p.b.id, p.err)
		}
	}
	return nil
}

// Predict unconstrained motion by running the existing integrators on copies.
// This keeps force, damping, gyroscopic, lock and speed-limit arithmetic shared.
func predictiveBodySweep(w *world, body *body, dt float32, substeps int) Sweep {
	sim := getBodySim(w, body)
	sweep := Sweep{LocalCenter: sim.localCenter, C1: ToVec3(sim.center), C2: ToVec3(sim.center), Q1: sim.transform.Q, Q2: sim.transform.Q}
	state := getBodyState(w, body)
	if state == nil {
		return sweep
	}
	sims, states := [1]bodySim{*sim}, [1]bodyState{*state}
	states[0].deltaPosition, states[0].deltaRotation = Vec3Zero, QuatIdentity
	count := MaxInt(1, substeps)
	context := stepContext{world: w, sims: sims[:], states: states[:], h: dt / float32(count), inv_dt: 1 / dt, maxLinearVelocity: w.maxLinearSpeed}
	block := solverBlock{count: 1}
	for i := 0; i < count; i++ {
		integrateVelocitiesTask(block, &context)
		integratePositionsTask(block, &context)
	}
	sweep.C2 = Add(sweep.C1, states[0].deltaPosition)
	sweep.Q2 = NormalizeQuat(MulQuat(states[0].deltaRotation, sweep.Q1))
	return sweep
}

func prepareSpeculativeCCD(w *world, dt float32, substeps int) {
	scratch := &w.speculativeCCD
	scratch.stats = SpeculativeCCDStats{}
	clear(scratch.contacts)
	clear(scratch.kinematics)
	scratch.kinematicIDs = scratch.kinematicIDs[:0]
	scratch.keys = scratch.keys[:0]
	scratch.shapes = scratch.shapes[:0]
	if !scratch.enabled || !w.enableContinuous || dt <= 0 {
		return
	}
	motionInvalidateMovingMeshCaches(w)
	fastCount := 0
	scratch.bodyIDs = scratch.bodyIDs[:0]
	for setID := range w.solverSets {
		if setID == staticSet || setID == disabledSet {
			continue
		}
		for _, sim := range w.solverSets[setID].bodySims {
			scratch.bodyIDs = append(scratch.bodyIDs, sim.bodyId)
		}
	}
	slices.Sort(scratch.bodyIDs)
	for _, id := range scratch.bodyIDs {
		body := &w.bodies[id]
		sweep := predictiveBodySweep(w, body, dt, substeps)
		chord := 2 * Length(InvMulQuat(sweep.Q1, sweep.Q2).V)
		for index := body.headShapeId; index != NullIndex; index = w.shapes[index].nextShapeId {
			shape := &w.shapes[index]
			if body.typ == KinematicBody {
				if scratch.kinematics == nil {
					scratch.kinematics = make(map[int32]kinematicCCDTarget)
				}
				scratch.kinematics[shape.id] = kinematicCCDTarget{*shape, sweep, continuousShapeBounds(shape, &sweep, 1)}
				scratch.kinematicIDs = append(scratch.kinematicIDs, shape.id)
			}
			if shape.sensorIndex != NullIndex {
				continue
			}
			travel := Distance(sweep.C1, sweep.C2) + float32(chord*continuousShapeRadius(shape, sweep.LocalCenter))
			fast := travel > body.safetyFactor*computeShapeMinExtent(shape, shape.localCentroid)
			if shape.typ == MeshShape || shape.typ == HeightShape {
				// A concave surface has no positive inner radius. Its distant
				// vertices cannot provide a safe motion threshold for a small visitor.
				fast = travel > 0
			}
			if fast {
				fastCount++
			}
			scratch.shapes = append(scratch.shapes, predictiveShape{shape.id, sweep, continuousShapeBounds(shape, &sweep, 1), fast})
		}
	}
	if len(scratch.kinematics) > 0 {
		scratch.kinematicTree = CreateDynamicTree(len(scratch.kinematics))
		for _, id := range scratch.kinematicIDs {
			scratch.kinematicTree.CreateProxy(scratch.kinematics[id].bounds, DefaultCategoryBits, uint64(id))
		}
	}
	if fastCount == 0 {
		return
	}
	// A temporary moving-shape tree includes both swept bounds. Querying only
	// current poses would miss two objects crossing perpendicular paths.
	tree := CreateDynamicTree(len(scratch.shapes))
	for i, shape := range scratch.shapes {
		tree.CreateProxy(shape.bounds, DefaultCategoryBits, uint64(i))
	}
	candidates := make(map[uint64][2]int)
	for i, shape := range scratch.shapes {
		if !shape.fast {
			continue
		}
		tree.Query(shape.bounds, DefaultMaskBits, false, func(_ int, index uint64, _ any) bool {
			j := int(index)
			if i == j {
				return true
			}
			a, b := scratch.shapes[i], scratch.shapes[j]
			if a.id > b.id {
				a, b = b, a
			}
			key := shapePairKey(int(a.id), int(b.id), 0)
			candidates[key] = [2]int{i, j}
			return true
		}, nil)
	}
	keys := make([]uint64, 0, len(candidates))
	scratch.stats.Candidates = len(candidates)
	for key := range candidates {
		keys = append(keys, key)
	}
	slices.Sort(keys)
	if scratch.contacts == nil {
		scratch.contacts = make(map[uint64]*predictiveContact)
	}
	for _, key := range keys {
		indices := candidates[key]
		a, b := scratch.shapes[indices[0]], scratch.shapes[indices[1]]
		if a.id > b.id {
			a, b = b, a
		}
		shapeA, shapeB := &w.shapes[a.id], &w.shapes[b.id]
		if !shouldCreatePair(w, int(a.id), int(b.id)) {
			continue
		}
		// Portable compounds are flattened to native convex children. Native
		// compounds need child-key contacts and retain their upstream coverage.
		if shapeA.typ == CompoundShape || shapeB.typ == CompoundShape {
			continue
		}
		if shapeB.typ == MeshShape || shapeB.typ == HeightShape {
			a, b, shapeA, shapeB = b, a, shapeB, shapeA
		}
		base := a.sweep.C1
		sa, sb := a.sweep, b.sweep
		sa.C1, sa.C2 = Sub(sa.C1, base), Sub(sa.C2, base)
		sb.C1, sb.C2 = Sub(sb.C1, base), Sub(sb.C2, base)
		scratch.stats.PairTests++
		hit := shapeTimeOfImpact(shapeA, shapeB, &sa, &sb, 1)
		if hit.State == TOIStateFailed {
			scratch.contacts[key] = &predictiveContact{a: a, b: b, contactID: NullIndex, err: errors.New("native time of impact did not converge")}
			scratch.keys = append(scratch.keys, key)
			continue
		}
		if hit.State != TOIStateHit || hit.Fraction <= 0 || hit.Fraction >= 1 {
			continue
		}
		p := &predictiveContact{a: a, b: b, fraction: hit.Fraction, contactID: NullIndex}
		scratch.contacts[key] = p
		scratch.keys = append(scratch.keys, key)
		if !containsKey(&w.broadPhase.pairSet, key) {
			createContact(w, shapeA, shapeB, 0)
		}
	}
}

func predictiveContactFor(w *world, c *contact) *predictiveContact {
	return w.speculativeCCD.contacts[shapePairKey(int(c.shapeIdA), int(c.shapeIdB), int(c.childIndex))]
}

func updatePredictiveContact(w *world, workerIndex int, c *contact, shapeA, shapeB *shape, arena arena) bool {
	p := predictiveContactFor(w, c)
	if p == nil || c.flags&buildingPredictiveContact != 0 {
		return false
	}
	p.contactID = c.contactId
	a, b := p.a, p.b
	if a.id != shapeA.id {
		a, b = b, a
	}
	// Contact generation must stay near the pair. Building absolute body
	// origins from a distant COM would round away rotated local COM offsets.
	base := a.sweep.C1
	p.origin = ToPos(base)
	sa, sb := a.sweep, b.sweep
	sa.C1, sa.C2 = Sub(sa.C1, base), Sub(sa.C2, base)
	sb.C1, sb.C2 = Sub(sb.C1, base), Sub(sb.C2, base)
	poseA, poseB := GetSweepTransform(&sa, p.fraction), GetSweepTransform(&sb, p.fraction)
	futureA, futureB := *shapeA, *shapeB
	xfA, xfB := WorldTransform{ToPos(poseA.P), poseA.Q}, WorldTransform{ToPos(poseB.P), poseB.Q}
	// Mesh traversal reads the visitor shape's cached AABB. Supply a private
	// copy at the same future, recentered pose; other contacts share the original.
	futureA.aabb = computeFatShapeAABB(shapeA, xfA, SpeculativeDistance())
	futureB.aabb = computeFatShapeAABB(shapeB, xfB, SpeculativeDistance())
	if c.flags&simMeshContact != 0 {
		c.meshContact.queryBounds = AABB{Vec3{fltMax, fltMax, fltMax}, Vec3{-fltMax, -fltMax, -fltMax}}
	}
	c.flags |= buildingPredictiveContact
	touching := updateContact(w, workerIndex, c, &futureA, a.sweep.LocalCenter, xfA, &futureB, b.sweep.LocalCenter, xfB, true, arena)
	c.flags &^= buildingPredictiveContact | relativeTransformValid
	if c.flags&simMeshContact != 0 {
		c.meshContact.queryBounds = AABB{Vec3{fltMax, fltMax, fltMax}, Vec3{-fltMax, -fltMax, -fltMax}}
	}
	if !touching {
		if !p.filtered {
			p.err = errors.New("native TOI did not produce a contact manifold")
		}
		return false
	}
	centerA, centerB := Lerp(sa.C1, sa.C2, p.fraction), Lerp(sb.C1, sb.C2, p.fraction)
	for i := range c.manifolds {
		m := &c.manifolds[i]
		for j := int32(0); j < m.PointCount; j++ {
			point := &m.Points[j]
			hitDifference := Add(Sub(centerB, centerA), Sub(point.AnchorB, point.AnchorA))
			point.AnchorA = RotateVector(a.sweep.Q1, InvRotateVector(poseA.Q, point.AnchorA))
			point.AnchorB = RotateVector(b.sweep.Q1, InvRotateVector(poseB.Q, point.AnchorB))
			startDifference := Add(Sub(b.sweep.C1, a.sweep.C1), Sub(point.AnchorB, point.AnchorA))
			point.Separation += Dot(Sub(startDifference, hitDifference), m.Normal)
			point.NormalImpulse, point.TotalNormalImpulse, point.NormalVelocity = 0, 0, 0
			point.Persisted = false
			if point.Separation < -LinearSlop() {
				p.err = errors.New("rotating predictive plane already penetrates separated start poses")
			}
		}
	}
	return true
}

func finishSpeculativeCCD(w *world) {
	for _, key := range w.speculativeCCD.keys {
		p := w.speculativeCCD.contacts[key]
		if p.contactID == NullIndex || p.err != nil || p.filtered {
			continue
		}
		c := &w.contacts[p.contactID]
		if c.contactId == NullIndex {
			continue
		}
		w.speculativeCCD.stats.Contacts++
		impact := false
		for _, m := range c.manifolds {
			for j := int32(0); j < m.PointCount; j++ {
				impact = impact || m.Points[j].TotalNormalImpulse > 0
			}
		}
		if impact {
			// As with a swept TOI, the next contact boundary must not allow
			// compliance to carry the remaining closing speed through the face.
			for _, edge := range c.edges {
				body := &w.bodies[edge.bodyId]
				if body.typ == DynamicBody {
					getBodySim(w, body).flags |= hadTimeOfImpact
					body.flags |= hadTimeOfImpact
				}
			}
		}
	}
}

func rejectPredictiveContact(w *world, c *contact) {
	if c.flags&buildingPredictiveContact != 0 {
		if p := predictiveContactFor(w, c); p != nil {
			p.filtered = true
		}
	}
}

func predictiveWorldPoint(w *world, c *contact, point Pos) Pos {
	if c.flags&buildingPredictiveContact != 0 {
		if p := predictiveContactFor(w, c); p != nil {
			return OffsetPos(p.origin, ToVec3(point))
		}
	}
	return point
}
