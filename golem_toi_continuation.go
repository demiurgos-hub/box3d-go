// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"container/heap"
	"errors"
	"fmt"
	"slices"
)

// ErrTOIContinuation reports incomplete local continuous contact response.
var ErrTOIContinuation = errors.New("box3d: local TOI continuation failed")

// TOIContinuationStats counts attempted work, including a failed native step.
// IslandBodies and ContactRows are sums over impact solves, not unique counts.
type TOIContinuationStats struct{ Candidates, PairTests, Events, IslandBodies, ContactRows int }

type toiPair struct{ a, b int32 }
type toiContactKey struct {
	toiPair
	child int32
}
type toiFeature struct {
	key      toiContactKey
	feature  uint32
	triangle int32
}
type toiBody struct {
	id       int32
	initial  bodySim
	sim      bodySim
	state    bodyState
	fraction float32
	revision uint64
}
type toiEvent struct {
	pair                 toiPair
	fraction             float32
	revisionA, revisionB uint64
}
type toiQueue []toiEvent

func (q toiQueue) Len() int { return len(q) }
func (q toiQueue) Less(i, j int) bool {
	a, b := q[i], q[j]
	return a.fraction < b.fraction || a.fraction == b.fraction && (a.pair.a < b.pair.a || a.pair.a == b.pair.a && a.pair.b < b.pair.b)
}
func (q toiQueue) Swap(i, j int) { q[i], q[j] = q[j], q[i] }
func (q *toiQueue) Push(v any)   { *q = append(*q, v.(toiEvent)) }
func (q *toiQueue) Pop() any     { n := len(*q) - 1; v := (*q)[n]; *q = (*q)[:n]; return v }

type toiContinuationState struct {
	enabled          bool
	err              error
	stats            TOIContinuationStats
	samples          []MotionStepSample
	handled          map[int32]bool
	noEligibleMotion bool
	engine           *toiEngine
	observing        bool
	solvedContacts   map[int32]uint32
}
type toiEngine struct {
	w                                    *world
	ctx                                  *stepContext
	bodies                               map[int32]*toiBody
	tree                                 DynamicTree
	proxies                              map[int32]int
	queue                                toiQueue
	queued                               map[toiPair][2]uint64
	active                               map[toiFeature]bool
	known                                map[toiContactKey]bool
	jointStates                          map[int32]jointSim
	jointEvents                          map[int32]bool
	lastSample                           float32
	impulseWitnesses                     map[toiContactKey]toiImpulseWitness
	rejected                             map[toiPair][2]uint64
	ownedBodies, ownedRows, positionWork int
}

const (
	toiEventLimit        = 256
	toiBodyLimit         = 4096
	toiRowLimit          = 65535
	toiPairLimit         = 1000000
	toiSampleLimit       = 1024
	toiOwnedBodyLimit    = 1 << 20
	toiOwnedRowLimit     = 1 << 20
	toiPositionWorkLimit = 65536
)

// SetTOIContinuation opts into bounded coupled contact continuation. It requires
// one native substep and continuous simulation. Native recording is unsupported.
// Discrete sensor lifecycle remains native. Swept sensor crossings must be replayed
// by the caller over the owned trajectory samples; native per-body sensor CCD is
// superseded along with the original solid sweep. Native CompoundShape contacts
// require flattened convex/mesh leaves, for static and moving bodies alike.
func (id WorldID) SetTOIContinuation(enabled bool) error {
	w := getUnlockedWorldFromId(id)
	if w == nil || w.recording != nil || enabled && !w.enableContinuous {
		return ErrObservedStepInvalid
	}
	w.toiContinuation = toiContinuationState{enabled: enabled}
	return nil
}

// TOIContinuationError returns the latest continuation failure, if any.
func (id WorldID) TOIContinuationError() error {
	w := getUnlockedWorldFromId(id)
	if w == nil {
		return ErrObservedStepInvalid
	}
	return w.toiContinuation.err
}

// GetTOIContinuationStats returns owned attempted-work counters for the last step.
func (id WorldID) GetTOIContinuationStats() TOIContinuationStats {
	w := getUnlockedWorldFromId(id)
	if w == nil {
		return TOIContinuationStats{}
	}
	return w.toiContinuation.stats
}
func resetTOIContinuation(w *world, substeps int) bool {
	s := &w.toiContinuation
	s.err = nil
	s.stats = TOIContinuationStats{}
	s.samples = nil
	s.handled = nil
	s.noEligibleMotion = false
	s.engine = nil
	clear(s.solvedContacts)
	if s.enabled && substeps != 1 {
		s.err = fmt.Errorf("%w: exactly one native substep is required", ErrTOIContinuation)
		return false
	}
	return true
}
func toiHandled(w *world, bodyID int32) bool {
	return w.toiContinuation.enabled && (w.toiContinuation.noEligibleMotion || w.toiContinuation.handled[bodyID])
}

func (e *toiEngine) body(id int32) *toiBody {
	if b := e.bodies[id]; b != nil {
		return b
	}
	b := &e.w.bodies[id]
	sim := *getBodySim(e.w, b)
	state := identityBodyState
	if s := getBodyState(e.w, b); s != nil {
		state = *s
	}
	state.flags = b.flags
	out := &toiBody{id: id, initial: sim, sim: sim, state: state, revision: 1}
	e.bodies[id] = out
	return out
}
func (b *toiBody) sweepAt(f float32) Sweep {
	t := float32(0)
	if b.fraction < 1 {
		t = ClampFloat((f-b.fraction)/(1-b.fraction), 0, 1)
	}
	endQ := NormalizeQuat(MulQuat(b.state.deltaRotation, b.sim.transform.Q))
	return Sweep{LocalCenter: b.sim.localCenter, C1: Add(ToVec3(b.sim.center), MulSV(t, b.state.deltaPosition)), C2: Add(ToVec3(b.sim.center), b.state.deltaPosition), Q1: NLerp(b.sim.transform.Q, endQ, t), Q2: endQ}
}
func (b *toiBody) poseAt(f float32) WorldTransform {
	s := b.sweepAt(f)
	return WorldTransform{ToPos(Sub(s.C1, RotateVector(s.Q1, s.LocalCenter))), s.Q1}
}
func (b *toiBody) advance(f float32) {
	s := b.sweepAt(f)
	b.sim.center = ToPos(s.C1)
	b.sim.transform = WorldTransform{ToPos(Sub(s.C1, RotateVector(s.Q1, s.LocalCenter))), s.Q1}
	r := MakeMatrixFromQuat(s.Q1)
	b.sim.invInertiaWorld = MulMM(MulMM(r, b.sim.invInertiaLocal), Transpose(r))
	b.state.deltaPosition = Vec3Zero
	b.state.deltaRotation = QuatIdentity
	b.fraction = f
}
func toiRotationRadius(s *shape, center Vec3) float32 {
	switch s.typ {
	case SphereShape:
		return Distance(s.sphere.Center, center)
	case CapsuleShape:
		return MaxFloat(Distance(s.capsule.Center1, center), Distance(s.capsule.Center2, center))
	default:
		return continuousShapeRadius(s, center)
	}
}
func toiPairFast(w *world, a, b *shape, sa, sb Sweep) bool {
	da, db := Sub(sa.C2, sa.C1), Sub(sb.C2, sb.C1)
	ca, cb := 2*Length(InvMulQuat(sa.Q1, sa.Q2).V), 2*Length(InvMulQuat(sb.Q1, sb.Q2).V)
	travel := Length(Sub(db, da)) + ca*toiRotationRadius(a, sa.LocalCenter) + cb*toiRotationRadius(b, sb.LocalCenter)
	ea, eb := computeShapeMinExtent(a, a.localCentroid)*w.bodies[a.bodyId].safetyFactor, computeShapeMinExtent(b, b.localCentroid)*w.bodies[b.bodyId].safetyFactor
	if a.typ == MeshShape || a.typ == HeightShape {
		if da == Vec3Zero && ca == 0 {
			ea = eb
		} else {
			ea = 0
		}
	}
	if b.typ == MeshShape || b.typ == HeightShape {
		if db == Vec3Zero && cb == 0 {
			eb = ea
		} else {
			eb = 0
		}
	}
	return travel > MinFloat(ea, eb)
}
func (e *toiEngine) shapeFast(s *shape) bool {
	b := e.body(s.bodyId)
	return toiShapeFast(s, b.sweepAt(b.fraction), e.w.bodies[s.bodyId].safetyFactor)
}
func toiShapeFast(s *shape, sw Sweep, safetyFactor float32) bool {
	chord := 2 * Length(InvMulQuat(sw.Q1, sw.Q2).V)
	travel := Length(Sub(sw.C2, sw.C1))
	if chord != 0 {
		travel += chord * toiRotationRadius(s, sw.LocalCenter)
	}
	limit := computeShapeMinExtent(s, s.localCentroid) * safetyFactor
	if s.typ == MeshShape || s.typ == HeightShape {
		limit = 0
	}
	return travel > limit
}
func (e *toiEngine) updateProxy(id int32) {
	s := &e.w.shapes[id]
	b := e.body(s.bodyId)
	sweep := b.sweepAt(b.fraction)
	bounds := continuousShapeBounds(s, &sweep, 1)
	if p, ok := e.proxies[id]; ok {
		e.tree.MoveProxy(p, bounds)
	} else {
		e.proxies[id] = e.tree.CreateProxy(bounds, DefaultCategoryBits, uint64(id))
	}
}
func (e *toiEngine) canonical(a, b int32) (toiPair, bool) {
	sa, sb := &e.w.shapes[a], &e.w.shapes[b]
	if e.w.bodies[sa.bodyId].setIndex == disabledSet || e.w.bodies[sb.bodyId].setIndex == disabledSet {
		return toiPair{}, false
	}
	if !shouldCreatePair(e.w, int(a), int(b)) || e.w.bodies[sa.bodyId].typ != DynamicBody && e.w.bodies[sb.bodyId].typ != DynamicBody {
		return toiPair{}, false
	}
	if !contactRegisters[sa.typ][sb.typ].supported {
		return toiPair{}, false
	}
	if !contactRegisters[sa.typ][sb.typ].primary {
		a, b = b, a
	}
	return toiPair{a, b}, true
}
func (e *toiEngine) candidates(id int32, visit func(toiPair) error) error {
	s := &e.w.shapes[id]
	b := e.body(s.bodyId)
	sw := b.sweepAt(b.fraction)
	bounds := continuousShapeBounds(s, &sw, 1)
	seen := map[toiPair]bool{}
	var failure error
	callback := func(_ int, user uint64, _ any) bool {
		other := int32(user)
		if id == other {
			return true
		}
		pair, ok := e.canonical(id, other)
		if !ok || seen[pair] {
			return true
		}
		seen[pair] = true
		e.w.toiContinuation.stats.Candidates++
		if e.w.toiContinuation.stats.Candidates > toiPairLimit {
			failure = errors.New("candidate capacity")
			return false
		}
		failure = visit(pair)
		return failure == nil
	}
	e.tree.Query(bounds, DefaultMaskBits, false, callback, nil)
	if failure != nil {
		return failure
	}
	e.w.broadPhase.trees[StaticBody].Query(bounds, DefaultMaskBits, false, callback, nil)
	if failure != nil {
		return failure
	}
	for _, bodyType := range []BodyType{DynamicBody, KinematicBody} {
		e.w.broadPhase.trees[bodyType].Query(bounds, DefaultMaskBits, false, func(proxy int, user uint64, c any) bool {
			if _, awake := e.proxies[int32(user)]; awake {
				return true
			}
			return callback(proxy, user, c)
		}, nil)
		if failure != nil {
			return failure
		}
	}
	return failure
}
func (e *toiEngine) enqueue(pair toiPair) error {
	a, b := &e.w.shapes[pair.a], &e.w.shapes[pair.b]
	if a.typ == CompoundShape || b.typ == CompoundShape {
		return errors.New("native compound continuation requires flattened convex/mesh leaves")
	}
	ba, bb := e.body(a.bodyId), e.body(b.bodyId)
	revisions := [2]uint64{ba.revision, bb.revision}
	if e.queued[pair] == revisions {
		return nil
	}
	e.queued[pair] = revisions
	f := MaxFloat(ba.fraction, bb.fraction)
	if f >= 1 {
		return nil
	}
	sa, sb := ba.sweepAt(f), bb.sweepAt(f)
	if e.isRejected(pair) {
		return nil
	}
	if !toiPairFast(e.w, a, b, sa, sb) && !(ba.revision == 1 && e.bulletFast(a.bodyId)) && !(bb.revision == 1 && e.bulletFast(b.bodyId)) {
		return nil
	}
	if a.typ == CompoundShape && (sa.C1 != sa.C2 || sa.Q1 != sa.Q2) {
		return errors.New("moving native compound continuation is unsupported; use convex leaves")
	}
	base := sa.C1
	sa.C1, sa.C2 = Sub(sa.C1, base), Sub(sa.C2, base)
	sb.C1, sb.C2 = Sub(sb.C1, base), Sub(sb.C2, base)
	e.w.toiContinuation.stats.PairTests++
	hit := shapeTimeOfImpact(a, b, &sa, &sb, 1)
	if hit.State == TOIStateFailed {
		return errors.New("native TOI did not converge")
	}
	// Native initial-contact fallback can discard an already touching closing row.
	closing, err := e.initialClosing(pair, f)
	if err != nil {
		return err
	}
	if closing {
		hit.State, hit.Fraction = TOIStateHit, 0
	}
	if hit.State != TOIStateHit || hit.Fraction >= 1 {
		return nil
	}
	if hit.Fraction == 0 && !closing {
		v := Sub(toiPointMotion(sb, hit.Point), toiPointMotion(sa, hit.Point))
		if Dot(hit.Normal, v) >= 0 {
			return nil
		}
	}
	heap.Push(&e.queue, toiEvent{pair, f + (1-f)*hit.Fraction, ba.revision, bb.revision})
	return nil
}
func toiPointMotion(s Sweep, p Vec3) Vec3 {
	q := MulQuat(s.Q2, Conjugate(s.Q1))
	if q.S < 0 {
		q = NegateQuat(q)
	}
	return Add(Sub(s.C2, s.C1), Cross(MulSV(2, q.V), Sub(p, s.C1)))
}

func runTOIContinuation(ctx *stepContext) {
	w := ctx.world
	s := &w.toiContinuation
	if !s.enabled || s.err != nil || ctx.dt <= 0 {
		return
	}
	eligible, err := toiHasEligibleMotion(ctx)
	if err != nil {
		s.err = fmt.Errorf("%w: %v", ErrTOIContinuation, err)
		return
	}
	if !eligible {
		// Preserve the ordinary no-event publication arithmetic and superseded
		// per-body CCD policy without allocating a local island workspace.
		for i := range ctx.sims {
			b := toiBody{sim: ctx.sims[i], state: ctx.states[i]}
			end := b.sweepAt(1)
			ctx.states[i].deltaPosition = Sub(end.C2, ToVec3(ctx.sims[i].center))
			ctx.states[i].deltaRotation = MulQuat(end.Q2, Conjugate(ctx.sims[i].transform.Q))
		}
		s.noEligibleMotion = true
		return
	}
	e := &toiEngine{w: w, ctx: ctx, bodies: map[int32]*toiBody{}, proxies: map[int32]int{}, queued: map[toiPair][2]uint64{}, active: map[toiFeature]bool{}, known: map[toiContactKey]bool{}, jointStates: map[int32]jointSim{}, rejected: map[toiPair][2]uint64{}}
	s.engine = e
	e.tree = CreateDynamicTree(len(ctx.sims))
	for i := range ctx.sims {
		sim, state := ctx.sims[i], ctx.states[i]
		e.bodies[sim.bodyId] = &toiBody{id: sim.bodyId, initial: sim, sim: sim, state: state, revision: 1}
		for id := w.bodies[sim.bodyId].headShapeId; id != NullIndex; id = w.shapes[id].nextShapeId {
			if w.shapes[id].sensorIndex == NullIndex {
				e.updateProxy(id)
			}
		}
	}
	ids := make([]int32, 0, len(e.proxies))
	for id := range e.proxies {
		ids = append(ids, id)
	}
	slices.Sort(ids)
	fail := func(err error) { s.err = fmt.Errorf("%w: %v", ErrTOIContinuation, err) }
	for _, id := range ids {
		if w.shapes[id].typ == CompoundShape {
			b := e.body(w.shapes[id].bodyId)
			sw := b.sweepAt(0)
			if sw.C1 != sw.C2 || sw.Q1 != sw.Q2 {
				fail(errors.New("moving native compound continuation is unsupported; use convex leaves"))
				return
			}
		}
		if e.shapeFast(&w.shapes[id]) || e.bulletFast(w.shapes[id].bodyId) {
			if err := e.candidates(id, e.enqueue); err != nil {
				fail(err)
				return
			}
		}
	}
	for e.queue.Len() > 0 {
		event := heap.Pop(&e.queue).(toiEvent)
		a, b := e.body(w.shapes[event.pair.a].bodyId), e.body(w.shapes[event.pair.b].bodyId)
		if a.revision != event.revisionA || b.revision != event.revisionB {
			continue
		}
		if err := e.checkPreSolve(event); err != nil {
			fail(err)
			return
		}
		if e.isRejected(event.pair) {
			continue
		}
		if s.stats.Events == 0 {
			if err := e.sample(0, e.ordinaryImpulses()); err != nil {
				fail(err)
				return
			}
		}
		s.stats.Events++
		if s.stats.Events > toiEventLimit {
			fail(errors.New("impact event capacity"))
			return
		}
		members, contacts, joints, err := e.collect(event)
		if err != nil {
			fail(err)
			return
		}
		s.stats.IslandBodies += len(members)
		if err = e.solveImpact(event, members, contacts, joints); err != nil {
			fail(err)
			return
		}
		for _, id := range members {
			body := &w.bodies[id]
			if body.typ == StaticBody {
				continue
			}
			for sid := body.headShapeId; sid != NullIndex; sid = w.shapes[sid].nextShapeId {
				if w.shapes[sid].sensorIndex == NullIndex {
					e.updateProxy(sid)
				}
			}
		}
		for _, id := range members {
			if w.bodies[id].typ == StaticBody {
				continue
			}
			for sid := w.bodies[id].headShapeId; sid != NullIndex; sid = w.shapes[sid].nextShapeId {
				if w.shapes[sid].sensorIndex == NullIndex {
					if err = e.candidates(sid, e.enqueue); err != nil {
						fail(err)
						return
					}
				}
			}
		}
	}
	if err := e.publish(); err != nil {
		fail(err)
	}
}

func (e *toiEngine) bulletFast(id int32) bool {
	b := e.body(id)
	return toiBulletFast(&b.sim, b.sweepAt(b.fraction), e.w.bodies[id].safetyFactor)
}
func (e *toiEngine) isRejected(pair toiPair) bool {
	a, b := e.body(e.w.shapes[pair.a].bodyId), e.body(e.w.shapes[pair.b].bodyId)
	return e.rejected[pair] == [2]uint64{a.revision, b.revision}
}
func (e *toiEngine) checkPreSolve(event toiEvent) error {
	_, err := e.build(event.pair, event.fraction, false)
	return err
}

func toiBulletFast(sim *bodySim, sweep Sweep, safetyFactor float32) bool {
	if sim.flags&isBullet == 0 {
		return false
	}
	travel := Length(Sub(sweep.C2, sweep.C1))
	chord := 2 * Length(InvMulQuat(sweep.Q1, sweep.Q2).V)
	if chord != 0 {
		travel += chord * Length(sim.maxExtent)
	}
	return travel > safetyFactor*sim.minExtent
}

// Inspect actual solver output before copying bodies or building swept trees.
// This uses the same eligibility as candidate generation, including every leaf
// of an eligible bullet and the zero-inner-extent moving-mesh policy.
func toiHasEligibleMotion(ctx *stepContext) (bool, error) {
	w := ctx.world
	for i := range ctx.sims {
		sim, state := ctx.sims[i], ctx.states[i]
		b := toiBody{sim: sim, state: state}
		sweep := b.sweepAt(0)
		body := &w.bodies[sim.bodyId]
		bullet := toiBulletFast(&sim, sweep, body.safetyFactor)
		for id := body.headShapeId; id != NullIndex; id = w.shapes[id].nextShapeId {
			shape := &w.shapes[id]
			if shape.sensorIndex != NullIndex {
				continue
			}
			if shape.typ == CompoundShape && (sweep.C1 != sweep.C2 || sweep.Q1 != sweep.Q2) {
				return false, errors.New("moving native compound continuation is unsupported; use convex leaves")
			}
			if bullet || toiShapeFast(shape, sweep, body.safetyFactor) {
				return true, nil
			}
		}
	}
	return false, nil
}
