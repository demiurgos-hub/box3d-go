// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"errors"
	"slices"
)

func toiKey(c *contact) toiContactKey {
	return toiContactKey{toiPair{c.shapeIdA, c.shapeIdB}, c.childIndex}
}
func (e *toiEngine) build(pair toiPair, f float32, snapshot bool) ([]contact, error) {
	w := e.w
	if e.isRejected(pair) {
		return nil, nil
	}
	a, b := w.shapes[pair.a], w.shapes[pair.b]
	if a.typ == CompoundShape || b.typ == CompoundShape {
		return nil, errors.New("native compound continuation requires flattened convex/mesh leaves")
	}
	ba, bb := e.body(a.bodyId), e.body(b.bodyId)
	xa, xb := ba.poseAt(f), bb.poseAt(f)
	a.aabb = computeFatShapeAABB(&a, xa, SpeculativeDistance())
	b.aabb = computeFatShapeAABB(&b, xb, SpeculativeDistance())
	sw := *w
	sw.manifoldAllocators = nil
	sw.manifoldAllocatorMutex = createMutex()
	sw.taskContexts = make([]taskContext, 1)
	sw.speculativeCCD = speculativeCCDScratch{}
	if snapshot {
		sw.preSolveFcn = nil
	} else if w.preSolveFcn != nil {
		sw.preSolveFcn = func(a, b ShapeID, p Pos, n Vec3, c any) bool {
			accepted := w.preSolveFcn(a, b, p, n, c)
			if !accepted {
				e.rejected[pair] = [2]uint64{ba.revision, bb.revision}
			}
			return accepted
		}
	}
	defer func() {
		for i := range sw.manifoldAllocators {
			destroyBlockAllocator(&sw.manifoldAllocators[i])
		}
		destroyMutex(sw.manifoldAllocatorMutex)
	}()
	arena := createArena(1 << 16)
	defer destroyArena(&arena)
	var out []contact
	appendChild := func(child int32) {
		c := contact{shapeIdA: a.id, shapeIdB: b.id, childIndex: child, flags: enableSpeculativePoints}
		c.edges[0].bodyId, c.edges[1].bodyId = a.bodyId, b.bodyId
		if (a.flags|b.flags)&uint8(enablePreSolveEvents) != 0 {
			c.flags |= simEnablePreSolveEvents
		}
		if w.bodies[a.bodyId].typ == StaticBody || w.bodies[b.bodyId].typ == StaticBody {
			c.flags |= contactStaticFlag
		}
		if a.typ == MeshShape || a.typ == HeightShape || a.typ == CompoundShape && GetCompoundChild(a.compound, int(child)).Type == MeshShape {
			c.flags |= simMeshContact
		}
		if updateContact(&sw, 0, &c, &a, ba.sim.localCenter, xa, &b, bb.sim.localCenter, xb, true, arena) {
			c.manifolds = append([]Manifold(nil), c.manifolds...)
			out = append(out, c)
		}
	}
	if a.typ == CompoundShape {
		local := b.aabb.Transform(InvertTransform(Transform{ToVec3(xa.P), xa.Q}))
		QueryCompound(a.compound, local, func(_ *CompoundData, child int, _ any) bool { appendChild(int32(child)); return true }, nil)
	} else {
		appendChild(NullIndex)
	}
	return out, nil
}
func (e *toiEngine) initialClosing(pair toiPair, f float32) (bool, error) {
	contacts, err := e.build(pair, f, false)
	if err != nil {
		return false, err
	}
	a, b := &e.w.shapes[pair.a], &e.w.shapes[pair.b]
	sa, sb := e.body(a.bodyId).sweepAt(f), e.body(b.bodyId).sweepAt(f)
	for _, c := range contacts {
		for _, m := range c.manifolds {
			for _, p := range m.Points[:m.PointCount] {
				if p.Separation <= 0 {
					point := Add(sa.C1, p.AnchorA)
					if Dot(m.Normal, Sub(toiPointMotion(sb, point), toiPointMotion(sa, point))) < 0 {
						return true, nil
					}
				}
			}
		}
	}
	return false, nil
}
func (e *toiEngine) collect(event toiEvent) ([]int32, []contact, []int32, error) {
	w := e.w
	members := map[int32]bool{}
	var pending []int32
	add := func(id int32) {
		if w.bodies[id].setIndex != disabledSet && w.bodies[id].typ != StaticBody && !members[id] {
			members[id] = true
			pending = append(pending, id)
		}
	}
	add(w.shapes[event.pair.a].bodyId)
	add(w.shapes[event.pair.b].bodyId)
	contacts := map[toiContactKey]contact{}
	joints := map[int32]bool{}
	visited := map[toiPair]bool{}
	visitPair := func(pair toiPair) error {
		if visited[pair] {
			return nil
		}
		visited[pair] = true
		cs, err := e.build(pair, event.fraction, false)
		if err != nil {
			return err
		}
		for _, c := range cs {
			contacts[toiKey(&c)] = c
			add(c.edges[0].bodyId)
			add(c.edges[1].bodyId)
		}
		return nil
	}
	if err := visitPair(event.pair); err != nil {
		return nil, nil, nil, err
	}
	for next := 0; next < len(pending); next++ {
		if len(pending) > toiBodyLimit {
			return nil, nil, nil, errors.New("local island body capacity")
		}
		id := pending[next]
		body := &w.bodies[id]
		if e.body(id).fraction > event.fraction {
			return nil, nil, nil, errors.New("local island time regression")
		}
		for key := body.headJointKey; key != NullIndex; {
			joint := &w.joints[key>>1]
			edge := key & 1
			key = joint.edges[edge].nextKey
			if joint.setIndex == disabledSet || w.bodies[joint.edges[1-edge].bodyId].setIndex == disabledSet {
				continue
			}
			joints[joint.jointId] = true
			add(joint.edges[1-edge].bodyId)
		}
		if body.typ == KinematicBody {
			continue
		}
		for key := body.headContactKey; key != NullIndex; {
			c := &w.contacts[key>>1]
			edge := key & 1
			key = c.edges[edge].nextKey
			if pair, ok := e.canonical(c.shapeIdA, c.shapeIdB); ok {
				if err := visitPair(pair); err != nil {
					return nil, nil, nil, err
				}
			}
		}
		for sid := body.headShapeId; sid != NullIndex; sid = w.shapes[sid].nextShapeId {
			if w.shapes[sid].sensorIndex == NullIndex {
				if err := e.candidates(sid, visitPair); err != nil {
					return nil, nil, nil, err
				}
			}
		}
	}
	slices.Sort(pending)
	keys := make([]toiContactKey, 0, len(contacts))
	for key := range contacts {
		keys = append(keys, key)
	}
	slices.SortFunc(keys, toiCompareKey)
	cs := make([]contact, 0, len(keys))
	for _, key := range keys {
		cs = append(cs, contacts[key])
		e.known[key] = true
	}
	js := make([]int32, 0, len(joints))
	for id := range joints {
		js = append(js, id)
	}
	slices.Sort(js)
	return pending, cs, js, nil
}
func toiCompareKey(a, b toiContactKey) int {
	if a.a != b.a {
		if a.a < b.a {
			return -1
		}
		return 1
	}
	if a.b != b.b {
		if a.b < b.b {
			return -1
		}
		return 1
	}
	if a.child < b.child {
		return -1
	}
	if a.child > b.child {
		return 1
	}
	return 0
}
func toiImpactBand(w *world, c *contact) float32 {
	a, b := &w.shapes[c.shapeIdA], &w.shapes[c.shapeIdB]
	radius := func(s *shape) float32 {
		if s.typ == SphereShape {
			return s.sphere.Radius
		}
		if s.typ == CapsuleShape {
			return s.capsule.Radius
		}
		return 0
	}
	ra := radius(a)
	if a.typ == CompoundShape {
		child := GetCompoundChild(a.compound, int(c.childIndex))
		if child.Type == SphereShape {
			ra = child.Sphere.Radius
		} else if child.Type == CapsuleShape {
			ra = child.Capsule.Radius
		}
	}
	r := ra + radius(b)
	return MaxFloat(0, MaxFloat(LinearSlop(), r-LinearSlop())+.25*LinearSlop()-r)
}

func (e *toiEngine) solveImpact(event toiEvent, members []int32, contacts []contact, jointIDs []int32) error {
	w := e.w
	remaining := (1 - event.fraction) * e.ctx.dt
	if remaining <= 0 {
		return nil
	}
	if err := e.sample(event.fraction, nil); err != nil {
		return err
	}
	for _, id := range members {
		e.body(id).advance(event.fraction)
	}
	var err error
	contacts, err = e.correctPositions(event, members, contacts)
	if err != nil {
		return err
	}
	sims := make([]bodySim, len(members))
	states := make([]bodyState, len(members))
	indices := map[int32]int32{}
	for i, id := range members {
		b := e.body(id)
		sims[i], states[i], indices[id] = b.sim, b.state, int32(i)
	}
	sw := *w
	sw.solverSets = append([]solverSet(nil), w.solverSets...)
	sw.solverSets[awakeSet].bodySims = sims
	sw.solverSets[awakeSet].bodyStates = states
	sw.contacts = contacts
	sw.constraintGraph = constraintGraph{}
	sw.enableWarmStarting = false
	sw.speculativeCCD = speculativeCCDScratch{}
	graph := &sw.constraintGraph
	color := &graph.colors[overflowIndex]
	count := 0
	for i := range sw.contacts {
		c := &sw.contacts[i]
		c.contactId = int32(i)
		c.encodedBodySimA, c.encodedBodySimB = -1, -1
		if index, ok := indices[c.edges[0].bodyId]; ok {
			c.encodedBodySimA = index
		}
		if index, ok := indices[c.edges[1].bodyId]; ok {
			c.encodedBodySimB = index
		}
		color.contacts = append(color.contacts, contactSpec{contactId: int32(i), manifoldStart: int32(count), manifoldCount: uint16(c.manifoldCount)})
		count += int(c.manifoldCount)
	}
	color.contactConstraints = make([]contactConstraint, len(contacts))
	color.manifoldConstraints = make([]manifoldConstraint, count)
	local := *e.ctx
	local.world = &sw
	local.graph = graph
	local.sims = sims
	local.states = states
	local.h = remaining
	local.inv_h = 1 / remaining
	local.enableWarmStarting = false
	local.retainJointImpulses = true
	local.jointBudgetH = e.ctx.h
	local.localJointBodies = func(id int32) (*bodySim, int32) {
		if index, ok := indices[id]; ok {
			return &sims[index], index
		}
		return &e.body(id).sim, NullIndex
	}
	local.overflowSpans = []contactPrepareSpan{{start: 0, count: int32(len(contacts)), contacts: color.contacts}, {start: int32(len(contacts))}}
	atomicStoreInt(&local.anyRestitution, 0)
	prepareContactsOverflow(&local)
	admitted := map[toiFeature]bool{}
	winnerRows := 0
	rowCount := 0
	for ci := range color.contactConstraints {
		constraint := &color.contactConstraints[ci]
		c := &contacts[ci]
		key := toiKey(c)
		band := toiImpactBand(w, c)
		for mi := range constraint.constraints {
			m := &constraint.constraints[mi]
			for pi := range m.points[:m.pointCount] {
				rowCount++
				p := &m.points[pi]
				mp := c.manifolds[mi].Points[pi]
				token := toiFeature{key, mp.FeatureId, mp.TriangleIndex}
				if (key.toiPair == event.pair || e.active[token]) && mp.Separation <= band {
					admitted[token] = true
					if key.toiPair == event.pair {
						winnerRows++
					}
					separation := Dot(Sub(p.rB, p.rA), m.normal) + p.baseSeparation
					p.baseSeparation -= MaxFloat(separation, 0)
				}
			}
		}
	}
	w.toiContinuation.stats.ContactRows += rowCount
	if rowCount > toiRowLimit {
		return errors.New("local contact row capacity")
	}
	if winnerRows == 0 {
		return errors.New("TOI winner has no admitted native contact row")
	}
	for _, id := range jointIDs {
		j, ok := e.jointStates[id]
		if !ok {
			j = *getJointSim(w, &w.joints[id])
		}
		local.retainJointImpulses = ok || w.joints[id].setIndex == awakeSet
		prepareJoint(&j, &local)
		color.jointSims = append(color.jointSims, j)
	}
	local.retainJointImpulses = true
	// Local TOI rows are cold-started; accumulated native joint budgets are not
	// applied again. Each pass solves joints before nonpenetration contacts.
	for pass := 0; pass < 8; pass++ {
		local.skipContactImpulseAccumulation = pass < 7
		solveJointsOverflow(&local, false)
		solveContactsOverflow(&local)
	}
	if atomicLoadInt(&local.anyRestitution) != 0 {
		for pass := 0; pass < int(sw.restitutionIterations); pass++ {
			applyRestitutionOverflow(&local)
		}
	}
	for _, j := range color.jointSims {
		e.jointStates[j.jointId] = j
		if j.forceThreshold < fltMax || j.torqueThreshold < fltMax {
			force, torque := getJointReactionAt(w, &j, e.ctx.inv_h, func(id int32) WorldTransform {
				return e.body(id).sim.transform
			})
			if force >= j.forceThreshold || torque >= j.torqueThreshold {
				if e.jointEvents == nil {
					e.jointEvents = map[int32]bool{}
				}
				e.jointEvents[j.jointId] = true
			}
		}
	}
	for token := range e.active {
		if _, ok := indices[w.shapes[token.key.a].bodyId]; ok {
			delete(e.active, token)
		} else if _, ok := indices[w.shapes[token.key.b].bodyId]; ok {
			delete(e.active, token)
		}
	}
	for token := range admitted {
		e.active[token] = true
	}
	deltas := map[toiFeature]float32{}
	e.impulseWitnesses = map[toiContactKey]toiImpulseWitness{}
	for i := range contacts {
		c := &contacts[i]
		e.impulseWitnesses[toiKey(c)] = toiImpulseWitness{c.manifolds, ToVec3(e.body(c.edges[0].bodyId).sim.center), ToVec3(e.body(c.edges[1].bodyId).sim.center)}
	}
	for ci, c := range color.contactConstraints {
		key := toiKey(&contacts[ci])
		for mi, m := range c.constraints {
			for pi, p := range m.points[:m.pointCount] {
				mp := contacts[ci].manifolds[mi].Points[pi]
				deltas[toiFeature{key, mp.FeatureId, mp.TriangleIndex}] += p.totalNormalImpulse
			}
		}
	}
	for i, id := range members {
		b := e.body(id)
		b.sim = sims[i]
		b.state = states[i]
		b.revision++
	}
	if err := e.sample(event.fraction, deltas); err != nil {
		return err
	}
	// Pose integration advances only the remaining local time; external forces,
	// damping and already consumed motor impulses are never integrated again.
	integratePositionsTask(solverBlock{count: uint16(len(states))}, &local)
	for i, id := range members {
		e.body(id).state = states[i]
	}
	return nil
}

// The position-only phase follows the NGS structure in Box2D 2.4.1
// b2ContactSolver::SolveTOIPositionConstraints. It changes no physical velocity.
// Native manifolds and world inverse inertia are rebuilt after each correction.
func (e *toiEngine) correctPositions(event toiEvent, members []int32, contacts []contact) ([]contact, error) {
	keys := make([]toiContactKey, len(contacts))
	for i := range contacts {
		keys[i] = toiKey(&contacts[i])
	}
	for pass := 0; pass < 20; pass++ {
		minimum := float32(0)
		changed := false
		for _, key := range keys {
			e.positionWork++
			if e.positionWork > toiPositionWorkLimit {
				return nil, errors.New("TOI position work capacity")
			}
			fresh, err := e.build(key.toiPair, event.fraction, false)
			if err != nil {
				return nil, err
			}
			for _, c := range fresh {
				if c.childIndex != key.child {
					continue
				}
				for _, m := range c.manifolds {
					for _, p := range m.Points[:m.PointCount] {
						minimum = MinFloat(minimum, p.Separation)
						if p.Separation >= -LinearSlop() {
							continue
						}
						a, b := e.body(c.edges[0].bodyId), e.body(c.edges[1].bodyId)
						ma, mb := a.sim.invMass, b.sim.invMass
						ia, ib := a.sim.invInertiaWorld, b.sim.invInertiaWorld
						if e.w.bodies[a.id].typ != DynamicBody {
							ma = 0
							ia = Mat3Zero
						}
						if e.w.bodies[b.id].typ != DynamicBody {
							mb = 0
							ib = Mat3Zero
						}
						ra, rb := Cross(p.AnchorA, m.Normal), Cross(p.AnchorB, m.Normal)
						la, lb := toiLockedVector(MulSV(ma, m.Normal), a.state.flags, lockLinearX), toiLockedVector(MulSV(mb, m.Normal), b.state.flags, lockLinearX)
						aa, ab := toiLockedVector(MulMV(ia, ra), a.state.flags, lockAngularX), toiLockedVector(MulMV(ib, rb), b.state.flags, lockAngularX)
						k := Dot(m.Normal, Add(la, lb)) + Dot(ra, aa) + Dot(rb, ab)
						if k <= 0 {
							continue
						}
						correction := MinFloat(.75*(-p.Separation-LinearSlop()), e.w.contactSpeed*e.ctx.dt)
						impulse := correction / k
						for _, change := range []struct {
							body            *toiBody
							linear, angular Vec3
						}{{a, MulSV(-impulse, la), MulSV(-impulse, aa)}, {b, MulSV(impulse, lb), MulSV(impulse, ab)}} {
							if e.w.bodies[change.body.id].typ != DynamicBody {
								continue
							}
							s := &change.body.sim
							s.center = OffsetPos(s.center, change.linear)
							s.transform.Q = integrateRotation(s.transform.Q, change.angular)
							s.transform.P = OffsetPos(s.center, Neg(RotateVector(s.transform.Q, s.localCenter)))
							r := MakeMatrixFromQuat(s.transform.Q)
							s.invInertiaWorld = MulMM(MulMM(r, s.invInertiaLocal), Transpose(r))
						}
						changed = true
						// The next native manifold must see the changed geometry.
						break
					}
				}
			}
		}
		if !changed && minimum < -1.5*LinearSlop() {
			return nil, errors.New("local TOI position correction has no movable response")
		}
		if minimum >= -1.5*LinearSlop() || !changed {
			break
		}
		if pass == 19 {
			return nil, errors.New("local TOI position correction did not converge")
		}
	}
	var result []contact
	for _, key := range keys {
		fresh, err := e.build(key.toiPair, event.fraction, false)
		if err != nil {
			return nil, err
		}
		for _, c := range fresh {
			if c.childIndex == key.child {
				result = append(result, c)
			}
		}
	}
	return result, nil
}

func (e *toiEngine) publish() error {
	w := e.w
	ids := make([]int32, 0, len(e.bodies))
	for id, b := range e.bodies {
		if w.bodies[id].setIndex != disabledSet && w.bodies[id].typ != StaticBody && (w.bodies[id].setIndex == awakeSet || b.revision > 1) {
			ids = append(ids, id)
		}
	}
	slices.Sort(ids)
	for _, id := range ids {
		if w.bodies[id].setIndex >= firstSleepingSet {
			wakeBody(w, &w.bodies[id])
		}
	}
	awake := &w.solverSets[awakeSet]
	e.ctx.sims, e.ctx.states = awake.bodySims, awake.bodyStates
	w.toiContinuation.handled = map[int32]bool{}
	for _, id := range ids {
		b := e.body(id)
		if w.bodies[id].setIndex != awakeSet {
			return errors.New("TOI publication requires enabled awake participant")
		}
		index := w.bodies[id].localIndex
		target := &awake.bodyStates[index]
		*target = b.state
		end := b.sweepAt(1)
		target.deltaPosition = Sub(end.C2, ToVec3(awake.bodySims[index].center))
		target.deltaRotation = MulQuat(end.Q2, Conjugate(awake.bodySims[index].transform.Q))
		w.toiContinuation.handled[id] = true
	}
	for id, j := range e.jointStates {
		*getJointSim(w, &w.joints[id]) = j
	}
	for id := range e.jointEvents {
		setBit(&w.taskContexts[0].jointStateBitSet, uint32(id))
	}
	if w.toiContinuation.stats.Events != 0 {
		if err := e.sample(1, nil); err != nil {
			return err
		}
	}
	return nil
}

func toiLockedVector(v Vec3, flags uint32, first uint32) Vec3 {
	if flags&first != 0 {
		v.X = 0
	}
	if flags&(first<<1) != 0 {
		v.Y = 0
	}
	if flags&(first<<2) != 0 {
		v.Z = 0
	}
	return v
}
