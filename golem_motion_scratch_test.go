package box3d

import (
	"reflect"
	"testing"
)

func motionContactFixture(t *testing.T, pairs int) (WorldID, []MotionBodySample) {
	t.Helper()
	w := motionStepWorld(t)
	bodies := make([]MotionBodySample, 0, pairs*2)
	for i := 0; i < pairs; i++ {
		for j := 0; j < 2; j++ {
			body := motionStepBody(w, DynamicBody, Pos{X: float32(i) * 4, Y: float32(j) * .999}, Vec3Zero)
			motionStepSphere(body, .5)
			bodies = append(bodies, MotionBodySample{Body: body})
		}
	}
	w.Step(1.0/240, 1)
	w.Step(1.0/240, 1)
	return w, bodies
}

func TestGolemMotionContactSamplesOwnTheirStorage(t *testing.T) {
	w, bodies := motionContactFixture(t, 8)
	scratch := motionStepScratch{}
	a := MotionStepSample{Bodies: bodies}
	motionCopyEvents(w, &a, &scratch)
	b := MotionStepSample{Bodies: bodies}
	motionCopyEvents(w, &b, &scratch)
	if len(a.Contacts) != 8 || !reflect.DeepEqual(a.Contacts, b.Contacts) {
		t.Fatalf("missing or inconsistent contacts: %d %d", len(a.Contacts), len(b.Contacts))
	}
	seen := map[ContactID]bool{}
	for _, body := range bodies {
		contacts := make([]ContactData, body.Body.GetContactCapacity())
		n := body.Body.GetContactData(contacts)
		for _, contact := range contacts[:n] {
			if seen[contact.ContactId] {
				continue
			}
			seen[contact.ContactId] = true
			if !reflect.DeepEqual(contact, a.Contacts[len(seen)-1]) {
				t.Fatal("sample differs from public body contact enumeration")
			}
		}
	}
	before := b.Contacts[0].Manifolds[0]
	a.Contacts[0].Manifolds[0].Normal = Vec3{X: 123}
	if b.Contacts[0].Manifolds[0] != before {
		t.Fatal("sample manifold storage aliases another sample")
	}
	for _, c := range a.Contacts {
		if cap(c.Manifolds) != len(c.Manifolds) {
			t.Fatal("appending a manifold may overwrite another contact")
		}
	}
	a.Contacts[0].Manifolds = append(a.Contacts[0].Manifolds, Manifold{Normal: Vec3{Z: 456}})
	c := MotionStepSample{Bodies: bodies}
	motionCopyEvents(w, &c, &scratch)
	if !reflect.DeepEqual(b.Contacts, c.Contacts) {
		t.Fatal("mutating a sample changed native contacts")
	}
	w.Step(1.0/240, 1)
	if !reflect.DeepEqual(b.Contacts, c.Contacts) {
		t.Fatal("a later solver step changed retained contacts")
	}
}

func TestGolemMotionContactSampleAllocations(t *testing.T) {
	w, bodies := motionContactFixture(t, 32)
	scratch := motionStepScratch{}
	allocs := testing.AllocsPerRun(20, func() {
		sample := MotionStepSample{Bodies: bodies}
		motionCopyEvents(w, &sample, &scratch)
		if len(sample.Contacts) != 32 {
			panic("missing sample contacts")
		}
	})
	t.Logf("32 retained contacts: %.0f allocations per sample", allocs)
	if allocs > 2 {
		t.Fatalf("contact sampling allocated %.0f times; want only owned contacts and manifolds", allocs)
	}
}

func TestGolemMotionLeafTraversalUsesBoundedScratch(t *testing.T) {
	w := motionStepWorld(t)
	a := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
	b := motionStepBody(w, DynamicBody, Pos{Y: 2}, Vec3Zero)
	sd := DefaultShapeDef()
	mesh := a.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	hull := MakeBoxHull(2, .1, 2)
	compound := CreateCompound(&CompoundDef{Hulls: []CompoundHullDef{{Hull: &hull.Base, Transform: TransformIdentity}}, HullCount: 1})
	if compound == nil {
		t.Fatal("cook compound")
	}
	compoundShape := a.CreateBakedCompoundShape(&sd, compound)
	sphere := motionStepSphere(b, .5)
	start, end := motionPose(0, 2, 0, QuatIdentity), motionPose(0, 0, 0, QuatIdentity)
	pose := motionPose(0, 0, 0, QuatIdentity)
	for name, target := range map[string]ShapeID{"mesh": mesh, "compound": compoundShape} {
		t.Run(name, func(t *testing.T) {
			want, err := sphere.CastMotion(target, start, end, pose, pose, .001, true)
			if err != nil || !want.Hit || want.Fraction <= 0 || want.Fraction >= 1 {
				t.Fatalf("expected swept hit: %+v %v", want, err)
			}
			allocs := testing.AllocsPerRun(20, func() {
				got, err := sphere.CastMotion(target, start, end, pose, pose, .001, true)
				if err != nil || got != want {
					panic("unstable cast result")
				}
			})
			// The existing contact manifold implementation allocates once per
			// touched leaf. Traversal callbacks and proxies need no heap storage.
			limit := float64(1)
			if name == "mesh" {
				limit = 2
			}
			if allocs > limit {
				t.Fatalf("leaf traversal allocated %.0f times; want at most %.0f", allocs, limit)
			}
		})
	}
}
