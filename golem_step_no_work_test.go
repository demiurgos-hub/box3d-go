package box3d

import "testing"

func TestNativeContinuationNoWorkSetupRetainsParticipants(t *testing.T) {
	w := continuationWorld(t)
	if err := w.SetSpeculativeCCD(true); err != nil {
		t.Fatal(err)
	}
	d := DefaultBodyDef()
	sd := DefaultShapeDef()
	for i := 0; i < 32; i++ {
		d.Position = Pos{X: float32(i + 10), Y: 20}
		b := w.CreateBody(&d)
		b.CreateSphereShape(&sd, &Sphere{Radius: .5})
	}
	d = DefaultBodyDef()
	d.Type = DynamicBody
	d.LinearVelocity = Vec3{X: 1}
	awake := w.CreateBody(&d)
	awake.CreateSphereShape(&sd, &Sphere{Radius: .2})
	d.Type = KinematicBody
	d.Position = Pos{Y: 5}
	d.LinearVelocity = Vec3Zero
	sleeper := w.CreateBody(&d)
	sleeper.CreateSphereShape(&sd, &Sphere{Radius: .5})
	sleeper.SetAwake(false)
	d.Type = DynamicBody
	d.Position = Pos{Y: 10}
	disabled := w.CreateBody(&d)
	disabled.CreateSphereShape(&sd, &Sphere{Radius: .5})
	disabled.Disable()
	native := getWorldFromId(w)
	samples := continuationStep(t, w)
	if native.toiContinuation.engine != nil || !native.toiContinuation.noEligibleMotion || native.toiContinuation.stats != (TOIContinuationStats{}) {
		t.Fatal("no eligible motion allocated continuation workspace")
	}
	if len(samples) != 1 || len(samples[0].Bodies) != 2 || samples[0].Bodies[0].Body != awake || samples[0].Bodies[1].Body != sleeper {
		t.Fatalf("enabled nonstatic ordering/coverage: %+v", samples)
	}
	ids := native.speculativeCCD.bodyIDs
	if len(ids) != 2 || ids[0] != awake.Index1-1 || ids[1] != sleeper.Index1-1 {
		t.Fatalf("predictive participant coverage: %v", ids)
	}
	// Eligibility is recomputed from this interval's actual solver output.
	d = DefaultBodyDef()
	d.Position = Pos{X: 2}
	wall := w.CreateBody(&d)
	h := MakeBoxHull(.01, 2, 2)
	wall.CreateHullShape(&sd, &h.Base)
	awake.SetLinearVelocity(Vec3{X: 400})
	continuationStep(t, w)
	if native.toiContinuation.engine == nil || native.toiContinuation.noEligibleMotion || native.toiContinuation.stats.Events == 0 || awake.GetPosition().X > 1.8 {
		t.Fatalf("later fast motion omitted: %v %+v", awake.GetPosition(), native.toiContinuation.stats)
	}
}
