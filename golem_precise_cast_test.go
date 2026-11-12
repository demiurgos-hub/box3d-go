package box3d

import "testing"

func TestGolemPreciseCastCoordinateEnvelopes(t *testing.T) {
	for _, origin := range []float32{0, 768, -768, 2304, -2304, 4090, -4090} {
		d := DefaultWorldDef()
		w := CreateWorld(&d)
		bd := DefaultBodyDef()
		bd.Position = Pos{X: origin + 3}
		body := w.CreateBody(&bd)
		sd := DefaultShapeDef()
		body.CreateSphereShape(&sd, &Sphere{Radius: 1})
		proxy := ShapeProxy{Points: []Vec3{{}}, Count: 1, Radius: .5}
		hit := false
		_, valid := w.PreciseCastShape(Pos{X: origin}, &proxy, Vec3{X: 6}, DefaultQueryFilter(), .000001, func(_ ShapeID, point Pos, _ Vec3, f float32, _ uint64, _, _ int, _ any) float32 {
			hit = true
			if AbsFloat(f*6-1.5) > .0001 || AbsFloat(point.X-(origin+2)) > .001 {
				t.Errorf("origin=%v distance=%v point=%v", origin, f*6, point)
			}
			return 1
		}, nil)
		if !valid || !hit {
			t.Errorf("origin=%v valid=%v hit=%v", origin, valid, hit)
		}
		w.Destroy()
	}
}

func TestGolemPreciseCastDoesNotUseSolverSlop(t *testing.T) {
	d := DefaultWorldDef()
	w := CreateWorld(&d)
	defer w.Destroy()
	bd := DefaultBodyDef()
	bd.Position = Pos{X: 2, Y: 2}
	body := w.CreateBody(&bd)
	sd := DefaultShapeDef()
	h := MakeBoxHull(.5, 2, 10)
	body.CreateHullShape(&sd, &h.Base)
	proxy := ShapeProxy{Points: []Vec3{{Y: -.5}, {Y: .5}}, Count: 2, Radius: .4}
	var hit bool
	var distance float32
	_, valid := w.PreciseCastShape(Pos{Y: .905}, &proxy, Vec3{X: 4}, DefaultQueryFilter(), .000001, func(_ ShapeID, _ Pos, n Vec3, f float32, _ uint64, _, _ int, _ any) float32 {
		hit = true
		distance = f * 4
		if n.X > -.999 {
			t.Fatal(n)
		}
		return 1
	}, nil)
	if !valid || !hit || AbsFloat(distance-1.1) > .00001 {
		t.Fatalf("valid=%v hit=%v distance=%v", valid, hit, distance)
	}
}
