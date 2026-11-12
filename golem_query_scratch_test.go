package box3d

import (
	"slices"
	"sync"
	"testing"
)

func TestGolemPreciseCastScratchReentryAndClip(t *testing.T) {
	w, _ := golemQueryWorld(t)
	for _, x := range []float32{3, 6, 9} {
		bd := DefaultBodyDef()
		bd.Position.X = x
		body := w.CreateBody(&bd)
		sd := DefaultShapeDef()
		body.CreateSphereShape(&sd, &Sphere{Radius: 1})
	}
	proxy := ShapeProxy{Points: []Vec3{{}}, Count: 1, Radius: .5}
	var all []float32
	_, valid := w.PreciseCastShape(Pos{}, &proxy, Vec3{X: 12}, DefaultQueryFilter(), 1e-6,
		func(_ ShapeID, _ Pos, _ Vec3, f float32, _ uint64, _, _ int, _ any) float32 {
			all = append(all, f)
			return 1
		}, nil)
	if !valid || len(all) != 3 {
		t.Fatalf("initial cast: valid=%v hits=%v", valid, all)
	}
	for _, mode := range []float32{1, 0, -1, .4} {
		var got, want []float32
		maxFraction := float32(1)
		for _, f := range all {
			if f <= maxFraction {
				want = append(want, f)
				if mode == 0 {
					break
				}
				if mode > 0 && mode < maxFraction {
					maxFraction = mode
				}
			}
		}
		_, valid := w.PreciseCastShape(Pos{}, &proxy, Vec3{X: 12}, DefaultQueryFilter(), 1e-6,
			func(_ ShapeID, _ Pos, _ Vec3, f float32, _ uint64, _, _ int, context any) float32 {
				if context != &all {
					t.Fatal("callback context changed")
				}
				var nested []float32
				_, ok := w.PreciseCastShape(Pos{}, &proxy, Vec3{X: 12}, DefaultQueryFilter(), 1e-6,
					func(_ ShapeID, _ Pos, _ Vec3, f float32, _ uint64, _, _ int, _ any) float32 {
						nested = append(nested, f)
						return 1
					}, nil)
				if !ok || !slices.Equal(nested, all) {
					t.Fatalf("nested cast changed: valid=%v got=%v want=%v", ok, nested, all)
				}
				got = append(got, f)
				return mode
			}, &all)
		if !valid || !slices.Equal(got, want) {
			t.Fatalf("mode %g: valid=%v got=%v want=%v", mode, valid, got, want)
		}
	}
}

func TestGolemQueryScratchConcurrentReaders(t *testing.T) {
	w, body := golemQueryWorld(t)
	sd := DefaultShapeDef()
	shape := body.CreateSphereShape(&sd, &Sphere{Radius: 1})
	var wg sync.WaitGroup
	for worker := range 8 {
		wg.Go(func() {
			proxy := ShapeProxy{Points: []Vec3{{X: .5 + float32(worker)*.1}}, Count: 1, Radius: .5}
			for range 100 {
				hit, found, valid := shape.ProxyPenetrationDeepest(Pos{}, &proxy, true)
				if !valid || !found || AbsFloat(hit.Depth-(1.5-proxy.Points[0].X)) > 1e-6 {
					t.Errorf("deepest scratch aliased: hit=%+v found=%v valid=%v", hit, found, valid)
					return
				}
				count := 0
				_, valid = w.PreciseCastShape(Pos{X: -3}, &proxy, Vec3{X: 6}, DefaultQueryFilter(), 1e-6,
					func(_ ShapeID, _ Pos, _ Vec3, _ float32, _ uint64, _, _ int, _ any) float32 { count++; return 1 }, nil)
				if !valid || count != 1 {
					t.Errorf("cast scratch aliased: valid=%v count=%d", valid, count)
					return
				}
			}
		})
	}
	wg.Wait()
}
