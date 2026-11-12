package box3d

import (
	"math"
	"math/rand/v2"
	"testing"
)

func TestGolemMeshScalarMatchesWideTransforms(t *testing.T) {
	rng := rand.New(rand.NewPCG(19, 71))
	value := func() float32 { return (rng.Float32() - .5) * 1000 }
	for iteration := 0; iteration < 2000; iteration++ {
		matrix := Matrix3{Vec3{value(), value(), value()}, Vec3{value(), value(), value()}, Vec3{value(), value(), value()}}
		translation := Vec3{value(), value(), value()}
		var lanes [3][4]Vec3
		for k := range lanes {
			for lane := range lanes[k] {
				lanes[k][lane] = Vec3{value(), value(), value()}
			}
		}
		var got [12]Vec3
		var indices [4]int
		var caches [4]triangleCache
		count := meshScalarSurvivors(matrix, translation, &lanes, 4, false, AABB{}, 0, caches[:], 0, indices[:], got[:], 0)
		if count != 4 {
			t.Fatal(count)
		}
		for k := 0; k < 3; k++ {
			ax := setW(lanes[k][0].X, lanes[k][1].X, lanes[k][2].X, lanes[k][3].X)
			ay := setW(lanes[k][0].Y, lanes[k][1].Y, lanes[k][2].Y, lanes[k][3].Y)
			az := setW(lanes[k][0].Z, lanes[k][1].Z, lanes[k][2].Z, lanes[k][3].Z)
			x := addW(addW(addW(mulW(splatW(matrix.Cx.X), ax), mulW(splatW(matrix.Cy.X), ay)), mulW(splatW(matrix.Cz.X), az)), splatW(translation.X))
			y := addW(addW(addW(mulW(splatW(matrix.Cx.Y), ax), mulW(splatW(matrix.Cy.Y), ay)), mulW(splatW(matrix.Cz.Y), az)), splatW(translation.Y))
			z := addW(addW(addW(mulW(splatW(matrix.Cx.Z), ax), mulW(splatW(matrix.Cy.Z), ay)), mulW(splatW(matrix.Cz.Z), az)), splatW(translation.Z))
			for lane := 0; lane < 4; lane++ {
				v := got[3*lane+k]
				if math.Float32bits(v.X) != math.Float32bits(laneW(x, lane)) || math.Float32bits(v.Y) != math.Float32bits(laneW(y, lane)) || math.Float32bits(v.Z) != math.Float32bits(laneW(z, lane)) {
					t.Fatalf("iteration %d vertex %d lane %d: scalar %+v wide %+v", iteration, k, lane, v, Vec3{laneW(x, lane), laneW(y, lane), laneW(z, lane)})
				}
			}
		}
	}
}

func TestGolemMeshScalarCullPreservesCacheAndOrder(t *testing.T) {
	var lanes [3][4]Vec3
	for k := 0; k < 3; k++ {
		lanes[k] = [4]Vec3{{X: -2}, {X: -1}, {X: 1}, {X: 2}}
	}
	var vertices [12]Vec3
	var indices [4]int
	var caches [4]triangleCache
	for i := range caches {
		caches[i].cache.satCache.Type = 1
	}
	count := meshScalarSurvivors(Matrix3{Cx: Vec3{X: 1}, Cy: Vec3{Y: 1}, Cz: Vec3{Z: 1}}, Vec3{}, &lanes, 4, true, AABB{LowerBound: Vec3{X: -1, Y: -1, Z: -1}, UpperBound: Vec3{X: 1, Y: 1, Z: 1}}, 0, caches[:], 0, indices[:], vertices[:], 0)
	if count != 2 || indices[0] != 1 || indices[1] != 2 || caches[0].cache.satCache.Type != 0 || caches[3].cache.satCache.Type != 0 || caches[1].cache.satCache.Type != 1 || caches[2].cache.satCache.Type != 1 {
		t.Fatalf("culling changed inclusive boundary/order/cache: count=%d indices=%v cache=%v", count, indices, caches)
	}
}
