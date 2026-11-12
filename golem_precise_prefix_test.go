package box3d

import (
	"fmt"
	"reflect"
	"slices"
	"testing"
)

type precisePrefixHit struct {
	shape           ShapeID
	point           Pos
	normal          Vec3
	fraction        float32
	material        uint64
	triangle, child int
}

// Retain the original full-sweep AABB/leaf traversal as an unpruned oracle.
// Only the already-tested geometric leaf solver is shared with production.
func precisePrefixOracle(w WorldID, origin Pos, proxy *ShapeProxy, translation Vec3, filter QueryFilter) ([]precisePrefixHit, TreeStats, bool) {
	const tolerance = float32(1e-6)
	bounds := preciseProxyBounds(proxy, translation, tolerance)
	worldBounds := AABB{Add(ToVec3(origin), bounds.LowerBound), Add(ToVec3(origin), bounds.UpperBound)}
	padding := 4 * fltEpsilon * MaxFloat(1, MaxFloat(AbsFloat(origin.X), MaxFloat(AbsFloat(origin.Y), AbsFloat(origin.Z))))
	worldBounds.LowerBound = Sub(worldBounds.LowerBound, Vec3{padding, padding, padding})
	worldBounds.UpperBound = Add(worldBounds.UpperBound, Vec3{padding, padding, padding})
	valid := true
	var hits []precisePrefixHit
	stats := w.OverlapAABB(worldBounds, filter, func(id ShapeID, _ any) bool {
		s, transform, ok := queryShape(id, origin)
		if !ok {
			valid = false
			return false
		}
		best, child := CastOutput{Fraction: 1, TriangleIndex: -1}, int32(-1)
		visitQueryGeometry(s, transform, bounds, func(leaf queryLeaf) {
			out, ok := preciseLeafCast(leaf, proxy, translation, tolerance)
			if !ok {
				valid = false
				return
			}
			if !out.Hit {
				return
			}
			if leaf.triangleIndex >= 0 && !filter.BackFaces {
				normal := RotateVector(leaf.transform.Q, Normalize(Cross(Sub(leaf.triangle[1], leaf.triangle[0]), Sub(leaf.triangle[2], leaf.triangle[0]))))
				if Dot(normal, out.Normal) < -fltEpsilon {
					return
				}
			}
			if !best.Hit || out.Fraction < best.Fraction || out.Fraction == best.Fraction && (leaf.child < child || leaf.child == child && leaf.triangleIndex < best.TriangleIndex) {
				best, child = out, leaf.child
				best.TriangleIndex = leaf.triangleIndex
			}
		})
		if best.Hit {
			hits = append(hits, precisePrefixHit{id, OffsetPos(origin, best.Point), best.Normal, best.Fraction, getShapeUserMaterialId(s, int(child), int(best.TriangleIndex)), int(best.TriangleIndex), int(child)})
		}
		return valid
	}, nil)
	return hits, stats, valid
}

func precisePrefixLess(a, b precisePrefixHit) int {
	if a.fraction < b.fraction {
		return -1
	}
	if a.fraction > b.fraction {
		return 1
	}
	if a.shape.Index1 < b.shape.Index1 {
		return -1
	}
	if a.shape.Index1 > b.shape.Index1 {
		return 1
	}
	if a.child < b.child {
		return -1
	}
	if a.child > b.child {
		return 1
	}
	if a.triangle < b.triangle {
		return -1
	}
	if a.triangle > b.triangle {
		return 1
	}
	return 0
}

func precisePrefixNearest(hits []precisePrefixHit) []precisePrefixHit {
	slices.SortFunc(hits, precisePrefixLess)
	if len(hits) == 0 {
		return hits
	}
	i := 1
	for i < len(hits) && hits[i].fraction == hits[0].fraction {
		i++
	}
	return hits[:i]
}

func TestGolemPrecisePrefixMatchesUnculledGeometry(t *testing.T) {
	for _, offset := range []float32{0, 768, 2304} {
		t.Run(fmt.Sprint(offset), func(t *testing.T) {
			w, _ := golemQueryWorld(t)
			body := func(kind BodyType, x, y float32) BodyID {
				d := DefaultBodyDef()
				d.Type, d.Position = kind, Pos{offset + x, offset + y, offset}
				return w.CreateBody(&d)
			}
			sd := DefaultShapeDef()
			sd.BaseMaterial.UserMaterialId = 17
			hull := MakeBoxHull(.5, .5, .5)
			// Equal witnesses on different bodies and trees must both survive clipping.
			body(StaticBody, 0, .5).CreateHullShape(&sd, &hull.Base)
			body(KinematicBody, 0, .5).CreateHullShape(&sd, &hull.Base)
			mesh := golemFlatMesh()
			m0, m1 := DefaultSurfaceMaterial(), DefaultSurfaceMaterial()
			m0.UserMaterialId, m1.UserMaterialId = 41, 42
			sd.Materials, sd.MaterialCount = []SurfaceMaterial{m0, m1}, 2
			body(StaticBody, 3, 0).CreateMeshShape(&sd, mesh, Vec3One)
			body(StaticBody, 3, -2).CreateMeshShape(&sd, mesh, Vec3One)
			cd := CompoundDef{Meshes: []CompoundMeshDef{
				{MeshData: mesh, Transform: TransformIdentity, Scale: Vec3One, Materials: sd.Materials, MaterialCount: 2},
				{MeshData: mesh, Transform: TransformIdentity, Scale: Vec3One, Materials: sd.Materials, MaterialCount: 2},
				{MeshData: mesh, Transform: Transform{P: Vec3{Y: -2}, Q: QuatIdentity}, Scale: Vec3One, Materials: sd.Materials, MaterialCount: 2},
			}, MeshCount: 3}
			body(StaticBody, 8, 0).CreateBakedCompoundShape(&sd, CreateCompound(&cd))
			box := MakeBoxHull(.2, .2, .2)
			proxies := []ShapeProxy{
				{Points: []Vec3{{}}, Count: 1, Radius: .2},
				{Points: []Vec3{{}, {Y: .4}}, Count: 2, Radius: .2},
				{Points: GetHullPoints(&box.Base), Count: 8},
			}
			for _, x := range []float32{0, 3, 8} {
				for i := range proxies {
					proxy := &proxies[i]
					for _, upward := range []bool{false, true} {
						for _, backfaces := range []bool{false, true} {
							origin, translation := Pos{offset + x, offset + 3, offset}, Vec3{Y: -6}
							if upward {
								origin.Y, translation.Y = offset-3, 6
							}
							filter := DefaultQueryFilter()
							filter.BackFaces = backfaces
							want, _, valid := precisePrefixOracle(w, origin, proxy, translation, filter)
							if !valid {
								t.Fatal("oracle did not converge")
							}
							for _, nearest := range []bool{false, true} {
								var got []precisePrefixHit
								_, valid = w.PreciseCastShape(origin, proxy, translation, filter, 1e-6, func(id ShapeID, point Pos, normal Vec3, f float32, material uint64, triangle, child int, _ any) float32 {
									got = append(got, precisePrefixHit{id, point, normal, f, material, triangle, child})
									if nearest {
										return f
									}
									return 1
								}, nil)
								expected := append([]precisePrefixHit(nil), want...)
								if nearest {
									got, expected = precisePrefixNearest(got), precisePrefixNearest(expected)
								} else {
									slices.SortFunc(got, precisePrefixLess)
									slices.SortFunc(expected, precisePrefixLess)
								}
								if !valid || !reflect.DeepEqual(got, expected) {
									t.Fatalf("x=%g proxy=%d upward=%v backfaces=%v nearest=%v valid=%v\ngot=%+v\nwant=%+v", x, i, upward, backfaces, nearest, valid, got, expected)
								}
							}
						}
					}
				}
			}
		})
	}
}

func TestGolemPrecisePrefixPrunesWithoutDroppingTies(t *testing.T) {
	w, _ := golemQueryWorld(t)
	sd := DefaultShapeDef()
	for i := range 64 {
		d := DefaultBodyDef()
		d.Position.X = 3 + float32(max(i-1, 0))*3
		w.CreateBody(&d).CreateSphereShape(&sd, &Sphere{Radius: 1})
	}
	proxy := ShapeProxy{Points: []Vec3{{}}, Count: 1, Radius: .5}
	translation := Vec3{X: 200}
	want, unculled, valid := precisePrefixOracle(w, Pos{}, &proxy, translation, DefaultQueryFilter())
	if !valid {
		t.Fatal("oracle did not converge")
	}
	var got []precisePrefixHit
	stats, valid := w.PreciseCastShape(Pos{}, &proxy, translation, DefaultQueryFilter(), 1e-6, func(id ShapeID, point Pos, normal Vec3, f float32, material uint64, triangle, child int, _ any) float32 {
		got = append(got, precisePrefixHit{id, point, normal, f, material, triangle, child})
		return f
	}, nil)
	got, want = precisePrefixNearest(got), precisePrefixNearest(want)
	if !valid || len(got) != 2 || !reflect.DeepEqual(got, want) || stats.LeafVisits >= unculled.LeafVisits {
		t.Fatalf("valid=%v clipped=%+v full=%+v got=%+v want=%+v", valid, stats, unculled, got, want)
	}
	t.Logf("candidate leaves: clipped=%d unculled=%d; equal-fraction hits=%d", stats.LeafVisits, unculled.LeafVisits, len(got))
	for _, kind := range []BodyType{KinematicBody, DynamicBody} {
		d := DefaultBodyDef()
		d.Type, d.Position.X = kind, 3
		w.CreateBody(&d).CreateSphereShape(&sd, &Sphere{Radius: 1})
	}
	// Zero translation and an initial hit retain exact geometric witnesses. A
	// callback's zero terminates; zero-fraction geometry alone does not terminate.
	for _, translation := range []Vec3{{}, {X: 200}} {
		origin := Pos{X: 3}
		want, _, valid := precisePrefixOracle(w, origin, &proxy, translation, DefaultQueryFilter())
		if !valid {
			t.Fatal("initial oracle did not converge")
		}
		var got []precisePrefixHit
		_, valid = w.PreciseCastShape(origin, &proxy, translation, DefaultQueryFilter(), 1e-6, func(id ShapeID, point Pos, normal Vec3, f float32, material uint64, triangle, child int, _ any) float32 {
			got = append(got, precisePrefixHit{id, point, normal, f, material, triangle, child})
			return 1
		}, nil)
		slices.SortFunc(got, precisePrefixLess)
		slices.SortFunc(want, precisePrefixLess)
		if !valid || !reflect.DeepEqual(got, want) {
			t.Fatalf("initial/zero cast changed: got=%+v want=%+v", got, want)
		}
		count := 0
		_, valid = w.PreciseCastShape(origin, &proxy, translation, DefaultQueryFilter(), 1e-6, func(_ ShapeID, _ Pos, _ Vec3, _ float32, _ uint64, _, _ int, _ any) float32 { count++; return 0 }, nil)
		if !valid || count != 1 {
			t.Fatalf("zero callback did not stop: valid=%v count=%d", valid, count)
		}
	}
}
