// Port of src/distance.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.
//
// Dirk Gregorius contributed portions of this code

package box3d

const (
	maxSimplexVertices = 4  // B3_MAX_SIMPLEX_VERTICES
	maxGJKIterations   = 32 // B3_MAX_GJK_ITERATIONS
)

// getProxySupport is b3GetProxySupport.
func getProxySupport(proxy *ShapeProxy, axis Vec3) int {
	count := int(proxy.Count)
	points := proxy.Points

	assert(count > 0, "count > 0")
	assert(points != nil, "points != NULL")

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and
	// vertices can potentially be far away from the origin (large).
	origin := points[0]
	maxIndex := 0
	var maxProjection float32 = 0.0

	for index := 1; index < count; index++ {
		// We subtract the first vertex since we are shifting into the origin.
		projection := Dot(axis, Sub(points[index], origin))
		if projection > maxProjection {
			maxIndex = index
			maxProjection = projection
		}
	}

	return maxIndex
}

// getPointSupport is b3GetPointSupport.
// PORT: the points pointer becomes a slice; count is kept because proxies carry their own count.
func getPointSupport(points []Vec3, count int, axis Vec3) int {
	assert(count > 0, "count > 0")
	assert(points != nil, "points != NULL")

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and
	// vertices can potentially be far away from the origin (large).
	origin := points[0]
	maxIndex := 0
	var maxProjection float32 = 0.0

	for index := 1; index < count; index++ {
		// We subtract the first vertex since we are shifting into the origin.
		projection := Dot(axis, Sub(points[index], origin))
		if projection > maxProjection {
			maxIndex = index
			maxProjection = projection
		}
	}

	return maxIndex
}

func barycentricCoordsEdge(out *[3]float32, a, b Vec3) {
	ab := Sub(b, a)

	// Last element is divisor
	divisor := Dot(ab, ab)

	out[0] = Dot(b, ab)
	out[1] = -Dot(a, ab)
	out[2] = divisor
}

func barycentricCoordsTri(out *[4]float32, a, b, c Vec3) {
	ab := Sub(b, a)
	ac := Sub(c, a)

	bXC := Cross(b, c)
	cXA := Cross(c, a)
	aXB := Cross(a, b)

	abXAc := Cross(ab, ac)

	// Last element is divisor
	divisor := Dot(abXAc, abXAc)

	out[0] = Dot(bXC, abXAc)
	out[1] = Dot(cXA, abXAc)
	out[2] = Dot(aXB, abXAc)
	out[3] = divisor
}

func barycentricCoordsTet(out *[5]float32, a, b, c, d Vec3) {
	ab := Sub(b, a)
	ac := Sub(c, a)
	ad := Sub(d, a)

	// Last element is divisor (forced to be positive)
	divisor := scalarTripleProduct(ab, ac, ad)

	var sign float32 = 1.0
	if divisor < 0.0 {
		sign = -1.0
	}
	out[0] = sign * scalarTripleProduct(b, c, d)
	out[1] = sign * scalarTripleProduct(a, d, c)
	out[2] = sign * scalarTripleProduct(a, b, d)
	out[3] = sign * scalarTripleProduct(a, c, b)
	out[4] = sign * divisor
}

func getMetric(simplex *Simplex) float32 {
	count := simplex.Count
	assert(1 <= count && count <= 4, "1 <= count && count <= 4")

	vertices := &simplex.Vertices

	switch count {
	case 1:
		return 0.0

	case 2:
		a := vertices[0].W
		b := vertices[1].W
		return Distance(a, b)

	case 3:
		a := vertices[0].W
		b := vertices[1].W
		c := vertices[2].W
		return Length(Cross(Sub(b, a), Sub(c, a))) / 2.0

	case 4:
		a := vertices[0].W
		b := vertices[1].W
		c := vertices[2].W
		d := vertices[3].W
		return scalarTripleProduct(Sub(b, a), Sub(c, a), Sub(d, a)) / 6.0

	default:
		assert(false, "!\"Should never get here!\"")
	}

	return 0.0
}

func writeCache(cache *SimplexCache, simplex *Simplex) {
	count := int(simplex.Count)
	cache.Metric = getMetric(simplex)
	cache.Count = uint16(count)
	for index := range count {
		cache.IndexA[index] = uint8(simplex.Vertices[index].IndexA)
		cache.IndexB[index] = uint8(simplex.Vertices[index].IndexB)
	}
}

func solveSimplex2(simplex *Simplex) bool {
	vs := &simplex.Vertices
	assert(simplex.Count == 2, "simplex->count == 2")

	// Vertex regions
	//float wAB[3];

	a := vs[0].W
	b := vs[1].W
	ab := Sub(b, a)

	// Last element is divisor
	divisor := Dot(ab, ab)

	u := Dot(b, ab)
	v := -Dot(a, ab)
	//wAB[2] = divisor;

	// V( A )
	if v <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0].A = 1.0

		return true
	}

	// V( B )
	if u <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0] = vs[1]
		vs[0].A = 1.0

		return true
	}

	// Edge region
	if divisor <= 0.0 {
		return false
	}

	// VR( AB )
	denominator := 1.0 / divisor
	vs[0].A = denominator * u
	vs[1].A = denominator * v

	return true
}

func solveSimplex3(simplex *Simplex) bool {
	vs := &simplex.Vertices
	assert(simplex.Count == 3, "simplex->count == 3")

	// Get simplex (be aware of aliasing here!)
	v1 := vs[0]
	v2 := vs[1]
	v3 := vs[2]

	// Vertex regions
	var wAB, wBC, wCA [3]float32
	barycentricCoordsEdge(&wAB, v1.W, v2.W)
	barycentricCoordsEdge(&wBC, v2.W, v3.W)
	barycentricCoordsEdge(&wCA, v3.W, v1.W)

	// VR( A )
	if wAB[1] <= 0.0 && wCA[0] <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0] = v1
		vs[0].A = 1.0

		return true
	}

	// VR( B )
	if wBC[1] <= 0.0 && wAB[0] <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0] = v2
		vs[0].A = 1.0

		return true
	}

	// VR( C )
	if wCA[1] <= 0.0 && wBC[0] <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0] = v3
		vs[0].A = 1.0

		return true
	}

	// Edge regions
	var wABC [4]float32
	barycentricCoordsTri(&wABC, v1.W, v2.W, v3.W)

	// VR( AB )
	if wABC[2] <= 0.0 && wAB[0] > 0.0 && wAB[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = v1
		vs[1] = v2

		// Normalize
		divisor := wAB[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wAB[0] / divisor
		vs[1].A = wAB[1] / divisor

		return true
	}

	// VR( BC )
	if wABC[0] <= 0.0 && wBC[0] > 0.0 && wBC[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = v2
		vs[1] = v3

		// Normalize
		divisor := wBC[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wBC[0] / divisor
		vs[1].A = wBC[1] / divisor

		return true
	}

	// VR( CA )
	if wABC[1] <= 0.0 && wCA[0] > 0.0 && wCA[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = v3
		vs[1] = v1

		// Normalize
		divisor := wCA[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wCA[0] / divisor
		vs[1].A = wCA[1] / divisor

		return true
	}

	// Face region
	divisor := wABC[3]
	if divisor <= 0.0 {
		return false
	}

	// VR( ABC )
	vs[0].A = wABC[0] / divisor
	vs[1].A = wABC[1] / divisor
	vs[2].A = wABC[2] / divisor

	return true
}

func solveSimplex4(simplex *Simplex) bool {
	vs := &simplex.Vertices

	// Get simplex (be aware of aliasing here!)
	assert(simplex.Count == 4, "simplex->count == 4")
	vertexA := vs[0]
	vertexB := vs[1]
	vertexC := vs[2]
	vertexD := vs[3]

	// Vertex region
	var wAB, wAC, wAD, wBC, wCD, wDB [3]float32
	barycentricCoordsEdge(&wAB, vertexA.W, vertexB.W)
	barycentricCoordsEdge(&wAC, vertexA.W, vertexC.W)
	barycentricCoordsEdge(&wAD, vertexA.W, vertexD.W)
	barycentricCoordsEdge(&wBC, vertexB.W, vertexC.W)
	barycentricCoordsEdge(&wCD, vertexC.W, vertexD.W)
	barycentricCoordsEdge(&wDB, vertexD.W, vertexB.W)

	// VR( A )
	if wAB[1] <= 0.0 && wAC[1] <= 0.0 && wAD[1] <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0] = vertexA

		vs[0].A = 1.0

		return true
	}

	// VR( B )
	if wAB[0] <= 0.0 && wDB[0] <= 0.0 && wBC[1] <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0] = vertexB

		vs[0].A = 1.0

		return true
	}

	// VR( C )
	if wAC[0] <= 0.0 && wBC[0] <= 0.0 && wCD[1] <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0] = vertexC

		vs[0].A = 1.0

		return true
	}

	// VR( D )
	if wAD[0] <= 0.0 && wCD[0] <= 0.0 && wDB[1] <= 0.0 {
		// Reduce simplex
		simplex.Count = 1
		vs[0] = vertexD

		vs[0].A = 1.0

		return true
	}

	// Edge region
	var wACB, wABD, wADC, wBCD [4]float32
	barycentricCoordsTri(&wACB, vertexA.W, vertexC.W, vertexB.W)
	barycentricCoordsTri(&wABD, vertexA.W, vertexB.W, vertexD.W)
	barycentricCoordsTri(&wADC, vertexA.W, vertexD.W, vertexC.W)
	barycentricCoordsTri(&wBCD, vertexB.W, vertexC.W, vertexD.W)

	// VR( AB )
	if wABD[2] <= 0.0 && wACB[1] <= 0.0 && wAB[0] > 0.0 && wAB[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = vertexA
		vs[1] = vertexB

		// Normalize
		divisor := wAB[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wAB[0] / divisor
		vs[1].A = wAB[1] / divisor

		return true
	}

	// VR( AC )
	if wACB[2] <= 0.0 && wADC[1] <= 0.0 && wAC[0] > 0.0 && wAC[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = vertexA
		vs[1] = vertexC

		// Normalize
		divisor := wAC[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wAC[0] / divisor
		vs[1].A = wAC[1] / divisor

		return true
	}

	// VR( AD )
	if wADC[2] <= 0.0 && wABD[1] <= 0.0 && wAD[0] > 0.0 && wAD[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = vertexA
		vs[1] = vertexD

		// Normalize
		divisor := wAD[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wAD[0] / divisor
		vs[1].A = wAD[1] / divisor

		return true
	}

	// VR( BC )
	if wACB[0] <= 0.0 && wBCD[2] <= 0.0 && wBC[0] > 0.0 && wBC[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = vertexB
		vs[1] = vertexC

		// Normalize
		divisor := wBC[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wBC[0] / divisor
		vs[1].A = wBC[1] / divisor

		return true
	}

	// VR( CD )
	if wADC[0] <= 0.0 && wBCD[0] <= 0.0 && wCD[0] > 0.0 && wCD[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = vertexC
		vs[1] = vertexD

		// Normalize
		divisor := wCD[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wCD[0] / divisor
		vs[1].A = wCD[1] / divisor

		return true
	}

	// VR( DB )
	if wABD[0] <= 0.0 && wBCD[1] <= 0.0 && wDB[0] > 0.0 && wDB[1] > 0.0 {
		// Reduce simplex
		simplex.Count = 2
		vs[0] = vertexD
		vs[1] = vertexB

		// Normalize
		divisor := wDB[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wDB[0] / divisor
		vs[1].A = wDB[1] / divisor

		return true
	}

	// Face regions
	var wABCD [5]float32
	barycentricCoordsTet(&wABCD, vertexA.W, vertexB.W, vertexC.W, vertexD.W)

	// VR( ACB )
	if wABCD[3] < 0.0 && wACB[0] > 0.0 && wACB[1] > 0.0 && wACB[2] > 0.0 {
		// Reduce simplex
		simplex.Count = 3
		vs[0] = vertexA
		vs[1] = vertexC
		vs[2] = vertexB

		// Normalize
		divisor := wACB[3]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wACB[0] / divisor
		vs[1].A = wACB[1] / divisor
		vs[2].A = wACB[2] / divisor

		return true
	}

	// VR( ABD )
	if wABCD[2] < 0.0 && wABD[0] > 0.0 && wABD[1] > 0.0 && wABD[2] > 0.0 {
		// Reduce simplex
		simplex.Count = 3
		vs[0] = vertexA
		vs[1] = vertexB
		vs[2] = vertexD

		// Normalize
		divisor := wABD[3]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wABD[0] / divisor
		vs[1].A = wABD[1] / divisor
		vs[2].A = wABD[2] / divisor

		return true
	}

	// VR( ADC )
	if wABCD[1] < 0.0 && wADC[0] > 0.0 && wADC[1] > 0.0 && wADC[2] > 0.0 {
		// Reduce simplex
		simplex.Count = 3
		vs[0] = vertexA
		vs[1] = vertexD
		vs[2] = vertexC

		// Normalize
		divisor := wADC[3]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wADC[0] / divisor
		vs[1].A = wADC[1] / divisor
		vs[2].A = wADC[2] / divisor

		return true
	}

	// VR( BCD )
	if wABCD[0] < 0.0 && wBCD[0] > 0.0 && wBCD[1] > 0.0 && wBCD[2] > 0.0 {
		// Reduce simplex
		simplex.Count = 3
		vs[0] = vertexB
		vs[1] = vertexC
		vs[2] = vertexD

		// Normalize
		divisor := wBCD[3]
		if divisor <= 0.0 {
			return false
		}

		vs[0].A = wBCD[0] / divisor
		vs[1].A = wBCD[1] / divisor
		vs[2].A = wBCD[2] / divisor

		return true
	}

	// *** Inside tetrahedron ***
	divisor := wABCD[4]
	if divisor <= 0.0 {
		return false
	}

	// VR( ABCD )
	vs[0].A = wABCD[0] / divisor
	vs[1].A = wABCD[1] / divisor
	vs[2].A = wABCD[2] / divisor
	vs[3].A = wABCD[3] / divisor

	return true
}

// computeWitnessPoints is b3ComputeWitnessPoints.
// PORT: the out-parameters become return values.
func computeWitnessPoints(simplex *Simplex) (vertexA, vertexB Vec3) {
	vs := &simplex.Vertices
	count := simplex.Count
	assert(1 <= count && count <= 4, "1 <= count && count <= 4")

	switch count {
	case 1:
		vertexA = vs[0].WA
		vertexB = vs[0].WB

	case 2:
		vertexA = Blend2(vs[0].A, vs[0].WA, vs[1].A, vs[1].WA)
		vertexB = Blend2(vs[0].A, vs[0].WB, vs[1].A, vs[1].WB)

	case 3:
		vertexA = blend3(vs[0].A, vs[0].WA, vs[1].A, vs[1].WA, vs[2].A, vs[2].WA)
		vertexB = blend3(vs[0].A, vs[0].WB, vs[1].A, vs[1].WB, vs[2].A, vs[2].WB)

	case 4:
		{
			// Force identical points and *zero* distance
			sum := Add(Blend2(vs[0].A, vs[0].WA, vs[1].A, vs[1].WA),
				Blend2(vs[2].A, vs[2].WA, vs[3].A, vs[3].WA))
			vertexA = sum
			vertexB = sum
		}

	default:
		assert(false, "!\"Should never get here!\"")
	}

	return vertexA, vertexB
}

// ShapeDistance computes the closest points between two shapes represented as point clouds.
// SimplexCache cache is input/output. On the first call set SimplexCache.Count to zero.
// The query runs in frame A, so the witness points and normal are returned in frame A.
// The underlying GJK algorithm may be debugged by passing in debug simplexes. You may pass in nil.
// PORT: simplexes + simplexCapacity become one slice; the capacity is len(simplexes).
func ShapeDistance(input *DistanceInput, cache *SimplexCache, simplexes []Simplex) DistanceOutput {
	simplexCapacity := len(simplexes)

	// The query runs in frame A using the relative pose of B in A.
	xf := input.Transform

	// Use matrices for faster math
	m := MakeMatrixFromQuat(xf.Q)
	mt := Transpose(m)

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	// Compute initial simplex from cache
	assert(cache.Count <= maxSimplexVertices, "cache->count <= B3_MAX_SIMPLEX_VERTICES")

	simplex := Simplex{}
	vs := &simplex.Vertices

	simplex.Count = int32(cache.Count)
	for i := 0; i < int(cache.Count); i++ {
		index1 := int(cache.IndexA[i])
		index2 := int(cache.IndexB[i])

		assert(0 <= index1 && index1 < int(proxyA.Count), "0 <= index1 && index1 < proxyA->count")
		assert(0 <= index2 && index2 < int(proxyB.Count), "0 <= index2 && index2 < proxyB->count")

		vertex1 := proxyA.Points[index1]
		vertex2 := Add(MulMV(m, proxyB.Points[index2]), xf.P)

		vs[i].IndexA = int32(index1)
		vs[i].IndexB = int32(index2)
		vs[i].WA = vertex1
		vs[i].WB = vertex2
		vs[i].W = Sub(vertex2, vertex1)
		vs[i].A = 0.0
	}

	// Compute the new simplex metric, if it is substantially
	// different than the old metric flush the simplex.
	if simplex.Count > 0 {
		metric1 := cache.Metric
		metric2 := getMetric(&simplex)

		// todo the tetrahedron metric can be negative
		if 2.0*metric1 < metric2 || metric2 < 0.5*metric1 || metric2 < fltEpsilon {
			// Flush the simplex
			simplex.Count = 0
		}
	}

	// If the cache is invalid or empty
	if simplex.Count == 0 {
		vertex1 := proxyA.Points[0]
		vertex2 := Add(MulMV(m, proxyB.Points[0]), xf.P)

		simplex.Count = 1
		simplex.Vertices[0].IndexA = 0
		simplex.Vertices[0].IndexB = 0
		simplex.Vertices[0].WA = vertex1
		simplex.Vertices[0].WB = vertex2
		simplex.Vertices[0].W = Sub(vertex2, vertex1)
		simplex.Vertices[0].A = 0.0
	}

	backup := Simplex{}

	simplexIndex := 0
	if simplexes != nil && simplexIndex < simplexCapacity {
		simplexes[simplexIndex] = simplex
		simplexIndex += 1
	}

	distanceOutput := DistanceOutput{}

	// Keep track of squared distance
	distanceSq := fltMax

	normal := Vec3Zero

	// Run GJK
	iteration := 0
	for ; iteration < maxGJKIterations; iteration++ {
		// Solve simplex
		solved := false
		switch simplex.Count {
		case 1:
			simplex.Vertices[0].A = 1.0
			solved = true

		case 2:
			solved = solveSimplex2(&simplex)

		case 3:
			solved = solveSimplex3(&simplex)

		case 4:
			solved = solveSimplex4(&simplex)

		default:
			assert(false, "!\"Should never get here!\"")
		}

		if solved == false {
			// No progress - reconstruct last simplex
			assert(backup.Count != 0, "backup.count != 0")
			simplex = backup
			break
		}

		if simplexes != nil && simplexIndex < simplexCapacity {
			simplexes[simplexIndex] = simplex
			simplexIndex += 1
			distanceOutput.Iterations = int32(iteration)
			distanceOutput.SimplexCount = int32(simplexIndex)
		}

		if simplex.Count == maxSimplexVertices {
			// Overlap
			localPointA, localPointB := computeWitnessPoints(&simplex)
			distanceOutput.PointA = localPointA
			distanceOutput.PointB = localPointB
			distanceOutput.Normal = Vec3Zero
			distanceOutput.Distance = 0.0
			distanceOutput.Iterations = int32(iteration)
			distanceOutput.SimplexCount = int32(simplexIndex)
			return distanceOutput
		}

		// Assure distance progression
		oldDistanceSq := distanceSq

		// Compute closest point
		closestPoint := Vec3{}

		switch simplex.Count {
		case 1:
			closestPoint = vs[0].W

		case 2:
			closestPoint = Blend2(vs[0].A, vs[0].W, vs[1].A, vs[1].W)

		case 3:
			closestPoint = blend3(vs[0].A, vs[0].W, vs[1].A, vs[1].W, vs[2].A, vs[2].W)

		case 4:
			closestPoint = Add(Blend2(vs[0].A, vs[0].W, vs[1].A, vs[1].W),
				Blend2(vs[2].A, vs[2].W, vs[3].A, vs[3].W))

		default:
			assert(false, "!\"Should never get here!\"")
		}

		distanceSq = Dot(closestPoint, closestPoint)

		if distanceSq >= oldDistanceSq {
			// No progress - reconstruct last simplex
			assert(backup.Count != 0, "backup.count != 0")
			simplex = backup
			break
		}

		// Build new tentative support point
		searchDirection := Vec3{}

		switch simplex.Count {
		case 1:
			{
				// v = -A
				searchDirection = Neg(vs[0].W)
			}

		case 2:
			{
				// v = (AB x AO) x AB
				a := vs[0].W
				b := vs[1].W

				ab := Sub(b, a)

				searchDirection = Cross(Cross(ab, Neg(a)), ab)
			}

		case 3:
			{
				// v = AB x AC or v = AC x AB
				a := vs[0].W
				b := vs[1].W
				c := vs[2].W

				ab := Sub(b, a)
				ac := Sub(c, a)

				n := Cross(ab, ac)

				if Dot(n, a) < 0.0 {
					searchDirection = n
				} else {
					searchDirection = Neg(n)
				}
			}

		default:
			assert(false, "!\"Should never get here!\"")
		}

		if LengthSquared(searchDirection) < 1000.0*fltMin {
			// The origin is probably contained by a line segment or triangle.
			// Thus the shapes are overlapped.
			localPointA, localPointB := computeWitnessPoints(&simplex)
			distanceOutput.PointA = localPointA
			distanceOutput.PointB = localPointB
			distanceOutput.Normal = Vec3Zero
			distanceOutput.Distance = 0.0
			distanceOutput.Iterations = int32(iteration)
			distanceOutput.SimplexCount = int32(simplexIndex)
			validate(Distance(localPointA, localPointB) < fltEpsilon, "b3Distance( localPointA, localPointB ) < FLT_EPSILON")
			return distanceOutput
		}

		normal = Neg(searchDirection)

		// Get new support points
		searchDirection1 := searchDirection
		indexA := getProxySupport(&input.ProxyA, Neg(searchDirection1))
		supportA := input.ProxyA.Points[indexA]
		searchDirection2 := MulMV(mt, searchDirection)
		indexB := getProxySupport(&input.ProxyB, searchDirection2)
		supportB := Add(MulMV(m, input.ProxyB.Points[indexB]), xf.P)

		// Save current simplex and add new vertex - this can fail if we detect cycling
		backup = simplex

		// Check for duplicate support points. This is the main termination criteria.
		duplicate := false
		for i := 0; i < int(simplex.Count); i++ {
			if int(vs[i].IndexA) == indexA && int(vs[i].IndexB) == indexB {
				duplicate = true
				break
			}
		}

		if duplicate {
			break
		}

		vs[simplex.Count].IndexA = int32(indexA)
		vs[simplex.Count].IndexB = int32(indexB)
		vs[simplex.Count].WA = supportA
		vs[simplex.Count].WB = supportB
		vs[simplex.Count].W = Sub(supportB, supportA)
		simplex.Count += 1
	}

	// Build witness points.
	localPointA, localPointB := computeWitnessPoints(&simplex)

	distanceOutput.PointA = localPointA
	distanceOutput.PointB = localPointB
	distanceOutput.Iterations = int32(iteration)
	distanceOutput.SimplexCount = int32(simplexIndex)

	normal = Normalize(normal)
	if IsNormalized(normal) == false {
		// Treat as overlap
		distanceOutput.Distance = 0.0
		distanceOutput.Normal = Vec3Zero
		return distanceOutput
	}

	// Results stay in frame A
	distanceOutput.Distance = Distance(localPointA, localPointB)
	distanceOutput.Normal = normal

	// Apply radii if requested
	if input.UseRadii {
		rA := input.ProxyA.Radius
		rB := input.ProxyB.Radius
		distanceOutput.Distance = MaxFloat(0.0, distanceOutput.Distance-rA-rB)

		// Keep closest points on perimeter even if overlapped, this way the points move smoothly.
		distanceOutput.PointA = Add(distanceOutput.PointA, MulSV(rA, normal))
		distanceOutput.PointB = Sub(distanceOutput.PointB, MulSV(rB, normal))
	}

	// Simplex is useful, cache it.
	writeCache(cache, &simplex)

	return distanceOutput
}

// Separation function:
// f(t) = (c2 + t * dp2 - c1 - t * dp1 ) * n

// Root finding : f(t) - target = 0
// (c2 + t * dp2 - c1 - t * dp1 ) * n - target = 0
// (c2 - c1) * n + t * (dp2 - dp1) * n - target = 0
// t = [target - (c2 - c1) * n] / [(dp2 - dp1) * n]
// t = (target - d) / [(dp2 - dp1) * n]

// ShapeCast performs a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and
// translation fraction. The query runs in frame A, so the hit point and normal are returned in frame A. Initially
// overlapping shapes return a hit at fraction zero with a zero normal, except
// when CanEncroach permits advancing within the retained rest separation.
func ShapeCast(input *ShapeCastPairInput) CastOutput {
	// Compute tolerance
	linearSlop := LinearSlop()
	totalRadius := input.ProxyA.Radius + input.ProxyB.Radius
	target := MaxFloat(linearSlop, totalRadius-linearSlop)
	tolerance := float32(0.25 * linearSlop)

	assert(target > tolerance, "target > tolerance")

	// Prepare input for distance query
	cache := SimplexCache{}

	var alpha float32 = 0.0

	distanceInput := DistanceInput{}
	distanceInput.ProxyA = input.ProxyA
	distanceInput.ProxyB = input.ProxyB
	distanceInput.UseRadii = false

	// The whole cast runs in frame A. Advance the relative pose of B in float each iteration,
	// which keeps the math near the local origin and avoids re-relativizing world poses.
	distanceInput.Transform = input.Transform

	delta2 := input.TranslationB
	distanceOutput := DistanceOutput{}
	output := CastOutput{}
	output.TriangleIndex = NullIndex

	iteration := 0
	const maxIterations = 20

	for ; iteration < maxIterations; iteration++ {
		output.Iterations += 1

		distanceOutput = ShapeDistance(&distanceInput, &cache, nil)

		if distanceOutput.Distance < target+tolerance {
			if iteration == 0 {
				if input.CanEncroach && distanceOutput.Distance > 2.0*linearSlop {
					target = distanceOutput.Distance - linearSlop
				} else if input.CanEncroach && distanceOutput.Distance > 0.0 {
					// This is for box movers and can't normally be reached by capsule
					// movers or regular shape casts.

					// Near-target start without overlap: advance by first order while
					// retaining a rest separation. Sweeps that cannot consume the free
					// gap (receding, tangential, grazing) miss.
					retained := float32(0.5 * linearSlop)
					needed := distanceOutput.Distance - retained
					approach := -Dot(delta2, distanceOutput.Normal)
					if approach <= MaxFloat(needed, 0.0) {
						// Moving away, no hit.
						return output
					}

					var fraction float32 = 0.0
					if needed > 0.0 {
						fraction = needed / approach
					}
					if fraction >= input.MaxFraction {
						// Still no hit because the max fraction is small.
						return output
					}

					output.Fraction = fraction
					output.Point = MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
					output.Normal = distanceOutput.Normal
					output.Hit = true
					return output
				} else {
					// Initial overlap
					output.Hit = true

					// Compute a common point
					c1 := MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
					c2 := MulAdd(distanceOutput.PointB, -input.ProxyB.Radius, distanceOutput.Normal)
					output.Point = Lerp(c1, c2, 0.5)
					return output
				}
			} else {
				// Logging for bad input data
				if distanceOutput.Distance > 0.0 && IsNormalized(distanceOutput.Normal) == false {
					for i := 0; i < int(input.ProxyA.Count); i++ {
						p := input.ProxyA.Points[i]
						log("pointA[%d] = {%.9f, %.9f, %.9f}", i, p.X, p.Y, p.Z)
					}
					log("radiusA = %.9f", input.ProxyA.Radius)

					for i := 0; i < int(input.ProxyB.Count); i++ {
						p := input.ProxyB.Points[i]
						log("pointB[%d] = {%.9f, %.9f, %.9f}", i, p.X, p.Y, p.Z)
					}
					log("radiusB = %.9f", input.ProxyB.Radius)

					{
						xf := input.Transform
						log("transform = {{%.9f, %.9f, %.9f}, {{%.9f, %.9f, %.9f}, %.9f}", xf.P.X, xf.P.Y, xf.P.Z, xf.Q.V.X,
							xf.Q.V.Y, xf.Q.V.Z, xf.Q.S)
					}

					{
						t := input.TranslationB
						log("t = {%.9f, %.9f, %.9f}", t.X, t.Y, t.Z)
					}

					canEncroach := 0
					if input.CanEncroach {
						canEncroach = 1
					}
					log("maxFraction = %.9f, canEncroach = %d", input.MaxFraction, canEncroach)

					// Numerical problem. Likely extreme input.
					return output
				}

				// Hitting this assert implies that the algorithm brought the shapes too close.
				// B3_ASSERT( distanceOutput.distance > 0.0f && b3IsNormalized( distanceOutput.normal ) );

				output.Fraction = alpha
				output.Point = MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
				output.Normal = distanceOutput.Normal
				output.Hit = true
				return output
			}
		}

		assert(distanceOutput.Distance > 0.0, "distanceOutput.distance > 0.0f")
		assert(IsNormalized(distanceOutput.Normal), "b3IsNormalized( distanceOutput.normal )")

		// Check if shapes are approaching each other
		denominator := Dot(delta2, distanceOutput.Normal)
		if denominator >= 0.0 {
			// Miss
			return output
		}

		// Advance sweep
		alpha += (target - distanceOutput.Distance) / denominator
		if alpha >= input.MaxFraction {
			// Success!
			return output
		}

		distanceInput.Transform.P = MulAdd(input.Transform.P, alpha, delta2)
	}

	// Failure!
	return output
}

// GetSweepTransform evaluates the transform sweep at a specific time.
func GetSweepTransform(sweep *Sweep, time float32) Transform {
	var transform Transform
	transform.Q = NLerp(sweep.Q1, sweep.Q2, time)
	transform.P = Sub(Lerp(sweep.C1, sweep.C2, time), RotateVector(transform.Q, sweep.LocalCenter))
	return transform
}

func getFinalSweepTransform(sweep *Sweep) Transform {
	var transform Transform
	transform.Q = sweep.Q2
	transform.P = Sub(sweep.C2, RotateVector(transform.Q, sweep.LocalCenter))
	return transform
}

func uniqueCount(vertexCount int, vertices *[3]int) int {
	assert(1 <= vertexCount && vertexCount <= 3, "1 <= vertexCount && vertexCount <= 3")

	switch vertexCount {
	case 1:
		return 1

	case 2:
		if vertices[0] != vertices[1] {
			return 2
		}
		return 1

	case 3:
		if vertices[0] != vertices[1] && vertices[0] != vertices[2] && vertices[1] != vertices[2] {
			// All different
			return 3
		}

		if vertices[0] == vertices[1] && vertices[0] == vertices[2] && vertices[1] == vertices[2] {
			// All equal
			return 1
		}

		return 2

	default:
		assert(false, "!\"Should never get here!\"")
	}

	return 0
}

// checkFastEdges checks if the cross product of two edges switches direction.
func checkFastEdges(xfA Transform, localEdgeA Vec3, xfB Transform, localEdgeB Vec3, axis0 Vec3) bool {
	// By taking the local witness axes we make sure that we
	// get the correct orientations (e.g. if one axis was flipped)!
	edgeA := RotateVector(xfA.Q, localEdgeA)
	edgeB := RotateVector(xfB.Q, localEdgeB)
	axis := Cross(edgeA, edgeB)
	return Dot(axis, axis0) < 0.0
}

// separationType is b3SeparationType.
type separationType int32

const (
	separationUnknown separationType = iota
	separationVertices
	separationEdges
	separationFaceA
	separationFaceB
)

// separationFunction is b3SeparationFunction.
type separationFunction struct {
	proxyA *ShapeProxy
	proxyB *ShapeProxy
	sweepA Sweep
	sweepB Sweep

	// These are associated with different bodies depending on the separation function type.
	// It could be two local vectors/points on the same body (for example, both on bodyA).
	witness1 Vec3
	witness2 Vec3

	typ separationType
}

func makeSeparationFunction(cache SimplexCache, proxyA *ShapeProxy, sweepA *Sweep, proxyB *ShapeProxy, sweepB *Sweep,
	worldNormal Vec3, t1 float32) separationFunction {
	assert(1 <= cache.Count && cache.Count <= 3, "1 <= cache.count && cache.count <= 3")
	validate(IsNormalized(worldNormal), "b3IsNormalized( worldNormal )")

	fcn := separationFunction{}
	fcn.proxyA = proxyA
	fcn.proxyB = proxyB
	fcn.sweepA = *sweepA
	fcn.sweepB = *sweepB
	fcn.typ = separationUnknown

	indexA := [3]int{int(cache.IndexA[0]), int(cache.IndexA[1]), int(cache.IndexA[2])}
	indexB := [3]int{int(cache.IndexB[0]), int(cache.IndexB[1]), int(cache.IndexB[2])}

	uniqueCountA := uniqueCount(int(cache.Count), &indexA)
	uniqueCountB := uniqueCount(int(cache.Count), &indexB)

	xfA1 := GetSweepTransform(sweepA, t1)
	xfB1 := GetSweepTransform(sweepB, t1)

	qA := xfA1.Q
	qB := xfB1.Q

	// Minimize round-off
	deltaP := Sub(xfB1.P, xfA1.P)

	switch cache.Count {
	case 1:
		{
			// Witness is the world space direction
			fcn.typ = separationVertices
			fcn.witness1 = worldNormal
		}

	case 2:
		{
			if uniqueCountA == 2 && uniqueCountB == 2 {
				// Edge/Edge
				vA1 := proxyA.Points[indexA[0]]
				localEdgeA := Sub(proxyA.Points[indexA[1]], vA1)
				localEdgeA = Normalize(localEdgeA)
				edgeA := RotateVector(qA, localEdgeA)

				vB1 := proxyB.Points[indexB[0]]
				localEdgeB := Sub(proxyB.Points[indexB[1]], vB1)
				localEdgeB = Normalize(localEdgeB)
				edgeB := RotateVector(qB, localEdgeB)

				axis := Cross(edgeA, edgeB)
				lengthSquared := LengthSquared(axis)

				// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
				const kToleranceSquared float32 = 0.05 * 0.05
				if lengthSquared < kToleranceSquared {
					// The axis is not safe to normalize so we use a world axis instead!
					fcn.typ = separationVertices
					fcn.witness1 = worldNormal
				} else {
					delta := Add(Sub(RotateVector(qB, vB1), RotateVector(qA, vA1)), deltaP)
					if Dot(delta, axis) < 0.0 {
						// Make axis point from A to B
						axis = Neg(axis)
						localEdgeB = Neg(localEdgeB)
					}

					// Check for possible sign flip in edge/edge cross product
					xfA2 := getFinalSweepTransform(sweepA)
					xfB2 := getFinalSweepTransform(sweepB)
					fastEdges := checkFastEdges(xfA2, localEdgeA, xfB2, localEdgeB, axis)
					if fastEdges == true {
						// Not safe to use local edges, fall back to initial world space axis instead
						fcn.typ = separationVertices
						fcn.witness1 = Normalize(axis)
					} else {
						// Edge cross product is safe. This converges faster than a fixed axis.
						fcn.typ = separationEdges
						fcn.witness1 = localEdgeA
						fcn.witness2 = localEdgeB
					}
				}
			} else {
				validate(IsNormalized(worldNormal), "b3IsNormalized( worldNormal )")

				// Vertex versus edge, use world axis witness
				fcn.typ = separationVertices
				fcn.witness1 = worldNormal
			}
		}

	case 3:
		{
			if uniqueCountA == 3 {
				vA1 := proxyA.Points[indexA[0]]
				vA2 := proxyA.Points[indexA[1]]
				vA3 := proxyA.Points[indexA[2]]
				localAxisA := Cross(Sub(vA2, vA1), Sub(vA3, vA1))
				localAxisA = Normalize(localAxisA)
				axisA := RotateVector(qA, localAxisA)

				localPointA := MulSV(1.0/3.0, Add(Add(vA1, vA2), vA3))
				localPointB := proxyB.Points[indexB[0]]
				delta := Add(Sub(RotateVector(qB, localPointB), RotateVector(qA, localPointA)), deltaP)

				if Dot(delta, axisA) < 0.0 {
					// Make axis point from A to B
					localAxisA = Neg(localAxisA)
				}

				// Witness is the local plane of faceA
				fcn.typ = separationFaceA
				fcn.witness1 = localAxisA
				fcn.witness2 = localPointA
			} else if uniqueCountB == 3 {
				vB1 := proxyB.Points[indexB[0]]
				vB2 := proxyB.Points[indexB[1]]
				vB3 := proxyB.Points[indexB[2]]
				localAxisB := Cross(Sub(vB2, vB1), Sub(vB3, vB1))
				localAxisB = Normalize(localAxisB)
				axisB := RotateVector(qB, localAxisB)

				localPointA := proxyA.Points[indexA[0]]
				localPointB := MulSV(1.0/3.0, Add(Add(vB1, vB2), vB3))
				delta := Sub(Sub(RotateVector(qA, localPointA), RotateVector(qB, localPointB)), deltaP)

				if Dot(delta, axisB) < 0.0 {
					// Make axis point from B to A
					localAxisB = Neg(localAxisB)
				}

				// Witness is the local plane of faceB
				fcn.typ = separationFaceB
				fcn.witness1 = localAxisB
				fcn.witness2 = localPointB
			} else {
				assert(uniqueCountA == 2 && uniqueCountB == 2, "uniqueCountA == 2 && uniqueCountB == 2")

				if indexA[0] == indexA[1] {
					// Make first two indices are unique
					indexA[1] = indexA[2]
					assert(indexA[0] != indexA[1], "indexA[0] != indexA[1]")
				}

				vA1 := proxyA.Points[indexA[0]]
				vA2 := proxyA.Points[indexA[1]]
				localEdgeA := Normalize(Sub(vA2, vA1))
				edgeA := RotateVector(qA, localEdgeA)

				if indexB[0] == indexB[1] {
					// Make first two indices are unique
					indexB[1] = indexB[2]
					assert(indexB[0] != indexB[1], "indexB[0] != indexB[1]")
				}

				vB1 := proxyB.Points[indexB[0]]
				vB2 := proxyB.Points[indexB[1]]
				localEdgeB := Normalize(Sub(vB2, vB1))
				edgeB := RotateVector(qB, localEdgeB)

				axis := Cross(edgeA, edgeB)
				lengthSquared := LengthSquared(axis)

				// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
				const kToleranceSquared float32 = 0.005 * 0.005
				if lengthSquared < kToleranceSquared {
					// The axis is not safe to normalize so we use a world axis instead!
					fcn.typ = separationVertices
					fcn.witness1 = worldNormal
				} else {
					delta := Add(Sub(RotateVector(qB, vB1), RotateVector(qA, vA1)), deltaP)
					if Dot(delta, axis) < 0.0 {
						// Make axis point from A to B
						axis = Neg(axis)
						localEdgeB = Neg(localEdgeB)
					}

					// Check for possible sign flip in edge/edge cross product
					xfA2 := getFinalSweepTransform(sweepA)
					xfB2 := getFinalSweepTransform(sweepB)
					fastEdges := checkFastEdges(xfA2, localEdgeA, xfB2, localEdgeB, axis)
					if fastEdges {
						// Not safe to use local edges, fall back to initial world space axis instead
						fcn.typ = separationVertices
						fcn.witness1 = Normalize(axis)
					} else {
						// Edge cross product is safe. This converges faster than a fixed axis.
						fcn.typ = separationEdges
						fcn.witness1 = localEdgeA
						fcn.witness2 = localEdgeB
					}
				}
			}
		}

	default:
		assert(false, "!\"Should never get here!\"")
	}

	return fcn
}

// findMinSeparation is b3FindMinSeparation.
// PORT: the index out-parameters become the second and third return values.
func findMinSeparation(fcn *separationFunction, t float32) (float32, int, int) {
	xfA := GetSweepTransform(&fcn.sweepA, t)
	xfB := GetSweepTransform(&fcn.sweepB, t)

	switch fcn.typ {
	case separationVertices:
		{
			axis := fcn.witness1

			localAxisA := InvRotateVector(xfA.Q, axis)
			localAxisB := InvRotateVector(xfB.Q, Neg(axis))

			indexA := getPointSupport(fcn.proxyA.Points, int(fcn.proxyA.Count), localAxisA)
			indexB := getPointSupport(fcn.proxyB.Points, int(fcn.proxyB.Count), localAxisB)

			deltaP := Sub(xfB.P, xfA.P)
			localPointA := fcn.proxyA.Points[indexA]
			localPointB := fcn.proxyB.Points[indexB]
			delta := Add(Sub(RotateVector(xfB.Q, localPointB), RotateVector(xfA.Q, localPointA)), deltaP)
			return Dot(delta, axis), indexA, indexB
		}

	case separationEdges:
		{
			edgeA := RotateVector(xfA.Q, fcn.witness1)
			edgeB := RotateVector(xfB.Q, fcn.witness2)
			axis := Cross(edgeA, edgeB)
			assert(axis.X != 0.0 || axis.Y != 0.0 || axis.Z != 0.0, "axis.x != 0.0f || axis.y != 0.0f || axis.z != 0.0f")
			axis = Normalize(axis)

			axisA := InvRotateVector(xfA.Q, axis)
			indexA := getPointSupport(fcn.proxyA.Points, int(fcn.proxyA.Count), axisA)

			axisB := InvRotateVector(xfB.Q, axis)
			indexB := getPointSupport(fcn.proxyB.Points, int(fcn.proxyB.Count), Neg(axisB))

			deltaP := Sub(xfB.P, xfA.P)
			localPointA := fcn.proxyA.Points[indexA]
			localPointB := fcn.proxyB.Points[indexB]
			delta := Add(Sub(RotateVector(xfB.Q, localPointB), RotateVector(xfA.Q, localPointA)), deltaP)

			return Dot(delta, axis), indexA, indexB
		}

	case separationFaceA:
		{
			normal := RotateVector(xfA.Q, fcn.witness1)
			indexA := -1
			pointA := TransformPoint(xfA, fcn.witness2)

			axisB := InvRotateVector(xfB.Q, normal)
			indexB := getPointSupport(fcn.proxyB.Points, int(fcn.proxyB.Count), Neg(axisB))
			pointB := TransformPoint(xfB, fcn.proxyB.Points[indexB])

			return Dot(Sub(pointB, pointA), normal), indexA, indexB
		}

	case separationFaceB:
		{
			normal := RotateVector(xfB.Q, fcn.witness1)

			axisA := InvRotateVector(xfA.Q, normal)
			indexA := getPointSupport(fcn.proxyA.Points, int(fcn.proxyA.Count), Neg(axisA))
			pointA := TransformPoint(xfA, fcn.proxyA.Points[indexA])

			indexB := -1
			pointB := TransformPoint(xfB, fcn.witness2)

			return Dot(Sub(pointA, pointB), normal), indexA, indexB
		}

	default:
		assert(false, "!\"Should never get here!\"")
	}

	// PORT: upstream leaves *indexA and *indexB unwritten here.
	return 0.0, 0, 0
}

func evaluateSeparation(fcn *separationFunction, index1, index2 int, beta float32) float32 {
	transform1 := GetSweepTransform(&fcn.sweepA, beta)
	transform2 := GetSweepTransform(&fcn.sweepB, beta)

	switch fcn.typ {
	case separationVertices:
		{
			axis := fcn.witness1

			point1 := TransformPoint(transform1, fcn.proxyA.Points[index1])
			point2 := TransformPoint(transform2, fcn.proxyB.Points[index2])

			return Dot(Sub(point2, point1), axis)
		}

	case separationEdges:
		{
			edge1 := RotateVector(transform1.Q, fcn.witness1)
			edge2 := RotateVector(transform2.Q, fcn.witness2)
			axis := Cross(edge1, edge2)
			axis = Normalize(axis)

			point1 := TransformPoint(transform1, fcn.proxyA.Points[index1])
			point2 := TransformPoint(transform2, fcn.proxyB.Points[index2])

			return Dot(Sub(point2, point1), axis)
		}

	case separationFaceA:
		{
			axis := RotateVector(transform1.Q, fcn.witness1)

			point1 := TransformPoint(transform1, fcn.witness2)
			point2 := TransformPoint(transform2, fcn.proxyB.Points[index2])

			return Dot(Sub(point2, point1), axis)
		}

	case separationFaceB:
		{
			axis := RotateVector(transform2.Q, fcn.witness1)

			point1 := TransformPoint(transform1, fcn.proxyA.Points[index1])
			point2 := TransformPoint(transform2, fcn.witness2)

			return Dot(Sub(point1, point2), axis)
		}

	default:
		assert(false, "!\"Should never get here!\"")
	}

	return 0.0
}

func forceFixedAxis(fcn *separationFunction, beta float32) {
	assert(fcn.typ == separationEdges, "fcn->type == b3_separationEdges")

	transform1 := GetSweepTransform(&fcn.sweepA, beta)
	transform2 := GetSweepTransform(&fcn.sweepB, beta)

	edge1 := RotateVector(transform1.Q, fcn.witness1)
	edge2 := RotateVector(transform2.Q, fcn.witness2)
	axis := Cross(edge1, edge2)
	axis = Normalize(axis)

	fcn.typ = separationVertices
	fcn.witness1 = axis
	fcn.witness2 = Vec3Zero
}

// TimeOfImpact computes the upper bound on time before two shapes penetrate. Time is represented as
// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
// non-tunneling collisions. If you change the time interval, you should call this function
// again.
// Time of Impact using root finding
func TimeOfImpact(input *TOIInput) TOIOutput {
	output := TOIOutput{}

	// Set these to invalid values so they can be validated on exit
	output.State = TOIStateUnknown
	output.Fraction = -1.0

	sweepA := input.SweepA
	sweepB := input.SweepB

	// Shift to origin
	origin := sweepA.C1
	sweepA.C1 = Vec3Zero
	sweepA.C2 = Sub(sweepA.C2, origin)
	sweepB.C1 = Sub(sweepB.C1, origin)
	sweepB.C2 = Sub(sweepB.C2, origin)

	proxyA := input.ProxyA
	proxyB := input.ProxyB

	maxPushBackIterations := int(proxyA.Count + proxyB.Count)
	tMax := input.MaxFraction

	// Setup target distance and tolerance
	linearSlop := LinearSlop()
	totalRadius := proxyA.Radius + proxyB.Radius
	target := MaxFloat(linearSlop, totalRadius-linearSlop)
	tolerance := float32(0.25 * linearSlop)
	assert(target > tolerance, "target > tolerance")

	var t1 float32 = 0.0
	const maxIterations = 25
	distanceIterations := 0

	// Prepare input for distance query.
	cache := SimplexCache{}
	distanceInput := DistanceInput{}
	distanceInput.ProxyA = proxyA
	distanceInput.ProxyB = proxyB
	distanceInput.UseRadii = false

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for {
		// Get the distance between shapes. We can also use the results to get a separating axis.
		xfA := GetSweepTransform(&sweepA, t1)
		xfB := GetSweepTransform(&sweepB, t1)
		distanceInput.Transform = InvMulTransforms(xfA, xfB)
		distanceOutput := ShapeDistance(&distanceInput, &cache, nil)
		output.Distance = distanceOutput.Distance

		// The distance query runs in frame A, project the witness data back to the shifted world
		worldNormal := RotateVector(xfA.Q, distanceOutput.Normal)
		worldPointA := TransformPoint(xfA, distanceOutput.PointA)
		worldPointB := TransformPoint(xfA, distanceOutput.PointB)

		output.DistanceIterations += 1
		distanceIterations += 1

		// If the shapes are overlapped, we give up on continuous collision.
		if distanceOutput.Distance <= 0.0 {
			output.State = TOIStateOverlapped
			output.Fraction = 0.0

			// Averaged hit point
			pA := MulAdd(worldPointA, proxyA.Radius, worldNormal)
			pB := MulAdd(worldPointB, -proxyB.Radius, worldNormal)
			output.Point = Lerp(pA, pB, 0.5)
			output.Point = Add(output.Point, origin)
			output.Normal = worldNormal

			break
		}

		if distanceOutput.Distance <= target+tolerance {
			// Success!
			output.State = TOIStateHit

			// Averaged hit point
			pA := MulAdd(worldPointA, proxyA.Radius, worldNormal)
			pB := MulAdd(worldPointB, -proxyB.Radius, worldNormal)
			output.Point = Lerp(pA, pB, 0.5)
			output.Point = Add(output.Point, origin)
			output.Normal = worldNormal
			output.Fraction = t1
			break
		}

		if distanceIterations == maxIterations {
			// Progress too slow. This can happen when a capsule rotates around a
			// triangle vertex.
			output.State = TOIStateFailed
			output.Fraction = t1

			// Averaged hit point
			pA := MulAdd(worldPointA, input.ProxyA.Radius, worldNormal)
			pB := MulAdd(worldPointB, -input.ProxyB.Radius, worldNormal)
			output.Point = Lerp(pA, pB, 0.5)
			output.Point = Add(output.Point, origin)
			output.Normal = worldNormal
			break
		}

		// Initialize the separating axis.
		function := makeSeparationFunction(cache, &proxyA, &sweepA, &proxyB, &sweepB, worldNormal, t1)

		// PORT: the disabled `#if B3_ENABLE_VALIDATION && 0` block is not ported.

		// Compute the TOI on the separating axis. We do this by successively resolving the deepest point.
		done := false
		t2 := tMax
		pushBackIterations := 0
		for {
			s2, indexA, indexB := findMinSeparation(&function, t2)

			// Dump the function seen by the root finder
			// 			for ( int Step = 0; Step <= 100; ++Step )
			// 				{
			// 				float Alpha = 0.01f * Step;
			// 				float Separation = Function.Evaluate( Index1, Index2, Alpha );
			//
			// 				b3Report( "s(%4.2g) = %g\n", Alpha, Separation );
			// 				}

			// Is the final configuration separated?
			if s2-target > tolerance {
				// Success!
				output.State = TOIStateSeparated
				output.Fraction = input.MaxFraction
				done = true
				break
			}

			// Has the separation reached tolerance?
			if s2 >= target-tolerance {
				// Advance the sweeps
				t1 = t2
				break
			}

			// Compute the initial separation of the witness points
			s1 := evaluateSeparation(&function, indexA, indexB, t1)

			// Check for overlap. This might happen if the root finder runs out of iterations.
			if s1 < target-tolerance {
				// Failed!
				validate(false, "false")
				output.State = TOIStateFailed
				output.Fraction = t1
				done = true
				break
			}

			// Has the separation reached tolerance?
			if s1 <= target+tolerance {
				// Success! t1 should hold the TOI (could be 0.0)
				output.State = TOIStateHit
				output.Fraction = t1
				done = true
				break
			}

			// Compute 1D root of: f(x) - target = 0
			rootIterationCount := 0
			maxRootIterations := 50
			a1 := t1
			a2 := t2
			for {
				// Use a mix of false position and bisection.
				var t float32
				if rootIterationCount&1 != 0 {
					// False position to improve convergence.
					t = a1 + float32((target-s1)*(a2-a1))/(s2-s1)
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2)
				}

				output.RootIterations += 1
				rootIterationCount += 1

				s := evaluateSeparation(&function, indexA, indexB, t)

				// Has the separation reached tolerance?
				if AbsFloat(s-target) <= tolerance {
					// t2 holds a tentative value for t1
					t2 = t
					break
				}

				// Ensure we continue to bracket the root.
				if s > target {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}

				if rootIterationCount == maxRootIterations {
					validate(false, "false")
					break
				}
			}

			// Restart the inner loop if we have a failing edge case.
			if rootIterationCount == maxRootIterations-1 && function.typ == separationEdges {
				validate(false, "false")

				rootIterationCount = 0
				t2 = input.MaxFraction
				forceFixedAxis(&function, t1)
				assert(function.typ != separationEdges, "function.type != b3_separationEdges")
			}

			output.PushBackIterations += 1
			pushBackIterations += 1

			if pushBackIterations == maxPushBackIterations {
				break
			}
		}

		if done {
			// Averaged hit point
			pA := MulAdd(worldPointA, input.ProxyA.Radius, worldNormal)
			pB := MulAdd(worldPointB, -input.ProxyB.Radius, worldNormal)
			output.Point = Lerp(pA, pB, 0.5)
			output.Point = Add(output.Point, origin)
			output.Normal = worldNormal
			break
		}
	}

	// It is expected that the state and fraction are set before reaching this
	assert(output.State != TOIStateUnknown, "output.state != b3_toiStateUnknown")
	assert(output.Fraction >= 0.0, "output.fraction >= 0.0f")

	return output
}
