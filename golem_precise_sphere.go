// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// A sphere swept against a triangle is a ray against its rounded Minkowski
// surface: two planar faces and three edge capsules (including their endcaps).
// Solving those features directly avoids a GJK fraction stalling at float32 ULPs.
func preciseSphereTriangle(leaf queryLeaf, center Vec3, radius float32, translation Vec3, tolerance float32) (CastOutput, bool) {
	pivot := leaf.triangle[0]
	points := [3]Vec3{{}, Sub(leaf.triangle[1], pivot), Sub(leaf.triangle[2], pivot)}
	for i := 0; i < 3; i++ {
		// The upstream capsule ray helper approximates smaller segments as a
		// sphere; leave such geometry to the generic precise path instead.
		if DistanceSquared(points[i], points[(i+1)%3]) < float32(.01*LinearSlop())*float32(.01*LinearSlop()) {
			return CastOutput{}, false
		}
	}
	originalCenter := center
	pivot = TransformPoint(leaf.transform, pivot)
	center = InvRotateVector(leaf.transform.Q, Sub(center, pivot))
	movement := InvRotateVector(leaf.transform.Q, translation)
	normal := Normalize(Cross(points[1], points[2]))
	if LengthSquared(normal) == 0 {
		return CastOutput{}, false
	}
	output := CastOutput{TriangleIndex: leaf.triangleIndex}
	closest := queryClosestTriangle(points[0], points[1], points[2], center)
	gap := Sub(center, closest.point)
	if Length(gap) <= radius+tolerance {
		output.Hit = true
		output.Point = closest.point
		output.Normal = Normalize(gap)
		if LengthSquared(output.Normal) == 0 {
			output.Normal = normal
		}
	} else {
		distance, approach := Dot(center, normal), Dot(movement, normal)
		if approach != 0 {
			for _, sign := range []float32{-1, 1} {
				fraction := (float32(sign*radius) - distance) / approach
				if fraction < 0 || fraction > 1 {
					continue
				}
				point := MulSub(MulAdd(center, fraction, movement), sign*radius, normal)
				inside := true
				for i := 0; i < 3; i++ {
					if Dot(Cross(Sub(points[(i+1)%3], points[i]), Sub(point, points[i])), normal) < 0 {
						inside = false
						break
					}
				}
				if inside && (!output.Hit || fraction < output.Fraction) {
					output = CastOutput{Hit: true, Fraction: fraction, Point: point, Normal: MulSV(sign, normal), TriangleIndex: leaf.triangleIndex}
				}
			}
		}
		for i := 0; i < 3; i++ {
			capsule := Capsule{Center1: points[i], Center2: points[(i+1)%3], Radius: radius}
			hit := RayCastCapsule(&capsule, &RayCastInput{Origin: center, Translation: movement, MaxFraction: 1})
			if hit.Hit && (!output.Hit || hit.Fraction < output.Fraction) {
				hit.Point = MulSub(hit.Point, radius, hit.Normal)
				output = hit
			}
		}
	}
	if output.Hit {
		output.Point = Add(pivot, RotateVector(leaf.transform.Q, output.Point))
		output.Normal = RotateVector(leaf.transform.Q, output.Normal)
		output.TriangleIndex = leaf.triangleIndex
	} else if tolerance > 0 {
		// Prefer the exact geometric boundary when it exists. A near miss or
		// endpoint can still lie in the caller's accepted distance band.
		return preciseSphereTriangle(leaf, originalCenter, radius+tolerance, translation, 0)
	}
	return output, true
}
