// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// The default path is unchanged. The opt-in path visits the same cooked geometry
// and uses the upstream narrowphase with winding facing the query's starting side.
func rayCastShapeWithBackFaces(s *shape, transform Transform, input *RayCastInput, backFaces bool) CastOutput {
	if !backFaces || isConvex(s.typ) {
		return rayCastShape(s, transform, input)
	}
	end := MulAdd(input.Origin, input.MaxFraction, input.Translation)
	bounds := AABB{Min(input.Origin, end), Max(input.Origin, end)}
	best := CastOutput{Fraction: input.MaxFraction, ChildIndex: -1, TriangleIndex: -1, MaterialIndex: -1}
	visitQueryGeometry(s, transform, bounds, func(leaf queryLeaf) {
		local := *input
		local.Origin = InvTransformPoint(leaf.transform, input.Origin)
		local.Translation = InvRotateVector(leaf.transform.Q, input.Translation)
		local.MaxFraction = best.Fraction
		var out CastOutput
		if leaf.triangleIndex >= 0 {
			tri := leaf.triangle
			if signedVolume(tri[0], tri[1], tri[2], local.Origin) < 0 {
				tri[1], tri[2] = tri[2], tri[1]
			}
			a := intersectRayTriangle(loadV(vec3Floats(&local.Origin)), loadV(vec3Floats(&local.Translation)),
				loadV(vec3Floats(&tri[0])), loadV(vec3Floats(&tri[1])), loadV(vec3Floats(&tri[2])))
			if a < best.Fraction {
				out = CastOutput{Hit: true, Fraction: a, Point: MulAdd(local.Origin, a, local.Translation),
					Normal: Normalize(Cross(Sub(tri[1], tri[0]), Sub(tri[2], tri[0])))}
			}
		} else {
			switch leaf.kind {
			case SphereShape:
				out = RayCastSphere(&leaf.sphere, &local)
			case CapsuleShape:
				out = RayCastCapsule(&leaf.capsule, &local)
			case HullShape:
				out = RayCastHull(leaf.hull, &local)
			}
		}
		if out.Hit && out.Fraction <= best.Fraction {
			best = queryLeafCastOutput(s, leaf, out)
		}
	})
	return best
}

func shapeCastShapeWithBackFaces(s *shape, transform Transform, input *ShapeCastInput, backFaces bool) CastOutput {
	if !backFaces || isConvex(s.typ) {
		return shapeCastShape(s, transform, input)
	}
	bounds := MakeAABB(input.Proxy.Points[:input.Proxy.Count], input.Proxy.Radius)
	delta := MulSV(input.MaxFraction, input.Translation)
	bounds.LowerBound = Add(bounds.LowerBound, Min(Vec3Zero, delta))
	bounds.UpperBound = Add(bounds.UpperBound, Max(Vec3Zero, delta))
	best := CastOutput{Fraction: input.MaxFraction, ChildIndex: -1, TriangleIndex: -1, MaterialIndex: -1}
	visitQueryGeometry(s, transform, bounds, func(leaf queryLeaf) {
		local := *input
		var points [MaxShapeCastPoints]Vec3
		local.Proxy.Count = int32(MinInt(int(input.Proxy.Count), MaxShapeCastPoints))
		center := Vec3Zero
		for i := range int(local.Proxy.Count) {
			points[i] = InvTransformPoint(leaf.transform, input.Proxy.Points[i])
			center = Add(center, points[i])
		}
		center = MulSV(1/float32(local.Proxy.Count), center)
		local.Proxy.Points = points[:local.Proxy.Count]
		local.Translation = InvRotateVector(leaf.transform.Q, input.Translation)
		local.MaxFraction = best.Fraction
		var out CastOutput
		if leaf.triangleIndex >= 0 {
			tri := leaf.triangle
			if signedVolume(tri[0], tri[1], tri[2], center) < 0 {
				tri[1], tri[2] = tri[2], tri[1]
			}
			// Keep the upstream triangle-relative origin and initial-overlap policy.
			origin := tri[0]
			trianglePoints := [3]Vec3{Vec3Zero, Sub(tri[1], origin), Sub(tri[2], origin)}
			pair := ShapeCastPairInput{ProxyA: ShapeProxy{trianglePoints[:], 3, 0}, ProxyB: local.Proxy,
				Transform: Transform{Neg(origin), QuatIdentity}, TranslationB: local.Translation,
				MaxFraction: best.Fraction, CanEncroach: local.CanEncroach}
			out = ShapeCast(&pair)
			out.Point = Add(out.Point, origin)
		} else {
			switch leaf.kind {
			case SphereShape:
				out = ShapeCastSphere(&leaf.sphere, &local)
			case CapsuleShape:
				out = ShapeCastCapsule(&leaf.capsule, &local)
			case HullShape:
				out = ShapeCastHull(leaf.hull, &local)
			}
		}
		if out.Hit && out.Fraction <= best.Fraction {
			best = queryLeafCastOutput(s, leaf, out)
		}
	})
	return best
}

func queryLeafCastOutput(s *shape, leaf queryLeaf, out CastOutput) CastOutput {
	out.Point = TransformPoint(leaf.transform, out.Point)
	out.Normal = RotateVector(leaf.transform.Q, out.Normal)
	out.ChildIndex, out.TriangleIndex = leaf.child, leaf.triangleIndex
	out.MaterialIndex = int32(getShapeMaterialIndex(s, int(leaf.child), int(leaf.triangleIndex)))
	return out
}
