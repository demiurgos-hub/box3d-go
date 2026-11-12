// Port of src/manifold.h and src/manifold.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.
//
// Dirk Gregorius contributed portions of this code

package box3d

// ---- manifold.h ----

// maxClipPoints is B3_MAX_CLIP_POINTS.
const maxClipPoints = 2 * MaxHullVertices

// separatingAxis is b3SeparatingAxis.
type separatingAxis struct {
	normal     Vec3
	separation float32
	indexA     int32
	indexB     int32
	typ        SeparatingFeature
}

// axisQuery is b3AxisQuery.
type axisQuery struct {
	faceA            separatingAxis
	faceB            separatingAxis
	edge             separatingAxis
	separatedFeature SeparatingFeature
}

// clipVertex is b3ClipVertex.
type clipVertex struct {
	position   Vec3
	separation float32
	pair       FeaturePair
}

// featureOwner is b3FeatureOwner.
type featureOwner int32

const (
	featureShapeA featureOwner = 0
	featureShapeB featureOwner = 1
)

// PORT: b3ComputeSeparatingAxis, b3ComputeSeparatingAxisAtWidth, b3CollideHullsAtWidth and the
// W4/W8 variants are declared here upstream and defined in convex_manifold.c; they are ported
// with that file.

// featurePairSingle is b3FeaturePair_single.
// For single point contact, such as sphere-sphere, sphere-capsule, sphere-triangle
var featurePairSingle = FeaturePair{}

// makeFeatureId is b3MakeFeatureId.
func makeFeatureId(pair FeaturePair) uint32 {
	id, ok := EncodeFeaturePair(pair)
	if !ok {
		panic("box3d: invalid contact feature pair")
	}
	return id
}

// getBestAxis is b3GetBestAxis.
func getBestAxis(query *axisQuery) separatingAxis {
	validate(query.faceA.typ == FaceAxisA, "query->faceA.type == b3_faceAxisA")
	validate(query.edge.typ == EdgePairAxis, "query->edge.type == b3_edgePairAxis")
	validate(query.faceB.typ == FaceAxisB, "query->faceB.type == b3_faceAxisB")

	if query.faceA.separation > query.faceB.separation {
		if query.edge.separation > query.faceA.separation {
			return query.edge
		}

		return query.faceA
	}

	if query.edge.separation > query.faceB.separation {
		return query.edge
	}

	return query.faceB
}

// ---- manifold.c ----

// findIncidentFace is b3FindIncidentFace.
// This was extended to make the wedge shape get the correct incident face.
// Instead of looking directly for the most anti-parallel face, we first find the closest vertex (passed in).
// Then we look for all edges coming out of that vertex and look for the edge that is
// most perpendicular to the reference normal.
// Then from that edge, we select the adjacent face that is most anti-parallel to the reference normal.
func findIncidentFace(hull *HullData, refNormal Vec3, vertexIndex int) int {
	assert(0 <= vertexIndex && vertexIndex < int(hull.VertexCount), "0 <= vertexIndex && vertexIndex < hull->vertexCount")

	vertices := GetHullVertices(hull)
	edges := GetHullEdges(hull)
	planes := GetHullPlanes(hull)
	points := GetHullPoints(hull)

	minEdgeIndex := -1
	minEdgeProjection := fltMax

	vertex := &vertices[vertexIndex]
	assert(vertex != nil, "vertex")

	edgeIndex := int(vertex.Edge)
	edge := &edges[edgeIndex]
	edgeOrigin := points[edge.Origin]
	assert(int(edge.Origin) == vertexIndex, "edge->origin == vertexIndex")

	for {
		twin := &edges[edge.Twin]
		twinOrigin := points[twin.Origin]

		axis := Normalize(Sub(twinOrigin, edgeOrigin))
		edgeProjection := AbsFloat(Dot(axis, refNormal))
		if edgeProjection < minEdgeProjection {
			minEdgeIndex = edgeIndex
			minEdgeProjection = edgeProjection
		}

		edgeIndex = int(twin.Next)
		edge = &edges[edgeIndex]
		assert(int(edge.Origin) == vertexIndex, "edge->origin == vertexIndex")
		if edge == &edges[vertex.Edge] {
			break
		}
	}
	assert(minEdgeIndex >= 0, "minEdgeIndex >= 0")

	minEdge := &edges[minEdgeIndex]
	minFaceIndex1 := int(minEdge.Face)
	minPlane1 := planes[minFaceIndex1]

	minTwin := &edges[minEdge.Twin]
	minFaceIndex2 := int(minTwin.Face)
	minPlane2 := planes[minFaceIndex2]

	if Dot(minPlane1.Normal, refNormal) < Dot(minPlane2.Normal, refNormal) {
		return minFaceIndex1
	}
	return minFaceIndex2
}

// makeFeaturePair is b3MakeFeaturePair.
func makeFeaturePair(owner1 featureOwner, index1 int, owner2 featureOwner, index2 int) FeaturePair {
	if index1 < 0 || index1 > MaxFeatureIndex || index2 < 0 || index2 > MaxFeatureIndex ||
		(owner1 != featureShapeA && owner1 != featureShapeB) || (owner2 != featureShapeA && owner2 != featureShapeB) {
		panic("box3d: contact feature exceeds encoding bounds")
	}

	var pair FeaturePair
	pair.Index1 = uint16(index1)
	pair.Owner1 = uint8(owner1)
	pair.Index2 = uint16(index2)
	pair.Owner2 = uint8(owner2)
	return pair
}

// flipPair is b3FlipPair.
// This logic seems wrong but it is designed so that choosing
// face A or B as the reference face does not change the resulting
// feature pair. This way the contact impulses are persisted even
// if there is reference face flip-flop. This is verified in the
// HullAndHull sample using the b3SATCache with a manual feature
// specified such as b3_manualFaceAxisA.
func flipPair(pair FeaturePair) FeaturePair {
	assert(pair.Owner1 == 0 || pair.Owner1 == 1, "pair.owner1 == 0 || pair.owner1 == 1")
	assert(pair.Owner2 == 0 || pair.Owner2 == 1, "pair.owner2 == 0 || pair.owner2 == 1")
	pair.Owner1, pair.Owner2 = pair.Owner2, pair.Owner1
	pair.Owner1 = 1 - pair.Owner1
	pair.Owner2 = 1 - pair.Owner2
	pair.Index1, pair.Index2 = pair.Index2, pair.Index1
	return pair
}

// validatePolygon is b3ValidatePolygon (B3_ENABLE_VALIDATION).
// PORT: always compiled; only called through validate, which is off without box3d_validate.
func validatePolygon(polygon []clipVertex, count int) bool {
	// Empty polygons are valid (we can clip away all points when re-constructing manifolds from cache)
	if count == 0 {
		return true
	}

	// Validate that incoming and outgoing edges match
	vertex1 := polygon[count-1]
	for i := range count {
		vertex2 := polygon[i]

		if vertex1.pair.Owner2 != vertex2.pair.Owner1 {
			return false
		}

		if vertex1.pair.Index2 != vertex2.pair.Index1 {
			return false
		}

		vertex1 = vertex2
	}

	return true
}

// clipPolygon is b3ClipPolygon.
// PORT: out and polygon are slices; count is kept (the polygon slice may be longer).
func clipPolygon(out []clipVertex, polygon []clipVertex, count int, clipPlane Plane, edge int, refPlane Plane) int {
	assert(count >= 3, "count >= 3")

	vertex1 := polygon[count-1]
	distance1 := planeSeparation(clipPlane, vertex1.position)
	outCount := 0

	for index := range count {
		vertex2 := polygon[index]
		distance2 := planeSeparation(clipPlane, vertex2.position)

		// Clip edge against plane (Sutherland-Hodgman clipping)
		if distance1 <= 0.0 && distance2 <= 0.0 {
			// Both vertices are behind the plane - keep vertex2
			out[outCount] = vertex2
			outCount += 1
		} else if distance1 <= 0.0 && distance2 > 0.0 {
			// Vertex1 is behind of the plane, vertex2 is in front -> intersection point
			fraction := distance1 / (distance1 - distance2)
			position := MulAdd(vertex1.position, fraction, Sub(vertex2.position, vertex1.position))

			// Keep intersection point and adjust outgoing edge
			var vertex clipVertex
			vertex.position = position
			vertex.separation = planeSeparation(refPlane, position)
			vertex.pair = vertex2.pair
			vertex.pair.Owner2 = uint8(featureShapeA)
			vertex.pair.Index2 = uint16(edge)
			out[outCount] = vertex
			outCount += 1
		} else if distance2 <= 0.0 && distance1 > 0.0 {
			// Vertex1 is in front, vertex2 is behind of the plane, -> intersection point
			fraction := distance1 / (distance1 - distance2)
			position := MulAdd(vertex1.position, fraction, Sub(vertex2.position, vertex1.position))

			// Keep intersection point and adjust incoming edge
			var vertex clipVertex
			vertex.position = position
			vertex.separation = planeSeparation(refPlane, position)
			vertex.pair = vertex1.pair
			vertex.pair.Owner1 = uint8(featureShapeA)
			vertex.pair.Index1 = uint16(edge)
			out[outCount] = vertex
			outCount += 1

			// And also keep vertex2
			out[outCount] = vertex2
			outCount += 1
		}

		// Keep vertex2 as starting vertex for next edge
		vertex1 = vertex2
		distance1 = distance2
	}

	validate(!validateEnabled || validatePolygon(out, outCount), "b3ValidatePolygon( out, outCount )")

	return outCount
}
