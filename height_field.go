// Port of src/height_field.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"unsafe"
)

/*
	Convention

	index = row * columnCount + column
	height = minHeight + heightScale * compressedHeights[index];

	column = index % columnCount;
	row = index / columnCount;

	x-axis : columns
	z-axis : rows

	00 --- 01 --- 02 --- 03 X
	|  0   |  1   |  2   |
	04 --- 05 --- 06 --- 07
	|  3   |  4   |  5   |
	08 --- 09 --- 10 --- 11
	|  6   |  7   |  8   |
	12 --- 13 --- 14 --- 15
	Z

	The quads exist before the column and row ends: row < rowCount - 1 and column < columnCount - 1
	quadIndex = row * (columnCount - 1) + column

	Quad origin index from quad index (needs row):
	index = quadIndex + row * columnCount

	Triangle index is related to the quad index
	triangleIndex = 2 * quadIndex + (0/1)
	quadIndex = triangleIndex / 2

	Row and column from quad index:
	row = quadIndex / (columnCount - 1)
	column = quadIndex - row * (columnCount - 1)

	The triangle diagonal is fixed.

	triangle0 = {00, 04, 01} -> {11, 21, 12}
	triangle1 = {04, 05, 01} -> {22, 12, 21}

	11      12
	00 ---- 01
	|     / |
	| 0 / 1 | 1
	| /     |
	04 ---- 05
	21     22

	For adjacency we have

	   NA
	   00 ---- 01 ---- 02
	   |     / |     / |
	NA | 0 / 1 | 2 / 3 |
	   | /     | /     |
	   04 ---- 05 ---- 06
	   |     / |     / |
	NA | 6 / 7 | 8 / 9 |
	   | /     | /     |
	   08 ---- 09 ---- 10

	   0: NA, NA, 1
	   1: 0, 6, 2

	Triangle layouts

	   11  3  12
	   1 ----  3
	   |     /
	 1 | 0 / 2
	   | /
	   2
	   21

			  12
			  2
			/ |
		2 / 1 | 1
		/     |
	   3 ---- 1
	   21  3  22
*/

// CreateHeightField creates a generic height field.
func CreateHeightField(data *HeightFieldDef) *HeightFieldData {
	columnCount := int(data.CountX)
	rowCount := int(data.CountZ)

	heightCount := columnCount * rowCount
	assert(heightCount >= 4, "heightCount >= 4")

	cellCount := (columnCount - 1) * (rowCount - 1)
	triangleCount := 2 * cellCount

	// Single blob: struct followed by the height, material, and flag arrays. Layout
	// mirrors b3HullData/b3MeshData so the recording path can copy it with one memcpy.
	byteCount := alignUp8(int(unsafe.Sizeof(HeightFieldData{})))
	heightsOffset := byteCount
	byteCount += alignUp8(heightCount * 2)
	materialOffset := byteCount
	byteCount += alignUp8(cellCount * 1)
	flagsOffset := byteCount
	byteCount += alignUp8(triangleCount * 1)

	// Zero the whole blob so alignment padding is defined. The construction-time hash
	// sweeps raw bytes and would otherwise pick up uninitialized padding.
	hf := (*HeightFieldData)(blobAlloc(byteCount))

	hf.Version = HeightFieldVersion
	hf.ByteCount = int32(byteCount)
	hf.Scale = data.Scale
	hf.ColumnCount = int32(columnCount)
	hf.RowCount = int32(rowCount)
	hf.HeightsOffset = int32(heightsOffset)
	hf.MaterialOffset = int32(materialOffset)
	hf.FlagsOffset = int32(flagsOffset)
	if data.ClockwiseWinding {
		hf.Clockwise = 1
	}

	compressedHeights := blobSlice[uint16](unsafe.Pointer(hf), int32(heightsOffset), heightCount)
	materialIndices := blobSlice[uint8](unsafe.Pointer(hf), int32(materialOffset), cellCount)
	flags := blobSlice[uint8](unsafe.Pointer(hf), int32(flagsOffset), triangleCount)

	heights := data.Heights

	assert(data.GlobalMinimumHeight <= data.GlobalMaximumHeight, "data->globalMinimumHeight <= data->globalMaximumHeight")
	hf.MinHeight = data.GlobalMinimumHeight
	hf.MaxHeight = data.GlobalMaximumHeight

	height := MaxFloat(hf.MaxHeight-hf.MinHeight, LinearSlop())
	hf.HeightScale = height / math.MaxUint16

	lowerHeightBound := hf.MaxHeight
	upperHeightBound := hf.MinHeight

	invHeightScale := 1.0 / hf.HeightScale
	for i := range heightCount {
		clampedHeight := ClampFloat(heights[i], hf.MinHeight, hf.MaxHeight)
		scaledHeight := (clampedHeight - hf.MinHeight) * invHeightScale
		compressedHeights[i] = uint16(MinFloat(scaledHeight, float32(math.MaxUint16)))

		lowerHeightBound = MinFloat(lowerHeightBound, clampedHeight)
		upperHeightBound = MaxFloat(upperHeightBound, clampedHeight)
	}

	// Use decompressed heights for accurate convexity metrics.
	decompressedHeights := make([]float32, heightCount)
	for i := range heightCount {
		decompressedHeights[i] = mulAdd(hf.MinHeight, hf.HeightScale, float32(compressedHeights[i]))
	}
	heights = decompressedHeights

	if data.MaterialIndices != nil {
		for i := range cellCount {
			materialIndices[i] = data.MaterialIndices[i]
		}
	} else {
		for i := range cellCount {
			materialIndices[i] = 0
		}
	}

	hf.AABB.LowerBound = Vec3{0.0, hf.Scale.Y * lowerHeightBound, 0.0}
	hf.AABB.UpperBound =
		Vec3{hf.Scale.X * float32(hf.ColumnCount-1), hf.Scale.Y * upperHeightBound, hf.Scale.Z * float32(hf.RowCount-1)}

	var cos5Deg float32 = 0.9962
	scale := hf.Scale

	triangleIndex := 0
	for row := 0; row < rowCount-1; row++ {
		for column := 0; column < columnCount-1; column++ {
			// todo compute convexity flags
			// This requires a couple things
			// - determine all 3 adjacent triangles for each triangle
			// - consider clockwise winding
			// - consider borders where there is no adjacent triangle

			triangleIndex1 := triangleIndex
			triangleIndex2 := triangleIndex + 1
			triangleIndex += 2

			cellIndex := row*(columnCount-1) + column

			if materialIndices[cellIndex] == HeightFieldHole {
				continue
			}

			flags1 := 0
			flags2 := 0

			var plane1, plane2 Plane
			var center1, center2 Vec3

			index11 := row*columnCount + column
			index12 := index11 + 1
			index21 := (row+1)*columnCount + column
			index22 := index21 + 1

			{
				height11 := heights[index11]
				height12 := heights[index12]
				height21 := heights[index21]
				height22 := heights[index22]

				x1 := float32(column)
				x2 := float32(column + 1)
				z1 := float32(row)
				z2 := float32(row + 1)

				// triangle 0 : 11, 21, 12
				var vs0 [3]Vec3
				vs0[0] = Mul(scale, Vec3{x1, height11, z1})
				vs0[1] = Mul(scale, Vec3{x1, height21, z2})
				vs0[2] = Mul(scale, Vec3{x2, height12, z1})
				plane1 = makePlaneFromPoints(vs0[0], vs0[1], vs0[2])

				center1 = MulSV(1.0/3.0, Add(Add(vs0[0], vs0[1]), vs0[2]))

				// triangle 1 : 22, 12, 21
				var vs1 [3]Vec3
				vs1[0] = Mul(scale, Vec3{x2, height22, z2})
				vs1[1] = Mul(scale, Vec3{x2, height12, z1})
				vs1[2] = Mul(scale, Vec3{x1, height21, z2})
				plane2 = makePlaneFromPoints(vs1[0], vs1[1], vs1[2])

				center2 = MulSV(1.0/3.0, Add(Add(vs1[0], vs1[1]), vs1[2]))

				separation := planeSeparation(plane1, vs1[0])
				cosAngle := Dot(plane1.Normal, plane2.Normal)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags1 |= int(ConcaveEdge2)
					flags2 |= int(ConcaveEdge2)
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags1 |= int(InverseConcaveEdge2)
					flags2 |= int(InverseConcaveEdge2)
				}
			}

			_ = center1
			_ = center2

			// top
			topCellIndex := (row-1)*(columnCount-1) + column
			if row > 0 && materialIndices[topCellIndex] != HeightFieldHole {
				assert(0 <= topCellIndex && topCellIndex < cellCount, "0 <= topCellIndex && topCellIndex < cellCount")

				r := row - 1
				c := column

				i11 := r*columnCount + c
				i12 := i11 + 1
				i21 := (r+1)*columnCount + c
				i22 := i21 + 1

				assert(i21 == index11, "i21 == index11")
				assert(i22 == index12, "i22 == index12")

				// float h11 = heights[i11];
				h12 := heights[i12]
				h21 := heights[i21]
				h22 := heights[i22]

				x1 := float32(c)
				x2 := float32(c + 1)
				z1 := float32(r)
				z2 := float32(r + 1)

				// triangle 1
				var vs [3]Vec3
				vs[0] = Mul(scale, Vec3{x2, h22, z2})
				vs[1] = Mul(scale, Vec3{x2, h12, z1})
				vs[2] = Mul(scale, Vec3{x1, h21, z2})

				n := Normalize(Cross(Sub(vs[1], vs[0]), Sub(vs[2], vs[0])))

				separation := planeSeparation(plane1, vs[1])
				cosAngle := Dot(plane1.Normal, n)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags1 |= int(ConcaveEdge3)
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags1 |= int(InverseConcaveEdge3)
				}
			}

			bottomCellIndex := (row+1)*(columnCount-1) + column
			if row+1 < rowCount-1 && materialIndices[bottomCellIndex] != HeightFieldHole {
				assert(0 <= bottomCellIndex && bottomCellIndex < cellCount, "0 <= bottomCellIndex && bottomCellIndex < cellCount")

				r := row + 1
				c := column

				i11 := r*columnCount + c
				i12 := i11 + 1
				i21 := (r+1)*columnCount + c
				// int i22 = i21 + 1;

				assert(i11 == index21, "i11 == index21")
				assert(i12 == index22, "i12 == index22")

				h11 := heights[i11]
				h12 := heights[i12]
				h21 := heights[i21]
				// float h22 = heights[i22];

				x1 := float32(c)
				x2 := float32(c + 1)
				z1 := float32(r)
				z2 := float32(r + 1)

				// triangle 0
				var vs [3]Vec3
				vs[0] = Mul(scale, Vec3{x1, h11, z1})
				vs[1] = Mul(scale, Vec3{x1, h21, z2})
				vs[2] = Mul(scale, Vec3{x2, h12, z1})

				n := Normalize(Cross(Sub(vs[1], vs[0]), Sub(vs[2], vs[0])))

				separation := planeSeparation(plane2, vs[1])
				cosAngle := Dot(plane2.Normal, n)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags2 |= int(ConcaveEdge3)
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags2 |= int(InverseConcaveEdge3)
				}
			}

			leftCellIndex := row*(columnCount-1) + column - 1
			if column-1 >= 0 && materialIndices[leftCellIndex] != HeightFieldHole {
				assert(0 <= leftCellIndex && leftCellIndex < cellCount, "0 <= leftCellIndex && leftCellIndex < cellCount")

				r := row
				c := column - 1

				i11 := r*columnCount + c
				i12 := i11 + 1
				i21 := (r+1)*columnCount + c
				i22 := i21 + 1

				assert(i12 == index11, "i12 == index11")
				assert(i22 == index21, "i22 == index21")

				// float h11 = heights[i11];
				h12 := heights[i12]
				h21 := heights[i21]
				h22 := heights[i22]

				x1 := float32(c)
				x2 := float32(c + 1)
				z1 := float32(r)
				z2 := float32(r + 1)

				// triangle 1
				var vs [3]Vec3
				vs[0] = Mul(scale, Vec3{x2, h22, z2})
				vs[1] = Mul(scale, Vec3{x2, h12, z1})
				vs[2] = Mul(scale, Vec3{x1, h21, z2})

				n := Normalize(Cross(Sub(vs[1], vs[0]), Sub(vs[2], vs[0])))

				separation := planeSeparation(plane1, vs[2])
				cosAngle := Dot(plane1.Normal, n)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags1 |= int(ConcaveEdge1)
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags1 |= int(InverseConcaveEdge1)
				}
			}

			rightCellIndex := row*(columnCount-1) + column + 1
			if column+1 < columnCount-1 && materialIndices[rightCellIndex] != HeightFieldHole {
				assert(0 <= rightCellIndex && rightCellIndex < cellCount, "0 <= rightCellIndex && rightCellIndex < cellCount")

				r := row
				c := column + 1

				i11 := r*columnCount + c
				i12 := i11 + 1
				i21 := (r+1)*columnCount + c
				// int i22 = i21 + 1;

				assert(i11 == index12, "i11 == index12")
				assert(i21 == index22, "i21 == index22")

				h11 := heights[i11]
				h12 := heights[i12]
				h21 := heights[i21]
				// float h22 = heights[i22];

				x1 := float32(c)
				x2 := float32(c + 1)
				z1 := float32(r)
				z2 := float32(r + 1)

				// triangle 0
				var vs [3]Vec3
				vs[0] = Mul(scale, Vec3{x1, h11, z1})
				vs[1] = Mul(scale, Vec3{x1, h21, z2})
				vs[2] = Mul(scale, Vec3{x2, h12, z1})

				n := Normalize(Cross(Sub(vs[1], vs[0]), Sub(vs[2], vs[0])))

				separation := planeSeparation(plane2, vs[2])
				cosAngle := Dot(plane2.Normal, n)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags2 |= int(ConcaveEdge1)
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags2 |= int(InverseConcaveEdge1)
				}
			}

			assert(0 <= flags1 && flags1 <= math.MaxUint8, "0 <= flags1 && flags1 <= UINT8_MAX")
			assert(0 <= flags2 && flags2 <= math.MaxUint8, "0 <= flags2 && flags2 <= UINT8_MAX")

			flags[triangleIndex1] = uint8(flags1)
			flags[triangleIndex2] = uint8(flags2)
		}
	}

	assert(triangleIndex == triangleCount, "triangleIndex == triangleCount")

	hf.Hash = 0
	hf.Hash = hash64NonZero(heightFieldBytes(hf))

	return hf
}

// PORT: _Static_assert( b3_concaveEdge3 == 4 * b3_concaveEdge1 ) and the inverse variant
// become constant expressions that overflow unless the condition holds.
const (
	_ uint = uint(ConcaveEdge3 - 4*ConcaveEdge1)
	_ uint = uint(4*ConcaveEdge1 - ConcaveEdge3)
	_ uint = uint(InverseConcaveEdge3 - 4*InverseConcaveEdge1)
	_ uint = uint(4*InverseConcaveEdge1 - InverseConcaveEdge3)
)

// getHeightFieldCellCorners is b3GetHeightFieldCellCorners.
// Decode the four corner vertices of a height field cell into local space.
// Output order matches the index naming used throughout this file:
// corners[0] = (column, row), corners[1] = (column + 1, row),
// corners[2] = (column, row + 1), corners[3] = (column + 1, row + 1).
func getHeightFieldCellCorners(hf *HeightFieldData, row, column int, corners *[4]Vec3) {
	assert(0 <= row && row < int(hf.RowCount)-1 && 0 <= column && column < int(hf.ColumnCount)-1,
		"0 <= row && row < hf->rowCount - 1 && 0 <= column && column < hf->columnCount - 1")

	columnCount := int(hf.ColumnCount)
	index11 := row*columnCount + column
	index12 := index11 + 1
	index21 := (row+1)*columnCount + column
	index22 := index21 + 1

	minHeight := hf.MinHeight
	heightScale := hf.HeightScale
	heights := GetHeightFieldCompressedHeights(hf)

	height11 := mulAdd(minHeight, heightScale, float32(heights[index11]))
	height12 := mulAdd(minHeight, heightScale, float32(heights[index12]))
	height21 := mulAdd(minHeight, heightScale, float32(heights[index21]))
	height22 := mulAdd(minHeight, heightScale, float32(heights[index22]))

	x1 := float32(column)
	x2 := float32(column + 1)
	z1 := float32(row)
	z2 := float32(row + 1)

	scale := hf.Scale
	corners[0] = Mul(scale, Vec3{x1, height11, z1})
	corners[1] = Mul(scale, Vec3{x2, height12, z1})
	corners[2] = Mul(scale, Vec3{x1, height21, z2})
	corners[3] = Mul(scale, Vec3{x2, height22, z2})
}

// getHeightFieldTriangle is b3GetHeightFieldTriangle.
func getHeightFieldTriangle(heightField *HeightFieldData, triangleIndex int) triangle {
	assert(0 <= triangleIndex, "0 <= triangleIndex")
	assert(triangleIndex < 2*int(heightField.ColumnCount-1)*int(heightField.RowCount-1),
		"triangleIndex < 2 * ( heightField->columnCount - 1 ) * ( heightField->rowCount - 1 )")

	var tri triangle
	tri.flags = int32(GetHeightFieldFlags(heightField)[triangleIndex])

	columnCount := int(heightField.ColumnCount)
	quadIndex := triangleIndex >> 1
	row := quadIndex / (columnCount - 1)
	column := quadIndex - row*(columnCount-1)

	index11 := row*columnCount + column
	index12 := index11 + 1
	index21 := (row+1)*columnCount + column
	index22 := index21 + 1

	cellIndex := row*(columnCount-1) + column

	assert(quadIndex == cellIndex, "quadIndex == cellIndex")
	assert(GetHeightFieldMaterialIndices(heightField)[cellIndex] != HeightFieldHole,
		"b3GetHeightFieldMaterialIndices( heightField )[cellIndex] != B3_HEIGHT_FIELD_HOLE")

	var corners [4]Vec3
	getHeightFieldCellCorners(heightField, row, column, &corners)

	if (triangleIndex & 1) == 0 {
		tri.vertices[0] = corners[0]
		tri.vertices[1] = corners[2]
		tri.vertices[2] = corners[1]
		tri.i1 = int32(index11)
		tri.i2 = int32(index21)
		tri.i3 = int32(index12)
	} else {
		tri.vertices[0] = corners[3]
		tri.vertices[1] = corners[1]
		tri.vertices[2] = corners[2]
		tri.i1 = int32(index22)
		tri.i2 = int32(index12)
		tri.i3 = int32(index21)
	}

	if heightField.Clockwise != 0 {
		tri.vertices[1], tri.vertices[2] = tri.vertices[2], tri.vertices[1]
		tri.i2, tri.i3 = tri.i3, tri.i2

		// Reversing winding swaps edge1 and edge3; edge2 (the diagonal) is preserved.
		flags := tri.flags
		edge1Bits := flags & int32(ConcaveEdge1|InverseConcaveEdge1)
		edge3Bits := flags & int32(ConcaveEdge3|InverseConcaveEdge3)
		flags &= ^int32(ConcaveEdge1 | ConcaveEdge3 | InverseConcaveEdge1 | InverseConcaveEdge3)
		flags |= edge1Bits << 2
		flags |= edge3Bits >> 2
		tri.flags = flags
	}

	return tri
}

// getHeightFieldMaterial is b3GetHeightFieldMaterial.
func getHeightFieldMaterial(heightField *HeightFieldData, triangleIndex int) int {
	assert(0 <= triangleIndex, "0 <= triangleIndex")
	assert(triangleIndex < 2*int(heightField.ColumnCount-1)*int(heightField.RowCount-1),
		"triangleIndex < 2 * ( heightField->columnCount - 1 ) * ( heightField->rowCount - 1 )")

	cellIndex := triangleIndex >> 1
	return int(GetHeightFieldMaterialIndices(heightField)[cellIndex])
}

// ComputeHeightFieldAABB computes the legacy authored-height bounding box of a
// transformed height field. It preserves upstream cooked-blob compatibility.
// Use ComputeHeightFieldQueryAABB for a conservative bound on quantized geometry.
func ComputeHeightFieldAABB(shape *HeightFieldData, transform Transform) AABB {
	return shape.AABB.Transform(transform)
}

// RayCastHeightField ray casts versus a height field in local space.
func RayCastHeightField(heightField *HeightFieldData, input *RayCastInput) CastOutput {
	shapeCastInput := ShapeCastInput{}
	// PORT: ( b3ShapeProxy ){ &input->origin, 1, 0.0f } aliases the input's origin.
	shapeCastInput.Proxy = ShapeProxy{unsafe.Slice(&input.Origin, 1), 1, 0.0}
	shapeCastInput.Translation = input.Translation
	shapeCastInput.MaxFraction = input.MaxFraction

	return ShapeCastHeightField(heightField, &shapeCastInput)
}

// ShapeCastHeightField shape casts versus a height field in local space.
// todo advance cast to the grid border immediately if it starts outside the row/column range
// todo terminate the cast immediately if it leaves the row/column range
func ShapeCastHeightField(heightField *HeightFieldData, input *ShapeCastInput) CastOutput {
	shapeBounds := MakeAABB(input.Proxy.Points[:input.Proxy.Count], input.Proxy.Radius)
	shapeTranslation := input.Translation
	scale := heightField.Scale

	// The shape start is the center of the proxy.
	shapeStart := shapeBounds.Center()
	shapeDelta := MulSV(input.MaxFraction, shapeTranslation)
	shapeEnd := Add(shapeStart, shapeDelta)

	result := CastOutput{}

	shapeExtents := shapeBounds.Extents()
	margin := Vec3{MaxAABBMargin(), MaxAABBMargin(), MaxAABBMargin()}
	heightBounds := queryHeightFieldLocalBounds(heightField)
	combinedBounds := AABB{Sub(Sub(heightBounds.LowerBound, shapeExtents), margin),
		Add(Add(heightBounds.UpperBound, shapeExtents), margin)}

	intersects, minFraction, maxFraction := rayCastAABB(combinedBounds, shapeStart, shapeEnd)
	if intersects == false {
		return result
	}

	// These are for walking the grid, not the triangle cast.
	// The triangle cast uses the unclamped ray and fraction.
	clampedStart := MulAdd(shapeStart, minFraction, shapeDelta)
	clampedDelta := MulSV(maxFraction-minFraction, shapeDelta)
	clampedEnd := Add(clampedStart, clampedDelta)

	// Preserve the un-shifted center sweep. clampedStart/clampedEnd get pushed out to the
	// leading box corner below to drive the grid DDA, but the swept-volume AABB used to
	// cull cells must stay centered on the actual shape path.
	centerStart := clampedStart
	centerEnd := clampedEnd

	// The grid traversal starts from the leading shape bounds corner
	var signX, signZ float32
	if shapeTranslation.X >= 0.0 {
		clampedStart.X += shapeExtents.X
		signX = 1.0
	} else {
		clampedStart.X -= shapeExtents.X
		signX = -1.0
	}

	if shapeTranslation.Z >= 0.0 {
		clampedStart.Z += shapeExtents.Z
		signZ = 1.0
	} else {
		clampedStart.Z -= shapeExtents.Z
		signZ = -1.0
	}

	// Shift the end as well
	clampedEnd = Add(clampedStart, clampedDelta)

	// Row and column range for the shape cast
	columnStart := int(floorf(clampedStart.X / scale.X))
	columnEnd := int(floorf(clampedEnd.X / scale.X))
	rowStart := int(floorf(clampedStart.Z / scale.Z))
	rowEnd := int(floorf(clampedEnd.Z / scale.Z))

	absClampedDelta := Abs(clampedDelta)

	// Precompute increments for row and column traversal.
	// The ray can be slightly tilted yet remain within a single row or column
	// once rasterized.
	var deltaAlphaX float32
	var nextFractionX float32
	var deltaColumn int

	if columnStart < columnEnd {
		assert(absClampedDelta.X > 0.0, "absClampedDelta.x > 0.0f")

		// Going forward on x columns
		deltaAlphaX = scale.X / absClampedDelta.X
		nextFractionX = (float32(scale.X*float32(columnStart+1)) - clampedStart.X) / absClampedDelta.X
		deltaColumn = 1
	} else if columnEnd < columnStart {
		assert(absClampedDelta.X > 0.0, "absClampedDelta.x > 0.0f")

		// Going backwards on x columns
		deltaAlphaX = scale.X / absClampedDelta.X
		nextFractionX = (clampedStart.X - float32(scale.X*float32(columnStart))) / absClampedDelta.X
		deltaColumn = -1
	} else {
		// Cast stays in a single column
		deltaAlphaX = 0.0
		nextFractionX = fltMax
		deltaColumn = 0
	}

	var deltaAlphaZ float32
	var nextFractionZ float32
	var deltaRow int

	if rowStart < rowEnd {
		assert(absClampedDelta.Z > 0.0, "absClampedDelta.z > 0.0f")

		// Going forward on z rows
		deltaAlphaZ = scale.Z / absClampedDelta.Z
		nextFractionZ = (float32(scale.Z*float32(rowStart+1)) - clampedStart.Z) / absClampedDelta.Z
		deltaRow = 1
	} else if rowEnd < rowStart {
		assert(absClampedDelta.Z > 0.0, "absClampedDelta.z > 0.0f")

		// Going backwards on z rows
		deltaAlphaZ = scale.Z / absClampedDelta.Z
		nextFractionZ = (clampedStart.Z - float32(scale.Z*float32(rowStart))) / absClampedDelta.Z
		deltaRow = -1
	} else {
		// Cast stays in a single row
		deltaAlphaZ = 0.0
		nextFractionZ = fltMax
		deltaRow = 0
	}

	// Column and row range for 2D projected initial shape bounds
	boxColumnHead := columnStart
	boxRowHead := rowStart

	boxColumnTail := int(floorf((clampedStart.X - float32(float32(2.0*signX)*shapeExtents.X)) / scale.X))
	boxRowTail := int(floorf((clampedStart.Z - float32(float32(2.0*signZ)*shapeExtents.Z)) / scale.Z))

	bestFraction := input.MaxFraction

	// nextFractionX / nextFractionZ advance in units of the clamped sweep
	// [minFraction, maxFraction], but bestFraction is a fraction of the full input
	// translation. Precompute the affine map from clamped space to input space so
	// the loop termination test compares like with like. Otherwise it can exit
	// early and miss a closer hit in a later cell.
	gridFractionScale := input.MaxFraction * (maxFraction - minFraction)
	gridFractionOffset := float32(input.MaxFraction * minFraction)

	rowCount := int(heightField.RowCount)
	columnCount := int(heightField.ColumnCount)
	cellCount := int(heightField.RowCount-1) * int(heightField.ColumnCount-1)

	pairInput := ShapeCastPairInput{}
	pairInput.ProxyB = input.Proxy
	pairInput.Transform = TransformIdentity
	pairInput.TranslationB = input.Translation
	pairInput.CanEncroach = input.CanEncroach

	var castBounds AABB
	castBounds.LowerBound = Sub(Min(centerStart, centerEnd), shapeExtents)
	castBounds.UpperBound = Add(Max(centerStart, centerEnd), shapeExtents)

	// The ray origin is the center of the shape.
	rayOrigin := loadV(vec3Floats(&shapeStart))
	rayTranslation := loadV(vec3Floats(&shapeTranslation))

	clockwise := heightField.Clockwise != 0

	for {
		var column1, column2 int
		if boxColumnTail < boxColumnHead {
			column1 = boxColumnTail
			column2 = boxColumnHead
		} else {
			column1 = boxColumnHead
			column2 = boxColumnTail
		}

		var row1, row2 int
		if boxRowTail < boxRowHead {
			row1 = boxRowTail
			row2 = boxRowHead
		} else {
			row1 = boxRowHead
			row2 = boxRowTail
		}

		for row := row1; row <= row2; row++ {
			if row < 0 || rowCount-1 <= row {
				continue
			}

			for column := column1; column <= column2; column++ {
				if column < 0 || columnCount-1 <= column {
					continue
				}

				cellIndex := row*(columnCount-1) + column
				assert(cellIndex < cellCount, "cellIndex < cellCount")

				materialIndex := GetHeightFieldMaterialIndices(heightField)[cellIndex]
				if materialIndex == HeightFieldHole {
					continue
				}

				var corners [4]Vec3
				getHeightFieldCellCorners(heightField, row, column, &corners)
				point11 := corners[0]
				point12 := corners[1]
				point21 := corners[2]
				point22 := corners[3]

				if clockwise {
					point12, point21 = point21, point12
				}

				// I know the min/max x and z values, but not the min/max heights.
				var bounds AABB
				bounds.LowerBound = Min(Min(point11, point12), Min(point21, point22))
				bounds.UpperBound = Max(Max(point11, point12), Max(point21, point22))

				if castBounds.Overlaps(bounds) == false {
					continue
				}

				quadIndex := row*(columnCount-1) + column
				triangleIndex1 := 2 * quadIndex
				triangleIndex2 := triangleIndex1 + 1

				if input.Proxy.Count == 1 && input.Proxy.Radius == 0.0 {
					// Ray cast
					{
						vertex1 := loadV(vec3Floats(&point11))
						vertex2 := loadV(vec3Floats(&point21))
						vertex3 := loadV(vec3Floats(&point12))

						alpha := intersectRayTriangle(rayOrigin, rayTranslation, vertex1, vertex2, vertex3)
						assert(0 <= alpha && alpha <= 1.0, "0 <= alpha && alpha <= 1.0f")

						if alpha < bestFraction {
							edge1 := Sub(point21, point11)
							edge2 := Sub(point12, point11)
							normal := Cross(edge1, edge2)

							result.Point = MulAdd(shapeStart, alpha, shapeTranslation)
							result.Normal = Normalize(normal)
							result.Fraction = alpha
							result.TriangleIndex = int32(triangleIndex1)
							result.MaterialIndex = int32(materialIndex)
							result.Hit = true
							bestFraction = alpha
						}
					}

					{
						vertex1 := loadV(vec3Floats(&point22))
						vertex2 := loadV(vec3Floats(&point12))
						vertex3 := loadV(vec3Floats(&point21))

						alpha := intersectRayTriangle(rayOrigin, rayTranslation, vertex1, vertex2, vertex3)
						assert(0 <= alpha && alpha <= 1.0, "0 <= alpha && alpha <= 1.0f")

						if alpha < bestFraction {
							edge1 := Sub(point22, point21)
							edge2 := Sub(point12, point21)
							normal := Cross(edge1, edge2)

							result.Point = MulAdd(shapeStart, alpha, shapeTranslation)
							result.Normal = Normalize(normal)
							result.Fraction = alpha
							result.TriangleIndex = int32(triangleIndex2)
							result.MaterialIndex = int32(materialIndex)
							result.Hit = true
							bestFraction = alpha
						}
					}
				} else {
					// Shape cast
					{
						// shapeStart is the center of the proxy.
						signedVolume := signedVolume(point11, point21, point12, shapeStart)

						if signedVolume >= 0.0 {
							// Shift origin to first vertex
							origin := point11
							triangleVertices := [3]Vec3{Vec3Zero, Sub(point21, origin), Sub(point12, origin)}
							pairInput.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}
							pairInput.MaxFraction = bestFraction
							pairInput.Transform.P = Neg(origin)

							pairOutput := ShapeCast(&pairInput)

							if pairOutput.Hit {
								bestFraction = pairOutput.Fraction
								result = pairOutput
								result.Point = Add(result.Point, origin)
								result.TriangleIndex = int32(triangleIndex1)
								result.MaterialIndex = int32(materialIndex)
							}
						}
					}

					{
						// shapeStart is the center of the proxy.
						signedVolume := signedVolume(point21, point22, point12, shapeStart)

						if signedVolume >= 0.0 {
							// Shift origin to first vertex
							origin := point21
							triangleVertices := [3]Vec3{Vec3Zero, Sub(point22, origin), Sub(point12, origin)}
							pairInput.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}
							pairInput.MaxFraction = bestFraction
							pairInput.Transform.P = Neg(origin)

							pairOutput := ShapeCast(&pairInput)

							if pairOutput.Hit {
								bestFraction = pairOutput.Fraction
								result = pairOutput
								result.Point = Add(result.Point, origin)
								result.TriangleIndex = int32(triangleIndex2)
								result.MaterialIndex = int32(materialIndex)
							}
						}
					}
				}
			}
		}

		// These fractions always increase to guarantee the loop eventually exits.
		// Map them from clamped-sweep space into input-translation space before
		// comparing against bestFraction.
		inputFractionX := fltMax
		if nextFractionX != fltMax {
			inputFractionX = mulAdd(gridFractionOffset, nextFractionX, gridFractionScale)
		}
		inputFractionZ := fltMax
		if nextFractionZ != fltMax {
			inputFractionZ = mulAdd(gridFractionOffset, nextFractionZ, gridFractionScale)
		}
		if inputFractionX > bestFraction && inputFractionZ > bestFraction {
			break
		}

		// Advance the cast to the next column or row
		if nextFractionX <= nextFractionZ {
			if boxColumnHead == columnEnd {
				// Hit the end already
				break
			}

			// Advance to next column
			boxColumnHead += deltaColumn

			// Build a single column to cast
			boxColumnTail = boxColumnHead

			if shapeExtents.Z == 0.0 {
				// Single row
				boxRowTail = boxRowHead
			} else {
				// Rasterize shape row
				rowIntercept := mulAdd(clampedStart.Z, nextFractionX, clampedDelta.Z)
				boxRowTail = int(floorf((rowIntercept - float32(float32(2.0*signZ)*shapeExtents.Z)) / scale.Z))
			}

			nextFractionX += deltaAlphaX
		} else {
			if boxRowHead == rowEnd {
				// Hit the end already
				break
			}

			// Advance to next row
			boxRowHead += deltaRow

			// Build a single row to cast
			boxRowTail = boxRowHead

			if shapeExtents.X == 0.0 {
				// Single column
				boxColumnTail = boxColumnHead
			} else {
				// Rasterize shape column
				columnIntercept := mulAdd(clampedStart.X, nextFractionZ, clampedDelta.X)
				boxColumnTail = int(floorf((columnIntercept - float32(float32(2.0*signX)*shapeExtents.X)) / scale.X))
			}

			nextFractionZ += deltaAlphaZ
		}
	}

	return result
}

// OverlapHeightField overlaps a shape versus a height field.
func OverlapHeightField(shape *HeightFieldData, shapeTransform Transform, proxy *ShapeProxy) bool {
	var buffer [MaxShapeCastPoints]Vec3
	localProxy := makeLocalProxy(proxy, shapeTransform, buffer[:])
	aabb := computeProxyAABB(&localProxy)

	scale := shape.Scale
	minRow := int(floorf(aabb.LowerBound.Z / scale.Z))
	maxRow := int(floorf(aabb.UpperBound.Z / scale.Z))
	minCol := int(floorf(aabb.LowerBound.X / scale.X))
	maxCol := int(floorf(aabb.UpperBound.X / scale.X))

	boundsMin := loadV(vec3Floats(&aabb.LowerBound))
	boundsMax := loadV(vec3Floats(&aabb.UpperBound))
	boundsCenter := mulV(halfVConst, addV(boundsMin, boundsMax))
	boundsExtent := subV(boundsMax, boundsCenter)

	var input DistanceInput
	input.ProxyB = localProxy
	input.Transform = TransformIdentity
	input.UseRadii = true

	cache := SimplexCache{}

	// Outer loop on rows and inner loop on columns so that triangle indices increase monotonically.
	for row := minRow; row <= maxRow; row++ {
		if row < 0 || int(shape.RowCount)-1 <= row {
			continue
		}

		for column := minCol; column <= maxCol; column++ {
			if column < 0 || int(shape.ColumnCount)-1 <= column {
				continue
			}

			cellIndex := row*int(shape.ColumnCount-1) + column
			assert(cellIndex < int(shape.RowCount-1)*int(shape.ColumnCount-1),
				"cellIndex < ( shape->rowCount - 1 ) * ( shape->columnCount - 1 )")
			material := GetHeightFieldMaterialIndices(shape)[cellIndex]
			if material == HeightFieldHole {
				continue
			}

			var corners [4]Vec3
			getHeightFieldCellCorners(shape, row, column, &corners)
			point11 := corners[0]
			point12 := corners[1]
			point21 := corners[2]
			point22 := corners[3]

			v11 := loadV(vec3Floats(&point11))
			v12 := loadV(vec3Floats(&point12))
			v21 := loadV(vec3Floats(&point21))
			v22 := loadV(vec3Floats(&point22))

			if testBoundsTriangleOverlap(boundsCenter, boundsExtent, v11, v21, v12) {
				triangleVertices := [3]Vec3{point11, point21, point12}
				input.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}

				// reset the cache
				cache.Count = 0

				// get distance between triangle and query shape
				output := ShapeDistance(&input, &cache, nil)

				tolerance := 0.1 * LinearSlop()
				if output.Distance < tolerance {
					// overlap detected
					return true
				}
			}

			if testBoundsTriangleOverlap(boundsCenter, boundsExtent, v21, v22, v12) {
				triangleVertices := [3]Vec3{point22, point12, point21}
				input.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}

				// reset the cache
				cache.Count = 0

				// get distance between triangle and query shape
				output := ShapeDistance(&input, &cache, nil)

				tolerance := 0.1 * LinearSlop()
				if output.Distance < tolerance {
					// overlap detected
					return true
				}
			}
		}
	}

	return false
}

// QueryHeightField queries a height field for triangles overlapping a local AABB.
func QueryHeightField(heightField *HeightFieldData, bounds AABB, fcn MeshQueryFcn, context any) {
	scale := heightField.Scale

	minRow := int(floorf(bounds.LowerBound.Z / scale.Z))
	maxRow := int(floorf(bounds.UpperBound.Z / scale.Z))
	minCol := int(floorf(bounds.LowerBound.X / scale.X))
	maxCol := int(floorf(bounds.UpperBound.X / scale.X))

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for row := minRow; row <= maxRow; row++ {
		if row < 0 || int(heightField.RowCount)-1 <= row {
			continue
		}

		for column := minCol; column <= maxCol; column++ {
			if column < 0 || int(heightField.ColumnCount)-1 <= column {
				continue
			}

			cellIndex := row*int(heightField.ColumnCount-1) + column
			assert(cellIndex < int(heightField.RowCount-1)*int(heightField.ColumnCount-1),
				"cellIndex < ( heightField->rowCount - 1 ) * ( heightField->columnCount - 1 )")
			material := GetHeightFieldMaterialIndices(heightField)[cellIndex]
			if material == HeightFieldHole {
				continue
			}

			var corners [4]Vec3
			getHeightFieldCellCorners(heightField, row, column, &corners)
			point11 := corners[0]
			point12 := corners[1]
			point21 := corners[2]
			point22 := corners[3]

			// I know the min/max x and z values, but not the min/max heights.
			// This could be done with no branching in SIMD.
			var cellBound AABB
			cellBound.LowerBound = Min(Min(point11, point12), Min(point21, point22))
			cellBound.UpperBound = Max(Max(point11, point12), Max(point21, point22))

			if bounds.Overlaps(cellBound) {
				quadIndex := row*int(heightField.ColumnCount-1) + column
				triangleIndex := 2 * quadIndex

				// PORT: the callback results are ignored, as upstream does.
				if heightField.Clockwise != 0 {
					fcn(point11, point12, point21, triangleIndex, context)
					fcn(point22, point21, point12, triangleIndex+1, context)
				} else {
					fcn(point11, point21, point12, triangleIndex, context)
					fcn(point22, point12, point21, triangleIndex+1, context)
				}
			}
		}
	}
}

// collideMoverAndHeightField is b3CollideMoverAndHeightField.
// PORT: b3PlaneResult* planes with capacity becomes a slice; capacity is len(planes).
func collideMoverAndHeightField(planes []PlaneResult, shape *HeightFieldData, mover *Capsule) int {
	capacity := len(planes)

	distanceInput := DistanceInput{}
	distanceInput.ProxyB = ShapeProxy{capsulePoints(mover), 2, 0.0}
	distanceInput.Transform = TransformIdentity
	distanceInput.UseRadii = false

	cache := SimplexCache{}

	radius := mover.Radius
	center := Lerp(mover.Center1, mover.Center2, 0.5)
	center1 := loadV(vec3Floats(&mover.Center1))
	center2 := loadV(vec3Floats(&mover.Center2))
	r := splatV(radius)
	boundsMin := subV(minV(center1, center2), r)
	boundsMax := addV(maxV(center1, center2), r)
	boundsCenter := mulV(halfVConst, addV(boundsMin, boundsMax))
	boundsExtent := subV(boundsMax, boundsCenter)

	localMinX := getXV(boundsMin)
	localMinZ := getZV(boundsMin)
	localMaxX := getXV(boundsMax)
	localMaxZ := getZV(boundsMax)

	scale := shape.Scale
	minRow := int(floorf(localMinZ / scale.Z))
	maxRow := int(floorf(localMaxZ / scale.Z))
	minCol := int(floorf(localMinX / scale.X))
	maxCol := int(floorf(localMaxX / scale.X))

	clockWise := shape.Clockwise != 0

	planeCount := 0

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for row := minRow; row <= maxRow; row++ {
		if row < 0 || int(shape.RowCount)-1 <= row {
			continue
		}

		for column := minCol; column <= maxCol; column++ {
			if column < 0 || int(shape.ColumnCount)-1 <= column {
				continue
			}

			cellIndex := row*int(shape.ColumnCount-1) + column
			assert(cellIndex < int(shape.RowCount-1)*int(shape.ColumnCount-1),
				"cellIndex < ( shape->rowCount - 1 ) * ( shape->columnCount - 1 )")
			material := GetHeightFieldMaterialIndices(shape)[cellIndex]
			if material == HeightFieldHole {
				continue
			}

			var corners [4]Vec3
			getHeightFieldCellCorners(shape, row, column, &corners)
			point11 := corners[0]
			point12 := corners[1]
			point21 := corners[2]
			point22 := corners[3]

			if clockWise {
				point12, point21 = point21, point12
			}

			v11 := loadV(vec3Floats(&point11))
			v12 := loadV(vec3Floats(&point12))
			v21 := loadV(vec3Floats(&point21))
			v22 := loadV(vec3Floats(&point22))

			overlap1 := testBoundsTriangleOverlap(boundsCenter, boundsExtent, v11, v21, v12)
			if overlap1 {
				signedVolume := signedVolume(point11, point21, point12, center)

				// Front side?
				if signedVolume >= 0.0 {
					triangleVertices := [3]Vec3{point11, point21, point12}
					distanceInput.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}

					// reset the cache
					cache.Count = 0

					// get distance between triangle and mover
					distanceOutput := ShapeDistance(&distanceInput, &cache, nil)

					if distanceOutput.Distance == 0.0 {
						// deep overlap
					} else if distanceOutput.Distance <= mover.Radius {
						triangleIndex := 2 * cellIndex
						plane := Plane{distanceOutput.Normal, mover.Radius - distanceOutput.Distance}
						planes[planeCount] = PlaneResult{plane, distanceOutput.PointA, int32(triangleIndex), 0, int32(material)}
						planeCount += 1

						if planeCount == capacity {
							return planeCount
						}
					}
				}
			}

			overlap2 := testBoundsTriangleOverlap(boundsCenter, boundsExtent, v21, v22, v12)
			if overlap2 {
				signedVolume := signedVolume(point22, point12, point21, center)

				// Front side?
				if signedVolume >= 0.0 {
					triangleVertices := [3]Vec3{point22, point12, point21}
					distanceInput.ProxyA = ShapeProxy{triangleVertices[:], 3, 0.0}

					// reset the cache
					cache.Count = 0

					// get distance between triangle and mover
					distanceOutput := ShapeDistance(&distanceInput, &cache, nil)

					if distanceOutput.Distance == 0.0 {
						// deep overlap
					} else if distanceOutput.Distance <= mover.Radius {
						triangleIndex := 2*cellIndex + 1
						plane := Plane{distanceOutput.Normal, mover.Radius - distanceOutput.Distance}
						planes[planeCount] = PlaneResult{plane, distanceOutput.PointA, int32(triangleIndex), 0, int32(material)}
						planeCount += 1

						if planeCount == capacity {
							return planeCount
						}
					}
				}
			}
		}
	}

	return planeCount
}

// CreateGrid creates a grid as a height field.
func CreateGrid(rowCount, columnCount int, scale Vec3, makeHoles bool) *HeightFieldData {
	heightCount := rowCount * columnCount
	heights := make([]float32, heightCount)

	for i := range rowCount {
		for j := range columnCount {
			k := i*columnCount + j
			heights[k] = 0.0
		}
	}

	cellCount := (rowCount - 1) * (columnCount - 1)
	materialIndices := make([]uint8, cellCount)

	for i := range rowCount - 1 {
		for j := range columnCount - 1 {
			k := i*(columnCount-1) + j

			if makeHoles && k > 0 && k%16 == 0 {
				materialIndices[k] = HeightFieldHole
			} else {
				materialIndices[k] = 0
			}
		}
	}

	data := HeightFieldDef{}
	data.Heights = heights
	data.MaterialIndices = materialIndices
	data.Scale = scale
	data.CountX = int32(columnCount)
	data.CountZ = int32(rowCount)
	data.GlobalMinimumHeight = -256.0
	data.GlobalMaximumHeight = 256.0
	data.ClockwiseWinding = false

	heightField := CreateHeightField(&data)

	return heightField
}

// CreateWave creates a wave grid as a height field.
func CreateWave(rowCount, columnCount int, scale Vec3, rowFrequency, columnFrequency float32, makeHoles bool) *HeightFieldData {
	heightCount := rowCount * columnCount
	heights := make([]float32, heightCount)

	omegaZ := 2.0 * Pi * rowFrequency
	omegaX := 2.0 * Pi * columnFrequency

	cs := CosSin{}

	for i := range rowCount {
		cs = ComputeCosSin(omegaZ * float32(i))
		rowHeight := cs.Sine

		for j := range columnCount {
			k := i*columnCount + j
			cs = ComputeCosSin(omegaX * float32(j))
			columnHeight := cs.Sine
			heights[k] = rowHeight * columnHeight
		}
	}

	cellCount := (rowCount - 1) * (columnCount - 1)
	materialIndices := make([]uint8, cellCount)

	for i := range rowCount - 1 {
		for j := range columnCount - 1 {
			k := i*(columnCount-1) + j

			if makeHoles && k > 0 && k%16 == 0 {
				materialIndices[k] = HeightFieldHole
			} else {
				materialIndices[k] = 0
			}
		}
	}

	data := HeightFieldDef{}
	data.Heights = heights
	data.MaterialIndices = materialIndices
	data.Scale = scale
	data.CountX = int32(columnCount)
	data.CountZ = int32(rowCount)
	data.GlobalMinimumHeight = -256.0
	data.GlobalMaximumHeight = 256.0
	data.ClockwiseWinding = false

	heightField := CreateHeightField(&data)

	return heightField
}

// DestroyHeightField destroys a height field.
// PORT: the Go garbage collector owns the blob; this is a no-op.
func DestroyHeightField(heightField *HeightFieldData) {
	_ = heightField
}

// DumpHeightData saves input height data to a file.
// PORT: fopen/fprintf become os.Create and fmt.Fprintf with the same formats; a file that
// cannot be created is silently skipped as upstream.
func DumpHeightData(data *HeightFieldDef, fileName string) {
	file, err := os.Create(fileName)
	if err != nil {
		return
	}
	defer file.Close()

	w := bufio.NewWriter(file)
	defer w.Flush()

	fmt.Fprintf(w, "%d %d\n", data.CountX, data.CountZ)
	fmt.Fprintf(w, "%.9f %.9f %.9f\n", float64(data.Scale.X), float64(data.Scale.Y), float64(data.Scale.Z))
	fmt.Fprintf(w, "%.9f %.9f\n", float64(data.GlobalMinimumHeight), float64(data.GlobalMaximumHeight))
	clockwiseWinding := 0
	if data.ClockwiseWinding {
		clockwiseWinding = 1
	}
	fmt.Fprintf(w, "%d\n", clockwiseWinding)

	heightCount := int(data.CountX * data.CountZ)
	for i := range heightCount {
		fmt.Fprintf(w, "%.9f\n", float64(data.Heights[i]))
	}

	materialCount := int((data.CountX - 1) * (data.CountZ - 1))
	for i := range materialCount {
		fmt.Fprintf(w, "%d\n", data.MaterialIndices[i])
	}
}

// LoadHeightField creates a height field by loading a previously saved height data.
// PORT: fscanf becomes fmt.Fscan (whitespace separated, float32 parsing is correctly rounded
// like strtof).
func LoadHeightField(fileName string) *HeightFieldData {
	file, err := os.Open(fileName)
	if err != nil {
		return nil
	}

	r := bufio.NewReader(file)

	data := HeightFieldDef{}

	// Read dimensions
	if n, _ := fmt.Fscan(r, &data.CountX, &data.CountZ); n != 2 {
		file.Close()
		return nil
	}

	// Read scale
	if n, _ := fmt.Fscan(r, &data.Scale.X, &data.Scale.Y, &data.Scale.Z); n != 3 {
		file.Close()
		return nil
	}

	// Read global height bounds
	if n, _ := fmt.Fscan(r, &data.GlobalMinimumHeight, &data.GlobalMaximumHeight); n != 2 {
		file.Close()
		return nil
	}

	// Read clockwise winding
	var clockwise int
	if n, _ := fmt.Fscan(r, &clockwise); n != 1 {
		file.Close()
		return nil
	}
	data.ClockwiseWinding = clockwise != 0

	// Allocate and read height data
	heightCount := int(data.CountX * data.CountZ)
	data.Heights = make([]float32, heightCount)

	for i := range heightCount {
		if n, _ := fmt.Fscan(r, &data.Heights[i]); n != 1 {
			file.Close()
			return nil
		}
	}

	// Allocate and read material indices
	materialCount := int((data.CountX - 1) * (data.CountZ - 1))
	data.MaterialIndices = make([]uint8, materialCount)

	for i := range materialCount {
		var materialIndex int
		if n, _ := fmt.Fscan(r, &materialIndex); n != 1 {
			file.Close()
			return nil
		}
		data.MaterialIndices[i] = uint8(materialIndex)
	}

	file.Close()

	// Create height field from loaded data
	heightField := CreateHeightField(&data)

	return heightField
}

// Height field blob layout lock.
// PORT: size lock with no upstream counterpart; the byte image depends on the C layout.
const (
	_ uintptr = unsafe.Sizeof(HeightFieldData{}) - 96
	_ uintptr = 96 - unsafe.Sizeof(HeightFieldData{})
)
