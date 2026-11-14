// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// MarshalHullData copies the backend-specific cooked image of a hull produced by
// this package. Nil returns nil. The image is version/layout dependent, not a
// portable geometry description, and this API does not accept foreign pointers.
func MarshalHullData(hull *HullData) []byte {
	if hull == nil {
		return nil
	}
	return append([]byte(nil), hullBytes(hull)...)
}

// MarshalMeshData copies the backend-specific cooked BVH image of a mesh produced
// by this package. Nil returns nil. Its format is version/layout dependent; retain
// portable source separately. This API does not accept foreign pointers.
func MarshalMeshData(mesh *MeshData) []byte {
	if mesh == nil {
		return nil
	}
	return append([]byte(nil), meshBytes(mesh)...)
}

// MarshalHeightFieldData copies the backend-specific image of a height field
// produced by this package. Nil returns nil. Its format is version dependent.
func MarshalHeightFieldData(hf *HeightFieldData) []byte {
	if hf == nil {
		return nil
	}
	return append([]byte(nil), heightFieldBytes(hf)...)
}

// GetHeightFieldTriangles returns decompressed local grid points, one per grid
// point, and the solid triangles in native layout and winding, three indices
// per triangle in triangle-index order.
func GetHeightFieldTriangles(hf *HeightFieldData) ([]Vec3, []int32) {
	columns, rows := int(hf.ColumnCount), int(hf.RowCount)
	heights := GetHeightFieldCompressedHeights(hf)
	points := make([]Vec3, columns*rows)
	for i := range points {
		points[i] = Mul(hf.Scale, Vec3{float32(i % columns), mulAdd(hf.MinHeight, hf.HeightScale, float32(heights[i])), float32(i / columns)})
	}
	materials := GetHeightFieldMaterialIndices(hf)
	var indices []int32
	for cell := range materials {
		if materials[cell] == HeightFieldHole {
			continue
		}
		i11, i12, i21, i22 := heightFieldCellIndices(hf, cell/(columns-1), cell%(columns-1))
		first, second := [3]int{i11, i21, i12}, [3]int{i22, i12, i21}
		if hf.Clockwise != 0 {
			first[1], first[2] = first[2], first[1]
			second[1], second[2] = second[2], second[1]
		}
		indices = append(indices, int32(first[0]), int32(first[1]), int32(first[2]), int32(second[0]), int32(second[1]), int32(second[2]))
	}
	return points, indices
}
