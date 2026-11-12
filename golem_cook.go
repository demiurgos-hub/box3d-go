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
