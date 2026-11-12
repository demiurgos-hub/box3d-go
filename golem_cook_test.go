package box3d

import "testing"

func TestGolemCookedBytesAreOwned(t *testing.T) {
	hull := MakeBoxHull(1, 2, 3)
	mesh := golemFlatMesh()
	hb, mb := MarshalHullData(&hull.Base), MarshalMeshData(mesh)
	if len(hb) != int(hull.Base.ByteCount) || len(mb) != int(mesh.ByteCount) {
		t.Fatal("cooked image size")
	}
	hb[0] ^= 0xff
	mb[0] ^= 0xff
	if MarshalHullData(&hull.Base)[0] == hb[0] || MarshalMeshData(mesh)[0] == mb[0] {
		t.Fatal("returned cooked bytes alias backend geometry")
	}
	if MarshalHullData(nil) != nil || MarshalMeshData(nil) != nil {
		t.Fatal("nil cooked image")
	}
}
