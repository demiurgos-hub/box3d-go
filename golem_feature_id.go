// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

// MaxFeatureIndex bounds each contact edge index. Bit 31 remains reserved, so
// the native invalid-feature sentinel (all bits set) cannot name a real pair.
const MaxFeatureIndex = (1 << 14) - 1

// EncodeFeaturePair returns the collision-free identity of two bounded edges.
// Bits 0..7 and 16..23 retain the original low-eight index bits; bits 8 and 24
// retain Owner2 and Owner1. Index2 bits 8..13 occupy ID bits 9..14, and Index1
// bits 8..13 occupy ID bits 25..30. Bits 15 and 31 are zero. Owners must be 0 or
// 1. All legacy pairs retain their exact numeric IDs; invalid inputs return false.
func EncodeFeaturePair(pair FeaturePair) (uint32, bool) {
	if pair.Owner1 > 1 || pair.Owner2 > 1 || pair.Index1 > MaxFeatureIndex || pair.Index2 > MaxFeatureIndex {
		return 0, false
	}
	return uint32(pair.Index2&255) | uint32(pair.Owner2)<<8 | uint32(pair.Index2>>8)<<9 |
		uint32(pair.Index1&255)<<16 | uint32(pair.Owner1)<<24 | uint32(pair.Index1>>8)<<25, true
}

// DecodeFeaturePair reverses EncodeFeaturePair and rejects reserved-bit IDs,
// including the native invalid-feature sentinel. It preserves edge owners.
func DecodeFeaturePair(id uint32) (FeaturePair, bool) {
	if id&0x80008000 != 0 {
		return FeaturePair{}, false
	}
	return FeaturePair{
		Owner1: uint8((id >> 24) & 1), Index1: uint16((id>>16)&255 | ((id>>25)&63)<<8),
		Owner2: uint8((id >> 8) & 1), Index2: uint16(id&255 | ((id>>9)&63)<<8),
	}, true
}
