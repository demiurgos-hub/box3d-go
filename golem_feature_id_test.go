package box3d

import "testing"

func TestGolemFeatureEncodingLegacyAndSentinel(t *testing.T) {
	for a := uint16(0); a < 256; a++ {
		for b := uint16(0); b < 256; b++ {
			for owners := uint8(0); owners < 4; owners++ {
				pair := FeaturePair{Owner1: owners >> 1, Index1: a, Owner2: owners & 1, Index2: b}
				id, ok := EncodeFeaturePair(pair)
				legacy := uint32(pair.Owner1)<<24 | uint32(a)<<16 | uint32(pair.Owner2)<<8 | uint32(b)
				decoded, valid := DecodeFeaturePair(id)
				if !ok || !valid || id != legacy || decoded != pair {
					t.Fatalf("legacy identity/roundtrip changed: %+v %x", pair, id)
				}
			}
		}
	}
	for _, id := range []uint32{0xffffffff, 1 << 15, 1 << 31} {
		if _, ok := DecodeFeaturePair(id); ok {
			t.Fatalf("accepted reserved ID %x", id)
		}
	}
	for _, pair := range []FeaturePair{{Owner1: 2}, {Owner2: 255}, {Index1: MaxFeatureIndex + 1}, {Index2: MaxFeatureIndex + 1}} {
		if _, ok := EncodeFeaturePair(pair); ok {
			t.Fatalf("truncated invalid pair %+v", pair)
		}
	}
}

func TestGolemFeatureEncodingExtendedInjective(t *testing.T) {
	values := []uint16{0, 1, 127, 128, 255, 256, 511, 512, 767, 768, 1023, 1535, MaxFeatureIndex}
	seen := map[uint32]FeaturePair{}
	for _, a := range values {
		for _, b := range values {
			for owners := uint8(0); owners < 4; owners++ {
				pair := FeaturePair{Owner1: owners >> 1, Index1: a, Owner2: owners & 1, Index2: b}
				id, ok := EncodeFeaturePair(pair)
				decoded, valid := DecodeFeaturePair(id)
				if !ok || !valid || decoded != pair {
					t.Fatal("extended roundtrip", pair, id, decoded)
				}
				if previous, exists := seen[id]; exists && previous != pair {
					t.Fatal("feature alias", previous, pair, id)
				}
				seen[id] = pair
			}
		}
	}
}
