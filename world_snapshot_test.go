// Go-only test: no upstream counterpart (upstream covers world_snapshot.c through the replay tests
// of test_recording.c). Serializes the world of the recording golden scene (recording_golden_test.go)
// after it ran, restores the image into a fresh world the way the player does, and checks that the
// restored world hashes the same, serializes to the same image and steps in lock step with the
// original. C images are rejected by the layout hash.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"os"
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

// snapTestReader builds the reader registry the player loads from a recording's registry block:
// one slot per entry, the bytes in an 8-byte aligned blob.
func snapTestReader(rec *Recording) *recReader {
	rdr := &recReader{}
	for i := range rec.registry.entries {
		e := &rec.registry.entries[i]
		blob := blobAlloc(int(e.byteCount))
		bytes := unsafe.Slice((*byte)(blob), e.byteCount)
		copy(bytes, e.bytes)
		rdr.slots = append(rdr.slots, registrySlot{kind: e.kind, byteCount: e.byteCount, bytes: bytes})
	}
	rdr.slotCount = len(rdr.slots)
	return rdr
}

func snapTestImage(w *world) ([]byte, *Recording) {
	rec := CreateRecording(0)
	buf := recBuffer{}
	n := serializeWorld(w, &buf, rec)
	Expect(n).To(Equal(int(buf.size)))
	return append([]byte(nil), recBufBytes(&buf)...), rec
}

var _ = Describe("WorldSnapshotTest", func() {
	It("RoundTrip", func() {
		_, worldId := recGoldenRun(1)
		defer worldId.Destroy()
		w := getWorldFromId(worldId)

		image, rec := snapTestImage(w)
		rdr := snapTestReader(rec)

		worldDef := DefaultWorldDef()
		shellId := CreateWorld(&worldDef)
		defer shellId.Destroy()
		shell := getWorldFromId(shellId)

		Expect(deserializeIntoShell(image, shell, rdr)).To(BeTrue())
		Expect(hashWorldState(shell)).To(Equal(hashWorldState(w)))

		// The restored world serializes to the same image with the same geometry ids
		again, rec2 := snapTestImage(shell)
		Expect(again).To(Equal(image))
		Expect(len(rec2.registry.entries)).To(Equal(len(rec.registry.entries)))
		for i := range rec.registry.entries {
			Expect(rec2.registry.entries[i].bytes).To(Equal(rec.registry.entries[i].bytes))
		}

		// Both worlds step in lock step
		for step := 0; step < 120; step++ {
			worldId.Step(1.0/60.0, 4)
			shellId.Step(1.0/60.0, 4)
			Expect(hashWorldState(shell)).To(Equal(hashWorldState(w)), "step %d", step)
		}
		Expect(shellId.GetAwakeBodyCount()).To(Equal(worldId.GetAwakeBodyCount()))
		Expect(shellId.GetCounters().ContactCount).To(Equal(worldId.GetCounters().ContactCount))

		// Restoring again over the populated shell (a backward seek) reproduces the image
		Expect(deserializeIntoShell(image, shell, rdr)).To(BeTrue())
		third, _ := snapTestImage(shell)
		Expect(third).To(Equal(image))
	})

	It("RejectsBadImages", func() {
		_, worldId := recGoldenRun(1)
		defer worldId.Destroy()
		w := getWorldFromId(worldId)

		image, rec := snapTestImage(w)
		rdr := snapTestReader(rec)

		worldDef := DefaultWorldDef()
		shellId := CreateWorld(&worldDef)
		defer shellId.Destroy()
		shell := getWorldFromId(shellId)

		// A C seed snapshot fails the layout hash
		cdata, err := os.ReadFile("testdata/recording_golden.b3rec")
		Expect(err).NotTo(HaveOccurred())
		_, csnap, _, _ := recGoldenParts(cdata)
		Expect(deserializeIntoShell(csnap, shell, rdr)).To(BeFalse())

		// Bad magic and short images
		bad := append([]byte(nil), image...)
		binary.LittleEndian.PutUint32(bad[4:8], 11)
		Expect(deserializeIntoShell(bad, shell, rdr)).To(BeFalse())
		bad = append([]byte(nil), image...)
		bad[0] ^= 0xFF
		Expect(deserializeIntoShell(bad, shell, rdr)).To(BeFalse())
		Expect(deserializeIntoShell(image[:8], shell, rdr)).To(BeFalse())
		Expect(deserializeIntoShell(nil, shell, rdr)).To(BeFalse())

		// A truncated image runs out of bytes
		Expect(deserializeIntoShell(image[:len(image)/2], shell, rdr)).To(BeFalse())

		// A missing geometry slot fails the load
		Expect(deserializeIntoShell(image, shell, &recReader{})).To(BeFalse())
	})
})
