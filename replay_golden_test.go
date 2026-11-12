// Fork reference: testdata/golem_post_toi applies only the documented native
// post-TOI contact correction to pinned C. Original upstream fixtures remain intact.
// Go-only test: no upstream counterpart. Replays a recording made by the C library
// (testdata/replay_golden.b3rec, written by testdata/replay_golden_gen.c from the pinned upstream
// sources efca0c184d40fd6a82016983f1601c4d23965867 compiled with -ffp-contract=off) and compares the
// final state of every body slot bit for bit with the original C run (testdata/replay_golden.txt).
// The recording is a joint-free scene started on the empty world: mesh, height field, hull,
// sphere, capsule and compound shapes, body and shape mutators, a late create, a destroy, an
// explosion, and every spatial query kind (some tagged). Each recorded StateHash (one per step) is
// checked against the Go world as the op stream replays, so a pass means every step matches C.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const replayGoldenSteps = 180

func replayGoldenVec(sb *strings.Builder, v Vec3) {
	fmt.Fprintf(sb, " %08x %08x %08x", math.Float32bits(v.X), math.Float32bits(v.Y), math.Float32bits(v.Z))
}

// replayGoldenPrintWorld mirrors PrintWorld in testdata/replay_golden_gen.c.
func replayGoldenPrintWorld(sb *strings.Builder, tag string, worldId WorldID) {
	world := getWorldFromId(worldId)
	for i := range world.bodies {
		if int(world.bodies[i].id) != i {
			continue
		}
		bodyId := makeBodyId(world, i)
		xf := bodyId.GetTransform()
		v := bodyId.GetLinearVelocity()
		w := bodyId.GetAngularVelocity()
		fmt.Fprintf(sb, "%s body %d %d |", tag, i, bodyId.Generation)
		replayGoldenVec(sb, ToVec3(xf.P))
		replayGoldenVec(sb, xf.Q.V)
		fmt.Fprintf(sb, " %08x", math.Float32bits(xf.Q.S))
		replayGoldenVec(sb, v)
		replayGoldenVec(sb, w)
		awake := 0
		if bodyId.IsAwake() {
			awake = 1
		}
		fmt.Fprintf(sb, " %d %d\n", awake, int(bodyId.GetType()))
	}
	fmt.Fprintf(sb, "%s world | %d %016x\n", tag, worldId.GetAwakeBodyCount(), hashWorldState(world))
}

func replayGoldenData() []byte {
	data, err := os.ReadFile("testdata/golem_hull_topology/replay_golden.b3rec")
	Expect(err).NotTo(HaveOccurred())
	return data
}

// replayGoldenWant returns the golden lines with the given tag, the tag replaced by "original".
func replayGoldenWant(tag string) []string {
	data, err := os.ReadFile("testdata/golem_hull_topology/replay_golden.txt")
	Expect(err).NotTo(HaveOccurred())
	var want []string
	for _, line := range strings.Split(strings.TrimRight(string(data), "\n"), "\n") {
		if strings.HasPrefix(line, tag+" ") {
			want = append(want, "original"+strings.TrimPrefix(line, tag))
		}
	}
	Expect(want).NotTo(BeEmpty())
	return want
}

func replayGoldenCompare(got string, want []string) {
	lines := strings.Split(strings.TrimRight(got, "\n"), "\n")
	for i := range min(len(want), len(lines)) {
		if want[i] != lines[i] {
			Fail(fmt.Sprintf("first mismatch at line %d:\n got  %s\n want %s", i+1, lines[i], want[i]))
		}
	}
	Expect(len(lines)).To(Equal(len(want)))
}

// replayGoldenHeadless replays the C recording with a reader that has no player (the headless
// path of b3RecReader, owner NULL): header, registry, the empty-world seed and the op stream.
func replayGoldenHeadless(workerCount int) string {
	data := replayGoldenData()
	hdr := recDecodeHeader(data)
	Expect(hdr.magic).To(Equal(recMagic))
	Expect(int(hdr.versionMajor)).To(Equal(recVersionMajor))
	Expect(int(hdr.pointerWidth)).To(Equal(recPointerWidth))
	Expect(hdr.lengthScale).To(Equal(float32(1.0)))
	headerEnd := recHeaderSize + int(hdr.snapshotSize)
	registryEnd := int(hdr.registryOffset)
	Expect(registryEnd).To(BeNumerically(">", headerEnd))

	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	var rdr recReader
	rdr.data = data
	rdr.size = len(data)
	rdr.cursor = headerEnd
	rdr.replayWorldId = worldId
	rdr.ok = true
	Expect(recLoadSlots(&rdr, data, len(data), hdr.registryOffset, hdr.registryByteCount)).To(BeTrue())
	// mesh, height field, the static box, compound, cube, rock, cylinder and the small box
	Expect(rdr.slotCount).To(Equal(8))
	// "probe" with id 42, and the "sweep" and per-step ray tags
	Expect(rdr.tagCount).To(BeNumerically(">=", 3))

	seed, ok := recReadForeignEmptySeed(data[recHeaderSize:headerEnd])
	Expect(ok).To(BeTrue())
	world := getWorldFromId(worldId)
	Expect(recWorldIsEmpty(world)).To(BeTrue())
	recApplyForeignSeed(world, &seed)

	frames := 0
	opCount := 0
	for rdr.cursor < registryEnd && rdr.ok {
		op := recDispatchOne(&rdr)
		if op < 0 || op == recOpDestroyWorld {
			break
		}
		opCount++
		if op == recOpStep {
			frames++
		}
		Expect(rdr.diverged).To(BeFalse(), fmt.Sprintf("diverged at frame %d (op 0x%02X)", frames, op))
	}
	Expect(rdr.ok).To(BeTrue())
	Expect(rdr.diverged).To(BeFalse())
	Expect(frames).To(Equal(replayGoldenSteps))
	Expect(opCount).To(BeNumerically(">", 3*replayGoldenSteps))

	var sb strings.Builder
	replayGoldenPrintWorld(&sb, "original", worldId)
	worldId.Destroy()
	return sb.String()
}

var _ = Describe("ReplayGoldenTest", func() {
	It("CRecordingHeader", func() {
		data := replayGoldenData()
		hdr := recDecodeHeader(data)
		Expect(hdr.magic).To(Equal(recMagic))
		Expect(int(hdr.versionMajor)).To(Equal(recVersionMajor))
		Expect(int(hdr.versionMinor)).To(Equal(recVersionMinor))
		Expect(hdr.bigEndian).To(Equal(uint8(0)))
		Expect(hdr.snapshotSize).To(BeNumerically(">", 0))
		Expect(hdr.registryOffset + hdr.registryByteCount).To(Equal(uint64(len(data))))
	})

	It("HeadlessReplayMatchesCorrectedC", func() {
		replayGoldenCompare(replayGoldenHeadless(1), replayGoldenWant("original"))
	})

	It("HeadlessReplayMatchesCorrectedCWithWorkers", func() {
		replayGoldenCompare(replayGoldenHeadless(2), replayGoldenWant("original"))
		replayGoldenCompare(replayGoldenHeadless(4), replayGoldenWant("original"))
	})

	It("PlayerReplayMatchesCorrectedC", func() {
		// deserializeIntoShell rejects nil data and a C snapshot before touching the world.
		Expect(deserializeIntoShell(nil, nil, nil)).To(BeFalse())
		Expect(replayGoldenCSeedRejected()).To(BeTrue())

		data := replayGoldenData()
		want := replayGoldenWant("replay")

		Expect(ValidateReplay(data, 1)).To(BeTrue())

		player := CreatePlayer(data, 1)
		Expect(player).NotTo(BeNil())
		info := player.GetInfo()
		Expect(int(info.FrameCount)).To(Equal(replayGoldenSteps))
		Expect(info.TimeStep).To(Equal(float32(1.0 / 60.0)))
		Expect(int(info.SubStepCount)).To(Equal(4))

		taggedProbe := false
		queryFrames := 0
		for player.StepFrame() {
			if player.GetFrameQueryCount() > 0 {
				queryFrames++
				Expect(player.GetFrameQueryCount()).To(Equal(9))
				q := player.GetFrameQuery(1)
				Expect(q.Type).To(Equal(RecQueryOverlapAABB))
				if q.Id == 42 && q.Name == "probe" {
					taggedProbe = true
				}
				Expect(player.GetFrameQuery(3).Type).To(Equal(RecQueryCastRay))
				Expect(player.GetFrameQuery(3).Id).To(Equal(uint64(player.GetFrame())))
				Expect(player.GetFrameQuery(6).Name).To(Equal("sweep"))
			}
		}
		Expect(player.HasDiverged()).To(BeFalse())
		Expect(player.GetDivergeFrame()).To(Equal(-1))
		Expect(player.GetFrame()).To(Equal(replayGoldenSteps))
		Expect(queryFrames).To(Equal(replayGoldenSteps / 10))
		Expect(taggedProbe).To(BeTrue())

		var sb strings.Builder
		replayGoldenPrintWorld(&sb, "original", player.GetWorldId())
		replayGoldenCompare(sb.String(), want)

		// Backward seek through a keyframe, then a restart through the C seed, each replayed to the end.
		player.SeekFrame(50)
		Expect(player.GetFrame()).To(Equal(50))
		for player.StepFrame() {
		}
		Expect(player.HasDiverged()).To(BeFalse())
		sb.Reset()
		replayGoldenPrintWorld(&sb, "original", player.GetWorldId())
		replayGoldenCompare(sb.String(), want)

		player.Restart()
		Expect(player.GetFrame()).To(Equal(0))
		for player.StepFrame() {
		}
		Expect(player.HasDiverged()).To(BeFalse())
		sb.Reset()
		replayGoldenPrintWorld(&sb, "original", player.GetWorldId())
		replayGoldenCompare(sb.String(), want)

		// Sub-stepping pauses before the Step of each frame that creates bodies (frames 1 and 60) and
		// otherwise advances a frame per call; it ends in the same state. Every recorded query frame
		// draws through the overlay.
		player.Restart()
		preSteps := 0
		drawCalls := 0
		var draw DebugDraw
		draw.DrawSegmentFcn = func(p1, p2 Pos, color HexColor, context any) { drawCalls++ }
		draw.DrawPointFcn = func(p Pos, size float32, color HexColor, context any) { drawCalls++ }
		draw.DrawSphereFcn = func(p Pos, radius float32, color HexColor, alpha float32, context any) { drawCalls++ }
		draw.DrawCapsuleFcn = func(p1, p2 Pos, radius float32, color HexColor, alpha float32, context any) { drawCalls++ }
		draw.DrawBoundsFcn = func(aabb AABB, color HexColor, context any) { drawCalls++ }
		draw.DrawStringFcn = func(p Pos, s string, color HexColor, context any) { drawCalls++ }
		for !player.IsAtEnd() {
			player.SubStepFrame()
			if player.IsAtPreStep() {
				preSteps++
			}
			player.DrawFrameQueries(&draw, -1, 1)
		}
		Expect(preSteps).To(Equal(2))
		Expect(drawCalls).To(BeNumerically(">", 0))
		Expect(player.HasDiverged()).To(BeFalse())
		sb.Reset()
		replayGoldenPrintWorld(&sb, "original", player.GetWorldId())
		replayGoldenCompare(sb.String(), want)

		player.Destroy()
	})
})

// replayGoldenCSeedRejected checks that the C seed snapshot of the golden recording fails
// deserializeIntoShell and leaves a live world untouched (same body count and the same state
// hash).
func replayGoldenCSeedRejected() bool {
	_, csnap, _, _ := recGoldenParts(replayGoldenData())
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)
	defer worldId.Destroy()
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyId := worldId.CreateBody(&bodyDef)
	box := MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := DefaultShapeDef()
	bodyId.CreateHullShape(&shapeDef, &box.Base)
	worldId.Step(1.0/60.0, 4)
	w := getWorldFromId(worldId)
	before := hashWorldState(w)
	count := getIdCount(&w.bodyIdPool)
	if deserializeIntoShell(csnap, w, &recReader{}) {
		return false
	}
	return hashWorldState(w) == before && getIdCount(&w.bodyIdPool) == count
}
