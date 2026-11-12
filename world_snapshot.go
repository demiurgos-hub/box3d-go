// Port of src/world_snapshot.h and src/world_snapshot.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"reflect"
	"sync"
	"unsafe"
)

// ---- world_snapshot.h ----

// serializeWorld is b3SerializeWorld (defined below).
// Serialize the live world into buf, interning shape geometry into rec->registry.
// On success buf holds a self-contained snapshot image. Returns the byte count.

// deserializeIntoShell is b3DeserializeIntoShell (defined below).
// Overwrite a freshly-created (shell) world with the simulation state held in the
// snapshot image [data, size). Geometry references are resolved via the shared
// registry slots in rdr. Returns false on a corrupt or incompatible image.

// ---- world_snapshot.c ----

// PORT: the snapshot image is Go-native (design: "Recording and replay"). Upstream memcpy's its
// internal structs (b3Body, b3Shape, b3Contact, ...) into the image and nulls their pointer fields.
// The Go structs hold slices, strings, `any` and pointers, which cannot travel as raw memory, so
// every struct copy goes through snapWPod / snapRPod: a pointer-free type is copied as its raw
// memory image (unsafe.Sizeof bytes, padding included), a type with pointer fields as the
// concatenation of its pointer-free field ranges in field order (pointer, slice, string,
// interface, map, func and chan fields are skipped; they are the fields upstream nulls and
// restores by hand). A restore zeroes the destination first and then fills only the pointer-free
// ranges, so it never writes a pointer slot behind the garbage collector's back. The layout hash
// is computed from these Go layouts, so a C image (or a Go image from a build with other struct
// layouts) fails the hash check and vice versa. Images are in native byte order like upstream's.

// Snapshot image magic 'BNS3' and version
const (
	snapMagic   uint32 = 0x33534E42 // B3_SNAP_MAGIC
	snapVersion uint32 = 12         // Golem: widened hull topology and SAT feature indices
)

const (
	snapFlagValidation      uint32 = 0x1 // B3_SNAP_FLAG_VALIDATION
	snapFlagDoublePrecision uint32 = 0x2 // B3_SNAP_FLAG_DOUBLE_PRECISION
)

// snapSegment is one pointer-free byte range of a Go type.
type snapSegment struct {
	offset uintptr
	size   uintptr
}

// snapLayout is how one Go type travels in a snapshot image.
type snapLayout struct {
	size        uintptr // unsafe.Sizeof
	podSize     int     // bytes in the image
	pointerFree bool    // copied as the raw memory image
	segments    []snapSegment
}

var snapLayouts sync.Map // reflect.Type -> *snapLayout

// snapTypePointerFree reports whether a type holds no pointers (so its raw memory can be copied).
func snapTypePointerFree(t reflect.Type) bool {
	switch t.Kind() {
	case reflect.Bool, reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64,
		reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64, reflect.Uintptr,
		reflect.Float32, reflect.Float64, reflect.Complex64, reflect.Complex128:
		return true
	case reflect.Array:
		return t.Len() == 0 || snapTypePointerFree(t.Elem())
	case reflect.Struct:
		for i := 0; i < t.NumField(); i++ {
			if snapTypePointerFree(t.Field(i).Type) == false {
				return false
			}
		}
		return true
	default:
		return false
	}
}

// snapAddSegments appends the pointer-free ranges of t at base, merging adjacent ranges.
func snapAddSegments(t reflect.Type, base uintptr, segments *[]snapSegment) {
	if t.Size() == 0 {
		return
	}

	if snapTypePointerFree(t) {
		n := len(*segments)
		if n > 0 && (*segments)[n-1].offset+(*segments)[n-1].size == base {
			(*segments)[n-1].size += t.Size()
		} else {
			*segments = append(*segments, snapSegment{base, t.Size()})
		}
		return
	}

	switch t.Kind() {
	case reflect.Array:
		elem := t.Elem()
		for i := 0; i < t.Len(); i++ {
			snapAddSegments(elem, base+uintptr(i)*elem.Size(), segments)
		}
	case reflect.Struct:
		for i := 0; i < t.NumField(); i++ {
			f := t.Field(i)
			snapAddSegments(f.Type, base+f.Offset, segments)
		}
	default:
		// Pointer-shaped field: not part of the image.
	}
}

// snapLayoutFor returns the cached layout of T.
func snapLayoutFor[T any]() *snapLayout {
	t := reflect.TypeFor[T]()
	if l, ok := snapLayouts.Load(t); ok {
		return l.(*snapLayout)
	}

	l := &snapLayout{size: t.Size(), pointerFree: snapTypePointerFree(t)}
	if l.pointerFree {
		l.podSize = int(t.Size())
		l.segments = []snapSegment{{0, t.Size()}}
	} else {
		snapAddSegments(t, 0, &l.segments)
		for _, s := range l.segments {
			l.podSize += int(s.size)
		}
	}
	actual, _ := snapLayouts.LoadOrStore(t, l)
	return actual.(*snapLayout)
}

// snapLayoutMix folds a Go type layout into the layout hash: its size, image size and ranges.
func snapLayoutMix[T any](h uint32) uint32 {
	l := snapLayoutFor[T]()
	mix := func(x uint32) {
		h ^= x
		h *= 16777619
	}
	mix(uint32(l.size))
	mix(uint32(l.podSize))
	mix(uint32(len(l.segments)))
	for _, s := range l.segments {
		mix(uint32(s.offset))
		mix(uint32(s.size))
	}
	return h
}

// computeLayoutHash is b3ComputeLayoutHash.
// Layout hash over all POD-copied structs + key constants.
// Changing a struct size updates this, catching ABI drift early.
// PORT: computed from the Go layouts (snapLayoutMix: size, image size and pointer-free ranges)
// of the Go counterparts of upstream's list, seeded with a Go marker so it never equals a C hash.
var computeLayoutHash = sync.OnceValue(func() uint32 {
	h := uint32(2166136261)
	mix := func(x uint32) {
		h ^= x
		h *= 16777619
	}
	mix(0x474F) // PORT: "GO" marker
	h = snapLayoutMix[body](h)
	h = snapLayoutMix[bodySim](h)
	h = snapLayoutMix[bodyState](h)
	h = snapLayoutMix[shape](h)
	h = snapLayoutMix[contact](h)
	h = snapLayoutMix[Manifold](h)
	h = snapLayoutMix[joint](h)
	h = snapLayoutMix[jointSim](h)
	h = snapLayoutMix[island](h)
	h = snapLayoutMix[islandSim](h)
	h = snapLayoutMix[contactLink](h)
	h = snapLayoutMix[jointLink](h)
	h = snapLayoutMix[sensor](h)
	h = snapLayoutMix[visitor](h)
	h = snapLayoutMix[solverSet](h)
	h = snapLayoutMix[graphColor](h)
	h = snapLayoutMix[DynamicTree](h)
	h = snapLayoutMix[TreeNode](h)
	h = snapLayoutMix[TreeProxy](h)
	h = snapLayoutMix[HullData](h)
	h = snapLayoutMix[HullVertex](h)
	h = snapLayoutMix[HullHalfEdge](h)
	h = snapLayoutMix[HullFace](h)
	h = snapLayoutMix[Plane](h)
	h = snapLayoutMix[MeshData](h)
	h = snapLayoutMix[MeshNode](h)
	h = snapLayoutMix[MeshTriangle](h)
	h = snapLayoutMix[HeightFieldData](h)
	h = snapLayoutMix[CompoundData](h)
	h = snapLayoutMix[CompoundCapsule](h)
	h = snapLayoutMix[CompoundSphere](h)
	mix(uint32(HullVersion & 0xFFFFFFFF))
	mix(uint32(HullVersion >> 32))
	mix(uint32(MeshVersion & 0xFFFFFFFF))
	mix(uint32(MeshVersion >> 32))
	mix(uint32(HeightFieldVersion & 0xFFFFFFFF))
	mix(uint32(HeightFieldVersion >> 32))
	mix(uint32(CompoundVersion & 0xFFFFFFFF))
	mix(uint32(CompoundVersion >> 32))
	h = snapLayoutMix[setItem](h)
	h = snapLayoutMix[idPool](h)
	h = snapLayoutMix[SurfaceMaterial](h)
	h = snapLayoutMix[contactSpec](h)
	h = snapLayoutMix[triangleCache](h)
	h = snapLayoutMix[nameEntry](h)
	h = snapLayoutMix[sensorHit](h)
	mix(GraphColorCount)
	mix(uint32(BodyTypeCount))
	mix(uint32(unsafe.Sizeof(uintptr(0))))
	return h
})

// snapHeader is b3SnapHeader.
type snapHeader struct {
	magic      uint32
	version    uint32
	layoutHash uint32
	flags      uint32
}

// snapReader is b3SnapReader.
// Bounds-checked read cursor
// PORT: const uint8_t* data is a slice; size is kept.
type snapReader struct {
	data   []byte
	cursor int
	size   int
	ok     bool
}

// snapRCheck is b3SnapRCheck.
func snapRCheck(r *snapReader, need int) {
	if need < 0 || int64(r.cursor)+int64(need) > int64(r.size) {
		r.ok = false
	}
}

// snapRBytes is b3SnapR_Bytes.
// PORT: void* dst + n is a byte slice.
func snapRBytes(r *snapReader, dst []byte) {
	n := len(dst)
	snapRCheck(r, n)
	if !r.ok {
		return
	}
	copy(dst, r.data[r.cursor:r.cursor+n])
	r.cursor += n
}

// snapRI32 is b3SnapR_I32.
func snapRI32(r *snapReader) int {
	var b [4]byte
	snapRBytes(r, b[:])
	return int(int32(binary.NativeEndian.Uint32(b[:])))
}

// snapRU32 is b3SnapR_U32.
func snapRU32(r *snapReader) uint32 {
	var b [4]byte
	snapRBytes(r, b[:])
	return binary.NativeEndian.Uint32(b[:])
}

// snapWI32 is b3SnapW_I32.
func snapWI32(buf *recBuffer, v int) {
	var b [4]byte
	binary.NativeEndian.PutUint32(b[:], uint32(int32(v)))
	recBufAppend(buf, b[:])
}

// snapWU32 is b3SnapW_U32.
func snapWU32(buf *recBuffer, v uint32) {
	var b [4]byte
	binary.NativeEndian.PutUint32(b[:], v)
	recBufAppend(buf, b[:])
}

// snapWBytes is b3SnapW_Bytes.
func snapWBytes(buf *recBuffer, src []byte) {
	recBufAppend(buf, src)
}

// snapWPod writes the image of one value: b3SnapW_Bytes( buf, v, sizeof( *v ) ) on a struct.
// PORT: Go-native image, see the note at the top of the file.
func snapWPod[T any](buf *recBuffer, v *T) {
	l := snapLayoutFor[T]()
	base := unsafe.Pointer(v)
	for _, s := range l.segments {
		recBufAppend(buf, unsafe.Slice((*byte)(unsafe.Add(base, s.offset)), s.size))
	}
}

// snapRPod reads the image of one value: b3SnapR_Bytes( r, dst, sizeof( *dst ) ) on a struct.
// PORT: only the pointer-free ranges are written; the caller resets the pointer fields (upstream
// nulls them after the read), so dst is zeroed first.
func snapRPod[T any](r *snapReader, dst *T) {
	l := snapLayoutFor[T]()
	snapRCheck(r, l.podSize)
	if !r.ok {
		return
	}
	var zero T
	*dst = zero
	base := unsafe.Pointer(dst)
	for _, s := range l.segments {
		snapRBytes(r, unsafe.Slice((*byte)(unsafe.Add(base, s.offset)), s.size))
	}
}

// snapWPodSlice writes the images of a run of values: b3SnapW_Bytes( buf, data, count * sizeof ).
func snapWPodSlice[T any](buf *recBuffer, values []T) {
	if len(values) == 0 {
		return
	}
	l := snapLayoutFor[T]()
	if l.pointerFree {
		recBufAppend(buf, unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(values))), uintptr(len(values))*l.size))
		return
	}
	for i := range values {
		snapWPod(buf, &values[i])
	}
}

// snapRPodSlice reads the images of a run of values: b3SnapR_Bytes( r, data, count * sizeof ).
func snapRPodSlice[T any](r *snapReader, values []T) {
	if len(values) == 0 {
		return
	}
	l := snapLayoutFor[T]()
	if l.pointerFree {
		snapRBytes(r, unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(values))), uintptr(len(values))*l.size))
		return
	}
	for i := range values {
		if !r.ok {
			return
		}
		snapRPod(r, &values[i])
	}
}

// snapPodSize is sizeof( T ) in the image.
func snapPodSize[T any]() int {
	return snapLayoutFor[T]().podSize
}

// snapMemSize is sizeof( T ) in memory.
func snapMemSize[T any]() int {
	return int(snapLayoutFor[T]().size)
}

// snapCheckCount is b3SnapCheckCount.
// Bounds check before allocating from image
func snapCheckCount(r *snapReader, count int, memSize int, minStreamBytes int) bool {
	if count < 0 || memSize < 0 || minStreamBytes < 0 {
		return false
	}
	if memSize > 0 && count > 0x7FFFFFFF/memSize {
		return false
	}
	remaining := int64(r.size) - int64(r.cursor)
	return int64(count)*int64(minStreamBytes) <= remaining
}

// serPodArray is b3SerPodArray.
// POD array: count + raw bytes
func serPodArray[T any](buf *recBuffer, arr []T) {
	snapWI32(buf, len(arr))
	if len(arr) > 0 {
		snapWPodSlice(buf, arr)
	}
}

// desPodArray is b3DesPodArray.
// PORT: the stream size per element is the Go image size (snapPodSize) and the memory size is
// unsafe.Sizeof; for upstream both are sizeof.
func desPodArray[T any](r *snapReader, arr *[]T) {
	cnt := snapRI32(r)
	elemSize := snapPodSize[T]()
	if r.ok && snapCheckCount(r, cnt, snapMemSize[T](), elemSize) == false {
		r.ok = false
	}
	if r.ok && cnt > 0 {
		arrayResize(arr, cnt)
		snapRPodSlice(r, *arr)
	} else if r.ok {
		*arr = (*arr)[:0]
	}
}

// serIdPool is b3SerIdPool.
// Id pool: nextIndex + freeArray
func serIdPool(buf *recBuffer, pool *idPool) {
	snapWI32(buf, int(pool.nextIndex))
	serPodArray(buf, pool.freeArray)
}

// desIdPool is b3DesIdPool.
func desIdPool(r *snapReader, pool *idPool) {
	pool.nextIndex = int32(snapRI32(r))
	desPodArray(r, &pool.freeArray)
}

// serBitSet is b3SerBitSet.
// BitSet: blockCount + raw words
func serBitSet(buf *recBuffer, bs *bitSet) {
	snapWU32(buf, bs.blockCount)
	if bs.blockCount > 0 {
		snapWPodSlice(buf, bs.bits[:bs.blockCount])
	}
}

// desBitSet is b3DesBitSet.
func desBitSet(r *snapReader, bs *bitSet) {
	blockCount := snapRU32(r)
	if r.ok && snapCheckCount(r, int(blockCount), 8, 8) == false {
		r.ok = false
	}
	destroyBitSet(bs)
	if !r.ok {
		return
	}
	blockCapacity := blockCount
	if blockCapacity == 0 {
		blockCapacity = 1
	}
	bs.bits = make([]uint64, blockCapacity)
	bs.blockCapacity = blockCapacity
	bs.blockCount = blockCount
	if blockCount > 0 {
		snapRPodSlice(r, bs.bits[:blockCount])
	}
}

// serHashSet is b3SerHashSet.
// HashSet: capacity + count + raw items (probe order depends on layout)
func serHashSet(buf *recBuffer, hs *hashSet) {
	snapWU32(buf, hs.capacity)
	snapWU32(buf, hs.count)
	if hs.capacity > 0 {
		snapWPodSlice(buf, hs.items[:hs.capacity])
	}
}

// desHashSet is b3DesHashSet.
func desHashSet(r *snapReader, hs *hashSet) {
	cap_ := snapRU32(r)
	cnt := snapRU32(r)
	valid := snapCheckCount(r, int(cap_), snapMemSize[setItem](), snapPodSize[setItem]()) &&
		(cap_&(cap_-1)) == 0 && cnt <= cap_
	if r.ok && valid == false && (cap_ != 0 || cnt != 0) {
		r.ok = false
	}
	destroySet(hs)
	if !r.ok {
		return
	}
	if cap_ > 0 {
		hs.items = make([]setItem, cap_)
		hs.capacity = cap_
		hs.count = cnt
		snapRPodSlice(r, hs.items)
	} else {
		hs.items = nil
		hs.capacity = 0
		hs.count = 0
	}
}

// serTree is b3SerTree.
// DynamicTree: version, scalars, then nodes and parents to nodeEnd and proxies at capacity.
// The empty node and the free pairs are sentinels below the end and travel as they are. Only the
// stale tail above the end, the spare array and the rebuild scratch are skipped.
func serTree(buf *recBuffer, tree *DynamicTree) {
	snapWPod(buf, &tree.Version)
	snapWI32(buf, int(tree.NodeEnd))
	snapWI32(buf, int(tree.PairFreeList))
	snapWI32(buf, int(tree.ProxyCount))
	snapWI32(buf, int(tree.ProxyCapacity))
	snapWI32(buf, int(tree.ProxyFreeList))
	if tree.DfsOrdered {
		snapWI32(buf, 1)
	} else {
		snapWI32(buf, 0)
	}
	if tree.NodeEnd > 0 {
		snapWPodSlice(buf, tree.Nodes[:tree.NodeEnd])
		snapWPodSlice(buf, tree.Parents[:tree.NodeEnd])
	}
	if tree.ProxyCapacity > 0 {
		snapWPodSlice(buf, tree.Proxies[:tree.ProxyCapacity])
	}
}

// desTree is b3DesTree.
func desTree(r *snapReader, tree *DynamicTree) {
	var version uint64
	snapRPod(r, &version)
	nodeEnd := snapRI32(r)
	pairFreeList := snapRI32(r)
	proxyCount := snapRI32(r)
	proxyCapacity := snapRI32(r)
	proxyFreeList := snapRI32(r)
	dfsOrdered := snapRI32(r) != 0

	if r.ok && snapCheckCount(r, nodeEnd, snapMemSize[TreeNode](), snapPodSize[TreeNode]()) == false {
		r.ok = false
	}

	if r.ok && snapCheckCount(r, proxyCapacity, snapMemSize[TreeProxy](), snapPodSize[TreeProxy]()) == false {
		r.ok = false
	}

	if r.ok && (nodeEnd < 2 || (nodeEnd&1) != 0 || proxyCapacity < 1) {
		r.ok = false
	}

	// Free existing allocation including the spare array and any rebuild scratch
	tree.Nodes = nil
	tree.Parents = nil
	tree.Proxies = nil
	tree.SwapNodes = nil
	tree.LeafIndices = nil
	tree.LeafNodes = nil
	tree.LeafBoxes = nil
	tree.LeafCenters = nil
	tree.BinIndices = nil
	tree.NodeEnd = 0
	tree.NodeCapacity = 0
	tree.ProxyCapacity = 0
	tree.RebuildCapacity = 0

	if !r.ok {
		return
	}

	tree.Version = version
	tree.NodeEnd = int32(nodeEnd)
	tree.NodeCapacity = int32(nodeEnd)
	tree.PairFreeList = int32(pairFreeList)
	tree.ProxyCount = int32(proxyCount)
	tree.ProxyCapacity = int32(proxyCapacity)
	tree.ProxyFreeList = int32(proxyFreeList)
	tree.DfsOrdered = dfsOrdered

	tree.Nodes = make([]TreeNode, nodeEnd)
	tree.Parents = make([]int32, nodeEnd)
	snapRPodSlice(r, tree.Nodes)
	snapRPodSlice(r, tree.Parents)

	tree.Proxies = make([]TreeProxy, proxyCapacity)
	snapRPodSlice(r, tree.Proxies)
}

// serSolverSet is b3SerSolverSet.
// Solver set: setIndex + 4 arrays (note: contactIndices is int array, not contactSims)
func serSolverSet(buf *recBuffer, set *solverSet) {
	snapWI32(buf, int(set.setIndex))
	serPodArray(buf, set.bodySims)
	serPodArray(buf, set.bodyStates)
	serPodArray(buf, set.jointSims)
	serPodArray(buf, set.contactIndices)
	serPodArray(buf, set.islandSims)
}

// desSolverSet is b3DesSolverSet.
func desSolverSet(r *snapReader, set *solverSet) {
	set.setIndex = int32(snapRI32(r))
	desPodArray(r, &set.bodySims)
	desPodArray(r, &set.bodyStates)
	desPodArray(r, &set.jointSims)
	desPodArray(r, &set.contactIndices)
	desPodArray(r, &set.islandSims)
}

// serNames is b3SerNames.
func serNames(buf *recBuffer, cache *nameCache) {
	snapWI32(buf, len(cache.entries))
	count := len(cache.entries)
	for i := 0; i < count; i++ {
		entry := &cache.entries[i]
		snapWU32(buf, entry.hash)
		snapWI32(buf, int(entry.length))
		recBufAppend(buf, []byte(entry.name[:entry.length]))
	}
}

// desNames is b3DesNames.
// PORT: the minimum stream size per entry is 8 (hash + length) as upstream; the memory size is
// unsafe.Sizeof( nameEntry ).
func desNames(r *snapReader, cache *nameCache) {
	count := snapRI32(r)

	if r.ok && snapCheckCount(r, count, snapMemSize[nameEntry](), 8) == false {
		r.ok = false
	}

	if r.ok == false {
		return
	}

	arrayReserve(&cache.entries, count)
	for i := 0; i < count; i++ {
		hash := snapRU32(r)
		length := snapRI32(r)
		if r.ok == false || length < 0 || length > r.size-r.cursor {
			r.ok = false
			return
		}
		name := make([]byte, length)
		snapRBytes(r, name)
		loadName(cache, hash, string(name), length)
	}
}

// serGraphColor is b3SerGraphColor.
// Graph color: bodySet (non-overflow only) + jointSims + convexContacts + contacts
func serGraphColor(buf *recBuffer, color *graphColor, isOverflow bool) {
	if !isOverflow {
		serBitSet(buf, &color.bodySet)
	}
	serPodArray(buf, color.jointSims)
	serPodArray(buf, color.convexContacts)
	serPodArray(buf, color.contacts)
	// wideConstraints / manifoldConstraints / contactConstraints are transient, not serialized
}

// desGraphColor is b3DesGraphColor.
func desGraphColor(r *snapReader, color *graphColor, isOverflow bool) {
	if !isOverflow {
		desBitSet(r, &color.bodySet)
	}
	desPodArray(r, &color.jointSims)
	desPodArray(r, &color.convexContacts)
	desPodArray(r, &color.contacts)
	// Transient pointers left at NULL/0 from shell
}

// serWorldConfig is b3SerWorldConfig.
// World simulation scalars (never host/callback/worker state)
func serWorldConfig(buf *recBuffer, world *world) {
	snapWPod(buf, &world.gravity)
	snapWPod(buf, &world.hitEventThreshold)
	snapWPod(buf, &world.restitutionThreshold)
	snapWI32(buf, int(world.restitutionIterations))
	snapWPod(buf, &world.maxLinearSpeed)
	snapWPod(buf, &world.isotropicInertiaTolerance)
	snapWPod(buf, &world.contactSpeed)
	snapWPod(buf, &world.contactHertz)
	snapWPod(buf, &world.contactDampingRatio)
	snapWPod(buf, &world.contactRecycleDistance)
	snapWPod(buf, &world.stepIndex)
	snapWI32(buf, int(world.splitIslandId))
	snapWPod(buf, &world.inv_h)
	snapWPod(buf, &world.inv_dt)
	snapWI32(buf, int(world.endEventArrayIndex))
	snapWI32(buf, int(world.compoundShapeCount))
	snapWPod(buf, &world.maxCapacity)
	flags := uint8(0)
	if world.enableSleep {
		flags |= 0x01
	}
	if world.enableWarmStarting {
		flags |= 0x02
	}
	if world.enableContinuous {
		flags |= 0x04
	}
	if world.enableSpeculative {
		flags |= 0x08
	}
	if world.enableRestitutionPropagation {
		flags |= 0x10
	}
	recBufAppend(buf, []byte{flags})
}

// desWorldConfig is b3DesWorldConfig.
func desWorldConfig(r *snapReader, world *world) {
	snapRPod(r, &world.gravity)
	snapRPod(r, &world.hitEventThreshold)
	snapRPod(r, &world.restitutionThreshold)
	world.restitutionIterations = int32(snapRI32(r))
	snapRPod(r, &world.maxLinearSpeed)
	snapRPod(r, &world.isotropicInertiaTolerance)
	snapRPod(r, &world.contactSpeed)
	snapRPod(r, &world.contactHertz)
	snapRPod(r, &world.contactDampingRatio)
	snapRPod(r, &world.contactRecycleDistance)
	snapRPod(r, &world.stepIndex)
	world.splitIslandId = int32(snapRI32(r))
	snapRPod(r, &world.inv_h)
	snapRPod(r, &world.inv_dt)
	world.endEventArrayIndex = int32(snapRI32(r))
	world.compoundShapeCount = int32(snapRI32(r))
	snapRPod(r, &world.maxCapacity)
	var flags [1]byte
	snapRBytes(r, flags[:])
	world.enableSleep = (flags[0] & 0x01) != 0
	world.enableWarmStarting = (flags[0] & 0x02) != 0
	world.enableContinuous = (flags[0] & 0x04) != 0
	world.enableSpeculative = (flags[0] & 0x08) != 0
	world.enableRestitutionPropagation = (flags[0] & 0x10) != 0
}

// clearShapeGeometry zeroes the geometry union of a shape: upstream's
// memset( &shape.capsule, 0, sizeof( shape.capsule ) ), which covers every union member.
// PORT: the union members are separate Go fields; all of them are reset.
func clearShapeGeometry(s *shape) {
	s.capsule = Capsule{}
	s.sphere = Sphere{}
	s.hull = nil
	s.mesh = Mesh{}
	s.heightField = nil
	s.compound = nil
}

// serShapes is b3SerShapes.
// Shapes carry pointer fields: materials, userData, userShape, and the geometry union.
// Serialize the POD scalars with pointers nulled, then the owned materials array, then geometry.
// A single material lives inline in the struct image.
// Hull/mesh/heightField/compound are interned into the recording registry; sphere/capsule inline.
func serShapes(buf *recBuffer, world *world, rec *Recording) {
	count := len(world.shapes)
	snapWI32(buf, count)

	for i := 0; i < count; i++ {
		shape := world.shapes[i]
		isLive := (int(shape.id) == i)

		// Null out pointer fields before writing the raw struct
		shape.materials = nil
		shape.userData = nil
		shape.userShape = nil
		// Zero the geometry union so free-slot images have deterministic bytes
		if !isLive {
			clearShapeGeometry(&shape)
		}
		snapWPod(buf, &shape)

		if !isLive {
			// Free slot: no materials or geometry
			snapWI32(buf, 0)  // materialCount
			snapWI32(buf, -1) // geometry kind sentinel
			continue
		}

		// Owned material array. Only multi material meshes and compounds have one. A single material
		// already rode along inline in the struct image, so write a zero length for it.
		src := &world.shapes[i]
		if src.materials != nil {
			snapWI32(buf, int(src.materialCount))
			snapWPodSlice(buf, src.materials[:src.materialCount])
		} else {
			snapWI32(buf, 0)
		}

		// Geometry
		switch src.typ {
		case SphereShape:
			snapWI32(buf, int(SphereShape))
			snapWPod(buf, &src.sphere)
		case CapsuleShape:
			snapWI32(buf, int(CapsuleShape))
			snapWPod(buf, &src.capsule)
		case HullShape:
			snapWI32(buf, int(HullShape))
			gid := recInternHull(rec, src.hull)
			snapWU32(buf, gid)
		case MeshShape:
			snapWI32(buf, int(MeshShape))
			gid := recInternMesh(rec, src.mesh.Data)
			snapWU32(buf, gid)
			snapWPod(buf, &src.mesh.Scale)
		case HeightShape:
			snapWI32(buf, int(HeightShape))
			gid := recInternHeightField(rec, src.heightField)
			snapWU32(buf, gid)
		case CompoundShape:
			snapWI32(buf, int(CompoundShape))
			gid := recInternCompound(rec, src.compound)
			snapWU32(buf, gid)
		default:
			// A live shape must have a known geometry type. Fail loudly rather than emit a shape
			// with no geometry that would silently lose its collision on restore.
			assert(false, "false")
			snapWI32(buf, -1)
		}
	}
}

// snapSlotBlob views a registry slot's bytes as a geometry blob (the (const b3HullData*) casts of
// slot->bytes upstream).
// PORT: the geometry headers are read in place, which needs 8-byte alignment. Slot bytes are
// allocated aligned by the player; an unaligned slice is copied into a fresh blob.
func snapSlotBlob(slot *registrySlot) unsafe.Pointer {
	bytes := slot.bytes
	if len(bytes) == 0 {
		return nil
	}
	p := unsafe.Pointer(unsafe.SliceData(bytes))
	if uintptr(p)%8 == 0 {
		return p
	}
	blob := blobAlloc(len(bytes))
	copy(unsafe.Slice((*byte)(blob), len(bytes)), bytes)
	return blob
}

// desShapes is b3DesShapes.
func desShapes(r *snapReader, world *world, rdr *recReader) {
	count := snapRI32(r)
	if r.ok && snapCheckCount(r, count, snapMemSize[shape](), snapPodSize[shape]()) == false {
		r.ok = false
	}
	if !r.ok {
		return
	}

	// Save renderer handles before the array is wiped. A keyframe restore is a deterministic replay
	// state, so a live shape that still occupies the same slot with the same generation is the same
	// shape with the same geometry. Carrying its handle over avoids tearing down and rebuilding every
	// GPU mesh on each seek, which the host (a 3D renderer) would otherwise pay for. Handles that are
	// not reclaimed below belong to shapes that are gone or were replaced, and get released so the host
	// pool does not leak across seeks.
	oldShapeCount := len(world.shapes)
	var savedUserShape []any
	var savedGeneration []uint16
	if oldShapeCount > 0 {
		savedUserShape = make([]any, oldShapeCount)
		savedGeneration = make([]uint16, oldShapeCount)
		for i := 0; i < oldShapeCount; i++ {
			old := &world.shapes[i]
			oldLive := (int(old.id) == i)
			if oldLive {
				savedUserShape[i] = old.userShape
			} else {
				savedUserShape[i] = nil
			}
			savedGeneration[i] = old.generation
		}
	}

	arrayResize(&world.shapes, count)
	clear(world.shapes[:count])

	for i := 0; i < count && r.ok; i++ {
		dst := &world.shapes[i]
		snapRPod(r, dst)
		// Pointer fields were written as NULL; set them cleanly
		dst.materials = nil
		dst.userData = nil
		dst.userShape = nil
		clearShapeGeometry(dst)

		isLive := (int(dst.id) == i)

		// Carry the renderer handle over when the same shape still occupies this slot. Consumed
		// handles are nulled so the teardown sweep only releases the ones that vanished.
		if isLive && i < oldShapeCount && savedUserShape != nil && savedUserShape[i] != nil &&
			savedGeneration[i] == dst.generation {
			dst.userShape = savedUserShape[i]
			savedUserShape[i] = nil
		}

		// Serializer writes: matCount, matData, geoKind, geoData
		matCount := snapRI32(r)

		if !r.ok {
			break
		}

		if !isLive {
			// Free slot: matCount=0, geoKind=-1
			_ = matCount
			snapRI32(r) // consume the geoKind sentinel
			continue
		}

		// Owned material array (written before geoKind in serializer). A zero length means the single
		// material is already inline in the restored struct image, so leave materialCount as restored.
		if matCount > 0 {
			if snapCheckCount(r, matCount, snapMemSize[SurfaceMaterial](), snapPodSize[SurfaceMaterial]()) == false {
				r.ok = false
				break
			}
			dst.materialCount = int32(matCount)
			dst.materials = make([]SurfaceMaterial, matCount)
			snapRPodSlice(r, dst.materials)
		} else {
			dst.materials = nil
		}

		geoKind := snapRI32(r)

		// Geometry
		switch ShapeType(geoKind) {
		case SphereShape:
			snapRPod(r, &dst.sphere)
		case CapsuleShape:
			snapRPod(r, &dst.capsule)
		case HullShape:
			gid := snapRU32(r)
			if !r.ok {
				break
			}
			slot := recGetSlot(rdr, gid, geometryHull)
			if slot == nil {
				r.ok = false
				break
			}
			// Hull is cloned into the world DB; pass raw bytes directly
			dst.hull = addHullToDatabase(world, (*HullData)(snapSlotBlob(slot)))
		case MeshShape:
			gid := snapRU32(r)
			var scale Vec3
			snapRPod(r, &scale)
			if !r.ok {
				break
			}
			slot := recGetSlot(rdr, gid, geometryMesh)
			if slot == nil {
				r.ok = false
				break
			}
			// Mesh is a self-contained blob used by reference; point straight at the pristine bytes.
			dst.mesh.Data = (*MeshData)(snapSlotBlob(slot))
			dst.mesh.Scale = scale
		case HeightShape:
			gid := snapRU32(r)
			if !r.ok {
				break
			}
			slot := recGetSlot(rdr, gid, geometryHeightField)
			if slot == nil {
				r.ok = false
				break
			}
			// Self-contained blob used by reference; point straight at the pristine bytes.
			dst.heightField = (*HeightFieldData)(snapSlotBlob(slot))
		case CompoundShape:
			gid := snapRU32(r)
			if !r.ok {
				break
			}
			slot := recGetSlot(rdr, gid, geometryCompound)
			if slot == nil {
				r.ok = false
				break
			}
			dst.compound = (*CompoundData)(snapSlotBlob(slot))
		default:
			// Unknown geometry kind means a corrupt or unsupported snapshot. Fail the load instead
			// of leaving a shape with no geometry.
			r.ok = false
		}
	}

	// Release handles for shapes that are gone or were replaced this restore, so the host pool and any
	// GPU resources they pinned do not leak across seeks.
	if savedUserShape != nil {
		for i := 0; i < oldShapeCount; i++ {
			if savedUserShape[i] != nil && world.destroyDebugShape != nil {
				world.destroyDebugShape(savedUserShape[i], world.userDebugShapeContext)
			}
		}
	}
}

// serContacts is b3SerContacts.
// Contact serialization. b3Contact is not fully POD:
// - manifolds: heap array of b3Manifold, allocated via b3AllocateManifolds
// - meshContact.triangleCache: heap b3Array, active when b3_simMeshContact flag is set
// Serialize: raw struct (with nulled manifolds/triangleCache), then manifolds, then triangleCache.
func serContacts(buf *recBuffer, world *world) {
	count := len(world.contacts)
	snapWI32(buf, count)

	for i := 0; i < count; i++ {
		c := &world.contacts[i]
		isLive := (int(c.contactId) == i)

		// Write raw struct with pointer fields zeroed
		copy_ := *c
		copy_.manifolds = nil
		if copy_.flags&simMeshContact != 0 {
			copy_.meshContact.triangleCache = nil
		}
		snapWPod(buf, &copy_)

		if !isLive {
			// Free slot: no heap data
			snapWI32(buf, 0) // manifoldCount
			// No triangleCache
			continue
		}

		// Manifolds
		snapWI32(buf, int(c.manifoldCount))
		if c.manifoldCount > 0 && c.manifolds != nil {
			snapWPodSlice(buf, c.manifolds[:c.manifoldCount])
		}

		// Mesh triangleCache
		if c.flags&simMeshContact != 0 {
			snapWI32(buf, len(c.meshContact.triangleCache))
			if len(c.meshContact.triangleCache) > 0 {
				snapWPodSlice(buf, c.meshContact.triangleCache)
			}
		}
	}
}

// desContacts is b3DesContacts.
func desContacts(r *snapReader, world *world) {
	count := snapRI32(r)
	if r.ok && snapCheckCount(r, count, snapMemSize[contact](), snapPodSize[contact]()) == false {
		r.ok = false
	}
	if !r.ok {
		return
	}

	arrayResize(&world.contacts, count)
	clear(world.contacts[:count])

	for i := 0; i < count && r.ok; i++ {
		dst := &world.contacts[i]
		snapRPod(r, dst)
		dst.manifolds = nil
		if dst.flags&simMeshContact != 0 {
			dst.meshContact.triangleCache = nil
		}

		isLive := (int(dst.contactId) == i)

		manifoldCount := snapRI32(r)

		if !r.ok {
			break
		}

		if isLive && manifoldCount > 0 {
			if snapCheckCount(r, manifoldCount, snapMemSize[Manifold](), snapPodSize[Manifold]()) == false {
				r.ok = false
				break
			}
			dst.manifolds = allocateManifolds(world, manifoldCount)
			dst.manifoldCount = int32(manifoldCount)
			snapRPodSlice(r, dst.manifolds[:manifoldCount])
		} else {
			dst.manifolds = nil
			dst.manifoldCount = 0
		}

		// Mesh triangleCache
		if isLive && (dst.flags&simMeshContact) != 0 {
			cacheCount := snapRI32(r)
			if !r.ok {
				break
			}
			if cacheCount > 0 {
				if snapCheckCount(r, cacheCount, snapMemSize[triangleCache](), snapPodSize[triangleCache]()) == false {
					r.ok = false
					break
				}
				arrayResize(&dst.meshContact.triangleCache, cacheCount)
				snapRPodSlice(r, dst.meshContact.triangleCache)
			}
		}
	}
}

// freeLiveSimElements is b3FreeLiveSimElements.
// Free per-object heap that b3DeserializeIntoShell will overwrite,
// so restoring over a populated world doesn't leak.
func freeLiveSimElements(world *world) {
	// Shape heap: materials and hull DB references
	for i := 0; i < len(world.shapes); i++ {
		s := &world.shapes[i]
		if int(s.id) != i {
			continue
		}
		// A single material lives inline (materials == NULL). Multi material meshes and compounds own
		// the array, so free it exactly as b3DestroyShapeAllocations does.
		if s.materials != nil {
			s.materials = nil
			s.materialCount = 0
		}
		// Hull is ref-counted in the world DB; release before overwrite so re-adding is ref-neutral.
		if s.typ == HullShape && s.hull != nil {
			removeHullFromDatabase(world, s.hull)
			s.hull = nil
		}
		// name / userData / userShape are host-owned; do not free
	}

	// Contact heap: manifolds + mesh triangleCache
	for i := 0; i < len(world.contacts); i++ {
		c := &world.contacts[i]
		if int(c.contactId) == i {
			if c.manifolds != nil {
				freeManifolds(world, c.manifolds, int(c.manifoldCount))
				c.manifolds = nil
				c.manifoldCount = 0
			}
			if c.flags&simMeshContact != 0 {
				arrayDestroy(&c.meshContact.triangleCache)
			}
		}
	}

	// Sensor heap: inner arrays
	for i := 0; i < len(world.sensors); i++ {
		sensor := &world.sensors[i]
		arrayDestroy(&sensor.hits)
		arrayDestroy(&sensor.overlaps1)
		arrayDestroy(&sensor.overlaps2)
	}

	// Island heap: inner arrays
	for i := 0; i < len(world.islands); i++ {
		island := &world.islands[i]
		arrayDestroy(&island.bodies)
		arrayDestroy(&island.contacts)
		arrayDestroy(&island.joints)
	}
}

// serializeWorld is b3SerializeWorld.
func serializeWorld(world *world, buf *recBuffer, rec *Recording) int {
	startSize := int(buf.size)

	// Snapshot header
	var hdr snapHeader
	hdr.magic = snapMagic
	hdr.version = snapVersion
	hdr.layoutHash = computeLayoutHash()
	hdr.flags = 0
	if validateEnabled {
		hdr.flags = snapFlagValidation
	}
	// PORT: BOX3D_DOUBLE_PRECISION is not ported; snapFlagDoublePrecision is never set.
	snapWPod(buf, &hdr)

	// World scalars
	serWorldConfig(buf, world)

	// 6 id pools (Box3D has no chainIdPool)
	serIdPool(buf, &world.bodyIdPool)
	serIdPool(buf, &world.shapeIdPool)
	serIdPool(buf, &world.contactIdPool)
	serIdPool(buf, &world.jointIdPool)
	serIdPool(buf, &world.islandIdPool)
	serIdPool(buf, &world.solverSetIdPool)

	// Solver sets
	setCount := len(world.solverSets)
	snapWI32(buf, setCount)
	for i := 0; i < setCount; i++ {
		serSolverSet(buf, &world.solverSets[i])
	}

	// Sparse body array (userData is host wiring, zero it on the copy)
	{
		bodyCount := len(world.bodies)
		snapWI32(buf, bodyCount)
		for i := 0; i < bodyCount; i++ {
			elem := world.bodies[i]
			elem.userData = nil
			snapWPod(buf, &elem)
		}
	}

	// Shape sparse array with geometry interning
	serShapes(buf, world, rec)

	serPodArray(buf, world.fatAABBs)

	// Contact sparse array with manifold and mesh triangleCache
	serContacts(buf, world)

	// Joint sparse array (userData scrubbed)
	{
		jointCount := len(world.joints)
		snapWI32(buf, jointCount)
		for i := 0; i < jointCount; i++ {
			elem := world.joints[i]
			elem.userData = nil
			snapWPod(buf, &elem)
		}
	}

	// Sensors: shapeId + 3 inner arrays each
	{
		sensorCount := len(world.sensors)
		snapWI32(buf, sensorCount)
		for i := 0; i < sensorCount; i++ {
			s := &world.sensors[i]
			snapWI32(buf, int(s.shapeId))
			serPodArray(buf, s.hits)
			serPodArray(buf, s.overlaps1)
			serPodArray(buf, s.overlaps2)
		}
	}

	// Islands: 4 scalars + 3 inner arrays each
	{
		islandCount := len(world.islands)
		snapWI32(buf, islandCount)
		for i := 0; i < islandCount; i++ {
			island := &world.islands[i]
			snapWI32(buf, int(island.setIndex))
			snapWI32(buf, int(island.localIndex))
			snapWI32(buf, int(island.islandId))
			snapWI32(buf, int(island.constraintRemoveCount))
			serPodArray(buf, island.bodies)
			serPodArray(buf, island.contacts)
			serPodArray(buf, island.joints)
		}
	}

	// Broad phase
	bp := &world.broadPhase
	for t := 0; t < int(BodyTypeCount); t++ {
		serTree(buf, &bp.trees[t])
	}
	serHashSet(buf, &bp.pairSet)

	// Constraint graph
	graph := &world.constraintGraph
	for c := 0; c < GraphColorCount; c++ {
		serGraphColor(buf, &graph.colors[c], c == overflowIndex)
	}

	serNames(buf, &world.names)

	return int(buf.size) - startSize
}

// deserializeIntoShell is b3DeserializeIntoShell.
// PORT: const uint8_t* data + size is a slice. printf becomes the box3d log function.
func deserializeIntoShell(data []byte, world *world, rdr *recReader) bool {
	size := len(data)
	if data == nil || size < snapPodSize[snapHeader]() {
		return false
	}

	// Validate header
	hdrReader := snapReader{data, 0, size, true}
	var hdr snapHeader
	snapRPod(&hdrReader, &hdr)
	if hdr.magic != snapMagic || hdr.version != snapVersion {
		log("b3DeserializeIntoShell: bad magic/version")
		return false
	}
	imageDouble := (hdr.flags & snapFlagDoublePrecision) != 0
	buildDouble := false
	if imageDouble != buildDouble {
		log("b3DeserializeIntoShell: precision mismatch")
		return false
	}
	if hdr.layoutHash != computeLayoutHash() {
		log("b3DeserializeIntoShell: layout hash mismatch")
		return false
	}

	readerStorage := snapReader{}
	r := &readerStorage
	r.data = data
	r.cursor = snapPodSize[snapHeader]()
	r.size = size
	r.ok = true

	// Free existing per-object heap before overwriting
	freeLiveSimElements(world)

	// 1. World scalars
	desWorldConfig(r, world)

	// 2. 6 id pools; destroy the pre-created sets' pool state first
	desIdPool(r, &world.bodyIdPool)
	desIdPool(r, &world.shapeIdPool)
	desIdPool(r, &world.contactIdPool)
	desIdPool(r, &world.jointIdPool)
	desIdPool(r, &world.islandIdPool)
	desIdPool(r, &world.solverSetIdPool)

	// 3. Solver sets: destroy inner arrays of existing sets first
	for i := 0; i < len(world.solverSets); i++ {
		set := &world.solverSets[i]
		arrayDestroy(&set.bodySims)
		arrayDestroy(&set.bodyStates)
		arrayDestroy(&set.jointSims)
		arrayDestroy(&set.contactIndices)
		arrayDestroy(&set.islandSims)
	}

	setCount := snapRI32(r)
	if r.ok && snapCheckCount(r, setCount, snapMemSize[solverSet](), 6*4) == false {
		r.ok = false
	}
	if r.ok {
		arrayResize(&world.solverSets, setCount)
		clear(world.solverSets[:setCount])
		for i := 0; i < setCount; i++ {
			desSolverSet(r, &world.solverSets[i])
		}
	}

	if !r.ok {
		return false
	}

	// 4. Body sparse array
	{
		bodyCount := snapRI32(r)
		if r.ok && snapCheckCount(r, bodyCount, snapMemSize[body](), snapPodSize[body]()) == false {
			r.ok = false
		}
		if r.ok {
			arrayResize(&world.bodies, bodyCount)
			for i := 0; i < bodyCount; i++ {
				snapRPod(r, &world.bodies[i])
				world.bodies[i].userData = nil
			}
		}
	}

	if !r.ok {
		return false
	}

	// 5. Shape sparse array
	desShapes(r, world, rdr)

	if !r.ok {
		return false
	}

	desPodArray(r, &world.fatAABBs)

	if !r.ok {
		return false
	}

	// 6. Contact sparse array
	desContacts(r, world)

	if !r.ok {
		return false
	}

	// 7. Joint sparse array
	{
		jointCount := snapRI32(r)
		if r.ok && snapCheckCount(r, jointCount, snapMemSize[joint](), snapPodSize[joint]()) == false {
			r.ok = false
		}
		if r.ok {
			arrayResize(&world.joints, jointCount)
			for i := 0; i < jointCount; i++ {
				snapRPod(r, &world.joints[i])
				world.joints[i].userData = nil
			}
		}
	}

	// 8. Sensors
	{
		arrayDestroy(&world.sensors)
		arrayCreate(&world.sensors)

		sensorCount := snapRI32(r)
		if r.ok && snapCheckCount(r, sensorCount, snapMemSize[sensor](), 4*4) == false {
			r.ok = false
		}
		if r.ok {
			arrayResize(&world.sensors, sensorCount)
			clear(world.sensors[:sensorCount])
		}

		for i := 0; i < sensorCount && r.ok; i++ {
			s := &world.sensors[i]
			s.shapeId = int32(snapRI32(r))
			arrayCreate(&s.hits)
			arrayCreate(&s.overlaps1)
			arrayCreate(&s.overlaps2)
			desPodArray(r, &s.hits)
			desPodArray(r, &s.overlaps1)
			desPodArray(r, &s.overlaps2)
		}
	}

	// 9. Islands
	{
		arrayDestroy(&world.islands)
		arrayCreate(&world.islands)

		islandCount := snapRI32(r)
		if r.ok && snapCheckCount(r, islandCount, snapMemSize[island](), 7*4) == false {
			r.ok = false
		}
		if r.ok {
			arrayResize(&world.islands, islandCount)
			clear(world.islands[:islandCount])
		}

		for i := 0; i < islandCount && r.ok; i++ {
			island := &world.islands[i]
			island.setIndex = int32(snapRI32(r))
			island.localIndex = int32(snapRI32(r))
			island.islandId = int32(snapRI32(r))
			island.constraintRemoveCount = int32(snapRI32(r))
			arrayCreate(&island.bodies)
			arrayCreate(&island.contacts)
			arrayCreate(&island.joints)
			desPodArray(r, &island.bodies)
			desPodArray(r, &island.contacts)
			desPodArray(r, &island.joints)
		}
	}

	// 10. Broad phase
	{
		bp := &world.broadPhase

		for t := 0; t < int(BodyTypeCount); t++ {
			desTree(r, &bp.trees[t])
		}
		desHashSet(r, &bp.pairSet)
		// The gathered moved siblings are per step scratch and stay at the shell's NULL
	}

	// 11. Constraint graph
	{
		graph := &world.constraintGraph
		for c := 0; c < GraphColorCount; c++ {
			desGraphColor(r, &graph.colors[c], c == overflowIndex)
		}
	}

	desNames(r, &world.names)

	return r.ok
}
