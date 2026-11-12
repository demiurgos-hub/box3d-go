package box3d

func meshContactPointCapacity(typ ShapeType, triangleCount int) int {
	if triangleCount == 0 {
		return 0
	}
	pointsPerTriangle := maxPointsPerTriangle
	switch typ {
	case SphereShape:
		pointsPerTriangle = 1
	case CapsuleShape:
		pointsPerTriangle = 2
	default:
		return pointsPerTriangle * triangleCount
	}
	// The unchanged survivor loop enters only with at least four free slots.
	// Even when every earlier triangle emits its maximum, the last may enter.
	return pointsPerTriangle*triangleCount + 4 - pointsPerTriangle
}

type meshTriangleInfo struct {
	i1, i2, i3, flags int32
}

// Vertices are already transformed in the survivor buffer. Preserve the native
// mirrored winding/edge metadata without reconstructing those vertices again.
func meshTriangleMetadata(tri MeshTriangle, flags uint8, flipped bool) meshTriangleInfo {
	result := meshTriangleInfo{i1: tri.Index1, i2: tri.Index2, i3: tri.Index3, flags: int32(flags)}
	if flipped {
		result.i2, result.i3 = tri.Index3, tri.Index2
		result.flags = 0
		if MeshEdgeFlags(flags)&InverseConcaveEdge1 != 0 {
			result.flags |= int32(ConcaveEdge1)
		}
		if MeshEdgeFlags(flags)&InverseConcaveEdge2 != 0 {
			result.flags |= int32(ConcaveEdge2)
		}
		if MeshEdgeFlags(flags)&InverseConcaveEdge3 != 0 {
			result.flags |= int32(ConcaveEdge3)
		}
	}
	return result
}
