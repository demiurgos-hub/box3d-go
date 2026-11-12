#include "box3d/box3d.h"
#include "determinism.h"
#include "stability.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    b3WorldDef def = b3DefaultWorldDef();
    def.workerCount = argc > 1 ? atoi(argv[1]) : 1;
    b3WorldId world = b3CreateWorld(&def);
    FallingRagdollData ragdolls = CreateFallingRagdolls(world);
    for (int i=0; i<1000; ++i) {
        b3World_Step(world, 1.0f/60.0f, 4);
        if (UpdateFallingRagdolls(world, &ragdolls)) break;
    }
    printf("ragdoll %d %08x\n", ragdolls.sleepStep, ragdolls.hash);
    DestroyFallingRagdolls(&ragdolls);
    b3DestroyWorld(world);
    world = b3CreateWorld(&def);
    MeshDropData mesh = CreateMeshDrop(world, b3Pos_zero);
    for (int i=0; i<1000; ++i) {
        b3World_Step(world, 1.0f/60.0f, 4);
        if (UpdateMeshDrop(world, &mesh)) break;
    }
    printf("meshdrop %d %08x\n", mesh.sleepStep, mesh.hash);
    DestroyMeshDrop(&mesh);
    b3DestroyWorld(world);
    return 0;
}
