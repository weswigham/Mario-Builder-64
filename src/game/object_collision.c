#include <PR/ultratypes.h>

#include "sm64.h"
#include "debug.h"
#include "interaction.h"
#include "mario.h"
#include "object_list_processor.h"
#include "spawn_object.h"
#include "engine/math_util.h"
#include "config/config_collision.h"
#include "config/config_world.h"

UNUSED struct Object *debug_print_obj_collision(struct Object *a) {
    struct Object *currCollidedObj;
    s32 i;

    for (i = 0; i < a->numCollidedObjs; i++) {
        print_debug_top_down_objectinfo("ON", 0);
        currCollidedObj = a->collidedObjs[i];
        if (currCollidedObj != gMarioObject) {
            return currCollidedObj;
        }
    }
    return NULL;
}

s32 detect_object_hitbox_overlap(struct Object *a, struct Object *b) {
    f32 dya_bottom = a->oPosY - a->hitboxDownOffset;
    f32 dyb_bottom = b->oPosY - b->hitboxDownOffset;
    f32 dx = a->oPosX - b->oPosX;
    f32 dz = a->oPosZ - b->oPosZ;
    f32 collisionRadius = a->hitboxRadius + b->hitboxRadius;
    f32 distance = sqr(dx) + sqr(dz);

    if (sqr(collisionRadius) > distance) {
        f32 dya_top = a->hitboxHeight + dya_bottom;
        f32 dyb_top = b->hitboxHeight + dyb_bottom;

        if (dya_bottom > dyb_top
            || dya_top < dyb_bottom
            || a->numCollidedObjs >= 4
            || b->numCollidedObjs >= 4) {
            return FALSE;
        }
        a->collidedObjs[a->numCollidedObjs] = b;
        b->collidedObjs[b->numCollidedObjs] = a;
        a->collidedObjInteractTypes |= b->oInteractType;
        b->collidedObjInteractTypes |= a->oInteractType;
        a->numCollidedObjs++;
        b->numCollidedObjs++;
        return TRUE;
    }

    return FALSE;
}

s32 detect_object_hurtbox_overlap(struct Object *a, struct Object *b) {
    f32 dya_bottom = a->oPosY - a->hitboxDownOffset;
    f32 dyb_bottom = b->oPosY - b->hitboxDownOffset;
    f32 dx = a->oPosX - b->oPosX;
    f32 dz = a->oPosZ - b->oPosZ;
    f32 collisionRadius = a->hurtboxRadius + b->hurtboxRadius;
    f32 distance = sqr(dx) + sqr(dz);

    if (a == gMarioObject) {
        b->oInteractionSubtype |= INT_SUBTYPE_DELAY_INVINCIBILITY;
    }

    if (sqr(collisionRadius) > distance) {
        f32 dya_top = a->hitboxHeight  + dya_bottom;
        f32 dyb_top = b->hurtboxHeight + dyb_bottom;

        if (dya_bottom > dyb_top || dya_top < dyb_bottom) {
            return FALSE;
        }
        if (a == gMarioObject) {
            b->oInteractionSubtype &= ~INT_SUBTYPE_DELAY_INVINCIBILITY;
        }
        return TRUE;
    }

    return FALSE;
}

void perform_collision_check(struct Object *a, struct Object *b) {
    if (b->oIntangibleTimer == 0) {
        if (detect_object_hitbox_overlap(a, b) && b->hurtboxRadius != 0.0f) {
            detect_object_hurtbox_overlap(a, b);
        }
    }
}

#ifdef ACCELERATED_COLLISION_LOOKUP
#ifndef ACCELERATED_COLLISION_LOOKUP_INIT
#define ACCELERATED_COLLISION_LOOKUP_INIT 1
// Static surfaces are bucketed into groups on the xz plane so only nearish surfaces are considered for collision
// This does the same, but for dynamic objects, with smaller buckets, no allocations, and in 3 dimensions

/**
 * How many cells in each dimension should we allocate. Higher is better for accuracy and speed, with diminishing returns beyond the tile width
 * of a level. 32 is large enough that a small level has 1 tile per cell, while larger levels are less accurate.
 * It needs to be at least large enough to hold CMM_MAX_OBJS elements, since every element might need to be inserted. The closer to maximum
 * occupancy it gets, the slower it gets to query, up to a worst case linear scan with extra overhead. (Which is just the non-accelerated case with extra overhead.)
 * With 2-byte cells, a 16x16x16 grid is 8k, while a 32x32x32 is 65k. The N64 CPU data cache is, conveniently, 8k, at 8 bytes/line.
 */
#define NUM_OBJECT_CELLS NUM_CELLS
#define OBJECT_CELL_SIZE CELL_SIZE

/**
 * Use this to convert game units to cell coordinates.
 */
#define GET_OBJECT_CELL_COORD(p)   ((CLAMP(((s32)(p) + LEVEL_BOUNDARY_MAX), 1, LEVEL_BOUNDARY_MAX*2-1) / OBJECT_CELL_SIZE) & (NUM_OBJECT_CELLS - 1))

/**
 * This cutoff is currently unsafe (since a maximally occupied grid could spray elements much farther away), but just exists to slice the logic before we
 * have an empty/non-colliding shell check. It's probably good enough for any non-degenerate scenarios, however, since our default grid size is generous and
 * is guaranteed to have low occupancy. With 512 max elements, a max search radius of 16 is the worst-case. The worst case would be something like 512 surface objects stacked
 * into the same tile, which'd then fill in all the space in the grid around that tile, out to about 8 tile-units. Twice that radius (hence, 16, to measure from one side of that
 * worst-case to the other) would be the "safe" maximal search distance.
 * Generally speaking, the physics logic should also forbid more than 8 or so of these from existing within the same tile-unit, so that worst case shouldn't actually be possible,
 * and a much lower maximum search distance is practical.
 */
#define MAX_SEARCH_DIST 3

#define TO_OBJECT_POOL_OFFSET(ptr) (ptr == 0 ? 0xFFFF : (u16)((uintptr_t)ptr - (uintptr_t)&gObjectPool))
#define FROM_OBJECT_POOL_OFFSET(off) (off == 0xFFFF ? 0 : (struct Object*)((uintptr_t)&gObjectPool + (uintptr_t)off))

// This is a 3d grid of pointers. It will mostly be empty. We will use it such that
// an object is positioned precisely at its' location in the world in this grid, or, should that be taken,
// another nearby space (as near as we can get). This is basically a zero-allocation spatial hashmap, which'll be filled
// by the collision-data-resetting functions, and is then queryable by the collision checking functions each frame.
// This *only* is used for OBJ_LIST_SURFACE objects, though you could extend this to handle other lists, if you wanted to.
u16 gObjectCells[NUM_OBJECT_CELLS][NUM_OBJECT_CELLS][NUM_OBJECT_CELLS];

void reset_object_cells() {
    u8 i,j,k;
    for (i=0; i<NUM_OBJECT_CELLS; i++)
        for (j=0; j<NUM_OBJECT_CELLS; j++)
            for (k=0; k<NUM_OBJECT_CELLS; k++)
                gObjectCells[i][j][k] = 0xFFFF;
}

/**
 * Use this to iterate cells near `a` until `func` returns `TRUE` or MAX_SEARCH_DIST is exhausted
 */
s32 iterate_nearby_object_cells(struct Object *a, s32 func(struct Object *, struct Object *, u8 cellX, u8 cellY, u8 cellZ, void* ctx), void* ctx) {
    u8 posX = GET_OBJECT_CELL_COORD(a->oPosX);
    u8 posY = GET_OBJECT_CELL_COORD(a->oPosY);
    u8 posZ = GET_OBJECT_CELL_COORD(a->oPosZ);
    s32 result = FALSE;
    // Exact match
    if ((result = (*func)(a, FROM_OBJECT_POOL_OFFSET(gObjectCells[posX][posY][posZ]), posX, posY, posZ, ctx))) {
        return result;
    }

    // Start with a (3d) ring (shell) like
    //  xxx
    //  x x
    //  xxx
    // and grow the search radius until max search radius is hit.
    // Note, "radius" here actually just means "number of shells around original",
    // not an actual distance metric. (Which you could use, but would be costlier.)
    // Checking a few too many grid spaces doesn't affect too much, since we can fit
    // multiple entries in a cache line, and look them up in a cache-friendly way.
    u8 minX = posX, maxX = posX;
    u8 minY = posY, maxY = posY;
    u8 minZ = posZ, maxZ = posZ;
    u8 radius = 1;
    u8 curX = 0, curY = 0, curZ = 0;
    for (;radius < MAX_SEARCH_DIST; radius++) {
        // increment/decrement max/min coodinates for shell, handling bounds
        minX = minX > 0 ? minX - 1 : minX;
        minY = minY > 0 ? minY - 1 : minY;
        minZ = minZ > 0 ? minZ - 1 : minZ;
        maxX = maxX < (NUM_OBJECT_CELLS - 1) ? maxX + 1 : maxX;
        maxY = maxY < (NUM_OBJECT_CELLS - 1) ? maxY + 1 : maxY;
        maxZ = maxZ < (NUM_OBJECT_CELLS - 1) ? maxZ + 1 : maxZ;

        for (curX = minX; curX<maxX; curX++)
            for (curY = minY; curY<maxY; curY++)
                for (curZ = minZ; curZ<maxZ; curZ++) {
                    // This loop selects all coordinates within the box defined by the given radius and centerpoint.
                    // We need to skip all coordinates from prior shells, so we prefer iterating likely "closer" things first.
                    // We can do that by checking that at least one coordinate is the centerpoint for that axis +/- the radius
                    // Note: this can be shorter if we swap to signed arithmetic, as you can use `abs(curX - posX) == radius` instead.
                    // Double note: overflow is unhandled, but it shouldn't make a difference, assuming the arithmetic is wrapping instead
                    // of saturating, to handle it unless the grid max size is less than `radius`
                    // Author's note: This could also skip the relevant coordinate in the iterator forward to avoid more of these checks, but would
                    // complicate this more, for unknown gains.
                    if (
                        curX == (posX - radius) ||
                        curX == (posX + radius) ||
                        curY == (posY - radius) ||
                        curY == (posY + radius) ||
                        curZ == (posZ - radius) ||
                        curZ == (posZ + radius)) {
                        if ((result = (*func)(a, FROM_OBJECT_POOL_OFFSET(gObjectCells[curX][curY][curZ]), curX, curY, curZ, ctx))) {
                            return result;
                        }
                    }
                }
    }
    return result;
}

s32 maybe_insert_into_object_cell(struct Object *a, struct Object *b, u8 cellX, u8 cellY, u8 cellZ, void* ctx) {
    if (!b) {
        gObjectCells[cellX][cellY][cellZ] = TO_OBJECT_POOL_OFFSET(a);
        return TRUE;
    }
    return FALSE;
}

s32 check_object_cell_collision(struct Object *a, struct Object *b, u8 cellX, u8 cellY, u8 cellZ, void* ctx) {
    if (!b) {
        return FALSE;
    }
    if (a == b) {
        return FALSE;
    }
    perform_collision_check(a, b);

    if (a->numCollidedObjs >= 4) {
        return TRUE; // Max collision count recorded, no need to search farther
    }
    // TODO: Stopping when we hit a shell of cells with no colliding elements is also an optimization, since we assume all colliders
    // are closeish to the same size. (If they are not, we could pretend large colliders *are* by inserting them
    // into multiple applicable cells.)
    return FALSE;
}

void insert_into_object_cell(struct Object *a) {
    iterate_nearby_object_cells(a, &maybe_insert_into_object_cell, NULL);
}

void check_collision_for_each_nearby(struct Object *a) {
    iterate_nearby_object_cells(a, &check_object_cell_collision, NULL);
}

#endif // ACCELERATED_COLLISION_LOOKUP_INIT
#endif // ACCELERATED_COLLISION_LOOKUP

void clear_object_collision(struct Object *a) {
    struct Object *nextObj = (struct Object *) a->header.next;

    while (nextObj != a) {
        nextObj->numCollidedObjs = 0;
        nextObj->collidedObjInteractTypes = 0;
        if (nextObj->oIntangibleTimer > 0) {
            nextObj->oIntangibleTimer--;
        }
        
#ifdef ACCELERATED_COLLISION_LOOKUP
        if ((struct ObjectNode *)a == &gObjectLists[OBJ_LIST_SURFACE]) {
            insert_into_object_cell(nextObj);
        }
#endif // ACCELERATED_COLLISION_LOOKUP
        nextObj = (struct Object *) nextObj->header.next;
    }
}

void check_collision_in_list(struct Object *a, struct Object *b, struct Object *c) {
    if (a->oIntangibleTimer == 0) {
#ifdef ACCELERATED_COLLISION_LOOKUP
        if ((struct ObjectNode *)c == &gObjectLists[OBJ_LIST_SURFACE]) {
            check_collision_for_each_nearby(a);
            return;
        }
#endif // ACCELERATED_COLLISION_LOOKUP
        while (b != c) {
            perform_collision_check(a, b);
            b = (struct Object *) b->header.next;
        }
    }
}

void check_player_object_collision(void) {
    struct Object *playerObj = (struct Object *) &gObjectLists[OBJ_LIST_PLAYER];
    struct Object   *nextObj = (struct Object *) playerObj->header.next;

    while (nextObj != playerObj) {
        check_collision_in_list(nextObj, (struct Object *) nextObj->header.next, playerObj);
        check_collision_in_list(nextObj,
                      (struct Object *)  gObjectLists[OBJ_LIST_POLELIKE].next,
                      (struct Object *) &gObjectLists[OBJ_LIST_POLELIKE]);
        check_collision_in_list(nextObj,
                      (struct Object *)  gObjectLists[OBJ_LIST_LEVEL].next,
                      (struct Object *) &gObjectLists[OBJ_LIST_LEVEL]);
        check_collision_in_list(nextObj,
                      (struct Object *)  gObjectLists[OBJ_LIST_GENACTOR].next,
                      (struct Object *) &gObjectLists[OBJ_LIST_GENACTOR]);
        check_collision_in_list(nextObj,
                      (struct Object *)  gObjectLists[OBJ_LIST_PUSHABLE].next,
                      (struct Object *) &gObjectLists[OBJ_LIST_PUSHABLE]);
        check_collision_in_list(nextObj,
                      (struct Object *)  gObjectLists[OBJ_LIST_SURFACE].next,
                      (struct Object *) &gObjectLists[OBJ_LIST_SURFACE]);
        check_collision_in_list(nextObj,
                      (struct Object *)  gObjectLists[OBJ_LIST_DESTRUCTIVE].next,
                      (struct Object *) &gObjectLists[OBJ_LIST_DESTRUCTIVE]);
        nextObj = (struct Object *) nextObj->header.next;
    }
}

void check_pushable_object_collision(void) {
    struct Object *pushableObj = (struct Object *) &gObjectLists[OBJ_LIST_PUSHABLE];
    struct Object *nextObj = (struct Object *) pushableObj->header.next;

    while (nextObj != pushableObj) {
        check_collision_in_list(nextObj, (struct Object *) nextObj->header.next, pushableObj);
        nextObj = (struct Object *) nextObj->header.next;
    }
}

void check_destructive_object_collision(void) {
    struct Object *destructiveObj = (struct Object *) &gObjectLists[OBJ_LIST_DESTRUCTIVE];
    struct Object *nextObj = (struct Object *) destructiveObj->header.next;

    while (nextObj != destructiveObj) {
        if (nextObj->oDistanceToMario < 6000.0f) {
            check_collision_in_list(nextObj, (struct Object *) nextObj->header.next, destructiveObj);
            check_collision_in_list(nextObj, (struct Object *) gObjectLists[OBJ_LIST_GENACTOR].next,
                          (struct Object *) &gObjectLists[OBJ_LIST_GENACTOR]);
            check_collision_in_list(nextObj, (struct Object *) gObjectLists[OBJ_LIST_PUSHABLE].next,
                          (struct Object *) &gObjectLists[OBJ_LIST_PUSHABLE]);
            check_collision_in_list(nextObj, (struct Object *) gObjectLists[OBJ_LIST_SURFACE].next,
                          (struct Object *) &gObjectLists[OBJ_LIST_SURFACE]);
        }
        nextObj = (struct Object *) nextObj->header.next;
    }
}

void detect_object_collisions(void) {
#ifdef ACCELERATED_COLLISION_LOOKUP
    reset_object_cells();
#endif // ACCELERATED_COLLISION_LOOKUP
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_POLELIKE]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_PLAYER]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_PUSHABLE]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_GENACTOR]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_LEVEL]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_SURFACE]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_DESTRUCTIVE]);
    check_player_object_collision();
    check_destructive_object_collision();
    check_pushable_object_collision();
}
