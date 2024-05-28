#include <PR/ultratypes.h>

#include "sm64.h"
#include "game/debug.h"
#include "game/level_update.h"
#include "game/mario.h"
#include "game/object_list_processor.h"
#include "math_util.h"
#include "surface_collision.h"
#include "surface_load.h"
#include "game/puppyprint.h"

#define static

/**************************************************
 *                      WALLS                     *
 **************************************************/

static s32 check_wall_vw(f32 d00, f32 d01, f32 d11, f32 d20, f32 d21, f32 invDenom) {
    f32 v = ((d11 * d20) - (d01 * d21)) * invDenom;
    if (v < 0.0f || v > 1.0f) {
        return TRUE;
    }

    f32 w = ((d00 * d21) - (d01 * d20)) * invDenom;
    if (w < 0.0f || w > 1.0f || v + w > 1.0f) {
        return TRUE;
    }

    return FALSE;
}

s32 check_wall_edge(Vec3f vert, Vec3f v2, f32 *d00, f32 *d01, f32 *invDenom, f32 *offset, f32 margin_radius) {
    if (FLT_IS_NONZERO(vert[1])) {
        f32 v = (v2[1] / vert[1]);
        if (v < 0.0f || v > 1.0f) {
            return TRUE;
        }

        *d00 = ((vert[0] * v) - v2[0]);
        *d01 = ((vert[2] * v) - v2[2]);
        *invDenom = sqrtf(sqr(*d00) + sqr(*d01));
        *offset = (*invDenom - margin_radius);

        return (*offset > 0.0f);
    }

    return TRUE;
}

void get_surface_normal_oo(f32 normal[4], struct Surface *surf) {
    f32 nx, ny, nz;
    f32 mag;

    register s32 x1, y1, z1;
    register s32 x2, y2, z2;
    register s32 x3, y3, z3;

    x1 = surf->vertex1[0];
    y1 = surf->vertex1[1];
    z1 = surf->vertex1[2];
    x2 = surf->vertex2[0];
    y2 = surf->vertex2[1];
    z2 = surf->vertex2[2];
    x3 = surf->vertex3[0];
    y3 = surf->vertex3[1];
    z3 = surf->vertex3[2];

    // (v2 - v1) x (v3 - v2)
    nx = (y2 - y1) * (z3 - z2) - (z2 - z1) * (y3 - y2);
    ny = (z2 - z1) * (x3 - x2) - (x2 - x1) * (z3 - z2);
    nz = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
    mag = sqrtf(nx * nx + ny * ny + nz * nz);
    mag = (f32)(1.0f / mag);
    nx *= mag;
    ny *= mag;
    nz *= mag;

    normal[0] = nx;
    normal[1] = ny;
    normal[2] = nz;
    normal[3] = -(nx * x1 + ny * y1 + nz * z1);
}

void get_surface_normal(f32 normal[3], struct Surface *surf) {
    f32 nx, ny, nz;
    f32 mag;

    register s32 x1, y1, z1;
    register s32 x2, y2, z2;
    register s32 x3, y3, z3;

    x1 = surf->vertex1[0];
    y1 = surf->vertex1[1];
    z1 = surf->vertex1[2];
    x2 = surf->vertex2[0];
    y2 = surf->vertex2[1];
    z2 = surf->vertex2[2];
    x3 = surf->vertex3[0];
    y3 = surf->vertex3[1];
    z3 = surf->vertex3[2];

    // (v2 - v1) x (v3 - v2)
    nx = (y2 - y1) * (z3 - z2) - (z2 - z1) * (y3 - y2);
    ny = (z2 - z1) * (x3 - x2) - (x2 - x1) * (z3 - z2);
    nz = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
    mag = sqrtf(nx * nx + ny * ny + nz * nz);
    mag = (f32)(1.0f / mag);
    nx *= mag;
    ny *= mag;
    nz *= mag;

    normal[0] = nx;
    normal[1] = ny;
    normal[2] = nz;
}

/**
 * Iterate through the list of walls until all walls are checked and
 * have given their wall push.
 */
static s32 find_wall_collisions_from_list(struct SurfaceNode *surfaceNode, struct WallCollisionData *data) {
    const f32 corner_threshold = -0.9f;
    struct Surface *surf;
    f32 offset;
    f32 radius = data->radius;

    Vec3f pos = { data->x, data->y + data->offsetY, data->z };
    Vec3f v0, v1, v2;
    f32 d00, d01, d11, d20, d21;
    f32 invDenom;
    TerrainData type = SURFACE_DEFAULT;
    s32 numCols = 0;

    f32 margin_radius = radius - 1.0f;

    // Stay in this loop until out of walls.
    while (surfaceNode != NULL) {
        surf        = surfaceNode->surface;
        surfaceNode = surfaceNode->next;
        type        = surf->type;

        // Exclude a large number of walls immediately to optimize.
        if (pos[1] < surf->lowerY || pos[1] > surf->upperY) continue;

        // Determine if checking for the camera or not.
        if (gCollisionFlags & COLLISION_FLAG_CAMERA) {
            if (surf_has_no_cam_collision(surf->type)) continue;
        } else {
            // If an object can pass through a vanish cap wall, pass through.
            if (SURFACE_IS_VANISH_CAP(type) && o != NULL) {
                // If an object can pass through a vanish cap wall, pass through.
                if (o->activeFlags & ACTIVE_FLAG_MOVE_THROUGH_GRATE) continue;
                // If Mario has a vanish cap, pass through the vanish cap wall.
                if (o == gMarioObject && gMarioState->flags & MARIO_VANISH_CAP) continue;
            }
        }

        f32 normal[4];
        get_surface_normal_oo(normal, surf);

        // Dot of normal and pos, + origin offset
        offset = (normal[0] * pos[0]) + (normal[1] * pos[1]) + (normal[2] * pos[2]) + normal[3];

        // Exclude surfaces outside of the radius.
        if (offset < -1 || offset > radius) continue;

        vec3_diff(v0, surf->vertex2, surf->vertex1);
        vec3_diff(v1, surf->vertex3, surf->vertex1);
        vec3_diff(v2, pos,           surf->vertex1);

        // Face
        d00 = vec3_dot(v0, v0);
        d01 = vec3_dot(v0, v1);
        d11 = vec3_dot(v1, v1);
        d20 = vec3_dot(v2, v0);
        d21 = vec3_dot(v2, v1);

        invDenom = (d00 * d11) - (d01 * d01);
        if (FLT_IS_NONZERO(invDenom)) {
            invDenom = 1.0f / invDenom;
        }

        if (check_wall_vw(d00, d01, d11, d20, d21, invDenom)) {
            if (offset < 0) {
                continue;
            }

            // Edge 1-2
            if (check_wall_edge(v0, v2, &d00, &d01, &invDenom, &offset, margin_radius)) {
                // Edge 1-3
                if (check_wall_edge(v1, v2, &d00, &d01, &invDenom, &offset, margin_radius)) {
                    vec3_diff(v1, surf->vertex3, surf->vertex2);
                    vec3_diff(v2, pos, surf->vertex2);
                    // Edge 2-3
                    if (check_wall_edge(v1, v2, &d00, &d01, &invDenom, &offset, margin_radius)) {
                        continue;
                    }
                }
            }

            // Check collision
            if (FLT_IS_NONZERO(invDenom)) {
                invDenom = (offset / invDenom);
            }

            // Update pos
            pos[0] += (d00 *= invDenom);
            pos[2] += (d01 *= invDenom);
            margin_radius += 0.01f;

            if ((d00 * normal[0]) + (d01 * normal[2]) < (corner_threshold * offset)) {
                continue;
            }
        } else {
            // Update pos
            pos[0] += normal[0] * (radius - offset);
            pos[2] += normal[2] * (radius - offset);
        }

        // Has collision
        if (data->numWalls < MAX_REFERENCED_WALLS) {
            data->walls[data->numWalls++] = surf;
        }
        numCols++;

        if (gCollisionFlags & COLLISION_FLAG_RETURN_FIRST) {
            break;
        }
    }

    data->x = pos[0];
    data->z = pos[2];
    return numCols;
}

/**
 * Formats the position and wall search for find_wall_collisions.
 */
s32 f32_find_wall_collision(f32 *xPtr, f32 *yPtr, f32 *zPtr, f32 offsetY, f32 radius) {
    struct WallCollisionData collision;

    collision.offsetY = offsetY;
    collision.radius = radius;

    collision.x = *xPtr;
    collision.y = *yPtr;
    collision.z = *zPtr;

    collision.numWalls = 0;

    s32 numCollisions = find_wall_collisions(&collision);

    *xPtr = collision.x;
    *yPtr = collision.y;
    *zPtr = collision.z;

    return numCollisions;
}

/**
 * Find wall collisions and receive their push.
 */
s32 find_wall_collisions(struct WallCollisionData *colData) {
    struct SurfaceNode *node;
    s32 numCollisions = 0;
    s32 x = colData->x;
    s32 z = colData->z;
    s32 y = colData->y;
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_wall);
    PUPPYPRINT_GET_SNAPSHOT();

    colData->numWalls = 0;

    if (is_outside_level_bounds(x, z)) {
        profiler_collision_update(first, PROFILER_TIME_COLLISION_WALL);
        return numCollisions;
    }

    // World (level) consists of a 16x16 grid. Find where the collision is on the grid (round toward -inf)
    s32 minCellX = GET_CELL_COORD(x - colData->radius);
    s32 minCellZ = GET_CELL_COORD(z - colData->radius);
    s32 minCellY = GET_CELL_COORD(y - colData->radius);
    s32 maxCellX = GET_CELL_COORD(x + colData->radius);
    s32 maxCellZ = GET_CELL_COORD(z + colData->radius);
    s32 maxCellY = GET_CELL_COORD(y + colData->radius);

    for (s32 cellX = minCellX; cellX <= maxCellX; cellX++) {
        for (s32 cellZ = minCellZ; cellZ <= maxCellZ; cellZ++) {
            for (s32 cellY = minCellY; cellY <= maxCellY; cellY++) {
                if (!(gCollisionFlags & COLLISION_FLAG_EXCLUDE_DYNAMIC)) {
                    // Check for surfaces belonging to objects.
                    node = gDynamicSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_WALLS];
                    numCollisions += find_wall_collisions_from_list(node, colData);
                }

                // Check for surfaces that are a part of level geometry.
                node = gStaticSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_WALLS];
                numCollisions += find_wall_collisions_from_list(node, colData);
            }
        }
    }

    gCollisionFlags &= ~(COLLISION_FLAG_RETURN_FIRST | COLLISION_FLAG_EXCLUDE_DYNAMIC | COLLISION_FLAG_INCLUDE_INTANGIBLE);
#ifdef VANILLA_DEBUG
    // Increment the debug tracker.
    gNumCalls.wall++;
#endif

    profiler_collision_update(first, PROFILER_TIME_COLLISION_WALL);
    return numCollisions;
}

/**
 * Collides with walls and returns the most recent wall.
 */
void resolve_and_return_wall_collisions(Vec3f pos, f32 offset, f32 radius, struct WallCollisionData *collisionData) {
    collisionData->x = pos[0];
    collisionData->y = pos[1];
    collisionData->z = pos[2];
    collisionData->radius = radius;
    collisionData->offsetY = offset;

    find_wall_collisions(collisionData);

    pos[0] = collisionData->x;
    pos[1] = collisionData->y;
    pos[2] = collisionData->z;
}

/**************************************************
 *                     CEILINGS                   *
 **************************************************/

void add_ceil_margin(s32 *x, s32 *z, Vec3s target1, Vec3s target2, f32 margin) {
    register f32 diff_x = target1[0] - *x + target2[0] - *x;
    register f32 diff_z = target1[2] - *z + target2[2] - *z;
    register f32 invDenom = margin / sqrtf(sqr(diff_x) + sqr(diff_z));

    *x += diff_x * invDenom;
    *z += diff_z * invDenom;
}

static s32 check_within_ceil_triangle_bounds(s32 x, s32 z, struct Surface *surf, f32 margin) {
    s32 addMargin = !SURFACE_IS_VANISH_CAP(surf->type) && !FLT_IS_NONZERO(margin);
    Vec3i vx, vz;
    vx[0] = surf->vertex1[0];
    vz[0] = surf->vertex1[2];
    if (addMargin) add_ceil_margin(&vx[0], &vz[0], surf->vertex2, surf->vertex3, margin);

    vx[1] = surf->vertex2[0];
    vz[1] = surf->vertex2[2];
    if (addMargin) add_ceil_margin(&vx[1], &vz[1], surf->vertex3, surf->vertex1, margin);

    // Checking if point is in bounds of the triangle laterally.
    if (((vz[0] - z) * (vx[1] - vx[0]) - (vx[0] - x) * (vz[1] - vz[0])) > 0) return FALSE;

    // Slight optimization by checking these later.
    vx[2] = surf->vertex3[0];
    vz[2] = surf->vertex3[2];
    if (addMargin) add_ceil_margin(&vx[2], &vz[2], surf->vertex1, surf->vertex2, margin);

    if (((vz[1] - z) * (vx[2] - vx[1]) - (vx[1] - x) * (vz[2] - vz[1])) > 0) return FALSE;
    if (((vz[2] - z) * (vx[0] - vx[2]) - (vx[2] - x) * (vz[0] - vz[2])) > 0) return FALSE;

    return TRUE;
}

extern f32 get_floor_height_at_location(s32 x, s32 z, struct Surface *surf);

/**
 * Iterate through the list of ceilings and find the first ceiling over a given point.
 */
static struct Surface *find_ceil_from_list(struct SurfaceNode *surfaceNode, s32 x, s32 y, s32 z, f32 *pheight) {
    register struct Surface *surf, *ceil = NULL;
    register f32 height;
    SurfaceType type = SURFACE_DEFAULT;
    *pheight = CELL_HEIGHT_LIMIT;
    // Stay in this loop until out of ceilings.
    while (surfaceNode != NULL) {
        surf = surfaceNode->surface;
        surfaceNode = surfaceNode->next;
        type = surf->type;

        // Exclude all ceilings below the point
        if (y > surf->upperY) continue;

        // Determine if checking for the camera or not
        if (gCollisionFlags & COLLISION_FLAG_CAMERA) {
            if (surf_has_no_cam_collision(surf->type)) {
                continue;
            }
        }

        // Check that the point is within the triangle bounds
        if (!check_within_ceil_triangle_bounds(x, z, surf, 1.5f)) continue;

        // Find the height of the ceil at the given location
        height = get_floor_height_at_location(x, z, surf);

        // Exclude ceilings above the previous lowest ceiling
        if (height > *pheight) continue;

        // Checks for ceiling interaction
        if (y > height) continue;

        // If an object can pass through a vanish cap wall, pass through.
        if (SURFACE_IS_VANISH_CAP(surf->type)) {
            // If Mario has a vanish cap, pass through the vanish cap wall.
            if (gCurrentObject != NULL && gCurrentObject == gMarioObject
                && (gMarioState->flags & MARIO_VANISH_CAP)) {
                continue;
            }
        }

        // Use the current ceiling
        *pheight = height;
        ceil = surf;

        // Exit the loop if it's not possible for another ceiling to be closer
        // to the original point, or if COLLISION_FLAG_RETURN_FIRST.
        if (height == y || (gCollisionFlags & COLLISION_FLAG_RETURN_FIRST)) break;
    }
    return ceil;
}

/**
 * Find the lowest ceiling above a given position and return the height.
 */
f32 find_ceil(f32 posX, f32 posY, f32 posZ, struct Surface **pceil) {
    f32 height        = CELL_HEIGHT_LIMIT;
    f32 dynamicHeight = CELL_HEIGHT_LIMIT;
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_ceil);
    PUPPYPRINT_GET_SNAPSHOT();
    s32 x = posX;
    s32 y = posY;
    s32 z = posZ;
    *pceil = NULL;

    if (is_outside_level_bounds(x, z)) {
        profiler_collision_update(first, PROFILER_TIME_COLLISION_CEIL);
        return height;
    }

    // Each level is split into cells to limit load, find the appropriate cell.
    s32 cellX = GET_CELL_COORD(x);
    s32 cellZ = GET_CELL_COORD(z);
    s32 cellY = GET_CELL_COORD(y);

    struct SurfaceNode *surfaceList;
    struct Surface *ceil = NULL;
    struct Surface *dynamicCeil = NULL;

    s32 includeDynamic = !(gCollisionFlags & COLLISION_FLAG_EXCLUDE_DYNAMIC);
    s32 cellYCheck = cellY;

    if (includeDynamic) {
        // Check for surfaces belonging to objects.
        surfaceList = gDynamicSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_CEILS];
        dynamicCeil = find_ceil_from_list(surfaceList, x, y, z, &dynamicHeight);
        while (dynamicCeil == NULL && cellYCheck < NUM_CELLS) {
            cellYCheck++;
            surfaceList = gDynamicSurfacePartition[cellZ][cellX][cellYCheck][SPATIAL_PARTITION_CEILS];
            dynamicCeil = find_ceil_from_list(surfaceList, x, y, z, &dynamicHeight);
        }
        cellYCheck = cellY;

        // In the next check, only check for ceilings lower than the previous check.
        height = dynamicHeight;
    }

    // Check for surfaces that are a part of level geometry.
    surfaceList = gStaticSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_CEILS];
    ceil = find_ceil_from_list(surfaceList, x, y, z, &height);
    while (ceil == NULL && cellYCheck < NUM_CELLS) {
        cellYCheck++;
        surfaceList = gStaticSurfacePartition[cellZ][cellX][cellYCheck][SPATIAL_PARTITION_CEILS];
        ceil = find_ceil_from_list(surfaceList, x, y, z, &height);
    }

    // Use the lower ceiling.
    if (includeDynamic && height >= dynamicHeight) {
        ceil   = dynamicCeil;
        height = dynamicHeight;
    }

    // To prevent accidentally leaving the floor tangible, stop checking for it.
    gCollisionFlags &= ~(COLLISION_FLAG_RETURN_FIRST | COLLISION_FLAG_EXCLUDE_DYNAMIC | COLLISION_FLAG_INCLUDE_INTANGIBLE);

    // Return the ceiling.
    *pceil = ceil;
#ifdef VANILLA_DEBUG
    // Increment the debug tracker.
    gNumCalls.ceil++;
#endif

    profiler_collision_update(first, PROFILER_TIME_COLLISION_CEIL);
    return height;
}

/**************************************************
 *                     FLOORS                     *
 **************************************************/

static s32 check_within_floor_triangle_bounds(s32 x, s32 z, struct Surface *surf) {
    Vec3i vx, vz;
    vx[0] = surf->vertex1[0];
    vz[0] = surf->vertex1[2];
    vx[1] = surf->vertex2[0];
    vz[1] = surf->vertex2[2];

    if (((vz[0] - z) * (vx[1] - vx[0]) - (vx[0] - x) * (vz[1] - vz[0])) < 0) return FALSE;

    vx[2] = surf->vertex3[0];
    vz[2] = surf->vertex3[2];

    if (((vz[1] - z) * (vx[2] - vx[1]) - (vx[1] - x) * (vz[2] - vz[1])) < 0) return FALSE;
    if (((vz[2] - z) * (vx[0] - vx[2]) - (vx[2] - x) * (vz[0] - vz[2])) < 0) return FALSE;
    return TRUE;
}

/**
 * Iterate through the list of floors and find the first floor under a given point.
 */
static struct Surface *find_floor_from_list(struct SurfaceNode *surfaceNode, s32 x, s32 y, s32 z, f32 *pheight) {
    register struct Surface *surf, *floor = NULL;
    register SurfaceType type = SURFACE_DEFAULT;
    register f32 height;
    register s32 bufferY = y + FIND_FLOOR_BUFFER;

    // Iterate through the list of floors until there are no more floors.
    while (surfaceNode != NULL) {
        surf = surfaceNode->surface;
        surfaceNode = surfaceNode->next;
        type        = surf->type;

        // To prevent the Merry-Go-Round room from loading when Mario passes above the hole that leads
        // there, SURFACE_INTANGIBLE is used. This prevent the wrong room from loading, but can also allow
        // Mario to pass through.
        if (!(gCollisionFlags & COLLISION_FLAG_INCLUDE_INTANGIBLE) && (type == SURFACE_INTANGIBLE)) {
            continue;
        }

        // Determine if we are checking for the camera or not.
        if (gCollisionFlags & COLLISION_FLAG_CAMERA) {
            if (surf_has_no_cam_collision(surf->type)) {
                continue;
            }
        }

        // Exclude all floors above the point.
        if (bufferY < surf->lowerY) continue;
        // Check that the point is within the triangle bounds.
        if (!check_within_floor_triangle_bounds(x, z, surf)) continue;

        // Get the height of the floor under the current location.
        height = get_floor_height_at_location(x, z, surf);

        // Exclude floors lower than the previous highest floor.
        if (height <= *pheight) continue;

        // Checks for floor interaction with a FIND_FLOOR_BUFFER unit buffer.
        if (bufferY < height) continue;

        // If an object can pass through a vanish cap wall, pass through.
        if (SURFACE_IS_VANISH_CAP(surf->type)) {
            // If Mario has a vanish cap, pass through the vanish cap wall.
            if (gCurrentObject != NULL && gCurrentObject == gMarioObject
                && (gMarioState->flags & MARIO_VANISH_CAP)) {
                continue;
            }
        }

        // Use the current floor
        *pheight = height;
        floor = surf;

        // Exit the loop if it's not possible for another floor to be closer
        // to the original point, or if COLLISION_FLAG_RETURN_FIRST.
        if ((height == bufferY) || (gCollisionFlags & COLLISION_FLAG_RETURN_FIRST)) break;
    }
    return floor;
}

// Generic triangle bounds func
ALWAYS_INLINE static s32 check_within_bounds_y_norm(s32 x, s32 z, struct Surface *surf) {
    f32 normal[3];
    get_surface_normal(normal, surf);
    if (normal[1] >= NORMAL_FLOOR_THRESHOLD) return check_within_floor_triangle_bounds(x, z, surf);
    return check_within_ceil_triangle_bounds(x, z, surf, 0);
}

// Find the height of the floor at a given location
ALWAYS_INLINE f32 get_floor_height_at_location(s32 x, s32 z, struct Surface *surf) {
    f32 normal[4];
    get_surface_normal_oo(normal, surf);
    return -(x * normal[0] + normal[2] * z + normal[3]) / normal[1];
}

/**
 * Find the height of the highest floor below a point.
 */
f32 find_floor_height(f32 x, f32 y, f32 z) {
    struct Surface *floor;
    return find_floor(x, y, z, &floor);
}

/**
 * Find the highest floor under a given position and return the height.
 */
f32 find_floor(f32 xPos, f32 yPos, f32 zPos, struct Surface **pfloor) {
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_floor);
    PUPPYPRINT_GET_SNAPSHOT();

    f32 height        = FLOOR_LOWER_LIMIT;
    f32 dynamicHeight = FLOOR_LOWER_LIMIT;

    //! (Parallel Universes) Because position is casted to an s16, reaching higher
    //  float locations can return floors despite them not existing there.
    //  (Dynamic floors will unload due to the range.)
    s32 x = xPos;
    s32 y = yPos;
    s32 z = zPos;

    *pfloor = NULL;

    if (is_outside_level_bounds(x, z)) {
        profiler_collision_update(first, PROFILER_TIME_COLLISION_FLOOR);
        return height;
    }
    // Each level is split into cells to limit load, find the appropriate cell.
    s32 cellX = GET_CELL_COORD(x);
    s32 cellZ = GET_CELL_COORD(z);
    s32 cellY = GET_CELL_COORD(y);

    struct SurfaceNode *surfaceList;
    struct Surface *floor = NULL;
    struct Surface *dynamicFloor = NULL;

    s32 includeDynamic = !(gCollisionFlags & COLLISION_FLAG_EXCLUDE_DYNAMIC);
    s32 cellYCheck = cellY;
    if (includeDynamic) {
        // Check for surfaces belonging to objects.
        surfaceList = gDynamicSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_FLOORS];
        dynamicFloor = find_floor_from_list(surfaceList, x, y, z, &dynamicHeight);
        while (dynamicFloor == NULL && cellYCheck > 0) {
            cellYCheck--;
            surfaceList = gDynamicSurfacePartition[cellZ][cellX][cellYCheck][SPATIAL_PARTITION_FLOORS];
            dynamicFloor = find_floor_from_list(surfaceList, x, y, z, &dynamicHeight);
        }
        cellYCheck = cellY;

        // In the next check, only check for floors higher than the previous check.
        height = dynamicHeight;
    }

    // Check for surfaces that are a part of level geometry.
    surfaceList = gStaticSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_FLOORS];
    floor = find_floor_from_list(surfaceList, x, y, z, &height);
    while (floor == NULL && cellYCheck > 0) {
        cellYCheck--;
        surfaceList = gStaticSurfacePartition[cellZ][cellX][cellYCheck][SPATIAL_PARTITION_FLOORS];
        floor = find_floor_from_list(surfaceList, x, y, z, &height);
    }

    // Use the higher floor.
    if (includeDynamic && height <= dynamicHeight) {
        floor  = dynamicFloor;
        height = dynamicHeight;
    }

    // To prevent accidentally leaving the floor tangible, stop checking for it.
    gCollisionFlags &= ~(COLLISION_FLAG_RETURN_FIRST | COLLISION_FLAG_EXCLUDE_DYNAMIC | COLLISION_FLAG_INCLUDE_INTANGIBLE);
    // If a floor was missed, increment the debug counter.
    if (floor == NULL) {
        gNumFindFloorMisses++;
    }

    // Return the floor.
    *pfloor = floor;
#ifdef VANILLA_DEBUG
    // Increment the debug tracker.
    gNumCalls.floor++;
#endif

    profiler_collision_update(first, PROFILER_TIME_COLLISION_FLOOR);
    return height;
}

f32 find_room_floor(f32 x, f32 y, f32 z, struct Surface **pfloor) {
    gCollisionFlags |= (COLLISION_FLAG_EXCLUDE_DYNAMIC | COLLISION_FLAG_INCLUDE_INTANGIBLE);

    return find_floor(x, y, z, pfloor);
}

// /**
//  * Get the room index at a given position.
//  */
// s32 get_room_at_pos(f32 x, f32 y, f32 z) {
//     if (gCurrentArea->surfaceRooms != NULL) {
//         struct Surface *floor;

//         find_room_floor(x, y, z, &floor);

//         if (floor != NULL) {
//             return floor->room;
//         }
//     }

//     return -1;
// }

// /**
//  * Find the highest water floor under a given position and return the height.
//  */
// f32 find_water_floor(s32 xPos, s32 yPos, s32 zPos, struct Surface **pfloor) {
//     f32 height = FLOOR_LOWER_LIMIT;

//     s32 x = xPos;
//     s32 y = yPos;
//     s32 z = zPos;

//     if (is_outside_level_bounds(x, z)) return height;

//     // Each level is split into cells to limit load, find the appropriate cell.
//     s32 cellX = GET_CELL_COORD(x);
//     s32 cellZ = GET_CELL_COORD(z);
//     s32 cellY = GET_CELL_COORD(y);

//     // Check for surfaces that are a part of level geometry.
//     struct SurfaceNode *surfaceList = gStaticSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_WATER].next;
//     struct Surface     *floor       = find_water_floor_from_list(surfaceList, x, y, z, &height);
//     while (floor == NULL && cellY > 0) {
//         cellY--;
//         surfaceList = gStaticSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_WATER].next;
//         floor       = find_water_floor_from_list(surfaceList, x, y, z, &height);
//     }
//     if (floor == NULL) {
//         height = FLOOR_LOWER_LIMIT;
//     } else {
//         *pfloor = floor;
//     }
// #ifdef VANILLA_DEBUG
//     // Increment the debug tracker.
//     gNumCalls.floor++;
// #endif
//     return height;
// }

/**************************************************
 *               ENVIRONMENTAL BOXES              *
 **************************************************/

// /**
//  * Finds the height of water at a given location.
//  */
// s32 find_water_level_and_floor(s32 x, s32 y, s32 z, struct Surface **pfloor) {
//     s32 val;
//     s32 loX, hiX, loZ, hiZ;
//     TerrainData *p = gEnvironmentRegions;
//     struct Surface *floor = NULL;
//     PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_water);
//     PUPPYPRINT_GET_SNAPSHOT();
//     s32 waterLevel = find_water_floor(x, y, z, &floor);

//     if (p != NULL && waterLevel == FLOOR_LOWER_LIMIT) {
//         s32 numRegions = *p++;

//         for (s32 i = 0; i < numRegions; i++) {
//             val = *p++;
//             loX = *p++;
//             loZ = *p++;
//             hiX = *p++;
//             hiZ = *p++;

//             // If the location is within a water box and it is a water box.
//             // Water is less than 50 val only, while above is gas and such.
//             if (loX < x && x < hiX && loZ < z && z < hiZ && val < 50) {
//                 // Set the water height. Since this breaks, only return the first height.
//                 waterLevel = *p;
//                 break;
//             }
//             p++;
//         }
//     } else {
//         *pfloor = floor;
//     }

//     profiler_collision_update(first);
//     return waterLevel;
// }

// /**
//  * Finds the height of water at a given location.
//  */
// s32 find_water_level(s32 x, s32 z) { // TODO: Allow y pos
//     s32 val;
//     s32 loX, hiX, loZ, hiZ;
//     TerrainData *p = gEnvironmentRegions;
//     struct Surface *floor = NULL;
//     PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_water);
//     PUPPYPRINT_GET_SNAPSHOT();
//     s32 waterLevel = find_water_floor(x, ((gCollisionFlags & COLLISION_FLAG_CAMERA) ? gLakituState.pos[1] : gMarioState->pos[1]), z, &floor);

//     if ((p != NULL) && (waterLevel == FLOOR_LOWER_LIMIT)) {
//         s32 numRegions = *p++;

//         for (s32 i = 0; i < numRegions; i++) {
//             val = *p++;
//             loX = *p++;
//             loZ = *p++;
//             hiX = *p++;
//             hiZ = *p++;

//             // If the location is within a water box and it is a water box.
//             // Water is less than 50 val only, while above is gas and such.
//             if (loX <= x && x <= hiX && loZ <= z && z <= hiZ && val < 50) {
//                 // Set the water height. Since this breaks, only return the first height.
//                 waterLevel = *p;
//                 break;
//             }
//             p++;
//         }
//     }

//     profiler_collision_update(first);

//     return waterLevel;
// }

// /**
//  * Finds the height of the poison gas (used only in HMC) at a given location.
//  */
// s32 find_poison_gas_level(s32 x, s32 z) {
//     s32 val;
//     s32 loX, hiX, loZ, hiZ;
//     s32 gasLevel = FLOOR_LOWER_LIMIT;
//     TerrainData *p = gEnvironmentRegions;
//     PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_water);
//     PUPPYPRINT_GET_SNAPSHOT();

//     if (p != NULL) {
//         s32 numRegions = *p++;

//         for (s32 i = 0; i < numRegions; i++) {
//             val = *p;

//             if (val >= 50) {
//                 loX = p[1];
//                 loZ = p[2];
//                 hiX = p[3];
//                 hiZ = p[4];

//                 // If the location is within a gas's box and it is a gas box.
//                 // Gas has a value of 50, 60, etc.
//                 if (loX < x && x < hiX && loZ < z && z < hiZ && val % 10 == 0) {
//                     // Set the gas height. Since this breaks, only return the first height.
//                     gasLevel = p[5];
//                     break;
//                 }
//             }

//             p += 6;
//         }
//     }
    
//     profiler_collision_update(first);
//     return gasLevel;
// }

/**************************************************
 *                      DEBUG                     *
 **************************************************/

#ifdef VANILLA_DEBUG
/**
 * Finds the length of a surface list for debug purposes.
 */
static s32 surface_list_length(struct SurfaceNode *list) {
    s32 count = 0;
    while (list != NULL) {
        list = list->next;
        count++;
    }
    return count;
}

/**
 * Print the area,number of walls, how many times they were called,
 * and some allocation information.
 */
void debug_surface_list_info(f32 xPos, f32 zPos, f32 yPos) {
    struct SurfaceNode *list;
    s32 numFloors = 0;
    s32 numWalls  = 0;
    s32 numCeils  = 0;

    s32 cellX = GET_CELL_COORD(xPos);
    s32 cellZ = GET_CELL_COORD(zPos);
    s32 cellZ = GET_CELL_COORD(yPos);

    list = gStaticSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_FLOORS];
    numFloors += surface_list_length(list);

    list = gDynamicSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_FLOORS];
    numFloors += surface_list_length(list);

    list = gStaticSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_WALLS];
    numWalls += surface_list_length(list);

    list = gDynamicSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_WALLS];
    numWalls += surface_list_length(list);

    list = gStaticSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_CEILS];
    numCeils += surface_list_length(list);

    list = gDynamicSurfacePartition[cellZ][cellX][cellY][SPATIAL_PARTITION_CEILS];
    numCeils += surface_list_length(list);

    print_debug_top_down_mapinfo("area   %x", cellY * sqr(NUM_CELLS) + cellZ * NUM_CELLS + cellX);

    // Names represent ground, walls, and roofs as found in SMS.
    print_debug_top_down_mapinfo("dg %d", numFloors);
    print_debug_top_down_mapinfo("dw %d", numWalls);
    print_debug_top_down_mapinfo("dr %d", numCeils);

    set_text_array_x_y(80, -3);

    print_debug_top_down_mapinfo("%d", gNumCalls.floor);
    print_debug_top_down_mapinfo("%d", gNumCalls.wall);
    print_debug_top_down_mapinfo("%d", gNumCalls.ceil);

    set_text_array_x_y(-80, 0);

    // listal- List Allocated?, statbg- Static Background?, movebg- Moving Background?
    print_debug_top_down_mapinfo("listal %d", gSurfaceNodesAllocated);
    print_debug_top_down_mapinfo("statbg %d", gNumStaticSurfaces);
    print_debug_top_down_mapinfo("movebg %d", (gSurfacesAllocated - gNumStaticSurfaces));

    gNumCalls.floor = 0;
    gNumCalls.ceil = 0;
    gNumCalls.wall = 0;
}
#endif
