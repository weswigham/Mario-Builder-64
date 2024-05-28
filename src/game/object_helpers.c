#include <PR/ultratypes.h>

#include "sm64.h"
#include "area.h"
#include "behavior_actions.h"
#include "behavior_data.h"
#include "camera.h"
#include "debug.h"
#include "dialog_ids.h"
#include "engine/behavior_script.h"
#include "engine/geo_layout.h"
#include "engine/math_util.h"
#include "engine/surface_collision.h"
#include "game_init.h"
#include "helper_macros.h"
#include "ingame_menu.h"
#include "interaction.h"
#include "level_table.h"
#include "level_update.h"
#include "mario.h"
#include "mario_actions_cutscene.h"
#include "memory.h"
#include "obj_behaviors.h"
#include "object_helpers.h"
#include "object_list_processor.h"
#include "rendering_graph_node.h"
#include "spawn_object.h"
#include "spawn_sound.h"
#include "puppylights.h"
#include "puppycamold.h"
#include "game/rovent.h"
#include "save_file.h"
#include "game/object_collision.h"
#include "audio/external.h"

static s32 clear_move_flag(u32 *bitSet, s32 flag);

void cur_obj_shake(struct Object *obj) {
    if (obj->oTimer % 2 == 0) {
        obj->oPosY += 5.0f;
    } else {
        obj->oPosY -= 5.0f;
    }
}

Gfx *geo_update_projectile_pos_from_parent(s32 callContext, UNUSED struct GraphNode *node, Mat4 mtx) {
    if (callContext == GEO_CONTEXT_RENDER) {
        struct Object *projObj = (struct Object *) gCurGraphNodeObject; // TODO: change global type to Object pointer
        if (projObj->prevObj) {
            obj_update_pos_from_parent_transformation(mtx, projObj->prevObj);
            obj_set_gfx_pos_from_pos(projObj->prevObj);
        }
    }
    return NULL;
}

Gfx *geo_update_layer_transparency(s32 callContext, struct GraphNode *node, UNUSED void *context) {
    Gfx *dlStart = NULL;

    if (callContext == GEO_CONTEXT_RENDER) {
        struct Object *objectGraphNode = (struct Object *) gCurGraphNodeObject; // TODO: change this to object pointer?
        struct GraphNodeGenerated *currentGraphNode = (struct GraphNodeGenerated *) node;
        s32 parameter = currentGraphNode->parameter;

        if (gCurGraphNodeHeldObject != NULL) {
            objectGraphNode = gCurGraphNodeHeldObject->objNode;
        }

        s32 objectOpacity = objectGraphNode->oOpacity;
        dlStart = alloc_display_list(sizeof(Gfx) * 3);

        Gfx *dlHead = dlStart;

        if (objectOpacity == 0xFF) {
            if (parameter == GEO_TRANSPARENCY_MODE_DECAL) {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_TRANSPARENT_DECAL);
            } else {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_OPAQUE);
            }

            objectGraphNode->oAnimState = TRANSPARENCY_ANIM_STATE_OPAQUE;
        } else {
            if (parameter == GEO_TRANSPARENCY_MODE_DECAL) {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_TRANSPARENT_DECAL);
            } else if (parameter == GEO_TRANSPARENCY_MODE_INTER) {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_TRANSPARENT_INTER);
            } else {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_TRANSPARENT);
            }

            objectGraphNode->oAnimState = TRANSPARENCY_ANIM_STATE_TRANSPARENT;

            if (parameter != GEO_TRANSPARENCY_MODE_NO_DITHER
                && (objectGraphNode->activeFlags & ACTIVE_FLAG_DITHERED_ALPHA)) {
                gDPSetAlphaCompare(dlHead++, G_AC_DITHER);
            }
        }
        gDPSetEnvColor(dlHead++, 255, 255, 255, objectOpacity);
        gSPEndDisplayList(dlHead);
    }

    return dlStart;
}

Gfx *geo_set_color_env(s32 callContext, struct GraphNode *node, UNUSED void *context) {
    Gfx *dlStart, *dlHead;
    struct Object *objectGraphNode;
    struct GraphNodeGenerated *currentGraphNode;
    dlStart = NULL;
    
    if (callContext == GEO_CONTEXT_RENDER) {
        objectGraphNode = (struct Object *) gCurGraphNodeObject; // TODO: change this to object pointer?
        currentGraphNode = (struct GraphNodeGenerated *) node;

        if (gCurGraphNodeHeldObject) {
            objectGraphNode = gCurGraphNodeHeldObject->objNode;
        }
        dlStart = alloc_display_list(sizeof(Gfx) * 3);
        dlHead = dlStart;
        currentGraphNode->fnNode.node.flags = (currentGraphNode->parameter << 8) | (currentGraphNode->fnNode.node.flags & 0xFF);

        objectGraphNode->oDmgFade /= 1.2;
        gDPSetEnvColor(dlHead++, objectGraphNode->oDmgFade, 0, 0, 255);
        gSPEndDisplayList(dlHead);
    }
    return dlStart;
}

Gfx *geo_switch_anim_state(s32 callContext, struct GraphNode *node, UNUSED void *context) {
    if (callContext == GEO_CONTEXT_RENDER) {
        struct Object *obj = gCurGraphNodeObjectNode;

        // move to a local var because GraphNodes are passed in all geo functions.
        // cast the pointer.
        struct GraphNodeSwitchCase *switchCase = (struct GraphNodeSwitchCase *) node;

        if (gCurGraphNodeHeldObject != NULL) {
            obj = gCurGraphNodeHeldObject->objNode;
        }

        // if the case is greater than the number of cases, set to 0 to avoid overflowing
        // the switch.
        if (obj->oAnimState >= switchCase->numCases) {
            obj->oAnimState = 0;
        }

        // assign the case number for execution.
        switchCase->selectedCase = obj->oAnimState;
    }

    return NULL;
}

Gfx *geo_switch_area(s32 callContext, struct GraphNode *node, UNUSED void *context) {
    struct GraphNodeSwitchCase *switchCase = (struct GraphNodeSwitchCase *) node;
    RoomData room;

    if (callContext == GEO_CONTEXT_RENDER && gMarioObject != NULL) {
        room = get_room_at_pos(
            gMarioObject->oPosX,
            gMarioObject->oPosY,
            gMarioObject->oPosZ
        );

        print_debug_top_down_objectinfo("areainfo %d", room);

        if (room > 0) {
            gMarioCurrentRoom = room;
            switchCase->selectedCase = (room - 1);
        }
    } else {
        switchCase->selectedCase = 0;
    }

    return NULL;
}

void obj_update_pos_from_parent_transformation(Mat4 a0, struct Object *a1) {
    f32 spC = a1->oParentRelativePosX;
    f32 sp8 = a1->oParentRelativePosY;
    f32 sp4 = a1->oParentRelativePosZ;

    a1->oPosX = spC * a0[0][0] + sp8 * a0[1][0] + sp4 * a0[2][0] + a0[3][0];
    a1->oPosY = spC * a0[0][1] + sp8 * a0[1][1] + sp4 * a0[2][1] + a0[3][1];
    a1->oPosZ = spC * a0[0][2] + sp8 * a0[1][2] + sp4 * a0[2][2] + a0[3][2];
}


void create_transformation_from_matrices(Mat4 a0, Mat4 a1, Mat4 a2) {
    f32 spC, sp8, sp4;

    spC = a2[3][0] * a2[0][0] + a2[3][1] * a2[0][1] + a2[3][2] * a2[0][2];
    sp8 = a2[3][0] * a2[1][0] + a2[3][1] * a2[1][1] + a2[3][2] * a2[1][2];
    sp4 = a2[3][0] * a2[2][0] + a2[3][1] * a2[2][1] + a2[3][2] * a2[2][2];

    a0[0][0] = a1[0][0] * a2[0][0] + a1[0][1] * a2[0][1] + a1[0][2] * a2[0][2];
    a0[0][1] = a1[0][0] * a2[1][0] + a1[0][1] * a2[1][1] + a1[0][2] * a2[1][2];
    a0[0][2] = a1[0][0] * a2[2][0] + a1[0][1] * a2[2][1] + a1[0][2] * a2[2][2];

    a0[1][0] = a1[1][0] * a2[0][0] + a1[1][1] * a2[0][1] + a1[1][2] * a2[0][2];
    a0[1][1] = a1[1][0] * a2[1][0] + a1[1][1] * a2[1][1] + a1[1][2] * a2[1][2];
    a0[1][2] = a1[1][0] * a2[2][0] + a1[1][1] * a2[2][1] + a1[1][2] * a2[2][2];

    a0[2][0] = a1[2][0] * a2[0][0] + a1[2][1] * a2[0][1] + a1[2][2] * a2[0][2];
    a0[2][1] = a1[2][0] * a2[1][0] + a1[2][1] * a2[1][1] + a1[2][2] * a2[1][2];
    a0[2][2] = a1[2][0] * a2[2][0] + a1[2][1] * a2[2][1] + a1[2][2] * a2[2][2];

    a0[3][0] = a1[3][0] * a2[0][0] + a1[3][1] * a2[0][1] + a1[3][2] * a2[0][2] - spC;
    a0[3][1] = a1[3][0] * a2[1][0] + a1[3][1] * a2[1][1] + a1[3][2] * a2[1][2] - sp8;
    a0[3][2] = a1[3][0] * a2[2][0] + a1[3][1] * a2[2][1] + a1[3][2] * a2[2][2] - sp4;

    a0[0][3] = 0;
    a0[1][3] = 0;
    a0[2][3] = 0;
    a0[3][3] = 1.0f;
}

void obj_set_held_state(struct Object *obj, const BehaviorScript *heldBehavior) {
    obj->parentObj = o;

    if (obj->oFlags & OBJ_FLAG_HOLDABLE) {
        if (heldBehavior == bhvCarrySomethingHeld   ) {
            obj->oHeldState = HELD_HELD;
        }

        if (heldBehavior == bhvCarrySomethingThrown ) {
            obj->oHeldState = HELD_THROWN;
        }

        if (heldBehavior == bhvCarrySomethingDropped) {
            obj->oHeldState = HELD_DROPPED;
        }
    } else {
        obj->curBhvCommand = segmented_to_virtual(heldBehavior);
        obj->bhvStackIndex = 0;
    }
}

f32 lateral_dist_between_objects(struct Object *obj1, struct Object *obj2) {
    register f32 dx = obj1->oPosX - obj2->oPosX;
    register f32 dz = obj1->oPosZ - obj2->oPosZ;

    return sqrtf(sqr(dx) + sqr(dz));
}

f32 dist_between_objects(struct Object *obj1, struct Object *obj2) {
    Vec3f d;
    vec3_diff(d, &obj2->oPosVec, &obj1->oPosVec);
    return vec3_mag(d);
}

// Skip sqrtf
f32 dist_between_objects_squared(struct Object *obj1, struct Object *obj2) {
    Vec3f d;
    vec3_diff(d, &obj2->oPosVec, &obj1->oPosVec);
    return vec3_sumsq(d);
}

void cur_obj_forward_vel_approach_upward(f32 target, f32 increment) {
    if (o->oForwardVel >= target) {
        o->oForwardVel = target;
    } else {
        o->oForwardVel += increment;
    }
}

s32 cur_obj_rotate_yaw_toward(s16 target, s16 increment) {
    s16 startYaw;

    startYaw = (s16) o->oMoveAngleYaw;
    o->oMoveAngleYaw = approach_s16_symmetric(o->oMoveAngleYaw, target, increment);

    if ((o->oAngleVelYaw = (s16)((s16) o->oMoveAngleYaw - startYaw)) == 0) {
        return TRUE;
    } else {
        return FALSE;
    }
}

s32 obj_angle_to_object(struct Object *obj1, struct Object *obj2) {
    return atan2s(obj2->oPosZ - obj1->oPosZ, obj2->oPosX - obj1->oPosX);
}

void adjust_faceangleyaw_to_angle(u16 targetangle) {
    if (targetangle > o->oFaceAngleYaw) {
        o->oFaceAngleYaw += abs_angle_diff(o->oFaceAngleYaw,targetangle)/8;
    }
    else
    {
        o->oFaceAngleYaw -= abs_angle_diff(o->oFaceAngleYaw,targetangle)/8;
    }
}

s32 obj_turn_toward_object(struct Object *obj, struct Object *target, s16 angleIndex, s16 turnAmount) {
    Vec3f d;
    s16 targetAngle = 0x0;
    s16 startAngle;

    switch (angleIndex) {
        case O_MOVE_ANGLE_PITCH_INDEX:
        case O_FACE_ANGLE_PITCH_INDEX:
            d[0] = target->oPosX - obj->oPosX;
            d[1] = -target->oPosY + obj->oPosY;
            d[2] = target->oPosZ - obj->oPosZ;

            targetAngle = atan2s(sqrtf(sqr(d[0]) + sqr(d[2])), d[1]);
            break;

        case O_MOVE_ANGLE_YAW_INDEX:
        case O_FACE_ANGLE_YAW_INDEX:
            d[0] = target->oPosX - obj->oPosX;
            d[2] = target->oPosZ - obj->oPosZ;

            targetAngle = atan2s(d[2], d[0]);
            break;
    }

    startAngle = o->rawData.asU32[angleIndex];
    o->rawData.asU32[angleIndex] = approach_s16_symmetric(startAngle, targetAngle, turnAmount);
    return targetAngle;
}

void obj_set_parent_relative_pos(struct Object *obj, s16 relX, s16 relY, s16 relZ) {
    obj->oParentRelativePosX = relX;
    obj->oParentRelativePosY = relY;
    obj->oParentRelativePosZ = relZ;
}

void obj_set_pos(struct Object *obj, s16 x, s16 y, s16 z) {
    obj->oPosX = x;
    obj->oPosY = y;
    obj->oPosZ = z;
}

void obj_set_angle(struct Object *obj, s16 pitch, s16 yaw, s16 roll) {
    obj->oFaceAnglePitch = pitch;
    obj->oFaceAngleYaw = yaw;
    obj->oFaceAngleRoll = roll;

    obj->oMoveAnglePitch = pitch;
    obj->oMoveAngleYaw = yaw;
    obj->oMoveAngleRoll = roll;
}

/*
 * Spawns an object at an absolute location with a specified angle.
 */
struct Object *spawn_object_abs_with_rot(struct Object *parent, s16 uselessArg, ModelID32 model,
                                         const BehaviorScript *behavior,
                                         s16 x, s16 y, s16 z, s16 pitch, s16 yaw, s16 roll) {
    // 'uselessArg' is unused in the function spawn_object_at_origin()
    struct Object *newObj = spawn_object_at_origin(parent, uselessArg, model, behavior);
    obj_set_pos(newObj, x, y, z);
    obj_set_angle(newObj, pitch, yaw, roll);

    return newObj;
}

/*
 * Spawns an object relative to the parent with a specified angle.
 */
struct Object *spawn_object_rel_with_rot(struct Object *parent, ModelID32 model, const BehaviorScript *behavior,
                                         s16 xOff, s16 yOff, s16 zOff, s16 pitch, s16 yaw, s16 roll) {
    struct Object *newObj = spawn_object_at_origin(parent, 0, model, behavior);
    newObj->oFlags |= OBJ_FLAG_TRANSFORM_RELATIVE_TO_PARENT;
    obj_set_parent_relative_pos(newObj, xOff, yOff, zOff);
    obj_set_angle(newObj, pitch, yaw, roll);

    return newObj;
}

struct Object *spawn_obj_with_transform_flags(struct Object *parent, ModelID32 model, const BehaviorScript *behavior) {
    struct Object *newObj = spawn_object(parent, model, behavior);
    newObj->oFlags |= OBJ_FLAG_UPDATE_TRANSFORM_FOR_THROW_MATRIX | OBJ_FLAG_SET_THROW_MATRIX_FROM_TRANSFORM;
    return newObj;
}

struct Object *spawn_water_droplet(struct Object *parent, struct WaterDropletParams *params) {
    struct Object *newObj = spawn_object(parent, params->model, params->behavior);

    if (params->flags & WATER_DROPLET_FLAG_RAND_ANGLE) {
        newObj->oMoveAngleYaw = random_u16();
    }

    if (params->flags & WATER_DROPLET_FLAG_RAND_ANGLE_INCR_BACKWARD) {
        newObj->oMoveAngleYaw = (s16)(newObj->oMoveAngleYaw + 0x8000)
                                + (s16) random_f32_around_zero(params->moveAngleRange);
    }

    if (params->flags & WATER_DROPLET_FLAG_RAND_ANGLE_INCR_FORWARD) {
        newObj->oMoveAngleYaw =
            (s16) newObj->oMoveAngleYaw + (s16) random_f32_around_zero(params->moveAngleRange);
    }

    if (params->flags & WATER_DROPLET_FLAG_SET_Y_TO_WATER_LEVEL) {
        newObj->oPosY = cmm_get_water_level(newObj->oPosX, newObj->oPosY, newObj->oPosZ);
    }

    if (params->flags & WATER_DROPLET_FLAG_RAND_OFFSET_XZ) {
        obj_translate_xz_random(newObj, params->moveRange);
    }

    if (params->flags & WATER_DROPLET_FLAG_RAND_OFFSET_XYZ) {
        obj_translate_xyz_random(newObj, params->moveRange);
    }

    newObj->oForwardVel = random_float() * params->randForwardVelScale + params->randForwardVelOffset;
    newObj->oVelY = random_float() * params->randYVelScale + params->randYVelOffset;

    f32 randomScale = random_float() * params->randSizeScale + params->randSizeOffset;
    obj_scale(newObj, randomScale);

    return newObj;
}

struct Object *spawn_object_at_origin(struct Object *parent, UNUSED s32 unusedArg, ModelID32 model,
                                      const BehaviorScript *behavior) {
    const BehaviorScript *behaviorAddr = segmented_to_virtual(behavior);
    struct Object *obj = create_object(behaviorAddr);

    obj->parentObj = parent;
    obj->header.gfx.areaIndex = parent->header.gfx.areaIndex;
    obj->header.gfx.activeAreaIndex = parent->header.gfx.areaIndex;

    geo_obj_init((struct GraphNodeObject *) &obj->header.gfx, gLoadedGraphNodes[model], gVec3fZero, gVec3sZero);

    return obj;
}

struct Object *spawn_object(struct Object *parent, ModelID32 model, const BehaviorScript *behavior) {
    struct Object *obj = spawn_object_at_origin(parent, 0, model, behavior);

    obj_copy_pos_and_angle(obj, parent);

    return obj;
}

struct Object *try_to_spawn_object(s16 offsetY, f32 scale, struct Object *parent, ModelID32 model,
                                   const BehaviorScript *behavior) {
    if (gFreeObjectList.next != NULL) {
        struct Object *obj = spawn_object(parent, model, behavior);
        obj->oPosY += offsetY;
        obj_scale(obj, scale);
        return obj;
    } else {
        return NULL;
    }
}

struct Object *spawn_object_with_scale(struct Object *parent, ModelID32 model, const BehaviorScript *behavior, f32 scale) {
    struct Object *obj = spawn_object_at_origin(parent, 0, model, behavior);

    obj_copy_pos_and_angle(obj, parent);
    obj_scale(obj, scale);

    return obj;
}

static void obj_build_relative_transform(struct Object *obj) {
    obj_build_transform_from_pos_and_angle(obj, O_PARENT_RELATIVE_POS_INDEX, O_FACE_ANGLE_INDEX);
    obj_translate_local(obj, O_POS_INDEX, O_PARENT_RELATIVE_POS_INDEX);
}

struct Object *spawn_object_relative(s16 behaviorParam, s16 relativePosX, s16 relativePosY, s16 relativePosZ,
                                     struct Object *parent, ModelID32 model, const BehaviorScript *behavior) {
    struct Object *obj = spawn_object_at_origin(parent, 0, model, behavior);

    obj_copy_pos_and_angle(obj, parent);
    obj_set_parent_relative_pos(obj, relativePosX, relativePosY, relativePosZ);
    obj_build_relative_transform(obj);

    obj->oBehParams2ndByte = behaviorParam;
    SET_BPARAM2(obj->oBehParams, behaviorParam);

    return obj;
}

struct Object *spawn_object_relative_with_scale(s16 behaviorParam, s16 relativePosX, s16 relativePosY,
                                                s16 relativePosZ, f32 scale, struct Object *parent,
                                                ModelID32 model, const BehaviorScript *behavior) {
    struct Object *obj = spawn_object_relative(behaviorParam, relativePosX, relativePosY, relativePosZ,
                                               parent, model, behavior);
    obj_scale(obj, scale);

    return obj;
}

void cur_obj_move_using_vel(void) {
    vec3f_add(&o->oPosVec, &o->oVelVec);
}

void obj_copy_graph_y_offset(struct Object *dst, struct Object *src) {
    dst->oGraphYOffset = src->oGraphYOffset;
}

void obj_copy_pos_and_angle(struct Object *dst, struct Object *src) {
    obj_copy_pos(dst, src);
    obj_copy_angle(dst, src);
}

void obj_copy_pos(struct Object *dst, struct Object *src) {
    vec3f_copy(&dst->oPosVec, &src->oPosVec);
}

void obj_copy_angle(struct Object *dst, struct Object *src) {
    vec3i_copy(&dst->oMoveAngleVec, &src->oMoveAngleVec);
    vec3i_copy(&dst->oFaceAngleVec, &src->oFaceAngleVec);
}

void obj_set_gfx_pos_from_pos(struct Object *obj) {
    vec3f_copy(obj->header.gfx.pos, &obj->oPosVec);
}

void obj_init_animation(struct Object *obj, s32 animIndex) {
    struct Animation **anims = o->oAnimations;
    geo_obj_init_animation(&obj->header.gfx, &anims[animIndex]);
}

void obj_apply_scale_to_transform(struct Object *obj) {
    Vec3f scale;
    vec3f_copy(scale, obj->header.gfx.scale);
    vec3_mul_val(obj->transform[0], scale[0]);
    vec3_mul_val(obj->transform[1], scale[1]);
    vec3_mul_val(obj->transform[2], scale[2]);
}

void obj_copy_scale(struct Object *dst, struct Object *src) {
    vec3f_copy(dst->header.gfx.scale, src->header.gfx.scale);
}

void obj_scale_xyz(struct Object *obj, f32 xScale, f32 yScale, f32 zScale) {
    vec3f_set(obj->header.gfx.scale, xScale, yScale, zScale);
}

void obj_scale(struct Object *obj, f32 scale) {
    vec3_same(obj->header.gfx.scale, scale);
}

void cur_obj_scale(f32 scale) {
    vec3_same(o->header.gfx.scale, scale);
}

void cur_obj_init_animation(s32 animIndex) {
    struct Animation **anims = o->oAnimations;
    geo_obj_init_animation(&o->header.gfx, &anims[animIndex]);
}

void super_cum_working(struct Object *obj, s32 animIndex) {
    struct Animation **anims = obj->oAnimations;
    geo_obj_init_animation(&obj->header.gfx, &anims[animIndex]);
}

void cur_obj_init_animation_with_sound(s32 animIndex) {
    struct Animation **anims = o->oAnimations;
    geo_obj_init_animation(&o->header.gfx, &anims[animIndex]);
    o->oSoundStateID = animIndex;
}

void cur_obj_init_animation_with_accel_and_sound(s32 animIndex, f32 accel) {
    struct Animation **anims = o->oAnimations;
    s32 animAccel = (s32)(accel * 65536.0f);
    geo_obj_init_animation_accel(&o->header.gfx, &anims[animIndex], animAccel);
    o->oSoundStateID = animIndex;
}

void obj_init_animation_with_sound(struct Object *obj, const struct Animation * const* animations, s32 animIndex) {
    struct Animation **anims = (struct Animation **)animations;
    obj->oAnimations = (struct Animation **)animations;
    geo_obj_init_animation(&obj->header.gfx, &anims[animIndex]);
    obj->oSoundStateID = animIndex;
}

void cur_obj_enable_rendering_and_become_tangible(struct Object *obj) {
    obj->header.gfx.node.flags |= GRAPH_RENDER_ACTIVE;
    obj->oIntangibleTimer = 0;
}

void cur_obj_enable_rendering(void) {
    o->header.gfx.node.flags |= GRAPH_RENDER_ACTIVE;
}

void cur_obj_disable_rendering_and_become_intangible(struct Object *obj) {
    obj->header.gfx.node.flags &= ~GRAPH_RENDER_ACTIVE;
    obj->oIntangibleTimer = -1;
}

void cur_obj_disable_rendering(void) {
    o->header.gfx.node.flags &= ~GRAPH_RENDER_ACTIVE;
}

void cur_obj_unhide(void) {
    o->header.gfx.node.flags &= ~GRAPH_RENDER_INVISIBLE;
}

void cur_obj_hide(void) {
    o->header.gfx.node.flags |= GRAPH_RENDER_INVISIBLE;
}

void cur_obj_set_pos_relative(struct Object *other, f32 dleft, f32 dy, f32 dforward) {
    f32 facingZ = coss(other->oMoveAngleYaw);
    f32 facingX = sins(other->oMoveAngleYaw);

    f32 dz = dforward * facingZ - dleft * facingX;
    f32 dx = dforward * facingX + dleft * facingZ;

    o->oMoveAngleYaw = other->oMoveAngleYaw;

    o->oPosX = other->oPosX + dx;
    o->oPosY = other->oPosY + dy;
    o->oPosZ = other->oPosZ + dz;
}

UNUSED void cur_obj_unused_init_on_floor(void) {
    cur_obj_enable_rendering();

    o->oPosY = find_floor_height(o->oPosX, o->oPosY, o->oPosZ);
    if (o->oPosY < FLOOR_LOWER_LIMIT_MISC) {
        cur_obj_set_pos_relative(o->parentObj, 0.0f, 0.0f, -70.0f);
        o->oPosY = find_floor_height(o->oPosX, o->oPosY, o->oPosZ);
    }
}

u32 get_object_list_from_behavior(const BehaviorScript *behavior) {
    u32 objectList;

    // If the first behavior command is "begin", then get the object list header
    // from there
    if ((behavior[0] >> 24) == 0) {
        objectList = (behavior[0] >> 16) & 0xFFFF;
    } else {
        objectList = OBJ_LIST_DEFAULT;
    }

    return objectList;
}

struct Object *cur_obj_nearest_object_with_behavior(const BehaviorScript *behavior) {
    f32 dist;
    return cur_obj_find_nearest_object_with_behavior(behavior, &dist);
}

f32 cur_obj_dist_to_nearest_object_with_behavior(const BehaviorScript *behavior) {
    f32 dist;
    if (cur_obj_find_nearest_object_with_behavior(behavior, &dist) == NULL) dist = 15000.0f;
    return dist;
}

#ifdef ACCELERATED_COLLISION_LOOKUP
struct nearby_iterator_context {
    uintptr_t *behaviorAddr;
    struct Object *result;
    f32 resultDist;
    u8 count;
};
s32 is_nearby_with_behavior(struct Object *a, struct Object *b, u8 cellX, u8 cellY, u8 cellZ, void* scratch) {
    struct nearby_iterator_context* ctx = scratch;
    ctx->count++;
    if (ctx->count > 9) {
        // All immediately adjacent cells tested, bail
        return TRUE;
    }
    if (!b || a == b) {
        return FALSE;
    }
    if (b->behavior == ctx->behaviorAddr
        && b->activeFlags != ACTIVE_FLAG_DEACTIVATED
    ) {
        f32 objDist = dist_between_objects(a, b);
        if (objDist < ctx->resultDist) {
            ctx->result = b;
            ctx->resultDist = objDist;
        }
        if (objDist > 0x20000) {
            return TRUE; // once we find an object too far away, bail
        }
    }
    return FALSE;
}
#endif // ACCELERATED_COLLISION_LOOKUP

struct Object *cur_obj_find_nearest_object_with_behavior(const BehaviorScript *behavior, f32 *dist) {
    uintptr_t *behaviorAddr = segmented_to_virtual(behavior);
    struct ObjectNode *listHead = &gObjectLists[get_object_list_from_behavior(behaviorAddr)];
    struct Object *obj = (struct Object *) listHead->next;
    struct Object *closestObj = NULL;
    f32 minDist = 0x20000;

#ifdef ACCELERATED_COLLISION_LOOKUP
    if (listHead == &gObjectLists[OBJ_LIST_SURFACE]) {
        struct nearby_iterator_context ctx;
        ctx.behaviorAddr = behaviorAddr;
        ctx.result = NULL;
        ctx.resultDist = minDist;
        ctx.count = 0;
        iterate_nearby_object_cells(o, &is_nearby_with_behavior, &ctx);
        if (ctx.result != NULL) {
            *dist = ctx.resultDist;
            return ctx.result;
        }
        // fallback to exhaustive traversal if nearby check is nonconclusive
    }
#endif // ACCELERATED_COLLISION_LOOKUP

    while (obj != (struct Object *) listHead) {
        if (obj->behavior == behaviorAddr
            && obj->activeFlags != ACTIVE_FLAG_DEACTIVATED
            && obj != o
        ) {
            f32 objDist = dist_between_objects(o, obj);
            if (objDist < minDist) {
                closestObj = obj;
                minDist = objDist;
            }
        }

        obj = (struct Object *) obj->header.next;
    }

    *dist = minDist;
    return closestObj;
}

struct Object *cur_obj_nearest_object_with_behavior_and_star_imbue(const BehaviorScript *behavior, f32 *dist) {
    uintptr_t *behaviorAddr = segmented_to_virtual(behavior);
    struct ObjectNode *listHead = &gObjectLists[get_object_list_from_behavior(behaviorAddr)];
    struct Object *obj = (struct Object *) listHead->next;
    struct Object *closestObj = NULL;
    f32 minDist = 0x20000;

    while (obj != (struct Object *) listHead) {
        if (obj->behavior == behaviorAddr
            && obj->activeFlags != ACTIVE_FLAG_DEACTIVATED
            && obj != o
            && obj->oImbue == IMBUE_STAR
        ) {
            f32 objDist = dist_between_objects(o, obj);
            if (objDist < minDist) {
                closestObj = obj;
                minDist = objDist;
            }
        }

        obj = (struct Object *) obj->header.next;
    }

    *dist = minDist;
    return closestObj;
}

struct Object *find_unimportant_object(void) {
    struct ObjectNode *listHead = &gObjectLists[OBJ_LIST_UNIMPORTANT];
    struct ObjectNode *obj = listHead->next;

    if (listHead == obj) {
        return NULL;
    }

    return (struct Object *) obj;
}

s32 count_unimportant_objects(void) {
    struct ObjectNode *listHead = &gObjectLists[OBJ_LIST_UNIMPORTANT];
    struct ObjectNode *obj = listHead->next;
    s32 count = 0;

    while (listHead != obj) {
        count++;
        obj = obj->next;
    }

    return count;
}

s32 count_objects_with_behavior(const BehaviorScript *behavior) {
    uintptr_t *behaviorAddr = segmented_to_virtual(behavior);
    struct ObjectNode *listHead = &gObjectLists[get_object_list_from_behavior(behaviorAddr)];
    struct ObjectNode *obj = listHead->next;
    s32 count = 0;

    while (listHead != obj) {
        if (((struct Object *) obj)->behavior == behaviorAddr) {
            count++;
        }

        obj = obj->next;
    }

    return count;
}

struct Object *cur_obj_find_nearby_held_actor(const BehaviorScript *behavior, f32 maxDist) {
    const BehaviorScript *behaviorAddr = segmented_to_virtual(behavior);
    struct ObjectNode *listHead = &gObjectLists[OBJ_LIST_GENACTOR];
    struct Object *obj = (struct Object *) listHead->next;
    struct Object *foundObj = NULL;

    while ((struct Object *) listHead != obj) {
        if (
            obj->behavior == behaviorAddr
            && obj->activeFlags != ACTIVE_FLAG_DEACTIVATED
            && obj->oHeldState != HELD_FREE
            && dist_between_objects(o, obj) < maxDist
        ) {
            // This includes the dropped and thrown states. By combining instant
            // release, this allows us to activate mama penguin remotely
            foundObj = obj;
            break;
        }

        obj = (struct Object *) obj->header.next;
    }

    return foundObj;
}

s32 count_imbued_red_coins(s32 objectList) {
    struct ObjectNode *listHead = &gObjectLists[objectList];
    struct ObjectNode *obj = listHead->next;
    s32 count = 0;

    while (obj != listHead) {
        if (((struct Object *) obj)->oImbue == IMBUE_RED_COIN) {
            count++;
        }
        obj = obj->next;
    }

    return count;
}

s32 count_red_coins(void) {
    struct ObjectNode *listHead = &gObjectLists[OBJ_LIST_LEVEL]; // red coin list
    struct ObjectNode *obj = listHead->next;
    s32 count = 0;

    while (obj != listHead) {
        if (((struct Object *) obj)->behavior == segmented_to_virtual(bhvRedCoin)) {
            count++;
        }
        obj = obj->next;
    }
    
    count += count_imbued_red_coins(OBJ_LIST_GENACTOR);
    count += count_imbued_red_coins(OBJ_LIST_SURFACE);
    return count;
}

static void cur_obj_reset_timer_and_subaction(void) {
    o->oTimer = 0;
    o->oSubAction = 0;
}

void cur_obj_change_action(s32 action) {
    o->oAction = action;
    o->oPrevAction = action;
    cur_obj_reset_timer_and_subaction();
}

void cur_obj_set_vel_from_mario_vel(f32 min, f32 mul) {
    f32 marioFwdVel = gMarioStates[0].forwardVel;
    f32 minVel = min * mul;

    if (marioFwdVel < minVel) {
        o->oForwardVel = minVel;
    } else {
        o->oForwardVel = marioFwdVel * mul;
    }
}

void cur_obj_reverse_animation(void) {
    if (o->header.gfx.animInfo.animFrame >= 0) {
        o->header.gfx.animInfo.animFrame--;
    }
}

void cur_obj_extend_animation_if_at_end(void) {
    s32 animFrame = o->header.gfx.animInfo.animFrame;
    s32 nearLoopEnd = o->header.gfx.animInfo.curAnim->loopEnd - 2;

    if (animFrame == nearLoopEnd) o->header.gfx.animInfo.animFrame--;
}

s32 cur_obj_check_if_near_animation_end(void) {
    s32 animFrame = o->header.gfx.animInfo.animFrame;
    s32 nearLoopEnd = o->header.gfx.animInfo.curAnim->loopEnd - 2;

    if ((o->header.gfx.animInfo.curAnim->flags & ANIM_FLAG_NOLOOP) && nearLoopEnd + 1 == animFrame) {
        return TRUE;
    }

    return animFrame == nearLoopEnd;
}

s32 cur_obj_check_if_at_animation_end(void) {
    return o->header.gfx.animInfo.animFrame == o->header.gfx.animInfo.curAnim->loopEnd - 1;
}

s32 cur_obj_check_anim_frame(s32 frame) {
    return o->header.gfx.animInfo.animFrame == frame;
}

s32 cur_obj_check_anim_frame_in_range(s32 startFrame, s32 rangeLength) {
    s32 animFrame = o->header.gfx.animInfo.animFrame;

    return animFrame >= startFrame && animFrame < startFrame + rangeLength;
}

s32 cur_obj_check_frame_prior_current_frame(s16 *frame) {
    s16 animFrame = o->header.gfx.animInfo.animFrame;

    while (*frame != -1) {
        if (*frame == animFrame) return TRUE;

        frame++;
    }

    return FALSE;
}

s32 mario_is_in_air_action(void) {
    return gMarioStates[0].action & ACT_FLAG_AIR;
}

s32 mario_is_dive_sliding(void) {
    return gMarioStates[0].action == ACT_DIVE_SLIDE;
}

void cur_obj_set_y_vel_and_animation(f32 yVel, s32 animIndex) {
    o->oVelY = yVel;
    cur_obj_init_animation_with_sound(animIndex);
}

void cur_obj_unrender_set_action_and_anim(s32 animIndex, s32 action) {
    cur_obj_become_intangible();
    cur_obj_disable_rendering();

    // only set animation if non-negative value
    if (animIndex >= 0) {
        cur_obj_init_animation_with_sound(animIndex);
    }

    o->oAction = action;
}

static void cur_obj_move_after_thrown_or_dropped(f32 forwardVel, f32 velY) {
    o->oMoveFlags = 0;
    o->oFloorHeight = find_floor(o->oPosX, o->oPosY + 160.0f, o->oPosZ, &o->oFloor);

    if (o->oFloorHeight > o->oPosY) {
        o->oPosY = o->oFloorHeight;
    } else if (o->oFloorHeight < FLOOR_LOWER_LIMIT_MISC) {
        //! OoB failsafe
        obj_copy_pos(o, gMarioObject);
        o->oFloorHeight = find_floor(o->oPosX, o->oPosY, o->oPosZ, &o->oFloor);
    }

    o->oForwardVel = forwardVel;
    o->oVelY = velY;

    if (o->oForwardVel != 0) {
        cur_obj_move_y(/*gravity*/ -4.0f, /*bounciness*/ -0.1f, /*buoyancy*/ 2.0f);
    }
}

void cur_obj_get_thrown_or_placed(f32 forwardVel, f32 velY, s32 thrownAction) {
    if (o->behavior == segmented_to_virtual(bhvBowser)) {
        // Interestingly, when bowser is thrown, he is offset slightly to
        // Mario's right
        cur_obj_set_pos_relative(o->parentObj, -41.684f, 85.859f, 321.577f);
    }

    cur_obj_become_tangible();
    cur_obj_enable_rendering();

    o->oHeldState = HELD_FREE;

    if ((o->oInteractionSubtype & INT_SUBTYPE_HOLDABLE_NPC) || forwardVel == 0.0f) {
        cur_obj_move_after_thrown_or_dropped(0.0f, 0.0f);
    } else {
        o->oAction = thrownAction;
        cur_obj_move_after_thrown_or_dropped(forwardVel, velY);
    }
}

void cur_obj_get_dropped(void) {
    cur_obj_become_tangible();
    cur_obj_enable_rendering();

    o->oHeldState = HELD_FREE;
    cur_obj_move_after_thrown_or_dropped(0.0f, 0.0f);
}

void obj_set_model(struct Object *obj, ModelID16 modelID) {
    obj->header.gfx.sharedChild = gLoadedGraphNodes[modelID];
}

void cur_obj_set_model(ModelID16 modelID) {
    o->header.gfx.sharedChild = gLoadedGraphNodes[modelID];
}

s32 obj_has_model(struct Object *obj, ModelID16 modelID) {
    return (obj->header.gfx.sharedChild == gLoadedGraphNodes[modelID]);
}

s32 cur_obj_has_model(ModelID16 modelID) {
    return (o->header.gfx.sharedChild == gLoadedGraphNodes[modelID]);
}

// HackerSM64 function
ModelID32 obj_get_model_id(struct Object *obj) {
    if (obj->header.gfx.sharedChild != NULL) {
        for (s32 i = MODEL_NONE; i < MODEL_ID_COUNT; i++) {
            if (obj->header.gfx.sharedChild == gLoadedGraphNodes[i]) {
                return i;
            }
        }
    }
    return MODEL_NONE;
}

void mario_set_flag(s32 flag) {
    gMarioStates[0].flags |= flag;
}

s32 cur_obj_clear_interact_status_flag(s32 flag) {
    if (o->oInteractStatus & flag) {
        o->oInteractStatus &= (flag ^ 0xFFFFFFFF);
        return TRUE;
    }
    return FALSE;
}

/**
 * Mark an object to be unloaded at the end of the frame.
 */
void obj_mark_for_deletion(struct Object *obj) {
#ifdef PUPPYLIGHTS
    obj_disable_light(obj);
#endif

    //! This clears all activeFlags. Since some of these flags disable behavior,
    //  setting it to 0 could potentially enable unexpected behavior. After an
    //  object is marked for deletion, it still updates on that frame (I think),
    //  so this is worth looking into.
    //! NOTE: Changing this can cause reference issues!
    obj->activeFlags = ACTIVE_FLAG_DEACTIVATED;
}

void cur_obj_disable(void) {
    cur_obj_disable_rendering();
    cur_obj_hide();
    cur_obj_become_intangible();
}

void cur_obj_become_intangible(void) {
    // When the timer is negative, the object is intangible and the timer
    // doesn't count down
    o->oIntangibleTimer = -1;
}

void cur_obj_become_tangible(void) {
    o->oIntangibleTimer = 0;
}

void obj_become_tangible(struct Object *obj) {
    obj->oIntangibleTimer = 0;
}

void cur_obj_update_floor_height(void) {
    struct Surface *floor;
    f32 floorBuffer = (o->oFlags & OBJ_FLAG_SIMPLE_WALL_CHECKS) ? 0.f : MAX(o->oWallHitboxRadius-FIND_FLOOR_BUFFER, 0);
    o->oFloorHeight = find_floor(o->oPosX, o->oPosY + floorBuffer, o->oPosZ, &floor);
}

struct Surface *cur_obj_update_floor_height_and_get_floor(void) {
    struct Surface *floor;
    f32 floorBuffer = (o->oFlags & OBJ_FLAG_SIMPLE_WALL_CHECKS) ? 0.f : MAX(o->oWallHitboxRadius-FIND_FLOOR_BUFFER, 0);
    o->oFloorHeight = find_floor(o->oPosX, o->oPosY + floorBuffer, o->oPosZ, &floor);
    return floor;
}

static void apply_drag_to_value(f32 *value, f32 dragStrength) {
    f32 decel;

    if (*value != 0) {
        //! Can overshoot if |*value| > 1 / (dragStrength * 0.0001)
        decel = sqr(*value) * (dragStrength * 0.0001L);

        if (*value > 0) {
            *value -= decel;
            if (*value < 0.001L) {
                *value = 0;
            }
        } else {
            *value += decel;
            if (*value > -0.001L) {
                *value = 0;
            }
        }
    }
}

void cur_obj_apply_drag_xz(f32 dragStrength) {
    apply_drag_to_value(&o->oVelX, dragStrength);
    apply_drag_to_value(&o->oVelZ, dragStrength);
}

static void cur_obj_move_xz(f32 steepSlopeNormalY, s32 careAboutEdgesAndSteepSlopes) {
    struct Surface *intendedFloor;

    f32 intendedX = o->oPosX + o->oVelX;
    f32 intendedZ = o->oPosZ + o->oVelZ;

    if (o->oQuicksandDepth > 0) {
        intendedX = o->oPosX + (o->oVelX*.5f);
        intendedZ = o->oPosZ + (o->oVelZ*.5f);
    }

    f32 floorBuffer = (o->oFlags & OBJ_FLAG_SIMPLE_WALL_CHECKS) ? FIND_FLOOR_BUFFER : MAX(o->oWallHitboxRadius-FIND_FLOOR_BUFFER, 0);

    f32 intendedFloorHeight = find_floor(intendedX, o->oPosY + floorBuffer, intendedZ, &intendedFloor);
    f32 deltaFloorHeight = intendedFloorHeight - o->oFloorHeight;

    o->oMoveFlags &= ~OBJ_MOVE_HIT_EDGE;


    if (intendedFloorHeight < FLOOR_LOWER_LIMIT_MISC) {
        // Don't move into OoB
        o->oMoveFlags |= OBJ_MOVE_HIT_EDGE;
        return;
    }

    Vec3f normal;
    get_surface_normal(normal, intendedFloor);
    
    if (deltaFloorHeight < 5.0f) {
        if (!careAboutEdgesAndSteepSlopes) {
            // If we don't care about edges or steep slopes, okay to move
            o->oPosX = intendedX;
            o->oPosZ = intendedZ;
        } else if (deltaFloorHeight < -50.0f && (o->oMoveFlags & OBJ_MOVE_ON_GROUND)) {
            // Don't walk off an edge
            o->oMoveFlags |= OBJ_MOVE_HIT_EDGE;
        } else if (normal[1] > steepSlopeNormalY) {
            // Allow movement onto a slope, provided it's not too steep
            o->oPosX = intendedX;
            o->oPosZ = intendedZ;
        } else {
            // We are likely trying to move onto a steep downward slope
            o->oMoveFlags |= OBJ_MOVE_HIT_EDGE;
        }
    } else if (normal[1] > steepSlopeNormalY || o->oPosY > intendedFloorHeight) {
        // Allow movement upward, provided either:
        // - The target floor is flat enough (e.g. walking up stairs)
        // - We are above the target floor (most likely in the air)
        o->oPosX = intendedX;
        o->oPosZ = intendedZ;
    }
    // We are likely trying to move onto a steep upward slope
}

static void cur_obj_move_update_underwater_flags(void) {
    f32 decelY = (f32)(sqrtf(o->oVelY * o->oVelY) * (o->oDragStrength * 7.0f)) / 100.0L;

    if (o->oVelY > 0) {
        o->oVelY -= decelY;
    } else {
        o->oVelY += decelY;
    }

    if (o->oPosY < o->oFloorHeight) {
        o->oPosY = o->oFloorHeight;
        o->oMoveFlags |= OBJ_MOVE_UNDERWATER_ON_GROUND;
    } else {
        o->oMoveFlags |= OBJ_MOVE_UNDERWATER_OFF_GROUND;
    }
}

void cur_obj_check_danger_floors() {
    o->oMoveFlags &= ~(OBJ_MOVE_ABOVE_LAVA | OBJ_MOVE_ABOVE_DEATH_BARRIER);
    if (o->oFloor) {
        u32 floorType = o->oFloor->type;
        if (SURFACE_IS_BURNING(floorType)) {
            o->oMoveFlags |= OBJ_MOVE_ABOVE_LAVA;
        } else if ((floorType == SURFACE_DEATH_PLANE) || (floorType == SURFACE_VERTICAL_WIND)) {
            //! This maybe misses SURFACE_WARP
            o->oMoveFlags |= OBJ_MOVE_ABOVE_DEATH_BARRIER;
        }
    }
}

static void cur_obj_move_update_ground_air_flags(UNUSED f32 gravity, f32 bounciness) {
    o->oMoveFlags &= ~OBJ_MOVE_BOUNCE;
    f32 floorOffset = ((o->oMoveFlags & (OBJ_MOVE_ON_GROUND | OBJ_MOVE_LANDED) && (o->oVelY < 5.f)) ? 100.f : 0.f);

    if (o->oPosY < (o->oFloorHeight + floorOffset)) {
        // On the first frame that we touch the ground, set OBJ_MOVE_LANDED.
        // On subsequent frames, set OBJ_MOVE_ON_GROUND
        if (!(o->oMoveFlags & OBJ_MOVE_ON_GROUND)) {
            if (clear_move_flag(&o->oMoveFlags, OBJ_MOVE_LANDED)) {
                o->oMoveFlags |= OBJ_MOVE_ON_GROUND;
            } else {
                o->oMoveFlags |= OBJ_MOVE_LANDED;
            }
        }

        o->oPosY = o->oFloorHeight;

        if (o->oVelY < 0.0f) {
            o->oVelY *= bounciness;
        }

        if (o->oVelY > 5.0f) {
            //! This overestimates since velY could be > 5 here
            //! without bounce (e.g. jump into misa).
            o->oMoveFlags |= OBJ_MOVE_BOUNCE;
        }
    } else {
        o->oMoveFlags &= ~OBJ_MOVE_LANDED;
        if (clear_move_flag(&o->oMoveFlags, OBJ_MOVE_ON_GROUND)) {
            o->oMoveFlags |= OBJ_MOVE_LEFT_GROUND;
        }
    }

    o->oMoveFlags &= ~OBJ_MOVE_MASK_IN_WATER;
    cur_obj_check_danger_floors();
}

static f32 cur_obj_move_y_and_get_water_level(f32 gravity, f32 buoyancy) {
    o->oVelY += gravity + buoyancy;
    if (o->oVelY < -78.0f) {
        o->oVelY = -78.0f;
    }

    o->oPosY += o->oVelY;
    if (o->activeFlags & ACTIVE_FLAG_IGNORE_ENV_BOXES) {
        return FLOOR_LOWER_LIMIT;
    }

    return cmm_get_water_level(o->oPosX, o->oPosY, o->oPosZ);
}

void cur_obj_move_y(f32 gravity, f32 bounciness, f32 buoyancy) {
    f32 waterLevel;

    o->oMoveFlags &= ~OBJ_MOVE_LEFT_GROUND;

    if (o->oMoveFlags & OBJ_MOVE_AT_WATER_SURFACE) {
        if (o->oVelY > 5.0f) {
            o->oMoveFlags &= ~OBJ_MOVE_MASK_IN_WATER;
            o->oMoveFlags |= OBJ_MOVE_LEAVING_WATER;
        }
    }

    if (!(o->oMoveFlags & OBJ_MOVE_MASK_IN_WATER)) {
        waterLevel = cur_obj_move_y_and_get_water_level(gravity, 0.0f);
        if (o->oPosY > waterLevel) {
            //! We only handle floor collision if the object does not enter
            //  water. This allows e.g. coins to clip through floors if they
            //  enter water on the same frame.
            cur_obj_move_update_ground_air_flags(gravity, bounciness);
        } else {
            o->oMoveFlags |= OBJ_MOVE_ENTERED_WATER;
            o->oMoveFlags &= ~OBJ_MOVE_MASK_ON_GROUND;
            if (o->oWallHitboxRadius < 200.0f) {
                cur_obj_play_sound_2(SOUND_OBJ_DIVING_INTO_WATER);
            } else {
                cur_obj_play_sound_2(SOUND_OBJ_DIVING_IN_WATER);
            }
        }
    } else {
        o->oMoveFlags &= ~OBJ_MOVE_ENTERED_WATER;

        waterLevel = cur_obj_move_y_and_get_water_level(gravity, buoyancy);
        if (o->oPosY < waterLevel) {
            cur_obj_move_update_underwater_flags();
        } else {
            if (o->oPosY < o->oFloorHeight) {
                o->oPosY = o->oFloorHeight;
                o->oMoveFlags &= ~OBJ_MOVE_MASK_IN_WATER;
            } else {
                if ((o->oPosY - waterLevel) < buoyancy) { // Still at surface
                    o->oPosY = waterLevel;
                    o->oVelY = 0.0f;
                    o->oMoveFlags &= ~(OBJ_MOVE_UNDERWATER_OFF_GROUND | OBJ_MOVE_UNDERWATER_ON_GROUND);
                    o->oMoveFlags |= OBJ_MOVE_AT_WATER_SURFACE;
                } else {
                    o->oMoveFlags &= ~OBJ_MOVE_MASK_IN_WATER;
                }
            }
        }
    }

    COND_BIT((!(o->oMoveFlags & (OBJ_MOVE_MASK_ON_GROUND | OBJ_MOVE_AT_WATER_SURFACE | OBJ_MOVE_UNDERWATER_OFF_GROUND))), o->oMoveFlags, OBJ_MOVE_IN_AIR);
}

static s32 clear_move_flag(u32 *bitSet, s32 flag) {
    if (*bitSet & flag) {
        *bitSet &= flag ^ 0xFFFFFFFF;
        return TRUE;
    } else {
        return FALSE;
    }
}

void cur_obj_unused_resolve_wall_collisions(f32 offsetY, f32 radius) {
    if (radius > 0.1L) {
        f32_find_wall_collision(&o->oPosX, &o->oPosY, &o->oPosZ, offsetY, radius);
    }
}

void cur_obj_move_xz_using_fvel_and_yaw(void) {
    o->oVelX = o->oForwardVel * sins(o->oMoveAngleYaw);
    o->oVelZ = o->oForwardVel * coss(o->oMoveAngleYaw);

    o->oPosX += o->oVelX;
    o->oPosZ += o->oVelZ;
}

void cur_obj_move_y_with_terminal_vel(void) {
    if (o->oVelY < -70.0f) {
        o->oVelY = -70.0f;
    }

    o->oPosY += o->oVelY;
}

void cur_obj_compute_vel_xz(void) {
    o->oVelX = o->oForwardVel * sins(o->oMoveAngleYaw);
    o->oVelZ = o->oForwardVel * coss(o->oMoveAngleYaw);
}

f32 increment_velocity_toward_range(f32 value, f32 center, f32 zeroThreshold, f32 increment) {
    f32 relative;
    if ((relative = value - center) > 0) {
        if (relative < zeroThreshold) {
            return 0.0f;
        } else {
            return -increment;
        }
    } else {
        if (relative > -zeroThreshold) {
            return 0.0f;
        } else {
            return increment;
        }
    }
}

s32 obj_check_if_collided_with_object(struct Object *obj1, struct Object *obj2) {
    s32 i;
    for (i = 0; i < obj1->numCollidedObjs; i++) {
        if (obj1->collidedObjs[i] == obj2) {
            return TRUE;
        }
    }

    return FALSE;
}

void cur_obj_set_behavior(const BehaviorScript *behavior) {
    o->behavior = segmented_to_virtual(behavior);
}

void obj_set_behavior(struct Object *obj, const BehaviorScript *behavior) {
    obj->behavior = segmented_to_virtual(behavior);
}

s32 cur_obj_has_behavior(const BehaviorScript *behavior) {
    return (o->behavior == segmented_to_virtual(behavior));
}

s32 obj_has_behavior(struct Object *obj, const BehaviorScript *behavior) {
    return (obj->behavior == segmented_to_virtual(behavior));
}

f32 cur_obj_lateral_dist_from_mario_to_home(void) {
    f32 dx = o->oHomeX - gMarioObject->oPosX;
    f32 dz = o->oHomeZ - gMarioObject->oPosZ;
    return sqrtf(sqr(dx) + sqr(dz));
}

f32 cur_obj_lateral_dist_to_home(void) {
    f32 dx = o->oHomeX - o->oPosX;
    f32 dz = o->oHomeZ - o->oPosZ;
    return sqrtf(sqr(dx) + sqr(dz));
}

f32 pythag(f32 dx, f32 dz) {
    f32 dist;

    dist = sqrtf(dx * dx + dz * dz);
    return dist;
}

s32 cur_obj_outside_home_square(f32 halfLength) {
    if (o->oHomeX - halfLength > o->oPosX) return TRUE;
    if (o->oHomeX + halfLength < o->oPosX) return TRUE;
    if (o->oHomeZ - halfLength > o->oPosZ) return TRUE;
    if (o->oHomeZ + halfLength < o->oPosZ) return TRUE;
    return FALSE;
}

s32 cur_obj_outside_home_rectangle(f32 minX, f32 maxX, f32 minZ, f32 maxZ) {
    if (o->oHomeX + minX > o->oPosX) return TRUE;
    if (o->oHomeX + maxX < o->oPosX) return TRUE;
    if (o->oHomeZ + minZ > o->oPosZ) return TRUE;
    if (o->oHomeZ + maxZ < o->oPosZ) return TRUE;
    return FALSE;
}

void cur_obj_set_pos_to_home(void) {
    vec3f_copy(&o->oPosVec, &o->oHomeVec);
}

void cur_obj_set_pos_to_home_and_stop(void) {
    cur_obj_set_pos_to_home();

    o->oForwardVel = 0;
    o->oVelY = 0;
}

void cur_obj_shake_y(f32 amount) {
    //! Technically could cause a bit of drift, but not much
    if ((o->oTimer & 0x1) == 0) {
        o->oPosY += amount;
    } else {
        o->oPosY -= amount;
    }
}

void cur_obj_start_cam_event(UNUSED struct Object *obj, s32 cameraEvent) {
    gPlayerCameraState->cameraEvent = (s16) cameraEvent;
    gSecondCameraFocus = o;
}

void obj_set_billboard(struct Object *obj) {
    obj->header.gfx.node.flags |= GRAPH_RENDER_BILLBOARD;
}

void cur_obj_set_hitbox_radius_and_height(f32 radius, f32 height) {
    o->hitboxRadius = radius;
    o->hitboxHeight = height;
}

void cur_obj_set_hurtbox_radius_and_height(f32 radius, f32 height) {
    o->hurtboxRadius = radius;
    o->hurtboxHeight = height;
}

static void obj_spawn_loot_coins(struct Object *obj, s32 numCoins, f32 baseYVel,
                                    const BehaviorScript *coinBehavior,
                                    s16 posJitter, ModelID16 model) {
    s32 i;
    struct Surface *floor;
    struct Object *coin;

    f32 spawnHeight = find_floor(obj->oPosX, obj->oPosY, obj->oPosZ, &floor);
    if (obj->oPosY - spawnHeight > 100.0f) {
        spawnHeight = obj->oPosY;
    }

    for (i = 0; i < numCoins; i++) {
        if (obj->oNumLootCoins <= 0) {
            break;
        }

        obj->oNumLootCoins--;

        coin = spawn_object(obj, model, coinBehavior);
        obj_translate_xz_random(coin, posJitter);
        coin->oPosY = spawnHeight + 5.f;
        coin->oCoinBaseYVel = baseYVel;
    }
}

void obj_spawn_loot_blue_coins(struct Object *obj, s32 numCoins, f32 baseYVel, s16 posJitter) {
    obj_spawn_loot_coins(obj, numCoins, baseYVel, bhvBlueCoinJumping, posJitter, MODEL_BLUE_COIN);
}

void obj_spawn_loot_yellow_coins(struct Object *obj, s32 numCoins, f32 baseYVel) {
    obj_spawn_loot_coins(obj, numCoins, baseYVel, bhvSingleCoinGetsSpawned, 0, MODEL_YELLOW_COIN);
}

void cur_obj_spawn_loot_coin_at_mario_pos(void) {
    if (o->oNumLootCoins <= 0) {
        return;
    }

    o->oNumLootCoins--;

    struct Object *coin = spawn_object(o, MODEL_YELLOW_COIN, bhvSingleCoinGetsSpawned);
    coin->oVelY = 30.0f;

    obj_copy_pos(coin, gMarioObject);
}

UNUSED f32 cur_obj_abs_y_dist_to_home(void) {
    return absf(o->oHomeY - o->oPosY);
}

UNUSED s32 cur_obj_advance_looping_anim(void) {
    s32 animFrame = o->header.gfx.animInfo.animFrame;
    s32 loopEnd = o->header.gfx.animInfo.curAnim->loopEnd;

    if (animFrame < 0) {
        animFrame = 0;
    } else if (loopEnd - 1 == animFrame) {
        animFrame = 0;
    } else {
        animFrame++;
    }

    return (animFrame << 16) / loopEnd;
}

static s32 cur_obj_detect_steep_floor(s16 steepAngleDegrees) {
    struct Surface *intendedFloor;
    if (o->oForwardVel != 0.0f) {
        f32 intendedX = o->oPosX + o->oVelX;
        f32 intendedZ = o->oPosZ + o->oVelZ;
        f32 intendedFloorHeight = find_floor(intendedX, o->oPosY, intendedZ, &intendedFloor);
        f32 deltaFloorHeight = intendedFloorHeight - o->oFloorHeight;

        if (!intendedFloor) {
            return FALSE;
        }

        Vec3f normal;
        get_surface_normal(normal, intendedFloor);

        if (intendedFloorHeight < FLOOR_LOWER_LIMIT_MISC) {
            o->oWallAngle = (o->oMoveAngleYaw + 0x8000);
            return TRUE;
        } else if ((normal[1] < coss((s16)(steepAngleDegrees * (0x10000 / 360))))
                   && (deltaFloorHeight > 0)
                   && (intendedFloorHeight > o->oPosY)) {
            o->oWallAngle = atan2s(normal[2], normal[0]);
            return TRUE;
        } else {
            return FALSE;
        }
    }

    return FALSE;
}

s32 cur_obj_resolve_wall_collisions(void) {
    f32 radius = o->oWallHitboxRadius;
    s32 wallFound = FALSE;
    if (radius <= 0.1f) return FALSE;

    struct WallCollisionData collisionData;
    collisionData.radius  = radius;
    collisionData.x = (s16) o->oPosX;
    collisionData.y = (s16) o->oPosY;
    collisionData.z = (s16) o->oPosZ;
    collisionData.offsetY = collisionData.radius + 1.f;
    if (o->oFlags & OBJ_FLAG_SIMPLE_WALL_CHECKS) collisionData.offsetY = 10.f;

    s32 numCollisions = find_wall_collisions(&collisionData);
    if (numCollisions != 0) {
        struct Surface *wall = collisionData.walls[collisionData.numWalls - 1];
        o->oWallAngle = SURFACE_YAW(wall);
        wallFound = TRUE;
    }

    if ((!o->oFlags & OBJ_FLAG_SIMPLE_WALL_CHECKS) && (radius > 100.f)) { // extra check for chonky boys
        collisionData.offsetY += 100.f;
        numCollisions = find_wall_collisions(&collisionData);
        if (numCollisions != 0) {
            struct Surface *wall = collisionData.walls[collisionData.numWalls - 1];
            o->oWallAngle = SURFACE_YAW(wall);
            wallFound = TRUE;
        }
    }

    if (wallFound) {
        o->oPosX = collisionData.x;
        o->oPosY = collisionData.y;
        o->oPosZ = collisionData.z;
        return (abs_angle_diff(o->oWallAngle, o->oMoveAngleYaw) > 0x4000);
    }

    return FALSE;
}

static void cur_obj_update_floor(void) {
    struct Surface *floor = cur_obj_update_floor_height_and_get_floor();
    o->oFloor = floor;

    if (floor != NULL) {
        SurfaceType floorType = floor->type;
        o->oFloorType = floorType;
    } else {
        o->oFloorType = SURFACE_DEFAULT;
        o->oFloorRoom = 0;
    }
}


void cur_obj_update_ceiling(void) {
    struct Surface *ceil;
    f32 ceilHeight = find_ceil(o->oPosX, o->oPosY + o->hitboxHeight - 10.f, o->oPosZ, &ceil);

    if (!ceil) return;

    if (o->oVelY >= 0.f) {
        if ((o->oPosY + o->oVelY + o->hitboxHeight - o->hitboxDownOffset) > ceilHeight) {
            // ugly code to avoid self ceiling checks on wood platforms
            if (o->behavior == segmented_to_virtual(bhvWoodPlat)) {
                if (ceil->object == o->prevObj) {
                    return;
                }
            }
            
            f32 targetYPos = ceilHeight - o->hitboxHeight + o->hitboxDownOffset;
            if (targetYPos > o->oFloorHeight) { // dont move into a floor
                o->oPosY = targetYPos;
            }
            o->oMoveFlags |= OBJ_MOVE_HIT_WALL;
            o->oVelY = 0.f;
            o->oForwardVel = 0.f;
        }
    }
}

static void cur_obj_update_floor_and_resolve_wall_collisions(s16 steepSlopeDegrees) {
    obj_add_self_to_physics_list();
    if (o->activeFlags & ACTIVE_FLAG_FAR_AWAY) {
        cur_obj_update_floor();
        o->oMoveFlags &= ~(OBJ_MOVE_HIT_WALL | OBJ_MOVE_MASK_IN_WATER);

        if (o->oPosY > o->oFloorHeight) {
            o->oMoveFlags |= OBJ_MOVE_IN_AIR;
        }
    } else {
        o->oMoveFlags &= ~OBJ_MOVE_HIT_WALL;
        if (cur_obj_resolve_wall_collisions()) {
            o->oMoveFlags |= OBJ_MOVE_HIT_WALL;
        }

        cur_obj_update_floor();

        if (o->oPosY > o->oFloorHeight) {
            o->oMoveFlags |= OBJ_MOVE_IN_AIR;
        }

        if (cur_obj_detect_steep_floor(steepSlopeDegrees)) {
            o->oMoveFlags |= OBJ_MOVE_HIT_WALL;
        }

        cur_obj_update_ceiling();
    }
}

void cur_obj_update_floor_and_walls(void) {
    cur_obj_update_floor_and_resolve_wall_collisions(60);
}

void cur_obj_move_standard(s16 steepSlopeAngleDegrees) {
    f32 gravity = o->oGravity;
    f32 bounciness = o->oBounciness;
    f32 buoyancy = o->oBuoyancy;
    f32 dragStrength = o->oDragStrength;
    f32 steepSlopeNormalY;
    s32 careAboutEdgesAndSteepSlopes = FALSE;
    s32 negativeSpeed = FALSE;

    obj_add_self_to_physics_list();

    //! Because some objects allow these active flags to be set but don't
    //  avoid updating when they are, we end up with "partial" updates, where
    //  an object's internal state will be updated, but it doesn't move.
    //  This allows numerous glitches and is typically referred to as
    //  deactivation (though this term has a different meaning in the code).
    //  Objects that do this will be marked with //PARTIAL_UPDATE.
    if (!(o->activeFlags & (ACTIVE_FLAG_FAR_AWAY | ACTIVE_FLAG_IN_DIFFERENT_ROOM))) {
        if (steepSlopeAngleDegrees < 0) {
            careAboutEdgesAndSteepSlopes = TRUE;
            steepSlopeAngleDegrees = -steepSlopeAngleDegrees;
        }
        // Optimize for the most commonly used values
        if (steepSlopeAngleDegrees == 78) {
            steepSlopeNormalY =  COS78;
        } else if (steepSlopeAngleDegrees == -78) {
            steepSlopeNormalY = -COS78;
        } else {
            steepSlopeNormalY = coss(DEGREES(steepSlopeAngleDegrees));
        }

        cur_obj_compute_vel_xz();
        cur_obj_apply_drag_xz(dragStrength);

        cur_obj_move_xz(steepSlopeNormalY, careAboutEdgesAndSteepSlopes);
        cur_obj_move_y(gravity, bounciness, buoyancy);

        if (o->oForwardVel < 0.0f) {
            negativeSpeed = TRUE;
        }
        o->oForwardVel = sqrtf(sqr(o->oVelX) + sqr(o->oVelZ));
        if (negativeSpeed == TRUE) {
            o->oForwardVel = -o->oForwardVel;
        }

        cur_obj_floor_interactions(0);
    } else {
        cur_obj_move_y(gravity, bounciness, buoyancy);
        cur_obj_floor_interactions(0);
    }
}

UNUSED static s32 cur_obj_within_bounds(f32 bounds) {
    if (o->oPosX < -bounds || bounds < o->oPosX) return FALSE;
    if (o->oPosY < -bounds || bounds < o->oPosY) return FALSE;
    if (o->oPosZ < -bounds || bounds < o->oPosZ) return FALSE;
    return TRUE;
}

void cur_obj_move_using_vel_and_gravity(void) {
    o->oVelY += o->oGravity; //! No terminal velocity
    vec3f_add(&o->oPosVec, &o->oVelVec);
}

void cur_obj_move_using_fvel_and_gravity(void) {
    cur_obj_compute_vel_xz();
    cur_obj_move_using_vel_and_gravity(); //! No terminal velocity
}

void obj_set_pos_relative(struct Object *obj, struct Object *other, f32 dleft, f32 dy, f32 dforward) {
    f32 facingZ = coss(other->oMoveAngleYaw);
    f32 facingX = sins(other->oMoveAngleYaw);

    f32 dz = dforward * facingZ - dleft * facingX;
    f32 dx = dforward * facingX + dleft * facingZ;

    obj->oMoveAngleYaw = other->oMoveAngleYaw;

    obj->oPosX = other->oPosX + dx;
    obj->oPosY = other->oPosY + dy;
    obj->oPosZ = other->oPosZ + dz;
}

s32 cur_obj_angle_to_home(void) {
    f32 dx = o->oHomeX - o->oPosX;
    f32 dz = o->oHomeZ - o->oPosZ;
    return atan2s(dz, dx);
}

f32 floatclamp(f32 flote, f32 clampn, f32 clampp) {
    if (flote < clampn) {
        return clampn;
    }
    if (flote > clampp) {
        return clampp;
    }
    return flote;
}

void obj_set_gfx_pos_at_obj_pos(struct Object *obj1, struct Object *obj2) {
    vec3f_copy_y_off(obj1->header.gfx.pos, &obj2->oPosVec, obj2->oGraphYOffset);

    obj1->header.gfx.angle[0] = obj2->oMoveAnglePitch & 0xFFFF;
    obj1->header.gfx.angle[1] = obj2->oMoveAngleYaw & 0xFFFF;
    obj1->header.gfx.angle[2] = obj2->oMoveAngleRoll & 0xFFFF;
}

/**
 * Transform the vector at localTranslateIndex into the object's local
 * coordinates, and then add it to the vector at posIndex.
 */
void obj_translate_local(struct Object *obj, s16 posIndex, s16 localTranslateIndex) {
    Vec3f d;
    vec3f_copy(d, &obj->rawData.asF32[localTranslateIndex]);

    obj->rawData.asF32[posIndex + 0] += ((obj->transform[0][0] * d[0]) + (obj->transform[1][0] * d[1]) + (obj->transform[2][0] * d[2]));
    obj->rawData.asF32[posIndex + 1] += ((obj->transform[0][1] * d[0]) + (obj->transform[1][1] * d[1]) + (obj->transform[2][1] * d[2]));
    obj->rawData.asF32[posIndex + 2] += ((obj->transform[0][2] * d[0]) + (obj->transform[1][2] * d[1]) + (obj->transform[2][2] * d[2]));
}

void obj_build_transform_from_pos_and_angle(struct Object *obj, s16 posIndex, s16 angleIndex) {
    Vec3f translate;
    vec3f_copy(translate, &obj->rawData.asF32[posIndex]);
    Vec3s rotation;
    vec3i_to_vec3s(rotation,  &obj->rawData.asS32[angleIndex]);
    mtxf_rotate_zxy_and_translate(obj->transform, translate, rotation);
}

void obj_set_throw_matrix_from_transform(struct Object *obj) {
    if (obj->oFlags & OBJ_FLAG_UPDATE_TRANSFORM_FOR_THROW_MATRIX) {
        obj_build_transform_from_pos_and_angle(obj, O_POS_INDEX, O_FACE_ANGLE_INDEX);
        obj_apply_scale_to_transform(obj);
    }

    obj->header.gfx.throwMatrix = &obj->transform;

    obj_scale(obj, 1.0f);
}

void obj_build_transform_relative_to_parent(struct Object *obj) {
    struct Object *parent = obj->parentObj;

    obj_build_transform_from_pos_and_angle(obj, O_PARENT_RELATIVE_POS_INDEX, O_FACE_ANGLE_INDEX);
    obj_apply_scale_to_transform(obj);
    mtxf_mul(obj->transform, obj->transform, parent->transform);

    vec3f_copy(&obj->oPosVec, obj->transform[3]);

    obj->header.gfx.throwMatrix = &obj->transform;

    obj_scale(obj, 1.0f);
}

void obj_create_transform_from_self(struct Object *obj) {
    obj->oFlags &= ~OBJ_FLAG_TRANSFORM_RELATIVE_TO_PARENT;
    obj->oFlags |= OBJ_FLAG_SET_THROW_MATRIX_FROM_TRANSFORM;
    vec3f_copy(obj->transform[3], &obj->oPosVec);
}

void cur_obj_rotate_move_angle_using_vel(void) {
    vec3i_add(&o->oMoveAngleVec, &o->oAngleVelVec);
}

void cur_obj_rotate_face_angle_using_vel(void) {
    vec3i_add(&o->oFaceAngleVec, &o->oAngleVelVec);
}

void cur_obj_set_face_angle_to_move_angle(void) {
    vec3i_copy(&o->oFaceAngleVec, &o->oMoveAngleVec);
}

// removed unused arg: define allows for any number of args
s32 cur_obj_follow_path_new(void) {
    if (o->oPathedPrevWaypointFlags == 0) {
        o->oPathedPrevWaypoint = o->oPathedStartWaypoint;
        o->oPathedPrevWaypointFlags = WAYPOINT_FLAGS_INITIALIZED;
    }

    struct Waypoint *startWaypoint = o->oPathedStartWaypoint;
    struct Waypoint *lastWaypoint = o->oPathedPrevWaypoint;
    struct Waypoint *targetWaypoint;
    Vec3s prevToNext, objToNext;

    if ((lastWaypoint + 1)->flags != WAYPOINT_FLAGS_END) {
        targetWaypoint = lastWaypoint + 1;
    } else {
        targetWaypoint = startWaypoint;
    }

    o->oPathedPrevWaypointFlags = lastWaypoint->flags | WAYPOINT_FLAGS_INITIALIZED;

    vec3_diff(prevToNext, targetWaypoint->pos, lastWaypoint->pos);
    vec3_diff(objToNext, targetWaypoint->pos, &o->oPosVec);
    prevToNext[1] = 0;
    objToNext[1] = 0;

    o->oPathedTargetYaw = atan2s(objToNext[2], objToNext[0]);
    o->oPathedTargetPitch = atan2s(sqrtf(sqr(objToNext[0]) + sqr(objToNext[2])), -objToNext[1]);

    // If dot(prevToNext, objToNext) <= 0 (i.e. reached other side of target waypoint)
    if (vec3_dot(prevToNext, objToNext) <= 0.0f) {
        o->oPathedPrevWaypoint = targetWaypoint;
        if ((targetWaypoint + 1)->flags == WAYPOINT_FLAGS_END) {
            return PATH_REACHED_END;
        } else {
            return PATH_REACHED_WAYPOINT;
        }
    }

    return PATH_NONE;
}

void chain_segment_init(struct ChainSegment *segment) {
    vec3_zero(segment->pos);
    vec3_zero(segment->angle);
}

f32 random_f32_around_zero(f32 diameter) {
    return random_float() * diameter - diameter / 2;
}

void obj_scale_random(struct Object *obj, f32 rangeLength, f32 minScale) {
    f32 scale = random_float() * rangeLength + minScale;
    obj_scale(obj, scale);
}

void obj_translate_xyz_random(struct Object *obj, f32 rangeLength) {
    obj->oPosX += random_f32_around_zero(rangeLength);
    obj->oPosY += random_f32_around_zero(rangeLength);
    obj->oPosZ += random_f32_around_zero(rangeLength);
}

void obj_translate_xz_random(struct Object *obj, f32 rangeLength) {
    obj->oPosX += random_f32_around_zero(rangeLength);
    obj->oPosZ += random_f32_around_zero(rangeLength);
}

static void obj_build_vel_from_transform(struct Object *obj) {
    Vec3f vel = { 0.f, 0.f, obj->oForwardVel };

    linear_mtxf_mul_vec3f(obj->transform, &obj->oVelVec, vel);
}

void cur_obj_set_pos_via_transform(void) {
    obj_build_transform_from_pos_and_angle(o, O_PARENT_RELATIVE_POS_INDEX, O_MOVE_ANGLE_INDEX);
    obj_build_vel_from_transform(o);
    vec3f_add(&o->oPosVec, &o->oVelVec);
}

s32 cur_obj_reflect_move_angle_off_wall(void) {
    return (s16)(o->oWallAngle - ((s16) o->oMoveAngleYaw - (s16) o->oWallAngle) + 0x8000);
}

void cur_obj_spawn_particles(struct SpawnParticlesInfo *info) {
    struct Object *particle;
    s32 i;
    f32 scale;
    s32 numParticles = info->count;

    // If there are a lot of objects already, limit the number of particles
    if ((gPrevFrameObjectCount > (OBJECT_POOL_CAPACITY - 90)) && numParticles > 10) {
        numParticles = 10;
    }

    // We're close to running out of object slots, so don't spawn particles at
    // all
    // if (gPrevFrameObjectCount > (OBJECT_POOL_CAPACITY - 30)) {
    //     numParticles = 0;
    // }

    for (i = 0; i < numParticles; i++) {
        scale = random_float() * (info->sizeRange * 0.1f) + info->sizeBase * 0.1f;

        particle = spawn_object(o, info->model, bhvWhitePuffExplosion);

        particle->oBehParams2ndByte = info->behParam;
        particle->oMoveAngleYaw = random_u16();
        particle->oGravity = info->gravity;
        particle->oDragStrength = info->dragStrength;

        particle->oPosY += info->offsetY;
        particle->oForwardVel = random_float() * info->forwardVelRange + info->forwardVelBase;
        particle->oVelY = random_float() * info->velYRange + info->velYBase;

        obj_scale(particle, scale);
    }
}

void obj_set_hitbox(struct Object *obj, struct ObjectHitbox *hitbox) {
    if (!(obj->oFlags & OBJ_FLAG_HITBOX_WAS_SET)) {
        obj->oFlags |= OBJ_FLAG_HITBOX_WAS_SET;

        obj->oInteractType = hitbox->interactType;
        obj->oDamageOrCoinValue = hitbox->damageOrCoinValue;
        obj->oHealth = hitbox->health;
        obj->oNumLootCoins = hitbox->numLootCoins;

        cur_obj_become_tangible();
    }

    obj->hitboxRadius = obj->header.gfx.scale[0] * hitbox->radius;
    obj->hitboxHeight = obj->header.gfx.scale[1] * hitbox->height;
    obj->hurtboxRadius = obj->header.gfx.scale[0] * hitbox->hurtboxRadius;
    obj->hurtboxHeight = obj->header.gfx.scale[1] * hitbox->hurtboxHeight;
    obj->hitboxDownOffset = obj->header.gfx.scale[1] * hitbox->downOffset;
}

s32 cur_obj_wait_then_blink(s32 timeUntilBlinking, s32 numBlinks) {
    if (o->oTimer >= timeUntilBlinking) {
        s32 timeBlinking = o->oTimer - timeUntilBlinking;
        if (timeBlinking & 0x1) {
            o->header.gfx.node.flags |= GRAPH_RENDER_INVISIBLE;
            if (timeBlinking / 2 > numBlinks) {
                return TRUE;
            }
        } else {
            o->header.gfx.node.flags &= ~GRAPH_RENDER_INVISIBLE;
        }
    }

    return FALSE;
}

s32 cur_obj_is_mario_ground_pounding_platform(void) {
    if (gMarioObject->platform == o) {
        if (gMarioStates[0].action == ACT_GROUND_POUND_LAND) {
            return TRUE;
        }
    }

    return FALSE;
}

void spawn_mist_particles(void) {
    spawn_mist_particles_variable(0, 0, 46.0f);
}

void spawn_mist_particles_with_sound(u32 soundMagic) {
    spawn_mist_particles_variable(0, 0, 46.0f);
    create_sound_spawner(soundMagic);
}

void cur_obj_push_mario_away(f32 radius) {
    f32 marioRelX = gMarioObject->oPosX - o->oPosX;
    f32 marioRelZ = gMarioObject->oPosZ - o->oPosZ;
    f32 marioDist = sqr(marioRelX) + sqr(marioRelZ);

    if (marioDist < sqr(radius)) {
        marioDist = (radius - sqrtf(marioDist)) / radius;
        //! If this function pushes Mario out of bounds, it will trigger Mario's
        //  oob failsafe
        gMarioStates[0].pos[0] += marioDist * marioRelX;
        gMarioStates[0].pos[2] += marioDist * marioRelZ;
    }
}

void cur_obj_push_mario_away_from_cylinder(f32 radius, f32 extentY) {
    if (absf(gMarioObject->oPosY - o->oPosY) < extentY) {
        cur_obj_push_mario_away(radius);
    }
}

void bhv_dust_smoke_loop(void) {
    vec3f_add(&o->oPosVec, &o->oVelVec);

    if (o->oSmokeTimer == 10) {
        obj_mark_for_deletion(o);
    }

    o->oSmokeTimer++;
}

s32 cur_obj_set_direction_table(s8 *pattern) {
    o->oToxBoxMovementPattern = pattern;
    o->oToxBoxMovementStep = 0;

    return *(s8 *) o->oToxBoxMovementPattern;
}

s32 cur_obj_progress_direction_table(void) {
    s8 action;
    s8 *pattern  = o->oToxBoxMovementPattern;
    s32 nextStep = o->oToxBoxMovementStep + 1;

    if (pattern[nextStep] != -1) {
        action = pattern[nextStep];
        o->oToxBoxMovementStep++;
    } else {
        action = pattern[0];
        o->oToxBoxMovementStep = 0;
    }

    return action;
}

void cur_obj_scale_over_time(s32 axis, s32 times, f32 start, f32 end) {
    f32 scale = ((end - start) * ((f32) o->oTimer / times)) + start;

    if (axis & SCALE_AXIS_X) {
        o->header.gfx.scale[0] = scale;
    }

    if (axis & SCALE_AXIS_Y) {
        o->header.gfx.scale[1] = scale;
    }

    if (axis & SCALE_AXIS_Z) {
        o->header.gfx.scale[2] = scale;
    }
}

void cur_obj_set_pos_to_home_with_debug(void) {
    vec3_sum(&o->oPosVec, &o->oHomeVec, gDebugInfo[DEBUG_PAGE_ENEMYINFO]);

    cur_obj_scale(gDebugInfo[DEBUG_PAGE_ENEMYINFO][3] / 100.0f + 1.0f);
}

s32 cur_obj_is_mario_on_platform(void) {
    return gMarioObject->platform == o;
}

s32 cur_obj_shake_y_until(s32 cycles, s32 amount) {
    if (o->oTimer & 0x1) {
        o->oPosY -= amount;
        // Return FALSE since o->oTimer can't be equal to (cycles * 2) if it is odd.
        return FALSE;
    } else {
        o->oPosY += amount;
    }

    return o->oTimer == cycles * 2;
}

void cur_obj_call_action_function(ObjActionFunc actionFunctions[]) {
    ObjActionFunc actionFunction = actionFunctions[o->oAction];
    actionFunction();
}

s32 cur_obj_mario_far_away(void) {
    Vec3f d;
    vec3f_diff(d, &o->oHomeVec, &gMarioObject->oPosVec);

    return o->oDistanceToMario > 2000.0f && vec3_sumsq(d) > sqr(2000.0f);
}

s32 is_mario_moving_fast_or_in_air(s32 speedThreshold) {
    return (
        gMarioState->forwardVel > speedThreshold
        || (gMarioState->action & ACT_FLAG_AIR)
    );
}

s32 is_item_in_array(s8 item, s8 *array) {
    while (*array != -1) {
        if (*array == item) {
            return TRUE;
        }

        array++;
    }

    return FALSE;
}

s32 cur_obj_set_hitbox_and_die_if_attacked(struct ObjectHitbox *hitbox, s32 deathSound, s32 noLootCoins) {
    s32 interacted = FALSE;

    if (hitbox != NULL) {
        obj_set_hitbox(o, hitbox);
        }

    if (noLootCoins) {
        o->oNumLootCoins = 0;
    }

    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        if (o->oInteractStatus & INT_STATUS_WAS_ATTACKED) {
            if ((o->oHealth < 2) || (o->oInteractStatus & INT_STATUS_TOUCHED_BOB_OMB)) {
                if (o->oNumLootCoins > 0) {
                    gMarioState->DeadRexes ++;
                }
                spawn_mist_particles();
                obj_spawn_loot_yellow_coins(o, o->oNumLootCoins, 20.0f);
                //gMarioState->EA_LEFT --;
                //gMarioState->EA_ACTIVE --;
                obj_mark_for_deletion(o);
                create_sound_spawner(deathSound);
            } else {
                o->oDmgFade = 255;
                if (save_file_get_badge_equip() & (1<<3)) {
                    o->oHealth-=2;
                } else {
                    o->oHealth--;
                }
            }
        } else {
            interacted = TRUE;
        }
    }

    o->oInteractStatus = INT_STATUS_NONE;
    return interacted;
}


void obj_explode_and_spawn_coins(f32 mistSize, s32 coinType) {
    spawn_mist_particles_variable(0, 0, mistSize);
    spawn_triangle_break_particles(30, MODEL_DIRT_ANIMATION, 3.0f, TINY_DIRT_PARTICLE_ANIM_STATE_YELLOW);
    obj_mark_for_deletion(o);

    if (coinType == COIN_TYPE_YELLOW) {
        obj_spawn_loot_yellow_coins(o, o->oNumLootCoins, 20.0f);
    } else if (coinType == COIN_TYPE_BLUE) {
        obj_spawn_loot_blue_coins(o, o->oNumLootCoins, 20.0f, 150);
    }
}

void obj_set_collision_data(struct Object *obj, const void *segAddr) {
    obj->collisionData = segmented_to_virtual(segAddr);
}

void cur_obj_if_hit_wall_bounce_away(void) {
    if (o->oMoveFlags & OBJ_MOVE_HIT_WALL) {
        o->oMoveAngleYaw = o->oWallAngle;
    }
}

s32 cur_obj_hide_if_mario_far_away_y(f32 distY) {
    if (absf(o->oPosY - gMarioObject->oPosY) < distY) {
        cur_obj_unhide();
        return FALSE;
    } else {
        cur_obj_hide();
        return TRUE;
    }
}

Gfx *geo_offset_klepto_held_object(s32 callContext, struct GraphNode *node, UNUSED Mat4 mtx) {
    if (callContext == GEO_CONTEXT_RENDER) {
        vec3s_set(((struct GraphNodeTranslationRotation *) node->next)->translation, 300, 300, 0);
    }

    return NULL;
}

s32 obj_is_hidden(struct Object *obj) {
    if (obj->header.gfx.node.flags & GRAPH_RENDER_INVISIBLE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

void enable_time_stop(void) {
    gTimeStopState |= TIME_STOP_ENABLED;
}

void disable_time_stop(void) {
    gTimeStopState &= ~TIME_STOP_ENABLED;
}

void set_time_stop_flags(s32 flags) {
    gTimeStopState |= flags;
}

void clear_time_stop_flags(s32 flags) {
    gTimeStopState = gTimeStopState & (flags ^ 0xFFFFFFFF);
}

s32 cur_obj_can_mario_activate_textbox(f32 radius, f32 height, UNUSED s32 unused) {
    return (
        o->oDistanceToMario < 1500.0f
        && o->oPosY < gMarioObject->oPosY + 160.0f
        && gMarioObject->oPosY < o->oPosY + height
        && !(gMarioStates[0].action & ACT_FLAG_AIR)
        && lateral_dist_between_objects(o, gMarioObject) < radius
        && mario_ready_to_speak()
    );
}

s32 cur_obj_can_mario_activate_textbox_2(f32 radius, f32 height) {
    // The last argument here is unused. When this function is called directly the argument is always set to 0x7FFF.
    return cur_obj_can_mario_activate_textbox(radius, height, 0x1000);
}

static void cur_obj_end_dialog(s32 dialogFlags, s32 dialogResult) {
    o->oDialogResponse = dialogResult;
    o->oDialogState++;

    if (!(dialogFlags & DIALOG_FLAG_TIME_STOP_ENABLED)) {
        set_mario_npc_dialog(MARIO_DIALOG_STOP);
    }
}

s32 cur_obj_update_dialog(s32 actionArg, s32 dialogFlags, s32 dialogID, UNUSED s32 unused) {
    s32 dialogResponse = DIALOG_RESPONSE_NONE;

    switch (o->oDialogState) {
#ifdef BUGFIX_DIALOG_TIME_STOP
        case DIALOG_STATUS_ENABLE_TIME_STOP:
            // Patched :(
            // Wait for Mario to be ready to speak, and then enable time stop
            if (mario_ready_to_speak() || gMarioState->action == ACT_READING_NPC_DIALOG) {
                gTimeStopState |= TIME_STOP_ENABLED;
                o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
                o->oDialogState++;
            } else {
                break;
            }
            // Fall through so that Mario's action is interrupted immediately
            // after time is stopped
            FALL_THROUGH;
#else
        case DIALOG_STATUS_ENABLE_TIME_STOP:
            //! We enable time stop even if Mario is not ready to speak. This
            //  allows us to move during time stop as long as Mario never enters
            //  an action that can be interrupted with text.
            if (gMarioState->health >= 0x100) {
                gTimeStopState |= TIME_STOP_ENABLED;
                o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
                o->oDialogState++;
            }
            break;
#endif
        case DIALOG_STATUS_INTERRUPT:
            // Interrupt until Mario is actually speaking with the NPC
            if (set_mario_npc_dialog(actionArg) == MARIO_DIALOG_STATUS_SPEAK) {
                o->oDialogState++;
            }
            break;

        case DIALOG_STATUS_START_DIALOG:
            // Starts dialog, depending of the flag defined, it calls
            // a default dialog or a dialog with response.
            if (dialogFlags & DIALOG_FLAG_TEXT_RESPONSE) {
                create_dialog_box_with_response(dialogID);
            } else if (dialogFlags & DIALOG_FLAG_TEXT_DEFAULT) {
                create_dialog_box(dialogID);
            }
            o->oDialogState++;
            break;

        case DIALOG_STATUS_STOP_DIALOG:
            // Stops dialog, if the flag dialog response was called
            // then it defines the value to let the object do the rest.
            if (dialogFlags & DIALOG_FLAG_TEXT_RESPONSE) {
                if (gDialogResponse != DIALOG_RESPONSE_NONE) {
                    cur_obj_end_dialog(dialogFlags, gDialogResponse);
                }
            } else if (dialogFlags & DIALOG_FLAG_TEXT_DEFAULT) {
                if (get_dialog_id() == DIALOG_NONE) {
                    cur_obj_end_dialog(dialogFlags, DIALOG_RESPONSE_NOT_DEFINED);
                }
            } else {
                cur_obj_end_dialog(dialogFlags, DIALOG_RESPONSE_NOT_DEFINED);
            }
            break;

        case DIALOG_STATUS_DISABLE_TIME_STOP:
            // We disable time stop for a few seconds when Mario is no longer
            // speaking or the flag is defined, then we enable it again.
            // Usually, an object disables time stop using a separate function
            // after a certain condition is met.
            if (gMarioState->action != ACT_READING_NPC_DIALOG || (dialogFlags & DIALOG_FLAG_TIME_STOP_ENABLED)) {
                gTimeStopState &= ~TIME_STOP_ENABLED;
                o->activeFlags &= ~ACTIVE_FLAG_INITIATED_TIME_STOP;
                dialogResponse = o->oDialogResponse;
                o->oDialogState = DIALOG_STATUS_ENABLE_TIME_STOP;
            }
            break;

        default:
            o->oDialogState = DIALOG_STATUS_ENABLE_TIME_STOP;
            break;
    }

    return dialogResponse;
}

s32 cur_obj_update_dialog_with_cutscene(s32 actionArg, s32 dialogFlags, s32 cutsceneTable, s32 dialogID) {
    s32 dialogResponse = DIALOG_RESPONSE_NONE;
    s32 doneTurning = TRUE;

    switch (o->oDialogState) {
#ifdef BUGFIX_DIALOG_TIME_STOP
        case DIALOG_STATUS_ENABLE_TIME_STOP:
            // Wait for Mario to be ready to speak, and then enable time stop
            if (mario_ready_to_speak() || gMarioState->action == ACT_READING_NPC_DIALOG) {
                gTimeStopState |= TIME_STOP_ENABLED;
                o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
                o->oDialogState++;
                o->oDialogResponse = DIALOG_RESPONSE_NONE;
            } else {
                break;
            }
            // Fall through so that Mario's action is interrupted immediately
            // after time is stopped
            FALL_THROUGH;
#else
        case DIALOG_STATUS_ENABLE_TIME_STOP:
            //! We enable time stop even if Mario is not ready to speak. This
            //  allows us to move during time stop as long as Mario never enters
            //  an action that can be interrupted with text.
            if (gMarioState->health >= 0x0100) {
                gTimeStopState |= TIME_STOP_ENABLED;
                o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
                o->oDialogState++;
                o->oDialogResponse = DIALOG_RESPONSE_NONE;
            }
            break;
#endif
        case DIALOG_STATUS_INTERRUPT:
            // Additional flag that makes the NPC rotate towards to Mario
            if (dialogFlags & DIALOG_FLAG_TURN_TO_MARIO) {
                doneTurning = cur_obj_rotate_yaw_toward(obj_angle_to_object(o, gMarioObject), 0x800);
                // Failsafe just in case it takes more than 33 frames somehow
                if (o->oDialogResponse > DIALOG_RESPONSE_MAXIMUM) {
                    doneTurning = TRUE;
                }
            }
            // Interrupt status until Mario is actually speaking with the NPC and if the
            // object is done turning to Mario
            if (set_mario_npc_dialog(actionArg) == MARIO_DIALOG_STATUS_SPEAK && doneTurning) {
                o->oDialogResponse = DIALOG_RESPONSE_NONE;
                o->oDialogState++;
            } else {
                o->oDialogResponse++; // treated as a timer for the failsafe
            }
            break;

        case DIALOG_STATUS_START_DIALOG:
            // Special check for Cap Switch cutscene since the cutscene itself
            // handles what dialog should use
            if (cutsceneTable == CUTSCENE_CAP_SWITCH_PRESS) {
                if ((o->oDialogResponse = cutscene_object_without_dialog(cutsceneTable, o))) {
                    o->oDialogState++;
                }
            } else {
                // General dialog cutscene function, most of the time
                // the "CUTSCENE_DIALOG" cutscene is called
                if ((o->oDialogResponse = cutscene_object_with_dialog(cutsceneTable, o, dialogID))) {
                    o->oDialogState++;
                }
            }
            break;

        case DIALOG_STATUS_STOP_DIALOG:
            // If flag defined, keep time stop enabled until the object
            // decided to disable it independently
            if (dialogFlags & DIALOG_FLAG_TIME_STOP_ENABLED) {
                dialogResponse = o->oDialogResponse;
                o->oDialogState = DIALOG_STATUS_ENABLE_TIME_STOP;
            } else if (gMarioState->action != ACT_READING_NPC_DIALOG) {
                // Disable time stop, then enable time stop for a frame
                // until the set_mario_npc_dialog function disables it
                gTimeStopState &= ~TIME_STOP_ENABLED;
                o->activeFlags &= ~ACTIVE_FLAG_INITIATED_TIME_STOP;
                dialogResponse = o->oDialogResponse;
                o->oDialogState = DIALOG_STATUS_ENABLE_TIME_STOP;
            } else {
                // And finally stop Mario dialog status
                set_mario_npc_dialog(MARIO_DIALOG_STOP);
            }
            break;
    }

    return dialogResponse;
}


void cur_obj_align_gfx_with_floor(void) {
    struct Surface *floor;
    Vec3f position;

    vec3f_copy(position, &o->oPosVec);

    find_floor(position[0], position[1], position[2], &floor);
    if (floor != NULL) {
        Vec3f floorNormal;
        get_surface_normal(floorNormal, floor);

        mtxf_align_terrain_normal(o->transform, floorNormal, position, o->oFaceAngleYaw);
        o->header.gfx.throwMatrix = &o->transform;
    }
}

//
UNUSED s32 mario_is_within_rectangle(s16 minX, s16 maxX, s16 minZ, s16 maxZ) {
    if (gMarioObject->oPosX < minX || maxX < gMarioObject->oPosX) {
        return FALSE;
    }

    if (gMarioObject->oPosZ < minZ || maxZ < gMarioObject->oPosZ) {
        return FALSE;
    }

    return TRUE;
}

void cur_obj_shake_screen(s32 shake) {
    set_camera_shake_from_point(shake, o->oPosX, o->oPosY, o->oPosZ);
}

s32 obj_attack_collided_from_other_object(struct Object *obj) {
    if (obj->numCollidedObjs != 0) {
        struct Object *other = obj->collidedObjs[0];

        if (other != gMarioObject) {
            switch (other->oInteractType) {
                case INTERACT_BULLY:
                case INTERACT_HIT_FROM_BELOW:
                case INTERACT_BOUNCE_TOP:
                case INTERACT_BREAKABLE:
                    break;
                case INTERACT_GRABBABLE:
                    if (other->behavior != segmented_to_virtual(bhvBobomb) &&
                        other->behavior != segmented_to_virtual(bhvBowser)) {
                        return FALSE;
                    }
                    break;
                default:
                    return FALSE;
            }
            other->oInteractStatus |= INT_STATUS_WAS_ATTACKED | INT_STATUS_INTERACTED
                                      | INT_STATUS_TOUCHED_BOB_OMB + ATTACK_FAST_ATTACK;
            return TRUE;
        }
    }

    return FALSE;
}

s32 cur_obj_was_attacked_or_ground_pounded(void) {
    s32 attacked = FALSE;

    if ((o->oInteractStatus & INT_STATUS_INTERACTED)
        && (o->oInteractStatus & INT_STATUS_WAS_ATTACKED)) {
        attacked = TRUE;
    }

    if (cur_obj_is_mario_ground_pounding_platform()) {
        attacked = TRUE;
    }

    o->oInteractStatus = INT_STATUS_NONE;
    return attacked;
}

void obj_copy_behavior_params(struct Object *dst, struct Object *src) {
    dst->oBehParams = src->oBehParams;
    dst->oBehParams2ndByte = src->oBehParams2ndByte;
}

void cur_obj_init_animation_and_anim_frame(s32 animIndex, s32 animFrame) {
    cur_obj_init_animation_with_sound(animIndex);
    o->header.gfx.animInfo.animFrame = animFrame;
}

s32 cur_obj_init_animation_and_check_if_near_end(s32 animIndex) {
    cur_obj_init_animation_with_sound(animIndex);
    return cur_obj_check_if_near_animation_end();
}

void cur_obj_init_animation_and_extend_if_at_end(s32 animIndex) {
    cur_obj_init_animation_with_sound(animIndex);
    cur_obj_extend_animation_if_at_end();
}

s32 cur_obj_check_grabbed_mario(void) {
    if (o->oInteractStatus & INT_STATUS_GRABBED_MARIO) {
        o->oKingBobombHoldingMarioState = HELD_HELD;
        cur_obj_become_intangible();
        return TRUE;
    }

    return FALSE;
}

s32 sPlayerGrabReleaseState;

s32 player_performed_grab_escape_action(void) {
    if (gPlayer1Controller->stickMag < 30.0f) {
        sPlayerGrabReleaseState = FALSE;
    }

    if (sPlayerGrabReleaseState && (gPlayer1Controller->stickMag > 40.0f)) {
        sPlayerGrabReleaseState = TRUE;
        return TRUE;
    }

    if (gPlayer1Controller->buttonPressed & (A_BUTTON | B_BUTTON | Z_TRIG)) {
        return TRUE;
    }

    return FALSE;
}

void cur_obj_unused_play_footstep_sound(s32 animFrame1, s32 animFrame2, s32 sound) {
    if (cur_obj_check_anim_frame(animFrame1) || cur_obj_check_anim_frame(animFrame2)) {
        cur_obj_play_sound_2(sound);
    }
}

void enable_time_stop_including_mario(void) {
    gTimeStopState |= TIME_STOP_ENABLED | TIME_STOP_MARIO_AND_DOORS;
    o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
}

void disable_time_stop_including_mario(void) {
    gTimeStopState &= ~(TIME_STOP_ENABLED | TIME_STOP_MARIO_AND_DOORS);
    o->activeFlags &= ~ACTIVE_FLAG_INITIATED_TIME_STOP;
}

s32 cur_obj_check_interacted(void) {
    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        o->oInteractStatus = INT_STATUS_NONE;
        return TRUE;
    } else {
        return FALSE;
    }
}

void cur_obj_spawn_loot_blue_coin(void) {
    if (o->oNumLootCoins >= 5) {
        spawn_object(o, MODEL_BLUE_COIN, bhvMrIBlueCoin);
        o->oNumLootCoins -= 5;
    }
}

void cur_obj_spawn_star_at_y_offset(f32 targetX, f32 targetY, f32 targetZ, f32 offsetY) {
    f32 objectPosY = o->oPosY;
    o->oPosY += offsetY + gDebugInfo[DEBUG_PAGE_ENEMYINFO][0];
    spawn_default_star(targetX, targetY, targetZ);
    o->oPosY = objectPosY;
}

struct Surface * cur_obj_get_interact_floor(u8 move_standard_or_object_step) {
    struct Surface * floor = o->oFloor;
    u8 move_condition = (o->oMoveFlags & OBJ_MOVE_MASK_ON_GROUND);
    if (move_standard_or_object_step) {
        //using object_step instead of move standard
        floor = sObjFloor;
        move_condition = (o->oFloorHeight + .1f > o->oPosY);
    }

    if ((floor != NULL) && (move_condition)) {
        return floor;
    }
    return NULL;
}

void arbritrary_death_coin_release(void) {
    // If toby fox can get away with the entire undertale dialog system being stored in a single switch statement, i can get
    // away with one teeny hardcoded elseif
    if (cur_obj_has_behavior(bhvMotos)) {
        struct Object * coin = spawn_object(o, MODEL_BLUE_COIN, bhvBlueCoinMotos);
        cur_obj_play_sound_2(SOUND_GENERAL_COIN_SPURT);
        coin->oForwardVel = 10.0f;
        coin->oVelY = 20.0f;
        coin->oMoveAngleYaw = (f32)(o->oFaceAngleYaw + 0x8000) + random_float() * 1024.0f;

        // drop mario if he's held
        if (o->prevObj) {
            o->prevObj = NULL;
            o->oInteractStatus &= ~INT_STATUS_GRABBED_MARIO;
            gMarioObject->oInteractStatus |= INT_STATUS_MARIO_THROWN_BY_OBJ | INT_STATUS_MARIO_DROPPED_BY_OBJ;
        }
    } else if (cur_obj_has_behavior(bhvChuckya)) {
        // drop mario if he's held
        if (o->prevObj) {
            o->prevObj = NULL;
            o->oInteractStatus &= ~INT_STATUS_GRABBED_MARIO;
            gMarioObject->oInteractStatus |= INT_STATUS_MARIO_THROWN_BY_OBJ | INT_STATUS_MARIO_DROPPED_BY_OBJ;
            obj_spawn_loot_yellow_coins(o, o->oNumLootCoins, 20.0f);
        }
    } else if (cur_obj_has_behavior(bhvBobomb)) {
        obj_spawn_yellow_coins(o, 1);
    } else if (cur_obj_has_behavior(bhvSmallBully)) {
        obj_spawn_yellow_coins(o, 1);
    } else if (cur_obj_has_behavior(bhvScaredKoopa)) {
        struct Object * coin = spawn_object(o, MODEL_BLUE_COIN, bhvBlueCoinMotos);
        cur_obj_play_sound_2(SOUND_GENERAL_COIN_SPURT);
        coin->oForwardVel = 10.0f;
        coin->oVelY = 20.0f;
        coin->oMoveAngleYaw = (f32)(o->oFaceAngleYaw + 0x8000) + random_float() * 1024.0f;
    } else if (cur_obj_has_behavior(bhvBigBully)) {
        cur_obj_drop_imbued_object(400);
    } else if (cur_obj_has_behavior(bhvMoneybag)||cur_obj_has_behavior(bhvMoneybagHidden)) {
        obj_spawn_yellow_coins(o, 3);
    } else {
        // default
        obj_spawn_loot_yellow_coins(o, o->oNumLootCoins, 20.0f);
    }
}

struct Surface *sInteractFloor;

s32 is_obj_interacting_with_noteblock(u8 move_standard_or_object_step) {
    struct Surface * floor = cur_obj_get_interact_floor(move_standard_or_object_step);

    if ((floor) && (floor->object != NULL) && obj_has_behavior(floor->object,bhvNoteblock)) {
        return TRUE; // adachi true
    }
    return FALSE; // lightning gif
}

void cur_obj_interact_with_noteblock(void) {
    if ((sInteractFloor->object != NULL) && obj_has_behavior(sInteractFloor->object,bhvNoteblock)) {
        struct Object * noteblock_interacting = sInteractFloor->object;
        o->oVelY = 95.0f;

        noteblock_interacting->oTimer = 0;
        noteblock_interacting->oVelY = 50.0f;
        cur_obj_play_sound_2(SOUND_GENERAL_CRAZY_BOX_BOING_SLOW);
    }
}

void cur_obj_interact_with_floor_switch(void) {
    if ((o->oFlags & OBJ_FLAG_ACTIVATES_FLOOR_SWITCH) && (sInteractFloor->object != NULL) && obj_has_behavior(sInteractFloor->object,bhvFloorSwitchHiddenObjects)) {
        struct Object * switch_interacting = sInteractFloor->object;

        if (switch_interacting->oAction == PURPLE_SWITCH_ACT_IDLE) {
            switch_interacting->oAction = PURPLE_SWITCH_ACT_PRESSED;
        }
    }

    if ((o->oFlags & OBJ_FLAG_ACTIVATES_FLOOR_SWITCH) && (sInteractFloor->object != NULL) && obj_has_behavior(sInteractFloor->object,bhvOnOffButton)) {
        struct Object * switch_interacting = sInteractFloor->object;

        if ((switch_interacting->oAction == 1)&&(switch_interacting->oTimer > 30)) {
            cmm_play_onoff = switch_interacting->oBehParams2ndByte;
            play_sound(SOUND_GENERAL2_BUTTON_PRESS, gGlobalSoundSource);
        }
    }
}

void cur_obj_interact_with_quicksand(void) {
    if (o->oFlags & OBJ_FLAG_IMMUNE_TO_FLOOR_DEATH) {
        return;
    }

    if ((sInteractFloor->type == SURFACE_INSTANT_QUICKSAND || sInteractFloor->type == SURFACE_DEEP_QUICKSAND)) {
        // drag is applied in the object step function
        if (o->oQuicksandDepthToDie != 0) {
            //cur_obj_play_sound_1(SOUND_MOVING_QUICKSAND_DEATH);
            o->oQuicksandDepth ++;
            o->oVelY = 0.0f;
            u8 die_condition = (o->oQuicksandDepth == o->oQuicksandDepthToDie);
            if (o->oQuicksandDepthToDie == 255) {
                // Most objects tend to visually be double their hitbox hight
                // If not, just set quicksanddepthtodie manually
                die_condition = (o->oQuicksandDepth > o->hitboxHeight);
            }
            if (die_condition) {
                arbritrary_death_coin_release();
                mark_obj_for_deletion(o);
            }
        } else {
            // non sinking objects can sink a teeny bit
            if (o->oQuicksandDepth < 15) {
                o->oQuicksandDepth++;
            }
        }

        if ((sInteractFloor->type == SURFACE_DEEP_QUICKSAND)&&(o->oQuicksandDepth > 15)) {
            o->oQuicksandDepth=15;
        }
    } else {
        if (o->oQuicksandDepth-10<=0) {
            o->oQuicksandDepth = 0;
        } else {
            o->oQuicksandDepth -= 10;
        }
        // get out of the quicksand
    }
}

void cur_obj_interact_with_lava(u8 move_standard_or_object_step) {
    struct Surface * floor = cur_obj_get_interact_floor(move_standard_or_object_step);

    if ((floor) && SURFACE_IS_BURNING(floor->type)) {
        // drag is applied in the object step function
        arbritrary_death_coin_release();
        mark_obj_for_deletion(o);
    }
}

s32 is_cur_obj_interact_with_lava(u8 move_standard_or_object_step) {
    struct Surface * floor = cur_obj_get_interact_floor(move_standard_or_object_step);

    if ((floor) && SURFACE_IS_BURNING(floor->type)) {
        // drag is applied in the object step function
        return TRUE;
    }
    return FALSE;
}

void cur_obj_interact_with_moving_platform(void) {
    o->oPosX += sInteractFloor->object->oDisplaceVec[0];
    o->oPosZ += sInteractFloor->object->oDisplaceVec[2];

    if (sInteractFloor->type == SURFACE_CONVEYOR) {
        struct Object * conveyor_interacting = sInteractFloor->object;

        s16 currentAngle = conveyor_interacting->oFaceAngleYaw;
        f32 perpendicularDistance = (o->oPosX - conveyor_interacting->oPosX) * coss(currentAngle) - (o->oPosZ - conveyor_interacting->oPosZ) * sins(currentAngle);
		// If too close to edge, tilt angle a little
		if (perpendicularDistance < -80.0f) {
			currentAngle += 0x2000;
		} else if (perpendicularDistance > 80.0f) {
			currentAngle -= 0x2000;
		}
        o->oPosX += sins(currentAngle) * 10.76f;
        o->oPosZ += coss(currentAngle) * 10.76f;
    }
}

void cur_obj_floor_interactions(u8 move_standard_or_object_step) {
    sInteractFloor = cur_obj_get_interact_floor(move_standard_or_object_step);
    if (!sInteractFloor) {
        return;
    }

    cur_obj_interact_with_quicksand();
    if (sInteractFloor->object == NULL) return;
    cur_obj_interact_with_noteblock();
    cur_obj_interact_with_floor_switch();
    cur_obj_interact_with_moving_platform();
}

void cur_obj_set_home_if_safe(void) {
    if (!o->oFloor) return;
    if (o->oFloorHeight < o->oPosY - 10.f) return;
    if (SURFACE_IS_UNSAFE(o->oFloorType)) return;
    if (o->oFloor->object != NULL) return;
    if (o->oImbue == IMBUE_STAR) return;

    vec3f_copy(&o->oHomeVec,&o->oPosVec);
}

s32 cur_obj_die_if_on_death_barrier(s32 offset) {
    if (o->oFloorType == SURFACE_DEATH_PLANE && o->oPosY < o->oFloorHeight + 100.f) {
        cur_obj_drop_imbued_object(offset);
        return TRUE;
    }
    return FALSE;
}

void spawn_mist_at_obj(struct Object *obj) {
    Vec3f oldPos;
    vec3f_copy(oldPos, &o->oPosVec);
    vec3f_copy(&o->oPosVec, &obj->oPosVec);
    spawn_mist_particles();
    vec3f_copy(&o->oPosVec, oldPos);
}

void cur_obj_drop_imbued_object(s32 y_offset) {
    struct Object * dropobj;
    if (o->oImbue == IMBUE_NONE) return;
    if (o->oImbue == IMBUE_STAR) {
        struct Surface *ptr;
        f32 ceilY = find_ceil(o->oHomeX,o->oHomeY,o->oHomeZ,&ptr);
        spawn_default_star(o->oHomeX,MIN(o->oHomeY+y_offset, ceilY - 75.f),o->oHomeZ);
        return;
    }

    switch(o->oImbue) {
        case IMBUE_BLUE_COIN:
            cur_obj_play_sound_2(SOUND_GENERAL_COIN_SPURT);
            dropobj = spawn_object(o, MODEL_BLUE_COIN, bhvBlueCoinMotos);
            vec3f_copy(&dropobj->oPosVec,&o->oPosVec);
            dropobj->oForwardVel = 10.0f;
            dropobj->oVelY = 20.0f;
            s16 angle = obj_angle_to_object(o, gMarioObject);
            dropobj->oMoveAngleYaw = angle + random_float() * 1024.0f;
            break;
        case IMBUE_THREE_COINS:
            obj_spawn_yellow_coins(o, 3);
            break;
        case IMBUE_RED_SWITCH:
        case IMBUE_BLUE_SWITCH:;
            struct Surface *floor;
            f32 floorY = find_floor(o->oHomeX,o->oHomeY+10.f,o->oHomeZ,&floor);
            // Abort spawn if over death barrier
            if (floor && floor->type != SURFACE_DEATH_PLANE) {
                dropobj = spawn_object(o,MODEL_MAKER_BUTTON,bhvOnOffButton);
                if (o->oImbue == IMBUE_BLUE_SWITCH) dropobj->oBehParams2ndByte = 1;
                vec3f_copy(&dropobj->oPosVec,&o->oHomeVec);
                dropobj->oPosY = floorY;
                dropobj->oMoveAngleYaw = 0;
                spawn_mist_at_obj(dropobj);
            }
            break;
        case IMBUE_RED_COIN:
            dropobj = spawn_object(o,MODEL_RED_COIN,bhvRedCoin);
            vec3f_copy(&dropobj->oPosVec,&o->oHomeVec);
            spawn_mist_at_obj(dropobj);
            break;
    }
    o->oImbue = IMBUE_NONE;
}


#ifdef ACCELERATED_COLLISION_LOOKUP
s32 objects_near_to_collision(struct Object *a, struct Object *b, u8 cellX, u8 cellY, u8 cellZ, void* ctx) {
    if (!b || a == b) {
        return FALSE;
    }
    f32 sqrLateralDist;
    vec3f_get_lateral_dist_squared(&a->oPosVec, &b->oPosVec, &sqrLateralDist);
    return sqrLateralDist < sqr(a->oCollisionDistance);
}
#endif // ACCELERATED_COLLISION_LOOKUP

struct Object * physics_object_list_head = NULL;
struct Object * physics_object_list_tail = NULL;

s32 obj_with_physics_is_near(void) {
    if (physics_object_list_head == NULL) {return FALSE;}
#ifdef ACCELERATED_COLLISION_LOOKUP
    return iterate_nearby_object_cells(o, &objects_near_to_collision, NULL);
#else
    struct Object * obj = physics_object_list_head;
    while (obj != NULL) {
        if (obj == o || obj == o->prevObj) {obj = obj->nextPhysicsObj; continue;}
        f32 sqrLateralDist;
        vec3f_get_lateral_dist_squared(&o->oPosVec, &obj->oPosVec, &sqrLateralDist);
        if (sqrLateralDist < sqr(o->oCollisionDistance)) {
            return TRUE;
        }
        obj = obj->nextPhysicsObj;
    }

    return FALSE;
#endif // ACCELERATED_COLLISION_LOOKUP
}

void obj_add_self_to_physics_list(void) {
    if (!o->oHasPhysics) {
        o->oHasPhysics = TRUE;
        if (physics_object_list_head) {
            physics_object_list_tail->nextPhysicsObj = o;
            physics_object_list_tail = o;
        } else {
            physics_object_list_head = o;
            physics_object_list_tail = o;
        }
    }
}

