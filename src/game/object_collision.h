#ifndef OBJECT_COLLISION_H
#define OBJECT_COLLISION_H

void detect_object_collisions(void);

#ifdef ACCELERATED_COLLISION_LOOKUP
s32 iterate_nearby_object_cells(struct Object *a, s32 func(struct Object *, struct Object *, u8 cellX, u8 cellY, u8 cellZ, void* ctx), void* ctx);
#endif // ACCELERATED_COLLISION_LOOKUP

#endif // OBJECT_COLLISION_H
