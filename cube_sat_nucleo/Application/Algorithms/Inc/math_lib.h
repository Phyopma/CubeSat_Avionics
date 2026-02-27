#ifndef MATH_LIB_H
#define MATH_LIB_H

#include <math.h>

typedef struct {
    float x, y, z;
} vec3_t;

typedef struct {
    float w, x, y, z;
} quat_t;

// Vector Operations
float Vec3_Dot(vec3_t a, vec3_t b);
vec3_t Vec3_Cross(vec3_t a, vec3_t b);
float Vec3_Norm(vec3_t a);
vec3_t Vec3_Normalize(vec3_t a);
vec3_t Vec3_Add(vec3_t a, vec3_t b);
vec3_t Vec3_Sub(vec3_t a, vec3_t b);
vec3_t Vec3_ScalarMult(vec3_t a, float s);

// Quaternion Operations
quat_t Quat_Mult(quat_t q1, quat_t q2);
quat_t Quat_Inverse(quat_t q);
vec3_t Quat_RotateVec(quat_t q, vec3_t v);

#endif // MATH_LIB_H
