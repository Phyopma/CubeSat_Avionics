#include "math_lib.h"

float Vec3_Dot(vec3_t a, vec3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3_t Vec3_Cross(vec3_t a, vec3_t b) {
    return (vec3_t){
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x
    };
}

float Vec3_Norm(vec3_t a) {
    return sqrtf(Vec3_Dot(a, a));
}

vec3_t Vec3_Normalize(vec3_t a) {
    float n = Vec3_Norm(a);
    if (n < 1e-6f) return (vec3_t){0, 0, 0};
    return Vec3_ScalarMult(a, 1.0f / n);
}

vec3_t Vec3_Add(vec3_t a, vec3_t b) {
    return (vec3_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

vec3_t Vec3_Sub(vec3_t a, vec3_t b) {
    return (vec3_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

vec3_t Vec3_ScalarMult(vec3_t a, float s) {
    return (vec3_t){a.x * s, a.y * s, a.z * s};
}

quat_t Quat_Mult(quat_t q1, quat_t q2) {
    return (quat_t){
        .w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        .x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        .y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        .z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    };
}

quat_t Quat_Inverse(quat_t q) {
    float n2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    return (quat_t){q.w / n2, -q.x / n2, -q.y / n2, -q.z / n2};
}

vec3_t Quat_RotateVec(quat_t q, vec3_t v) {
    quat_t v_quat = {0, v.x, v.y, v.z};
    quat_t q_inv = Quat_Inverse(q);
    quat_t v_rotated = Quat_Mult(Quat_Mult(q, v_quat), q_inv);
    return (vec3_t){v_rotated.x, v_rotated.y, v_rotated.z};
}
