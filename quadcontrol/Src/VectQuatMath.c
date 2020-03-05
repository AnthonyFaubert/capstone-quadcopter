
#include "VectQuatMath.h"

// WARNING: ab != ba
// <1,0,0,0>*x = x*<1,0,0,0> = x
// This is equivalent to cross product when W=0 for both vectors
void QuaternionsMultiply(Quaternion* result, Quaternion a, Quaternion b) {
  result->w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
  result->x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
  result->y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x);
  result->z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w);
}

// conj(a*b) = conj(b)*conj(a), normalized quaternions: q*conj(q) = q*(q^-1) = (q^-1)*q = 1
Quaternion QuaternionConjugate(Quaternion a) {
  Quaternion result = {a.w, -a.x, -a.y, -a.z};
  return result;
}

// Rotates a 3D vector stored in a Quaternion struct by a quaternion and then returns the resulting 3D vector stored in a Quaternion struct.
Quaternion QuaternionRotateVector(Quaternion rotation, Quaternion vector) {
  Quaternion result;
  QuaternionsMultiply(&result, rotation, vector);
  QuaternionsMultiply(&result, result, QuaternionConjugate(rotation));
  return result;
}

float QuaternionCheckMagnitude(Quaternion q) {
  return q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
}

// Multiply two 3D vectors
void Vectors2Multiply(float* result, float* v, float* u) {
  for (int i = 0; i < 4; i++) {
    result[i] = v[i] * u[i];
  }
}
// Multiply a 3D vector by a scalar
void VectorScale(float* result, float scalar, float* v) {
  for (int i = 0; i < 4; i++) {
    result[i] = scalar * v[i];
  }
}
// Add 3 3D vectors together
void Vectors3Add(float* result, float* v, float* u, float* w) {
  for (int i = 0; i < 4; i++) {
    result[i] = v[i] + u[i] + w[i];
  }
}
// Add a scalar to a 3D vector
void VectorScalarAdd(float*result, float scalar, float* v) {
  for (int i = 0; i < 4; i++) {
    result[i] = scalar + v[i];
  }
}
