
#ifndef __VECT_QUAT_MATH_H__
#define __VECT_QUAT_MATH_H__

// W = cos(alpha/2), X = x*sin(alpha/2), Y = y*sin(alpha/2), Z = z*sin(alpha/2)
// where W,X,Y,Z are the quaternion values and x,y,z is the vector defining your direction, which alpha rotates around 
typedef struct {
  float w, x, y, z; // W,X,Y,Z, not capitalized because it's inconvenient
} Quaternion;

// result = a * b
// WARNING: ab != ba
// <1,0,0,0>*x = x*<1,0,0,0> = x
// This is equivalent to cross product when W=0 for both vectors
void QuaternionsMultiply(Quaternion* result, Quaternion a, Quaternion b);

// result = conj(a)
// conj(a*b) = conj(b)*conj(a), normalized quaternions: q*conj(q) = q*(q^-1) = (q^-1)*q = 1
void QuaternionConjugate(Quaternion* result, Quaternion a);

// Multiply two 3D vectors (result = v .* u)
void Vectors2Multiply(float* result, float* v, float* u);
// Multiply a 3D vector by a scalar (result = scalar * v)
void VectorScale(float* result, float scalar, float* v);
// Add 3 3D vectors together (result = v + u + w)
void Vectors3Add(float* result, float* v, float* u, float* w);
// Add a scalar to a 3D vector (result = scalar .+ v)
void VectorScalarAdd(float* result, float scalar, float* v);

#endif
