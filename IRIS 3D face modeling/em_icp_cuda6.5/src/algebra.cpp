/*
  Copyright (c) 2010 Marcos Slomp, Toru Tamaki

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without
  restriction, including without limitation the rights to use,
  copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.
*/


/* Author: Marcos Slomp; Created: January 2010 */

#include "algebra.h"
#include <math.h>

void Transform(vector3 t, const matrix3x3 T, const vector3 v)
{
  t[0] = ScalarProduct(T[0], v);
  t[1] = ScalarProduct(T[1], v);
  t[2] = ScalarProduct(T[2], v);
}

void CrossProduct(vector3 vr, const vector3 v1, const vector3 v2)
{
  vr[0] = v1[1]*v2[2] - v1[2]*v2[1];
  vr[1] = v1[2]*v2[0] - v1[0]*v2[2];
  vr[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

float ScalarProduct(const vector3 v1, const vector3 v2)
{
  const float dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
  return(dot);
}

void AxisAngleRotation(matrix3x3 R, const vector3 axis, const float angle)
{
  /* The matrix derivation is very easy, actually. The link below provides
     some nice geometrical intuition and a detailed derivation:
     http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm */

  const float s = sinf(angle);
  const float c = cosf(angle);
  const float t = 1 - c;
  vector3 A;
  Normalize(A, axis);

  R[0][0] = t*A[0]*A[0] + c;
  R[0][1] = t*A[0]*A[1] - s*A[2];
  R[0][2] = t*A[0]*A[2] + s*A[1];

  R[1][0] = t*A[0]*A[1] + s*A[2];
  R[1][1] = t*A[1]*A[1] + c;
  R[1][2] = t*A[1]*A[2] - s*A[0];

  R[2][0] = t*A[0]*A[2] - s*A[1];
  R[2][1] = t*A[1]*A[2] + s*A[0];
  R[2][2] = t*A[2]*A[2] + c;
}

void Rotate(vector3 r, const vector3 v, const vector3 A, const float theta)
{
  matrix3x3 R;
  AxisAngleRotation(R, A, theta);
  Transform(r, R, v);
}

void VectorAddition(vector3 vr, const vector3 v1, const vector3 v2)
{
  vr[0] = v1[0] + v2[0];
  vr[1] = v1[1] + v2[1];
  vr[2] = v1[2] + v2[2];
}

void VectorSubtraction(vector3 vr, const vector3 v1, const vector3 v2)
{
  vr[0] = v1[0] - v2[0];
  vr[1] = v1[1] - v2[1];
  vr[2] = v1[2] - v2[2];
}

void VectorScale(vector3 vr, const vector3 v, const float s)
{
  vr[0] = v[0] * s;
  vr[1] = v[1] * s;
  vr[2] = v[2] * s;
}

void VectorAssign(vector3 vr, const vector3 v)
{
  vr[0] = v[0];
  vr[1] = v[1];
  vr[2] = v[2];
}

float Norm(const vector3 v)
{
  const float dot = ScalarProduct(v, v);
  const float norm = sqrtf(dot);
  return(norm);
}

void Normalize(vector3 vr, const vector3 v)
{
  const float norm = Norm(v);
  const float scale = 1.0f/norm;
  VectorScale(vr, v, scale);
}
