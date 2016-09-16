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

#ifndef _BASIC_LINEAR_ALGEBRA_H_
#define _BASIC_LINEAR_ALGEBRA_H_

/* Author: Marcos Slomp; Created: January 2010 */

/* The basic elements of the algebra are defined here (namely 3D vector
   and 3x3 matrix), as well as some very useful operations on them.
   They are defined here as simple, standard C static size array types.
   By definition, static sized arrays in C are pointers, so there will be
   no performance overhead when passing them as arguments in the functions
   declared below (they will be passed by-reference and not by-copy).
   More "handy" declarations are possible (like using structs and unions),
   but I want to keep things simple and avoid non-standard C declarations
   like unnamed structs, etc. */
typedef float vector3 [3];
typedef float matrix3x3 [3][3];

void Transform(vector3 t, const matrix3x3 T, const vector3 v);

void CrossProduct(vector3 vr, const vector3 v1, const vector3 v2);
float ScalarProduct(const vector3 v1, const vector3 v2);

void AxisAngleRotation(matrix3x3 R, const vector3 axis, const float angle);
void Rotate(vector3 r, const vector3 v, const vector3 A, const float theta);

void VectorAddition(vector3 vr, const vector3 v1, const vector3 v2);
void VectorSubtraction(vector3 vr, const vector3 v1, const vector3 v2);
void VectorScale(vector3 vr, const vector3 v, const float s);
void VectorAssign(vector3 vr, const vector3 v);

float Norm(const vector3 v);
void Normalize(vector3 vr, const vector3 v);

#endif/*_BASIC_LINEAR_ALGEBRA_H_*/
