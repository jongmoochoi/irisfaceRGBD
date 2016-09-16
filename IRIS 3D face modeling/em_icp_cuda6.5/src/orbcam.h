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


#ifndef _ORBITAL_CAMERA_H_
#define _ORBITAL_CAMERA_H_

/* Author: Marcos Slomp; Created: January 2010 */

#include "algebra.h"

typedef struct orbit* orbcamera_t;

orbcamera_t orbCreateCamera();
void orbDeleteCamera(orbcamera_t cam);

void orbGetCameraPosition(vector3 pos, const orbcamera_t cam);
void orbGetViewingDirection(vector3 dir, const orbcamera_t cam);
void orbGetViewingUpVector(vector3 up, const orbcamera_t cam);
void orbGetOrbitalPoint(vector3 pos, const orbcamera_t cam);
float orbGetOrbitalDistance(const orbcamera_t cam);

void orbResetCamera(orbcamera_t cam);
void orbResetPosition(orbcamera_t cam);
void orbResetOrientation(orbcamera_t cam);

void orbSetOrbitalPoint(orbcamera_t cam, const vector3 center);
void orbSetOrbitalDistance(orbcamera_t cam, const float distance);

void orbZoom(orbcamera_t cam, const float stepsize);

void orbOrbitHorizontally(orbcamera_t cam, const float angle);
void orbOrbitVertically(orbcamera_t cam, const float angle);
void orbRoll(orbcamera_t cam, const float angle);

void orbLoadOpenGLViewTransform(orbcamera_t cam);

#endif/*_ORBITAL_CAMERA_H_*/
