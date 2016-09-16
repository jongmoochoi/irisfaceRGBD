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

#include "orbcam.h"
#include <stdlib.h>

typedef struct
{
  vector3 u;
  vector3 v;
  vector3 n;
} basis3;

struct orbit
{
  vector3 center;
  basis3  orientation;
  float   distance;
} pivot;

orbcamera_t orbCreateCamera()
{
  orbcamera_t cam = (orbcamera_t) malloc(sizeof(pivot));
  orbResetCamera(cam);
  return(cam);
}

void orbDeleteCamera(orbcamera_t cam)
{
  if (NULL == cam)
    return;

  free(cam);
}

void orbGetCameraPosition(vector3 pos, const orbcamera_t cam)
{
  VectorScale(pos, cam->orientation.n, cam->distance);
}

void orbGetViewingDirection(vector3 dir, const orbcamera_t cam)
{
  VectorScale(dir, cam->orientation.n, -1.0f);
}

void orbGetViewingUpVector(vector3 up, const orbcamera_t cam)
{
  VectorAssign(up, cam->orientation.v);
}

void orbGetOrbitalPoint(vector3 pos, const orbcamera_t cam)
{
  VectorAssign(pos, cam->center);
}

float orbGetOrbitalDistance(const orbcamera_t cam)
{
  return(cam->distance);
}

void orbResetCamera(orbcamera_t cam)
{
  orbResetPosition(cam);
  orbResetOrientation(cam);
}

void orbResetPosition(orbcamera_t cam)
{
  vector3 origin = { 0.0f, 0.0f, 0.0f };
  orbSetOrbitalPoint(cam, origin);
  orbSetOrbitalDistance(cam, 5.0f);
}

void orbResetOrientation(orbcamera_t cam)
{
  vector3 X = { 1.0f, 0.0f, 0.0f };
  vector3 Y = { 0.0f, 1.0f, 0.0f };
  vector3 Z = { 0.0f, 0.0f, 1.0f };

  VectorAssign(cam->orientation.u, X);
  VectorAssign(cam->orientation.v, Y);
  VectorAssign(cam->orientation.n, Z);
}

void orbSetOrbitalPoint(orbcamera_t cam, const vector3 center)
{
  VectorAssign(cam->center, center);
}

void orbSetOrbitalDistance(orbcamera_t cam, const float distance)
{
  cam->distance = distance;
}

void orbZoom(orbcamera_t cam, const float stepsize)
{
  orbSetOrbitalDistance(cam, cam->distance + stepsize*10.0f);
}

void orbOrbitHorizontally(orbcamera_t cam, const float angle)
{
  vector3 r;
  Rotate(r, cam->orientation.u, cam->orientation.v, angle);
  Normalize(r, r);
  VectorAssign(cam->orientation.u, r);
  CrossProduct(cam->orientation.n, cam->orientation.u, cam->orientation.v);
}

void orbOrbitVertically(orbcamera_t cam, const float angle)
{
  vector3 r;
  Rotate(r, cam->orientation.v, cam->orientation.u, angle);
  Normalize(r, r);
  VectorAssign(cam->orientation.v, r);
  CrossProduct(cam->orientation.n, cam->orientation.u, cam->orientation.v);
}

void orbRoll(orbcamera_t cam, const float angle)
{
  vector3 r;
  Rotate(r, cam->orientation.v, cam->orientation.n, angle);
  Normalize(r, r);
  VectorAssign(cam->orientation.v, r);
  CrossProduct(cam->orientation.u, cam->orientation.v, cam->orientation.n);
}

#ifdef WIN32
  #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
void orbLoadOpenGLViewTransform(orbcamera_t cam)
{
  vector3 eye;
  vector3 dir;
  vector3 up;
  vector3 to;

  orbGetCameraPosition(eye, cam);
  orbGetViewingDirection(dir, cam);
  orbGetViewingUpVector(up, cam);
  VectorAddition(to, eye, dir);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(eye[0], eye[1], eye[2], to[0], to[1], to[2], up[0], up[1], up[2]);
}
