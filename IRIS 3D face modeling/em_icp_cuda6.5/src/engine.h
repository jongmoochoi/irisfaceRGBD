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

#include "DATA.h"


#ifndef _POINT_CLOUD_ENGINE_H_
#define _POINT_CLOUD_ENGINE_H_

bool EngineInit();
bool EngineShutDown();

void setWinFlag();
bool isWinFlag();
bool isUpdated();

bool EngineIteration();
bool EngineIteration(int Ysize, float* points2,
                     float* h_Y, const float* h_R, const float* h_t);


bool EnginePointCloudData(const unsigned int index, const float* points, const int count);
bool EnginePointCloudDecoration(const unsigned int index, const float R, const float G, const float B, const float pointsize);

bool EngineCameraSetup(const float distance);



void initEngineRGB();
void setEngineRGB(float RGB, int i);
void freeEngineRGB();


void initEngineMesh(int nb_mesh);
void setEngineMesh(int v1, int v2, int v3, int i) ;
void freeEngineMesh();

void initEngineXYZ();
void setEngineXYZ(float XYZ, int i);
void freeEngineXYZ();

void initEngineNormals();
void setEngineNormals(float N, int i);
void freeEngineNormals();

void setNbEngineMesh(int nb);


#endif//_POINT_CLOUD_ENGINE_H_
