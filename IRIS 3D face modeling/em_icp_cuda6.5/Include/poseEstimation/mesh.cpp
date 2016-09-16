/*
	mesh.cpp
	contains mesh processing functions

	author: Jatuporn Toy Leksut
	version 1.5 11/14/2014
*/
#include "facePoseEstimation.h"

// extern
void loadMesh(const string meshFile, Mesh& out_mesh);


///////////////////////////////////////////////////////////////////

/*
load mesh
*/
void loadMesh(const string meshFile, Mesh& out_mesh)
{ 
	if (!OpenMesh::IO::read_mesh(out_mesh, meshFile)) {
		printf("Failed to load mesh file: %s\n", meshFile.c_str());
		exit(-1);
	}

	out_mesh.request_face_normals();
	out_mesh.request_vertex_normals();
	out_mesh.request_vertex_colors();
	out_mesh.update_normals(); 
}
