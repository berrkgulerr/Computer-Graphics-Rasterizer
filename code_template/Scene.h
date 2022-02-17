#ifndef _SCENE_H_
#define _SCENE_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "Camera.h"
#include "Color.h"
#include "Mesh.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Matrix4.h"

using namespace std;

class Scene
{
public:
	Color backgroundColor;
	bool cullingEnabled;

	vector< vector<Color> > image;
	vector< Camera* > cameras;
	vector< Vec3* > vertices;
	vector< Color* > colorsOfVertices;
	vector< Scaling* > scalings;
	vector< Rotation* > rotations;
	vector< Translation* > translations;
	vector< Mesh* > meshes;

	Scene(const char *xmlPath);

	void initializeImage(Camera* camera);
	void forwardRenderingPipeline(Camera* camera);
	int makeBetweenZeroAnd255(double value);
	void writeImageToPPMFile(Camera* camera);
	void convertPPMToPNG(string ppmFileName, int osType);

	void line_rasterization(Vec3 v1, Vec3 v2, Camera* camera);
	void triangle_rasterization(Vec3 v0, Vec3 v1, Vec3 v2, Camera* camera);
	double line_equation(int x0, int y0, int x1, int y1, int x, int y);
	bool visible(double den, double num, double *te, double *tl);
	void mid_point_line_algorithm(Vec3 vector0, Vec3 vector1);
	int decider(Vec3* vector0 , Vec3* vector1, bool control);
};

#endif