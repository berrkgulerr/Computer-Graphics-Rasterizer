#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Mesh.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

using namespace tinyxml2;
using namespace std;

/*
	Transformations, clipping, culling, rasterization are done here.
	You may define helper functions.
*/

double Scene::line_equation(int x0,int y0, int x1,int y1,int x,int y){

	return (double)(x*(y0-y1) + y*(x1-x0) + x0*y1 - y0*x1);
}

void Scene::triangle_rasterization(Vec3 vector0, Vec3 vector1,  Vec3 vector2, Camera* camera){
	int xmin = min(vector0.x,min(vector1.x,vector2.x));
	int ymin = min(vector0.y,min(vector1.y,vector2.y));
	int xmax = max(vector0.x,max(vector1.x,vector2.x));
	int ymax = max(vector0.y,max(vector1.y,vector2.y));
	int x0 = vector0.x;
	int y0 = vector0.y;
	int x1 = vector1.x;
	int y1 = vector1.y;
	int x2 = vector2.x;
	int y2 = vector2.y;
	Color lastColor;
	Color* color0 = colorsOfVertices[vector0.colorId - 1];
	Color* color1 = colorsOfVertices[vector1.colorId - 1];
	Color* color2 = colorsOfVertices[vector2.colorId - 1];

	for(int y = ymin ; y<= ymax ; y++ ){
		for(int x = xmin ; x <= xmax ; x++){
			if(x < camera->horRes && x>=0 && y < camera->verRes && y >= 0) {

				double alpha = line_equation(x2,y2,x1,y1,x,y) / line_equation(x2,y2,x1,y1,x0,y0);
				double beta = line_equation(x0,y0,x2,y2,x,y) / line_equation(x0,y0,x2,y2,x1,y1);
				double gamma = line_equation(x1,y1,x0,y0,x,y) / line_equation(x1,y1,x0,y0,x2,y2);

				if (alpha >= 0 && beta >= 0 && gamma >= 0) {
					lastColor.r = makeBetweenZeroAnd255(color0->r * alpha + color1->r * beta + color2->r * gamma);
					lastColor.b = makeBetweenZeroAnd255(color0->b * alpha + color1->b * beta + color2->b * gamma);
					lastColor.g = makeBetweenZeroAnd255(color0->g * alpha + color1->g * beta + color2->g * gamma);
					image[x][y] = lastColor;
				}

			}
		}
	}
}

// Liang-Barsky Algorithm  Visible Function
bool Scene::visible(double den, double num, double *te, double *tl){
	double t;

	if(den>0){
		t = num/den;
		if(t>*tl)
			return false;
		if(t>*te)
			*te=t;
	}
	else if(den<0){
		t = num/den;
		if(t<*te)
			return false;
		if(t<*tl)
			*tl = t;
	}
	else if(num>0){
		return  false;
	}
	return true;
}
int Scene::decider(Vec3* vector0 , Vec3* vector1, bool control){
	int decider_int;
	int temp;
	if(control) {
		temp = vector0->x;
		vector0->x = vector0->y;
		vector0->y = temp;
		temp = vector1->x;
		vector1->x = vector1->y;
		vector1->y = temp;
	}
	if(vector1->x < vector0->x){
		temp = vector0->x;
		vector0->x = vector1->x;
		vector1->x = temp;
		temp = vector0->y;
		vector0->y = vector1->y;
		vector1->y = temp;	
		temp = vector0->z;
		vector0->z = vector1->z;
		vector1->z = temp;
		temp = vector0->colorId;
		vector0->colorId = vector1->colorId;
		vector1->colorId = temp;
	}
	if(vector0->y < vector1->y)
		decider_int = 1;
	else
		decider_int = -1;
	return decider_int;

}


void Scene::mid_point_line_algorithm(Vec3 vector0 , Vec3 vector1) {
	Color color;
	Color dc;

	bool controlys = abs(vector1.y - vector0.y) > abs(vector1.x- vector0.x);
	if(controlys) {
		swap(vector0.x,vector0.y);
		swap(vector1.x, vector1.y);
	}
	if(vector1.x<vector0.x){
		swap(vector0,vector1);
	}
	int step;
	if(vector0.y<vector1.y)
		step = 1;
	else
		step = -1;
	int x1_x0 = vector1.x - vector0.x;
	int y1_y0 = abs(vector1.y-vector0.y);
	int y = vector0.y;
	float d = -1*y1_y0 + 0.5 * (x1_x0);

	color.r = colorsOfVertices[vector0.colorId - 1]->r;
	color.b = colorsOfVertices[vector0.colorId - 1]->b;
	color.g = colorsOfVertices[vector0.colorId - 1]->g;

	dc.r = (colorsOfVertices[vector1.colorId - 1]->r - color.r) / x1_x0;
	dc.b = (colorsOfVertices[vector1.colorId - 1]->b - color.b) / x1_x0;
	dc.g = (colorsOfVertices[vector1.colorId - 1]->g - color.g) / x1_x0;


	for (int x = (int)vector0.x; x <=(int)vector1.x; x++) {

		color.r = makeBetweenZeroAnd255(color.r);
		color.b = makeBetweenZeroAnd255(color.b);
		color.g = makeBetweenZeroAnd255(color.g);
		if(controlys){
			image[y][x] = color;
		}else{
			image[x][y] = color;
		}

		if (d < 0) {
			y = y + step;
			d += (-1*y1_y0+x1_x0);
		} else {
			d += -1*y1_y0;

		}
		color.r += dc.r;
		color.b += dc.b;
		color.g += dc.g;
	}
}
void Scene::line_rasterization(Vec3 vector1, Vec3 vector2, Camera* camera){
	double te = 0; 
	double tl = 1;
	bool bool_visible = false;
	int xmin = 0;
	int ymin = 0;
	int xmax = camera->horRes-1;
	int ymax = camera->verRes-1;
	vector1.x = (int)vector1.x;
	vector2.x = (int)vector2.x;
	vector1.y = (int)vector1.y;
	vector2.y = (int)vector2.y;

	int x0 = vector1.x;
	int y0 = vector1.y;
	int x1 = vector2.x;
	int y1 = vector2.y;

	double dx = x1-x0;
	double dy = y1-y0;


	if(visible(dx,(xmin-x0),&te,&tl)) {
		if (visible(-1*dx, (x0-xmax), &te, &tl)) {
			if (visible(dy, (ymin-y0), &te, &tl)){
				if (visible(-1*dy, (y0-ymax), &te, &tl)) {
					bool_visible = true;
					if (tl < 1) {
						vector2.x = vector1.x + dx * tl;
						vector2.y = vector1.y + dy * tl;
					}
					if (te > 0) {
						vector1.x = vector1.x + dx * te;
						vector1.y = vector1.y + dy * te;
					}
					
					mid_point_line_algorithm(vector1,vector2);

				}
			}
		}
	}

}
void Scene::forwardRenderingPipeline(Camera *camera)
{
	double n = camera->near;
	double f = camera->far;
	double l = camera->left;
	double r = camera->right;
	double t = camera->top;
	double b = camera->bottom;
	double nx = camera->horRes;
	double ny = camera->verRes;


	double camera_transformation[4][4] = {
			{camera->u.x, camera->u.y, camera->u.z, -(camera->u.x*camera->pos.x + camera->u.y*camera->pos.y + camera->u.z*camera->pos.z)},
			{camera->v.x, camera->v.y, camera->v.z, -(camera->v.x*camera->pos.x + camera->v.y*camera->pos.y + camera->v.z*camera->pos.z)},
			{camera->w.x, camera->w.y, camera->w.z, -(camera->w.x*camera->pos.x + camera->w.y*camera->pos.y + camera->w.z*camera->pos.z)},
			{0, 0, 0, 1}};

	Matrix4 projection_matrix;
	if (camera->projectionType) { // Perspective
		double matrix[4][4] = {{2*n/(r-l), 0, (r+l)/(r-l), 0},
							{0, 2*n/(t-b), (t+b)/(t-b), 0},
							{0, 0, -(f+n)/(f-n), -2*f*n/(f-n)},
							{0, 0, -1, 0}};
		projection_matrix = Matrix4(matrix);
	}
	else { // Ortographic
    double matrix[4][4] = {{2/(r-l), 0, 0, -(r+l)/(r-l)},
                           {0, 2/(t-b), 0, -(t+b)/(t-b)},
                           {0, 0, -2/(f-n), -(f+n)/(f-n)},
                           {0, 0, 0, 1}};
		projection_matrix = Matrix4(matrix);
	}

	Matrix4 camera_transformation_matrix = Matrix4(camera_transformation);
	Vec4 view_port_matrix[3] = {Vec4(nx/2, 0, 0, (nx-1)/2, -1),
									Vec4(0, ny/2, 0, (ny-1)/2, -1),
									Vec4(0, 0, 0.5, 0.5, -1)};
	for (Mesh *mesh:meshes)
	{
		Matrix4 modeling_transformation_matrix = getIdentityMatrix();
		int number_of_transformations = mesh->numberOfTransformations;
		for (int i = 0; i < number_of_transformations; i++) {
			Matrix4 type_matrix = Matrix4();
			char type = mesh->transformationTypes[i];
			int id = mesh->transformationIds[i];

			if (type == 't'){
				for (auto translation : translations) {
					if (translation->translationId == id) {
						double translation_matrix[4][4] = {{1, 0, 0, translation->tx},
															{0, 1, 0, translation->ty},
															{0, 0, 1, translation->tz},
															{0, 0, 0, 1}};
						type_matrix =  Matrix4(translation_matrix);
					}
				}
			}
			else if(type == 'r'){
				for (auto rotation:rotations) {
					if (rotation->rotationId == id) {
						double ux = rotation->ux;
						double uy = rotation->uy;
						double uz = rotation->uz;
						Vec3 u(ux, uy, uz, -1);

						u = normalizeVec3(u);
						ux = u.x;
						uy = u.y;
						uz = u.z;
						int tag = 0;
						double min = ux;
						if(uy < min){
							min = uy;
							tag = 1;
						}
						if(uz < min){
							min = uz;
							tag = 2;
						}
						Vec3 v;
						switch (tag) {
							case 0:
								v = Vec3(0, -uz, uy, -1);
								break;

							case 1:
								v = Vec3(-uz, 0, ux, -1);
								break;

							case 2:
								v = Vec3(-uy, ux, 0, -1);
								break;

							default:
								break;
						}
						v = normalizeVec3(v);
						Vec3 w = crossProductVec3(u, v);
						w = normalizeVec3(w);

						double angle = rotation->angle * (0.01745327777777); // RADIAN VALUE
						double rotation_matrix[4][4] = {{1, 0, 0, 0},
														{0, cos(angle), -sin(angle), 0},
														{0, sin(angle), cos(angle), 0},
														{0, 0, 0, 1}};
						Matrix4 r_x(rotation_matrix);
						double m_matrix[4][4] = {{u.x, u.y, u.z, 0},
											{v.x, v.y, v.z, 0},
											{w.x, w.y, w.z, 0},
											{0, 0, 0, 1}};
						Matrix4 m(m_matrix);

						double m_matrix_inverse[4][4] = {{u.x, v.x, w.x, 0},
														{u.y, v.y, w.y, 0},
														{u.z, v.z, w.z, 0},
														{0, 0, 0, 1}};
						Matrix4 m_inverse(m_matrix_inverse);

						type_matrix = multiplyMatrixWithMatrix(m_inverse, multiplyMatrixWithMatrix(r_x, m));
					}
				}
			}
			else if(type == 's'){
					for (auto scaling : scalings) {
						if (scaling->scalingId == id) {
							double scaling_matrix[4][4] = {{scaling->sx, 0, 0, 0},
															{0, scaling->sy, 0, 0},
															{0, 0, scaling->sz, 0},
															{0, 0, 0, 1}};
							type_matrix = Matrix4(scaling_matrix);
						}
					}
			}
			else{
				type_matrix = nullptr;
			}
			modeling_transformation_matrix = multiplyMatrixWithMatrix(type_matrix,modeling_transformation_matrix);
		}

		Matrix4 mult_camera_with_modeling = multiplyMatrixWithMatrix(camera_transformation_matrix, modeling_transformation_matrix);
		Matrix4 mult_projection_camera_with_modeling = multiplyMatrixWithMatrix(projection_matrix, mult_camera_with_modeling);

		for(Triangle triangle:mesh->triangles)
		{	
			Vec3* first_vertex = vertices[triangle.getFirstVertexId()-1];
			Vec3* second_vertex = vertices[triangle.getSecondVertexId()-1];
			Vec3* third_vertex = vertices[triangle.getThirdVertexId()-1];
			Vec4 vec4_first_vertex(first_vertex->x, first_vertex->y, first_vertex->z, 1, first_vertex->colorId);
			Vec4 vec_4_second_vertex(second_vertex->x, second_vertex->y, second_vertex->z, 1, second_vertex->colorId);
			Vec4 vec_4_third_vertex(third_vertex->x, third_vertex->y, third_vertex->z, 1, third_vertex->colorId);

			vec4_first_vertex = multiplyMatrixWithVec4 (mult_projection_camera_with_modeling, vec4_first_vertex);
			vec_4_second_vertex = multiplyMatrixWithVec4 (mult_projection_camera_with_modeling, vec_4_second_vertex);
			vec_4_third_vertex = multiplyMatrixWithVec4 (mult_projection_camera_with_modeling, vec_4_third_vertex);

			if(vec4_first_vertex.t != 0) {
				vec4_first_vertex.x = vec4_first_vertex.x/vec4_first_vertex.t;
				vec4_first_vertex.y = vec4_first_vertex.y/vec4_first_vertex.t;
				vec4_first_vertex.z = vec4_first_vertex.z/vec4_first_vertex.t;
				vec4_first_vertex.t = 1;
			}
			if(vec_4_second_vertex.t != 0) {
				vec_4_second_vertex.x = vec_4_second_vertex.x/vec_4_second_vertex.t;
				vec_4_second_vertex.y = vec_4_second_vertex.y/vec_4_second_vertex.t;
				vec_4_second_vertex.z = vec_4_second_vertex.z/vec_4_second_vertex.t;
				vec_4_second_vertex.t = 1;
			}
			if(vec_4_third_vertex.t != 0) {
				vec_4_third_vertex.x = vec_4_third_vertex.x/vec_4_third_vertex.t;
				vec_4_third_vertex.y = vec_4_third_vertex.y/vec_4_third_vertex.t;
				vec_4_third_vertex.z = vec_4_third_vertex.z/vec_4_third_vertex.t;
				vec_4_third_vertex.t = 1;
			}

			Vec3 transformed_first_vertex;
			Vec3 transformed_second_vertex;
			Vec3 transformed_third_vertex;
			for (int i = 0; i < 3; i++) {
				Vec4 row = view_port_matrix[i];

				transformed_first_vertex.setElementAt(i,row.x*vec4_first_vertex.x + row.y*vec4_first_vertex.y + row.z*vec4_first_vertex.z + row.t*vec4_first_vertex.t);

				transformed_second_vertex.setElementAt(i,row.x*vec_4_second_vertex.x + row.y*vec_4_second_vertex.y + row.z*vec_4_second_vertex.z + row.t*vec_4_second_vertex.t);
				
				transformed_third_vertex.setElementAt(i,row.x*vec_4_third_vertex.x + row.y*vec_4_third_vertex.y + row.z*vec_4_third_vertex.z + row.t*vec_4_third_vertex.t);
			}

			if (cullingEnabled) {
				Vec3 edge1 = subtractVec3(transformed_second_vertex, transformed_first_vertex);
				Vec3 edge2 = subtractVec3(transformed_third_vertex, transformed_first_vertex);
				Vec3 normal = crossProductVec3(edge2, edge1);
				normal = normalizeVec3(normal);
				Vec3 viewing_vector = transformed_first_vertex;
				viewing_vector = normalizeVec3(viewing_vector);
				if (dotProductVec3(viewing_vector, normal) > 0) {
					continue;
				}
			}

			transformed_first_vertex.colorId = first_vertex->colorId;
			transformed_second_vertex.colorId = second_vertex->colorId;
			transformed_third_vertex.colorId = third_vertex->colorId;
 			
			// Solid
			if (mesh->type) {
				triangle_rasterization(transformed_first_vertex, transformed_second_vertex, transformed_third_vertex, camera);
			}
			// Wireframe
			else {
				line_rasterization(transformed_first_vertex, transformed_second_vertex, camera);
				line_rasterization(transformed_second_vertex, transformed_third_vertex, camera);
				line_rasterization(transformed_third_vertex, transformed_first_vertex, camera);
			}

		}
	}

}


/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL) {
		str = pElement->GetText();
		
		if (strcmp(str, "enabled") == 0) {
			cullingEnabled = true;
		}
		else {
			cullingEnabled = false;
		}
	}

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		// read projection type
		str = pCamera->Attribute("type");

		if (strcmp(str, "orthographic") == 0) {
			cam->projectionType = 0;
		}
		else {
			cam->projectionType = 1;
		}

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read meshes
	pElement = pRoot->FirstChildElement("Meshes");

	XMLElement *pMesh = pElement->FirstChildElement("Mesh");
	XMLElement *meshElement;
	while (pMesh != NULL)
	{
		Mesh *mesh = new Mesh();

		pMesh->QueryIntAttribute("id", &mesh->meshId);

		// read projection type
		str = pMesh->Attribute("type");

		if (strcmp(str, "wireframe") == 0) {
			mesh->type = 0;
		}
		else {
			mesh->type = 1;
		}

		// read mesh transformations
		XMLElement *pTransformations = pMesh->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			mesh->transformationTypes.push_back(transformationType);
			mesh->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		mesh->numberOfTransformations = mesh->transformationIds.size();

		// read mesh faces
		char *row;
		char *clone_str;
		int v1, v2, v3;
		XMLElement *pFaces = pMesh->FirstChildElement("Faces");
        str = pFaces->GetText();
		clone_str = strdup(str);

		row = strtok(clone_str, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);
			
			if (result != EOF) {
				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		meshes.push_back(mesh);

		pMesh = pMesh->NextSiblingElement("Mesh");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}