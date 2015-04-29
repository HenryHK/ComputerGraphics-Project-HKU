/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <random>  
#include <ctime>

#include <omp.h>

Raytracer::Raytracer() : _root(new SceneDagNode()) {

}

Raytracer::~Raytracer() {
	// Clean up

	// Delete light sources
	for (int i = 0; i < _lightSource.size(); i++)
	{
		delete _lightSource[i];
	}

	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material *mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;
}

void Raytracer::addLightSource( LightSource* light ) {
	_lightSource.push_back(light);
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}

bool flatFlag = false;
void Raytracer::flattenScene(SceneDagNode* node, Matrix4x4 parentMTW, Matrix4x4 parentWTM)
{
	if (node->obj)
	{
		node->parentModelToWorld = parentMTW;
		node->parentWorldToModel = parentWTM;
		node->modelToWorld = parentMTW * node->trans;
		node->worldToModel = node->invtrans * parentWTM;

		parentMTW = node->modelToWorld;
		parentWTM = node->worldToModel;
	}

	SceneDagNode *childPtr = node->child;
	while (childPtr != NULL) {
		flattenScene(childPtr, parentMTW, parentWTM);
		childPtr = childPtr->next;
	}
}

void Raytracer::traverseScene(SceneDagNode* node, Ray3D& ray) {

	//Flattened scene traverse
	if (!flatFlag) {	//call flattenScene if current scene graph not flat
		flattenScene(node, _modelToWorld, _worldToModel);
		flatFlag = true;
	}
	SceneDagNode *childPtr;
	// Applies transformation of the current node to the global transformation matrices.
	_modelToWorld = node->modelToWorld;
	_worldToModel = node->worldToModel;
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray);
		childPtr = childPtr->next;
	}
	// Removes transformation of the current node from the global transformation matrices.
	_worldToModel = node->parentWorldToModel;
	_modelToWorld = node->parentModelToWorld;
}

void Raytracer::computeShading( Ray3D& ray ) { //TODO

	Colour color(0, 0, 0);

	for (int i = 0; i < _lightSource.size(); i++)
	{
		LightSource *curLight = _lightSource[i];
		Colour curShade = curLight->shade(ray, _shadeMode);
		//curLight->shade(ray, _shadeMode);
		if (_shadowFlag)
		{
			/*
				Implementation of soft shadows, by sampling the light direction with
				random jitter, then compute average shadow shade
			*/
			double theta = 2 * M_PI * erand();
			Vector3D unitOffset(cos(theta), 0, sin(theta));
			Point3D randLightPos = curLight->get_position() + erand() * unitOffset;

			// Shadow direction
			Vector3D s_dir = randLightPos - ray.intersection.point;

			double lightDistance = s_dir.length();

			s_dir.normalize();

			// Shadow origin
			Point3D s_origin = ray.intersection.point + EPS * s_dir;

			// Shadow ray
			Ray3D s_ray(s_origin, s_dir);

			// Traverse scene using shadow ray
			traverseScene(_root, s_ray);

			// Check if it's a shadow ray
			if (!s_ray.intersection.none && s_ray.intersection.t_value < lightDistance)
			{
				continue; //outside the shadow range
			}
			else{
				
			}
		}
		color += curShade;
	}
	color.clamp();
	ray.col = color;
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray, int max_depth ) {
	Colour col(0.0, 0.0, 0.0); 

	traverseScene(_root, ray); 
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {

		computeShading(ray);
		col = ray.col;

		// You'll want to call shadeRay recursively (with a different ray, 
		// of course) here to implement reflection/refraction effects.  

		if (max_depth > 0)
		{

			// Normal at intersection surface
			Vector3D N = ray.intersection.normal;

			// Incidence direction
			Vector3D I = -ray.dir;
			
			double R = 1.0;

			// Refraction
			if (ray.intersection.mat->transmissive)
			{	
				double n1 = 1.0;
				double n2 = 1.0;

				// Refraction color
				Colour refrColor(0, 0, 0);

				
				if (I.dot(N) > 0)
				{
					// Entering transmissive object
					n2 = ray.intersection.mat->refrac_idx;
				}
				else
				{
					// Leaving transmissive object
					n1 = ray.intersection.mat->refrac_idx;
					N = -N;
				}
				
				// Compute refraction direction
				double n = n1 / n2;
				double cosI = N.dot(I);
				double sinT2 = n * n * (1.0 - cosI * cosI);
				if (sinT2 <= 1)
				{
					double cosT = std::sqrt(1.0 - sinT2);
					Vector3D refr_dir = n * I - (n * cosI + cosT) * N;
					refr_dir.normalize();

					Point3D refr_origin = ray.intersection.point + EPS * refr_dir;
					Ray3D refr_ray(refr_origin, refr_dir);

					double Rs = (n1 * cosI - n2 * cosT) / (n1 * cosI + n2 * cosT);
					double Rp = (n1 * cosT - n2 * cosI) / (n1 * cosT + n2 * cosI);
					R = (Rs * Rs + Rp * Rp) / 2.0;

					refrColor = (1.0 - R) * ray.intersection.mat->getTextureColor(ray) * shadeRay(refr_ray, max_depth - 1);
				}

				// Add refraction
				col += refrColor;
			}

			// Primary reflection direction
			Vector3D refl_dir = 2 * N.dot(I) * N - I;
			refl_dir.normalize();

			// Reflection origin offset a little to avoid same intersection
			Point3D refl_origin = ray.intersection.point + EPS * refl_dir;

			// Reflection color
			Colour reflColor(0, 0, 0);

			// Glossy reflection
			Vector3D u = refl_dir.cross(N);
			u.normalize();
			Vector3D v = refl_dir.cross(u);
			v.normalize();

			// Generate random ray direction
			Ray3D refl_ray;
			refl_ray.origin = refl_origin;

			// Generate cosine weighted random hemisphere samples
			double theta = acos(pow(erand(), 1.0 / (ray.intersection.mat->specular_exp + 1)));
			double phi = 2 * M_PI * erand();
			double x = sin(theta) * cos(phi);
			double y = sin(theta) * sin(phi);
			double z = cos(theta);

			refl_ray.dir = x * u + y * v + z * refl_dir;
			refl_ray.dir.normalize();

			reflColor += shadeRay(refl_ray, max_depth - 1);

			// Add reflection
			col += R * ray.intersection.mat->specular * ray.intersection.mat->getTextureColor(ray) * reflColor;
		}
	}

	col.clamp();
	return col; 
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
	Vector3D up, double fov, int max_depth, int samples, double aperture, double focalLength, ShadeMode mode, bool shadowFlag, double gamma, char* fileName) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	_shadeMode = mode;
	_shadowFlag = shadowFlag;
	double factor = (height / 2.0) / tan(fov * M_PI / 360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	for (int i = 0; i < _scrHeight; i++) {
		fprintf(stderr, "\rRendering %5.2f%%", 100.0 * i / (_scrHeight - 1));
		for (int j = 0; j < _scrWidth; j++) {

			//anti-aliasing using super-sampling
			Colour totalColor(0.0, 0.0, 0.0);
			int numSample = 4;
			for (int h = 0; h < numSample; h++){
				for (int k = 0; k < numSample; k++){
					double sampleDist1 = (h + 1.0*rand() / RAND_MAX) / numSample;
					double sampleDist2 = (k + 1.0*rand() / RAND_MAX) / numSample;

					Point3D origin(0, 0, 0);
					Point3D imagePlane;
					imagePlane[0] = (-double(width) / 2 + sampleDist1 + j) / factor;
					imagePlane[1] = (-double(height) / 2 + sampleDist2 + i) / factor;
					imagePlane[2] = -1;

					Point3D originW = viewToWorld * imagePlane;
					Vector3D directionW = viewToWorld * (imagePlane - origin);
					directionW.normalize();
					Ray3D ray(originW, directionW);
					Colour color = shadeRay(ray,max_depth);

					totalColor = totalColor + color;
				}
			}

			Colour col((totalColor[0] / pow(numSample, 2)),
				(totalColor[1] / pow(numSample, 2)), (totalColor[2] / pow(numSample, 2)));
			col = Colour(std::fmin(1.0, col[0]), std::fmin(1.0, col[1]), std::fmin(1.0, col[2]));

			_rbuffer[i*width + j] = int(col[0] * 255);
			_gbuffer[i*width + j] = int(col[1] * 255);
			_bbuffer[i*width + j] = int(col[2] * 255);
		}
	}

	// Flush buffer, save image to file
	flushPixelBuffer(fileName);
}


int main(int argc, char* argv[])
{
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320;
	int height = 240;

	int scene = 2;

	// Maximum trace depth
	int max_depth = 6; 

	// samples per pixel (ssp) in AAX
	int samples = 16;

	if (argc == 5) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
		samples = atoi(argv[3]) / 16;
		scene = atoi(argv[4]);
	}
	else {
		printf("Usage: \"RayTracer [width] [height] [samples] [option]\", option==1, task 1; option==2, task2 must-dos; option==3, full features\n");
		exit(-1);
	}

	// Loading texture map
	Texture worldmap("earth_physical.bmp");
	Texture checkerboard("checkerboard.bmp");
	Texture azeroth_map("azeroth.bmp");
	// Define materials for shading
	Material gold(Colour(0.3, 0.3, 0.3), 
		Colour(0.75164, 0.60648, 0.22648),
		Colour(0.628281, 0.555802, 0.366065),
		51.2, 1, false, NULL);

	Material jade(Colour(0, 0, 0), 
		Colour(0.54, 0.89, 0.63),
		Colour(0.316228, 0.316228, 0.316228),
		12.8, 1, false, NULL);

	Material red(Colour(0.1, 0.0, 0.0), 
		Colour(0.4, 0.4, 0.4),
		Colour(0.6, 0.05, 0.05),
		1, 1, false, NULL);

	Material blue(Colour(0.0, 0.0, 0.1), 
		Colour(0.4, 0.4, 0.4),
		Colour(0.05, 0.05, 0.6),
		1, 1, false, NULL);

	Material white(Colour(0.01, 0.01, 0.01), 
		Colour(0.5, 0.5, 0.5),
		Colour(0.5, 0.5, 0.5),
		1, 1, false, NULL);

	Material earth(Colour(0.1, 0.1, 0.1), 
		Colour(0.8, 0.8, 0.8),
		Colour(0.1, 0.1, 0.1),
		10, 1, false, &worldmap);

	Material azeroth(Colour(0.1, 0.1, 0.1),
		Colour(0.8, 0.8, 0.8),
		Colour(0.1, 0.1, 0.1),
		10, 1, false, &azeroth_map);

	Material silver(Colour(0.19125, 0.19125, 0.19125), 
		Colour(0.50754, 0.50754, 0.50754),
		Colour(0.508273, 0.508273, 0.508273),
		100, 1.3, false, NULL);

	Material glass(Colour(0.001, 0.001, 0.001), 
		Colour(0.0, 0.0, 0.0),
		Colour(0.999, 0.999, 0.999),
		10000, 1.5, true, NULL);

	Material mirror(Colour(0.001, 0.001, 0.001),
		Colour(0.0, 0.0, 0.0),
		Colour(0.999, 0.999, 0.999),
		10000, 1.5, false, NULL);

	Material glossyMirror(Colour(0.01, 0.01, 0.01), 
		Colour(0.1, 0.1, 0.1),
		Colour(0.9, 0.9, 0.9),
		1000, 1.5, false, NULL);

	Material board(Colour(0.01, 0.01, 0.01),
		Colour(0.09, 0.09, 0.09),
		Colour(0.9, 0.9, 0.9),
		10000, 1.5, false, &checkerboard);


	// Load scene
	switch (scene)
	{
		case 0:
		{//task1
		

		// Defines a point light source.
		raytracer.addLightSource(new PointLight(Point3D(0, 0, 5),
			Colour(0.9, 0.9, 0.9)));

		// Add a unit square into the scene with material mat.
		SceneDagNode* sphere = raytracer.addObject(new UnitSphere(), &gold);
		SceneDagNode* plane = raytracer.addObject(new UnitSquare(), &jade);

		// Apply some transformations to the unit square.
		double factor1[3] = { 1.0, 2.0, 1.0 };
		double factor2[3] = { 6.0, 6.0, 6.0 };
		raytracer.translate(sphere, Vector3D(0, 0, -5));
		raytracer.rotate(sphere, 'x', -45);
		raytracer.rotate(sphere, 'z', 45);
		raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

		raytracer.translate(plane, Vector3D(0, 0, -7));
		raytracer.rotate(plane, 'z', 45);
		raytracer.scale(plane, Point3D(0, 0, 0), factor2);

		// Render the scene, feel free to make the image smaller for
		// testing purposes.	

		char *filenames[3][2] = {
				{ "sig_view1.bmp", "sig_view2.bmp" },
				{"diffuse_view1.bmp", "diffuse_view2.bmp"},
				{"phong_view1.bmp", "phong_view2.bmp"}
		};

		for (int i = 0; i < 3; i++)
		{
			Point3D eye(0, 0, 1);
			Vector3D view(0, 0, -1);
			Vector3D up(0, 1, 0);
			double fov = 60;
			raytracer.render(width, height, eye, view, up, fov, static_cast<ShadeMode>(i), false, filenames[i][0]);

			// Render it from a different point of view.
			Point3D eye2(4, 2, 1);
			Vector3D view2(-4, -2, -6);
			raytracer.render(width, height, eye2, view2, up, fov, static_cast<ShadeMode>(i), false, filenames[i][1]);
		}
		break;
		}
		case 1:
		{
				  // Defines a point light source.
				  raytracer.addLightSource(new PointLight(Point3D(0, 0, 5),
					  Colour(0.9, 0.9, 0.9)));

				  // Add a unit square into the scene with material mat.
				  SceneDagNode* sphere = raytracer.addObject(new UnitSphere(), &gold);
				  SceneDagNode* plane = raytracer.addObject(new UnitSquare(), &jade);

				  // Apply some transformations to the unit square.
				  double factor1[3] = { 1.0, 2.0, 1.0 };
				  double factor2[3] = { 6.0, 6.0, 6.0 };
				  raytracer.translate(sphere, Vector3D(0, 0, -5));
				  raytracer.rotate(sphere, 'x', -45);
				  raytracer.rotate(sphere, 'z', 45);
				  raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

				  raytracer.translate(plane, Vector3D(0, 0, -7));
				  raytracer.rotate(plane, 'z', 45);
				  raytracer.scale(plane, Point3D(0, 0, 0), factor2);

				  // Render the scene, feel free to make the image smaller for
				  // testing purposes.	

				  char *filenames[3][2] = {
					  { "sig_view1.bmp", "sig_view2.bmp" },
					  { "diffuse_view1.bmp", "diffuse_view2.bmp" },
					  { "phong_view1.bmp", "phong_view2.bmp" }
				  };

				  for (int i = 0; i < 3; i++)
				  {
					  Point3D eye(0, 0, 1);
					  Vector3D view(0, 0, -1);
					  Vector3D up(0, 1, 0);
					  double fov = 60;
					  raytracer.render(width, height, eye, view, up, fov, static_cast<ShadeMode>(i), true, filenames[i][0]);

					  // Render it from a different point of view.
					  Point3D eye2(4, 2, 1);
					  Vector3D view2(-4, -2, -6);
					  raytracer.render(width, height, eye2, view2, up, fov, static_cast<ShadeMode>(i), true, filenames[i][1]);
				  }
				  break;
		}
		case 2:
		{
			  // Scene 2

			  // Camera parameters.
			  Point3D eye(0, 2, 10);
			  Vector3D view(0, 0, -1);
			  Vector3D up(0, 1, 0);
			  double fov = 60;
			  double aperture = 0.6;
			  double focalLength = 12;

			  // Defines a point light source.
			  raytracer.addLightSource(new PointLight(Point3D(0, 6, 3),
				  Colour(0.2, 0.2, 0.2), Colour(0.8, 0.8, 0.8), Colour(0.8, 0.8, 0.8)));

			  // Construct scene
			  SceneDagNode* floor = raytracer.addObject(new UnitSquare(), &glossyMirror);
			  SceneDagNode* ceiling = raytracer.addObject(new UnitSquare(), &jade);
			  SceneDagNode* leftWall = raytracer.addObject(new UnitSquare(), &blue);
			  SceneDagNode* rightWall = raytracer.addObject(new UnitSquare(), &red);
			  SceneDagNode* backWall = raytracer.addObject(new UnitSquare(), &white);
			  SceneDagNode* cylinder = raytracer.addObject(new UnitCylinder(), &gold);
			  SceneDagNode* earthSphere = raytracer.addObject(new UnitSphere(), &azeroth);
			  SceneDagNode* glassSphere = raytracer.addObject(new UnitSphere(), &glass);

			  // Apply transformations
			  double wallScale[3] = { 100.0, 100.0, 100.0 };
			  raytracer.translate(floor, Vector3D(0, -3, 0));
			  raytracer.rotate(floor, 'x', -90);
			  raytracer.scale(floor, Point3D(0, 0, 0), wallScale);

			  raytracer.translate(backWall, Vector3D(0, 0, -7));
			  raytracer.scale(backWall, Point3D(0, 0, 0), wallScale);

			  raytracer.translate(leftWall, Vector3D(-7, 0, 0));
			  raytracer.rotate(leftWall, 'y', 90);
			  raytracer.scale(leftWall, Point3D(0, 0, 0), wallScale);

			  raytracer.translate(rightWall, Vector3D(7, 0, 0));
			  raytracer.rotate(rightWall, 'y', -90);
			  raytracer.scale(rightWall, Point3D(0, 0, 0), wallScale);

			  raytracer.translate(ceiling, Vector3D(0, 7, 0));
			  raytracer.rotate(ceiling, 'x', 90);
			  raytracer.scale(ceiling, Point3D(0, 0, 0), wallScale);

			  double cylinderScale[3] = { 1.5, 2.0, 1.5 };
			  raytracer.translate(cylinder, Vector3D(-4, -2, -4));
			  raytracer.scale(cylinder, Point3D(0, 0, 0), cylinderScale);

			  double sphereScale[3] = { 2.0, 2.0, 2.0 };
			  raytracer.translate(earthSphere, Vector3D(3, 3, -3));
			  raytracer.rotate(earthSphere, 'y', -25);
			  raytracer.rotate(earthSphere, 'z', -23.5);
			  raytracer.scale(earthSphere, Point3D(0, 0, 0), sphereScale);

			  raytracer.translate(glassSphere, Vector3D(1, -0.9, -0.5));
			  raytracer.scale(glassSphere, Point3D(0, 0, 0), sphereScale);


			  raytracer.render(width, height, eye, view, up, fov, max_depth, samples, aperture, focalLength, WithSpecular, true, 2.2, "extended_raytracing_scene_1.bmp");
			  break;
	}
	}

	return 0;
}

