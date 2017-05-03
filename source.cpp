#define _CRT_SECURE_NO_WARNINGS
// Credit to www.eecs.oregonstate.edu/capstone/opengl/sample.cpp
// for help with smooth mouse scaling.

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h> // (or others, depending on the system in use)
#include <iostream>

#include <ctime>
#include <string>
#include <vector>
#include <set>

// GLM header file ***
#include "glm/glm.hpp"

using namespace std;
using namespace glm;

// Location of OFF file input ***
static const char* inputPath = "fandisk_noise.off";

// Output location
static const char* outputPath = "";


// Define constants for algorithm ***
static const int NF = 3;
static const int NV = 10;
static const float LAMBDA = 0.6307f;

// User defined sigma c (radius) and sigma s (standard deviation
// of intesity differneces between center and area) ***
static float sigC = 0.0f;
static float sigS = 0.0f;

// Position of light ***
vec3 light_pos = { 1.0f, 1.0f, 1.0f };
const GLfloat PURPLE[3] = { 0.5f, 0.0f, 1.0f };

const float ANGFACT = { 1. };
const float MINSCALE = { 0.00f };
const float SCLFACT = { 0.010f };
const float TFACT = { 0.30f };
bool fullscreen = false;
bool mouseDown = false;
int mouseMode = 0;
int	transformation = 0;
static int menuExit;
int drawType = 2;
int mousex, mousey = 0;
int win = 0;

void draw();
void menu(int);

// transformation variables
float xrot = 0.0f;
float yrot = 0.0f;

float scale = 1.0f;
float tx = 0.0f;
float ty = 0.0f;

float xdiff = 0.0f;
float ydiff = 0.0f;


enum transformation
{
	ROTATE,
	SCALE,
	TRANSLATE
};

// Updated Mesh object with glm::vec3 types ***
struct Mesh {
	int nv;
	int nf;
	int ne;
	vec3* vertex;
	ivec3* face;
	vec3* normal;
	vec3* centroid;
};

// List of vertex neighbors
static vector<set<int>> vertexNeighbors;
// #####
static vector<set<int>> faceNeighbors;

// TODO: Find best way to implement this
// List of edge neighbors ***
static pair<int,int>* edgeNeighbors;


// Spatial smoothing function ***
float smoothing_func(float x)
{
	return exp(-x*x / 2*sigC*sigC);
}

// Influence function ***
float influence_func(float x)
{
	return exp(-x*x / 2*sigS*sigS);
}

// Intensity difference between surface normals ***
float intensity_diff(vec3 u, vec3 v)
{
	return dot(v, v - u);
}

// Apply bilateral filter to smooth surface normals ***
void bilateral_filter(Mesh *mesh)
{
	// Create array to hold new normals
	vec3* newNormals = (vec3*) malloc(sizeof(vec3) * mesh->nf);

	for (int i = 0; i < mesh->nf; i++)
	{
		// Get normal and centroid of face i
		vec3 ni = mesh->normal[i];
		vec3 ci = mesh->centroid[i];

		vec3 topSum = { 0.0f, 0.0f, 0.0f };
		float normTerm = 0.0f;

		// Iterate through each neighbor of face i
		for (int j : vertexNeighbors[i])
		{
			// Get normal and centroid of neighbor j
			vec3 nj = mesh->normal[j];
			vec3 cj = mesh->centroid[j];

			// Calculate Ws and Wc
			float wc = smoothing_func(distance(cj, ci));
			float ws = influence_func(intensity_diff(ni, nj));

			// Sum up values
			topSum += (wc * ws * nj);
			normTerm += (wc * ws);
		}
		// Normalize smoothed normal
		vec3 smoothedNormal = topSum / normTerm;
		normalize(smoothedNormal);
		newNormals[i] = smoothedNormal;
	}

	// Copy new array of normals to mesh array of normals
	// TODO: Make sure this works
	memcpy(mesh->normal, newNormals, sizeof(vec3) * mesh->nf);

	// #####
	std::free(newNormals);
}

// Adjust vertices using Least Squares Error method ***
void lse_correction(Mesh *mesh)
{
	vec3* newVertices = (vec3*) malloc(sizeof(vec3) * mesh->nv);

	for (int i = 0; i < mesh->nv; i++)
	{

		// TODO: Implement LSE correction

	}

	// TODO: Make sure this works
	// Copy new vertices to mesh vertices
	//memcpy(mesh->vertex, newVertices, sizeof(vec3) * mesh->nv);
	
	// #####
	std::free(newVertices);
}

void smooth(Mesh *mesh) {
	// Smooth the surface normals ***
	for (int i = 0; i < NF; i++)
	{
		bilateral_filter(mesh);
	}
	// Adjust the vertices ***
	for (int i = 0; i < NV; i++)
	{
		lse_correction(mesh);
	}
}

// #####
// Add neighbors of each face within radius sigC
void calculateFaceNeighbors(Mesh *mesh, float sigC)
{
	for (int i = 0; i < mesh->nf; i++)
	{
		for (int j = 0; j < mesh->nf; j++)
		{

		}
	}
}

// TODO: Adds neighbors of each point ***
void addVertexNeighbors(Mesh *mesh, int a, int b, int c)
{
	vertexNeighbors[a].insert(b);
	vertexNeighbors[a].insert(c);

	vertexNeighbors[b].insert(a);
	vertexNeighbors[b].insert(c);

	vertexNeighbors[c].insert(a);
	vertexNeighbors[c].insert(b);
}

// Calculate surface normals with glm functions ***
vec3 addNormal(Mesh *mesh, int a, int b, int c)
{
	vec3 u = mesh->vertex[a] - mesh->vertex[b];
	vec3 v = mesh->vertex[c] - mesh->vertex[b];
	return normalize(cross(v, u));
}

// Calculate centroids of triangle face ***
vec3 addCentroid(Mesh *mesh, int a, int b, int c)
{
	return (mesh->vertex[a] + mesh->vertex[b] + mesh->vertex[c]) / 3.0f;
}

Mesh* readPolygon()
{
	printf("Reading mesh file...\n");
	int num, n, m;
	int a, b, c, d;
	float x, y, z;
	Mesh *surfmesh;
	char line[256];
	FILE *fin;

	if ((fin = fopen(inputPath, "r")) == NULL) {
		printf("read error...\n");
		getchar();
		exit(0);
	};

	/* OFF format */
	while (fgets(line, 256, fin) != NULL) {
		if (line[0] == 'O' && line[1] == 'F' && line[2] == 'F')
			break;
	}
	fscanf(fin, "%d %d %d\n", &m, &n, &num);

	// Read first line and save in Mesh
	surfmesh = (Mesh*)malloc(sizeof(Mesh));
	surfmesh->nv = m;
	surfmesh->nf = n;
	surfmesh->ne = num;

	// Allocate space for lists based on mesh size ***
	surfmesh->vertex = (vec3*)malloc(sizeof(vec3) * m);
	surfmesh->face = (ivec3*)malloc(sizeof(ivec3) * n);
	surfmesh->normal = (vec3*)malloc(sizeof(vec3) * n);
	surfmesh->centroid = (vec3*)malloc(sizeof(vec3) * n);

	// Allocate space for the neighborhoods
	// #####
	vertexNeighbors.resize(m);
	faceNeighbors.resize(n);

	// TODO: Figure out how to get neighboring faces of edges
	//edgeNeighbors = (pair<int,int>*) malloc(sizeof(pair<int, int>) * num);

	for (n = 0; n < surfmesh->nv; n++) {
		fscanf(fin, "%f %f %f\n", &x, &y, &z);
		surfmesh->vertex[n].x = x;
		surfmesh->vertex[n].y = y;
		surfmesh->vertex[n].z = z;
	}

	for (n = 0; n < surfmesh->nf; n++) {
		fscanf(fin, "%d %d %d %d\n", &a, &b, &c, &d);
		surfmesh->face[n].x = b;
		surfmesh->face[n].y = c;
		surfmesh->face[n].z = d;

		// Add neighbors for each vertex ***
		addVertexNeighbors(surfmesh, b, c, d);
		// Calculate face surface normal ***
		surfmesh->normal[n] = addNormal(surfmesh, b, c, d);
		// Calculate triangle centroid ***
		surfmesh->centroid[n] = addCentroid(surfmesh, b, c, d);

		if (a != 3)
			printf("Errors: reading surfmesh .... \n");
	}
	fclose(fin);

	printf("Done reading mesh file...\n");
	return surfmesh;
}


// Surface mesh obtained from .off file
Mesh* surfmesh = readPolygon();


void createMenu(void) {
	//////////
	// MENU //
	//////////

	// Create an entry
	menuExit = glutCreateMenu(menu);
	glutAddMenuEntry("Point", 1);
	glutAddMenuEntry("Fill", 2);
	glutAddMenuEntry("Line", 3);
	glutAddMenuEntry("Both", 4);
	glutAddMenuEntry("Exit Program", 0);

	// Let the menu respond on the right mouse button
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void menu(int value) {
	if (value == 0) {
		glutDestroyWindow(win);
		exit(0);
	}
	else {
		drawType = value;
	}

	// you would want to redraw now
	glutPostRedisplay();
}

void draw()
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// change mode depending on menu selection
	switch (drawType)
	{
	case 1:
		glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		break;
	case 2:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		break;
	case 3:
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		break;
	}
	
	for (int i = 0; i < surfmesh->nf; ++i) {
		// Use face's surface normal for lighting ***
		glNormal3f(surfmesh->normal[i].x, surfmesh->normal[i].y, surfmesh->normal[i].z);
		glBegin(GL_TRIANGLES);
			glVertex3f(surfmesh->vertex[surfmesh->face[i].x].x, surfmesh->vertex[surfmesh->face[i].x].y, surfmesh->vertex[surfmesh->face[i].x].z);
			glVertex3f(surfmesh->vertex[surfmesh->face[i].y].x, surfmesh->vertex[surfmesh->face[i].y].y, surfmesh->vertex[surfmesh->face[i].y].z);
			glVertex3f(surfmesh->vertex[surfmesh->face[i].z].x, surfmesh->vertex[surfmesh->face[i].z].y, surfmesh->vertex[surfmesh->face[i].z].z);
		glEnd();
	}

	// Both FILL and LINE 
	if (drawType == 4) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		GLfloat white[3] = { 1.0f, 1.0f, 1.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, white);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);
		for (int i = 0; i < surfmesh->nf; ++i) {
			glBegin(GL_TRIANGLES);
				glVertex3f(surfmesh->vertex[surfmesh->face[i].x].x, surfmesh->vertex[surfmesh->face[i].x].y, surfmesh->vertex[surfmesh->face[i].x].z);
				glVertex3f(surfmesh->vertex[surfmesh->face[i].y].x, surfmesh->vertex[surfmesh->face[i].y].y, surfmesh->vertex[surfmesh->face[i].y].z);
				glVertex3f(surfmesh->vertex[surfmesh->face[i].z].x, surfmesh->vertex[surfmesh->face[i].z].y, surfmesh->vertex[surfmesh->face[i].z].z);
			glEnd();
		}
		glDisable(GL_POLYGON_OFFSET_FILL);

	}
}

bool init()
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glClearDepth(1.0f);

	glEnable(GL_LIGHTING);

	return true;
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	// Adjust light position based on translate and scale ***
	GLfloat lp[3] = { scale * (light_pos.x + tx), scale * (light_pos.y + ty), scale * light_pos.z };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, PURPLE);
	glLightfv(GL_LIGHT0, GL_POSITION, lp);
	glEnable(GL_LIGHT0);

	// Change eye position for translation
	gluLookAt(
		tx, ty, 100.0f,
		tx, ty, 0.0f,
		0.0f, 1.0f, 0.0f);

	// Rotate with rotation vars
	glRotatef(xrot, 1.0f, 0.0f, 0.0f);
	glRotatef(yrot, 0.0f, 1.0f, 0.0f);

	// Scale with scale variables
	glScalef(scale, scale, scale);

	glPushMatrix();
	draw();
	glPopMatrix();

	glFlush();
	glutSwapBuffers();
}

void resize(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(45.0f, 1.0f * w / h, 1.0f, -100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'r':
		transformation = ROTATE;
		break;
	case 'R':
		transformation = ROTATE;
		break;
	case 's':
		transformation = SCALE;
		break;
	case 'S':
		transformation = SCALE;
		break;
	case 't':
		transformation = TRANSLATE;
		break;
	case 'T':
		transformation = TRANSLATE;
		break;
	case 27:
		exit(1);
		break;
	}
}

void specialKeyboard(int key, int x, int y)
{
	if (key == GLUT_KEY_F1)
	{
		fullscreen = !fullscreen;

		if (fullscreen)
			glutFullScreen();
		else
		{
			glutReshapeWindow(500, 500);
			glutPositionWindow(50, 50);
		}
	}
}

void mouse(int button, int state, int x, int y)
{

	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		mouseDown = true;
		mousex = x;
		mousey = y;
	}

	else
		mouseDown = false;
}

void mouseMotion(int x, int y)
{

	if (mouseDown)
	{
		int dx = x - mousex;
		int dy = y - mousey;

		switch (transformation)
		{
		case ROTATE:
			xrot += (ANGFACT*dy);
			yrot += (ANGFACT*dx);
			break;
		case SCALE:
			scale += SCLFACT * (float)(dx - dy);
			if (scale < MINSCALE)
				scale = MINSCALE;
			break;
		case TRANSLATE:
			tx -= dx * TFACT;
			ty += dy * TFACT;
			break;
		}

		mousex = x;
		mousey = y;
		glutPostRedisplay();
	}


}

// Writes mesh to a file specified by filepath
void writeMeshToFile(Mesh *mesh, const char *filepath)
{
	printf("Writing mesh to file...\n");
	FILE* file = fopen(filepath, "w");

	fprintf(file, "OFF\n");
	fprintf(file, "%d   %d   %d\n", mesh->nv, mesh->nf, mesh->ne);

	for (int i = 0; i < mesh->nv; i++)
	{
		fprintf(file, "%f   %f   %f\n", mesh->vertex[i].x, mesh->vertex[i].y, mesh->vertex[i].z);
	}
	for (int j = 0; j < mesh->nf; j++)
	{
		fprintf(file, "3 %d %d %d\n", mesh->face[j].x, mesh->face[j].y, mesh->face[j].z);
	}

	fclose(file);
	printf("Done writing mesh to file.\n");
}

void idle()
{
	if (!mouseDown)
	{
		xrot += 0.3f;
		yrot += 0.4f;
	}

	glutPostRedisplay();
}

int main(int argc, char *argv[])
{
	smooth(surfmesh);

	/*
	// Write the smoothed mesh to OFF file
	string meshPathOut(outputPath);
	meshPathOut.append("smooth" + to_string(ITERATIONS) + ".off");
	writeMeshToFile(surfmesh, meshPathOut.c_str());
	//*/

	glutInit(&argc, argv);

	glutInitWindowPosition(50, 50);
	glutInitWindowSize(500, 500);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	win = glutCreateWindow("3D Polygon");
	createMenu();

	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKeyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	glutReshapeFunc(resize);
	glutIdleFunc(idle);

	if (!init())
	{
		printf("Failed to initialize GLUT\n");
		getchar();
		return 1;
	}

	glutMainLoop();

	// #####
	std::free(surfmesh->vertex);
	std::free(surfmesh->face);
	std::free(surfmesh->normal);
	std::free(surfmesh->centroid);
	std::free(surfmesh);
	return 0;
}