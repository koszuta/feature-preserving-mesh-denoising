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
#include <unordered_set>
#include <algorithm>
#include <climits>
#include <chrono>
#include <fstream>
#include <thread>

// GLM header file ***
#include <glm/glm.hpp>

using namespace std;

// Location of OFF file input ***
static const char* inputPath = "fandisk_noise.off";

// Output location
static const char* outputPath = "";


// Define constants for algorithm ***
static const int NF = 3;
static const int NV = 10;
static const float LAMBDA = 0.01f;// 0.6307f;

// User defined sigma c (radius) and sigma s (standard deviation
// of intesity differneces between center and area) ***
static float sigC = 0.0f;
static float sigS = 0.0f;

static int userFace = -1;
static unordered_set<int> facesWithinRadius;

// Position of light ***
glm::vec3 light_pos = { 1.0f, 1.0f, 1.0f };
const GLfloat PURPLE[3] = { 0.5f, 0.0f, 1.0f };
const GLfloat WHITE[3] = { 1.0f, 1.0f, 1.0f };
const GLfloat BLACK[3] = { 0.0f, 0.0f, 0.0f };

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
	glm::vec3* vertex;
	glm::ivec3* face;
	glm::vec3* normal;
	glm::vec3* centroid;
};

// List of vertex neighbors
static vector<set<int>> neighborVertices;
static vector<set<int>> neighborFaces;

static vector<set<int>> BF_faceNeighbors;


// Intensity difference between surface normals ***
float intensity_diff(glm::vec3 u, glm::vec3 v)
{
	return glm::dot(u, u - v);
}

// Get user defined center face and radius
void getRadius(Mesh* mesh)
{
	string temp = "";
	while (strcmp(temp.c_str(), "y"))
	{
		temp = "";
		cout << "Enter face at center of area expected to be smooth, in range [0, " << mesh->nf << ")" << endl;

		int face;
		cin.clear();
		if (!(cin >> face) || 0 > face || face > mesh->nf-1)
		{
			cout << "OUT OF RANGE" << endl;
			cin.clear();
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			continue;
		}

		userFace = face;
		cout << "Use face " << face << "? (y/n)" << endl;
		cin >> temp;
	}

	temp = "";
	while (strcmp(temp.c_str(), "y"))
	{
		temp = "";
		cout << "Enter radius of the smooth area..." << endl;
		facesWithinRadius.clear();

		float radius;
		cin.clear();
		if (!(cin >> radius) || radius <= 0)
		{
			cout << "RADIUS MUST BE > 0" << endl;
			cin.clear();
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			continue;
		}
		sigC = radius;
		for (int i = 0; i < mesh->nf; i++)
		{
			if (i != userFace && sigC > glm::distance(mesh->centroid[i], mesh->centroid[userFace]))
			{
				facesWithinRadius.insert(i);
			}
		}
		cout << "Use radius " << radius << "? (y/n)" << endl;
		cin >> temp;
	}

	printf("\nface = %d, radius = %f\n", userFace, sigC);

	vector<int> toVisit;
	unordered_set<int> visitedVertices;
	glm::ivec3 fCenter = mesh->face[userFace];
	glm::vec3 nCenter = mesh->normal[userFace];
	glm::vec3 cCenter = mesh->centroid[userFace];
	for (int v = 0; v < fCenter.length(); v++)
	{
		visitedVertices.insert(fCenter[v]);
		for (int f : neighborFaces[fCenter[v]])
		{
			if (f != userFace && find(toVisit.begin(), toVisit.end(), f) == toVisit.end())
			{
				toVisit.push_back(f);
			}
		}
	}

	int N = 0;
	float sum = 0.0f;

	for (int j = 0; j < toVisit.size(); j++)
	{
		glm::ivec3 fj = mesh->face[toVisit[j]];

		// Get normal and centroid of face j
		glm::vec3 nj = mesh->normal[toVisit[j]];
		glm::vec3 cj = mesh->centroid[toVisit[j]];
		float distance = glm::distance(cj, cCenter);
		if (distance < sigC)
		{
			for (int v = 0; v < fj.length(); v++)
			{
				if (visitedVertices.insert(fj[v]).second)
				{
					for (int f : neighborFaces[fj[v]])
					{
						if (f != userFace && find(toVisit.begin(), toVisit.end(), f) == toVisit.end())
						{
							toVisit.push_back(f);
						}
					}
				}
			}
			
			// Add intensity difference between normal j and center
			sum += pow(intensity_diff(nCenter, nj), 2);
			N++;
		}
	}
	sigS = sqrt(sum / N);
	printf("standard deviation = %f\n", sigS);
}

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

// Apply bilateral filter to smooth surface normals ***
void bilateral_filter(Mesh *mesh)
{
	// Create array to hold new normals
	glm::vec3* newNormals = (glm::vec3*) malloc(sizeof(glm::vec3) * mesh->nf);

	for (int i = 0; i < mesh->nf; i++)
	{
		// Get normal and centroid of face i
		glm::vec3 ni = mesh->normal[i];
		glm::vec3 ci = mesh->centroid[i];

		vector<int> toVisit;
		unordered_set<int> visitedVertices;
		glm::ivec3 fi = mesh->face[i];
		for (int v = 0; v < fi.length(); v++)
		{
			visitedVertices.insert(fi[v]);
			for (int f : neighborFaces[fi[v]])
			{
				if (f != i && find(toVisit.begin(), toVisit.end(), f) == toVisit.end())
				{
					toVisit.push_back(f);
				}
			}
		}

		glm::vec3 top(0.0f);
		float bottom = 0.0f;

		for (int j = 0; j < toVisit.size(); j++)
		{
			glm::ivec3 fj = mesh->face[toVisit[j]];

			// Get normal and centroid of face j
			glm::vec3 nj = mesh->normal[toVisit[j]];
			glm::vec3 cj = mesh->centroid[toVisit[j]];
			float distance = glm::distance(cj, ci);
			if (distance < ceil(2 * sigC))
			{
				for (int v = 0; v < fj.length(); v++)
				{
					if (visitedVertices.insert(fj[v]).second)
					{
						for (int f : neighborFaces[fj[v]])
						{
							if (f != i && find(toVisit.begin(), toVisit.end(), f) == toVisit.end())
							{
								toVisit.push_back(f);
							}
						}
					}
				}

				// Calculate Ws and Wc
				float wc = smoothing_func(distance);
				float ws = influence_func(intensity_diff(ni, nj));

				top += wc * ws * nj;
				bottom += wc * ws;
			}
		}

		// Normalize smoothed normal
		glm::vec3 smoothedNormal = top / bottom;
		smoothedNormal = glm::normalize(smoothedNormal);
		newNormals[i] = smoothedNormal;
	}

	// Copy new array of normals to mesh array of normals
	// TODO: Make sure this works
	printf("BF: Copying new normals to mesh\n");
	memcpy(mesh->normal, newNormals, sizeof(glm::vec3) * mesh->nf);
	free(newNormals);
}

// Adjust vertices using Least Squares Error method ***
void lse_correction(Mesh *mesh)
{
	glm::vec3* newVertices = (glm::vec3*) malloc(sizeof(glm::vec3) * mesh->nv);

	for (int i = 0; i < mesh->nv; i++)
	{
		//printf("LSE: Vertex %d\n", i);
		glm::vec3 sum;
		for (int j : neighborVertices[i])
		{
			vector<int> edgeNeighbors(2);
			set_intersection(neighborFaces[i].begin(), neighborFaces[i].end(), neighborFaces[j].begin(), neighborFaces[j].end(), edgeNeighbors.begin());
			edgeNeighbors.shrink_to_fit();
			if (edgeNeighbors.size() != 2)
			{
				printf("Got %d edge neighbors, should be 2\n", edgeNeighbors.size());
			}
			glm::vec3 sum2;
			for (int f : edgeNeighbors)
			{
				sum2 += outerProduct(mesh->normal[f], mesh->normal[f]) * (mesh->vertex[j] - mesh->vertex[i]);
			}
			sum += sum2;
		}
		newVertices[i] = mesh->vertex[i] + LAMBDA * sum;
	}
	// TODO: Make sure this works
	// Copy new vertices to mesh vertices
	printf("LSE: Copying new vertices to mesh\n");
	memcpy(mesh->vertex, newVertices, sizeof(glm::vec3) * mesh->nv);
	free(newVertices);
}

void smooth(Mesh *mesh) {
	chrono::time_point<chrono::system_clock> start, end;
	chrono::duration<double> duration;
	// Smooth the surface normals ***
	for (int i = 0; i < NF; i++)
	{
		start = chrono::system_clock::now();

		bilateral_filter(mesh);

		end = chrono::system_clock::now();
		duration = end - start;
		printf("BF duration = %f\n", duration);
	}
	// Adjust the vertices ***
	for (int i = 0; i < NV; i++)
	{
		start = chrono::system_clock::now();

		lse_correction(mesh);

		end = chrono::system_clock::now();
		duration = end - start;
		printf("LSE duration = %f\n", duration);
	}
}

// TODO: Adds neighbors of each point ***
void addVertexNeighbors(int a, int b, int c)
{
	neighborVertices[a].insert(b);
	neighborVertices[a].insert(c);

	neighborVertices[b].insert(a);
	neighborVertices[b].insert(c);

	neighborVertices[c].insert(a);
	neighborVertices[c].insert(b);
}

// Add neighboring face normal to vertex
void addNeighborNormals(int n, int a, int b, int c)
{
	neighborFaces[a].insert(n);
	neighborFaces[b].insert(n);
	neighborFaces[c].insert(n);
}

// Calculate surface normals with glm functions ***
glm::vec3 addNormal(Mesh *mesh, int a, int b, int c)
{
	glm::vec3 u = mesh->vertex[a] - mesh->vertex[b];
	glm::vec3 v = mesh->vertex[c] - mesh->vertex[b];
	return glm::normalize(glm::cross(v, u));
}

// Calculate centroids of triangle face ***
glm::vec3 addCentroid(Mesh *mesh, int a, int b, int c)
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
	surfmesh = (Mesh*) malloc(sizeof(Mesh));
	surfmesh->nv = m;
	surfmesh->nf = n;
	surfmesh->ne = num;

	// Allocate space for lists based on mesh size ***
	surfmesh->vertex = (glm::vec3*)malloc(sizeof(glm::vec3) * surfmesh->nv);
	surfmesh->face = (glm::ivec3*)malloc(sizeof(glm::ivec3) * surfmesh->nf);
	surfmesh->normal = (glm::vec3*)malloc(sizeof(glm::vec3) * surfmesh->nf);
	surfmesh->centroid = (glm::vec3*)malloc(sizeof(glm::vec3) * surfmesh->nf);

	// Allocate space for the neighborhoods
	neighborVertices.resize(surfmesh->nv);
	neighborFaces.resize(surfmesh->nv);

	// TODO: Figure out how to get neighboring faces of edges
	BF_faceNeighbors.resize(surfmesh->nf);

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
		addVertexNeighbors(b, c, d);
		// Add normal neighbors for each vertex
		addNeighborNormals(n, b, c, d);
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
		if (i == userFace)
		{
			const GLfloat HIGHLIGHT[3] = { 1.0f - PURPLE[0], 1.0f - PURPLE[1], 1.0f - PURPLE[2] };
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, HIGHLIGHT);
		}
		else if (facesWithinRadius.count(i) != 0)
		{
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, WHITE);
		}
		else
		{
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, PURPLE);
		}
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

void doSmooth()
{
	getRadius(surfmesh);

	chrono::time_point<chrono::system_clock> start = chrono::system_clock::now();

	smooth(surfmesh);

	chrono::time_point<chrono::system_clock> end = chrono::system_clock::now();
	chrono::duration<double> duration2 = end - start;
	printf("Total duration = %f\n", duration2);

	//*
	// Write the smoothed mesh to OFF file
	string meshPathOut(outputPath);
	meshPathOut.append("smooth.off");
	writeMeshToFile(surfmesh, meshPathOut.c_str());
	//*/
}

int main(int argc, char *argv[])
{
	//thread t1(doSmooth);
	doSmooth();

	glutInit(&argc, argv);

	glutInitWindowPosition(50, 50);
	glutInitWindowSize(500, 500);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	win = glutCreateWindow("Feature Preserving Mesh Denoising");
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

	std::free(surfmesh->vertex);
	std::free(surfmesh->face);
	std::free(surfmesh->normal);
	std::free(surfmesh->centroid);
	std::free(surfmesh);

	//t1.join();
	return 0;
}