#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/file_dialog_open.h>
#include <filesystem> // c++17
#include "deformmodel.h"

#define DEBUG

struct DeformParameters
{
	// Scene statistics(Display):
	unsigned int meshes = 0;
	unsigned int fps = 0;
	unsigned int vertices = 0;
	unsigned int triangles = 0;
	unsigned int memory = 0;

	// Visualization control(Control):
	bool showTriangles = false;
	bool showForces = false;
	bool showBoundingbox = false;
	bool pauseAnimation = false;
	DeformMode mode = LINEAR;
	bool setGravity = false;

	// Global Parameters for deformation(Modify):
	float Rb = 0.1f;
	float beta = 0.8f;
	float tau = 0.8f;
	float forceAmp = 10.f; // force 
	float forcePick = 15.f; // pick for
	float dt = 0.01f; // time step
	float perturbation = 0.1f;
};


struct Pick
{
	bool picked = false;
	DeformModel* object;
	int mouseX, mouseY;
	// parameters in Libigl:
	unsigned int fid = 0;
	unsigned int vi = 0;
	// barycentric coordinate
	Eigen::Vector3f bc;
};

enum ForceDir
{
	NONE,
	UP,
	DOWN,
	LEFT,
	RIGHT,
	FORWARD,
	BACKWARD
};

class ViewControl
{
public:
	ViewControl();

	// Interfaces:
	//inline void addModel(Rawdata& d) { rawModels.push_back(d); }
	inline void addModel(RawModel& d) { rawModels.push_back(d); }
	inline void addModel(DeformModel& d) { models.push_back(d); }
	inline void removeModel(size_t id) { models.erase(models.begin() + id); }
	inline void launch() { viewer.launch(); }

	// Control
	void load();
	void simulate();

	// get:
	inline size_t getRawSize() { return rawModels.size(); }
	inline size_t getDeformSize() { return models.size(); }

	// set:
	void clearAllFixedPoints();

private:
	// Functions

	void initMenu();
	void initEvents();

	// Attributes

	std::vector<RawModel> rawModels;
	std::vector<DeformModel> models;
	DeformParameters params;
	Pick pick;
	ForceDir inputForce = NONE;

	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiMenu menu;

};

