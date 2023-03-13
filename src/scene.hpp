#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/file_dialog_open.h>
#include <filesystem>							// c++17
#include "models/deformmodel.hpp"

struct gui_parameters {
	// Global Parameters for deformation(Modify):
	float Rb				= 0.1f;
	float beta				= 0.8f;
	float tau				= 0.8f;
	float forceAmp			= 10.f;		// force 
	float forcePick			= 15.f;		// pick for
	float dt				= 0.01f;	// time step
	float perturbation		= 0.1f;

	// Visualization control(Control):
	MODE mode				= LINEAR;
	bool showTriangles		= false;
	bool showForces			= false;
	bool showBoundingbox	= false;
	bool pauseAnimation		= false;
	bool setGravity			= false;

	// Scene statistics(Display):
	unsigned int meshes		= 0;
	unsigned int fps		= 0;
	unsigned int vertices	= 0;
	unsigned int triangles	= 0;
	unsigned int memory		= 0;

};


struct pick_t {
	int				mouseX, mouseY;
	bool			picked	=	false;
	DeformModel*	object;

	unsigned int	fid		=	0;		// Libigl: face id
	unsigned int	vi		=	0;		// Libigl: vertex index
	Eigen::Vector3f bc;					// barycentric coordinate
};

enum DIRECTION
{
	NONE,
	UP,
	DOWN,
	LEFT,
	RIGHT,
	FORWARD,
	BACKWARD
};


struct scene_structure {

	// ****************************** //
	// Attributes
	// ****************************** //

	std::vector<RawModel>		rawModels;
	std::vector<DeformModel>	models;
	gui_parameters				params;
	pick_t						pick;
	DIRECTION					inputForce = NONE;

	igl::opengl::glfw::Viewer	viewer;
	igl::opengl::glfw::imgui::ImGuiMenu menu;

	// ****************************** //
	// interfaces
	// ****************************** //

	void initialize();


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

	void clearAllFixedPoints();


	// ****************************** //
	// functions (call from inside)
	// ****************************** //

private:
	void initScene();
	void initMenu();
	void initEvents();

	void primitive_cube(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

};

