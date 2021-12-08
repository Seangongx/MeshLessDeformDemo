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
	bool show_triangles = false;
	bool pause = false;
	// Scene statistics:
	unsigned int meshes = 0;
	unsigned int fps = 0;
	unsigned int vertices = 0;
	unsigned int triangles = 0;
	unsigned int memory = 0;
};


struct Pick
{
	bool picked = false;
	DeformModel* object;
	int mouseX, mouseY;
	unsigned int fid = 0;
	unsigned int vi = 0;
	// barycentric coordinate
	Eigen::Vector3f bc; 
};


class ViewControl
{
public:
	ViewControl();

	// Interfaces:
	inline void addModel(Rawdata& d) {
		rawModels.push_back(d);
	}
	inline void addModel(DeformModel& d) {
		models.push_back(d);
	}
	inline void removeModel(size_t id) {
		models.erase(models.begin() + id);
	}
	inline void launch() {
		viewer.launch();
	}
	void load();

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

	std::vector<Rawdata> rawModels;
	std::vector<DeformModel> models;
	DeformParameters params;
	Pick pick;

	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiMenu menu;

};

