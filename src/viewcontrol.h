#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/file_dialog_open.h>
#include <filesystem> // c++17
#include "deformmodel.h"

struct Rawdata
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

};

class ViewControl
{
public:
	ViewControl();

	// Interfaces:
	inline void addModel(Rawdata& d) { rawModels.push_back(d); }
	inline void addModel(DeformModel& d) { deformModels.push_back(d); }
	inline void deleteModel(size_t id) { deformModels.erase(deformModels.begin() + id); }
	void load();

	// get:
	inline size_t geRawSize() { return rawModels.size(); }
	inline size_t geDeformSize() { return deformModels.size(); }

	// set:

private:
	// Functions
	void initMenu();

	// Attributes
	std::vector<Rawdata> rawModels;
	std::vector<DeformModel> deformModels;
	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiMenu menu;

};

