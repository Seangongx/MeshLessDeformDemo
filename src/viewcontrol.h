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
	inline void addModel(Rawdata& d) {
		rawModels.push_back(d);
	}
	inline void addModel(DeformModel& d) {
		colors.emplace(viewer.data().id, 0.5 * Eigen::RowVector3d::Random().array() + 0.5);
		deformModels.push_back(d);
	}
	inline void removeModel(size_t id) {
		deformModels.erase(deformModels.begin() + id);
	}
	inline void launch() {
		viewer.launch();
	}
	void load();

	// get:
	inline size_t getRawSize() { return rawModels.size(); }
	inline size_t getDeformSize() { return deformModels.size(); }

	// set:

private:
	// Functions

	void initMenu();
	void initEvents();

	// Attributes

	std::vector<Rawdata> rawModels;
	std::vector<DeformModel> deformModels;
	// Colors only for deformModels
	std::map<int, Eigen::RowVector3d> colors;

	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiMenu menu;

};

