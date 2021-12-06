#include "viewcontrol.h"
#include <functional> // for bind functions


ViewControl::ViewControl() 
{
	initMenu();
    initEvents();

}

void ViewControl::load() {

    viewer.data().clear();

    for (size_t t = 0; t < rawModels.size(); t++) {
        viewer.data().set_mesh(rawModels[t].V, rawModels[t].F);
        viewer.append_mesh();
    }

	for (size_t t = 0; t < deformModels.size(); t++) {
		viewer.data().set_mesh(deformModels[t].V(), deformModels[t].F());
        viewer.append_mesh();
	}
	viewer.data().set_face_based(true);

};


void ViewControl::initMenu()
{
	viewer.plugins.push_back(&menu);

    //menu.callback_draw_viewer_window = [&]() {
    //    // Define next window position + size
    //    //ImGui::SetNextWindowPos(ImVec2(10.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
    //    //ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_FirstUseEver);
    //    ImGui::Begin("Meshless Deformations Based on Shape Matching demo", nullptr, ImGuiWindowFlags_NoSavedSettings);

    //    bool dirty = false;

    //    if (ImGui::CollapsingHeader("ModelData", ImGuiTreeNodeFlags_DefaultOpen))
    //    {
    //        if (ImGui::Button("Load OBJ..."))
    //        {
    //            std::string const filename = igl::file_dialog_open();
    //            std::filesystem::path const loadModel(filename);
    //            if (std::filesystem::exists(loadModel) && std::filesystem::is_regular_file(loadModel))
    //            {
    //                ;
    //            }
    //            else {
    //                std::cerr << "ERROR: FAIL TO LOAD MODEL" << std::endl;
    //            }
    //        }
    //    }

    //    // Expose the same variable directly ...
    //    if (ImGui::CollapsingHeader("Integration", ImGuiTreeNodeFlags_DefaultOpen))
    //    {
    //        ;
    //    }

    //    if (ImGui::CollapsingHeader("Deformation types", ImGuiTreeNodeFlags_DefaultOpen))
    //    {
    //        ;
    //    }

    //    if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen))
    //    {
    //        ;
    //    }

    //    if (ImGui::CollapsingHeader("Data", ImGuiTreeNodeFlags_DefaultOpen))
    //    {
    //        ;
    //    }

    //    if (dirty)
    //    {
    //        ;
    //    }

    //    ImGui::End();
    //};

}

void ViewControl::initEvents()
{
    viewer.callback_key_down = [this](igl::opengl::glfw::Viewer& viewer, int key, int modifier) -> bool {
        std::cout << "Key: " << key << " " << (unsigned int)key << std::endl;
        size_t id = key - '0';
        viewer.data().clear();
        viewer.data().set_mesh(deformModels[id].V(), deformModels[id].F());
        viewer.core().align_camera_center(deformModels[id].V(), deformModels[id].F());
        viewer.data().set_colors(colors[id]);

        return true;
    };
}


