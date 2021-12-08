#include "viewcontrol.h"
#include <functional> // for bind functions
#include <chrono>
#include <igl/unproject.h>
#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/unproject.h>

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

	for (size_t t = 0; t < models.size(); t++) {

#ifdef DEBUG
        std::cout << "model " << t << ":";
        std::cout << "( " << models[t].V().rows() << " vertices, " << models[t].F().rows() << " triangles )" << std::endl;
#endif // DEBUG

		viewer.data().set_mesh(models[t].V(), models[t].F());
        viewer.append_mesh();
        if(!models[t].hasTEX())
            viewer.data(t).set_colors(models[t].getColor());
        viewer.data().show_lines = false;
        //viewer.data().double_sided = true;
	}
    //viewer.data().set_face_based(true);
};


void ViewControl::initMenu()
{
    DeformParameters params;

	viewer.plugins.push_back(&menu);

    menu.callback_draw_viewer_window = [&]() {
        // Define menu position & size
        ImVec2 menuPos(ImGui::GetMainViewport()->Pos.x + 20, ImGui::GetMainViewport()->Pos.y + 20);
        ImVec2 menuSize(300, 600);
        ImGui::SetNextWindowPos(menuPos, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(menuSize, ImGuiCond_FirstUseEver);
        ImGui::Begin("Meshless Deformations Based on Shape Matching demo_GONG Xun", nullptr, ImGuiWindowFlags_NoSavedSettings);


        if (ImGui::CollapsingHeader("ModelData", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Load OBJ..."))
            {
                std::string const filename = igl::file_dialog_open();
                std::filesystem::path const loadModel(filename);
                if (std::filesystem::exists(loadModel) && std::filesystem::is_regular_file(loadModel))
                {
                    DeformModel openModel(filename, getDeformSize());
                    openModel.translate(Eigen::RowVector3d(2.5 * Eigen::RowVector3d::Random().array() + 2.5));
                    addModel(openModel);
                    int tid = models.size() - 1;
                    viewer.data().clear();
                    viewer.data().set_mesh(models[tid].V(), models[tid].F());
                    viewer.append_mesh();
                    viewer.data(tid).set_colors(models[tid].getColor());
                    viewer.core().align_camera_center(models[tid].V(), models[tid].F());

                }
                else {
                    std::cerr << "ERROR: FAIL TO LOAD MODEL" << std::endl;
                }
            }
        }

        if (ImGui::CollapsingHeader("Statistics", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Text("Meshs: %d", params.meshes);
            ImGui::Text("FPS: %d", params.fps);
            ImGui::Text("Vertices: %d", params.vertices);
            ImGui::Text("Triangles: %d", params.triangles);
            ImGui::Text("Memory cost: %d(Kb)", params.memory / 1024);
        }

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

        ImGui::End();

    };

    viewer.core().is_animating = true;

}

void ViewControl::initEvents()
{
    // Switch models between 0~9
    viewer.callback_key_down = [this](igl::opengl::glfw::Viewer& viewer, int key, int modifier) -> bool {
        std::cout << "Key: " << key << " " << (unsigned char)key << std::endl;
        int id = key - '0';
        if (id < 10 && id > -1) {
            std::cout << "Model (" << id << ") is desplayed\n";
            viewer.core().align_camera_center(models[id].V(), models[id].F());
            return true;
        }

        return false;
    };


    // Mouse click for picking

    viewer.callback_mouse_down =
        [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        using button_type = igl::opengl::glfw::Viewer::MouseButton;
        if (static_cast<button_type>(button) != button_type::Left)
            return false;

        // fixing some points
        if (modifier == GLFW_MOD_SHIFT)
        {
            double const x = static_cast<double>(viewer.current_mouse_x);
            double const y = viewer.core().viewport(3) - static_cast<double>(viewer.current_mouse_y);
            int fid;
            Eigen::Vector3f bc;

#ifdef DEBUG
            std::cout << "SHIFT DOWN:\n";
            std::cout << "In screen ( x: " << x << ", y:" << y << " )\n";
#endif
            for (int i = 0; i < models.size(); i++)
            {
                bool const hit = igl::unproject_onto_mesh(
                    Eigen::Vector2f(x, y),
                    viewer.core().view,
                    viewer.core().proj,
                    viewer.core().viewport,
                    models[i].V(),
                    models[i].F(),
                    fid,
                    bc);

                if (hit) {
                    auto const& F = models[i].F();
                    Eigen::Vector3i const face(F(fid, 0), F(fid, 1), F(fid, 2));

                    unsigned int closest_vertex = face(0);
                    if (bc(1) > bc(0) && bc(1) > bc(2))
                        closest_vertex = face(1);
                    else if (bc(2) > bc(0) && bc(2) > bc(1))
                        closest_vertex = face(2);

                    models[i].setFixed(closest_vertex, !models[i].isFixed(closest_vertex));

#ifdef DEBUG
                    std::cout << "---The picked fixed vertex\n" << models[i].V().row(closest_vertex) << "\n---has been hitted\n" << "\n";
#endif
                }
            }

            return true;
        }

        // picking some points
        if (modifier == GLFW_MOD_CONTROL)
        {
            double const x = static_cast<double>(viewer.current_mouse_x);
            double const y = viewer.core().viewport(3) - static_cast<double>(viewer.current_mouse_y);
            int fid;
            Eigen::Vector3f bc;

#ifdef DEBUG
            std::cout << "CONTROL DOWN:\n";
            std::cout << "In screen ( x: " << x << ", y:"  << y << " )\n";
#endif

            for (int i = 0; i < models.size(); i++)
            {
                bool const hit = igl::unproject_onto_mesh(
                    Eigen::Vector2f(x, y),
                    viewer.core().view,
                    viewer.core().proj,
                    viewer.core().viewport,
                    models[i].V(),
                    models[i].F(),
                    fid,
                    bc);

                if (hit)
                {
                    pick.picked = true;
                    pick.fid = fid;
                    pick.object = &models[i];
                    pick.bc = bc;

                    pick.mouseX = viewer.current_mouse_x;
                    pick.mouseY = viewer.current_mouse_y;

                    auto const& F = pick.object->F();
                    Eigen::Vector3i const face(F(pick.fid, 0), F(pick.fid, 1), F(pick.fid, 2));

                    unsigned int closest_vertex = face(0);
                    if (pick.bc(1) > pick.bc(0) && pick.bc(1) > pick.bc(2))
                        closest_vertex = face(1);
                    else if (pick.bc(2) > pick.bc(0) && pick.bc(2) > pick.bc(1))
                        closest_vertex = face(2);

                    pick.vi = closest_vertex;
#ifdef DEBUG
                    std::cout << "---The picked move vertex\n" << models[i].V().row(closest_vertex) << "\n---has been hitted\n" << "\n";
#endif
                }
            }
            return true;
        }

        return false;
    };

    viewer.callback_mouse_up =
        [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        using button_type = igl::opengl::glfw::Viewer::MouseButton;
        if (static_cast<button_type>(button) != button_type::Left)
            return false;
        if (pick.picked) 
            pick.picked = false;
        return true;
    };

    viewer.callback_pre_draw = [this](igl::opengl::glfw::Viewer& viewer) -> bool {
        auto const begin = std::chrono::high_resolution_clock::now();

        if (!(models.size() > 0))
            return false;

        if (pick.picked)
        {
            // Eigen::Vector3d picked_vertex_position{
            //     pick.object->V()(pick.vi, 0),
            //     pick.object->V()(pick.vi, 1),
            //     pick.object->V()(pick.vi, 2)};

            double const x1 = static_cast<double>(pick.mouseX);
            double const y1 = viewer.core().viewport(3) - static_cast<double>(pick.mouseY);

            double const x2 = static_cast<double>(viewer.current_mouse_x);
            double const y2 = viewer.core().viewport(3) - static_cast<double>(viewer.current_mouse_y);

            Eigen::Vector3d const p1 = igl::unproject(
                Eigen::Vector3f(x1, y1, .5f),
                viewer.core().view,
                viewer.core().proj,
                viewer.core().viewport)
                .cast<double>();
            Eigen::Vector3d const p2 = igl::unproject(
                Eigen::Vector3f(x2, y2, .5f),
                viewer.core().view,
                viewer.core().proj,
                viewer.core().viewport)
                .cast<double>();

            Eigen::Vector3d d = (p2 - p1).normalized();
            //object.forces().row(pick.vi) += d * static_cast<double>(params.pick_force);

            pick.mouseX = viewer.current_mouse_x;
            pick.mouseY = viewer.current_mouse_y;
        }

        if (!params.pause)
        {
            //simulate();
        }

        // draw fixed points
        viewer.data().clear_points();
        for (auto m : models) {
            Eigen::MatrixX3d tempVertex(m.countFixed(), 3);
            Eigen::MatrixX3d tempColor(m.countFixed(), 3);
            for (unsigned int i = 0, j = 0; i < m.V().rows(); ++i)
            {
                if (!m.isFixed(i))
                    continue;
                tempVertex.block(j, 0, 1, 3) = m.V().block(i, 0, 1, 3);
                tempColor.block(j, 0, 1, 3) = Eigen::RowVector3d(128., 128., 0.);
                ++j;
            }
            viewer.data().add_points(tempVertex, tempColor);
        }



        auto const end = std::chrono::high_resolution_clock::now();
        auto const period_in_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
        params.fps = 1'000'000'000 / period_in_nanoseconds;

        // statistics:
        for (int i = 0; i < models.size(); i++)
        {
            params.vertices += models[i].V().rows();
            params.triangles += models[i].F().rows();
            params.memory += models[i].getMemoryBytes();
        }

        return false;
    };


}


void ViewControl::clearAllFixedPoints()
{
    for (auto m :models) {
        m.clearFixed();
    }
}

