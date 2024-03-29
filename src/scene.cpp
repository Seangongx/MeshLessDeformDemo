#include "scene.hpp"
#include <functional> // for bind functions
#include <chrono>
#include <igl/unproject.h>
#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/unproject.h>

void scene_structure::initialize() 
{
    initScene();
	initMenu();
    initEvents();
}

void scene_structure::load() {

    viewer.data().clear();
    //viewer.data().double_sided = true;

    for (size_t t = 0; t < rawModels.size(); t++) {
        viewer.data().show_lines = false;
        viewer.data().set_mesh(rawModels[t].V(), rawModels[t].F());
        viewer.append_mesh();
        if (!rawModels[t].hasTEX())
            viewer.data(t).set_colors(rawModels[t].getColor());
    }

	for (size_t t = 0; t < models.size(); t++) {
        viewer.data().show_lines = false;
#ifdef DEBUG
        std::cout << "model " << t << ":";
        std::cout << "( " << models[t].V().rows() << " vertices, " << models[t].F().rows() << " triangles )" << std::endl;
#endif // DEBUG

		viewer.data().set_mesh(models[t].V(), models[t].F());
        viewer.append_mesh();
        if(!models[t].hasTEX())
            viewer.data(t).set_colors(models[t].getColor());
	}
    //viewer.data().set_face_based(true);
};

void scene_structure::initMenu()
{

	viewer.plugins.push_back(&menu);

    menu.callback_draw_viewer_window = [&]() {
        // Define menu position & size
        ImVec2 menuSize(300, 600);
        ImVec2 menuPos(ImGui::GetMainViewport()->Size.x - 20 - 300, ImGui::GetMainViewport()->Pos.y + 20);
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
                    //openModel.translate(Eigen::RowVector3d(2.5 * Eigen::RowVector3d::Random().array() + 2.5));
                    addModel(openModel);
                    int tid = models.size() - 1;
                    viewer.data().clear();
                    viewer.data().set_mesh(models[tid].V(), models[tid].F());
                    viewer.append_mesh();
                    viewer.data(tid).set_colors(models[tid].getColor());
                    //viewer.core().align_camera_center(models[tid].V(), models[tid].F());
                }
                else {
                    std::cerr << "ERROR: FAIL TO LOAD MODEL" << std::endl;
                }
            }
        }

        if (ImGui::CollapsingHeader("Integration", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::SliderFloat("Velocity Damping", &params.Rb, 0.f, 10.f);
            ImGui::SliderFloat("Force Amplitude", &params.forceAmp, 0.f, 1000.f);
            ImGui::SliderFloat("Picking Force", &params.forcePick, 1.f, 100.f);
            ImGui::SliderFloat("Tau", &params.tau, 0.f, 1.f);
            ImGui::SliderFloat("Beta", &params.beta, 0.f, 1.f);
            ImGui::SliderFloat("Regularization Perturbation", &params.perturbation, 0.f, 0.1f);
            ImGui::SliderFloat("Time step", &params.dt, 0.001f, 1.0f);
            ImGui::Checkbox("Activate gravity", &params.setGravity);
            ImGui::Checkbox("Pause Animation", &params.pauseAnimation);
        }

        if (ImGui::CollapsingHeader("Deformation Mode", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::RadioButton("Rigid", (int*)(&params.mode), 3))
            {
                params.mode = RIGID;
            }
            if (ImGui::RadioButton("Linear", (int*)(&params.mode), 0))
            {
                params.mode = LINEAR;
            }
            if (ImGui::RadioButton("Quadratic", (int*)(&params.mode), 1))
            {
                params.mode = QUADRATIC;
            }
            //if (ImGui::RadioButton("Plastic", (int*)(&params.mode), 2))
            //{
            //    ;
            //}
        }

        if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Checkbox("Show force field", &params.showForces);
            ImGui::Checkbox("Show triangles", &params.showTriangles);
            ImGui::Checkbox("Show bounding box", &params.showBoundingbox);
        }

        if (ImGui::CollapsingHeader("Statistics", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Text("Meshs: %d", params.meshes);
            ImGui::Text("FPS: %d", params.fps);
            ImGui::Text("Vertices: %d", params.vertices);
            ImGui::Text("Triangles: %d", params.triangles);
            ImGui::Text("Memory cost: %d(Kb)", params.memory / 1024);
        }

        ImGui::End();

    };

    // important setting:
    viewer.core().is_animating = true;

}

void scene_structure::initEvents()
{
    // Switch models between 0~9
    viewer.callback_key_down = [this](igl::opengl::glfw::Viewer& viewer, int key, int modifier) -> bool {
        std::cout << "Key: " << key << " " << (unsigned char)key << std::endl;
        int id = key - '0';
        if (id < 10 && id > -1) {
            std::cout << "Model (" << id << ") is desplayed\n";
            //viewer.core().align_camera_center(models[id].V(), models[id].F());
            return true;
        }

        return false;
    };

    viewer.callback_key_pressed = [this](igl::opengl::glfw::Viewer& viewer, int key, int modifiers) -> bool {
        switch (key)
        {
        case 'W': inputForce = UP; break;
        case 'S': inputForce = DOWN; break;
        case 'A': inputForce = LEFT; break;
        case 'D': inputForce = RIGHT; break;
        case 'Q': inputForce = BACKWARD; break;
        case 'E': inputForce = FORWARD; break;
        default: return false;
        }
        return true;
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
            // Cast a ray in the view direction starting from the mouse position
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

                    unsigned int closestVertex = face(0);
                    if (pick.bc(1) > pick.bc(0) && pick.bc(1) > pick.bc(2))
                        closestVertex = face(1);
                    else if (pick.bc(2) > pick.bc(0) && pick.bc(2) > pick.bc(1))
                        closestVertex = face(2);

                    pick.vi = closestVertex;
#ifdef DEBUG
                    std::cout << "---The picked move vertex\n" << models[i].V().row(closestVertex) << "\n---has been hitted\n" << "\n";
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

        if (!params.pauseAnimation)
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

        viewer.data().show_lines = params.showTriangles;

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

void scene_structure::simulate()
{
    if (rawModels.size() + models.size() < 1)
        return;

    for (int i = 0; i < models.size(); i++)
    {
        if (inputForce != NONE)
        {
            Eigen::Vector3d const geometric_center = models[i].V().colwise().mean();
            Eigen::Vector4d force4d{ 0.0, 0.0, 0.0, 1.0 };

            switch (inputForce)
            {
            case UP: force4d(1) = 1.0; break;
            case DOWN: force4d(1) = -1.0; break;
            case LEFT: force4d(0) = -1.0; break;
            case RIGHT: force4d(0) = 1.0; break;
            case FORWARD: force4d(2) = 1.0; break;
            case BACKWARD: force4d(2) = -1.0; break;
            }

            force4d *= params.forceAmp;
            force4d.w() = 1.0;

            // convert force to world space
            Eigen::Matrix4d const screen_to_world_transform =
                (viewer.core().proj * viewer.core().view).inverse().cast<double>();
            Eigen::Vector3d const force = (screen_to_world_transform * force4d).segment(0, 3);
            Eigen::MatrixXd const f = models[i].apply_force(geometric_center, force);

            if (params.showForces)
            {
                Eigen::VectorXd const s = f.rowwise().norm();
                viewer.data().set_data(s);
            }

            inputForce = NONE;
        }

        if (params.setGravity)
        {
            models[i].apply_gravity();
        }

        if (!params.showForces)
        {
            Eigen::VectorXd s(models[i].V().rows());
            s.setConstant(1.0);
            viewer.data().set_data(s);
        }

        models[i].setMode(params.mode);
        models[i].setTau(params.tau);
        models[i].setbeta(params.Rb);
        models[i].setBeta(params.beta);
        models[i].setPerturbation(params.perturbation);

        if (params.mode == QUADRATIC)
        {
            models[i].integrate_quadratic(params.dt);
        }
        else
        {
            models[i].integrate(params.dt);
        }

        viewer.data().V = models[i].V();

    }
};


void scene_structure::clearAllFixedPoints()
{
    for (auto m :models) {
        m.clearFixed();
    }
}


// ****************************** //
// functions (call from inside)
// ****************************** //

// Default scene with three walls and one ground
void scene_structure::initScene()
{
    // Given a plane
    Eigen::MatrixXd V_Plane;
    Eigen::MatrixXi F_Plane;

    primitive_cube(V_Plane, F_Plane);
    std::vector<Eigen::MatrixXd> planesV;
    std::vector<Eigen::MatrixXi> planesF;

    // Store all planes
    for (int i = 0; i < 4; i++) {
        Eigen::MatrixXd tempV = V_Plane;
        Eigen::MatrixXi tempF = F_Plane;
        planesV.push_back(tempV);
        planesF.push_back(tempF);
    }

    // Left Wall
    RawModel p1(planesV[1], planesF[1], getDeformSize());
    p1.scale(Eigen::RowVector3d(0.1, 20, 20));
    p1.translate(Eigen::RowVector3d(-10.1, 0, -10));
    p1.setColor(Eigen::RowVector3d(221. / 255, 221. / 255, 240. / 255)); // red
    addModel(p1);
    // Middle Wall
    RawModel p2(planesV[2], planesF[2], getDeformSize());
    p2.scale(Eigen::RowVector3d(20, 20, 0.1));
    p2.translate(Eigen::RowVector3d(-10, 0, -10.1));
    p2.setColor(Eigen::RowVector3d(118. / 255, 118. / 255, 118. / 255)); // green
    addModel(p2);
    // Ground
    RawModel p3(planesV[3], planesF[3], getDeformSize());
    p3.scale(Eigen::RowVector3d(20, 0.1, 20));
    p3.translate(Eigen::RowVector3d(-10, -0.1, -10));
    p3.setColor(Eigen::RowVector3d(221. / 255, 221. / 255, 221. / 255)); // blue
    addModel(p3);

    // Initial settings
    viewer.core().camera_eye = Eigen::Vector3f(2.f, 2.f, 2.f);
    viewer.core().background_color = Eigen::Vector4f(251. / 255, 251. / 255, 251. / 255, 1);
}

void scene_structure::primitive_cube(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V = (Eigen::MatrixXd(8, 3) <<
        0.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 1.0, 0.0,
        0.0, 1.0, 1.0,
        1.0, 0.0, 0.0,
        1.0, 0.0, 1.0,
        1.0, 1.0, 0.0,
        1.0, 1.0, 1.0).finished().array() + 0.0001; //forget what it is...

    // convert index to index-1 (from 0)
    F = (Eigen::MatrixXi(12, 3) <<
        1, 7, 5,
        1, 3, 7,
        1, 4, 3,
        1, 2, 4,
        3, 8, 7,
        3, 4, 8,
        5, 7, 8,
        5, 8, 6,
        1, 5, 6,
        1, 6, 2,
        2, 6, 8,
        2, 8, 4).finished().array() - 1;
}