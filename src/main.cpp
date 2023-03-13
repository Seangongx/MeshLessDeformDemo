/*  Course Project of INF-574 in Ecole Polytechnique
*   Meshless Deformations Based on Shape Matching
*   https://dl.acm.org/doi/10.1145/1073204.1073216
*   Demo Author: Xun GONG(telecom-paris)
*   Date: 12-12-2021
*
*   Libigl example: 407 409 708
*/

#include <cstdlib>
#include <igl/opengl/glfw/Viewer.h>
#include "models/deformmodel.hpp"
#include "scene.hpp"
#include <random>
#define PI 3.14159265358979323846

#define DEBUG

using namespace std;


int main(int argc, char* argv[])
{
    scene_structure scene;

    std::cout << "Initialize data of the scene ..." << std::endl;
    scene.initialize();
    std::cout << "Initialization success" << std::endl;

#ifndef DEBUG
    DeformModel cube("../data/cube.obj", vc.getDeformSize());
    vc.addModel(cube);
#endif

    // Model Select
    if (argc < 2) {
        std::cerr << "MAYBE FAIL TO LOAD YOU WANT...\n \
                     REMEMBER DIFFERENT PATH BETWEEN WINDOWS AND LINUX" << std::endl;
    }
    else
    {
        // pseudorandom
        std::random_device dev;
        std::mt19937 gen(dev());
        std::uniform_real_distribution<> rd(0.f, 8.f);

        for (size_t i = 1; i < argc; i++) {

            std::string filepath(argv[i]);
            DeformModel dm(filepath, scene.getDeformSize());
            dm.translate(Eigen::RowVector3d(rd(gen), 0.f, rd(gen)));
            scene.addModel(dm);
        }
    }

    // Plot the mesh
    scene.load();
    scene.launch();

}

