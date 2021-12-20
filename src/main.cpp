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
#include "deformmodel.h"
#include "viewcontrol.h"
#define PI  3.14159265358979323846

using namespace std;

/// <summary>
/// Inner test Cube
/// </summary>
/// <param name="V"> Vertices data </param>
/// <param name="F"> Indices data </param>
void LoadCube(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V = (Eigen::MatrixXd(8, 3) <<
        0.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 1.0, 0.0,
        0.0, 1.0, 1.0,
        1.0, 0.0, 0.0,
        1.0, 0.0, 1.0,
        1.0, 1.0, 0.0,
        1.0, 1.0, 1.0).finished().array() + 0.0001;
    // cout << "V:\n" << V << endl;

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
    // cout << "F:\n" << F << endl;
}

/// <summary>
/// Inner test Scene with three walls and one ground
/// </summary>
/// <param name="V"> Vertices data </param>
/// <param name="F"> Indices data</param>
void LoadDefaultScene(ViewControl& vc) {
    // Given a plane
    Eigen::MatrixXd V_Plane;
    Eigen::MatrixXi F_Plane;

    LoadCube(V_Plane, F_Plane);
    std::vector<Eigen::MatrixXd> planesV;
    std::vector<Eigen::MatrixXi> planesF;


    for (int i = 0; i < 4; i++) {
        Eigen::MatrixXd tempV = V_Plane;
        Eigen::MatrixXi tempF = F_Plane;
        planesV.push_back(tempV);
        planesF.push_back(tempF);
    }

    // Set default camera position
    RawModel eye(planesV[0], planesF[0], vc.getDeformSize());
    eye.scale(Eigen::RowVector3d(0.05, 0.05, 0.05));
    eye.translate(Eigen::RowVector3d(30, 30, 30));
    eye.setColor(Eigen::RowVector3d(1, 1, 1)); // light eye
    vc.addModel(eye);
    // Middle
    //Rawdata d1 = { tempV[1], tempF[1] };
    //RawModel p1(tempV[1], tempF[1], vc.getDeformSize());
    RawModel p1(planesV[1], planesF[1], vc.getDeformSize());
    p1.scale(Eigen::RowVector3d(20, 0.1, 20));
    p1.rotate(Eigen::RowVector3d(0, 0, 1), PI / 2);
    p1.translate(Eigen::RowVector3d(-10, -0.1, -10));
    p1.setColor(Eigen::RowVector3d(0.9, 0, 0)); // red
    vc.addModel(p1);
    // Right
    RawModel p2(planesV[2], planesF[2], vc.getDeformSize());
    p2.scale(Eigen::RowVector3d(20, 0.1, 20));
    p2.rotate(Eigen::RowVector3d(1, 0, 0), -PI / 2);
    p2.translate(Eigen::RowVector3d(-10, -0.1, -10));
    p2.setColor(Eigen::RowVector3d(0, 0.9, 0)); // green
    vc.addModel(p2);
    // Ground
    RawModel p3(planesV[3], planesF[3], vc.getDeformSize());
    p3.scale(Eigen::RowVector3d(20, 0.1, 20));
    p3.translate(Eigen::RowVector3d(-10, -0.1, -10));
    p3.setColor(Eigen::RowVector3d(0, 0, 0.9)); // blue
    vc.addModel(p3);

    //vc.setDefaultViewPosition(eye);
}


int main(int argc, char* argv[])
{
    ViewControl vc;
    LoadDefaultScene(vc);

    DeformModel cube("../data/cube.obj", vc.getDeformSize());
    vc.addModel(cube);

    // Model Select
    if (argc < 2) {
        std::cerr << "MAYBE FAIL TO LOAD YOU WANT...\n \
                     REMEMBER DIFFERENT PATH BETWEEN WINDOWS AND LINUX" << std::endl;
    }
    else
    {
        for (size_t i = 1; i < argc; i++) {

            std::string filepath(argv[i]);
            DeformModel dm(filepath, vc.getDeformSize());
            //dm.translate(Eigen::RowVector3d(2.5 * Eigen::RowVector3d::Random().array() + 2.5));
            vc.addModel(dm);
        }
    }

    // Plot the mesh
    vc.load();
    vc.launch();

}

