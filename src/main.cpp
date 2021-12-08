/*  Course Project of INF-574 in Ecole Polytechnique
*   Meshless Deformations Based on Shape Matching
*   https://dl.acm.org/doi/10.1145/1073204.1073216
*   Demo Author: Xun GONG(telecom-paris)
*   Date: 12-12-2021
*
*   example: 407 409 708
*/

#include <cstdlib>
#include <igl/opengl/glfw/Viewer.h>
#include "deformmodel.h"
#include "viewcontrol.h"

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
/// Inner test Plane
/// </summary>
/// <param name="V"> Vertices data </param>
/// <param name="F"> Indices data</param>
void LoadPlane(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V = (Eigen::MatrixXd(4, 3) <<
        1.0, 0.0, 1.0,
        1.0, 0.0, -1.0,
        -1.0, 0.0, 1.0,
        -1.0, 0.0, -1.0).finished();
    // cout << "V:\n" << V << endl;

    // convert index to index-1 (from 0)
    F = (Eigen::MatrixXi(2, 3) <<
        1, 3, 2,
        2, 3, 4).finished().array() - 1;
    // cout << "F:\n" << F << endl;
}



int main(int argc, char* argv[])
{
    ViewControl vc;
    // Given a plane
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    LoadCube(V2, F2);
    Rawdata plane = { V2, F2 };
    DeformModel deformPlane(plane, vc.getDeformSize());
    deformPlane.scale(Eigen::RowVector3d(20, 0.05, 20));
    deformPlane.translate(Eigen::RowVector3d(-10, -0.025, -10));
    vc.addModel(deformPlane);

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
            dm.translate(Eigen::RowVector3d(2.5 * Eigen::RowVector3d::Random().array() + 2.5));
            vc.addModel(dm);
        }
    }

    // Plot the mesh
    vc.load();
    vc.launch();

}

