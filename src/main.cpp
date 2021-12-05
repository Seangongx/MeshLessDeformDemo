/*  Course Project of INF-574 in Ecole Polytechnique
*   Meshless Deformations Based on Shape Matching
*   https://dl.acm.org/doi/10.1145/1073204.1073216
*   Demo Author: Xun GONG(telecom-paris)
*   Date: 12-12-2021
*/

#include <cstdlib>
#include <igl/opengl/glfw/Viewer.h>
#include "deformmodel.h"
#include "viewcontrol.h"

using namespace std;

/// <summary>
/// Inner test Data
/// </summary>
/// <param name="V"> Vertice data </param>
/// <param name="F"> Indice data </param>
void LoadCube(Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
    V = (Eigen::MatrixXd(8, 3) <<
        0.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 1.0, 0.0,
        0.0, 1.0, 1.0,
        1.0, 0.0, 0.0,
        1.0, 0.0, 1.0,
        1.0, 1.0, 0.0,
        1.0, 1.0, 1.0).finished();
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


int main(int argc, char *argv[])
{
    ViewControl vc;

    // Model Select
    if (argc < 1) {
        std::cerr << "MAYBE FAIL TO LOAD YOU WANT..." << std::endl;

        // Test cube data
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        LoadCube(V, F);
        Rawdata cube = { V, F };

        vc.addModel(cube);
    }
    else
    {
        std::string filepath(argv[1]);

        DeformModel dm(filepath, vc.geDeformSize());
        vc.addModel(dm);
    }

    // Plot the mesh
    vc.load();

}
