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
/// Author: GONG Xun(telecom-paris)
/// Date:  26/9/2021 INF-574 TP1
/// </summary>
/// <param name="V">Vertices data</param>
/// <param name="t">translation</param>
void translate(Eigen::MatrixXd& V, Eigen::RowVector3d& t)
{
    for (int i = 0; i < V.rows(); i++)
    {
        V.row(i) += t;
    }
}

/// <summary>
/// Author: GONG Xun(telecom-paris)
/// Date:  26/9/2021 INF-574 TP1
/// </summary>
/// <param name="V">Vertices data</param>
/// <param name="t">translation</param>
void scale(Eigen::MatrixXd& V, Eigen::RowVector3d& t)
{
    for (int i = 0; i < V.rows(); i++)
    {
        V.row(i).x() *= t.x();
        V.row(i).y() *= t.y();
        V.row(i).z() *= t.z();
    }
}

/// <summary>
/// Author: GONG Xun(telecom-paris)
/// Date:  26/9/2021 INF-574 TP1
/// </summary>
/// <param name="V"> Vertices data </param>
/// <param name="u"> Transform axis</param>
/// <param name="theta"> Angle using double</param>
void Rotation(Eigen::MatrixXd& V, Eigen::RowVector3d u, double theta) {
    u.normalize();
    // never forget to normalize
    Eigen::Quaterniond q = Eigen::Quaterniond(cos(theta) / 2, sin(theta) * u.x(), sin(theta) * u.y(), sin(theta) * u.z()).normalized();

    for (int i = 0; i < V.rows(); i++)
    {
        Eigen::Quaterniond x = Eigen::Quaterniond(0, V(i, 0), V(i, 1), V(i, 2));    // .normalized(); we don't need normalize vector
        Eigen::Quaterniond v_ = q * x * q.conjugate();                       //v_.normalize();
        V.row(i) = Eigen::Vector3d(v_.x(), v_.y(), v_.z());
    }
}

/// <summary>
/// Inner test Scene with three walls and one ground
/// </summary>
/// <param name="V"> Vertices data </param>
/// <param name="F"> Indices data</param>
void LoadDefaultScene(Eigen::MatrixXd& V, Eigen::MatrixXi& F, ViewControl& vc) {
    LoadCube(V, F);
    Eigen::MatrixXd tempV[4];
    Eigen::MatrixXi tempF[4];
    for (int i = 0; i < 4; i++) {
        tempV[i] = V;
        tempF[i] = F;
    }
    // Left

    // Middle

    // Right

    // Ground

    Rawdata plane = { tempV[3], tempF[3] };
    DeformModel deformPlane(plane, vc.getDeformSize());
    deformPlane.scale(Eigen::RowVector3d(20, 0.05, 20));
    deformPlane.translate(Eigen::RowVector3d(-10, -0.025, -10));
    vc.addModel(deformPlane);

}


int main(int argc, char* argv[])
{
    ViewControl vc;
    // Given a plane
    Eigen::MatrixXd V_Plane;
    Eigen::MatrixXi F_Plane;
    LoadDefaultScene(V_Plane, F_Plane, vc);

    std::cout << vc.getDeformSize() << std::endl;
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
            dm.translate(Eigen::RowVector3d(2.5 * Eigen::RowVector3d::Random().array() + 2.5));
            vc.addModel(dm);
        }
    }

    // Plot the mesh
    vc.load();
    vc.launch();

}

