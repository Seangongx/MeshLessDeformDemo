#include <igl/opengl/glfw/Viewer.h>
#include <igl/read_triangle_mesh.h>
#include <igl/png/readPNG.h>

int main(int argc, char* argv[])
{
    igl::opengl::glfw::Viewer v;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("../data/bunny.obj", V, F);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
    igl::png::readPNG("../data/checker.png", R, G, B, A);
    v.data().set_mesh(V, F);
    v.data().set_texture(R, G, B, A);
    v.data().use_matcap = true;
    v.data().show_lines = false;
    v.launch();
}