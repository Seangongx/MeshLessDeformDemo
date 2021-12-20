#include "rawmodel.h"
#include <igl/readOBJ.h>
#include <iostream>
//#include <igl/png/readPNG.h>
#include <Eigen/Eigenvalues> //solver


RawModel::RawModel(Eigen::MatrixXd& V, Eigen::MatrixXi& F, size_t id)
{
    m_filepath = "../";
    m_id = id;
    m_color = 0.5 * Eigen::RowVector3d::Random().array() + 0.5;

    m_V = V;
    m_F = F;
    std::cout << m_filepath << "'s model vertices: " << m_V.size() << " and Model Faces: " << m_F.rows() << std::endl;
}

RawModel::RawModel(Rawdata& data, size_t id)
{
    RawModel(data.V, data.F, id);
}

RawModel::RawModel(std::string const& filepath, size_t id)
{
    m_filepath = filepath;
    m_id = id;
    if (!loadOBJ(filepath)) {
        std::cerr << "LOAD FAIL: " << filepath << std::endl;
    }
}

bool RawModel::loadOBJ(std::string const& filepath)
{
    reset();
    bool const result = igl::readOBJ(filepath, m_V, m_F);

    if (!result) {
        std::cerr << "FROM [" << filepath << "] OBJ READ RESULT FALSE\n";
        return result;
    }
    else {
        m_color = 0.5 * Eigen::RowVector3d::Random().array() + 0.5;
    }
    return result;
}

void RawModel::translate(Eigen::RowVector3d& t)
{
    for (int i = 0; i < m_V.rows(); i++)
        m_V.row(i) += t;
}

void RawModel::scale(Eigen::RowVector3d& t)
{
    for (int i = 0; i < m_V.rows(); i++) {
        m_V.row(i).x() *= t.x();
        m_V.row(i).y() *= t.y();
        m_V.row(i).z() *= t.z();
    }
}

/**
* calculate a rotation of angle 'theta' around a given direction defined by vector 'u'
* @param u  a vector corresponding to the rotation axis
* @param theta  rotation angle
  */
void RawModel::rotate(Eigen::RowVector3d u, double theta) {

    u.normalize(); // never forget to normalize axis
    Eigen::Quaterniond q = Eigen::Quaterniond(
        cos(theta / 2),
        sin(theta / 2) * u.x(),
        sin(theta / 2) * u.y(),
        sin(theta / 2) * u.z()
    ).normalized();
    for (int i = 0; i < m_V.rows(); i++)
    {
        Eigen::Quaterniond x = Eigen::Quaterniond(0, m_V(i, 0), m_V(i, 1), m_V(i, 2));
        Eigen::Quaterniond v_ = q * x * q.conjugate();
        m_V.row(i) = Eigen::Vector3d(v_.x(), v_.y(), v_.z());
    }
}




void RawModel::reset() {
    // properties
    // m_id  = 0; // do not reset this unless remove this instance
    m_filepath;

    // data
    m_V.setZero();
    m_F.setZero();
    m_color.setOnes(); // set light source
    m_R.setZero();
    m_G.setZero();
    m_B.setZero();
    m_A.setZero();

    // states
    bool m_hasTEX = false;

}

/* ----------------------TODO------------------------ */
//Need to rebuild makefile
//bool DeformModel::loadTEX(std::string const& filepath)
//{
//    bool const result = igl::png::readPNG(filepath, m_R, m_G, m_B, m_A);
//    if (result) {
//        m_hasTEX = result;
//    }
//    return result;
//}