#include "deformmodel.h"
#include <igl/readOBJ.h>
#include <iostream>
//#include <igl/png/readPNG.h>

DeformModel::DeformModel(Rawdata& data, size_t id)
{
    m_filepath = "../";
    m_id = id;
    m_V = data.V;
    m_F = data.F;
    m_dtV = m_V;
    m_fixed.resize(m_V.rows(), 1);
    m_fixed.setConstant(false);
    m_color = 0.5 * Eigen::RowVector3d::Random().array() + 0.5;
}

DeformModel::DeformModel(std::string const& filepath, size_t id)
{
    m_filepath = filepath;
    m_id = id;
    if (!loadOBJ(filepath)) {
        std::cerr << "LOAD FAIL: " << filepath << std::endl;
    }
}


bool DeformModel::loadOBJ(std::string const& filepath)
{
    reset();
    bool const result = igl::readOBJ(filepath, m_V, m_F);
    m_dtV = m_V;

    m_fixed.resize(m_V.rows(), 1);
    m_fixed.setConstant(false);


    if (result) {
        m_color = 0.5 * Eigen::RowVector3d::Random().array() + 0.5;
    }
    return result;
}
void DeformModel::reset()
{
    m_V.setZero();
    m_F.setZero();
    m_dtV.setZero();
    m_fixed.setConstant(false);
}

void DeformModel::translate(Eigen::RowVector3d& t)
{
    for (int i = 0; i < m_V.rows(); i++)
    {
        m_V.row(i) += t;
    }
    m_dtV = m_V;
}

void DeformModel::scale(Eigen::RowVector3d& t)
{

    for (int i = 0; i < m_V.rows(); i++)
    {
        m_V.row(i).x() *= t.x();
        m_V.row(i).y() *= t.y();
        m_V.row(i).z() *= t.z();
    }
    m_dtV = m_V;
}

size_t DeformModel::getMemoryBytes()
{
    std::size_t mem = 0;
    mem += m_V.rows() * m_V.cols() * sizeof(decltype(m_V)::CoeffReturnType);
    mem += m_F.rows() * m_F.cols() * sizeof(decltype(m_F)::CoeffReturnType);
    mem += m_dtV.rows() * m_dtV.cols() * sizeof(decltype(m_dtV)::CoeffReturnType);
    //mem += Q_.rows() * Q_.cols() * sizeof(decltype(Q_)::CoeffReturnType);
    //mem += AqqInv_.rows() * AqqInv_.cols() * sizeof(decltype(AqqInv_)::CoeffReturnType);
    //mem += Qquadratic_.rows() * Qquadratic_.cols() *
    //    sizeof(decltype(Qquadratic_)::CoeffReturnType);
    //mem += AqqInvQuadratic_.rows() * AqqInvQuadratic_.cols() *
    //    sizeof(decltype(AqqInvQuadratic_)::CoeffReturnType);
    //mem += fixed_.rows() * fixed_.cols() * sizeof(decltype(fixed_)::CoeffReturnType);
    //mem += f_.rows() * f_.cols() * sizeof(decltype(f_)::CoeffReturnType);
    //mem += F_.rows() * F_.cols() * sizeof(decltype(F_)::CoeffReturnType);

    return mem;
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