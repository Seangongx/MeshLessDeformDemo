#include "deformmodel.h"
#include <igl/readOBJ.h>
#include <iostream>

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

    return result;
}


void DeformModel::reset()
{
    m_V.setZero();
    m_F.setZero();
    std::cout << "RESET MODEL: " << m_id << std::endl;

}