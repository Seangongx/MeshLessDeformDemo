#include "deformmodel.h"
#include <igl/readOBJ.h>
#include <iostream>
//#include <igl/png/readPNG.h>
#include <Eigen/Eigenvalues> //solver

DeformModel::DeformModel(Rawdata& data, size_t id)
{
    m_filepath = "../";
    m_id = id;
    m_color = 0.5 * Eigen::RowVector3d::Random().array() + 0.5;

    m_V = data.V;
    m_F = data.F;
    m_dtV = m_V;
    m_fixed.resize(m_V.rows(), 1);
    m_fixed.setConstant(false);

#pragma region deform

    m_velocities.resizeLike(m_V); m_velocities.setZero();
    m_forces.resizeLike(m_V); m_forces.setZero();

    m_qi_3d = m_V.rowwise() - m_V.colwise().mean(); // relative cors
    assert(m_qi_3d.rows() == m_V.rows() && m_qi_3d.cols() == m_V.cols());

    // init m_Aqq_3d_inverse
    m_Aqq_3d_inverse.setZero();
    for (unsigned int i = 0; i < m_qi_3d.rows(); i++)
    {
        assert(m_qi_3d.cols() == 3);

        Eigen::Vector3d const qi = m_qi_3d.block(i, 0, 1, m_qi_3d.cols()).transpose();
        assert(qi.rows() == 3 && qi.cols() == 1);
        m_Aqq_3d_inverse += m_mi * qi * qi.transpose();
    }
    assert(m_Aqq_3d_inverse.determinant() != 0.0, "Aqq must be invertible");
    m_Aqq_3d_inverse = m_Aqq_3d_inverse.inverse().eval();

    // init m_Aqq_9d_inverse
    m_qi_9d.resize(m_qi_3d.rows(), 9);
    m_qi_9d.setZero();
    for (unsigned int i = 0; i < m_qi_9d.rows(); i++)
    {
        Eigen::RowVector3d const qi = m_qi_3d.block(i, 0, 1, m_qi_3d.cols());
        Eigen::Matrix<double, 9, 1> qi_bar;
        qi_bar << qi.x(), qi.y(), qi.z(), qi.x()* qi.x(), qi.y()* qi.y(), qi.z()* qi.z(),
            qi.x()* qi.y(), qi.y()* qi.z(), qi.z()* qi.x();
        assert(qi_bar.rows() == 9 && qi_bar.cols() == 1);
        m_qi_9d.block(i, 0, 1, 9) = qi_bar.transpose();
        m_Aqq_9d_inverse += m_mi * qi_bar * qi_bar.transpose();
    }

    Eigen::SelfAdjointEigenSolver<decltype(m_Aqq_9d_inverse)> solver(m_Aqq_9d_inverse);
    std::ostringstream oss{};
    auto const info = solver.info();

    Eigen::Matrix<double, 9, 1> eigenvalues = solver.eigenvalues();
    // regularization
    if (std::abs(solver.eigenvalues().minCoeff()) < 1e-6)
    {
        eigenvalues.array() += m_perturbation;
    }

    // calculate m_Aqq_9d_inverse
    Eigen::Matrix<double, 9, 9> inversion;
    inversion.setZero();
    for (unsigned int i = 0; i < 9; i++)
    {
        double const eigenvalue = eigenvalues(i);
        inversion(i, i) = 1.0 / eigenvalue;
    }
    m_Aqq_9d_inverse = solver.eigenvectors() * inversion * solver.eigenvectors().transpose();

#pragma endregion

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

    if (!result) {
        std::cerr << "FROM [" << filepath << "] OBJ READ RESULT FALSE\n";
        return result;
    }

    m_dtV = m_V;
    m_fixed.resize(m_V.rows(), 1);
    m_fixed.setConstant(false);

#pragma region deform

    m_velocities.resizeLike(m_V); m_velocities.setZero();
    m_forces.resizeLike(m_V); m_forces.setZero();

    // xi0 - xcm0
    m_qi_3d = m_V.rowwise() - m_V.colwise().mean(); // relative cors
    assert(m_qi_3d.rows() == m_V.rows() && m_qi_3d.cols() == m_V.cols());

    // init m_Aqq_3d_inverse
    m_Aqq_3d_inverse.setZero();
    for (unsigned int i = 0; i < m_qi_3d.rows(); i++)
    {
        assert(m_qi_3d.cols() == 3);

        Eigen::Vector3d const qi = m_qi_3d.block(i, 0, 1, m_qi_3d.cols()).transpose();
        assert(qi.rows() == 3 && qi.cols() == 1);
        m_Aqq_3d_inverse += m_mi * qi * qi.transpose();
    }
    assert(m_Aqq_3d_inverse.determinant() != 0.0, "Aqq must be invertible");
    m_Aqq_3d_inverse = m_Aqq_3d_inverse.inverse().eval();

    // init m_Aqq_9d_inverse
    m_qi_9d.resize(m_qi_3d.rows(), 9);
    m_qi_9d.setZero();
    for (unsigned int i = 0; i < m_qi_9d.rows(); i++)
    {
        Eigen::RowVector3d const qi = m_qi_3d.block(i, 0, 1, m_qi_3d.cols());
        Eigen::Matrix<double, 9, 1> qi_bar;
        qi_bar << qi.x(), qi.y(), qi.z(), qi.x()* qi.x(), qi.y()* qi.y(), qi.z()* qi.z(),
            qi.x()* qi.y(), qi.y()* qi.z(), qi.z()* qi.x();
        assert(qi_bar.rows() == 9 && qi_bar.cols() == 1);
        m_qi_9d.block(i, 0, 1, 9) = qi_bar.transpose();
        m_Aqq_9d_inverse += m_mi * qi_bar * qi_bar.transpose();
    }

    Eigen::SelfAdjointEigenSolver<decltype(m_Aqq_9d_inverse)> solver(m_Aqq_9d_inverse);
    std::ostringstream oss{};
    auto const info = solver.info();

    Eigen::Matrix<double, 9, 1> eigenvalues = solver.eigenvalues();
    // regularization
    if (std::abs(solver.eigenvalues().minCoeff()) < 1e-6)
    {
        eigenvalues.array() += m_perturbation;
    }

    // calculate m_Aqq_9d_inverse
    Eigen::Matrix<double, 9, 9> inversion;
    inversion.setZero();
    for (unsigned int i = 0; i < 9; i++)
    {
        double const eigenvalue = eigenvalues(i);
        inversion(i, i) = 1.0 / eigenvalue;
    }
    m_Aqq_9d_inverse = solver.eigenvectors() * inversion * solver.eigenvectors().transpose();

#pragma endregion

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

    m_qi_3d.setZero();
    m_qi_9d.setZero();
    m_Aqq_3d_inverse.setZero();
    m_Aqq_9d_inverse.setZero();
    m_velocities.setZero();
    m_forces.setZero();
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

Eigen::MatrixX3d DeformModel::apply_force(Eigen::Vector3d const& location, Eigen::Vector3d const& force)
{
    Eigen::Vector3d const direction = force.normalized();

    // build our test function for distributing the force on the mesh
    Eigen::VectorXd t(m_dtV.rows());
    t.setZero();

    for (unsigned int i = 0; i < t.rows(); ++i)
    {
        Eigen::Vector3d const r = m_dtV.block(i, 0, 1, m_dtV.cols()).transpose() - location;
        double const s1 = r.dot(-direction);
        double const theta = std::acos(s1 / (r.norm() * direction.norm()));
        double const s2 = r.norm() * std::sin(theta);

        t(i) = std::abs(theta) < 3.14159 / 2.0 ? std::exp(-theta) : 0.0;
    }

    double const tmax = t.maxCoeff();
    double const force_interpolation_weight = 1.0 / tmax;

    // compute forces
    Eigen::MatrixXd f(t.rows(), force.rows());
    for (unsigned int i = 0; i < force.rows(); ++i)
    {
        assert((t.array() * force_interpolation_weight).maxCoeff() <= 1.0 &&
            (t.array() * force_interpolation_weight).minCoeff() >= 0.0,
            "test function must be bounded by [0,1]");
        f.block(0, i, f.rows(), 1) = t.array() * force_interpolation_weight * force(i);
    }

    // accumulate force
    m_forces += f;

    return f;
}


void DeformModel::integrate(double dt)
{
    if (m_mode != ROTATE && m_mode != LINEAR)
        return;

    // pi = xi - xcm
    Eigen::Vector3d const xcm = m_dtV.colwise().mean().transpose();
    Eigen::MatrixX3d const P = m_dtV.rowwise() - xcm.transpose();

    Eigen::Matrix3d Apq;
    Apq.setZero();

    assert(P.rows() == m_qi_3d.rows() && P.cols() == m_qi_3d.cols());

    // Apq = [sum(mi*pi*qiT)]
    for (unsigned int i = 0; i < P.rows(); ++i)
    {
        assert(P.cols() == 3);
        Eigen::Vector3d const qi = m_qi_3d.block(i, 0, 1, m_qi_3d.cols()).transpose();
        Eigen::Vector3d const pi = P.block(i, 0, 1, P.cols()).transpose();
        assert(pi.rows() == 3 && pi.cols() == 1);

        Apq += m_mi * pi * qi.transpose();
    }

    // S = sqrt(ApqT * Apq)
    Eigen::Matrix3d ApqSquared = (Apq.transpose() * Apq);
    Eigen::SelfAdjointEigenSolver<decltype(ApqSquared)> eigensolver(ApqSquared);
    Eigen::Matrix3d SInverse = eigensolver.operatorInverseSqrt();
    // Rotation Matrix R = Apq * SInverse
    Eigen::Matrix3d const R = Apq * SInverse;

    Eigen::Matrix3d A = Apq * m_Aqq_3d_inverse;
    double const volume = A.determinant();
    A = A / std::cbrt(volume); //4.2 make sure the volume is conserved

    double const beta = (m_mode == ROTATE ? 0.0 : m_Beta);
    Eigen::Matrix3d const T = m_Beta * A + (1 - m_Beta) * R; // Interpolate for Transform Matrix

    Eigen::MatrixX3d const g = (T * m_qi_3d.transpose()).transpose().rowwise() + xcm.transpose();

    double const alpha = dt / m_tau; // 3.5 discussed the problem

    Eigen::MatrixX3d const elasticity = alpha * (g - m_dtV);
    Eigen::MatrixX3d const acceleration = m_forces / m_mi;

    for (unsigned int i = 0; i < m_dtV.rows(); ++i)
    {
        if (m_fixed(i))
        {
            m_velocities.block(i, 0, 1, 3) = Eigen::RowVector3d{ 0.0, 0.0, 0.0 };
        }
        else
        {
            m_velocities.block(i, 0, 1, 3) = 
                m_velocities.block(i, 0, 1, 3) + 
                (elasticity.block(i, 0, 1, 3) / dt) +
                (dt * acceleration.block(i, 0, 1, 3)) -
                m_beta * m_velocities.block(i, 0, 1, 3);
        }

        m_dtV.block(i, 0, 1, 3) = m_dtV.block(i, 0, 1, 3) + dt * m_velocities.block(i, 0, 1, 3);
    }

    m_forces.setZero(); 
}


void DeformModel::integrate_quadratic(double dt)
{
    if (m_mode != QUADRATIC)
        return;

    Eigen::RowVector3d const xcm = m_dtV.colwise().mean();
    Eigen::MatrixX3d const P = m_dtV.rowwise() - xcm;

    // compute quadratic terms
    Eigen::Matrix<double, 3, 9> ApqTilde; 
    ApqTilde.setZero();

    assert(m_qi_9d.rows() == m_qi_3d.rows());
    assert(m_qi_9d.rows() == m_dtV.rows());
    assert(m_qi_9d.rows() == m_V.rows());
    for (unsigned int i = 0; i < m_qi_9d.rows(); ++i)
    {
        Eigen::Vector3d pi = P.block(i, 0, 1, 3).transpose();
        Eigen::Matrix<double, 1, 9> qitilde_transpose = m_qi_9d.block(i, 0, 1, 9);

        ApqTilde += m_mi * pi * qitilde_transpose;
    }

    // compute linear terms
    Eigen::Matrix3d Apq;
    Apq.setZero();

    assert(P.rows() == m_qi_3d.rows() && P.cols() == m_qi_3d.cols());

    // Apq = [sum(mi*pi*qit)]
    for (unsigned int i = 0; i < P.rows(); ++i)
    {
        assert(P.cols() == 3);
        Eigen::Vector3d const qi = m_qi_3d.block(i, 0, 1, m_qi_3d.cols()).transpose();
        Eigen::Vector3d const pi = P.block(i, 0, 1, P.cols()).transpose();
        assert(pi.rows() == 3 && pi.cols() == 1);

        Apq += m_mi * pi * qi.transpose();
    }

    Eigen::Matrix3d ApqSquared = (Apq.transpose() * Apq);

    Eigen::SelfAdjointEigenSolver<decltype(ApqSquared)> eigensolver(ApqSquared);
    Eigen::Matrix3d SInv = eigensolver.operatorInverseSqrt();

    Eigen::Matrix3d const R = Apq * SInv;

    Eigen::Matrix<double, 3, 9> Rtilde;
    Rtilde.setZero();
    Rtilde.block(0, 0, 3, 3) = R;

    Eigen::Matrix<double, 3, 9> Atilde = ApqTilde * m_Aqq_9d_inverse;

    // ensure volume preservation in Linear but not here
    // auto A = Atilde.block(0, 0, 3, 3);
    // double const volume = A.determinant();
    // A = A / std::cbrt(volume);

    Eigen::Matrix<double, 3, 9> const T = m_Beta * Atilde + (1 - m_Beta) * Rtilde;

    Eigen::MatrixX3d const shape = (T * m_qi_9d.transpose()).transpose();
    Eigen::MatrixX3d const g = shape.rowwise() + xcm;

    double const alpha = dt / m_tau;

    Eigen::MatrixX3d const elasticity = alpha * (g - m_dtV);
    Eigen::MatrixX3d const acceleration = m_forces / m_mi;

    for (unsigned int i = 0; i < m_dtV.rows(); ++i)
    {
        if (m_fixed(i))
        {
            m_velocities.block(i, 0, 1, 3) = Eigen::RowVector3d{ 0.0, 0.0, 0.0 };
        }
        else
        {
            m_velocities.block(i, 0, 1, 3) = 
                m_velocities.block(i, 0, 1, 3) + 
                (elasticity.block(i, 0, 1, 3) / dt) +
                (dt * acceleration.block(i, 0, 1, 3)) -
                m_beta * m_velocities.block(i, 0, 1, 3);

            //Eigen::RowVector3d const vel = m_velocities.block(i, 0, 1, 3);
            //vel + vel;
        }

        m_dtV.block(i, 0, 1, 3) = m_dtV.block(i, 0, 1, 3) + dt * m_velocities.block(i, 0, 1, 3);
    }

    m_forces.setZero();
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