#pragma once

#include "rawmodel.hpp"

enum MODE
{ 
	RIGID,
	LINEAR, 
	QUADRATIC
	// PLASTIC
};

class DeformModel
{
public:
	DeformModel() {}
	~DeformModel() {};
	DeformModel(Rawdata& data, size_t id);
	DeformModel(std::string const& filepath, size_t id);

	Eigen::MatrixX3d apply_force(Eigen::Vector3d const& location, Eigen::Vector3d const& force);
	void apply_gravity() { m_forces.rowwise() += Eigen::RowVector3d{ 0.f, -9.8f, 0.f }; }
	void integrate(double dt);
	void integrate_quadratic(double dt);

#pragma region get

	inline Eigen::MatrixXd& V() { return m_V; }
	inline Eigen::MatrixXi& F() { return m_F; }
	inline Eigen::MatrixX3d& X() { return m_X; }

	inline Eigen::MatrixX3d& forces() { return m_forces; }
	inline Eigen::MatrixX3d& velocities() { return m_velocities; }
	inline Eigen::MatrixX3d& positions() { return m_X; }

	inline Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R() { return m_R; }
	inline Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G() { return m_G; }
	inline Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B() { return m_B; }
	inline Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A() { return m_A; }

	inline double alpha() const { return m_alpha; }
	inline double beta() const { return m_beta; }
	inline double Beta() const { return m_Beta; }
	inline double tau() const { return m_tau; }
	inline double perturbation() const { return m_perturbation; }


	inline size_t getId() { return m_id; }
	inline MODE getMode() const { return m_mode; }
	inline Eigen::RowVector3d getColor() { return m_color; } // no set for now


	inline unsigned int countFixed() const { return m_fixed.count(); }
	inline bool isFixed(unsigned int vi) const { return m_fixed(vi); }
	inline bool hasTEX() { return m_hasTEX; }
	size_t getMemoryBytes();

#pragma endregion

#pragma region set

	void reset();

	bool loadOBJ(std::string const& filepath);
	//bool loadTEX(std::string const& filepath); // TODO:

	inline void setId(size_t id) { m_id = id; }
	inline void setMode(MODE mode) { m_mode = mode; }
	inline void setalpha(double a) { m_alpha = a; }
	inline void setbeta(double b) { m_beta = b; }
	inline void setBeta(double B) { m_Beta = B; }
	inline void setTau(double t) { m_tau = t; }
	inline void setPerturbation(double p) { m_perturbation = p; }

	void setFixed(unsigned int vi, bool fixed = true) { m_fixed(vi) = fixed; }
	void clearFixed() { m_fixed.fill(false); }

#pragma endregion


#pragma region tranform

	void translate(Eigen::RowVector3d& t);
	void scale(Eigen::RowVector3d& s);
	void rotate(Eigen::RowVector3d u, double theta);

#pragma endregion

private:

	// ****************************** //
	// precompute data
	// ****************************** //

	Eigen::MatrixXd			m_V; // initial Vertices
	Eigen::MatrixXi			m_F; // initial Faces(Triangles for now)
	Eigen::RowVector3d		m_color;
	Eigen::MatrixX3d		m_forces; // instant force in intergration
	Eigen::MatrixX3d		m_velocities;
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> m_R, m_G, m_B, m_A;
	Eigen::Array<bool, Eigen::Dynamic, 1>		m_fixed;

	Eigen::MatrixX3d		m_X; // store position (m_X == m_V at the begin)
	Eigen::MatrixX3d		m_qi_3d; // store qi = xi0 - xicm0
	Eigen::Matrix<double, Eigen::Dynamic, 9>	m_qi_9d; // store qi = xi0 - xicm0 (extended to 9 dimensions)
	Eigen::Matrix3d			m_Aqq_3d_inverse; // precomputed Aqq for 3 dimensions
	Eigen::Matrix<double, 9, 9>					m_Aqq_9d_inverse; // precomputed Aqq for 9 dimensions

	// ****************************** //
	// control parameters
	// ****************************** //

	MODE	m_mode			= RIGID;
	double		m_alpha			= 0.0;	// stiffness parameter
	double		m_beta			= 0.0;	//
	double		m_tau			= 0.0;	// time constant to vary velocity update for varying time steps.
	double		m_Beta			= 0.0;	// rotate parameter
	double		m_perturbation	= 0.1;
	double		m_mi			= 1.0;	// default mass weight

	// ****************************** //
	// model properties
	// ****************************** //

	std::string	m_filepath;
	size_t		m_id;
	bool		m_hasTEX = false;

};

