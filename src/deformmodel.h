#pragma once

#include <Eigen/Core>

struct Rawdata
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

};

class DeformModel
{
public:
	DeformModel() {}
	DeformModel(Rawdata& data, size_t id);
	DeformModel(std::string const& filepath, size_t id);


#pragma region tranform

	void translate(Eigen::RowVector3d& t);
	void scale(Eigen::RowVector3d& s);

#pragma endregion

#pragma region get

	inline Eigen::MatrixXd& V() { return m_V; }
	inline Eigen::MatrixXi& F() { return m_F; }
	inline Eigen::MatrixXd& dtV() { return m_dtV; }

	inline Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R() { return m_R; }
	inline Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G() { return m_G; }
	inline Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B() { return m_B; }
	inline Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A() { return m_A; }

	inline size_t getID() { return m_id; }
	inline Eigen::RowVector3d getColor() { return m_color; }
	inline bool hasTEX() { return m_hasTEX; }
	size_t getMemoryBytes();

#pragma endregion

#pragma region set

	bool loadOBJ(std::string const& filepath);
	//bool loadTEX(std::string const& filepath); // TODO:

	void reset();
	inline void setID(size_t id) { m_id = id; }

	bool isFixed(unsigned int vi) const { return m_fixed(vi); }
	void setFixed(unsigned int vi, bool fixed = true) { m_fixed(vi) = fixed; }
	unsigned int countFixed() const { return m_fixed.count(); }
	void clearFixed() { m_fixed.fill(false); }

#pragma endregion

private:
	// data
	Eigen::MatrixXd m_V;
	Eigen::MatrixXi m_F;
	Eigen::RowVector3d m_color;
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> m_R, m_G, m_B, m_A;
	// m_dtV == m_V at the begin
	Eigen::MatrixXd m_dtV;
	Eigen::Array<bool, Eigen::Dynamic, 1> m_fixed;

	// properties

	size_t m_id;
	std::string m_filepath;

	// status
	bool m_hasTEX = false;

};

