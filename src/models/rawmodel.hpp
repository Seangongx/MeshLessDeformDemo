#pragma once

#include <Eigen/Core>

struct Rawdata
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
};

class RawModel 
{
public:
	RawModel() {};
	~RawModel() {};
	RawModel(Eigen::MatrixXd& V, Eigen::MatrixXi& F, size_t id);
	RawModel(Rawdata& data, size_t id);
	RawModel(std::string const& filepath, size_t id);

#pragma region get

	inline Eigen::MatrixXd& V() { return m_V; }
	inline Eigen::MatrixXi& F() { return m_F; }

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
	inline void setId(size_t id) { m_id = id; }
	inline void setColor(Eigen::RowVector3d color) { m_color = color; }

	void reset();

	bool loadOBJ(std::string const& filepath);
	//bool loadTEX(std::string const& filepath); // TODO:

#pragma endregion


#pragma region tranform

	void translate(Eigen::RowVector3d& t);
	void scale(Eigen::RowVector3d& s);
	void rotate(Eigen::RowVector3d u, double theta);

#pragma endregion

private:
	// data

	Eigen::MatrixXd m_V; //Vertices
	Eigen::MatrixXi m_F; //Faces(Triangles for now)
	Eigen::RowVector3d m_color;
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> m_R, m_G, m_B, m_A;

	// properties

	std::string m_filepath;
	size_t m_id;

	// states

	bool m_hasTEX = false;
	bool m_hasCOLOR = false;

};

