#pragma once

#include <Eigen/Core>

class DeformModel
{
public:
	DeformModel();
	DeformModel(std::string const& filepath, size_t id);

	inline Eigen::MatrixXd & V() { return m_V; }
	inline Eigen::MatrixXi & F() { return m_F; }

	bool loadOBJ(std::string const& filepath);
	void reset();

	// get:
	inline size_t getID() { return m_id; }

	// set:
	inline void setID(size_t id) { m_id = id; }


private:
	Eigen::MatrixXd m_V;
	Eigen::MatrixXi m_F;
	size_t m_id;
	std::string m_filepath;

};

