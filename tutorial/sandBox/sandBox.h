#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include "igl/opengl/ViewerData.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
	void pre_draw();
	void SetQueue(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
	//void SetQueue();
	double x_direction = 0.01;
	double y_direction = 0;

	//assignment3
	Eigen::Vector3d destination;
	void Ik();
	bool isIK=false;
	//int linkNumber;
private:
	// Prepare array-based edge data structures and priority queue
	typedef std::set<std::pair<double, int>> PriorityQueue;
	std::vector<PriorityQueue*> Q;
	std::vector<std::vector<PriorityQueue::iterator>> Qit;
	std::vector<Eigen::MatrixXd*> C;
	std::vector<Eigen::VectorXi*> EMAP; //vector Xi its int vector size X (unknown yet)
	std::vector<Eigen::MatrixXi*> E,EF,EI;
	std::vector<int> num_collapsed;

	//EI - connects edge to vertex index in triangle
	//EF - connects edges to faces
	//EMAP - connects faces to edges
	//E- – edges <index of source vertex, index of destination vertex>

	//assignment3
	
	
	void Animate();
};

