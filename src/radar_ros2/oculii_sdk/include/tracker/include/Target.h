#ifndef TARGET_H_
#define TARGET_H_
#include <vector>
#include <queue>
#include<Eigen/Core>
#include<Eigen/Dense>
#include "type_definition.h"
#include "TrackBox.h"
#include <unordered_map>
using namespace std;
using namespace Eigen;



class Target
{public:


	Target(int nStates);
	Target(Cluster& cluster);
	~Target();
	void postUpdates();
	float locateTargetSameLane(float x, float y, float z, float rdot);
	float locateTargetNextLane(float x, float y, float z, float rdot);
	void updateClusterInfo(Cluster& cluster, int frameID);
	void calculateUpdates(Cluster& cluster, int frameID);

	TrackBox box;

	MatrixXd states;
	MatrixXd stateCov;
	float sizeX;
	float sizeY;
	float sizeZ;
	int id;  //To-do unsigned int type
	int age;
	int numSeen;
	int numMissed;
	int status {0};
	float heading;
	float speed;
	float yaw;
	float acceleration;

	float cluster_vx;
	float cluster_vy;
	float cluster_rd;
	int cluster_motion;
	int cluster_nDet;
	CartesianPoint cluster_odd;
	CartesianPoint cluster_even;
	float egoSpeed;
	int turnIndex;
	deque<bool> associationHistory;
	int motionStatus;
	//Box box;
private:
	void updateBox();
	void updateMotion();
};



#endif

