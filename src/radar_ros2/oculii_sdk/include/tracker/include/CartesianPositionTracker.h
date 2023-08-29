#ifndef CARTESIANPOSITIONTRACKER_H_
#define CARTESIANPOSITIONTRACKER_H_
#include "BaseTracker.h"
#include "type_definition.h"
#include "Target.h"
class CartesianPositionTracker : public BaseTracker
{
public:
	CartesianPositionTracker();
	~CartesianPositionTracker();
	void timePredict(Target& track);
	//void setInputs(Cluster& cluster, Track& track);
	void update(Cluster& cluster, Target& taregt);
	//void setInputs(Cluster& cluster, Target& track);
	//void setEgo(float egoSpeed, float m_fr, float yaw);
	Target creatTarget(Cluster& cluster);
private:
	virtual MatrixXd stateFunction(MatrixXd s);
	virtual MatrixXd measurementFunction(MatrixXd m);
	void setCKF(Target& target);
	static constexpr size_t n_stateVector = 8;
	int n_measurementVector{ 3 };
	//Cluster& cluster;
	//Track& track;
	MatrixXd stateVector;
	//MatrixXd stateCov;
	MatrixXd measurementVector;
	void getStates(Target& target);
	//float deltaT;
	//float egoSpeed;
	//float yaw;
};
#endif




