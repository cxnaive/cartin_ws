#ifndef POLORSTATETRACKER_H_
#define POLORSTATETRACKER_H_
#include "BaseTracker.h"
#include "type_definition.h"
#include "Target.h"
class PolarStateTracker : public BaseTracker
{
public:
	PolarStateTracker();
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
	int n_stateVector{ 6 };
	int n_measurementVector{ 4 };
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


