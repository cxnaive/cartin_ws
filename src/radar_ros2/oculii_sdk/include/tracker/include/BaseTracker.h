#ifndef BASETRACKER_H_
#define BASETRACKER_H_
#include <iostream>
#include <cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include "type_definition.h"
#include "Target.h"
using namespace std;
using namespace Eigen;

class BaseTracker {
public:
	~BaseTracker();
	void predict(MatrixXd& x);
	void sensorUpdate(MatrixXd& x, const MatrixXd z);
	virtual MatrixXd stateFunction(MatrixXd x) = 0;
	virtual MatrixXd measurementFunction(MatrixXd x) = 0;
	virtual Target creatTarget(Cluster& cluster) = 0;
	virtual void timePredict(Target& target) = 0;
	virtual void update(Cluster& cluster, Target& taregt)=0;
	void setEgo(float m_fr, float egoSpeed, float yaw);

	float egoSpeed;
	float deltaT;
	float yaw;

protected:
	int n;      
	int m;      
	MatrixXd Q;	
	MatrixXd R;	
	MatrixXd P;	
};
#endif
