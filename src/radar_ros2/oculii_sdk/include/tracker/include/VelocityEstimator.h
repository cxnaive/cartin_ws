#ifndef VELOCITYESTIMATOR_H_
#define VELOCITYESTIMATOR_H_
#include "type_definition.h"
#include <vector>
#include <math.h>
#include <utility>
class VelocityEstimator
{
public:
	explicit VelocityEstimator(float egoSpeedIn);
	pair<float, float> estimateVelocity(Cluster &cluster, float& cost, int& numCandidates);
private:
	struct Parameters
	{
		float minAngleDiff{ 0.003f *3.1415926/180.0 }; //degree
		float minDopplerDiff{ 0.003f }; //m/s
		int maxNumberOfPairs{ 100 };
		float maxError{ 1.5 };
		float speedErrorGate{ 0.5 };
	} parameters;
	pair<float, int> calculateCost(float vx, float vy, Cluster &cluster);
	void selectCandidates(Cluster &cluster);
	pair<float, float> estimateVelocityUSing2Detections(Point& p1, Point& p2);
	struct velocitySolution {
		int index1;
		int index2;
		float vx;
		float vy;
		float cost;
		int numOutLier;
	};
	vector<velocitySolution> candidates;
	float egoSpeed;
};
#endif
