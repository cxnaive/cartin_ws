#ifndef Measurement_H_
#define Measurement_H_
#include <stdint.h>
#include <deque>
#include <string>
#include <vector>
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <algorithm>
#include <iostream>
#include "type_definition.h"
#include "tracker_defs.h"
#include "TargetList.h"
#include "VelocityEstimator.h"
using namespace std;


class Measurement
{
public:
	Measurement(int m_nObs, float* m_obs, deque<Cluster> &m_clusters, float egoSpeedIn, float fr, float yawIn);
	void updateTrackVelocityMeasurements(TargetList& targetList);
	void clusterDetections(TargetList& targetList);
	
	
private:
	struct Parameters {
		float latDist = 2.5f;  //To-Do distance consideration
		float longDist = 2.5f;
		float heiDist = 2.0f;
		float dopDist = 2.0f;
		float stationaryMinSpeed = 1.0f;
		float maxSpeed = 50.0; //m/s
		float movingPointsRatio = 0.33;
		float minTurnR = 1; 
		float accMax = 20;
	} parameters;

	void updateBestPoint(Cluster& cl, TargetList& targetList);
	void clusterWithTargets(TargetList& targetList);
	void clusterSelf();
	void boxCluster(Cluster& cl);
	void boxClusters();
	void estimateVelocities();
	void setStatus();
	void associateCluster(TargetList& targetList);

	void updateTrackSize(TargetList& targetList);  //To-Do move to track class
	int m_nObs;
	float* m_obs;
	vector<bool> used;
	deque<Cluster>& clusters;
	float egoSpeed;
	float yaw;
	float frameRate;

};

#endif

