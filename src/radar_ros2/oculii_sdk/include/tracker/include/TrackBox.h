#ifndef TRACKBOX_H_
#define TRACKBOX_H_
#include <deque>
#include <utility>
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <set>
#include "type_definition.h"
using namespace std;

enum class BoxPointLocationIndex
{
	FrontSideCenter,
	FrontSideLeft,
	LeftSideCenter,
	RearSideLeft,
	RearSideCenter,
	RearSideRight,
	RightSideCenter,
	FrontSideRight	
};
class TrackBox
{
public:
	struct Parameters
	{
		float distanceGate{ 1.0f };
		float maxHeight{ 3.0f };  //To-Do consider different values for different vehicles
		float maxLength{ 10.0f };
		float maxWidth{ 3.0f };
		float w2lMax{ 1.2f };
	} parameters;
	pair<float, float> associateWithDetection(float x, float y, float z);
	void updateSize(Cluster& cluster);
	void initialize(Cluster& cluster);
	void setHeading(float angle);
	float getHeading();
	void setCenter(float x, float y, float z);
	void setTrackingPoint(float x, float y, float z, float rd);
	void setCorners();
//private:
	CartesianPoint project(float x, float y, float z);
	CartesianPoint projectBack(float x, float y, float z);
	CartesianPoint trackingPoint;
	BoxPointLocationIndex trackingPointLocation;
	CartesianPoint center;
	float length{ -9999.0f };
	float width{ -9999.0f };
	float height{ -9999.0f };
	float heading{ -9999.0f };
	vector<CartesianPoint> corners {vector<CartesianPoint>(4)};
	CartesianPoint clusterFrontCenter;
	CartesianPoint clusterRearCenter;
private:
	void updateClusterCenter(vector<CartesianPoint>& ponits, float xMin, float xMax);
	//map<float, set<float>> pSorted;
};

#endif

