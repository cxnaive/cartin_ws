#ifndef EGOMOTION_H_
#define EGOMOTION_H_
#include <deque>
#include <utility>
#include <iostream>
#include <limits>
#include <math.h>

using namespace std;

class EgoMotion
{
public:
	//EgoMotion(float frameRate);
	void updateEgoState(float speed, float yaw, int frameRate);
	pair<pair<float, float>, pair<float, float>> compensateEgoMotion(float x, float y, float vx, float vy);
	struct Parameter {
		float maxAge{ 0.5f };      //s
		float maxAcc{ 10.0f };     // m/s^2
		float minTurnR{ 5.0f };    //m
		float maxSpeed{ 50.0f };   //m/s
		float maxYaw{ 1.0f };      //   rad/s
	} parameters;
private:
	struct EgoState {
		float speed;
		float yaw;
		float accleration;
	};
	bool isGoodMotionState(float speed, float yaw, float acc) const;
	deque<EgoState> states;
	int numStates{ 3 };
	float deltaT{ 0.0f };
	float age{ numeric_limits<float>::max()};
};

#endif

