#ifndef TYPE_DEFINITION_H_
#define TYPE_DEFINITION_H_
#include <stdint.h>
#include <deque>
#include <string>
#include <utility>
#include <limits>

using namespace std;
struct Point
{
	float range{ -9999.0f };
	float rdot{ -9999.0f };
	float alpha{ -9999.0f };
	float beta{ -9999.0f };
	float power{ -9999.0f };
	int stationaryFlag{ -9999 };
	float x{ -9999.0f };
	float y{ -9999.0f };
	float z{ -9999.0f };
	int radarId{-9999};
	float true_alpha{ -9999.0f };
	int index{-1};
	float distXY {-9999.0};
};

struct Box
{	float xMin{ -9999.0f };
	float xMax{ -9999.0f };
	float yMin{ -9999.0f };
	float yMax{ -9999.0f };
	float zMin{ -9999.0f };
	float zMax{ -9999.0f };
};

struct CartesianPoint
{
	float x{ -9999.0f };
	float y{ -9999.0f };
	float z{ -9999.0f };
	float rd{-9999.0f };
};

struct accumPoint
{
	uint32_t  trackId{ 0u };
	uint32_t  status{ 0u };
	float x{ -9999.0f };
	float y{ -9999.0f };
	float z{ -9999.0f };
	float power{ -9999.0f };
};

struct Cluster
{
	uint16_t        id{ 0u };
	int        trackId{ 0};
	uint32_t        status{ 0u };
	uint32_t        moTionStatus{ 0u };
	deque<Point>    points;
	Point           center;
	float           speed[3]{ -9999.0f, -9999.0f, -9999.0f };
	Box				box;
	float           longitudinalSpeed{ -9999.0f };
	float           lateralSpeed{ -9999.0f };
	float           verticalSpeed{ -9999.0f };
	int             numVelocityCanidates{ -1 };
	float           velocityEstimatorCost{ 9999.0 };
};

struct accumCluster
{
	uint16_t        id;
	deque<int>      trackId;
	uint32_t        status;// 0 stationary | 1 moving
	float           speed[3];
	deque<Point>    points;
	Point           center;
};

struct assocCluster
{
	uint32_t        trackId;
	uint32_t        status;// 0 stationary | 1 moving
	float           speed[3];// vx, vy, vz of the track, 3d heading angle
	deque<Point>    points;
	float           sx; // radian , length / 2 , width / 2, distance from edge to center
	float           sy;
	float           sz;
	Point           center;
	string          clusterId;
};

struct trackObj
{
	uint32_t targetID;
	uint32_t trackID;
	uint8_t status;
	uint8_t age;
	uint8_t cnt;
	uint8_t missed;
	float x;
	float y;
	float z;
	float xdot;
	float ydot;
	float zdot;
	float range;
	float rdot;
	float alpha;
	float beta;
	float speed;
	float max_speed;
	float dist;
	bool isMoving;// false: stationary | true: moving
	float sx;
	float sy;
	float sz;
	float sconfidence;// track size confidence
	float vsx;
	float vsy;
	float vsz;
	float lengthSize;
	float widthSize;
};
#endif