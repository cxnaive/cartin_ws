#ifndef TARGETLIST_H_
#define TARGETLIST_H_
#include "BaseTracker.h"
#include "CartesianPositionTracker.h"
#include "CartesianVelocityTracker.h"
#include "PolarPointSpeedTracker.h"
#include "StationaryTracker.h"
#include "EgoMotion.h"
#include <algorithm>
#include <memory>
#include <limits>
class TargetList
{
public:
	void update(deque<Cluster>& m_clusters);
	void predict();
	vector<Target> targets;
	void setEgo(float frameRateIn, float egoSpeedIn, float yawIn, int frameIdIn);
	void compensateEgoMotion(EgoMotion& egoMotion);
	void updateBox();
	pair<int, int> findTarget(float x, float y, float z, float rdot);
private:
	void manageTargets();
	void createTarget(BaseTracker& tracker, Cluster& cluster);
	void buildIDDict();
	int selectTracker(Target& target);
	CartesianPositionTracker CPTTracker;
	CartesianVelocityTracker CVTTracker;
	StationaryTracker STTracker;
	//void updateTarget(Cluster& cluster, Target& target);
	int maxID{ 0 };
	queue<int> spareIDs;
	unordered_map<int, int> idDict;
	//std::unique_ptr<BaseTracker> trackerPtr{ nullptr }; 
	BaseTracker* trackerPtr { nullptr };
	float frameRate{ std::numeric_limits<float>::min() };
	float egoSpeed{std::numeric_limits<float>::min()};
	float yaw{ std::numeric_limits<float>::min() };

	int frameID{-1};
};
#endif
