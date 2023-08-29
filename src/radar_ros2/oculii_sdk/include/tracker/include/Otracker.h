#ifndef SFTRK_UTRACK_
#define SFTRK_UTRACK_

#include <vector>
#include <fstream>
#include <deque>
#include <array>
#include <time.h>
#include "type_definition.h"
#include "Measurement.h"
#include "TargetList.h"
#include "EgoMotion.h"
#include "radar_defs.h"
#define IS360TRACKER					 


#define PRINT_OUT_DEBUGGING_INFO false // print some debugging info
#define IS_SAVE_DATA false
#define CRC_OUTPUT_SIZE 4

using namespace std;
using oculii::RadarFrameAccuracy;
using oculii::RadarDetectionData;
using oculii::RadarTrackerData;

class Otracker
{
	public:
		~Otracker();

		void init(float frameRate);
		void calTracker(vector<RadarTrackerData>& trkPacket, const vector< RadarDetectionData>& detPacket, 
			int frameId, float egoSpeed, float egoYaw);

		// inputs
		int m_nObs{ 0 };
		float* m_obs{ nullptr };
		bool	isSaveData;				// SAVE DATA OR NOT
	private:

		void runTracker(int frameId, float egoSpeed, float yaw,
			int m_nObs, float* m_obs);
		void outputTracks(vector<RadarTrackerData>& trkPacket);
		float m_fr;
		clock_t m_profile[6];
		ofstream m_detBeforeRotateWriter;
		ofstream m_cpclWriter;
		ofstream m_detWriter;
		ofstream m_clusterWriter;
		ofstream m_tkWriter;
		ofstream m_profileWriter;

		deque<Cluster> m_cluster;
		TargetList targetList;
		EgoMotion egoMotion;
	};


#endif  //SFTRK_UTRACK_
