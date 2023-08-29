/* Copyright (c) Oculii 2017 */
#ifndef TRACKER_SYSTEM_DEFINES_
#define TRACKER_SYSTEM_DEFINES_

#include <stdint.h>

/* NOTE: For future integration: all configuration parameters are
 * located in offr_config.h file*/

#define IS360TRACKER   // comment out it if not using 360 tracker
#define MAXIMUM_NUM		(5U)
#ifdef IS360TRACKER
	#define UNSYNCED_MULTIMODE // define this macro when we cannot make sure all sensors have same mode in combinedpcl before sending to 360 tracker program as input
#endif // IS360TRACKER

#ifndef _4D_TRACKER
#define _4D_TRACKER
#endif

#ifndef GTRACK
#define GTRACK        /* Used to re-route buffer allocation to L3 */
#endif

#define OUT_LSB_E1   (10.0f)
#define OUT_LSB_E2   (100.0f)

//#define USE_OFFR_ALLOCATOR
#define TRACK_MAX_OBJS 150 //MAX number of measurements that tracker can accept, in firmware dss

#ifdef USE_OFFR_ALLOCATOR
extern Offr_Allocator gAllocator;
#endif

/* Tracker Settings */
#define AR_ALPHA (0.8f)

#ifdef _4D_TRACKER
	#define N_STATE 6
	#define N_MEASURE 4
#elif defined _3D_TRACKER
	#define N_STATE 4
	#define N_MEASURE 3
#endif 

#define N_RDOT 5

/* Implementation Limits */
#ifndef IS360TRACKER
//#define MAX_TRACKS 100 
#define MAX_ASSIGN 10 
#define MAX_MEASURE 1800         /* Should be equal to gMmwDssMCB.cliCommonCfg.outModeCfg.maxMSSOutDots */
#else
//#define MAX_TRACKS 1000
#define MAX_ASSIGN 20
#define MAX_MEASURE 5000

#endif // !IS360TRACKER

/* Track Life Requirements */
#define AGE_OUT_LIMIT 6
#define AGE_IN_REQUIRED 5 
#define M_N_REQUIRED 0.6f

/* Measurement Index Location */
#ifndef IS360TRACKER
#define N_OBS_MEASURE 6
#else
#define N_OBS_MEASURE 8
#define RADAR_INDEX 6
#define TRUE_ALPHA_MEAS_INDEX 7
#endif // !IS360TRACKER

#define RANGE_MEAS_INDEX 	  0
#define DOPPLER_MEAS_INDEX 	  1
#define HORI_ANGLE_MEAS_INDEX 2

#ifdef _4D_TRACKER 
	#define VERT_ANGLE_MEAS_INDEX 3
	#define POWER_MEAS_INDEX 	  4
	#define FLAG_MEAS_INDEX       5

#elif defined _3D_TRACKER 
	#define POWER_MEAS_INDEX 3
	#define FLAG_MEAS_INDEX 4

#endif 



/* Multi-Level Tracker Gating Thresholds */
#define RANGE_INI_TH		2.0 / 2.0 
#define DOPPLER_INI_TH		1.5 * 0.2235 / 1.0
#define ANGLE_INI_TH		2.0 * 0.0524
/* Multi-Level Track Initiatization Thresholds */
#define RANGE_MEAS_GATE					5
#define DOPPLER_MEAS_GATE				0.8235f * 2.0f
#define ANGLE_MEAS_GATE					0.1024 * 2.0
/* Track Clustering Thresholds */
#define TRACK_CLUSTER_X_THRESHOLD	2
#define TRACK_CLUSTER_Z_THRESHOLD	4
#define TRACK_CLUSTER_D_THRESHOLD	1 * 3600 / 1609.34
/* Tracker Tuning (?) */
#define Q_SCALE						10.0f

#define TRACK_STATIONARY_SPEED		0.18
#define TRACK_STABILITY_PT			6
#define _4D_TRACK_OUTPUT			23
#define _3D_TRACK_OUTPUT			12

/* Range Rate LMS Filter Parameters */
#define TRACK_LMS_EPS 0.01
#define TRACK_LMS_MU 0.5
#define TRACK_LMS_BUFFER_SIZE 8
#define TRACK_LMS_NUM_PARA 7

/* Track Optimization Parameters */
#define XWR1843_DSP
#ifdef XWR1843_DSP
	#define MIN_RV_CLUSTER_TIME				3000
	#define TRACK_PREDICT_TIME				81	
#endif

#define RANGE_SCORE			10.0f
#define AGE_SCORE			20.0f
#define ASSOC_ELEMENT				5

/* Math Constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TINY 1e-20
/* Macro Messages */
#define RANGE_ERR_CHK

//#define RANGE_ERR_MSG(r1,c1,r2,c2) printf("%s:%d:\tINDEX ERROR -> \
//        (%d x %d) v. (%d x %d)\n", __FILE__, __LINE__, r1, c1, r2, c2)
//#define NOT_SQUARE() printf("%s:%d\tMatrix is not square\n",__FILE__,__LINE__)
//#define MEM_ERR() printf("%s:%d\tUnable to allocate memory\n",__FILE__,__LINE__)
//#define SINGULAR_ERR(f) printf("%s:%dSingular matrix in routine %s",__FILE__,__LINE__,f)

//
#define RANGE_ERR_MSG(r1,c1,r2,c2)
#define NOT_SQUARE()
#define MEM_ERR()
#define SINGULAR_ERR(f)

// TRACKER MATRIX
#define HALF_STATE			(N_STATE/2)
#define STATE_DOUBLE		(N_STATE * N_STATE)
#define STATE_DOUBLE_HALF   (N_STATE/2)*(N_STATE/2)
#define HALF_STATE_DOUBLE	(HALF_STATE * HALF_STATE)
#define HALF_STATE_THRID	((N_STATE / 2) * N_STATE) 
#define	MEASURE_STATE		(N_MEASURE * N_STATE)
#define	MEASURE_MEASURE		(N_MEASURE * N_MEASURE)

// TRACKER PREDICT
#define STACK_SZ_PREDT  ((N_STATE + 3u * STATE_DOUBLE + 4u * MEASURE_STATE + 1u * MEASURE_MEASURE) * 4u)
#define M_R_ALPHA_IDX	(2u*N_MEASURE + 2u)
#define M_R_BETA_IDX	(3u*N_MEASURE + 3u)
#define M_F_1ROW_X		(0u * N_STATE + 0u)
#define M_F_1ROW_Z		(0u * N_STATE + 4u)
#define M_F_2ROW_XDOT	(1u * N_STATE + 1u)
#define M_F_2ROW_ZDOT	(1u * N_STATE + 5u)
#define M_F_5ROW_X		(4u * N_STATE + 0u)
#define M_F_5ROW_Z		(4u * N_STATE + 4u)
#define M_F_6ROW_XDOT	(5u * N_STATE + 1u)
#define M_F_6ROW_ZDOT	(5u * N_STATE + 5u)

// UPDATE
#define STACK_SZ_TRKER_UPDATE ((N_MEASURE + 2u * N_STATE + 9u * HALF_STATE_DOUBLE + 5u * HALF_STATE_THRID + 2u * STATE_DOUBLE) * 4u)
#define STACK_SZ_TRK_UPDATE	  ((2u * N_MEASURE + N_STATE + 2u * MEASURE_STATE + STATE_DOUBLE) * 4u) 

// NEW
#define MAX_TRK_STATUS    (3u)

#define ASSOC_X_TH  (6.0f)
#define ASSOC_Y_TH  (4.0f)
#define ASSOC_Z_TH  (8.0f)

#define ASSOC_NORM_X	 (6.0f * 6.0f)
#define ASSOC_NORM_Y	 (4.0f * 4.0f)
#define ASSOC_NORM_Z     (8.0f * 8.0f)
#define ASSOC_NORM_RDOT  (0.8235f * 2.0f * 0.8235f * 2.0f)

#define TRK_MANAGE_R	 (50.0f)

#define DEG2RAD			 (float)(3.141592653f/180.0f)
#define RAD2DEG			 (float)(180.0f/3.141592653f)

#define EGO_BAND_TRK		(2.0f)
#define PI_HALF				(0.5f * M_PI)
#define PI_ONE_HALF			(1.5f * M_PI)
#define PI_DOUBLE			(2.0f * M_PI)

#define ROI_FLAG			(7)

#define NEW_TRK_FORMAT

#define RELEASE_MODE_TRK	

#endif  //TRACKER_SYSTEM_DEFINES_
