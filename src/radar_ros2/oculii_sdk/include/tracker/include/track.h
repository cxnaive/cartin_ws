/* Copyright (c) Oculii 2017 */
#ifndef OFFR_TRACK_H_
#define OFFR_TRACK_H_

#include "tracker_defs.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

struct Track{

   uint8_t status;
   uint8_t type;
   //uint8_t lock;

   uint16_t assoc_buf[MAX_ASSIGN];
   
   uint8_t num_assoc;

   uint16_t id;
   uint16_t targetid;
   uint32_t age;
   uint16_t cnt;
   uint16_t missed;

   float    score;
   //float   wrap_buf[MAX_ASSIGN];

   float xx[N_STATE];
   //float PP[N_STATE*N_STATE];
   float zz[N_MEASURE];
   //float SS[N_MEASURE*N_MEASURE];
   //float SSinv[N_MEASURE*N_MEASURE];
   //float KK[N_MEASURE*N_STATE];
   float x[N_STATE];
   float P[N_STATE*N_STATE];
   float z[N_MEASURE];

   //float x1[N_STATE/2];
   //float x2[N_STATE/2+1];
   float centers[N_MEASURE];
   float S[N_MEASURE*N_MEASURE];
   float Sinv[N_MEASURE*N_MEASURE];
   float K[N_MEASURE*N_STATE];   

   //float lms_t[TRACK_LMS_BUFFER_SIZE + 1];
   //float lms_rdot[TRACK_LMS_BUFFER_SIZE + 1];
   //float lms_para[TRACK_LMS_NUM_PARA];   // a, b, c, d, a1, N1, beta
   //float lms_hv[3];
   float rdot_hat;
   //float max_speed;
   //float avg_speed;
   //uint8_t lms_ptr;

   //float dt;
   //float t;
   //float z0[N_MEASURE];
   float v[N_STATE / 2];

   float penalty_factor;
   //float rv_penalty_factor;

   //float dist;
   float av[N_STATE / 2];

   float power;

   uint8_t  u8Kienmtc;
   uint8_t  u8NewTarget;
   uint8_t  u8Measured;
   //uint8_t  u8Coast;


   void *next;
   uint16_t list_id;

   float lengthSize;
   float widthSize;
   float cluster_lateralSpeed;
   float cluster_longitudinalSpeed;
   float egoSpeed;
};

typedef struct Track Track;



/**
 * @brief 
 *
 * @param obj
 */
//void track_destroy(struct Track *obj);
//void track_ddestroy(struct Track **obj);


#endif
