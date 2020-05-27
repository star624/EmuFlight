
#include <stdbool.h>

#include "platform.h"
#include "math.h"

#include "dynLpf.h"

#include "fc/fc_rc.h"

#include "build/debug.h"

#include "common/filter.h"

#include "sensors/gyro.h"

#include "fc/rc_controls.h"

//DEFINITIONS
//-----------
#define DYNLPF_HYTEREIS  2.0f  //Value in °/s

//TYPES
//-----
  typedef struct {
      pt1Filter_t pt1;          //PT1 filter
      biquadFilter_t biquad;    //Biquad filter

      float Fc;                 //Cutoff freq
      pt1Filter_t pt1Fc;    //PT1 on Fc

      bool Dyn_Fc;         //Dynamic E or Fixed E
  } dynlpf_t;


//VARIABLES
//---------
    static dynlpf_t dynLpf[3];

    static float Fmin_init,Fmax;
    static float throttleGain;
    static float dynFcThreshold;
    static float dynGainOnError;

//////////////////////////////
//                          //
//       DYN PT1 INIT       //
//                          //
//////////////////////////////
void init_dynLpf(void)
{
    const float gyroDt = gyro.targetLooptime * 1e-6f;

    //Init PT1
    float gain = pt1FilterGain(gyroConfig()->dynlpf_fmin, gyroDt);
    pt1FilterInit(&dynLpf[0].pt1, gain);
    pt1FilterInit(&dynLpf[1].pt1, gain);
    pt1FilterInit(&dynLpf[2].pt1, gain);

    //Init Biquad
    biquadFilterInitLPF(&dynLpf[0].biquad, gyroConfig()->dynlpf_fmin, gyro.targetLooptime);
    biquadFilterInitLPF(&dynLpf[1].biquad, gyroConfig()->dynlpf_fmin, gyro.targetLooptime);
    biquadFilterInitLPF(&dynLpf[2].biquad, gyroConfig()->dynlpf_fmin, gyro.targetLooptime);

    //Fc filter
    gain = pt1FilterGain(gyroConfig()->dynlpf_fc_fc, gyroDt);
    pt1FilterInit(&dynLpf[0].pt1Fc, gain);
    pt1FilterInit(&dynLpf[1].pt1Fc, gain);
    pt1FilterInit(&dynLpf[2].pt1Fc, gain);

    dynLpf[0].Fc = gyroConfig()->dynlpf_fmin;
    dynLpf[1].Fc = gyroConfig()->dynlpf_fmin;
    dynLpf[2].Fc = gyroConfig()->dynlpf_fmin;

    dynLpf[0].Dyn_Fc = false;
    dynLpf[1].Dyn_Fc = false;
    dynLpf[2].Dyn_Fc = false;

    Fmax                 = (float)gyroConfig()->dynlpf_fmax;         //PT1 maxFc in Hz
    Fmin_init            = (float)gyroConfig()->dynlpf_fmin;         //PT1 min Fc in Hz

    throttleGain         = (float)gyroConfig()->dynlpf_throttle_gain;

    dynFcThreshold       = (float)(gyroConfig()->dynlpf_center_threshold);   //Min Setpoint & Gyro value to rise PT1 Fc
    dynGainOnError       = (float)(gyroConfig()->dynlpf_gain);
}

//////////////////////////////
//                          //
//      DYN LPF PROCESS     //
//         on ratio         //
//                          //
//////////////////////////////
#pragma GCC push_options
#pragma GCC optimize("O3")

FAST_CODE float dynlpf_process_type1(dynlpf_t* filter, float input, float target) {

float newFc, Fmin;
float throttle;
float Average;

Fmin = Fmin_init;
throttle  = (rcCommand[THROTTLE] - 1000.0f) * 0.1f; //Throttle scaled to [0-100]
const float gyroDt = gyro.targetLooptime * 1e-6f;

    //Compute average between setpoint and Gyro
    //-----------------------------------------
        Average = (target + input) * 0.5f;

    //Check if setpoint or gyro are high enought to compute "e" ratio
    //---------------------------------------------------------------
        if(filter->Dyn_Fc) {
            if ( (float)(fabs(Average)) < (dynFcThreshold - DYNLPF_HYTEREIS) ) {
                filter->Dyn_Fc = false;
            }
        }else{
            //Enable Dyn_Fc when stick or Quad move
            if ( (float)(fabs(Average)) > (dynFcThreshold + DYNLPF_HYTEREIS) ) {
                filter->Dyn_Fc = true;
            }
        }

    //Rise Fmin according to Throttle;
    //--------------------------------
            Fmin += throttle * throttleGain;

    //Compute e & Fc
    //--------------
        if(filter->Dyn_Fc) {

            //Avoid division by 0.0f
                if(target == 0.0f)              { target = 0.00001f; }
                if(filter->pt1.state == 0.0f)   { filter->pt1.state = 0.0001f; }

            //Compute e factor
                float Error, e;
                Error =  (float)fabs( target - input );
                e =  Error / Average;                           //Compute ratio between Error and average. e is image of noise in % of signal

            //New freq
                newFc = Fmin + dynGainOnError * 100.0f  * powf(e, 3.0f);  //"e" power 3 and multiply by a gain

        } else {
                newFc  = Fmin;
        }

    //Limit & Filter newFc
    //---------------------
        constrainf(newFc, Fmin, Fmax);

        //Filter the cut-off freq ;)
        newFc = pt1FilterApply(&filter->pt1Fc, newFc);

    //Update filter
    //------------------
    if (gyroConfig()->dynlpf_filter_type == DYN_LPF_PT1) {
    pt1FilterUpdateCutoff(&filter->pt1, pt1FilterGain(newFc, gyroDt));
    } else {
    biquadFilterUpdateLPF(&filter->biquad, newFc, gyro.targetLooptime);
    }
        filter->Fc = newFc;

    //Apply filter
    //------------
    float output;

    if (gyroConfig()->dynlpf_filter_type == DYN_LPF_PT1) {
        output = pt1FilterApply(&filter->pt1, input);
    } else {
        output = biquadFilterApplyDF1(&filter->biquad, input);
    }

 return output;
}


//////////////////////////////
//                          //
//      DYN LPF PROCESS     //
//         on error         //
//                          //
//////////////////////////////
FAST_CODE float dynlpf_process_type2(dynlpf_t* filter, float input, float target) {

float newFc, Fmin;
float throttle;

Fmin = Fmin_init;
throttle  = (rcCommand[THROTTLE] - 1000.0f) * 0.1f; //Throttle scaled to [0-100]
const float gyroDt = gyro.targetLooptime * 1e-6f;

    //Rise Fmin according to Throttle;
    //--------------------------------
            Fmin += throttle * throttleGain;

    //Compute new Fc according to error.
    //----------------------------------
        float Error, Min, scaleFactor;

        //Compute scale factor : scaleFactor = 1 near to 0°/s and 0,25 next to 1000°/s
            Min = (float)MIN(input, target);
            scaleFactor = 1.0f / (1.0f + 3.0f * (Min / 1000.0f));

        //Compute Error
            Error =  (float)fabs( target - input ) * scaleFactor;

        //newFc
            float e = Error * dynGainOnError * 0.01f;
            newFc = Fmin + powf(e, 4.0f);               //"e" power 4 to amplify big error.


    //Limit & Filter newFc
    //---------------------
        constrainf(newFc, Fmin, Fmax);

        //Filter the cut-off freq ;)
        newFc = pt1FilterApply(&filter->pt1Fc, newFc);


    //Update filter
    //------------------
    if(gyroConfig()->dynlpf_filter_type == DYN_LPF_PT1) {
    pt1FilterUpdateCutoff(&filter->pt1, pt1FilterGain(newFc, gyroDt));
    } else {
    biquadFilterUpdateLPF(&filter->biquad, newFc, gyro.targetLooptime);
    }
        filter->Fc = newFc;

    //Apply filter
    //------------
    float output;

    if (gyroConfig()->dynlpf_filter_type == DYN_LPF_PT1) {
        output = pt1FilterApply(&filter->pt1, input);
    } else {
        output = biquadFilterApplyDF1(&filter->biquad, input);
    }

 return output;
}


//////////////////////////////
//                          //
//      DYN LPF APPLY       //
//                          //
//////////////////////////////

FAST_CODE float dynLpfApply(int axis, float input)
{

float output;
float target = getSetpointRate(axis);


  //Apply filter if filter is enable.
    if(gyroConfig()->dynlpf_enable != 0) {
        if(gyroConfig()->dynlpf_type == DYN_LPF_ON_RATIO) {
              output = dynlpf_process_type1(&dynLpf[axis], input, target);
        } else {
              output = dynlpf_process_type2(&dynLpf[axis], input, target);
        }
    } else {
      output = input;
      dynLpf[axis].Fc = 0;  //To show filter is disable in blackbox.
    }


  //Blackbox
    // if(axis == 0) {
    //     DEBUG_SET(DEBUG_DYN_LPF, 0, (int16_t)(lrintf(input)));
    //     DEBUG_SET(DEBUG_DYN_LPF, 1, (int16_t)(lrintf(output)));
    //     DEBUG_SET(DEBUG_DYN_LPF, 2, (int16_t)(lrintf(dynLpf[axis].Fc)));
    //     DEBUG_SET(DEBUG_DYN_LPF, 3, (int16_t)(lrintf(target)));
    // }

return output;
}

#pragma GCC pop_options
