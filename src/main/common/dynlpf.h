#pragma once

typedef enum {
  DYN_LPF_ON_RATIO = 0,
  DYN_LPF_ON_ERROR = 1,
} DYN_LPF_TYPE;

typedef enum {
  DYN_LPF_PT1 = 0,
  DYN_LPF_BIQUAD = 1,
} DYN_LPF_FILTER_TYPE;

#define DEFAULT_DYNLPF_ENABLE              1       //Enable DYN_LPF2 by default

#define DEFAULT_DYNLPF_CENTER_THRESHOLD    5.0f   //Value in °/s

#define DEFAULT_DYNLPF_FMIN                60.0f   //Fmin in Hz
#define DEFAULT_DYNLPF_FMAX               600.0f   //user Fmax in Hz
#define DEFAULT_DYNLPF_GAIN                70      //Gain

#define DEFAULT_DYNLPF_THROTTLE_GAIN       12      // 12Hz / % throrrle over 35%

#define DEFAULT_DYNLPF_FC_FC              10.0f     //Cut of freq on FC value


#define DEFAULT_DYNLPF_TYPE            DYN_LPF_ON_RATIO //Default

#define DEFAULT_DYNLPF_FILTER_TYPE      DYN_LPF_PT1     //Default to PT1, set to 1 for Biquad


extern void init_dynLpf(void);
extern float dynLpfApply(int axis, float input);
