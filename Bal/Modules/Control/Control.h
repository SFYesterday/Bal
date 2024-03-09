#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "struct_typedef.h"
#include "Binary.h"
#include "math.h"

typedef enum
{
    PID_IMPROVE_NONE = B00000000,                // 0000 0000
    PID_Integral_Limit = B00000001,              // 0000 0001
    PID_Derivative_On_Measurement = B00000010,   // 0000 0010
    PID_Trapezoid_Intergral = B00000100,         // 0000 0100
    PID_Proportional_On_Measurement = B00001000, // 0000 1000
    PID_OutputFilter = B00010000,                // 0001 0000
    PID_ChangingIntegrationRate = B00100000,     // 0010 0000
    PID_DerivativeFilter = B01000000,            // 0100 0000
    PID_ErrorHandle = B10000000,                 // 1000 0000
} PidImprovement_e;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三个参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新项 1为上一项 2为上上项
    fp32 error[3]; //偏差项 0最新项 1为上一项 2为上上项

} PidInstance;


typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PidImprovement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PidInitConfig_s;

typedef struct Control
{
    fp32 Dis,TargetDis;
    fp32 Vel,TargetVel;
    fp32 Pitch,TargetPitch;
    fp32 Pitch_w;
    fp32 Theta,TargetTheta;
    fp32 Theta_w,TargetTheta_w;
}Lqr_s;

typedef __packed struct
{
    fp32 input;        
    fp32 out;          
    fp32 num[1];       
    fp32 frame_period; 
} first_order_filter_type_t;

extern void PID_Init(PidInstance *pid, uint8_t mode, PidInitConfig_s *PidInit);
extern fp32 PID_Calc(PidInstance *pid, fp32 ref, fp32 set);
extern void PID_clear(PidInstance *pid);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
fp32 LqrBanlanceAndSpeedCal(Lqr_s Lqr);
fp32 LqrTurnCal(Lqr_s Lqr);
#endif
