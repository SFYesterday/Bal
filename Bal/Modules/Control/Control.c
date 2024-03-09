#include "control.h"
#include "main.h"

fp32 Set1,Set2;

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Init(PidInstance *pid, uint8_t mode, PidInitConfig_s *PidInit)
{
    if (pid == NULL || &PidInit == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PidInit->Kp;
    pid->Ki = PidInit->Ki;
    pid->Kd = PidInit->Kd;
    pid->max_out = PidInit->MaxOut;
    pid->max_iout = PidInit->IntegralLimit;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_Calc(PidInstance *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {   
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(PidInstance *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
/**
 * @brief 一阶低通滤波初始化
 * 
 * @param first_order_filter_type 低通滤波结构体
 * @param frame_period 间隔时间
 * @param num 滤波参数
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}
/**
 * @brief 一阶低通滤波计算
 * 
 * @param first_order_filter_type 低通滤波结构体
 * @param input 输入
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}
//LQR速度和角度计算
fp32 LqrBanlanceAndSpeedCal(Lqr_s Lqr)
{
    static fp32 u;
    static fp32 K[4] = {-4.4721 ,-16.1293 ,-25.5075 ,-15.7246};
    //{-4.4721,-33.4123,-17.8933,-7.8016};
    //{-4.4721,-29.5764,-13.8113,-7.7952};
    //{-4.4721,-29.9712 ,-14.2084 ,-15.5418};
    Set1 = Lqr.TargetDis - Lqr.Dis;
    Set2 = Lqr.TargetVel - Lqr.Vel;
    u = -((Lqr.TargetDis - Lqr.Dis)*K[0] +(Lqr.TargetVel - Lqr.Vel) *K[1]) + (-Lqr.Pitch)*K[2] + (-Lqr.Pitch_w)*K[3];
    // u = (Lqr.TargetDis - Lqr.Dis)*K[0] +(Lqr.TargetVel - Lqr.Vel) *K[1] + (Lqr.TargetPitch -Lqr.Pitch)*K[2] + (-Lqr.Pitch_w)*K[3];
    return u;
}
fp32 LqrTurnCal(Lqr_s Lqr)
{
    static fp32 u;
    static fp32 k[2] = {-7.7460,-4.7464};
    u = (Lqr.TargetTheta - Lqr.Theta)*k[0] + (Lqr.TargetTheta_w - Lqr.Theta_w)*k[1];
    return u;
}

