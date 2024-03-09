#ifndef __MOTORTYPEDEF_H__
#define __MOTORTYPEDEF_H__

#include "main.h"
#include "Bsp_can.h"
#include "Control.h"
#include "Binary.h"

typedef enum
{
    OPEN_LOOP = B0000,
    CURRENT_LOOP = B0001,
    SPEED_LOOP = B0010,
    ANGLE_LOOP = B0100,
    TORQUE_LOOP = B1000,

    // only for checking
    SPEED_AND_CURRENT_LOOP = B0011,
    ANGLE_AND_SPEED_LOOP = B0110,
    ALL_THREE_LOOP = B0111,
} CloseloopType_e;

typedef enum
{
    FEEDFORWARD_NONE = B00,
    CURRENT_FEEDFORWARD = B01,
    SPEED_FEEDFORWARD = B10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} FeedfowardType_e;
/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED,
} FeedbackSource_e;

/* 电机正反转标志 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1
} MotorReverseFlag_e;

/* 反馈量正反标志 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} FeedbackReverseFlag_e;
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    MF9025,
} MotorType_e;

typedef struct
{
    float *other_angle_feedback_ptr; 
    float *other_speed_feedback_ptr;

    float *speed_feedforward_ptr;
    float *current_feedforward_ptr; 

    PidInstance current_PID;
    PidInstance speed_PID;
    PidInstance angle_PID;
    PidInstance Torque_PID;

    float Ref; // 将会作为每个环的输入和输出顺次通过串级闭环
} MotorController_s;

typedef struct 
{
    CloseloopType_e OuterLoopType;              // 最外层的闭环,未设置时默认为最高级的闭环
    CloseloopType_e CloseLoopType;              // 使用几个闭环(串级)
    MotorReverseFlag_e MotorReverseFlag;       // 是否反转
    FeedbackReverseFlag_e FeedbackReverseFlag; // 反馈是否反向
    FeedbackSource_e AngleFeedbackSource;       // 角度反馈类型
    FeedbackSource_e SpeedFeedbackSource;       // 速度反馈类型
    FeedfowardType_e FeedForwardFlag;            // 前馈标志

}MotorControlSetting_s;

typedef struct MotorTypedef
{
    float *other_angle_feedback_ptr; // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr; // 速度反馈数据指针,单位为angle per sec

    float *speed_feedforward_ptr;   // 速度前馈数据指针
    float *current_feedforward_ptr; // 电流前馈数据指针

    PidInitConfig_s CurrentPid;
    PidInitConfig_s SpeedPid;
    PidInitConfig_s AnglePid;
    PidInitConfig_s TorquePid;
}MotorControllerInit_s;


typedef struct
{
    Motor_Working_Type_e StopFlag;
    MotorControllerInit_s ControllerParamInitConfig;
    MotorControlSetting_s ControllerSettingInitConfig;
    MotorType_e MotorType;
    CanInitConfig_s CanInitConfig;
} MotorInitConfig_s;

#endif

