#ifndef __DJMOTOR_H
#define __DJMOTOR_H

#include "main.h"
#include "Bsp_can.h"
#include "struct_typedef.h"
#include "Control.h"
#include "MotorTypedef.h"
//GM3508电机速度环PID参数
#define GM3508SpeedPidKp 0.0f
#define GM3508SpeedPidKi 0.0f
#define GM3508SpeedPidKd 0.0f
//GM3508电机角度环PID参数，作摩擦轮并未使用
#define GM3508AnglePidKp 0.0f
#define GM3508AnglePidKi 0.0f
#define GM3508AnglePidKd 0.0f

//电机数量设置
#define GM3508MotorNum 2
//电机直径，暂未知道摩擦轮直径，后续可能需要修改
#define DJMotorRadius 0.021f

#define DJMotorQuantity 7

typedef enum{
    GM3508AllID = 0x200,
    GM3508RightFrictionWheels = 0x201,
    GM3508LeftFrictionWheels = 0x202,
}GM3508MotorID;

typedef struct 
{
    //电机返回数据
    int8_t temp;
    int16_t current;
    int16_t speed;
    uint16_t encoder;

    fp32 TotalAngle;    //总角度
    int32_t TotalCir;   //总圈数

    uint16_t LastEcd;
    //计算实际速度
    fp32 RealSpeed;
    fp32 RealAngle;
    fp32 RealTorque;
}DJMotorMeasure;

typedef struct 
{
    uint8_t SendGroup;
    uint8_t MessageNum;

    MotorController_s DJMotorControler;

    MotorType_e MotorType;
    MotorControlSetting_s MotorSetting;

    Motor_Working_Type_e DJMotorStopFlag;
    DJMotorMeasure DJMotorMesure;
    CanInitConfig_s DJMotorCanInitConfig;
    CANInstance *DJMotorCanInstance;
}DJMotorInstance;

static void DJMotorMeasureDispose(CANInstance *_Instance);
static void MotorSenderGrouping(DJMotorInstance *motor, CanInitConfig_s *config);
DJMotorInstance *DJMotorInit(MotorInitConfig_s *Config);


#endif

