#ifndef __MF9025_H
#define __MF9025_H

#include "main.h"
#include "Bsp_can.h"
#include "Control.h"
#include "string.h"
#include "struct_typedef.h"
#include "MotorTypedef.h"

// #define MultiMotorCmd    //多电机控制命令
#define SingleMotorCmd  //单电机控制命令
/* 单电机控制命令的PID参数 */
//MF9025电机速度环PID参数
#define MF9025SpeedPidKp    0.0f
#define MF9025SpeedPidKi    0.0f
//MF9025电机位置环PID参数
#define MF9025PositionPidKp 0.0f
#define MF9025PositionPidKi 0.0f
//MF9025电机转矩环PID参数
#define MF9025TurquePidKp   200.0f
#define MF9025TurquePidKi   50.0f

//MF9025电机数量
#define MF9025MotorQuantity 2

//轮子半径
#define WhellRadius 0.09f

//最大扭矩
#define MAXTORQUE 1.0f

/**
 * @brief 电机ID
 * 
 */
typedef enum
{
    MF9025AllID = 0x280,
    MF9025RightMotor = 0x141,
    MF9025LeftMotor = 0x142,
}MF9025MotorID;

/**
 * @brief MF9025单电机命令
 * 
 */
typedef enum
{
    TorqueControl = 0xA1,
    SpeedControl = 0xA2,
    AngleControl = 0XA6,
}MF9025Com;

/**
 * @brief MF025电机返回数据结构体
 * 
 */
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
}MF9025Measure;

typedef struct 
{
    uint8_t MessageNum;

    MotorController_s MF9025Controler;

    MotorType_e MotorType;
    MotorControlSetting_s MotorSetting;

    Motor_Working_Type_e MF9025StopFlag;
    MF9025Measure MF9025MotorMesure;
    CanInitConfig_s MF9025CanInitConfig;
    CANInstance *MF9025CanInstance;
}MF9025Instance;


MF9025Instance *MF9025Init(MotorInitConfig_s *Config);
extern void MF9025MeasureDispose(CANInstance *_Instance);
void MF9025MotorGroup(MF9025Instance *motor, CanInitConfig_s *config);
void MF9025Start(CANInstance *caninstance);
void MF9025Stop(CANInstance *caninstance);
void MF9025TorqueControl(CANInstance *caninstance, int16_t Torque);
void MF9025PidParaSet(CANInstance *caninstance);
void TorqueLimit(MF9025Instance *MotorTorque);
void MF9025MultiMotorCom(CANInstance *caninstance , uint16_t Current[4]);
void MF9025DataSet(MF9025Instance *PidSet ,fp32 ref);
void MF9025ControlLoop(void);
void test(void);

#endif

