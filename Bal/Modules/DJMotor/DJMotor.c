/**
 * @file 3508.c/h
 * @author Haojie Suo (1484132514@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-16
 * 
 * @copyright 版权归属成都大学Ultra战队所有
 * 
 */
#include "DJMotor.h"
#include "Bsp_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
uint8_t IDx;
static DJMotorInstance *DJMotorinstance[DJMotorQuantity] = {NULL};
static CANInstance SenderAssignment[6] = {
    [0] = {.CanHandle = &hcan1, .TxConf.StdId = 0x1ff, .TxConf.IDE = CAN_ID_STD, .TxConf.RTR = CAN_RTR_DATA, .TxConf.DLC = 0x08, .TxBuff = {0}},
    [1] = {.CanHandle = &hcan1, .TxConf.StdId = 0x200, .TxConf.IDE = CAN_ID_STD, .TxConf.RTR = CAN_RTR_DATA, .TxConf.DLC = 0x08, .TxBuff = {0}},
    [2] = {.CanHandle = &hcan1, .TxConf.StdId = 0x2ff, .TxConf.IDE = CAN_ID_STD, .TxConf.RTR = CAN_RTR_DATA, .TxConf.DLC = 0x08, .TxBuff = {0}},
    [3] = {.CanHandle = &hcan2, .TxConf.StdId = 0x1ff, .TxConf.IDE = CAN_ID_STD, .TxConf.RTR = CAN_RTR_DATA, .TxConf.DLC = 0x08, .TxBuff = {0}},
    [4] = {.CanHandle = &hcan2, .TxConf.StdId = 0x200, .TxConf.IDE = CAN_ID_STD, .TxConf.RTR = CAN_RTR_DATA, .TxConf.DLC = 0x08, .TxBuff = {0}},
    [5] = {.CanHandle = &hcan2, .TxConf.StdId = 0x2ff, .TxConf.IDE = CAN_ID_STD, .TxConf.RTR = CAN_RTR_DATA, .TxConf.DLC = 0x08, .TxBuff = {0}},
};
static uint8_t SendGroupEnable[6] = {0};

static void DJMotorMeasureDispose(CANInstance *_Instance)
{
    /*具体为什么这么使用，请看湖南大学跃鹿战队代码框架开源*/
    uint8_t *RxBuff = _Instance->RxBuff;
    DJMotorInstance *DJMotor = (DJMotorInstance *)_Instance->ID;
    DJMotorMeasure *Measure = &DJMotor->DJMotorMesure;
    /*电机数据计算，详解请看C620电调CAN通信协议*/
    Measure->encoder = (uint16_t)RxBuff[0]<<8 | RxBuff[1];
    Measure->speed = (uint16_t)RxBuff[2]<<8 | RxBuff[3];
    Measure->current = (uint16_t)RxBuff[4]<<8 | RxBuff[5];
    Measure->temp = RxBuff[6];
    /*这里是3508电机作摩擦轮电机使用，无其他使用方式，直接计算，9025电机同理*/
    Measure->RealSpeed = (Measure->speed)/60.0f*PI*DJMotorRadius*DJMotorRadius;
    Measure->RealAngle = (Measure->encoder)/8191.0f*360.0f;
}

static void MotorSenderGrouping(DJMotorInstance *motor, CanInitConfig_s *config)
{
    uint8_t MotorID = config->TxID - 1; // 下标从零开始,先减一方便赋值
    uint8_t MotorSendNum;
    uint8_t MotorGrouping;
    switch (motor->MotorType)
    {
    case M3508:
    case M2006:
        if (MotorID < 4) // 根据ID分组
        {
            MotorSendNum = MotorID;
            MotorGrouping = config->CanHandle == &hcan1 ? 1 : 4;
        }
        else
        {
            MotorSendNum = MotorID - 4;
            MotorGrouping = config->CanHandle == &hcan1 ? 0 : 3;
        }

        // 计算接收id并设置分组发送id
        config->RxID = 0x200 + MotorID + 1;   // 把ID+1,进行分组设置
        SendGroupEnable[MotorGrouping] = 1; // 设置发送标志位,防止发送空帧
        motor->MessageNum = MotorSendNum;
        motor->SendGroup = MotorGrouping;
        
        break;
    case GM6020:
        if (MotorID < 4)
        {
            MotorSendNum = MotorID;
            MotorGrouping = config->CanHandle == &hcan1 ? 0 : 3;
        }
        else
        {
            MotorSendNum = MotorID - 4;
            MotorGrouping = config->CanHandle == &hcan1 ? 2 : 5;
        }

        config->RxID = 0x204 + MotorID + 1;   // 把ID+1,进行分组设置
        SendGroupEnable[MotorGrouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->MessageNum = MotorSendNum;
        motor->SendGroup = MotorGrouping;
        break;
    default:
        break;
    }
}

DJMotorInstance *DJMotorInit(MotorInitConfig_s *Config) 
{
    DJMotorInstance *_Instance = (DJMotorInstance *)malloc(sizeof(DJMotorInstance));
    memset(_Instance, 0, sizeof(DJMotorInstance));

    _Instance->MotorType = Config->MotorType;
    _Instance->MotorSetting = Config->ControllerSettingInitConfig;
    //Pid初始化
    // DJMotroPidParaSet(_Instance->DJMotroCanInstance);
    PID_Init(&_Instance->DJMotorControler.current_PID,PID_POSITION,&Config->ControllerParamInitConfig.CurrentPid);
    PID_Init(&_Instance->DJMotorControler.speed_PID,PID_POSITION,&Config->ControllerParamInitConfig.SpeedPid);
    PID_Init(&_Instance->DJMotorControler.angle_PID,PID_POSITION,&Config->ControllerParamInitConfig.AnglePid);
    PID_Init(&_Instance->DJMotorControler.Torque_PID,PID_POSITION,&Config->ControllerParamInitConfig.TorquePid);

    _Instance->DJMotorControler.other_angle_feedback_ptr = Config->ControllerParamInitConfig.other_angle_feedback_ptr;
    _Instance->DJMotorControler.other_speed_feedback_ptr = Config->ControllerParamInitConfig.other_speed_feedback_ptr;
    _Instance->DJMotorControler.current_feedforward_ptr = Config->ControllerParamInitConfig.current_feedforward_ptr;
    _Instance->DJMotorControler.speed_feedforward_ptr = Config->ControllerParamInitConfig.speed_feedforward_ptr;

    MotorSenderGrouping(_Instance, &Config->CanInitConfig);

    Config->CanInitConfig.CanMoudleCallBack = DJMotorMeasureDispose;
    Config->CanInitConfig.ID = _Instance;
    _Instance->DJMotorCanInstance = CANRegister(&Config->CanInitConfig);

    // DJMotroPidParaSet(_Instance->DJMotroCanInstance);  //单电机控制命令的Pid参数设置

    _Instance->DJMotorStopFlag = MOTOR_ENALBED;

    DJMotorinstance[IDx++] = _Instance;
    return _Instance;
}

void DJMotorStop(DJMotorInstance *_Instance)
{
    _Instance->DJMotorStopFlag = MOTOR_STOP;
}

void DJMotorControlLoop()
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJMotorInstance *motor;
    MotorControlSetting_s *motor_setting; // 电机控制参数
    MotorController_s *motor_controller;   // 电机控制器
    DJMotorMeasure *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < IDx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = DJMotorinstance[i];
        motor_setting = &motor->MotorSetting;
        motor_controller = &motor->DJMotorControler;
        measure = &motor->DJMotorMesure;
        pid_ref = motor_controller->Ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->MotorReverseFlag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->CloseLoopType & ANGLE_LOOP) && motor_setting->OuterLoopType == ANGLE_LOOP)
        {
            if (motor_setting->AngleFeedbackSource == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->TotalAngle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PID_Calc(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->CloseLoopType & SPEED_LOOP) && (motor_setting->OuterLoopType & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->FeedForwardFlag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->SpeedFeedbackSource == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->RealSpeed;
            // 更新pid_ref进入下一个环
            pid_ref = PID_Calc(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->FeedForwardFlag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->CloseLoopType & CURRENT_LOOP)
        {
            pid_ref = PID_Calc(&motor_controller->current_PID, measure->current, pid_ref);
        }

        if (motor_setting->FeedbackReverseFlag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        set = (int16_t)pid_ref;

        // 分组填入发送数据
        group = motor->SendGroup;
        num = motor->MessageNum;
        SenderAssignment[group].TxBuff[2 * num] = (uint8_t)(set >> 8);         // 低八位
        SenderAssignment[group].TxBuff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->DJMotorStopFlag == MOTOR_STOP)
            memset(SenderAssignment[group].TxBuff + 2 * num, 0, 16u);
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i)
    {
        if (SendGroupEnable[i])
        {
            CanTransmit(&SenderAssignment[i]);
        }
    }
}




