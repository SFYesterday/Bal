/**
 * @file MF9025.c/h
 * @author Haojie Suo (1484132514@qq.com)
 * @brief 9025电机的控制函数
 * @version 0.1
 * @date 2023-11-16
 * 
 * @copyright 版权归属成都大学Ultra战队
 * 特别鸣谢湖南大学（哈哈，其实抄袭了好多）
 */
#include "MF9025.h"

extern CAN_HandleTypeDef hcan1;
//MF9025电机注册
static MF9025Instance *MF9025MotorInstace[MF9025MotorQuantity] = {NULL};
//注册时会自增，保存该实例
static uint8_t IDx;    
//发送ID设置，因为不用接收，不必在Bsp_can层注册
#ifdef MultiMotorCmd
static CANInstance MF9025Send = {.CanHandle = &hcan1, .TxConf.StdId = 0x280, .TxConf.IDE = CAN_ID_STD,
                                     .TxConf.RTR = CAN_RTR_DATA, .TxConf.DLC = 0x08, .TxBuff = {0}}; 
#endif
fp32 data;
/**
 * @brief MF9025电机初始化函数
 * 
 */
MF9025Instance *MF9025Init(MotorInitConfig_s *Config)
{
    MF9025Instance *_Instance = (MF9025Instance *)malloc(sizeof(MF9025Instance));
    memset(_Instance, 0, sizeof(MF9025Instance));

    _Instance->MotorType = Config->MotorType;
    _Instance->MotorSetting = Config->ControllerSettingInitConfig;
    //Pid初始化
    // MF9025PidParaSet(_Instance->MF9025CanInstance);
    PID_Init(&_Instance->MF9025Controler.current_PID,PID_POSITION,&Config->ControllerParamInitConfig.CurrentPid);
    PID_Init(&_Instance->MF9025Controler.speed_PID,PID_POSITION,&Config->ControllerParamInitConfig.SpeedPid);
    PID_Init(&_Instance->MF9025Controler.angle_PID,PID_POSITION,&Config->ControllerParamInitConfig.AnglePid);
    PID_Init(&_Instance->MF9025Controler.Torque_PID,PID_POSITION,&Config->ControllerParamInitConfig.TorquePid);

    _Instance->MF9025Controler.other_angle_feedback_ptr = Config->ControllerParamInitConfig.other_angle_feedback_ptr;
    _Instance->MF9025Controler.other_speed_feedback_ptr = Config->ControllerParamInitConfig.other_speed_feedback_ptr;
    _Instance->MF9025Controler.current_feedforward_ptr = Config->ControllerParamInitConfig.current_feedforward_ptr;
    _Instance->MF9025Controler.speed_feedforward_ptr = Config->ControllerParamInitConfig.speed_feedforward_ptr;

    MF9025MotorGroup(_Instance, &Config->CanInitConfig);

    Config->CanInitConfig.CanMoudleCallBack = MF9025MeasureDispose;
    Config->CanInitConfig.ID = _Instance;
    _Instance->MF9025CanInstance = CANRegister(&Config->CanInitConfig);

    // MF9025PidParaSet(_Instance->MF9025CanInstance);  //单电机控制命令的Pid参数设置

    _Instance->MF9025StopFlag = MOTOR_ENALBED;

    MF9025MotorInstace[IDx++] = _Instance;
    return _Instance;
}
/**
 * @brief MF9025电机数据处理函数
 * 
 * @param _Instance CAN实例
 */
static void MF9025MeasureDispose(CANInstance *_Instance)
{
    /*具体为什么这么使用，请看湖南大学跃鹿战队代码框架开源*/
    uint8_t *RxBuff = _Instance->RxBuff;
    MF9025Instance *MF9025 = (MF9025Instance *)_Instance->ID;
    MF9025Measure *Measure = &MF9025->MF9025MotorMesure;
    /*电机数据计算，详解看MF9025电机CAN通信协议书*/
    Measure->temp = RxBuff[1];
    Measure->current = (int16_t)(RxBuff[3]<<8 | RxBuff[2]);
    Measure->speed = (int16_t)(RxBuff[5] <<8 | RxBuff[4]);
    Measure->encoder = (uint16_t)(RxBuff[7] <<8 | RxBuff[6]);
    //实际角度和速度计算
    Measure->RealSpeed = ((Measure->speed)/360.0f) *2.0f* PI *WhellRadius;//这里是计算了线速度，看需求使用

    Measure->RealAngle = (Measure->encoder)/65536.0f*360.0f;
    Measure->RealTorque = (Measure->current)*(16.5f/2048.0f)*0.32f;
    if (Measure->encoder - Measure->LastEcd > 32768)
        Measure->TotalCir--;
    else if (Measure->encoder - Measure->LastEcd < -32768)
        Measure->TotalCir++;
    Measure->LastEcd = Measure->encoder;
    Measure->TotalAngle = Measure->TotalCir * 360 + Measure->RealAngle;
}

void MF9025MotorGroup(MF9025Instance *motor, CanInitConfig_s *config)
{
    uint8_t Id = config->TxID - 1;
    uint8_t SendNum;
    SendNum = Id;
    config->TxID = 0x140 + Id + 1;
    config->RxID = 0x140 + Id + 1;   // 把ID+1,进行分组设置
    motor->MessageNum = SendNum;
}

/**
 * @brief MF9025电机启动命令
 * 
 * @param caninstance can实例
 */
void MF9025Start(CANInstance *caninstance)
{
    uint8_t StartData[8] = {0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//电机启动命令
    memcpy(caninstance->TxBuff ,StartData ,8);
    CanTransmit(caninstance);
}

/**
 * @brief MF9025电机停止命令
 * 
 * @param caninstance can实例
 */
void MF9025Stop(CANInstance *caninstance)
{
    uint8_t StartData[8] = {0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//电机停止命令
    memcpy(caninstance->TxBuff ,StartData ,8);
    CanTransmit(caninstance);
}

/**
 * @brief MF9025电机扭矩控制
 * 
 * @param caninstance 
 * @param Torque 
 */
void MF9025TorqueControl(CANInstance *caninstance, int16_t Torque)
{
    uint8_t TorqueData[8] = {0};
    TorqueData[0] = TorqueControl;
    TorqueData[4] = (uint8_t)(Torque);
    TorqueData[5] = (uint8_t)(Torque >> 8);
    memcpy(caninstance->TxBuff ,TorqueData ,8);
    CanTransmit(caninstance);
}
/**
 * @brief MF9025单电机电机pid参数设置
 * 
 * @param caninstance can实例
 */
void MF9025PidParaSet(CANInstance *caninstance)
{
    uint8_t PidData[8] = {0x31 , 0x00 , MF9025PositionPidKp , MF9025PositionPidKi ,\
                          MF9025SpeedPidKp , MF9025SpeedPidKi , MF9025TurquePidKp , MF9025TurquePidKi};
    memcpy(caninstance->TxBuff ,PidData ,8);
    CanTransmit(caninstance);
}


/**
 * @brief MF9025多电机控制，不过没有用到
 * 
 * @param caninstance can实例
 * @param Current 电流值
 */
void MF9025MultiMotorCom(CANInstance *caninstance , uint16_t Current[4])
{
    uint8_t iqControl[8];
    iqControl[0] = (uint8_t)Current[0];
    iqControl[1] = (uint8_t)Current[0]>>8;
    iqControl[2] = (uint8_t)Current[1];
    iqControl[3] = (uint8_t)Current[1]>>8;
    iqControl[4] = (uint8_t)Current[2];
    iqControl[5] = (uint8_t)Current[2]>>8;
    iqControl[6] = (uint8_t)Current[3];
    iqControl[7] = (uint8_t)Current[3]>>8;
    memcpy(caninstance->TxBuff ,iqControl ,8);
    CanTransmit(caninstance);
}

//扭矩和扭矩电流的关系 T=Ki MF9025电机扭矩常数为1.4N*M/A，给定的扭矩对应关系为1：16，即16mA对应1
//给定的参数关系计算关系式为X=T/K/16
/**
 * @brief MF9025电机参数设置
 * 
 * @param PidSet 
 * @param ref 
 */
void MF9025DataSet(MF9025Instance *PidSet ,fp32 ref)
{
    PidSet->MF9025Controler.Ref = ref;
}
/**
 * @brief MF9025电机循环控制
 * 
 */
void MF9025ControlLoop()
{
    #ifdef SingleMotorCmd
        MF9025Instance *motor;
        MotorControlSetting_s *motor_setting; // 电机控制参数
        MotorController_s *motor_controller;   // 电机控制器
        fp32 set;
        for (size_t i = 0; i < IDx; ++i)
        { // 减小访存开销,先保存指针引用
            motor = MF9025MotorInstace[i];
            motor_setting = &motor->MotorSetting;
            motor_controller = &motor->MF9025Controler;
            set = motor_controller->Ref;
            if (motor_setting->MotorReverseFlag == MOTOR_DIRECTION_REVERSE)
            set *= -1; // 设置反转
            if (motor->MF9025StopFlag == MOTOR_STOP)
            set = 0;
            data = set;
            MF9025TorqueControl(motor->MF9025CanInstance,set);
        }
    #endif
    #ifdef MultiMotorCmd
        // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
        uint8_t num; // 电机组号和组内编号
        int16_t set;        // 电机控制CAN发送设定值
        MF9025Instance *motor;
        MotorControlSetting_s *motor_setting; // 电机控制参数
        MotorController_s *motor_controller;   // 电机控制器
        MF9025Measure *measure;           // 电机测量值
        float pid_measure, Ref;             // 电机PID测量值和设定值
        // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
        for (size_t i = 0; i < IDx; ++i)
        { // 减小访存开销,先保存指针引用
            motor = MF9025MotorInstace[i];
            motor_setting = &motor->MotorSetting;
            motor_controller = &motor->MF9025Controler;
            measure = &motor->MF9025MotorMesure;
            Ref = motor_controller->Ref; // 保存设定值,防止motor_controller->Ref在计算过程中被修改
            if (motor_setting->MotorReverseFlag == MOTOR_DIRECTION_REVERSE)
                Ref *= -1; // 设置反转

            // Ref会顺次通过被启用的闭环充当数据的载体
            // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
            if ((motor_setting->CloseLoopType & ANGLE_LOOP) && motor_setting->OuterLoopType == ANGLE_LOOP)
            {
                if (motor_setting->AngleFeedbackSource == OTHER_FEED)
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                else
                    pid_measure = measure->TotalAngle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
                // 更新Ref进入下一个环
                Ref = PID_Calc(&motor_controller->angle_PID, pid_measure, Ref);
            }

            // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
            if ((motor_setting->CloseLoopType & SPEED_LOOP) && (motor_setting->OuterLoopType & (ANGLE_LOOP | SPEED_LOOP)))
            {
                if (motor_setting->FeedForwardFlag & SPEED_FEEDFORWARD)
                    Ref += *motor_controller->speed_feedforward_ptr;

                if (motor_setting->SpeedFeedbackSource == OTHER_FEED)
                    pid_measure = *motor_controller->other_speed_feedback_ptr;
                else // MOTOR_FEED
                    pid_measure = measure->RealSpeed;
                // 更新Ref进入下一个环
                Ref = PID_Calc(&motor_controller->speed_PID, pid_measure, Ref);
            }
            if ((motor_setting->CloseLoopType & TORQUE_LOOP) && (motor_setting->OuterLoopType == TORQUE_LOOP))
            {
                pid_measure = measure->RealTorque;
                Ref = PID_Calc(&motor_controller->Torque_PID,pid_measure, Ref);
            }
            // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
            if (motor_setting->FeedForwardFlag & CURRENT_FEEDFORWARD)
            {
                Ref += *motor_controller->current_feedforward_ptr;
            }
            if (motor_setting->CloseLoopType & CURRENT_LOOP)
            {
                Ref = PID_Calc(&motor_controller->current_PID, measure->current, Ref);
            }

            if (motor_setting->FeedbackReverseFlag == FEEDBACK_DIRECTION_REVERSE)
            {
                Ref *= -1;
            }

            // 获取最终输出
            set = (int16_t)Ref;
            // 分组填入发送数据
            num = motor->MessageNum;
            MF9025Send.TxBuff[num*2] = (uint8_t)(set);         // 低八位
            MF9025Send.TxBuff[num*2 + 1] = (uint8_t)((set >>8)+1); // 高八位

            // 若该电机处于停止状态,直接将buff置零
            if (motor->MF9025StopFlag == MOTOR_STOP)
                memset(MF9025Send.TxBuff + 2 * num, 0, 16u);
        }

        CanTransmit(&MF9025Send);
    #endif
}

// void test(void)
// {
//     send->TxBuff[0] = 0xf4;
//     send->TxBuff[1] = 0x01;
//     send->TxBuff[2] = 0xf4;
//     send->TxBuff[3] = 0x01;
//     send->TxBuff[4] = 0xf4;
//     send->TxBuff[5] = 0x01;
//     send->TxBuff[6] = 0xf4;
//     send->TxBuff[7] = 0x01;
//     CanTransmit(&MF9025Send);
// }
