#include "Chassis_task.h"
/**
 * @brief 遥控器死区限制
 * 
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
extern CAN_HandleTypeDef hcan1;
//底盘信息结构体定义
ChassisTask_s ChassisMove;
//左右电机指针
MF9025Instance *RightMotor, *LeftMotor;
fp32 Test , Test1;
const static fp32 ChassisCurrent[1] = {0.3333333333f};
void ChassisTask(void const *pvParameters)
{
    ChassisInit();
    vTaskDelay(374);
    while (1)
    {
        //根据遥控器选择模式
        ChassisModeSet(&ChassisMove);
        //根据模式选择控制函数，函数内部后续需要修改
        ChassisBehaviourControl(&ChassisMove);
        //Lqr数据更新
        LqrDataUpdata(&ChassisMove);
        #ifdef LqrControl
        ChassisMove.BalTorque = LqrBanlanceAndSpeedCal(ChassisMove.LqrData);
        // Lqr输出控制量，控制转向
        // ChassisMove.SteerTorque = LqrTurnCal(ChassisMove.LqrData);
        // Pid输出转向控制量
        // Test = ChassisMove.Chassis_RC->rc.ch[2]*0.001309f;
        // Test1 = -ChassisMove.LqrData.Theta_w;
        // ChassisMove.SteerTorque = PID_Calc(&RightMotor->MF9025Controler.Torque_PID, Test1, Test);
        #endif
        #ifdef PidControl
        //这里的Pid的句柄是暂时写的，只是为了验证，实际使用需要更改
        ChassisMove.BalTorque = -PID_Calc(&RightMotor->MF9025Controler.angle_PID, ChassisMove.LqrData.Pitch,ChassisMove.LqrData.TargetPitch);
        // ChassisMove.SpeedTorque = -PID_Calc(&RightMotor->MF9025Controler.speed_PID, ChassisMove.LqrData.Vel,ChassisMove.LqrData.TargetVel);
        ChassisMove.SteerTorque = PID_Calc(&RightMotor->MF9025Controler.current_PID, -ChassisMove.LqrData.Theta_w, ChassisMove.Chassis_RC->rc.ch[2]*0.001309f);
        #endif
        // first_order_filter_cali(ChassisMove.CurrentFOF,ChassisMove.BalTorque);
        //根据LQR计算的力矩，计算左右轮力矩
        ChassisWheelTorqueSet(&ChassisMove);
        //力矩参数赋值
        MF9025DataSet(RightMotor,ChassisMove.RightTorqueCurrent);
        MF9025DataSet(LeftMotor,ChassisMove.LeftTorqueCurrent);
        //9025电机循环
        MF9025ControlLoop();     
        vTaskDelay(2);
    }
}

/**
 * @todo 这样的赋值太不优雅了，希望来个更优雅的赋值
 * @brief 底盘初始化
 * 
 */
void ChassisInit(void)
{
    ChassisMove.Chassis_RC = get_remote_control_point();
    ChassisMove.Ins_Angle = get_INS_angle_point();
    ChassisMove.GyroData = get_gyro_data_point();
    ChassisMove.AccelData = get_accel_data_point();
    ChassisMove.LqrData.Dis = ChassisMove.LqrData.Pitch = ChassisMove.LqrData.Pitch_w = ChassisMove.LqrData.Vel = 0;
    ChassisMove.LqrData.TargetDis = ChassisMove.LqrData.TargetVel = 0;
    MotorInitConfig_s MotorInitConfig = 
    {
        .CanInitConfig.CanHandle = &hcan1,
        .ControllerParamInitConfig = 
        {
            .AnglePid = 
            {
                .Kp = 1.5f,
                .Ki = 0.0f,
                .Kd = 150.0f,
                .IntegralLimit = 1.3f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 2.0f,
            },
            .SpeedPid = 
            {
                .Kp = 3.0f,
                .Ki = 0.0F,
                .Kd = 0.0f,
                .IntegralLimit = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 1.0f,
            },
            .CurrentPid = //这里先暂用电流环pid做转向pid
            {
                .Kp = 3.0f,
                .Ki = 0.0F,
                .Kd = 0.0f,
                .IntegralLimit = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 1.0f,
            }
        },
        .ControllerSettingInitConfig = 
        {
            .AngleFeedbackSource = MOTOR_FEED,
            .SpeedFeedbackSource = MOTOR_FEED,
            .OuterLoopType = TORQUE_LOOP,
            .CloseLoopType = TORQUE_LOOP ,
        },
        .MotorType = MF9025,
    };
    MotorInitConfig.CanInitConfig.TxID = 1;
    MotorInitConfig.ControllerSettingInitConfig.MotorReverseFlag = MOTOR_DIRECTION_NORMAL;
    LeftMotor = MF9025Init(&MotorInitConfig);
    MotorInitConfig.CanInitConfig.TxID = 2;
    MotorInitConfig.ControllerSettingInitConfig.MotorReverseFlag = MOTOR_DIRECTION_REVERSE;
    RightMotor = MF9025Init(&MotorInitConfig);
    first_order_filter_init(ChassisMove.CurrentFOF,0.02f,ChassisCurrent);

}
/**
 * @brief 扭矩限幅
 * 
 * @param ChassisTorque 
 */
void ChassisWheelTorqueCurrentLimit(ChassisTask_s *ChassisTorque,fp32 count)
{
    if (ChassisTorque->LeftTorqueCurrent > count)
    {
        ChassisTorque->LeftTorqueCurrent = count;
    }else if (ChassisTorque->LeftTorqueCurrent < -count)
    {
        ChassisTorque->LeftTorqueCurrent = -count;
    }
    if (ChassisTorque->RightTorqueCurrent > count)
    {
        ChassisTorque->RightTorqueCurrent = count;
    }else if (ChassisTorque->RightTorqueCurrent < -count)
    {
        ChassisTorque->RightTorqueCurrent = -count;
    }
}
/**
 * @brief 底盘各个轮子扭据计算，并计算扭矩电流赋值
 * 
 * @param ChassisTorque 
 */
void ChassisWheelTorqueSet(ChassisTask_s *ChassisTorque)
{
    //扭矩计算
    #ifdef LqrControl
    ChassisTorque->LeftTorque = 0.5f * ChassisTorque->BalTorque + 0.5f * 0.85f * ChassisTorque->SteerTorque;
    ChassisTorque->RightTorque = 0.5f * ChassisTorque->BalTorque - 0.5f * ChassisTorque->SteerTorque;
    #endif
    #ifdef PidControl
    ChassisTorque->LeftTorque = ChassisTorque->BalTorque - 0.2f * ChassisTorque->SpeedTorque + 0.5f * 1.05f*ChassisTorque->SteerTorque;
    ChassisTorque->RightTorque = ChassisTorque->BalTorque - 0.2f * 1.2f * ChassisTorque->SpeedTorque - 0.5f * ChassisTorque->SteerTorque;
    #endif
    //转换为扭矩电流
    ChassisTorque->LeftTorqueCurrent = ChassisTorque->LeftTorque*MF9025TORQUEPROPORTION;
    ChassisTorque->RightTorqueCurrent = ChassisTorque->RightTorque*MF9025TORQUEPROPORTION;
    ChassisWheelTorqueCurrentLimit(ChassisTorque,1500);
}
/**
 * @brief 底盘模式设置
 * 
 * @param ChassisModeSet 
 * 感觉可以改成switch结构，不知道会不会速度快一点
 */
void ChassisModeSet(ChassisTask_s *ChassisModeSet)
{
    if (ChassisModeSet->Chassis_RC->rc.s[CHASSIS_MODE_CHANNEL] == NULL)
    {
        ChassisModeSet->ChassisMode = CHASSIS_NO_MOVE;
    }
    else if(switch_is_up(ChassisModeSet->Chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        ChassisModeSet->ChassisMode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else if (switch_is_mid(ChassisModeSet->Chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        ChassisModeSet->ChassisMode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(ChassisModeSet->Chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        ChassisModeSet->ChassisMode = CHASSIS_ZERO_FORCE;
    }
}
/**
 * @brief 根据控制模式选择行为方式
 * 
 * @param ChassisModeSet 
 */
void ChassisBehaviourControl(ChassisTask_s *ChassisModeSet)
{
    switch (ChassisModeSet->ChassisMode)
    {
    case CHASSIS_NO_MOVE:
        ChassisNoMove(ChassisModeSet);
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        ChassisFollowGimbalYaw(ChassisModeSet);
        break;
    case CHASSIS_NO_FOLLOW_YAW:
        ChassisNoFollowGimbalYaw(ChassisModeSet);
        break;
    case CHASSIS_ZERO_FORCE:
        ChassisZeroForce();
        break;
    default:
        break;
    }
}
/**
 * @brief 根据遥控器的值计算速度
 * 
 * @param ChassisRCtoVel 
 */
void ChassisDataSet(ChassisTask_s *ChassisRCtoVel)
{
    fp32 VxSetChannel, VySetChannel;
    rc_deadband_limit(ChassisRCtoVel->Chassis_RC->rc.ch[CHASSIS_X_CHANNEL], VxSetChannel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(ChassisRCtoVel->Chassis_RC->rc.ch[CHASSIS_Z_CHANNEL], VySetChannel, CHASSIS_RC_DEADLINE);
    //键盘控制
    if (ChassisRCtoVel->Chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        VxSetChannel = MAX_V_SPEED;
    }
    else if (ChassisRCtoVel->Chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        VxSetChannel = -MAX_V_SPEED;
    }

    if (ChassisRCtoVel->Chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        VySetChannel = MAX_V_SPEED;
    }
    else if (ChassisRCtoVel->Chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        VySetChannel = -MAX_V_SPEED;
    }
    //没有值直接给零
    if (VxSetChannel < CHASSIS_RC_DEADLINE * CHASSIS_V_SEN && VxSetChannel > -CHASSIS_RC_DEADLINE * CHASSIS_V_SEN)
    {
        VxSetChannel = 0.0f;
    }
    if (VySetChannel < CHASSIS_RC_DEADLINE * CHASSIS_V_SEN && VySetChannel > -CHASSIS_RC_DEADLINE * CHASSIS_V_SEN)
    {
         VySetChannel = 0.0f;
    }
    VxSetChannel *= CHASSIS_V_SEN;
    VySetChannel *= CHASSIS_V_SEN;
    //因为是平衡步兵，所以需要判断哪个方向的值大，往哪个方向前进，感觉操作手控制遥控器不脑残，不会鬼畜
    if (fabs(VxSetChannel) > fabs(VySetChannel))
    {
        VySetChannel = 0;
    }
    else
    {
        VxSetChannel = 0;
    }
    ChassisRCtoVel->Vx = VxSetChannel;
    ChassisRCtoVel->Vy = VySetChannel;
}
/**
 * @brief 底盘静止模式
 * 
 * @param ChassisMode 
 */
void ChassisNoMove(ChassisTask_s *ChassisMode)
{
    RightMotor->MF9025StopFlag = MOTOR_ENALBED;
    LeftMotor->MF9025StopFlag = MOTOR_ENALBED;
    ChassisMode->Vx = 0.0f;
    ChassisMode->Vy = 0.0f;
    ChassisMode->Wz = 0.0f;
}
/**
 * @todo 需要写跟随的代码
 * @brief 底盘跟随云台YAW模式
 * 
 * @param ChassisMode 
 */
void ChassisFollowGimbalYaw(ChassisTask_s *ChassisMode)
{
    RightMotor->MF9025StopFlag = MOTOR_ENALBED;
    LeftMotor->MF9025StopFlag = MOTOR_ENALBED;
    ChassisDataSet(ChassisMode);
}
/**
 * @todo 需要写YAW轴转动角度
 * @brief 底盘不跟随云台YAW轴模式
 * 
 * @param ChassisMode 底盘指针
 */
void ChassisNoFollowGimbalYaw(ChassisTask_s *ChassisMode)
{
    RightMotor->MF9025StopFlag = MOTOR_ENALBED;
    LeftMotor->MF9025StopFlag = MOTOR_ENALBED;
    ChassisDataSet(ChassisMode);
}
/**
 * @brief 无力模式，在车辆失控时使用
 * 
 */
void ChassisZeroForce()
{
    RightMotor->MF9025StopFlag = MOTOR_STOP;
    LeftMotor->MF9025StopFlag = MOTOR_STOP;
    return;
}
/**
 * @brief Lqr参数更新
 * 
 * @param Lqr Lqr结构体
 */
// void LqrDataUpdata(ChassisTask_s *Lqr)
// {
//     fp32 LVel,RVel,Vel;
//     fp32 LastAccel;
//     Lqr->LqrData.Theta = ChassisMove.Ins_Angle[0] - 0.01f;
//     Lqr->LqrData.Theta_w = Lqr->GyroData[0];
//     Lqr->LqrData.Pitch = ChassisMove.Ins_Angle[1] + BALANCEANGLE;
//     Lqr->LqrData.Pitch_w = Lqr->GyroData[1];
//     //目标值计算
//     Lqr->LqrData.TargetVel = ChassisMove.Vx;
//     Lqr->LqrData.TargetDis += (Lqr->LqrData.TargetVel)*0.002f;
//     #ifdef MotorDataRe
//     RVel = RightMotor->MF9025MotorMesure.RealSpeed;
//     LVel = LeftMotor->MF9025MotorMesure.RealSpeed;
//     //判断正反转，速度置反，是否需要放在接收函数里？
//     if (RightMotor->MotorSetting.MotorReverseFlag == MOTOR_DIRECTION_REVERSE)
//     {
//         RVel *= -1;
//     }
//     else if (LeftMotor->MotorSetting.MotorReverseFlag == MOTOR_DIRECTION_REVERSE)
//     {
//         LVel *= -1;
//     }
//     Vel = -(RVel+LVel)/2.0f;
//     #endif
//     #ifdef GyroDataRe
//     Vel += (Lqr->ChassisLastData.LastAccel + Lqr->AccelData[0])*0.02f/2.0f;//梯形积分
//     #endif
//     //设置模式
//     if (Lqr->LqrData.TargetVel)
//     {
//         Lqr->RobotMotionState.RobotState = AthleticState;
//     }else
//     {
//         Lqr->RobotMotionState.RobotState = RestingState;
//     }
//     //判断模式是否相同
//     if (Lqr->RobotMotionState.LastState != Lqr->RobotMotionState.RobotState)
//     {
//         Lqr->LqrData.TargetDis = Lqr->LqrData.Dis = 0;
//     }
//     //依据模式给不同的控制量
//     if (Lqr->RobotMotionState.RobotState == AthleticState)
//     {
//         Lqr->LqrData.Vel = Vel;
//         // Lqr->LqrData.Dis += (Lqr->ChassisLastData.LastVel + Lqr->LqrData.Vel)*0.002f/2.0f;//积分计算
//         Lqr->LqrData.Dis = (LeftMotor->MF9025MotorMesure.TotalAngle - RightMotor->MF9025MotorMesure.TotalAngle)* PI *WhellRadius;
//         if(Lqr->LqrData.TargetDis - Lqr->LqrData.Dis > 0.1f)
//             {Lqr->LqrData.TargetDis = Lqr->LqrData.Dis + 0.1f;} 
//         else if(Lqr->LqrData.TargetDis - Lqr->LqrData.Dis < -0.1f)
//             {Lqr->LqrData.TargetDis =Lqr->LqrData.Dis - 0.1f;}
//             //限制速度目标在当前速度的±0.3m/s内
//         if(Lqr->LqrData.TargetVel - Lqr->LqrData.Vel > 0.3f)
//             {Lqr->LqrData.TargetVel = Lqr->LqrData.Vel + 0.3f;}
//         else if(Lqr->LqrData.TargetVel - Lqr->LqrData.Vel < -0.3f)
//             {Lqr->LqrData.TargetVel = Lqr->LqrData.Vel - 0.3f;}
//     }else if (Lqr->RobotMotionState.RobotState == RestingState)
//     {
//         Lqr->LqrData.Vel = Vel;
//         Lqr->LqrData.TargetVel = 0;
//         Lqr->LqrData.Dis += (Lqr->ChassisLastData.LastVel + Lqr->LqrData.Vel)*0.002f/2.0f;
//         Lqr->LqrData.TargetDis = 0;
//     }
//     Lqr->RobotMotionState.LastState = Lqr->RobotMotionState.RobotState;
//     //测试版，需要解决陀螺仪YAW轴漂移问题,或者直接使用6020云台电机的编码器值
//     if (ChassisMove.Vy>0)
//     {
//         Lqr->LqrData.TargetTheta = Lqr->LqrData.Theta - 0.1f;
//         Lqr->LqrData.TargetTheta_w = Lqr->LqrData.Theta_w - 0.01f;
//     }else if (ChassisMove.Vy<0)
//     {
//         Lqr->LqrData.TargetTheta = Lqr->LqrData.Theta + 0.1f;
//         Lqr->LqrData.TargetTheta_w = Lqr->LqrData.Theta_w + 0.01f;
//     }else
//     {
//         Lqr->LqrData.TargetTheta = Lqr->LqrData.Theta;
//         Lqr->LqrData.TargetTheta_w = Lqr->LqrData.Theta_w;
//     }
//     Lqr->ChassisLastData.LastVel = Lqr->LqrData.Vel;
//     Lqr->ChassisLastData.LastAccel = Lqr->AccelData[0];
// }

void LqrDataUpdata(ChassisTask_s *Lqr)
{
    fp32 LVel,RVel,Vel;
    Lqr->LqrData.Theta = ChassisMove.Ins_Angle[0] - 0.01f;
    Lqr->LqrData.Theta_w = Lqr->GyroData[0];
    Lqr->LqrData.Pitch = ChassisMove.Ins_Angle[1] + BALANCEANGLE;
    Lqr->LqrData.Pitch_w = Lqr->GyroData[1];
    //目标值计算
    Lqr->LqrData.TargetVel = ChassisMove.Vx;
    Lqr->LqrData.TargetDis += (Lqr->LqrData.TargetVel)*0.002f;
    #ifdef MotorDataRe
    RVel = RightMotor->MF9025MotorMesure.RealSpeed;
    LVel = LeftMotor->MF9025MotorMesure.RealSpeed;
    //判断正反转，速度置反，是否需要放在接收函数里？
    if (RightMotor->MotorSetting.MotorReverseFlag == MOTOR_DIRECTION_REVERSE)
    {
        RVel *= -1;
    }
    else if (LeftMotor->MotorSetting.MotorReverseFlag == MOTOR_DIRECTION_REVERSE)
    {
        LVel *= -1;
    }
    Vel = -(RVel+LVel)/2.0f;
    #endif
    #ifdef GyroDataRe
    Vel += (Lqr->ChassisLastData.LastAccel + Lqr->AccelData[0])*0.02f/2.0f;//梯形积分
    #endif
    //设置模式
        Lqr->LqrData.Vel = Vel;
        // Lqr->LqrData.Dis += (Lqr->ChassisLastData.LastVel + Lqr->LqrData.Vel)*0.002f/2.0f;//积分计算
        Lqr->LqrData.Dis = -(LeftMotor->MF9025MotorMesure.TotalAngle - RightMotor->MF9025MotorMesure.TotalAngle)/360.0f* PI *WhellRadius;
        if(Lqr->LqrData.TargetDis - Lqr->LqrData.Dis > 0.15f)
            {Lqr->LqrData.TargetDis = Lqr->LqrData.Dis + 0.15f;} 
        else if(Lqr->LqrData.TargetDis - Lqr->LqrData.Dis < -0.15f)
            {Lqr->LqrData.TargetDis =Lqr->LqrData.Dis - 0.15f;}
            //限制速度目标在当前速度的±0.3m/s内
        if(Lqr->LqrData.TargetVel - Lqr->LqrData.Vel > 0.35f)
            {Lqr->LqrData.TargetVel = Lqr->LqrData.Vel + 0.35f;}
        else if(Lqr->LqrData.TargetVel - Lqr->LqrData.Vel < -0.35f)
            {Lqr->LqrData.TargetVel = Lqr->LqrData.Vel - 0.35f;}
    //测试版，需要解决陀螺仪YAW轴漂移问题,或者直接使用6020云台电机的编码器值
    if (ChassisMove.Vy>0)
    {
        Lqr->LqrData.TargetTheta = Lqr->LqrData.Theta - 0.1f;
        Lqr->LqrData.TargetTheta_w = Lqr->LqrData.Theta_w - 0.01f;
    }else if (ChassisMove.Vy<0)
    {
        Lqr->LqrData.TargetTheta = Lqr->LqrData.Theta + 0.1f;
        Lqr->LqrData.TargetTheta_w = Lqr->LqrData.Theta_w + 0.01f;
    }else
    {
        Lqr->LqrData.TargetTheta = Lqr->LqrData.Theta;
        Lqr->LqrData.TargetTheta_w = Lqr->LqrData.Theta_w;
    }
}
/**
 * @brief Get the Chassis Data Point object
 * @brief 获取底盘数据指针
 * 
 * @return ChassisTask_s* 
 */
ChassisTask_s *GetChassisDataPoint()
{
    return &ChassisMove;
}

