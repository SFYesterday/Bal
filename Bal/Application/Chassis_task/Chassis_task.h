#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MF9025.h"
#include "MotorTypedef.h"
#include "remote_control.h"
#include "math.h"
#include "INS_task.h"
#include "Control.h"
#include "User.h"

//键盘映射
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
//最大速度定义，暂时还没测试，后续测试
#define MAX_V_SPEED   2
//遥控器模式定义，就是上面的右侧拨杆
#define CHASSIS_MODE_CHANNEL 0
//遥控器通道定义，为什么这样可以自己Debug看看
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 0
//遥控器转换为车辆速度(m/s)的比例，遥感最大值为最大660
#define CHASSIS_V_SEN 0.030303030303030303030f//0.0045454545454545f
// #define CHASSIS_V_SEN 1.422727272727273f //
//遥控器死区
#define CHASSIS_RC_DEADLINE 10
//平衡角度
#define BALANCEANGLE 0.034f
//MF9025电机扭矩电流比例常数
#define MF9025TORQUEPROPORTION  195.3125f//387.87878787878787878787878787879f//274.348f
//电机数据或者陀螺仪作为LQR的数据更新
#define MotorDataRe
// #define GyroDataRe
// 选择PID控制还是LQR控制
// #define PidControl
#define LqrControl

typedef enum
{
  CHASSIS_ZERO_FORCE,                   
  CHASSIS_NO_MOVE,
  CHASSIS_FOLLOW_GIMBAL_YAW,
  CHASSIS_NO_FOLLOW_YAW,           
} ChassisBehaviour_e;

typedef enum
{
  AthleticState,
  RestingState,
}RobotMotionStatus_e;

typedef struct
{
  RobotMotionStatus_e RobotState;
  uint8_t LastState;
}RobotMotionStatus_s;
typedef struct 
{
  fp32 LastVel;
  fp32 LastAccel;
}ChassisLastData_s;


typedef struct 
{
   const RC_ctrl_t *Chassis_RC;
   const fp32 *Ins_Angle;
   const fp32 *GyroData;
   const fp32 *AccelData;
   
   fp32 Vx;
   fp32 Vy;                         //我这里是设计了下，如果有y方向会直接转弯
   fp32 Wz;

   fp32 BalTorque;
   fp32 SteerTorque;
   fp32 SpeedTorque;      //PID控制器使用

   fp32 LeftTorque;
   fp32 RightTorque;

   fp32 LeftTorqueCurrent;
   fp32 RightTorqueCurrent;

   RobotMotionStatus_s RobotMotionState;
   first_order_filter_type_t *CurrentFOF;
   
   ChassisLastData_s ChassisLastData;

   Lqr_s LqrData;
   ChassisBehaviour_e ChassisMode;
}ChassisTask_s;


void ChassisTask(void const *pvParameters);
void ChassisInit(void);
void ChassisWheelTorqueCurrentLimit(ChassisTask_s *ChassisTorque,fp32 count);
void ChassisWheelTorqueSet(ChassisTask_s *ChassisTorque);
void ChassisModeSet(ChassisTask_s *ChassisModeSet);
void ChassisBehaviourControl(ChassisTask_s *ChassisModeSet);
void ChassisDataSet(ChassisTask_s *ChassisRCtoVel);
void ChassisNoMove(ChassisTask_s *ChassisMode);
void ChassisFollowGimbalYaw(ChassisTask_s *ChassisMode);
void ChassisNoFollowGimbalYaw(ChassisTask_s *ChassisMode);
void ChassisZeroForce(void);
void LqrDataUpdata(ChassisTask_s *Lqr);
ChassisTask_s *GetChassisDataPoint(void);




#endif

