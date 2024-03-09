#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"
#include "string.h"
#include "stdlib.h"

#define MotorNum 7

#pragma pack(1)
typedef struct _
{
    CAN_HandleTypeDef *CanHandle; // can句柄
    CAN_TxHeaderTypeDef TxConf;    // CAN报文发送配置
    uint32_t TxID;                // 发送id
    uint32_t TxMailbox;           // CAN消息填入的邮箱号
    uint8_t TxBuff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t RxBuff[8];            // 接收缓存,最大消息长度为8
    uint32_t RxID;                // 接收id
    uint8_t RxLen;                // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*CanMoudleCallBack)(struct _ *); // callback needs an instance to tell among registered ones
    void *ID;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;
#pragma pack()

typedef struct
{
    CAN_HandleTypeDef *CanHandle;              // can句柄
    uint32_t TxID;                             // 发送id
    uint32_t RxID;                             // 接收id
    void (*CanMoudleCallBack)(CANInstance *); // 处理接收数据的回调函数
    void *ID;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CanInitConfig_s;


void CanFilterInit(void);
void CanTransmit(CANInstance *caninstance);
CANInstance *CANRegister(CanInitConfig_s *config);
void CanFilterInit(void);
void CanTransmit(CANInstance *caninstance);


#endif

