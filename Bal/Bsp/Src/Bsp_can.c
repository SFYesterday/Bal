#include "Bsp_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*can实例，储存在这里方便钩子函数遍历*/
static CANInstance *Can_Instance[MotorNum]  = {NULL};
static uint8_t IDx;

/**
 * @brief can的滤波初始化
uint8_t can_rx_buff[8];

/**
 * @brief can���˲���ʼ������
 * 
 * @return ** void 
 */

void CanFilterInit(void)
{

    CAN_FilterTypeDef can_filter_st1;
	CAN_FilterTypeDef can_filter_st2;
	
    can_filter_st1.FilterActivation = ENABLE;
    can_filter_st1.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st1.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st1.FilterIdHigh = 0x0000;
    can_filter_st1.FilterIdLow = 0x0000;
    can_filter_st1.FilterMaskIdHigh = 0x0000;
    can_filter_st1.FilterMaskIdLow = 0x0000;
    can_filter_st1.SlaveStartFilterBank = 0;
    can_filter_st1.FilterBank = 0;
    can_filter_st1.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    can_filter_st2.FilterActivation = ENABLE;
    can_filter_st2.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st2.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st2.FilterIdHigh = 0x0000;
    can_filter_st2.FilterIdLow = 0x0000;
    can_filter_st2.FilterMaskIdHigh = 0x0000;
    can_filter_st2.FilterMaskIdLow = 0x0000;	
    can_filter_st2.FilterFIFOAssignment = CAN_RX_FIFO1;
    can_filter_st2.SlaveStartFilterBank = 14;
    can_filter_st2.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);


}

CANInstance *CANRegister(CanInitConfig_s *config)
{ 
    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
    instance->TxConf.StdId = config->TxID; // 发送id
    instance->TxConf.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->TxConf.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->TxConf.DLC = 0x08;            // 默认发送长度为8
    // 设置回调函数和接收发送id
    instance->CanHandle = config->CanHandle;
    instance->TxID = config->TxID; // 好像没用,可以删掉
    instance->RxID = config->RxID;
    instance->CanMoudleCallBack = config->CanMoudleCallBack;
    instance->ID = config->ID;

    Can_Instance[IDx++] = instance; // 将实例保存到can_instance中


    return instance; // 返回can实例指针
}

/**
 * @brief can发送信息
 * 
 * @param caninstance can实例 
 */
void CanTransmit(CANInstance *caninstance)
{
    HAL_CAN_AddTxMessage(caninstance->CanHandle, &caninstance->TxConf, caninstance->TxBuff, &caninstance->TxMailbox);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}


static void CANFIFOxCallback(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef RxConf; 
    uint8_t CanRxBuff[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox)) // FIFO缓冲区有值
    {
        HAL_CAN_GetRxMessage(hcan, fifox, &RxConf, CanRxBuff); // 接收FIFO数据
        for (size_t i = 0; i < IDx; i++)
        { 
            if (hcan == Can_Instance[i]->CanHandle && RxConf.StdId == Can_Instance[i]->RxID)   // 两者相等说明这是要找的实例
            {
                memcpy(Can_Instance[i]->RxBuff, CanRxBuff, RxConf.DLC); // 消息拷贝到对应实例
                if (Can_Instance[i]->CanMoudleCallBack != NULL) // 回调函数不为空就调用
                {
                    Can_Instance[i]->RxLen = RxConf.DLC;                      // 保存接收到的数据长度
                    Can_Instance[i]->CanMoudleCallBack(Can_Instance[i]);     // 触发回调进行数据解析和处理
                }
                return;
            }
        }
    }
}


/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0); 
    CANFIFOxCallback(hcan, CAN_RX_FIFO0); // ���������Լ�д�ĺ�����������Ϣ
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1); 
    CANFIFOxCallback(hcan, CAN_RX_FIFO1); // ���������Լ�д�ĺ�����������Ϣ
}


