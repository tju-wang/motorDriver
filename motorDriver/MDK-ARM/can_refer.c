
#include "main.h"

#define F407VET6_BOARD_CAN_ID    	  0x001
#define SENSOR_BOARD_CAN_ID  	  	  0x002
#define ANOTHER_SENSOR_BOARD_CAN_ID   0x003
#define THIRD_SENSOR_BOARD_CAN_ID  	  0x004

	 

//2个3级深度的FIFO

#define   CAN1FIFO   CAN_RX_FIFO0	           

#define   CAN2FIFO   CAN_RX_FIFO1  // 

CAN_TxHeaderTypeDef     TxMeg;

CAN_RxHeaderTypeDef     RxMeg;

 

void CAN_User_Init(CAN_HandleTypeDef* hcan )   //用户初始化函数

{

   CAN_FilterTypeDef  sFilterConfig;

   HAL_StatusTypeDef  HAL_Status;

	

  TxMeg.IDE=CAN_ID_STD;//CAN_ID_EXT;

  TxMeg.RTR=CAN_RTR_DATA;

	

  sFilterConfig.FilterBank = 0;                       //过滤器0

  sFilterConfig.FilterMode =  CAN_FILTERMODE_IDLIST;  //设为列表模式    

  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;    

		

  sFilterConfig.FilterIdHigh = F407VET6_BOARD_CAN_ID<<5;   //基本ID放入到STID中  

  sFilterConfig.FilterIdLow  = SENSOR_BOARD_CAN_ID <<5;    

		

  sFilterConfig.FilterMaskIdHigh =ANOTHER_SENSOR_BOARD_CAN_ID<<5;

  sFilterConfig.FilterMaskIdLow  =THIRD_SENSOR_BOARD_CAN_ID <<5; 

  sFilterConfig.FilterFIFOAssignment = CAN1FIFO;    //接收到的报文放入到FIFO0中 

 

  sFilterConfig.FilterActivation = ENABLE;  	//激活过滤器

  sFilterConfig.SlaveStartFilterBank  = 0; 

 

  HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

  HAL_Status=HAL_CAN_Start(hcan);  //开启CAN

  if(HAL_Status!=HAL_OK){

	printf("开启CAN失败\r\n");	

 }	

 HAL_Status=HAL_CAN_ActivateNotification(hcan,   CAN_IT_RX_FIFO0_MSG_PENDING);

 if(HAL_Status!=HAL_OK){

	printf("开启挂起中段允许失败\r\n");	

  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //接收回调函数

{

  uint8_t  Data[8];

  HAL_StatusTypeDef	HAL_RetVal;

  if(hcan ==&hcan1){	

    HAL_RetVal=HAL_CAN_GetRxMessage(hcan,  CAN1FIFO, &RxMeg,  Data);

    if ( HAL_OK==HAL_RetVal){                              			

      //在这里接收数据

    }

  }

}

 

//发送数据函数

uint8_t CANx_SendNormalData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)

{

	HAL_StatusTypeDef	HAL_RetVal;

        uint16_t SendTimes,SendCNT=0;

	uint8_t  FreeTxNum=0;

	TxMeg.StdId=ID;

	if(!hcan || ! pData ||!Len)  return 1;

	SendTimes=Len/8+(Len%8?1:0);

	FreeTxNum=HAL_CAN_GetTxMailboxesFreeLevel(hcan);

	TxMeg.DLC=8;

	while(SendTimes--){

		if(0==SendTimes){

			if(Len%8)

				TxMeg.DLC=Len%8;

		}

		while(0==FreeTxNum){

			FreeTxNum=HAL_CAN_GetTxMailboxesFreeLevel(hcan);

		}

		HAL_Delay(1);   //没有延时很有可能会发送失败

		HAL_RetVal=HAL_CAN_AddTxMessage(hcan,&TxMeg,pData+SendCNT,(uint32_t*)CAN_TX_MAILBOX0); 

		if(HAL_RetVal!=HAL_OK)

		{

			return 2;

		}

		SendCNT+=8;

	}

	

  return 0;

}
