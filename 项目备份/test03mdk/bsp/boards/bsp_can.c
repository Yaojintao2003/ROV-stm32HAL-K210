#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;//can1
//extern CAN_HandleTypeDef hcan2;

//筛选器配置
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;//筛选器使能（开启）
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//筛选器掩码模式,还有列表模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//掩码用32位表示
    can_filter_st.FilterIdHigh = 0x0000;//具体Id要求高16位
    can_filter_st.FilterIdLow = 0x0000;  //具体Id要求低16位
    can_filter_st.FilterMaskIdHigh = 0x0000; //掩码高16位全设置为0，表示对所有位报文Id高16位都不关心
    can_filter_st.FilterMaskIdLow = 0x0000;//掩码低16位全设置为0，表示对所有位报文Id低16位都不关心
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 0;//14改为0
    /*这里说明一下为什么填0
    			首先一个筛选器有28个组。一块stm32板子的can外设共用一个can外设
    			比如F4系列基本都有两个can模块，如果只用一个can的时候，我们FilterBank可选0-13，因此下面就是0，其实填0-13都可以。
    			如果用两个can，则要分配两个筛选器组。一个是FilterBank
    			则
    			can1的筛选器组选择0-13。如CAN1_FilerConf.FilterBank=0;
    			can2的筛选器组选择14-27。如CAN1_FilerConf.SlaveStartFilterBank=14;
    		*/

	/*	此处&hcan1指明是can1的筛选器配置，但实际上can1和can2的筛选器都配置好了。因为两个can是共用的。
		这是因为STM32的双路CAN共用过滤器组，
		而且过滤器组寄存器与CAN1配置寄存器在物理上是挨着的，HAL库将这些寄存器合并在一个结构里访问而已。
		下面通过调用 "HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)" 配置can筛选器即可生效。
		无需再调用HAL_CAN_ConfigFilter(&hcan2,&CAN1_FilerConf)
	*/
    //HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
   // HAL_CAN_Start(&hcan2);
   // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}
