/**
 * @file TensionMeter.cpp
 * @author lez
 * @brief HX711  tension meter
 * @version 0.1
 * @date 2024-12-18 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "D:\Work_stm\ZLLC_2026_Dart\User\Device\Inc\dvc_TensionMeter.h"
#include <string.h> 


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
uint8_t test_meter_tension[4];
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
/**
 * @brief 初始化拉力计
 * 
 */
void Class_TensionMeter::Init()
{
    Now_Tension = 0.0f;
    Last_Update_Time = HAL_GetTick();
}

/**
 * @brief 串口接收回调函数
 * @param Rx_Data 接收到的数据缓冲区
 * @param Length 数据长度
 */
void Class_TensionMeter::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 更新时间戳
    Last_Update_Time = HAL_GetTick();

    // 既然发送端是 memcpy(data, &weight, 4); 发送的
    // 接收端直接 memcpy 回来即可
    if (Length >= 4)
    {
        memcpy(test_meter_tension, Rx_Data, 4);
        memcpy(&Now_Tension, test_meter_tension, 4);
    }
}


/* Function prototypes -------------------------------------------------------*/



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
