/**
 * @file TensionMeter.h
 * @author lez
 * @brief HX711  tension meter
 * @version 0.1
 * @date 2024-12-18 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef DVC_TENSIONMETER_H
#define DVC_TENSIONMETER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

class Class_TensionMeter
{
public:
    void Init();
    
    // 串口接收回调函数
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    // 获取拉力值接口
    inline float Get_Tension();
    
    // 判断传感器是否在线（用于安全保护）
    inline bool Is_Normal();

protected:
    float Now_Tension = 0.0f;       // 当前拉力值
    unsigned long Raw_Tension = 0;  //拉力计源数据
    uint32_t Last_Update_Time = 0;  // 上次更新时间，用于判断离线

};

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 获取当前拉力值
 * @return float 拉力值
 */
inline float Class_TensionMeter::Get_Tension()
{
    return Now_Tension ; 
}

/**
 * @brief 判断传感器状态是否正常（超时检测）
 * @return true 正常
 * @return false 离线
 */
inline bool Class_TensionMeter::Is_Normal()
{
    // 如果超过 200ms 没有收到数据，认为离线
    if (HAL_GetTick() - Last_Update_Time > 200)
    {
        return false;
    }
    return true;
}

/* Function prototypes -------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
