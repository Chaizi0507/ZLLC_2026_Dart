/**
 * @file crt_gimbal.cpp
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_FSM_Yaw_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{   
    #ifdef Second_Cali
    Status[Now_Status_Serial].Time++;
    //#ifdef Second_Cali
    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0)://向左堵转
        {
            Gimbal->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Yaw.Set_Target_Omega_Radian(2.0f);
            if(abs(Gimbal->Motor_Yaw.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(1);
            }
        }
        break;
        case (1)://左侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Left = Gimbal->Motor_Yaw.Get_Now_Angle();
                Set_Status(2);
            }
            else{
                Set_Status(0);
            }
        }
        break;
        case (2)://向右堵转
        {
            Gimbal->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Yaw.Set_Target_Omega_Radian(-2.0f);
            if(abs(Gimbal->Motor_Yaw.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(3);
            }
        }
        break;
        case (3)://右侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Right = Gimbal->Motor_Yaw.Get_Now_Angle();
                Set_Status(4);
            }
            else{
                Set_Status(2);
            }
        }
        break;
        case (4)://正常控制流程
        {
            Gimbal->TIM_Calculate_PeriodElapsedCallback();
        }
        break;
    }
    #endif
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case (0): // 校准开始状态
    {

        if (Gimbal->Motor_Yaw.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && Calibration_Status[0] == Calibration_Status_None)
        {
            Calibration_Status[0] = (Enum_Calibration_Status)Gimbal->Motor_Calibration(&Gimbal->Motor_Yaw,
                                                    &Gimbal->Motor_Yaw.Calibration_Variables.Calibrate_Offset,
                                                    Gimbal->Motor_Yaw.Calibration_Variables.Calibrate_Speed,
                                                    Gimbal->Motor_Yaw.Calibration_Variables.Calibrate_Stiffness,
                                                    Gimbal->Motor_Yaw.Calibration_Variables.calibration_count);
        }
       if (Gimbal->Motor_Pitch_L.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && Calibration_Status[1] == Calibration_Status_None)
        {
            Calibration_Status[1] = (Enum_Calibration_Status)Gimbal->Motor_Calibration(&Gimbal->Motor_Pitch_L,
                                                    &Gimbal->Motor_Pitch_L.Calibration_Variables.Calibrate_Offset,
                                                    Gimbal->Motor_Pitch_L.Calibration_Variables.Calibrate_Speed,
                                                    Gimbal->Motor_Pitch_L.Calibration_Variables.Calibrate_Stiffness,
                                                    Gimbal->Motor_Pitch_L.Calibration_Variables.calibration_count);
        }
        if (Gimbal->Motor_Pitch_R.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && Calibration_Status[2] == Calibration_Status_None)
        {
            Calibration_Status[2] = (Enum_Calibration_Status)Gimbal->Motor_Calibration(&Gimbal->Motor_Pitch_R,
                                                    &Gimbal->Motor_Pitch_R.Calibration_Variables.Calibrate_Offset,
                                                    Gimbal->Motor_Pitch_R.Calibration_Variables.Calibrate_Speed,
                                                    Gimbal->Motor_Pitch_R.Calibration_Variables.Calibrate_Stiffness,
                                                    Gimbal->Motor_Pitch_R.Calibration_Variables.calibration_count);
        }
        if (Calibration_Status[0] && Calibration_Status[1] && Calibration_Status[2])
            Set_Status(1);
    }
    break;
    case (1)://正常状态
    {   
        Gimbal->Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

        if (Gimbal->Motor_Yaw.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE ||
            Gimbal->Motor_Pitch_L.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE ||
            Gimbal->Motor_Pitch_R.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
        {
            if (Gimbal->Motor_Yaw.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
                Calibration_Status[0] = Calibration_Status_None;
            if (Gimbal->Motor_Pitch_L.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
                Calibration_Status[1] = Calibration_Status_None;
            if(Gimbal->Motor_Pitch_R.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
                Calibration_Status[2] = Calibration_Status_None;

            Set_Status(0); // 电机失能后重新校准
        }
    }
    break;
    }

}
/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    FSM_Yaw_Calibration.Init(5,0);

    Motor_Pitch_L.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pitch_L.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Pitch_L.Get_Output_Max(), Motor_Pitch_L.Get_Output_Max());
    Motor_Pitch_L.Init(&hfdcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);
    Motor_Pitch_L.Calibration_Init(0.0f,0.0f);//待调参

    Motor_Pitch_R.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pitch_R.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Pitch_R.Get_Output_Max(), Motor_Pitch_R.Get_Output_Max());
    Motor_Pitch_R.Init(&hfdcan2, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);
    Motor_Pitch_R.Calibration_Init(0.0f,0.0f);//待调参

    Motor_Yaw.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Yaw.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA);
    Motor_Yaw.Calibration_Init(0.0f,0.0f);//待调参 
}


bool Class_Gimbal::Motor_Calibration(Class_DJI_Motor_C610_Dart_Type *motor,float *Cali_Offset,float Cali_Omega,float Cali_Max_Out,uint16_t &count)
{
    //记录电机堵转时间
	//设置为速度环校准
	motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	motor->Set_Target_Omega_Radian(Cali_Omega);
	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(motor->Get_Now_Torque()) >= Cali_Max_Out) && (fabs(motor->Get_Now_Omega_Radian()) < 0.01f*PI) )
	{
		count++;
		//当到达一定时间，判定为堵转
		if(count >= 50)
		{
			count = 0;
			//改为开环，设置力矩为0
			motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
			//分别记录2006的零位角度
			*Cali_Offset = motor->Get_Now_Radian();
            //设置电机的电流为0
            motor->Set_Target_Torque(0.0f);
			return true;
		}	
	}
	else
	{
		count=0;
	}
	return false;
}

void Class_DJI_Motor_C610_Dart_Type::Calibration_Init(float Cali_Speed,float Cali_Stiffness)
{
    //初始化
    Calibration_Variables.Calibrate_Speed = Cali_Speed;
    Calibration_Variables.Calibrate_Stiffness = Cali_Stiffness;
}

void Class_DJI_Motor_C610_Dart_Type::TIM_Calculate_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        //默认闭环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Transform_Target_Angle);
        PID_Angle.Set_Now(Transform_Now_Angle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Radian = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}
/**
 * @brief 输出到电机
 *
 */

void Class_Gimbal::Output()
{
    switch (Gimbal_Control_Type)
    {
    case Gimbal_Control_Type_DISABLE:
    {
        

        break;
    }
    case Gimbal_Control_Type_NORMAL:
    {
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Yaw.Transform_Target_Angle = Motor_Yaw.Calibration_Variables.Calibrate_Offset + Target_Yaw_Angle;
        Motor_Pitch_L.Transform_Target_Angle = Motor_Pitch_L.Calibration_Variables.Calibrate_Offset + Target_Pitch_Angle;
        Motor_Pitch_R.Transform_Target_Angle = Motor_Pitch_R.Calibration_Variables.Calibrate_Offset + Target_Pitch_Angle;
        Motor_Yaw.Transform_Now_Angle = Motor_Yaw.Get_Now_Radian();
        Motor_Pitch_L.Transform_Now_Angle = Motor_Pitch_L.Get_Now_Radian();
        Motor_Pitch_R.Transform_Now_Angle = Motor_Pitch_R.Get_Now_Radian();
        // Motor_Yaw.Set_Target_Angle(Motor_Yaw.Calibration_Variables.Calibrate_Offset + Target_Yaw_Angle);//±号待修订
        // Motor_Pitch_L.Set_Target_Angle(Motor_Pitch_L.Calibration_Variables.Calibrate_Offset + Target_Pitch_Angle);//±号待修订
        // Motor_Pitch_R.Set_Target_Angle(Motor_Pitch_R.Calibration_Variables.Calibrate_Offset + Target_Pitch_Angle);//±号待修订

        break;
    }
    default:
        break;
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    //控制模式
    Output();

    //PID输出
    Motor_Yaw.TIM_Calculate_PeriodElapsedCallback();
    Motor_Pitch_L.TIM_Calculate_PeriodElapsedCallback();
    Motor_Pitch_R.TIM_Calculate_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
