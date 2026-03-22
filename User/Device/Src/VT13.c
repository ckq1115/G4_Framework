//
// Created by CaoKangqi on 2026/3/21.
//
#include "VT13.h"
#include "CKQ_MATH.h"

float VT_OneFilter(float last , float now , float thresholdValue);
uint8_t VT_F_KEY_STATUS(uint64_t  KEY , uint8_t PRESS_TIME);
float VT_deadband(float Value, float minValue, float maxValue);
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);


void VT13_Resovled(uint8_t* Data, VT13_Typedef* VT13)
{
	if(Data[0] == 0XA9 && Data[1] == 0X53 && (verify_crc16_check_sum(Data,21) == 1))
	{
		VT13->CRC_flag = 1;//校验通过
		VT13_UNION_Typdef VT13_UNION = {0};

		static uint8_t Key_Q_Lock = 0; // 0是开锁，1是上锁
		static uint8_t Key_E_Lock = 0;
		static uint8_t Key_R_Lock = 0;
		static uint8_t Key_F_Lock = 0;
		static uint8_t Key_G_Lock = 0;
		static uint8_t Key_Z_Lock = 0;
		static uint8_t Key_X_Lock = 0;
		static uint8_t Key_C_Lock = 0;
		static uint8_t Key_V_Lock = 0;
		static uint8_t Key_B_Lock = 0;
		static uint8_t Key_Shift_Lock = 0;
		static uint8_t Key_Ctrl_Lock = 0;
		static uint8_t Mouse_R_Lock = 0;
		static uint8_t Mouse_L_Lock = 0;

		memcpy(VT13_UNION.GetData , Data , 21);

		//遥控数据
		VT13->Remote.Channel[0] = VT13_UNION.DataNeaten.ch_0 - 1024;
		VT13->Remote.Channel[1] = VT13_UNION.DataNeaten.ch_1 - 1024;
		VT13->Remote.Channel[2] = VT13_UNION.DataNeaten.ch_2 - 1024;
		VT13->Remote.Channel[3] = VT13_UNION.DataNeaten.ch_3 - 1024;

		VT13->Remote.wheel = VT13_UNION.DataNeaten.wheel - 1024;

		VT13->Remote.mode_sw = VT13_UNION.DataNeaten.mode_sw;
		VT13->Remote.pause = VT13_UNION.DataNeaten.pause;
		VT13->Remote.fn_1 = VT13_UNION.DataNeaten.fn_1;
		VT13->Remote.fn_2 = VT13_UNION.DataNeaten.fn_2;
		VT13->Remote.trigger = VT13_UNION.DataNeaten.trigger;

		//*对点按和长按的区分*//
		VT13->Mouse.L_State = VT_F_KEY_STATUS(VT13_UNION.DataNeaten.mouse_left , VT13->Mouse.L_PressTime);
		VT13->Mouse.R_State = VT_F_KEY_STATUS(VT13_UNION.DataNeaten.mouse_right  , VT13->Mouse.R_PressTime);
		VT13->Mouse.M_State = VT_F_KEY_STATUS(VT13_UNION.DataNeaten.mouse_middle  , VT13->Mouse.M_PressTime);

		//*键盘的按键*//
		VT13->KeyBoard.W = VT_F_KEY_STATUS(VT13_UNION.DataNeaten.KeyBoard_W , VT13->KeyBoard.W_PressTime);
		VT13->KeyBoard.A = VT_F_KEY_STATUS(VT13_UNION.DataNeaten.KeyBoard_A , VT13->KeyBoard.A_PressTime);
		VT13->KeyBoard.S = VT_F_KEY_STATUS(VT13_UNION.DataNeaten.KeyBoard_S , VT13->KeyBoard.S_PressTime);
		VT13->KeyBoard.D = VT_F_KEY_STATUS(VT13_UNION.DataNeaten.KeyBoard_D , VT13->KeyBoard.D_PressTime);

		//*鼠标滤波*//
		VT13->Mouse.X_Flt = VT_OneFilter(VT13->Mouse.X_Flt , (float) VT13_UNION.DataNeaten.mouse_x , 500);
		VT13->Mouse.Y_Flt = VT_OneFilter(VT13->Mouse.Y_Flt , (float) VT13_UNION.DataNeaten.mouse_y , 500);
		VT13->Mouse.Z_Flt = VT_OneFilter(VT13->Mouse.Z_Flt , (float) VT13_UNION.DataNeaten.mouse_z , 500);

		//*鼠标死区*//
		VT13->Mouse.X_Flt = VT_deadband(VT13->Mouse.X_Flt,-3.0e-3,3.0e-3);
		VT13->Mouse.Y_Flt = VT_deadband(VT13->Mouse.Y_Flt,-3.0e-3,3.0e-3);
		VT13->Mouse.Z_Flt = VT_deadband(VT13->Mouse.Z_Flt,-3.0e-3,3.0e-3);

		// Shift
		VT13->KeyBoard.Shift = VT13_UNION.DataNeaten.KeyBoard_Shift;
		if (VT13->KeyBoard.Shift == 1 && Key_Shift_Lock == 0)
		{
			VT13->KeyBoard.Shift_PreeNumber = !VT13->KeyBoard.Shift_PreeNumber;
			Key_Shift_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.Shift == 0 && Key_Shift_Lock == 1)
		{
			Key_Shift_Lock = 0; // 开锁
		}
		// Ctrl
		VT13->KeyBoard.Ctrl = VT13_UNION.DataNeaten.KeyBoard_Ctrl;
		if (VT13->KeyBoard.Ctrl == 1 && Key_Ctrl_Lock == 0)
		{
			VT13->KeyBoard.Ctrl_PreeNumber = !VT13->KeyBoard.Ctrl_PreeNumber;
			Key_Ctrl_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.Ctrl == 0 && Key_Ctrl_Lock == 1)
		{
			Key_Ctrl_Lock = 0; // 开锁
		}
		// Q
		VT13->KeyBoard.Q = VT13_UNION.DataNeaten.KeyBoard_Q;
		if (VT13->KeyBoard.Q == 1 && Key_Q_Lock == 0)
		{
			VT13->KeyBoard.Q_PreeNumber = !VT13->KeyBoard.Q_PreeNumber;
			Key_Q_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.Q == 0 && Key_Q_Lock == 1)
		{
			Key_Q_Lock = 0; // 开锁
		}
		// E
		VT13->KeyBoard.E = VT13_UNION.DataNeaten.KeyBoard_E;
		if (VT13->KeyBoard.E == 1 && Key_E_Lock == 0)
		{
			VT13->KeyBoard.E_PreeNumber = !VT13->KeyBoard.E_PreeNumber;
			Key_E_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.E == 0 && Key_E_Lock == 1)
		{
			Key_E_Lock = 0; // 开锁
		}
		// R
		VT13->KeyBoard.R = VT13_UNION.DataNeaten.KeyBoard_R;
		if (VT13->KeyBoard.R == 1 && Key_R_Lock == 0)
		{
			VT13->KeyBoard.R_PreeNumber = !VT13->KeyBoard.R_PreeNumber;
			Key_R_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.R == 0 && Key_R_Lock == 1)
		{
			Key_R_Lock = 0; // 开锁
		}
		// F
		VT13->KeyBoard.F = VT13_UNION.DataNeaten.KeyBoard_F;
		if (VT13->KeyBoard.F == 1 && Key_F_Lock == 0)
		{
			VT13->KeyBoard.F_PreeNumber = !VT13->KeyBoard.F_PreeNumber;
			Key_F_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.F == 0 && Key_F_Lock == 1)
		{
			Key_F_Lock = 0; // 开锁
		}
		// Z
		VT13->KeyBoard.Z = VT13_UNION.DataNeaten.KeyBoard_Z;
		if (VT13->KeyBoard.Z == 1 && Key_Z_Lock == 0)
		{
			VT13->KeyBoard.Z_PreeNumber = !VT13->KeyBoard.Z_PreeNumber;
			Key_Z_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.Z == 0 && Key_Z_Lock == 1)
		{
			Key_Z_Lock = 0; // 开锁
		}
		// G
		VT13->KeyBoard.G = VT13_UNION.DataNeaten.KeyBoard_G;
		if (VT13->KeyBoard.G == 1 && Key_G_Lock == 0)
		{
			VT13->KeyBoard.G_PreeNumber = !VT13->KeyBoard.G_PreeNumber;
			Key_G_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.G == 0 && Key_G_Lock == 1)
		{
			Key_G_Lock = 0; // 开锁
		}
		// X
		VT13->KeyBoard.X = VT13_UNION.DataNeaten.KeyBoard_X;
		if (VT13->KeyBoard.X == 1 && Key_X_Lock == 0)
		{
			VT13->KeyBoard.X_PreeNumber = !VT13->KeyBoard.X_PreeNumber;
			Key_X_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.X == 0 && Key_X_Lock == 1)
		{
			Key_X_Lock = 0; // 开锁
		}
		// C
		VT13->KeyBoard.C = VT13_UNION.DataNeaten.KeyBoard_C;
		if (VT13->KeyBoard.C == 1 && Key_C_Lock == 0)
		{
			VT13->KeyBoard.C_PreeNumber = !VT13->KeyBoard.C_PreeNumber;
			Key_C_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.C == 0 && Key_C_Lock == 1)
		{
			Key_C_Lock = 0; // 开锁
		}

		VT13->KeyBoard.V = VT13_UNION.DataNeaten.KeyBoard_V;
		if (VT13->KeyBoard.V == 1 && Key_V_Lock == 0)
		{
			VT13->KeyBoard.V_PreeNumber = !VT13->KeyBoard.V_PreeNumber;
			Key_V_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.V == 0 && Key_V_Lock == 1)
		{
			Key_V_Lock = 0; // 开锁
		}

		VT13->KeyBoard.B = VT13_UNION.DataNeaten.KeyBoard_B;
		if (VT13->KeyBoard.B == 1 && Key_B_Lock == 0)
		{
			VT13->KeyBoard.B_PreeNumber = !VT13->KeyBoard.B_PreeNumber;
			Key_B_Lock = 1; // 上锁
		}
		else if (VT13->KeyBoard.B == 0 && Key_B_Lock == 1)
		{
			Key_B_Lock = 0; // 开锁
		}

		//RUI_V_DBUS->Mouse.L_State = RUI_V_DBUS_UNION.DataNeaten.Mouse_L;
		if (VT13_UNION.DataNeaten.mouse_left == 1)
		{
			VT13->Mouse.L_PressTime++;

				if(VT13->Mouse.L_PressTime > 250)//按压时间接近边界时，重置按压时间，防止归零时，鼠标状态变成单击-------待测
				{VT13->Mouse.L_PressTime = 20;}
		}
		else if (VT13_UNION.DataNeaten.mouse_left == 0)
		{
			VT13->Mouse.L_PressTime = 0;
		}

		//RUI_V_DBUS->Mouse.R_State = RUI_V_DBUS_UNION.DataNeaten.Mouse_R;
		if (VT13_UNION.DataNeaten.mouse_right == 1)
		{
			VT13->Mouse.R_PressTime++;

				if(VT13->Mouse.R_PressTime > 250)
				{
					VT13->Mouse.R_PressTime = 0;
				}
		}
		else if (VT13_UNION.DataNeaten.mouse_right == 0)
		{
					VT13->Mouse.R_PressTime = 0;
		}

		if (VT13_UNION.DataNeaten.mouse_middle == 1)
		{
			VT13->Mouse.M_PressTime++;

				if(VT13->Mouse.M_PressTime > 250)
				{
					VT13->Mouse.M_PressTime = 0;
				}
		}
		else if (VT13_UNION.DataNeaten.mouse_middle == 0)
		{
					VT13->Mouse.M_PressTime = 0;
		}
	}
	else
	{
		VT13->CRC_flag = 0;//校验不通过
	}
}


/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞 COPY 赵澍
 *	@performance:	    //鼠标滤波
 *	@parameter:		    //上一次的值//当前值//尖峰敏感度
 *	@time:				//22-11-23 16:39
 *	@ReadMe:			//使用一阶低通滤波(改进版)
                        //尖峰敏感度：越小对尖峰越敏感	(一般取值为最大值的20%)
 ************************************************************万能分隔符**************************************************************/
float VT_OneFilter(float last , float now , float thresholdValue)
{
	// 减小平滑滤波值会增大对于细小毛刺的过滤程度
	// 增加尖峰滤波值会增大对于尖峰数值的响应程度
	const float sensitivlFilter = 0.8f; // 尖峰滤波值//小于1
	const float numbFilter = 0.2f; // 平滑滤波值//小于1

	if (MATH_ABS_float(MATH_ABS_float(last) - MATH_ABS_float(now)) >= thresholdValue)
	{
		return (float) (now * sensitivlFilter + last * (1 - sensitivlFilter));
	}
	else
	{
		return (float) (now * numbFilter + last * (1 - numbFilter));
	}
}
/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //按键长按短按
 *	@parameter:		    //
 *	@time:				//23-04-26 21:40
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t VT_F_KEY_STATUS(uint64_t  KEY , uint8_t PRESS_TIME)
{
	static uint8_t press_time = 0;
	if (KEY == 1)
	{
		if (PRESS_TIME <= 20)
		{
			PRESS_TIME++;
			return RUI_DF_KEY_CLICK;
		}
		else
		{
			return  RUI_DF_KEY_PRESS; // 长按
		}
	}
	else
	{
		PRESS_TIME = 0;
		return  RUI_DF_KEY_UP;
	}


}

//浮点死区
float VT_deadband(float Value, float minValue, float maxValue)
{
	if (Value < maxValue && Value > minValue)
	{
		Value = 0.0f;
	}
	return Value;
}


