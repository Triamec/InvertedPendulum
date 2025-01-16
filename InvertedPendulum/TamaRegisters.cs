// Copyright © 2017 Triamec Motion AG

using Triamec.Tama.Rlid19;

static class TamaRegisters {
	#region Tama parameters floats

	public static float PF00_DStateCtrl => Register.Application.Parameters.Floats[0];
	public static float P01_fStateCtrl_Hz => Register.Application.Parameters.Floats[1];
	public static float P02_fIntCtrl => Register.Application.Parameters.Floats[2];
	public static float P03_fVel_Hz => Register.Application.Parameters.Floats[3];

	public static float PF10_Vel_m_s => Register.Application.Parameters.Floats[10];
	public static float PF11_Acc_m2_s => Register.Application.Parameters.Floats[11];
	public static float PF12_Jrk_m3_s => Register.Application.Parameters.Floats[12];
	public static float PF13_Radius0_m => Register.Application.Parameters.Floats[13];
	public static float PF14_Radius1_m => Register.Application.Parameters.Floats[14];

    public static float PF16_Vel_Circle_m_s => Register.Application.Parameters.Floats[16];
    public static float PF17_Acc_Circle_m2_s => Register.Application.Parameters.Floats[17];
    public static float PF18_Jrk_Circle_m3_s => Register.Application.Parameters.Floats[18];


    public static float PF21_CalibOffsetB_LL { get { return Register.Application.Parameters.Floats[21]; } set { Register.Application.Parameters.Floats[21] = value; } }
	public static float PF22_CalibOffsetA_LL { get { return Register.Application.Parameters.Floats[22]; } set { Register.Application.Parameters.Floats[22] = value; } }
	public static float PF23_CalibOffsetB_HL { get { return Register.Application.Parameters.Floats[23]; } set { Register.Application.Parameters.Floats[23] = value; } }
	public static float PF24_CalibOffsetA_HL { get { return Register.Application.Parameters.Floats[24]; } set { Register.Application.Parameters.Floats[24] = value; } }
    public static float PF25_CalibOffsetB_S  { get { return Register.Application.Parameters.Floats[25]; } set { Register.Application.Parameters.Floats[25] = value; } }
    public static float PF26_CalibOffsetA_S  { get { return Register.Application.Parameters.Floats[26]; } set { Register.Application.Parameters.Floats[26] = value; } }


    #endregion

    #region Tama variables floats

    public static float XPosMoveCmd_VF00 { set { Register.Application.Variables.Floats[00] = value; } get { return Register.Application.Variables.Floats[00]; } }
	public static float YPosMoveCmd_VF01 { set { Register.Application.Variables.Floats[01] = value; } get { return Register.Application.Variables.Floats[01]; } }

	public static float XPosCenter_VF10 { set { Register.Application.Variables.Floats[10] = value; } get { return Register.Application.Variables.Floats[10]; } }
	public static float YPosCenter_VF11 { set { Register.Application.Variables.Floats[11] = value; } get { return Register.Application.Variables.Floats[11]; } }
	public static float XPosTable_VF12 { set { Register.Application.Variables.Floats[12] = value; } get { return Register.Application.Variables.Floats[12]; } }
	public static float YPosTable_VF13 { set { Register.Application.Variables.Floats[13] = value; } get { return Register.Application.Variables.Floats[13]; } }
	public static float XPosPP_VF14 { set { Register.Application.Variables.Floats[14] = value; } get { return Register.Application.Variables.Floats[14]; } }
	public static float YPosPP_VF15 { set { Register.Application.Variables.Floats[15] = value; } get { return Register.Application.Variables.Floats[15]; } }
	public static float XAccTable_VF16 { set { Register.Application.Variables.Floats[16] = value; } get { return Register.Application.Variables.Floats[16]; } }
	public static float YAccTable_VF17 { set { Register.Application.Variables.Floats[17] = value; } get { return Register.Application.Variables.Floats[17]; } }

	public static float XIclinationPos_VF18 { set { Register.Application.Variables.Floats[18] = value; } get { return Register.Application.Variables.Floats[18]; } }
	public static float YIclinationPos_VF19 { set { Register.Application.Variables.Floats[19] = value; } get { return Register.Application.Variables.Floats[19]; } }
	public static float XIclinationVel_VF20 { set { Register.Application.Variables.Floats[20] = value; } get { return Register.Application.Variables.Floats[20]; } }
	public static float YIclinationVel_VF21 { set { Register.Application.Variables.Floats[21] = value; } get { return Register.Application.Variables.Floats[21]; } }
	public static float PlatformOrientation_VF22 { set { Register.Application.Variables.Floats[22] = value; } get { return Register.Application.Variables.Floats[22]; } }

	#endregion

	#region Tama variables integers

	public static int TamaVersion_VI00 { set { Register.Application.Variables.Integers[0] = value; } get { return Register.Application.Variables.Integers[0]; } }
	public static int WarningBitField_VI01 { set { Register.Application.Variables.Integers[1] = value; } get { return Register.Application.Variables.Integers[1]; } }
	public static int BarIdentification_VI02 { set { Register.Application.Variables.Integers[2] = value; } get { return Register.Application.Variables.Integers[2]; } }

	public static int V06_SubState1 { set { Register.Application.Variables.Integers[6] = value; } get { return Register.Application.Variables.Integers[6]; } }
	public static int V07_SubState2 { set { Register.Application.Variables.Integers[7] = value; } get { return Register.Application.Variables.Integers[7]; } }

	#endregion
}

