// Copyright © 2017 Triamec Motion AG

using Triamec.Tama.Rlid19;
using Triamec.Tama.Vmid5;

static class Beam {
	/*****************************************************************
    * type definitions
    *****************************************************************/
	public enum BeamId {
		Invalid = 0,
		BeamLow = 1,
		BeamHigh = 2,
		BeamVerySort = 3
	}

	/*****************************************************************
    * constants
    *****************************************************************/
	public const float TwoPi = 2.0f * Math.PI;

	/*****************************************************************
    * public members
    *****************************************************************/
	
	/// <summary>inclination angle x direction</summary>
	public static float xIncPos;
	
	/// <summary> inclination angle y direction</summary>
	public static float yIncPos;

	/// <summary>velocity of inclination angle x direction</summary>
	public static float xIncVel;

	/// <summary>velocity of inclination angle x direction</summary>
	public static float yIncVel;

	/// <summary>old inclination angle x direction used for velocity calculation</summary>
	public static float xIncPos_Old;
	
	/// <summary>old inclination angle y direction used for velocity calculation</summary>
	public static float yIncPos_Old;

	/// <summary>velocity filter coefficient</summary>
	public static float alphaVel;
	
	/// <summary>beam identification</summary>
	public static BeamId beamId;

	/// <summary>controller gain</summary>
	public static float[] K;
	
	/// <summary>length of physical pendulum</summary>
	public static float l0;
	
	/// <summary>offset used for inclination calculation in B-direction</summary>
	public static float IncBOffset;

	/// <summary>offset used for inclination calculation in A-direction</summary>
	public static float IncAOffset;

	/// <summary>gain used for inclination calculation in B-direction</summary>
	public static float IncBGain;

	/// <summary>gain used for inclination calculation in A-direction</summary>
	public static float IncAGain;

	/// <summary>integrator limit</summary>
	public static float IntegratorLimit;


	/// <summary>calibration value B</summary>
	public static float calibValueB;

	/// <summary>calibration value A</summary>
	public static float calibValueA;

	/// <summary>filter constant used for calibration</summary>
	public static float alphaCalib;

	/// <summary>flag to indicate very short beam</summary>
	public static bool isVeryShortBeamRequested;

	/// <summary>
	/// Static constructor replacement.
	/// </summary>
	public static void InitBeam() {
		K = new float[5];
		beamId = BeamId.Invalid;
		alphaCalib = Math.Exp(-Parameter.CalibFiltFreq * TwoPi * Parameter.SamplingTime);
	}

	public static void CalibOffset() {
		IncBOffset = -calibValueB;
		IncAOffset = -calibValueA;

		xIncVel = 0.0f;
		yIncVel = 0.0f;
		xIncPos_Old = 0.0f;
		yIncPos_Old = 0.0f;

		switch (beamId) {
			case BeamId.BeamLow:
				TamaRegisters.PF21_CalibOffsetB_LL = IncBOffset;
				TamaRegisters.PF22_CalibOffsetA_LL = IncAOffset;
				break;

			case BeamId.BeamHigh:
				TamaRegisters.PF23_CalibOffsetB_HL = IncBOffset;
				TamaRegisters.PF24_CalibOffsetA_HL = IncAOffset;
				break;

			case BeamId.BeamVerySort:

				// do not change calibration values
				break;
		}
	}

	/// <summary>
	/// Inclination calculation.
	/// </summary>
	public static void CalcInclination() {
		float cosXi = Math.Cos(Robot.xi);
		float sinXi = Math.Sin(Robot.xi);

		float bInc = (Register.Axes_1.Signals.PositionController.Encoders_1.PhaseB + IncBOffset) * IncBGain;
		float aInc = (Register.Axes_1.Signals.PositionController.Encoders_1.PhaseA + IncAOffset) * IncAGain;

		// map angles to xy coordinates
		xIncPos = cosXi * bInc - sinXi * aInc;
		yIncPos = sinXi * bInc + cosXi * aInc;

		// calculation of angular velocity
		xIncVel = alphaVel * xIncVel + (1.0f - alphaVel) * (xIncPos - xIncPos_Old) * Parameter.SamplingFrequency;
		yIncVel = alphaVel * yIncVel + (1.0f - alphaVel) * (yIncPos - yIncPos_Old) * Parameter.SamplingFrequency;
		xIncPos_Old = xIncPos;
		yIncPos_Old = yIncPos;
	}

	public static bool CalcCalibValues() {

		// used for calibration
		calibValueB = alphaCalib * calibValueB + (1 - alphaCalib) * Register.Axes_1.Signals.PositionController.Encoders_1.PhaseB;
		calibValueA = alphaCalib * calibValueA + (1 - alphaCalib) * Register.Axes_1.Signals.PositionController.Encoders_1.PhaseA;

		// check if deviation out of limit
		return Math.Fabs(calibValueB - Register.Axes_1.Signals.PositionController.Encoders_1.PhaseB) > Parameter.CalibIncThreshold ||
			Math.Fabs(calibValueA - Register.Axes_1.Signals.PositionController.Encoders_1.PhaseA) > Parameter.CalibIncThreshold;
	}

	public static bool IsBeamReady() {
		bool isReady = false;
		BeamId bId = IdentifyBeam();
		if (bId != BeamId.Invalid) {
			if (beamId == bId) {
				if (Math.Fabs(xIncPos) < Parameter.LockInThreshold && Math.Fabs(yIncPos) < Parameter.LockInThreshold) {

					// reset velocity values
					xIncVel = 0.0f;
					yIncVel = 0.0f;
					xIncPos = 0.0f;
					yIncPos = 0.0f;
					xIncPos_Old = 0.0f;
					yIncPos_Old = 0.0f;
					isReady = true;
				}
			} else {
				SetParameters(bId);
			}
		}
		return isReady;
	}

	public static bool IsBeamLost() {
		bool isLost = false;
		BeamId bId = IdentifyBeam();
		if (bId != beamId || Math.Fabs(xIncPos) > Parameter.LockOutThreshold || Math.Fabs(yIncPos) > Parameter.LockOutThreshold) {
			isLost = true;
		}

		return isLost;
	}

	public static BeamId IdentifyBeam() {
		BeamId beamId;
		float beamIdLevel = Register.Axes_0.Signals.OptionModule.AnalogIn[0];

		if (beamIdLevel < Parameter.BeamUndervoltage) {
			Utilities.SetWarning(Utilities.Warning.BeamDetectionUndervoltage);
		}
		if (beamIdLevel < Parameter.BeamIdLowLevel) {
			beamId = isVeryShortBeamRequested ? BeamId.BeamVerySort : BeamId.BeamLow;
		} else if (beamIdLevel > Parameter.BeamIdHighLevel) {
			beamId = BeamId.BeamHigh;
		} else {
			beamId = BeamId.Invalid;
		}
		TamaRegisters.BarIdentification_VI02 = (int)beamId;
		return beamId;
	}

	public static void SetParameters(BeamId bId) {
		float WN2;
		beamId = bId;
		switch (bId) {
			case BeamId.BeamLow: // assigned to short beam 160mm
				WN2 = Parameter.WNLowLevel * Parameter.WNLowLevel;
				l0 = Parameter.L0LowLevel;
				IncBOffset = TamaRegisters.PF21_CalibOffsetB_LL;
				IncAOffset = TamaRegisters.PF22_CalibOffsetA_LL;
				IncBGain = Parameter.HallGainB_LL;
				IncAGain = Parameter.HallGainA_LL;
				break;

			case BeamId.BeamHigh: // assigned to long beam 480mm
				WN2 = Parameter.WNHighLevel * Parameter.WNHighLevel;
				l0 = Parameter.L0HighLevel;
				IncBOffset = TamaRegisters.PF23_CalibOffsetB_HL;
				IncAOffset = TamaRegisters.PF24_CalibOffsetA_HL;
				IncBGain = Parameter.HallGainB_HL;
				IncAGain = Parameter.HallGainA_HL;
				break;

			case BeamId.BeamVerySort:
				WN2 = Parameter.WNLowLevel * Parameter.WNVeryShort;
				l0 = Parameter.L0VeryShort;
				IncBOffset = TamaRegisters.PF21_CalibOffsetB_LL;
				IncAOffset = TamaRegisters.PF22_CalibOffsetA_LL;
				IncBGain = Parameter.HallGainB_LL;
				IncAGain = Parameter.HallGainA_LL;
				break;

			default:
				WN2 = Parameter.WNLowLevel * Parameter.WNLowLevel;
				l0 = Parameter.L0LowLevel;
				IncBOffset = TamaRegisters.PF21_CalibOffsetB_LL;
				IncAOffset = TamaRegisters.PF22_CalibOffsetA_LL;
				IncBGain = Parameter.HallGainB_LL;
				IncAGain = Parameter.HallGainA_LL;
				break;
		}

		alphaVel = Math.Exp(-TamaRegisters.P03_fVel_Hz * TwoPi * Parameter.SamplingTime);

		float wi = TamaRegisters.P02_fIntCtrl * TwoPi;
		float w0 = TamaRegisters.P01_fStateCtrl_Hz * TwoPi;
		float w02 = w0 * w0;
		float w03 = w02 * w0;
		float D0 = TamaRegisters.PF00_DStateCtrl;

		float a0 = w03 * w0;
		float a1 = 4.0f * D0 * w03;
		float a2 = (2.0f * w02 + 4 * D0 * D0 * w02 + WN2);
		float a3 = 4.0f * D0 * w0;

		// gain used to transformation from controller normal form output to acceleration of the effector
		float Kt = -1.0f / WN2;

		// controller gain
		K[0] = Kt * (a0 * wi) * Parameter.SamplingTime;
		K[1] = Kt * (a0 + a1 * wi);
		K[2] = Kt * (a1 + a2 * wi);
		K[3] = Kt * (a2 + a3 * wi);
		K[4] = Kt * (a3 + wi);

		// integrator limit
		IntegratorLimit = Math.Fabs(Parameter.IntegraorLimit * Parameter.g * K[3]);

		// reset calibration values
		calibValueB = -IncBOffset;
		calibValueA = -IncAOffset;
	}
}
