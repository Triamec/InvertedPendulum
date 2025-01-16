// Copyright © 2017 Triamec Motion AG

using Triamec.Tama.Vmid5;

static class Parameter {

    /// <summary>tama version</summary>
    public const int TamaVersion = 2;

    /// <summary>Sampling frequency [Hz]</summary>
    public const float SamplingFrequency = 10000.0f;

	/// <summary>Sampling time [s]</summary>
	public const float SamplingTime = 1.0f / SamplingFrequency;

	/// <summary>deg to radian [rad/deg]</summary>
	public const float DegToRad = Math.PI / 180.0f;

	/// <summary>Gravity [m/s^2]</summary>
	public const float g = 9.81f;

	/// <summary>Delay [inc] before activation of controller</summary>
	public const int RegulatingDelay = (int)(0.2f * SamplingFrequency);

	/// <summary>Delay [inc] used to stabilize calibration</summary>
	public const int CalibrationDelay = (int)(8f * SamplingFrequency);

	/// <summary>Threshold [inc] to prolong calibration interval</summary>
	public const float CalibIncThreshold = 0.01f / HallGainB_LL;

	/// <summary>Threshold [deg] used for input switch detection</summary>
	public const float MotTempSwitchThreshold = 150f;

	#region axis parameters
	
	/// <summary>Lower limit phi0 [rad]</summary>
	public const float Phi0PosMin = -180f * DegToRad;

	/// <summary>Upper limit phi0 [rad]</summary>
	public const float Phi0PosMax = 60f * DegToRad;

	/// <summary>Lower limit phi1 [rad]</summary>
	public const float Phi1PosMin = -60f * DegToRad;

	/// <summary>Upper limit phi1 [rad]</summary>
	public const float Phi1PosMax = 180f * DegToRad;

	/// <summary>Lower limit delta phi1-phi0 [rad]</summary>
	public const float PhiDeltaPosMin = 0.0f * DegToRad;
															 
	/// <summary>Upper limit delta phi1-phi0 [rad]</summary>
	public const float PhiDeltaPosMax = 180.0f * DegToRad;
	
	/// <summary>Home position [rad] phi0</summary>
	public const float Phi0HomePosition = -45f * DegToRad;

	/// <summary>Home position [rad] phi1</summary>
	public const float Phi1HomePosition = 45f * DegToRad;

	/// <summary>Speed [rad/s] used for taxi motion</summary>
	public const float TaxiSpeed = 2.0f;
	#endregion

	#region controller parameters

	/// <summary>Lock in threshold [rad]</summary>
	public const float LockInThreshold = 2.0f * DegToRad;

	/// <summary>Lock out threshold [rad]</summary>
	public const float LockOutThreshold = 35.0f * DegToRad;

	/// <summary>Integrator limit [rad]</summary>
	public const float IntegraorLimit = 2.5f * DegToRad;

	/// <summary>max acceleration output [m/s^2]</summary>
	public const float MaxAccOut = 10.0f;

	#endregion

	#region robot geometry

	/// <summary>Length [m] of beam 1</summary>
	public const float LengthBeam1 = 0.34712f;

	/// <summary>Length [m] of beam 2</summary>
	public const float LengthBeam2 = 0.41063f;

	/// <summary>Distance [m] between motor shafts</summary>
	public const float DistanceMotorShaft = 0.11f;

	#endregion

	#region beam

	/// <summary>Filter bandwidth [Hz] used for calibration</summary>
	public const float CalibFiltFreq = 0.1f;
	
	/// <summary>Undervoltage [V] beam level</summary>
	public const float BeamUndervoltage = 0.3f;

	#region beam low level (160mm)

	/// <summary>Low level [V] used for beam identification</summary>
	public const float BeamIdLowLevel = 1.1f;

    /// <summary>Natural frequency [1/s] of beam.</summary>
    public const float WNLowLevel160mm = 9.0f;

	/// <summary>Length [m] of mathematical pendulum</summary>
	public const float L0LowLevel160mm = g / WNLowLevel160mm / WNLowLevel160mm;

	/// <summary>Hall sensor gain [rad/inc] in B direction.</summary>
	public const float HallGainB_LL = 5.82786e-005f;

	/// <summary>Hall sensor gain [rad/inc], in A direction.</summary>
	public const float HallGainA_LL = -5.81337e-005f;

	#endregion

	#region beam low level (22mm, very short)

	/// <summary>Natural frequency [Hz] of beam</summary>
	public const float WNVeryShort22mm = 15.0f; //8.5f;
	
	/// <summary>length [m] of mathematical pendulum</summary>
	public const float L0VeryShort22mm = g / WNVeryShort22mm / WNVeryShort22mm;

	#endregion

	#region beam high level (480mm)

	/// <summary>High level [V] used for beam identification.</summary>
	public const float BeamIdHighLevel = 2.1f;
	
	/// <summary>Natural frequency [Hz] of beam.</summary>
	public const float WNHighLevel970mm = 3.8f;
    public const float WNHighLevel460mm = 5.0f;

    /// <summary>Length [m] of mathematical pendulum</summary>
    public const float L0HighLevel970mm = g / WNHighLevel970mm / WNHighLevel970mm;
    public const float L0HighLevel460mm = g / WNHighLevel460mm / WNHighLevel460mm;

    /// <summary>Hall sensor gain [rad/inc] in B direction</summary>
    public const float HallGainB_HL = -5.92493e-005f;
	
	/// <summary>Hall sensor gain [rad/inc] in B direction</summary>
	public const float HallGainA_HL = 5.83887e-005f;

	#endregion
	#endregion

	#region path planner

	public const int NumberOfCircleTurns = 12;
	public const int NumberOfLissajousTurns = 2;
	
	/// <summary>Wait time [s] after move</summary>
	public const float WaitAfterMove = 1.0f;

	/// <summary>Circle radius [m]</summary>
	public const float CircleRadius = 0.03f;

	/// <summary>Acceleration of time used for circle ramp up</summary>
	public const float CircleAccLow = 0.013f;

	/// <summary>Acceleration of time used for circle ramp up</summary>
	public const float CircleAccHigh = 0.01f;

	/// <summary>Acceleration of time used for circle ramp up</summary>
	public const float CircleAccVeryShort = 0.01f;
	
	#endregion
}

