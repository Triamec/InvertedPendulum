// Copyright © 2017 Triamec Motion AG

# define FAST_SEQUENCE

using Triamec.Tama.Vmid5;

/// <remarks>
/// The resulting dynamics can be read from <see cref="xState"/> and <see cref="yState"/>.
/// </remarks>
static class PendulumPathPlanner {
	public enum MoveType {
		None = 0,
		ToPoint,
		Circle,
		Lissajous,
		StayAtPosition
	}

	const float OneOverSqrtTwo = 0.70710678118654752440084436210485f;
	const float TwoPi = Math.PI * 2.0f;

	public static Utilities.MoveState xState;
	public static Utilities.MoveState yState;

	/// <summary>Search state</summary>
	static MoveType _moveType;

	/// <summary>Position offset y</summary>
	static float _x0;

	/// <summary>Position offset x</summary>
	static float _y0;

	/// <summary>Current radius</summary>
	static float _radius;

	/// <summary>Start angle</summary>
	static float _psi0;

	/// <summary>Cosine of move angle</summary>
	static float _cosA;

	/// <summary>Sine of move angle</summary>
	static float _sinA;

	/// <summary>Count down</summary>
	static int _durationCountdown;

	/// <summary>Duration to stay after move</summary>
	static float _stayDuration;

	/// <summary>Move sequence state</summary>
	static int _sequenceState;

    /// <summary>lissajous scaling</summary>
    static float lissajousScalingGain;

    /// <summary>
    /// Reset states
    /// </summary>
    public static void Reset(float xPos, float yPos) {
		_moveType = MoveType.None;
		_sequenceState = 0;

		// reset state
		xState.pos = xPos;
		xState.vel = 0.0f;
		xState.acc = 0.0f;
		xState.jrk = 0.0f;

		yState.pos = yPos;
		yState.vel = 0.0f;
		yState.acc = 0.0f;
		yState.jrk = 0.0f;

	}

	public static bool RunMoveSequence() {
		bool isDone = false;
		if (_moveType == MoveType.None) {
#if FAST_SEQUENCE
            switch (_sequenceState)
            {
                case 0:
                    InitToPointMove(Robot.xPosHome, Robot.yPosHome + 0.2f, 
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s, 
                        0.5f);
                    _sequenceState++;
                    break;
                case 1:
                    InitToPointMove(Robot.xPosHome - 0.25f, Robot.yPosHome + 0.2f,
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
                        0.5f);
                    _sequenceState++;
                    break;
                case 2:
                    InitToPointMove(Robot.xPosHome - 0.25f, Robot.yPosHome - 0.2f, 
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s, 
                        0.5f);
                    _sequenceState++;
                    break;
                case 3:
                    InitToPointMove(Robot.xPosHome, Robot.yPosHome - 0.2f,
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
                        0.5f);
                    _sequenceState++;
                    break;
                case 4:
                    InitToPointMove(Robot.xPosHome, Robot.yPosHome, 
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
                        0.0f);
                    _sequenceState++;
                    break;
                case 5:
                    // adjust interpolation error
                    InitToPointMove(Robot.xPosHome, Robot.yPosHome, 
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s/5.0f, TamaRegisters.PF12_Jrk_m3_s,
                        Parameter.WaitAfterMove);
                    _sequenceState = 7;
                    break;

                case 7:
                    {
                        float rotAngle;
                        switch (Beam.beamId)
                        {
                            case Beam.BeamId.BeamVerySort:
                                rotAngle = -1.0f;
                                break;
                            case Beam.BeamId.BeamHigh:
                                rotAngle = -1.0f;
                                break;
                            default:
                                rotAngle = -1.4f;
                                break;
                        }
                        InitCircleMove(0.0f, 0.0f, rotAngle,
                            TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
                            0.0f);
                        _sequenceState++;
                    }
                    break;
                case 8:
                    {
                        float rotAngle;
                        switch (Beam.beamId)
                        {
                            case Beam.BeamId.BeamVerySort:
                                rotAngle = 2.0f;
                                break;
                            case Beam.BeamId.BeamHigh:
                                rotAngle = 2.0f;
                                break;
                            default:
                                rotAngle = 2.8f;
                                break;
                        }
                        InitCircleMove(0.0f, 0.0f, rotAngle,
                            TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
                            0.0f);
                    }
                    _sequenceState++;
                    break;
                case 9:
                    {
                        float rotAngle;
                        switch (Beam.beamId)
                        {
                            case Beam.BeamId.BeamVerySort:
                                rotAngle = -1.0f;
                                break;
                            case Beam.BeamId.BeamHigh:
                                rotAngle = -1.0f;
                                break;
                            default:
                                rotAngle = -1.4f;
                                break;
                        }
                        InitCircleMove(0.0f, 0.0f, rotAngle,
                            TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
                            0.0f);
                    }
                    _sequenceState++;
                    break;
                case 10:
                    // adjust interpolation error
                    InitToPointMove(Robot.xPosHome , Robot.yPosHome, 
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s / 5.0f, 0.2f, 
                        Parameter.WaitAfterMove);
                    _sequenceState++;
                    break;
                case 11:
                    // spiral move
                    float dynGain;
                    switch (Beam.beamId)
                    {
                        case Beam.BeamId.BeamLow:
                            dynGain = 1.2f;                  
                            break;
                        case Beam.BeamId.BeamVerySort:
                            dynGain = 1.0f;
                            break;
                        case Beam.BeamId.BeamHigh:
                        default:
                            dynGain = 1.0f;
                            break;
                    }                  
                    InitCircleMove(Robot.xPosHome - Parameter.CircleRadius, Robot.yPosHome, 
                        Parameter.NumberOfCircleTurns * 2.0f * Math.PI, 
                        TamaRegisters.PF16_Vel_Circle_m_s* dynGain, TamaRegisters.PF17_Acc_Circle_m2_s, TamaRegisters.PF18_Jrk_Circle_m3_s, 
                        0.0f);
                    _sequenceState++;
                    break;
                case 12:
                    // adjust interpolation error
                    InitToPointMove(Robot.xPosHome, Robot.yPosHome,
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s / 5.0f, 0.2f,
                        Parameter.WaitAfterMove);
                    _sequenceState++;
                    break;
                case 13:
                    switch (Beam.beamId)
                    {
                        case Beam.BeamId.BeamLow:
                            dynGain = 1.0f;
                            lissajousScalingGain = 1.0f;
                            break;
                        case Beam.BeamId.BeamHigh:
                            dynGain = 0.7f;
                            lissajousScalingGain = 0.7f;
                            break;
                        case Beam.BeamId.BeamVerySort:
                            dynGain = 0.2f;
                            lissajousScalingGain = 0.8f;
                            break;
                        
                        default:
                            dynGain = 1.2f;
                            break;
                    }
                    InitLissajousMove(TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s* dynGain, TamaRegisters.PF12_Jrk_m3_s* dynGain,
                        0.0f);
                    _sequenceState ++;
                    break;
                case 14:
                    // adjust interpolation error
                    InitToPointMove(Robot.xPosHome, Robot.yPosHome,
                        TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s / 5.0f, 0.2f,
                        Parameter.WaitAfterMove);
                    _sequenceState++;
                    break;
                case 15:
                    _sequenceState = 0;
                    isDone = true;
                    break;
            }    
#else

            switch (_sequenceState) {

				case 0:
					InitToPointMove(Robot.xPosHome, Robot.yPosHome + 0.2f,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 1:
					InitToPointMove(Robot.xPosHome - 0.3f, Robot.yPosHome + 0.2f,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 2:
					InitToPointMove(Robot.xPosHome - 0.3f, Robot.yPosHome - 0.2f,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 3:
					InitToPointMove(Robot.xPosHome, Robot.yPosHome - 0.2f,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 4:
					// adjust interpolation error
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
						0.0f);
					_sequenceState++;
					break;

				case 5:
					// adjust interpolation error
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, 0.2f,
						0.0f);
					_sequenceState++;
					break;

				case 6:
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 7: {
						float rotAngle;
						switch (Beam.beamId) {
							case Beam.BeamId.BeamVerySort:
								rotAngle = -1.0f;
								break;
							default:
								rotAngle = -1.4f;
								break;
						}
						InitCircleMove(0.0f, 0.0f, rotAngle,
							TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
							Parameter.WaitAfterMove);
						_sequenceState++;
					}
					break;

				case 8: {
						float rotAngle;
						switch (Beam.beamId) {
							case Beam.BeamId.BeamVerySort:
								rotAngle = 2.0f;
								break;
							default:
								rotAngle = 2.8f;
								break;
						}
						InitCircleMove(0.0f, 0.0f, rotAngle,
							TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
							Parameter.WaitAfterMove);
					}
					_sequenceState++;
					break;

				case 9: {
						float rotAngle;
						switch (Beam.beamId) {
							case Beam.BeamId.BeamVerySort:
								rotAngle = -1.0f;
								break;
							default:
								rotAngle = -1.4f;
								break;
						}
						InitCircleMove(0.0f, 0.0f, rotAngle,
							TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
							0.0f);
					}
					_sequenceState++;
					break;

				case 10:
					// adjust interpolation error
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, 0.2f,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 11:
					float acc;
					switch (Beam.beamId) {
						case Beam.BeamId.BeamLow:
							acc = Parameter.CircleAccLow;
							break;
						case Beam.BeamId.BeamVerySort:
							acc = Parameter.CircleAccVeryShort;
							break;
						case Beam.BeamId.BeamHigh:
						default:
							acc = Parameter.CircleAccHigh;
							break;
					}
					InitCircleMove(Robot.xPosHome - Parameter.CircleRadius, Robot.yPosHome,
						Parameter.NumberOfCircleTurns * 2.0f * Math.PI, TamaRegisters.PF10_Vel_m_s, acc, TamaRegisters.PF12_Jrk_m3_s,
						0.0f);
					_sequenceState++;
					break;

				case 12:
					// adjust interpolation error
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, 0.2f,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 13:
					InitLissajousMove(TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, TamaRegisters.PF12_Jrk_m3_s,
						0.0f);
					_sequenceState++;
					break;

				case 14:
					// adjust interpolation error
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, 0.2f,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 15:
					_sequenceState = 0;
					isDone = true;
					break;
			}
#endif
        }
        return isDone;
	}

	public static bool RunDynamicMoveSequence() {
		bool isDone = false;
		if (_moveType == MoveType.None) {
			float vMax = 4.0f;
			float aMax = 20.0f;
			float jMax = 100.0f;
			float waitAfter = 0.1f;
			switch (_sequenceState) {
				case 0:
					InitToPointMove(Robot.xPosHome, Robot.yPosHome + 0.2f,
						vMax, aMax, jMax,
						waitAfter);
					_sequenceState++;
					break;

				case 1:
					InitToPointMove(Robot.xPosHome - 0.3f, Robot.yPosHome + 0.2f,
						vMax, aMax, jMax,
						waitAfter);
					_sequenceState++;
					break;

				case 2:
					InitToPointMove(Robot.xPosHome - 0.3f, Robot.yPosHome - 0.2f,
						vMax, aMax, jMax,
						waitAfter);
					_sequenceState++;
					break;

				case 3:
					InitToPointMove(Robot.xPosHome, Robot.yPosHome - 0.2f,
						vMax, aMax, jMax,
						waitAfter);
					_sequenceState++;
					break;

				case 4:
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						vMax, aMax, jMax,
						0.0f);
					_sequenceState++;
					break;

				case 5:
					// adjust interpolation error
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, 0.2f,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 6:
                    lissajousScalingGain = 1.0f;
                    InitLissajousMove(vMax, aMax, jMax,
						0.0f);
					_sequenceState++;
					break;

				case 7:
					// adjust interpolation error
					InitToPointMove(Robot.xPosHome, Robot.yPosHome,
						TamaRegisters.PF10_Vel_m_s, TamaRegisters.PF11_Acc_m2_s, 0.2f,
						Parameter.WaitAfterMove);
					_sequenceState++;
					break;

				case 8:
					_sequenceState = 0;
					isDone = true;
					break;
			}
		}
		return isDone;
	}

    /// <summary>
	/// Execution of path planner step
	/// </summary>
	public static void DoMoveStep(bool cmdRun) {
		bool isMoveDone = false;
		bool isPPDone = false;
		switch (_moveType) {

			case MoveType.ToPoint:
				isPPDone = PathPlanner3rdOrder.DoMoveStep(cmdRun);
				CalcToPointTraj();
				break;

			case MoveType.Circle:
				isPPDone = PathPlanner3rdOrder.DoMoveStep(cmdRun);
				CalcCircleTraj();
				break;

			case MoveType.Lissajous:
				isPPDone = PathPlanner3rdOrder.DoMoveStep(cmdRun);
				CalcLissajousTraj();
				break;

			case MoveType.StayAtPosition:
				if (_durationCountdown <= 0) {
					isMoveDone = true;
				} else {
					_durationCountdown--;
				}
				CalcStayAtPos();
				break;

			case MoveType.None:
			default:
				isMoveDone = true;
				break;
		}

		if (isPPDone) {
			InitStayAtPosition(_stayDuration);
		}

		if (isMoveDone) {
			_moveType = MoveType.None;
		}
	}

	public static void InitStayAtPosition(float duration) {
		_moveType = MoveType.StayAtPosition;
		_durationCountdown = (int)(duration * Parameter.SamplingFrequency);
		_x0 = xState.pos;
		_y0 = yState.pos;
	}

	/// <summary>
	/// Calculation delta position trajectory
	/// </summary>
	static void CalcStayAtPos() {
		// x axis
		xState.pos = _x0;
		xState.vel = 0.0f;
		xState.acc = 0.0f;
		xState.jrk = 0.0f;

		// y axis 
		yState.pos = _y0;
		yState.vel = 0.0f;
		yState.acc = 0.0f;
		yState.jrk = 0.0f;
	}

    /// <summary>
	/// Initialization delta position trajectory
	/// </summary>
    public static void InitToPointMove(float xEnd, float yEnd, float vMax, float aMax, float jMax, float stayAfterDuration) {
		_moveType = MoveType.ToPoint;
		_stayDuration = stayAfterDuration;
		_x0 = xState.pos;
		_y0 = yState.pos;
		// distance to end point
		float dx = xEnd - xState.pos;
		float dy = yEnd - yState.pos;
		float delta = Math.Sqrt(dx * dx + dy * dy);
		// mapping to axes
		if (delta > 0) {
			_cosA = dx / delta;
			_sinA = dy / delta;
		} else {
			_cosA = 0.0f;
			_sinA = 0.0f;
		}

		// prepare path planner    
		PathPlanner3rdOrder.InitPositionMove(delta, vMax, aMax, jMax);
	}

    /// <summary>
	/// Calculation delta position trajectory
	/// </summary>
	static void CalcToPointTraj() {
		// x axis
		xState.pos = PathPlanner3rdOrder.Pos * _cosA + _x0;
		xState.vel = PathPlanner3rdOrder.Vel * _cosA;
		xState.acc = PathPlanner3rdOrder.Acc * _cosA;
		xState.jrk = PathPlanner3rdOrder.Jrk * _cosA;

		// y axis
		yState.pos = PathPlanner3rdOrder.Pos * _sinA + _y0;
		yState.vel = PathPlanner3rdOrder.Vel * _sinA;
		yState.acc = PathPlanner3rdOrder.Acc * _sinA;
		yState.jrk = PathPlanner3rdOrder.Jrk * _sinA;
	}

	public static void InitCircleMove(float xCenter, float yCenter, float rotationAngle, float vMax, float aMax, float jMax, float stayAfterDuration) {
		// set path planner type
		_moveType = MoveType.Circle;
		_stayDuration = stayAfterDuration;
		// radius and angle are defined by the connection form the center of rotation to the current position
		_x0 = xCenter;
		_y0 = yCenter;
		float dx = (xState.pos - xCenter);
		float dy = (yState.pos - yCenter);
		_psi0 = Math.Atan2(dy, dx);
		_radius = Math.Sqrt(dx * dx + dy * dy);
		// max angular velocity depending on linear restrictions
		float w = 0.0f;
		if (_radius > 0.0f) {
			w = vMax / _radius;
		}

		// prepare path planner tau = tau(t)
		// to avoid multiplication with w during trajectory calculation, the constraints are multiplied by w: psi = tau * w
		PathPlanner3rdOrder.InitPositionMove(rotationAngle, w, aMax * w / vMax, jMax * w / vMax);
	}

	static void CalcCircleTraj() {
		float psi = PathPlanner3rdOrder.Pos + _psi0;

		float sinPsiR = Math.Sin(psi) * _radius;
		float cosPsiR = Math.Cos(psi) * _radius;
		float vel = PathPlanner3rdOrder.Vel;
		float vel2 = vel * vel;
		float vel3 = vel2 * vel;

		// x axis dynamics
		xState.pos = cosPsiR + _x0;
		xState.vel = -sinPsiR * vel;
		xState.acc = -cosPsiR * vel2 - sinPsiR * PathPlanner3rdOrder.Acc;
		xState.jrk = sinPsiR * (vel3 - PathPlanner3rdOrder.Jrk) - 3.0f * cosPsiR * vel * PathPlanner3rdOrder.Acc;

		// y axis dynamics
		yState.pos = sinPsiR + _y0;
		yState.vel = cosPsiR * vel;
		yState.acc = -sinPsiR * vel2 + cosPsiR * PathPlanner3rdOrder.Acc;
		yState.jrk = -cosPsiR * (vel3 - PathPlanner3rdOrder.Jrk) - 3.0f * sinPsiR * vel * PathPlanner3rdOrder.Acc;
	}

	public static void InitLissajousMove(float vMax, float aMax, float jMax, float stayAfterDuration) {
		// set path planner type
		_moveType = MoveType.Lissajous;
		_stayDuration = stayAfterDuration;
		// radius and angle are defined by the connection form the center of rotation to the current position
		_x0 = Robot.xPosHome - TamaRegisters.PF13_Radius0_m* lissajousScalingGain * OneOverSqrtTwo;
		_y0 = Robot.yPosHome;
		_psi0 = Math.PI / 4.0f;

		// max angular velocity depending on linear restrictions
		float w = 0.0f;
		_radius = TamaRegisters.PF13_Radius0_m > TamaRegisters.PF14_Radius1_m ? TamaRegisters.PF13_Radius0_m : TamaRegisters.PF14_Radius1_m;
		if (_radius > 0.0f) {
			w = 0.5f * vMax / _radius;
		}

		// prepare path planner tau = tau(t)
		// to avoid multiplication with w during trajectory calculation, the constraints are multiplied by w: psi = tau * w
		PathPlanner3rdOrder.InitPositionMove(Parameter.NumberOfLissajousTurns * 2 * Math.PI, w, aMax * w / vMax, jMax * w / vMax);
	}

	static void CalcLissajousTraj() {
		float psiX = (2 * PathPlanner3rdOrder.Pos + _psi0);
		float psiY = PathPlanner3rdOrder.Pos;

		float vel = PathPlanner3rdOrder.Vel;
		float vel2 = vel * vel;
		float vel3 = vel2 * vel;

		// x axis dynamics
		float cosPsiR = Math.Cos(psiX) * TamaRegisters.PF13_Radius0_m * lissajousScalingGain;
		float sinPsiR = Math.Sin(psiX) * TamaRegisters.PF13_Radius0_m * lissajousScalingGain;
		xState.pos = sinPsiR + _x0;
		xState.vel = 2.0f * cosPsiR * vel;
		xState.acc = -4.0f * sinPsiR * vel2 + 2.0f * cosPsiR * PathPlanner3rdOrder.Acc;
		xState.jrk = -cosPsiR * (8.0f * vel3 - 2.0f * PathPlanner3rdOrder.Jrk) - 12.0f * sinPsiR * vel * PathPlanner3rdOrder.Acc;

		// y axis dynamics
		cosPsiR = Math.Cos(psiY) * TamaRegisters.PF14_Radius1_m * lissajousScalingGain;
		sinPsiR = Math.Sin(psiY) * TamaRegisters.PF14_Radius1_m * lissajousScalingGain;
		yState.pos = sinPsiR + _y0;
		yState.vel = cosPsiR * vel;
		yState.acc = -sinPsiR * vel2 + cosPsiR * PathPlanner3rdOrder.Acc;
		yState.jrk = -cosPsiR * (vel3 - PathPlanner3rdOrder.Jrk) - 3.0f * sinPsiR * vel * PathPlanner3rdOrder.Acc;
	}
}

