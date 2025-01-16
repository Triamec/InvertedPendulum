// Copyright © 2025 Triamec Motion AG

using Triamec.Tama.Rlid19;
using Triamec.Tama.Vmid5;
using Triamec.TriaLink;

[Tama]
class InvertedPendulum {
	public enum Command {
		NoCommand = 0,      // defined value to allow use of axis module in TamSE 
		Start = 1,          // defined value to allow use of axis module in TamSE 
		Stop = 2,           // defined value to allow use of axis module in TamSE 
		Disable = 3,
		MoveDynamic = 4,
		MoveToPhiZero = 5,
		ResetWarning = 6,
		ResetError = 7
	}

	enum State {
		Idle = 0,           // defined value to allow use of axis module in TamSE
		Startup = 1,
		Enabling = 2,
		MoveHomePos = 3,
		BeamDetection = 4,
		RegulatingDelay = 5,
		Calibration = 6,
		MoveSequence = 7,
		MovePhiZero = 8,
		Uncouple = 9,
		MoveDynamic = 10
	}

	/// <summary>Main state</summary>
	static State _state = State.Startup;

	/// <summary>Run is set to false if stopping is required.</summary>
	static bool _run;
	
	static bool _isControllerEnabled;
	static bool _isDynamicMoveEnabled;
	static bool _resetActive;
	static int _delayCountdown;

	/// <summary>State of switch at motor temperature input 0</summary>
	static bool _motTemp0SwitchStateOld;

	/// <summary>State of switch at motor temperature input 1</summary>
	static bool _motTemp1SwitchStateOld;

	/// <summary>Flag to indicate dry run request</summary>
	static bool _isDryRunRequestedOrStart;

	static InvertedPendulum() {
        TamaRegisters.TamaVersion_VI00 = Parameter.TamaVersion;
        _state = State.Startup;
        Beam.InitBeam();
		Controller.InitController();
    }

	[TamaTask(Task.IsochronousMain)]
	static void IsochronousTask() {

		// get command
		var command = (Command)Register.Application.TamaControl.IsochronousMainCommand;

		// check system state
		_run = CheckSystem(_run) && command != Command.Stop;

		// do controller calculation 
		Beam.CalcInclination();
		if (_isControllerEnabled) {
			PendulumPathPlanner.DoMoveStep(_run);
			Controller.DoControllerStep();
			Controller.DoBackwardKinematics();
		}

		if (_isDynamicMoveEnabled) {
			PendulumPathPlanner.DoMoveStep(_run);
			Controller.DoDynamicMoveStep();
		}

		// main state machine 
		switch (_state) {
			case State.Idle:
				switch (command) {
					case Command.Start:
						Utilities.ResetWarnings();
                        _delayCountdown = 10000;
                        _state = State.Startup;
						break;

					case Command.MoveToPhiZero:
						_run = true;
						AxisHandler.Move(AxisHandler.AxisId.Phi0, 0.0f, Parameter.TaxiSpeed, PathPlannerCommand.MoveAbsolute_Vel);
						AxisHandler.Move(AxisHandler.AxisId.Phi1, 0.0f, Parameter.TaxiSpeed, PathPlannerCommand.MoveAbsolute_Vel);
						_state = State.MovePhiZero;
						break;

					case Command.ResetWarning:
						Utilities.ResetWarnings();
						break;

					case Command.Disable:
						Register.Axes_0.Commands.General.Event = AxisEvent.DisableAxis;
						Register.Axes_1.Commands.General.Event = AxisEvent.DisableAxis;
						break;

					case Command.ResetError:
						if (Utilities.IsDeviceOrAxisErrorPending()) {
							
							// reset fault
							Register.General.Commands.ExternalError = false;
							Register.General.Commands.Internals.Event = DeviceEvent.ResetFault;
							Register.Axes_0.Commands.General.Event = AxisEvent.ResetError;
							Register.Axes_1.Commands.General.Event = AxisEvent.ResetError;
						}
						break;

					case Command.NoCommand:
					case Command.Stop:
					default:
						_run = false;
						break;
				}
				CheckTempSwitchState();
				if (_isDryRunRequestedOrStart) {
					Utilities.ResetWarnings();
                    _delayCountdown = 10000;
                    _state = State.Startup;
				}
				command = Command.NoCommand;
				break;

			case State.Startup:
				// Check for device or axes error
				if (Utilities.IsDeviceOrAxisErrorPending()) {
                    if (_delayCountdown > 0) {
                        // wait for DC bus
                        if (Register.General.Signals.DcBusVoltage > Register.General.Parameters.DcBusVoltageLowerLimit) {
                            _delayCountdown--;
                        }

                        Utilities.SetWarning(Utilities.Warning.DeviceOrAxisError);
                    } else if (!_resetActive) { 
						// try to reset
						Register.General.Commands.ExternalError = false;
						Register.General.Commands.Internals.Event = DeviceEvent.ResetFault;
						Register.Axes_0.Commands.General.Event = AxisEvent.ResetError;
						Register.Axes_1.Commands.General.Event = AxisEvent.ResetError;
						_resetActive = true;
					} else {
						Utilities.SetWarning(Utilities.Warning.ResetFailed);
					}
				} else {
					// enable axes
					_resetActive = false;
					_run = true;
					_isControllerEnabled = false;
					AxisHandler.Init();
					_state = State.Enabling;
				}
				break;

			case State.Enabling:
				if (AxisHandler.EnableAxes(_run)) {
					_state = GetNextState(State.MoveHomePos, command);
				}
				break;

			case State.MoveHomePos:
				if (AxisHandler.MoveHome(_run)) {
					Controller.Reset();
					_state = GetNextState(State.BeamDetection, command);
				}
				break;

			case State.BeamDetection:
				CheckTempSwitchState();
				if (!_run) {
					_state = GetNextState(State.BeamDetection, command);
				} else if (Beam.IsBeamReady()) {
					AxisHandler.CoupleAxes();
					Controller.Reset();
					PendulumPathPlanner.Reset(Robot.xPosHome, Robot.yPosHome);
					_delayCountdown = Parameter.RegulatingDelay;
					_state = State.RegulatingDelay;
				} else if (_isDryRunRequestedOrStart) {
					AxisHandler.CoupleAxes();
					Controller.Reset();
					PendulumPathPlanner.Reset(Robot.xPosHome, Robot.yPosHome);
					_isDynamicMoveEnabled = true;
					_state = State.MoveDynamic;
				}
				break;

			case State.RegulatingDelay:
				// delay used to stabilize velocity filter
				if (!_run) {
					_state = GetNextState(State.RegulatingDelay, command);
				} else if (_delayCountdown <= 0) {
					_isControllerEnabled = true;
					_delayCountdown = Parameter.CalibrationDelay;
					_state = State.Calibration;
				} else {
					_delayCountdown--;
				}
				break;

			case State.Calibration:
				// update calibration calculation
				if (Beam.CalcCalibValues()) {
					// reset counter if deviation is out of limit
					_delayCountdown = Parameter.CalibrationDelay;
				}
				if (Beam.IsBeamLost() || Utilities.IsPositionOutOfLimit() || Robot.isOutOfRange) {
					_isControllerEnabled = false;
					_state = State.Uncouple;
				} else if (_delayCountdown <= 0) {
					Controller.CalibOffset();
					PendulumPathPlanner.Reset(Robot.xPosHome, Robot.yPosHome);
					_state = State.MoveSequence;
				} else {
					_delayCountdown--;
				}
				break;

			case State.MoveSequence:
				if (Beam.IsBeamLost() || Utilities.IsPositionOutOfLimit() || Robot.isOutOfRange) {
					_isControllerEnabled = false;
					_state = State.Uncouple;
				} else if (PendulumPathPlanner.RunMoveSequence()) {
					_delayCountdown = Parameter.CalibrationDelay;
					_state = State.Calibration;
				}
				break;

			case State.MovePhiZero:
				if (AxisHandler.IsPathPlannerDone(AxisHandler.AxisId.Phi0)
					&& AxisHandler.IsPathPlannerDone(AxisHandler.AxisId.Phi1)) {
					_state = State.Idle;
				}
				break;

			case State.Uncouple:
				if (AxisHandler.UncoupleAxes()) {
					Beam.beamId = Beam.BeamId.Invalid;
					Beam.isVeryShortBeamRequested = false;
					if (_run) {
						AxisHandler.Init();
						_state = State.MoveHomePos;
					} else {
						_state = GetNextState(State.Idle, command);
					}
				}
				break;

			case State.MoveDynamic:
				if (!_run || Robot.isOutOfRange) {
					_isDynamicMoveEnabled = false;
					_state = State.Uncouple;
				} else if (PendulumPathPlanner.RunDynamicMoveSequence()) {
					_isDynamicMoveEnabled = false;
					_state = State.Uncouple;
				}
				break;
		}

		TamaRegisters.XPosTable_VF12 = Robot.xPos;
		TamaRegisters.YPosTable_VF13 = Robot.yPos;
		TamaRegisters.XPosPP_VF14 = PendulumPathPlanner.xState.pos;
		TamaRegisters.YPosPP_VF15 = PendulumPathPlanner.yState.pos;
		TamaRegisters.XIclinationPos_VF18 = Beam.xIncPos;
		TamaRegisters.YIclinationPos_VF19 = Beam.yIncPos;
		TamaRegisters.XIclinationVel_VF20 = Beam.xIncVel;
		TamaRegisters.YIclinationVel_VF21 = Beam.yIncVel;
		TamaRegisters.PlatformOrientation_VF22 = Robot.xi;

		Register.Application.TamaControl.IsochronousMainState = (int)_state;
		Register.Application.TamaControl.IsochronousMainCommand = (int)command;
	}

	/// <summary>
	/// Check if the system state is OK.
	/// </summary>
	static bool CheckSystem(bool run) {

		// just allow external move commands if axis are not coupled and state is Idle
		if (AxisHandler.IsAxesCouplingActive() || _state != State.Idle) {
			// reset external move commands
			if (Register.Axes_0.Commands.General.Event == AxisEvent.MoveCommand) {
				Register.Axes_0.Commands.General.Event = AxisEvent.None;
			}
			if (Register.Axes_1.Commands.General.Event == AxisEvent.MoveCommand) {
				Register.Axes_1.Commands.General.Event = AxisEvent.None;
			}
		}

		// check if stopping is required
		run &= !Utilities.IsDeviceOrAxisErrorPending();

		return run;
	}

	/// <summary>
	/// Returns next state depending on system state.
	/// </summary>
	static State GetNextState(State nextState, Command command) {
		if (command == Command.Stop) {
			nextState = State.Idle;
		} else if (!_run) {
            _delayCountdown = 10000;
            nextState = State.Startup;
		}
		return nextState;
	}

	/// <summary>
	/// Motor temperature is used to start dry run.
	/// </summary>
	static void CheckTempSwitchState() {

		// temp switch 0
		bool motTempSwitchState = Register.Axes_0.Signals.General.MotorTemperature > Parameter.MotTempSwitchThreshold;
		
		// trigger falling edge
		if (_motTemp0SwitchStateOld && !motTempSwitchState) {
			Beam.isVeryShortBeamRequested = true;
		}
		_motTemp0SwitchStateOld = motTempSwitchState;

		// temp switch 1
		_isDryRunRequestedOrStart = false;
		motTempSwitchState = Register.Axes_1.Signals.General.MotorTemperature > Parameter.MotTempSwitchThreshold;
		
		// trigger falling edge
		if (_motTemp1SwitchStateOld && !motTempSwitchState) {
			_isDryRunRequestedOrStart = true;
		}
		_motTemp1SwitchStateOld = motTempSwitchState;
	}
}
