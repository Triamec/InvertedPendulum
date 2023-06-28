// Copyright © 2017 Triamec Motion AG

using Triamec.TriaLink;
using Triamec.Tama.Rlid19;

static class AxisHandler {

	/// <summary>States used for enabling.</summary>
	enum EnablingState {
		Idle = 0,
		SwitchOn,
		EnablePhi0,
		EnablePhi1,
		MoveHome
	}

	public enum MoveHomeState {
		Idle = 0,
		WaitMoveDelta,
		WaitDone
	}

	public enum AxisId {
		Phi0 = 0,
		Phi1 = 1
	}

	public enum UncoupleState {
		Init = 0,
		Stop = 1,
		WaitMoveDone = 2
	}

	/// <summary>Decrement counter for decoupling.</summary>
	static UncoupleState _uncoupleState;
	
	/// <summary>state used for enabling</summary>
	static EnablingState _enablingState;

	/// <summary>move home state</summary>
	static MoveHomeState _moveHomeState;

	public static void Init() {
		_enablingState = EnablingState.Idle;
		_moveHomeState = MoveHomeState.Idle;
	}

	public static bool EnableAxes(bool run) {
		bool isDone = false;
		switch (_enablingState) {
			case EnablingState.Idle:
				if (run) {
					_enablingState = EnablingState.SwitchOn;
				} else {
					isDone = true;
				}
				break;

			case EnablingState.SwitchOn:
				if (run) {
					if (SwitchOn()) {
						_enablingState = EnablingState.EnablePhi0;
					}
				} else {
					isDone = true;
					_enablingState = EnablingState.Idle;
				}
				break;

			case EnablingState.EnablePhi0:
				if (run) {
					if (Enable(AxisId.Phi0)) {
						_enablingState = EnablingState.EnablePhi1;
					}
				} else {
					isDone = true;
					_enablingState = EnablingState.Idle;
				}
				break;

			case EnablingState.EnablePhi1:
				if (run) {
					if (Enable(AxisId.Phi1)) {
						_enablingState = EnablingState.MoveHome;
						_moveHomeState = MoveHomeState.Idle;
					}
				} else {
					isDone = true;
					_enablingState = EnablingState.Idle;
				}
				break;

			case EnablingState.MoveHome:
				if (MoveHome(run) || !run) {
					isDone = true;
					_enablingState = EnablingState.Idle;
				}
				break;
		}
		TamaRegisters.V06_SubState1 = (int)_enablingState;
		return isDone;
	}
	
	static bool Enable(AxisId axisId) {
		bool isDone = false;
		switch (axisId) {
			case AxisId.Phi0:
				switch (Register.Axes_0.Signals.General.AxisState) {
					case AxisState.Disabled:

						// enable axis
						Register.Axes_0.Commands.General.Event = AxisEvent.EnableAxis;
						break;

					case AxisState.NotReady:
					case AxisState.Enabling:
					case AxisState.ErrorStopping:
					case AxisState.Stopping:
						break;

					default:

						// drive is enabled
						isDone = true;
						break;
				}
				break;

			case AxisId.Phi1:
				switch (Register.Axes_1.Signals.General.AxisState) {
					case AxisState.Disabled:

						// enable axis
						Register.Axes_1.Commands.General.Event = AxisEvent.EnableAxis;
						break;

					case AxisState.NotReady:
					case AxisState.Enabling:
					case AxisState.ErrorStopping:
					case AxisState.Stopping:
						break;

					default:

						// drive is enabled
						isDone = true;
						break;
				}
				break;

			default:
				Utilities.SetWarning(Utilities.Warning.InternalUndefinedCase);
				break;
		}
		return isDone;
	}

	/// <summary>
	/// Move to home position.
	/// </summary>
	public static bool MoveHome(bool run) {
		bool isDone = false;

		if (_moveHomeState != MoveHomeState.WaitDone && !run) {
			Stop(AxisId.Phi0);
			Stop(AxisId.Phi1);
			_moveHomeState = MoveHomeState.WaitDone;
		}

		switch (_moveHomeState) {
			case MoveHomeState.Idle:

				// decide which arm to move based on the center location
				if (Register.Axes_0.Signals.PositionController.Encoders_0.PositionFloat + Register.Axes_1.Signals.PositionController.Encoders_0.PositionFloat > 0.0f) {
					
					// move with arm 1
					float pos = Register.Axes_0.Signals.PositionController.Encoders_0.PositionFloat + (Parameter.Phi1HomePosition - Parameter.Phi0HomePosition);
					Move(AxisId.Phi1, pos, Parameter.TaxiSpeed, PathPlannerCommand.MoveAbsolute_Vel);
				} else {
					
					// move with arm 0
					float pos = Register.Axes_1.Signals.PositionController.Encoders_0.PositionFloat - (Parameter.Phi1HomePosition - Parameter.Phi0HomePosition);
					Move(AxisId.Phi0, pos, Parameter.TaxiSpeed, PathPlannerCommand.MoveAbsolute_Vel);
				}
				_moveHomeState = MoveHomeState.WaitMoveDelta;
				break;

			case MoveHomeState.WaitMoveDelta:
				if (IsPathPlannerDone(AxisId.Phi0) && IsPathPlannerDone(AxisId.Phi1)) {
					Move(AxisId.Phi0, Parameter.Phi0HomePosition, Parameter.TaxiSpeed, PathPlannerCommand.MoveAbsolute_Vel);
					Move(AxisId.Phi1, Parameter.Phi1HomePosition, Parameter.TaxiSpeed, PathPlannerCommand.MoveAbsolute_Vel);
					_moveHomeState = MoveHomeState.WaitDone;
				}
				break;

			case MoveHomeState.WaitDone:
				if (IsPathPlannerDone(AxisId.Phi0) && IsPathPlannerDone(AxisId.Phi1)) {
					isDone = true;
				}
				break;
		}
		TamaRegisters.V07_SubState2 = (int)_moveHomeState;
		return isDone;
	}

	static bool SwitchOn() {
		bool done = false;

		switch (Register.General.Signals.DriveState) {
			case (DeviceState.FaultPending):

				// reset fault
				Register.General.Commands.Internals.Event = DeviceEvent.ResetFault;
				break;

			case (DeviceState.ReadyToSwitchOn):

				// switch the drive on
				Register.General.Commands.Internals.Event = DeviceEvent.SwitchOn;
				break;

			case DeviceState.Operational:
				done = true;
				break;
		}
		return done;
	}

	public static bool UncoupleAxes() {
		bool isDone = false;

		switch (_uncoupleState) {
			case UncoupleState.Init:
				Register.Axes_0.Commands.PathPlanner.StreamV = GetLimitedState(Register.Axes_0.Commands.PathPlanner.StreamV, Register.Axes_0.Parameters.PathPlanner.VelocityMaximum);
				Register.Axes_0.Commands.PathPlanner.StreamA = GetLimitedState(Register.Axes_0.Commands.PathPlanner.StreamA, Register.Axes_0.Parameters.PathPlanner.AccelerationMaximum);
				Register.Axes_1.Commands.PathPlanner.StreamV = GetLimitedState(Register.Axes_1.Commands.PathPlanner.StreamV, Register.Axes_1.Parameters.PathPlanner.VelocityMaximum);
				Register.Axes_1.Commands.PathPlanner.StreamA = GetLimitedState(Register.Axes_1.Commands.PathPlanner.StreamA, Register.Axes_1.Parameters.PathPlanner.AccelerationMaximum);

				Register.Axes_0.Commands.PathPlanner.Command = PathPlannerCommand.Stop;
				Register.Axes_1.Commands.PathPlanner.Command = PathPlannerCommand.Stop;
				_uncoupleState = UncoupleState.WaitMoveDone;
				break;

			case UncoupleState.WaitMoveDone:
				if (IsPathPlannerDone(AxisId.Phi0) && IsPathPlannerDone(AxisId.Phi1)) {
					isDone = true;
				}
				break;

			default:
				Utilities.SetWarning(Utilities.Warning.InternalUndefinedCase);
				break;
		}
		return isDone;
	}

	public static void CoupleAxes() {

		// check if axis is in valid state
		if ((Register.Axes_0.Signals.General.AxisState == AxisState.Standstill ||
			Register.Axes_0.Signals.General.AxisState == AxisState.DirectCoupledMotion) &&
			(Register.Axes_1.Signals.General.AxisState == AxisState.Standstill ||
			Register.Axes_1.Signals.General.AxisState == AxisState.DirectCoupledMotion)) {

			// phi0 axis: check if not already coupled
			if (Register.Axes_0.Signals.General.AxisState != AxisState.DirectCoupledMotion) {
				Register.Axes_0.Commands.PathPlanner.StreamX = Register.Axes_0.Signals.PathPlanner.Position;
				Register.Axes_0.Commands.PathPlanner.StreamV = 0.0f;
				Register.Axes_0.Commands.PathPlanner.StreamA = 0.0f;
				Register.Application.Axes_0.Commanded.Position = Register.Axes_0.Signals.PathPlanner.Position;
				Register.Application.Axes_0.Commanded.Velocity = 0.0f;
				Register.Application.Axes_0.Commanded.Acceleration = 0.0f;
				Register.Axes_0.Commands.PathPlanner.Command = PathPlannerCommand.MoveDirectCoupled;
			}

			// phi1 axis: check if not already coupled
			if (Register.Axes_0.Signals.General.AxisState != AxisState.DirectCoupledMotion) {
				Register.Axes_1.Commands.PathPlanner.StreamX = Register.Axes_1.Signals.PathPlanner.Position;
				Register.Axes_1.Commands.PathPlanner.StreamV = 0.0f;
				Register.Axes_1.Commands.PathPlanner.StreamA = 0.0f;
				Register.Application.Axes_1.Commanded.Position = Register.Axes_1.Signals.PathPlanner.Position;
				Register.Application.Axes_1.Commanded.Velocity = 0.0f;
				Register.Application.Axes_1.Commanded.Acceleration = 0.0f;
				Register.Axes_1.Commands.PathPlanner.Command = PathPlannerCommand.MoveDirectCoupled;
			}

			// prepare uncouple
			_uncoupleState = UncoupleState.Init;
		} else {
			Utilities.SetWarning(Utilities.Warning.AxisNotReadyForCoupling);
		}
	}

	public static bool IsAxesCouplingActive() =>
		
		// check if axis are coupled
		(Register.Axes_0.Signals.General.AxisState == AxisState.DirectCoupledMotion &&
			Register.Axes_1.Signals.General.AxisState == AxisState.DirectCoupledMotion);

	public static void Move(AxisId axisId, float xNew, float vNew, PathPlannerCommand command) {
		switch (axisId) {
			case AxisId.Phi0:
				Register.Axes_0.Commands.PathPlanner.Vnew = vNew;
				Register.Axes_0.Commands.PathPlanner.Xnew = xNew;
				Register.Axes_0.Commands.PathPlanner.Command = command;
				break;

			case AxisId.Phi1:
				Register.Axes_1.Commands.PathPlanner.Vnew = vNew;
				Register.Axes_1.Commands.PathPlanner.Xnew = xNew;
				Register.Axes_1.Commands.PathPlanner.Command = command;
				break;

			default:
				Utilities.SetWarning(Utilities.Warning.InternalUndefinedCase);
				break;
		}
	}

	public static void Stop(AxisId axisId) {
		switch (axisId) {
			case AxisId.Phi0:
				Register.Axes_0.Commands.PathPlanner.Command = PathPlannerCommand.Stop;
				break;

			case AxisId.Phi1:
				Register.Axes_1.Commands.PathPlanner.Command = PathPlannerCommand.Stop;
				break;

			default:
				Utilities.SetWarning(Utilities.Warning.InternalUndefinedCase);
				break;
		}
	}

	public static float GetLimitedState(float state, float Limit) {
		if (state > Limit) {
			state = Limit;
		} else if (state < -Limit) {
			state = -Limit;
		}
		return state;
	}

	public static bool IsPathPlannerDone(AxisId axisId) {
		bool isDone = false;
		switch (axisId) {
			case AxisId.Phi0:
				isDone = Register.Axes_0.Signals.PathPlanner.Done;
				break;

			case AxisId.Phi1:
				isDone = Register.Axes_1.Signals.PathPlanner.Done;
				break;

			default:
				Utilities.SetWarning(Utilities.Warning.InternalUndefinedCase);
				break;
		}
		return isDone;
	}
}

