// Copyright © 2017 Triamec Motion AG

using Triamec.Tama.Rlid19;

/// <summary>Spiral path planner.</summary>
static class Controller {
	enum ControllerState {
		Idle = 0,
		Init,
		Running,
		Moving,
	}

	/// <summary>actual state (x)</summary>	
	public static float[] xAct;

	/// <summary>actual state (y)</summary>	
	public static float[] yAct;

	/// <summary>States of x</summary>
	static Utilities.MoveState _xOutputState;

	/// <summary>States of y</summary>
	static Utilities.MoveState _yOutputState;

	/// <summary>Sampling time, in seconds</summary>
	const float TS = Parameter.SamplingTime;

	/// <summary>Sampling time squared, in square seconds.</summary>
	const float TS2 = TS * TS; // s^2

	/// <summary>
	/// Static constructor replacement.
	/// </summary>
	public static void InitController() {
		xAct = new float[5];
		yAct = new float[5];
	}

	public static void Reset() {
		Robot.SetHomePosition();

		_xOutputState.pos = Robot.xPos;
		_xOutputState.vel = 0.0f;
		_xOutputState.acc = 0.0f;
		_yOutputState.pos = Robot.yPos;
		_yOutputState.vel = 0.0f;
		_yOutputState.acc = 0.0f;

		xAct[0] = 0.0f;
		yAct[0] = 0.0f;
	}

	/// <summary>
	/// Calibrate offset
	/// </summary>
	public static void CalibOffset() {
		Beam.CalibOffset();

		// reset integrator
		xAct[0] = 0.0f;
		yAct[0] = 0.0f;
	}

	public static void DoControllerStep() {
		// transform to controller normal form
		xAct[4] = -Parameter.g * Beam.xIncVel;
		xAct[3] = -Parameter.g * Beam.xIncPos;
		xAct[2] = _xOutputState.vel - Beam.xIncVel * Beam.l0;
		xAct[1] = _xOutputState.pos - Beam.xIncPos * Beam.l0;
		// integrator x
		xAct[0] = xAct[0] + (PendulumPathPlanner.xState.pos - xAct[1]) * Beam.K[0];
		xAct[0] = (xAct[0] > Beam.IntegratorLimit) ? Beam.IntegratorLimit :
				 ((xAct[0] < -Beam.IntegratorLimit) ? -Beam.IntegratorLimit : xAct[0]);

		yAct[4] = -Parameter.g * Beam.yIncVel;
		yAct[3] = -Parameter.g * Beam.yIncPos;
		yAct[2] = _yOutputState.vel - Beam.yIncVel * Beam.l0;
		yAct[1] = _yOutputState.pos - Beam.yIncPos * Beam.l0;
		// integrator y
		yAct[0] = yAct[0] + (PendulumPathPlanner.yState.pos - yAct[1]) * Beam.K[0];
		yAct[0] = (yAct[0] > Beam.IntegratorLimit) ? Beam.IntegratorLimit :
				 ((yAct[0] < -Beam.IntegratorLimit) ? -Beam.IntegratorLimit : yAct[0]);

		// do controller calculation
		_xOutputState.acc = (xAct[0]                                         // x0: integral of pos error
			+ Beam.K[1] * (PendulumPathPlanner.xState.pos - xAct[1])        // x1: pos
			+ Beam.K[2] * (PendulumPathPlanner.xState.vel - xAct[2])        // x2: dpos
			+ Beam.K[3] * (PendulumPathPlanner.xState.acc - xAct[3])        // x3: ddpos
			+ Beam.K[4] * (PendulumPathPlanner.xState.jrk - xAct[4]));      // x4: dddpos

		_yOutputState.acc = (yAct[0]                                         // y0: integral of pos error
			+ Beam.K[1] * (PendulumPathPlanner.yState.pos - yAct[1])        // y1: pos
			+ Beam.K[2] * (PendulumPathPlanner.yState.vel - yAct[2])        // y2: dpos
			+ Beam.K[3] * (PendulumPathPlanner.yState.acc - yAct[3])        // y3: ddpos
			+ Beam.K[4] * (PendulumPathPlanner.yState.jrk - yAct[4]));      // y4: dddpos

		// truncate acceleration
		_xOutputState.acc = _xOutputState.acc > Parameter.MaxAccOut ? Parameter.MaxAccOut : (_xOutputState.acc < -Parameter.MaxAccOut ? -Parameter.MaxAccOut : _xOutputState.acc);
		_yOutputState.acc = _yOutputState.acc > Parameter.MaxAccOut ? Parameter.MaxAccOut : (_yOutputState.acc < -Parameter.MaxAccOut ? -Parameter.MaxAccOut : _yOutputState.acc);

		// integrate output signal acc -> vel -> pos 
		_xOutputState.pos = _xOutputState.pos + _xOutputState.vel * TS + 0.5f * _xOutputState.acc * TS2;
		_xOutputState.vel += _xOutputState.acc * TS;
		_yOutputState.pos = _yOutputState.pos + _yOutputState.vel * TS + 0.5f * _yOutputState.acc * TS2;
		_yOutputState.vel += _yOutputState.acc * TS;

		TamaRegisters.XPosCenter_VF10 = xAct[1];
		TamaRegisters.YPosCenter_VF11 = yAct[1];
		TamaRegisters.XAccTable_VF16 = _xOutputState.acc;
		TamaRegisters.YAccTable_VF17 = _yOutputState.acc;
	}

	public static void DoDynamicMoveStep() {

		// caclulate backward kinematics
		Robot.BackwardKinematcs(PendulumPathPlanner.xState, PendulumPathPlanner.yState);
		
		// set output signal phi0
		Register.Axes_0.Commands.PathPlanner.StreamX = Robot.phi0.pos;
		Register.Axes_0.Commands.PathPlanner.StreamV = Robot.phi0.vel;
		Register.Axes_0.Commands.PathPlanner.StreamA = Robot.phi0.acc;
		
		// set output signal phi1
		Register.Axes_1.Commands.PathPlanner.StreamX = Robot.phi1.pos;
		Register.Axes_1.Commands.PathPlanner.StreamV = Robot.phi1.vel;
		Register.Axes_1.Commands.PathPlanner.StreamA = Robot.phi1.acc;
	}

	public static void DoBackwardKinematics() {

		// calculate backward kinematics
		Robot.BackwardKinematcs(_xOutputState, _yOutputState);
		
		// set output signal phi0
		Register.Axes_0.Commands.PathPlanner.StreamX = Robot.phi0.pos;
		Register.Axes_0.Commands.PathPlanner.StreamV = Robot.phi0.vel;
		Register.Axes_0.Commands.PathPlanner.StreamA = Robot.phi0.acc;
		
		// set output signal phi1
		Register.Axes_1.Commands.PathPlanner.StreamX = Robot.phi1.pos;
		Register.Axes_1.Commands.PathPlanner.StreamV = Robot.phi1.vel;
		Register.Axes_1.Commands.PathPlanner.StreamA = Robot.phi1.acc;
	}
}

