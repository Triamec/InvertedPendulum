// Copyright © 2017 Triamec Motion AG

using System;
using Triamec.Tam.Registers;
using Triamec.Tama.Rlid19;

static class Utilities {
	[Flags]
	public enum Warning {
		DeviceOrAxisError = 1,
		AxesAreNotEnabled = 2,
		AxisNotReadyForCoupling = 4,
		AxesAreNotCoupled = 8,
		SwLimitViolation = 16,
		InvalidBarType = 32,
		ResetFailed = 64,
		BeamDetectionUndervoltage = 128,
		InternalUndefinedCase = 256
	}

	public struct MoveState {
		public float pos;
		public float vel;
		public float acc;
		public float jrk;
	};
	

	public static void SetWarning(Warning warning) =>
		TamaRegisters.WarningBitField_VI01 |= (int)warning;

	public static void ResetWarnings() => TamaRegisters.WarningBitField_VI01 = 0;

	public static bool IsPositionOutOfLimit() {

		// check sw limits and actual move direction
		float deltaPos = Register.Axes_1.Signals.PathPlanner.PositionFloat - Register.Axes_0.Signals.PathPlanner.PositionFloat;

		if (Register.Axes_0.Signals.PathPlanner.PositionFloat < Parameter.Phi0PosMin ||
			Register.Axes_0.Signals.PathPlanner.PositionFloat > Parameter.Phi0PosMax ||
			Register.Axes_1.Signals.PathPlanner.PositionFloat < Parameter.Phi1PosMin ||
			Register.Axes_1.Signals.PathPlanner.PositionFloat > Parameter.Phi1PosMax ||
			deltaPos < Parameter.PhiDeltaPosMin ||
			deltaPos > Parameter.PhiDeltaPosMax) {
			AxisHandler.Stop(AxisHandler.AxisId.Phi0);
			AxisHandler.Stop(AxisHandler.AxisId.Phi1);
			SetWarning(Warning.SwLimitViolation);
			return true;
		} else {
			return false;
		}
	}

	public static bool IsDeviceOrAxisErrorPending() {

		// check for axis or drive error
		if (Register.General.Signals.DriveError != DeviceErrorIdentification.None ||
			Register.Axes_0.Signals.General.AxisError != AxisErrorIdentification.None ||
			Register.Axes_1.Signals.General.AxisError != AxisErrorIdentification.None) {

			SetWarning(Warning.DeviceOrAxisError);
			return true;
		} else {
			return false;
		}
	}
}
