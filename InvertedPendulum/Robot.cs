// Copyright © 2017 Triamec Motion AG

using Triamec.Tama.Vmid5;
using Triamec.Tama.Rlid19;

static class Robot {
	const float B = Parameter.LengthBeam1;
	const float C = Parameter.LengthBeam2;
	const float DHalf = Parameter.DistanceMotorShaft / 2.0f;
	const float B2pC2 = B * B + C * C;
	const float Inv_2bc = 1 / (2 * B * C);
	const float BDivC = B / C;
	//const float BDivC2 = BDivC * BDivC;
	const float R2Max = (B + C) * (B + C);
	const float R2Lim = 0.98f * R2Max;

	/// <summary>Beam angle of motor 0</summary>
	public static Utilities.MoveState phi0;

	/// <summary>Beam angle of motor 1</summary>
	public static Utilities.MoveState phi1;

	/// <summary>Position x</summary>
	public static float xPos;

	/// <summary>Position y</summary>
	public static float yPos;

	/// <summary>Angle of platform beam relative to x axis</summary>
	public static float xi;

	/// <summary>Position origin x</summary>
	public static float xPosHome;

	/// <summary>Position origin y</summary>
	public static float yPosHome;

	public static bool isOutOfRange;

	public static void ForwardKinematics(float phi0, float phi1) {

		// location of first joint
		float x01 = Math.Cos(phi0) * B;
		float y01 = Math.Sin(phi0) * B - DHalf;
		float x11 = Math.Cos(phi1) * B;
		float y11 = Math.Sin(phi1) * B + DHalf;

		// center location between first joints
		float xc = (x01 + x11) / 2;
		float yc = (y01 + y11) / 2;
		
		// distance between first joints
		float d1 = Math.Sqrt((x11 - x01) * (x11 - x01) + (y11 - y01) * (y11 - y01));
		
		// distance(xc, yc) to(x, y)
		float dc = Math.Sqrt(C * C - (d1 * d1 / 4.0f));

		// effector coordinates
		xPos = xc + dc / d1 * (y11 - y01);
		yPos = yc - dc / d1 * (x11 - x01);
		
		// angle of platform beam relative to x axis
		xi = Math.Atan2(yPos - y01, xPos - x01);
	}

	public static void SetHomePosition() {
		ForwardKinematics(
			Register.Axes_0.Signals.PositionController.Encoders_0.PositionFloat,
			Register.Axes_1.Signals.PositionController.Encoders_0.PositionFloat
			);
		xPosHome = xPos;
		yPosHome = yPos;
		isOutOfRange = false;
	}

	public static void BackwardKinematcs(Utilities.MoveState x, Utilities.MoveState y) {
		xPos = x.pos;
		yPos = y.pos;
		isOutOfRange = false;
		BwArmKin(x.pos, /*x.vel, x.acc, */ y.pos + DHalf, /* y.vel, y.acc, */ AxisHandler.AxisId.Phi0);
		BwArmKin(x.pos, /*x.vel, x.acc, */ y.pos - DHalf, /* y.vel, y.acc, */ AxisHandler.AxisId.Phi1);
	}

	static void BwArmKin(float x, /*float dx, float ddx, */ float y, /* float dy, float ddy, */AxisHandler.AxisId axisId) {
		
		// distance motors to effector squared
		float aSq = x * x + y * y;
		//       float daSq = 2 * (x * dx + y * dy);
		//       float ddaSq = 2 * (dx * dx + x * ddx + dy * dy + y * ddy);
		//       float inv_aSq = 1 / aSq;

		// angle to end effector
		float psi = Math.Atan2(y, x);
		//       float dpsi = (x * dy - dx * y) * inv_aSq;
		//       float ddpsi = ((dx * dy + x * ddy - ddx * y - dx * dy) - dpsi * daSq) * inv_aSq;

		// Angle between b and c
		float cos_alpha = (-aSq + B2pC2) * Inv_2bc;
		float alpha = Math.Acos(cos_alpha);
		float sin_alpha = Math.Sin(alpha);
		//       float inv_sin_alpha = 1 / sin_alpha;
		//       float dalpha = daSq * inv_2bc * inv_sin_alpha;
		//       float ddalpha = inv_sin_alpha * (ddaSq * inv_2bc - dalpha * dalpha * cos_alpha);

		// angle between a and b
		//       float num = BDivC * cos_alpha - 1.0f;
		//       float inv_den = 1 / (1 - 2 * BDivC * cos_alpha + BDivC2);
		float gamma = Math.Atan2(sin_alpha, BDivC - cos_alpha);
		//       float dgamma = dalpha * num * inv_den;
		//       float ddgamma = (ddalpha * num + dalpha * dalpha * bdivc * sin_alpha * (1 - bdivc2) * inv_den) * inv_den;

		switch (axisId) {
			case AxisHandler.AxisId.Phi0:
				phi0.pos = psi - gamma;
				phi0.vel = 0.0f; // AxisHandler.getLimitedState(dpsi - dgamma, Register.Axes_0.Parameters.PathPlanner.VelocityMaximum);
				phi0.acc = 0.0f; // AxisHandler.getLimitedState(ddpsi - ddgamma, Register.Axes_0.Parameters.PathPlanner.AccelerationMaximum);
				xi = phi0.pos + Math.PI - alpha;
				break;
			case AxisHandler.AxisId.Phi1:
				phi1.pos = psi + gamma;
				phi1.vel = 0.0f; // AxisHandler.getLimitedState(dpsi + dgamma, Register.Axes_1.Parameters.PathPlanner.VelocityMaximum);
				phi1.acc = 0.0f; // AxisHandler.getLimitedState(ddpsi + ddgamma, Register.Axes_1.Parameters.PathPlanner.AccelerationMaximum);
				break;
		}

		// check if within rang
		isOutOfRange |= (aSq > R2Lim);
	}
}
