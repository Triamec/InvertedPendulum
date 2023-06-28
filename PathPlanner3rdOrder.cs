// Copyright © 2017 Triamec Motion AG

using Triamec.Tama.Vmid5;

/// <remarks>The move can be stopped at any time. </remarks>
static class PathPlanner3rdOrder {

	/// <summary>Sampling frequency [1/s]</summary>
	const float Fs = Parameter.SamplingFrequency;

	/// <summary>Sampling frequency squared [1/s^2]</summary>
	const float Fs_p2 = Fs * Fs;
								
	/// <summary>Sampling frequency to its third potency [1/s^3]</summary>
	const float Fs_p3 = Fs_p2 * Fs;

	/// <summary>Sampling time [s]</summary>
	const float Ts = 1.0f / Fs;

	/// <summary>Sampling time squared [s^2]</summary>
	const float Ts_p2 = Ts * Ts;

	/// <summary>Sampling time to its third potency [s^3]</summary>
	const float Ts_p3 = Ts_p2 * Ts;

	const float OneSixth = 1.0f / 6.0f;
	const float OneThird = 1.0f / 3.0f;

	/// <summary>Actual jerk</summary>
	static float _jrkAct;

	/// <summary>Number jerk segment counts</summary>
	static int _nJrk;

	/// <summary>Number acceleration segment counts</summary>
	static int _nAcc;

	/// <summary>Number velocity segment counts</summary>
	static int _nVel;

	/// <summary>End position</summary>
	static float _posEnd;

	static float _jrkMax;

	/// <summary>Actual acceleration</summary>
	static float _accAct;

	/// <summary>>Actual velocity</summary>
	static float _velAct;

	/// <summary>Jerk segment counter</summary>
	static int _cntJrkPos;

	/// <summary>Jerk segment counter</summary>
	static int _cntJrkNeg;

	/// <summary>Number acceleration segment counts</summary>
	static int _cntAcc;

	/// <summary>Number velocity segment counts</summary>
	static int _cntVel;

	/// <summary>stopping</summary>
	static bool _stopMove;

	static bool _isVelMove;

	/// <summary>
	/// Gets the position
	/// </summary>
	public static float Pos { get; private set; }

	/// <summary>
	/// Gets the velocity.
	/// </summary>
	public static float Vel => _velAct * Fs;

    /// <summary>
	/// Gets the acceleration
	/// </summary>
	public static float Acc => _accAct * Fs_p2;

	/// <summary>
	/// Gets the jerk.
	/// </summary>
	public static float Jrk => _jrkAct * Fs_p3;

	/// <param name="run">If <see langword="false"/>, the move decelerates until velocity is zero.</param>
	/// <remarks>
	/// This method updates the current path planner state and sets 
	/// isDone to true if the move is done.
	/// </remarks>
	public static bool DoMoveStep(bool run) {
		_stopMove = _stopMove || !run;
		bool isDone = false;

		// update states
		Pos = Pos + _velAct + 0.5f * _accAct + OneSixth * _jrkAct;
		_velAct = _velAct + _accAct + 0.5f * _jrkAct;
		_accAct += _jrkAct;

		// check if run is requested
		if (!_stopMove) {
			
			// accelerate to desired velocity
			if (_cntJrkPos < _nJrk) {
				
				// increase acceleration
				_cntJrkPos++;
				_jrkAct = _jrkMax;
			} else if (_cntAcc < _nAcc) {
				
				// constant acceleration
				_cntAcc++;
				_jrkAct = 0.0f;
			} else if (_cntJrkNeg < _cntJrkPos) {
				
				// reduce acceleration
				_cntJrkNeg++;
				_jrkAct = -_jrkMax;
			} else if (_cntVel < _nVel || _isVelMove) {
				
				// velocity reached
				_cntVel++;
				_jrkAct = 0.0f;
			} else {
				_stopMove = true;
			}
		}

		if (_stopMove) {
			
			// stop is requested
			if (_cntJrkNeg < 2 * _cntJrkPos) {
				
				// increase negative acceleration
				_cntJrkNeg++;
				_jrkAct = -_jrkMax;
			} else if (_cntAcc > 0) {
				
				// constant negative acceleration
				_cntAcc--;
				_jrkAct = 0.0f;
			} else if (_cntJrkPos > 0) {

				// reduce negative acceleration 
				_jrkAct = _jrkMax;
				_cntJrkPos--;
			} else {
				_cntJrkNeg = 0;
				isDone = true;
				_jrkAct = 0.0f;
				_accAct = 0.0f;
				_velAct = 0.0f;
				//if (!_isVelMove) {
					//posAct = posEnd;
				//}
			}
		}
		return isDone;
	}

	static void Reset() {
		_cntJrkPos = 1; // as first jerk is already set at init
		_cntAcc = 0;
		_cntJrkNeg = 0;
		_cntVel = 0;

		_jrkAct = 0.0f;
		_accAct = 0.0f;
		_velAct = 0.0f;
		Pos = 0.0f;

		_stopMove = false;
	}

	/// <summary>
	/// Set the parameters of the position move.
	/// </summary>
	public static void InitPositionMove(float pos, float vel, float acc, float jrk) {
		_isVelMove = false;

		// if one parameter is zero, set all to zero
		if ((pos == 0.0f) || (vel == 0.0f) || (acc == 0.0f) || (jrk == 0.0f)) {
			_nVel = 0;
			_nAcc = 0;
			_nJrk = 0;
			_jrkMax = 0.0f;
			return;
		}

		// store velocity with direction
		_posEnd = pos;

		// just use positive values and adjust time scaling -> t' = t/ts
		float invJerk = 1.0f / ((jrk >= 0 ? jrk : -jrk) * Ts_p3);
		pos = Math.Fabs(pos) * invJerk;
		vel = Math.Fabs(vel) * invJerk * Ts;
		acc = Math.Fabs(acc) * invJerk * Ts_p2;

		// calculation of nJrk based on the different possible constraints
		// to not violate a constraint we take the smallest nJrk
		float n = acc;                                 // acceleration constraint
		float nTmp = Math.Sqrt(vel);                   // velocity constraint
		n = n < nTmp ? n : nTmp;
		nTmp = Math.Pow(0.5f * pos, OneThird);           // distance
		n = n < nTmp ? n : nTmp;
		_nJrk = n > 1 ? (int)n : 1; //floor;
		float nJrkF = (float)_nJrk;

		// calculation of nAcc based on the different possible constraints
		// to not violate a constraint we take the smallest nAcc
		n = vel / nJrkF - nJrkF;          // velocity constraint
		nTmp = (-1.5f * nJrkF + Math.Sqrt(0.25f * nJrkF * nJrkF + pos / nJrkF));  // distance
		n = n < nTmp ? n : nTmp;
		n = n > 0 ? n : 0;
		_nAcc = (int)n; // floor
		float nAccF = (float)_nAcc;

		// calculation of nVel based on the different possible constraints
		n = pos / ((nJrkF + nAccF) * nJrkF) - (2.0f * nJrkF + nAccF);
		n = n > 0 ? n : 0;
		// ceil calculation
		_nVel = (int)n; _nVel = _nVel < n ? _nVel + 1 : _nVel;
		float nVelF = (float)_nVel;

		// recalc jerk to reach end position exactly
		_jrkMax = _posEnd / ((2.0f * nJrkF + nAccF + nVelF) * (nJrkF + nAccF) * nJrkF);
		Reset();
		_jrkAct = _jrkMax;
	}

	/// <summary>
	/// Set the parameters of the velocity move.
	/// </summary>
	public static void InitVelocityMove(float vel, float acc, float jrk) {
		_isVelMove = true;

		// if one parameter is zero, set all to zero
		if ((vel == 0.0f) || (acc == 0.0f) || (jrk == 0.0f)) {
			_nAcc = 0;
			_nJrk = 0;
			_jrkMax = 0.0f;
			return;
		}

		// store velocity with direction
		_nVel = 0;

		// just use positive values and adjust time scaling -> t' = t/ts
		float invJerk = 1.0f / ((jrk >= 0 ? jrk : -jrk) * Ts_p3);
		vel = Math.Fabs(vel) * invJerk * Ts;
		acc = Math.Fabs(acc) * invJerk * Ts_p2;

		// calculation of nJrk based on the different possible constraints
		// to not violate a constraint we take the smallest nJrk
		float n = acc;                                 // acceleration constraint
		float nTmp = Math.Sqrt(vel);                      // velocity constraint
		n = n < nTmp ? n : nTmp;
		_nJrk = n > 1 ? (int)n : 1; //floor;
		float nJrkF = (float)_nJrk;

		// calculation of nAcc based on the different possible constraints
		// to not violate a constraint we take the smallest nAcc
		n = vel / nJrkF - _nJrk;          // velocity constraint
										 // ceil calculation
		_nAcc = (int)n; _nAcc = _nAcc < n ? _nAcc + 1 : _nAcc;
		float nAccF = (float)_nAcc;

		// recalc jerk to reach end position exactly
		_jrkMax = vel / (nJrkF * (nJrkF + nAccF));
		Reset();
		_jrkAct = _jrkMax;
	}
}
