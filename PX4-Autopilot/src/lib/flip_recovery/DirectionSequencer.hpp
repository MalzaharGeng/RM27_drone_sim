#pragma once

#include <cmath>
#include <cstdint>

namespace flip_recovery
{

// No ESC acknowledgement is available: completion means ten frames were
// submitted, followed by a guard interval, NOT confirmed mechanical reversal.
class DirectionSequencer
{
public:

	enum class Phase : uint8_t { Idle, Stop, Normal, Reverse, Wait, Drive };

	static constexpr uint64_t STOP_US = 300000;
	static constexpr uint64_t WAIT_US = 100000;
	static constexpr uint8_t REPETITIONS = 10;

	void update(uint64_t now, bool drive, bool safe, uint8_t reverse_mask)
	{
		reverse_mask &= 0x0f;
		// An unsafe vehicle may receive STOP only. Remember that the ESCs may
		// still be reversed and restore them after safety has returned.
		if (!safe) {
			if (ownsOutputs()) {
				_restoring = true;
				_target = 0;
				_phase = Phase::Stop;
				_since = now;
			}

			_safe = false;
			return;
		}

		_safe = true;

		if (_phase == Phase::Idle) {
			if (drive) {
				begin(now, reverse_mask, false);
			}

			return;
		}

		// Finish an already started restoration before accepting a new drive.
		if (!_restoring && (!drive || reverse_mask != _target)) {
			begin(now, drive ? reverse_mask : 0, !drive);
		}

		if (_phase == Phase::Stop && now - _since >= STOP_US) {
			_remaining = REPETITIONS;
			_phase = (_target == 0x0f) ? Phase::Reverse : Phase::Normal;
		}

		if (_phase == Phase::Wait && now - _since >= WAIT_US) {
			_phase = _restoring ? Phase::Idle : Phase::Drive;
		}
	}

	// Called ONLY after a complete command frame has been submitted for all
	// requested motors. Scheduling iterations alone must not consume frames.
	void commandSent(uint64_t now)
	{
		if (!_safe || !command() || _remaining == 0) {
			return;
		}

		if (--_remaining == 0) {
			if (_phase == Phase::Normal && _target != 0) {
				_phase = Phase::Reverse;
				_remaining = REPETITIONS;

			} else {
				_phase = Phase::Wait;
				_since = now;
			}
		}
	}

	bool ownsOutputs() const { return _phase != Phase::Idle; }
	bool driving() const { return _safe && _phase == Phase::Drive; }
	Phase phase() const { return _phase; }
	bool restoring() const { return _restoring; }

	// Ramp actual magnitude AFTER switching, rather than jumping straight to
	// the planner target that ramped while the motors were held stopped.
	void throttle(uint64_t now, const float requested[4], float slew_rate, float output[4])
	{
		const float dt = _last_throttle != 0 && now >= _last_throttle
				 ? fminf(0.05f, (now - _last_throttle) * 1e-6f) : 0.f;
		_last_throttle = now;
		for (unsigned i = 0; i < 4; ++i) {
			_throttle[i] = driving() ? fminf(fabsf(requested[i]), _throttle[i] + slew_rate * dt) : 0.f;
			output[i] = _throttle[i];
		}
	}
	float throttle(unsigned i) const { return _throttle[i]; }
	uint8_t command() const { return !_safe ? 0 : (_phase == Phase::Normal ? 20 : (_phase == Phase::Reverse ? 21 : 0)); }
	uint8_t commandMask() const { return _phase == Phase::Normal ? static_cast<uint8_t>(0x0f & ~_target) : _target; }

	// Logical motors must each occur once. The caller supplies output-function
	// assignments; all four must belong to this same output backend.
	static bool mapMotors(const int *motor_ids, unsigned count, unsigned channels[4])
	{
		uint8_t found = 0;

		for (unsigned i = 0; i < count; ++i) {
			const int id = motor_ids[i];

			if (id < 0 || id >= 4) { continue; }

			if (found & (1u << id)) { return false; }

			found |= 1u << id;
			channels[id] = i;
		}

		return found == 0x0f;
	}

private:

	void begin(uint64_t now, uint8_t target, bool restoring)
	{
		_phase = Phase::Stop;
		_since = now;
		_target = target;
		_restoring = restoring;
	}

	Phase _phase{Phase::Idle};
	uint64_t _since{0};
	uint8_t _target{0};
	uint8_t _remaining{0};
	bool _restoring{false};
	bool _safe{false};
	uint64_t _last_throttle{0};
	float _throttle[4] {};
};

} // namespace flip_recovery
