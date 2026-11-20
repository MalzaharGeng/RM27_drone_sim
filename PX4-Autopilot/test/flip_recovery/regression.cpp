#include <flip_recovery/FlipPlanner.hpp>
#include <flip_recovery/DirectionSequencer.hpp>
#include <cstdio>
#include <cstring>
#include <initializer_list>

using namespace flip_recovery;
static constexpr float radians(float degrees) { return degrees * 0.017453292519943295f; }
static int failures = 0;
static int checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); } } while (0)

static Layout layout()
{
	Layout l; l.valid = true;
	l.px[0] = 1; l.py[0] = 1; l.px[1] = -1; l.py[1] = -1;
	l.px[2] = 1; l.py[2] = -1; l.px[3] = -1; l.py[3] = 1;
	return l;
}

static Input input(float roll, uint64_t now)
{
	Input in; in.now = now; in.enabled = in.safe = in.mode_active = true;
	const matrix::Quatf q{matrix::Eulerf{radians(roll), 0, 0}};
	for (int i = 0; i < 4; ++i) { in.q[i] = q(i); }
	return in;
}

static void plannerTests()
{
	Settings s; const Layout l = layout();
	uint8_t low = 0, high = 0;
	// Positive roll lowers the RIGHT side in NED: motors 1 and 4.
	CHECK(FlipPlanner::lowestPair(l, matrix::Dcmf{matrix::Eulerf{radians(80.f), 0, 0}}, low, high));
	CHECK(low == 0x09 && high == 0x06);
	CHECK(FlipPlanner::lowestPair(l, matrix::Dcmf{matrix::Eulerf{0, radians(80.f), 0}}, low, high));
	CHECK(low == 0x0a && high == 0x05); // positive pitch lowers the rear

	FlipPlanner p; p.update(input(120, 1000000), s, l);
	auto out = p.update(input(150, 1820000), s, l);
	CHECK(p.state() != FlipPlanner::State::Failed); // valid flip-over progress
	CHECK(out.rev_mask == 0x0f);

	FlipPlanner coast; coast.update(input(80, 1000000), s, l);
	out = coast.update(input(50, 1100000), s, l);
	CHECK(coast.state() == FlipPlanner::State::Coast);
	CHECK(!out.fwd_mask && !out.rev_mask);
	coast.update(input(10, 1200000), s, l);
	CHECK(coast.state() == FlipPlanner::State::Coast);
	coast.update(input(10, 2220000), s, l);
	CHECK(coast.state() == FlipPlanner::State::Done);

	FlipPlanner bounced; bounced.update(input(80, 1000000), s, l);
	bounced.update(input(50, 1100000), s, l);
	bounced.update(input(100, 4200000), s, l);
	CHECK(bounced.state() == FlipPlanner::State::Failed);

	FlipPlanner invalid; invalid.update(input(80, 1000000), s, l);
	Input bad = input(80, 1100000); bad.q[0] = NAN;
	out = invalid.update(bad, s, l);
	CHECK(invalid.state() == FlipPlanner::State::Failed);
	CHECK(out.reason_code == ReasonSafetyLost);

	FlipPlanner exited; exited.update(input(80, 1000000), s, l);
	Input off = input(80, 1100000); off.mode_active = false;
	out = exited.update(off, s, l);
	CHECK(out.reason_code == ReasonModeExited);
	out = exited.update(off, s, l);
	CHECK(out.reason_code == ReasonModeExited);
	out = exited.update(input(80, 1200000), s, l);
	CHECK(out.reason_code == ReasonNone);
	// Both roll signs and both strategies, including the acos fold at 180 deg.
	for (float sign : {-1.f, 1.f}) {
		FlipPlanner over; over.update(input(sign * 120, 1000000), s, l);
		for (int step = 1; step <= 7; ++step) {
			out = over.update(input(sign * (120 + step * 10), 1000000 + step * 120000), s, l);
			CHECK(over.state() != FlipPlanner::State::Failed);
		}
		Settings back_s = s; back_s.strategy = 1;
		FlipPlanner back; back.update(input(sign * 120, 1000000), back_s, l);
		out = back.update(input(sign * 100, 1820000), back_s, l);
		CHECK(back.state() != FlipPlanner::State::Failed);
		FlipPlanner wrong; wrong.update(input(sign * 120, 1000000), s, l);
		out = wrong.update(input(sign * 100, 1820000), s, l);
		CHECK(out.reason_code == ReasonNoProgress);
	}
	FlipPlanner stuck; stuck.update(input(120, 1000000), s, l);
	out = stuck.update(input(120, 1820000), s, l);
	CHECK(out.reason_code == ReasonNoProgress);
	Input unsafe_in = input(80, 1100000); unsafe_in.safe = false;
	FlipPlanner unsafe_p; unsafe_p.update(input(80, 1000000), s, l);
	out = unsafe_p.update(unsafe_in, s, l); CHECK(out.reason_code == ReasonSafetyLost);
	CHECK(!out.active && !out.rev_mask && !out.fwd_mask);
	FlipPlanner coast_exit; coast_exit.update(input(80, 1000000), s, l);
	coast_exit.update(input(50, 1100000), s, l);
	Input leave = input(10, 1200000); leave.mode_active = false;
	out = coast_exit.update(leave, s, l); CHECK(out.reason_code == ReasonModeExited);
	Layout invalid_layout = l; invalid_layout.px[0] = NAN;
	CHECK(!FlipPlanner::lowestPair(invalid_layout, matrix::Dcmf{}, low, high));

}

static void sequencerTests()
{
	DirectionSequencer q;
	q.update(1000000, true, true, 0x05);
	CHECK(q.ownsOutputs() && !q.driving() && !q.command());
	q.update(1299999, true, true, 0x05); CHECK(!q.command());
	q.update(1300000, true, true, 0x05);
	CHECK(q.command() == 20 && q.commandMask() == 0x0a);
	for (int i = 0; i < 9; ++i) { q.commandSent(1300000 + i * 1000); }
	CHECK(q.command() == 20);
	q.commandSent(1309000); CHECK(q.command() == 21 && q.commandMask() == 0x05);
	for (int i = 0; i < 10; ++i) { q.commandSent(1310000 + i * 1000); }
	CHECK(!q.command() && !q.driving());
	q.update(1418999, true, true, 0x05); CHECK(!q.driving());
	q.update(1419000, true, true, 0x05); CHECK(q.driving());
	float request[4] = {-0.2f, 0.1f, -0.2f, 0.1f}, magnitude[4] {};
	q.throttle(1419000, request, 0.5f, magnitude); CHECK(magnitude[0] == 0.f);
	q.throttle(1439000, request, 0.5f, magnitude); CHECK(fabsf(magnitude[0] - 0.01f) < 1e-5f);
	q.update(1440000, false, true, 0); CHECK(!q.driving() && q.restoring());
	q.throttle(1440000, request, 0.5f, magnitude); CHECK(magnitude[0] == 0.f);
	q.update(1740000, false, true, 0); CHECK(q.command() == 20 && q.commandMask() == 0x0f);
	for (int i = 0; i < 10; ++i) { q.commandSent(1740000 + i * 1000); }
	q.update(1849000, false, true, 0); CHECK(!q.ownsOutputs());

	DirectionSequencer unsafe;
	unsafe.update(1000000, true, true, 0x0f);
	unsafe.update(1300000, true, true, 0x0f); CHECK(unsafe.command() == 21);
	unsafe.commandSent(1300000);
	unsafe.update(1310000, true, false, 0x0f);
	CHECK(unsafe.ownsOutputs() && !unsafe.command() && !unsafe.driving());
	unsafe.update(2000000, false, true, 0);
	CHECK(unsafe.command() == 20 && unsafe.commandMask() == 0x0f);

	int ids[6] = {2, -1, 0, 3, -1, 1}; unsigned channels[4] {};
	CHECK(DirectionSequencer::mapMotors(ids, 6, channels));
	CHECK(channels[0] == 2 && channels[1] == 5 && channels[2] == 0 && channels[3] == 3);
	ids[1] = 0; CHECK(!DirectionSequencer::mapMotors(ids, 6, channels));
	DirectionSequencer interrupted;
	interrupted.update(1000000, true, true, 0x0f);
	interrupted.update(1300000, true, true, 0x0f);
	CHECK(interrupted.command() == 21);
	interrupted.commandSent(1300000);
	interrupted.update(1301000, false, true, 0);
	CHECK(interrupted.restoring() && !interrupted.command());
	interrupted.update(1601000, true, true, 0x0f);
	CHECK(interrupted.command() == 20 && interrupted.commandMask() == 0x0f);
	// New DRIVE does not cancel restoration; merely calling update cannot
	// consume a command burst, even after a long scheduler pause.
	interrupted.update(9000000, true, true, 0x0f);
	CHECK(interrupted.command() == 20 && !interrupted.driving());
	for (int i = 0; i < 10; ++i) { interrupted.commandSent(9000000 + i * 1000); }
	interrupted.update(9109000, true, true, 0x0f);
	CHECK(!interrupted.ownsOutputs());

}

int main()
{
	plannerTests(); sequencerTests();
	std::printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
