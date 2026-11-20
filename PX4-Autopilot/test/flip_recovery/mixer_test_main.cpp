// Minimal functional-test entry point using the production uORB and parameter
// libraries. No PX4 vehicle process, estimator or simulator is started.
#include <gtest/gtest.h>
#include <uORB/Subscription.hpp>
#include <lib/parameters/param.h>

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	uORB::Manager::initialize();
	param_init();
	return RUN_ALL_TESTS();
}
