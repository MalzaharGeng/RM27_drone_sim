px4_add_module(
	MODULE modules__flip_recovery
	MAIN flip_recovery
	STACK_MAIN 2048
	SRCS
		FlipRecovery.cpp
		FlipRecovery.hpp
	DEPENDS
		px4_work_queue
		mathlib
	)
