px4_add_module(
	MODULE examples__ck_app
	MAIN test_app
	SRCS
		test_app.cpp
		ck_filters.cpp
	DEPENDS
	)
