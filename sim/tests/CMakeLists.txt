include(FetchContent)

# Fetch GoogleTest
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Test executable
add_executable(sim_tests
    test_quadrotor_dynamics.cpp
)

target_link_libraries(sim_tests
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        sim_lib
)

# Discover tests - use POST_BUILD to ensure binary exists
gtest_discover_tests(sim_tests
    DISCOVERY_MODE PRE_TEST
)
