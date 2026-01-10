#include <unity.h>

void test_assert(void) {
    TEST_ASSERT(true);
}

void setUp(void) {
    // set up code
}

void tearDown(void) {
    // clean up code
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_assert);
    UNITY_END();
    
    return 0;
}
