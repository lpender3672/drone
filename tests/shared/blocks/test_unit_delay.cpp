#include <gtest/gtest.h>
#include "blocks/unit_delay.hpp"

using shared::UnitDelayBlock;

TEST(UnitDelayBlock, IsMarkedAsDelay) {
    UnitDelayBlock<int> d("d");
    EXPECT_TRUE(d.is_delay());
}

TEST(UnitDelayBlock, InitialOutputIsDefaultConstructed) {
    UnitDelayBlock<int> d("d");
    EXPECT_EQ(d.output().get(), 0);
}

TEST(UnitDelayBlock, InitialOutputCanBeSpecified) {
    UnitDelayBlock<int> d("d", 42);
    EXPECT_EQ(d.output().get(), 42);
}

// Core z⁻¹ semantics: out at tick N == in at tick N-1.
TEST(UnitDelayBlock, OutputAtTickNEqualsInputAtTickNMinus1) {
    UnitDelayBlock<int> d("d", /*initial=*/0);

    // Tick 0: input 7, output is the initial value (no prior input).
    d.input().set(7);
    d.update(0);
    EXPECT_EQ(d.output().get(), 0);

    // Tick 1: input 11, output is the input that was set during tick 0.
    d.input().set(11);
    d.update(1);
    EXPECT_EQ(d.output().get(), 7);

    // Tick 2: output is the input from tick 1.
    d.input().set(99);
    d.update(2);
    EXPECT_EQ(d.output().get(), 11);

    // Tick 3: output is the input from tick 2.
    d.input().set(0);
    d.update(3);
    EXPECT_EQ(d.output().get(), 99);
}

TEST(UnitDelayBlock, RespectsUpdatePeriod) {
    UnitDelayBlock<int> d("d", 0, /*update_period_us=*/5000);
    d.input().set(7);

    EXPECT_TRUE(d.update(0));        // first call always due
    EXPECT_FALSE(d.update(1000));    // dt < period
    EXPECT_TRUE(d.update(5000));     // dt == period
}
