#include "Mapped_Encoder.h"

#include "unit_test_framework.hpp"

// Test that functions work as expected under basic conditions
TEST(functions) {
    Mapped_Encoder pot = Mapped_Encoder(1,0,180,0,180,180,"tester");

    ASSERT_EQUAL(pot.name(), "tester");

    ASSERT_EQUAL(pot.read(), 0);

    ASSERT_EQUAL(pot.readRaw(), 0);

    ASSERT_EQUAL(pot.floatMap(90), 90);

    pot.set_max(90);

    ASSERT_EQUAL(pot.floatMap(45), 90);

    pot.set_min(45);

    ASSERT_EQUAL(pot.floatMap(67.5), 90);
} 

TEST(full_analog_range) {
    ASSERT_TRUE(true);
}


TEST_MAIN()