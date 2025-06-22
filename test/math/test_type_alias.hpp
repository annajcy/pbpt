#pragma once

#include <gtest/gtest.h>

#ifdef FLOAT_64BIT
#define EXPECT_FLOAT_EQ EXPECT_DOUBLE_EQ 
#endif