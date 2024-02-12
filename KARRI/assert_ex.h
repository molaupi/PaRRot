#pragma once

// Patrick Steil
// copied from there https://stackoverflow.com/a/2193717
#include <cassert>

#ifndef NDEBUG
#define ASSERT_EX(condition, statement) \
    do {                                \
        if (!(condition)) {             \
            statement;                  \
            assert(condition);          \
        }                               \
    } while (false)
#else
#define ASSERT_EX(condition, statement) ((void)0)
#endif
