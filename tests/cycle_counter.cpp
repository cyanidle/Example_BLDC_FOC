#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "cycle_counter.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

int main() {
    cycle_counter::Extension state;
    CHECK(cycle_counter::extend(state, 10) == 10);
    CHECK(cycle_counter::extend(state, 20) == 20);
    CHECK(cycle_counter::extend(state, 20) == 20);

    CHECK(cycle_counter::extend(state, UINT32_MAX - 10U) == UINT32_MAX - 10U);
    CHECK(cycle_counter::extend(state, 5) == (uint64_t{1} << 32) + 5U);
    CHECK(cycle_counter::extend(state, 6) == (uint64_t{1} << 32) + 6U);

    CHECK(cycle_counter::extend(state, UINT32_MAX - 1U) ==
        (uint64_t{1} << 32) + UINT32_MAX - 1U);
    CHECK(cycle_counter::extend(state, 7) == (uint64_t{2} << 32) + 7U);

    std::cout << "cycle_counter: ok\n";
    return 0;
}
