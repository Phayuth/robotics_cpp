#include "shape.h"
#include <chrono>
#include <iostream>

int main() {
    Rectangle rec1(0.4, 1.3, 3.3, 4.4);
    rec1.print_properties();

    Rectangle rec2(0.4, 1.3, 2.2, 31.1);
    rec2.print_properties();

    bool res = check_rectangle(rec1, rec2);
    std::cout << res << std::endl;
    // 9873[ns]

    bool res2 = check_rectangle_r(rec1, rec2);
    std::cout << res2 << std::endl;
    // 10285[ns]

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
                     .count()
              << "[µs]" << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)
                     .count()
              << "[ns]" << std::endl;
    return 0;
}