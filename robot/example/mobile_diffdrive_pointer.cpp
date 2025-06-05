#include <cmath>
#include <iostream>

// with pointer
float v = 0.1;   // v[m/s]
float omega = 0; // omega[rad/s]
float Ts = 0.03; // Ts[s]

float x = 0, y = 0, t = 0; // initial value for coordinate
float *xptr = &x, *yptr = &y, *tptr = &t;

void printing(float x, float y, float t) {
    std::cout << "x = " << x << " , y = " << y << " , theta = " << t << std::endl;
}

int main(int argc, char const *argv[]) {
    for (int i = 0; i < 100; ++i) {
        printing(x, y, t);
        *xptr += v * cos(t) * Ts;
        *yptr += v * sin(t) * Ts;
        *tptr += omega * Ts;
    }
    return 0;
}
