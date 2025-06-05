#include "collision.h"
#include <iostream>

int main() {
    Rectangle rec1(0.4, 1.3, 3.3, 4.4);
    rec1.print_properties();

    Line l1(-1.0, 1.0, 0.0, 5.0);

    Rectangle rec2(0.4, 1.3, 3.3, 4.4);
    rec1.print_properties();

    bool c = check_link_v_rectangle(l1, rec1);
    printf("The collision is %d \n", c);

    return 0;
}