#include "shape.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

Rectangle::Rectangle(float x, float y, float w, float h) {
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
}

void Rectangle::print_properties() {
    printf("Rectangle Shape with x: %f, y: %f, w: %f, h: %f \n", x, y, w, h);
}

Rectangle::~Rectangle() {
}
/*
checkking rectangle collsion with pass by value
*/
bool check_rectangle(Rectangle rec1, Rectangle rec2) {
    if ((rec1.x + rec1.w >= rec2.x) && (rec1.x <= rec2.x + rec2.w) &&
        (rec1.y + rec1.h >= rec2.y) && (rec1.y <= rec2.y + rec2.h)) {
        return true;
    }
    return false;
}
/*
checking rectangle collision with pass by reference
*/
bool check_rectangle_r(Rectangle &rec1, Rectangle &rec2) {
    if ((rec1.x + rec1.w >= rec2.x) && (rec1.x <= rec2.x + rec2.w) &&
        (rec1.y + rec1.h >= rec2.y) && (rec1.y <= rec2.y + rec2.h)) {
        return true;
    }
    return false;
}
