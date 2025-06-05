#include "collision.h"
#include <iostream>

Rectangle::Rectangle(double x, double y, double h, double w) {
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

Line::Line(double xs, double ys, double xe, double ye) {
    this->xs = xs;
    this->ys = ys;
    this->xe = xe;
    this->ye = ye;
}

Line::~Line() {
}

bool check_rectangle(Rectangle rec1, Rectangle rec2) {
    if ((rec1.x + rec1.w >= rec2.x) && (rec1.x <= rec2.x + rec2.w) &&
        (rec1.y + rec1.h >= rec2.y) && (rec1.y <= rec2.y + rec2.h)) {
        return true;
    }
    return false;
}

bool check_line_v_line(Line line1, Line line2) {
    auto x1 = line1.xs;
    auto y1 = line1.ys;
    auto x2 = line1.xe;
    auto y2 = line1.ye;

    auto x3 = line2.xs;
    auto y3 = line2.ys;
    auto x4 = line2.xe;
    auto y4 = line2.ye;

    double uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) /
                ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    double uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) /
                ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

    if ((uA >= 0.0) && (uA <= 1.0) && (uB >= 0.0) && (uB <= 1.0)) {
        return true;
    }
    return false;
}

bool check_link_v_rectangle(Line line, Rectangle rec) {

    auto l1 = Line(rec.x, rec.y, rec.x + rec.w, rec.y);
    auto l2 = Line(rec.x + rec.w, rec.y, rec.x + rec.w, rec.y + rec.h);
    auto l3 = Line(rec.x + rec.w, rec.y + rec.h, rec.x, rec.y + rec.h);
    auto l4 = Line(rec.x, rec.y + rec.h, rec.x, rec.y);

    if (check_line_v_line(line, l1) || check_line_v_line(line, l2) ||
        check_line_v_line(line, l3) || check_line_v_line(line, l4)) {
        return true;
    }
    return false;
}
