#ifndef COLLISION_H
#define COLLISION_H

class Rectangle {
    public:
        double x;
        double y;
        double w;
        double h;

        Rectangle(double x, double y, double h, double w);
        ~Rectangle();

        void print_properties();
};

class Line {
    public:
        double xs;
        double ys;
        double xe;
        double ye;

        Line(double xs, double ys, double xe, double ye);
        ~Line();
};

bool check_rectangle(Rectangle rec1, Rectangle rec2);
bool check_line_v_line(Line line1, Line line2);
bool check_link_v_rectangle(Line line, Rectangle rec);
#endif