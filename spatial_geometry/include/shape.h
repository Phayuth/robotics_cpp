#ifndef SHAPE_H
#define SHAPE_H
class Rectangle {
    public:
        float x;
        float y;
        float w;
        float h;

        Rectangle(float x, float y, float w, float h);
        ~Rectangle();

        void print_properties();
};
bool check_rectangle(Rectangle rec1, Rectangle rec2);
bool check_rectangle_r(Rectangle &rec1, Rectangle &rec2);
#endif