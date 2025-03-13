#include "point2d.h"
#include <iostream>
Point2D::Point2D(int x_val, int y_val) : x(x_val), y(y_val) {}
bool Point2D::operator==(const Point2D& other) const { 
    return (this->x == other.x) && (this->y == other.y);
};

