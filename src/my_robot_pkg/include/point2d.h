#ifndef POINT2D_H
#define POINT2D_H

#include <iostream>

/**
 * @struct Point2D
 * @brief Represents a 2D point with integer coordinates.
 */
struct Point2D {
    int x; ///< X-coordinate of the point.
    int y; ///< Y-coordinate of the point.

    /**
     * @brief Constructs a Point2D with given coordinates.
     * @param x_val The x-coordinate.
     * @param y_val The y-coordinate.
     */
    Point2D(int x_val, int y_val);

    /**
     * @brief Equality operator to compare two points.
     * @param other Another Point2D to compare with.
     * @return True if both points have the same coordinates, false otherwise.
     */
    bool operator==(const Point2D& other) const;
};

#endif // POINT2D_H
