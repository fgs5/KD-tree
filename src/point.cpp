#include "primitives.h"

#include <cmath>

bool is_equal(double x, double y)
{
    return std::fabs(x - y) < std::numeric_limits<double>::epsilon();
}

Point::Point(double x, double y)
    : x_coord(x)
    , y_coord(y)
{
}

double Point::x() const
{
    return x_coord;
}

double Point::y() const
{
    return y_coord;
}

double Point::distance(const Point & other) const
{
    double x_dif = x() - other.x();
    double y_dif = y() - other.y();
    return std::sqrt(x_dif * x_dif + y_dif * y_dif);
}

bool Point::operator<(const Point & other) const
{
    return (is_equal(y(), other.y()) ? x() < other.x() : y() < other.y());
}
bool Point::operator>(const Point & other) const
{
    return (is_equal(y(), other.y()) ? x() > other.x() : y() > other.y());
}
bool Point::operator<=(const Point & other) const
{
    return !operator>(other);
}
bool Point::operator>=(const Point & other) const
{
    return !operator<(other);
}
bool Point::operator==(const Point & other) const
{
    return is_equal(y(), other.y()) && is_equal(x(), other.x());
}
bool Point::operator!=(const Point & other) const
{
    return !operator==(other);
}
