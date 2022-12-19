#include "primitives.h"

Rect::Rect(const Point & given_bottom_left, const Point & given_top_right)
    : bottom_left(given_bottom_left)
    , top_right(given_top_right)
{
}

Point Rect::get_bottom_left() const
{
    return bottom_left;
}

Point Rect::get_top_right() const
{
    return top_right;
}

double Rect::xmin() const
{
    return bottom_left.x();
}
double Rect::ymin() const
{
    return bottom_left.y();
}
double Rect::xmax() const
{
    return top_right.x();
}
double Rect::ymax() const
{
    return top_right.y();
}
double Rect::distance(const Point & p) const
{
    if (contains(p)) {
        return 0;
    }
    if (p.y() > ymax()) {
        if (p.x() < xmin()) {
            return p.distance(Point(xmin(), ymax()));
        }
        return (p.x() > xmax() ? p.distance(Point(xmax(), ymax())) : p.y() - ymax());
    }
    if (p.y() < ymin()) {
        if (p.x() < xmin()) {
            return p.distance(Point(xmin(), ymin()));
        }
        return (p.x() > xmax() ? p.distance(Point(xmax(), ymin())) : ymin() - p.y());
    }
    return (p.x() < xmin() ? xmin() - p.x() : p.x() - xmax());
}

bool Rect::contains(const Rect & rect) const
{
    return xmin() <= rect.xmin() && rect.xmax() <= xmax() && ymin() <= rect.ymin() && rect.ymax() <= ymax();
}

bool Rect::contains(const Point & p) const
{
    return xmin() <= p.x() && p.x() <= xmax() && ymin() <= p.y() && p.y() <= ymax();
}

bool Rect::intersects(const Rect & other) const
{
    bool other_left_between_this = xmin() <= other.xmin() && other.xmin() <= xmax();
    bool other_right_between_this = xmin() <= other.xmax() && other.xmax() <= xmax();
    bool other_top_between_this = ymin() <= other.ymax() && other.ymax() <= ymax();
    bool other_bottom_between_this = ymin() <= other.ymin() && other.ymin() <= ymax();

    bool upper_left = other_left_between_this && other_top_between_this;
    bool upper_right = other_right_between_this && other_top_between_this;
    bool lower_left = other_left_between_this && other_bottom_between_this;
    bool lower_right = other_right_between_this && other_bottom_between_this;

    bool cur_is_mid_left = other_right_between_this && other.ymin() <= ymin() && ymax() <= other.ymax();
    bool cur_is_mid_right = other_left_between_this && other.ymin() <= ymin() && ymax() <= other.ymax();

    return contains(other) || other.contains(*this) || upper_left || upper_right || lower_left || lower_right || cur_is_mid_left || cur_is_mid_right;
}
