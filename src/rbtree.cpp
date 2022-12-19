#include "primitives.h"

namespace rbtree {

PointSet::PointSet(const std::string & filename)
{
    if (!filename.empty()) {
        std::ifstream file(filename);
        assert(file.good());
        std::string temp;
        while (!file.eof()) {
            double x, y;
            file >> x >> y;
            put(Point(x, y));
        }
    }
}

bool PointSet::empty() const
{
    return m_set.empty();
}

std::size_t PointSet::size() const
{
    return m_set.size();
}

void PointSet::put(const Point & point)
{
    m_set.insert(point);
}

bool PointSet::contains(const Point & point) const
{
    return m_set.find(point) != m_set.end();
}

// second iterator points to an element out of range
std::pair<PointSet::iterator, PointSet::iterator> PointSet::range(const Rect & rect) const
{
    std::shared_ptr<std::vector<Point>> result = std::make_shared<std::vector<Point>>();
    for (PointSet::iterator iter = begin(); iter != end(); ++iter) {
        if (rect.contains({iter->x(), iter->y()})) {
            result->push_back(*iter);
        }
    }
    return std::make_pair(iterator(result, result->begin()), iterator(result, result->end()));
}

PointSet::iterator PointSet::begin() const
{
    return PointSet::iterator(this, m_set.begin());
}

PointSet::iterator PointSet::end() const
{
    return PointSet::iterator(this, m_set.end());
}

std::optional<Point> PointSet::nearest(const Point & point) const
{
    if (empty()) {
        return {};
    }
    return *std::min_element(begin(), end(), [point](const auto & a, const auto & b) { return point.distance(a) < point.distance(b); });
}

// second iterator points to an element out of range
std::pair<PointSet::iterator, PointSet::iterator> PointSet::nearest(const Point & p, std::size_t k) const
{
    std::shared_ptr<std::vector<std::pair<double, Point>>> result = std::make_shared<std::vector<std::pair<double, Point>>>();
    for (PointSet::iterator iter = begin(); iter != end(); ++iter) {
        result->push_back(std::make_pair(p.distance(*iter), *iter));
        std::push_heap(result->begin(), result->end());
        if (result->size() > k) {
            std::pop_heap(result->begin(), result->end());
            result->pop_back();
        }
    }
    return std::make_pair(iterator(result, result->begin()), iterator(result, result->end()));
}

} // namespace rbtree
