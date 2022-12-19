#include "primitives.h"

namespace kdtree {

//constructing a tree from a vector provides a better balance than constructing it via multiple put operations
void PointSet::constructor_impl(std::vector<Point> input) //NOLINT "input can have const qualifier" -- we change its order via std::unique
{
    if (input.empty()) {
        return;
    }
    auto new_end = std::unique(input.begin(), input.end());
    root = build_tree(input.begin(), new_end, true);
    update();
}

Point PointSet::update_top_right(const std::shared_ptr<Node> & left_son, const std::shared_ptr<Node> & right_son)
{
    return Point(
            std::max(left_son->region.get_top_right().x(), right_son->region.get_top_right().x()),
            std::max(left_son->region.get_top_right().y(), right_son->region.get_top_right().y()));
}

Point PointSet::update_bottom_left(const std::shared_ptr<Node> & left_son, const std::shared_ptr<Node> & right_son)
{
    return Point(
            std::min(left_son->region.get_bottom_left().x(), right_son->region.get_bottom_left().x()),
            std::min(left_son->region.get_bottom_left().y(), right_son->region.get_bottom_left().y()));
}

bool is_equal(double x, double y)
{
    return std::fabs(x - y) < std::numeric_limits<double>::epsilon();
}

std::shared_ptr<PointSet::Node> PointSet::build_tree(std::vector<Point>::iterator start, std::vector<Point>::iterator finish, bool depth)
{
    if (finish - start == 1) {
        return std::make_shared<Node>(*start, Rect(*start, *start), nullptr, nullptr, nullptr);
    }
    auto lambda = (depth ? [](const Point & a, const Point & b) { return a.x() < b.x(); } : [](const Point & a, const Point & b) { return a.y() < b.y(); });
    std::sort(start, finish, lambda);

    std::size_t median = std::upper_bound(start, finish, *(start + (finish - start) / 2), lambda) - start - 1;

    std::shared_ptr<Node> left_son(build_tree(start, start + median, !depth));
    std::shared_ptr<Node> right_son(build_tree(start + median, finish, !depth));
    Point bottom_left = update_bottom_left(left_son, right_son);
    Point top_right = update_top_right(left_son, right_son);
    std::shared_ptr<Node> cur = std::make_shared<Node>(*(start + median), Rect(bottom_left, top_right), left_son, right_son, nullptr);
    left_son->parent = cur;
    right_son->parent = cur;

    return cur;
}

void PointSet::update()
{
    if (empty()) {
        return;
    }
    if (begin_pointer == nullptr) {
        begin_pointer = root;
        end_pointer = root;
    }
    while (begin_pointer->left != nullptr) {
        begin_pointer = begin_pointer->left;
    }
    while (end_pointer->right != nullptr) {
        end_pointer = end_pointer->right;
    }
}

std::shared_ptr<PointSet::Node> PointSet::next(std::shared_ptr<Node> cur) const
{
    if (cur == end_pointer) {
        return nullptr;
    }
    std::shared_ptr<Node> prev = cur;
    cur = cur->parent.lock();
    while (cur != root && prev != cur->left) {
        prev = cur;
        cur = cur->parent.lock();
    }
    while (cur != root && cur->right == nullptr) {
        cur = cur->parent.lock();
    }
    cur = cur->right;
    while (cur->left != nullptr) {
        cur = cur->left;
    }
    return cur;
}

bool PointSet::contains(const Point & point) const
{
    std::shared_ptr<Node> result(find(root, point, true).first);
    return result != nullptr && point == result->data;
}

std::pair<std::shared_ptr<PointSet::Node>, bool> PointSet::find(std::shared_ptr<Node> cur, const Point & to_find, bool depth) const
{
    while (cur->left != nullptr) {
        if (depth) {
            cur = (cur->data.x() < to_find.x() ? cur->right : cur->left);
        }
        else {
            cur = (cur->data.y() < to_find.y() ? cur->right : cur->left);
        }
        depth = !depth;
    }
    return std::make_pair(cur, depth);
}

void PointSet::restore(const std::shared_ptr<Node> & cur)
{
    Point bottom_left = update_bottom_left(cur->left, cur->right);
    Point top_right = update_top_right(cur->left, cur->right);
    cur->region = Rect(bottom_left, top_right);
    if (cur->parent.lock() != nullptr) {
        restore(cur->parent.lock());
    }
}

void PointSet::put(const Point & point)
{
    if (root == nullptr) {
        root = std::make_shared<Node>(point, Rect(point, point), nullptr, nullptr, nullptr);
        ++m_size;
    }
    else {
        std::pair<std::shared_ptr<Node>, bool> result = find(root, point, true);
        std::shared_ptr<Node> cur = result.first;
        if (cur->data == point) {
            return;
        }
        bool depth = result.second;
        ++m_size;
        std::shared_ptr<Node> left = std::make_shared<Node>(cur->data, cur->region, nullptr, nullptr, cur);
        std::shared_ptr<Node> right = std::make_shared<Node>(point, Rect(point, point), nullptr, nullptr, cur);
        if ((depth && cur->data.x() > point.x()) || (!depth && cur->data.y() > point.y())) {
            std::swap(left, right);
            cur->data = point;
        }
        cur->data = left->data;
        cur->left = left;
        cur->right = right;
        restore(cur);
    }
    update();
}

//marks all the points in a subtree as a result
void PointSet::report_subtree(const std::shared_ptr<Node> & cur, const std::shared_ptr<std::vector<Point>> & result)
{
    if (cur->left == nullptr) {
        result->push_back(cur->data);
    }
    else {
        report_subtree(cur->left, result);
        report_subtree(cur->right, result);
    }
}

//prevents copy-paste
void PointSet::search_range_child(const std::shared_ptr<Node> & child, const Rect & rect, const std::shared_ptr<std::vector<Point>> & result)
{
    if (rect.contains(child->region)) {
        report_subtree(child, result);
    }
    else if (rect.intersects(child->region)) {
        search_range(child, rect, result);
    }
}

void PointSet::search_range(const std::shared_ptr<Node> & cur, const Rect & rect, const std::shared_ptr<std::vector<Point>> & result)
{
    if (cur->left == nullptr) {
        if (rect.contains(cur->data)) {
            result->push_back(cur->data);
        }
    }
    else {
        search_range_child(cur->left, rect, result);
        search_range_child(cur->right, rect, result);
    }
}

bool PointSet::empty() const
{
    return m_size == 0;
}

std::size_t PointSet::size() const
{
    return m_size;
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::range(const Rect & rect) const
{
    std::shared_ptr<std::vector<Point>> result = std::make_shared<std::vector<Point>>();
    search_range(root, rect, result);
    return std::make_pair(iterator(result, result->begin()), iterator(result, result->end()));
}

PointSet::iterator PointSet::begin() const
{
    return iterator(this, begin_pointer);
}

PointSet::iterator PointSet::end() const
{
    return iterator(this, nullptr);
}

//with a one-point result
Point PointSet::nearest_impl(const std::shared_ptr<Node> & cur, const Point & point, double min)
{
    min = std::min(min, cur->data.distance(point));
    if (cur->left == nullptr) {
        return cur->data;
    }
    else {
        Point left = cur->data;
        Point right = cur->data;
        if (cur->left->region.distance(point) <= min) {
            left = nearest_impl(cur->left, point, min);
        }
        if (cur->right->region.distance(point) <= min) {
            right = nearest_impl(cur->right, point, min);
        }
        double left_dist = point.distance(left);
        double right_dist = point.distance(right);

        return (left_dist < right_dist ? left : right);
    }
}

std::optional<Point> PointSet::nearest(const Point & point) const
{
    if (root == nullptr) {
        return {};
    }
    return nearest_impl(root, point, point.distance(root->data));
}

//with multiple-points result
void PointSet::nearest_impl(const std::shared_ptr<Node> & cur, const Point & point, std::size_t k, std::shared_ptr<std::vector<std::pair<double, Point>>> & heap)
{
    if (cur->left == nullptr) {
        heap->push_back({point.distance(cur->data), cur->data});
        std::push_heap(heap->begin(), heap->end());
        if (heap->size() > k) {
            std::pop_heap(heap->begin(), heap->end());
            heap->pop_back();
        }
    }
    else {
        nearest_impl(cur->left, point, k, heap);
        nearest_impl(cur->right, point, k, heap);
    }
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::nearest(const Point & p, std::size_t k) const
{
    std::shared_ptr<std::vector<std::pair<double, Point>>> result = std::make_shared<std::vector<std::pair<double, Point>>>();
    nearest_impl(root, p, k, result);
    return std::make_pair(iterator(result, result->begin()), iterator(result, result->end()));
}

PointSet::PointSet(const std::string & filename)
{
    if (!filename.empty()) {
        std::ifstream file(filename);
        assert(file.good());
        std::string temp;
        std::vector<Point> input;
        while (!file.eof()) {
            double x, y;
            file >> x >> y;
            input.push_back(Point(x, y));
        }
        constructor_impl(std::move(input));
    }
}

} // namespace kdtree
