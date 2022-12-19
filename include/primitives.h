#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

class Point
{
private:
    double x_coord;
    double y_coord;

public:
    Point(double x, double y);

    double x() const;
    double y() const;
    double distance(const Point & other) const;

    bool operator<(const Point & other) const;
    bool operator>(const Point & other) const;
    bool operator<=(const Point & other) const;
    bool operator>=(const Point & other) const;
    bool operator==(const Point & other) const;
    bool operator!=(const Point & other) const;

    friend std::ostream & operator<<(std::ostream & stream, const Point & point)
    {
        return stream << point.x() << " " << point.y();
    }
};

class Rect
{
private:
    Point bottom_left;
    Point top_right;

public:
    Rect(const Point & given_bottom_left, const Point & given_top_right);

    Point get_bottom_left() const;
    Point get_top_right() const;

    double xmin() const;
    double ymin() const;
    double xmax() const;
    double ymax() const;
    double distance(const Point & p) const;

    bool contains(const Rect & rect) const;
    bool contains(const Point & p) const;
    bool intersects(const Rect &) const;
};

namespace rbtree {

class PointSet
{
private:
    std::set<Point> m_set;

public:
    class iterator
    {
        using set_iterator = std::set<Point>::iterator;
        using vector_iterator = std::vector<Point>::iterator;
        using heap_iterator = std::vector<std::pair<double, Point>>::iterator;
        using set_ptr = const PointSet *;
        using vector_ptr = std::shared_ptr<std::vector<Point>>;
        using heap_ptr = std::shared_ptr<std::vector<std::pair<double, Point>>>;

        std::variant<set_iterator, vector_iterator, heap_iterator> m_current;
        std::variant<vector_ptr, set_ptr, heap_ptr> m_set;

        bool range() const
        {
            return std::holds_alternative<vector_iterator>(m_current);
        }

        bool nearest() const
        {
            return std::holds_alternative<heap_iterator>(m_current);
        }

    public:
        using value_type = Point;
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using pointer = const Point *;
        using reference = const Point &;

        iterator(set_ptr given_m_set, set_iterator given_m_current)
            : m_current(given_m_current)
            , m_set(given_m_set)
        {
        }

        iterator(vector_ptr given_m_set, vector_iterator given_m_current)
            : m_current(given_m_current)
            , m_set(std::move(given_m_set))
        {
        }

        iterator(heap_ptr given_m_set, heap_iterator given_m_current)
            : m_current(given_m_current)
            , m_set(std::move(given_m_set))
        {
        }

        iterator() = default;

        friend bool operator==(const iterator & lhs, const iterator & rhs)
        {
            return lhs.m_set == rhs.m_set && lhs.m_current == rhs.m_current;
        }

        friend bool operator!=(const iterator & lhs, const iterator & rhs)
        {
            return !(lhs == rhs);
        }

        pointer operator->() const
        {
            if (range()) {
                return &(*std::get<vector_iterator>(m_current));
            }
            if (nearest()) {
                return &(std::get<heap_iterator>(m_current)->second);
            }
            return &*std::get<set_iterator>(m_current);
        }

        reference operator*() const
        {
            if (range()) {
                return *std::get<vector_iterator>(m_current);
            }
            if (nearest()) {
                return std::get<heap_iterator>(m_current)->second;
            }
            return *std::get<set_iterator>(m_current);
        }

        iterator & operator++()
        {
            if (range()) {
                ++std::get<vector_iterator>(m_current);
            }
            else if (nearest()) {
                ++std::get<heap_iterator>(m_current);
            }
            else {
                ++std::get<set_iterator>(m_current);
            }
            return *this;
        }

        iterator operator++(int)
        {
            auto tmp = *this;
            operator++();
            return tmp;
        }
    };

    PointSet(const std::string & filename = {});

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    // second iterator points to an element out of range
    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const;
    iterator end() const;

    std::optional<Point> nearest(const Point &) const;
    // second iterator points to an element out of range
    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator<<(std::ostream & stream, const PointSet & set)
    {
        for (auto iter = set.begin(); iter != set.end(); iter++) {
            stream << *iter << "; ";
        }
        return stream;
    }
};

} // namespace rbtree

namespace kdtree {

class PointSet
{
private:
    struct Node
    {
        std::shared_ptr<Node> left;
        std::shared_ptr<Node> right;
        std::weak_ptr<Node> parent;
        Rect region;
        Point data;

        Node(Point given_data, Rect given_region, std::shared_ptr<Node> given_left, std::shared_ptr<Node> given_right, const std::shared_ptr<Node> & given_parent)
            : left(std::move(given_left))
            , right(std::move(given_right))
            , parent(given_parent)
            , region(given_region)
            , data(given_data)
        {
        }
    };

    std::shared_ptr<Node> begin_pointer;
    std::shared_ptr<Node> end_pointer;
    std::shared_ptr<Node> root;
    std::size_t m_size = 0;

    static void report_subtree(const std::shared_ptr<Node> & cur, const std::shared_ptr<std::vector<Point>> & result);
    static void search_range_child(const std::shared_ptr<Node> & child, const Rect & rect, const std::shared_ptr<std::vector<Point>> & result);

    std::pair<std::shared_ptr<Node>, bool> find(std::shared_ptr<Node> cur, const Point & to_find, bool depth) const;

    void update();
    static void restore(const std::shared_ptr<Node> & cur);
    static Point update_bottom_left(const std::shared_ptr<Node> & left_son, const std::shared_ptr<Node> & right_son);
    static Point update_top_right(const std::shared_ptr<Node> & left_son, const std::shared_ptr<Node> & right_son);

    std::shared_ptr<Node> next(std::shared_ptr<Node> cur) const;

    static Point nearest_impl(const std::shared_ptr<Node> & cur, const Point & point, double min);
    static void nearest_impl(const std::shared_ptr<Node> & cur, const Point & point, std::size_t k, std::shared_ptr<std::vector<std::pair<double, Point>>> & heap);
    static void search_range(const std::shared_ptr<Node> & cur, const Rect & rect, const std::shared_ptr<std::vector<Point>> & result);

    void constructor_impl(std::vector<Point> input);
    static std::shared_ptr<Node> build_tree(std::vector<Point>::iterator start, std::vector<Point>::iterator finish, bool depth);

public:
    PointSet(const std::string & filename = {});

    class iterator
    {
        using node_ptr = std::shared_ptr<Node>;
        using vector_iterator = std::vector<Point>::iterator;
        using heap_iterator = std::vector<std::pair<double, Point>>::iterator;
        using set_ptr = const PointSet *;
        using vector_ptr = std::shared_ptr<std::vector<Point>>;
        using heap_ptr = std::shared_ptr<std::vector<std::pair<double, Point>>>;

        std::variant<node_ptr, vector_iterator, heap_iterator> m_current = nullptr;
        std::variant<vector_ptr, set_ptr, heap_ptr> m_tree;

        bool range() const
        {
            return std::holds_alternative<vector_iterator>(m_current);
        }

        bool nearest() const
        {
            return std::holds_alternative<heap_iterator>(m_current);
        }

    public:
        using value_type = Point;
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using pointer = const Point *;
        using reference = const Point &;

        iterator(set_ptr given_m_tree, node_ptr given_m_current)
            : m_current(given_m_current)
            , m_tree(given_m_tree)
        {
        }

        iterator(vector_ptr given_m_tree, vector_iterator given_m_current)
            : m_current(given_m_current)
            , m_tree(std::move(given_m_tree))
        {
        }

        iterator(heap_ptr given_m_tree, heap_iterator given_m_current)
            : m_current(given_m_current)
            , m_tree(std::move(given_m_tree))
        {
        }

        iterator() = default;

        friend bool operator==(const iterator & lhs, const iterator & rhs)
        {
            return lhs.m_tree == rhs.m_tree && lhs.m_current == rhs.m_current;
        }

        friend bool operator!=(const iterator & lhs, const iterator & rhs)
        {
            return !(lhs == rhs);
        }

        pointer operator->() const
        {
            if (range()) {
                return &*std::get<vector_iterator>(m_current);
            }
            if (nearest()) {
                return &std::get<heap_iterator>(m_current)->second;
            }
            return &std::get<node_ptr>(m_current)->data;
        }

        reference operator*() const
        {
            if (range()) {
                return *std::get<vector_iterator>(m_current);
            }
            if (nearest()) {
                return std::get<heap_iterator>(m_current)->second;
            }
            return std::get<node_ptr>(m_current)->data;
        }

        iterator & operator++()
        {
            if (range()) {
                ++std::get<vector_iterator>(m_current);
            }
            else if (nearest()) {
                ++std::get<heap_iterator>(m_current);
            }
            else {
                std::get<node_ptr>(m_current) = std::get<set_ptr>(m_tree)->next(std::get<node_ptr>(m_current));
            }
            return *this;
        }

        iterator operator++(int)
        {
            auto tmp = *this;
            operator++();
            return tmp;
        }
    };

    bool empty() const;
    std::size_t size() const;
    void put(const Point & point);
    bool contains(const Point & point) const;

    std::pair<iterator, iterator> range(const Rect & rect) const;
    iterator begin() const;
    iterator end() const;

    std::optional<Point> nearest(const Point & point) const;
    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator<<(std::ostream & stream, const PointSet & set)
    {
        for (auto iter = set.begin(); iter != set.end(); iter++) {
            stream << *iter << "; ";
        }
        return stream;
    }
};

} // namespace kdtree
