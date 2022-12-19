#include "primitives.h"

#include <iostream>

template <typename T>
void test(T & set)
{
    for (double i = 0; i < 10; ++i) {
        set.put({i, i});
    }
    for (double i = 0; i < 10; ++i) {
        assert(set.contains({i, i}));
    }
    auto [first, last] = set.range({{3, 3}, {7, 7}});
    std::cout << "Next line must contain dots from 3 to 7." << std::endl;
    while (first != last) {
        std::cout << *first << "; ";
        first++;
    }
    std::cout << '\n';
    auto [start, finish] = set.nearest({5, 5}, 3);
    std::cout << "Next line must contain dots from 4 to 6." << std::endl;
    while (start != finish) {
        std::cout << *start << "; ";
        start++;
    }
    std::cout << '\n';
    std::cout << "Next line must contain the entire tree." << std::endl;
    std::cout << set << std::endl;
}

int main()
{
    rbtree::PointSet rb;
    kdtree::PointSet kd;
    std::cout << "Testing rb-tree:" << std::endl;
    test(rb);
    std::cout << "#################################################" << '\n';
    std::cout << "Testing kd-tree:" << std::endl;
    test(kd);
}
