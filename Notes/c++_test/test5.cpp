/*
 * @Author: wangdezhao
 * @Date: 2022-04-18 13:26:49
 * @LastEditTime: 2022-04-18 13:29:57
 * @FilePath: /c++_test/test5.cpp
 * @Copyright:
 */
#include <iostream>
#include <memory>

class Point {
public:
    Point(int x, int y) : x_(x), y_(y) {}

    void setX(int newVal) { x_ = newVal; }

    void setY(int newVal) { y_ = newVal; }

    int getX() {
        return x_;
    }

    int getY() {
        return y_;
    }

private:
    int x_;
    int y_;
};

struct RectDate {
    RectDate(Point point, Point point1) : ulhc(point), lrhc(point1) {
    }

    Point ulhc;
    Point lrhc;
};


class Rec {
public:
    Rec(Point point1, Point point2) {
        pDate = std::make_shared<RectDate>(point1, point2);
    }

    const Point &upperLeft() const { return pDate->ulhc; }

    Point &lowerLeft() const { return pDate->lrhc; }

private:
    std::shared_ptr<RectDate> pDate;
};

int main() {
    Point coord1(0, 0);
    Point coord2(1, 1);

    const Rec rec (coord1, coord2);

//    rec.upperLeft().setX(2);

rec.lowerLeft().setX(3);
//    std::cout << coord2.getX() << "," << coord2.getY() << '\n';
//    std::cout << rec.upperLeft().getX() << '\n';
    std::cout << rec.lowerLeft().getX() << '\n';

}


