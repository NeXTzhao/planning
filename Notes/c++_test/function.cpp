//
// Created by next on 2022/4/7.
//

#include <cassert>

class Plus {
public:
    Plus(const int data) :
            data(data) {}

    int operator()(const int value) const {
        return value + data;
    }

private:
    const int data;
};


#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <iostream>

class Shape {
public:
    enum ShapeColor {
        Red, Green, Blue
    };

    void doDraw(ShapeColor color = Blue) const{
        draw(color);
    }
private:
    virtual void draw(ShapeColor color) const = 0;
};

class Rectangle : public Shape {
private:
    virtual void draw(ShapeColor color) const {
        std::cout << "Rectangle:"<< color << '\n';
    }
};

class Circle : public Shape {
private:
    virtual void draw(ShapeColor color) const {
        std::cout << "Circle:" << color << '\n';
    }

};

using namespace std;

int main() {
    Shape *ps;
    Shape *pc = new Circle;
    Shape *pr = new Rectangle;

    ps = pc;
    ps->doDraw(Shape::Red);
    ps = pr;
    ps->doDraw(Shape::Green);

    struct point{
        int x;
    };

    point p;
    p.x=3;

//    Plus plusOne(1);
//    assert(plusOne(2) == 3);
//
//    Plus plusTwo(2);
//    assert(plusTwo(2) == 4);

//    string a = "a";
//    string b = "b";
//    vector<string> s{a,b};
//    vector<vector<string>> str{s};
//
//    unordered_map<string, map<string, int>> tar;
//
//    for(const auto& vec: str){
//        tar[vec[0]][vec[1]]++;
//    }

//    tar["a"]["b"] += 1;
//    std::cout << tar["a"]["b"] <<'\n';
////    for(const auto & item: tar){
////        std::cout<<item.first<<'\n';
////        std::cout<<item.second.at(0)<<'\n';
////    }


    return 0;
}
