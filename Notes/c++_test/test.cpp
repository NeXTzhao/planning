/***
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-03-30 16:39:44
 * @LastEditors: your name
 * @Brief:
 * @FilePath: /Notes/c++_test/test.cpp
 * @Copyright:
 */

#include <iostream>
#include <string>
#include <memory>

class te1 {
public:
    int a = 4;

    te1() = default;

    te1(const te1 &) {
        std::cout << "copy construct" << std::endl;
    }

    ~te1() {
        std::cout << "delete fun" << std::endl;
    }
//    te1 &operator=(const te1 &) {
//        std::cout << "copy assigment" << std::endl;
//    }
};


class test {
public:
    test() {};

public:
    static const int b = 4;
    const std::string te = "127.0.0.1";
    const int a = 4;

};

const te1 &fun() {
    te1 te;
    return te;
}

int main() {
//    test t;
//    std::cout << t.te << std::endl;
//    te1 t2;
//    std::cout << t2.a << std::endl;

    auto b = fun().a;
    std::cout << b << std::endl;

    return 0;
}