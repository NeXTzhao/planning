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

namespace myOperator {

    class Rational {
    public:
        Rational(int numerator = 0, int denominator = 1) {};

//    const Rational operator*(const Rational &rhs) const {}

        int numerator() const {
            return num1;
        }

        int denominator() const {
            return num2;
        }

    private:
        int num1;
        int num2;
    };

    const Rational operator*(const Rational &lhs, const Rational &rhs) {
        return Rational(lhs.numerator() * rhs.numerator(), lhs.denominator() * rhs.denominator());
    }
}

using namespace myOperator;
int main() {
    Rational one(2, 3);
    Rational two(4, 5);
    Rational result = one * two;
    Rational result1 = 2 * one;
    Rational result2 = one * 2;

    auto a = one.numerator();
    std::cout << a << std::endl;


    return 0;
}
