
#include <iostream>
#include <cassert>

int main() {
    int a = 3;
    int b = 4;

    auto f = [&]() {
        a = 5;
        b = 6;
    };

    f();
    std::cout << "a,b:" << a << "," << b << std::endl;
}