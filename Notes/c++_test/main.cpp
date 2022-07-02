//
// Created by next on 22-4-19.
//
#include <iostream>
#include "A.h"

using namespace std;

#include <vector>

inline int A::max() {
    return a > b ? a : b;
}

int main() {

    vector<double> vec;
    int index = 0;
    for (int i = 0; i < 3; ++i) {
        vec[index] = i;
        index++;
        vec[index] = i + 1;
        index++;
    }

    for (auto item: vec) {
        std::cout << "item:" << item << '\n';
    }
    return 0;
}