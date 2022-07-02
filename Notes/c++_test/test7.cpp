//
// Created by next on 22-6-24.
//
#include <iostream>

using namespace std;

#include <vector>



int main() {
    vector<double> vec(6,0.0);
    int index = 0;
    for (int i = 0; i < 3; ++i) {
        vec[index] += i;
        index++;
        vec[index] += i + 1;
        index++;
    }

    for (auto item: vec) {
        std::cout << "item:" << item << '\n';
    }
    return 0;
}