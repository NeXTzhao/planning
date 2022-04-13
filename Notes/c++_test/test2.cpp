/*** 
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-04-07 10:29:39
 * @LastEditors: your name
 * @Brief: 
 * @FilePath: /c++_test/test2.cpp
 * @Copyright:  
 */
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

int test(int a) {
    return a - 1;
}

int test2(int (*fun)(int), int b) {
    int c = fun(10) + b;
    return c;
}

int add(int a, int b) {
    return a + b;
}

int sub(int a, int b) {
    return a - b;
}

void func(int a, int b, int(*ptr)(int c, int d)) {
    cout << ptr(a, b) << endl;
}

template<typename T>
void bubblesort(T *a, int n) {
    bool sorted = false;
    while (!sorted) {
        sorted = true;
        for (int i = 0; i < n - 1; ++i) {
            if (a[i] > a[i + 1]) {
                std::swap(a[i], a[i + 1]);
                sorted = false;
            }
            n--;
        }
    }
}

struct DataPool {
    int a_;
    int b_;

    DataPool(int a, int b) : a_(a), b_(b) {}

    friend ostream &operator<<(ostream &output,
                               const DataPool &D) {
        output << "a_ : " << D.a_ << " b_ : " << D.b_;
        return output;
    }

};

bool sortMyself(const DataPool &x, const DataPool &y) {
    return x.b_ > y.b_;

}

int main() {
//    func(2, 3, add);
//    func(5, 9, sub);


////    vector<int> arr{5,2,5,7,1,-3,99,56};
//    int a[8] = {5, 2, 5, 7, 1, -3, 99, 56};
////    bubblesort<vector<int>>(&arr,8);
//    bubblesort<int>(a, 8);
//    for (const auto &item: a) {
//        std::cout << item << std::endl;
//    }

    int a = 3;
    int b = 4;
    auto c = [=]()mutable { a = 5; b = 6; };
    c();
    cout << "a,b:" << a << "," << b << endl;

    vector<DataPool> vec;
    vec.push_back(DataPool(1, 1));
    vec.push_back(DataPool(2, 2));
    vec.push_back(DataPool(3, 3));
    vec.push_back(DataPool(4, 4));

//    std::sort(vec.begin(), vec.end(), sortMyself);

    std::sort(vec.begin(), vec.end(), [](const DataPool &a, const DataPool &b) -> bool {
        return a.b_ > b.b_;
    });

    for (const auto &item: vec) {
        cout << item << endl;
    }
    return 0;
}