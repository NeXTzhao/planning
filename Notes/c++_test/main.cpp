//
// Created by next on 22-4-19.
//
#include <iostream>
#include "A.h"
using namespace std;
inline int A::max()
{
    return a > b ? a : b;
}

int main()
{
    A a(3, 5);
    cout << a.max() << endl;
    return 0;
}