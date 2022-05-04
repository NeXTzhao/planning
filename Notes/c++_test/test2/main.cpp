//
// Created by next on 22-4-26.
//
#include "test2.h"

class shape{
public:
    virtual void  test() = 0;
    virtual void fun(){
        std::cout << "this is shape function"<<'\n';
    }
};

void shape::test() {
    std::cout << "myself test"<<'\n';
}

class race :public shape{
public:
    void test(){
        std::cout << "race "<<'\n';
    }
    void aabb(){
        std::cout<< "aabb"<<'\n';
    }
};

int main(){
    test2 te2;
    te2.fun();


//    shape * p2 = new race;
    race * p2 = new race;
    p2->fun();

    p2->test();
    p2->shape::test();
    p2->aabb();
}