#include "sin_cos_table.h"
#include <cmath>
#include <iostream>
using namespace std;

template<typename T>
class Test {
public:
    Test() = default;
    explicit Test(const T &a = 1, const T &b = 1) : a_(a), b_(b) {}

    T getA() const {
        return a_;
    }

    T getB() const {
        return b_;
    }
    template<typename T1>
    friend Test<T1> operator*(const Test<T1> &lhs, const Test<T1> &rhs);
    //    {
    //        return Test<T1>(lhs.getA() * rhs.getA(), lhs.getB() * rhs.getB());
    //    }

private:
    T a_, b_;
};

template<typename T1>
Test<T1> operator*(const Test<T1> &lhs, const Test<T1> &rhs) {
    return Test<T1>(lhs.getA() * rhs.getA(), lhs.getB() * rhs.getB());
}

class Myclass {
public:
    Myclass(int x) : _x(x){};
    int operator()(const int n) const {
        return n * _x;
    }

private:
    int _x;
};

void sin_table_() {
    double theta = 0.0;
    double x = M_PI_2 / 200;
    /* for (int i = 199; i >= 0; --i) {
        if (i == 0) {
            sin(M_PI_2);
            printf("%f,\n", sin(M_PI_2));
            break;
        }
        printf("%f,\n", sin(theta));
        theta += x;
    }
*/
    for (int i = 199; i >= 0; --i) {
        if (i == 0) {
            printf("%f,\n", cos(M_PI_2));
            break;
        }
        printf("%f,\n", cos(theta));
        theta += x;
    }
}


int main() {
    Test<int> c1(1, 2);
    Test<int> c2(2, 3);
    Test<int> c3 = c1 * c2;

    Myclass Obj1(5);
    cout << Obj1(3) << endl;

    //    sin_table();
    double a = 0.0;
    double b = 0.7853981633974483;//45
    double c = M_PI_2;            //90
    double d = M_PI;              //180
    double e = 1.5 * M_PI;
    double e1 = 1.7 * M_PI;

    double f = 2 * M_PI;
    double g = 2.5 * M_PI;

    printf("theta,sin,sin(theta):%f,%f,%f\n", a, sin(a), find_sin(a));
    printf("theta,cos,cos(theta):%f,%f,%f\n", a, cos(a), find_cos(a));

    printf("theta,sin,sin(theta):%f,%f,%f\n", b, sin(b), find_sin(b));
    printf("theta,cos,cos(theta):%f,%f,%f\n", b, cos(b), find_cos(b));

    printf("theta,sin,sin(theta):%f,%f,%f\n", c, sin(c), find_sin(c));
    printf("theta,cos,cos(theta):%f,%f,%f\n", c, cos(c), find_cos(c));

    printf("theta,sin,sin(theta):%f,%f,%f\n", d, sin(d), find_sin(d));
    printf("theta,cos,cos(theta):%f,%f,%f\n", d, cos(d), find_cos(d));

    printf("theta,sin,sin(theta):%f,%f,%f\n", e, sin(e), find_sin(e));
    printf("theta,cos,cos(theta):%f,%f,%f\n", e, cos(e), find_cos(e));

    printf("theta,sin,sin(theta):%f,%f,%f\n", e1, sin(e1), find_sin(e1));
    printf("theta,cos,cos(theta):%f,%f,%f\n", e1, cos(e1), find_cos(e1));

    printf("theta,sin,sin(theta):%f,%f,%f\n", f, sin(f), find_sin(f));
    printf("theta,cos,cos(theta):%f,%f,%f\n", f, cos(f), find_cos(f));

    printf("theta,sin,sin(theta):%f,%f,%f\n", g, sin(g), find_sin(g));
    printf("theta,cos,cos(theta):%f,%f,%f\n", g, cos(g), find_cos(g));

//    double xx = -1.3;
//    int x1 = static_cast<int>(xx);
//    if (xx - x1 >= 0.5) {
//        x1 += 1;
//    } else {
//        x1 = x1;
//    }
//    std::cout << "x:::"<< x1 << std::endl;

    
    std::cout <<ty_sin(a)<<std::endl;
    std::cout <<ty_sin(b)<<std::endl;
    std::cout <<ty_sin(c)<<std::endl;
    std::cout <<ty_sin(d)<<std::endl;
    std::cout <<ty_sin(e)<<std::endl;
    std::cout <<ty_sin(f)<<std::endl;

    return 0;
}