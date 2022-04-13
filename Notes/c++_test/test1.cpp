////
//// Created by next on 2022/4/6.
////
//
////
//// Created by next on 2022/4/6.
////
//
//#include <iostream>
//#include <memory>
//#include <utility>
//
//namespace Widget1 {
//    template<typename T>
//    class WidgetImpl {
//    public:
//        WidgetImpl(T a, T b, T c) : a_(a), b_(b), c_(c) {};
//
//        void print() {
//            std::cout << "a,b,c:" << a_ << "," << b_ << "," << c_ << std::endl;
//        }
//
//    private:
//        T a_, b_, c_;
//    };
//
//    template<typename T>
//    class Widget {
//    public:
//        Widget(T a, T b, T c) {
//            auto wPtr = std::make_shared<WidgetImpl<T>>(a, b, c);
//            pImpl = wPtr;
//        }
//
//        void swap(Widget &rhs) {
//            using std::swap;
//            swap(pImpl, rhs.pImpl);
//        }
//
//        void show() { pImpl->print(); }
//
//    private:
//        std::shared_ptr<WidgetImpl<T>> pImpl;
//    };
//
//    template<typename T>
//    class Widget2 {
//    };
//
//    template<typename T>
//    void Swap(T &obj1, T &obj2) {
//        using std::swap;
//        swap(obj1, obj2);
//    }
//}  // namespace Widget1
//
//using namespace Widget1;
//
//int main() {
//    Widget<int> w1(1, 2, 3);
//    Widget<int> w2(3, 4, 5);
//
//    //    w1.swap(w2);
//    int a = 3, b = 4;
//    //传统stl中的swap
//    Swap(a, b);
////    std::cout << "a,b:" << a << "," << b << std::endl;
//    //成员函数swap
//    Swap(w1, w2);
//    w1.show();
//    w2.show();
//
//    return 0;
//}

#include<iostream>
#include<functional>

using namespace std;

class Plus {
public:
    int plus(int a, int b) {
        return a + b;
    }
};

int main() {
    Plus p;
    // 指针形式调用成员函数
    function<int(int, int)> func1 = std::bind(&Plus::plus, &p, placeholders::_1, placeholders::_2);
    // 对象形式调用成员函数
    function<int(int, int)> func2 = std::bind(&Plus::plus, p, placeholders::_1, placeholders::_2);
    cout << func1(1, 2) << endl; //3
    cout << func2(1, 2) << endl; //3
    return 0;
}