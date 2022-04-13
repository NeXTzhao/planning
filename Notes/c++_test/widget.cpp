//
// Created by next on 2022/4/6.
//

#include <iostream>
#include <memory>
#include <utility>

namespace Widget1 {
    class WidgetImpl {
    public:
        WidgetImpl(int a, int b, int c) : a_(a), b_(b), c_(c) {};

        void print() {
            std::cout << "a,b,c:" << a_ << "," << b_ << "," << c_ << std::endl;
        }

    private:
        int a_, b_, c_;
    };

    class Widget {
    public:
        Widget(std::shared_ptr<WidgetImpl> wPtr) : pImpl(wPtr) {}

        void swap(Widget &rhs) {
            using std::swap;
            swap(pImpl, rhs.pImpl);
        }

        void show() {
            pImpl->print();
        }

    private:
        std::shared_ptr<WidgetImpl> pImpl;
    };

//namespace std {
//    template<>
//    void swap<Widget>(Widget &a, Widget &b) noexcept(__and_<is_nothrow_move_constructible<Widget>, is_nothrow_move_assignable<Widget>>::value) {
//        a.swap(b);
//    }
//}

    template<typename T>
    void Swap(T &obj1, T &obj2) {
        using std::swap;
        swap(obj1, obj2);
    }
}

using namespace Widget1;

int main() {
    auto a = std::make_shared<WidgetImpl>(1, 2, 3);
    auto b = std::make_shared<WidgetImpl>(4, 5, 6);

    Widget w1(a);
    Widget w2(b);

//    w1.swap(w2);
//    doSomething(w1,w2);
//    Swap(w1,w2);
    int a1 = 3, b1 = 4;
    Swap(a1, b1);
    std::cout << "a1,b1:" << a1 <<","<< b1 << std::endl;
    w1.show();
    w2.show();

    return 0;
}