////
//// Created by next on 2022/4/6.
////
//
////
//// Created by next on 2022/4/6.
////
//
// #include <iostream>
// #include <memory>
// #include <utility>
//
// namespace Widget1 {
//    template<typename T>
//    class WidgetImpl {
//    public:
//        WidgetImpl(T a, T b, T c) : a_(a), b_(b), c_(c) {};
//
//        void print() {
//            std::cout << "a,b,c:" << a_ << "," << b_ << "," << c_ <<
//            std::endl;
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
// using namespace Widget1;
//
// int main() {
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

#include "matplotlibcpp.h"
#include <functional>
#include <iostream>
#include <vector>

using namespace std;
namespace plt = matplotlibcpp;

std::vector<double> a{1, 2, 3, 4, 5, 6, 7, 8, 9};
std::vector<double> b{1, 4, 6, 8, 10, 12, 14, 16, 18};


// clang

void draw() {
  plt::named_plot("ref_line_kappa", a, b);
//  plt::named_plot("ref_line", b);

  plt::legend();
  plt::axis("equal");
  // plt::legend() 指在图例中显示 “ref_line_XY”等表述信息
  plt::legend();
  //   plt::axis("equal");
  // save figure

  char const* filename = "./../picture/basic.png";
  std::cout << "Saving result to " << filename << std::endl;
  plt::save(filename);
}
int main() {
  draw();
  return 0;
}