//
// Created by vtd on 23-8-1.
//
#include <chrono>
#include <complex>
#include <iostream>

class Timer {
 public:
  Timer() : start_time_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_time_ = std::chrono::high_resolution_clock::now(); }

  double elapsed() const {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto start_time =
        std::chrono::time_point_cast<std::chrono::microseconds>(start_time_);
    auto end_time_us =
        std::chrono::time_point_cast<std::chrono::microseconds>(end_time);
    auto duration = end_time_us - start_time;
    return duration.count() * 1e-3;
  }

  void printElapsed(const std::string &message = "") const {
    std::cout << message << " Elapsed time: " << elapsed() << " ms."
              << std::endl;
  }

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

//int main() {
//  Timer timer; // 创建计时器对象
//
//  // 模拟耗时操作，这里使用一个简单的循环
//  double sum = 0.0;
//  for (int i = 0; i < 1000000; ++i) {
//    sum += std::pow(i, 2);
//  }
//
//  // 输出耗时结果
//  timer.printElapsed("Operation");
//
//  // 在另一个代码块中再次测试耗时
//  timer.reset(); // 重置计时器
//  for (int i = 0; i < 2000000; ++i) {
//    sum += std::pow(i, 2);
//  }
//
//  // 输出耗时结果
//  timer.printElapsed("Another operation");
//
//  return 0;
//}

/**
 * @brief 两种计算时间的方法相差在0.3ms左右
 * @return
 */
int main() {
  auto start_Construct = std::chrono::high_resolution_clock::now();
  double sum = 0.0;
  for (int i = 0; i < 1000000; ++i) {
    sum += std::pow(i, 2);
  }

  auto end_Construct = std::chrono::high_resolution_clock::now();
  auto Construct_time = std::chrono::duration_cast<std::chrono::microseconds>(
                            end_Construct - start_Construct)
                            .count();
  std::cout << "Construct time: " << Construct_time / 1000.0 << "ms\n";


  auto start_Construct1 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 2000000; ++i) {
    sum += std::pow(i, 2);
  }

  auto end_Construct1 = std::chrono::high_resolution_clock::now();
  auto Construct_time1 = std::chrono::duration_cast<std::chrono::microseconds>(
                            end_Construct1 - start_Construct1)
                            .count();
  std::cout << "Construct time1: " << Construct_time1 / 1000.0 << "ms\n";
  return 0;
}