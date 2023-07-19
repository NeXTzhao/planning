/**
 * @file aws_test.cpp
 * @brief 测试aws toolkit的自动生成代码功能
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2023-06-14 17:21:43
 * 
 * @copyright Copyright (c) 2023 
 */
#include <iostream>

// 冒泡排序
void bubbleSort(int* arr, int len) {
  for (int i = 0; i < len; i++) {
    for (int j = 0; j < len - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

// 快速排序
void quickSort(int* arr, int left, int right) {
  if (left < right) {
    int i = left;
    int j = right;
    int pivot = arr[left];  // 选择第一个元素作为基准值
    while (i < j) {
      while (i < j && arr[j] >= pivot) {
        j--;
      }
      if (i < j) {
        arr[i] = arr[j];  // 将较小的元素移到左边
        i++;
      }
      while (i < j && arr[i] < pivot) {
        i++;
      }
      if (i < j) {
        arr[j] = arr[i];  // 将较大的元素移到右边
        j--;
      }
    }
    arr[i] = pivot;               // 将基准值放入正确的位置
    quickSort(arr, left, i - 1);  // 对基准值左边的子数组进行递归排序
    quickSort(arr, i + 1, right);  // 对基准值右边的子数组进行递归排序
  }
}

// 冒泡排序耗时
void test_0() {
  int start = clock();
  int arr[] = {3, 5, 2, 1, 4};
  int len = sizeof(arr) / sizeof(arr[0]);
  bubbleSort(arr, len);
  int end = clock();
  std::cout << "快速排序耗时：" << (end - start) << "ms" << std::endl;
  for (int i = 0; i < len; i++) {
    std::cout << arr[i] << " ";
    std::cout << std::endl;
  }
}

// 快速排序耗时
void test_1() {
  int start = clock();
  int arr[] = {3, 5, 2, 1, 4};
  int len = sizeof(arr) / sizeof(arr[0]);
  quickSort(arr, 0, len - 1);
  int end = clock();
  std::cout << "快速排序耗时：" << (end - start) << "ms" << std::endl;
  for (int i = 0; i < len; i++) {
    std::cout << arr[i] << " ";
    std::cout << std::endl;
  }
}

int main() {
  test_0();
  test_1();

  return 0;
}

