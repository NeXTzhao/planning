#include <Python.h>

#include <iostream>
using namespace std;

int main(int argc, char *argv[]) {
  Py_Initialize();
  PyRun_SimpleString(
      "import sys; sys.path.insert(0, "
      "'/home/next/planning/Notes/python_test/c++_python/src')");

  PyObject *pModule, *pFunc;  // 模块和函数指针
  PyObject *pArgs, *pValue;   // 参数和返回值指针

  // 获取test模块
  // 网络上的资料大多数用的是PyString_FromString 这个函数是2.x版本的
  // 在3.x版本中被废弃了 用PyUnicode_FromString代替
  pModule = PyImport_ImportModule("test1");
  if (pModule == NULL) {
    printf("ERROR importing module\n");
    exit(-1);
  }

  // 获取greet函数
  pFunc = PyObject_GetAttrString(pModule, "add_number");
  if (pFunc == NULL) {
    printf("ERROR getting Hello attribute\n");
    exit(-1);
  }
  pArgs = PyTuple_New(1);
  PyTuple_SetItem(pArgs, 0, PyLong_FromLong(3));

  pValue = PyObject_CallObject(pFunc, pArgs);

  // 解析返回值
  int result;
  result = PyLong_AsLong(pValue);
  cout << result << endl;
  Py_Finalize();
  return 0;
}