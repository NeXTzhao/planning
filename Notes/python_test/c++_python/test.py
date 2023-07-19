import ctypes

cpp = ctypes.cdll.LoadLibrary
lib = cpp("/home/next/planning/Notes/python_test/c++_python/libtest.so")
a = lib.sum(1, 2)
print(a)
