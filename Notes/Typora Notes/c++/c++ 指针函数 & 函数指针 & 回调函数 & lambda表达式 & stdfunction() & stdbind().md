# 函数相关

## 函数指针

> **指向函数的指针变量**，即重点是一个指针。一种特殊的指针，它指向函数的入口；
> 要声明指向特定类型的函数指针，可以首先编写这个函数的原型，然后用(\*p)来替换函数名，这样p就是这类函数的指针。

### 1	格式

```c++
//格式: 类型说明符 (*函数名)(参数)
//指向函数的指针包含了函数的地址，可以通过它来调用函数，其实这里不能称为函数名，应该叫做指针的变量名。这个特殊的指针指向一个返回整型值的函数实现地址。
int (*p) (int x);
```

### 2	sample

```c++
#include <iostream>
int add(int a, int b) {
    return a + b;
}

int sub(int a, int b) {
    return a - b;
}

void func(int a, int b, int(*ptr)(int c, int d)) {
    cout << ptr(a, b) << endl;
}


int main() {
    func(2, 3, add);
    func(5, 9, sub);

	return 0；
}
```

## 指针函数

> *首先它是一个函数，只不过这个函数的返回值是一个地址值。指针函数一定有函数返回值，而且在主调函数中，函数返回值必须赋给同类型的指针变量*

### 1	格式

```c++
//格式：类型标识符 *函数名(参数表)
//指针函数：带指针的函数，即本质是一个函数。函数返回类型是某一类型的指针
int  *f(x，y);
```

### 2	sample

```c++
int *f(int a, int b) {  
    int *p = (int *)malloc(sizeof(int));  
    memset(p, 0, sizeof(int));  
    *p = a + b;    
    return p;  
} 

int main(){
	int *p1 = NULL;  
    p1 = f(1, 2);  
}
```



## std::function()

### 1	定义

类模版`std::function`是一种通用、多态的函数封装。`std::function`的实例可以对任何可以调用的目标实体进行存储、复制、和调用操作，这些目标实体包括普通函数、Lambda表达式、函数指针、以及其它函数对象等。`std::function`对象是对C++中现有的可调用实体的一种类型安全的包裹（我们知道像函数指针这类可调用实体，是类型不安全的）。

### 2	sample

```c++
#include <functional>
#include <iostream>
using namespace std;
 
std::function< int(int)> Functional;
// Functional(1);
 
// 普通函数
int TestFunc(int a)
{
    return a;
}
 
// Lambda表达式
auto lambda = [](int a)->int{ return a; };
 
// 仿函数(functor)
class Functor
{
public:
    int operator()(int a)
    {
        return a;
    }
};
 
// 1.类成员函数
// 2.类静态函数
class TestClass
{
public:
    int ClassMember(int a) { return a; }
    static int StaticMember(int a) { return a; }
};
 
int main()
{
    // 普通函数
    Functional = TestFunc;
    int result = Functional(10);
    cout << "普通函数："<< result << endl;
 
    // Lambda表达式
    Functional = lambda;
    result = Functional(20);
    cout << "Lambda表达式："<< result << endl;
 
    // 仿函数
    Functor testFunctor;
    Functional = testFunctor;
    result = Functional(30);
    cout << "仿函数："<< result << endl;
 
    // 类成员函数
    TestClass testObj;
    Functional = std::bind(&TestClass::ClassMember, testObj, std::placeholders::_1);
    result = Functional(40);
    cout << "类成员函数："<< result << endl;
 
    // 类静态函数
    Functional = TestClass::StaticMember;
    result = Functional(50);
    cout << "类静态函数："<< result << endl;
 
    return 0;
}
```

## std::bind()

### 1	定义

可以将bind函数看作是一个通用的函数适配器，它接受一个可调用对象，生成一个新的可调用对象来“适应”原对象的参数列表。

```c++
//形式
auto newCallable = bind(callable,arg_list);`

//newCallable本身是一个可调用对象
//arg_list是一个逗号分隔的参数列表，对应给定的callable的参数。即，当我们调用newCallable时，newCallable会调用callable,并传给它arg_list中的参数。

//arg_list中的参数可能包含形如_n的名字，其中n是一个整数，这些参数是“占位符”，表示newCallable的参数，它们占据了传递给newCallable的参数的“位置”。数值n表示生成的可调用对象中参数的位置：_1为newCallable的第一个参数，_2为第二个参数，以此类推。
```

bind()是一个基于模板的函数，顾明思意它的作用是绑定并返回一个 std::function 对象。

**那么什么是“绑定”？**

​		它本身作为延迟计算的思想的一种实现，作为一个调用过程当中的转发者而存在，返回一个 std::function 对象。它与 std::function 不同的是，function 是模板类，bind 是模板函数，而 bind 返回的可调用对象可以直接给 function 进行包装并保存。

**为什么要进行“包装”与“转发”呢？**

​		首先，不规范的解释是，function 的作用是包装，它可以包装类成员函数，但却无法生成类成员函数的可调用对象。而 std::bind 则是可以生成。

​		因此，function 与 bind 结合后，便成为了 C++ 中类成员函数作为回调函数的一种规范的实现方式。

### 2	绑定普通函数

```c++
#include<iostream>
#include<functional>
using namespace std;

int plus(int a,int b)
{
   return a+b;
}
int main()
{
  //表示绑定函数plus 参数分别由调用 func1 的第一，二个参数指定
   function<int(int,int)> func1 = std::bind(plus, placeholders::_1, placeholders::_2);
   
  //func2的类型为 function<void(int, int, int)> 与func1类型一样
   auto  func2 = std::bind(plus,1,2);   //表示绑定函数 plus 的第一，二为： 1， 2 
   cout<<func1(1,2)<<endl; //3
   cout<<func2()<<endl; //3
   retunrn 0;
}
```

### 3	绑定类的成员函数

```c++
#include<iostream>
#include<functional>
using namespace std;
class Plus
{
   public:
   	int plus(int a,int b)
   	{
   	    return a+b;
   	}
}
int main()
{
   Plus p;
   // 指针形式调用成员函数
   function<int(int,int)> func1 = std::bind(&Plus::plus,&p, placeholders::_1, placeholders::_2);
  // 对象形式调用成员函数
   function<int(int,int)> func2 = std::bind(&Plus::plus,p, placeholders::_1, placeholders::_2);
   cout<<func1(1,2)<<endl; //3
   cout<<func2(1,2)<<endl; //3
   retunrn 0;
}

//占位符_1位于placeholders的命名空间，而placeholders位于std的命名空间中
```

### 3	绑定类的静态成员函数

```c++
#include<iostream>
#include<functional>
using namespace std;
class Plus
{
	public:
		static int plus(int a,int b)
		{
		    return a+b;
		}
}
int main()
{
   function<int(int,int)> func1 = std::bind(&Plus::plus, placeholders::_1, placeholders::_2);
   cout<<func1(1,2)<<endl; //3
   retunrn 0;
}
```

## 仿函数

### 1	定义

仿函数（Functor）又称为**函数对象（Function Object）**是一个能行使函数功能的类。在 C++ 中，可以重载operator ()来实现函子，调用仿函数，实际上就是通过类对象调用重载后的 operator() 运算符。

### 2	sample

仿函数是一个对象，因此它具有成员变量和成员函数。普通函数如下

```c++
class StringAppend {
public:
    explicit StringAppend(const string& str) : ss(str){}
    void operator() (const string& str) const {
         cout << str << ' ' << ss << endl;
    }
private:
    const string ss;
};

int main() {
    StringAppend myFunctor2("and world!"); //初始化ss
    myFunctor2("Hello"); //调用重载后的 (),实现调用成员函数的目的
}

//输出： Hello and world!
```



## lambda

表达式Lambda 提供了一种更简单的方法来编写仿函数。它是匿名函数的语法糖。它减少了我们需要在仿函数中编写的样板。

### 1	定义

```c++
表达式Lambda 提供了一种更简单的方法来编写函子。它是匿名函子的语法糖。它减少了我们需要在仿函数中编写的样板。
要指定返回值类型才加 ->
// 一般形式,加上mutable才能对传入的值进行修改
[capture] (params) mutable exception-> return type { function body }

// 常量形式，即不可更改捕获列表中的值
[capture list] (params list) -> return type {function body}

// 根据函数体推断返回类型
[capture list] (params list) {function body}

// 无参函数
[capture list] {function body}

capture：捕获作用域内变量，后面详述
params：传入参数
mutable：可变的，用来说明可以修改捕获的变量
exception：异常
return type：返回类型
function body：函数体
```

### 2	sample

```c++
struct DataPool {
    int a_;
    int b_;

    DataPool(int a, int b) : a_(a), b_(b) {}

    friend ostream &operator<<(ostream &output,
                               const DataPool &D) {
        output << "a_ : " << D.a_ << " b_ : " << D.b_;
        return output;
    }

};

bool sortMyself(const DataPool &x, const DataPool &y) {
    return x.b_ > y.b_;

}

int main() {
    vector<DataPool> vec;
    vec.push_back(DataPool(1, 1));
    vec.push_back(DataPool(2, 2));
    vec.push_back(DataPool(3, 3));
    vec.push_back(DataPool(4, 4));

//    std::sort(vec.begin(), vec.end(), sortMyself); //利用回调函数实现
	
    //通过lambda表达式实现
    std::sort(vec.begin(), vec.end(), [](const DataPool &a, const DataPool &b) -> bool {
        return a.b_ > b.b_;
    });
    return 0;
}
```

```c++
std::function<int(int)>  f1 = [](int a){ return a; };
std::function<int(void)> f2 = std::bind([](int a){ return a; }, 123);
```



## 回调函数

> 回调函数就是一个**通过函数指针调用的函数**。如果你把函数的指针（地址）作为参数传递给另一个函数，当这个指针被用来调用其所指向的函数时，我们就说这是回调函数。回调函数不是由该函数的实现方直接调用，而是在特定的事件或条件发生时由另外的一方调用的，用于对该事件或条件进行响应。 
>
> 
>
> **回调函数是做为参数传递的一种函数**，在早期C样式编程当中，回调函数必须依赖函数指针来实现。
>
> 而后的C++语言当中，又引入了 std::function 与 std::bind 来配合进行回调函数实现。

### 1	回调机制

**什么是回调函数？**

编程分为两类：系统编程（system programming）和应用编程（application programming）。所谓系统编程，简单来说，就是编写**库**；而应用编程就是利用写好的各种库来编写具某种功用的程序，也就是**应用**。系统程序员会给自己写的库留下一些接口，即API（application programming interface，应用编程接口），以供应用程序员使用。所以在抽象层的图示里，库位于应用的底下。

当程序跑起来时，一般情况下，[应用程序](https://www.zhihu.com/search?q=应用程序&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"answer"%2C"sourceId"%3A27459821})（application program）会时常通过API调用库里所预先备好的函数。但是有些库函数（library function）却要求应用先传给它一个函数，好在合适的时候调用，以完成目标任务。这个被传入的、后又被调用的函数就称为**回调函数**（callback function）。

**回调机制**

1. 定义一个函数（普通函数即可）； 

2. 将此函数的地址注册给调用者； 

3. 特定的事件或条件发生时，调用者使用函数指针调用回调函数。 

   注：必须要注意的是，**实现函数的类型必须要和函数指针的类型声明一致，也就是返回值和参数表（个数、类型）要完全一致。**

### 2	sample

```c++
int add(int a, int b) {
    return a + b;
}

int sub(int a, int b) {
    return a - b;
}

void func(int a, int b, int(*ptr)(int c, int d)) { //通过函数指针实现回调
    cout << ptr(a, b) << endl;
}

int main() {
    func(2, 3, add);//通过指定特定的方法对事件做出相应
    func(5, 9, sub);
}
```



