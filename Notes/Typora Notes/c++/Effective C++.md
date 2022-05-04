> 好记性不如烂笔头！！！

# 6 继承与面向对象设计

## item 32：确定你的public继承塑膜出 is-a 关系

```c++
class people{ };

class student : public people{ };

student is a people;
此说明子类是一个父类对象，父类的每一件事都适用于子类，子类相当于特殊化的父类。反之子类的方法父类可能不适用
```

此外，我们在设计时应当遵循"**在编译期发生错误**"，远比“**在运行期才检测到错误**”要好。



## item 33：避免遮掩继承而来的名称

> 偷走我的好名字的人呀......害我变得好可怜~

### 1 继承关系的作用域

```c++
class base{
private:
  int x;
public:
  virtual void fun() = 0;
	vitual void fun2();
};

class base : public drived{
public:
  vortual void fun();
  void fun3(){
    fun2();
  };
};

//上述继承关系的作用域表示为：
namespace base{
	int x;
  fun();
  
  namespace drived{
    fun();
    fun1();
  }
}
```

总结：

1. 子类继承父类，相当于把子类嵌入到父类当中
2. 如果子类作用域中没有找到相关的函数或变量，便会到外围作用域中查找，如果一直没有找到，最后回到global中查找

### 2 使用using声明式或转交函数

问题引入

```c++
class base{
public:
  virtual void fun();
  virtual void fun(int x);
};

class drived: public base{
public：
  using base::fun;
  virtual void fun();	
};

int main(){
  int x;
  drived de;
  de.fun();
  de.fun(x);
}
```

上述案例如果不加 `using base::fun` , 则会导致`de.fun(x)`报错，因为程序遵循以作用域为基础的“名称掩盖原则”，意思是子类作用域中的函数名会覆盖掉父类中的所有同名函数名(只要名称相同，就全部掩盖掉)，那么为了使用父类中重载的函数，就可以在父类中使用 `using`来声明某个函数，让其在子类的作用域内可见

### 3 转交函数(forwarding function)

```c++
class base{
public:
  virtual void fun();
  virtual void fun(int x);
};

class drived: public base{
public：
  virtual void fun(){
  	base::fun();	//1.通过这种方式实现，只指定使用某个特定的版本
  								//2.此处也体现了 inline 的作用
	}	
};
```

## item 34：区分接口继承和实现继承

public的继承由两部分组成：**函数接口继承**(function interfaces)和**函数实现**(function implementations)

```c++
class shape{
public:
	virtual void draw() const = 0; //pure virtual 纯虚函数
	virtual void error(const std::string& msg); // impure virtual
	int objectID() const; //non-virtual
};

class Ractangle: public shape{...};
```

### 1 纯虚函数

声明纯虚函数的目的是为了让`derived classes` 只继承函数的接口

意思是父类里面的定义有纯虚函数，那么在继承他的子类中必须有一个对这个纯虚函数的实现

```c++
shape * ps = new shape;  //这种定义是错误的，shape是抽象对象，不能被实例化
shape * ps1 = new Ractangle; //正确
```

- 父类的纯虚函数必须在子类中重新申明，但是在父类中也可以对纯虚函数有一份自己的声明

```c++
class shape{
public:
	virtual void draw() const = 0; //pure virtual 纯虚函数
	...
};
//父类有一份自己对纯虚函数的实现
void shape::draw(){
	std::cout << "this is base draw"<<std::endl;
}

class Race : public draw{
 public:
  	void draw(){
      shape::draw(); //直接调用父类的实现
    }
};
```

### 2 虚函数

声明 `impure virtual` 的目的是，让子类继承该函数的接口和缺省实现

```c++
void shape::error(const std::string& msg){
	std::cout << "this is error!"<<std::endl;
}
```

假如shape的error函数有如上定义，那么说明在子类中必须实现一个自己的error函数，但这不是强制性的，如果在子类中不想去实现，也可以调用父类中的虚函数error

```c++
Ractangle * ps1 = new Ractangle;
ps1->error(); //这样调用是正确的
```

虚函数中其声明部分变现为接口(**在子类中必须使用的**)，其定义部分则表现为缺省行为(**在子类中可以使用，但必须明确申请调用**)

### 3 non-virtual

`non-virtual` 函数声明是为了让子类继承函数的接口以及一份强制的实现

意思是这个函数不允许被重新定义，其不变性凌驾于特异性，例如每个类有个编号，这个编号不允许被修改

```c++
class shape{
public:
	...
	int objectID() const; //non-virtual
};

class Ractangle: public shape{...};

Ractangle * race = new Ractangle;
race->objectID();
```



