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
2. 如果子类作用域中没有找到相关的函数或变量，便会到外围作用域中查找，如果一直没有找到，最后回到`global`中查找

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



## item 35：考虑virtual函数以外的其他选择

### 1 由Non-Virtual Interface 手法实现 *Template Method* 模式

### 2 由Function Pointers实现 *Strategy* 模式

### 3 由`tr1::function` 完成 *Strategy* 模式

### 4 古典 *Strategy* 模式

## item 36：绝不重新定义继承而来的non-virtual函数

- `non-virtual` 函数在继承体系中例如 `A::fun` 和 `B:fun` 都是静态绑定，子类重新定义父类中的`non-virtual` 函数，子类中会自动屏蔽掉父类中的同名函数
- `virtual` 属于动态绑定，在运行期决定自身所绑定的对象

```c++
class A{
public:
     void fun(){
        std::cout << "this is A fun"<<'\n';
    }
    virtual void fun1(){
        std::cout << "this is A fun1"<<'\n';
    }
};

class B: public A{
public:
    void fun(){
        std::cout << "this is B fun"<< '\n';
    }
    void fun1(){
        std::cout << "this is B fun1"<<'\n';
    }
};

using namespace std;
int main() {
    B b;
    A* pa = &b;
    pa->fun(); //this is A fun
    pa->fun1(); //this is B fun1

    B* pb = &b;
    pb->fun(); //this is B fun
    pb->fun1();//this is B fun1
}

//输出
this is A fun
this is B fun1
this is B fun
this is B fun1
```



## item 37：绝不重新定义继承而来的缺省参数值

**`virtual` 函数为动态绑定，而缺省参数值为静态绑定**

### 1 何为动态绑定？

运行时所指定的对象，例如`A* a = new B;` 运行过程中确定a的类型为B

### 2 何为静态绑定

声明时确定的类型，例如`A* a = new B;` 声明中a的静态类型为A

### 3 如果重新定义了继承而来的缺省参数值，会出现什么问题？

父类为 `Shape`,子类为 `Rectangle`和 `Circle`

尽管子类中重写了父类中的基函数，但是子类中的缺省参数不管如何定义，它任然是父类中定义好的缺省参数，也就缺省参数为静态绑定的性质

```c++
class Shape {
public:
    enum ShapeColor {
        Red, Green, Blue
    };
    virtual void draw(ShapeColor color = Blue) const {
        std::cout << "shape:" << color << '\n';
    }
};

class Rectangle : public Shape {
public:
    virtual void draw(ShapeColor color = Green) const {
        std::cout << "Rectangle:"<< color << '\n';
    }
};

class Circle : public Shape {
public:
    virtual void draw(ShapeColor color) const {
        std::cout << "Circle:" << color << '\n';
    }
};

int main() {
    Shape *ps;
    Shape *pc = new Circle;
    Shape *pr = new Rectangle;

    ps = pc;
    ps->draw();
    ps = pr;
    ps->draw();
}
//输出
Circle:2 
Rectangle:2
```

### 4 如何解决上述问题？

1. 借鉴item 35中的`virtual` 函数替代方案：令父类中的普通函数去调用私有的虚函数，而父类的私有虚函数在子类中重新定义.
2. 一个成员函数被定义为private属性，标志着其只能被当前类的其他成员函数(或友元函数)所访问。而virtual修饰符则强调父类的成员函数可以在子类中被重写，因为重写之时并没有与父类发生任何的调用关系，故而重写是被允许的。

> 1. 编译器不检查虚函数的各类属性。被virtual修饰的成员函数，不论他们是private、protect或是public的，都会被统一的放置到虚函数表中。对父类进行派生时，子类会继承到拥有相同偏移地址的虚函数表（相同偏移地址指，各虚函数相对于VPTR指针的偏移），则子类就会被允许对这些虚函数进行重载。且重载时可以给重载函数定义新的属性，例如public，其只标志着该重载函数在该子类中的访问属性为public，和父类的private属性没有任何关系！
> 2. 缺省参数是声明或定义函数时为函数的参数指定一个默认值。在调用该函数时，如果没有指定实参则采用该默认值，否则使用指定的实参

```c++
class Shape {
public:
    enum ShapeColor {
        Red, Green, Blue
    };

    void doDraw(ShapeColor color = Blue) const{
        draw(color);
    }
private:
    virtual void draw(ShapeColor color) const = 0;
};

class Rectangle : public Shape {
private:
    virtual void draw(ShapeColor color) const {
        std::cout << "Rectangle:"<< color << '\n';
    }
};

class Circle : public Shape {
private:
    virtual void draw(ShapeColor color) const {
        std::cout << "Circle:" << color << '\n';
    }

};

using namespace std;

int main() {
    Shape *ps;
    Shape *pc = new Circle;
    Shape *pr = new Rectangle;

    ps = pc;
    ps->doDraw(Shape::Red);
    ps = pr;
    ps->doDraw(Shape::Green);
}
//输出
Circle:0
Rectangle:1
```



## item 38：通过复合塑膜出has-a或“根据某物实现出”

### 1 复合是什么？

复合(composition)是类型之间的一种关系，当某种类型的对象内部含有其他类型的对象，便是复合关系，例如

```c++
class Address {...};

class person{
public:
  ...
private:
  string name;
  Address add;	//一个对象中含有其他类型的对象，是一种复合关系
}
```

> 注意：复合跟public继承有本质区别，public变现为“is-a”的关系，而复合的其中一个变现为“has-a”，你可以说一个人有一个家庭住址，但不能说一个人是一个家庭住址

### 2 复合的表现？

- 应用领域

  表现为"has-a"的关系，类似于我是个人，我有一个家庭住址

- 实现领域

  表现为“is-implemented-in-terms-of”（**根据某物实现出**），例如

  ```c++
  想实现一个Set接口，但我不必去自己造轮子，可以用STL中的list去帮我实现我所需要的功能
  
  template<class T>
  class Set{
    public:
    	...
    private:
    	std::list<T> rep; 		//用来表述Set的数据
  };
  ```



## item 39：明智而谨慎的使用private继承

```
，应为复合关系中二者并没有实际的逻辑关联
```

### 1 案例

```c++
class person{...};
class student: private person{..};

void eat(const person& p);

int main(){
  person p;
  student s;
  
  eat(p);		//yes
  eat(s);		//no,为什么出错，见下文分析
}
```

### 2 private继承的特性

1. 如果class之间的继承关系是private的，那么编译器将不会自动生成一个子类对象，也就是说不会出现像public继承那样，将派生类自动转换为基类，所以eat(s)会报错
2. 由private继承而来的所有成员，在derived class中都会变成private属性，即使他们在原来的base class中是protected或public

### 3 private继承的用意

当我们只想从别的类中使用到他的某些特性，就将其声明为private，他只是一种实现技术，这就是为什么private继承过来的成员在自身的class内都是private的

### 4 复合 vs private继承

上一篇提到，复合的其中之一表现是"根据某物实现出"，即调用其他的对象来完成自身的功能，那么和private的继承便有些类似，问题在于如何在这两种方法之间做取舍呢？

答案：**尽可能使用复合，必要时才使用private继承**(什么时候是必要？几乎没有什么时候是必要。。。)

### 5 用复合来实现private继承

private继承：

```c++
class Timer{
public:
	explicit Timer(int tickFrequency);
	virtual void onTick() const;
};

class widget : private Timer{
private:
	virtual void onTick() const;
	...
};

这样有一个坏处在于：
  如果widget有一个子类，那他的子类并不能使用 onTick()函数(因为pri)
```

用复合实现private继承：

```c++
class widget{
private:
	class widgetTimer:public Timer{
	public:
		virtual void onTick()const;
		...
	};
	WidgetTimer timer;
	...
};
```

### 6 private继承可以造成empty base最优化(优点)

没太懂
