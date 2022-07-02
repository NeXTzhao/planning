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



## item 40：明智而谨慎的使用多重继承

略过(基本不会用到多重继承)



# 7 模板与泛型编程

## item 41：了解隐式接口与编译期多态

### class和面向对象编程

- 显式接口(explicit interfaces)

  在源码中可见，即接口在其对应的头文件中明确可见

- 运行期多态(runtime polymorphism)

  某些是virtual的成员函数，在运行中确定接口，可能是基类的，也可能是派生类的接口

### Template和泛型编程

- 隐式接口(implicit interfaces)
  - 模板函数要想有效编译，必须支持一组隐式接口
- 编译期多态(compile-time polymorphism)
  - 以不同的template参数实例化函数模板从而调用不同的函数
  - 多态通过template实例化和函数解析重载发生于编译期



## item 42：了解typename的双重意义

> 首先明确一点template声明中，class和typename没有区别

思考一下如下情形

```c++
template<typename C>
void print2nd(const C& container){
		C::const_iterator* x;
}
```

问：x是一个什么类型？

​	local变量？or 指针？

能得出上述结论的前提是，我们已经知道`C::const_iterator`是个类型，但如果他不是个类型，而恰巧有个static的成员变量声明为`const_iterator`或是其他global名称，上述的推论就是错的，所以引入下面的结论。

### template中的嵌套从属名称不是类型

`C::const_iterator* x;`的本意应该是声明一个x局部变量，他是一个指针，指向一个`C::const_iterator* x;`，想要达到此目的，只需要在前面加上`typename`即可，即

```c++
template<typename C>
void print2nd(const C& container){
		typename C::const_iterator* x;
}
```

### typename在嵌套从属类型名称使用规则

1. typename只被用来验明嵌套从属类名名称，其他名称不应该存在

   ```c++
   template<typename C>			//允许使用class或者typename
   void print2nd(const C& container){ // 不允许使用typename
   		typename C::const_iterator* x; //必须使用typename
   }
   ```

2. typename不可以出现在`base classes list`内的嵌套从属类型之前，也不能出现在`member initialization list`(成员初始列)中作为base list修饰符

   ```c++
   template<typename C>			
   class Derived : public base<C>::Nested{ //base classes list中不允许出现typename
     public:
     	explicit Derived(int x):Base<C>::Nested(x){ //成员初始列中不允许出现typename
         typename Base<C>::Nested temp; //既不是base也不是member，则必须加上typename
         ...
       }
   }
   ```

### typedef简化类型(仅仅是少打几行代码的作用)

```c++
template<typename IterT>
void workWithIterator(IterT iter){
	typedef typename std::iterator_traits<IterT>::value_type value_type;
	value_type temp(*iter);
}
```



## item 43：学习处理模板化基类内的名称

```c++
class a{
public:
    void fun() const{
        std::cout << "this is class a" << std::endl;
    }
};

template<typename T>
class collect {
public:
    void print() const {
        std::cout << "print select" << std::endl;
    }
};

template<typename T>
class animal : public collect<T>{
public:
    void eat(const a & info) {
        T t;
        info.fun();
        //t->print();
        this->print();
    }
};


int main() {
    animal<a> an;
    a A;
    an.eat(A);
}
```

### 解决c++“不进入 templatized base classes观察”的行为失效

1. 在base class 函数调用前加上`this->`

2. 使用using声明(扩大搜索范围)

3. 明确指出被调用的函数位于base class

   这种方法有一个明确的坏处，如果调用者是virtual函数，这种方法会关闭“virtual”的动态绑定行为



## item 44：将与参数无关的代码抽离templates



## item 45：运用成员函数模板接受所有兼容类型

### 同一个template的不同实例化之间没有联系

> 这里意指如果以带有base-derived关系的B，D两类型分别实例化某个template，产生出来的两个实例并不带有base-derived关系

对于传统的classes，以下案例可以顺利通过编译：

```c++
class t1 {};
class t2 : public t1 {};
class t3 : public t2 {};

int main(){
    t1* a = new t2;
    t1* b = new t3;  //t1 和 t2、t3满足“is-a”的关系
    const auto c = a;
}
```

而对于template却不一定行

```c++
class t1 {};
class t2 : public t1 {};
class t3 : public t2 {};

template<typename T>
class SmartPtr{
public:
    explicit SmartPtr(T* realPtr){}
};

int main(){
    SmartPtr<t1> a1 = SmartPtr<t2>(new t2);  //编译不通过

}
```

编译器会视`SmartPtr<t1> ` 和 `SmartPtr<t2>`为两个完全不同的classes

### 成员函数模板(member function template)

> 成员函数模板的作用就是为class生成函数

#### 泛化copy构造函数

在上述智能指针实例中，每一个语句都会创建一个智能指针对象，所以现在应该关注如何编写智能指针的构造函数，使其行为能够满足我们转型需要，简单来说就是**通过派生类自动构造一个基类**，这里就要用到**成员函数模板**，如下所示

```c++
template<typename T>
class SmartPtr {
public:
    template<typename U>                  // member template,
    SmartPtr(const SmartPtr<U>& other);   // 未来生成copy构造函数
    ...
};
int main(){
   SmartPtr<t1> a1 = SmartPtr<t2>(new t2); //这样的操作就是正确的 
}
```

上述代码表现为：对任一类型的T和U，都可以根据`SmartPtr<U>`生成一个`SmartPtr<T>`，因为`SmartPtr<T>`有个构造函数接受一个`SmartPtr<U>`参数。这一类构造函数根据对象u创建对象t，而u和t的类型是同一个template的不同具现体，有时这种做法称之为**泛化copy构造函数**

> 上面的泛化copy构造函数并未被声明为explicit。那是蓄意的，因为原始指针类型之间的转换（例如从derived class指针转为base class指针）是隐式转换，无需明白写出转型动作（cast），所以让智能指针效仿这种行径也属合理。在模板化构造函数中略去explicit就是为了这个目的。

**问题来了**

希望根据`SmartPtr<Bottom>`创建一个`SmartPtr<Top>`，却不希望根据一个`SmartPtr<Top>`创建一个`SmartPtr<Bottom>`，所以要对成员函数进行筛选

**解决办法：**

使用成员初值列来初始化`SmartPtr<T>`之内类型为T*的成员变量，并以类型为U*的指针（由`SmartPtr<U>`持有）作为初值。这个行为只有当“**存在某个隐式转换可将一个U指针转为一个T指针**”时才能通过编译，而那正是我们想要的。最终效果是`SmartPtr<T>`现在有了一个泛化copy构造函数，这个构造函数只在其所获得的实参隶属适当（兼容）类型时才通过编译。

#### 成员函数模板的使用不局限于构造函数，还时常扮演赋值操作

```c++
template<class T>
class shared_ptr {
public:
    template <class Y>                 
     explicit shared_ptr(Y* p);                        // 构造，来自任何兼容的内置指针
    template <class Y>
     shared_ptr(shared_ptr<Y> const& r);               // 或shared_ptr
    template <class Y>
     explicit shared_ptr(weak_ptr<Y> const& r);        // 或weak_ptr
    template <class Y>
     explicit shared_ptr(auto_ptr<Y>& r);              // 或auto_ptr
    template <class Y>                                 
     shared_ptr& operator=(shared_ptr<Y> const& r);    // 赋值，来自任何兼容的shared_ptr
    template <class Y>
     shared_ptr& operator=(auto_ptr<Y>& r);            // 或auto_ptr 
    ...
};
```

​        **上述所有构造函数都是explicit，唯有“泛化copy构造函数”除外**。那意味从某个shared_ptr类型隐式转换至另一个shared_ptr类型是被允许的，但从某个内置指针或从其他智能指针类型进行隐式转换则不被认可（如果是显示转换和cast强制转型动作倒是可以）。另一个趣味点是传递给tr1::shared_ptr构造函数和assignment操作符的auto_ptrs并未被声明为const，与之形成对比的则是tr1::shared_ptrs和tr1::weak_ptrs都以const传递。

#### 声明member templates用于“泛化copy构造”或“泛化assignment操作”你还是需要声明正常的copy构造函数和copy assignment操作符

> member templates并不改变语音规则，而语言规则说，如果程序需要一个copy构造函数，你却没有声明它，编译器会为你暗自生成一个。在class内声明泛化copy构造函数并不会阻止编译器生成它们自己的copy构造函数，所以如果你想要控制copy构造函数的方方面面，你必须同时声明泛化copy构造函数和“正常的”copy构造函数。相同规则也适用于赋值操作。

```c++
template<class T>
class shared_ptr {
public:
    shared_ptr(shared_ptr const& r);    // copy构造函数
 
    template <class Y>
     shared_ptr(shared_ptr<Y> const& r);               // 泛化copy构造函数
                
     shared_ptr& operator=(shared_ptr<Y> const& r);    // copy assignment
    template <class Y>
     shared_ptr& operator=(shared_ptr<Y> const& );     // 泛化copy assignment
    ...
};
```







