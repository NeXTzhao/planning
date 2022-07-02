# Rust程序设计

## 所有权(ownership)

> 所有权（系统）是 Rust 最为与众不同的特性，它让 Rust 无需垃圾回收（garbage collector）即可保障内存安全。
>
> 跟踪哪部分代码正在使用堆上的哪些数据，最大限度的减少堆上的重复数据的数量，以及清理堆上不再使用的数据确保不会耗尽空间，这些问题正是所有权系统要处理的，所以**所有权的主要目的就是为了管理堆数据**

### **所有权规则**

1. Rust 中每一个值都被一个变量所拥有，该变量被称为值的所有者
2. 一个值同时只能被一个变量所拥有，或者说一个值只能拥有一个所有者
3. 当所有者(变量)离开作用域范围时，这个值将被丢弃(drop)

### **转移所有权**

基本数据类型都是通过自动拷贝的方式来进行赋值

```rust
let x = 5;
let y = x;

println!("x,y:::{},{}",x,y); 
//输出：x,y:::5,5
```

当 `s1` 赋予 `s2` 后，Rust 认为 `s1` 不再有效，因此也无需在 `s1` 离开作用域后 `drop` 任何东西，这就是把所有权从 `s1` 转移给了 `s2`，`s1` 在被赋予 `s2` 后就马上失效了。

```rust
let s1 = String::from("hello");
let s2 = s1;

println!("{}, world!", s1);

//报错
3 |     let s2 = s1;
  |         -- value moved here
4 |
5 |     println!("{}, world!", s1);
  |                            ^^ value used here after move
```

<img src="/home/next/routing_planning/Notes/Typora Notes/rust/jpg/v2-3ec77951de6a17584b5eb4a3838b4b61_1440w.jpg" alt="v2-3ec77951de6a17584b5eb4a3838b4b61_1440w" style="zoom: 33%;" />

### 克隆(深拷贝)

1. Rust 永远也不会自动创建数据的 “深拷贝”

2. 可以通过`clone`的方法来进行深拷贝，如果频繁调用会极大降低程序性能

   ```rust
   let s1 = String::from("hello");
   let s2 = s1.clone();
   
   println!("s1 = {}, s2 = {}", s1, s2);
   ```

### 拷贝(浅拷贝)

```rust
fn main() {
let x = 5;
let y = x;

println!("x = {}, y = {}", x, y);
}
//这段代码没有调用clone,也同样实现了类似深拷贝的效果
```

1. Rust 有一个叫做 `Copy` 的特征，可以用在类似整型这样在栈中存储的类型。如果一个类型拥有 `Copy` 特征，一个旧的变量在被赋值给其他变量后仍然可用。
2.  任何基本类型的组合可以 `Copy` ，不需要分配内存或某种形式资源的类型是可以 `Copy` 的。
   1. 所有整数类型，比如 `u32`。
   2. 布尔类型，`bool`，它的值是 `true` 和 `false`。
   3. 所有浮点数类型，比如 `f64`。
   4. 字符类型，`char`。
   5. 元组，当且仅当其包含的类型也都是 `Copy` 的时候。比如，`(i32, i32)` 是 `Copy` 的，但 `(i32, String)` 就不是。
   6. 不可变引用 `&T` ，**但是注意: 可变引用 `&mut T` 是不可以 Copy的**

### 函数传值与返回

1. 将值传递给函数，一样会发生 `移动` 或者 `复制`

   ```rust
   fn main() {
       let s = String::from("hello");  // s 进入作用域
   
       takes_ownership(s);             // s 的值移动到函数里 ...
                                       // ... 所以到这里不再有效
   
       let x = 5;                      // x 进入作用域
   
       makes_copy(x);                  // x 应该移动函数里，
                                       // 但 i32 是 Copy 的，所以在后面可继续使用 x
   
   } // 这里, x 先移出了作用域，然后是 s。但因为 s 的值已被移走，
     // 所以不会有特殊操作
   
   fn takes_ownership(some_string: String) { // some_string 进入作用域
       println!("{}", some_string);
   } // 这里，some_string 移出作用域并调用 `drop` 方法。占用的内存被释放
   
   fn makes_copy(some_integer: i32) { // some_integer 进入作用域
       println!("{}", some_integer);
   } // 这里，some_integer 移出作用域。不会有特殊操作
   ```

2. 函数返回值也有所有权

   ```rust
   fn main() {
       let s1 = gives_ownership();         // gives_ownership 将返回值
                                           // 移给 s1
   
       let s2 = String::from("hello");     // s2 进入作用域
   
       let s3 = takes_and_gives_back(s2);  // s2 被移动到
                                           // takes_and_gives_back 中,
                                           // 它也将返回值移给 s3
   } // 这里, s3 移出作用域并被丢弃。s2 也移出作用域，但已被移走，
     // 所以什么也不会发生。s1 移出作用域并被丢弃
   
   fn gives_ownership() -> String {             // gives_ownership 将返回值移动给
                                                // 调用它的函数
   
       let some_string = String::from("hello"); // some_string 进入作用域.
   
       some_string                              // 返回 some_string 并移出给调用的函数
   }
   
   // takes_and_gives_back 将传入字符串并返回该值
   fn takes_and_gives_back(a_string: String) -> String { // a_string 进入作用域
   
       a_string  // 返回 a_string 并移出给调用的函数
   }
   ```

## **引用与借用**

Rust 通过 `借用(Borrowing)` 这个概念来达成使用**某个变量的指针或者引用**目的，**获取变量的引用，称之为借用(borrowing)**。

### 引用与解引用

```rust
fn main() {
    let x = 5;
    let y = &x;

    assert_eq!(5, x);
    assert_eq!(5, *y);
}
```

### 不可变引用

```rust
fn main() {
    let s = String::from("hello");

    change(&s);
}

fn change(some_string: &String) {
    some_string.push_str(", world");
}
//报错：String是不可变引用
```

### 可变引用

首先，声明 `s` 是可变类型，其次创建一个可变的引用 `&mut s` 和接受可变引用参数 `some_string: &mut String` 的函数

```rust
fn main() {
    let mut s = String::from("hello");

    change(&mut s);
}

fn change(some_string: &mut String) {
    some_string.push_str(", world");
}
```

#### 可变引用同时只能存在一个

```rust
fn main() {
let mut s = String::from("hello");

let r1 = &mut s;
let r2 = &mut s;

println!("{}, {}", r1, r2);
}
//运行会报错
```

错误原因在于：

​		第一个可变借用 `r1` 必须要持续到最后一次使用的位置 `println!`，在 `r1` 创建和最后一次使用之间，我们又尝试创建第二个可变引用 `r2`，此时会发生**数据竞争**

数据竞争会由以下行为产生：

- 两个或更多的指针同时访问同一数据
- 至少有一个指针被用来写入数据
- 没有同步数据访问的机制

#### 可变引用与不可变引用不能同时存在

```rust

#![allow(unused)]
fn main() {
let mut s = String::from("hello");

let r1 = &s; // 没问题
let r2 = &s; // 没问题
let r3 = &mut s; // 大问题

println!("{}, {}, and {}", r1, r2, r3);
}
```

这样的限制好处在于：每个人都只读这一份数据而不做修改，因此不用担心数据被污染

#### NLL

对于这种编译器优化行为，Rust 专门起了一个名字 —— **Non-Lexical Lifetimes(NLL)**，专门用于找到某个引用在作用域(`}`)结束前就不再被使用的代码位置

### 悬垂引用(Dangling References)

悬垂引用也叫做悬垂指针，意思为指针指向某个值后，这个值被释放掉了，而指针仍然存在，其指向的内存可能不存在任何值或已被其它变量重新使用。在 Rust 中编译器可以确保引用永远也不会变成悬垂状态：当你拥有一些数据的引用，编译器可以确保数据不会在其引用之前被释放，要想释放数据，必须先停止其引用的使用。

## 方法 Method

Rust的方法通常跟结构体、枚举、特征(Trait)一起使用

### 定义方法

Rust中使用`impl`来定义方法

```Rust
#![allow(unused)]
fn main() {
struct Circle {
    x: f64,
    y: f64,
    radius: f64,
}

impl Circle {
    // new是Circle的关联函数，因为它的第一个参数不是self，且new并不是关键字
    // 这种方法往往用于初始化当前结构体的实例
    fn new(x: f64, y: f64, radius: f64) -> Circle {
        Circle {
            x: x,
            y: y,
            radius: radius,
        }
    }

    // Circle的方法，&self表示借用当前的Circle结构体
    fn area(&self) -> f64 {
        std::f64::consts::PI * (self.radius * self.radius)
    }
}
}
```

**Rust与其他语言在方法定义上的区别**

![v2-0d848e960f3279999eab4b1317f6538e_1440w](/home/next/routing_planning/Notes/Typora Notes/rust/jpg/v2-0d848e960f3279999eab4b1317f6538e_1440w.png)

### 方法名跟结构体字段名相同

```rust
pub struct Rectangle {
    width: u32,
    height: u32,
}

impl Rectangle {
    pub fn new(width: u32, height: u32) -> Self {
        Rectangle { width, height }
    }
    pub fn width(&self) -> u32 {
        return self.width;
    }
}
```

### 关联函数

方法定义在`impl`中，且没有`self`的函数称为**关联函数**

```rust

#![allow(unused)]
fn main() {
#[derive(Debug)]
struct Rectangle {
    width: u32,
    height: u32,
}

impl Rectangle {
    fn new(w: u32, h: u32) -> Rectangle {
        Rectangle { width: w, height: h }
    }
}
}

```

## 集合类型

### 动态数组Vector

- 创建动态数组

```rust
let v: Vec<i32> = Vec::new();
or 
let v = vec![1,2,3];
```

- 添加元素

```rust
let mut v = Vec::new(); //必须声明为mut
v.push(1);
```

- 从Vector中读取元素

  1. 下标索引

     ```rust
     let v = vec![1, 2, 3, 4, 5];
     
     let third: &i32 = &v[2];
     ```

  1. .get

     ```rust
     match v.get(2) {
         Some(third) => println!("第三个元素是 {}", third),
         None => println!("去你的第三个元素，根本没有！"),
     }
     ```

- 同时借用多个元素

  ```rust
  #![allow(unused)]
  fn main() {
  let mut v = vec![1, 2, 3, 4, 5];
  
  let first = &v[0];
  
  v.push(6);
  
  println!("The first element is: {}", first);
  }
  //这样编译将会报错,把println!("The first element is: {}", first);移到v.push(6)就不会报错
  ```

​		原因：引用的作用域 `s` 从创建开始，一直持续到它最后一次使用的地方，这个跟变量的作用域有所不同，变量的作用域从创建持续到某一个花括号

- 迭代遍历Vector中的元素

  ```rust
  fn main() {
  let mut v = vec![1, 2, 3];
  for i in &mut v {
      *i += 10
  }
  }
  ```

## 错误处理

- 可恢复类

   `Result<T, E>` 类型，用于处理可恢复的错误

- 不可恢复类

   `panic!` 宏，在程序遇到不可恢复的错误时停止执行

## 智能指针

