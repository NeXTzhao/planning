# Rust介绍

## 基础语法

### 变量 & 常量

```rust
变量的绑定 	let a = 5;
变量的可变性 let mut a = 5;
					  a = 6;
常量	const MAX_POINTS: u32 = 1000;
```

### 变量遮蔽

```rust
fn main() {
    let x = 5;
    // 在main函数的作用域内对之前的x进行遮蔽
    let x = x + 1;

    {
        // 在当前的花括号作用域内，对之前的x进行遮蔽
        let x = x * 2;
    }
}
```

### 基本类型

#### 整型

| 长度        | 有符号类型 | 无符号类型 |
| ----------- | ---------- | ---------- |
| 8 位        | `i8`       | `u8`       |
| 16 位       | `i16`      | `u16`      |
| 32 位(默认) | `i32`      | `u32`      |
| 64 位       | `i64`      | `u64`      |
| 128 位      | `i128`     | `u128`     |
| 视架构而定  | `isize`    | `usize`    |

#### 浮点

| 32位 | `f32` |
| ---- | ----- |
| 64位 | `f64` |

```rust 
let a = 1.0; //默认是f64
let b : f32 = 3.0;
```

#### 复合类型

> Rust 有两个原生的复合类型：元组（tuple）和数组（array）

##### 元组

```rust
fn main() {
    let tup = (500, 6.4, 1); // 绑定
    let (x, y, z) = tup; // 解构
 		
  or
    let five_hundred = tup.0;
    let six_point_four = tup.1;
    let one = tup.2;
}
```

程序首先创建了一个元组并绑定到 `tup` 变量上。接着使用了 `let` 和一个模式将 `tup` 分成了三个不同的变量，`x`、`y` 和 `z`。这叫做 **解构**（destructuring），因为它将一个元组拆成了三个部分

##### 数组

> 程序在索引操作中使用一个无效的值时导致 **编译时** 错误。

```rust
fn main() {
    let a = [1, 2, 3, 4, 5];

    let first = a[0];
    let second = a[1];
}
```

#### 序列

> 序列只允许用于数字或字符类型

```rust
1..5 ==> [1,5)
1..=5 ==> [1,5]

for i in 1..=5{
 println!("{}",i);
}

for i in 'a'..='z' {
    println!("{}",i);
}
```

#### 函数

> Rust 不关心函数定义于何处，只要定义了就行

### 语句与表达式

语句没有返回值

表达式有返回值

```rust
fn add_with_extra(x: i32, y: i32) -> i32 {
    let x = x + 1; // 语句
    let y = y + 5; // 语句
    x + y // 表达式
}
```

## 所有权

### 管理运行时计算机内存的方式

1. **语言中具有垃圾回收机制(*GC*)**，在程序运行时不断地寻找不再使用的内存
2. **手动分配和释放内存**，在程序中，通过函数调用的方式来申请和释放内存
3. **通过所有权管理内存**，编译器在**编译**时会根据一系列规则进行检查

#### 来自C语言的案例

```c++
int* foo() {
    int a;          // 变量a的作用域开始
    a = 100;
    char *c = "xyz";   // 变量c的作用域开始
    return &a;
}                   // 变量a和c的作用域结束
```

问题：

1. 局部变量 `a` 存在栈中，在离开作用域后，`a` 所申请的栈上内存都会被系统回收，从而造成了 `悬空指针(Dangling Pointer)` 的问题
2. `c` 的值是常量字符串，存储于常量区，可能这个函数我们只调用了一次，也可能我们不再会使用这个字符串，但 `"xyz"` 只有当整个程序结束后系统才能回收这片内存

### 所有权规则

1. Rust 中的每一个值都有一个被称为其 **所有者**（*owner*）的变量。
2. 值在任一时刻有且只有一个所有者。
3. 当所有者（变量）离开作用域，这个值将被丢弃。

### **转移所有权**

> 基本数据类型都是通过**自动拷贝**的方式来进行赋值
>

```rust
let x = 5;
let y = x;

println!("x,y:::{},{}",x,y); 
//输出：x,y:::5,5
```



当 `s1` 赋予 `s2` 后，Rust 认为 `s1` 不再有效，因此也无需在 `s1` 离开作用域后 `drop` 任何东西，这就是把所有权从 `s1` 转移给了 `s2`，`s1` 在被赋予 `s2` 后就马上失效了。

```rust
let s1 = String::from("hello");
let s2 = s1; // 问题解决方式：let s2 = s1.clone();

println!("{}, world!", s1);

//报错
3 |     let s2 = s1;
  |         -- value moved here
4 |
5 |     println!("{}, world!", s1);
  |                            ^^ value used here after move
```

违反了：**一个值只允许有一个所有者**，而现在这个值（堆上的真实字符串数据）有了两个所有者：`s1` 和 `s2`，当变量离开作用域后，Rust 会自动调用 `drop` 函数并清理变量的堆内存。不过由于两个 `String` 变量指向了同一位置。这就有了一个问题：当 `s1` 和 `s2` 离开作用域，它们都会尝试释放相同的内存。这是一个叫做 **二次释放（double free）** 的错误，也是之前提到过的内存安全性 BUG 之一

<img src="/home/next/routing_planning/Notes/Typora Notes/rust/jpg/v2-3ec77951de6a17584b5eb4a3838b4b61_1440w.jpg" alt="v2-3ec77951de6a17584b5eb4a3838b4b61_1440w" style="zoom: 33%;" />

小结：

Rust中基本类型的赋值操作表现出copy的行为，复合类型的赋值操作表现出move的行为，如果想让复合类型的赋值操作表现出copy的行为，就要显式的调用clone

#### 可变引用和不可变引用

```rust
// 不可变引用
fn main() {
    let s1 = String::from("hello");

    let len = calculate_length(&s1);

    println!("The length of '{}' is {}.", s1, len);
}

fn calculate_length(s: &String) -> usize {
    s.len()
}

// 可变引用
fn main() {
    let mut s = String::from("hello");

    change(&mut s);
}

fn change(some_string: &mut String) {
    some_string.push_str(", world");
}
```

#### 可变引用同时只能存在一个

不过可变引用并不是随心所欲、想用就用的，它有一个很大的限制： **同一作用域，特定数据只能有一个可变引用**：

```rust
let mut s = String::from("hello");

let r1 = &mut s;
let r2 = &mut s;

println!("{}, {}", r1, r2);
```

以上代码会报错：

```rust
error[E0499]: cannot borrow `s` as mutable more than once at a time 同一时间无法对 `s` 进行两次可变借用
 --> src/main.rs:5:14
  |
4 |     let r1 = &mut s;
  |              ------ first mutable borrow occurs here 首个可变引用在这里借用
5 |     let r2 = &mut s;
  |              ^^^^^^ second mutable borrow occurs here 第二个可变引用在这里借用
6 |
7 |     println!("{}, {}", r1, r2);
  |                        -- first borrow later used here 第一个借用在这里使用
```

这种限制的好处就是使 **Rust 在编译期就避免数据竞争**，数据竞争可由以下行为造成：

- 两个或更多的指针同时访问同一数据
- 至少有一个指针被用来写入数据
- 没有同步数据访问的机制

#### 可变引用与不可变引用不能同时存在

```rust
let mut s = String::from("hello");

let r1 = &s; // 没问题
let r2 = &s; // 没问题
let r3 = &mut s; // 大问题

println!("{}, {}, and {}", r1, r2, r3);
```

错误如下：

```rust
error[E0502]: cannot borrow `s` as mutable because it is also borrowed as immutable
        // 无法借用可变 `s` 因为它已经被借用了不可变
 --> src/main.rs:6:14
  |
4 |     let r1 = &s; // 没问题
  |              -- immutable borrow occurs here 不可变借用发生在这里
5 |     let r2 = &s; // 没问题
6 |     let r3 = &mut s; // 大问题
  |              ^^^^^^ mutable borrow occurs here 可变借用发生在这里
7 |
8 |     println!("{}, {}, and {}", r1, r2, r3);
  |                                -- immutable borrow later used here 不可变借用在这里使用
```

正在借用不可变引用的用户，肯定不希望他借用的东西，被另外一个人莫名其妙改变了。多个不可变借用被允许是因为没有人会去试图修改数据，每个人都只读这一份数据而不做修改，因此不用担心数据被污染

### 悬垂引用(Dangling References)

悬垂引用也叫做悬垂指针，意思为指针指向某个值后，这个值被释放掉了，而指针仍然存在，其指向的内存可能不存在任何值或已被其它变量重新使用。在 Rust 中编译器可以确保引用永远也不会变成悬垂状态：**当你拥有一些数据的引用，编译器可以确保数据不会在其引用之前被释放，要想释放数据，必须先停止其引用的使用**。

```rust
fn main() {
    let no = dangle();
}

fn dangle() -> &i32 {
    let s = 3;
     &s  
} // 这里 s 离开作用域并被丢弃。其内存被释放
```

编译报错：

```bash
error[E0106]: missing lifetime specifier
  --> src/main.rs:22:16
   |
22 | fn dangle() -> &i32 {
   |                ^ expected named lifetime parameter
   |
   = help: this function's return type contains a borrowed value, but there is no value for it to be borrowed from //找不到借用值的来源
help: consider using the `'static` lifetime
   |
22 | fn dangle() -> &'static i32 {
   |                ~~~~~~~~
```

解决办法：

```rust
fn main() {
    let no = foo();
}

fn foo() -> i32 {
    let a = 3;
     a  //将所有权移交出去  
}
```

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
    //方法定义在impl中，且没有self的函数称为关联函数
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

`self` 依然有所有权的概念：

- `self` 表示 `Rectangle` 的所有权转移到该方法中，这种形式用的较少
- `&self` 表示该方法对 `Rectangle` 的不可变借用
- `&mut self` 表示可变借用

总之，`self` 的使用就跟函数参数一样，要严格遵守 Rust 的所有权规则

### 泛型、*trait*与生命周期

- 泛型

  ```rust
  struct Point<X1, Y1> {
      x: X1,
      y: Y1,
  }
  
  impl<X1, Y1> Point<X1, Y1> {
      fn mixup<X2, Y2>(self, other: Point<X2, Y2>) -> Point<X1, Y2> {
          Point {
              x: self.x,
              y: other.y,
          }
      }
  }
  
  fn main() {
      let p1 = Point { x: 5, y: 10.4 };
      let p2 = Point { x: "Hello", y: 'c' };
  
      let p3 = p1.mixup(p2);
  
      println!("p3.x = {}, p3.y = {}", p3.x, p3.y);
  }
  ```

- trait

  > *trait* 类似于其他语言中的常被称为 **接口**（*interfaces*）的功能

  ```rust
  pub trait Summary {
      fn summarize(&self) -> String;
  }
  pub struct Post {
      pub title: String, // 标题
      pub author: String, // 作者
      pub content: String, // 内容
  }
  
  impl Summary for Post {
      fn summarize(&self) -> String {
          format!("文章{}, 作者是{}", self.title, self.author)
      }
  }
  fn main() {
      let post = Post{title: "Rust语言简介".to_string(),author: "Sunface".to_string(), content: "Rust棒极了!".to_string()};
  
      println!("{}",post.summarize());
  }
  ```

- 生命周期

  - 避免了悬垂引用

    ```rust
    {
      let r;
      {
        let x = 5;
        r = &x;
      }
      println!("r: {}", r);
    }
    ```
    
  
  运行错误：
  
  ```rust
    $ cargo run
       Compiling chapter10 v0.1.0 (file:///projects/chapter10)
    error[E0597]: `x` does not live long enough
      --> src/main.rs:7:17
       |
    7  |             r = &x;
       |                 ^^ borrowed value does not live long enough
    8  |         }
       |         - `x` dropped here while still borrowed
    9  | 
    10 |         println!("r: {}", r);
       |                           - borrow later used here
    
    For more information about this error, try `rustc --explain E0597`.
    error: could not compile `chapter10` due to previous error
  ```
  
- 借用检查器
  
  > Rust 编译器有一个 **借用检查器**（*borrow checker*），它比较作用域来确保所有的借用都是有效的
  
  ```rust
        {
            let r;                // ---------+-- 'a
                                  //          |
            {                     //          |
                let x = 5;        // -+-- 'b  |
                r = &x;           //  |       |
            }                     // -+       |
                                  //          |
            println!("r: {}", r); //          |
        }  
  ```
  
  这里将 `r` 的生命周期标记为 `'a` 并将 `x` 的生命周期标记为 `'b`。如你所见，内部的 `'b` 块要比外部的生命周期 `'a` 小得多。在编译时，Rust 比较这两个生命周期的大小，并发现 `r` 拥有生命周期 `'a`，不过它引用了一个拥有生命周期 `'b` 的对象。程序被拒绝编译，因为生命周期 `'b` 比生命周期 `'a` 要小：**被引用的对象比它的引用者存在的时间更短**。
  
- 函数中的泛型生命周期
  
  ```rust
    fn main() {
        let string1 = String::from("abcd");
        let string2 = "xyz";
    
        let result = longest(string1.as_str(), string2);
        println!("The longest string is {}", result);
    }
    
    fn longest(x: &str, y: &str) -> &str {
        if x.len() > y.len() {
            x
        } else {
            y
        }
    }
    // 编译报错
    9 | fn longest(x: &str, y: &str) -> &str {
      |               ----     ----     ^ expected named lifetime parameter
  ```
  
- 生命周期注解语法
  
  > 生命周期注解描述了多个引用生命周期相互的关系，而不影响其生命周期
  
  ```rust
    &i32        // 引用
    &'a i32     // 带有显式生命周期的引用
    &'a mut i32 // 带有显式生命周期的可变引用
  ```
  
  ```rust
    fn main() {
        let string1 = String::from("abcd");
        let string2 = "xyz";
    
        let result = longest(string1.as_str(), string2);
        println!("The longest string is {}", result);
    }
    
    fn longest<'a>(x: &'a str, y: &'a str) -> &'a str {
        if x.len() > y.len() {
            x
        } else {
            y
        }
    }
  ```
  
  它的实际含义是 `longest` 函数返回的引用的生命周期与传入该函数的引用的生命周期的较小者一致

## 返回值和错误处理

Rust 中的错误主要分为两类：

- **可恢复错误**，通常用于从系统全局角度来看可以接受的错误，例如处理用户的访问、操作等错误，这些错误只会影响某个用户自身的操作进程，而不会对系统的全局稳定性产生影响
- **不可恢复错误**，刚好相反，该错误通常是全局性或者系统性的错误，例如数组越界访问，系统启动时发生了影响启动流程的错误等等，这些错误的影响往往对于系统来说是致命的

很多编程语言，并不会区分这些错误，而是直接采用异常的方式去处理。Rust 没有异常，但是 Rust 也有自己的办法：`Result<T, E>` 用于可恢复错误，`panic!` 用于不可恢复错误。

### panic! 与不可恢复错误

> `panic!` 宏，当调用执行该宏时，**程序会打印出一个错误信息，展开报错点往前的函数调用堆栈，最后退出程序**

```rust
fn main() {
    let v = vec![1, 2, 3];
    v[99];
}
```

运行报错：

```rust
thread 'main' panicked at 'index out of bounds: the len is 3 but the index is 99', src/main.rs:4:5
note: run with `RUST_BACKTRACE=1` environment variable to display a backtrace
```

通过 `RUST_BACKTRACE=1 cargo run` 或 `RUST_BACKTRACE=full`来知道该问题之前经过了哪些调用环节

```rust
thread 'main' panicked at 'index out of bounds: the len is 3 but the index is 99', src/main.rs:4:5
stack backtrace:
   0: rust_begin_unwind
             at /rustc/59eed8a2aac0230a8b53e89d4e99d55912ba6b35/library/std/src/panicking.rs:517:5
   1: core::panicking::panic_fmt
             at /rustc/59eed8a2aac0230a8b53e89d4e99d55912ba6b35/library/core/src/panicking.rs:101:14
   2: core::panicking::panic_bounds_check
             at /rustc/59eed8a2aac0230a8b53e89d4e99d55912ba6b35/library/core/src/panicking.rs:77:5
   3: <usize as core::slice::index::SliceIndex<[T]>>::index
             at /rustc/59eed8a2aac0230a8b53e89d4e99d55912ba6b35/library/core/src/slice/index.rs:184:10
   4: core::slice::index::<impl core::ops::index::Index<I> for [T]>::index
             at /rustc/59eed8a2aac0230a8b53e89d4e99d55912ba6b35/library/core/src/slice/index.rs:15:9
   5: <alloc::vec::Vec<T,A> as core::ops::index::Index<I>>::index
             at /rustc/59eed8a2aac0230a8b53e89d4e99d55912ba6b35/library/alloc/src/vec/mod.rs:2465:9
   6: world_hello::main
             at ./src/main.rs:4:5
   7: core::ops::function::FnOnce::call_once
             at /rustc/59eed8a2aac0230a8b53e89d4e99d55912ba6b35/library/core/src/ops/function.rs:227:5
note: Some details are omitted, run with `RUST_BACKTRACE=full` for a verbose backtrace.

```

上面的代码就是一次**栈展开**(也称栈回溯)，它包含了函数调用的顺序

### 可恢复的错误 Result

`Result<T, E>` 是一个枚举类型，定义如下：

```rust
enum Result<T, E> {
    Ok(T),
    Err(E),
}
```

泛型参数 `T` 代表成功时存入的正确值的类型，存放方式是 `Ok(T)`，`E` 代表错误是存入的错误值，存放方式是 `Err(E)`

```rust
#![allow(unused)]
fn main() {
use std::fs::File;
use std::io::{self, Read};

fn read_username_from_file() -> Result<String, io::Error> {
    // 打开文件，f是`Result<文件句柄,io::Error>`
    let f = File::open("hello.txt");

    let mut f = match f {
        // 打开文件成功，将file句柄赋值给f
        Ok(file) => file,
        // 打开文件失败，将错误返回(向上传播)
        Err(e) => return Err(e),
    };
    // 创建动态字符串s
    let mut s = String::new();
    // 从f文件句柄读取数据并写入s中
    match f.read_to_string(&mut s) {
        // 读取成功，返回Ok封装的字符串
        Ok(_) => Ok(s),
        // 将错误向上传播
        Err(e) => Err(e),
    }
}
}
```

用`？`来简化上述代码

```rust
#![allow(unused)]
fn main() {
use std::fs::File;
use std::io;
use std::io::Read;

fn read_username_from_file() -> Result<String, io::Error> {
    let mut s = String::new();

    File::open("hello.txt")?.read_to_string(&mut s)?;

    Ok(s)
}
}
```

`?` 还能实现链式调用，`File::open` 遇到错误就返回，没有错误就将 `Ok` 中的值取出来用于下一个方法调用

## Cargo 包管理工具

做了四件事：

- 引入两个元数据文件，包含项目的方方面面信息: `Cargo.toml` 和 `Cargo.lock`
- 获取和构建项目的依赖，例如 `Cargo.toml` 中的依赖包版本描述，以及从 `crates.io` 下载包
  - [`crates.io`](https://crates.io/) 是 Rust 社区维护的中心化注册服务，用户可以在其中寻找和下载所需的包。对于 `cargo` 来说，默认就是从这里下载依赖
- 调用 `rustc` (或其它编译器) 并使用的正确的参数来构建项目，例如 `cargo build`
- 引入一些惯例，让项目的使用更加简单

```bash
cargo new hello_world

.
├── Cargo.toml
└── src
    └── main.rs

1 directory, 2 files
```

### Cargo.toml

```rust
[package]
name = "hello_world"
version = "0.1.0"
edition = "2021"

[dependencies]
rand = "*"
optimization_engine = "*"
libc = "*"
```

### Cargo.lock

> 无需手动去管理依赖库

```bash
[[package]]
name = "rand"
version = "0.8.5"
source = "registry+https://github.com/rust-lang/crates.io-index"
checksum = "34af8d1a0e25924bc5b7c43c079c942339d8f0a8b57c39049bef581b46327404"
dependencies = [
 "libc",
 "rand_chacha",
 "rand_core",
]
```

`Cargo.toml` 和 `Cargo.lock` 是 `Cargo` 的两个元配置文件，但是它们有不同的目的:

- 前者从用户的角度出发来描述项目信息和依赖管理，因此它是由用户来编写
- 后者包含了依赖的精确描述信息，它是由 `Cargo` 自行维护，因此不要去手动修改


## FFI(Foreign Function Interface)

## 调用外部函数

`crate lib`它为 C 类型提供各种类型定义

```rust
[dependencies]
libc = "0.2.0"
```

`lib.rs`

```rust
extern "C" {
    fn cos(input: f64) -> f64;
    fn sin(input: f64) -> f64;
    fn pow(input: f64, exp: f64) -> f64;
}

fn main() {
    let x = unsafe {sin(0.2) };
}
```

属性`no_mangle`，用来关闭 Rust 的名称修改（name mangling）功能

## 从C代码调用rust代码

1. `cargo new helllo_world --lib`

   ```rust
   #[no_mangle]
   pub extern "C" fn hello_from_rust() {
       println!("Hello from Rust!");
   }
   fn main() {}
   ```

2. 修改`Cargo.toml`

   ```rust
   [lib]
   crate-type = ["cdylib"] //staticlib
   ```

3. `Cargo build`生成xx.a//xx.so

4. 调用rust代码

   ```rust
   extern void hello_from_rust();
   
   int main(void) {
       hello_from_rust();
       return 0;
   }
   ```



# OpEn(Optimization Engine)求解器

<img src="/home/next/routing_planning/Notes/Typora Notes/rust/jpg/about-open.png" alt="about-open" style="zoom:67%;" />

![openbenchmark](/home/next/routing_planning/Notes/Typora Notes/rust/jpg/openbenchmark.png)

## 求解

OpEn解决以下形式的问题：
$$
\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&\ \ f(u, p)\\
\mathrm{subject\ to} &\ \ u \in U(p)\end{split}
$$

1. 定于变量 $u$
   $$
   u = [\theta_i,kappa_i,x_i,y_i,...,\theta_n,kappa_n,x_n,y_n,\Delta s_0...,\Delta s_{n-1}]_{5n-1}\\
   $$

   ```rust
   let u_size = 5 * size - 1;
   let mut u = vec![0.0; u_size];
   let variable_index = size * 4;
   for i in 0..size {
     let index = 4 * i;
     u[index] = theta[i];
     u[index + 1] = kappa[i];
     u[index + 2] = x[i];
     u[index + 3] = y[i];
     if i <= size - 2 {
     u[variable_index + i] = delta[i];
     }
   }
   ```

2. 定义 cost function
   $$
   cos function= \ \ \sum_{i}^{n-1}[(x_{i+1} - x_i-\int_{0}^{ s}\cos(\theta(s))ds)^2 + (y_{i+1} - y_i-\int_{0}^{ s}\sin(\theta(s))ds)^2]
   $$


   ```rust
   let xy_cost_function = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
     *c = err_x_y_cost(&u, size);
     Ok(())
   };
   ```

   第一个参数 $u$ 是函数的参数。第二个 $c$ 参数是对结果（成本）的可变引用。该函数返回该类型的状态码`Result<(), SolverError>`，该状态码`Ok(())`表示计算成功

2. 定义成本梯度 gradient of the cost 

   ```rust
   let xy_cost_function_grad = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError>{
     cost_function_grad(u, size, grad);
     Ok(())
   };
   ```
   
4. 定义自变量的约束 Constraints

   ![Screenshot from 2022-07-04 00-15-35](/home/next/routing_planning/Notes/Typora Notes/rust/jpg/Screenshot from 2022-07-04 00-15-35.png)

   ```rust
   let mut variate_lower_bound = vec![-f64::INFINITY; u_size];
   let mut variate_upper_bound = vec![f64::INFINITY; u_size];
   
   let fault_dis = 0.1;
   
   for i in 0..size {
     let index = i * 4;
     // x
     variate_lower_bound[index + 2] = u[index + 2] - fault_dis;
     variate_upper_bound[index + 2] = u[index + 2] + fault_dis;
     // y
     variate_lower_bound[index + 3] = u[index + 3] - fault_dis;
     variate_upper_bound[index + 3] = u[index + 3] + fault_dis;
   }
   
   let bounds = Rectangle::new(Some(&variate_lower_bound), Some(&variate_upper_bound));
   ```

5. 定义问题 Problems

   ```rust
   let problem = Problem::new(&bounds, xy_cost_function_grad, xy_cost_function);
   ```

6. 调用求解器

   OpEn使用两个步骤

   1.  **cache** 

      [`PANOCCache`](https://docs.rs/optimization_engine/*/optimization_engine/core/panoc/struct.PANOCCache.html)允许一个问题的多个实例拥有一个公共工作区，这样我们就不需要不必要地释放和重新分配内存。一旦我们需要解决类似的问题，缓存对象将被重用

      ```rust
      /// Tolerance of inner solver
      const EPSILON_TOLERANCE: f64 = 1e-03;
      
      /// LBFGS memory
      const LBFGS_MEMORY: usize = 350;
      
      /// Maximum number of iterations of the solver
      const MAX_ITERATIONS: usize = 500;
      
      let mut panoc_cache = PANOCCache::new(u_size, EPSILON_TOLERANCE, LBFGS_MEMORY);
      ```

   2. 调用优化器

      ```rust
      let mut panoc = PANOCOptimizer::new(problem, &mut			panoc_cache).with_max_iter(MAX_ITERATIONS);
      let status = panoc.solve(&mut u)?;
      
      println!("Panoc status: {:#?}", status);
      ```

最终求解结果：

```bash
Panoc status: SolverStatus {
    exit_status: Converged,
    num_iter: 5,
    solve_time: 317µs,
    fpr_norm: 0.0009076639882809229,
    cost_value: 6.399853744019397e-6,
}

final_xs.size = 62,ref_x.size = 62
```

![Figure_1](/home/next/routing_planning/Notes/Typora Notes/rust/jpg/Figure_1.png)







