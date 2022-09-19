[TOC]

# `GNU C`编译器扩展语法

不同编译器，出于开发环境、硬件平台、性能优化的需要，除了支持C语言标准，还会自己做一些扩展。`GCC`编译器也对`C`语言标准做了很多扩展。

- 零长度数组。
- 语句表达式。
- 内建函数
- `__attribute__`特殊属性声明。
- 标号元素。
- `case`范围。
- ...

## 指定初始化

### 指定初始化数组元素

`C99`标准改进了数组的初始化方式:

- 支持指定元素初始化，不再按照固定的顺序初始化:

  ```c
  int array[100] = {[10] = 10, [30] = 200};
  ```

​		在早期C语言标准不支持指定初始化时，`GCC`编译器就已经支持指定初始化了，因此这个特性也被看作`GCC`编译器的一个扩展特性。

- 支持数组中某一个索引范围的元素初始化:

  ```c
  int array[100] = {[10 ... 30] = 1,  [50 ... 80]};
  ```

  `GNU C`支持`...`表示范围扩展，`...`也可以使用在`switch-case`语句中:

  ```c
  #include <stdio.h>
  int main(void) {
      int index = 4;
      switch(index) {
          case 1:
              break;
          case 2 ... 8:
              printf("%d\n", i);
              break;
          case 9:
              break;
          default:
              break;
      }
      return 0;
  }
  ```

  **`...`和其两端的数据范围`2`和`8`之间也要有空格，不能写成`2...8`的形式，否则会报编译错误。**

- 指定初始化结构体成员:在`GNU C`中可以通过结构域来指定初始化某个成员。

  ```c
  struct student {
      char name[20];
      int age;
  }
  int main(void) {
      struct student stu = {
      	.name = "caesar";
          .age = 18;
      };
      return 0;
  }
  ```

  在`Linux`内核中大量使用了这种初始化，比如`kvm`模块:

  ```c
  static struct kvm_x86_ops vmx_x86_ops __initdata = {
  	.name = "kvm_intel",
  
  	.hardware_unsetup = hardware_unsetup,
  
  	.hardware_enable = hardware_enable,
  	.hardware_disable = hardware_disable,
  	.cpu_has_accelerated_tpr = report_flexpriority,
  	.has_emulated_msr = vmx_has_emulated_msr,
  
  	.vm_size = sizeof(struct kvm_vmx),
  	.vm_init = vmx_vm_init,
  
  	.vcpu_create = vmx_create_vcpu,
  	.vcpu_free = vmx_free_vcpu,
  	.vcpu_reset = vmx_vcpu_reset,
      ...};
  ```

## 语句表达式

`GNU C`对`C`语言标准作了扩展，允许在一个表达式里内嵌语句，允许在表达式内部使用局部变量、`for`循环和`goto`跳转语句。这种类型的语句表达式称之为语句表达式。

```c
({表达式1; 表达式2; 表达式3;})
```

语句表达式的值为内嵌语句中最后一个表达式的值:如下`sum`的值为`45`。

```c
int sum = 0;
sum = ({
    int s = 0;
    for (int i = 0; i < 10; i++)
        s = s + i;
    s;
});
```

### 在宏定义中使用语句表达式

使用语句表达式和宏定义来求两个数的最大值:

```c
#define max(x, y) ({	\
	typeof(x) _x = (x); \
	typeof(y) _y = (y);	\
	(void) (&_x == &_y);\
	_x > _y ? _x:_y;
})
```

- `typeof`是`GNU C`新曾的关键字，用来获取数据类型。

- 定义了两个局部变量`_x`和`_y`分别存储`x`和`y`的值。

- `(void) (&_x == &_y)`的作用有两个:

  1. 对于不同类型的指针比较，编译器会发出警告。

     ```bash
     warning: comparison of distinct pointer types lacks a cast
     ```

  2. 两个数进行比较运算，运算的结果却没有用到，有些编译器可能会给出一个`warning`，加一个`(void)`后，就可以消除这个警告。

在`Linux`内核中大量使用了语句表达式。

## `typeof`与`container_of`宏

### `typeof`

`ANSI C`定义了`sizeof`关键字，用来获取一个变量或者数据类型在内存中所占的字节数。`GNU C`扩展了一个关键字`typeof`，用来获取一个变量或者表达式的类型。

`typeof`的参数有两种类型:

```c
int i;
typeof(i) j = 20; // int j = 20;
typeof(int *) a;  // int *a;
int f();
typeof(f())  k;	  // int k;
```

### `container_of`

`container_of`宏在`Linux`内核中被广泛使用。`container_of`宏的作用是根据某一成员的地址，获取这个结构体的首地址。这个宏有三个参数:`type`为结构体类型，`member`为结构体内的成员，`ptr`为结构体内成员`member`的地址。如下宏定义来自于`Linux-5.17.5`。

- `container_of`:

  ```c
  /**
   * container_of - cast a member of a structure out to the containing structure
   * @ptr:	the pointer to the member.
   * @type:	the type of the container struct this is embedded in.
   * @member:	the name of the member within the struct.
   *
   */
  #define container_of(ptr, type, member) ({				\
  	void *__mptr = (void *)(ptr);					\
  	static_assert(__same_type(*(ptr), ((type *)0)->member) ||	\
  		      __same_type(*(ptr), void),			\
  		      "pointer type mismatch in container_of()");	\
  	((type *)(__mptr - offsetof(type, member))); })
  ```

- `__same_type`:其实就是内健函数`__builtin_types_compatible_p(typeof(a), typeof(b))`:

  ```c
  #define __same_type(a, b) __builtin_types_compatible_p(typeof(a), typeof(b))
  ```

-  `static_assert`:这个宏就是把`C 11`标准中的`_Static_assert`做了包装:

  ```c
  /**
   * static_assert - check integer constant expression at build time
   *
   * static_assert() is a wrapper for the C11 _Static_assert, with a
   * little macro magic to make the message optional (defaulting to the
   * stringification of the tested expression).
   *
   * Contrary to BUILD_BUG_ON(), static_assert() can be used at global
   * scope, but requires the expression to be an integer constant
   * expression (i.e., it is not enough that __builtin_constant_p() is
   * true for expr).
   *
   * Also note that BUILD_BUG_ON() fails the build if the condition is
   * true, while static_assert() fails the build if the expression is
   * false.
   */
  #define static_assert(expr, ...) __static_assert(expr, ##__VA_ARGS__, #expr)
  #define __static_assert(expr, msg, ...) _Static_assert(expr, msg)
  ```

- `offsetof`:

- `offsetof`:`__compiler_offsetof`用来确认编译器中是否内健了功能同`offsetof`宏一样的宏，若存在内健的`__compiler_offsetof`，则使用内健函数`__builtin_offsetof(a, b)`,等价于使用`((size_t)&((TYPE *)0)->MEMBER)`。

  ```c
  #undef offsetof
  #ifdef __compiler_offsetof
  #define offsetof(TYPE, MEMBER)	__compiler_offsetof(TYPE, MEMBER)
  #else
  #define offsetof(TYPE, MEMBER)	((size_t)&((TYPE *)0)->MEMBER)
  #endif
  ```

  - `((size_t)&((TYPE *)0)->MEMBER)`说明:

    1. `(TYPE *)0`将`0`转换为`TYPE`类型的指针常量;
    2. `((TYPE *)0)->MEMBER`访问结构中的数据成员`MEMBER`;
    3. `&((TYPE *)0)->MEMBER`取出数据成员`MEMBER`的地址;
    4. `((size_t)&((TYPE*)0)->MEMBER)`转换类型;

    **巧妙之处在于将`0`转换成`(TYPE*)`类型的指针常量，结构以内存空间首地址`0`作为起始地址，则成员地址自然为偏移地址。**

- `__compiler_offsetof`:最终调用的是`GCC`内健函数`__builtin_offsetof`:

  ```c
  #define __compiler_offsetof(a, b)	__builtin_offsetof(a, b)
  ```

明白了内核中`container_of`宏的原理后，我们可以在用户态定义相同的宏并看看使用效果:

```c
#include <stdio.h>

#define static_assert(expr, ...) __static_assert(expr, ##__VA_ARGS__, #expr)
#define __static_assert(expr, msg, ...) _Static_assert(expr, msg)
#define offsetof(TYPE, MEMBER)	__compiler_offsetof(TYPE, MEMBER)

#define __same_type(a, b) __builtin_types_compatible_p(typeof(a), typeof(b))
#define __compiler_offsetof(a, b)	__builtin_offsetof(a, b)
#define container_of(ptr, type, member) ({				\
	void *__mptr = (void *)(ptr);					\
	static_assert(__same_type(*(ptr), ((type *)0)->member) ||	\
		      __same_type(*(ptr), void),			\
		      "pointer type mismatch in container_of()");	\
	((type *)(__mptr - offsetof(type, member))); })

struct container_of_test {
    int page;
    long node;
    long zone; 
};

int main() {
    struct container_of_test ct1;
    printf("address of ct1:%p\n", &ct1);
    struct container_of_test *ct2;
    ct2 = container_of(&ct1.node, struct  container_of_test, node);
    printf("address of ct2:%p\n", ct2);
    return 0;
}
```

使用`gcc -E container_of_test.c`将宏展开最终得到如下的代码:

```c
struct container_of_test {
    int page;
    long node;
    long zone;
};

int main() {
    struct container_of_test ct1;
    printf("address of ct1:%p\n", &ct1);
    struct container_of_test *ct2;
    ct2 = ({ void *__mptr = (void *)(&ct1.node); _Static_assert(__builtin_types_compatible_p(typeof(*(&ct1.node)), typeof(((struct container_of_test *)0)->node)) || __builtin_types_compatible_p(typeof(*(&ct1.node)), typeof(void)), "pointer type mismatch in container_of()"); ((struct container_of_test *)(__mptr - __builtin_offsetof(struct container_of_test, node))); });
    printf("address of ct2:%p\n", ct2);
    return 0;
}
```

使用`gcc -o container_of_test container_of_test.c`后执行`./container_of_test`:

```shell
# caesar @ caesar-PC in ~/study/tmpcode on git:master x [8:50:40] 
$ ./container_of_test 
address of ct1:0x7ffdd7fa30a0
address of ct2:0x7ffdd7fa30a0
```

## 变长数组与零长度数组

- 零长度数组:数组长度为`0`的数组。
- 变长数组:数组长度在编译时不确定，在程序运行时才确定。

### 变长数组

`C99`标准规定，可以定义一个变长数组:

```c
int len;
scanf("%d", &len);
int array[len];
```

### 零长度数组

零长度数组是`GNU C`扩展的，在非`GCC`编译环境下可能会编译报错或者警告:

```c
int array[0];
```

零长度数组占用的内存空间大小为`0`，如下代码编译打印后`array size`为`0`:

```c
#include <stdio.h>

int main(void) {
    int array[0];

    printf("array address:%p\tarray size:%d\n", &array, sizeof(array));
    return 0;
}
```

运行结果:

```bash
# caesar @ caesar-PC in ~/study/tmpcode on git:master x [15:24:49] 
$ ./array    
array address:0x7ffe815c8dd0	array size:0
```

一般而言，零长度数组一般单独使用的机会比较少，其常常作为结构体的一个成员，构成一个变长结构体:

```c
struct buffer {
    int len;
    int a[0];
};
```

这个结构体的大小`sizeof(struct buffer)`为`4`。如下为示例:

```c
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct buffer {
    int len;
    char arr[0];
};

int main(void) {
    struct buffer* buf;
    printf("buf size:%d\n", sizeof(*buf));	// 打印4
    buf = malloc(sizeof(struct buffer) + 20);

    buf->len = 20;
    strcpy(buf->arr, "Hello,World!\n");

    puts(buf->arr);
    
    // 打印地址， 0， 4
    printf("arr address:%p\tarr size:%d\tbuf size:%d\n", &(buf->arr), sizeof(buf->arr), sizeof(*buf));
    free(buf);
    return 0;
}
```

## 属性声明
