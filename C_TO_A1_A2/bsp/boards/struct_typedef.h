#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

// 引入标准库，让编译器自己决定 int32_t 到底是什么
#include <stdint.h>

// 保留标准库里没有的自定义类型
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif


