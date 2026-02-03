#ifndef USEFUL_FUNCTION_H
#define USEFUL_FUNCTION_H

// 引入系统字符串库，解决 strcpy 问题
#include <string.h>

// 如果你有自定义的 strcpy 实现，请把它改名为 my_strcpy
// void strcpy(char *dest, const char *src); // 这行必须注释掉或删除

int starts_with(const char *array, const char *prefix);

#endif
