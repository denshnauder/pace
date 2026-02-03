#include "usefulFunction.h"

/*
void strcpy(char *dest, const char *src) {
    while (*src) {
        *dest++ = *src++;
    }
}
*/

int starts_with(const char *array, const char *prefix) {
    while (*prefix) {
        if (*array != *prefix) {
            return 0; 
        }
        array++;
        prefix++;
    }
    return 1;  
}