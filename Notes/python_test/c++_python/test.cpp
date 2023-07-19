#include <stdio.h>
#include <stdlib.h>

extern "C" int sum(int a, int b);
int sum(int a, int b){
    printf("this is cpp program");
    return a+b;
}