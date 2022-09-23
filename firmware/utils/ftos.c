
#include "ftos.h"

// 直接打印

void double_printf(double value, int decimalDigits)
{
    int i = 1;
    int intPart, fractPart;

    for (; decimalDigits != 0; i *= 10, decimalDigits--);
    intPart = (int)value;

    fractPart = (int)((value - (double)(int)value) * i);
    if (fractPart < 0) fractPart *= -1;
    printf("%d.%d", intPart, fractPart);
}

// 返回值的模式

char * double_string(double value, int decimalDigits)
{
    static char str[16];

    // int decimalDigits = 3;
    int i = 1;
    int intPart, fractPart;

    for (; decimalDigits != 0; i *= 10, decimalDigits--);
    intPart = (int)value;

    fractPart = (int)((value - (double)(int)value) * i);
    if (fractPart < 0) fractPart *= -1;
    // printf("%i.%i", intPart, fractPart);

    snprintf((char *)str, 16, "%d.%d", intPart, fractPart);

    return str;
}
