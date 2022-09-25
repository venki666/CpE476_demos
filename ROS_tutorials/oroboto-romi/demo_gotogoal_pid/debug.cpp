#include <Romi32U4.h>

#include "debug.h"

#ifdef __DEBUG__

char report[80];
char floatBuf1[16], floatBuf2[16], floatBuf3[16], floatBuf4[16], floatBuf5[16];

/**
 * Helper to print doubles
 */
char *ftoa(char *a, double f)
{
    char *ret = a;

    if (f < 0)
    {
       *a++ = '-';
    }

    f = fabs(f);

    long heiltal = (long)f;
    if ( ! itoa(heiltal, a, 10))
    {
       *a = 'X';
    }
    while (*a != '\0') a++;
    *a++ = '.';

    sprintf(a, "%04d", abs((long)((f - (double)heiltal) * 10000.0)));
    return ret;
}

#endif
