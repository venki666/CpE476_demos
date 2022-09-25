#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __DEBUG__
extern char report[80];
extern char floatBuf1[16], floatBuf2[16], floatBuf3[16], floatBuf4[16], floatBuf5[16];

char *ftoa(char *a, double f);
#endif

#endif // __DEBUG_H__
