#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>

typedef unsigned char byte;
typedef enum {UNPROGRAMMED, IDLE, ONE_SHOT, FREE_RUN} timer_type;
int dprintf(const char *fmt, ...);
void byte_RMW(byte *aReg, byte left, byte right, byte value);
bool timer1_reloadOneShot(void);
void timer1_setup(float seconds, timer_type type, void (*callback)(void*), void *context);
unsigned char intLock(void);
void intUnlock(int value);
void timer1_holdoff(void); // SLOW it from reaching terminal count
void rightShift(bool bit, unsigned char *shiftbucket, unsigned int bytesInBucket);

#define STOP(msg) HALT(__FILE__, __LINE__, msg)
void HALT( const char *file, int line, const char *msg);
void watchdogSetup(void (*calldog)(void*));
int xprintf(const char *fmt, ...);
#define ELEMENTS(x) (sizeof(x)/ sizeof(x[0]))
void xmemset(unsigned char *from, unsigned length, char value);
void createStackDump();
uint16_t getPeakStackUsage(void);
void showCharInBinary(unsigned char me);
void shift_left(bool bit, unsigned char *dest, unsigned numElements);
void shift_right(bool bit, unsigned char *dest, unsigned numElements);
unsigned int justify_left(unsigned char *dest, unsigned numElements);
void showCharArrayInBinary(unsigned char *who, unsigned length);
void showCharArrayInHex(unsigned char *who, unsigned length);
unsigned int shiftout_left(unsigned char *dest, unsigned numElements, unsigned numBits);

#define INTLOCK //unsigned char ___foo = SREG; cli();
#define INTUNLOCK //SREG=___foo;

#endif


