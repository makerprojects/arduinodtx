/* arduinotx_lib.h - Library
** 2013-08-11 aPrintfln()
** 2013-08-14 getProgmemStrchr(), getProgmemStrpos()
** 2013-08-16 del getProgmemStrchr()
*/


#ifndef arduinotx_lib_h
#define arduinotx_lib_h
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdarg.h>

#define getProgmemByteArrayValue(array, idx) (pgm_read_byte((array) + (idx)))
#define getProgmemIntArrayValue(array, idx) (pgm_read_word((array) + (idx)))

byte serialInit(long bauds_lng = 9600L);
void aprintfln(const char *fmt_str, ... );
void aPrintfln(const char *fmt_pstr, ... );
void aprintf(const char *fmt_str, ... );
void aPrintf(const char *fmt_str, ... );
int serialWrite(char c, FILE *f);
int getProgmemStrpos(PGM_P pgm_str, const char c_chr);
char *getProgmemStrArrayValue(char *out_buffer_str, PGM_P *array_str, int idx_int, size_t buffersize_int);
int findProgmemStrArrayIndex(PGM_P *array_str, const char *value_str, int nitems_int = 32767);
void printProgmemStrArray(PGM_P *array_str, int nitems_int = 32767);
byte Isblank(const char *line_str);
char *Trimwhitespace(char *out_line_str);
char *TimeString(unsigned long seconds_lng, char *out_buffer_str);
byte ishexdigit(char a_chr);
unsigned long hex2dec(char *hex_str, byte length_byt);
#endif
