/*
 * Real implementations of the four string helpers dsplibs.o's edprintf uses
 * (sysdep_vsnprintf / sysdep_strlen / sysdep_strcpy / sysdep_strcat), for the
 * V.34 tier-3 group C differential test: v34setuptxmit -> settxlevel ->
 * GetVPcmMinimalTxPowerReduction -> edprintf runs unconditionally.
 * Signatures from the disassembly of edprintf (b09f0): vsnprintf(buf, 0x100,
 * fmt, ap) with ap = &first vararg (a plain i386 va_list), strlen(buf),
 * strcpy(dst, src), strcat(dst, src). Build with stub_env.c's
 * -DSTUB_ENV_REAL_STRING.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

int sysdep_vsnprintf(char *buf, unsigned int n, const char *fmt, va_list ap) {
  return vsnprintf(buf, n, fmt, ap);
}
unsigned int sysdep_strlen(const char *s) { return (unsigned int)strlen(s); }
char *sysdep_strcpy(char *d, const char *s) { return strcpy(d, s); }
char *sysdep_strcat(char *d, const char *s) { return strcat(d, s); }
