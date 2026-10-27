/*
 * Minimal stand-ins for the 23 symbols dsplibs.o imports from the rest of
 * slmodemd (see NOTES.md's "Inbound" list, reconfirmed via `nm -u dsplibs.o`).
 *
 * dsplibs.o is a single relocatable object, so linking against it for even
 * one function (RingDetector_*, FPM_*, B103FP_*) pulls in the WHOLE object
 * and therefore requires every symbol it references to resolve, even though
 * any one test harness only ever exercises one code path through it. Of
 * these, sysdep_malloc/free/memset and dsplibs_debug_level/printf get real
 * behavior (memset was added for B103FP_create, which - unlike
 * RingDetector_* or FPM_* - actually calls it); the rest exist purely to
 * satisfy the linker and abort() loudly if that assumption is ever wrong
 * for a given test.
 *
 * __divdi3/__moddi3 and memcpy are NOT stubbed here - they're supplied
 * automatically by libgcc/libc when linking with `gcc -m32` (not `ld`
 * directly).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

/* --- actually reachable from RingDetector_*, FPM_*, B103FP_* --- */

unsigned int dsplibs_debug_level = 0;

int dsplibs_debug_printf(const char *fmt, ...)
{
  /* Real (not a no-op) since test/v8_fsm_diff_main.c's debug-string check
   * scenario needs actual output to diff - every other scenario in this
   * project leaves dsplibs_debug_level at its default 0, so this change is
   * inert for them (the `if (1 < dsplibs_debug_level)` guards around every
   * call site never fire). */
  va_list ap;
  int ret;
  va_start(ap, fmt);
  ret = vprintf(fmt, ap);
  va_end(ap);
  return ret;
}

/* H01: optional heap-pattern mode (inert unless the environment asks for it).  In a fresh test process malloc() hands out zero pages, so a field a
 * port constructor forgets to write reads 0 -- as in the blob -- and the differential test cannot see it, while in a live slmodemd the heap is
 * dirty (found by N02: V32FP_recreate).  H01_HEAPFILL=<byte> fills every block with that byte; H01_DUMP=<dir> additionally writes
 * <dir>/<exe>.<argv1>.f<fill>.txt with every block's contents when the code frees it and at exit (pointers into blocks as P<block>+<off>, other
 * pointers into the executable as X); test/cross/h01_heap.cmp.py compares the blob's and the port's dumps of two fills.  See docs/bugs.md (H01). */
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#define H01_MAXB 8192
static struct { void *p; unsigned size; int freed; } h01_b[H01_MAXB];
static int h01_n, h01_init, h01_fill = -1;
static const char *h01_dir;
static FILE *h01_out;
static long h01_exe_lo, h01_exe_hi;
static void h01_atexit(void);
static void h01_emit(int i, char state);
static void h01_setup(void)
{
  const char *f = getenv("H01_HEAPFILL");
  h01_init = 1;
  if (f && *f) h01_fill = (int)(strtol(f, 0, 0) & 0xff);
  h01_dir = getenv("H01_DUMP");
  if (h01_dir && !*h01_dir) h01_dir = 0;
  if (h01_dir) atexit(h01_atexit);
}
void *sysdep_malloc(unsigned int size)
{
  void *p;
  if (!h01_init) h01_setup();
  p = malloc(size);
  if (p && h01_fill >= 0) memset(p, h01_fill, size);
  if (p && h01_dir && h01_n < H01_MAXB) { h01_b[h01_n].p = p; h01_b[h01_n].size = size; h01_b[h01_n].freed = 0; h01_n++; }
  return p;
}

void sysdep_free(void *mem)
{
  int i;
  for (i = 0; i < h01_n; i++) if (h01_b[i].p == mem && !h01_b[i].freed) { h01_emit(i, 'F'); h01_b[i].freed = 1; break; }
  free(mem);
}

static void h01_open(void)
{
  char name[512], exe[256], arg1[64] = "0", cmd[1024];
  int k, fd;
  ssize_t n;
  const char *base;
  if (h01_out) return;
  n = readlink("/proc/self/exe", exe, sizeof exe - 1);
  if (n <= 0) return;
  exe[n] = 0;
  base = strrchr(exe, '/'); base = base ? base + 1 : exe;
  fd = open("/proc/self/cmdline", O_RDONLY);
  if (fd >= 0) {
    n = read(fd, cmd, sizeof cmd - 1);
    close(fd);
    if (n > 0) { cmd[n] = 0; for (k = 0; k < n && cmd[k]; k++) {} if (k + 1 < n) { strncpy(arg1, cmd + k + 1, sizeof arg1 - 1); arg1[sizeof arg1 - 1] = 0; } }
    for (k = 0; arg1[k]; k++) if (arg1[k] == '/' ) arg1[k] = '_';
  }
  {
    FILE *m = fopen("/proc/self/maps", "r");
    char line[512];
    while (m && fgets(line, sizeof line, m)) {
      unsigned long lo, hi;
      if (sscanf(line, "%lx-%lx", &lo, &hi) == 2 && strstr(line, base)) { if (!h01_exe_lo || (long)lo < h01_exe_lo) h01_exe_lo = (long)lo; if ((long)hi > h01_exe_hi) h01_exe_hi = (long)hi; }
    }
    if (m) fclose(m);
  }
  snprintf(name, sizeof name, "%s/%s.%s.f%d.txt", h01_dir, base, arg1, h01_fill);
  h01_out = fopen(name, "w");
}
/* one block's contents: "B <index> <size> <state>" (state F = snapshot taken when the code frees it, L = still live at exit), then the words */
static void h01_emit(int i, char state)
{
  const unsigned char *b = (const unsigned char *)h01_b[i].p;
  int k;
  h01_open();
  if (!h01_out) return;
  fprintf(h01_out, "B %d %u %c\n", i, h01_b[i].size, state);
  for (k = 0; k + 4 <= (int)h01_b[i].size; k += 4) {
    unsigned v, j;
    int hit = 0;
    memcpy(&v, b + k, 4);
    for (j = 0; (int)j < h01_n && !hit; j++) {
      unsigned long lo = (unsigned long)(uintptr_t)h01_b[j].p;
      if (v && v >= lo && v <= lo + h01_b[j].size) { fprintf(h01_out, " P%u+%lu", j, (unsigned long)v - lo); hit = 1; }
    }
    if (!hit) { if ((long)v >= h01_exe_lo && (long)v < h01_exe_hi) fprintf(h01_out, " X"); else fprintf(h01_out, " %08x", v); }
  }
  for (k = h01_b[i].size & ~3u; k < (int)h01_b[i].size; k++) fprintf(h01_out, " b%02x", b[k]);
  fprintf(h01_out, "\n");
}
static void h01_atexit(void)
{
  int i;
  for (i = 0; i < h01_n; i++) if (!h01_b[i].freed) h01_emit(i, 'L');
  if (h01_out) fclose(h01_out);
}

void *sysdep_memset(void *dst, int c, size_t len)
{
  return memset(dst, c, len);
}

/* --- unreachable: linker-satisfying only --- */

static void unreachable(const char *name)
{
  fprintf(stderr, "stub_env: unexpectedly called %s (this test "
                   "should never reach this)\n", name);
  abort();
}

void sysdep_memcpy(void) { unreachable("sysdep_memcpy"); }
void sysdep_sprintf(void) { unreachable("sysdep_sprintf"); }
/* The V.34 tier-3 group C test (v34setuptxmit -> settxlevel ->
 * GetVPcmMinimalTxPowerReduction -> edprintf) really reaches these four;
 * it defines them itself (test/stub_env_str.c) and builds this file with
 * -DSTUB_ENV_REAL_STRING. Every other test keeps the abort-if-reached stubs. */
#ifndef STUB_ENV_REAL_STRING
void sysdep_strcat(void) { unreachable("sysdep_strcat"); }
void sysdep_strcpy(void) { unreachable("sysdep_strcpy"); }
void sysdep_strlen(void) { unreachable("sysdep_strlen"); }
void sysdep_vsnprintf(void) { unreachable("sysdep_vsnprintf"); }
#endif
void modem_debug_log_data(void) { unreachable("modem_debug_log_data"); }
void modem_dp_deregister(void) { unreachable("modem_dp_deregister"); }
void modem_dp_register(void) { unreachable("modem_dp_register"); }
void modem_get_bits(void) { unreachable("modem_get_bits"); }
void modem_get_param(void) { unreachable("modem_get_param"); }
void modem_get_sreg(void) { unreachable("modem_get_sreg"); }
void modem_put_bits(void) { unreachable("modem_put_bits"); }
void modem_recv_from_tty(void) { unreachable("modem_recv_from_tty"); }
void modem_send_to_tty(void) { unreachable("modem_send_to_tty"); }
void modem_set_param(void) { unreachable("modem_set_param"); }
