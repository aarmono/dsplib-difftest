/*
 * R13 differential harness: the caller-ID modem and API of dsplibs.o (cid_modem, create_cid, create_cid_dtmf, reset_cid, data_formatted_output,
 * data_unformatted_output, data_raw, cid_create / cid_delete / cid_progress / cid_get_strings / cid_freq_sampl / cid_reset / cid_threshold / cid_value and the entry
 * points CID_create / CID_delete / CID_process) vs src/cid/cid_modem.c (+ src/cid/cid_detect.c, src/fpm/fpm_div32.c, src/cid/cid_tables.c, src/fpm/fpm_mrf.c as the real callees of each
 * side).  Typed objects of src/cid/cid.h on both sides (identical i386 layout at -m32); the 32-bit BLOB run is the x86-64 golden.  Everything printed is
 * layout independent (names, not offsets; the sizes of the malloc requests are printed as object classes), all signals are integer-only.
 *
 * Callee recorders (identical text on both sides): sysdep_malloc / sysdep_free (deterministic heap: red zones, poisoned frees, id-patterned fresh blocks that
 * are laid out by the i386 byte index so that the blob's reads of uninitialised bytes see the same values on x86-64; failure injection; sysdep_free(NULL) is
 * logged, not fatal), sysdep_memset, sysdep_strlen, modem_send_to_tty (the caller-ID delivery path), dsplibs_debug_printf (stub_env.c, printed).
 *
 * Scenarios (each its own process; a coverage counter that is zero exits 2, a harness violation exits 3):
 *   0 lifecycle: CID_create / CID_delete (fs 8000 / 9600 / invalid, formats, debug 0 / 2, allocation failures 1 and 2), cid_create / cid_delete / cid_reset /
 *     cid_freq_sampl / cid_threshold / cid_value in every mode, re-creation of a live object, create_cid / create_cid_dtmf / reset_cid on static and heap objects;
 *   1 result strings: data_formatted_output / data_unformatted_output / data_raw / cid_get_strings over SDMF and MDMF messages with every parameter code (date, number,
 *     name, 'P' / 'O' absence reasons, other codes -> the hex "MESG" strings), empty / short / long fields, count 0, odd lengths;
 *   2 cid_modem alone: poked states (tone counter, gain, byte framer states 0 .. 5, threshold), noise / tone / silence blocks of 1 .. 206 samples at 8000 / 9600 Hz,
 *     the checksum branch (good / bad / length 0), whole messages fed in blocks;
 *   3 CID_process end to end: Bell 202 / V.23 caller-ID FSK messages (SDMF / MDMF, name, date and time, number, P / O, bad checksum, truncated, noise, level and DC
 *     changes, chunks of 1 .. 600 samples) at 8000 / 9600 Hz, formats 0 .. 3, debug 0 / 2: the text delivered to the tty;
 *   4 cid_progress / cid_get_strings in every mode (FSK, DTMF, automatic, done states) with FSK and DTMF caller-ID messages.
 */
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cid/cid.h"

typedef unsigned long long u64;
extern void *sysdep_malloc(unsigned int size);
extern void sysdep_free(void *mem);
extern void *sysdep_memset(void *dst, int c, size_t len);
extern size_t sysdep_strlen(const char *s);
extern int modem_send_to_tty(struct modem *m, const char *buf, int n);

/* ------------------------------------------------------------------------------------------------ canonical output, PRNG, coverage (as test/r04_common.h) */
static u64 g_hash = 1469598103934665603ULL, g_lines;
static int g_verbose; /* R13_VERBOSE=1: print every emitted line (debugging a mismatch; the hashed text is the same) */
static void hash_bytes(const void *p, size_t n)
{
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    g_hash ^= b[i];
    g_hash *= 1099511628211ULL;
  }
}
static void emit(int show, const char *fmt, ...)
{
  char buf[2048];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  hash_bytes(buf, strlen(buf));
  hash_bytes("\n", 1);
  g_lines++;
  if (show || g_verbose) puts(buf);
}
static void die3(const char *fmt, ...) __attribute__((noreturn, format(printf, 1, 2)));
static void die3(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  fprintf(stderr, "HARNESS/MODEL VIOLATION: ");
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  va_end(ap);
  exit(3);
}
static unsigned g_lcg = 0x2545f491u;
static unsigned rnd(void)
{
  unsigned x;
  g_lcg = g_lcg * 1664525u + 1013904223u;
  x = g_lcg;
  x ^= x >> 15;
  x *= 0x2c1b3c6du;
  x ^= x >> 12;
  x *= 0x297a2d39u;
  x ^= x >> 15;
  return x;
}
static unsigned rmod(unsigned m) { return m ? rnd() % m : 0; }
static int rpct(int pct) { return (int)rmod(100) < pct; }
static int rrange(int lo, int hi) { return lo + (int)rmod((unsigned)(hi - lo + 1)); }
static short rshort(void) { return (short)rnd(); }
#define COV_MAX 160
static unsigned long g_cov[COV_MAX];
static const char *g_covname[COV_MAX];
static void cov_def(int id, const char *name) { g_covname[id] = name; }
static void cov_hit(int id) { g_cov[id]++; }
static void cov_finish(void)
{
  int i, bad = 0;
  for (i = 0; i < COV_MAX; i++) {
    if (!g_covname[i]) continue;
    emit(1, "COV %-52s %lu", g_covname[i], g_cov[i]);
    if (g_cov[i] == 0) {
      fprintf(stderr, "COVERAGE FAIL: %s was never reached\n", g_covname[i]);
      bad = 1;
    }
  }
  emit(1, "HASH %016llx lines %llu", g_hash, g_lines);
  fflush(stdout);
  if (bad) exit(2);
}
static u64 fnv_bytes(const void *p, unsigned n)
{
  u64 h = 1469598103934665603ULL;
  const unsigned char *b = (const unsigned char *)p;
  unsigned i;
  for (i = 0; i < n; i++) {
    h ^= b[i];
    h *= 1099511628211ULL;
  }
  return h;
}
static u64 fnv_shorts(const short *p, unsigned n)
{
  u64 h = 1469598103934665603ULL;
  unsigned i;
  for (i = 0; i < n; i++) {
    h ^= (unsigned short)p[i] & 0xffu;
    h *= 1099511628211ULL;
    h ^= ((unsigned short)p[i] >> 8) & 0xffu;
    h *= 1099511628211ULL;
  }
  return h;
}

enum {
  CV_LIFE_OK = 0, CV_LIFE_9600, CV_LIFE_BADFS, CV_LIFE_FAIL1, CV_LIFE_FAIL2, CV_LIFE_MODE0, CV_LIFE_MODE1, CV_LIFE_MODE5, CV_LIFE_MODEN, CV_LIFE_RECREATE,
  CV_LIFE_DBG, CV_LIFE_STATIC, CV_LIFE_NULLFREE,
  CV_STR_SDMF = 20, CV_STR_MDMF, CV_STR_NAME, CV_STR_NUM, CV_STR_ABSNUM, CV_STR_ABSNAME, CV_STR_MESG, CV_STR_MESG2, CV_STR_NOCOUNT, CV_STR_UNFMT, CV_STR_DIGITS, CV_STR_RAW,
  CV_STR_NODATE, CV_STR_NONAME, CV_STR_SHORTMESG,
  CV_MOD_TONE = 40, CV_MOD_NOTONE, CV_MOD_FSD, CV_MOD_ST0, CV_MOD_ST1, CV_MOD_ST2, CV_MOD_ST3, CV_MOD_BYTE, CV_MOD_CKGOOD, CV_MOD_CKBAD, CV_MOD_LEN0, CV_MOD_AGC, CV_MOD_AGCBIG,
  CV_MOD_9600, CV_MOD_8000, CV_MOD_DBG, CV_MOD_RET1, CV_MOD_RET2, CV_MOD_RETM1, CV_MOD_RET3, CV_MOD_MSGDONE, CV_MOD_GAIN0, CV_MOD_DEFAULTST, CV_MOD_LIMIT,
  CV_E2E_SDMF = 70, CV_E2E_MDMF, CV_E2E_NAME, CV_E2E_BELL, CV_E2E_V23, CV_E2E_9600, CV_E2E_8000, CV_E2E_FAIL, CV_E2E_BADCK, CV_E2E_TRUNC, CV_E2E_NOISE, CV_E2E_FMT2,
  CV_E2E_DBG, CV_E2E_TTY, CV_E2E_CHUNK1, CV_E2E_CHUNKBIG, CV_E2E_PO, CV_E2E_4STR, CV_E2E_SUCC2,
  CV_PRG_FSK = 100, CV_PRG_DTMF, CV_PRG_AUTO, CV_PRG_DONE2, CV_PRG_DONE3, CV_PRG_R0, CV_PRG_R1, CV_PRG_R2, CV_PRG_R3, CV_PRG_DTMFFAIL, CV_PRG_FSKFAIL, CV_PRG_DIGITS,
  CV_PRG_FRAME160, CV_PRG_FRAME192, CV_PRG_MODE3FAIL, CV_PRG_DBG, CV_PRG_STRINGS, CV_PRG_AUTOFSK, CV_PRG_AUTODTMF, CV_PRG_RESET
};

/* ------------------------------------------------------------------------------------------------ deterministic heap + callee recorders */
#define ARENA_SZ (1u << 21)
#define REDZ 48u
#define MAXBLK 2048
enum { CL_NONE = 0, CL_CID, CL_DTMF, CL_FSK, CL_HDL };
typedef struct {
  unsigned char *p;
  unsigned size, id;
  int freed, cls;
} Blk;
static unsigned char g_arena[ARENA_SZ] __attribute__((aligned(64)));
static size_t g_off = 64;
static Blk g_blk[MAXBLK];
static unsigned g_nblk, g_next_id = 1, g_live;
static unsigned long g_nmalloc_call;
static unsigned long g_fail_at; /* the g_fail_at-th sysdep_malloc of the trial fails (1-based; 0 = never) */
static unsigned g_trial_mallocs;

static int size_class(unsigned size)
{
  if (size == 0x3fc || size == (unsigned)sizeof(CidObj)) return CL_CID;
  if (size == 0x38c || size == (unsigned)sizeof(CidDtmf)) return CL_DTMF;
  if (size == 0x160 || size == (unsigned)sizeof(CidFsk)) return CL_FSK;
  if (size == 8 || size == (unsigned)sizeof(CidHandle)) return CL_HDL;
  return CL_NONE;
}
static const char *class_name(int cls)
{
  switch (cls) {
  case CL_CID: return "CID";
  case CL_DTMF: return "DTMF";
  case CL_FSK: return "FSK";
  case CL_HDL: return "HDL";
  default: return "";
  }
}
/* byte offset of this host's object -> byte offset in the blob's i386 object (the pattern of a fresh block is a function of the i386 index) */
static unsigned i386_off(int cls, unsigned i)
{
  switch (cls) {
  case CL_DTMF:
    if (i < offsetof(CidDtmf, blkp)) return i;
    if (i < offsetof(CidDtmf, blkp) + sizeof(void *)) return 0x338 + (i - (unsigned)offsetof(CidDtmf, blkp)) % 4;
    return i - ((unsigned)sizeof(CidDtmf) - 0x38c);
  case CL_FSK:
    if (i < offsetof(CidFsk, mrf)) return i;
    if (i < offsetof(CidFsk, cfgThr)) return 0xc + (i - (unsigned)offsetof(CidFsk, mrf)) % 0x1c;
    return i - ((unsigned)sizeof(CidFsk) - 0x160);
  case CL_CID:
    if (i < 2 * sizeof(void *)) return i % 8;
    return i - (unsigned)(2 * sizeof(void *) - 8);
  default: return i;
  }
}
static unsigned char pat_byte(unsigned id, int cls, unsigned i)
{
  unsigned j = i386_off(cls, i);
  return (unsigned char)(0x35u + id * 13u + j * 7u + (j >> 6));
}
void *sysdep_malloc(unsigned int size)
{
  size_t asz = ((size_t)size + 15u) & ~(size_t)15u, i;
  unsigned id;
  unsigned char *p;
  int cls = size_class(size);
  g_nmalloc_call++;
  g_trial_mallocs++;
  if (g_fail_at && g_trial_mallocs == g_fail_at) {
    emit(0, "M fail %s%u", class_name(cls), cls ? 0u : size);
    return NULL;
  }
  id = g_next_id++;
  if (g_off + REDZ + asz + REDZ > ARENA_SZ || g_nblk >= MAXBLK) die3("arena exhausted");
  memset(g_arena + g_off, 0xCC, REDZ);
  p = g_arena + g_off + REDZ;
  for (i = 0; i < asz; i++) p[i] = pat_byte(id, cls, (unsigned)i);
  memset(p + asz, 0xCC, REDZ);
  g_off += REDZ + asz + REDZ;
  g_blk[g_nblk].p = p;
  g_blk[g_nblk].size = size;
  g_blk[g_nblk].id = id;
  g_blk[g_nblk].freed = 0;
  g_blk[g_nblk].cls = cls;
  g_nblk++;
  g_live++;
  if (cls) emit(0, "M %u %s", id, class_name(cls));
  else emit(0, "M %u %u", id, size);
  return p;
}
static Blk *find_blk(const void *q)
{
  unsigned i;
  const unsigned char *p = (const unsigned char *)q;
  for (i = 0; i < g_nblk; i++)
    if (p >= g_blk[i].p && p < g_blk[i].p + g_blk[i].size) return &g_blk[i];
  return NULL;
}
static void blk_check(const Blk *b, const char *what)
{
  size_t asz = ((size_t)b->size + 15u) & ~(size_t)15u, k;
  for (k = 0; k < REDZ; k++)
    if (b->p[asz + k] != 0xCC || b->p[(size_t)0 - REDZ + k] != 0xCC) die3("block %u: a byte outside the block was written (%s)", b->id, what);
}
void sysdep_free(void *q)
{
  Blk *b;
  if (!q) {
    emit(0, "F NULL");
    return;
  }
  b = find_blk(q);
  if (!b || b->p != (unsigned char *)q) die3("sysdep_free of a pointer that is not a block base");
  if (b->freed) die3("double free of block %u", b->id);
  blk_check(b, "at free");
  b->freed = 1;
  g_live--;
  memset(b->p, 0xDD, ((size_t)b->size + 15u) & ~(size_t)15u);
  emit(0, "F %u", b->id);
}
/* where a pointer lives: "b<id>+<i386 offset>" inside a heap block, "NULL" */
static void ptr_desc(char *buf, size_t n, const void *q)
{
  Blk *b;
  if (!q) {
    snprintf(buf, n, "NULL");
    return;
  }
  b = find_blk(q);
  if (b) {
    snprintf(buf, n, "b%u+%u", b->id, i386_off(b->cls, (unsigned)((const unsigned char *)q - b->p)));
    return;
  }
  snprintf(buf, n, "?");
}
void *sysdep_memset(void *dst, int c, size_t len)
{
  char d[40];
  Blk *b = find_blk(dst);
  unsigned nl = (unsigned)len;
  if (b && (const unsigned char *)dst + len > b->p + b->size) die3("memset of %u bytes overruns block %u", (unsigned)len, b->id);
  if (b && b->freed) die3("memset of a freed block");
  if (len == 8 || len == sizeof(CidHandle)) nl = 8;
  ptr_desc(d, sizeof d, dst);
  emit(0, "S %s %d %u", d, c, nl);
  return memset(dst, c, len);
}
void *sysdep_memcpy(void *d, const void *s, size_t n) { return memcpy(d, s, n); } /* only for the link: nothing in the caller-ID code calls it */
size_t sysdep_strlen(const char *s)
{
  size_t n = strlen(s);
  emit(0, "L %u", (unsigned)n);
  return n;
}
static int g_modem_tag[4];
static unsigned g_tty_calls, g_tty_bytes;
static char g_tty[8192];
static size_t g_tty_len;
int modem_send_to_tty(struct modem *m, const char *buf, int n)
{
  char esc[1300];
  int i, k = 0, mi = -1;
  for (i = 0; i < 4; i++)
    if (m == (void *)&g_modem_tag[i]) mi = i;
  for (i = 0; i < n && k < (int)sizeof esc - 8; i++) {
    unsigned char c = (unsigned char)buf[i];
    if (c == '\r') { esc[k++] = '\\'; esc[k++] = 'r'; }
    else if (c == '\n') { esc[k++] = '\\'; esc[k++] = 'n'; }
    else if (c >= 0x20 && c < 0x7f) esc[k++] = (char)c;
    else k += snprintf(esc + k, 6, "\\x%02x", c);
  }
  esc[k] = 0;
  emit(0, "T m%d n=%d '%s'", mi, n, esc);
  g_tty_calls++;
  g_tty_bytes += (unsigned)n;
  if (g_tty_len + (size_t)n + 1 < sizeof g_tty) {
    memcpy(g_tty + g_tty_len, buf, (size_t)n);
    g_tty_len += (size_t)n;
    g_tty[g_tty_len] = 0;
  }
  return n;
}
static void trial_end(const char *what, int expect_leak)
{
  unsigned i;
  size_t k;
  for (i = 0; i < g_nblk; i++) {
    const Blk *b = &g_blk[i];
    size_t asz = ((size_t)b->size + 15u) & ~(size_t)15u;
    blk_check(b, what);
    if (b->freed) {
      for (k = 0; k < asz; k++)
        if (b->p[k] != 0xDD) die3("block %u written after free (%s)", b->id, what);
    }
  }
  if (g_live != 0 && !expect_leak) emit(1, "LEAK %s %u live blocks", what, g_live);
  g_nblk = 0;
  g_live = 0;
  g_off = 64;
  g_fail_at = 0;
  g_trial_mallocs = 0;
}

/* ------------------------------------------------------------------------------------------------ integer-only signals */
static int isin(unsigned phase)
{
  unsigned x = phase & 0x7fffu, u = (x * (32768u - x)) >> 15, y = (16u * u << 15) / ((5u << 15) - 4u * u);
  return (phase & 0x8000u) ? -(int)y : (int)y;
}
static unsigned inc_of(int f, int fs) { return (unsigned)((((unsigned long long)(unsigned)f) << 32) / (unsigned)fs); }
static short clip16(long long v) { return (short)(v > 32767 ? 32767 : v < -32768 ? -32768 : v); }
#define SIGMAX 400000
static short g_sig[SIGMAX];
static unsigned g_sn;
static unsigned g_ph1, g_ph2;
static void put(long long v) { if (g_sn < SIGMAX) g_sig[g_sn++] = clip16(v); }
static void gen_noise(unsigned n, int amp)
{
  unsigned i;
  for (i = 0; i < n; i++) { int r = (int)(rnd() >> 8) % (2 * amp + 1) - amp; put(r); }
}
static void gen_tone2(unsigned n, int f1, int f2, int a1, int a2, int fs, int noise, int dc)
{
  unsigned i, i1 = inc_of(f1, fs), i2 = inc_of(f2, fs);
  for (i = 0; i < n; i++) {
    long long v = (long long)a1 * isin(g_ph1 >> 16) / 32768 + (long long)a2 * isin(g_ph2 >> 16) / 32768 + dc;
    if (noise) { int r = (int)(rnd() >> 8) % (2 * noise + 1) - noise; v += r; }
    put(v);
    g_ph1 += i1;
    g_ph2 += i2;
  }
}
static const int dtmf_lo[4] = { 697, 770, 852, 941 }, dtmf_hi[4] = { 1209, 1336, 1477, 1633 };
static const short dtmf_code[16] = { 1, 2, 3, 10, 4, 5, 6, 11, 7, 8, 9, 12, 14, 0, 15, 13 };
static void gen_dtmf(unsigned n, int code, int fs, int a1, int a2, int noise)
{
  int k;
  for (k = 0; k < 16; k++) if (dtmf_code[k] == code) break;
  if (k == 16) { gen_noise(n, noise ? noise : 1); return; }
  gen_tone2(n, dtmf_lo[k / 4], dtmf_hi[k % 4], a1, a2, fs, noise, 0);
}
/* a caller-ID DTMF message: silence, 'D' (13), digits, 'C' (12) with tone / gap lengths in samples */
static void dtmf_message(int fs, int ndig, int withStart, int withEnd, int rand_codes, int amp, int noise)
{
  int i;
  int tone = fs * rrange(50, 110) / 1000, gap = fs * rrange(45, 110) / 1000;
  gen_noise((unsigned)rrange(fs / 20, fs / 3), rpct(50) ? 4 : 60);
  if (withStart) {
    gen_dtmf((unsigned)tone, 13, fs, amp, amp, noise);
    gen_noise((unsigned)gap, noise ? noise : 4);
  }
  for (i = 0; i < ndig; i++) {
    int code = rand_codes ? (rpct(70) ? dtmf_code[rmod(16)] : rrange(-1, 18)) : (int)rmod(10);
    int tn = tone + (rpct(10) ? rrange(-tone / 2, tone / 2) : 0), gp = gap + (rpct(10) ? rrange(-gap / 2, gap / 2) : 0);
    int tw = rpct(20) ? amp / 2 : amp;
    gen_dtmf((unsigned)tn, code, fs, amp, tw, noise);
    gen_noise((unsigned)gp, noise ? noise : 4);
  }
  if (withEnd) {
    gen_dtmf((unsigned)tone, 12, fs, amp, amp, noise);
    gen_noise((unsigned)gap, noise ? noise : 4);
  }
}

/* FSK caller-ID: bit stream -> phase continuous FSK (mark / space tones) */
static unsigned char g_bits[200000];
static unsigned g_nbits;
static void bits_push(int b) { if (g_nbits < sizeof g_bits) g_bits[g_nbits++] = (unsigned char)b; }
static void bits_byte(int v)
{
  int i;
  bits_push(0);
  for (i = 0; i < 8; i++) bits_push((v >> i) & 1);
  bits_push(1);
}
static void gen_fsk(int fs, int fm, int fsp, int amp, int noise, int dc, int baud)
{
  unsigned i, ph = g_ph1, im = inc_of(fm, fs), is = inc_of(fsp, fs);
  unsigned tb = 0;
  for (i = 0; i < g_nbits; ) {
    int b = g_bits[i];
    long long v = (long long)amp * isin(ph >> 16) / 32768 + dc;
    if (noise) { int r = (int)(rnd() >> 8) % (2 * noise + 1) - noise; v += r; }
    put(v);
    ph += b ? im : is;
    tb += (unsigned)baud;
    if (tb >= (unsigned)fs) { tb -= (unsigned)fs; i++; }
    if (g_sn >= SIGMAX - 2) break;
  }
  g_ph1 = ph;
}

/* ------------------------------------------------------------------------------------------------ messages */
typedef struct { unsigned char b[140]; int n; } Msg;
static void msg_sum(Msg *m, int bad)
{
  int i, s = 0;
  for (i = 0; i < m->n; i++) s += m->b[i];
  m->b[m->n++] = (unsigned char)(-s + (bad ? 1 + (int)rmod(255) : 0));
}
static void msg_puts(Msg *m, const char *s) { while (*s) m->b[m->n++] = (unsigned char)*s++; }
static void msg_digits(char *d, int n) { int i; for (i = 0; i < n; i++) d[i] = (char)('0' + rmod(10)); d[n] = 0; }
/* SDMF: type 4, length, MMDDHHMM, number ("P" private, "O" out of area, "" none); returns the message without the checksum when nosum */
static void sdmf(Msg *m, const char *num, int bad)
{
  char date[9];
  int len;
  date[0] = (char)('0' + rmod(2)); date[1] = (char)('1' + rmod(9)); date[2] = (char)('0' + rmod(3)); date[3] = (char)('1' + rmod(9));
  date[4] = (char)('0' + rmod(2)); date[5] = (char)('0' + rmod(10)); date[6] = (char)('0' + rmod(6)); date[7] = (char)('0' + rmod(10)); date[8] = 0;
  m->n = 0;
  len = 8 + (int)strlen(num);
  m->b[m->n++] = 0x04;
  m->b[m->n++] = (unsigned char)len;
  msg_puts(m, date);
  msg_puts(m, num);
  msg_sum(m, bad);
}
static void tlv(Msg *m, int type, const char *s)
{
  m->b[m->n++] = (unsigned char)type;
  m->b[m->n++] = (unsigned char)strlen(s);
  msg_puts(m, s);
}
/* MDMF: type 0x80; kind bits: 1 date, 2 number, 4 name, 8 number absent 'P'/'O' (type 4), 16 name absent (type 8), 32 other types */
static void mdmf(Msg *m, int kind, int bad)
{
  char date[9], num[24], name[40];
  int nn = rrange(1, 14), i, lenpos;
  msg_digits(date, 8);
  date[0] = (char)('0' + rmod(2));
  msg_digits(num, nn);
  for (i = 0; i < rrange(1, 20); i++) name[i] = (char)(rpct(15) ? ' ' : 'A' + rmod(26));
  name[i] = 0;
  m->n = 0;
  m->b[m->n++] = 0x80;
  lenpos = m->n++;
  if (kind & 1) tlv(m, 1, date);
  if (kind & 2) tlv(m, 2, num);
  if (kind & 8) tlv(m, 4, rpct(50) ? "P" : "O");
  if (kind & 4) tlv(m, 7, name);
  if (kind & 16) tlv(m, 8, rpct(50) ? "P" : "O");
  if (kind & 32) {
    static const int types[] = { 3, 5, 6, 9, 0x0b, 0x11, 0x13, 0x77 };
    int k, c = rrange(1, 3);
    for (k = 0; k < c; k++) {
      char o[12];
      int l = rrange(0, 7);
      for (i = 0; i < l; i++) o[i] = (char)(rmod(256) | 1);
      o[l] = 0;
      for (i = 0; i < l; i++) if (!o[i]) o[i] = 1;
      tlv(m, types[rmod(8)], o);
    }
  }
  m->b[lenpos] = (unsigned char)(m->n - 2);
  msg_sum(m, bad);
}
/* the bit stream of a message the way the telephone network sends it */
static void bits_message(const Msg *m, int seizure, int mark, int tail, int trunc)
{
  int i, n = trunc >= 0 && trunc < m->n ? trunc : m->n;
  g_nbits = 0;
  for (i = 0; i < seizure / 2; i++) { bits_push(0); bits_push(1); }
  for (i = 0; i < mark; i++) bits_push(1);
  for (i = 0; i < n; i++) bits_byte(m->b[i]);
  for (i = 0; i < tail; i++) bits_push(1);
}

/* ------------------------------------------------------------------------------------------------ dumps (layout independent) */
static void dump_fsk(const char *tag, int show, const CidFsk *f)
{
  emit(show, "%s F agc=%d dc=%d s04=%d s08=%d thr28=%u fs=%u inc=%d | mrf a=%08x b=%08x,%08x ph=%d fr=%d wp=%d bl=%d buf=%d co=%d | lp=%d,%d lpf=%016llx hist=%016llx cur=%u np=%d nn=%d "
       "thr=%d mean=%d neg=%d pos=%d run=%d,%d,%d i1=%d,%d i2=%d,%d z=%d,%d,%d,%d p74=%d,%d,%d,%d s0a=%d s15e=%d out=%016llx | tc=%u o=%u,%u s=%d,%d ones=%d st=%d acc=%d bi=%d cnt=%d msg=%016llx",
       tag, f->agc, f->dc, f->s04, f->s08, f->cfgThr, f->fs, f->toneInc, (unsigned)f->mrf.a.w, (unsigned)f->mrf.b.w[0], (unsigned)f->mrf.b.w[1], f->mrf.phase, f->mrf.frac, f->mrf.wpos,
       f->mrf.buflen, f->mrf.buf != NULL, f->mrf.coefs != NULL, f->lpfPos, f->pos, fnv_shorts(f->lpf, 17), fnv_shorts(f->hist, 5), f->cur, f->nPos, f->nNeg, f->thr, f->mean,
       f->negAcc, f->posAcc, f->run, f->runAlt, f->quiet, f->iirMtd1[0], f->iirMtd1[1], f->iirMtd2[0], f->iirMtd2[1], f->z86[0], f->z86[1], f->z86[2], f->z86[3], f->pad_0074[0], f->pad_0074[1], f->pad_0074[2], f->pad_0074[3], f->s0a, f->s15e, fnv_shorts((const short *)f->out, 36), f->tc, f->out[0],
       f->out[1], f->s150, f->s152, f->ones, f->state, f->acc, f->bitIdx, f->count, fnv_bytes(f->msg, sizeof f->msg));
}
static void dump_dtmf(const char *tag, int show, const CidDtmf *d)
{
  char dig[48];
  int i;
  const char *bp = d->blkp == NULL ? "null" : d->blkp == d->blk ? "blk" : "other";
  for (i = 0; i < 16; i++) snprintf(dig + 2 * i, 3, "%02x", (unsigned char)d->digits[i]);
  emit(show, "%s D st0=%d w4=%d s8=%d last=%u cnt=%d nd=%d ph=%d half=%d fl=%d min=%u bp=%s fs=%u mode=%d dig=%s d16=%02x%02x%02x%02x ib=%d,%d lo0=%d,%d lo1=%d,%d s354=%d,%d s35c=%d,%d s=%d "
       "mtd=%016llx blk=%016llx st=%016llx",
       tag, d->state0, d->w004, d->s008, d->lastDig, d->cnt, d->nDig, d->phase, d->halfFlag, d->fails, d->minEn, bp, d->fs, d->mode, dig, (unsigned char)d->digits[16], (unsigned char)d->digits[17], (unsigned char)d->digits[18], (unsigned char)d->digits[19], d->iirBand[0], d->iirBand[1], d->iirLow0[0],
       d->iirLow0[1], d->iirLow1[0], d->iirLow1[1], d->s354, d->s356, d->s35c[0], d->s35c[1], d->samples, fnv_shorts(&d->iirMtd[0][0], 16), fnv_shorts(d->blk, 300),
       fnv_shorts(d->stash, 100));
}
static void dump_cid(const char *tag, int show, const CidObj *c)
{
  char t2[64];
  if (!c) { emit(show, "%s C NULL", tag); return; }
  emit(show, "%s C mode=%d fmt=%d fill=%d dtmf=%d fsk=%d buf=%016llx str=%016llx", tag, c->mode, c->fmt, c->fill, c->dtmf != NULL, c->fsk != NULL,
       fnv_shorts(c->buf, (unsigned)(c->fill > 0 && c->fill <= 200 ? c->fill : 0)), fnv_bytes(c->str, sizeof c->str));
  snprintf(t2, sizeof t2, "%s.", tag);
  if (c->fsk) dump_fsk(t2, show, c->fsk);
  if (c->dtmf) dump_dtmf(t2, show, c->dtmf);
}
static void show_str(const char *tag, int show, const char *s, int max)
{
  char esc[1400];
  int i, k = 0;
  for (i = 0; i < max && k < (int)sizeof esc - 8; i++) {
    unsigned char c = (unsigned char)s[i];
    if (c == 0) esc[k++] = '|';
    else if (c >= 0x20 && c < 0x7f) esc[k++] = (char)c;
    else k += snprintf(esc + k, 6, "\\x%02x", c);
  }
  esc[k] = 0;
  emit(show, "%s [%s]", tag, esc);
}

/* ------------------------------------------------------------------------------------------------ 0: lifecycle */
static CidFsk g_f;
static CidDtmf g_d;
static void sc_life(void)
{
  int t, dbg, k;
  cov_def(CV_LIFE_OK, "life: CID_create ok"); cov_def(CV_LIFE_9600, "life: fs 9600"); cov_def(CV_LIFE_BADFS, "life: invalid fs"); cov_def(CV_LIFE_FAIL1, "life: malloc 1 fails");
  cov_def(CV_LIFE_FAIL2, "life: malloc 2 fails"); cov_def(CV_LIFE_MODE0, "life: mode 0"); cov_def(CV_LIFE_MODE1, "life: mode 1"); cov_def(CV_LIFE_MODE5, "life: mode 5");
  cov_def(CV_LIFE_MODEN, "life: mode < 0 / 2..4 / > 5"); cov_def(CV_LIFE_RECREATE, "life: re-create a live object"); cov_def(CV_LIFE_DBG, "life: debug lines");
  cov_def(CV_LIFE_STATIC, "life: create on static objects"); cov_def(CV_LIFE_NULLFREE, "life: sysdep_free(NULL) (mode 1 delete)");
  for (dbg = 0; dbg < 3; dbg++) {
    dsplibs_debug_level = (unsigned)dbg;
    if (dbg > 1) cov_hit(CV_LIFE_DBG);
    for (t = 0; t < 36; t++) {
      static const int fss[6] = { 8000, 9600, 7200, 0, 44100, -1 };
      int fs = fss[t % 6], fmt = t / 6 % 4;
      CidHandle *h;
      char tag[24];
      snprintf(tag, sizeof tag, "L%d.%d", dbg, t);
      h = CID_create(&g_modem_tag[t & 3], fs, fmt);
      emit(1, "%s CID_create fs=%d fmt=%d -> %s", tag, fs, fmt, h ? "handle" : "NULL");
      if (h) {
        char hd[32];
        ptr_desc(hd, sizeof hd, h);
        emit(1, "%s handle %s modem=%d cid=%d", tag, hd, h->modem == (void *)&g_modem_tag[t & 3], h->cid != NULL);
        dump_cid(tag, t < 8, h->cid);
        cov_hit(CV_LIFE_OK);
        if (fs == 9600) cov_hit(CV_LIFE_9600);
        CID_delete(h);
      } else {
        cov_hit(CV_LIFE_BADFS);
      }
      trial_end(tag, 0);
    }
  }
  dsplibs_debug_level = 0;
  for (k = 1; k <= 2; k++) { /* allocation failures: malloc 1 (the handle) and 2 (the CID object); a later failure crashes the blob */
    CidHandle *h;
    g_fail_at = (unsigned long)k;
    h = CID_create(&g_modem_tag[0], 8000, 1);
    emit(1, "FAIL%d -> %s", k, h ? "handle" : "NULL");
    if (h) die3("allocation failure %d not reported", k);
    cov_hit(k == 1 ? CV_LIFE_FAIL1 : CV_LIFE_FAIL2);
    trial_end("fail", 0);
  }
  /* cid_create in every mode, with the setters */
  for (t = -3; t <= 8; t++) {
    CidObj *c;
    char tag[24];
    int dbgl;
    for (dbgl = 0; dbgl < 3; dbgl++) {
      int s;
      dsplibs_debug_level = (unsigned)dbgl;
      snprintf(tag, sizeof tag, "M%d.%d", t, dbgl);
      c = cid_create(NULL, 1 + (t & 3), t);
      emit(1, "%s cid_create mode=%d -> %s", tag, t, c ? "obj" : "NULL");
      dump_cid(tag, 1, c);
      if (t == 0) cov_hit(CV_LIFE_MODE0); else if (t == 1) cov_hit(CV_LIFE_MODE1); else if (t >= 2) cov_hit(t >= 2 && c && c->mode == 5 ? CV_LIFE_MODE5 : CV_LIFE_MODEN); else cov_hit(CV_LIFE_MODEN);
      for (s = 0; s < 5; s++) {
        static const int fss[5] = { 8000, 9600, 0x2580, 7200, 16000 };
        if (c->fsk) { c->fsk->toneInc = 0; c->fsk->cfgThr = 77; }
        if (c->dtmf) { c->dtmf->fs = 1; c->dtmf->mode = 3; }
        cid_freq_sampl(c, fss[s]);
        dump_cid(tag, 0, c);
        cid_threshold(c, s * 3 - 1);
        cid_value(c, s);
        dump_cid(tag, 0, c);
        c->fill = 77;
        if (c->fsk) c->fsk->toneInc = 0;
        if (s == 2 && c->mode > 1) c->mode = 3;
        cid_reset(c);
        dump_cid(tag, 0, c);
      }
      dump_cid(tag, 1, c);
      if (t == 2 || t == 0) { /* re-create a live object: no allocation, the sub-objects are re-initialised */
        CidObj *c2 = cid_create(c, 7, 0);
        emit(1, "%s recreate -> %s", tag, c2 == c ? "same" : "other");
        dump_cid(tag, 1, c2);
        cov_hit(CV_LIFE_RECREATE);
      }
      if (c->mode == 1) cov_hit(CV_LIFE_NULLFREE);
      cid_delete(c);
      trial_end(tag, 0);
    }
  }
  dsplibs_debug_level = 0;
  /* create_cid / create_cid_dtmf / reset_cid on static and heap objects, junk first (the cleared fields are overwritten, the others kept) */
  for (t = 0; t < 40; t++) {
    int dbgl = t % 3;
    char tag[24];
    CidFsk *pf;
    CidDtmf *pd;
    dsplibs_debug_level = (unsigned)dbgl;
    snprintf(tag, sizeof tag, "S%d", t);
    memset(&g_f, 0, sizeof g_f);
    memset(&g_d, 0, sizeof g_d);
    if (t & 2) { /* junk */
      int i;
      g_f.agc = (int)rnd(); g_f.dc = rshort(); g_f.cfgThr = (unsigned short)rnd(); g_f.fs = (unsigned short)rnd(); g_f.toneInc = rshort(); g_f.tc = (unsigned short)rnd();
      g_f.count = rshort(); g_f.state = rshort(); g_f.ones = rshort(); g_f.acc = rshort(); g_f.bitIdx = rshort(); g_f.thr = (int)rnd(); g_f.mean = (int)rnd();
      g_f.negAcc = (int)rnd(); g_f.posAcc = (int)rnd(); g_f.nPos = rshort(); g_f.nNeg = rshort(); g_f.run = rshort(); g_f.runAlt = rshort(); g_f.quiet = rshort();
      for (i = 0; i < 17; i++) g_f.lpf[i] = rshort();
      for (i = 0; i < 120; i++) g_f.msg[i] = (unsigned char)rnd();
      g_f.lpfPos = rshort(); g_f.pos = rshort(); g_f.cur = (unsigned short)rnd();
      for (i = 0; i < 5; i++) g_f.hist[i] = rshort();
      g_f.iirMtd1[0] = rshort(); g_f.iirMtd2[1] = rshort();
      g_d.state0 = rshort(); g_d.cnt = rshort(); g_d.nDig = rshort(); g_d.phase = rshort(); g_d.fails = rshort(); g_d.fs = (unsigned short)rnd(); g_d.mode = rshort();
      g_d.samples = (int)rnd(); g_d.minEn = (unsigned short)rnd(); g_d.lastDig = (unsigned short)rnd();
      for (i = 0; i < 16; i++) g_d.digits[i] = (char)rnd();
      for (i = 0; i < 16; i++) g_d.iirMtd[i / 2][i & 1] = rshort();
    }
    g_f.mrf.buf = NULL;
    pf = create_cid(&g_f);
    pd = create_cid_dtmf(&g_d);
    emit(1, "%s create static -> %d %d", tag, pf == &g_f, pd == &g_d);
    dump_fsk(tag, t < 6, &g_f);
    dump_dtmf(tag, t < 6, &g_d);
    cov_hit(CV_LIFE_STATIC);
    /* a reset keeps the MRF buffer */
    reset_cid(&g_f);
    dump_fsk(tag, t < 6, &g_f);
    reset_dtmf(&g_d);
    dump_dtmf(tag, t < 6, &g_d);
    trial_end(tag, 1);
    if (t < 6) { /* heap objects */
      CidFsk *hf = create_cid(NULL);
      CidDtmf *hd = create_cid_dtmf(NULL);
      dump_fsk("H", 1, hf);
      dump_dtmf("H", 1, hd);
      reset_cid(hf);
      sysdep_free(hf->mrf.buf);
      sysdep_free(hf);
      sysdep_free(hd);
      trial_end("H", 0);
    }
  }
  dsplibs_debug_level = 0;
}


/* Is it safe to run data_formatted_output / data_unformatted_output on this message?  The blob loops forever on a parameter walk with a length byte <= 0xfe (-2) and
 * reads behind the msg buffer for lengths that run past it: an independent walk of the same loops (indices must stay inside msg[0 .. 0x78), at most 300 steps per
 * loop) decides; the harness only calls the functions for messages that pass.  (The port stops such walks after 4096 steps and reads 0 behind the object.) */
static int mw_bad;
static int mw(const CidFsk *f, int i) { if (i < 0 || i >= 0x78) { mw_bad = 1; return 0; } return f->msg[i]; }
static int mws(const CidFsk *f, int i) { return (signed char)mw(f, i); }
static short mw_find(const CidFsk *f, int type)
{
  short cx = (short)mws(f, 1), p = 2;
  int steps = 0;
  while (p < cx) {
    if (mw(f, p) == type) return p;
    p = (short)(mws(f, p + 1) + p + 2);
    if (++steps > 300) mw_bad = 1;
    if (mw_bad) return -1;
  }
  return -1;
}
static int msg_ok_fmt(const CidFsk *f)
{
  short pd, pn, pm;
  int k, n, steps = 0;
  mw_bad = 0;
  if (f->count == 0) return 1;
  if (mw(f, 0) != 0x80) {
    n = mw(f, 1) + 2;
    for (k = 0; k < n && k < 0xff; k++) mw(f, k);
    return !mw_bad;
  }
  pd = mw_find(f, 1); pm = mw_find(f, 7); pn = mw_find(f, 2);
  if (mw_bad) return 0;
  for (k = 0; k < 10; k++) mw(f, pd + k);
  if (pn > 0) { n = mw(f, pn + 1) + 2; for (k = 0; k < n && k < 0xff; k++) mw(f, pn + k); }
  if (pm > 0) { n = mw(f, pm + 1) + 2; for (k = 0; k < n && k < 0xff; k++) mw(f, pm + k); }
  {
    short si = (short)mws(f, 1), eb = 2, pos = -1;
    while (eb < si) {
      int cl = mw(f, eb);
      if (cl != 1 && cl != 7 && cl != 2) { if (eb != 0) { pos = eb; break; } eb = (short)(mws(f, 1) + 2); }
      else eb = (short)(mws(f, eb + 1) + eb + 2);
      if (++steps > 300) return 0;
    }
    while (pos > 0 && !mw_bad) {
      short next = -1;
      if ((unsigned char)mw(f, pos + 1) > 1) { n = mws(f, pos + 1) + 2; if (n > 0xf5) n = 0xf5; for (k = 0; k < n; k++) mw(f, pos + k); }
      si = (short)mws(f, 1);
      eb = pos;
      if (eb < si) for (;;) {
        int cl = mw(f, eb);
        if (cl != 1 && cl != 7 && cl != 2) { if (eb != pos) { next = eb; break; } eb = (short)(mws(f, pos + 1) + pos + 2); }
        else eb = (short)(mws(f, eb + 1) + eb + 2);
        if (!(eb < si)) break;
        if (++steps > 300) return 0;
      }
      pos = next;
    }
  }
  return !mw_bad;
}
static int msg_ok_unf(const CidFsk *f)
{
  int n = (signed char)f->msg[1] + 2;
  return n <= 0x78;
}

/* ------------------------------------------------------------------------------------------------ 1: result strings */
static void load_msg(CidFsk *f, const Msg *m, int count)
{
  memset(f->msg, 0, sizeof f->msg);
  memcpy(f->msg, m->b, (size_t)m->n);
  f->count = (short)count;
}
static unsigned g_skip_fmt;
static void str_calls(const char *tag, int show, CidFsk *f)
{
  static char out[640];
  int k;
  int okf = msg_ok_fmt(f), oku = msg_ok_unf(f);
  g_skip_fmt += !okf;
  for (k = 0; k < 3; k++) {
    memset(out, 0xAA, sizeof out);
    if (k == 0) { if (!okf) continue; data_formatted_output(f, out); }
    else if (!oku) continue;
    else if (k == 1) data_unformatted_output(f, out);
    else data_raw(f->msg, out);
    emit(0, "%s k=%d all=%016llx", tag, k, fnv_bytes(out, sizeof out));
    if (k == 0) show_str(tag, show, out, 120);
    else if (k == 1) show_str(tag, show && 0, out, 40);
  }
}
static void sc_str(void)
{
  int t, kind;
  static char outa[640], outb[640];
  cov_def(CV_STR_SDMF, "str: SDMF"); cov_def(CV_STR_MDMF, "str: MDMF"); cov_def(CV_STR_NAME, "str: MDMF name"); cov_def(CV_STR_NUM, "str: MDMF number"); cov_def(CV_STR_ABSNUM, "str: absent number P/O");
  cov_def(CV_STR_ABSNAME, "str: absent name P/O"); cov_def(CV_STR_MESG, "str: MESG hex string"); cov_def(CV_STR_MESG2, "str: two MESG parameters (overwrite)");
  cov_def(CV_STR_NOCOUNT, "str: count 0"); cov_def(CV_STR_UNFMT, "str: unformatted"); cov_def(CV_STR_DIGITS, "str: DTMF digits"); cov_def(CV_STR_RAW, "str: data_raw");
  cov_def(CV_STR_NODATE, "str: MDMF without date"); cov_def(CV_STR_NONAME, "str: MDMF without name"); cov_def(CV_STR_SHORTMESG, "str: short parameter (length <= 1)");
  for (t = 0; t < 1500; t++) {
    Msg m;
    char tag[24], num[24];
    int count, nn;
    snprintf(tag, sizeof tag, "R%d", t);
    memset(&g_f, 0, sizeof g_f);
    kind = (int)rmod(8);
    if (t < 6) kind = t;
    if (kind <= 1) { /* SDMF */
      nn = kind == 0 ? rrange(0, 14) : 1;
      if (kind == 1) { num[0] = rpct(50) ? 'P' : 'O'; num[1] = 0; }
      else { msg_digits(num, nn); }
      sdmf(&m, num, rpct(5));
      cov_hit(CV_STR_SDMF);
    } else {
      int k = (rpct(70) ? 1 : 0) | (rpct(75) ? 2 : 0) | (rpct(60) ? 4 : 0) | (rpct(20) ? 8 : 0) | (rpct(15) ? 16 : 0) | (rpct(25) ? 32 : 0);
      if (t >= 6 && t < 14) k = (t - 6 + 1) * 4 % 64 | 1;
      mdmf(&m, k, rpct(5));
      cov_hit(CV_STR_MDMF);
      if (k & 4) cov_hit(CV_STR_NAME);
      if (k & 2) cov_hit(CV_STR_NUM);
      if (k & 8) cov_hit(CV_STR_ABSNUM);
      if (k & 16) cov_hit(CV_STR_ABSNAME);
      if (k & 32) cov_hit(CV_STR_MESG);
      if (!(k & 1)) cov_hit(CV_STR_NODATE);
      if (!(k & 4)) cov_hit(CV_STR_NONAME);
    }
    if (m.n > 100) die3("message too long");
    count = rpct(80) ? m.n : rpct(50) ? 0 : rrange(0, m.n);
    if (count == 0) cov_hit(CV_STR_NOCOUNT);
    load_msg(&g_f, &m, count);
    if (rpct(10)) { /* mutated bytes: the lengths stay <= 40 so that every read stays inside the object */
      int p = rrange(0, m.n - 1);
      g_f.msg[p] = (unsigned char)(p == 1 ? rmod(60) : rpct(50) ? (unsigned char)rmod(16) : (unsigned char)rmod(256));
      if (p > 1 && rpct(30)) g_f.msg[p] = (unsigned char)rmod(40);
      if (p == 0) g_f.msg[0] = rpct(50) ? 0x80 : 0x04;
    }
    if (t % 40 == 7) { /* two unknown parameters: the second header overwrites the hex string of the first */
      Msg q;
      mdmf(&q, 1 | 2 | 32, 0);
      load_msg(&g_f, &q, q.n);
      cov_hit(CV_STR_MESG2);
    }
    if (t % 60 == 11) { /* unknown parameters of length 0 / 1 / 0xff (a negative length steps one byte) */
      Msg q;
      q.n = 0;
      q.b[q.n++] = 0x80; q.b[q.n++] = 0;
      tlv(&q, 1, "10051230");
      q.b[q.n++] = 0x03; q.b[q.n++] = 0;
      q.b[q.n++] = 0x05; q.b[q.n++] = 1; q.b[q.n++] = 'x';
      if (t % 120 == 11) { q.b[q.n++] = 0x09; q.b[q.n++] = 0xff; q.b[q.n++] = 0x11; q.b[q.n++] = 0x12; }
      q.b[q.n++] = 0x0b; q.b[q.n++] = 3; q.b[q.n++] = 'a'; q.b[q.n++] = 'b'; q.b[q.n++] = 'c';
      tlv(&q, 2, "5551234");
      q.b[1] = (unsigned char)(q.n - 2);
      msg_sum(&q, 0);
      load_msg(&g_f, &q, q.n);
      cov_hit(CV_STR_SHORTMESG);
    }
    str_calls(tag, t < 40, &g_f);
    /* the formatted and unformatted output twice into dirty buffers: the same text */
    memset(outa, 0x11, sizeof outa); memset(outb, 0x22, sizeof outb);
    if (msg_ok_fmt(&g_f)) { data_formatted_output(&g_f, outa); data_formatted_output(&g_f, outb); } else { outa[0] = outb[0] = 0; }
    if (g_f.count != 0 && strcmp(outa, outb)) { show_str("A", 1, outa, 100); show_str("B", 1, outb, 100); die3("formatted output depends on the buffer content t=%d", t); }
    if (g_f.count != 0 && msg_ok_unf(&g_f)) { cov_hit(CV_STR_UNFMT); cov_hit(CV_STR_RAW); }
  }
  /* cid_get_strings in every mode */
  for (t = 0; t < 120; t++) {
    static const int modes[6] = { 0, 1, 2, 3, 5, 4 };
    int mode = modes[t % 6], fmt = (t / 6) % 4;
    int cmode = mode == 0 || mode == 2 ? 0 : mode == 1 ? 1 : 5;
    CidObj *c = cid_create(NULL, fmt, cmode);
    Msg m;
    char tag[24], num[16];
    char *s;
    int i;
    snprintf(tag, sizeof tag, "G%d", t);
    msg_digits(num, rrange(0, 10));
    if (rpct(50)) { sdmf(&m, num, 0); } else { mdmf(&m, 1 | 2 | 4 | (rpct(30) ? 32 : 0), 0); }
    if (c->fsk) load_msg(c->fsk, &m, m.n);
    if (c->dtmf) for (i = 0; i < 20; i++) c->dtmf->digits[i] = (char)(rpct(60) ? '0' + (int)rmod(10) : rpct(50) ? 0 : (int)rmod(256));
    c->mode = mode;
    if (c->fsk && (c->mode == 0 || c->mode == 2) && !msg_ok_fmt(c->fsk)) c->fsk->count = 0;
    s = cid_get_strings(c);
    if (s != c->str) die3("cid_get_strings result is not the string area");
    emit(1, "%s mode=%d fmt=%d", tag, mode, fmt);
    show_str(tag, t < 24, s, 160);
    emit(0, "%s all=%016llx", tag, fnv_bytes(s, 600));
    if (mode == 1 || mode == 3 || mode == 5 || mode == 4) cov_hit(CV_STR_DIGITS);
    c->mode = cmode;
    cid_delete(c);
    trial_end(tag, 0);
  }
  emit(1, "formatted calls skipped (the blob would loop or read behind the buffer): %u", g_skip_fmt);
}

/* ------------------------------------------------------------------------------------------------ 2: cid_modem alone */
static int thr_hi(int n, int fs) { int t = (int)((fs == 9600 ? 0xc000u : 0xa000u) / (unsigned)n); if (t <= 0) t = 1; return t * 18 >> 8; }
static int thr_lo(int n, int fs) { int t = (int)((fs == 9600 ? 0xc000u : 0xa000u) / (unsigned)n); if (t <= 0) t = 1; return t * 12 >> 8; }
static void modem_cov(int r, int pre_state, int pre_cnt, int pre_agc, int pre_tc, int n, int fs)
{
  if (fs == 9600) cov_hit(CV_MOD_9600); else if (fs == 8000) cov_hit(CV_MOD_8000);
  if (r == 1) cov_hit(CV_MOD_RET1); else if (r == 2) cov_hit(CV_MOD_RET2); else if (r == -1) cov_hit(CV_MOD_RETM1); else if (r == 3) cov_hit(CV_MOD_RET3);
  if (g_f.tc > (unsigned short)pre_tc && pre_tc >= 0 && g_f.tc <= (unsigned short)(pre_tc + 20)) cov_hit(CV_MOD_TONE);
  if (g_f.tc == 0 && pre_tc > 0) cov_hit(CV_MOD_NOTONE);
  if (r == 2) {
    cov_hit(CV_MOD_FSD);
    if (pre_state == 0) cov_hit(CV_MOD_ST0); else if (pre_state == 1) cov_hit(CV_MOD_ST1); else if (pre_state == 2) cov_hit(CV_MOD_ST2); else if (pre_state == 3) cov_hit(CV_MOD_ST3);
    else cov_hit(CV_MOD_DEFAULTST);
    if (g_f.count > pre_cnt) cov_hit(CV_MOD_BYTE);
  }
  if (r == 3) cov_hit(CV_MOD_CKGOOD);
  if (r == -1 && pre_cnt >= 3) cov_hit(CV_MOD_CKBAD);
  if (r == -1 && pre_cnt > 2 && g_f.msg[1] == 0) cov_hit(CV_MOD_LEN0);
  if (g_f.agc != pre_agc && pre_agc != 0) cov_hit(CV_MOD_AGC);
  if (pre_agc == 0 && g_f.agc > 0) cov_hit(CV_MOD_GAIN0);
  if (g_f.agc > 8) cov_hit(CV_MOD_AGCBIG);
  if (r != 1 && pre_cnt >= 0x73) cov_hit(CV_MOD_LIMIT);
  (void)n;
}
static unsigned long long g_ret_hist[8];
static int modem_call(const char *tag, int show, const short *in, int n, int fs)
{
  int pst = g_f.state, pcnt = g_f.count, pagc = g_f.agc, ptc = g_f.tc, r;
  r = cid_modem(in, (unsigned short)n, &g_f);
  emit(show, "%s n=%d -> %d", tag, n, r);
  modem_cov(r, pst, pcnt, pagc, ptc, n, fs);
  g_ret_hist[r + 1 > 0 && r + 1 < 8 ? r + 1 : 0]++;
  return r;
}
static void fsk_fresh(int fs)
{
  memset(&g_f, 0, sizeof g_f);
  g_f.mrf.buf = NULL;
  create_cid(&g_f);
  g_f.fs = (unsigned short)fs;
}
static void gen_block(int kind, int n, int fs, int amp)
{
  g_sn = 0;
  switch (kind) {
  case 0: gen_tone2((unsigned)n, 1200, 0, amp, 0, fs, 0, 0); break;
  case 1: gen_tone2((unsigned)n, 2200, 0, amp, 0, fs, 0, 0); break;
  case 2: gen_tone2((unsigned)n, 1300, 0, amp, 0, fs, amp / 8, 0); break;
  case 3: gen_noise((unsigned)n, amp); break;
  case 4: gen_noise((unsigned)n, 3); break;
  case 5: { int i; for (i = 0; i < n; i++) put(rshort()); } break;
  case 6: { int i; for (i = 0; i < n; i++) put((i / 3) & 1 ? 32767 : -32768); } break;
  case 7: gen_tone2((unsigned)n, 1200, 0, amp, 0, fs, 0, rrange(-8000, 8000)); break;
  default: gen_tone2((unsigned)n, rrange(300, 3000), 0, amp, 0, fs, 0, 0); break;
  }
}
static void poke_fsk(int fs, int n)
{
  int i, hi = thr_hi(n, fs), lo = thr_lo(n, fs);
  fsk_fresh(fs);
  switch (rmod(6)) {
  case 0: g_f.tc = 0; break;
  case 1: g_f.tc = (unsigned short)(lo + (int)rmod(3)); break;
  case 2: g_f.tc = (unsigned short)(hi - 1 + (int)rmod(3)); break;
  case 3: g_f.tc = (unsigned short)(hi + (int)rmod(40)); break;
  case 4: g_f.tc = (unsigned short)(rpct(60) ? rrange(0, 0x7ffe) : rrange(0x7fff, 0xffff)); break;
  default: g_f.tc = (unsigned short)(lo - 1 + (int)rmod(hi - lo + 3)); break;
  }
  g_f.agc = rpct(30) ? 0 : rpct(30) ? 1 : rpct(50) ? rrange(1, 12) : (int)(rnd() >> 12);
  g_f.dc = (short)(rpct(50) ? 0 : rshort() >> 2);
  g_f.cfgThr = (unsigned short)(rpct(60) ? 2 : rrange(0, 6));
  g_f.state = (short)(rpct(60) ? (int)rmod(4) : rrange(0, 6));
  g_f.ones = (short)rrange(-3, 20);
  g_f.acc = (short)rmod(256);
  g_f.bitIdx = (short)(rpct(70) ? (int)rmod(9) : rrange(-2, 40));
  g_f.count = (short)(rpct(60) ? (int)rmod(4) : rrange(0, 0x71));
  for (i = 0; i < 0x78; i++) g_f.msg[i] = (unsigned char)rnd();
  g_f.msg[1] = (unsigned char)(rpct(60) ? rrange(1, 0x70) : rrange(0, 0x7d));
  if (rpct(30) && g_f.msg[1] <= 0x73) { /* a message that is complete with a good checksum */
    int len = g_f.msg[1] + 2, s = 0;
    for (i = 0; i < len; i++) s += g_f.msg[i];
    g_f.msg[len] = (unsigned char)(-s);
    g_f.count = (short)(g_f.msg[1] + 3 > 0x73 ? 0x73 : g_f.msg[1] + 3);
  }
  for (i = 0; i < 17; i++) g_f.lpf[i] = (short)(rshort() >> 3);
  for (i = 0; i < 5; i++) g_f.hist[i] = (short)(rshort() >> 3);
  g_f.lpfPos = (short)rmod(17);
  g_f.pos = (short)rmod(5);
  g_f.cur = (unsigned short)rmod(2);
  g_f.nPos = (short)rmod(140);
  g_f.nNeg = (short)rmod(20);
  g_f.thr = rpct(50) ? 0 : rrange(-300, 3000);
  g_f.mean = rpct(40) ? 0 : rrange(0, 4000);
  g_f.negAcc = rrange(-100000, 0);
  g_f.posAcc = rrange(0, 200000);
  g_f.run = (short)rrange(-8, 9);
  g_f.runAlt = (short)rrange(-8, 5);
  g_f.quiet = (short)rrange(-8, 30);
  g_f.iirMtd1[0] = (short)(rshort() >> 3); g_f.iirMtd2[1] = (short)(rshort() >> 3);
}
static void sc_modem(void)
{
  int t, c, k;
  cov_def(CV_MOD_TONE, "modem: tone counter up"); cov_def(CV_MOD_NOTONE, "modem: tone counter cleared"); cov_def(CV_MOD_FSD, "modem: FSD path (2)");
  cov_def(CV_MOD_ST0, "modem: framer state 0"); cov_def(CV_MOD_ST1, "modem: framer state 1"); cov_def(CV_MOD_ST2, "modem: framer state 2"); cov_def(CV_MOD_ST3, "modem: framer state 3");
  cov_def(CV_MOD_BYTE, "modem: a byte stored"); cov_def(CV_MOD_CKGOOD, "modem: checksum good (3)"); cov_def(CV_MOD_CKBAD, "modem: checksum bad (-1)"); cov_def(CV_MOD_LEN0, "modem: length 0 (-1)");
  cov_def(CV_MOD_AGC, "modem: gain adapted"); cov_def(CV_MOD_AGCBIG, "modem: gain > 8"); cov_def(CV_MOD_9600, "modem: 9600"); cov_def(CV_MOD_8000, "modem: 8000");
  cov_def(CV_MOD_RET1, "modem: result 1"); cov_def(CV_MOD_RET2, "modem: result 2"); cov_def(CV_MOD_RETM1, "modem: result -1"); cov_def(CV_MOD_RET3, "modem: result 3");
  cov_def(CV_MOD_GAIN0, "modem: gain from 0"); cov_def(CV_MOD_DEFAULTST, "modem: framer state 4 / 5 (default)"); cov_def(CV_MOD_LIMIT, "modem: count at the limit");
  cov_def(CV_MOD_DBG, "modem: debug line");
  /* A: poked states, random blocks */
  for (t = 0; t < 6000; t++) {
    int fs = rpct(50) ? 8000 : 9600;
    int n0 = rpct(60) ? (fs == 8000 ? 160 : 192) : rpct(70) ? rrange(1, 206) : rrange(1, 20);
    char tag[24];
    snprintf(tag, sizeof tag, "A%d", t);
    poke_fsk(fs, n0);
    dsplibs_debug_level = rpct(10) ? 2 : rpct(5) ? 1 : 0;
    for (c = 0; c < 5; c++) {
      int n = c == 0 ? n0 : rpct(70) ? n0 : rrange(1, 206);
      int kind = (int)rmod(10), amp = rpct(10) ? 32767 : rpct(20) ? rrange(10, 300) : rrange(500, 20000);
      if (g_f.tc > 0 && rpct(50)) kind = 0;
      gen_block(kind, n, fs, amp);
      if (g_f.msg[1] > 0x7d) g_f.msg[1] = 0x20; /* a block that completes a message of length > 0x7d would make the blob read behind the object */
      modem_call(tag, t < 20, g_sig, n, fs);
      if (c == 0 || c == 4) dump_fsk(tag, 0, &g_f);
      if (dsplibs_debug_level > 1) cov_hit(CV_MOD_DBG);
    }
    dump_fsk(tag, t < 12, &g_f);
    trial_end(tag, 1);
  }
  dsplibs_debug_level = 0;
  /* B: whole messages, fed in blocks of 160 / 192 / random */
  for (t = 0; t < 200; t++) {
    int fs = (t & 1) ? 9600 : 8000, mdm = rpct(50), bell = rpct(70), pos = 0, calls = 0, rets[8] = { 0 };
    Msg m;
    char tag[24], num[16];
    int blk = t % 3 == 0 ? (fs == 8000 ? 160 : 192) : t % 3 == 1 ? 0 : rrange(40, 206);
    snprintf(tag, sizeof tag, "B%d", t);
    msg_digits(num, rrange(0, 12));
    if (mdm) mdmf(&m, 1 | 2 | (rpct(60) ? 4 : 0) | (rpct(10) ? 8 : 0) | (rpct(10) ? 16 : 0) | (rpct(10) ? 32 : 0), rpct(8)); else sdmf(&m, num, rpct(8));
    bits_message(&m, rrange(160, 320) & ~1, rrange(80, 200), 40, rpct(8) ? rrange(2, m.n) : -1);
    g_sn = 0; g_ph1 = rnd();
    gen_noise((unsigned)rrange(0, 600), rpct(50) ? 3 : 100);
    gen_fsk(fs, bell ? 1200 : 1300, bell ? 2200 : 2100, rrange(1500, 12000), rpct(30) ? rrange(10, 500) : 0, rpct(15) ? rrange(-3000, 3000) : 0, 1200);
    gen_noise(3000, 3);
    fsk_fresh(fs);
    while (pos < (int)g_sn) {
      int n = blk > 0 ? blk : rrange(1, 206), r;
      if (pos + n > (int)g_sn) break;
      r = modem_call(tag, 0, g_sig + pos, n, fs);
      rets[r + 1 > 0 && r + 1 < 8 ? r + 1 : 0]++;
      if (calls < 30 || r != 1) { if ((calls & 7) == 0 || r != 1) dump_fsk(tag, 0, &g_f); }
      pos += n; calls++;
    }
    dump_fsk(tag, t < 8, &g_f);
    emit(t < 8, "%s fs=%d blk=%d calls=%d rets=%d,%d,%d,%d count=%d", tag, fs, blk, calls, rets[0], rets[2], rets[3], rets[4], g_f.count);
    { static char out[640]; memset(out, 0xAA, sizeof out); if (msg_ok_fmt(&g_f)) data_formatted_output(&g_f, out); else g_skip_fmt++; show_str(tag, t < 8, out, 100); emit(0, "%s out=%016llx", tag, fnv_bytes(out, sizeof out)); }
    trial_end(tag, 1);
  }
  /* C: the checksum branch with real and damaged messages */
  for (t = 0; t < 1200; t++) {
    int fs = (t & 1) ? 9600 : 8000, n = fs == 8000 ? 160 : 192, i, len, s = 0;
    char tag[24];
    snprintf(tag, sizeof tag, "C%d", t);
    fsk_fresh(fs);
    g_f.tc = (unsigned short)rrange(thr_hi(n, fs) + 1, 0x7ffe);
    g_f.agc = rrange(0, 5);
    len = rpct(10) ? rrange(0, 3) : rpct(25) ? rrange(0x6d, 0x73) : rrange(1, 0x73);
    for (i = 0; i < 0x78; i++) g_f.msg[i] = (unsigned char)rnd();
    g_f.msg[1] = (unsigned char)len;
    for (i = 0; i < len + 2; i++) s += g_f.msg[i];
    g_f.msg[len + 2] = (unsigned char)(-s + (rpct(30) ? 1 + (int)rmod(255) : 0));
    g_f.count = (short)(rpct(40) ? (len + 3 > 0x73 ? 0x73 : len + 3) : rpct(40) ? (len + 3 > 0x73 ? 0x73 : len + 3) + rrange(-2, 1) : rrange(0, 0x73));
    if (g_f.count < 0) g_f.count = 0;
    if (rpct(8)) g_f.count = (short)(g_f.count > 2 ? rrange(0, 2) : g_f.count);
    if (t % 25 == 3) g_f.count = 0x73;
    gen_block((int)rmod(5), n, fs, 3000);
    modem_call(tag, t < 12, g_sig, n, fs);
    dump_fsk(tag, t < 12, &g_f);
    trial_end(tag, 1);
  }
  (void)k;
  emit(1, "rets %llu %llu %llu %llu", g_ret_hist[0], g_ret_hist[2], g_ret_hist[3], g_ret_hist[4]);
}

/* ------------------------------------------------------------------------------------------------ 3: CID_process end to end */
static void make_stream(int fs, int bell, const Msg *m, int seizure, int mark, int trunc, int noise, int dc, int amp, int lead)
{
  g_nbits = 0;
  bits_message(m, seizure, mark, 40, trunc);
  g_sn = 0;
  g_ph1 = rnd();
  gen_noise((unsigned)lead, rpct(50) ? 3 : 100);
  gen_fsk(fs, bell ? 1200 : 1300, bell ? 2200 : 2100, amp, noise, dc, 1200);
  gen_noise(2500, 3);
}
static void feed_process(const char *tag, int show, CidHandle *h, int chunkmode, int *calls)
{
  unsigned pos = 0;
  int c = 0;
  while (pos < g_sn) {
    int n = chunkmode == 0 ? 160 : chunkmode == 1 ? 192 : chunkmode == 2 ? rrange(1, 600) : chunkmode == 3 ? 1 : chunkmode == 4 ? 600 : chunkmode == 5 ? 191 : chunkmode == 6 ? 193 : 384;
    if (pos + (unsigned)n > g_sn) n = (int)(g_sn - pos);
    if (chunkmode == 3 && pos > 3000) n = 160;
    CID_process(h, g_sig + pos, n);
    if ((c & 15) == 3) { CID_process(h, g_sig, 0); CID_process(h, g_sig, -5); }
    if (n == 1) cov_hit(CV_E2E_CHUNK1);
    if (n > 192) cov_hit(CV_E2E_CHUNKBIG);
    if ((c & 31) == 0) dump_cid(tag, 0, h->cid);
    if (h->cid->fsk->count >= 0x73 && h->cid->fsk->msg[1] > 0x7d) die3("e2e: a message of length > 0x7d reached the checksum (the blob reads behind the object)");
    pos += (unsigned)n;
    c++;
  }
  *calls = c;
  (void)show;
}
static void sc_e2e(void)
{
  int t;
  cov_def(CV_E2E_SDMF, "e2e: SDMF decoded"); cov_def(CV_E2E_MDMF, "e2e: MDMF decoded"); cov_def(CV_E2E_NAME, "e2e: name delivered"); cov_def(CV_E2E_BELL, "e2e: Bell 202");
  cov_def(CV_E2E_V23, "e2e: V.23"); cov_def(CV_E2E_9600, "e2e: 9600 Hz decoded"); cov_def(CV_E2E_8000, "e2e: 8000 Hz decoded"); cov_def(CV_E2E_FAIL, "e2e: no message");
  cov_def(CV_E2E_BADCK, "e2e: bad checksum stream"); cov_def(CV_E2E_TRUNC, "e2e: truncated stream"); cov_def(CV_E2E_NOISE, "e2e: noisy stream"); cov_def(CV_E2E_FMT2, "e2e: unformatted (hex) delivery");
  cov_def(CV_E2E_DBG, "e2e: debug 2"); cov_def(CV_E2E_TTY, "e2e: tty sends"); cov_def(CV_E2E_CHUNK1, "e2e: 1-sample chunks"); cov_def(CV_E2E_CHUNKBIG, "e2e: chunks > 192");
  cov_def(CV_E2E_PO, "e2e: P / O absence delivered"); cov_def(CV_E2E_4STR, "e2e: four strings delivered"); cov_def(CV_E2E_SUCC2, "e2e: repeated success");
  for (t = 0; t < 360; t++) {
    int fs = (t % 5 < 3) ? 8000 : 9600, bell = rpct(70), fmt = t % 4, dbg = (t % 9 == 0) ? 2 : (t % 9 == 4) ? 1 : 0, kind = t % 11, chunk = t % 8, calls, bad = 0, trunc = -1, noise = 0;
    int seizure = rrange(200, 320) & ~1, mark = rrange(80, 190), lead = rrange(0, 1500), amp = rpct(15) ? rrange(500, 3000) : rrange(4000, 14000), dc = rpct(15) ? rrange(-2000, 2000) : 0;
    Msg m;
    char tag[24], num[20];
    unsigned c0;
    CidHandle *h;
    snprintf(tag, sizeof tag, "E%d", t);
    msg_digits(num, rrange(3, 12));
    switch (kind) {
    case 0: sdmf(&m, num, 0); break;
    case 1: sdmf(&m, rpct(50) ? "P" : "O", 0); break;
    case 2: sdmf(&m, "", 0); break;
    case 3: mdmf(&m, 1 | 2, 0); break;
    case 4: mdmf(&m, 1 | 2 | 4, 0); break;
    case 5: mdmf(&m, 1 | 8 | 16, 0); break;
    case 6: mdmf(&m, 1 | 2 | 4 | 32, 0); break;
    case 7: if (rpct(50)) sdmf(&m, num, 1); else mdmf(&m, 1 | 2 | 4, 1); bad = 1; break;
    case 8: mdmf(&m, 1 | 2 | 4, 0); trunc = rrange(3, m.n - 2); break;
    case 9: sdmf(&m, num, 0); seizure = 0; mark = rrange(40, 100); break; /* no seizure */
    default: if (rpct(50)) sdmf(&m, num, 0); else mdmf(&m, 1 | 2 | 4, 0); noise = rrange(300, 2500); break;
    }
    if (noise) cov_hit(CV_E2E_NOISE);
    if (bad) cov_hit(CV_E2E_BADCK);
    if (trunc >= 0) cov_hit(CV_E2E_TRUNC);
    dsplibs_debug_level = (unsigned)dbg;
    if (dbg > 1) cov_hit(CV_E2E_DBG);
    make_stream(fs, bell, &m, seizure, mark, trunc, noise, dc, amp, lead);
    h = CID_create(&g_modem_tag[t & 3], fs, fmt);
    if (!h) die3("CID_create failed");
    g_tty[0] = 0;
    g_tty_len = 0;
    c0 = g_tty_calls;
    feed_process(tag, t < 12, h, chunk, &calls);
    emit(1, "%s fs=%d bell=%d fmt=%d kind=%d chunk=%d dbg=%d samples=%u calls=%d tty=%u", tag, fs, bell, fmt, kind, chunk, dbg, g_sn, calls, g_tty_calls - c0);
    show_str(tag, 1, g_tty, 300);
    dump_cid(tag, t < 12, h->cid);
    if (g_tty_calls > c0) {
      cov_hit(CV_E2E_TTY);
      if (strstr(g_tty, "DATE = ")) {
        cov_hit(m.b[0] == 0x04 ? CV_E2E_SDMF : CV_E2E_MDMF);
        cov_hit(fs == 9600 ? CV_E2E_9600 : CV_E2E_8000);
        cov_hit(bell ? CV_E2E_BELL : CV_E2E_V23);
        if (strstr(g_tty, "NAME = ")) cov_hit(CV_E2E_NAME);
        if (strstr(g_tty, "NMBR = P") || strstr(g_tty, "NMBR = O")) cov_hit(CV_E2E_PO);
        if (g_tty_calls - c0 >= 8) cov_hit(CV_E2E_4STR);
        if (g_tty_calls - c0 >= 16) cov_hit(CV_E2E_SUCC2);
      } else if (fmt == 2) {
        cov_hit(CV_E2E_FMT2);
      }
    } else {
      cov_hit(CV_E2E_FAIL);
    }
    CID_delete(h);
    trial_end(tag, 0);
  }
  dsplibs_debug_level = 0;
}

/* ------------------------------------------------------------------------------------------------ 4: cid_progress in every mode */
static void sc_prog(void)
{
  int t;
  static const int modes[10] = { 0, 1, 5, 5, 0, 1, 2, 3, -1, 6 };
  cov_def(CV_PRG_FSK, "prog: mode 0"); cov_def(CV_PRG_DTMF, "prog: mode 1"); cov_def(CV_PRG_AUTO, "prog: mode 5"); cov_def(CV_PRG_DONE2, "prog: mode 2"); cov_def(CV_PRG_DONE3, "prog: mode 3");
  cov_def(CV_PRG_R0, "prog: result 0"); cov_def(CV_PRG_R1, "prog: result 1"); cov_def(CV_PRG_R2, "prog: result 2"); cov_def(CV_PRG_R3, "prog: result 3");
  cov_def(CV_PRG_DTMFFAIL, "prog: DTMF failure (2)"); cov_def(CV_PRG_FSKFAIL, "prog: FSK failure"); cov_def(CV_PRG_DIGITS, "prog: DTMF digits delivered"); cov_def(CV_PRG_FRAME160, "prog: frame 160");
  cov_def(CV_PRG_FRAME192, "prog: frame 192"); cov_def(CV_PRG_MODE3FAIL, "prog: mode 3 failure"); cov_def(CV_PRG_DBG, "prog: debug lines"); cov_def(CV_PRG_STRINGS, "prog: strings after success");
  cov_def(CV_PRG_AUTOFSK, "prog: automatic mode, FSK message"); cov_def(CV_PRG_AUTODTMF, "prog: automatic mode, DTMF message"); cov_def(CV_PRG_RESET, "prog: FSK reset in the loop");
  for (t = 0; t < 420; t++) {
    int mode = modes[t % 10], cm = mode == 0 ? 0 : mode == 1 ? 1 : 5, fs = rpct(50) ? 8000 : 9600, fmt = t % 4, kind = (int)rmod(5), chunk = (int)rmod(6);
    Msg m;
    char tag[24], num[16];
    CidObj *c;
    unsigned pos = 0;
    int calls = 0, res[4] = { 0, 0, 0, 0 }, nstr = 0;
    snprintf(tag, sizeof tag, "P%d", t);
    dsplibs_debug_level = (t % 8 == 0 || t % 5 == 2) ? 2 : (t % 8 == 3) ? 1 : 0;
    if (dsplibs_debug_level > 1) cov_hit(CV_PRG_DBG);
    msg_digits(num, rrange(3, 12));
    if (rpct(50)) sdmf(&m, num, rpct(10)); else mdmf(&m, 1 | 2 | 4, rpct(10));
    g_sn = 0; g_ph1 = rnd(); g_ph2 = rnd();
    if (kind == 0 || kind == 2) { /* FSK message (then maybe a DTMF one) */
      int bell = rpct(70);
      bits_message(&m, rrange(200, 300) & ~1, rrange(80, 190), 40, -1);
      gen_noise((unsigned)rrange(0, 800), 3);
      gen_fsk(fs, bell ? 1200 : 1300, bell ? 2200 : 2100, rrange(4000, 12000), 0, 0, 1200);
      gen_noise(2500, 3);
    }
    if (kind == 1 || kind == 2 || kind == 3) dtmf_message(fs, rrange(3, 12), 1, kind != 3, 0, rrange(2500, 9000), 0);
    if (kind == 4) gen_noise((unsigned)(fs * 2), rpct(50) ? 200 : 3000);
    if (kind == 3) gen_noise((unsigned)(fs * rrange(1, 4)), 3);
    c = cid_create(NULL, fmt, cm);
    c->mode = mode;
    if (mode == 0) cov_hit(CV_PRG_FSK); else if (mode == 1) cov_hit(CV_PRG_DTMF); else if (mode == 5) cov_hit(CV_PRG_AUTO); else if (mode == 2) cov_hit(CV_PRG_DONE2); else if (mode == 3) cov_hit(CV_PRG_DONE3);
    if (c->dtmf) c->dtmf->fs = (unsigned short)fs;
    if (c->fsk) c->fsk->fs = (unsigned short)fs;
    if (fs == 8000) cov_hit(CV_PRG_FRAME160); else cov_hit(CV_PRG_FRAME192);
    if (mode == 3 || mode == 2) { /* a done state: the DTMF detector continues from a poked phase */
      if (c->dtmf) { c->dtmf->phase = (short)rrange(0, 3); c->dtmf->nDig = (short)rrange(-1, 5); }
    }
    while (pos < g_sn) {
      int n = chunk == 0 ? 160 : chunk == 1 ? 192 : chunk == 2 ? rrange(1, 700) : chunk == 3 ? 37 : chunk == 4 ? 1000 : 400, r;
      short pn;
      int preCnt;
      if (pos + (unsigned)n > g_sn) n = (int)(g_sn - pos);
      pn = (short)n;
      preCnt = c->fsk ? c->fsk->count : 0;
      r = cid_progress(c, g_sig + pos, 0, &pn);
      if (cm == 5 && preCnt > 2 && c->fsk->count == 0) cov_hit(CV_PRG_RESET);
      if (pn != 0) die3("cid_progress did not clear the count");
      emit(calls < 40 && t < 10, "%s call=%d n=%d -> %d mode=%d fill=%d", tag, calls, n, r, c->mode, c->fill);
      res[r < 0 || r > 3 ? 0 : r]++;
      if (r == 0) cov_hit(CV_PRG_R0); else if (r == 1) cov_hit(CV_PRG_R1); else if (r == 2) cov_hit(CV_PRG_R2); else if (r == 3) cov_hit(CV_PRG_R3);
      if (r == 2 && cm != 0) cov_hit(CV_PRG_DTMFFAIL);
      if (r == 2 && (mode == 0)) cov_hit(CV_PRG_FSKFAIL);
      if (r == 2 && c->mode == 3) cov_hit(CV_PRG_MODE3FAIL);
      if (r != 0 || (calls & 15) == 0) dump_cid(tag, 0, c);
      if (r == 1) {
        char *s = cid_get_strings(c);
        show_str(tag, t < 40, s, 140);
        emit(0, "%s str=%016llx", tag, fnv_bytes(s, 600));
        nstr++;
        cov_hit(CV_PRG_STRINGS);
        if (c->mode == 3 || c->mode == 1) cov_hit(CV_PRG_DIGITS);
        if (cm == 5 && c->mode == 2) cov_hit(CV_PRG_AUTOFSK);
        if (cm == 5 && c->mode == 3) cov_hit(CV_PRG_AUTODTMF);
      }
      pos += (unsigned)n;
      calls++;
    }
    dump_cid(tag, t < 10, c);
    emit(t < 10, "%s mode=%d fs=%d kind=%d chunk=%d calls=%d res=%d,%d,%d,%d strs=%d", tag, mode, fs, kind, chunk, calls, res[0], res[1], res[2], res[3], nstr);
    c->mode = cm;
    cid_delete(c);
    trial_end(tag, 0);
  }
  dsplibs_debug_level = 0;
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("R13_SEED");
  g_verbose = getenv("R13_VERBOSE") != NULL;
  if (seed) g_lcg ^= (unsigned)strtoul(seed, NULL, 0) * 2654435761u;
  g_lcg ^= (unsigned)sc * 0x9e3779b9u;
  switch (sc) {
  case 0: sc_life(); break;
  case 1: sc_str(); break;
  case 2: sc_modem(); break;
  case 3: sc_e2e(); break;
  case 4: sc_prog(); break;
  default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  cov_finish();
  return 0;
}
