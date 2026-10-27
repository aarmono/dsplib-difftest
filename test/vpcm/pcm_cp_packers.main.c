/*
 * Differential test harness for port chunk P12 (src/vpcm/pcm_cp_packers.cpp): the free functions around the V.90 CP frame and the V90MappingParams block of dsplibs.o -
 * V90CPPacker, float2Bits (both overloads), displaySpectralParams, getDataBitRate / setDataBitRate, getConstellationsIndex, getConstellationMask / getCodecConstellationMask,
 * setConstellationMask / setCodecConstellationMask (11 functions).
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/pcm_cp_packers.sh against the blob (a copy of dsplibs.o in which `edprintf` is WEAK so that the recorder
 * below wins; `dsplibs_debug_printf` and the allocator are the harness'), against the port (-O0 / -O2; the P11 port object supplies the tables fltTable_1 / fltTable_2) and for
 * x86-64 (diffed against the 32-bit BLOB's output = the golden, test/golden/vpcm/pcm_cp_packers.blob32.out).  The functions are reached through __asm__ labels (C names and the C++ mangled
 * ones); the structures are the C mirrors generated from test/vpcm/v92_cp.layout (V90MappingParams, tagV90AdditionalCPinfo: no pointers).
 *
 * What is compared: every return value, every output buffer (the `short` frame, the mask words, the bit arrays, the mapping block after every call), every edprintf /
 * dsplibs_debug_printf call (the text).  INDEPENDENT MODELS (written from the frame definition, not from the port; the same style as test/vpcm/v92_cp.main.c): the V90 frame writer
 * (sequential bits with the 17-bit start-bit grid inserted automatically, the Q-format fields in integer units of 2^-13 / 2^-6), a standard bit-serial CRC-CCITT, the dedupe of the
 * six lists, list <-> mask packing, the "%c%d.%06d" digits in double arithmetic (exact for every float).
 *
 * Scenarios (each its own process): 0 float2Bits (short* and unsigned char*) incl. the range-check messages, 1 get/setDataBitRate + getConstellationsIndex + the four mask
 * functions (+ round trips), 2 V90CPPacker vs the frame model (+ the debug dump), 3 displaySpectralParams.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "v92_cp_layout.h"

#define L(m) __asm__(m)

extern void f2b_s(float, short *, int) L("_Z10float2BitsfPsi");
extern void f2b_u(float, unsigned char *, int) L("_Z10float2BitsfPhi");
extern int pk(MapPar90 *, AddCp *, short *, int) L("_Z11V90CPPackerP16V90MappingParamsP22tagV90AdditionalCPinfoPsi");
extern void disp(MapPar90 *) L("displaySpectralParams");
extern int get_rate(MapPar90 *, int) L("getDataBitRate");
extern void set_rate(MapPar90 *, int, int) L("setDataBitRate");
extern int get_idx(MapPar90 *, unsigned *) L("getConstellationsIndex");
extern void get_mask(MapPar90 *, int, unsigned short *) L("getConstellationMask");
extern void get_cmask(MapPar90 *, int, unsigned short *) L("getCodecConstellationMask");
extern void set_mask(MapPar90 *, int, const short *) L("setConstellationMask");
extern void set_cmask(MapPar90 *, int, const short *) L("setCodecConstellationMask");
extern unsigned int dsplibs_debug_level;
extern float fltTable1[7];    /* the blob's GLOBAL data tables, byte-compared through the output (the port defines its own copies) */
extern float fltTable2[16];

/* ------------------------------------------------------------------------------------------------ recorders (edprintf / dsplibs_debug_printf) */
#define TXTMAX 64
static unsigned g_nedp, g_ndbg;
static char g_ed_text[TXTMAX][300], g_dbg_text[TXTMAX][300];
static unsigned g_ed_in_call, g_dbg_in_call;
static int g_show;
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
void edprintf(const char *fmt, ...) {
  char buf[600];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (g_ed_in_call < TXTMAX) { g_ed_text[g_ed_in_call][0] = 0; esc_cat(g_ed_text[g_ed_in_call], 300, buf); }
  h_emit(g_show, "edprintf %s", g_ed_in_call < TXTMAX ? g_ed_text[g_ed_in_call] : "(many)");
  g_ed_in_call++;
  g_nedp++;
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (g_dbg_in_call < TXTMAX) { g_dbg_text[g_dbg_in_call][0] = 0; esc_cat(g_dbg_text[g_dbg_in_call], 300, buf); }
  h_emit(g_show, "dbgprintf %s", g_dbg_in_call < TXTMAX ? g_dbg_text[g_dbg_in_call] : "(many)");
  g_dbg_in_call++;
  g_ndbg++;
  return 0;
}

static void *onew(size_t n) { void *p = h_obj_new(n); memset(p, 0xa5, n); return p; }
static uint64_t fnv_upd(uint64_t h, const void *p, size_t n) {
  const uint8_t *q = (const uint8_t *)p;
  size_t i;
  for (i = 0; i < n; i++) { h ^= q[i]; h *= 0x100000001b3ull; }
  return h;
}
#define FNV0 0xcbf29ce484222325ull
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)
static void cov_def(int id, const char *name) { h_cov_define(id, name); }
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }
#define COV(id) h_cov_hit(id)
static uint8_t gf_byte(uint32_t seed, uint32_t idx) { uint32_t x = (seed ^ (idx * 0x9e3779b1u)) * 2654435761u; x ^= x >> 15; x *= 2246822519u; return (uint8_t)(x >> 24); }
static void garbage_fill(void *o, size_t n, uint32_t seed) { size_t k; uint8_t *p = (uint8_t *)o; for (k = 0; k < n; k++) p[k] = gf_byte(seed, (uint32_t)k); }

static void *onew_canary(size_t n) { void *p = onew(n + 32); memset((uint8_t *)p + n, 0xc3, 32); return p; }
static void canary_check(const void *p, size_t n, const char *what) {
  unsigned c;
  for (c = 0; c < 32; c++) if (((const uint8_t *)p)[n + c] != 0xc3) h_die3("%s wrote past the %zu-byte block (byte +%u)", what, n, c);
}
static HFld g_f90[24];
static int g_nf90;
static unsigned long g_dumpctr;
#define DUMP90(tag, o) h_dump_obj(h_sample(&g_dumpctr, 12, 97), (tag), (o), g_f90, g_nf90)

/* ------------------------------------------------------------------------------------------------ models */
static uint32_t rfloat(int wide) {
  uint32_t b;
  switch (h_rmod(11)) {
    case 0: return 0;
    case 1: return 0x80000000u;
    case 2: { static const uint32_t sp[] = {0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x7f7fffffu, 0xff7fffffu, 0x00000001u, 0x80000001u, 0x3f800000u, 0xbf800000u, 0x41000000u, 0x40ffffffu, 0x3f7fffffu,
                                    0x00800000u, 0x7fa00000u, 0xffa00000u, 0x7f7ffff0u, 0x3f800001u, 0x41000001u, 0xc1000000u, 0xbf800001u, 0xbf7fffffu, 0x3b800000u, 0x3c000000u, 0xb8000000u, 0x4f000000u, 0xcf000000u, 0x4effffffu};
      return sp[h_rmod(sizeof sp / sizeof sp[0])]; }
    case 3: case 4: { float f = (float)h_rmod(wide ? 65536u : 128u); int k, sh = wide ? 13 : 6; for (k = 0; k < sh; k++) f *= 0.5f; if (h_rpct(40)) f = -f; return h_bits_of(f); }
    case 5: { float f = (float)h_rmod(wide ? 1u << 20 : 1u << 12); int k, sh = wide ? 13 : 6; for (k = 0; k < sh; k++) f *= 0.5f; if (h_rpct(50)) f = -f; return h_bits_of(f); }
    case 6: return h_rnd();
    case 7: { float f = (float)h_rmod(2000); int k; for (k = 0; k < 10; k++) f *= 0.5f; if (h_rpct(50)) f = -f; return h_bits_of(f); }
    case 8: { float f = (float)h_rnd(); int k; for (k = h_rmod(40); k > 0; k--) f *= 0.5f; if (h_rpct(50)) f = -f; return h_bits_of(f); }
    default: b = (h_rnd() & 0x80000000u) | (0x3c000000u + (h_rnd() & 0x03ffffffu)); return b;
  }
}
static float bf(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }

/* the model of a Q-format field: bits (smallest weight first) of |f| in integer units, `n` entries, weights w[]; returns bits in out[0..n-1] with out[k] = bit of weight index n-1-k */
static void model_q(float f, const unsigned *w, int n, int shift, uint8_t *out) {
  double v, scale = 1.0;
  uint8_t bit[16];
  int i;
  for (i = 0; i < shift; i++) scale *= 2.0;
  v = (f < 0 ? -(double)f : (double)f);
  if (f != f) v = (double)f;
  v = v * scale;
  for (i = 0; i < n; i++) { if ((double)w[i] > v) bit[i] = 0; else { bit[i] = 1; v -= (double)w[i]; } }
  for (i = 0; i < n; i++) out[i] = bit[n - 1 - i];   /* position 0 = the entry with the smallest weight */
}
static const unsigned W2[16] = {32768, 16384, 8192, 4096, 2048, 1024, 512, 256, 128, 64, 16, 16, 8, 4, 2, 1};
static const unsigned W1[7] = {64, 32, 16, 8, 4, 2, 1};

static void model_crc(const short *b, unsigned endp, uint8_t out[16]) {
  uint16_t r = 0xffff;
  unsigned p, k;
  for (p = 0x12; p < endp; p++) {
    unsigned fb;
    if (p % 17 == 0) continue;
    fb = ((r >> 15) ^ ((unsigned)b[p] & 1u)) & 1u;
    r = (uint16_t)(r << 1);
    if (fb) r ^= 0x1021;
  }
  for (k = 0; k < 16; k++) out[k] = (uint8_t)((r >> (15 - k)) & 1);
}

typedef struct { uint8_t listA[6][128], listB[6][128]; uint32_t cnt[6]; } Lists;
static void rnd_lists(Lists *l, int dup_kind) {
  int g, k;
  memset(l, 0, sizeof *l);
  for (g = 0; g < 6; g++) {
    unsigned n = h_rpct(15) ? 0 : h_rmod(h_rpct(20) ? 129 : 30);
    l->cnt[g] = n;
    for (k = 0; k < 128; k++) { l->listA[g][k] = (uint8_t)h_rmod(128); l->listB[g][k] = (uint8_t)h_rmod(128); }
  }
  if (dup_kind) {
    int n = 1 + h_rmod(4);
    for (k = 0; k < n; k++) {
      int a = h_rmod(6), b = h_rmod(6);
      unsigned how = h_rmod(4);
      if (a == b) continue;
      l->cnt[b] = l->cnt[a];
      memcpy(l->listA[b], l->listA[a], 128);
      if (how != 1) memcpy(l->listB[b], l->listB[a], 128);
      if (how == 2 && l->cnt[a]) l->listA[b][l->cnt[a] - 1] ^= 1;
      if (how == 3 && l->cnt[a]) l->listB[b][l->cnt[a] - 1] ^= 4;
    }
  }
}
static void rnd_mp(MapPar90 *mp, int dup) {
  Lists l;
  int g, k;
  garbage_fill(mp, sizeof *mp, h_rnd());
  rnd_lists(&l, dup);
  for (g = 0; g < 6; g++) { memcpy(mp->listA_ + 128 * g, l.listA[g], 128); memcpy(mp->listB_ + 128 * g, l.listB[g], 128); mp->cnt_[g] = l.cnt[g]; }
  mp->w0_ = h_rpct(60) ? 8 + h_rmod(60) : h_rnd();
  mp->flag_ = h_rpct(50) ? h_rmod(2) : h_rnd();
  mp->w620_ = h_rpct(60) ? h_rmod(4) : h_rnd();
  mp->w624_ = h_rpct(60) ? h_rmod(4) : h_rnd();
  { uint32_t b[4]; for (k = 0; k < 4; k++) b[k] = rfloat(0); memcpy(&mp->f628_, &b[0], 4); memcpy(&mp->f62c_, &b[1], 4); memcpy(&mp->f630_, &b[2], 4); memcpy(&mp->f634_, &b[3], 4); }
}
static void model_mask(uint16_t out[8], const uint8_t *list, unsigned n) {
  unsigned e;
  memset(out, 0, 16);
  for (e = 0; e < n; e++) out[list[e] >> 4] |= (uint16_t)(1u << (list[e] & 15));
}
/* the dedupe model: returns n, idx[], grp[] */
static unsigned model_dedupe(const MapPar90 *mp, unsigned *idx, unsigned *grp) {
  unsigned n = 1, g, b;
  idx[0] = 0; grp[0] = 0;
  for (g = 1; g < 6; g++) {
    int found = 0;
    for (b = 0; b < n && !found; b++) {
      unsigned d = idx[b];
      if (mp->cnt_[g] != mp->cnt_[d]) continue;
      if (!memcmp(mp->listA_ + 128 * g, mp->listA_ + 128 * d, mp->cnt_[g]) && !memcmp(mp->listB_ + 128 * g, mp->listB_ + 128 * d, mp->cnt_[g])) { grp[g] = grp[d]; found = 1; }
    }
    if (!found) { idx[n] = g; grp[g] = n; n++; }
  }
  return n;
}

/* ------------------------------------------------------------------------------------------------ the %c%d.%06d digits in double arithmetic */
static void split_model(double v, int *sg, int *ip, int *fr) {
  double a = v < 0 ? -v : v;
  long long t, fi;
  double fq;
  if (v != v || !(v > -2147483649.0 && v < 2147483648.0)) t = (long long)(int)0x80000000u;
  else t = (long long)v;
  *ip = (a != a || !(a < 2147483648.0)) ? (int)0x80000000u : (int)(long long)a;
  fq = (v - (double)t) * 1000000.0;
  if (fq != fq || !(fq > -2147483649.0 && fq < 2147483648.0)) fi = (long long)(int)0x80000000u; else fi = (long long)fq;
  *fr = (int)(unsigned)(fi < 0 ? (unsigned)(-fi) : (unsigned)fi);
  if (fi == (long long)(int)0x80000000u) *fr = (int)0x80000000u;
  *sg = (v <= 0.0) ? 0x2d : 0x2b;
}

enum { CV_F_S0 = 0, CV_F_S1, CV_F_U0, CV_F_U1, CV_F_OTHERMODE, CV_F_MSG0, CV_F_MSG1, CV_F_NOMSG, CV_F_LVL0, CV_F_NAN, CV_F_INF, CV_F_NEG, CV_F_ZERO, CV_F_NEGZERO, CV_F_EXACT, CV_F_BIG, CV_F_EDGE8, CV_F_EDGE1, CV_F_EDGEM1 };
enum { CV_R_GET = 0, CV_R_SET, CV_R_FLAG0, CV_R_FLAG1, CV_R_IDX, CV_R_IDX_DUP, CV_R_IDX_NODUP, CV_R_IDX_PARTIAL, CV_R_MASK, CV_R_CMASK, CV_R_SETMASK, CV_R_SETCMASK, CV_R_GHI, CV_R_RT, CV_R_CNT0, CV_R_G6PLUS };
enum { CV_K_CALL = 0, CV_K_FLAGB, CV_K_NOFLAGB, CV_K_N1, CV_K_N2, CV_K_N6, CV_K_CLEARDOWN, CV_K_W4_0, CV_K_W4_1, CV_K_MSGS, CV_K_BIGFIELDS, CV_K_NOCLEAR, CV_K_LVL0, CV_K_LVL2, CV_K_S, CV_K_QUOTE, CV_K_W14NEG, CV_K_CNT0, CV_K_FLAG2 };
enum { CV_D_CALL = 0, CV_D_NAN, CV_D_BIG, CV_D_NEG, CV_D_ZERO, CV_D_FRAC, CV_D_PLUS };

/* ================================================================================================ scenario 0: float2Bits */
static void sc_f2b(void) {
  int trial, k;
  cov_def(CV_F_S0, "short mode 0"); cov_def(CV_F_S1, "short mode 1"); cov_def(CV_F_U0, "uchar mode 0"); cov_def(CV_F_U1, "uchar mode 1"); cov_def(CV_F_OTHERMODE, "other mode (no-op)");
  cov_def(CV_F_MSG0, "Q3.13 message"); cov_def(CV_F_MSG1, "Q1.6 message"); cov_def(CV_F_NOMSG, "in range, level 2: no message"); cov_def(CV_F_LVL0, "out of range at level < 2"); cov_def(CV_F_NAN, "NaN");
  cov_def(CV_F_INF, "inf"); cov_def(CV_F_NEG, "negative"); cov_def(CV_F_ZERO, "+0"); cov_def(CV_F_NEGZERO, "-0"); cov_def(CV_F_EXACT, "exactly on the grid"); cov_def(CV_F_BIG, "|f| >= 8");
  cov_def(CV_F_EDGE8, "f == 8"); cov_def(CV_F_EDGE1, "f == 1 (mode 1)"); cov_def(CV_F_EDGEM1, "f == -1 (mode 1)");
  h_scenario_begin(0);
  relax_reserved();
  h_emit(1, "tables fltTable1 %016llx fltTable2 %016llx", (unsigned long long)fnv_upd(FNV0, fltTable1, sizeof fltTable1), (unsigned long long)fnv_upd(FNV0, fltTable2, sizeof fltTable2));
  for (trial = 0; trial < 12000; trial++) {
    uint32_t fb;
    float f;
    unsigned mode = h_rpct(45) ? 0 : (h_rpct(80) ? 1 : (h_rpct(50) ? 2 : h_rnd()));
    unsigned lvl = h_rpct(8) ? (h_rpct(50) ? 0x80000000u + h_rmod(3) : 0xffffffffu - h_rmod(2)) : (h_rpct(50) ? 2 : h_rmod(5));
    int uc = h_rpct(25);
    short sb[24], swant[24];
    unsigned char ub[24], uwant[24];
    uint8_t q[16];
    int nmsg, wantmsg = 0;
    fb = rfloat(mode == 0);
    if (mode == 1 && h_rpct(30)) fb = h_rpct(50) ? 0x3f800000u : (h_rpct(50) ? 0xbf800000u : (h_rpct(50) ? 0x3f800001u : 0xbf800001u));
    if (mode == 0 && h_rpct(15)) fb = h_rpct(50) ? 0x41000000u : 0x41000001u;
    f = bf(fb);
    for (k = 0; k < 24; k++) { sb[k] = (short)(0x5a00 + k); ub[k] = (uint8_t)(0xa0 + k); }
    memcpy(swant, sb, sizeof sb); memcpy(uwant, ub, sizeof ub);
    dsplibs_debug_level = lvl;
    g_dbg_in_call = 0;
    if (!uc) {
      if (mode == 0) {
        model_q(f, W2, 16, 13, q);
        for (k = 0; k < 16; k++) swant[2 + k] = q[k];
        wantmsg = (f > 8.0f || !(f >= 0.0f));
      } else if (mode == 1) {
        model_q(f, W1, 7, 6, q);
        for (k = 0; k < 7; k++) swant[2 + k] = q[k];
        swant[2 + 7] = !(f >= 0.0f);
        wantmsg = (f > 1.0f || !(f >= -1.0f));
      }
      f2b_s(f, sb + 2, (int)mode);
      if (mode == 0) COV(CV_F_S0); else if (mode == 1) COV(CV_F_S1); else COV(CV_F_OTHERMODE);
      for (k = 0; k < 24; k++) if (sb[k] != swant[k]) h_die3("float2Bits(short) %08x mode %u: element %d is %d, model %d", fb, mode, k, sb[k], swant[k]);
      h_emit(0, "f2b s %08x %u lvl %u:%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d dbg %u", fb, mode, lvl, sb[2], sb[3], sb[4], sb[5], sb[6], sb[7], sb[8], sb[9], sb[10], sb[11], sb[12], sb[13], sb[14], sb[15], sb[16], sb[17], sb[18], g_dbg_in_call);
    } else {
      if (mode == 0) { model_q(f, W2, 16, 13, q); for (k = 0; k < 16; k++) uwant[2 + k] = q[k]; }
      else if (mode == 1) { model_q(f, W1, 7, 6, q); for (k = 0; k < 7; k++) uwant[2 + k] = q[k]; uwant[2 + 7] = !(f >= 0.0f); }
      f2b_u(f, ub + 2, (int)mode);
      if (mode == 0) COV(CV_F_U0); else if (mode == 1) COV(CV_F_U1); else COV(CV_F_OTHERMODE);
      for (k = 0; k < 24; k++) if (ub[k] != uwant[k]) h_die3("float2Bits(uchar) %08x mode %u: element %d is %d, model %d", fb, mode, k, ub[k], uwant[k]);
      h_emit(0, "f2b u %08x %u:%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x dbg %u", fb, mode, ub[2], ub[3], ub[4], ub[5], ub[6], ub[7], ub[8], ub[9], ub[10], ub[11], ub[12], ub[13], ub[14], ub[15], ub[16], ub[17], ub[18], ub[19], g_dbg_in_call);
      wantmsg = 0;
      if (g_dbg_in_call) h_die3("float2Bits(uchar) printed");
    }
    nmsg = (int)g_dbg_in_call;
    if (!uc && (mode == 0 || mode == 1)) {
      int expn = (wantmsg && lvl > 1) ? 1 : 0;
      if (nmsg != expn) h_die3("float2Bits %08x mode %u level %u: %d messages, model %d", fb, mode, lvl, nmsg, expn);
      if (expn) { if (strcmp(g_dbg_text[0], mode == 0 ? "Q3.13 format violation!!\\r\\n" : "Q1.6 format violation!!\\r\\n")) h_die3("message text '%s'", g_dbg_text[0]); if (mode == 0) COV(CV_F_MSG0); else COV(CV_F_MSG1); }
      else if (wantmsg) COV(CV_F_LVL0); else if (lvl > 1) COV(CV_F_NOMSG);
    } else if (!uc && nmsg) h_die3("float2Bits(short) mode %u printed", mode);
    if (f != f) COV(CV_F_NAN); else if (f > 3.4e38f || f < -3.4e38f) COV(CV_F_INF); else if (f < 0) COV(CV_F_NEG); else if (fb == 0) COV(CV_F_ZERO);
    if (fb == 0x80000000u) COV(CV_F_NEGZERO);
    if (f == 8.0f) COV(CV_F_EDGE8);
    if (mode == 1 && f == 1.0f) COV(CV_F_EDGE1);
    if (mode == 1 && f == -1.0f) COV(CV_F_EDGEM1);
    if (f >= 8.0f || f <= -8.0f) COV(CV_F_BIG);
    { float a = f < 0 ? -f : f; if (a == a && a < 8.0f) { double u = (double)a * 8192.0; if (u == (double)(long long)u) COV(CV_F_EXACT); } }
  }
}

/* ================================================================================================ scenario 1: rate, index, masks */
static void sc_masks(void) {
  int trial, k, g;
  cov_def(CV_R_GET, "getDataBitRate"); cov_def(CV_R_SET, "setDataBitRate"); cov_def(CV_R_FLAG0, "flag 0 (offset 8)"); cov_def(CV_R_FLAG1, "flag != 0 (offset 0x14)"); cov_def(CV_R_IDX, "getConstellationsIndex");
  cov_def(CV_R_IDX_DUP, "a duplicate found"); cov_def(CV_R_IDX_NODUP, "all distinct"); cov_def(CV_R_IDX_PARTIAL, "equal in A only"); cov_def(CV_R_MASK, "getConstellationMask"); cov_def(CV_R_CMASK, "getCodecConstellationMask");
  cov_def(CV_R_SETMASK, "setConstellationMask"); cov_def(CV_R_SETCMASK, "setCodecConstellationMask"); cov_def(CV_R_GHI, "group number >= 6 (maps to 0)"); cov_def(CV_R_RT, "round trip"); cov_def(CV_R_CNT0, "empty list");
  cov_def(CV_R_G6PLUS, "group 6 / 7 / big");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 4000; trial++) {
    MapPar90 *mp;
    uint8_t pre[sizeof(MapPar90)];
    unsigned which = h_rmod(6);
    h_pool_reset();
    mp = (MapPar90 *)onew_canary(sizeof(MapPar90));
    rnd_mp(mp, trial & 1);
    memcpy(pre, mp, sizeof pre);
    if (which == 0) {   /* data rate */
      int flag = h_rpct(50) ? 0 : (h_rpct(50) ? 1 : (int)h_rnd());
      int r = get_rate(mp, flag);
      COV(CV_R_GET);
      EXPECT_EQ("getDataBitRate", (uint32_t)r, mp->w0_ - (flag ? 0x14u : 8u));
      if (flag) COV(CV_R_FLAG1); else COV(CV_R_FLAG0);
      { int nr = h_rpct(50) ? (int)h_rmod(200) : (int)h_rnd(); set_rate(mp, flag, nr); COV(CV_R_SET); EXPECT_EQ("setDataBitRate", mp->w0_, (uint32_t)nr + (flag ? 0x14u : 8u)); }
      pre[0] = 0; ((MapPar90 *)pre)->w0_ = mp->w0_;
      if (memcmp(pre + 4, (uint8_t *)mp + 4, sizeof(MapPar90) - 4)) h_die3("rate functions changed other fields");
      h_emit(0, "rate %d flag %d w0 %u", r, flag, mp->w0_);
    } else if (which == 1) {   /* getConstellationsIndex */
      unsigned out[8], idx[6], grp[6], n;
      int rn;
      for (k = 0; k < 8; k++) out[k] = 0xdead0000u + (unsigned)k;
      rn = get_idx(mp, out);
      COV(CV_R_IDX);
      n = model_dedupe((MapPar90 *)pre, idx, grp);
      EXPECT_EQ("getConstellationsIndex n", (unsigned)rn, n);
      for (g = 0; g < 6; g++) EXPECT_EQ("getConstellationsIndex out", out[g], grp[g]);
      EXPECT_EQ("getConstellationsIndex guard", out[6], 0xdead0006u);
      for (g = 0; g < (int)n; g++) EXPECT_EQ("getConstellationsIndex idx_", mp->idx_[g], idx[g]);
      { MapPar90 *w = (MapPar90 *)pre; for (g = 0; g < (int)n; g++) w->idx_[g] = idx[g]; if (memcmp(w, mp, sizeof *mp)) h_die3("getConstellationsIndex changed something else"); }
      if (n == 6) COV(CV_R_IDX_NODUP); else COV(CV_R_IDX_DUP);
      { unsigned gg, d; MapPar90 *w = (MapPar90 *)pre; for (gg = 1; gg < 6; gg++) for (d = 0; d < gg; d++) if (w->cnt_[gg] == w->cnt_[d] && !memcmp(w->listA_ + 128 * gg, w->listA_ + 128 * d, w->cnt_[gg]) && memcmp(w->listB_ + 128 * gg, w->listB_ + 128 * d, w->cnt_[gg])) COV(CV_R_IDX_PARTIAL); }
      h_emit(0, "idx n %d [%u %u %u %u %u %u]", rn, out[0], out[1], out[2], out[3], out[4], out[5]);
      DUMP90("idx", mp);
    } else if (which == 2 || which == 3) {   /* getConstellationMask / getCodecConstellationMask */
      int gsel = h_rpct(15) ? 6 + (int)h_rmod(3) : (h_rpct(5) ? (int)(h_rnd() & 0x7fffffffu) : (int)h_rmod(6));
      unsigned short out[10], want[8];
      unsigned idx[6], grp[6];
      int eg = gsel < 6 ? gsel : 0;
      const MapPar90 *w = (const MapPar90 *)pre;
      unsigned gi;
      for (g = 0; g < 6; g++) mp->idx_[g] = h_rmod(6);   /* valid representatives (the real caller runs getConstellationsIndex first) */
      memcpy(pre, mp, sizeof pre);
      for (k = 0; k < 10; k++) out[k] = 0xbeef;
      (void)idx; (void)grp;
      gi = w->idx_[eg];
      if (which == 2) { get_mask(mp, gsel, out); model_mask(want, w->listA_ + 128 * gi, w->cnt_[gi]); COV(CV_R_MASK); }
      else { get_cmask(mp, gsel, out); model_mask(want, w->listB_ + 128 * gi, w->cnt_[gi]); COV(CV_R_CMASK); }
      for (k = 0; k < 8; k++) if (out[k] != want[k]) h_die3("get%sConstellationMask g %d: word %d is 0x%04x, model 0x%04x", which == 3 ? "Codec" : "", gsel, k, out[k], want[k]);
      if (out[8] != 0xbeef || out[9] != 0xbeef) h_die3("get mask wrote past 8 words");
      if (gsel >= 6) COV(CV_R_GHI);
      if (gsel >= 6) COV(CV_R_G6PLUS);
      if (w->cnt_[gi] == 0) COV(CV_R_CNT0);
      if (memcmp(pre, mp, sizeof pre)) h_die3("get mask changed the block");
      h_emit(0, "mask w%d g %d %04x %04x %04x %04x %04x %04x %04x %04x", which, gsel, out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7]);
    } else {   /* setConstellationMask / setCodecConstellationMask + round trip */
      int gsel = h_rpct(15) ? 6 + (int)h_rmod(3) : (int)h_rmod(6);
      int eg = gsel < 6 ? gsel : 0;
      short in[8];
      unsigned c, j, b, pa = 0;
      uint8_t exp[128];
      MapPar90 *w = (MapPar90 *)pre;
      for (j = 0; j < 8; j++) in[j] = (short)(h_rpct(25) ? 0 : (h_rpct(10) ? 0xffff : h_rnd()));
      for (j = 8; j-- > 0;) for (b = 16; b-- > 0;) if ((uint16_t)in[j] & (1u << (15 - b))) exp[pa++] = (uint8_t)(16 * j + b);
      if (which == 4) { set_mask(mp, gsel, in); COV(CV_R_SETMASK); memcpy(w->listA_ + 128 * eg, exp, pa); }
      else { set_cmask(mp, gsel, in); COV(CV_R_SETCMASK); memcpy(w->listB_ + 128 * eg, exp, pa); }
      w->cnt_[eg] = pa;
      if (memcmp(pre, mp, sizeof pre)) h_die3("set mask g %d: block differs from the model", gsel);
      if (gsel >= 6) { COV(CV_R_GHI); COV(CV_R_G6PLUS); }
      /* round trip: the get functions give the words back (the set order is descending positions; reversed bit order cancels) */
      mp->idx_[eg] = (uint32_t)eg;
      {
        unsigned short out[8];
        (which == 4 ? get_mask : get_cmask)(mp, eg, out);
        for (c = 0; c < 8; c++) { unsigned rv = 0, bb2; for (bb2 = 0; bb2 < 16; bb2++) if ((uint16_t)in[c] & (1u << bb2)) rv |= 1u << (15 - bb2); if (out[c] != rv) h_die3("mask round trip: word %u is 0x%04x, bit reversal of 0x%04x is 0x%04x (set unpacks bit i to element 16 j + 15 - i, get packs element e to bit e & 15)", c, out[c], (uint16_t)in[c], rv); }
        COV(CV_R_RT);
      }
      h_emit(0, "setmask w%d g %d cnt %u hash %016llx", which, gsel, pa, (unsigned long long)fnv_upd(FNV0, mp, sizeof *mp));
      DUMP90("setmask", mp);
    }
    canary_check(mp, sizeof(MapPar90), "mask / index / rate function");
    h_trial_end("masks");
  }
  (void)k;
}

/* ================================================================================================ scenario 2: V90CPPacker */
#define OUTN 1900
typedef struct { short exp[OUTN]; uint8_t spec[OUTN]; unsigned pos, P; } Fr;
static void fput(Fr *m, unsigned p, int v) { m->exp[p] = (short)v; m->spec[p] = 1; }
static void fraw(Fr *m, int v) {
  if (m->pos % 17 == 0) { fput(m, m->pos, 0); m->pos++; }
  fput(m, m->pos, v);
  m->pos++;
}
static void fval(Fr *m, int32_t v, int n) { int i; for (i = 0; i < n; i++) fraw(m, (v >> (i < 31 ? i : 31)) & 1); }

static void sc_pack(void) {
  int trial, k, g;
  cov_def(CV_K_CALL, "call"); cov_def(CV_K_FLAGB, "flag_ == 1 (codec arrays)"); cov_def(CV_K_NOFLAGB, "flag_ != 1"); cov_def(CV_K_N1, "one distinct list"); cov_def(CV_K_N2, "two distinct lists"); cov_def(CV_K_N6, "six distinct lists");
  cov_def(CV_K_CLEARDOWN, "cleardown != 0"); cov_def(CV_K_W4_0, "w4 == 0"); cov_def(CV_K_W4_1, "w4 != 0"); cov_def(CV_K_MSGS, "Q-format message inside the packer"); cov_def(CV_K_BIGFIELDS, "wide fields (> 1)");
  cov_def(CV_K_NOCLEAR, "cleardown == 0"); cov_def(CV_K_LVL0, "level < 2"); cov_def(CV_K_LVL2, "level >= 2"); cov_def(CV_K_S, "w10 != 0 (s)"); cov_def(CV_K_QUOTE, "w0 != 0 (')"); cov_def(CV_K_W14NEG, "w14 negative");
  cov_def(CV_K_CNT0, "an empty list"); cov_def(CV_K_FLAG2, "flag_ == 2 (not 1)");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 1500; trial++) {
    MapPar90 *mp;
    AddCp *ad;
    short *out;
    short want[OUTN], pre[OUTN];
    uint8_t spec[OUTN];
    Fr m;
    unsigned idx[6], grp[6], n, lvl = h_rpct(8) ? (h_rpct(50) ? 0x80000000u + h_rmod(3) : 0xffffffffu - h_rmod(2)) : (h_rpct(50) ? 2 : h_rmod(4)), P, nmsgs = 0;
    int cleardown = h_rpct(20) ? (h_rpct(50) ? 1 : (int)h_rnd()) : 0, ret, wantret;
    uint8_t crc[16];
    unsigned long long hbefore;
    h_pool_reset();
    mp = (MapPar90 *)onew_canary(sizeof(MapPar90));
    ad = (AddCp *)onew(sizeof(AddCp));
    { uint8_t *raw = (uint8_t *)onew(OUTN * sizeof(short) + 64); memset(raw, 0xc3, 32); memset(raw + 32 + OUTN * sizeof(short), 0xc3, 32); out = (short *)(raw + 32); }
    rnd_mp(mp, 1);
    mp->flag_ = h_rpct(60) ? 1 : (h_rpct(50) ? 0 : (h_rpct(50) ? 2 : h_rnd()));
    mp->w620_ = h_rpct(70) ? h_rmod(4) : h_rnd();
    mp->w624_ = h_rpct(70) ? h_rmod(4) : h_rnd();
    mp->w0_ = h_rpct(80) ? 8 + h_rmod(60) : h_rnd();
    if (h_rpct(40)) for (g = 0; g < 6; g++) mp->cnt_[g] = h_rmod(25);
    ad->w0_ = h_rpct(50) ? h_rmod(2) : h_rnd(); ad->w4_ = h_rpct(50) ? h_rmod(2) : h_rnd(); { uint32_t b = rfloat(1); memcpy(&ad->f8_, &b, 4); }
    ad->wc_ = h_rpct(50) ? h_rmod(2) : h_rnd(); ad->w10_ = h_rpct(50) ? h_rmod(2) : h_rnd(); ad->w14_ = h_rpct(30) ? (h_rpct(50) ? 0xffff8000u + h_rmod(0x8000) : h_rmod(0x8000)) : h_rnd();
    for (k = 0; k < OUTN; k++) out[k] = (short)(h_rpct(50) ? h_rnd() : (0x7100 + (k & 0xff)));
    memcpy(pre, out, sizeof pre);
    memset(&m, 0, sizeof m);
    /* the model: dedupe, then the sequential frame */
    n = model_dedupe(mp, idx, grp);
    { uint16_t *dummy = NULL; (void)dummy; }
    {
      uint8_t q[16];
      int i, rate = (int)(mp->w0_ - (ad->w4_ ? 0x14u : 8u));
      if (cleardown) rate = 0;
      for (i = 0; i <= 16; i++) fput(&m, (unsigned)i, 1);
      fput(&m, 0x11, 0);
      fput(&m, 0x12, 0);
      m.pos = 0x13;
      fraw(&m, (int16_t)ad->w4_);
      fval(&m, rate, 5);
      fval(&m, 0, 5);
      fraw(&m, (int16_t)ad->w10_);
      fval(&m, (int32_t)mp->w620_, 2);
      fraw(&m, (int16_t)ad->w0_);
      fraw(&m, (int16_t)ad->wc_);
      fval(&m, (int16_t)ad->w14_, 13);
      fval(&m, (int32_t)mp->w624_, 2);
      model_q(ad->f8_, W2, 16, 13, q); for (i = 0; i < 16; i++) fraw(&m, q[i]);
      model_q(mp->f628_, W1, 7, 6, q); for (i = 0; i < 7; i++) fraw(&m, q[i]); fraw(&m, !(mp->f628_ >= 0.0f));
      model_q(mp->f62c_, W1, 7, 6, q); for (i = 0; i < 7; i++) fraw(&m, q[i]); fraw(&m, !(mp->f62c_ >= 0.0f));
      model_q(mp->f630_, W1, 7, 6, q); for (i = 0; i < 7; i++) fraw(&m, q[i]); fraw(&m, !(mp->f630_ >= 0.0f));
      model_q(mp->f634_, W1, 7, 6, q); for (i = 0; i < 7; i++) fraw(&m, q[i]); fraw(&m, !(mp->f634_ >= 0.0f));
      for (g = 0; g < 4; g++) fval(&m, (int32_t)grp[g], 4);
      for (g = 4; g < 6; g++) fval(&m, (int32_t)grp[g], 4);
      fraw(&m, mp->flag_ == 1);
      for (i = 0; i < 7; i++) fraw(&m, 0);
      for (g = 0; g < (int)n; g++) { uint16_t mk[8]; unsigned e; model_mask(mk, mp->listA_ + 128 * idx[g], mp->cnt_[idx[g]]); for (e = 0; e < 8; e++) fval(&m, mk[e], 16); }
      if (mp->flag_ == 1) for (g = 0; g < (int)n; g++) { uint16_t mk[8]; unsigned e; model_mask(mk, mp->listB_ + 128 * idx[g], mp->cnt_[idx[g]]); for (e = 0; e < 8; e++) fval(&m, mk[e], 16); }
      P = m.pos;
      if (P % 17) h_die3("model: arrays end off the grid (%u)", P);
      fput(&m, P, 0);
      {
        short sb[OUTN];
        for (i = 0; i < (int)P; i++) sb[i] = m.exp[i];
        model_crc(sb, P, crc);
      }
      for (i = 0; i < 16; i++) fput(&m, P + 1 + (unsigned)i, crc[i]);
      for (i = 0; i < 3; i++) fput(&m, P + 0x11 + (unsigned)i, 0);
      wantret = (int)P + 20;
    }
    memcpy(want, pre, sizeof want);
    memcpy(spec, m.spec, sizeof spec);
    for (k = 0; k < OUTN; k++) if (spec[k]) want[k] = m.exp[k];
    dsplibs_debug_level = lvl;
    g_dbg_in_call = 0; g_ed_in_call = 0;
    hbefore = fnv_upd(FNV0, mp, sizeof *mp);
    (void)hbefore;
    ret = pk(mp, ad, out, cleardown);
    COV(CV_K_CALL);
    { unsigned c; for (c = 0; c < 32; c++) if (((uint8_t *)out)[(int)c - 32] != 0xc3 || ((uint8_t *)out)[OUTN * sizeof(short) + c] != 0xc3) h_die3("V90CPPacker wrote outside the output buffer (guard byte %u)", c); }
    EXPECT_EQ("V90CPPacker return", (uint32_t)ret, (uint32_t)wantret);
    for (k = 0; k < OUTN; k++) if (out[k] != want[k]) h_die3("V90CPPacker: out[%d] (0x%x) is %d, model %d (spec %d, P %u)", k, k, out[k], want[k], spec[k], P);
    /* the mapping block: only idx_ is written (getConstellationsIndex) */
    { MapPar90 w; memcpy(&w, mp, sizeof w); for (g = 0; g < (int)n; g++) EXPECT_EQ("idx_", mp->idx_[g], idx[g]); }
    /* debug output: the cleardown message (level > 1), the six format messages of float2Bits at their positions, the dump through edprintf */
    {
      unsigned expd = 0, i;
      char txt[300];
      if (cleardown && lvl > 1) { if (strcmp(g_dbg_text[expd], "V90CPPacker: CLEARDOWN indicated !\\r\\n")) h_die3("cleardown text '%s'", g_dbg_text[expd]); expd++; }
      if (lvl > 1) {
        float v8 = ad->f8_, a1 = mp->f628_, a2 = mp->f62c_, b1 = mp->f630_, b2 = mp->f634_;
        if (v8 > 8.0f || !(v8 >= 0.0f)) { if (strcmp(g_dbg_text[expd], "Q3.13 format violation!!\\r\\n")) h_die3("Q3.13 text '%s'", g_dbg_text[expd]); expd++; nmsgs++; }
        { float qv[4] = {a1, a2, b1, b2}; for (i = 0; i < 4; i++) if (qv[i] > 1.0f || !(qv[i] >= -1.0f)) { if (strcmp(g_dbg_text[expd], "Q1.6 format violation!!\\r\\n")) h_die3("Q1.6 text '%s'", g_dbg_text[expd]); expd++; nmsgs++; } }
      }
      if (g_dbg_in_call != expd) h_die3("V90CPPacker: %u dbgprintf calls, model %u (level %u cleardown %d)", g_dbg_in_call, expd, lvl, cleardown);
      if (nmsgs) COV(CV_K_MSGS);
      if (g_ed_in_call != 8) h_die3("V90CPPacker: %u edprintf calls, expected 8", g_ed_in_call);
      snprintf(txt, sizeof txt, "V90CP packed: CP%s%s%s bits:\n", ad->w4_ ? "" : "t", ad->w10_ ? "s" : "", ad->w0_ ? "'" : "");
      { char e2[300]; e2[0] = 0; esc_cat(e2, 300, txt); if (strcmp(g_ed_text[0], e2)) h_die3("header '%s' model '%s'", g_ed_text[0], e2); }
      for (i = 0; i < 7; i++) {
        char row[100], e2[300];
        unsigned c;
        row[0] = 0;
        for (c = 0; c < 17; c++) { char t1[8]; snprintf(t1, sizeof t1, "%1d", (int)out[0x11 * (i + 1) + c]); strcat(row, t1); }
        strcat(row, "\r\n");
        e2[0] = 0; esc_cat(e2, 300, row);
        if (strcmp(g_ed_text[1 + i], e2)) h_die3("row %u '%s' model '%s'", i, g_ed_text[1 + i], e2);
      }
    }
    if (mp->flag_ == 1) COV(CV_K_FLAGB); else COV(CV_K_NOFLAGB);
    if (mp->flag_ == 2) COV(CV_K_FLAG2);
    if (n == 1) COV(CV_K_N1); else if (n == 2) COV(CV_K_N2); else if (n == 6) COV(CV_K_N6);
    if (cleardown) COV(CV_K_CLEARDOWN); else COV(CV_K_NOCLEAR);
    if (ad->w4_) COV(CV_K_W4_1); else COV(CV_K_W4_0);
    if (ad->w10_) COV(CV_K_S);
    if (ad->w0_) COV(CV_K_QUOTE);
    if ((int16_t)ad->w14_ < 0) COV(CV_K_W14NEG);
    if (ad->w4_ > 1 || ad->w0_ > 1 || ad->wc_ > 1 || ad->w10_ > 1) COV(CV_K_BIGFIELDS);
    if (lvl > 1) COV(CV_K_LVL2); else COV(CV_K_LVL0);
    for (g = 0; g < 6; g++) if (mp->cnt_[g] == 0) { COV(CV_K_CNT0); break; }
    h_emit(0, "pack n %u P %u ret %d lvl %u cd %d hash %016llx", n, P, ret, lvl, cleardown, (unsigned long long)fnv_upd(FNV0, out, OUTN * sizeof(short)));
    canary_check(mp, sizeof(MapPar90), "V90CPPacker");
    h_trial_end("pack");
  }
}

/* ================================================================================================ scenario 3: displaySpectralParams */
static void sc_disp(void) {
  int trial;
  cov_def(CV_D_CALL, "call"); cov_def(CV_D_NAN, "NaN"); cov_def(CV_D_BIG, "|v| >= 2^31 (integer indefinite)"); cov_def(CV_D_NEG, "negative"); cov_def(CV_D_ZERO, "zero (sign '-')"); cov_def(CV_D_FRAC, "fraction digits"); cov_def(CV_D_PLUS, "positive (sign '+')");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 6000; trial++) {
    MapPar90 *mp;
    uint32_t fb[4];
    float f[4];
    int k;
    h_pool_reset();
    mp = (MapPar90 *)onew_canary(sizeof(MapPar90));
    garbage_fill(mp, sizeof *mp, h_rnd());
    mp->w620_ = h_rpct(60) ? h_rmod(8) : h_rnd();
    mp->w624_ = h_rpct(60) ? h_rmod(8) : h_rnd();
    for (k = 0; k < 4; k++) fb[k] = rfloat(h_rpct(50));
    memcpy(&mp->f628_, &fb[0], 4); memcpy(&mp->f62c_, &fb[1], 4); memcpy(&mp->f630_, &fb[2], 4); memcpy(&mp->f634_, &fb[3], 4);
    for (k = 0; k < 4; k++) f[k] = bf(fb[k]);
    g_ed_in_call = 0;
    {
      uint8_t pre[sizeof(MapPar90)];
      memcpy(pre, mp, sizeof pre);
      disp(mp);
      if (memcmp(pre, mp, sizeof pre)) h_die3("displaySpectralParams changed the block");
    }
    COV(CV_D_CALL);
    if (g_ed_in_call != 6) h_die3("displaySpectralParams: %u edprintf calls", g_ed_in_call);
    {
      char b[300], e2[300];
      static const char *nm[4] = {"A1 ", "A2 ", "B1 ", "B2 "};
      snprintf(b, sizeof b, "V90MappingParams: Sr = %d\r\n", (int)mp->w620_); e2[0] = 0; esc_cat(e2, 300, b); if (strcmp(g_ed_text[0], e2)) h_die3("Sr line '%s' model '%s'", g_ed_text[0], e2);
      snprintf(b, sizeof b, "V90MappingParams: Id = %d\r\n", (int)mp->w624_); e2[0] = 0; esc_cat(e2, 300, b); if (strcmp(g_ed_text[1], e2)) h_die3("Id line '%s' model '%s'", g_ed_text[1], e2);
      for (k = 0; k < 4; k++) {
        int sg, ip, fr;
        split_model((double)f[k], &sg, &ip, &fr);
        snprintf(b, sizeof b, "V90MappingParams: %s = %c%d.%06d\r\n", nm[k], sg, ip, fr);
        e2[0] = 0; esc_cat(e2, 300, b);
        if (strcmp(g_ed_text[2 + k], e2)) h_die3("%s line '%s' model '%s' (value %08x)", nm[k], g_ed_text[2 + k], e2, fb[k]);
        if (f[k] != f[k]) COV(CV_D_NAN); else if (f[k] >= 2147483648.0f || f[k] <= -2147483648.0f) COV(CV_D_BIG); else if (f[k] < 0) COV(CV_D_NEG); else if (f[k] == 0) COV(CV_D_ZERO); else COV(CV_D_PLUS);
        if (f[k] == f[k] && f[k] > -1e9f && f[k] < 1e9f && (f[k] - (float)(long long)f[k]) != 0.0f) COV(CV_D_FRAC);
      }
    }
    h_emit(0, "disp %08x %08x %08x %08x", fb[0], fb[1], fb[2], fb[3]);
    canary_check(mp, sizeof(MapPar90), "displaySpectralParams");
    h_trial_end("disp");
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P12_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P12_DBG")) g_show = 1;
  p11_layout_check();
  h_x87_check();
  h_arena_init();
  {
    int i, n = 0;
    for (i = 0; i < NF_MapPar90; i++) {
      HFld e = f_MapPar90[i];
      if (e.kind == 'b' && e.count > 64) e.kind = 'x';
      g_f90[n++] = e;
    }
    g_nf90 = n;
  }
  switch (sc) {
    case 0: sc_f2b(); break;
    case 1: sc_masks(); break;
    case 2: sc_pack(); break;
    case 3: sc_disp(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu edprintf %u dbgprintf %u", h_nmalloc, h_nfree, g_nedp, g_ndbg);
  h_cov_finish();
  return 0;
}
