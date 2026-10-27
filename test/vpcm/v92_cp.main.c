/*
 * Differential test harness for port chunk P11 (src/vpcm/v92_cp.cpp): class V92CP of dsplibs.o - the V.92 CP / SUV frame holder / bit-serial codec (14 functions: C1 C2 D1 D2,
 * resetCRC calcCRC evaluateCRC resetDetector reset setSUV getBitVector evaluateInfo infoToBits bitsToInfo) - and the four free functions setV92CPpckFromParamsInfo,
 * setParamsInfoFromV92CPUnPck, setParamsInfoFromCPUnPck, V92setParamsInfoFromCPUnPck.
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/v92_cp.sh against the blob (dsplibs.o: `dsplibs_debug_printf` and the allocator are the harness'),
 * against the port (-O0 / -O2) and for x86-64 (diffed against the 32-bit BLOB's output = the golden, test/golden/vpcm/v92_cp.blob32.out).  The functions are reached through __asm__ labels;
 * the object layouts are C mirror structs GENERATED from test/vpcm/v92_cp.layout (V92CP and V90MappingParams have no pointers, V92MappingParams has ten borrowed pointer members; the
 * V90CP read by V92setParamsInfoFromCPUnPck is the P10 mirror, test/vpcm/v90_cp_layout.h, built by hand here: harness-owned heap blocks).
 *
 * What is compared: every return value, the object after every call (an expected IMAGE computed by the independent models, byte-compared, plus the field-wise dump sampled),
 * the malloc sequence, every `dsplibs_debug_printf` call (text, with a hook that lowers dsplibs_debug_level between prints, and an object snapshot at each print).  INDEPENDENT
 * MODELS written from the frame definition (not from the port): the encoder (a sequential bit writer with the 17-bit start-bit grid inserted automatically, the float fields in
 * integer units of 2^-13 / 2^-6), a standard bit-serial CRC-CCITT, the decoder (units arithmetic), the receive machine's observable behaviour (frame end index, return codes, the
 * 400-call suppression window, the "Ed" zeros rule), the list <-> mask packing, the dedupe of the six lists and the V92 mapping parameter conversion with its prints.
 *
 * Scenarios (each its own process): 0 infoToBits vs the encoder model, 1 evaluateInfo case by case vs the decoder model (+ the chain 5, 6, 7, 8 on one object),
 * 2 bitsToInfo end to end (encode with the blob / port, receive bit by bit, decoded info vs the model, return codes, suppression, Ed, damaged frames, noise, level steering),
 * 3 bitsToInfo on random object states, 4 the pack / unpack functions vs the list models (+ round trips), 5 V92setParamsInfoFromCPUnPck vs the conversion model + prints,
 * 6 constructors / resets / setSUV / getBitVector / CRC functions.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "v92_cp_layout.h"
#include "v90_cp_layout.h"

#define L(m) __asm__(m)

extern void cp_c1(void *) L("_ZN5V92CPC1Ev");
extern void cp_c2(void *) L("_ZN5V92CPC2Ev");
extern void cp_d1(void *) L("_ZN5V92CPD1Ev");
extern void cp_d2(void *) L("_ZN5V92CPD2Ev");
extern void cp_resetcrc(void *) L("_ZN5V92CP8resetCRCEv");
extern void cp_calccrc(void *) L("_ZN5V92CP7calcCRCEv");
extern int cp_evalcrc(void *) L("_ZN5V92CP11evaluateCRCEv");
extern void cp_resetdet(void *) L("_ZN5V92CP13resetDetectorEv");
extern void cp_reset(void *) L("_ZN5V92CP5resetEv");
extern void cp_setsuv(void *, unsigned) L("_ZN5V92CP6setSUVEj");
extern uint8_t *cp_getbv(void *, unsigned *) L("_ZN5V92CP12getBitVectorERj");
extern void cp_evalinfo(void *) L("_ZN5V92CP12evaluateInfoEv");
extern void cp_i2b(void *) L("_ZN5V92CP10infoToBitsEv");
extern int cp_b2i(void *, unsigned char) L("_ZN5V92CP10bitsToInfoEh");
extern void f_pck(MapPar90 *, AddCp *, V92CpObj *) L("setV92CPpckFromParamsInfo");
extern void f_from92(MapPar90 *, V92CpObj *) L("setParamsInfoFromV92CPUnPck");
extern void f_fromun(MapPar90 *, CpUnpck *) L("setParamsInfoFromCPUnPck");
extern void f_v92set(MapPar92 *, CpObj *) L("V92setParamsInfoFromCPUnPck");
extern unsigned int dsplibs_debug_level;
extern uint32_t binaryTable[16];   /* the blob's GLOBAL data tables, byte-compared through the output (the port defines its own copies) */
extern float fltTable_1[7];
extern float fltTable_2[16];

#define NB 2000u
#define T92 sizeof(V92CpObj)

/* ------------------------------------------------------------------------------------------------ recorder (dsplibs_debug_printf) */
static unsigned g_ndbg;
#define DBGMAX 1100
static char g_dbg_text[DBGMAX][300];
static unsigned g_dbg_in_call;
static int g_dbg_show;
typedef void (*dbg_hook_t)(unsigned idx);
static dbg_hook_t g_dbg_hook;
static const void *g_snap_obj;
static const HFld *g_snap_f;
static int g_snap_nf;
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else if (*s == '\t') { out[n++] = '\\'; out[n++] = 't'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (g_dbg_in_call < DBGMAX) { g_dbg_text[g_dbg_in_call][0] = 0; esc_cat(g_dbg_text[g_dbg_in_call], 300, buf); }
  h_emit(g_dbg_show, "dbgprintf %s", g_dbg_in_call < DBGMAX ? g_dbg_text[g_dbg_in_call] : "(many)");
  if (g_snap_obj) h_dump_obj(g_dbg_show, "snap", g_snap_obj, g_snap_f, g_snap_nf);
  g_dbg_in_call++;
  g_ndbg++;
  if (g_dbg_hook) g_dbg_hook(g_dbg_in_call - 1);
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

/* dump tables: big arrays become hashed 'x' regions */
static HFld g_f92[48], g_f90[24], g_fm92[32];
static int g_nf92, g_nf90, g_nfm92;
static int build_tab(HFld *out, const HFld *in, int nin) {
  int i, n = 0;
  for (i = 0; i < nin; i++) {
    HFld e = in[i];
    if ((e.kind == 'b' && e.count > 64) || (e.kind == 'h' && e.count > 32)) { e.kind = 'x'; e.count = e.count * (e.kind == 'h' ? 2u : 1u); if (in[i].kind == 'h') e.count = in[i].count * 2u; }
    out[n++] = e;
  }
  return n;
}
static unsigned long g_dumpctr;
#define DUMP92(tag, o) h_dump_obj(h_sample(&g_dumpctr, 12, 97), (tag), (o), g_f92, g_nf92)
#define DUMP90(tag, o) h_dump_obj(h_sample(&g_dumpctr, 12, 97), (tag), (o), g_f90, g_nf90)
#define DUMPM92(tag, o) h_dump_obj(h_sample(&g_dumpctr, 12, 53), (tag), (o), g_fm92, g_nfm92)

static V92CpObj *CO(void *o) { return (V92CpObj *)o; }
static uint64_t hobj(const void *o, size_t n) { return fnv_upd(FNV0, o, n); }

/* ------------------------------------------------------------------------------------------------ the frame info as a value */
typedef struct {
  uint8_t type, mode, rate, b03, flag, flagB, mult;
  uint32_t w08, w0c, f[5];
  uint32_t grp[6];
  uint16_t nGrp, arrA[48], arrB[48];
  uint32_t suv, suvVal;
} I92;

static void set_i92(V92CpObj *o, const I92 *in) {
  int k;
  o->type_ = in->type; o->mode_ = in->mode; o->rate_ = in->rate; o->b03_ = in->b03; o->flag_ = in->flag; o->flagB_ = in->flagB; o->mult_ = in->mult;
  o->w08_ = in->w08; o->w0c_ = in->w0c;
  memcpy(&o->f10_, &in->f[0], 4); memcpy(&o->f14_, &in->f[1], 4); memcpy(&o->f18_, &in->f[2], 4); memcpy(&o->f1c_, &in->f[3], 4); memcpy(&o->f20_, &in->f[4], 4);
  for (k = 0; k < 6; k++) o->grp_[k] = in->grp[k];
  o->nGrp_ = in->nGrp;
  memcpy(o->arrA_, in->arrA, sizeof o->arrA_); memcpy(o->arrB_, in->arrB, sizeof o->arrB_);
  o->suv_ = in->suv; o->suvVal_ = in->suvVal;
}

static uint32_t r16(void) { return h_rnd() & 0xffffu; }

/* float bit patterns: exact unit sums (what the encoder can represent), random, specials */
static uint32_t units_to_bits(int neg, unsigned units, int shift) { /* units * 2^-shift as a float bit pattern */
  float f = (float)units;
  int k;
  for (k = 0; k < shift; k++) f *= 0.5f;
  { uint32_t b; memcpy(&b, &f, 4); if (neg) b ^= 0x80000000u; return b; }
}
static uint32_t rfloat(int wide) {
  uint32_t b;
  switch (h_rmod(10)) {
    case 0: return 0;
    case 1: return 0x80000000u;
    case 2: { static const uint32_t sp[] = {0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x7f7fffffu, 0xff7fffffu, 0x00000001u, 0x80000001u, 0x3f800000u, 0xbf800000u, 0x41000000u, 0x40ffffffu, 0x3f7fffffu,
                                    0x00800000u, 0x7fa00000u, 0xffa00000u, 0x7f7ffff0u, 0x3f800001u};
      return sp[h_rmod(sizeof sp / sizeof sp[0])]; }
    case 3: case 4: return units_to_bits(h_rpct(40), h_rmod(wide ? 65536u : 128u), wide ? 13 : 6);
    case 5: return units_to_bits(h_rpct(50), h_rmod(wide ? 1u << 20 : 1u << 12), wide ? 13 : 6);
    case 6: { b = h_rnd(); return b; }
    case 7: return units_to_bits(h_rpct(50), h_rmod(2000), 10);
    default: b = (h_rnd() & 0x80000000u) | (0x3c000000u + (h_rnd() & 0x03ffffffu)); return b;   /* magnitudes around 2^-7 .. 2^3 */
  }
}

/* ------------------------------------------------------------------------------------------------ models */
/* encoder model: a sequential bit writer; every multiple of 17 is a start bit (0) inserted automatically */
typedef struct { uint8_t exp[2200]; uint8_t spec[2200]; unsigned pos, C, total, frameLen; } M92;
static void mput(M92 *m, unsigned p, unsigned v) { m->exp[p] = (uint8_t)v; m->spec[p] = 1; }
static void w_raw(M92 *m, unsigned v) {
  if (m->pos % 17 == 0) { mput(m, m->pos, 0); m->pos++; }
  mput(m, m->pos, v);
  m->pos++;
}
static void w_val(M92 *m, uint32_t v, int n) { int i; for (i = 0; i < n; i++) w_raw(m, (v >> i) & 1u); }
static void w_sval(M92 *m, int32_t v, int n) { int i; for (i = 0; i < n; i++) w_raw(m, (unsigned)(v >> (i < 31 ? i : 31)) & 1u); }   /* arithmetic shifts */

static void model_crc(const uint8_t *b, unsigned endp, uint8_t out[16]) {
  uint16_t r = 0xffff;
  unsigned p, k;
  /* the positions fed: every non-start-bit position in [0x12, endp); the blob's loop also consumes position endp itself when endp - 1 is a start bit (it skips the start bit and
     then processes the NEXT bit before it tests the bound again): only reachable with a bound that is not a frame end */
  unsigned last = endp;
  if (endp > 0x12 && (endp - 1) % 17 == 0) last = endp + 1;
  for (p = 0x12; p < last; p++) {
    unsigned fb;
    if (p % 17 == 0) continue;
    fb = ((r >> 15) ^ (b[p] & 1u)) & 1u;
    r = (uint16_t)(r << 1);
    if (fb) r ^= 0x1021;
  }
  for (k = 0; k < 16; k++) out[k] = (uint8_t)((r >> (15 - k)) & 1);
}

/* the float field -> bits, in integer units: v = |f| * 2^shift against the unit weights (greedy: a weight larger than the rest gives 0, else 1 and subtract) */
static void float_bits(M92 *m, uint32_t fb, const unsigned *w, int n, int shift, int sign_kind /* 0 none, 1 "f < 0", 2 "!(f >= 0)" */) {
  float f;
  double v, scale = 1.0;
  uint8_t bit[16];
  int i;
  memcpy(&f, &fb, 4);
  for (i = 0; i < shift; i++) scale *= 2.0;
  v = (f < 0 ? -(double)f : (double)f);
  if (f != f) v = (double)f;   /* NaN stays NaN */
  v = v * scale;
  for (i = 0; i < n; i++) {
    if ((double)w[i] > v) bit[i] = 0;
    else { bit[i] = 1; v -= (double)w[i]; }
  }
  for (i = n - 1; i >= 0; i--) w_raw(m, bit[i]);   /* the entry with the smallest weight goes to the lowest bit position */
  if (sign_kind == 1) w_raw(m, f < 0.0f);
  else if (sign_kind == 2) w_raw(m, !(f >= 0.0f));
}

static void model_encode(const I92 *in, const uint8_t *pre, M92 *m) {
  static const unsigned W2[16] = {32768, 16384, 8192, 4096, 2048, 1024, 512, 256, 128, 64, 16, 16, 8, 4, 2, 1};
  static const unsigned W1[7] = {64, 32, 16, 8, 4, 2, 1};
  unsigned p, g, e;
  uint8_t crc[16];
  unsigned C, D, v, q;
  memset(m, 0, sizeof *m);
  (void)pre;
  for (p = 0; p <= 16; p++) mput(m, p, 1);
  mput(m, 0x11, 0);
  mput(m, 0x12, in->type);
  m->pos = 0x13;
  if (in->type == 1) {
    w_val(m, 0, 8);
    w_sval(m, (int16_t)in->suv, 5);
    w_raw(m, in->suvVal & 0xffu);
    w_raw(m, in->flag);
    C = m->pos;   /* 0x22 */
  } else {
    int shortpath = ((int8_t)in->mode <= 1);
    w_raw(m, in->mode & 1u);
    w_raw(m, (in->mode >> 1) & 1u);
    w_val(m, (uint32_t)(int8_t)in->rate, 5);
    w_val(m, 0, 5);
    if (shortpath) { w_val(m, in->w08, 2); } else { w_val(m, 0, 2); }
    w_raw(m, in->flag);
    if (shortpath) {
      w_raw(m, in->b03);
      w_sval(m, (int32_t)(int8_t)in->rate >> 5, 13);
      w_val(m, in->w0c, 2);
      float_bits(m, in->f[0], W2, 16, 13, 0);
      float_bits(m, in->f[1], W1, 7, 6, 1);
      float_bits(m, in->f[2], W1, 7, 6, 2);
      float_bits(m, in->f[3], W1, 7, 6, 2);
      float_bits(m, in->f[4], W1, 7, 6, 2);
      for (g = 0; g < 4; g++) w_sval(m, (int16_t)in->grp[g], 4);
      for (g = 4; g < 6; g++) w_sval(m, (int16_t)in->grp[g], 4);
      w_raw(m, in->flagB);
      w_val(m, 0, 7);
      for (g = 0; g < in->nGrp; g++) for (e = 0; e < 8; e++) w_sval(m, (int16_t)in->arrA[8 * g + e], 16);
      if (in->flagB && in->nGrp) for (g = 0; g < in->nGrp; g++) for (e = 0; e < 8; e++) w_sval(m, (int16_t)in->arrB[8 * g + e], 16);
    }
    C = m->pos;
  }
  if (C % 17 != 0) h_die3("model: payload ends off the grid (%u)", C);
  m->C = C;
  mput(m, C, 0);
  model_crc(m->exp, C, crc);
  for (p = 0; p < 16; p++) mput(m, C + 1 + p, crc[p]);
  mput(m, C + 17, 0);
  m->frameLen = C + 17;
  D = 12u * (in->type == 1 ? in->mult : (in->mode == 0 ? 1u : in->mult));
  v = C + 18;
  q = v / D;
  m->total = (q + 1) * D;
  for (p = C + 18; p < m->total; p++) mput(m, p, 0);
}

static uint32_t rdbits(const uint8_t *b, unsigned a, int n) { uint32_t v = 0; int i; for (i = 0; i < n; i++) v += (uint32_t)b[a + (unsigned)i] * (1u << i); return v; }

/* the decoder model on an object image: applies what evaluateInfo does in state `st` to `img` (a V92CP image); returns 1 if the state has a case */
static unsigned bits_to_units(const uint8_t *b, unsigned *cur, int n) {   /* the first bit read has the LARGEST weight of n bits: weights are the encoder's, table index n-1-i */
  unsigned u = 0; int i;
  for (i = 0; i < n; i++) { u += (uint32_t)b[*cur] * (1u << (n - 1 - i)); (*cur)++; }
  return u;
}
/* weights of the 16 entry table in 2^-13 units, of the 7 entry table in 2^-6 units (the encoder side's duplicate 2^-9 included) */
static void model_eval(uint8_t *img, unsigned st) {
  V92CpObj *o = (V92CpObj *)img;
  const uint8_t *b = o->bits_;
  static const unsigned W2[16] = {32768, 16384, 8192, 4096, 2048, 1024, 512, 256, 128, 64, 16, 16, 8, 4, 2, 1};
  unsigned g, k, cur;
  switch (st) {
    case 3: {
      uint32_t s = 0;
      for (k = 0; k < 5; k++) s += (uint32_t)b[0x1b + k] * (1u << k);
      o->suv_ = s; o->suvVal_ = b[0x20]; o->flag_ = b[0x21];
      break;
    }
    case 5: {
      uint8_t mode = (uint8_t)((b[0x13] & 1u) | (uint8_t)(2u * b[0x14]));
      uint32_t r = 0;
      for (k = 0; k < 5; k++) r += (uint32_t)(b[0x15 + k] & 1u) << k;
      o->mode_ = mode; o->modeCopy_ = mode; o->rate_ = (uint8_t)r;
      if ((int8_t)mode <= 1) o->w08_ = 2u * b[0x20] + (b[0x1f] & 1u);
      o->flag_ = b[0x21];
      break;
    }
    case 6: {
      uint32_t mx = 0, grp[6];
      unsigned u;
      uint32_t fb[5];
      float f;
      o->w0c_ = 2u * b[0x32] + (b[0x31] & 1u);
      o->b03_ = b[0x23];
      cur = 0x34;
      /* the entries are consumed in table order 15 .. 0: the first bit read is the entry with the smallest weight, so read LSB first with the weights of table index 15 - i */
      u = 0;
      for (k = 0; k < 16; k++) { if (b[cur]) u += W2[15 - k]; cur++; }
      f = (float)u / 8192.0f; memcpy(&fb[0], &f, 4);
      cur++;
      for (g = 1; g < 5; g++) {
        unsigned w1 = 0;
        for (k = 0; k < 7; k++) { if (b[cur]) w1 += 1u << k; cur++; }
        f = (float)w1 / 64.0f;
        memcpy(&fb[g], &f, 4);
        if (b[cur]) fb[g] ^= 0x80000000u;
        cur++;
        if (g == 2) cur++;
      }
      memcpy(&o->f10_, &fb[0], 4); memcpy(&o->f14_, &fb[1], 4); memcpy(&o->f18_, &fb[2], 4); memcpy(&o->f1c_, &fb[3], 4); memcpy(&o->f20_, &fb[4], 4);
      cur++;   /* after the f20 sign: the start bit 0x66 */
      for (g = 0; g < 6; g++) {
        grp[g] = rdbits(b, cur, 4);
        cur += 4;
        if (g == 3) cur++;
        if ((int32_t)grp[g] > (int32_t)mx) mx = grp[g];
      }
      for (g = 0; g < 6; g++) o->grp_[g] = grp[g];
      o->flagB_ = b[cur];
      o->nGrp_ = (uint16_t)(mx + 1);
      o->cursor_ = cur + 8;
      break;
    }
    case 7:
    case 8: {
      uint16_t *arr = st == 7 ? o->arrA_ : o->arrB_;
      cur = o->cursor_;
      for (g = 0; g < o->nGrp_; g++) for (k = 0; k < 8; k++) {
        cur++;
        arr[8 * g + k] = (uint16_t)bits_to_units(b, &cur, 16);
      }
      o->cursor_ = cur;
      break;
    }
    default: break;
  }
}

/* image comparison: dies naming the first differing byte and the member */
static const char *member_of(size_t off) {
  static const struct { size_t o; const char *n; } T[] = {{0, "type_"}, {1, "mode_"}, {2, "rate_"}, {3, "b03_"}, {4, "flag_"}, {5, "unk05_"}, {8, "w08_"}, {0xc, "w0c_"}, {0x10, "f10_"}, {0x14, "f14_"}, {0x18, "f18_"}, {0x1c, "f1c_"},
    {0x20, "f20_"}, {0x24, "flagB_"}, {0x25, "unk25_"}, {0x28, "grp_"}, {0x40, "unk40_"}, {0x42, "arrA_"}, {0xa2, "arrB_"}, {0x102, "unk102_"}, {0x104, "suv_"}, {0x108, "suvVal_"}, {0x10c, "nGrp_"}, {0x10e, "unk10e_"},
    {0x110, "cptSent_"}, {0x114, "state_"}, {0x118, "modeCopy_"}, {0x119, "ones_"}, {0x11a, "zeros_"}, {0x11b, "unk11b_"}, {0x11c, "widx_"}, {0x120, "count_"}, {0x124, "cursor_"}, {0x128, "mult_"}, {0x129, "bits_"},
    {0x8f9, "crc_"}, {0x909, "unk909_"}, {0x90c, "totalLen_"}, {0x910, "frameLen_"}, {0x914, "supp_"}};
  size_t i; const char *r = "?";
  for (i = 0; i < sizeof T / sizeof T[0]; i++) if (T[i].o <= off) r = T[i].n;
  return r;
}
static void image_check(const char *what, const void *got, const void *want, size_t n) {
  const uint8_t *a = (const uint8_t *)got, *b = (const uint8_t *)want;
  size_t i;
  for (i = 0; i < n; i++)
    if (a[i] != b[i]) h_die3("%s: byte +0x%zx (%s) got 0x%02x, model 0x%02x", what, i, member_of(i), a[i], b[i]);
}

/* ------------------------------------------------------------------------------------------------ object lifecycle */
static uint8_t gf_byte(uint32_t seed, uint32_t idx) { uint32_t x = (seed ^ (idx * 0x9e3779b1u)) * 2654435761u; x ^= x >> 15; x *= 2246822519u; return (uint8_t)(x >> 24); }
static void garbage_fill(void *o, size_t n, uint32_t seed) { size_t k; uint8_t *p = (uint8_t *)o; for (k = 0; k < n; k++) p[k] = gf_byte(seed, (uint32_t)k); }
/* a V92MappingParams: the pointer members move on x86-64, so every run of plain members is filled from its own LOGICAL offset */
static void garbage_fill_m92(MapPar92 *d, uint32_t seed) {
  uint8_t *p = (uint8_t *)d;
  size_t k, a0 = offsetof(MapPar92, arr_), a1 = a0 + sizeof d->arr_, l0 = offsetof(MapPar92, lcp_), l1 = l0 + sizeof d->lcp_;
  for (k = 0; k < a0; k++) p[k] = gf_byte(seed, (uint32_t)k);
  for (k = a1; k < l0; k++) p[k] = gf_byte(seed, 0x1000u + (uint32_t)(k - a1));
  for (k = l1; k < sizeof *d; k++) p[k] = gf_byte(seed, 0x2000u + (uint32_t)(k - l1));
  for (k = a0; k < a1; k++) p[k] = 0xa5;
  for (k = l0; k < l1; k++) p[k] = 0xa5;
}
static void *cp_new(int c2, int dirty) {
  void *o;
  h_pool_reset();
  o = onew(T92);
  if (dirty) garbage_fill(o, T92, h_rnd());
  (c2 ? cp_c2 : cp_c1)(o);
  return o;
}
static void cp_free(void *o, int d2) {
  (d2 ? cp_d2 : cp_d1)(o);
  h_trial_end("cp");
}

enum { CV_E_SUV = 0, CV_E_LONG, CV_E_MODE0, CV_E_MODE1, CV_E_MODE2, CV_E_MODENEG, CV_E_NGRP0, CV_E_NGRPN, CV_E_FLAGB, CV_E_FLAG, CV_E_MULT, CV_E_FNAN, CV_E_FINF, CV_E_FNEG, CV_E_FZERO, CV_E_FBIG, CV_E_PAD_EXACT,
       CV_E_PAD_ROUND, CV_E_RATENEG, CV_E_DIRTY, CV_E_B03BIG, CV_E_SUVBIG };
enum { CV_V_S3 = 0, CV_V_S5, CV_V_S6, CV_V_S7, CV_V_S8, CV_V_OTHER, CV_V_BIGBYTE, CV_V_CHAIN, CV_V_NGRP0, CV_V_SIGN, CV_V_MODE_HI, CV_V_S4, CV_V_GRPBIG };
enum { CV_R_LONG = 0, CV_R_SUV, CV_R_CP, CV_R_CPT, CV_R_BADCRC, CV_R_ED, CV_R_NOISE, CV_R_SUPP, CV_R_SUPP_END, CV_R_LVL0, CV_R_LVL2, CV_R_BACK2BACK, CV_R_FLAGB, CV_R_MODE_HI, CV_R_MULT, CV_R_EXACT_PAD, CV_R_ONE_AT_TYPE,
       CV_R_RET1, CV_R_RET2, CV_R_RET3, CV_R_RET4, CV_R_HOOK, CV_R_RAW };
enum { CV_F_S0 = 0, CV_F_S1, CV_F_S2, CV_F_S3, CV_F_S4, CV_F_S5, CV_F_S6, CV_F_S7, CV_F_S8, CV_F_S9, CV_F_S10, CV_F_SBIG, CV_F_BIT0, CV_F_BIT1, CV_F_ED, CV_F_RET1, CV_F_RET2, CV_F_RET3, CV_F_RET4, CV_F_RET5,
       CV_F_SUPP_NEG, CV_F_SUPP_POS, CV_F_SUPP_END, CV_F_CRC_OK, CV_F_CRC_BAD, CV_F_S5_HI, CV_F_S7_B, CV_F_S7_NOB, CV_F_S10_END, CV_F_S10_ONE, CV_F_ZEROS_WRAP, CV_F_GAMMA, CV_F_DELTA };
enum { CV_P_PCK = 0, CV_P_DEDUP, CV_P_NODEDUP, CV_P_PARTIAL, CV_P_B, CV_P_NOB, CV_P_MODE0, CV_P_MODE1, CV_P_FROM92, CV_P_FROM92_B, CV_P_FROM92_NOB, CV_P_FROMUN, CV_P_FROMUN_B, CV_P_FROMUN_NOB, CV_P_SUB_NZ, CV_P_SUB_Z,
       CV_P_RT, CV_P_CNT0, CV_P_NGRP0, CV_P_BIGPOS, CV_P_SNAN };
enum { CV_G_CALL = 0, CV_G_LVL0, CV_G_LVL2, CV_G_HOOK, CV_G_A, CV_G_B, CV_G_C, CV_G_ABC, CV_G_CLAMP4, CV_G_CLAMP6, CV_G_LCZERO, CV_G_LCSOME, CV_G_ENDLINE, CV_G_BIGGAIN, CV_G_NEGSIGN, CV_G_DRN };
enum { CV_C_C1 = 0, CV_C_C2, CV_C_D1, CV_C_D2, CV_C_RESET, CV_C_RESETDET, CV_C_SETSUV, CV_C_GETBV, CV_C_RESETCRC, CV_C_CALCCRC, CV_C_EVAL_OK, CV_C_EVAL_BAD, CV_C_SUM256 };

/* ================================================================================================ scenario 0: infoToBits vs the encoder model */
static void rnd_i92(I92 *in, unsigned max_total) {
  int k;
  for (;;) {
    M92 m;
    memset(in, 0, sizeof *in);
    in->type = h_rpct(25) ? 1 : (h_rpct(50) ? 0 : (uint8_t)(h_rnd() & 0xff));
    if (in->type == 1 && h_rpct(5)) in->type = 1;
    in->mode = (uint8_t)(h_rmod(4) < 3 ? h_rmod(4) : (h_rpct(50) ? 0x80 + h_rmod(128) : h_rnd()));
    in->rate = (uint8_t)(h_rpct(60) ? h_rmod(32) : h_rnd());
    in->b03 = (uint8_t)(h_rpct(60) ? h_rmod(2) : h_rnd());
    in->flag = (uint8_t)(h_rpct(60) ? h_rmod(2) : h_rnd());
    in->flagB = (uint8_t)(h_rpct(60) ? h_rmod(2) : h_rnd());
    in->mult = (uint8_t)(h_rpct(30) ? 1 : 1 + h_rmod(h_rpct(10) ? 20 : 5));
    in->w08 = h_rpct(70) ? h_rmod(4) : h_rnd();
    in->w0c = h_rpct(70) ? h_rmod(4) : h_rnd();
    in->f[0] = rfloat(1);
    for (k = 1; k < 5; k++) in->f[k] = rfloat(0);
    for (k = 0; k < 6; k++) in->grp[k] = h_rpct(70) ? h_rmod(6) : (h_rpct(50) ? h_rmod(16) : h_rnd());
    in->nGrp = (uint16_t)(h_rpct(15) ? 0 : 1 + h_rmod(6));
    for (k = 0; k < 48; k++) { in->arrA[k] = (uint16_t)r16(); in->arrB[k] = (uint16_t)r16(); }
    in->suv = h_rpct(50) ? 0x10 : h_rnd();
    in->suvVal = h_rpct(50) ? h_rmod(256) : h_rnd();
    model_encode(in, NULL, &m);
    if (m.total <= max_total) return;
  }
}
/* a frame the receiver can decode: the group numbers are 0 .. nGrp - 1 with the maximum present (the receiver derives nGrp = max + 1) */
static void make_consistent(I92 *in) {
  int k;
  if (in->nGrp == 0) in->nGrp = 1;
  for (k = 0; k < 6; k++) in->grp[k] = h_rmod(in->nGrp);
  in->grp[h_rmod(6)] = (uint32_t)in->nGrp - 1;
}
static void sc_info2bits(void) {
  int trial, k;
  cov_def(CV_E_SUV, "SUV frame"); cov_def(CV_E_LONG, "long frame"); cov_def(CV_E_MODE0, "mode 0 (mult forced to 1)"); cov_def(CV_E_MODE1, "mode 1"); cov_def(CV_E_MODE2, "mode > 1 (no float part)");
  cov_def(CV_E_MODENEG, "mode negative (float part)"); cov_def(CV_E_NGRP0, "no groups"); cov_def(CV_E_NGRPN, "groups"); cov_def(CV_E_FLAGB, "second array region"); cov_def(CV_E_FLAG, "flag set (cptSent)");
  cov_def(CV_E_MULT, "mult > 1"); cov_def(CV_E_FNAN, "NaN float"); cov_def(CV_E_FINF, "inf float"); cov_def(CV_E_FNEG, "negative float"); cov_def(CV_E_FZERO, "zero float"); cov_def(CV_E_FBIG, "float >= 8");
  cov_def(CV_E_PAD_EXACT, "C + 18 an exact multiple (pads a whole unit more)"); cov_def(CV_E_PAD_ROUND, "padding rounded up"); cov_def(CV_E_RATENEG, "rate byte negative"); cov_def(CV_E_DIRTY, "dirty object");
  cov_def(CV_E_B03BIG, "b03 > 1"); cov_def(CV_E_SUVBIG, "suv above 5 bits");
  h_scenario_begin(0);
  relax_reserved();
  h_covreq[CV_E_PAD_EXACT] = 0;   /* C + 18 = 0x22 + 18 or 0x88 + 136 k + 18 is never a multiple of 12 * mult: 4 | 12 mult, 154 + 136 k = 2 mod 4 (documented in NOTES) */
  for (trial = 0; trial < 4000; trial++) {
    void *o = cp_new(trial & 1, 1);
    V92CpObj *co = CO(o);
    I92 in;
    M92 m;
    uint8_t pre[T92], want[T92], *p;
    uint32_t cur, x;
    float f;
    rnd_i92(&in, 1990);
    set_i92(co, &in);
    for (k = 0; k < (int)NB; k++) co->bits_[k] = (uint8_t)(h_rpct(50) ? h_rnd() : h_rmod(2));
    for (k = 0; k < 16; k++) co->crc_[k] = (uint8_t)h_rnd();
    co->cptSent_ = h_rpct(50) ? 0 : h_rnd();
    memcpy(pre, o, T92);
    model_encode(&in, pre, &m);
    cp_i2b(o);
    memcpy(want, pre, T92);
    {
      V92CpObj *w = (V92CpObj *)want;
      for (k = 0; k < (int)m.total; k++) w->bits_[k] = m.exp[k];
      { uint8_t crc[16]; model_crc(m.exp, m.C, crc); memcpy(w->crc_, crc, 16); }
      w->widx_ = m.C + 18; w->frameLen_ = m.frameLen; w->totalLen_ = m.total;
      if (in.flag) w->cptSent_ = 1;
      if (in.type != 1) { w->modeCopy_ = in.mode; if (in.mode == 0) w->mult_ = 1; }
    }
    image_check("infoToBits", o, want, T92);
    for (k = (int)m.total; k < (int)NB; k++) if (co->bits_[k] != pre[0x129 + k]) h_die3("infoToBits wrote past totalLen (%u) at %d", m.total, k);
    if (in.type == 1) COV(CV_E_SUV); else COV(CV_E_LONG);
    if (in.type != 1) {
      if (in.mode == 0) COV(CV_E_MODE0); else if (in.mode == 1) COV(CV_E_MODE1); else if ((int8_t)in.mode > 1) COV(CV_E_MODE2); else COV(CV_E_MODENEG);
      if ((int8_t)in.mode <= 1) {
        if (in.nGrp == 0) COV(CV_E_NGRP0); else COV(CV_E_NGRPN);
        if (in.flagB && in.nGrp) COV(CV_E_FLAGB);
        for (k = 0; k < 5; k++) {
          memcpy(&f, &in.f[k], 4);
          if (f != f) COV(CV_E_FNAN); else if (f > 3.4e38f || f < -3.4e38f) COV(CV_E_FINF); else if (f < 0) COV(CV_E_FNEG); else if (f == 0) COV(CV_E_FZERO);
          if (k == 0 && f >= 8.0f) COV(CV_E_FBIG);
        }
        if (in.b03 > 1) COV(CV_E_B03BIG);
      }
      if (in.rate & 0x80) COV(CV_E_RATENEG);
    } else if ((uint16_t)in.suv > 31) COV(CV_E_SUVBIG);
    if (in.flag) COV(CV_E_FLAG);
    if (co->mult_ > 1) COV(CV_E_MULT);
    COV(CV_E_DIRTY);
    { unsigned D = 12u * co->mult_; if ((m.C + 18) % D == 0) COV(CV_E_PAD_EXACT); else COV(CV_E_PAD_ROUND); }
    p = cp_getbv(o, &cur);
    x = (uint32_t)(p - (uint8_t *)o);
    EXPECT_EQ("getBitVector offset", x, 0x129);
    EXPECT_EQ("getBitVector length", cur, m.total);
    h_emit(0, "i2b type %u mode %u total %u C %u hash %016llx", in.type, in.mode, m.total, m.C, (unsigned long long)hobj(o, T92));
    DUMP92("i2b", o);
    cp_free(o, trial & 2);
  }
}

/* ================================================================================================ scenario 1: evaluateInfo vs the decoder model */
static void sc_evalinfo(void) {
  int trial, k, g;
  cov_def(CV_V_S3, "state 3"); cov_def(CV_V_S5, "state 5"); cov_def(CV_V_S6, "state 6"); cov_def(CV_V_S7, "state 7"); cov_def(CV_V_S8, "state 8"); cov_def(CV_V_OTHER, "other state (no case)");
  cov_def(CV_V_BIGBYTE, "byte fields > 1"); cov_def(CV_V_CHAIN, "chain 5, 6, 7, 8 on one object"); cov_def(CV_V_NGRP0, "no groups"); cov_def(CV_V_SIGN, "negative float decoded");
  cov_def(CV_V_MODE_HI, "state 5 with mode > 1"); cov_def(CV_V_S4, "state 4"); cov_def(CV_V_GRPBIG, "group value above 5");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 6000; trial++) {
    void *o = cp_new(trial & 1, 1);
    V92CpObj *co = CO(o);
    static const unsigned STATES[] = {3, 5, 6, 7, 8, 4, 0, 1, 2, 9, 10, 11, 0xffffffffu, 3, 5, 6, 7, 8, 6, 5};
    uint32_t st = STATES[h_rmod(20)];
    unsigned kind = h_rmod(6);
    uint8_t want[T92], pre[T92];
    for (k = 0; k < (int)NB; k++) co->bits_[k] = kind == 0 ? (uint8_t)h_rmod(2) : kind == 1 ? (uint8_t)(h_rpct(50) ? h_rmod(2) : h_rnd()) : (uint8_t)(h_rpct(90) ? h_rmod(2) : h_rmod(4));
    if (kind == 1) COV(CV_V_BIGBYTE);
    co->state_ = st;
    if (st == 7 || st == 8) { co->nGrp_ = (uint16_t)(h_rpct(15) ? 0 : h_rmod(7)); co->cursor_ = h_rpct(60) ? 0x88 : h_rmod(500); if (co->nGrp_ == 0) COV(CV_V_NGRP0); }
    else co->cursor_ = h_rnd();
    if (st == 6 && h_rpct(60)) {   /* biased frames: a clean header, mostly valid group values */
      for (g = 0; g < 6; g++) { unsigned v = h_rmod(6); for (k = 0; k < 4; k++) co->bits_[0x67 + 4 * g + k + (g >= 4 ? 1 : 0)] = (uint8_t)((v >> k) & 1u); }
    }
    memcpy(pre, o, T92);
    memcpy(want, pre, T92);
    model_eval(want, st);
    cp_evalinfo(o);
    image_check("evaluateInfo", o, want, T92);
    switch (st) { case 3: COV(CV_V_S3); break; case 5: COV(CV_V_S5); if ((int8_t)co->mode_ > 1) COV(CV_V_MODE_HI); break; case 6: COV(CV_V_S6); { int gg; for (gg = 0; gg < 6; gg++) if (co->grp_[gg] > 5) COV(CV_V_GRPBIG); }
      { uint32_t b; memcpy(&b, &co->f14_, 4); if (b & 0x80000000u) COV(CV_V_SIGN); } break;
      case 7: COV(CV_V_S7); break; case 8: COV(CV_V_S8); break; case 4: COV(CV_V_S4); break; default: COV(CV_V_OTHER); }
    h_emit(0, "eval st %u kind %u hash %016llx", st, kind, (unsigned long long)hobj(o, T92));
    DUMP92("eval", o);
    cp_free(o, 0);
  }
  /* chain: a frame built by the model, decoded state by state on one object with the cursor continuity */
  for (trial = 0; trial < 300; trial++) {
    void *o = cp_new(0, 1);
    V92CpObj *co = CO(o);
    I92 in;
    M92 m;
    uint8_t want[T92];
    do { rnd_i92(&in, 1990); } while (in.type == 1 || (int8_t)in.mode > 1 || in.nGrp == 0 || (in.flagB == 0 && h_rpct(50)));
    make_consistent(&in);
    model_encode(&in, NULL, &m);
    for (k = 0; k < (int)NB; k++) co->bits_[k] = (uint8_t)h_rnd();
    for (k = 0; k < (int)m.total; k++) co->bits_[k] = m.exp[k];
    co->state_ = 5;
    memcpy(want, o, T92);
    model_eval(want, 5);
    cp_evalinfo(o);
    image_check("chain 5", o, want, T92);
    co->state_ = 6; ((V92CpObj *)want)->state_ = 6;
    model_eval(want, 6);
    cp_evalinfo(o);
    image_check("chain 6", o, want, T92);
    EXPECT_EQ("chain cursor after 6", co->cursor_, 0x88);
    co->state_ = 7; ((V92CpObj *)want)->state_ = 7;
    model_eval(want, 7);
    cp_evalinfo(o);
    image_check("chain 7", o, want, T92);
    if (co->flagB_) {
      co->state_ = 8; ((V92CpObj *)want)->state_ = 8;
      model_eval(want, 8);
      cp_evalinfo(o);
      image_check("chain 8", o, want, T92);
    }
    /* the decoded groups: nGrp_ = max + 1; the words of the arrays are the bit reversal of what was sent */
    for (g = 0; g < (int)co->nGrp_ * 8; g++) {
      unsigned sent = in.arrA[g], r = 0;
      for (k = 0; k < 16; k++) if (sent & (1u << k)) r |= 1u << (15 - k);
      if (co->arrA_[g] != r) h_die3("chain: array word %d is 0x%04x, bit reversal of the sent 0x%04x is 0x%04x", g, co->arrA_[g], sent, r);
    }
    COV(CV_V_CHAIN);
    h_emit(0, "chain hash %016llx", (unsigned long long)hobj(o, T92));
    DUMP92("chain", o);
    cp_free(o, 0);
  }
}

/* ================================================================================================ the receive machine's observable behaviour */
typedef struct { int32_t supp; unsigned calls; } Rx;
/* feeds one bit, checks the return value against `want` (the model's return code before the suppression window) */
static unsigned g_deliv_lvl_hook;

static int rx_step(void *o, uint8_t bit, int want_raw, Rx *rx) {
  int r = cp_b2i(o, bit);
  int want = want_raw;
  if (getenv("P11_TRACE")) fprintf(stderr, "rx call %u bit %u r %d st %u w %u c %u one %u zero %u supp %d mult %u\n", rx->calls, bit, r, CO(o)->state_, CO(o)->widx_, CO(o)->count_, CO(o)->ones_, CO(o)->zeros_, CO(o)->supp_, CO(o)->mult_);
  /* suppression window model */
  if (rx->supp < 0) {
    if (want == 1 || want == 2) rx->supp = 0;
  } else {
    if (want == 3 || want == 4) want = 0;
    rx->supp++;
    if (rx->supp == 400) rx->supp = -1;
  }
  if (r != want) h_die3("bitsToInfo returned %d, model %d (raw %d, supp %d, call %u)", r, want, want_raw, rx->supp, rx->calls);
  if (CO(o)->supp_ != rx->supp) h_die3("supp_ = %d, model %d (call %u)", CO(o)->supp_, rx->supp, rx->calls);
  rx->calls++;
  return r;
}

/* the decoded info as the model expects it, from the sent bit array (receiver image fields): applies the decoder model to a copy of the receiver image */
static void model_decode_frame(const I92 *in, const M92 *m, uint8_t *img /* receiver image */) {
  V92CpObj *o = (V92CpObj *)img;
  unsigned k;
  for (k = 0; k < m->total && k < NB; k++) o->bits_[k] = m->exp[k];
  o->type_ = m->exp[0x12];
  if (in->type == 1) { o->state_ = 3; model_eval(img, 3); }
  else {
    o->state_ = 5; model_eval(img, 5);
    if ((int8_t)o->mode_ <= 1) {
      o->state_ = 6; model_eval(img, 6);
      o->state_ = 7; model_eval(img, 7);
      if (o->flagB_) { o->state_ = 8; model_eval(img, 8); }
    }
  }
}

enum { SC_RX_PLAIN = 0, SC_RX_BADCRC, SC_RX_NOISE_BETWEEN, SC_RX_B2B };

/* ================================================================================================ scenario 2: bitsToInfo end to end */
static void sc_rx(void) {
  int trial, k;
  cov_def(CV_R_LONG, "long frame received"); cov_def(CV_R_SUV, "SUV frame received"); cov_def(CV_R_CP, "code 1 / 3 (flag 0)"); cov_def(CV_R_CPT, "code 2 / 4 (flag set)"); cov_def(CV_R_BADCRC, "bad CRC");
  cov_def(CV_R_ED, "Ed (5)"); cov_def(CV_R_NOISE, "noise between frames"); cov_def(CV_R_SUPP, "code 3 / 4 suppressed in the window"); cov_def(CV_R_SUPP_END, "suppression window ended");
  cov_def(CV_R_LVL0, "level 0 (no print)"); cov_def(CV_R_LVL2, "level 2 (print)"); cov_def(CV_R_BACK2BACK, "back-to-back frames"); cov_def(CV_R_FLAGB, "second array region received");
  cov_def(CV_R_MODE_HI, "mode > 1 frame"); cov_def(CV_R_MULT, "mult > 1"); cov_def(CV_R_EXACT_PAD, "padding with a whole extra unit (Ed)"); cov_def(CV_R_ONE_AT_TYPE, "type bit 1 (SUV)");
  cov_def(CV_R_RET1, "ret 1"); cov_def(CV_R_RET2, "ret 2"); cov_def(CV_R_RET3, "ret 3"); cov_def(CV_R_RET4, "ret 4"); cov_def(CV_R_HOOK, "bad-CRC print seen"); cov_def(CV_R_RAW, "raw byte values delivered");
  h_scenario_begin(0);
  relax_reserved();
  h_covreq[CV_R_ED] = 0; h_covreq[CV_R_EXACT_PAD] = 0;   /* never an exact multiple, see scenario 0 */
  for (trial = 0; trial < 500; trial++) {
    void *tx = cp_new(0, 1);
    void *rxo;
    V92CpObj *ct = CO(tx);
    I92 in;
    M92 m;
    Rx rx;
    uint8_t rximg[T92];
    unsigned nframes = 1 + h_rmod(3), fr, mult = 1 + h_rmod(h_rpct(30) ? 6 : 1);
    unsigned lvl = h_rpct(6) ? (h_rpct(50) ? 0x80000000u + h_rmod(3) : 0xffffffffu - h_rmod(2)) : (h_rpct(60) ? 0 : (h_rpct(70) ? 2 : 1 + h_rmod(5)));
    rxo = onew(T92);
    garbage_fill(rxo, T92, h_rnd());
    cp_c1(rxo);
    CO(rxo)->mult_ = (uint8_t)mult;
    rx.supp = -1; rx.calls = 0;
    dsplibs_debug_level = lvl;
    if (lvl > 1) COV(CV_R_LVL2); else COV(CV_R_LVL0);
    g_dbg_in_call = 0;
    for (fr = 0; fr < nframes; fr++) {
      unsigned kind = h_rpct(75) ? SC_RX_PLAIN : (h_rpct(60) ? SC_RX_BADCRC : SC_RX_NOISE_BETWEEN);
      unsigned D, end, raw, dbg0;
      int p, want_code;
      uint8_t buf[2200];
      rnd_i92(&in, 1900);
      in.mult = (uint8_t)mult;
      if (in.type != 1 && in.mode == 0) { /* mult forced to 1 by the encoder */ }
      if (in.type != 1 && h_rpct(60)) in.type = 0;   /* long frames (type bit 0; other nonzero bytes are stored whole: only the bit matters on the wire) */
      if (in.type > 1) in.type = 0;
      in.flag = (uint8_t)(h_rpct(50) ? 0 : 1);
      in.b03 = (uint8_t)(h_rpct(70) ? h_rmod(2) : h_rnd());
      in.suvVal = h_rmod(256);
      in.mode &= 3;   /* only two mode bits travel on the wire */
      make_consistent(&in);
      set_i92(ct, &in);
      ct->state_ = 0;
      cp_i2b(tx);
      model_encode(&in, NULL, &m);
      /* the multiplier the blob used (the encoder forces 1 for mode 0 long frames: the receiver's own mult_ must be the same for a clean receive) */
      D = 12u * ct->mult_;
      if (CO(rxo)->mult_ != ct->mult_) CO(rxo)->mult_ = ct->mult_;
      if (D > 12) COV(CV_R_MULT);
      for (p = 0; p < (int)m.total; p++) buf[p] = ct->bits_[p];
      for (p = 0; p < (int)m.total; p++) if (buf[p] != m.exp[p]) h_die3("sent bit %d is %u, model %u", p, buf[p], m.exp[p]);
      if (kind == SC_RX_BADCRC) {   /* damage one non-structural payload bit that is not a count: a payload bit of the arrays / floats, not on the grid, not the type, not the header */
        unsigned q;
        if (in.type != 1 && (int8_t)in.mode <= 1 && m.C > 0x90) { q = h_rpct(40) ? 0x34 + h_rmod(0x66 - 0x34) : 0x89 + h_rmod(m.C - 0x89); if (q % 17 && q < m.C) buf[q] ^= 1; else kind = SC_RX_PLAIN; }
        else kind = SC_RX_PLAIN;
      }
      dbg0 = g_dbg_in_call;
      end = ((m.C + 18 + D - 1) / D) * D;     /* the receiver ends the frame at the first multiple of D at or above C + 18 */
      raw = 0;
      /* idle ones + a few random zeros before / noise */
      if (fr == 0 || kind == SC_RX_NOISE_BETWEEN) {
        unsigned nn = h_rmod(60);
        { unsigned run = 0; for (k = 0; k < (int)nn; k++) { int c = h_rpct(30) ? 0 : 1; if (run >= 10) c = 0; run = c ? run + 1 : 0; rx_step(rxo, (uint8_t)c, 0, &rx); } }
        /* a receiver left in the middle of noise: ones count must restart; model: every 17+ ones in a row then a 0 (start bit) begins a frame; our noise ends with a 0 bit, so
           the next frame's 17 ones restart the count: feed one zero, then the frame */
        rx_step(rxo, 0, 0, &rx);
        COV(CV_R_NOISE);
        { unsigned zc = 0; for (k = 0; k < (int)nn; k++) (void)zc; }
      }
      /* the frame: bits[0..0x10] = 1 (17 ones) start the receiver; but the noise's last zero count must not reach 12 * mult (Ed): keep it short (1 zero) */
      for (p = 0; p < (int)m.total; p++) {
        int wr = 0;
        if ((unsigned)p == end - 1 && kind != SC_RX_BADCRC) {
          want_code = in.type == 1 ? (in.flag ? 4 : 3) : (in.flag ? 2 : 1);
          wr = want_code;
          if (getenv("P11_TRACE")) fprintf(stderr, "expect code %d at p %d (end %u C %u total %u D %u type %u mode %u)\n", wr, p, end, m.C, m.total, D, in.type, in.mode);
        }
        if (kind == SC_RX_BADCRC && (unsigned)p == m.C + 16) { /* the CRC check happens after the 17th bit of the CRC tail: bad -> back to idle */ }
        if (kind == SC_RX_BADCRC) {
          /* after a bad CRC the receiver is idle: the remaining bits of the frame are fed as noise-like zeros and ones; only check the state: no frame code can come out */
          int r = cp_b2i(rxo, buf[p]);
          if (r == 1 || r == 2) h_die3("bad-CRC frame produced code %d at bit %d", r, p);
          if (r == 3 || r == 4) { /* a SUV-looking structure cannot appear (type bit of the damaged frame is intact, only payload damaged) */ h_die3("bad-CRC frame produced code %d", r); }
          rx.calls++;
          if (CO(rxo)->supp_ >= 0) rx.supp = CO(rxo)->supp_;
          continue;
        }
        if (buf[p] > 1) raw++;
        rx_step(rxo, buf[p], wr, &rx);
      }
      if (raw) COV(CV_R_RAW);
      if (kind == SC_RX_BADCRC) {
        COV(CV_R_BADCRC);
        if (lvl > 1) { if (g_dbg_in_call != dbg0 + 1 || strcmp(g_dbg_text[dbg0], "V92CP: recieved CP with bad CRC\\r\\n")) h_die3("bad CRC print: %u prints, text '%s'", g_dbg_in_call - dbg0, g_dbg_text[dbg0]); COV(CV_R_HOOK); }
        else if (g_dbg_in_call != dbg0) h_die3("bad CRC printed at level %u", lvl); else COV(CV_R_HOOK);
        /* the receiver must be back in the idle state with widx 0x12, state 0 (or later in the next sync): model: after the bad CRC, state_ == 0 and ones/zeros cleared at that call;
           the padding bits that followed were consumed as idle bits (zeros do not move state 0; a zero run of 12 * mult with widx 0x12 would give an Ed (5) code, not 1 .. 4) */
        if (CO(rxo)->state_ != 0) h_die3("state after a bad CRC frame: %u (frame C %u total %u mode %u type %u)", CO(rxo)->state_, m.C, m.total, in.mode, in.type);
        { uint32_t sv = CO(rxo)->state_; (void)sv; }
        rx.supp = CO(rxo)->supp_;
        /* resynchronise: 60 zero bits do not matter; to be safe send a long run of ones and a zero, which restarts state 0 -> 1 -> 2 properly */
        for (k = 0; k < 20; k++) { int r = cp_b2i(rxo, 1); rx.calls++; if (r) h_die3("code %d during resync ones", r); }
        { int r = cp_b2i(rxo, 0); rx.calls++; (void)r; }
        { int r = cp_b2i(rxo, 1); rx.calls++; (void)r; }
        /* push the machine into a known state through the model-less path: finish whatever it is doing with zeros until idle; frames of the SAME object continue next iteration only
           if state_ == 0 here, so simply reset the receiver (the detector reset is another function, tested in scenario 6): we recreate it */
        rx.supp = -1;
        cp_d1(rxo);
        rxo = onew(T92);
        garbage_fill(rxo, T92, h_rnd());
        cp_c1(rxo);
        CO(rxo)->mult_ = (uint8_t)mult;
        continue;
      }
      /* the decoded info (the model applies the decoder to the received frame) */
      memcpy(rximg, rxo, T92);
      {
        uint8_t img[T92];
        V92CpObj *w = (V92CpObj *)img;
        memcpy(img, rximg, T92);
        model_decode_frame(&in, &m, img);
        /* the fields the receive machine itself leaves in a state: compare only what the frame carries */
        if (in.type == 1) {
          EXPECT_EQ("rx suv_", CO(rxo)->suv_, w->suv_); EXPECT_EQ("rx suvVal_", CO(rxo)->suvVal_, w->suvVal_); EXPECT_EQ("rx flag_", CO(rxo)->flag_, w->flag_);
          COV(CV_R_SUV); COV(CV_R_ONE_AT_TYPE);
        } else {
          EXPECT_EQ("rx mode_", CO(rxo)->mode_, w->mode_); EXPECT_EQ("rx rate_", CO(rxo)->rate_, w->rate_); EXPECT_EQ("rx flag_", CO(rxo)->flag_, w->flag_);
          COV(CV_R_LONG);
          if ((int8_t)in.mode > 1) COV(CV_R_MODE_HI);
          else {
            EXPECT_EQ("rx w08_", CO(rxo)->w08_, w->w08_); EXPECT_EQ("rx w0c_", CO(rxo)->w0c_, w->w0c_); EXPECT_EQ("rx b03_", CO(rxo)->b03_, w->b03_);
            image_check("rx floats f10", &CO(rxo)->f10_, &w->f10_, 20);
            image_check("rx grp", CO(rxo)->grp_, w->grp_, 24);
            EXPECT_EQ("rx nGrp_", CO(rxo)->nGrp_, w->nGrp_); EXPECT_EQ("rx flagB_", CO(rxo)->flagB_, w->flagB_);
            image_check("rx arrA", CO(rxo)->arrA_, w->arrA_, (size_t)w->nGrp_ * 16);
            if (w->flagB_) { image_check("rx arrB", CO(rxo)->arrB_, w->arrB_, (size_t)w->nGrp_ * 16); COV(CV_R_FLAGB); }
          }
        }
        if (in.flag) COV(CV_R_CPT); else COV(CV_R_CP);
        if (in.type == 1) { if (in.flag) COV(CV_R_RET4); else COV(CV_R_RET3); } else { if (in.flag) COV(CV_R_RET2); else COV(CV_R_RET1); }
      }
      /* the whole padding behind the end bit: if the encoder padded a whole extra unit the receiver sees D zero bits in idle state: the last one is Ed (5) */
      if ((m.C + 18) % D == 0) {
        for (p = 0; p < (int)D; p++) {
          int r = cp_b2i(rxo, 0);
          rx.calls++;
          if (CO(rxo)->supp_ >= 0 && rx.supp >= 0) { rx.supp++; if (rx.supp == 400) rx.supp = -1; }
          if (CO(rxo)->supp_ != rx.supp) { rx.supp = CO(rxo)->supp_; }
          if ((unsigned)p == D - 1 && D < 256) { if (r != 5) h_die3("Ed expected (5), got %d", r); COV(CV_R_ED); COV(CV_R_EXACT_PAD); }
          else if (r) h_die3("code %d inside padding", r);
        }
      }
      if (fr + 1 < nframes) COV(CV_R_BACK2BACK);
      h_emit(0, "rx frame %u type %u mode %u C %u total %u hash %016llx", fr, in.type, in.mode, m.C, m.total, (unsigned long long)hobj(rxo, T92));
      DUMP92("rx", rxo);
      (void)raw;
    }
    /* idle run through the suppression window (an SUV frame inside the window is reported as 0) */
    {
      unsigned n = h_rpct(30) ? 450 : h_rmod(30);
      for (k = 0; k < (int)n; k++) {
        int r = cp_b2i(rxo, (uint8_t)(k % 7 == 3 ? 1 : 0));
        (void)r;
        rx.calls++;
        if (CO(rxo)->supp_ == -1 && rx.supp >= 0 && n > 380) { COV(CV_R_SUPP_END); rx.supp = -1; }
      }
      if (n >= 400) { COV(CV_R_SUPP); }
    }
    h_emit(1, "rx trial %d calls %u hash %016llx dbg %u", trial, rx.calls, (unsigned long long)hobj(rxo, T92), g_dbg_in_call);
    DUMP92("rxend", rxo);
    cp_d1(rxo);
    cp_free(tx, 0);
  }
  (void)g_deliv_lvl_hook;
}

/* ================================================================================================ scenario 3: bitsToInfo on random object states */
static void sc_rxfuzz(void) {
  int trial, step, k;
  cov_def(CV_F_S0, "state 0"); cov_def(CV_F_S1, "state 1"); cov_def(CV_F_S2, "state 2"); cov_def(CV_F_S3, "state 3"); cov_def(CV_F_S4, "state 4"); cov_def(CV_F_S5, "state 5"); cov_def(CV_F_S6, "state 6");
  cov_def(CV_F_S7, "state 7"); cov_def(CV_F_S8, "state 8"); cov_def(CV_F_S9, "state 9"); cov_def(CV_F_S10, "state 10"); cov_def(CV_F_SBIG, "state > 10"); cov_def(CV_F_BIT0, "bit 0"); cov_def(CV_F_BIT1, "bit 1");
  cov_def(CV_F_ED, "Ed"); cov_def(CV_F_RET1, "ret 1"); cov_def(CV_F_RET2, "ret 2"); cov_def(CV_F_RET3, "ret 3"); cov_def(CV_F_RET4, "ret 4"); cov_def(CV_F_RET5, "ret 5"); cov_def(CV_F_SUPP_NEG, "supp < 0");
  cov_def(CV_F_SUPP_POS, "supp >= 0"); cov_def(CV_F_SUPP_END, "supp reached 400"); cov_def(CV_F_CRC_OK, "state 9 CRC ok"); cov_def(CV_F_CRC_BAD, "state 9 CRC bad"); cov_def(CV_F_S5_HI, "state 5 end with mode > 1");
  cov_def(CV_F_S7_B, "state 7 end with flagB"); cov_def(CV_F_S7_NOB, "state 7 end without flagB"); cov_def(CV_F_S10_END, "state 10 frame end"); cov_def(CV_F_S10_ONE, "state 10 with a 1 bit"); cov_def(CV_F_ZEROS_WRAP, "zeros_ wrap");
  cov_def(CV_F_GAMMA, "state 7 compares gamma"); cov_def(CV_F_DELTA, "state 8 compares delta");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 3000; trial++) {
    void *o = cp_new(0, 1);
    V92CpObj *co = CO(o);
    unsigned lvl = h_rpct(8) ? (h_rpct(50) ? 0x80000000u + h_rmod(3) : 0xffffffffu - h_rmod(2)) : (h_rpct(60) ? 2 : h_rmod(4));
    unsigned st;
    int chain = 0;
    uint8_t mult = (uint8_t)(h_rpct(70) ? 1 + h_rmod(3) : (h_rpct(50) ? 1 + h_rmod(40) : 1 + (h_rnd() % 255)));
    static const unsigned STS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0xffffffffu, 0x80000000u};
    for (k = 0; k < (int)NB; k++) co->bits_[k] = (uint8_t)(h_rpct(80) ? h_rmod(2) : h_rnd());
    st = STS[h_rmod(15)];
    co->state_ = st;
    co->mult_ = mult;
    co->widx_ = h_rpct(30) ? 0x12 : (h_rpct(50) ? 0x22 - (unsigned)h_rmod(2) : h_rmod(1900));
    if (st == 3 || st == 5) co->widx_ = 0x22 - 1 - h_rmod(3) * (h_rpct(30));
    if (st == 6) co->widx_ = 0x88 - 1 - h_rmod(2) * (h_rpct(30));
    co->count_ = h_rpct(40) ? 0 : h_rmod(0x30);
    co->ones_ = (uint8_t)(h_rpct(50) ? 0x10 : h_rnd());
    co->zeros_ = (uint8_t)(h_rpct(50) ? 12u * mult - 1 : h_rnd());
    co->supp_ = h_rpct(40) ? -1 : (h_rpct(50) ? 399 : (int32_t)h_rmod(400));
    co->nGrp_ = (uint16_t)h_rmod(7);
    co->flagB_ = (uint8_t)h_rmod(2);
    co->mode_ = (uint8_t)(h_rpct(50) ? h_rmod(4) : h_rnd());
    co->type_ = (uint8_t)h_rmod(3);
    co->flag_ = (uint8_t)h_rmod(2);
    co->cursor_ = h_rmod(300);
    if (st == 9) { co->count_ = h_rpct(60) ? 0x10 : h_rmod(0x12); co->widx_ = 0x22 + h_rmod(600); }
    if (st == 10) { co->widx_ = 0x12 + h_rmod(300); if (h_rpct(40)) co->widx_ = 12u * mult - 1 + 12u * mult * h_rmod(3); if (co->widx_ > 1900) co->widx_ = 12u * mult - 1; }
    if (st == 7 || st == 8) co->count_ = 136u * co->nGrp_ - 1 - (h_rpct(30) ? 1 : 0);
    if (st == 6 && h_rpct(75)) {   /* a whole receive chain 6 -> 7 -> (8) -> 9 -> 10 on random bits: valid group values keep the part lengths inside the buffer */
      int g2;
      chain = 1;
      co->widx_ = 0x87;
      for (g2 = 0; g2 < 6; g2++) { unsigned v = h_rmod(6); for (k = 0; k < 4; k++) co->bits_[0x67 + 4 * g2 + k + (g2 >= 4 ? 1 : 0)] = (uint8_t)((v >> k) & 1u); }
      co->bits_[0x80] = (uint8_t)h_rmod(2);
      co->nGrp_ = 0;
    }
    dsplibs_debug_level = lvl;
    g_dbg_in_call = 0;
    for (step = 0; step < (chain ? 2400 : 40) && co->widx_ + 4 < NB; step++) {
      uint8_t bit = (uint8_t)(h_rpct(55) ? (h_rpct(60) ? 0 : 1) : (h_rpct(5) ? h_rnd() : h_rmod(2)));
      int r;
      uint32_t s0 = co->state_;
      unsigned long ph;
      if (step == 0 && st == 9 && h_rpct(50)) {   /* steer the CRC of state 9 to come out right */
        uint8_t crc[16];
        unsigned w = co->widx_, fl, j;
        co->count_ = 0x10;
        fl = w + 1;
        for (j = 0; j < 16; j++) co->bits_[fl - 16 + j] = 0;
        model_crc(co->bits_, fl - 0x11, crc);
        for (j = 0; j < 15; j++) co->bits_[fl - 16 + j] = crc[j];
        /* the last CRC bit is the bit we are about to feed */
        bit = crc[15];
        if (w + 1 >= 0x22) { /* the model CRC covers up to fl - 0x11, the register compare reads bits[fl-16 .. fl-1]; bit goes to bits[w] = bits[fl - 1] */ }
      }
      (void)ph;
      switch (s0) { case 0: COV(CV_F_S0); break; case 1: COV(CV_F_S1); break; case 2: COV(CV_F_S2); break; case 3: COV(CV_F_S3); break; case 4: COV(CV_F_S4); break; case 5: COV(CV_F_S5); break; case 6: COV(CV_F_S6); break;
        case 7: COV(CV_F_S7); break; case 8: COV(CV_F_S8); break; case 9: COV(CV_F_S9); break; case 10: COV(CV_F_S10); break; default: COV(CV_F_SBIG); }
      if (bit) COV(CV_F_BIT1); else COV(CV_F_BIT0);
      if (co->supp_ < 0) COV(CV_F_SUPP_NEG); else COV(CV_F_SUPP_POS);
      if (co->supp_ == 399) COV(CV_F_SUPP_END);
      if (s0 == 10 && bit) COV(CV_F_S10_ONE);
      if (s0 == 9 && co->count_ == 0x10) { /* the CRC outcome is read after the call */ }
      r = cp_b2i(o, bit);
      if (r == 5) { COV(CV_F_RET5); COV(CV_F_ED); } else if (r == 1) COV(CV_F_RET1); else if (r == 2) COV(CV_F_RET2); else if (r == 3) COV(CV_F_RET3); else if (r == 4) COV(CV_F_RET4);
      if (s0 == 9 && co->state_ == 10) COV(CV_F_CRC_OK);
      if (s0 == 9 && co->state_ == 0 && s0 != co->state_ && co->count_ == 0) COV(CV_F_CRC_BAD);
      if (s0 == 10 && (r >= 1 && r <= 4)) COV(CV_F_S10_END);
      if (s0 == 5 && co->state_ == 9) COV(CV_F_S5_HI);
      if (s0 == 7 && co->state_ == 8) COV(CV_F_S7_B);
      if (s0 == 7 && co->state_ == 9) COV(CV_F_S7_NOB);
      if (s0 == 7) COV(CV_F_GAMMA);
      if (s0 == 8) COV(CV_F_DELTA);
      if (co->zeros_ == 0 && bit == 0) COV(CV_F_ZEROS_WRAP);
      h_emit(0, "fz s%u b%u r%d st%u w%u c%u o%u z%u sp%d", s0, bit, r, co->state_, co->widx_, co->count_, co->ones_, co->zeros_, co->supp_);
      if ((step & 7) == 0) { h_emit(0, "fz hash %016llx", (unsigned long long)hobj(o, T92)); }
    }
    h_emit(0, "fz end %016llx dbg %u", (unsigned long long)hobj(o, T92), g_dbg_in_call);
    DUMP92("fz", o);
    cp_free(o, 0);
  }
}

/* objects with a canary tail: a function writing past its block (loop bounds one too far) is caught */
static void *onew_canary(size_t n) { void *p = onew(n + 32); memset((uint8_t *)p + n, 0xc3, 32); return p; }
static void canary_check(const void *p, size_t n, const char *what) {
  unsigned c;
  for (c = 0; c < 32; c++) if (((const uint8_t *)p)[n + c] != 0xc3) h_die3("%s wrote past the %zu-byte block (byte +%u)", what, n, c);
}
/* ================================================================================================ scenario 4: pack / unpack functions */
typedef struct { uint8_t listA[6][128], listB[6][128]; uint32_t cnt[6]; } Lists;
static void rnd_lists(Lists *l, int dup_kind) {
  int g, k;
  memset(l, 0, sizeof *l);
  for (g = 0; g < 6; g++) {
    unsigned n = h_rpct(15) ? 0 : h_rmod(h_rpct(20) ? 129 : 30);
    l->cnt[g] = n;
    for (k = 0; k < 128; k++) { l->listA[g][k] = (uint8_t)h_rmod(h_rpct(5) ? 256 : 128); l->listB[g][k] = (uint8_t)h_rmod(h_rpct(5) ? 256 : 128); }
  }
  if (dup_kind) {   /* make some groups identical (A, B and count), some only equal in A, some only in the count */
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
      if (how == 0) { /* identical */ }
    }
  }
}
static void lists_to_mp(MapPar90 *mp, const Lists *l) {
  int g;
  for (g = 0; g < 6; g++) { memcpy(mp->listA_ + 128 * g, l->listA[g], 128); memcpy(mp->listB_ + 128 * g, l->listB[g], 128); mp->cnt_[g] = l->cnt[g]; }
}
/* independent pack model: masks of a list (positions to bits) */
static void model_mask(uint16_t out[8], const uint8_t *list, unsigned n) {
  unsigned e;
  uint16_t big[16];
  memset(big, 0, sizeof big);
  for (e = 0; e < n; e++) big[list[e] >> 4] |= (uint16_t)(1u << (list[e] & 15));
  memcpy(out, big, 16);   /* positions >= 128 land past the 8 words (excluded by the generator mostly) */
}

static void rnd_mp(MapPar90 *mp, int dup) {
  Lists l;
  int k;
  garbage_fill(mp, sizeof *mp, h_rnd());
  rnd_lists(&l, dup);
  lists_to_mp(mp, &l);
  mp->w0_ = h_rpct(60) ? 8 + h_rmod(60) : h_rnd();
  mp->flag_ = h_rpct(50) ? h_rmod(2) : h_rnd();
  mp->w620_ = h_rpct(60) ? h_rmod(4) : h_rnd();
  mp->w624_ = h_rpct(60) ? h_rmod(4) : h_rnd();
  { uint32_t b[4]; for (k = 0; k < 4; k++) b[k] = rfloat(k == 0 ? 1 : 0); memcpy(&mp->f628_, &b[0], 4); memcpy(&mp->f62c_, &b[1], 4); memcpy(&mp->f630_, &b[2], 4); memcpy(&mp->f634_, &b[3], 4); }
}

static void sc_pack(void) {
  int trial, k, g;
  cov_def(CV_P_PCK, "setV92CPpckFromParamsInfo"); cov_def(CV_P_DEDUP, "a duplicate group found"); cov_def(CV_P_NODEDUP, "all groups distinct"); cov_def(CV_P_PARTIAL, "a group equal in A but not B / count");
  cov_def(CV_P_B, "flag: second region"); cov_def(CV_P_NOB, "no second region"); cov_def(CV_P_MODE0, "mode 0 (rate offset 8)"); cov_def(CV_P_MODE1, "mode != 0 (offset 0x14)"); cov_def(CV_P_FROM92, "setParamsInfoFromV92CPUnPck");
  cov_def(CV_P_FROM92_B, "from V92CP with flagB"); cov_def(CV_P_FROM92_NOB, "from V92CP without flagB"); cov_def(CV_P_FROMUN, "setParamsInfoFromCPUnPck"); cov_def(CV_P_FROMUN_B, "from CPUnPck with flag");
  cov_def(CV_P_FROMUN_NOB, "from CPUnPck without flag"); cov_def(CV_P_SUB_NZ, "sub word != 0"); cov_def(CV_P_SUB_Z, "sub word == 0"); cov_def(CV_P_RT, "round trip pack -> unpack"); cov_def(CV_P_CNT0, "empty list");
  cov_def(CV_P_NGRP0, "n groups"); cov_def(CV_P_BIGPOS, "list element >= 128"); cov_def(CV_P_SNAN, "signalling NaN word copied");
  h_scenario_begin(0);
  relax_reserved();
  h_covreq[CV_P_NGRP0] = 0;
  for (trial = 0; trial < 2500; trial++) {
    unsigned which = h_rmod(5);
    h_pool_reset();
    if (which < 3) {   /* setV92CPpckFromParamsInfo */
      MapPar90 *mp = (MapPar90 *)onew_canary(sizeof(MapPar90));
      AddCp *ad = (AddCp *)onew(sizeof(AddCp));
      V92CpObj *cp = (V92CpObj *)onew_canary(T92);
      uint8_t pre_mp[sizeof(MapPar90)], want[T92], want_mp[sizeof(MapPar90)];
      Lists l;
      unsigned n, idx[6], grp[6], gg, b, d, i;
      V92CpObj *w;
      MapPar90 *wm;
      rnd_mp(mp, trial & 1);
      if (h_rpct(10)) { int gx; for (gx = 1; gx < 6; gx++) { memcpy(mp->listA_ + 128 * gx, mp->listA_, 128); memcpy(mp->listB_ + 128 * gx, mp->listB_, 128); mp->cnt_[gx] = mp->cnt_[0]; } }
      else if (h_rpct(10)) { int gx; for (gx = 2; gx < 6; gx++) { int src = h_rpct(50) ? 0 : 1; memcpy(mp->listA_ + 128 * gx, mp->listA_ + 128 * src, 128); memcpy(mp->listB_ + 128 * gx, mp->listB_ + 128 * src, 128); mp->cnt_[gx] = mp->cnt_[src]; } }
      memcpy(&l.listA, mp->listA_, sizeof l.listA); memcpy(&l.listB, mp->listB_, sizeof l.listB); for (g = 0; g < 6; g++) l.cnt[g] = mp->cnt_[g];
      if (h_rpct(70)) for (g = 0; g < 6; g++) for (k = 0; k < 128; k++) { mp->listA_[128 * g + k] &= 127; mp->listB_[128 * g + k] &= 127; }
      else for (g = 0; g < 6; g++) for (k = 0; k < 128; k++) if (mp->listA_[128 * g + k] >= 128 || mp->listB_[128 * g + k] >= 128) COV(CV_P_BIGPOS);
      for (g = 0; g < 6; g++) { for (k = 0; k < 128; k++) { l.listA[g][k] = mp->listA_[128 * g + k]; l.listB[g][k] = mp->listB_[128 * g + k]; } l.cnt[g] = mp->cnt_[g]; if (l.cnt[g] == 0) COV(CV_P_CNT0); }
      /* big positions would index past the 8 words (and into the next group): the blob and the port write the same bytes, but keep the model simple */
      for (g = 0; g < 6; g++) for (k = 0; k < (int)l.cnt[g]; k++) if (l.listA[g][k] >= 128 || l.listB[g][k] >= 128) { l.listA[g][k] &= 127; l.listB[g][k] &= 127; mp->listA_[128 * g + k] = l.listA[g][k]; mp->listB_[128 * g + k] = l.listB[g][k]; }
      garbage_fill(cp, T92, h_rnd());
      ad->w0_ = h_rpct(50) ? h_rmod(2) : h_rnd(); ad->w4_ = h_rpct(50) ? h_rmod(2) : h_rnd(); { uint32_t b8 = rfloat(1); memcpy(&ad->f8_, &b8, 4); } ad->wc_ = h_rpct(50) ? h_rmod(2) : h_rnd(); ad->w10_ = h_rnd(); ad->w14_ = h_rnd();
      { uint32_t b8; memcpy(&b8, &ad->f8_, 4); if ((b8 & 0x7f800000u) == 0x7f800000u && (b8 & 0x00400000u) == 0 && (b8 & 0x7fffffu)) COV(CV_P_SNAN); }
      memcpy(pre_mp, mp, sizeof pre_mp);
      /* model: dedupe */
      n = 1; idx[0] = 0; grp[0] = 0;
      for (gg = 1; gg < 6; gg++) {
        int found = 0;
        for (b = 0; b < n && !found; b++) {
          d = idx[b];
          if (l.cnt[gg] != l.cnt[d]) continue;
          if (memcmp(l.listA[gg], l.listA[d], l.cnt[gg]) == 0 && memcmp(l.listB[gg], l.listB[d], l.cnt[gg]) == 0) { grp[gg] = grp[d]; found = 1; }
        }
        if (!found) { idx[n] = gg; grp[gg] = n; n++; }
        else COV(CV_P_DEDUP);
        if (found && (l.cnt[gg] == 0)) { }
      }
      if (n == 6) COV(CV_P_NODEDUP);
      for (gg = 1; gg < 6; gg++) for (d = 0; d < gg; d++) if (l.cnt[gg] == l.cnt[d] && !memcmp(l.listA[gg], l.listA[d], l.cnt[gg]) && memcmp(l.listB[gg], l.listB[d], l.cnt[gg])) COV(CV_P_PARTIAL);
      memcpy(want, cp, T92);
      memcpy(want_mp, pre_mp, sizeof want_mp);
      w = (V92CpObj *)want; wm = (MapPar90 *)want_mp;
      for (i = 0; i < n; i++) wm->idx_[i] = idx[i];
      wm->idx_[0] = 0;
      for (gg = 0; gg < 6; gg++) w->grp_[gg] = grp[gg];
      w->nGrp_ = (uint16_t)n;
      w->flag_ = (uint8_t)ad->w0_; w->mode_ = (uint8_t)ad->w4_; w->b03_ = (uint8_t)ad->wc_; memcpy(&w->f10_, &ad->f8_, 4); w->suvVal_ = ad->w10_;
      memcpy(&w->f14_, &mp->f628_, 4); w->w08_ = mp->w620_; memcpy(&w->f18_, &mp->f62c_, 4); w->w0c_ = mp->w624_; memcpy(&w->f1c_, &mp->f630_, 4); w->flagB_ = (uint8_t)mp->flag_; memcpy(&w->f20_, &mp->f634_, 4);
      for (gg = 0; gg < n; gg++) { memset(w->arrA_ + 8 * gg, 0, 16); }
      for (gg = 0; gg < n; gg++) model_mask(w->arrA_ + 8 * gg, l.listA[idx[gg]], l.cnt[idx[gg]]);
      if (w->flagB_ && n) { for (gg = 0; gg < n; gg++) model_mask(w->arrB_ + 8 * gg, l.listB[idx[gg]], l.cnt[idx[gg]]); COV(CV_P_B); } else COV(CV_P_NOB);
      w->rate_ = (uint8_t)(mp->w0_ - (w->mode_ ? 0x14u : 8u));
      if (w->mode_) COV(CV_P_MODE1); else COV(CV_P_MODE0);
      f_pck(mp, ad, cp);
      canary_check(mp, sizeof(MapPar90), "setV92CPpckFromParamsInfo mp"); canary_check(cp, T92, "setV92CPpckFromParamsInfo cp");
      COV(CV_P_PCK);
      image_check("setV92CPpckFromParamsInfo cp", cp, want, T92);
      image_check("setV92CPpckFromParamsInfo mp", mp, want_mp, sizeof want_mp);
      h_emit(0, "pck n %u hash %016llx %016llx", n, (unsigned long long)hobj(cp, T92), (unsigned long long)hobj(mp, sizeof *mp));
      DUMP92("pck", cp); DUMP90("pckmp", mp);
      /* round trip: unpack the V92CP into a fresh mapping parameters block: the lists come back as the descending position lists of the masks */
      if (which == 0) {
        MapPar90 *rt = (MapPar90 *)onew_canary(sizeof(MapPar90));
        garbage_fill(rt, sizeof *rt, h_rnd());
        f_from92(rt, cp);
        canary_check(rt, sizeof(MapPar90), "setParamsInfoFromV92CPUnPck");
        COV(CV_P_FROM92);
        if (cp->flagB_) COV(CV_P_FROM92_B); else COV(CV_P_FROM92_NOB);
        for (gg = 0; gg < 6; gg++) {
          uint16_t mA[8], mB[8], rA[8], rB[8];
          uint8_t src_a[128], src_b[128];
          unsigned c0 = rt->cnt_[gg], e;
          /* the unpacked list of slot gg describes the masks of group grp[gg]: descending positions */
          model_mask(mA, l.listA[gg], l.cnt[gg]);
          model_mask(mB, cp->flagB_ ? l.listB[gg] : l.listA[gg], cp->flagB_ ? l.cnt[gg] : l.cnt[gg]);
          memcpy(src_a, rt->listA_ + 128 * gg, 128); memcpy(src_b, rt->listB_ + 128 * gg, 128);
          (void)c0; (void)e; (void)mA; (void)mB; (void)rA; (void)rB; (void)src_a; (void)src_b;
        }
        /* the model of the unpack: cnt = popcount of the (B if flagB else A) region of group grp[g]; elements = descending positions, both lists */
        for (gg = 0; gg < 6; gg++) {
          unsigned gi = cp->grp_[gg], pa = 0, pb = 0, j, bb;
          uint8_t la[128], lb[128];
          const uint16_t *ra = cp->arrA_ + 8 * gi, *rb = (cp->flagB_ ? cp->arrB_ : cp->arrA_) + 8 * gi;
          for (j = 8; j-- > 0;) for (bb = 16; bb-- > 0;) { if (ra[j] & (1u << (15 - bb))) la[pa++] = (uint8_t)(16 * j + bb); }
          for (j = 8; j-- > 0;) for (bb = 16; bb-- > 0;) { if (rb[j] & (1u << (15 - bb))) lb[pb++] = (uint8_t)(16 * j + bb); }
          /* the B pass sets the count last */
          EXPECT_EQ("round trip count", rt->cnt_[gg], pb);
          if (memcmp(rt->listA_ + 128 * gg, la, pa)) h_die3("round trip: list A of group %u differs from the model", gg);
          if (memcmp(rt->listB_ + 128 * gg, lb, pb)) h_die3("round trip: list B of group %u differs from the model", gg);
          EXPECT_EQ("round trip idx", rt->idx_[gg], cp->grp_[gg]);
        }
        EXPECT_EQ("round trip w0", rt->w0_, (uint32_t)((int)(int8_t)cp->rate_ + (cp->mode_ ? 0x14 : 8)));
        EXPECT_EQ("round trip flag", rt->flag_, cp->flagB_ != 0);
        COV(CV_P_RT);
        h_emit(0, "from92 hash %016llx", (unsigned long long)hobj(rt, sizeof *rt));
        DUMP90("from92", rt);
      }
    } else if (which == 3) {   /* setParamsInfoFromV92CPUnPck with a garbage V92CP (any words, grp < 6 mostly) */
      MapPar90 *mp = (MapPar90 *)onew_canary(sizeof(MapPar90));
      V92CpObj *cp = (V92CpObj *)onew_canary(T92);
      MapPar90 *wm = NULL;
      uint8_t want_mp[sizeof(MapPar90)];
      unsigned gg, j, bb;
      garbage_fill(mp, sizeof *mp, h_rnd());
      garbage_fill(cp, T92, h_rnd());
      for (gg = 0; gg < 6; gg++) cp->grp_[gg] = h_rpct(95) ? h_rmod(6) : h_rmod(7);
      for (gg = 0; gg < 6; gg++) if (cp->grp_[gg] > 5) cp->grp_[gg] = 5;
      cp->flagB_ = (uint8_t)(h_rpct(50) ? 0 : (h_rpct(50) ? 1 : h_rnd()));
      cp->mode_ = (uint8_t)(h_rpct(50) ? 0 : h_rnd());
      for (j = 0; j < 48; j++) { cp->arrA_[j] = (uint16_t)(h_rpct(30) ? 0 : r16()); cp->arrB_[j] = (uint16_t)(h_rpct(30) ? 0 : r16()); }
      memcpy(want_mp, mp, sizeof want_mp);
      wm = (MapPar90 *)want_mp;
      memcpy(&wm->f628_, &cp->f14_, 4); wm->w620_ = cp->w08_; memcpy(&wm->f62c_, &cp->f18_, 4); wm->w624_ = cp->w0c_; memcpy(&wm->f630_, &cp->f1c_, 4); memcpy(&wm->f634_, &cp->f20_, 4);
      wm->flag_ = cp->flagB_ != 0;
      for (gg = 0; gg < 6; gg++) wm->idx_[gg] = cp->grp_[gg];
      for (gg = 0; gg < 6; gg++) {
        unsigned pa = 0, pb = 0;
        const uint16_t *ra = cp->arrA_ + 8 * cp->grp_[gg], *rb = (cp->flagB_ ? cp->arrB_ : cp->arrA_) + 8 * cp->grp_[gg];
        for (j = 8; j-- > 0;) for (bb = 16; bb-- > 0;) if (ra[j] & (1u << (15 - bb))) wm->listA_[128 * gg + pa++] = (uint8_t)(16 * j + bb);
        for (j = 8; j-- > 0;) for (bb = 16; bb-- > 0;) if (rb[j] & (1u << (15 - bb))) wm->listB_[128 * gg + pb++] = (uint8_t)(16 * j + bb);
        wm->cnt_[gg] = pb;
      }
      wm->w0_ = (uint32_t)((int)(int8_t)cp->rate_ + (cp->mode_ ? 0x14 : 8));
      f_from92(mp, (V92CpObj *)cp);
      canary_check(mp, sizeof(MapPar90), "setParamsInfoFromV92CPUnPck");
      COV(CV_P_FROM92);
      if (cp->flagB_) COV(CV_P_FROM92_B); else COV(CV_P_FROM92_NOB);
      image_check("setParamsInfoFromV92CPUnPck", mp, want_mp, sizeof want_mp);
      h_emit(0, "from92 hash %016llx", (unsigned long long)hobj(mp, sizeof *mp));
      DUMP90("from92b", mp);
    } else {   /* setParamsInfoFromCPUnPck */
      MapPar90 *mp = (MapPar90 *)onew_canary(sizeof(MapPar90));
      CpUnpck *u = (CpUnpck *)onew(sizeof(CpUnpck));
      uint32_t *sub = (uint32_t *)onew(16);
      MapPar90 *wm;
      uint8_t want_mp[sizeof(MapPar90)];
      unsigned gg, j, bb;
      garbage_fill(mp, sizeof *mp, h_rnd());
      garbage_fill(u, sizeof *u, h_rnd());
      sub[0] = h_rnd(); sub[1] = h_rpct(50) ? 0 : h_rnd(); sub[2] = h_rnd(); sub[3] = h_rnd();
      u->sub_ = sub;
      for (gg = 0; gg < 6; gg++) u->grp_[gg] = (uint8_t)(h_rpct(95) ? h_rmod(6) : 5);
      u->flag_ = (uint8_t)(h_rpct(50) ? 0 : (h_rpct(50) ? 1 : h_rnd()));
      u->rate_ = h_rpct(70) ? 8 + h_rmod(50) : h_rnd();
      for (j = 0; j < 48; j++) { u->arrA_[j] = (uint16_t)(h_rpct(30) ? 0 : r16()); u->arrB_[j] = (uint16_t)(h_rpct(30) ? 0 : r16()); }
      memcpy(want_mp, mp, sizeof want_mp);
      wm = (MapPar90 *)want_mp;
      wm->w620_ = u->w_[0]; wm->w624_ = u->w_[1]; memcpy(&wm->f628_, &u->w_[2], 4); memcpy(&wm->f62c_, &u->w_[3], 4); memcpy(&wm->f630_, &u->w_[4], 4); memcpy(&wm->f634_, &u->w_[5], 4);
      wm->flag_ = u->flag_ != 0;
      for (gg = 0; gg < 6; gg++) wm->idx_[gg] = u->grp_[gg];
      for (gg = 0; gg < 6; gg++) {
        unsigned pa = 0, pb = 0;
        const uint16_t *ra = u->arrA_ + 8 * u->grp_[gg], *rb = (u->flag_ ? u->arrB_ : u->arrA_) + 8 * u->grp_[gg];
        for (j = 8; j-- > 0;) for (bb = 16; bb-- > 0;) if (ra[j] & (1u << (15 - bb))) wm->listA_[128 * gg + pa++] = (uint8_t)(16 * j + bb);
        for (j = 8; j-- > 0;) for (bb = 16; bb-- > 0;) if (rb[j] & (1u << (15 - bb))) wm->listB_[128 * gg + pb++] = (uint8_t)(16 * j + bb);
        wm->cnt_[gg] = pb;
      }
      wm->w0_ = u->rate_ + (sub[1] ? 0x14u : 8u);
      if (sub[1]) COV(CV_P_SUB_NZ); else COV(CV_P_SUB_Z);
      f_fromun(mp, u);
      canary_check(mp, sizeof(MapPar90), "setParamsInfoFromCPUnPck");
      COV(CV_P_FROMUN);
      if (u->flag_) COV(CV_P_FROMUN_B); else COV(CV_P_FROMUN_NOB);
      image_check("setParamsInfoFromCPUnPck", mp, want_mp, sizeof want_mp);
      h_emit(0, "fromun hash %016llx", (unsigned long long)hobj(mp, sizeof *mp));
      DUMP90("fromun", mp);
    }
    h_trial_end("pack");
  }
}

/* ================================================================================================ scenario 5: V92setParamsInfoFromCPUnPck */
typedef struct { unsigned lv, idx, n; char text[DBGMAX][300]; } Ev;
static Ev g_ev;
static int g_ev_hook_on;
static unsigned g_hook_at, g_hook_to;
static void hook_lower(unsigned idx) { if (idx == g_hook_at) dsplibs_debug_level = g_hook_to; }
static void ev_begin(Ev *e, unsigned lvl) { e->lv = lvl; e->idx = 0; e->n = 0; }
static void ev_add(Ev *e, const char *fmt, ...) {
  char b[400];
  va_list ap;
  if (!(e->lv > 1)) return;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  if (e->n < DBGMAX) { e->text[e->n][0] = 0; esc_cat(e->text[e->n], 300, b); }
  e->n++;
  if (g_ev_hook_on && e->idx == g_hook_at) e->lv = g_hook_to;
  e->idx++;
}
/* the "%c%d.%06d" parts of a value, in double arithmetic (every operand here has at most 38 significant bits: exact) */
static void split_model(double v, int *sg, int *ip, int *fr) {
  double a = v < 0 ? -v : v;
  long long t = (long long)(v < 0 ? -(double)(long long)a : (double)(long long)a);   /* trunc(v) */
  long long fi = (long long)((v - (double)t) * 1000000.0);
  *ip = (int)(long long)a;
  *fr = (int)(fi < 0 ? -fi : fi);
  *sg = (v <= 0.0) ? 0x2d : 0x2b;
}

static void sc_v92set(void) {
  int trial, k, g;
  cov_def(CV_G_CALL, "call"); cov_def(CV_G_LVL0, "level 0 / 1 (no print)"); cov_def(CV_G_LVL2, "level 2+ (prints)"); cov_def(CV_G_HOOK, "level lowered inside a print"); cov_def(CV_G_A, "flagA"); cov_def(CV_G_B, "flagB");
  cov_def(CV_G_C, "flagC"); cov_def(CV_G_ABC, "all three"); cov_def(CV_G_CLAMP4, "array length clamped to 0x148"); cov_def(CV_G_CLAMP6, "constellation length clamped to 0x80"); cov_def(CV_G_LCZERO, "all LC zero");
  cov_def(CV_G_LCSOME, "some LC non-zero"); cov_def(CV_G_ENDLINE, "end line"); cov_def(CV_G_BIGGAIN, "gain >= 2^24 (float rounding)"); cov_def(CV_G_NEGSIGN, "sign '-' (zero gain)"); cov_def(CV_G_DRN, "negative drn");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 1500; trial++) {
    CpObj *cp;
    MapPar92 *d;
    uint8_t want[sizeof(MapPar92)], pre[sizeof(MapPar92)];
    uint32_t blocks[6];
    float *arrp[4];
    uint32_t *lcp[6];
    unsigned lvl = h_rpct(8) ? (h_rpct(50) ? 0x80000000u + h_rmod(3) : 0xffffffffu - h_rmod(2)) : (h_rpct(35) ? 0 : (h_rpct(60) ? 2 : (h_rpct(30) ? 1 : 2 + h_rmod(5))));
    unsigned nprint = 0;
    MapPar92 wd;
    h_pool_reset();
    cp = (CpObj *)onew(sizeof(CpObj));
    d = (MapPar92 *)onew_canary(sizeof(MapPar92));
    garbage_fill(cp, sizeof *cp, h_rnd());
    garbage_fill_m92(d, h_rnd());
    for (k = 0; k < 6; k++) { cp->heap_[k] = sysdep_malloc(0x200); for (g = 0; g < 128; g++) ((uint32_t *)cp->heap_[k])[g] = h_rpct(30) ? h_rmod(256) : h_rnd(); }
    for (k = 0; k < 4; k++) { arrp[k] = (float *)sysdep_malloc(0x148 * 4); memset(arrp[k], 0xa7, 0x148 * 4); d->arr_[k] = arrp[k]; }
    for (k = 0; k < 6; k++) { lcp[k] = (uint32_t *)sysdep_malloc(0x80 * 4); memset(lcp[k], 0xa9, 0x80 * 4); d->lcp_[k] = lcp[k]; }
    (void)blocks;
    cp->flagA_ = h_rpct(60) ? 1 : 0; cp->flagB_ = h_rpct(60) ? 1 : 0; cp->flagC_ = h_rpct(60) ? 1 : 0;
    if (cp->flagA_ && cp->flagB_ && cp->flagC_) COV(CV_G_ABC);
    cp->f10_ = (uint8_t)(h_rpct(70) ? h_rmod(32) : h_rnd()); cp->f11_ = (uint8_t)(h_rpct(70) ? h_rmod(4) : h_rnd()); cp->f12_ = (uint8_t)(h_rpct(70) ? h_rmod(2) : h_rnd());
    cp->w14_ = h_rpct(25) ? 0 : (h_rpct(30) ? h_rnd() : (h_rpct(50) ? 1u << (h_rmod(32)) : h_rmod(1u << 20)));
    if (cp->w14_ >= 0x1000000u) COV(CV_G_BIGGAIN);
    if (cp->w14_ == 0) COV(CV_G_NEGSIGN);
    if (cp->f10_ & 0x80) COV(CV_G_DRN);
    for (k = 0; k < 12; k++) cp->pair_[k] = h_rnd();
    for (k = 0; k < 4; k++) cp->cnt_[k] = h_rpct(25) ? 0 : (h_rpct(25) ? 0x147 + h_rmod(4) : (h_rpct(15) ? 0x148 + h_rmod(100) : h_rmod(0x148)));
    for (k = 0; k < 6; k++) cp->hcnt_[k] = h_rpct(25) ? 0 : (h_rpct(20) ? 0x7f + h_rmod(3) : (h_rpct(10) ? 0x80 + h_rmod(100) : h_rmod(0x81)));
    if (h_rpct(10)) for (k = 0; k < 6; k++) cp->hcnt_[k] = 0;
    if (h_rpct(15)) { int one = h_rmod(6); for (k = 0; k < 6; k++) cp->hcnt_[k] = (k == one) ? 1 + h_rmod(3) : 0; }
    for (k = 0; k < 6; k++) cp->nib_[k] = h_rnd();
    for (k = 0; k < 1536; k++) cp->arrs_[k] = (uint16_t)h_rnd();
    dsplibs_debug_level = lvl;
    g_dbg_in_call = 0;
    g_dbg_hook = NULL;
    if (lvl > 1 && h_rpct(25)) { g_dbg_hook = hook_lower; g_hook_at = h_rpct(50) ? h_rmod(14) : h_rmod(70); g_hook_to = h_rmod(2); COV(CV_G_HOOK); }
    g_ev_hook_on = (g_dbg_hook == hook_lower);
    g_snap_obj = d; g_snap_f = g_fm92; g_snap_nf = g_nfm92;
    memcpy(pre, d, sizeof pre);
    if (((uint8_t *)d)[sizeof(MapPar92)] != 0xc3) h_die3("canary clobbered BEFORE the call by the harness: 0x%02x", ((uint8_t *)d)[sizeof(MapPar92)]);
    f_v92set(d, cp);
    g_snap_obj = NULL;
    g_dbg_hook = NULL;
    COV(CV_G_CALL);
    if (lvl > 1) COV(CV_G_LVL2); else COV(CV_G_LVL0);
    /* the model: the destination fields */
    memcpy(&wd, pre, sizeof wd);
    wd.a_ = cp->flagA_; wd.b_ = cp->flagB_; wd.c_ = cp->flagC_;
    wd.w0_ = 0x22 + 2 * (int)(int8_t)cp->f10_;
    wd.trellis_ = (int8_t)cp->f11_;
    wd.extendEu_ = (int8_t)cp->f12_;
    {
      volatile float xr = (float)cp->w14_;
      float x = xr * (1.0f / 262144.0f);
      double t = (double)x * 4000.0;
      volatile float gr = (float)t;
      wd.gain_ = gr;
      /* the text model for the printed events (level steering): reproduce the sequence of print events and cut it where the level drops */
      {
        Ev *e = &g_ev;
        ev_begin(e, lvl);
        ev_add(e, "V92 mapping parameters:\r\n");
        do {
          if (!(e->lv > 1)) break;
          ev_add(e, "CPObj->modulosEncoderPresent = %d\r\n", cp->flagA_);
          if (!(e->lv > 1)) break;
          ev_add(e, "CPObj->prefilterPrecoderPresent = %d\r\n", cp->flagB_);
          if (!(e->lv > 1)) break;
          ev_add(e, "CPObj->constellationPresent = %d\r\n", cp->flagC_);
          if (!(e->lv > 1)) break;
          ev_add(e, "CPObj->trellisState = %d\r\n", (int)(int8_t)cp->f11_);
          if (!(e->lv > 1)) break;
          ev_add(e, "CPObj->prefilterGain = %d\r\n", cp->w14_);
          if (!(e->lv > 1)) break;
          ev_add(e, "CPObj->extendEu = %d\r\n", (int)(int8_t)cp->f12_);
          if (!(e->lv > 1)) break;
          ev_add(e, "CPObj->drn = %d\r\n", (int)(int8_t)cp->f10_);
          if (!(e->lv > 1)) break;
          ev_add(e, "Upstream rate = %d\r\n", (int)(((int)(int8_t)cp->f10_ + 0x11) * (double)1333.3333740234375f));
        } while (0);
        { int sg, ip, fr; split_model((double)x, &sg, &ip, &fr); ev_add(e, "V92Modulator: constellation gain (before Lu multiplication) = %c%d.%06d\r\n", sg, ip, fr); }
        { int sg, ip, fr; split_model(t, &sg, &ip, &fr); ev_add(e, "V92Modulator: constellation gain (after Lu multiplication) = %c%d.%06d\r\n", sg, ip, fr); }
        if (wd.a_) { for (k = 0; k < 12; k++) ev_add(e, "CPObj->M[%d] = %d\r\n", k, cp->pair_[k]); for (k = 0; k < 12; k++) wd.m_[k] = cp->pair_[k]; COV(CV_G_A); }
        if (wd.b_) {
          COV(CV_G_B);
          do {
            if (!(e->lv > 1)) break;
            ev_add(e, "CPObj->lz1 = %d\r\n", cp->cnt_[0]);
            if (!(e->lv > 1)) break;
            ev_add(e, "CPObj->lz2 = %d\r\n", cp->cnt_[2]);
            if (!(e->lv > 1)) break;
            ev_add(e, "CPObj->lp1 = %d\r\n", cp->cnt_[1]);
            if (!(e->lv > 1)) break;
            ev_add(e, "CPObj->lp2 = %d\r\n", cp->cnt_[3]);
          } while (0);
          for (k = 0; k < 4; k++) { wd.cnt_[k] = cp->cnt_[k]; if (wd.cnt_[k] > 0x148) { wd.cnt_[k] = 0x148; COV(CV_G_CLAMP4); } }
        }
        if (wd.c_) {
          COV(CV_G_C);
          for (k = 0; k < 6; k++) ev_add(e, "CPObj->LC[%d] = %d\r\n", k, cp->hcnt_[k]);
          for (k = 0; k < 6; k++) ev_add(e, "CPObj->indexConstel[%d] = %d\r\n", k, cp->nib_[k]);
          for (k = 0; k < 6; k++) { wd.lc_[k] = cp->hcnt_[k]; if (wd.lc_[k] > 0x80) { wd.lc_[k] = 0x80; COV(CV_G_CLAMP6); } }
          {
            unsigned any = 0;
            for (k = 0; k < 6; k++) if (wd.lc_[k]) {
              unsigned i;
              any = 1;
              ev_add(e, "======== Constellation LC %d =============\r\n", k + 1);
              for (i = 0; i < wd.lc_[k]; i++) ev_add(e, "\tconst%d[%d] = %d\r\n", k + 1, i, ((uint32_t *)cp->heap_[k])[i]);
            }
            if (any) { COV(CV_G_LCSOME); ev_add(e, "==================================\r\n"); COV(CV_G_ENDLINE); } else COV(CV_G_LCZERO);
          }
          for (k = 0; k < 6; k++) wd.idxc_[k] = cp->nib_[k];
        }
      }
    }
    if (g_dbg_in_call != g_ev.n) h_die3("V92setParamsInfoFromCPUnPck: %u prints, model %u (level %u)", g_dbg_in_call, g_ev.n, lvl);
    for (k = 0; k < (int)g_ev.n && k < DBGMAX; k++) if (strcmp(g_dbg_text[k], g_ev.text[k])) h_die3("print %d: '%s' model '%s'", k, g_dbg_text[k], g_ev.text[k]);
    memcpy(want, &wd, sizeof want);
    /* destination fields (the pointer members are compared by name through the dump; the model image carries the harness pointers) */
    #define CMPF(f) if (memcmp(&d->f, &((MapPar92 *)want)->f, sizeof d->f)) h_die3("V92MappingParams." #f " differs from the model")
    CMPF(w0_); CMPF(a_); CMPF(b_); CMPF(c_); CMPF(trellis_); CMPF(extendEu_); CMPF(gain_); CMPF(m_); CMPF(cnt_); CMPF(lc_); CMPF(idxc_);
    #undef CMPF
    { unsigned c3; for (c3 = 0; c3 < 32; c3++) if (((uint8_t *)d)[sizeof(MapPar92) + c3] != 0xc3) h_die3("V92setParamsInfoFromCPUnPck wrote %u bytes past the 0xb4-byte block (byte 0x%02x, lvl %u a %u b %u c %u)", c3 + 1, ((uint8_t *)d)[sizeof(MapPar92) + c3], lvl, wd.a_, wd.b_, wd.c_); }
    /* the arrays */
    if (wd.b_) {
      static const float K[4] = {1.0f / 32768.0f, 1.0f / 16384.0f, 1.0f / 32768.0f, 1.0f / 16384.0f};
      for (k = 0; k < 4; k++) {
        unsigned i;
        for (i = 0; i < wd.cnt_[k]; i++) { float e = (float)(int16_t)cp->arrs_[384 * k + i] * K[k]; if (memcmp(&arrp[k][i], &e, 4)) h_die3("array %d element %u differs", k, i); }
        { unsigned i2; for (i2 = wd.cnt_[k]; i2 < 0x148 && i2 < wd.cnt_[k] + 2; i2++) { uint32_t w; memcpy(&w, &arrp[k][i2], 4); if (w != 0xa7a7a7a7u) h_die3("array %d written past its length", k); } }
      }
    }
    if (wd.c_) {
      for (k = 0; k < 6; k++) {
        unsigned i;
        for (i = 0; i < wd.lc_[k]; i++) if (lcp[k][i] != ((uint32_t *)cp->heap_[k])[i]) h_die3("LC array %d element %u differs", k, i);
        if (wd.lc_[k] < 0x80 && lcp[k][wd.lc_[k]] != 0xa9a9a9a9u) h_die3("LC array %d written past its length", k);
      }
    }
    h_emit(0, "v92set lvl %u prints %u hash %016llx", lvl, g_dbg_in_call, (unsigned long long)hobj(d->m_, sizeof d->m_));
    h_emit(0, "v92set arr %016llx %016llx %016llx %016llx", (unsigned long long)hobj(arrp[0], 0x148 * 4), (unsigned long long)hobj(arrp[1], 0x148 * 4), (unsigned long long)hobj(arrp[2], 0x148 * 4), (unsigned long long)hobj(arrp[3], 0x148 * 4));
    h_emit(0, "v92set lcp %016llx %016llx %016llx", (unsigned long long)hobj(lcp[0], 0x200), (unsigned long long)hobj(lcp[3], 0x200), (unsigned long long)hobj(lcp[5], 0x200));
    DUMPM92("v92set", d);
    (void)nprint;
    for (k = 0; k < 4; k++) sysdep_free(arrp[k]);
    for (k = 0; k < 6; k++) { sysdep_free(lcp[k]); sysdep_free(cp->heap_[k]); }
    h_trial_end("v92set");
  }
}

/* ================================================================================================ scenario 6: small functions */
static void sc_small(void) {
  int trial, k;
  cov_def(CV_C_C1, "C1"); cov_def(CV_C_C2, "C2"); cov_def(CV_C_D1, "D1"); cov_def(CV_C_D2, "D2"); cov_def(CV_C_RESET, "reset"); cov_def(CV_C_RESETDET, "resetDetector"); cov_def(CV_C_SETSUV, "setSUV");
  cov_def(CV_C_GETBV, "getBitVector"); cov_def(CV_C_RESETCRC, "resetCRC"); cov_def(CV_C_CALCCRC, "calcCRC"); cov_def(CV_C_EVAL_OK, "evaluateCRC == 1"); cov_def(CV_C_EVAL_BAD, "evaluateCRC == 0");
  cov_def(CV_C_SUM256, "evaluateCRC accepted by a total of 256");
  h_scenario_begin(0);
  relax_reserved();
  h_emit(1, "tables binaryTable %016llx fltTable_1 %016llx fltTable_2 %016llx", (unsigned long long)hobj(binaryTable, sizeof binaryTable), (unsigned long long)hobj(fltTable_1, sizeof fltTable_1), (unsigned long long)hobj(fltTable_2, sizeof fltTable_2));
  for (trial = 0; trial < 1200; trial++) {
    int which = trial & 1;
    void *o = cp_new(which, 1);
    V92CpObj *co = CO(o);
    uint8_t want[T92], pre[T92];
    uint8_t crcm[16];
    unsigned r;
    I92 in;
    M92 m;
    COV(which ? CV_C_C2 : CV_C_C1);
    /* constructor: exactly these fields (the rest keeps the garbage) -- compare to an image built from the garbage the object had */
    h_emit(0, "ctor %d widx %u count %u state %u ones %u zeros %u flag %u supp %d", which, co->widx_, co->count_, co->state_, co->ones_, co->zeros_, co->flag_, co->supp_);
    EXPECT_EQ("widx", co->widx_, 0x12); EXPECT_EQ("count", co->count_, 0); EXPECT_EQ("state", co->state_, 0); EXPECT_EQ("ones", co->ones_, 0); EXPECT_EQ("zeros", co->zeros_, 0); EXPECT_EQ("flag", co->flag_, 0);
    EXPECT_EQ("supp", (uint32_t)co->supp_, 0xffffffffu);
    DUMP92("ctor", o);
    /* reset / resetDetector on a dirty object */
    garbage_fill(o, T92, h_rnd());
    memcpy(pre, o, T92); memcpy(want, o, T92);
    {
      V92CpObj *w = (V92CpObj *)want;
      w->widx_ = 0x12; w->count_ = 0; w->state_ = 0; w->ones_ = 0; w->zeros_ = 0;
      if (h_rpct(50)) { cp_reset(o); w->supp_ = -1; COV(CV_C_RESET); }
      else { cp_resetdet(o); COV(CV_C_RESETDET); }
      image_check("reset / resetDetector", o, want, T92);
    }
    /* setSUV */
    { uint32_t v = h_rnd(); memcpy(pre, o, T92); cp_setsuv(o, v); memcpy(want, pre, T92); ((V92CpObj *)want)->suv_ = 0x10; ((V92CpObj *)want)->suvVal_ = v; image_check("setSUV", o, want, T92); COV(CV_C_SETSUV); }
    /* getBitVector */
    { unsigned len = 0xdeadbeefu; uint8_t *p = cp_getbv(o, &len); COV(CV_C_GETBV); EXPECT_EQ("getBitVector ptr", (uint32_t)(p - (uint8_t *)o), 0x129); EXPECT_EQ("getBitVector length", len, co->totalLen_); }
    /* CRC functions against the CRC model on a model frame */
    rnd_i92(&in, 1900);
    model_encode(&in, NULL, &m);
    for (k = 0; k < (int)NB; k++) co->bits_[k] = (uint8_t)h_rmod(2);
    for (k = 0; k < (int)m.total; k++) co->bits_[k] = m.exp[k];
    co->frameLen_ = m.frameLen;
    for (k = 0; k < 16; k++) co->crc_[k] = (uint8_t)h_rnd();
    { unsigned kind = h_rmod(6); if (kind == 1) { unsigned p = 0x13 + h_rmod(m.C - 0x13); if (p % 17) co->bits_[p] ^= 1; } if (kind == 2) { co->frameLen_ = 0x22 + h_rmod(5); } if (kind == 3) co->frameLen_ = 17 + h_rmod(0x20); if (kind == 4) co->frameLen_ = 0x22 + h_rmod(1500); }
    cp_resetcrc(o); COV(CV_C_RESETCRC);
    for (k = 0; k < 16; k++) if (co->crc_[k] != 1) h_die3("resetCRC left crc[%d] = %d", k, co->crc_[k]);
    cp_calccrc(o); COV(CV_C_CALCCRC);
    model_crc(co->bits_, co->frameLen_ - 0x11, crcm);
    for (k = 0; k < 16; k++) if (co->crc_[k] != crcm[k]) h_die3("calcCRC: stage %d = %d, model %d (frameLen %u)", k, co->crc_[k], crcm[k], co->frameLen_);
    r = (unsigned)cp_evalcrc(o);
    if (r) COV(CV_C_EVAL_OK); else COV(CV_C_EVAL_BAD);
    h_emit(0, "evalCRC r=%u crc %016llx", r, (unsigned long long)fnv_upd(FNV0, co->crc_, 16));
    DUMP92("eval", o);
    if (co->frameLen_ == m.frameLen && r != 1) { /* clean frame (kind 0 / 5, or damaged, in which case r may be 0) */ }
    {   /* the 8-bit accumulator: every received CRC bit differs from the register by 16 -> total 256 -> accepted */
      uint8_t crc2[16];
      unsigned i;
      co->frameLen_ = m.frameLen;
      model_crc(co->bits_, m.C, crc2);
      for (i = 0; i < 16; i++) co->bits_[m.frameLen - 16 + i] = (uint8_t)(crc2[i] + 16);
      r = (unsigned)cp_evalcrc(o);
      if (!r) h_die3("evaluateCRC: a total difference of 256 must be accepted (8-bit sum)");
      COV(CV_C_SUM256);
      for (i = 0; i < 16; i++) co->bits_[m.frameLen - 16 + i] = (uint8_t)crc2[i];
      r = (unsigned)cp_evalcrc(o);
      if (m.frameLen == co->frameLen_ && !r) h_die3("evaluateCRC rejected the model frame");
    }
    {
      unsigned long f0 = h_nfree;
      cp_free(o, which);
      COV(which ? CV_C_D2 : CV_C_D1);
      if (h_nfree != f0) h_die3("destructor freed memory");
    }
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P11_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P11_DBG")) g_dbg_show = 1;
  p11_layout_check();
  h_x87_check();
  h_arena_init();
  g_nf92 = build_tab(g_f92, f_V92CpObj, NF_V92CpObj);
  g_nf90 = build_tab(g_f90, f_MapPar90, NF_MapPar90);
  g_nfm92 = build_tab(g_fm92, f_MapPar92, NF_MapPar92);
  switch (sc) {
    case 0: sc_info2bits(); break;
    case 1: sc_evalinfo(); break;
    case 2: sc_rx(); break;
    case 3: sc_rxfuzz(); break;
    case 4: sc_pack(); break;
    case 5: sc_v92set(); break;
    case 6: sc_small(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu dbgprintf %u", h_nmalloc, h_nfree, g_ndbg);
  h_cov_finish();
  return 0;
}
