/*
 * Differential test harness for port chunk P15 (src/vpcm/v92_modulus_encoder.cpp): class V92ModulusEncoder (C1, C2, reset, progress; 4 functions, 64-bit divide arithmetic).
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/v92_modulus_encoder.sh against the blob (dsplibs.o), against the port (-O0 / -O2; the 32-bit port links the SAME
 * libgcc helpers __divdi3 / __moddi3 as the blob, the x86-64 port uses native 64-bit division) and for x86-64 (diffed against the 32-bit BLOB's output = the golden,
 * test/golden/vpcm/v92_modulus_encoder.blob32.out).  The functions are reached through __asm__ labels with the mangled names; the object is the C mirror generated from test/vpcm/v92_modulus_encoder.layout (no pointers,
 * 64-bit fields as word pairs), reset() reads the V92MappingParams mirror of test/vpcm/v92_cp.layout (member-wise filled: its pointer members move on x86-64).
 *
 * What is compared: every word of the object after every call (also the words a call must not touch), the 12 digits written and a guard word behind them, the digits of the modes 1 / 2.
 * INDEPENDENT MODELS (unsigned __int128 arithmetic, written from the definitions, not from the port): the radix products and the (a_, b_) shift search of reset(); the bit
 * accumulation and the flag / threshold logic of progress(); the digit chain of the SIMPLE path (the number fits 63 bits and is non-negative: plain mixed-radix division, which is what the
 * vendor's code computes there); for the large-number path (high part > 0: the vendor reduces hi * 2^63 with the CONSTANT -2^63) blob and port are compared against each other and
 * against the exact digits of hi * 2^63 + lo, the statistics of which are emitted (the vendor result is not always the exact one).
 *
 * Inputs are boundary-dense: radices 1, 2, 3, 2^k +- 1, 2^31 - 1, 2^31, 2^32 - 1, random; bit counts 0, 1, 62 .. 66, 126 .. 130, random; flags 0 / 1 / garbage; (a_, b_) from reset() or
 * 64-bit boundary values (0, 1, 2^31, 2^32, 2^62, 2^63 - 1, 2^63, 2^64 - 1).  A zero radix traps in the blob (a division by zero in __divdi3): excluded.
 *
 * Scenarios (each its own process): 0 constructors / modes 1, 2 / unknown modes, 1 reset vs the model, 2 progress mode 0 (reset-made and boundary (a_, b_) states), 3 progress mode 0 on
 * the large-number path (high part > 0, steered), 4 chains of calls (the flag carries over between calls).
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "v92_cp_layout.h"
#include "v92_modulus_encoder_layout.h"

#define L(m) __asm__(m)

extern void me_c1(void *) L("_ZN17V92ModulusEncoderC1Ev");
extern void me_c2(void *) L("_ZN17V92ModulusEncoderC2Ev");
extern void me_reset(void *, void *) L("_ZN17V92ModulusEncoder5resetEP16V92MappingParams");
extern void me_prog(void *, unsigned char *, unsigned *) L("_ZN17V92ModulusEncoder8progressEPhPj");


static int g_show;
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
static HFld g_fe[16];
static int g_ne;
static unsigned long g_dumpctr;
#define DUMPE(tag, o) h_dump_obj(h_sample(&g_dumpctr, 12, 97), (tag), (o), g_fe, g_ne)

static uint64_t get64(const uint32_t w[2]) { return (uint64_t)w[0] | ((uint64_t)w[1] << 32); }
static void put64(uint32_t w[2], uint64_t v) { w[0] = (uint32_t)v; w[1] = (uint32_t)(v >> 32); }

/* ------------------------------------------------------------------------------------------------ input generators */
static uint32_t rrad(void) {
  static const uint32_t fx[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 100, 127, 128, 129, 255, 256, 257, 1000, 4095, 4096, 4097, 65535, 65536, 65537,
                                0x7fffffffu, 0x80000000u, 0x80000001u, 0xfffffffeu, 0xffffffffu, 0x10000u, 0x1000000u, 0xffffffu, 0x3fffffffu, 0x40000000u};
  switch (h_rmod(6)) {
    case 0: case 1: return fx[h_rmod(sizeof fx / sizeof fx[0])];
    case 2: return 1 + h_rmod(20);
    case 3: return 1 + h_rmod(1000);
    case 4: return h_rnd() | 1u;
    default: return 1 + (h_rnd() >> h_rmod(32));
  }
}
static void rnd_radices(uint32_t m[12], int allow_zero) {
  unsigned regime = h_rmod(8);
  int k;
  for (k = 0; k < 12; k++) {
    switch (regime) {
      case 0: m[k] = 1 + h_rmod(8); break;                 /* small: the product never overflows */
      case 1: m[k] = 1 + h_rmod(256); break;               /* medium: 12 * 8 bits = 96 bits: overflows */
      case 2: m[k] = rrad(); break;
      case 3: m[k] = 1u << h_rmod(32); break;              /* powers of two */
      case 4: m[k] = h_rpct(50) ? 1 + h_rmod(4) : (h_rnd() | 0x80000000u); break;
      case 6: m[k] = h_rpct(85) ? 1 : 1 + h_rmod(3); break;                  /* products 1, 2, 3, 4 ...: the edges of the P > 0 / w > 0 tests */
      case 7: m[k] = h_rpct(60) ? 1 : (1u << h_rmod(6)); break;             /* ones and small powers of two: Q1 = 1, Q2 = 2^j */
      default: m[k] = h_rpct(30) ? 1 : rrad(); break;
    }
    if (allow_zero && h_rpct(2)) m[k] = 0;
  }
}
static uint64_t rb64(void) {
  static const uint64_t fx[] = {0, 1, 2, 3, 0x7fffffffull, 0x80000000ull, 0xffffffffull, 0x100000000ull, 0x100000001ull, 0x3fffffffffffffffull, 0x4000000000000000ull, 0x7ffffffffffffffeull,
                                0x7fffffffffffffffull, 0x8000000000000000ull, 0x8000000000000001ull, 0xc000000000000000ull, 0xfffffffffffffffeull, 0xffffffffffffffffull};
  switch (h_rmod(4)) {
    case 0: return fx[h_rmod(sizeof fx / sizeof fx[0])];
    case 1: return ((uint64_t)h_rnd() << 32) | h_rnd();
    case 2: return (((uint64_t)h_rnd() << 32) | h_rnd()) >> h_rmod(64);
    default: return fx[h_rmod(sizeof fx / sizeof fx[0])] + (uint64_t)(int)(h_rmod(5)) - 2;
  }
}

/* ------------------------------------------------------------------------------------------------ models (plain 64-bit / 4 x 32-bit limb arithmetic: no __int128 on i386) */
/* the radix products and the shift search of reset() (all products wrap mod 2^64) */
static void model_reset(const uint32_t m[12], uint64_t *prod, uint64_t *a, uint64_t *b, int *kout) {
  uint64_t q1 = 1, q2 = 1, p;
  int i, k;
  for (i = 0; i < 6; i++) q1 *= m[i];
  for (i = 6; i < 12; i++) q2 *= m[i];
  p = q1 * q2;
  *prod = p;
  *kout = 0;
  if ((int64_t)p > 0) {
    int found = 0;
    for (k = 1; k <= 8 && !found; k++) {
      int64_t t = (int64_t)q2 >> k;
      uint64_t w = q1 * (uint64_t)t;
      if (k == 8 || (int64_t)w <= 0) found = k;
    }
    *kout = found;
    if (found == 8) { *a = 0; *b = p; return; }
    k = found;
  } else k = 0;
  {
    int64_t t = (int64_t)q2 >> (k + 1);
    uint64_t w = q1 * (uint64_t)t;
    *a = (uint64_t)((int64_t)w >> (62 - k));
    *b = (w << (k + 1)) & 0x7fffffffffffffffull;
  }
}
/* does the exact product of the twelve radices exceed 2^64 - 1? */
static int product_wraps(const uint32_t m[12]) {
  uint64_t acc = 1;
  int i;
  for (i = 0; i < 12; i++) {
    if (m[i] == 0) return 0;
    if (acc > 0xffffffffffffffffull / m[i]) return 1;
    acc *= m[i];
  }
  return 0;
}

typedef struct { uint64_t hi, lo; } Num;   /* hi * 2^63 + lo */
static Num model_accum(const unsigned char *bits, uint32_t nbits) {
  Num n;
  int e;
  n.hi = 0; n.lo = 0;
  if (nbits < 0x40) { for (e = (int)nbits - 1; e >= 0; e--) n.lo = (n.lo << 1) | (bits[e] & 1u); }
  else {
    for (e = (int)nbits - 1; e >= 0x3f; e--) n.hi = (n.hi << 1) | (bits[e] & 1u);
    for (e = 0x3e; e >= 0; e--) n.lo = (n.lo << 1) | (bits[e] & 1u);
  }
  return n;
}

/* the threshold test, flag logic and the normalised (Xp, Yp) of progress mode 0; returns the new flag */
static uint32_t model_prep(const V92ModEncObj *o, Num n, int64_t *Xp, int64_t *Yp) {
  int64_t a = (int64_t)get64(o->a_), X = (int64_t)n.hi, Y = (int64_t)n.lo;
  int64_t a2 = (a + (a < 0 ? 1 : 0)) >> 1;   /* a / 2, truncating */
  uint64_t bm1 = get64(o->b_) - 1;
  int64_t rem = a - 2 * a2;                  /* a % 2 in {-1, 0, 1} */
  int64_t bh = ((int64_t)bm1 + ((int64_t)bm1 < 0 ? 1 : 0)) >> 1;
  uint64_t R = (uint64_t)bh + ((uint64_t)rem << 62);
  uint32_t t = !(X < a2 || (X == a2 && Y < (int64_t)R));
  *Xp = X; *Yp = Y;
  if (o->flag_) {
    *Xp = a ? (int64_t)((uint64_t)a - (uint64_t)X) : 0;
    *Yp = (int64_t)(get64(o->b_) - (uint64_t)Y - 1);
  }
  if (*Yp < 0 && *Xp > 0) { *Yp = (int64_t)((uint64_t)*Yp + 0x8000000000000000ull); *Xp = (int64_t)((uint64_t)*Xp - 1); }
  return o->flag_ ^ t;
}

/* the simple path: Xp <= 0: plain mixed-radix division of Yp (signed truncating division, the digit zero-extended before the subtraction) */
static void model_simple(int64_t Yp, const uint32_t m[12], uint32_t d[12]) {
  int64_t q = Yp;
  int k;
  for (k = 0; k <= 10; k++) {
    uint32_t dk = (uint32_t)(q % (int64_t)m[k]);
    d[k] = dk;
    q = (int64_t)((uint64_t)q - dk) / (int64_t)m[k];
  }
  d[11] = (uint32_t)q;
}
/* the exact digits of the unsigned 128-bit number hi * 2^63 + lo (hi, lo >= 0): d_k = n mod m_k, n = n / m_k, and the final quotient (which is the vendor's d_11 = (n - d_10) / m_10) */
static void ideal_digits(int64_t hi, int64_t lo, const uint32_t m[12], uint32_t d[12]) {
  uint32_t n[4], c;
  uint64_t h = (uint64_t)hi, l = (uint64_t)lo;
  int k, i;
  /* n = h * 2^63 + l as four 32-bit limbs (little endian) */
  { uint64_t lo64 = (h << 63) + l; uint64_t hi64 = h >> 1; if (lo64 < l) hi64++;   /* h << 63 keeps bit 0 of h at bit 63; the carry of the addition goes to the high word */
    n[0] = (uint32_t)lo64; n[1] = (uint32_t)(lo64 >> 32); n[2] = (uint32_t)hi64; n[3] = (uint32_t)(hi64 >> 32); }
  for (k = 0; k <= 10; k++) {
    uint64_t rem = 0;
    for (i = 3; i >= 0; i--) { uint64_t cur = (rem << 32) | n[i]; n[i] = (uint32_t)(cur / m[k]); rem = cur % m[k]; }
    d[k] = (uint32_t)rem;
  }
  c = n[0];
  d[11] = c;
}

enum { CV_C_C1 = 0, CV_C_C2, CV_C_MODE1, CV_C_MODE2, CV_C_MODEX, CV_C_KEEP,
       CV_R_CALL, CV_R_K0, CV_R_KMID, CV_R_K8, CV_R_PNEG, CV_R_PZERO, CV_R_OVF, CV_R_SMALL, CV_R_W0, CV_R_ZERORAD,
       CV_P_CALL, CV_P_SIMPLE, CV_P_BIG, CV_P_FLAG, CV_P_NOFLAG, CV_P_T0, CV_P_T1, CV_P_NORM, CV_P_N63, CV_P_NLT64, CV_P_NGE64, CV_P_NEGY, CV_P_CASEB, CV_P_CASEC, CV_P_H0, CV_P_S1,
       CV_P_IDEAL_EQ, CV_P_IDEAL_NE, CV_P_SIMPLE_MODEL, CV_P_ASET, CV_P_ZERO_A, CV_P_CHAIN, CV_P_RESETMADE, CV_P_BOUNDARY, CV_P_ODD_A, CV_P_NEG_A, CV_P_COMPL_ZERO_A };

/* ================================================================================================ scenario 0 */
static void sc_ctor(void) {
  int trial, k;
  cov_def(CV_C_C1, "ctor C1"); cov_def(CV_C_C2, "ctor C2"); cov_def(CV_C_MODE1, "mode 1"); cov_def(CV_C_MODE2, "mode 2"); cov_def(CV_C_MODEX, "mode not 0 / 1 / 2"); cov_def(CV_C_KEEP, "words kept");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 600; trial++) {
    V92ModEncObj *o, pre;
    unsigned dig[16];
    uint32_t mode;
    h_pool_reset();
    o = (V92ModEncObj *)onew_canary(sizeof *o);
    garbage_fill(o, sizeof *o, h_rnd());
    memcpy(&pre, o, sizeof pre);
    if (trial % 2) { me_c1(o); COV(CV_C_C1); } else { me_c2(o); COV(CV_C_C2); }
    /* words 0x18 .. 0x48 are zero, the others are as before */
    for (k = 0; k < 12; k++) EXPECT_EQ("ctor radix word", o->m_[k], 0);
    EXPECT_EQ("ctor nbits", o->nbits_, 0);
    if (memcmp(o->prod_, pre.prod_, 24) || o->flag_ != pre.flag_ || o->mode_ != pre.mode_) h_die3("ctor changed a word it must not touch");
    COV(CV_C_KEEP);
    /* modes 1 / 2 / others on a random radix set */
    for (k = 0; k < 12; k++) o->m_[k] = h_rpct(20) ? (uint32_t)h_rnd() : rrad();
    o->nbits_ = h_rnd();
    mode = h_rpct(35) ? 1 : h_rpct(50) ? 2 : (h_rpct(50) ? (uint32_t)(3 + h_rmod(5)) : (h_rpct(50) ? 0x80000000u + h_rmod(3) : 0xffffffffu - h_rmod(3)));
    if (mode == 1 || mode == 2 || h_rpct(30)) { /* keep */ }
    o->mode_ = mode;
    for (k = 0; k < 16; k++) dig[k] = 0xd1900000u + (unsigned)k;
    memcpy(&pre, o, sizeof pre);
    me_prog(o, (unsigned char *)dig, dig);
    if (mode == 1) {
      static const int nz[12] = {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0};
      for (k = 0; k < 12; k++) EXPECT_EQ("mode 1 digit", dig[k], nz[k] ? o->m_[k] - 1u : 0u);
      COV(CV_C_MODE1);
    } else if (mode == 2) {
      static const int nz[12] = {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1};
      for (k = 0; k < 12; k++) EXPECT_EQ("mode 2 digit", dig[k], nz[k] ? o->m_[k] - 1u : 0u);
      COV(CV_C_MODE2);
    } else {
      for (k = 0; k < 12; k++) EXPECT_EQ("unknown mode: digits untouched", dig[k], 0xd1900000u + (unsigned)k);
      COV(CV_C_MODEX);
    }
    for (k = 12; k < 16; k++) EXPECT_EQ("digit guard", dig[k], 0xd1900000u + (unsigned)k);
    if (memcmp(&pre, o, sizeof pre)) h_die3("progress (mode %u) changed the object", mode);
    h_emit(0, "ctor mode %u dig %08x %08x %08x %08x", mode, dig[0], dig[1], dig[2], dig[11]);
    DUMPE("enc", o);
    canary_check(o, sizeof *o, "V92ModulusEncoder");
    h_trial_end("ctor");
  }
}

/* ================================================================================================ scenario 1: reset */
static void sc_reset(void) {
  int trial, k;
  cov_def(CV_R_CALL, "reset"); cov_def(CV_R_K0, "k == 0 (P <= 0)"); cov_def(CV_R_KMID, "k in 1 .. 7"); cov_def(CV_R_K8, "k == 8 (a = 0, b = P)"); cov_def(CV_R_PNEG, "P negative");
  cov_def(CV_R_PZERO, "P zero"); cov_def(CV_R_OVF, "product wrapped"); cov_def(CV_R_SMALL, "small product (no wrap)"); cov_def(CV_R_W0, "negative / zero number of bits"); cov_def(CV_R_ZERORAD, "a zero radix (reset only)");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 6000; trial++) {
    V92ModEncObj *o, pre;
    MapPar92 *p;
    uint32_t m[12];
    uint64_t prod, a, b;
    int kk;
    h_pool_reset();
    o = (V92ModEncObj *)onew_canary(sizeof *o);
    p = (MapPar92 *)onew(sizeof *p);
    garbage_fill(o, sizeof *o, h_rnd());
    rnd_radices(m, 1);
    p->w0_ = (int32_t)(h_rpct(60) ? h_rmod(130) : h_rnd()); p->a_ = h_rnd(); p->b_ = h_rnd(); p->c_ = h_rnd(); p->trellis_ = (int32_t)h_rnd(); p->extendEu_ = (int32_t)h_rnd();
    { uint32_t gb = h_rnd(); memcpy(&p->gain_, &gb, 4); }
    for (k = 0; k < 12; k++) p->m_[k] = m[k];
    for (k = 0; k < 4; k++) p->cnt_[k] = h_rnd();
    for (k = 0; k < 6; k++) { p->lc_[k] = h_rnd(); p->idxc_[k] = h_rnd(); }
    memcpy(&pre, o, sizeof pre);
    me_reset(o, p);
    COV(CV_R_CALL);
    model_reset(m, &prod, &a, &b, &kk);
    EXPECT_EQ("reset prod", get64(o->prod_), prod);
    EXPECT_EQ("reset a", get64(o->a_), a);
    EXPECT_EQ("reset b", get64(o->b_), b);
    EXPECT_EQ("reset nbits", o->nbits_, (uint32_t)p->w0_);
    for (k = 0; k < 12; k++) EXPECT_EQ("reset radix", o->m_[k], m[k]);
    EXPECT_EQ("reset flag", o->flag_, 0);
    EXPECT_EQ("reset mode", o->mode_, 0);
    if (kk == 0) COV(CV_R_K0); else if (kk == 8) COV(CV_R_K8); else COV(CV_R_KMID);
    if ((int64_t)prod < 0) COV(CV_R_PNEG);
    if (prod == 0) COV(CV_R_PZERO);
    if (product_wraps(m)) COV(CV_R_OVF); else COV(CV_R_SMALL);
    if (p->w0_ <= 0) COV(CV_R_W0);
    for (k = 0; k < 12; k++) if (!m[k]) { COV(CV_R_ZERORAD); break; }
    h_emit(0, "reset k %d prod %016llx a %016llx b %016llx", kk, (unsigned long long)prod, (unsigned long long)a, (unsigned long long)b);
    DUMPE("reset", o);
    canary_check(o, sizeof *o, "reset");
    h_trial_end("reset");
  }
}

/* ================================================================================================ scenarios 2 / 3 / 4: progress mode 0 */
static void fill_bits(unsigned char *bits, uint32_t nbits) {
  unsigned i;
  unsigned style = h_rmod(6);
  for (i = 0; i < 200; i++) {
    unsigned v;
    switch (style) { case 0: v = h_rnd() & 1; break; case 1: v = 1; break; case 2: v = 0; break; case 3: v = (i == (nbits ? nbits - 1 : 0)); break; case 4: v = h_rpct(90); break; default: v = h_rpct(10); }
    bits[i] = (unsigned char)(h_rpct(30) ? (v | (h_rnd() & 0xfe)) : v);   /* only bit 0 counts */
  }
}

static void run_progress(int big, int ntrials, int chain) {
  int trial, k;
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < ntrials; trial++) {
    V92ModEncObj *o;
    MapPar92 *p;
    unsigned char *bits;
    unsigned *dig;
    uint32_t m[12];
    int calls = chain ? 2 + (int)h_rmod(4) : 1, ci;
    h_pool_reset();
    o = (V92ModEncObj *)onew_canary(sizeof *o);
    p = (MapPar92 *)onew(sizeof *p);
    bits = (unsigned char *)onew(256);
    dig = (unsigned *)onew_canary(16 * sizeof(unsigned));
    garbage_fill(o, sizeof *o, h_rnd());
    rnd_radices(m, 0);
    if (big && h_rpct(50)) { m[0] = 2 + h_rmod(8); m[1] = h_rpct(50) ? 2 + h_rmod(8) : (1u << (1 + h_rmod(31))); if (h_rpct(30)) m[0] = 1u << (1 + h_rmod(31)); }
    {
      static const uint32_t nb[] = {0, 1, 2, 31, 32, 33, 62, 63, 64, 65, 66, 100, 125, 126, 127, 128, 129, 130, 140};
      uint32_t nbits = h_rpct(50) ? nb[h_rmod(sizeof nb / sizeof nb[0])] : h_rmod(142);
      if (big && nbits < 64) nbits = 64 + h_rmod(70);
      p->w0_ = (int32_t)nbits; p->a_ = h_rnd(); p->b_ = h_rnd(); p->c_ = h_rnd(); p->trellis_ = (int32_t)h_rnd(); p->extendEu_ = (int32_t)h_rnd();
      { uint32_t gb = h_rnd(); memcpy(&p->gain_, &gb, 4); }
      for (k = 0; k < 12; k++) p->m_[k] = m[k];
      for (k = 0; k < 4; k++) p->cnt_[k] = h_rnd();
      for (k = 0; k < 6; k++) { p->lc_[k] = h_rnd(); p->idxc_[k] = h_rnd(); }
    }
    me_reset(o, p);
    if (h_rpct(35)) {   /* boundary (a_, b_) instead of the reset-made pair */
      put64(o->a_, rb64()); put64(o->b_, rb64());
      COV(CV_P_BOUNDARY);
    } else COV(CV_P_RESETMADE);
    o->flag_ = h_rpct(50) ? 0 : (h_rpct(50) ? 1 : h_rnd());
    o->mode_ = 0;
    for (ci = 0; ci < calls; ci++) {
      V92ModEncObj pre;
      Num n;
      int64_t Xp, Yp;
      uint32_t newflag, savedflag, d[12], dd[12];
      uint64_t hh;
      fill_bits(bits, o->nbits_);
      if (big && h_rpct(60)) {   /* steer: make the high part non-zero (bits 63 ..): ones in the upper bytes */
        unsigned i;
        for (i = 63; i < 140; i++) if (h_rpct(55)) bits[i] |= 1;
      }
      for (k = 0; k < 16; k++) dig[k] = 0xd1900000u + (unsigned)k;
      memcpy(&pre, o, sizeof pre);
      savedflag = o->flag_;
      n = model_accum(bits, o->nbits_);
      newflag = model_prep(o, n, &Xp, &Yp);
      me_prog(o, bits, dig);
      COV(CV_P_CALL);
      /* the object: only flag_ may change (to the model's value) */
      EXPECT_EQ("progress flag", o->flag_, newflag);
      pre.flag_ = newflag;
      if (memcmp(&pre, o, sizeof pre)) h_die3("progress changed an object word other than the flag");
      for (k = 12; k < 16; k++) EXPECT_EQ("digit guard", dig[k], 0xd1900000u + (unsigned)k);
      {
        uint32_t oldflag = o->flag_ ^ newflag;   /* the flag on entry: new = old ^ t with t in {0, 1}, so old != 0 iff (old ^ new) ... recomputed from the saved copy below */
        (void)oldflag;
      }
      if (savedflag != 0) COV(CV_P_FLAG); else COV(CV_P_NOFLAG);
      if (newflag != savedflag) COV(CV_P_T1); else COV(CV_P_T0);
      if (o->nbits_ >= 0x40) COV(CV_P_NGE64); else COV(CV_P_NLT64);
      if (o->nbits_ == 63) COV(CV_P_N63);
      if (Yp < 0) COV(CV_P_NEGY);
      if (get64(o->a_) & 1) COV(CV_P_ODD_A);
      if ((int64_t)get64(o->a_) < 0) COV(CV_P_NEG_A);
      if (get64(o->a_) == 0) COV(CV_P_ZERO_A);
      for (k = 0; k < 12; k++) d[k] = dig[k];
      hh = fnv_upd(FNV0, d, sizeof d);
      if (Xp <= 0) {
        COV(CV_P_SIMPLE);
        model_simple(Yp, m, dd);
        for (k = 0; k < 12; k++) if (d[k] != dd[k]) h_die3("simple path digit %d: got %u, model %u (Xp %lld Yp %lld nbits %u)", k, d[k], dd[k], (long long)Xp, (long long)Yp, o->nbits_);
        COV(CV_P_SIMPLE_MODEL);
      } else {
        int agree = 1;
        COV(CV_P_BIG);
        ideal_digits(Xp, Yp < 0 ? 0 : Yp, m, dd);
        for (k = 0; k < 12; k++) if (d[k] != dd[k]) agree = 0;
        if (Yp >= 0) { if (agree) COV(CV_P_IDEAL_EQ); else COV(CV_P_IDEAL_NE); }
        if ((uint64_t)Xp < m[0]) COV(CV_P_CASEB); else COV(CV_P_CASEC);
        h_emit(0, "big ideal %d X %lld Y %lld m %u %u %u %u %u %u %u %u %u %u %u %u d %u %u %u %u %u %u %u %u %u %u %u %u", Yp >= 0 ? agree : -1, (long long)Xp, (long long)Yp, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10], d[11]);
      }
      h_emit(0, "prog n %u flag %u hash %016llx d %08x %08x %08x %08x", o->nbits_, o->flag_, (unsigned long long)hh, d[0], d[1], d[2], d[11]);
      DUMPE("prog", o);
      if (chain && ci > 0) COV(CV_P_CHAIN);
    }
    canary_check(o, sizeof *o, "progress");
    canary_check(dig, 16 * sizeof(unsigned), "progress digits");
    h_trial_end("prog");
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P15_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P15_DBG")) g_show = 1;
  p11_layout_check();
  p15_layout_check();
  h_x87_check();
  h_arena_init();
  {
    int i, n = 0;
    for (i = 0; i < NF_V92ModEncObj; i++) g_fe[n++] = f_V92ModEncObj[i];
    g_ne = n;
  }
  switch (sc) {
    case 0: sc_ctor(); break;
    case 1: sc_reset(); break;
    case 2:
      cov_def(CV_P_CALL, "progress call"); cov_def(CV_P_SIMPLE, "simple path (high part <= 0)"); cov_def(CV_P_BIG, "large-number path (high part > 0)"); cov_def(CV_P_FLAG, "flag set on entry");
      cov_def(CV_P_NOFLAG, "flag clear on entry"); cov_def(CV_P_T1, "flag set by the call"); cov_def(CV_P_T0, "flag clear before and after"); cov_def(CV_P_NGE64, "nbits >= 64"); cov_def(CV_P_NLT64, "nbits < 64");
      cov_def(CV_P_N63, "nbits == 63"); cov_def(CV_P_NEGY, "negative low part after the fix-up"); cov_def(CV_P_SIMPLE_MODEL, "simple path checked against the model"); cov_def(CV_P_RESETMADE, "reset-made (a_, b_)");
      cov_def(CV_P_BOUNDARY, "boundary (a_, b_)"); cov_def(CV_P_ZERO_A, "a_ == 0"); cov_def(CV_P_ODD_A, "odd a_"); cov_def(CV_P_NEG_A, "negative a_");
      h_covreq[CV_P_BIG] = 0; h_covreq[CV_P_NEGY] = 0; h_covreq[CV_P_T0] = 0;
      run_progress(0, 6000, 0);
      break;
    case 3:
      cov_def(CV_P_CALL, "progress call"); cov_def(CV_P_BIG, "large-number path (high part > 0)"); cov_def(CV_P_CASEB, "0 < high < m0"); cov_def(CV_P_CASEC, "high >= m0");
      cov_def(CV_P_IDEAL_EQ, "vendor digits == exact digits"); cov_def(CV_P_IDEAL_NE, "vendor digits != exact digits"); cov_def(CV_P_NGE64, "nbits >= 64"); cov_def(CV_P_FLAG, "flag set on entry");
      h_covreq[CV_P_IDEAL_EQ] = 0;   /* the vendor's large-number digits are never the exact digits of hi * 2^63 + lo: see NOTES (P15), blob vs port only */
      run_progress(1, 8000, 0);
      break;
    case 4:
      cov_def(CV_P_CALL, "progress call"); cov_def(CV_P_CHAIN, "second or later call on the same object"); cov_def(CV_P_BIG, "large-number path (high part > 0)"); cov_def(CV_P_SIMPLE, "simple path");
      run_progress(0, 3000, 1);
      break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu", h_nmalloc, h_nfree);
  h_cov_finish();
  return 0;
}
