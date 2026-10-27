/*
 * Differential test harness for port chunk P14 (src/vpcm/pcm_jd.cpp): classes V90Jd (15 functions) and V92Jd (21 functions) - the V.90 / V.92 phase-3 "Jd" frame holders and bit-serial
 * codecs - and the four free functions V92createConstellations / V92createFilterCoefficients / V92deleteConstellations / V92deleteFilterCoefficients (40 functions).
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/pcm_jd.sh against the blob (dsplibs.o), against the port (-O0 / -O2) and for x86-64 (diffed against the
 * 32-bit BLOB's output = the golden, test/golden/vpcm/pcm_jd.blob32.out).  The functions are reached through __asm__ labels with the mangled names (C names for the four free functions); the
 * objects are the C mirrors generated from test/vpcm/pcm_jd.layout (V90Jd, V92Jd: no pointers), test/vpcm/pcm_parameters.layout (the V90Parameters block the constructors read) and test/vpcm/v92_cp.layout
 * (V92MappingParams: the block the free functions fill; real pointer members, so it widens on x86-64).  The objects are built by the blob's / the port's OWN constructors where
 * the scenario starts from a constructed object, and garbage-filled otherwise.
 *
 * What is compared: every return value and out-parameter, the WHOLE object (bytes, every byte of the 0x90 / 0xdc) after every call, the V92MappingParams field-wise with the heap
 * blocks it owns, the malloc / free log (order and sizes).  INDEPENDENT MODELS (written from the frame definition, not from the port): the TX frame of the byte-per-bit buffer
 * (17 ones, start bits, two rows, CRC-CCITT of the parity of 32 bytes, four zeros) with the CRC as a standard 16-bit register, the receive side as a STREAM oracle (a frame built
 * by the model must complete at exactly its last bit, a damaged frame never does, the decoded rate mask / constellation bits / look-ahead / phase equal the sent ones), and the
 * closure TX frame -> receiver.  Plus per-state random-object fuzzing (blob vs port only) of all three receive state machines.
 *
 * Scenarios (each its own process): 0 V90Jd ctor / setters / getters / TX frame, 1 V90Jd receiver, 2 V92Jd ctor / setters / getters / phase / TX frames, 3 V92Jd receivers (data and
 * phase codecs), 4 the four free functions.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "pcm_parameters_layout.h"
#include "v92_cp_layout.h"
#include "pcm_jd_layout.h"

#define L(m) __asm__(m)

/* V90Jd */
extern void v90_c1(void *, void *) L("_ZN5V90JdC1EP13V90Parameters");
extern void v90_c2(void *, void *) L("_ZN5V90JdC2EP13V90Parameters");
extern void v90_d1(void *) L("_ZN5V90JdD1Ev");
extern void v90_d2(void *) L("_ZN5V90JdD2Ev");
extern void v90_setla(void *, unsigned) L("_ZN5V90Jd15setMaxLookaheadEh");
extern void v90_setcs(void *, unsigned, unsigned) L("_ZN5V90Jd14setConstelSizeEhh");
extern void v90_setrm(void *, int) L("_ZN5V90Jd12setRatesMaskEi");
extern int v90_getrm(void *) L("_ZN5V90Jd12getRatesMaskEv");
extern void v90_getcs(void *, unsigned char *, unsigned char *) L("_ZN5V90Jd19getConstelationSizeEPhS0_");
extern unsigned char v90_getla(void *) L("_ZN5V90Jd15getMaxLookaheadEv");
extern void v90_resetcrc(void *) L("_ZN5V90Jd8resetCrcEv");
extern void v90_pack(void *) L("_ZN5V90Jd8packDataEv");
extern void v90_unreset(void *) L("_ZN5V90Jd11unPackResetEv");
extern int v90_unpack(void *, int) L("_ZN5V90Jd10unPackDataEi");
extern unsigned char *v90_getbv(void *) L("_ZN5V90Jd12getBitVectorEv");
/* V92Jd */
extern void v92_c1(void *, void *) L("_ZN5V92JdC1EP13V90Parameters");
extern void v92_c2(void *, void *) L("_ZN5V92JdC2EP13V90Parameters");
extern void v92_d1(void *) L("_ZN5V92JdD1Ev");
extern void v92_d2(void *) L("_ZN5V92JdD2Ev");
extern void v92_setph(void *, float) L("_ZN5V92Jd10setJdPhaseEf");
extern void v92_setla(void *, unsigned) L("_ZN5V92Jd15setMaxLookaheadEh");
extern void v92_setrm(void *, int) L("_ZN5V92Jd12setRatesMaskEi");
extern void v92_setcs(void *, unsigned, unsigned) L("_ZN5V92Jd14setConstelSizeEhh");
extern float v92_getph(void *) L("_ZN5V92Jd10getJdPhaseEv");
extern int v92_getrm(void *) L("_ZN5V92Jd12getRatesMaskEv");
extern void v92_getcs(void *, unsigned char *, unsigned char *) L("_ZN5V92Jd19getConstelationSizeEPhS0_");
extern unsigned char v92_getla(void *) L("_ZN5V92Jd15getMaxLookaheadEv");
extern void v92_resetcrc(void *) L("_ZN5V92Jd8resetCrcEv");
extern void v92_packph(void *) L("_ZN5V92Jd15packJdPhaseDataEv");
extern void v92_packd(void *) L("_ZN5V92Jd10packJdDataEv");
extern void v92_unreset(void *) L("_ZN5V92Jd13unPackJdResetEv");
extern void v92_unphreset(void *) L("_ZN5V92Jd18unPackJdPhaseResetEv");
extern int v92_unphase(void *, int) L("_ZN5V92Jd17unPackJdPhaseDataEi");
extern int v92_undata(void *, int) L("_ZN5V92Jd12unPackJdDataEi");
extern unsigned char *v92_getbv(void *) L("_ZN5V92Jd14getJdBitVectorEv");
extern unsigned char *v92_getphbv(void *) L("_ZN5V92Jd19getJdPhaseBitVectorEv");
/* the free functions */
extern void mk_cons(void *) L("V92createConstellations");
extern void mk_filt(void *) L("V92createFilterCoefficients");
extern void rm_cons(void *) L("V92deleteConstellations");
extern void rm_filt(void *) L("V92deleteFilterCoefficients");

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
static HFld g_f90[8], g_f92[16], g_fm[32];
static int g_n90, g_n92, g_nm;
static unsigned long g_dumpctr;
#define DUMP90(tag, o) h_dump_obj(h_sample(&g_dumpctr, 12, 97), (tag), (o), g_f90, g_n90)
#define DUMP92(tag, o) h_dump_obj(h_sample(&g_dumpctr, 12, 97), (tag), (o), g_f92, g_n92)
static float bf(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }

/* ------------------------------------------------------------------------------------------------ models */
/* CRC-CCITT register (all ones) of the parity of each value; reg[k] = bit (15 - k) */
static void model_crc(const uint8_t *vals, unsigned n, uint32_t reg[16]) {
  uint16_t r = 0xffff;
  unsigned i, k;
  for (i = 0; i < n; i++) {
    unsigned fb = ((r >> 15) ^ ((unsigned)vals[i] & 1u)) & 1u;
    r = (uint16_t)(r << 1);
    if (fb) r ^= 0x1021;
  }
  for (k = 0; k < 16; k++) reg[k] = (r >> (15 - k)) & 1;
}

/* the expected byte buffer (72 bytes) of a TX frame: pre = the 0x48 bytes before the call; variant 0 = V90Jd::packData / getBitVector, 1 = V92Jd::packJdData, 2 = V92Jd::packJdPhaseData */
static void model_tx(const uint8_t *pre, int variant, uint8_t *out, uint32_t reg[16]) {
  uint8_t v[32];
  unsigned i;
  memcpy(out, pre, 0x48);
  for (i = 0; i <= 16; i++) out[i] = 1;
  out[17] = 0; out[34] = 0; out[51] = 0;
  out[68] = 0; out[69] = 0; out[70] = 0; out[71] = 0;
  if (variant == 1) { for (i = 0x29; i <= 0x2e; i++) out[i] = 0; out[0x30] = 0; }
  if (variant == 2) { for (i = 0x23; i <= 0x2e; i++) out[i] = 0; out[0x32] = 0; }
  for (i = 0; i < 16; i++) { v[i] = out[18 + i]; v[16 + i] = out[35 + i]; }
  model_crc(v, 32, reg);
  for (i = 0; i < 16; i++) out[52 + i] = (uint8_t)reg[i];
}

/* the wire sequence of one frame: payload bits (32, in the receiver's order) -> the bits to feed; layout[0] = segment lengths */
typedef struct { uint8_t bit[128]; unsigned n; } Wire;
static void w_put(Wire *w, int b) { w->bit[w->n++] = (uint8_t)b; }
/* kind 0 V90 data, 1 V92 data, 2 V92 phase: the three have the same wire format (17 ones, 0, 16, 0, 16 (split 12 + 4 / 11 + 1 + 4 / 12 + 4 inside the receiver), 0, 16 CRC, 4 trailing) */
static void build_wire(Wire *w, const uint8_t payload[32], int dmg, unsigned dmgpos) {
  unsigned i;
  uint8_t pv[32];
  uint32_t reg[16];
  w->n = 0;
  memcpy(pv, payload, 32);
  model_crc(pv, 32, reg);
  for (i = 0; i < 17; i++) w_put(w, 1);
  w_put(w, 0);
  for (i = 0; i < 16; i++) w_put(w, pv[i]);
  w_put(w, 0);
  for (i = 16; i < 32; i++) w_put(w, pv[i]);
  w_put(w, 0);
  for (i = 0; i < 16; i++) w_put(w, (int)reg[i]);
  for (i = 0; i < 4; i++) w_put(w, 0);
  if (dmg == 1) w->bit[dmgpos] ^= 1;   /* flip any bit of the frame body */
}

/* ------------------------------------------------------------------------------------------------ the rx front end: kind 0 V90, 1 V92 data, 2 V92 phase */
static int rx(int kind, void *o, int bit) { return kind == 0 ? v90_unpack(o, bit) : kind == 1 ? v92_undata(o, bit) : v92_unphase(o, bit); }
static void rxreset(int kind, void *o) { if (kind == 0) v90_unreset(o); else if (kind == 1) v92_unreset(o); else v92_unphreset(o); }
static size_t objsz(int kind) { return kind == 0 ? sizeof(V90JdObj) : sizeof(V92JdObj); }

enum { CV_A_C1 = 0, CV_A_C2, CV_A_D, CV_A_SETLA, CV_A_SETCS, CV_A_SETRM, CV_A_GETS, CV_A_RESETCRC, CV_A_PACK, CV_A_GETBV, CV_A_UNRESET, CV_A_GARB, CV_A_NEGMASK, CV_A_BIGMASK,
       CV_R_FRAME, CV_R_DMG, CV_R_NOISE, CV_R_STATE0, CV_R_STATE1, CV_R_STATE2, CV_R_STATE3, CV_R_STATE4, CV_R_STATE5, CV_R_STATE6, CV_R_STATE7, CV_R_STATE8, CV_R_STATE9, CV_R_STATEBIG,
       CV_R_RET1, CV_R_POST, CV_R_CRCFAIL, CV_R_STARTBAD, CV_R_MARKERBAD, CV_R_WRAP, CV_R_GARBBIT, CV_R_CLOSURE, CV_R_FUZZ,
       CV_P_NAN, CV_P_INF, CV_P_NEG, CV_P_BIG, CV_P_FRAC, CV_P_ZERO, CV_P_EXACT, CV_P_SETPH,
       CV_F_CONS, CV_F_FILT, CV_F_DELC, CV_F_DELF, CV_F_NULLS, CV_F_ALLNULL, CV_F_FAIL, CV_F_PARTIAL };

/* ================================================================================================ scenarios 0 / 2: constructors, setters, getters, TX frames */
static void sc_static(int v92) {
  int trial, k;
  cov_def(CV_A_C1, "ctor C1"); cov_def(CV_A_C2, "ctor C2"); cov_def(CV_A_D, "dtor D1 / D2"); cov_def(CV_A_SETLA, "setMaxLookahead"); cov_def(CV_A_SETCS, "setConstelSize"); cov_def(CV_A_SETRM, "setRatesMask");
  cov_def(CV_A_GETS, "getters"); cov_def(CV_A_RESETCRC, "resetCrc"); cov_def(CV_A_PACK, "packData / packJdData / packJdPhaseData"); cov_def(CV_A_GETBV, "getBitVector variants"); cov_def(CV_A_UNRESET, "unPack reset functions");
  cov_def(CV_A_GARB, "garbage in the upper bits of a byte argument"); cov_def(CV_A_NEGMASK, "negative rate mask"); cov_def(CV_A_BIGMASK, "mask bits above 27");
  if (v92) { cov_def(CV_P_NAN, "phase NaN"); cov_def(CV_P_INF, "phase inf"); cov_def(CV_P_NEG, "negative phase"); cov_def(CV_P_BIG, "phase >= 2^63 / 65536"); cov_def(CV_P_FRAC, "phase with a fraction"); cov_def(CV_P_ZERO, "phase 0");
             cov_def(CV_P_EXACT, "phase an exact multiple of 2^-16 in range"); cov_def(CV_P_SETPH, "setJdPhase"); }
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 3000; trial++) {
    void *o;
    V90ParObj *par;
    int mask, expmask, i;
    uint32_t pw;
    unsigned sz = (unsigned)objsz(v92);
    uint8_t pre[sizeof(V92JdObj)], img[sizeof(V92JdObj)];
    uint8_t tx[0x48];
    uint32_t reg[16];
    unsigned op = h_rmod(14);
    unsigned la = h_garbage_hi(h_rmod(256));
    unsigned c1 = h_garbage_hi(h_rmod(256)), c2 = h_garbage_hi(h_rmod(256));
    float phf;
    h_pool_reset();
    o = onew_canary(sz);
    par = (V90ParObj *)onew(sizeof(V90ParObj));
    garbage_fill(o, sz, h_rnd());
    garbage_fill(par, sizeof *par, h_rnd());
    mask = h_rpct(30) ? (int)(h_rnd()) : (h_rpct(30) ? (int)(h_rnd() & 0x0fffffffu) : (int)(h_rnd() & 0xffffu));
    if (h_rpct(8)) mask = (int)0xffffffffu;
    if (h_rpct(8)) mask = 0;
    if (h_rpct(5)) mask = (int)0x80000000u;
    if (mask < 0) COV(CV_A_NEGMASK);
    if ((uint32_t)mask >> 28) COV(CV_A_BIGMASK);
    if (la >> 8 || c1 >> 8) COV(CV_A_GARB);
    { uint32_t b; switch (h_rmod(10)) { case 0: b = 0x7fc00000u; break; case 1: b = 0xff800000u; break; case 2: b = 0x7f800000u; break; case 3: b = 0x5f000000u; break; case 4: b = 0xdf000000u; break;
        case 5: b = 0x3f800000u; break; case 6: b = h_rnd(); break; case 7: b = 0; break; case 8: b = 0x80000000u; break; default: b = 0x38000000u + (h_rnd() & 0x0fffffffu); }
      phf = bf(b); }
    par->digitalRateMask = mask; par->maxSpectralShaperLookahead = (int32_t)(h_rpct(50) ? h_rnd() : h_rmod(8)); par->v34Phase4Constellation = (int32_t)h_rnd(); par->v34RrnConstellation = (int32_t)h_rnd();
    par->v92DigitalRateMask = mask; par->v92MaxSpectralShaperLookahead = par->maxSpectralShaperLookahead; par->v92JdPhase = phf;
    memcpy(pre, o, sz);
    /* ------- the constructor (C1 or C2) against the expected image */
    {
      int c2v = h_rpct(40);
      memcpy(img, pre, sz);
      if (!v92) {
        V90JdObj *m = (V90JdObj *)img;
        uint8_t *b = m->bits_;
        m->ones_ = 0; m->idx_ = 0; m->state_ = 0;
        b[0x31] = (uint8_t)(par->maxSpectralShaperLookahead & 1); b[0x32] = (uint8_t)((par->maxSpectralShaperLookahead >> 1) & 1);
        b[0x2f] = (uint8_t)par->v34Phase4Constellation; b[0x30] = (uint8_t)par->v34RrnConstellation;
        for (i = 0; i < 16; i++) b[0x12 + i] = (uint8_t)((mask >> i) & 1);
        for (i = 0; i < 12; i++) b[0x23 + i] = (uint8_t)((mask >> (16 + i)) & 1);
        if (c2v) { v90_c2(o, par); COV(CV_A_C2); } else { v90_c1(o, par); COV(CV_A_C1); }
      } else {
        V92JdObj *m = (V92JdObj *)img;
        uint8_t *b = m->bits_, *ph = m->phase_;
        double v;
        uint64_t w = 0;
        m->ones_ = 0; m->idx_ = 0; m->stateJd_ = 0; m->statePhase_ = 0;
        b[0x2f] = 0; ph[0x2f] = 1; ph[0x30] = 0; ph[0x31] = 0;
        v = (double)phf * 65536.0;
        if (v == v && v > -9223372036854775809.0 && v < 9223372036854775808.0) w = (uint64_t)(int64_t)v;
        for (i = 0; i < 16; i++) ph[0x12 + i] = (uint8_t)((w >> i) & 1);
        b[0x31] = (uint8_t)(par->v92MaxSpectralShaperLookahead & 1); b[0x32] = (uint8_t)((par->v92MaxSpectralShaperLookahead >> 1) & 1);
        for (i = 0; i < 16; i++) b[0x12 + i] = (uint8_t)((mask >> i) & 1);
        for (i = 0; i < 11; i++) b[0x23 + i] = (uint8_t)((mask >> (16 + i)) & 1);
        if (c2v) { v92_c2(o, par); COV(CV_A_C2); } else { v92_c1(o, par); COV(CV_A_C1); }
      }
      if (memcmp(o, img, sz)) h_die3("%s constructor %s: object differs from the model at byte %d", v92 ? "V92Jd" : "V90Jd", c2v ? "C2" : "C1", (int)(((uint8_t *)o)[0] != img[0]) );
      if (v92) { if (phf != phf) COV(CV_P_NAN);
      else if (phf > 3.4e38f || phf < -3.4e38f) COV(CV_P_INF);
      else if (phf < 0) COV(CV_P_NEG);
      else if (phf == 0) COV(CV_P_ZERO); }
      if (v92 && phf == phf && phf < 1e30f && phf > -1e30f) {
        double vv = (double)phf * 65536.0;
        if (vv >= 9.2e18 || vv <= -9.2e18) COV(CV_P_BIG);
        if (vv == (double)(int64_t)vv) COV(CV_P_EXACT); else COV(CV_P_FRAC);
      } else if (v92 && phf == phf && (phf >= 1e19f || phf <= -1e19f)) COV(CV_P_BIG);
      h_emit(0, "ctor %d hash %016llx", v92, (unsigned long long)fnv_upd(FNV0, o, sz));
    }
    pw = 0;
    (void)pw;
    /* ------- one operation on the constructed object */
    memcpy(pre, o, sz);
    switch (op) {
      case 0: case 1: case 2: {   /* setMaxLookahead / setConstelSize / setRatesMask against the model, then the getters */
        uint8_t *bits = v92 ? ((V92JdObj *)img)->bits_ : ((V90JdObj *)img)->bits_;
        memcpy(img, o, sz);
        if (op == 0) { if (v92) v92_setla(o, la); else v90_setla(o, la); COV(CV_A_SETLA); bits[0x31] = (uint8_t)(la & 1); bits[0x32] = (uint8_t)((la >> 1) & 1); }
        else if (op == 1) {
          if (v92) { v92_setcs(o, c1, c2); ((V92JdObj *)img)->phase_[0x30] = (uint8_t)c1; ((V92JdObj *)img)->phase_[0x31] = (uint8_t)c2; }
          else { v90_setcs(o, c1, c2); bits[0x2f] = (uint8_t)c1; bits[0x30] = (uint8_t)c2; }
          COV(CV_A_SETCS);
        } else {
          int n2 = v92 ? 11 : 12;
          mask = (int)h_rnd();   /* a NEW mask (the constructor stored `mask` already) */
          if (v92) v92_setrm(o, mask); else v90_setrm(o, mask);
          COV(CV_A_SETRM);
          for (i = 0; i < 16; i++) bits[0x12 + i] = (uint8_t)((mask >> i) & 1);
          for (i = 0; i < n2; i++) bits[0x23 + i] = (uint8_t)((mask >> (16 + i)) & 1);
        }
        if (memcmp(o, img, sz)) h_die3("setter %u (%s): object differs from the model", op, v92 ? "V92Jd" : "V90Jd");
        break;
      }
      case 3: case 4: case 5: {   /* getters on a garbage buffer: the byte regions are read as they are */
        unsigned char ra = 0x55, rb = 0x66;
        uint8_t *bits = v92 ? ((V92JdObj *)o)->bits_ : ((V90JdObj *)o)->bits_;
        uint8_t *ph = v92 ? ((V92JdObj *)o)->phase_ : NULL;
        int rm = v92 ? v92_getrm(o) : v90_getrm(o);
        unsigned char la2 = v92 ? v92_getla(o) : v90_getla(o);
        float phv = 0;
        expmask = 0;
        for (i = 0; i < 16; i++) if (bits[i]) expmask |= 1 << i;
        for (i = 0; i < 12; i++) if (bits[0x10 + i]) expmask |= 1 << (16 + i);
        EXPECT_EQ("getRatesMask", (uint32_t)rm, (uint32_t)expmask);
        EXPECT_EQ("getMaxLookahead", la2, ((bits[0x1f] & 1) * 2) + (bits[0x1e] & 1));
        if (v92) v92_getcs(o, &ra, &rb); else v90_getcs(o, &ra, &rb);
        if (!v92) { EXPECT_EQ("getConstelationSize a", ra, bits[0x1c]); EXPECT_EQ("getConstelationSize b", rb, bits[0x1d]); }
        else {
          unsigned w = 0;
          EXPECT_EQ("getConstelationSize a", ra, ph[0x1d]); EXPECT_EQ("getConstelationSize b", rb, ph[0x1e]);
          phv = v92_getph(o);
          for (i = 0; i < 16; i++) if (ph[i]) w |= 1u << i;
          EXPECT_EQ("getJdPhase", h_bits_of(phv), h_bits_of((float)w / 65536.0f));
        }
        COV(CV_A_GETS);
        if (memcmp(o, pre, sz)) h_die3("a getter changed the object");
        h_emit(0, "get mask %08x la %u cs %u %u ph %08x", (unsigned)rm, la2, ra, rb, v92 ? h_bits_of(phv) : 0);
        break;
      }
      case 6: {   /* resetCrc */
        memcpy(img, o, sz);
        if (v92) { v92_resetcrc(o); for (k = 0; k < 16; k++) ((V92JdObj *)img)->crc_[k] = 1; } else { v90_resetcrc(o); for (k = 0; k < 16; k++) ((V90JdObj *)img)->crc_[k] = 1; }
        COV(CV_A_RESETCRC);
        if (memcmp(o, img, sz)) h_die3("resetCrc differs from the model");
        break;
      }
      case 7: case 8: case 9: case 10: {   /* the TX frame builders: with bytes of 0 / 1 / anything in the "send" rows */
        uint8_t *bits = v92 ? ((V92JdObj *)o)->bits_ : ((V90JdObj *)o)->bits_;
        uint8_t *ph = v92 ? ((V92JdObj *)o)->phase_ : NULL;
        uint32_t *crc = v92 ? ((V92JdObj *)o)->crc_ : ((V90JdObj *)o)->crc_;
        unsigned char *rv;
        int variant = !v92 ? 0 : (op & 1) ? 1 : 2;
        uint8_t *base = variant == 2 ? ph : bits;
        if (h_rpct(60)) for (i = 0; i < 0x48; i++) base[i] = (uint8_t)(h_rpct(50) ? (h_rnd() & 1) : h_rnd());
        ((uint8_t *)o)[0] = (uint8_t)h_rnd(); ((uint8_t *)o)[1] = (uint8_t)h_rnd();   /* the receive state must stay untouched (a CRC copy one word too long would overwrite it) */
        if (v92) { ((V92JdObj *)o)->stateJd_ = h_rnd(); ((V92JdObj *)o)->statePhase_ = h_rnd(); } else ((V90JdObj *)o)->state_ = h_rnd();
        memcpy(pre, o, sz);
        model_tx(base, variant, tx, reg);
        if (op >= 9) {
          if (!v92) rv = v90_getbv(o); else if (variant == 1) rv = v92_getbv(o); else rv = v92_getphbv(o);
          COV(CV_A_GETBV);
          EXPECT_EQ("getBitVector return", (uintptr_t)rv, (uintptr_t)base);
        } else {
          if (!v92) v90_pack(o); else if (variant == 1) v92_packd(o); else v92_packph(o);
          COV(CV_A_PACK);
        }
        for (i = 0; i < 0x48; i++) if (base[i] != tx[i]) h_die3("TX frame variant %d: byte %d is %02x, model %02x", variant, i, base[i], tx[i]);
        for (i = 0; i < 16; i++) EXPECT_EQ("TX frame crc register", crc[i], reg[i]);
        { size_t j; for (j = 0; j < sz; j++) { size_t lo = variant == 2 ? 0x4a : 2; if (j >= lo && j < lo + 0x48) continue; if (j >= (v92 ? 0x94u : 0x4cu) && j < (v92 ? 0x94u : 0x4cu) + 64) continue; if (((uint8_t *)o)[j] != pre[j]) h_die3("TX frame builder changed byte 0x%zx outside its buffer", j); } }
        break;
      }
      case 11: case 12: {   /* unPackReset / unPackJdReset / unPackJdPhaseReset */
        memcpy(img, o, sz);
        if (!v92) { v90_unreset(o); ((V90JdObj *)img)->idx_ = 0; ((V90JdObj *)img)->state_ = 0; ((V90JdObj *)img)->ones_ = 0; }
        else if (op == 11) { v92_unreset(o); ((V92JdObj *)img)->idx_ = 0; ((V92JdObj *)img)->stateJd_ = 0; ((V92JdObj *)img)->ones_ = 0; }
        else { v92_unphreset(o); ((V92JdObj *)img)->idx_ = 0; ((V92JdObj *)img)->statePhase_ = 0; ((V92JdObj *)img)->ones_ = 0; }
        COV(CV_A_UNRESET);
        if (memcmp(o, img, sz)) h_die3("reset differs from the model");
        break;
      }
      default: {   /* setJdPhase (V92Jd) / the destructors */
        if (v92) {
          uint64_t w = 0;
          double v;
          { uint32_t b = h_rpct(30) ? h_rnd() : (h_rpct(50) ? (0x37000000u + (h_rnd() & 0x0fffffffu)) : (h_rpct(50) ? 0x7fc00000u : 0xdf000000u)); phf = bf(b); }
          v = (double)phf * 65536.0;
          memcpy(img, o, sz);
          if (v == v && v > -9223372036854775809.0 && v < 9223372036854775808.0) w = (uint64_t)(int64_t)v;
          for (i = 0; i < 16; i++) ((V92JdObj *)img)->phase_[0x12 + i] = (uint8_t)((w >> i) & 1);
          v92_setph(o, phf);
          COV(CV_P_SETPH);
          if (memcmp(o, img, sz)) h_die3("setJdPhase %08x differs from the model", h_bits_of(phf));
        }
        if (h_rpct(50)) { if (v92) v92_d1(o); else v90_d1(o); } else { if (v92) v92_d2(o); else v90_d2(o); }
        COV(CV_A_D);
        break;
      }
    }
    canary_check(o, sz, "Jd object");
    h_emit(0, "op %u hash %016llx", op, (unsigned long long)fnv_upd(FNV0, o, sz));
    if (v92) DUMP92("v92jd", o); else DUMP90("v90jd", o);
    h_trial_end("jd");
  }
}

/* ================================================================================================ scenarios 1 / 3: the receivers */
static void rnd_payload(uint8_t p[32], int v92kind) {
  unsigned i;
  for (i = 0; i < 32; i++) p[i] = (uint8_t)(h_rpct(50) ? 1 : 0);
  if (h_rpct(10)) for (i = 0; i < 32; i++) p[i] = (uint8_t)h_rpct(15);
  if (v92kind == 1) p[28] = 0;      /* the V92 data frame ends its payload with a marker bit 0 at index 0x1c */
  if (v92kind == 2) p[28] = 1;      /* the phase frame's marker is 1 */
  (void)v92kind;
}

static void sc_rx(int v92) {
  int trial;
  static const int kmap[2] = {1, 2};
  cov_def(CV_R_FRAME, "valid frame completed at its last bit"); cov_def(CV_R_DMG, "damaged frame never completes"); cov_def(CV_R_NOISE, "noise in front of a frame"); cov_def(CV_R_STATE0, "state 0 reached");
  cov_def(CV_R_STATE1, "state 1 reached"); cov_def(CV_R_STATE2, "state 2 reached"); cov_def(CV_R_STATE3, "state 3 reached"); cov_def(CV_R_STATE4, "state 4 reached"); cov_def(CV_R_STATE5, "state 5 reached");
  cov_def(CV_R_STATE6, "state 6 reached"); cov_def(CV_R_STATE7, "state 7 reached"); cov_def(CV_R_STATE8, "state 8 reached");
  if (v92) { cov_def(CV_R_STATE9, "state 9 reached (V92 data)"); cov_def(CV_R_MARKERBAD, "frame with a bad marker bit"); }
  cov_def(CV_R_STATEBIG, "state > top (default)"); cov_def(CV_R_RET1, "return value 1"); cov_def(CV_R_POST, "bits after the frame"); cov_def(CV_R_CRCFAIL, "CRC mismatch reset"); cov_def(CV_R_STARTBAD, "bad start bit reset");
  cov_def(CV_R_WRAP, "index wrap after the frame"); cov_def(CV_R_GARBBIT, "a bit argument with garbage in the upper bits / > 1"); cov_def(CV_R_CLOSURE, "TX frame fed to the receiver"); cov_def(CV_R_FUZZ, "random object state fuzz");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 1800; trial++) {
    int kind = v92 ? kmap[h_rmod(2)] : 0;
    size_t sz = objsz(kind);
    uint8_t *o;
    int step;
    h_pool_reset();
    o = (uint8_t *)onew_canary(sz);
    garbage_fill(o, sz, h_rnd());
    rxreset(kind, o);
    if (kind != 0) { /* the other codec's state must not matter: leave it garbage but legal */ ((V92JdObj *)o)->stateJd_ = h_rmod(10); ((V92JdObj *)o)->statePhase_ = h_rmod(9); if (kind == 1) rxreset(1, o); else rxreset(2, o); }
    if (trial % 3 != 2) {
      /* ---- stream oracle: noise, then a frame (valid / damaged), flush, a second valid frame, a few bits more */
      Wire w, w2;
      uint8_t pay[32], pay2[32], val[32];
      uint64_t sh = FNV0;
      unsigned i, nn = h_rpct(60) ? h_rmod(40) : 0, ran = 0;
      int dmg = h_rpct(35) ? 1 : 0, dmgpos = 0, markerbad = 0;
      uint32_t exp_end = 0;
      rnd_payload(pay, kind);
      rnd_payload(pay2, kind);
      if (kind != 0 && h_rpct(8)) { pay[28] ^= 1; markerbad = 1; }   /* a bad marker: the frame is rejected at the end */
      build_wire(&w, pay, 0, 0);
      if (dmg) {   /* flip one bit of the frame body: not the preamble (it would only delay the start) and keep clear of the trailing bits */
        dmgpos = 17 + (int)h_rmod(51);
        w.bit[dmgpos] ^= 1;
      }
      build_wire(&w2, pay2, 0, 0);
      /* noise: no run of 16 or more ones (a run of 17 would start a frame) */
      {
        unsigned run = 0;
        for (i = 0; i < nn; i++) {
          int b = h_rpct(50);
          if (b && run >= 15) b = 0;
          run = b ? run + 1 : 0;
          if (rx(kind, o, b) != 0) h_die3("noise bit %u completed a frame", i);
          if (i == 0) COV(CV_R_NOISE);
        }
        if (nn && run) { rx(kind, o, 0); }   /* end the run of ones */
      }
      ran = 0;
      memcpy(val, pay, 32);
      for (i = 0; i < w.n; i++) {
        int bit = w.bit[i];
        int wire = bit;
        int r;
        if (h_rpct(10) && (i < 52 || i >= 68) && i != 47) {   /* (the 16 CRC bits and the marker are compared as bytes: no garbage there) garbage: the tests are on the whole int, the stored byte is the low byte (kept odd so that its parity is the bit); the preamble may use any non-zero int */
          wire = bit ? (int)(h_rnd() | 1) : 0;
          if (i < 17 && bit && h_rpct(40)) wire = (int)(h_rnd() & 0xffffff00u) | (h_rpct(50) ? 0 : (int)h_rmod(256));
          if (wire == 0 && bit) wire = 0x100;
          COV(CV_R_GARBBIT);
        }
        r = rx(kind, o, wire);
        sh = fnv_upd(sh, o, sz);
        if (i >= 18 && i <= 33) val[i - 18] = (uint8_t)wire; else if (i >= 35 && i <= 50) val[16 + i - 35] = (uint8_t)wire;
        ran += (unsigned)r;
        if (!dmg && !markerbad && i == w.n - 1) { if (r != 1) h_die3("a valid frame did not complete at its last bit (kind %d)", kind); COV(CV_R_FRAME); COV(CV_R_RET1); exp_end = 1; }
        else if (r != 0) h_die3("frame bit %u (of %u, dmg %d marker %d kind %d) returned %d", i, w.n, dmg, markerbad, kind, r);
      }
      if (dmg || markerbad) { COV(CV_R_DMG); if (markerbad) COV(CV_R_MARKERBAD); }
      if (exp_end) {
        /* decoded fields */
        const uint8_t *bits = kind == 2 ? ((V92JdObj *)o)->phase_ : (kind == 1 ? ((V92JdObj *)o)->bits_ : ((V90JdObj *)o)->bits_);
        int exprm = 0;
        for (i = 0; i < 16; i++) if (val[i]) exprm |= 1 << i;
        for (i = 0; i < 12; i++) if (val[16 + i]) exprm |= 1 << (16 + i);
        if (kind == 0) {
          unsigned char a, b;
          EXPECT_EQ("decoded rate mask", (uint32_t)v90_getrm(o), (uint32_t)exprm);
          v90_getcs(o, &a, &b);
          EXPECT_EQ("decoded constellation a", a, val[28]); EXPECT_EQ("decoded constellation b", b, val[29]);
          EXPECT_EQ("decoded look-ahead", v90_getla(o), (val[30] & 1) + 2 * (val[31] & 1));
        } else if (kind == 1) {
          unsigned char a, b;
          EXPECT_EQ("decoded rate mask", (uint32_t)v92_getrm(o), (uint32_t)exprm);
          EXPECT_EQ("decoded look-ahead", v92_getla(o), (val[30] & 1) + 2 * (val[31] & 1));
          (void)a; (void)b;
        } else {
          unsigned char a, b;
          unsigned wv = 0;
          float phv = v92_getph(o);
          for (i = 0; i < 16; i++) if (val[i]) wv |= 1u << i;
          EXPECT_EQ("decoded phase", h_bits_of(phv), h_bits_of((float)wv / 65536.0f));
          v92_getcs(o, &a, &b);
          EXPECT_EQ("decoded phase constellation a", a, val[29]); EXPECT_EQ("decoded phase constellation b", b, val[30]);
        }
        (void)bits;
        /* bits after the frame: the state machine stays in its last state and counts */
        for (i = 0; i < 1 + h_rmod(6); i++) { if (rx(kind, o, (int)h_rmod(2)) != 0) h_die3("a bit after the frame returned 1 (kind %d)", kind); COV(CV_R_POST); }
        if (h_rpct(10)) {   /* the index wraps after 256 bits: 256 - 4 - 1 more calls bring it back to 0x34 */
          unsigned cnt = 0, j;
          for (j = 0; j < 400; j++) { int r = rx(kind, o, 0); cnt += (unsigned)r; }
          if (cnt < 1) h_die3("the receiver never reported the frame again after the index wrapped");
          COV(CV_R_WRAP);
        }
      }
      /* flush then a second valid frame: must complete at exactly its last bit, nothing before */
      rxreset(kind, o);
      for (i = 0; i < w2.n; i++) {
        int r = rx(kind, o, w2.bit[i]);
        sh = fnv_upd(sh, o, sz);
        if (i == w2.n - 1) { if (r != 1) h_die3("the second frame did not complete (kind %d)", kind); COV(CV_R_FRAME); COV(CV_R_RET1); }
        else if (r != 0) h_die3("second frame bit %u returned %d (kind %d)", i, r, kind);
      }
      (void)ran;
      if (dmg) { if (dmgpos == 17 || dmgpos == 34 || dmgpos == 51) COV(CV_R_STARTBAD); else COV(CV_R_CRCFAIL); }
      h_emit(0, "rx stream kind %d dmg %d mb %d hash %016llx obj %016llx", kind, dmg, markerbad, (unsigned long long)sh, (unsigned long long)fnv_upd(FNV0, o, sz));
    } else {
      /* ---- random object states and random bits (blob vs port; every state incl. the default, indices that stay inside the object) */
      unsigned n = 30 + h_rmod(200), i;
      uint64_t hh = FNV0;
      COV(CV_R_FUZZ);
      for (step = 0; step < (int)n; step++) {
        uint32_t *st;
        int bit;
        if (step == 0 || h_rpct(25)) {
          unsigned s = h_rpct(8) ? 10 + h_rmod(1000) : h_rmod(kind == 1 ? 10 : 9);
          if (h_rpct(3)) s = 0x80000000u + h_rmod(5);
          st = kind == 0 ? &((V90JdObj *)o)->state_ : kind == 1 ? &((V92JdObj *)o)->stateJd_ : &((V92JdObj *)o)->statePhase_;
          *st = s;
          ((uint8_t *)o)[1] = (uint8_t)(h_rpct(50) ? (kind == 1 ? (s == 9 ? 0x30 + h_rmod(6) : h_rmod(0x31)) : h_rmod(0x31)) : h_rmod(0x40));
          ((uint8_t *)o)[0] = (uint8_t)(h_rpct(50) ? h_rmod(20) : h_rnd());
          if (s <= 9) { if (kind == 0 && s == 0) COV(CV_R_STATE0); }
          if (h_rpct(30) && kind != 0) ((V92JdObj *)o)->phase_[0x1c] = (uint8_t)h_rmod(3);
          if (h_rpct(30)) ((uint8_t *)o)[0x1e] = (uint8_t)h_rmod(3);
        }
        bit = h_rpct(60) ? (int)h_rmod(2) : (h_rpct(70) ? 0 : (int)h_rnd());
        {
          int r = rx(kind, o, bit);
          hh = fnv_upd(hh, o, sz);
          hh = fnv_upd(hh, &r, sizeof r);
          st = kind == 0 ? &((V90JdObj *)o)->state_ : kind == 1 ? &((V92JdObj *)o)->stateJd_ : &((V92JdObj *)o)->statePhase_;
          { uint32_t s2 = *st; if (s2 == 0) COV(CV_R_STATE0); else if (s2 == 1) COV(CV_R_STATE1); else if (s2 == 2) COV(CV_R_STATE2); else if (s2 == 3) COV(CV_R_STATE3); else if (s2 == 4) COV(CV_R_STATE4);
            else if (s2 == 5) COV(CV_R_STATE5); else if (s2 == 6) COV(CV_R_STATE6); else if (s2 == 7) COV(CV_R_STATE7); else if (s2 == 8) COV(CV_R_STATE8); else if (s2 == 9 && v92) COV(CV_R_STATE9); else COV(CV_R_STATEBIG); }
          if (r) COV(CV_R_RET1);
          if (((uint8_t *)o)[1] == 0x34) COV(CV_R_WRAP);
        }
        /* keep the receive index inside the object */
        if (((uint8_t *)o)[1] > 0x40 && h_rpct(60)) ((uint8_t *)o)[1] = (uint8_t)h_rmod(0x30);
        (void)i;
      }
      h_emit(0, "rx fuzz kind %d hash %016llx", kind, (unsigned long long)hh);
    }
    canary_check(o, sz, "Jd receiver");
    if (kind == 0) DUMP90("v90rx", o); else DUMP92("v92rx", o);
    h_trial_end("rx");
  }
  /* closure: the frames the TX builders make are valid receiver input (rate bits above 21 are not sent by V92Jd) */
  {
    int t;
    for (t = 0; t < 400; t++) {
      int kind = v92 ? kmap[t & 1] : 0, variant = kind == 0 ? 0 : (kind == 1 ? 1 : 2);
      size_t sz = objsz(kind);
      uint8_t *txo, *rxo, *tb;
      unsigned i;
      V90ParObj *par;
      h_pool_reset();
      txo = (uint8_t *)onew_canary(sz);
      rxo = (uint8_t *)onew_canary(sz);
      par = (V90ParObj *)onew(sizeof(V90ParObj));
      garbage_fill(par, sizeof *par, h_rnd());
      par->digitalRateMask = (int32_t)h_rnd(); par->v92DigitalRateMask = par->digitalRateMask;
      par->v34Phase4Constellation = (int32_t)h_rnd(); par->v34RrnConstellation = (int32_t)h_rnd();
      par->maxSpectralShaperLookahead = (int32_t)h_rmod(4); par->v92MaxSpectralShaperLookahead = par->maxSpectralShaperLookahead;
      par->v92JdPhase = (float)h_rmod(65536) / 65536.0f;
      garbage_fill(txo, sz, h_rnd());
      garbage_fill(rxo, sz, h_rnd());
      if (kind == 0) v90_c1(txo, par); else v92_c1(txo, par);
      if (kind == 0) { v90_c1(rxo, par); v90_unreset(rxo); } else { v92_c1(rxo, par); if (kind == 1) v92_unreset(rxo); else v92_unphreset(rxo); }
      if (kind == 2) { ((V92JdObj *)txo)->phase_[0x30] = (uint8_t)h_rmod(2); ((V92JdObj *)txo)->phase_[0x31] = (uint8_t)h_rmod(2); }
      if (kind == 0) { ((V90JdObj *)txo)->bits_[0x2f] = (uint8_t)h_rmod(2); ((V90JdObj *)txo)->bits_[0x30] = (uint8_t)h_rmod(2); }
      if (kind == 1) { ((V92JdObj *)txo)->phase_[0x30] = 0; }
      tb = kind == 0 ? v90_getbv(txo) : kind == 1 ? v92_getbv(txo) : v92_getphbv(txo);
      for (i = 0; i < 0x48; i++) {
        int r = rx(kind, rxo, tb[i]);
        if (i == 0x47) { if (r != 1) h_die3("closure: TX frame (kind %d) was not accepted at its last bit", kind); COV(CV_R_CLOSURE); }
        else if (r != 0) h_die3("closure: TX frame bit %u returned %d (kind %d)", i, r, kind);
      }
      /* the receiver decodes what the transmitter sent */
      if (kind == 0) EXPECT_EQ("closure rate mask", (uint32_t)v90_getrm(rxo), (uint32_t)par->digitalRateMask & 0x0fffffffu);
      if (kind == 1) EXPECT_EQ("closure rate mask", (uint32_t)v92_getrm(rxo), (uint32_t)par->v92DigitalRateMask & 0x003fffffu);
      (void)variant;
      h_emit(0, "closure kind %d hash %016llx", kind, (unsigned long long)fnv_upd(FNV0, rxo, sz));
      h_trial_end("closure");
    }
  }
}

/* ================================================================================================ scenario 4: the four free functions */
static void sc_free(void) {
  int trial, k;
  cov_def(CV_F_CONS, "V92createConstellations"); cov_def(CV_F_FILT, "V92createFilterCoefficients"); cov_def(CV_F_DELC, "V92deleteConstellations"); cov_def(CV_F_DELF, "V92deleteFilterCoefficients");
  cov_def(CV_F_NULLS, "delete with some NULL pointers"); cov_def(CV_F_ALLNULL, "delete with all pointers NULL"); cov_def(CV_F_FAIL, "create with a failing malloc"); cov_def(CV_F_PARTIAL, "delete of a partially filled block");
  h_scenario_begin(1);
  relax_reserved();
  for (trial = 0; trial < 600; trial++) {
    MapPar92 *p;
    unsigned fail = 0;
    uint8_t pre[sizeof(MapPar92)];
    unsigned nfree0;
    h_pool_reset();
    p = (MapPar92 *)onew_canary(sizeof(MapPar92));
    /* member-wise garbage (the pointer members move on x86-64: a byte-wise fill would differ between the 32-bit golden and the 64-bit run) */
    p->w0_ = (int32_t)h_rnd(); p->a_ = h_rnd(); p->b_ = h_rnd(); p->c_ = h_rnd(); p->trellis_ = (int32_t)h_rnd(); p->extendEu_ = (int32_t)h_rnd();
    { uint32_t gb = h_rnd(); memcpy(&p->gain_, &gb, 4); }
    for (k = 0; k < 12; k++) p->m_[k] = h_rnd();
    for (k = 0; k < 4; k++) p->cnt_[k] = h_rnd();
    for (k = 0; k < 6; k++) p->lc_[k] = h_rnd();
    for (k = 0; k < 6; k++) p->idxc_[k] = h_rnd();
    for (k = 0; k < 4; k++) p->arr_[k] = NULL;
    for (k = 0; k < 6; k++) p->lcp_[k] = NULL;
    if (trial % 4 == 0 || trial % 4 == 1) {
      /* create: malloc order, sizes, the pointers stored (also when malloc fails: no check) */
      int cons = trial % 4 == 0;
      unsigned nm0 = (unsigned)h_nmalloc;
      if (h_rpct(25)) { fail = 1 + h_rmod(cons ? 6 : 4); h_fail_malloc_n = fail; COV(CV_F_FAIL); }
      if (cons) { mk_cons(p); COV(CV_F_CONS); } else { mk_filt(p); COV(CV_F_FILT); }
      h_fail_malloc_n = 0;
      EXPECT_EQ("number of mallocs", (unsigned)h_nmalloc - nm0, (cons ? 6u : 4u) - (fail ? 1u : 0u));
      for (k = 0; k < (cons ? 6 : 4); k++) {
        void *q = cons ? p->lcp_[k] : p->arr_[k];
        if ((unsigned)k + 1 == fail) { if (q != NULL) h_die3("failed malloc %d stored a non-NULL pointer", k); }
        else {
          HAl *a = h_find_al(q, 0);
          if (!a) h_die3("%s[%d] is not a block", cons ? "lcp_" : "arr_", k);
          EXPECT_EQ("block size", a->size, cons ? 0x200u : 0x600u);
          if (k > 0) { HAl *b = h_find_al(cons ? p->lcp_[k - 1] : p->arr_[k - 1], 0); if (b && a->id != b->id + 1 && (unsigned)k != fail) h_die3("allocation ids are not consecutive"); }
        }
      }
    } else {
      /* delete: allocate some blocks (all, or with NULL holes), call, every non-NULL block freed, pointers left alone */
      int cons = trial % 4 == 2, n = cons ? 6 : 4, nn = 0, allnull = h_rpct(8);
      for (k = 0; k < n; k++) {
        void *q = (allnull || h_rpct(30)) ? NULL : sysdep_malloc(cons ? 0x200u : 0x600u);
        if (cons) p->lcp_[k] = (uint32_t *)q ? q : NULL; else p->arr_[k] = q;
        if (q) nn++;
      }
      if (allnull) COV(CV_F_ALLNULL); else if (nn < n) COV(CV_F_NULLS);
      if (nn && nn < n) COV(CV_F_PARTIAL);
      memcpy(pre, p, sizeof pre);
      nfree0 = (unsigned)h_nfree;
      if (cons) { rm_cons(p); COV(CV_F_DELC); } else { rm_filt(p); COV(CV_F_DELF); }
      EXPECT_EQ("number of frees", (unsigned)h_nfree - nfree0, (unsigned)nn);
      if (memcmp(pre, p, sizeof pre)) h_die3("delete changed the block (the pointers are not cleared)");
      for (k = 0; k < n; k++) { void *q = cons ? p->lcp_[k] : p->arr_[k]; if (q) { HAl *a = h_find_al(q, 0); if (!a || !a->freed) h_die3("block %d was not freed", k); } }
    }
    h_dump_obj(h_sample(&g_dumpctr, 8, 17), "map92", p, g_fm, g_nm);
    canary_check(p, sizeof(MapPar92), "V92MappingParams");
    /* release what the trial still owns (frees are logged too: identical in the blob and the port) */
    h_trial_end("free");
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P14_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P14_DBG")) g_show = 1;
  p07_layout_check();
  p11_layout_check();
  p14_layout_check();
  h_x87_check();
  h_arena_init();
  {
    int i, n = 0;
    for (i = 0; i < NF_V90JdObj; i++) { HFld e = f_V90JdObj[i]; if (e.kind == 'b' && e.count > 64) e.kind = 'x'; g_f90[n++] = e; }
    g_n90 = n;
    n = 0;
    for (i = 0; i < NF_V92JdObj; i++) { HFld e = f_V92JdObj[i]; if (e.kind == 'b' && e.count > 64) e.kind = 'x'; g_f92[n++] = e; }
    g_n92 = n;
    n = 0;
    for (i = 0; i < NF_MapPar92; i++) { HFld e = f_MapPar92[i]; if (e.kind == 'p' && (!strcmp(e.name, "arr_") || !strcmp(e.name, "lcp_"))) e.kind = 'o'; g_fm[n++] = e; }
    g_nm = n;
  }
  switch (sc) {
    case 0: sc_static(0); break;
    case 1: sc_rx(0); break;
    case 2: sc_static(1); break;
    case 3: sc_rx(1); break;
    case 4: sc_free(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu", h_nmalloc, h_nfree);
  h_cov_finish();
  return 0;
}
