/*
 * Differential test harness for port chunk P09 (src/vpcm/v90_mp.cpp): class V90MP of dsplibs.o - the V.90 phase-4 MP frame holder / bit-serial codec
 * (16 functions: C1 C2 D1 D2, resetCRC calcCRC evaluateCRC resetDetector reset getBitVector evaluateInfo calcSequenceLength infoToBits PrintBase2 bitsToInfo
 * printNofRecievedMpMpNot).
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/v90_mp.sh against the blob (dsplibs.o itself: `dsplibs_debug_printf` is the harness'
 * recorder), against the port (-O0 / -O2) and for x86-64 (diffed against the 32-bit BLOB's output = the golden, test/golden/vpcm/v90_mp.blob32.out).  The functions are reached
 * through __asm__ labels with the mangled names (identical in blob and port, C1 AND C2 / D1 AND D2); the object layout is a C mirror struct GENERATED from
 * test/vpcm/v90_mp.layout.  V90MP has no pointer member: its layout is identical on i386 and x86-64.
 *
 * What is compared: every return value, the whole object after every call (a raw FNV hash of the 0x124 bytes per call, the field-wise dump sampled), every
 * `dsplibs_debug_printf` call (text; recorder with a hook that changes dsplibs_debug_level between the prints of one call: the blob re-reads it).  INDEPENDENT MODELS
 * (written from the frame definition: a bit-position table of every field, a standard 16-bit CRC-CCITT register, the V.34 style start-bit grid), checked in every
 * scenario that has one (exit 3 on a violation): the encoder model (info -> expected bit positions incl. the blob's vendor bug for type 0), the decoder model (buffer ->
 * expected info, garbage bytes included), the CRC model, the PrintBase2 string model, and the end-to-end receive model (a model-built frame fed bit by bit must give MP /
 * MPnot at the last bit with the right info and counters; a flipped data bit gives "bad CRC"; the flip of bit 0x70 is repaired by the blob's retry).
 *
 * Scenarios (each its own process): 0 CRC functions (calcCRC / evaluateCRC / resetCRC) vs the CRC model, 1 evaluateInfo vs the decoder model, 2 infoToBits vs the
 * encoder model (+ round trip), 3 bitsToInfo: model-built frames, damaged frames, noise, Ed, level steering, 4 bitsToInfo on fully random object states (every case of the
 * jump table, the default case, writes through widx_ into the CRC / D region), 5 constructors / reset / resetDetector / D1 / D2 / getBitVector / calcSequenceLength /
 * PrintBase2 / printNofRecievedMpMpNot.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "v90_mp_layout.h"

#define L(m) __asm__(m)

extern void mp_c1(void *) L("_ZN5V90MPC1Ev");
extern void mp_c2(void *) L("_ZN5V90MPC2Ev");
extern void mp_d1(void *) L("_ZN5V90MPD1Ev");
extern void mp_d2(void *) L("_ZN5V90MPD2Ev");
extern void mp_resetcrc(void *) L("_ZN5V90MP8resetCRCEv");
extern void mp_calccrc(void *) L("_ZN5V90MP7calcCRCEv");
extern int mp_evalcrc(void *) L("_ZN5V90MP11evaluateCRCEv");
extern void mp_resetdet(void *) L("_ZN5V90MP13resetDetectorEv");
extern void mp_reset(void *) L("_ZN5V90MP5resetEv");
extern uint8_t *mp_getbv(void *, unsigned *) L("_ZN5V90MP12getBitVectorERj");
extern void mp_evalinfo(void *) L("_ZN5V90MP12evaluateInfoEv");
extern void mp_calcseq(void *) L("_ZN5V90MP18calcSequenceLengthEv");
extern void mp_i2b(void *) L("_ZN5V90MP10infoToBitsEv");
extern void mp_pb2(void *, char *, unsigned long, unsigned short) L("_ZN5V90MP10PrintBase2EPcmt");
extern int mp_b2i(void *, int) L("_ZN5V90MP10bitsToInfoEi");
extern void mp_prnof(void *) L("_ZN5V90MP23printNofRecievedMpMpNotEv");
extern unsigned int dsplibs_debug_level;

#define OBJSZ 0x124u
#define BITS_OFF 0x1c

/* ------------------------------------------------------------------------------------------------ recorder (dsplibs_debug_printf) */
static unsigned g_ndbg;
static char g_dbg_text[64][300];   /* the texts of the prints of the current call window */
static unsigned g_dbg_in_call;
static int g_dbg_show;
typedef void (*dbg_hook_t)(unsigned idx);
static dbg_hook_t g_dbg_hook;
static unsigned g_plan[16], g_plan_len;
static int g_plan_on;
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
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
  if (g_dbg_in_call < 64) { g_dbg_text[g_dbg_in_call][0] = 0; esc_cat(g_dbg_text[g_dbg_in_call], 300, buf); }
  h_emit(g_dbg_show, "dbgprintf %s", g_dbg_in_call < 64 ? g_dbg_text[g_dbg_in_call] : "(many)");
  g_dbg_in_call++;
  g_ndbg++;
  if (g_plan_on && g_plan_len) dsplibs_debug_level = g_plan[g_dbg_in_call - 1 < g_plan_len ? g_dbg_in_call - 1 : g_plan_len - 1];
  if (g_dbg_hook) g_dbg_hook(g_dbg_in_call - 1);
  return 0;
}

static void *onew(size_t n) { void *p = h_obj_new(n); memset(p, 0xa5, n); return p; }
static uint64_t fnv_blk(const void *p, size_t n) {
  const uint8_t *q = (const uint8_t *)p;
  uint64_t h = 0xcbf29ce484222325ull;
  size_t i;
  for (i = 0; i < n; i++) { h ^= q[i]; h *= 0x100000001b3ull; }
  return h;
}
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)

static const char *g_covname[H_COV_MAX];
static void cov_def(int id, const char *name) { h_cov_define(id, name); g_covname[id] = name; }
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }
#define COV(id) h_cov_hit(id)

static unsigned long g_dumpctr;
#define DUMP(tag, obj) h_dump_obj(h_sample(&g_dumpctr, 12, 211), (tag), (obj), f_MpObj, NF_MpObj)
static uint8_t *bitsp(void *o) { return (uint8_t *)o + BITS_OFF; }   /* the buffer, addressed by raw offset (byte layout is the same on every architecture) */
static MpObj *MO(void *o) { return (MpObj *)o; }

/* ------------------------------------------------------------------------------------------------ models */
typedef struct { uint8_t type, rate, trellis, nonLin, shaping, cPack; uint16_t mask; int16_t h[6]; } Info;

static const int H_BASE[6] = { 0x34, 0x45, 0x56, 0x67, 0x78, 0x89 };

typedef struct { uint8_t exp[512]; uint8_t spec[512]; unsigned total, frameLen; } Mdl;
static void mput(Mdl *m, int pos, unsigned v) { m->exp[pos] = (uint8_t)v; m->spec[pos] = 1; }

/* standard bit-serial CRC-CCITT (0x1021, preset 0xffff, MSB first) over the data bits p = 0x12 .. endp - 1 except the start-bit grid (p % 17 == 0) */
static void model_crc(const uint8_t *b, unsigned endp, uint8_t out[16]) {
  uint16_t r = 0xffff;
  unsigned p, k;
  for (p = 0x12; p < endp; p++) {
    unsigned fb;
    if (p % 17 == 0) continue;
    fb = ((r >> 15) ^ (b[p] & 1u)) & 1u;
    r = (uint16_t)(r << 1);
    if (fb) r ^= 0x1021;
  }
  for (k = 0; k < 16; k++) out[k] = (uint8_t)((r >> (15 - k)) & 1);
}

/* info -> the bits infoToBits writes (vendor_bug = 1: the blob's type-0 branch wipes the CRC it copied; 0: the intended frame, used to BUILD receive streams) */
static void model_encode(const Info *in, unsigned D, int vendor_bug, const uint8_t *pre, Mdl *m) {
  unsigned type = in->type, k, p, v, q;
  uint8_t crc[16];
  int j;
  memset(m, 0, sizeof *m);
  for (p = 0; p <= 16; p++) mput(m, (int)p, 1);
  mput(m, 17, 0);
  mput(m, 0x12, type);
  for (p = 0x13; p <= 0x17; p++) mput(m, (int)p, 0);
  for (k = 0; k < 4; k++) mput(m, 0x18 + (int)k, (in->rate >> k) & 1u);
  mput(m, 0x1c, 0);
  if (in->trellis <= 3) { mput(m, 0x1d, in->trellis & 1u); mput(m, 0x1e, (in->trellis >> 1) & 1u); }
  else if (pre) { m->exp[0x1d] = pre[0x1d]; m->exp[0x1e] = pre[0x1e]; }   /* untouched by the blob: the CRC covers the old bits */
  mput(m, 0x1f, in->nonLin);
  mput(m, 0x20, in->shaping);
  mput(m, 0x21, in->cPack);
  mput(m, 0x22, 0);
  mput(m, 0x23, 0);
  for (k = 0; k < 14; k++) mput(m, 0x24 + (int)k, (in->mask >> k) & 1u);
  mput(m, 0x32, 0);
  mput(m, 0x33, 0);
  m->frameLen = type == 0 ? 0x55 : 0xbb;
  v = m->frameLen + 1;
  q = v / D;
  m->total = (v == q * D) ? (v & 255u) : (((q + 1) * D) & 255u);
  if (type != 0) {
    for (j = 0; j < 6; j++) {
      for (k = 0; k < 16; k++) mput(m, H_BASE[j] + (int)k, ((uint16_t)in->h[j] >> k) & 1u);
    }
    for (p = 0x44; p <= 0x99; p += 17) mput(m, (int)p, 0);
    for (p = 0x9a; p <= 0xaa; p++) mput(m, (int)p, 0);
    model_crc(m->exp, 0xaa, crc);
    for (k = 0; k < 16; k++) mput(m, 0xab + (int)k, crc[k]);
    mput(m, 0xbb, 0);
    for (p = 0xbc; p < m->total; p++) mput(m, (int)p, 0);
  } else {
    for (p = 0x34; p <= 0x44; p++) mput(m, (int)p, 0);
    model_crc(m->exp, 0x44, crc);
    for (k = 0; k < 16; k++) mput(m, 0x45 + (int)k, crc[k]);
    if (vendor_bug) {
      mput(m, 0x45, 0);
      for (p = 0x45; p < m->total; p++) mput(m, (int)p, 0);
    } else {
      mput(m, 0x55, 0);
      for (p = 0x56; p < m->total; p++) mput(m, (int)p, 0);
    }
  }
}

/* the buffer -> info (decoder model; the buffer may hold garbage bytes) */
static void model_gather(const uint8_t *b, unsigned ft, Info *o) {
  unsigned k;
  int j;
  memset(o, 0, sizeof *o);
  o->type = (uint8_t)ft;
  for (k = 0; k < 4; k++) o->rate |= (uint8_t)((b[0x18 + k] & 1u) << k);
  for (k = 0; k < 14; k++) if (b[0x24 + k] != 0) o->mask |= (uint16_t)(1u << k);
  o->trellis = (uint8_t)((b[0x1d] & 1u) | ((b[0x1e] & 1u) << 1));
  o->nonLin = b[0x1f];
  o->shaping = b[0x20];
  o->cPack = b[0x21];
  if (ft != 0)
    for (j = 0; j < 6; j++) {
      unsigned v = 0;
      for (k = 0; k < 16; k++) v |= (b[H_BASE[j] + k] & 1u) << k;
      o->h[j] = (int16_t)v;
    }
}

static void get_info(const MpObj *o, Info *i) {
  i->type = o->type_; i->rate = o->rate_; i->trellis = o->trellis_; i->nonLin = o->nonLin_; i->shaping = o->shaping_; i->cPack = o->cPack_;
  i->mask = o->rateMask_;
  i->h[0] = (int16_t)o->h1Re_; i->h[1] = (int16_t)o->h1Im_; i->h[2] = (int16_t)o->h2Re_; i->h[3] = (int16_t)o->h2Im_; i->h[4] = (int16_t)o->h3Re_; i->h[5] = (int16_t)o->h3Im_;
}
static void set_info(MpObj *o, const Info *i) {
  o->type_ = i->type; o->rate_ = i->rate; o->trellis_ = i->trellis; o->nonLin_ = i->nonLin; o->shaping_ = i->shaping; o->cPack_ = i->cPack;
  o->rateMask_ = i->mask;
  o->h1Re_ = (uint16_t)i->h[0]; o->h1Im_ = (uint16_t)i->h[1]; o->h2Re_ = (uint16_t)i->h[2]; o->h2Im_ = (uint16_t)i->h[3]; o->h3Re_ = (uint16_t)i->h[4]; o->h3Im_ = (uint16_t)i->h[5];
}
static int info_eq(const Info *a, const Info *b) { return !memcmp(a, b, sizeof *a); }

static uint8_t rbyte_kind(void) {   /* a byte with interesting classes */
  switch (h_rmod(8)) {
    case 0: return 0;
    case 1: return 1;
    case 2: return 0xff;
    case 3: return 0x80;
    case 4: return (uint8_t)(1 + h_rmod(3));
    default: return (uint8_t)h_rnd();
  }
}
static void rnd_info(Info *in, int legit) {
  int j;
  if (legit) {
    in->type = (uint8_t)h_rmod(2);
    in->rate = (uint8_t)h_rmod(16);
    in->trellis = (uint8_t)h_rmod(4);
    in->nonLin = (uint8_t)h_rmod(2);
    in->shaping = (uint8_t)h_rmod(2);
    in->cPack = (uint8_t)h_rmod(2);
    in->mask = (uint16_t)(h_rnd() & 0x3fffu);
  } else {
    in->type = h_rpct(60) ? (uint8_t)h_rmod(2) : rbyte_kind();
    in->rate = rbyte_kind();
    in->trellis = rbyte_kind();
    in->nonLin = rbyte_kind();
    in->shaping = rbyte_kind();
    in->cPack = rbyte_kind();
    in->mask = (uint16_t)h_rnd();
  }
  for (j = 0; j < 6; j++) {
    switch (h_rmod(6)) {
      case 0: in->h[j] = 0; break;
      case 1: in->h[j] = -1; break;
      case 2: in->h[j] = (int16_t)0x8000; break;
      case 3: in->h[j] = 0x7fff; break;
      default: in->h[j] = (int16_t)h_rnd(); break;
    }
  }
}

/* coverage ids (per scenario; 60 / 61 are reserved by the library) */
enum { CV_CRC_CALC = 0, CV_CRC_EVAL_OK, CV_CRC_EVAL_BAD, CV_CRC_T0, CV_CRC_T1, CV_CRC_TOTHER, CV_CRC_RESET, CV_CRC_GARB, CV_CRC_SUM256, CV_CRC_FLEN_SMALL, CV_CRC_BIGBYTE };
enum { CV_EI_T0 = 0, CV_EI_T1, CV_EI_TOTHER, CV_EI_GARB, CV_EI_MASKBYTE, CV_EI_BIGBYTE, CV_EI_ALL1, CV_EI_ALL0 };
enum { CV_IB_T0 = 0, CV_IB_T1, CV_IB_TOTHER, CV_IB_TR0, CV_IB_TR1, CV_IB_TR2, CV_IB_TR3, CV_IB_TRBAD, CV_IB_TRNEG, CV_IB_D1, CV_IB_DBIG, CV_IB_DEXACT, CV_IB_TRUNC, CV_IB_ROUNDTRIP, CV_IB_GARBINFO,
       CV_IB_NEGRATE, CV_IB_MASKNEG, CV_IB_PRE, CV_IB_T0_CRCWIPE, CV_IB_OVER };
enum { CV_RX_MP_T0 = 0, CV_RX_MP_T1, CV_RX_MPNOT_T0, CV_RX_MPNOT_T1, CV_RX_BADCRC, CV_RX_MODGOOD, CV_RX_BADCRC_T0, CV_RX_ED_IDLE, CV_RX_ED_BUSY, CV_RX_ABORT1, CV_RX_NOISE, CV_RX_CNT3, CV_RX_LVL0,
       CV_RX_LVL1, CV_RX_LVL2, CV_RX_LVL3, CV_RX_LVL_BIG, CV_RX_HOOK, CV_RX_D0, CV_RX_DBIG, CV_RX_BACK2BACK, CV_RX_PRNOT, CV_RX_ED_FLIP, CV_RX_PARTIAL };
enum { CV_RF_S0 = 0, CV_RF_S1, CV_RF_S2, CV_RF_S3, CV_RF_S4, CV_RF_SDEF, CV_RF_ED, CV_RF_BIT0, CV_RF_BIT1, CV_RF_WIDX_HI, CV_RF_FLIP70, CV_RF_RET1, CV_RF_RET2, CV_RF_RET3, CV_RF_MODGOOD, CV_RF_BADCRC,
       CV_RF_FLEN_SMALL, CV_RF_TYPE0, CV_RF_TYPE1 };
enum { CV_MC_C1 = 0, CV_MC_C2, CV_MC_D1, CV_MC_D2, CV_MC_RESET, CV_MC_RESETDET, CV_MC_GETBV, CV_MC_SEQ, CV_MC_SEQ_EXACT, CV_MC_SEQ_ROUND, CV_MC_SEQ_TRUNC, CV_MC_PB2, CV_MC_PB2_0, CV_MC_PB2_BIG, CV_MC_PB2_ZEROV,
       CV_MC_PRNOF, CV_MC_PRNOF_OFF, CV_MC_RESETCRC, CV_MC_PB2_33 };

/* ================================================================================================ scenario 0: the CRC functions */
static void sc_crc(void) {
  int trial, k;
  cov_def(CV_CRC_CALC, "calcCRC"); cov_def(CV_CRC_EVAL_OK, "evaluateCRC == 1"); cov_def(CV_CRC_EVAL_BAD, "evaluateCRC == 0"); cov_def(CV_CRC_T0, "type 0 (bound 0x44)");
  cov_def(CV_CRC_T1, "type 1 (bound 0xaa)"); cov_def(CV_CRC_TOTHER, "type byte other"); cov_def(CV_CRC_RESET, "resetCRC"); cov_def(CV_CRC_GARB, "garbage buffer / CRC bytes");
  cov_def(CV_CRC_SUM256, "evaluateCRC accepted by a sum of 256"); cov_def(CV_CRC_FLEN_SMALL, "frameLen_ < 16 (compare window before the buffer)"); cov_def(CV_CRC_BIGBYTE, "buffer byte > 1");
  h_covreq[CV_CRC_SUM256] = 0;   /* seen if it occurs; the sum-of-256 acceptance is a property of the blob, also pinned by the targeted trials below */
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 6000; trial++) {
    void *o; h_pool_reset(); o = onew(OBJSZ);
    uint8_t *b = bitsp(o);
    unsigned kind = h_rmod(8);
    unsigned type;
    Info in;
    Mdl m;
    uint8_t crcm[16];
    MpObj *mo = MO(o);
    int r;
    mp_c1(o);
    /* buffer: a model frame (0/1 bits), possibly damaged, or garbage */
    rnd_info(&in, 1);
    type = in.type;
    model_encode(&in, 1 + h_rmod(6), 0, NULL, &m);
    for (k = 0; k < 256 && k < 0xe6; k++) b[k] = m.spec[k] ? m.exp[k] : (uint8_t)h_rmod(2);
    mo->frameType_ = (uint8_t)type;
    mo->frameLen_ = (uint8_t)m.frameLen;
    mo->totalLen_ = (uint8_t)m.total;
    for (k = 0; k < 16; k++) mo->crc_[k] = (uint8_t)h_rnd();
    if (kind == 1 && type) { b[0x12 + h_rmod(60)] ^= 1; }              /* damaged data bit (may hit a start-bit position: the grid ignores it) */
    if (kind == 2) { for (k = 0; k < 0xb0; k++) b[k] = (uint8_t)h_rnd(); COV(CV_CRC_GARB); }
    if (kind == 3) { for (k = 0; k < 0xb0; k++) b[k] = rbyte_kind(); mo->frameLen_ = (uint8_t)h_rnd(); COV(CV_CRC_GARB); if (mo->frameLen_ < 16) COV(CV_CRC_FLEN_SMALL); }
    if (kind == 4) { mo->frameType_ = (uint8_t)(1 + h_rmod(255)); }
    if (kind == 5) { for (k = 0; k < 0xb0; k++) if (b[k] > 1) COV(CV_CRC_BIGBYTE); b[0x20 + h_rmod(40)] = (uint8_t)(2 + h_rmod(200)); COV(CV_CRC_BIGBYTE); }
    if (mo->frameType_ == 0) COV(CV_CRC_T0); else if (mo->frameType_ == 1) COV(CV_CRC_T1); else COV(CV_CRC_TOTHER);
    {
      uint8_t save[OBJSZ];
      memcpy(save, o, OBJSZ);
      /* calcCRC from a random register */
      mp_calccrc(o);
      COV(CV_CRC_CALC);
      h_emit(0, "calcCRC crc %016llx", (unsigned long long)fnv_blk((uint8_t *)o + 0x102, 16));
      DUMP("calc", o);
      /* the model: register all ones is not what calcCRC does (it continues from the object's register): only the preset + calc pair is model-checked below */
      memcpy(o, save, OBJSZ);
    }
    mp_resetcrc(o);
    COV(CV_CRC_RESET);
    for (k = 0; k < 16; k++) if (mo->crc_[k] != 1) h_die3("resetCRC left crc[%d] = %d", k, mo->crc_[k]);
    mp_calccrc(o);
    {
      /* preset + calcCRC = the model CRC register (for 0/1 buffers, type byte 0 / non-zero) */
      int clean = 1;
      for (k = 0; k < 0xb0; k++) if (b[k] > 1) clean = 0;
      if (clean) {
        model_crc(b, mo->frameType_ == 0 ? 0x44u : 0xaau, crcm);
        for (k = 0; k < 16; k++) if (mo->crc_[k] != crcm[k]) h_die3("calcCRC: stage %d = %d, model %d (type 0x%x)", k, mo->crc_[k], crcm[k], mo->frameType_);
      }
    }
    r = mp_evalcrc(o);
    h_emit(0, "evalCRC r=%d crc %016llx", r, (unsigned long long)fnv_blk((uint8_t *)o + 0x102, 16));
    DUMP("eval", o);
    if (r) COV(CV_CRC_EVAL_OK); else COV(CV_CRC_EVAL_BAD);
    if (kind == 0 && !(type == 0 && 0)) {
      /* an undamaged model frame (correct CRC bits at frameLen - 16 ..): evaluateCRC must say 1 (type 0 and 1 both: the model frame here is the INTENDED frame) */
      if (!r) h_die3("evaluateCRC rejected a model frame (type %u)", type);
    }
    if (kind == 1 && type && r && 0) h_die3("unreachable");
  }
  /* targeted: the accumulator is 8 bits wide, a register that differs from the received bits by a total of exactly 256 is accepted (all stages differ by 16: 16 * 16 = 256) */
  for (k = 0; k < 8; k++) {
    void *o; h_pool_reset(); o = onew(OBJSZ);
    uint8_t *b = bitsp(o);
    MpObj *mo = MO(o);
    Info in;
    Mdl m;
    int i, r;
    mp_c1(o);
    rnd_info(&in, 1);
    in.type = (uint8_t)(k & 1);
    model_encode(&in, 1, 0, NULL, &m);
    for (i = 0; i < 0xb0; i++) b[i] = m.spec[i] ? m.exp[i] : 0;
    mo->frameType_ = in.type;
    mo->frameLen_ = (uint8_t)m.frameLen;
    /* make every received CRC bit differ from the register by exactly 16 (garbage bytes 16 / 17): the sum is 256 -> 0 */
    {
      uint8_t crcm[16];
      model_crc(b, in.type ? 0xaau : 0x44u, crcm);
      for (i = 0; i < 16; i++) b[m.frameLen - 16 + i] = (uint8_t)(crcm[i] + 16);
    }
    r = mp_evalcrc(o);
    h_emit(1, "sum256 trial %d r=%d", k, r);
    if (r) COV(CV_CRC_SUM256);
    if (!r) h_die3("evaluateCRC: a total difference of 256 must be accepted (8-bit sum)");
    DUMP("sum256", o);
  }
}

/* ================================================================================================ scenario 1: evaluateInfo */
static void sc_evalinfo(void) {
  int trial, k;
  cov_def(CV_EI_T0, "frameType_ 0"); cov_def(CV_EI_T1, "frameType_ 1"); cov_def(CV_EI_TOTHER, "frameType_ other"); cov_def(CV_EI_GARB, "garbage buffer"); cov_def(CV_EI_MASKBYTE, "mask byte > 1 (tested != 0)");
  cov_def(CV_EI_BIGBYTE, "nonLin / shaping / cPack byte > 1"); cov_def(CV_EI_ALL1, "all ones"); cov_def(CV_EI_ALL0, "all zeros");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 12000; trial++) {
    void *o; h_pool_reset(); o = onew(OBJSZ);
    uint8_t *b = bitsp(o);
    unsigned kind = h_rmod(7);
    unsigned ft;
    Info exp, got;
    mp_c1(o);
    switch (h_rmod(4)) { case 0: ft = 0; COV(CV_EI_T0); break; case 1: ft = 1; COV(CV_EI_T1); break; case 2: ft = 1 + h_rmod(255); COV(CV_EI_TOTHER); break; default: ft = h_rmod(2); ft ? COV(CV_EI_T1) : COV(CV_EI_T0); break; }
    MO(o)->frameType_ = (uint8_t)ft;
    /* stale info fields (evaluateInfo overwrites every one) */
    { int j; MpObj *mo = MO(o); Info st; rnd_info(&st, 0); set_info(mo, &st); (void)j; }
    for (k = 0; k < 0xb0; k++) {
      switch (kind) {
        case 0: b[k] = (uint8_t)h_rmod(2); break;
        case 1: b[k] = 1; break;
        case 2: b[k] = 0; break;
        case 3: b[k] = rbyte_kind(); COV(CV_EI_GARB); break;
        case 4: b[k] = (uint8_t)(h_rpct(50) ? h_rmod(2) : 2 + h_rmod(254)); break;
        default: b[k] = (uint8_t)h_rmod(2); break;
      }
    }
    if (kind == 1) COV(CV_EI_ALL1);
    if (kind == 2) COV(CV_EI_ALL0);
    if (kind == 5) { /* mask bytes with a clear low bit but non-zero (the blob tests != 0, not & 1) and info bytes > 1 */
      for (k = 0x24; k < 0x32; k++) b[k] = (uint8_t)(h_rpct(50) ? 2 : 0);
      b[0x1f] = (uint8_t)(2 + h_rmod(250)); b[0x20] = (uint8_t)(2 + h_rmod(250)); b[0x21] = (uint8_t)(2 + h_rmod(250));
      COV(CV_EI_MASKBYTE); COV(CV_EI_BIGBYTE);
    }
    if (kind == 6) { for (k = 0x34; k < 0x9a; k++) b[k] = (uint8_t)(h_rnd()); }   /* h bits: only bit 0 of each byte counts */
    model_gather(b, ft, &exp);
    mp_evalinfo(o);
    get_info(MO(o), &got);
    if (!info_eq(&exp, &got)) h_die3("evaluateInfo model mismatch (trial %d, frameType 0x%x)", trial, ft);
    DUMP("ei", o);
    h_emit(0, "ei %d %016llx", trial, (unsigned long long)fnv_blk(o, OBJSZ));
  }
}

/* ================================================================================================ scenario 2: infoToBits */
static void sc_info2bits(void) {
  int trial, k;
  cov_def(CV_IB_T0, "type 0"); cov_def(CV_IB_T1, "type 1"); cov_def(CV_IB_TOTHER, "type > 1"); cov_def(CV_IB_TR0, "trellis 0"); cov_def(CV_IB_TR1, "trellis 1"); cov_def(CV_IB_TR2, "trellis 2");
  cov_def(CV_IB_TR3, "trellis 3"); cov_def(CV_IB_TRBAD, "trellis 4..127 (bits 0x1d / 0x1e untouched)"); cov_def(CV_IB_TRNEG, "trellis negative (signed compare: untouched)"); cov_def(CV_IB_D1, "D == 1");
  cov_def(CV_IB_DBIG, "D > 255 (8-bit truncation of the length)"); cov_def(CV_IB_DEXACT, "frameLen + 1 an exact multiple of D"); cov_def(CV_IB_TRUNC, "totalLen truncated to 8 bits");
  cov_def(CV_IB_ROUNDTRIP, "round trip checked"); cov_def(CV_IB_GARBINFO, "garbage info fields"); cov_def(CV_IB_NEGRATE, "rate byte negative"); cov_def(CV_IB_MASKNEG, "rate mask with bit 15 set");
  cov_def(CV_IB_PRE, "buffer pre-filled with garbage"); cov_def(CV_IB_T0_CRCWIPE, "type 0: CRC wiped (vendor bug) checked"); cov_def(CV_IB_OVER, "totalLen > 0xe6 (zero fill reaches the CRC / D region)");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 14000; trial++) {
    void *o; h_pool_reset(); o = onew(OBJSZ);
    uint8_t *b = bitsp(o);
    MpObj *mo = MO(o);
    Info in, back;
    Mdl m;
    unsigned D, i;
    int legit = h_rpct(55);
    uint8_t pre[256];
    mp_c1(o);
    rnd_info(&in, legit);
    if (!legit) COV(CV_IB_GARBINFO);
    if (h_rpct(10)) in.type = (uint8_t)(2 + h_rmod(254));
    switch (h_rmod(8)) {
      case 0: D = 1; COV(CV_IB_D1); break;
      case 1: D = 256 + h_rmod(2000); COV(CV_IB_DBIG); break;
      case 2: D = 86; COV(CV_IB_DEXACT); break;                                  /* 0x55 + 1 */
      case 3: D = 188; COV(CV_IB_DEXACT); break;                                 /* 0xbb + 1 */
      case 4: D = 2 + h_rmod(10); break;
      case 5: D = 1 + h_rmod(300); break;
      case 6: D = 87 + h_rmod(100); break;
      default: D = 1 + h_rmod(40); break;
    }
    if (in.type == 0 ? (86 % D == 0) : (188 % D == 0)) COV(CV_IB_DEXACT);
    mo->trn2dD_ = D;
    /* pre-fill the whole buffer region with 0 / 1 / garbage bytes: a position the blob does not write keeps its value */
    for (i = 0; i < 256; i++) { pre[i] = h_rpct(50) ? (uint8_t)h_rmod(2) : (uint8_t)h_rnd(); b[i] = pre[i]; }
    COV(CV_IB_PRE);
    for (k = 0; k < 16; k++) mo->crc_[k] = (uint8_t)h_rnd();
    /* the CRC region (0x102..) is part of `pre` addressing above: bits_ + 230 = crc_; infoToBits rewrites the crc, so only positions < 0xe6 are compared with `pre` */
    for (k = 0; k < 16; k++) pre[0xe6 + k] = mo->crc_[k];
    set_info(mo, &in);
    mo->trn2dD_ = D;   /* the pre-fill above covered 0x114 .. 0x11b (bits 0xf8..0xff): restore D */
    if (in.type == 0) COV(CV_IB_T0); else if (in.type == 1) COV(CV_IB_T1); else COV(CV_IB_TOTHER);
    switch (in.trellis) { case 0: COV(CV_IB_TR0); break; case 1: COV(CV_IB_TR1); break; case 2: COV(CV_IB_TR2); break; case 3: COV(CV_IB_TR3); break; default: if (in.trellis & 0x80) COV(CV_IB_TRNEG); else COV(CV_IB_TRBAD); }
    if ((int8_t)in.rate < 0) COV(CV_IB_NEGRATE);
    if (in.mask & 0x8000) COV(CV_IB_MASKNEG);
    {
      unsigned frameLen = in.type == 0 ? 0x55 : 0xbb, v = frameLen + 1, q = v / D;
      unsigned tot = (v == q * D) ? (v & 255u) : (((q + 1) * D) & 255u);
      if (((q + 1) * D) > 255 && v != q * D) COV(CV_IB_TRUNC);
      if (tot > 0xe6) COV(CV_IB_OVER);
    }
    {
      uint8_t save_info[OBJSZ];
      uint32_t d_before = mo->trn2dD_;
      (void)d_before;
      memcpy(save_info, o, OBJSZ);
      model_encode(&in, D, 1, pre, &m);
      mp_i2b(o);
      DUMP("i2b", o);
      h_emit(0, "i2b %d %016llx", trial, (unsigned long long)fnv_blk(o, OBJSZ));
      /* model: every specified position holds the expected bit (positions < 0xe6 are the buffer; 0xe6 .. 0xff overlay crc_ / unknown / D / lengths: model-checked only if below total and
       * the frame stays inside the buffer; otherwise blob-vs-port only), every other position of the buffer keeps its pre-value */
      if (m.total <= 0xe6) {
        for (i = 0; i < 0xe6; i++) {
          uint8_t want = m.spec[i] ? m.exp[i] : pre[i];
          if (b[i] != want) h_die3("infoToBits: bits[0x%x] = %d, model %d (type %u trellis %u D %u trial %d)", i, b[i], want, in.type, in.trellis, D, trial);
        }
        if (in.type == 0) COV(CV_IB_T0_CRCWIPE);
        EXPECT_EQ("frameType_", mo->frameType_, in.type);
        EXPECT_EQ("frameLen_", mo->frameLen_, m.frameLen);
        EXPECT_EQ("totalLen_", mo->totalLen_, m.total);
        /* the CRC register after infoToBits holds the model CRC (type != 0: the one transmitted at 0xab .., type 0: computed and copied, then wiped from the buffer) */
        {
          uint8_t crcm[16];
          model_crc(m.exp, in.type == 0 ? 0x44u : 0xaau, crcm);
          for (k = 0; k < 16; k++) if (mo->crc_[k] != crcm[k]) h_die3("infoToBits: crc_[%d] = %d, model %d", k, mo->crc_[k], crcm[k]);
        }
        /* info fields are unchanged */
        { Info now; get_info(mo, &now); if (!info_eq(&now, &in)) h_die3("infoToBits changed the info fields"); }
        /* round trip (legit domain, type 1): evaluateInfo gives the info back */
        if (legit && in.type == 1) {
          Info rt;
          mp_evalinfo(o);
          get_info(mo, &rt);
          COV(CV_IB_ROUNDTRIP);
          if (!info_eq(&rt, &in)) h_die3("infoToBits -> evaluateInfo round trip failed (trial %d)", trial);
          DUMP("rt", o);
        }
        (void)back;
      }
    }
  }
}

/* ================================================================================================ scenarios 3 / 4: bitsToInfo */
static unsigned g_levels[] = { 0, 1, 2, 3, 4, 7, 0xffffffffu, 0x80000000u };
static unsigned pick_level(void) { return g_levels[h_rmod(8)]; }
static unsigned g_hook_to, g_hook_at;
static void hook_set(unsigned idx) { if (idx == g_hook_at) { dsplibs_debug_level = g_hook_to; COV(CV_RX_HOOK); } }

typedef struct { int ret; unsigned nprints; } Step;
static Step feed(void *o, int bit, int show) {
  Step s;
  g_dbg_in_call = 0;
  s.ret = mp_b2i(o, bit);
  s.nprints = g_dbg_in_call;
  h_emit(show, "b %d r %d p %u o %016llx", bit, s.ret, s.nprints, (unsigned long long)fnv_blk(o, OBJSZ));
  return s;
}
static void feed_run(void *o, int bit, unsigned n) { while (n--) (void)feed(o, bit, 0); }

/* a model frame as a bit stream: 17 ones, the start bit, then positions 0x12 .. total - 1 of the (intended) frame */
static unsigned frame_stream(const Info *in, unsigned D, uint8_t *out) {
  Mdl m;
  unsigned n = 0, p;
  model_encode(in, D, 0, NULL, &m);
  for (p = 0; p < m.total; p++) out[n++] = m.spec[p] ? m.exp[p] : 0;
  return n;
}

static void expect_prints(unsigned level, int is_mpnot, int is_first3, unsigned got) {
  unsigned want = 0;
  if (is_first3) {
    if (level > 1) want += 1;
    if (!is_mpnot) { if (level > 2) want += 4; }
    else if (level > 1) want += 4;
  }
  if (got != want) h_die3("print count %u, model %u (level 0x%x mpnot %d first3 %d)", got, want, level, is_mpnot, is_first3);
}

static void sc_rx(void) {
  int trial, k;
  cov_def(CV_RX_MP_T0, "MP, short frame"); cov_def(CV_RX_MP_T1, "MP, long frame"); cov_def(CV_RX_MPNOT_T0, "MPnot, short"); cov_def(CV_RX_MPNOT_T1, "MPnot, long"); cov_def(CV_RX_BADCRC, "bad CRC (long)");
  cov_def(CV_RX_MODGOOD, "modified good CRC (bit 0x70 repaired)"); cov_def(CV_RX_BADCRC_T0, "bad CRC (short frame: no retry flip)"); cov_def(CV_RX_ED_IDLE, "Ed detected while idle");
  cov_def(CV_RX_ED_BUSY, "2D zeros inside a frame (no Ed)"); cov_def(CV_RX_ABORT1, "a 1 instead of the start bit"); cov_def(CV_RX_NOISE, "noise"); cov_def(CV_RX_CNT3, "third frame (counter > 2: no prints)");
  cov_def(CV_RX_LVL0, "level 0"); cov_def(CV_RX_LVL1, "level 1"); cov_def(CV_RX_LVL2, "level 2"); cov_def(CV_RX_LVL3, "level 3 / 4 / 7"); cov_def(CV_RX_LVL_BIG, "level 0xffffffff / 0x80000000 (unsigned compare)");
  cov_def(CV_RX_HOOK, "level lowered inside a print"); cov_def(CV_RX_D0, "D == 0 (noise only)"); cov_def(CV_RX_BACK2BACK, "frames back to back"); cov_def(CV_RX_PRNOT, "printNofRecievedMpMpNot");
  cov_def(CV_RX_ED_FLIP, "Ed with zeros_ wrap (256 zeros, D == 0)"); cov_def(CV_RX_PARTIAL, "partial frame then restart");
  h_covreq[CV_RX_ED_FLIP] = 0;
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 1500; trial++) {
    void *o; h_pool_reset(); o = onew(OBJSZ);
    MpObj *mo = MO(o);
    static const unsigned DSET[] = { 16, 24, 32, 43, 47, 64, 94, 188 };   /* ceil(188 / D) * D and ceil(86 / D) * D stay below 256: the length is not truncated */
    unsigned D = h_rpct(10) ? DSET[h_rmod(8)] : (1 + h_rmod(12));
    unsigned level = pick_level();
    int nframes = 1 + (int)h_rmod(5);
    int f;
    mp_c1(o);
    mo->trn2dD_ = D;
    dsplibs_debug_level = level;
    g_hook_at = 0xffffffffu; g_dbg_hook = NULL;
    if (h_rpct(25)) { g_dbg_hook = hook_set; g_hook_at = h_rmod(6); g_hook_to = h_rmod(4); }   /* lower the level after print 0 .. 5 of the call: every threshold of every print is re-read */
    if (level == 0) COV(CV_RX_LVL0); else if (level == 1) COV(CV_RX_LVL1); else if (level == 2) COV(CV_RX_LVL2); else if (level < 8) COV(CV_RX_LVL3); else COV(CV_RX_LVL_BIG);
    for (f = 0; f < nframes; f++) {
      Info in;
      uint8_t st[300];
      unsigned n, i, kind = h_rmod(10), pre;
      int exp_ret = 0, mpnot, damaged = 0, flip = -1;
      Info gather;
      unsigned nMp0 = mo->nMp_, nMpNot0 = mo->nMpNot_;
      rnd_info(&in, 1);
      if (in.type == 0) memset(in.h, 0, sizeof in.h);   /* a short frame carries no h coefficients: the decoder gives 0 */
      n = frame_stream(&in, D, st);
      mpnot = in.cPack != 0;
      /* noise before the frame (ends with a zero so that the 17 ones start a fresh run) */
      pre = h_rmod(40);
      for (i = 0; i < pre; i++) {
        Step s = feed(o, h_rpct(40) ? 1 : 0, 0);
        COV(CV_RX_NOISE);
        if (s.ret == 3) COV(CV_RX_ED_IDLE);
      }
      if (kind == 0) { /* a long zero run while idle: Ed after 2 * D zeros */
        unsigned z;
        feed_run(o, 1, 1);
        for (z = 0; z < 2 * D + 2; z++) { Step s = feed(o, 0, 0); if (s.ret == 3) COV(CV_RX_ED_IDLE); }
        continue;
      }
      feed_run(o, 0, 1);
      if (kind == 1) { /* 16 ones then the start bit: not enough ones: stays in state 0 */
        feed_run(o, 1, 16);
        feed_run(o, 0, 1);
        continue;
      }
      if (kind == 2) { /* 17 ones, then a 1 instead of the start bit: state 1 resets */
        feed_run(o, 1, 17);
        feed_run(o, 1, 1);
        COV(CV_RX_ABORT1);
        continue;
      }
      /* the frame */
      feed_run(o, 1, 17);
      feed_run(o, 0, 1);
      if (kind == 3 && in.type) { flip = 0x13 + (int)h_rmod(0xa8 - 0x13); while (flip % 17 == 0) flip++; damaged = 1; }          /* damaged data bit */
      if (kind == 4 && in.type) { flip = 0x70; damaged = 2; }                                                                /* the bit the blob repairs */
      if (kind == 5) { flip = 0x13 + (int)h_rmod(0x43 - 0x13); while (flip % 17 == 0) flip++; damaged = 1; }                 /* damaged data bit (any type) */
      if (kind == 6) { /* a partial frame (truncated) followed by a restart */
        unsigned cut = 0x13 + h_rmod(n - 0x13);
        for (i = 0x12; i < cut; i++) (void)feed(o, st[i], 0);
        COV(CV_RX_PARTIAL);
        { unsigned guard = 0; while (!(mo->state_ == 0 && mo->widx_ == 0x12) && guard++ < 600) (void)feed(o, (int)h_rmod(2), 0); }   /* run the junk frame out */
        continue;
      }
      if (damaged == 1 && flip == 0x70 && in.type) damaged = 2;   /* the blob's retry flips exactly this buffer bit back */
      if (damaged && flip >= 0 && flip < (int)n) st[flip] ^= 1;
      for (i = 0x12; i < n; i++) {
        Step s = feed(o, st[i], 0);
        if ((unsigned)mo->zeros_ == 2 * D && mo->widx_ != 0x12 && mo->state_ != 0) COV(CV_RX_ED_BUSY);
        if (i + 1 < n && s.ret != 0 && !(s.ret == 3)) h_die3("early return code %d at bit %u of %u", s.ret, i, n);
        if (i + 1 == n) {
          int want;
          if (!damaged || damaged == 2) {
            if (damaged == 2 && in.type) { COV(CV_RX_MODGOOD); }
            want = mpnot ? 2 : 1;
            if (n >= 0x13 && s.ret != want && !(s.ret == 3)) h_die3("frame end: return %d, model %d (type %d cPack %d D %u damaged %d)", s.ret, want, in.type, in.cPack, D, damaged);
            exp_ret = want;
          } else {
            if (in.type) COV(CV_RX_BADCRC); else COV(CV_RX_BADCRC_T0);
            if (s.ret != 0 && s.ret != 3) h_die3("damaged frame: return %d", s.ret);
            exp_ret = 0;
          }
        }
      }
      (void)exp_ret;
      if (!damaged || damaged == 2) {
        unsigned nm = mo->nMp_ - nMp0, nn = mo->nMpNot_ - nMpNot0;
        if (nm + nn != 1) h_die3("frame counters moved by %u + %u (type %d D %u kind %u damaged %d n %u state %u widx %u total %u frameLen %u)", nm, nn, in.type, D, kind, damaged, n, mo->state_, mo->widx_, mo->totalLen_, mo->frameLen_);
        model_gather(bitsp(o), in.type, &gather);
        { Info got; get_info(mo, &got); if (!info_eq(&got, &gather)) h_die3("decoded info != buffer model"); }
        /* the decoded info equals the sent info (the flip of bit 0x70 is repaired, so the buffer holds the sent bit again) */
        { Info got; get_info(mo, &got); if (!info_eq(&got, &in)) h_die3("decoded info != sent info (type %d)", in.type); }
        if (mpnot) COV(in.type ? CV_RX_MPNOT_T1 : CV_RX_MPNOT_T0); else COV(in.type ? CV_RX_MP_T1 : CV_RX_MP_T0);
        if (mo->nMp_ + mo->nMpNot_ > 2) COV(CV_RX_CNT3);
        if (f > 0) COV(CV_RX_BACK2BACK);
      }
      if (damaged == 1 && mo->state_ != 0) h_die3("bad CRC: state_ = %u", mo->state_);
      DUMP("rx", o);
    }
    /* the counters' print */
    g_dbg_hook = NULL;
    dsplibs_debug_level = level;
    g_dbg_in_call = 0;
    mp_prnof(o);
    COV(CV_RX_PRNOT);
    h_emit(0, "prnof prints %u", g_dbg_in_call);
    if (g_dbg_in_call != (level > 1 ? 1u : 0u)) h_die3("printNofRecievedMpMpNot: %u prints at level 0x%x", g_dbg_in_call, level);
    DUMP("rxend", o);
  }
  /* print-count model: a clean frame at every fixed level, 3 frames of each class (no hook) */
  for (k = 0; k < 8; k++) {
    int cls;
    for (cls = 0; cls < 4; cls++) {
      void *o; h_pool_reset(); o = onew(OBJSZ);
      MpObj *mo = MO(o);
      Info in;
      uint8_t st[300];
      unsigned n, i, rep;
      mp_c1(o);
      mo->trn2dD_ = 2 + (unsigned)(k % 3);   /* D == 1 would print "Ed detected" at the type bit of a short frame (2 zeros while widx_ is still 0x12) */
      g_dbg_hook = NULL;
      for (rep = 0; rep < 4; rep++) {
        Step s = { 0, 0 };
        unsigned tot = 0;
        dsplibs_debug_level = g_levels[k];
        rnd_info(&in, 1);
        in.type = (uint8_t)(cls & 1);
        in.cPack = (uint8_t)((cls >> 1) & 1);
        n = frame_stream(&in, mo->trn2dD_, st);
        feed_run(o, 1, 1);   /* break the zero run of the previous frame's tail (2 * D zeros while idle print "Ed detected") */
        feed_run(o, 0, 1);
        feed_run(o, 1, 17);
        feed_run(o, 0, 1);
        for (i = 0x12; i < n; i++) { g_dbg_in_call = 0; s = feed(o, st[i], 0); tot += s.nprints; }
        expect_prints(g_levels[k], in.cPack != 0, rep < 2, tot);
        if (rep == 2) COV(CV_RX_CNT3);
        if (s.ret != (in.cPack ? 2 : 1)) h_die3("model frame: return %d", s.ret);
      }
      DUMP("lvl", o);
    }
  }
  /* damaged frames and Ed at fixed levels: the print of the CRC messages */
  for (k = 0; k < 8; k++) {
    unsigned lvl = g_levels[k];
    int mode;
    for (mode = 0; mode < 3; mode++) {   /* 0 bad CRC (long), 1 repaired (bit 0x70), 2 Ed */
      void *o; h_pool_reset(); o = onew(OBJSZ);
      MpObj *mo = MO(o);
      Info in;
      uint8_t st[300];
      unsigned n, i, tot = 0, want = 0;
      mp_c1(o);
      mo->trn2dD_ = 2;
      g_dbg_hook = NULL;
      dsplibs_debug_level = lvl;
      rnd_info(&in, 1);
      in.type = 1;
      n = frame_stream(&in, 2, st);
      feed_run(o, 0, 1);
      if (mode == 2) {
        unsigned z;
        for (z = 0; z < 4; z++) { g_dbg_in_call = 0; Step s = feed(o, 0, 0); tot += s.nprints; if (s.ret == 3) want = lvl > 1 ? 1u : 0u; }
        if (tot != want) h_die3("Ed prints %u, model %u", tot, want);
        continue;
      }
      feed_run(o, 1, 17);
      feed_run(o, 0, 1);
      st[mode == 0 ? 0x30 : 0x70] ^= 1;
      for (i = 0x12; i < n; i++) { g_dbg_in_call = 0; Step s = feed(o, st[i], 0); tot += s.nprints; (void)s; }
      if (mode == 0) { want = lvl > 1 ? 1u : 0u; if (tot != want) h_die3("bad CRC prints %u, model %u (level 0x%x)", tot, want, lvl); }
      else {
        /* repaired: "modified good CRC" at level > 2 (the MP detected print + mask + h lines follow for the first frame: counted separately) */
        unsigned base = 0;
        if (in.cPack == 0) { if (lvl > 1) base += 1; if (lvl > 2) base += 4; } else { if (lvl > 1) base += 5; }
        want = base + (lvl > 2 ? 1u : 0u);
        if (tot != want) h_die3("repaired frame prints %u, model %u (level 0x%x cPack %d)", tot, want, lvl, in.cPack);
      }
      DUMP("dmg", o);
    }
  }
  g_dbg_hook = NULL;
  /* D == 0: only noise that never reaches the length computation (state 2 would divide by zero); Ed fires on every 1 bit while idle and when the zero run wraps */
  for (trial = 0; trial < 60; trial++) {
    void *o; h_pool_reset(); o = onew(OBJSZ);
    unsigned i, n = 600;
    mp_c1(o);
    MO(o)->trn2dD_ = 0;
    dsplibs_debug_level = pick_level();
    COV(CV_RX_D0);
    for (i = 0; i < n; i++) {
      int bit;
      unsigned r = h_rmod(100);
      /* never 17 ones followed by a 0 followed by anything: cap the run of ones at 12 */
      bit = r < 40 ? 1 : 0;
      if (bit && MO(o)->ones_ >= 12) bit = 0;
      (void)feed(o, bit, 0);
    }
    if (trial % 6 == 0) { /* 256 zeros: zeros_ wraps to 0 and compares equal to 2 * D = 0 */
      for (i = 0; i < 300; i++) (void)feed(o, 0, 0);
      COV(CV_RX_ED_FLIP);
    }
    DUMP("d0", o);
  }
}

/* fully random object states: every case of the jump table, the default case, writes through widx_ (0 .. 255 reaches crc_, D and the lengths) */
static void sc_rxfuzz(void) {
  int trial, k;
  cov_def(CV_RF_S0, "state 0"); cov_def(CV_RF_S1, "state 1"); cov_def(CV_RF_S2, "state 2"); cov_def(CV_RF_S3, "state 3"); cov_def(CV_RF_S4, "state 4"); cov_def(CV_RF_SDEF, "state > 4 (no case)");
  cov_def(CV_RF_ED, "Ed"); cov_def(CV_RF_BIT0, "bit 0"); cov_def(CV_RF_BIT1, "bit != 0"); cov_def(CV_RF_WIDX_HI, "widx_ >= 0xe6"); cov_def(CV_RF_FLIP70, "long frame bad CRC (bit 0x70 flipped)");
  cov_def(CV_RF_RET1, "return 1"); cov_def(CV_RF_RET2, "return 2"); cov_def(CV_RF_RET3, "return 3"); cov_def(CV_RF_MODGOOD, "modified good CRC"); cov_def(CV_RF_BADCRC, "bad CRC");
  cov_def(CV_RF_FLEN_SMALL, "frameLen_ < 16"); cov_def(CV_RF_TYPE0, "frameType_ 0 CRC bound"); cov_def(CV_RF_TYPE1, "frameType_ != 0 CRC bound");
  h_covreq[CV_RF_MODGOOD] = 0;   /* a random frame never repairs itself: scenario 3 covers it */
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 6000; trial++) {
    void *o; h_pool_reset(); o = onew(OBJSZ);
    MpObj *mo = MO(o);
    unsigned ncalls = 20 + h_rmod(200), i;
    unsigned st = h_rmod(10);
    Info in;
    mp_c1(o);
    for (k = 0; k < 0xe6; k++) mo->bits_[k] = h_rpct(60) ? (uint8_t)h_rmod(2) : rbyte_kind();
    rnd_info(&in, 0);
    set_info(mo, &in);
    mo->state_ = st < 5 ? st : (st == 5 ? 5 : (st == 6 ? 0xffffffffu : (h_rpct(50) ? 5 + h_rmod(100) : h_rmod(5))));
    mo->frameType_ = (uint8_t)(h_rpct(50) ? h_rmod(2) : h_rnd());
    mo->ones_ = (uint8_t)(h_rpct(50) ? 16 + h_rmod(3) : h_rnd());
    mo->zeros_ = (uint8_t)(h_rpct(30) ? h_rmod(4) : h_rnd());
    mo->widx_ = (uint8_t)(h_rpct(40) ? 0x12 : (h_rpct(50) ? h_rmod(0xe0) : h_rnd()));
    mo->trn2dD_ = h_rpct(40) ? 1 + h_rmod(5) : (h_rpct(70) ? 1 + h_rmod(300) : (h_rpct(30) ? (h_rpct(50) ? 0x80000000u : 0x40000000u) : 1 + (h_rnd() & 0xffffffu)));   /* 2 * D wraps to 0 for 0x80000000, 4 * D for 0x40000000 */
    mo->totalLen_ = (uint8_t)(h_rpct(50) ? h_rmod(0xe6) : h_rnd());
    mo->frameLen_ = (uint8_t)(h_rpct(40) ? (h_rpct(50) ? 0x55 : 0xbb) : (h_rpct(30) ? h_rmod(24) : h_rnd()));
    mo->nMp_ = h_rpct(50) ? h_rmod(4) : h_rnd();
    mo->nMpNot_ = h_rpct(50) ? h_rmod(4) : h_rnd();
    for (k = 0; k < 16; k++) mo->crc_[k] = (uint8_t)(h_rpct(60) ? h_rmod(2) : h_rnd());
    if (h_rpct(10)) mo->frameLen_ = (uint8_t)(0x6d + h_rmod(5));   /* the long-frame threshold of the repair step (> 0x6f) */
    if (mo->state_ == 3 && h_rpct(50)) mo->widx_ = (uint8_t)(mo->frameLen_ - 1);   /* the next bit completes the frame: CRC check, repair, bad CRC */
    if (mo->state_ == 4 && h_rpct(50)) mo->widx_ = (uint8_t)(mo->totalLen_ - 1);   /* the next bit ends the frame: evaluateInfo and the detected prints with garbage info bytes */
    dsplibs_debug_level = pick_level();
    g_dbg_hook = NULL;
    if (h_rpct(25)) { g_dbg_hook = hook_set; g_hook_at = h_rmod(6); g_hook_to = h_rmod(4); }
    for (i = 0; i < ncalls; i++) {
      int bit = h_rpct(25) ? 1 + (int)h_rmod(3) * (h_rpct(10) ? 255 : 1) : 0;
      uint32_t s0 = mo->state_;
      Step s;
      if (h_rpct(60)) bit = (int)h_rmod(2);
      if (mo->frameLen_ < 16) COV(CV_RF_FLEN_SMALL);
      if (mo->widx_ >= 0xe6) COV(CV_RF_WIDX_HI);
      if (s0 == 0) COV(CV_RF_S0); else if (s0 == 1) COV(CV_RF_S1); else if (s0 == 2) COV(CV_RF_S2); else if (s0 == 3) COV(CV_RF_S3); else if (s0 == 4) COV(CV_RF_S4); else COV(CV_RF_SDEF);
      if (bit) COV(CV_RF_BIT1); else COV(CV_RF_BIT0);
      if (s0 == 2 && (mo->trn2dD_ == 0 || (mo->widx_ >= 0xf8 && mo->widx_ <= 0xfb))) continue;   /* the stored bit may zero D before the length division (#DE in the blob) */
      if (s0 == 4 && mo->trn2dD_ == 0) continue;
      if (s0 == 3) { if (mo->frameType_ == 0) COV(CV_RF_TYPE0); else COV(CV_RF_TYPE1); }
      s = feed(o, bit, 0);
      if (s.ret == 1) COV(CV_RF_RET1); else if (s.ret == 2) COV(CV_RF_RET2); else if (s.ret == 3) { COV(CV_RF_RET3); COV(CV_RF_ED); }
      if (s0 == 3 && mo->state_ == 4 && s.nprints > 0) COV(CV_RF_MODGOOD);
      if (s0 == 3 && mo->state_ == 0 && s.nprints > 0) COV(CV_RF_BADCRC);
      if (s0 == 3 && s.nprints == 0 && mo->state_ == 4) COV(CV_RF_FLIP70);
      if (i % 37 == 0) DUMP("rf", o);
      /* keep the object alive in the interesting states */
      if (h_rpct(4)) { mo->state_ = h_rmod(6); mo->widx_ = (uint8_t)(h_rpct(50) ? 0x12 : h_rnd()); mo->frameLen_ = (uint8_t)(h_rpct(50) ? (h_rpct(50) ? 0x55 : 0xbb) : h_rnd()); if (mo->trn2dD_ == 0) mo->trn2dD_ = 1; }
    }
    DUMP("rfend", o);
  }
  g_dbg_hook = NULL;
}

/* ================================================================================================ scenario 5: small functions */
static void model_pb2(char *out, uint32_t v, unsigned nbits) {
  /* the value's bits from position nbits' - 1 down to 0 with nbits' = nbits & 31 ... exactly: mask starts at 1 << ((n - 1) mod 32) and runs down; n == 0 starts at bit 31 and
   * suppresses leading zeros; a non-zero n prints every bit from the start position */
  uint32_t mask = nbits == 0 ? 0x80000000u : (1u << ((nbits - 1) & 31));
  int started = nbits != 0;
  char *p = out;
  for (; mask; mask >>= 1) {
    if (v & mask) { *p++ = '1'; started = 1; }
    else if (started) *p++ = '0';
  }
  *p = 0;
}
static void sc_small(void) {
  int trial, k;
  cov_def(CV_MC_C1, "C1"); cov_def(CV_MC_C2, "C2"); cov_def(CV_MC_D1, "D1"); cov_def(CV_MC_D2, "D2"); cov_def(CV_MC_RESET, "reset"); cov_def(CV_MC_RESETDET, "resetDetector"); cov_def(CV_MC_GETBV, "getBitVector");
  cov_def(CV_MC_SEQ, "calcSequenceLength"); cov_def(CV_MC_SEQ_EXACT, "length an exact multiple"); cov_def(CV_MC_SEQ_ROUND, "length rounded up"); cov_def(CV_MC_SEQ_TRUNC, "length truncated to 8 bits");
  cov_def(CV_MC_PB2, "PrintBase2"); cov_def(CV_MC_PB2_0, "PrintBase2 nbits 0"); cov_def(CV_MC_PB2_BIG, "PrintBase2 nbits > 32"); cov_def(CV_MC_PB2_ZEROV, "PrintBase2 value 0");
  cov_def(CV_MC_PRNOF, "printNof at level > 1"); cov_def(CV_MC_PRNOF_OFF, "printNof at level <= 1"); cov_def(CV_MC_RESETCRC, "resetCRC"); cov_def(CV_MC_PB2_33, "PrintBase2 nbits 33 (shift count wraps)");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 4000; trial++) {
    void *o; h_pool_reset(); o = onew(OBJSZ);
    MpObj *mo = MO(o);
    unsigned which = h_rmod(6);
    /* stale garbage everywhere (the constructors write only six fields) */
    for (k = 0; k < (int)OBJSZ; k++) ((uint8_t *)o)[k] = (uint8_t)h_rnd();
    switch (which) {
      case 0: mp_c1(o); COV(CV_MC_C1); break;
      case 1: mp_c2(o); COV(CV_MC_C2); break;
      case 2: mp_reset(o); COV(CV_MC_RESET); break;
      case 3: mp_resetdet(o); COV(CV_MC_RESETDET); break;
      case 4: mp_d1(o); COV(CV_MC_D1); break;
      default: mp_d2(o); COV(CV_MC_D2); break;
    }
    DUMP("ctor", o);
    h_emit(0, "ctor %u %016llx", which, (unsigned long long)fnv_blk(o, OBJSZ));
    if (which <= 2) { EXPECT_EQ("widx", mo->widx_, 0x12); EXPECT_EQ("state", mo->state_, 0); EXPECT_EQ("ones", mo->ones_, 0); EXPECT_EQ("zeros", mo->zeros_, 0); }
    if (which == 0 || which == 1 || which == 2) { EXPECT_EQ("nMp", mo->nMp_, 0); EXPECT_EQ("nMpNot", mo->nMpNot_, 0); }
    if (which == 3) { EXPECT_EQ("widx", mo->widx_, 0x12); EXPECT_EQ("state", mo->state_, 0); }
    /* getBitVector */
    {
      unsigned len = 0xdeadbeefu;
      uint8_t *p = mp_getbv(o, &len);
      COV(CV_MC_GETBV);
      if (p != bitsp(o)) h_die3("getBitVector returned %p, buffer %p", (void *)p, (void *)bitsp(o));
      EXPECT_EQ("getBitVector length", len, mo->totalLen_);
    }
    /* calcSequenceLength */
    {
      unsigned D = h_rpct(30) ? 1 + h_rmod(8) : (h_rpct(50) ? 1 + h_rmod(300) : 1 + (h_rnd() & 0xfffffu));
      unsigned fl = h_rpct(40) ? (h_rpct(50) ? 0x55 : 0xbb) : h_rmod(256);
      unsigned v = fl + 1, q = v / D, want = (v == q * D) ? (v & 255u) : (((q + 1) * D) & 255u);
      mo->trn2dD_ = D;
      mo->frameLen_ = (uint8_t)fl;
      mp_calcseq(o);
      COV(CV_MC_SEQ);
      if (v == q * D) COV(CV_MC_SEQ_EXACT); else COV(CV_MC_SEQ_ROUND);
      if ((q + 1) * D > 255 && v != q * D) COV(CV_MC_SEQ_TRUNC);
      EXPECT_EQ("calcSequenceLength", mo->totalLen_, want);
      DUMP("seq", o);
    }
    /* resetCRC */
    mp_resetcrc(o);
    COV(CV_MC_RESETCRC);
    for (k = 0; k < 16; k++) EXPECT_EQ("resetCRC", mo->crc_[k], 1);
    /* PrintBase2 */
    {
      char a[80], e[80];
      uint32_t v = h_rpct(15) ? 0u : (h_rpct(30) ? (uint32_t)(1u << h_rmod(32)) : h_rnd());
      unsigned nb = h_rpct(20) ? 0 : (h_rpct(50) ? 1 + h_rmod(32) : (h_rpct(50) ? 33 + h_rmod(40) : h_rnd() & 0xffffu));
      memset(a, 0x7e, sizeof a);
      mp_pb2(o, a, (unsigned long)v, (unsigned short)nb);
      model_pb2(e, v, nb);
      COV(CV_MC_PB2);
      if (nb == 0) COV(CV_MC_PB2_0);
      if (nb > 32) COV(CV_MC_PB2_BIG);
      if (nb == 33) COV(CV_MC_PB2_33);
      if (v == 0) COV(CV_MC_PB2_ZEROV);
      if (strcmp(a, e)) h_die3("PrintBase2(0x%x, %u): '%s' model '%s'", v, nb, a, e);
      h_emit(0, "pb2 %u %u %s", v, nb, a);
      /* nothing beyond the terminator is written */
      { size_t l = strlen(a); if ((uint8_t)a[l + 1] != 0x7e) h_die3("PrintBase2 wrote past the terminator"); }
    }
    /* printNofRecievedMpMpNot */
    {
      unsigned lvl = pick_level();
      mo->nMp_ = h_rnd(); mo->nMpNot_ = h_rnd();
      dsplibs_debug_level = lvl;
      g_dbg_in_call = 0;
      mp_prnof(o);
      if (lvl > 1) {
        char want[100];
        COV(CV_MC_PRNOF);
        snprintf(want, sizeof want, "V90MP: received %d MP, %d MPNot\\r\\n", (int)mo->nMp_, (int)mo->nMpNot_);
        if (g_dbg_in_call != 1 || strcmp(g_dbg_text[0], want)) h_die3("printNof text '%s' want '%s' (%u prints)", g_dbg_text[0], want, g_dbg_in_call);
      } else {
        COV(CV_MC_PRNOF_OFF);
        if (g_dbg_in_call != 0) h_die3("printNof printed at level %u", lvl);
      }
    }
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  if (getenv("P09_DBG")) g_dbg_show = 1;
  const char *seed = getenv("P09_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  p09_layout_check();
  h_x87_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_crc(); break;
    case 1: sc_evalinfo(); break;
    case 2: sc_info2bits(); break;
    case 3: sc_rx(); break;
    case 4: sc_rxfuzz(); break;
    case 5: sc_small(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: dbgprintf %u", g_ndbg);
  h_cov_finish();
  return 0;
}
