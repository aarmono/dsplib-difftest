/*
 * Differential test harness for port chunk P31 (src/vpcm/v90_phase4_modulator.cpp): the other 43 methods of V90Phase4Modulator (constructor C1 / C2 with the Scrambler member and the
 * V90BitsToSymbol it allocates, destructor D1 / D2, reset, generateSymbol, setMappingParams, setRdRtSymbols / setRfSymbols, the owner's event methods, the dead generators).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_phase4_modulator.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port
 * (-O0 / -O2: the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_phase4_modulator.blob32.out).  The methods are reached through
 * __asm__ labels; the object is the generated C mirror (test/vpcm/v90_phase4_modulator_symbol.layout), the callees' objects are plain heap blocks addressed through the mirrors of P09 (V90MP), P10 (V90CP), P11 (V90MappingParams),
 * P29 (V90BitsToSymbol).
 *
 * Every callee is a RECORDER with scripted results (Scrambler<uchar,uchar> ctor / dtor / process / processAllOnes / processAllZeros / reset, V90BitsToSymbol ctor / dtor / reset / setSymbolsBlockSize /
 * nofBitsForNextTime / process x2, V90MP / V90CP::getBitVector, V90CP::infoToBits, alaw2linear / ulaw2linear, generateV90Symbol / generateV92Symbol (P30: recorders here), edprintf,
 * dsplibs_debug_printf): each record carries the callee, its arguments and a SNAPSHOT (hash) of the whole object, the CP's words and the callee objects' words at that moment (store order before each
 * call / print).  Two hooks change the object FROM INSIDE a callee (an alaw / ulaw conversion flips pcm_, a symbol generator flips flag_) so that "re-read after the call" is observable.
 * What is compared: every returned value, the whole object after every call (field-wise dump + the model's S: every field, the 12000 scrambled bytes included), the callees' blocks (bytes untouched except the
 * CP's words), the canary behind the object, every record, the allocation counters.  The INDEPENDENT MODEL (mdl_*: written from porting/decompiled/dp/p31_phase4_modulator_b.txt, not from the port source)
 * predicts the result, the new object state and every record; a model violation exits 3.  Calls that would crash the REAL blob (divide by a zero frame length / D, a NULL CP / mapping parameters /
 * MP / V90BitsToSymbol dereference) are excluded by the driver.
 *
 * Scenarios (each its own process):
 *   0 owner events (recived* / exit* / reset*RRN / enterRepeatedCPd / setSessionFlag / setNextStateAfterTRN2d): random states / counts around every frame boundary, debug levels, NULL pointers
 *   1 patterns and generators (generateRi ... generateSUVd, generateDataSymbolBefore*, setMappingParams, setRdRtSymbols / setRfSymbols with the pcm flip hook)
 *   2 constructor (C1 and C2) / destructor (D1 and D2) / reset / generateSymbol
 */

#include <limits.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../harness/harness.h"
#include "v90_mp_layout.h"
#include "v90_cp_layout.h"
#include "v92_cp_layout.h"
#include "pcm_bits_to_symbol_layout.h"
#include "v90_phase4_modulator_symbol_layout.h"

#define L(m) __asm__(m)

extern unsigned int dsplibs_debug_level;
#define M(n) "_ZN18V90Phase4Modulator" n
/* the methods under test */
extern void f_ctor(void *, void *, unsigned, void *, void *, void *, void *, void *, unsigned) L(M("C1EP13V90ParametersjP15V90BitsToSymbolP5V90MPP16V90MappingParamsS7_P5V90CPj"));
extern void f_ctor2(void *, void *, unsigned, void *, void *, void *, void *, void *, unsigned) L(M("C2EP13V90ParametersjP15V90BitsToSymbolP5V90MPP16V90MappingParamsS7_P5V90CPj"));
extern void f_dtor(void *) L(M("D1Ev"));
extern void f_dtor2(void *) L(M("D2Ev"));
extern void f_resetBefor(void *) L(M("13resetBeforRRNEv"));
extern void f_setFlag(void *, unsigned) L(M("14setSessionFlagEj"));
extern short f_ri(void *) L(M("10generateRiEv"));
extern short f_riNot(void *) L(M("13generateRiNotEv"));
extern void f_enterRep(void *) L(M("16enterRepeatedCPdEv"));
extern short f_rdrt(void *) L(M("12generateRdRtEv"));
extern short f_rdrtNot(void *) L(M("15generateRdRtNotEv"));
extern void f_resetRRN2(void *) L(M("21resetRRNSecondSectionEv"));
extern short f_rf(void *) L(M("10generateRfEv"));
extern short f_rfNot(void *) L(M("13generateRfNotEv"));
extern void f_exitMPNot(void *) L(M("9exitMPNotEv"));
extern void f_recSUV(void *) L(M("10recivedSUVEv"));
extern void f_recP1(void *) L(M("27recivedPartOneSilenceRrnSUVEv"));
extern void f_recP2(void *) L(M("27recivedPartTwoSilenceRrnSUVEv"));
extern void f_recCP(void *) L(M("9recivedCPEv"));
extern void f_recSUVtag(void *) L(M("13recivedSUVtagEv"));
extern void f_recP1tag(void *) L(M("30recivedPartOneSilenceRrnSUVtagEv"));
extern void f_recP2tag(void *) L(M("30recivedPartTwoSilenceRrnSUVtagEv"));
extern void f_recCPtag(void *) L(M("12recivedCPtagEv"));
extern void f_recE2u(void *) L(M("10recivedE2uEv"));
extern void f_recFirstE2u(void *) L(M("18recivedFirstRrnE2uEv"));
extern void f_exitSil(void *) L(M("11exitSilenceEv"));
extern void f_recFirstSUVu(void *) L(M("26recivedFirstSUVuPartTwoRrnEv"));
extern void f_exitRi(void *) L(M("6exitRiEv"));
extern void f_setNext(void *, unsigned) L(M("22setNextStateAfterTRN2dE20Phase4ModulatorState"));
extern void f_exitMP(void *) L(M("6exitMPEv"));
extern void f_setMap(void *, void *) L(M("16setMappingParamsEP16V90MappingParams"));
extern void f_setRdRt(void *, void *) L(M("14setRdRtSymbolsEP16V90MappingParams"));
extern void f_setRf(void *, void *) L(M("12setRfSymbolsEP16V90MappingParams"));
extern short f_befFPE(void *) L(M("27generateDataSymbolBeforeFPEEv"));
extern short f_befRRN(void *) L(M("27generateDataSymbolBeforeRRNEv"));
extern short f_b1d(void *) L(M("11generateB1dEv"));
extern short f_trn(void *) L(M("13generateTRN2dEv"));
extern short f_ed(void *) L(M("10generateEdEv"));
extern short f_mp(void *) L(M("10generateMPEv"));
extern short f_cpd(void *) L(M("11generateCPdEv"));
extern short f_suvd(void *) L(M("12generateSUVdEv"));
extern short f_gensym(void *) L(M("14generateSymbolEv"));
extern void f_reset(void *, int, unsigned, unsigned, unsigned, unsigned) L(M("5resetE7PcmTypeh20Phase4ModulatorStatejj"));

/* the interposed callees */
extern void c_scr_ctor(void *, unsigned, unsigned, unsigned) L("_ZN9ScramblerIhhEC1Ejjj");
extern void c_scr_dtor(void *) L("_ZN9ScramblerIhhED1Ev");
extern void c_scr_proc(void *, const unsigned char *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE7processEPKhPhj");
extern void c_scr_ones(void *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE14processAllOnesEPhj");
extern void c_scr_zeros(void *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE15processAllZerosEPhj");
extern void c_scr_reset(void *, unsigned char) L("_ZN9ScramblerIhhE5resetEh");
extern void c_b2s_ctor(void *, unsigned, void *) L("_ZN15V90BitsToSymbolC1EjP13V90Parameters");
extern void c_b2s_dtor(void *) L("_ZN15V90BitsToSymbolD1Ev");
extern unsigned c_b2s_nof(void *) L("_ZN15V90BitsToSymbol18nofBitsForNextTimeEv");
extern unsigned c_b2s_pbits(void *, unsigned char *, unsigned) L("_ZN15V90BitsToSymbol7processEPhj");
extern unsigned c_b2s_pout(void *, unsigned *, short *) L("_ZN15V90BitsToSymbol7processERjPs");
extern void c_b2s_reset(void *, void *, int) L("_ZN15V90BitsToSymbol5resetEP16V90MappingParams7PcmType");
extern unsigned c_b2s_ssbs(void *, unsigned) L("_ZN15V90BitsToSymbol19setSymbolsBlockSizeEj");
extern unsigned char *c_mp_gbv(void *, unsigned *) L("_ZN5V90MP12getBitVectorERj");
extern unsigned char *c_cp_gbv(void *, unsigned *) L("_ZN5V90CP12getBitVectorERj");
extern void c_cp_i2b(void *) L("_ZN5V90CP10infoToBitsEv");
extern short c_gen90(void *) L(M("17generateV90SymbolEv"));
extern short c_gen92(void *) L(M("17generateV92SymbolEv"));

typedef Ph4Obj O;
#define SZ sizeof(O)
enum { CAN = 64, BITSN = 12000, BLK = 3016 };

/* ------------------------------------------------------------------------------------------------ the neutral state S (impl and model) */
#define UL(X) X(flag, flag_) X(state, state_) X(cnt, cnt_) X(event, event_) X(next, nextState_) X(cpCnt, cpCnt_) X(w20, w20_) X(w24, w24_) X(w28, w28_) X(w2c, w2c_) X(w30, w30_) X(w34, w34_) \
  X(pcm, pcm_) X(limit, limit_) X(mpLen, mpLen_) X(mpFrame, mpFrame_) X(target, target_) X(cpLen, cpLen_) X(cpFrame, cpFrame_) X(arg8, arg8_) X(w2f9c, w2f9c_) X(w2fa0, w2fa0_) X(notOwned, notOwned_)
#define PL(X) X(b2s, b2s_) X(mp, mp_) X(data, dataMp_) X(trn, trn2dMp_) X(cp, cp_) X(sbuf, scr_buf_) X(shead, scr_head0_) X(st10, scr_t1_0_) X(send, scr_end_) X(swr, scr_wr_) X(st1, scr_t1_) \
  X(st2, scr_t2_) X(mpbits, mpBits_) X(cpbits, cpBits_) X(params, params_)
enum {
#define X(n, m) U_##n,
  UL(X)
#undef X
  U_N
};
enum {
#define X(n, m) P_##n,
  PL(X)
#undef X
  P_N
};
typedef struct {
  uint32_t u[U_N];
  uintptr_t p[P_N];
  uint32_t stail;                                   /* scr_tail_ (a pointer-sized slot: the low 32 bits) */
  uint32_t mpNotSeen, cpFlag, level;
  uint16_t rdrt[6], rf[12];
  uint8_t pad15[3], pad1d[3], pad3e[2];
  uint32_t cpType, cpF13, cpWa0, cpD, mpD, b2sExtra, dataW0, trnW0;
  uint8_t bits[BITSN];
} S;
#define F(x) (s->u[U_##x])
#define P(x) (s->p[P_##x])

static O *g_o;
static uint8_t *g_can;
static void *g_b2s, *g_mp, *g_data, *g_trn, *g_cp, *g_params, *g_hist, *g_blk[4];   /* blk 0 / 1 = MP bit vectors, 2 / 3 = CP bit vectors */

static uint32_t fnv(const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; uint32_t h = 2166136261u; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static const char *bname(unsigned id) {
  static const char *const nm[] = { "NULL", "blk0", "blk1", "blk2", "blk3" };
  return id <= 4 ? nm[id] : "other";
}
static unsigned blk_id(const void *p) {
  int k;
  if (!p) return 0;
  for (k = 0; k < 4; k++) if (p == g_blk[k]) return (unsigned)k + 1;
  return 9;
}
/* an architecture independent id of a pointer value */
static unsigned ptr_id(uintptr_t v) {
  const void *q = (const void *)v;
  const HAl *a;
  if (!v) return 0;
  if (q == g_b2s) return 1;
  if (q == g_mp) return 2;
  if (q == g_data) return 3;
  if (q == g_trn) return 4;
  if (q == g_cp) return 5;
  if (q == g_params) return 6;
  if (q == g_hist) return 7;
  { int k; for (k = 0; k < 4; k++) if (q == g_blk[k]) return 8u + (unsigned)k; }
  if ((v >> 16) == 0xdead) return 0x10000u | (unsigned)(v & 0xffffu);
  if ((uintptr_t)q == (uintptr_t)((uint8_t *)g_o + 0x58) && 0) return 12;
  a = h_find_al(q, 1);
  if (a) return 0x20000u + a->id * 64u + (unsigned)((const uint8_t *)q - a->p);
  if (v >= (uintptr_t)g_o && v < (uintptr_t)g_o + SZ) return 0x40000u + (unsigned)(v - (uintptr_t)g_o) / (unsigned)sizeof(void *);
  return 0x30000u;
}
#define CPW(f) (((CpObj *)g_cp)->f)
static void load_impl(S *s) {
  const O *o = g_o;
  memset(s, 0, sizeof *s);
#define X(n, m) s->u[U_##n] = o->m;
  UL(X)
#undef X
#define X(n, m) s->p[P_##n] = (uintptr_t)o->m;
  PL(X)
#undef X
  s->stail = (uint32_t)o->scr_tail_;
  s->mpNotSeen = o->mpNotSeen_; s->cpFlag = o->cpFlag_; s->level = o->level_;
  memcpy(s->rdrt, o->rdrt_, sizeof s->rdrt); memcpy(s->rf, o->rf_, sizeof s->rf);
  memcpy(s->pad15, o->pad15_, 3); memcpy(s->pad1d, o->pad1d_, 3); memcpy(s->pad3e, o->pad3e_, 2);
  s->cpType = CPW(type_); s->cpF13 = CPW(f13_); s->cpWa0 = CPW(wCa0_); s->cpD = CPW(trn2dD_);
  s->mpD = ((MpObj *)g_mp)->trn2dD_; s->b2sExtra = ((B2S90 *)g_b2s)->extra_; s->dataW0 = ((MapPar90 *)g_data)->w0_; s->trnW0 = ((MapPar90 *)g_trn)->w0_;
  memcpy(s->bits, o->bits_, BITSN);
}
static void store_impl(const S *s) {
  O *o = g_o;
#define X(n, m) o->m = s->u[U_##n];
  UL(X)
#undef X
#define X(n, m) o->m = (void *)s->p[P_##n];
  PL(X)
#undef X
  o->scr_tail_ = s->stail;
  o->mpNotSeen_ = (uint8_t)s->mpNotSeen; o->cpFlag_ = (uint8_t)s->cpFlag; o->level_ = (uint16_t)s->level;
  memcpy(o->rdrt_, s->rdrt, sizeof s->rdrt); memcpy(o->rf_, s->rf, sizeof s->rf);
  memcpy(o->pad15_, s->pad15, 3); memcpy(o->pad1d_, s->pad1d, 3); memcpy(o->pad3e_, s->pad3e, 2);
  CPW(type_) = s->cpType; CPW(f13_) = (uint8_t)s->cpF13; CPW(wCa0_) = s->cpWa0; CPW(trn2dD_) = s->cpD;
  ((MpObj *)g_mp)->trn2dD_ = s->mpD; ((B2S90 *)g_b2s)->extra_ = s->b2sExtra; ((MapPar90 *)g_data)->w0_ = s->dataW0; ((MapPar90 *)g_trn)->w0_ = s->trnW0;
  memcpy(o->bits_, s->bits, BITSN);
}
/* the bit-vector id of a pointer value (0 NULL, 1..4 the harness blocks, 9 other) */
static unsigned vid(uintptr_t p) { return blk_id((const void *)p); }

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
enum { NSC = 16, LOGMAX = 24, TXTMAX = 220 };
typedef struct {
  unsigned nof[NSC], nxt[NSC]; int16_t sym[NSC], gsym[NSC]; unsigned mplen[4], cplen[4];
  unsigned lseed;
  int flip_pcm_at, flip_flag_at;                    /* the index (within one operation) of the alaw / ulaw call / the symbol generator call that flips pcm_ / flag_; -1 = none */
} Script;
typedef struct { unsigned inof, inxt, isym, imp, icp, iscr, ial, igen; } Cur;
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Script g_sc;
static Cur g_ic;
static Log g_il;
static int g_show;
static unsigned long g_free_base;

static unsigned bitgen(unsigned iscr, unsigned i) { return ((iscr * 2654435761u + i * 40503u + 12345u) >> 11) & 1u; }
static int lin_val(unsigned kind, unsigned arg, unsigned idx) {
  unsigned v = arg * 2654435761u + kind * 40503u + g_sc.lseed + idx * 977u;
  v ^= v >> 15; v *= 2246822519u; v ^= v >> 13;
  switch (v % 11) {
    case 0: return -32768;
    case 1: return 32768;
    case 2: return 0x12345;
    case 3: return 0;
    case 4: return -1;
    default: return (int)((v >> 8) & 0xffffu) - 32768;
  }
}
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
/* a hash of everything the callee could see: every numeric field, the pointer ids, the arrays, the CP words and the callee objects' words */
static uint32_t snap_hash(const S *s) {
  uint32_t buf[U_N + P_N + 64], n = 0, i;
  for (i = 0; i < U_N; i++) buf[n++] = s->u[i];
  for (i = 0; i < P_N; i++) buf[n++] = ptr_id(s->p[i]);
  buf[n++] = s->stail; buf[n++] = s->mpNotSeen; buf[n++] = s->cpFlag; buf[n++] = s->level;
  buf[n++] = s->cpType; buf[n++] = s->cpF13; buf[n++] = s->cpWa0; buf[n++] = s->cpD; buf[n++] = s->mpD; buf[n++] = s->b2sExtra; buf[n++] = s->dataW0; buf[n++] = s->trnW0;
  buf[n++] = fnv(s->rdrt, sizeof s->rdrt); buf[n++] = fnv(s->rf, sizeof s->rf);
  buf[n++] = fnv(s->pad15, 3) ^ fnv(s->pad1d, 3) ^ fnv(s->pad3e, 2);
  buf[n++] = fnv(s->bits, BITSN);
  return fnv(buf, n * 4);
}
static void snapshot(char *out, size_t cap, const S *s) {
  snprintf(out, cap, "|st=%u cnt=%u pcm=%u fl=%u s=%08x", s->u[U_state], s->u[U_cnt], s->u[U_pcm], s->u[U_flag], snap_hash(s));
}
static void rec_add(Log *lg, const S *s, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2], snap[TXTMAX];
  va_list ap;
  Rec *r = &lg->r[lg->n < LOGMAX ? lg->n : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  snapshot(snap, sizeof snap, s);
  r->t[0] = kind; r->t[1] = ' '; r->t[2] = 0;
  esc_cat(r->t, sizeof r->t, buf);
  if (strlen(r->t) + strlen(snap) + 1 < sizeof r->t) strcat(r->t, snap);
  lg->n++; lg->total++;
}
static void emit_rec(const Rec *r) { h_emit(g_show, "%s", r->t); }
static S g_tmp;
#define IMPL_REC(kind, ...) do { load_impl(&g_tmp); rec_add(&g_il, &g_tmp, kind, __VA_ARGS__); emit_rec(&g_il.r[(g_il.n - 1) < LOGMAX ? g_il.n - 1 : LOGMAX - 1]); } while (0)

static const char *mpname(const void *p) { return p == g_data ? "data" : p == g_trn ? "trn2d" : p ? "other" : "NULL"; }
#define CHECK_THIS(p, what) do { if ((p) != (what)) h_die3("callee %s got an unexpected object %p", __func__, (p)); } while (0)

/* the recorders (impl side) */
void c_scr_ctor(void *t, unsigned a, unsigned b, unsigned c) {
  CHECK_THIS(t, (void *)&g_o->scr_buf_);
  IMPL_REC('c', "scr.ctor %u %u %u", a, b, c);
}
void c_scr_dtor(void *t) {
  CHECK_THIS(t, (void *)&g_o->scr_buf_);
  IMPL_REC('c', "scr.dtor freed=%lu", h_nfree - g_free_base);
}
void c_scr_proc(void *t, const unsigned char *in, unsigned char *out, unsigned n) {
  unsigned id = blk_id(in), i;
  CHECK_THIS(t, (void *)&g_o->scr_buf_);
  IMPL_REC('c', "scr.process in=%s n=%u hin=%08x", bname(id), n, id >= 1 && id <= 4 ? fnv(in, n) : 0u);
  if (out != g_o->bits_) h_die3("scr.process: out is not bits_");
  if (n > BITSN) h_die3("scr.process: n %u beyond bits_", n);
  for (i = 0; i < n; i++) out[i] = (uint8_t)bitgen(g_ic.iscr, i);
  g_ic.iscr++;
}
void c_scr_ones(void *t, unsigned char *out, unsigned n) {
  unsigned i;
  CHECK_THIS(t, (void *)&g_o->scr_buf_);
  IMPL_REC('c', "scr.ones n=%u%s", n, out == g_o->bits_ ? " out=bits" : " out=OTHER");
  if (out != g_o->bits_ || n > BITSN) h_die3("scr.ones: bad out / n");
  for (i = 0; i < n; i++) out[i] = (uint8_t)bitgen(g_ic.iscr, i);
  g_ic.iscr++;
}
void c_scr_zeros(void *t, unsigned char *out, unsigned n) {
  unsigned i;
  CHECK_THIS(t, (void *)&g_o->scr_buf_);
  IMPL_REC('c', "scr.zeros n=%u%s", n, out == g_o->bits_ ? " out=bits" : " out=OTHER");
  if (out != g_o->bits_ || n > BITSN) h_die3("scr.zeros: bad out / n");
  for (i = 0; i < n; i++) out[i] = (uint8_t)bitgen(g_ic.iscr, i);
  g_ic.iscr++;
}
void c_scr_reset(void *t, unsigned char v) {
  CHECK_THIS(t, (void *)&g_o->scr_buf_);
  IMPL_REC('c', "scr.reset v=%u", (unsigned)v);
}
void c_b2s_ctor(void *t, unsigned cap, void *params) {
  const HAl *a = h_find_al(t, 0);
  IMPL_REC('c', "b2s.ctor %s cap=%u params=%u", a && a->p == (const uint8_t *)t && !a->freed ? "heap" : "OTHER", cap, ptr_id((uintptr_t)params));
}
void c_b2s_dtor(void *t) {
  IMPL_REC('c', "b2s.dtor this=%u freed=%lu", ptr_id((uintptr_t)t), h_nfree - g_free_base);
}
unsigned c_b2s_nof(void *t) {
  unsigned v = g_sc.nof[g_ic.inof++ % NSC];
  CHECK_THIS(t, g_b2s);
  IMPL_REC('c', "b2s.nof -> %u", v);
  return v;
}
unsigned c_b2s_pbits(void *t, unsigned char *in, unsigned n) {
  CHECK_THIS(t, g_b2s);
  IMPL_REC('c', "b2s.process(bits,%u)%s h=%08x", n, in == g_o->bits_ ? "" : " in=OTHER", in == g_o->bits_ && n <= BITSN ? fnv(in, n) : 0u);
  return 0;
}
unsigned c_b2s_pout(void *t, unsigned *nref, short *out) {
  unsigned v = g_sc.nxt[g_ic.inxt++ % NSC];
  short y = g_sc.sym[g_ic.isym++ % NSC];
  CHECK_THIS(t, g_b2s);
  IMPL_REC('c', "b2s.process(&n,&out) -> n=%u out=%d", v, (int)y);
  *nref = v;
  *out = y;
  return 0;
}
void c_b2s_reset(void *t, void *mp, int pcm) {
  CHECK_THIS(t, g_b2s);
  IMPL_REC('c', "b2s.reset mp=%s pcm=%u", mpname(mp), (unsigned)pcm);
}
unsigned c_b2s_ssbs(void *t, unsigned n) {
  CHECK_THIS(t, g_b2s);
  IMPL_REC('c', "b2s.setSymbolsBlockSize %u", n);
  return 0;
}
unsigned char *c_mp_gbv(void *t, unsigned *len) {
  unsigned k = g_ic.imp++, l = g_sc.mplen[k % 4];
  CHECK_THIS(t, g_mp);
  IMPL_REC('c', "MP.getBitVector len=%s -> blk%u,%u", (void *)len == (void *)&g_o->mpLen_ ? "&mpLen" : (void *)len == (void *)&g_o->cpLen_ ? "&cpLen" : "OTHER", k % 2, l);
  *len = l;
  return (unsigned char *)g_blk[k % 2];
}
unsigned char *c_cp_gbv(void *t, unsigned *len) {
  unsigned k = g_ic.icp++, l = g_sc.cplen[k % 4];
  CHECK_THIS(t, g_cp);
  IMPL_REC('c', "CP.getBitVector len=%s -> blk%u,%u", (void *)len == (void *)&g_o->mpLen_ ? "&mpLen" : (void *)len == (void *)&g_o->cpLen_ ? "&cpLen" : "OTHER", 2 + k % 2, l);
  *len = l;
  return (unsigned char *)g_blk[2 + k % 2];
}
void c_cp_i2b(void *t) {
  CHECK_THIS(t, g_cp);
  IMPL_REC('c', "CP.infoToBits");
}
/* the symbol generators of P30: scripted symbol; they advance cnt_ (so that later snapshots differ) and may flip flag_ */
static short rec_gen(int v92, void *t) {
  unsigned idx = g_ic.igen++;
  short y = g_sc.gsym[idx % NSC];
  CHECK_THIS(t, (void *)g_o);
  IMPL_REC('c', "gen%d", v92 ? 92 : 90);
  g_o->cnt_++;
  if (g_sc.flip_flag_at == (int)idx) g_o->flag_ = g_o->flag_ ? 0u : 1u;
  return y;
}
short c_gen90(void *t) { return rec_gen(0, t); }
short c_gen92(void *t) { return rec_gen(1, t); }
static int rec_lin(unsigned kind, int arg) {
  unsigned idx = g_ic.ial++;
  int v = lin_val(kind, (unsigned)arg, idx);
  IMPL_REC('c', "%s %d", kind ? "ulaw" : "alaw", arg);
  if (g_sc.flip_pcm_at == (int)idx) g_o->pcm_ = g_o->pcm_ ? 0u : 1u;
  return v;
}
int alaw2linear(int a) { return rec_lin(0, a); }
int ulaw2linear(int u) { return rec_lin(1, u); }
void edprintf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  IMPL_REC('e', "%s", buf);
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  IMPL_REC('d', "%s", buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ coverage: per-site counters in their own section */
typedef struct { unsigned long n; unsigned char hit, grp; const char *name; } CovSite;
#define CV(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, (g), (nm) }; cs_.hit = 1; } while (0)
extern CovSite __start_covsites[], __stop_covsites[];
static void cov_flush(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) if (c->hit) { c->n++; c->hit = 0; } }
static void cov_clear(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) c->hit = 0; }
enum { G_EV = 0, G_GEN = 1, G_LIFE = 2 };

/* ------------------------------------------------------------------------------------------------ the independent model */
typedef struct { S s; Cur c; Log l; int crash; unsigned dbg; void *alloc; unsigned nalloc, nfree_exp; } Mdl;
#define CRASH(m) do { (m)->crash = 1; return 0; } while (0)
static void m_rec(Mdl *m, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  rec_add(&m->l, &m->s, kind, "%s", buf);
}
static int16_t m_neg(int16_t v) { return (int16_t)(-(int)v); }
static void m_ed(Mdl *m, const char *fmt, unsigned a) { char b[TXTMAX]; snprintf(b, sizeof b, fmt, (int)a); m_rec(m, 'e', "%s", b); }
static unsigned m_nof(Mdl *m) { unsigned v = g_sc.nof[m->c.inof++ % NSC]; m_rec(m, 'c', "b2s.nof -> %u", v); return v; }
static void m_pbits(Mdl *m, unsigned n) { m_rec(m, 'c', "b2s.process(bits,%u) h=%08x", n, n <= BITSN ? fnv(m->s.bits, n) : 0u); }
static void m_pout(Mdl *m, unsigned *n, int16_t *out) {
  unsigned v = g_sc.nxt[m->c.inxt++ % NSC];
  int16_t y = g_sc.sym[m->c.isym++ % NSC];
  m_rec(m, 'c', "b2s.process(&n,&out) -> n=%u out=%d", v, (int)y);
  *n = v; *out = y;
}
static void m_fill(Mdl *m, unsigned n) { unsigned i; for (i = 0; i < n; i++) m->s.bits[i] = (uint8_t)bitgen(m->c.iscr, i); m->c.iscr++; }
static void m_scr_bits(Mdl *m, const char *what, unsigned n) { m_rec(m, 'c', "scr.%s n=%u out=bits", what, n); m_fill(m, n); }
static void m_scr_vec(Mdl *m, uintptr_t vp, unsigned n) {
  unsigned id = vid(vp);
  const unsigned char *src = id >= 1 && id <= 4 ? (const unsigned char *)vp : NULL;
  m_rec(m, 'c', "scr.process in=%s n=%u hin=%08x", bname(id), n, src ? fnv(src, n) : 0u);
  m_fill(m, n);
}
static int m_lin(Mdl *m, unsigned kind, unsigned arg) {
  unsigned idx = m->c.ial++;
  int v = lin_val(kind, arg, idx);
  m_rec(m, 'c', "%s %d", kind ? "ulaw" : "alaw", (int)arg);
  if (g_sc.flip_pcm_at == (int)idx) m->s.u[U_pcm] = m->s.u[U_pcm] ? 0u : 1u;
  return v;
}
static void m_gen(Mdl *m, int v92, int16_t *y) {
  unsigned idx = m->c.igen++;
  *y = g_sc.gsym[idx % NSC];
  m_rec(m, 'c', "gen%d", v92 ? 92 : 90);
  m->s.u[U_cnt]++;
  if (g_sc.flip_flag_at == (int)idx) m->s.u[U_flag] = m->s.u[U_flag] ? 0u : 1u;
}
static uintptr_t blkp(unsigned k) { return (uintptr_t)g_blk[k]; }
static void m_getbv(Mdl *m, int mp) {   /* V90MP / V90CP::getBitVector into mpLen_ / cpLen_, the bit pointer stored */
  S *s = &m->s;
  unsigned k = mp ? m->c.imp++ : m->c.icp++, l = mp ? g_sc.mplen[k % 4] : g_sc.cplen[k % 4];
  m_rec(m, 'c', mp ? "MP.getBitVector len=&mpLen -> blk%u,%u" : "CP.getBitVector len=&cpLen -> blk%u,%u", mp ? k % 2 : 2 + k % 2, l);
  if (mp) { F(mpLen) = l; P(mpbits) = blkp(k % 2); } else { F(cpLen) = l; P(cpbits) = blkp(2 + k % 2); }
}
#define NEEDP(m, x) do { if (!(m)->s.p[P_##x]) CRASH(m); } while (0)

/* the frame lengths of the MP / CP: the stale value is tested first by the callers; the refresh is 6 * len / D (32-bit product) */
static int m_refresh_cp(Mdl *m) { S *s = &m->s; if (m->s.cpD == 0) return 0; F(cpFrame) = (6u * F(cpLen)) / m->s.cpD; return 1; }
static int m_refresh_mp(Mdl *m) { S *s = &m->s; if (m->s.mpD == 0) return 0; F(mpFrame) = (6u * F(mpLen)) / m->s.mpD; return 1; }

/* "enter Ed" */
static void m_enter_ed(Mdl *m, const char *fmt) {
  S *s = &m->s;
  if (!P(b2s)) m->crash = 1;
  m_ed(m, fmt, F(cnt));
  F(cnt) = 0; F(state) = 0x10; F(target) = m->s.b2sExtra + 12u;
}
static const char *ED = "V90Phase4Modulator: enter Ed @ %d\r\n";

/* the three ways the V.92 owner events pick the next state from the current one: 5 -> 0xa, 7 / 8 -> 9, others none (signed compare with 5, then state - 7 as unsigned) */
static int m_tag_next(unsigned st, unsigned *to) {
  int32_t sv = (int32_t)st;
  if (sv == 5) { *to = 0xa; CV(G_EV, "tag state 5"); return 1; }
  if (sv < 5) { CV(G_EV, "tag state below 5 (signed)"); return 0; }
  if (st == 7 || st == 8) { *to = 9; CV(G_EV, "tag state 7 / 8"); return 1; }
  CV(G_EV, "tag state other (6, >= 9)");
  return 0;
}

/* enter CPd (V.92 state 7) from state 5 / the silence methods */
static int m_enter_cpd(Mdl *m, unsigned c) {
  S *s = &m->s;
  CV(G_EV, "enter CPd");
  m_ed(m, "V90Phase4Modulator: enter CPd @ %d\r\n", c);
  F(cnt) = 0; F(state) = 7; s->cpType = 0;
  m_rec(m, 'c', "CP.infoToBits");
  m_getbv(m, 0);
  F(w2fa0) = 1;
  if (!m_refresh_cp(m)) return 0;
  return 1;
}
static int m_suv_end(Mdl *m) {
  S *s = &m->s;
  unsigned c = F(cnt);
  if (F(cpFrame) == 0) return 0;
  if (c % F(cpFrame) != 0) { F(state) = 6; CV(G_EV, "suv end: state 6"); return 1; }
  return m_enter_cpd(m, c);
}
static int m_sil_end(Mdl *m, unsigned c) {
  S *s = &m->s;
  if (c % 6 != 0) { F(state) = 0x19; CV(G_EV, "silence: not a boundary"); return 1; }
  CV(G_EV, "silence: enter Rt");
  m_ed(m, "V90Phase4Modulator: enter Rt @ %d\r\n", c);
  F(state) = 0x1a; F(cnt) = 0;
  return 1;
}
/* the tail of the tag methods: frame boundary -> Ed (the format differs), else state `st`; then w20_ = 1 */
static int m_tag_end(Mdl *m, unsigned c, unsigned st, const char *fmt) {
  S *s = &m->s;
  if (F(cpFrame) == 0) return 0;
  if (c % F(cpFrame) != 0) { F(state) = st; CV(G_EV, "tag end: state change"); }
  else { CV(G_EV, "tag end: enter Ed"); m_enter_ed(m, fmt); }
  F(w20) = 1;
  return 1;
}

static int16_t m_ri(const S *s) {
  unsigned r = (s->u[U_cnt] - 1u) % 6u;
  return r < 3 ? (int16_t)s->level : (int16_t)(0u - s->level);
}
static int16_t m_data_step(Mdl *m, int kind) {   /* 0 ones, 1 zeros, 2 MP, 3 CP */
  S *s = &m->s;
  unsigned n = m_nof(m);
  int16_t y;
  if (n) {
    if (kind == 0) { m_scr_bits(m, "ones", n); m_pbits(m, n); }
    else if (kind == 1) { m_scr_bits(m, "zeros", n); m_pbits(m, n); }
    else if (kind == 2) { m_scr_vec(m, P(mpbits), F(mpLen)); m_pbits(m, F(mpLen)); }
    else { m_scr_vec(m, P(cpbits), F(cpLen)); m_pbits(m, F(cpLen)); }
  }
  m_pout(m, &n, &y);
  return y;
}
static int16_t m_drain(Mdl *m, int v) {   /* generateDataSymbolBeforeFPE (0) / BeforeRRN (1) */
  S *s = &m->s;
  unsigned n;
  int16_t y;
  m_pout(m, &n, &y);
  if (n) {
    CV(G_GEN, "drain: n != 0");
    m_ed(m, v ? "V90Phase4Modulator: enter Rd @ %d\r\n" : "V90Phase4Modulator: enter Rf @ %d\r\n", F(cnt));
    F(state) = v ? 0x15 : 0x1d; F(cnt) = 0;
  } else CV(G_GEN, "drain: n == 0");
  return y;
}

/* op numbers */
enum {
  O_RESETBEFOR, O_SETFLAG, O_RI, O_RINOT, O_ENTERREP, O_RDRT, O_RDRTNOT, O_RESETRRN2, O_RF, O_RFNOT, O_EXITMPNOT, O_RECSUV, O_RECP1, O_RECP2, O_RECCP, O_RECSUVTAG, O_RECP1TAG, O_RECP2TAG,
  O_RECCPTAG, O_RECE2U, O_RECFIRSTE2U, O_EXITSIL, O_RECFIRSTSUVU, O_EXITRI, O_SETNEXT, O_EXITMP, O_SETMAP, O_SETRDRT, O_SETRF, O_BEFFPE, O_BEFRRN, O_B1D, O_TRN, O_ED, O_MP, O_CPD, O_SUVD,
  O_GENSYM, O_RESET, O_CTOR, O_DTOR, O_N
};
typedef struct { uint32_t u[6]; uintptr_t p[8]; int c2; } Args;
static const char *const k_opname[O_N] = {
  "resetBeforRRN", "setSessionFlag", "generateRi", "generateRiNot", "enterRepeatedCPd", "generateRdRt", "generateRdRtNot", "resetRRNSecondSection", "generateRf", "generateRfNot", "exitMPNot",
  "recivedSUV", "recivedPartOneSilenceRrnSUV", "recivedPartTwoSilenceRrnSUV", "recivedCP", "recivedSUVtag", "recivedPartOneSilenceRrnSUVtag", "recivedPartTwoSilenceRrnSUVtag", "recivedCPtag",
  "recivedE2u", "recivedFirstRrnE2u", "exitSilence", "recivedFirstSUVuPartTwoRrn", "exitRi", "setNextStateAfterTRN2d", "exitMP", "setMappingParams", "setRdRtSymbols", "setRfSymbols",
  "generateDataSymbolBeforeFPE", "generateDataSymbolBeforeRRN", "generateB1d", "generateTRN2d", "generateEd", "generateMP", "generateCPd", "generateSUVd", "generateSymbol", "reset", "ctor", "dtor"
};

/* the six / twelve symbols from the mapping parameters: byte index of each entry and its sign */
static const int k_rdrt_sign[6] = { 1, 1, 1, -1, -1, -1 };
static const int k_rf_sign[12] = { 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1 };

static int16_t mdl_op(Mdl *m, int op, const Args *a) {
  S *s = &m->s;
  unsigned i, c, to;
  int16_t y = 0;
  switch (op) {
    case O_RESETBEFOR: F(w2f9c) = 0; F(w2fa0) = 0; F(w24) = 1; F(w28) = 0; F(w2c) = 0; F(w30) = 0; F(w34) = 0; F(w20) = 0; return 0;
    case O_SETFLAG: F(flag) = a->u[0]; return 0;
    case O_RI: return m_ri(s);
    case O_RINOT: return m_neg(m_ri(s));
    case O_ENTERREP:
      NEEDP(m, cp);
      s->cpFlag = 0; F(cpCnt) = 0;
      if (m->dbg > 1) { char b[TXTMAX]; snprintf(b, sizeof b, "V90Phase4Modulator: enter repeatedCPd @ %d\r\n", (int)F(cnt)); m_rec(m, 'd', "%s", b); CV(G_EV, "enterRepeatedCPd print"); }
      else CV(G_EV, "enterRepeatedCPd quiet");
      F(state) = 8; s->cpType = 0;
      m_rec(m, 'c', "CP.infoToBits");
      m_getbv(m, 0);
      if (s->cpD == 0) CRASH(m);
      { unsigned f = (6u * F(cpLen)) / s->cpD; F(cnt) = 0; F(cpFrame) = f; }
      return 0;
    case O_RDRT: return (int16_t)s->rdrt[(F(cnt) - 1u) % 6u];
    case O_RDRTNOT: return m_neg((int16_t)s->rdrt[(F(cnt) - 1u) % 6u]);
    case O_RF: return (int16_t)s->rf[(F(cnt) - 1u) % 12u];
    case O_RFNOT: return m_neg((int16_t)s->rf[(F(cnt) - 1u) % 12u]);
    case O_RESETRRN2:
      NEEDP(m, cp);
      F(w30) = 1; F(w20) = 0; s->cpFlag = 0; F(cpCnt) = 0; s->cpF13 = 0; F(w2f9c) = 0; F(w2fa0) = 0;
      return 0;
    case O_EXITMPNOT:
      if (F(state) != 0xe) { CV(G_EV, "exitMPNot: other state"); return 0; }
      c = F(cnt);
      if (c == 0) { CV(G_EV, "exitMPNot: count 0"); return 0; }
      if (F(mpFrame) == 0) CRASH(m);
      if (c % F(mpFrame) != 0) { F(state) = 0xf; CV(G_EV, "exitMPNot: wait"); return 0; }
      CV(G_EV, "exitMPNot: Ed");
      m_enter_ed(m, ED);
      return 0;
    case O_RECSUV:
      if (F(w2fa0) != 0) { CV(G_EV, "recivedSUV: w2fa0 set"); return 0; }
      if (F(state) != 5) { CV(G_EV, "recivedSUV: not state 5"); return 0; }
      NEEDP(m, cp);
      if (!m_suv_end(m)) CRASH(m);
      return 0;
    case O_RECP1: NEEDP(m, cp); s->cpF13 = 1; return 0;
    case O_RECP2:
      i = F(state);
      if (i == 0x17 || i == 0x18) { CV(G_EV, "recP2: state 0x17 / 0x18"); return 0; }
      if (i == 0x19 || i == 0x1a || i == 0x1b) { CV(G_EV, "recP2: state 0x19 .. 0x1b"); return 0; }
      if (F(w2fa0) != 0) { CV(G_EV, "recP2: w2fa0 set"); return 0; }
      if (i != 5) { CV(G_EV, "recP2: not state 5"); return 0; }
      NEEDP(m, cp);
      if (!m_suv_end(m)) CRASH(m);
      return 0;
    case O_RECCP: NEEDP(m, cp); F(w2f9c) = 1; s->cpF13 = 1; return 0;
    case O_RECSUVTAG:
    case O_RECP2TAG:
      s->cpFlag = 0; F(cpCnt) = 0;
      if (F(w2f9c) == 0) { CV(G_EV, "SUVtag: w2f9c clear"); return 0; }
      if (F(w20) != 0) { CV(G_EV, "SUVtag: w20 set"); return 0; }
      if (!m_tag_next(F(state), &to)) return 0;
      if (!m_tag_end(m, F(cnt), to, ED)) CRASH(m);
      return 0;
    case O_RECP1TAG:
      NEEDP(m, cp);
      s->cpF13 = 1;
      if (F(w20) != 0) { CV(G_EV, "P1tag: w20 set"); return 0; }
      if (!m_tag_end(m, F(cnt), 0xa, ED)) CRASH(m);
      return 0;
    case O_RECCPTAG:
      s->cpFlag = 0; F(cpCnt) = 0;
      if (F(w20) == 0) { CV(G_EV, "CPtag: w20 clear"); return 0; }
      if (F(w2f9c) == 0) {
        NEEDP(m, cp);
        CV(G_EV, "CPtag: first CP");
        F(w2f9c) = 1; s->cpF13 = 1; s->cpType = 1;
        if (F(cpFrame) == 0) CRASH(m);
        if (F(cnt) % F(cpFrame) != 0) { F(state) = 0xc; F(w20) = 1; CV(G_EV, "CPtag: first CP, state 0xc"); return 0; }
        CV(G_EV, "CPtag: first CP, state 0xb");
        F(state) = 0xb; F(cnt) = 0; F(w20) = 1;
        m_getbv(m, 0);
        if (s->cpD == 0) CRASH(m);
        F(cpFrame) = (6u * F(cpLen)) / s->cpD;
        return 0;
      }
      c = F(cnt);
      if (F(cpFrame) == 0) CRASH(m);
      if (c % F(cpFrame) == 0) { CV(G_EV, "CPtag: Ed"); m_enter_ed(m, ED); F(w20) = 1; return 0; }
      if (!m_tag_next(F(state), &to)) return 0;
      F(state) = to; F(w20) = 1;
      return 0;
    case O_RECE2U:
      if (F(w20) != 0) { CV(G_EV, "E2u: w20 set"); return 0; }
      c = F(cnt);
      if (F(cpFrame) == 0) CRASH(m);
      if (c % F(cpFrame) == 0) { CV(G_EV, "E2u: Ed"); m_enter_ed(m, ED); F(w20) = 1; return 0; }
      if (!m_tag_next(F(state), &to)) return 0;
      F(state) = to; F(w20) = 1;
      return 0;
    case O_RECFIRSTE2U:
      if (F(w20) != 0) { CV(G_EV, "FirstRrnE2u: w20 set"); return 0; }
      if (!m_tag_end(m, F(cnt), 0xa, "V90Phase4Modulator: enter Ed first at RRN @ %d\r\n")) CRASH(m);
      return 0;
    case O_EXITSIL:
      i = F(state);
      if (!(i == 0x17 || i == 0x18)) { CV(G_EV, "exitSilence: other state"); return 0; }
      if (F(cnt) == 0) { CV(G_EV, "exitSilence: count 0"); return 0; }
      m_sil_end(m, F(cnt));
      return 0;
    case O_RECFIRSTSUVU:
      i = F(state);
      if (i == 0x17 || i == 0x18) {
        if (F(cnt) == 0) { CV(G_EV, "FirstSUVu: silence, count 0"); return 0; }
        m_sil_end(m, F(cnt));
        return 0;
      }
      if (i != 5) { CV(G_EV, "FirstSUVu: other state"); return 0; }
      if (F(w2fa0) != 0) { CV(G_EV, "FirstSUVu: w2fa0 set"); return 0; }
      NEEDP(m, cp);
      if (!m_suv_end(m)) CRASH(m);
      return 0;
    case O_EXITRI:
      if (F(state) != 0) { CV(G_EV, "exitRi: other state"); return 0; }
      c = F(cnt);
      if (c == 0) { CV(G_EV, "exitRi: count 0"); return 0; }
      if (c % 6 != 0) { F(state) = 1; CV(G_EV, "exitRi: state 1"); return 0; }
      CV(G_EV, "exitRi: RiNot");
      m_ed(m, "V90Phase4Modulator: enter RiNot @ %d\r\n", c);
      F(state) = 2; F(cnt) = 0;
      return 0;
    case O_SETNEXT: F(next) = a->u[0]; return 0;
    case O_EXITMP:
      if (F(state) != 4) { CV(G_EV, "exitMP: other state"); return 0; }
      c = F(cnt);
      if (c == 0) { CV(G_EV, "exitMP: count 0"); return 0; }
      if (F(mpFrame) == 0) CRASH(m);
      if (c % F(mpFrame) != 0) { F(state) = 0xd; CV(G_EV, "exitMP: state 0xd"); return 0; }
      NEEDP(m, mp);
      CV(G_EV, "exitMP: MPNot");
      m_ed(m, "V90Phase4Modulator: enter MPNot @ %d\r\n", c);
      F(state) = 0xe; F(cnt) = 0;
      m_getbv(m, 1);
      if (!m_refresh_mp(m)) CRASH(m);
      return 0;
    case O_SETMAP:
      if (a->p[0] == 0) {
        if (m->dbg > 1) { m_rec(m, 'd', "V90Phase4Modulator: ERROR: Null mappingParams @ setMappingParams\r\n"); CV(G_GEN, "setMappingParams: NULL, print"); }
        else CV(G_GEN, "setMappingParams: NULL, quiet");
        return 0;
      }
      NEEDP(m, b2s);
      CV(G_GEN, "setMappingParams: ok");
      m_rec(m, 'c', "b2s.reset mp=%s pcm=%u", mpname((void *)a->p[0]), F(pcm));
      m_rec(m, 'c', "b2s.setSymbolsBlockSize 1");
      return 0;
    case O_SETRDRT:
    case O_SETRF: {
      const MapPar90 *mp = (const MapPar90 *)a->p[0];
      unsigned n = op == O_SETRDRT ? 6u : 12u;
      for (i = 0; i < n; i++) {
        unsigned char b = mp->listA_[0x80 * (i % 6)];
        int sign = op == O_SETRDRT ? k_rdrt_sign[i] : k_rf_sign[i];
        int v;
        unsigned p0 = F(pcm);
        if (F(pcm) != 0) { CV(G_GEN, "setRdRt/Rf: A-law"); v = m_lin(m, 0, (unsigned)((b & 0x7f) ^ 0xd5)); }
        else { CV(G_GEN, "setRdRt/Rf: mu-law"); v = m_lin(m, 1, (unsigned)(~(unsigned)(b & 0x7f) & 0xffu)); }
        if ((F(pcm) != 0) != (p0 != 0)) CV(G_GEN, "setRdRt/Rf: pcm_ flipped by a conversion");
        if (i >= 1 && (F(pcm) != 0) != (p0 != 0) && i < n - 1) CV(G_GEN, "setRdRt/Rf: pcm_ flipped mid-way");
        if (sign < 0) { CV(G_GEN, "setRdRt/Rf: negated"); v = -v; }
        if (op == O_SETRDRT) s->rdrt[i] = (uint16_t)v; else s->rf[i] = (uint16_t)v;
      }
      return 0;
    }
    case O_BEFFPE: NEEDP(m, b2s); return m_drain(m, 0);
    case O_BEFRRN: NEEDP(m, b2s); return m_drain(m, 1);
    case O_B1D: case O_TRN: NEEDP(m, b2s); return m_data_step(m, 0);
    case O_ED: NEEDP(m, b2s); return m_data_step(m, 1);
    case O_MP: NEEDP(m, b2s); return m_data_step(m, 2);
    case O_CPD: case O_SUVD: NEEDP(m, b2s); return m_data_step(m, 3);
    case O_GENSYM:
      if (F(flag) != 0) { CV(G_LIFE, "generateSymbol: V.92"); m_gen(m, 1, &y); } else { CV(G_LIFE, "generateSymbol: V.90"); m_gen(m, 0, &y); }
      return y;
    case O_RESET: {
      unsigned pcm = a->u[0], code = a->u[1] & 0xffu, n = a->u[3];
      int lv;
      F(pcm) = pcm; F(limit) = a->u[4]; F(cnt) = 0; F(state) = a->u[2]; F(event) = 0;
      if (pcm != 0) { CV(G_LIFE, "reset: A-law"); lv = m_lin(m, 0, (code & 0x7f) ^ 0xd5); } else { CV(G_LIFE, "reset: mu-law"); lv = m_lin(m, 1, (code & 0x7f) ^ 0xff); }
      s->level = (uint16_t)lv;
      if ((F(pcm) != 0) != (pcm != 0)) CV(G_LIFE, "reset: pcm_ flipped by the conversion");
      m_rec(m, 'c', "scr.reset v=0");
      s->mpNotSeen = 0; F(w24) = 0; F(w28) = 0; F(w2c) = 0;
      F(next) = F(flag) == 0 ? 4u : 5u;
      F(w2f9c) = 0; F(w2fa0) = 0; F(w30) = 0; F(cpCnt) = 0; s->cpFlag = 0; F(w20) = 0;
      for (i = 0; i < n; i++) {
        unsigned f0 = F(flag) != 0;
        if (F(flag) != 0) { CV(G_LIFE, "reset: loop V.92"); m_gen(m, 1, &y); } else { CV(G_LIFE, "reset: loop V.90"); m_gen(m, 0, &y); }
        if ((F(flag) != 0) != f0) CV(G_LIFE, "reset: flag_ flipped inside a generator");
      }
      if (n == 0) CV(G_LIFE, "reset: no loop");
      return 0;
    }
    case O_CTOR:   /* the harness computed the allocation (m->alloc) from the impl run */
      m_rec(m, 'c', "scr.ctor %u %u %u", 0x12u, 0x17u, 0x63u);
      P(params) = a->p[0]; P(cp) = a->p[5]; F(arg8) = a->u[4]; P(mp) = a->p[2]; P(data) = a->p[3]; F(flag) = a->u[0]; F(w2f9c) = 0; P(trn) = a->p[4]; F(w2fa0) = 0;
      if (a->p[1] != 0) { CV(G_LIFE, "ctor: V90BitsToSymbol passed in"); P(b2s) = a->p[1]; F(notOwned) = 1; }
      else {
        CV(G_LIFE, "ctor: V90BitsToSymbol allocated");
        m_rec(m, 'c', "b2s.ctor heap cap=%u params=%u", 0x140u, ptr_id(a->p[0]));
        P(b2s) = (uintptr_t)m->alloc; F(notOwned) = 0; m->nalloc = 1;
      }
      return 0;
    case O_DTOR:
      if (F(notOwned) == 0 && P(b2s) != 0) {
        CV(G_LIFE, "dtor: owned V90BitsToSymbol freed");
        m_rec(m, 'c', "b2s.dtor this=%u freed=0", ptr_id(P(b2s)));
        m->nfree_exp = 1;
      } else if (F(notOwned) != 0) CV(G_LIFE, "dtor: not owned");
      else CV(G_LIFE, "dtor: no V90BitsToSymbol");
      m_rec(m, 'c', "scr.dtor freed=%u", m->nfree_exp);
      return 0;
    default: h_die3("model: op %d", op);
  }
  return y;
}

/* ------------------------------------------------------------------------------------------------ comparison */
static Mdl g_m;
static unsigned long g_nops, g_nskipped, g_dumpctr, g_ntrials;
static const char *g_what = "";
static int g_pstate_ok = 1;
#define CMPN(a_, b_, nm) do { if ((a_) != (b_)) h_die3("%s: %s: model %llu (0x%llx), impl %llu (0x%llx)", g_what, nm, (unsigned long long)(a_), (unsigned long long)(a_), (unsigned long long)(b_), (unsigned long long)(b_)); } while (0)
static void cmp_S(const S *a, const S *b) {
  unsigned i;
  static const char *const un[] = {
#define X(n, m) #m,
    UL(X)
#undef X
  };
  static const char *const pn[] = {
#define X(n, m) #m,
    PL(X)
#undef X
  };
  for (i = 0; i < U_N; i++) CMPN(a->u[i], b->u[i], un[i]);
  for (i = 0; i < P_N; i++) CMPN(a->p[i], b->p[i], pn[i]);
  CMPN(a->stail, b->stail, "scr_tail_"); CMPN(a->mpNotSeen, b->mpNotSeen, "mpNotSeen_"); CMPN(a->cpFlag, b->cpFlag, "cpFlag_"); CMPN(a->level, b->level, "level_");
  CMPN(a->cpType, b->cpType, "cp type_"); CMPN(a->cpF13, b->cpF13, "cp f13_"); CMPN(a->cpWa0, b->cpWa0, "cp wCa0_"); CMPN(a->cpD, b->cpD, "cp trn2dD_"); CMPN(a->mpD, b->mpD, "mp trn2dD_");
  CMPN(a->b2sExtra, b->b2sExtra, "b2s extra_"); CMPN(a->dataW0, b->dataW0, "data w0_"); CMPN(a->trnW0, b->trnW0, "trn w0_");
  for (i = 0; i < 6; i++) if (a->rdrt[i] != b->rdrt[i]) h_die3("%s: rdrt[%u]: model 0x%x impl 0x%x", g_what, i, a->rdrt[i], b->rdrt[i]);
  for (i = 0; i < 12; i++) if (a->rf[i] != b->rf[i]) h_die3("%s: rf[%u]: model 0x%x impl 0x%x", g_what, i, a->rf[i], b->rf[i]);
  if (memcmp(a->pad15, b->pad15, 3) || memcmp(a->pad1d, b->pad1d, 3) || memcmp(a->pad3e, b->pad3e, 2)) h_die3("%s: a never-touched pad changed", g_what);
  for (i = 0; i < BITSN; i++) if (a->bits[i] != b->bits[i]) h_die3("%s: bits_[%u]: model %u impl %u", g_what, i, a->bits[i], b->bits[i]);
}
static HFld g_ft[96];
static int g_nft;
static void dump_obj(const char *tag, int show) {
  if (!g_nft) {
    int i;
    g_nft = NF_Ph4Obj;
    if (g_nft > 96) h_die3("field table");
    memcpy(g_ft, f_Ph4Obj, sizeof(HFld) * (size_t)g_nft);
    for (i = 0; i < g_nft; i++) if (!strcmp(g_ft[i].name, "bits_")) { g_ft[i].kind = 'x'; g_ft[i].count = BITSN; }   /* 12000 scrambled bytes: hashed, not listed */
  }
  h_dump_obj(show, tag, g_o, g_ft, g_nft);
  g_dumpctr++;
}
/* the callees' objects: saved before an operation, compared after (nothing but the CP's words type_ / f13_ may change) */
static uint8_t sv_b2s[0x40], sv_mp[0x200], sv_data[0x700], sv_trn[0x700], sv_cp[0x4000], sv_blk[4][BLK];
static void save_blocks(void) {
  memcpy(sv_b2s, g_b2s, sizeof sv_b2s); memcpy(sv_mp, g_mp, sizeof sv_mp); memcpy(sv_data, g_data, sizeof sv_data); memcpy(sv_trn, g_trn, sizeof sv_trn); memcpy(sv_cp, g_cp, sizeof sv_cp);
  { int k; for (k = 0; k < 4; k++) memcpy(sv_blk[k], g_blk[k], BLK); }
}
static void check_blocks(const S *model) {
  int k;
  uint8_t exp[0x4000];
  if (memcmp(sv_b2s, g_b2s, sizeof sv_b2s)) h_die3("%s: the V90BitsToSymbol block was written", g_what);
  if (memcmp(sv_mp, g_mp, sizeof sv_mp)) h_die3("%s: the V90MP block was written", g_what);
  if (memcmp(sv_data, g_data, sizeof sv_data)) h_die3("%s: the data mapping parameters were written", g_what);
  if (memcmp(sv_trn, g_trn, sizeof sv_trn)) h_die3("%s: the TRN2d mapping parameters were written", g_what);
  for (k = 0; k < 4; k++) if (memcmp(sv_blk[k], g_blk[k], BLK)) h_die3("%s: a bit block was written", g_what);
  memcpy(exp, sv_cp, sizeof exp);
  ((CpObj *)exp)->type_ = model->cpType; ((CpObj *)exp)->f13_ = (uint8_t)model->cpF13;
  if (memcmp(exp, g_cp, sizeof(CpObj))) h_die3("%s: the CP block differs from the model beyond its words type_ / f13_", g_what);
}
static unsigned long g_symctr;
static void log_cmp(const Mdl *m) {
  unsigned i, n = m->l.n < LOGMAX ? m->l.n : LOGMAX;
  if (m->l.n != g_il.n) {
    for (i = 0; i < (m->l.n < LOGMAX ? m->l.n : LOGMAX); i++) fprintf(stderr, "model %u: %s\n", i, m->l.r[i].t);
    for (i = 0; i < (g_il.n < LOGMAX ? g_il.n : LOGMAX); i++) fprintf(stderr, "impl  %u: %s\n", i, g_il.r[i].t);
    h_die3("%s: %u records expected, %u produced", g_what, m->l.n, g_il.n);
  }
  for (i = 0; i < n; i++)
    if (strcmp(m->l.r[i].t, g_il.r[i].t)) h_die3("%s: record %u differs:\n  model %s\n  impl  %s", g_what, i, m->l.r[i].t, g_il.r[i].t);
}

/* the call of the method under test */
static int is_short_op(int op) {
  switch (op) {
    case O_RI: case O_RINOT: case O_RDRT: case O_RDRTNOT: case O_RF: case O_RFNOT: case O_BEFFPE: case O_BEFRRN: case O_B1D: case O_TRN: case O_ED: case O_MP: case O_CPD: case O_SUVD: case O_GENSYM: return 1;
    default: return 0;
  }
}
static int16_t call_impl(int op, const Args *a) {
  void *o = g_o;
  switch (op) {
    case O_RESETBEFOR: f_resetBefor(o); return 0;
    case O_SETFLAG: f_setFlag(o, a->u[0]); return 0;
    case O_RI: return f_ri(o);
    case O_RINOT: return f_riNot(o);
    case O_ENTERREP: f_enterRep(o); return 0;
    case O_RDRT: return f_rdrt(o);
    case O_RDRTNOT: return f_rdrtNot(o);
    case O_RESETRRN2: f_resetRRN2(o); return 0;
    case O_RF: return f_rf(o);
    case O_RFNOT: return f_rfNot(o);
    case O_EXITMPNOT: f_exitMPNot(o); return 0;
    case O_RECSUV: f_recSUV(o); return 0;
    case O_RECP1: f_recP1(o); return 0;
    case O_RECP2: f_recP2(o); return 0;
    case O_RECCP: f_recCP(o); return 0;
    case O_RECSUVTAG: f_recSUVtag(o); return 0;
    case O_RECP1TAG: f_recP1tag(o); return 0;
    case O_RECP2TAG: f_recP2tag(o); return 0;
    case O_RECCPTAG: f_recCPtag(o); return 0;
    case O_RECE2U: f_recE2u(o); return 0;
    case O_RECFIRSTE2U: f_recFirstE2u(o); return 0;
    case O_EXITSIL: f_exitSil(o); return 0;
    case O_RECFIRSTSUVU: f_recFirstSUVu(o); return 0;
    case O_EXITRI: f_exitRi(o); return 0;
    case O_SETNEXT: f_setNext(o, a->u[0]); return 0;
    case O_EXITMP: f_exitMP(o); return 0;
    case O_SETMAP: f_setMap(o, (void *)a->p[0]); return 0;
    case O_SETRDRT: f_setRdRt(o, (void *)a->p[0]); return 0;
    case O_SETRF: f_setRf(o, (void *)a->p[0]); return 0;
    case O_BEFFPE: return f_befFPE(o);
    case O_BEFRRN: return f_befRRN(o);
    case O_B1D: return f_b1d(o);
    case O_TRN: return f_trn(o);
    case O_ED: return f_ed(o);
    case O_MP: return f_mp(o);
    case O_CPD: return f_cpd(o);
    case O_SUVD: return f_suvd(o);
    case O_GENSYM: return f_gensym(o);
    case O_RESET: f_reset(o, (int)a->u[0], a->u[1], a->u[2], a->u[3], a->u[4]); return 0;
    case O_CTOR:
      if (a->c2) f_ctor2(o, (void *)a->p[0], a->u[0], (void *)a->p[1], (void *)a->p[2], (void *)a->p[3], (void *)a->p[4], (void *)a->p[5], a->u[4]);
      else f_ctor(o, (void *)a->p[0], a->u[0], (void *)a->p[1], (void *)a->p[2], (void *)a->p[3], (void *)a->p[4], (void *)a->p[5], a->u[4]);
      return 0;
    case O_DTOR: if (a->c2) f_dtor2(o); else f_dtor(o); return 0;
    default: h_die3("op %d", op);
  }
  return 0;
}

/* one operation: model first (a call that would crash the blob is skipped), then the object; the constructor runs the object first (the model needs the allocation) */
static int do_op(int op, const Args *a) {
  Mdl m;
  S cur, ns;
  int16_t ms = 0, is = 0;
  unsigned long nm0, nf0;
  g_what = k_opname[op];
  m = g_m;
  m.l.n = 0; m.crash = 0; m.alloc = NULL; m.nalloc = 0; m.nfree_exp = 0;
  m.dbg = dsplibs_debug_level;
  m.c.ial = m.c.igen = 0;
  g_ic.ial = g_ic.igen = 0;
  cov_clear();
  if (op != O_CTOR) {
    ms = mdl_op(&m, op, a);
    if (m.crash) { g_nskipped++; cov_clear(); return 0; }
  }
  load_impl(&cur);
  cmp_S(&g_m.s, &cur);   /* in sync before the call */
  save_blocks();
  g_il.n = 0;
  nm0 = h_nmalloc; nf0 = h_nfree; g_free_base = nf0;
  g_show = h_sample(&g_symctr, 12, 500);
  is = call_impl(op, a);
  __asm__ volatile("" ::: "memory");
  if (op == O_CTOR) {
    if (h_nmalloc - nm0 == 1) m.alloc = h_al[h_last_alloc].p;
    ms = mdl_op(&m, op, a);
  }
  load_impl(&ns);
  if (is_short_op(op) && ms != is) h_die3("%s: result: model %d, impl %d", g_what, (int)ms, (int)is);
  cmp_S(&m.s, &ns);
  log_cmp(&m);
  check_blocks(&m.s);
  if (memcmp(&m.c, &g_ic, sizeof m.c)) h_die3("%s: the callee cursors differ", g_what);
  if (h_nmalloc - nm0 != m.nalloc) h_die3("%s: %lu allocations, expected %u", g_what, h_nmalloc - nm0, m.nalloc);
  if (h_nfree - nf0 != m.nfree_exp) h_die3("%s: %lu frees, expected %u", g_what, h_nfree - nf0, m.nfree_exp);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i); }
  g_m = m;
  h_emit(g_show, "op %s -> %d st=%u cnt=%u", g_what, (int)is, ns.u[U_state], ns.u[U_cnt]);
  dump_obj("op", g_show);
  cov_flush();
  g_nops++;
  return 1;
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(65) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static unsigned pick_pat32(void) {
  static const unsigned v[] = { 0, 1, 2, 0x7fffffffu, 0x80000000u, 0xffffffffu, 0xfffffffeu, 0x18, 0x180, 0x3e7c };
  return h_rpct(60) ? h_rnd() : v[h_rmod(sizeof v / sizeof v[0])];
}
static unsigned pick01(void) { return h_rpct(40) ? 0u : h_rpct(65) ? 1u : pick_pat32(); }
static void fill_random(void *p, size_t n) { uint8_t *q = (uint8_t *)p; size_t i; for (i = 0; i < n; i++) q[i] = (uint8_t)h_rnd(); }
static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) {
    g_sc.nof[i] = h_rpct(35) ? 0u : rnd_range(1, 40);
    g_sc.nxt[i] = h_rpct(45) ? 0u : rnd_range(1, 40);
    g_sc.sym[i] = h_rpct(8) ? (int16_t)(h_rpct(50) ? -32768 : 32767) : (int16_t)h_rnd();
    g_sc.gsym[i] = h_rpct(8) ? (int16_t)(h_rpct(50) ? -32768 : 32767) : (int16_t)h_rnd();
  }
  for (i = 0; i < 4; i++) { g_sc.mplen[i] = h_rpct(20) ? rnd_range(2, 5) : rnd_range(2, 3000); g_sc.cplen[i] = h_rpct(20) ? rnd_range(2, 5) : rnd_range(2, 3000); }
  g_sc.lseed = h_rnd();
  g_sc.flip_pcm_at = h_rpct(35) ? (int)h_rmod(12) : -1;
  g_sc.flip_flag_at = h_rpct(30) ? (int)h_rmod(5) : -1;
}
static void alloc_fakes(void) {
  int k;
  if (sizeof(CpObj) > 0x4000 || sizeof(MpObj) > 0x200 || sizeof(MapPar90) > 0x700 || sizeof(B2S90) > 0x40) h_die3("a fake object block is too small");
  g_b2s = sysdep_malloc(0x40); g_mp = sysdep_malloc(0x200); g_data = sysdep_malloc(0x700); g_trn = sysdep_malloc(0x700); g_cp = sysdep_malloc(0x4000);
  g_params = sysdep_malloc(0x40); g_hist = sysdep_malloc(0x40);
  for (k = 0; k < 4; k++) { g_blk[k] = sysdep_malloc(BLK); }
  fill_random(g_b2s, 0x40); fill_random(g_mp, 0x200); fill_random(g_data, 0x700); fill_random(g_trn, 0x700); fill_random(g_cp, 0x4000); fill_random(g_hist, 0x40);
  for (k = 0; k < 4; k++) { uint8_t *q = (uint8_t *)g_blk[k]; unsigned i; for (i = 0; i < BLK; i++) q[i] = (uint8_t)(h_rpct(50) ? (h_rnd() & 1u) : h_rnd()); }
}
static void free_fakes(void) {
  int k;
  sysdep_free(g_b2s); sysdep_free(g_mp); sysdep_free(g_data); sysdep_free(g_trn); sysdep_free(g_cp); sysdep_free(g_params); sysdep_free(g_hist);
  for (k = 0; k < 4; k++) sysdep_free(g_blk[k]);
}
/* the object: random in every field (architecture independent: the table drives the fill) */
static void prefill_obj(void) {
  uint8_t *o = (uint8_t *)g_o;
  int i;
  unsigned k, n;
  memset(o, 0, SZ);
  for (i = 0; i < NF_Ph4Obj; i++) {
    const HFld *f = &f_Ph4Obj[i];
    n = f->count > 1 ? f->count : 1;
    if (f->kind == 'x') { for (k = 0; k < f->count; k++) o[f->off + k] = (uint8_t)h_rnd(); continue; }
    for (k = 0; k < n; k++) {
      uint32_t v = h_rnd();
      uint8_t *q;
      switch (f->kind) {
        case 'b': q = o + f->off + k; *q = (uint8_t)v; break;
        case 'h': q = o + f->off + 2u * k; memcpy(q, &v, 2); break;
        case 'u': q = o + f->off + 4u * k; memcpy(q, &v, 4); break;
        default: { uintptr_t pv = 0xdead0000u | (v & 0xffffu); q = o + f->off + sizeof(void *) * k; memcpy(q, &pv, sizeof pv); break; }
      }
    }
  }
  memset(o + SZ, 0x5a, CAN);
}
static unsigned pick_state(void) {
  static const unsigned like[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0, 4, 5, 0xe, 0x17, 0x18, 4, 0 };
  if (h_rpct(10)) return pick_pat32();
  if (h_rpct(10)) return rnd_range(0, 40);
  if (h_rpct(8)) return like[h_rmod(sizeof like / sizeof like[0])] | (h_rpct(50) ? 0x10000u : 0x80000000u);   /* garbage in the upper bits of the state */
  return like[h_rmod(sizeof like / sizeof like[0])];
}
/* the symbol counter: around the frame boundaries / the six-symbol boundary (the table is the harness' own steering, not the model) */
static unsigned pick_cnt(const S *s) {
  unsigned cf = s->u[U_cpFrame], mf = s->u[U_mpFrame], r = h_rmod(100), k = 1 + h_rmod(4), c;
  int d = h_rpct(25) ? (int)h_rmod(3) - 1 : 0;
  if (r < 30) c = k * cf; else if (r < 50) c = k * mf; else if (r < 70) c = 6u * k; else if (r < 80) c = 0; else if (r < 85) c = (h_rpct(50) ? 0xfffffffcu : 0xfffffff4u) - 6u * h_rmod(4); else c = pick_pat32();
  return c + (unsigned)d;
}
static uintptr_t pick_vec(void) {
  static const unsigned mb[] = { 0, 1, 2, 9, 1, 2, 3, 4 };
  unsigned id = mb[h_rmod(8)];
  return id == 0 ? 0 : id <= 4 ? (uintptr_t)g_blk[id - 1] : (uintptr_t)g_hist;
}
static void init_state(S *s) {
  unsigned lim, i;
  memset(s, 0, sizeof *s);
  F(flag) = h_rpct(50) ? h_rmod(2) : pick_pat32();
  F(state) = pick_state();
  F(event) = pick_pat32();
  F(next) = h_rpct(60) ? 4u : pick_pat32() & 0x1fu;
  s->mpNotSeen = h_rpct(50) ? 1u : (h_rpct(40) ? 0u : (h_rpct(50) ? 0x80u : 0xffu));
  s->cpFlag = h_rpct(50) ? 1u : (h_rpct(40) ? 0u : (h_rpct(50) ? 2u : 0xffu));
  lim = h_rpct(30) ? (0xfffffce0u + h_rmod(4) - 1u) : h_rpct(50) ? rnd_range(0, 1000) : pick_pat32();
  F(limit) = lim;
  F(cpCnt) = h_rpct(50) ? lim + 0x320u + (unsigned)(int)h_rmod(3) - 1u : pick_pat32();
  F(w20) = pick01(); F(w28) = pick_pat32(); F(w24) = pick01(); F(w2c) = pick01(); F(w30) = pick01(); F(w34) = h_rpct(40) ? 0u : pick_pat32();
  F(pcm) = h_rpct(85) ? h_rmod(2) : pick_pat32();
  { static const unsigned a[] = { 0, 1, 0x7fff, 0x8000, 0xffff, 0x1234, 0xfffe }; s->level = h_rpct(50) ? a[h_rmod(7)] : h_rnd() & 0xffffu; }
  for (i = 0; i < 6; i++) s->rdrt[i] = (uint16_t)(h_rpct(10) ? 0x8000 : h_rnd());
  for (i = 0; i < 12; i++) s->rf[i] = (uint16_t)(h_rpct(10) ? 0x8000 : h_rnd());
  fill_random(s->pad15, 3); fill_random(s->pad1d, 3); fill_random(s->pad3e, 2);
  F(mpLen) = rnd_range(2, 3000); F(cpLen) = rnd_range(2, 3000);
  s->mpD = h_rpct(1) ? 0u : rnd_range(1, 12); s->cpD = h_rpct(1) ? 0u : rnd_range(1, 12);
  F(mpFrame) = h_rpct(1) ? 0u : h_rpct(80) ? rnd_range(1, 9) : rnd_range(1, 100);
  F(cpFrame) = h_rpct(1) ? 0u : h_rpct(80) ? rnd_range(1, 9) : rnd_range(1, 100);
  F(arg8) = pick_pat32(); F(w2f9c) = pick01(); F(w2fa0) = pick01(); F(notOwned) = pick_pat32() & 1u;
  P(mpbits) = pick_vec(); P(cpbits) = pick_vec();
  P(b2s) = (uintptr_t)g_b2s; P(mp) = h_rpct(88) ? (uintptr_t)g_mp : 0; P(data) = h_rpct(85) ? (uintptr_t)g_data : 0; P(trn) = h_rpct(88) ? (uintptr_t)g_trn : 0; P(cp) = h_rpct(94) ? (uintptr_t)g_cp : 0;
  P(params) = (uintptr_t)g_params;
  P(sbuf) = (uintptr_t)g_hist; P(shead) = (uintptr_t)g_hist + 4; P(st10) = (uintptr_t)g_hist + 8; P(send) = (uintptr_t)g_hist + 16; P(swr) = (uintptr_t)g_hist + 4; P(st1) = (uintptr_t)g_hist + 8;
  P(st2) = (uintptr_t)g_hist + 16; s->stail = 17;
  s->cpType = pick_pat32(); s->cpF13 = h_rnd() & 0xffu; s->cpWa0 = pick_pat32();
  s->b2sExtra = h_rpct(20) ? 0xfffffff4u + h_rmod(12) : h_rpct(50) ? rnd_range(0, 200) : pick_pat32();
  s->dataW0 = pick_pat32(); s->trnW0 = pick_pat32();
  fill_random(s->bits, BITSN);
  F(cnt) = pick_cnt(s);
  F(target) = h_rpct(60) ? F(cnt) + 1u : pick_pat32();
}
static void new_trial(void) {
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  S s;
  g_o = (O *)raw; g_can = raw + SZ;
  prefill_obj();
  alloc_fakes();
  gen_script();
  memset(&g_ic, 0, sizeof g_ic); g_il.n = 0;
  init_state(&s);
  dsplibs_debug_level = pick_level();
  store_impl(&s);
  memset(&g_m, 0, sizeof g_m);
  load_impl(&g_m.s);
  g_m.dbg = dsplibs_debug_level;
  g_what = "trial setup";
  dump_obj("init", 0);
  g_ntrials++;
}
static void end_trial(void) {
  free_fakes();
  h_trial_end("p31");
}
/* the state re-aimed between operations (the model follows) */
static void reaim(void) {
  S t = g_m.s, n;
  init_state(&n);
  if (h_rpct(70)) { t.u[U_cnt] = pick_cnt(&t); }
  if (h_rpct(60)) { t.u[U_state] = pick_state(); }
  if (h_rpct(35)) { t.u[U_w20] = n.u[U_w20]; t.u[U_w2f9c] = n.u[U_w2f9c]; t.u[U_w2fa0] = n.u[U_w2fa0]; }
  if (h_rpct(25)) { t.u[U_cpFrame] = n.u[U_cpFrame]; t.u[U_mpFrame] = n.u[U_mpFrame]; t.u[U_cpLen] = n.u[U_cpLen]; t.u[U_mpLen] = n.u[U_mpLen]; t.cpD = n.cpD; t.mpD = n.mpD; }
  if (h_rpct(20)) { t.u[U_pcm] = n.u[U_pcm]; t.level = n.level; }
  if (h_rpct(20)) { t.u[U_flag] = n.u[U_flag]; }
  if (h_rpct(25)) { t.u[U_target] = t.u[U_cnt] + 1u; t.b2sExtra = n.b2sExtra; }
  store_impl(&t);
  g_m.s = t;
  if (h_rpct(15)) { dsplibs_debug_level = pick_level(); g_m.dbg = dsplibs_debug_level; }
  if (h_rpct(30)) gen_script();
}
static uintptr_t pick_mp(int allow_null) {
  unsigned r = h_rmod(10);
  if (allow_null && r < 2) return 0;
  return r < 6 ? (uintptr_t)g_data : (uintptr_t)g_trn;
}
static void gen_args(int op, Args *a) {
  memset(a, 0, sizeof *a);
  switch (op) {
    case O_SETFLAG: case O_SETNEXT: a->u[0] = pick_pat32(); break;
    case O_SETMAP: a->p[0] = pick_mp(1); break;
    case O_SETRDRT: case O_SETRF: a->p[0] = pick_mp(0); break;
    case O_RESET:
      a->u[0] = h_rpct(88) ? h_rmod(2) : pick_pat32();
      a->u[1] = h_garbage_hi(h_rnd() & 0xffu);
      a->u[2] = pick_state();
      a->u[3] = h_rpct(30) ? 0u : rnd_range(1, 5);
      a->u[4] = pick_pat32();
      break;
    default: break;
  }
}
static const int k_ev[] = { O_RESETBEFOR, O_SETFLAG, O_ENTERREP, O_RESETRRN2, O_EXITMPNOT, O_RECSUV, O_RECP1, O_RECP2, O_RECCP, O_RECSUVTAG, O_RECP1TAG, O_RECP2TAG, O_RECCPTAG, O_RECE2U,
  O_RECFIRSTE2U, O_EXITSIL, O_RECFIRSTSUVU, O_EXITRI, O_SETNEXT, O_EXITMP };
static const int k_gen[] = { O_RI, O_RINOT, O_RDRT, O_RDRTNOT, O_RF, O_RFNOT, O_SETMAP, O_SETRDRT, O_SETRF, O_BEFFPE, O_BEFRRN, O_B1D, O_TRN, O_ED, O_MP, O_CPD, O_SUVD };
static const int k_life[] = { O_GENSYM, O_RESET, O_SETFLAG, O_GENSYM, O_RESET };
static void trial_ops(const int *tab, unsigned ntab) {
  unsigned ops = rnd_range(1, 9), k;
  new_trial();
  for (k = 0; k < ops; k++) {
    int op = tab[h_rmod(ntab)];
    Args a;
    if (k > 0 && h_rpct(65)) reaim();
    gen_args(op, &a);
    do_op(op, &a);
  }
  end_trial();
}

/* the constructor (C1 / C2): the object holds random bytes, the arguments are the harness' fake objects or NULL; an owned V90BitsToSymbol is freed afterwards */
static void trial_ctor(void) {
  Args a;
  new_trial();
  memset(&a, 0, sizeof a);
  { S t = g_m.s; unsigned k; for (k = 0; k < P_N; k++) if (k == P_b2s || k == P_mp || k == P_data || k == P_trn || k == P_cp || k == P_params) t.p[k] = 0xdead0000u | (h_rnd() & 0xffffu);   /* the arguments differ from what was there */
    store_impl(&t); g_m.s = t; }
  a.p[0] = h_rpct(85) ? (uintptr_t)g_params : (h_rpct(50) ? 0u : (uintptr_t)g_hist);
  a.p[1] = h_rpct(50) ? (uintptr_t)g_b2s : 0u;
  a.p[2] = h_rpct(80) ? (uintptr_t)g_mp : 0u;
  a.p[3] = h_rpct(80) ? (uintptr_t)g_data : 0u;
  a.p[4] = h_rpct(80) ? (uintptr_t)g_trn : 0u;
  a.p[5] = h_rpct(80) ? (uintptr_t)g_cp : 0u;
  a.u[0] = pick01();
  a.u[4] = pick_pat32();
  a.c2 = h_rpct(30);
  do_op(O_CTOR, &a);
  if (g_m.s.u[U_notOwned] == 0 && g_o->b2s_ != NULL) sysdep_free(g_o->b2s_);
  end_trial();
}
/* the destructor (D1 / D2) over the three kinds of V90BitsToSymbol ownership; the object must be left exactly as it was */
static void trial_dtor(void) {
  Args a;
  S t;
  unsigned mode = h_rmod(4);
  new_trial();
  t = g_m.s;
  memset(&a, 0, sizeof a);
  if (mode == 0 || mode == 1) { t.u[U_notOwned] = 0; t.p[P_b2s] = (uintptr_t)sysdep_malloc(0x24); }   /* owned, heap block */
  else if (mode == 2) { t.u[U_notOwned] = 0; t.p[P_b2s] = 0; }                                         /* owned flag with no block */
  else { t.u[U_notOwned] = h_rpct(50) ? 1u : pick_pat32() | 1u; t.p[P_b2s] = h_rpct(60) ? (uintptr_t)g_b2s : (uintptr_t)g_hist; }
  store_impl(&t);
  g_m.s = t;
  a.c2 = h_rpct(40);
  do_op(O_DTOR, &a);
  end_trial();
}

/* ------------------------------------------------------------------------------------------------ main */
static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0;
  const char *seed = getenv("P31_SEED");
  CovSite *c;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P31_DBG")) g_show = 1;
  p09_layout_check(); p10_layout_check(); p11_layout_check(); p29_layout_check(); p30_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  if (p29_probe_B2S90[0] != 0x24) h_log_map_add(p29_probe_B2S90[0], 0x24);   /* the port's V90BitsToSymbol (x86-64) is logged with the blob's size */
  switch (sc) {
    case 0: for (tr = 0; tr < 6000; tr++) trial_ops(k_ev, sizeof k_ev / sizeof k_ev[0]); break;
    case 1: for (tr = 0; tr < 5000; tr++) trial_ops(k_gen, sizeof k_gen / sizeof k_gen[0]); break;
    case 2:
      for (tr = 0; tr < 1500; tr++) trial_ops(k_life, sizeof k_life / sizeof k_life[0]);
      for (tr = 0; tr < 1200; tr++) trial_ctor();
      for (tr = 0; tr < 1200; tr++) trial_dtor();
      break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu ops %lu skipped %lu trials %lu dumps %lu", h_nmalloc, h_nfree, g_nops, g_nskipped, g_ntrials, g_dumpctr);
  for (c = __start_covsites; c < __stop_covsites; c++) {
    if (c->grp != sc) continue;
    h_emit(1, "COV %s %lu", c->name, c->n);
    if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P31_NOCOV")) ? 2 : 0;
}
