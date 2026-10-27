/*
 * Differential test harness for port chunk P33 (src/vpcm/v92_phase4_modulator.cpp): the other 35 methods of V92Phase4Modulator (36 symbols with the C2 / D2 twins; generateSymbol is P32): constructor C1 / C2
 * (the Scrambler member and the V92Mapper it allocates), destructor D1 / D2, reset, resetBeforRRN / resetBeforFPE, the owner's event methods (exitCPt, exitTRN2u, recived*), setMappingParams, the generators.
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v92_phase4_modulator.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2:
 * the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v92_phase4_modulator.blob32.out).  The methods are reached through __asm__ labels; the object
 * is the generated C mirror (test/vpcm/v92_phase4_modulator_symbol.layout), the callees' objects are plain heap blocks addressed through the mirrors of P11 (V92CP), P15 (V92ModulusEncoder), P29 (V92BitsToSymbol / V92Transmitter).
 *
 * Every callee is a RECORDER with scripted results (Scrambler<uchar,uchar> ctor / dtor / process(uchar) / processAllOnes / processAllZeros / reset, V92Mapper ctor / dtor / reset / process, V92BitsToSymbol
 * nofBitsForNextTime / process x2 / reset / setSymbolsBlockSize, V92CP::infoToBits / getBitVector, V92Phase4Modulator::generateSymbol (reset calls it n times), edprintf, dsplibs_debug_printf): each
 * record carries the callee, its arguments and a SNAPSHOT of the object and of the CP's words (store order before each call / print).  What is compared: every returned value, the whole object after every
 * call (field-wise dump + the model's S), the callees' blocks (bytes untouched except the CP's five words and the modulus encoder's mode_), the canary behind the object, every record, the allocation
 * counters.  The INDEPENDENT MODEL (m_*: written from porting/decompiled/dp/p33_phase4_modulator_b.txt, not from the port source) predicts the result, the new object state and every record; a model violation
 * exits 3.  Calls that would crash the REAL blob (divide by a zero frame length / mult_ / cpLen_, a NULL CP or mapper dereference, a bit vector that is not a harness block, mult_ == 0 where the block
 * XOR would touch bits_[-1]) are excluded by the driver.
 *
 * Scenarios (each its own process):
 *   0 owner events (reset*, exit*, recived*, enterRepeatedCP): random states / counts around every frame boundary, debug levels, NULL pointers
 *   1 generators (generateRu .. generateCPt, the BeforeFPE / BeforeRRN drains, setMappingParams)
 *   2 constructor (C1 and C2) / destructor (D1 and D2) / reset
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
#include "v92_cp_layout.h"
#include "v92_modulus_encoder_layout.h"
#include "pcm_bits_to_symbol_layout.h"
#include "v92_phase4_modulator_symbol_layout.h"

#define L(m) __asm__(m)

extern unsigned int dsplibs_debug_level;
#define M(n) "_ZN18V92Phase4Modulator" n
/* the methods under test */
extern void f_ctor(void *, void *, void *, void *, void *) L(M("C1EP13V92ParametersP15V92BitsToSymbolP5V92CPP16V92MappingParams"));
extern void f_ctor2(void *, void *, void *, void *, void *) L(M("C2EP13V92ParametersP15V92BitsToSymbolP5V92CPP16V92MappingParams"));
extern void f_dtor(void *) L(M("D1Ev"));
extern void f_dtor2(void *) L(M("D2Ev"));
extern void f_resetRRN(void *) L(M("13resetBeforRRNEv"));
extern void f_resetFPE(void *) L(M("13resetBeforFPEEv"));
extern void f_enterRep(void *) L(M("15enterRepeatedCPEv"));
extern short f_ru(void *) L(M("10generateRuEv"));
extern short f_ruNot(void *) L(M("13generateRuNotEv"));
extern void f_resetRRN2(void *) L(M("21resetRRNSecondSectionEv"));
extern void f_exitCPt(void *) L(M("7exitCPtEv"));
extern void f_exitTRN2u(void *) L(M("9exitTRN2uEv"));
extern void f_recSUV(void *) L(M("10recivedSUVEv"));
extern void f_recP1(void *) L(M("27recivedPartOneSilenceRrnSUVEv"));
extern void f_recP2(void *) L(M("27recivedPartTwoSilenceRrnSUVEv"));
extern void f_recCP(void *) L(M("9recivedCPEv"));
extern void f_recSUVtag(void *) L(M("13recivedSUVtagEv"));
extern void f_recP1tag(void *) L(M("30recivedPartOneSilenceRrnSUVtagEv"));
extern void f_recP2tag(void *) L(M("30recivedPartTwoSilenceRrnSUVtagEv"));
extern void f_recCPtag(void *) L(M("12recivedCPtagEv"));
extern void f_recEd(void *) L(M("9recivedEdEv"));
extern void f_recFirstEd(void *) L(M("17recivedFirstRrnEdEv"));
extern void f_recRt(void *) L(M("9recivedRtEv"));
extern void f_setMap(void *, void *) L(M("16setMappingParamsEP16V92MappingParams"));
extern short f_befFPE(void *) L(M("27generateDataSymbolBeforeFPEEv"));
extern short f_befRRN(void *) L(M("27generateDataSymbolBeforeRRNEv"));
extern short f_rm(void *) L(M("10generateRmEv"));
extern short f_b1u(void *) L(M("11generateB1uEv"));
extern short f_trn2u(void *) L(M("13generateTRN2uEv"));
extern short f_e2u(void *) L(M("11generateE2uEv"));
extern short f_cpu(void *) L(M("11generateCPuEv"));
extern short f_suvu(void *) L(M("12generateSUVuEv"));
extern short f_e1u(void *) L(M("11generateE1uEv"));
extern short f_cpt(void *) L(M("11generateCPtEv"));
extern void f_reset(void *, int, unsigned, unsigned, unsigned, unsigned) L(M("5resetEsh23V92Phase4ModulatorStatejj"));

/* the interposed callees */
extern void c_scr_ctor(void *, unsigned, unsigned, unsigned) L("_ZN9ScramblerIhhEC1Ejjj");
extern void c_scr_dtor(void *) L("_ZN9ScramblerIhhED1Ev");
extern unsigned char c_scr_p1(void *, unsigned char) L("_ZN9ScramblerIhhE7processEh");
extern void c_scr_ones(void *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE14processAllOnesEPhj");
extern void c_scr_zeros(void *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE15processAllZerosEPhj");
extern void c_scr_reset(void *, unsigned char) L("_ZN9ScramblerIhhE5resetEh");
extern void c_map_ctor1(void *) L("_ZN9V92MapperC1Ev");
extern void c_map_ctor2(void *) L("_ZN9V92MapperC2Ev");
extern void c_map_dtor1(void *) L("_ZN9V92MapperD1Ev");
extern void c_map_dtor2(void *) L("_ZN9V92MapperD2Ev");
extern void c_map_reset(void *, int, unsigned) L("_ZN9V92Mapper5resetEsh");
extern short c_map_proc(void *, unsigned char *) L("_ZN9V92Mapper7processEPh");
extern unsigned c_b2s_nof(void *) L("_ZN15V92BitsToSymbol18nofBitsForNextTimeEv");
extern unsigned c_b2s_pbits(void *, unsigned char *, unsigned) L("_ZN15V92BitsToSymbol7processEPhj");
extern unsigned c_b2s_pout(void *, unsigned *, short *) L("_ZN15V92BitsToSymbol7processERjPs");
extern void c_b2s_reset(void *, void *) L("_ZN15V92BitsToSymbol5resetEP16V92MappingParams");
extern unsigned c_b2s_setblk(void *, unsigned) L("_ZN15V92BitsToSymbol19setSymbolsBlockSizeEj");
extern void c_cp_i2b(void *) L("_ZN5V92CP10infoToBitsEv");
extern unsigned char *c_cp_gbv(void *, unsigned *) L("_ZN5V92CP12getBitVectorERj");
extern short c_gensym(void *) L(M("14generateSymbolEv"));

typedef V92Ph4Obj O;
#define SZ sizeof(O)
enum { CAN = 64, BITSN = 300, BLK = 3016, NBLK = 3 };

/* ------------------------------------------------------------------------------------------------ the neutral state (impl and model) */
typedef struct {
  uint32_t state, cnt, w08, event, cpCnt, w20, w24, w28, w2c, w30, w34, w38, w3c, limit, inv, cpLen, cpFrame, w1b8, w1bc, w1c0, w1c4;
  uint32_t cpFlag, level, b42, mult;
  uint8_t pad10[8], pad1d[3], pad1b4[4];
  uint32_t cpBitsId;                 /* 0 NULL, 1 / 2 / 3 = blk0 / 1 / 2, 9 other */
  uint32_t pMp, pCp, pB2s, pParams, pMapper;   /* pointer ids: 0 NULL, 1 b2s, 2 mp, 3 cp, 4 params, 5 mapper, 6 hist, 7 the block of this operation, 9 other */
  uint32_t cpType, cpFlagCp, cpSuv, cpSent, cpMult, meMode;
  uint8_t bits[BITSN];
} S;

static O *g_o;
static uint8_t *g_can;
static void *g_b2s, *g_tx, *g_me, *g_mapper, *g_mp, *g_cp, *g_params, *g_hist, *g_blk[NBLK], *g_mblk;

static uint32_t fnv(const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; uint32_t h = 2166136261u; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static const char *bname(unsigned id) {
  static const char *const nm[] = { "NULL", "blk0", "blk1", "blk2" };
  return id <= 3 ? nm[id] : "other";
}
static unsigned blk_id(const void *p) {
  int k;
  if (!p) return 0;
  for (k = 0; k < NBLK; k++) if (p == g_blk[k]) return (unsigned)k + 1;
  return 9;
}
static unsigned pid(const void *p) {
  if (!p) return 0;
  if (p == g_b2s) return 1;
  if (p == g_mp) return 2;
  if (p == g_cp) return 3;
  if (p == g_params) return 4;
  if (p == g_mapper) return 5;
  if (p == g_hist) return 6;
  if (g_mblk && p == g_mblk) return 7;
  return 9;
}
static void *idp(unsigned id) {
  switch (id) {
    case 0: return NULL;
    case 1: return g_b2s;
    case 2: return g_mp;
    case 3: return g_cp;
    case 4: return g_params;
    case 5: return g_mapper;
    case 6: return g_hist;
    case 7: return g_mblk;
    default: return (void *)(uintptr_t)0xdead1234u;
  }
}
#define CPW(f) (((V92CpObj *)g_cp)->f)
#define MEW(f) (((V92ModEncObj *)g_me)->f)
static void load_impl(S *s) {
  const O *o = g_o;
  memset(s, 0, sizeof *s);
  s->state = o->state_; s->cnt = o->cnt_; s->w08 = o->w08_; s->event = o->event_; s->cpCnt = o->cpCnt_; s->cpFlag = o->cpFlag_;
  s->w20 = o->w20_; s->w24 = o->w24_; s->w28 = o->w28_; s->w2c = o->w2c_; s->w30 = o->w30_; s->w34 = o->w34_; s->w38 = o->w38_; s->w3c = o->w3c_;
  s->level = o->level_; s->b42 = o->b42_; s->mult = o->mult_; s->limit = o->limit_; s->inv = o->inv_;
  s->cpLen = o->cpLen_; s->cpFrame = o->cpFrame_; s->w1b8 = o->w1b8_; s->w1bc = o->w1bc_; s->w1c0 = o->w1c0_; s->w1c4 = o->w1c4_;
  memcpy(s->pad10, o->pad10_, 8); memcpy(s->pad1d, o->pad1d_, 3); memcpy(s->pad1b4, o->pad1b4_, 4);
  s->cpBitsId = blk_id(o->cpBits_);
  s->pMp = pid(o->mp_); s->pCp = pid(o->cp_); s->pB2s = pid(o->b2s_); s->pParams = pid(o->params_); s->pMapper = pid(o->mapper_);
  s->cpType = CPW(type_); s->cpFlagCp = CPW(flag_); s->cpSuv = CPW(suvVal_); s->cpSent = CPW(cptSent_); s->cpMult = CPW(mult_); s->meMode = MEW(mode_);
  memcpy(s->bits, o->bits_, BITSN);
}
static void store_impl(const S *s) {
  O *o = g_o;
  o->state_ = s->state; o->cnt_ = s->cnt; o->w08_ = s->w08; o->event_ = s->event; o->cpCnt_ = s->cpCnt; o->cpFlag_ = (uint8_t)s->cpFlag;
  o->w20_ = s->w20; o->w24_ = s->w24; o->w28_ = s->w28; o->w2c_ = s->w2c; o->w30_ = s->w30; o->w34_ = s->w34; o->w38_ = s->w38; o->w3c_ = s->w3c;
  o->level_ = (uint16_t)s->level; o->b42_ = (uint8_t)s->b42; o->mult_ = (uint8_t)s->mult; o->limit_ = s->limit; o->inv_ = s->inv;
  o->cpLen_ = s->cpLen; o->cpFrame_ = s->cpFrame; o->w1b8_ = s->w1b8; o->w1bc_ = s->w1bc; o->w1c0_ = s->w1c0; o->w1c4_ = s->w1c4;
  memcpy(o->pad10_, s->pad10, 8); memcpy(o->pad1d_, s->pad1d, 3); memcpy(o->pad1b4_, s->pad1b4, 4);
  o->cpBits_ = s->cpBitsId >= 1 && s->cpBitsId <= NBLK ? g_blk[s->cpBitsId - 1] : s->cpBitsId == 9 ? (void *)g_hist : NULL;
  o->mp_ = idp(s->pMp); o->cp_ = idp(s->pCp); o->b2s_ = idp(s->pB2s); o->params_ = idp(s->pParams); o->mapper_ = idp(s->pMapper);
  CPW(type_) = (uint8_t)s->cpType; CPW(flag_) = (uint8_t)s->cpFlagCp; CPW(suvVal_) = s->cpSuv; CPW(cptSent_) = s->cpSent; CPW(mult_) = (uint8_t)s->cpMult; MEW(mode_) = s->meMode;
  memcpy(o->bits_, s->bits, BITSN);
}

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
enum { NSC = 16, LOGMAX = 24, TXTMAX = 640 };
typedef struct { unsigned nof[NSC], nxt[NSC]; int16_t sym[NSC], msym[NSC], gsym[NSC]; uint8_t sb[NSC]; unsigned cplen[4]; } Script;
typedef struct { unsigned inof, inxt, isym, imsym, igsym, isb, icp, iscr; } Cur;
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Script g_sc;
static Cur g_ic;                       /* the impl's cursor */
static Log g_il;                       /* the impl's records */
static int g_show;
static unsigned long g_free_base;

static unsigned bitgen(unsigned iscr, unsigned i) { return ((iscr * 2654435761u + i * 40503u + 12345u) >> 11) & 1u; }
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
static void snapshot(char *out, size_t cap, const S *s) {
  snprintf(out, cap, "|st=%u cnt=%u w08=%u ev=%u cc=%u cf=%u w20=%u w24=%u w28=%u w2c=%u w30=%u w34=%u w38=%u w3c=%u inv=%u cl=%u cfr=%u cb=%u b8=%u bc=%u c0=%u c4=%u lv=%u b42=%u m=%u lim=%u cp=%u,%u,%u,%u,%u me=%u p=%u,%u,%u,%u,%u bh=%08x",
           s->state, s->cnt, s->w08, s->event, s->cpCnt, s->cpFlag, s->w20, s->w24, s->w28, s->w2c, s->w30, s->w34, s->w38, s->w3c, s->inv, s->cpLen, s->cpFrame, s->cpBitsId,
           s->w1b8, s->w1bc, s->w1c0, s->w1c4, s->level, s->b42, s->mult, s->limit, s->cpType, s->cpFlagCp, s->cpSuv, s->cpSent, s->cpMult, s->meMode, s->pMp, s->pCp, s->pB2s, s->pParams,
           s->pMapper, fnv(s->bits, BITSN));
}
static void rec_add(Log *lg, const S *s, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2], snap[TXTMAX * 2];
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

static const char *mpname(const void *p) { return p == g_mp ? "mp" : p ? "other" : "NULL"; }
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
unsigned char c_scr_p1(void *t, unsigned char b) {
  unsigned char v = g_sc.sb[g_ic.isb++ % NSC];
  CHECK_THIS(t, (void *)&g_o->scr_buf_);
  IMPL_REC('c', "scr.process(%u) -> %u", (unsigned)b, (unsigned)v);
  return v;
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
static void rec_map_ctor(void *t) {
  const HAl *a = h_find_al(t, 0);
  IMPL_REC('c', "map.ctor %s size=%u", a && a->p == (const uint8_t *)t && !a->freed ? "heap" : "OTHER", a ? a->size : 0u);
}
void c_map_ctor1(void *t) { rec_map_ctor(t); }
void c_map_ctor2(void *t) { rec_map_ctor(t); }
static void rec_map_dtor(void *t) {
  IMPL_REC('c', "map.dtor this=%u freed=%lu", pid(t), h_nfree - g_free_base);
}
void c_map_dtor1(void *t) { rec_map_dtor(t); }
void c_map_dtor2(void *t) { rec_map_dtor(t); }
void c_map_reset(void *t, int level, unsigned b) {
  CHECK_THIS(t, g_o->mapper_);
  IMPL_REC('c', "map.reset level=%d b=%u", (int)(int16_t)level, (unsigned)(b & 0xffu));
}
short c_map_proc(void *t, unsigned char *bits) {
  short y = g_sc.msym[g_ic.imsym++ % NSC];
  CHECK_THIS(t, g_o->mapper_);
  IMPL_REC('c', "map.process%s -> %d", bits == g_o->bits_ ? "" : " in=OTHER", (int)y);
  return y;
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
void c_b2s_reset(void *t, void *mp) {
  CHECK_THIS(t, g_b2s);
  IMPL_REC('c', "b2s.reset mp=%s", mpname(mp));
}
unsigned c_b2s_setblk(void *t, unsigned n) {
  CHECK_THIS(t, g_b2s);
  IMPL_REC('c', "b2s.setSymbolsBlockSize %u", n);
  return 0;
}
void c_cp_i2b(void *t) {
  CHECK_THIS(t, g_cp);
  IMPL_REC('c', "CP.infoToBits");
}
unsigned char *c_cp_gbv(void *t, unsigned *len) {
  unsigned k = g_ic.icp++, l = g_sc.cplen[k % 4];
  CHECK_THIS(t, g_cp);
  IMPL_REC('c', "CP.getBitVector len=%s -> blk%u,%u", (void *)len == (void *)&g_o->cpLen_ ? "&cpLen" : "OTHER", k % NBLK, l);
  *len = l;
  return (unsigned char *)g_blk[k % NBLK];
}
/* generateSymbol (P32): a scripted symbol; it advances cnt_ (so that later snapshots differ) */
short c_gensym(void *t) {
  short y = g_sc.gsym[g_ic.igsym++ % NSC];
  CHECK_THIS(t, (void *)g_o);
  IMPL_REC('c', "genSym -> %d", (int)y);
  g_o->cnt_++;
  return y;
}
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
#define NEEDCP(m) do { if (!(m)->s.pCp) CRASH(m); } while (0)
#define NEEDDIV(m, v) do { if ((v) == 0) CRASH(m); } while (0)
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
static void m_dbg(Mdl *m, const char *t) { if (m->dbg > 1) m_rec(m, 'd', "%s", t); }
static void m_dbg_n(Mdl *m, const char *fmt, unsigned a) { if (m->dbg > 1) { char b[TXTMAX]; snprintf(b, sizeof b, fmt, (int)a); m_rec(m, 'd', "%s", b); } }
static unsigned m_nof(Mdl *m) { unsigned v = g_sc.nof[m->c.inof++ % NSC]; m_rec(m, 'c', "b2s.nof -> %u", v); return v; }
static void m_pbits(Mdl *m, unsigned n) { m_rec(m, 'c', "b2s.process(bits,%u) h=%08x", n, n <= BITSN ? fnv(m->s.bits, n) : 0u); }
static void m_pout(Mdl *m, unsigned *n, int16_t *out) {
  unsigned v = g_sc.nxt[m->c.inxt++ % NSC];
  int16_t y = g_sc.sym[m->c.isym++ % NSC];
  m_rec(m, 'c', "b2s.process(&n,&out) -> n=%u out=%d", v, (int)y);
  *n = v; *out = y;
}
static void m_fill(Mdl *m, unsigned n) { unsigned i; for (i = 0; i < n; i++) m->s.bits[i] = (uint8_t)bitgen(m->c.iscr, i); m->c.iscr++; }
static void m_scr_bulk(Mdl *m, int ones, unsigned n) { m_rec(m, 'c', ones ? "scr.ones n=%u out=bits" : "scr.zeros n=%u out=bits", n); m_fill(m, n); }
static unsigned m_scr1(Mdl *m, unsigned b) { unsigned v = g_sc.sb[m->c.isb++ % NSC]; m_rec(m, 'c', "scr.process(%u) -> %u", b, v); return v; }
static int16_t m_map(Mdl *m) { int16_t y = g_sc.msym[m->c.imsym++ % NSC]; m_rec(m, 'c', "map.process -> %d", (int)y); return y; }
static void m_i2b(Mdl *m) { m_rec(m, 'c', "CP.infoToBits"); }
static void m_getbv(Mdl *m) {
  unsigned k = m->c.icp++, l = g_sc.cplen[k % 4];
  m_rec(m, 'c', "CP.getBitVector len=&cpLen -> blk%u,%u", k % NBLK, l);
  m->s.cpLen = l; m->s.cpBitsId = 1 + k % NBLK;
}
/* a CP restart with the CP's own u8 multiplier / this->mult_: infoToBits, the new bit vector; the divisor is read afterwards (0 traps in the blob) */
static int m_cp_restart(Mdl *m, int own, unsigned *f) {
  S *s = &m->s;
  unsigned dv;
  m_i2b(m);
  m_getbv(m);
  dv = own ? s->cpMult : s->mult;
  if (dv == 0) return 0;
  *f = s->cpLen / dv;
  return 1;
}
/* "enter E2u" print + state; fmt differs; w1b8_ from w1bc_ */
static void m_e2u(Mdl *m, const char *fmt) {
  S *s = &m->s;
  unsigned c = s->cnt;
  m_ed(m, fmt, c);
  s->state = 0xf; s->cnt = 0;
  if (s->w1bc == 0) { s->w1b8 = 0xc; CV(G_EV, "enter E2u: w1b8_ = 0xc"); } else { s->w1b8 = 0xd; CV(G_EV, "enter E2u: w1b8_ = 0xd"); }
}
static const char *E2U = "V92Phase4Modulator: enter E2u @ %d\r\n";
static const char *E2UF = "V92Phase4Modulator: enter E2u First at RRN @ %d\r\n";
/* the state after a tag at a non-boundary (recivedCPtag / recivedEd): 5 -> 9; signed below 5: none; 0xc / 0xd -> 0xb; others none.  1 = w20_ = 1 follows */
static int m_tag_state(Mdl *m, unsigned *to) {
  S *s = &m->s;
  int32_t sv = (int32_t)s->state;
  (void)m;
  if (sv == 5) { *to = 9; CV(G_EV, "tag state 5 -> 9"); return 1; }
  if (sv < 5) { CV(G_EV, "tag state below 5 (signed)"); return 0; }
  if (s->state == 0xc || s->state == 0xd) { *to = 0xb; CV(G_EV, "tag state 0xc / 0xd -> 0xb"); return 1; }
  CV(G_EV, "tag state other (6 .. 0xb, > 0xd)");
  return 0;
}
/* the end test of recivedSUV / recivedPartTwoSilenceRrnSUV */
static int m_suv_to_cpu(Mdl *m) {
  S *s = &m->s;
  unsigned c, f;
  if (s->w1c4 != 0) { CV(G_EV, "recivedSUV: w1c4_ set"); return 1; }
  if (s->state != 5) { CV(G_EV, "recivedSUV: not state 5"); return 1; }
  c = s->cnt;
  NEEDDIV(m, s->cpFrame);
  if (c % s->cpFrame != 0) { s->state = 6; CV(G_EV, "recivedSUV: state 6"); return 1; }
  CV(G_EV, "recivedSUV: enter CPu");
  m_ed(m, "V92Phase4Modulator: on recivedSUV enter CPu @ %d\r\n", c);
  s->state = 0xc; s->cnt = 0;
  if (!s->pCp) return 0;
  s->cpType = 0;
  if (!m_cp_restart(m, 1, &f)) return 0;
  s->w1c4 = 1; s->cpFrame = f;
  return 1;
}

/* the block a bit vector id names: only the harness blocks are readable */
static int bits_ok(const S *s, unsigned idx) { return s->cpBitsId >= 1 && s->cpBitsId <= NBLK && idx < BLK; }
/* the scrambled CP bit loop of generateCPu / generateSUVu for i in [0, n) */
static int m_cp_bits(Mdl *m, unsigned n) {
  S *s = &m->s;
  unsigned i;
  for (i = 0; i < n; i++) {
    unsigned bit, r;
    if (!bits_ok(s, s->w08)) return 0;
    bit = ((const uint8_t *)g_blk[s->cpBitsId - 1])[s->w08];
    r = m_scr1(m, bit);
    s->bits[i] = (uint8_t)r;
    if (s->cpLen == 0) return 0;
    s->w08 = (s->w08 + 1u) % s->cpLen;
  }
  return 1;
}
/* the mapper block: the last byte XORed with inv_ (mult_ == 0 would touch bits_[-1]: excluded) */
static int m_mapblock(Mdl *m, int16_t *out) {
  S *s = &m->s;
  uint32_t v;
  if (s->mult == 0 || s->pMapper != 5) return 0;
  v = (uint32_t)s->bits[s->mult - 1] ^ s->inv;
  s->inv = v;
  s->bits[s->mult - 1] = (uint8_t)v;
  *out = m_map(m);
  return 1;
}
static int16_t m_diff(Mdl *m, uint32_t v) {
  m->s.inv = v;
  if (v) { CV(G_GEN, "differential: negative"); return (int16_t)(0u - (uint32_t)(uint16_t)m->s.level); }
  CV(G_GEN, "differential: positive");
  return (int16_t)(uint16_t)m->s.level;
}
static int16_t m_pat(const S *s, int neg) {
  unsigned r = (s->cnt - 1u) % 6u;
  int first = r <= 2;
  int16_t v = (int16_t)s->level;
  return (first != 0) != (neg != 0) ? v : m_neg(v);
}
/* the ones / zeros data step through the queue */
static int16_t m_data_step(Mdl *m, int ones) {
  unsigned n = m_nof(m);
  int16_t y;
  if (n) {
    if (ones) CV(G_GEN, "data ones: bits"); else CV(G_GEN, "data zeros: bits");
    m_scr_bulk(m, ones, n); m_pbits(m, n);
  } else CV(G_GEN, "data step: no bits");
  m_pout(m, &n, &y);
  return y;
}

/* op numbers */
enum {
  O_RESETRRN, O_RESETFPE, O_ENTERREP, O_RU, O_RUNOT, O_RESETRRN2, O_EXITCPT, O_EXITTRN2U, O_RECSUV, O_RECP1, O_RECP2, O_RECCP, O_RECSUVTAG, O_RECP1TAG, O_RECP2TAG, O_RECCPTAG, O_RECED,
  O_RECFIRSTED, O_RECRT, O_SETMAP, O_BEFFPE, O_BEFRRN, O_RM, O_B1U, O_TRN2U, O_E2U, O_CPU, O_SUVU, O_E1U, O_CPT, O_RESET, O_CTOR, O_DTOR, O_N
};
typedef struct { uint32_t u[6]; uintptr_t p[6]; int c2; } Args;
static const char *const k_opname[O_N] = {
  "resetBeforRRN", "resetBeforFPE", "enterRepeatedCP", "generateRu", "generateRuNot", "resetRRNSecondSection", "exitCPt", "exitTRN2u", "recivedSUV", "recivedPartOneSilenceRrnSUV",
  "recivedPartTwoSilenceRrnSUV", "recivedCP", "recivedSUVtag", "recivedPartOneSilenceRrnSUVtag", "recivedPartTwoSilenceRrnSUVtag", "recivedCPtag", "recivedEd", "recivedFirstRrnEd", "recivedRt",
  "setMappingParams", "generateDataSymbolBeforeFPE", "generateDataSymbolBeforeRRN", "generateRm", "generateB1u", "generateTRN2u", "generateE2u", "generateCPu", "generateSUVu", "generateE1u",
  "generateCPt", "reset", "ctor", "dtor"
};

static int16_t mdl_op(Mdl *m, int op, const Args *a) {
  S *s = &m->s;
  unsigned c, f, to;
  int16_t y = 0;
  switch (op) {
    case O_RESETRRN:
      NEEDCP(m);
      s->w1c4 = 0; s->w1c0 = 0; s->cnt = 0; s->cpSent = 0; s->w28 = 1; s->w2c = 0; s->w30 = 0; s->w34 = 0; s->w38 = 0; s->w20 = 0;
      CV(G_EV, "resetBeforRRN");
      return 0;
    case O_RESETFPE: s->cnt = 0; s->w3c = 1; CV(G_EV, "resetBeforFPE"); return 0;
    case O_ENTERREP:
      s->cpFlag = 0; s->cpCnt = 0;
      if (m->dbg > 1) { CV(G_EV, "enterRepeatedCP: print"); m_dbg_n(m, "V92Phase4Modulator: enter repeatedCPu @ %d\r\n", s->cnt); } else CV(G_EV, "enterRepeatedCP: quiet");
      s->state = 0xd;
      NEEDCP(m);
      s->cpType = 0;
      if (!m_cp_restart(m, 1, &f)) CRASH(m);
      s->cnt = 0; s->cpFrame = f;
      return 0;
    case O_RU: CV(G_GEN, "generateRu"); return m_pat(s, 0);
    case O_RUNOT: CV(G_GEN, "generateRuNot"); return m_pat(s, 1);
    case O_RESETRRN2:
      NEEDCP(m);
      CV(G_EV, "resetRRNSecondSection");
      s->w1c4 = 0; s->w34 = 1; s->w20 = 0; s->cpFlag = 0; s->cpCnt = 0; s->w1c0 = 0; s->cpSent = 0; s->cpFlagCp = 0;
      return 0;
    case O_EXITCPT:
      if (s->state != 0) { CV(G_EV, "exitCPt: other state"); return 0; }
      c = s->cnt;
      if (c == 0) { CV(G_EV, "exitCPt: count 0"); return 0; }
      NEEDDIV(m, s->cpFrame);
      if ((c - 0x18u) % s->cpFrame != 0) { s->state = 1; CV(G_EV, "exitCPt: state 1"); return 0; }
      CV(G_EV, "exitCPt: enter E1u");
      m_ed(m, "V92Phase4Modulator: enter E1u @ %d\r\n", c);
      s->state = 2; s->cnt = 0;
      return 0;
    case O_EXITTRN2U:
      if (s->state != 3) { CV(G_EV, "exitTRN2u: other state"); return 0; }
      if (s->cnt == 0) { CV(G_EV, "exitTRN2u: count 0"); return 0; }
      CV(G_EV, "exitTRN2u: state 4");
      s->state = 4;
      return 0;
    case O_RECSUV: case O_RECP2:
      if (!m_suv_to_cpu(m)) CRASH(m);
      return 0;
    case O_RECP1: NEEDCP(m); s->cpFlagCp = 1; CV(G_EV, "recP1"); return 0;
    case O_RECCP: NEEDCP(m); s->w1c0 = 1; s->cpFlagCp = 1; s->cpSent = 0; CV(G_EV, "recivedCP"); return 0;
    case O_RECSUVTAG: case O_RECP2TAG: {
      int five;
      s->cpFlag = 0; s->cpCnt = 0;
      if (m->dbg > 1) { CV(G_EV, "SUVtag: print"); m_dbg(m, "recivedSUVtag called\r\n"); } else CV(G_EV, "SUVtag: quiet");
      if (s->w1c0 == 0) { CV(G_EV, "SUVtag: w1c0_ clear"); return 0; }
      NEEDCP(m);
      if (s->cpSent == 0) { CV(G_EV, "SUVtag: cptSent_ clear"); return 0; }
      if (s->w20 != 0) { CV(G_EV, "SUVtag: w20_ set"); return 0; }
      five = (int32_t)s->state == 5;
      if (!five) {
        if ((int32_t)s->state < 5) { CV(G_EV, "SUVtag: state below 5 (signed)"); return 0; }
        if (!(s->state == 0xc || s->state == 0xd)) { CV(G_EV, "SUVtag: state other"); return 0; }
      }
      c = s->cnt;
      NEEDDIV(m, s->cpFrame);
      if (c % s->cpFrame != 0) { s->state = five ? 9u : 0xbu; if (five) CV(G_EV, "SUVtag: state 5 -> 9"); else CV(G_EV, "SUVtag: state 0xc / 0xd -> 0xb"); }
      else { CV(G_EV, "SUVtag: enter E2u"); m_e2u(m, E2U); }
      s->w20 = 1;
      return 0;
    }
    case O_RECP1TAG:
      NEEDCP(m);
      if (s->w20 == 0) {
        c = s->cnt;
        if (s->cpFlagCp == 1) {
          NEEDDIV(m, s->cpFrame);
          if (c % s->cpFrame == 0) { CV(G_EV, "P1tag: flag 1, enter E2u First"); m_e2u(m, E2UF); }
          else { CV(G_EV, "P1tag: flag 1, state 9"); s->state = 9; }
          s->w20 = 1;
        } else {
          s->cpFlagCp = 1;
          NEEDDIV(m, s->cpFrame);
          if (c % s->cpFrame == 0) {
            CV(G_EV, "P1tag: flag 0, boundary: SUV CP restart");
            s->state = 0xa; s->w20 = 1; s->cnt = 0; s->cpType = 1;
            if (!m_cp_restart(m, 1, &f)) CRASH(m);
            s->cpFrame = f;
          } else { CV(G_EV, "P1tag: flag 0, state 8"); s->state = 8; s->w20 = 1; }
        }
      } else CV(G_EV, "P1tag: w20_ set");
      s->cpFlagCp = 1;
      return 0;
    case O_RECCPTAG:
      s->cpFlag = 0; s->cpCnt = 0;
      if (s->w1c0 == 0) {
        if (s->w20 != 0) { CV(G_EV, "CPtag: first, w20_ set"); return 0; }
        NEEDCP(m);
        s->w1c0 = 1; s->cpFlagCp = 1; s->cpType = 1;
        NEEDDIV(m, s->cpFrame);
        if (s->cnt % s->cpFrame == 0) {
          CV(G_EV, "CPtag: first, boundary: state 0xa");
          s->state = 0xa; s->cnt = 0;
          if (!m_cp_restart(m, 1, &f)) CRASH(m);
          s->cpFrame = f;
        } else { CV(G_EV, "CPtag: first, state 8"); s->state = 8; }
        s->w20 = 1;
        return 0;
      }
      if (s->w20 != 0) { CV(G_EV, "CPtag: w20_ set"); return 0; }
      c = s->cnt;
      NEEDDIV(m, s->cpFrame);
      if (c % s->cpFrame == 0) { CV(G_EV, "CPtag: enter E2u"); m_e2u(m, E2U); s->w20 = 1; return 0; }
      if (m_tag_state(m, &to)) { s->state = to; s->w20 = 1; }
      return 0;
    case O_RECED:
      if (s->w20 != 0) { CV(G_EV, "recivedEd: w20_ set"); return 0; }
      c = s->cnt;
      NEEDDIV(m, s->cpFrame);
      if (c % s->cpFrame == 0) { CV(G_EV, "recivedEd: enter E2u"); m_e2u(m, E2U); s->w20 = 1; return 0; }
      if (m_tag_state(m, &to)) { s->state = to; s->w20 = 1; }
      return 0;
    case O_RECFIRSTED:
      if (s->w20 != 0) { CV(G_EV, "FirstRrnEd: w20_ set"); return 0; }
      c = s->cnt;
      NEEDDIV(m, s->cpFrame);
      if (c % s->cpFrame != 0) { s->state = 9; s->w20 = 1; CV(G_EV, "FirstRrnEd: state 9"); return 0; }
      m_ed(m, E2UF, c);
      s->state = 0xf; f = s->w1bc; s->cnt = 0; s->w1b8 = 0xc;
      if (f != 0) { CV(G_EV, "FirstRrnEd: extended error"); m_rec(m, 'e', "V92Phase4Modulator: ERROR: E2u is extended in RRN !!!\r\n"); } else CV(G_EV, "FirstRrnEd: ok");
      s->w20 = 1;
      return 0;
    case O_RECRT:
      if (s->state != 0x17) { CV(G_EV, "recivedRt: other state"); return 0; }
      if (s->w38 == 0) { CV(G_EV, "recivedRt: w38_ clear"); return 0; }
      c = s->cnt;
      if (c <= 0x95fu || c != (c / 12u) * 12u) { s->state = 0x18; CV(G_EV, "recivedRt: state 0x18"); return 0; }
      if (!s->pCp) {
        s->state = 0x1d; s->cnt = 0;
        if (m->dbg > 1) { CV(G_EV, "recivedRt: no CP, print"); m_dbg(m, "V92Phase4Modulator: ERROR: Null CP at RRN @ end of TRN2d\r\n"); } else CV(G_EV, "recivedRt: no CP, quiet");
        return 0;
      }
      CV(G_EV, "recivedRt: enter SUV");
      m_ed(m, "V92Phase4Modulator: on recivedRt enter SUV @ %d\r\n", c);
      s->cnt = 0; s->w08 = 0; s->state = 5; s->cpMult = s->mult; s->cpType = 1; s->cpSuv = 0;
      if (!m_cp_restart(m, 0, &f)) CRASH(m);
      s->cpFrame = f;
      return 0;
    case O_SETMAP:
      if (a->p[0] == 0) {
        if (m->dbg > 1) { CV(G_GEN, "setMappingParams: NULL, print"); m_dbg(m, "V92Phase4Modulator: ERROR: Null mappingParams @ setMappingParams\r\n"); } else CV(G_GEN, "setMappingParams: NULL, quiet");
        return 0;
      }
      CV(G_GEN, "setMappingParams: ok");
      m_rec(m, 'c', "b2s.reset mp=%s", mpname((void *)a->p[0]));
      m_rec(m, 'c', "b2s.setSymbolsBlockSize 1");
      return 0;
    case O_BEFFPE: case O_BEFRRN: {
      unsigned n;
      m_pout(m, &n, &y);
      if (n) {
        if (op == O_BEFFPE) { CV(G_GEN, "BeforeFPE: n != 0"); m_ed(m, "V92Phase4Modulator: enter Rm @ %d\r\n", s->cnt); s->state = 0x1a; s->cnt = 0; s->meMode = 1; }
        else { CV(G_GEN, "BeforeRRN: n != 0"); m_ed(m, "V92Phase4Modulator: enter Ru @ %d\r\n", s->cnt); s->state = 0x13; s->cnt = 0; }
      } else { if (op == O_BEFFPE) CV(G_GEN, "BeforeFPE: n == 0"); else CV(G_GEN, "BeforeRRN: n == 0"); }
      return y;
    }
    case O_RM: case O_B1U: return m_data_step(m, 1);
    case O_TRN2U:
      NEEDDIV(m, s->mult);
      CV(G_GEN, "generateTRN2u");
      m_scr_bulk(m, 1, s->mult);
      if (!m_mapblock(m, &y)) CRASH(m);
      return y;
    case O_E2U:
      if (s->w3c == 0) {
        NEEDDIV(m, s->mult);
        CV(G_GEN, "generateE2u: mapper path");
        m_scr_bulk(m, 0, s->mult);
        if (!m_mapblock(m, &y)) CRASH(m);
        return y;
      }
      CV(G_GEN, "generateE2u: queue path");
      return m_data_step(m, 0);
    case O_CPU: case O_SUVU:
      if (s->w3c == 0) {
        CV(G_GEN, "CPu / SUVu: mapper path");
        if (!m_cp_bits(m, s->mult)) CRASH(m);
        if (!m_mapblock(m, &y)) CRASH(m);
        return y;
      } else {
        unsigned n = m_nof(m);
        if (n) {
          CV(G_GEN, "CPu / SUVu: queue path, bits");
          if (!m_cp_bits(m, n)) CRASH(m);
          m_pbits(m, n);
        } else CV(G_GEN, "CPu / SUVu: queue path, no bits");
        m_pout(m, &n, &y);
        return y;
      }
    case O_E1U: {
      unsigned r = m_scr1(m, 0) & 0xffu;
      CV(G_GEN, "generateE1u");
      return m_diff(m, r ^ s->inv);
    }
    case O_CPT:
      c = s->cnt;
      if (c <= 0x18u) { CV(G_GEN, "generateCPt: toggle"); return m_diff(m, s->inv ^ 1u); }
      else {
        unsigned idx, bit, r;
        NEEDDIV(m, s->cpLen);
        idx = (c - 0x19u) % s->cpLen;
        if (!bits_ok(s, idx)) CRASH(m);
        CV(G_GEN, "generateCPt: bits");
        bit = ((const uint8_t *)g_blk[s->cpBitsId - 1])[idx];
        r = m_scr1(m, bit) & 0xffu;
        return m_diff(m, r ^ s->inv);
      }
    case O_RESET: {
      unsigned b = a->u[1] & 0xffu, n = a->u[3], i;
      s->event = 0; s->limit = a->u[4]; s->level = a->u[0] & 0xffffu; s->b42 = b; s->state = a->u[2]; s->mult = (b + 2u) & 0xffu; s->cnt = 0;
      if (s->pMapper != 5) CRASH(m);
      m_rec(m, 'c', "map.reset level=%d b=%u", (int)(int16_t)s->level, b);
      m_rec(m, 'c', "scr.reset v=0");
      s->inv = 0;
      NEEDCP(m);
      s->w1c0 = 0; s->cpSent = 0; s->w1c4 = 0; s->w28 = 0; s->w3c = 0; s->w2c = 0; s->w30 = 0; s->w34 = 0; s->cpCnt = 0; s->cpFlag = 0; s->w20 = 0;
      s->cpMult = 1; s->cpType = 0;
      m_i2b(m);
      m_getbv(m);
      s->w1bc = 0;
      if (n == 0) CV(G_LIFE, "reset: no loop"); else CV(G_LIFE, "reset: generateSymbol loop");
      for (i = 0; i < n; i++) {
        int16_t g = g_sc.gsym[m->c.igsym++ % NSC];
        m_rec(m, 'c', "genSym -> %d", (int)g);
        s->cnt++;
      }
      if (b >= 0xfeu) CV(G_LIFE, "reset: mult_ wraps");
      if ((int16_t)s->level < 0) CV(G_LIFE, "reset: negative level");
      return 0;
    }
    case O_CTOR:   /* the harness computed the allocation (m->alloc) from the impl run */
      m_rec(m, 'c', "scr.ctor %u %u %u", 5u, 0x17u, 0x63u);
      m_rec(m, 'c', "map.ctor heap size=%u", 0x2cu);
      s->pMapper = 7; s->w1c0 = 0; s->pMp = pid((void *)a->p[2]); s->pParams = pid((void *)a->p[0]); s->w1c4 = 0; s->pCp = pid((void *)a->p[1]); s->cpSent = 0;
      s->pB2s = pid((void *)a->p[3]); s->cpCnt = 0; s->cpFlag = 0;
      m->nalloc = 1;
      CV(G_LIFE, "ctor");
      if (!s->pMp) CV(G_LIFE, "ctor: mp NULL");
      if (!s->pB2s) CV(G_LIFE, "ctor: b2s NULL");
      return 0;
    case O_DTOR:
      if (s->pMapper != 0) {
        CV(G_LIFE, "dtor: mapper freed");
        m_rec(m, 'c', "map.dtor this=%u freed=0", s->pMapper);
        m->nfree_exp = 1;
      } else CV(G_LIFE, "dtor: no mapper");
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
#define CMPF(f) do { if (a->f != b->f) h_die3("%s: %s: model %llu (0x%llx), impl %llu (0x%llx)", g_what, #f, (unsigned long long)a->f, (unsigned long long)a->f, (unsigned long long)b->f, (unsigned long long)b->f); } while (0)
static void cmp_S(const S *a, const S *b) {
  unsigned i;
  CMPF(state); CMPF(cnt); CMPF(w08); CMPF(event); CMPF(cpCnt); CMPF(w20); CMPF(w24); CMPF(w28); CMPF(w2c); CMPF(w30); CMPF(w34); CMPF(w38); CMPF(w3c); CMPF(limit); CMPF(inv); CMPF(cpLen); CMPF(cpFrame);
  CMPF(w1b8); CMPF(w1bc); CMPF(w1c0); CMPF(w1c4); CMPF(cpFlag); CMPF(level); CMPF(b42); CMPF(mult); CMPF(cpBitsId); CMPF(pMp); CMPF(pCp); CMPF(pB2s); CMPF(pParams); CMPF(pMapper);
  CMPF(cpType); CMPF(cpFlagCp); CMPF(cpSuv); CMPF(cpSent); CMPF(cpMult); CMPF(meMode);
  if (memcmp(a->pad10, b->pad10, 8) || memcmp(a->pad1d, b->pad1d, 3) || memcmp(a->pad1b4, b->pad1b4, 4)) h_die3("%s: a never-touched pad changed", g_what);
  for (i = 0; i < BITSN; i++) if (a->bits[i] != b->bits[i]) h_die3("%s: bits_[%u]: model %u impl %u", g_what, i, a->bits[i], b->bits[i]);
}
static HFld g_ft[96];
static int g_nft;
static void dump_obj(const char *tag, int show) {
  if (!g_nft) {
    int i;
    g_nft = NF_V92Ph4Obj;
    if (g_nft > 96) h_die3("field table");
    memcpy(g_ft, f_V92Ph4Obj, sizeof(HFld) * (size_t)g_nft);
    for (i = 0; i < g_nft; i++) if (!strcmp(g_ft[i].name, "bits_")) { g_ft[i].kind = 'x'; g_ft[i].count = BITSN; }   /* 300 scrambled bytes: hashed, not listed */
  }
  h_dump_obj(show, tag, g_o, g_ft, g_nft);
  g_dumpctr++;
}
/* the objects the callees get: saved before an operation, compared after (nothing but the CP's five words and the modulus encoder's mode_ may change) */
static uint8_t sv_b2s[0x40], sv_tx[0x80], sv_me[0x60], sv_map[0x40], sv_mp[0x100], sv_cp[0x1000], sv_blk[NBLK][BLK], sv_par[0x40];
static void save_blocks(void) {
  int k;
  memcpy(sv_b2s, g_b2s, sizeof sv_b2s); memcpy(sv_tx, g_tx, sizeof sv_tx); memcpy(sv_me, g_me, sizeof sv_me); memcpy(sv_map, g_mapper, sizeof sv_map);
  memcpy(sv_mp, g_mp, sizeof sv_mp); memcpy(sv_cp, g_cp, sizeof sv_cp); memcpy(sv_par, g_params, sizeof sv_par);
  for (k = 0; k < NBLK; k++) memcpy(sv_blk[k], g_blk[k], BLK);
}
static void check_blocks(const S *model) {
  int k;
  uint8_t exp[0x1000], expme[0x60];
  if (memcmp(sv_b2s, g_b2s, sizeof sv_b2s)) h_die3("%s: the V92BitsToSymbol block was written", g_what);
  if (memcmp(sv_tx, g_tx, sizeof sv_tx)) h_die3("%s: the V92Transmitter block was written", g_what);
  if (memcmp(sv_map, g_mapper, sizeof sv_map)) h_die3("%s: the V92Mapper block was written", g_what);
  if (memcmp(sv_mp, g_mp, sizeof sv_mp)) h_die3("%s: the mapping parameters were written", g_what);
  if (memcmp(sv_par, g_params, sizeof sv_par)) h_die3("%s: the V92Parameters block was written", g_what);
  for (k = 0; k < NBLK; k++) if (memcmp(sv_blk[k], g_blk[k], BLK)) h_die3("%s: a bit block was written", g_what);
  memcpy(exp, sv_cp, sizeof exp);
  ((V92CpObj *)exp)->type_ = (uint8_t)model->cpType; ((V92CpObj *)exp)->flag_ = (uint8_t)model->cpFlagCp; ((V92CpObj *)exp)->suvVal_ = model->cpSuv; ((V92CpObj *)exp)->cptSent_ = model->cpSent;
  ((V92CpObj *)exp)->mult_ = (uint8_t)model->cpMult;
  if (memcmp(exp, g_cp, sizeof(V92CpObj))) h_die3("%s: the CP block differs from the model beyond its five words", g_what);
  memcpy(expme, sv_me, sizeof expme);
  ((V92ModEncObj *)expme)->mode_ = model->meMode;
  if (memcmp(expme, g_me, sizeof expme)) h_die3("%s: the modulus-encoder block differs from the model beyond mode_", g_what);
}
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

static int is_short_op(int op) {
  switch (op) {
    case O_RU: case O_RUNOT: case O_BEFFPE: case O_BEFRRN: case O_RM: case O_B1U: case O_TRN2U: case O_E2U: case O_CPU: case O_SUVU: case O_E1U: case O_CPT: return 1;
    default: return 0;
  }
}
static int16_t call_impl(int op, const Args *a) {
  void *o = g_o;
  switch (op) {
    case O_RESETRRN: f_resetRRN(o); return 0;
    case O_RESETFPE: f_resetFPE(o); return 0;
    case O_ENTERREP: f_enterRep(o); return 0;
    case O_RU: return f_ru(o);
    case O_RUNOT: return f_ruNot(o);
    case O_RESETRRN2: f_resetRRN2(o); return 0;
    case O_EXITCPT: f_exitCPt(o); return 0;
    case O_EXITTRN2U: f_exitTRN2u(o); return 0;
    case O_RECSUV: f_recSUV(o); return 0;
    case O_RECP1: f_recP1(o); return 0;
    case O_RECP2: f_recP2(o); return 0;
    case O_RECCP: f_recCP(o); return 0;
    case O_RECSUVTAG: f_recSUVtag(o); return 0;
    case O_RECP1TAG: f_recP1tag(o); return 0;
    case O_RECP2TAG: f_recP2tag(o); return 0;
    case O_RECCPTAG: f_recCPtag(o); return 0;
    case O_RECED: f_recEd(o); return 0;
    case O_RECFIRSTED: f_recFirstEd(o); return 0;
    case O_RECRT: f_recRt(o); return 0;
    case O_SETMAP: f_setMap(o, (void *)a->p[0]); return 0;
    case O_BEFFPE: return f_befFPE(o);
    case O_BEFRRN: return f_befRRN(o);
    case O_RM: return f_rm(o);
    case O_B1U: return f_b1u(o);
    case O_TRN2U: return f_trn2u(o);
    case O_E2U: return f_e2u(o);
    case O_CPU: return f_cpu(o);
    case O_SUVU: return f_suvu(o);
    case O_E1U: return f_e1u(o);
    case O_CPT: return f_cpt(o);
    case O_RESET: f_reset(o, (int)a->u[0], a->u[1], a->u[2], a->u[3], a->u[4]); return 0;
    case O_CTOR:   /* p[0] params, p[1] cp, p[2] mp, p[3] b2s */
      if (a->c2) f_ctor2(o, (void *)a->p[0], (void *)a->p[3], (void *)a->p[1], (void *)a->p[2]);
      else f_ctor(o, (void *)a->p[0], (void *)a->p[3], (void *)a->p[1], (void *)a->p[2]);
      return 0;
    case O_DTOR: if (a->c2) f_dtor2(o); else f_dtor(o); return 0;
    default: h_die3("op %d", op);
  }
  return 0;
}

/* one operation: model first (a call that would crash the blob is skipped), then the object; the constructor runs the object first (the model needs the allocation) */
static unsigned long g_symctr;
static int do_op(int op, const Args *a) {
  Mdl m;
  S cur, ns;
  int16_t ms = 0, is = 0;
  unsigned long nm0, nf0;
  g_what = k_opname[op];
  m = g_m;
  m.l.n = 0; m.crash = 0; m.alloc = NULL; m.nalloc = 0; m.nfree_exp = 0;
  m.dbg = dsplibs_debug_level;
  m.c.igsym = 0;
  g_ic.igsym = 0;
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
    if (h_nmalloc - nm0 != 1) h_die3("ctor: %lu allocations", h_nmalloc - nm0);
    g_mblk = h_al[h_last_alloc].p;
    m.alloc = g_mblk;
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
  h_emit(g_show, "op %s -> %d st=%u cnt=%u", g_what, (int)is, ns.state, ns.cnt);
  dump_obj("op", g_show);
  cov_flush();
  g_nops++;
  return 1;
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(55) ? 2u : h_rpct(30) ? (h_rpct(50) ? 0u : 1u) : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static unsigned pick_pat32(void) {
  static const unsigned v[] = { 0, 1, 2, 0x7fffffffu, 0x80000000u, 0xffffffffu, 0xfffffffeu, 0x18, 0x180, 0x240 };
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
    g_sc.msym[i] = h_rpct(8) ? (int16_t)(h_rpct(50) ? -32768 : 32767) : (int16_t)h_rnd();
    g_sc.gsym[i] = h_rpct(8) ? (int16_t)(h_rpct(50) ? -32768 : 32767) : (int16_t)h_rnd();
    g_sc.sb[i] = (uint8_t)(h_rpct(40) ? (h_rnd() & 1u) : h_rnd());
  }
  for (i = 0; i < 4; i++) g_sc.cplen[i] = h_rpct(20) ? rnd_range(2, 5) : rnd_range(2, 3000);
}
static void alloc_fakes(void) {
  int k;
  if (sizeof(V92CpObj) > 0x1000) h_die3("a fake object block is too small");
  g_b2s = sysdep_malloc(0x40); g_tx = sysdep_malloc(0x80); g_me = sysdep_malloc(0x60); g_mapper = sysdep_malloc(0x40); g_mp = sysdep_malloc(0x100); g_cp = sysdep_malloc(0x1000);
  g_params = sysdep_malloc(0x40); g_hist = sysdep_malloc(0x40);
  for (k = 0; k < NBLK; k++) { g_blk[k] = sysdep_malloc(BLK); }
  fill_random(g_b2s, 0x40); fill_random(g_tx, 0x80); fill_random(g_me, 0x60); fill_random(g_mapper, 0x40); fill_random(g_mp, 0x100); fill_random(g_cp, 0x1000); fill_random(g_params, 0x40); fill_random(g_hist, 0x40);
  for (k = 0; k < NBLK; k++) { uint8_t *q = (uint8_t *)g_blk[k]; unsigned i; for (i = 0; i < BLK; i++) q[i] = (uint8_t)(h_rpct(50) ? (h_rnd() & 1u) : h_rnd()); }
  ((B2S92 *)g_b2s)->tx_ = g_tx;                 /* the chain V92BitsToSymbol -> V92Transmitter -> V92ModulusEncoder (mode_) */
  ((TxObj *)g_tx)->me_ = g_me;
  g_mblk = NULL;
}
static void free_fakes(void) {
  int k;
  sysdep_free(g_b2s); sysdep_free(g_tx); sysdep_free(g_me); sysdep_free(g_mapper); sysdep_free(g_mp); sysdep_free(g_cp); sysdep_free(g_params); sysdep_free(g_hist);
  for (k = 0; k < NBLK; k++) sysdep_free(g_blk[k]);
}
/* the object: random in every field (architecture independent: the table drives the fill) */
static void prefill_obj(void) {
  uint8_t *o = (uint8_t *)g_o;
  int i;
  unsigned k, n;
  memset(o, 0, SZ);
  for (i = 0; i < NF_V92Ph4Obj; i++) {
    const HFld *f = &f_V92Ph4Obj[i];
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
  static const unsigned like[] = { 0, 3, 5, 5, 5, 6, 8, 9, 0xa, 0xb, 0xc, 0xc, 0xd, 0xd, 0xf, 0x17, 0x17, 0x18, 1, 2, 4, 7, 0xe, 0x10, 0x13, 0x1a, 0x1d };
  if (h_rpct(8)) return pick_pat32();
  if (h_rpct(8)) return rnd_range(0, 40);
  if (h_rpct(8)) return like[h_rmod(sizeof like / sizeof like[0])] | (h_rpct(50) ? 0x10000u : 0x80000000u);   /* garbage in the upper bits of the state: signed / unsigned compares */
  return like[h_rmod(sizeof like / sizeof like[0])];
}
/* the symbol counter: around the frame boundaries, 0x18 + k * frame, the 0x95f / 12 threshold of recivedRt, the six-symbol boundary (the table is the harness' own steering, not the model) */
static unsigned pick_cnt(const S *s) {
  unsigned cf = s->cpFrame, r = h_rmod(100), k = 1 + h_rmod(4), c;
  int d = h_rpct(25) ? (int)h_rmod(3) - 1 : 0;
  if (r < 30) c = k * cf;
  else if (r < 42) c = 0x18u + k * cf;
  else if (r < 50) c = 0x18u;
  else if (r < 64) c = 0x95fu + 12u * h_rmod(3) + (h_rpct(30) ? 1u : 0u);
  else if (r < 72) c = 6u * k;
  else if (r < 78) c = 0;
  else if (r < 82) c = (h_rpct(50) ? 0xfffffffcu : 0xfffffff4u) - 6u * h_rmod(4);
  else c = h_rpct(50) ? rnd_range(0, 60) : pick_pat32();
  return c + (unsigned)d;
}
static void init_state(S *s) {
  unsigned lim;
  memset(s, 0, sizeof *s);
  s->state = pick_state();
  s->event = pick_pat32();
  s->cpFlag = h_rpct(50) ? 1u : (h_rpct(40) ? 0u : (h_rpct(50) ? 2u : 0xffu));
  lim = h_rpct(30) ? (0xfffffce0u + h_rmod(4) - 1u) : h_rpct(50) ? rnd_range(0, 1000) : pick_pat32();
  s->limit = lim;
  s->cpCnt = h_rpct(50) ? lim + 0x320u + (unsigned)(int)h_rmod(3) - 1u : pick_pat32();
  s->w20 = pick01(); s->w1c0 = pick01(); s->w1c4 = pick01(); s->w2c = pick_pat32();
  s->w28 = pick_pat32(); s->w30 = pick_pat32(); s->w34 = pick_pat32(); s->w24 = pick_pat32();
  s->w38 = h_rpct(50) ? 0u : pick_pat32(); s->w3c = h_rpct(50) ? 0u : pick_pat32();
  s->w1bc = pick01();
  s->w1b8 = h_rpct(50) ? 0xcu + h_rmod(2) : rnd_range(1, 20);
  s->inv = h_rpct(30) ? h_rmod(3) : pick_pat32();
  { static const unsigned a[] = { 0, 1, 0x7fff, 0x8000, 0xffff, 0x1234, 0xfffe }; s->level = h_rpct(50) ? a[h_rmod(7)] : h_rnd() & 0xffffu; }
  s->b42 = h_rnd() & 0xffu;
  s->mult = h_rpct(3) ? 0u : h_rpct(5) ? 1u : h_rpct(75) ? rnd_range(2, 40) : rnd_range(1, 255);
  fill_random(s->pad10, 8); fill_random(s->pad1d, 3); fill_random(s->pad1b4, 4);
  s->cpLen = h_rpct(2) ? 0u : rnd_range(2, 3000);
  s->w08 = h_rpct(85) ? (s->cpLen ? h_rmod(s->cpLen + 3u) : 0u) : pick_pat32();
  s->cpFrame = h_rpct(1) ? 0u : h_rpct(80) ? rnd_range(1, 9) : rnd_range(1, 100);
  s->cpBitsId = h_rpct(2) ? 0u : h_rpct(2) ? 9u : 1u + h_rmod(NBLK);
  s->pMp = h_rpct(85) ? 2u : 0u; s->pCp = h_rpct(92) ? 3u : 0u; s->pB2s = 1; s->pParams = 4; s->pMapper = h_rpct(94) ? 5u : h_rpct(50) ? 0u : 9u;
  s->cpType = pick_pat32() & 0xffu; s->cpFlagCp = h_rpct(40) ? 1u : h_rpct(50) ? 0u : (h_rnd() & 0xffu); s->cpSuv = pick_pat32(); s->cpSent = h_rpct(45) ? 0u : pick_pat32();
  s->cpMult = h_rpct(2) ? 0u : rnd_range(1, 255); s->meMode = pick_pat32();
  fill_random(s->bits, BITSN);
  s->cnt = pick_cnt(s);
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
  g_o->scr_buf_ = g_hist; g_o->scr_head0_ = (char *)g_hist + 4; g_o->scr_t1_0_ = (char *)g_hist + 8; g_o->scr_end_ = (char *)g_hist + 16;
  g_o->scr_wr_ = (char *)g_hist + 4; g_o->scr_t1_ = (char *)g_hist + 8; g_o->scr_t2_ = (char *)g_hist + 16; g_o->scr_tail_ = 17;
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
  h_trial_end("p33");
}
/* the state re-aimed between operations (the model follows) */
static void reaim(void) {
  S t = g_m.s, n;
  init_state(&n);
  if (h_rpct(70)) t.cnt = pick_cnt(&t);
  if (h_rpct(60)) t.state = pick_state();
  if (h_rpct(35)) { t.w20 = n.w20; t.w1c0 = n.w1c0; t.w1c4 = n.w1c4; t.cpSent = n.cpSent; }
  if (h_rpct(25)) { t.cpFrame = n.cpFrame; t.cpLen = n.cpLen; t.w08 = n.w08; t.cpBitsId = n.cpBitsId; }
  if (h_rpct(20)) { t.cpFlagCp = n.cpFlagCp; t.w1bc = n.w1bc; t.w38 = n.w38; t.w3c = n.w3c; }
  if (h_rpct(15)) { t.mult = n.mult; t.cpMult = n.cpMult; }
  store_impl(&t);
  g_m.s = t;
  if (h_rpct(15)) { dsplibs_debug_level = pick_level(); g_m.dbg = dsplibs_debug_level; }
  if (h_rpct(30)) gen_script();
}
static void gen_args(int op, Args *a) {
  memset(a, 0, sizeof *a);
  switch (op) {
    case O_SETMAP: a->p[0] = h_rpct(30) ? 0u : (uintptr_t)g_mp; break;
    case O_RESET: {
      static const unsigned bs[] = { 0, 1, 2, 3, 0xfd, 0xfe, 0xff, 0x7f, 0x80 };
      a->u[0] = h_rpct(30) ? (h_rnd() & 0xffffu) | (h_rnd() & 0xffff0000u) : h_rnd() & 0xffffu;
      a->u[1] = h_garbage_hi(h_rpct(30) ? bs[h_rmod(sizeof bs / sizeof bs[0])] : h_rnd() & 0x3fu);
      a->u[2] = pick_state();
      a->u[3] = h_rpct(30) ? 0u : rnd_range(1, 5);
      a->u[4] = pick_pat32();
      break;
    }
    default: break;
  }
}
static const int k_ev[] = { O_RESETRRN, O_RESETFPE, O_ENTERREP, O_RESETRRN2, O_EXITCPT, O_EXITTRN2U, O_RECSUV, O_RECP1, O_RECP2, O_RECCP, O_RECSUVTAG, O_RECP1TAG, O_RECP2TAG, O_RECCPTAG, O_RECED,
  O_RECFIRSTED, O_RECRT, O_RECSUV, O_RECSUVTAG, O_RECCPTAG, O_RECRT, O_EXITCPT };
static const int k_gen[] = { O_RU, O_RUNOT, O_SETMAP, O_BEFFPE, O_BEFRRN, O_RM, O_B1U, O_TRN2U, O_E2U, O_CPU, O_SUVU, O_E1U, O_CPT, O_CPU, O_SUVU, O_CPT, O_E2U };
static const int k_life[] = { O_RESET };
/* steering of the narrow paths: recivedRt needs state 0x17, w38_ != 0 and a count above 0x95f that is a multiple of 12 (and no CP for the error path) */
static void aim(int op) {
  S t = g_m.s;
  static const unsigned tg[] = { 5, 5, 0xc, 0xd, 8, 6 };
  if (!h_rpct(60)) return;
  switch (op) {
    case O_RECRT:
      t.state = 0x17; t.w38 = h_rpct(85) ? pick_pat32() | 1u : 0u;
      t.cnt = h_rpct(75) ? 0x960u + 12u * h_rmod(4) : h_rpct(50) ? 0x95fu : 0x960u - 12u * h_rmod(3);
      if (h_rpct(18)) t.cnt = h_rpct(50) ? 0xfffffffcu - 12u * h_rmod(3) : 0x80000004u + 12u * h_rmod(3);   /* multiples of 12 with the top bit set: the signed compare against 0x95f differs */
      if (h_rpct(30)) t.pCp = 0;
      break;
    case O_RECSUVTAG: case O_RECP2TAG:
      t.w1c0 = 1; t.cpSent = pick_pat32() | 1u; t.w20 = 0; t.state = tg[h_rmod(6)];
      if (t.cpFrame) t.cnt = h_rpct(60) ? t.cpFrame * (1u + h_rmod(3)) : t.cpFrame * (1u + h_rmod(3)) + 1u;
      break;
    case O_EXITCPT:
      t.state = 0; t.cnt = t.cpFrame ? 0x18u + t.cpFrame * h_rmod(4) + (h_rpct(30) ? 1u : 0u) : 5u;
      break;
    case O_EXITTRN2U:
      t.state = h_rpct(80) ? 3u : pick_state(); t.cnt = h_rpct(30) ? 0u : rnd_range(1, 50);
      break;
    case O_RECCPTAG: case O_RECED: case O_RECFIRSTED: case O_RECP1TAG:
      t.w20 = h_rpct(85) ? 0u : 1u; t.w1c0 = h_rpct(50); t.state = tg[h_rmod(6)];
      if (t.cpFrame) t.cnt = h_rpct(50) ? t.cpFrame * (1u + h_rmod(3)) : t.cpFrame * (1u + h_rmod(3)) + 1u;
      break;
    default: return;
  }
  store_impl(&t);
  g_m.s = t;
}
static void trial_ops(const int *tab, unsigned ntab) {
  unsigned ops = rnd_range(1, 9), k;
  new_trial();
  for (k = 0; k < ops; k++) {
    int op = tab[h_rmod(ntab)];
    Args a;
    if (k > 0 && h_rpct(65)) reaim();
    aim(op);
    gen_args(op, &a);
    do_op(op, &a);
  }
  end_trial();
}

/* the constructor (C1 / C2): the object holds random bytes, the arguments are the harness' fake objects or NULL (the CP is never NULL: the blob dereferences it); the V92Mapper it allocates is freed afterwards */
static void trial_ctor(void) {
  Args a;
  new_trial();
  memset(&a, 0, sizeof a);
  { S t = g_m.s; t.pMp = t.pCp = t.pB2s = t.pParams = t.pMapper = 9; store_impl(&t); g_m.s = t; }   /* the arguments differ from what was there */
  a.p[0] = h_rpct(80) ? (uintptr_t)g_params : (h_rpct(50) ? 0u : (uintptr_t)g_hist);
  a.p[1] = (uintptr_t)g_cp;
  a.p[2] = h_rpct(80) ? (uintptr_t)g_mp : 0u;
  a.p[3] = h_rpct(75) ? (uintptr_t)g_b2s : 0u;
  a.c2 = h_rpct(35);
  do_op(O_CTOR, &a);
  sysdep_free(g_mblk);
  end_trial();
}
/* the destructor (D1 / D2): with a heap V92Mapper (freed) and with none; the object must be left exactly as it was (the pointer is not cleared) */
static void trial_dtor(void) {
  Args a;
  S t;
  unsigned mode = h_rmod(4);
  new_trial();
  t = g_m.s;
  memset(&a, 0, sizeof a);
  if (mode == 0 || mode == 1) { g_mblk = sysdep_malloc(0x2c); t.pMapper = 7; }
  else t.pMapper = 0;
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
  const char *seed = getenv("P33_SEED");
  CovSite *c;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P33_DBG")) g_show = 1;
  p11_layout_check(); p15_layout_check(); p29_layout_check(); p32_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  switch (sc) {
    case 0: for (tr = 0; tr < 7000; tr++) trial_ops(k_ev, sizeof k_ev / sizeof k_ev[0]); break;
    case 1: for (tr = 0; tr < 6000; tr++) trial_ops(k_gen, sizeof k_gen / sizeof k_gen[0]); break;
    case 2:
      for (tr = 0; tr < 2000; tr++) trial_ops(k_life, sizeof k_life / sizeof k_life[0]);
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
  return (bad && !getenv("P33_NOCOV")) ? 2 : 0;
}
