/*
 * Differential test harness for port chunk P30 (src/vpcm/v90_phase4_modulator_symbol.cpp): V90Phase4Modulator::generateV90Symbol and ::generateV92Symbol, the two symbol generators of the V.90 / V.92
 * phase-4 (TX) state machine (6,157 B, two jump tables, no x87).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_phase4_modulator_symbol.sh against the blob (dsplibs.o with the 13 callees and edprintf WEAK so that the recorders below win), against
 * the port (-O0 / -O2: the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_phase4_modulator_symbol.blob32.out).  The functions are reached
 * through __asm__ labels; the object is the generated C mirror (test/vpcm/v90_phase4_modulator_symbol.layout), the callees' objects are plain heap blocks addressed through the mirrors of P09 (V90MP), P10 (V90CP), P11
 * (V90MappingParams) and P29 (V90BitsToSymbol).
 *
 * Every callee is a RECORDER with scripted results (the Scrambler<uchar,uchar> writes scripted bits into bits_; V90BitsToSymbol::nofBitsForNextTime / process deliver scripted bit counts and symbols;
 * V90MP / V90CP::getBitVector deliver scripted lengths and one of two harness bit blocks); each record carries the callee, its arguments (pointers as identities: bits_, mpblk0, cpblk1, &mpLen_, the
 * data / TRN2d mapping parameters, ...) and a SNAPSHOT of the object and of the CP's four touched words (store order before each call / print).  edprintf / dsplibs_debug_printf are recorders too.
 * What is compared: every returned symbol, the whole object after every call (field-wise dump; every field, the 12000 scrambled bytes included), the blocks of the callees' objects (bytes untouched except
 * the CP's four words), the canary behind the object, every record.  The INDEPENDENT MODEL (mdl_*: a table of state descriptors interpreted by a small engine, written from porting/decompiled/dp/p30_*.txt, not
 * from the port source) predicts the symbol, the new object state, every record; a model violation exits 3.  Calls that would crash the REAL blob (divide by zero in a frame length / D, a NULL CP /
 * mapping-parameter dereference) are excluded by the driver.
 *
 * Scenarios (each its own process):
 *   0 generateV90Symbol: random states / counts around every threshold, debug levels, NULL pointers
 *   1 generateV92Symbol: the same
 *   2 natural runs: V.90 and V.92 from Ri to Phase4-terminated with the owner's events poked in (every state in order, 16,000+ symbols per TRN2d)
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

extern short f_sym90(void *) L("_ZN18V90Phase4Modulator17generateV90SymbolEv");
extern short f_sym92(void *) L("_ZN18V90Phase4Modulator17generateV92SymbolEv");
extern unsigned int dsplibs_debug_level;

/* the interposed callees */
extern void c_scr_proc(void *, const unsigned char *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE7processEPKhPhj");
extern void c_scr_ones(void *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE14processAllOnesEPhj");
extern void c_scr_zeros(void *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE15processAllZerosEPhj");
extern void c_scr_reset(void *, unsigned char) L("_ZN9ScramblerIhhE5resetEh");
extern unsigned c_b2s_nof(void *) L("_ZN15V90BitsToSymbol18nofBitsForNextTimeEv");
extern unsigned c_b2s_pbits(void *, unsigned char *, unsigned) L("_ZN15V90BitsToSymbol7processEPhj");
extern unsigned c_b2s_pout(void *, unsigned *, short *) L("_ZN15V90BitsToSymbol7processERjPs");
extern void c_b2s_rns(void *, void *, int) L("_ZN15V90BitsToSymbol15resetNoSpectralEP16V90MappingParams7PcmType");
extern unsigned char *c_mp_gbv(void *, unsigned *) L("_ZN5V90MP12getBitVectorERj");
extern unsigned char *c_cp_gbv(void *, unsigned *) L("_ZN5V90CP12getBitVectorERj");
extern void c_cp_i2b(void *) L("_ZN5V90CP10infoToBitsEv");
extern void c_disp(void *) L("displaySpectralParams");

typedef Ph4Obj O;
#define SZ sizeof(O)
enum { CAN = 64, BITSN = 12000, BLK = 3016 };

/* ------------------------------------------------------------------------------------------------ the neutral state (impl and model) */
typedef struct {
  uint32_t flag, state, cnt, event, next, cpCnt, w20, w24, w28, w2c, w30, w34, pcm, limit, mpLen, mpFrame, target, cpLen, cpFrame, arg8, w2f9c, w2fa0, notOwned;
  uint32_t mpNotSeen, cpFlag, amp;
  uint16_t rdrt[6], rf[12];
  uint8_t pad15[3], pad1d[3], pad3e[2];
  uint32_t mpBitsId, cpBitsId;                      /* 0 NULL, 1 / 2 mpblk0 / 1, 3 / 4 cpblk0 / 1, 9 other */
  uint32_t hasB2s, hasMp, hasData, hasTrn, hasCp;   /* pointer members: non-NULL */
  uint32_t cpType, cpF13, cpWa0, cpD, mpD, b2sExtra, dataW0, trnW0;
  uint8_t bits[BITSN];
} S;

static O *g_o;
static uint8_t *g_can;
static void *g_b2s, *g_mp, *g_data, *g_trn, *g_cp, *g_params, *g_hist, *g_blk[4];   /* blk 0 / 1 = MP bit vectors, 2 / 3 = CP bit vectors */
static unsigned g_cpsz;

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
#define CPW(f) (((CpObj *)g_cp)->f)
static void load_impl(S *s) {
  const O *o = g_o;
  memset(s, 0, sizeof *s);
  s->flag = o->flag_; s->state = o->state_; s->cnt = o->cnt_; s->event = o->event_; s->next = o->nextState_; s->cpCnt = o->cpCnt_;
  s->w20 = o->w20_; s->w24 = o->w24_; s->w28 = o->w28_; s->w2c = o->w2c_; s->w30 = o->w30_; s->w34 = o->w34_; s->pcm = o->pcm_; s->limit = o->limit_;
  s->mpLen = o->mpLen_; s->mpFrame = o->mpFrame_; s->target = o->target_; s->cpLen = o->cpLen_; s->cpFrame = o->cpFrame_; s->arg8 = o->arg8_; s->w2f9c = o->w2f9c_; s->w2fa0 = o->w2fa0_; s->notOwned = o->notOwned_;
  s->mpNotSeen = o->mpNotSeen_; s->cpFlag = o->cpFlag_; s->amp = o->level_;
  memcpy(s->rdrt, o->rdrt_, sizeof s->rdrt); memcpy(s->rf, o->rf_, sizeof s->rf);
  memcpy(s->pad15, o->pad15_, 3); memcpy(s->pad1d, o->pad1d_, 3); memcpy(s->pad3e, o->pad3e_, 2);
  s->mpBitsId = blk_id(o->mpBits_); s->cpBitsId = blk_id(o->cpBits_);
  s->hasB2s = o->b2s_ != NULL; s->hasMp = o->mp_ != NULL; s->hasData = o->dataMp_ != NULL; s->hasTrn = o->trn2dMp_ != NULL; s->hasCp = o->cp_ != NULL;
  s->cpType = CPW(type_); s->cpF13 = CPW(f13_); s->cpWa0 = CPW(wCa0_); s->cpD = CPW(trn2dD_);
  s->mpD = ((MpObj *)g_mp)->trn2dD_; s->b2sExtra = ((B2S90 *)g_b2s)->extra_; s->dataW0 = ((MapPar90 *)g_data)->w0_; s->trnW0 = ((MapPar90 *)g_trn)->w0_;
  memcpy(s->bits, o->bits_, BITSN);
}
/* the object, from a neutral state (initial setup); the pointer members are chosen by the has* flags and the bit-vector ids */
static void store_impl(const S *s) {
  O *o = g_o;
  o->flag_ = s->flag; o->state_ = s->state; o->cnt_ = s->cnt; o->event_ = s->event; o->nextState_ = s->next; o->cpCnt_ = s->cpCnt;
  o->w20_ = s->w20; o->w24_ = s->w24; o->w28_ = s->w28; o->w2c_ = s->w2c; o->w30_ = s->w30; o->w34_ = s->w34; o->pcm_ = s->pcm; o->limit_ = s->limit;
  o->mpLen_ = s->mpLen; o->mpFrame_ = s->mpFrame; o->target_ = s->target; o->cpLen_ = s->cpLen; o->cpFrame_ = s->cpFrame; o->arg8_ = s->arg8; o->w2f9c_ = s->w2f9c; o->w2fa0_ = s->w2fa0; o->notOwned_ = s->notOwned;
  o->mpNotSeen_ = (uint8_t)s->mpNotSeen; o->cpFlag_ = (uint8_t)s->cpFlag; o->level_ = (uint16_t)s->amp;
  memcpy(o->rdrt_, s->rdrt, sizeof s->rdrt); memcpy(o->rf_, s->rf, sizeof s->rf);
  memcpy(o->pad15_, s->pad15, 3); memcpy(o->pad1d_, s->pad1d, 3); memcpy(o->pad3e_, s->pad3e, 2);
  o->mpBits_ = s->mpBitsId >= 1 && s->mpBitsId <= 4 ? g_blk[s->mpBitsId - 1] : s->mpBitsId == 9 ? (void *)g_hist : NULL;
  o->cpBits_ = s->cpBitsId >= 1 && s->cpBitsId <= 4 ? g_blk[s->cpBitsId - 1] : s->cpBitsId == 9 ? (void *)g_hist : NULL;
  o->b2s_ = s->hasB2s ? g_b2s : NULL; o->mp_ = s->hasMp ? g_mp : NULL; o->dataMp_ = s->hasData ? g_data : NULL; o->trn2dMp_ = s->hasTrn ? g_trn : NULL; o->cp_ = s->hasCp ? g_cp : NULL;
  CPW(type_) = s->cpType; CPW(f13_) = (uint8_t)s->cpF13; CPW(wCa0_) = s->cpWa0; CPW(trn2dD_) = s->cpD;
  ((MpObj *)g_mp)->trn2dD_ = s->mpD; ((B2S90 *)g_b2s)->extra_ = s->b2sExtra; ((MapPar90 *)g_data)->w0_ = s->dataW0; ((MapPar90 *)g_trn)->w0_ = s->trnW0;
  memcpy(o->bits_, s->bits, BITSN);
}

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
enum { NSC = 16, LOGMAX = 24, TXTMAX = 640 };
typedef struct { unsigned nof[NSC], nxt[NSC]; int16_t sym[NSC]; unsigned mplen[4], cplen[4]; } Script;
typedef struct { unsigned inof, inxt, isym, imp, icp, iscr; } Cur;
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Script g_sc;
static Cur g_ic;                       /* the impl's cursor */
static Log g_il;                       /* the impl's records */
static int g_show;

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
/* one record: kind, the text and the snapshot (everything the callee could see) */
static void snapshot(char *out, size_t cap, const S *s) {
  snprintf(out, cap, "|st=%u cnt=%u ev=%u nx=%u mns=%u cc=%u cf=%u w20=%u w24=%u w28=%u w2c=%u w30=%u w34=%u pcm=%u tg=%u mf=%u cfr=%u ml=%u cl=%u mb=%u cb=%u w9c=%u wa0=%u cp=%u,%u,%u,%u bh=%08x",
           s->state, s->cnt, s->event, s->next, s->mpNotSeen, s->cpCnt, s->cpFlag, s->w20, s->w24, s->w28, s->w2c, s->w30, s->w34, s->pcm, s->target, s->mpFrame, s->cpFrame, s->mpLen, s->cpLen,
           s->mpBitsId, s->cpBitsId, s->w2f9c, s->w2fa0, s->cpType, s->cpF13, s->cpWa0, s->cpD, fnv(s->bits, BITSN));
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

static const char *mpname(const void *p) { return p == g_data ? "data" : p == g_trn ? "trn2d" : p ? "other" : "NULL"; }
#define CHECK_THIS(p, what) do { if ((p) != (what)) h_die3("callee %s got an unexpected object %p", __func__, (p)); } while (0)

/* the recorders (impl side) */
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
void c_b2s_rns(void *t, void *mp, int pcm) {
  CHECK_THIS(t, g_b2s);
  IMPL_REC('c', "b2s.resetNoSpectral mp=%s pcm=%u", mpname(mp), (unsigned)pcm);
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
void c_disp(void *mp) { IMPL_REC('c', "displaySpectralParams mp=%s", mpname(mp)); }
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

/* ------------------------------------------------------------------------------------------------ coverage */
enum {
  C_TRIALS = 1, C_OPS, C_LV0, C_LV1, C_LV2, C_LVHI,
  /* per state of each generator: the state was dispatched (V = generateV90Symbol, W = generateV92Symbol) */
  V_ST0 = 8, V_ST_LAST = V_ST0 + 31, W_ST0 = 40, W_ST_LAST = W_ST0 + 31,
  /* end actions */
  X_RINOT_ENTER = 72, X_V90_RINOT_END, X_V92_RINOT_END, X_V90_TRN_MP, X_V90_TRN_DIRECT, X_V90_TRN_NOMP_PRINT, X_V90_TRN_NOMP_QUIET, X_MPNOT_ENTER, X14_NOTSEEN_STATE, X14_NOTSEEN_OTHER, X14_ZERO, X14_ED,
  X14_WAIT, X14_UNSEEN, X_ED_ENTER_V90, X_ED_END_B1D, X_ED_END_NODATA_PRINT, X_ED_END_NODATA_QUIET, X_TERMINATE, X_RD_ENTER, X_RD_QUIET, X_RDNOT_ENTER, X_TRN_REENTER_V90, X_V92_TRN_CP,
  X_V92_TRN_NOCP_PRINT, X_V92_TRN_NOCP_QUIET, X5_NOMOD, X5_ZERO, X5_RESTART, X5_REPEAT_PRINT, X5_REPEAT_QUIET, X5_CPFLAG, X6_ENTER, X7_END, X8_RESTART, X8_NOMOD, X8_ZERO, X_ED_ENTER_V92_MOD,
  X_ED_ENTER_V92_EQ, X_FINAL_SUVD, X_V92_ED_SILENCE_A, X_V92_ED_SILENCE_B, X_V92_ED_B1D, X_V92_ED_NODATA_PRINT, X_V92_ED_NODATA_QUIET, X_V92_ED_COND_W24, X_V92_ED_COND_W2C, X_V92_ED_COND_W30,
  X_TRN_REENTER_V92, X_SIL24_WAIT, X_SIL24_GO, X_SIL25_GO, X_SIL25_WAIT, X_RT_ENTER_RT, X_RTNOT_ENTER, X_SUVD_RRN, X_RF_ENTER, X_RFNOT_ENTER, X_SUVD_RF,
  X_ILL_PRINT, X_ILL_QUIET, X_ZERO, X_PAT_RI_POS, X_PAT_RI_NEG, X_DATA_NOF0, X_DATA_NOFN, X_DRAIN_N0, X_DRAIN_NN, X_EVENT4, X_EVENT7, X_CNT_WRAP,
  COV_MAX
};
static unsigned long g_covn[COV_MAX];
static unsigned char g_covreq[COV_MAX];
static unsigned char g_opcov[COV_MAX];
#define COVM(id) (g_opcov[id] = 1)
static void cov_flush(void) { int i; for (i = 0; i < COV_MAX; i++) if (g_opcov[i]) { g_covn[i]++; g_opcov[i] = 0; } }

/* ------------------------------------------------------------------------------------------------ the independent model */
typedef struct { S s; Cur c; Log l; int crash; unsigned dbg; } Mdl;
#define CRASH(m) do { (m)->crash = 1; return; } while (0)
static void m_rec(Mdl *m, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  rec_add(&m->l, &m->s, kind, "%s", buf);
}
static unsigned m_nof(Mdl *m) { unsigned v = g_sc.nof[m->c.inof++ % NSC]; m_rec(m, 'c', "b2s.nof -> %u", v); return v; }
static void m_pbits(Mdl *m, unsigned n) { m_rec(m, 'c', "b2s.process(bits,%u) h=%08x", n, n <= BITSN ? fnv(m->s.bits, n) : 0u); }
static void m_pout(Mdl *m, unsigned *n, short *out) {
  unsigned v = g_sc.nxt[m->c.inxt++ % NSC];
  short y = g_sc.sym[m->c.isym++ % NSC];
  m_rec(m, 'c', "b2s.process(&n,&out) -> n=%u out=%d", v, (int)y);
  *n = v; *out = y;
}
static void m_fill(Mdl *m, unsigned n) { unsigned i; for (i = 0; i < n; i++) m->s.bits[i] = (uint8_t)bitgen(m->c.iscr, i); m->c.iscr++; }
static void m_scr_ones(Mdl *m, unsigned n) { m_rec(m, 'c', "scr.ones n=%u out=bits", n); m_fill(m, n); }
static void m_scr_zeros(Mdl *m, unsigned n) { m_rec(m, 'c', "scr.zeros n=%u out=bits", n); m_fill(m, n); }
static void m_scr_reset(Mdl *m) { m_rec(m, 'c', "scr.reset v=0"); }
static const char *blkname(unsigned id) { return id == 1 ? "0" : id == 2 ? "1" : id == 3 ? "2" : id == 4 ? "3" : ""; }
static void m_scr_mp(Mdl *m, unsigned n) {
  unsigned id = m->s.mpBitsId;
  const unsigned char *src = id >= 1 && id <= 4 ? (const unsigned char *)g_blk[id - 1] : NULL;
  m_rec(m, 'c', "scr.process in=%s n=%u hin=%08x", bname(id), n, src ? fnv(src, n) : 0u);
  m_fill(m, n);
}
static void m_scr_cp(Mdl *m, unsigned n) {
  unsigned id = m->s.cpBitsId;
  const unsigned char *src = id >= 1 && id <= 4 ? (const unsigned char *)g_blk[id - 1] : NULL;
  m_rec(m, 'c', "scr.process in=%s n=%u hin=%08x", bname(id), n, src ? fnv(src, n) : 0u);
  m_fill(m, n);
}
static void m_rns(Mdl *m, const char *which) { m_rec(m, 'c', "b2s.resetNoSpectral mp=%s pcm=%u", which, m->s.pcm); }
static void m_ed(Mdl *m, const char *fmt, unsigned a) { char b[TXTMAX]; snprintf(b, sizeof b, fmt, (int)a); m_rec(m, 'e', "%s", b); }
static void m_ed0(Mdl *m, const char *t) { m_rec(m, 'e', "%s", t); }
static void m_dbg(Mdl *m, const char *t) { if (m->dbg > 1) m_rec(m, 'd', "%s", t); }
static void m_fetch_mp(Mdl *m) {
  unsigned k = m->c.imp++, l = g_sc.mplen[k % 4];
  if (!m->s.hasMp) CRASH(m);
  m_rec(m, 'c', "MP.getBitVector len=&mpLen -> blk%u,%u", k % 2, l);
  m->s.mpLen = l; m->s.mpBitsId = 1 + k % 2;
  if (m->s.mpD == 0) CRASH(m);
  m->s.mpFrame = (6 * m->s.mpLen) / m->s.mpD;
}
static void m_start_cp(Mdl *m) {
  unsigned k, l;
  if (!m->s.hasCp) CRASH(m);
  m_rec(m, 'c', "CP.infoToBits");
  k = m->c.icp++; l = g_sc.cplen[k % 4];
  m_rec(m, 'c', "CP.getBitVector len=&cpLen -> blk%u,%u", 2 + k % 2, l);
  m->s.cpLen = l; m->s.cpBitsId = 3 + k % 2;
  if (m->s.cpD == 0) CRASH(m);
  m->s.cpFrame = (6 * m->s.cpLen) / m->s.cpD;
}

/* state descriptors: kind, output source / data step, end condition, action */
enum { K_NONE, K_ZERO, K_PAT, K_DATA };
enum { P_RI, P_RDRT, P_RF, P_ZERO };
enum { D_ONES, D_ZEROS, D_MP, D_CP, D_DRAIN };
enum { E_NONE, E_EQ, E_MOD6, E_MODMP, E_MODCP, E_EQCPF, E_EQTARGET, E_DRAIN, E_SPECIAL };
enum { A_NONE, A_RINOT_ENTER, A_V90_RINOT_END, A_V92_RINOT_END, A_V90_TRN_END, A_MPNOT_ENTER, A_ED_ENTER, A_V90_ED_END, A_V92_ED_END, A_TERMINATE, A_RD_ENTER, A_RDNOT_ENTER, A_TRN_REENTER_V90,
       A_V92_TRN_END, A_CPD_ENTER, A_CPD_END, A_FINAL_SUVD, A_TRN_REENTER_V92, A_RT_ENTER, A_RTNOT_ENTER, A_SUVD_RRN, A_RF_ENTER, A_RFNOT_ENTER, A_SUVD_RF, A_SPECIAL };
typedef struct { uint8_t kind, src, neg, end, act; uint32_t arg; } Desc;
#define DP(src, neg, end, arg, act) { K_PAT, src, neg, end, act, arg }
#define DD(step, end, arg, act) { K_DATA, step, 0, end, act, arg }
#define DZ { K_ZERO, 0, 0, 0, 0, 0 }
#define DN { K_NONE, 0, 0, 0, 0, 0 }
static const Desc g_d90[28] = {
  /* 0 */ DP(P_RI, 0, E_NONE, 0, A_NONE), DP(P_RI, 0, E_MOD6, 0, A_RINOT_ENTER), DP(P_RI, 1, E_EQ, 0x18, A_V90_RINOT_END), DD(D_ONES, E_EQ, 0x3e7c, A_V90_TRN_END), DD(D_MP, E_NONE, 0, A_NONE),
  /* 5 */ DN, DN, DN, DN, DN, DN, DN, DN,
  /* 13 */ DD(D_MP, E_MODMP, 0, A_MPNOT_ENTER), DD(D_MP, E_SPECIAL, 14, A_SPECIAL), DD(D_MP, E_MODMP, 0, A_ED_ENTER), DD(D_ZEROS, E_EQTARGET, 0, A_V90_ED_END), DD(D_ONES, E_EQ, 0x120, A_TERMINATE),
  /* 18 */ DD(D_ONES, E_NONE, 0, A_NONE), DZ, DD(D_DRAIN, E_DRAIN, 0, A_RD_ENTER), DP(P_RDRT, 0, E_EQ, 0x180, A_RDNOT_ENTER), DP(P_RDRT, 1, E_EQ, 0x18, A_TRN_REENTER_V90), DN, DN, DN, DZ, DZ
};
static const Desc g_d92[31] = {
  /* 0 */ DP(P_RI, 0, E_NONE, 0, A_NONE), DP(P_RI, 0, E_MOD6, 0, A_RINOT_ENTER), DP(P_RI, 1, E_EQ, 0x18, A_V92_RINOT_END), DD(D_ONES, E_EQ, 0x3e7c, A_V92_TRN_END), DN,
  /* 5 */ DD(D_CP, E_SPECIAL, 5, A_SPECIAL), DD(D_CP, E_MODCP, 0, A_CPD_ENTER), DD(D_CP, E_EQCPF, 0, A_CPD_END), DD(D_CP, E_SPECIAL, 8, A_SPECIAL), DD(D_CP, E_MODCP, 0, A_ED_ENTER),
  /* 10 */ DD(D_CP, E_MODCP, 0, A_ED_ENTER), DD(D_CP, E_EQCPF, 0, A_ED_ENTER), DD(D_CP, E_MODCP, 0, A_FINAL_SUVD), DN, DN, DN,
  /* 16 */ DD(D_ZEROS, E_EQTARGET, 0, A_V92_ED_END), DD(D_ONES, E_EQ, 0x120, A_TERMINATE), DD(D_ONES, E_NONE, 0, A_NONE), DZ, DD(D_DRAIN, E_DRAIN, 0, A_RD_ENTER),
  /* 21 */ DP(P_RDRT, 0, E_EQ, 0x180, A_RDNOT_ENTER), DP(P_RDRT, 1, E_EQ, 0x18, A_TRN_REENTER_V92), DZ, DP(P_ZERO, 0, E_SPECIAL, 24, A_SPECIAL), DP(P_ZERO, 0, E_MOD6, 0, A_RT_ENTER),
  /* 26 */ DP(P_RDRT, 0, E_EQ, 0x180, A_RTNOT_ENTER), DP(P_RDRT, 1, E_EQ, 0x18, A_SUVD_RRN), DD(D_DRAIN, E_DRAIN, 0, A_RF_ENTER), DP(P_RF, 0, E_EQ, 0x180, A_RFNOT_ENTER), DP(P_RF, 1, E_EQ, 0x18, A_SUVD_RF)
};
static int16_t m_neg(int16_t v) { return (int16_t)(-(int)v); }
static void m_enter_ed(Mdl *m) { m_ed(m, "V90Phase4Modulator: enter Ed @ %d\r\n", m->s.cnt); m->s.cnt = 0; m->s.state = 0x10; m->s.target = m->s.b2sExtra + 12; }
static void m_ed_end_data(Mdl *m, int v92) {
  if (m->s.hasData) {
    m_ed(m, "V90Phase4Modulator: enter B1d @ %d\r\n", m->s.cnt);
    m->s.state = 0x11; m->s.cnt = 0;
    m_rns(m, "data"); m_scr_reset(m);
    COVM(v92 ? X_V92_ED_B1D : X_ED_END_B1D);
  } else {
    m->s.state = 0x13; m->s.cnt = 0;
    COVM(m->dbg > 1 ? (v92 ? X_V92_ED_NODATA_PRINT : X_ED_END_NODATA_PRINT) : (v92 ? X_V92_ED_NODATA_QUIET : X_ED_END_NODATA_QUIET));
    m_dbg(m, "V90Phase4Modulator: ERROR: Null dataPhaseMappingParams @ end of Ed\r\n");
  }
}
static void m_action(Mdl *m, unsigned act) {
  S *s = &m->s;
  switch (act) {
    case A_RINOT_ENTER: COVM(X_RINOT_ENTER); m_ed(m, "V90Phase4Modulator: enter RiNot @ %d\r\n", s->cnt); s->state = 2; s->cnt = 0; break;
    case A_V90_RINOT_END: COVM(X_V90_RINOT_END); s->state = 3; s->cnt = 0; m_ed0(m, "V90Phase4Modulator: enter TRN2d\r\n"); m_scr_reset(m); break;
    case A_V92_RINOT_END: COVM(X_V92_RINOT_END); s->state = 3; s->cnt = 0; m_ed0(m, "V90Phase4Modulator: RiNot Terminated\r\n"); break;
    case A_V90_TRN_END:
      if (!s->hasMp) {
        s->state = 0x13; s->cnt = 0;
        COVM(m->dbg > 1 ? X_V90_TRN_NOMP_PRINT : X_V90_TRN_NOMP_QUIET);
        m_dbg(m, "V90Phase4Modulator: ERROR: Null MP @ end of TRN2d\r\n");
        break;
      }
      s->state = s->next;
      { char b[TXTMAX]; snprintf(b, sizeof b, "V90Phase4Modulator: enter %s @ %d\n", s->state == 4 ? "MP" : "DIRECTLY to MPNot", 0x3e7c); m_rec(m, 'e', "%s", b); }
      COVM(s->state == 4 ? X_V90_TRN_MP : X_V90_TRN_DIRECT);
      s->cnt = 0;
      m_fetch_mp(m);
      break;
    case A_MPNOT_ENTER: COVM(X_MPNOT_ENTER); m_ed(m, "V90Phase4Modulator: enter MPNot @ %d\r\n", s->cnt); s->state = 0xe; s->cnt = 0; m_fetch_mp(m); break;
    case A_ED_ENTER: COVM(m->s.state == 15 ? X_ED_ENTER_V90 : s->state == 11 ? X_ED_ENTER_V92_EQ : X_ED_ENTER_V92_MOD); m_enter_ed(m); break;
    case A_V90_ED_END: m_ed_end_data(m, 0); break;
    case A_V92_ED_END:
      if (s->w24 == 0) COVM(X_V92_ED_COND_W24);
      if (s->w24 != 0 && s->w2c != 0 && s->w30 == 0) {
        m_ed(m, "V90Phase4Modulator: enter Silence @ %d\r\n", s->cnt);
        s->cnt = 0; s->state = s->w34 == 0 ? 0x18 : 0x17;
        COVM(s->w34 == 0 ? X_V92_ED_SILENCE_A : X_V92_ED_SILENCE_B);
      } else {
        if (s->w24 != 0 && s->w2c != 0) COVM(X_V92_ED_COND_W30);
        if (s->w24 != 0 && s->w2c == 0) COVM(X_V92_ED_COND_W2C);
        m_ed_end_data(m, 1);
      }
      break;
    case A_TERMINATE: COVM(X_TERMINATE); m_ed(m, "V90Phase4Modulator: Phase4 Terminated @ %d\r\n", 0x120); s->state = 0x12; s->cnt = 0; s->event = 7; COVM(X_EVENT7); break;
    case A_RD_ENTER: m_ed(m, "V90Phase4Modulator: enter Rd @ %d\r\n", s->cnt); s->state = 0x15; s->cnt = 0; COVM(X_RD_ENTER); break;
    case A_RDNOT_ENTER: COVM(X_RDNOT_ENTER); m_ed(m, "V90Phase4Modulator: enter RdNot @ %d\r\n", 0x180); s->state = 0x16; s->cnt = 0; break;
    case A_TRN_REENTER_V90:
      if (!s->hasTrn) CRASH(m);
      COVM(X_TRN_REENTER_V90);
      m_ed(m, "V90Phase4Modulator: enter TRN2d @ %d\r\n", 0x18); s->state = 3; s->cnt = 0;
      m_rns(m, "trn2d"); m_ed0(m, "V90Phase4Modulator: TRN2d spectral parameters:\r\n"); m_rec(m, 'c', "displaySpectralParams mp=trn2d"); m_ed(m, "V90Phase4Modulator: TRN2d D = %d\r\n", s->trnW0);
      break;
    case A_TRN_REENTER_V92:
      if (!s->hasTrn || !s->hasCp) CRASH(m);
      COVM(X_TRN_REENTER_V92);
      m_ed(m, "V90Phase4Modulator: enter TRN2d @ %d\r\n", 0x18); s->cnt = 0; s->state = 3; s->cpD = s->trnW0;
      m_rns(m, "trn2d"); m_ed0(m, "V90Phase4Modulator: TRN2d spectral parameters:\r\n"); m_rec(m, 'c', "displaySpectralParams mp=trn2d"); m_ed(m, "V90Phase4Modulator: TRN2d D = %d\r\n", s->trnW0);
      break;
    case A_V92_TRN_END:
      if (s->hasCp) {
        COVM(X_V92_TRN_CP);
        s->state = 5; m_ed(m, "V90Phase4Modulator: enter SUVd @ %d\r\n", 0x3e7c); s->cnt = 0; s->cpType = 1; s->cpWa0 = s->w28; m_start_cp(m);
      } else {
        s->state = 0x13; s->cnt = 0;
        COVM(m->dbg > 1 ? X_V92_TRN_NOCP_PRINT : X_V92_TRN_NOCP_QUIET);
        m_dbg(m, "V90Phase4Modulator: ERROR: Null CP @ end of TRN2d\r\n");
      }
      s->event = 4; COVM(X_EVENT4);
      break;
    case A_CPD_ENTER: COVM(X6_ENTER); m_ed(m, "V90Phase4Modulator: enter CPd @ %d\r\n", s->cnt); s->cnt = 0; s->state = 7; if (!s->hasCp) CRASH(m); s->cpType = 0; m_start_cp(m); s->w2fa0 = 1; break;
    case A_CPD_END:
      COVM(X7_END); m_ed(m, "V90Phase4Modulator: CPd Terminated @ %d\r\n", s->cnt); s->cpCnt = 0; s->cpFlag = 1; s->state = 5; s->cnt = 0;
      if (!s->hasCp) CRASH(m);
      s->cpType = 1; m_start_cp(m);
      break;
    case A_FINAL_SUVD: COVM(X_FINAL_SUVD); m_ed(m, "V90Phase4Modulator: enter FinalSUVd @ %d\r\n", s->cnt); s->state = 0xb; s->cnt = 0; if (!s->hasCp) CRASH(m); s->cpType = 1; m_start_cp(m); break;
    case A_RT_ENTER: m_ed(m, "V90Phase4Modulator: enter Rt @ %d\r\n", s->cnt); s->state = 0x1a; s->cnt = 0; COVM(X_RT_ENTER_RT); break;
    case A_RTNOT_ENTER: COVM(X_RTNOT_ENTER); m_ed(m, "V90Phase4Modulator: enter RtNot @ %d\r\n", 0x180); s->state = 0x1b; s->cnt = 0; break;
    case A_SUVD_RRN:
      if (!s->hasCp) CRASH(m);
      COVM(X_SUVD_RRN);
      m_ed(m, "V90Phase4Modulator: enter SUVd at RRN @ %d\r\n", 0x18);
      s->state = 5; s->cnt = 0; s->w30 = 1; s->w20 = 0; s->cpFlag = 0; s->cpCnt = 0; s->cpF13 = 0; s->w2f9c = 0; s->w2fa0 = 0; s->cpType = 1; m_start_cp(m);
      break;
    case A_RF_ENTER: COVM(X_RF_ENTER); m_ed(m, "V90Phase4Modulator: enter Rf @ %d\r\n", s->cnt); s->state = 0x1d; s->cnt = 0; break;
    case A_RFNOT_ENTER: COVM(X_RFNOT_ENTER); m_ed(m, "V90Phase4Modulator: enter RfNot @ %d\r\n", 0x180); s->state = 0x1e; s->cnt = 0; break;
    case A_SUVD_RF:
      if (!s->hasCp || !s->hasData) CRASH(m);
      COVM(X_SUVD_RF);
      m_ed(m, "V90Phase4Modulator: enter SUVd @ %d\r\n", 0x18);
      s->cnt = 0; s->state = 5; s->cpType = 1; s->cpD = s->dataW0; s->cpWa0 = s->w28; m_start_cp(m);
      break;
    default: h_die3("model: action %u", act);
  }
}
/* the two end tests that are not a plain compare */
static void m_special(Mdl *m, unsigned which) {
  S *s = &m->s;
  if (which == 14) {
    if (s->mpNotSeen == 0) { COVM(X14_UNSEEN); return; }
    { int was = s->state == 0xe; s->mpNotSeen = 0; if (!was) { COVM(X14_NOTSEEN_OTHER); return; } }
    if (s->cnt == 0) { COVM(X14_ZERO); return; }
    if (s->mpFrame == 0) CRASH(m);
    if (s->cnt % s->mpFrame == 0) { COVM(X14_ED); m_enter_ed(m); } else { COVM(X14_WAIT); s->state = 0xf; }
  } else if (which == 5) {
    if (s->cpFrame == 0) CRASH(m);
    if (s->cnt % s->cpFrame != 0) { COVM(X5_NOMOD); return; }
    if (s->cnt == 0) { COVM(X5_ZERO); return; }
    COVM(X5_RESTART);
    m_start_cp(m);
    if (m->crash) return;
    if (s->cpCnt <= s->limit + 0x320u) return;
    s->cpFlag = 0; s->cpCnt = 0;
    COVM(m->dbg > 1 ? X5_REPEAT_PRINT : X5_REPEAT_QUIET);
    if (m->dbg > 1) { char b[TXTMAX]; snprintf(b, sizeof b, "V90Phase4Modulator: enter repeatedCPd @ %d\r\n", (int)s->cnt); m_rec(m, 'd', "%s", b); }
    s->cpType = 0; s->state = 8; m_start_cp(m);
    s->cnt = 0;
  } else if (which == 8) {
    if (s->cpFrame == 0) CRASH(m);
    if (s->cnt % s->cpFrame != 0) { COVM(X8_NOMOD); return; }
    if (s->cnt == 0) { COVM(X8_ZERO); return; }
    COVM(X8_RESTART);
    m_start_cp(m);
  } else if (which == 24) {
    if (s->cnt <= 0x95f) { COVM(X_SIL24_WAIT); return; }
    if (s->cnt % 6 != 0) { COVM(X_SIL24_WAIT); return; }
    COVM(X_SIL24_GO);
    m_action(m, A_RT_ENTER);
  }
}
static int16_t mdl_sym(Mdl *m, int v92) {
  S *s = &m->s;
  const Desc *d;
  unsigned st = s->state, n = 0, r;
  int16_t out = 0, y = 0;
  s->cnt++;
  s->event = 0;
  if (s->cnt == 0) COVM(X_CNT_WRAP);
  d = st < (v92 ? 31u : 28u) ? (v92 ? &g_d92[st] : &g_d90[st]) : NULL;
  COVM((v92 ? W_ST0 : V_ST0) + (st < 31 ? st : 31));
  if (!d || d->kind == K_NONE) {
    COVM(m->dbg > 1 ? X_ILL_PRINT : X_ILL_QUIET);
    m_dbg(m, "V90Phase4Modulator: Illegal state\r\n");
    return 0;
  }
  if (d->kind == K_ZERO) { COVM(X_ZERO); return 0; }
  if (d->kind == K_PAT) {
    r = (s->cnt - 1) % 6;
    switch (d->src) {
      case P_RI: out = r < 3 ? (int16_t)s->amp : m_neg((int16_t)s->amp); COVM(r < 3 ? X_PAT_RI_POS : X_PAT_RI_NEG); break;
      case P_RDRT: out = (int16_t)s->rdrt[r]; break;
      case P_RF: out = (int16_t)s->rf[(s->cnt - 1) % 12]; break;
      default: out = 0; break;
    }
    if (d->neg) out = m_neg(out);
  } else {
    if (d->src == D_DRAIN) {
      m_pout(m, &n, &y);
      COVM(n ? X_DRAIN_NN : X_DRAIN_N0);
    } else {
      n = m_nof(m);
      COVM(n ? X_DATA_NOFN : X_DATA_NOF0);
      if (n) {
        unsigned len = d->src == D_MP ? s->mpLen : s->cpLen;
        switch (d->src) {
          case D_ONES: m_scr_ones(m, n); m_pbits(m, n); break;
          case D_ZEROS: m_scr_zeros(m, n); m_pbits(m, n); break;
          case D_MP: m_scr_mp(m, len); m_pbits(m, len); break;
          default: m_scr_cp(m, len); m_pbits(m, len); break;
        }
      }
      m_pout(m, &n, &y);
    }
    out = y;
    if (v92 && st == 5 && s->cpFlag != 0) { s->cpCnt++; COVM(X5_CPFLAG); }
  }
  switch (d->end) {
    case E_NONE: break;
    case E_EQ: if (s->cnt == d->arg) m_action(m, d->act); break;
    case E_MOD6: if (s->cnt % 6 == 0) m_action(m, d->act); else if (d->act == A_RT_ENTER) COVM(X_SIL25_WAIT); break;
    case E_MODMP: if (s->mpFrame == 0) { m->crash = 1; return 0; } if (s->cnt % s->mpFrame == 0) m_action(m, d->act); break;
    case E_MODCP: if (s->cpFrame == 0) { m->crash = 1; return 0; } if (s->cnt % s->cpFrame == 0) m_action(m, d->act); break;
    case E_EQCPF: if (s->cnt == s->cpFrame) m_action(m, d->act); break;
    case E_EQTARGET: if (s->cnt == s->target) m_action(m, d->act); break;
    case E_DRAIN: if (n != 0) m_action(m, d->act); else COVM(X_RD_QUIET); break;
    case E_SPECIAL: m_special(m, d->arg); break;
    default: h_die3("model: end kind");
  }
  return out;
}

/* ------------------------------------------------------------------------------------------------ comparison */
static Mdl g_m;                       /* the model, in sync with the object between operations */
static unsigned long g_nops, g_nskipped, g_dumpctr, g_ntrials;
static const char *g_what = "";
#define CMPF(f) do { if (a->f != b->f) h_die3("%s: %s: model %llu (0x%llx), impl %llu (0x%llx)", g_what, #f, (unsigned long long)a->f, (unsigned long long)a->f, (unsigned long long)b->f, (unsigned long long)b->f); } while (0)
static void cmp_S(const S *a, const S *b) {
  unsigned i;
  CMPF(flag); CMPF(state); CMPF(cnt); CMPF(event); CMPF(next); CMPF(cpCnt); CMPF(w20); CMPF(w24); CMPF(w28); CMPF(w2c); CMPF(w30); CMPF(w34); CMPF(pcm); CMPF(limit); CMPF(mpLen); CMPF(mpFrame);
  CMPF(target); CMPF(cpLen); CMPF(cpFrame); CMPF(arg8); CMPF(w2f9c); CMPF(w2fa0); CMPF(notOwned); CMPF(mpNotSeen); CMPF(cpFlag); CMPF(amp); CMPF(mpBitsId); CMPF(cpBitsId);
  CMPF(hasB2s); CMPF(hasMp); CMPF(hasData); CMPF(hasTrn); CMPF(hasCp); CMPF(cpType); CMPF(cpF13); CMPF(cpWa0); CMPF(cpD); CMPF(mpD); CMPF(b2sExtra); CMPF(dataW0); CMPF(trnW0);
  for (i = 0; i < 6; i++) if (a->rdrt[i] != b->rdrt[i]) h_die3("%s: rdrt[%u]", g_what, i);
  for (i = 0; i < 12; i++) if (a->rf[i] != b->rf[i]) h_die3("%s: rf[%u]", g_what, i);
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
/* the objects the callees get: saved before an operation, compared after (nothing but the CP's four words may change) */
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
  ((CpObj *)exp)->type_ = model->cpType; ((CpObj *)exp)->f13_ = (uint8_t)model->cpF13; ((CpObj *)exp)->wCa0_ = model->cpWa0; ((CpObj *)exp)->trn2dD_ = model->cpD;
  if (memcmp(exp, g_cp, sizeof(CpObj))) h_die3("%s: the CP block differs from the model beyond its four words", g_what);
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
/* one operation: model first (a call that would crash the blob is skipped), then the object */
static int do_op(int v92) {
  Mdl m;
  S cur, ns;
  int16_t ms, is;
  g_what = v92 ? "generateV92Symbol" : "generateV90Symbol";
  m = g_m;
  m.l.n = 0;
  m.dbg = dsplibs_debug_level;
  ms = mdl_sym(&m, v92);
  if (m.crash) { g_nskipped++; return 0; }
  load_impl(&cur);
  cmp_S(&g_m.s, &cur);   /* in sync before the call */
  save_blocks();
  g_il.n = 0;
  g_show = h_sample(&g_symctr, 12, 500);
  is = v92 ? f_sym92(g_o) : f_sym90(g_o);
  __asm__ volatile("" ::: "memory");
  load_impl(&ns);
  if (ms != is) h_die3("%s: symbol: model %d, impl %d", g_what, (int)ms, (int)is);
  cmp_S(&m.s, &ns);
  log_cmp(&m);
  check_blocks(&m.s);
  if (memcmp(&m.c, &g_ic, sizeof m.c)) h_die3("%s: the callee cursors differ", g_what);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i); }
  g_m = m;
  h_emit(g_show, "sym %d st=%u cnt=%u ev=%u", (int)is, ns.state, ns.cnt, ns.event);
  dump_obj("op", g_show);
  cov_flush();
  g_nops++;
  g_covn[C_OPS]++;
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
static void fill_random(void *p, size_t n) { uint8_t *q = (uint8_t *)p; size_t i; for (i = 0; i < n; i++) q[i] = (uint8_t)h_rnd(); }
static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) {
    g_sc.nof[i] = h_rpct(35) ? 0u : rnd_range(1, 40);
    g_sc.nxt[i] = h_rpct(45) ? 0u : rnd_range(1, 40);
    g_sc.sym[i] = h_rpct(8) ? (int16_t)(h_rpct(50) ? -32768 : 32767) : (int16_t)h_rnd();
  }
  for (i = 0; i < 4; i++) { g_sc.mplen[i] = h_rpct(20) ? rnd_range(2, 5) : rnd_range(2, 3000); g_sc.cplen[i] = h_rpct(20) ? rnd_range(2, 5) : rnd_range(2, 3000); }
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
/* the object: random in every field (architecture independent: the table drives the fill), then the chosen state */
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
static int is_v92_state_ok(unsigned st, int v92) { return st < (v92 ? 31u : 28u); }
static unsigned pick_state(int v92) {
  if (h_rpct(12)) return rnd_range(0, 40);                       /* illegal / out-of-table */
  return rnd_range(0, v92 ? 30 : 27);
}
/* the count to start from so that the next call ends the state (or is one off); the table is the harness' own steering, not the model */
static unsigned near_cnt(const S *s, int v92) {
  unsigned st = s->state, j = rnd_range(1, 6), c;
  int d = h_rpct(30) ? (int)h_rmod(3) - 1 : 0;
  if (!is_v92_state_ok(st, v92)) return pick_pat32();
  switch (st) {
    case 1: case 25: c = h_rpct(15) ? (h_rpct(50) ? 0xfffffffbu : 0xfffffff9u) - 6u * h_rmod(4) : 6u * j - 1u; break;
    case 2: case 22: case 27: case 30: c = 0x17; break;
    case 3: c = 0x3e7b; break;
    case 13: c = j * s->mpFrame - 1u; break;
    case 14: c = h_rpct(10) ? 0xffffffffu : j * s->mpFrame - 1u; break;
    case 15: c = j * s->mpFrame - 1u; break;
    case 16: c = s->target - 1u; break;
    case 17: c = 0x11f; break;
    case 21: case 26: case 29: c = 0x17f; break;
    case 24: c = h_rpct(15) ? 0x7fffffffu + h_rmod(20) : 0x95e + h_rmod(8); break;
    case 5: case 6: case 8: case 9: case 10: case 12: c = (h_rpct(10) ? 0u : j) * s->cpFrame - 1u; break;
    case 7: case 11: c = s->cpFrame - 1u; break;
    default: c = h_rpct(50) ? h_rmod(40) : pick_pat32(); break;
  }
  return c + (unsigned)d;
}
static void init_state(S *s, int v92) {
  unsigned lim;
  memset(s, 0, sizeof *s);
  s->flag = h_rpct(50) ? (v92 ? 1u : 0u) : pick_pat32();
  s->state = pick_state(v92);
  s->event = pick_pat32();
  s->next = h_rpct(60) ? 4u : pick_pat32() & 0x1fu;
  s->mpNotSeen = h_rpct(50) ? 1u : (h_rpct(40) ? 0u : (h_rpct(50) ? 0x80u : 0xffu));
  s->cpFlag = h_rpct(50) ? 1u : (h_rpct(40) ? 0u : (h_rpct(50) ? 2u : 0xffu));
  lim = h_rpct(30) ? (0xfffffce0u + h_rmod(4) - 1u) : h_rpct(50) ? rnd_range(0, 1000) : pick_pat32();
  s->limit = lim;
  s->cpCnt = h_rpct(50) ? lim + 0x320u + (unsigned)(int)h_rmod(3) - 1u : pick_pat32();
  s->w20 = pick_pat32(); s->w28 = pick_pat32();
  s->w24 = h_rpct(65) ? rnd_range(1, 5) : 0u; s->w2c = h_rpct(65) ? rnd_range(1, 5) : 0u; s->w30 = h_rpct(40) ? rnd_range(1, 5) : 0u; s->w34 = h_rpct(40) ? 0u : pick_pat32();
  s->pcm = h_rpct(50) ? h_rmod(2) : pick_pat32();
  { static const unsigned a[] = { 0, 1, 0x7fff, 0x8000, 0xffff, 0x1234, 0xfffe }; s->amp = h_rpct(50) ? a[h_rmod(7)] : h_rnd() & 0xffffu; }
  { unsigned i; for (i = 0; i < 6; i++) s->rdrt[i] = (uint16_t)(h_rpct(10) ? 0x8000 : h_rnd()); for (i = 0; i < 12; i++) s->rf[i] = (uint16_t)(h_rpct(10) ? 0x8000 : h_rnd()); }
  fill_random(s->pad15, 3); fill_random(s->pad1d, 3); fill_random(s->pad3e, 2);
  s->mpLen = rnd_range(2, 3000); s->cpLen = rnd_range(2, 3000);
  s->mpD = h_rpct(1) ? 0u : rnd_range(1, 12); s->cpD = h_rpct(1) ? 0u : rnd_range(1, 12);
  s->mpFrame = h_rpct(1) ? 0u : h_rpct(80) ? rnd_range(1, 9) : rnd_range(1, 100);
  s->cpFrame = h_rpct(1) ? 0u : h_rpct(80) ? rnd_range(1, 9) : rnd_range(1, 100);
  s->arg8 = pick_pat32(); s->w2f9c = pick_pat32(); s->w2fa0 = pick_pat32(); s->notOwned = pick_pat32() & 1u;
  { static const unsigned mb[] = { 0, 1, 2, 9, 1, 2 }, cb[] = { 0, 3, 4, 9, 3, 4 }; s->mpBitsId = mb[h_rmod(6)]; s->cpBitsId = cb[h_rmod(6)]; }
  s->hasB2s = 1; s->hasMp = h_rpct(85); s->hasData = h_rpct(80); s->hasTrn = h_rpct(88); s->hasCp = h_rpct(80);
  s->cpType = pick_pat32(); s->cpF13 = h_rnd() & 0xffu; s->cpWa0 = pick_pat32();
  s->b2sExtra = h_rpct(20) ? 0xfffffff4u + h_rmod(12) : h_rpct(50) ? rnd_range(0, 200) : pick_pat32();
  s->dataW0 = pick_pat32(); s->trnW0 = pick_pat32();
  fill_random(s->bits, BITSN);
  s->cnt = near_cnt(s, v92);
  s->target = h_rpct(60) ? s->cnt + 1u : h_rpct(50) ? s->cnt + 2u : pick_pat32();
}
static void new_trial(int v92) {
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  S s;
  g_o = (O *)raw; g_can = raw + SZ;
  prefill_obj();
  alloc_fakes();
  gen_script();
  memset(&g_ic, 0, sizeof g_ic); g_il.n = 0;
  init_state(&s, v92);
  dsplibs_debug_level = pick_level();
  g_o->params_ = g_params; g_o->scr_buf_ = g_hist; g_o->scr_head0_ = (char *)g_hist + 4; g_o->scr_t1_0_ = (char *)g_hist + 8; g_o->scr_end_ = (char *)g_hist + 16;
  g_o->scr_wr_ = (char *)g_hist + 4; g_o->scr_t1_ = (char *)g_hist + 8; g_o->scr_t2_ = (char *)g_hist + 16; g_o->scr_tail_ = 17;
  store_impl(&s);
  memset(&g_m, 0, sizeof g_m);
  load_impl(&g_m.s);
  g_m.dbg = dsplibs_debug_level;
  if (g_dumpctr == 0 || h_rpct(20)) { g_what = "trial setup"; }
  dump_obj("init", 0);
  g_ntrials++;
  g_covn[C_TRIALS]++;
  g_covn[dsplibs_debug_level == 0 ? C_LV0 : dsplibs_debug_level == 1 ? C_LV1 : dsplibs_debug_level == 2 ? C_LV2 : C_LVHI]++;
}
static void end_trial(void) {
  free_fakes();
  h_trial_end("p30");
}
static void poke_cnt(unsigned c) { g_m.s.cnt = c; g_o->cnt_ = c; }
static void poke_state(unsigned st) { g_m.s.state = st; g_o->state_ = st; }

/* one random trial: a few operations, the count re-aimed at a threshold between them */
static void trial_random(int v92) {
  unsigned ops = rnd_range(1, 9), k;
  new_trial(v92);
  for (k = 0; k < ops; k++) {
    if (k > 0 && h_rpct(55)) {
      unsigned c = near_cnt(&g_m.s, v92);
      poke_cnt(c);
      if (h_rpct(25)) { unsigned t = c + 1u; g_m.s.target = t; g_o->target_ = t; }
      if (h_rpct(10)) { unsigned st = pick_state(v92); poke_state(st); }
    }
    do_op(v92);
  }
  end_trial();
}

/* ------------------------------------------------------------------------------------------------ natural runs (the owner's events poked in) */
static void chain_setup(int v92) {
  S *s = &g_m.s;
  s->hasB2s = 1; s->hasMp = 1; s->hasData = 1; s->hasTrn = 1; s->hasCp = 1;
  s->mpD = rnd_range(1, 6); s->cpD = rnd_range(1, 6);
  s->mpFrame = rnd_range(1, 9); s->cpFrame = rnd_range(1, 9);
  s->mpLen = rnd_range(2, 200); s->cpLen = rnd_range(2, 200);
  s->next = 4; s->mpNotSeen = 0; s->cpFlag = 0; s->cpCnt = 0; s->limit = 0; s->cnt = rnd_range(0, 5); s->state = 1; s->event = 0;
  s->w24 = 1; s->w2c = 1; s->w30 = 0; s->w34 = 0; s->b2sExtra = rnd_range(0, 60);
  s->mpBitsId = 1; s->cpBitsId = 3;
  (void)v92;
  store_impl(s);
  load_impl(&g_m.s);
}
static void poke_mpnot(void) { g_m.s.mpNotSeen = 1; g_o->mpNotSeen_ = 1; }
static void poke_cpflag(unsigned v) { g_m.s.cpFlag = v; g_o->cpFlag_ = (uint8_t)v; }
static void chain_v90(int via_rd) {
  unsigned i, tail = 0;
  new_trial(0);
  dsplibs_debug_level = 2; g_m.dbg = 2;
  chain_setup(0);
  if (via_rd) poke_state(20);
  for (i = 0; i < 60000 && tail < 40; i++) {
    unsigned st = g_m.s.state;
    if (st == 4 && g_m.s.cnt >= 40) poke_state(13);
    if (st == 14 && g_m.s.cnt == 3) poke_mpnot();
    if (st == 0x12 || st == 0x13) tail++;
    do_op(0);
  }
  end_trial();
}
static void chain_v92(void) {
  unsigned i, tail = 0, phase = 0;
  new_trial(1);
  dsplibs_debug_level = 2; g_m.dbg = 2;
  chain_setup(1);
  for (i = 0; i < 80000 && tail < 40; i++) {
    unsigned st = g_m.s.state;
    if (st == 5 && phase == 0 && g_m.s.cnt >= 3 * g_m.s.cpFrame) { phase = 1; poke_state(6); }
    else if (st == 5 && phase == 1 && g_m.s.cnt >= 3 * g_m.s.cpFrame) { phase = 2; }   /* stays in 5 until the repeated-CPd count runs out */
    else if (st == 8 && g_m.s.cnt >= 3 * g_m.s.cpFrame) { phase = 3; poke_state(12); }
    else if (st == 5 && phase == 3 && g_m.s.cnt >= 2 * g_m.s.cpFrame) { phase = 4; poke_state(9); g_m.s.w30 = 1; g_o->w30_ = 1; }
    if (st == 5 && phase == 0) poke_cpflag(0);
    if (st == 7 || (st == 5 && phase == 2)) poke_cpflag(g_m.s.cpFlag ? g_m.s.cpFlag : 1);
    if (st == 0x12 || st == 0x13) tail++;
    do_op(1);
  }
  end_trial();
}

/* ------------------------------------------------------------------------------------------------ main */
static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
static void req_range(int lo, int hi) { int i; for (i = lo; i <= hi; i++) g_covreq[i] = 1; }
static void req_common(void) {
  g_covreq[C_TRIALS] = g_covreq[C_OPS] = g_covreq[C_LV0] = g_covreq[C_LV1] = g_covreq[C_LV2] = g_covreq[C_LVHI] = 1;
  g_covreq[X_ILL_PRINT] = g_covreq[X_ILL_QUIET] = g_covreq[X_ZERO] = g_covreq[X_PAT_RI_POS] = g_covreq[X_PAT_RI_NEG] = g_covreq[X_DATA_NOF0] = g_covreq[X_DATA_NOFN] = g_covreq[X_DRAIN_N0] = g_covreq[X_DRAIN_NN] = 1;
  g_covreq[X_RINOT_ENTER] = g_covreq[X_ED_ENTER_V90 - 0] = 0;
  g_covreq[X_TERMINATE] = g_covreq[X_RD_ENTER] = g_covreq[X_RD_QUIET] = g_covreq[X_RDNOT_ENTER] = g_covreq[X_RINOT_ENTER] = g_covreq[X_EVENT7] = 1;
}
static const int *const k_none = 0;
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, i, tr, bad = 0;
  const char *seed = getenv("P30_SEED");
  (void)k_none;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P30_DBG")) g_show = 1;
  p09_layout_check(); p10_layout_check(); p11_layout_check(); p29_layout_check(); p30_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  g_covreq[H_COV_STRADDLE] = 0; g_covreq[H_COV_DTOR_NULL] = 0; h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  req_common();
  switch (sc) {
    case 0:
      req_range(V_ST0 + 0, V_ST0 + 4); req_range(V_ST0 + 13, V_ST0 + 22);
      g_covreq[X_V90_RINOT_END] = g_covreq[X_V90_TRN_MP] = g_covreq[X_V90_TRN_DIRECT] = g_covreq[X_V90_TRN_NOMP_PRINT] = g_covreq[X_V90_TRN_NOMP_QUIET] = g_covreq[X_MPNOT_ENTER] = 1;
      g_covreq[X14_NOTSEEN_STATE] = 0; g_covreq[X14_NOTSEEN_OTHER] = 0; g_covreq[X14_ZERO] = g_covreq[X14_ED] = g_covreq[X14_WAIT] = g_covreq[X14_UNSEEN] = 1;
      g_covreq[X_ED_ENTER_V90] = g_covreq[X_ED_END_B1D] = g_covreq[X_ED_END_NODATA_PRINT] = g_covreq[X_ED_END_NODATA_QUIET] = g_covreq[X_TRN_REENTER_V90] = 1;
      for (tr = 0; tr < 2500; tr++) trial_random(0);
      break;
    case 1:
      req_range(W_ST0 + 0, W_ST0 + 3); req_range(W_ST0 + 5, W_ST0 + 12); req_range(W_ST0 + 16, W_ST0 + 30);
      g_covreq[X_V92_RINOT_END] = g_covreq[X_V92_TRN_CP] = g_covreq[X_V92_TRN_NOCP_PRINT] = g_covreq[X_V92_TRN_NOCP_QUIET] = g_covreq[X5_NOMOD] = g_covreq[X5_ZERO] = g_covreq[X5_RESTART] = 1;
      g_covreq[X5_REPEAT_PRINT] = g_covreq[X5_REPEAT_QUIET] = g_covreq[X5_CPFLAG] = g_covreq[X6_ENTER] = g_covreq[X7_END] = g_covreq[X8_RESTART] = g_covreq[X8_NOMOD] = g_covreq[X8_ZERO] = 1;
      g_covreq[X_ED_ENTER_V92_MOD] = g_covreq[X_ED_ENTER_V92_EQ] = g_covreq[X_FINAL_SUVD] = g_covreq[X_V92_ED_SILENCE_A] = g_covreq[X_V92_ED_SILENCE_B] = g_covreq[X_V92_ED_B1D] = 1;
      g_covreq[X_V92_ED_NODATA_PRINT] = g_covreq[X_V92_ED_NODATA_QUIET] = g_covreq[X_V92_ED_COND_W24] = g_covreq[X_V92_ED_COND_W2C] = g_covreq[X_V92_ED_COND_W30] = 1;
      g_covreq[X_TRN_REENTER_V92] = g_covreq[X_SIL24_WAIT] = g_covreq[X_SIL24_GO] = g_covreq[X_SIL25_WAIT] = g_covreq[X_RT_ENTER_RT] = g_covreq[X_RTNOT_ENTER] = g_covreq[X_SUVD_RRN] = 1;
      g_covreq[X_RF_ENTER] = g_covreq[X_RFNOT_ENTER] = g_covreq[X_SUVD_RF] = g_covreq[X_EVENT4] = 1;
      for (tr = 0; tr < 3000; tr++) trial_random(1);
      break;
    case 2:
      { static const int vs[] = { 1, 2, 3, 4, 13, 14, 15, 16, 17, 18, 20, 21, 22 }, ws[] = { 1, 2, 3, 5, 6, 7, 8, 12, 16, 17, 18, 24, 26, 27 }; unsigned q;
        for (q = 0; q < sizeof vs / sizeof vs[0]; q++) g_covreq[V_ST0 + vs[q]] = 1;
        for (q = 0; q < sizeof ws / sizeof ws[0]; q++) g_covreq[W_ST0 + ws[q]] = 1; }
      g_covreq[X_ZERO] = 0; g_covreq[X_RD_QUIET] = 0; g_covreq[X_DRAIN_N0] = 0; g_covreq[X_DATA_NOF0] = 0; g_covreq[X_PAT_RI_NEG] = 0;
      g_covreq[C_LV0] = g_covreq[C_LV1] = g_covreq[C_LV2] = g_covreq[C_LVHI] = g_covreq[X_ILL_PRINT] = g_covreq[X_ILL_QUIET] = g_covreq[X_PAT_RI_POS] = 0;
      g_covreq[X_V90_RINOT_END] = g_covreq[X_V90_TRN_MP] = g_covreq[X_MPNOT_ENTER] = g_covreq[X_ED_ENTER_V90] = g_covreq[X_ED_END_B1D] = g_covreq[X_TRN_REENTER_V90] = 1;
      g_covreq[X_V92_RINOT_END] = g_covreq[X_V92_TRN_CP] = g_covreq[X5_RESTART] = g_covreq[X5_REPEAT_PRINT] = g_covreq[X6_ENTER] = g_covreq[X7_END] = g_covreq[X_FINAL_SUVD] = g_covreq[X_V92_ED_SILENCE_A] = 1;
      g_covreq[X_RT_ENTER_RT] = g_covreq[X_RTNOT_ENTER] = g_covreq[X_SUVD_RRN] = 1;
      g_covreq[X_ILL_QUIET] = g_covreq[X_ILL_PRINT] = 0;
      g_covreq[X_TRN_REENTER_V92] = 0;
      chain_v90(0); chain_v90(1);
      chain_v92();
      break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu ops %lu skipped %lu trials %lu dumps %lu", h_nmalloc, h_nfree, g_nops, g_nskipped, g_ntrials, g_dumpctr);
  for (i = 1; i < COV_MAX; i++) {
    h_emit(1, "COV %03d %lu", i, g_covn[i]);
    if (g_covreq[i] && g_covn[i] == 0) { fprintf(stderr, "COVERAGE FAIL: counter %d was never reached\n", i); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P30_NOCOV")) ? 2 : 0;
}
