/*
 * Differential test harness for port chunk P32 (src/vpcm/v92_phase4_modulator_symbol.cpp): V92Phase4Modulator::generateSymbol, the symbol generator of the V.92 phase-4 (TX) state machine (4,055 B, one jump
 * table of 30 entries, no x87).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v92_phase4_modulator_symbol.sh against the blob (dsplibs.o with the 15 callees and edprintf WEAK so that the recorders below win), against
 * the port (-O0 / -O2: the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v92_phase4_modulator_symbol.blob32.out).  The function is reached through an
 * __asm__ label; the object is the generated C mirror (test/vpcm/v92_phase4_modulator_symbol.layout), the callees' objects are plain heap blocks addressed through the mirrors of P11 (V92CP), P15 (V92ModulusEncoder) and P29
 * (V92BitsToSymbol / V92Transmitter).
 *
 * Every callee is a RECORDER with scripted results (Scrambler<uchar,uchar>::process(uchar) returns scripted bytes, processAllOnes / processAllZeros write scripted bits into bits_; V92BitsToSymbol
 * nofBitsForNextTime / process deliver scripted bit counts and symbols; V92Mapper::process, V92Phase4Modulator::generateSUVu / generateCPu return scripted symbols; V92CP::getBitVector delivers a
 * scripted length and one of two harness bit blocks); each record carries the callee, its arguments (pointers as identities) and a SNAPSHOT of the object and of the CP's words (store order before each
 * call / print).  edprintf / dsplibs_debug_printf are recorders too.  What is compared: every returned symbol, the whole object after every call (field-wise dump), the blocks of the callees' objects (bytes
 * untouched except the CP's five words and the modulus encoder's mode_), the canary behind the object, every record.  The INDEPENDENT MODEL (mdl_*: a table of state descriptors interpreted by a small
 * engine, written from porting/decompiled/dp/p32_*.txt, not from the port source) predicts the symbol, the new object state and every record; a model violation exits 3.  Calls that would crash the REAL blob
 * (divide by zero in a frame length, a NULL CP / bit-vector dereference, mult_ == 0) are excluded by the driver.
 *
 * Scenarios (each its own process):
 *   0 random trials: states around every threshold, debug levels, NULL pointers
 *   1 the same with a second stream of seeds and other steering weights (cp-heavy states)
 *   2 natural runs with the owner's events poked in (Ru -> E1u -> TRN2u -> SUVu -> CPu -> E2u -> B1u -> terminated; FPE; RRN)
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

extern short f_gen(void *) L("_ZN18V92Phase4Modulator14generateSymbolEv");
extern unsigned int dsplibs_debug_level;

/* the interposed callees */
extern unsigned char c_scr_p1(void *, unsigned char) L("_ZN9ScramblerIhhE7processEh");
extern void c_scr_ones(void *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE14processAllOnesEPhj");
extern void c_scr_zeros(void *, unsigned char *, unsigned) L("_ZN9ScramblerIhhE15processAllZerosEPhj");
extern void c_scr_reset(void *, unsigned char) L("_ZN9ScramblerIhhE5resetEh");
extern unsigned c_b2s_nof(void *) L("_ZN15V92BitsToSymbol18nofBitsForNextTimeEv");
extern unsigned c_b2s_pbits(void *, unsigned char *, unsigned) L("_ZN15V92BitsToSymbol7processEPhj");
extern unsigned c_b2s_pout(void *, unsigned *, short *) L("_ZN15V92BitsToSymbol7processERjPs");
extern void c_b2s_reset(void *, void *) L("_ZN15V92BitsToSymbol5resetEP16V92MappingParams");
extern unsigned c_b2s_setblk(void *, unsigned) L("_ZN15V92BitsToSymbol19setSymbolsBlockSizeEj");
extern short c_map_proc(void *, unsigned char *) L("_ZN9V92Mapper7processEPh");
extern void c_cp_suv(void *, unsigned) L("_ZN5V92CP6setSUVEj");
extern void c_cp_i2b(void *) L("_ZN5V92CP10infoToBitsEv");
extern unsigned char *c_cp_gbv(void *, unsigned *) L("_ZN5V92CP12getBitVectorERj");
extern short c_gen_suvu(void *) L("_ZN18V92Phase4Modulator12generateSUVuEv");
extern short c_gen_cpu(void *) L("_ZN18V92Phase4Modulator11generateCPuEv");

typedef V92Ph4Obj O;
#define SZ sizeof(O)
enum { CAN = 64, BITSN = 300, BLK = 3016 };

/* ------------------------------------------------------------------------------------------------ the neutral state (impl and model) */
typedef struct {
  uint32_t state, cnt, w08, event, cpCnt, w20, w24, w28, w2c, w30, w34, w38, w3c, limit, inv, cpLen, cpFrame, w1b8, w1bc, w1c0, w1c4;
  uint32_t cpFlag, level, b42, mult;
  uint8_t pad10[8], pad1d[3], pad1b4[4];
  uint32_t cpBitsId;                 /* 0 NULL, 1 / 2 / 3 = blk0 / 1 / 2, 9 other */
  uint32_t hasMp, hasCp;             /* pointer members: non-NULL */
  uint32_t cpType, cpFlagCp, cpSuv, cpSent, cpMult, meMode;
  uint8_t bits[BITSN];
} S;

static O *g_o;
static uint8_t *g_can;
static void *g_b2s, *g_tx, *g_me, *g_mapper, *g_mp, *g_cp, *g_params, *g_hist, *g_blk[3];
static unsigned long g_symctr;

static uint32_t fnv(const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; uint32_t h = 2166136261u; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static const char *bname(unsigned id) {
  static const char *const nm[] = { "NULL", "blk0", "blk1", "blk2" };
  return id <= 3 ? nm[id] : "other";
}
static unsigned blk_id(const void *p) {
  int k;
  if (!p) return 0;
  for (k = 0; k < 3; k++) if (p == g_blk[k]) return (unsigned)k + 1;
  return 9;
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
  s->hasMp = o->mp_ != NULL; s->hasCp = o->cp_ != NULL;
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
  o->cpBits_ = s->cpBitsId >= 1 && s->cpBitsId <= 3 ? g_blk[s->cpBitsId - 1] : s->cpBitsId == 9 ? (void *)g_hist : NULL;
  o->mp_ = s->hasMp ? g_mp : NULL; o->cp_ = s->hasCp ? g_cp : NULL;
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
  snprintf(out, cap, "|st=%u cnt=%u w08=%u ev=%u cc=%u cf=%u w20=%u w24=%u w28=%u w2c=%u w30=%u w34=%u w38=%u w3c=%u inv=%u cl=%u cfr=%u cb=%u b8=%u bc=%u c0=%u c4=%u cp=%u,%u,%u,%u,%u me=%u bh=%08x",
           s->state, s->cnt, s->w08, s->event, s->cpCnt, s->cpFlag, s->w20, s->w24, s->w28, s->w2c, s->w30, s->w34, s->w38, s->w3c, s->inv, s->cpLen, s->cpFrame, s->cpBitsId,
           s->w1b8, s->w1bc, s->w1c0, s->w1c4, s->cpType, s->cpFlagCp, s->cpSuv, s->cpSent, s->cpMult, s->meMode, fnv(s->bits, BITSN));
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

static int g_flip;                      /* a trial whose "enter B1u" print drops mp_ (the dead re-test of mp_ in state 15 becomes reachable) */
static const char *mpname(const void *p) { return p == g_mp ? "mp" : p ? "other" : "NULL"; }
#define CHECK_THIS(p, what) do { if ((p) != (what)) h_die3("callee %s got an unexpected object %p", __func__, (p)); } while (0)

/* the recorders (impl side) */
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
short c_map_proc(void *t, unsigned char *bits) {
  short y = g_sc.msym[g_ic.imsym++ % NSC];
  CHECK_THIS(t, g_mapper);
  IMPL_REC('c', "map.process%s -> %d", bits == g_o->bits_ ? "" : " in=OTHER", (int)y);
  return y;
}
void c_cp_suv(void *t, unsigned v) {
  CHECK_THIS(t, g_cp);
  IMPL_REC('c', "CP.setSUV %u", v);
}
void c_cp_i2b(void *t) {
  CHECK_THIS(t, g_cp);
  IMPL_REC('c', "CP.infoToBits");
}
unsigned char *c_cp_gbv(void *t, unsigned *len) {
  unsigned k = g_ic.icp++, l = g_sc.cplen[k % 4];
  CHECK_THIS(t, g_cp);
  IMPL_REC('c', "CP.getBitVector len=%s -> blk%u,%u", (void *)len == (void *)&g_o->cpLen_ ? "&cpLen" : "OTHER", k % 2, l);
  *len = l;
  return (unsigned char *)g_blk[k % 2];
}
short c_gen_suvu(void *t) {
  short y = g_sc.gsym[g_ic.igsym++ % NSC];
  CHECK_THIS(t, (void *)g_o);
  IMPL_REC('c', "generateSUVu -> %d", (int)y);
  return y;
}
short c_gen_cpu(void *t) {
  short y = g_sc.gsym[g_ic.igsym++ % NSC];
  CHECK_THIS(t, (void *)g_o);
  IMPL_REC('c', "generateCPu -> %d", (int)y);
  return y;
}
void edprintf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  IMPL_REC('e', "%s", buf);
  if (g_flip && strstr(buf, "enter B1u @")) g_o->mp_ = NULL;
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
  W_ST0 = 8, W_ST_LAST = W_ST0 + 31,                       /* the state was dispatched (31 = beyond the table) */
  X_E1U_ENTER = 40, X_E1U_END, X_T4_END_CP, X_T4_END_NOCP_PRINT, X_T4_END_NOCP_QUIET, X5_NOMOD, X5_ZERO, X5_RESTART, X5_REPEAT_PRINT, X5_REPEAT_QUIET, X5_CPFLAG, X6_END_PRINT, X6_END_QUIET,
  X8_END, X_E2U_ENTER_C, X_E2U_ENTER_D, X12_END, X13_RESTART, X13_ZERO, X15_RRN, X15_NOMP_PRINT, X15_NOMP_QUIET, X15_B1U, X15_B1U_NOMP_PRINT, X15_B1U_NOMP_QUIET, X15_FB1U, X15_MAP, X15_DATA,
  X16_TERM, X17_B1U, X17_NOMP_PRINT, X17_NOMP_QUIET, X18_WAIT_N, X18_WAIT_0, X19_END, X20_END_PRINT, X20_END_QUIET, X23_END, X23_NOCP_PRINT, X23_NOCP_QUIET, X23_BELOW, X24_END, X24_NOCP_PRINT,
  X24_NOCP_QUIET, X25_WAIT_N, X25_WAIT_0, X26_END, X27_END_PRINT, X27_END_QUIET, X27_NOMP, X_ILL_PRINT, X_ILL_QUIET, X_ZERO, X_RU_TOGGLE, X_RU_BITS, X_RU_NEG, X_RU_POS, X_DATA_NOF0, X_DATA_NOFN,
  X_PAT_POS, X_PAT_NEG, X_EVENT9, X_CNT_WRAP, X_W1B8_C, X_W1B8_D, X_MAP_MULT1, X_E2U_RRN_W38_0, X_E2U_RRN_W38_1, X_INV_MULTI, X_DRAIN_N0, X_DRAIN_NN,
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
static void m_scr_bulk(Mdl *m, int ones, unsigned n) { m_rec(m, 'c', ones ? "scr.ones n=%u out=bits" : "scr.zeros n=%u out=bits", n); m_fill(m, n); }
static unsigned m_scr1(Mdl *m, unsigned b) { unsigned v = g_sc.sb[m->c.isb++ % NSC]; m_rec(m, 'c', "scr.process(%u) -> %u", b, v); return v; }
static void m_scr_reset(Mdl *m) { m_rec(m, 'c', "scr.reset v=0"); }
static int16_t m_map(Mdl *m) { int16_t y = g_sc.msym[m->c.imsym++ % NSC]; m_rec(m, 'c', "map.process -> %d", (int)y); return y; }
static int16_t m_gen(Mdl *m, int cpu) { int16_t y = g_sc.gsym[m->c.igsym++ % NSC]; m_rec(m, 'c', cpu ? "generateCPu -> %d" : "generateSUVu -> %d", (int)y); return y; }
static void m_ed(Mdl *m, const char *fmt, unsigned a) { char b[TXTMAX]; snprintf(b, sizeof b, fmt, (int)a); m_rec(m, 'e', "%s", b); if (g_flip && strstr(b, "enter B1u @")) m->s.hasMp = 0; }
static void m_dbg(Mdl *m, const char *t) { if (m->dbg > 1) m_rec(m, 'd', "%s", t); }
static void m_dbg_n(Mdl *m, const char *fmt, unsigned a) { if (m->dbg > 1) { char b[TXTMAX]; snprintf(b, sizeof b, fmt, (int)a); m_rec(m, 'd', "%s", b); } }
/* a CP restart: infoToBits, the bit vector, the frame length; divisor = this->mult (which = 0) or the CP's own byte (1) */
static void m_cp_restart(Mdl *m, int own) {
  S *s = &m->s;
  unsigned k, l, dv;
  if (!s->hasCp) CRASH(m);
  m_rec(m, 'c', "CP.infoToBits");
  k = m->c.icp++; l = g_sc.cplen[k % 4];
  m_rec(m, 'c', "CP.getBitVector len=&cpLen -> blk%u,%u", k % 2, l);
  s->cpLen = l; s->cpBitsId = 1 + k % 2;
  dv = own ? s->cpMult : s->mult;
  if (dv == 0) CRASH(m);
  s->cpFrame = s->cpLen / dv;
}

/* state descriptors: kind, generator, end condition, action */
enum { K_ILL, K_ZERO, K_RU, K_E1U, K_MAP, K_GEN, K_E2U, K_DATA, K_DRAIN, K_PAT };
enum { E_NONE, E_RU1, E_EQW1B8, E_T4, E_T23, E_T24, E_5, E_MODCP, E_EQCPF, E_13, E_EQ, E_DRAIN };
enum { A_NONE, A_E1U_ENTER, A_E1U_END, A_T4, A_T23, A_T24, A_SUV_TO_CPU, A_FINAL_SUVU, A_E2U, A_CPU_TERM, A_E2U_END, A_TERMINATE, A_B1U, A_WAIT_RU, A_RU_END, A_RUNOT_END, A_WAIT_RM, A_RM_END, A_FPE_END };
typedef struct { uint8_t kind, gen, neg, end, act; uint32_t arg; } Desc;
#define DI { K_ILL, 0, 0, 0, 0, 0 }
static const Desc g_d[30] = {
  /* 0 */ { K_RU, 0, 0, E_NONE, A_NONE, 0 }, { K_RU, 0, 0, E_RU1, A_E1U_ENTER, 0 }, { K_E1U, 0, 0, E_EQW1B8, A_E1U_END, 0 }, { K_MAP, 0, 0, E_NONE, A_NONE, 0 }, { K_MAP, 0, 0, E_T4, A_T4, 0 },
  /* 5 */ { K_GEN, 0, 0, E_5, A_NONE, 0 }, { K_GEN, 0, 0, E_MODCP, A_SUV_TO_CPU, 0 }, DI, { K_GEN, 0, 0, E_MODCP, A_FINAL_SUVU, 0 }, { K_GEN, 0, 0, E_MODCP, A_E2U, 0 },
  /* 10 */ { K_GEN, 0, 0, E_EQCPF, A_E2U, 0 }, { K_GEN, 1, 0, E_MODCP, A_E2U, 0 }, { K_GEN, 1, 0, E_EQCPF, A_CPU_TERM, 0 }, { K_GEN, 1, 0, E_13, A_NONE, 0 }, DI,
  /* 15 */ { K_E2U, 0, 0, E_EQW1B8, A_E2U_END, 0 }, { K_DATA, 1, 0, E_EQ, A_TERMINATE, 0x240 }, { K_DATA, 1, 0, E_EQ, A_B1U, 0x240 }, { K_DRAIN, 0, 0, E_DRAIN, A_WAIT_RU, 0 }, { K_PAT, 0, 0, E_EQ, A_RU_END, 0x180 },
  /* 20 */ { K_PAT, 0, 1, E_EQ, A_RUNOT_END, 0x18 }, DI, DI, { K_MAP, 0, 0, E_T23, A_T23, 0 }, { K_MAP, 0, 0, E_T24, A_T24, 0 },
  /* 25 */ { K_DRAIN, 0, 0, E_DRAIN, A_WAIT_RM, 0 }, { K_DATA, 1, 0, E_EQ, A_RM_END, 0x180 }, { K_DATA, 1, 0, E_EQ, A_FPE_END, 0x18 }, { K_DATA, 1, 0, E_NONE, A_NONE, 0 }, { K_ZERO, 0, 0, 0, 0, 0 }
};
static int16_t m_neg(int16_t v) { return (int16_t)(-(int32_t)v); }
/* the differential symbol: inv takes the value, the amplitude is the u16 level, negated when nonzero */
static int16_t m_diff(Mdl *m, uint32_t v) {
  m->s.inv = v;
  if (v > 1) COVM(X_INV_MULTI);
  if (v) { COVM(X_RU_NEG); return (int16_t)(0u - (uint32_t)(uint16_t)m->s.level); }
  COVM(X_RU_POS);
  return (int16_t)(uint16_t)m->s.level;
}
static void m_enter_e2u(Mdl *m) {
  S *s = &m->s;
  m_ed(m, "V92Phase4Modulator: enter E2u @ %d\r\n", s->cnt);
  s->state = 0xf; s->cnt = 0;
  if (s->w1bc == 0) { s->w1b8 = 0xc; COVM(X_W1B8_C); } else { s->w1b8 = 0xd; COVM(X_W1B8_D); }
}
static void m_set_queue_mapping(Mdl *m, int which) {
  S *s = &m->s;
  if (!s->hasMp) { COVM(which == 0 ? (m->dbg > 1 ? X15_B1U_NOMP_PRINT : X15_B1U_NOMP_QUIET) : which == 1 ? (m->dbg > 1 ? X17_NOMP_PRINT : X17_NOMP_QUIET) : X27_NOMP); m_dbg(m, "V92Phase4Modulator: ERROR: Null mappingParams @ setMappingParams\r\n"); }
  else { m_rec(m, 'c', "b2s.reset mp=mp"); m_rec(m, 'c', "b2s.setSymbolsBlockSize 1"); }
}
/* the five stores that every SUV restart of the RRN second part shares are written inline in the two actions: they differ in order and in the CP's byte */
static void m_action(Mdl *m, unsigned act, int16_t *out) {
  S *s = &m->s;
  (void)out;
  switch (act) {
    case A_NONE: break;
    case A_E1U_ENTER: COVM(X_E1U_ENTER); m_ed(m, "V92Phase4Modulator: enter E1u @ %d\r\n", s->cnt); s->state = 2; s->cnt = 0; s->w1b8 = 0xc; break;
    case A_E1U_END: COVM(X_E1U_END); s->state = 3; s->cnt = 0; m_scr_reset(m); break;
    case A_T4:
      if (s->hasCp) {
        COVM(X_T4_END_CP);
        m_ed(m, "V92Phase4Modulator: on TRN2uModulationExit enter SUV @ %d\r\n", s->cnt);
        s->state = 5; s->cnt = 0; s->w08 = 0; s->cpMult = s->mult; s->cpType = 1;
        m_rec(m, 'c', "CP.setSUV %u", s->w2c);
        m_cp_restart(m, 0);
      } else {
        COVM(m->dbg > 1 ? X_T4_END_NOCP_PRINT : X_T4_END_NOCP_QUIET);
        s->state = 0x1d; s->cnt = 0;
        m_dbg(m, "V92Phase4Modulator: ERROR: Null CP @ end of TRN2d\r\n");
      }
      break;
    case A_T23:
      if (!s->hasCp) {
        COVM(m->dbg > 1 ? X23_NOCP_PRINT : X23_NOCP_QUIET);
        s->state = 0x1d; s->cnt = 0;
        m_dbg(m, "V92Phase4Modulator: ERROR: Null CP @ RRN @ end of TRN2u\r\n");
      } else {
        COVM(X23_END);
        m_ed(m, "V92Phase4Modulator: RRN: Terminate TRN2uSecond @ %d\r\n", s->cnt);
        s->w34 = 1; s->w20 = 0; s->cpFlag = 0; s->cpCnt = 0; s->w1c4 = 0; s->w1c0 = 0; s->cpFlagCp = 0; s->cpSent = 0;
        s->state = 5; s->cnt = 0; s->w08 = 0; s->cpType = 1; s->cpSuv = 0;
        m_cp_restart(m, 0);
      }
      break;
    case A_T24:
      if (!s->hasCp) {
        COVM(m->dbg > 1 ? X24_NOCP_PRINT : X24_NOCP_QUIET);
        s->state = 0x1d; s->cnt = 0;
        m_dbg(m, "V92Phase4Modulator: ERROR: Null CP @ RRN @ end of TRN2u\r\n");
      } else {
        COVM(X24_END);
        m_ed(m, "V92Phase4Modulator: on TRN2uRrnSecondExit enter SUV @ %d\r\n", s->cnt);
        s->w34 = 1; s->w1c4 = 0; s->w1c0 = 0; s->w20 = 0; s->cpFlag = 0; s->cpCnt = 0; s->cpFlagCp = 0; s->cpSent = 0;
        s->state = 5; s->cnt = 0; s->w08 = 0; s->cpMult = s->mult; s->cpType = 1; s->cpSuv = 0;
        m_cp_restart(m, 0);
      }
      break;
    case A_SUV_TO_CPU:
      COVM(m->dbg > 1 ? X6_END_PRINT : X6_END_QUIET);
      m_dbg_n(m, "V92Phase4Modulator: on SUVuToCPuModulation enter CPu @ %d\r\n", s->cnt);
      s->state = 0xc; s->cnt = 0; s->cpType = 0;
      if (!s->hasCp) CRASH(m);
      m_rec(m, 'c', "CP.infoToBits");
      { unsigned k = m->c.icp++, l = g_sc.cplen[k % 4];
        m_rec(m, 'c', "CP.getBitVector len=&cpLen -> blk%u,%u", k % 2, l);
        s->cpLen = l; s->cpBitsId = 1 + k % 2; s->w1c4 = 1;
        if (s->cpMult == 0) CRASH(m);
        s->cpFrame = l / s->cpMult; }
      break;
    case A_FINAL_SUVU:
      COVM(X8_END); m_ed(m, "V92Phase4Modulator: enter FinalSUVu @ %d\r\n", s->cnt);
      s->state = 0xa; s->cnt = 0; s->cpType = 1; m_cp_restart(m, 1);
      break;
    case A_E2U: COVM(s->state == 0x9 || s->state == 0xb ? X_E2U_ENTER_C : X_E2U_ENTER_D); m_enter_e2u(m); break;
    case A_CPU_TERM:
      COVM(X12_END); m_ed(m, "V92Phase4Modulator: CPu Terminated @ %d\r\n", s->cnt);
      s->cpCnt = 0; s->cpFlag = 1; s->state = 5; s->cnt = 0; s->cpType = 1; m_cp_restart(m, 1);
      break;
    case A_E2U_END:
      if (s->w28 != 0 && s->w30 != 0 && s->w34 == 0) {
        COVM(s->w38 == 0 ? X_E2U_RRN_W38_0 : X_E2U_RRN_W38_1); COVM(X15_RRN);
        m_ed(m, "V92Phase4Modulator: enter TRN2u Second at RRN @ %d\r\n", s->cnt);
        s->cnt = 0; s->state = 0x17; s->w24 = s->w38 == 0 ? 0x1f44 : 0xfa0;
      } else if (!s->hasMp) {
        COVM(m->dbg > 1 ? X15_NOMP_PRINT : X15_NOMP_QUIET);
        s->state = 0x1d; s->cnt = 0;
        m_dbg(m, "V92Phase4Modulator: ERROR: Null dataPhaseMappingParams @ end of Ed\r\n");
      } else if (s->w3c == 0) {
        COVM(X15_B1U);
        m_ed(m, "V92Phase4Modulator: enter B1u @ %d\r\n", s->cnt);
        s->state = 0x10; m_set_queue_mapping(m, 0); s->cnt = 0; m_scr_reset(m);
      } else {
        COVM(X15_FB1U);
        m_ed(m, "V92Phase4Modulator: enter FB1u @ %d\r\n", s->cnt);
        s->state = 0x11; s->cnt = 0; m_scr_reset(m);
      }
      break;
    case A_TERMINATE: COVM(X16_TERM); m_ed(m, "V92Phase4Modulator: Phase4 Terminated @ %d\r\n", 0x240); s->state = 0x1c; s->cnt = 0; s->event = 9; COVM(X_EVENT9); break;
    case A_B1U: COVM(X17_B1U); m_ed(m, "V92Phase4Modulator: enter B1u @ %d\r\n", 0x240); m_set_queue_mapping(m, 1); s->state = 0x10; s->cnt = 0; break;
    case A_WAIT_RU: COVM(X18_WAIT_N); m_ed(m, "V92Phase4Modulator: enter Ru @ %d\r\n", s->cnt); s->state = 0x13; s->cnt = 0; break;
    case A_RU_END: COVM(X19_END); s->state = 0x14; s->cnt = 0; break;
    case A_RUNOT_END:
      COVM(m->dbg > 1 ? X20_END_PRINT : X20_END_QUIET);
      m_dbg_n(m, "V92Phase4Modulator: RRN: enter TRN2uModulation @ %d\r\n", 0x18);
      s->state = 3; s->cnt = 0; m_scr_reset(m); s->inv = 0;
      break;
    case A_WAIT_RM: COVM(X25_WAIT_N); m_ed(m, "V92Phase4Modulator: enter Rm @ %d\r\n", s->cnt); s->state = 0x1a; s->cnt = 0; s->meMode = 1; break;
    case A_RM_END: COVM(X26_END); s->state = 0x1b; s->cnt = 0; s->meMode = 2; break;
    case A_FPE_END:
      COVM(m->dbg > 1 ? X27_END_PRINT : X27_END_QUIET);
      m_dbg_n(m, "V92Phase4Modulator: FPE: enter SUVu @ %d\r\n", 0x18);
      if (!s->hasCp) CRASH(m);
      s->state = 5; s->cnt = 0; s->cpMult = s->mult; s->cpType = 1;
      m_rec(m, 'c', "CP.setSUV %u", s->w2c);
      m_cp_restart(m, 0);
      if (m->crash) return;
      m_set_queue_mapping(m, 2);
      break;
    default: h_die3("model: action %u", act);
  }
}
static int16_t mdl_sym(Mdl *m) {
  S *s = &m->s;
  const Desc *d;
  unsigned st = s->state, n = 0, r, c;
  int16_t out = 0, y = 0;
  s->cnt++;
  s->event = 0;
  if (s->cnt == 0) COVM(X_CNT_WRAP);
  d = st < 30 ? &g_d[st] : NULL;
  COVM(W_ST0 + (st < 31 ? st : 31));
  if (!d || d->kind == K_ILL) {
    COVM(m->dbg > 1 ? X_ILL_PRINT : X_ILL_QUIET);
    m_dbg(m, "V92Phase4Modulator: Illegal state\r\n");
    return 0;
  }
  c = s->cnt;
  switch (d->kind) {
    case K_ZERO: COVM(X_ZERO); return 0;
    case K_RU:
      if (c <= 0x18) { COVM(X_RU_TOGGLE); out = m_diff(m, s->inv ^ 1u); }
      else {
        unsigned b;
        if (s->cpLen == 0 || s->cpBitsId < 1 || s->cpBitsId > 3) { m->crash = 1; return 0; }
        b = ((const uint8_t *)g_blk[s->cpBitsId - 1])[(c - 0x19) % s->cpLen];
        COVM(X_RU_BITS);
        r = m_scr1(m, b);
        out = m_diff(m, (r & 0xffu) ^ s->inv);
      }
      if (d->end == E_RU1) { if (s->cpLen == 0) { m->crash = 1; return 0; } if ((c - 0x18) % s->cpLen == 0) m_action(m, d->act, &out); }
      return out;
    case K_E1U:
      r = m_scr1(m, 0);
      out = m_diff(m, (r & 0xffu) ^ s->inv);
      if (s->cnt == s->w1b8) m_action(m, d->act, &out);
      return out;
    case K_PAT: {
      int16_t lv = (int16_t)s->level;
      r = (c - 1) % 6;
      COVM(d->neg ? X_PAT_NEG : X_PAT_POS);
      if (d->neg) out = r <= 2 ? m_neg(lv) : lv; else out = r <= 2 ? lv : m_neg(lv);
      if (c == d->arg) m_action(m, d->act, &out);
      return out;
    }
    case K_MAP:
    case K_E2U:
      if (d->kind == K_E2U && s->w3c != 0) goto data;
      if (s->mult == 0) { m->crash = 1; return 0; }
      if (d->kind == K_E2U) m_scr_bulk(m, 0, s->mult); else m_scr_bulk(m, 1, s->mult);
      if (s->mult == 1) COVM(X_MAP_MULT1);
      { uint32_t v = (uint32_t)s->bits[s->mult - 1] ^ s->inv; s->inv = v; s->bits[s->mult - 1] = (uint8_t)v; if (v > 1) COVM(X_INV_MULTI); }
      out = m_map(m);
      if (d->kind == K_E2U) COVM(X15_MAP);
      break;
    case K_GEN:
      out = m_gen(m, d->gen);
      if (st == 5 && s->cpFlag != 0) { s->cpCnt++; COVM(X5_CPFLAG); }
      break;
    case K_DATA:
    data:
      n = m_nof(m);
      COVM(n ? X_DATA_NOFN : X_DATA_NOF0);
      if (d->kind == K_E2U) COVM(X15_DATA);
      if (n) { m_scr_bulk(m, d->kind == K_DATA, n); m_pbits(m, n); }
      m_pout(m, &n, &y);
      out = y;
      break;
    case K_DRAIN:
      m_pout(m, &n, &y);
      COVM(n ? X_DRAIN_NN : X_DRAIN_N0);
      out = y;
      break;
    default: h_die3("model: kind");
  }
  c = s->cnt;
  switch (d->end) {
    case E_NONE: break;
    case E_EQ: if (c == d->arg) m_action(m, d->act, &out); break;
    case E_EQW1B8: if (c == s->w1b8) m_action(m, d->act, &out); break;
    case E_T4: if (c > 0x3137 && c % 12 == 0) m_action(m, d->act, &out); break;
    case E_T23: if (c >= s->w24) { if (c % 12 == 0) m_action(m, d->act, &out); else COVM(X23_BELOW); } break;
    case E_T24: if (c > 0x95f && c % 12 == 0) m_action(m, d->act, &out); break;
    case E_MODCP: if (s->cpFrame == 0) { m->crash = 1; return 0; } if (c % s->cpFrame == 0) m_action(m, d->act, &out); break;
    case E_EQCPF: if (c == s->cpFrame) m_action(m, d->act, &out); break;
    case E_13:
      if (s->cpFrame == 0) { m->crash = 1; return 0; }
      if (c % s->cpFrame != 0) break;
      if (c == 0) { COVM(X13_ZERO); break; }
      COVM(X13_RESTART); m_cp_restart(m, 0);
      break;
    case E_5:
      if (s->cpFrame == 0) { m->crash = 1; return 0; }
      if (c % s->cpFrame != 0) { COVM(X5_NOMOD); break; }
      if (c == 0) { COVM(X5_ZERO); break; }
      COVM(X5_RESTART);
      m_cp_restart(m, 0);
      if (m->crash) return 0;
      if (s->cpCnt <= s->limit + 0x320u) break;
      s->cpFlag = 0; s->cpCnt = 0;
      COVM(m->dbg > 1 ? X5_REPEAT_PRINT : X5_REPEAT_QUIET);
      m_dbg_n(m, "V92Phase4Modulator: enter repeatedCPu @ %d\r\n", s->cnt);
      s->state = 0xd; s->cpType = 0; m_cp_restart(m, 1);
      s->cnt = 0;
      break;
    case E_DRAIN: if (n != 0) m_action(m, d->act, &out); else COVM(st == 18 ? X18_WAIT_0 : X25_WAIT_0); break;
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
  CMPF(state); CMPF(cnt); CMPF(w08); CMPF(event); CMPF(cpCnt); CMPF(w20); CMPF(w24); CMPF(w28); CMPF(w2c); CMPF(w30); CMPF(w34); CMPF(w38); CMPF(w3c); CMPF(limit); CMPF(inv); CMPF(cpLen); CMPF(cpFrame);
  CMPF(w1b8); CMPF(w1bc); CMPF(w1c0); CMPF(w1c4); CMPF(cpFlag); CMPF(level); CMPF(b42); CMPF(mult); CMPF(cpBitsId); CMPF(hasMp); CMPF(hasCp);
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
static uint8_t sv_b2s[0x40], sv_tx[0x80], sv_me[0x60], sv_map[0x40], sv_mp[0x100], sv_cp[0x1000], sv_blk[3][BLK], sv_par[0x40];
static void save_blocks(void) {
  int k;
  memcpy(sv_b2s, g_b2s, sizeof sv_b2s); memcpy(sv_tx, g_tx, sizeof sv_tx); memcpy(sv_me, g_me, sizeof sv_me); memcpy(sv_map, g_mapper, sizeof sv_map);
  memcpy(sv_mp, g_mp, sizeof sv_mp); memcpy(sv_cp, g_cp, sizeof sv_cp); memcpy(sv_par, g_params, sizeof sv_par);
  for (k = 0; k < 3; k++) memcpy(sv_blk[k], g_blk[k], BLK);
}
static void check_blocks(const S *model) {
  int k;
  uint8_t exp[0x1000], expme[0x60];
  if (memcmp(sv_b2s, g_b2s, sizeof sv_b2s)) h_die3("%s: the V92BitsToSymbol block was written", g_what);
  if (memcmp(sv_tx, g_tx, sizeof sv_tx)) h_die3("%s: the V92Transmitter block was written", g_what);
  if (memcmp(sv_map, g_mapper, sizeof sv_map)) h_die3("%s: the V92Mapper block was written", g_what);
  if (memcmp(sv_mp, g_mp, sizeof sv_mp)) h_die3("%s: the mapping parameters were written", g_what);
  if (memcmp(sv_par, g_params, sizeof sv_par)) h_die3("%s: the V92Parameters block was written", g_what);
  for (k = 0; k < 3; k++) if (memcmp(sv_blk[k], g_blk[k], BLK)) h_die3("%s: a bit block was written", g_what);
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
/* one operation: model first (a call that would crash the blob is skipped), then the object */
static int do_op(void) {
  Mdl m;
  S cur, ns;
  int16_t ms, is;
  g_what = "generateSymbol";
  m = g_m;
  m.l.n = 0;
  m.dbg = dsplibs_debug_level;
  ms = mdl_sym(&m);
  if (m.crash) { if (getenv("P32_DBG2") && g_nskipped < 12) fprintf(stderr, "skip: st=%u cnt=%u cpFrame=%u cpLen=%u mult=%u cpMult=%u\n", g_m.s.state, g_m.s.cnt, g_m.s.cpFrame, g_m.s.cpLen, g_m.s.mult, g_m.s.cpMult); g_nskipped++; return 0; }
  load_impl(&cur);
  cmp_S(&g_m.s, &cur);   /* in sync before the call */
  save_blocks();
  g_il.n = 0;
  g_show = h_sample(&g_symctr, 12, 500);
  is = f_gen(g_o);
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
  return h_rpct(55) ? 2u : h_rpct(30) ? (h_rpct(50) ? 0u : 1u) : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static unsigned pick_pat32(void) {
  static const unsigned v[] = { 0, 1, 2, 0x7fffffffu, 0x80000000u, 0xffffffffu, 0xfffffffeu, 0x18, 0x180, 0x240 };
  return h_rpct(60) ? h_rnd() : v[h_rmod(sizeof v / sizeof v[0])];
}
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
  for (k = 0; k < 3; k++) { g_blk[k] = sysdep_malloc(BLK); }
  fill_random(g_b2s, 0x40); fill_random(g_tx, 0x80); fill_random(g_me, 0x60); fill_random(g_mapper, 0x40); fill_random(g_mp, 0x100); fill_random(g_cp, 0x1000); fill_random(g_params, 0x40); fill_random(g_hist, 0x40);
  for (k = 0; k < 3; k++) { uint8_t *q = (uint8_t *)g_blk[k]; unsigned i; for (i = 0; i < BLK; i++) q[i] = (uint8_t)(h_rpct(50) ? (h_rnd() & 1u) : h_rnd()); }
  ((B2S92 *)g_b2s)->tx_ = g_tx;                 /* the chain V92BitsToSymbol -> V92Transmitter -> V92ModulusEncoder (mode_) */
  ((TxObj *)g_tx)->me_ = g_me;
}
static void free_fakes(void) {
  int k;
  sysdep_free(g_b2s); sysdep_free(g_tx); sysdep_free(g_me); sysdep_free(g_mapper); sysdep_free(g_mp); sysdep_free(g_cp); sysdep_free(g_params); sysdep_free(g_hist);
  for (k = 0; k < 3; k++) sysdep_free(g_blk[k]);
}
/* the object: random in every field (architecture independent: the table drives the fill), then the chosen state */
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
  if (h_rpct(10)) return rnd_range(0, 40);                       /* illegal / out-of-table */
  return rnd_range(0, 29);
}
/* the count to start from so that the next call ends the state (or is one off); the table is the harness' own steering, not the model */
static unsigned near_cnt(const S *s) {
  unsigned st = s->state, j = rnd_range(1, 6), c;
  int d = h_rpct(30) ? (int)h_rmod(3) - 1 : 0;
  switch (st) {
    case 0: case 1: c = h_rpct(25) ? rnd_range(0, 0x1a) : 0x18u + j * s->cpLen - 1u; break;
    case 2: c = s->w1b8 - 1u; break;
    case 4: c = h_rpct(12) ? (h_rpct(50) ? 0xfffffffbu - 12u * h_rmod(3) : 0x80000003u + 12u * h_rmod(3)) : h_rpct(20) ? 0x3137u + 12u * h_rmod(3) - (h_rpct(30) ? 12u : 0u) : 0x3137u; break;
    case 23: c = h_rpct(25) ? 12u * j - 1u : s->w24 + 12u - (s->w24 % 12u) - 1u; break;
    case 24: c = h_rpct(12) ? (h_rpct(50) ? 0xfffffffbu - 12u * h_rmod(3) : 0x80000003u + 12u * h_rmod(3)) : h_rpct(20) ? 0x95fu + 12u * h_rmod(3) - (h_rpct(30) ? 12u : 0u) : 0x95fu; break;
    case 3: c = h_rpct(50) ? h_rmod(40) : pick_pat32(); break;
    case 5: case 6: case 8: case 9: case 11: case 13: c = (h_rpct(10) ? 0u : j) * s->cpFrame - 1u; break;
    case 10: case 12: c = s->cpFrame - 1u; break;
    case 15: c = s->w1b8 - 1u; break;
    case 16: case 17: c = 0x23f; break;
    case 19: c = 0x17f; break;
    case 20: case 27: c = 0x17; break;
    case 26: c = 0x17f; break;
    default: c = h_rpct(50) ? h_rmod(40) : pick_pat32(); break;
  }
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
  s->w08 = pick_pat32(); s->w20 = pick_pat32(); s->w2c = pick_pat32(); s->w1c0 = pick_pat32(); s->w1c4 = pick_pat32();
  s->w28 = h_rpct(65) ? rnd_range(1, 5) : 0u; s->w30 = h_rpct(65) ? rnd_range(1, 5) : 0u; s->w34 = h_rpct(60) ? 0u : pick_pat32();
  s->w38 = h_rpct(50) ? 0u : pick_pat32(); s->w3c = h_rpct(50) ? 0u : pick_pat32();
  s->w1bc = h_rpct(50) ? 0u : pick_pat32();
  s->w24 = h_rpct(30) ? pick_pat32() : 12u * rnd_range(1, 30) + (h_rpct(30) ? h_rmod(12) : 0u);
  s->w1b8 = h_rpct(50) ? 0xcu + h_rmod(2) : rnd_range(1, 20);
  s->inv = h_rpct(30) ? h_rmod(3) : pick_pat32();
  { static const unsigned a[] = { 0, 1, 0x7fff, 0x8000, 0xffff, 0x1234, 0xfffe }; s->level = h_rpct(50) ? a[h_rmod(7)] : h_rnd() & 0xffffu; }
  s->b42 = h_rnd() & 0xffu;
  s->mult = h_rpct(2) ? 0u : h_rpct(5) ? 1u : h_rpct(70) ? rnd_range(2, 40) : rnd_range(1, 255);
  fill_random(s->pad10, 8); fill_random(s->pad1d, 3); fill_random(s->pad1b4, 4);
  s->cpLen = h_rpct(2) ? 0u : rnd_range(2, 3000);
  s->cpFrame = h_rpct(1) ? 0u : h_rpct(80) ? rnd_range(1, 9) : rnd_range(1, 100);
  { static const unsigned cb[] = { 0, 1, 2, 3, 9, 1, 2 }; s->cpBitsId = h_rpct(3) ? 0u : h_rpct(5) ? 9u : cb[1 + h_rmod(3)]; }
  s->hasMp = h_rpct(82); s->hasCp = h_rpct(82);
  s->cpType = pick_pat32() & 0xffu; s->cpFlagCp = h_rnd() & 0xffu; s->cpSuv = pick_pat32(); s->cpSent = pick_pat32(); s->cpMult = h_rpct(2) ? 0u : rnd_range(1, 255); s->meMode = pick_pat32();
  fill_random(s->bits, BITSN);
  s->cnt = near_cnt(s);
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
  g_flip = h_rpct(15);
  g_o->params_ = g_params; g_o->b2s_ = g_b2s; g_o->mapper_ = g_mapper; g_o->scr_buf_ = g_hist; g_o->scr_head0_ = (char *)g_hist + 4; g_o->scr_t1_0_ = (char *)g_hist + 8; g_o->scr_end_ = (char *)g_hist + 16;
  g_o->scr_wr_ = (char *)g_hist + 4; g_o->scr_t1_ = (char *)g_hist + 8; g_o->scr_t2_ = (char *)g_hist + 16; g_o->scr_tail_ = 17;
  store_impl(&s);
  memset(&g_m, 0, sizeof g_m);
  load_impl(&g_m.s);
  g_m.dbg = dsplibs_debug_level;
  g_what = "trial setup";
  dump_obj("init", 0);
  g_ntrials++;
  g_covn[C_TRIALS]++;
  g_covn[dsplibs_debug_level == 0 ? C_LV0 : dsplibs_debug_level == 1 ? C_LV1 : dsplibs_debug_level == 2 ? C_LV2 : C_LVHI]++;
}
static void end_trial(void) {
  free_fakes();
  h_trial_end("p32");
}
static void poke_cnt(unsigned c) { g_m.s.cnt = c; g_o->cnt_ = c; }
static void poke_state(unsigned st) { g_m.s.state = st; g_o->state_ = st; }

/* one random trial: a few operations, the count re-aimed at a threshold between them */
static void trial_random(void) {
  unsigned ops = rnd_range(1, 9), k;
  new_trial();
  for (k = 0; k < ops; k++) {
    if (k > 0 && h_rpct(55)) {
      unsigned c = near_cnt(&g_m.s);
      poke_cnt(c);
      if (h_rpct(10)) { unsigned st = pick_state(); poke_state(st); }
    }
    do_op();
  }
  end_trial();
}

/* ------------------------------------------------------------------------------------------------ natural runs (the owner's events poked in) */
static void chain_setup(unsigned w3c, unsigned rrn) {
  S *s = &g_m.s;
  s->hasMp = 1; s->hasCp = 1;
  s->mult = rnd_range(2, 40); s->cpMult = s->mult; s->cpLen = s->mult * rnd_range(2, 9); s->cpFrame = s->cpLen / s->mult; s->cpBitsId = 1;
  { unsigned q; for (q = 0; q < 4; q++) g_sc.cplen[q] = s->mult * rnd_range(1, 9); }
  s->cpFlag = 0; s->cpCnt = 0; s->limit = 0; s->cnt = rnd_range(0, 5); s->event = 0; s->inv = 0; s->level = 0x1234;
  s->w1bc = rnd_range(0, 1); s->w1b8 = 0xc; s->w3c = w3c; s->w38 = rnd_range(0, 1); s->w2c = rnd_range(1, 20);
  s->w28 = rrn; s->w30 = rrn; s->w34 = 0; s->w24 = 12u * rnd_range(1, 5);
  store_impl(s);
  load_impl(&g_m.s);
}
/* the owner of the blob (V92Modulator / VPcmFloModem) moves the state: the pokes follow the debug strings; `kind` selects the route */
static void chain(unsigned kind) {
  unsigned i, tail = 0, phase = 0;
  new_trial();
  dsplibs_debug_level = 2; g_m.dbg = 2;
  chain_setup(kind == 1, kind == 2);
  poke_state(kind == 3 ? 19 : kind == 4 ? 18 : 1);
  for (i = 0; i < 90000 && tail < 40; i++) {
    unsigned st = g_m.s.state;
    S *s = &g_m.s;
    if (kind <= 2) {
      if (st == 3 && phase == 0 && s->cnt >= 30) { phase = 1; poke_state(4); }
      else if (st == 5 && phase == 1) {
        if (s->cnt >= 3 * s->cpFrame) { phase = 2; s->cpFlag = 1; g_o->cpFlag_ = 1; }
      } else if (st == 0xd && phase == 2 && s->cnt >= 3 * s->cpFrame) { phase = 3; poke_state(12); poke_cnt(0); }
      else if (st == 5 && phase == 3 && s->cnt >= 2 * s->cpFrame) { phase = 4; poke_state(6); }
      else if (st == 0xc && phase == 4 && s->cnt >= 1) { phase = 5; poke_state(8); }
      else if (st == 0xa && phase == 5) {
        phase = 6;
        if (kind == 1) { poke_state(9); } else if (kind == 2) { poke_state(11); }
      }
    } else if (kind == 3) {
      if (st == 3 && phase == 0 && s->cnt >= 20) { phase = 1; poke_state(24); }
      else if (st == 5 && phase == 1 && s->cnt >= 2 * s->cpFrame) { phase = 2; poke_state(9); }
    } else if (kind == 4) {
      if (st == 0x13 && phase == 0) { phase = 1; poke_state(25); }
      else if (st == 5 && phase == 1 && s->cnt >= 2 * s->cpFrame) { phase = 2; poke_state(9); }
    }
    if (st == 0x1c || st == 0x1d) tail++;
    do_op();
  }
  end_trial();
}

/* ------------------------------------------------------------------------------------------------ main */
static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
static void req_range(int lo, int hi) { int i; for (i = lo; i <= hi; i++) g_covreq[i] = 1; }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, i, tr, bad = 0;
  const char *seed = getenv("P32_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P32_DBG")) g_show = 1;
  p11_layout_check(); p15_layout_check(); p29_layout_check(); p32_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  g_covreq[H_COV_STRADDLE] = 0; g_covreq[H_COV_DTOR_NULL] = 0; h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  g_covreq[C_TRIALS] = g_covreq[C_OPS] = 1;
  switch (sc) {
    case 0:
    case 1:
      g_covreq[C_LV0] = g_covreq[C_LV1] = g_covreq[C_LV2] = g_covreq[C_LVHI] = 1;
      req_range(W_ST0 + 0, W_ST0 + 6); req_range(W_ST0 + 8, W_ST0 + 13); req_range(W_ST0 + 15, W_ST0 + 20); req_range(W_ST0 + 23, W_ST0 + 29); g_covreq[W_ST0 + 31] = 1;
      g_covreq[X_E1U_ENTER] = g_covreq[X_E1U_END] = g_covreq[X_T4_END_CP] = g_covreq[X_T4_END_NOCP_PRINT] = g_covreq[X_T4_END_NOCP_QUIET] = g_covreq[X5_NOMOD] = g_covreq[X5_ZERO] = g_covreq[X5_RESTART] = 1;
      g_covreq[X5_REPEAT_PRINT] = g_covreq[X5_REPEAT_QUIET] = g_covreq[X5_CPFLAG] = g_covreq[X6_END_PRINT] = g_covreq[X6_END_QUIET] = g_covreq[X8_END] = g_covreq[X_E2U_ENTER_C] = g_covreq[X_E2U_ENTER_D] = 1;
      g_covreq[X12_END] = g_covreq[X13_RESTART] = g_covreq[X13_ZERO] = g_covreq[X15_RRN] = g_covreq[X15_NOMP_PRINT] = g_covreq[X15_NOMP_QUIET] = g_covreq[X15_B1U] = g_covreq[X15_B1U_NOMP_PRINT] = 1;
      g_covreq[X15_B1U_NOMP_QUIET] = g_covreq[X15_FB1U] = g_covreq[X15_MAP] = g_covreq[X15_DATA] = g_covreq[X16_TERM] = g_covreq[X17_B1U] = g_covreq[X17_NOMP_PRINT] = g_covreq[X17_NOMP_QUIET] = 1;
      g_covreq[X18_WAIT_N] = g_covreq[X18_WAIT_0] = g_covreq[X19_END] = g_covreq[X20_END_PRINT] = g_covreq[X20_END_QUIET] = g_covreq[X23_END] = g_covreq[X23_NOCP_PRINT] = g_covreq[X23_NOCP_QUIET] = 1;
      g_covreq[X23_BELOW] = g_covreq[X24_END] = g_covreq[X24_NOCP_PRINT] = g_covreq[X24_NOCP_QUIET] = g_covreq[X25_WAIT_N] = g_covreq[X25_WAIT_0] = g_covreq[X26_END] = g_covreq[X27_END_PRINT] = 1;
      g_covreq[X27_END_QUIET] = g_covreq[X27_NOMP] = g_covreq[X_ILL_PRINT] = g_covreq[X_ILL_QUIET] = g_covreq[X_ZERO] = g_covreq[X_RU_TOGGLE] = g_covreq[X_RU_BITS] = g_covreq[X_RU_NEG] = g_covreq[X_RU_POS] = 1;
      g_covreq[X_DATA_NOF0] = g_covreq[X_DATA_NOFN] = g_covreq[X_PAT_POS] = g_covreq[X_PAT_NEG] = g_covreq[X_EVENT9] = g_covreq[X_W1B8_C] = g_covreq[X_W1B8_D] = g_covreq[X_MAP_MULT1] = 1;
      g_covreq[X_E2U_RRN_W38_0] = g_covreq[X_E2U_RRN_W38_1] = g_covreq[X_INV_MULTI] = g_covreq[X_DRAIN_N0] = g_covreq[X_DRAIN_NN] = 1;
      for (tr = 0; tr < (sc == 0 ? 4000 : 3000); tr++) trial_random();
      break;
    case 2:
      { static const int ws[] = { 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 20, 23, 24, 25, 26, 27, 28 }; unsigned q;
        for (q = 0; q < sizeof ws / sizeof ws[0]; q++) g_covreq[W_ST0 + ws[q]] = 1; }
      g_covreq[X_E1U_ENTER] = g_covreq[X_E1U_END] = g_covreq[X_T4_END_CP] = g_covreq[X5_RESTART] = g_covreq[X5_REPEAT_PRINT] = g_covreq[X12_END] = g_covreq[X6_END_PRINT] = g_covreq[X8_END] = 1;
      g_covreq[X_E2U_ENTER_D] = g_covreq[X15_B1U] = g_covreq[X15_FB1U] = g_covreq[X16_TERM] = g_covreq[X17_B1U] = g_covreq[X15_RRN] = g_covreq[X23_END] = g_covreq[X24_END] = 1;
      g_covreq[X18_WAIT_N] = g_covreq[X19_END] = g_covreq[X20_END_PRINT] = g_covreq[X25_WAIT_N] = g_covreq[X26_END] = g_covreq[X27_END_PRINT] = g_covreq[X_EVENT9] = 1;
      for (tr = 0; tr < 5; tr++) chain((unsigned)tr);
      for (tr = 0; tr < 3; tr++) { chain(0); chain(1); chain(2); }
      break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu ops %lu skipped %lu trials %lu dumps %lu", h_nmalloc, h_nfree, g_nops, g_nskipped, g_ntrials, g_dumpctr);
  for (i = 1; i < COV_MAX; i++) {
    h_emit(1, "COV %03d %lu", i, g_covn[i]);
    if (g_covreq[i] && g_covn[i] == 0) { fprintf(stderr, "COVERAGE FAIL: counter %d was never reached\n", i); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P32_NOCOV")) ? 2 : 0;
}
