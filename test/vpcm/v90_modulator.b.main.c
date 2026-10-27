/*
 * Differential test harness for port chunk P39, part B (src/vpcm/v92_modulator.cpp: V92Modulator, 18 symbols): C1 / C2, D1 / D2, getV92TxFilterDelay, enterPhase3, enterPhase4, exitJa, exitSilence, exitSuSecond,
 * exitTRN1uSecond, exitCPt, initiateRRN, initiateFPE, enterDataPhase, mkResampledSignal, progress, reset.  Derived from test/vpcm/v90_modulator.a.main.c (the same recorder / model / snapshot design; shared pieces in
 * test/vpcm/p39_common.h).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_modulator.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2) and for
 * x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_modulator.b.blob32.out).  The functions are reached through __asm__ labels; the object is the generated C mirror (test/vpcm/v90_modulator.layout), the objects it drives are
 * the mirrors of P07 (V92Parameters), P11 (V92CP, V92MappingParams), P15 (V92ModulusEncoder), P27 (V92Phase3Modulator), P29 (V92BitsToSymbol, V92Transmitter), P32 (V92Phase4Modulator) and small local mirrors of
 * Queue<float>, FloatFIR and ResamplerTimingOffset (the resampler is a VIRTUAL class: the constructor recorder installs a fake vtable whose slot 1 is the deleting destructor, so the destructor's virtual call
 * is observed).
 *
 * Every callee is a RECORDER with scripted results and side effects: Scrambler<int,uchar> C1 / D1 / reset / process, V92BitsToSymbol C1 / D1 / setSymbolsBlockSize / nofBitsForNextTime / process, V92Phase3Modulator
 * C1 / D1 / reset / generateSymbol / exitJa / exitSilence / exitSuSecond / exitTRN1u, V92Phase4Modulator C1 / D1 / reset / generateSymbol / exitCPt / resetBeforRRN / resetBeforFPE, V92CP::infoToBits, Queue<float>
 * C1 / D1 / reset / write x2 / read (they move rd_ / wr_), FloatFIR C1 / D1 / reset / process, ResamplerTimingOffset C1 / setTimingOffset / resample / getNormalizedPhase / setNormalizedPhase (+ the deleting
 * destructor through the vtable), edprintf, dsplibs_debug_printf; sysdep_malloc / sysdep_free are the harness allocator.  What is compared: the whole object and every foreign block after every call (against the
 * independent model), every record (callee, arguments, results, a snapshot of the object at the moment of the call), the canary behind the object, the canonical output (golden).  The INDEPENDENT MODEL (m_*) is
 * written from porting/decompiled/dp/p39_modulators.txt in the blob's control-flow shape (the jump targets of the disassembly), not from the port source.
 *
 * Scenarios (each its own process): 0 constructors C1 / C2, 1 destructors D1 / D2 (after a C1), 2 the small methods, 3 mkResampledSignal, 4 progress.
 */

#include "p39_common.h"
#include "pcm_parameters_layout.h"
#include "v92_cp_layout.h"
#include "v92_modulus_encoder_layout.h"
#include "pcm_phase3_modulators_layout.h"
#include "pcm_bits_to_symbol_layout.h"
#include "v92_phase4_modulator_symbol_layout.h"
#include "v90_modulator_layout.h"

typedef ModV92Obj T;

#define CT_ARGS void *, unsigned, void *, void *, void *, void *, void *, void *
extern void f_c1(CT_ARGS) L("_ZN12V92ModulatorC1EjP13V92Phase2InfoP5V92JaP19tagV90DILdescriptorP5V92CPP16V92MappingParamsP13V92Parameters");
extern void f_c2(CT_ARGS) L("_ZN12V92ModulatorC2EjP13V92Phase2InfoP5V92JaP19tagV90DILdescriptorP5V92CPP16V92MappingParamsP13V92Parameters");
extern void f_d1(void *) L("_ZN12V92ModulatorD1Ev");
extern void f_d2(void *) L("_ZN12V92ModulatorD2Ev");
extern unsigned f_delay(void *) L("_ZNK12V92Modulator19getV92TxFilterDelayEv");
extern void f_ep3(void *) L("_ZN12V92Modulator11enterPhase3Ev");
extern void f_ep4(void *) L("_ZN12V92Modulator11enterPhase4Ev");
extern void f_exja(void *) L("_ZN12V92Modulator6exitJaEv");
extern void f_exsil(void *) L("_ZN12V92Modulator11exitSilenceEv");
extern void f_exsu(void *) L("_ZN12V92Modulator12exitSuSecondEv");
extern void f_extrn(void *) L("_ZN12V92Modulator15exitTRN1uSecondEv");
extern void f_excpt(void *) L("_ZN12V92Modulator7exitCPtEv");
extern int f_rrn(void *) L("_ZN12V92Modulator11initiateRRNEv");
extern int f_fpe(void *) L("_ZN12V92Modulator11initiateFPEEv");
extern void f_edp(void *) L("_ZN12V92Modulator14enterDataPhaseEv");
extern void f_mk(void *, unsigned *) L("_ZN12V92Modulator17mkResampledSignalERj");
extern void f_prog(void *, int *, unsigned *, float *, unsigned) L("_ZN12V92Modulator8progressEPiRjPfj");
extern void f_reset(void *) L("_ZN12V92Modulator5resetEv");

/* by-value floats: raw words on i386 (the x87 would quiet a signalling NaN in the harness' own call), floats on x86-64 */
#if defined(__i386__)
typedef uint32_t FW;
static inline uint32_t fw_bits(FW v) { return v; }
#else
typedef float FW;
static inline uint32_t fw_bits(FW v) { uint32_t b; memcpy(&b, &v, 4); return b; }
#endif

/* the interposed callees */
extern void c_scrctor(void *, unsigned, unsigned, unsigned) L("_ZN9ScramblerIihEC1Ejjj");
extern void c_scrdtor(void *) L("_ZN9ScramblerIihED1Ev");
extern void c_scrreset(void *, int) L("_ZN9ScramblerIihE5resetEi");
extern void c_scrproc(void *, const int *, unsigned char *, unsigned) L("_ZN9ScramblerIihE7processEPKiPhj");
extern void c_b2sctor(void *, unsigned, void *) L("_ZN15V92BitsToSymbolC1EjP13V92Parameters");
extern void c_b2sdtor(void *) L("_ZN15V92BitsToSymbolD1Ev");
extern unsigned c_b2sssb(void *, unsigned) L("_ZN15V92BitsToSymbol19setSymbolsBlockSizeEj");
extern unsigned c_b2snof(void *) L("_ZN15V92BitsToSymbol18nofBitsForNextTimeEv");
extern unsigned c_b2sproc(void *, unsigned char *, unsigned *, short *) L("_ZN15V92BitsToSymbol7processEPhRjPs");
extern void c_p3ctor(void *, void *) L("_ZN18V92Phase3ModulatorC1EP13V92Parameters");
extern void c_p3dtor(void *) L("_ZN18V92Phase3ModulatorD1Ev");
extern void c_p3reset(void *, short, unsigned, unsigned, void *, void *, unsigned) L("_ZN18V92Phase3Modulator5resetEs23V92Phase3ModulatorStatejP5V92JaPK19tagV90DILdescriptorj");
extern short c_p3gen(void *) L("_ZN18V92Phase3Modulator14generateSymbolEv");
extern void c_p3exja(void *) L("_ZN18V92Phase3Modulator6exitJaEv");
extern void c_p3exsil(void *) L("_ZN18V92Phase3Modulator11exitSilenceEv");
extern void c_p3exsu(void *) L("_ZN18V92Phase3Modulator12exitSuSecondEv");
extern void c_p3extrn(void *) L("_ZN18V92Phase3Modulator9exitTRN1uEv");
extern void c_p4ctor(void *, void *, void *, void *, void *) L("_ZN18V92Phase4ModulatorC1EP13V92ParametersP15V92BitsToSymbolP5V92CPP16V92MappingParams");
extern void c_p4dtor(void *) L("_ZN18V92Phase4ModulatorD1Ev");
extern void c_p4reset(void *, short, unsigned char, unsigned, unsigned, unsigned) L("_ZN18V92Phase4Modulator5resetEsh23V92Phase4ModulatorStatejj");
extern short c_p4gen(void *) L("_ZN18V92Phase4Modulator14generateSymbolEv");
extern void c_p4excpt(void *) L("_ZN18V92Phase4Modulator7exitCPtEv");
extern void c_p4rbr(void *) L("_ZN18V92Phase4Modulator13resetBeforRRNEv");
extern void c_p4rbf(void *) L("_ZN18V92Phase4Modulator13resetBeforFPEEv");
extern void c_cpinfo(void *) L("_ZN5V92CP10infoToBitsEv");
extern void c_qctor(void *, unsigned) L("_ZN5QueueIfEC1Ej");
extern void c_qdtor(void *) L("_ZN5QueueIfED1Ev");
extern void c_qreset(void *) L("_ZN5QueueIfE5resetEv");
extern int c_qw1(void *, FW) L("_ZN5QueueIfE5writeEf");
extern int c_qwn(void *, float *, unsigned) L("_ZN5QueueIfE5writeEPfj");
extern int c_qrd(void *, float *, unsigned) L("_ZN5QueueIfE4readEPfj");
extern void c_firctor(void *, unsigned, float *, unsigned) L("_ZN8FloatFIRC1EjPfj");
extern void c_firdtor(void *) L("_ZN8FloatFIRD1Ev");
extern void c_firreset(void *) L("_ZN8FloatFIR5resetEv");
extern void c_firproc(void *, const float *, float *, unsigned) L("_ZN8FloatFIR7processEPKfPfj");
extern void c_rsctor(void *, unsigned, FW, unsigned, FW, FW, unsigned) L("_ZN21ResamplerTimingOffsetC1Ejfjffj");
extern void c_rssto(void *, FW) L("_ZN21ResamplerTimingOffset15setTimingOffsetEf");
extern void c_rsres(void *, const float *, unsigned, float *, unsigned *) L("_ZN9Resampler8resampleEPKfjPfRj");
extern float c_rsgnp(void *) L("_ZNK9Resampler18getNormalizedPhaseEv");
extern void c_rssnp(void *, FW) L("_ZN9Resampler18setNormalizedPhaseEf");

/* local mirrors (the layouts of the real classes are asserted by the port header on x86-64 and by the sizes below) */
typedef struct { float *buf, *last, *rd, *wr; unsigned size; } QueueObj;
typedef struct { float *coef, *hist; unsigned taps, histLen; int room; } FirObj;
#if defined(__x86_64__)
#define RSZ 88u
#else
#define RSZ 0x4cu
#endif

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_MOD, B_INFO, B_JA, B_DIL, B_MP, B_CP, B_PAR, B_P3, B_P4, B_B2S, B_TX, B_ME, B_RS, B_Q, B_QB, B_FIR, B_SYM, B_FIN, B_FOUT, B_BIT, B_FCARRY, B_IN, B_OUT, B_NB, B_CNT, NB };
static const char *const bname[NB] = { "mod", "info", "ja", "dil", "mp", "cp", "par", "p3", "p4", "b2s", "tx", "me", "rs", "q", "qbuf", "fir", "symbuf", "fin", "fout", "bitbuf", "fcarry", "in", "out", "nbits", "cnt" };
typedef struct { void *b[NB]; } W;
static W g_iw, g_mw;
static uint8_t *g_can;
static int g_blkidx[NB];
#define WT(w) ((T *)(w)->b[B_MOD])
#define WP3(w) ((Ph3V92Obj *)(w)->b[B_P3])
#define WP4(w) ((V92Ph4Obj *)(w)->b[B_P4])
#define WB2S(w) ((B2S92 *)(w)->b[B_B2S])
#define WTX(w) ((TxObj *)(w)->b[B_TX])
#define WME(w) ((V92ModEncObj *)(w)->b[B_ME])
#define WCP(w) ((V92CpObj *)(w)->b[B_CP])
#define WPAR(w) ((V92ParObj *)(w)->b[B_PAR])
#define WQ(w) ((QueueObj *)(w)->b[B_Q])
#define WFIR(w) ((FirObj *)(w)->b[B_FIR])
static unsigned info_rtd(const W *w) { uint32_t v; memcpy(&v, (uint8_t *)w->b[B_INFO] + 4, 4); return v; }
static size_t blk_size(int id) { return g_blk[g_blkidx[id]].n; }

/* a pointer into one of the world's buffers: "name+offset" */
static void locate(const void *p, int *id, long *off) {
  int k, pass;
  *id = -1; *off = 0;
  if (!p) return;
  for (pass = 0; pass < 2; pass++)      /* pass 0: strictly inside a block; pass 1: one past the end of one */
    for (k = 0; k < NB; k++) {
      const uint8_t *b = (const uint8_t *)g_iw.b[k];
      if (!b) continue;
      if ((const uint8_t *)p >= b && ((const uint8_t *)p < b + blk_size(k) || (pass && (const uint8_t *)p == b + blk_size(k)))) { *id = k; *off = (const uint8_t *)p - b; return; }
    }
}
static const char *pd(int id, long off) {
  static char b[8][40];
  static int n;
  char *o = b[n++ & 7];
  if (id < 0) snprintf(o, 40, "wild");
  else snprintf(o, 40, "%s+%ld", bname[id], off);
  return o;
}

/* ------------------------------------------------------------------------------------------------ scripts, cursors */
typedef struct {
  uint32_t sym3[NSC], ev3[NSC], gst3[NSC], gst3on[NSC], f3on[NSC], f3v[NSC];   /* V92Phase3Modulator::generateSymbol: result, event_ afterwards, optional state_ afterwards, optional MODULATOR state flip */
  uint32_t sym4[NSC], ev4[NSC], gst4[NSC], gst4on[NSC], f4on[NSC], f4v[NSC];
  uint32_t nof[NSC], ssb[NSC], flip[NSC], flipv[NSC];
  uint32_t pm[NSC], psym[NSC], pnew[NSC], pret[NSC];
  uint32_t sbits[NSC];
  uint32_t x3s[NSC], x3e[NSC], x4s[NSC], x4e[NSC];
  uint32_t qw[NSC], qr[NSC], qret[NSC];                  /* Queue: the new write / read index, the results */
  uint32_t fb[NSC];                                      /* FloatFIR::process: output pattern */
  uint32_t rn[NSC], rf[NSC];                             /* Resampler::resample: output count and pattern */
  uint32_t gph[NSC];                                     /* getNormalizedPhase: result bits */
} Script;
typedef struct { unsigned sym3, sym4, nof, ssb, bproc, scrproc, x3, x4, qw, qr, fir, rsres, gph; } Cur;
static Script g_sc;
static Cur g_ic, g_mc;
static Log g_il, g_ml;
static int g_show;
static unsigned long g_symctr;
static unsigned g_blkn, g_arg1;   /* blk_ and the constructor's argument of the object (the buffer sizes) */
static int g_snap_subs = 1;
static void *g_rsvt[4];

#define CHECK(p, what) do { if ((p) != (what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

static void obj_snapshot(char *out, size_t cap, int model) {
  const W *w = model ? &g_mw : &g_iw;
  const T *t = WT(w);
  uint32_t h = 2166136261u;
  int i;
  for (i = 0; i < NF_ModV92Obj; i++) {
    const HFld *f = &f_ModV92Obj[i];
    unsigned n = f->count ? f->count : 1, es;
    const uint8_t *q = (const uint8_t *)t + f->off;
    switch (f->kind) {
      case 'p': case 'o': { void *v; int id; memcpy(&v, q, sizeof v); id = tok_of(v); h = fnv_upd(h, &id, sizeof id); continue; }
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      default: es = 4; break;
    }
    h = fnv_upd(h, q, (size_t)n * es);
  }
  {
    const Ph3V92Obj *p3 = g_snap_subs ? WP3(w) : NULL;
    const V92Ph4Obj *p4 = g_snap_subs ? WP4(w) : NULL;
    const V92ModEncObj *me = g_snap_subs ? WME(w) : NULL;
    const V92CpObj *cp = WCP(w);
    snprintf(out, cap, "|h=%08x st=%u cnt=%u ev=%u sp=%u/%u cur=%u p3=%u/%u p4=%u/%u me=%u cp=%u", h, (unsigned)t->state_, (unsigned)t->cnt_, (unsigned)t->event_, (unsigned)t->split_, (unsigned)t->splitAt_,
             (unsigned)t->cur_, p3 ? (unsigned)p3->state_ : 0u, p3 ? (unsigned)p3->event_ : 0u, p4 ? (unsigned)p4->state_ : 0u, p4 ? (unsigned)p4->event_ : 0u, me ? (unsigned)me->mode_ : 0u,
             cp ? (unsigned)cp->flag_ : 0u);
  }
}

/* ------------------------------------------------------------------------------------------------ callee semantics (shared by the impl's recorders and the model) */
#define REC(w, lg, kind, ...) rec_add((lg), (w) == &g_mw, (kind), __VA_ARGS__)
static void cb_scrctor(W *w, Log *l, unsigned a, unsigned b, unsigned c) { REC(w, l, 'c', "scr.ctor(%u,%u,%u)", a, b, c); WT(w)->scr_tail_ = 0x5C000000u | (a << 16) | (b << 8) | c; }
static void cb_scrdtor(W *w, Log *l) { REC(w, l, 'c', "scr.dtor"); WT(w)->scr_tail_ = 0xDEAD5C00u; }
static void cb_scrreset(W *w, Log *l, int v) { REC(w, l, 'c', "scr.reset(%d)", v); WT(w)->scr_tail_ = 0x5C100000u + (unsigned)v; }
static void cb_scrproc(W *w, Cur *c, Log *l, int in, int out, unsigned n) {
  const int *ip = (const int *)w->b[B_IN];
  unsigned k = c->scrproc++ % NSC, j;
  uint8_t *o = (uint8_t *)w->b[B_BIT];
  REC(w, l, 'c', "scr.process(in=%s out=%s n=%u in0=%08x)", pd(in, 0), pd(out, 0), n, n ? (unsigned)ip[0] : 0u);
  for (j = 0; j < n; j++) o[j] = (uint8_t)(g_sc.sbits[k] + j * 7u);
}
static void cb_b2sctor(W *w, Log *l, unsigned cap, int params) { REC(w, l, 'c', "b2s.ctor(cap=%u params=%s)", cap, idn(params)); WB2S(w)->cap_ = cap; WB2S(w)->w0_ = 0xB25C0001u; }
static void cb_b2sdtor(W *w, Log *l) { REC(w, l, 'c', "b2s.dtor"); WB2S(w)->cap_ = 0xDEADB25u; }
static unsigned cb_b2sssb(W *w, Cur *c, Log *l, unsigned n) {
  unsigned k = c->ssb++ % NSC;
  REC(w, l, 'c', "b2s.setSymbolsBlockSize(%u) -> %u", n, g_sc.ssb[k]);
  if (g_sc.flip[k]) WT(w)->state_ = g_sc.flipv[k];
  return g_sc.ssb[k];
}
static unsigned cb_b2snof(W *w, Cur *c, Log *l) { unsigned k = c->nof++ % NSC; REC(w, l, 'c', "b2s.nofBitsForNextTime -> %u", g_sc.nof[k]); return g_sc.nof[k]; }
static unsigned cb_b2sproc(W *w, Cur *c, Log *l, int in, unsigned nin, int out) {
  unsigned k = c->bproc++ % NSC, m, j;
  int16_t *sy = (int16_t *)w->b[B_SYM];
  unsigned *nb = (unsigned *)w->b[B_NB];
  REC(w, l, 'c', "b2s.process(in=%s n=%u out=%s) -> %u", pd(in, 0), nin, pd(out, 0), g_sc.pret[k]);
  m = g_sc.pm[k] % (g_blkn + 1);
  for (j = 0; j < m; j++) sy[j] = (int16_t)(g_sc.psym[k] + j * 0x1111u);
  *nb = g_sc.pnew[k];
  return g_sc.pret[k];
}
static void cb_p3ctor(W *w, Log *l, int params) { REC(w, l, 'c', "p3.ctor(params=%s)", idn(params)); WP3(w)->state_ = 0xC3C30000u; }
static void cb_p3dtor(W *w, Log *l) { REC(w, l, 'c', "p3.dtor"); WP3(w)->cnt_ = 0xDEAD0003u; }
static void cb_p3reset(W *w, Log *l, int s, unsigned st, unsigned n, int ja, int dil, unsigned a6) {
  REC(w, l, 'c', "p3.reset(%d st=%u n=%u ja=%s dil=%s a6=%u)", s, st, n, idn(ja), idn(dil), a6);
  WP3(w)->state_ = st; WP3(w)->event_ = 0;
}
static int cb_p3gen(W *w, Cur *c, Log *l) {
  unsigned k = c->sym3++ % NSC;
  int r = (int)g_sc.sym3[k];
  REC(w, l, 'c', "p3.generateSymbol -> %d", (int)(int16_t)r);
  WP3(w)->event_ = g_sc.ev3[k];
  if (g_sc.gst3on[k]) WP3(w)->state_ = g_sc.gst3[k];
  if (g_sc.f3on[k]) WT(w)->state_ = g_sc.f3v[k];
  return r;
}
static void cb_p3exit(W *w, Cur *c, Log *l, const char *nm) { unsigned k = c->x3++ % NSC; REC(w, l, 'c', "p3.%s", nm); WP3(w)->state_ = g_sc.x3s[k]; WP3(w)->event_ = g_sc.x3e[k]; }
static void cb_p4ctor(W *w, Log *l, int params, int b2s, int cp, int mp) { REC(w, l, 'c', "p4.ctor(params=%s b2s=%s cp=%s mp=%s)", idn(params), idn(b2s), idn(cp), idn(mp)); WP4(w)->state_ = 0xC4C40000u; }
static void cb_p4dtor(W *w, Log *l) { REC(w, l, 'c', "p4.dtor"); WP4(w)->cnt_ = 0xDEAD0004u; }
static void cb_p4reset(W *w, Log *l, int lv, unsigned code, unsigned st, unsigned n, unsigned lim) {
  REC(w, l, 'c', "p4.reset(%d code=%u st=%u n=%u limit=%u)", lv, code & 0xffu, st, n, lim);
  WP4(w)->state_ = st; WP4(w)->event_ = 0;
}
static int cb_p4gen(W *w, Cur *c, Log *l) {
  unsigned k = c->sym4++ % NSC;
  int r = (int)g_sc.sym4[k];
  REC(w, l, 'c', "p4.generateSymbol -> %d", (int)(int16_t)r);
  WP4(w)->event_ = g_sc.ev4[k];
  if (g_sc.gst4on[k]) WP4(w)->state_ = g_sc.gst4[k];
  if (g_sc.f4on[k]) WT(w)->state_ = g_sc.f4v[k];
  return r;
}
static void cb_p4plain(W *w, Log *l, const char *nm) { REC(w, l, 'c', "p4.%s", nm); }
static void cb_p4exit(W *w, Cur *c, Log *l, const char *nm) { unsigned k = c->x4++ % NSC; REC(w, l, 'c', "p4.%s", nm); WP4(w)->state_ = g_sc.x4s[k]; WP4(w)->event_ = g_sc.x4e[k]; }
static void cb_cpinfo(W *w, Log *l) { REC(w, l, 'c', "cp.infoToBits flag=%u", (unsigned)WCP(w)->flag_); }
static void cb_qctor(W *w, Log *l, unsigned cap) {
  QueueObj *q = WQ(w);
  float *buf = (float *)g_iw.b[B_QB];   /* the pointers are the impl's block (the model's copy holds the same values) */
  REC(w, l, 'c', "q.ctor(%u)", cap);
  q->size = cap + 1; q->buf = buf; q->last = buf + cap; q->rd = buf; q->wr = buf;
}
static void cb_qdtor(W *w, Log *l) { REC(w, l, 'c', "q.dtor"); WQ(w)->size = 0xDEAD0005u; }
static void cb_qreset(W *w, Log *l) { QueueObj *q = WQ(w); REC(w, l, 'c', "q.reset"); q->rd = q->buf; q->wr = q->buf; }
static int cb_qw1(W *w, Cur *c, Log *l, uint32_t bits) {
  QueueObj *q = WQ(w);
  unsigned k = c->qw++ % NSC;
  REC(w, l, 'c', "q.write(%08x) -> %d", bits, (int)g_sc.qret[k]);
  q->wr = q->buf + g_sc.qw[k] % q->size;
  return (int)g_sc.qret[k];
}
static int cb_qwn(W *w, Cur *c, Log *l, int sid, long soff, unsigned n) {
  QueueObj *q = WQ(w);
  unsigned k = c->qw++ % NSC;
  REC(w, l, 'c', "q.write(%s, %u) -> %d", pd(sid, soff), n, (int)g_sc.qret[k]);
  q->wr = q->buf + g_sc.qw[k] % q->size;
  return (int)g_sc.qret[k];
}
static int cb_qrd(W *w, Cur *c, Log *l, int did, long doff, unsigned n) {
  QueueObj *q = WQ(w);
  float *dst = (float *)((uint8_t *)w->b[did] + doff);
  unsigned k = c->qr++ % NSC, j;
  REC(w, l, 'c', "q.read(%s, %u) -> %d", pd(did, doff), n, (int)g_sc.qret[k]);
  for (j = 0; j < n; j++) { uint32_t v = g_sc.fb[k] + j * 0x10203u; memcpy(&dst[j], &v, 4); }
  q->rd = q->buf + g_sc.qr[k] % q->size;
  return (int)g_sc.qret[k];
}
static void cb_firctorh(W *w, Log *l, unsigned taps, uint32_t tabhash, unsigned extra) {
  REC(w, l, 'c', "fir.ctor(taps=%u coefhash=%08x extra=%u)", taps, tabhash, extra);
  WFIR(w)->taps = taps; WFIR(w)->histLen = 0xF1F10001u;
}
static void cb_firdtor(W *w, Log *l) { REC(w, l, 'c', "fir.dtor"); WFIR(w)->taps = 0xDEAD0006u; }
static void cb_firreset(W *w, Log *l) { REC(w, l, 'c', "fir.reset"); WFIR(w)->room = 0x5252; }
static void cb_firproc(W *w, Cur *c, Log *l, int iid, long ioff, int oid, long ooff, unsigned n) {
  unsigned k = c->fir++ % NSC, j;
  float *o = (float *)((uint8_t *)w->b[oid] + ooff);
  REC(w, l, 'c', "fir.process(in=%s out=%s n=%u)", pd(iid, ioff), pd(oid, ooff), n);
  for (j = 0; j < n; j++) { uint32_t v = g_sc.fb[k] * 3u + j * 0x30201u; memcpy(&o[j], &v, 4); }
}
static void cb_rsctor(W *w, Log *l, unsigned L_, uint32_t step, unsigned taps, uint32_t cutoff, uint32_t ppm, unsigned hist) {
  REC(w, l, 'c', "rs.ctor(L=%u step=%08x taps=%u cutoff=%08x ppm=%08x hist=%u)", L_, step, taps, cutoff, ppm, hist);
  { uint32_t mk = 0x52530001u; memcpy((uint8_t *)w->b[B_RS] + 2 * sizeof(void *), &mk, 4); }
}
static void cb_rssto(W *w, Log *l, uint32_t bits) { REC(w, l, 'c', "rs.setTimingOffset(%08x)", bits); }
static void cb_rsres(W *w, Cur *c, Log *l, int iid, long ioff, unsigned n, int oid, long ooff, unsigned *nout) {
  unsigned k = c->rsres++ % NSC, cnt, j, capf = 0;
  float *o = (float *)((uint8_t *)w->b[oid] + ooff);
  if (oid >= 0) capf = (unsigned)((blk_size(oid) - (size_t)ooff) / 4);
  cnt = g_sc.rn[k] % (capf / 2 + 1);
  if (oid == B_FCARRY && cnt == 0) cnt = 1;   /* EXCLUDED: an empty second part after a phase wrap makes the blob copy 2^32 - 1 floats (its `dec; cmp 0; jbe` is unsigned) */
  REC(w, l, 'c', "rs.resample(in=%s n=%u out=%s) -> %u", pd(iid, ioff), n, pd(oid, ooff), cnt);
  for (j = 0; j < cnt; j++) { uint32_t v = g_sc.rf[k] + j * 0x01010u; memcpy(&o[j], &v, 4); }
  *nout = cnt;
}
static float cb_rsgnp(W *w, Cur *c, Log *l) { unsigned k = c->gph++ % NSC; float f; memcpy(&f, &g_sc.gph[k], 4); REC(w, l, 'c', "rs.getNormalizedPhase -> %08x", g_sc.gph[k]); return f; }
static void cb_rssnp(W *w, Log *l, uint32_t bits) { REC(w, l, 'c', "rs.setNormalizedPhase(%08x)", bits); }
static void cb_rsd0(W *w, Log *l) { REC(w, l, 'c', "rs.deletingDtor"); { uint32_t mk = 0xDEAD5253u; memcpy((uint8_t *)w->b[B_RS] + 2 * sizeof(void *), &mk, 4); } }
static void cb_ed(W *w, Log *l, char kind, const char *s) { REC(w, l, kind, "%s", s); }

/* the impl's recorders */
#define IW (&g_iw)
#define IL (&g_il)
#define IC (&g_ic)
static void adopt(int id, void *p, const char *what) {
  HAl *a = h_find_al(p, 0);
  if (!a || a->p != (uint8_t *)p) h_die3("%s: the object is not a heap block", what);
  if (g_iw.b[id] == NULL) { g_iw.b[id] = p; g_obj[id] = p; }
  else CHECK(p, g_iw.b[id]);
}
void c_scrctor(void *d, unsigned a, unsigned b, unsigned c) { CHECK(d, &WT(IW)->scr_buf_); cb_scrctor(IW, IL, a, b, c); }
void c_scrdtor(void *d) { CHECK(d, &WT(IW)->scr_buf_); cb_scrdtor(IW, IL); }
void c_scrreset(void *d, int v) { CHECK(d, &WT(IW)->scr_buf_); cb_scrreset(IW, IL, v); }
void c_scrproc(void *d, const int *in, unsigned char *out, unsigned n) {
  int i1, i2; long o1, o2;
  CHECK(d, &WT(IW)->scr_buf_);
  locate(in, &i1, &o1); locate(out, &i2, &o2);
  cb_scrproc(IW, IC, IL, i1, i2, n);
}
void c_b2sctor(void *d, unsigned cap, void *params) { adopt(B_B2S, d, "b2s.ctor"); cb_b2sctor(IW, IL, cap, tok_of(params)); }
void c_b2sdtor(void *d) { CHECK(d, IW->b[B_B2S]); cb_b2sdtor(IW, IL); }
unsigned c_b2sssb(void *d, unsigned n) { CHECK(d, IW->b[B_B2S]); return cb_b2sssb(IW, IC, IL, n); }
unsigned c_b2snof(void *d) { CHECK(d, IW->b[B_B2S]); return cb_b2snof(IW, IC, IL); }
unsigned c_b2sproc(void *d, unsigned char *in, unsigned *n, short *out) {
  int i1, i2; long o1, o2;
  CHECK(d, IW->b[B_B2S]); CHECK((void *)n, IW->b[B_NB]);
  locate(in, &i1, &o1); locate(out, &i2, &o2);
  return cb_b2sproc(IW, IC, IL, i1, *n, i2);
}
void c_p3ctor(void *d, void *params) { adopt(B_P3, d, "p3.ctor"); cb_p3ctor(IW, IL, tok_of(params)); }
void c_p3dtor(void *d) { CHECK(d, IW->b[B_P3]); cb_p3dtor(IW, IL); }
void c_p3reset(void *d, short s, unsigned st, unsigned n, void *ja, void *dil, unsigned a6) { CHECK(d, IW->b[B_P3]); cb_p3reset(IW, IL, s, st, n, tok_of(ja), tok_of(dil), a6); }
short c_p3gen(void *d) { CHECK(d, IW->b[B_P3]); return (short)cb_p3gen(IW, IC, IL); }
void c_p3exja(void *d) { CHECK(d, IW->b[B_P3]); cb_p3exit(IW, IC, IL, "exitJa"); }
void c_p3exsil(void *d) { CHECK(d, IW->b[B_P3]); cb_p3exit(IW, IC, IL, "exitSilence"); }
void c_p3exsu(void *d) { CHECK(d, IW->b[B_P3]); cb_p3exit(IW, IC, IL, "exitSuSecond"); }
void c_p3extrn(void *d) { CHECK(d, IW->b[B_P3]); cb_p3exit(IW, IC, IL, "exitTRN1u"); }
void c_p4ctor(void *d, void *params, void *b2s, void *cp, void *mp) { adopt(B_P4, d, "p4.ctor"); cb_p4ctor(IW, IL, tok_of(params), tok_of(b2s), tok_of(cp), tok_of(mp)); }
void c_p4dtor(void *d) { CHECK(d, IW->b[B_P4]); cb_p4dtor(IW, IL); }
void c_p4reset(void *d, short lv, unsigned char code, unsigned st, unsigned n, unsigned lim) { CHECK(d, IW->b[B_P4]); cb_p4reset(IW, IL, lv, code, st, n, lim); }
short c_p4gen(void *d) { CHECK(d, IW->b[B_P4]); return (short)cb_p4gen(IW, IC, IL); }
void c_p4excpt(void *d) { CHECK(d, IW->b[B_P4]); cb_p4exit(IW, IC, IL, "exitCPt"); }
void c_p4rbr(void *d) { CHECK(d, IW->b[B_P4]); cb_p4plain(IW, IL, "resetBeforRRN"); }
void c_p4rbf(void *d) { CHECK(d, IW->b[B_P4]); cb_p4plain(IW, IL, "resetBeforFPE"); }
void c_cpinfo(void *d) { CHECK(d, IW->b[B_CP]); cb_cpinfo(IW, IL); }
void c_qctor(void *d, unsigned cap) { adopt(B_Q, d, "q.ctor"); cb_qctor(IW, IL, cap); }
void c_qdtor(void *d) { CHECK(d, IW->b[B_Q]); cb_qdtor(IW, IL); }
void c_qreset(void *d) { CHECK(d, IW->b[B_Q]); cb_qreset(IW, IL); }
int c_qw1(void *d, FW v) { CHECK(d, IW->b[B_Q]); return cb_qw1(IW, IC, IL, fw_bits(v)); }
int c_qwn(void *d, float *src, unsigned n) { int i; long o; CHECK(d, IW->b[B_Q]); locate(src, &i, &o); return cb_qwn(IW, IC, IL, i, o, n); }
int c_qrd(void *d, float *dst, unsigned n) { int i; long o; CHECK(d, IW->b[B_Q]); locate(dst, &i, &o); if (i < 0) h_die3("q.read: the destination is not a block of the world"); return cb_qrd(IW, IC, IL, i, o, n); }
static uint32_t tab_hash(const float *c) { return fnv_upd(2166136261u, c, 36 * 4); }
void c_firctor(void *d, unsigned taps, float *coef, unsigned extra) { adopt(B_FIR, d, "fir.ctor"); cb_firctorh(IW, IL, taps, tab_hash(coef), extra); }
void c_firdtor(void *d) { CHECK(d, IW->b[B_FIR]); cb_firdtor(IW, IL); }
void c_firreset(void *d) { CHECK(d, IW->b[B_FIR]); cb_firreset(IW, IL); }
void c_firproc(void *d, const float *in, float *out, unsigned n) {
  int i1, i2; long o1, o2;
  CHECK(d, IW->b[B_FIR]);
  locate(in, &i1, &o1); locate(out, &i2, &o2);
  if (i2 < 0) h_die3("fir.process: the output is not a block of the world");
  cb_firproc(IW, IC, IL, i1, o1, i2, o2, n);
}
void c_rsctor(void *d, unsigned L_, FW step, unsigned taps, FW cutoff, FW ppm, unsigned hist) {
  adopt(B_RS, d, "rs.ctor");
  *(void **)d = &g_rsvt[0];   /* the fake vtable: slot 0 = D1, slot 1 = the deleting destructor */
  cb_rsctor(IW, IL, L_, fw_bits(step), taps, fw_bits(cutoff), fw_bits(ppm), hist);
}
void c_rssto(void *d, FW p) { CHECK(d, IW->b[B_RS]); cb_rssto(IW, IL, fw_bits(p)); }
void c_rsres(void *d, const float *in, unsigned n, float *out, unsigned *nout) {
  int i1, i2; long o1, o2;
  CHECK(d, IW->b[B_RS]);
  locate(in, &i1, &o1); locate(out, &i2, &o2);
  if (i2 < 0) h_die3("rs.resample: the output is not a block of the world");
  cb_rsres(IW, IC, IL, i1, o1, n, i2, o2, nout);
}
float c_rsgnp(void *d) { CHECK(d, IW->b[B_RS]); return cb_rsgnp(IW, IC, IL); }
void c_rssnp(void *d, FW v) { CHECK(d, IW->b[B_RS]); cb_rssnp(IW, IL, fw_bits(v)); }
static void rs_d1(void *d) { (void)d; h_die3("the resampler's complete destructor (vtable slot 0) was called"); }
static void rs_d0(void *d) { CHECK(d, IW->b[B_RS]); cb_rsd0(IW, IL); sysdep_free(d); }
void edprintf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  cb_ed(IW, IL, 'e', buf);
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  cb_ed(IW, IL, 'd', buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the independent model (the blob's control flow, from porting/decompiled/dp/p39_modulators.txt) */
#define MW (&g_mw)
#define MC (&g_mc)
#define ML (&g_ml)
static void m_pf(W *w, char kind, const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  cb_ed(w, ML, kind, b);
}
enum { G_CTOR = 0, G_DTOR = 1, G_SMALL = 2, G_MK = 3, G_PROG = 4 };
#define DBG() (dsplibs_debug_level > 1)
#define SZ_B2S ((unsigned)sizeof(B2S92))
#define SZ_P3 ((unsigned)sizeof(Ph3V92Obj))
#define SZ_P4 ((unsigned)sizeof(V92Ph4Obj))
static void expect_block(const void *p, unsigned size, const char *what) {
  HAl *a = h_find_al(p, 0);
  if (!a || a->p != (const uint8_t *)p || a->size != size) h_die3("model: %s is not a heap block of %u bytes", what, size);
}
static uint32_t fbits(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
/* (int)(0.5 + x * 5/6) in extended precision (fild QWORD, fmul, fadd, fistp QWORD): the low dword */
static unsigned scale56(unsigned x) {
  long double v = (long double)(uint64_t)x * (long double)0.83333331f + (long double)0.5f;
  return (unsigned)(uint64_t)v;
}
static const uint32_t TXTAB[36] = { 0x3999472b, 0xa28644fc, 0xb9f12a7d, 0xbaa9e207, 0xbb326d41, 0xbba194f1, 0xbc041965, 0xbc479cff, 0xbc8d72b3, 0xbcbded54, 0xbcf36330, 0xbd15a2db, 0xbd3141df, 0xbd4ae557,
                                    0xbd60e8bc, 0xbd71d2cd, 0xbd7c78ed, 0x3f7019e5, 0xbd7c78ed, 0xbd71d2cd, 0xbd60e8bc, 0xbd4ae557, 0xbd3141df, 0xbd15a2db, 0xbcf36330, 0xbcbded54, 0xbc8d72b3, 0xbc479cff,
                                    0xbc041965, 0xbba194f1, 0xbb326d41, 0xbaa9e207, 0xb9f12a7d, 0xa28644fc, 0x3999472b, 0 };

/* the Queue's used count: the pointer difference (wr + size - rd) in elements, remainder by size */
static unsigned qused(W *w) {
  QueueObj *q = WQ(w);
  long wi = (long)(q->wr - q->buf), ri = (long)(q->rd - q->buf);
  return (unsigned)(wi + (long)q->size - ri) % q->size;
}

typedef struct { void *a[6]; unsigned arg1; void *sym, *fin, *bit, *fout, *fcarry, *b2s, *rs, *p3, *p4, *q, *fir; } CtorArgs;   /* a: info ja dil cp mp params */
static void m_primeq(W *w) {
  T *t = WT(w);
  uint32_t zero = 0;
  unsigned i;
  t->half_ = (unsigned)(((int32_t)WPAR(w)->modulatorQueueLength) >> 1);
  for (i = 0; i < t->half_; i++) cb_qw1(w, MC, ML, zero);
}
static void m_ctor(W *w, const CtorArgs *c) {
  T *t = WT(w);
  unsigned n = scale56(c->arg1), a1 = c->arg1;
  CV(G_CTOR, "ctor");
  cb_scrctor(w, ML, 5, 0x17, 0x63);
  if (DBG()) { CV(G_CTOR, "ctor: print"); m_pf(w, 'd', "V92Modulator constraction\r\n"); }
  t->info_ = c->a[0]; t->blk_ = n; t->ja_ = c->a[1]; t->cp_ = c->a[3]; t->dil_ = c->a[2]; t->mp_ = c->a[4]; t->params_ = c->a[5];
  expect_block(c->sym, n * 2 + 0x14, "symBuf_"); t->symBuf_ = c->sym;
  expect_block(c->fin, (n + 10) * 4, "fin_"); t->fin_ = c->fin;
  expect_block(c->bit, n * 8, "bitBuf_"); t->bitBuf_ = c->bit;
  expect_block(c->fout, a1 * 4 + 0x28, "fout_"); t->fout_ = c->fout;
  expect_block(c->fcarry, a1 * 4 + 0x28, "fcarry_"); t->fcarry_ = c->fcarry;
  expect_block(c->b2s, SZ_B2S, "b2s_"); cb_b2sctor(w, ML, n * 3, tok_of(c->a[5])); t->b2s_ = c->b2s;
  expect_block(c->rs, RSZ, "rs_"); cb_rsctor(w, ML, 120, 0x42c80000u, 16, 0x3f7ae148u, 0, 0); t->rs_ = c->rs;
  expect_block(c->p3, SZ_P3, "p3_"); cb_p3ctor(w, ML, tok_of(c->a[5])); t->p3_ = c->p3;
  expect_block(c->p4, SZ_P4, "p4_"); cb_p4ctor(w, ML, tok_of(c->a[5]), tok_of(c->b2s), tok_of(c->a[3]), tok_of(c->a[4])); t->p4_ = c->p4;
  expect_block(c->q, (unsigned)sizeof(QueueObj), "q_"); cb_qctor(w, ML, (unsigned)WPAR(w)->modulatorQueueLength); t->q_ = c->q;
  expect_block(c->fir, (unsigned)sizeof(FirObj), "fir_"); cb_firctorh(w, ML, 0x24, fnv_upd(2166136261u, TXTAB, sizeof TXTAB), 0x63); t->fir_ = c->fir;
  /* the reset sequence */
  if (DBG()) m_pf(w, 'd', "V92Modulator reset\r\n");
  cb_scrreset(w, ML, 0);
  t->state_ = 0; t->cnt_ = 0; t->event_ = 0; t->split_ = 0; t->cur_ = t->blk_;
  cb_qreset(w, ML);
  t->codeA_ = 0; t->codeB_ = 0; t->ppm_ = 0.0f;
  m_primeq(w);
  cb_firreset(w, ML);
}
static void m_dtor(W *w) {
  T *t = WT(w);
  CV(G_DTOR, "dtor");
  if (t->p3_) { CV(G_DTOR, "dtor: p3"); cb_p3dtor(w, ML); }
  if (t->p4_) { CV(G_DTOR, "dtor: p4"); cb_p4dtor(w, ML); }
  if (t->b2s_) { CV(G_DTOR, "dtor: b2s"); cb_b2sdtor(w, ML); }
  if (t->rs_) { CV(G_DTOR, "dtor: resampler (virtual)"); cb_rsd0(w, ML); }
  if (t->q_) { CV(G_DTOR, "dtor: queue"); cb_qdtor(w, ML); }
  if (t->fir_) { CV(G_DTOR, "dtor: fir"); cb_firdtor(w, ML); }
  cb_scrdtor(w, ML);
}
static unsigned m_delay(W *w) {
  if (WPAR(w)->v92ApplyTxShapingFilter != 0) { CV(G_SMALL, "getV92TxFilterDelay: filter"); return 0x12u; }
  CV(G_SMALL, "getV92TxFilterDelay: none");
  return 0u;
}
static void m_reset(W *w) {
  T *t = WT(w);
  CV(G_SMALL, "reset");
  if (DBG()) m_pf(w, 'd', "V92Modulator reset\r\n");
  cb_scrreset(w, ML, 0);
  t->state_ = 0; t->cnt_ = 0; t->event_ = 0; t->split_ = 0; t->cur_ = t->blk_;
  cb_qreset(w, ML);
  t->codeA_ = 0; t->codeB_ = 0; t->ppm_ = 0.0f;
  m_primeq(w);
  cb_firreset(w, ML);
}
static void m_p4reset(W *w, unsigned st, unsigned code) { cb_p4reset(w, ML, 0xfa0, code, st, 0, info_rtd(w)); }
static void m_ep3(W *w) {
  T *t = WT(w);
  if (t->state_ == 1) { CV(G_SMALL, "enterPhase3: already"); return; }
  CV(G_SMALL, "enterPhase3");
  if (DBG()) m_pf(w, 'd', "V92Modulator enter Phase 3\r\n");
  cb_p3reset(w, ML, 0xfa0, 0, 0, tok_of(t->ja_), tok_of(t->dil_), info_rtd(w));
  t->state_ = 1; t->cnt_ = 0; t->event_ = 0; t->split_ = 0;
}
static void m_ep4(W *w) {
  T *t = WT(w);
  if (t->state_ == 2) { CV(G_SMALL, "enterPhase4: already"); return; }
  CV(G_SMALL, "enterPhase4");
  if (DBG()) m_pf(w, 'd', "V92Modulator: enter Phase 4\r\n");
  m_p4reset(w, 0, t->codeA_);
  t->state_ = 2; t->cnt_ = 0; t->event_ = 0;
}
static void m_p3x(W *w, unsigned want, const char *what, const char *msg, const char *callee) {
  T *t = WT(w);
  (void)what;
  if (WP3(w)->state_ != want) return;
  if (DBG()) m_pf(w, 'd', "%s", msg);
  cb_p3exit(w, MC, ML, callee);
  t->event_ = 0;
}
static void m_exja(W *w) { if (WP3(w)->state_ == 4) CV(G_SMALL, "exitJa"); else CV(G_SMALL, "exitJa: other state"); m_p3x(w, 4, "exitJa: other state", "V92Modulator: exit Ja\r\n", "exitJa"); }
static void m_exsil(W *w) { if (WP3(w)->state_ == 6) CV(G_SMALL, "exitSilence"); else CV(G_SMALL, "exitSilence: other state"); m_p3x(w, 6, "exitSilence: other state", "V92Modulator: exit Silence\r\n", "exitSilence"); }
static void m_exsu(W *w) { if (WP3(w)->state_ == 9) CV(G_SMALL, "exitSuSecond"); else CV(G_SMALL, "exitSuSecond: other state"); m_p3x(w, 9, "exitSuSecond: other state", "V92Modulator: exit SuSecond\r\n", "exitSuSecond"); }
static void m_extrn(W *w) { if (WP3(w)->state_ == 0xc) CV(G_SMALL, "exitTRN1uSecond"); else CV(G_SMALL, "exitTRN1uSecond: other state"); m_p3x(w, 0xc, "exitTRN1uSecond: other state", "V92Modulator: exit TRN1uSecond\r\n", "exitTRN1u"); }
static void m_excpt(W *w) {
  T *t = WT(w);
  if (WP4(w)->state_ != 0) { CV(G_SMALL, "exitCPt: other state"); return; }
  CV(G_SMALL, "exitCPt");
  if (DBG()) m_pf(w, 'd', "V92Modulator: exit CPt\r\n");
  cb_p4exit(w, MC, ML, "exitCPt");
  t->event_ = 0;
}
static void m_edp(W *w) {
  T *t = WT(w);
  if (t->state_ == 3) { CV(G_SMALL, "enterDataPhase: already"); return; }
  CV(G_SMALL, "enterDataPhase");
  m_pf(w, 'e', "V92Modulator:enter  Data Phase:\r\n");
  t->state_ = 3; t->cnt_ = 0; t->event_ = 0xa;
  cb_b2sssb(w, MC, ML, t->blk_);
}
static int m_req(W *w, int fpe) {
  T *t = WT(w);
  unsigned st;
  if (t->state_ != 3) {
    if (fpe) CV(G_SMALL, "initiateFPE: refused"); else CV(G_SMALL, "initiateRRN: refused");
    if (DBG()) m_pf(w, 'd', fpe ? "V92Modulator: FPE requested but NOT approved\r\n" : "V92Modulator: RRN requested but NOT approved\r\n");
    return -1;
  }
  if (fpe) CV(G_SMALL, "initiateFPE"); else CV(G_SMALL, "initiateRRN");
  if (DBG()) m_pf(w, 'd', fpe ? "V92Modulator: FPE requested, enter Phase 4\r\n" : "V92Modulator: RRN requested, enter Phase 4\r\n");
  t->state_ = 2; t->cnt_ = 0;
  cb_b2sssb(w, MC, ML, 1);
  if (fpe) WME(w)->mode_ = 1;
  if (cb_b2snof(w, MC, ML) == 0) {
    if (fpe) CV(G_SMALL, "initiateFPE: no bits"); else CV(G_SMALL, "initiateRRN: no bits");
    m_pf(w, 'e', fpe ? "V92Modulator: Phase4Modulator state initialized to DataToRmModulation\r\n" : "V92Modulator: Phase4Modulator state initialized to DataToRuModulation\r\n");
    st = fpe ? 0x19u : 0x12u;
  } else {
    if (fpe) CV(G_SMALL, "initiateFPE: bits"); else CV(G_SMALL, "initiateRRN: bits");
    m_pf(w, 'e', fpe ? "V92Modulator: Phase4Modulator state initialized to RmModulation\r\n" : "V92Modulator: Phase4Modulator state initialized to RuModulation\r\n");
    st = fpe ? 0x1au : 0x13u;
  }
  m_p4reset(w, st, t->codeB_);
  t->event_ = 0;
  cb_p4plain(w, ML, fpe ? "resetBeforFPE" : "resetBeforRRN");
  WCP(w)->flag_ = 0;
  cb_cpinfo(w, ML);
  return 0;
}
/* the phase print: "%c%d.%05d" of skew_: trunc(|v|), the sign '+' unless v <= 0, |trunc((v - trunc v) * 100000)| */
static int ldtrunc(long double v) { if (!(v > -2147483649.0L && v < 2147483648.0L)) return (int)0x80000000u; return (int)v; }
static void m_phaseprint(W *w, float skew) {
  long double v = skew;
  int ip = ldtrunc(v), ia = ldtrunc(v < 0 ? -v : v);
  int fp = ldtrunc((v - (long double)ip) * (long double)100000.0f);
  unsigned afp = fp < 0 ? 0u - (unsigned)fp : (unsigned)fp;
  m_pf(w, 'd', "V92Modulator: setPhase = %c%d.%05d\r\n", (v <= 0.0L) ? '-' : '+', ia, (int)afp);
}
static int m_corr(W *w, long double s) {
  int wrapped = 0;
  if (!((long double)1.0L > s)) { wrapped = 1; s = s - (long double)1.0L; }
  cb_rssnp(w, ML, fbits((float)s));
  return wrapped;
}
static void m_mk(W *w, unsigned *outc) {
  T *t = WT(w);
  if (!(t->state_ == 1 && t->split_ != 0)) {
    CV(G_MK, "mk: plain");
    cb_rsres(w, MC, ML, B_FIN, 0, t->cur_, B_FOUT, 0, outc);
    return;
  }
  {
    int wrapped = 0;
    unsigned cnt1 = 0, cnt2 = 0, i, m;
    float *fout = (float *)w->b[B_FOUT], *carry = (float *)w->b[B_FCARRY];
    cb_rsres(w, MC, ML, B_FIN, 0, t->splitAt_, B_FOUT, 0, &cnt1);
    if (t->split_ == 2) {
      CV(G_MK, "mk: split 2");
      if (DBG()) { CV(G_MK, "mk: split 2 print"); m_phaseprint(w, t->skew_); }
      { float ph = cb_rsgnp(w, MC, ML); wrapped = m_corr(w, (long double)ph + (long double)t->skew_); }
    } else if (t->split_ == 1) {
      CV(G_MK, "mk: split 1");
      if (DBG()) { CV(G_MK, "mk: split 1 print"); m_pf(w, 'd', "V92Modulator: setPhase = 0.5\r\n"); }
      { float ph = cb_rsgnp(w, MC, ML); wrapped = m_corr(w, (long double)ph + (long double)0.5f); }
    } else CV(G_MK, "mk: split other");
    cb_rsres(w, MC, ML, B_FIN, (long)t->splitAt_ * 4, t->cur_ - t->splitAt_, B_FCARRY, 0, &cnt2);
    if (wrapped) {
      CV(G_MK, "mk: wrapped");
      *outc = cnt1 + cnt2 - 1;
      m = cnt2 - 1;
      for (i = 0; i < m; i++) memcpy(&fout[cnt1 + i], &carry[i + 1], 4);
    } else {
      CV(G_MK, "mk: not wrapped");
      *outc = cnt1 + cnt2;
      for (i = 0; i < cnt2; i++) memcpy(&fout[cnt1 + i], &carry[i], 4);
    }
    t->split_ = 0; t->splitAt_ = 0;
  }
}
static void m_prog(W *w, unsigned n) {
  T *t = WT(w);
  int16_t *sy = (int16_t *)w->b[B_SYM];
  float *fin = (float *)w->b[B_FIN];
  unsigned *nb = (unsigned *)w->b[B_NB];
  unsigned tmp = scale56(n), cur, i, st, outc = 0;
  uint32_t ppmbits;
  t->event_ = 0;
  t->cnt_ += tmp;
  cur = t->half_ - qused(w) + tmp;
  st = t->state_;
  if (st == 1) {
    unsigned edx = st;
    CV(G_PROG, "progress: state 1");
    t->cur_ = cur;
    i = 0;
    if (cur > 0) {
      for (;;) {
        if (edx == 1) {
          unsigned ev;
          sy[i] = (int16_t)cb_p3gen(w, MC, ML);
          ev = WP3(w)->event_;
          if (ev != 0) {
            CV(G_PROG, "progress: p3 event");
            t->event_ = ev;
            if (ev == 5) { CV(G_PROG, "progress: split 1"); t->split_ = 1; t->splitAt_ = i; }
            else if (ev == 7) { CV(G_PROG, "progress: split 2"); t->split_ = 2; t->splitAt_ = i; }
          }
          if (t->event_ == 8) {
            if (t->state_ != 2) {
              CV(G_PROG, "progress: p3 -> phase 4");
              if (DBG()) m_pf(w, 'd', "V92Modulator: enter Phase 4\r\n");
              m_p4reset(w, 0, t->codeA_);
              t->state_ = 2; t->cnt_ = 0; t->event_ = 0;
            } else CV(G_PROG, "progress: event 8, already phase 4");
            if (DBG()) m_pf(w, 'd', "V92Modulator progress: Enter phase4\r\n");
          }
        } else {
          CV(G_PROG, "progress: state 1, p4 symbol");
          sy[i] = (int16_t)cb_p4gen(w, MC, ML);
          if (WP4(w)->event_ != 0) t->event_ = WP4(w)->event_;
        }
        i++;
        if (t->cur_ <= i) break;
        edx = t->state_;
      }
    } else CV(G_PROG, "progress: state 1, nothing to do");
    *nb = 0;
  } else if (st == 0) {
    CV(G_PROG, "progress: state 0");
    t->cur_ = tmp;
    for (i = 0; i < tmp; i++) sy[i] = 0;
    *nb = 0;
  } else if ((int)st < 0) {
    CV(G_PROG, "progress: negative state");
    t->cur_ = cur;
    if (DBG()) m_pf(w, 'd', "V92Modulator progress: Illegal state\r\n");
  } else {
    t->cur_ = cur;
    if (st == 2) {
      CV(G_PROG, "progress: state 2");
      i = 0;
      if (cur > 0) {
        do {
          sy[i] = (int16_t)cb_p4gen(w, MC, ML);
          if (WP4(w)->event_ != 0) t->event_ = WP4(w)->event_;
          i++;
        } while (t->cur_ > i);
      }
      if (t->event_ == 9) {
        if (t->state_ != 3) {
          CV(G_PROG, "progress: state 2 -> data phase");
          m_pf(w, 'e', "V92Modulator:enter  Data Phase:\r\n");
          t->state_ = 3; t->cnt_ = 0; t->event_ = 0xa;
          cb_b2sssb(w, MC, ML, t->blk_);
        } else CV(G_PROG, "progress: state 2, already data");
        *nb = cb_b2snof(w, MC, ML);
      } else { CV(G_PROG, "progress: state 2, no event 9"); *nb = 0; }
    } else if (st == 3) {
      CV(G_PROG, "progress: state 3");
      cb_scrproc(w, MC, ML, B_IN, B_BIT, *nb);
      cb_b2sproc(w, MC, ML, B_BIT, *nb, B_SYM);
    } else {
      CV(G_PROG, "progress: illegal state");
      if (DBG()) m_pf(w, 'd', "V92Modulator progress: Illegal state\r\n");
    }
  }
  for (i = 0; i < t->cur_; i++) fin[i] = (float)sy[i];
  memcpy(&ppmbits, &t->ppm_, 4);
  cb_rssto(w, ML, ppmbits);
  m_mk(w, &outc);
  if (WPAR(w)->v92ApplyTxShapingFilter != 0) { CV(G_PROG, "progress: filter"); cb_firproc(w, MC, ML, B_FOUT, 0, B_FOUT, 0, outc); } else CV(G_PROG, "progress: no filter");
  cb_qwn(w, MC, ML, B_FOUT, 0, outc);
  cb_qrd(w, MC, ML, B_OUT, 0, n);
  {
    QueueObj *q;
    unsigned c = t->half_ - qused(w) + t->blk_;
    int empty;
    t->cur_ = c;
    if (t->state_ == 3) { CV(G_PROG, "progress: state 3 tail"); *nb = cb_b2sssb(w, MC, ML, c); }
    q = WQ(w);
    if (q->rd == q->wr) { CV(G_PROG, "progress: queue empty"); empty = 1; }
    else if (q->size - qused(w) - 1 == 0) { CV(G_PROG, "progress: queue full"); empty = 1; }
    else { CV(G_PROG, "progress: queue ok"); empty = 0; }
    if (empty) {
      if (DBG()) m_pf(w, 'd', "V92Modulator: Queue is Empty/Full !!!\r\n");
      t->event_ = 1;
    }
  }
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static uint32_t pick_float(void) {
  static const uint32_t v[] = { 0x00000000u, 0x80000000u, 0x3f000000u, 0xbf000000u, 0x3e800000u, 0x3f800000u, 0x3f7fffffu, 0x40490fdbu, 0x47f12064u, 0x501502f9u, 0xd01502f9u, 0x358637bdu, 0x7fc00000u,
                                0x7f800000u, 0xff800000u, 0x00000001u, 0x4f000000u, 0xcf000000u, 0x4f800000u, 0x3dcccccdu, 0xbdcccccdu, 0x3f666666u, 0x3f99999au, 0xbf99999au, 0x3f7ffffeu };
  uint32_t b;
  if (h_rpct(60)) return v[h_rmod(sizeof v / sizeof v[0])];
  b = h_rnd();
  if (((b >> 23) & 0xffu) == 0xffu) b |= 0x400000u;   /* never a signalling NaN (g++ -O0 on i386 quiets one when it copies a float) */
  if (h_rpct(50)) b = (b & 0x807fffffu) | ((0x70u + h_rmod(0x10u)) << 23);   /* moderate magnitudes */
  return b;
}
static void gen_script(void) {
  unsigned i;
  static const int16_t syms[] = { 0, 1, -1, 100, -100, 32767, -32768, 4000, -4000, 0x7f, 0x1234, 255, -255 };
  static const uint32_t e3[] = { 0, 0, 0, 5, 7, 8, 8, 1, 9 };
  static const uint32_t e4[] = { 0, 0, 0, 9, 7, 8, 1 };
  for (i = 0; i < NSC; i++) {
    uint32_t lo = h_rpct(60) ? (uint16_t)syms[h_rmod(sizeof syms / sizeof syms[0])] : (h_rnd() & 0xffffu);
    g_sc.sym3[i] = (h_rnd() & 0xffff0000u) | lo;
    g_sc.ev3[i] = h_rpct(60) ? 0u : e3[h_rmod(sizeof e3 / sizeof e3[0])];
    g_sc.gst3on[i] = h_rpct(20) ? 1u : 0u; g_sc.gst3[i] = h_rmod(16);
    g_sc.f3on[i] = h_rpct(6) ? 1u : 0u; g_sc.f3v[i] = h_rpct(60) ? 2u : h_rmod(5);
    lo = h_rpct(60) ? (uint16_t)syms[h_rmod(sizeof syms / sizeof syms[0])] : (h_rnd() & 0xffffu);
    g_sc.sym4[i] = (h_rnd() & 0xffff0000u) | lo;
    g_sc.ev4[i] = h_rpct(60) ? 0u : e4[h_rmod(sizeof e4 / sizeof e4[0])];
    g_sc.gst4on[i] = h_rpct(20) ? 1u : 0u; g_sc.gst4[i] = h_rmod(32);
    g_sc.f4on[i] = h_rpct(6) ? 1u : 0u; g_sc.f4v[i] = h_rpct(60) ? 3u : h_rmod(5);
    g_sc.nof[i] = h_rpct(40) ? 0u : h_rnd();
    g_sc.ssb[i] = h_rnd();
    g_sc.flip[i] = h_rpct(15) ? 1u : 0u; g_sc.flipv[i] = h_rpct(50) ? 3u : h_rmod(6);
    g_sc.pm[i] = h_rnd(); g_sc.psym[i] = h_rnd(); g_sc.pnew[i] = h_rpct(30) ? 0u : h_rnd() % 400u; g_sc.pret[i] = h_rnd();
    g_sc.sbits[i] = h_rnd();
    g_sc.x3s[i] = h_rmod(16); g_sc.x3e[i] = h_rpct(50) ? 8u : h_rpct(40) ? 0u : h_rnd() % 12u;
    g_sc.x4s[i] = h_rmod(32); g_sc.x4e[i] = h_rpct(50) ? 0u : h_rnd() % 12u;
    g_sc.qw[i] = h_rnd(); g_sc.qr[i] = h_rnd(); g_sc.qret[i] = h_rnd();
    g_sc.fb[i] = h_rnd();
    g_sc.rn[i] = h_rpct(25) ? h_rmod(3) : h_rnd(); g_sc.rf[i] = h_rnd();
    g_sc.gph[i] = pick_float();
  }
}
static void fill_obj(T *t) {
  int i;
  for (i = 0; i < NF_ModV92Obj; i++) {
    const HFld *f = &f_ModV92Obj[i];
    unsigned n = f->count ? f->count : 1, es, tot, j;
    uint8_t *q = (uint8_t *)t + f->off;
    switch (f->kind) {
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'u': case 'i': es = 4; break;
      case 'f': { uint32_t v = pick_float(); memcpy(q, &v, 4); continue; }
      case 'p': case 'o': { void *v = NULL; uintptr_t r = h_rnd(); r = (r << 4) ^ (uintptr_t)h_rnd(); v = (void *)((r & 0x3ffffff0u) | 1u); memcpy(q, &v, sizeof v); continue; }
      default: continue;
    }
    tot = n * es;
    for (j = 0; j < tot; j += 4) { uint32_t r = h_rnd(); memcpy(q + j, &r, tot - j < 4 ? tot - j : 4); }
  }
}
static void reg(int id, void *p, size_t n) { g_iw.b[id] = p; g_blkidx[id] = blk_add(bname[id], p, n); g_obj[id] = p; g_idname[id] = bname[id]; }
static void reg_after(int id, void *p, size_t n) { g_iw.b[id] = p; g_blkidx[id] = blk_add(bname[id], p, n); g_idname[id] = bname[id]; }
/* the heap sizes of the buffers must not equal the (x86-64) sizes the malloc log maps */
static int size_free(unsigned s) {
  static const unsigned bad[] = { 0x20, 0x30, 0x50, 0x80, 0x1cc, 0x208, 0x14, 0x28, 0x4c, 0x58, 0x398 };
  unsigned k;
  for (k = 0; k < sizeof bad / sizeof bad[0]; k++) if (s == bad[k]) return 0;
  return s != (unsigned)sizeof(B2S92) && s != (unsigned)sizeof(Ph3V92Obj) && s != (unsigned)sizeof(V92Ph4Obj) && s != (unsigned)sizeof(QueueObj) && s != (unsigned)sizeof(FirObj) && s != RSZ;
}
static unsigned pick_arg1(void) {
  for (;;) {
    unsigned a = rnd_range(2, 48), n = scale56(a);
    if (size_free(n * 2 + 0x14) && size_free((n + 10) * 4) && size_free(n * 8) && size_free(a * 4 + 0x28)) return a;
  }
}
static void new_world(int with_subs) {
  int k;
  T *t;
  uint8_t *raw = (uint8_t *)h_obj_new(sizeof(T) + CAN);
  memset(&g_iw, 0, sizeof g_iw);
  blk_reset();
  g_nobj = NB;
  for (k = 0; k < NB; k++) { g_obj[k] = NULL; g_idname[k] = bname[k]; }
  memset(raw, 0, sizeof(T));
  memset(raw + sizeof(T), 0x5a, CAN);
  t = (T *)raw; g_can = raw + sizeof(T);
  fill_obj(t);
  reg(B_MOD, raw, sizeof(T));
  reg(B_INFO, h_obj_new(0x2c), 0x2c);
  reg(B_JA, h_obj_new(0x40), 0x40);
  reg(B_DIL, h_obj_new(0x40), 0x40);
  reg(B_MP, h_obj_new(sizeof(MapPar92)), sizeof(MapPar92));
  reg(B_CP, h_obj_new(sizeof(V92CpObj)), sizeof(V92CpObj));
  reg(B_PAR, h_obj_new(sizeof(V92ParObj)), sizeof(V92ParObj));
  reg(B_QB, h_obj_new(64 * 4), 64 * 4);
  reg(B_IN, h_obj_new(8 * 64 * 4), 8 * 64 * 4);
  reg(B_OUT, h_obj_new(64 * 4), 64 * 4);
  reg(B_NB, h_obj_new(16), 16);
  reg(B_CNT, h_obj_new(16), 16);
  { uint32_t v = h_rnd(); memcpy((uint8_t *)g_iw.b[B_INFO] + 4, &v, 4); }
  WCP(&g_iw)->flag_ = (uint8_t)h_rnd();
  WPAR(&g_iw)->v92ApplyTxShapingFilter = (int32_t)(h_rpct(50) ? 0u : h_rpct(70) ? 1u : h_rnd());
  WPAR(&g_iw)->modulatorQueueLength = (int32_t)(h_rpct(10) ? 0u : rnd_range(0, 20));
  { unsigned i; float *qb = (float *)g_iw.b[B_QB]; for (i = 0; i < 64; i++) { uint32_t v = h_rnd(); memcpy(&qb[i], &v, 4); } }
  if (with_subs) {
    unsigned a1 = pick_arg1(), blk = scale56(a1);
    g_arg1 = a1; g_blkn = blk;
    t->blk_ = blk;
    reg(B_SYM, h_obj_new(blk * 2 + 0x14 + 8), blk * 2 + 0x14);
    reg(B_FIN, h_obj_new((blk + 10) * 4 + 8), (blk + 10) * 4);
    reg(B_BIT, h_obj_new(blk * 8 + 8), blk * 8);
    reg(B_FOUT, h_obj_new(a1 * 4 + 0x28 + 8), a1 * 4 + 0x28);
    reg(B_FCARRY, h_obj_new(a1 * 4 + 0x28 + 8), a1 * 4 + 0x28);
    reg(B_P3, h_obj_new(sizeof(Ph3V92Obj)), sizeof(Ph3V92Obj));
    reg(B_P4, h_obj_new(sizeof(V92Ph4Obj)), sizeof(V92Ph4Obj));
    reg(B_B2S, h_obj_new(sizeof(B2S92)), sizeof(B2S92));
    reg(B_TX, h_obj_new(sizeof(TxObj)), sizeof(TxObj));
    reg(B_ME, h_obj_new(sizeof(V92ModEncObj)), sizeof(V92ModEncObj));
    reg(B_RS, h_obj_new(RSZ), RSZ);
    reg(B_Q, h_obj_new(sizeof(QueueObj)), sizeof(QueueObj));
    reg(B_FIR, h_obj_new(sizeof(FirObj)), sizeof(FirObj));
    t->info_ = g_iw.b[B_INFO]; t->ja_ = g_iw.b[B_JA]; t->dil_ = g_iw.b[B_DIL]; t->mp_ = g_iw.b[B_MP]; t->cp_ = g_iw.b[B_CP]; t->params_ = g_iw.b[B_PAR];
    t->p3_ = g_iw.b[B_P3]; t->p4_ = g_iw.b[B_P4]; t->b2s_ = g_iw.b[B_B2S]; t->rs_ = g_iw.b[B_RS]; t->q_ = g_iw.b[B_Q]; t->fir_ = g_iw.b[B_FIR];
    t->symBuf_ = g_iw.b[B_SYM]; t->fin_ = g_iw.b[B_FIN]; t->bitBuf_ = g_iw.b[B_BIT]; t->fout_ = g_iw.b[B_FOUT]; t->fcarry_ = g_iw.b[B_FCARRY];
    ((B2S92 *)g_iw.b[B_B2S])->tx_ = g_iw.b[B_TX];
    ((TxObj *)g_iw.b[B_TX])->me_ = g_iw.b[B_ME];
    WME(&g_iw)->mode_ = h_rnd();
    *(void **)g_iw.b[B_RS] = &g_rsvt[0];
    { unsigned len = (unsigned)WPAR(&g_iw)->modulatorQueueLength;
      QueueObj *q = WQ(&g_iw);
      float *qb = (float *)g_iw.b[B_QB];
      unsigned sz = h_rpct(10) ? 1u : len + 1u;   /* a queue of the parameter's size (or the smallest) */
      q->size = sz; q->buf = qb; q->last = qb + sz - 1;
      q->rd = qb + h_rmod(sz); q->wr = qb + h_rmod(sz);
      t->half_ = h_rpct(85) ? (unsigned)(len >> 1) : h_rmod(11);
      t->cur_ = h_rnd();
    }
    { unsigned i; int16_t *sy = (int16_t *)g_iw.b[B_SYM]; float *fi = (float *)g_iw.b[B_FIN];
      for (i = 0; i < blk + 10; i++) { sy[i] = (int16_t)h_rnd(); fi[i] = 0; { uint32_t v = h_rnd(); memcpy(&fi[i], &v, 4); } }
      { unsigned n2 = a1 + 10; float *fo = (float *)g_iw.b[B_FOUT], *fc = (float *)g_iw.b[B_FCARRY]; for (i = 0; i < n2; i++) { uint32_t v = h_rnd(); memcpy(&fo[i], &v, 4); v = h_rnd(); memcpy(&fc[i], &v, 4); } } }
  }
}
static void pick_subs(T *t) {
  static const unsigned s3[] = { 0, 4, 6, 9, 0xc, 9, 4, 1, 2, 15, 7 };
  static const unsigned s4[] = { 0, 0, 1, 3, 9, 0x12, 5 };
  Ph3V92Obj *p3 = WP3(&g_iw);
  V92Ph4Obj *p4 = WP4(&g_iw);
  p3->state_ = h_rpct(85) ? s3[h_rmod(sizeof s3 / sizeof s3[0])] : h_rnd();
  p3->event_ = h_rpct(50) ? 8u : h_rpct(50) ? 0u : h_rnd();
  p4->state_ = h_rpct(85) ? s4[h_rmod(sizeof s4 / sizeof s4[0])] : h_rnd();
  p4->event_ = h_rpct(50) ? 0u : h_rpct(50) ? 9u : h_rnd();
  t->state_ = h_rpct(80) ? h_rmod(5) : h_rpct(50) ? 0xffffffffu : h_rnd();
  t->cnt_ = h_rpct(50) ? h_rmod(300) : h_rpct(30) ? 0xfffffff0u + h_rmod(16) : h_rnd();
  t->event_ = h_rnd();
  t->split_ = h_rpct(30) ? 0u : h_rpct(35) ? 1u : h_rpct(50) ? 2u : h_rmod(5);
  t->splitAt_ = h_rpct(85) ? h_rmod(g_blkn + 1) : h_rmod(g_blkn + 8);
  t->codeA_ = (uint8_t)h_rnd(); t->codeB_ = (uint8_t)h_rnd();
  { uint32_t v = pick_float(); memcpy(&t->skew_, &v, 4); v = pick_float(); memcpy(&t->ppm_, &v, 4); }
}
/* the queue's used count must not exceed half_ (else cur = half_ - used + n wraps and the blob itself runs far past its buffers) */
static void fix_queue(T *t) {
  QueueObj *q = WQ(&g_iw);
  unsigned sz = q->size, maxu = (sz - 1 < t->half_) ? sz - 1 : t->half_, u, rd;
  u = h_rpct(30) ? maxu : h_rmod(maxu + 1);
  rd = h_rmod(sz);
  q->rd = q->buf + rd;
  q->wr = q->buf + (rd + u) % sz;
}
static unsigned long g_ncalls;
static int g_showall;
static const char *g_what = "setup";
static int g_skip[NB];
static void emit_state(const char *tag) {
  char s[700];
  obj_snapshot(s, sizeof s, 0);
  h_emit(g_show, "%s blk=%u half=%u skew=%08x ppm=%08x %s", tag, (unsigned)WT(&g_iw)->blk_, (unsigned)WT(&g_iw)->half_, fbits(WT(&g_iw)->skew_), fbits(WT(&g_iw)->ppm_), s);
}
static void world_model(void) { int k; for (k = 0; k < NB; k++) g_mw.b[k] = g_iw.b[k] ? g_blk[g_blkidx[k]].m : NULL; }

/* op: 0 C1  1 C2  2 D1  3 D2  4 getV92TxFilterDelay  5 reset  6 enterPhase3  7 enterPhase4  8 exitJa  9 exitSilence  10 exitSuSecond  11 exitTRN1uSecond  12 exitCPt  13 initiateRRN  14 initiateFPE
 *     15 enterDataPhase  16 mkResampledSignal  17 progress */
typedef struct { int op; CtorArgs ca; unsigned n; } Call;
static const char *const opname[] = { "C1", "C2", "D1", "D2", "getV92TxFilterDelay", "reset", "enterPhase3", "enterPhase4", "exitJa", "exitSilence", "exitSuSecond", "exitTRN1uSecond", "exitCPt",
                                      "initiateRRN", "initiateFPE", "enterDataPhase", "mkResampledSignal", "progress" };
static void do_call(Call *c) {
  int cmpres = 0;
  long ir = 0, mr = 0;
  T *t = WT(&g_iw);
  unsigned i;
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = opname[c->op];
  g_snapfn = obj_snapshot;
  g_snap_subs = c->op > 3;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  h_emit(g_show, "call %s n=%u lvl=%u", opname[c->op], c->n, dsplibs_debug_level);
  switch (c->op) {
    case 0: f_c1(t, c->ca.arg1, c->ca.a[0], c->ca.a[1], c->ca.a[2], c->ca.a[3], c->ca.a[4], c->ca.a[5]); break;
    case 1: f_c2(t, c->ca.arg1, c->ca.a[0], c->ca.a[1], c->ca.a[2], c->ca.a[3], c->ca.a[4], c->ca.a[5]); break;
    case 2: f_d1(t); break;
    case 3: f_d2(t); break;
    case 4: ir = (long)f_delay(t); cmpres = 1; break;
    case 5: f_reset(t); break;
    case 6: f_ep3(t); break;
    case 7: f_ep4(t); break;
    case 8: f_exja(t); break;
    case 9: f_exsil(t); break;
    case 10: f_exsu(t); break;
    case 11: f_extrn(t); break;
    case 12: f_excpt(t); break;
    case 13: ir = f_rrn(t); cmpres = 1; break;
    case 14: ir = f_fpe(t); cmpres = 1; break;
    case 15: f_edp(t); break;
    case 16: f_mk(t, (unsigned *)g_iw.b[B_CNT]); break;
    case 17: f_prog(t, (int *)g_iw.b[B_IN], (unsigned *)g_iw.b[B_NB], (float *)g_iw.b[B_OUT], c->n); break;
  }
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  if (c->op <= 1) {   /* the constructor: the heap blocks it allocated, for the model and the later calls */
    c->ca.sym = t->symBuf_; c->ca.fin = t->fin_; c->ca.bit = t->bitBuf_; c->ca.fout = t->fout_; c->ca.fcarry = t->fcarry_;
    c->ca.b2s = t->b2s_; c->ca.rs = t->rs_; c->ca.p3 = t->p3_; c->ca.p4 = t->p4_; c->ca.q = t->q_; c->ca.fir = t->fir_;
    if (!c->ca.sym || !c->ca.fin || !c->ca.bit || !c->ca.fout || !c->ca.fcarry) h_die3("the constructor did not allocate its buffers");
    if (c->ca.b2s != g_iw.b[B_B2S] || c->ca.rs != g_iw.b[B_RS] || c->ca.p3 != g_iw.b[B_P3] || c->ca.p4 != g_iw.b[B_P4] || c->ca.q != g_iw.b[B_Q] || c->ca.fir != g_iw.b[B_FIR])
      h_die3("the constructor's sub-objects are not the ones the recorders saw");
    reg_after(B_SYM, c->ca.sym, ((HAl *)h_find_al(c->ca.sym, 0))->size);
    reg_after(B_FIN, c->ca.fin, ((HAl *)h_find_al(c->ca.fin, 0))->size);
    reg_after(B_BIT, c->ca.bit, ((HAl *)h_find_al(c->ca.bit, 0))->size);
    reg_after(B_FOUT, c->ca.fout, ((HAl *)h_find_al(c->ca.fout, 0))->size);
    reg_after(B_FCARRY, c->ca.fcarry, ((HAl *)h_find_al(c->ca.fcarry, 0))->size);
    reg_after(B_B2S, c->ca.b2s, sizeof(B2S92));
    reg_after(B_RS, c->ca.rs, RSZ);
    reg_after(B_P3, c->ca.p3, sizeof(Ph3V92Obj));
    reg_after(B_P4, c->ca.p4, sizeof(V92Ph4Obj));
    reg_after(B_Q, c->ca.q, sizeof(QueueObj));
    reg_after(B_FIR, c->ca.fir, sizeof(FirObj));
    world_model();
  }
  /* ---- the model */
  switch (c->op) {
    case 0: case 1: m_ctor(MW, &c->ca); break;
    case 2: case 3: m_dtor(MW); break;
    case 4: mr = (long)m_delay(MW); break;
    case 5: m_reset(MW); break;
    case 6: m_ep3(MW); break;
    case 7: m_ep4(MW); break;
    case 8: m_exja(MW); break;
    case 9: m_exsil(MW); break;
    case 10: m_exsu(MW); break;
    case 11: m_extrn(MW); break;
    case 12: m_excpt(MW); break;
    case 13: mr = m_req(MW, 0); break;
    case 14: mr = m_req(MW, 1); break;
    case 15: m_edp(MW); break;
    case 16: m_mk(MW, (unsigned *)MW->b[B_CNT]); break;
    case 17: m_prog(MW, c->n); break;
  }
  if (cmpres && (int)mr != (int)ir) h_die3("%s: result: model %d, impl %d", g_what, (int)mr, (int)ir);
  if (cmpres) h_emit(g_show, "ret %d", (int)ir);
  log_cmp(&g_ml, &g_il, g_what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the callee cursors differ", g_what);
  { int k; size_t keep[NB];
    for (k = 0; k < NB; k++) if (g_iw.b[k] && g_skip[k]) { keep[k] = g_blk[g_blkidx[k]].n; g_blk[g_blkidx[k]].n = 0; }
    blk_compare(g_what);
    for (k = 0; k < NB; k++) if (g_iw.b[k] && g_skip[k]) g_blk[g_blkidx[k]].n = keep[k]; }
  for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i);
  g_snap_subs = 1;
  emit_state("end");
  cov_flush();
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static void pick_ctor_args(Call *c) {
  void *known[6] = { g_iw.b[B_INFO], g_iw.b[B_JA], g_iw.b[B_DIL], g_iw.b[B_CP], g_iw.b[B_MP], g_iw.b[B_PAR] };
  int k;
  for (k = 0; k < 6; k++) {
    c->ca.a[k] = known[k];
    if (k != 5 && h_rpct(6)) c->ca.a[k] = NULL;      /* params_ is dereferenced by the constructor */
    else if (k != 5 && h_rpct(5)) c->ca.a[k] = known[h_rmod(6)];
    else if (k != 5 && h_rpct(3)) { uintptr_t r = h_rnd(); c->ca.a[k] = (void *)((r & 0x3ffffff0u) | 1u); }
  }
  c->ca.arg1 = pick_arg1();
  g_arg1 = c->ca.arg1; g_blkn = scale56(c->ca.arg1);
}
static void trial(int sc) {
  Call c;
  T *t;
  unsigned ncalls = 1, k;
  memset(g_skip, 0, sizeof g_skip);
  new_world(sc >= 2);
  t = WT(&g_iw);
  gen_script();
  dsplibs_debug_level = pick_level();
  if (sc == 1) ncalls = 2;
  if (sc == 2 && h_rpct(30)) ncalls = rnd_range(2, 4);
  if (sc == 3 && h_rpct(20)) ncalls = 2;
  if (sc == 4 && h_rpct(30)) ncalls = rnd_range(2, 3);
  for (k = 0; k < ncalls; k++) {
    memset(&c, 0, sizeof c);
    if (k > 0 && h_rpct(50)) gen_script();
    switch (sc) {
      case 0:
        c.op = (int)h_rmod(2);
        pick_ctor_args(&c);
        break;
      case 1:
        if (k == 0) {
          c.op = (int)h_rmod(2);
          pick_ctor_args(&c);
        } else {
          c.op = 2 + (int)h_rmod(2);
          if (h_rpct(20)) { sysdep_free(t->p3_); t->p3_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->p4_); t->p4_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->b2s_); t->b2s_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->rs_); t->rs_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->q_); t->q_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->fir_); t->fir_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->symBuf_); t->symBuf_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->fin_); t->fin_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->bitBuf_); t->bitBuf_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->fout_); t->fout_ = NULL; }
          if (h_rpct(20)) { sysdep_free(t->fcarry_); t->fcarry_ = NULL; }
          g_skip[B_SYM] = g_skip[B_FIN] = g_skip[B_BIT] = g_skip[B_FOUT] = g_skip[B_FCARRY] = g_skip[B_P3] = g_skip[B_P4] = g_skip[B_B2S] = g_skip[B_RS] = g_skip[B_Q] = g_skip[B_FIR] = 1;
        }
        break;
      case 2: {
        static const int ops[] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 13, 14, 8, 9, 10, 11, 12, 6, 7, 15 };
        c.op = ops[h_rmod(sizeof ops / sizeof ops[0])];
        pick_subs(t);
        if (h_rpct(60)) {
          if (c.op == 13 || c.op == 14 || c.op == 15) t->state_ = h_rpct(75) ? 3u : h_rmod(5);
          if (c.op == 6) t->state_ = h_rpct(50) ? 1u : h_rmod(5);
          if (c.op == 7) t->state_ = h_rpct(50) ? 2u : h_rmod(5);
        }
        break;
      }
      case 3:
        c.op = 16;
        pick_subs(t);
        t->state_ = h_rpct(70) ? 1u : h_rmod(4);
        t->cur_ = h_rpct(85) ? rnd_range(0, g_blkn + 10) : h_rnd() % 400u;
        if (t->cur_ > g_blkn + 10 && h_rpct(70)) t->cur_ = rnd_range(0, g_blkn + 10);
        break;
      default: {
        unsigned i, nb;
        c.op = 17;
        pick_subs(t);
        t->state_ = h_rpct(85) ? h_rmod(4) : h_rpct(50) ? h_rnd() : 4u + h_rmod(3);
        fix_queue(t);
        c.n = h_rpct(8) ? 0u : rnd_range(1, g_arg1);
        nb = h_rpct(20) ? 0u : rnd_range(0, 8 * g_blkn);
        memcpy(g_iw.b[B_NB], &nb, 4);
        for (i = 0; i < 8 * 64; i++) ((int *)g_iw.b[B_IN])[i] = (int)h_rnd();
        for (i = 0; i < 64; i++) ((uint32_t *)g_iw.b[B_OUT])[i] = h_rnd();
        break;
      }
    }
    if (h_rpct(10)) dsplibs_debug_level = pick_level();
    do_call(&c);
    if (sc == 1 && k == 0) { g_blkn = scale56(c.ca.arg1); g_arg1 = c.ca.arg1; }
  }
  h_trial_end("p39b");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("P39B_SEED");
  CovSite *c;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P39B_ALL")) g_showall = 1;
  p07_layout_check(); p11_layout_check(); p15_layout_check(); p27_layout_check(); p29_layout_check(); p32_layout_check(); p39_layout_check();
  h_x87_check();
  h_arena_init();
  h_log_map_add(sizeof(B2S92), 0x20); h_log_map_add(sizeof(Ph3V92Obj), 0x50); h_log_map_add(sizeof(V92Ph4Obj), 0x1cc); h_log_map_add(sizeof(QueueObj), 0x14); h_log_map_add(sizeof(FirObj), 0x14);
  h_log_map_add(RSZ, 0x4c);
  g_rsvt[0] = (void *)rs_d1; g_rsvt[1] = (void *)rs_d0;
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P39B_TRIALS") ? atoi(getenv("P39B_TRIALS")) : (sc >= 2 ? 40000 : 15000);
  if (sc < 0 || sc > 4) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (c = __start_covsites; c < __stop_covsites; c++) {
    if (c->grp != sc) continue;
    h_emit(1, "COV %s %lu", c->name, c->n);
    if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P39B_NOCOV")) ? 2 : 0;
}
