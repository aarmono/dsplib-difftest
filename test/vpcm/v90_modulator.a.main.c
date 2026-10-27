/*
 * Differential test harness for port chunk P39, part A (src/vpcm/v90_modulator.cpp: V90Modulator, 19 symbols): C1 / C2, D1 / D2, setSessionFlag, reset, enterPhase3, enterPhase4, exitJd, exitJdPhase, exitDIL, exitRi,
 * acknowledgeCPReception, acknowledgeCPNotReception, acknowledgeEReception, enterDataPhase, initiateRRN, initiateFPE, progress.  Derived from test/vpcm/v90_phase4_demod.main.c (recorders + independent model + per-record snapshot).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_modulator.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2, the port
 * object alone: the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_modulator.a.blob32.out).  The functions are reached through __asm__ labels; the object is the generated
 * C mirror (test/vpcm/v90_modulator.layout), the objects it drives are the mirrors of P07 (V90Parameters), P09 (V90MP), P10 (V90CP), P11 (V90MappingParams), P27 (V90Phase3Modulator), P28 (V90Mapper), P29 (V90BitsToSymbol), P30
 * (V90Phase4Modulator): member-wise access only (their layouts differ on x86-64).
 *
 * Every callee is a RECORDER with scripted results and side effects (the sub-modulators' state_ / event_ after generateSymbol and exit*, V90BitsToSymbol's results and the symbols / bit count it writes, the
 * state_ flip of setSymbolsBlockSize, the state V90Phase4Modulator::setNextStateAfterTRN2d leaves): Scrambler<int,uchar> C1 / D1 / reset / process, V90BitsToSymbol C1 / D1 / setSymbolsBlockSize /
 * nofBitsForNextTime / process, V90Phase3Modulator C1 / D1 / setSessionFlag / reset / generateSymbol / exitJd / exitJdPhase / exitDIL, V90Phase4Modulator C1 / D1 / setSessionFlag / reset / generateSymbol /
 * setMappingParams / setRdRtSymbols / setRfSymbols / resetBeforRRN / exitRi / exitMP / exitMPNot / setNextStateAfterTRN2d, V90MP::infoToBits, V90CP::infoToBits, displaySpectralParams, edprintf,
 * dsplibs_debug_printf; sysdep_malloc / sysdep_free are the harness allocator (malloc log, red zones, poisoned frees).  What is compared: the whole object and every foreign block after every call (against the
 * independent model), every record (callee, arguments, results, a snapshot of the object at the moment of the call), the canary behind the object, the canonical output (golden).  The INDEPENDENT MODEL (m_*:
 * written from porting/decompiled/dp/p39_modulators.txt in the blob's control-flow shape, not from the port source) predicts the final world and every record; a violation exits 3.
 *
 * Scenarios (each its own process): 0 constructors C1 / C2, 1 destructors D1 / D2 (after a C1), 2 the small methods, 3 progress.
 */

#include "p39_common.h"
#include "pcm_parameters_layout.h"
#include "v90_mp_layout.h"
#include "v90_cp_layout.h"
#include "v92_cp_layout.h"
#include "pcm_phase3_modulators_layout.h"
#include "v90_mapper_layout.h"
#include "pcm_bits_to_symbol_layout.h"
#include "v90_phase4_modulator_symbol_layout.h"
#include "v90_modulator_layout.h"

typedef ModV90Obj T;

/* the functions under test */
#define CT_ARGS void *, unsigned, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, unsigned
extern void f_c1(CT_ARGS) L("_ZN12V90ModulatorC1EjP13V90Phase2InfoP5V90JdP5V92JdP19tagV90DILdescriptorP16V90MappingParamsS9_P22tagV90AdditionalCPinfoP5V90CPP5V90MPP13V90Parametersj");
extern void f_c2(CT_ARGS) L("_ZN12V90ModulatorC2EjP13V90Phase2InfoP5V90JdP5V92JdP19tagV90DILdescriptorP16V90MappingParamsS9_P22tagV90AdditionalCPinfoP5V90CPP5V90MPP13V90Parametersj");
extern void f_d1(void *) L("_ZN12V90ModulatorD1Ev");
extern void f_d2(void *) L("_ZN12V90ModulatorD2Ev");
extern void f_ssf(void *, unsigned) L("_ZN12V90Modulator14setSessionFlagEj");
extern void f_reset(void *) L("_ZN12V90Modulator5resetEv");
extern void f_ep3(void *) L("_ZN12V90Modulator11enterPhase3Ev");
extern void f_ep4(void *) L("_ZN12V90Modulator11enterPhase4Ev");
extern void f_ejd(void *) L("_ZN12V90Modulator6exitJdEv");
extern void f_ejdp(void *) L("_ZN12V90Modulator11exitJdPhaseEv");
extern void f_edil(void *) L("_ZN12V90Modulator7exitDILEv");
extern void f_eri(void *) L("_ZN12V90Modulator6exitRiEv");
extern void f_ackcp(void *) L("_ZN12V90Modulator22acknowledgeCPReceptionEv");
extern void f_ackcpn(void *) L("_ZN12V90Modulator25acknowledgeCPNotReceptionEv");
extern void f_acke(void *) L("_ZN12V90Modulator21acknowledgeEReceptionEv");
extern void f_edp(void *) L("_ZN12V90Modulator14enterDataPhaseEv");
extern int f_rrn(void *) L("_ZN12V90Modulator11initiateRRNEv");
extern int f_fpe(void *) L("_ZN12V90Modulator11initiateFPEEv");
extern void f_prog(void *, int *, unsigned *, float *, unsigned) L("_ZN12V90Modulator8progressEPiRjPfj");

/* the interposed callees */
extern void c_scrctor(void *, unsigned, unsigned, unsigned) L("_ZN9ScramblerIihEC1Ejjj");
extern void c_scrdtor(void *) L("_ZN9ScramblerIihED1Ev");
extern void c_scrreset(void *, int) L("_ZN9ScramblerIihE5resetEi");
extern void c_scrproc(void *, const int *, unsigned char *, unsigned) L("_ZN9ScramblerIihE7processEPKiPhj");
extern void c_b2sctor(void *, unsigned, void *) L("_ZN15V90BitsToSymbolC1EjP13V90Parameters");
extern void c_b2sdtor(void *) L("_ZN15V90BitsToSymbolD1Ev");
extern unsigned c_b2sssb(void *, unsigned) L("_ZN15V90BitsToSymbol19setSymbolsBlockSizeEj");
extern unsigned c_b2snof(void *) L("_ZN15V90BitsToSymbol18nofBitsForNextTimeEv");
extern unsigned c_b2sproc(void *, unsigned char *, unsigned *, short *) L("_ZN15V90BitsToSymbol7processEPhRjPs");
extern void c_p3ctor(void *, void *, unsigned) L("_ZN18V90Phase3ModulatorC1EP13V90Parametersj");
extern void c_p3dtor(void *) L("_ZN18V90Phase3ModulatorD1Ev");
extern void c_p3ssf(void *, unsigned) L("_ZN18V90Phase3Modulator14setSessionFlagEj");
extern void c_p3reset(void *, int, unsigned char, unsigned, unsigned, void *, void *, void *, unsigned) L("_ZN18V90Phase3Modulator5resetE7PcmTypeh20Phase3ModulatorStatejP5V90JdP5V92JdPK19tagV90DILdescriptorj");
extern short c_p3gen(void *) L("_ZN18V90Phase3Modulator14generateSymbolEv");
extern void c_p3exitjd(void *) L("_ZN18V90Phase3Modulator6exitJdEv");
extern void c_p3exitjdp(void *) L("_ZN18V90Phase3Modulator11exitJdPhaseEv");
extern void c_p3exitdil(void *) L("_ZN18V90Phase3Modulator7exitDILEv");
extern void c_p4ctor(void *, void *, unsigned, void *, void *, void *, void *, void *, unsigned) L("_ZN18V90Phase4ModulatorC1EP13V90ParametersjP15V90BitsToSymbolP5V90MPP16V90MappingParamsS7_P5V90CPj");
extern void c_p4dtor(void *) L("_ZN18V90Phase4ModulatorD1Ev");
extern void c_p4ssf(void *, unsigned) L("_ZN18V90Phase4Modulator14setSessionFlagEj");
extern void c_p4reset(void *, int, unsigned char, unsigned, unsigned, unsigned) L("_ZN18V90Phase4Modulator5resetE7PcmTypeh20Phase4ModulatorStatejj");
extern short c_p4gen(void *) L("_ZN18V90Phase4Modulator14generateSymbolEv");
extern void c_p4smp(void *, void *) L("_ZN18V90Phase4Modulator16setMappingParamsEP16V90MappingParams");
extern void c_p4rdrt(void *, void *) L("_ZN18V90Phase4Modulator14setRdRtSymbolsEP16V90MappingParams");
extern void c_p4rf(void *, void *) L("_ZN18V90Phase4Modulator12setRfSymbolsEP16V90MappingParams");
extern void c_p4rbr(void *) L("_ZN18V90Phase4Modulator13resetBeforRRNEv");
extern void c_p4exitri(void *) L("_ZN18V90Phase4Modulator6exitRiEv");
extern void c_p4exitmp(void *) L("_ZN18V90Phase4Modulator6exitMPEv");
extern void c_p4exitmpn(void *) L("_ZN18V90Phase4Modulator9exitMPNotEv");
extern void c_p4next(void *, unsigned) L("_ZN18V90Phase4Modulator22setNextStateAfterTRN2dE20Phase4ModulatorState");
extern void c_mpinfo(void *) L("_ZN5V90MP10infoToBitsEv");
extern void c_cpinfo(void *) L("_ZN5V90CP10infoToBitsEv");
extern void c_dsp(void *) L("displaySpectralParams");

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_MOD, B_INFO, B_JD, B_V92JD, B_DIL, B_ADDCP, B_MP1, B_MP2, B_MP, B_CP, B_PAR, B_P3, B_P4, B_B2S, B_MAPPER, B_SYM, B_BIT, B_IN, B_OUT, B_NB, NB };
static const char *const bname[NB] = { "mod", "info", "jd", "v92jd", "dil", "addcp", "mp1", "mp2", "mp", "cp", "par", "p3", "p4", "b2s", "mapper", "symbuf", "bitbuf", "in", "out", "nbits" };
typedef struct { void *b[NB]; } W;
static W g_iw, g_mw;
static uint8_t *g_can;
#define WT(w) ((T *)(w)->b[B_MOD])
#define WP3(w) ((Ph3V90Obj *)(w)->b[B_P3])
#define WP4(w) ((Ph4Obj *)(w)->b[B_P4])
#define WB2S(w) ((B2S90 *)(w)->b[B_B2S])
#define WMAPPER(w) ((MapObj *)(w)->b[B_MAPPER])
#define WMP(w) ((MpObj *)(w)->b[B_MP])
#define WCP(w) ((CpObj *)(w)->b[B_CP])
#define WMP1(w) ((MapPar90 *)(w)->b[B_MP1])
#define WPAR(w) ((V90ParObj *)(w)->b[B_PAR])
#define WINFO(w) ((uint8_t *)(w)->b[B_INFO])
static int g_blkidx[NB];

static unsigned info_pcm(const W *w) { uint32_t v; memcpy(&v, WINFO(w), 4); return v; }
static unsigned info_rtd(const W *w) { uint32_t v; memcpy(&v, WINFO(w) + 4, 4); return v; }
static unsigned info_ui(const W *w) { return WINFO(w)[8]; }

/* ------------------------------------------------------------------------------------------------ scripts, cursors */
typedef struct {
  uint32_t sym3[NSC], ev3[NSC], gst3[NSC], gst3on[NSC];       /* V90Phase3Modulator::generateSymbol: result (garbage above 16 bits), event_ afterwards, optional state_ afterwards */
  uint32_t sym4[NSC], ev4[NSC], gst4[NSC], gst4on[NSC];
  uint32_t nof[NSC], ssb[NSC], flip[NSC], flipv[NSC];          /* V90BitsToSymbol */
  uint32_t pm[NSC], psym[NSC], pnew[NSC], pret[NSC];           /* V90BitsToSymbol::process: symbols written, symbols value base, new bit count, result */
  uint32_t sbits[NSC];                                         /* Scrambler::process: output bits */
  uint32_t nx[NSC];                                            /* setNextStateAfterTRN2d: the state it leaves */
  uint32_t f3on[NSC], f3v[NSC];                                /* generateSymbol of phase 3 may flip the MODULATOR's state_ too */
  uint32_t f4on[NSC], f4v[NSC];                                /* generateSymbol of phase 4 may also flip the MODULATOR's state_ (the owner reads it again) */
  uint32_t x3s[NSC], x3e[NSC], x4s[NSC], x4e[NSC];             /* the exit* functions: state_ / event_ they leave */
} Script;
typedef struct { unsigned sym3, sym4, nof, ssb, bproc, scrproc, nx, x3, x4; } Cur;
static Script g_sc;
static Cur g_ic, g_mc;
static Log g_il, g_ml;
static int g_show;
static unsigned long g_symctr;
static unsigned g_blkn;     /* blk_ of the object (the sizes of the symbol / bit buffers) */

#define CHECK(p, what) do { if ((p) != (what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

static int g_snap_subs = 1;   /* 0 during a constructor: the sub-objects do not exist yet */
static void obj_snapshot(char *out, size_t cap, int model) {
  const W *w = model ? &g_mw : &g_iw;
  const T *t = WT(w);
  uint32_t h = 2166136261u;
  int i;
  for (i = 0; i < NF_ModV90Obj; i++) {
    const HFld *f = &f_ModV90Obj[i];
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
    const Ph3V90Obj *p3 = g_snap_subs ? WP3(w) : NULL;
    const Ph4Obj *p4 = g_snap_subs ? WP4(w) : NULL;
    const MpObj *mp = WMP(w);
    const CpObj *cp = WCP(w);
    snprintf(out, cap, "|h=%08x st=%u cnt=%u ev=%u p3=%u/%u p4=%u/%u/%u mp=%u/%08x cp=%u/%08x", h, (unsigned)t->state_, (unsigned)t->cnt_, (unsigned)t->event_, p3 ? (unsigned)p3->state_ : 0u,
             p3 ? (unsigned)p3->event_ : 0u, p4 ? (unsigned)p4->state_ : 0u, p4 ? (unsigned)p4->event_ : 0u, p4 ? (unsigned)p4->mpNotSeen_ : 0u, mp ? (unsigned)mp->cPack_ : 0u,
             mp ? (unsigned)mp->trn2dD_ : 0u, cp ? (unsigned)cp->f13_ : 0u, cp ? (unsigned)cp->trn2dD_ : 0u);
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
  REC(w, l, 'c', "scr.process(in=%s out=%s n=%u in0=%08x)", idn(in), idn(out), n, n ? (unsigned)ip[0] : 0u);
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
  (void)nin;
  REC(w, l, 'c', "b2s.process(in=%s n=%u out=%s) -> %u", idn(in), nin, idn(out), g_sc.pret[k]);
  m = g_sc.pm[k] % (g_blkn + 1);
  for (j = 0; j < m; j++) sy[j] = (int16_t)(g_sc.psym[k] + j * 0x1111u);
  *nb = g_sc.pnew[k];
  return g_sc.pret[k];
}
static void cb_p3ctor(W *w, Log *l, int params, unsigned flag) { REC(w, l, 'c', "p3.ctor(params=%s flag=%u)", idn(params), flag); WP3(w)->flag_ = flag; WP3(w)->state_ = 0xC3C30000u; }
static void cb_p3dtor(W *w, Log *l) { REC(w, l, 'c', "p3.dtor"); WP3(w)->flag_ = 0xDEAD0003u; }
static void cb_p3ssf(W *w, Log *l, unsigned f) { REC(w, l, 'c', "p3.setSessionFlag(%u)", f); WP3(w)->flag_ = f; }
static void cb_p3reset(W *w, Log *l, int pcm, unsigned code, unsigned st, unsigned n, int jd, int v92jd, int dil, unsigned tb) {
  REC(w, l, 'c', "p3.reset(pcm=%d code=%u st=%u n=%u jd=%s v92jd=%s dil=%s tb=%u)", pcm, code & 0xffu, st, n, idn(jd), idn(v92jd), idn(dil), tb);
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
static void cb_p4ctor(W *w, Log *l, int params, unsigned flag, int b2s, int mp, int dmp, int tmp, int cp, unsigned a8) {
  REC(w, l, 'c', "p4.ctor(params=%s flag=%u b2s=%s mp=%s dataMp=%s trn2dMp=%s cp=%s a8=%u)", idn(params), flag, idn(b2s), idn(mp), idn(dmp), idn(tmp), idn(cp), a8);
  WP4(w)->flag_ = flag; WP4(w)->state_ = 0xC4C40000u;
}
static void cb_p4dtor(W *w, Log *l) { REC(w, l, 'c', "p4.dtor"); WP4(w)->flag_ = 0xDEAD0004u; }
static void cb_p4ssf(W *w, Log *l, unsigned f) { REC(w, l, 'c', "p4.setSessionFlag(%u)", f); WP4(w)->flag_ = f; }
static void cb_p4reset(W *w, Log *l, int pcm, unsigned code, unsigned st, unsigned n, unsigned lim) {
  REC(w, l, 'c', "p4.reset(pcm=%d code=%u st=%u n=%u limit=%u)", pcm, code & 0xffu, st, n, lim);
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
static void cb_p4mp(W *w, Log *l, const char *nm, int mp) { REC(w, l, 'c', "p4.%s(%s)", nm, idn(mp)); }
static void cb_p4plain(W *w, Log *l, const char *nm) { REC(w, l, 'c', "p4.%s", nm); }
static void cb_p4exit(W *w, Cur *c, Log *l, const char *nm) { unsigned k = c->x4++ % NSC; REC(w, l, 'c', "p4.%s", nm); WP4(w)->state_ = g_sc.x4s[k]; WP4(w)->event_ = g_sc.x4e[k]; }
static void cb_p4next(W *w, Cur *c, Log *l, unsigned st) { unsigned k = c->nx++ % NSC; REC(w, l, 'c', "p4.setNextStateAfterTRN2d(%u)", st); WP4(w)->state_ = g_sc.nx[k]; }
static void cb_mpinfo(W *w, Log *l) { REC(w, l, 'c', "mp.infoToBits cPack=%u", (unsigned)WMP(w)->cPack_); }
static void cb_cpinfo(W *w, Log *l) { REC(w, l, 'c', "cp.infoToBits f13=%u", (unsigned)WCP(w)->f13_); }
static void cb_dsp(W *w, Log *l, int mp) { REC(w, l, 'c', "displaySpectralParams(%s w0=%u)", idn(mp), mp >= 0 && mp < NB ? (unsigned)((MapPar90 *)w->b[mp])->w0_ : 0u); }
static void cb_ed(W *w, Log *l, char kind, const char *s) { REC(w, l, kind, "%s", s); }

/* the impl's recorders */
#define IW (&g_iw)
#define IL (&g_il)
#define IC (&g_ic)
void c_scrctor(void *d, unsigned a, unsigned b, unsigned c) { CHECK(d, &WT(IW)->scr_buf_); cb_scrctor(IW, IL, a, b, c); }
void c_scrdtor(void *d) { CHECK(d, &WT(IW)->scr_buf_); cb_scrdtor(IW, IL); }
void c_scrreset(void *d, int v) { CHECK(d, &WT(IW)->scr_buf_); cb_scrreset(IW, IL, v); }
void c_scrproc(void *d, const int *in, unsigned char *out, unsigned n) { CHECK(d, &WT(IW)->scr_buf_); cb_scrproc(IW, IC, IL, tok_of(in), tok_of(out), n); }
static void adopt(int id, void *p, const char *what) {
  HAl *a = h_find_al(p, 0);
  if (!a || a->p != (uint8_t *)p) h_die3("%s: the object is not a heap block", what);
  if (g_iw.b[id] == NULL) { g_iw.b[id] = p; g_obj[id] = p; }
  else CHECK(p, g_iw.b[id]);
}
void c_b2sctor(void *d, unsigned cap, void *params) { adopt(B_B2S, d, "b2s.ctor"); cb_b2sctor(IW, IL, cap, tok_of(params)); }
void c_b2sdtor(void *d) { CHECK(d, IW->b[B_B2S]); cb_b2sdtor(IW, IL); }
unsigned c_b2sssb(void *d, unsigned n) { CHECK(d, IW->b[B_B2S]); return cb_b2sssb(IW, IC, IL, n); }
unsigned c_b2snof(void *d) { CHECK(d, IW->b[B_B2S]); return cb_b2snof(IW, IC, IL); }
unsigned c_b2sproc(void *d, unsigned char *in, unsigned *n, short *out) { CHECK(d, IW->b[B_B2S]); CHECK((void *)n, IW->b[B_NB]); return cb_b2sproc(IW, IC, IL, tok_of(in), *n, tok_of(out)); }
void c_p3ctor(void *d, void *params, unsigned flag) { adopt(B_P3, d, "p3.ctor"); cb_p3ctor(IW, IL, tok_of(params), flag); }
void c_p3dtor(void *d) { CHECK(d, IW->b[B_P3]); cb_p3dtor(IW, IL); }
void c_p3ssf(void *d, unsigned f) { CHECK(d, IW->b[B_P3]); cb_p3ssf(IW, IL, f); }
void c_p3reset(void *d, int pcm, unsigned char code, unsigned st, unsigned n, void *jd, void *v92jd, void *dil, unsigned tb) {
  CHECK(d, IW->b[B_P3]);
  cb_p3reset(IW, IL, pcm, code, st, n, tok_of(jd), tok_of(v92jd), tok_of(dil), tb);
}
short c_p3gen(void *d) { CHECK(d, IW->b[B_P3]); return (short)cb_p3gen(IW, IC, IL); }
void c_p3exitjd(void *d) { CHECK(d, IW->b[B_P3]); cb_p3exit(IW, IC, IL, "exitJd"); }
void c_p3exitjdp(void *d) { CHECK(d, IW->b[B_P3]); cb_p3exit(IW, IC, IL, "exitJdPhase"); }
void c_p3exitdil(void *d) { CHECK(d, IW->b[B_P3]); cb_p3exit(IW, IC, IL, "exitDIL"); }
void c_p4ctor(void *d, void *params, unsigned flag, void *b2s, void *mp, void *dmp, void *tmp, void *cp, unsigned a8) {
  adopt(B_P4, d, "p4.ctor");
  cb_p4ctor(IW, IL, tok_of(params), flag, tok_of(b2s), tok_of(mp), tok_of(dmp), tok_of(tmp), tok_of(cp), a8);
}
void c_p4dtor(void *d) { CHECK(d, IW->b[B_P4]); cb_p4dtor(IW, IL); }
void c_p4ssf(void *d, unsigned f) { CHECK(d, IW->b[B_P4]); cb_p4ssf(IW, IL, f); }
void c_p4reset(void *d, int pcm, unsigned char code, unsigned st, unsigned n, unsigned lim) { CHECK(d, IW->b[B_P4]); cb_p4reset(IW, IL, pcm, code, st, n, lim); }
short c_p4gen(void *d) { CHECK(d, IW->b[B_P4]); return (short)cb_p4gen(IW, IC, IL); }
void c_p4smp(void *d, void *mp) { CHECK(d, IW->b[B_P4]); cb_p4mp(IW, IL, "setMappingParams", tok_of(mp)); }
void c_p4rdrt(void *d, void *mp) { CHECK(d, IW->b[B_P4]); cb_p4mp(IW, IL, "setRdRtSymbols", tok_of(mp)); }
void c_p4rf(void *d, void *mp) { CHECK(d, IW->b[B_P4]); cb_p4mp(IW, IL, "setRfSymbols", tok_of(mp)); }
void c_p4rbr(void *d) { CHECK(d, IW->b[B_P4]); cb_p4plain(IW, IL, "resetBeforRRN"); }
void c_p4exitri(void *d) { CHECK(d, IW->b[B_P4]); cb_p4exit(IW, IC, IL, "exitRi"); }
void c_p4exitmp(void *d) { CHECK(d, IW->b[B_P4]); cb_p4exit(IW, IC, IL, "exitMP"); }
void c_p4exitmpn(void *d) { CHECK(d, IW->b[B_P4]); cb_p4exit(IW, IC, IL, "exitMPNot"); }
void c_p4next(void *d, unsigned st) { CHECK(d, IW->b[B_P4]); cb_p4next(IW, IC, IL, st); }
void c_mpinfo(void *d) { CHECK(d, IW->b[B_MP]); cb_mpinfo(IW, IL); }
void c_cpinfo(void *d) { CHECK(d, IW->b[B_CP]); cb_cpinfo(IW, IL); }
void c_dsp(void *mp) { cb_dsp(IW, IL, tok_of(mp)); }
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
enum { G_CTOR = 0, G_DTOR = 1, G_SMALL = 2, G_PROG = 3 };
#define DBG() (dsplibs_debug_level > 1)

/* the expected sizes of the heap blocks (i386 value; the mirror's sizeof on x86-64 = the real class size) */
#define SZ_B2S ((unsigned)sizeof(B2S90))
#define SZ_P3 ((unsigned)sizeof(Ph3V90Obj))
#define SZ_P4 ((unsigned)sizeof(Ph4Obj))
static void expect_block(const void *p, unsigned size, const char *what) {
  HAl *a = h_find_al(p, 0);
  if (!a || a->p != (const uint8_t *)p || a->size != size) h_die3("model: %s is not a heap block of %u bytes", what, size);
}

/* a constructor: the model runs AFTER the impl; the heap blocks are the impl's (taken from its object) */
typedef struct { void *a[10]; unsigned blk, session; void *sym, *bit, *b2s, *p3, *p4; } CtorArgs;   /* a: info jd v92jd dil mp1 mp2 addcp cp mp params; the blocks: the impl's allocations */
static void m_ctor(W *w, const CtorArgs *c) {
  T *t = WT(w);
  void *sym = c->sym, *bit = c->bit, *b2s = c->b2s, *p3 = c->p3, *p4 = c->p4;
  unsigned blk = c->blk;
  CV(G_CTOR, "ctor");
  cb_scrctor(w, ML, 0x12, 0x17, 0x63);
  t->mp1_ = c->a[4]; t->info_ = c->a[0]; t->jd_ = c->a[1]; t->v92jd_ = c->a[2]; t->dil_ = c->a[3]; t->mp2_ = c->a[5]; t->mp_ = c->a[8]; t->cp_ = c->a[7]; t->addCp_ = c->a[6];
  t->session_ = c->session; t->blk_ = blk; t->params_ = c->a[9];
  expect_block(sym, blk * 2, "symBuf_");
  t->symBuf_ = sym;
  expect_block(bit, blk * 8, "bitBuf_");
  t->bitBuf_ = bit;
  expect_block(b2s, SZ_B2S, "b2s_");
  cb_b2sctor(w, ML, blk * 3 + 5000, tok_of(c->a[9]));
  t->b2s_ = b2s;
  expect_block(p3, SZ_P3, "p3_");
  cb_p3ctor(w, ML, tok_of(c->a[9]), c->session);
  t->p3_ = p3;
  expect_block(p4, SZ_P4, "p4_");
  cb_p4ctor(w, ML, tok_of(c->a[9]), c->session, tok_of(b2s), tok_of(c->a[8]), tok_of(c->a[5]), tok_of(c->a[4]), tok_of(c->a[7]), 0xc);
  t->p4_ = p4;
}
static void m_dtor(W *w) {
  T *t = WT(w);
  CV(G_DTOR, "dtor");
  if (t->p3_) { CV(G_DTOR, "dtor: p3"); cb_p3dtor(w, ML); }
  if (t->p4_) { CV(G_DTOR, "dtor: p4"); cb_p4dtor(w, ML); }
  if (t->b2s_) { CV(G_DTOR, "dtor: b2s"); cb_b2sdtor(w, ML); }
  cb_scrdtor(w, ML);
}

static void m_ssf(W *w, unsigned f) {
  T *t = WT(w);
  CV(G_SMALL, "setSessionFlag");
  t->session_ = f;
  cb_p3ssf(w, ML, f);
  cb_p4ssf(w, ML, f);
}
static void m_reset(W *w) {
  T *t = WT(w);
  CV(G_SMALL, "reset");
  if (DBG()) m_pf(w, 'd', "V90Modulator reset\r\n");
  cb_scrreset(w, ML, 0);
  t->state_ = 0; t->cnt_ = 0; t->event_ = 0;
}
static void m_p4reset(W *w, unsigned st) { cb_p4reset(w, ML, (int)info_pcm(w), info_ui(w), st, 0, info_rtd(w)); }
static void m_ep3(W *w) {
  T *t = WT(w);
  if (t->state_ == 1) { CV(G_SMALL, "enterPhase3: already"); return; }
  CV(G_SMALL, "enterPhase3");
  if (DBG()) m_pf(w, 'd', "V90Modulator enter Phase 3\r\n");
  cb_p3reset(w, ML, (int)info_pcm(w), info_ui(w), 0, 0, tok_of(t->jd_), tok_of(t->v92jd_), tok_of(t->dil_), info_rtd(w));
  t->state_ = 1; t->cnt_ = 0; t->event_ = 0;
}
static void m_ep4(W *w) {
  T *t = WT(w);
  if (t->state_ == 2) { CV(G_SMALL, "enterPhase4: already"); return; }
  CV(G_SMALL, "enterPhase4");
  if (DBG()) m_pf(w, 'd', "V90Modulator: enter Phase 4\r\n");
  m_p4reset(w, 0);
  t->state_ = 2; t->cnt_ = 0; t->event_ = 0;
}
static void m_ejd(W *w) {
  T *t = WT(w);
  if (WP3(w)->state_ != 3) { CV(G_SMALL, "exitJd: other state"); return; }
  CV(G_SMALL, "exitJd");
  if (DBG()) m_pf(w, 'd', "V90Modulator: exit Jd\r\n");
  cb_p3exit(w, MC, ML, "exitJd");
  t->event_ = 0;
}
static void m_ejdp(W *w) {
  T *t = WT(w);
  if (WP3(w)->state_ != 5) { CV(G_SMALL, "exitJdPhase: other state"); return; }
  CV(G_SMALL, "exitJdPhase");
  if (DBG()) m_pf(w, 'd', "V90Modulator: exit JdPhase\r\n");
  cb_p3exit(w, MC, ML, "exitJdPhase");
  t->event_ = 0;
}
static void m_edil(W *w) {
  T *t = WT(w);
  if (WP3(w)->state_ != 9) { CV(G_SMALL, "exitDIL: other state"); return; }
  CV(G_SMALL, "exitDIL");
  if (DBG()) m_pf(w, 'd', "V90Modulator: exit DIL\r\n");
  cb_p3exit(w, MC, ML, "exitDIL");
  if (WP3(w)->event_ != 6) { CV(G_SMALL, "exitDIL: event not 6"); return; }
  if (t->state_ == 2) { CV(G_SMALL, "exitDIL: already phase 4"); return; }
  CV(G_SMALL, "exitDIL: enter phase 4");
  if (DBG()) m_pf(w, 'd', "V90Modulator: enter Phase 4\r\n");
  m_p4reset(w, 0);
  t->state_ = 2; t->cnt_ = 0; t->event_ = 0;
}
static void m_eri(W *w) {
  T *t = WT(w);
  if (WP4(w)->state_ != 0) { CV(G_SMALL, "exitRi: other state"); return; }
  CV(G_SMALL, "exitRi");
  if (DBG()) m_pf(w, 'd', "V90Modulator: exit Ri\r\n");
  cb_p4exit(w, MC, ML, "exitRi");
  t->event_ = 0;
  cb_p4mp(w, ML, "setMappingParams", tok_of(t->mp1_));
  m_pf(w, 'e', "V90Modulator: TRN2d spectral parameters:\r\n");
  cb_dsp(w, ML, tok_of(t->mp1_));
  m_pf(w, 'e', "V90Modulator: TRN2d D = %d\r\n", (int)WMP1(w)->w0_);
  if (t->session_) { CV(G_SMALL, "exitRi: V.92 (CP)"); WCP(w)->trn2dD_ = WMP1(w)->w0_; }
  else { CV(G_SMALL, "exitRi: V.90 (MP)"); WMP(w)->trn2dD_ = WMP1(w)->w0_; }
}
static void m_ackcp(W *w) {
  T *t = WT(w);
  CV(G_SMALL, "ackCP");
  WMP(w)->cPack_ = 1;
  cb_mpinfo(w, ML);
  cb_p4next(w, MC, ML, 0xe);
  if (WP4(w)->state_ != 4) { CV(G_SMALL, "ackCP: state not 4"); return; }
  CV(G_SMALL, "ackCP: exit MP");
  if (DBG()) m_pf(w, 'd', "V90Modulator: exit MP due to CP reception\r\n");
  cb_p4exit(w, MC, ML, "exitMP");
  t->event_ = 0;
}
static void m_ackcpn(W *w) {
  T *t = WT(w);
  unsigned st = WP4(w)->state_;
  if (st == 0xe) {
    CV(G_SMALL, "ackCPNot: MPNot");
    if (DBG()) m_pf(w, 'd', "V90Modulator: exit MPNot due to CPNot reception\r\n");
    cb_p4exit(w, MC, ML, "exitMPNot"); t->event_ = 0;
  } else if (st == 4) {
    CV(G_SMALL, "ackCPNot: MP");
    WMP(w)->cPack_ = 1;
    cb_mpinfo(w, ML);
    if (DBG()) m_pf(w, 'd', "V90Modulator: exit MP due to CPNot reception\r\n");
    cb_p4exit(w, MC, ML, "exitMP"); t->event_ = 0;
  } else if (st == 0xd) {
    CV(G_SMALL, "ackCPNot: delayed");
    WP4(w)->mpNotSeen_ = 1;
    if (DBG()) m_pf(w, 'd', "V90Modulator: setting delayed MPNot exit due to CPNot reception\r\n");
  } else CV(G_SMALL, "ackCPNot: other state");
}
static void m_acke(W *w) {
  T *t = WT(w);
  unsigned st = WP4(w)->state_;
  if (st == 0xe) {
    CV(G_SMALL, "ackE: MPNot");
    if (DBG()) m_pf(w, 'd', "V90Modulator: exit MPNot due to E reception\r\n");
    cb_p4exit(w, MC, ML, "exitMPNot"); t->event_ = 0;
  } else if (st == 0xd) {
    CV(G_SMALL, "ackE: delayed");
    WP4(w)->mpNotSeen_ = 1;
    if (DBG()) m_pf(w, 'd', "V90Modulator: setting delayed MPNot exit due to E reception\r\n");
  } else CV(G_SMALL, "ackE: other state");
}
/* the rate of the data phase print: (int)(0.5 + (u32)(w0 * 8000) * (1/6 f)) in extended precision */
static unsigned m_rate(W *w) {
  uint32_t v = (uint32_t)((uint32_t)WMAPPER(w)->w0_ * 8000u);
  long double r = 0.5L + (long double)v * (long double)0.16666667f;
  return (unsigned)(uint64_t)r;
}
static void m_enterdata(W *w, int G) {   /* the body of enterDataPhase / the inlined copy in progress (state_ != 3) */
  T *t = WT(w);
  m_pf(w, 'e', "V90Modulator: Data Phase spectral parameters:\r\n");
  cb_dsp(w, ML, tok_of(t->mp2_));
  t->state_ = 3; t->cnt_ = 0; t->event_ = 8;
  cb_b2sssb(w, MC, ML, t->blk_);
  if (DBG()) {
    unsigned arg = 0;
    if (t->state_ == 3) { if (G == G_SMALL) CV(G_SMALL, "enterDataPhase: rate print"); else CV(G_PROG, "progress: rate print"); arg = m_rate(w); }
    else { if (G == G_SMALL) CV(G_SMALL, "enterDataPhase: state flipped by the callee"); else CV(G_PROG, "progress: state flipped by the callee"); }
    m_pf(w, 'd', "V90Modulator: enter Data Phase, Rate = %d [bps]\r\n", (int)arg);
  }
}
static void m_edp(W *w) {
  if (WT(w)->state_ == 3) { CV(G_SMALL, "enterDataPhase: already"); return; }
  CV(G_SMALL, "enterDataPhase");
  m_enterdata(w, G_SMALL);
}
static int m_rrn(W *w) {
  T *t = WT(w);
  unsigned st;
  if (t->state_ != 3) { CV(G_SMALL, "initiateRRN: refused"); if (DBG()) m_pf(w, 'd', "V90Modulator: RRN requested but NOT approved\r\n"); return -1; }
  CV(G_SMALL, "initiateRRN");
  if (DBG()) m_pf(w, 'd', "V90Modulator: RRN requested, enter Phase 4\r\n");
  t->state_ = 2; t->cnt_ = 0;
  cb_b2sssb(w, MC, ML, 1);
  if (cb_b2snof(w, MC, ML) == 0) { CV(G_SMALL, "initiateRRN: no bits"); m_pf(w, 'e', "V90Modulator: Phase4Modulator state initialized to DataToRdModulation\r\n"); st = 0x14; }
  else { CV(G_SMALL, "initiateRRN: bits"); m_pf(w, 'e', "V90Modulator: Phase4Modulator state initialized to RdModulation\r\n"); st = 0x15; }
  m_p4reset(w, st);
  t->event_ = 0;
  cb_p4plain(w, ML, "resetBeforRRN");
  cb_p4mp(w, ML, "setRdRtSymbols", tok_of(t->mp2_));
  if (t->session_) { CV(G_SMALL, "initiateRRN: V.92 (CP)"); WCP(w)->f13_ = 0; cb_cpinfo(w, ML); }
  else { CV(G_SMALL, "initiateRRN: V.90 (MP)"); WMP(w)->cPack_ = 0; cb_mpinfo(w, ML); }
  return 0;
}
static int m_fpe(W *w) {
  T *t = WT(w);
  unsigned st;
  if (t->state_ != 3) { CV(G_SMALL, "initiateFPE: refused"); if (DBG()) m_pf(w, 'd', "V90Modulator: FPE requested but NOT approved\r\n"); return -1; }
  CV(G_SMALL, "initiateFPE");
  if (DBG()) m_pf(w, 'd', "V90Modulator: FPE requested, enter Phase 4\r\n");
  t->state_ = 2; t->cnt_ = 0;
  cb_b2sssb(w, MC, ML, 1);
  if (cb_b2snof(w, MC, ML) == 0) { CV(G_SMALL, "initiateFPE: no bits"); m_pf(w, 'e', "V90Modulator: Phase4Modulator state initialized to DataToRfModulation\r\n"); st = 0x1c; }
  else { CV(G_SMALL, "initiateFPE: bits"); m_pf(w, 'e', "V90Modulator: Phase4Modulator state initialized to RfModulation\r\n"); st = 0x1d; }
  m_p4reset(w, st);
  t->event_ = 0;
  cb_p4mp(w, ML, "setRfSymbols", tok_of(t->mp2_));
  WCP(w)->f13_ = 0;
  cb_cpinfo(w, ML);
  return 0;
}
/* progress: the blob's branch structure (jump targets 1a880 = the float conversion) */
static void m_prog(W *w, unsigned n) {
  T *t = WT(w);
  int16_t *sy = (int16_t *)w->b[B_SYM];
  float *out = (float *)w->b[B_OUT];
  unsigned *nb = (unsigned *)w->b[B_NB];
  unsigned i, eax = t->state_;
  t->cnt_ += n;
  t->event_ = 0;
  if ((int)eax <= 1 && eax != 1) {             /* state 0 or negative (the blob's jle / jne) */
    if (eax != 0) { CV(G_PROG, "progress: negative state"); if (DBG()) m_pf(w, 'd', "V90Modulator progress: Illegal state\r\n"); goto conv; }
    CV(G_PROG, "progress: state 0");
    for (i = 0; i < n; i++) sy[i] = 0;
    *nb = 0;
    goto conv;
  }
  if (eax == 1) {
    CV(G_PROG, "progress: state 1");
    i = 0;
    if (i < n) {
      for (;;) {
        if (eax == 1) {
          sy[i] = (int16_t)cb_p3gen(w, MC, ML);
          if (WP3(w)->event_ != 0) { CV(G_PROG, "progress: p3 event"); t->event_ = WP3(w)->event_; }
          if (t->event_ == 6 && t->state_ != 2) {
            CV(G_PROG, "progress: p3 -> phase 4");
            if (DBG()) m_pf(w, 'd', "V90Modulator: enter Phase 4\r\n");
            m_p4reset(w, 0);
            t->state_ = 2; t->cnt_ = 0; t->event_ = 0;
          }
        } else {
          CV(G_PROG, "progress: state 1, p4 symbol");
          sy[i] = (int16_t)cb_p4gen(w, MC, ML);
          if (WP4(w)->event_ != 0) t->event_ = WP4(w)->event_;
        }
        i++;
        if (i >= n) break;
        eax = t->state_;
      }
    }
    *nb = 0;
    goto conv;
  }
  if (eax == 2) {
    CV(G_PROG, "progress: state 2");
    for (i = 0; i < n; i++) {
      sy[i] = (int16_t)cb_p4gen(w, MC, ML);
      if (WP4(w)->event_ != 0) t->event_ = WP4(w)->event_;
    }
    if (t->event_ != 7) { CV(G_PROG, "progress: state 2, no event 7"); *nb = 0; goto conv; }
    if (t->state_ != 3) { CV(G_PROG, "progress: state 2 -> data phase"); m_enterdata(w, G_PROG); }
    else CV(G_PROG, "progress: state 2, already data");
    *nb = cb_b2snof(w, MC, ML);
    goto conv;
  }
  if (eax == 3) {
    CV(G_PROG, "progress: state 3");
    cb_scrproc(w, MC, ML, B_IN, B_BIT, *nb);
    cb_b2sproc(w, MC, ML, B_BIT, *nb, B_SYM);
    if (WPAR(w)->debugDigitalModemInitiateRrn != 0 && t->cnt_ > (unsigned)WPAR(w)->debugDigitalModemInitiateRrnTime) {
      CV(G_PROG, "progress: RRN requested by the debug parameter");
      m_rrn(w);
      t->event_ = 9;
    }
    goto conv;
  }
  CV(G_PROG, "progress: illegal state");
  if (DBG()) m_pf(w, 'd', "V90Modulator progress: Illegal state\r\n");
conv:
  for (i = 0; i < n; i++) out[i] = (float)sy[i];
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static void gen_script(void) {
  unsigned i;
  static const int16_t syms[] = { 0, 1, -1, 100, -100, 32767, -32768, 4000, -4000, 0x7f, 0x1234, 255, -255 };
  static const uint32_t evs[] = { 0, 0, 0, 6, 7, 8, 1, 9, 0xffffffffu };
  for (i = 0; i < NSC; i++) {
    uint32_t lo = h_rpct(60) ? (uint16_t)syms[h_rmod(sizeof syms / sizeof syms[0])] : (h_rnd() & 0xffffu);
    g_sc.sym3[i] = (h_rnd() & 0xffff0000u) | lo;
    g_sc.ev3[i] = h_rpct(65) ? 0u : evs[h_rmod(sizeof evs / sizeof evs[0])];
    g_sc.gst3on[i] = h_rpct(20) ? 1u : 0u; g_sc.gst3[i] = h_rmod(16);
    lo = h_rpct(60) ? (uint16_t)syms[h_rmod(sizeof syms / sizeof syms[0])] : (h_rnd() & 0xffffu);
    g_sc.sym4[i] = (h_rnd() & 0xffff0000u) | lo;
    g_sc.ev4[i] = h_rpct(60) ? 0u : evs[h_rmod(sizeof evs / sizeof evs[0])];
    g_sc.gst4on[i] = h_rpct(20) ? 1u : 0u; g_sc.gst4[i] = h_rmod(32);
    g_sc.nof[i] = h_rpct(40) ? 0u : h_rnd();
    g_sc.ssb[i] = h_rnd();
    g_sc.flip[i] = h_rpct(20) ? 1u : 0u; g_sc.flipv[i] = h_rpct(50) ? 3u : h_rmod(6);
    g_sc.pm[i] = h_rnd(); g_sc.psym[i] = h_rnd(); g_sc.pnew[i] = h_rpct(30) ? 0u : h_rnd() % 400u; g_sc.pret[i] = h_rnd();
    g_sc.sbits[i] = h_rnd();
    g_sc.f3on[i] = h_rpct(8) ? 1u : 0u; g_sc.f3v[i] = h_rpct(60) ? 2u : h_rmod(5);
    g_sc.f4on[i] = h_rpct(8) ? 1u : 0u; g_sc.f4v[i] = h_rpct(60) ? 3u : h_rmod(5);
    g_sc.nx[i] = h_rpct(40) ? 4u : h_rpct(30) ? 0xdu : h_rpct(30) ? 0xeu : h_rmod(32);
    g_sc.x3s[i] = h_rmod(16); g_sc.x3e[i] = h_rpct(50) ? 6u : h_rpct(40) ? 0u : h_rnd() % 12u;
    g_sc.x4s[i] = h_rmod(32); g_sc.x4e[i] = h_rpct(50) ? 0u : h_rnd() % 12u;
  }
}
/* random garbage in every non-pointer member of the object (member-wise by the field table: arch independent) */
static void fill_obj(T *t) {
  int i;
  for (i = 0; i < NF_ModV90Obj; i++) {
    const HFld *f = &f_ModV90Obj[i];
    unsigned n = f->count ? f->count : 1, es, tot, j;
    uint8_t *q = (uint8_t *)t + f->off;
    switch (f->kind) {
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'u': case 'i': case 'f': es = 4; break;
      case 'p': case 'o': { void *v = NULL; uintptr_t r = h_rnd(); r = (r << 4) ^ (uintptr_t)h_rnd(); v = (void *)((r & 0x3ffffff0u) | 1u); memcpy(q, &v, sizeof v); continue; }
      default: continue;
    }
    tot = n * es;
    for (j = 0; j < tot; j += 4) { uint32_t r = h_rnd(); memcpy(q + j, &r, tot - j < 4 ? tot - j : 4); }
  }
}
static void reg(int id, void *p, size_t n) { g_iw.b[id] = p; g_blkidx[id] = blk_add(bname[id], p, n); g_obj[id] = p; g_idname[id] = bname[id]; }
static unsigned pick_blk(void) {
  for (;;) {
    unsigned b = rnd_range(1, 40);
    unsigned s2 = b * 2, s8 = b * 8;
    if (s2 == SZ_B2S || s8 == SZ_B2S || s2 == SZ_P3 || s8 == SZ_P3 || s8 == SZ_P4) continue;
    /* the log maps the pointer-holding classes' sizes; keep the buffers away from them (i386 values 0x24, 0x398, 0x2fac and the x86-64 ones) */
    if (s2 == 0x24 || s8 == 0x24 || s8 == 0x30 || s2 == 0x30) continue;
    return b;
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
  g_blk[g_blkidx[B_MOD]].n = sizeof(T);   /* the canary is checked separately */
  reg(B_INFO, opaque_new(0x24), 0x24);
  reg(B_JD, opaque_new(0x40), 0x40);
  reg(B_V92JD, opaque_new(0x40), 0x40);
  reg(B_DIL, opaque_new(0x40), 0x40);
  reg(B_ADDCP, opaque_new(0x40), 0x40);
  reg(B_MP1, opaque_new(sizeof(MapPar90)), sizeof(MapPar90));
  reg(B_MP2, opaque_new(sizeof(MapPar90)), sizeof(MapPar90));
  reg(B_MP, opaque_new(sizeof(MpObj)), sizeof(MpObj));
  reg(B_CP, opaque_new(sizeof(CpObj)), sizeof(CpObj));
  reg(B_PAR, opaque_new(sizeof(V90ParObj)), sizeof(V90ParObj));
  reg(B_IN, opaque_new(8 * 40 * 4), 8 * 40 * 4);
  reg(B_OUT, opaque_new(41 * 4), 41 * 4);
  reg(B_NB, opaque_new(16), 16);
  /* member-wise setup of the foreign objects the functions read */
  { uint32_t v = h_rpct(30) ? (uint32_t)h_rmod(2) : h_rnd(); memcpy(WINFO(&g_iw), &v, 4); v = h_rnd(); memcpy(WINFO(&g_iw) + 4, &v, 4); WINFO(&g_iw)[8] = (uint8_t)h_rnd(); }
  WMP1(&g_iw)->w0_ = h_rnd(); ((MapPar90 *)g_iw.b[B_MP2])->w0_ = h_rnd();
  WMP(&g_iw)->cPack_ = (uint8_t)h_rnd(); WMP(&g_iw)->trn2dD_ = h_rnd();
  WCP(&g_iw)->f13_ = (uint8_t)h_rnd(); WCP(&g_iw)->trn2dD_ = h_rnd();
  WPAR(&g_iw)->debugDigitalModemInitiateRrn = (int32_t)(h_rpct(50) ? 0u : h_rpct(70) ? 1u : h_rnd());
  WPAR(&g_iw)->debugDigitalModemInitiateRrnTime = (int32_t)(h_rpct(60) ? rnd_range(0, 200) : h_rnd());
  if (with_subs) {
    unsigned blk = pick_blk();
    g_blkn = blk;
    t->blk_ = blk;
    reg(B_SYM, h_obj_new(blk * 2 + 8), blk * 2 + 8);   /* bytes beyond 2 * blk are never written: a guard */
    reg(B_BIT, h_obj_new(blk * 8 + 8), blk * 8 + 8);
    reg(B_P3, opaque_new(sizeof(Ph3V90Obj)), sizeof(Ph3V90Obj));
    reg(B_P4, opaque_new(sizeof(Ph4Obj)), sizeof(Ph4Obj));
    reg(B_B2S, opaque_new(sizeof(B2S90)), sizeof(B2S90));
    reg(B_MAPPER, opaque_new(sizeof(MapObj)), sizeof(MapObj));
    t->info_ = g_iw.b[B_INFO]; t->jd_ = g_iw.b[B_JD]; t->v92jd_ = g_iw.b[B_V92JD]; t->dil_ = g_iw.b[B_DIL]; t->mp1_ = g_iw.b[B_MP1]; t->mp2_ = g_iw.b[B_MP2]; t->addCp_ = g_iw.b[B_ADDCP];
    t->mp_ = g_iw.b[B_MP]; t->cp_ = g_iw.b[B_CP]; t->params_ = g_iw.b[B_PAR]; t->p3_ = g_iw.b[B_P3]; t->p4_ = g_iw.b[B_P4]; t->b2s_ = g_iw.b[B_B2S];
    t->symBuf_ = g_iw.b[B_SYM]; t->bitBuf_ = g_iw.b[B_BIT];
    ((B2S90 *)g_iw.b[B_B2S])->mapper_ = g_iw.b[B_MAPPER];
    WMAPPER(&g_iw)->w0_ = h_rpct(50) ? (uint32_t)rnd_range(1, 6) : h_rnd();
    { unsigned i; int16_t *sy = (int16_t *)g_iw.b[B_SYM]; for (i = 0; i < blk + 4; i++) sy[i] = (int16_t)h_rnd(); }
  }
}
static void pick_subs(T *t) {
  static const unsigned s3[] = { 0, 3, 5, 9, 9, 3, 5, 1, 2, 15 };
  static const unsigned s4[] = { 0, 0, 4, 4, 0xd, 0xd, 0xe, 0xe, 3, 7, 0x12 };
  Ph3V90Obj *p3 = WP3(&g_iw);
  Ph4Obj *p4 = WP4(&g_iw);
  p3->state_ = h_rpct(85) ? s3[h_rmod(sizeof s3 / sizeof s3[0])] : h_rnd();
  p3->event_ = h_rpct(50) ? 6u : h_rpct(50) ? 0u : h_rnd();
  p4->state_ = h_rpct(85) ? s4[h_rmod(sizeof s4 / sizeof s4[0])] : h_rnd();
  p4->event_ = h_rpct(50) ? 0u : h_rpct(50) ? 7u : h_rnd();
  p4->mpNotSeen_ = (uint8_t)h_rnd();
  p3->flag_ = h_rnd(); p4->flag_ = h_rnd();
  t->session_ = pick_flag();
  t->state_ = h_rpct(80) ? h_rmod(5) : h_rpct(50) ? 0xffffffffu : h_rnd();
  t->cnt_ = h_rpct(50) ? h_rmod(300) : h_rpct(30) ? 0xfffffff0u + h_rmod(16) : h_rnd();
  t->event_ = h_rnd();
}
static unsigned long g_ncalls;
static int g_showall;
static const char *g_what = "setup";
static int g_skip[NB];
static void emit_state(const char *tag) {
  const T *t = WT(&g_iw);
  char s[600];
  obj_snapshot(s, sizeof s, 0);
  h_emit(g_show, "%s sess=%u blk=%u %s", tag, (unsigned)t->session_, (unsigned)t->blk_, s);
}
static void world_model(void) { int k; for (k = 0; k < NB; k++) g_mw.b[k] = g_iw.b[k] ? g_blk[g_blkidx[k]].m : NULL; }

/* one call: the impl first (its heap blocks are known to the model afterwards), the model on the copies, then everything is compared.
 * op: 0 C1  1 C2  2 D1  3 D2  4 setSessionFlag  5 reset  6 enterPhase3  7 enterPhase4  8 exitJd  9 exitJdPhase  10 exitDIL  11 exitRi  12 ackCP  13 ackCPNot  14 ackE  15 enterDataPhase  16 initiateRRN
 *     17 initiateFPE  18 progress */
typedef struct { int op; CtorArgs ca; unsigned u; unsigned n; } Call;
static const char *const opname[] = { "C1", "C2", "D1", "D2", "setSessionFlag", "reset", "enterPhase3", "enterPhase4", "exitJd", "exitJdPhase", "exitDIL", "exitRi", "ackCP", "ackCPNot", "ackE",
                                      "enterDataPhase", "initiateRRN", "initiateFPE", "progress" };
static void reg_after(int id, void *p, size_t n) { g_iw.b[id] = p; g_blkidx[id] = blk_add(bname[id], p, n); g_idname[id] = bname[id]; }
static void do_call(Call *c) {
  int mr = 0, ir = 0, cmpres = 0;
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
  h_emit(g_show, "call %s u=%u n=%u lvl=%u", opname[c->op], c->u, c->n, dsplibs_debug_level);
  /* ---- the impl */
  switch (c->op) {
    case 0: f_c1(t, c->ca.blk, c->ca.a[0], c->ca.a[1], c->ca.a[2], c->ca.a[3], c->ca.a[4], c->ca.a[5], c->ca.a[6], c->ca.a[7], c->ca.a[8], c->ca.a[9], c->ca.session); break;
    case 1: f_c2(t, c->ca.blk, c->ca.a[0], c->ca.a[1], c->ca.a[2], c->ca.a[3], c->ca.a[4], c->ca.a[5], c->ca.a[6], c->ca.a[7], c->ca.a[8], c->ca.a[9], c->ca.session); break;
    case 2: f_d1(t); break;
    case 3: f_d2(t); break;
    case 4: f_ssf(t, c->u); break;
    case 5: f_reset(t); break;
    case 6: f_ep3(t); break;
    case 7: f_ep4(t); break;
    case 8: f_ejd(t); break;
    case 9: f_ejdp(t); break;
    case 10: f_edil(t); break;
    case 11: f_eri(t); break;
    case 12: f_ackcp(t); break;
    case 13: f_ackcpn(t); break;
    case 14: f_acke(t); break;
    case 15: f_edp(t); break;
    case 16: ir = f_rrn(t); cmpres = 1; break;
    case 17: ir = f_fpe(t); cmpres = 1; break;
    case 18: f_prog(t, (int *)g_iw.b[B_IN], (unsigned *)g_iw.b[B_NB], (float *)g_iw.b[B_OUT], c->n); break;
  }
  __asm__ volatile("" ::: "memory");
  { for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t); }
  if (c->op <= 1) {   /* the constructor: the heap blocks it allocated, for the model and the later calls */
    c->ca.sym = t->symBuf_; c->ca.bit = t->bitBuf_; c->ca.b2s = t->b2s_; c->ca.p3 = t->p3_; c->ca.p4 = t->p4_;
    if (!c->ca.sym || !c->ca.bit || !g_iw.b[B_B2S] || !g_iw.b[B_P3] || !g_iw.b[B_P4]) h_die3("the constructor did not allocate its blocks");
    if (c->ca.b2s != g_iw.b[B_B2S] || c->ca.p3 != g_iw.b[B_P3] || c->ca.p4 != g_iw.b[B_P4]) h_die3("the constructor's sub-objects are not the ones the recorders saw");
    reg_after(B_SYM, c->ca.sym, ((HAl *)h_find_al(c->ca.sym, 0))->size);
    reg_after(B_BIT, c->ca.bit, ((HAl *)h_find_al(c->ca.bit, 0))->size);
    reg_after(B_B2S, c->ca.b2s, sizeof(B2S90));
    reg_after(B_P3, c->ca.p3, sizeof(Ph3V90Obj));
    reg_after(B_P4, c->ca.p4, sizeof(Ph4Obj));
    world_model();
  }
  /* ---- the model */
  switch (c->op) {
    case 0: case 1: m_ctor(MW, &c->ca); break;
    case 2: case 3: m_dtor(MW); break;
    case 4: m_ssf(MW, c->u); break;
    case 5: m_reset(MW); break;
    case 6: m_ep3(MW); break;
    case 7: m_ep4(MW); break;
    case 8: m_ejd(MW); break;
    case 9: m_ejdp(MW); break;
    case 10: m_edil(MW); break;
    case 11: m_eri(MW); break;
    case 12: m_ackcp(MW); break;
    case 13: m_ackcpn(MW); break;
    case 14: m_acke(MW); break;
    case 15: m_edp(MW); break;
    case 16: mr = m_rrn(MW); break;
    case 17: mr = m_fpe(MW); break;
    case 18: m_prog(MW, c->n); break;
  }
  if (cmpres && mr != ir) h_die3("%s: result: model %d, impl %d", g_what, mr, ir);
  if (cmpres) h_emit(g_show, "ret %d", ir);
  log_cmp(&g_ml, &g_il, g_what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the callee cursors differ", g_what);
  { int k; size_t keep[NB]; for (k = 0; k < NB; k++) if (g_iw.b[k] && g_skip[k]) { keep[k] = g_blk[g_blkidx[k]].n; g_blk[g_blkidx[k]].n = 0; }
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
  void *known[10] = { g_iw.b[B_INFO], g_iw.b[B_JD], g_iw.b[B_V92JD], g_iw.b[B_DIL], g_iw.b[B_MP1], g_iw.b[B_MP2], g_iw.b[B_ADDCP], g_iw.b[B_CP], g_iw.b[B_MP], g_iw.b[B_PAR] };
  int k;
  for (k = 0; k < 10; k++) {
    c->ca.a[k] = known[k];
    if (h_rpct(6)) c->ca.a[k] = NULL;
    else if (h_rpct(5)) c->ca.a[k] = known[h_rmod(10)];
    else if (h_rpct(3)) { uintptr_t r = h_rnd(); c->ca.a[k] = (void *)((r & 0x3ffffff0u) | 1u); }
  }
  c->ca.blk = pick_blk();
  c->ca.session = pick_flag();
  g_blkn = c->ca.blk;
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
  if (sc == 3 && h_rpct(30)) ncalls = rnd_range(2, 3);
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
          /* a constructed object: sometimes with some of its heap pointers NULL (the destructor tests each) */
          c.op = 2 + (int)h_rmod(2);
          if (h_rpct(25)) { sysdep_free(t->p3_); t->p3_ = NULL; }
          if (h_rpct(25)) { sysdep_free(t->p4_); t->p4_ = NULL; }
          if (h_rpct(25)) { sysdep_free(t->b2s_); t->b2s_ = NULL; }
          if (h_rpct(25)) { sysdep_free(t->symBuf_); t->symBuf_ = NULL; }
          if (h_rpct(25)) { sysdep_free(t->bitBuf_); t->bitBuf_ = NULL; }
          g_skip[B_SYM] = g_skip[B_BIT] = g_skip[B_P3] = g_skip[B_P4] = g_skip[B_B2S] = 1;   /* freed by the destructor: poisoned */
        }
        break;
      case 2: {
        static const int ops[] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 10, 11, 13, 14, 16, 17, 6, 7 };
        c.op = ops[h_rmod(sizeof ops / sizeof ops[0])];
        c.u = pick_flag();
        pick_subs(t);
        if (h_rpct(60)) {   /* the data phase is the interesting state of the RRN / FPE requests and enterDataPhase */
          if (c.op == 16 || c.op == 17 || c.op == 15) t->state_ = h_rpct(75) ? 3u : h_rmod(5);
          if (c.op == 6) t->state_ = h_rpct(50) ? 1u : h_rmod(5);
          if (c.op == 7) t->state_ = h_rpct(50) ? 2u : h_rmod(5);
        }
        break;
      }
      default: {
        unsigned i, nb;
        c.op = 18;
        pick_subs(t);
        t->state_ = h_rpct(85) ? h_rmod(4) : h_rpct(50) ? h_rnd() : 4u + h_rmod(3);
        c.n = h_rpct(10) ? 0u : rnd_range(1, g_blkn);
        nb = h_rpct(20) ? 0u : rnd_range(0, 8 * g_blkn);
        memcpy(g_iw.b[B_NB], &nb, 4);
        for (i = 0; i < 8 * 40; i++) ((int *)g_iw.b[B_IN])[i] = (int)h_rnd();
        for (i = 0; i < 41; i++) ((uint32_t *)g_iw.b[B_OUT])[i] = h_rnd();
        if (h_rpct(30)) WPAR(&g_iw)->debugDigitalModemInitiateRrnTime = (int32_t)(t->cnt_ - rnd_range(0, 3) + c.n - 1);
        break;
      }
    }
    if (h_rpct(10)) dsplibs_debug_level = pick_level();
    do_call(&c);
    if (sc == 1 && k == 0 && c.ca.blk) g_blkn = c.ca.blk;
  }
  h_trial_end("p39a");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("P39A_SEED");
  CovSite *c;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P39A_ALL")) g_showall = 1;
  p07_layout_check(); p09_layout_check(); p10_layout_check(); p11_layout_check(); p27_layout_check(); p28_layout_check(); p29_layout_check(); p30_layout_check(); p39_layout_check();
  h_x87_check();
  h_arena_init();
  h_log_map_add(sizeof(B2S90), 0x24); h_log_map_add(sizeof(Ph3V90Obj), 0x398); h_log_map_add(sizeof(Ph4Obj), 0x2fac);
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P39A_TRIALS") ? atoi(getenv("P39A_TRIALS")) : (sc == 3 ? 40000 : sc == 2 ? 40000 : 15000);
  if (sc < 0 || sc > 3) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (c = __start_covsites; c < __stop_covsites; c++) {
    if (c->grp != sc) continue;
    h_emit(1, "COV %s %lu", c->name, c->n);
    if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P39A_NOCOV")) ? 2 : 0;
}
