/*
 * Differential test harness for port chunk P45 (src/vpcm/pcm_modem.cpp): V90Modem (C1 / C2, D1 / D2, printTitle, reset, setSessionFlag, progress) and V92Modem (C1 / C2, D1 / D2, printTitle, reset, progress).
 * Derived from test/vpcm/v90_modulator.a.main.c (recorders + independent model + per-record snapshot) and test/vpcm/v90_demodulator.main.main.c (constructor / destructor checks against the allocator table).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/pcm_modem.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2: the port
 * object alone) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/pcm_modem.blob32.out).  The functions are reached through __asm__ labels; the objects are the generated C mirrors
 * (test/vpcm/pcm_modem.layout: ModemV90Obj / ModemV92Obj; V90Parameters of p07 for probingMode); every other object is an opaque block.
 *
 * Every callee is a RECORDER (void, no scripted result): V90MP / V90CP / V90Parameters / V90Phase2Info / V90Jd / V92Jd / V90Modulator / V90Demodulator constructors and destructors, the modulator / demodulator
 * setSessionFlag / reset / progress, setDilDescriptor, and on the V.92 side V92Parameters / V92Phase2Info / V92CP / V92Modulator, the four V92create* / V92delete*, V92DILdescriptorPacker, edprintf and
 * dsplibs_debug_printf.  After recording, EVERY recorder may POKE the modem object (script): side_, session_, dil_, mod_, dem_, params_, the +0x54 word of the modem parameter block, the debug level, so the
 * time at which the blob / port reloads a member is observable (the record carries the arguments read from the members and a hash of the whole object at that moment).  What is compared: every record (callee,
 * arguments, snapshot), the whole object (field table, after every call), every foreign block (copies, byte-wise), the canary behind the object, the allocation table (sizes and order of the constructor's blocks, the
 * freed flags after a destructor), the canonical output (golden).  The INDEPENDENT MODEL (m90_* / m92_*: written from porting/decompiled/dp/p45_modem.txt in the blob's control-flow shape, not from the port source) predicts
 * the final object and every record; a violation exits 3.
 *
 * Scenarios (own process each): 0 V90 constructor, 1 V90 destructor (after a constructor), 2 V90 printTitle / reset / setSessionFlag / progress, 3 V92 constructor, 4 V92 destructor, 5 V92 printTitle / reset /
 * progress, 6 V90 lifecycles (constructor, methods, destructor), 7 V92 lifecycles.
 */

#include "p39_common.h"
#include "pcm_parameters_layout.h"
#include "pcm_modem_layout.h"

extern const unsigned p45_sizes[];
static const unsigned i386_sz90[] = { 0x558, 0x24, 0x90, 0xdc, 0x70, 0x298 };
static const unsigned i386_sz92[] = { 0xdc, 0x2c, 0x918, 0xb4, 0x90 };

/* ---- the functions under test ---- */
extern void f90_c1(void *, unsigned, void *, void *, unsigned, unsigned, unsigned) L("_ZN8V90ModemC1E12V90ModemSideP19_tagModemParametersP19tagV90DILdescriptorj20V90ComputationalModej");
extern void f90_c2(void *, unsigned, void *, void *, unsigned, unsigned, unsigned) L("_ZN8V90ModemC2E12V90ModemSideP19_tagModemParametersP19tagV90DILdescriptorj20V90ComputationalModej");
extern void f90_d1(void *) L("_ZN8V90ModemD1Ev");
extern void f90_d2(void *) L("_ZN8V90ModemD2Ev");
extern void f90_title(void *) L("_ZN8V90Modem10printTitleEv");
extern void f90_reset(void *, unsigned) L("_ZN8V90Modem5resetEj");
extern void f90_ssf(void *, unsigned) L("_ZN8V90Modem14setSessionFlagEj");
extern void f90_prog(void *, int *, unsigned *, float *, unsigned) L("_ZN8V90Modem8progressEPiRjPfj");
extern void f92_c1(void *, unsigned, void *, unsigned, void *, unsigned) L("_ZN8V92ModemC1E12V92ModemSideP19_tagModemParametersjP19tagV90DILdescriptor20V92ComputationalMode");
extern void f92_c2(void *, unsigned, void *, unsigned, void *, unsigned) L("_ZN8V92ModemC2E12V92ModemSideP19_tagModemParametersjP19tagV90DILdescriptor20V92ComputationalMode");
extern void f92_d1(void *) L("_ZN8V92ModemD1Ev");
extern void f92_d2(void *) L("_ZN8V92ModemD2Ev");
extern void f92_title(void *) L("_ZN8V92Modem10printTitleEv");
extern void f92_reset(void *) L("_ZN8V92Modem5resetEv");
extern void f92_prog(void *, int *, unsigned *, float *, unsigned) L("_ZN8V92Modem8progressEPiRjPfj");

/* ---- the interposed callees ---- */
extern void c_mpctor(void *) L("_ZN5V90MPC1Ev");
extern void c_mpdtor(void *) L("_ZN5V90MPD1Ev");
extern void c_cpctor(void *) L("_ZN5V90CPC1Ev");
extern void c_cpdtor(void *) L("_ZN5V90CPD1Ev");
extern void c_parctor(void *, void *) L("_ZN13V90ParametersC1EP19_tagModemParameters");
extern void c_pardtor(void *) L("_ZN13V90ParametersD1Ev");
extern void c_infoctor(void *, void *) L("_ZN13V90Phase2InfoC1EP13V90Parameters");
extern void c_jdctor(void *, void *) L("_ZN5V90JdC1EP13V90Parameters");
extern void c_jddtor(void *) L("_ZN5V90JdD1Ev");
extern void c_jd92ctor(void *, void *) L("_ZN5V92JdC1EP13V90Parameters");
extern void c_jd92dtor(void *) L("_ZN5V92JdD1Ev");
extern void c_modctor(void *, unsigned, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, unsigned)
    L("_ZN12V90ModulatorC1EjP13V90Phase2InfoP5V90JdP5V92JdP19tagV90DILdescriptorP16V90MappingParamsS9_P22tagV90AdditionalCPinfoP5V90CPP5V90MPP13V90Parametersj");
extern void c_moddtor(void *) L("_ZN12V90ModulatorD1Ev");
extern void c_modssf(void *, unsigned) L("_ZN12V90Modulator14setSessionFlagEj");
extern void c_modreset(void *) L("_ZN12V90Modulator5resetEv");
extern void c_modprog(void *, int *, unsigned *, float *, unsigned) L("_ZN12V90Modulator8progressEPiRjPfj");
extern void c_demctor(void *, unsigned, void *, void *, void *, void *, void *, void *, void *, void *, void *, unsigned, void *, unsigned, unsigned)
    L("_ZN14V90DemodulatorC1EjP13V90Phase2InfoP5V90JdP5V92JdP19tagV90DILdescriptorP16V90MappingParamsS9_P22tagV90AdditionalCPinfoP5V90CPP5V90MP23__tHardwareCodecTypes__P13V90Parameters20V90ComputationalModej");
extern void c_demdtor(void *) L("_ZN14V90DemodulatorD1Ev");
extern void c_demssf(void *, unsigned) L("_ZN14V90Demodulator14setSessionFlagEj");
extern void c_demreset(void *, unsigned) L("_ZN14V90Demodulator5resetEj");
extern void c_demprog(void *, int *, unsigned *, float *, unsigned) L("_ZN14V90Demodulator8progressEPiRjPfj");
extern void c_setdil(void *, unsigned) L("_Z16setDilDescriptorP19tagV90DILdescriptor7DilType");
extern void c_par92ctor(void *, void *) L("_ZN13V92ParametersC1EP19_tagModemParameters");
extern void c_par92dtor(void *) L("_ZN13V92ParametersD1Ev");
extern void c_info92ctor(void *, void *) L("_ZN13V92Phase2InfoC1EP13V92Parameters");
extern void c_cp92ctor(void *) L("_ZN5V92CPC1Ev");
extern void c_cp92dtor(void *) L("_ZN5V92CPD1Ev");
extern void c_mkconst(void *) L("V92createConstellations");
extern void c_mkfilt(void *) L("V92createFilterCoefficients");
extern void c_rmconst(void *) L("V92deleteConstellations");
extern void c_rmfilt(void *) L("V92deleteFilterCoefficients");
extern void c_mod92ctor(void *, unsigned, void *, void *, void *, void *, void *, void *) L("_ZN12V92ModulatorC1EjP13V92Phase2InfoP5V92JaP19tagV90DILdescriptorP5V92CPP16V92MappingParamsP13V92Parameters");
extern void c_mod92dtor(void *) L("_ZN12V92ModulatorD1Ev");
extern void c_mod92prog(void *, int *, unsigned *, float *, unsigned) L("_ZN12V92Modulator8progressEPiRjPfj");
extern void c_mod92reset(void *) L("_ZN12V92Modulator5resetEv");
extern void c_mod92ep3(void *) L("_ZN12V92Modulator11enterPhase3Ev");
extern void c_pack92(void *, unsigned char *, int *) L("_Z22V92DILdescriptorPackerP19tagV90DILdescriptorPhPi");

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_OBJ, B_MPAR, B_DIL, B_PAR, B_PAR2, B_INFO, B_JD, B_JD92, B_MOD, B_DEM, B_CP, B_MAP, B_IN, B_NB, B_OUT, NBLK };
enum { X_MP1 = NBLK, X_MP2, X_ADDCP, X_MPV, X_CPV, X_JA, X_JABITS, X_JALEN, NID };
static const char *const bname[NID] = { "obj", "mpar", "dil", "par", "par2", "info", "jd", "jd92", "mod", "dem", "cp", "map", "in", "nb", "out",
                                        "obj.mp1_", "obj.mp2_", "obj.addCp_", "obj.mp_", "obj.cp_", "obj.ja_", "obj.ja_.bits_", "obj.ja_.len_" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
static int g_kind;                    /* 90 or 92: the class of the object under test */
static size_t g_osz;
#define O90(w) ((ModemV90Obj *)(w)->b[B_OBJ])
#define O92(w) ((ModemV92Obj *)(w)->b[B_OBJ])

enum { G_V90C = 0, G_V90D, G_V90M, G_V92C, G_V92D, G_V92M };
#define DBG() (dsplibs_debug_level > 1)

/* ------------------------------------------------------------------------------------------------ the poke script */
typedef struct { uint32_t kind[NSC], val[NSC]; } Script;
typedef struct { unsigned pk; } Cur;
static Script g_sc;
static Cur g_ic, g_mc;
static Log g_il, g_ml;
static int g_show, g_showall;
static unsigned long g_symctr, g_ncalls;
static unsigned g_pokemask;           /* bit k: poke kind k allowed */
static const char *g_what = "setup";
static int g_freector;
/* sysdep_malloc wrapper (the shared harness library is compiled with -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free): the constructor's blocks are logged with their i386 size by
 * ALLOCATION ORDER (V92Parameters and V92MappingParams have the same size on x86-64 but not on i386: the size-keyed h_log_map_add cannot tell them apart) */
void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
static unsigned g_seq;
void *sysdep_malloc(unsigned size) {
  const unsigned *tab = g_kind == 90 ? i386_sz90 : i386_sz92;
  unsigned n = g_kind == 90 ? 6 : 5;
  if (g_seq < n) h_log_next_size = tab[g_seq];
  if (g_kind == 90 && g_seq == 4 && size == p45_sizes[5]) h_log_next_size = tab[5];
  g_seq++;
  return h_sysdep_malloc(size);
}
void sysdep_free(void *q) { h_sysdep_free(q); }

static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) {
    g_sc.kind[i] = h_rpct(80) ? 0u : 1u + h_rmod(10);
    g_sc.val[i] = h_rnd();
  }
}
static uint32_t pick_side(uint32_t v) {
  static const uint32_t s[] = { 0, 1, 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 5, 1 };
  return s[v % (sizeof s / sizeof s[0])];
}
static void *cand(unsigned v, int blk) { return v % 3 == 0 ? NULL : v % 3 == 1 ? (void *)g_iw.b[blk] : (void *)g_iw.b[B_PAR2]; }
/* the pokes the recorders apply AFTER recording (the same function runs in the impl's and the model's world) */
static void poke(W *w, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  if (kind == 10) {
    static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu };
    dsplibs_debug_level = lvs[(v >> 2) % (sizeof lvs / sizeof lvs[0])];
    return;
  }
  if (g_kind == 90) {
    ModemV90Obj *t = O90(w);
    switch (kind) {
      case 1: t->side_ = pick_side(v >> 3); break;
      case 2: t->session_ = v; break;
      case 3: t->dil_ = v & 4 ? g_iw.b[B_DIL] : cand(v >> 3, B_DIL); break;
      case 4: t->mod_ = cand(v >> 3, B_MOD); break;
      case 5: t->dem_ = cand(v >> 3, B_DEM); break;
      case 6: t->params_ = (v >> 3) & 1 ? g_iw.b[B_PAR] : g_iw.b[B_PAR2]; break;
      case 7: memcpy(w->b[B_MPAR] + 0x54, &v, 4); break;
      case 8: t->info_ = cand(v >> 3, B_INFO); t->jd_ = cand(v >> 5, B_JD); t->jd92_ = cand(v >> 7, B_JD92); break;
      default: break;
    }
  } else {
    ModemV92Obj *t = O92(w);
    switch (kind) {
      case 1: t->side_ = pick_side(v >> 3); break;
      case 3: t->dil_ = v & 4 ? g_iw.b[B_DIL] : cand(v >> 3, B_DIL); break;
      case 4: t->mod_ = cand(v >> 3, B_MOD); break;
      case 6: t->params_ = (v >> 3) & 1 ? g_iw.b[B_PAR] : g_iw.b[B_PAR2]; break;
      case 7: memcpy(w->b[B_MPAR] + 0x54, &v, 4); break;
      case 8: t->info_ = cand(v >> 3, B_INFO); t->cp_ = cand(v >> 5, B_CP); break;
      case 9: t->mapping_ = cand(v >> 3, B_MAP); break;
      default: break;
    }
  }
}

/* ------------------------------------------------------------------------------------------------ the snapshot of the object at the moment of a record */
static void obj_snapshot(char *out, size_t cap, int model) {
  const W *w = model ? &g_mw : &g_iw;
  const HFld *fl = g_kind == 90 ? f_ModemV90Obj : f_ModemV92Obj;
  int nf = g_kind == 90 ? NF_ModemV90Obj : NF_ModemV92Obj, i;
  const uint8_t *t = w->b[B_OBJ];
  uint32_t h = 2166136261u;
  for (i = 0; i < nf; i++) {
    const HFld *f = &fl[i];
    unsigned n = f->count ? f->count : 1, es, j;
    const uint8_t *q = t + f->off;
    switch (f->kind) {
      case 'p': case 'o':
        for (j = 0; j < n; j++) { void *v; int id; memcpy(&v, q + j * sizeof v, sizeof v); id = tok_of(v); h = fnv_upd(h, &id, sizeof id); }
        continue;
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      default: es = 4; break;
    }
    h = fnv_upd(h, q, (size_t)n * es);
  }
  snprintf(out, cap, "|h=%08x lvl=%u", h, dsplibs_debug_level);
}

/* ------------------------------------------------------------------------------------------------ callee semantics: record, then poke */
#define REC(w, c, lg, ...) do { rec_add((lg), (w) == &g_mw, 'c', __VA_ARGS__); poke((w), (c)); } while (0)
static void cb_ed(W *w, Cur *c, Log *l, char kind, const char *s) { rec_add(l, w == &g_mw, kind, "%s", s); poke(w, c); }
#define IW (&g_iw)
#define IL (&g_il)
#define IC (&g_ic)
#define MW (&g_mw)
#define ML (&g_ml)
#define MC (&g_mc)
/* idn() of p39_common.h rotates through 8 buffers: a record with more heap tokens would overwrite its own earlier ones */
static const char *nm45(int id) {
  static char b[32][24];
  static int n;
  char *o = b[n++ & 31];
  if (id >= 0 && id < g_nobj) return g_idname[id];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  snprintf(o, 24, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}
#define T_(p) nm45(tok_of(p))
/* the impl's recorders (the text format is shared with the model's calls below) */
#define F_MPCTOR "V90MP.ctor(this=%s)"
#define F_MPDTOR "V90MP.dtor(this=%s)"
#define F_CPCTOR "V90CP.ctor(this=%s)"
#define F_CPDTOR "V90CP.dtor(this=%s)"
#define F_PARCTOR "par.ctor(this=%s p=%s)"
#define F_PARDTOR "par.dtor(this=%s)"
#define F_INFOCTOR "info.ctor(this=%s params=%s)"
#define F_JDCTOR "jd.ctor(this=%s params=%s)"
#define F_JDDTOR "jd.dtor(this=%s)"
#define F_JD92CTOR "jd92.ctor(this=%s params=%s)"
#define F_JD92DTOR "jd92.dtor(this=%s)"
#define F_MODCTOR "mod.ctor(this=%s blk=%u info=%s jd=%s jd92=%s dil=%s mp1=%s mp2=%s addcp=%s cp=%s mp=%s params=%s session=%u)"
#define F_MODDTOR "mod.dtor(this=%s)"
#define F_MODSSF "mod.setSessionFlag(this=%s f=%u)"
#define F_MODRESET "mod.reset(this=%s)"
#define F_MODPROG "mod.progress(this=%s in=%s nb=%s out=%s n=%u nbv=%u)"
#define F_DEMCTOR "dem.ctor(this=%s blk=%u info=%s jd=%s jd92=%s dil=%s mp1=%s mp2=%s addcp=%s cp=%s mp=%s codec=%u params=%s mode=%u session=%u)"
#define F_DEMDTOR "dem.dtor(this=%s)"
#define F_DEMSSF "dem.setSessionFlag(this=%s f=%u)"
#define F_DEMRESET "dem.reset(this=%s qc=%u)"
#define F_DEMPROG "dem.progress(this=%s out=%s cnt=%s in=%s n=%u cntv=%u)"
#define F_SETDIL "setDilDescriptor(dil=%s type=%u)"
#define F_PAR92CTOR "par92.ctor(this=%s p=%s)"
#define F_PAR92DTOR "par92.dtor(this=%s)"
#define F_INFO92CTOR "info92.ctor(this=%s params=%s)"
#define F_CP92CTOR "cp92.ctor(this=%s)"
#define F_CP92DTOR "cp92.dtor(this=%s)"
#define F_MKCONST "V92createConstellations(%s)"
#define F_MKFILT "V92createFilterCoefficients(%s)"
#define F_RMCONST "V92deleteConstellations(%s)"
#define F_RMFILT "V92deleteFilterCoefficients(%s)"
#define F_MOD92CTOR "mod92.ctor(this=%s blk=%u info=%s ja=%s dil=%s cp=%s map=%s params=%s)"
#define F_MOD92DTOR "mod92.dtor(this=%s)"
#define F_MOD92PROG "mod92.progress(this=%s in=%s nb=%s out=%s n=%u nbv=%u)"
#define F_MOD92RESET "mod92.reset(this=%s)"
#define F_MOD92EP3 "mod92.enterPhase3(this=%s)"
#define F_PACK92 "V92DILdescriptorPacker(dil=%s out=%s len=%s)"

static unsigned rd32(const void *p) { uint32_t v; if (!p) return 0xdeadbeefu; memcpy(&v, p, 4); return v; }
void c_mpctor(void *d) { REC(IW, IC, IL, F_MPCTOR, T_(d)); }
void c_mpdtor(void *d) { REC(IW, IC, IL, F_MPDTOR, T_(d)); }
void c_cpctor(void *d) { REC(IW, IC, IL, F_CPCTOR, T_(d)); }
void c_cpdtor(void *d) { REC(IW, IC, IL, F_CPDTOR, T_(d)); }
void c_parctor(void *d, void *p) { REC(IW, IC, IL, F_PARCTOR, T_(d), T_(p)); }
void c_pardtor(void *d) { REC(IW, IC, IL, F_PARDTOR, T_(d)); }
void c_infoctor(void *d, void *p) { REC(IW, IC, IL, F_INFOCTOR, T_(d), T_(p)); }
void c_jdctor(void *d, void *p) { REC(IW, IC, IL, F_JDCTOR, T_(d), T_(p)); }
void c_jddtor(void *d) { REC(IW, IC, IL, F_JDDTOR, T_(d)); }
void c_jd92ctor(void *d, void *p) { REC(IW, IC, IL, F_JD92CTOR, T_(d), T_(p)); }
void c_jd92dtor(void *d) { REC(IW, IC, IL, F_JD92DTOR, T_(d)); }
void c_modctor(void *d, unsigned blk, void *info, void *jd, void *jd92, void *dil, void *mp1, void *mp2, void *acp, void *cp, void *mp, void *par, unsigned s) {
  REC(IW, IC, IL, F_MODCTOR, T_(d), blk, T_(info), T_(jd), T_(jd92), T_(dil), T_(mp1), T_(mp2), T_(acp), T_(cp), T_(mp), T_(par), s);
}
void c_moddtor(void *d) { REC(IW, IC, IL, F_MODDTOR, T_(d)); }
void c_modssf(void *d, unsigned f) { REC(IW, IC, IL, F_MODSSF, T_(d), f); }
void c_modreset(void *d) { REC(IW, IC, IL, F_MODRESET, T_(d)); }
void c_modprog(void *d, int *in, unsigned *nb, float *out, unsigned n) { REC(IW, IC, IL, F_MODPROG, T_(d), T_(in), T_(nb), T_(out), n, rd32(nb)); }
void c_demctor(void *d, unsigned blk, void *info, void *jd, void *jd92, void *dil, void *mp1, void *mp2, void *acp, void *cp, void *mp, unsigned codec, void *par, unsigned mode, unsigned s) {
  REC(IW, IC, IL, F_DEMCTOR, T_(d), blk, T_(info), T_(jd), T_(jd92), T_(dil), T_(mp1), T_(mp2), T_(acp), T_(cp), T_(mp), codec, T_(par), mode, s);
}
void c_demdtor(void *d) { REC(IW, IC, IL, F_DEMDTOR, T_(d)); }
void c_demssf(void *d, unsigned f) { REC(IW, IC, IL, F_DEMSSF, T_(d), f); }
void c_demreset(void *d, unsigned qc) { REC(IW, IC, IL, F_DEMRESET, T_(d), qc); }
void c_demprog(void *d, int *out, unsigned *cnt, float *in, unsigned n) { REC(IW, IC, IL, F_DEMPROG, T_(d), T_(out), T_(cnt), T_(in), n, rd32(cnt)); }
void c_setdil(void *d, unsigned t) { REC(IW, IC, IL, F_SETDIL, T_(d), t); }
void c_par92ctor(void *d, void *p) { REC(IW, IC, IL, F_PAR92CTOR, T_(d), T_(p)); }
void c_par92dtor(void *d) { REC(IW, IC, IL, F_PAR92DTOR, T_(d)); }
void c_info92ctor(void *d, void *p) { REC(IW, IC, IL, F_INFO92CTOR, T_(d), T_(p)); }
void c_cp92ctor(void *d) { REC(IW, IC, IL, F_CP92CTOR, T_(d)); }
void c_cp92dtor(void *d) { REC(IW, IC, IL, F_CP92DTOR, T_(d)); }
void c_mkconst(void *p) { REC(IW, IC, IL, F_MKCONST, T_(p)); }
void c_mkfilt(void *p) { REC(IW, IC, IL, F_MKFILT, T_(p)); }
void c_rmconst(void *p) { REC(IW, IC, IL, F_RMCONST, T_(p)); }
void c_rmfilt(void *p) { REC(IW, IC, IL, F_RMFILT, T_(p)); }
void c_mod92ctor(void *d, unsigned blk, void *info, void *ja, void *dil, void *cp, void *map, void *par) {
  REC(IW, IC, IL, F_MOD92CTOR, T_(d), blk, T_(info), T_(ja), T_(dil), T_(cp), T_(map), T_(par));
}
void c_mod92dtor(void *d) { REC(IW, IC, IL, F_MOD92DTOR, T_(d)); }
void c_mod92prog(void *d, int *in, unsigned *nb, float *out, unsigned n) { REC(IW, IC, IL, F_MOD92PROG, T_(d), T_(in), T_(nb), T_(out), n, rd32(nb)); }
void c_mod92reset(void *d) { REC(IW, IC, IL, F_MOD92RESET, T_(d)); }
void c_mod92ep3(void *d) { REC(IW, IC, IL, F_MOD92EP3, T_(d)); }
void c_pack92(void *dil, unsigned char *out, int *len) { REC(IW, IC, IL, F_PACK92, T_(dil), T_(out), T_(len)); }
void edprintf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  cb_ed(IW, IC, IL, 'e', buf);
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  cb_ed(IW, IC, IL, 'd', buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the independent model (the blob's control flow, from porting/decompiled/dp/p45_modem.txt) */
/* the model's records: the same text as the recorders, with block tokens spelled by id (heap blocks by their token) */
#define MREC(...) do { rec_add(ML, 1, 'c', __VA_ARGS__); poke(MW, MC); } while (0)
static void mpf(char kind, const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  cb_ed(MW, MC, ML, kind, b);
}
static const char *tk(const void *p) { return T_(p); }
static const char *ix(int id) { return bname[id]; }

#define STARS "*********************************************************\r\n"
#define PRIV "*******         This is a PRIVATE version         *******\r\n"
#define FORSL "*******     for the use of SL DSP group only      *******\r\n"
static void m90_title(void) {
  mpf('e', STARS); mpf('e', PRIV); mpf('e', FORSL);
  if (DBG()) {
    CV(G_V90M, "v90 title: banner");
    mpf('d', STARS);
    if (DBG()) {
      mpf('d', "V90Modem Version: 2.98  (25-Mar-04)\r\n");
      if (DBG()) mpf('d', STARS);
      else CV(G_V90M, "v90 title: level dropped after the version line");
    } else CV(G_V90M, "v90 title: level dropped after the first stars");
  } else CV(G_V90M, "v90 title: no banner");
  mpf('e', "V90Modem Version Description:\r\n");
  mpf('e', "Modified Quick Connect without Memory + Train Time + Constel Power\r\n");
  mpf('e', "Components: Floreat, ADI, ACD, New BLL\r\n");
  if (DBG()) mpf('d', STARS);
}
static void m92_title(void) {
  mpf('e', STARS); mpf('e', PRIV); mpf('e', FORSL);
  if (DBG()) {
    CV(G_V92M, "v92 title: banner");
    mpf('d', STARS);
    if (DBG()) {
      mpf('d', "V92Modem Version: 1.1  (9-Apr-01)\r\n");
      if (DBG()) mpf('d', STARS);
      else CV(G_V92M, "v92 title: level dropped after the version line");
    } else CV(G_V92M, "v92 title: level dropped after the first stars");
  } else CV(G_V92M, "v92 title: no banner");
  mpf('e', "V92Modem Version Description:\r\n");
  mpf('e', "Memory cleanups + dil descriptor crash fix\r\n");
  mpf('e', STARS);
}

/* the blocks the constructor allocated: the allocator table entries n0 .. n0 + count - 1 (the arena is a bump allocator: address order = allocation order) */
static unsigned g_n0;
static int g_ctor_illegal;     /* the model's constructor took the illegal-side path: mod_ / dem_ keep their old (garbage) bytes */
static unsigned g_nblkexp;     /* blocks the impl's constructor allocated */
static unsigned g_nblkused;    /* blocks the model's constructor used */
static void *blkp(unsigned k, unsigned size) {
  if (k >= g_nblkexp) h_die3("model: the constructor should allocate block %u, the impl allocated only %u", k, g_nblkexp);
  if (h_al[g_n0 + k].size != size) h_die3("model: block %u has %u bytes, expected %u", k, h_al[g_n0 + k].size, size);
  if (k + 1 > g_nblkused) g_nblkused = k + 1;
  return h_al[g_n0 + k].p;
}

static void m90_ctor(unsigned side, unsigned blk, unsigned mode, unsigned session, void *dil) {
  W *w = MW;
  ModemV90Obj *t = O90(w);
  void *par, *info, *jd, *jd92, *sub;
  unsigned codec;
  CV(G_V90C, "v90 ctor");
  MREC(F_MPCTOR, ix(X_MPV));
  MREC(F_CPCTOR, ix(X_CPV));
  if (DBG()) mpf('d', "V90Modem Construction (as %s Modem)\r\n", side == 0 ? "Digital" : "Analog");
  m90_title();
  t->side_ = side; t->dil_ = dil; t->session_ = session;
  par = blkp(0, p45_sizes[0]);
  MREC(F_PARCTOR, tk(par), ix(B_MPAR));
  t->params_ = par;
  info = blkp(1, p45_sizes[1]);
  MREC(F_INFOCTOR, tk(info), tk(t->params_));
  t->info_ = info;
  jd = blkp(2, p45_sizes[2]);
  MREC(F_JDCTOR, tk(jd), tk(t->params_));
  t->jd_ = jd;
  jd92 = blkp(3, p45_sizes[3]);
  MREC(F_JD92CTOR, tk(jd92), tk(t->params_));
  t->jd92_ = jd92;
  if (t->side_ == 0) {
    CV(G_V90C, "v90 ctor: side 0 (modulator)");
    sub = blkp(4, p45_sizes[4]);
    MREC(F_MODCTOR, tk(sub), blk, tk(t->info_), tk(t->jd_), tk(t->jd92_), tk(t->dil_), ix(X_MP1), ix(X_MP2), ix(X_ADDCP), ix(X_CPV), ix(X_MPV), tk(t->params_), t->session_);
    t->mod_ = sub;
    t->dem_ = NULL;
  } else if (t->side_ == 1) {
    CV(G_V90C, "v90 ctor: side 1 (demodulator)");
    t->mod_ = NULL;
    sub = blkp(4, p45_sizes[5]);
    memcpy(&codec, w->b[B_MPAR] + 0x54, 4);
    MREC(F_DEMCTOR, tk(sub), blk, tk(t->info_), tk(t->jd_), tk(t->jd92_), tk(t->dil_), ix(X_MP1), ix(X_MP2), ix(X_ADDCP), ix(X_CPV), ix(X_MPV), codec, tk(t->params_), mode, t->session_);
    t->dem_ = sub;
  } else {
    CV(G_V90C, "v90 ctor: illegal side"); g_ctor_illegal = 1;
    if (DBG()) mpf('d', "V90Modem Constructor: Illegal modemSide\r\n");
  }
}
static void m90_dtor(void) {
  ModemV90Obj *t = O90(MW);
  CV(G_V90D, "v90 dtor");
  if (DBG()) mpf('d', "V90Modem Destruction\r\n");
  if (t->side_ > 1) {
    CV(G_V90D, "v90 dtor: side > 1");
    if (DBG()) mpf('d', "V90Modem Destructor: Illegal modemSide\r\n");
  }
  if (t->mod_) { CV(G_V90D, "v90 dtor: mod"); MREC(F_MODDTOR, tk(t->mod_)); }
  if (t->dem_) { CV(G_V90D, "v90 dtor: dem"); MREC(F_DEMDTOR, tk(t->dem_)); }
  if (t->jd_) { CV(G_V90D, "v90 dtor: jd"); MREC(F_JDDTOR, tk(t->jd_)); }
  if (t->jd92_) { CV(G_V90D, "v90 dtor: jd92"); MREC(F_JD92DTOR, tk(t->jd92_)); }
  if (t->params_) { CV(G_V90D, "v90 dtor: params"); MREC(F_PARDTOR, tk(t->params_)); }
  MREC(F_CPDTOR, ix(X_CPV));
  MREC(F_MPDTOR, ix(X_MPV));
}
static void m90_reset(unsigned qc) {
  ModemV90Obj *t = O90(MW);
  if (DBG()) mpf('d', "V90Modem Reset, qcFlag = %d\r\n", (int)qc);
  m90_title();
  if (t->side_ == 0) { CV(G_V90M, "v90 reset: side 0"); MREC(F_MODRESET, tk(t->mod_)); return; }
  if (t->side_ == 1) {
    V90ParObj *p = (V90ParObj *)t->params_;
    CV(G_V90M, "v90 reset: side 1");
    if (p->probingMode != 0) { CV(G_V90M, "v90 reset: probing mode masks the quick connect"); mpf('e', "due to probe mode quick connect is masked !!!\r\n"); qc = 0; }
    MREC(F_SETDIL, tk(t->dil_), qc != 0 ? 1u : 0u);
    MREC(F_DEMRESET, tk(t->dem_), qc);
    return;
  }
  CV(G_V90M, "v90 reset: illegal side");
  if (DBG()) mpf('d', "V90Modem Reset: Illegal modemSide\r\n");
}
static void m90_ssf(unsigned f) {
  ModemV90Obj *t = O90(MW);
  unsigned side = t->side_;
  t->session_ = f;
  if (side == 0) { CV(G_V90M, "v90 ssf: side 0"); MREC(F_MODSSF, tk(t->mod_), f); }
  else if (side == 1) { CV(G_V90M, "v90 ssf: side 1"); MREC(F_DEMSSF, tk(t->dem_), f); }
  else CV(G_V90M, "v90 ssf: other side");
}
static void m90_prog(unsigned n) {
  ModemV90Obj *t = O90(MW);
  if (t->side_ == 0) { CV(G_V90M, "v90 progress: side 0"); MREC(F_MODPROG, tk(t->mod_), ix(B_IN), ix(B_NB), ix(B_OUT), n, rd32(MW->b[B_NB])); }
  else if (t->side_ == 1) { CV(G_V90M, "v90 progress: side 1"); MREC(F_DEMPROG, tk(t->dem_), ix(B_IN), ix(B_NB), ix(B_OUT), n, rd32(MW->b[B_NB])); }
  else {
    CV(G_V90M, "v90 progress: illegal side");
    if (DBG()) mpf('d', "V90Modem progress: Illegal modemSide\r\n");
  }
}
static void m92_ctor(unsigned side, unsigned blk, void *dil) {
  W *w = MW;
  ModemV92Obj *t = O92(w);
  void *par, *info, *cp, *map;
  CV(G_V92C, "v92 ctor");
  if (DBG()) mpf('d', "V92Modem Construction (as %s Modem)\r\n", side == 0 ? "Digital" : "Analog");
  m92_title();
  t->side_ = side;
  par = blkp(0, p45_sizes[6]);
  MREC(F_PAR92CTOR, tk(par), ix(B_MPAR));
  t->params_ = par;
  t->dil_ = dil;
  info = blkp(1, p45_sizes[7]);
  MREC(F_INFO92CTOR, tk(info), tk(t->params_));
  t->info_ = info;
  cp = blkp(2, p45_sizes[8]);
  MREC(F_CP92CTOR, tk(cp));
  t->cp_ = cp;
  map = blkp(3, p45_sizes[9]);
  t->mapping_ = map;
  MREC(F_MKCONST, tk(map));
  MREC(F_MKFILT, tk(t->mapping_));
  if (t->side_ == 0) { CV(G_V92C, "v92 ctor: side 0"); t->mod_ = NULL; }
  else if (t->side_ == 1) {
    void *sub = blkp(4, p45_sizes[10]);
    CV(G_V92C, "v92 ctor: side 1 (modulator)");
    MREC(F_MOD92CTOR, tk(sub), blk, tk(t->info_), ix(X_JA), tk(dil), tk(t->cp_), tk(t->mapping_), tk(t->params_));
    t->mod_ = sub;
  } else {
    CV(G_V92C, "v92 ctor: illegal side"); g_ctor_illegal = 1;
    if (DBG()) mpf('d', "V92Modem Constructor: Illegal modemSide\r\n");
  }
}
static void m92_dtor(void) {
  ModemV92Obj *t = O92(MW);
  CV(G_V92D, "v92 dtor");
  if (DBG()) mpf('d', "V92Modem Destruction\r\n");
  if (t->side_ > 1) {
    CV(G_V92D, "v92 dtor: side > 1");
    if (DBG()) mpf('d', "V92Modem Destructor: Illegal modemSide\r\n");
  }
  MREC(F_RMCONST, tk(t->mapping_));
  MREC(F_RMFILT, tk(t->mapping_));
  if (t->mod_) { CV(G_V92D, "v92 dtor: mod"); MREC(F_MOD92DTOR, tk(t->mod_)); }
  if (t->cp_) { CV(G_V92D, "v92 dtor: cp"); MREC(F_CP92DTOR, tk(t->cp_)); }
  if (t->params_) { CV(G_V92D, "v92 dtor: params"); MREC(F_PAR92DTOR, tk(t->params_)); }
  if (t->info_) { CV(G_V92D, "v92 dtor: info"); t->info_ = NULL; }
}
static void m92_reset(void) {
  ModemV92Obj *t = O92(MW);
  if (DBG()) mpf('d', "V92Modem Reset\r\n");
  m92_title();
  if (t->side_ == 0) { CV(G_V92M, "v92 reset: side 0"); return; }
  if (t->side_ == 1) {
    CV(G_V92M, "v92 reset: side 1");
    MREC(F_PACK92, tk(t->dil_), ix(X_JABITS), ix(X_JA));
    MREC(F_MOD92RESET, tk(t->mod_));
    MREC(F_MOD92EP3, tk(t->mod_));
    return;
  }
  CV(G_V92M, "v92 reset: illegal side");
  if (DBG()) mpf('d', "V92Modem Reset: Illegal modemSide\r\n");
}
static void m92_prog(unsigned n) {
  ModemV92Obj *t = O92(MW);
  if (t->side_ == 0) { CV(G_V92M, "v92 progress: side 0"); return; }
  if (t->side_ == 1) { CV(G_V92M, "v92 progress: side 1"); MREC(F_MOD92PROG, tk(t->mod_), ix(B_IN), ix(B_NB), ix(B_OUT), n, rd32(MW->b[B_NB])); return; }
  CV(G_V92M, "v92 progress: illegal side");
  if (DBG()) mpf('d', "V92Modem progress: Illegal modemSide\r\n");
}

/* ------------------------------------------------------------------------------------------------ world setup */
static void reg(int id, void *p, size_t n) { g_iw.b[id] = p; g_blkidx[id] = blk_add(bname[id], p, n); g_obj[id] = p; g_idname[id] = bname[id]; }
static void fill_obj(uint8_t *t) {
  const HFld *fl = g_kind == 90 ? f_ModemV90Obj : f_ModemV92Obj;
  int nf = g_kind == 90 ? NF_ModemV90Obj : NF_ModemV92Obj, i;
  for (i = 0; i < nf; i++) {
    const HFld *f = &fl[i];
    unsigned n = f->count ? f->count : 1, es, tot, j;
    uint8_t *q = t + f->off;
    switch (f->kind) {
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'u': case 'i': case 'f': es = 4; break;
      case 'p': case 'o': {
        unsigned e;
        for (e = 0; e < n; e++) { void *v; uintptr_t r = h_rnd(); r = (r << 4) ^ (uintptr_t)h_rnd(); v = (void *)((r & 0x3ffffff0u) | 1u); memcpy(q + e * sizeof v, &v, sizeof v); }
        continue;
      }
      default: continue;
    }
    tot = n * es;
    for (j = 0; j < tot; j += 4) { uint32_t r = h_rnd(); memcpy(q + j, &r, tot - j < 4 ? tot - j : 4); }
  }
}
static void *newblk(int id, size_t n) { void *p = h_obj_new(n); reg(id, p, n); return p; }
static void new_world(int kind, int built) {
  uint8_t *raw;
  int k;
  g_kind = kind;
  g_osz = kind == 90 ? sizeof(ModemV90Obj) : sizeof(ModemV92Obj);
  raw = (uint8_t *)h_obj_new(g_osz + CAN);
  memset(&g_iw, 0, sizeof g_iw);
  blk_reset();
  g_nobj = NID;
  for (k = 0; k < NID; k++) { g_obj[k] = NULL; g_idname[k] = bname[k]; }
  memset(raw, 0, g_osz);
  memset(raw + g_osz, 0x5a, CAN);
  g_can = raw + g_osz;
  fill_obj(raw);
  reg(B_OBJ, raw, g_osz);
  g_blk[g_blkidx[B_OBJ]].n = g_osz;
  { uint8_t *m = (uint8_t *)newblk(B_MPAR, 0x100); uint32_t v = h_rnd(); memcpy(m + 0x54, &v, 4); }
  newblk(B_DIL, 0x40);
  { V90ParObj *p = (V90ParObj *)newblk(B_PAR, sizeof(V90ParObj)); p->probingMode = h_rpct(50) ? 0 : h_rpct(60) ? 1 : (int32_t)h_rnd(); }
  { V90ParObj *p = (V90ParObj *)newblk(B_PAR2, sizeof(V90ParObj)); p->probingMode = h_rpct(50) ? 0 : h_rpct(60) ? 1 : (int32_t)h_rnd(); }
  newblk(B_INFO, 0x40); newblk(B_JD, 0x40); newblk(B_JD92, 0x40); newblk(B_MOD, 0x40); newblk(B_DEM, 0x40); newblk(B_CP, 0x40); newblk(B_MAP, 0x40);
  newblk(B_IN, 64 * 4); newblk(B_NB, 16); newblk(B_OUT, 64 * 4);
  { unsigned i; for (i = 0; i < 64; i++) { ((uint32_t *)g_iw.b[B_IN])[i] = h_rnd(); ((uint32_t *)g_iw.b[B_OUT])[i] = h_rnd(); } }
  { uint32_t v = h_rpct(30) ? 0u : h_rnd(); memcpy(g_iw.b[B_NB], &v, 4); blk_sync_model(); }
  /* interior pointers */
  if (kind == 90) {
    g_obj[X_MP1] = raw + offsetof(ModemV90Obj, mp1_); g_obj[X_MP2] = raw + offsetof(ModemV90Obj, mp2_); g_obj[X_ADDCP] = raw + offsetof(ModemV90Obj, addCp_);
    g_obj[X_MPV] = raw + offsetof(ModemV90Obj, mp_); g_obj[X_CPV] = raw + offsetof(ModemV90Obj, cp_head_);
  } else {
    g_obj[X_JA] = raw + offsetof(ModemV92Obj, jaLen_); g_obj[X_JABITS] = raw + offsetof(ModemV92Obj, jaBits_);
  }
  if (built) {   /* a hand-built object whose pointer members point to the world's blocks */
    if (kind == 90) {
      ModemV90Obj *t = (ModemV90Obj *)raw;
      t->params_ = g_iw.b[B_PAR]; t->info_ = g_iw.b[B_INFO]; t->jd_ = g_iw.b[B_JD]; t->jd92_ = g_iw.b[B_JD92]; t->dil_ = g_iw.b[B_DIL];
      t->mod_ = h_rpct(8) ? NULL : g_iw.b[B_MOD]; t->dem_ = h_rpct(8) ? NULL : g_iw.b[B_DEM];
      t->side_ = pick_side(h_rnd()); t->session_ = pick_flag();
    } else {
      ModemV92Obj *t = (ModemV92Obj *)raw;
      t->params_ = g_iw.b[B_PAR]; t->info_ = g_iw.b[B_INFO]; t->cp_ = g_iw.b[B_CP]; t->mapping_ = g_iw.b[B_MAP]; t->dil_ = g_iw.b[B_DIL];
      t->mod_ = h_rpct(8) ? NULL : g_iw.b[B_MOD];
      t->side_ = pick_side(h_rnd());
    }
    blk_sync_model();
  }
}
static void world_model(void) { int k; for (k = 0; k < NBLK; k++) g_mw.b[k] = g_blk[g_blkidx[k]].m; }
static void emit_state(const char *tag) {
  const HFld *fl = g_kind == 90 ? f_ModemV90Obj : f_ModemV92Obj;
  int nf = g_kind == 90 ? NF_ModemV90Obj : NF_ModemV92Obj;
  h_dump_obj(g_show, tag, g_iw.b[B_OBJ], fl, nf);
}


/* ------------------------------------------------------------------------------------------------ one call */
enum { OP_C1, OP_C2, OP_D1, OP_D2, OP_TITLE, OP_RESET, OP_SSF, OP_PROG };
static const char *const opname[] = { "C1", "C2", "D1", "D2", "printTitle", "reset", "setSessionFlag", "progress" };
typedef struct { int op; unsigned side, blk, mode, session, u, n; void *dil; } Call;

static void do_call(Call *c) {
  unsigned i, nm0 = (unsigned)h_nmalloc, lv0, cnt = 0;
  int g = g_kind == 90, isctor = c->op <= OP_C2, isdtor = c->op == OP_D1 || c->op == OP_D2;
  uint8_t *t = g_iw.b[B_OBJ];
  void *own[6];
  unsigned nown = 0;
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = opname[c->op];
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  lv0 = dsplibs_debug_level;
  g_seq = 0;
  h_emit(g_show, "call %s%d side=%u blk=%u mode=%u session=%u u=%u n=%u lvl=%u", opname[c->op], g_kind, c->side, c->blk, c->mode, c->session, c->u, c->n, dsplibs_debug_level);
  if (isdtor) {   /* the blocks the destructor must free: every non-NULL owner pointer of the object (the pokes of a destructor never change pointers) */
    if (g) { ModemV90Obj *o = O90(&g_iw); void *l[6]; l[0] = o->mod_; l[1] = o->dem_; l[2] = o->info_; l[3] = o->jd_; l[4] = o->jd92_; l[5] = o->params_; for (i = 0; i < 6; i++) if (l[i]) own[nown++] = l[i]; }
    else { ModemV92Obj *o = O92(&g_iw); void *l[5]; l[0] = o->mapping_; l[1] = o->mod_; l[2] = o->cp_; l[3] = o->params_; l[4] = o->info_; for (i = 0; i < 5; i++) if (l[i]) own[nown++] = l[i]; }
  }
  /* ---- the impl */
  switch (c->op) {
    case OP_C1: if (g) f90_c1(t, c->side, g_iw.b[B_MPAR], c->dil, c->blk, c->mode, c->session); else f92_c1(t, c->side, g_iw.b[B_MPAR], c->blk, c->dil, c->mode); break;
    case OP_C2: if (g) f90_c2(t, c->side, g_iw.b[B_MPAR], c->dil, c->blk, c->mode, c->session); else f92_c2(t, c->side, g_iw.b[B_MPAR], c->blk, c->dil, c->mode); break;
    case OP_D1: if (g) f90_d1(t); else f92_d1(t); break;
    case OP_D2: if (g) f90_d2(t); else f92_d2(t); break;
    case OP_TITLE: if (g) f90_title(t); else f92_title(t); break;
    case OP_RESET: if (g) f90_reset(t, c->u); else f92_reset(t); break;
    case OP_SSF: f90_ssf(t, c->u); break;
    case OP_PROG: if (g) f90_prog(t, (int *)g_iw.b[B_IN], (unsigned *)g_iw.b[B_NB], (float *)g_iw.b[B_OUT], c->n); else f92_prog(t, (int *)g_iw.b[B_IN], (unsigned *)g_iw.b[B_NB], (float *)g_iw.b[B_OUT], c->n); break;
  }
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  if (isctor) {
    cnt = (unsigned)h_nmalloc - nm0;
    if (cnt < 4 || cnt > 5) h_die3("the constructor allocated %u blocks", cnt);
    g_nblkexp = cnt; g_nblkused = 0; g_ctor_illegal = 0;
    g_n0 = h_last_alloc + 1 - cnt;
  }
  /* ---- the model (the debug level it starts with is the impl's) */
  dsplibs_debug_level = lv0;
  switch (c->op) {
    case OP_C1: case OP_C2: if (g) m90_ctor(c->side, c->blk, c->mode, c->session, c->dil); else m92_ctor(c->side, c->blk, c->dil); break;
    case OP_D1: case OP_D2: if (g) m90_dtor(); else m92_dtor(); break;
    case OP_TITLE: if (g) m90_title(); else m92_title(); break;
    case OP_RESET: if (g) m90_reset(c->u); else m92_reset(); break;
    case OP_SSF: m90_ssf(c->u); break;
    case OP_PROG: if (g) m90_prog(c->n); else m92_prog(c->n); break;
  }
  log_cmp(&g_ml, &g_il, g_what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the poke cursors differ", g_what);
  if (isctor && g_nblkused != cnt) h_die3("%s: the model used %u blocks, the impl allocated %u", g_what, g_nblkused, cnt);
  blk_compare(g_what);
  for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i);
  if (isdtor) {
    for (i = 0; i < nown; i++) { HAl *a = h_find_al(own[i], 0); if (!a || !a->freed) h_die3("%s: owner block %u was not freed", g_what, i); }
    if (g) CV(G_V90D, "v90 dtor: every owner block freed"); else CV(G_V92D, "v92 dtor: every owner block freed");
  }
  emit_state("D");
  if (isctor && g_freector) { for (i = 0; i < cnt; i++) sysdep_free(h_al[g_n0 + i].p); }
  cov_flush();
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static void pick_ctor(Call *c) {
  c->side = pick_side(h_rnd());
  c->blk = h_rpct(50) ? h_rmod(64) : h_rnd();
  c->mode = h_rpct(60) ? h_rmod(3) : h_rnd();
  c->session = pick_flag();
  { unsigned r = h_rmod(10); c->dil = r < 2 ? NULL : r < 3 ? (void *)g_iw.b[B_PAR2] : (void *)g_iw.b[B_DIL]; }
  c->op = h_rpct(50) ? OP_C1 : OP_C2;
}
/* a constructor that took the illegal path leaves the owner pointers it did not set (random garbage): clear them as an owner would */
static void tidy_after_ctor(void) {
  if (!g_ctor_illegal) return;
  if (g_kind == 90) { ModemV90Obj *o = O90(&g_iw); o->mod_ = o->dem_ = NULL; }
  else { ModemV92Obj *o = O92(&g_iw); o->mod_ = NULL; }
}
static unsigned mask_of(const int *k, int n) { unsigned m = 0; int i; for (i = 0; i < n; i++) m |= 1u << k[i]; return m; }
static void set_mask(int phase) {   /* 0 ctor only, 1 ctor of a lifecycle (no pointer pokes), 2 methods of a hand-built object, 3 methods of a built object, 4 destructor */
  static const int all[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }, life[] = { 1, 2, 3, 10 }, dt[] = { 1, 2, 10 };
  switch (phase) {
    case 0: g_pokemask = mask_of(all, 10); break;
    case 1: g_pokemask = mask_of(dt, 3); break;
    case 2: g_pokemask = mask_of(all, 10); break;
    case 3: g_pokemask = mask_of(life, 4); break;
    default: g_pokemask = mask_of(dt, 3); break;
  }
}
static void method_call(Call *c) {
  c->op = (int)OP_TITLE + (int)h_rmod(g_kind == 90 ? 4 : 3);
  if (g_kind == 92 && c->op == OP_SSF) c->op = OP_PROG;
  c->u = h_rpct(40) ? h_rmod(2) : h_rnd();
  c->n = h_rpct(20) ? 0u : h_rnd();
}
static void trial(int sc) {
  Call c;
  int kind = sc == 0 || sc == 1 || sc == 2 || sc == 6 ? 90 : 92, k, nm;
  int built = sc == 2 || sc == 5;
  new_world(kind, built);
  gen_script();
  dsplibs_debug_level = pick_level();
  g_freector = 0;
  memset(&c, 0, sizeof c);
  switch (sc) {
    case 0: case 3:
      set_mask(0); g_freector = 1;
      pick_ctor(&c);
      do_call(&c);
      break;
    case 1: case 4: {
      set_mask(1);
      pick_ctor(&c);
      if (h_rpct(70)) c.side = (unsigned)h_rmod(2);   /* mostly a real side: the owner pointers then exist */
      do_call(&c);
      tidy_after_ctor();
      /* a constructed object: sometimes with some owner pointers NULL (the destructor tests each) and the side changed */
      if (kind == 90) {
        ModemV90Obj *o = O90(&g_iw);
        void **pp[6];
        pp[0] = &o->mod_; pp[1] = &o->dem_; pp[2] = &o->info_; pp[3] = &o->jd_; pp[4] = &o->jd92_; pp[5] = &o->params_;
        for (k = 0; k < 6; k++) if (*pp[k] && h_rpct(25)) { sysdep_free(*pp[k]); *pp[k] = NULL; }
        if (h_rpct(30)) o->side_ = pick_side(h_rnd());
      } else {
        ModemV92Obj *o = O92(&g_iw);
        void **pp[4];
        pp[0] = &o->mod_; pp[1] = &o->cp_; pp[2] = &o->params_; pp[3] = &o->info_;
        for (k = 0; k < 4; k++) if (*pp[k] && h_rpct(25)) { sysdep_free(*pp[k]); *pp[k] = NULL; }
        if (o->mapping_ && h_rpct(20)) { sysdep_free(o->mapping_); o->mapping_ = NULL; }
        if (h_rpct(30)) o->side_ = pick_side(h_rnd());
      }
      gen_script();
      set_mask(4);
      memset(&c, 0, sizeof c);
      c.op = h_rpct(50) ? OP_D1 : OP_D2;
      do_call(&c);
      break;
    }
    case 2: case 5:
      nm = h_rpct(30) ? (int)rnd_range(2, 4) : 1;
      for (k = 0; k < nm; k++) {
        if (k) gen_script();
        set_mask(2);
        memset(&c, 0, sizeof c);
        method_call(&c);
        if (h_rpct(10)) dsplibs_debug_level = pick_level();
        do_call(&c);
      }
      break;
    default:
      set_mask(1);
      pick_ctor(&c);
      if (h_rpct(60)) c.side = (unsigned)h_rmod(2);
      do_call(&c);
      tidy_after_ctor();
      nm = (int)rnd_range(1, 4);
      for (k = 0; k < nm; k++) {
        gen_script();
        set_mask(3);
        memset(&c, 0, sizeof c);
        method_call(&c);
        if (h_rpct(10)) dsplibs_debug_level = pick_level();
        do_call(&c);
      }
      gen_script();
      set_mask(4);
      memset(&c, 0, sizeof c);
      c.op = h_rpct(50) ? OP_D1 : OP_D2;
      do_call(&c);
      break;
  }
  h_trial_end("p45");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr, k;
  const char *seed = getenv("P45_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P45_ALL")) g_showall = 1;
  if (sc < 0 || sc > 7) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  p07_layout_check(); p45_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P45_TRIALS") ? atoi(getenv("P45_TRIALS")) : (sc == 2 || sc == 5 ? 15000 : 8000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P45_NOCOV")) ? 2 : 0;
}
