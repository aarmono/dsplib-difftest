/*
 * Differential test harness for port chunks P08 + P07 (src/vpcm/pcm_parameters_load.cpp, src/vpcm/pcm_parameters.cpp): V90Parameters (all 9 + 2 functions incl. the 295-call
 * loadParams) and V92Parameters (7 + 2 functions incl. the 54-call loadParams).
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/pcm_parameters.main.sh against the blob (a copy of dsplibs.o in which Vparser_read_int /
 * Vparser_read_float / edprintf are WEAK so that the recorders below win), against the port (-O0 / -O2) and for x86-64 (diffed against the 32-bit BLOB's output =
 * the golden, test/golden/vpcm/pcm_parameters.main.blob32.out).  The functions are reached through __asm__ labels with the mangled names (identical in blob and port; C1 AND C2, D1 AND D2),
 * the object layouts are C mirror structs GENERATED from test/vpcm/pcm_parameters.layout (REAL pointer members: the 64-bit layout moves every word by 4 bytes, so everything is
 * compared FIELD-WISE by member name; a 32-bit offset of the blob is mapped to the platform offset through the member table).
 *
 * What is compared: every object after every call (all 341 / 54 words + the params pointer, floats as bit patterns), the whole sequence of Vparser_read_* calls
 * (reader, key text, destination word, parser argument), every edprintf / dsplibs_debug_printf call (text) and the OBJECT SNAPSHOT at every such call (a field-wise
 * hash of the object while the function runs: the store order of setToDefault relative to its two debug prints, the state after k parser calls).  INDEPENDENT
 * MODELS: (1) the call sequences / default words are the blob's own tables extracted from the machine code by tools/params_gen.py (test/vpcm/p07_model.h; the port
 * source is generated from the same extraction, so for these the blob-vs-port comparison is the real check), (2) the head of V90Parameters::setToDefault (rate mask:
 * true division, loops over k = 2 .. 14), the three special words (+0x48, +0x364, +0x4cc), (3) loadModemParamsData in pure integer arithmetic (q = x / 5, v = q / 2,
 * ip = q / 2, fp = 50 * (q & 1), float word of q / 2.0 by a double), including the order in which it re-reads the dp runtime block (the recorders modify the block
 * between the prints), (4) the composition of constructor / init (event order, stores) - a model violation exits 3.
 *
 * Scenarios (each its own process): 0 V92Parameters::loadParams, 1 V90Parameters::loadParams (P08), 2 V92Parameters setToDefault / C1 / C2 / init / D1 / D2,
 * 3 V90Parameters::setToDefault, 4 V90Parameters::loadModemParamsData, 5 V90Parameters C1 / C2 / init / initSession / D1 / D2 (integrated event sequences).
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "pcm_parameters_layout.h"
#include "p07_model.h"

#define L(m) __asm__(m)

/* ---- V90Parameters ---- */
extern void v90_c1(void *, void *) L("_ZN13V90ParametersC1EP19_tagModemParameters");
extern void v90_c2(void *, void *) L("_ZN13V90ParametersC2EP19_tagModemParameters");
extern void v90_d1(void *) L("_ZN13V90ParametersD1Ev");
extern void v90_d2(void *) L("_ZN13V90ParametersD2Ev");
extern void v90_init(void *) L("_ZN13V90Parameters4initEv");
extern void v90_isess(void *) L("_ZN13V90Parameters11initSessionEv");
extern void v90_load(void *, char *) L("_ZN13V90Parameters10loadParamsEPc");
extern void v90_def(void *) L("_ZN13V90Parameters12setToDefaultEv");
extern void v90_lmpd(void *) L("_ZN13V90Parameters19loadModemParamsDataEv");
/* ---- V92Parameters ---- */
extern void v92_c1(void *, void *) L("_ZN13V92ParametersC1EP19_tagModemParameters");
extern void v92_c2(void *, void *) L("_ZN13V92ParametersC2EP19_tagModemParameters");
extern void v92_d1(void *) L("_ZN13V92ParametersD1Ev");
extern void v92_d2(void *) L("_ZN13V92ParametersD2Ev");
extern void v92_init(void *) L("_ZN13V92Parameters4initEv");
extern void v92_load(void *, char *) L("_ZN13V92Parameters10loadParamsEPc");
extern void v92_def(void *) L("_ZN13V92Parameters12setToDefaultEv");
extern unsigned int dsplibs_debug_level;

#define W90 341 /* words after the pointer */
#define W92 54

/* ------------------------------------------------------------------------------------------------ member table access
 * A 32-bit offset of the blob (4 .. 0x554 / 0xd8) is the member number (off - 4) / 4; entry 0 of the table is the params pointer. */
typedef struct {
  const HFld *f;
  int nf;
  size_t size;
  int words;
} Cls;
static const Cls C90 = { f_V90ParObj, NF_V90ParObj, sizeof(V90ParObj), W90 };
static const Cls C92 = { f_V92ParObj, NF_V92ParObj, sizeof(V92ParObj), W92 };
static uint32_t *wp(const Cls *c, void *o, unsigned off32) {
  if (off32 < 4 || (off32 & 3) || (off32 - 4) / 4 >= (unsigned)c->words) h_die3("bad model offset 0x%x", off32);
  return (uint32_t *)((uint8_t *)o + c->f[(off32 - 4) / 4 + 1].off);
}
static uint32_t wget(const Cls *c, const void *o, unsigned off32) { return *wp(c, (void *)o, off32); }
static size_t plat_off(const Cls *c, unsigned off32) { return c->f[(off32 - 4) / 4 + 1].off; }
static void **params_slot(const Cls *c, void *o) { return (void **)((uint8_t *)o + c->f[0].off); }

/* field-wise dump in slices (a line holds at most ~100 fields) */
static void dump_obj(int show, const char *tag, const Cls *c, const void *o) {
  int i, n = c->nf, k = 0;
  char t[64];
  for (i = 0; i < n; i += 90, k++) {
    sprintf(t, "%s.%d", tag, k);
    h_dump_obj(show, t, o, c->f + i, n - i < 90 ? n - i : 90);
  }
}
/* field-wise hash of the object (the pointer slot excluded): the "snapshot" taken while a function runs */
static uint64_t snap_obj(const Cls *c, const void *o) {
  uint64_t h = 0xcbf29ce484222325ull;
  int i, j;
  for (i = 1; i < c->nf; i++) {
    const uint8_t *q = (const uint8_t *)o + c->f[i].off;
    for (j = 0; j < 4; j++) { h ^= q[j]; h *= 0x100000001b3ull; }
  }
  return h;
}
static uint64_t fnv_str(const char *s, uint64_t h) {
  for (; *s; s++) { h ^= (uint8_t)*s; h *= 0x100000001b3ull; }
  return h;
}

/* ------------------------------------------------------------------------------------------------ recorders */
enum { EV_INT = 0, EV_FLT = 1, EV_ED = 2, EV_DBG = 3 };
typedef struct {
  int type;
  char text[220];     /* key (parser calls) or the formatted text (prints) */
  const void *dst;    /* parser calls: the destination address */
  const void *parser; /* parser calls: the first argument */
  uint64_t snap;      /* object snapshot at the call */
} Ev;
#define MAXEV 1200
static Ev g_ev[MAXEV];
static int g_nev;
static const Cls *g_snap_cls;
static const void *g_snap_obj;
static void (*g_hook)(int idx, int type);
static int g_wr_mode;        /* the parser recorder stores a pseudo-random word into the destination */
static uint32_t g_salt;
static int g_lct_minus1;     /* the value written for LINE_CONNECTION_TYPE is -1 */
static unsigned g_nedp, g_ndbg, g_nvp;
static unsigned g_vpidx;      /* index of the parser call within the current event window (the written value depends on it) */

static uint32_t wval(const char *key, unsigned k, uint32_t salt) {
  uint64_t h = fnv_str(key, 0x9e3779b97f4a7c15ull ^ ((uint64_t)salt << 20) ^ k);
  uint32_t v = (uint32_t)(h ^ (h >> 32));
  if (g_lct_minus1 && !strcmp(key, "LINE_CONNECTION_TYPE")) return 0xffffffffu;
  switch (h % 11) {
    case 0: return 0;
    case 1: return 0xffffffffu;
    case 2: return 0x7fc00000u | (v & 0x3fffff);   /* a quiet NaN word (a float member holds whatever the file said) */
    default: return v;
  }
}
static void ev_add(int type, const char *text, const void *dst, const void *parser) {
  Ev *e;
  if (g_nev >= MAXEV) h_die3("too many events");
  e = &g_ev[g_nev];
  e->type = type;
  snprintf(e->text, sizeof e->text, "%s", text);
  e->dst = dst;
  e->parser = parser;
  e->snap = g_snap_obj ? snap_obj(g_snap_cls, g_snap_obj) : 0;
  g_nev++;
  if (g_hook) g_hook(g_nev - 1, type);
}
static int vp_common(int type, void *parser, const char *name, void *dst) {
  char buf[64];
  g_nvp++;
  /* the recorder's own index of this call: the value written depends on the position in the sequence */
  if (g_wr_mode) { uint32_t w = wval(name, g_vpidx, g_salt); memcpy(dst, &w, 4); }
  g_vpidx++;
  (void)buf;
  ev_add(type, name, dst, parser);
  return 0;
}
int Vparser_read_int(void *parser, const char *name, int *dst) { return vp_common(EV_INT, parser, name, dst); }
int Vparser_read_float(void *parser, const char *name, float *dst) { return vp_common(EV_FLT, parser, name, dst); }

static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
void edprintf(const char *fmt, ...) {
  char buf[300], esc[320];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  esc[0] = 0;
  esc_cat(esc, sizeof esc, buf);
  g_nedp++;
  ev_add(EV_ED, esc, NULL, NULL);
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[300], esc[320];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  esc[0] = 0;
  esc_cat(esc, sizeof esc, buf);
  g_ndbg++;
  ev_add(EV_DBG, esc, NULL, NULL);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ helpers */
static void *onew(size_t n) { void *p = h_obj_new(n); memset(p, 0xa5, n); return p; }
static const char *kname[4] = { "int", "flt", "ed", "dbg" };

static uint32_t rw(void) {
  switch (h_rmod(8)) {
    case 0: return 0;
    case 1: return 0xffffffffu;
    case 2: return 0x80000000u;
    case 3: return h_rmod(40);
    case 4: return 1u << h_rmod(32);
    default: return h_rnd();
  }
}
/* stale object content: one random word per member (a fixed number of draws: independent of sizeof), the pointer slot = p */
static void fill_obj(const Cls *c, void *o, void *p) {
  int i;
  memset(o, 0xa5, c->size);
  for (i = 1; i < c->nf; i++) { uint32_t w = rw(); memcpy((uint8_t *)o + c->f[i].off, &w, 4); }
  *params_slot(c, o) = p;
}

/* the dp runtime block (ModParObj, harness-owned, 0x100 bytes from sysdep_malloc): the fields the V90Parameters functions read */
static char g_path[] = "/etc/slmodem/v90.params";
static ModParObj *new_modpar(int with_path) {
  ModParObj *P = (ModParObj *)sysdep_malloc(0x100);
  memset(P, 0xcd, 0x100);
  P->flags0 = (uint8_t)h_rnd();
  P->upstreamMinRate = rw();
  P->upstreamMaxRate = rw();
  P->powerReduction = rw();
  P->connectionType = rw();
  P->flags50 = (uint8_t)h_rnd();
  P->path = with_path ? g_path : NULL;
  return P;
}

/* the harness' event window: arms the recorders for one call */
static void ev_begin(const Cls *c, const void *snap_obj_, int wr_mode, uint32_t salt) {
  g_nev = 0;
  g_vpidx = 0;
  g_snap_cls = c;
  g_snap_obj = snap_obj_;
  g_wr_mode = wr_mode;
  g_salt = salt;
}
static void ev_end(void) {
  g_snap_obj = NULL;
  g_snap_cls = NULL;
  g_wr_mode = 0;
  g_hook = NULL;
}
/* canonical lines for the events (hashed; sampled onto stdout) */
static void ev_emit(int show, const char *tag, const Cls *c, const void *obj) {
  int i;
  for (i = 0; i < g_nev; i++) {
    const Ev *e = &g_ev[i];
    if (e->type <= EV_FLT) {
      long off = e->dst ? (long)((const uint8_t *)e->dst - (const uint8_t *)obj) : 0;
      int k = -1, j;
      for (j = 0; j < c->nf; j++) if ((long)c->f[j].off == off) k = j;
      h_emit(show, "%s ev%d %s %s -> %s parser=%s snap=%016llx", tag, i, kname[e->type], e->text, k >= 0 ? c->f[k].name : "?",
             e->parser == g_path ? "path" : e->parser ? "other" : "NULL", (unsigned long long)e->snap);
    } else {
      h_emit(show, "%s ev%d %s %s snap=%016llx", tag, i, kname[e->type], e->text, (unsigned long long)e->snap);
    }
  }
}

/* coverage ids */
static const char *g_covname[H_COV_MAX];
static void cov_def(int id, const char *name) { h_cov_define(id, name); g_covname[id] = name; }
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }

/* ================================================================================================ scenarios 0 / 1: loadParams */
enum { CV_LP_PASSIVE = 0, CV_LP_WRITE, CV_LP_INT, CV_LP_FLT, CV_LP_PARSER_NULL, CV_LP_PARSER_OTHER, CV_LP_DUPSLOT, CV_LP_SEQ, CV_LP_NANWORD };

/* the independent model of loadParams = the blob's own call table; returns the number of calls */
typedef struct { int fl; const char *key; unsigned off; } MCall;
static void check_calls_against_model(const Cls *c, const void *o, const MCall *m, int nm, const void *parser, const char *who) {
  int i;
  if (g_nev != nm) h_die3("%s: %d parser calls, blob table has %d", who, g_nev, nm);
  for (i = 0; i < nm; i++) {
    const Ev *e = &g_ev[i];
    if (e->type != (m[i].fl ? EV_FLT : EV_INT)) h_die3("%s call %d: reader %s, table %s", who, i, kname[e->type], m[i].fl ? "flt" : "int");
    if (strcmp(e->text, m[i].key)) h_die3("%s call %d: key '%s', table '%s'", who, i, e->text, m[i].key);
    if (e->dst != (const void *)((const uint8_t *)o + plat_off(c, m[i].off))) h_die3("%s call %d (%s): destination differs from the table (+0x%x)", who, i, m[i].key, m[i].off);
    if (e->parser != parser) h_die3("%s call %d (%s): parser argument differs", who, i, m[i].key);
  }
}

static void sc_loadparams(int v92) {
  const Cls *c = v92 ? &C92 : &C90;
  const MCall *m = v92 ? (const MCall *)mdl_v92_calls : (const MCall *)mdl_v90_calls;
  const int nm = v92 ? MDL_V92_NCALLS : MDL_V90_NCALLS;
  void (*load)(void *, char *) = v92 ? v92_load : v90_load;
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_LP_PASSIVE, "passive recorder (the real stub: the object must not change)");
  cov_def(CV_LP_WRITE, "writing recorder (the destination word is stored)");
  cov_def(CV_LP_INT, "Vparser_read_int calls"); cov_def(CV_LP_FLT, "Vparser_read_float calls");
  cov_def(CV_LP_PARSER_NULL, "parser argument NULL"); cov_def(CV_LP_PARSER_OTHER, "parser argument not the path");
  cov_def(CV_LP_DUPSLOT, "two keys into one slot (the last write wins)"); cov_def(CV_LP_SEQ, "sequence compared");
  cov_def(CV_LP_NANWORD, "a NaN word written into a float member");
  h_scenario_begin(1);
  relax_reserved();
  if (v92) h_covreq[CV_LP_DUPSLOT] = 0;   /* V92Parameters has no two keys into one slot */
  if (!v92 && nm != 295) h_die3("model: %d calls", nm);
  if (v92 && nm != 54) h_die3("model: %d calls", nm);
  for (t = 0; t < (v92 ? 600u : 400u); t++) {
    ModParObj *P = new_modpar(1);
    uint8_t *o = (uint8_t *)onew(c->size), *pre = (uint8_t *)onew(c->size), *exp = (uint8_t *)onew(c->size);
    char *parser = (t % 11 == 0) ? NULL : (t % 5 == 0) ? g_path + 3 : g_path;
    int wr = h_rpct(60), i;
    uint32_t salt = h_rnd();
    fill_obj(c, o, P);
    memcpy(pre, o, c->size);
    ev_begin(c, o, wr, salt);
    load(o, parser);
    ev_end();
    check_calls_against_model(c, o, m, nm, parser, v92 ? "V92Parameters::loadParams" : "V90Parameters::loadParams");
    /* the object: unchanged (passive) or exactly the written words (the last write wins) */
    memcpy(exp, pre, c->size);
    if (wr) {
      for (i = 0; i < nm; i++) {
        uint32_t w = wval(m[i].key, (unsigned)i, salt);
        memcpy(wp(c, exp, m[i].off), &w, 4);
        if (m[i].fl && (w & 0x7f800000u) == 0x7f800000u && (w & 0x7fffff)) h_cov_hit(CV_LP_NANWORD);
      }
    }
    if (memcmp(o, exp, c->size)) h_die3("%s: object differs from the model (%s mode)", v92 ? "V92Parameters::loadParams" : "V90Parameters::loadParams", wr ? "write" : "passive");
    for (i = 0; i < g_nev; i++) h_cov_hit(g_ev[i].type == EV_INT ? CV_LP_INT : CV_LP_FLT);
    h_cov_hit(wr ? CV_LP_WRITE : CV_LP_PASSIVE);
    if (!parser) h_cov_hit(CV_LP_PARSER_NULL); else if (parser != g_path) h_cov_hit(CV_LP_PARSER_OTHER);
    if (!v92) h_cov_hit(CV_LP_DUPSLOT);
    h_cov_hit(CV_LP_SEQ);
    dump_obj(h_sample(&ctr, 3, 301), v92 ? "lp92" : "lp90", c, o);
    ev_emit(h_sample(&ctr, 2, 4999), v92 ? "lp92" : "lp90", c, o);
    for (i = 0; i < g_nev; i++) acc = acc * 1000003u + g_ev[i].snap + fnv_str(g_ev[i].text, 0);
    sysdep_free(P);
    h_trial_end("lp");
  }
  h_emit(1, "lp hash %016llx", (unsigned long long)acc);
}

/* ================================================================================================ scenario 2: V92Parameters setToDefault / C1 / C2 / init / D1 / D2 */
enum { CV_V2_DEF = 0, CV_V2_C1, CV_V2_C2, CV_V2_INIT, CV_V2_PATH, CV_V2_NOPATH, CV_V2_D1, CV_V2_D2, CV_V2_STALE, CV_V2_WRITE, CV_V2_SEQ };
static void apply_v92_defaults(const Cls *c, void *o) {
  int i;
  for (i = 0; i < MDL_V92_NDFLT; i++) memcpy(wp(c, o, mdl_v92_dflt[i].off), &mdl_v92_dflt[i].w, 4);
}
static void sc_v92(void) {
  const Cls *c = &C92;
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_V2_DEF, "setToDefault called directly"); cov_def(CV_V2_C1, "C1"); cov_def(CV_V2_C2, "C2"); cov_def(CV_V2_INIT, "init");
  cov_def(CV_V2_PATH, "params->path non-NULL: loadParams runs"); cov_def(CV_V2_NOPATH, "params->path NULL"); cov_def(CV_V2_D1, "D1"); cov_def(CV_V2_D2, "D2");
  cov_def(CV_V2_STALE, "stale object (defaults overwrite random words)"); cov_def(CV_V2_WRITE, "writing recorder after the defaults"); cov_def(CV_V2_SEQ, "sequence compared");
  h_scenario_begin(1);
  relax_reserved();
  if (MDL_V92_NDFLT != 54) h_die3("model: %d defaults", MDL_V92_NDFLT);
  for (t = 0; t < 500; t++) {
    int path = h_rpct(60), op = h_rmod(4), wr = h_rpct(60), i;
    ModParObj *P = new_modpar(path);
    uint8_t *o = (uint8_t *)onew(c->size), *pre = (uint8_t *)onew(c->size), *exp = (uint8_t *)onew(c->size);
    uint32_t salt = h_rnd();
    void *oldp = P;
    fill_obj(c, o, oldp);
    if (op != 3) *params_slot(c, o) = (void *)(uintptr_t)(0x1000 + h_rmod(0x100));   /* a stale (wrong) pointer: the ctor must store P; direct setToDefault must leave it */
    if (op == 3) *params_slot(c, o) = P;                                              /* init: the object already knows its block */
    memcpy(pre, o, c->size);
    memcpy(exp, pre, c->size);
    ev_begin(c, o, wr, salt);
    if (op == 0) { v92_def(o); h_cov_hit(CV_V2_DEF); }
    else if (op == 1) { v92_c1(o, P); h_cov_hit(CV_V2_C1); }
    else if (op == 2) { v92_c2(o, P); h_cov_hit(CV_V2_C2); }
    else { v92_init(o); h_cov_hit(CV_V2_INIT); }
    ev_end();
    /* model */
    apply_v92_defaults(c, exp);
    if (op == 1 || op == 2) *params_slot(c, exp) = P;
    if (op == 0) {
      if (g_nev != 0) h_die3("setToDefault made %d parser calls", g_nev);
    } else if (path) {
      check_calls_against_model(c, o, (const MCall *)mdl_v92_calls, MDL_V92_NCALLS, g_path, "V92Parameters ctor / init");
      if (wr) for (i = 0; i < MDL_V92_NCALLS; i++) { uint32_t w = wval(mdl_v92_calls[i].key, (unsigned)i, salt); memcpy(wp(c, exp, mdl_v92_calls[i].off), &w, 4); }
      h_cov_hit(CV_V2_PATH);
      if (wr) h_cov_hit(CV_V2_WRITE);
    } else {
      if (g_nev != 0) h_die3("V92Parameters ctor / init without a path made %d parser calls", g_nev);
      h_cov_hit(CV_V2_NOPATH);
    }
    if (memcmp(o, exp, c->size)) h_die3("V92Parameters op %d: object differs from the model", op);
    h_cov_hit(CV_V2_STALE);
    h_cov_hit(CV_V2_SEQ);
    dump_obj(h_sample(&ctr, 3, 301), "v92", c, o);
    ev_emit(h_sample(&ctr, 2, 4999), "v92", c, o);
    for (i = 0; i < g_nev; i++) acc = acc * 1000003u + g_ev[i].snap;
    /* the destructors are `ret`: the object does not change */
    memcpy(pre, o, c->size);
    if (h_rpct(50)) { v92_d1(o); h_cov_hit(CV_V2_D1); } else { v92_d2(o); h_cov_hit(CV_V2_D2); }
    if (memcmp(o, pre, c->size)) h_die3("V92Parameters destructor modified the object");
    sysdep_free(P);
    h_trial_end("v92");
  }
  h_emit(1, "v92 hash %016llx", (unsigned long long)acc);
}

/* ================================================================================================ V90Parameters: independent models */
#define O_PROBING 0x4u
#define O_LINECONN 0xcu
#define O_RATEMASK 0x48u
#define O_SILENCESCR 0x364u
#define O_DIGPOWRED 0x380u
#define O_TRN2DEVAL 0x420u
#define O_ANSPCMLEN 0x4ccu
#define O_SENSISP 0x4f8u
#define O_MAXTXIDX 0x4fcu

static const char *T_BADRATE = "V90Parameters: bad upstream rate, using default values\\n";   /* the texts are escaped like the recorder's */
static const char *T_RATES = "V90Parameters: upStream min rate : %d upStream max rate : %d  Rate mask :%x\\n";
typedef struct { unsigned lo, hi, mask; int bad; int clamped_by_isp; } RateM;
/* the rate window: lo / hi = min / max rate in units of 2400 bit/s, clamped to 2 .. 14, the mask has bit (k - 2) set for every allowed k.  Written from the
 * disassembly's meaning with TRUE division and plain loops */
static RateM model_rate(unsigned minr, unsigned maxr, unsigned f4f8, unsigned f4fc) {
  RateM r;
  unsigned k;
  r.lo = minr / 2400u;
  r.hi = maxr / 2400u;
  r.clamped_by_isp = 0;
  if (f4f8 != 0 && r.hi > f4fc) { r.hi = f4fc; r.clamped_by_isp = 1; }
  if (r.lo < 2) r.lo = 2; else if (r.lo > 14) r.lo = 14;
  if (r.hi < 2) r.hi = 2; else if (r.hi > 14) r.hi = 14;
  r.bad = r.lo > r.hi;
  if (r.bad) { r.lo = 2; r.hi = 14; }
  r.mask = 0;
  for (k = 2; k <= 14; k++) if (k >= r.lo && k <= r.hi) r.mask |= 1u << (k - 2);
  return r;
}
static void set_w(const Cls *c, void *o, unsigned off32, uint32_t w) { memcpy(wp(c, o, off32), &w, 4); }
/* the object after the head of setToDefault (everything stored before the first debug print) */
static void model_head(void *o, unsigned ratemask) {
  int i;
  for (i = 0; i < MDL_V90_NHEAD; i++) set_w(&C90, o, mdl_v90_head[i].off, mdl_v90_head[i].w);
  set_w(&C90, o, O_RATEMASK, ratemask);
}
static void model_setdefault(void *o, const RateM *r, unsigned f4f8, uint8_t b0) {
  int i;
  model_head(o, r->mask);
  for (i = 0; i < MDL_V90_NBODY; i++) set_w(&C90, o, mdl_v90_body[i].off, mdl_v90_body[i].w);
  set_w(&C90, o, O_ANSPCMLEN, (b0 & 1u) ? 0x960u : 0x1e3cu);
  if (f4f8 == 0) set_w(&C90, o, O_SILENCESCR, 1u);
}

/* the debug-level plan: after the k-th debug print the level becomes plan[k] (a recorder changing the global between the two prints of setToDefault) */
static unsigned g_plan[4];
static int g_plan_on;
static void plan_hook(int idx, int type) {
  (void)idx;
  if (type == EV_DBG && g_plan_on) { static unsigned k; (void)k; }
}
static int g_dbg_seen;
static void plan_hook2(int idx, int type) {
  (void)idx;
  if (type == EV_DBG) {
    if (g_plan_on && g_dbg_seen < 4) dsplibs_debug_level = g_plan[g_dbg_seen];
    g_dbg_seen++;
  }
}
static const unsigned LV[] = { 0, 1, 2, 3, 2, 2, 5, 0xffffffffu, 0x80000000u, 0x80000001u, 7 };
#define NLV ((unsigned)(sizeof LV / sizeof LV[0]))

static uint32_t pick_rate(void) {
  switch (h_rmod(10)) {
    case 0: return 0;
    case 1: return 2399;
    case 2: return 2400u * h_rmod(18);
    case 3: return 2400u * h_rmod(18) + 1;
    case 4: return 2400u * h_rmod(18) - 1;
    case 5: return 0xffffffffu;
    case 6: return 0x80000000u + h_rmod(5000);
    case 7: return 2400u * (1 + h_rmod(16)) + h_rmod(2400);
    default: return h_rnd();
  }
}
static uint32_t pick_f4fc(void) {
  static const uint32_t v[] = { 0, 1, 2, 3, 4, 12, 13, 14, 15, 16, 0xffffffffu, 0x80000000u, 0x80000001u, 100 };
  return h_rpct(80) ? v[h_rmod(sizeof v / sizeof v[0])] : h_rnd();
}
static uint32_t pick_f4f8(void) {
  static const uint32_t v[] = { 0, 0, 1, 0xffffffffu, 0x100u, 0x80000000u };
  return h_rpct(85) ? v[h_rmod(sizeof v / sizeof v[0])] : h_rnd();
}

/* ================================================================================================ scenario 3: V90Parameters::setToDefault */
enum { CV_SD_BAD = 0, CV_SD_OK, CV_SD_LOCLAMP, CV_SD_HICLAMP, CV_SD_ISP, CV_SD_ISP_NOCLAMP_UNSIGNED, CV_SD_ISP0, CV_SD_PRINT1, CV_SD_PRINT2, CV_SD_NOPRINT, CV_SD_LVL_CUT, CV_SD_BIGLVL,
       CV_SD_B0_1, CV_SD_B0_0, CV_SD_B0_ODD, CV_SD_364_SET, CV_SD_364_KEEP, CV_SD_FULL, CV_SD_SNAP, CV_SD_LOW_EQ_HI, CV_SD_ONE_BIT, CV_SD_MASK_FULL, CV_SD_DIVEDGE, CV_SD_STALE_4F8 };
static void sc_v90_default(void) {
  const Cls *c = &C90;
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_SD_BAD, "lo > hi: bad upstream rate (defaults 2 .. 14)"); cov_def(CV_SD_OK, "lo <= hi"); cov_def(CV_SD_LOCLAMP, "lo clamped up to 2"); cov_def(CV_SD_HICLAMP, "hi clamped down to 14");
  cov_def(CV_SD_ISP, "sensitive ISP: hi clamped to maxTxRateIndexForSensitiveIsp"); cov_def(CV_SD_ISP_NOCLAMP_UNSIGNED, "sensitive ISP with a limit >= 2^31 (unsigned compare: no clamp)");
  cov_def(CV_SD_ISP0, "sensitiveIspDetected == 0"); cov_def(CV_SD_PRINT1, "first print (bad rate)"); cov_def(CV_SD_PRINT2, "rate-mask print"); cov_def(CV_SD_NOPRINT, "level <= 1 (no print)");
  cov_def(CV_SD_LVL_CUT, "level lowered between the two prints"); cov_def(CV_SD_BIGLVL, "level >= 2^31 (unsigned compare)"); cov_def(CV_SD_B0_1, "flags0 bit 0 set"); cov_def(CV_SD_B0_0, "flags0 bit 0 clear");
  cov_def(CV_SD_B0_ODD, "flags0 with other bits set"); cov_def(CV_SD_364_SET, "+0x364 stored (sensitive ISP flag 0)"); cov_def(CV_SD_364_KEEP, "+0x364 keeps its value (flag != 0)");
  cov_def(CV_SD_FULL, "mask 2 .. 14 (all 13 bits)"); cov_def(CV_SD_SNAP, "snapshots compared"); cov_def(CV_SD_LOW_EQ_HI, "lo == hi"); cov_def(CV_SD_ONE_BIT, "mask with one bit");
  cov_def(CV_SD_MASK_FULL, "mask 0x1fff"); cov_def(CV_SD_DIVEDGE, "rates on / next to a multiple of 2400"); cov_def(CV_SD_STALE_4F8, "stale +0x364 non-zero and flag != 0");
  h_scenario_begin(1);
  relax_reserved();
  if (MDL_V90_NHEAD != 17 || MDL_V90_NBODY != 319) h_die3("model: head %d body %d", MDL_V90_NHEAD, MDL_V90_NBODY);
  for (t = 0; t < 6000; t++) {
    ModParObj *P = new_modpar(h_rpct(50));
    uint8_t *o = (uint8_t *)onew(c->size), *pre = (uint8_t *)onew(c->size), *exp = (uint8_t *)onew(c->size), *sn = (uint8_t *)onew(c->size);
    unsigned lvl0 = LV[h_rmod(NLV)];
    unsigned f4f8, f4fc;
    RateM r;
    int i, evi = 0;
    P->upstreamMinRate = pick_rate();
    P->upstreamMaxRate = h_rpct(15) ? P->upstreamMinRate : pick_rate();
    if (h_rpct(25)) { P->upstreamMinRate = 2400u * (2 + h_rmod(14)) + (h_rpct(30) ? 2399u : 0u); P->upstreamMaxRate = 2400u * (2 + h_rmod(14)) + (h_rpct(30) ? 2399u : 0u); }
    P->flags0 = h_rpct(50) ? (uint8_t)(h_rnd() & 1) : (uint8_t)h_rnd();
    fill_obj(c, o, P);
    f4f8 = pick_f4f8(); f4fc = pick_f4fc();
    set_w(c, o, O_SENSISP, f4f8); set_w(c, o, O_MAXTXIDX, f4fc);
    if (h_rpct(30)) set_w(c, o, O_SILENCESCR, 0);
    if (h_rpct(30)) set_w(c, o, O_RATEMASK, 0xffffffffu);
    memcpy(pre, o, c->size);
    g_plan_on = h_rpct(50);
    for (i = 0; i < 4; i++) g_plan[i] = LV[h_rmod(NLV)];
    g_dbg_seen = 0;
    dsplibs_debug_level = lvl0;
    ev_begin(c, o, 0, 0);
    g_hook = plan_hook2;
    v90_def(o);
    ev_end();
    dsplibs_debug_level = 0;
    /* model */
    r = model_rate(P->upstreamMinRate, P->upstreamMaxRate, f4f8, f4fc);
    memcpy(exp, pre, c->size);
    model_setdefault(exp, &r, f4f8, P->flags0);
    if (memcmp(o, exp, c->size)) h_die3("V90Parameters::setToDefault: object differs from the model");
    {
      unsigned lvl = lvl0, nexp = 0;
      Ev ex[2];
      char tx[2][300];
      uint64_t sh[2];
      int ndbg = 0;
      /* the prints and what the object holds when they happen */
      memcpy(sn, pre, c->size);
      model_head(sn, 0);
      sh[0] = snap_obj(c, sn);
      model_head(sn, r.mask);
      sh[1] = snap_obj(c, sn);
      if (r.bad && lvl > 1) {
        snprintf(tx[nexp], sizeof tx[0], "%s", T_BADRATE);
        ex[nexp].snap = sh[0];
        nexp++;
        if (g_plan_on) lvl = g_plan[0];
        h_cov_hit(CV_SD_PRINT1);
        if (g_plan_on) h_cov_hit(CV_SD_LVL_CUT);
      }
      if (lvl > 1) {
        snprintf(tx[nexp], sizeof tx[0], T_RATES, (int)(r.lo * 2400u), (int)(r.hi * 2400u), r.mask);
        ex[nexp].snap = sh[1];
        nexp++;
        h_cov_hit(CV_SD_PRINT2);
      } else if (!r.bad || lvl0 <= 1) h_cov_hit(CV_SD_NOPRINT);
      if (g_nev != (int)nexp) h_die3("V90Parameters::setToDefault: %d prints, model %u (level %u, bad %d, plan %d)", g_nev, nexp, lvl0, r.bad, g_plan_on);
      for (i = 0; i < g_nev; i++) {
        char esc[320];
        esc[0] = 0;
        esc_cat(esc, sizeof esc, tx[i]);   /* the model text still holds its escapes as typed: unescaped in the strings above */
        (void)esc;
        if (g_ev[i].type != EV_DBG) h_die3("print %d is not a debug print", i);
        { char want[320]; want[0] = 0; esc_cat(want, sizeof want, tx[i]); (void)want; }
        if (strcmp(g_ev[i].text, tx[i]) && 1) h_die3("print %d: '%s', model '%s'", i, g_ev[i].text, tx[i]);
        if (g_ev[i].snap != ex[i].snap) h_die3("print %d: the object snapshot at the print differs from the model (store order)", i);
        h_cov_hit(CV_SD_SNAP);
        ndbg++;
      }
      (void)ndbg;
    }
    if (r.bad) h_cov_hit(CV_SD_BAD); else h_cov_hit(CV_SD_OK);
    if (P->upstreamMinRate / 2400u < 2) h_cov_hit(CV_SD_LOCLAMP);
    if (P->upstreamMaxRate / 2400u > 14 && !(f4f8 != 0 && P->upstreamMaxRate / 2400u > f4fc)) h_cov_hit(CV_SD_HICLAMP);
    if (r.clamped_by_isp) h_cov_hit(CV_SD_ISP);
    if (f4f8 != 0 && f4fc >= 0x80000000u && P->upstreamMaxRate / 2400u > 0) h_cov_hit(CV_SD_ISP_NOCLAMP_UNSIGNED);
    if (f4f8 == 0) { h_cov_hit(CV_SD_ISP0); h_cov_hit(CV_SD_364_SET); } else { h_cov_hit(CV_SD_364_KEEP); if (wget(c, pre, O_SILENCESCR) != 1) h_cov_hit(CV_SD_STALE_4F8); }
    if (lvl0 >= 0x80000000u) h_cov_hit(CV_SD_BIGLVL);
    if (P->flags0 & 1) h_cov_hit(CV_SD_B0_1); else h_cov_hit(CV_SD_B0_0);
    if (P->flags0 & 0xfe) h_cov_hit(CV_SD_B0_ODD);
    if (r.mask == 0x1fff) { h_cov_hit(CV_SD_FULL); h_cov_hit(CV_SD_MASK_FULL); }
    if (r.lo == r.hi) h_cov_hit(CV_SD_LOW_EQ_HI);
    if ((r.mask & (r.mask - 1)) == 0) h_cov_hit(CV_SD_ONE_BIT);
    if (P->upstreamMinRate % 2400u == 0 || P->upstreamMinRate % 2400u == 2399 || P->upstreamMaxRate % 2400u == 0 || P->upstreamMaxRate % 2400u == 2399) h_cov_hit(CV_SD_DIVEDGE);
    dump_obj(h_sample(&ctr, 3, 601), "sd90", c, o);
    ev_emit(h_sample(&ctr, 3, 1499), "sd90", c, o);
    for (i = 0; i < g_nev; i++) acc = acc * 1000003u + g_ev[i].snap + fnv_str(g_ev[i].text, 0);
    (void)evi;
    sysdep_free(P);
    h_trial_end("sd90");
  }
  h_emit(1, "sd90 hash %016llx", (unsigned long long)acc);
}

/* ================================================================================================ loadModemParamsData: independent model (pure integer arithmetic) */
typedef struct { int type; char text[300]; uint64_t snap; } MEv;
static MEv g_mev[1300];
static int g_nmev;
static void mev_add(int type, uint64_t snap, const char *fmt, ...) {
  va_list ap;
  MEv *e = &g_mev[g_nmev++];
  va_start(ap, fmt);
  e->type = type;
  vsnprintf(e->text, sizeof e->text, fmt, ap);
  va_end(ap);
  e->snap = snap;
}
static void compare_events(const char *who) {
  int i;
  if (g_nev != g_nmev) {
    for (i = 0; i < g_nev; i++) fprintf(stderr, "  got %d: %s %s\n", i, kname[g_ev[i].type], g_ev[i].text);
    for (i = 0; i < g_nmev; i++) fprintf(stderr, "  exp %d: %s %s\n", i, kname[g_mev[i].type], g_mev[i].text);
    h_die3("%s: %d events, model %d", who, g_nev, g_nmev);
  }
  for (i = 0; i < g_nev; i++) {
    if (g_ev[i].type != g_mev[i].type) h_die3("%s event %d: type %s, model %s", who, i, kname[g_ev[i].type], kname[g_mev[i].type]);
    if (strcmp(g_ev[i].text, g_mev[i].text)) h_die3("%s event %d: '%s', model '%s'", who, i, g_ev[i].text, g_mev[i].text);
    if (g_ev[i].snap != g_mev[i].snap) h_die3("%s event %d (%s): object snapshot differs from the model", who, i, g_ev[i].text);
  }
}
/* loadModemParamsData on the model object `exp`; the dp block read is `Pa` / `Pb` as selected by sel[k] after the k-th print (the recorder swaps this->params) */
static unsigned g_lm_flags;   /* coverage bits of the last model run */
enum { LMF_PR0 = 1, LMF_PRSMALL = 2, LMF_PRBIG = 4, LMF_PROBE = 8, LMF_CTM1 = 16, LMF_CTKEEP = 32, LMF_FRAC = 64, LMF_ROUND = 128, LMF_SWAPUSED = 256 };
static void model_lmpd(uint8_t *exp, const ModParObj *Pa, const ModParObj *Pb, const unsigned char *sel) {
  const Cls *c = &C90;
  const ModParObj *cur = Pa;
  int k = 0;
  unsigned pr, probe, ct, e;
  g_lm_flags = 0;
  pr = cur->powerReduction;
  mev_add(EV_ED, snap_obj(c, exp), "V90Parameters: Debug - tempPR = %d\\r\\n", (int)pr);
  if (sel[k++]) { cur = Pb; g_lm_flags |= LMF_SWAPUSED; } else cur = Pa;
  if (pr != 0) {
    unsigned q = pr / 5u;
    double v = (double)q / 2.0;                 /* exact */
    volatile float vf = (float)v;               /* the member is the float rounding of v (q / 2 needs up to 30 bits) */
    uint32_t w;
    int sign = (q == 0) ? '-' : '+';
    memcpy(&w, (const void *)&vf, 4);
    set_w(c, exp, O_DIGPOWRED, w);
    mev_add(EV_ED, snap_obj(c, exp), "V90Parameters: setting power reduction to  = %c%d.%02d\\r\\n", sign, (int)(q / 2u), (int)((q & 1u) ? 50 : 0));
    if (sel[k++]) { cur = Pb; g_lm_flags |= LMF_SWAPUSED; } else cur = Pa;
    if (q == 0) g_lm_flags |= LMF_PRSMALL;
    if (q > (1u << 25)) g_lm_flags |= LMF_PRBIG;
    if (q & 1u) g_lm_flags |= LMF_FRAC;
    if (q > (1u << 25) && (q & 1u) && (float)v != (float)(v - 0.5)) g_lm_flags |= LMF_ROUND;
  } else g_lm_flags |= LMF_PR0;
  probe = (cur->flags50 >> 1) & 1u;
  mev_add(EV_ED, snap_obj(c, exp), "V90Parameters: Debug - tempProbe = %d\\r\\n", (int)probe);
  if (sel[k++]) { cur = Pb; g_lm_flags |= LMF_SWAPUSED; } else cur = Pa;
  if (probe) { set_w(c, exp, O_PROBING, probe); g_lm_flags |= LMF_PROBE; }
  ct = cur->connectionType;
  mev_add(EV_ED, snap_obj(c, exp), "V90Parameters: Debug - tempConnectionType = %d\\r\\n", (int)ct);
  if (sel[k++]) { cur = Pb; g_lm_flags |= LMF_SWAPUSED; } else cur = Pa;
  if (wget(c, exp, O_LINECONN) == 0xffffffffu) { set_w(c, exp, O_LINECONN, ct); g_lm_flags |= LMF_CTM1; } else g_lm_flags |= LMF_CTKEEP;
  e = cur->flags50 & 1u;
  set_w(c, exp, O_TRN2DEVAL, e);
  mev_add(EV_ED, snap_obj(c, exp), "V90Parameters: Debug - trn2d_mean_error_std_evaluation_enable = %d\\r\\n", (int)e);
}

/* ================================================================================================ scenario 4: V90Parameters::loadModemParamsData */
enum { CV_LM_PR0 = 0, CV_LM_PRSMALL, CV_LM_PRBIG, CV_LM_FRAC, CV_LM_ROUND, CV_LM_PROBE, CV_LM_PROBE0, CV_LM_CTM1, CV_LM_CTKEEP, CV_LM_SWAP, CV_LM_F50_BIT0, CV_LM_F50_BIT1, CV_LM_F50_HI, CV_LM_CT_NEG,
       CV_LM_PRMAX, CV_LM_PRHALF, CV_LM_THIS4, CV_LM_SNAP };
static ModParObj *g_lm_pa, *g_lm_pb;
static unsigned char g_lm_sel[8];
static int g_lm_edseen;
static void *g_lm_obj;
static void lm_hook(int idx, int type) {
  (void)idx;
  if (type == EV_ED) {
    if (g_lm_edseen < 8 && g_lm_obj) *params_slot(&C90, g_lm_obj) = g_lm_sel[g_lm_edseen] ? (void *)g_lm_pb : (void *)g_lm_pa;
    g_lm_edseen++;
  }
}
static uint32_t pick_pr(void) {
  static const uint32_t v[] = { 0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 14, 15, 99, 100, 101, 5u << 24, (5u << 25) + 5u, (5u << 25) + 10u, (5u << 25) - 5u, 0xffffffffu, 0x80000000u, 0x7fffffffu,
                                0xccccccccu, 0xfffffffbu, 0xfffffffau, 4294967290u, 0x40000000u, 3u << 29, 0x1999999au };
  switch (h_rmod(5)) {
    case 0: return v[h_rmod(sizeof v / sizeof v[0])];
    case 1: return 5u * h_rmod(2000) + h_rmod(5);
    case 2: return 5u * (1u << (10 + h_rmod(21))) + h_rmod(10) - 3u;
    default: return h_rpct(50) ? h_rmod(100000) : h_rnd();
  }
}
static void sc_v90_lmpd(void) {
  const Cls *c = &C90;
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_LM_PR0, "powerReduction == 0 (no power print)"); cov_def(CV_LM_PRSMALL, "powerReduction 1 .. 4 (q == 0: '-0.00')"); cov_def(CV_LM_PRBIG, "q > 2^25 (the float member is rounded)");
  cov_def(CV_LM_FRAC, "odd q (.50)"); cov_def(CV_LM_ROUND, "float rounding changes the value"); cov_def(CV_LM_PROBE, "probe bit set: probingMode stored"); cov_def(CV_LM_PROBE0, "probe bit clear");
  cov_def(CV_LM_CTM1, "lineConnectionType == -1: replaced"); cov_def(CV_LM_CTKEEP, "lineConnectionType kept"); cov_def(CV_LM_SWAP, "this->params swapped between the prints (re-read order)");
  cov_def(CV_LM_F50_BIT0, "flags50 bit 0"); cov_def(CV_LM_F50_BIT1, "flags50 bit 1"); cov_def(CV_LM_F50_HI, "flags50 upper bits"); cov_def(CV_LM_CT_NEG, "connectionType >= 2^31 / -1");
  cov_def(CV_LM_PRMAX, "powerReduction >= 2^31"); cov_def(CV_LM_PRHALF, "q odd and small"); cov_def(CV_LM_THIS4, "stale probingMode kept (probe 0)"); cov_def(CV_LM_SNAP, "snapshots compared");
  h_scenario_begin(1);
  relax_reserved();
  for (t = 0; t < 8000; t++) {
    ModParObj *Pa = new_modpar(0), *Pb = new_modpar(0);
    uint8_t *o = (uint8_t *)onew(c->size), *pre = (uint8_t *)onew(c->size), *exp = (uint8_t *)onew(c->size);
    int i;
    Pa->powerReduction = pick_pr(); Pb->powerReduction = pick_pr();
    if (h_rpct(30)) Pa->flags50 = (uint8_t)(h_rnd() & 3);
    if (h_rpct(30)) Pa->connectionType = h_rpct(50) ? 0xffffffffu : 0x80000000u;
    fill_obj(c, o, Pa);
    if (h_rpct(55)) set_w(c, o, O_LINECONN, 0xffffffffu);
    memcpy(pre, o, c->size);
    memcpy(exp, o, c->size);
    for (i = 0; i < 8; i++) g_lm_sel[i] = h_rpct(60) ? 0 : (unsigned char)h_rpct(50);
    g_lm_pa = Pa; g_lm_pb = Pb; g_lm_edseen = 0; g_lm_obj = o;
    ev_begin(c, o, 0, 0);
    g_hook = lm_hook;
    v90_lmpd(o);
    ev_end();
    g_lm_obj = NULL;
    g_nmev = 0;
    model_lmpd(exp, Pa, Pb, g_lm_sel);
    compare_events("V90Parameters::loadModemParamsData");
    h_cov_hit(CV_LM_SNAP);
    /* the object: the model's stores, params slot = the block selected by the last print */
    *params_slot(c, exp) = *params_slot(c, o);
    if (memcmp(o, exp, c->size)) h_die3("V90Parameters::loadModemParamsData: object differs from the model");
    if (g_lm_flags & LMF_PR0) h_cov_hit(CV_LM_PR0);
    if (g_lm_flags & LMF_PRSMALL) h_cov_hit(CV_LM_PRSMALL);
    if (g_lm_flags & LMF_PRBIG) h_cov_hit(CV_LM_PRBIG);
    if (g_lm_flags & LMF_FRAC) h_cov_hit(CV_LM_FRAC);
    if (g_lm_flags & LMF_ROUND) h_cov_hit(CV_LM_ROUND);
    if (g_lm_flags & LMF_PROBE) h_cov_hit(CV_LM_PROBE); else { h_cov_hit(CV_LM_PROBE0); if (wget(c, pre, O_PROBING) != 0) h_cov_hit(CV_LM_THIS4); }
    if (g_lm_flags & LMF_CTM1) h_cov_hit(CV_LM_CTM1);
    if (g_lm_flags & LMF_CTKEEP) h_cov_hit(CV_LM_CTKEEP);
    if (g_lm_flags & LMF_SWAPUSED) h_cov_hit(CV_LM_SWAP);
    if (Pa->flags50 & 1 || Pb->flags50 & 1) h_cov_hit(CV_LM_F50_BIT0);
    if (Pa->flags50 & 2 || Pb->flags50 & 2) h_cov_hit(CV_LM_F50_BIT1);
    if (Pa->flags50 & 0xfc || Pb->flags50 & 0xfc) h_cov_hit(CV_LM_F50_HI);
    if (Pa->connectionType >= 0x80000000u || Pb->connectionType >= 0x80000000u) h_cov_hit(CV_LM_CT_NEG);
    if (Pa->powerReduction >= 0x80000000u) h_cov_hit(CV_LM_PRMAX);
    if (Pa->powerReduction != 0 && (Pa->powerReduction / 5u) < 100 && ((Pa->powerReduction / 5u) & 1u)) h_cov_hit(CV_LM_PRHALF);
    dump_obj(h_sample(&ctr, 3, 601), "lm90", c, o);
    ev_emit(h_sample(&ctr, 3, 1499), "lm90", c, o);
    for (i = 0; i < g_nev; i++) acc = acc * 1000003u + g_ev[i].snap + fnv_str(g_ev[i].text, 0);
    sysdep_free(Pa); sysdep_free(Pb);
    h_trial_end("lm90");
  }
  h_emit(1, "lm90 hash %016llx", (unsigned long long)acc);
}

/* ================================================================================================ scenario 5: V90Parameters C1 / C2 / init / initSession / D1 / D2 (integrated event sequences) */
enum { CV_V9_C1 = 0, CV_V9_C2, CV_V9_INIT, CV_V9_ISESS, CV_V9_PATH, CV_V9_NOPATH, CV_V9_WRITE, CV_V9_LCT_M1, CV_V9_BADRATE, CV_V9_LVLCUT, CV_V9_D1, CV_V9_D2, CV_V9_ISP, CV_V9_STALE4FC, CV_V9_SNAP, CV_V9_SEQ };
static void sc_v90_ctor(void) {
  const Cls *c = &C90;
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_V9_C1, "C1"); cov_def(CV_V9_C2, "C2"); cov_def(CV_V9_INIT, "init"); cov_def(CV_V9_ISESS, "initSession"); cov_def(CV_V9_PATH, "path non-NULL: loadParams runs");
  cov_def(CV_V9_NOPATH, "path NULL"); cov_def(CV_V9_WRITE, "writing recorder"); cov_def(CV_V9_LCT_M1, "LINE_CONNECTION_TYPE read as -1 (replaced by loadModemParamsData)"); cov_def(CV_V9_BADRATE, "bad-rate print");
  cov_def(CV_V9_LVLCUT, "level lowered between the prints"); cov_def(CV_V9_D1, "D1"); cov_def(CV_V9_D2, "D2"); cov_def(CV_V9_ISP, "init with a sensitive-ISP limit (clamps the rate)");
  cov_def(CV_V9_STALE4FC, "init reads stale +0x4f8 / +0x4fc"); cov_def(CV_V9_SNAP, "snapshots compared"); cov_def(CV_V9_SEQ, "sequence compared");
  h_scenario_begin(1);
  relax_reserved();
  for (t = 0; t < 2500; t++) {
    int path = h_rpct(60), op = h_rmod(4), wr = h_rpct(60), i;
    ModParObj *P = new_modpar(path);
    uint8_t *o = (uint8_t *)onew(c->size), *pre = (uint8_t *)onew(c->size), *exp = (uint8_t *)onew(c->size), *sn = (uint8_t *)onew(c->size);
    uint32_t salt = h_rnd();
    unsigned lvl0 = LV[h_rmod(NLV)], lvl;
    unsigned f4f8, f4fc;
    RateM r;
    const unsigned char sel0[8] = { 0 };
    P->upstreamMinRate = pick_rate();
    P->upstreamMaxRate = h_rpct(20) ? P->upstreamMinRate : pick_rate();
    if (h_rpct(30)) { P->upstreamMinRate = 2400u * (2 + h_rmod(14)); P->upstreamMaxRate = 2400u * (2 + h_rmod(14)); }
    P->flags0 = (uint8_t)h_rnd();
    P->powerReduction = h_rpct(30) ? 0 : pick_pr();
    if (h_rpct(30)) P->flags50 = (uint8_t)(h_rnd() & 3);
    fill_obj(c, o, h_rpct(50) ? (void *)P : (void *)(uintptr_t)0x2000);
    g_lct_minus1 = h_rpct(35);
    if (op == 2 || op == 3) *params_slot(c, o) = P;           /* init / initSession: the object already knows its block */
    memcpy(pre, o, c->size);
    memcpy(exp, o, c->size);
    g_plan_on = h_rpct(50);
    for (i = 0; i < 4; i++) g_plan[i] = LV[h_rmod(NLV)];
    g_dbg_seen = 0;
    dsplibs_debug_level = lvl0;
    ev_begin(c, o, wr, salt);
    g_hook = plan_hook2;
    if (op == 0) { v90_c1(o, P); h_cov_hit(CV_V9_C1); }
    else if (op == 1) { v90_c2(o, P); h_cov_hit(CV_V9_C2); }
    else if (op == 2) { v90_init(o); h_cov_hit(CV_V9_INIT); }
    else { v90_isess(o); h_cov_hit(CV_V9_ISESS); }
    ev_end();
    dsplibs_debug_level = 0;
    /* ---- the model ---- */
    g_nmev = 0;
    if (op == 3) {
      set_w(c, exp, O_SENSISP, 0);
      set_w(c, exp, O_MAXTXIDX, 14);
      if (g_nev != 0) h_die3("initSession made %d events", g_nev);
    } else {
      if (op == 0 || op == 1) { set_w(c, exp, O_SENSISP, 0); *params_slot(c, exp) = P; set_w(c, exp, O_MAXTXIDX, 14); }
      f4f8 = wget(c, exp, O_SENSISP);
      f4fc = wget(c, exp, O_MAXTXIDX);
      r = model_rate(P->upstreamMinRate, P->upstreamMaxRate, f4f8, f4fc);
      /* setToDefault: the prints (with the object holding the head) */
      lvl = lvl0;
      memcpy(sn, exp, c->size);
      model_head(sn, 0);
      {
        uint64_t s0 = snap_obj(c, sn), s1;
        model_head(sn, r.mask);
        s1 = snap_obj(c, sn);
        if (r.bad && lvl > 1) {
          mev_add(EV_DBG, s0, "%s", T_BADRATE);
          if (g_plan_on) { lvl = g_plan[0]; h_cov_hit(CV_V9_LVLCUT); }
          h_cov_hit(CV_V9_BADRATE);
        }
        if (lvl > 1) mev_add(EV_DBG, s1, T_RATES, (int)(r.lo * 2400u), (int)(r.hi * 2400u), r.mask);
      }
      model_setdefault(exp, &r, f4f8, P->flags0);
      if (path) {
        /* loadParams: the parser calls, each seeing the object after the writes so far */
        for (i = 0; i < MDL_V90_NCALLS; i++) {
          if (wr) { uint32_t w = wval(mdl_v90_calls[i].key, (unsigned)i, salt); set_w(c, exp, mdl_v90_calls[i].off, w); }
          mev_add(mdl_v90_calls[i].fl ? EV_FLT : EV_INT, snap_obj(c, exp), "%s", mdl_v90_calls[i].key);
        }
        h_cov_hit(CV_V9_PATH);
        if (wr) h_cov_hit(CV_V9_WRITE);
      } else h_cov_hit(CV_V9_NOPATH);
      /* loadModemParamsData */
      if (wget(c, exp, O_LINECONN) == 0xffffffffu) h_cov_hit(CV_V9_LCT_M1);
      model_lmpd(exp, P, P, sel0);
      if (op == 2 && f4f8 != 0 && r.clamped_by_isp) h_cov_hit(CV_V9_ISP);
      if (op == 2) h_cov_hit(CV_V9_STALE4FC);
      /* events: the parser events also compare the destination; compare_events compares type / text / snapshot, the destination is checked below */
      {
        int k = 0;
        for (i = 0; i < g_nev; i++) {
          if (g_ev[i].type <= EV_FLT) {
            unsigned off = mdl_v90_calls[k].off;
            if (g_ev[i].dst != (const void *)(o + plat_off(c, off)) || g_ev[i].parser != g_path) h_die3("ctor / init: parser call %d: destination / parser argument differs", k);
            k++;
          }
        }
      }
      compare_events("V90Parameters ctor / init");
      h_cov_hit(CV_V9_SNAP);
    }
    if (memcmp(o, exp, c->size)) {
      for (i = 1; i < c->nf; i++) if (memcmp(o + c->f[i].off, exp + c->f[i].off, 4)) { fprintf(stderr, "word %s (+0x%x): got %08x model %08x\n", c->f[i].name, 4 * i, *(uint32_t *)(o + c->f[i].off), *(uint32_t *)(exp + c->f[i].off)); break; }
      h_die3("V90Parameters op %d: object differs from the model (path %d, write %d)", op, path, wr);
    }
    h_cov_hit(CV_V9_SEQ);
    dump_obj(h_sample(&ctr, 3, 301), "v90", c, o);
    ev_emit(h_sample(&ctr, 2, 4999), "v90", c, o);
    for (i = 0; i < g_nev; i++) acc = acc * 1000003u + g_ev[i].snap + fnv_str(g_ev[i].text, 0);
    g_lct_minus1 = 0;
    memcpy(pre, o, c->size);
    if (h_rpct(50)) { v90_d1(o); h_cov_hit(CV_V9_D1); } else { v90_d2(o); h_cov_hit(CV_V9_D2); }
    if (memcmp(o, pre, c->size)) h_die3("V90Parameters destructor modified the object");
    sysdep_free(P);
    h_trial_end("v90");
  }
  h_emit(1, "v90 hash %016llx", (unsigned long long)acc);
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P07_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  p07_layout_check();
  h_x87_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_loadparams(1); break;
    case 1: sc_loadparams(0); break;
    case 2: sc_v92(); break;
    case 3: sc_v90_default(); break;
    case 4: sc_v90_lmpd(); break;
    case 5: sc_v90_ctor(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu parser %u edprintf %u dbgprintf %u", h_nmalloc, h_nfree, g_nvp, g_nedp, g_ndbg);
  h_cov_finish();
  return 0;
}
