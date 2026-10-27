/*
 * R05 differential harness (1 of 3): the V.22 / V.22bis PPS (transmit pulse shaper) and MRF (receive sample-rate converter) - V22_PPS_init / _free / _filter and
 * V22_MRF_init / _free / _filter - real dsplibs.o vs src/v22/v22_pulse_shaping.c + src/v22/v22_rx_resampler.c (+ src/v22/v22_tables.c: the PPS / MRF coefficient tables and the cfg blocks).
 * Library test/r04_common.h (hashed canonical output, deterministic heap with red zones, sysdep_malloc / _free / _memcpy recorders, coverage counters exit 2, 3 = harness
 * violation); the typed objects of src/v22/v22.h on both sides, the 32-bit BLOB run is the x86-64 golden.  No other V.22 function is called by these six (no callee recorder
 * besides the allocator / memcpy).  The engine's buffers (PPS / MRF coefficient buffers) are harness arrays, as V22FP_create's malloc()ed ones.
 *
 * Scenarios (each its own process):
 *   0 PPS lifecycle: init (allocate / no allocation) over garbage-filled objects with coefficient buffers of every kind (the real PPSv22_COFFS, random, extremes) and cfg words
 *     (c0 incl. step offsets); every field, allocation order / sizes, the reordered coefficient buffers vs an independent model, histories cleared, free order, double init (leak);
 *   1 PPS filter over randomly poked state (flag, phase 0 .. 0x27, fill 0 .. 5, step offset, rd, lim) and call sizes 0 .. 60, a symbol stream with garbage high bytes, maps of 256 entries;
 *   2 PPS end to end: the real maps and coefficients, long symbol streams, calls of every size, one-sample call loops; flag 2 / -1 / 5 targeted (the n -= flag arithmetic);
 *   3 MRF lifecycle (as 0) and MRF filter over poked state (step 1 .. 6, phase 0 .. 8, wpos 0 .. 0x3c) with call sizes 0 .. 90, history shifts, partial calls;
 *   4 MRF end to end: the real coefficients, long streams (tones, noise, saturation, random), call sizes 1 / 2 / 3 / 7 / 40 / random, the 20 / 9 ratio.
 */
#include "../r04_common.h"
#include "v22/v22.h"
#include "v22/v22_tables.h"

enum {
  CV_P_INIT_A = 0, CV_P_INIT_N, CV_P_FREE, CV_P_DBL, CV_P_MODEL, CV_P_FLAG, CV_P_NOFLAG, CV_P_FILL_LOW, CV_P_FILL_HIGH, CV_P_SHIFT, CV_P_ZERO_N, CV_P_WRAP_PH, CV_P_RD_WRAP,
  CV_P_MANY, CV_P_ALT_PATH, CV_P_MAIN_PATH, CV_P_STEP_OFF, CV_P_FLAG2, CV_P_NEGFLAG, CV_P_END_FLAG, CV_P_HIBYTE, CV_P_LIM_ONE, CV_P_ONE_CALL,
  CV_M_INIT_A = 40, CV_M_INIT_N, CV_M_FREE, CV_M_DBL, CV_M_MODEL, CV_M_PARTIAL, CV_M_FULL, CV_M_SHIFT_FULL, CV_M_SHIFT_PART, CV_M_PATH_A, CV_M_PATH_B, CV_M_STEP2, CV_M_STEP3,
  CV_M_STEPOTHER, CV_M_ZERO_N, CV_M_MANY_OUT, CV_M_PH_MAX, CV_M_ONE, CV_M_NEG_PH_UNUSED, CV_M_MULTI, CV_M_SAT, CV_M_E2E
};

/* the engine's coefficient buffers sit inside guarded arrays: 8 sentinel shorts in front of and behind the used part, checked after every init (a loop bound error would write them) */
static short g_cIx[8 + 0x78 + 8], g_cQx[8 + 0x78 + 8], g_mcx[8 + 0x10e + 8];
#define g_cI (g_cIx + 8)
#define g_cQ (g_cQx + 8)
#define g_mc (g_mcx + 8)
static void guards_set(void)
{
  int i;
  for (i = 0; i < 8; i++) {
    g_cIx[i] = g_cIx[8 + 0x78 + i] = 0x5a5a; g_cQx[i] = g_cQx[8 + 0x78 + i] = 0x5a5b; g_mcx[i] = g_mcx[8 + 0x10e + i] = 0x5a5c;
  }
}
static void guards_check(const char *what)
{
  int i;
  for (i = 0; i < 8; i++)
    if (g_cIx[i] != 0x5a5a || g_cIx[8 + 0x78 + i] != 0x5a5a || g_cQx[i] != 0x5a5b || g_cQx[8 + 0x78 + i] != 0x5a5b || g_mcx[i] != 0x5a5c || g_mcx[8 + 0x10e + i] != 0x5a5c)
      die3("%s wrote outside the coefficient buffer", what);
}
static V22Pps g_p;
static V22Mrf g_m;
static V22Circ g_c;
static unsigned short g_sym[64];
static short g_imap[256], g_qmap[256];
static short g_out[70000];
static short g_in[8192 + 64];

static void dump_pps(int show, const char *tag, const V22Pps *p, int deep)
{
  char a[40], b[40];
  ptr_desc(a, sizeof a, p->histI);
  ptr_desc(b, sizeof b, p->histQ);
  emit(show, "%s c0=%08x w3=%d flag=%d ph=%d fill=%d taps=%d hI=%s hQ=%s", tag, (unsigned)p->cfg.c0.w, p->cfg.w3, p->flag, p->phase, p->fill, p->taps, a, b);
  if (deep && p->histI && p->histQ) {
    emit(show, "%s H hI=%d,%d,%d,%d,%d,%d hQ=%d,%d,%d,%d,%d,%d c1=%016llx c2=%016llx", tag, p->histI[0], p->histI[1], p->histI[2], p->histI[3], p->histI[4], p->histI[5], p->histQ[0],
         p->histQ[1], p->histQ[2], p->histQ[3], p->histQ[4], p->histQ[5], fnv_shorts(p->cfg.coefI, 0x78), fnv_shorts(p->cfg.coefQ, 0x78));
  }
}
static void dump_mrf(int show, const char *tag, const V22Mrf *m, int deep)
{
  char a[40];
  ptr_desc(a, sizeof a, m->buf);
  emit(show, "%s w1=%d step=%d ph=%d wpos=%d len=%d buf=%s", tag, m->cfg.w1, m->step, m->phase, m->wpos, m->len, a);
  if (deep && m->buf) emit(show, "%s H buf=%016llx coefs=%016llx", tag, fnv_shorts(m->buf, 60), fnv_shorts(m->cfg.coefs, 0x10e));
}

static void fill_coefs(short *c, int n, int kind, const short *real)
{
  int i;
  for (i = 0; i < n; i++) {
    switch (kind) {
      case 0: c[i] = real[i]; break;
      case 1: c[i] = rshort(); break;
      case 2: c[i] = (short)(rpct(50) ? 32767 : -32768); break;
      case 3: c[i] = (short)(real[i] >> 2); break;
      case 4: c[i] = (short)i; break; /* identity: the reorder is visible */
      default: c[i] = (short)rrange(-4000, 4000); break;
    }
  }
}

/* ------------------------------------------------------------------------------------------------ PPS */
static void pps_init(int alloc, int fill, int ckind, unsigned w0)
{
  V22PpsCfg cfg;
  guards_set();
  fill_coefs(g_cI, 0x78, ckind, PPSv22_COFFS);
  fill_coefs(g_cQ, 0x78, (ckind + 1) % 6, PPSv22_COFFS);
  cfg.c0.w = (int)w0;
  cfg.coefI = g_cI;
  cfg.coefQ = g_cQ;
  cfg.w3 = (int)(rpct(50) ? 0 : rnd());
  if (alloc) memset(&g_p, fill, sizeof g_p);
  V22_PPS_init(&g_p, &cfg, alloc);
  guards_check("V22_PPS_init");
  cov_hit(alloc ? CV_P_INIT_A : CV_P_INIT_N);
}
static void sc_pps_life(void)
{
  int t;
  cov_def(CV_P_INIT_A, "pps init: allocate"); cov_def(CV_P_INIT_N, "pps init: no allocation"); cov_def(CV_P_FREE, "pps free"); cov_def(CV_P_DBL, "pps double init (leak)");
  cov_def(CV_P_MODEL, "pps reorder checked against the model");
  for (t = 0; t < 300; t++) {
    int fill = (t % 3 == 0) ? 0xA5 : (t % 3 == 1) ? 0x00 : (int)(rnd() & 0xff);
    char tag[32];
    snprintf(tag, sizeof tag, "PL%d", t);
    pps_init(1, fill, t % 6, rnd());
    dump_pps(t < 10, tag, &g_p, 1);
    { /* independent model of the reorder: i = 0x50 .. 0x77, j = i, i - 0x28, ... >= 0 (the identity-coded buffer shows the source index of every entry) */
      short e1[0x78];
      int i, j, n = 0;
      for (i = 0x50; i <= 0x77; i++)
        for (j = i; j >= 0; j -= 0x28) { e1[n] = (short)j; n++; }
      if (t % 6 == 4) { if (memcmp(e1, g_cI, sizeof e1) != 0) die3("pps reorder (I) differs from the model"); cov_hit(CV_P_MODEL); }
      if (t % 6 == 3) { if (memcmp(e1, g_cQ, sizeof e1) != 0) die3("pps reorder (Q) differs from the model"); cov_hit(CV_P_MODEL); }
    }
    {
      int k;
      for (k = 0; k < 3; k++) if (g_p.histI[k] != 0 || g_p.histQ[k] != 0) die3("pps hist %d", k);
    }
    g_p.fill = 5;
    pps_init(0, 0, (t + 2) % 6, rnd());
    dump_pps(t < 10, "PLe", &g_p, 1);
    if (t % 17 == 5) {
      pps_init(1, 0x33, 4, 0);
      cov_hit(CV_P_DBL);
      dump_pps(0, "PLd", &g_p, 1);
      trial_end("pps-leak", 1);
      continue;
    }
    V22_PPS_free(&g_p);
    cov_hit(CV_P_FREE);
    trial_end("pps-life", 0);
  }
}

static void pps_maps(int kind)
{
  int i;
  for (i = 0; i < 256; i++) {
    if (kind == 0) { g_imap[i] = SMCv22_IMAP_2400BPS[i & 15]; g_qmap[i] = SMCv22_QMAP_2400BPS[i & 15]; }
    else if (kind == 1) { g_imap[i] = SMCv22_IMAP_1200BPS[i & 15]; g_qmap[i] = SMCv22_QMAP_1200BPS[i & 15]; }
    else if (kind == 2) { g_imap[i] = rshort(); g_qmap[i] = rshort(); }
    else { g_imap[i] = (short)(rpct(50) ? 32767 : -32768); g_qmap[i] = (short)(rpct(50) ? 32767 : -32768); }
  }
}
static void pps_src(int lim, int symmax, int hikind)
{
  int i;
  memset(&g_c, 0, sizeof g_c);
  g_c.buf = g_sym;
  g_c.lim = (short)lim;
  for (i = 0; i < 64; i++) {
    unsigned lo = rmod((unsigned)symmax + 1u);
    unsigned hi = hikind == 0 ? 0 : hikind == 1 ? 0xff : rmod(256);
    g_sym[i] = (unsigned short)((hi << 8) | lo);
  }
}
static int pps_call(const char *tag, int show, int n)
{
  V22Pps before = g_p;
  int r, nn = n;
  V22Circ cb = g_c;
  memset(g_out, 0x5a, 80 * sizeof(short));
  r = (int)V22_PPS_filter(&g_p, &g_c, g_out, (unsigned short)n);
  if (r > 66000) die3("too many outputs %d", r);
  emit(show, "%s n=%d -> %d out=%016llx", tag, nn, r, fnv_shorts(g_out, (unsigned)(r & 0xffff)));
  if (r < 6) { int i; for (i = 0; i < r; i++) emit(0, "%s o%d=%d", tag, i, g_out[i]); }
  emit(0, "%s rd %d->%d", tag, cb.rd, g_c.rd);
  if (n == 0) cov_hit(CV_P_ZERO_N);
  if (n == 1) cov_hit(CV_P_ONE_CALL);
  if (r > 5) cov_hit(CV_P_MANY);
  if (before.flag == 1) cov_hit(CV_P_FLAG); else if (before.flag == 0) cov_hit(CV_P_NOFLAG);
  if (before.flag == 2 || before.flag == 5) cov_hit(CV_P_FLAG2);
  if (before.flag < 0) cov_hit(CV_P_NEGFLAG);
  if (g_p.flag) cov_hit(CV_P_END_FLAG);
  if (g_p.fill < before.fill + 1 && n > 2) cov_hit(CV_P_SHIFT);
  if (g_p.phase < before.phase) cov_hit(CV_P_WRAP_PH);
  if (g_c.rd < cb.rd) cov_hit(CV_P_RD_WRAP);
  if (before.fill > 2 && r) cov_hit(CV_P_MAIN_PATH);
  if (before.fill <= 2 && r) cov_hit(CV_P_ALT_PATH);
  if (before.cfg.c0.s[0] != 0) cov_hit(CV_P_STEP_OFF);
  if (g_c.lim == 1) cov_hit(CV_P_LIM_ONE);
  return r;
}
static void pps_poke(int flagkind)
{
  g_p.flag = (short)(flagkind == 0 ? (int)rmod(2) : (flagkind == 1 ? ((int[]){0, 1, 2, 5, -1}[rmod(5)]) : 1));
  g_p.phase = (short)(rpct(60) ? rrange(0, 0x27) : rpct(50) ? 0x27 : rrange(0x20, 0x27));
  g_p.fill = (short)(rpct(70) ? rrange(0, 5) : rrange(0, 2));
  g_p.cfg.c0.s[0] = (unsigned short)(rpct(50) ? 0 : rpct(60) ? rrange(0, 30) : rrange(0, 36));
  { int k; for (k = 0; k < 6; k++) { g_p.histI[k] = (short)(rpct(30) ? 0 : rpct(30) ? rshort() : rrange(-3000, 3000)); g_p.histQ[k] = (short)(rpct(30) ? 0 : rrange(-3000, 3000)); } }
}
static void sc_pps_filter(int e2e)
{
  int t, c;
  static const int sizes[] = {0, 1, 1, 2, 3, 4, 5, 8, 12, 17, 30, 45, 60};
  cov_def(CV_P_FLAG, "pps: flag set at the start"); cov_def(CV_P_NOFLAG, "pps: flag clear at the start"); cov_def(CV_P_SHIFT, "pps: history shifted");
  cov_def(CV_P_WRAP_PH, "pps: phase wrapped"); cov_def(CV_P_RD_WRAP, "pps: read index wrapped"); cov_def(CV_P_MANY, "pps: > 5 outputs");
  cov_def(CV_P_ALT_PATH, "pps: fill <= 2 path"); cov_def(CV_P_MAIN_PATH, "pps: fill > 2 path"); cov_def(CV_P_STEP_OFF, "pps: step offset nonzero"); 
  cov_def(CV_P_ONE_CALL, "pps: one-sample call");
  if (!e2e) { cov_def(CV_P_ZERO_N, "pps: count 0"); cov_def(CV_P_END_FLAG, "pps: ends with flag set"); cov_def(CV_P_FLAG2, "pps: flag 2 / 5"); cov_def(CV_P_NEGFLAG, "pps: negative flag"); cov_def(CV_P_HIBYTE, "pps: garbage high byte in the symbols"); cov_def(CV_P_LIM_ONE, "pps: limit 1"); }
  for (t = 0; t < (e2e ? 24 : 400); t++) {
    char tag[32];
    int lim = e2e ? 12 : (rpct(30) ? 1 : rpct(50) ? rrange(2, 12) : rrange(2, 64));
    snprintf(tag, sizeof tag, "%c%d", e2e ? 'Q' : 'P', t);
    pps_init(1, 0xA5, e2e ? 0 : (t % 6), e2e ? 0 : (rpct(50) ? 0 : (unsigned)rrange(0, 36)));
    if (e2e && t % 4 == 3) g_p.cfg.c0.s[0] = (unsigned short)(t % 5);
    pps_maps(e2e ? t % 2 : (int)rmod(4));
    g_p.imap = g_imap;
    g_p.qmap = g_qmap;
    { int smax = e2e ? 15 : (rpct(30) ? 255 : 15); int hk = e2e ? 0 : (int)rmod(3); pps_src(lim, smax, hk); } /* (no two PRNG calls in one expression: the evaluation order is unspecified) */
    if (!e2e) cov_hit(CV_P_HIBYTE);
    g_c.rd = (short)rmod((unsigned)lim);
    if (e2e) {
      int done = 0, calls = 0;
      g_p.flag = (short)(t % 5 == 4 ? 1 : 0);
      while (done < 12000) {
        int n = (t % 6 == 5) ? 1 : (rpct(30) ? rrange(1, 5) : rrange(1, 60));
        if (t % 9 == 8 && calls % 41 == 0) g_p.flag = (short)(rpct(50) ? 2 : -1); /* targeted: the arithmetic n -= flag */
        if (g_p.flag == 2 || g_p.flag == -1) n = rrange(3, 20);
        if (rpct(10)) { int i; for (i = 0; i < 64; i++) g_sym[i] = (unsigned short)rmod(16); }
        pps_call(tag, (t < 2 && calls < 8), n);
        done += n;
        calls++;
        if (calls % 50 == 0) dump_pps(0, tag, &g_p, 1);
      }
      dump_pps(1, tag, &g_p, 1);
      emit(1, "%s calls=%d", tag, calls);
    } else {
      for (c = 0; c < 30; c++) {
        int n = sizes[rmod(sizeof sizes / sizeof sizes[0])];
        if (rpct(60)) pps_poke((t % 11 == 10) ? 1 : 0);
        if (g_p.flag == 2 || g_p.flag == 5 || g_p.flag < 0) n = 6 + (int)rmod(10);
        if (rpct(15)) g_c.rd = (short)rmod((unsigned)lim);
        pps_call(tag, (t < 6 && c < 6), n);
        dump_pps(t < 4 && c < 4, tag, &g_p, 0);
      }
      dump_pps(t < 20, tag, &g_p, 1);
    }
    V22_PPS_free(&g_p);
    cov_hit(CV_P_FREE);
    trial_end("pps", 0);
  }
}

/* ------------------------------------------------------------------------------------------------ MRF */
static void mrf_init(int alloc, int fill, int ckind)
{
  V22MrfCfg cfg;
  guards_set();
  fill_coefs(g_mc, 0x10e, ckind, MRFv22_COFFS);
  cfg.coefs = g_mc;
  cfg.w1 = (int)(rpct(50) ? 0 : rnd());
  if (alloc) memset(&g_m, fill, sizeof g_m);
  V22_MRF_init(&g_m, &cfg, alloc);
  guards_check("V22_MRF_init");
  cov_hit(alloc ? CV_M_INIT_A : CV_M_INIT_N);
}
static void sc_mrf_life(void)
{
  int t;
  cov_def(CV_M_INIT_A, "mrf init: allocate"); cov_def(CV_M_INIT_N, "mrf init: no allocation"); cov_def(CV_M_FREE, "mrf free"); cov_def(CV_M_DBL, "mrf double init (leak)");
  cov_def(CV_M_MODEL, "mrf reorder checked against the model");
  for (t = 0; t < 300; t++) {
    char tag[32];
    int fill = (t % 3 == 0) ? 0xA5 : (t % 3 == 1) ? 0x00 : (int)(rnd() & 0xff);
    snprintf(tag, sizeof tag, "ML%d", t);
    mrf_init(1, fill, t % 6);
    dump_mrf(t < 10, tag, &g_m, 1);
    if (t % 6 == 4) {
      short e[0x10e];
      int i, j, n = 0;
      for (i = 0x105; i <= 0x10d; i++)
        for (j = i; j >= 0; j -= 9) e[n++] = (short)j;
      if (n != 0x10e || memcmp(e, g_mc, sizeof e) != 0) die3("mrf reorder differs from the model");
      cov_hit(CV_M_MODEL);
    }
    { int k; for (k = 0; k < 30; k++) if (g_m.buf[k] != 0) die3("mrf buf %d", k); }
    g_m.wpos = 40;
    mrf_init(0, 0, (t + 3) % 6);
    dump_mrf(t < 10, "MLe", &g_m, 1);
    if (t % 17 == 5) {
      mrf_init(1, 0x33, 4);
      cov_hit(CV_M_DBL);
      dump_mrf(0, "MLd", &g_m, 1);
      trial_end("mrf-leak", 1);
      continue;
    }
    V22_MRF_free(&g_m);
    cov_hit(CV_M_FREE);
    trial_end("mrf-life", 0);
  }
}
static int mrf_call(const char *tag, int show, int n)
{
  V22Mrf before = g_m;
  int r;
  g_in_base = g_in;
  g_in_len = (unsigned)(n > 0 ? n : 0);
  memset(g_out, 0x5a, 128 * sizeof(short));
  r = (int)V22_MRF_filter(&g_m, g_in, g_out, (short)n);
  g_in_base = NULL;
  if (r > 200) die3("too many outputs %d", r);
  emit(show, "%s n=%d -> %d out=%016llx buf=%016llx", tag, n, r, fnv_shorts(g_out, (unsigned)r), fnv_shorts(g_m.buf, 60));
  if (r < 6) { int i; for (i = 0; i < r; i++) emit(0, "%s o%d=%d", tag, i, g_out[i]); }
  if (n == 0) cov_hit(CV_M_ZERO_N);
  if (n == 1) cov_hit(CV_M_ONE);
  if (n > 0 && n < before.step) cov_hit(CV_M_PARTIAL);
  if (n >= before.step && n > 0) cov_hit(CV_M_FULL);
  if (n > 0 && before.wpos + (n < before.step ? n : before.step) > 0x3c) cov_hit(n < before.step ? CV_M_SHIFT_PART : CV_M_SHIFT_FULL);
  if (r) { if (g_m.wpos >= 0) cov_hit(CV_M_MULTI); }
  if (r > 5) cov_hit(CV_M_MANY_OUT);
  if (r) cov_hit(before.wpos + (n < before.step ? n : before.step) >= before.len || before.wpos >= before.len ? CV_M_PATH_A : CV_M_PATH_B);
  if (r && before.step == 2) cov_hit(CV_M_STEP2);
  if (r && before.step == 3) cov_hit(CV_M_STEP3);
  if (r && before.step != 2 && before.step != 3) cov_hit(CV_M_STEPOTHER);
  if (before.phase == 8) cov_hit(CV_M_PH_MAX);
  return r;
}
static void mrf_in(int n, int kind)
{
  int i;
  for (i = 0; i < n; i++) {
    switch (kind) {
      case 0: g_in[i] = rshort(); break;
      case 1: g_in[i] = (short)(rpct(50) ? 32767 : -32768); break;
      case 2: g_in[i] = 0; break;
      case 3: g_in[i] = (short)rrange(-300, 300); break;
      case 4: g_in[i] = (short)((i * 1171) % 20000 - 10000); break;
      default: g_in[i] = (short)(rshort() >> 3); break;
    }
  }
}
static void sc_mrf_filter(int e2e)
{
  int t, c;
  static const int sizes[] = {0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 11, 12, 30, 31, 60, 61, 90};
  cov_def(CV_M_PARTIAL, "mrf: partial call"); cov_def(CV_M_FULL, "mrf: full step"); cov_def(CV_M_SHIFT_FULL, "mrf: history shift, full path"); cov_def(CV_M_SHIFT_PART, "mrf: history shift, partial path");
  cov_def(CV_M_PATH_A, "mrf: window behind the start (wpos >= len)"); cov_def(CV_M_PATH_B, "mrf: window at the start (wpos < len)"); cov_def(CV_M_STEP2, "mrf: step 2"); cov_def(CV_M_STEP3, "mrf: step 3");
  cov_def(CV_M_ONE, "mrf: count 1"); cov_def(CV_M_MANY_OUT, "mrf: > 5 outputs"); cov_def(CV_M_PH_MAX, "mrf: phase 8"); cov_def(CV_M_MULTI, "mrf: outputs");
  if (!e2e) { cov_def(CV_M_STEPOTHER, "mrf: step other than 2 / 3"); cov_def(CV_M_ZERO_N, "mrf: count 0"); }
  for (t = 0; t < (e2e ? 30 : 400); t++) {
    char tag[32];
    snprintf(tag, sizeof tag, "%c%d", e2e ? 'N' : 'M', t);
    mrf_init(1, 0xA5, e2e ? 0 : (t % 6));
    if (e2e) {
      int kind = t % 6, done = 0, calls = 0;
      while (done < 20000) {
        int n = (t % 5 == 4) ? 1 : (t % 5 == 3) ? 3 : (rpct(20) ? rrange(1, 4) : rrange(1, 90));
        if (n > 90) n = 90;
        mrf_in(n, kind == 5 ? 5 : kind);
        mrf_call(tag, (t < 2 && calls < 8), n);
        done += n;
        calls++;
        if (calls % 60 == 0) dump_mrf(0, tag, &g_m, 1);
      }
      if (kind == 1) cov_hit(CV_M_SAT);
      dump_mrf(1, tag, &g_m, 1);
      emit(1, "%s calls=%d", tag, calls);
      cov_hit(CV_M_E2E);
    } else {
      for (c = 0; c < 30; c++) {
        int n = sizes[rmod(sizeof sizes / sizeof sizes[0])];
        if (rpct(50)) {
          g_m.step = (short)(rpct(60) ? rrange(2, 3) : rrange(1, 6));
          g_m.phase = (short)(rpct(60) ? rrange(0, 8) : 8);
          g_m.wpos = (short)(rpct(30) ? rrange(0x30, 0x3c) : rrange(0, 0x3c));
          { int k; for (k = 0; k < 60; k++) { int sh = (int)rmod(8); short rv = rshort(); g_m.buf[k] = (short)(rpct(30) ? 0 : rv >> sh); } }
        }
        mrf_in(n > 0 ? n : 1, (int)rmod(6));
        mrf_call(tag, (t < 6 && c < 6), n);
        dump_mrf(t < 4 && c < 4, tag, &g_m, 0);
      }
      dump_mrf(t < 20, tag, &g_m, 1);
    }
    V22_MRF_free(&g_m);
    cov_hit(CV_M_FREE);
    trial_end("mrf", 0);
  }
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("R05_SEED");
  if (seed) g_lcg ^= (unsigned)strtoul(seed, NULL, 0) * 2654435761u;
  g_lcg ^= (unsigned)sc * 0x9e3779b9u;
  switch (sc) {
    case 0: sc_pps_life(); break;
    case 1: sc_pps_filter(0); break;
    case 2: sc_pps_filter(1); break;
    case 3: sc_mrf_life(); sc_mrf_filter(0); break;
    case 4: sc_mrf_filter(1); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  emit(1, "calls: malloc %lu free %lu memcpy %lu", g_nmalloc, g_nfree, g_nmemcpy);
  cov_finish();
  return 0;
}
