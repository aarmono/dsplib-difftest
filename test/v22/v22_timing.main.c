/*
 * R04 differential harness (2 of 2): the V.22 / V.22bis SRE (symbol-timing recovery) - V22_SRE_init / _free / _recover - real dsplibs.o vs src/v22/v22_timing.c
 * (+ src/v22/v22_tables.c).  Same architecture as test/v22/v22_equalizer.main.c (library test/r04_common.h: hashed canonical output, deterministic heap, sysdep_malloc / _free /
 * _memcpy recorders, coverage counters exit 2, 3 = harness violation; the typed object of src/v22/v22.h on both sides; the 32-bit BLOB run is the x86-64 golden).
 * FPM_atan is the real callee of each side.
 *
 * Scenarios (each its own process):
 *   0 lifecycle: init with allocate = 1 / 0 over garbage-filled objects (every written field, allocation order and sizes, the REORDERED coefficient table checked against an
 *     independent model of the polyphase reorder, the cleared history prefix / state), free (order), re-init without allocation, double init (leak);
 *   1 recover over randomly poked state (mode, integ, avgAngle / avgMag, wantTrack, pllEnable, idx / frac, blk, cnt, sym, clk, rem 0 .. 6, wpos around the history-shift
 *     boundary, the accumulators, the six IIR state words) and call sizes 0 .. 40 incl. one-sample calls and partial calls;
 *   2 end to end, real signals: PSK tones at the symbol rate with random phase jumps, offset / drifting timing (a resampled stream), noise, saturation, silence, random: long
 *     streams so the mode machine (0 -> 1 -> 2 -> 1), the settling counter, the interpolator phase and the coefficient re-synthesis evolve;
 *   4 sweeps: an estimate-block state restored before every call while one state word sweeps (integrator over all 65536 values, averaged angle +-0x1500, magnitude list, counter, fraction);
 *   3 mode-machine drive: streams built to hold each mode, the tracking enable on / off, the one-sample call loops, state dumps after every symbol.
 */
#include "../r04_common.h"
#include "v22/v22.h"
#include "v22/v22_tables.h"

enum {
  CV_INIT_ALLOC = 0, CV_INIT_NOALLOC, CV_FREE, CV_DOUBLE_INIT, CV_MODEL,
  CV_PART_POS = 10, CV_PART_NEG_UNUSED, CV_REM0, CV_REM1, CV_REM2, CV_FULL, CV_SHIFT_FULL, CV_SHIFT_PART, CV_WIN_LOW, CV_WIN_HIGH, CV_MANY_OUT, CV_ZERO_COUNT,
  CV_CLK_WRAP = 30, CV_BLK_WAIT, CV_ESTIMATE, CV_TRACK_ON, CV_TRACK_OFF, CV_RESET_BR, CV_ATAN_PATH, CV_ZERO_PATH, CV_MODE0, CV_MODE1, CV_MODE2, CV_TO_MODE1_FROM2, CV_TO_MODE2,
  CV_CNT_LO, CV_CNT_HI, CV_PLL_OFF, CV_COEF_RESYNTH, CV_IDX_NEG_WRAP, CV_FRAC_WRAP, CV_FRAC_NEG, CV_ACC_NONZERO, CV_MODE2_SKIP,
  CV_E2E_TONE = 70, CV_E2E_NOISE, CV_E2E_SAT, CV_E2E_SILENCE, CV_E2E_RANDOM, CV_E2E_ONE, CV_E2E_DRIFT
};

static V22Sre g_s;
static short g_in[4096 + 64];
static short g_out[8192];

static void dump(int show, const char *tag, const V22Sre *s, int deep)
{
  char a[40], b[40], c[40];
  ptr_desc(a, sizeof a, s->coefs);
  ptr_desc(b, sizeof b, s->hist);
  ptr_desc(c, sizeof c, s->state);
  emit(show, "%s mode=%d integ=%d aA=%d aM=%d want=%d fc=%d pllE=%d len=%d wpos=%d coefs=%s hist=%s state=%s accX=%d accY=%d frac=%d idx=%d blk=%d cnt=%d sym=%d clk=%d rem=%d", tag, s->mode,
       s->integ, s->avgAngle, s->avgMag, s->wantTrack, s->pllFlagC, s->pllEnable, s->len, s->wpos, a, b, c, s->accX, s->accY, s->frac, s->idx, s->blk, s->cnt, s->sym, s->clk, s->rem);
  if (deep && s->coefs && s->hist && s->state)
    emit(show, "%s H coefs=%016llx hist=%016llx st=%d,%d,%d,%d,%d,%d", tag, fnv_shorts(s->coefs, 270), fnv_shorts(s->hist, 54), s->state[0], s->state[1], s->state[2], s->state[3], s->state[4],
         s->state[5]);
}

static void fresh(int fill)
{
  memset(&g_s, fill, sizeof g_s);
  V22_SRE_init(&g_s, 1);
  cov_hit(CV_INIT_ALLOC);
}
static void drop(void)
{
  V22_SRE_free(&g_s);
  cov_hit(CV_FREE);
}

/* independent model of the init coefficient table: e = 0x104 .. 0x10d, j = e, e - 10, ... >= 0 */
static void model_coefs(short *dst)
{
  int e, j, n = 0;
  for (e = 0x104; e <= 0x10d; e++)
    for (j = e; j >= 0; j -= 10) dst[n++] = SREv22_COFFS[j];
  if (n != 270) die3("model count %d", n);
}

static int call_rc(const char *tag, int show, int n)
{
  V22Sre before = g_s;
  short coefs_before[270];
  int r;
  memcpy(coefs_before, g_s.coefs, sizeof coefs_before);
  g_in_base = g_in;
  g_in_len = (unsigned)(n > 0 ? n : 0);
  memset(g_out, 0x5a, 64 * sizeof(short));
  r = (int)V22_SRE_recover(&g_s, g_in, g_out, (short)n);
  g_in_base = NULL;
  if (r > 4000) die3("too many outputs %d", r);
  emit(show, "%s n=%d -> %d out=%016llx hist=%016llx", tag, n, r, fnv_shorts(g_out, (unsigned)r), fnv_shorts(g_s.hist, 54));
  if (r < 6) { int i; for (i = 0; i < r; i++) emit(0, "%s o%d=%d", tag, i, g_out[i]); }
  if (n == 0) cov_hit(CV_ZERO_COUNT);
  else if (n < before.rem) cov_hit(CV_PART_POS);
  else cov_hit(CV_FULL);
  if (before.rem == 0 && n) cov_hit(CV_REM0);
  if (before.rem == 1) cov_hit(CV_REM1);
  if (before.rem == 2) cov_hit(CV_REM2);
  if (r > 5) cov_hit(CV_MANY_OUT);
  if (n && before.wpos + (n < before.rem ? n : before.rem) > 0x36) cov_hit(n < before.rem ? CV_SHIFT_PART : CV_SHIFT_FULL);
  if (r) {
    if (before.wpos + before.rem < before.len) cov_hit(CV_WIN_LOW); else cov_hit(CV_WIN_HIGH);
    if (g_s.mode == 0) cov_hit(CV_MODE0);
    if (g_s.mode == 1) cov_hit(CV_MODE1);
    if (g_s.mode == 2) cov_hit(CV_MODE2);
    if (before.mode == 2 && g_s.mode == 1) cov_hit(CV_TO_MODE1_FROM2);
    if (before.mode == 1 && g_s.mode == 2) cov_hit(CV_TO_MODE2);
    if (before.cnt > 0x40) cov_hit(CV_CNT_HI);
    if (before.cnt <= 0x40 && before.wantTrack) cov_hit(CV_CNT_LO);
    if (!before.pllEnable) cov_hit(CV_PLL_OFF);
    if (before.wantTrack) cov_hit(CV_TRACK_ON); else cov_hit(CV_TRACK_OFF);
    if (before.clk + r >= 6 || r > 5) cov_hit(CV_CLK_WRAP);
    if (g_s.sym > before.sym || before.blk > before.sym + 1) cov_hit(CV_BLK_WAIT);
    if (g_s.sym == 0 && before.sym != 0) cov_hit(CV_ESTIMATE);
    if (before.avgMag <= 1 && before.mode != g_s.mode) cov_hit(CV_RESET_BR);
    if (g_s.accX != 0 || g_s.accY != 0) cov_hit(CV_ACC_NONZERO);
    if (g_s.frac != before.frac) cov_hit(CV_FRAC_WRAP);
    if (g_s.idx != before.idx) cov_hit(CV_IDX_NEG_WRAP);
    if (g_s.integ != before.integ) cov_hit(CV_ATAN_PATH);
    if (g_s.wantTrack == 0 && before.wantTrack) cov_hit(CV_ZERO_PATH);
    if (g_s.mode == 2 && before.mode == 2 && g_s.wantTrack) cov_hit(CV_MODE2_SKIP);
    if (memcmp(coefs_before, g_s.coefs, sizeof coefs_before) != 0) cov_hit(CV_COEF_RESYNTH);
    if (g_s.frac < 0) cov_hit(CV_FRAC_NEG);
  }
  return r;
}

static void fill_in(int n, int kind)
{
  int i;
  for (i = 0; i < n; i++) {
    switch (kind) {
      case 0: g_in[i] = rshort(); break;
      case 1: g_in[i] = (short)(rpct(50) ? 32767 : -32768); break;
      case 2: g_in[i] = 0; break;
      case 3: g_in[i] = (short)rrange(-300, 300); break;
      case 4: g_in[i] = (short)((i * 1171) % 20000 - 10000); break;
      default: g_in[i] = (short)(rshort() >> 4); break;
    }
  }
}

static void sc_life(void)
{
  int t;
  short model[270];
  cov_def(CV_INIT_ALLOC, "init: allocate"); cov_def(CV_INIT_NOALLOC, "init: no allocation"); cov_def(CV_FREE, "free"); cov_def(CV_DOUBLE_INIT, "double init (leak)");
  cov_def(CV_MODEL, "coefficient reorder checked against the model");
  model_coefs(model);
  for (t = 0; t < 300; t++) {
    int fill = (t % 3 == 0) ? 0xA5 : (t % 3 == 1) ? 0x00 : (int)(rnd() & 0xff);
    char tag[32];
    snprintf(tag, sizeof tag, "L%d", t);
    fresh(fill);
    dump(t < 10, tag, &g_s, 1);
    if (memcmp(g_s.coefs, model, sizeof model) != 0) die3("coefficient reorder differs from the model");
    cov_hit(CV_MODEL);
    {
      int k;
      for (k = 0; k < 27; k++) if (g_s.hist[k] != 0) die3("hist %d", k);
      for (k = 0; k < 6; k++) if (g_s.state[k] != 0) die3("state %d", k);
    }
    fill_in(40, t % 4);
    call_rc("Lr", 0, 5 + (int)rmod(30));
    dump(0, tag, &g_s, 1);
    g_s.integ = 99;
    V22_SRE_init(&g_s, 0);
    cov_hit(CV_INIT_NOALLOC);
    dump(t < 10, "Lre", &g_s, 1);
    if (memcmp(g_s.coefs, model, sizeof model) != 0) die3("coefficient reorder (re-init) differs from the model");
    if (t % 17 == 5) {
      V22_SRE_init(&g_s, 1);
      cov_hit(CV_DOUBLE_INIT);
      dump(0, "Ldbl", &g_s, 1);
      trial_end("life-leak", 1);
      continue;
    }
    drop();
    trial_end("life", 0);
  }
}

static void poke(void)
{
  g_s.mode = (short)rrange(0, 2);
  g_s.integ = (short)(rpct(30) ? (rpct(50) ? 0x7fff : -32768) : rpct(40) ? rrange(-3000, 3000) : rshort());
  g_s.avgAngle = (short)(rpct(30) ? rrange(-10, 10) : rpct(30) ? rrange(0x7f0, 0x810) : rshort());
  g_s.avgMag = (short)(rpct(40) ? rrange(0, 4) : rpct(20) ? (rpct(50) ? 0x7fff : -32768) : rrange(0, 5000));
  g_s.wantTrack = rpct(60);
  g_s.pllFlagC = rpct(50);
  g_s.pllEnable = rpct(85);
  g_s.idx = (short)rrange(0, 9);
  g_s.frac = (short)(rpct(70) ? rrange(0, 0x7ff) : rpct(50) ? rrange(0, 0x1000) : rrange(0, 0x7fff));
  g_s.blk = (short)(rpct(50) ? 3 : rpct(50) ? 0xc : rrange(0, 14));
  g_s.cnt = (short)(rpct(50) ? rrange(0x3c, 0x44) : rrange(0, 0x60));
  g_s.sym = (short)rrange(0, 14);
  g_s.clk = (short)rrange(0, 5);
  g_s.rem = (short)(rpct(60) ? 1 : rrange(0, 6));
  g_s.wpos = (short)(rpct(40) ? rrange(0, 0x36) : rpct(50) ? rrange(0x30, 0x36) : rrange(0, 0x1b));
  g_s.accX = (int)(rpct(50) ? (int)(rnd() & 0xffffff) - 0x800000 : (int)rnd());
  g_s.accY = (int)(rpct(50) ? (int)(rnd() & 0xffffff) - 0x800000 : (int)rnd());
  if (rpct(30)) { g_s.accX = (int)((unsigned)(rrange(-120, 120)) << 16) + (int)rmod(0x10000); }
  if (rpct(30)) { int k; for (k = 0; k < 6; k++) g_s.state[k] = (short)(rpct(30) ? 0 : rpct(30) ? rshort() : rrange(-2000, 2000)); }
}

static void sc_rc(void)
{
  int t, c;
  static const int sizes[] = {0, 1, 1, 1, 2, 3, 5, 6, 7, 11, 12, 13, 19, 25, 40};
  cov_def(CV_PART_POS, "recover: partial call"); cov_def(CV_REM0, "recover: rem = 0"); cov_def(CV_REM1, "recover: rem = 1"); cov_def(CV_REM2, "recover: rem = 2");
  cov_def(CV_FULL, "recover: full symbol"); cov_def(CV_SHIFT_FULL, "recover: history shift, full path"); cov_def(CV_SHIFT_PART, "recover: history shift, partial path");
  cov_def(CV_WIN_LOW, "recover: window at the start (wpos < len)"); cov_def(CV_WIN_HIGH, "recover: window behind the start"); cov_def(CV_MANY_OUT, "recover: > 5 outputs in one call");
  cov_def(CV_ZERO_COUNT, "recover: count 0");
  cov_def(CV_CLK_WRAP, "recover: symbol clock wrapped"); cov_def(CV_BLK_WAIT, "recover: estimate block not complete"); cov_def(CV_ESTIMATE, "recover: estimate block complete");
  cov_def(CV_TRACK_ON, "recover: tracking on"); cov_def(CV_TRACK_OFF, "recover: tracking off"); cov_def(CV_RESET_BR, "recover: low-magnitude reset branch"); cov_def(CV_ATAN_PATH, "recover: PLL integrator moved");
  cov_def(CV_ZERO_PATH, "recover: tracking dropped"); cov_def(CV_MODE0, "recover: mode 0"); cov_def(CV_MODE1, "recover: mode 1"); cov_def(CV_MODE2, "recover: mode 2");
  cov_def(CV_TO_MODE1_FROM2, "recover: mode 2 -> 1"); cov_def(CV_TO_MODE2, "recover: mode 1 -> 2"); cov_def(CV_CNT_LO, "recover: settling counter <= 0x40"); cov_def(CV_CNT_HI, "recover: settling counter > 0x40");
  cov_def(CV_PLL_OFF, "recover: pll disabled"); cov_def(CV_COEF_RESYNTH, "recover: coefficients re-synthesised"); cov_def(CV_IDX_NEG_WRAP, "recover: interpolator phase changed");
  cov_def(CV_FRAC_WRAP, "recover: fraction changed"); cov_def(CV_ACC_NONZERO, "recover: accumulators nonzero after");
  cov_def(CV_MODE2_SKIP, "recover: mode 2 estimate kept tracking");
  for (t = 0; t < 400; t++) {
    char tag[32];
    snprintf(tag, sizeof tag, "R%d", t);
    fresh(0xA5);
    for (c = 0; c < 30; c++) {
      int n = sizes[rmod(sizeof sizes / sizeof sizes[0])];
      if (rpct(50)) poke();
      fill_in(n > 0 ? n : 1, (int)rmod(6));
      if (n == 1 && rpct(40)) g_s.rem = 1;
      call_rc(tag, (t < 6 && c < 6), n);
      dump(t < 4 && c < 4, tag, &g_s, 0);
    }
    dump(t < 20, tag, &g_s, 1);
    drop();
    trial_end("rc", 0);
  }
}

/* a PSK-like symbol-rate tone: the interpolator sees 10 times ... the blob's input is the 7200 Hz stream (rem 1 per symbol on average); the signal is only a generator */
static const short c12[12] = {32767, 28378, 16384, 0, -16384, -28378, -32767, -28378, -16384, 0, 16384, 28378};
static void gen_signal(short *p, int n, int kind, int fm, int *pos, int *ph, int drift)
{
  int i;
  for (i = 0; i < n; i++) {
    int v;
    int pp = (*pos)++;
    if (pp % 6 == 0) *ph = (int)rmod(4) * 3;
    v = (c12[(fm * pp + *ph + (drift ? pp / 41 : 0)) % 12] * 3) >> 2;
    switch (kind) {
      case 0: break;
      case 1: v += rrange(-1500, 1500); break;
      case 2: v = v * 2; break;
      case 3: v = 0; break;
      case 4: v = rshort(); break;
      default: v += rrange(-8000, 8000); break;
    }
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    p[i] = (short)v;
  }
}
static void sc_e2e(int mode_drive)
{
  int run;
  cov_def(CV_E2E_TONE, "e2e: clean tone"); cov_def(CV_E2E_NOISE, "e2e: noisy");
  if (!mode_drive) { cov_def(CV_E2E_SAT, "e2e: saturated"); cov_def(CV_E2E_SILENCE, "e2e: silence"); cov_def(CV_E2E_RANDOM, "e2e: random"); }
  cov_def(CV_E2E_ONE, "e2e: one-sample calls"); cov_def(CV_E2E_DRIFT, "e2e: drifting timing");
  cov_def(CV_MODE0, "e2e: mode 0"); cov_def(CV_MODE1, "e2e: mode 1"); cov_def(CV_MODE2, "e2e: mode 2"); cov_def(CV_TO_MODE2, "e2e: 1 -> 2"); cov_def(CV_TO_MODE1_FROM2, "e2e: 2 -> 1");
  cov_def(CV_COEF_RESYNTH, "e2e: coefficients re-synthesised"); cov_def(CV_ESTIMATE, "e2e: estimate"); cov_def(CV_CNT_HI, "e2e: settling counter > 0x40");
  cov_def(CV_REM0, "e2e: rem 0"); cov_def(CV_REM2, "e2e: rem 2"); cov_def(CV_REM1, "e2e: rem 1");
  for (run = 0; run < (mode_drive ? 24 : 40); run++) {
    int kind = mode_drive ? (run % 3 == 0 ? 0 : run % 3 == 1 ? 1 : 5) : run % 6, fm = 1 + (run / 6) % 5, pos = 0, ph = 0, done = 0, total = 0, drift = (run % 4) == 3;
    char tag[32];
    snprintf(tag, sizeof tag, "E%d", run);
    fresh(0xA5);
    if (run % 7 == 3) g_s.pllEnable = 0;
    if (mode_drive) { g_s.mode = (short)(run % 3); g_s.wantTrack = 1; g_s.cnt = (short)(run % 2 ? 0x41 : 0x3a); }
    if (kind == 0) cov_hit(CV_E2E_TONE);
    if (kind == 1 || kind == 5) cov_hit(CV_E2E_NOISE);
    if (kind == 2) cov_hit(CV_E2E_SAT);
    if (kind == 3) cov_hit(CV_E2E_SILENCE);
    if (kind == 4) cov_hit(CV_E2E_RANDOM);
    if (drift) cov_hit(CV_E2E_DRIFT);
    while (done < 30000) {
      int n = (run % 8 == 7) ? 1 : (rpct(20) ? rrange(1, 6) : rrange(1, 40));
      gen_signal(g_in, n, kind, fm, &pos, &ph, drift);
      call_rc(tag, (run < 2 && done < 300), n);
      if (n == 1) cov_hit(CV_E2E_ONE);
      done += n;
      total++;
      if ((total % 97) == 0) dump(0, tag, &g_s, 1);
      if (mode_drive && (total % 7) == 0) dump(0, tag, &g_s, 0);
    }
    dump(1, tag, &g_s, 1);
    emit(1, "%s calls=%d", tag, total);
    drop();
    trial_end("e2e", 0);
  }
}

/* ------------------------------------------------------------------------------------------------ scenario 4: sweeps
 * An estimate-block state (the symbol clock about to wrap, the block complete, tracking on, pll on) is restored before EVERY call and ONE input is processed while one
 * state word sweeps its whole interesting range: the PLL integrator over all 65536 values (every value of the clamped phase step), the averaged angle over +-0x1500 (the
 * 0x7ff / 4 mode-switch thresholds), the averaged magnitude over a list incl. negative values, the settling counter, the interpolator fraction / phase.  This reaches the exact
 * boundary values of every 16-bit range test of the estimate path. */
static void sweep_one(const V22Sre *tmpl, const short *coefs0, const short *hist0, const short *st0)
{
  g_s = *tmpl;
  memcpy(g_s.coefs, coefs0, 270 * sizeof(short));
  memcpy(g_s.hist, hist0, 54 * sizeof(short));
  memcpy(g_s.state, st0, 6 * sizeof(short));
}
static void sc_sweep(void)
{
  int t, v, kind;
  V22Sre tmpl;
  short coefs0[270], hist0[54], st0[6];
  cov_def(CV_MODE0, "sweep: mode 0 reached"); cov_def(CV_MODE1, "sweep: mode 1 reached"); cov_def(CV_MODE2, "sweep: mode 2 reached"); cov_def(CV_TO_MODE1_FROM2, "sweep: 2 -> 1");
  cov_def(CV_TO_MODE2, "sweep: 1 -> 2"); cov_def(CV_COEF_RESYNTH, "sweep: coefficients re-synthesised"); cov_def(CV_ESTIMATE, "sweep: estimate block completed");
  fresh(0xA5);
  for (t = 0; t < 18; t++) {
    int mode = t % 3;
    memcpy(coefs0, g_s.coefs, sizeof coefs0);
    for (v = 0; v < 54; v++) hist0[v] = (short)(rpct(50) ? rshort() : rrange(-3000, 3000));
    for (v = 0; v < 6; v++) st0[v] = (short)(rpct(30) ? 0 : rrange(-2000, 2000));
    tmpl = g_s;
    tmpl.mode = (short)mode;
    tmpl.wantTrack = 1;
    tmpl.pllFlagC = 1;
    tmpl.pllEnable = 1;
    tmpl.len = 0x1b;
    tmpl.wpos = (short)rrange(0x1b, 0x30);
    tmpl.idx = (short)rrange(0, 9);
    tmpl.frac = (short)rrange(0, 0x7ff);
    tmpl.blk = 3;
    tmpl.sym = 2;
    tmpl.clk = 5;
    tmpl.rem = 1;
    tmpl.cnt = (short)((t / 3) % 2 ? 0x41 : 0x30);
    tmpl.avgMag = 100;
    tmpl.avgAngle = (short)rrange(-3000, 3000);
    tmpl.integ = (short)rrange(-3000, 3000);
    tmpl.accX = (int)(rrange(-200, 200) * 65536 + (int)rmod(65536));
    tmpl.accY = (int)(rrange(-200, 200) * 65536 + (int)rmod(65536));
    if (mode == 2 && (t / 6) % 2) tmpl.accX = (int)(rrange(-0x20, 0x20) * 65536); /* mode 2 with the tracking window |x| <= 0x31 closed: the estimate keeps tracking */
    g_in[0] = rshort();
    for (kind = 0; kind < 5; kind++) {
      int lo, hi, step = 1;
      static const int mags[] = {-32768, -1000, -3, -2, -1, 0, 1, 2, 3, 4, 100, 32767};
      switch (kind) {
        case 0: lo = -32768; hi = 32767; break;       /* integ */
        case 1: lo = -0x1500; hi = 0x1500; break;     /* avgAngle */
        case 2: lo = 0; hi = (int)(sizeof mags / sizeof mags[0]) - 1; break; /* avgMag list */
        case 3: lo = 0x38; hi = 0x44; break;          /* cnt */
        default: lo = 0; hi = 0x1000; break;          /* frac */
      }
      for (v = lo; v <= hi; v += step) {
        V22Sre before;
        int r;
        sweep_one(&tmpl, coefs0, hist0, st0);
        switch (kind) {
          case 0: g_s.integ = (short)v; break;
          case 1: g_s.avgAngle = (short)v; break;
          case 2: g_s.avgMag = (short)mags[v]; break;
          case 3: g_s.cnt = (short)v; break;
          default: g_s.frac = (short)v; break;
        }
        before = g_s;
        g_in_base = g_in;
        g_in_len = 1;
        r = (int)V22_SRE_recover(&g_s, g_in, g_out, 1);
        g_in_base = NULL;
        emit(0, "S%d.%d.%d -> %d o=%d mode=%d integ=%d aA=%d aM=%d want=%d fc=%d frac=%d idx=%d blk=%d cnt=%d sym=%d clk=%d rem=%d aX=%d aY=%d c=%016llx", t, kind, v, r, r ? g_out[0] : 0, g_s.mode,
             g_s.integ, g_s.avgAngle, g_s.avgMag, g_s.wantTrack, g_s.pllFlagC, g_s.frac, g_s.idx, g_s.blk, g_s.cnt, g_s.sym, g_s.clk, g_s.rem, g_s.accX, g_s.accY, fnv_shorts(g_s.coefs, 270));
        if (g_s.mode == 0) cov_hit(CV_MODE0);
        if (g_s.mode == 1) cov_hit(CV_MODE1);
        if (g_s.mode == 2) cov_hit(CV_MODE2);
        if (before.mode == 2 && g_s.mode == 1) cov_hit(CV_TO_MODE1_FROM2);
        if (before.mode == 1 && g_s.mode == 2) cov_hit(CV_TO_MODE2);
        if (memcmp(coefs0, g_s.coefs, sizeof coefs0) != 0) cov_hit(CV_COEF_RESYNTH);
        if (g_s.sym == 0 && g_s.clk == 0) cov_hit(CV_ESTIMATE);
      }
      emit(t == 0, "sweep %d.%d done", t, kind);
    }
  }
  drop();
  trial_end("sweep", 0);
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("R04_SEED");
  if (seed) g_lcg ^= (unsigned)strtoul(seed, NULL, 0) * 2654435761u;
  g_lcg ^= (unsigned)sc * 0x9e3779b9u;
  switch (sc) {
    case 0: sc_life(); break;
    case 1: sc_rc(); break;
    case 2: sc_e2e(0); break;
    case 3: sc_e2e(1); break;
    case 4: sc_sweep(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  emit(1, "calls: malloc %lu free %lu memcpy %lu", g_nmalloc, g_nfree, g_nmemcpy);
  cov_finish();
  return 0;
}
