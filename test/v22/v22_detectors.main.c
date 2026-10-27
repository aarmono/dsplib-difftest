/*
 * R05 differential harness (3 of 3): the V.22 / V.22bis detectors Detect_Retrain / Detect_Rmloop2_ACK / Detect_1s / Detect_v22 - real dsplibs.o vs src/v22/v22_detectors.c.
 * Library test/r04_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation); the typed objects of src/v22/v22.h on both sides, the 32-bit BLOB run is
 * the x86-64 golden.  Detect_v22's callees FPM_AGC_agc / FPM_MTD_detect are the REAL ones of each side (blob: dsplibs.o; port: src/fpm/fpm_agc.c, fpm_mtd.c, fpm_math.c, fpm_iir.c,
 * fpm_tables.c), driven through objects the harness builds with the V.22 values of AGCv22_CFG2 / MTDv22_CFG / _CFG2 / MTDv22_COEF / _COEF2 (copied from the blob: readelf), no
 * table handle needed.  dsplibs_debug_printf is the stub's (real output, the line "V22: Detect_V22 OK!!!" is part of the diffed text).
 *
 * Scenarios (each its own process):
 *   0 Detect_Retrain: the patterns that make it fire (even words 0x000c..., odd words with bits 3:2 clear: runs of equal word pairs) with 0 .. k perturbed words swept across the
 *     threshold, random counts 0 .. 600 incl. odd counts and counts > 3640 (the 16-bit running sums wrap), random data, the exact tie of |2 s2 - s1| and the threshold;
 *   1 Detect_Rmloop2_ACK and Detect_1s: data near the K the mode selects (10 / 2 and 15 / 3: mode 0x960, 0x4b0, 0, -1, 0x961), 0 .. k perturbed samples, counts 0 .. 700, big counts
 *     (16-bit sum wrap), Detect_1s with the factor swept over its whole range for fixed data (the comparison flips at one factor: the boundary and its neighbours are probed),
 *     the tie (>= for Rmloop2_ACK, > for Detect_1s);
 *   2 Detect_v22: 160-sample blocks of: tones at and around the two detector bands (1600 / 1640 Hz -> mtd2, 1980 .. 2040 Hz -> mtd1), silence, noise, mixtures, level steps, changes at
 *     segment boundaries, saturation; shadow copies of the MTD / AGC objects run the same chain without the zeroing to count every (r1, r2) pair of the real callees;
 *     the result 1 (cA > 2: three / four mtd2 = 0 segments; cB > 2: three / four mtd1 = 0 segments) and 0, the zeroed segments, the debug line at dsplibs_debug_level 2.
 */
#include "../r04_common.h"
#include "v22/v22.h"

extern unsigned int dsplibs_debug_level;
extern void FPM_AGC_init(FpmAgc *obj, const int *cfg, int allocate);
extern void FPM_AGC_agc(FpmAgc *agc, short *buf, unsigned short n);
extern unsigned int FPM_MTD_detect(FpmMtd *obj, short *buf, short n);

enum {
  CV_R_POS = 0, CV_R_NEG, CV_R_RUN_BREAK, CV_R_TIE, CV_R_N0, CV_R_ODD_N, CV_R_BIGN, CV_R_RUN_ONLY, CV_R_SUM_ONLY, CV_R_RANDOM,
  CV_A_POS = 20, CV_A_NEG, CV_A_TIE, CV_A_MODE960, CV_A_MODEOTHER, CV_A_N0, CV_A_BIGN,
  CV_O_POS = 30, CV_O_NEG, CV_O_TIE, CV_O_MODE960, CV_O_MODEOTHER, CV_O_N0, CV_O_BIGN, CV_O_FLIP,
  CV_D_RET1 = 40, CV_D_RET0, CV_D_CA, CV_D_CB, CV_D_BOTH, CV_D_ZERO_SEG, CV_D_KEEP_SEG, CV_D_PRINT, CV_D_NOPRINT,
  CV_D_P00 = 50, CV_D_P01, CV_D_P02, CV_D_P10, CV_D_P11, CV_D_P12, CV_D_P20, CV_D_P21, CV_D_P22, CV_D_CA_RESET, CV_D_CB_RESET, CV_D_CB1_ZERO, CV_D_CB2_KEEP, CV_D_CA3_ONLY, CV_D_CB3_ONLY, CV_D_CA2, CV_D_CA1, CV_D_CB2
};

static unsigned short g_d[40000];
static unsigned short g_n;

/* ------------------------------------------------------------------------------------------------ Retrain */
static unsigned short pick_with_bits(int v)
{
  unsigned short x = (unsigned short)rnd();
  x = (unsigned short)((x & ~0xcu) | ((unsigned)(v & 3) << 2));
  return x;
}
static void gen_retrain(int n, int vx, int vy, int exact)
{
  int i;
  unsigned short e = pick_with_bits(vx), o = pick_with_bits(vy);
  for (i = 0; i < n; i++) g_d[i] = (i & 1) ? (exact ? o : pick_with_bits(vy)) : (exact ? e : pick_with_bits(vx));
}
static unsigned retrain_call(const char *tag, int show, int n)
{
  unsigned r;
  g_n = (unsigned short)n;
  r = Detect_Retrain(g_d, &g_n);
  emit(show, "%s n=%d -> %u", tag, n, r);
  if (n == 0) cov_hit(CV_R_N0);
  if (n & 1) cov_hit(CV_R_ODD_N);
  if (n > 3640) cov_hit(CV_R_BIGN);
  if (r) cov_hit(CV_R_POS); else cov_hit(CV_R_NEG);
  return r;
}
static void sc_retrain(void)
{
  int t, k, n, i;
  cov_def(CV_R_POS, "retrain: fires"); cov_def(CV_R_NEG, "retrain: does not fire"); cov_def(CV_R_N0, "retrain: n = 0"); cov_def(CV_R_ODD_N, "retrain: odd n"); cov_def(CV_R_BIGN, "retrain: n > 3640");
  cov_def(CV_R_RANDOM, "retrain: random data"); cov_def(CV_R_RUN_ONLY, "retrain: run ok, sums fail"); cov_def(CV_R_SUM_ONLY, "retrain: sums ok, run fails");
  cov_def(CV_R_RUN_BREAK, "retrain: run broken by a perturbation");
  for (t = 0; t < 12000; t++) {
    int vx = rpct(70) ? 3 : (int)rmod(4), vy = rpct(70) ? 0 : (int)rmod(4);
    char tag[16];
    unsigned r;
    n = rpct(10) ? (int)rmod(6) : rpct(70) ? rrange(6, 120) : rpct(60) ? rrange(100, 600) : (rpct(50) ? rrange(3600, 3700) : rrange(3700, 20000));
    snprintf(tag, sizeof tag, "T%d", t);
    gen_retrain(n, vx, vy, 1);
    r = retrain_call(tag, t < 40, n);
    (void)r;
    /* perturb k words, sweep k across the threshold */
    for (k = 1; k <= 12 && k <= n; k++) {
      int j;
      for (j = 0; j < k; j++) {
        int pos = (int)rmod((unsigned)n);
        g_d[pos] = pick_with_bits(rpct(50) ? 0 : (int)rmod(4));
        if (rpct(30)) g_d[pos] = (unsigned short)rnd();
      }
      r = retrain_call(tag, 0, n);
      if (!r) cov_hit(CV_R_RUN_BREAK);
    }
    if (t % 7 == 0) { /* run ok (all equal pairs) but sums fail, and sums ok but no run */
      { int a = (int)rmod(3); int b = (int)rmod(4); gen_retrain(n, a, b, 1); }
      if (!retrain_call(tag, 0, n)) cov_hit(CV_R_RUN_ONLY);
      gen_retrain(n, 3, 0, 0);
      if (!retrain_call(tag, 0, n)) cov_hit(CV_R_SUM_ONLY);
    }
    if (t % 5 == 0) {
      for (i = 0; i < n; i++) g_d[i] = (unsigned short)rnd();
      retrain_call(tag, 0, n);
      cov_hit(CV_R_RANDOM);
    }
  }
  /* the tie: n even, even words v = 3, odd words v = 0 except `m` odd words with v = 1: 2 s2 - s1 = 9 h - m, thr = (9 h * 0x7eb8) >> 15 with h = n / 2 -> sweep m 0 .. h */
  cov_def(CV_R_TIE, "retrain: |2 s2 - s1| equals the threshold");
  for (n = 2; n <= 200; n += 2) {
    int h = n / 2, m, thr = (9 * h * 0x7eb8) >> 15;
    for (m = 0; m <= h; m++) {
      char tag[16];
      gen_retrain(n, 3, 0, 1);
      for (i = 0; i < m; i++) g_d[2 * i + 1] = g_d[1]; /* keep the odd run: all odd words equal; only the SUM changes via the bits: use m words with v = 1 */
      for (i = 0; i < n; i++) if ((i & 1) && i / 2 < m) g_d[i] = (unsigned short)((g_d[i] & ~0xcu) | 4u);
      for (i = 1; i < n; i += 2) if (i / 2 >= m) g_d[i] = (unsigned short)((g_d[i] & ~0xcu));
      for (i = 1; i < n; i += 2) g_d[i] = (unsigned short)((g_d[i] & ~0xfffu) | (g_d[i] & 0xfffu)); /* (words stay random in the other bits: the run may break) */
      snprintf(tag, sizeof tag, "Y%d.%d", n, m);
      retrain_call(tag, 0, n);
      if (9 * h - m == thr || 9 * h - m == thr + 1) cov_hit(CV_R_TIE);
    }
  }
}

/* ------------------------------------------------------------------------------------------------ Rmloop2_ACK / 1s */
static void gen_k(int n, int k, int jitter)
{
  int i;
  for (i = 0; i < n; i++) g_d[i] = (unsigned short)(k + (jitter ? rrange(-jitter, jitter) : 0));
}
static unsigned ack_call(const char *tag, int show, int n, int mode)
{
  unsigned r;
  g_n = (unsigned short)n;
  r = Detect_Rmloop2_ACK(g_d, &g_n, (short)mode);
  emit(show, "%s n=%d mode=%d -> %u", tag, n, mode, r);
  if (r) cov_hit(CV_A_POS); else cov_hit(CV_A_NEG);
  if (mode == 0x960) cov_hit(CV_A_MODE960); else cov_hit(CV_A_MODEOTHER);
  if (n == 0) cov_hit(CV_A_N0);
  if (n > 600) cov_hit(CV_A_BIGN);
  return r;
}
static unsigned one_call(const char *tag, int show, int n, int mode, int factor)
{
  unsigned r;
  g_n = (unsigned short)n;
  r = Detect_1s(g_d, &g_n, (short)mode, (short)factor);
  emit(show, "%s n=%d mode=%d f=%d -> %u", tag, n, mode, factor, r);
  if (r) cov_hit(CV_O_POS); else cov_hit(CV_O_NEG);
  if (mode == 0x960) cov_hit(CV_O_MODE960); else cov_hit(CV_O_MODEOTHER);
  if (n == 0) cov_hit(CV_O_N0);
  if (n > 600) cov_hit(CV_O_BIGN);
  return r;
}
static void ack_exact_ties(void);
static void sc_ack_1s(void)
{
  static const int modes[] = {0x960, 0x960, 0x4b0, 0, -1, 0x961, 0x960, 0x2580};
  int t, n, i, f;
  cov_def(CV_A_POS, "ack: fires"); cov_def(CV_A_NEG, "ack: does not fire"); cov_def(CV_A_MODE960, "ack: mode 0x960"); cov_def(CV_A_MODEOTHER, "ack: other mode"); cov_def(CV_A_N0, "ack: n = 0");
  cov_def(CV_A_BIGN, "ack: n > 600");
  cov_def(CV_O_POS, "1s: fires"); cov_def(CV_O_NEG, "1s: does not fire"); cov_def(CV_O_MODE960, "1s: mode 0x960"); cov_def(CV_O_MODEOTHER, "1s: other mode"); cov_def(CV_O_N0, "1s: n = 0");
  cov_def(CV_O_BIGN, "1s: n > 600"); cov_def(CV_O_FLIP, "1s: the factor sweep flips the result");
  cov_def(CV_A_TIE, "ack: threshold tie probed"); cov_def(CV_O_TIE, "1s: threshold tie probed");
  for (t = 0; t < 6000; t++) {
    int mode = modes[rmod(sizeof modes / sizeof modes[0])];
    int ka = mode == 0x960 ? 10 : 2, ko = mode == 0x960 ? 15 : 3;
    char tag[16];
    int j, jit = rpct(40) ? 0 : rrange(1, 4);
    n = rpct(8) ? (int)rmod(4) : rpct(70) ? rrange(4, 120) : rpct(50) ? rrange(100, 700) : rrange(700, 12000);
    snprintf(tag, sizeof tag, "A%d", t);
    gen_k(n, ka, jit);
    ack_call(tag, t < 40, n, mode);
    for (j = 1; j <= 8 && j <= n; j++) {
      { unsigned pos = rmod((unsigned)n); g_d[pos] = (unsigned short)(rpct(50) ? ka + rrange(-6, 6) : (int)rmod(40)); }
      ack_call(tag, 0, n, mode);
    }
    if (t % 6 == 0) { for (i = 0; i < n; i++) g_d[i] = (unsigned short)rnd(); ack_call(tag, 0, n, mode); }
    snprintf(tag, sizeof tag, "O%d", t);
    gen_k(n, ko, jit);
    one_call(tag, t < 40, n, mode, 0x7eb8);
    for (j = 1; j <= 8 && j <= n; j++) {
      { unsigned pos = rmod((unsigned)n); g_d[pos] = (unsigned short)(rpct(50) ? ko + rrange(-8, 8) : (int)rmod(50)); }
      one_call(tag, 0, n, mode, rpct(60) ? 0x7eb8 : (int)(short)rnd());
    }
    if (t % 6 == 0) { for (i = 0; i < n; i++) g_d[i] = (unsigned short)rnd(); one_call(tag, 0, n, mode, (int)(short)rnd()); }
  }
  /* factor sweep: fixed data, the factor over the whole signed 16-bit range in steps (and every value around the flip) */
  for (t = 0; t < 300; t++) {
    int mode = modes[rmod(sizeof modes / sizeof modes[0])], ko = mode == 0x960 ? 15 : 3, prev = -1, flips = 0;
    char tag[16];
    n = rrange(1, 80);
    gen_k(n, ko, rrange(0, 3));
    { int cnt = 1 + (int)rmod((unsigned)n); for (i = 0; i < cnt; i++) { unsigned pos = rmod((unsigned)n); g_d[pos] = (unsigned short)(ko + rrange(-5, 5)); } }
    snprintf(tag, sizeof tag, "F%d", t);
    for (f = -32768; f <= 32767; f += 17) {
      int r = (int)one_call(tag, 0, n, mode, f) != 0;
      if (prev >= 0 && r != prev) {
        int q;
        flips++;
        for (q = f - 17; q <= f; q++) one_call(tag, 0, n, mode, q);
        cov_hit(CV_O_TIE);
      }
      prev = r;
    }
    if (flips) cov_hit(CV_O_FLIP);
    emit(t < 30, "%s flips=%d", tag, flips);
  }
  ack_exact_ties();
  /* ack tie: data K - d_i: 2 s2 - s1 = sum (K^2 - (d - K)^2); sweep the number of units off K */
  for (n = 1; n <= 60; n++) {
    int mode, m;
    for (mode = 0x960; mode >= 0x95f; mode--) {
      int ka = mode == 0x960 ? 10 : 2;
      for (m = 0; m <= n + 2; m++) {
        char tag[16];
        gen_k(n, ka, 0);
        for (i = 0; i < m && i < n; i++) g_d[i] = (unsigned short)(ka + 1);
        if (m > n) g_d[0] = (unsigned short)(ka + (m - n));
        snprintf(tag, sizeof tag, "K%d.%d", n, m);
        ack_call(tag, 0, n, mode);
        cov_hit(CV_A_TIE);
      }
    }
  }
}

/* the exact tie of Detect_Rmloop2_ACK: sum (d - K)^2 = D is swept over T - 3 .. T + 3 where T = n K^2 - ((n K^2 * 0x7eb8) >> 15) (the threshold), the deviations are built greedily from squares */
static void ack_exact_ties(void)
{
  int n, mode;
  for (n = 1; n <= 400; n++) {
    for (mode = 0x960; mode >= 0x95f; mode--) {
      int ka = mode == 0x960 ? 10 : 2, nk2 = n * ka * ka, thr = (nk2 * 0x7eb8) >> 15, T = nk2 - thr, D;
      for (D = T - 3; D <= T + 3; D++) {
        int i, rem = D;
        char tag[16];
        if (D < 0) continue;
        gen_k(n, ka, 0);
        for (i = 0; i < n && rem > 0; i++) {
          int e = 0;
          while ((e + 1) * (e + 1) <= rem) e++;
          g_d[i] = (unsigned short)(ka + e);
          rem -= e * e;
        }
        if (rem) continue;
        snprintf(tag, sizeof tag, "X%d.%d", n, D - T);
        ack_call(tag, 0, n, mode);
        cov_hit(CV_A_TIE);
      }
    }
  }
}

/* ------------------------------------------------------------------------------------------------ Detect_v22 */
static const short C1[15] = {-15099, 15739, -5530, 5533, 15739, -15099, 15739, -6137, 6141, 15739, -15099, 15736, -19474, 19484, 15736}; /* MTDv22_COEF */
static const short C2[15] = {-15099, 15739, 4920, -4924, 15739, -15099, 15739, -6137, 6141, 15739, -15099, 15736, -19474, 19484, 15736}; /* MTDv22_COEF2 */
static const short AA[2] = {16384, 32604}, AB[2] = {16384, 164};                                                                         /* AGC_DEF_ALPHA / BETA */
static short g_b1[8], g_b2[8];
static FpmMtd g_m1, g_m2;
static V22DetCore g_core;
static V22DetTones g_tones;
static V22DetEngine g_eng;

static void det_init(int cfg2, int thr_variant)
{
  int cfg[6] = {0x202ee0, 0x3e80001, 0x280a3d, 0, 0, 0x9e};
  if (!cfg2) { cfg[0] = 0x204268; cfg[2] = 0x240a3d; }
  memset(&g_m1, 0, sizeof g_m1);
  memset(&g_m2, 0, sizeof g_m2);
  memset(g_b1, 0, sizeof g_b1);
  memset(g_b2, 0, sizeof g_b2);
  g_m1.coef = C1; g_m1.c1.w = 0x747c0003; g_m1.c2.w = 0xa; g_m1.buf = g_b1;
  g_m2.coef = C2; g_m2.c1.w = 0x6d4c0003; g_m2.c2.w = 0xa; g_m2.buf = g_b2;
  if (thr_variant == 1) { g_m1.c2.w = 2; g_m2.c2.w = 0x300; } /* NOT the real V.22 configuration (both thresholds are 10): different energy thresholds make (r1, r2) = (2, x) / (x, 2) reachable */
  if (thr_variant == 2) { g_m1.c2.w = 0x300; g_m2.c2.w = 2; }
  memset(&g_core, 0, sizeof g_core);
  memset(&g_tones, 0, sizeof g_tones);
  FPM_AGC_init(&g_core.agc, cfg, 1);
  g_core.agc.alpha = AA;
  g_core.agc.beta = AB;
  g_tones.mtd1 = &g_m1;
  g_tones.mtd2 = &g_m2;
  g_eng.tones = &g_tones;
  g_eng.core = &g_core;
}
static void det_dump(int show, const char *tag)
{
  const FpmAgc *a = &g_core.agc;
  emit(show, "%s m1 dc=%d,%d e14=%d e16=%d b=%d,%d,%d,%d,%d,%d | m2 dc=%d,%d e14=%d e16=%d b=%d,%d,%d,%d,%d,%d", tag, g_m1.dc[0], g_m1.dc[1], g_m1.diffEnergy, g_m1.inEnergy, g_b1[0], g_b1[1], g_b1[2], g_b1[3], g_b1[4],
       g_b1[5], g_m2.dc[0], g_m2.dc[1], g_m2.diffEnergy, g_m2.inEnergy, g_b2[0], g_b2[1], g_b2[2], g_b2[3], g_b2[4], g_b2[5]);
  emit(show, "%s agc a=%08x,%08x,%08x w5=%d act=%d det=%d hist=%d s22=%d rcp=%d shift=%d frz=%d", tag, (unsigned)a->a.w[0], (unsigned)a->a.w[1], (unsigned)a->a.w[2], a->w5, a->active, a->detect,
       a->hist, a->s22, a->rcp, a->shift, a->freeze);
}
static short g_sig[160];
static int g_sampn;
/* integer-only sine (Bhaskara's approximation in Q15: the signals must be bit identical on i386 and x86-64, no libm): phase 0 .. 65535 -> -32767 .. 32767 */
static int isin(unsigned phase)
{
  unsigned x = phase & 0x7fffu, u = (x * (32768u - x)) >> 15, y = (16u * u << 15) / ((5u << 15) - 4u * u);
  return (phase & 0x8000u) ? -(int)y : (int)y;
}
static void gen_seg(short *p, int n, int kind, int f1, int f2, int amp, int noise)
{
  int i;
  unsigned inc1 = (unsigned)(((long long)f1 * 65536 + 3600) / 7200), inc2 = (unsigned)(((long long)f2 * 65536 + 3600) / 7200);
  for (i = 0; i < n; i++) {
    long long v = 0;
    unsigned ph1 = (unsigned)g_sampn * inc1, ph2 = (unsigned)g_sampn * inc2;
    switch (kind) {
      case 0: v = 0; break;
      case 1: v = (long long)amp * isin(ph1) / 32768; break;
      case 2: v = (long long)amp * isin(ph1) / 32768 + (long long)amp * isin(ph2) / 65536; break;
      case 3: v = rrange(-amp, amp); break;
      case 4: v = (g_sampn / 3 & 1) ? amp : -amp; break;
      default: v = (long long)amp * isin(ph1) / 8192; break; /* saturates */
    }
    if (noise > 0) v += rrange(-noise, noise);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    p[i] = (short)v;
    g_sampn++;
  }
}
static unsigned det_call(const char *tag, int show, int dbg)
{
  short shadow[160];
  FpmMtd s1 = g_m1, s2 = g_m2;
  short sb1[8], sb2[8];
  FpmAgc sa = g_core.agc;
  unsigned r;
  int seg, cA = 0, cB = 0;
  memcpy(shadow, g_sig, sizeof shadow);
  memcpy(sb1, g_b1, sizeof sb1);
  memcpy(sb2, g_b2, sizeof sb2);
  s1.buf = sb1;
  s2.buf = sb2;
  FPM_AGC_agc(&sa, shadow, 160);
  for (seg = 0; seg < 4; seg++) {
    int r1 = (short)FPM_MTD_detect(&s1, shadow + 40 * seg, 40), r2 = (short)FPM_MTD_detect(&s2, shadow + 40 * seg, 40);
    if (r1 >= 0 && r1 <= 2 && r2 >= 0 && r2 <= 2) cov_hit(CV_D_P00 + r1 * 3 + r2);
    if (r2 != 0) { if (cA > 0) cov_hit(CV_D_CA_RESET); cA = 0; } else cA++;
    if (r1 != 0) { if (cB > 0) cov_hit(CV_D_CB_RESET); cB = 0; } else { cB++; if (cB <= 1) cov_hit(CV_D_CB1_ZERO); else cov_hit(CV_D_CB2_KEEP); }
  }
  dsplibs_debug_level = dbg;
  r = Detect_v22(&g_eng, g_sig);
  dsplibs_debug_level = 0;
  emit(show, "%s -> %u out=%016llx", tag, r, fnv_shorts(g_sig, 160));
  if (r) cov_hit(CV_D_RET1); else cov_hit(CV_D_RET0);
  if (cA == 2) cov_hit(CV_D_CA2);
  if (cA == 1) cov_hit(CV_D_CA1);
  if (cB == 2) cov_hit(CV_D_CB2);
  if (cA > 2) cov_hit(CV_D_CA);
  if (cB > 2) cov_hit(CV_D_CB);
  if (cA > 2 && cB > 2) cov_hit(CV_D_BOTH);
  if (cA > 2 && cB <= 2) cov_hit(CV_D_CA3_ONLY);
  if (cB > 2 && cA <= 2) cov_hit(CV_D_CB3_ONLY);
  {
    int z = 0;
    for (seg = 0; seg < 160; seg++) z += g_sig[seg] == 0;
    if (z) cov_hit(CV_D_ZERO_SEG);
    if (z < 160) cov_hit(CV_D_KEEP_SEG);
  }
  if (dbg > 1 && cA > 2) cov_hit(CV_D_PRINT); else cov_hit(CV_D_NOPRINT);
  return r;
}
static void sc_detect(void)
{
  static const int fs[] = {1600, 1620, 1640, 1980, 2000, 2020, 2040, 1500, 1700, 1900, 2100, 100, 3600, 1200, 2400, 900, 3000};
  int t, c, seg;
  cov_def(CV_D_RET1, "v22: returns 1"); cov_def(CV_D_RET0, "v22: returns 0"); cov_def(CV_D_CA, "v22: cA > 2"); cov_def(CV_D_CB, "v22: cB > 2"); cov_def(CV_D_BOTH, "v22: both counters > 2");
  cov_def(CV_D_ZERO_SEG, "v22: zeroed samples"); cov_def(CV_D_KEEP_SEG, "v22: kept samples"); cov_def(CV_D_PRINT, "v22: debug line printed"); cov_def(CV_D_NOPRINT, "v22: no debug line");
  cov_def(CV_D_P00, "v22: (r1, r2) = (0, 0)"); cov_def(CV_D_P01, "v22: (0, 1)"); cov_def(CV_D_P02, "v22: (0, 2)"); cov_def(CV_D_P10, "v22: (1, 0)"); cov_def(CV_D_P11, "v22: (1, 1)");
  cov_def(CV_D_P12, "v22: (1, 2)"); cov_def(CV_D_P20, "v22: (2, 0)"); cov_def(CV_D_P21, "v22: (2, 1)"); cov_def(CV_D_P22, "v22: (2, 2)");
  cov_def(CV_D_CA_RESET, "v22: cA reset by mtd2 != 0"); cov_def(CV_D_CB_RESET, "v22: cB reset by mtd1 != 0"); cov_def(CV_D_CB1_ZERO, "v22: first mtd1 = 0 segment zeroed");
  cov_def(CV_D_CB2_KEEP, "v22: later mtd1 = 0 segments kept"); cov_def(CV_D_CA3_ONLY, "v22: cA > 2, cB <= 2"); cov_def(CV_D_CB3_ONLY, "v22: cB > 2, cA <= 2"); cov_def(CV_D_CA2, "v22: cA == 2 at the end"); cov_def(CV_D_CA1, "v22: cA == 1 at the end"); cov_def(CV_D_CB2, "v22: cB == 2 at the end");
  for (t = 0; t < 300; t++) {
    char tag[32];
    int kind = t % 12;
    int amp = (int[]){300, 1500, 8000, 20000, 32000, 60, 4000}[rmod(7)];
    int f1 = fs[rmod(sizeof fs / sizeof fs[0])], f2 = fs[rmod(sizeof fs / sizeof fs[0])];
    snprintf(tag, sizeof tag, "V%d", t);
    det_init(t % 3 != 0, t % 10 >= 7 ? 1 + (t % 2) : 0);
    g_sampn = 0;
    for (c = 0; c < 60; c++) {
      /* each block: one signal kind, or a change at a segment boundary */
      int split = rpct(25);
      for (seg = 0; seg < 4; seg++) {
        int k = kind;
        if (split && seg >= 2) { k = (int)rmod(6); f1 = fs[rmod(sizeof fs / sizeof fs[0])]; }
        if (kind == 11) k = (int)rmod(6);
        if (kind < 6) k = rpct(80) ? 1 : (int)rmod(6);
        { int fj = rpct(30) ? rrange(-30, 30) : 0; int am = rpct(10) ? amp / 5 : amp; int nz = rpct(25) ? amp / 20 : 0; gen_seg(g_sig + 40 * seg, 40, k % 6, f1 + fj, f2, am, nz); }
      }
      if (rpct(5)) { int i; for (i = 0; i < 160; i++) g_sig[i] = rshort(); }
      if (rpct(20)) { f1 = fs[rmod(sizeof fs / sizeof fs[0])]; amp = (int[]){300, 1500, 8000, 20000, 60}[rmod(5)]; }
      det_call(tag, t < 6 && c < 12, rpct(30) ? 2 : (int)rmod(2));
      if (c % 10 == 0) det_dump(0, tag);
    }
    det_dump(t < 10, tag);
  }
  /* every segment of a block independent: tone (1600 / 2020 band), silence or noise, so the counters see every run length 0 .. 4 (cA == 2, cB == 2 ...) */
  for (t = 0; t < 200; t++) {
    char tag[32];
    snprintf(tag, sizeof tag, "Z%d", t);
    det_init(t % 2, 0);
    g_sampn = 0;
    for (c = 0; c < 60; c++) {
      for (seg = 0; seg < 4; seg++) {
        int pick = (int)rmod(5);
        int fz = pick < 2 ? 1620 : pick < 4 ? 2020 : 1000;
        gen_seg(g_sig + 40 * seg, 40, pick == 4 ? (rpct(50) ? 0 : 3) : 1, fz, 0, 8000, 0);
      }
      det_call(tag, 0, rpct(30) ? 2 : 0);
    }
    det_dump(t < 3, tag);
  }
  /* deterministic runs that hit the two long-tone branches */
  for (t = 0; t < 40; t++) {
    char tag[32];
    int f = fs[t % 7];
    snprintf(tag, sizeof tag, "W%d", t);
    det_init(t % 2, 0);
    g_sampn = 0;
    for (c = 0; c < 20; c++) {
      gen_seg(g_sig, 160, 1, f, 0, t % 5 == 0 ? 30000 : 8000, 0);
      det_call(tag, t < 4, 2);
    }
    det_dump(t < 4, tag);
  }
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("R05_SEED");
  if (seed) g_lcg ^= (unsigned)strtoul(seed, NULL, 0) * 2654435761u;
  g_lcg ^= (unsigned)sc * 0x9e3779b9u;
  switch (sc) {
    case 0: sc_retrain(); break;
    case 1: sc_ack_1s(); break;
    case 2: sc_detect(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  cov_finish();
  return 0;
}
