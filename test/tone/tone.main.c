/*
 * R09 differential harness: toneiir_* / cadence_* / GetFP_Value / Get_Detection_Threshold_Table - real dsplibs.o vs src/tone/tone_iir.c + tone_cadence.c + tone_tables.c + tone_table_words.c.
 * Library test/tone/r09_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation, deterministic heap, callee recorders sysdep_* / modem_get_param /
 * dsplibs_debug_printf); the typed objects of src/tone/tone_detect.h on both sides (the ToneIir layout is the blob's on every host), the 32-bit BLOB run is the x86-64 golden.
 * No libm: every signal is integer (an integer two-pole resonator at the 8 kHz frequencies of OSC_HZ, an LCG for noise).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 pure functions: GetFP_Value over a grid and random pairs (all sign combinations, the 16-bit truncation of the count, a == 0), Get_Detection_Threshold_Table for every
 *     u8 level (inside the table, in front of it, behind it), toneiir_get_default_configuration;
 *   1 toneiir lifecycle: toneiir_create with random configs (tables of every set, random coefficient arrays, na / nb of every clamp class, windows, integration lengths, NULL cfg,
 *     NULL / fresh / reused object, malloc failure, debug levels), reset, delete, dumps of the whole object (the uninitialised e1 -> p copy included);
 *   2 toneiir_progress: tones / noise / silence / steps through every table set (the CP_* sets, every Filter_* row, random coefficients), windows 1 .. 160, every window end dumped,
 *     every branch of the window decision (hit / miss-run / reset / hold, the three conditions, the level) observed from the counters;
 *   3 cadence_create: every tone type x filter index x sub-index x parameter variant (zero limits, invalid buffer length, loose, levels), extra, object reuse, malloc failures, debug levels;
 *   4 cadence_progress on cadence patterns (busy / ring-back / congestion / dial detectors fed with tone bursts: matching, jittered, mismatching, alternating, no-answer, series);
 *   5 cadence_progress fuzz: random object state (state, cnt, idx, cycles, on / off arrays, series values, loose, flags) with tiny windows and random bursts.
 * R09_VERBOSE=1 prints every line (debugging), R09_SEED selects another input stream.
 */
#include "r09_common.h"

/* integer resonator: y[n] = (K * y[n-1] >> 14) - y[n-2] */
static const int OSC_HZ[21] = {100, 200, 276, 300, 350, 400, 425, 440, 480, 500, 520, 550, 600, 620, 630, 700, 800, 1000, 1500, 2000, 3000};
static const int OSC_K[21] = {32667, 32365, 32001, 31863, 31538, 31164, 30959, 30831, 30467, 30274, 30073, 29758, 29197, 28959, 28838, 27939, 26510, 23170, 12540, 0, -23170};
static const int OSC_S[21] = {1285, 2563, 3524, 3825, 4447, 5063, 5368, 5550, 6031, 6270, 6507, 6859, 7438, 7667, 7780, 8561, 9630, 11585, 15137, 16384, 11585};
#define NOSC 21
typedef struct {
  int k, y1, y2;
} Osc;
static void osc_init(Osc *o, int fi, int amp)
{
  o->k = OSC_K[fi];
  o->y2 = 0;
  o->y1 = (int)(((long long)amp * OSC_S[fi]) >> 14);
}
static short osc_next(Osc *o)
{
  int out = o->y1, y = (int)(((long long)o->k * o->y1) >> 14) - o->y2;
  o->y2 = o->y1;
  o->y1 = y;
  return (short)out;
}
static short noise(int amp) { return amp ? (short)((int)rmod((unsigned)(2 * amp + 1)) - amp) : 0; }

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_FP_PP, CV_FP_PN, CV_FP_NP, CV_FP_NN, CV_FP_ZERO, CV_FP_WRAP, CV_THR_IN, CV_THR_BEFORE, CV_THR_AFTER, CV_DEFCFG,
  CV_C_NULLOBJ, CV_C_REUSE, CV_C_NULLCFG, CV_C_FAIL, CV_C_CLAMP_NA, CV_C_CLAMP_NB, CV_C_NEG_N, CV_C_DBG, CV_C_TABLES, CV_C_RANDTAB, CV_C_RESET, CV_C_DELETE, CV_C_INTEG_NEG,
  CV_P_RET0, CV_P_RET1, CV_P_RET2, CV_P_HIT, CV_P_MISSRUN, CV_P_HOLDRST, CV_P_NOCHANGE, CV_P_F1FAIL, CV_P_F2FAIL, CV_P_F3FAIL, CV_P_LEVELFAIL, CV_P_ALLPASS_T, CV_P_SHIFTMASK, CV_P_POSOUT, CV_P_NEGSCALE, CV_P_E1WRAP,
  CV_K_TYPE0, CV_K_TYPE1, CV_K_TYPE2, CV_K_TYPE3, CV_K_TYPE4, CV_K_TYPE9, CV_K_FILT0, CV_K_FILT1, CV_K_FILT2, CV_K_FILT3, CV_K_FILT4, CV_K_FILT5, CV_K_FILT6, CV_K_FILT7, CV_K_FILT8,
  CV_K_ROW, CV_K_NOROW, CV_K_FILTER_TAB, CV_K_CP_TAB, CV_K_ZEROLIM0, CV_K_ZEROLIM2, CV_K_ZEROLIM3, CV_K_DISABLED2, CV_K_DISABLED3, CV_K_BUFINVALID, CV_K_LOOSE, CV_K_INTEGHI, CV_K_EXTRA,
  CV_K_REUSE_TI, CV_K_MALLOC_FAIL1, CV_K_MALLOC_FAIL2, CV_K_TIFAIL, CV_K_DBG, CV_K_TYPECLAMP,
  CV_R_R2_FLAG, CV_R_R2_ST0_RESET, CV_R_R2_ST0_CNT, CV_R_R2_ST1, CV_R_R1_ST1, CV_R_R1_RESETOFF, CV_R_R1_ST0, CV_R_CYCLE, CV_R_JUDGE_NORMAL, CV_R_JUDGE_SERIES, CV_R_JUDGE_LOOSE,
  CV_R_COND_B, CV_R_COND_C, CV_R_COND_SER, CV_R_COND_LOOSE, CV_R_RECOG, CV_R_NOANSWER, CV_R_IDXWRAP, CV_R_RET1, CV_R_RET7, CV_R_TOLLOW, CV_R_NOCOND, CV_R_COND_C_ONLY, CV_R_LOOSE_FAIL_N,
  CV_R_LOOSE_FAIL_END, CV_R_SER_FAIL, CV_R_B_FAIL, CV_R_DBGMSG
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_FP_PP, "GetFP_Value a*b > 0 (both +)");
      cov_def(CV_FP_NN, "GetFP_Value both negative");
      cov_def(CV_FP_PN, "GetFP_Value a > 0, b < 0");
      cov_def(CV_FP_NP, "GetFP_Value a < 0, b > 0");
      cov_def(CV_FP_ZERO, "GetFP_Value a == 0");
      cov_def(CV_FP_WRAP, "GetFP_Value count > 32767 (16-bit truncation)");
      cov_def(CV_THR_IN, "threshold level inside the table (30..45)");
      cov_def(CV_THR_BEFORE, "threshold level 46..255 (in front of the table)");
      cov_def(CV_THR_AFTER, "threshold level 0..29 (behind the table)");
      cov_def(CV_DEFCFG, "default configuration dumped");
      break;
    case 1:
      cov_def(CV_C_NULLOBJ, "toneiir_create obj NULL (alloc)");
      cov_def(CV_C_REUSE, "toneiir_create reused object");
      cov_def(CV_C_NULLCFG, "toneiir_create cfg NULL");
      cov_def(CV_C_FAIL, "toneiir_create malloc failure");
      cov_def(CV_C_CLAMP_NA, "na clamped (> 24)");
      cov_def(CV_C_CLAMP_NB, "nb clamped (> 24)");
      cov_def(CV_C_NEG_N, "na / nb <= 0");
      cov_def(CV_C_DBG, "toneiir_create debug print");
      cov_def(CV_C_TABLES, "table-set configs");
      cov_def(CV_C_RANDTAB, "random coefficient configs");
      cov_def(CV_C_RESET, "toneiir_reset");
      cov_def(CV_C_DELETE, "toneiir_delete");
      cov_def(CV_C_INTEG_NEG, "negative integration");
      break;
    case 2:
      cov_def(CV_P_RET0, "progress mid-window");
      cov_def(CV_P_RET1, "progress window end, no tone (1)");
      cov_def(CV_P_RET2, "progress window end, tone (2)");
      cov_def(CV_P_HIT, "window: hits++ run++");
      cov_def(CV_P_MISSRUN, "window: run > miss -> hits++, run = 0");
      cov_def(CV_P_HOLDRST, "window: hold == 0 -> counters cleared");
      cov_def(CV_P_NOCHANGE, "window: counters unchanged");
      cov_def(CV_P_F1FAIL, "window: only f1 (e1 >= e0/4) fails");
      cov_def(CV_P_F2FAIL, "window: only f2 fails");
      cov_def(CV_P_F3FAIL, "window: only f3 fails");
      cov_def(CV_P_LEVELFAIL, "window: conditions hold, level fails");
      cov_def(CV_P_SHIFTMASK, "shift count >= 32 / negative (mod 32)");
      cov_def(CV_P_POSOUT, "final scale > 0");
      cov_def(CV_P_NEGSCALE, "final scale <= 0");
      break;
    case 3:
      cov_def(CV_K_TYPE0, "cadence_create type 0 (busy)");
      cov_def(CV_K_TYPE1, "type 1 (dial)");
      cov_def(CV_K_TYPE2, "type 2 (congestion)");
      cov_def(CV_K_TYPE3, "type 3 (ringback)");
      cov_def(CV_K_TYPE4, "type 4");
      cov_def(CV_K_TYPE9, "type >= 5");
      cov_def(CV_K_FILT0, "filter index 0");
      cov_def(CV_K_FILT1, "filter index 1");
      cov_def(CV_K_FILT2, "filter index 2");
      cov_def(CV_K_FILT3, "filter index 3");
      cov_def(CV_K_FILT4, "filter index 4");
      cov_def(CV_K_FILT5, "filter index 5");
      cov_def(CV_K_FILT6, "filter index 6");
      cov_def(CV_K_FILT7, "filter index 7");
      cov_def(CV_K_FILT8, "filter index > 7 / negative");
      cov_def(CV_K_ROW, "sub-index 1..7 (a Filter_* row)");
      cov_def(CV_K_NOROW, "sub-index outside 1..7");
      cov_def(CV_K_FILTER_TAB, "Filter_* table chosen");
      cov_def(CV_K_CP_TAB, "CP_* table chosen");
      cov_def(CV_K_ZEROLIM0, "busy limits zero -> defaults");
      cov_def(CV_K_DISABLED2, "congestion disabled");
      cov_def(CV_K_DISABLED3, "ringback disabled");
      cov_def(CV_K_BUFINVALID, "buffer length 0 -> 0x29a");
      cov_def(CV_K_LOOSE, "busy loose detection");
      cov_def(CV_K_INTEGHI, "integration > 100");
      cov_def(CV_K_EXTRA, "extra > 0");
      cov_def(CV_K_REUSE_TI, "existing object with ToneIir");
      cov_def(CV_K_MALLOC_FAIL1, "cadence malloc failure");
      cov_def(CV_K_MALLOC_FAIL2, "ToneIir malloc failure");
      cov_def(CV_K_DBG, "debug prints");
      cov_def(CV_K_TYPECLAMP, "type written back clamped");
      break;
    case 6:
      cov_def(CV_R_R2_ST0_CNT, "r == 2, state 0, count");
      cov_def(CV_R_R2_ST1, "r == 2, state 1 -> off[idx]");
      cov_def(CV_R_R1_ST1, "r == 1, state 1");
      cov_def(CV_R_R1_ST0, "r == 1, state 0 -> on[idx]");
      cov_def(CV_R_JUDGE_NORMAL, "judge normal");
      cov_def(CV_R_JUDGE_SERIES, "judge series");
      cov_def(CV_R_JUDGE_LOOSE, "judge loose");
      cov_def(CV_R_COND_B, "condition B");
      cov_def(CV_R_COND_C, "condition C (loose)");
      cov_def(CV_R_COND_C_ONLY, "normal: C holds, B fails (model)");
      cov_def(CV_R_COND_SER, "condition series");
      cov_def(CV_R_RECOG, "cadence recognised (reset)");
      cov_def(CV_R_RET1, "returns 1");
      cov_def(CV_R_TOLLOW, "tolerance < 3 raised");
      cov_def(CV_R_NOCOND, "judged, no condition");
      break;
    case 4:
    case 5:
      cov_def(CV_R_R2_FLAG, "r == 2 with flag26c");
      cov_def(CV_R_R2_ST0_RESET, "r == 2, state 0, maxOn == 0 -> restart");
      cov_def(CV_R_R2_ST0_CNT, "r == 2, state 0, count");
      cov_def(CV_R_R2_ST1, "r == 2, state 1 -> off[idx]");
      cov_def(CV_R_R1_ST1, "r == 1, state 1");
      cov_def(CV_R_R1_RESETOFF, "r == 1, state 1, count > resetOff");
      cov_def(CV_R_R1_ST0, "r == 1, state 0 -> on[idx]");
      cov_def(CV_R_CYCLE, "cycle counter");
      cov_def(CV_R_JUDGE_NORMAL, "judge normal");
      cov_def(CV_R_JUDGE_SERIES, "judge series");
      cov_def(CV_R_JUDGE_LOOSE, "judge loose");
      cov_def(CV_R_COND_B, "condition B");
      cov_def(CV_R_COND_C, "condition C (loose)");
      cov_def(CV_R_COND_SER, "condition series");
      cov_def(CV_R_RECOG, "cadence recognised (reset)");
      cov_def(CV_R_NOANSWER, "no answer (7)");
      if (scen == 5) cov_def(CV_R_IDXWRAP, "idx wrap"); /* 60 unrecognised on-periods in a row: the fuzz sets idx near the end */
      cov_def(CV_R_RET1, "returns 1");
      cov_def(CV_R_RET7, "returns 7");
      cov_def(CV_R_TOLLOW, "tolerance < 3 raised");
      cov_def(CV_R_NOCOND, "judged, no condition");
      break;
  }
}

/* ---------------------------------------------------------------------------------------------- tables / dumps */
static short g_rc_a[12], g_rc_b[12], g_rc_s[8];
static int g_rc_hot;
static const short *g_set_a[40], *g_set_b[40], *g_set_s[40];
static const char *g_set_name[40];
static int g_nset;
static void set_add(const char *n, const short *a, const short *b, const short *s)
{
  g_set_name[g_nset] = n;
  g_set_a[g_nset] = a;
  g_set_b[g_nset] = b;
  g_set_s[g_nset] = s;
  g_nset++;
}
static void tabs_init(void)
{
  ToneIirCfg dc;
  g_nhtab = 0;
  g_nset = 0;
#define HCP(n) htab_add(#n "_a", n##_a, 24); htab_add(#n "_b", n##_b, 24); htab_add(#n "_scales", n##_scales, 10); set_add(#n, n##_a, n##_b, n##_scales);
  HCP(CP_450_630) HCP(CP_276_504) HCP(CP_100_550) HCP(CP_350_600)
#define HFL(n) htab_add(#n "_a", n##_a, 168); htab_add(#n "_b", n##_b, 168); htab_add(#n "_scales", n##_scales, 70); \
  { int r_; for (r_ = 0; r_ < 7; r_++) set_add(#n, n##_a + 12 * r_, n##_b + 12 * r_, n##_scales + 5 * r_); }
  HFL(Filter_350_500) HFL(Filter_100_550) HFL(Filter_276_504)
  toneiir_get_default_configuration(&dc);
  htab_add("default_a", tone_ptr(dc.a_w), 24);
  htab_add("default_b", tone_ptr(dc.b_w), 24);
  htab_add("default_scales", tone_ptr(dc.scales_w), 10);
  htab_add("allpass_b", tone_ptr(toneiir_configuration_allpass.b_w), 2);
  htab_add("rand_a", g_rc_a, sizeof g_rc_a);
  htab_add("rand_b", g_rc_b, sizeof g_rc_b);
  htab_add("rand_s", g_rc_s, sizeof g_rc_s);
}
static void dump_cfg(const char *tag, const ToneIirCfg *c)
{
  char a[48], b[48], s[48];
  emit(0, "%s a=%s b=%s na=%d nb=%d win=%d ratio=%d mode=%d level=%d lhi=%d integ=%d miss=%d hold=%d sc=%s", tag, dword(c->a_w, a, sizeof a), dword(c->b_w, b, sizeof b), c->na, c->nb,
       c->win, c->ratio, c->mode, c->level, c->levelHi, c->integ, c->miss, c->hold, dword(c->scales_w, s, sizeof s));
}
static void dump_ti(const char *tag, const ToneIir *t)
{
  if (!t) {
    emit(0, "%s NULL", tag);
    return;
  }
  dump_cfg(tag, &t->cfg);
  emit(0, "%s cnt=%d e0=%d e1=%d p=%d pad=%d integThr=%d hits=%d run=%d w=%016llx y=%016llx", tag, t->cnt, t->e0, t->e1, t->p, t->pad9a, t->integThr, t->hits, t->run,
       fnv_shorts(t->w, 25), fnv_shorts(t->y, 25));
}
static void dump_cad(const char *tag, const Cadence *c)
{
  char a[48], b[48], s[48];
  if (!c) {
    emit(0, "%s NULL", tag);
    return;
  }
  emit(0, "%s ti=b%u state=%d cnt=%d idx=%d cycles=%d lim=%d/%d/%d/%d N=%d f26c=%d resetOff=%d w274=%d w278=%d w27c=%d ord=%d/%d tabs=%s,%s,%s", tag, blk_id_of(c->ti), c->state, c->cnt,
       c->idx, c->cycles, c->maxOn, c->maxOff, c->minOn, c->minOff, c->maxCycles, c->reportTone, c->resetOff, c->scaledLimitA, c->scaledLimitB, c->callerParamA, c->ordA, c->ordB,
       dptr(c->tabA, a, sizeof a), dptr(c->tabB, b, sizeof b), dptr(c->tabScales, s, sizeof s));
  emit(0, "%s thr=%d integ=%d buf=%d name=%s w2a4=%d allpass=%d loose=%d ser=%d,%d,%d,%d w2c0=%016llx series=%d/%d modem=%d on=%016llx off=%016llx pad=%016llx", tag, c->thr, c->integ,
       c->bufLen, c->name ? c->name : "NULL", c->callerParamB, c->allpass, c->loose, c->ser[0], c->ser[1], c->ser[2], c->ser[3], fnv_ints(c->reservedB, 4), c->seriesFlag, c->seriesMin,
       c->modem == FAKEM, fnv_ints(c->on, 60), fnv_ints(c->off, 60), fnv_ints(c->reservedA, 25));
  if (c->ti) dump_ti(tag, c->ti);
}

/* ---------------------------------------------------------------------------------------------- scenario 0 */
static void scen0(void)
{
  static const int A[] = {0, 1, -1, 2, 3, -3, 5, 7, 8, 10, 16, 100, 160, 255, 666, 1000, 4096, 16384, 32767, -32768, 12345, -777, 20000, -20000};
  static const int B[] = {1, -1, 2, 3, -3, 7, 160, 666, 1000, -1000, 16384, 32767, -32768, 9999, 4, 5000, -123};
  unsigned i, j;
  int v, n;
  ToneIirCfg dc;
  for (i = 0; i < sizeof A / sizeof A[0]; i++)
    for (j = 0; j < sizeof B / sizeof B[0]; j++) {
      short a = (short)A[i], b = (short)B[j];
      long long cnt = ((long long)(A[i] < 0 ? -A[i] : A[i]) << 14) / (B[j] < 0 ? -B[j] : B[j]);
      if (cnt > 6000000) continue;
      v = GetFP_Value(a, b);
      emit(0, "FP %d %d -> %d", a, b, v);
      if (a == 0) cov_hit(CV_FP_ZERO);
      else if (a > 0 && b > 0) cov_hit(CV_FP_PP);
      else if (a > 0 && b < 0) cov_hit(CV_FP_PN);
      else if (a < 0 && b > 0) cov_hit(CV_FP_NP);
      else cov_hit(CV_FP_NN);
      if (cnt > 32767) cov_hit(CV_FP_WRAP);
    }
  for (i = 0; i < 4000; i++) {
    short a = (short)(rnd() >> 8), b;
    long long cnt;
    switch (rmod(4)) {
      case 0: a = (short)rrange(-40, 40); break;
      case 1: a = (short)rrange(-2000, 2000); break;
      default: break;
    }
    do b = (short)(rnd() >> 4); while (b == 0);
    if (rpct(30)) b = (short)rrange(-300, 300);
    if (b == 0) b = 7;
    cnt = ((long long)(a < 0 ? -(int)a : a) << 14) / (b < 0 ? -(int)b : b);
    if (cnt > 200000) continue;
    v = GetFP_Value(a, b);
    emit(0, "FP %d %d -> %d", a, b, v);
    if (a == 0) cov_hit(CV_FP_ZERO);
    else if (a > 0 && b > 0) cov_hit(CV_FP_PP);
    else if (a > 0 && b < 0) cov_hit(CV_FP_PN);
    else if (a < 0 && b > 0) cov_hit(CV_FP_NP);
    else cov_hit(CV_FP_NN);
    if (cnt > 32767) cov_hit(CV_FP_WRAP);
  }
  for (n = 0; n < 256; n++) {
    v = Get_Detection_Threshold_Table((short)n);
    emit(0, "THR %d -> %d", n, v);
    if (n >= 30 && n <= 45) cov_hit(CV_THR_IN);
    else if (n > 45) cov_hit(CV_THR_BEFORE);
    else cov_hit(CV_THR_AFTER);
  }
  toneiir_get_default_configuration(&dc);
  dump_cfg("DEFCFG", &dc);
  cov_hit(CV_DEFCFG);
  dump_cfg("ALLPASS", &toneiir_configuration_allpass);
}

/* ---------------------------------------------------------------------------------------------- scenario 1 */
static void rand_coefs(void)
{
  int i;
  for (i = 0; i < 12; i++) {
    g_rc_a[i] = (short)(rpct(50) ? rrange(-9000, 9000) : (short)rnd());
    g_rc_b[i] = (short)(rpct(50) ? rrange(-9000, 9000) : (short)rnd());
  }
  for (i = 0; i < 3; i++) {
    g_rc_a[3 * i] = 8192;
    g_rc_b[3 * i] = (short)rrange(2000, 9000);
  }
  for (i = 0; i < 8; i++) {
    static const short SP[] = {-1, -2, 31, 32, 33, -32, -33, 64};
    int r = (int)rmod(100);
    g_rc_s[i] = (short)(r < 55 ? rrange(0, 8) : r < 70 ? rrange(-5, 70) : r < 85 ? SP[rmod(sizeof SP / sizeof SP[0])] : (short)rnd());
  }
  g_rc_hot = rpct(25);
  if (g_rc_hot) { /* identity sections (a = {1, 0, 0}, b = {1, 0, 0} in Q13): the output follows the input, a full-scale input drives the energies over 32767 (near the 16-bit limit: e1 = 327 + (e1 * 16220 >> 14) has the fixed point 32767, so e1 never wraps negative) */
    for (i = 0; i < 4; i++) {
      g_rc_a[3 * i] = 8192;
      g_rc_a[3 * i + 1] = g_rc_a[3 * i + 2] = 0;
      g_rc_b[3 * i] = 8192;
      g_rc_b[3 * i + 1] = g_rc_b[3 * i + 2] = 0;
    }
    for (i = 0; i < 4; i++) g_rc_s[i] = 0;
    g_rc_s[4] = (short)rrange(-1, 1);
  }
}
static void mkcfg(ToneIirCfg *c, int *tabkind)
{
  int s = rrange(0, g_nset - 1);
  static const int NN[] = {0, 1, 2, 5, 11, 12, 13, 20, 24, 25, 26, 40, 100, -1, -3};
  memset(c, 0, sizeof *c);
  if (rpct(75)) {
    c->a_w = tone_word(g_set_a[s]);
    c->b_w = tone_word(g_set_b[s]);
    c->scales_w = tone_word(g_set_s[s]);
    *tabkind = 0;
  } else {
    rand_coefs();
    c->a_w = tone_word(g_rc_a);
    c->b_w = tone_word(g_rc_b);
    c->scales_w = tone_word(g_rc_s);
    *tabkind = 1;
  }
  c->na = NN[rmod(sizeof NN / sizeof NN[0])];
  c->nb = NN[rmod(sizeof NN / sizeof NN[0])];
  c->win = rpct(10) ? rrange(1, 32000) : rrange(1, 200);
  if (rpct(30)) c->win = rrange(1, 12);
  c->ratio = (short)(rpct(60) ? 10571 : rpct(50) ? rrange(0, 16384) : (short)rnd());
  c->mode = (short)(rpct(60) ? 2 : (short)rnd());
  c->level = (short)(rpct(50) ? rrange(20, 600) : (short)rnd());
  c->levelHi = (short)(rpct(60) ? 0 : (short)rnd());
  c->integ = rpct(50) ? rrange(0, 4000) : rpct(50) ? rrange(-100, 100000) : (int)rnd();
  c->miss = rpct(20) ? rrange(-3, -1) : rpct(60) ? rrange(0, 6) : rrange(-3, 2000);
  c->hold = rpct(50) ? 0 : (int)rmod(3);
}
static void scen1(void)
{
  int trial;
  for (trial = 0; trial < 700; trial++) {
    ToneIirCfg cfg;
    ToneIir *t = NULL, *t2;
    int tabkind, mode = (int)rmod(6), k;
    dsplibs_debug_level = lvl_value(rpct(60) ? 0 : (int)rmod(7));
    mkcfg(&cfg, &tabkind);
    cov_hit(tabkind ? CV_C_RANDTAB : CV_C_TABLES);
    if (cfg.na > 24) cov_hit(CV_C_CLAMP_NA);
    if (cfg.nb > 24) cov_hit(CV_C_CLAMP_NB);
    if (cfg.na <= 0 || cfg.nb <= 0) cov_hit(CV_C_NEG_N);
    if (cfg.integ < 0) cov_hit(CV_C_INTEG_NEG);
    if (dsplibs_debug_level > 1) cov_hit(CV_C_DBG);
    emit(0, "TRIAL %d mode %d", trial, mode);
    dump_cfg("IN", &cfg);
    if (mode == 5) { /* the allocator fails */
      g_fail_malloc = 0;
      t = toneiir_create(NULL, &cfg);
      emit(0, "CREATE-FAIL %s", t ? "obj" : "NULL");
      if (t) die3("toneiir_create returned an object with a failing allocator");
      cov_hit(CV_C_FAIL);
      trial_end("tone fail");
      continue;
    }
    if (mode == 4) { /* cfg NULL: the default configuration */
      t = toneiir_create(NULL, NULL);
      cov_hit(CV_C_NULLCFG);
      cov_hit(CV_C_NULLOBJ);
    } else {
      t = toneiir_create(NULL, &cfg);
      cov_hit(CV_C_NULLOBJ);
    }
    if (!t) die3("toneiir_create failed");
    dump_ti("T1", t);
    for (k = 0; k < 5; k++) { /* a few samples (the arithmetic is covered by scenario 2) */
      short x = (short)(rpct(50) ? noise(3000) : (short)rnd());
      int r = toneiir_progress(t, x);
      emit(0, "P %d %d", x, r);
    }
    dump_ti("T1p", t);
    if (mode <= 1) { /* reuse: create again over the dirty object (p receives its e1) */
      ToneIirCfg c2;
      int tk;
      mkcfg(&c2, &tk);
      t2 = toneiir_create(t, &c2);
      cov_hit(CV_C_REUSE);
      if (t2 != t) die3("toneiir_create did not reuse the object");
      dump_ti("T2", t2);
    }
    if (mode == 2) {
      toneiir_reset(t);
      cov_hit(CV_C_RESET);
      dump_ti("T1r", t);
    }
    toneiir_delete(t);
    cov_hit(CV_C_DELETE);
    trial_end("tone create");
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 2 */
typedef struct {
  const short *a, *b, *s;
  int na, nb;
} CoefSet;

static void window_cov(const ToneIir *t, int hits0, int run0, int r)
{
  if (r == 0) {
    cov_hit(CV_P_RET0);
    return;
  }
  cov_hit(r == 2 ? CV_P_RET2 : CV_P_RET1);
  if (t->hits == hits0 + 1 && t->run == run0 + 1) cov_hit(CV_P_HIT);
  else if (t->hits == hits0 + 1 && t->run == 0) cov_hit(CV_P_MISSRUN);
  else if (t->hits == 0 && t->run == 0 && (hits0 | run0)) cov_hit(CV_P_HOLDRST);
  else if (t->hits == hits0 && t->run == run0) cov_hit(CV_P_NOCHANGE);
}
/* the window's three conditions from the energies of the closing sample (coverage classes only; the comparison with the blob is the dumps) */
static void cond_cov(short e0, short e1, short p, short ratio, short level)
{
  {
  int f1 = e1 >= (e0 >> 2), f2 = e1 >= (((int)p * ratio) >> 14), f3 = p >= (((int)e1 * ratio) >> 14), lv = e1 >= level;
  if (!f1 && f2 && f3) cov_hit(CV_P_F1FAIL);
  if (f1 && !f2 && f3) cov_hit(CV_P_F2FAIL);
  if (f1 && f2 && !f3) cov_hit(CV_P_F3FAIL);
  if (f1 && f2 && f3 && !lv) cov_hit(CV_P_LEVELFAIL);
  }
}
static void scen2(void)
{
  static const int WIN[] = {1, 2, 3, 4, 8, 16, 40, 100, 160};
  int trial, setn;
  for (trial = 0; trial < 520; trial++) {
    ToneIirCfg cfg;
    ToneIir *t;
    int tk, nwin, wi = rrange(0, (int)(sizeof WIN / sizeof WIN[0]) - 1), sig, fi, amp, n, r, nsamp, seg;
    const short *sc;
    Osc o1, o2;
    u64 rh = 1469598103934665603ULL;
    mkcfg(&cfg, &tk);
    setn = rrange(0, g_nset - 1);
    if (tk == 0 && rpct(70)) { /* typical: a real set, a sane configuration */
      cfg.a_w = tone_word(g_set_a[setn]);
      cfg.b_w = tone_word(g_set_b[setn]);
      cfg.scales_w = tone_word(g_set_s[setn]);
      cfg.na = cfg.nb = 12;
      cfg.ratio = 10571;
      cfg.mode = 2;
      cfg.level = (short)rrange(20, 700);
      cfg.miss = rpct(15) ? rrange(-3, -1) : rrange(0, 4);
      cfg.hold = (int)rmod(2);
    }
    cfg.win = WIN[wi];
    /* integThr = GetFP_Value(8, win) * integ >> 14 = about 8 * integ / win windows: a few windows (or a wild value) */
    cfg.integ = rpct(75) ? rrange(0, 6) * cfg.win / 8 + (cfg.win == 1 ? 0 : 0) : rrange(-5, 2000);
    if (cfg.integ < 0 && rpct(70)) cfg.integ = 3;
    t = toneiir_create(NULL, &cfg);
    if (!t) die3("create");
    sc = tone_ptr(t->cfg.scales_w);
    if (sc[4] > 0) cov_hit(CV_P_POSOUT);
    else cov_hit(CV_P_NEGSCALE);
    for (n = 0; n < 5; n++)
      if (sc[n] < 0 || sc[n] > 31) cov_hit(CV_P_SHIFTMASK);
    emit(0, "TRIAL %d set %d tk %d", trial, setn, tk);
    dump_ti("S2", t);
    nwin = rrange(10, 70);
    nsamp = nwin * cfg.win;
    fi = rrange(0, NOSC - 1);
    if (rpct(60)) fi = rrange(3, 14); /* in band */
    amp = rpct(50) ? rrange(500, 9000) : rrange(30, 25000);
    sig = (int)rmod(7);
    if (tk == 1 && g_rc_hot) {
      sig = rpct(50) ? 7 : 8; /* full scale: a square wave / a constant */
      nsamp = rrange(400, 1500);
    }
    osc_init(&o1, fi, amp);
    osc_init(&o2, rrange(3, 14), amp / 2);
    seg = rrange(1, 12) * cfg.win;
    for (n = 0; n < nsamp; n++) {
      short x;
      int hits0 = t->hits, run0 = t->run;
      switch (sig) {
        case 0: x = osc_next(&o1); break;                                     /* a steady tone */
        case 1: x = (short)((int)osc_next(&o1) + noise(amp / 4)); break;       /* tone + noise */
        case 2: x = noise(amp); break;                                        /* noise */
        case 3: x = (n / seg) & 1 ? 0 : osc_next(&o1); break;                 /* bursts */
        case 4: x = (short)(((int)osc_next(&o1) + osc_next(&o2)) / 2); break; /* two tones */
        case 5: x = (n / seg) & 1 ? noise(20) : osc_next(&o1); break;         /* bursts over low noise */
        case 7: x = (n / 3) & 1 ? 32767 : -32768; break;                     /* full-scale square wave */
        case 8: x = (short)(rpct(10) ? -32768 : 32767); break;               /* full-scale, mostly constant */
        default: x = rpct(2) ? (short)rnd() : osc_next(&o1); break;           /* spikes */
      }
      if (rpct(1) && sig != 2) x = (short)(x + (short)rnd());
      if (t->cnt + 1 == t->cfg.win) { /* this sample closes a window: a shadow copy that does not close it gives the energies the decision sees */
        ToneIir sh = *t;
        sh.cfg.win = sh.cnt + 2;
        toneiir_progress(&sh, x);
        cond_cov(sh.e0, sh.e1, t->p, t->cfg.ratio, t->cfg.level);
      }
      r = toneiir_progress(t, x);
      window_cov(t, hits0, run0, r);
      rh = (rh ^ (u64)(unsigned)r) * 1099511628211ULL;
      if (r) emit(0, "W %d %d e0=%d e1=%d p=%d hits=%d run=%d thr=%d", n, r, t->e0, t->e1, t->p, t->hits, t->run, t->integThr);
    }
    emit(0, "S2END r=%016llx", rh);
    dump_ti("S2e", t);
    toneiir_delete(t);
    trial_end("tone progress");
  }
}

/* ---------------------------------------------------------------------------------------------- cadence helpers */
typedef struct {
  unsigned type;
  int filt, sub, buf, N, tol, level, integ, loose, extra;
  int maxOn, minOn, minOff, maxOff; /* the parameter values (scaled to buffers by cadence_create) */
} CadSpec;
static void prm_set(const CadSpec *s)
{
  memset(g_prm, 0, sizeof g_prm);
  g_prm[0x26] = s->filt;
  g_prm[0x2b] = s->filt;
  g_prm[0x33] = s->filt;
  g_prm[0x39] = s->filt;
  g_prm[0x27] = s->level;
  g_prm[0x3a] = s->buf;
  g_prm[0x3d] = s->sub;
  g_prm[0x22] = s->integ;
  g_prm[0x23] = s->tol;
  g_prm[0x3e] = s->loose;
  g_prm[0x17] = s->maxOn, g_prm[0x16] = s->minOn, g_prm[0x19] = s->minOff, g_prm[0x1a] = s->maxOff, g_prm[0x18] = s->N; /* busy */
  g_prm[0x2f] = s->maxOn, g_prm[0x2e] = s->minOn, g_prm[0x31] = s->minOff, g_prm[0x32] = s->maxOff, g_prm[0x30] = s->N; /* ringback */
  g_prm[0x35] = s->maxOn, g_prm[0x34] = s->minOn, g_prm[0x37] = s->minOff, g_prm[0x38] = s->maxOff, g_prm[0x36] = s->N; /* congestion */
}
static void dbg_hook(const char *m)
{
  if (strstr(m, "CONDITION B")) cov_hit(CV_R_COND_B);
  else if (strstr(m, "CONDITION C")) cov_hit(CV_R_COND_C);
  else if (strstr(m, "SERIES --- SATISFIED")) cov_hit(CV_R_COND_SER);
  else if (strstr(m, "BUSY cadence recognized")) cov_hit(CV_R_RECOG);
  else if (strstr(m, "CYCLES_COUNTER")) cov_hit(CV_R_CYCLE);
}
static void spec_random(CadSpec *s)
{
  static const unsigned TY[] = {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 9, 0xffffffffu};
  static const int BUF[] = {0, 1, 8, 40, 160, 666, 4000, -160, 32767, 32768, 33, 20};
  static const int SUB[] = {-2, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 100};
  static const int INT[] = {0, 1, 2, 3, 5, 10, 50, -1};
  s->type = TY[rmod(sizeof TY / sizeof TY[0])];
  s->filt = rpct(10) ? rrange(-2, 12) : rrange(0, 7);
  s->sub = SUB[rmod(sizeof SUB / sizeof SUB[0])];
  s->buf = BUF[rmod(sizeof BUF / sizeof BUF[0])];
  s->N = rpct(10) ? rrange(0, 130) : rrange(1, 12);
  s->tol = rrange(0, 9);
  s->level = rpct(60) ? rrange(30, 45) : rrange(0, 255);
  s->integ = INT[rmod(sizeof INT / sizeof INT[0])];
  s->loose = rpct(25) ? (int)rmod(3) : 0;
  s->extra = rpct(60) ? 0 : rrange(-2, 5);
  s->maxOn = rpct(15) ? 0 : rrange(1, 120);
  s->minOn = rpct(15) ? 0 : rrange(1, 120);
  s->minOff = rpct(15) ? 0 : rrange(1, 120);
  s->maxOff = rpct(15) ? 0 : rrange(1, 120);
}

/* ---------------------------------------------------------------------------------------------- scenario 3 */
static Cadence *cad_create_spec(Cadence *obj, const CadSpec *s, CadenceParams *p)
{
  p->type = s->type;
  prm_set(s);
  return cadence_create(obj, p, s->extra, FAKEM);
}
static void scen3(void)
{
  int trial;
  for (trial = 0; trial < 1600; trial++) {
    CadSpec s, s2;
    CadenceParams p;
    Cadence *c, *c1 = NULL;
    int mode = (int)rmod(8);
    char tb[48];
    spec_random(&s);
    if (rpct(40) && (s.buf == 0 || (short)s.buf == 0)) s.buf = 160;
    if ((short)s.buf == 0 && s.buf != 0) s.buf = 160; /* GetFP_Value(1, 0) never returns */
    dsplibs_debug_level = lvl_value(rpct(50) ? 0 : (int)rmod(7));
    memset(&p, 0, sizeof p);
    p.callerParamA = (int)rnd();
    p.callerParamB = (int)rnd();
    p.type = s.type;
    emit(0, "TRIAL %d mode %d type %u filt %d sub %d buf %d extra %d", trial, mode, s.type, s.filt, s.sub, s.buf, s.extra);
    if (s.type == 0) cov_hit(CV_K_TYPE0);
    else if (s.type == 1) cov_hit(CV_K_TYPE1);
    else if (s.type == 2) cov_hit(CV_K_TYPE2);
    else if (s.type == 3) cov_hit(CV_K_TYPE3);
    else if (s.type == 4) cov_hit(CV_K_TYPE4);
    else cov_hit(CV_K_TYPE9);
    if (s.filt >= 0 && s.filt <= 7) cov_hit(CV_K_FILT0 + s.filt);
    else cov_hit(CV_K_FILT8);
    if (dsplibs_debug_level > 1) cov_hit(CV_K_DBG);
    if (mode == 6) { /* the cadence allocation fails */
      g_fail_malloc = 0;
      c = cad_create_spec(NULL, &s, &p);
      emit(0, "FAIL0 %s", c ? "obj" : "NULL");
      if (c) die3("cadence_create returned an object with a failing allocator");
      cov_hit(CV_K_MALLOC_FAIL1);
      trial_end("cad fail0");
      continue;
    }
    if (mode == 7) { /* the ToneIir allocation fails */
      g_fail_malloc = 1;
      c = cad_create_spec(NULL, &s, &p);
      dump_cad("FAIL1", c);
      emit(0, "TYPE-OUT %u", p.type);
      if (c) {
        if (c->ti) die3("ToneIir exists although its allocation failed");
        cov_hit(CV_K_TIFAIL);
        cadence_delete(c);
      }
      cov_hit(CV_K_MALLOC_FAIL2);
      trial_end("cad fail1");
      continue;
    }
    if (mode >= 4) { /* reuse: an object that already has its ToneIir (the first create is not the one under comparison for its own sake: it is dumped too) */
      CadenceParams p1;
      spec_random(&s2);
      if ((short)s2.buf == 0 && s2.buf != 0) s2.buf = 160;
      memset(&p1, 0, sizeof p1);
      p1.type = s2.type;
      c1 = cad_create_spec(NULL, &s2, &p1);
      dump_cad("FIRST", c1);
      if (!c1) {
        trial_end("cad first");
        continue;
      }
      cov_hit(CV_K_REUSE_TI);
      /* the fields cadence_create must (re)initialise are dirty: a previous life of the object */
      c1->allpass = (int)rnd();
      c1->idx = rrange(0, 70);
      c1->cnt = (int)rmod(100);
      c1->seriesFlag = (int)rmod(3);
      c1->scaledLimitA = rrange(-50, 500);
      c1->scaledLimitB = rrange(-50, 500);
      c1->callerParamA = (int)rnd();
      c1->reportTone = (int)rmod(3);
      c1->name = NULL;
    }
    c = cad_create_spec(c1, &s, &p);
    emit(0, "TYPE-OUT %u", p.type);
    if (p.type != (s.type > 4 ? 4u : s.type)) die3("type write-back");
    if (s.type > 4) cov_hit(CV_K_TYPECLAMP);
    dump_cad("CAD", c);
    if (c1 && c && c != c1) die3("cadence_create did not reuse the object");
    if (c) {
      if (strncmp(dptr(c->tabA, tb, sizeof tb), "Filter", 6) == 0) cov_hit(CV_K_FILTER_TAB);
      else cov_hit(CV_K_CP_TAB);
      if (s.sub >= 1 && s.sub <= 7) cov_hit(CV_K_ROW);
      else cov_hit(CV_K_NOROW);
      if (s.type == 0 && s.loose) cov_hit(CV_K_LOOSE);
      if (s.extra > 0) cov_hit(CV_K_EXTRA);
      if (c->integ > 0 && s.type != 0 && s.type != 2 && s.type != 3 && s.integ * 100 > 100) cov_hit(CV_K_INTEGHI);
      cadence_delete(c);
    }
    if (s.type == 0 && (s.maxOn == 0 || s.minOn == 0 || s.minOff == 0 || s.maxOff == 0)) cov_hit(CV_K_ZEROLIM0);
    if ((s.type == 2 || s.type == 3) && (s.maxOn == 0 || s.minOn == 0 || s.minOff == 0 || s.maxOff == 0)) cov_hit(s.type == 2 ? CV_K_DISABLED2 : CV_K_DISABLED3);
    if (s.type != 0 && s.type != 2 && s.type != 3 && s.buf == 0) cov_hit(CV_K_BUFINVALID);
    trial_end("cad create");
  }
}

/* ---------------------------------------------------------------------------------------------- scenarios 4 and 5 */
static unsigned long g_calls;
static int model_absd(int a, int b) { int d = a - b; return d < 0 ? -d : d; }
/* one sample through cadence_progress with the coverage classes of the state machine; a shadow of the ToneIir gives the result r the cadence sees */
static int cad_step(Cadence *c, short x, u64 *rh)
{
  ToneIir sh = *c->ti;
  int r = toneiir_progress(&sh, x), ret;
  int st = c->state, cnt = c->cnt, idx = c->idx, series = c->seriesFlag, flag = c->reportTone, loose = c->loose, N = c->maxCycles, resetOff = c->resetOff, maxOn = c->maxOn;
  if (r == 1 && st == 0 && idx + 1 >= N && !series && !loose && N - 1 <= idx && idx > 5 && idx < 60) { /* an independent model of condition C without B (coverage class only) */
    int n1 = idx, tol = (int)g_prm[0x23], onn = cnt, offn = c->off[n1], offp = c->off[n1 - 1];
    int (*ab)(int, int) = model_absd;
    if (tol <= 2) tol = 3;
    if (!(offn >= c->minOff && offn <= c->maxOff && onn >= c->minOn && onn <= c->maxOn)) {}
    else {
      int fB = ab(offn, offp) < tol && ab(offn, c->off[n1 - 2]) < tol && ab(onn, c->on[n1 - 1]) < tol && ab(onn, c->on[n1 - 2]) < tol;
      int fC = ab(offn, c->off[n1 - 2]) < tol && ab(offn, c->off[n1 - 4]) < tol && ab(onn, c->on[n1 - 2]) < tol && ab(onn, c->on[n1 - 4]) < tol && ab(offp, c->off[n1 - 3]) < tol &&
               ab(offp, c->off[n1 - 5]) < tol && ab(c->on[n1 - 1], c->on[n1 - 3]) < tol && ab(c->on[n1 - 1], c->on[n1 - 5]) < tol;
      if (!fB && fC) cov_hit(CV_R_COND_C_ONLY);
    }
  }
  ret = cadence_progress(c, x);
  g_calls++;
  *rh = (*rh ^ (u64)(unsigned)ret) * 1099511628211ULL;
  /* every transient of the state machine counts: the small state after each call goes into the running hash (and the slots it may have written) */
  *rh = (*rh ^ (u64)(unsigned)c->state) * 1099511628211ULL;
  *rh = (*rh ^ (u64)(unsigned)c->cnt) * 1099511628211ULL;
  *rh = (*rh ^ (u64)(unsigned)c->idx) * 1099511628211ULL;
  *rh = (*rh ^ (u64)(unsigned)c->cycles) * 1099511628211ULL;
  if (idx >= 0 && idx < 60) {
    *rh = (*rh ^ (u64)(unsigned)c->on[idx]) * 1099511628211ULL;
    *rh = (*rh ^ (u64)(unsigned)c->off[idx]) * 1099511628211ULL;
  }
  *rh = (*rh ^ (u64)(unsigned)c->ti->hits) * 1099511628211ULL;
  *rh = (*rh ^ (u64)(unsigned)c->ti->run) * 1099511628211ULL;
  if (r == 2) {
    if (flag) cov_hit(CV_R_R2_FLAG);
    if (st == 0 && !series && maxOn == 0) cov_hit(CV_R_R2_ST0_RESET);
    else if (st == 0) cov_hit(CV_R_R2_ST0_CNT);
    else if (st == 1) cov_hit(CV_R_R2_ST1);
  } else if (r == 1) {
    if (st == 1) {
      cov_hit(CV_R_R1_ST1);
      if (cnt + 1 > resetOff) cov_hit(CV_R_R1_RESETOFF);
    } else if (st == 0) {
      cov_hit(CV_R_R1_ST0);
      if (idx + 1 >= N) {
        cov_hit(series ? CV_R_JUDGE_SERIES : loose ? CV_R_JUDGE_LOOSE : CV_R_JUDGE_NORMAL);
        if (g_prm[0x23] <= 2) cov_hit(CV_R_TOLLOW);
        if (ret != 1) cov_hit(CV_R_NOCOND);
      }
      if (idx + 1 > 0x3b) cov_hit(CV_R_IDXWRAP);
    }
  }
  if (ret == 1) cov_hit(CV_R_RET1);
  if (ret == 7) cov_hit(CV_R_RET7), cov_hit(CV_R_NOANSWER);
  if (ret) emit(0, "R %lu %d", g_calls, ret);
  if (ret == 7 && rpct(50)) { /* the caller restarts the detector (cycles is only cleared by cadence_reset); else the next samples return 7 again */
    cadence_reset(c);
    emit(0, "RESET cyc=%d st=%d idx=%d cnt=%d", c->cycles, c->state, c->idx, c->cnt);
  }
  return ret;
}
static void cad_snapshot(const char *tag, const Cadence *c, u64 rh)
{
  emit(0, "%s st=%d cnt=%d idx=%d cyc=%d on=%016llx off=%016llx e1=%d p=%d hits=%d run=%d rh=%016llx", tag, c->state, c->cnt, c->idx, c->cycles, fnv_ints(c->on, 60), fnv_ints(c->off, 60), c->ti->e1,
       c->ti->p, c->ti->hits, c->ti->run, rh);
}

static void scen4(void)
{
  int trial;
  for (trial = 0; trial < 150; trial++) {
    CadSpec s;
    CadenceParams p;
    Cadence *c;
    u64 rh = 1469598103934665603ULL;
    int fi, amp, P, on[3], off[3], jit, i, seg, nseg, k, sil, kind, bufl, dwell;
    Osc o;
    memset(&p, 0, sizeof p);
    spec_random(&s);
    kind = (int)rmod(10); /* 0..4 busy-like repeat, 5 ringback, 6 congestion, 7 dial, 8 no answer, 9 series */
    s.type = kind == 5 ? 3 : kind == 6 ? 2 : kind == 7 ? (rpct(50) ? 1 : 4) : 0;
    s.buf = 160;
    if (s.type == 1 || s.type == 4) s.buf = rpct(50) ? 160 : rpct(50) ? 40 : 20;
    s.filt = rrange(0, 7);
    s.sub = rpct(70) ? 0 : rrange(1, 7);
    if (s.type == 1 || s.type == 4) s.sub = rpct(30) ? rrange(1, 7) : 0;
    if (s.type == 1 || s.type == 4) s.integ = rrange(1, 2);
    s.level = rrange(30, 45);
    s.N = rrange(2, 8);
    s.tol = rpct(10) ? rrange(-3, -1) : rrange(0, 8);
    s.extra = rpct(85) ? 0 : 1;
    s.loose = (s.type == 0 && rpct(30)) ? 1 : 0;
    s.integ = rrange(1, 4);
    P = rpct(60) ? 1 : rrange(2, 3);
    for (i = 0; i < 3; i++) {
      on[i] = rrange(5, 28);
      off[i] = rrange(5, 28);
    }
    if (kind == 8) {
      for (i = 0; i < 3; i++) off[i] = 200;
    }
    jit = rpct(50) ? 0 : rpct(50) ? 1 : rpct(50) ? s.tol : s.tol + 2;
    if (jit < 0) jit = 1; /* (a negative tolerance parameter is a test of the compare, not a jitter) */
    /* parameter limits (in the units of the parameters: scaled by about 0.5 per buffer by cadence_create) around the first pair: right, or off by a bit */
    s.maxOn = 2 * (on[0] + rrange(1, 4)) + (rpct(12) ? 20 : 0);
    s.minOn = 2 * (on[0] - rrange(1, 4)) + (rpct(12) ? -30 : 0);
    s.maxOff = 2 * (off[0] + rrange(1, 4));
    s.minOff = 2 * (off[0] - rrange(1, 4)) + (rpct(12) ? 60 : 0);
    if (s.minOn < 1) s.minOn = 1;
    if (s.minOff < 1) s.minOff = 1;
    if (kind == 8) s.maxOff = 150;
    if (rpct(8)) s.maxOn = 0; /* zero limits: defaults (busy) or a disabled detector */
    dsplibs_debug_level = lvl_value(rpct(25) ? 0 : (int)rmod(7));
    c = cad_create_spec(NULL, &s, &p);
    emit(0, "TRIAL %d kind %d type %u P %d jit %d tol %d N %d", trial, kind, s.type, P, jit, s.tol, s.N);
    if (getenv("R09_TRACE")) fprintf(stderr, "T4 %d kind %d type %u dbg %u\n", trial, kind, s.type, dsplibs_debug_level);
    if (!c) {
      emit(0, "DISABLED");
      cov_hit(CV_R_CYCLE); /* (placeholder: the create was not usable; keeps the trial counted) */
      trial_end("cad disabled");
      continue;
    }
    dump_cad("C4", c);
    if (kind == 9) { /* series: the first two cycles' values as the reference (+- a little) */
      c->seriesFlag = 1;
      c->ser[0] = on[0] + (rpct(70) ? 0 : rrange(-2, 2));
      c->ser[1] = off[0] + (rpct(70) ? 0 : rrange(-2, 2));
      c->ser[2] = on[1 % P] + (rpct(70) ? 0 : rrange(-2, 2));
      c->ser[3] = off[1 % P] + (rpct(70) ? 0 : rrange(-2, 2));
      c->seriesMin = rrange(0, 4);
    }
    fi = rpct(80) ? rrange(5, 9) : rrange(0, NOSC - 1);
    amp = rpct(90) ? rrange(2500, 14000) : rrange(40, 400);
    bufl = c->bufLen;
    if (bufl <= 0) bufl = 1;
    dwell = 0;
    nseg = (kind == 8) ? 14 : rrange(14, 26);
    sil = (int)rmod(3);
    for (seg = 0; seg < nseg; seg++) {
      int pr = seg % P, don = on[pr] + (jit ? rrange(-jit, jit) : 0), doff = off[pr] + (jit ? rrange(-jit, jit) : 0);
      if (kind == 7) { /* dial tone: a long tone, then silence */
        don = rrange(30, 80);
        doff = rrange(20, 90);
      }
      if (don < 1) don = 1;
      if (doff < 1) doff = 1;
      osc_init(&o, fi, amp);
      for (k = 0; k < don * bufl; k++) cad_step(c, osc_next(&o), &rh);
      for (k = 0; k < doff * bufl; k++) {
        short x = sil == 0 ? 0 : sil == 1 ? noise(12) : (short)(k & 1 ? 40 : -40);
        cad_step(c, x, &rh);
      }
      dwell++;
      cad_snapshot("SEG", c, rh);
    }
    emit(0, "T4END %d", dwell);
    dump_cad("C4e", c);
    cadence_delete(c);
    trial_end("cad run");
  }
}

static void scen5(void)
{
  int trial;
  for (trial = 0; trial < 500; trial++) {
    CadSpec s;
    CadenceParams p;
    Cadence *c;
    u64 rh = 1469598103934665603ULL;
    int i, n, nn, b1, b2, jit, mode;
    Osc o;
    memset(&p, 0, sizeof p);
    spec_random(&s);
    s.type = (unsigned)rmod(4);
    s.sub = 0;
    s.buf = 160;
    s.N = rrange(1, 9);
    s.tol = rrange(0, 6);
    s.integ = 1;
    s.maxOn = rrange(1, 60);
    s.minOn = rrange(1, 30);
    s.maxOff = rrange(1, 60);
    s.minOff = rrange(1, 30);
    s.extra = 0;
    s.level = rrange(30, 45);
    dsplibs_debug_level = lvl_value(rpct(25) ? 0 : (int)rmod(7));
    c = cad_create_spec(NULL, &s, &p);
    if (!c) {
      trial_end("fuzz disabled");
      continue;
    }
    /* tiny windows so that every sample is (nearly) a window end: the cadence sees a result per sample */
    c->ti->cfg.win = rrange(1, 3);
    c->ti->integThr = rrange(-1, 2);
    c->ti->cfg.level = (short)rrange(1, 300);
    c->ti->cfg.hold = (int)rmod(2);
    c->ti->cfg.miss = rrange(0, 3);
    toneiir_reset(c->ti);
    mode = (int)rmod(4);
    c->state = (int)rmod(rpct(5) ? 3 : 2);
    c->cnt = rrange(0, 30);
    c->idx = rpct(10) ? rrange(0, 0x3b) : rrange(0, 12);
    c->cycles = rpct(80) ? rrange(0, 8) : rrange(-2, 12);
    c->loose = mode == 1;
    c->seriesFlag = mode == 2;
    c->seriesMin = rpct(15) ? rrange(-3, -1) : rrange(0, 6);
    c->reportTone = (int)rmod(2);
    c->maxCycles = mode == 1 ? rrange(1, 9) : rpct(10) ? rrange(-2, 0) : rrange(1, 9);
    c->resetOff = rpct(8) ? rrange(-2, 0) : rrange(1, 40);
    c->maxOn = rpct(10) ? 0 : rpct(8) ? rrange(-5, -1) : rrange(1, 60);
    c->minOn = rpct(8) ? rrange(-5, -1) : rrange(1, 30);
    c->maxOff = rpct(8) ? rrange(-5, -1) : rrange(1, 60);
    c->minOff = rpct(8) ? rrange(-5, -1) : rrange(1, 30);
    b1 = rrange(2, 40);
    b2 = rrange(2, 40);
    jit = rpct(50) ? 0 : rrange(0, 4);
    for (i = 0; i < 60; i++) {
      c->on[i] = (i & 1 ? b2 : b1) + (jit ? rrange(-jit, jit) : 0);
      c->off[i] = (i & 1 ? b1 : b2) + (jit ? rrange(-jit, jit) : 0);
      if (rpct(10)) c->on[i] = rrange(0, 60);
      if (rpct(10)) c->off[i] = rrange(0, 60);
      if (rpct(3)) c->on[i] = rrange(-30, -1);
      if (rpct(3)) c->off[i] = rrange(-30, -1);
    }
    if (rpct(60)) { /* limits that bracket the pattern: the conditions can hold */
      int lo = b1 < b2 ? b1 : b2, hi = b1 < b2 ? b2 : b1;
      c->maxOn = hi + 3;
      c->maxOff = hi + 3;
      c->minOn = lo > 3 ? lo - 3 : 0;
      c->minOff = lo > 3 ? lo - 3 : 0;
    }
    c->ser[0] = c->on[0] + (rpct(70) ? 0 : rrange(-3, 3));
    c->ser[1] = c->off[1] + (rpct(70) ? 0 : rrange(-3, 3));
    c->ser[2] = c->on[1] + (rpct(70) ? 0 : rrange(-3, 3));
    c->ser[3] = c->off[2] + (rpct(70) ? 0 : rrange(-3, 3));
    if (mode == 2) c->seriesMin = rrange(0, 3);
    emit(0, "FUZZ %d mode %d", trial, mode);
    dump_cad("F5", c);
    nn = rrange(300, 3000);
    osc_init(&o, rrange(5, 9), rrange(2000, 12000));
    for (n = 0; n < nn;) {
      int len = rrange(1, 40), kk = (int)rmod(4), q;
      for (q = 0; q < len && n < nn; q++, n++) {
        short x = kk == 0 ? osc_next(&o) : kk == 1 ? 0 : kk == 2 ? noise(30) : (short)rnd();
        cad_step(c, x, &rh);
        if (rmod(500) == 0) { /* cadence_reset at an arbitrary moment */
          cadence_reset(c);
          emit(0, "RESET* st=%d cnt=%d idx=%d cyc=%d hits=%d run=%d e1=%d p=%d", c->state, c->cnt, c->idx, c->cycles, c->ti->hits, c->ti->run, c->ti->e1, c->ti->p);
        }
      }
      if ((n & 255) < 40) cad_snapshot("F5s", c, rh);
    }
    dump_cad("F5e", c);
    cadence_delete(c);
    trial_end("cad fuzz");
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 6: exact judging */
/* the ToneIir is bent into a result generator: win = 1, integThr = -1, input 0.  With level -32768 every window passes the conditions (r = 2), with 32767 none does (r = 1), so the harness
 * dictates the results of toneiir_progress sample by sample and builds exact on / off durations, tolerance boundaries, limits and series values. */
static int cad_r(Cadence *c, int r, u64 *rh)
{
  c->ti->cfg.level = (short)(r == 2 ? -32768 : 32767);
  return cad_step(c, 0, rh);
}
static int edge_delta(int t)
{
  static const int E[] = {0, 0, 0, -1, 1, 0};
  switch (rmod(7)) {
    case 0: return t - 1;
    case 1: return t;
    case 2: return t + 1;
    case 3: return -(t - 1);
    case 4: return -t;
    case 5: return -(t + 1);
    default: return E[rmod(6)];
  }
}
static void scen6(void)
{
  int trial;
  for (trial = 0; trial < 3000; trial++) {
    CadSpec s;
    CadenceParams p;
    Cadence *c;
    u64 rh = 1469598103934665603ULL;
    int mode = (int)rmod(10), N, n1, t, i, k, a1, a2, b1, b2, don, doff, ret;
    memset(&p, 0, sizeof p);
    memset(&s, 0, sizeof s);
    s.type = rpct(75) ? 0 : rpct(50) ? 2 : 3;
    s.level = 40;
    s.buf = 160;
    s.integ = 1;
    s.tol = rpct(12) ? rrange(-2, 2) : rrange(3, 6);
    s.maxOn = s.minOn = s.maxOff = s.minOff = 20;
    s.N = 5;
    s.loose = (s.type == 0 && mode >= 4 && mode < 7) ? 1 : 0; /* modes 0-3 normal, 4-6 loose, 7-9 series */
    dsplibs_debug_level = lvl_value(rpct(25) ? 0 : (int)rmod(7));
    c = cad_create_spec(NULL, &s, &p);
    if (!c) die3("scenario 6 create failed");
    c->ti->cfg.win = 1;
    c->ti->integThr = -1;
    c->ti->cnt = 0;
    c->ti->p = 0; /* (the freshly allocated ToneIir holds garbage in p: the first window would not pass) */
    t = s.tol <= 2 ? 3 : s.tol;
    if (mode >= 4 && mode < 7 && !c->loose) c->loose = 1;
    if (mode < 4 || mode >= 7) c->loose = 0;
    c->seriesFlag = mode >= 7;
    /* the cycle count N and the slot being judged */
    N = mode >= 4 && mode < 7 ? rrange(1, 6) : rpct(65) ? rrange(7, 10) : rpct(15) ? rrange(-2, 0) : rrange(1, 9);
    n1 = N >= 1 ? N - 1 : rrange(0, 8);
    c->maxCycles = N;
    c->idx = n1;
    c->state = 1;
    c->cnt = 0;
    c->cycles = rpct(8) ? rrange(8, 10) : rrange(0, 3);
    /* arrays: a period-2 (or period-1) pattern with one perturbed element */
    a1 = rrange(4, 30);
    b1 = rrange(4, 30);
    if (rpct(50) || mode >= 4) {
      a2 = a1;
      b2 = b1; /* period 1 */
    } else {
      a2 = a1 + (rpct(85) ? t + rrange(0, 4) : t - 1 + rrange(-2, 0)); /* period 2: (a1, a2) differ by at least the tolerance (condition B fails, C can hold) */
      b2 = b1 + (rpct(85) ? t + rrange(0, 4) : -(t + rrange(0, 4)));
      if (b2 < 2) b2 = b1 + t + 1;
    }
    for (i = 0; i < 60; i++) {
      c->on[i] = 0;
      c->off[i] = 0;
    }
    for (i = 0; i < n1 && i < 60; i++) { /* the slots in front of n1; parity relative to n1: odd distance = a1 / b1 */
      int odd = (n1 - i) & 1;
      c->on[i] = odd ? a1 : a2;
      c->off[i] = odd ? b1 : b2;
    }
    if (n1 >= 1 && rpct(70)) { /* one perturbed element within the window of the comparisons */
      int e = n1 - rrange(1, 5), d = edge_delta(t);
      if (e >= 0) {
        if (rpct(50)) c->on[e] += d;
        else c->off[e] += d;
      }
    }
    don = a2 + (rpct(75) ? 0 : edge_delta(t));
    doff = b2 + (rpct(75) ? 0 : edge_delta(t));
    if (mode >= 4 && mode < 7) { /* loose: the first slot (n1 - N + 1 = 0) is the reference of the last comparison */
      c->off[0] = rpct(60) ? doff + rrange(0, 2) : doff + edge_delta(t);
      c->on[0] = rpct(60) ? don : don + edge_delta(t);
    }
    if (don < 1) don = 1;
    if (doff < 1) doff = 1;
    /* limits around the durations of the slot being judged (boundaries included) */
    c->minOn = don - (rpct(80) ? rrange(0, 2) : rrange(-2, 0) - 1);
    c->maxOn = don + (rpct(80) ? rrange(0, 2) : rrange(-2, 0) - 1);
    c->minOff = doff - (rpct(80) ? rrange(0, 2) : rrange(-2, 0) - 1);
    c->maxOff = doff + (rpct(80) ? rrange(0, 2) : rrange(-2, 0) - 1);
    if (rpct(8)) { /* a negative limit (a signed compare against a non-negative duration) */
      switch (rmod(4)) {
        case 0: c->minOn = rrange(-5, -1); break;
        case 1: c->maxOn = rrange(-5, -1); break;
        case 2: c->minOff = rrange(-5, -1); break;
        default: c->maxOff = rrange(-5, -1); break;
      }
    }
    if (mode >= 7) { /* series: the reference values of on[0], off[1], on[1], off[2] */
      for (i = 0; i < 4; i++) {
        int v = i == 0 ? c->on[0] : i == 1 ? c->off[1] : i == 2 ? c->on[1] : c->off[2];
        if (n1 < 3) v = rrange(4, 30);
        c->ser[i] = v + (rpct(60) ? 0 : edge_delta(t));
      }
      c->seriesMin = n1 + rrange(-1, 1);
      if (rpct(10)) c->seriesMin = rrange(-3, -1);
    }
    emit(0, "TRIAL %d mode %d N %d n1 %d tol %d on %d off %d", trial, mode, N, n1, s.tol, don, doff);
    dump_cad("C6", c);
    if (mode < 4 && rpct(6)) { /* restart: in state 0 with a non-zero count and no maxOn, the first tone result restarts the cycle (and clears the count) */
      c->maxOn = 0;
      c->seriesFlag = 0;
      c->state = 0;
      c->cnt = rrange(1, 20);
      emit(0, "RESTART-MODE cnt=%d", c->cnt);
      ret = cad_r(c, 2, &rh);
      cad_r(c, 1, &rh);
      cad_r(c, 2, &rh);
      cad_r(c, 2, &rh);
      cad_r(c, 1, &rh);
      cov_hit(CV_R_RET1);
      emit(0, "RESTART ret %d", ret);
    } else {
    /* the slot: silence of doff samples, then a tone of don + 1 samples (the first one switches state), then one silent sample: the judge */
    for (k = 0; k < doff; k++) cad_r(c, 1, &rh);
    for (k = 0; k < don + 1; k++) cad_r(c, 2, &rh);
    ret = cad_r(c, 1, &rh);
    }
    emit(0, "JUDGE ret %d st=%d idx=%d cyc=%d", ret, c->state, c->idx, c->cycles);
    cad_snapshot("C6s", c, rh);
    /* a tail of random cycles (idx wrap, further judging, the cycles / no-answer path) */
    for (i = 0; i < 4; i++) {
      int d1 = rrange(1, 25), d2 = rrange(1, 25);
      if (rpct(10)) cadence_reset(c);
      for (k = 0; k < d2; k++) cad_r(c, 1, &rh);
      for (k = 0; k < d1; k++) cad_r(c, 2, &rh);
      cad_r(c, 1, &rh);
    }
    dump_cad("C6e", c);
    cadence_delete(c);
    trial_end("cad exact");
  }
}

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : -1;
  const char *e;
  if (scen < 0 || scen > 6) {
    fprintf(stderr, "usage: %s scenario(0..6)\n", argv[0]);
    return 2;
  }
  g_verbose = (e = getenv("R09_VERBOSE")) != NULL && *e == '1';
  if ((e = getenv("R09_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(e, NULL, 0);
  g_lcg += (unsigned)scen * 0x9e3779b9u;
  tabs_init();
  cov_setup(scen);
  switch (scen) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    case 4: g_dbg_hook = dbg_hook; scen4(); break;
    case 5: g_dbg_hook = dbg_hook; scen5(); break;
    case 6: g_dbg_hook = dbg_hook; scen6(); break;
    default: break;
  }
  cov_finish();
  return 0;
}
