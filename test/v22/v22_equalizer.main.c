/*
 * R04 differential harness (1 of 2): the V.22 / V.22bis FSE - V22_FSE_init / _free / _getdiag / _receive and the decision callbacks FSEv22_decision12 / _decision24 -
 * real dsplibs.o vs src/v22/v22_equalizer.c (+ src/v22/v22_tables.c).  ONE source, no #ifdef around the code under test; the same typed object (src/v22/v22.h: the i386 layout at
 * -m32, wider pointers on x86-64) is used on both sides and everything printed is architecture independent, so the 32-bit BLOB run is the golden of the x86-64 build
 * (test/golden/v22/v22_equalizer.blob32.out.gz, test/lib/tier_golden64.sh).  Library: test/r04_common.h (canonical hashed output, deterministic heap with red zones and the
 * sysdep_malloc / sysdep_free / sysdep_memcpy recorders, coverage counters: exit 2 = a counter is zero, 3 = harness violation).  FPM_atan / FPM_phasor are the real
 * callees of each side.
 *
 * Scenarios (each its own process):
 *   0 lifecycle: init with allocate = 1 / 0 over objects pre-filled with garbage (every written field, allocation order and sizes, the taps = cfg >> 2 reversed, the cleared
 *     history), the config extremes, re-init without allocation after use, V22_FSE_free (order), getdiag (always 0), double init (leak) ;
 *   1 receive with a scripted fake decision callback over randomly poked state (rem 0..6, phase, wpos incl. the history-shift boundary, pllCnt around 0x30, gainSel, mu, pll /
 *     adapt enables, errAvg / errPow / carrier extremes), every callback answer range (angle full 16-bit incl. > +-0x4000, mag incl. negative / extreme, bits), call sizes
 *     (0, 1 .. 6, 7, 13, 40, 78, negative, short-partial) -> every output, the whole state, the callback's view, the memcpy log;
 *   2 FSEv22_decision12 called directly: a grid over the whole (I, Q) plane (step 512 and its +-1 neighbours), random, the nOut index 0 .. 13 and every SMC state word ;
 *   3 FSEv22_decision24 called directly: the same, step 256, the +-0x2000 / 0x1000 decision boundaries ;
 *   5 mass: 1.5 million single-symbol calls over fully randomised state (hits the exact boundary values of every 16-bit range test) ;
 *   4 end to end: the real decisions behind V22_FSE_receive on synthetic PSK / QAM tones (matched, offset, noisy, saturated, random) at 1200 and 2400 bit/s over long streams
 *     and many call sizes (so the taps, PLL, error averages evolve), plus loops of one-sample calls.
 */
#include "../r04_common.h"
#include "v22/v22.h"
#include "v22/v22_tables.h"

enum {
  CV_INIT_ALLOC = 0, CV_INIT_NOALLOC, CV_FREE, CV_GETDIAG, CV_DOUBLE_INIT,
  CV_RX_PART_POS = 10, CV_RX_PART_NEG, CV_RX_PART_ZERO_REM, CV_RX_FULL, CV_RX_SHIFT_FULL, CV_RX_SHIFT_PART, CV_RX_BASE_LOW, CV_RX_BASE_HIGH, CV_RX_MANY_SYMS, CV_RX_ZERO_COUNT,
  CV_RX_PLL_OFF, CV_RX_ADAPT_OFF, CV_RX_ADAPT_ON, CV_RX_PLLCNT_LO, CV_RX_PLLCNT_HI, CV_RX_GS0, CV_RX_GS1, CV_RX_GS2, CV_RX_CB_ANG_HI, CV_RX_CB_ANG_LO, CV_RX_CB_MAG_NEG,
  CV_RX_ERRPOW_POS, CV_RX_ERRPOW_ZERO, CV_RX_MU1, CV_RX_CARR_NEG, CV_RX_CARR_POS, CV_RX_SMC_NONZERO,
  CV_D12_CALL = 50, CV_D12_NOMATCH, CV_D12_ALL4, CV_D12_PREV_NONZERO, CV_D12_NOUT_HI, CV_D12_TIE,
  CV_D24_CALL = 60, CV_D24_NOMATCH, CV_D24_ALL16, CV_D24_NEG_I, CV_D24_NEG_Q, CV_D24_HI_I, CV_D24_BOUND, CV_D24_TIE, CV_D24_NOUT_HI, CV_D24_PREV_NONZERO,
  CV_MASS_CALLS = 100,
  CV_E2E_12 = 80, CV_E2E_24, CV_E2E_ONE_SAMPLE, CV_E2E_ADAPT, CV_E2E_PLL_SWITCH, CV_E2E_NOISE, CV_E2E_SAT, CV_E2E_BITS_VAR
};

/* ------------------------------------------------------------------------------------------------ the object under test */
static V22Fse g_f;
static unsigned short g_smc;
static short g_srcI[49], g_srcQ[49];
static const V22FseCfg g_cfgp = {g_srcI, g_srcQ};
static short g_in[4096 + 64];
static short g_bits[512];

static int fn_id(V22FseDecision d);

static void dump(int show, const char *tag, const V22Fse *f, int deep)
{
  char a[40], b[40], c[40], d[40], e[40], g[40], h[40], i[40], j[40];
  unsigned char pad[16];
  u64 hp = 1469598103934665603ULL;
  int k;
  memcpy(pad, f->pad4c, 10);
  for (k = 0; k < 10; k++) {
    hp ^= pad[k];
    hp *= 1099511628211ULL;
  }
  ptr_desc(a, sizeof a, f->outI);
  ptr_desc(b, sizeof b, f->outQ);
  ptr_desc(c, sizeof c, f->tapsI);
  ptr_desc(d, sizeof d, f->tapsQ);
  ptr_desc(e, sizeof e, f->hist);
  ptr_desc(g, sizeof g, f->buf44);
  ptr_desc(h, sizeof h, f->buf48);
  snprintf(i, sizeof i, "%s", f->srcI == g_srcI ? "cfgI" : f->srcI ? "other" : "NULL");
  snprintf(j, sizeof j, "%s", f->srcQ == g_srcQ ? "cfgQ" : f->srcQ ? "other" : "NULL");
  emit(show, "%s src=%s,%s mu=%d gs=%d integ=%d p0e=%d s10=%d pllE=%d s18=%d adE=%d eA=%d eP=%d out=%s,%s cnt=%d nOut=%d taps=%s,%s hist=%s wpos=%d car=%d ph=%d s42=%d b44=%s b48=%s pad=%016llx pllCnt=%d rem=%d p5a=%d smc=%s dec=%d",
       tag, i, j, f->mu, f->gainSel, f->integ, f->pad0e, f->s10, f->pllEnable, f->s18, f->adaptEnable, f->errAvg, f->errPow, a, b, f->count, f->nOut, c, d, e, f->wpos, f->carrier, f->phase,
       f->s42, g, h, hp, f->pllCnt, f->rem, f->pad5a, f->smc == &g_smc ? "smc" : f->smc ? "other" : "NULL", fn_id(f->decide));
  if (deep) {
    emit(show && 0, "%s buf hashes", tag);
    if (f->tapsI && f->tapsQ && f->hist && f->outI && f->outQ) {
      emit(show, "%s H taps=%016llx,%016llx hist=%016llx out=%016llx,%016llx", tag, fnv_shorts(f->tapsI, 49), fnv_shorts(f->tapsQ, 49), fnv_shorts(f->hist, 98), fnv_shorts(f->outI, 14),
           fnv_shorts(f->outQ, 14));
    }
  }
}

/* ------------------------------------------------------------------------------------------------ decision callbacks */
static int g_cb_mode;      /* 0 random / full range, 1 near-correct, 2 extremes */
static unsigned long g_cb_calls;
static int g_cb_log = 1;
static unsigned fake_cb(V22Fse *f, short *angle, short *mag)
{
  short a, m;
  unsigned bits = rnd() & 0xffff;
  g_cb_calls++;
  if (g_cb_log) emit(0, "CB n=%d I=%d Q=%d smc=%u", f->nOut, f->outI[f->nOut], f->outQ[f->nOut], g_smc);
  switch (g_cb_mode) {
    case 1: a = (short)rrange(-0x4000, 0x4000); m = (short)rrange(0x1000, 0x7000); break;
    case 2: a = (short)(rpct(50) ? (rpct(50) ? 0x7fff : -32768) : rshort()); m = (short)(rpct(30) ? -32768 : rpct(30) ? 32767 : rshort()); break;
    default: a = rshort(); m = (short)(rnd() & 0x7fff); break;
  }
  if (a > 0x4000 || a < -0x4000) cov_hit(CV_RX_CB_ANG_HI);
  if (m < 0) cov_hit(CV_RX_CB_MAG_NEG);
  *angle = a;
  *mag = m;
  return bits;
}
static int fn_id(V22FseDecision d)
{
  if (!d) return 0;
  if (d == fake_cb) return 1;
  if (d == FSEv22_decision12) return 2;
  if (d == FSEv22_decision24) return 3;
  return 9;
}

/* ------------------------------------------------------------------------------------------------ config tables */
static const short c12[12] = {32767, 28378, 16384, 0, -16384, -28378, -32767, -28378, -16384, 0, 16384, 28378};
static const short s12[12] = {0, 16384, 28378, 32767, 28378, 16384, 0, -16384, -28378, -32767, -28378, -16384};
static void make_cfg(int kind, int m)
{
  int k;
  for (k = 0; k < 49; k++) {
    switch (kind) {
      case 0: g_srcI[k] = (short)((FSEv22_COFFS[k] * c12[(m * k) % 12]) >> 15); g_srcQ[k] = (short)((FSEv22_COFFS[k] * s12[(m * k) % 12]) >> 15); break; /* matched tone */
      case 1: g_srcI[k] = FSEv22_COFFS[k]; g_srcQ[k] = 0; break;
      case 2: g_srcI[k] = 0; g_srcQ[k] = 0; break;
      case 3: g_srcI[k] = (short)(rpct(50) ? -32768 : 32767); g_srcQ[k] = (short)(rpct(50) ? -32768 : 32767); break;
      case 4: g_srcI[k] = rshort(); g_srcQ[k] = rshort(); break;
      default: g_srcI[k] = (short)((FSEv22_COFFS[k] * c12[(m * k) % 12]) >> 14); g_srcQ[k] = (short)((FSEv22_COFFS[k] * s12[(m * k) % 12]) >> 14); break;
    }
  }
}

/* a fresh object: garbage-filled struct, init with allocation */
static void fresh(int cfgkind, int m, int fill)
{
  memset(&g_f, fill, sizeof g_f);
  make_cfg(cfgkind, m);
  V22_FSE_init(&g_f, &g_cfgp, 1);
  g_smc = 0;
  g_f.smc = &g_smc;
  g_f.decide = fake_cb;
  cov_hit(CV_INIT_ALLOC);
}
static void drop(void)
{
  V22_FSE_free(&g_f);
  cov_hit(CV_FREE);
}

/* ------------------------------------------------------------------------------------------------ one receive call with coverage bookkeeping */
static int call_rx(const char *tag, int show, int n, short *bits_out)
{
  V22Fse before = g_f;
  int r, i;
  unsigned long cb0 = g_cb_calls;
  g_in_base = g_in;
  g_in_len = (unsigned)(n > 0 ? n : 0);
  if (n > 78) die3("count %d too large for the 0x1c byte output buffers", n);
  r = V22_FSE_receive(&g_f, g_in, bits_out, (short)n);
  g_in_base = NULL;
  emit(show, "%s n=%d -> %d hist=%016llx out=%016llx,%016llx", tag, n, r, fnv_shorts(g_f.hist, 98), fnv_shorts(g_f.outI, 14), fnv_shorts(g_f.outQ, 14));
  emit(show, "%s bits=%016llx", tag, fnv_shorts(bits_out, (unsigned)r));
  for (i = 0; i < r && i < 14; i++) emit(0, "%s sym %d %d %d", tag, i, g_f.outI[i], g_f.outQ[i]);
  if (n > 0 && n < before.rem) cov_hit(CV_RX_PART_POS);
  else if (n != 0 && (short)n < before.rem) cov_hit(CV_RX_PART_NEG);
  if (before.rem == 0 && n != 0) cov_hit(CV_RX_PART_ZERO_REM);
  if (n == 0) cov_hit(CV_RX_ZERO_COUNT);
  if (n >= before.rem && n > 0) cov_hit(CV_RX_FULL);
  if (r > 5) cov_hit(CV_RX_MANY_SYMS);
  if (before.wpos + (n < before.rem ? n : before.rem) > 0x62 && n != 0) cov_hit(n >= before.rem ? CV_RX_SHIFT_FULL : CV_RX_SHIFT_PART);
  if (r > 0) {
    if (before.wpos + before.rem <= 0x30) cov_hit(CV_RX_BASE_LOW);
    if (before.wpos + before.rem > 0x30 || r > 1) cov_hit(CV_RX_BASE_HIGH);
    if (!before.pllEnable) cov_hit(CV_RX_PLL_OFF);
    if (!before.adaptEnable) cov_hit(CV_RX_ADAPT_OFF);
    if (before.adaptEnable) cov_hit(CV_RX_ADAPT_ON);
    if (before.pllEnable) {
      if (before.pllCnt <= 0x30) cov_hit(CV_RX_PLLCNT_LO); else cov_hit(CV_RX_PLLCNT_HI);
      if (g_f.gainSel == 0) cov_hit(CV_RX_GS0);
      if (g_f.gainSel == 1) cov_hit(CV_RX_GS1);
      if (g_f.gainSel == 2) cov_hit(CV_RX_GS2);
    }
    if (before.mu == 1) cov_hit(CV_RX_MU1);
    if (g_f.errPow > 0) cov_hit(CV_RX_ERRPOW_POS); else cov_hit(CV_RX_ERRPOW_ZERO);
    if (g_f.carrier < -0x2000) cov_hit(CV_RX_CARR_NEG);
    if (g_f.carrier > 0x2000) cov_hit(CV_RX_CARR_POS);
    if (g_smc) cov_hit(CV_RX_SMC_NONZERO);
  }
  (void)cb0;
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

/* ------------------------------------------------------------------------------------------------ scenario 0 */
static void sc_life(void)
{
  int t, k;
  cov_def(CV_INIT_ALLOC, "init: allocate"); cov_def(CV_INIT_NOALLOC, "init: no allocation (re-init)"); cov_def(CV_FREE, "free"); cov_def(CV_GETDIAG, "getdiag");
  cov_def(CV_DOUBLE_INIT, "double init (leak)");
  for (t = 0; t < 400; t++) {
    int kind = t % 6, fill = (t % 3 == 0) ? 0xA5 : (t % 3 == 1) ? 0x00 : (int)(rnd() & 0xff);
    char tag[32];
    snprintf(tag, sizeof tag, "L%d", t);
    fresh(kind, 1 + (int)rmod(5), fill);
    dump(t < 12, tag, &g_f, 1);
    /* the taps are the config >> 2, reversed; the history is clear; verify independently */
    for (k = 0; k <= 0x30; k++) {
      if (g_f.tapsI[0x30 - k] != (short)(g_srcI[k] >> 2) || g_f.tapsQ[0x30 - k] != (short)(g_srcQ[k] >> 2)) die3("tap %d", k);
    }
    for (k = 0; k < 98; k++) if (g_f.hist[k] != 0) die3("hist %d", k);
    /* use it, re-init without allocation with a new config, use again */
    fill_in(40, t % 4);
    call_rx("Lr", 0, 7 + (int)rmod(30), g_bits);
    dump(0, tag, &g_f, 1);
    make_cfg((kind + 1) % 6, 2);
    g_f.mu = (short)rmod(2);
    V22_FSE_init(&g_f, &g_cfgp, 0);
    cov_hit(CV_INIT_NOALLOC);
    dump(t < 12, "Lre", &g_f, 1);
    {
      int a = (int)rnd(), b = (int)rnd();
      int out[4] = {1, 2, 3, 4};
      int r = V22_FSE_getdiag(&g_f, a, out, b);
      emit(t < 3, "getdiag %d out=%d,%d,%d,%d", r, out[0], out[1], out[2], out[3]);
      cov_hit(CV_GETDIAG);
    }
    if (t % 17 == 5) { /* a second allocating init without free: the first seven blocks leak (the blob does the same) */
      V22_FSE_init(&g_f, &g_cfgp, 1);
      cov_hit(CV_DOUBLE_INIT);
      dump(0, "Ldbl", &g_f, 1);
      trial_end("life-leak", 1);
      continue;
    }
    drop();
    trial_end("life", 0);
  }
}

/* ------------------------------------------------------------------------------------------------ scenario 1 */
static void poke(void)
{
  g_f.rem = (short)(rpct(60) ? rrange(1, 6) : rrange(0, 6));
  g_f.phase = (short)rrange(0, 5);
  g_f.wpos = (short)(rpct(40) ? rrange(0, 0x62) : rpct(50) ? rrange(0x58, 0x62) : rrange(0, 0x31));
  g_f.pllCnt = (short)(rpct(50) ? rrange(0x2c, 0x34) : rrange(0, 0x40));
  if (rpct(5)) g_f.pllCnt = (short)rrange(-3, 0);
  g_f.gainSel = (short)rrange(0, 2);
  g_f.mu = (short)rrange(0, 1);
  g_f.pllEnable = rpct(80);
  g_f.adaptEnable = rpct(80);
  g_f.errAvg = (short)(rpct(30) ? (rpct(50) ? 0x7fff : -32768) : rpct(30) ? rrange(-0x2000, 0x2000) : rshort());
  g_f.errPow = (short)(rpct(20) ? 0 : rpct(30) ? -rrange(0, 30000) : rrange(0, 0x7fff));
  g_f.carrier = (short)(rpct(30) ? (rpct(50) ? 0x7fff : -32768) : rshort());
  g_f.integ = (short)(rpct(30) ? (rpct(50) ? 0x7fff : -32768) : rshort());
  g_f.errAvg = rpct(10) ? (short)0x1999 : g_f.errAvg;
  g_smc = (unsigned short)rnd();
  if (rpct(30)) g_f.mu = 0;
}
static void sc_rx(void)
{
  int t, c;
  static const int sizes[] = {0, 1, 2, 3, 4, 5, 6, 7, 11, 12, 13, 18, 19, 25, 40, 61, 78, -1, -2, -5, -32768, 32767 - 32767 + 1};
  cov_def(CV_RX_PART_POS, "rx: partial call, count > 0"); cov_def(CV_RX_PART_NEG, "rx: partial call, count < 0"); cov_def(CV_RX_PART_ZERO_REM, "rx: rem = 0 (full path)");
  cov_def(CV_RX_FULL, "rx: full symbol"); cov_def(CV_RX_SHIFT_FULL, "rx: history shift, full path"); cov_def(CV_RX_SHIFT_PART, "rx: history shift, partial path");
  cov_def(CV_RX_BASE_LOW, "rx: window at the start of the history"); cov_def(CV_RX_BASE_HIGH, "rx: window behind the start"); cov_def(CV_RX_MANY_SYMS, "rx: > 5 symbols in one call");
  cov_def(CV_RX_ZERO_COUNT, "rx: count 0"); cov_def(CV_RX_PLL_OFF, "rx: pll disabled"); cov_def(CV_RX_ADAPT_OFF, "rx: adaptation disabled"); cov_def(CV_RX_ADAPT_ON, "rx: adaptation enabled");
  cov_def(CV_RX_PLLCNT_LO, "rx: pllCnt <= 0x30"); cov_def(CV_RX_PLLCNT_HI, "rx: pllCnt > 0x30"); cov_def(CV_RX_GS0, "rx: gainSel 0"); cov_def(CV_RX_GS1, "rx: gainSel 1");
  cov_def(CV_RX_GS2, "rx: gainSel 2"); cov_def(CV_RX_CB_ANG_HI, "rx: callback angle beyond +-0x4000"); cov_def(CV_RX_CB_MAG_NEG, "rx: callback magnitude negative");
  cov_def(CV_RX_ERRPOW_POS, "rx: errPow > 0 after"); cov_def(CV_RX_ERRPOW_ZERO, "rx: errPow <= 0 after"); cov_def(CV_RX_MU1, "rx: mu index 1"); cov_def(CV_RX_CARR_NEG, "rx: carrier < -0x2000");
  cov_def(CV_RX_CARR_POS, "rx: carrier > 0x2000"); cov_def(CV_RX_SMC_NONZERO, "rx: smc nonzero");
  for (t = 0; t < 300; t++) {
    char tag[32];
    snprintf(tag, sizeof tag, "R%d", t);
    fresh(t % 6, 1 + (int)rmod(5), 0xA5);
    g_cb_mode = t % 3;
    for (c = 0; c < 24; c++) {
      int n = sizes[rmod(sizeof sizes / sizeof sizes[0])];
      if (rpct(50)) poke();
      if (n > 78) n = 78;
      fill_in(n > 0 ? n : 1, (int)rmod(6));
      if (n == 1 && rpct(50)) g_f.rem = 1;
      call_rx(tag, (t < 6 && c < 6), n, g_bits);
      dump(t < 4 && c < 4, tag, &g_f, 0);
    }
    dump(t < 20, tag, &g_f, 1);
    drop();
    trial_end("rx", 0);
  }
}

/* ------------------------------------------------------------------------------------------------ scenario 2 / 3 */
static void dec_one(int is24, short I, short Q, int nOut, unsigned short smcPrev, int show)
{
  short ang = (short)0x5a5a, mag = (short)0x6b6b;
  unsigned r;
  g_f.outI[nOut] = I;
  g_f.outQ[nOut] = Q;
  g_f.nOut = (short)nOut;
  g_smc = smcPrev;
  r = is24 ? FSEv22_decision24(&g_f, &ang, &mag) : FSEv22_decision12(&g_f, &ang, &mag);
  emit(show, "D%d I=%d Q=%d n=%d smc=%u -> ret=%u ang=%d mag=%d smc=%u", is24 ? 24 : 12, I, Q, nOut, smcPrev, r, ang, mag, g_smc);
  if (is24) {
    cov_hit(CV_D24_CALL);
    if (I < 0) cov_hit(CV_D24_NEG_I); else cov_hit(CV_D24_HI_I);
    if (Q < 0) cov_hit(CV_D24_NEG_Q);
    if (smcPrev) cov_hit(CV_D24_PREV_NONZERO);
    if (nOut > 6) cov_hit(CV_D24_NOUT_HI);
    if (I == 0x2000 || I == 0x1fff || I == -0x2000 || I == -0x2001 || I == 0x1000 || I == -0x1000) cov_hit(CV_D24_BOUND);
  } else {
    cov_hit(CV_D12_CALL);
    if (smcPrev) cov_hit(CV_D12_PREV_NONZERO);
    if (nOut > 6) cov_hit(CV_D12_NOUT_HI);
  }
}
static void sc_dec(int is24)
{
  int step = is24 ? 256 : 512, i, q, d, n = 0;
  unsigned seen = 0;
  if (is24) {
    cov_def(CV_D24_CALL, "d24: call"); cov_def(CV_D24_NEG_I, "d24: I < 0"); cov_def(CV_D24_NEG_Q, "d24: Q < 0"); cov_def(CV_D24_HI_I, "d24: I >= 0"); cov_def(CV_D24_BOUND, "d24: I at a boundary");
    cov_def(CV_D24_NOUT_HI, "d24: nOut > 6"); cov_def(CV_D24_PREV_NONZERO, "d24: previous smc state nonzero");
  } else {
    cov_def(CV_D12_CALL, "d12: call"); cov_def(CV_D12_PREV_NONZERO, "d12: previous smc state nonzero"); cov_def(CV_D12_NOUT_HI, "d12: nOut > 6");
  }
  fresh(0, 1, 0xA5);
  for (i = -32768; i < 32768; i += step) {
    for (q = -32768; q < 32768; q += step) {
      for (d = 0; d < 3; d++) {
        short I = (short)(i + (d == 1) - (d == 2)), Q = (short)(q + (d == 2) - (d == 1));
        if (d && !is24 && (rnd() & 7)) continue;
        dec_one(is24, I, Q, (int)rmod(14), (unsigned short)(rpct(50) ? 0 : rpct(70) ? (rnd() & 0xc) : rnd()), (n++ % 997) == 0);
      }
    }
  }
  /* the decision regions of the 2400 table: points on the +-0x2000 / 0x1000 / 0 lines and their neighbours, every smc previous state */
  {
    static const short pts[] = {0, 1, -1, 0x1000, 0x1001, 0xfff, -0x1000, -0x1001, -0xfff, 0x2000, 0x2001, 0x1fff, -0x2000, -0x2001, -0x1fff, 0x3000, 0x3001, 0x2fff, -0x3000, -0x3001, -0x2fff,
                                0x4000, 0x4001, 0x3fff, -0x4000, -0x4001, -0x3fff, 0x7fff, -32768, -32767, 0x6000, -0x6000, 0x5000, -0x5000, 0x0800, -0x0800, 0x1800, -0x1800, 0x2800, 0x3800};
    unsigned a, b, s;
    for (a = 0; a < sizeof pts / sizeof pts[0]; a++)
      for (b = 0; b < sizeof pts / sizeof pts[0]; b++)
        for (s = 0; s < 16; s += 1 + (unsigned)(a + b) % 3) dec_one(is24, pts[a], pts[b], (int)(s % 14), (unsigned short)(s * 0x11 + (s & 3)), (a == b && s == 0));
  }
  for (i = 0; i < 400000; i++) {
    short I = rshort(), Q = rshort();
    if (rpct(30)) {
      I = (short)((I & ~0xfff) + (rpct(50) ? 0x1000 : 0x2000) * (rpct(50) ? 1 : -1));
      Q = (short)((Q & ~0xfff) + (rpct(50) ? 0x1000 : 0x2000) * (rpct(50) ? 1 : -1));
    }
    if (rpct(30)) { I = (short)(I / 4); Q = (short)(Q / 4); }
    dec_one(is24, I, Q, (int)rmod(14), (unsigned short)rnd(), (i % 49999) == 0);
    seen++;
  }
  /* the SMC state word is read through f->smc: a state with all 16 values of the previous quadrant bits */
  emit(1, "decisions %u", seen);
  drop();
  trial_end("dec", 0);
}

/* ------------------------------------------------------------------------------------------------ scenario 4: end to end */
static int g_ph[12];
static void gen_signal(short *p, int n, int kind, int fm, int is24, unsigned *symclk, int *symph, int *symamp, int *pos)
{
  int i;
  for (i = 0; i < n; i++) {
    int v;
    if ((*pos)++ % 6 == 0) {
      *symph = (int)rmod(4) * 3;
      *symamp = is24 ? (rpct(50) ? 3 : 1) : 3;
    }
    (void)symclk;
    v = (c12[(fm * (*pos) + *symph) % 12] * *symamp) >> 2;   /* up to 3/4 of full scale: saturation comes from the kind below */
    switch (kind) {
      case 0: break;
      case 1: v += rrange(-2000, 2000); break;
      case 2: v = v * 2; if (v > 32767) v = 32767; if (v < -32768) v = -32768; break;
      case 3: v = rshort(); break;
      case 4: v = (v >> 4) + rrange(-30, 30); break;
      default: v += rrange(-9000, 9000); break;
    }
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    p[i] = (short)v;
  }
}
static void sc_e2e(void)
{
  int run, c;
  cov_def(CV_E2E_12, "e2e: 1200"); cov_def(CV_E2E_24, "e2e: 2400"); cov_def(CV_E2E_ONE_SAMPLE, "e2e: one-sample calls"); cov_def(CV_E2E_ADAPT, "e2e: adaptation ran");
  cov_def(CV_E2E_PLL_SWITCH, "e2e: gainSel changed during a stream"); cov_def(CV_E2E_NOISE, "e2e: noisy stream"); cov_def(CV_E2E_SAT, "e2e: saturated stream"); cov_def(CV_E2E_BITS_VAR, "e2e: decoded bits vary");
  for (run = 0; run < 40; run++) {
    int is24 = run & 1, kind = (run / 2) % 6, fm = 1 + (run / 12) % 5, cfgkind = (run % 7 == 6) ? 5 : 0;
    int pos = 0, symph = 0, symamp = 3, total = 0, gsStart, done = 0;
    unsigned symclk = 0, bitsum = 0, lastbits = 0xffffffffu;
    char tag[32];
    snprintf(tag, sizeof tag, "E%d", run);
    fresh(cfgkind, (run % 3 == 2) ? 2 : 1, 0xA5);
    g_f.decide = is24 ? FSEv22_decision24 : FSEv22_decision12;
    g_f.mu = (short)(run % 4 == 3);
    if (run % 5 == 4) g_f.pllEnable = 0;
    if (run % 6 == 5) g_f.adaptEnable = 0;
    cov_hit(is24 ? CV_E2E_24 : CV_E2E_12);
    if (kind == 1 || kind == 5) cov_hit(CV_E2E_NOISE);
    if (kind == 2) cov_hit(CV_E2E_SAT);
    gsStart = g_f.gainSel;
    while (done < 14000) {
      int n = (run % 8 == 7) ? 1 : (rpct(15) ? rrange(1, 6) : rrange(1, 40));
      gen_signal(g_in, n, kind, fm, is24, &symclk, &symph, &symamp, &pos);
      call_rx(tag, (run < 2 && done < 200), n, g_bits);
      if (n == 1) cov_hit(CV_E2E_ONE_SAMPLE);
      {
        int r;
        for (r = 0; r < g_f.nOut; r++) {
          bitsum += (unsigned)(unsigned short)g_bits[r];
          if (lastbits != 0xffffffffu && lastbits != (unsigned)(unsigned short)g_bits[r]) cov_hit(CV_E2E_BITS_VAR);
          lastbits = (unsigned)(unsigned short)g_bits[r];
        }
      }
      if (g_f.gainSel != gsStart) cov_hit(CV_E2E_PLL_SWITCH);
      if (g_f.pllCnt > 0x30 && g_f.adaptEnable && g_f.errPow > 0) cov_hit(CV_E2E_ADAPT);
      done += n;
      total++;
      if ((total % 53) == 0) dump(0, tag, &g_f, 1);
    }
    dump(1, tag, &g_f, 1);
    emit(1, "%s bitsum=%u calls=%d", tag, bitsum, total);
    drop();
    trial_end("e2e", 0);
  }
  (void)c;
}

/* ------------------------------------------------------------------------------------------------ scenario 5: a million single-symbol calls over fully randomised state: hits the exact
 * boundary values of every 16-bit range test (+-0x4000 of the angle wraps, the 0x1999 / 0x666 error thresholds, the window start 0x30 / 0x31, the history shift 0x62) */
static void sc_mass(void)
{
  int it;
  cov_def(CV_MASS_CALLS, "mass: single-symbol calls");
  fresh(4, 1, 0xA5);
  g_log_memcpy = 0;
  g_cb_log = 0;
  for (it = 0; it < 1500000; it++) {
    int k;
    short bits[4];
    g_cb_mode = (int)rmod(3);
    g_f.rem = 1;
    g_f.phase = (short)rrange(0, 5);
    g_f.wpos = (short)(rpct(30) ? rrange(0x2e, 0x32) : rpct(30) ? rrange(0x5f, 0x62) : rrange(0, 0x62));
    g_f.pllCnt = (short)(rpct(40) ? rrange(0x2e, 0x33) : rpct(30) ? 0x40 + (int)rmod(8) : rrange(0, 0x40));
    g_f.gainSel = (short)rrange(0, 2);
    g_f.mu = (short)rrange(0, 1);
    g_f.pllEnable = rpct(92);
    g_f.adaptEnable = rpct(90);
    g_f.errAvg = (short)(rpct(35) ? rrange(-0x2000, 0x2000) : rpct(35) ? rrange(-0x800, 0x800) : rshort());
    g_f.errPow = (short)rrange(0, 0x7fff);
    g_f.carrier = (short)(rpct(60) ? rshort() : rrange(-0x5000, 0x5000));
    g_f.integ = (short)(rpct(60) ? rshort() : rrange(-0x5000, 0x5000));
    g_smc = (unsigned short)rnd();
    if (it % 1000 == 0) for (k = 0; k < 49; k++) { g_f.tapsI[k] = rshort(); g_f.tapsQ[k] = rshort(); }
    if (rpct(50)) { for (k = 0; k < 98; k++) g_f.hist[k] = (short)(rpct(50) ? rshort() : rrange(-4000, 4000)); }
    g_in[0] = rshort();
    g_in_base = g_in;
    g_in_len = 1;
    {
      int r = V22_FSE_receive(&g_f, g_in, bits, 1);
      g_in_base = NULL;
      emit(it % 200000 == 0, "M%d -> %d b=%d I=%d Q=%d car=%d int=%d eA=%d eP=%d gs=%d cnt=%d", it, r, bits[0], g_f.outI[0], g_f.outQ[0], g_f.carrier, g_f.integ, g_f.errAvg, g_f.errPow, g_f.gainSel, g_f.pllCnt);
      cov_hit(CV_MASS_CALLS);
    }
  }
  dump(1, "Mend", &g_f, 1);
  drop();
  trial_end("mass", 0);
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("R04_SEED");
  if (seed) g_lcg ^= (unsigned)strtoul(seed, NULL, 0) * 2654435761u;
  g_lcg ^= (unsigned)sc * 0x9e3779b9u;
  switch (sc) {
    case 0: sc_life(); break;
    case 1: sc_rx(); break;
    case 2: sc_dec(0); break;
    case 3: sc_dec(1); break;
    case 4: sc_e2e(); break;
    case 5: sc_mass(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  emit(1, "calls: malloc %lu free %lu memcpy %lu callbacks %lu", g_nmalloc, g_nfree, g_nmemcpy, g_cb_calls);
  cov_finish();
  return 0;
}
