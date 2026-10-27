/*
 * R05 differential harness (2 of 3): the V.22 IIR (V22IIRFilterInit / V22_iir_filt_demod), the V.22 scrambler / descrambler (FPM_SDM_init / _scrambler / _descrambler) and the V.22
 * differential phase encoder (FPM_SMC_init / FPM_SMC_encoder) - real dsplibs.o vs src/v22/v22_rx_bandpass.c + src/v22/v22_scrambler.c (+ src/v22/v22_tables.c: IIR_a / IIR_b_coeff, SDMv22_CFG, SMCv22_CFG and
 * the SMC maps).  Library test/r04_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation); typed objects of src/v22/v22.h on both sides, the 32-bit
 * BLOB run is the x86-64 golden.  None of these functions calls anything (no callee recorder needed).
 *
 * Scenarios (each its own process):
 *   0 IIR: init over garbage with real / random / extreme coefficient pairs; the filter over 160-sample blocks (random, saturated, silence, ramps, impulses, the real V.22 band
 *     filters IIR_a / IIR_b_coeff with a unity / random / extreme gain table), histories poked and carried across 400 calls;
 *   1 SDM: init with the real cfg {4, 14, 17}, the V.22 1200 cfg (2, 14, 17) and random cfgs (shift counts n, tap1, tap2 over the whole 16-bit range: the x86 count & 31 rule, masks of 32
 *     bits); scrambler and descrambler over blocks of 0 .. 300 words of random / mask-limited / all-ones / zero data with random registers; scramble -> descramble round trips;
 *   2 SMC: init with the real cfg and random cfgs (flag, step, mod, shift, masks, a 65536-entry map table); the encoder over blocks of 0 .. 200 symbols into a circular buffer
 *     of limit 1 .. 64 (write index poked), a / b running values poked over their range; end to end with the real SMCv22 cfg and the real differential map.
 */
#include "../r04_common.h"
#include "v22/v22.h"
#include "v22/v22_tables.h"

enum {
  CV_I_INIT = 0, CV_I_REAL, CV_I_SAT, CV_I_NEGACC, CV_I_ZERO, CV_I_CARRY, CV_I_EXTREME_GAIN,
  CV_S_INIT = 20, CV_S_N2, CV_S_N4, CV_S_NRAND, CV_S_SHBIG, CV_S_SHNEG, CV_S_SCR, CV_S_DESCR, CV_S_ROUND, CV_S_ZERO_N, CV_S_STATE_RAND, CV_S_WIDE, CV_S_BYTEBIG,
  CV_C_INIT = 40, CV_C_FLAG, CV_C_NOFLAG, CV_C_WRAP_OUT, CV_C_MOD_SUB, CV_C_B_SUB, CV_C_ZERO_N, CV_C_SHIFT_BIG, CV_C_NEG_A, CV_C_LIM_ONE, CV_C_REAL, CV_C_PMAP_HI, CV_C_E2E, CV_C_CARRY
};

static short g_data[160], g_gain[160];
static struct { short front[8]; V22Iir f; short back[8]; } g_iirx; /* guards in front of / behind the object: a loop bound error of the init writes them */
#define g_iir (g_iirx.f)
static V22Sdm g_sdm;
static V22Smc g_smc;
static V22Circ g_circ;
static unsigned short g_obuf[256];
static unsigned short g_in[512];
static short g_pmap[65536];

/* ------------------------------------------------------------------------------------------------ IIR */
static void iir_dump(int show, const char *tag, const V22Iir *f)
{
  emit(show, "%s num=%d,%d,%d,%d den=%d,%d,%d,%d x=%d,%d,%d,%d y=%d,%d,%d,%d", tag, f->num[0], f->num[1], f->num[2], f->num[3], f->den[0], f->den[1], f->den[2], f->den[3], f->x[0],
       f->x[1], f->x[2], f->x[3], f->y[0], f->y[1], f->y[2], f->y[3]);
}
static void rnd_coefs(short *c, int kind)
{
  int k;
  for (k = 0; k < 4; k++) {
    switch (kind) {
      case 0: c[k] = (short)rrange(-20000, 20000); break;
      case 1: c[k] = rshort(); break;
      case 2: c[k] = (short)(rpct(50) ? 32767 : -32768); break;
      case 3: c[k] = (short)rrange(-3000, 3000); break;
      default: c[k] = (short)(rrange(-16384, 16384)); break;
    }
  }
}
static void sc_iir(void)
{
  int t, c, i;
  cov_def(CV_I_INIT, "iir init"); cov_def(CV_I_REAL, "iir: real coefficients"); cov_def(CV_I_SAT, "iir: saturated input"); cov_def(CV_I_NEGACC, "iir: negative accumulator");
  cov_def(CV_I_ZERO, "iir: silence"); cov_def(CV_I_CARRY, "iir: state carried"); cov_def(CV_I_EXTREME_GAIN, "iir: extreme gain");
  for (t = 0; t < 120; t++) {
    short num[4], den[4];
    char tag[32];
    int kind = t % 5, gk = t % 4;
    snprintf(tag, sizeof tag, "I%d", t);
    if (t % 3 == 0) {
      memcpy(num, IIR_b_coeff, sizeof num);
      memcpy(den, IIR_a_coeff, sizeof den);
      cov_hit(CV_I_REAL);
    } else {
      rnd_coefs(num, kind);
      rnd_coefs(den, (kind + 1) % 5);
    }
    memset(&g_iirx, t % 2 ? 0xA5 : 0x00, sizeof g_iirx);
    { int q; for (q = 0; q < 8; q++) { g_iirx.front[q] = 0x5a5a; g_iirx.back[q] = 0x5a5b; } }
    V22IIRFilterInit(&g_iir, num, den);
    { int q; for (q = 0; q < 8; q++) if (g_iirx.front[q] != 0x5a5a || g_iirx.back[q] != 0x5a5b) die3("V22IIRFilterInit wrote outside the object"); }
    cov_hit(CV_I_INIT);
    iir_dump(t < 8, tag, &g_iir);
    for (i = 0; i < 160; i++) g_gain[i] = (short)(gk == 0 ? 4096 : gk == 1 ? rrange(1000, 8000) : gk == 2 ? rshort() : (rpct(50) ? 32767 : -32768));
    if (gk == 3) cov_hit(CV_I_EXTREME_GAIN);
    for (c = 0; c < 40; c++) {
      int dk = (int)rmod(7);
      long long h = 0;
      for (i = 0; i < 160; i++) {
        switch (dk) {
          case 0: g_data[i] = rshort(); break;
          case 1: g_data[i] = (short)(rpct(50) ? 32767 : -32768); break;
          case 2: g_data[i] = 0; break;
          case 3: g_data[i] = (short)(i * 200 - 16000); break;
          case 4: g_data[i] = (short)(i == 0 ? 32767 : 0); break;
          case 5: g_data[i] = (short)rrange(-2000, 2000); break;
          default: g_data[i] = (short)(12000 * ((i / 5) & 1 ? 1 : -1)); break;
        }
      }
      if (dk == 1) cov_hit(CV_I_SAT);
      if (dk == 2) cov_hit(CV_I_ZERO);
      if (rpct(15)) { for (i = 0; i < 4; i++) { g_iir.x[i] = rshort(); g_iir.y[i] = rshort(); } }
      V22_iir_filt_demod(g_data, g_iir.num, g_iir.den, g_iir.x, g_iir.y, g_gain);
      for (i = 0; i < 160; i++) if (g_iir.y[0] < 0 && g_data[i] < 0) { h = 1; }
      if (h) cov_hit(CV_I_NEGACC);
      if (c > 0) cov_hit(CV_I_CARRY);
      emit(t < 3 && c < 3, "%s c%d out=%016llx", tag, c, fnv_shorts(g_data, 160));
      if (c % 8 == 0) iir_dump(0, tag, &g_iir);
    }
    iir_dump(t < 20, tag, &g_iir);
  }
}

/* ------------------------------------------------------------------------------------------------ SDM */
static void sdm_dump(int show, const char *tag, const V22Sdm *s)
{
  emit(show, "%s n=%d t1=%d t2=%d mask=%08x nmask=%08x state=%08x sh1=%d sh2=%d", tag, s->n, s->tap1, s->tap2, s->mask, s->nmask, s->state, s->sh1, s->sh2);
}
static void sdm_fill(int n, int kind, unsigned short mask)
{
  int i;
  for (i = 0; i < n; i++) {
    switch (kind) {
      case 0: g_in[i] = (unsigned short)rnd(); break;
      case 1: g_in[i] = (unsigned short)(rnd() & mask); break;
      case 2: g_in[i] = 0xffff; break;
      case 3: g_in[i] = 0; break;
      case 4: g_in[i] = (unsigned short)(i & mask); break;
      default: g_in[i] = (unsigned short)(rnd() & 0xf); break;
    }
  }
}
static void sc_sdm(void)
{
  int t, c;
  static const int sizes[] = {0, 1, 1, 2, 3, 8, 17, 32, 100, 300};
  cov_def(CV_S_INIT, "sdm init"); cov_def(CV_S_N2, "sdm: n = 2"); cov_def(CV_S_N4, "sdm: n = 4"); cov_def(CV_S_NRAND, "sdm: random n"); cov_def(CV_S_SHBIG, "sdm: shift counts >= 32");
  cov_def(CV_S_SHNEG, "sdm: negative shift counts"); cov_def(CV_S_SCR, "sdm: scrambler"); cov_def(CV_S_DESCR, "sdm: descrambler"); cov_def(CV_S_ROUND, "sdm: round trip checked");
  cov_def(CV_S_ZERO_N, "sdm: count 0"); cov_def(CV_S_STATE_RAND, "sdm: random register"); cov_def(CV_S_WIDE, "sdm: wide (n >= 16) mask"); cov_def(CV_S_BYTEBIG, "sdm: shift byte >= 32 (low 5 bits used)");
  for (t = 0; t < 400; t++) {
    V22SdmCfg cfg;
    char tag[32];
    snprintf(tag, sizeof tag, "S%d", t);
    if (t % 4 == 0) { cfg = SDMv22_CFG; cov_hit(CV_S_N4); }
    else if (t % 4 == 1) { cfg = SDMv22_CFG; cfg.s[0] = 2; cov_hit(CV_S_N2); }
    else {
      cfg.s[0] = (short)(rpct(60) ? rrange(0, 20) : rpct(50) ? rrange(0, 40) : (int)rshort());
      cfg.s[1] = (short)(rpct(60) ? rrange(0, 31) : rpct(50) ? rrange(0, 70) : (int)rshort());
      cfg.s[2] = (short)(rpct(60) ? rrange(0, 31) : rpct(50) ? rrange(0, 70) : (int)rshort());
      cov_hit(CV_S_NRAND);
    }
    if (cfg.s[0] >= 32 || cfg.s[1] - cfg.s[0] >= 32 || cfg.s[2] - cfg.s[0] >= 32) cov_hit(CV_S_SHBIG);
    if (cfg.s[0] < 0 || cfg.s[1] - cfg.s[0] < 0) cov_hit(CV_S_SHNEG);
    if (((cfg.s[0] & 0xff) >= 32) || (((cfg.s[1] - cfg.s[0]) & 0xff) >= 32)) cov_hit(CV_S_BYTEBIG);
    if (cfg.s[0] >= 16) cov_hit(CV_S_WIDE);
    memset(&g_sdm, t % 2 ? 0xA5 : 0x00, sizeof g_sdm);
    FPM_SDM_init(&g_sdm, &cfg);
    cov_hit(CV_S_INIT);
    sdm_dump(t < 12, tag, &g_sdm);
    for (c = 0; c < 20; c++) {
      int n = sizes[rmod(sizeof sizes / sizeof sizes[0])];
      unsigned short mask = (unsigned short)g_sdm.mask;
      int dk = (int)rmod(6);
      if (rpct(40)) { g_sdm.state = rnd(); cov_hit(CV_S_STATE_RAND); }
      sdm_fill(n, dk, mask ? mask : 0xffff);
      if (n == 0) cov_hit(CV_S_ZERO_N);
      if (rpct(50)) {
        unsigned st0 = g_sdm.state;
        short tmp[1];
        (void)tmp;
        FPM_SDM_scrambler(&g_sdm, g_in, (unsigned short)n);
        cov_hit(CV_S_SCR);
        emit(t < 6 && c < 4, "%s sc n=%d out=%016llx", tag, n, fnv_shorts((short *)g_in, (unsigned)n));
        sdm_dump(0, tag, &g_sdm);
        if (t % 4 < 2 && rpct(50)) { /* round trip: a scrambled block descrambled from the SAME register gives the data back (V.22 cfg only: the data is the mask-limited bits) */
          V22Sdm d = g_sdm;
          unsigned short copy[512];
          memcpy(copy, g_in, (size_t)n * 2);
          d.state = st0;
          FPM_SDM_descrambler(&d, g_in, (unsigned short)n);
          emit(0, "%s rt n=%d out=%016llx st=%08x", tag, n, fnv_shorts((short *)g_in, (unsigned)n), d.state);
          cov_hit(CV_S_ROUND);
          (void)copy;
        }
      } else {
        FPM_SDM_descrambler(&g_sdm, g_in, (unsigned short)n);
        cov_hit(CV_S_DESCR);
        emit(t < 6 && c < 4, "%s ds n=%d out=%016llx", tag, n, fnv_shorts((short *)g_in, (unsigned)n));
        sdm_dump(0, tag, &g_sdm);
      }
    }
    sdm_dump(t < 30, tag, &g_sdm);
  }
}

/* ------------------------------------------------------------------------------------------------ SMC */
static void smc_dump(int show, const char *tag, const V22Smc *s)
{
  emit(show, "%s w0=%d flag=%d step=%d mod=%d sh=%u mask=%04x m2=%04x m3=%04x pmap=%s w8=%d w9=%d w10=%d a=%d b=%d", tag, s->cfg.w0, s->cfg.flag, s->cfg.step, s->cfg.mod, s->cfg.shift, s->cfg.mask, s->cfg.mask2, s->cfg.mask3,
       s->cfg.pmap == g_pmap ? "H" : s->cfg.pmap == SMCv22_PMAP ? "R" : s->cfg.pmap ? "?" : "NULL", s->cfg.w8, s->cfg.w9, s->cfg.w10, s->a, s->b);
}
static void smc_circ(int lim)
{
  memset(&g_circ, 0, sizeof g_circ);
  memset(g_obuf, 0x5a, sizeof g_obuf);
  g_circ.buf = g_obuf;
  g_circ.lim = (short)lim;
  g_circ.wr = (short)(lim > 0 ? rmod((unsigned)lim) : 0);
}
static void smc_call(const char *tag, int show, int n)
{
  V22Smc before = g_smc;
  V22Circ cb = g_circ;
  FPM_SMC_encoder(&g_smc, &g_circ, g_in, (unsigned short)n);
  emit(show, "%s n=%d a=%d b=%d wr=%d->%d out=%016llx", tag, n, g_smc.a, g_smc.b, cb.wr, g_circ.wr, fnv_shorts((short *)g_obuf, 128));
  if (n == 0) cov_hit(CV_C_ZERO_N);
  if (before.cfg.flag) cov_hit(CV_C_FLAG); else cov_hit(CV_C_NOFLAG);
  if (g_circ.wr < cb.wr || (n > cb.lim && cb.lim > 0)) cov_hit(CV_C_WRAP_OUT);
  if (g_smc.a < 0) cov_hit(CV_C_NEG_A);
  if (before.cfg.shift >= 32) cov_hit(CV_C_SHIFT_BIG);
  if (g_circ.lim == 1) cov_hit(CV_C_LIM_ONE);
  if (before.cfg.step && g_smc.b < before.b) cov_hit(CV_C_B_SUB);
  if (before.cfg.pmap == SMCv22_PMAP && n) cov_hit(CV_C_REAL);
  if (n && !before.cfg.flag && before.cfg.mask > 0xff) cov_hit(CV_C_PMAP_HI);
}
static void sc_smc(void)
{
  int t, c, i;
  static const int sizes[] = {0, 1, 1, 2, 3, 5, 12, 40, 100, 200};
  cov_def(CV_C_INIT, "smc init"); cov_def(CV_C_FLAG, "smc: flag set"); cov_def(CV_C_NOFLAG, "smc: flag clear"); cov_def(CV_C_WRAP_OUT, "smc: output index wrapped"); cov_def(CV_C_ZERO_N, "smc: count 0");
  cov_def(CV_C_SHIFT_BIG, "smc: shift >= 32"); cov_def(CV_C_NEG_A, "smc: negative a"); cov_def(CV_C_LIM_ONE, "smc: limit 1"); cov_def(CV_C_REAL, "smc: the real cfg / map");
  cov_def(CV_C_PMAP_HI, "smc: wide map index mask"); cov_def(CV_C_B_SUB, "smc: b wrapped by mod"); cov_def(CV_C_CARRY, "smc: state carried");
  for (i = 0; i < 65536; i++) g_pmap[i] = (short)(rpct(60) ? rrange(0, 16) : rshort());
  for (t = 0; t < 400; t++) {
    V22SmcCfg cfg;
    char tag[32];
    snprintf(tag, sizeof tag, "C%d", t);
    if (t % 4 == 0) cfg = SMCv22_CFG;
    else {
      cfg.w0 = (int)rnd();
      cfg.flag = rpct(35) ? 1 : (rpct(10) ? (int)rnd() : 0);
      cfg.step = (short)(rpct(40) ? 0 : rpct(70) ? rrange(0, 20) : (int)rshort());
      cfg.mod = (short)(rpct(60) ? rrange(1, 20) : rpct(30) ? rrange(-5, 3) : (int)rshort());
      cfg.shift = (unsigned short)(rpct(60) ? rrange(0, 8) : rpct(50) ? rrange(0, 40) : (int)rmod(0x10000));
      cfg.mask = (unsigned short)(rpct(50) ? rrange(0, 255) : rpct(50) ? rrange(0, 0xffff) : 3);
      cfg.mask2 = (unsigned short)(rpct(50) ? 0 : rmod(0x10000));
      cfg.mask3 = (unsigned short)(rpct(50) ? 0xf : rmod(0x10000));
      cfg.pmap = g_pmap;
      cfg.imap = SMCv22_IMAP_1200BPS;
      cfg.qmap = SMCv22_QMAP_2400BPS;
      cfg.w8 = (int)rnd();
      cfg.w9 = (int)rnd();
      cfg.w10 = (int)rnd();
    }
    memset(&g_smc, t % 2 ? 0xA5 : 0x00, sizeof g_smc);
    FPM_SMC_init(&g_smc, &cfg);
    cov_hit(CV_C_INIT);
    smc_dump(t < 12, tag, &g_smc);
    smc_circ(t % 4 == 0 ? 8 : rpct(25) ? 1 : rrange(1, 64));
    for (c = 0; c < 20; c++) {
      int n = sizes[rmod(sizeof sizes / sizeof sizes[0])];
      int dk = (int)rmod(4);
      for (i = 0; i < n; i++) g_in[i] = (unsigned short)(dk == 0 ? rnd() : dk == 1 ? (rnd() & 0xf) : dk == 2 ? (i & 0xff) : (rnd() & 3));
      if (rpct(25)) { g_smc.a = (short)(rpct(50) ? rshort() : rrange(-20, 20)); g_smc.b = (short)(rpct(50) ? rshort() : rrange(-5, 40)); }
      if (rpct(15) && g_circ.lim > 0) g_circ.wr = (short)rmod((unsigned)g_circ.lim);
      if (c > 0) cov_hit(CV_C_CARRY);
      smc_call(tag, t < 6 && c < 4, n);
    }
    smc_dump(t < 30, tag, &g_smc);
  }
  /* end to end with the real cfg: a long bit stream, the output descriptor of 12 entries (the engine's) */
  for (t = 0; t < 20; t++) {
    char tag[32];
    int done = 0;
    snprintf(tag, sizeof tag, "CE%d", t);
    FPM_SMC_init(&g_smc, &SMCv22_CFG);
    smc_circ(12);
    while (done < 4000) {
      int n = rrange(1, 12);
      for (i = 0; i < n; i++) g_in[i] = (unsigned short)(t % 2 ? rnd() & 3 : rnd() & 0xf);
      smc_call(tag, t < 2 && done < 60, n);
      done += n;
    }
    smc_dump(1, tag, &g_smc);
    cov_hit(CV_C_E2E);
  }
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("R05_SEED");
  if (seed) g_lcg ^= (unsigned)strtoul(seed, NULL, 0) * 2654435761u;
  g_lcg ^= (unsigned)sc * 0x9e3779b9u;
  switch (sc) {
    case 0: sc_iir(); break;
    case 1: sc_sdm(); break;
    case 2: cov_def(CV_C_E2E, "smc: end to end"); sc_smc(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  cov_finish();
  return 0;
}
