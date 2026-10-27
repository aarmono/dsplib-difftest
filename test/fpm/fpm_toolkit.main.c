/*
 * Differential test harness for the shared FPM_* DSP primitives toolkit
 * (fpm_math/fpm_iir/fpm_mrf/fpm_agc/fpm_tone/fpm_fsm/fpm_fsd/fpm_mtd).
 *
 * Same pattern as rd_diff_main.c: this ONE source is compiled into two
 * binaries - fpm_test_blob (linked against the real dsplibs.o, whose
 * FPM_* functions are globally exported per `nm dsplibs.o`) and
 * fpm_test_port (linked against ../src/fpm/fpm_*.c). All test-fixture
 * coefficient/config arrays are private `static const` copies here (byte-
 * exact, copied from ../src/fpm/fpm_tables.c / ../src/fsk/fsk_tables.c) -
 * NOT the real src/ table .c files - because several of those symbol names
 * (B103_IIR_LPF, AGCb103_CFG, ...) are themselves GLOBAL in dsplibs.o, so
 * linking their definitions into the *_blob binary alongside dsplibs.o
 * would be a duplicate-symbol error. Keeping the fixtures `static` here
 * sidesteps that entirely - only the FPM_* function entry points differ
 * between the two binaries.
 */

#include <stdio.h>
#include <string.h>
#include "../src/fpm/fpm.h"	/* U01: the typed objects (i386 layout at -m32 = the blob's raw object, real pointer members on x86-64) */

/* U01: a table pointer inside a cfg word.  -m32: the address; x86-64: the CFGP handle of the same-named table (the fixture copy below is registered
 * as that table, so the port's cfgp_ptr() resolves it).  No effect on what the blob side computes. */
#if defined(__x86_64__)
#define HW(sym, ptr) (cfgp_set(CFGH_##sym, (ptr)), CFGP(sym))
#else
#define HW(sym, ptr) ((int)(long)(ptr))
#endif

/* ---- FPM_* entry points under test (same signatures for both link targets) ---- */
extern void FPM_phasor(short *p);
extern void FPM_phasor_demod(short *p);
extern unsigned int FPM_sqrt_dp(unsigned int x);
extern int FPM_div(unsigned short a, short *mantissa, short *shift);
extern short FPM_rms(const short *data, short count);
extern short FPM_iir_filt(short seed, const short *coefs, short *state, short count);
extern void FPM_iir_filt_II(short *data, short *coefs, short *state, short param_4, short param_5);
extern int FPM_MRF_init(FpmMrf *obj, const int *cfg, int param_3);
extern unsigned int FPM_MRF_filter(FpmMrf *obj, short *in, short *out, short count);
extern void FPM_AGC_init(FpmAgc *obj, const int *cfg, int param_3);
extern void FPM_AGC_agc(FpmAgc *obj, short *data, unsigned short count);
extern void FPM_AGC_Freeze(FpmAgc *obj);
extern void FPM_AGC_Release(FpmAgc *obj);
extern unsigned int dsplibs_debug_level;
extern FpmTone *FPM_TONE_create(FpmTone *obj, const int *cfg);
extern void FPM_TONE_delete(FpmTone *obj);
extern int FPM_TONE_detect(FpmTone *obj, short *data, short count);
extern int FPM_TONE_generate(FpmTone *obj, short *out, short count);
extern int FPM_TONE_generate_demod(FpmTone *obj, short *out, short count);
extern void FPM_TONE_set_freq(FpmTone *obj, short freq);
extern void FPM_TONE_set_scale(FpmTone *obj, short scale);
extern void FPM_FSM_init(FpmFsm *obj, const int *cfg);
extern short FPM_FSM_modulate(FpmFsm *obj, unsigned short *bits, short *out, short count);
extern void FPM_FSM_delete(FpmFsm *obj);
extern void FPM_FSD_init(FpmFsd *obj, const int *cfg, int param_3);
extern unsigned int FPM_FSD_demodulate(FpmFsd *obj, unsigned short *in, short *out, short count);
extern void FPM_FSD_free(FpmFsd *obj);
extern FpmMtd *FPM_MTD_create(FpmMtd *obj, const int *cfg);
extern unsigned int FPM_MTD_detect(FpmMtd *obj, short *data, short count);
extern void FPM_MTD_delete(FpmMtd *obj);

/* ---- private byte-exact fixture copies (see file header) ---- */

static const short g_ToneLPF[53] = {
    -56, -45, -27, -3, 26, 64, 109, 161, 220, 286, 357, 434,
    516, 600, 687, 773, 860, 943, 1023, 1098, 1166, 1226, 1276, 1317,
    1346, 1364, 1370, 1364, 1346, 1317, 1276, 1226, 1166, 1098, 1023, 943,
    860, 773, 687, 600, 516, 434, 357, 286, 220, 161, 109, 64,
    26, -3, -27, -45, -56,
};

static const short g_B103_IIR_LPF[15] = {
    -8989, 233, 23020, 162, 233, -12864, 16384, 22466, -15266, 16384,
    -15520, 16384, 22435, -19789, 16384,
};

static const short g_B103_CHAN_INTRP[15] = {
    56, -104, 259, -583, 1225, -2768, 13089, 6543, -2521, 1368,
    -774, 414, -198, 83, -46,
};

static const short g_B103_MRF_FILT_RX[90] = {
    -948, -349, -370, -359, -307, -211, -67, 123, 355, 619,
    902, 1189, 1462, 1697, 1877, 1979, 1987, 1887, 1669, 1332,
    881, 328, -307, -997, -1709, -2407, -3053, -3607, -4034, -4302,
    -4385, -4269, -3948, -3426, -2720, -1857, -874, 186, 1272, 2331,
    3312, 4165, 4847, 5321, 5565, 5565, 5321, 4847, 4165, 3312,
    2331, 1272, 186, -874, -1857, -2720, -3426, -3948, -4269, -4385,
    -4302, -4034, -3607, -3053, -2407, -1709, -997, -307, 328, 881,
    1332, 1669, 1887, 1987, 1979, 1877, 1697, 1462, 1189, 902,
    619, 355, 123, -67, -211, -307, -359, -370, -349, -948,
};

static const short g_MTDb103_COEF[10] = {
    -3431, 2331, 5232, 4205, 2331, -12607, 16384, -4629, 19426, 16384,
};

static const short g_AGC_DEF_ALPHA[2] = { 16384, 32604 };
static const short g_AGC_DEF_BETA[2] = { 16384, 1638 };

static const int g_FPM_TONE_CFG_raw[9] = {
    0x6ccc0834, 0x600001c2, 0x00010148, 0x00007800, 0, 0x00000035, 0x00000000, 0x00284000, 0x00000000,
};

static int g_agcb103_cfg[6];
static int g_fsd_cfg[7];
static int g_mtd_cfg[3];
static int g_tone_cfg[9];

static void init_fixtures(void)
{
  int i;
  g_agcb103_cfg[0] = 0xa4000;
  g_agcb103_cfg[1] = 0x3e80050;
  g_agcb103_cfg[2] = 0x240001;
  g_agcb103_cfg[3] = HW(B103_AGC_DEF_ALPHA, g_AGC_DEF_ALPHA);
  g_agcb103_cfg[4] = HW(B103_AGC_DEF_BETA, g_AGC_DEF_BETA);
  g_agcb103_cfg[5] = 0x9e;

  g_fsd_cfg[0] = HW(B103_CHAN_INTRP, g_B103_CHAN_INTRP);
  g_fsd_cfg[1] = 0x4000f;
  g_fsd_cfg[2] = HW(B103_IIR_LPF, g_B103_IIR_LPF);
  g_fsd_cfg[3] = 0xa0003;
  g_fsd_cfg[4] = 0x80001;
  g_fsd_cfg[5] = 0xa00006;
  g_fsd_cfg[6] = 0;

  g_mtd_cfg[0] = HW(MTDb103_COEF, g_MTDb103_COEF);
  g_mtd_cfg[1] = 0x36660002;
  g_mtd_cfg[2] = 2;

  for (i = 0; i < 9; i++) g_tone_cfg[i] = g_FPM_TONE_CFG_raw[i];
  g_tone_cfg[4] = HW(ToneLPF, g_ToneLPF);
}

static void dump_shorts(const char *label, const short *buf, int n)
{
  int i;
  printf("%s:", label);
  for (i = 0; i < n; i++) printf(" %d", buf[i]);
  printf("\n");
}

/* deterministic synthetic "audio" - no rand(), so both binaries see the
 * exact same bytes without depending on libc RNG behavior */
static void gen_wave(short *buf, int n, int amp, int period, int seed)
{
  int i;
  for (i = 0; i < n; i++) {
    int t = i % period;
    int tri = t < period / 2 ? t : period - t;
    int v = (tri * amp * 2) / period - amp / 2;
    v += ((i * 131 + seed * 977) % 61) - 30; /* deterministic dither */
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    buf[i] = (short)v;
  }
}

static void test_phasor(void)
{
  short ph[4];
  int i;
  /*
   * Phase is only ever driven non-negative (0..0x7fff) by real callers -
   * FPM_TONE_create seeds it from `freq*0x8312+0x1000>>0xd` (always >=0 for
   * a positive freq) and every generator wraps it back into 0..0x7fff after
   * each call (`if (0x7fff<iVar4) iVar4 += -0x8000`). A negative phase was
   * tried here first and diverged from the real binary - not a port bug,
   * just an invalid/never-exercised input: the `uVar5 = iVar4 >> 0xd`
   * quadrant index relies on iVar4 >= 0 to stay in the small 0..3 range;
   * for negative iVar4 it becomes a huge unsigned value indexing wildly out
   * of bounds, which is undefined behavior in C and free to differ from
   * whatever the original binary's raw pointer arithmetic happened to hit.
   */
  printf("== FPM_phasor / FPM_phasor_demod ==\n");
  for (i = 0; i < 8; i++) {
    ph[0] = (short)(i * 4000);
    ph[1] = 0; ph[2] = 0; ph[3] = 2000;
    FPM_phasor(ph);
    printf("phasor[%d] -> %d %d %d %d\n", i, ph[0], ph[1], ph[2], ph[3]);
  }
  for (i = 0; i < 8; i++) {
    ph[0] = (short)(i * 4000);
    ph[1] = 0; ph[2] = 0; ph[3] = 2000;
    FPM_phasor_demod(ph);
    printf("phasor_demod[%d] -> %d %d %d %d\n", i, ph[0], ph[1], ph[2], ph[3]);
  }
}

static void test_sqrt_div(void)
{
  static const unsigned int sqrt_inputs[] = {
      0, 1, 2, 3, 100, 1000, 65535, 0x10000, 0x12345678, 0x7fffffff, 0xffffffff, 0x20000000, 0x1fffffff};
  static const unsigned short div_inputs[] = {1, 2, 3, 100, 1000, 32767, 32768, 40000, 65535, 0x8000, 0x7fff};
  unsigned int i;
  printf("== FPM_sqrt_dp ==\n");
  for (i = 0; i < sizeof(sqrt_inputs) / sizeof(sqrt_inputs[0]); i++) {
    printf("sqrt_dp(0x%x) = %u\n", sqrt_inputs[i], FPM_sqrt_dp(sqrt_inputs[i]));
  }
  printf("== FPM_div ==\n");
  for (i = 0; i < sizeof(div_inputs) / sizeof(div_inputs[0]); i++) {
    short mant = -1, shift = -1;
    int rc = FPM_div(div_inputs[i], &mant, &shift);
    printf("div(%u) = rc=%d mant=%d shift=%d\n", div_inputs[i], rc, mant, shift);
  }
  {
    short mant = -1, shift = -1;
    int rc = FPM_div(0, &mant, &shift);
    printf("div(0) = rc=%d mant=%d shift=%d\n", rc, mant, shift);
  }
}

static void test_rms(void)
{
  short buf[64];
  printf("== FPM_rms ==\n");
  gen_wave(buf, 64, 8000, 33, 1);
  printf("rms(64,tri) = %d\n", FPM_rms(buf, 64));
  gen_wave(buf, 20, 200, 7, 2);
  printf("rms(20,small) = %d\n", FPM_rms(buf, 20));
  memset(buf, 0, sizeof(buf));
  printf("rms(64,zero) = %d\n", FPM_rms(buf, 64));
}

static void test_iir(void)
{
  short state[32];
  short data[40];
  int i;
  printf("== FPM_iir_filt ==\n");
  memset(state, 0, sizeof(state));
  gen_wave(data, 40, 12000, 17, 3);
  for (i = 0; i < 40; i++) {
    short seed = (short)(data[i] >> 2);
    short out = FPM_iir_filt(seed, g_B103_IIR_LPF, state, 3);
    printf("iir_filt[%d] seed=%d -> %d\n", i, seed, out);
  }
  printf("== FPM_iir_filt_II ==\n");
  memset(state, 0, sizeof(state));
  gen_wave(data, 40, 9000, 23, 4);
  for (i = 0; i < 40; i++) {
    short one = data[i];
    FPM_iir_filt_II(&one, g_B103_IIR_LPF, state, 3, 1);
    printf("iir_filt_II[%d] in=%d -> %d\n", i, data[i], one);
  }
}

static void test_mrf(void)
{
  FpmMrf obj;
  int cfg[4];
  short in[300];
  short out[64];
  int i;
  printf("== FPM_MRF_init/filter (RX-shaped cfg) ==\n");
  memset(&obj, 0, sizeof(obj));
  cfg[0] = 0xa0003;
  cfg[1] = HW(B103_MRF_FILT_RX, g_B103_MRF_FILT_RX);
  cfg[2] = 0x5a;
  cfg[3] = 0;
  FPM_MRF_init(&obj, cfg, 1);
  gen_wave(in, 300, 15000, 29, 5);
  {
    unsigned int n = FPM_MRF_filter(&obj, in, out, 300);
    printf("mrf_filter produced %u samples\n", n);
    dump_shorts("mrf_out", out, (int)n);
  }
}

static void test_agc(void)
{
  FpmAgc obj;
  short data[48];
  printf("== FPM_AGC_init/agc ==\n");
  memset(&obj, 0, sizeof(obj));
  FPM_AGC_init(&obj, g_agcb103_cfg, 1);
  gen_wave(data, 48, 500, 13, 6);
  dump_shorts("agc_in", data, 48);
  FPM_AGC_agc(&obj, data, 48);
  dump_shorts("agc_out", data, 48);

  memset(&obj, 0, sizeof(obj));
  FPM_AGC_init(&obj, g_agcb103_cfg, 1);
  gen_wave(data, 48, 20000, 41, 7);
  dump_shorts("agc_in2", data, 48);
  FPM_AGC_agc(&obj, data, 48);
  dump_shorts("agc_out2", data, 48);
}

/*
 * N02: FPM_AGC_agc state sweep.  The first two calls above start from the init state (gain history 0 / reciprocal state 0) and never reach the
 * "FPM_div reciprocal" branch with a power-of-two gain (FPM_div's mantissa output is 0x8000 there, which the blob reads with movzx (unsigned) and
 * the first port version sign-extended; found by the N01/N02 V.32bis replay).  Sweep the gain history (obj short 0x10) over every power of two,
 * 0x7fff, negative values and a pseudo-random set, with the freeze flag (obj word 0x28) clear and set, at three signal levels (so the
 * reciprocal branch, the mute branch and both clamps are taken).
 */
static void test_agc_sweep(void)
{
  static const int amps[3] = { 300, 3000, 25000 };
  FpmAgc obj;
  short data[72];
  unsigned seed = 12345;
  int gi, a, fr, i, ncase = 0;
  /* H01: the debug-level >= 2 prints of FPM_AGC_init / Freeze / Release ("AGC_Release", the dropped-argument tail call "AGC %s %s", "AGC_Freeze") */
  {
    FpmAgc dobj;
    memset(&dobj, 0, sizeof dobj);
    dsplibs_debug_level = 2;
    printf("== FPM_AGC debug prints ==\n");
    FPM_AGC_init(&dobj, g_agcb103_cfg, 1);
    FPM_AGC_Freeze(&dobj);
    FPM_AGC_Release(&dobj);
    FPM_AGC_init(&dobj, g_agcb103_cfg, 0);
    dsplibs_debug_level = 0;
    printf("debug obj: %08x %08x %08x %08x\n", dobj.active, dobj.detect, (unsigned)(unsigned short)dobj.hist | ((unsigned)(unsigned short)dobj.s22 << 16), dobj.freeze);
  }
  printf("== FPM_AGC_agc state sweep ==\n");
  for (gi = 0; gi < 80; gi++) {
    int g;
    if (gi < 16) g = 1 << gi;				/* 1 .. 0x8000 (the last one wraps to -32768 as a short) */
    else if (gi < 32) g = -(1 << (gi - 16));
    else if (gi < 36) g = (gi == 32) ? 0x7fff : (gi == 33) ? 0x7ffe : (gi == 34) ? 0x4001 : 0x3fff;
    else { seed = seed * 1103515245u + 12345u; g = (int)((seed >> 8) & 0xffff); }
    for (a = 0; a < 3; a++) {
      for (fr = 0; fr < 2; fr++) {
        unsigned int sum = 0;	/* U01: 32 bit on both (unsigned long is 64 bit on x86-64) */
        memset(&obj, 0, sizeof(obj));
        FPM_AGC_init(&obj, g_agcb103_cfg, 1);
        obj.hist = (short)g;			/* gain history */
        obj.rcp = (short)(gi * 37);		/* previous reciprocal gain */
        obj.shift = (short)(gi % 5);		/* previous shift */
        obj.freeze = fr;				/* freeze flag (byte 0x28) */
        gen_wave(data, 72, amps[a], 13 + gi % 9, 3 + gi);
        FPM_AGC_agc(&obj, data, 72);
        for (i = 0; i < 72; i++) sum = sum * 31u + (unsigned short)data[i];
        printf("agc_sweep g=%d a=%d fr=%d: hist=%d rcp=%d shift=%d flag=%d sum=%08lx\n", g, a, fr, obj.hist, obj.rcp, obj.shift, obj.detect, (unsigned long)sum);
        ncase++;
      }
    }
  }
  printf("agc_sweep cases %d\n", ncase);
}

static void test_tone(void)
{
  FpmTone *tone;
  short out[32];
  short data[64];
  int i, r;
  printf("== FPM_TONE_create/generate/detect ==\n");
  tone = FPM_TONE_create(0, g_tone_cfg);
  FPM_TONE_set_freq(tone, 0x8b1);
  FPM_TONE_set_scale(tone, 0x7c00);
  r = FPM_TONE_generate(tone, out, 32);
  printf("tone_generate rc=%d\n", r);
  dump_shorts("tone_gen", out, 32);

  r = FPM_TONE_generate_demod(tone, out, 32);
  printf("tone_generate_demod rc=%d\n", r);
  dump_shorts("tone_gen_demod", out, 32);

  gen_wave(data, 64, 9000, 19, 8);
  for (i = 0; i < 4; i++) {
    int det = FPM_TONE_detect(tone, data + i * 16, 16);
    printf("tone_detect[%d] = %d\n", i, det);
  }
  FPM_TONE_delete(tone);
}

static void test_fsm(void)
{
  FpmFsm obj;
  unsigned short bits[8] = {0, 1, 0, 1, 1, 0, 0, 1};
  short out[64];
  short rc;
  printf("== FPM_FSM_init/modulate ==\n");
  memset(&obj, 0, sizeof(obj));
  {
    int fsm_cfg[2];
    fsm_cfg[0] = 0x042e04f6; /* packed mark/space freq words, Bell103 Originate-shaped */
    fsm_cfg[1] = 0x00180000;
    FPM_FSM_init(&obj, fsm_cfg);
  }
  rc = FPM_FSM_modulate(&obj, bits, out, 8);
  printf("fsm_modulate rc=%d\n", rc);
  dump_shorts("fsm_out", out, (int)rc);
  FPM_FSM_delete(&obj);
}

static void test_fsd(void)
{
  FpmFsd obj;
  short in[200];
  short out[32];
  unsigned int n;
  printf("== FPM_FSD_init/demodulate ==\n");
  memset(&obj, 0, sizeof(obj));
  FPM_FSD_init(&obj, g_fsd_cfg, 1);
  gen_wave(in, 200, 11000, 21, 9);
  n = FPM_FSD_demodulate(&obj, (unsigned short *)in, out, 200);
  printf("fsd_demodulate produced %u bits\n", n);
  dump_shorts("fsd_bits", out, (int)n);
  FPM_FSD_free(&obj);
}

static void test_mtd(void)
{
  FpmMtd *obj;
  short data[80];
  unsigned int r1, r2;
  printf("== FPM_MTD_create/detect ==\n");
  obj = FPM_MTD_create(0, g_mtd_cfg);
  gen_wave(data, 80, 6000, 15, 10);
  r1 = FPM_MTD_detect(obj, data, 80);
  printf("mtd_detect[1] = %u\n", r1);
  gen_wave(data, 80, 100, 15, 11);
  r2 = FPM_MTD_detect(obj, data, 80);
  printf("mtd_detect[2] = %u\n", r2);
  FPM_MTD_delete(obj);
}

int main(void)
{
  init_fixtures();
  test_phasor();
  test_sqrt_div();
  test_rms();
  test_iir();
  test_mrf();
  test_agc();
  test_agc_sweep();
  test_tone();
  test_fsm();
  test_fsd();
  test_mtd();
  return 0;
}
