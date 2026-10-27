/*
 * Differential test harness for the hand-cleaned subset of V.8's own
 * primitive toolkit (src/v8/v8.c) - see that file's header comment for
 * exactly what is and is not covered (the handshake FSM itself and the
 * object-lifecycle/dp_operations glue are explicitly NOT ported yet, so
 * this harness calls every function directly with synthetic inputs, the
 * same pattern as test/fpm_diff_main.c and test/rd_diff_main.c).
 *
 * Same pattern as fpm_diff_main.c: this ONE source is compiled into two
 * binaries - v8_test_blob (linked against the real dsplibs.o, whose
 * V.8 functions are globally exported per `nm dsplibs.o`) and v8_test_port
 * (linked against ../src/v8/v8.c + ../src/v8/v8_tables.c). The one coefficient
 * value this harness must supply itself (v8_c2100, passed as an explicit
 * argument to v8_detectorinit rather than referenced internally) is kept as
 * a private `static const` copy here - NOT included from src/v8/v8_tables.c -
 * for the same reason fpm_diff_main.c's fixtures are private copies: several
 * table names in dsplibs.o are LOCAL (not exported), so there is no actual
 * link conflict here, but keeping the harness self-contained and symmetric
 * between the two build targets is the established convention.
 *
 * IMPORTANT: several V.8 functions store/consume RAW POINTERS into fields
 * of the same object (ring-buffer read/write positions, etc.). This harness
 * never prints a raw pointer value - only prints ordinary sample/state
 * VALUES read back from fixed, known byte offsets - since pointer values
 * are heap/stack addresses that differ between the two separately-linked
 * binaries and are never meaningful to diff directly.
 */

#include <stdio.h>
#include <string.h>
#include "v8_hobj.h"

/* ---- V.8 toolkit entry points under test (same signatures for both link targets) ---- */
extern int v8_absfn(short param_1);
extern int v8_mpyint(short param_1, short param_2);
extern unsigned char charFlip(unsigned char param_1);
extern void v8_copycoeff(short *param_1, const short *param_2, short param_3);

extern unsigned int v8_getbit(void *param_1);
extern unsigned int V8SetMessage(void *param_1, int param_2, const void *param_3, int param_4);
extern unsigned int V8GetMessage(void *param_1, void *param_2, int *param_3);

extern unsigned int v8_rxinit(void *param_1);
extern unsigned int v8_txinit(void *param_1);
extern unsigned int v8_rxreadqueue(void *param_1);
extern unsigned int v8_txwritequeue(void *param_1);

extern void checkSignalStability(void *param_1);
extern unsigned int V8agc(void *param_1);
extern void v8_detectorinit(void *param_1, void *param_2, const void *param_3, short param_4,
                             short param_5, short param_6, short param_7, short param_8);
extern unsigned int v8_tone_detect(void *param_1, void *param_2, short *param_3);
extern void v8_dftupdate(void *param_1, short param_2, short *param_3, short param_4);
extern void v8_dftenergy(void *param_1, short param_2, unsigned char param_3);
extern void v8_phase_rev_init(void *param_1);
extern void v8_phase_rev_detect(void *param_1, const void *param_2, short param_3);
extern void v8_ansaminit(void *param_1);
extern void v8_ansamgenerate(void *param_1, short *param_2);
extern void v8_TONEq_init(void *param_1);
extern void v8_TONEq_generate(void *param_1, short *param_2);
extern short v8_cosread(unsigned char param_1);

extern void v8_V21_Init(void *param_1, short param_2, short param_3);
extern void V8_V21_reset(void *param_1);
extern void v8_fskmodulate(void *param_1, short param_2);
extern short v8_fsktxfilter(void *param_1, short param_2);
extern void v8_fskdemodulate(void *param_1);

/* private copy of the ANSam/CI DFT detector's coefficient block (real ELF
 * symbol `c2100`, 0x5670, LOCAL) - see src/v8/v8_tables.c */
static const short g_c2100[8] = {0, 0, 0, 0, -6608, 15416, -5792, 15416};

/* Big enough for every byte offset (up to ~0xeb8) any tested function
 * touches on the "full V.8 object" - real object is sysdep_malloc(0xec4). */
#define V8OBJ_SIZE 0xee0

static void dump_shorts(const char *label, const short *buf, int n)
{
  int i;
  printf("%s:", label);
  for (i = 0; i < n; i++) printf(" %d", buf[i]);
  printf("\n");
}

static void gen_wave(short *buf, int n, int amp, int period, int seed)
{
  int i;
  for (i = 0; i < n; i++) {
    int t = i % period;
    int tri = t < period / 2 ? t : period - t;
    int v = (tri * amp * 2) / period - amp / 2;
    v += ((i * 131 + seed * 977) % 61) - 30;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    buf[i] = (short)v;
  }
}

static void test_math_helpers(void)
{
  static const short vals[] = {0, 1, -1, 100, -100, 32767, -32768, 12345, -12345};
  unsigned int i, j;
  printf("== v8_absfn/v8_mpyint/charFlip ==\n");
  for (i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
    printf("absfn(%d) = %d\n", vals[i], v8_absfn(vals[i]));
  }
  for (i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
    for (j = 0; j < sizeof(vals) / sizeof(vals[0]); j++) {
      printf("mpyint(%d,%d) = %d\n", vals[i], vals[j], v8_mpyint(vals[i], vals[j]));
    }
  }
  for (i = 0; i < 256; i += 7) {
    printf("charFlip(%u) = %u\n", i, (unsigned int)charFlip((unsigned char)i));
  }
  {
    short dst[10];
    static const short src[10] = {1, -2, 3, -4, 5, -6, 7, -8, 9, -10};
    v8_copycoeff(dst, src, 10);
    dump_shorts("copycoeff", dst, 10);
  }
}

static void test_message(void)
{
  unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  unsigned char decoded[32];
  static const unsigned char msg1[] = "TESTMSG";
  static const unsigned char msg2[] = "\x01\x23\x45\x67\x89\xab\xcd\xef\x10\x20\x30\x40\x50\x60\x70\x80\x90";
  int i, cap;
  unsigned int rc;

  printf("== V8SetMessage/V8GetMessage/v8_getbit (side=0 / type=1 -> +0xcd4) ==\n");
  memset(obj, 0, sizeof(obj)); /* +0xa44 side == 0 (caller) by zero-default */
  rc = V8SetMessage(obj, 1, msg1, (int)(sizeof(msg1) - 1));
  printf("SetMessage rc=%u\n", rc);
  {
    void *slot = obj + 0xcd4;
    printf("getbit stream:");
    for (i = 0; i < 120; i++) {
      unsigned int bit = v8_getbit(slot);
      printf(" %u", bit);
    }
    printf("\n");
    printf("crc after stream = 0x%x\n", *(unsigned short *)((char *)slot + 0x1e));
    printf("repeat count = %d\n", *(short *)((char *)slot + 0x2c));
  }
  cap = (int)sizeof(decoded);
  memset(decoded, 0xAA, sizeof(decoded));
  rc = V8GetMessage(obj, decoded, &cap);
  printf("GetMessage rc=%u cap=%d bytes:", rc, cap);
  for (i = 0; i < cap && i < (int)sizeof(decoded); i++) printf(" %02x", decoded[i]);
  printf("\n");

  printf("== V8SetMessage/V8GetMessage (side=1 / type=0 -> +0xc54) ==\n");
  memset(obj, 0, sizeof(obj));
  *(int *)(obj + 0xa44) = 1; /* answer side */
  rc = V8SetMessage(obj, 0, msg2, (int)sizeof(msg2));
  printf("SetMessage(len=%d, truncated) rc=%u\n", (int)sizeof(msg2), rc);
  {
    void *slot = obj + 0xc54;
    printf("getbit stream:");
    for (i = 0; i < 200; i++) {
      unsigned int bit = v8_getbit(slot);
      printf(" %u", bit);
    }
    printf("\n");
  }
  cap = (int)sizeof(decoded);
  memset(decoded, 0xAA, sizeof(decoded));
  rc = V8GetMessage(obj, decoded, &cap);
  printf("GetMessage rc=%u cap=%d bytes:", rc, cap);
  for (i = 0; i < cap && i < (int)sizeof(decoded); i++) printf(" %02x", decoded[i]);
  printf("\n");

  printf("== V8SetMessage error paths ==\n");
  memset(obj, 0, sizeof(obj));
  printf("SetMessage(badtype=9) rc=%u\n", V8SetMessage(obj, 9, msg1, 3));
  printf("SetMessage(len=0) rc=%u\n", V8SetMessage(obj, 1, msg1, 0));
}

static void test_ring_buffers(void)
{
  unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  int i;

  printf("== v8_rxinit/v8_txinit/v8_rxreadqueue/v8_txwritequeue ==\n");
  memset(obj, 0, sizeof(obj));
  v8_rxinit(obj);
  v8_txinit(obj);
  /* seed the rxreadqueue source ring (+0x11c, 0x80 shorts) with a wave */
  {
    short wave[0x80];
    gen_wave(wave, 0x80, 9000, 19, 5);
    memcpy(obj + 0x11c, wave, sizeof(wave));
  }
  for (i = 0; i < 40; i++) {
    v8_rxreadqueue(obj);
    if (i % 8 == 0) {
      dump_shorts("rxreadqueue_out", (short *)(obj + 0x5c8), 4);
    }
  }
  /* seed the txwritequeue source (+0x5c0, 4 shorts) each call, dump a fixed
   * slice of its destination ring (+0x228) */
  for (i = 0; i < 40; i++) {
    short one[4];
    one[0] = (short)(i * 37 - 500);
    one[1] = (short)(i * -19 + 42);
    one[2] = (short)(i * 5);
    one[3] = (short)(-i);
    memcpy(obj + 0x5c0, one, sizeof(one));
    v8_txwritequeue(obj);
    if (i % 8 == 0) {
      dump_shorts("txwritequeue_ring_head", (short *)(obj + 0x228), 8);
    }
  }
}

static void test_agc(void)
{
  unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  short wave[0x80];
  int side, i;

  for (side = 0; side < 2; side++) {
    printf("== V8agc (side=%d) ==\n", side);
    memset(obj, 0, sizeof(obj));
    v8_rxinit(obj);
    v8_txinit(obj);
    *(int *)(obj + 0xa44) = side;
    gen_wave(wave, 0x80, 6000, 23, side + 11);
    memcpy(obj + 0x11c, wave, sizeof(wave));
    for (i = 0; i < 20; i++) {
      V8agc(obj);
      dump_shorts("agc_out", (short *)(obj + 0x5c8), 4);
      printf("agc gain=%d level=%d energy=%d\n", *(short *)(obj + 0x38),
             *(short *)(obj + 0x36), *(int *)(obj + 0x30));
    }
  }
}

static void test_signal_stability(void)
{
  unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  int i;
  printf("== checkSignalStability ==\n");
  memset(obj, 0, sizeof(obj));
  *(short *)(obj + 0xa2) = 0x200;
  for (i = 0; i < 260; i++) {
    *(short *)(obj + 0x38) = (short)(0x200 + (i * 3) % 40 - 20);
    checkSignalStability(obj);
    if (i % 20 == 0) {
      printf("stability[%d] a0=%d a4=%d a6=%d\n", i, *(short *)(obj + 0xa0),
             *(short *)(obj + 0xa4), *(short *)(obj + 0xa6));
    }
  }
}

static void test_detector(void)
{
  unsigned char obj[V8H_BUFSZ(0x40)] V8H_ALIGN;
  unsigned char detector[0x40];
  short samples[40];
  int i;

  printf("== v8_detectorinit/v8_tone_detect ==\n");
  memset(obj, 0, sizeof(obj));
  memset(detector, 0, sizeof(detector));
  v8_detectorinit(obj, detector, g_c2100, 0, 100, 0x32, 0x5dc, 0);
  printf("obj flags after init = 0x%x\n", *(unsigned short *)(obj + 0x26));

  gen_wave(samples, 40, 4000, 13, 3);
  for (i = 0; i < 40; i += 8) {
    short *start = samples + i;
    short *end = samples + i + 8;
    V8_PS(short *, obj, 0x2c, end);
    unsigned int rc = v8_tone_detect(obj, detector, start);
    printf("tone_detect[%d] rc=%u levelAvg=%d confirmCnt=%d\n", i, rc,
           *(short *)(detector + 0x12), *(short *)(detector + 8));
  }
  dump_shorts("tone_detect_samples_after", samples, 40);
}

static void test_dft(void)
{
  unsigned short slot[8];
  short samples[24];
  int i;

  printf("== v8_dftupdate/v8_dftenergy ==\n");
  memset(slot, 0, sizeof(slot));
  gen_wave(samples, 24, 3000, 7, 9);
  for (i = 0; i < 24; i += 4) {
    v8_dftupdate(slot, 1, samples + i, 4);
    printf("dft slot after %d: ring=%u accumI=%d accumQ=%d\n", i + 4, slot[0],
           *(int *)(slot + 2), *(int *)(slot + 4));
  }
  v8_dftenergy(slot, 1, 1);
  printf("dft energy = %d\n", *(short *)((char *)slot + 0xc));
}

static void test_phase_rev(void)
{
  unsigned char obj[0x100];
  short samples[48];
  int i;

  printf("== v8_phase_rev_init/v8_phase_rev_detect ==\n");
  memset(obj, 0xCC, sizeof(obj));
  v8_phase_rev_init(obj);
  gen_wave(samples, 48, 2500, 17, 21);
  for (i = 0; i < 48; i += 8) {
    v8_phase_rev_detect(obj, samples + i, 8);
    printf("phase_rev[%d] flag=%d acc0=%d acc1=%d acc2=%d ctr=%d\n", i,
           *(unsigned short *)(obj + 0xdc), *(int *)obj, *(int *)(obj + 4),
           *(int *)(obj + 8), *(short *)(obj + 0xc));
  }
}

static void test_ansam_tonesq(void)
{
  unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  short out[4];
  int i;

  printf("== v8_ansaminit/v8_ansamgenerate ==\n");
  memset(obj, 0, sizeof(obj));
  *(short *)(obj + 0xa42) = 0x4000;
  v8_V21_Init(obj, 0, 0);
  v8_ansaminit(obj);
  for (i = 0; i < 10; i++) {
    v8_ansamgenerate(obj, out);
    dump_shorts("ansam_out", out, 4);
  }

  printf("== v8_TONEq_init/v8_TONEq_generate ==\n");
  memset(obj, 0, sizeof(obj));
  *(short *)(obj + 0xa42) = 0x4000;
  v8_V21_Init(obj, 0, 0);
  v8_TONEq_init(obj);
  for (i = 0; i < 10; i++) {
    v8_TONEq_generate(obj, out);
    dump_shorts("toneq_out", out, 4);
  }

  printf("== v8_cosread ==\n");
  for (i = 0; i < 256; i += 5) {
    printf("cosread(%d) = %d\n", i, v8_cosread((unsigned char)i));
  }
}

static void test_fsk(void)
{
  unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  int band, filt, i;

  for (band = 0; band < 2; band++) {
    for (filt = 0; filt < 2; filt++) {
      printf("== v8_V21_Init/v8_fskmodulate/v8_fsktxfilter (band=%d filt=%d) ==\n", band, filt);
      memset(obj, 0, sizeof(obj));
      v8_txinit(obj); /* v8_fskmodulate tail-calls v8_txwritequeue, which needs +0x224 set up */
      *(short *)(obj + 0xa42) = 0x4000;
      v8_V21_Init(obj, (short)band, (short)filt);
      for (i = 0; i < 12; i++) {
        v8_fskmodulate(obj, (short)(i & 1));
        dump_shorts("fskmod_out", (short *)(obj + 0x5c0), 4);
      }
    }
  }

  printf("== v8_fskdemodulate ==\n");
  memset(obj, 0, sizeof(obj));
  *(short *)(obj + 0xa42) = 0x4000;
  v8_V21_Init(obj, 0, 0);
  for (i = 0; i < 30; i++) {
    short wave[4];
    gen_wave(wave, 4, 5000, 9, i + 30);
    memcpy(obj + 0x5c8, wave, sizeof(wave));
    v8_fskdemodulate(obj);
    if (i % 5 == 0) {
      printf("fskdemod[%d] bitcnt=%d shiftreg=0x%x\n", i, *(short *)(obj + 0xc38),
             *(unsigned short *)(obj + 0xc3a));
    }
  }
}

int main(void)
{
  V8H_REG(v8_c2100, g_c2100); /* x86-64: v8_detectorinit stores the table as a CFGP handle; resolve it to this private copy */
  test_math_helpers();
  test_message();
  test_ring_buffers();
  test_agc();
  test_signal_stability();
  test_detector();
  test_dft();
  test_phase_rev();
  test_ansam_tonesq();
  test_fsk();
  return 0;
}
