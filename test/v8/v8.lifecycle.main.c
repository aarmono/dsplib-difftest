/*
 * True end-to-end differential test for V.8's object-lifecycle layer -
 * V8Create/V8Process/V8Control/V8UpdateModemParameters/V8Delete - the last
 * layer needed to drive a real V8 object through its actual public API,
 * rather than poking v8handshak/its state fields directly the way every
 * earlier V.8 test in this project does (test/v8/v8.main.main.c,
 * test/v8/v8.init.main.c, test/v8/v8.fsm.main.c, test/v8/v8.jm.main.c).
 *
 * Same one-source-two-binaries pattern as every other test here: this file
 * is compiled once against the real dsplibs.o (V8Create/V8Process/
 * V8Control/V8UpdateModemParameters/V8Delete are all globally exported
 * symbols per `nm ../dsplibs.o`) and once against ../src/v8/v8.c +
 * ../src/v8/v8_tables.c.
 *
 * Unlike the FSM-only tests, this harness does NOT need to manually poke
 * `+0x21c` (the TX-sample-count field v8handshak's own outer loop checks) -
 * V8Process itself decrements it by 1 per input sample consumed, while
 * v8handshak's own TX generation (tx-phase 5's silence / tx-phase 6's CM
 * tone / etc, via v8_fsktxfilter -> v8_txwritequeue) increments it by 4 per
 * output sample produced. Driving through the real V8Process entry point
 * lets these two already-verified mechanisms interact exactly as they do in
 * production - the "+0x21c gotcha" documented in NOTES.md for the
 * poke-based FSM tests was a test-harness artifact of NOT going through
 * this layer, not a real system property.
 *
 * V8OBJ_SIZE is set to EXACTLY 0xec4 (V8Create's own `sysdep_malloc(0xec4)`
 * size) - unlike test/v8/v8.init.main.c's 0xee0-with-margin convention,
 * which dumped the harness's OWN oversized stack buffer, `obj` here is the
 * real, exactly-sized heap allocation V8Create returns, so dumping past
 * 0xec4 bytes would be a real heap overread.
 *
 * Scenarios drive an all-zero (silence) RX signal for 2000 8-sample
 * V8Process calls (16000 samples) - deliberately never forming a real
 * ANSam/CM/JM sync word (same "sustained silence never triggers a false
 * detection" reasoning already established for B103/V23/V8's other tests),
 * so both the answer-side (ANSam-wait) and caller-side (CM-tone-generation
 * then rx-phase-0x28 preamble) scenarios stay inside states this project has
 * already individually verified via test/v8/v8.fsm.main.c - but now
 * exercised together, through the real object-lifecycle entry points, for
 * the first time. This is an honest, deliberately bounded scope: 2000 calls
 * is enough to exercise V8Create/V8Process/the real +0x21c dynamics/
 * V8Control/V8UpdateModemParameters/V8Delete working together, but is NOT
 * enough to reach either side's full signal-detect timeout (that would take
 * on the order of the FSM-only tests' 2400-tick thresholds, translated
 * through V8Process's variable ticks-per-sample ratio into tens of
 * thousands of samples) - seeing the steady-state (non-terminal) portion of
 * both sides' post-init behavior match bit-exact is what this test proves.
 */

#include <stdio.h>
#include <string.h>
#include "v8_hobj.h"

#if defined(__x86_64__)
extern void *V8Create(const V8Cfg *param_1);
#else
extern void *V8Create(const unsigned int *param_1);
#endif
extern void V8Delete(void *param_1);
extern unsigned int V8Process(void *param_1, const short *param_2, short *param_3, int param_4);
extern unsigned int V8Control(void *param_1, int param_2);
extern unsigned int V8UpdateModemParameters(void *param_1, unsigned char *param_2);

#define V8OBJ_SIZE 0xec4

/* Same two pointer-field exclusion lists as test/v8/v8.init.main.c (see
 * its header comment for the full rationale) - V8Process/V8Control/
 * V8UpdateModemParameters introduce no NEW pointer field beyond what
 * v8handshakinit/v8_rxinit/v8_txinit already write (confirmed by reading
 * their raw decompiles: V8Process only touches the already-listed
 * +0x220/+0x118 ring cursors; V8Control and V8UpdateModemParameters write
 * no pointer fields of their own). */
static const int ptr_field_offsets[] = {0x2c, 0x114, 0x118, 0x220, 0x224, 0xc48, 0xc4c, 0xc50};
static const int external_ptr_field_offsets[] = {0xa58, 0xad8, 0xdd8, 0xddc, 0xde0, 0xde4};

static void print_ptr_field(const char *label, unsigned char *obj, int off)
{
  char *val;
  V8H_PGET(val, obj, off);
  if (val >= (char *)obj && val < (char *)obj + V8OBJ_SIZE) {
    printf("%s=off:0x%x\n", label, (int)(val - (char *)obj));
  } else {
    unsigned int raw;
    V8H_PRAW(raw, obj, off);
    printf("%s=raw:0x%08x\n", label, raw);
  }
}

static void dump_obj(const char *tag, unsigned char *obj)
{
  unsigned char scratch[V8OBJ_SIZE];
  unsigned int i;
  memcpy(scratch, obj, V8OBJ_SIZE);
  for (i = 0; i < sizeof(external_ptr_field_offsets) / sizeof(external_ptr_field_offsets[0]); i++) {
    memset(scratch + external_ptr_field_offsets[i], 0, 4);
  }
  for (i = 0; i < sizeof(ptr_field_offsets) / sizeof(ptr_field_offsets[0]); i++) {
    memset(scratch + ptr_field_offsets[i], 0, 4);
  }
  printf("%s bytes:", tag);
  for (i = 0; i < V8OBJ_SIZE; i++) printf("%02x", scratch[i]);
  printf("\n");
  for (i = 0; i < sizeof(ptr_field_offsets) / sizeof(ptr_field_offsets[0]); i++) {
    char label[32];
    sprintf(label, "%s.%x", tag, ptr_field_offsets[i]);
    print_ptr_field(label, obj, ptr_field_offsets[i]);
  }
}

typedef struct {
  const char *label;
  int side, opMode, sigTO, msgTO;
  int tone; /* H01: 0 = silence; 1 = a 2000 Hz tone (8 kHz, period 4) with a little LCG noise on rx, so the detector / AGC state is not all zero */
  int dbg;  /* H01: dsplibs_debug_level for this scenario (2 prints "State changed from %s to %s" and the V8Control names, i.e. v8StatusName / v8ControlName) */
} scenario_t;

extern unsigned int dsplibs_debug_level;

static const scenario_t scenarios[] = {
  {"answer_silence", 0, 0, 1, 3, 0, 0},
  {"caller_silence", 1, 0, 1, 3, 0, 0},
  {"answer_tone", 0, 0, 1, 3, 1, 0},
  {"caller_tone", 1, 0, 1, 3, 1, 0},
  {"answer_tone_debug", 0, 0, 1, 3, 1, 2},
  {"caller_silence_debug", 1, 0, 1, 3, 0, 2},
};

int main(void)
{
  unsigned int si;
  for (si = 0; si < sizeof(scenarios) / sizeof(scenarios[0]); si++) {
    const scenario_t *s = &scenarios[si];
#if defined(__x86_64__)
    V8Cfg cfg; /* the capabilities word is a real pointer: src/v8/v8_obj.h */
#else
    unsigned int cfg[6];
#endif
    unsigned char caps[32];
    unsigned char *capsPtr = caps;
    void *obj;
    short rxBuf[8];
    short txBuf[8];
    int call;

    memset(caps, 0, sizeof(caps));
#if defined(__x86_64__)
    cfg.side = (unsigned int)s->side;
    cfg.opMode = (unsigned int)s->opMode;
    cfg.sigTimeout = (unsigned int)s->sigTO;
    cfg.msgTimeout = (unsigned int)s->msgTO;
    cfg.pad = 0;
    cfg.caps = capsPtr;
#else
    cfg[0] = (unsigned int)s->side;
    cfg[1] = (unsigned int)s->opMode;
    cfg[2] = (unsigned int)s->sigTO;
    cfg[3] = (unsigned int)s->msgTO;
    cfg[4] = 0;
    memcpy(&cfg[5], &capsPtr, sizeof(capsPtr));
#endif

    printf("== %s ==\n", s->label);
    dsplibs_debug_level = (unsigned int)s->dbg;
#if defined(__x86_64__)
    obj = V8Create(&cfg);
#else
    obj = V8Create(cfg);
#endif
    dump_obj("after_create", (unsigned char *)obj);

    memset(rxBuf, 0, sizeof(rxBuf)); /* silence, never RX-ready */
    for (call = 0; call < 2000; call++) {
      unsigned int ret;
      int i;
      if (s->tone) {
        static const short tone4[4] = {0, 6000, 0, -6000};
        static unsigned int lcg = 12345u;
        for (i = 0; i < 8; i++) {
          lcg = lcg * 1103515245u + 12345u;
          rxBuf[i] = (short)(tone4[i & 3] + (int)((lcg >> 16) & 0xff) - 128);
        }
      }
      memset(txBuf, 0x66, sizeof(txBuf)); /* poison, catch missed writes */
      ret = V8Process(obj, rxBuf, txBuf, 8);
      printf("call %d ret=%u tx=", call, ret);
      for (i = 0; i < 8; i++) printf("%04x", (unsigned short)txBuf[i]);
      printf("\n");
    }
    dump_obj("after_process_loop", (unsigned char *)obj);

    {
      unsigned int ctlRet = V8Control(obj, 1);
      printf("control1_ret=%u\n", ctlRet);
      if (s->dbg) {
        int t;
        for (t = 0; t < 4; t++) printf("control%d_ret=%u\n", t, V8Control(obj, t)); /* 3 = the unsupported-type message */
      }
      dump_obj("after_control", (unsigned char *)obj);
    }

    {
      unsigned char dpParam[0x20];
      unsigned int upRet;
      int i;
      memset(dpParam, 0x77, sizeof(dpParam));
      upRet = V8UpdateModemParameters(obj, dpParam);
      printf("update_ret=%u dp:", upRet);
      for (i = 0; i < (int)sizeof(dpParam); i++) printf("%02x", dpParam[i]);
      printf("\n");
    }

    V8Delete(obj);
    printf("deleted_ok\n");
  }
  return 0;
}
