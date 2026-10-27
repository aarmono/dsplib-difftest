/*
 * Differential test harness for V.32/V.32bis's top-level object lifecycle:
 * V32FP_create/V32FP_recreate/V32FP_delete/V32FP_control/V32FP_status/
 * V32FP_GetDiagnostics/V32FP_GetCleanedSamples (src/v32/v32_engine.c).
 *
 * Structured around these GLOBAL entry points directly (confirmed via `nm
 * ../dsplibs.o` - see NOTES.md) rather than v32_create/_delete/_process,
 * which are LOCAL symbols in dsplibs.o and only reachable via v32_ops/
 * dp_v32_init - the same "test the real public entry points" approach this
 * project's V.32 primitive/DSP toolkit chunk already established
 * (test/v32/v32_engine.primitives.main.c).
 *
 * V32FP_create's own parameter convention is reverse-engineered from
 * v32_create's own call site (porting/decompiled/dp/v32_state.c) - 6 ints: role,
 * look-ahead, timeout, margin, enable-word, tx-rate (as two duplicate
 * ushorts) - see src/v32/v32_engine.c's own header comment for the full field-by-
 * field derivation. This harness always passes enable=1 (bit0 set), the
 * SAME convention the real v32_create always uses - this is what makes
 * V32FP_recreate's "if role==0/1 and skip-bit clear, do the inline
 * handshake transition" branch PROVABLY unreachable (see src/v32/v32_engine.c's
 * header comment), so this test never touches the not-yet-hand-cleaned
 * NextState/Hdx handler layer (next chunk's scope).
 *
 * Both roles this test drives (Originate=0, Answer=nonzero) stay at rate
 * class 0 (4800bps, tx rate 0x12c0) - the only rate class that never calls
 * the out-of-scope VTBv32_init (see src/v32/v32_datapath.c's SetRxModeV32).
 * LocLoop (role 2) is NOT tested: V32FP_create's own public wrapper can
 * only ever encode role 0 or "not 0" (a boolean, per the derivation above),
 * so LocLoop is not reachable through this entry point at all in the real
 * object's own usage - out of this chunk's scope by construction, not an
 * oversight.
 *
 * ONE SCENARIO PER PROCESS (argv[1] selects which), same convention as
 * every other *_diff_main.c in this project (isolates each scenario's own
 * heap/state, sidesteps any cross-instance issue the real dsplibs.o might
 * not tolerate).
 *
 * ==========================================================================
 * UPDATE (this chunk): scenarios 2..11 - V32FP_control's own RngInit/
 * RngResp "reconnect" and "full reconnect" dispatch branches
 * ==========================================================================
 * V32OrgNextState/V32AnsNextState/V32RngInitNextState/V32RngRespNextState
 * are all real now (prior chunks), so V32FP_control's own two remaining
 * `v32_unported()` stubs - the RngInit/RngResp mid-call rate-renegotiation
 * dispatch (ctl options byte bit 0x8 set, bit 0x4 clear) and the "full
 * reconnect" dispatch (bit 0x4 set) - are hand-cleaned this chunk (see
 * src/v32/v32_engine.c's own V32FP_control comment for the full derivation). scenarios
 * 2-7 drive the RngInit branch (default/no-match plus all 4 named rate
 * values, including both sub-cases of the 9600bps class-1-vs-2 tie-break);
 * scenario 8 drives the RngResp branch (via a direct `hdxobj+0x76=5` poke -
 * role 5 is reachable in principle through this exact dispatch branch, but
 * poking it directly sidesteps needing a live rate-renegotiation handshake
 * just to reach it, the same "construct normally, then poke the specific
 * field this chunk's own dispatch layer would otherwise set" precedent
 * already used for LocLoop/RngInit/RngResp's own entry paths); scenarios
 * 9-10 drive the full-reconnect branch's two role bodies (Originate/Answer,
 * each for the first time exercising `V32FP_recreate`'s REINIT-IN-PLACE
 * path - `obj==cfgIn`, non-NULL - previously "ported but not exercised" per
 * NOTES.md); scenario 11 drives the full-reconnect branch's own
 * `bigbuf+0x50d8`-gated rate-override switch. All new scenarios stay at
 * rate class 0 (matching every other V.32 chunk's own VTBv32_init-avoidance
 * convention).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);
extern int V32FP_status(char *obj, short *statusOut);
extern int V32FP_control(char *obj, short *ctl);
extern void V32FP_GetDiagnostics(char *obj, int mode, int *outBuf, int maxCount);
extern char *V32FP_GetCleanedSamples(char *obj, int *outCount);

#define STATUS_WORDS 24

static void dump_status(const char *label, short *st)
{
  int i;
  printf("%s:", label);
  for (i = 0; i < STATUS_WORDS; i++) printf(" %d", (int)st[i]);
  printf("\n");
}

static void run_scenario(const char *label, int role)
{
  int cfg[6];
  char *obj;
  short status[STATUS_WORDS];
  unsigned char ctlBytes[40];
  int diag[8];
  int cleaned;
  void *p;

  printf("== %s ==\n", label);

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = role;          /* role selector: 0 = Originate, nonzero = Answer */
  cfg[1] = 100;           /* look-ahead */
  cfg[2] = 60000;         /* timeout */
  cfg[3] = 700;           /* margin */
  cfg[4] = 1;             /* enable word, bit0 set - see header comment */
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* tx rate 4800, both halves */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    return;
  }
  obj = (char *)(p);

  memset(status, 0, sizeof(status));
  V32FP_status(obj, status);
  dump_status("status after create", status);

  /* Plausible control code: options byte (ctl+0xd) has bits 0x4/0x8 BOTH
   * clear, so neither of V32FP_control's two NextState-calling branches is
   * reached (see src/v32/v32_engine.c's V32FP_control header comment) - this is a
   * real, deliberate scenario-design choice, not an accidental miss. The
   * ctl+10 word IS set nonzero, to also exercise the (NextState-free)
   * V32_SAMPLE_LEN-ratio field-update block. */
  memset(ctlBytes, 0, sizeof(ctlBytes));
  ctlBytes[12] = 0x03;             /* ctl[6] low byte */
  ctlBytes[13] = 0x00;             /* options byte: bits 0x4/0x8 clear */
  *(unsigned int *)(ctlBytes + 20) = 1;      /* ctl+10 (int): nonzero */
  *(unsigned int *)(ctlBytes + 24) = 0x1234; /* ctl+0xc */
  *(unsigned int *)(ctlBytes + 28) = 0x5678; /* ctl+0xe */

  {
    int ret = V32FP_control(obj, (short *)(void *)ctlBytes);
    printf("control ret=%d\n", ret);
  }

  memset(status, 0, sizeof(status));
  V32FP_status(obj, status);
  dump_status("status after control", status);

  memset(diag, 0, sizeof(diag));
  V32FP_GetDiagnostics(obj, 0, diag, 8);
  {
    int i;
    printf("diagnostics(mode=0):");
    for (i = 0; i < 8; i++) printf(" %d", diag[i]);
    printf("\n");
  }

  cleaned = -1;
  {
    /* V32FP_GetCleanedSamples's return value is a raw heap pointer (the
     * bigbuf+0x50d0 scratch buffer address) - legitimately different
     * between the blob-linked and port-linked processes (different heaps),
     * so only its null-ness is comparable, not its bit pattern - same
     * convention this project's other *_diff_main.c harnesses already use
     * for pointer-valued returns. */
    char *ret = V32FP_GetCleanedSamples(obj, &cleaned);
    printf("cleanedsamples nonnull=%d count=%d\n", ret != 0, cleaned);
  }

  V32FP_delete(obj);
  printf("== done ==\n");
}

struct scenario {
  const char *label;
  int role;
};

static const struct scenario SCENARIOS[] = {
    {"Originate (role=0)", 0},
    {"Answer (role=1)", 1},
};
#define NUM_PLAIN_SCENARIOS (int)(sizeof(SCENARIOS) / sizeof(SCENARIOS[0]))

/*
 * reconnect_scenario: drives V32FP_control's two now-hand-cleaned
 * NextState-calling branches (see this file's own header comment and
 * src/v32/v32_engine.c's V32FP_control comment for the full derivation). A fresh
 * object is built via V32FP_create for every scenario (matching this
 * harness's "one scenario per process" convention - no state carries over
 * between scenarios even when this struct array holds several).
 */
struct reconnect_scenario {
  const char *label;
  int role;                  /* obj+0 role bool for V32FP_create */
  unsigned char ctlOptions;  /* ctl+0xd: bit 0x4 = full-reconnect, bit 0x8 =
                               * RngInit/RngResp reconnect (only meaningful
                               * with bit 0x4 clear) */
  short ctlRate;             /* ctl[0] - RngInit/RngResp branch's negotiated
                               * rate word (unused by the full-reconnect
                               * branch) */
  unsigned int ctlWord10;    /* ctl+0x10 - RngInit branch's 9600bps
                               * class-1-vs-2 tie-breaker (unused otherwise) */
  int pokeRngRespRole;       /* poke hdxobj+0x76=5 before the control call,
                               * to reach the RngResp (responder) side of the
                               * reconnect branch */
  int pokeSnrFlag;           /* poke bigbuf+0x50d8=1 (+ pokeConnectRate into
                               * bigbuf+0x2e) before the control call, to
                               * reach the full-reconnect branch's own
                               * rate-override switch */
  short pokeConnectRate;
};

static const struct reconnect_scenario RECONNECT_SCENARIOS[] = {
    /* RngInit (initiator) reconnect - default/no-match rate -> class 0 */
    {"RngInit reconnect (rate=other, class 0)", 0, 0x08, 0x0064, 0, 0, 0, 0},
    /* RngInit reconnect - 14400bps -> class 5 */
    {"RngInit reconnect (rate=0x3840, class 5)", 0, 0x08, 0x3840, 0, 0, 0, 0},
    /* RngInit reconnect - 12000bps -> class 4 */
    {"RngInit reconnect (rate=0x2ee0, class 4)", 0, 0x08, 0x2ee0, 0, 0, 0, 0},
    /* RngInit reconnect - 9600bps, ctl+0x10==0 -> class 2 */
    {"RngInit reconnect (rate=0x2580, word10=0, class 2)", 0, 0x08, 0x2580, 0, 0, 0, 0},
    /* RngInit reconnect - 9600bps, ctl+0x10!=0 -> class 1 */
    {"RngInit reconnect (rate=0x2580, word10!=0, class 1)", 0, 0x08, 0x2580, 0x99, 0, 0, 0},
    /* RngInit reconnect - 7200bps -> class 3 */
    {"RngInit reconnect (rate=0x1c20, class 3)", 0, 0x08, 0x1c20, 0, 0, 0, 0},
    /* RngResp (responder) reconnect - reached via a direct hdxobj+0x76=5
     * poke, the same "construct normally, then poke the role field this
     * dispatch layer would otherwise set" precedent already used by
     * test/v32/v32_engine.rng.main.c for these same two roles. */
    {"RngResp reconnect (role poke)", 0, 0x08, 0, 0, 1, 0, 0},
    /* Full reconnect, Originate role, no bigbuf+0x50d8 rate override */
    {"Full reconnect (Originate, no override)", 0, 0x04, 0, 0, 0, 0, 0},
    /* Full reconnect, Answer role, no bigbuf+0x50d8 rate override */
    {"Full reconnect (Answer, no override)", 1, 0x04, 0, 0, 0, 0, 0},
    /* Full reconnect, Originate role, WITH the bigbuf+0x50d8 rate override
     * (connectRate=3 -> 0x12c0/4800bps, staying at rate class 0) */
    {"Full reconnect (Originate, override connectRate=3)", 0, 0x04, 0, 0, 0, 1, 3},
};
#define NUM_RECONNECT_SCENARIOS \
  (int)(sizeof(RECONNECT_SCENARIOS) / sizeof(RECONNECT_SCENARIOS[0]))
#define NUM_SCENARIOS (NUM_PLAIN_SCENARIOS + NUM_RECONNECT_SCENARIOS)

static void run_reconnect_scenario(const struct reconnect_scenario *sc)
{
  int cfg[6];
  char *obj, *hdxobj, *bigbuf;
  short status[STATUS_WORDS];
  unsigned char ctlBytes[40];
  int ret;
  void *p;

  printf("== %s ==\n", sc->label);

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = sc->role;
  cfg[1] = 100;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1;
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* tx rate 4800, both halves */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    return;
  }
  obj = (char *)(p);
  hdxobj = V32W_PG(char *, obj, 0x64);
  bigbuf = V32W_PG(char *, obj, 0x68);

  if (sc->pokeRngRespRole) {
    *(short *)(intptr_t)(hdxobj + 0x76) = 5;
  }
  if (sc->pokeSnrFlag) {
    *(short *)(intptr_t)(bigbuf + 0x50d8) = 1;
    *(short *)(intptr_t)(bigbuf + 0x2e) = sc->pokeConnectRate;
  }

  memset(status, 0, sizeof(status));
  V32FP_status(obj, status);
  dump_status("status before control", status);

  /* ctl[6] (byte 0xc) left 0, ctl+10 (byte 0x14) left 0 (so the
   * NextState-free SAMPLE_LEN-ratio block - already exercised by scenarios
   * 0/1 - is skipped here, keeping these scenarios focused on the
   * NextState-calling branches alone). ctl+0xc/+0xe (bytes 0x18/0x1c) are
   * still given distinct nonzero values purely so the unconditional
   * bigbuf p[5]/p[6] field writes remain visible in the status dump. */
  memset(ctlBytes, 0, sizeof(ctlBytes));
  ctlBytes[13] = sc->ctlOptions;
  *(short *)(ctlBytes + 0) = sc->ctlRate;             /* ctl[0] */
  *(unsigned int *)(ctlBytes + 0x10) = sc->ctlWord10; /* ctl+0x10 (byte offset) */
  *(unsigned int *)(ctlBytes + 24) = 0x1234;          /* ctl+0xc */
  *(unsigned int *)(ctlBytes + 28) = 0x5678;          /* ctl+0xe */

  ret = V32FP_control(obj, (short *)(void *)ctlBytes);
  printf("control ret=%d optionsAfter=0x%02x\n", ret, (unsigned)ctlBytes[13]);

  printf("raw after control: state=%d flags=0x%02x txRate=%d rxRate=%d obj1c=%d "
         "hdxstate=%d hdxrole=%d hdxelapsed=%d hdxbudget=%d hdx44=%d "
         "bigbuf28=%d bigbuf2a=%d bigbuf2e=%d bigbuf50d8=%d\n",
         (int)(signed char)*(unsigned char *)(intptr_t)(obj + 0x30),
         (unsigned)*(unsigned char *)(intptr_t)(obj + 0x31),
         (int)*(short *)(intptr_t)(obj + 2), (int)*(short *)(intptr_t)(obj + 4),
         *(int *)(intptr_t)(obj + 0x1c), (int)*(short *)(intptr_t)(hdxobj + 0x74),
         (int)*(short *)(intptr_t)(hdxobj + 0x76), *(int *)(intptr_t)(hdxobj + 0x7c),
         *(int *)(intptr_t)(hdxobj + 0x78), (int)*(unsigned short *)(intptr_t)(hdxobj + 0x44),
         (int)*(short *)(intptr_t)(bigbuf + 0x28), (int)*(short *)(intptr_t)(bigbuf + 0x2a),
         (int)*(short *)(intptr_t)(bigbuf + 0x2e), (int)*(short *)(intptr_t)(bigbuf + 0x50d8));

  memset(status, 0, sizeof(status));
  V32FP_status(obj, status);
  dump_status("status after control", status);

  V32FP_delete(obj);
  printf("== done ==\n");
}

int main(int argc, char **argv)
{
  int idx;
  if (argc != 2) {
    fprintf(stderr, "usage: %s <scenario-index 0..%d>\n", argv[0], NUM_SCENARIOS - 1);
    return 2;
  }
  idx = atoi(argv[1]);
  if (idx < 0 || idx >= NUM_SCENARIOS) {
    fprintf(stderr, "scenario index out of range\n");
    return 2;
  }
  if (idx < NUM_PLAIN_SCENARIOS) {
    run_scenario(SCENARIOS[idx].label, SCENARIOS[idx].role);
  } else {
    run_reconnect_scenario(&RECONNECT_SCENARIOS[idx - NUM_PLAIN_SCENARIOS]);
  }
  return 0;
}
