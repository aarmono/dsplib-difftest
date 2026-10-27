/*
 * Differential test harness for V.8's handshake FSM (v8handshak) - see
 * src/v8/v8.c's header comment on v8handshak for exactly what is and is not
 * covered. Originally just the ANSWER side's immediate post-init sequence
 * (tx-phase 5 silence, rx-phase 0x19 sub-state 0x19 ANSam-wait, rx-phase
 * 0xb parked-post-timeout). Extended to also cover the CALLER side's
 * immediate post-init sequence: tx-phase 6 (CM tone generation - two phase
 * accumulators feeding two v8_cosread lookups multiplied together, then
 * v8_fsktxfilter) and rx-phase 0x20 (a fixed 601-call AGC-settle counter
 * transitioning to rx-phase 0x28/sub-state 0x29, not itself covered by this
 * port - scenarios below stop at or before that transition).
 *
 * Same pattern as test/v8/v8.init.main.c: v8handshak_test_blob (linked
 * against the real dsplibs.o - v8handshak is a globally exported symbol per
 * `nm ../dsplibs.o`) vs v8handshak_test_port (linked against ../src/v8/v8.c +
 * ../src/v8/v8_tables.c). Builds an ANSWER-side object exactly the way
 * V8Create does (mirroring test/v8/v8.init.main.c's field writes), calls
 * the real v8handshakinit() to get a genuine post-init state, then drives
 * v8handshak() call-by-call.
 *
 * v8handshak() itself does not touch obj+0x110 (RX samples ready) or
 * obj+0x5c8 (RX input sample buffer) - both are normally maintained by the
 * still-unported V8Process/v8_rxreadqueue plumbing. Since this harness calls
 * v8handshak() directly (bypassing V8Process entirely, the same "direct
 * field poke" approach test/v8/v8.init.main.c uses for v8handshakinit),
 * it pokes both fields itself before every call: obj+0x110 to a fixed value
 * >=6 (or 0, to test the "not enough RX samples yet" early-return branch),
 * and obj+0x5c8 to all-zero (silence) RX samples - deterministically never
 * triggering a real ANSam detection in v8_tone_detect (already bit-exact
 * verified in the V.8 toolkit pass), so every scenario below stays inside
 * the sub-state 0x19 branch until (if at all) it times out into rx-phase
 * 0xb, without ever needing to hit this port's `abort()` guards for
 * not-yet-implemented states.
 *
 * Never prints a raw pointer value - obj+0xc48/0xc4c (message-sequence
 * pointers set by v8handshakinit) are never read back by this test.
 */

#include <stdio.h>
#include <string.h>
#include "v8_hobj.h"

extern unsigned int v8handshakinit(void *param_1);
extern unsigned int v8handshak(void *param_1);
extern unsigned int dsplibs_debug_level;

/* Same convention as test/v8/v8.main.main.c / test/v8/v8.init.main.c. */
#define V8OBJ_SIZE 0xee0

static void run_scenario(const char *label, int side, int sigTO, int rxReady, int numCalls)
{
  /* caps must outlive v8handshakinit (obj+0xa58 stores its address) for the
   * whole scenario, even though this path never dereferences it again
   * afterward for an all-zero caps buffer - kept alive here regardless, to
   * avoid relying on that. */
  static unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  static unsigned char caps[32];
  unsigned char *capsPtr = caps;
  int opMode = 0, msgTO = 0, a54 = 0;
  unsigned short a42 = 0x4000;
  int i;

  memset(obj, 0x55, V8H_BUFSZ(V8OBJ_SIZE));
  memset(caps, 0, sizeof(caps));
  memcpy(obj + 0xa44, &side, 4);
  memcpy(obj + 0xa48, &opMode, 4);
  memcpy(obj + 0xa4c, &sigTO, 4);
  memcpy(obj + 0xa50, &msgTO, 4);
  memcpy(obj + 0xa54, &a54, 4);
  V8H_PSET(obj, 0xa58, capsPtr);
  memcpy(obj + 0xa42, &a42, 2);

  v8handshakinit(obj);

  printf("== %s ==\n", label);
  for (i = 0; i < numCalls; i++) {
    unsigned int ret;
    short rxSamp[4] = {0, 0, 0, 0};
    short rxReadyVal = (short)rxReady;
    /* v8_txinit leaves obj+0x21c (TX samples generated so far) already
     * above obj+0xa3e's target (0x20 >= 0x10), so v8handshak's TX-phase
     * generation loop would never actually run on any call unless
     * something drains it back down first - normally the still-unported
     * V8Process/audio-output side does this as it consumes queued TX
     * samples. Poking it to 0 before every call simulates "the output
     * side has consumed everything queued so far", which is what actually
     * makes this harness exercise the tx-phase 5 (silence) loop at all;
     * see src/v8/v8.c's v8handshak header comment. */
    unsigned short zero21c = 0;
    unsigned short rxPhase, subState, txBuf[4], after21c, db6, f3c;
    short dac;
    int e64;

    memcpy(obj + 0x21c, &zero21c, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));

    ret = v8handshak(obj);

    memcpy(&rxPhase, obj + 0x9d6, 2);
    memcpy(&subState, obj + 0x9d8, 2);
    memcpy(&e64, obj + 0xe64, 4);
    memcpy(txBuf, obj + 0x5c0, sizeof(txBuf));
    memcpy(&after21c, obj + 0x21c, 2);
    memcpy(&db6, obj + 0xdb6, 2);
    memcpy(&f3c, obj + 0x3c, 2);
    memcpy(&dac, obj + 0xdac, 2);

    printf("call %d: ret=%u rxphase=0x%x sub=0x%x e64=%d tx21c=0x%x tx=%04x,%04x,%04x,%04x db6=0x%x f3c=0x%x dac=%d\n",
           i, ret, rxPhase, subState, e64, after21c,
           (unsigned short)txBuf[0], (unsigned short)txBuf[1],
           (unsigned short)txBuf[2], (unsigned short)txBuf[3],
           db6, f3c, dac);
  }
}

/* --- rx-phase 0x28 (main CM/JM FSK-demod tick) coverage ---
 *
 * S5 continues S4's caller-side object past the rx-phase 0x20->0x28
 * transition, driving further calls with all-zero (silence) RX input - the
 * same "never triggers a real detection" rationale as every other scenario
 * here, just for v8_fskdemodulate instead of v8_tone_detect. This exercises
 * the shared preamble (V8agc + v8_fskdemodulate + the c3e/c40/c42/c44/c46
 * bit-accumulation bookkeeping + the c38/c3a resync decision) under real,
 * if degenerate, conditions: sustained silence drives the mark/space
 * correlator to a stable comparison, so the demodulated bit stream settles
 * into a constant (all-0 or all-1) run rather than ever assembling the
 * specific alternating 0xc0f/0xd55 sync words sub-state 0x29 looks for -
 * confirmed by inspection of the resulting c3a trace below, not assumed in
 * advance, so this scenario never needs this port's abort() guards for
 * sub-state 0x28/0x2c/generic-lost-sync.
 *
 * Sub-state 0x29's own two pattern-match branches, and sub-state 0x23's
 * timeout/slot-advance logic, are NOT reachable this way (forming a chosen
 * 12-bit sync pattern via the real correlator would require reverse-
 * engineering a specific RX sample sequence - out of scope for this pass).
 * Covered instead by direct field-poke scenarios below (same "poke the
 * fields a still-unported caller would normally drive" approach already
 * established for +0x110/+0x21c elsewhere in this file), each engineered so
 * v8_fskdemodulate's own bit-accumulation this call is a guaranteed no-op
 * (+0xc36, its 12-sample sub-frame counter, poked to 0 so the call returns
 * before ever touching +0xc38/+0xc3a - verified against its disassembly-
 * confirmed early-return at `if (*(short*)(p+0xc36)!=0xc) return;`, see
 * src/v8/v8.c's v8_fskdemodulate) and the c44/c46 resync-reset guaranteed not
 * to fire (+0xc44/+0xc46 poked equal), so the crafted +0xc3a value survives
 * untouched all the way to the sub-state dispatch this test targets. */
static void run_rx0x28_natural(void)
{
  static unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  static unsigned char caps[32];
  unsigned char *capsPtr = caps;
  int side = 1, opMode = 0, sigTO = 0, msgTO = 0, a54 = 0;
  unsigned short a42 = 0x4000;
  int i;

  memset(obj, 0x55, V8H_BUFSZ(V8OBJ_SIZE));
  memset(caps, 0, sizeof(caps));
  memcpy(obj + 0xa44, &side, 4);
  memcpy(obj + 0xa48, &opMode, 4);
  memcpy(obj + 0xa4c, &sigTO, 4);
  memcpy(obj + 0xa50, &msgTO, 4);
  memcpy(obj + 0xa54, &a54, 4);
  V8H_PSET(obj, 0xa58, capsPtr);
  memcpy(obj + 0xa42, &a42, 2);

  v8handshakinit(obj);

  printf("== S5_caller_rx0x28_steady ==\n");
  for (i = 0; i < 700; i++) {
    unsigned int ret;
    short rxSamp[4] = {0, 0, 0, 0};
    unsigned short rxReadyVal = 8, zero21c = 0;
    unsigned short rxPhase, subState, c38v, c3av, c44v, c46v;

    memcpy(obj + 0x21c, &zero21c, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));

    ret = v8handshak(obj);

    if (i < 610 || i % 20 == 0 || i >= 695) {
      memcpy(&rxPhase, obj + 0x9d6, 2);
      memcpy(&subState, obj + 0x9d8, 2);
      memcpy(&c38v, obj + 0xc38, 2);
      memcpy(&c3av, obj + 0xc3a, 2);
      memcpy(&c44v, obj + 0xc44, 2);
      memcpy(&c46v, obj + 0xc46, 2);
      printf("call %d: ret=%u rxphase=0x%x sub=0x%x c38=%d c3a=0x%x c44=%d c46=%d\n",
             i, ret, rxPhase, subState, (short)c38v, c3av, (short)c44v, (short)c46v);
    }
  }
}

/* Fresh answer-side object (rx-phase/sub-state/demod fields directly poked -
 * see run_rx0x28_natural's header comment above for why each poke is safe)
 * used by every case-0x28 sub-state scenario below. `capBit10` controls
 * whether the capabilities struct's 0x10 bit is set before v8handshakinit
 * runs, since that bit gates whether v8handshakinit wires up obj+0xc50 at
 * all (needed for the 0xd55/QCA1-path scenario, harmless for the others). */
static void make_case28_obj(unsigned char *obj, unsigned char *caps, int capBit10)
{
  unsigned char *capsPtr = caps;
  int side = 0, opMode = 0, sigTO = 0, msgTO = 0, a54 = 0;
  unsigned short a42 = 0x4000;

  memset(obj, 0x55, V8H_BUFSZ(V8OBJ_SIZE));
  memset(caps, 0, 32);
  if (capBit10) caps[2] = 0x10;
  memcpy(obj + 0xa44, &side, 4);
  memcpy(obj + 0xa48, &opMode, 4);
  memcpy(obj + 0xa4c, &sigTO, 4);
  memcpy(obj + 0xa50, &msgTO, 4);
  memcpy(obj + 0xa54, &a54, 4);
  V8H_PSET(obj, 0xa58, capsPtr);
  memcpy(obj + 0xa42, &a42, 2);

  v8handshakinit(obj);
}

static void dump_case28(const char *label, unsigned char *obj, unsigned int ret)
{
  unsigned short txPhase, rxPhase, sub, c38v, c3av, db6, dbc;
  char *c4cPtr, *c50Ptr, *c48Ptr;
  unsigned short c4c0, c4c14, c500, c5014;

  memcpy(&txPhase, obj + 0x9d4, 2);
  memcpy(&rxPhase, obj + 0x9d6, 2);
  memcpy(&sub, obj + 0x9d8, 2);
  memcpy(&c38v, obj + 0xc38, 2);
  memcpy(&c3av, obj + 0xc3a, 2);
  memcpy(&db6, obj + 0xdb6, 2);
  memcpy(&dbc, obj + 0xdbc, 2);
  V8H_PGET(c4cPtr, obj, 0xc4c);
  V8H_PGET(c50Ptr, obj, 0xc50);
  V8H_PGET(c48Ptr, obj, 0xc48);
  memcpy(&c4c0, c4cPtr + 0, 2);
  memcpy(&c4c14, c4cPtr + 0x28, 2);
  /* obj+0xc50 is only ever wired up by v8handshakinit when the caller-
   * supplied capability struct's 0x10 bit is set (see make_case28_obj's
   * `capBit10` and v8handshakinit's own comment) - scenarios that leave it
   * unset legitimately leave this pointer field as this test's 0x55 fill
   * pattern, so it must not be dereferenced then (confirmed by an actual
   * SIGSEGV the first time this test ran without this guard). */
  if ((char *)obj <= c50Ptr && c50Ptr < (char *)obj + V8OBJ_SIZE) {
    memcpy(&c500, c50Ptr + 0, 2);
    memcpy(&c5014, c50Ptr + 0x28, 2);
  } else {
    c500 = c5014 = 0xdead;
  }

  /* c4c/c48/c50 all point within the same object in both binaries - printed
   * as offsets from the object base (meaningful across two separately-
   * linked binaries), never as raw addresses, same convention as
   * test/v8/v8.init.main.c. */
  printf("%s: ret=%u tx=0x%x rx=0x%x sub=0x%x c38=%d c3a=0x%x db6=%d dbc=%d "
         "c48off=0x%x c4coff=0x%x c4c[0]=0x%x c4c[0x14]=0x%x "
         "c50off=%s c50[0]=0x%x c50[0x14]=0x%x\n",
         label, ret, txPhase, rxPhase, sub, (short)c38v, c3av, db6, dbc,
         (unsigned int)(c48Ptr - (char *)obj), (unsigned int)(c4cPtr - (char *)obj),
         c4c0, c4c14,
         ((char *)obj <= c50Ptr && c50Ptr < (char *)obj + V8OBJ_SIZE) ? "valid" : "n/a",
         c500, c5014);
}

static void run_case28_pokes(void)
{
  static unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  static unsigned char caps[32];
  short rxSamp[4] = {0, 0, 0, 0};
  unsigned short rxReadyVal = 8;

  /* B1: sub-state 0x29, c3a's low 12 bits == 0xc0f (CI/CM sync word found) -
   * should seed the c4c word array (index 0 = 0xf, indices 1..14 = 0x3ff,
   * index 0x14 = 0xffff), sub-state -> 0x28, db6=dbc=1, c38=c3a=0. */
  {
    unsigned short rxPhase = 0x28, sub = 0x29, c3a = 0xc0f, c38 = 0, c36 = 0, c44 = 0, c46 = 0;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc3a, &c3a, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("B1_sub29_c0f_match", obj, ret);
  }

  /* B2: sub-state 0x29, c3a's low 12 bits == 0xd55, capability bit 0x10 set
   * (QCA1 path) - should seed the c50 word array (index 0 = 0x155, indices
   * 1..5 = 0x3ff, index 0x14 = 0xffff), sub-state -> 0x2c, db6=dbc=1,
   * c38=c3a=0. */
  {
    unsigned short rxPhase = 0x28, sub = 0x29, c3a = 0xd55, c38 = 0, c36 = 0, c44 = 0, c46 = 0;
    unsigned int ret;
    make_case28_obj(obj, caps, 1);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc3a, &c3a, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("B2_sub29_d55_match_capbit", obj, ret);
  }

  /* B3: sub-state 0x29, c3a matches neither pattern - no state change other
   * than the (already-covered) shared preamble's own bookkeeping. */
  {
    unsigned short rxPhase = 0x28, sub = 0x29, c3a = 0x123, c38 = 0, c36 = 0, c44 = 0, c46 = 0;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc3a, &c3a, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("B3_sub29_no_match", obj, ret);
  }

  /* B4: sub-state 0x23, slot+0x34==0, db6==0 - advances +0xc48 to obj+0xc94,
   * sets db6=1. */
  {
    unsigned short rxPhase = 0x28, sub = 0x23, c36 = 0, c44 = 0, c46 = 0, db6 = 0;
    unsigned short zero = 0;
    unsigned int ret;
    char *slot;
    make_case28_obj(obj, caps, 0);
    V8H_PGET(slot, obj, 0xc48);
    memcpy(slot + 0x34, &zero, 2);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0xdb6, &db6, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("B4_sub23_slot_advance", obj, ret);
  }

  /* B5: sub-state 0x23, slot+0x34!=0, db6==0 - no change. */
  {
    unsigned short rxPhase = 0x28, sub = 0x23, c36 = 0, c44 = 0, c46 = 0, db6 = 0;
    unsigned short one = 1;
    unsigned int ret;
    char *slot;
    make_case28_obj(obj, caps, 0);
    V8H_PGET(slot, obj, 0xc48);
    memcpy(slot + 0x34, &one, 2);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0xdb6, &db6, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("B5_sub23_slot_not_ready", obj, ret);
  }

  /* B6: sub-state 0x23, db6==0x103 -> this call's increment hits the 0x104
   * timeout: tx-phase->5, rx-phase->99 (0x63), return 2. */
  {
    unsigned short rxPhase = 0x28, sub = 0x23, c36 = 0, c44 = 0, c46 = 0, db6 = 0x103;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0xdb6, &db6, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("B6_sub23_timeout", obj, ret);
  }

  /* B7: rx-phase 99 (0x63) itself, reached after a prior call like B6 - a
   * further call should just return 2 every time with no state change. */
  {
    unsigned short rxPhase = 99;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("B7_rxphase99_terminal", obj, ret);
  }

  /* B8: top-level rx-phase 0x23 (NOT the sub-state 0x23 tested above) -
   * just forwards to v8_rxreadqueue (already bit-exact verified) and
   * returns 0; dump_case28 will show sub/c38/c3a unchanged. */
  {
    unsigned short rxPhase = 0x23;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("B8_rxphase0x23_toplevel", obj, ret);
  }
}

/* --- sub-state 0x2c (QCA1/LAPM detector) coverage ---
 *
 * Directly engineers the word buffer at +0xc50 (only wired up by
 * v8handshakinit when the caps struct's 0x10 bit is set - same
 * capBit10=1 requirement as run_case28_pokes' B2) plus +0xdbc (word index)
 * and this call's own decoded word (via the same c38=10/c3a=word10/c36=0
 * poke trick already established) to hit each of sub-state 0x2c's three
 * outcomes once its 6th word arrives (dbc reaches 6): a QCA1d (digital
 * connection) match, a QCA1a-style match, and a no-match reset back to
 * sub-state 0x29. w1/w4 values chosen via a small brute-force search over
 * the real (w1&0x3b9)==0x181 / (w1&0x391)==0x81 conditions (see this pass's
 * derivation in NOTES.md) - not guessed. */
static void run_case2c_pokes(void)
{
  static unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  static unsigned char caps[32];
  short rxSamp[4] = {0, 0, 0, 0};
  unsigned short rxReadyVal = 8;

  /* C1: wbuf[1]=0x181 ((w1&0x3b9)==0x181 -> QCA1d), wbuf[2]=0x3ff,
   * wbuf[3]=0x155 (dword at wbuf+4 == 0x15503ff), wbuf[4]=0x181 (==w1), this
   * call's word10=0x3f0 (wbuf[5]&0x3f0==0x3f0) - terminal: returns 2,
   * tx-phase->5, rx-phase->99. */
  {
    unsigned short rxPhase = 0x28, sub = 0x2c, c36 = 0, c44 = 0, c46 = 0;
    unsigned short c38 = 10, c3a = 0x3f0, dbc = 5;
    unsigned short w1 = 0x181, w2 = 0x3ff, w3 = 0x155, w4 = 0x181;
    unsigned int ret;
    char *wbufPtr;
    make_case28_obj(obj, caps, 1);
    V8H_PGET(wbufPtr, obj, 0xc50);
    memcpy(wbufPtr + 2, &w1, 2);
    memcpy(wbufPtr + 4, &w2, 2);
    memcpy(wbufPtr + 6, &w3, 2);
    memcpy(wbufPtr + 8, &w4, 2);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc3a, &c3a, 2);
    memcpy(obj + 0xdbc, &dbc, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("C1_sub2c_qca1d_match", obj, ret);
  }

  /* C2: same, but w1=w4=0x81 ((w1&0x391)==0x81, and (w1&0x3b9)!=0x181, so the
   * QCA1a branch runs instead) - transitions to rx-phase/sub-state 0x19
   * (ANSam-wait, for LAPM), NOT terminal, returns 0. */
  {
    unsigned short rxPhase = 0x28, sub = 0x2c, c36 = 0, c44 = 0, c46 = 0;
    unsigned short c38 = 10, c3a = 0x3f0, dbc = 5;
    unsigned short w1 = 0x81, w2 = 0x3ff, w3 = 0x155, w4 = 0x81;
    unsigned int ret;
    char *wbufPtr;
    make_case28_obj(obj, caps, 1);
    V8H_PGET(wbufPtr, obj, 0xc50);
    memcpy(wbufPtr + 2, &w1, 2);
    memcpy(wbufPtr + 4, &w2, 2);
    memcpy(wbufPtr + 6, &w3, 2);
    memcpy(wbufPtr + 8, &w4, 2);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc3a, &c3a, 2);
    memcpy(obj + 0xdbc, &dbc, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("C2_sub2c_qca1a_match", obj, ret);
  }

  /* C3: w1=w4=0 - matches neither (w1&0x3b9)==0x181 nor (w1&0x391)==0x81, so
   * the overall AND is false regardless of the other fields - resets
   * sub-state back to 0x29, returns 0. */
  {
    unsigned short rxPhase = 0x28, sub = 0x2c, c36 = 0, c44 = 0, c46 = 0;
    unsigned short c38 = 10, c3a = 0x3f0, dbc = 5;
    unsigned short w1 = 0, w2 = 0x3ff, w3 = 0x155, w4 = 0;
    unsigned int ret;
    char *wbufPtr;
    make_case28_obj(obj, caps, 1);
    V8H_PGET(wbufPtr, obj, 0xc50);
    memcpy(wbufPtr + 2, &w1, 2);
    memcpy(wbufPtr + 4, &w2, 2);
    memcpy(wbufPtr + 6, &w3, 2);
    memcpy(wbufPtr + 8, &w4, 2);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc3a, &c3a, 2);
    memcpy(obj + 0xdbc, &dbc, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("C3_sub2c_no_match_reset", obj, ret);
  }

  /* C4: dbc==3 (not yet the 6th word) - confirms the ==6 check doesn't fire
   * prematurely; wbuf[dbc] gets this call's word written but no pattern
   * check runs, sub-state stays 0x2c untouched. */
  {
    unsigned short rxPhase = 0x28, sub = 0x2c, c36 = 0, c44 = 0, c46 = 0;
    unsigned short c38 = 10, c3a = 0x1a5, dbc = 3;
    unsigned int ret;
    make_case28_obj(obj, caps, 1);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc3a, &c3a, 2);
    memcpy(obj + 0xdbc, &dbc, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("C4_sub2c_dbc_not_yet_6", obj, ret);
  }
}

/* --- generic lost-sync timeout coverage (sub-state outside
 * {0x28,0x29,0x23,0x2c,0x32,0x33}) ---
 *
 * Uses sub-state 0x24 (a real, reachable value - rx-phase 0x19's own
 * tone-detect transition sets it) with directly-poked +0xdb8/+0xdb6
 * bookkeeping fields and a directly-poked word10 (via the same
 * c38=10/c3a=word10/c36=0 trick as elsewhere in this file) to drive the
 * 10-bit-scan loop precisely: word10=1 is a single 1-bit preceded by 9 zero
 * bits (MSB-first over the 10-bit word), so one call with a fresh (0,0)
 * (db8,db6) hits the "9 zeros then a 1" pattern once (db6->1, not yet
 * terminal); a second identical call hits it again (db6->2, terminal).
 * word10=0 (never a 1 bit) never hits the pattern at all. Sub-state 0x32 is
 * separately confirmed to no-op the whole cascade (disassembly-verified
 * `LEA EBP,[EDX-0x32]; CMP BP,1; JBE ...`), even with an otherwise-matching
 * word10. */
static void run_generic_timeout_pokes(void)
{
  static unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  static unsigned char caps[32];
  short rxSamp[4] = {0, 0, 0, 0};
  unsigned short rxReadyVal = 8;
  unsigned short rxPhase = 0x28, sub = 0x24, c36 = 0, c44 = 0, c46 = 0;
  unsigned short c38 = 10, c3a = 1, db8 = 0, db6 = 0;
  unsigned int ret;

  /* D1/D2: first and second "9 zeros then a 1" hits on the same object. */
  make_case28_obj(obj, caps, 0);
  memcpy(obj + 0x9d6, &rxPhase, 2);
  memcpy(obj + 0x9d8, &sub, 2);
  memcpy(obj + 0xc38, &c38, 2);
  memcpy(obj + 0xc3a, &c3a, 2);
  memcpy(obj + 0xc36, &c36, 2);
  memcpy(obj + 0xc44, &c44, 2);
  memcpy(obj + 0xc46, &c46, 2);
  memcpy(obj + 0xdb8, &db8, 2);
  memcpy(obj + 0xdb6, &db6, 2);
  memcpy(obj + 0x110, &rxReadyVal, 2);
  memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
  ret = v8handshak(obj);
  dump_case28("D1_generic_timeout_first_hit", obj, ret);

  memcpy(obj + 0x9d6, &rxPhase, 2);
  memcpy(obj + 0x9d8, &sub, 2);
  memcpy(obj + 0xc38, &c38, 2);
  memcpy(obj + 0xc3a, &c3a, 2);
  memcpy(obj + 0xc36, &c36, 2);
  memcpy(obj + 0xc44, &c44, 2);
  memcpy(obj + 0xc46, &c46, 2);
  memcpy(obj + 0x110, &rxReadyVal, 2);
  memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
  ret = v8handshak(obj);
  dump_case28("D2_generic_timeout_second_hit_terminal", obj, ret);

  /* D3: word10=0 (all-zero, no 1 bits ever) on a fresh object - db8 just
   * accumulates by 10 every call, db6 never changes. */
  {
    unsigned short c3a0 = 0, zero = 0;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc3a, &c3a0, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0xdb8, &zero, 2);
    memcpy(obj + 0xdb6, &zero, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("D3_generic_timeout_all_zero_word", obj, ret);
  }

  /* D4: sub-state 0x32 - the no-op case - even with c38==10 and a word10
   * that would otherwise hit D1's pattern, nothing should change. */
  {
    unsigned short sub32 = 0x32, zero = 0;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d6, &rxPhase, 2);
    memcpy(obj + 0x9d8, &sub32, 2);
    memcpy(obj + 0xc38, &c38, 2);
    memcpy(obj + 0xc3a, &c3a, 2);
    memcpy(obj + 0xc36, &c36, 2);
    memcpy(obj + 0xc44, &c44, 2);
    memcpy(obj + 0xc46, &c46, 2);
    memcpy(obj + 0xdb8, &zero, 2);
    memcpy(obj + 0xdb6, &zero, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_case28("D4_subst0x32_noop", obj, ret);
  }
}

/* --- rx-phase 0x19, sub-state != 0x19 (phase-reversal-accumulation branch)
 * coverage ---
 *
 * Uses make_case28_obj's answer-side setup (side=0, so v8handshakinit leaves
 * dd0=0, da0=0, dbe=1, tx-phase=5, rx-phase=sub-state=0x19 - the same
 * default post-init state every other answer-side scenario in this file
 * starts from), then pokes sub-state to 0x24 (the real value rx-phase 0x19
 * sub-state 0x19's own tone-detect branch transitions into) plus +0xdb6 to
 * 0x960 directly (so this call's own +1 increment crosses the ">0x960"
 * threshold immediately, without 2400+ calls of setup) and the handful of
 * fields the threshold body reads: +0xda0 (energy magnitude), +0xc1c
 * (secondary abort flag), +0xdd0 (a capability-derived latch bit) together
 * with the live capabilities byte's own bit 4 (0x10) - both of which feed
 * the SAME `(capsByte>>4)&1` term the real object ANDs against +0xdd0 (see
 * src/v8/v8.c's comment on this branch for the confirmed operator-precedence
 * reading) - and +0xdbe/the initial tx-phase value for the two outcomes
 * that depend on them. RX samples stay all-zero throughout (only this
 * branch's OWN accumulator/threshold bookkeeping is under test here, not
 * v8_phase_rev_detect's/checkSignalStability's/v8_dftupdate's own signal
 * response - those are already bit-exact verified by the V.8 toolkit pass).
 */
static void dump_phase_rev(const char *label, unsigned char *obj, unsigned int ret)
{
  unsigned short txPhase, rxPhase, sub, db6, a26, f3c, a3c, dbe;

  memcpy(&txPhase, obj + 0x9d4, 2);
  memcpy(&rxPhase, obj + 0x9d6, 2);
  memcpy(&sub, obj + 0x9d8, 2);
  memcpy(&db6, obj + 0xdb6, 2);
  memcpy(&a26, obj + 0x26, 2);
  memcpy(&f3c, obj + 0x3c, 2);
  memcpy(&a3c, obj + 0xa3c, 2);
  memcpy(&dbe, obj + 0xdbe, 2);

  printf("%s: ret=%u tx=0x%x rx=0x%x sub=0x%x db6=%d a26=0x%x f3c=0x%x a3c=%d dbe=%d\n",
         label, ret, txPhase, rxPhase, sub, db6, a26, f3c, a3c, dbe);
}

static void run_rx0x19_phase_reversal_pokes(void)
{
  static unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  static unsigned char caps[32];
  short rxSamp[4] = {0, 0, 0, 0};
  unsigned short rxReadyVal = 8;
  unsigned short sub24 = 0x24, db6thresh = 0x960;

  /* P1: threshold crossed, da0<400 && c1c==0, ansamCond!=0 (dd0=1,
   * capsByte bit4=1), tx-phase starts at 5 (not 0x2d) - expect tx-phase set
   * to 5 (a no-op in practice), return 1, rx-phase/sub-state unchanged. */
  {
    unsigned short da0 = 0, c1c = 0, dd0 = 1, tx = 5;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0x10;
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0x9d4, &tx, 2);
    memcpy(obj + 0xdb6, &db6thresh, 2);
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P1_reject_ansam_txphase5", obj, ret);
  }

  /* P2: same as P1, but tx-phase starts at 0x2d - expect the terminal
   * "handshake failed" transition instead: tx-phase=5, rx-phase=99,
   * return 2. */
  {
    unsigned short da0 = 0, c1c = 0, dd0 = 1, tx = 0x2d;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0x10;
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0x9d4, &tx, 2);
    memcpy(obj + 0xdb6, &db6thresh, 2);
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P2_reject_ansam_txphase2d_terminal", obj, ret);
  }

  /* P3: threshold crossed, da0<400 && c1c==0, ansamCond==0 (dd0=0) - the
   * inner tx-phase modification is skipped entirely; return 1, tx-phase
   * (started at 6) untouched. */
  {
    unsigned short da0 = 0, c1c = 0, dd0 = 0, tx = 6;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0x10;
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0x9d4, &tx, 2);
    memcpy(obj + 0xdb6, &db6thresh, 2);
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P3_reject_ansamcond0_notouch", obj, ret);
  }

  /* P4: threshold crossed, da0>=400 (skips the reject branch entirely),
   * ansamCond==0 (dd0=0), dbe==0 - the ANSAM-detected sub-branch's own
   * `if(dbe!=0)` guard fails, so nothing happens at all; return 0. */
  {
    unsigned short da0 = 500, c1c = 0, dd0 = 0, dbe = 0;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0;
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0xdb6, &db6thresh, 2);
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0xdbe, &dbe, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P4_ansam_dbe0_noop", obj, ret);
  }

  /* P5: da0>=400, ansamCond==0 (dd0=0), dbe!=0 (v8handshakinit's own
   * default for an opMode==0 answer-side object, left untouched here),
   * capsByte bit 0x10 == 0 -> full ANSAM-detected transition ending at
   * tx-phase 0x17. */
  {
    unsigned short da0 = 500, c1c = 0, dd0 = 0;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0;
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0xdb6, &db6thresh, 2);
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P5_ansam_detected_tx0x17", obj, ret);
  }

  /* P6: same as P5, but capsByte bit 0x10 set - ANSAM-detected transition
   * ends at tx-phase 0x2b instead (dd0 stays 0, so ansamCond is still 0
   * despite the bit now being set - it only affects the FINAL tx-phase
   * choice, a separate read of the same capsByte). */
  {
    unsigned short da0 = 500, c1c = 0, dd0 = 0;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0x10;
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0xdb6, &db6thresh, 2);
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P6_ansam_detected_tx0x2b", obj, ret);
  }

  /* P7: da0>=400, ansamCond!=0 (dd0=1, capsByte bit4=1) - takes the `else`
   * branch instead: tx-phase=0x2d, return 0. */
  {
    unsigned short da0 = 500, c1c = 0, dd0 = 1;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0x10;
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0xdb6, &db6thresh, 2);
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P7_ansamcond_txphase2d", obj, ret);
  }

  /* P8: same outcome as P7 (tx-phase=0x2d), but reached via the OTHER half
   * of the outer `&&` - da0<400 this time, c1c!=0 instead - confirming
   * both operands of `da0<400 && c1c==0` are genuinely read, not just the
   * first one. */
  {
    unsigned short da0 = 0, c1c = 1, dd0 = 1;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0x10;
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0xdb6, &db6thresh, 2);
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P8_c1c_forces_reject_path", obj, ret);
  }

  /* P9: below the threshold (db6 starts at 0, one call brings it to 1, well
   * under 0x960) - just exercises checkSignalStability/v8_phase_rev_detect
   * (and, with +0xa6 forced to 1, v8_dftupdate too) under real, if
   * degenerate (all-zero RX), conditions; return 0, no phase/sub-state
   * change. */
  {
    unsigned short zero = 0, one = 1;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d8, &sub24, 2);
    memcpy(obj + 0xdb6, &zero, 2);
    memcpy(obj + 0xa6, &one, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("P9_below_threshold_with_dft", obj, ret);
  }
}

/* Directly exercises tx-phase 0x17's "Timeout waiting for %s message..."
 * debug print (gated on dsplibs_debug_level>1, so silent in every other
 * scenario in this file) for BOTH sides, to confirm the two extracted
 * .rodata strings it substitutes are byte-exact - notably the CALLER side's
 * (+0xa44==1) string, resolved via direct .rodata byte inspection at its
 * computed file offset (see src/v8/v8.c's case 0x17 comment), is the literal
 * two-character "CJ", NOT "CM" as the "%s message" phrasing might suggest -
 * worth an explicit round-trip check given how easy that would be to
 * "correct" by assumption instead of trusting the extracted bytes (this
 * test's own first draft had the CJ/JM <-> side mapping backwards in its
 * prose comments until this run's actual stdout output - "V8: Timeout
 * waiting for JM message..." for side=0, "...CJ..." for side=1 - corrected
 * it). Poking +0xe60==+0xe64
 * (both 5) hits the `cnt==limit` first-time-timeout sub-branch, the one that
 * actually reaches the debug_printf call (see src/v8/v8.c's case 0x17
 * comment - the `cnt!=limit` sub-branch returns before ever printing). */
static void run_case17_debug_string_check(void)
{
  int side;

  for (side = 0; side <= 1; side++) {
    static unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
    static unsigned char caps[32];
    unsigned char *capsPtr = caps;
    int opMode = 0, sigTO = 0, msgTO = 0, a54 = 0;
    unsigned short a42 = 0x4000, tx17 = 0x17;
    int e60 = 5, e64 = 5;
    unsigned int ret;
    short rxSamp[4] = {0, 0, 0, 0};
    unsigned short rxReadyVal = 0;

    memset(obj, 0x55, V8H_BUFSZ(V8OBJ_SIZE));
    memset(caps, 0, sizeof(caps));
    memcpy(obj + 0xa44, &side, 4);
    memcpy(obj + 0xa48, &opMode, 4);
    memcpy(obj + 0xa4c, &sigTO, 4);
    memcpy(obj + 0xa50, &msgTO, 4);
    memcpy(obj + 0xa54, &a54, 4);
    V8H_PSET(obj, 0xa58, capsPtr);
    memcpy(obj + 0xa42, &a42, 2);
    v8handshakinit(obj);

    {
      /* v8_txinit leaves +0x21c above +0xa3e's target (see this project's
       * established "poke it to 0 before every call" convention, used by
       * every other scenario in this file) - without this, the TX-phase
       * generation loop's own bound check would skip the switch entirely
       * and this scenario would silently never reach tx-phase 0x17 at all. */
      unsigned short zero21c = 0;
      memcpy(obj + 0x21c, &zero21c, 2);
    }
    memcpy(obj + 0x9d4, &tx17, 2);
    memcpy(obj + 0xe60, &e60, 4);
    memcpy(obj + 0xe64, &e64, 4);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));

    dsplibs_debug_level = 2;
    printf("== R%d_case17_debug_string (side=%d) ==\n", side, side);
    ret = v8handshak(obj);
    dsplibs_debug_level = 0;
    printf("ret=%u\n", ret);
  }
}

/* --- tx-phase 0x17/0x2b (CM/JM message tick) and rx-phase 4 coverage ---
 *
 * tx-phase 0x17/0x2b are only ever entered via the phase-reversal branch's
 * ANSAM-detected transition just exercised above (P5/P6) - rather than
 * re-deriving a synthetic message slot, this drives the REAL post-P5/P6
 * object state (tx-phase 0x17 or 0x2b, rx-phase 0x28, sub-state 0x29) for
 * further calls, so v8_getbit reads the SAME real message data
 * v8handshakinit's own initTxSequence call already staged at +0xc48 (every
 * scenario in this file calls the real v8handshakinit first) - no synthetic
 * message setup needed, and no risk of feeding either binary data the real
 * object wouldn't also see in practice.
 *
 * rx-phase 4 (tx-phase 6's own CM-timeout target - see run_scenario's S3/S4
 * for tx-phase 6 itself) is covered two ways: E1 exercises it as SET (via
 * msgTO, a real, un-poked timeout - confirms case 6's own timeout branch,
 * not just case 0x17/0x2b's), and E2 directly pokes rx-phase=4 with
 * tx-phase=5 (silence, already verified) to confirm the dispatch itself is
 * a genuine no-op (disassembly-confirmed, see src/v8/v8.c's case 4 comment) -
 * since case 6's own timeout, once fired, never actually falls through to
 * the RX dispatch on any subsequent call (it short-circuits with an
 * immediate `return 1` every time - see src/v8/v8.c's case 0x17 comment), so
 * E2's direct poke is the only way to observe rx-phase 4's dispatch body
 * itself, not just the transition into it. */
static void run_case17_2b_and_rxphase4(void)
{
  static unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  static unsigned char caps[32];
  short rxSamp[4] = {0, 0, 0, 0};
  unsigned short rxReadyVal = 8;

  /* E1: caller side, msgTO=1 (e60 = (1*0x2580)>>2 = 0x960 = 2400), sigTO=0
   * (e5c=-1, irrelevant here since caller-side starts at rx-phase 0x20, not
   * 0x19), rxReady=0 throughout (isolates tx-phase 6's own CM-timeout logic
   * from any RX-phase interaction) - runs past the exact call index (2400)
   * the real e60 timeout fires at, confirming rx-phase transitions to 4
   * (and that every call after it keeps returning 1 immediately without
   * further state change, matching the "hard return, RX dispatch never
   * reached" reading above). */
  {
    unsigned char *capsPtr = caps;
    int side = 1, opMode = 0, sigTO = 0, msgTO = 1, a54 = 0;
    unsigned short a42 = 0x4000;
    int i;

    memset(obj, 0x55, V8H_BUFSZ(V8OBJ_SIZE));
    memset(caps, 0, sizeof(caps));
    memcpy(obj + 0xa44, &side, 4);
    memcpy(obj + 0xa48, &opMode, 4);
    memcpy(obj + 0xa4c, &sigTO, 4);
    memcpy(obj + 0xa50, &msgTO, 4);
    memcpy(obj + 0xa54, &a54, 4);
    V8H_PSET(obj, 0xa58, capsPtr);
    memcpy(obj + 0xa42, &a42, 2);
    v8handshakinit(obj);

    printf("== E1_caller_msgTO_cmtimeout ==\n");
    for (i = 0; i < 2404; i++) {
      unsigned int ret;
      unsigned short rxReadyZero = 0, zero21c = 0;
      unsigned short txPhase, rxPhase;
      int e64;

      memcpy(obj + 0x21c, &zero21c, 2);
      memcpy(obj + 0x110, &rxReadyZero, 2);
      memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
      ret = v8handshak(obj);
      if (i < 2398 ? (i % 400 == 0) : 1) {
        memcpy(&txPhase, obj + 0x9d4, 2);
        memcpy(&rxPhase, obj + 0x9d6, 2);
        memcpy(&e64, obj + 0xe64, 4);
        printf("call %d: ret=%u tx=0x%x rx=0x%x e64=%d\n", i, ret, txPhase, rxPhase, e64);
      }
    }
  }

  /* E2: direct poke - tx-phase 5 (silence), rx-phase 4 - confirms the
   * dispatch body itself (not reachable via E1's transition alone, since
   * case 6's own timeout never falls through to the RX dispatch on any
   * call - see this function's header comment): a genuine no-op, return 0,
   * no field changes other than tx-phase 5's own (already-verified) silence
   * generation. */
  {
    unsigned short tx = 5, rx4 = 4;
    unsigned int ret;
    make_case28_obj(obj, caps, 0);
    memcpy(obj + 0x9d4, &tx, 2);
    memcpy(obj + 0x9d6, &rx4, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("E2_rxphase4_noop", obj, ret);
  }

  /* E3/E4: continue a fresh P5-equivalent object (tx-phase 0x17 after the
   * ANSAM-detected transition, capsByte bit 0x10 == 0) for 60 further calls
   * to exercise tx-phase 0x17's own FSK-modulate-and-refill-bit loop
   * against the real staged message data - msgTO left at 0 (e60=-1) so the
   * "waiting for CM/JM" timeout branch never fires, isolating the steady
   * modulate loop itself. */
  {
    unsigned short da0 = 500, c1c = 0, dd0 = 0, sub24 = 0x24;
    unsigned int ret;
    int i;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0;
    memcpy(obj + 0x9d8, &sub24, 2);
    {
      unsigned short db6thresh = 0x960;
      memcpy(obj + 0xdb6, &db6thresh, 2);
    }
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("E3_ansam_detected_setup_tx0x17", obj, ret);

    printf("== E4_tx0x17_steady ==\n");
    for (i = 0; i < 60; i++) {
      unsigned short zero21c = 0;
      unsigned short txPhase, a3c, c28;

      memcpy(obj + 0x21c, &zero21c, 2);
      memcpy(obj + 0x110, &rxReadyVal, 2);
      memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
      ret = v8handshak(obj);
      memcpy(&txPhase, obj + 0x9d4, 2);
      memcpy(&a3c, obj + 0xa3c, 2);
      memcpy(&c28, obj + 0xc28, 2);
      printf("call %d: ret=%u tx=0x%x a3c=%d c28=%d\n", i, ret, txPhase, a3c, c28);
    }
  }

  /* E5/E6: same, but with capsByte bit 0x10 set at setup time (so the
   * ANSAM-detected transition ends at tx-phase 0x2b instead), run past the
   * 0x3c (60) baud periods tx-phase 0x2b's own loop counts before switching
   * itself to tx-phase 0x17 and swapping +0xc48 to point at +0xc54 (the
   * same slot tx-phase 0x17 alone would have used) - each "baud period" is
   * 4 calls (c28 advances by 4 per call, refilling the bit once it reaches
   * c26), so 0x3c periods is up to 0x3c*4=240 calls; run 250 to see the
   * transition land. */
  {
    unsigned short da0 = 500, c1c = 0, dd0 = 0, sub24 = 0x24;
    unsigned int ret;
    int i;
    make_case28_obj(obj, caps, 0);
    caps[2] = 0x10;
    memcpy(obj + 0x9d8, &sub24, 2);
    {
      unsigned short db6thresh = 0x960;
      memcpy(obj + 0xdb6, &db6thresh, 2);
    }
    memcpy(obj + 0xda0, &da0, 2);
    memcpy(obj + 0xc1c, &c1c, 2);
    memcpy(obj + 0xdd0, &dd0, 2);
    memcpy(obj + 0x110, &rxReadyVal, 2);
    memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
    ret = v8handshak(obj);
    dump_phase_rev("E5_ansam_detected_setup_tx0x2b", obj, ret);

    printf("== E6_tx0x2b_steady ==\n");
    for (i = 0; i < 250; i++) {
      unsigned short zero21c = 0;
      unsigned short txPhase, a3c, c28, dc0;
      char *c48Ptr;

      memcpy(obj + 0x21c, &zero21c, 2);
      memcpy(obj + 0x110, &rxReadyVal, 2);
      memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
      ret = v8handshak(obj);
      if (i < 235 ? (i % 40 == 0) : 1) {
        memcpy(&txPhase, obj + 0x9d4, 2);
        memcpy(&a3c, obj + 0xa3c, 2);
        memcpy(&c28, obj + 0xc28, 2);
        memcpy(&dc0, obj + 0xdc0, 2);
        V8H_PGET(c48Ptr, obj, 0xc48);
        printf("call %d: ret=%u tx=0x%x a3c=%d c28=%d dc0=%d c48off=0x%x\n",
               i, ret, txPhase, a3c, c28, dc0, (unsigned int)(c48Ptr - (char *)obj));
      }
    }
  }

  /* E7: tx-phase 0x2d's own plain cosine tone - direct poke, tx-phase 0x2d,
   * rxReady=0 so RX side never interferes, a few calls to see the phase
   * accumulator (+0xdd2/+0xdd4) advance and the TX buffer fill. +0xdd4
   * (the increment) is left at whatever v8handshakinit set it to (the
   * ANSWER-side init path never touches +0xdd2/+0xdd4 itself unless the
   * caps struct's 0x10 bit was set at init time - see v8handshakinit's own
   * comment - so this scenario sets capBit10=1 to get a real, non-zero
   * increment rather than a degenerate always-zero tone). */
  {
    unsigned short tx2d = 0x2d;
    unsigned short rxReadyZero = 0;
    unsigned int ret;
    int i;
    make_case28_obj(obj, caps, 1);
    memcpy(obj + 0x9d4, &tx2d, 2);

    printf("== E7_tx0x2d_steady ==\n");
    for (i = 0; i < 10; i++) {
      unsigned short zero21c = 0;
      unsigned short txBuf[4], dd2v;

      memcpy(obj + 0x21c, &zero21c, 2);
      memcpy(obj + 0x110, &rxReadyZero, 2);
      memcpy(obj + 0x5c8, rxSamp, sizeof(rxSamp));
      ret = v8handshak(obj);
      memcpy(txBuf, obj + 0x5c0, sizeof(txBuf));
      memcpy(&dd2v, obj + 0xdd2, 2);
      printf("call %d: ret=%u dd2=0x%x tx=%04x,%04x,%04x,%04x\n",
             i, ret, dd2v, (unsigned short)txBuf[0], (unsigned short)txBuf[1],
             (unsigned short)txBuf[2], (unsigned short)txBuf[3]);
    }
  }
}

int main(void)
{
  /* S0: sigTO=1 -> e5c = (1*0x2580)>>2 = 0x960 = 2400, so the ANSam-wait
   * timeout fires exactly at call index 2400 (0-based), transitioning
   * rx-phase 0x19 -> 0xb; calls after that exercise the rx-phase 0xb
   * early-return branch. rxReady=8 (>=6) every call. */
  run_scenario("S0_answer_sigTO1_timeout", 0, 1, 8, 2405);

  /* S1: rxReady=0 (<6) every call - the RX-phase dispatch never runs at
   * all; state should stay exactly at its post-init values (rx-phase/sub
   * both 0x19, e64=0, tx buffer always zero) for every call. */
  run_scenario("S1_answer_rxnotready", 0, 1, 0, 20);

  /* S2: sigTO=2 -> e5c=4800, only run 20 calls (far short of timeout) -
   * exercises the "still waiting, e64 incrementing, tone never detected"
   * steady loop without ever reaching the timeout transition. */
  run_scenario("S2_answer_sigTO2_steady", 0, 2, 8, 20);

  /* S3: CALLER side (obj+0xa44=1), sigTO=0 -> e5c=-1 (no CM timeout, so
   * tx-phase 6's not-yet-ported CM-timeout->rx-phase-4 branch is never
   * taken), rxReady=0 (<6) every call so the rx-phase dispatch never runs -
   * isolates tx-phase 6's CM tone generation on its own, independent of
   * rx-phase 0x20. */
  run_scenario("S3_caller_txonly", 1, 0, 0, 30);

  /* S4: CALLER side, sigTO=0, rxReady=8 every call, run up to AND INCLUDING
   * the rx-phase 0x20 AGC-settle counter's transition call (index 600,
   * 0-based; obj's +0xdb6 counter starts at 0 from v8handshakinit's
   * caller-side init and needs to exceed 0x258 to transition) - exercises
   * tx-phase 6 continuously together with rx-phase 0x20's counter/AGC tick
   * and its transition into rx-phase 0x28/sub-state 0x29. Stops at exactly
   * 601 calls (not one more): rx-phase 0x28 itself is NOT covered by this
   * port (only the transition INTO it, and the fields the transition
   * writes - +0x9d6/+0x9d8/+0x3c/+0xdb6 - are), so a 602nd call would hit
   * this port's deliberate abort() for an unimplemented rx-phase. */
  run_scenario("S4_caller_rx0x20_transition", 1, 0, 8, 601);

  run_rx0x28_natural();
  run_case28_pokes();
  run_case2c_pokes();
  run_generic_timeout_pokes();
  run_rx0x19_phase_reversal_pokes();
  run_case17_debug_string_check();
  run_case17_2b_and_rxphase4();

  return 0;
}
