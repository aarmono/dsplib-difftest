/*
 * Differential test harness for rebuildJMSequence/evaluateRxJMSequence -
 * the CI/CM/JM sequence-matching helpers v8handshak's rx-phase 0x28 sub-
 * state 0x28 completion body calls (see src/v8/v8.c's case-0x28 comment).
 * Both are globally-exported dsplibs.o symbols per `nm`, so - exactly like
 * test/v8/v8.main.main.c and test/fpm_diff_main.c - this harness calls them
 * DIRECTLY with a synthetic, harness-owned V.8 object buffer and synthetic
 * CM/JM message arrays + capabilities struct, rather than trying to drive
 * the full (mostly still-unported) handshake FSM to naturally reach the
 * point where either function gets called. This is a deliberate scope
 * choice: it fully tests these two functions' own logic (the actual thing
 * this pass ported), without needing to first resolve which physical buffer
 * genuinely holds live CM/JM octets at that point in the FSM's still-unported
 * surrounding states (rx-phase 0x24, tx-phase 0x17's V8GetMessage-based
 * receive path, etc. - left for whichever future pass hand-cleans those).
 *
 * Object field layout used here (byte offsets into the same ~0xec4-byte V.8
 * object every other V.8 test uses):
 *   +0xc54 (CM message octet array, rebuildJMSequence's fixed read source)
 *   +0xc7c (short, CM message length)
 *   +0xcd4 (JM message octet array, evaluateRxJMSequence's fixed read source)
 *   +0xcfc (short, JM message length)
 *   +0xa58 (pointer to the caller-owned capabilities struct - here, a
 *     harness-local static buffer, never itself compared since raw pointer
 *     values differ between the two binaries)
 *   +0xc48 (pointer to the outgoing JM reply's message slot - here, pointed
 *     at +0xc94 within the same object, far enough past +0xc54's CM data
 *     that even a full 40-entry CM message can't overlap it)
 *   +0xebc/+0xebe/+0xec0/+0xec2 (call-function/protocol match state -
 *     explicitly zeroed before each call, matching v8handshakinit's own
 *     initial zeroing of these fields)
 *
 * Capabilities struct layout (matches src/v8/v8.c's initTxSequence/
 * v8handshakinit comments - byte offsets into a small separate buffer):
 *   [0] bit3 ("mod"?), bit5, bit6; [1] bit0, bit6, bit7; [2] bits 0,1,2,3
 *     (raw-call-function flag), 8 ("digital connection"?)
 *   [0x18..] raw call-function ASCII string (NUL-terminated, <=4 chars)
 *   [0x1c..] raw protocol ASCII string (NUL-terminated, <=4 chars)
 *   [0x20..] call-function-range match string (<=8 chars)
 *   [0x28..] protocol-range match string (<=8 chars)
 *
 * Every scenario dumps the resulting object bytes at +0xc94.. (the JM reply
 * that got written) plus +0xebc/+0xebe/+0xec0/+0xec2 (the decided call-
 * function/protocol) - no raw pointers are ever printed.
 */

#include <stdio.h>
#include <string.h>
#include "v8_hobj.h"

extern unsigned char charFlip(unsigned char param_1);
extern void rebuildJMSequence(void *param_1);
extern void evaluateRxJMSequence(void *param_1);

#define V8OBJ_SIZE 0xee0

static void dump_range(const char *label, const unsigned char *obj, int off, int n)
{
  int i;
  printf("%s:", label);
  for (i = 0; i < n; i++) printf(" %02x", obj[off + i]);
  printf("\n");
}

static void dump_state(const char *label, const unsigned char *obj)
{
  short ebc, ebe, ec0, ec2;
  memcpy(&ebc, obj + 0xebc, 2);
  memcpy(&ebe, obj + 0xebe, 2);
  memcpy(&ec0, obj + 0xec0, 2);
  memcpy(&ec2, obj + 0xec2, 2);
  printf("%s: ebc=%d ebe=%d ec0=0x%x ec2=0x%x\n", label, ebc, ebe,
         (unsigned short)ec0, (unsigned short)ec2);
  dump_range(label, obj, 0xc94, 24);
}

static void put16(unsigned char *p, unsigned short v)
{
  memcpy(p, &v, 2);
}

static void setup_common(unsigned char *obj)
{
  memset(obj, 0, V8OBJ_SIZE);
}

/* --- rebuildJMSequence scenarios --- */

static void set_cm(unsigned char *obj, const unsigned short *entries, int n)
{
  int i;
  for (i = 0; i < n; i++) put16(obj + 0xc54 + i * 2, entries[i]);
  put16(obj + 0xc7c, (unsigned short)n);
}

static void run_rebuild(const char *label, const unsigned char *capb, int capb_len,
                         const unsigned short *cm, int cm_len)
{
  unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  unsigned char capbuf[0x40];
  unsigned char *capbptr;

  setup_common(obj);
  memset(capbuf, 0, sizeof(capbuf));
  memcpy(capbuf, capb, (size_t)capb_len);
  /* +0xa58 must hold a real pointer for rebuildJMSequence to dereference;
   * store it as an actual pointer (not compared directly - see dump_state,
   * which never reads +0xa58 back). */
  capbptr = capbuf;
  V8H_PSET(obj, 0xa58, capbptr);
  set_cm(obj, cm, cm_len);
  /* +0xc48 is self-relative (object base + 0xc94); store the real pointer
   * for rebuildJMSequence to use, matching how v8handshak's own +0xc48
   * field always holds a within-object pointer. */
  {
    unsigned char *slotptr = obj + 0xc94;
    V8H_PSET(obj, 0xc48, slotptr);
  }

  rebuildJMSequence(obj);
  dump_state(label, obj);
}

/* --- evaluateRxJMSequence scenarios --- */

static void run_evaluate(const char *label, const unsigned char *capb, int capb_len,
                          const unsigned short *jm, int jm_len)
{
  unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
  unsigned char capbuf[0x40];
  unsigned char *capbptr;
  int i;

  setup_common(obj);
  memset(capbuf, 0, sizeof(capbuf));
  memcpy(capbuf, capb, (size_t)capb_len);
  capbptr = capbuf;
  V8H_PSET(obj, 0xa58, capbptr);
  for (i = 0; i < jm_len; i++) put16(obj + 0xcd4 + i * 2, jm[i]);
  put16(obj + 0xcfc, (unsigned short)jm_len);

  evaluateRxJMSequence(obj);
  dump_state(label, obj);
}

int main(void)
{
  printf("== rebuildJMSequence ==\n");
  {
    /* R1: plain-bitfield call function (CM entry 0x109, capb[1] bit7 set ->
     * FAX-RX-to-caller match), no V.90 protocol entries at all -> falls to
     * the default-LAPM (0xa9) protocol fallback. */
    unsigned char capb[0x30] = {0};
    unsigned short cm[] = {0x109};
    capb[1] = 0x80;
    run_rebuild("R1_bitfield_faxrx_default_protocol", capb, sizeof(capb), cm, 1);
  }
  {
    /* R2: no call-function-group entry in CM at all -> "NO Call Function
     * Match" default zeroing path, then same default-protocol fallback. */
    unsigned char capb[0x30] = {0};
    unsigned short cm[] = {0x161};
    run_rebuild("R2_no_callfn_match", capb, sizeof(capb), cm, 1);
  }
  {
    /* R3: raw call-function string ("D") in capb[0x18], matching CM's raw-
     * encoded octet ((charFlip('D')&0xff)*2|1) with capb[2] bit2 set. */
    unsigned char capb[0x30] = {0};
    unsigned short cm[1];
    capb[2] = 0x04;
    capb[0x18] = 'D';
    cm[0] = (unsigned short)(((unsigned int)charFlip('D') & 0xff) * 2 | 1);
    run_rebuild("R3_raw_callfn_match", capb, sizeof(capb), cm, 1);
  }
  {
    /* R4: same raw-call-function setup as R3, but CM's raw octet does NOT
     * match capb's string -> falls through to the bitfield fallback path. */
    unsigned char capb[0x30] = {0};
    unsigned short cm[1];
    capb[2] = 0x04;
    capb[0x18] = 'D';
    capb[1] = 0x40; /* bitfield DATA-default fallback so a match still occurs downstream */
    cm[0] = (unsigned short)(((unsigned int)charFlip('X') & 0xff) * 2 | 1);
    run_rebuild("R4_raw_callfn_nomatch_fallback", capb, sizeof(capb), cm, 1);
  }
  {
    /* R5: protocol matched via the default-LAPM literal 0xa9 group entry,
     * no range string (capb[0x28]==0). */
    unsigned char capb[0x30] = {0};
    unsigned short cm[] = {0x109, 0xa9};
    capb[1] = 0x40;
    run_rebuild("R5_protocol_default_lapm_literal", capb, sizeof(capb), cm, 2);
  }
  {
    /* R6: protocol matched via the range-match string at capb[0x28] ("Q"),
     * CM's group entry raw-encodes 'Q' the same way the call-function range
     * string does. */
    unsigned char capb[0x30] = {0};
    unsigned short cm[2];
    capb[1] = 0x40;
    capb[0x28] = 'Q';
    cm[0] = 0x109;
    cm[1] = (unsigned short)(((unsigned int)charFlip('Q') & 0xff) * 2 | 1);
    run_rebuild("R6_protocol_range_match", capb, sizeof(capb), cm, 2);
  }
  {
    /* R7: V.90 "digital connection" path - capb[0] bit3 set, CM carries a
     * 0x141-masked mod-indicator entry (bit3 set) plus 0x161/0x1c1-masked
     * digital-connection/pcm entries selecting the QCA1 branch, and
     * capb[2] bit3 set so the protocol-range QCA1 search actually runs. */
    unsigned char capb[0x30] = {0};
    unsigned short cm[] = {0x109, 0x149, 0x163, 0x1c5};
    capb[1] = 0x40;
    capb[0] = 0x08;
    capb[2] = 0x08;
    capb[0x1c] = 'Z';
    run_rebuild("R7_v90_digital_connection", capb, sizeof(capb), cm, 4);
  }
  {
    /* R8: same V.90 bits as R7 but capb[2] bit3 clear -> the secondary
     * digital-connection test fails, falling back to LAB_000861b0. */
    unsigned char capb[0x30] = {0};
    unsigned short cm[] = {0x109, 0x149, 0x163, 0x1c5};
    capb[1] = 0x40;
    capb[0] = 0x08;
    run_rebuild("R8_v90_digital_connection_fallback", capb, sizeof(capb), cm, 4);
  }

  printf("== evaluateRxJMSequence ==\n");
  {
    /* E1: plain-bitfield call function (JM entry 0x107, capb[1] bit6 set ->
     * DATA match), protocol matched via literal 0xa9. */
    unsigned char capb[0x30] = {0};
    unsigned short jm[] = {0x107, 0xa9};
    capb[1] = 0x40;
    run_evaluate("E1_bitfield_data_default_protocol", capb, sizeof(capb), jm, 2);
  }
  {
    /* E2: no call-function-group entry, no protocol-group entry either. */
    unsigned char capb[0x30] = {0};
    unsigned short jm[] = {0x1c9};
    run_evaluate("E2_no_match_at_all", capb, sizeof(capb), jm, 1);
  }
  {
    /* E3: raw call-function string ("F") matching, capb[2] bit2 set. */
    unsigned char capb[0x30] = {0};
    unsigned short jm[1];
    capb[2] = 0x04;
    capb[0x18] = 'F';
    jm[0] = (unsigned short)(((unsigned int)charFlip('F') & 0xff) * 2 | 1);
    run_evaluate("E3_raw_callfn_match", capb, sizeof(capb), jm, 1);
  }
  {
    /* E4: protocol matched via range-match string at capb[0x1c] ("R"). */
    unsigned char capb[0x30] = {0};
    unsigned short jm[2];
    capb[1] = 0x40;
    capb[2] = 0x08;
    capb[0x1c] = 'R';
    jm[0] = 0x107;
    jm[1] = (unsigned short)(((unsigned int)charFlip('R') & 0xff) * 2 | 1);
    run_evaluate("E4_protocol_range_match", capb, sizeof(capb), jm, 2);
  }
  {
    /* E5: protocol group entry present but capb has neither bit2 set nor a
     * range string -> takes the "no protocol capability advertised at all"
     * immediate-match branch (any 0xa1-masked entry is accepted). */
    unsigned char capb[0x30] = {0};
    unsigned short jm[] = {0x107, 0xab};
    capb[1] = 0x40;
    run_evaluate("E5_protocol_immediate_accept", capb, sizeof(capb), jm, 2);
  }

  return 0;
}
