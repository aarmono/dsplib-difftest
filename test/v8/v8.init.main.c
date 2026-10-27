/*
 * Differential test harness for v8handshakinit + initTxSequence (the V.8
 * handshake object's one-time setup path - see src/v8/v8.c's "handshake init
 * path" section header comment for exactly what this covers and what's
 * still NOT ported: v8handshak itself, rebuildJMSequence/
 * evaluateRxJMSequence, V8UpdateModemParameters, and the object-lifecycle
 * glue).
 *
 * Same pattern as test/v8/v8.main.main.c: this ONE source is compiled into two
 * binaries - v8_init_test_blob (linked against the real dsplibs.o, whose
 * v8handshakinit is a globally exported symbol per `nm ../dsplibs.o`) and
 * v8_init_test_port (linked against ../src/v8/v8.c + ../src/v8/v8_tables.c).
 *
 * v8handshakinit is called directly on a harness-owned buffer standing in
 * for the object V8Create allocates via `sysdep_malloc(0xec4)` (V8OBJ_SIZE,
 * shared with test/v8/v8.main.main.c's convention, is slightly larger for
 * margin). Since V8Create does NOT memset this buffer before writing its own
 * handful of fields and calling v8handshakinit (confirmed by reading
 * porting/decompiled/dp/v8.c's V8Create - no sysdep_memset call appears), and
 * sysdep_malloc's real backing allocator is not part of what's under test,
 * this harness fills the buffer with a fixed, non-zero, non-canonical byte
 * pattern (0x55) before every scenario - the same "catch a missed field" tactic
 * already used for B103/V23's cfg-construction differential tests and for
 * v8_phase_rev_init's own bug in the earlier V.8 toolkit pass.
 *
 * Exactly replicates V8Create's own field writes ahead of its
 * v8handshakinit(iVar2) call (obj+0xa44/0xa48/0xa4c/0xa50/0xa54/0xa58/0xa42 -
 * see porting/decompiled/dp/v8.c's V8Create), so this test's obj+0xa58 "capabilities"
 * pointer is a harness-owned, synthetic byte struct - NOT slmodemd's real
 * one (dsplibs.o never exposes that struct's field meanings to itself
 * either, and V8Create treats it as pure caller-owned memory, so a synthetic
 * struct is exactly as meaningful as a real one for a bit-exact behavioral
 * comparison - same rationale as every other cfg-word differential test in
 * this project).
 *
 * IMPORTANT: obj+0xc48/+0xc4c/+0xc50 hold RAW POINTERS into the same buffer
 * (v8handshakinit points them at fixed offsets within the object itself).
 * Never diff these as raw bytes - the buffer's own absolute address differs
 * between the two separately-linked binaries. Instead: if the stored value
 * falls inside [obj, obj+V8OBJ_SIZE), print it as an OFFSET from obj (a
 * self-relative value, comparable across binaries); otherwise (the field was
 * never written this scenario) print the raw 4 bytes, which - since the fill
 * pattern is a fixed compile-time constant, not an address - IS directly
 * comparable across binaries. All other bytes of the object are dumped
 * verbatim as hex.
 */

#include <stdio.h>
#include <string.h>
#include "v8_hobj.h"

extern unsigned int v8handshakinit(void *param_1);

/* Big enough for every byte offset v8handshakinit/initTxSequence touch
 * (up to +0xec2) - matches test/v8/v8.main.main.c's V8OBJ_SIZE convention. */
#define V8OBJ_SIZE 0xee0

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

/* Every self-relative pointer field v8handshakinit's own writes (0xc48/
 * 0xc4c/0xc50) OR the already-verified v8_rxinit/v8_txinit it calls (0x2c,
 * 0x114/0x118, 0x220/0x224 - see src/v8/v8.c's v8_rxinit/v8_txinit) can store
 * into this object. All of them must be excluded from the raw hex dump and
 * compared as self-relative offsets instead - see this file's header
 * comment. */
static const int ptr_field_offsets[] = {0x2c, 0x114, 0x118, 0x220, 0x224, 0xc48, 0xc4c, 0xc50};

/* Pointers into DATA that legitimately lives at a DIFFERENT address in the
 * two separately-linked binaries even when the port is 100% correct: the
 * harness's own synthetic capabilities struct (0xa58, a stack address in
 * this process, set by the harness itself and never written by the code
 * under test), v8_detectorinit's coefficient-table pointer field (0xad8 -
 * see src/v8/v8.c's v8_detectorinit comment: "+0x00 (4B): coefficient-array
 * pointer" - points at v8_c2100 in the port vs. dsplibs.o's own internal
 * `c2100` symbol, two separate copies of identical data at different
 * addresses), and V8_setFilters' 4 V.21 filter-table pointers (0xdd8/0xddc/
 * 0xde0/0xde4, written via v8_V21_Init on the caller/CM side - see
 * src/v8/v8.c's V8_setFilters call site). None of these are self-relative, so
 * there is nothing meaningful to compare for them at all - exclude from the
 * dump entirely (zero, don't report), unlike the self-relative ones above. */
static const int external_ptr_field_offsets[] = {0xa58, 0xad8, 0xdd8, 0xddc, 0xde0, 0xde4};

static void dump_obj(unsigned char *obj)
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
  printf("bytes:");
  for (i = 0; i < V8OBJ_SIZE; i++) printf("%02x", scratch[i]);
  printf("\n");
  for (i = 0; i < sizeof(ptr_field_offsets) / sizeof(ptr_field_offsets[0]); i++) {
    char label[16];
    sprintf(label, "%x", ptr_field_offsets[i]);
    print_ptr_field(label, obj, ptr_field_offsets[i]);
  }
}

/* Builds a synthetic 32-byte capabilities struct covering every bit/byte
 * v8handshakinit + initTxSequence read from obj+0xa58 (see src/v8/v8.c's
 * header comment for the full bit-to-offset map this was derived from):
 *   dword (bytes 0-3): CF_* bits below
 *   word10 (bytes 0x10-0x13): the "raw CI/CM control word" v8handshakinit's
 *     own special branch reads (as both a dword and byte 0x10's bit0)
 *   rawcallfn: up to 4 chars written at +0x18 (NUL-terminated / zero-padded)
 *   rawproto:  up to 4 chars written at +0x1c (NUL-terminated / zero-padded)
 */
#define CF_RAWCALLFN    0x40000u /* byte2 bit2 */
#define CF_RAWPROTO     0x80000u /* byte2 bit3 */
#define CF_CALLFN_4000  0x4000u  /* byte1 bit6 */
#define CF_CALLFN_8000  0x8000u  /* byte1 bit7 */
#define CF_CALLFN_10000 0x10000u /* byte2 bit0 */
#define CF_CALLFN_20000 0x20000u /* byte2 bit1 */
#define CF_BIT3         0x8u     /* byte0 bit3 */
#define CF_BIT5         0x20u    /* byte0 bit5 */
#define CF_BIT6         0x40u    /* byte0 bit6 */
#define CF_SIGNBIT      0x80u    /* byte0 bit7 */
#define CF_BIT8         0x100u   /* byte1 bit0 */
#define CF_BIT9         0x200u   /* byte1 bit1 */
#define CF_BIT10        0x400u   /* byte1 bit2 */
#define CF_BIT11        0x800u   /* byte1 bit3 */
#define CF_BYTE1_BIT4   0x1000u  /* byte1 bit4 */
#define CF_BYTE1_BIT5   0x2000u  /* byte1 bit5 */
#define CF_RAWCI        0x100000u /* byte2 bit4 - v8handshakinit's own special branch */
#define CF_RAWCI_ALT    0x400000u /* byte2 bit6 - inner control-word branch */

static void build_caps(unsigned char *caps, unsigned int dword, unsigned int word10,
                        const char *rawcallfn, const char *rawproto)
{
  size_t n;
  memset(caps, 0, 32);
  memcpy(caps, &dword, 4);
  memcpy(caps + 0x10, &word10, 4);
  if (rawcallfn) {
    n = strlen(rawcallfn);
    if (n > 4) n = 4;
    memcpy(caps + 0x18, rawcallfn, n);
  }
  if (rawproto) {
    n = strlen(rawproto);
    if (n > 4) n = 4;
    memcpy(caps + 0x1c, rawproto, n);
  }
}

typedef struct {
  const char *label;
  int side, opMode, sigTO, msgTO;
  unsigned int dword, word10;
  const char *rawcallfn, *rawproto;
} scenario_t;

static const scenario_t scenarios[] = {
  {"S00_answer_allzero_defaults", 0, 0, 0, 0, 0, 0, NULL, NULL},
  {"S01_answer_rawcallfn_novaliddata", 0, 1, 5, 3, CF_RAWCALLFN, 0, NULL, NULL},
  {"S02_answer_rawcallfn_valid2char", 0, 1, 5, 3, CF_RAWCALLFN, 0, "XY", NULL},
  {"S03_answer_rawcallfn_valid4char", 0, 0, -1, -1, CF_RAWCALLFN, 0, "WXYZ", NULL},
  {"S04_answer_callfn_4000", 0, 0, 0, 0, CF_CALLFN_4000, 0, NULL, NULL},
  {"S05_answer_callfn_8000", 0, 0, 0, 0, CF_CALLFN_8000, 0, NULL, NULL},
  {"S06_answer_callfn_10000", 0, 0, 0, 0, CF_CALLFN_10000, 0, NULL, NULL},
  {"S07_answer_callfn_20000", 0, 0, 0, 0, CF_CALLFN_20000, 0, NULL, NULL},
  {"S08_answer_bits_3_5_6_7", 0, 2, 7, 9,
   CF_BIT3 | CF_BIT5 | CF_BIT6 | CF_SIGNBIT, 0, NULL, NULL},
  {"S09_answer_bits_8_9_10_11", 0, 2, 7, 9, CF_BIT8 | CF_BIT9 | CF_BIT10 | CF_BIT11, 0, NULL, NULL},
  {"S10_answer_byte1_bits_4_5", 0, 0, 0, 0, CF_BYTE1_BIT4 | CF_BYTE1_BIT5, 0, NULL, NULL},
  {"S11_answer_rawproto_novaliddata", 0, 0, 0, 0, CF_RAWPROTO, 0, NULL, NULL},
  {"S12_answer_rawproto_valid3char", 0, 0, 0, 0, CF_RAWPROTO, 0, NULL, "abc"},
  {"S13_answer_rawproto_plus_bit3_trailer", 0, 0, 0, 0, CF_RAWPROTO | CF_BIT3, 0, NULL, "pqrs"},
  {"S14_answer_rawci_altmask_set", 0, 0, 0, 0, CF_RAWCI | CF_RAWCI_ALT, 0x00000001u, NULL, NULL},
  {"S15_answer_rawci_altmask_clear", 0, 0, 0, 0, CF_RAWCI, 0x00000000u, NULL, NULL},
  {"S16_answer_rawci_word10_bit0", 0, 0, 0, 0, CF_RAWCI, 0x000000ffu, NULL, NULL},
  {"S17_answer_everything_combined", 0, 3, 11, 13,
   CF_RAWCALLFN | CF_BIT3 | CF_BIT5 | CF_BIT6 | CF_SIGNBIT | CF_BIT8 | CF_BIT9 | CF_BIT10 |
       CF_BIT11 | CF_BYTE1_BIT4 | CF_BYTE1_BIT5 | CF_RAWPROTO | CF_RAWCI | CF_RAWCI_ALT,
   0x12345678u, "QR", "ST"},
  {"S18_caller_defaults", 1, 0, 0, 0, 0, 0, NULL, NULL},
  {"S19_caller_positive_timeouts", 1, 4, 6, 8, 0, 0, NULL, NULL},
  {"S20_caller_bits_set", 1, 1, 2, 3, CF_CALLFN_8000 | CF_BIT9 | CF_BIT11, 0, NULL, NULL},
  {"S21_caller_rawcallfn_and_proto", 1, 0, 0, 0, CF_RAWCALLFN | CF_RAWPROTO, 0, "AB", "CD"},
  {"S22_noop_side2", 2, 0, 0, 0, 0, 0, NULL, NULL},
  {"S23_noop_side_neg1", -1, 0, 0, 0, 0, 0, NULL, NULL},
};

int main(void)
{
  unsigned int si;
  for (si = 0; si < sizeof(scenarios) / sizeof(scenarios[0]); si++) {
    const scenario_t *s = &scenarios[si];
    unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;
    unsigned char caps[32];

    memset(obj, 0x55, sizeof(obj));
    build_caps(caps, s->dword, s->word10, s->rawcallfn, s->rawproto);

    /* Exactly mirrors V8Create's own writes ahead of its v8handshakinit()
     * call - see porting/decompiled/dp/v8.c. */
    memcpy(obj + 0xa44, &s->side, 4);
    memcpy(obj + 0xa48, &s->opMode, 4);
    memcpy(obj + 0xa4c, &s->sigTO, 4);
    memcpy(obj + 0xa50, &s->msgTO, 4);
    memset(obj + 0xa54, 0, 4);
    {
      unsigned char *capsPtr = caps;
      V8H_PSET(obj, 0xa58, capsPtr);
    }
    {
      unsigned short a42 = 0x4000;
      memcpy(obj + 0xa42, &a42, 2);
    }

    printf("== %s ==\n", s->label);
    v8handshakinit(obj);
    dump_obj(obj);
    printf("caps_after:");
    {
      int i;
      for (i = 0; i < 32; i++) printf("%02x", caps[i]);
    }
    printf("\n");
  }
  return 0;
}
