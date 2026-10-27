/*
 * Differential test harness for port chunk P10 (src/vpcm/v90_cp.cpp): class V90CP of dsplibs.o - the V.90 phase-4 CP frame holder / bit-serial codec
 * (15 functions: C1 C2 D1 D2, resetCRC calcCRC evaluateCRC resetDetector reset getBitVector evaluateInfo calcSequenceLength infoToBits bitsToInfo
 * printNofRecievedMpMpNot).
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/v90_cp.sh against the blob (dsplibs.o itself: `dsplibs_debug_printf` and the allocator are
 * the harness'), against the port (-O0 / -O2) and for x86-64 (diffed against the 32-bit BLOB's output = the golden, test/golden/vpcm/v90_cp.blob32.out).  The functions are reached
 * through __asm__ labels with the mangled names (identical in blob and port, C1 AND C2 / D1 AND D2); the object layout is a C mirror struct GENERATED from test/vpcm/v90_cp.layout
 * (REAL pointer members: the six owned heap blocks widen on x86-64, every offset behind them moves; everything is compared by member name / through the mirror).
 *
 * What is compared: every return value, the object after every call (a light hash of the head fields per call, the FULL arch-independent hash - head, buffer, CRC, tail and
 * the six heap blocks - at the ends of frames and every 64th call, the field-wise dump sampled), the malloc / free sequence, every `dsplibs_debug_printf` call (text; recorder
 * with a hook that lowers dsplibs_debug_level between the prints).  INDEPENDENT MODELS (written from the frame definition: the V.34 style 17-bit start-bit grid, field order
 * and widths, a standard bit-serial CRC-CCITT, the cursor arithmetic of the decoder as relative offsets), exit 3 on a violation: the encoder model (info -> expected bit
 * positions, CRC, lengths), the decoder model (buffer -> info for every part, with the cursor), the CRC model, the end-to-end model (object A encodes, object B receives bit by
 * bit: the decoded info must equal the sent info and the return codes follow the frame type and the suppression window).
 *
 * Scenarios (each its own process): 0 CRC functions vs the CRC model, 1 evaluateInfo case by case vs the decoder model, 2 infoToBits vs the encoder model (+ round trip),
 * 3 bitsToInfo: model-built frames end to end, damaged frames, noise, Ed, suppression window, level steering, 4 bitsToInfo on fully random object states (every case of the
 * jump table, the default cases, the buffer-overflow message), 5 constructors / destructor (malloc log, NULL blocks) / reset / resetDetector / getBitVector /
 * calcSequenceLength / printNofRecievedMpMpNot.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "v90_cp_layout.h"

#define L(m) __asm__(m)

extern void cp_c1(void *) L("_ZN5V90CPC1Ev");
extern void cp_c2(void *) L("_ZN5V90CPC2Ev");
extern void cp_d1(void *) L("_ZN5V90CPD1Ev");
extern void cp_d2(void *) L("_ZN5V90CPD2Ev");
extern void cp_resetcrc(void *) L("_ZN5V90CP8resetCRCEv");
extern void cp_calccrc(void *) L("_ZN5V90CP7calcCRCEv");
extern int cp_evalcrc(void *) L("_ZN5V90CP11evaluateCRCEv");
extern void cp_resetdet(void *) L("_ZN5V90CP13resetDetectorEv");
extern void cp_reset(void *) L("_ZN5V90CP5resetEv");
extern uint8_t *cp_getbv(void *, unsigned *) L("_ZN5V90CP12getBitVectorERj");
extern void cp_evalinfo(void *) L("_ZN5V90CP12evaluateInfoEv");
extern void cp_calcseq(void *) L("_ZN5V90CP18calcSequenceLengthEv");
extern void cp_i2b(void *) L("_ZN5V90CP10infoToBitsEv");
extern int cp_b2i(void *, unsigned char) L("_ZN5V90CP10bitsToInfoEh");
extern void cp_prnof(void *) L("_ZN5V90CP23printNofRecievedMpMpNotEv");
extern unsigned int dsplibs_debug_level;

#define BUFSZ 12000u
#define MAXBITS 13000

/* ------------------------------------------------------------------------------------------------ recorder (dsplibs_debug_printf) */
static unsigned g_ndbg;
static char g_dbg_text[64][300];
static unsigned g_dbg_in_call;
static int g_dbg_show;
typedef void (*dbg_hook_t)(unsigned idx);
static dbg_hook_t g_dbg_hook;
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (g_dbg_in_call < 64) { g_dbg_text[g_dbg_in_call][0] = 0; esc_cat(g_dbg_text[g_dbg_in_call], 300, buf); }
  h_emit(g_dbg_show, "dbgprintf %s", g_dbg_in_call < 64 ? g_dbg_text[g_dbg_in_call] : "(many)");
  g_dbg_in_call++;
  g_ndbg++;
  if (g_dbg_hook) g_dbg_hook(g_dbg_in_call - 1);
  return 0;
}

static void *onew(size_t n) { void *p = h_obj_new(n); memset(p, 0xa5, n); return p; }
static uint64_t fnv_upd(uint64_t h, const void *p, size_t n) {
  const uint8_t *q = (const uint8_t *)p;
  size_t i;
  for (i = 0; i < n; i++) { h ^= q[i]; h *= 0x100000001b3ull; }
  return h;
}
#define FNV0 0xcbf29ce484222325ull
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)

static const char *g_covname[H_COV_MAX];
static void cov_def(int id, const char *name) { h_cov_define(id, name); g_covname[id] = name; }
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }
#define COV(id) h_cov_hit(id)

/* ------------------------------------------------------------------------------------------------ object access / canonical dump / hashes */
static CpObj *CO(void *o) { return (CpObj *)o; }
static uint8_t *BITS(void *o) { return CO(o)->bits_; }

/* the dump table: the generated one with the big arrays turned into hashed 'x' regions and the heap pointer array expanded into six owners */
static HFld g_dumpf[80];
static int g_ndumpf;
static char g_names[8][16];
static void build_dump_table(void) {
  int i, k, n = 0;
  for (i = 0; i < NF_CpObj; i++) {
    const HFld *f = &f_CpObj[i];
    if (f->kind == 'o' && f->count > 1) {
      for (k = 0; k < (int)f->count; k++) {
        HFld e;
        snprintf(g_names[k], sizeof g_names[k], "heap%d_", k);
        e.name = g_names[k]; e.off = f->off + (size_t)k * sizeof(void *); e.kind = 'o'; e.count = 0;
        g_dumpf[n++] = e;
      }
    } else if ((f->kind == 'b' && f->count > 64) || (f->kind == 'h' && f->count > 32)) {
      HFld e = *f;
      e.kind = 'x';
      e.count = f->count * (f->kind == 'h' ? 2u : 1u);
      g_dumpf[n++] = e;
    } else {
      g_dumpf[n++] = *f;
    }
  }
  g_ndumpf = n;
}
static unsigned long g_dumpctr;
#define DUMP(tag, obj) h_dump_obj(h_sample(&g_dumpctr, 12, 211), (tag), (obj), g_dumpf, g_ndumpf)

/* arch-independent hashes: the head (everything before the pointer array), the buffer + CRC + tail (everything after it) and the six heap blocks */
static uint64_t cp_hash(const void *o, int full) {
  const uint8_t *p = (const uint8_t *)o;
  size_t ho = offsetof(CpObj, heap_), after = ho + 6 * sizeof(void *);
  uint64_t h = FNV0;
  int k;
  h = fnv_upd(h, p, ho);
  if (full) {
    h = fnv_upd(h, p + after, sizeof(CpObj) - after);
    for (k = 0; k < 6; k++) {
      const uint8_t *blk;
      memcpy(&blk, p + ho + (size_t)k * sizeof(void *), sizeof blk);
      if (blk) h = fnv_upd(h, blk, 0x200);
    }
  } else {
    h = fnv_upd(h, p + after, 0x1b8 - 0xb8 + 0x40);   /* the first 0x100 bytes of the buffer are enough for a light per-call hash */
    /* ... plus the three bytes the state machine writes next: the buffer at widx_ */
    { uint32_t w; memcpy(&w, p + offsetof(CpObj, widx_), 4); if (w < BUFSZ) h = fnv_upd(h, p + offsetof(CpObj, bits_) + w, 1); }
    h = fnv_upd(h, p + offsetof(CpObj, crc_), 16);
  }
  return h;
}

/* ------------------------------------------------------------------------------------------------ the info as a value (what the frame carries) */
typedef struct {
  uint32_t type, fa, fb, fc;
  uint8_t f10, f11, f12, f13;
  uint32_t w14, wCa0;
  uint32_t pair[12], cnt[4], hcnt[6], nib[6];
  uint16_t arr[4][384];
  uint32_t heap[6][128];
} Info;

static void get_info(const CpObj *o, Info *in) {
  int g, k;
  memset(in, 0, sizeof *in);
  in->type = o->type_; in->fa = o->flagA_; in->fb = o->flagB_; in->fc = o->flagC_;
  in->f10 = o->f10_; in->f11 = o->f11_; in->f12 = o->f12_; in->f13 = o->f13_; in->w14 = o->w14_; in->wCa0 = o->wCa0_;
  memcpy(in->pair, o->pair_, sizeof in->pair); memcpy(in->cnt, o->cnt_, sizeof in->cnt); memcpy(in->hcnt, o->hcnt_, sizeof in->hcnt); memcpy(in->nib, o->nib_, sizeof in->nib);
  for (g = 0; g < 4; g++) memcpy(in->arr[g], o->arrs_ + 384 * g, sizeof in->arr[g]);
  for (g = 0; g < 6; g++) {
    const uint32_t *h = (const uint32_t *)o->heap_[g];
    if (h) for (k = 0; k < 128; k++) in->heap[g][k] = h[k];
  }
}
static void set_info(CpObj *o, const Info *in) {
  int g, k;
  o->type_ = in->type; o->flagA_ = in->fa; o->flagB_ = in->fb; o->flagC_ = in->fc;
  o->f10_ = in->f10; o->f11_ = in->f11; o->f12_ = in->f12; o->f13_ = in->f13; o->w14_ = in->w14; o->wCa0_ = in->wCa0;
  memcpy(o->pair_, in->pair, sizeof in->pair); memcpy(o->cnt_, in->cnt, sizeof in->cnt); memcpy(o->hcnt_, in->hcnt, sizeof in->hcnt); memcpy(o->nib_, in->nib, sizeof in->nib);
  for (g = 0; g < 4; g++) memcpy(o->arrs_ + 384 * g, in->arr[g], sizeof in->arr[g]);
  for (g = 0; g < 6; g++) {
    uint32_t *h = (uint32_t *)o->heap_[g];
    if (h) for (k = 0; k < 128; k++) h[k] = in->heap[g][k];
  }
}

/* random legitimate info: short or long frame, small counts (a long frame stays well inside the 12,000-bit buffer) */
static uint32_t r16(void) { return h_rnd() & 0xffffu; }
static void rnd_info(Info *in, int legit, int want_long, unsigned budget) {
  int g, k;
  unsigned left = budget;
  memset(in, 0, sizeof *in);
  if (want_long < 0) want_long = h_rpct(75);
  if (!want_long) {
    in->type = 1;
    in->wCa0 = (uint8_t)(legit ? h_rmod(2) : h_rnd());
    in->f13 = (uint8_t)(legit ? h_rmod(2) : h_rnd());
    in->f12 = (uint8_t)h_rnd();
    in->w14 = r16();
    return;
  }
  in->type = 0;
  in->fa = h_rpct(60); in->fb = h_rpct(60); in->fc = h_rpct(60);
  in->f10 = (uint8_t)h_rmod(32);
  in->f11 = (uint8_t)h_rmod(4);
  in->f12 = (uint8_t)(legit ? h_rmod(2) : h_rnd());
  in->f13 = (uint8_t)(legit ? h_rmod(2) : h_rnd());
  in->w14 = r16();
  for (k = 0; k < 12; k++) in->pair[k] = h_rmod(256);
  for (k = 0; k < 6; k++) in->nib[k] = h_rmod(16);
  for (g = 0; g < 4; g++) {
    unsigned n = h_rpct(30) ? 0 : h_rmod(1 + (left < 40 ? left : 40));
    if (n > left) n = left;
    left -= n;
    in->cnt[g] = n;
    for (k = 0; k < (int)n; k++) in->arr[g][k] = (uint16_t)r16();
  }
  for (g = 0; g < 6; g++) {
    unsigned n = h_rpct(30) ? 0 : h_rmod(1 + (left < 30 ? left : 30));
    if (n > left) n = left;
    left -= n;
    in->hcnt[g] = n;
    for (k = 0; k < (int)n; k++) in->heap[g][k] = r16();
  }
}

/* ------------------------------------------------------------------------------------------------ models */
/* encoder model: bit positions of the frame, with the 17-bit start-bit grid written by the writer itself */
typedef struct { uint8_t exp[MAXBITS]; uint8_t spec[MAXBITS]; uint8_t sflag[MAXBITS]; unsigned total, frameLen, C, pos, mark; } Mdl;
static void mput(Mdl *m, unsigned p, unsigned v) { m->exp[p] = (uint8_t)v; m->spec[p] = 1; }
static void w_bit(Mdl *m, unsigned v) {
  if (m->pos % 17 == 0) { mput(m, m->pos, 0); m->pos++; }
  mput(m, m->pos, v);
  m->sflag[m->pos] = (uint8_t)m->mark;
  m->pos++;
}
static void w_val(Mdl *m, uint32_t v, int n) { int i; for (i = 0; i < n; i++) w_bit(m, (v >> i) & 1u); }

static void model_crc(const uint8_t *b, unsigned endp, uint8_t out[16]) {
  uint16_t r = 0xffff;
  unsigned p, k;
  for (p = 0x12; p < endp; p++) {
    unsigned fb;
    if (p % 17 == 0) continue;
    fb = ((r >> 15) ^ (b[p] & 1u)) & 1u;
    r = (uint16_t)(r << 1);
    if (fb) r ^= 0x1021;
  }
  for (k = 0; k < 16; k++) out[k] = (uint8_t)((r >> (15 - k)) & 1);
}

/* pre: the buffer before infoToBits (positions the blob leaves alone keep it; the f11 bits 0x1b / 0x1c are left alone for f11 > 3 and the CRC covers them) */
static void model_encode(const Info *in, unsigned D, const uint8_t *pre, Mdl *m) {
  unsigned p, k, g, v, q, C;
  uint8_t crc[16];
  memset(m, 0, sizeof *m);
  for (p = 0; p <= 16; p++) mput(m, p, 1);
  mput(m, 17, 0);
  mput(m, 0x12, in->type & 0xffu);
  m->sflag[0x12] = 1;
  if (in->type != 0) {
    for (p = 0x13; p <= 0x1f; p++) mput(m, p, 0);
    mput(m, 0x20, in->wCa0 & 0xffu);
    mput(m, 0x21, in->f13);
    C = 0x22;
  } else {
    mput(m, 0x13, in->fa & 0xffu); mput(m, 0x14, in->fb & 0xffu); mput(m, 0x15, in->fc & 0xffu);
    m->sflag[0x13] = m->sflag[0x14] = m->sflag[0x15] = 1;
    for (k = 0; k < 5; k++) mput(m, 0x16 + k, (in->f10 >> k) & 1u);
    if (in->f11 <= 3) { mput(m, 0x1b, in->f11 & 1u); mput(m, 0x1c, (in->f11 >> 1) & 1u); }
    else if (pre) { m->exp[0x1b] = pre[0x1b]; m->exp[0x1c] = pre[0x1c]; }
    mput(m, 0x1d, in->f12);
    mput(m, 0x1e, 0); mput(m, 0x1f, 0); mput(m, 0x20, 0);
    mput(m, 0x21, in->f13);
    mput(m, 0x22, 0);
    for (k = 0; k < 16; k++) mput(m, 0x23 + k, (in->w14 >> k) & 1u);
    m->pos = 0x33;
    if (in->fa) for (g = 0; g < 12; g++) w_val(m, in->pair[g], 8);
    if (in->fb) {
      for (g = 0; g < 4; g++) { m->mark = 1; w_val(m, in->cnt[g], 9); m->mark = 0; for (k = 0; k < 7; k++) w_bit(m, 0); }
      for (g = 0; g < 4; g++) for (k = 0; k < in->cnt[g]; k++) w_val(m, in->arr[g][k], 16);
    }
    if (in->fc) {
      for (k = 0; k < 6; k++) w_val(m, in->nib[k], 4);
      for (k = 0; k < 8; k++) w_bit(m, 0);
      for (g = 0; g < 6; g++) { m->mark = 1; w_val(m, in->hcnt[g], 8); m->mark = 0; }
      for (g = 0; g < 6; g++) for (k = 0; k < in->hcnt[g]; k++) w_val(m, in->heap[g][k], 16);
    }
    C = m->pos;
  }
  if (C % 17 != 0) h_die3("model: payload ends off the grid (%u)", C);
  m->C = C;
  mput(m, C, 0);
  model_crc(m->exp, C, crc);
  for (k = 0; k < 16; k++) mput(m, C + 1 + k, crc[k]);
  mput(m, C + 17, 0);
  m->frameLen = C + 17;
  v = C + 0x12;
  q = v / D;
  m->total = (q * D == v) ? v : (q + 1) * D;
  for (p = C + 18; p < m->total; p++) mput(m, p, 0);
}

static uint32_t rd(const uint8_t *b, unsigned a, int n) { uint32_t v = 0; int i; for (i = 0; i < n; i++) v |= (uint32_t)(b[a + (unsigned)i] & 1u) << i; return v; }

/* the decoder model: what evaluateInfo does in `state` with the cursor c0 on `info` (updated in place); returns the expected cursor */
static uint32_t model_eval(Info *in, const uint8_t *b, uint32_t state, uint32_t c0, uint32_t cursor_in) {
  unsigned g, k, e = 0, j;
  (void)cursor_in;
  switch (state) {
    case 3: in->wCa0 = b[0x20]; in->f13 = b[0x21]; return c0;
    case 5:
      in->f10 = (uint8_t)rd(b, 0x16, 5); in->f12 = b[0x1d]; in->f11 = (uint8_t)rd(b, 0x1b, 2); in->f13 = b[0x21]; in->w14 = rd(b, 0x23, 16);
      return 0x32;
    case 6:
      for (g = 0; g < 6; g++) { in->pair[2 * g] = rd(b, c0 + 2 + 17 * g, 8); in->pair[2 * g + 1] = rd(b, c0 + 10 + 17 * g, 8); }
      return 0x98;
    case 7:
      for (g = 0; g < 4; g++) in->cnt[g] = rd(b, c0 + 2 + 17 * g, 9);
      return c0 + 0x44;
    case 8:
      for (g = 0; g < 4; g++) for (k = 0; k < in->cnt[g]; k++) { in->arr[g][k] = (uint16_t)rd(b, c0 + 2 + 17 * e, 16); e++; }
      return c0 + 17 * e;
    case 10: {
      unsigned X = c0 + 1;
      for (k = 0; k < 4; k++) in->nib[k] = rd(b, X + 1 + 4 * k, 4);
      for (k = 4; k < 6; k++) in->nib[k] = rd(b, X + 18 + 4 * (k - 4), 4);
      for (g = 0; g < 3; g++) { in->hcnt[2 * g] = rd(b, X + 35 + 17 * g, 8); in->hcnt[2 * g + 1] = rd(b, X + 43 + 17 * g, 8); }
      return X + 84;
    }
    case 11:
      for (j = 0; j < 6; j++) for (k = 0; k < in->hcnt[j]; k++) { in->heap[j][k] = rd(b, c0 + 2 + 17 * e, 16); e++; }
      return c0 + 17 * e;
    default: return c0;
  }
}

/* info comparison: only what the frame carries (arrays up to their counts) */
static int info_same(const Info *a, const Info *b, int long_frame, const char **why) {
  unsigned g, k;
#define NE(x) if (a->x != b->x) { *why = #x; return 0; }
  NE(type) NE(f13)
  if (!long_frame) { NE(wCa0) return 1; }
  NE(fa) NE(fb) NE(fc) NE(f10) NE(f11) NE(f12) NE(w14)
  if (a->fa) for (k = 0; k < 12; k++) if (a->pair[k] != b->pair[k]) { *why = "pair"; return 0; }
  if (a->fb) {
    for (k = 0; k < 4; k++) if (a->cnt[k] != b->cnt[k]) { *why = "cnt"; return 0; }
    for (g = 0; g < 4; g++) for (k = 0; k < a->cnt[g]; k++) if (a->arr[g][k] != b->arr[g][k]) { *why = "arr"; return 0; }
  }
  if (a->fc) {
    for (k = 0; k < 6; k++) if (a->nib[k] != b->nib[k] || a->hcnt[k] != b->hcnt[k]) { *why = "nib/hcnt"; return 0; }
    for (g = 0; g < 6; g++) for (k = 0; k < a->hcnt[g]; k++) if (a->heap[g][k] != b->heap[g][k]) { *why = "heap"; return 0; }
  }
  return 1;
#undef NE
}

static uint8_t rbyte_kind(void) {
  switch (h_rmod(8)) {
    case 0: return 0;
    case 1: return 1;
    case 2: return 0xff;
    case 3: return 0x80;
    case 4: return (uint8_t)(1 + h_rmod(3));
    default: return (uint8_t)h_rnd();
  }
}

/* object lifecycle: construct through the blob / port constructor (six malloc(0x200)), destroy through the destructor, end the trial */
/* garbage fill, a function of (seed, LOGICAL offset): the head before the six heap pointers and the tail after them are filled independently, so the same bytes land in the same
 * fields on i386 and x86-64 (the pointer array is 24 bytes bigger there), and the number of PRNG draws does not depend on sizeof */
static uint8_t gf_byte(uint32_t seed, uint32_t idx) { uint32_t x = (seed ^ (idx * 0x9e3779b1u)) * 2654435761u; x ^= x >> 15; x *= 2246822519u; return (uint8_t)(x >> 24); }
static void garbage_fill_cp(void *o, uint32_t seed, int keep_heap_ptrs) {
  uint8_t *p = (uint8_t *)o;
  size_t ho = offsetof(CpObj, heap_), after = ho + 6 * sizeof(void *), k;
  for (k = 0; k < ho; k++) p[k] = gf_byte(seed, (uint32_t)k);
  for (k = after; k < sizeof(CpObj); k++) p[k] = gf_byte(seed, 0x10000u + (uint32_t)(k - after));
  if (!keep_heap_ptrs) for (k = ho; k < after; k++) p[k] = 0xa5;
}
static void *cp_new(int c2) {
  void *o;
  h_pool_reset();
  o = onew(sizeof(CpObj));
  (c2 ? cp_c2 : cp_c1)(o);
  return o;
}
static void cp_free(void *o, int d2) {
  (d2 ? cp_d2 : cp_d1)(o);
  h_trial_end("cp");
}
/* a second live object inside one trial (no pool / heap reset, destroyed without ending the trial) */
static void *cp_new_nested(int c2) {
  void *o = onew(sizeof(CpObj));
  (c2 ? cp_c2 : cp_c1)(o);
  return o;
}
static void cp_free_nested(void *o, int d2) { (d2 ? cp_d2 : cp_d1)(o); }

enum { CV_CRC_CALC = 0, CV_CRC_OK, CV_CRC_BAD, CV_CRC_SHORT, CV_CRC_LONG, CV_CRC_GARB, CV_CRC_SUM256, CV_CRC_RESET, CV_CRC_FLEN_SMALL, CV_CRC_RUN0 };
enum { CV_EI_S3 = 0, CV_EI_S5, CV_EI_S6, CV_EI_S7, CV_EI_S8, CV_EI_S10, CV_EI_S11, CV_EI_OTHER, CV_EI_GARB, CV_EI_ALIGNED, CV_EI_UNALIGNED, CV_EI_CNT0, CV_EI_BIGBYTE, CV_EI_CHAIN };
enum { CV_IB_SHORT = 0, CV_IB_LONG, CV_IB_A, CV_IB_B, CV_IB_C, CV_IB_ABC, CV_IB_NONE, CV_IB_F11BAD, CV_IB_F11NEG, CV_IB_D1, CV_IB_DBIG, CV_IB_EXACT, CV_IB_ARR, CV_IB_HEAP, CV_IB_ROUNDTRIP, CV_IB_GARB,
       CV_IB_F10NEG, CV_IB_PAIRNEG, CV_IB_PRE, CV_IB_HEAPNEG };
enum { CV_RX_LONG = 0, CV_RX_SHORT, CV_RX_CP, CV_RX_CPT, CV_RX_BADCRC, CV_RX_ED, CV_RX_ABORT, CV_RX_NOISE, CV_RX_SUPP, CV_RX_SUPP_END, CV_RX_ONE_IN_TAIL, CV_RX_LVL0, CV_RX_LVL1, CV_RX_LVL2, CV_RX_LVL_BIG,
       CV_RX_HOOK, CV_RX_BACK2BACK, CV_RX_PRNOF, CV_RX_A, CV_RX_B, CV_RX_C, CV_RX_ABC, CV_RX_NOPART, CV_RX_ED_BUSY };
enum { CV_RF_S0 = 0, CV_RF_S1, CV_RF_S2, CV_RF_S3, CV_RF_S4, CV_RF_S5, CV_RF_S6, CV_RF_S7, CV_RF_S8, CV_RF_S9, CV_RF_S10, CV_RF_S11, CV_RF_S12, CV_RF_S13, CV_RF_SBIG, CV_RF_BIT0, CV_RF_BIT1, CV_RF_ED,
       CV_RF_OVERFLOW, CV_RF_RET1, CV_RF_RET2, CV_RF_RET3, CV_RF_RET4, CV_RF_RET5, CV_RF_SUPP_NEG, CV_RF_SUPP_POS, CV_RF_SUPP_END, CV_RF_CRC_OK, CV_RF_CRC_BAD, CV_RF_S5_NOA, CV_RF_S6_NOB, CV_RF_S8_C, CV_RF_S8_NOC,
       CV_RF_S13_ONE, CV_RF_S13_END, CV_RF_DWRAP, CV_RF_ALPHA, CV_RF_BETA };
enum { CV_MC_C1 = 0, CV_MC_C2, CV_MC_D1, CV_MC_D2, CV_MC_RESET, CV_MC_RESETDET, CV_MC_GETBV, CV_MC_SEQ, CV_MC_SEQ_EXACT, CV_MC_SEQ_ROUND, CV_MC_PRNOF, CV_MC_PRNOF_OFF, CV_MC_RESETCRC, CV_MC_DTOR_NULL1, CV_MC_DTOR_NULLALL,
       CV_MC_DTOR_NULLSOME };

/* ================================================================================================ scenario 0: CRC functions */
static void sc_crc(void) {
  int trial, k;
  cov_def(CV_CRC_CALC, "calcCRC"); cov_def(CV_CRC_OK, "evaluateCRC == 1"); cov_def(CV_CRC_BAD, "evaluateCRC == 0"); cov_def(CV_CRC_SHORT, "short frame"); cov_def(CV_CRC_LONG, "long frame");
  cov_def(CV_CRC_GARB, "garbage buffer / frame length"); cov_def(CV_CRC_SUM256, "evaluateCRC accepted by a sum of 256"); cov_def(CV_CRC_RESET, "resetCRC"); cov_def(CV_CRC_FLEN_SMALL, "frameLen_ <= 0x12 + 17 (no CRC loop)");
  cov_def(CV_CRC_RUN0, "frameLen_ < 17");
  h_scenario_begin(0);
  relax_reserved();
  h_covreq[CV_CRC_RUN0] = 0;
  for (trial = 0; trial < 1500; trial++) {
    void *o = cp_new(0);
    CpObj *co = CO(o);
    Info in;
    Mdl m;
    unsigned kind = h_rmod(8), r;
    uint8_t crcm[16];
    rnd_info(&in, 1, -1, 120);
    model_encode(&in, 1 + h_rmod(8), NULL, &m);
    for (k = 0; k < (int)m.total && k < MAXBITS; k++) co->bits_[k] = m.spec[k] ? m.exp[k] : (uint8_t)h_rmod(2);
    co->frameLen_ = m.frameLen;
    for (k = 0; k < 16; k++) co->crc_[k] = (uint8_t)h_rnd();
    if (in.type) COV(CV_CRC_SHORT); else COV(CV_CRC_LONG);
    if (kind == 1) { unsigned p = 0x13 + h_rmod(m.C - 0x13); if (p % 17) co->bits_[p] ^= 1; }
    if (kind == 2) { for (k = 0; k < 400; k++) co->bits_[k] = (uint8_t)h_rnd(); COV(CV_CRC_GARB); }
    if (kind == 3) { for (k = 0; k < 400; k++) co->bits_[k] = rbyte_kind(); co->frameLen_ = 17 + h_rmod(300); COV(CV_CRC_GARB); if (co->frameLen_ < 0x23) COV(CV_CRC_FLEN_SMALL); }
    if (kind == 4) { co->frameLen_ = 0x22 + h_rmod(5); COV(CV_CRC_FLEN_SMALL); }
    if (kind == 5) { co->frameLen_ = h_rmod(0x23); COV(CV_CRC_FLEN_SMALL); if (co->frameLen_ < 17) { COV(CV_CRC_RUN0); co->frameLen_ += 17; } }
    cp_calccrc(o);
    COV(CV_CRC_CALC);
    h_emit(0, "calcCRC %016llx", (unsigned long long)fnv_upd(FNV0, co->crc_, 16));
    DUMP("calc", o);
    cp_resetcrc(o);
    COV(CV_CRC_RESET);
    for (k = 0; k < 16; k++) if (co->crc_[k] != 1) h_die3("resetCRC left crc[%d] = %d", k, co->crc_[k]);
    cp_calccrc(o);
    if (kind == 0 || kind == 1 || kind == 4 || kind == 6 || kind == 7) {
      /* clean 0/1 buffer: preset + calcCRC = the model register over [0x12, frameLen - 17) */
      model_crc(co->bits_, co->frameLen_ - 0x11, crcm);
      for (k = 0; k < 16; k++) if (co->crc_[k] != crcm[k]) h_die3("calcCRC: stage %d = %d, model %d (frameLen %u kind %u)", k, co->crc_[k], crcm[k], co->frameLen_, kind);
    }
    r = (unsigned)cp_evalcrc(o);
    h_emit(0, "evalCRC r=%u crc %016llx", r, (unsigned long long)fnv_upd(FNV0, co->crc_, 16));
    DUMP("eval", o);
    if (r) COV(CV_CRC_OK); else COV(CV_CRC_BAD);
    if (kind == 0 && !r) h_die3("evaluateCRC rejected a model frame (type %u)", in.type);
    if (kind == 1 && r && 0) h_die3("unreachable");
    cp_free(o, 0);
  }
  /* the 8-bit accumulator: every received CRC bit differs from the register by 16 -> total 256 -> accepted */
  for (k = 0; k < 6; k++) {
    void *o = cp_new(0);
    CpObj *co = CO(o);
    Info in;
    Mdl m;
    uint8_t crcm[16];
    int i;
    unsigned r;
    rnd_info(&in, 1, k & 1, 60);
    model_encode(&in, 1, NULL, &m);
    for (i = 0; i < (int)m.total; i++) co->bits_[i] = m.spec[i] ? m.exp[i] : 0;
    co->frameLen_ = m.frameLen;
    model_crc(co->bits_, m.C, crcm);
    for (i = 0; i < 16; i++) co->bits_[m.frameLen - 16 + (unsigned)i] = (uint8_t)(crcm[i] + 16);
    r = (unsigned)cp_evalcrc(o);
    h_emit(1, "sum256 %d r=%u", k, r);
    if (!r) h_die3("evaluateCRC: a total difference of 256 must be accepted (8-bit sum)");
    COV(CV_CRC_SUM256);
    DUMP("sum256", o);
    cp_free(o, 0);
  }
}

/* ================================================================================================ scenario 1: evaluateInfo */
static void sc_evalinfo(void) {
  int trial, k, g;
  cov_def(CV_EI_S3, "state 3"); cov_def(CV_EI_S5, "state 5"); cov_def(CV_EI_S6, "state 6"); cov_def(CV_EI_S7, "state 7"); cov_def(CV_EI_S8, "state 8"); cov_def(CV_EI_S10, "state 10"); cov_def(CV_EI_S11, "state 11");
  cov_def(CV_EI_OTHER, "other state (no case)"); cov_def(CV_EI_GARB, "garbage buffer bytes"); cov_def(CV_EI_ALIGNED, "cursor on the 17-bit grid"); cov_def(CV_EI_UNALIGNED, "cursor off the grid");
  cov_def(CV_EI_CNT0, "zero counts"); cov_def(CV_EI_BIGBYTE, "byte fields > 1"); cov_def(CV_EI_CHAIN, "full chain 5, 6, 7, 8, 10, 11 on one object");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 5000; trial++) {
    void *o = cp_new(trial & 1);
    CpObj *co = CO(o);
    static const unsigned STATES[] = { 3, 5, 6, 7, 8, 10, 11, 0, 1, 2, 4, 9, 12, 13, 14, 0xffffffffu };
    uint32_t st = STATES[h_rmod(16)], c0;
    unsigned kind = h_rmod(6);
    Info before, exp, got;
    uint32_t expc;
    for (k = 0; k < 3000; k++) co->bits_[k] = kind == 0 ? (uint8_t)h_rmod(2) : kind == 1 ? (uint8_t)rbyte_kind() : kind == 2 ? (uint8_t)(h_rpct(50) ? h_rmod(2) : 2 + h_rmod(254)) : (uint8_t)h_rmod(2);
    if (kind == 1 || kind == 2) COV(CV_EI_GARB);
    /* stale info: arbitrary words everywhere (what the case does not write stays) */
    rnd_info(&before, 0, -1, 100);
    for (g = 0; g < 4; g++) for (k = 0; k < 384; k++) before.arr[g][k] = (uint16_t)h_rnd();
    for (g = 0; g < 6; g++) for (k = 0; k < 128; k++) before.heap[g][k] = h_rnd() & 0xffffu;
    set_info(co, &before);
    /* counts for the cases that loop: small (the heap blocks hold 128 words, the arrays 384) */
    if (st == 8) { for (g = 0; g < 4; g++) co->cnt_[g] = h_rpct(25) ? 0 : h_rmod(25); if (h_rpct(12)) for (g = 0; g < 4; g++) co->cnt_[g] = 0; }
    if (st == 11) { for (g = 0; g < 6; g++) co->hcnt_[g] = h_rpct(25) ? 0 : h_rmod(20); }
    if (st == 8 || st == 11) { unsigned z = 1; for (g = 0; g < 4; g++) z &= co->cnt_[g] == 0; if (st == 8 && z) COV(CV_EI_CNT0); }
    c0 = h_rpct(60) ? (uint32_t)(0x32 + 17 * h_rmod(30)) : (uint32_t)h_rmod(1500);
    if (c0 % 17 == 16) COV(CV_EI_ALIGNED); else COV(CV_EI_UNALIGNED);
    if (kind == 3) { for (k = 0x13; k < 0x40; k++) co->bits_[k] = (uint8_t)rbyte_kind(); COV(CV_EI_BIGBYTE); }
    co->state_ = st;
    co->cursor_ = c0;
    get_info(co, &before);
    exp = before;
    expc = model_eval(&exp, co->bits_, st, c0, c0);
    cp_evalinfo(o);
    switch (st) { case 3: COV(CV_EI_S3); break; case 5: COV(CV_EI_S5); break; case 6: COV(CV_EI_S6); break; case 7: COV(CV_EI_S7); break; case 8: COV(CV_EI_S8); break; case 10: COV(CV_EI_S10); break; case 11: COV(CV_EI_S11); break; default: COV(CV_EI_OTHER); }
    get_info(co, &got);
    if (memcmp(&exp, &got, sizeof exp)) h_die3("evaluateInfo state %u: decoder model mismatch (trial %d, cursor %u)", st, trial, c0);
    if (co->cursor_ != expc) h_die3("evaluateInfo state %u: cursor %u, model %u (c0 %u)", st, co->cursor_, expc, c0);
    h_emit(0, "ei %d st %u cur %u %016llx", trial, st, co->cursor_, (unsigned long long)cp_hash(o, 1));
    DUMP("ei", o);
    cp_free(o, trial & 1);
  }
  /* the whole chain on one object, as the receive machine runs it (cursor continuity): 5 -> 6 -> 7 -> 8 -> 10 -> 11 against the model */
  for (trial = 0; trial < 300; trial++) {
    void *o = cp_new(0);
    CpObj *co = CO(o);
    static const unsigned CH[] = { 5, 6, 7, 8, 10, 11 };
    Info in, exp, got;
    int i;
    for (k = 0; k < 6000; k++) co->bits_[k] = (uint8_t)h_rmod(2);
    rnd_info(&in, 1, 1, 150);
    set_info(co, &in);
    exp = in;
    co->cursor_ = 0x77;   /* anything: case 5 sets it */
    for (i = 0; i < 6; i++) {
      uint32_t expc;
      if (CH[i] == 7) { /* counts from the bits just decoded would exceed the small arrays: clamp (state 8 / 11 read them from the object) */ }
      co->state_ = CH[i];
      expc = model_eval(&exp, co->bits_, CH[i], i == 0 ? 0x77 : co->cursor_, 0);
      if (CH[i] == 7) { /* the 9-bit counts are random bits: keep the later loops inside the arrays */
        cp_evalinfo(o);
        for (g = 0; g < 4; g++) { co->cnt_[g] &= 15; exp.cnt[g] = co->cnt_[g]; }
        get_info(co, &got);
        if (memcmp(&exp, &got, sizeof exp)) { Info e2 = exp; unsigned g2; for (g2 = 0; g2 < 4; g2++) e2.cnt[g2] = got.cnt[g2]; if (memcmp(&e2, &got, sizeof e2)) h_die3("chain: state 7 mismatch"); }
        exp = got;
        if (co->cursor_ != expc) h_die3("chain: state 7 cursor");
        continue;
      }
      if (CH[i] == 10) { /* heap counts: 8 bits each: clamp after decoding for state 11 */
        cp_evalinfo(o);
        for (g = 0; g < 6; g++) co->hcnt_[g] &= 15;
        get_info(co, &got);
        exp = got;
        if (co->cursor_ != expc) h_die3("chain: state 10 cursor");
        continue;
      }
      cp_evalinfo(o);
      get_info(co, &got);
      if (memcmp(&exp, &got, sizeof exp)) h_die3("chain: state %u model mismatch (trial %d)", CH[i], trial);
      if (co->cursor_ != expc) h_die3("chain: state %u cursor %u model %u", CH[i], co->cursor_, expc);
      DUMP("chain", o);
    }
    COV(CV_EI_CHAIN);
    h_emit(0, "chain %d %016llx", trial, (unsigned long long)cp_hash(o, 1));
    cp_free(o, 0);
  }
}

/* the receive of a frame produced by infoToBits (preamble: a zero, 17 ones, the start bit) */
static void feed_bits_to(void *o2, const uint8_t *bits, unsigned frameLen) {
  unsigned i;
  (void)cp_b2i(o2, 0);
  for (i = 0; i < 17; i++) (void)cp_b2i(o2, 1);
  (void)cp_b2i(o2, 0);
  for (i = 0x12; i < frameLen; i++) (void)cp_b2i(o2, bits[i]);
}

/* ================================================================================================ scenario 2: infoToBits */
static void sc_info2bits(void) {
  int trial, k;
  cov_def(CV_IB_SHORT, "short frame"); cov_def(CV_IB_LONG, "long frame"); cov_def(CV_IB_A, "part A"); cov_def(CV_IB_B, "part B"); cov_def(CV_IB_C, "part C"); cov_def(CV_IB_ABC, "parts A + B + C");
  cov_def(CV_IB_NONE, "long frame without a part"); cov_def(CV_IB_F11BAD, "f11 > 3 (bits 0x1b / 0x1c untouched)"); cov_def(CV_IB_F11NEG, "f11 negative (signed compare)"); cov_def(CV_IB_D1, "D == 1");
  cov_def(CV_IB_DBIG, "D large"); cov_def(CV_IB_EXACT, "length an exact multiple of D"); cov_def(CV_IB_ARR, "arrays with elements"); cov_def(CV_IB_HEAP, "heap arrays with elements");
  cov_def(CV_IB_ROUNDTRIP, "round trip checked"); cov_def(CV_IB_GARB, "garbage info bytes"); cov_def(CV_IB_F10NEG, "f10 with bit 7 set (arithmetic shift)"); cov_def(CV_IB_PAIRNEG, "pair / array word with bit 15 set");
  cov_def(CV_IB_PRE, "buffer pre-filled with garbage"); cov_def(CV_IB_HEAPNEG, "heap word with bit 15 set");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 2500; trial++) {
    void *o = cp_new(0);
    CpObj *co = CO(o);
    Info in;
    Mdl m;
    unsigned D, i;
    int legit = h_rpct(60);
    uint8_t pre[BUFSZ];
    rnd_info(&in, legit, -1, 90);
    if (!legit) {
      in.fa = h_rpct(50) ? h_rmod(2) : h_rnd(); in.fb = h_rpct(50) ? h_rmod(2) : h_rnd(); in.fc = h_rpct(50) ? h_rmod(2) : h_rnd();   /* flags: any non-zero value selects the part (the byte stored is the low byte) */
      in.f11 = rbyte_kind();
      in.f10 = (uint8_t)h_rnd();
      if (in.type) in.type = h_rpct(50) ? 1u : (h_rnd() | 1u) & 0xffu;      /* non-zero type: a non-zero byte (a multiple of 256 would store a 0 bit: excluded) */
      for (k = 0; k < 12; k++) in.pair[k] = h_rnd() & 0xffffu;
      for (k = 0; k < 6; k++) in.nib[k] = h_rnd() & 0xffffu;
      for (k = 0; k < 6; k++) { unsigned n; for (n = 0; n < in.hcnt[k]; n++) in.heap[k][n] = h_rnd(); }
      COV(CV_IB_GARB);
      for (k = 0; k < 6; k++) in.hcnt[k] &= 0xffu;   /* encoded as 8-bit values */
    }
    if (!legit) { /* the counts are encoded modulo 2^9 / 2^8 (the loops use the full word): keep the full words small, the encoded bits are what the model checks */ }
    switch (h_rmod(7)) { case 0: D = 1; COV(CV_IB_D1); break; case 1: D = 1 + h_rmod(40); break; case 2: D = 300 + h_rmod(5000); COV(CV_IB_DBIG); break; case 3: D = 17 + 17 * h_rmod(40); break; case 4: D = 6; break; default: D = 1 + h_rmod(12); break; }
    co->trn2dD_ = D;
    for (i = 0; i < BUFSZ; i++) { pre[i] = h_rpct(50) ? (uint8_t)h_rmod(2) : (uint8_t)h_rnd(); co->bits_[i] = pre[i]; }
    COV(CV_IB_PRE);
    for (k = 0; k < 16; k++) co->crc_[k] = (uint8_t)h_rnd();
    set_info(co, &in);
    if (in.type) COV(CV_IB_SHORT); else { COV(CV_IB_LONG); if (in.fa) COV(CV_IB_A); if (in.fb) COV(CV_IB_B); if (in.fc) COV(CV_IB_C); if (in.fa && in.fb && in.fc) COV(CV_IB_ABC); if (!in.fa && !in.fb && !in.fc) COV(CV_IB_NONE); }
    if (!in.type && in.f11 > 3) { if (in.f11 & 0x80) COV(CV_IB_F11NEG); else COV(CV_IB_F11BAD); }
    if (!in.type && (in.f10 & 0x80)) COV(CV_IB_F10NEG);
    { unsigned g, any = 0; for (g = 0; g < 4; g++) if (in.fb && in.cnt[g]) any = 1; if (any) COV(CV_IB_ARR); any = 0; for (g = 0; g < 6; g++) if (in.fc && in.hcnt[g]) any = 1; if (any) COV(CV_IB_HEAP); }
    for (k = 0; k < 12; k++) if (in.fa && (in.pair[k] & 0x8000)) COV(CV_IB_PAIRNEG);
    { unsigned g, n; for (g = 0; g < 6; g++) for (n = 0; n < in.hcnt[g]; n++) if (in.fc && (in.heap[g][n] & 0x8000)) COV(CV_IB_HEAPNEG); }
    {
      unsigned expect_ok;
      model_encode(&in, D, pre, &m);
      expect_ok = m.total + 18 < BUFSZ;
      cp_i2b(o);
      DUMP("i2b", o);
      h_emit(0, "i2b %d %016llx", trial, (unsigned long long)cp_hash(o, 1));
      if (expect_ok) {
        for (i = 0; i < m.total; i++) {
          uint8_t want = m.spec[i] ? m.exp[i] : pre[i];
          if (co->bits_[i] != want) h_die3("infoToBits: bits[%u] = %d, model %d (type %u D %u trial %d)", i, co->bits_[i], want, in.type, D, trial);
        }
        for (i = m.total; i < m.total + 40 && i < BUFSZ; i++) if (co->bits_[i] != pre[i]) h_die3("infoToBits wrote past the frame (bits[%u])", i);
        EXPECT_EQ("frameLen_", co->frameLen_, m.frameLen);
        EXPECT_EQ("totalLen_", co->totalLen_, m.total);
        EXPECT_EQ("widx_", co->widx_, m.C + 18);
        if (m.total == m.C + 0x12) COV(CV_IB_EXACT);
        if (in.type == 0) { EXPECT_EQ("b0ca8", co->b0ca8_, 0); }
        { uint8_t crcm[16]; model_crc(m.exp, m.C, crcm); for (k = 0; k < 16; k++) if (co->crc_[k] != crcm[k]) h_die3("infoToBits: crc_[%d] = %d, model %d", k, co->crc_[k], crcm[k]); }
        { Info now; const char *why = ""; get_info(co, &now); if (!info_same(&now, &in, 1, &why) && legit) h_die3("infoToBits changed the info (%s)", why); }
      }
      (void)expect_ok;
    }
    /* round trip (legit domain, long and short): a second object receives the frame bit by bit and decodes the same info */
    if (legit && m.total + 18 < BUFSZ && D <= 60 && (in.type || ((!in.fb || in.cnt[0] + in.cnt[1] + in.cnt[2] + in.cnt[3] != 0) && (!in.fc || in.hcnt[0] + in.hcnt[1] + in.hcnt[2] + in.hcnt[3] + in.hcnt[4] + in.hcnt[5] != 0)))) {
      void *o2 = cp_new_nested(1);
      CpObj *c2 = CO(o2);
      Info got;
      const char *why = "";
      unsigned guard = 0;
      int ret = 0;
      c2->trn2dD_ = D;
      dsplibs_debug_level = 0;
            feed_bits_to(o2, co->bits_, co->frameLen_);
      while (!ret && guard++ < 12) ret = cp_b2i(o2, 0);
      get_info(c2, &got);
      if (!ret || !info_same(&got, &in, in.type == 0, &why)) h_die3("infoToBits -> bitsToInfo round trip failed (%s, trial %d, type %u, ret %d)", why, trial, in.type, ret);
      COV(CV_IB_ROUNDTRIP);
      DUMP("rt", o2);
      cp_free_nested(o2, 1);
    }
    cp_free(o, 0);
  }
}

/* ================================================================================================ scenario 3: bitsToInfo end to end */
static unsigned g_levels[] = { 0, 1, 2, 3, 7, 0xffffffffu, 0x80000000u, 2 };
static unsigned pick_level(void) { return g_levels[h_rmod(8)]; }
static unsigned g_hook_to, g_hook_at;
static void hook_set(unsigned idx) { if (idx == g_hook_at) { dsplibs_debug_level = g_hook_to; COV(CV_RX_HOOK); } }

typedef struct { int ret; unsigned nprints; } Step;
typedef struct { int32_t supp; } Supp;
static int supp_model(Supp *s, uint32_t raw) {
  if (s->supp < 0) {
    if (raw - 1 > 1) return (int)raw;
    s->supp = 0;
    return (int)raw;
  } else {
    int32_t n = s->supp + 1;
    if (raw == 4 || raw == 2) raw = 0;
    if (n == 800) { s->supp = -1; return (int)raw; }
    s->supp = n;
    return (int)raw;
  }
}
static unsigned g_dbg_kind, g_dbg_a, g_dbg_b, g_dbg_c;
static Supp *g_sp;       /* when set, every feed() call is checked against the suppression-window model (raw code 0, or 5 when the blob reports Ed) */
static int g_end_raw = -1;  /* the raw code expected from THIS call (a frame end), -1 = none */
static Step feed(void *o, int bit, int show) {
  Step s;
  g_dbg_in_call = 0;
  s.ret = cp_b2i(o, (unsigned char)bit);
  s.nprints = g_dbg_in_call;
  h_emit(show, "b %d r %d p %u o %016llx", bit, s.ret, s.nprints, (unsigned long long)cp_hash(o, 0));
  { static uint32_t last_state = 99; if (getenv("P10_TRACE2") && CO(o)->state_ != last_state) { fprintf(stderr, "pos %u state %u -> %u widx %u count %u cursor %u\n", g_dbg_b, last_state, CO(o)->state_, CO(o)->widx_, CO(o)->count_, CO(o)->cursor_); last_state = CO(o)->state_; } }
  if (getenv("P10_TRACE") && g_dbg_b >= 1000) fprintf(stderr, "call pos %u bit %d ret %d state %u widx %u supp_ %d cnt %u\n", g_dbg_b, bit, s.ret, CO(o)->state_, CO(o)->widx_, CO(o)->supp_, CO(o)->count_);
  if (g_sp) {
    uint32_t raw = g_end_raw >= 0 ? (uint32_t)g_end_raw : (s.ret == 5 ? 5u : 0u);
    int want = supp_model(g_sp, raw);
    if (CO(o)->supp_ != g_sp->supp) h_die3("suppression window: object supp_ %d, model %d after the call (raw %u ret %d kind %u frame %u pos %u frameLen %u widx %u)", CO(o)->supp_, g_sp->supp, raw, s.ret, g_dbg_kind, g_dbg_a, g_dbg_b, g_dbg_c, CO(o)->widx_);
    if (want != s.ret) h_die3("return %d, suppression-window model %d (raw %u, supp %d; object: state %u widx %u supp_ %d bit %d kind %u a %u b %u)", s.ret, want, raw, g_sp->supp, CO(o)->state_, CO(o)->widx_, CO(o)->supp_, bit, g_dbg_kind, g_dbg_a, g_dbg_b);
  }
  return s;
}
/* a frame must be receivable: a part with all counts 0 would wait for alpha = 0 / beta = 0 bits for ever (the state machine compares count_ + 1 == alpha) */
static void fix_for_rx(Info *in) {
  if (in->type) return;
  if (in->fb && in->cnt[0] + in->cnt[1] + in->cnt[2] + in->cnt[3] == 0) { in->cnt[0] = 1; in->arr[0][0] = (uint16_t)r16(); }
  if (in->fc && in->hcnt[0] + in->hcnt[1] + in->hcnt[2] + in->hcnt[3] + in->hcnt[4] + in->hcnt[5] == 0) { in->hcnt[0] = 1; in->heap[0][0] = r16(); }
}
static void feed_run(void *o, int bit, unsigned n) { while (n--) (void)feed(o, bit, 0); }

/* feed a model frame to the receiver `o` (already past noise): preamble, payload, then zeros until the frame ends (the first write index that is a multiple of 6 after the
 * CRC: NOT totalLen_); the end call is the one with the frame's raw code (1 / 2 long, 3 / 4 short); checks the decoded info against `in` */
static void feed_frame(void *o, const Info *in, const Mdl *m, Supp *sp, int check) {
  unsigned i, tail = 6 - (m->frameLen % 6), k;
  uint32_t raw = in->type ? (in->f13 == 0 ? 3u : 4u) : (in->f13 == 0 ? 1u : 2u);
  Step s;
  (void)sp;
  feed_run(o, 0, 1);
  feed_run(o, 1, 17);
  feed_run(o, 0, 1);
  g_dbg_c = m->frameLen;
  if (getenv("P10_TRACE")) fprintf(stderr, "frame type %u fa %u fb %u fc %u cnt %u %u %u %u hcnt %u %u %u %u %u %u frameLen %u\n", in->type, in->fa, in->fb, in->fc, in->cnt[0], in->cnt[1], in->cnt[2], in->cnt[3], in->hcnt[0], in->hcnt[1], in->hcnt[2], in->hcnt[3], in->hcnt[4], in->hcnt[5], m->frameLen);
  for (i = 0x12; i < m->frameLen; i++) {
    g_dbg_b = i;
    s = feed(o, m->spec[i] ? m->exp[i] : 0, 0);
    if (s.ret != 0 && s.ret != 5) h_die3("early return code %d at bit %u of %u", s.ret, i, m->frameLen);
  }
  for (k = 0; k < tail; k++) {
    g_end_raw = (k + 1 == tail) ? (int)raw : -1;
    g_dbg_b = 1000 + k;
    if (k + 1 == tail && sp->supp >= 0 && (raw == 2 || raw == 4)) COV(CV_RX_SUPP);
    s = feed(o, 0, 0);
    g_end_raw = -1;
  }
  if (check) {
    Info got;
    const char *why = "";
    get_info(CO(o), &got);
    if (!info_same(&got, in, in->type == 0, &why)) h_die3("decoded info != sent info (%s, type %u)", why, in->type);
  }
}

static void sc_rx(void) {
  int trial, k;
  cov_def(CV_RX_LONG, "long frame received"); cov_def(CV_RX_SHORT, "short frame received"); cov_def(CV_RX_CP, "CP (f13 == 0)"); cov_def(CV_RX_CPT, "f13 != 0"); cov_def(CV_RX_BADCRC, "bad CRC");
  cov_def(CV_RX_ED, "Ed (code 5) while idle"); cov_def(CV_RX_ABORT, "a 1 instead of the start bit"); cov_def(CV_RX_NOISE, "noise"); cov_def(CV_RX_SUPP, "suppression window: a code 2 / 4 reported as 0");
  cov_def(CV_RX_SUPP_END, "suppression window ends (800 calls)"); cov_def(CV_RX_ONE_IN_TAIL, "a 1 bit in the tail (state 13)"); cov_def(CV_RX_LVL0, "level 0"); cov_def(CV_RX_LVL1, "level 1");
  cov_def(CV_RX_LVL2, "level 2 / 3 / 7"); cov_def(CV_RX_LVL_BIG, "level 0xffffffff / 0x80000000 (unsigned compare)"); cov_def(CV_RX_HOOK, "level lowered inside a print"); cov_def(CV_RX_BACK2BACK, "frames back to back");
  cov_def(CV_RX_PRNOF, "printNofRecievedMpMpNot"); cov_def(CV_RX_A, "part A"); cov_def(CV_RX_B, "part B"); cov_def(CV_RX_C, "part C"); cov_def(CV_RX_ABC, "parts A + B + C"); cov_def(CV_RX_NOPART, "long frame without a part");
  cov_def(CV_RX_ED_BUSY, "2D zeros inside a frame (no Ed)");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 400; trial++) {
    void *o = cp_new(0);
    CpObj *co = CO(o);
    unsigned D = h_rpct(20) ? (h_rpct(50) ? 6 : 1 + h_rmod(60)) : (1 + h_rmod(8));
    unsigned level = pick_level();
    int nframes = 1 + (int)h_rmod(4), f;
    Supp sp;
    sp.supp = -1;
    g_sp = &sp;
    co->trn2dD_ = D;
    dsplibs_debug_level = level;
    g_dbg_hook = NULL;
    g_hook_at = 0xffffffffu;
    if (h_rpct(25)) { g_dbg_hook = hook_set; g_hook_at = h_rmod(3); g_hook_to = h_rmod(4); }
    if (level == 0) COV(CV_RX_LVL0); else if (level == 1) COV(CV_RX_LVL1); else if (level < 8) COV(CV_RX_LVL2); else COV(CV_RX_LVL_BIG);
    for (f = 0; f < nframes; f++) {
      Info in;
      Mdl m;
      unsigned kind = h_rmod(10), pre = h_rmod(30), i, z;
      g_dbg_kind = kind; g_dbg_a = (unsigned)f; g_dbg_b = D;
      rnd_info(&in, 1, -1, 120);
      fix_for_rx(&in);
      model_encode(&in, D, NULL, &m);
      for (i = 0; i < pre; i++) { Step s = feed(o, h_rpct(40) ? 1 : 0, 0); COV(CV_RX_NOISE); if (s.ret == 5) COV(CV_RX_ED); }
      if (kind == 0) { /* idle zero run: Ed after 2 * D zeros */
        feed_run(o, 1, 1);
        for (z = 0; z < 2 * D + 3; z++) { Step s = feed(o, 0, 0); if (s.ret == 5) COV(CV_RX_ED); }
        continue;
      }
      if (kind == 1) { feed_run(o, 0, 1); feed_run(o, 1, 16); feed_run(o, 0, 1); continue; }               /* 16 ones: stays in state 0 */
      if (kind == 2) { feed_run(o, 0, 1); feed_run(o, 1, 17); feed_run(o, 1, 1); COV(CV_RX_ABORT); continue; }  /* a 1 instead of the start bit */
      if (kind == 3) { /* damaged data bit: bad CRC, the receiver returns to state 0 */
        unsigned p, tries = 0;
        do { p = 0x13 + h_rmod(m.C - 0x13); } while ((p % 17 == 0 || m.sflag[p]) && ++tries < 1000);   /* a flip of the type bit, a flag or a count would change the frame structure */
        if (!(p % 17 == 0 || m.sflag[p]) && p < m.C) m.exp[p] ^= 1; else m.exp[0x20] ^= 1;
        feed_run(o, 0, 1); feed_run(o, 1, 17); feed_run(o, 0, 1);
        { unsigned lim = m.frameLen + 8; for (i = 0x12; i < lim; i++) (void)feed(o, i < m.total && m.spec[i] ? m.exp[i] : 0, 0); }
        COV(CV_RX_BADCRC);
        { unsigned guard = 0; while (!(co->state_ == 0 && co->widx_ == 0x12) && guard++ < 14000) (void)feed(o, (int)h_rmod(2), 0); }
        continue;
      }
      if (kind == 4) { /* a 1 bit in the tail ends the frame at once (state 13): the return code is the frame's */
        uint32_t raw = in.type ? (in.f13 == 0 ? 3u : 4u) : (in.f13 == 0 ? 1u : 2u);
        feed_run(o, 0, 1); feed_run(o, 1, 17); feed_run(o, 0, 1);
        for (i = 0x12; i < m.frameLen; i++) (void)feed(o, m.spec[i] ? m.exp[i] : 0, 0);
        g_end_raw = (int)raw;
        (void)feed(o, 1, 0);
        g_end_raw = -1;
        COV(CV_RX_ONE_IN_TAIL);
        continue;
      }
      /* the clean frame, end to end (a zero precedes the preamble so that the 17 ones start a fresh run) */
      {
        feed_frame(o, &in, &m, &sp, 1);
        if (in.type) COV(CV_RX_SHORT); else { COV(CV_RX_LONG); if (in.fa) COV(CV_RX_A); if (in.fb) COV(CV_RX_B); if (in.fc) COV(CV_RX_C); if (in.fa && in.fb && in.fc) COV(CV_RX_ABC); if (!in.fa && !in.fb && !in.fc) COV(CV_RX_NOPART); }
        if (in.f13 == 0) COV(CV_RX_CP); else COV(CV_RX_CPT);
        if (f > 0) COV(CV_RX_BACK2BACK);
        DUMP("rx", o);
        h_emit(0, "frame end %016llx", (unsigned long long)cp_hash(o, 1));
      }
      /* idle calls inside the suppression window (the counter advances per call; at 800 it ends) */
      { unsigned idle = h_rpct(8) ? 900 : h_rmod(20); for (i = 0; i < idle; i++) { int32_t before = sp.supp; (void)feed(o, 0, 0); if (before == 799) COV(CV_RX_SUPP_END); } }
    }
    g_dbg_hook = NULL;
    g_sp = NULL;
    dsplibs_debug_level = level;
    g_dbg_in_call = 0;
    cp_prnof(o);
    COV(CV_RX_PRNOF);
    if (g_dbg_in_call != (level > 1 ? 1u : 0u)) h_die3("printNofRecievedMpMpNot: %u prints at level 0x%x", g_dbg_in_call, level);
    DUMP("rxend", o);
    cp_free(o, 0);
  }
  /* 2D zeros inside a frame never fire Ed (the write index is not 0x12) */
  for (trial = 0; trial < 5; trial++) {
    void *o = cp_new(0);
    CpObj *co = CO(o);
    Info in;
    Mdl m;
    unsigned i;
    int fired = 0;
    rnd_info(&in, 1, 1, 20);
    in.f10 = 0; in.f13 = 0; in.w14 = 0;
    co->trn2dD_ = 2 + (unsigned)trial;   /* D == 1 fires Ed at the type bit of a long frame (2 zeros while widx_ is still 0x12) */
    dsplibs_debug_level = 0;
    model_encode(&in, co->trn2dD_, NULL, &m);
    feed_run(o, 0, 1); feed_run(o, 1, 17); feed_run(o, 0, 1);
    for (i = 0x12; i < 0x30; i++) { Step s = feed(o, m.spec[i] ? m.exp[i] : 0, 0); if ((unsigned)co->zeros_ >= 2 * co->trn2dD_ && co->widx_ != 0x12) COV(CV_RX_ED_BUSY); if (s.ret == 5) fired = 1; }
    if (fired) h_die3("Ed fired inside a frame");
    cp_free(o, 0);
  }
  (void)k;
}

/* ================================================================================================ scenario 4: random states */
static void sc_rxfuzz(void) {
  int trial, k;
  cov_def(CV_RF_S0, "state 0"); cov_def(CV_RF_S1, "state 1"); cov_def(CV_RF_S2, "state 2"); cov_def(CV_RF_S3, "state 3"); cov_def(CV_RF_S4, "state 4"); cov_def(CV_RF_S5, "state 5"); cov_def(CV_RF_S6, "state 6");
  cov_def(CV_RF_S7, "state 7"); cov_def(CV_RF_S8, "state 8"); cov_def(CV_RF_S9, "state 9 (no case)"); cov_def(CV_RF_S10, "state 10"); cov_def(CV_RF_S11, "state 11"); cov_def(CV_RF_S12, "state 12"); cov_def(CV_RF_S13, "state 13");
  cov_def(CV_RF_SBIG, "state > 13"); cov_def(CV_RF_BIT0, "bit 0"); cov_def(CV_RF_BIT1, "bit != 0"); cov_def(CV_RF_ED, "Ed (code 5)"); cov_def(CV_RF_OVERFLOW, "buffer overflow message path"); cov_def(CV_RF_RET1, "return 1");
  cov_def(CV_RF_RET2, "return 2"); cov_def(CV_RF_RET3, "return 3"); cov_def(CV_RF_RET4, "return 4"); cov_def(CV_RF_RET5, "return 5"); cov_def(CV_RF_SUPP_NEG, "supp_ < 0"); cov_def(CV_RF_SUPP_POS, "supp_ >= 0");
  cov_def(CV_RF_SUPP_END, "supp_ reaches 800"); cov_def(CV_RF_CRC_OK, "state 12 CRC ok"); cov_def(CV_RF_CRC_BAD, "state 12 CRC bad"); cov_def(CV_RF_S5_NOA, "state 5 end, flagA_ == 0"); cov_def(CV_RF_S6_NOB, "state 6 end, flagB_ == 0");
  cov_def(CV_RF_S8_C, "state 8 end -> 10"); cov_def(CV_RF_S8_NOC, "state 8 end -> 12"); cov_def(CV_RF_S13_ONE, "state 13 with a 1 bit"); cov_def(CV_RF_S13_END, "state 13 frame end"); cov_def(CV_RF_DWRAP, "2 * D wraps to 0");
  cov_def(CV_RF_ALPHA, "state 8 count reaches alpha"); cov_def(CV_RF_BETA, "state 11 count reaches beta");
  h_covreq[CV_RF_ALPHA] = 0; h_covreq[CV_RF_BETA] = 0;   /* alpha / beta are reached by the model frames of scenario 3 */
  h_scenario_begin(0);
  relax_reserved();
  h_covreq[CV_RF_S8_C] = 0; h_covreq[CV_RF_S8_NOC] = 0;   /* state 8 ends only after alpha (= 17 * sum of the counts) calls: scenario 3 */
  for (trial = 0; trial < 6000; trial++) {
    void *o = cp_new(0);
    CpObj *co = CO(o);
    unsigned ncalls = 20 + h_rmod(160), i;
    Info in;
    unsigned st;
    for (k = 0; k < 3500; k++) co->bits_[k] = h_rpct(60) ? (uint8_t)h_rmod(2) : rbyte_kind();
    for (k = 3500; k < (int)BUFSZ; k += 7) co->bits_[k] = (uint8_t)h_rnd();
    rnd_info(&in, 0, -1, 60);
    set_info(co, &in);
    st = h_rmod(16);
    co->state_ = st < 14 ? st : (st == 14 ? 14 + h_rmod(100) : 0xffffffffu);
    co->ones_ = (uint8_t)(h_rpct(50) ? 15 + h_rmod(4) : h_rnd());
    co->zeros_ = (uint8_t)(h_rpct(30) ? h_rmod(4) : h_rnd());
    co->widx_ = h_rpct(35) ? 0x12 : (h_rpct(60) ? 0x13 + h_rmod(0x90) : (h_rpct(70) ? 0x12 + h_rmod(400) : 0x20 + h_rmod(3000)));
    co->count_ = h_rpct(60) ? h_rmod(0x60) : h_rmod(30);
    co->cursor_ = h_rmod(1200);
    co->trn2dD_ = h_rpct(35) ? 1 + h_rmod(4) : (h_rpct(70) ? 1 + h_rmod(300) : (h_rpct(30) ? (h_rpct(50) ? 0x80000000u : 0x40000000u) : 1 + (h_rnd() & 0xffffffu)));
    co->nMp_ = h_rnd(); co->nMpNot_ = h_rnd();
    co->supp_ = h_rpct(50) ? -1 : (h_rpct(70) ? (int32_t)h_rmod(805) : (int32_t)h_rnd());
    for (k = 0; k < 16; k++) co->crc_[k] = (uint8_t)(h_rpct(60) ? h_rmod(2) : h_rnd());
    co->flagA_ = h_rpct(50) ? h_rmod(2) : h_rnd(); co->flagB_ = h_rpct(50) ? h_rmod(2) : h_rnd(); co->flagC_ = h_rpct(50) ? h_rmod(2) : h_rnd();
    co->type_ = h_rpct(50) ? h_rmod(2) : h_rnd();
    co->f13_ = (uint8_t)h_rmod(2);
    for (k = 0; k < 4; k++) co->cnt_[k] = h_rpct(30) ? 0 : h_rmod(24);
    for (k = 0; k < 6; k++) co->hcnt_[k] = h_rpct(30) ? 0 : h_rmod(20);
    switch (co->state_) {
      case 3: if (h_rpct(60)) co->count_ = 14; break;
      case 4: co->count_ = h_rmod(4); if (h_rpct(50)) co->count_ = 2; if (h_rpct(60)) co->widx_ = 0x13 + co->count_; break;
      case 5: if (h_rpct(60)) co->widx_ = 0x32; break;
      case 6: if (h_rpct(60)) co->widx_ = 0x98; break;
      case 7: if (h_rpct(60)) co->count_ = 0x43; break;
      case 10: if (h_rpct(60)) co->count_ = 0x54; break;
      case 12: if (h_rpct(60)) co->count_ = 0x10; break;
      case 13: if (h_rpct(50)) co->widx_ = 6 * (3 + h_rmod(20)) - 1; break;
      default: break;
    }
    if (h_rpct(6) && (co->state_ == 3 || co->state_ == 8 || co->state_ == 10 || co->state_ == 11 || co->state_ == 12)) co->widx_ = BUFSZ - 2 + h_rmod(4);   /* the 12,000-bit limit */
    dsplibs_debug_level = pick_level();
    g_dbg_hook = NULL;
    if (h_rpct(20)) { g_dbg_hook = hook_set; g_hook_at = h_rmod(2); g_hook_to = h_rmod(4); }
    for (i = 0; i < ncalls; i++) {
      int bit = h_rpct(25) ? 1 + (int)h_rmod(3) * (h_rpct(10) ? 85 : 1) : 0;
      uint32_t s0 = co->state_;
      Step s;
      uint32_t w0 = co->widx_, supp0 = (uint32_t)co->supp_;
      if (h_rpct(60)) bit = (int)h_rmod(2);
      if (s0 == 12 && co->count_ == 0x10 && co->widx_ >= 0x12 && co->widx_ < BUFSZ - 20 && h_rpct(40)) {
        /* make the CRC come out right: the last CRC bit is the bit fed, the 15 before it are written now (the data between 0x12 and the CRC stays as it is) */
        unsigned fl = co->widx_ + 1;
        uint8_t crcm[16];
        if (fl >= 0x12 + 0x11) {
          int j;
          model_crc(co->bits_, fl - 0x11, crcm);
          for (j = 0; j < 15; j++) co->bits_[fl - 16 + (unsigned)j] = crcm[j];
          bit = crcm[15];
        }
      }
      if (co->trn2dD_ == 0) co->trn2dD_ = 1;
      if (s0 == 0) COV(CV_RF_S0); else if (s0 == 1) COV(CV_RF_S1); else if (s0 == 2) COV(CV_RF_S2); else if (s0 == 3) COV(CV_RF_S3); else if (s0 == 4) COV(CV_RF_S4); else if (s0 == 5) COV(CV_RF_S5);
      else if (s0 == 6) COV(CV_RF_S6); else if (s0 == 7) COV(CV_RF_S7); else if (s0 == 8) COV(CV_RF_S8); else if (s0 == 9) COV(CV_RF_S9); else if (s0 == 10) COV(CV_RF_S10); else if (s0 == 11) COV(CV_RF_S11);
      else if (s0 == 12) COV(CV_RF_S12); else if (s0 == 13) COV(CV_RF_S13); else COV(CV_RF_SBIG);
      if (bit) COV(CV_RF_BIT1); else COV(CV_RF_BIT0);
      if (co->trn2dD_ == 0x80000000u || co->trn2dD_ == 0x40000000u) COV(CV_RF_DWRAP);
      if ((int32_t)supp0 < 0) COV(CV_RF_SUPP_NEG); else COV(CV_RF_SUPP_POS);
      if (supp0 == 799u) COV(CV_RF_SUPP_END);
      if ((s0 == 3 || s0 == 8 || s0 == 10 || s0 == 11 || s0 == 12) && w0 >= BUFSZ) COV(CV_RF_OVERFLOW);
      s = feed(o, bit, 0);
      if (s.ret == 1) COV(CV_RF_RET1); else if (s.ret == 2) COV(CV_RF_RET2); else if (s.ret == 3) COV(CV_RF_RET3); else if (s.ret == 4) COV(CV_RF_RET4); else if (s.ret == 5) { COV(CV_RF_RET5); COV(CV_RF_ED); }
      if (s0 == 12 && co->state_ == 13) COV(CV_RF_CRC_OK);
      if (s0 == 12 && co->state_ == 0 && co->widx_ == 0x12 && w0 >= 0x12 && co->count_ == 0) COV(CV_RF_CRC_BAD);
      if (s0 == 5 && co->state_ != 5 && co->flagA_ == 0) COV(CV_RF_S5_NOA);
      if (s0 == 6 && co->state_ != 6 && co->flagB_ == 0) COV(CV_RF_S6_NOB);
      if (s0 == 8 && co->state_ == 10) COV(CV_RF_S8_C);
      if (s0 == 8 && co->state_ == 12) COV(CV_RF_S8_NOC);
      if (s0 == 13 && bit) COV(CV_RF_S13_ONE);
      if (s0 == 13 && co->state_ == 0) COV(CV_RF_S13_END);
      if (i % 37 == 0) DUMP("rf", o);
      if (h_rpct(4)) { co->state_ = h_rmod(14); co->widx_ = h_rpct(50) ? 0x12 : 0x12 + h_rmod(3000); }
    }
    DUMP("rfend", o);
    h_emit(0, "rfend %016llx", (unsigned long long)cp_hash(o, 1));
    cp_free(o, 0);
  }
  g_dbg_hook = NULL;
}

/* ================================================================================================ scenario 5: small functions / lifecycle */
static void sc_small(void) {
  int trial, k;
  cov_def(CV_MC_C1, "C1"); cov_def(CV_MC_C2, "C2"); cov_def(CV_MC_D1, "D1"); cov_def(CV_MC_D2, "D2"); cov_def(CV_MC_RESET, "reset"); cov_def(CV_MC_RESETDET, "resetDetector"); cov_def(CV_MC_GETBV, "getBitVector");
  cov_def(CV_MC_SEQ, "calcSequenceLength"); cov_def(CV_MC_SEQ_EXACT, "length an exact multiple"); cov_def(CV_MC_SEQ_ROUND, "length rounded up"); cov_def(CV_MC_PRNOF, "printNof at level > 1"); cov_def(CV_MC_PRNOF_OFF, "printNof at level <= 1");
  cov_def(CV_MC_RESETCRC, "resetCRC"); cov_def(CV_MC_DTOR_NULL1, "dtor with one NULL block"); cov_def(CV_MC_DTOR_NULLALL, "dtor with all blocks NULL"); cov_def(CV_MC_DTOR_NULLSOME, "dtor with several NULL blocks");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 1500; trial++) {
    void *o;
    CpObj *co;
    unsigned which = h_rmod(2);
    h_pool_reset();
    o = onew(sizeof(CpObj));
    co = CO(o);
    garbage_fill_cp(o, h_rnd(), 0);   /* stale garbage everywhere */
    /* the constructor: six allocations of 0x200 in order, then the stores */
    {
      unsigned long m0 = h_nmalloc;
      (which ? cp_c2 : cp_c1)(o);
      COV(which ? CV_MC_C2 : CV_MC_C1);
      if (h_nmalloc - m0 != 6) h_die3("constructor made %lu allocations", h_nmalloc - m0);
    }
    EXPECT_EQ("widx", co->widx_, 0x12); EXPECT_EQ("count", co->count_, 0); EXPECT_EQ("state", co->state_, 0); EXPECT_EQ("ones", co->ones_, 0); EXPECT_EQ("zeros", co->zeros_, 0);
    EXPECT_EQ("nMp", co->nMp_, 0); EXPECT_EQ("nMpNot", co->nMpNot_, 0); EXPECT_EQ("f13", co->f13_, 0); EXPECT_EQ("supp", (uint32_t)co->supp_, 0xffffffffu);
    DUMP("ctor", o);
    h_emit(0, "ctor %u %016llx", which, (unsigned long long)cp_hash(o, 1));
    /* reset / resetDetector on a dirty object: which fields they leave alone */
    garbage_fill_cp(o, h_rnd(), 1);   /* dirty everything but the heap pointers */
    if (h_rpct(50)) { cp_reset(o); COV(CV_MC_RESET); EXPECT_EQ("widx", co->widx_, 0x12); EXPECT_EQ("state", co->state_, 0); EXPECT_EQ("nMp", co->nMp_, 0); EXPECT_EQ("supp", (uint32_t)co->supp_, 0xffffffffu); }
    else { cp_resetdet(o); COV(CV_MC_RESETDET); EXPECT_EQ("widx", co->widx_, 0x12); EXPECT_EQ("count", co->count_, 0); EXPECT_EQ("state", co->state_, 0); EXPECT_EQ("ones", co->ones_, 0); EXPECT_EQ("zeros", co->zeros_, 0); }
    DUMP("reset", o);
    h_emit(0, "reset %016llx", (unsigned long long)cp_hash(o, 1));
    /* getBitVector / calcSequenceLength / resetCRC / printNof */
    {
      unsigned len = 0xdeadbeefu;
      uint8_t *p = cp_getbv(o, &len);
      COV(CV_MC_GETBV);
      if (p != BITS(o)) h_die3("getBitVector returned %p, buffer %p", (void *)p, (void *)BITS(o));
      EXPECT_EQ("getBitVector length", len, co->totalLen_);
    }
    {
      uint32_t D = h_rpct(30) ? 1 + h_rmod(8) : (h_rpct(50) ? 1 + h_rmod(300) : 1 + (h_rnd() & 0xfffffu));
      uint32_t fl = h_rpct(40) ? 0x22 + 17 * h_rmod(60) : h_rnd() & 0xfffffu;
      uint32_t v = fl + 1, q = v / D, want = (q * D == v) ? v : (q + 1) * D;
      co->trn2dD_ = D;
      co->frameLen_ = fl;
      cp_calcseq(o);
      COV(CV_MC_SEQ);
      if (q * D == v) COV(CV_MC_SEQ_EXACT); else COV(CV_MC_SEQ_ROUND);
      EXPECT_EQ("calcSequenceLength", co->totalLen_, want);
      DUMP("seq", o);
    }
    cp_resetcrc(o);
    COV(CV_MC_RESETCRC);
    for (k = 0; k < 16; k++) EXPECT_EQ("resetCRC", co->crc_[k], 1);
    {
      unsigned lvl = pick_level();
      co->nMp_ = h_rnd(); co->nMpNot_ = h_rnd();
      dsplibs_debug_level = lvl;
      g_dbg_in_call = 0;
      cp_prnof(o);
      if (lvl > 1) {
        char want[100];
        COV(CV_MC_PRNOF);
        snprintf(want, sizeof want, "V90MP: received %d MP, %d MPNot\\r\\n", (int)co->nMp_, (int)co->nMpNot_);
        if (g_dbg_in_call != 1 || strcmp(g_dbg_text[0], want)) h_die3("printNof text '%s' want '%s'", g_dbg_text[0], want);
      } else {
        COV(CV_MC_PRNOF_OFF);
        if (g_dbg_in_call != 0) h_die3("printNof printed at level %u", lvl);
      }
    }
    /* the destructor frees the non-NULL blocks in order; NULL pointers are skipped (the harness stores NULL into some slots first: they leak by design, so the leaks are freed by hand) */
    {
      unsigned long f0 = h_nfree;
      void *blk[6];
      unsigned mask = h_rpct(60) ? 0 : (h_rpct(10) ? 0x3fu : h_rmod(64)), nnull = 0;
      for (k = 0; k < 6; k++) blk[k] = co->heap_[k];
      for (k = 0; k < 6; k++) if (mask & (1u << k)) { co->heap_[k] = NULL; nnull++; }
      (which ? cp_d2 : cp_d1)(o);
      COV(which ? CV_MC_D2 : CV_MC_D1);
      if (h_nfree - f0 != 6 - nnull) h_die3("destructor freed %lu blocks, expected %u", h_nfree - f0, 6 - nnull);
      if (nnull == 1) COV(CV_MC_DTOR_NULL1); else if (nnull == 6) COV(CV_MC_DTOR_NULLALL); else if (nnull > 1) COV(CV_MC_DTOR_NULLSOME);
      for (k = 0; k < 6; k++) if (mask & (1u << k)) sysdep_free(blk[k]);
      h_emit(0, "dtor mask %u frees %lu", mask, h_nfree - f0);
    }
    h_trial_end("small");
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P10_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P10_DBG")) g_dbg_show = 1;
  p10_layout_check();
  h_x87_check();
  h_arena_init();
  build_dump_table();
  switch (sc) {
    case 0: sc_crc(); break;
    case 1: sc_evalinfo(); break;
    case 2: sc_info2bits(); break;
    case 3: sc_rx(); break;
    case 4: sc_rxfuzz(); break;
    case 5: sc_small(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu dbgprintf %u", h_nmalloc, h_nfree, g_ndbg);
  h_cov_finish();
  return 0;
}
