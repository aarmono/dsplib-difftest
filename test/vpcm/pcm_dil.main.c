/*
 * Differential test harness for port chunk P13 (src/vpcm/pcm_dil.cpp): the DIL descriptor helpers and packers of dsplibs.o - setDilDescriptor, getSegmentPointer, calculateDilLength
 * (both overloads), DILdescriptorPacker (V.90 frame, shorts) and V92DILdescriptorPacker (V.92 frame, bytes, debug prints) (6 functions).
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/pcm_dil.sh against the blob (dsplibs.o; `edprintf` weakened so that the recorder below wins;
 * `dsplibs_debug_printf` and the allocator are the harness'), against the port (-O0 / -O2) and for x86-64 (diffed against the 32-bit BLOB's output = the golden
 * test/golden/vpcm/pcm_dil.blob32.out).  The functions are reached through __asm__ labels; the descriptor is the C mirror generated from test/vpcm/pcm_dil.layout (no pointers).
 *
 * What is compared: return values, every byte of the output frames (guard words in front of and behind, spec'd positions vs untouched positions), the length cell, the whole
 * descriptor after every call, every edprintf / dsplibs_debug_printf call (the text; the `%X` pointer words are mapped to D / OUT / LEN so that the 64-bit run, which prints the
 * low 32 bits of a pointer, is comparable).  INDEPENDENT MODELS written from the frame definition (not from the port): a sequential frame writer whose start-bit grid (every index
 * that is a multiple of 17 carries a 0) is inserted automatically, a standard CRC-CCITT (x^16 + x^12 + x^5 + 1, all ones, over the bits after the 18th slot without the grid) of the
 * parity of each word, the zone / row counts as plain integer ceilings, and the expected image of setDilDescriptor as an FNV hash of the bytes extracted from the blob's .data
 * (a Python script, not the port).  Scenarios (each its own process): 0 setDilDescriptor, 1 getSegmentPointer + calculateDilLength (both overloads, cross-checked), 2 the V.90
 * packer, 3 the V.92 packer.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "pcm_dil_layout.h"

#define L(m) __asm__(m)

extern void set_dil(DilDesc *, int) L("_Z16setDilDescriptorP19tagV90DILdescriptor7DilType");
extern int seg_ptr(int, int) L("_Z17getSegmentPointer7PcmTypei");
extern int dil_len_t(int, int) L("_Z18calculateDilLength7DilType7PcmType");
extern int dil_len_d(DilDesc *, int) L("_Z18calculateDilLengthP19tagV90DILdescriptor7PcmType");
extern void pack90(DilDesc *, short *, short *) L("DILdescriptorPacker");
extern void pack92(DilDesc *, unsigned char *, int *) L("_Z22V92DILdescriptorPackerP19tagV90DILdescriptorPhPi");
extern unsigned int dsplibs_debug_level;

/* ------------------------------------------------------------------------------------------------ recorders */
#define TXTMAX 64
static unsigned g_nedp, g_ndbg;
static char g_ed_text[TXTMAX][300], g_dbg_text[TXTMAX][300];
static unsigned g_ed_in_call, g_dbg_in_call;
static int g_show;
static unsigned g_k_d, g_k_out, g_k_len;   /* low 32 bits of the pointers of the current call */
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
void edprintf(const char *fmt, ...) {
  char buf[600];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (g_ed_in_call < TXTMAX) { g_ed_text[g_ed_in_call][0] = 0; esc_cat(g_ed_text[g_ed_in_call], 300, buf); }
  h_emit(g_show, "edprintf %s", g_ed_in_call < TXTMAX ? g_ed_text[g_ed_in_call] : "(many)");
  g_ed_in_call++;
  g_nedp++;
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[400], f2[200];
  va_list ap;
  const char *x = strstr(fmt, "%X");
  va_start(ap, fmt);
  if (x) {   /* a pointer word: the blob passes 32 bits, the 64-bit port the low 32 bits of a 64-bit pointer */
    unsigned a = va_arg(ap, unsigned);
    const char *nm = a == g_k_d ? "D" : a == g_k_out ? "OUT" : a == g_k_len ? "LEN" : "?";
    size_t n = (size_t)(x - fmt);
    memcpy(f2, fmt, n);
    f2[n] = 0;
    strcat(f2, nm);
    strcat(f2, x + 2);
    snprintf(buf, sizeof buf, "%s", f2);
  } else {
    vsnprintf(buf, sizeof buf, fmt, ap);
  }
  va_end(ap);
  if (g_dbg_in_call < TXTMAX) { g_dbg_text[g_dbg_in_call][0] = 0; esc_cat(g_dbg_text[g_dbg_in_call], 300, buf); }
  h_emit(g_show, "dbgprintf %s", g_dbg_in_call < TXTMAX ? g_dbg_text[g_dbg_in_call] : "(many)");
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
static void cov_def(int id, const char *name) { h_cov_define(id, name); }
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }
#define COV(id) h_cov_hit(id)
static uint8_t gf_byte(uint32_t seed, uint32_t idx) { uint32_t x = (seed ^ (idx * 0x9e3779b1u)) * 2654435761u; x ^= x >> 15; x *= 2246822519u; return (uint8_t)(x >> 24); }
static void garbage_fill(void *o, size_t n, uint32_t seed) { size_t k; uint8_t *p = (uint8_t *)o; for (k = 0; k < n; k++) p[k] = gf_byte(seed, (uint32_t)k); }
static void *onew_canary(size_t n) { void *p = onew(n + 32); memset((uint8_t *)p + n, 0xc3, 32); return p; }
static void canary_check(const void *p, size_t n, const char *what) {
  unsigned c;
  for (c = 0; c < 32; c++) if (((const uint8_t *)p)[n + c] != 0xc3) h_die3("%s wrote past the %zu-byte block (byte +%u)", what, n, c);
}
static HFld g_fd[16];
static int g_nfd;
static unsigned long g_dumpctr;
#define DUMPD(tag, o) h_dump_obj(h_sample(&g_dumpctr, 12, 97), (tag), (o), g_fd, g_nfd)

/* ------------------------------------------------------------------------------------------------ models */
/* the CRC-CCITT over bits 0x12 .. endp - 1 without the grid, all-ones start, of the parity of each element */
static void model_crc(const int *b, unsigned endp, uint8_t out[16]) {
  uint16_t r = 0xffff;
  unsigned p, k;
  for (p = 0x12; p < endp; p++) {
    unsigned fb;
    if (p % 17 == 0) continue;
    fb = ((r >> 15) ^ ((unsigned)b[p] & 1u)) & 1u;
    r = (uint16_t)(r << 1);
    if (fb) r ^= 0x1021;
  }
  for (k = 0; k < 16; k++) out[k] = (uint8_t)((r >> (15 - k)) & 1);
}

#define OUTN 3300
typedef struct { int exp[OUTN]; uint8_t spec[OUTN]; unsigned pos; } Fr;
static void fput(Fr *m, unsigned p, int v) { if (p >= OUTN) h_die3("model: frame beyond %d", OUTN); m->exp[p] = v; m->spec[p] = 1; }
static void fgrid(Fr *m) { if (m->pos % 17 == 0) { fput(m, m->pos, 0); m->pos++; } }
static void fraw(Fr *m, int v) { fgrid(m); fput(m, m->pos, v); m->pos++; }
static void fbits(Fr *m, int32_t v, int n) { int i; for (i = 0; i < n; i++) fraw(m, (v >> (i < 31 ? i : 31)) & 1); }
static unsigned ceil_div(unsigned a, unsigned b) { return (a + b - 1) / b; }

/* the frame of the descriptor d, `v92` selects the V.92 variant; returns the length */
static unsigned model_frame(Fr *m, const DilDesc *d, int v92, unsigned *z1out) {
  unsigned i, j, n1, n2, z;
  int cb[OUTN];
  uint8_t crc[16];
  memset(m, 0, sizeof *m);
  for (i = 0; i < 17; i++) fput(m, i, 1);
  m->pos = 17;
  fbits(m, d->N_, 8);
  for (i = 0; i < 8; i++) fraw(m, 0);
  fbits(m, (int)d->Lsp_ - 1, 7);
  fraw(m, 0);
  fbits(m, (int)d->Ltp_ - 1, 7);
  fraw(m, 0);
  /* zone 1 */
  n1 = 17 * ceil_div(d->Lsp_, 16);
  for (i = 0; i < d->Lsp_; i++) fraw(m, d->sp_[i]);
  while (m->pos < 52 + n1) { fput(m, m->pos, 0); m->pos++; }
  n2 = 17 * ceil_div(d->Ltp_, 16);
  for (i = 0; i < d->Ltp_; i++) fraw(m, d->tp_[i]);
  while (m->pos < 52 + n1 + n2) { fput(m, m->pos, 0); m->pos++; }
  if (m->pos < 52) m->pos = 52;   /* both zones empty: the grid slot 51 is written by the next fraw */
  for (i = 0; i < 16; i += 2) { fbits(m, d->hr_[i], 7); fraw(m, 0); fbits(m, d->hr_[i + 1], 7); fraw(m, 0); }
  for (j = 0; j < ceil_div(d->N_, 2); j++) { fbits(m, d->to_[2 * j], 7); fraw(m, 0); fbits(m, d->to_[2 * j + 1], 7); fraw(m, 0); }
  fgrid(m);
  if (m->pos % 17 != 1) h_die3("model: rows end off the grid (%u)", m->pos);
  z = m->pos - 1;
  *z1out = z;
  if (v92) {
    for (i = 0; i < 16; i++) fraw(m, 1);
    for (i = 0; i < 16; i++) { if (i == 0) fgrid(m); fput(m, m->pos, 0); m->pos++; }
    fgrid(m);
    z = m->pos - 1;
  }
  for (i = 0; i < OUTN && i < z; i++) cb[i] = m->spec[i] ? m->exp[i] : 0;
  model_crc(cb, z, crc);
  for (i = 0; i < 16; i++) fraw(m, crc[i]);
  fgrid(m);
  if (m->pos & 1) { fput(m, m->pos, 0); m->pos++; }
  return m->pos;
}

enum { CV_S_T0 = 0, CV_S_T1, CV_S_EDP0, CV_S_EDP1, CV_S_HASH,
       CV_G_P0, CV_G_P1, CV_G_SEG0, CV_G_SEGMID, CV_G_SEGEND, CV_G_NEG, CV_G_BIG, CV_G_LIVE, CV_G_DEAD, CV_G_NULL, CV_G_N0, CV_G_SEG8, CV_G_XCHECK, CV_G_DEADBIG,
       CV_K_CALL, CV_K_LSP0, CV_K_LTP0, CV_K_N0, CV_K_NODD, CV_K_NEVEN, CV_K_ZODD, CV_K_ZEVEN, CV_K_INS1, CV_K_INS2, CV_K_L16, CV_K_L255, CV_K_ONES, CV_K_BIGTO,
       CV_V_LVL0, CV_V_LVL2, CV_V_HOOK, CV_V_FULL, CV_V_CHAINBREAK };

/* ================================================================================================ scenario 0: setDilDescriptor */
static void sc_set(void) {
  int trial;
  static const unsigned long long want_hash[2] = {0xcc95b27054db5b9eull, 0x40025783559e0892ull};   /* the expected images, from the bytes of the blob's .data (extract script) */
  cov_def(CV_S_T0, "type 0"); cov_def(CV_S_T1, "type 1"); cov_def(CV_S_EDP0, "ADI message"); cov_def(CV_S_EDP1, "ADI_QC message"); cov_def(CV_S_HASH, "expected image hash checked");
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 400; trial++) {
    DilDesc *d;
    int t = trial & 1;
    DilDesc pre;
    h_pool_reset();
    d = (DilDesc *)onew_canary(sizeof(DilDesc));
    if (trial >= 2) garbage_fill(d, sizeof *d, h_rnd());
    memcpy(&pre, d, sizeof pre);
    g_ed_in_call = 0;
    set_dil(d, t);
    if (t == 0) COV(CV_S_T0); else COV(CV_S_T1);
    if (g_ed_in_call != 1) h_die3("setDilDescriptor: %u messages", g_ed_in_call);
    if (strcmp(g_ed_text[0], t == 0 ? "V90DilDescriptorSettings: DIL descriptor set to option ADI.\\r\\n" : "V90DilDescriptorSettings: DIL descriptor set to option ADI_QC.\\r\\n")) h_die3("message '%s'", g_ed_text[0]);
    if (t == 0) COV(CV_S_EDP0); else COV(CV_S_EDP1);
    if (trial < 2) {   /* the 0xa5-filled descriptor: the image must be the one extracted from the .data bytes */
      uint64_t h = fnv_upd(FNV0, d, sizeof *d);
      uint8_t ref[sizeof(DilDesc)];
      memset(ref, 0xa5, sizeof ref);
      if (h != want_hash[t]) h_die3("setDilDescriptor(%d): image hash %016llx, extracted %016llx", t, (unsigned long long)h, want_hash[t]);
      COV(CV_S_HASH);
    }
    /* fields not written must keep their garbage: the written ranges are 0..2, 3 + Lsp, 0x83 + Ltp, 0x103 .. 0x112, 0x113 + N */
    {
      unsigned i;
      for (i = 0; i < sizeof pre; i++) {
        int written = i < 3 || (i >= 3 && i < 3u + d->Lsp_) || (i >= 0x83 && i < 0x83u + d->Ltp_) || (i >= 0x103 && i < 0x113) || (i >= 0x113 && i < 0x113u + d->N_);
        if (!written && ((uint8_t *)d)[i] != ((uint8_t *)&pre)[i]) h_die3("setDilDescriptor changed byte 0x%x outside its ranges", i);
      }
    }
    canary_check(d, sizeof(DilDesc), "setDilDescriptor");
    h_emit(0, "set t %d hash %016llx", t, (unsigned long long)fnv_upd(FNV0, d, sizeof *d));
    DUMPD("set", d);
    h_trial_end("set");
  }
}

/* ================================================================================================ scenario 1: getSegmentPointer / calculateDilLength */
static int model_seg(int p, int v) {
  int i, cnt = p == 1 ? 7 : 8;
  for (i = 0; i < cnt; i++) { int lim = 15 + 16 * i + (p == 1 ? 16 : 0); if (lim >= v) return i; }
  return cnt;
}
static void rnd_desc(DilDesc *d) {
  unsigned k;
  garbage_fill(d, sizeof *d, h_rnd());
  d->N_ = (uint8_t)(h_rpct(15) ? h_rmod(3) : h_rmod(256));
  for (k = 0; k < 256; k++) d->to_[k] = (uint8_t)(h_rpct(70) ? h_rmod(128) : h_rmod(256));
  if (h_rpct(20)) for (k = 0; k < 256; k++) d->to_[k] = (uint8_t)(k * 3);
  if (h_rpct(40)) for (k = 0; k < 16; k++) d->hr_[k] = (uint8_t)h_rmod(40);
}
static void sc_len(void) {
  int trial, v, p;
  cov_def(CV_G_P0, "pcm 0"); cov_def(CV_G_P1, "pcm 1"); cov_def(CV_G_SEG0, "segment 0"); cov_def(CV_G_SEGMID, "a middle segment"); cov_def(CV_G_SEGEND, "past every limit"); cov_def(CV_G_NEG, "negative value");
  cov_def(CV_G_BIG, "huge value"); cov_def(CV_G_LIVE, "calculateDilLength(descriptor)"); cov_def(CV_G_DEAD, "calculateDilLength(type)"); cov_def(CV_G_NULL, "NULL descriptor"); cov_def(CV_G_N0, "N == 0");
  cov_def(CV_G_SEG8, "segment 8 (H[8] = REF[0])"); cov_def(CV_G_XCHECK, "dead == live on a setDilDescriptor image"); cov_def(CV_G_DEADBIG, "type > 1");
  h_scenario_begin(0);
  relax_reserved();
  /* getSegmentPointer: every value around the limits for both types (the signed compare: negative values and values >= 2^31 - 1) */
  for (p = 0; p < 2; p++) {
    static const int ext[] = {-2147483647 - 1, -2147483647, -65536, -300, -2, -1, 0, 1, 14, 15, 16, 30, 31, 32, 47, 48, 63, 64, 79, 80, 95, 96, 111, 112, 126, 127, 128, 129, 255, 256, 65535, 2147483646, 2147483647};
    unsigned e;
    for (v = -300; v <= 300; v++) {
      int g = seg_ptr(p, v), w = model_seg(p, v);
      if (g != w) h_die3("getSegmentPointer(%d, %d) = %d, model %d", p, v, g, w);
      h_emit(0, "seg %d %d -> %d", p, v, g);
      if (p == 0) COV(CV_G_P0); else COV(CV_G_P1);
      if (g == 0) COV(CV_G_SEG0); else if (g == (p == 1 ? 7 : 8)) COV(CV_G_SEGEND); else COV(CV_G_SEGMID);
    }
    for (e = 0; e < sizeof ext / sizeof ext[0]; e++) {
      int g = seg_ptr(p, ext[e]), w = model_seg(p, ext[e]);
      if (g != w) h_die3("getSegmentPointer(%d, %d) = %d, model %d", p, ext[e], g, w);
      h_emit(0, "seg %d %d -> %d", p, ext[e], g);
      if (ext[e] < 0) COV(CV_G_NEG);
      if (ext[e] > 65535 || ext[e] < -65535) COV(CV_G_BIG);
    }
  }
  for (trial = 0; trial < 3000; trial++) {
    DilDesc *d;
    int pc = (int)h_rmod(2), got, want = 0;
    unsigned j;
    h_pool_reset();
    d = (DilDesc *)onew_canary(sizeof(DilDesc));
    if (trial % 7 == 0) {   /* a null descriptor */
      got = dil_len_d(NULL, pc);
      EXPECT_EQ("calculateDilLength(NULL)", got, 0);
      COV(CV_G_NULL);
      h_emit(0, "len null %d -> %d", pc, got);
    } else if (trial % 7 == 1) {   /* the dead overload; also: on the image setDilDescriptor makes it equals the live one */
      int t = (int)h_rmod(8);
      if (h_rpct(20)) t = (int)(0x7fffffffu - h_rmod(3));
      got = dil_len_t(t, pc);
      COV(CV_G_DEAD);
      if (t > 1) { EXPECT_EQ("calculateDilLength(type > 1)", got, 0); COV(CV_G_DEADBIG); }
      else {
        garbage_fill(d, sizeof *d, h_rnd());
        g_ed_in_call = 0;
        set_dil(d, t);
        want = dil_len_d(d, pc);
        EXPECT_EQ("calculateDilLength(type) vs (set descriptor)", got, want);
        COV(CV_G_XCHECK);
      }
      h_emit(0, "len type %d %d -> %d", t, pc, got);
    } else {
      rnd_desc(d);
      if (d->N_ == 0) COV(CV_G_N0);
      for (j = 0; j < d->N_; j++) { int s = model_seg(pc, d->to_[j]); want += 6 + 6 * (s < 8 ? d->hr_[s] : d->hr_[8]); if (s == 8) COV(CV_G_SEG8); }
      got = dil_len_d(d, pc);
      EXPECT_EQ("calculateDilLength(d)", got, want);
      COV(CV_G_LIVE);
      h_emit(0, "len live %d N %u -> %d", pc, d->N_, got);
    }
    canary_check(d, sizeof(DilDesc), "calculateDilLength");
    h_trial_end("len");
  }
  (void)v;
}

/* ================================================================================================ scenarios 2 / 3: the packers */
static unsigned g_hook_at, g_hook_to;
static void hook_lower(unsigned idx) { if (idx == g_hook_at) dsplibs_debug_level = g_hook_to; }

static void rnd_pack_desc(DilDesc *d, int trial, int *swept) {
  unsigned k;
  static const unsigned edge[] = {0, 1, 2, 15, 16, 17, 31, 32, 33, 47, 48, 49, 64, 100, 119, 120, 121, 127, 128, 129, 143, 144, 145, 254, 255};
  garbage_fill(d, sizeof *d, h_rnd());
  d->N_ = (uint8_t)(h_rpct(30) ? edge[h_rmod(sizeof edge / sizeof edge[0])] : (h_rpct(50) ? 0x90 : h_rmod(256)));
  d->Lsp_ = (uint8_t)(h_rpct(40) ? edge[h_rmod(sizeof edge / sizeof edge[0])] : (h_rpct(30) ? 0x3c : h_rmod(256)));
  d->Ltp_ = (uint8_t)(h_rpct(40) ? edge[h_rmod(sizeof edge / sizeof edge[0])] : (h_rpct(30) ? 0x78 : h_rmod(256)));
  if (trial < 768) {   /* every byte value of each of N / Lsp / Ltp (proves the ceilings of the blob's floating-point row counts) */
    unsigned v = (unsigned)trial & 255;
    if (trial < 256) d->N_ = (uint8_t)v; else if (trial < 512) d->Lsp_ = (uint8_t)v; else d->Ltp_ = (uint8_t)v;
    *swept = 1;
  }
  switch (h_rmod(5)) {
    case 0: for (k = 0; k < 128; k++) { d->sp_[k] = (uint8_t)(h_rnd() & 1); d->tp_[k] = (uint8_t)(h_rnd() & 1); } for (k = 0; k < 256; k++) d->to_[k] = (uint8_t)(h_rnd() & 0x7f); break;
    case 1: for (k = 0; k < 128; k++) { d->sp_[k] = 1; d->tp_[k] = 1; } break;
    case 2: for (k = 0; k < 128; k++) { d->sp_[k] = 0; d->tp_[k] = 0; } break;
    default: break;
  }
  if (h_rpct(25)) for (k = 0; k < 16; k++) d->hr_[k] = (uint8_t)(h_rpct(50) ? 0 : 0xff);
}

static void sc_pack(int v92) {
  int trial;
  cov_def(CV_K_CALL, "call"); cov_def(CV_K_LSP0, "Lsp == 0"); cov_def(CV_K_LTP0, "Ltp == 0"); cov_def(CV_K_N0, "N == 0"); cov_def(CV_K_NODD, "N odd"); cov_def(CV_K_NEVEN, "N even");
  cov_def(CV_K_ZODD, "last grid index odd (length + 19)"); cov_def(CV_K_ZEVEN, "last grid index even (length + 18)"); cov_def(CV_K_INS1, "start bit inserted inside zone 1"); cov_def(CV_K_INS2, "start bit inserted inside zone 2");
  cov_def(CV_K_L16, "Lsp a multiple of 16"); cov_def(CV_K_L255, "Lsp or Ltp 255"); cov_def(CV_K_ONES, "all-one symbols"); cov_def(CV_K_BIGTO, "TO rows with N >= 0x90");
  if (v92) { cov_def(CV_V_LVL0, "level < 2"); cov_def(CV_V_LVL2, "level >= 2"); cov_def(CV_V_HOOK, "level lowered inside print k"); cov_def(CV_V_FULL, "all 8 messages"); cov_def(CV_V_CHAINBREAK, "entry chain cut short"); }
  h_scenario_begin(0);
  relax_reserved();
  for (trial = 0; trial < 2600; trial++) {
    DilDesc *d, dpre;
    uint8_t *raw;
    short *o90 = NULL, *l90 = NULL;
    unsigned char *o92 = NULL;
    int *l92 = NULL;
    short want90[OUTN], pre90[OUTN];
    unsigned char want92[OUTN], pre92[OUTN];
    Fr m;
    unsigned len, k, z1, z, lvl, nmsg = 0, expn = 0;
    int swept = 0, wantlen;
    char expt[16][200];
    h_pool_reset();
    d = (DilDesc *)onew_canary(sizeof(DilDesc));
    rnd_pack_desc(d, trial, &swept);
    memcpy(&dpre, d, sizeof dpre);
    raw = (uint8_t *)onew((size_t)OUTN * 2 + 128);
    memset(raw, 0xc3, 64);
    memset(raw + 64 + (size_t)OUTN * (v92 ? 1 : 2), 0xc3, 64);
    if (!v92) { o90 = (short *)(raw + 64); l90 = (short *)onew_canary(8); *l90 = (short)0x5a5a; for (k = 0; k < OUTN; k++) o90[k] = (short)(0x7100 + (k & 0xff)); memcpy(pre90, o90, sizeof pre90); }
    else { o92 = raw + 64; l92 = (int *)onew_canary(8); *l92 = 0x5a5a5a5a; for (k = 0; k < OUTN; k++) o92[k] = (unsigned char)(0x71 + (k & 0x7f)); memcpy(pre92, o92, sizeof pre92); }
    len = model_frame(&m, d, v92, &z1);
    z = (v92 ? z1 + 34 : z1);
    wantlen = (int)len;
    lvl = h_rpct(10) ? (h_rpct(50) ? 0x80000000u + h_rmod(3) : 0xffffffffu - h_rmod(2)) : (h_rpct(60) ? 2 + h_rmod(2) : h_rmod(2));
    if (!v92) lvl = 0;
    dsplibs_debug_level = lvl;
    g_dbg_hook = NULL;
    if (v92 && lvl > 1 && h_rpct(35)) { g_dbg_hook = hook_lower; g_hook_at = h_rmod(8); g_hook_to = h_rmod(2); COV(CV_V_HOOK); }
    g_dbg_in_call = 0; g_ed_in_call = 0;
    g_k_d = (unsigned)(uintptr_t)d;
    g_k_out = (unsigned)(uintptr_t)(v92 ? (void *)o92 : (void *)o90);
    g_k_len = (unsigned)(uintptr_t)(v92 ? (void *)l92 : (void *)l90);
    if (v92) {
      /* expected print sequence: simulate the guards (each re-reads the unsigned level; the hook lowers it after print idx) */
      unsigned lv = lvl, idx = 0, chain;
      unsigned char pr = 0;
      const char *msg[8];
      static char tail[2][200];
      msg[0] = "## Debug: pParamObj address = D\\r\\n"; msg[1] = "## Debug: BitVector address = OUT\\r\\n"; msg[2] = "## Debug: Cnt address = LEN\\r\\n";
      snprintf(expt[3], 200, "## Debug: pParamObj->N = %d\\r\\n", d->N_); snprintf(expt[4], 200, "## Debug: pParamObj->Lsp = %d\\r\\n", d->Lsp_); snprintf(expt[5], 200, "## Debug: pParamObj->Ltp = %d\\r\\n", d->Ltp_);
      snprintf(tail[0], 200, "#%%# Debug: last bit before V.92 info = %u\\r\\n", z1);
      snprintf(tail[1], 200, "#%%# Debug: last bit after V.92 info = %u, temp calculation = %u\\r\\n", z, z1 + 33);
      msg[3] = expt[3]; msg[4] = expt[4]; msg[5] = expt[5]; msg[6] = tail[0]; msg[7] = tail[1];
      (void)pr;
      for (chain = 0; chain < 8; chain++) {
        if (!(lv > 1)) { if (chain < 6) { COV(CV_V_CHAINBREAK); chain = 5; } continue; }
        strcpy(expt[expn], msg[chain]); expn++;
        if (g_dbg_hook && idx == g_hook_at) lv = g_hook_to;
        idx++;
      }
      if (expn == 8) COV(CV_V_FULL);
      if (lvl > 1) COV(CV_V_LVL2); else COV(CV_V_LVL0);
    }
    if (!v92) pack90(d, o90, l90); else pack92(d, o92, l92);
    g_dbg_hook = NULL;
    COV(CV_K_CALL);
    /* compare: every model position, the untouched positions, the guards, the length cell, the descriptor */
    if (!v92) {
      memcpy(want90, pre90, sizeof want90);
      for (k = 0; k < OUTN; k++) if (m.spec[k]) want90[k] = (short)m.exp[k];
      for (k = 0; k < OUTN; k++) if (o90[k] != want90[k]) h_die3("V90 packer: element %u is 0x%04x, model 0x%04x (N %u Lsp %u Ltp %u; model length %u)", k, (unsigned short)o90[k], (unsigned short)want90[k], d->N_, d->Lsp_, d->Ltp_, len);
      EXPECT_EQ("V90 packer length", (unsigned short)*l90, (unsigned)wantlen);
      for (k = 0; k < 64; k++) if (raw[k] != 0xc3 || raw[64 + (size_t)OUTN * 2 + k] != 0xc3) h_die3("V90 packer wrote outside its buffer (guard %u)", k);
      canary_check(l90, 8, "V90 packer length cell");
    } else {
      memcpy(want92, pre92, sizeof want92);
      for (k = 0; k < OUTN; k++) if (m.spec[k]) want92[k] = (unsigned char)m.exp[k];
      for (k = 0; k < OUTN; k++) if (o92[k] != want92[k]) h_die3("V92 packer: byte %u is 0x%02x, model 0x%02x (N %u Lsp %u Ltp %u; model length %u)", k, o92[k], want92[k], d->N_, d->Lsp_, d->Ltp_, len);
      EXPECT_EQ("V92 packer length", (unsigned)*l92, (unsigned)wantlen);
      for (k = 0; k < 64; k++) if (raw[k] != 0xc3 || raw[64 + (size_t)OUTN + k] != 0xc3) h_die3("V92 packer wrote outside its buffer (guard %u)", k);
      canary_check(l92, 8, "V92 packer length cell");
      nmsg = g_dbg_in_call;
      if (nmsg != expn) h_die3("V92 packer printed %u messages, model %u (level %u)", nmsg, expn, lvl);
      for (k = 0; k < expn; k++) if (strcmp(g_dbg_text[k], expt[k])) h_die3("V92 message %u: '%s', model '%s'", k, g_dbg_text[k], expt[k]);
    }
    if (g_ed_in_call) h_die3("packer called edprintf");
    if (memcmp(d, &dpre, sizeof dpre)) h_die3("packer changed the descriptor");
    canary_check(d, sizeof(DilDesc), "packer descriptor");
    if (!d->Lsp_) COV(CV_K_LSP0);
    if (!d->Ltp_) COV(CV_K_LTP0);
    if (!d->N_) COV(CV_K_N0);
    if (d->N_ & 1) COV(CV_K_NODD); else COV(CV_K_NEVEN);
    if (z & 1) COV(CV_K_ZODD); else COV(CV_K_ZEVEN);
    if (d->Lsp_ >= 17 && d->Lsp_ >= 1) COV(CV_K_INS1);
    if (d->Ltp_ >= 17) COV(CV_K_INS2);
    if (d->Lsp_ && (d->Lsp_ & 15) == 0) COV(CV_K_L16);
    if (d->Lsp_ == 255 || d->Ltp_ == 255) COV(CV_K_L255);
    if (d->N_ >= 0x90) COV(CV_K_BIGTO);
    { unsigned i, all = 1; for (i = 0; i < d->Lsp_; i++) if (d->sp_[i] != 1) all = 0; if (all && d->Lsp_) COV(CV_K_ONES); }
    h_emit(0, "pack%d N %u Lsp %u Ltp %u len %d hash %016llx", v92 ? 92 : 90, d->N_, d->Lsp_, d->Ltp_, wantlen,
           (unsigned long long)(v92 ? fnv_upd(FNV0, o92, (size_t)OUTN) : fnv_upd(FNV0, o90, (size_t)OUTN * 2)));
    (void)swept;
    h_trial_end("pack");
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P13_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P13_DBG")) g_show = 1;
  p13_layout_check();
  h_x87_check();
  h_arena_init();
  {
    int i, n = 0;
    for (i = 0; i < NF_DilDesc; i++) {
      HFld e = f_DilDesc[i];
      if (e.kind == 'b' && e.count > 64) e.kind = 'x';
      g_fd[n++] = e;
    }
    g_nfd = n;
  }
  switch (sc) {
    case 0: sc_set(); break;
    case 1: sc_len(); break;
    case 2: sc_pack(0); break;
    case 3: sc_pack(1); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu edprintf %u dbgprintf %u", h_nmalloc, h_nfree, g_nedp, g_ndbg);
  h_cov_finish();
  return 0;
}
