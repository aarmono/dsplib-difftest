#ifndef R24_COMMON_H
#define R24_COMMON_H
/*
 * R24 differential harness: the V.17 receiver's decision callbacks (FAX_FSE_decision_16pt / _32pt / _64pt / _128pt / _AB, FSE_Bridge_det, FSE_decision_eqtrn) of the real dsplibs.o (-DTEST_BLOB, -m32)
 * vs src/fax/fax_v17_decision.c + src/fax/fax_v17_viterbi.c + src/fax/fax_v17_decision_tables.c.  Everything printed is architecture independent (ints only, no pointers): the 32-bit BLOB run is the golden of x86-64.
 * Integer only (no libm).  The output is a canonical hash (R24_VERBOSE=1 prints every record, R24_SEED=n other input streams); coverage counters exit 2 on a zero, 3 = harness violation.
 *
 * Two builds (the scripts define them):
 *   -DR24_REC   scenarios 0 .. 3 (id r24_dec): the Viterbi decoder is a harness RECORDER `VTB_decoder` (the blob's own is weakened with objcopy for the blob run): it logs its arguments and the VTB
 *               state it reads, returns a value that depends on every field and mutates the state, so the call arguments, the typed-VTB field mapping (the x86-64 scratch image of src/fax/fax_v17_viterbi.c)
 *               and the state write-back are all checked.
 *   -DR24_REAL  scenarios 4 .. 5 (id r24_vtb): the REAL Viterbi decoder (blob: its own; port: the ported VTB_decoder of src/v32/v32_datapath.c, at -m32 called directly, on x86-64 through the scratch
 *               image of FAXVTB_decoder), the decisions on all four trellis families and the typed front end FAXVTB_decoder directly.
 *
 * Scenarios:  0 tables (every DECv17_* table hashed, FSEv17_decision identity, sizes)
 *             1 the four trellis decisions: random inputs (full range, near points, midpoints / ties, threshold boundaries by inverse rotation, scans), randomised history objects, idx, state pokes
 *             2 FAX_FSE_decision_AB / FSE_Bridge_det / FSE_decision_eqtrn: random states + counters (wrap, limits, flag, sel), then the whole chain AB -> eqtrn -> Bridge / trellis by cfg.cb
 *             3 call-counter wrap and boundary states for all seven functions
 *             4 the decisions with the REAL Viterbi decoder (every family x every decision), streams
 *             5 FAXVTB_decoder (blob: VTB_decoder on the raw object) on random typed objects, all four families
 */
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fax/fax_v17_decision.h"
#include "fax/fax_v17_decision_tables.h"
#include "v32/v32.h"
#ifdef R24_REAL
#include "v32/v32_viterbi_tables.h"
#endif

typedef unsigned long long u64;

/* ------------------------------------------------------------------------------------------------ canonical output */
static u64 g_hash = 1469598103934665603ULL, g_lines;
static int g_verbose;
static void hash_u32(unsigned v)
{
  int k;
  for (k = 0; k < 4; k++) {
    g_hash ^= (v >> (8 * k)) & 0xffu;
    g_hash *= 1099511628211ULL;
  }
}
/* one record: a tag and n ints */
static void rec(int tag, const int *v, int n)
{
  int i;
  hash_u32((unsigned)tag);
  hash_u32((unsigned)n);
  for (i = 0; i < n; i++) hash_u32((unsigned)v[i]);
  g_lines++;
  if (g_verbose) {
    printf("%llu t%d", g_lines, tag);
    for (i = 0; i < n; i++) printf(" %d", v[i]);
    printf("\n");
  }
}
static void emit(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void emit(const char *fmt, ...)
{
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  {
    const char *q;
    for (q = buf; *q; q++) {
      g_hash ^= (unsigned char)*q;
      g_hash *= 1099511628211ULL;
    }
  }
  g_lines++;
  puts(buf);
}
static void die3(const char *fmt, ...) __attribute__((noreturn, format(printf, 1, 2)));
static void die3(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  fprintf(stderr, "HARNESS VIOLATION: ");
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  va_end(ap);
  exit(3);
}
static unsigned g_lcg = 0x2545f491u;
static unsigned rnd(void)
{
  unsigned x;
  g_lcg = g_lcg * 1664525u + 1013904223u;
  x = g_lcg;
  x ^= x >> 15;
  x *= 0x2c1b3c6du;
  x ^= x >> 12;
  x *= 0x297a2d39u;
  x ^= x >> 15;
  return x;
}
static unsigned rmod(unsigned m) { return m ? rnd() % m : 0; }
static int rpct(int pct) { return (int)rmod(100) < pct; }
static int rrange(int lo, int hi) { return lo + (int)rmod((unsigned)(hi - lo + 1)); }
static short rs16(void) { return (short)rnd(); }

/* ------------------------------------------------------------------------------------------------ coverage counters (observable classes) */
enum {
  CV_PT16 = 0,        /* 16 */
  CV_PT32 = 16,       /* 32 */
  CV_PT64 = 48,       /* 64 */
  CV_PT128 = 112,     /* 128 */
  CV_AB_A = 240, CV_AB_B, CV_AB_RESET, CV_AB_C44, CV_AB_NORESET_BIG,
  CV_EQ_PT0, CV_EQ_PT1, CV_EQ_PT2, CV_EQ_PT3, CV_EQ_END_FLAG0, CV_EQ_END_FLAG1, CV_EQ_MID,
  CV_BR_SW0, CV_BR_SW1, CV_BR_SW2, CV_BR_SW3, CV_BR_PT0, CV_BR_PT1, CV_BR_PT2, CV_BR_PT3,
  CV_WRAP_16, CV_WRAP_32, CV_WRAP_64, CV_WRAP_128, CV_WRAP_BR, CV_WRAP_EQ,
  CV_CHAIN_AB_EQ, CV_CHAIN_EQ_BR, CV_CHAIN_EQ_T16, CV_CHAIN_EQ_T32, CV_CHAIN_EQ_T64, CV_CHAIN_EQ_T128, CV_CHAIN_BR_T16, CV_CHAIN_BR_T32, CV_CHAIN_BR_T64, CV_CHAIN_BR_T128,
  CV_VTB_CALLS, CV_VTB_FAM1, CV_VTB_FAM2, CV_VTB_FAM3, CV_VTB_FAM4,
  CV_MAX
};
static unsigned long g_cov[CV_MAX];
static const char *g_covname[CV_MAX];
static int g_covlo[8], g_covhi[8], g_ncovr;
static void cov_range(int lo, int hi) { g_covlo[g_ncovr] = lo; g_covhi[g_ncovr++] = hi; }
static void cov_def(int id, const char *name) { g_covname[id] = name; }
static void cov_hit(int id) { g_cov[id]++; }
static void cov_finish(void)
{
  int i, bad = 0, r;
  for (i = 0; i < CV_MAX; i++) {
    if (!g_covname[i]) continue;
    emit("COV %-40s %lu", g_covname[i], g_cov[i]);
    if (g_cov[i] == 0) {
      fprintf(stderr, "COVERAGE FAIL: %s was never reached\n", g_covname[i]);
      bad = 1;
    }
  }
  for (r = 0; r < g_ncovr; r++) {
    int lo = g_covlo[r], hi = g_covhi[r], missing = 0;
    for (i = lo; i < hi; i++)
      if (g_cov[i] == 0) missing++;
    emit("COVR %d..%d missing %d", lo, hi - 1, missing);
    if (missing) {
      fprintf(stderr, "COVERAGE FAIL: %d of the points %d..%d never decided\n", missing, lo, hi - 1);
      bad = 1;
    }
  }
  emit("HASH %016llx lines %llu", g_hash, g_lines);
  fflush(stdout);
  if (bad) exit(2);
}

/* ------------------------------------------------------------------------------------------------ callee recorders */
int dsplibs_debug_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int dsplibs_debug_printf(const char *fmt, ...)
{
  char b[256];
  char *q;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  for (q = b; *q; q++)
    if (*q == '\n') *q = '|';
  emit("DBG %s", b);
  return 0;
}
extern unsigned int dsplibs_debug_level;

#ifdef R24_REC
/* the Viterbi decoder recorder: arguments and every field of the VTB object it can see (the blob's raw offsets at sub), then a state-changing, field-dependent result */
void VTB_decoder(char *sub, short i, short q, short *out);
void VTB_decoder(char *sub, short i, short q, short *out)
{
  char *p = (char *)(intptr_t)sub;
  short *m = (short *)(p + 0x04);
  int v[24], k, acc;
  v[0] = i;
  v[1] = q;
  for (k = 0; k < 8; k++) v[2 + k] = m[k];
  v[10] = *(short *)(p + 0x14);
  v[11] = *(int *)(p + 0x18);
  v[12] = *(int *)(p + 0x1c);
  v[13] = *(int *)(p + 0x20);
  v[14] = *(short *)(p + 0x24);
  v[15] = *(int *)(p + 0x28);
  v[16] = *(short *)(p + 0x2c);
  v[17] = *(short *)(p + 0x2e);
  v[18] = *(short *)(p + 0x30);
  v[19] = *(unsigned short *)(p + 0x32);
  v[20] = *(short *)(p + 0x34);
  rec(900, v, 21);
  acc = i * 31 + q * 17 + m[0] + m[7] * 5 + v[10] * 3 + v[14] * 11 + v[16] + v[18] * 7 + v[19] + v[20] * 13 + v[11] + v[12] * 3 + v[13] * 5 + v[15] * 7 + v[17];
  *out = (short)acc;
  for (k = 0; k < 8; k++) m[k] = (short)(m[k] + (i ^ k));
  *(short *)(p + 0x14) = (short)((*(short *)(p + 0x14) + 1) & 15);
  *(short *)(p + 0x30) = (short)(*(short *)(p + 0x30) * 3 + q);
  cov_hit(CV_VTB_CALLS);
}
#endif

/* ------------------------------------------------------------------------------------------------ objects under test */
static FaxFse g_fse;
static short g_oi[32], g_oq[32];
/* RS9: FaxDec17's first word is the pointer `sgd` only (it used to be an anonymous union { int w00; FaxSgd *sgd; }).  The harness still dumps and randomises the whole word as an int
 * (the decisions never touch it), through an int-view union that wraps the object; g_h is the object, H_W00 the int view: the same low 4 bytes at -m32 and -m64 */
static union { FaxDec17 h; struct { int w00; } v; } g_hu;
#define g_h (g_hu.h)
#define H_W00 (g_hu.v.w00)
static short g_ring[0x100]; /* the VTB history ring (0x200 bytes) */

typedef int (*Dec)(FaxFse *, short *, short *);
enum { D16, D32, D64, D128, DAB, DBR, DEQ, NDEC };
static const Dec g_dec[NDEC] = {FAX_FSE_decision_16pt, FAX_FSE_decision_32pt, FAX_FSE_decision_64pt, FAX_FSE_decision_128pt, FAX_FSE_decision_AB, FSE_Bridge_det, FSE_decision_eqtrn};
static const char *const g_decname[NDEC] = {"16pt", "32pt", "64pt", "128pt", "AB", "Bridge", "eqtrn"};

static int cb_id(FaxFseCb cb)
{
  int i;
  if (!cb) return -1;
  for (i = 0; i < NDEC; i++)
    if ((Dec)cb == g_dec[i]) return i;
  return 99;
}

/* point identity of a trellis decision: the table index whose (angle, magnitude) pair equals the output (-1 = none) */
static int point_of_unused(int which, short ang, short mag)
{
  int i, n = which == D16 ? 16 : which == D32 ? 32 : which == D64 ? 64 : 128;
  const short *A = which == D16 ? DECv17_ANGL7200 : which == D32 ? DECv17_ANGL9600T : which == D64 ? DECv17_ANGL12000 : DECv17_ANGL14400;
  const short *M = which == D16 ? NULL : which == D32 ? DECv17_MAG9600T : which == D64 ? DECv17_MAG12000 : DECv17_MAG14400;
  for (i = 0; i < n; i++)
    if (A[i] == ang && (!M || M[i] == mag)) return i;
  return -1;
}

/* a table-address word (CFGP handle / address) is link dependent: in the REAL build the known VTB tables are recorded by index */
static int cfgw(int w)
{
#ifdef R24_REAL
  static const int known[] = {CFGP(VTBv32_IMAP16T), CFGP(VTBv32_QMAP16T), CFGP(VTB_BOUND_7200), CFGP(VTB_REGION_7200), CFGP(VTBv32_IMAP32), CFGP(VTBv32_QMAP32), CFGP(VTB_BOUND_9600), CFGP(VTB_REGION_9600),
                              CFGP(VTBv32_IMAP64), CFGP(VTBv32_QMAP64), CFGP(VTB_BOUND_12000), CFGP(VTB_REGION_12000), CFGP(VTBv32_IMAP128), CFGP(VTBv32_QMAP128), CFGP(VTB_BOUND_14400), CFGP(VTB_REGION_14400)};
  int i;
  for (i = 0; i < 16; i++)
    if (known[i] == w) return 1000 + i;
#endif
  return w;
}

/* the whole observable state: every member (no pointers) of the history object and the FSE block the callbacks can touch */
static int dump_state(int *v, int ret, short ang, short mag)
{
  int n = 0, k;
  v[n++] = ret;
  v[n++] = ang;
  v[n++] = mag;
  v[n++] = g_fse.eqForce;
  v[n++] = g_fse.pllEnable;
  v[n++] = g_fse.predEnable;
  v[n++] = g_fse.eqEnable;
  v[n++] = g_fse.muIdx;
  for (k = 0; k < 3; k++) v[n++] = g_fse.cfg.mu[k];
  v[n++] = cb_id(g_fse.cfg.cb);
  v[n++] = g_fse.idx;
  v[n++] = g_fse.loopState;
  v[n++] = g_fse.freqAcc;
  v[n++] = g_fse.phaseErrAvg;
  v[n++] = g_fse.decErrAvg;
  v[n++] = g_fse.count;
  v[n++] = g_fse.pos;
  v[n++] = H_W00;
  for (k = 0; k < 8; k++) v[n++] = g_h.vtb.metric[k];
  v[n++] = g_h.vtb.wr;
  v[n++] = g_h.vtb.s16;
  v[n++] = cfgw(g_h.vtb.imap);
  v[n++] = cfgw(g_h.vtb.qmap);
  v[n++] = cfgw(g_h.vtb.bound);
  v[n++] = g_h.vtb.sel;
  v[n++] = g_h.vtb.s26;
  v[n++] = cfgw(g_h.vtb.region);
  v[n++] = g_h.vtb.size;
  v[n++] = g_h.vtb.depth;
  v[n++] = g_h.vtb.prevQuad;
  v[n++] = g_h.vtb.lowMask;
  v[n++] = g_h.vtb.shift;
  v[n++] = g_h.vtb.s36;
  v[n++] = g_h.prevAngle;
  v[n++] = g_h.abI0;
  v[n++] = g_h.abQ0;
  v[n++] = g_h.abI1;
  v[n++] = g_h.abQ1;
  v[n++] = g_h.abI2;
  v[n++] = g_h.abQ2;
  v[n++] = g_h.accA;
  v[n++] = g_h.accB;
  v[n++] = g_h.s4e;
  v[n++] = g_h.w50;
  v[n++] = g_h.s54;
  v[n++] = g_h.s56;
  v[n++] = g_h.s58;
  v[n++] = g_h.cnt;
  v[n++] = g_h.reg;
  v[n++] = g_h.flag;
  v[n++] = g_h.sel;
  v[n++] = g_h.s66;
  v[n++] = g_h.ctr;
  v[n++] = g_h.s6a;
  return n;
}
static u64 ring_hash(void)
{
  u64 h = 1469598103934665603ULL;
  int i;
  for (i = 0; i < 0x100; i++) {
    h ^= (unsigned short)g_ring[i] & 0xffu;
    h *= 1099511628211ULL;
    h ^= ((unsigned short)g_ring[i] >> 8) & 0xffu;
    h *= 1099511628211ULL;
  }
  return h;
}

static void fse_link(int idx)
{
  g_fse.outI = g_oi;
  g_fse.outQ = g_oq;
  g_fse.cfg.hist = &g_h;
  g_fse.idx = (short)idx;
}

/* ------------------------------------------------------------------------------------------------ state generators */
static int rval(int mode)
{
  switch (mode) {
    case 0: return (short)rnd();
    case 1: return rrange(-64, 64);
    case 2: return 0;
    default: {
      static const int b[] = {0, 1, -1, 0x7fff, -0x8000, 0x4000, -0x4000, 0x3fff, 0x2000, -0x2000, 0x1fff, 0x2001, 0xff, 0x100, 0x7ffe, -0x7fff, 0x16a0, 0x2d41};
      return b[rmod(sizeof b / sizeof b[0])];
    }
  }
}
static void vtb_family(FaxVtb *v, int fam, short *ring)
{
  memset(v, 0, sizeof *v);
  v->hist = ring;
#ifdef R24_REAL
  v->depth = 0x10;
  v->sel = (short)fam;
  v->shift = (short)fam;
  switch (fam) {
    case 1: v->imap = CFGP(VTBv32_IMAP16T); v->qmap = CFGP(VTBv32_QMAP16T); v->bound = CFGP(VTB_BOUND_7200); v->region = CFGP(VTB_REGION_7200); v->size = 2; v->lowMask = 7; break;
    case 2: v->imap = CFGP(VTBv32_IMAP32); v->qmap = CFGP(VTBv32_QMAP32); v->bound = CFGP(VTB_BOUND_9600); v->region = CFGP(VTB_REGION_9600); v->size = 4; v->lowMask = 0xf; break;
    case 3: v->imap = CFGP(VTBv32_IMAP64); v->qmap = CFGP(VTBv32_QMAP64); v->bound = CFGP(VTB_BOUND_12000); v->region = CFGP(VTB_REGION_12000); v->size = 6; v->lowMask = 0x1f; break;
    default: v->imap = CFGP(VTBv32_IMAP128); v->qmap = CFGP(VTBv32_QMAP128); v->bound = CFGP(VTB_BOUND_14400); v->region = CFGP(VTB_REGION_14400); v->size = 8; v->lowMask = 0x3f; break;
  }
#else
  (void)fam;
#endif
}
/* randomise the history object and the FSE block's relevant members; the recorder build also randomises the VTB fields (any values: the recorder just reads them) */
static void rand_state(int mode)
{
  short *w;
  int k, m;
  memset(&g_h, 0, sizeof g_h);
  memset(&g_fse, 0, sizeof g_fse);
  w = &g_h.abI0;
  (void)w;
  g_h.prevAngle = (unsigned short)rval(rpct(30) ? 3 : mode);
  g_h.abI0 = (short)rval(mode);
  g_h.abQ0 = (short)rval(mode);
  g_h.abI1 = (short)rval(mode);
  g_h.abQ1 = (short)rval(mode);
  g_h.abI2 = (short)rval(mode);
  g_h.abQ2 = (short)rval(mode);
  g_h.accA = (short)rval(rpct(50) ? 1 : mode);
  g_h.accB = (short)rval(rpct(50) ? 1 : mode);
  g_h.s4e = (short)rval(0);
  g_h.w50 = rval(0);
  g_h.s54 = (short)rval(0);
  g_h.s56 = (short)rval(0);
  g_h.s58 = (short)rval(0);
  m = rpct(60) ? 3 : mode;
  g_h.cnt = (short)(rpct(40) ? rrange(-3, 0x120) : rval(m));
  if (rpct(30)) g_h.cnt = (short)(rpct(50) ? 0x3e + rrange(-1, 1) : 0xc8 + rrange(-1, 1));
  {
    int r1 = rval(0), r2 = rval(0); /* sequenced: the order of the operands of one expression is unspecified */
    g_h.reg = rpct(50) ? (int)((unsigned)r1 * 65537u + (unsigned)r2) : r1;
  }
  g_h.flag = rpct(40) ? 0 : rpct(70) ? 1 : rval(0);
  g_h.sel = (short)rrange(0, 3);
  g_h.s66 = (short)rval(0);
  g_h.ctr = (unsigned short)(rpct(40) ? 0x7ffe + rrange(0, 2) : rpct(10) ? 0xffff : rval(0));
  g_h.s6a = (short)rval(0);
  H_W00 = rval(0);
  g_h.vtb.hist = g_ring;
#ifdef R24_REC
  for (k = 0; k < 8; k++) g_h.vtb.metric[k] = (short)rval(mode);
  g_h.vtb.wr = (short)rrange(0, 15);
  g_h.vtb.s16 = (short)rval(0);
  g_h.vtb.imap = rval(0);
  g_h.vtb.qmap = rval(0);
  g_h.vtb.bound = rval(0);
  g_h.vtb.sel = (short)rrange(0, 5);
  g_h.vtb.s26 = (short)rval(0);
  g_h.vtb.region = rval(0);
  g_h.vtb.size = (short)rrange(0, 9);
  g_h.vtb.depth = 16;
  g_h.vtb.prevQuad = (short)rval(0);
  g_h.vtb.lowMask = (unsigned short)rval(0);
  g_h.vtb.shift = (short)rval(0);
  g_h.vtb.s36 = (short)rval(0);
#else
  (void)k;
#endif
  g_fse.eqForce = rval(0);
  g_fse.pllEnable = rval(0);
  g_fse.predEnable = rval(0);
  g_fse.eqEnable = rval(0);
  g_fse.muIdx = (short)rval(0);
  g_fse.cfg.mu[0] = (short)rval(0);
  g_fse.cfg.mu[1] = (short)rval(0);
  g_fse.cfg.mu[2] = (short)rval(0);
  g_fse.cfg.cb = (FaxFseCb)g_dec[rmod(NDEC)];
  g_fse.loopState = (short)rval(0);
  g_fse.freqAcc = rval(0);
  g_fse.phaseErrAvg = (short)rval(0);
  g_fse.decErrAvg = (short)rval(0);
  g_fse.count = (unsigned short)rval(0);
  g_fse.pos = (short)rval(0);
  for (k = 0; k < 32; k++) {
    g_oi[k] = (short)rval(0);
    g_oq[k] = (short)rval(0);
  }
}

/* ------------------------------------------------------------------------------------------------ one call + observable coverage */
static short ang_of_pt4(int k) { return DECv17_ANGL4800[k & 3]; }
static int g_step_ring; /* REAL builds: the VTB history ring is part of every record */

static int step(int which, int x, int y, int idx, short ang_in, short mag_in)
{
  unsigned short ctr0 = g_h.ctr;
  short cnt0 = g_h.cnt;
  int flag0 = g_h.flag, c440 = g_fse.pllEnable, mu1 = g_fse.cfg.mu[1], mi0 = g_fse.muIdx;
  int cb0 = cb_id(g_fse.cfg.cb);
  short a = ang_in, m = mag_in;
  int v[96], n, ret;
  g_oi[idx] = (short)x;
  g_oq[idx] = (short)y;
  fse_link(idx);
  ret = g_dec[which](&g_fse, &a, &m);
  n = dump_state(v, ret, a, m);
  if (g_step_ring) {
    v[n++] = (int)ring_hash();
    v[n++] = (int)(ring_hash() >> 32);
  }
  rec(which, v, n);
  if (which <= D128) {
    int p, np = which == D16 ? 16 : which == D32 ? 32 : which == D64 ? 64 : 128;
    const short *A = which == D16 ? DECv17_ANGL7200 : which == D32 ? DECv17_ANGL9600T : which == D64 ? DECv17_ANGL12000 : DECv17_ANGL14400;
    const short *M = which == D16 ? NULL : which == D32 ? DECv17_MAG9600T : which == D64 ? DECv17_MAG12000 : DECv17_MAG14400;
    for (p = 0; p < np; p++)
      if (A[p] == a && (!M || M[p] == m)) cov_hit((which == D16 ? CV_PT16 : which == D32 ? CV_PT32 : which == D64 ? CV_PT64 : CV_PT128) + p);
    if (ctr0 == 0x7fff) cov_hit(CV_WRAP_16 + which);
  } else if (which == DAB) {
    if (ret == 3) cov_hit(CV_AB_A);
    if (ret == 2) cov_hit(CV_AB_B);
    if (ret == 0) cov_hit(CV_AB_RESET);
    if (c440 != 1 && g_fse.pllEnable == 1) cov_hit(CV_AB_C44);
  } else if (which == DBR) {
    int k;
    if (ctr0 == 0x7fff) cov_hit(CV_WRAP_BR);
    if (cnt0 > 0x3e) cov_hit(CV_BR_SW0 + (g_h.sel & 3));
    for (k = 0; k < 4; k++)
      if (a == DECv17_ANGL4800[k]) cov_hit(CV_BR_PT0 + k);
  } else {
    if (ctr0 == 0x7fff) cov_hit(CV_WRAP_EQ);
    if (ret >= 0 && ret < 4) cov_hit(CV_EQ_PT0 + ret);
    if (g_fse.muIdx == 1 && g_fse.cfg.mu[1] == 0x51f && (mi0 != 1 || mu1 != 0x51f)) cov_hit(flag0 == 0 ? CV_EQ_END_FLAG0 : CV_EQ_END_FLAG1);
    else cov_hit(CV_EQ_MID);
  }
  (void)cb0;
  return ret;
}

/* ------------------------------------------------------------------------------------------------ input generators */
static int g_coords[512], g_ncoords;
static void coords_init(void)
{
  static const short *const tabs[] = {DECv17_IMAP16, DECv17_QMAP16, DECv17_IMAP64, DECv17_QMAP64, DECv17_ANA_IMAP128, DECv17_ANA_QMAP128, DECv17_IMAP4, DECv17_QMAP4, DECv17_ANA_QMAP};
  static const int tn[] = {16, 16, 64, 64, 32, 32, 4, 4, 8};
  int base[256], nb = 0, t, i, j, k;
  static const int thr[] = {0x16a0, 0x2d41, 0x3891, 0x3892, 0x2799, 0x3e3a, 0x32e9, 0x21f1, 0x3891, 0x2000, 0x1fff, 0, 0x7fff};
  for (t = 0; t < 9; t++)
    for (i = 0; i < tn[t]; i++) {
      for (j = 0; j < nb; j++)
        if (base[j] == tabs[t][i]) break;
      if (j == nb && nb < 255) base[nb++] = tabs[t][i];
    }
  for (i = 0; i < (int)(sizeof thr / sizeof thr[0]); i++) {
    for (j = 0; j < nb; j++)
      if (base[j] == thr[i]) break;
    if (j == nb && nb < 255) base[nb++] = thr[i];
  }
  g_ncoords = 0;
  for (i = 0; i < nb; i++)
    for (k = -2; k <= 2; k++)
      if (g_ncoords < 512) g_coords[g_ncoords++] = base[i] + k;
}
static int rcoord(void) { return g_coords[rmod((unsigned)g_ncoords)]; }
static int sat16(int v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : v; }
static void rxy(int *x, int *y)
{
  int m = (int)rmod(10);
  if (m < 3) {
    *x = (short)rnd();
    *y = (short)rnd();
  } else if (m < 4) {
    *x = rrange(-300, 300);
    *y = rrange(-300, 300);
  } else if (m < 8) {
    int c1 = rcoord(), n1 = rpct(50) ? 0 : rrange(-40, 40), c2 = rcoord(), n2 = rpct(50) ? 0 : rrange(-40, 40);
    *x = sat16(c1 + n1);
    *y = sat16(c2 + n2);
    if (rpct(50)) *x = -*x;
    if (rpct(50)) *y = -*y;
  } else {
    *x = rval(3);
    *y = rval(3);
  }
}

/* (x, y) that rotate to (xr, yr) in rotation `sel` (the inverse of the decisions' rotation, then +-2 around it) */
static void unrotate(int sel, int xr, int yr, int *x, int *y)
{
  int cs = DECv17_COS_ROT_ANGLE[sel], sn = DECv17_SIN_ROT_ANGLE[sel];
  *x = sat16((cs * xr + sn * yr) >> 15);
  *y = sat16((cs * yr - sn * xr) >> 15);
}

static void fresh_hist_noise(int mode)
{
  rand_state(mode);
}

/* ------------------------------------------------------------------------------------------------ scenario 0: tables */
static void hash_table(const char *name, const short *t, int n)
{
  u64 h = 1469598103934665603ULL;
  int i;
  for (i = 0; i < n; i++) {
    h ^= (unsigned short)t[i] & 0xffu;
    h *= 1099511628211ULL;
    h ^= ((unsigned short)t[i] >> 8) & 0xffu;
    h *= 1099511628211ULL;
  }
  emit("TABLE %-22s %3d %016llx", name, n, h);
}
#define HT(n) hash_table(#n, n, (int)(sizeof n / sizeof n[0]))
static void scen0(void)
{
  int i;
  HT(DECv17_ANA_IMAP128);
  HT(DECv17_ANA_QMAP);
  HT(DECv17_ANA_QMAP128);
  HT(DECv17_ANGL12000);
  HT(DECv17_ANGL14400);
  HT(DECv17_ANGL4800);
  HT(DECv17_ANGL7200);
  HT(DECv17_ANGL9600T);
  HT(DECv17_COS_ROT_ANGLE);
  HT(DECv17_IMAP16);
  HT(DECv17_IMAP4);
  HT(DECv17_IMAP64);
  HT(DECv17_MAG12000);
  HT(DECv17_MAG14400);
  HT(DECv17_MAG7200);
  HT(DECv17_MAG9600T);
  HT(DECv17_MAP_BRIDGE);
  HT(DECv17_MAP_TRN);
  HT(DECv17_QMAP16);
  HT(DECv17_QMAP4);
  HT(DECv17_QMAP64);
  HT(DECv17_SIN_ROT_ANGLE);
  for (i = 0; i < 4; i++) emit("FSEv17_decision[%d] = %s", i, cb_id(FSEv17_decision[i]) >= 0 && cb_id(FSEv17_decision[i]) < NDEC ? g_decname[cb_id(FSEv17_decision[i])] : "?");
  if (cb_id(FSEv17_decision[0]) != D16 || cb_id(FSEv17_decision[1]) != D32 || cb_id(FSEv17_decision[2]) != D64 || cb_id(FSEv17_decision[3]) != D128) die3("FSEv17_decision order");
}

/* ------------------------------------------------------------------------------------------------ scenario 1: the trellis decisions */
static void trellis_one(int which)
{
  int i, j, t, x, y, sel;
  static const int thrs[] = {0x16a0, 0x16a1, 0x16a2, 0x2d40, 0x2d41, 0x2d42, 0x3891, 0x3892, 0x3893, 0, 1, -1};
  /* a) random states and inputs */
  for (i = 0; i < 60000; i++) {
    rand_state(i % 4);
    rxy(&x, &y);
    {
      int ix = (int)rmod(16);
      short ai = rs16(), mi = rs16();
      step(which, x, y, ix, ai, mi);
    }
  }
  /* b) the lattice: every pair of table coordinates (+-2) x a few random states */
  for (i = 0; i < g_ncoords; i += 1)
    for (j = 0; j < g_ncoords; j += 3) {
      if (((i * 7 + j) & 7) == 0) rand_state(rmod(4));
      step(which, g_coords[i], g_coords[j], 0, 0, 0);
      step(which, -g_coords[i], g_coords[j], 1, 0, 0);
    }
  /* c) thresholds of the rotated plane: for each rotation and each pair of thresholds (+-1) the inverse rotation, +-2 around it */
  for (sel = 0; sel < 4; sel++)
    for (i = 0; i < (int)(sizeof thrs / sizeof thrs[0]); i++)
      for (j = 0; j < (int)(sizeof thrs / sizeof thrs[0]); j++)
        for (t = 0; t < 9; t++) {
          int xr = thrs[i] + (t % 3) - 1, yr = thrs[j] + (t / 3) - 1, x0, y0, dx, dy;
          int sg;
          for (sg = 0; sg < 2; sg++) {
            unrotate(sel, sg ? -xr : xr, sg ? yr : -yr, &x0, &y0);
            for (dx = -2; dx <= 2; dx++)
              for (dy = -2; dy <= 2; dy++) {
                if (((dx + 2) * 5 + dy + 2 + t) % 4 == 0) rand_state(rmod(4));
                step(which, sat16(x0 + dx), sat16(y0 + dy), (dx + 2) * 3, 0, 0);
              }
          }
        }
  /* d) scans: every 5th x for 24 y, every 5th y for 24 x */
  for (j = 0; j < 24; j++) {
    int fixed = rcoord();
    fixed += rrange(-3, 3);
    if (rpct(30)) fixed = rs16();
    rand_state(rmod(4));
    for (x = -32768; x < 32768; x += 5) step(which, x, fixed, 5, 0, 0);
    for (y = -32768; y < 32768; y += 5) step(which, fixed, y, 6, 0, 0);
  }
}
static void scen1(void)
{
  int w;
  for (w = D16; w <= D128; w++) trellis_one(w);
}

/* ------------------------------------------------------------------------------------------------ scenario 2: AB / Bridge / eqtrn and the chain */
static int g_ab_ang;
static void gen_ab_sym(int phase, int *x, int *y, short *ang, short *mag)
{
  static const int px[4] = {4096, -4096, -4096, 4096}, py[4] = {4096, 4096, -4096, -4096};
  if (phase == 0) { /* stationary */
    *x = px[0] + rrange(0, 2);
    *y = py[0] + rrange(0, 2);
    g_ab_ang += rrange(-80, 80);
    *mag = (short)(0x3299 + rrange(-30, 30));
  } else if (phase == 1) { /* the four points */
    int k = (int)rmod(4);
    *x = px[k] + rrange(-60, 60);
    *y = py[k] + rrange(-60, 60);
    g_ab_ang += rpct(50) ? 0x4000 + rrange(-200, 200) : 0x8000 + rrange(-200, 200);
    *mag = (short)(0x3299 + rrange(-900, 900));
  } else { /* wild */
    *x = (short)rnd();
    *y = (short)rnd();
    g_ab_ang += (short)rnd();
    *mag = (short)rnd();
  }
  *ang = (short)g_ab_ang;
}
static void chain_stream(int flag, int sel, int longrun)
{
  int i, n, phase = 0, cb_before, cb_after, x, y, nsteps;
  short ang, mag;
  rand_state(rmod(4));
  g_h.cnt = (short)(rpct(30) ? rrange(0, 0x60) : 0);
  g_h.ctr = (unsigned short)(rpct(20) ? 0x7ff0 : 0);
  g_h.flag = flag;
  g_h.sel = (short)sel;
  g_h.accA = g_h.accB = 0;
  g_h.prevAngle = (unsigned short)rs16();
  g_h.abI0 = g_h.abQ0 = g_h.abI1 = g_h.abQ1 = g_h.abI2 = g_h.abQ2 = 0;
  g_fse.cfg.cb = FAX_FSE_decision_AB;
  g_ab_ang = (short)rnd();
  nsteps = longrun ? 3300 + 400 : 1300;
  n = 0;
  for (i = 0; i < nsteps; i++) {
    int w;
    if (i < 230) phase = rpct(97) ? 0 : 1;
    else if (i < 300) phase = rpct(60) ? 1 : 2;
    else phase = rpct(70) ? 1 : rpct(50) ? 0 : 2;
    gen_ab_sym(phase, &x, &y, &ang, &mag);
    w = cb_id(g_fse.cfg.cb);
    if (w < 0 || w >= NDEC) die3("chain: callback %d", w);
    cb_before = w;
    step(w, x, y, rmod(16), ang, mag);
    cb_after = cb_id(g_fse.cfg.cb);
    if (cb_before == DAB && cb_after == DEQ) cov_hit(CV_CHAIN_AB_EQ);
    if (cb_before == DEQ && cb_after == DBR) cov_hit(CV_CHAIN_EQ_BR);
    if (cb_before == DEQ && cb_after >= D16 && cb_after <= D128) cov_hit(CV_CHAIN_EQ_T16 + cb_after);
    if (cb_before == DBR && cb_after >= D16 && cb_after <= D128) cov_hit(CV_CHAIN_BR_T16 + cb_after);
    n++;
  }
}
static void scen2(void)
{
  int i, x, y, w, k;
  /* AB: random states, with and without the history equal to the symbol (T <= thr) */
  for (i = 0; i < 90000; i++) {
    rand_state(i % 4);
    rxy(&x, &y);
    if (rpct(35)) {
      g_h.abI1 = (short)x;
      g_h.abQ1 = (short)y;
      g_h.abI2 = g_h.abI0;
      g_h.abQ2 = g_h.abQ0;
      if (rpct(50)) {
        g_h.abI2 = (short)x;
        g_h.abQ2 = (short)y;
        g_h.abI0 = (short)x;
        g_h.abQ0 = (short)y;
      }
    }
    if (rpct(25)) g_h.cnt = (short)(0xc8 + rrange(-2, 2));
    if (rpct(15)) g_h.cnt = (short)(0x96 + rrange(-2, 2));
    {
      int ix = (int)rmod(16);
      short ai = rs16(), mi;
      if (rpct(30)) mi = (short)(0x3299 + rrange(-3000, 3000));
      else mi = rs16();
      step(DAB, x, y, ix, ai, mi);
    }
  }
  /* AB: the angle step around +-0x4000 / the 0x8000 wrap, the smoothers */
  for (i = 0; i < 20000; i++) {
    short prev;
    rand_state(rmod(4));
    prev = (short)rnd();
    g_h.prevAngle = (unsigned short)prev;
    g_h.accA = (short)rval(rpct(50) ? 1 : 0);
    g_h.accB = (short)rval(rpct(50) ? 1 : 0);
    g_h.cnt = (short)rrange(0, 0xd0);
    rxy(&x, &y);
    {
      int dlt = rrange(-3, 3), w8 = rpct(30) ? 0x8000 : 0, w16 = rpct(10) ? 0x10000 : 0;
      short mi = rs16();
      step(DAB, x, y, 0, (short)(prev + 0x4000 + dlt + w8 + w16), mi);
    }
  }
  /* AB: T next to the threshold thr = ((A * A + B * B >> 15) * 0x5555) >> 14 (harness-side arithmetic of the smoothers: a small angle step, magnitude 0 keeps A, B' = 31 B >> 5) */
  for (i = 0; i < 40000; i++) {
    int A, B, v, thr, tgt, d1, k, ia;
    short ah, bh;
    rand_state(rmod(4));
    ah = (short)(rpct(20) ? rrange(0, 40) : rrange(0, 19000));
    bh = (short)(rpct(20) ? rrange(0, 40) : rrange(0, 19000));
    A = ah;
    B = (bh * 31) >> 5;
    v = (A * A + B * B) >> 15;
    thr = (v * 0x5555) >> 14;
    tgt = thr + rrange(-2, 2);
    if (tgt < 0) tgt = 0;
    for (d1 = 0; d1 < 32767 && ((d1 + 1) * (d1 + 1)) <= (tgt << 15) && (d1 + 1) * (d1 + 1) >= 0; d1++) {}
    d1 += rrange(-1, 1);
    if (d1 < 0) d1 = 0;
    if (d1 > 32767) d1 = 32767;
    g_h.accA = ah;
    g_h.accB = bh;
    g_h.cnt = (short)(0xc8 + rrange(0, 2));
    g_h.prevAngle = 0x1000;
    g_h.abI0 = (short)rrange(-20000, 20000);
    g_h.abQ0 = (short)rrange(-20000, 20000);
    g_h.abI2 = g_h.abI0;
    g_h.abQ2 = g_h.abQ0;
    x = rrange(-10000, 10000);
    y = rrange(-10000, 10000);
    g_h.abI1 = (short)(x - d1);
    g_h.abQ1 = (short)y;
    ia = 0x1000 + rrange(0, 0x3000);
    for (k = 0; k < 2; k++) {
      step(DAB, x, y, 3, (short)ia, 0);
      g_h.cnt = (short)(0xc8 + rrange(0, 2));
      g_h.accA = ah;
      g_h.accB = bh;
      g_h.prevAngle = 0x1000;
    }
  }
  /* Bridge */
  for (i = 0; i < 80000; i++) {
    rand_state(i % 4);
    if (rpct(40)) g_h.cnt = (short)(0x3e + rrange(-2, 2));
    if (rpct(30)) g_h.sel = (short)rrange(0, 3);
    rxy(&x, &y);
    if (rpct(40)) {
      int sx = rpct(50) ? 1 : -1, mx = rpct(50) ? 4096 : 12288, nx = rrange(-4, 4);
      int sy = rpct(50) ? 1 : -1, my = rpct(50) ? 4096 : 12288, ny = rrange(-4, 4);
      x = sx * mx + nx;
      y = sy * my + ny;
    }
    step(DBR, x, y, (int)rmod(16), 0, 0);
  }
  /* eqtrn */
  for (i = 0; i < 80000; i++) {
    int limit;
    rand_state(i % 4);
    limit = 0x25 + (g_h.flag == 0 ? 0xb7a : 0);
    if (rpct(50)) g_h.cnt = (short)(limit + rrange(-3, 2));
    if (rpct(10)) g_h.flag = 0;
    rxy(&x, &y);
    step(DEQ, x, y, (int)rmod(16), 0, 0);
  }
  /* the training sequence register: 0 / 1 / all patterns of the 7 bits and the 7 + 16 high bits */
  for (i = 0; i < 4096; i++) {
    rand_state(rmod(4));
    g_h.reg = (i < 2048) ? (i << 14) : (int)(((unsigned)(i & 0x7f) << 25) | ((unsigned)(i >> 7) << 21) | rnd() % 3);
    g_h.cnt = 0;
    step(DEQ, 0, 0, 0, 0, 0);
  }
  /* the chain AB -> eqtrn -> Bridge / the trellis decisions (the callback pointer drives it, as FAXFSE_receive does) */
  for (w = 0; w < 16; w++)
    for (k = 0; k < 4; k++) chain_stream(w & 1 ? 0 : 1, ((w >> 1) + k) & 3, 1 + (w & 1));
  for (w = 0; w < 24; w++) chain_stream(w & 1, rrange(0, 3), 0);
}

/* ------------------------------------------------------------------------------------------------ scenario 3: counters and boundary states of all seven */
static void scen3(void)
{
  static const int ctrs[] = {0x7ffe, 0x7fff, 0x8000, 0xffff, 0, 1, 0x3fff, 0x4000, 0x4001, 0x7ffd, 0x8001};
  static const int cnts[] = {0, 1, 0x24, 0x25, 0x26, 0x3d, 0x3e, 0x3f, 0x95, 0x96, 0x97, 0xc7, 0xc8, 0xc9, 0xb9e, 0xb9f, 0xba0, -1, 0x7fff, -0x8000, 0x7ffe};
  int w, i, j, k, x, y;
  for (w = 0; w < NDEC; w++)
    for (i = 0; i < (int)(sizeof ctrs / sizeof ctrs[0]); i++)
      for (j = 0; j < (int)(sizeof cnts / sizeof cnts[0]); j++)
        for (k = 0; k < 6; k++) {
          rand_state(k < 2 ? 3 : (int)rmod(4));
          g_h.ctr = (unsigned short)ctrs[i];
          g_h.cnt = (short)cnts[j];
          g_h.flag = (k & 1) ? 0 : (int)rnd();
          g_h.sel = (short)(k & 3);
          rxy(&x, &y);
          {
            int ix = (int)rmod(16);
            short ai = rs16(), mi = rs16();
            step(w, x, y, ix, ai, mi);
          }
        }
  /* the extremes of the inputs */
  for (w = 0; w < NDEC; w++)
    for (i = 0; i < 4; i++)
      for (j = 0; j < 4; j++) {
        static const int e[4] = {-32768, 32767, -32767, 0};
        rand_state(3);
        step(w, e[i], e[j], 0, (short)e[j], (short)e[i]);
        step(w, e[j], e[i], 15, (short)e[i], (short)e[j]);
      }
}

/* ------------------------------------------------------------------------------------------------ scenario 4: the decisions with the real Viterbi decoder */
#ifdef R24_REAL
static void scen4(void)
{
  int fam, w, s, i, x, y, noise;
  g_step_ring = 1;
  for (fam = 1; fam <= 4; fam++)
    for (w = D16; w <= D128; w++)
      for (s = 0; s < 40; s++) {
        rand_state(rmod(4));
        memset(g_ring, 0, sizeof g_ring);
        vtb_family(&g_h.vtb, fam, g_ring);
        if (rpct(30)) g_h.vtb.prevQuad = (short)rrange(0, 3);
        for (i = 0; i < 4; i++) g_h.vtb.metric[i * 2] = (short)rrange(-100, 100);
        {
          static const int nz[5] = {0, 60, 500, 3000, 12000};
          noise = nz[rmod(5)];
        }
        for (i = 0; i < 500; i++) {
          {
            int gx = rrange(-7, 7), nx = rrange(-noise, noise), gy = rrange(-7, 7), ny = rrange(-noise, noise);
            x = gx * 2048 + nx;
            y = gy * 2048 + ny;
          }
          if (rpct(5)) rxy(&x, &y);
          if (rpct(2)) {
            g_h.vtb.metric[rmod(8)] = (short)rval(0);
            g_h.vtb.prevQuad = (short)rrange(0, 3); /* VTB_DIFF_TBL has 16 entries: prevQuad must be 0 .. 3 (a wild value reads the following .rodata: not comparable) */
          }
          step(w, sat16(x), sat16(y), (int)rmod(16), 0, 0);
          cov_hit(CV_VTB_FAM1 + fam - 1);
        }
      }
}

/* ------------------------------------------------------------------------------------------------ scenario 5: the typed Viterbi front end */
extern void VTB_decoder(char *sub, short i, short q, short *out);
static void run_vtb(FaxVtb *v, short i, short q, short *out)
{
#ifdef TEST_BLOB
  VTB_decoder((char *)(v), i, q, out);
#else
  FAXVTB_decoder(v, i, q, out);
#endif
}
static void scen5(void)
{
  int fam, s, i, k, x, y;
  FaxVtb v;
  for (fam = 1; fam <= 4; fam++)
    for (s = 0; s < 12; s++) {
      memset(g_ring, 0, sizeof g_ring);
      vtb_family(&v, fam, g_ring);
      v.prevQuad = (short)rrange(0, 3);
      for (i = 0; i < 4000; i++) {
        short out = 0x5a5a;
        int r[40], n = 0;
        {
          int gx = rrange(-7, 7), nx = rrange(-300, 300), gy = rrange(-7, 7), ny = rrange(-300, 300);
          x = gx * 2048 + nx;
          y = gy * 2048 + ny;
        }
        if (rpct(15)) rxy(&x, &y);
        if (rpct(3)) v.metric[rmod(8)] = (short)rval(rpct(50) ? 0 : 1);
        if (rpct(3)) v.prevQuad = (short)rrange(0, 3);
        if (rpct(2)) v.wr = (short)rrange(0, 15);
        run_vtb(&v, (short)sat16(x), (short)sat16(y), &out);
        r[n++] = out;
        for (k = 0; k < 8; k++) r[n++] = v.metric[k];
        r[n++] = v.wr;
        r[n++] = v.prevQuad;
        r[n++] = v.sel;
        r[n++] = v.size;
        r[n++] = v.lowMask;
        r[n++] = v.shift;
        r[n++] = v.depth;
        r[n++] = cfgw(v.imap);
        r[n++] = cfgw(v.qmap);
        r[n++] = cfgw(v.bound);
        r[n++] = cfgw(v.region);
        r[n++] = (int)ring_hash();
        r[n++] = (int)(ring_hash() >> 32);
        rec(800 + fam, r, n);
        cov_hit(CV_VTB_FAM1 + fam - 1);
        cov_hit(CV_VTB_CALLS);
      }
    }
}
#endif

/* ------------------------------------------------------------------------------------------------ driver */
static void r24_cov_setup(int scen)
{
  int i;
  static char names[CV_MAX][32];
  (void)names;
  switch (scen) {
    case 1:
      cov_range(CV_PT16, CV_PT16 + 16);
      cov_range(CV_PT32, CV_PT32 + 32);
      cov_range(CV_PT64, CV_PT64 + 64);
      cov_range(CV_PT128, CV_PT128 + 128);
      cov_def(CV_WRAP_16, "ctr wrap 16pt");
      cov_def(CV_WRAP_32, "ctr wrap 32pt");
      cov_def(CV_WRAP_64, "ctr wrap 64pt");
      cov_def(CV_WRAP_128, "ctr wrap 128pt");
#ifdef R24_REC
      cov_def(CV_VTB_CALLS, "VTB recorder calls");
#endif
      break;
    case 2:
      cov_def(CV_AB_A, "AB large angle step");
      cov_def(CV_AB_B, "AB small angle step");
      cov_def(CV_AB_RESET, "AB reset (-> eqtrn)");
      cov_def(CV_AB_C44, "AB sets c44");
      cov_def(CV_EQ_PT0, "eqtrn point 0");
      cov_def(CV_EQ_PT1, "eqtrn point 1");
      cov_def(CV_EQ_PT2, "eqtrn point 2");
      cov_def(CV_EQ_PT3, "eqtrn point 3");
      cov_def(CV_EQ_END_FLAG0, "eqtrn end, flag 0");
      cov_def(CV_EQ_END_FLAG1, "eqtrn end, flag != 0");
      cov_def(CV_EQ_MID, "eqtrn mid sequence");
      for (i = 0; i < 4; i++) {
        static const char *const sw[4] = {"Bridge -> sel 0", "Bridge -> sel 1", "Bridge -> sel 2", "Bridge -> sel 3"};
        static const char *const bp[4] = {"Bridge point 0", "Bridge point 1", "Bridge point 2", "Bridge point 3"};
        cov_def(CV_BR_SW0 + i, sw[i]);
        cov_def(CV_BR_PT0 + i, bp[i]);
      }
      cov_def(CV_WRAP_BR, "ctr wrap Bridge");
      cov_def(CV_WRAP_EQ, "ctr wrap eqtrn");
      cov_def(CV_CHAIN_AB_EQ, "chain AB -> eqtrn");
      cov_def(CV_CHAIN_EQ_BR, "chain eqtrn -> Bridge");
      cov_def(CV_CHAIN_EQ_T16, "chain eqtrn -> 16pt");
      cov_def(CV_CHAIN_EQ_T32, "chain eqtrn -> 32pt");
      cov_def(CV_CHAIN_EQ_T64, "chain eqtrn -> 64pt");
      cov_def(CV_CHAIN_EQ_T128, "chain eqtrn -> 128pt");
      cov_def(CV_CHAIN_BR_T16, "chain Bridge -> 16pt");
      cov_def(CV_CHAIN_BR_T32, "chain Bridge -> 32pt");
      cov_def(CV_CHAIN_BR_T64, "chain Bridge -> 64pt");
      cov_def(CV_CHAIN_BR_T128, "chain Bridge -> 128pt");
      break;
    case 3:
      cov_def(CV_WRAP_16, "ctr wrap 16pt");
      cov_def(CV_WRAP_32, "ctr wrap 32pt");
      cov_def(CV_WRAP_64, "ctr wrap 64pt");
      cov_def(CV_WRAP_128, "ctr wrap 128pt");
      cov_def(CV_WRAP_BR, "ctr wrap Bridge");
      cov_def(CV_WRAP_EQ, "ctr wrap eqtrn");
      cov_def(CV_AB_C44, "AB sets c44");
      cov_def(CV_AB_RESET, "AB reset (-> eqtrn)");
      cov_def(CV_EQ_END_FLAG0, "eqtrn end, flag 0");
      cov_def(CV_EQ_END_FLAG1, "eqtrn end, flag != 0");
      break;
    case 4:
      cov_def(CV_VTB_FAM1, "VTB family 1 (16)");
      cov_def(CV_VTB_FAM2, "VTB family 2 (32)");
      cov_def(CV_VTB_FAM3, "VTB family 3 (64)");
      cov_def(CV_VTB_FAM4, "VTB family 4 (128)");
      cov_range(CV_PT16, CV_PT16 + 16);
      cov_range(CV_PT32, CV_PT32 + 32);
      cov_range(CV_PT64, CV_PT64 + 64);
      cov_range(CV_PT128, CV_PT128 + 128);
      break;
    case 5:
      cov_def(CV_VTB_FAM1, "VTB family 1 (16)");
      cov_def(CV_VTB_FAM2, "VTB family 2 (32)");
      cov_def(CV_VTB_FAM3, "VTB family 3 (64)");
      cov_def(CV_VTB_FAM4, "VTB family 4 (128)");
      cov_def(CV_VTB_CALLS, "VTB direct calls");
      break;
    default: break;
  }
}
static int r24_main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R24_SEED");
  g_verbose = getenv("R24_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)strtoul(sd, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)scen * 7919u;
  coords_init();
  r24_cov_setup(scen);
  dsplibs_debug_level = 2;
  switch (scen) {
#ifdef R24_REC
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
#endif
#ifdef R24_REAL
    case 4: scen4(); break;
    case 5: scen5(); break;
#endif
    default: fprintf(stderr, "scenario not in this build\n"); return 2;
  }
  cov_finish();
  return 0;
}
#endif
