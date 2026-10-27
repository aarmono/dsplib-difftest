/*
 * Differential test harness for port chunk P29 (src/vpcm/pcm_bits_to_symbol.cpp): class V92Transmitter (6 functions), class V90BitsToSymbol (11) and class V92BitsToSymbol (10): the TX bits ->
 * symbols front ends of the V.90 / V.92 modulators.
 *
 * ONE C source, built by test/vpcm/pcm_bits_to_symbol.main.sh in three flavours (-DP29_BIN_A / none = B / -DP29_BIN_C): against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2) and
 * for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out).  The functions under test are reached through __asm__ labels; the objects are the generated C mirrors (test/vpcm/pcm_bits_to_symbol.layout),
 * the V90MappingParams / V92MappingParams blocks the P11 mirrors.  Every callee of the classes is a recorder (the behaviour of these classes IS the call pattern: arguments, order, what is stored between
 * the calls):
 *   flavour A (scenarios 0, 1): V90Mapper (ctor C1 / C2, dtor D1 / D2, reset, resetNoSpectral, process) and V92Transmitter (the same six): the BitsToSymbol queue logic is tested against independent models
 *   flavour B (scenario 2): V92ModulusEncoder, V92Precoder, V92PreFilter, V92ConvolutionEncoder (all their functions the transmitter calls): V92Transmitter itself is the real blob / the port
 *   flavour C (scenario 3, -DP29_BIN_C): NO sub-object recorder: V92Transmitter over the REAL V92ModulusEncoder / V92Precoder / V92PreFilter / V92ConvolutionEncoder (the blob; the P02 / P15 / P16 port objects)
 *
 * Scenarios (each its own process):
 *   0 V90BitsToSymbol: C1 / C2 (store order probed at every malloc), D1 / D2 (also with NULL mapper / queue pointers), reset / resetNoSpectral, setSymbolsBlockSize, nofBitsForNextTime, the three process()
 *     overloads (queue, block hand-over, compaction, overflow / underflow / not-set codes, the bit count of the next call, first_ flag), debug prints at every level
 *   1 V92BitsToSymbol: the same for the V.92 class (divisor 12, no extra_, no resetNoSpectral)
 *   2 V92Transmitter: C1 / C2 (store order at each malloc, the six blocks), D1 / D2 (NULL members; the pre_ clear), reset (the sub-object calls and the debug dump of every parameter: Gain / m0 .. m11 /
 *     arrays: texts against an independent model of the "%c%d.%07d" split, level changes in the middle of the dump), process (bit collection, the three precoder / convolutional-encoder passes with their
 *     par chain, the pre-filter, the gain scaling with truncation / the 0x8000 overflow value)
 *   3 integration (flavour C): V92Transmitter ctor / reset (valid constellation tables, radices, taps in multiples of 4, bounded feedback) / process / dtor over the real sub-objects: blob vs port only (no model),
 *     every output symbol hashed, the transmitter and the sub-objects dumped after each operation
 *
 * What is compared: every returned value, every output buffer (marker-filled beforehand), the bit count written back, the whole object after every operation (field-wise incl. the heap blocks it owns), the
 * canary bytes behind each object, every recorded callee call and debug print (kind, text, a snapshot of the object at the call: store order), the malloc log.  INDEPENDENT MODELS (written from the
 * disassembly facts, not from the port code): the `mq_` functions (queue), the `mt_` functions (transmitter).  A model mismatch exits 3.
 */

#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../harness/harness.h"
#include "v92_cp_layout.h"
#include "v92_tx_coding_layout.h"
#include "pcm_bits_to_symbol_layout.h"

#define L(m) __asm__(m)

/* V90BitsToSymbol */
extern void q90_c1(void *, unsigned, void *) L("_ZN15V90BitsToSymbolC1EjP13V90Parameters");
extern void q90_c2(void *, unsigned, void *) L("_ZN15V90BitsToSymbolC2EjP13V90Parameters");
extern void q90_d1(void *) L("_ZN15V90BitsToSymbolD1Ev");
extern void q90_d2(void *) L("_ZN15V90BitsToSymbolD2Ev");
extern void q90_reset(void *, void *, int) L("_ZN15V90BitsToSymbol5resetEP16V90MappingParams7PcmType");
extern void q90_resetns(void *, void *, int) L("_ZN15V90BitsToSymbol15resetNoSpectralEP16V90MappingParams7PcmType");
extern unsigned q90_nof(void *) L("_ZN15V90BitsToSymbol18nofBitsForNextTimeEv");
extern unsigned q90_setsize(void *, unsigned) L("_ZN15V90BitsToSymbol19setSymbolsBlockSizeEj");
extern unsigned q90_pa(void *, unsigned char *, unsigned *, short *) L("_ZN15V90BitsToSymbol7processEPhRjPs");
extern unsigned q90_pb(void *, unsigned char *, unsigned) L("_ZN15V90BitsToSymbol7processEPhj");
extern unsigned q90_pc(void *, unsigned *, short *) L("_ZN15V90BitsToSymbol7processERjPs");
/* V92BitsToSymbol */
extern void q92_c1(void *, unsigned, void *) L("_ZN15V92BitsToSymbolC1EjP13V92Parameters");
extern void q92_c2(void *, unsigned, void *) L("_ZN15V92BitsToSymbolC2EjP13V92Parameters");
extern void q92_d1(void *) L("_ZN15V92BitsToSymbolD1Ev");
extern void q92_d2(void *) L("_ZN15V92BitsToSymbolD2Ev");
extern void q92_reset(void *, void *) L("_ZN15V92BitsToSymbol5resetEP16V92MappingParams");
extern unsigned q92_nof(void *) L("_ZN15V92BitsToSymbol18nofBitsForNextTimeEv");
extern unsigned q92_setsize(void *, unsigned) L("_ZN15V92BitsToSymbol19setSymbolsBlockSizeEj");
extern unsigned q92_pa(void *, unsigned char *, unsigned *, short *) L("_ZN15V92BitsToSymbol7processEPhRjPs");
extern unsigned q92_pb(void *, unsigned char *, unsigned) L("_ZN15V92BitsToSymbol7processEPhj");
extern unsigned q92_pc(void *, unsigned *, short *) L("_ZN15V92BitsToSymbol7processERjPs");
/* V92Transmitter */
extern void tx_c1(void *) L("_ZN14V92TransmitterC1Ev");
extern void tx_c2(void *) L("_ZN14V92TransmitterC2Ev");
extern void tx_d1(void *) L("_ZN14V92TransmitterD1Ev");
extern void tx_d2(void *) L("_ZN14V92TransmitterD2Ev");
extern void tx_reset(void *, void *) L("_ZN14V92Transmitter5resetEP16V92MappingParams");
extern void tx_process(void *, unsigned char *, unsigned, short *, unsigned *) L("_ZN14V92Transmitter7processEPhjPsRj");
extern unsigned int dsplibs_debug_level;

typedef B2S90 O90;
typedef B2S92 O92;
typedef TxObj OT;
#define SZ90 sizeof(O90)
#define SZ92 sizeof(O92)
#define SZT sizeof(OT)
#define CAN 64

/* ------------------------------------------------------------------------------------------------ small helpers */
/* the heap objects that hold pointers are larger on x86-64 (the port allocates sizeof): the malloc log and the recorders' texts use the i386 sizes */
#ifdef __x86_64__
#define MAPPER_SZ 0x728u
#else
#define MAPPER_SZ 0x704u
#endif
static unsigned norm_size(unsigned sz) {
  if (sz == sizeof(TxObj)) return 0x60;
  if (sz == sizeof(PcObj)) return 0x80;
  if (sz == sizeof(PfObj)) return 0x14;
  if (sz == MAPPER_SZ) return 0x704;
  return sz;
}
static int g_show;
static unsigned long g_nprint, g_ncall;
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)
static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
static void can_fill(uint8_t *c) { memset(c, 0x5a, CAN); }
static void can_check(const uint8_t *c, const char *w) { unsigned i; for (i = 0; i < CAN; i++) if (c[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", w, i); }

/* architecture-independent random content of an object before its constructor runs (the pool's own fill depends on sizeof, which differs on x86-64): the field table drives it */
static void prefill(void *obj, size_t sz, const HFld *f, int nf) {
  int i;
  unsigned k, n;
  uint8_t *o = (uint8_t *)obj;
  memset(o, 0, sz);
  for (i = 0; i < nf; i++) {
    n = f[i].count > 1 ? f[i].count : 1;
    if (f[i].kind == 'x') { for (k = 0; k < f[i].count; k++) o[f[i].off + k] = (uint8_t)h_rnd(); continue; }
    for (k = 0; k < n; k++) {
      uint8_t *q;
      uint32_t v = h_rnd();
      switch (f[i].kind) {
        case 'b': q = o + f[i].off + k; *q = (uint8_t)v; break;
        case 'h': q = o + f[i].off + 2u * k; memcpy(q, &v, 2); break;
        case 'u': case 'i': case 'f': q = o + f[i].off + 4u * k; memcpy(q, &v, 4); break;
        default: { uintptr_t pv = 0xdead0000u | (v & 0xffffu); q = o + f[i].off + sizeof(void *) * k; memcpy(q, &pv, sizeof pv); break; }
      }
    }
  }
}

/* the debug level the code under test sees */
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(60) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
}

/* ------------------------------------------------------------------------------------------------ expected-record queue and recorders */
#define TXT 300
#define EXPMAX 16384
static char (*g_exp)[TXT];
static unsigned g_nexp, g_expi;
static int g_expcheck;           /* 1: every record must be the next one the model expects */
static void exp_clear(void) { g_nexp = 0; g_expi = 0; }
static void exp_push(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void exp_push(const char *fmt, ...) {
  va_list ap;
  if (g_nexp >= EXPMAX) h_die3("expected-record queue overflow");
  va_start(ap, fmt);
  vsnprintf(g_exp[g_nexp++], TXT, fmt, ap);
  va_end(ap);
}
static void exp_done(const char *what) {
  if (g_expi != g_nexp) h_die3("%s: the model expected %u more call / print records (next: %s)", what, g_nexp - g_expi, g_exp[g_expi]);
  exp_clear();
}
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else if (*s == '\t') { out[n++] = '\\'; out[n++] = 't'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
/* the object under test (snapshot at each print / callee call) */
static O90 *g_cur90;
static O92 *g_cur92;
static OT *g_curT;
static void snapshot(void) {
  if (g_cur90) h_dump_obj(0, "snap", g_cur90, f_B2S90, NF_B2S90);
  if (g_cur92) h_dump_obj(0, "snap", g_cur92, f_B2S92, NF_B2S92);
  if (g_curT) h_dump_obj(0, "snap", g_curT, f_TxObj, NF_TxObj);
}
static void rec(char kind, const char *text) {
  char esc[TXT * 2];
  esc[0] = 0;
  esc_cat(esc, sizeof esc, text);
  if (g_expcheck) {
    if (g_expi >= g_nexp) h_die3("unexpected %c-record: %s", kind, text);
    if (strcmp(g_exp[g_expi], text) != 0) h_die3("%c-record mismatch: got \"%s\", the model expected \"%s\"", kind, text, g_exp[g_expi]);
    g_expi++;
  }
  snapshot();
  h_emit(g_show, "%cprintf %s", kind, esc);
  g_nprint++;
}
/* a print can lower the debug level (the code under test must test it afresh at every print site) */
static unsigned g_prints_seen, g_drop_after, g_drop_to;
int dsplibs_debug_printf(const char *fmt, ...) {
  va_list ap;
  char buf[TXT];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  rec('d', buf);
  g_prints_seen++;
  if (g_drop_after && g_prints_seen == g_drop_after) dsplibs_debug_level = g_drop_to;
  return 0;
}
static void callrec(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void callrec(const char *fmt, ...) {
  va_list ap;
  char buf[TXT];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  rec('c', buf);
  g_ncall++;
}
static unsigned long long fnv(const void *p, size_t n) {
  const uint8_t *b = (const uint8_t *)p;
  unsigned long long h = 0xcbf29ce484222325ULL;
  size_t i;
  for (i = 0; i < n; i++) { h ^= b[i]; h *= 0x100000001b3ULL; }
  return h;
}

/* ------------------------------------------------------------------------------------------------ coverage ids */
enum {
  C_TRIALS = 1,
  /* queue classes */
  Q_C1, Q_C2, Q_D1, Q_D2, Q_D_NULLMAP, Q_D_NULLBUF, Q_RESET, Q_RESETNS, Q_K0, Q_KPOS, Q_SETSIZE, Q_SET_ZERO, Q_SET_LT, Q_SET_GE, Q_SET_HUGE, Q_NOF, Q_NOF_ZERO, Q_NOF_EXACT, Q_NOF_ROUND, Q_NOF_FIRST,
  Q_NOF_NOFIRST, Q_NOF_WRAP, Q_PA, Q_PA_NOTSET, Q_PA_OK, Q_PA_OVER, Q_PA_UNDER, Q_PA_SHIFT, Q_PA_EXACTFILL, Q_PA_NEXT_ZERO, Q_PA_NEXT_NZ, Q_PB, Q_PB_NOTSET, Q_PB_OK, Q_PB_OVER, Q_PC, Q_PC_NOTSET, Q_PC_OK,
  Q_PC_UNDER, Q_PC_SHIFT, Q_PC_NEXT_ZERO, Q_PC_NEXT_NZ, Q_FIRST_CLEARED, Q_FIRST_POKED, Q_PRINT_ON, Q_PRINT_OFF, Q_DROP, Q_MAP_ZERO, Q_MAP_CNT, Q_BLK_GT_CAP_SKIP, Q_FILL_CARRY, Q_V92_DIV12, Q_HUGE_PROC,
  /* V92Transmitter */
  T_C1, T_C2, T_D1, T_D2, T_D_NULL_BUF, T_D_NULL_ME, T_D_NULL_FLAG, T_D_NULL_PRE, T_D_NULL_PF, T_D_NULL_CE, T_RESET, T_RESET_DUMP, T_RESET_QUIET, T_RESET_DROP, T_CNT0_0, T_CNT0_NZ, T_CNT1_0, T_CNT1_NZ,
  T_CNT2_0, T_CNT2_NZ, T_CNT3_0, T_CNT3_NZ, T_SIGN_PLUS, T_SIGN_MINUS, T_SIGN_NAN, T_SIGN_ZERO, T_SPLIT_HUGE, T_SPLIT_FRAC, T_SPLIT_NEG, T_PROC, T_PROC_NODATA, T_PROC_PARTIAL, T_PROC_GROUP, T_PROC_MULTI,
  T_PROC_CARRY, T_PROC_W0_ONE, T_PROC_OVF_POS, T_PROC_OVF_NEG, T_PROC_EDGE_POS, T_PROC_EDGE_NEG, T_PROC_NAN, T_PROC_INF, T_PROC_TRUNC_NEG, T_PROC_TRUNC_POS, T_PROC_GAIN_NAN, T_PROC_PAR, T_PROC_BYTE,
  T_GAIN_SNAN, T_RESET_PAR_FILL,
  /* integration (real sub-objects) */
  I_TRIALS, I_C1, I_C2, I_D1, I_D2, I_RESET, I_PRINT, I_TR0, I_TR1, I_TR2, I_PROC, I_GROUPS, I_PARTIAL, I_FIR, I_NOFIR, I_PF, I_NOPF, I_OUTNZ,
  COV_MAX
};
static unsigned long g_covn[COV_MAX];
static unsigned char g_covreq[COV_MAX];
static void cov(int id) { g_covn[id]++; }
static const char *cov_name(int id) {
  static char buf[COV_MAX][40];
  if (!buf[id][0]) snprintf(buf[id], sizeof buf[id], "cov %03d", id);
  return buf[id];
}

/* ================================================================================================ the BitsToSymbol classes: recorders (flavour A and B) */
/* the mapper recorders (both flavours: the port object of flavour B refers to them) */
static unsigned g_map_cnt;                 /* symbols the next V90Mapper / V92Transmitter ::process call delivers */
static unsigned g_map_seed;
static void *g_map_owner;                  /* the heap block `this` must be (the Mapper / the Transmitter of the queue under test) */
static const char *g_map_name = "mapper";
static unsigned char *g_in_ptr;            /* the input bits the driver handed over */
static void *g_q_buf;                      /* buf_ of the queue under test */
static unsigned g_q_bufsz;                 /* its size in shorts */
static void *g_q_params;
static void *g_cur_mp;                     /* the mapping-params block the driver passes */
static void chk_this(const void *t, const char *what) {
  if (t != g_map_owner) h_die3("%s: `this` is not the %s block of the queue under test", what, g_map_name);
}
static void h_map_ctor(void *t, void *p) {
  HAl *a = h_find_al(t, 0);
  if (!a) h_die3("V90Mapper ctor on memory that is not a heap block");
  g_map_owner = t;
  callrec("mapper ctor size=0x%x params=%s", norm_size(a->size), p == g_q_params ? "PB" : "OTHER");
}
void h_m_c1(void *t, void *p) L("_ZN9V90MapperC1EP13V90Parameters");
void h_m_c2(void *t, void *p) L("_ZN9V90MapperC2EP13V90Parameters");
void h_m_c1(void *t, void *p) { h_map_ctor(t, p); }
void h_m_c2(void *t, void *p) { h_map_ctor(t, p); }
void h_m_d1(void *t) L("_ZN9V90MapperD1Ev");
void h_m_d2(void *t) L("_ZN9V90MapperD2Ev");
void h_m_d1(void *t) { chk_this(t, "mapper dtor"); callrec("mapper dtor"); }
void h_m_d2(void *t) { chk_this(t, "mapper dtor"); callrec("mapper dtor"); }
void h_m_reset(void *t, void *mp, int pcm) L("_ZN9V90Mapper5resetEP16V90MappingParams7PcmType");
void h_m_reset(void *t, void *mp, int pcm) { chk_this(t, "mapper reset"); callrec("mapper reset mp=%s pcm=%d", mp == g_cur_mp ? "MP" : "OTHER", pcm); }
void h_m_resetns(void *t, void *mp, int pcm) L("_ZN9V90Mapper15resetNoSpectralEP16V90MappingParams7PcmType");
void h_m_resetns(void *t, void *mp, int pcm) { chk_this(t, "mapper resetNoSpectral"); callrec("mapper resetNoSpectral mp=%s pcm=%d", mp == g_cur_mp ? "MP" : "OTHER", pcm); }
/* the symbols a recorder writes: a pure function of the call's position; clamped to the queue block (the real mapper never writes outside it either) */
static void map_write(short *out, unsigned *outCnt, const char *who) {
  unsigned i, room, cnt = g_map_cnt;
  long off = (long)(out - (short *)g_q_buf);
  (void)who;
  room = off >= 0 && (unsigned long)off <= g_q_bufsz ? g_q_bufsz - (unsigned)off : 0;
  for (i = 0; i < cnt && i < room; i++) out[i] = (short)(g_map_seed * 31u + i * 7u + 3u);
  *outCnt = cnt;
}
void h_m_process(void *t, unsigned char *in, unsigned n, short *out, unsigned *outCnt) L("_ZN9V90Mapper7processEPhjPsRj");
void h_m_process(void *t, unsigned char *in, unsigned n, short *out, unsigned *outCnt) {
  chk_this(t, "mapper process");
  callrec("mapper process in=%s n=%u out=buf+%ld cnt=%u", in == g_in_ptr ? "IN" : "OTHER", n, (long)(out - (short *)g_q_buf), g_map_cnt);
  map_write(out, outCnt, "mapper");
}

#ifdef P29_BIN_A
/* V92Transmitter recorders (the V92BitsToSymbol scenario) */
void h_t_c1(void *t) L("_ZN14V92TransmitterC1Ev");
void h_t_c2(void *t) L("_ZN14V92TransmitterC2Ev");
static void h_t_ctor(void *t) {
  HAl *a = h_find_al(t, 0);
  if (!a) h_die3("V92Transmitter ctor on memory that is not a heap block");
  g_map_owner = t;
  callrec("tx ctor size=0x%x", norm_size(a->size));
}
void h_t_c1(void *t) { h_t_ctor(t); }
void h_t_c2(void *t) { h_t_ctor(t); }
void h_t_d1(void *t) L("_ZN14V92TransmitterD1Ev");
void h_t_d2(void *t) L("_ZN14V92TransmitterD2Ev");
void h_t_d1(void *t) { chk_this(t, "tx dtor"); callrec("tx dtor"); }
void h_t_d2(void *t) { chk_this(t, "tx dtor"); callrec("tx dtor"); }
void h_t_reset(void *t, void *mp) L("_ZN14V92Transmitter5resetEP16V92MappingParams");
void h_t_reset(void *t, void *mp) { chk_this(t, "tx reset"); callrec("tx reset mp=%s", mp == g_cur_mp ? "MP" : "OTHER"); }
void h_t_process(void *t, unsigned char *in, unsigned n, short *out, unsigned *outCnt) L("_ZN14V92Transmitter7processEPhjPsRj");
void h_t_process(void *t, unsigned char *in, unsigned n, short *out, unsigned *outCnt) {
  chk_this(t, "tx process");
  callrec("tx process in=%s n=%u out=buf+%ld cnt=%u", in == g_in_ptr ? "IN" : "OTHER", n, (long)(out - (short *)g_q_buf), g_map_cnt);
  map_write(out, outCnt, "tx");
}
#endif

/* ================================================================================================ the queue model (independent of the port) */
typedef struct {
  unsigned cap, fill, w0, extra, blk;
  uint8_t first;
  int ver;                       /* 90 / 92 */
  short buf[4096];
} MQ;
static MQ g_mq;
static unsigned mq_div(const MQ *m) { return m->ver == 90 ? 6u : 12u; }
/* the number of data bits the next call must bring for d symbols still to queue (32-bit wrapping arithmetic, exact multiples through (w0 * d) / div) */
static unsigned mq_bits(const MQ *m, unsigned d) {
  unsigned div = mq_div(m), rem = d % div, q = d / div;
  if (rem == 0) return (unsigned)((uint32_t)(m->w0 * d) / div);
  return (unsigned)((uint32_t)(q + 1u) * m->w0);
}
static unsigned mq_need(const MQ *m, unsigned blk, unsigned fill) {
  unsigned d;
  if (blk <= fill) return 0;
  d = blk - fill;
  if (m->ver == 90 && m->first) d += m->extra;
  return mq_bits(m, d);
}
static void mq_clear_first(MQ *m) { m->first = 0; }
/* the three process() overloads; kind 0 = (in, n&, out), 1 = (in, n), 2 = (n&, out).  The mapper's delivery (cnt symbols of the recorder's pattern) is applied by the caller.  Returns the code. */
static unsigned mq_process(MQ *m, int kind, unsigned nsyms, const short *symsrc, short *outbuf, unsigned *nio, int *did_over)
{
  unsigned ret = 0, i, ncopy;
  *did_over = 0;
  if (m->blk == 0) { mq_clear_first(m); return 1; }
  if (kind != 2) {
    unsigned room = m->cap - m->fill;
    for (i = 0; i < nsyms && i < room; i++) m->buf[m->fill + i] = symsrc[i];
    m->fill += nsyms;
  }
  if (kind == 1) {
    if (m->fill > m->cap) { ret = 2; *did_over = 1; m->fill = m->blk; }
    mq_clear_first(m);
    return ret;
  }
  if (kind == 0 && m->fill >= m->blk) {
    if (m->fill > m->cap) { ret = 2; *did_over = 1; m->fill = m->blk; }
    ncopy = m->blk;
  } else if (kind == 2 && m->fill >= m->blk) {
    ncopy = m->blk;
  } else {
    ret = 3;
    ncopy = m->fill;
  }
  for (i = 0; i < ncopy; i++) outbuf[i] = m->buf[i];
  if (ret == 3) m->fill = 0;
  if (m->fill > m->blk) {
    unsigned rest = m->fill - m->blk;
    memmove(m->buf, m->buf + m->blk, rest * sizeof(short));
    m->fill = rest;
  } else {
    m->fill = 0;
  }
  *nio = mq_need(m, m->blk, m->fill);
  mq_clear_first(m);
  return ret;
}

typedef struct { void *mapper, *params, *buf; unsigned cap, fill, w0, extra, blk; uint8_t first; } QView;
static QView qview(int ver, void *o) {
  QView v;
  memset(&v, 0, sizeof v);
  if (ver == 90) { O90 *p = (O90 *)o; v.mapper = p->mapper_; v.params = p->params_; v.buf = p->buf_; v.cap = p->cap_; v.fill = p->fill_; v.w0 = p->w0_; v.extra = p->extra_; v.blk = p->blk_; v.first = p->first_; }
  else { O92 *p = (O92 *)o; v.mapper = p->tx_; v.params = p->params_; v.buf = p->buf_; v.cap = p->cap_; v.fill = p->fill_; v.w0 = p->w0_; v.blk = p->blk_; v.first = p->first_; }
  return v;
}
static void qcheck(int ver, void *o, const char *what, int w0_defined, int extra_defined) {
  QView v = qview(ver, o);
  char b[100];
  snprintf(b, sizeof b, "%s: fill", what); EXPECT_EQ(b, v.fill, g_mq.fill);
  snprintf(b, sizeof b, "%s: blk", what); EXPECT_EQ(b, v.blk, g_mq.blk);
  snprintf(b, sizeof b, "%s: cap", what); EXPECT_EQ(b, v.cap, g_mq.cap);
  snprintf(b, sizeof b, "%s: first", what); EXPECT_EQ(b, v.first, g_mq.first);
  if (w0_defined) { snprintf(b, sizeof b, "%s: w0", what); EXPECT_EQ(b, v.w0, g_mq.w0); }
  if (ver == 90 && extra_defined) { snprintf(b, sizeof b, "%s: extra", what); EXPECT_EQ(b, v.extra, g_mq.extra); }
  if (memcmp(v.buf, g_mq.buf, (size_t)g_mq.cap * sizeof(short)) != 0) h_die3("%s: the queue block differs from the model", what);
}

/* ================================================================================================ queue trials (scenarios 0, 1) */
static unsigned pick_cap(void) {
  static const unsigned c[] = { 1, 2, 3, 5, 6, 7, 12, 13, 24, 25, 48, 100, 6, 12 };
  return c[h_rmod(sizeof c / sizeof c[0])];
}
static void set_mapcnt(unsigned cap, unsigned fill) {
  unsigned r = h_rmod(10);
  g_map_seed = h_rnd();
  if (r < 2) g_map_cnt = 0;
  else if (r < 6) g_map_cnt = h_rmod(7);
  else if (r < 8) g_map_cnt = 6 * (1 + h_rmod(3));
  else g_map_cnt = cap > fill ? cap - fill + h_rmod(3) : h_rmod(4);   /* sometimes beyond the queue: the overflow path */
  if (g_map_cnt > 2000) g_map_cnt = 2000;
  if (g_map_cnt) cov(Q_MAP_CNT); else cov(Q_MAP_ZERO);
}
static void trial_queue(int ver) {
  int c2 = h_rpct(50), ops, k;
  size_t SZ = ver == 90 ? SZ90 : SZ92;
  const HFld *flds = ver == 90 ? f_B2S90 : f_B2S92;
  int nf = ver == 90 ? NF_B2S90 : NF_B2S92;
  uint8_t *o = (uint8_t *)h_obj_new(SZ + CAN);
  uint8_t *cans = o + SZ;
  unsigned cap = pick_cap();
  void *params;
  MapPar90 *mp90 = NULL;
  MapPar92 *mp92 = NULL;
  uint8_t inbuf[64];
  short outbuf[256];
  unsigned i;
  can_fill(cans);
  prefill(o, SZ, flds, nf);
  g_cur90 = ver == 90 ? (O90 *)o : NULL;
  g_cur92 = ver == 92 ? (O92 *)o : NULL;
  g_curT = NULL;
  params = sysdep_malloc(16);
  g_q_params = params;
  g_in_ptr = inbuf;
  dsplibs_debug_level = pick_level();
  exp_clear(); g_expcheck = 1;
  g_drop_after = 0;
  mp90 = (MapPar90 *)h_obj_new(sizeof(MapPar90));
  mp92 = (MapPar92 *)h_obj_new(sizeof(MapPar92));
  memset(mp90, 0, sizeof *mp90);
  memset(mp92, 0, sizeof *mp92);
  g_cur_mp = ver == 90 ? (void *)mp90 : (void *)mp92;
  /* the constructor: the mapper (or transmitter) block first, its constructor, then the queue block */
  cov(c2 ? Q_C2 : Q_C1);
  g_map_owner = NULL;
  g_map_name = ver == 90 ? "mapper" : "transmitter";
  if (ver == 90) exp_push("mapper ctor size=0x704 params=PB");
  else exp_push("tx ctor size=0x60");
  if (ver == 90) { H_CTOR_PROBE(o, flds, nf, sizeof(O90), (c2 ? q90_c2(o, cap, params) : q90_c1(o, cap, params))); }
  else { H_CTOR_PROBE(o, flds, nf, sizeof(O92), (c2 ? q92_c2(o, cap, params) : q92_c1(o, cap, params))); }
  exp_done("queue ctor");
  g_expcheck = 1;
  {
    QView v = qview(ver, o);
    HAl *am = h_find_al(v.mapper, 0), *ab = h_find_al(v.buf, 0);
    if (!am || am->p != (uint8_t *)v.mapper || norm_size(am->size) != (ver == 90 ? 0x704u : 0x60u)) h_die3("the constructor's first block is not the %s block (size 0x%x)", ver == 90 ? "Mapper" : "Transmitter", ver == 90 ? 0x704 : 0x60);
    if (!ab || ab->p != (uint8_t *)v.buf || ab->size != 2 * cap) h_die3("the queue block has the wrong size");
    if (am->p != (uint8_t *)g_map_owner) h_die3("the mapper's constructor was not called on the first block");
    EXPECT_EQ("params_", (uintptr_t)v.params, (uintptr_t)params);
    memset(&g_mq, 0, sizeof g_mq);
    g_mq.ver = ver; g_mq.cap = cap; g_mq.fill = 0; g_mq.blk = 0; g_mq.first = 1;
    memcpy(g_mq.buf, v.buf, (size_t)cap * sizeof(short));
    g_q_buf = v.buf; g_q_bufsz = cap;
    g_map_owner = v.mapper;
    qcheck(ver, o, "after the constructor", 0, 0);
    /* w0_ / extra_ keep the prefilled garbage: adopt it into the model */
    { QView w = qview(ver, o); g_mq.w0 = w.w0; g_mq.extra = w.extra; }
    h_dump_obj(0, "ctor", o, flds, nf);
  }
  can_check(cans, "constructor");
  ops = 30 + (int)h_rmod(60);
  for (k = 0; k < ops; k++) {
    unsigned which = k == 0 ? 0 : h_rmod(100);
    if (which < 12) {
      /* reset / resetNoSpectral */
      int ns = ver == 90 && h_rpct(35);
      unsigned w0 = h_rpct(20) ? h_rnd() : (unsigned)(28 + h_rmod(60));
      int pcm = (int)h_rmod(3);
      if (ver == 90) {
        static const unsigned ks[] = { 0, 1, 2, 3, 6, 4, 5, 0, 0xffffffffu, 7 };
        unsigned K = ks[h_rmod(sizeof ks / sizeof ks[0])];
        unsigned dep = h_rpct(20) ? h_rnd() : h_rmod(12);
        mp90->w0_ = w0; mp90->w620_ = K; mp90->w624_ = dep;
        if (ns) { exp_push("mapper resetNoSpectral mp=MP pcm=%d", pcm); q90_resetns(o, mp90, pcm); g_mq.w0 = w0; cov(Q_RESETNS); }
        else {
          exp_push("mapper reset mp=MP pcm=%d", pcm); q90_reset(o, mp90, pcm);
          g_mq.w0 = w0; g_mq.extra = K ? (unsigned)((uint32_t)(dep * 6u) / K) : 0u; g_mq.fill = 0; g_mq.blk = 0; g_mq.first = 1;
          cov(Q_RESET); cov(K ? Q_KPOS : Q_K0);
        }
      } else {
        mp92->w0_ = (int32_t)w0;
        exp_push("tx reset mp=MP"); q92_reset(o, mp92);
        g_mq.w0 = w0; g_mq.fill = 0; g_mq.blk = 0; g_mq.first = 1;
        cov(Q_RESET);
      }
      exp_done("reset");
      qcheck(ver, o, "reset", 1, 1);
      h_dump_obj(0, "reset", o, flds, nf);
    } else if (which < 30) {
      /* setSymbolsBlockSize / nofBitsForNextTime */
      unsigned r = h_rmod(10), blk, got, want;
      if (r < 6) blk = h_rmod(cap + 1);
      else if (r < 8) blk = cap;
      else if (r < 9) blk = g_mq.fill + h_rmod(3);
      else blk = h_rpct(50) ? 0x80000000u + h_rmod(5) : h_rnd();
      if (h_rpct(25)) g_mq.w0 = h_rpct(50) ? h_rnd() : h_rmod(100);     /* the formula reads w0_, extra_ whatever they are: poke them (V90Parameters-free: plain members) */
      if (ver == 90 && h_rpct(20)) g_mq.extra = h_rnd();
      if (h_rpct(20)) {
        static const uint8_t fv[] = { 1, 2, 0x80, 0xff, 3 };
        g_mq.first = h_rpct(35) ? 1 : h_rpct(70) ? fv[h_rmod(sizeof fv)] : (uint8_t)(1 + h_rmod(255));
      }
      if (h_rpct(10)) g_mq.fill = h_rmod(cap + 1);
      /* the poke goes into the object as well */
      if (ver == 90) { O90 *q = (O90 *)o; q->w0_ = g_mq.w0; q->extra_ = g_mq.extra; q->first_ = g_mq.first; q->fill_ = g_mq.fill; }
      else { O92 *q = (O92 *)o; q->w0_ = g_mq.w0; q->first_ = g_mq.first; q->fill_ = g_mq.fill; }
      if (g_mq.first > 1) cov(Q_FIRST_POKED);
      if (which < 22) {
        cov(Q_SETSIZE);
        g_mq.blk = blk;
        want = mq_need(&g_mq, blk, g_mq.fill);
        got = ver == 90 ? q90_setsize(o, blk) : q92_setsize(o, blk);
        cov(blk == 0 ? Q_SET_ZERO : blk < g_mq.fill ? Q_SET_LT : Q_SET_GE);
        if (blk >= 0x80000000u) cov(Q_SET_HUGE);
      } else {
        cov(Q_NOF);
        want = mq_need(&g_mq, g_mq.blk, g_mq.fill);
        got = ver == 90 ? q90_nof(o) : q92_nof(o);
      }
      EXPECT_EQ("bit count", got, want);
      if (want == 0) cov(Q_NOF_ZERO);
      else if (g_mq.blk > g_mq.fill) {
        unsigned d = g_mq.blk - g_mq.fill;
        if (ver == 90 && g_mq.first) { d += g_mq.extra; cov(Q_NOF_FIRST); } else if (ver == 90) cov(Q_NOF_NOFIRST);
        cov(d % mq_div(&g_mq) == 0 ? Q_NOF_EXACT : Q_NOF_ROUND);
        if ((uint64_t)g_mq.w0 * d > 0xffffffffull) cov(Q_NOF_WRAP);
      }
      if (ver == 92) cov(Q_V92_DIV12);
      qcheck(ver, o, "setSymbolsBlockSize / nofBitsForNextTime", 1, 1);
      h_dump_obj(0, "size", o, flds, nf);
    } else {
      /* process: only with blk <= cap (a block larger than the queue would read past it; the blob does the same) */
      int kind = (int)h_rmod(3), over, huge;
      unsigned nio, ret, want, nbits;
      short ref[256];
      short symtmp[2100];
      unsigned i2;
      huge = g_mq.blk >= 0x80000000u;
      if (g_mq.blk > cap && !(huge && kind != 1)) { cov(Q_BLK_GT_CAP_SKIP); continue; }   /* a huge block size is safe on the paths that only underflow (no copy of blk entries) */
      if (huge) cov(Q_HUGE_PROC);
      for (i2 = 0; i2 < 64; i2++) inbuf[i2] = (uint8_t)h_rnd();
      for (i2 = 0; i2 < 256; i2++) outbuf[i2] = (short)0xa5a5;
      memcpy(ref, outbuf, sizeof ref);
      nbits = h_rpct(20) ? h_rnd() : h_rmod(200);
      nio = nbits;
      set_mapcnt(cap, g_mq.fill);
      if (huge && g_map_cnt > cap - g_mq.fill) g_map_cnt = cap - g_mq.fill;
      for (i2 = 0; i2 < g_map_cnt && i2 < 2100; i2++) symtmp[i2] = (short)(g_map_seed * 31u + i2 * 7u + 3u);
      if (kind != 2 && g_mq.blk != 0) exp_push("%s process in=IN n=%u out=buf+%u cnt=%u", ver == 90 ? "mapper" : "tx", nbits, g_mq.fill, g_map_cnt);
      /* debug prints: the model expects the text of the code the model returns */
      want = mq_process(&g_mq, kind, g_map_cnt, symtmp, ref, &nio, &over);
      if (want == 1) { cov(Q_PRINT_ON); }
      {
        const char *nm = ver == 90 ? "V90BitsToSymbol" : "V92BitsToSymbol";
        const char *what = want == 1 ? "SIZE_NOT_SET" : want == 2 ? "BUFFER_OVERFLOW" : want == 3 ? "BUFFER_UNDERFLOW" : NULL;
        if (what) {
          if (dsplibs_debug_level > 1) { exp_push("%s - error: process called, %s\r\n", nm, what); cov(Q_PRINT_ON); } else cov(Q_PRINT_OFF);
        }
      }
      if (kind == 0) { cov(Q_PA); ret = ver == 90 ? q90_pa(o, inbuf, &nbits, outbuf) : q92_pa(o, inbuf, &nbits, outbuf); }
      else if (kind == 1) { cov(Q_PB); ret = ver == 90 ? q90_pb(o, inbuf, nbits) : q92_pb(o, inbuf, nbits); nio = nbits; }
      else { cov(Q_PC); ret = ver == 90 ? q90_pc(o, &nbits, outbuf) : q92_pc(o, &nbits, outbuf); }
      exp_done("process");
      EXPECT_EQ("process return code", ret, want);
      if (kind != 1) {
        if (want != 1) EXPECT_EQ("bits for the next call", nbits, nio);
        else EXPECT_EQ("bits (not set: untouched)", nbits, nio);
      }
      if (memcmp(outbuf, ref, sizeof ref) != 0) h_die3("process: the output symbols differ from the model");
      if (kind == 0) cov(want == 1 ? Q_PA_NOTSET : want == 0 ? Q_PA_OK : want == 2 ? Q_PA_OVER : Q_PA_UNDER);
      if (kind == 1) cov(want == 1 ? Q_PB_NOTSET : want == 0 ? Q_PB_OK : Q_PB_OVER);
      if (kind == 2) cov(want == 1 ? Q_PC_NOTSET : want == 0 ? Q_PC_OK : Q_PC_UNDER);
      if (kind != 1 && want != 1) {
        cov(nio == 0 ? (kind == 0 ? Q_PA_NEXT_ZERO : Q_PC_NEXT_ZERO) : (kind == 0 ? Q_PA_NEXT_NZ : Q_PC_NEXT_NZ));
        if (g_mq.fill) { cov(kind == 0 ? Q_PA_SHIFT : Q_PC_SHIFT); cov(Q_FILL_CARRY); }
      }
      cov(Q_FIRST_CLEARED);
      qcheck(ver, o, "process", 1, 1);
      h_dump_obj(0, "process", o, flds, nf);
    }
    can_check(cans, "queue object");
  }
  (void)i;
  /* destruction: the mapper and the queue block are freed; NULL members are skipped */
  {
    int d2 = h_rpct(50), nullmap = h_rpct(15), nullbuf = h_rpct(15);
    QView v = qview(ver, o);
    if (nullmap) { sysdep_free(v.mapper); if (ver == 90) ((O90 *)o)->mapper_ = NULL; else ((O92 *)o)->tx_ = NULL; cov(Q_D_NULLMAP); }
    if (nullbuf) { sysdep_free(v.buf); if (ver == 90) ((O90 *)o)->buf_ = NULL; else ((O92 *)o)->buf_ = NULL; cov(Q_D_NULLBUF); }
    cov(d2 ? Q_D2 : Q_D1);
    if (!nullmap) exp_push(ver == 90 ? "mapper dtor" : "tx dtor");
    if (ver == 90) { if (d2) q90_d2(o); else q90_d1(o); } else { if (d2) q92_d2(o); else q92_d1(o); }
    __asm__ volatile("" ::: "memory");
    exp_done("queue dtor");
    can_check(cans, "queue dtor");
    h_dump_obj(0, "dtor", o, flds, nf);
    sysdep_free(params);
  }
  g_cur90 = NULL; g_cur92 = NULL;
  h_trial_end("queue");
  cov(C_TRIALS);
}

#ifndef P29_BIN_C
/* ================================================================================================ V92Transmitter: sub-object recorders (both flavours: the port object refers to them) */
static MapPar92 *g_mp92;
static unsigned g_pc_script[12];            /* the float WORDS V92Precoder::process writes (row * 4 + i) */
static unsigned g_pf_script[12];            /* the float WORDS V92PreFilter::process writes */
static float *g_fb_base;                    /* out pointer of the first precoder call of a group */
static int *g_state_ptr;
static const char *own(const void *p, const void *q) { return p == q ? "OWN" : "OTHER"; }
static const char *arrname(const float *p) {
  int k;
  static const char *n[4] = { "arr0", "arr1", "arr2", "arr3" };
  if (!g_mp92) return "?";
  for (k = 0; k < 4; k++) if ((const void *)p == g_mp92->arr_[k]) return n[k];
  return "OTHER";
}
static void chk_blk(const void *t, const void *want, const char *what) { if (t != want) h_die3("%s: `this` is not the sub-object block of the transmitter", what); }
static unsigned blk_size(const void *t) { HAl *a = h_find_al(t, 0); if (!a || a->p != (const uint8_t *)t) h_die3("sub-object constructor on memory that is not a heap block"); return norm_size(a->size); }
/* world functions of the recorders (the model uses the same) */
static unsigned long long me_hash(const uint8_t *buf) { return fnv(buf, 0x50); }
static unsigned me_digit(unsigned long long h, unsigned k) { return (unsigned)(h >> (k % 5u * 8u)) * 2654435761u + k * 40503u; }
static int pc_state(const unsigned *u, int row, int par, int i) { return (int)(u[(row * 4 + i) % 12] + (unsigned)par * 3u + (unsigned)i * 17u + (unsigned)row * 1000u); }
static int ce_ret(const int *a) { return (int)((unsigned)a[0] * 3u + (unsigned)a[1] * 5u + (unsigned)a[2] * 7u + (unsigned)a[3] * 11u + ((unsigned)a[1] << 3)); }

/* V92ModulusEncoder */
void h_me_c1(void *t) L("_ZN17V92ModulusEncoderC1Ev");
void h_me_c2(void *t) L("_ZN17V92ModulusEncoderC2Ev");
void h_me_c1(void *t) { callrec("me ctor size=0x%x", blk_size(t)); }
void h_me_c2(void *t) { callrec("me ctor size=0x%x", blk_size(t)); }
void h_me_reset(void *t, void *mp) L("_ZN17V92ModulusEncoder5resetEP16V92MappingParams");
void h_me_reset(void *t, void *mp) { chk_blk(t, g_curT->me_, "me reset"); callrec("me reset mp=%s", mp == g_cur_mp ? "MP" : "OTHER"); }
void h_me_progress(void *t, unsigned char *bits, unsigned *digits) L("_ZN17V92ModulusEncoder8progressEPhPj");
void h_me_progress(void *t, unsigned char *bits, unsigned *digits) {
  unsigned long long h = me_hash(bits);
  unsigned k;
  chk_blk(t, g_curT->me_, "me progress");
  callrec("me progress buf=%s digits=%s h=%016llx", own(bits, g_curT->buf_), own(digits, g_curT->digits_), h);
  for (k = 0; k < 12; k++) digits[k] = me_digit(h, k);
}
/* V92Precoder */
void h_pc_c1(void *t, unsigned n) L("_ZN11V92PrecoderC1Ej");
void h_pc_c2(void *t, unsigned n) L("_ZN11V92PrecoderC2Ej");
void h_pc_c1(void *t, unsigned n) { callrec("pre ctor size=0x%x ntaps=0x%x", blk_size(t), n); }
void h_pc_c2(void *t, unsigned n) { callrec("pre ctor size=0x%x ntaps=0x%x", blk_size(t), n); }
void h_pc_d1(void *t) L("_ZN11V92PrecoderD1Ev");
void h_pc_d2(void *t) L("_ZN11V92PrecoderD2Ev");
void h_pc_d1(void *t) { chk_blk(t, g_curT->pre_, "pre dtor"); callrec("pre dtor"); }
void h_pc_d2(void *t) { chk_blk(t, g_curT->pre_, "pre dtor"); callrec("pre dtor"); }
void h_pc_reset(void *t, void *mp) L("_ZN11V92Precoder5resetEP16V92MappingParams");
void h_pc_reset(void *t, void *mp) { chk_blk(t, g_curT->pre_, "pre reset"); callrec("pre reset mp=%s", mp == g_cur_mp ? "MP" : "OTHER"); }
void h_pc_setc(void *t, float *c1, float *c2, unsigned n1, unsigned n2) L("_ZN11V92Precoder15setCoefficientsEPfS0_jj");
void h_pc_setc(void *t, float *c1, float *c2, unsigned n1, unsigned n2) {
  chk_blk(t, g_curT->pre_, "pre setCoefficients");
  callrec("pre setCoef c1=%s c2=%s n1=%u n2=%u", arrname(c1), arrname(c2), n1, n2);
}
void h_pc_process(void *t, unsigned *u, int row, int par, int *state, float *out) L("_ZN11V92Precoder7processEPjiiPiPf");
void h_pc_process(void *t, unsigned *u, int row, int par, int *state, float *out) {
  int i;
  char st[16];
  chk_blk(t, g_curT->pre_, "pre process");
  if (row == 0) { g_state_ptr = state; g_fb_base = out; snprintf(st, sizeof st, "new"); }
  else snprintf(st, sizeof st, "%s", state == g_state_ptr ? "same" : "diff");
  callrec("pre process u=%s row=%d par=%d state=%s out=+%ld", own(u, g_curT->digits_), row, par, st, (long)(out - g_fb_base));
  for (i = 0; i < 4; i++) {
    state[i] = pc_state(u, row, par, i);
    memcpy(&out[i], &g_pc_script[(unsigned)(row * 4 + i) % 12u], 4);
  }
}
/* V92PreFilter */
void h_pf_c1(void *t, unsigned n) L("_ZN12V92PreFilterC1Ej");
void h_pf_c2(void *t, unsigned n) L("_ZN12V92PreFilterC2Ej");
void h_pf_c1(void *t, unsigned n) { callrec("pf ctor size=0x%x ntaps=0x%x", blk_size(t), n); }
void h_pf_c2(void *t, unsigned n) { callrec("pf ctor size=0x%x ntaps=0x%x", blk_size(t), n); }
void h_pf_d1(void *t) L("_ZN12V92PreFilterD1Ev");
void h_pf_d2(void *t) L("_ZN12V92PreFilterD2Ev");
void h_pf_d1(void *t) { chk_blk(t, g_curT->pf_, "pf dtor"); callrec("pf dtor"); }
void h_pf_d2(void *t) { chk_blk(t, g_curT->pf_, "pf dtor"); callrec("pf dtor"); }
void h_pf_reset(void *t) L("_ZN12V92PreFilter5resetEv");
void h_pf_reset(void *t) { chk_blk(t, g_curT->pf_, "pf reset"); callrec("pf reset"); }
void h_pf_setc(void *t, float *c1, float *c2, unsigned n1, unsigned n2) L("_ZN12V92PreFilter15setCoefficientsEPfS0_jj");
void h_pf_setc(void *t, float *c1, float *c2, unsigned n1, unsigned n2) {
  chk_blk(t, g_curT->pf_, "pf setCoefficients");
  callrec("pf setCoef c1=%s c2=%s n1=%u n2=%u", arrname(c1), arrname(c2), n1, n2);
}
void h_pf_process(void *t, float *in, float *out) L("_ZN12V92PreFilter7processEPfS0_");
void h_pf_process(void *t, float *in, float *out) {
  char w[200];
  unsigned k, n = 0, x;
  chk_blk(t, g_curT->pf_, "pf process");
  w[0] = 0;
  for (k = 0; k < 12; k++) { memcpy(&x, &in[k], 4); n += (unsigned)snprintf(w + n, sizeof w - n, "%s%08x", k ? "," : "", x); }
  callrec("pf process in=%s out=%s words=%s", in == g_fb_base ? "FB" : "OTHER", out == in ? "IN" : "NEW", w);
  for (k = 0; k < 12; k++) memcpy(&out[k], &g_pf_script[k], 4);
}
/* V92ConvolutionEncoder */
void h_ce_c1(void *t) L("_ZN21V92ConvolutionEncoderC1Ev");
void h_ce_c2(void *t) L("_ZN21V92ConvolutionEncoderC2Ev");
void h_ce_c1(void *t) { callrec("ce ctor size=0x%x", blk_size(t)); }
void h_ce_c2(void *t) { callrec("ce ctor size=0x%x", blk_size(t)); }
void h_ce_d1(void *t) L("_ZN21V92ConvolutionEncoderD1Ev");
void h_ce_d2(void *t) L("_ZN21V92ConvolutionEncoderD2Ev");
void h_ce_d1(void *t) { chk_blk(t, g_curT->ce_, "ce dtor"); callrec("ce dtor"); }
void h_ce_d2(void *t) { chk_blk(t, g_curT->ce_, "ce dtor"); callrec("ce dtor"); }
void h_ce_reset(void *t, int mode) L("_ZN21V92ConvolutionEncoder5resetEi");
void h_ce_reset(void *t, int mode) { chk_blk(t, g_curT->ce_, "ce reset"); callrec("ce reset mode=%d", mode); }
int h_ce_process(void *t, int *a) L("_ZN21V92ConvolutionEncoder7processEPi");
int h_ce_process(void *t, int *a) {
  int r = ce_ret(a);
  chk_blk(t, g_curT->ce_, "ce process");
  callrec("ce process a=%d,%d,%d,%d state=%s ret=%d", a[0], a[1], a[2], a[3], a == g_state_ptr ? "STATE" : "OTHER", r);
  return r;
}

#ifndef P29_BIN_A
/* ================================================================================================ V92Transmitter model and trials (flavour B) */
typedef struct { unsigned w0, fill; uint8_t buf[0x50]; unsigned digits[12]; int par; unsigned gain; } MT;
static MT g_mt;
static int mt_toi(long double x) { return (isnan(x) || !(x > -2147483649.0L && x < 2147483648.0L)) ? INT_MIN : (int)x; }
static int mt_abs(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }
/* the sign / integer part / fraction digits of the blob's "%c%d.%07d" */
static void md_split(unsigned bits, int *sign, int *ip, int *fp) {
  float f;
  long double v, frac;
  int t;
  memcpy(&f, &bits, 4);
  v = (long double)f;
  t = mt_toi(v);
  *ip = mt_toi(fabsl(v));
  frac = v - (long double)t;
  *fp = mt_abs(mt_toi(frac * 10000000.0L));
  *sign = (v > 0.0L || isnan(v)) ? '+' : '-';
  if (isnan(v)) cov(T_SIGN_NAN); else if (v > 0.0L) cov(T_SIGN_PLUS); else if (v == 0.0L) cov(T_SIGN_ZERO); else cov(T_SIGN_MINUS);
  if (*ip == INT_MIN) cov(T_SPLIT_HUGE);
  if (*fp != 0 && *fp != INT_MIN) cov(T_SPLIT_FRAC);
  if (v < 0.0L && *fp != 0) cov(T_SPLIT_NEG);
}
/* the model's debug level: the real level, lowered by the print hook after the g_drop_after-th print */
static unsigned g_mlvl, g_mseen;
static void mp_emit(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void mp_emit(const char *fmt, ...) {
  va_list ap;
  char b[TXT];
  if (!(g_mlvl > 1)) return;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  exp_push("%s", b);
  g_mseen++;
  if (g_drop_after && g_mseen == g_drop_after) { g_mlvl = g_drop_to; cov(T_RESET_DROP); }
}
static void mt_array(const char *tag, const float *arr, unsigned cnt) {
  unsigned i;
  for (i = 0; i < cnt; i++) {
    int s, ip, fp;
    unsigned w;
    memcpy(&w, &arr[i], 4);
    if (g_mlvl > 1) { md_split(w, &s, &ip, &fp); mp_emit("%s[%d] = %c%d.%07d\r\n", tag, (int)i, s, ip, fp); }
  }
}
static const char *kSepT = "---------------------------\r\n";
static void mt_reset(MT *m, const MapPar92 *mp) {
  unsigned i;
  int s, ip, fp;
  exp_push("me reset mp=MP");
  exp_push("pre reset mp=MP");
  exp_push("pre setCoef c1=arr0 c2=arr1 n1=%u n2=%u", mp->cnt_[0], mp->cnt_[1]);
  exp_push("pf reset");
  exp_push("pf setCoef c1=arr2 c2=arr3 n1=%u n2=%u", mp->cnt_[2], mp->cnt_[3]);
  exp_push("ce reset mode=%d", mp->trellis_);
  memcpy(&m->gain, &mp->gain_, 4);
  m->w0 = (unsigned)mp->w0_;
  g_mseen = 0;
  mp_emit("On V92 Transmitter reset, here are modulation parameters:\r\n");
  mp_emit("================== General ==================\r\n");
  md_split(m->gain, &s, &ip, &fp);
  mp_emit("Gain = %c%d.%07d\r\n", s, ip, fp);
  mp_emit("K = %d\r\n", (int)m->w0);
  mp_emit("trellisType = %d\r\n", mp->trellis_);
  mp_emit("extendEu = %d\r\n", mp->extendEu_);
  for (i = 0; i < 12; i++) mp_emit("m%u = %d\r\n", i, (int)mp->m_[i]);
  mp_emit("================== Pre Coder ================\r\n");
  mp_emit("lz1 = %d\r\n", (int)mp->cnt_[0]);
  mp_emit("lp1 = %d\r\n", (int)mp->cnt_[1]);
  if (mp->cnt_[0]) { mp_emit("%s", kSepT); mt_array("z1", mp->arr_[0], mp->cnt_[0]); mp_emit("%s", kSepT); }
  if (mp->cnt_[1]) { mp_emit("%s", kSepT); mt_array("p1", mp->arr_[1], mp->cnt_[1]); mp_emit("%s", kSepT); }
  mp_emit("================== Pre Filter ===============\r\n");
  mp_emit("lz2 = %d\r\n", (int)mp->cnt_[2]);
  mp_emit("lp2 = %d\r\n", (int)mp->cnt_[3]);
  if (mp->cnt_[2]) { mp_emit("%s", kSepT); mt_array("z2", mp->arr_[2], mp->cnt_[2]); mp_emit("%s", kSepT); }
  if (mp->cnt_[3]) { mp_emit("%s", kSepT); mt_array("p2", mp->arr_[3], mp->cnt_[3]); mp_emit("%s", kSepT); }
  m->fill = 0;
  m->par = 0;
}
static short mt_conv(unsigned fbits, unsigned gbits) {
  float f, g;
  long double p, t;
  memcpy(&f, &fbits, 4);
  memcpy(&g, &gbits, 4);
  p = (long double)f * (long double)g;
  if (isnan(p)) return (short)0x8000;
  t = truncl(p);
  if (t < -32768.0L || t > 32767.0L) return (short)0x8000;
  return (short)(int)t;
}
static void mt_process(MT *m, const uint8_t *bits, unsigned n, short *out, unsigned *cnt) {
  unsigned i, pos = 0, k;
  for (i = 0; i < n; i++) {
    unsigned long long h;
    m->buf[m->fill] = bits[i];
    m->fill++;
    if (m->fill < m->w0) continue;
    h = me_hash(m->buf);
    exp_push("me progress buf=OWN digits=OWN h=%016llx", h);
    for (k = 0; k < 12; k++) m->digits[k] = me_digit(h, k);
    for (k = 0; k < 3; k++) {
      int st[4], j;
      for (j = 0; j < 4; j++) st[j] = pc_state(m->digits, (int)k, m->par, j);
      exp_push("pre process u=OWN row=%u par=%d state=%s out=+%u", k, m->par, k == 0 ? "new" : "same", k * 4);
      m->par = ce_ret(st);
      exp_push("ce process a=%d,%d,%d,%d state=STATE ret=%d", st[0], st[1], st[2], st[3], m->par);
      cov(T_PROC_PAR);
    }
    {
      char w[200];
      unsigned nn = 0;
      w[0] = 0;
      for (k = 0; k < 12; k++) nn += (unsigned)snprintf(w + nn, sizeof w - nn, "%s%08x", k ? "," : "", g_pc_script[k]);
      exp_push("pf process in=FB out=NEW words=%s", w);
    }
    for (k = 0; k < 12; k++) {
      float f, g;
      long double p;
      memcpy(&f, &g_pf_script[k], 4);
      memcpy(&g, &m->gain, 4);
      p = (long double)f * (long double)g;
      out[pos + k] = mt_conv(g_pf_script[k], m->gain);
      if (isnan(p)) cov(T_PROC_NAN);
      else if (isinf(p)) cov(T_PROC_INF);
      else if (p >= 32768.0L) cov(T_PROC_OVF_POS);
      else if (p <= -32769.0L) cov(T_PROC_OVF_NEG);
      else if (p >= 32767.0L) cov(T_PROC_EDGE_POS);
      else if (p <= -32768.0L) cov(T_PROC_EDGE_NEG);
      else if (p < 0.0L && p != truncl(p)) cov(T_PROC_TRUNC_NEG);
      else if (p > 0.0L && p != truncl(p)) cov(T_PROC_TRUNC_POS);
    }
    pos += 12;
    m->fill -= m->w0;
    cov(T_PROC_GROUP);
  }
  *cnt = pos;
}
static unsigned tx_fbits(void) {
  static const unsigned e[] = { 0x00000000u, 0x80000000u, 0x3f800000u, 0xbf800000u, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0xffc00000u, 0x7fa00001u, 0xffa00001u, 0x00000001u, 0x007fffffu, 0x7f7fffffu,
                                0x3f000000u, 0x3fc00000u, 0xc0100000u, 0x3dfcd6e9u, 0x322bcc77u, 0x4b189680u, 0x4f000000u, 0xcf000000u, 0x4efffffeu, 0x461c3c00u, 0xc61c4000u, 0x46fffe00u, 0x47000000u, 0xc7000100u };
  switch (h_rmod(4)) {
    case 0: return e[h_rmod(sizeof e / sizeof e[0])];
    case 1: return h_bits_of((float)h_rrange(-100000, 100000) / 1024.0f);
    case 2: return h_bits_of((float)h_rrange(-30000, 30000) * 0.0001234f);
    default: return h_rnd();
  }
}
static unsigned tx_gain_bits(void) {
  static const unsigned e[] = { 0x3f800000u, 0x3f000000u, 0x40000000u, 0x3fc00000u, 0xbf800000u, 0x00000000u, 0x43800000u, 0x38800000u, 0x7fc00000u, 0x7fa00001u, 0x7f800000u, 0x40800000u, 0x3e800000u, 0x3f7fffffu };
  return h_rpct(70) ? e[h_rmod(sizeof e / sizeof e[0])] : tx_fbits();
}
static unsigned tx_pf_word(unsigned gbits) {
  static const float tg[] = { 32767.0f, 32767.5f, 32767.99f, 32768.0f, 32768.5f, -32767.5f, -32768.0f, -32768.5f, -32768.99f, -32769.0f, 0.5f, -0.5f, 0.99999994f, -0.99999994f, 1.5f, -1.5f, 100.25f, -77.75f, 1e9f };
  float g, f;
  memcpy(&g, &gbits, 4);
  if (h_rpct(55) && g != 0.0f && !isnan(g) && !isinf(g)) {
    f = tg[h_rmod(sizeof tg / sizeof tg[0])] / g;     /* the product is the boundary value up to the rounding of the quotient */
    return h_bits_of(f);
  }
  return h_rpct(60) ? h_bits_of(tg[h_rmod(sizeof tg / sizeof tg[0])]) : tx_fbits();
}
static float g_arrs[4][32];
static void trial_tx(void) {
  int c2 = h_rpct(50), ops, k, w0ok = 1;
  OT *t = (OT *)h_obj_new(SZT + CAN);
  uint8_t *cans = (uint8_t *)t + SZT;
  MapPar92 *mp = (MapPar92 *)h_obj_new(sizeof(MapPar92));
  uint8_t inbuf[256];
  short outbuf[200], ref[200];
  unsigned i;
  can_fill(cans);
  prefill(t, SZT, f_TxObj, NF_TxObj);
  memset(mp, 0, sizeof *mp);
  g_mp92 = mp; g_cur_mp = mp; g_curT = t; g_cur90 = NULL; g_cur92 = NULL;
  dsplibs_debug_level = pick_level();
  exp_clear(); g_expcheck = 1; g_drop_after = 0;
  cov(c2 ? T_C2 : T_C1);
  exp_push("me ctor size=0x54"); exp_push("ce ctor size=0x2008"); exp_push("pre ctor size=0x80 ntaps=0x140"); exp_push("pf ctor size=0x14 ntaps=0x140");
  H_CTOR_PROBE(t, f_TxObj, NF_TxObj, sizeof(OT), (c2 ? tx_c2(t) : tx_c1(t)));
  exp_done("tx ctor");
  g_expcheck = 1;
  {
    struct { void *p; unsigned sz; const char *n; } b[6];
    unsigned j;
    b[0].p = t->buf_; b[0].sz = 0x50; b[0].n = "buf_";
    b[1].p = t->me_; b[1].sz = 0x54; b[1].n = "me_";
    b[2].p = t->flag_; b[2].sz = 1; b[2].n = "flag_";
    b[3].p = t->ce_; b[3].sz = 0x2008; b[3].n = "ce_";
    b[4].p = t->pre_; b[4].sz = 0x80; b[4].n = "pre_";
    b[5].p = t->pf_; b[5].sz = 0x14; b[5].n = "pf_";
    for (j = 0; j < 6; j++) {
      HAl *a = h_find_al(b[j].p, 0);
      if (!a || a->p != (uint8_t *)b[j].p || norm_size(a->size) != b[j].sz) h_die3("the constructor's block %s is missing or has the wrong size (expected 0x%x)", b[j].n, b[j].sz);
    }
    EXPECT_EQ("flag_ byte", *(uint8_t *)t->flag_, 0);
    EXPECT_EQ("w0_ after the constructor", t->w0_, 0);
    EXPECT_EQ("fill_ after the constructor", t->fill_, 0);
    memset(&g_mt, 0, sizeof g_mt);
    g_mt.w0 = 0; g_mt.fill = 0;
    memcpy(g_mt.buf, t->buf_, 0x50);
    memcpy(g_mt.digits, t->digits_, sizeof g_mt.digits);
    g_mt.par = t->par_; memcpy(&g_mt.gain, &t->gain_, 4);
    h_dump_obj(0, "ctor", t, f_TxObj, NF_TxObj);
  }
  can_check(cans, "tx constructor");
  ops = 20 + (int)h_rmod(40);
  for (k = 0; k < ops; k++) {
    unsigned which = k == 0 ? 0 : h_rmod(100);
    if (which < 35) {
      /* reset: arrays, counts, header parameters */
      static const unsigned cnts[] = { 0, 1, 2, 3, 5, 7, 12, 31, 0, 1, 4 };
      unsigned j, lvl = pick_level();
      for (j = 0; j < 4; j++) {
        unsigned cn = cnts[h_rmod(sizeof cnts / sizeof cnts[0])];
        mp->cnt_[j] = cn;
        mp->arr_[j] = g_arrs[j];
        for (i = 0; i < 32; i++) { unsigned w = tx_fbits(); memcpy(&g_arrs[j][i], &w, 4); }
        cov(cn ? (j == 0 ? T_CNT0_NZ : j == 1 ? T_CNT1_NZ : j == 2 ? T_CNT2_NZ : T_CNT3_NZ) : (j == 0 ? T_CNT0_0 : j == 1 ? T_CNT1_0 : j == 2 ? T_CNT2_0 : T_CNT3_0));
      }
      mp->w0_ = h_rpct(85) ? (int32_t)(1 + h_rmod(0x50)) : h_rpct(50) ? (int32_t)h_rnd() : (int32_t)(1 + h_rmod(8));
      w0ok = mp->w0_ >= 1 && mp->w0_ <= 0x50;
      if (h_rpct(30)) { mp->w0_ = (int32_t)(1 + h_rmod(0x50)); w0ok = 1; }
      { unsigned w = tx_gain_bits(); memcpy(&mp->gain_, &w, 4); if (w == 0x7fa00001u || w == 0xffa00001u) cov(T_GAIN_SNAN); if (isnan(mp->gain_)) cov(T_PROC_GAIN_NAN); }
      mp->trellis_ = h_rpct(70) ? (int32_t)h_rmod(3) : (int32_t)h_rnd();
      mp->extendEu_ = h_rpct(60) ? (int32_t)h_rmod(2) : (int32_t)h_rnd();
      for (j = 0; j < 12; j++) mp->m_[j] = h_rpct(60) ? 2u + h_rmod(40) : h_rnd();
      dsplibs_debug_level = lvl;
      g_mlvl = lvl;
      g_prints_seen = 0;
      g_drop_after = h_rpct(30) ? 1 + h_rmod(60) : 0;
      g_drop_to = h_rpct(50) ? 0u : 1u;
      cov(T_RESET);
      if (lvl > 1) cov(T_RESET_DUMP); else cov(T_RESET_QUIET);
      if (t->flag_ && h_rpct(40)) *(uint8_t *)t->flag_ = (uint8_t)(1 + h_rmod(255));   /* reset clears the byte (it is never read) */
      mt_reset(&g_mt, mp);
      tx_reset(t, mp);
      exp_done("tx reset");
      g_drop_after = 0;
      { unsigned gw; memcpy(&gw, &t->gain_, 4); EXPECT_EQ("gain_ (the params' float word)", gw, g_mt.gain); }
      EXPECT_EQ("w0_", t->w0_, g_mt.w0);
      EXPECT_EQ("fill_", t->fill_, 0);
      EXPECT_EQ("par_", t->par_, 0);
      EXPECT_EQ("flag_ byte", *(uint8_t *)t->flag_, 0);
      cov(T_RESET_PAR_FILL);
      h_dump_obj(0, "reset", t, f_TxObj, NF_TxObj);
    } else if (w0ok) {
      /* process: groups of w0 bits */
      unsigned n, cnt = 0xdeadbeefu, refcnt = 0xfeedfaceu, ng = h_rmod(4), r = h_rmod(10);
      unsigned gbits;
      n = r < 2 ? 0 : r < 4 ? h_rmod(g_mt.w0 + 1) : ng * g_mt.w0 + h_rmod(g_mt.w0 + 3);
      if (n > 250) n = 250;
      if (h_rpct(10)) n = h_rmod(4);
      for (i = 0; i < 256; i++) inbuf[i] = h_rpct(85) ? (uint8_t)(h_rnd() & 1) : (uint8_t)h_rnd();
      for (i = 0; i < 12; i++) g_pc_script[i] = tx_fbits();
      gbits = g_mt.gain;
      for (i = 0; i < 12; i++) g_pf_script[i] = tx_pf_word(gbits);
      for (i = 0; i < 200; i++) outbuf[i] = (short)0xa5a5;
      memcpy(ref, outbuf, sizeof ref);
      cov(T_PROC);
      if (n == 0) cov(T_PROC_NODATA);
      if (g_mt.w0 == 1) cov(T_PROC_W0_ONE);
      if (g_mt.fill != 0) cov(T_PROC_CARRY);
      for (i = 0; i < n; i++) if (inbuf[i] > 1) { cov(T_PROC_BYTE); break; }
      if (n && g_mt.fill + n < g_mt.w0) cov(T_PROC_PARTIAL);
      mt_process(&g_mt, inbuf, n, ref, &refcnt);
      if (refcnt > 200) h_die3("the driver produced more symbols than the output buffer holds");
      tx_process(t, inbuf, n, outbuf, &cnt);
      exp_done("tx process");
      EXPECT_EQ("outCnt", cnt, refcnt);
      if (memcmp(outbuf, ref, sizeof ref) != 0) {
        for (i = 0; i < 200; i++) if (outbuf[i] != ref[i]) h_die3("process: symbol %u is 0x%04x, the model says 0x%04x", i, (unsigned)(uint16_t)outbuf[i], (unsigned)(uint16_t)ref[i]);
      }
      EXPECT_EQ("fill_", t->fill_, g_mt.fill);
      EXPECT_EQ("par_", t->par_, g_mt.par);
      if (memcmp(t->buf_, g_mt.buf, 0x50) != 0) h_die3("process: the bit buffer differs from the model");
      if (refcnt) if (memcmp(t->digits_, g_mt.digits, sizeof g_mt.digits) != 0) h_die3("process: the digits differ from the model");
      if (refcnt >= 24) cov(T_PROC_MULTI);
      h_dump_obj(0, "process", t, f_TxObj, NF_TxObj);
    }
    can_check(cans, "tx object");
  }
  /* destruction: NULL members are skipped; the precoder pointer is cleared */
  {
    int d2 = h_rpct(50);
    void *old_pre = t->pre_;
    if (h_rpct(10)) { sysdep_free(t->buf_); t->buf_ = NULL; cov(T_D_NULL_BUF); }
    if (h_rpct(10)) { sysdep_free(t->me_); t->me_ = NULL; cov(T_D_NULL_ME); }
    if (h_rpct(10)) { sysdep_free(t->flag_); t->flag_ = NULL; cov(T_D_NULL_FLAG); }
    if (h_rpct(10)) { sysdep_free(t->pre_); t->pre_ = NULL; old_pre = NULL; cov(T_D_NULL_PRE); }
    if (h_rpct(10)) { sysdep_free(t->pf_); t->pf_ = NULL; cov(T_D_NULL_PF); }
    if (h_rpct(10)) { sysdep_free(t->ce_); t->ce_ = NULL; cov(T_D_NULL_CE); }
    cov(d2 ? T_D2 : T_D1);
    if (t->pre_) exp_push("pre dtor");
    if (t->pf_) exp_push("pf dtor");
    if (t->ce_) exp_push("ce dtor");
    if (d2) tx_d2(t); else tx_d1(t);
    __asm__ volatile("" ::: "memory");
    exp_done("tx dtor");
    if (t->pre_ != NULL) h_die3("the destructor did not clear pre_ (the blob stores 0 there after the precoder's destruction)");
    (void)old_pre;
    can_check(cans, "tx dtor");
    h_dump_obj(0, "dtor", t, f_TxObj, NF_TxObj);
  }
  g_curT = NULL; g_mp92 = NULL;
  h_trial_end("transmitter");
  cov(C_TRIALS);
}
#endif

#endif /* !P29_BIN_C */

#ifdef P29_BIN_C
/* ================================================================================================ integration: V92Transmitter over the REAL sub-objects (blob / the P15, P16, P02 port objects); no model: blob vs port */
static void fill_small_tab(uint32_t *t) {
  unsigned i, kind = h_rmod(3), sc = 1u + h_rmod(4);
  for (i = 0; i < 1024; i++) t[i] = kind == 0 ? (2u * i + 1u) * sc % 1001u : kind == 1 ? h_rmod(1001) : 77u + (i & 3u);
}
static unsigned small_fbits(void) {
  /* |c| <= 1/64 and at most 16 taps: the precoder's feedback loop (s = fb1 + X + fb2, fb = FIR(s)) stays bounded, so every symbol finds a candidate below the 1e12 limit (the blob uses stale stack slots otherwise) */
  static const unsigned e[] = { 0x00000000u, 0x3c800000u, 0xbc800000u, 0x3c000000u, 0xbc000000u, 0x3b800000u, 0x3c4ccccdu };
  return h_rpct(60) ? e[h_rmod(sizeof e / sizeof e[0])] : h_bits_of((float)h_rrange(-8, 8) / 512.0f);
}
static void integ_dump(OT *t, const char *tag, int subs) {
  uint8_t tmp[sizeof(PcObj) > sizeof(PfObj) ? sizeof(PcObj) : sizeof(PfObj)];
  PcObj *pc = (PcObj *)tmp;
  PfObj *pf = (PfObj *)tmp;
  h_dump_obj(0, tag, t, f_TxObj, NF_TxObj);
  h_emit(g_show, "%s me %016llx ce %016llx", tag, fnv(t->me_, 0x54), fnv(t->ce_, 0x2008));
  if (!subs) return;       /* right after the constructor most members of the two blocks are the allocator's fill, which depends on the block's size (x86-64: larger) */
  memcpy(tmp, t->pre_, sizeof(PcObj));
  if (pc->idxc_ != NULL && !h_find_al(pc->idxc_, 1)) pc->idxc_ = NULL;     /* borrowed from the (non-heap) params block */
  h_dump_obj(0, tag, pc, f_PcObj, NF_PcObj);
  memcpy(tmp, t->pf_, sizeof(PfObj));
  h_dump_obj(0, tag, pf, f_PfObj, NF_PfObj);
}
static float g_iarr[4][24];
static void trial_integ(void) {
  int c2 = h_rpct(50), ops, k, ready = 0;
  OT *t = (OT *)h_obj_new(SZT + CAN);
  uint8_t *cans = (uint8_t *)t + SZT;
  MapPar92 *mp = (MapPar92 *)h_obj_new(sizeof(MapPar92));
  uint32_t *tabs[6];
  uint8_t inbuf[256];
  short out[200];
  unsigned i, w0 = 1;
  can_fill(cans);
  prefill(t, SZT, f_TxObj, NF_TxObj);
  memset(mp, 0, sizeof *mp);
  g_curT = t; g_cur_mp = mp; g_cur90 = NULL; g_cur92 = NULL; g_expcheck = 0;
  dsplibs_debug_level = pick_level();
  for (i = 0; i < 6; i++) { tabs[i] = (uint32_t *)sysdep_malloc(4096u); fill_small_tab(tabs[i]); }
  cov(c2 ? I_C2 : I_C1);
  H_CTOR_PROBE(t, f_TxObj, NF_TxObj, sizeof(OT), (c2 ? tx_c2(t) : tx_c1(t)));
  integ_dump(t, "ctor", 0);
  ops = 8 + (int)h_rmod(14);
  for (k = 0; k < ops; k++) {
    if (k == 0 || h_rpct(25)) {
      static const unsigned cn[] = { 0, 0, 4, 8, 12, 16, 4 };      /* FloatFIR / FloatIIR taps are rounded down to a multiple of 4: fewer than 4 would wrap a zero-length history (crashes the blob) */
      unsigned j;
      for (j = 0; j < 6; j++) {
        mp->lcp_[j] = tabs[j];
        mp->idxc_[j] = h_rmod(6);
        mp->lc_[j] = 1u + h_rmod(h_rpct(30) ? 4 : 128);
      }
      for (j = 0; j < 12; j++) mp->m_[j] = 2u + h_rmod(h_rpct(20) ? 2 : 7);
      for (j = 0; j < 4; j++) {
        mp->cnt_[j] = cn[h_rmod(sizeof cn / sizeof cn[0])];
        mp->arr_[j] = g_iarr[j];
        for (i = 0; i < 24; i++) { unsigned w = small_fbits(); memcpy(&g_iarr[j][i], &w, 4); }
      }
      cov(mp->cnt_[0] || mp->cnt_[1] ? I_FIR : I_NOFIR);
      cov(mp->cnt_[2] || mp->cnt_[3] ? I_PF : I_NOPF);
      mp->trellis_ = (int32_t)h_rmod(3);
      cov(mp->trellis_ == 0 ? I_TR0 : mp->trellis_ == 1 ? I_TR1 : I_TR2);
      mp->extendEu_ = (int32_t)h_rmod(2);
      { static const unsigned gb[] = { 0x3f800000u, 0x3f000000u, 0x40000000u, 0x42c80000u, 0x3dcccccdu, 0x44800000u }; unsigned w = gb[h_rmod(sizeof gb / sizeof gb[0])]; memcpy(&mp->gain_, &w, 4); }
      w0 = 1u + h_rmod(11);
      mp->w0_ = (int32_t)w0;
      dsplibs_debug_level = pick_level();
      if (dsplibs_debug_level > 1) cov(I_PRINT);
      tx_reset(t, mp);
      cov(I_RESET);
      ready = 1;
      integ_dump(t, "reset", 1);
    } else if (ready) {
      unsigned n = h_rpct(15) ? h_rmod(w0 + 1) : (1u + h_rmod(3)) * w0 + h_rmod(w0 + 1), cnt = 0xdeadbeefu;
      unsigned long long hh = 0xcbf29ce484222325ULL;
      for (i = 0; i < 256; i++) inbuf[i] = (uint8_t)(h_rnd() & 1);
      for (i = 0; i < 200; i++) out[i] = (short)0xa5a5;
      cov(I_PROC);
      tx_process(t, inbuf, n, out, &cnt);
      h_emit(g_show, "process n=%u cnt=%u h=%016llx", n, cnt, fnv(out, sizeof out));
      (void)hh;
      if (cnt > 200) h_die3("process wrote %u symbols", cnt);
      if (cnt) cov(I_GROUPS); else cov(I_PARTIAL);
      for (i = 0; i < cnt; i++) if (out[i]) { cov(I_OUTNZ); break; }
      integ_dump(t, "process", 1);
    }
    can_check(cans, "tx object");
  }
  { int d2 = h_rpct(50); cov(d2 ? I_D2 : I_D1); if (d2) tx_d2(t); else tx_d1(t); }
  __asm__ volatile("" ::: "memory");
  h_dump_obj(0, "dtor", t, f_TxObj, NF_TxObj);
  for (i = 0; i < 6; i++) sysdep_free(tabs[i]);
  g_curT = NULL;
  h_trial_end("integration");
  cov(I_TRIALS);
}
#endif
/* ================================================================================================ main */
static const int req_q90[] = { C_TRIALS, Q_C1, Q_C2, Q_D1, Q_D2, Q_D_NULLMAP, Q_D_NULLBUF, Q_RESET, Q_RESETNS, Q_K0, Q_KPOS, Q_SETSIZE, Q_SET_ZERO, Q_SET_LT, Q_SET_GE, Q_SET_HUGE, Q_NOF, Q_NOF_ZERO, Q_NOF_EXACT,
                               Q_NOF_ROUND, Q_NOF_FIRST, Q_NOF_NOFIRST, Q_NOF_WRAP, Q_PA, Q_PA_NOTSET, Q_PA_OK, Q_PA_OVER, Q_PA_UNDER, Q_PA_SHIFT, Q_PA_NEXT_ZERO, Q_PA_NEXT_NZ, Q_PB, Q_PB_NOTSET, Q_PB_OK, Q_PB_OVER,
                               Q_PC, Q_PC_NOTSET, Q_PC_OK, Q_PC_UNDER, Q_PC_SHIFT, Q_PC_NEXT_ZERO, Q_PC_NEXT_NZ, Q_FIRST_CLEARED, Q_FIRST_POKED, Q_PRINT_ON, Q_PRINT_OFF, Q_MAP_ZERO, Q_MAP_CNT, Q_FILL_CARRY, Q_HUGE_PROC, 0 };
static const int req_q92[] = { C_TRIALS, Q_C1, Q_C2, Q_D1, Q_D2, Q_D_NULLMAP, Q_D_NULLBUF, Q_RESET, Q_SETSIZE, Q_SET_ZERO, Q_SET_LT, Q_SET_GE, Q_SET_HUGE, Q_NOF, Q_NOF_ZERO, Q_NOF_EXACT, Q_NOF_ROUND, Q_NOF_WRAP,
                               Q_PA, Q_PA_NOTSET, Q_PA_OK, Q_PA_OVER, Q_PA_UNDER, Q_PA_SHIFT, Q_PA_NEXT_ZERO, Q_PA_NEXT_NZ, Q_PB, Q_PB_NOTSET, Q_PB_OK, Q_PB_OVER, Q_PC, Q_PC_NOTSET, Q_PC_OK, Q_PC_UNDER, Q_PC_SHIFT,
                               Q_PC_NEXT_ZERO, Q_PC_NEXT_NZ, Q_FIRST_CLEARED, Q_FIRST_POKED, Q_PRINT_ON, Q_PRINT_OFF, Q_MAP_ZERO, Q_MAP_CNT, Q_FILL_CARRY, Q_V92_DIV12, Q_HUGE_PROC, 0 };
static const int req_i[] = { I_TRIALS, I_C1, I_C2, I_D1, I_D2, I_RESET, I_PRINT, I_TR0, I_TR1, I_TR2, I_PROC, I_GROUPS, I_PARTIAL, I_FIR, I_NOFIR, I_PF, I_NOPF, I_OUTNZ, 0 };
static const int req_t[] = { C_TRIALS, T_C1, T_C2, T_D1, T_D2, T_D_NULL_BUF, T_D_NULL_ME, T_D_NULL_FLAG, T_D_NULL_PRE, T_D_NULL_PF, T_D_NULL_CE, T_RESET, T_RESET_DUMP, T_RESET_QUIET, T_RESET_DROP, T_CNT0_0, T_CNT0_NZ,
                             T_CNT1_0, T_CNT1_NZ, T_CNT2_0, T_CNT2_NZ, T_CNT3_0, T_CNT3_NZ, T_SIGN_PLUS, T_SIGN_MINUS, T_SIGN_NAN, T_SIGN_ZERO, T_SPLIT_HUGE, T_SPLIT_FRAC, T_SPLIT_NEG, T_PROC, T_PROC_NODATA,
                             T_PROC_PARTIAL, T_PROC_GROUP, T_PROC_MULTI, T_PROC_CARRY, T_PROC_W0_ONE, T_PROC_OVF_POS, T_PROC_OVF_NEG, T_PROC_EDGE_POS, T_PROC_EDGE_NEG, T_PROC_NAN, T_PROC_INF, T_PROC_TRUNC_NEG,
                             T_PROC_TRUNC_POS, T_PROC_GAIN_NAN, T_PROC_PAR, T_PROC_BYTE, T_GAIN_SNAN, T_RESET_PAR_FILL, 0 };

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, i, tr, bad = 0;
  const char *seed = getenv("P29_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P29_DBG")) g_show = 1;
  p11_layout_check();
  p29_layout_check();
  h_x87_check();
  h_arena_init();
  h_log_map_add(sizeof(TxObj), 0x60); h_log_map_add(sizeof(PcObj), 0x80); h_log_map_add(sizeof(PfObj), 0x14); h_log_map_add(MAPPER_SZ, 0x704);
  g_exp = malloc((size_t)EXPMAX * TXT);
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0;
  h_covreq[H_COV_DTOR_NULL] = 0;
#if defined(P29_BIN_A)
  if (sc != 0 && sc != 1) { fprintf(stderr, "scenario %d is not in flavour A\n", sc); return 2; }
#elif defined(P29_BIN_C)
  if (sc != 3) { fprintf(stderr, "scenario %d is not in flavour C\n", sc); return 2; }
  p16_layout_check();
#else
  if (sc != 2) { fprintf(stderr, "scenario %d is not in flavour B\n", sc); return 2; }
#endif
  { const int *r = sc == 0 ? req_q90 : sc == 1 ? req_q92 : sc == 3 ? req_i : req_t; for (; *r; r++) g_covreq[*r] = 1; }
  switch (sc) {
    case 0: for (tr = 0; tr < 400; tr++) trial_queue(90); break;
    case 1: for (tr = 0; tr < 400; tr++) trial_queue(92); break;
#if !defined(P29_BIN_A) && !defined(P29_BIN_C)
    case 2: for (tr = 0; tr < 300; tr++) trial_tx(); break;
#endif
#ifdef P29_BIN_C
    case 3: for (tr = 0; tr < 250; tr++) trial_integ(); break;
#endif
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu prints %lu callee calls %lu", h_nmalloc, h_nfree, g_nprint, g_ncall);
  for (i = 1; i < COV_MAX; i++) {
    h_emit(1, "COV %-8s %lu", cov_name(i), g_covn[i]);
    if (g_covreq[i] && g_covn[i] == 0) { fprintf(stderr, "COVERAGE FAIL: counter %d was never reached\n", i); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P29_NOCOV")) ? 2 : 0;
}
