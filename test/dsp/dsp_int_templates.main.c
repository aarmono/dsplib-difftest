/*
 * Differential test harness for port chunk P01 (src/dsp/dsp_int_templates.cpp): the 51 integer class-template member
 * functions of dsplibs.o (Scrambler / Descrambler / Parallel + Serial differential coders / Queue<float>).
 *
 * ONE C source, no #ifdef, built three ways by test/dsp/dsp_int_templates.sh:
 *   p01_test_blob   linked against the real dsplibs.o (the 51 template instances are WEAK members of the blob)
 *   p01_test_port   linked against the port object (g++ -m32 -O0 / -O2 -fno-exceptions -fno-rtti), no blob
 *   (optional)      -m64 build of the same source against the 64-bit port object, compared with the 32-bit blob's output
 * The functions under test are reached through __asm__ labels carrying the Itanium-mangled names, which are the same in the
 * blob and in the C++ port (the port gets them from the compiler).  Nothing here includes the port headers: the layouts of the
 * five object kinds are MIRRORED below as C structs with REAL pointer members, so that on x86-64 the mirror widens exactly like
 * the port's classes.  The mirror is checked against the blob (the blob build runs it) and against the port header
 * (test/p01_layout_probe.cpp, linked into every binary: it exports sizeof / offsetof of the real classes).
 *
 * Each scenario runs in its OWN process (argv[1]).  The sysdep_malloc / sysdep_free stubs are defined by the shared harness
 * library test/harness/harness.[ch] (the shared test/stub_env.c is built with them renamed): a deterministic bump allocator with poisoned red zones and a per-block fill
 * pattern derived from the allocation id, an allocation TABLE (sorted by address) and a LOG.  Every canonical text line (object
 * fields by NAME, pointers normalised to "A<id>+<byte offset>" - never a raw address -, the whole heap block, the malloc / free
 * sequence) is fed into a 64-bit FNV hash; sampled lines are also printed.  The stdout of a scenario therefore is
 * architecture independent: the 32-bit blob run is the GOLDEN output a 64-bit port run is diffed against.
 *
 * INDEPENDENT MODELS (written from the disassembly facts in the port header comment, not from the port code): a delay-line model
 * of the scrambler / descrambler (arrays of past symbols, no pointers, no wrap logic), a cyclic-array model of the queue, a
 * lane-array model of the parallel differential coders, a one-line model of the serial ones.  A model mismatch exits 3.
 * Coverage counters derived from the INPUTS exit 2 when a required class of input was never produced.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"


#define L(m) __asm__(m)
/* ---- Queue<float> ---- */
extern void q_reset(void *) L("_ZN5QueueIfE5resetEv");
extern void q_dtor(void *) L("_ZN5QueueIfED1Ev");
extern void q_ctor(void *, unsigned) L("_ZN5QueueIfEC1Ej");
extern int q_write1(void *, float) L("_ZN5QueueIfE5writeEf");
extern int q_read(void *, void *, unsigned) L("_ZN5QueueIfE4readEPfj");
extern int q_writeN(void *, void *, unsigned) L("_ZN5QueueIfE5writeEPfj");
/* ---- Scrambler<uchar,uchar> ---- */
extern void shh_reset(void *, uint32_t) L("_ZN9ScramblerIhhE5resetEh");
extern void shh_dtor(void *) L("_ZN9ScramblerIhhED1Ev");
extern uint8_t shh_p1(void *, uint32_t) L("_ZN9ScramblerIhhE7processEh");
extern void shh_rhi(void *) L("_ZN9ScramblerIhhE19resetHistoryIndexesEv");
extern void shh_cht(void *) L("_ZN9ScramblerIhhE15copyHistoryTailEv");
extern void shh_ones(void *, void *, unsigned) L("_ZN9ScramblerIhhE14processAllOnesEPhj");
extern void shh_zeros(void *, void *, unsigned) L("_ZN9ScramblerIhhE15processAllZerosEPhj");
extern void shh_pn(void *, const void *, void *, unsigned) L("_ZN9ScramblerIhhE7processEPKhPhj");
extern void shh_ctor(void *, unsigned, unsigned, unsigned) L("_ZN9ScramblerIhhEC1Ejjj");
/* ---- Scrambler<int,uchar> ---- */
extern void sih_cht(void *) L("_ZN9ScramblerIihE15copyHistoryTailEv");
extern void sih_reset(void *, uint32_t) L("_ZN9ScramblerIihE5resetEi");
extern void sih_ctor(void *, unsigned, unsigned, unsigned) L("_ZN9ScramblerIihEC1Ejjj");
extern void sih_dtor(void *) L("_ZN9ScramblerIihED1Ev");
extern void sih_pn(void *, const void *, void *, unsigned) L("_ZN9ScramblerIihE7processEPKiPhj");
extern void sih_rhi(void *) L("_ZN9ScramblerIihE19resetHistoryIndexesEv");
/* ---- Scrambler<uchar,int> ---- */
extern void shi_dtor(void *) L("_ZN9ScramblerIhiED1Ev");
extern void shi_rhi(void *) L("_ZN9ScramblerIhiE19resetHistoryIndexesEv");
extern void shi_ctor(void *, unsigned, unsigned, unsigned) L("_ZN9ScramblerIhiEC1Ejjj");
extern int shi_p1(void *, uint32_t) L("_ZN9ScramblerIhiE7processEh");
extern void shi_reset(void *, uint32_t) L("_ZN9ScramblerIhiE5resetEh");
extern void shi_cht(void *) L("_ZN9ScramblerIhiE15copyHistoryTailEv");
/* ---- Descrambler<uchar,int> ---- */
extern void dhi_rhi(void *) L("_ZN11DescramblerIhiE19resetHistoryIndexesEv");
extern void dhi_pn(void *, const void *, void *, unsigned) L("_ZN11DescramblerIhiE7processEPKhPij");
extern void dhi_ctor(void *, unsigned, unsigned, unsigned) L("_ZN11DescramblerIhiEC1Ejjj");
extern int dhi_p1(void *, uint32_t) L("_ZN11DescramblerIhiE7processEh");
extern void dhi_reset(void *, uint32_t) L("_ZN11DescramblerIhiE5resetEh");
extern void dhi_cht(void *) L("_ZN11DescramblerIhiE15copyHistoryTailEv");
extern void dhi_dtor(void *) L("_ZN11DescramblerIhiED1Ev");
/* ---- Descrambler<int,int> ---- */
extern void dii_ctor(void *, unsigned, unsigned, unsigned) L("_ZN11DescramblerIiiEC1Ejjj");
extern void dii_rhi(void *) L("_ZN11DescramblerIiiE19resetHistoryIndexesEv");
extern void dii_cht(void *) L("_ZN11DescramblerIiiE15copyHistoryTailEv");
extern void dii_dtor(void *) L("_ZN11DescramblerIiiED1Ev");
extern void dii_reset(void *, uint32_t) L("_ZN11DescramblerIiiE5resetEi");
extern int dii_p1(void *, int) L("_ZN11DescramblerIiiE7processEi");
/* ---- ParallelDifferential{Encoder,Decoder}<uchar> ---- */
extern void pde_dtor(void *) L("_ZN27ParallelDifferentialEncoderIhED1Ev");
extern void pde_ctor(void *, unsigned) L("_ZN27ParallelDifferentialEncoderIhEC1Ej");
extern int pde_reset(void *, unsigned, uint32_t) L("_ZN27ParallelDifferentialEncoderIhE5resetEjh");
extern void pde_process(void *, void *, void *) L("_ZN27ParallelDifferentialEncoderIhE7processEPhS1_");
extern void pdd_dtor(void *) L("_ZN27ParallelDifferentialDecoderIhED1Ev");
extern void pdd_ctor(void *, unsigned) L("_ZN27ParallelDifferentialDecoderIhEC1Ej");
extern int pdd_reset(void *, unsigned, uint32_t) L("_ZN27ParallelDifferentialDecoderIhE5resetEjh");
extern void pdd_process(void *, void *, void *) L("_ZN27ParallelDifferentialDecoderIhE7processEPhS1_");
/* ---- SerialDifferential ---- */
extern uint8_t sde_process(void *, uint32_t) L("_ZN25SerialDifferentialEncoderIhE7processEh");
extern uint8_t sdd_process(void *, uint32_t) L("_ZN25SerialDifferentialDecoderIhE7processEh");
extern int sddi_process(void *, int) L("_ZN25SerialDifferentialDecoderIiE7processEi");

/* the layout probe (C++, compiled from the PORT header; exports sizeof / offsetof of the real classes) */
extern const unsigned p01_layout_probe[];
extern const unsigned p01_layout_probe_count;

/* ---------------------------------------------------------------------------------------------- mirrors of the objects */
typedef struct { void *buf, *head0, *t1_0, *end, *wr, *t1, *t2; uint32_t tail; } ScrObj; /* Scrambler / Descrambler, 4 variants */
typedef struct { void *state; uint32_t cap, n; } ParObj;
typedef struct { void *buf, *last, *rd, *wr; uint32_t size; } QObj;
typedef struct { uint8_t state; } SerH;
typedef struct { int32_t state; } SerI;

#if defined(__i386__)
_Static_assert(sizeof(ScrObj) == 0x20 && offsetof(ScrObj, buf) == 0 && offsetof(ScrObj, head0) == 4 && offsetof(ScrObj, t1_0) == 8 &&
                   offsetof(ScrObj, end) == 0xc && offsetof(ScrObj, wr) == 0x10 && offsetof(ScrObj, t1) == 0x14 &&
                   offsetof(ScrObj, t2) == 0x18 && offsetof(ScrObj, tail) == 0x1c,
               "Scrambler/Descrambler layout (measured from the blob)");
_Static_assert(sizeof(ParObj) == 0xc && offsetof(ParObj, cap) == 4 && offsetof(ParObj, n) == 8, "ParallelDifferential layout");
_Static_assert(sizeof(QObj) == 0x14 && offsetof(QObj, last) == 4 && offsetof(QObj, rd) == 8 && offsetof(QObj, wr) == 0xc &&
                   offsetof(QObj, size) == 0x10,
               "Queue<float> layout");
#endif


/* field tables for the shared harness library's h_dump_obj (kinds: 'o' = OWNING heap pointer, its block is dumped too; 'p' =
 * pointer normalised to A<id>+<off>; 'u' = u32) */
static const HFld f_scr[] = { HFLD(ScrObj, buf, 'o'), HFLD(ScrObj, head0, 'p'), HFLD(ScrObj, t1_0, 'p'), HFLD(ScrObj, end, 'p'),
                             HFLD(ScrObj, wr, 'p'), HFLD(ScrObj, t1, 'p'), HFLD(ScrObj, t2, 'p'), HFLD(ScrObj, tail, 'u') };
static const HFld f_par[] = { HFLD(ParObj, state, 'o'), HFLD(ParObj, cap, 'u'), HFLD(ParObj, n, 'u') };
static const HFld f_q[] = { HFLD(QObj, buf, 'o'), HFLD(QObj, last, 'p'), HFLD(QObj, rd, 'p'), HFLD(QObj, wr, 'p'), HFLD(QObj, size, 'u') };


/* ===================================================================================================== scenario 0 */
/* SerialDifferentialEncoder<uchar> / SerialDifferentialDecoder<uchar>: exhaustive 256 x 256 + chained round trip */
enum { C0_SDE = 0, C0_SDD, C0_ALL, C0_RT, C0_STATE_BIT7 };
static void sc_serial_h(void) {
  unsigned s, x, i;
  unsigned long ctr = 0, rt_ok = 0;
  SerH *e = (SerH *)h_obj_new(sizeof(SerH)), *d = (SerH *)h_obj_new(sizeof(SerH));
  h_cov_define(C0_SDE, "sde.process calls");
  h_cov_define(C0_SDD, "sdd.process calls");
  h_cov_define(C0_ALL, "all 256x256 (state,in)");
  h_cov_define(C0_RT, "encode->decode round trips");
  h_cov_define(C0_STATE_BIT7, "state/in with bit 7 set");
  for (s = 0; s < 256; s++)
    for (x = 0; x < 256; x++) {
      uint8_t r;
      e->state = (uint8_t)s;
      r = sde_process(e, h_garbage_hi(x));
      h_cov_hit(C0_SDE);
      if (r != (uint8_t)(s ^ x) || e->state != (uint8_t)(s ^ x)) h_die3("sde s=%u x=%u -> r=%u st=%u", s, x, r, e->state);
      h_emit(h_sample(&ctr, 40, 997), "SDE s=%u x=%u r=%u st=%u", s, x, r, e->state);
      d->state = (uint8_t)s;
      r = sdd_process(d, h_garbage_hi(x));
      h_cov_hit(C0_SDD);
      if (r != (uint8_t)(s ^ x) || d->state != (uint8_t)x) h_die3("sdd s=%u x=%u -> r=%u st=%u", s, x, r, d->state);
      h_emit(0, "SDD s=%u x=%u r=%u st=%u", s, x, r, d->state);
      h_cov_hit(C0_ALL);
      if ((s | x) & 0x80) h_cov_hit(C0_STATE_BIT7);
    }
  e->state = d->state = 0x5a;
  for (i = 0; i < 200000; i++) {
    uint8_t in = (uint8_t)h_rnd(), y = sde_process(e, h_garbage_hi(in)), out = sdd_process(d, h_garbage_hi(y));
    h_cov_hit(C0_SDE);
    h_cov_hit(C0_SDD);
    if (out != in) h_die3("round trip failed at %u", i);
    h_cov_hit(C0_RT);
    rt_ok++;
    h_emit(0, "RT %u %u %u", in, y, out);
  }
  h_emit(1, "serial uchar: %lu round trips, final enc state %u dec state %u", rt_ok, e->state, d->state);
}

/* SerialDifferentialDecoder<int> */
enum { C1_CALLS = 0, C1_EDGE, C1_NEG, C1_HIGH };
static void sc_serial_i(void) {
  static const int32_t edge[] = { 0, 1, -1, 2, 0x7fffffff, (int32_t)0x80000000u, (int32_t)0x80000001u, 0x55555555, (int32_t)0xaaaaaaaau,
                                  0xff, 0x100, (int32_t)0xffff0000u, 0x00010000 };
  unsigned long ctr = 0;
  unsigned a, b, i;
  SerI *d = (SerI *)h_obj_new(sizeof(SerI));
  h_cov_define(C1_CALLS, "sddi.process calls");
  h_cov_define(C1_EDGE, "edge x edge pairs");
  h_cov_define(C1_NEG, "negative input");
  h_cov_define(C1_HIGH, "bits 31..16 differ");
  for (a = 0; a < sizeof edge / sizeof edge[0]; a++)
    for (b = 0; b < sizeof edge / sizeof edge[0]; b++) {
      int r;
      d->state = edge[a];
      r = sddi_process(d, edge[b]);
      h_cov_hit(C1_CALLS);
      h_cov_hit(C1_EDGE);
      if (r != (int32_t)((uint32_t)edge[a] ^ (uint32_t)edge[b]) || d->state != edge[b]) h_die3("sddi %d %d -> %d", edge[a], edge[b], r);
      h_emit(h_sample(&ctr, 30, 13), "SDDI s=%d x=%d r=%d st=%d", edge[a], edge[b], r, d->state);
    }
  d->state = 0;
  for (i = 0; i < 300000; i++) {
    int32_t x = (int32_t)h_rnd(), prev = d->state, r;
    if (h_rpct(5)) x = (int32_t)(h_rnd() & 0xffu);
    r = sddi_process(d, x);
    h_cov_hit(C1_CALLS);
    if (x < 0) h_cov_hit(C1_NEG);
    if (((uint32_t)x ^ (uint32_t)prev) >> 16) h_cov_hit(C1_HIGH);
    if (r != (int32_t)((uint32_t)prev ^ (uint32_t)x) || d->state != x) h_die3("sddi random %d %d", prev, x);
    h_emit(h_sample(&ctr, 0, 25000), "SDDI s=%d x=%d r=%d", prev, x, r);
    if (i < 64 || (i % 50000) == 0) h_emit(0, "x");
  }
  h_emit(1, "serial int: final state %d", d->state);
}

/* ============================================================================ scenarios 2 / 3: parallel differential */
enum { C2_CTOR = 0, C2_DTOR, C2_RESET_OK, C2_RESET_ERR, C2_RESET_ZERO, C2_RESET_CAP, C2_RESET_MID, C2_RESET_HUGE, C2_CAP0,
       C2_PROC, C2_PROC0, C2_PROC_INPLACE, C2_PROC_FULL, C2_HEAPZERO, C2_RESET_BITS, C2_NERR_KEEP };
static void sc_parallel(int dec) {
  static const unsigned caps[] = { 0, 1, 2, 3, 5, 8, 16, 17, 31, 64, 100, 255, 300, 480 };
  unsigned long ctr = 0;
  unsigned trial, ntrials = 1600;
  h_cov_define(C2_CTOR, "ctor");
  h_cov_define(C2_DTOR, "dtor");
  h_cov_define(C2_RESET_OK, "reset ok (n <= cap)");
  h_cov_define(C2_RESET_ERR, "reset error (n > cap)");
  h_cov_define(C2_RESET_ZERO, "reset n == 0");
  h_cov_define(C2_RESET_CAP, "reset n == cap");
  h_cov_define(C2_RESET_MID, "reset 0 < n < cap");
  h_cov_define(C2_RESET_HUGE, "reset n == 0xffffffff");
  h_cov_define(C2_CAP0, "cap == 0 (malloc(0))");
  h_cov_define(C2_PROC, "process");
  h_cov_define(C2_PROC0, "process with n_ == 0");
  h_cov_define(C2_PROC_INPLACE, "process in place");
  h_cov_define(C2_PROC_FULL, "process with n_ == cap > 0");
  h_cov_define(C2_HEAPZERO, "ctor zero-fills the block");
  h_cov_define(C2_RESET_BITS, "reset value with high bits");
  h_cov_define(C2_NERR_KEEP, "failed reset leaves n_ and lanes");
  h_scenario_begin(0);
  for (trial = 0; trial < ntrials; trial++) {
    unsigned cap = trial < sizeof caps / sizeof caps[0] ? caps[trial] : (h_rpct(20) ? (unsigned)h_rrange(250, 600) : (unsigned)h_rrange(0, 70));
    ParObj *o = (ParObj *)h_obj_new(sizeof(ParObj));
    uint8_t model[700], in[700], out[700];
    unsigned nm = 0, nops, k, i;
    unsigned long nm0 = h_nmalloc, nf0 = h_nfree;
    int op;
    h_wild = 0;
    H_CTOR_PROBE(o, f_par, 3, sizeof(ParObj), if (dec) pdd_ctor(o, cap); else pde_ctor(o, cap));
    h_cov_hit(C2_CTOR);
    if (h_nmalloc != nm0 + 1 || h_al[h_last_alloc].size != cap) h_die3("ctor must allocate exactly %u bytes once", cap);
    if (cap == 0) h_cov_hit(C2_CAP0);
    if (o->cap != cap || o->n != 0 || o->state != h_al[h_last_alloc].p) h_die3("ctor fields");
    for (i = 0; i < cap; i++) if (((uint8_t *)o->state)[i] != 0) h_die3("ctor did not zero lane %u", i);
    h_cov_hit(C2_HEAPZERO);
    memset(model, 0, sizeof model);
    h_dump_obj(h_sample(&ctr, 12, 211), dec ? "PDD ctor" : "PDE ctor", o, f_par, 3);
    h_check_not_wild("parallel ctor");
    nops = (unsigned)h_rrange(4, 40);
    for (op = 0; op < (int)nops; op++) {
      if (h_rpct(35)) { /* reset */
        unsigned n;
        uint32_t v = h_rnd();
        int r, expect, pick = (int)h_rmod(8);
        n = pick == 0 ? 0 : pick == 1 ? cap : pick == 2 ? cap + 1 : pick == 3 ? 0xffffffffu : pick == 4 ? cap + (unsigned)h_rrange(1, 40) : (unsigned)h_rrange(0, (int)cap);
        if (v & 0xff00u) h_cov_hit(C2_RESET_BITS);
        r = dec ? pdd_reset(o, n, v) : pde_reset(o, n, v);
        expect = n > cap;
        if (r != expect) h_die3("reset(%u) with cap %u returned %d", n, cap, r);
        if (r == 0) {
          for (i = 0; i < n; i++) model[i] = (uint8_t)v;
          nm = n;
          h_cov_hit(C2_RESET_OK);
          if (n == 0) h_cov_hit(C2_RESET_ZERO);
          else if (n == cap) h_cov_hit(C2_RESET_CAP);
          else h_cov_hit(C2_RESET_MID);
        } else {
          h_cov_hit(C2_RESET_ERR);
          if (n == 0xffffffffu) h_cov_hit(C2_RESET_HUGE);
          if (o->n == nm) h_cov_hit(C2_NERR_KEEP);
        }
        if (o->n != nm) h_die3("n_ is %u, expected %u", o->n, nm);
        h_emit(h_sample(&ctr, 12, 2999), "%s reset n=%u -> %d n_=%u", dec ? "PDD" : "PDE", n, r, o->n);
      } else { /* process */
        int inplace = h_rpct(25);
        uint8_t *src = in, *dst = inplace ? in : out;
        for (i = 0; i < nm + 8; i++) { in[i] = (uint8_t)h_rnd(); out[i] = 0xEE; }
        if (h_rpct(15)) for (i = 0; i < nm; i++) in[i] = (uint8_t)(i * 3);
        {
          uint8_t save[700];
          memcpy(save, in, nm + 8);
          if (dec) pdd_process(o, src, dst); else pde_process(o, src, dst);
          for (i = 0; i < nm; i++) {
            uint8_t y = dec ? (uint8_t)(save[i] ^ model[i]) : (uint8_t)(save[i] ^ model[i]);
            if (dst[i] != y) h_die3("%s process lane %u: got %u expected %u", dec ? "PDD" : "PDE", i, dst[i], y);
            model[i] = dec ? save[i] : y;
          }
          for (i = nm; i < nm + 8; i++)
            if (dst[i] != (inplace ? save[i] : 0xEE)) h_die3("process wrote beyond n_ lanes");
        }
        h_cov_hit(C2_PROC);
        if (nm == 0) h_cov_hit(C2_PROC0);
        if (inplace && nm) h_cov_hit(C2_PROC_INPLACE);
        if (nm == cap && cap) h_cov_hit(C2_PROC_FULL);
        h_emit(0, "P n=%u y0=%u y1=%u", nm, nm > 0 ? dst[0] : 0u, nm > 1 ? dst[1] : 0u);
      }
      for (k = 0; k < cap; k++)
        if (((uint8_t *)o->state)[k] != model[k]) h_die3("lane %u of the state differs from the model (%u vs %u)", k, ((uint8_t *)o->state)[k], model[k]);
      h_dump_obj(h_sample(&ctr, 12, 2999), dec ? "PDD" : "PDE", o, f_par, 3);
    }
    h_check_not_wild("parallel");
    h_maybe_null_first_ptr(o, 8);
    if (dec) pdd_dtor(o); else pde_dtor(o);
    h_cov_hit(C2_DTOR);
    if (h_nfree != nf0 + 1) h_die3("dtor must free the block exactly once");
    h_trial_end(dec ? "pdd" : "pde");
  }
}

/* ======================================================================================== scenarios 4 / 5: Queue<float> */
enum { C4_CTOR = 0, C4_DTOR, C4_RESET, C4_W1_OK, C4_W1_FULL, C4_W1_WRAP, C4_WN_OK_NOWRAP, C4_WN_LANDS_END, C4_WN_WRAP, C4_WN_FAIL,
       C4_WN_ZERO, C4_WN_HUGE, C4_WN_FILL, C4_R_OK_NOWRAP, C4_R_LANDS_END, C4_R_WRAP, C4_R_FAIL, C4_R_ZERO, C4_R_HUGE, C4_R_DRAIN,
       C4_CAP0, C4_ARB, C4_BIG, C4_WN_WRAP1, C4_R_WRAP1, C4_EXH, C4_TAIL0, C4_TAILNEG, C4_HUGE16 };
#define QMAX 70016u
typedef struct { unsigned size, rd, wr; uint32_t store[QMAX]; } QModel;
static float g_qbuf[QMAX + 16], g_qdst[QMAX + 16];
static uint32_t g_qbits[QMAX + 16];
static unsigned qm_used(const QModel *m) { return (m->wr + m->size - m->rd) % m->size; }
static unsigned q_index(const QObj *o, const void *p) { return (unsigned)(((const uint8_t *)p - (const uint8_t *)o->buf) / 4); }
static void q_verify(const QObj *o, const QModel *m, int full, const char *what) {
  unsigned i;
  if (o->size != m->size || q_index(o, o->rd) != m->rd || q_index(o, o->wr) != m->wr)
    h_die3("queue %s: rd/wr/size are %u/%u/%u, model %u/%u/%u", what, q_index(o, o->rd), q_index(o, o->wr), o->size, m->rd, m->wr, m->size);
  if (full)
    for (i = 0; i < m->size; i++) {
      uint32_t b;
      memcpy(&b, (const uint8_t *)o->buf + 4u * i, 4);
      if (b != m->store[i]) h_die3("queue %s: cell %u holds %08x, model %08x", what, i, b, m->store[i]);
    }
}
/* one operation on queue + model; op 0 write(float), 1 write(float*,n), 2 read(float*,n), 3 reset */
static void q_do_op(QObj *o, QModel *m, int op, unsigned n, unsigned long *ctr, int showit) {
  float *buf = g_qbuf;
  uint32_t *bits = g_qbits;
  unsigned i, used = qm_used(m), room = m->size - used;
  int r, expect;
  if (op == 0 && m->wr >= m->size) h_die3("harness: write(float) must not be issued with wr_ one past the end (it would store out of the block)");
  if ((op == 1 && m->wr == m->size) || (op == 2 && m->rd == m->size)) h_cov_hit(C4_TAIL0);
  if (op == 0) {
    uint32_t v = h_rnd();
    float f;
    if (h_rpct(12)) v = (h_rnd() & 0x80000000u) | 0x7f800000u | (h_rnd() & 0x007fffffu); /* includes SIGNALLING NaNs: the blob moves the word */
    memcpy(&f, &v, 4); /* bits exactly (no x87 load here at -O2; if a compiler did quiet the argument the BLOB run would exit 3 first) */
    r = q_write1(o, f);
    expect = (room - 1 == 0) ? -1 : 0;
    if (expect == 0) {
      m->store[m->wr] = v;
      if (m->wr == m->size - 1) h_cov_hit(C4_W1_WRAP);
      m->wr = (m->wr + 1) % m->size;
      h_cov_hit(C4_W1_OK);
    } else {
      h_cov_hit(C4_W1_FULL);
    }
  } else if (op == 1) {
    for (i = 0; i < (n < QMAX ? n + 8 : 16u); i++) {
      bits[i] = h_rnd();
      if (h_rpct(12)) bits[i] = (h_rnd() & 0x80000000u) | 0x7f800000u | (h_rnd() & 0x007fffffu); /* Inf / quiet and SIGNALLING NaNs: bits must survive (the blob moves, never loads, them) */
      memcpy(&buf[i], &bits[i], 4); /* memcpy: no x87 load in the harness */
    }
    r = q_writeN(o, buf, n);
    expect = (room - 1 < n) ? -1 : 0;
    if (expect == 0) {
      int tail_i = (int)m->size - (int)m->wr;           /* may be 0 (wr_ == last_+1) or negative (wr_ beyond it) */
      unsigned done = tail_i > 0 ? (unsigned)tail_i : 0u;
      if (tail_i < (int)n) { /* wrap path: `tail` elements up to the end, the rest from the start of the block */
        for (i = 0; i < n; i++) m->store[i < done ? m->wr + i : i - done] = bits[i];
        m->wr = n - done;
        h_cov_hit(C4_WN_WRAP);
        if (done + 1 == n) h_cov_hit(C4_WN_WRAP1);
        if (tail_i < 0) h_cov_hit(C4_TAILNEG);
      } else {
        for (i = 0; i < n; i++) m->store[m->wr + i] = bits[i];
        m->wr += n;
        if (m->wr == m->size) m->wr = 0;
        if (n == 0) h_cov_hit(C4_WN_ZERO); else h_cov_hit(C4_WN_OK_NOWRAP);
        if (n && tail_i == (int)n) h_cov_hit(C4_WN_LANDS_END);
      }
      if (n == room - 1 && n) h_cov_hit(C4_WN_FILL);
    } else {
      h_cov_hit(C4_WN_FAIL);
      if (n == 0xffffffffu || n >= 0x80000000u) h_cov_hit(C4_WN_HUGE);
    }
  } else if (op == 2) {
    float *dst = g_qdst;
    memset(dst, 0xEE, (n < QMAX ? n + 8 : 16u) * 4u);
    r = q_read(o, dst, n);
    expect = (used < n) ? -1 : 0;
    if (expect == 0) {
      int tail_i = (int)m->size - (int)m->rd;
      unsigned done = tail_i > 0 ? (unsigned)tail_i : 0u;
      for (i = 0; i < n; i++) {
        uint32_t b = h_bits_of(dst[i]), want = m->store[(tail_i < (int)n) ? (i < done ? m->rd + i : i - done) : m->rd + i];
        if (b != want) h_die3("queue read element %u: %08x vs model %08x", i, b, want);
      }
      for (i = n; i < n + 8; i++) {
        uint32_t b = h_bits_of(dst[i]);
        if (b != 0xEEEEEEEEu) h_die3("queue read wrote beyond n");
      }
      if (tail_i < (int)n) {
        m->rd = n - done;
        h_cov_hit(C4_R_WRAP);
        if (done + 1 == n) h_cov_hit(C4_R_WRAP1);
        if (tail_i < 0) h_cov_hit(C4_TAILNEG);
      } else {
        m->rd += n;
        if (m->rd == m->size) m->rd = 0;
        if (n == 0) h_cov_hit(C4_R_ZERO); else h_cov_hit(C4_R_OK_NOWRAP);
        if (n && tail_i == (int)n) h_cov_hit(C4_R_LANDS_END);
      }
      if (n == used && n) h_cov_hit(C4_R_DRAIN);
    } else {
      h_cov_hit(C4_R_FAIL);
      if (n >= 0x80000000u) h_cov_hit(C4_R_HUGE);
      if (h_bits_of(dst[0]) != 0xEEEEEEEEu) h_die3("failed read wrote data");
    }
  } else {
    q_reset(o);
    m->rd = m->wr = 0;
    h_cov_hit(C4_RESET);
    r = expect = 0;
  }
  if (r != expect) h_die3("queue op %d n=%u (size %u rd %u wr %u): returned %d, expected %d", op, n, m->size, m->rd, m->wr, r, expect);
  q_verify(o, m, m->size <= 96 || (*ctr & 15) == 0, "op");
  h_emit(0, "Q op=%d n=%u r=%d rd=%u wr=%u", op, n, r, m->rd, m->wr);
  if (showit) h_dump_obj(1, "Q", o, f_q, 5);
}
static void q_trial(unsigned cap, int arbitrary, int nops, unsigned long *ctr) {
  QObj *o = (QObj *)h_obj_new(sizeof(QObj));
  QModel *m = (QModel *)malloc(sizeof(QModel));
  unsigned i;
  unsigned long nm0 = h_nmalloc, nf0 = h_nfree;
  int k;
  h_wild = 0;
  H_CTOR_PROBE(o, f_q, 5, sizeof(QObj), q_ctor(o, cap));
  h_cov_hit(C4_CTOR);
  if (h_nmalloc != nm0 + 1 || h_al[h_last_alloc].size != (cap + 1) * 4u) h_die3("queue ctor must allocate (cap+1)*4 bytes once");
  if (cap == 0) h_cov_hit(C4_CAP0);
  if (cap > 192) h_cov_hit(C4_BIG);
  if (cap > 32767) h_cov_hit(C4_HUGE16);
  if (o->size != cap + 1 || o->buf != h_al[h_last_alloc].p || (uint8_t *)o->last != (uint8_t *)o->buf + 4u * cap || o->rd != o->buf || o->wr != o->buf)
    h_die3("queue ctor fields");
  m->size = cap + 1;
  m->rd = m->wr = 0;
  for (i = 0; i < m->size; i++) memcpy(&m->store[i], (uint8_t *)o->buf + 4u * i, 4); /* unwritten cells: whatever the allocator poisoned */
  if (arbitrary) {
    /* rd_ / wr_ == last_ + 1 (one past the end, "tail == 0") or even last_ + 2 .. + 4 ("tail < 0"): the blob treats both as the wrap case */
    m->rd = h_rpct(25) ? m->size + h_rmod(5) : h_rmod(m->size);
    m->wr = h_rpct(25) ? m->size + h_rmod(5) : h_rmod(m->size);
    o->rd = (uint8_t *)o->buf + 4u * m->rd;
    o->wr = (uint8_t *)o->buf + 4u * m->wr;
    h_cov_hit(C4_ARB);
  }
  q_verify(o, m, 1, "ctor");
  h_dump_obj(h_sample(ctr, 20, 997), arbitrary ? "Q ctor/arb" : "Q ctor", o, f_q, 5);
  for (k = 0; k < nops; k++) {
    int op = (int)h_rmod(h_rpct(10) ? 4 : 3);
    unsigned used = qm_used(m), room = m->size - used, n;
    if (op == 0 && m->wr >= m->size) op = 1;
    int pick = (int)h_rmod(10);
    if (op == 0) n = 0;
    else if (op == 1) n = pick == 0 ? 0 : pick == 1 ? room - 1 : pick == 2 ? room : pick == 3 ? 0xffffffffu : pick == 4 ? 0x80000000u + h_rmod(5) : (unsigned)h_rrange(0, (int)(room + 2));
    else if (op == 2) n = pick == 0 ? 0 : pick == 1 ? used : pick == 2 ? used + 1 : pick == 3 ? 0xffffffffu : pick == 4 ? 0x80000000u + h_rmod(5) : (unsigned)h_rrange(0, (int)(used + 2));
    else n = 0;
    if (n >= QMAX && n < 0x80000000u) n = 0;
    if (cap >= 65536 && k < 2 && !arbitrary) { op = k == 0 ? 1 : 2; n = k == 0 ? room - 1 : used; } /* one transfer of > 65535 elements (16-bit counter traps) */
    q_do_op(o, m, op, n, ctr, 0);
  }
  q_verify(o, m, 1, "final");
  h_dump_obj(h_sample(ctr, 20, 997), "Q end", o, f_q, 5);
  h_check_not_wild("queue");
  h_maybe_null_first_ptr(o, 8);
  q_dtor(o);
  h_cov_hit(C4_DTOR);
  if (h_nfree != nf0 + 1) h_die3("queue dtor must free once");
  free(m);
  h_trial_end("queue");
}
static void sc_queue_random(void) {
  static const unsigned caps[] = { 0, 1, 2, 3, 4, 5, 7, 8, 15, 16, 31, 32, 63, 100, 191, 192, 193, 500, 1000, 4000, 32766, 32767, 32768, 40000, 65535, 70000 };
  unsigned long ctr = 0;
  unsigned t;
  h_cov_define(C4_CTOR, "ctor"); h_cov_define(C4_DTOR, "dtor"); h_cov_define(C4_RESET, "reset()");
  h_cov_define(C4_W1_OK, "write(float) ok"); h_cov_define(C4_W1_FULL, "write(float) full"); h_cov_define(C4_W1_WRAP, "write(float) at last_");
  h_cov_define(C4_WN_OK_NOWRAP, "write(n) no wrap"); h_cov_define(C4_WN_LANDS_END, "write(n) lands on last_+1");
  h_cov_define(C4_WN_WRAP, "write(n) wraps"); h_cov_define(C4_WN_WRAP1, "write(n) wraps by one"); h_cov_define(C4_WN_FAIL, "write(n) fails");
  h_cov_define(C4_WN_ZERO, "write(0)"); h_cov_define(C4_WN_HUGE, "write(n >= 2^31)"); h_cov_define(C4_WN_FILL, "write(n) fills to full");
  h_cov_define(C4_R_OK_NOWRAP, "read(n) no wrap"); h_cov_define(C4_R_LANDS_END, "read(n) lands on last_+1");
  h_cov_define(C4_R_WRAP, "read(n) wraps"); h_cov_define(C4_R_WRAP1, "read(n) wraps by one"); h_cov_define(C4_R_FAIL, "read(n) fails");
  h_cov_define(C4_R_ZERO, "read(0)"); h_cov_define(C4_R_HUGE, "read(n >= 2^31)"); h_cov_define(C4_R_DRAIN, "read(n) drains");
  h_cov_define(C4_CAP0, "capacity 0 (size 1)"); h_cov_define(C4_ARB, "arbitrary rd/wr state"); h_cov_define(C4_BIG, "size > 192");
  h_cov_define(C4_TAIL0, "n-op with rd_/wr_ one past the end");
  h_cov_define(C4_TAILNEG, "n-op with rd_/wr_ beyond last_+1");
  h_cov_define(C4_HUGE16, "capacity > 32767 (16-bit index traps)");
  h_scenario_begin(1);
  for (t = 0; t < 2400; t++) {
    unsigned cap = t < sizeof caps / sizeof caps[0] ? caps[t] : (unsigned)h_rrange(0, 40);
    q_trial(cap, t % 3 == 2, cap > 500 ? (cap > 5000 ? 60 : 400) : 120, &ctr);
  }
  h_emit(1, "queue random done");
}
/* exhaustive: every (size <= 6, rd, wr) state x every single operation incl. every n up to size+2, plus all 3-op sequences from a small alphabet */
static void sc_queue_exh(void) {
  unsigned long ctr = 0;
  unsigned cap, r, w, k, a, b, c;
  h_cov_define(C4_CTOR, "ctor"); h_cov_define(C4_DTOR, "dtor"); h_cov_define(C4_W1_OK, "write(float) ok"); h_cov_define(C4_W1_FULL, "write(float) full");
  h_cov_define(C4_W1_WRAP, "write(float) at last_");
  h_cov_define(C4_WN_OK_NOWRAP, "write(n) no wrap"); h_cov_define(C4_WN_LANDS_END, "write(n) lands on last_+1"); h_cov_define(C4_WN_WRAP, "write(n) wraps");
  h_cov_define(C4_WN_WRAP1, "write(n) wraps by one"); h_cov_define(C4_WN_FAIL, "write(n) fails"); h_cov_define(C4_WN_ZERO, "write(0)");
  h_cov_define(C4_R_OK_NOWRAP, "read(n) no wrap"); h_cov_define(C4_R_LANDS_END, "read(n) lands on last_+1"); h_cov_define(C4_R_WRAP, "read(n) wraps");
  h_cov_define(C4_R_WRAP1, "read(n) wraps by one"); h_cov_define(C4_R_FAIL, "read(n) fails"); h_cov_define(C4_R_ZERO, "read(0)");
  h_cov_define(C4_CAP0, "capacity 0 (size 1)"); h_cov_define(C4_ARB, "arbitrary rd/wr state"); h_cov_define(C4_EXH, "exhaustive (state,op) pairs");
  h_cov_define(C4_RESET, "reset()");
  h_cov_define(C4_TAIL0, "n-op with rd_/wr_ one past the end");
  h_cov_define(C4_TAILNEG, "n-op with rd_/wr_ beyond last_+1");
  h_scenario_begin(1);
  for (cap = 0; cap <= 5; cap++) {
    unsigned size = cap + 1;
    for (r = 0; r <= size + 2; r++)       /* rd_ / wr_ in 0 .. size+2: size == last_+1, beyond that last_+2 .. */
      for (w = 0; w <= size + 2; w++)
        for (k = 0; k < 2 + 2 * (size + 3); k++) { /* op index: 0 write1, 1 reset, then writeN n=0..size+2, readN n=0..size+2 */
          QObj *o;
          if (k == 0 && w >= size) continue;   /* write(float) at last_+1 or beyond would store outside the block */
          o = (QObj *)h_obj_new(sizeof(QObj));
          QModel *m = (QModel *)malloc(sizeof(QModel));
          unsigned i;
          H_CTOR_PROBE(o, f_q, 5, sizeof(QObj), q_ctor(o, cap));
          h_cov_hit(C4_CTOR);
          if (cap == 0) h_cov_hit(C4_CAP0);
          h_cov_hit(C4_ARB);
          m->size = size;
          for (i = 0; i < size; i++) memcpy(&m->store[i], (uint8_t *)o->buf + 4u * i, 4);
          m->rd = r; m->wr = w;
          o->rd = (uint8_t *)o->buf + 4u * r;
          o->wr = (uint8_t *)o->buf + 4u * w;
          if (k == 0) q_do_op(o, m, 0, 0, &ctr, 0);
          else if (k == 1) q_do_op(o, m, 3, 0, &ctr, 0);
          else if (k < 2 + size + 3) q_do_op(o, m, 1, k - 2, &ctr, 0);
          else q_do_op(o, m, 2, k - 2 - (size + 3), &ctr, 0);
          h_cov_hit(C4_EXH);
          h_dump_obj(h_sample(&ctr, 6, 997), "QX", o, f_q, 5);
          h_maybe_null_first_ptr(o, 8);
          q_dtor(o);
          h_cov_hit(C4_DTOR);
          free(m);
          h_trial_end("queue exh");
        }
  }
  /* all sequences of length 4 over {write1, writeN(2), readN(2), writeN(3), readN(1)} from the reset state, caps 1..4 */
  for (cap = 1; cap <= 4; cap++) {
    unsigned size = cap + 1;
    for (a = 0; a < 5 * 5 * 5 * 5; a++) {
      QObj *o = (QObj *)h_obj_new(sizeof(QObj));
      QModel *m = (QModel *)malloc(sizeof(QModel));
      unsigned i, code = a;
      H_CTOR_PROBE(o, f_q, 5, sizeof(QObj), q_ctor(o, cap));
      h_cov_hit(C4_CTOR);
      m->size = size; m->rd = m->wr = 0;
      for (i = 0; i < size; i++) memcpy(&m->store[i], (uint8_t *)o->buf + 4u * i, 4);
      for (b = 0; b < 4; b++, code /= 5) {
        c = code % 5;
        if (c == 0) q_do_op(o, m, 0, 0, &ctr, 0);
        else if (c == 1) q_do_op(o, m, 1, 2, &ctr, 0);
        else if (c == 2) q_do_op(o, m, 2, 2, &ctr, 0);
        else if (c == 3) q_do_op(o, m, 1, 3, &ctr, 0);
        else q_do_op(o, m, 2, 1, &ctr, 0);
      }
      h_dump_obj(h_sample(&ctr, 0, 211), "QS", o, f_q, 5);
      h_maybe_null_first_ptr(o, 8);
      q_dtor(o);
      h_cov_hit(C4_DTOR);
      free(m);
      h_trial_end("queue seq");
    }
  }
  h_emit(1, "queue exhaustive done");
}

/* =================================================================== scenarios 6..11: Scrambler / Descrambler variants */
typedef struct {
  const char *name;
  int desc;  /* 0 Scrambler, 1 Descrambler */
  int esz;   /* sizeof(H): 1 (uchar history) or 4 (int history) */
  int oint;  /* O is int (process(h) returns int / array output is int[]) instead of uchar */
  int hasP1, hasPN, hasAll;
  void (*ctor)(void *, unsigned, unsigned, unsigned);
  void (*dtor)(void *);
  void (*reset)(void *, uint32_t);
  int (*p1)(void *, uint32_t);
  void (*pn)(void *, const void *, void *, unsigned);
  void (*ones)(void *, void *, unsigned);
  void (*zeros)(void *, void *, unsigned);
  void (*rhi)(void *);
  void (*cht)(void *);
} VOps;
static int w_shh_p1(void *o, uint32_t v) { return shh_p1(o, v); }
static int w_shi_p1(void *o, uint32_t v) { return shi_p1(o, v); }
static int w_dhi_p1(void *o, uint32_t v) { return dhi_p1(o, v); }
static int w_dii_p1(void *o, uint32_t v) { return dii_p1(o, (int)v); }
static const VOps V_SHH = { "Scrambler<uchar,uchar>", 0, 1, 0, 1, 1, 1, shh_ctor, shh_dtor, shh_reset, w_shh_p1, shh_pn, shh_ones, shh_zeros, shh_rhi, shh_cht };
static const VOps V_SIH = { "Scrambler<int,uchar>", 0, 4, 0, 0, 1, 0, sih_ctor, sih_dtor, sih_reset, NULL, sih_pn, NULL, NULL, sih_rhi, sih_cht };
static const VOps V_SHI = { "Scrambler<uchar,int>", 0, 1, 1, 1, 0, 0, shi_ctor, shi_dtor, shi_reset, w_shi_p1, NULL, NULL, NULL, shi_rhi, shi_cht };
static const VOps V_DHI = { "Descrambler<uchar,int>", 1, 1, 1, 1, 1, 0, dhi_ctor, dhi_dtor, dhi_reset, w_dhi_p1, dhi_pn, NULL, NULL, dhi_rhi, dhi_cht };
static const VOps V_DII = { "Descrambler<int,int>", 1, 4, 1, 1, 0, 0, dii_ctor, dii_dtor, dii_reset, w_dii_p1, NULL, NULL, NULL, dii_rhi, dii_cht };

/* the delay-line model: hist[k-1] = symbol of k steps ago (k = 1 .. p2), no pointers, no wrap logic */
typedef struct { unsigned p1, p2; uint32_t hist[400]; } SM;
static void sm_reset(SM *m, uint32_t v) { unsigned k; for (k = 0; k < m->p2; k++) m->hist[k] = v & 1u; }
static void sm_push(SM *m, uint32_t x) {
  if (m->p2) { memmove(m->hist + 1, m->hist, (m->p2 - 1) * sizeof(uint32_t)); m->hist[0] = x; }
}
/* scrambler: y = in ^ y[n-p1] ^ y[n-p2] (low byte), stored zero-extended; mode 0 normal, 1 constant-1 input, 2 constant-0 input */
static uint32_t sm_scr(SM *m, uint32_t in, int mode) {
  uint32_t h1 = m->hist[m->p1 - 1], h2 = m->hist[m->p2 - 1];
  uint32_t y = (uint8_t)(h1 ^ (mode == 0 ? in : mode == 1 ? 1u : 0u) ^ h2);
  sm_push(m, y);
  return y;
}
/* descrambler: out = x ^ x[n-p1] ^ x[n-p2], x = the input as an H (the new symbol is visible to a tap of 0) */
static uint32_t sm_desc(SM *m, uint32_t in, int esz) {
  uint32_t x = esz == 1 ? (in & 0xffu) : in;
  uint32_t w1 = m->p1 == 0 ? x : m->hist[m->p1 - 1], w2 = m->p2 == 0 ? x : m->hist[m->p2 - 1];
  uint32_t out = x ^ w1 ^ w2;
  sm_push(m, x);
  return out;
}

enum { CV_CTOR = 0, CV_DTOR, CV_RESET0, CV_RESET1, CV_RESETHI, CV_P1, CV_PN, CV_PN0, CV_ONES, CV_ZEROS, CV_WRAP, CV_WRAP_P1, CV_WRAP_PN,
       CV_SLACK_MIN, CV_SLACK_BIG, CV_P1EQP2, CV_P1IS1, CV_P1IS0, CV_P2IS0, CV_MANYWRAPS, CV_RHI, CV_CHT, CV_ARBSTART, CV_SMEAR, CV_OOB_TAP,
       CV_P1_LT_P2 };
static unsigned sidx(const ScrObj *o, int esz, const void *p) { return (unsigned)(((const uint8_t *)p - (const uint8_t *)o->buf) / (size_t)esz); }
static uint32_t scell(const ScrObj *o, int esz, unsigned idx) {
  return esz == 1 ? ((const uint8_t *)o->buf)[idx] : (uint32_t)((const int32_t *)o->buf)[idx];
}
/* structural + model check of a (modelled) object: pointers relate as designed, the window of cells equals the model's history */
static void sv_verify(const ScrObj *o, const VOps *v, const SM *m, unsigned slack, const char *what) {
  unsigned esz = (unsigned)v->esz, k, wr = sidx(o, v->esz, o->wr);
  if (wr > slack) h_die3("%s %s: wr_ index %u beyond head0 (%u)", v->name, what, wr, slack);
  if (sidx(o, v->esz, o->t1) != wr + m->p1 || sidx(o, v->esz, o->t2) != wr + m->p2)
    h_die3("%s %s: taps are at %u/%u, expected %u/%u", v->name, what, sidx(o, v->esz, o->t1), sidx(o, v->esz, o->t2), wr + m->p1, wr + m->p2);
  if (sidx(o, v->esz, o->head0) != slack || sidx(o, v->esz, o->end) != slack + m->p2 || sidx(o, v->esz, o->t1_0) != slack + m->p1 || o->tail != m->p2)
    h_die3("%s %s: constant fields changed", v->name, what);
  for (k = 1; k <= m->p2; k++)
    if (scell(o, v->esz, wr + k) != m->hist[k - 1])
      h_die3("%s %s: history cell wr+%u holds %u, model %u (esz %u)", v->name, what, k, scell(o, v->esz, wr + k), m->hist[k - 1], esz);
}
static void vcheck_alloc(const ScrObj *o, const VOps *v, unsigned p1, unsigned p2, unsigned slack, unsigned long nm0) {
  const uint8_t *b = (const uint8_t *)o->buf;
  size_t es = (size_t)v->esz;
  if (h_nmalloc != nm0 + 1 || h_al[h_last_alloc].size != (p2 + slack + 1) * es) h_die3("%s ctor must allocate (p2+slack+1)*%u bytes once", v->name, v->esz);
  if (b != h_al[h_last_alloc].p || (const uint8_t *)o->head0 != b + slack * es || (const uint8_t *)o->t1_0 != b + (slack + p1) * es ||
      (const uint8_t *)o->end != b + (slack + p2) * es || o->wr != o->head0 || (const uint8_t *)o->t1 != (const uint8_t *)o->t1_0 || o->t2 != o->end || o->tail != p2)
    h_die3("%s ctor fields (p1 %u p2 %u slack %u)", v->name, p1, p2, slack);
}
static void v_action(const VOps *v, ScrObj *o, SM *m, unsigned slack, int modelled, unsigned long *ctr, int act, int stat_only) {
  unsigned i, n, wr0 = sidx(o, v->esz, o->wr);
  unsigned wraps = 0;
  (void)stat_only;
  if (act == 0) { /* process(h) x m */
    unsigned cnt = (unsigned)h_rrange(1, 40);
    uint32_t acc = 0;
    for (i = 0; i < cnt; i++) {
      uint32_t in = h_rnd(), got, exp = 0;
      unsigned before = sidx(o, v->esz, o->wr);
      uint32_t arg = v->esz == 1 ? h_garbage_hi(in & 0xffu) : in;
      got = (uint32_t)v->p1(o, arg);
      h_cov_hit(CV_P1);
      if (before == 0) { wraps++; h_cov_hit(CV_WRAP_P1); }
      if (modelled) {
        exp = v->desc ? sm_desc(m, in, v->esz) : sm_scr(m, in & 0xffu, 0);
        if (got != (v->oint || v->esz == 4 ? exp : (exp & 0xffu))) h_die3("%s process(%u): got %u model %u", v->name, in, got, exp);
      }
      acc = acc * 31u + got;
    }
    h_emit(0, "P1 x%u acc=%u", cnt, acc);
  } else if (act == 1) { /* process(const H*, O*, n) */
    uint8_t in8[96];
    uint32_t in32[96], out32[96];
    uint8_t out8[96];
    n = h_rpct(10) ? 0u : (unsigned)h_rrange(0, 70);
    for (i = 0; i < n + 8; i++) { in8[i] = (uint8_t)h_rnd(); in32[i] = h_rnd(); out8[i] = 0xEE; out32[i] = 0xEEEEEEEEu; }
    if (v->esz == 1 ? 0 : 1) { /* int input: sometimes small values */
      if (h_rpct(30)) for (i = 0; i < n; i++) in32[i] &= 0xffu;
    }
    v->pn(o, v->esz == 1 ? (const void *)in8 : (const void *)in32, v->oint ? (void *)out32 : (void *)out8, n);
    h_cov_hit(CV_PN);
    if (n == 0) h_cov_hit(CV_PN0);
    for (i = 0; i < n; i++) {
      uint32_t got = v->oint ? out32[i] : out8[i];
      if (modelled) {
        uint32_t in = v->esz == 1 ? in8[i] : in32[i];
        uint32_t exp = v->desc ? sm_desc(m, in, v->esz) : sm_scr(m, v->esz == 1 ? in : in, 0);
        if (got != (v->oint ? exp : (exp & 0xffu))) h_die3("%s process[%u of %u]: got %u model %u", v->name, i, n, got, exp);
      }
    }
    for (i = n; i < n + 8; i++)
      if ((v->oint ? out32[i] : out8[i]) != (v->oint ? 0xEEEEEEEEu : 0xEEu)) h_die3("%s process wrote beyond n", v->name);
    {
      u64 h = 1469598103934665603ULL;
      for (i = 0; i < n; i++) { h ^= v->oint ? out32[i] : out8[i]; h *= 1099511628211ULL; }
      h_emit(0, "PN n=%u h=%016llx", n, h);
    }
    if (n > wr0) { wraps++; h_cov_hit(CV_WRAP_PN); } /* wr_ walks down from wr0: the (wr0+1)-th step wraps */
  } else if (act == 2 || act == 3) { /* processAllOnes / processAllZeros (uchar out) */
    uint8_t out8[96];
    n = (unsigned)h_rrange(0, 70);
    memset(out8, 0xEE, sizeof out8);
    if (act == 2) v->ones(o, out8, n); else v->zeros(o, out8, n);
    h_cov_hit(act == 2 ? CV_ONES : CV_ZEROS);
    for (i = 0; i < n; i++)
      if (modelled) {
        uint32_t exp = sm_scr(m, 0, act == 2 ? 1 : 2);
        if (out8[i] != exp) h_die3("%s processAll%s[%u]: got %u model %u", v->name, act == 2 ? "Ones" : "Zeros", i, out8[i], exp);
      }
    for (i = n; i < n + 8; i++) if (out8[i] != 0xEE) h_die3("%s processAll wrote beyond n", v->name);
    { u64 h = 1469598103934665603ULL; for (i = 0; i < n; i++) { h ^= out8[i]; h *= 1099511628211ULL; } h_emit(0, "ALL%d n=%u h=%016llx", act, n, h); }
    if (n > wr0) h_cov_hit(CV_WRAP);
  } else { /* reset(v) */
    static const uint32_t vals[] = { 0, 1, 2, 3, 0xfe, 0xff, 0x80, 0x100, 0x101, 0xffffffffu, 0x7fffffffu, 0x80000000u };
    uint32_t val = h_rpct(60) ? vals[h_rmod(sizeof vals / sizeof vals[0])] : h_rnd();
    uint32_t arg = v->esz == 1 ? h_garbage_hi(val & 0xffu) : val;
    v->reset(o, arg);
    if (modelled) sm_reset(m, (v->esz == 1 ? val & 0xffu : val) & 1u);
    h_cov_hit((val & 1) ? CV_RESET1 : CV_RESET0);
    if (arg & ~1u) h_cov_hit(CV_RESETHI);
    h_emit(0, "RESET %u", val);
  }
  if (wraps) h_cov_hit(CV_WRAP);
  if (modelled) sv_verify(o, v, m, slack, "after action");
  h_dump_obj(h_sample(ctr, 8, 1499), v->name, o, f_scr, 8);
  h_check_not_wild(v->name);
}
/* one trial: ctor, a random mixture of actions, dtor */
static void v_trial(const VOps *v, unsigned p1, unsigned p2, unsigned slack, int modelled, int arbstart, unsigned long *ctr) {
  ScrObj *o = (ScrObj *)h_obj_new(sizeof(ScrObj));
  SM m;
  unsigned i, nact = (unsigned)h_rrange(10, 40);
  unsigned long nm0 = h_nmalloc, nf0 = h_nfree;
  unsigned lo = v->desc ? 0u : 1u;
  (void)lo;
  h_wild = 0;
  m.p1 = p1;
  m.p2 = p2;
  memset(m.hist, 0, sizeof m.hist);
  H_CTOR_PROBE(o, f_scr, 8, sizeof(ScrObj), v->ctor(o, p1, p2, slack));
  h_cov_hit(CV_CTOR);
  vcheck_alloc(o, v, p1, p2, slack, nm0);
  sm_reset(&m, 0);
  if (modelled) sv_verify(o, v, &m, slack, "after ctor");
  else { /* unmodelled configs still must have initialised every cell the ctor's reset(0) covers */
    unsigned k;
    for (k = 1; k <= p2; k++) if (scell(o, v->esz, slack + k) != 0) h_die3("%s ctor: history cell %u not cleared", v->name, k);
  }
  if (p2 == 1) { /* nothing: kept for readability of coverage */ }
  if (slack + 1 == p2) h_cov_hit(CV_SLACK_MIN);
  if (slack > p2 + 8) h_cov_hit(CV_SLACK_BIG);
  if (p1 == p2) h_cov_hit(CV_P1EQP2);
  if (p1 == 1) h_cov_hit(CV_P1IS1);
  if (p1 == 0) h_cov_hit(CV_P1IS0);
  if (p2 == 0) h_cov_hit(CV_P2IS0);
  if (p1 < p2) h_cov_hit(CV_P1_LT_P2);
  if (p1 > p2) h_cov_hit(CV_OOB_TAP);
  if (slack + 1 < p2) h_cov_hit(CV_SMEAR);
  h_dump_obj(h_sample(ctr, 8, 1499), v->name, o, f_scr, 8);
  if (arbstart) { /* start from an arbitrary valid position (as reached after some wraps), cells keep their contents */
    unsigned s = h_rmod(slack + 1);
    o->wr = (uint8_t *)o->buf + (size_t)s * (size_t)v->esz;
    o->t1 = (uint8_t *)o->wr + (size_t)p1 * (size_t)v->esz;
    o->t2 = (uint8_t *)o->wr + (size_t)p2 * (size_t)v->esz;
    h_cov_hit(CV_ARBSTART);
    h_dump_obj(h_sample(ctr, 8, 1499), v->name, o, f_scr, 8);
  }
  for (i = 0; i < nact; i++) {
    int act;
    unsigned pick = h_rmod(100);
    if (!modelled && pick < 6) { /* direct calls of the two helpers (unmodelled only: they break the delay-line invariant) */
      if (pick < 3) { v->rhi(o); h_cov_hit(CV_RHI); } else { v->cht(o); h_cov_hit(CV_CHT); }
      h_emit(0, "HELPER %u", pick);
      h_dump_obj(h_sample(ctr, 8, 1499), v->name, o, f_scr, 8);
      continue;
    }
    if (pick < 40 && v->hasP1) act = 0;
    else if (pick < 75 && v->hasPN) act = 1;
    else if (pick < 85 && v->hasAll) act = 2 + (int)h_rmod(2);
    else if (pick < 95) act = 4;
    else act = v->hasP1 ? 0 : v->hasPN ? 1 : 4;
    if (act == 1 && !v->hasPN) act = v->hasP1 ? 0 : 4;
    v_action(v, o, &m, slack, modelled, ctr, act, 0);
  }
  if (modelled) {
    unsigned steps = slack < 100 ? 1u : 0u, s;
    if (steps) h_cov_hit(CV_MANYWRAPS);
    /* run the model past many wraps without per-action overhead */
    for (s = 0; s < 4 * (slack + 1) + 9 && s < 500; s++) {
      unsigned wrb = sidx(o, v->esz, o->wr);
      if (v->hasP1) {
        uint32_t in = h_rnd(), arg = v->esz == 1 ? h_garbage_hi(in & 0xffu) : in, got = (uint32_t)v->p1(o, arg), exp;
        exp = v->desc ? sm_desc(&m, in, v->esz) : sm_scr(&m, in & 0xffu, 0);
        if (got != (v->oint || v->esz == 4 ? exp : (exp & 0xffu))) h_die3("%s long-run process mismatch", v->name);
        h_cov_hit(CV_P1);
        if (wrb == 0) h_cov_hit(CV_WRAP);
      } else {
        uint32_t in32[1], out32[1];
        uint8_t out8[1], in8[1];
        uint32_t exp, got;
        in32[0] = h_rnd();
        in8[0] = (uint8_t)in32[0];
        v->pn(o, v->esz == 1 ? (const void *)in8 : (const void *)in32, v->oint ? (void *)out32 : (void *)out8, 1);
        exp = v->desc ? sm_desc(&m, v->esz == 1 ? in8[0] : in32[0], v->esz) : sm_scr(&m, v->esz == 1 ? in8[0] : in32[0], 0);
        got = v->oint ? out32[0] : out8[0];
        if (got != (v->oint ? exp : (exp & 0xffu))) h_die3("%s long-run block mismatch", v->name);
        h_cov_hit(CV_PN);
        if (wrb == 0) h_cov_hit(CV_WRAP);
      }
    }
    sv_verify(o, v, &m, slack, "after long run");
  }
  h_dump_obj(h_sample(ctr, 8, 1499), v->name, o, f_scr, 8);
  h_check_not_wild(v->name);
  h_maybe_null_first_ptr(o, 8);
  v->dtor(o);
  h_cov_hit(CV_DTOR);
  if (h_nfree != nf0 + 1) h_die3("%s dtor must free once", v->name);
  h_trial_end(v->name);
}
static void v_cov_setup(const VOps *v, int modelled_only) {
  h_cov_define(CV_CTOR, "ctor"); h_cov_define(CV_DTOR, "dtor");
  h_cov_define(CV_RESET0, "reset(v & 1 == 0)"); h_cov_define(CV_RESET1, "reset(v & 1 == 1)");
  h_cov_define(CV_RESETHI, "reset(v with bits above bit 0 / garbage)");
  if (v->hasP1) h_cov_define(CV_P1, "process(h)"); else h_cov_define(CV_P1, "process(h) [n/a]");
  if (v->hasPN) h_cov_define(CV_PN, "process(array)"); else h_cov_define(CV_PN, "process(array) [n/a]");
  h_cov_define(CV_PN0, "process(array, n == 0)");
  h_cov_define(CV_ONES, "processAllOnes"); h_cov_define(CV_ZEROS, "processAllZeros");
  h_cov_define(CV_WRAP, "wrap (copyHistoryTail path)"); h_cov_define(CV_WRAP_P1, "wrap inside process(h)"); h_cov_define(CV_WRAP_PN, "wrap inside process(array)");
  h_cov_define(CV_SLACK_MIN, "config slack+1 == p2"); h_cov_define(CV_SLACK_BIG, "config slack > p2+8");
  h_cov_define(CV_P1EQP2, "config p1 == p2"); h_cov_define(CV_P1IS1, "config p1 == 1"); h_cov_define(CV_P1IS0, "config p1 == 0");
  h_cov_define(CV_P2IS0, "config p2 == 0"); h_cov_define(CV_MANYWRAPS, "long run past many wraps");
  h_cov_define(CV_RHI, "direct resetHistoryIndexes"); h_cov_define(CV_CHT, "direct copyHistoryTail");
  h_cov_define(CV_ARBSTART, "arbitrary start position"); h_cov_define(CV_SMEAR, "config slack+1 < p2 (overlapping tail copy)");
  h_cov_define(CV_OOB_TAP, "config p1 > p2 (tap beyond the history)"); h_cov_define(CV_P1_LT_P2, "config p1 < p2");
  if (!v->hasP1) { h_covreq[CV_P1] = 0; h_covreq[CV_WRAP_P1] = 0; }
  if (!v->hasPN) { h_covreq[CV_PN] = 0; h_covreq[CV_PN0] = 0; h_covreq[CV_WRAP_PN] = 0; }
  if (!v->hasAll) { h_covreq[CV_ONES] = 0; h_covreq[CV_ZEROS] = 0; }
  if (!v->desc) { h_covreq[CV_P1IS0] = 0; h_covreq[CV_P2IS0] = 0; }
  if (modelled_only) {
    h_covreq[CV_RHI] = h_covreq[CV_CHT] = 0; h_covreq[CV_SMEAR] = 0; h_covreq[CV_OOB_TAP] = 0; h_covreq[CV_ARBSTART] = 0;
    h_covreq[CV_P1IS0] = h_covreq[CV_P2IS0] = 0;
  } else {
    h_covreq[CV_MANYWRAPS] = 0;
  }
}
static void sc_variant_modelled(const VOps *v) {
  static const unsigned fixed[][3] = { { 1, 1, 0 }, { 1, 1, 1 }, { 1, 2, 1 }, { 2, 2, 1 }, { 1, 3, 2 }, { 1, 3, 5 }, { 3, 3, 2 }, { 18, 23, 22 },
                                       { 18, 23, 23 }, { 5, 23, 40 }, { 1, 23, 22 }, { 23, 23, 22 }, { 4, 4, 3 }, { 10, 64, 63 }, { 1, 64, 200 },
                                       { 0, 0, 0 }, { 0, 1, 0 }, { 0, 5, 4 }, { 0, 23, 22 }, { 0, 64, 70 } };
  unsigned long ctr = 0;
  unsigned t, nf = (unsigned)(sizeof fixed / sizeof fixed[0]);
  v_cov_setup(v, 1);
  h_scenario_begin(v->esz == 4);
  /* P1IS0 / P2IS0 are legal for descramblers only */
  if (v->desc) { h_covreq[CV_P1IS0] = 1; h_covreq[CV_P2IS0] = 1; }
  for (t = 0; t < nf; t++) {
    unsigned p1 = fixed[t][0], p2 = fixed[t][1], slack = fixed[t][2];
    if (!v->desc && p1 == 0) continue;
    if (!v->desc && p2 == 0) continue;
    if (slack + 1 < p2) h_die3("fixed config table violates slack + 1 >= p2");
    v_trial(v, p1, p2, slack, 1, 0, &ctr);
    v_trial(v, p1, p2, slack, 1, 0, &ctr);
    v_trial(v, p1, p2, slack, 1, 0, &ctr);
    v_trial(v, p1, p2, slack, 1, 0, &ctr);
  }
  for (t = 0; t < 600; t++) {
    unsigned p2 = (unsigned)h_rrange(1, 64), p1 = (unsigned)h_rrange(v->desc ? 0 : 1, (int)p2), slack = p2 - 1 + h_rmod(40);
    v_trial(v, p1, p2, slack, 1, 0, &ctr);
  }
  h_emit(1, "%s modelled trials done", v->name);
}
/* unmodelled: blob-vs-port only, covers the overlapping tail copy, tap 0 / beyond history, arbitrary start positions, direct helper calls */
static void sc_variant_odd(void) {
  static const VOps *all[] = { &V_SHH, &V_SIH, &V_SHI, &V_DHI, &V_DII };
  unsigned long ctr = 0;
  unsigned vi, t;
  /* union coverage over the five variants: required classes are defined once, hit by any */
  v_cov_setup(&V_SHH, 0);
  h_covreq[CV_P1IS0] = h_covreq[CV_P2IS0] = 1;
  h_scenario_begin(1);
  for (vi = 0; vi < 5; vi++)
    for (t = 0; t < 1000; t++) {
      unsigned p2 = (unsigned)h_rrange(0, 30), p1 = (unsigned)h_rrange(0, (int)p2 + (h_rpct(15) ? 4 : 0)), slack = (unsigned)h_rrange(0, (int)p2 + 10);
      if (t < 40) { p2 = (unsigned)h_rrange(3, 20); p1 = (unsigned)h_rrange(1, (int)p2); slack = (unsigned)h_rrange(0, (int)p2 - 2); } /* overlapping copy */
      if (t == 40) { p1 = 0; p2 = 0; slack = 0; }
      if (t == 41) { p1 = 0; p2 = 5; slack = 6; }
      if (all[vi]->desc == 0 && p2 == 0) { p2 = 1; if (p1 > 1) p1 = 1; }
      v_trial(all[vi], p1, p2, slack, 0, t % 2 == 1, &ctr);
    }
  h_emit(1, "odd configs done");
}

/* ---------------------------------------------------------------------------------------------- layout cross-check */
static void layout_check(void) {
  unsigned k = 0, i;
  static const size_t soff[8] = { offsetof(ScrObj, buf), offsetof(ScrObj, head0), offsetof(ScrObj, t1_0), offsetof(ScrObj, end),
                                  offsetof(ScrObj, wr), offsetof(ScrObj, t1), offsetof(ScrObj, t2), offsetof(ScrObj, tail) };
  const unsigned *p = p01_layout_probe;
  if (p01_layout_probe_count != 65) h_die3("layout probe has %u entries, expected 65", p01_layout_probe_count);
  for (i = 0; i < 5; i++) { /* Scrambler hh ih hi, Descrambler hi ii */
    unsigned j;
    if (p[k++] != sizeof(ScrObj)) h_die3("sizeof the Scrambler/Descrambler class (%u) differs from the harness mirror (%zu)", p[k - 1], sizeof(ScrObj));
    for (j = 0; j < 8; j++) if (p[k++] != soff[j]) h_die3("Scrambler/Descrambler member %u offset differs from the mirror", j);
  }
  for (i = 0; i < 2; i++) { /* PDE, PDD */
    if (p[k++] != sizeof(ParObj) || p[k++] != offsetof(ParObj, state) || p[k++] != offsetof(ParObj, cap) || p[k++] != offsetof(ParObj, n))
      h_die3("ParallelDifferential layout differs from the mirror");
  }
  if (p[k++] != sizeof(QObj) || p[k++] != offsetof(QObj, buf) || p[k++] != offsetof(QObj, last) || p[k++] != offsetof(QObj, rd) ||
      p[k++] != offsetof(QObj, wr) || p[k++] != offsetof(QObj, size))
    h_die3("Queue layout differs from the mirror");
  if (p[k++] != sizeof(SerH) || p[k++] != 0 || p[k++] != sizeof(SerH) || p[k++] != 0 || p[k++] != sizeof(SerI) || p[k++] != 0)
    h_die3("SerialDifferential layout differs from the mirror");
  if (k != 65) h_die3("layout probe walk ended at %u", k);
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P01_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  layout_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_serial_h(); break;
    case 1: sc_serial_i(); break;
    case 2: sc_parallel(0); break;
    case 3: sc_parallel(1); break;
    case 4: sc_queue_random(); break;
    case 5: sc_queue_exh(); break;
    case 6: sc_variant_modelled(&V_SHH); break;
    case 7: sc_variant_modelled(&V_SIH); break;
    case 8: sc_variant_modelled(&V_SHI); break;
    case 9: sc_variant_modelled(&V_DHI); break;
    case 10: sc_variant_modelled(&V_DII); break;
    case 11: sc_variant_odd(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu", h_nmalloc, h_nfree);
  h_cov_finish();
  return 0;
}
