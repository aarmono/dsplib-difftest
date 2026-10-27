/*
 * Differential test harness for V.34 "tier 7a" (src/v34/v34_receiver.c): receiver
 * (the V.34 RX data-mode symbol processor; ELF 0x5be90, 4,326 B, GLOBAL cdecl).
 *
 * One source, two binaries (same convention as test/v34_tier{4a,5b,6a}_diff_main.c):
 *   v34_tier7a_test_blob - linked against the real dsplibs.o (-DTEST_BLOB)
 *   v34_tier7a_test_port - linked against src/v34/v34_receiver.c + the ported callees + tables
 * Each scenario runs in its OWN process (argv[1]).
 *
 * WHICH CALLEES ARE REAL: `receiver` calls V34TimingFilter, V34TimingPrefilter, V34Equalizer*,
 * V34scrambler, TimingV34, V34demodulate (LOCAL in the blob), decoderv34, sysdep_memset and the debug
 * printf.  In the blob binary all of them are the BLOB'S OWN (dsplibs.o); in the port binaries they are
 * the already-verified ports (src/v34/v34_dsp_primitives.c, v34_init.c, v34_agc_demod.c, v34_frame_decoder.c).  The
 * harness never substitutes a callee, so the differential result covers receiver's wiring of every
 * one of them.  (The blob's LOCAL V34demodulate is only needed by the harness's private ORACLE copy of
 * receiver, which is compiled from the pristine port source and calls it through a thunk defined
 * here that reaches the blob's LOCAL function by address arithmetic - as in test/v34/v34_agc_demod.main.c;
 * the real receiver inside dsplibs.o needs no thunk.)
 *
 * Objects: tagV34Object is not a struct.  A random fake object (OBJSZ = 0xb400 bytes, byte-offset
 * accessors) is filled from the LCG and then given a VALID configuration for the pointer slots and
 * ranges the callees dereference (RX ring pointer, rotation table pointer/half-length/index, the
 * 36-entry history ring index, the T/2 timing coefficient pointers via V34TimingFiltersInit, the
 * equaliser circular index, TimingV34's nonzero divisors, the vectpp index, the 300-entry history ring
 * index, ...).  Everything else stays random.  The result of each call is the hash of the WHOLE object
 * (pointer slots normalised: in-object -> offset, tables / functions -> ids; raw addresses are never
 * compared across the two processes) plus, for the frame scenario, the recorded descrambler calls.
 *
 * INDEPENDENT MODELS (checked on BOTH sides after every call; exit 3 if violated): (a) the T/2
 * interpolation-phase loop (final b+0x1ac, number of V34demodulate calls -> sink pointer b+0x130);
 * (b) the retrain / RRN counter state machine (b+0x798 and flags 0x40/0x20 from the pre-state and the
 * equaliser output, which the blob leaves in b+0x268/0x26a); (c) the signal-energy level (36-term sum,
 * normalisation, sqrt lookup, compare against obj+0x230) written from scratch with 64-bit arithmetic.
 *
 * FRAME MODE (decoderv34 with flags&0x98 == 0x98 reached through receiver): decodeDepth / demapFrame
 * have unchecked table reads (see the tier-4a notes).  Every frame-scenario state is therefore first run
 * through an ORACLE: a private copy of the PRISTINE src/v34/v34_receiver.c is compiled into this harness as
 * oracle_receiver; its decoderv34 call is replaced by a PROBE that copies the object as it is at that
 * moment (exactly what decoderv34 would see) and returns, and the tier-4a index oracle
 * (oracle_decoderv34) is then run on the copy.  A state that would read outside grid[529] /
 * gInvertPat[16] / the trellis, or vectpp[48], is discarded and re-drawn (deterministically, identically
 * in both processes).  The oracle sources are always the pristine files, so mutating the port under test
 * (env PORT_T7A) cannot change the input stream.
 *
 * Scenarios: 0 single calls (fresh random states, all mode/count classes), 1 the same at debug level 2
 * (every V34* print compared as stdout), 2 chain: retrain / RRN counter, 3 chain: PLL, error window and
 * equaliser adaptation, 4 chain: TRN symbol counts through vectpp / scrambler / shifted-TRN2 / decoder,
 * 5 frame mode (oracle-gated), 6 frame mode at debug level 2, 7 S-S1 / signal-energy exhaustive classes, 8 TimingV34 zero divisors,
 * 9..16 = scenarios 0..7 with the TimingV34 sample indices inside S[] (T07).
 * Every scenario keeps COVERAGE COUNTERS (pre-state predicates and observed effects) and exits 2 if a
 * branch class was never reached; exit 3 = harness assumption / model violated.
 */

#define main t4a_unused_main
#include "v34_frame_decoder.main.c"
#undef main

#include <signal.h>
#include <stdarg.h>
#include <sys/wait.h>
#include <unistd.h>

/* ------------------------------------------------------------ functions under test / callees */
struct tagV34Object;
extern void receiver(struct tagV34Object *obj);
extern void V34TimingFiltersInit(void *timingState);
extern const short hsine1200[16], hsine1680[80], hsine1800[32], hsine2000[48], hsine2400[16];
extern const short V34TimingHPFilterCoeff[40];
extern const short V34TimingPrefilterCoeff[40];
extern const short vectpp[96];
#ifdef TEST_BLOB
extern const short port_vectpp[96];
extern const short port_v8_costbl[256];
extern const short costbl[256];
#endif

#ifdef TEST_BLOB
/* the blob's LOCAL regparm(3) V34demodulate, reached through the GLOBAL `bitreverse` (see tier 2) */
typedef void(__attribute__((regparm(3))) * demod_fn_t)(void *);
extern unsigned short bitreverse(unsigned short x, short nbits);
void V34demodulate(V34Rx *b) {
  static demod_fn_t fn;
  if (!fn) {
    static const unsigned char sig[12] = {0x55, 0x57, 0x89, 0xc7, 0x56, 0x53, 0x83, 0xec, 0x1c, 0x8b, 0x48, 0x04};
    unsigned char *p = (unsigned char *)&bitreverse - 0x5d770 + 0x5af10;
    if (memcmp(p, sig, sizeof sig) != 0) {
      fprintf(stderr, "V34demodulate address arithmetic failed (prologue mismatch)\n");
      exit(3);
    }
    fn = (demod_fn_t)p;
  }
  fn(b);
}
#endif

/* ------------------------------------------------------------ oracle copy of receiver */
static unsigned char g_probe[OBJSZ];
static int g_probe_taken;
static void oracle_dec_probe(void *obj) {
  V34I_SYNC_IMG(); /* x86-64: the oracle copy of the port runs on a SHADOW object; the probe copies the IMAGE as it is at this moment (= obj at -m32) */
  memcpy(g_probe, V34I_IMGP(obj), OBJSZ);
  g_probe_taken = 1;
}
int port_trace_vectpp(int idx) {
  if (idx < 0 || idx > 47) {
    g_oob = 1;
    return 1;
  }
  return 0;
}
#undef OBJ_BYTES
#undef S16
#undef U16
#undef S32
#undef U32
#undef PTR
#undef U8
#define PORT_TRACE_INDEX 1
#define receiver oracle_receiver
#define decoderv34 oracle_dec_probe
#define sar32 t7o_sar32
#define mul32 t7o_mul32
#define vectpp_at t7o_vectpp_at
#define rx_taps t7o_rx_taps
#define rx_phase_error t7o_rx_phase_error
#define rx_tail t7o_rx_tail
#ifndef PORT_T7A_ORACLE_SRC
#define PORT_T7A_ORACLE_SRC "../src/v34/v34_receiver.c"
#endif
#include PORT_T7A_ORACLE_SRC
#undef receiver
#undef decoderv34
#undef sar32
#undef mul32
#undef vectpp_at
#undef rx_taps
#undef rx_phase_error
#undef rx_tail
#undef PORT_TRACE_INDEX
#undef OBJ_BYTES
#undef S16
#undef U16
#undef S32
#undef U32
#undef PTR
#undef U8

/* T06: the oracle copy of receiver is native-typed port code: on x86-64 it runs on the shadow of the image object, like the shimmed functions under test */
#if defined(__x86_64__)
static void V34ORC_receiver(void *obj) { v34i_enter(); oracle_receiver((void *)v34i_in(obj)); v34i_leave(); }
#else
#define V34ORC_receiver oracle_receiver
#endif
/* ---- accessors (again; the oracle include undefined the ones tier4a's harness defined) ---- */
/* V34SHIMS BEGIN (generated by tools/tier_rewrite.py shims; do not edit) */
#if defined(__x86_64__)
static void V34SH_receiver(void *obj) { v34i_enter(); receiver((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_V34TimingFiltersInit(void *timingState) { v34i_enter(); V34TimingFiltersInit((void *)v34i_in((void *)timingState)); v34i_leave(); }
static void *const V34REAL_receiver = (void *)receiver; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_V34TimingFiltersInit = (void *)V34TimingFiltersInit; /* the real address (the name is a macro for the shim below) */
#define receiver V34SH_receiver
#define V34TimingFiltersInit V34SH_V34TimingFiltersInit
#endif
/* V34SHIMS END */
#define OB(o, off) ((unsigned char *)(o) + (off))
#define S16(o, off) (*(int16_t *)OB(o, off))
#define U16(o, off) (*(uint16_t *)OB(o, off))
#define I32(o, off) (*(int32_t *)OB(o, off))
#define U32(o, off) (*(uint32_t *)OB(o, off))
#define BO 0x264
#define bS16(o, off) S16(o, BO + (off))
#define bU16(o, off) U16(o, BO + (off))
#define bI32(o, off) I32(o, BO + (off))
#define bU32(o, off) U32(o, BO + (off))
#define bPTR(o, off) PTRS(o, BO + (off))

static void die(const char *msg) {
  fflush(stdout);
  fprintf(stderr, "HARNESS ERROR: %s\n", msg);
  exit(3);
}

/* debug-print hook: every print is counted by kind (coverage of the level-2 paths) and forwarded to stdout;
 * test/stub_env.c is compiled with its own dsplibs_debug_printf renamed (-Ddsplibs_debug_printf=...) */
static long g_pr_equ, g_pr_retrain, g_pr_reneg, g_pr_agc, g_pr_energy, g_pr_shift, g_pr_ss1, g_pr_dec_reneg, g_pr_other;
int dsplibs_debug_printf(const char *fmt, ...) {
  va_list ap;
  int r;
  if (strstr(fmt, "V34EQU")) g_pr_equ++;
  else if (strstr(fmt, "V34RETRAIN")) g_pr_retrain++;
  else if (strstr(fmt, "V34RENEG, RRN")) g_pr_reneg++;
  else if (strstr(fmt, "V34RENEG, may be")) g_pr_dec_reneg++;
  else if (strstr(fmt, "abcddetect")) g_pr_agc++;
  else if (strstr(fmt, "Signal Energy")) g_pr_energy++;
  else if (strstr(fmt, "shifted TRN2")) g_pr_shift++;
  else if (strstr(fmt, "S-S1")) g_pr_ss1++;
  else g_pr_other++;
  va_start(ap, fmt);
  r = vprintf(fmt, ap);
  va_end(ap);
  return r;
}

/* ------------------------------------------------------------ tables / ids */
static const struct {
  const short *t;
  int L;
} g_rot[5] = {{hsine1200, 8}, {hsine1800, 16}, {hsine2400, 8}, {hsine1680, 40}, {hsine2000, 24}};

static void tables7_check(void) {
#ifdef TEST_BLOB
  if (memcmp(vectpp, port_vectpp, 192)) {
    fprintf(stderr, "table vectpp: blob != port copy\n");
    exit(3);
  }
  if (memcmp(costbl, port_v8_costbl, 512)) {
    fprintf(stderr, "table costbl: blob != port v8_costbl\n");
    exit(3);
  }
#endif
}

/* normalised value of a pointer slot: in-object -> offset, known external -> id, else marker */
static uint32_t norm_ptr(const unsigned char *o, uint32_t v) {
  uintptr_t p = (uintptr_t)v, base = (uintptr_t)o;
  int i;
  if (p >= base && p < base + OBJSZ) return 0x10000000u + (uint32_t)(p - base);
  for (i = 0; i < 5; i++)
    if (p == (uintptr_t)g_rot[i].t) return 0x20000000u + (uint32_t)i;
  if (p == (uintptr_t)V34TimingHPFilterCoeff) return 0x20000010u;
  if (p == (uintptr_t)V34TimingPrefilterCoeff) return 0x20000011u;
  if (p == (uintptr_t)g_coef) return 0x20000020u;
  if (p == (uintptr_t)fn_addr(0)) return 0x20000030u;
  if (p == (uintptr_t)fn_addr(1)) return 0x20000031u;
  if (p == (uintptr_t)fn_addr(2)) return 0x20000032u;
  return 0x3fffffffu;
}

static int g_frame_slots; /* frame scenarios: obj+0xe48 / obj+0xa24 hold pointers */
static unsigned int rx_hash(const unsigned char *o) {
  unsigned int h = FNV0;
  size_t i;
  for (i = 0; i < OBJSZ / 4; i++) {
    uint32_t w;
    memcpy(&w, o + 4 * i, 4);
    if (4 * i == BO + 4 || 4 * i == BO + 0x130 || 4 * i == BO + 0x1b4 || 4 * i == 0x50c + 0x114 || 4 * i == 0x50c + 0x118 ||
        (g_frame_slots && (4 * i == SLOT_FN || 4 * i == SLOT_COEF)))
      w = norm_ptr(o, w);
    h = fnv32(h, w);
  }
  return h;
}

/* ------------------------------------------------------------ generators */
static short samp(int cls) {
  switch (cls) {
    case 0: return 0;
    case 1: return (short)rr(-30, 30);
    case 2: return (short)rr(-500, 500);
    case 3: return (short)rr(-4000, 4000);
    default: return (short)rnd16();
  }
}
static void fill_samples(short *p, int n, int cls) {
  int i;
  for (i = 0; i < n; i++) p[i] = samp(cls);
}

static uint16_t gen_flags(int trn, int frame) {
  uint16_t f = rnd16();
  static const uint16_t bits[] = {0x4, 0x8, 0x10, 0x20, 0x40, 0x100, 0x200, 0x1000, 0x2000};
  size_t i;
  for (i = 0; i < sizeof bits / sizeof bits[0]; i++) {
    if (lcg_bool()) f |= bits[i]; else f = (uint16_t)(f & ~bits[i]);
  }
  if (frame) f |= 0x98u; else f &= (uint16_t)~0x80u; /* decoderv34 frame mode only in the frame scenarios */
  if (trn) f |= 0x400u; else f &= (uint16_t)~0x400u;
  return f;
}

/* symbol count b+0x124 for a TRN-mode call (X = the effective count the code compares) */
static int16_t pick_cnt_trn(uint16_t f, int want) {
  int X, c8 = (f & 8) != 0;
  switch (want) {
    case 0: return (int16_t)(lcg_bool() ? rr(-5, 0x11) : (rr(0, 3) == 0 ? -32768 : rr(0x10, 0x11))); /* early return */
    case 1: X = rr(0x12, 0x132); break;                      /* vectpp when !(f&8) */
    case 2: X = rr(0x133, 0x142); break;                     /* scrambler #1 only */
    case 3: X = rr(0x143, 0x152); break;                     /* scrambler #2 window */
    case 4: X = 0x153; break;                                /* shifted-TRN2 test */
    case 5: X = rr(0x154, 0x332); break;
    case 6: X = rr(0x333, 0x340); break;                     /* decoder boundary */
    case 7: X = rr(0x341, 0x7000); break;
    case 8: X = 0x332; break;
    case 9: X = 0x333; break;
    default: /* X wraps negative through the 16-bit +0x120 (only with flag 8) */
      if (c8) return (int16_t)rr(0x7ee1, 0x7fff);
      X = rr(0x12, 0x7fff);
      break;
  }
  return (int16_t)(c8 ? X - 0x120 : X);
}
/* symbol count for a non-TRN call */
static int16_t pick_cnt_norm(void) {
  switch (rr(0, 13)) {
    case 0: case 1: case 2: return 0;
    case 3: return 1;
    case 4: return 2;
    case 5: return (int16_t)rr(3, 0x3f);
    case 6: return 0x40;
    case 7: return 0x41;
    case 8: return (int16_t)rr(0x42, 0x67);
    case 9: return 0x68;
    case 10: return 0x69;
    case 11: return (int16_t)rr(0x6a, 0x7530);
    case 12: return (int16_t)(lcg_bool() ? 0x7530 : 0x7531);
    default: return (int16_t)(lcg_bool() ? rr(0x7532, 0x7fff) : rr(-32768, -1));
  }
}

static int16_t pick_cnt798(void) {
  switch (rr(0, 12)) {
    case 0: return 0;
    case 1: return (int16_t)0x8b;
    case 2: return (int16_t)0x8c;
    case 3: return (int16_t)0x8d;
    case 4: return (int16_t)rr(-0x84, -0x78);
    case 5: return -0x85;
    case 6: return -0x77;
    case 7: return (int16_t)rr(-0x76, 0x8a);
    case 8: return 0x7fff;
    case 9: return -32768;
    case 10: return -0x41; /* frame mode: 0x9fc alias */
    case 11: return -0x46;
    default: return (int16_t)rnd16();
  }
}

/* valid TimingV34 configuration (mirrors seed_tv34 of test/v34/v34_init.main.c) */
/* T07: scenarios 9..16 = scenarios 0..7 with the TimingV34 sample indices b+0x1ec / b+0x1ee inside S[] (0..19; S[] is 22 shorts), so the recorded output does not
 * depend on the members behind S[] (i386 vs x86-64 layout); the original scenarios keep the -90..150 draw (their goldens are unchanged). */
static int g_t07;

static void seed_timing(unsigned char *o) {
  int k, reg = rr(0, 8);
  switch (lcg_mod(10)) {
    case 0: bU16(o, 0x1c0) = 0xffff; break;
    case 1: bU16(o, 0x1c0) = 0; break;
    case 2: case 3: bU16(o, 0x1c0) = 1; break;
    case 4: bU16(o, 0x1c0) = 0xfffe; break;
    case 5: bU16(o, 0x1c0) = (uint16_t)rr(9, 40); break;
    default: bU16(o, 0x1c0) = (uint16_t)rr(2, 8); break;
  }
  bI32(o, 0x1c8) = (int)lcg_mod(3);
  switch (lcg_mod(6)) {
    case 0: bU16(o, 0x232) = 0xffff; break;
    case 1: bU16(o, 0x232) = (uint16_t)rr(0xfff0, 0xfffe); break;
    case 2: bU16(o, 0x232) = 0; break;
    default: bU16(o, 0x232) = (uint16_t)rr(1, 30); break;
  }
  if (lcg_mod(2)) bU16(o, 0x230) = (uint16_t)((bS16(o, 0x232) > 0 ? bS16(o, 0x232) : 3) - 1);
  else bU16(o, 0x230) = (uint16_t)rr(0, 40);
  bS16(o, 0x234) = (lcg_mod(3) ? (short)rr(0, 15000) : (short)rnd16());
  bS16(o, 0x236) = (lcg_mod(3) ? (short)rr(0, 300) : (short)rnd16());
  if (lcg_mod(3)) { bS16(o, 0x1ec) = 1; bS16(o, 0x1ee) = 2; }
  else if (lcg_mod(2)) { bS16(o, 0x1ec) = 2; bS16(o, 0x1ee) = 1; }
  else { bS16(o, 0x1ec) = (short)(g_t07 ? rr(0, 19) : rr(-90, 150)); bS16(o, 0x1ee) = (short)(g_t07 ? rr(0, 19) : rr(-90, 150)); } /* S[idx] reads must stay clear of the in-object pointer slots (b+0x1b4 at idx -98, obj+0x620/0x624 at idx 162/164) */
  for (k = 0; k < 12; k++) bS16(o, 0x278 + 2 * k) = samp(reg <= 4 ? reg : rr(0, 4));
  bU32(o, 0x1e0) = lcg_mod(3) ? (uint32_t)rr(-100000, 100000) : lcg_next();
  bU32(o, 0x1d8) = lcg_mod(3) ? (uint32_t)rr(-40000, 40000) : lcg_next();
  switch (lcg_mod(5)) {
    case 0: bU16(o, 0x1be) = (uint16_t)rr(1, 40); break;
    case 1: bU16(o, 0x1be) = (uint16_t)rr(0x8000, 0xfffe); break;
    default: bU16(o, 0x1be) = (uint16_t)rr(100, 700); break;
  }
  if (bU16(o, 0x1be) == 0) bU16(o, 0x1be) = 1;
  bS16(o, 0x1cc) = (lcg_mod(3) ? (short)rr(-3000, 3000) : (short)rnd16());
  bU16(o, 0x1ce) = (uint16_t)(lcg_mod(4) ? rr(0, 60) : rr(0, 0x7ffe));
  bS16(o, 0x1d2) = (short)(lcg_mod(3) ? rr(1, 50) : rr(-5, 32767));
  if (lcg_mod(3) == 0) bS16(o, 0x1d2) = (short)rr(-3, (int)(short)(uint16_t)(bU16(o, 0x1ce) + 1));
}

/*
 * Complete valid random state.  trn = flags&0x400 in effect, frame = frame scenario (0x98 bits set, region
 * generated, small equaliser output so the symbol stays inside the decoder's lattice), want = symbol-count
 * class for the TRN branches.  Deterministic in the LCG state and in the buffer address only.
 */
static void build_state(unsigned char *o, int trn, int frame, int want, int eqcls) {
  int k, rot, L, cls;
  uint16_t f;

  fill_random(o, OBJSZ);

  /* ---- V34demodulate / RX ring / AGC ---- */
  bPTR(o, 0x130) = NULL; /* written by receiver at entry; scrubbed so a random word never normalises differently in two buffers */
  bPTR(o, 4) = o + BO + 0x0c + 4 * rr(0, 63);
  bS16(o, 0x19c) = (int16_t)(rr(0, 9) == 0 ? rr(34, 35) : rr(0, 35));
  rot = rr(0, 4);
  L = g_rot[rot].L;
  bPTR(o, 0x1b4) = (void *)g_rot[rot].t;
  bU16(o, 0x1ba) = (uint16_t)L;
  bU16(o, 0x1b8) = (uint16_t)(rr(0, 3) == 0 ? rr(0, L) : 1);
  bU16(o, 0x1bc) = (uint16_t)rr(0, L - 1);
  switch (rr(0, 5)) {
    case 0: bS16(o, 0x136) = 0x200; break;
    case 1: bS16(o, 0x136) = (int16_t)rr(0, 100); break;
    case 2: bS16(o, 0x136) = (int16_t)rr(0, 0x7fff); break;
    case 3: bS16(o, 0x136) = 0x4000; break;
    default: bS16(o, 0x136) = (int16_t)rnd16(); break;
  }
  bU16(o, 0x12a) = (uint16_t)(rr(0, 2) ? rr(0, 5) : (int)rnd16());
  if (rr(0, 2) == 0) bU32(o, 0x12c) = (uint32_t)rr(0, 100000);
  fill_samples((short *)OB(o, BO + 0x13c), 36, rr(0, 4)); /* 36-entry history ring (also the energy-check input) */
  bS16(o, 0x19c) = (int16_t)(bS16(o, 0x19c) & 0x3f);
  if (bS16(o, 0x19c) > 35) bS16(o, 0x19c) = 35;
  fill_samples((short *)OB(o, BO + 0x0c), 128, rr(0, 4));

  /* ---- T/2 timing interpolation loop ---- */
  switch (rr(0, 9)) {
    case 0: case 1: case 2: case 3: case 4: case 5: bS16(o, 0x128) = 4; break;
    case 6: bS16(o, 0x128) = (int16_t)rr(1, 5); break;
    case 7: bS16(o, 0x128) = 0; break;
    default: bS16(o, 0x128) = (int16_t)rr(-3, -1); break;
  }
  {
    int M, step;
    switch (rr(0, 9)) {
      case 0: case 1: case 2: case 3: case 4: case 5: M = 16000; break;
      case 6: M = rr(1, 32767); break;
      case 7: M = 0; break;
      case 8: M = rr(-32768, -1); break;
      default: M = rr(1, 8); break;
    }
    switch (rr(0, 5)) {
      case 0: case 1: step = 12800; break;
      case 2: step = M > 0 ? rr(0, 2 * M) : (int)rnd16(); break;
      case 3: step = M > 0 ? rr(M, 2 * M) : (int)rnd16(); break; /* two-wrap iterations */
      case 4: step = rr(0, 65535); break;
      default: step = M > 0 ? rr(0, M) : 0; break;
    }
    bU16(o, 0x1b0) = (uint16_t)M;
    bU16(o, 0x1ae) = (uint16_t)step;
    bU16(o, 0x1ac) = (uint16_t)(M > 0 && rr(0, 3) ? rr(0, M) : (int)rnd16());
  }
  cls = rr(0, 4);
  for (k = 0; k < 4; k++) bS16(o, 0x240 + 2 * k) = samp(cls);

  /* ---- timing loop (TimingV34) + timing / equaliser sub-objects ---- */
  seed_timing(o);
  V34TimingFiltersInit(o + 0x50c);
  {
    unsigned char *eq = o + 0x630;
    int ci = eqcls >= 0 ? eqcls : rr(0, 4), cq = eqcls >= 0 ? eqcls : rr(0, 4), cd = rr(0, 4);
    fill_samples((short *)(eq + 0x000), 160, cd);            /* iDelay, qDelay */
    fill_samples((short *)(eq + 0x140), 80, ci);             /* iCoefHi */
    fill_samples((short *)(eq + 0x1e0), 80, cq);             /* qCoefHi */
    if (rr(0, 3) == 0) { /* the two tap windows of the shifted-TRN2 test with independent amplitudes */
      int a = rr(0, 4), c = rr(0, 4);
      fill_samples((short *)(eq + 0x140) + 0x20, 16, a);
      fill_samples((short *)(eq + 0x1e0) + 0x20, 16, a);
      fill_samples((short *)(eq + 0x140) + 0x3c, 16, c);
      fill_samples((short *)(eq + 0x1e0) + 0x3c, 16, c);
    }
    I32(o, 0x630 + 0x3c0) = rr(0, 79);
  }
  bS16(o, 0x798) = pick_cnt798();

  /* ---- retrain detector history, PLL, error windows ---- */
  cls = rr(0, 3);
  for (k = 0; k < 4; k++) bS16(o, 0x268 + 2 * k) = samp(cls);
  if (rr(0, 3) == 0) { /* A / B near the equaliser output: small distances */
    for (k = 0; k < 4; k++) bS16(o, 0x268 + 2 * k) = (int16_t)rr(-1500, 1500);
  }
  switch (rr(0, 3)) { /* realistic phasor most of the time */
    case 0: bS16(o, 0x1f2) = 0x4000; bS16(o, 0x1f4) = 0; break;
    case 1: {
      int a = rr(0, 255);
      bS16(o, 0x1f2) = costbl[(a + 0x40) & 0xff]; /* cos-like */
      bS16(o, 0x1f4) = costbl[a];
      break;
    }
    default: bS16(o, 0x1f2) = (int16_t)rnd16(); bS16(o, 0x1f4) = (int16_t)rnd16(); break;
  }
  bS16(o, 0x200) = (int16_t)(rr(0, 2) ? rr(0, 20) : (int)rnd16());
  bS16(o, 0x202) = (int16_t)(rr(0, 2) ? rr(0, 20) : (int)rnd16());
  bU16(o, 0x21c) = (uint16_t)(rr(0, 2) == 0 ? 0x3ff : (rr(0, 2) == 0 ? rr(0, 0x3ff) : (int)rnd16()));
  bU32(o, 0x220) = (uint32_t)(rr(0, 1) ? rr(-1000, 100000) : (int)lcg_next());
  bU32(o, 0x228) = (uint32_t)(rr(0, 1) ? rr(-1000, 100000) : (int)lcg_next());
  bU32(o, 0x24c) = (uint32_t)(rr(0, 1) ? rr(0, 100000) : (int)lcg_next());
  bS16(o, 0x120) = (int16_t)(rr(0, 4) == 0 ? rr(44, 47) : rr(0, 47));
  {
    int ri = rr(0, 19);
    S16(o, 0x2aa4) = (int16_t)(ri == 0 ? 0x12b : (ri == 1 ? 0x12c : (ri == 2 ? -1 : (ri == 3 ? -2 : (ri == 4 ? rr(0x12d, 0x200) : rr(0, 0x12a))))));
  }
  switch (rr(0, 4)) {
    case 0: I32(o, 0x230) = rr(-5, 5); break;
    case 1: I32(o, 0x230) = rr(0, 60); break;
    case 2: I32(o, 0x230) = rr(0, 0x7fff); break;
    case 3: I32(o, 0x230) = rr(-40000, 40000); break;
    default: I32(o, 0x230) = (int32_t)lcg_next(); break;
  }
  if (I32(o, 0) == 10) I32(o, 0) = 11;

  /* ---- flags and symbol counters ---- */
  f = gen_flags(trn, frame);
  bU16(o, 0x122) = f;
  if (trn) {
    bS16(o, 0x124) = pick_cnt_trn(f, want >= 0 ? want : rr(0, 10));
  } else {
    bS16(o, 0x124) = pick_cnt_norm();
  }
  bS16(o, 0x218) = (int16_t)rnd16();
  bU32(o, 0x1a0) = lcg_next();
  bU32(o, 0x1a4) = lcg_next();

  if (frame) {
    gen_region(o, (int)(lcg_next() & 1));
    tame_frame_params(o);
    U16(o, 0xa00) = pick_a00();
    U16(o, 0xe4c) = pick_e4c();
    U16(o, 0x4ca) = (uint16_t)(rr(0, 5) == 0 ? rr(0x7fe0, 0x7fff) : rr(-5, 80));
    /* re-assert receiver's own fields the region generator / fill may have overlapped */
    bU16(o, 0x122) = f;
    bS16(o, 0x124) = (int16_t)rr(0x213, 0x7edf); /* (short)(cnt + 0x120) > 0x332 with flag 8 (0x7ee0 would wrap negative) */
    bS16(o, 0x798) = pick_cnt798();
  }
}

/* ------------------------------------------------------------ independent models */
typedef struct {
  int have_phase;
  unsigned ph_final;
  int ndemod;
} phase_model_t;

/* stage (1): the T/2 interpolation phase bookkeeping (from the machine code, written independently) */
static phase_model_t model_phase(const unsigned char *pre) {
  phase_model_t m = {0, 0, 0};
  unsigned ph = bU16(pre, 0x1ac), step = bU16(pre, 0x1ae), Mz = bU16(pre, 0x1b0);
  int Ms = (int16_t)Mz, n = bS16(pre, 0x128), i;
  for (i = 0; i < n; i++) {
    unsigned sum = ph + step, s = sum & 0xffff;
    if ((int)s >= Ms) {
      unsigned t = (sum - Mz) & 0xffffffffu;
      if ((int)(t & 0xffff) < Ms) {
        ph = t & 0xffff;
        m.ndemod += 1;
      } else {
        ph = (t - Mz) & 0xffff;
        m.ndemod += 2;
      }
    } else {
      ph = s;
    }
  }
  m.ph_final = ph;
  m.have_phase = (n > 0);
  return m;
}

/* the shifted-TRN2 test (count 0x153, flags&0x10, scr path): sum |c|^2 over equaliser taps 0x20..0x2f vs 0x3c..0x4b
 * (the coefficient arrays are unchanged until the adaptation at the very end of the call, so the PRE-state is what
 * the test sees); a positive result copies b+0x1a4 -> b+0x1a0 (after the first scrambler step) and always clears b+0x1a4 */
static long g_shift_true, g_shift_false;
static void check_shifted(const unsigned char *pre, const unsigned char *post) {
  uint32_t s1 = 0, s2 = 0;
  int k;
  for (k = 0x20; k <= 0x2f; k++) {
    int32_t a = bS16(pre, 0x50c + 2 * k), c = bS16(pre, 0x5ac + 2 * k);
    s1 += (uint32_t)(a * a) + (uint32_t)(c * c);
  }
  for (k = 0x3c; k <= 0x4b; k++) {
    int32_t a = bS16(pre, 0x50c + 2 * k), c = bS16(pre, 0x5ac + 2 * k);
    s2 += (uint32_t)(a * a) + (uint32_t)(c * c);
  }
  if (bU32(post, 0x1a4) != 0) die("shifted-TRN2: b+0x1a4 not cleared");
  if ((int32_t)s2 > (int32_t)s1) {
    g_shift_true++;
    if (bU32(post, 0x1a0) != bU32(pre, 0x1a4)) die("shifted-TRN2 model: expected b+0x1a0 = old b+0x1a4");
  } else {
    g_shift_false++;
    if (bU32(post, 0x1a0) == bU32(pre, 0x1a4)) die("shifted-TRN2 model: b+0x1a0 unexpectedly equals old b+0x1a4");
  }
}

/* stage (8): the signal-energy level, written independently with 64-bit arithmetic */
static int model_energy_level(const unsigned char *pre) {
  int64_t sum = 0;
  uint32_t e;
  int k, sh = 0;
  for (k = 0; k < 36; k++) {
    int64_t x = bS16(pre, 0x13c + 2 * k);
    int64_t t = (x * 0x38e) >> 15; /* floor */
    sum += t * x;
  }
  e = (uint32_t)sum; /* the machine wraps at 32 bits */
  if (e == 0) return 0;
  while (e <= 0x1fffffffu) { e <<= 1; sh++; }
  {
    uint32_t mant = (e >> 15) & 0xffff;
    int64_t q;
    int ix;
    if (sh & 1) mant >>= 1;
    q = ((int64_t)mant + 0x40) >> 7;
    ix = (int)((uint16_t)(q - 0x40));
    if (ix > 0xbf) ix = 0xbf;
    return (int16_t)(uint16_t)(((uint32_t)(uint16_t)FPM_sqrt_table[ix]) >> ((sh >> 1) & 31));
  }
}

/* ------------------------------------------------------------ the call driver */
typedef struct {
  logger_t lg;
  long calls;
  long n_pos, n_zero, n_neg, w0, w1, w2, odd;
  long trn, norm, early, vect, dec, scr, scr2, x153, f8, f10, wrapneg;
  long gate_pass, gate_ret, cls0, cls1, cls2, c40, gt40, gt68;
  long ss1, ss1_sum0, ss1_disc, ss1_ok, ss1_eq;
  long reached, win, win_sat, adapt, center, ring_wrap, ring_neg, vidx_wrap;
  long frame_dec, f100_set, f100_clr, tail100;
} stat_t;

static long g_rt_inc, g_rt_flag40, g_rt_dec, g_rt_rrn, g_rt_reset, g_mode1_post;
static int g_frame_mode;

/* stage (3): the retrain / RRN counter state machine, predicted from the pre-state and the equaliser output
 * (which the machine leaves in b+0x268/0x26a); flags 0x40 / 0x20 and the counter are compared */
static void check_retrain(const unsigned char *pre, const unsigned char *post) {
  int32_t I = bS16(post, 0x268), Q = bS16(post, 0x26a);
  int32_t d1i = (int16_t)(I - bU16(pre, 0x268)), d1q = (int16_t)(Q - bU16(pre, 0x26a));
  int32_t d2i = (int16_t)(I - bU16(pre, 0x26c)), d2q = (int16_t)(Q - bU16(pre, 0x26e));
  int16_t e1 = (int16_t)(((int64_t)d1i * d1i + (int64_t)d1q * d1q) >> 14);
  int16_t e2 = (int16_t)(((int64_t)d2i * d2i + (int64_t)d2q * d2q) >> 14);
  int c = bS16(pre, 0x798);
  uint16_t fl = bU16(pre, 0x122), pc = bU16(pre, 0x798);
  uint16_t want_fl = fl, want_c;

  if (bU16(post, 0x26c) != bU16(pre, 0x268) || bU16(post, 0x26e) != bU16(pre, 0x26a)) die("retrain model: history shift");
  if (e1 <= 0x80) {
    if ((int16_t)(pc + 1) <= 0x8c) { want_c = (uint16_t)(pc + 1); g_rt_inc++; }
    else { want_c = 0; want_fl |= 0x40; g_rt_flag40++; }
  } else if (e2 <= 0x80) {
    want_c = (uint16_t)(pc - 1);
    g_rt_dec++;
  } else {
    if (c + 0x78 <= 0 && c + 0x84 > 0) { want_fl |= 0x20; g_rt_rrn++; }
    want_c = 0;
    g_rt_reset++;
  }
  if ((bU16(post, 0x122) & 0x60) != (want_fl & 0x60)) die("retrain model: flags 0x40/0x20 mismatch");
  if (bU16(post, 0x798) != want_c) die("retrain model: counter mismatch");
}

static void model_checks(stat_t *st, const unsigned char *pre, const unsigned char *post) {
  phase_model_t pm = model_phase(pre);
  uint32_t sink = norm_ptr(post, bU32(post, 0x130));
  uint32_t want = 0x10000000u + (uint32_t)(BO + 0x10c + 2 * pm.ndemod);

  /* (a) interpolation phase loop */
  /* (TimingV34 with mode 1 runs setInitialPhase, which rewrites b+0x1ac after the loop: no prediction then) */
  if (bU16(pre, 0x1c0) != 1) {
    if (pm.have_phase && bU16(post, 0x1ac) != pm.ph_final) die("phase model: b+0x1ac mismatch");
    if (!pm.have_phase && bU16(post, 0x1ac) != bU16(pre, 0x1ac)) die("phase model: b+0x1ac changed with n<=0");
  }
  if (sink != want) die("phase model: sink pointer (V34demodulate call count) mismatch");
  /* (b) retrain counter (TRN mode) */
  if (bU16(pre, 0x122) & 0x400) check_retrain(pre, post);
  /* (b2) shifted-TRN2 test */
  if ((bU16(pre, 0x122) & 0x400) && (bU16(pre, 0x122) & 0x10) && bS16(pre, 0x124) > 0x11) {
    int cnt = bS16(pre, 0x124), X = (bU16(pre, 0x122) & 8) ? (int16_t)(cnt + 0x120) : cnt;
    if (!((!(bU16(pre, 0x122) & 8) && cnt <= 0x132) || X > 0x332) && (int16_t)X == 0x153) check_shifted(pre, post);
  }
  /* (c) S-S1 hit (non-TRN -> flags 0x600) and the signal-energy level */
  if (!(bU16(pre, 0x122) & 0x400) && (bU16(post, 0x122) & 0x400)) {
    int lvl = model_energy_level(post); /* nothing after the loop writes the ring */
    st->ss1++;
    if (lvl == 0) st->ss1_sum0++;
    if (lvl <= I32(pre, 0x230)) {
      st->ss1_disc++;
      if (lvl == I32(pre, 0x230)) st->ss1_eq++;
      if (I32(post, 0) != 10) die("energy model: expected a disconnect request (obj+0 == 10)");
    } else {
      st->ss1_ok++;
      if (I32(post, 0) != I32(pre, 0)) die("energy model: unexpected write to obj+0");
    }
    if ((bU16(post, 0x122) & 0x700) != 0x600) die("S-S1: flags are not (flags & ~0x100) | 0x600");
    if (bS16(post, 0x124) != 0) die("S-S1: b+0x124 not cleared");
  } else if (I32(post, 0) != I32(pre, 0)) {
    die("obj+0 written outside the energy path");
  }
}

/* coverage classification from the pre-state (independent predicates) and observed effects */
static void classify(stat_t *st, const unsigned char *pre, const unsigned char *post) {
  uint16_t fl = bU16(pre, 0x122);
  int n = bS16(pre, 0x128), cnt = bS16(pre, 0x124), i;
  int trn = (fl & 0x400) != 0;

  if (bS16(post, 0x1c0) == 1) g_mode1_post++; /* informational: TimingV34 never leaves the mode at 1 */
  if (n > 0) st->n_pos++; else if (n == 0) st->n_zero++; else st->n_neg++;
  {
    unsigned ph = bU16(pre, 0x1ac), step = bU16(pre, 0x1ae), Mz = bU16(pre, 0x1b0);
    int Ms = (int16_t)Mz;
    for (i = 0; i < n; i++) {
      unsigned sum = ph + step, s = sum & 0xffff;
      if ((int)s >= Ms) {
        unsigned t = sum - Mz;
        if ((int)(t & 0xffff) < Ms) { ph = t & 0xffff; st->w1++; }
        else { ph = (t - Mz) & 0xffff; st->w2++; }
      } else {
        ph = s;
        st->w0++;
      }
      if (i & 1) st->odd++;
    }
  }
  {
    int idx = S16(pre, 0x2aa4);
    if (idx < 0) st->ring_neg++;
    if ((uint16_t)(idx + 1) > 0x12b) st->ring_wrap++;
  }
  if (trn) {
    st->trn++;
    if (cnt <= 0x11) {
      st->early++;
    } else {
      int X = (fl & 8) ? (int16_t)(cnt + 0x120) : cnt;
      if (fl & 8) st->f8++;
      if (!(fl & 8) && cnt <= 0x132) {
        st->vect++;
        if (bS16(pre, 0x120) + 1 > 0x2f) st->vidx_wrap++;
      } else if (X > 0x332) {
        st->dec++;
      } else {
        st->scr++;
        if (X < 0) st->wrapneg++;
        if (fl & 0x10) {
          st->f10++;
          if ((uint16_t)(X - 0x143) <= 0xf) st->scr2++;
          if ((int16_t)X == 0x153) st->x153++;
        }
      }
    }
  } else {
    st->norm++;
    if (bS16(post, 0x218) == 0x7000 || bU16(post, 0x122) != fl) st->gate_pass++; else st->gate_ret++;
    if (bS16(post, 0x218) == 0x7000 && bS16(pre, 0x218) != 0x7000) {
      if (cnt == 0) {
        st->cls0++;
        if (bS16(post, 0x124) == 1) st->cls1++;
        if (bS16(post, 0x124) == 2) st->cls2++;
      }
      if (cnt == 0x40) st->c40++;
      if (cnt > 0x40 && (bU16(post, 0x122) & 0x200)) st->gt40++;
      if (cnt > 0x68 && ((bU16(post, 0x122) & 0x100) || (bU16(post, 0x122) & 0x400))) st->gt68++;
    }
  }
  /* did the error accumulators run? (b+0x21c advances by one iff they did) */
  if (bU16(post, 0x21c) == (uint16_t)((bU16(pre, 0x21c) + 1u) & 0x3ff)) {
    st->reached++;
    if (trn) st->adapt++; else st->center++;
    if (bU16(post, 0x21c) == 0) {
      st->win++;
      if (bS16(post, 0x21a) == 0x7fff || bS16(post, 0x224) == 0x7fff) st->win_sat++;
    }
  } else if (bU16(post, 0x21c) != bU16(pre, 0x21c)) {
    die("b+0x21c changed without advancing by one");
  } else if (trn && cnt > 0x11) {
    st->tail100++; /* TRN call that passed the front end but skipped the accumulators: flags & 0x100 */
  }
  if (g_frame_mode && trn) {
    if (bS16(post, 0x218) == 0x2000) st->frame_dec++;
    if (!(fl & 0x100) && (bU16(post, 0x122) & 0x100)) st->f100_set++;
    if ((fl & 0x100) && !(bU16(post, 0x122) & 0x100)) st->f100_clr++;
  }
}

static long g_dump_at = -1;
static const char *g_dump_file;
static void dump_obj(const char *file, const unsigned char *o) {
  FILE *f = fopen(file, "wb");
  if (f) {
    fwrite(o, 1, OBJSZ, f);
    fclose(f);
  }
}
static unsigned int do_call(stat_t *st, unsigned char *w, const unsigned char *pre, long a, long b) {
  unsigned int h;
  receiver(w);
  if (g_dump_at == st->calls && g_dump_file) {
    char n[300];
    snprintf(n, sizeof n, "%s.pre", g_dump_file);
    dump_obj(n, pre);
    snprintf(n, sizeof n, "%s.post", g_dump_file);
    dump_obj(n, w);
  }
  h = rx_hash(w);
  model_checks(st, pre, w);
  classify(st, pre, w);
  st->calls++;
  log_call(&st->lg, h, a, b);
  return h;
}

static void stat_init(stat_t *s, const char *name, long every) {
  memset(s, 0, sizeof *s);
  s->lg.name = name;
  s->lg.all = FNV0;
  s->lg.every = every;
}

static void need_loop(stat_t *s, int all) {
  need("n > 0 calls", s->n_pos);
  if (all) {
    need("n == 0 calls", s->n_zero);
    need("n < 0 calls", s->n_neg);
  }
  need("interpolation iterations, no wrap", s->w0);
  need("interpolation iterations, one wrap", s->w1);
  need("interpolation iterations, two wraps", s->w2);
  need("odd iterations (T/2 prefilter -> equaliser delay line)", s->odd);
  need("history ring index wrap", s->ring_wrap);
}
static void need_retrain(void) {
  need("retrain: e1<=0x80 counter increment", g_rt_inc);
  need("retrain: counter overflow -> flag 0x40", g_rt_flag40);
  need("retrain: e2<=0x80 counter decrement", g_rt_dec);
  need("retrain: RRN window -> flag 0x20", g_rt_rrn);
  need("retrain: else/else counter reset", g_rt_reset);
}

/* ================================================================== scenario 0/1: single calls */
static void scen_single(int dbg) {
  stat_t st;
  unsigned char *o = xalloc(OBJSZ), *w = xalloc(OBJSZ);
  long iter, total = dbg ? 12000 : 80000;

  stat_init(&st, dbg ? "receiver(single,debug)" : "receiver(single)", dbg ? 1000000 : 4000);
  g_frame_slots = 0;
  g_frame_mode = 0;
  for (iter = 0; iter < total; iter++) {
    int trn = (int)(lcg_next() >> 30) & 1;
    build_state(o, trn, 0, -1, -1);
    {
      int steer = rr(0, 23);
      if (steer < 2) {
        /* zero-error state: equaliser output 0 (zero coefficients), phasor 0 -> rotated symbol 0, error sums 0,
         * window about to publish: the published sums are exactly 0 (boundary of the `< 0` saturation tests) */
        memset(o + 0x630 + 0x140, 0, 320);
        bS16(o, 0x1f2) = 0;
        bS16(o, 0x1f4) = 0;
        bU32(o, 0x220) = 0;
        bU32(o, 0x228) = 0;
        bU16(o, 0x21c) = 0x3ff;
        bU16(o, 0x122) = (uint16_t)(bU16(o, 0x122) & ~0x100u);
        if (lcg_bool()) bU16(o, 0x122) = (uint16_t)(bU16(o, 0x122) & ~0x1000u);
      } else if (steer < 5) {
        /* symbol-count boundary of the V34EQU print (b+0x124 <= 0x7530): TRN, decoder path, window about to publish */
        static const int16_t cb[3] = {0x752f, 0x7530, 0x7531};
        bU16(o, 0x122) = (uint16_t)((bU16(o, 0x122) | 0x400u) & ~0x108u);
        bS16(o, 0x124) = cb[steer - 2];
        bU16(o, 0x21c) = 0x3ff;
      } else if (steer < 8) {
        /* shifted-TRN2 test at its boundary: both tap windows zero except single +-1 coefficients, so the two sums
         * are 0 / 1 / 1 / 1 (s2 == s1 + 1, equal, s1 == s2 + 1) */
        int k;
        bU16(o, 0x122) = (uint16_t)((bU16(o, 0x122) | 0x410u) & ~0x8u);
        bS16(o, 0x124) = 0x153;
        for (k = 0x20; k <= 0x2f; k++) { bS16(o, 0x50c + 2 * k) = 0; bS16(o, 0x5ac + 2 * k) = 0; }
        for (k = 0x3c; k <= 0x4b; k++) { bS16(o, 0x50c + 2 * k) = 0; bS16(o, 0x5ac + 2 * k) = 0; }
        if (steer == 5) bS16(o, 0x50c + 2 * rr(0x3c, 0x4b)) = (int16_t)(lcg_bool() ? 1 : -1);          /* s2 = 1, s1 = 0 */
        else if (steer == 6) { bS16(o, 0x5ac + 2 * rr(0x20, 0x2f)) = (int16_t)(lcg_bool() ? 1 : -1);
                               bS16(o, 0x5ac + 2 * rr(0x3c, 0x4b)) = (int16_t)(lcg_bool() ? 1 : -1); } /* s1 = s2 = 1 */
        else bS16(o, 0x50c + 2 * rr(0x20, 0x2f)) = (int16_t)(lcg_bool() ? 1 : -1);                     /* s1 = 1, s2 = 0 */
      }
    }
    if (dbg) {
      static const unsigned int lv[6] = {2u, 0u, 1u, 0x80000000u, 3u, 0xffffffffu};
      dsplibs_debug_level = lv[iter % 6];
    }
    memcpy(w, o, OBJSZ);
    do_call(&st, w, o, (long)bU16(w, 0x122), (long)bS16(w, 0x124));
  }
  dsplibs_debug_level = 0;
  log_done(&st.lg);
  need_loop(&st, 1);
  need("TRN calls", st.trn);
  need("non-TRN calls", st.norm);
  need("TRN early return (count <= 0x11, memset)", st.early);
  need("TRN vectpp reference symbol", st.vect);
  need("TRN vectpp index wrap (0x2f -> 0)", st.vidx_wrap);
  need("TRN decoderv34 (slicer) path", st.dec);
  need("TRN scrambler path", st.scr);
  need("TRN scrambler #2 window (flags&0x10, 0x143..0x152)", st.scr2);
  need("TRN shifted-TRN2 test (count 0x153, flags&0x10)", st.x153);
  need("shifted-TRN2 test positive (s2 > s1)", g_shift_true);
  need("shifted-TRN2 test negative", g_shift_false);
  need("flags&8 (+0x120 on the count)", st.f8);
  need("X wrapped negative (short)(cnt+0x120)", st.wrapneg);
  need_retrain();
  need("non-TRN gate passed (b+0x1c0 > 1)", st.gate_pass);
  need("non-TRN gate return (b+0x1c0 <= 1)", st.gate_ret);
  printf("  info: calls that leave b+0x1c0 == 1 after TimingV34 = %ld (the gate's ==1 boundary is unreachable)\n", g_mode1_post);
  need("non-TRN classification (count 0) -> 1", st.cls1);
  need("non-TRN classification (count 0) -> 2", st.cls2);
  need("non-TRN count == 0x40", st.c40);
  need("non-TRN count > 0x40 sets flag 0x200", st.gt40);
  need("non-TRN count > 0x68 (flag 0x100 / S-S1)", st.gt68);
  need("S-S1 hit -> energy check", st.ss1);
  need("energy level == 0", st.ss1_sum0);
  need("energy check: at or below the threshold (obj+0 = 10)", st.ss1_disc);
  need("energy check: above the threshold", st.ss1_ok);
  need("error accumulators reached, TRN (V34EqualizerAdapt)", st.adapt);
  need("error accumulators reached, non-TRN (V34EqualizerCenterAdapt)", st.center);
  need("error window published (b+0x21c wrapped to 0)", st.win);
  need("window: saturated 0x7fff value", st.win_sat);
  need("TRN call that skipped the accumulators (flags & 0x100)", st.tail100);
  if (dbg) {
    need("print: V34EQU", g_pr_equ);
    need("print: V34RETRAIN", g_pr_retrain);
    need("print: V34RENEG, RRN request", g_pr_reneg);
    need("print: V34AGC ... AGC frozen", g_pr_agc);
    need("print: Signal Energy below Threshold", g_pr_energy);
    need("print: Detected shifted TRN2", g_pr_shift);
    need("print: S-S1 is detected", g_pr_ss1);
  }
  free(o);
  free(w);
}

/* ================================================================== chains (persistent state) */
static void world_fix(unsigned char *o) {
  /* what the surrounding system guarantees: valid ring index, nonzero divisors, no S-S1-forbidden garbage */
  int idx = S16(o, 0x2aa4);
  if (idx < 0 || idx > 0x12b) S16(o, 0x2aa4) = (int16_t)rr(0, 0x12b);
  if ((short)(uint16_t)(bU16(o, 0x1ce) + 1) == 0) bU16(o, 0x1ce) = 0;
  if (bU16(o, 0x1be) == 0) bU16(o, 0x1be) = 1;
  if (I32(o, 0) == 10) I32(o, 0) = 11;
  if (bS16(o, 0x120) < 0 || bS16(o, 0x120) > 47) bS16(o, 0x120) = 0;
  if (bS16(o, 0x19c) < 0 || bS16(o, 0x19c) > 35) bS16(o, 0x19c) = 0;
}

/* kind 0: retrain counter; 1: PLL / window / adaptation (non-TRN); 2: TRN symbol-count sweep */
static void scen_chain(int kind, int f8) {
  static const char *names[3] = {"receiver(chain,retrain)", "receiver(chain,pll-window)", "receiver(chain,trn-sweep)"};
  stat_t st;
  unsigned char *live = xalloc(OBJSZ), *pre = xalloc(OBJSZ);
  long iter, total = kind == 2 ? 30000 : 40000;
  int remain = 0, cnt = 0x10;

  stat_init(&st, names[kind], 4000);
  g_frame_slots = 0;
  g_frame_mode = 0;
  for (iter = 0; iter < total; iter++) {
    if (remain <= 0) {
      uint16_t f;
      build_state(live, kind != 1, 0, -1, kind == 0 ? (rr(0, 2) ? 0 : 1) : (kind == 1 ? rr(0, 3) : -1));
      f = bU16(live, 0x122);
      if (kind == 0) {
        f |= 0x400;
        bS16(live, 0x798) = (int16_t)(rr(0, 2) ? rr(0x70, 0x8b) : pick_cnt798());
      } else if (kind == 1) {
        f &= (uint16_t)~0x500u; /* non-TRN, adaptation on */
        bS16(live, 0x1c0) = (int16_t)rr(2, 8);
        bU16(live, 0x21c) = (uint16_t)rr(0x3f0, 0x3ff);
      } else {
        f |= 0x400;
        f = (uint16_t)(f8 ? (f | 8) : (f & ~8u));
        if (lcg_bool()) f &= (uint16_t)~0x100u;
        cnt = rr(0, 3) ? 0x10 : rr(0x10, 0x13);
        bU16(live, 0x21c) = (uint16_t)rr(0x3f0, 0x3ff);
      }
      bU16(live, 0x122) = f;
      remain = kind == 2 ? rr(900, 1300) : rr(200, 700);
    }
    remain--;
    /* the surrounding system supplies fresh input samples and the symbol counter every call */
    fill_samples((short *)OB(live, BO + 0x0c), 128, rr(0, 3));
    if (kind == 1) {
      int r = rr(0, 5);
      bS16(live, 0x124) = (int16_t)(r == 0 ? 0 : (r == 1 ? 1 : (r == 2 ? 2 : rr(3, 0x3f))));
      if (bS16(live, 0x1c0) <= 1) bS16(live, 0x1c0) = 2;
    } else if (kind == 2) {
      bS16(live, 0x124) = (int16_t)cnt;
      cnt += 1;
      if (cnt > 0x360) cnt = 0x10;
    } else if (rr(0, 24) == 0) {
      int k;
      for (k = 0; k < 4; k++) bS16(live, 0x268 + 2 * k) = (int16_t)rr(-6000, 6000); /* A and B far from the output */
      if (lcg_bool()) bS16(live, 0x798) = (int16_t)rr(-0x84, -0x77);
    } else if (rr(0, 9) == 0) {
      bS16(live, 0x268) = (int16_t)rr(-6000, 6000); /* A far, B near */
      bS16(live, 0x26a) = (int16_t)rr(-6000, 6000);
    }
    world_fix(live);
    memcpy(pre, live, OBJSZ);
    do_call(&st, live, pre, (long)bU16(live, 0x122), (long)bS16(live, 0x124));
  }
  log_done(&st.lg);
  need_loop(&st, 0);
  if (kind == 0) need_retrain();
  if (kind == 1) {
    need("non-TRN gate passed", st.gate_pass);
    need("error accumulators reached, non-TRN (V34EqualizerCenterAdapt)", st.center);
    need("error window published", st.win);
    need("non-TRN classification -> 1", st.cls1);
  }
  if (kind == 2) {
    if (!f8) {
      need("TRN vectpp reference symbol", st.vect);
      need("TRN vectpp index wrap", st.vidx_wrap);
    } else {
      need("flags&8 chain calls", st.f8);
    }
    need("TRN scrambler path", st.scr);
    need("TRN decoder (slicer) path", st.dec);
    need("TRN early return", st.early);
    need("error accumulators reached, TRN (V34EqualizerAdapt)", st.adapt);
    need("error window published", st.win);
    need("TRN call that skipped the accumulators (flags & 0x100)", st.tail100);
  }
  free(live);
  free(pre);
}

/* ================================================================== scenario 7: S-S1 / energy classes */
static void scen_ss1(void) {
  stat_t st;
  unsigned char *o = xalloc(OBJSZ), *w = xalloc(OBJSZ);
  long iter, total = 60000, disc_eq = 0, disc_lt = 0, ok_gt = 0;

  stat_init(&st, "receiver(S-S1,energy)", 4000);
  g_frame_slots = 0;
  g_frame_mode = 0;
  for (iter = 0; iter < total; iter++) {
    int lvl;
    build_state(o, 0, 0, -1, rr(2, 4));
    bU16(o, 0x122) = (uint16_t)(bU16(o, 0x122) & ~0x400u);
    bS16(o, 0x124) = (int16_t)(rr(0, 3) ? rr(0x69, 0x100) : rr(0x69, 0x7fff));
    bS16(o, 0x1c0) = (int16_t)rr(2, 8);
    /* energy ring: every class, including all-zero and the two extremes */
    switch (rr(0, 6)) {
      case 0: fill_samples((short *)OB(o, BO + 0x13c), 36, 0); break;
      case 1: fill_samples((short *)OB(o, BO + 0x13c), 36, 1); break;
      case 2: { int k; for (k = 0; k < 36; k++) ((short *)OB(o, BO + 0x13c))[k] = (short)(lcg_bool() ? 32767 : -32768); break; }
      case 3: fill_samples((short *)OB(o, BO + 0x13c), 36, 3); break;
      default: fill_samples((short *)OB(o, BO + 0x13c), 36, rr(2, 4)); break;
    }
    /* V34demodulate/agc may rewrite the ring during the call (the loop feeds the history ring through V34demodulate
     * when n > 0): the energy model therefore needs the ring as the check sees it - use n == 0 for the boundary
     * steering below, n > 0 elsewhere (the model then only classifies, the check compares against post-state). */
    if (rr(0, 2) == 0) bS16(o, 0x128) = 0;
    lvl = model_energy_level(o);
    if (bS16(o, 0x128) == 0) {
      switch (rr(0, 5)) { /* threshold steered onto the boundary of the (n == 0) prediction */
        case 0: I32(o, 0x230) = lvl; break;
        case 1: I32(o, 0x230) = lvl - 1; break;
        case 2: I32(o, 0x230) = lvl + 1; break;
        default: break;
      }
    }
    memcpy(w, o, OBJSZ);
    do_call(&st, w, o, (long)bU16(w, 0x122), (long)bS16(w, 0x124));
    if (bS16(o, 0x128) == 0 && (bU16(w, 0x122) & 0x400) && !(bU16(o, 0x122) & 0x400)) {
      if (lvl == I32(o, 0x230)) disc_eq++;
      else if (lvl < I32(o, 0x230)) disc_lt++;
      else ok_gt++;
    }
  }
  log_done(&st.lg);
  need_loop(&st, 0);
  need("S-S1 hit", st.ss1);
  need("energy: level == 0", st.ss1_sum0);
  need("energy: level == threshold (disconnect)", st.ss1_eq);
  need("energy: level below threshold (n == 0 steered)", disc_lt);
  need("energy: level above threshold (n == 0 steered)", ok_gt);
  need("energy: level == threshold (n == 0 steered)", disc_eq);
  need("energy: above threshold", st.ss1_ok);
  free(o);
  free(w);
}

/* ================================================================== scenario 5/6: frame mode */
static void scen7_frame(int dbg) {
  stat_t st;
  unsigned char *o = xalloc(OBJSZ), *w = xalloc(OBJSZ), *scratch = xalloc(OBJSZ), *spre = xalloc(OBJSZ);
  long iter, total = dbg ? 4000 : 24000, rec_calls = 0, shells = 0, fn_cnt[3] = {0, 0, 0};

  stat_init(&st, dbg ? "receiver(frame,debug)" : "receiver(frame)", dbg ? 1000000 : 3000);
  g_frame_slots = 1;
  g_frame_mode = 1;
  for (iter = 0; iter < total; iter++) {
    unsigned int snap, h, hs = 0;
    int eqc, tries;

    g_fnmode = (int)((iter / 1000) % 3);
    for (tries = 0;; tries++) {
      unsigned int lvl_saved = dsplibs_debug_level;
      int oob;
      if (tries > 400) die("cannot find an oracle-clean frame-mode receiver state");
      snap = g_lcg;
      eqc = rr(0, 5) < 4 ? rr(0, 1) : -1;
      build_state(scratch, 1, 1, -1, eqc);
      hs = rx_hash(scratch);
      memcpy(spre, scratch, OBJSZ);
      dsplibs_debug_level = 0;
      g_rec_on = 0;
      g_oob = 0;
      g_probe_taken = 0;
      
      V34ORC_receiver(scratch);
      oob = g_oob;
      if (!oob) {
        if (!g_probe_taken) die("oracle: the frame-mode state never reached decoderv34");
        oob = !orc_ok_decoder(g_probe);
      }
      dsplibs_debug_level = lvl_saved;
      if (!oob) break;
      
    }
    orc_commit();
    g_lcg = snap;
    eqc = rr(0, 5) < 4 ? rr(0, 1) : -1;
    build_state(o, 1, 1, -1, eqc);
    if (rx_hash(o) != hs) {
      size_t k2;
      for (k2 = 0; k2 < OBJSZ / 4; k2++) {
        uint32_t x, y;
        memcpy(&x, spre + 4 * k2, 4);
        memcpy(&y, o + 4 * k2, 4);
        if (x != y) fprintf(stderr, "  word 0x%zx: %08x vs %08x\n", 4 * k2, x, y);
      }
      die("frame scenario: the re-drawn state differs from the oracle-checked one");
    }
    memcpy(w, o, OBJSZ);
    if (dbg) {
      static const unsigned int lv[6] = {2u, 0u, 1u, 0x80000000u, 3u, 0xffffffffu};
      dsplibs_debug_level = lv[iter % 6];
    }
    if (g_fnmode == 0) {
      g_rec_h = FNV0;
      g_rec_n = 0;
      g_rec_obj = w;
      g_rec_on = 1;
    }
    h = do_call(&st, w, o, (long)bU16(w, 0x122), (long)bS16(w, 0x124));
    g_rec_on = 0;
    if (g_fnmode == 0) {
      /* fold the recorded descrambler calls into the per-call hash line (do_call already logged h) */
      st.lg.all = fnv32(fnv32(st.lg.all, g_rec_h), (uint32_t)g_rec_n);
      rec_calls += g_rec_n;
      if (g_rec_n) shells++;
    }
    fn_cnt[g_fnmode]++;
    (void)h;
  }
  dsplibs_debug_level = 0;
  log_done(&st.lg);
  printf("  oracle runs %ld, rejected %ld\n", g_orc_runs, g_orc_reject);
  need_loop(&st, 0);
  need("TRN calls", st.trn);
  need("decoderv34 frame mode ran (obj+0x47c == 0x2000)", st.frame_dec);
  need("decoder set flag 0x100 (reloaded by receiver)", st.f100_set);
  need("decoder cleared flag 0x100 (reloaded by receiver)", st.f100_clr);
  need("error accumulators reached, TRN", st.adapt);
  need("TRN call that skipped the accumulators (flags & 0x100)", st.tail100);
  need("error window published", st.win);
  need_retrain();
  if (dbg) need("print: decoderv34 V34RENEG (may be renegotiation)", g_pr_dec_reneg);
  need("descrambler recorder", fn_cnt[0]);
  need("real GPC", fn_cnt[1]);
  need("real GPA", fn_cnt[2]);
  need("descrambler calls recorded", rec_calls);
  need("calls that ran the shell/put tail", shells);
  free(o);
  free(w);
  free(scratch);
}

/* ================================================================== scenario 8: TimingV34's unguarded divisions */
/* The two idivs of TimingV34 (by (short)(b+0x1ce+1) and by (short)b+0x1be) are unguarded: a zero divisor traps
 * (SIGFPE) in the blob.  Such a state is never built by any producer (rxinit/SetupDemodulator set the symbol
 * constant, the window counter is small), so the differential scenarios keep both nonzero; this probe documents that
 * the blob traps and that the port traps the same way, in forked children (a crash of the real blob is thereby
 * observed, not silently excluded). */
static void probe_div(int which) {
  unsigned char *o = xalloc(OBJSZ);
  pid_t pid;
  int st = 0;

  build_state(o, 0, 0, -1, -1);
  bS16(o, 0x128) = 4;
  bU16(o, 0x1c0) = 2;      /* mode 2: no setInitialPhase, no limit logic below */
  bU16(o, 0x232) = 0xffff; /* window limit off */
  bS16(o, 0x1ec) = 1;
  bS16(o, 0x1ee) = 2;
  if (which == 0) {        /* divide by n == (short)(b+0x1ce + 1) == 0: 0xffff + 1, window 0 -> division path */
    bU16(o, 0x1ce) = 0xffff;
    bS16(o, 0x1d2) = 0;
    bU16(o, 0x1be) = 100;
  } else {                 /* divide by T == 0: n = 6 >= window 5 */
    bU16(o, 0x1ce) = 5;
    bS16(o, 0x1d2) = 5;
    bU16(o, 0x1be) = 0;
  }
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    receiver(o);
    _exit(0);
  }
  if (pid < 0) die("fork failed");
  waitpid(pid, &st, 0);
  if (WIFSIGNALED(st)) printf("TimingV34 zero-divisor probe %d: child killed by signal %d (SIGFPE=%d)\n", which, WTERMSIG(st), SIGFPE);
  else printf("TimingV34 zero-divisor probe %d: child exited normally with status %d\n", which, WEXITSTATUS(st));
  if (!(WIFSIGNALED(st) && WTERMSIG(st) == SIGFPE)) {
    fprintf(stderr, "zero-divisor probe %d: expected SIGFPE\n", which);
    g_fail = 1;
  }
  free(o);
}
static void scen_zerodiv(void) {
  g_frame_slots = 0;
  g_frame_mode = 0;
  probe_div(0);
  probe_div(1);
}

int main(int argc, char **argv) {
  int s = argc > 1 ? atoi(argv[1]) : -1;
  const char *v = getenv("V34T7_VERBOSE");
  if (v) g_verbose = atoi(v);
  if (getenv("V34T7_DUMP")) g_dump_at = atol(getenv("V34T7_DUMP"));
  g_dump_file = getenv("V34T7_DUMPFILE");
  V34I_REG(g_orc, OBJSZ); /* T06: the frame-scenario oracle (tier 4a's decoderv34 copy) runs on this scratch image through the shimmed oracle_* functions */
  if (s >= 9 && s <= 16) { g_t07 = 1; s -= 9; }
  tables_check();
  tables7_check();
  switch (s) {
    case 0: scen_single(0); break;
    case 1: scen_single(1); break;
    case 2: scen_chain(0, 0); break;
    case 3: scen_chain(1, 0); break;
    case 4: scen_chain(2, 0); scen_chain(2, 1); break;
    case 5: scen7_frame(0); break;
    case 6: scen7_frame(1); break;
    case 7: scen_ss1(); break;
    case 8: scen_zerodiv(); break;
    default: fprintf(stderr, "usage: %s <scenario 0-16>\n", argv[0]); return 1;
  }
  fflush(stdout);
  return g_fail ? 2 : 0;
}
