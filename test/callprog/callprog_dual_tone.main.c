/*
 * R11 (id r11_tone): differential test of the call-progress DSP helpers: TONE_read, FP_Pow, _iir_filter_create / _progress / _delete, Dual_TONE_create / _detect / _delete
 * (the real dsplibs.o vs src/callprog/callprog_dual_tone.c + src/callprog/callprog_tables.c).  Built twice from this one source (-DTEST_BLOB for the 32-bit blob run); each scenario runs in its own process:
 *   0  TONE_read: every low word (and garbage in the upper half) + FP_Pow: a dense sweep -0x20000 .. 0x20000 and 300,000 random 32-bit inputs, with structural checks
 *   1  the IIR pre-filter: creation (NULL / dirty object, nA / nB 0 .. 25, random coefficient / scale sets; the blob does not check malloc, so no failure injection), progress on random / sine / DC / extreme / zero input
 *      for chunk sizes 0 .. 200 (the whole object, 0xdc bytes, is hashed after every call), delete (NULL and a block)
 *   2  Dual_TONE: create (malloc failure), detect on silence, noise, 2100 Hz and 2250 Hz bursts of every length class, both tones, other tones, chunk sizes 0 .. 1000 (all state
 *      hashed after every call), directed decision tests (poked energies / ratio / thr / counters) reaching every result 0 .. 5, long runs for the 0x4ff / 0x500 sample counters
 *   3  the same IIR / Dual_TONE objects driven through the real CALLPROG coefficient set (a = _filter_a_coef ...) with a long mixed signal
 * Output: hashed lines (every call logs its result and object hash); required coverage counters exit 2 on a zero, 3 = harness violation.  Env R11_SEED, R11_VERBOSE=1.
 */
#include "r11_common.h"

extern int TONE_read(int);
extern int FP_Pow(int);

static const short k_a[12] = {8192, -9150, 3671, 8192, -6770, 5246, 8192, -15686, 7643, 2787, 1540, 2787};   /* the CALLPROG a / b / scale sets (copied from the blob rodata) */
static const short k_b[12] = {8192, 13289, 8192, 8192, -16379, 8192, 8192, 3322, 8192, 16384, -16182, 7781};
static const short k_s[5] = {5, 0, 0, 0, 0};

static u64 hash_obj(const void *p, size_t n) { return fnv_bytes(p, n); }
static void dirty(void *p, size_t n)
{
  unsigned char *b = (unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) b[i] = (unsigned char)rnd();
}

enum { C_TR0, C_TR1, C_TR2, C_TR3, C_FPA, C_FPB, C_FPZ, C_FPFULL, C_IC0, C_IC1, C_ICN, C_IP, C_ISC, C_ID0, C_ID1, C_DC, C_DCF, C_DR0, C_DR1, C_DR2, C_DR3, C_DR4, C_DR5, C_DD, C_DWRAP, C_DN0 };

static void s0(void)
{
  int i;
  unsigned x;
  u64 h = 0;
  /* structural checks of the table */
  if (TONE_read(0) != 16384 || TONE_read(0x200) != 0 || TONE_read(0x400) != -16384 || TONE_read(0x600) != 0) die3("TONE_read does not look like a cosine");
  for (i = -0x10000; i <= 0x10000; i++) {
    int r = TONE_read(i);
    unsigned v = (unsigned)(short)i & 0x7ffu;
    if (v <= 0x200) cov_hit(C_TR0); else if (v <= 0x400) cov_hit(C_TR1); else if (v <= 0x600) cov_hit(C_TR2); else cov_hit(C_TR3);
    h = fnv_mix(h, (unsigned)r);
    if ((i & 0xfff) == 0xfff) emit(0, "TR %d %016llx", i, h);
  }
  for (i = 0; i < 20000; i++) {
    int idx = (int)rnd();
    h = fnv_mix(h, (unsigned)TONE_read(idx));
  }
  emit(1, "TONE_read %016llx", h);
  h = 0;
  for (i = -0x20000; i <= 0x20000; i++) {
    int r = FP_Pow(i);
    if ((unsigned)i + 0x3fffu <= 0x7ffeu) cov_hit(C_FPA); else cov_hit(C_FPB);
    if (i == 0) cov_hit(C_FPZ);
    h = fnv_mix(h, (unsigned)r);
    if ((i & 0x3fff) == 0x3fff) emit(0, "FP %d %016llx", i, h);
  }
  for (i = 0; i < 300000; i++) {
    int xv;
    switch (i % 4) {
    case 0: x = rnd(); break;
    case 1: x = (unsigned)(int)(short)rnd(); break;
    case 2: x = (unsigned)((int)(rnd() & 0x7ffffu) - 0x40000); break;
    default: x = rnd() & 0x1fffu ? (unsigned)((int)(rnd() % 0x8000u) - 0x4000) : 0x80000000u; break;
    }
    xv = (int)x;
    h = fnv_mix(h, (unsigned)FP_Pow(xv));
    cov_hit(C_FPFULL);
  }
  emit(1, "FP_Pow %016llx", h);
  /* edge inputs */
  {
    static const int e[] = {0x3ffe, 0x3fff, 0x4000, -0x3fff, -0x4000, -0x4001, 0x7fffffff, (int)0x80000000, -1, 1, 16383, 16384, -16384, 0x10000, -0x10000};
    for (i = 0; i < (int)(sizeof e / sizeof e[0]); i++) emit(1, "FPE %d -> %d", e[i], FP_Pow(e[i]));
  }
}

static void iir_dump(const char *tag, const IirFilter *f, int res) { emit(0, "%s %d %016llx", tag, res, hash_obj(f, sizeof(IirFilter))); }

static void fill_signal(short *b, int n, int kind, Osc *o)
{
  int i;
  for (i = 0; i < n; i++) {
    switch (kind) {
    case 0: b[i] = (short)rnd(); break;
    case 1: b[i] = (short)osc_next(o); break;
    case 2: b[i] = 1234; break;
    case 3: b[i] = (short)((i & 1) ? 32767 : -32768); break;
    case 4: b[i] = 0; break;
    case 5: b[i] = (short)(osc_next(o) / 8 + (int)(rnd() & 0x3ff) - 512); break;
    default: b[i] = (short)(rnd() & 0xf) - 8; break;
    }
  }
}

static void s1(void)
{
  int t, k;
  for (t = 0; t < 400; t++) {
    IirFilter dirtyobj;
    IirFilter *f;
    short a[26], b[26], sc[5];
    int nA = (t % 3 == 0) ? 12 : rrange(0, 25), nB = (t % 3 == 0) ? 12 : rrange(0, 25);
    int useobj = rpct(40);
    Osc o;
    dirty(&dirtyobj, sizeof dirtyobj);
    for (k = 0; k < 26; k++) {
      a[k] = (t % 4 == 0) ? k_a[k % 12] : (short)rnd();
      b[k] = (t % 4 == 0) ? k_b[k % 12] : (short)rnd();
    }
    for (k = 0; k < 5; k++) sc[k] = (t % 4 == 0) ? k_s[k] : (short)(rnd() & 0x3f) - (short)(rnd() & 7);
    if (t % 5 == 4) sc[4] = (short)rrange(-3, 40);
    f = useobj ? &dirtyobj : NULL;
    f = _iir_filter_create(f, nA, nB, a, b, sc);
    cov_hit(useobj ? C_IC1 : C_IC0);
    if (!f) die3("create returned NULL");
    if (nA != 12 || nB != 12) cov_hit(C_ICN);
    iir_dump("IC", f, nA);
    for (k = 0; k < 14; k++) {
      short buf[200], ref[200];
      int n = (k < 3) ? k : rrange(0, 200);
      int kind = rrange(0, 6);
      osc_init(&o, osc_c(rrange(1, 600) * 5, 8000), 20000);
      fill_signal(buf, 200, kind, &o);
      memcpy(ref, buf, sizeof buf);
      _iir_filter_progress(f, n, buf);
      cov_hit(C_IP);
      if (n > 0 && memcmp(&buf[n], &ref[n], (size_t)(200 - n) * sizeof(short)) != 0) die3("_iir_filter_progress wrote past n");
      if (sc[4] > 0) cov_hit(C_ISC);
      emit(0, "IP n=%d kind=%d out=%016llx obj=%016llx", n, kind, fnv_bytes(buf, (size_t)n * sizeof(short)), hash_obj(f, sizeof(IirFilter)));
    }
    if (useobj) {
      emit(0, "IDEL skipped (caller object)");
    } else {
      _iir_filter_delete(f);
      cov_hit(C_ID1);
    }
    _iir_filter_delete(NULL);
    cov_hit(C_ID0);
    trial_end("iir");
  }
  emit(1, "IIR done");
}

static void dt_dump(const char *tag, const DualTone *d, int res) { emit(0, "%s %d %016llx", tag, res, hash_obj(d, sizeof(DualTone))); }

static void s2(void)
{
  int t, k;
  for (t = 0; t < 150; t++) {
    DualTone *d;
    short buf[1100];
    Osc o1, o2;
    if (rpct(5)) g_fail_malloc = 0;
    d = Dual_TONE_create();
    if (!d) {
      cov_hit(C_DC);
      emit(0, "DCREATE NULL");
      trial_end("dual");
      continue;
    }
    cov_hit(C_DCF);
    dt_dump("DC", d, 0);
    for (k = 0; k < 40; k++) {
      int n, kind = rrange(0, 7), i, r;
      switch (rrange(0, 4)) { case 0: n = rrange(0, 4); break; case 1: n = rrange(5, 170); break; case 2: n = rrange(171, 1000); break; case 3: n = 160; break; default: n = rrange(0, 1000); }
      osc_init(&o1, osc_c(2100, 8000), 4000 + (int)rmod(26000));
      osc_init(&o2, osc_c(2250, 8000), 4000 + (int)rmod(26000));
      for (i = 0; i < n; i++) {
        switch (kind) {
        case 0: buf[i] = 0; break;
        case 1: buf[i] = (short)rnd(); break;
        case 2: case 3: buf[i] = (short)osc_next(&o1); break;
        case 4: buf[i] = (short)osc_next(&o2); break;
        case 5: buf[i] = (short)((osc_next(&o1) + osc_next(&o2)) / 2); break;
        case 6: buf[i] = (short)(osc_next(&o1) / 4 + (int)(rnd() & 0x7ff) - 1024); break;
        default: buf[i] = (short)(osc_c(rrange(300, 3400), 8000) / 2); break;
        }
      }
      r = Dual_TONE_detect(d, buf, n);
      if (r >= 0 && r <= 5) cov_hit(C_DR0 + r); else die3("Dual_TONE_detect returned %d", r);
      dt_dump("DD", d, r);
      cov_hit(C_DD);
      emit(0, "DD n=%d kind=%d res=%d", n, kind, r);
    }
    /* long runs: the 0x4ff / 0x500 sample counters (results 2 / 3 / 4 / 5) */
    for (k = 0; k < 14; k++) {
      int i, r, which = rrange(0, 2);
      osc_init(&o1, osc_c(which == 1 ? 2250 : 2100, 8000), 12000);
      for (i = 0; i < 12; i++) {
        int j, n = rrange(100, 400);
        for (j = 0; j < n; j++) buf[j] = (short)osc_next(&o1);
        r = Dual_TONE_detect(d, buf, n);
        if (r >= 0 && r <= 5) cov_hit(C_DR0 + r); else die3("Dual_TONE_detect returned %d", r);
        dt_dump("DL", d, r);
      }
    }
    /* directed decisions: poke the state, one empty / one-sample call */
    for (k = 0; k < 120; k++) {
      int r;
      short one = (short)(rnd() & 0xf);
      d->e[0] = (unsigned)rmod(rpct(50) ? 100 : 400000);
      d->e[1] = (unsigned)rmod(rpct(50) ? 100 : 400000);
      d->e[2] = (unsigned)rmod(rpct(50) ? 100 : 400000);
      d->ratio = (short)(rpct(70) ? 0xe2 : rrange(-300, 600));
      d->thr = (short)(rpct(70) ? 1 : rrange(-5, 500));
      d->n1 = (short)(rpct(50) ? rrange(0x4fd, 0x502) : rrange(0, 0x900));
      d->n2 = (short)(rpct(50) ? rrange(0x4fd, 0x502) : rrange(0, 0x900));
      if (rpct(10)) d->n1 = (short)rnd();
      if (rpct(50)) { /* the notch filters' delay lines are normally equal pairs (g[0] == g[8], g[1] == g[9]): poke them apart, and the pre-filter states */
        int q;
        for (q = 0; q < 12; q++) d->g[q] = (short)(rpct(50) ? (int)rnd() : rrange(-300, 300));
        for (q = 0; q < 12; q++) d->st[q / 4][q % 4] = (short)(rpct(50) ? (int)rnd() : rrange(-300, 300));
      }
      {
        short in3[3];
        int nn = rpct(30) ? 0 : rrange(1, 3), q;
        for (q = 0; q < 3; q++) in3[q] = (short)(rpct(50) ? (int)rnd() : rrange(-2000, 2000));
        r = Dual_TONE_detect(d, nn ? in3 : &one, nn);
      }
      if (r >= 0 && r <= 5) cov_hit(C_DR0 + r); else die3("Dual_TONE_detect returned %d", r);
      dt_dump("DP", d, r);
    }
    /* the sample counters are shorts: a count that wraps */
    d->n1 = 32760;
    d->n2 = 32760;
    {
      short z[20] = {0};
      int r = Dual_TONE_detect(d, z, 20);
      dt_dump("DW", d, r);
      cov_hit(C_DWRAP);
    }
    Dual_TONE_delete(d);
    trial_end("dual");
  }
  emit(1, "DUAL done");
}

static void s3(void)
{
  /* the IIR filter and the dual-tone detector in the roles CALLPROG gives them: the real coefficient set, a long mixed call-progress-like signal */
  IirFilter *f = _iir_filter_create(NULL, 12, 12, k_a, k_b, k_s);
  DualTone *d = Dual_TONE_create();
  Osc o1, o2;
  int seg;
  if (!f || !d) die3("create failed");
  for (seg = 0; seg < 400; seg++) {
    short buf[160], c[160];
    int n = rrange(1, 160), i, kind = rrange(0, 5), r;
    osc_init(&o1, osc_c(rpct(50) ? 2100 : 2250, 8000), 9000 + (int)rmod(20000));
    osc_init(&o2, osc_c(rrange(300, 700), 8000), 6000);
    for (i = 0; i < n; i++) {
      switch (kind) {
      case 0: buf[i] = (short)(osc_next(&o1)); break;
      case 1: buf[i] = (short)((osc_next(&o2) + osc_next(&o1) / 4) / 2); break;
      case 2: buf[i] = (short)(rnd() & 0x3ff) - 512; break;
      case 3: buf[i] = 0; break;
      default: buf[i] = (short)osc_next(&o2); break;
      }
    }
    memcpy(c, buf, sizeof buf);
    _iir_filter_progress(f, n, c);
    r = Dual_TONE_detect(d, buf, n);
    emit(0, "MIX n=%d kind=%d filt=%016llx res=%d f=%016llx d=%016llx", n, kind, fnv_bytes(c, (size_t)n * sizeof(short)), r, hash_obj(f, sizeof *f), hash_obj(d, sizeof *d));
    if (r >= 0 && r <= 5) cov_hit(C_DR0 + r);
  }
  _iir_filter_delete(f);
  Dual_TONE_delete(d);
  trial_end("mix");
  emit(1, "MIX done");
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R11_SEED");
  g_verbose = getenv("R11_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)atoi(sd) * 2654435761u;
  g_lcg += (unsigned)sc * 0x9e3779b9u;
  cov_def(C_TR0, "TONE_read 0 .. 0x200"); cov_def(C_TR1, "TONE_read 0x201 .. 0x400"); cov_def(C_TR2, "TONE_read 0x401 .. 0x600"); cov_def(C_TR3, "TONE_read 0x601 .. 0x7ff");
  cov_def(C_FPA, "FP_Pow in-range variant"); cov_def(C_FPB, "FP_Pow shifted variant"); cov_def(C_FPZ, "FP_Pow(0)"); cov_def(C_FPFULL, "FP_Pow random 32-bit");
  cov_def(C_IC0, "_iir_filter_create allocating"); cov_def(C_IC1, "_iir_filter_create caller object"); 
  cov_def(C_ICN, "_iir_filter_create non-standard counts"); cov_def(C_IP, "_iir_filter_progress"); cov_def(C_ISC, "_iir_filter_progress final scale > 0");
  cov_def(C_ID0, "_iir_filter_delete NULL"); cov_def(C_ID1, "_iir_filter_delete block");
  cov_def(C_DC, "Dual_TONE_create fails"); cov_def(C_DCF, "Dual_TONE_create"); cov_def(C_DD, "Dual_TONE_detect");
  cov_def(C_DR0, "Dual result 0"); cov_def(C_DR1, "Dual result 1"); cov_def(C_DR2, "Dual result 2"); cov_def(C_DR3, "Dual result 3"); cov_def(C_DR4, "Dual result 4"); cov_def(C_DR5, "Dual result 5");
  cov_def(C_DWRAP, "Dual sample counter wrap");
  switch (sc) {
  case 0: cov_def(C_IC0, 0); cov_def(C_IC1, 0); cov_def(C_ICN, 0); cov_def(C_IP, 0); cov_def(C_ISC, 0); cov_def(C_ID0, 0); cov_def(C_ID1, 0);
          cov_def(C_DC, 0); cov_def(C_DCF, 0); cov_def(C_DD, 0); cov_def(C_DR0, 0); cov_def(C_DR1, 0); cov_def(C_DR2, 0); cov_def(C_DR3, 0); cov_def(C_DR4, 0); cov_def(C_DR5, 0); cov_def(C_DWRAP, 0); s0(); break;
  case 1: cov_def(C_TR0, 0); cov_def(C_TR1, 0); cov_def(C_TR2, 0); cov_def(C_TR3, 0); cov_def(C_FPA, 0); cov_def(C_FPB, 0); cov_def(C_FPZ, 0); cov_def(C_FPFULL, 0);
          cov_def(C_DC, 0); cov_def(C_DCF, 0); cov_def(C_DD, 0); cov_def(C_DR0, 0); cov_def(C_DR1, 0); cov_def(C_DR2, 0); cov_def(C_DR3, 0); cov_def(C_DR4, 0); cov_def(C_DR5, 0); cov_def(C_DWRAP, 0); s1(); break;
  case 2: cov_def(C_TR0, 0); cov_def(C_TR1, 0); cov_def(C_TR2, 0); cov_def(C_TR3, 0); cov_def(C_FPA, 0); cov_def(C_FPB, 0); cov_def(C_FPZ, 0); cov_def(C_FPFULL, 0);
          cov_def(C_IC0, 0); cov_def(C_IC1, 0); cov_def(C_ICN, 0); cov_def(C_IP, 0); cov_def(C_ISC, 0); cov_def(C_ID0, 0); cov_def(C_ID1, 0); s2(); break;
  default: cov_def(C_TR0, 0); cov_def(C_TR1, 0); cov_def(C_TR2, 0); cov_def(C_TR3, 0); cov_def(C_FPA, 0); cov_def(C_FPB, 0); cov_def(C_FPZ, 0); cov_def(C_FPFULL, 0);
          cov_def(C_IC0, 0); cov_def(C_IC1, 0); cov_def(C_ICN, 0); cov_def(C_IP, 0); cov_def(C_ISC, 0); cov_def(C_ID0, 0); cov_def(C_ID1, 0);
          cov_def(C_DC, 0); cov_def(C_DCF, 0); cov_def(C_DD, 0); cov_def(C_DR1, 0); cov_def(C_DR2, 0); cov_def(C_DR3, 0); cov_def(C_DR4, 0); cov_def(C_DR5, 0); cov_def(C_DWRAP, 0); cov_def(C_DR0, 0); s3(); break;
  }
  cov_finish();
  return 0;
}
