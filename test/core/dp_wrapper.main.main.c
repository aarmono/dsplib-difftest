/*
 * R02 differential harness, id r02_wrapper: dp_wrapper_create / dp_wrapper_run / dp_wrapper_delete, the real dsplibs.o (32-bit) vs src/core/dp_wrapper.c.
 * ONE source (no #ifdef around the code under test): the same typed objects (src/core/dp_glue.h: i386 layout at -m32, wider pointers on x86-64) are used on both sides and
 * the printed output is architecture independent, so the 32-bit BLOB run is the golden of the x86-64 build (test/golden/r02_wrapper_blob32.out.gz).
 *
 * Every callee is a recorder (test/r02_common.h): sysdep_malloc / free / memset / memcpy, RcFixed_Create / Delete / Resample (a deterministic fake: the wrapper only
 * needs a handle and a count), the engine callback (this file: h_cb, identity or a transform, scripted return values), dsplibs_debug_printf.  The wrapper blocks are
 * dumped through the typed struct (rings: count / wr / rd / half + a hash of the 384 samples; staging buffers; converters by Create index).
 *
 * Scenarios (argv[1]; each in its own process; exit 2 = a coverage counter is zero, 3 = harness / independent-model violation):
 *   0 create: parameter validation (frag / rate / engine rate = 0, frag > 0xc0, negative frag), the equal-rate path, every pair of {8000, 9600, 48000, 7777} x {...} x
 *     frag values (the 6 legal pairs create two converters, the rest NULL), RcFixed_Create failing at the first / second call (cleanup of one / both), sysdep_malloc
 *     failing, delete with / without converters, oversize blk (create only), the product wrap-around
 *   1 run, equal rates, identity callback: independent model out[k] = k < blk ? 0 : in[k - blk]; chunk sizes 1, blk - 1, blk, blk + 1, 2 blk - 1, 2 blk, 3 blk + 7, random;
 *     cnt = 0 and negative (no work), callback return values (last non-zero wins), several wrappers interleaved
 *   2 run, the six rate pairs x frag sets (blk <= 192) with the fake converters: odd chunk sizes, both ring wraps, a transform callback, callback returns
 *   3 run on pokes: ring positions / counts / halves set to arbitrary legal values, an asymmetric wrapper (rcIn or rcOut removed), blk 1 and 192, unequal rates whose
 *     engine block is not blk, the destination / source of every memcpy at every ring offset
 */
#include "../r02_common.h"

enum {
  CV_CR_EQ = 0, CV_CR_ZERO_FRAG, CV_CR_ZERO_RATE, CV_CR_ZERO_ENG, CV_CR_BIG_FRAG, CV_CR_NEG_FRAG, CV_CR_NOPAIR, CV_CR_MALLOC_FAIL, CV_CR_RC1_FAIL, CV_CR_RC2_FAIL, CV_CR_DEL_PLAIN, CV_CR_DEL_RC,
  CV_CR_BLK_BIG, CV_CR_WRAP,
  CV_PAIR = 20, /* 6 ids: create with converter idx 2 .. 7 */
  CV_RUN_EQ = 30, CV_RUN_MODEL, CV_RUN_CNT0, CV_RUN_CNTNEG, CV_RUN_CB_RET, CV_RUN_MULTI, CV_RUN_SPLIT, CV_RUN_FULL,
  CV_RUN_PAIR = 40, /* 6 ids: run processed blocks with converter idx 2 .. 7 */
  CV_RUN_XFORM = 50, CV_RUN_WRAP_IN, CV_RUN_WRAP_OUT, CV_RUN_CB_CNT_NE_BLK, CV_RUN_NO_RCIN, CV_RUN_NO_RCOUT, CV_RUN_BLK1, CV_RUN_BLK192, CV_RUN_POKE, CV_RUN_MULTIBLOCK, CV_RUN_HALF1
};

static struct dp g_dp[2];
static DpWrapper *g_w;

/* ------------------------------------------------------------------------------------------------ the engine callback */
static int g_cb_mode;          /* 0 = identity (out = in), 1 = out = 3 * in + call counter */
static unsigned g_cb_calls;
static unsigned long g_cb_ret_mask = 0; /* bit (call & 31) set: return a non-zero value for that call */
static int g_cb_last_n;
static int h_cb(struct dp *dp, void *in, void *out, int n)
{
  char di[96], dout[96];
  const short *s = (const short *)in;
  short *d = (short *)out;
  int j, r = 0;
  describe(in, di, sizeof di);
  describe(out, dout, sizeof dout);
  for (j = 0; j < n; j++) d[j] = g_cb_mode == 0 ? s[j] : (short)(s[j] * 3 + (int)g_cb_calls);
  if ((g_cb_ret_mask >> (g_cb_calls & 31)) & 1) r = (int)(g_cb_calls * 7u + 5u);
  emit(0, "CB dp=%d in=%s out=%s n=%d h=%016llx -> %d", dp == &g_dp[0] ? 0 : (dp == &g_dp[1] ? 1 : -1), di, dout, n, fnv_shorts(s, (size_t)n), r);
  g_cb_calls++;
  g_cb_last_n = n;
  return r;
}

/* ------------------------------------------------------------------------------------------------ dumps */
static void dump_ring(const char *tag, const DpRing *r)
{
  emit(0, "  %s count=%u wr=%u rd=%u half=%u data=%016llx", tag, r->count, r->wr, r->rd, r->half, fnv_shorts(r->data, DPW_RING));
}
static void dump_w(int show, const char *tag, const DpWrapper *w)
{
  if (!w) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s owner=%d cb=%d priv=%d rcIn=%d rcOut=%d blk=%u inbuf=%016llx outbuf=%016llx", tag, w->owner == &g_dp[0] ? 0 : (w->owner == &g_dp[1] ? 1 : (w->owner == NULL ? -1 : -2)),
       w->cb == h_cb, w->priv != NULL, rc_idx(w->rcIn), rc_idx(w->rcOut), w->blk, fnv_shorts(w->inbuf, 192),
       fnv_shorts(w->outbuf, 192));
  dump_ring("in ", &w->in);
  dump_ring("out", &w->out);
}

/* ------------------------------------------------------------------------------------------------ a run-call helper with checks */
static short g_in[4096], g_out[4096];
static int run_chunk(DpWrapper *w, struct dp *dp, int cnt, int show)
{
  int r, i;
  for (i = 0; i < cnt && i < 4096; i++) g_in[i] = (short)rnd();
  for (i = 0; i < 4096; i++) g_out[i] = 0x5a5a;
  reg_clear();
  reg_add(g_in, sizeof g_in, 2, "IN");
  reg_add(g_out, sizeof g_out, 2, "OUT");
  dp->dp_data = w;
  r = dp_wrapper_run(dp, g_in, g_out, cnt);
  for (i = cnt > 0 ? cnt : 0; i < 4096; i++)
    if (g_out[i] != 0x5a5a) die3("run wrote past the output (cnt=%d, index %d)", cnt, i);
  emit(show, "RUN cnt=%d -> %d out=%016llx", cnt, r, fnv_shorts(g_out, cnt > 0 ? (size_t)cnt : 0));
  return r;
}

/* ================================================================================================ scenario 0: create / delete */
static const int k_rates[5] = {8000, 9600, 48000, 7777, 11025};
static void sc_create(void)
{
  int a, b, f, i;
  static const int fr[] = {1, 7, 40, 160, 192, 193, 0x7fffffff, -1, -5};
  cov_def(CV_CR_EQ, "create: equal rates (no converters)");
  cov_def(CV_CR_ZERO_FRAG, "create: frag 0");
  cov_def(CV_CR_ZERO_RATE, "create: rate 0");
  cov_def(CV_CR_ZERO_ENG, "create: engine rate 0");
  cov_def(CV_CR_BIG_FRAG, "create: frag > 0xc0");
  cov_def(CV_CR_NEG_FRAG, "create: negative frag accepted");
  cov_def(CV_CR_NOPAIR, "create: no converter pair (NULL)");
  cov_def(CV_CR_MALLOC_FAIL, "create: sysdep_malloc fails");
  cov_def(CV_CR_RC1_FAIL, "create: first RcFixed_Create fails");
  cov_def(CV_CR_RC2_FAIL, "create: second RcFixed_Create fails");
  cov_def(CV_CR_DEL_PLAIN, "delete: no converters");
  cov_def(CV_CR_DEL_RC, "delete: two converters");
  cov_def(CV_CR_BLK_BIG, "create: blk > 192 (create only)");
  cov_def(CV_CR_WRAP, "create: frag * rate wraps 32 bits");
  for (i = 0; i < 6; i++) cov_defn(CV_PAIR + i, "create: converter idx", i + 2);
  arena_init();

  /* validation */
  g_dp[0].id = DP_V32;
  for (f = 0; f < 4; f++) {
    static const int args[4][3] = {{0, 9600, 8000}, {160, 0, 8000}, {160, 9600, 0}, {0xc1, 9600, 8000}};
    DpWrapper *w = dp_wrapper_create(&g_dp[0], h_cb, args[f][0], args[f][1], args[f][2]);
    emit(1, "invalid %d -> %s", f, w ? "WRAPPER" : "NULL");
    if (w) die3("invalid parameters created a wrapper");
    cov_hit(f == 0 ? CV_CR_ZERO_FRAG : (f == 1 ? CV_CR_ZERO_RATE : (f == 2 ? CV_CR_ZERO_ENG : CV_CR_BIG_FRAG)));
  }
  trial_end("validation");

  for (a = 0; a < 5; a++)
    for (b = 0; b < 5; b++)
      for (f = 0; f < (int)(sizeof fr / sizeof fr[0]); f++) {
        DpWrapper *w;
        char tag[64];
        rc_reset();
        w = dp_wrapper_create(&g_dp[a & 1], h_cb, fr[f], k_rates[a], k_rates[b]);
        snprintf(tag, sizeof tag, "create rate=%d eng=%d frag=%d", k_rates[a], k_rates[b], fr[f]);
        dump_w(1, tag, w);
        if (w) {
          if (a == b) cov_hit(CV_CR_EQ);
          if (w->rcIn) {
            int idx = rc_idx(w->rcIn);
            cov_hit(CV_PAIR + idx - 2);
            if (!w->rcOut) die3("rcIn without rcOut");
          }
          if (fr[f] < 0) cov_hit(CV_CR_NEG_FRAG);
          if (w->blk > 192) cov_hit(CV_CR_BLK_BIG);
          if (w->rcIn) cov_hit(CV_CR_DEL_RC);
          else cov_hit(CV_CR_DEL_PLAIN);
          dp_wrapper_delete(w);
        } else if (a != b && fr[f] > 0 && fr[f] <= 0xc0) {
          cov_hit(CV_CR_NOPAIR);
        }
        trial_end(tag);
      }

  /* 32-bit wrap-around of frag * rate and negative rates (equal rates: no converters, the wrapper is created with whatever blk the arithmetic gives) */
  {
    static const int wr[][3] = {{192, 0x01000001, 0x01000001}, {100, 0x7fffffff, 0x7fffffff}, {5, -4, -4}, {1, 1, 2}, {3, -7, 2}, {192, 0x01000001, 48000}};
    for (i = 0; i < 6; i++) {
      DpWrapper *w;
      char tag[64];
      rc_reset();
      w = dp_wrapper_create(&g_dp[0], h_cb, wr[i][0], wr[i][1], wr[i][2]);
      snprintf(tag, sizeof tag, "wrap frag=%d rate=%d eng=%d", wr[i][0], wr[i][1], wr[i][2]);
      dump_w(1, tag, w);
      if (w) {
        if (wr[i][1] == wr[i][2]) cov_hit(CV_CR_WRAP);
        dp_wrapper_delete(w);
      }
      trial_end(tag);
    }
  }
  /* RcFixed_Create failures: the legal pair (9600, 8000) fails at the first / the second converter; malloc failure first */
  for (f = 0; f < 3; f++) {
    DpWrapper *w;
    rc_reset();
    g_rc_fail_mask = f == 0 ? 1u : (f == 1 ? 2u : 0u);
    g_fail_malloc = f == 2 ? 0 : -1;
    w = dp_wrapper_create(&g_dp[0], h_cb, 160, 9600, 8000);
    dump_w(1, f == 0 ? "rc1 fails" : (f == 1 ? "rc2 fails" : "malloc fails"), w);
    if (w) die3("a failing callee still created the wrapper");
    cov_hit(f == 0 ? CV_CR_RC1_FAIL : (f == 1 ? CV_CR_RC2_FAIL : CV_CR_MALLOC_FAIL));
    trial_end("failures");
  }
  /* the same with another pair so that the idx of the failing delete is visible */
  rc_reset();
  g_rc_fail_mask = 2u;
  g_w = dp_wrapper_create(&g_dp[0], h_cb, 40, 48000, 8000);
  dump_w(1, "48000/8000 rc2 fails", g_w);
  trial_end("failures2");
  cov_finish();
}

/* ================================================================================================ scenario 1: equal rates */
static void run_equal(int blk, int *chunks, int nchunks, int model, int use_ret)
{
  DpWrapper *w;
  static short ref_in[1 << 16];
  size_t tin = 0, tout = 0;
  int i, k;
  rc_reset();
  w = dp_wrapper_create(&g_dp[0], h_cb, blk, 8000, 8000);
  if (!w) die3("create equal rates blk %d", blk);
  dump_w(0, "created", w);
  g_cb_mode = 0;
  g_cb_ret_mask = use_ret ? 0x92492492ul : 0;
  g_cb_calls = 0;
  for (i = 0; i < nchunks; i++) {
    int cnt = chunks[i], r;
    r = run_chunk(w, &g_dp[0], cnt, 0);
    if (cnt > 0 && tin + (size_t)cnt <= sizeof ref_in / 2) {
      memcpy(ref_in + tin, g_in, 2 * (size_t)cnt);
      tin += (size_t)cnt;
      if (model) {
        for (k = 0; k < cnt; k++) {
          short want = (tout + (size_t)k < (size_t)blk) ? 0 : ref_in[tout + (size_t)k - (size_t)blk];
          if (g_out[k] != want) die3("model: out[%zu] = %d, expected %d (blk %d)", tout + (size_t)k, g_out[k], want, blk);
        }
        cov_hit(CV_RUN_MODEL);
      }
      tout += (size_t)cnt;
    }
    if (cnt == 0) cov_hit(CV_RUN_CNT0);
    if (cnt < 0) cov_hit(CV_RUN_CNTNEG);
    if (r != 0) cov_hit(CV_RUN_CB_RET);
    if (cnt > blk) cov_hit(CV_RUN_MULTIBLOCK);
    if (cnt > 0 && cnt < blk) cov_hit(CV_RUN_SPLIT);
    if (cnt == blk) cov_hit(CV_RUN_FULL);
    if (w->in.half != 0) cov_hit(CV_RUN_HALF1);
    if (w->in.wr < (unsigned)cnt && cnt > 0) cov_hit(CV_RUN_WRAP_IN);
    if (w->out.rd < (unsigned)cnt && cnt > 0) cov_hit(CV_RUN_WRAP_OUT);
  }
  dump_w(1, "end", w);
  cov_hit(CV_RUN_EQ);
  dp_wrapper_delete(w);
  trial_end("equal");
}
static void sc_equal(void)
{
  static const int blks[] = {1, 2, 5, 40, 100, 160, 192};
  int bi, i;
  int chunks[64];
  cov_def(CV_RUN_EQ, "run equal: wrappers run");
  cov_def(CV_RUN_MODEL, "run equal: identity model checked");
  cov_def(CV_RUN_CNT0, "run: cnt == 0");
  cov_def(CV_RUN_CNTNEG, "run: cnt < 0");
  cov_def(CV_RUN_CB_RET, "run: callback return propagated");
  cov_def(CV_RUN_MULTI, "run: two wrappers interleaved");
  cov_def(CV_RUN_SPLIT, "run: chunk smaller than a block");
  cov_def(CV_RUN_FULL, "run: chunk == block");
  cov_def(CV_RUN_WRAP_IN, "run: input ring wrapped");
  cov_def(CV_RUN_WRAP_OUT, "run: output ring wrapped");
  cov_def(CV_RUN_MULTIBLOCK, "run: chunk larger than a block");
  cov_def(CV_RUN_HALF1, "run: second half in use");
  arena_init();
  for (bi = 0; bi < (int)(sizeof blks / sizeof blks[0]); bi++) {
    int blk = blks[bi], n = 0;
    chunks[n++] = 1;
    chunks[n++] = blk > 1 ? blk - 1 : 1;
    chunks[n++] = blk;
    chunks[n++] = blk + 1;
    chunks[n++] = 2 * blk - 1;
    chunks[n++] = 2 * blk;
    chunks[n++] = 3 * blk + 7;
    chunks[n++] = 0;
    chunks[n++] = -3;
    chunks[n++] = 1;
    for (i = 0; i < 30; i++) chunks[n++] = (int)rmod(blk * 3 + 5);
    run_equal(blk, chunks, n, 1, 0);
    run_equal(blk, chunks, n, 1, 1);
  }
  /* two wrappers alive at once, alternating calls */
  {
    DpWrapper *w1, *w2;
    int i2;
    rc_reset();
    w1 = dp_wrapper_create(&g_dp[0], h_cb, 40, 8000, 8000);
    w2 = dp_wrapper_create(&g_dp[1], h_cb, 17, 8000, 8000);
    g_cb_mode = 1;
    g_cb_ret_mask = 0;
    for (i2 = 0; i2 < 40; i2++) {
      run_chunk(w1, &g_dp[0], (int)rmod(90) + 1, 0);
      run_chunk(w2, &g_dp[1], (int)rmod(50) + 1, 0);
    }
    dump_w(1, "multi w1", w1);
    dump_w(1, "multi w2", w2);
    cov_hit(CV_RUN_MULTI);
    dp_wrapper_delete(w1);
    dp_wrapper_delete(w2);
    trial_end("multi");
  }
  cov_finish();
}

/* ================================================================================================ scenario 2: converters */
static void sc_pairs(void)
{
  struct pr {
    int rate, eng, nf, f[5];
  } pr[6] = {{8000, 9600, 4, {2, 6, 160, 192, 0}},   {9600, 8000, 4, {1, 5, 100, 160, 0}},  {8000, 48000, 4, {6, 30, 60, 192, 0}},
             {48000, 8000, 3, {1, 7, 32, 0, 0}},     {9600, 48000, 4, {5, 40, 100, 192, 0}}, {48000, 9600, 3, {1, 10, 38, 0, 0}}};
  int p, q, i, k;
  cov_def(CV_RUN_XFORM, "run: transform callback");
  cov_def(CV_RUN_CB_RET, "run: callback return propagated");
  cov_def(CV_RUN_WRAP_IN, "run: input ring wrapped");
  cov_def(CV_RUN_WRAP_OUT, "run: output ring wrapped");
  cov_def(CV_RUN_CB_CNT_NE_BLK, "run: engine block != blk");
  cov_def(CV_RUN_MULTIBLOCK, "run: chunk larger than a block");
  cov_def(CV_RUN_SPLIT, "run: chunk smaller than a block");
  for (i = 0; i < 6; i++) cov_defn(CV_RUN_PAIR + i, "run: converted blocks with idx", i + 2);
  arena_init();
  for (p = 0; p < 6; p++)
    for (q = 0; q < pr[p].nf; q++) {
      DpWrapper *w;
      int blk, n = 0, chunks[48];
      rc_reset();
      w = dp_wrapper_create(&g_dp[0], h_cb, pr[p].f[q], pr[p].rate, pr[p].eng);
      if (!w) die3("pair %d/%d frag %d", pr[p].rate, pr[p].eng, pr[p].f[q]);
      blk = (int)w->blk;
      if (blk > 192 || blk < 1) die3("harness: blk %d out of range", blk);
      dump_w(1, "pair", w);
      chunks[n++] = 1;
      chunks[n++] = blk;
      chunks[n++] = blk > 1 ? blk - 1 : 2;
      chunks[n++] = blk + 1;
      chunks[n++] = 2 * blk + 3;
      for (i = 0; i < 20; i++) chunks[n++] = (int)rmod(blk * 2 + 3) + 1;
      for (k = 0; k < 2; k++) {
        int idx = k == 0 ? 0 : 1;
        g_cb_mode = idx;
        g_cb_ret_mask = idx ? 0x11111111ul : 0;
        g_cb_calls = 0;
        for (i = 0; i < n; i++) {
          int r = run_chunk(w, &g_dp[0], chunks[i], 0), c = chunks[i];
          if (r) cov_hit(CV_RUN_CB_RET);
          if (c > blk) cov_hit(CV_RUN_MULTIBLOCK);
          if (c < blk) cov_hit(CV_RUN_SPLIT);
          if (w->in.wr < (unsigned)c) cov_hit(CV_RUN_WRAP_IN);
          if (w->out.rd < (unsigned)c) cov_hit(CV_RUN_WRAP_OUT);
        }
        if (idx) cov_hit(CV_RUN_XFORM);
        if (g_cb_calls > 0 && g_cb_last_n != blk) cov_hit(CV_RUN_CB_CNT_NE_BLK);
        cov_hit(CV_RUN_PAIR + rc_idx(w->rcIn) - 2);
      }
      dump_w(1, "pair end", w);
      dp_wrapper_delete(w);
      trial_end("pair");
    }
  cov_finish();
}

/* ================================================================================================ scenario 3: pokes */
static void poke_rings(DpWrapper *w)
{
  unsigned b2 = 2 * w->blk, i;
  w->in.wr = rmod(b2);
  w->out.rd = rmod(b2);
  w->in.count = rmod(w->blk + 1);
  w->out.count = rmod(b2 + 1);
  w->in.half = rpct(50) ? 0 : w->blk;
  w->out.half = rpct(50) ? 0 : w->blk;
  for (i = 0; i < DPW_RING; i++) {
    w->in.data[i] = (short)rnd();
    w->out.data[i] = (short)rnd();
  }
}
static void sc_pokes(void)
{
  static const struct {
    int rate, eng, frag;
  } cfgs[] = {{8000, 8000, 1},   {8000, 8000, 192},  {8000, 8000, 77},  {9600, 8000, 160}, {9600, 8000, 1},   {8000, 9600, 160}, {48000, 8000, 32},
              {8000, 48000, 192}, {9600, 48000, 100}, {48000, 9600, 38}, {9600, 8000, 5}};
  int c, t, i;
  cov_def(CV_RUN_NO_RCIN, "run: rcIn NULL, rcOut present");
  cov_def(CV_RUN_NO_RCOUT, "run: rcIn present, rcOut NULL");
  cov_def(CV_RUN_BLK1, "run: blk == 1");
  cov_def(CV_RUN_BLK192, "run: blk == 192");
  cov_def(CV_RUN_POKE, "run: poked ring state");
  cov_def(CV_RUN_WRAP_IN, "run: input ring wrapped");
  cov_def(CV_RUN_WRAP_OUT, "run: output ring wrapped");
  cov_def(CV_RUN_HALF1, "run: second half in use");
  cov_def(CV_RUN_CB_RET, "run: callback return propagated");
  arena_init();
  for (c = 0; c < (int)(sizeof cfgs / sizeof cfgs[0]); c++)
    for (t = 0; t < 12; t++) {
      DpWrapper *w;
      int blk;
      rc_reset();
      w = dp_wrapper_create(&g_dp[0], h_cb, cfgs[c].frag, cfgs[c].rate, cfgs[c].eng);
      if (!w) die3("poke cfg %d", c);
      blk = (int)w->blk;
      g_cb_mode = t & 1;
      g_cb_ret_mask = (t & 2) ? 0x5555ul : 0;
      g_cb_calls = 0;
      poke_rings(w);
      cov_hit(CV_RUN_POKE);
      if (t >= 8 && w->rcIn) {
        if (t & 1) {
          w->rcIn = NULL;
          cov_hit(CV_RUN_NO_RCIN);
        } else {
          w->rcOut = NULL;
          cov_hit(CV_RUN_NO_RCOUT);
        }
      }
      if (blk == 1) cov_hit(CV_RUN_BLK1);
      if (blk == 192) cov_hit(CV_RUN_BLK192);
      for (i = 0; i < 24; i++) {
        int cnt = (int)rmod((unsigned)(blk * 3 + 4)) + (i == 0 ? 1 : 0);
        int r = run_chunk(w, &g_dp[0], cnt, 0);
        if (r) cov_hit(CV_RUN_CB_RET);
        if (w->in.half) cov_hit(CV_RUN_HALF1);
        if (cnt > 0 && w->in.wr < (unsigned)cnt) cov_hit(CV_RUN_WRAP_IN);
        if (cnt > 0 && w->out.rd < (unsigned)cnt) cov_hit(CV_RUN_WRAP_OUT);
      }
      dump_w(1, "poke end", w);
      dp_wrapper_delete(w);
      trial_end("poke");
    }
  cov_finish();
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  g_verbose = getenv("R02_VERBOSE") != NULL;
  g_lcg = 0x2545f491u + 1000003u * (unsigned)sc;
  if (getenv("R02_SEED")) g_lcg ^= (unsigned)atoi(getenv("R02_SEED"));
  g_dp[0].id = DP_V32;
  g_dp[1].id = DP_V22;
  switch (sc) {
    case 0: sc_create(); break;
    case 1: sc_equal(); break;
    case 2: sc_pairs(); break;
    case 3: sc_pokes(); break;
    default: die3("no such scenario %d", sc);
  }
  return 0;
}
