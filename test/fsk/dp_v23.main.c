/*
 * R02 differential harness, id r02_v23: the V.23 dp glue (v23_create / v23_process / v23_delete = blob LOCAL functions, dp_v23_init / dp_v23_exit), the real dsplibs.o
 * (32-bit) vs src/fsk/dp_v23.c + src/core/dp_wrapper.c.  Same method as test/fsk/dp_v21_b103.main.c (read its header): the LOCAL functions are reached through the record
 * dp_v23_init registers and through the wrapper's callback slot, the real dp_wrapper runs on each side, every other callee is a recorder; the engine here is
 * CreateV23Modem / V23ModemMain / DeleteV23Modem (the recorder records the config block, every argument role, the tx-bit and sample hashes and writes scripted
 * outputs: the new tx-bit request, the rx bits, the audio, the status).
 *
 * Scenarios (argv[1]; own process each; exit 2 = a coverage counter is zero, 3 = harness violation):
 *   0 registration: dp_v23_init / dp_v23_exit
 *   1 create / delete lifecycle: id x caller x srate (9600 / 8000 / 7777 no pair / 48000 oversize, create only), debug levels, failures (glue / wrapper malloc, RcFixed,
 *     CreateV23Modem NULL), the config block {tone = caller == 0, 8000, 700} and rxBackend = caller == 0
 *   2 v23_process called directly (fabricated dp objects, modem / status from dp, rates from the glue): connected 0 / 1, want < 0 / 0 / 1 .. 100, get_bits counts, engine
 *     result 0 / 1 / 2 / negative / large, rx counts negative / 0 / 1 .. 100, caller 0 / 1 / 2 (rates 75 / 1200), dp status before, debug levels
 *   3 end to end: op->process = dp_wrapper_run with random chunk sizes at 9600 and 8000 Hz, an engine result script (connect, data, hangup, error, reconnect)
 */
#include "../r02_common.h"

enum {
  CV_REG_V23 = 0, CV_DEREG_V23, CV_OPS_PROC_IS_RUN, CV_OPS_NAME,
  CV_CR_V23 = 10, CV_CR_OTHER_ID, CV_CR_CALLER0, CV_CR_CALLER1, CV_CR_9600, CV_CR_8000, CV_CR_NOPAIR, CV_CR_MALLOC1, CV_CR_MALLOC2, CV_CR_RC_FAIL, CV_CR_ENG_FAIL, CV_CR_BIG,
  CV_DBG0, CV_DBG1, CV_DBG2, CV_DELETE, CV_CR_CFG_ORIG, CV_CR_CFG_ANS,
  CV_R_0 = 30, CV_R_1, CV_R_OTHER, CV_R_NEG, CV_CONN0, CV_CONN1, CV_WANT_NEG, CV_WANT_0, CV_WANT_POS, CV_GB_0, CV_GB_FULL, CV_GB_PART, CV_RXNEG, CV_RX0, CV_RXN,
  CV_FIRST_CONNECT, CV_RATE_75, CV_RATE_1200, CV_STAT_CHANGE, CV_STAT_SAME, CV_ALIAS_DP, CV_DPST_0, CV_DPST_1, CV_DPST_4, CV_RUN_E2E, CV_E2E_CONNECT, CV_E2E_DROP, CV_E2E_ERR, CV_E2E_BITS
};

/* ------------------------------------------------------------------------------------------------ the engine recorders */
typedef struct {
  int idx;
  int live;
} FakeEng;
static FakeEng g_eng[8];
static int g_neng, g_eng_fail;
static unsigned g_eng_calls;
static int g_eng_want_out = 6, g_eng_rx_out;    /* scripted *txBitCount / *nRxBitsOut results */
static int g_eng_ret = 0;                       /* scripted return value of the next V23ModemMain */
static int (*g_eng_ret_fn)(void);
static int eng_id(const void *p) { return (int)(((const FakeEng *)p) - g_eng); }

V23Modem *CreateV23Modem(V23Modem *self, short rxBackend, const V23Cfg *cfg)
{
  int i;
  int rate = cfg->rate, thr = cfg->thr;
  emit(0, "ENG create self=%s rxBackend=%d tone=%d rate=%d thr=%d", self ? "X" : "NULL", (int)rxBackend, (int)cfg->tone, rate, thr);
  if (g_eng_fail) {
    emit(0, "ENG create -> NULL");
    return NULL;
  }
  if (g_neng >= 8) die3("engine pool");
  i = g_neng++;
  g_eng[i].idx = i;
  g_eng[i].live = 1;
  emit(0, "ENG create -> e%d", i);
  return (V23Modem *)(void *)&g_eng[i];
}
void DeleteV23Modem(V23Modem *self)
{
  FakeEng *e = (FakeEng *)(void *)self;
  if (!e->live) die3("engine deleted twice");
  e->live = 0;
  emit(0, "ENG delete e%d", eng_id(e));
}
int V23ModemMain(V23Modem *self, int *txBitSrc, int *txBitCount, short *txAudioOut, int txSampleCount, short *rxAudioIn, int rxSampleCount, unsigned int *rxBitsOut, int *nRxBitsOut)
{
  FakeEng *e = (FakeEng *)(void *)self;
  char dtx[96], dtxo[96], drxi[96], drxo[96];
  int i, want = *txBitCount, ntx = want < 0 ? 0 : (want < 100 ? want : 100), ret;
  if (!e->live) die3("engine used after delete");
  describe(txBitSrc, dtx, sizeof dtx);
  describe(txAudioOut, dtxo, sizeof dtxo);
  describe(rxAudioIn, drxi, sizeof drxi);
  describe(rxBitsOut, drxo, sizeof drxo);
  ret = g_eng_ret_fn ? g_eng_ret_fn() : g_eng_ret;
  for (i = 0; i < txSampleCount; i++) txAudioOut[i] = (short)(rxAudioIn[i] + (short)(g_eng_calls * 11u));
  for (i = 0; i < g_eng_rx_out && i < 100; i++) rxBitsOut[i] = rnd();
  emit(0, "ENG main e%d tx=%s out=%s in=%s rx=%s txs=%d rxs=%d want=%d nrx=%d txh=%016llx inh=%016llx -> want=%d nrx=%d ret=%d", eng_id(e), dtx, dtxo, drxi, drxo, txSampleCount, rxSampleCount, want,
       *nRxBitsOut, fnv_ints(txBitSrc, (size_t)ntx), fnv_shorts(rxAudioIn, rxSampleCount > 0 ? (size_t)rxSampleCount : 0), g_eng_want_out, g_eng_rx_out, ret);
  *txBitCount = g_eng_want_out;
  *nRxBitsOut = g_eng_rx_out;
  g_eng_calls++;
  return ret;
}
static void eng_reset(void)
{
  g_neng = 0;
  g_eng_fail = 0;
  memset(g_eng, 0, sizeof g_eng);
}

/* ------------------------------------------------------------------------------------------------ helpers */
static struct dp_operations *g_ops;
static const char *slot_name(const void *fn) { return fn == (const void *)dp_wrapper_run ? "dp_wrapper_run" : (fn ? "other" : "NULL"); }

static void dump_glue(int show, const char *tag, struct dp *dp)
{
  V23Glue *g = (V23Glue *)dp;
  if (!dp) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s id=%d modem=m%d status=%u op=%s dp_data=%s caller=%d want=%d connected=%d eng=%d wrapper=%s tx=%016llx rx=%016llx", tag, (int)dp->id, modem_no(dp->modem), dp->status,
       dp->op == g_ops ? "ops" : (dp->op == NULL ? "NULL" : "other"), dp->dp_data == (void *)g->wrapper ? "wrapper" : (dp->dp_data == NULL ? "NULL" : "other"), g->caller, g->want,
       g->connected, g->eng ? eng_id(g->eng) : -1, g->wrapper ? "set" : "NULL", fnv_ints(g->txbits, 100), fnv_ints(g->rxbits, 100));
}
static void init_ops(void)
{
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  dp_v23_init();
  if (g_nreg_op != 1) die3("dp_v23_init registered %d", g_nreg_op);
  g_ops = g_reg_op[0];
}
static short g_in[1024], g_out[1024];
static void set_regions(void)
{
  reg_clear();
  reg_add(g_in, sizeof g_in, 2, "IN");
  reg_add(g_out, sizeof g_out, 2, "OUT");
}
static void fill_in(int n)
{
  int i;
  for (i = 0; i < n && i < 1024; i++) g_in[i] = (short)rnd();
  for (i = 0; i < 1024; i++) g_out[i] = 0x5a5a;
}
static struct dp *do_create(int mi, int id, int caller, int srate)
{
  struct dp *d;
  set_regions();
  d = g_ops->create(M_(mi), (enum DP_ID)id, caller, srate, 160, g_ops);
  return d;
}

/* ================================================================================================ scenario 0: registration */
static void sc_reg(void)
{
  int r, i;
  cov_def(CV_REG_V23, "dp_v23_init: DP_V23 registered");
  cov_def(CV_DEREG_V23, "dp_v23_exit: DP_V23 deregistered");
  cov_def(CV_OPS_PROC_IS_RUN, "record: process slot is dp_wrapper_run");
  cov_def(CV_OPS_NAME, "record: name v23, use_count 0, hangup NULL");
  arena_init();
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  r = dp_v23_init();
  emit(1, "dp_v23_init -> %d, %d registrations", r, g_nreg_op);
  for (i = 0; i < g_nreg_op; i++) emit(1, "registered id %d", g_reg_id[i]);
  if (g_nreg_op == 1) {
    if (g_reg_id[0] == DP_V23) cov_hit(CV_REG_V23);
    if (g_reg_op[0]->process == dp_wrapper_run) cov_hit(CV_OPS_PROC_IS_RUN);
    if (strcmp(g_reg_op[0]->name, "v23") == 0 && g_reg_op[0]->use_count == 0 && g_reg_op[0]->hangup == NULL && g_reg_op[0]->create && g_reg_op[0]->delete) cov_hit(CV_OPS_NAME);
  }
  dp_v23_exit();
  cov_hit(CV_DEREG_V23);
  cov_finish();
}

/* ================================================================================================ scenario 1: create / delete */
static void sc_lifecycle(void)
{
  static const int ids[3] = {DP_V23, 0, DP_B103};
  static const int callers[3] = {0, 1, 2};
  static const int rates[4] = {9600, 8000, 7777, 48000};
  int a, b, c, n = 0, f;
  cov_def(CV_CR_V23, "create: id DP_V23");
  cov_def(CV_CR_OTHER_ID, "create: other id");
  cov_def(CV_CR_CALLER0, "create: caller 0 (originate: tone, backend 1)");
  cov_def(CV_CR_CALLER1, "create: caller non-zero (answer: no tone, backend 0)");
  cov_def(CV_CR_9600, "create: srate 9600 (converters)");
  cov_def(CV_CR_8000, "create: srate 8000 (no converters)");
  cov_def(CV_CR_NOPAIR, "create: srate without a pair (NULL)");
  cov_def(CV_CR_MALLOC1, "create: glue malloc fails");
  cov_def(CV_CR_MALLOC2, "create: wrapper malloc fails");
  cov_def(CV_CR_RC_FAIL, "create: RcFixed_Create fails");
  cov_def(CV_CR_ENG_FAIL, "create: CreateV23Modem fails");
  cov_def(CV_CR_BIG, "create: srate 48000 (blk 960, create only)");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  cov_def(CV_DELETE, "delete via op->delete");
  arena_init();
  init_ops();
  for (a = 0; a < 3; a++)
    for (b = 0; b < 3; b++)
      for (c = 0; c < 4; c++) {
        struct dp *d;
        char tag[80];
        int level = n % 5;
        n++;
        dsplibs_debug_level = lvl_value(level);
        cov_hit(level == 0 ? CV_DBG0 : (level == 1 ? CV_DBG1 : CV_DBG2));
        rc_reset();
        eng_reset();
        d = do_create(n & 1, ids[a], callers[b], rates[c]);
        snprintf(tag, sizeof tag, "create id=%d caller=%d srate=%d level=%d", ids[a], callers[b], rates[c], level);
        dump_glue(1, tag, d);
        if (ids[a] == DP_V23) cov_hit(CV_CR_V23);
        else cov_hit(CV_CR_OTHER_ID);
        if (callers[b] == 0) cov_hit(CV_CR_CALLER0);
        else cov_hit(CV_CR_CALLER1);
        if (!d) {
          cov_hit(CV_CR_NOPAIR);
        } else {
          int r;
          if (rates[c] == 9600) cov_hit(CV_CR_9600);
          if (rates[c] == 8000) cov_hit(CV_CR_8000);
          if (rates[c] == 48000) cov_hit(CV_CR_BIG);
          r = g_ops->delete(d);
          emit(1, "delete -> %d", r);
          cov_hit(CV_DELETE);
        }
        trial_end(tag);
      }
  dsplibs_debug_level = 2;
  for (f = 0; f < 5; f++) {
    struct dp *d;
    char tag[80];
    rc_reset();
    eng_reset();
    g_fail_malloc = f == 0 ? 0 : (f == 1 ? 1 : -1);
    g_rc_fail_mask = f == 2 ? 1u : 0u;
    g_eng_fail = f == 3;
    d = do_create(0, DP_V23, 0, f == 4 ? 7777 : 9600);
    snprintf(tag, sizeof tag, "failure %d", f);
    dump_glue(1, tag, d);
    if (d) die3("a failing callee still created the glue (%d)", f);
    if (f == 0) cov_hit(CV_CR_MALLOC1);
    if (f == 1) cov_hit(CV_CR_MALLOC2);
    if (f == 2) cov_hit(CV_CR_RC_FAIL);
    if (f == 3) cov_hit(CV_CR_ENG_FAIL);
    trial_end(tag);
  }
  cov_finish();
}

/* ================================================================================================ scenario 2: process called directly */
static const int k_ret_list[] = {0, 0, 0, 1, 2, 3, -1, 5, 0x100, 0x7fffffff, (int)0x80000000u, 4};
static void sc_process(void)
{
  int t, it;
  struct dp alias;
  cov_def(CV_R_0, "engine result 0");
  cov_def(CV_R_1, "engine result 1");
  cov_def(CV_R_OTHER, "engine result >= 2");
  cov_def(CV_R_NEG, "engine result negative");
  cov_def(CV_CONN0, "connected 0 before");
  cov_def(CV_CONN1, "connected 1 before");
  cov_def(CV_WANT_NEG, "want < 0");
  cov_def(CV_WANT_0, "want == 0");
  cov_def(CV_WANT_POS, "want > 0 (bits fetched when connected)");
  cov_def(CV_GB_0, "get_bits returns 0");
  cov_def(CV_GB_FULL, "get_bits returns all requested");
  cov_def(CV_GB_PART, "get_bits returns fewer");
  cov_def(CV_RXNEG, "rx count negative");
  cov_def(CV_RX0, "rx count 0");
  cov_def(CV_RXN, "rx bits put (connected, count > 0)");
  cov_def(CV_FIRST_CONNECT, "result 0 with connected 0: connect (rates set)");
  cov_def(CV_RATE_75, "rate 75 (caller 0)");
  cov_def(CV_RATE_1200, "rate 1200 (caller != 0)");
  cov_def(CV_STAT_CHANGE, "dp status changes (V23STAT print at level 2)");
  cov_def(CV_STAT_SAME, "dp status unchanged");
  cov_def(CV_ALIAS_DP, "dp != glue (modem read from dp)");
  cov_def(CV_DPST_0, "dp status 0 before");
  cov_def(CV_DPST_1, "dp status 1 before");
  cov_def(CV_DPST_4, "dp status 4 before");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  for (t = 0; t < 6; t++) {
    struct dp *d;
    V23Glue *g;
    DpWrapper *w;
    rc_reset();
    eng_reset();
    d = do_create(0, DP_V23, t % 3, t & 1 ? 9600 : 8000);
    if (!d) die3("create");
    g = (V23Glue *)d;
    w = g->wrapper;
    for (it = 0; it < 260; it++) {
      struct dp *dp = &g->dp;
      int ret, lvl = (int)rmod(5);
      int r = k_ret_list[rmod((unsigned)(sizeof k_ret_list / sizeof k_ret_list[0]))];
      static const int wants[] = {0, 6, 1, 3, 100, -2, 8, 6};
      static const unsigned dst[] = {0, 1, 4, 1, 0};
      unsigned long nsp0 = g_nsp, npb0 = g_npb, ngb0 = g_ngb;
      unsigned st0;
      dsplibs_debug_level = lvl_value(lvl);
      cov_hit(lvl == 0 ? CV_DBG0 : (lvl == 1 ? CV_DBG1 : CV_DBG2));
      g->want = wants[rmod(8)];
      g->connected = rpct(55);
      if (rpct(15)) g->caller = (int)rmod(3);
      g->dp.status = dst[rmod(5)];
      if (rpct(50)) {
        alias = g->dp;
        alias.modem = M_(1 + (int)rmod(2));
        alias.status = dst[rmod(5)];
        alias.dp_data = w;
        dp = &alias;
        cov_hit(CV_ALIAS_DP);
      }
      st0 = dp->status;
      g_eng_ret = r;
      g->txbits[99] = (int)rnd();
      g->txbits[98] = (int)rnd();
      g_eng_want_out = rpct(20) ? -1 : (int)rmod(9);
      g_eng_rx_out = rpct(15) ? -3 : (rpct(30) ? 0 : (int)rmod(101));
      if (rpct(8)) g_eng_rx_out = 100;
      g_bits_give = (int)rmod(8);
      fill_in(160);
      set_regions();
      if (r == 0) cov_hit(CV_R_0);
      else if (r == 1) cov_hit(CV_R_1);
      else if (r < 0) cov_hit(CV_R_NEG);
      else cov_hit(CV_R_OTHER);
      cov_hit(g->connected ? CV_CONN1 : CV_CONN0);
      if (g->want < 0) cov_hit(CV_WANT_NEG);
      else if (g->want == 0) cov_hit(CV_WANT_0);
      else cov_hit(CV_WANT_POS);
      if (g->connected && g->want > 0) {
        if (g_bits_give == 0) cov_hit(CV_GB_0);
        else if (g_bits_give >= g->want) cov_hit(CV_GB_FULL);
        else cov_hit(CV_GB_PART);
      }
      if (g_eng_rx_out < 0) cov_hit(CV_RXNEG);
      else if (g_eng_rx_out == 0) cov_hit(CV_RX0);
      if (st0 == 0) cov_hit(CV_DPST_0);
      else if (st0 == 1) cov_hit(CV_DPST_1);
      else cov_hit(CV_DPST_4);
      {
        int conn0 = g->connected;
        ret = w->cb(dp, g_in, g_out, 160);
        if (g_npb > npb0) cov_hit(CV_RXN);
        if (r == 0 && !conn0) {
          cov_hit(CV_FIRST_CONNECT);
          if (g_nsp > nsp0) cov_hit(g->caller == 0 ? CV_RATE_75 : CV_RATE_1200);
        }
        if ((unsigned)ret != st0) cov_hit(CV_STAT_CHANGE);
        else cov_hit(CV_STAT_SAME);
      }
      (void)ngb0;
      emit(0, "process -> %d out=%016llx", ret, fnv_shorts(g_out, 160));
      dump_glue(0, "after", &g->dp);
      emit(0, "dp status %u", dp->status);
    }
    dump_glue(1, "process end", d);
    emit(1, "delete -> %d", g_ops->delete(d));
    trial_end("process");
  }
  cov_finish();
}

/* ================================================================================================ scenario 3: end to end */
static unsigned g_script_pos;
static int script_ret(void)
{
  /* result 0 = running, 1 = the engine reports "no carrier / hangup": a plausible session: connect, data, hangup, again, an error, reconnect */
  unsigned p = g_script_pos++ % 260u;
  if (p < 100) return 0;
  if (p < 108) return 1;
  if (p < 180) return 0;
  if (p < 184) return 3;
  if (p < 190) return 1;
  return 0;
}
static void sc_e2e(void)
{
  static const int rates[2] = {9600, 8000};
  int t, it;
  cov_def(CV_RUN_E2E, "end to end: op->process = dp_wrapper_run");
  cov_def(CV_E2E_CONNECT, "end to end: connect (status 1)");
  cov_def(CV_E2E_DROP, "end to end: result 1 after connect");
  cov_def(CV_E2E_ERR, "end to end: status 4");
  cov_def(CV_E2E_BITS, "end to end: tx bits fetched and rx bits put");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  dsplibs_debug_level = 2;
  cov_hit(CV_DBG2);
  for (t = 0; t < 4; t++) {
    struct dp *d;
    V23Glue *g;
    rc_reset();
    eng_reset();
    g_eng_calls = 0;
    d = do_create(t & 1, DP_V23, t & 1, rates[t & 1]);
    if (!d) die3("create e2e");
    g = (V23Glue *)d;
    g_script_pos = (unsigned)t * 23u;
    g_eng_ret_fn = script_ret;
    g_eng_rx_out = 3;
    g_eng_want_out = 6;
    g_bits_give = 6;
    for (it = 0; it < 90; it++) {
      int cnt = rpct(25) ? (int)rmod(40) + 1 : (int)rmod(450) + 100, r;
      unsigned long npb0 = g_npb, ngb0 = g_ngb;
      int conn0 = g->connected;
      if (rpct(5)) cnt = 0;
      fill_in(cnt);
      set_regions();
      r = g_ops->process(d, g_in, g_out, cnt);
      emit(0, "e2e cnt=%d -> %d out=%016llx", cnt, r, fnv_shorts(g_out, cnt > 0 ? (size_t)cnt : 0));
      cov_hit(CV_RUN_E2E);
      if (r == 1) cov_hit(CV_E2E_CONNECT);
      if (r == 4) cov_hit(CV_E2E_ERR);
      if (conn0 && !g->connected && g->dp.status == 0) cov_hit(CV_E2E_DROP);
      if (g_npb > npb0 && g_ngb > ngb0) cov_hit(CV_E2E_BITS);
      if (it % 10 == 0) dump_glue(0, "e2e", d);
    }
    dump_glue(1, "e2e end", d);
    emit(1, "delete -> %d", g_ops->delete(d));
    trial_end("e2e");
    g_eng_ret_fn = NULL;
  }
  cov_finish();
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  g_verbose = getenv("R02_VERBOSE") != NULL;
  g_lcg = 0x2545f491u + 1000003u * (unsigned)sc;
  if (getenv("R02_SEED")) g_lcg ^= (unsigned)atoi(getenv("R02_SEED"));
  switch (sc) {
    case 0: sc_reg(); break;
    case 1: sc_lifecycle(); break;
    case 2: sc_process(); break;
    case 3: sc_e2e(); break;
    default: die3("no such scenario %d", sc);
  }
  return 0;
}
