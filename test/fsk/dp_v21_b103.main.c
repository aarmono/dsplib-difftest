/*
 * R02 differential harness, id r02_b103: the Bell 103 / V.21 dp glue (b103_create / b103_process / b103_delete = blob LOCAL functions, dp_b103_init / dp_b103_exit),
 * the real dsplibs.o (32-bit) vs src/fsk/dp_v21_b103.c + src/core/dp_wrapper.c.  One source, no #ifdef around the code under test; the typed objects (src/core/dp_glue.h: i386 layout at
 * -m32, native `struct dp` / wider pointers on x86-64) are the same on both sides and every printed line is architecture independent, so the 32-bit BLOB run is the
 * golden of the x86-64 build (test/golden/r02_b103_blob32.out.gz).
 *
 * The LOCAL functions are reached the way slmodemd reaches them: through the record dp_b103_init registers (the modem_dp_register recorder captures it; this is also a
 * check of the record: name, use_count, slots).  The glue runs over the REAL dp_wrapper (blob's vs port's, linked on both sides); every other callee is a recorder
 * (test/r02_common.h): sysdep_*, modem_get_bits / put_bits / set_param, dsplibs_debug_printf, RcFixed_* (fake) and the engine B103FP_create / B103FP_modem / B103FP_delete
 * (this file: records cfg / buffer roles / counts / input hashes, writes scripted outputs, returns scripted engine statuses).
 *
 * Scenarios (argv[1]; own process each; exit 2 = a coverage counter is zero, 3 = harness violation):
 *   0 registration: dp_b103_init / dp_b103_exit (ids, record fields)
 *   1 create / delete lifecycle: id x caller x srate (9600, 8000 = equal rates, 48000 = oversize blk (create only), 7777 = no converter pair), debug levels 0 / 1 / 2,
 *     sysdep_malloc failing at the glue / the wrapper, RcFixed_Create failing, B103FP_create returning NULL (cleanup), delete order and dp_data cleared
 *   2 b103_process called directly through the wrapper's callback slot with fabricated dp objects (dp != the glue: modem / status are read from `dp`, the rates from the
 *     glue): every engine status class (low byte 0, 1 .. 4, 5 .. 6, 7, 8 .. 255, high bits set), state 0 / 6 / other, get_bits counts 0 .. n, rx counts 0 .. 100, dp status
 *     0 / 1 / 4 before the call, last-status equal / different, debug levels 0 / 1 / 2
 *   3 end to end: slmodemd's calls (op->create, op->process = dp_wrapper_run with random chunk sizes, op->delete) at 9600 and 8000 Hz with an engine status script that
 *     reaches the linked state and drops out again
 */
#include "../r02_common.h"

enum {
  CV_REG_B103 = 0, CV_REG_V21, CV_DEREG_B103, CV_DEREG_V21, CV_OPS_PROC_IS_RUN, CV_OPS_NAME,
  CV_CR_B103 = 10, CV_CR_V21, CV_CR_OTHER_ID, CV_CR_CALLER0, CV_CR_CALLER1, CV_CR_9600, CV_CR_8000, CV_CR_NOPAIR, CV_CR_MALLOC1, CV_CR_MALLOC2, CV_CR_RC_FAIL, CV_CR_ENG_FAIL, CV_CR_BIG,
  CV_DBG0, CV_DBG1, CV_DBG2, CV_DELETE,
  CV_ST_0 = 30, CV_ST_1_4, CV_ST_5_6, CV_ST_7, CV_ST_HI, CV_ST_HIGHBITS, CV_STATE0, CV_STATE6, CV_STATE_OTHER, CV_GB_0, CV_GB_FULL, CV_GB_PART, CV_RX0, CV_RXN, CV_RX_STATE0,
  CV_LAST_EQ, CV_LAST_NE, CV_DPST_0, CV_DPST_1, CV_DPST_4, CV_LINK_NEW, CV_LINK_AGAIN, CV_ERR_ST, CV_ALIAS_DP, CV_RATES_SET, CV_RUN_E2E, CV_E2E_LINKED, CV_E2E_DROP, CV_CR_V21_TXT
};

/* ------------------------------------------------------------------------------------------------ the engine recorders */
typedef struct {
  int idx;
  int live;
} FakeEng;
static FakeEng g_eng[8];
static int g_neng, g_eng_fail;
static unsigned g_eng_calls;
static int g_eng_tx_out = 6, g_eng_rx_out;      /* scripted *txCount / *rxCount results */
static unsigned g_eng_status = 0;               /* scripted return value of the next B103FP_modem */
static unsigned (*g_eng_status_fn)(void);
static int eng_id(const void *p) { return (int)(((const FakeEng *)p) - g_eng); }

B103Obj *B103FP_create(B103Obj *self, int *cfg)
{
  int i;
  emit(0, "ENG create self=%s cfg=%d,%d,%d,%d,%d,%d,%d", self ? "X" : "NULL", cfg[0], cfg[1], cfg[2], cfg[3], cfg[4], cfg[5], cfg[6]);
  if (g_eng_fail) {
    emit(0, "ENG create -> NULL");
    return NULL;
  }
  if (g_neng >= 8) die3("engine pool");
  i = g_neng++;
  g_eng[i].idx = i;
  g_eng[i].live = 1;
  emit(0, "ENG create -> e%d", i);
  return (B103Obj *)(void *)&g_eng[i];
}
void B103FP_delete(B103Obj *self)
{
  FakeEng *e = (FakeEng *)(void *)self;
  if (!e->live) die3("engine deleted twice");
  e->live = 0;
  emit(0, "ENG delete e%d", eng_id(e));
}
unsigned int B103FP_modem(B103Obj *self, int *txBits, short *txOut, short *rxIn, int *rxOut, unsigned short *txCount, unsigned short *rxCount)
{
  FakeEng *e = (FakeEng *)(void *)self;
  char dtx[96], dtxo[96], drxi[96], drxo[96];
  unsigned short txin = *txCount, rxin = *rxCount;
  int i, ntx = txin < 100 ? txin : 100;
  short *out = txOut;
  const short *rxi = rxIn;
  int *rxo = rxOut;
  unsigned st;
  if (!e->live) die3("engine used after delete");
  describe(txBits, dtx, sizeof dtx);
  describe(txOut, dtxo, sizeof dtxo);
  describe(rxIn, drxi, sizeof drxi);
  describe(rxOut, drxo, sizeof drxo);
  st = g_eng_status_fn ? g_eng_status_fn() : g_eng_status;
  /* the audio the engine "transmits": a function of the receive samples and the call counter; the rx bits it "demodulated": random ints (all 32 bits) */
  for (i = 0; i < (int)rxin; i++) out[i] = (short)(rxi[i] + (short)(g_eng_calls * 11u));
  for (i = 0; i < g_eng_rx_out && i < 100; i++) rxo[i] = (int)rnd();
  emit(0, "ENG modem e%d tx=%s txout=%s rxin=%s rxout=%s txin=%u rxin=%u txh=%016llx rxh=%016llx -> txout=%d rxout=%d st=%08x", eng_id(e), dtx, dtxo, drxi, drxo, txin, rxin,
       fnv_ints(txBits, (size_t)ntx), fnv_shorts(rxi, rxin), g_eng_tx_out, g_eng_rx_out, st);
  *txCount = (unsigned short)g_eng_tx_out;
  *rxCount = (unsigned short)g_eng_rx_out;
  g_eng_calls++;
  return st;
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
  B103Glue *g = (B103Glue *)dp;
  if (!dp) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s id=%d modem=m%d status=%u op=%s dp_data=%s caller=%d state=%d last=%d eng=%d wrapper=%s tx=%016llx rx=%016llx", tag, (int)dp->id, modem_no(dp->modem), dp->status,
       dp->op == g_ops ? "ops" : (dp->op == NULL ? "NULL" : "other"), dp->dp_data == (void *)g->wrapper ? "wrapper" : (dp->dp_data == NULL ? "NULL" : "other"), g->caller, g->state,
       g->last, g->eng ? eng_id(g->eng) : -1, g->wrapper ? "set" : "NULL", fnv_ints(g->txbits, 100), fnv_ints(g->rxbits, 100));
}
static void init_ops(void)
{
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  dp_b103_init();
  if (g_nreg_op != 2 || g_reg_op[0] != g_reg_op[1]) die3("dp_b103_init registered %d records", g_nreg_op);
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
  cov_def(CV_REG_B103, "dp_b103_init: DP_B103 registered");
  cov_def(CV_REG_V21, "dp_b103_init: DP_V21 registered");
  cov_def(CV_DEREG_B103, "dp_b103_exit: DP_B103 deregistered");
  cov_def(CV_DEREG_V21, "dp_b103_exit: DP_V21 deregistered");
  cov_def(CV_OPS_PROC_IS_RUN, "record: process slot is dp_wrapper_run");
  cov_def(CV_OPS_NAME, "record: name b103, use_count 0, hangup NULL");
  arena_init();
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  r = dp_b103_init();
  emit(1, "dp_b103_init -> %d, %d registrations", r, g_nreg_op);
  for (i = 0; i < g_nreg_op; i++) emit(1, "registered id %d", g_reg_id[i]);
  if (g_nreg_op == 2) {
    if (g_reg_id[0] == DP_B103) cov_hit(CV_REG_B103);
    if (g_reg_id[1] == DP_V21) cov_hit(CV_REG_V21);
    if (g_reg_op[0]->process == dp_wrapper_run) cov_hit(CV_OPS_PROC_IS_RUN);
    if (strcmp(g_reg_op[0]->name, "b103") == 0 && g_reg_op[0]->use_count == 0 && g_reg_op[0]->hangup == NULL && g_reg_op[0]->create && g_reg_op[0]->delete) cov_hit(CV_OPS_NAME);
    emit(1, "same record: %d", g_reg_op[0] == g_reg_op[1]);
  }
  /* exit: deregister recorder sees the same record for both ids (the order is in the hashed log) */
  g_cov[CV_DEREG_B103] = g_cov[CV_DEREG_V21] = 0;
  dp_b103_exit();
  cov_hit(CV_DEREG_B103);
  cov_hit(CV_DEREG_V21);
  cov_finish();
}

/* ================================================================================================ scenario 1: create / delete */
static void sc_lifecycle(void)
{
  static const int ids[4] = {DP_B103, DP_V21, 0, DP_V22};
  static const int callers[3] = {0, 1, 2};
  static const int rates[4] = {9600, 8000, 7777, 48000};
  int a, b, c, n = 0, f;
  cov_def(CV_CR_B103, "create: id DP_B103");
  cov_def(CV_CR_V21, "create: id DP_V21");
  cov_def(CV_CR_OTHER_ID, "create: other id");
  cov_def(CV_CR_CALLER0, "create: caller 0");
  cov_def(CV_CR_CALLER1, "create: caller non-zero");
  cov_def(CV_CR_9600, "create: srate 9600 (converters)");
  cov_def(CV_CR_8000, "create: srate 8000 (no converters)");
  cov_def(CV_CR_NOPAIR, "create: srate without a pair (NULL)");
  cov_def(CV_CR_MALLOC1, "create: glue malloc fails");
  cov_def(CV_CR_MALLOC2, "create: wrapper malloc fails");
  cov_def(CV_CR_RC_FAIL, "create: RcFixed_Create fails");
  cov_def(CV_CR_ENG_FAIL, "create: B103FP_create fails");
  cov_def(CV_CR_BIG, "create: srate 48000 (blk 960, create only)");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  cov_def(CV_DELETE, "delete via op->delete");
  cov_def(CV_CR_V21_TXT, "create: debug config print with V.21 / Bell103 name");
  arena_init();
  init_ops();
  for (a = 0; a < 4; a++)
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
        if (ids[a] == DP_B103) cov_hit(CV_CR_B103);
        else if (ids[a] == DP_V21) cov_hit(CV_CR_V21);
        else cov_hit(CV_CR_OTHER_ID);
        if (callers[b] == 0) cov_hit(CV_CR_CALLER0);
        else cov_hit(CV_CR_CALLER1);
        if (!d) {
          cov_hit(CV_CR_NOPAIR);
        } else {
          if (rates[c] == 9600) cov_hit(CV_CR_9600);
          if (rates[c] == 8000) cov_hit(CV_CR_8000);
          if (rates[c] == 48000) cov_hit(CV_CR_BIG);
          if (level >= 2 && (ids[a] == DP_V21 || ids[a] == DP_B103)) cov_hit(CV_CR_V21_TXT);
          {
            int r = g_ops->delete(d);
            emit(1, "delete -> %d", r);
            cov_hit(CV_DELETE);
          }
        }
        trial_end(tag);
      }
  dsplibs_debug_level = 2;
  for (f = 0; f < 6; f++) {
    struct dp *d;
    char tag[80];
    rc_reset();
    eng_reset();
    g_fail_malloc = f == 0 ? 0 : (f == 1 ? 1 : -1);
    g_rc_fail_mask = f == 2 ? 1u : (f == 3 ? 2u : 0u);
    g_eng_fail = f == 4;
    d = do_create(0, DP_B103, 0, f == 5 ? 7777 : 9600);
    snprintf(tag, sizeof tag, "failure %d", f);
    dump_glue(1, tag, d);
    if (d) die3("a failing callee still created the glue (%d)", f);
    if (f == 0) cov_hit(CV_CR_MALLOC1);
    if (f == 1) cov_hit(CV_CR_MALLOC2);
    if (f == 2 || f == 3) cov_hit(CV_CR_RC_FAIL);
    if (f == 4) cov_hit(CV_CR_ENG_FAIL);
    trial_end(tag);
  }
  cov_finish();
}

/* ================================================================================================ scenario 2: process called directly */
static const unsigned k_status_list[] = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 0x7fu, 0xffu, 0x100u, 0x107u, 0x106u, 0x205u, 0xffffff00u, 0xffffff07u, 0x80000001u};
static int g_cl_state, g_cl_lowclass;
static void sc_process(void)
{
  int t, it, k;
  struct dp alias;
  cov_def(CV_ST_0, "engine status low byte 0");
  cov_def(CV_ST_1_4, "engine status low byte 1..4");
  cov_def(CV_ST_5_6, "engine status low byte 5..6");
  cov_def(CV_ST_7, "engine status low byte 7");
  cov_def(CV_ST_HI, "engine status low byte >= 8");
  cov_def(CV_ST_HIGHBITS, "engine status with bits above the low byte");
  cov_def(CV_STATE0, "glue state 0");
  cov_def(CV_STATE6, "glue state 6");
  cov_def(CV_STATE_OTHER, "glue state other (1..5)");
  cov_def(CV_GB_0, "get_bits returns 0");
  cov_def(CV_GB_FULL, "get_bits returns all requested");
  cov_def(CV_GB_PART, "get_bits returns fewer");
  cov_def(CV_RX0, "rx count 0 with state != 0");
  cov_def(CV_RXN, "rx bits put (state != 0, count > 0)");
  cov_def(CV_RX_STATE0, "rx count > 0 with state 0 (not put)");
  cov_def(CV_LAST_EQ, "status equals the last status");
  cov_def(CV_LAST_NE, "status differs from the last status");
  cov_def(CV_DPST_0, "dp status 0 before");
  cov_def(CV_DPST_1, "dp status 1 before");
  cov_def(CV_DPST_4, "dp status 4 before");
  cov_def(CV_LINK_NEW, "linked: first time (rates set, state 6)");
  cov_def(CV_LINK_AGAIN, "linked: again (dp status already 1)");
  cov_def(CV_ERR_ST, "error status 4 returned");
  cov_def(CV_ALIAS_DP, "dp != glue (modem read from dp)");
  cov_def(CV_RATES_SET, "set_param calls (rates 300)");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  for (t = 0; t < 8; t++) {
    struct dp *d;
    B103Glue *g;
    DpWrapper *w;
    rc_reset();
    eng_reset();
    d = do_create(0, t & 1 ? DP_V21 : DP_B103, t & 2 ? 1 : 0, t & 4 ? 9600 : 8000);
    if (!d) die3("create");
    g = (B103Glue *)d;
    w = g->wrapper;
    for (it = 0; it < 220; it++) {
      struct dp *dp = &g->dp;
      int ret, lvl = (int)rmod(5);
      unsigned st = k_status_list[rmod((unsigned)(sizeof k_status_list / sizeof k_status_list[0]))];
      static const int states[] = {0, 6, 6, 3, 1, 2, 5, 4};
      static const unsigned dst[] = {0, 1, 4, 1, 0};
      dsplibs_debug_level = lvl_value(lvl);
      cov_hit(lvl == 0 ? CV_DBG0 : (lvl == 1 ? CV_DBG1 : CV_DBG2));
      g->state = states[rmod(8)];
      if (rpct(50)) g->last = (int)st;
      else g->last = (int)rnd();
      g->dp.status = dst[rmod(5)];
      if (rpct(50)) { /* a different dp object: its own modem and status; dp_data reaches the same wrapper */
        alias = g->dp;
        alias.modem = M_(1 + (int)rmod(2));
        alias.status = dst[rmod(5)];
        alias.dp_data = w;
        dp = &alias;
        cov_hit(CV_ALIAS_DP);
      }
      g_eng_status = st;
      g->txbits[99] = (int)rnd();
      g->txbits[98] = (int)rnd();
      g_eng_tx_out = (int)rmod(8);
      g_eng_rx_out = rpct(35) ? 0 : (int)rmod(101);
      if (rpct(10)) g_eng_rx_out = 100;
      g_bits_give = (int)rmod(8);
      fill_in(160);
      set_regions();
      if ((st & 0xff) == 0) cov_hit(CV_ST_0);
      else if ((st & 0xff) <= 4) cov_hit(CV_ST_1_4);
      else if ((st & 0xff) <= 6) cov_hit(CV_ST_5_6);
      else if ((st & 0xff) == 7) cov_hit(CV_ST_7);
      else cov_hit(CV_ST_HI);
      if (st > 0xff) cov_hit(CV_ST_HIGHBITS);
      if (g->state == 0) cov_hit(CV_STATE0);
      else if (g->state == 6) cov_hit(CV_STATE6);
      else cov_hit(CV_STATE_OTHER);
      if ((int)st == g->last) cov_hit(CV_LAST_EQ);
      else cov_hit(CV_LAST_NE);
      if (dp->status == 0) cov_hit(CV_DPST_0);
      else if (dp->status == 1) cov_hit(CV_DPST_1);
      else cov_hit(CV_DPST_4);
      if (g->state != 0) {
        if (g_bits_give == 0) cov_hit(CV_GB_0);
        else if (g_bits_give >= g->state) cov_hit(CV_GB_FULL);
        else cov_hit(CV_GB_PART);
        if (g_eng_rx_out == 0) cov_hit(CV_RX0);
        else cov_hit(CV_RXN);
      } else if (g_eng_rx_out > 0) {
        cov_hit(CV_RX_STATE0);
      }
      {
        unsigned long nsp0 = g_nsp;
        ret = w->cb(dp, g_in, g_out, 160);
        if (g_nsp > nsp0) cov_hit(CV_RATES_SET);
        if (ret == 4) cov_hit(CV_ERR_ST);
        if (ret == 1 && g_nsp > nsp0) cov_hit(CV_LINK_NEW);
        if (ret == 1 && g_nsp == nsp0) cov_hit(CV_LINK_AGAIN);
      }
      emit(0, "process -> %d out=%016llx", ret, fnv_shorts(g_out, 160));
      dump_glue(0, "after", &g->dp);
      emit(0, "dp status %u", dp->status);
    }
    dump_glue(1, "process end", d);
    emit(1, "delete -> %d", g_ops->delete(d));
    trial_end("process");
  }
  (void)k;
  (void)g_cl_state;
  (void)g_cl_lowclass;
  cov_finish();
}

/* ================================================================================================ scenario 3: end to end */
static unsigned g_script_pos;
static unsigned script_status(void)
{
  /* idle (0) for a while, a handful of mixed statuses, then linked (7) for a long run, an error and again idle: a plausible session */
  unsigned p = g_script_pos++ % 240u;
  if (p < 20) return 0;
  if (p < 26) return (p & 1) ? 0u : 2u;
  if (p < 150) return 7;
  if (p < 156) return 5;
  if (p < 160) return 1;
  if (p < 200) return 0;
  return 0x107;
}
static void sc_e2e(void)
{
  static const int rates[2] = {9600, 8000};
  int t, it;
  cov_def(CV_RUN_E2E, "end to end: op->process = dp_wrapper_run");
  cov_def(CV_E2E_LINKED, "end to end: linked state reached");
  cov_def(CV_E2E_DROP, "end to end: error status after linked");
  cov_def(CV_LINK_NEW, "end to end: rates set on the first link");
  cov_def(CV_LINK_AGAIN, "end to end: linked status repeated");
  cov_def(CV_RXN, "end to end: rx bits put");
  cov_def(CV_GB_FULL, "end to end: tx bits fetched");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  dsplibs_debug_level = 2;
  cov_hit(CV_DBG2);
  for (t = 0; t < 4; t++) {
    struct dp *d;
    B103Glue *g;
    rc_reset();
    eng_reset();
    g_eng_calls = 0;
    d = do_create(t & 1, t < 2 ? DP_B103 : DP_V21, t & 1, rates[t & 1]);
    if (!d) die3("create e2e");
    g = (B103Glue *)d;
    g_script_pos = (unsigned)t * 17u;
    g_eng_status_fn = script_status;
    g_eng_rx_out = 3;
    g_bits_give = 6;
    for (it = 0; it < 90; it++) {
      int cnt = rpct(25) ? (int)rmod(40) + 1 : (int)rmod(450) + 100, r;
      unsigned long nsp0 = g_nsp, npb0 = g_npb, ngb0 = g_ngb;
      unsigned st0 = g->dp.status;
      if (rpct(5)) cnt = 0;
      fill_in(cnt);
      set_regions();
      r = g_ops->process(d, g_in, g_out, cnt);
      emit(0, "e2e cnt=%d -> %d out=%016llx", cnt, r, fnv_shorts(g_out, cnt > 0 ? (size_t)cnt : 0));
      cov_hit(CV_RUN_E2E);
      if (r == 4 && st0 == 1) cov_hit(CV_E2E_DROP);
      if (g->dp.status == 1) cov_hit(CV_E2E_LINKED);
      if (g_nsp > nsp0) cov_hit(CV_LINK_NEW);
      if (r == 1 && st0 == 1) cov_hit(CV_LINK_AGAIN);
      if (g_npb > npb0) cov_hit(CV_RXN);
      if (g_ngb > ngb0) cov_hit(CV_GB_FULL);
      if (it % 10 == 0) dump_glue(0, "e2e", d);
    }
    dump_glue(1, "e2e end", d);
    emit(1, "delete -> %d", g_ops->delete(d));
    trial_end("e2e");
    g_eng_status_fn = NULL;
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
