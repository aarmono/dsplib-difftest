/*
 * R06 differential harness, id r06_glue: the V.22 / V.22bis / Bell 212A dp glue (v22_create / v22_process / v22_delete = blob LOCAL functions, dp_v22_init / dp_v22_exit), the real
 * dsplibs.o (32-bit) vs src/v22/dp_v22.c + src/core/dp_wrapper.c.  One source, no #ifdef around the code under test; the typed objects (src/v22/dp_v22.h) are the same on both sides and every
 * printed line is architecture independent, so the 32-bit BLOB run is the golden of the x86-64 build (test/golden/v22/dp_v22.glue.blob32.out.gz).
 *
 * The LOCAL functions are reached the way slmodemd reaches them: through the record dp_v22_init registers (captured by the modem_dp_register recorder).  The glue runs over the
 * REAL dp_wrapper (blob's vs port's); every other callee is a recorder (test/v22/r06_common.h: sysdep_*, modem_get_bits / put_bits / get_param / set_param, dsplibs_debug_printf, the fake
 * RcFixed) plus, here, a FAKE V.22 engine: V22FP_create / V22FP_modem / V22FP_delete record the arguments, write scripted outputs and return scripted statuses (the full 32-bit
 * return value: the glue must use only its low byte).  The REAL engine behind the glue is the subject of r06_e2e / r06_eng.
 *
 * Scenarios (argv[1]; own process each; exit 2 = a coverage counter is zero, 3 = harness violation):
 *   0 registration: dp_v22_init / dp_v22_exit (ids 122, 22, 212 in this order, record fields)
 *   1 create / delete lifecycle: id (the engine protocol word: 212 -> 2, 22 -> 1, others 0) x caller (0 / 1 / 2 / -1) x srate (9600, 8000, 48000, 7777 = no converter pair), debug
 *     levels 0 - 4, sysdep_malloc failing at the glue / the wrapper, RcFixed_Create failing, V22FP_create returning NULL (cleanup), delete order and dp_data cleared
 *   2 v22_process called directly through the wrapper's callback slot with fabricated dp objects: every engine status byte 0 - 255 (0, 1, 3, 4 and the error statuses) with garbage in
 *     the upper 24 bits of the return value, want 0 / 12 / other, bits 0 .. 33 (the shift count is taken modulo 32), dp status 0 / 1 / 4 before, get_bits counts, rx counts 0 / 1 .. 99 / 100 /
 *     101 / > 100 / sign bit set, debug levels (the V22STAT message appears only when the result changes the status)
 *   3 end to end through op->process = dp_wrapper_run (random chunk sizes) at 9600 and 8000 Hz with an engine status script: idle, restart (1), connect 2400 (3), connect 1200 (4), errors
 */
#include "r06_common.h"

enum {
  CV_REG_V22BIS = 0, CV_REG_V22, CV_REG_B212, CV_DEREG_ALL, CV_OPS_PROC_IS_RUN, CV_OPS_NAME, CV_REG_ORDER,
  CV_CR_V22BIS = 10, CV_CR_V22, CV_CR_B212, CV_CR_OTHERID, CV_CR_CALLER0, CV_CR_CALLER1, CV_CR_CALLER2, CV_CR_9600, CV_CR_8000, CV_CR_NOPAIR, CV_CR_MALLOC1, CV_CR_MALLOC2, CV_CR_RC_FAIL,
  CV_CR_ENG_FAIL, CV_CR_BIG, CV_DBG0, CV_DBG1, CV_DBG2, CV_DELETE,
  CV_ST0 = 40, CV_ST1, CV_ST2, CV_ST3, CV_ST4, CV_ST5, CV_ST16, CV_ST17, CV_ST255, CV_CONNECT2400, CV_CONNECT1200, CV_BITS0, CV_BITS2, CV_BITS4, CV_BITS_OTHER, CV_BITS_32, CV_WANT0, CV_WANT12,
  CV_WANT_OTHER, CV_RX0, CV_RXN, CV_RX_BIG, CV_RX_100, CV_RX_101, CV_RX_SIGN, CV_RX_DROPPED, CV_GB_0, CV_GB_FULL, CV_GB_PART, CV_GB_NEG, CV_ALIAS_DP, CV_DPST_0, CV_DPST_1, CV_DPST_4,
  CV_MSG_STAT, CV_NOMSG_SAME, CV_UPPER_GARBAGE, CV_RUN_E2E, CV_E2E_CONNECT, CV_E2E_RESTART, CV_E2E_ERR, CV_E2E_TXFETCH, CV_E2E_RXPUT, CV_E2E_1200, CV_E2E_2400
};

/* ------------------------------------------------------------------------------------------------ the fake engine */
typedef struct {
  int idx;
  int live;
} FakeEng;
static FakeEng g_eng[8];
static int g_neng, g_eng_fail;
static unsigned g_eng_calls;
static int g_eng_rx_out;                         /* scripted *rxCount result */
static unsigned g_eng_status;                    /* scripted low byte of the return value of the next V22FP_modem */
static unsigned (*g_eng_status_fn)(void);
static int eng_id(const void *p) { return (int)(((const FakeEng *)p) - g_eng); }

V22Engine *V22FP_create(V22Engine *eng, const int *cfg)
{
  int i;
  emit(0, "ENG create eng=%s cfg=%d,%d,%d,%d,%d,%d,%d", eng ? "set" : "NULL", cfg[0], cfg[1], cfg[2], cfg[3], cfg[4], cfg[5], cfg[6]);
  if (g_eng_fail) {
    emit(0, "ENG create -> NULL");
    return NULL;
  }
  if (g_neng >= 8) die3("engine pool");
  i = g_neng++;
  g_eng[i].idx = i;
  g_eng[i].live = 1;
  emit(0, "ENG create -> e%d", i);
  return (V22Engine *)(void *)&g_eng[i];
}
void V22FP_delete(V22Engine *obj)
{
  FakeEng *e = (FakeEng *)(void *)obj;
  if (!e->live) die3("engine deleted twice");
  e->live = 0;
  emit(0, "ENG delete e%d", eng_id(e));
}
unsigned int V22FP_modem(V22Engine *obj, const int *txBits, short *txOut, const short *rxIn, int *rxBits, const unsigned int *txCount, unsigned int *rxCount)
{
  FakeEng *e = (FakeEng *)(void *)obj;
  char dtx[96], dtxo[96], drxi[96], drxo[96];
  int txin = (int)*txCount;
  unsigned rxin = *rxCount;
  int i, ntx = txin > 0 && txin < 100 ? txin : (txin >= 100 ? 100 : 0);
  unsigned st, ret;
  if (!e->live) die3("engine used after delete");
  describe(txBits, dtx, sizeof dtx);
  describe(txOut, dtxo, sizeof dtxo);
  describe(rxIn, drxi, sizeof drxi);
  describe(rxBits, drxo, sizeof drxo);
  st = g_eng_status_fn ? g_eng_status_fn() : g_eng_status;
  /* the audio the engine "transmits": a function of the receive samples and the call counter; the rx bits it "demodulated": random ints (all 32 bits) */
  for (i = 0; i < (int)rxin && i < 1024; i++) txOut[i] = (short)(rxIn[i] + (short)(g_eng_calls * 11u));
  for (i = 0; i < g_eng_rx_out && i < 100; i++) rxBits[i] = (int)rnd();
  ret = (rnd() & 0xffffff00u) | (st & 0xffu);
  emit(0, "ENG modem e%d tx=%s txout=%s rxin=%s rxout=%s txin=%d rxin=%u txh=%016llx rxh=%016llx -> rxout=%d st=%02x", eng_id(e), dtx, dtxo, drxi, drxo, txin, rxin,
       fnv_ints(txBits, (size_t)ntx), fnv_shorts(rxIn, rxin < 1024 ? rxin : 1024), g_eng_rx_out, st & 0xff);
  *rxCount = (unsigned)g_eng_rx_out;
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
  V22Glue *g = (V22Glue *)dp;
  if (!dp) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s id=%d modem=m%d status=%u op=%s dp_data=%s bits=%d want=%d eng=%d wrapper=%s tx=%016llx rx=%016llx", tag, (int)dp->id, modem_no(dp->modem), dp->status,
       dp->op == g_ops ? "ops" : (dp->op == NULL ? "NULL" : "other"), dp->dp_data == (void *)g->wrapper ? "wrapper" : (dp->dp_data == NULL ? "NULL" : "other"), g->bits, g->want,
       g->eng ? eng_id(g->eng) : -1, g->wrapper ? "set" : "NULL", fnv_ints(g->txbits, 100), fnv_ints(g->rxbits, 100));
}
static void init_ops(void)
{
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  dp_v22_init();
  if (g_nreg_op != 3 || g_reg_op[0] != g_reg_op[1] || g_reg_op[1] != g_reg_op[2]) die3("dp_v22_init registered %d records", g_nreg_op);
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
  set_regions();
  return g_ops->create(M_(mi), (enum DP_ID)id, caller, srate, 160, g_ops);
}

/* ================================================================================================ scenario 0: registration */
static void sc_reg(void)
{
  int r, i;
  cov_def(CV_REG_V22BIS, "dp_v22_init: DP_V22BIS (122) registered first");
  cov_def(CV_REG_V22, "dp_v22_init: DP_V22 (22) registered second");
  cov_def(CV_REG_B212, "dp_v22_init: DP_B212 (212) registered third");
  cov_def(CV_DEREG_ALL, "dp_v22_exit: three deregistrations");
  cov_def(CV_OPS_PROC_IS_RUN, "record: process slot is dp_wrapper_run");
  cov_def(CV_OPS_NAME, "record: name v22, use_count 0, hangup NULL");
  cov_def(CV_REG_ORDER, "same record for all three ids");
  arena_init();
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  r = dp_v22_init();
  emit(1, "dp_v22_init -> %d, %d registrations", r, g_nreg_op);
  for (i = 0; i < g_nreg_op; i++) emit(1, "registered id %d", g_reg_id[i]);
  if (g_nreg_op == 3) {
    if (g_reg_id[0] == DP_V22BIS) cov_hit(CV_REG_V22BIS);
    if (g_reg_id[1] == DP_V22) cov_hit(CV_REG_V22);
    if (g_reg_id[2] == DP_B212) cov_hit(CV_REG_B212);
    if (g_reg_op[0]->process == dp_wrapper_run) cov_hit(CV_OPS_PROC_IS_RUN);
    if (strcmp(g_reg_op[0]->name, "v22") == 0 && g_reg_op[0]->use_count == 0 && g_reg_op[0]->hangup == NULL && g_reg_op[0]->create && g_reg_op[0]->delete) cov_hit(CV_OPS_NAME);
    if (g_reg_op[0] == g_reg_op[1] && g_reg_op[1] == g_reg_op[2]) cov_hit(CV_REG_ORDER);
    emit(1, "same record: %d", g_reg_op[0] == g_reg_op[1] && g_reg_op[1] == g_reg_op[2]);
  }
  g_nreg_op = 0;
  dp_v22_exit();
  cov_hit(CV_DEREG_ALL);
  cov_finish();
}

/* ================================================================================================ scenario 1: create / delete */
static void sc_lifecycle(void)
{
  static const int ids[5] = {DP_V22BIS, DP_V22, DP_B212, DP_V32, 0};
  static const int callers[4] = {0, 1, 2, -1};
  static const int rates[4] = {9600, 8000, 7777, 48000};
  int a, b, c, n = 0, f;
  cov_def(CV_CR_V22BIS, "create: id DP_V22BIS (protocol 0)");
  cov_def(CV_CR_V22, "create: id DP_V22 (protocol 1)");
  cov_def(CV_CR_B212, "create: id DP_B212 (protocol 2)");
  cov_def(CV_CR_OTHERID, "create: another id (protocol 0)");
  cov_def(CV_CR_CALLER0, "create: caller 0 (cfg[0] = 1)");
  cov_def(CV_CR_CALLER1, "create: caller non-zero (cfg[0] = 0)");
  cov_def(CV_CR_CALLER2, "create: caller negative");
  cov_def(CV_CR_9600, "create: srate 9600 (converters)");
  cov_def(CV_CR_8000, "create: srate 8000 (no converters)");
  cov_def(CV_CR_NOPAIR, "create: srate without a pair (NULL)");
  cov_def(CV_CR_MALLOC1, "create: glue malloc fails");
  cov_def(CV_CR_MALLOC2, "create: wrapper malloc fails");
  cov_def(CV_CR_RC_FAIL, "create: RcFixed_Create fails");
  cov_def(CV_CR_ENG_FAIL, "create: V22FP_create fails");
  cov_def(CV_CR_BIG, "create: srate 48000");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  cov_def(CV_DELETE, "delete via op->delete");
  arena_init();
  init_ops();
  for (a = 0; a < 5; a++)
    for (b = 0; b < 4; b++)
      for (c = 0; c < 4; c++) {
        struct dp *d;
        char tag[96];
        int level = n % 5;
        n++;
        dsplibs_debug_level = lvl_value(level);
        cov_hit(level == 0 ? CV_DBG0 : (level == 1 ? CV_DBG1 : CV_DBG2));
        rc_reset();
        eng_reset();
        d = do_create(n & 1, ids[a], callers[b], rates[c]);
        snprintf(tag, sizeof tag, "create id=%d caller=%d srate=%d level=%d", ids[a], callers[b], rates[c], level);
        dump_glue(1, tag, d);
        if (ids[a] == DP_V22BIS) cov_hit(CV_CR_V22BIS);
        else if (ids[a] == DP_V22) cov_hit(CV_CR_V22);
        else if (ids[a] == DP_B212) cov_hit(CV_CR_B212);
        else cov_hit(CV_CR_OTHERID);
        if (callers[b] == 0) cov_hit(CV_CR_CALLER0);
        else if (callers[b] < 0) cov_hit(CV_CR_CALLER2);
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
  for (f = 0; f < 6; f++) {
    struct dp *d;
    char tag[80];
    rc_reset();
    eng_reset();
    g_fail_malloc = f == 0 ? 0 : (f == 1 ? 1 : -1);
    g_rc_fail_mask = f == 2 ? 1u : (f == 3 ? 2u : 0u);
    g_eng_fail = f == 4;
    d = do_create(0, DP_V22BIS, 0, f == 5 ? 7777 : 9600);
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
static void sc_process(void)
{
  int t, it;
  struct dp alias;
  cov_def(CV_ST0, "engine status 0");
  cov_def(CV_ST1, "engine status 1 (want cleared)");
  cov_def(CV_ST2, "engine status 2 (error)");
  cov_def(CV_ST3, "engine status 3 (connect 2400)");
  cov_def(CV_ST4, "engine status 4 (connect 1200)");
  cov_def(CV_ST5, "engine status 5 .. 16 (error)");
  cov_def(CV_ST16, "engine status 16 (the last table entry)");
  cov_def(CV_ST17, "engine status 17 (first beyond the table)");
  cov_def(CV_ST255, "engine status 255");
  cov_def(CV_CONNECT2400, "connect: 2400 (bits 4, params 2 and 1)");
  cov_def(CV_CONNECT1200, "connect: 1200 (bits 2, params 2 and 1)");
  cov_def(CV_BITS0, "bits = 0");
  cov_def(CV_BITS2, "bits = 2");
  cov_def(CV_BITS4, "bits = 4");
  cov_def(CV_BITS_OTHER, "bits = another value below 32");
  cov_def(CV_BITS_32, "bits >= 32 or negative (shift count modulo 32)");
  cov_def(CV_WANT0, "want 0");
  cov_def(CV_WANT12, "want 12");
  cov_def(CV_WANT_OTHER, "want other");
  cov_def(CV_RX0, "rx count 0 with want != 0");
  cov_def(CV_RXN, "rx bits put (want != 0, count 1 .. 99)");
  cov_def(CV_RX_BIG, "rx count > 100 (clamped to 100)");
  cov_def(CV_RX_100, "rx count exactly 100 (not clamped)");
  cov_def(CV_RX_101, "rx count exactly 101");
  cov_def(CV_RX_SIGN, "rx count with the sign bit set (unsigned compare)");
  cov_def(CV_RX_DROPPED, "rx bits dropped (want 0)");
  cov_def(CV_GB_0, "get_bits returns 0");
  cov_def(CV_GB_FULL, "get_bits returns all requested");
  cov_def(CV_GB_PART, "get_bits returns fewer");
  cov_def(CV_GB_NEG, "get_bits returns a negative count");
  cov_def(CV_ALIAS_DP, "dp != glue (modem read from dp)");
  cov_def(CV_DPST_0, "dp status 0 before");
  cov_def(CV_DPST_1, "dp status 1 before");
  cov_def(CV_DPST_4, "dp status 4 before");
  cov_def(CV_MSG_STAT, "V22STAT message (status changes, level > 1)");
  cov_def(CV_NOMSG_SAME, "status unchanged (no message)");
  cov_def(CV_UPPER_GARBAGE, "garbage in the upper 24 bits of the engine return value");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  for (t = 0; t < 6; t++) {
    struct dp *d;
    V22Glue *g;
    DpWrapper *w;
    rc_reset();
    eng_reset();
    dsplibs_debug_level = 0;
    d = do_create(0, t % 3 == 0 ? DP_V22BIS : (t % 3 == 1 ? DP_V22 : DP_B212), t & 2 ? 1 : 0, t & 4 ? 9600 : 8000);
    if (!d) die3("create");
    g = (V22Glue *)d;
    w = g->wrapper;
    for (it = 0; it < 520; it++) {
      struct dp *dp = &g->dp;
      int ret, lvl = (int)rmod(5);
      unsigned st = it < 256 && rpct(40) ? (unsigned)it : (rpct(65) ? rmod(8) : rmod(256));
      static const int wants[] = {0, 12, 12, 12, 6, 1, 100, 7};
      static const unsigned dst[] = {0, 1, 4, 1, 0, 0};
      static const int bitsv[] = {0, 2, 4, 5, 3, 6, 4, 2, 31, 32, 33, -1, 8, 7};
      dsplibs_debug_level = lvl_value(lvl);
      cov_hit(lvl == 0 ? CV_DBG0 : (lvl == 1 ? CV_DBG1 : CV_DBG2));
      g->want = wants[rmod(8)];
      g->bits = bitsv[rmod(14)];
      g->dp.status = dst[rmod(6)];
      if (rpct(40)) { /* a different dp object: its own modem and status; dp_data reaches the same wrapper */
        alias = g->dp;
        alias.modem = M_(1 + (int)rmod(2));
        alias.status = dst[rmod(6)];
        alias.dp_data = w;
        dp = &alias;
        cov_hit(CV_ALIAS_DP);
      }
      g_eng_status = st;
      g->txbits[99] = (int)rnd();
      g->txbits[98] = (int)rnd();
      g_eng_rx_out = rpct(25) ? 0 : (rpct(15) ? (int)rmod(130) + 90 : (int)rmod(100));
      if (rpct(4)) g_eng_rx_out = rpct(50) ? -1 : (int)0x80000000u; /* an engine count with the sign bit set: the glue compares it UNSIGNED */
      if (rpct(8)) g_eng_rx_out = 100;
      if (rpct(8)) g_eng_rx_out = 101;
      g_bits_raw = rpct(30);
      g_bits_give = g_bits_raw ? (rpct(10) ? -1 : (int)rmod(101)) : (int)rmod(14);
      fill_in(160);
      set_regions();
      if (st == 0) cov_hit(CV_ST0);
      else if (st == 1) cov_hit(CV_ST1);
      else if (st == 2) cov_hit(CV_ST2);
      else if (st == 3) cov_hit(CV_ST3);
      else if (st == 4) cov_hit(CV_ST4);
      else if (st == 16) cov_hit(CV_ST16);
      else if (st == 17) cov_hit(CV_ST17);
      else if (st == 255) cov_hit(CV_ST255);
      else if (st < 16) cov_hit(CV_ST5);
      cov_hit(CV_UPPER_GARBAGE);
      if (g->want == 0) cov_hit(CV_WANT0);
      else if (g->want == 12) cov_hit(CV_WANT12);
      else cov_hit(CV_WANT_OTHER);
      if (g->bits == 0) cov_hit(CV_BITS0);
      else if (g->bits == 2) cov_hit(CV_BITS2);
      else if (g->bits == 4) cov_hit(CV_BITS4);
      else if (g->bits > 0 && g->bits < 32) cov_hit(CV_BITS_OTHER);
      else cov_hit(CV_BITS_32);
      if (dp->status == 0) cov_hit(CV_DPST_0);
      else if (dp->status == 1) cov_hit(CV_DPST_1);
      else cov_hit(CV_DPST_4);
      if (g->want != 0) {
        if (g_bits_give < 0 && g_bits_raw) cov_hit(CV_GB_NEG);
        else if (g_bits_give == 0) cov_hit(CV_GB_0);
        else if (g_bits_give >= g->want) cov_hit(CV_GB_FULL);
        else cov_hit(CV_GB_PART);
      }
      if (g->want == 0) cov_hit(CV_RX_DROPPED);
      else if (g_eng_rx_out == 0) cov_hit(CV_RX0);
      else if (g_eng_rx_out == 100) cov_hit(CV_RX_100);
      else if (g_eng_rx_out == 101) cov_hit(CV_RX_101);
      else if (g_eng_rx_out < 0) cov_hit(CV_RX_SIGN);
      else if (g_eng_rx_out > 101) cov_hit(CV_RX_BIG);
      else cov_hit(CV_RXN);
      {
        unsigned st0 = dp->status;
        unsigned long nsp0 = g_nsp;
        ret = w->cb(dp, g_in, g_out, 160);
        if (dp->status == st0) cov_hit(CV_NOMSG_SAME);
        else cov_hit(CV_MSG_STAT);
        if (g_nsp > nsp0) cov_hit(g->bits == 4 ? CV_CONNECT2400 : CV_CONNECT1200);
      }
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
static unsigned script_status(void)
{
  /* idle, a restart (1), a connect at 2400 (3) that repeats, data (0), a retrain restart, a connect at 1200 (4), errors (2, 9, 17, 0x41), idle */
  unsigned p = g_script_pos++ % 300u;
  if (p < 20) return 0;
  if (p < 25) return 1;
  if (p < 30) return 3;
  if (p < 120) return 0;
  if (p < 124) return 1;
  if (p < 128) return 4;
  if (p < 200) return 0;
  if (p < 210) return 2;
  if (p < 220) return 9;
  if (p < 230) return 17;
  if (p < 240) return 0x41;
  if (p < 250) return 5;
  return 0;
}
static void sc_e2e(void)
{
  static const int rates[2] = {9600, 8000};
  int t, it;
  cov_def(CV_RUN_E2E, "end to end: op->process = dp_wrapper_run");
  cov_def(CV_E2E_CONNECT, "end to end: connect (rates set)");
  cov_def(CV_E2E_RESTART, "end to end: engine status 1 restart");
  cov_def(CV_E2E_ERR, "end to end: error status (4)");
  cov_def(CV_E2E_TXFETCH, "end to end: tx bits fetched");
  cov_def(CV_E2E_RXPUT, "end to end: rx bits put");
  cov_def(CV_E2E_1200, "end to end: connect at 1200");
  cov_def(CV_E2E_2400, "end to end: connect at 2400");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  dsplibs_debug_level = 2;
  cov_hit(CV_DBG2);
  for (t = 0; t < 4; t++) {
    struct dp *d;
    V22Glue *g;
    rc_reset();
    eng_reset();
    g_eng_calls = 0;
    d = do_create(t & 1, t < 2 ? DP_V22BIS : DP_B212, t & 1, rates[t & 1]);
    if (!d) die3("create e2e");
    g = (V22Glue *)d;
    g_script_pos = (unsigned)t * 17u;
    g_eng_status_fn = script_status;
    g_eng_rx_out = 12;
    g_bits_give = 12;
    for (it = 0; it < 220; it++) {
      int cnt = rpct(25) ? (int)rmod(60) + 1 : (int)rmod(450) + 100, r;
      unsigned long nsp0 = g_nsp, npb0 = g_npb, ngb0 = g_ngb;
      int bits0 = g->bits;
      if (rpct(5)) cnt = 0;
      fill_in(cnt);
      set_regions();
      r = g_ops->process(d, g_in, g_out, cnt);
      emit(0, "e2e cnt=%d -> %d out=%016llx", cnt, r, fnv_shorts(g_out, cnt > 0 ? (size_t)cnt : 0));
      cov_hit(CV_RUN_E2E);
      if (g_nsp > nsp0) {
        cov_hit(CV_E2E_CONNECT);
        if (g->bits == 2) cov_hit(CV_E2E_1200);
        if (g->bits == 4) cov_hit(CV_E2E_2400);
      }
      if (g->want == 0 && bits0 != 0) cov_hit(CV_E2E_RESTART);
      if (r == 4) cov_hit(CV_E2E_ERR);
      if (g_npb > npb0) cov_hit(CV_E2E_RXPUT);
      if (g_ngb > ngb0) cov_hit(CV_E2E_TXFETCH);
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
