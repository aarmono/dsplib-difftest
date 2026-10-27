/*
 * R03 differential harness, id r03_glue: the V.32 / V.32bis dp glue (v32_create / v32_process / v32_delete = blob LOCAL functions, dp_v32_init / dp_v32_exit), the real
 * dsplibs.o (32-bit) vs src/v32/dp_v32.c + src/core/dp_wrapper.c.  One source, no #ifdef around the code under test; the typed objects (src/core/dp_glue.h) are the same on both sides
 * and every printed line is architecture independent, so the 32-bit BLOB run is the golden of the x86-64 build (test/golden/v32/dp_v32.glue.blob32.out.gz).
 *
 * The LOCAL functions are reached the way slmodemd reaches them: through the record dp_v32_init registers (captured by the modem_dp_register recorder).  The glue runs over the
 * REAL dp_wrapper (blob's vs port's); every other callee is a recorder (test/r02_common.h: sysdep_*, modem_get_bits / put_bits / get_param / set_param, dsplibs_debug_printf,
 * the fake RcFixed) plus, here, the V.32 engine entry points V32FP_create / V32FP_modem / V32FP_status / V32FP_GetCleanedSamples / V32FP_delete and modem_debug_log_data (a fake engine:
 * records the arguments, writes scripted outputs, returns scripted statuses).  The REAL engine behind the glue is the subject of r03_e2e.
 *
 * Scenarios (argv[1]; own process each; exit 2 = a coverage counter is zero, 3 = harness violation):
 *   0 registration: dp_v32_init / dp_v32_exit (ids 0x20 / 0x84, record fields)
 *   1 create / delete lifecycle: id x caller x srate (9600, 8000 = equal rates, 48000, 7777 = no converter pair), debug levels 0 - 4, modem params 4 (tx rate: below / at / above
 *     14400, huge unsigned) and 5 (physical delay: below / at / above the 0xd8 - 0x30 clip, negative), sysdep_malloc failing at the glue / the wrapper, RcFixed_Create failing,
 *     V32FP_create returning NULL (cleanup), delete order and dp_data cleared
 *   2 v32_process called directly through the wrapper's callback slot with fabricated dp objects: every engine status 0 - 255 (connect 3 / 4 / 13 / 24 - 27 with every
 *     V32FP_status rate pair, no carrier 12 / 14, error 16 - 23, ok 0 1 2 5 - 11 15 28, unknown > 28), want 0 / 12 / other, bits, dp status 0 / 1 / 4 before, get_bits counts,
 *     rx counts 0 / 1 .. 99 / 100 / > 100, cleaned-sample counts and pointer, debug levels
 *   3 end to end through op->process = dp_wrapper_run (random chunk sizes) at 9600 and 8000 Hz with an engine status script that connects (rate changes), drops and errors
 */
#include "../r02_common.h"

#define CV_(n) n
enum {
  CV_REG_V32 = 0, CV_REG_V32BIS, CV_DEREG_V32, CV_DEREG_V32BIS, CV_OPS_PROC_IS_RUN, CV_OPS_NAME,
  CV_CR_V32 = 10, CV_CR_V32BIS, CV_CR_CALLER0, CV_CR_CALLER1, CV_CR_9600, CV_CR_8000, CV_CR_NOPAIR, CV_CR_MALLOC1, CV_CR_MALLOC2, CV_CR_RC_FAIL, CV_CR_ENG_FAIL, CV_CR_BIG,
  CV_DBG0, CV_DBG1, CV_DBG2, CV_DELETE, CV_TX_LOW, CV_TX_CLIP, CV_TX_HUGE, CV_DLY_LOW, CV_DLY_CLIP, CV_DLY_EXACT, CV_DLY_NEG, CV_CFG_ROLE0, CV_CFG_ROLE1,
  CV_ST_OK = 40, CV_ST_CONN3, CV_ST_CONN4, CV_ST_CONN13, CV_ST_CONN24, CV_ST_CONN27, CV_ST_NOCARR12, CV_ST_NOCARR14, CV_ST_ERR16, CV_ST_ERR23, CV_ST_UNKNOWN, CV_ST_28, CV_ST_15,
  CV_RATE_2580, CV_RATE_2EE0, CV_RATE_3840, CV_RATE_12C0, CV_RATE_1C20, CV_RATE_OTHER, CV_BITS_REPORT, CV_ABORT4, CV_WANT0, CV_WANT12, CV_WANT_OTHER, CV_RX0, CV_RXN, CV_RX_BIG,
  CV_RX_EXACT99, CV_RX_DROPPED, CV_GB_0, CV_GB_FULL, CV_GB_PART, CV_CLEAN_LOG, CV_CLEAN_NULL, CV_CLEAN_ZERO, CV_ALIAS_DP, CV_RATES_SET, CV_DPST_0, CV_DPST_1, CV_DPST_4,
  CV_RUN_E2E, CV_E2E_CONNECT, CV_E2E_RATECHG, CV_E2E_ERR, CV_E2E_STATE4, CV_BITS3, CV_BITS5
};

/* V32Glue and B103Glue have the same shape: the shared describe() / log_size() of r02_common.h treat them alike */
_Static_assert(sizeof(V32Glue) == sizeof(B103Glue) && offsetof(V32Glue, txbits) == offsetof(B103Glue, txbits) && offsetof(V32Glue, rxbits) == offsetof(B103Glue, rxbits), "V32Glue shape");

/* ------------------------------------------------------------------------------------------------ the engine recorders */
typedef struct {
  int idx;
  int live;
  short clean[160];
} FakeEng;
static FakeEng g_eng[8];
static int g_neng, g_eng_fail;
static unsigned g_eng_calls;
static int g_eng_rx_out;                         /* scripted *rxCount result */
static unsigned g_eng_status;                    /* scripted return value of the next V32FP_modem */
static unsigned (*g_eng_status_fn)(void);
static int g_clean_mode, g_clean_n;              /* GetCleanedSamples: 0 = NULL pointer, 1 = the engine buffer; count */
static short g_stat_tx, g_stat_rx;               /* V32FP_status result words 1 and 2 */
static unsigned g_stat_fn_rate[8], g_stat_pos;
static int eng_id(const void *p) { return (int)(((const FakeEng *)p) - g_eng); }
static FakeEng *eng_of(const void *a) { return (FakeEng *)(void *)(intptr_t)a; }

struct V32Wrap *V32FP_create(int *cfg6, int param2)
{
  int i;
  emit(0, "ENG create cfg=%d,%d,%d,%d,%d,%08x p2=%d", cfg6[0], cfg6[1], cfg6[2], cfg6[3], cfg6[4], (unsigned)cfg6[5], param2);
  if (g_eng_fail) {
    emit(0, "ENG create -> NULL");
    return NULL;
  }
  if (g_neng >= 8) die3("engine pool");
  i = g_neng++;
  g_eng[i].idx = i;
  g_eng[i].live = 1;
  emit(0, "ENG create -> e%d", i);
  return (struct V32Wrap *)(void *)&g_eng[i];
}
void V32FP_delete(struct V32Wrap *obj)
{
  FakeEng *e = eng_of(obj);
  if (!e->live) die3("engine deleted twice");
  e->live = 0;
  emit(0, "ENG delete e%d", eng_id(e));
}
unsigned char V32FP_modem(struct V32Wrap *obj, void *txBits, void *txOut, void *rxIn, void *rxBits, unsigned int *txCount, unsigned int *rxCount)
{
  FakeEng *e = eng_of(obj);
  char dtx[96], dtxo[96], drxi[96], drxo[96];
  unsigned txin = *txCount, rxin = *rxCount;
  int i, ntx = txin < 100 ? (int)txin : 100;
  short *out = (short *)txOut;
  const short *rxi = (const short *)rxIn;
  int *rxo = (int *)rxBits;
  unsigned st;
  if (!e->live) die3("engine used after delete");
  describe(txBits, dtx, sizeof dtx);
  describe(txOut, dtxo, sizeof dtxo);
  describe(rxIn, drxi, sizeof drxi);
  describe(rxBits, drxo, sizeof drxo);
  st = g_eng_status_fn ? g_eng_status_fn() : g_eng_status;
  /* the audio the engine "transmits": a function of the receive samples and the call counter; the rx bits it "demodulated": random ints (all 32 bits) */
  for (i = 0; i < (int)rxin && i < 1024; i++) out[i] = (short)(rxi[i] + (short)(g_eng_calls * 11u));
  for (i = 0; i < g_eng_rx_out && i < 100; i++) rxo[i] = (int)rnd();
  emit(0, "ENG modem e%d tx=%s txout=%s rxin=%s rxout=%s txin=%d rxin=%u txh=%016llx rxh=%016llx -> rxout=%d st=%02x", eng_id(e), dtx, dtxo, drxi, drxo, (int)txin, rxin,
       fnv_ints((int *)txBits, (size_t)ntx), fnv_shorts(rxi, rxin < 1024 ? rxin : 1024), g_eng_rx_out, st & 0xff);
  *txCount = 7;
  *rxCount = (unsigned)g_eng_rx_out;
  g_eng_calls++;
  return (unsigned char)st;
}
int V32FP_status(struct V32Wrap *obj, short *statusOut)
{
  FakeEng *e = eng_of(obj);
  int i;
  if (!e->live) die3("status after delete");
  for (i = 0; i < 22; i++) statusOut[i] = (short)rnd();
  statusOut[1] = g_stat_tx;
  statusOut[2] = g_stat_rx;
  emit(0, "ENG status e%d -> tx=%d rx=%d", eng_id(e), (int)g_stat_tx, (int)g_stat_rx);
  return 1;
}
char *V32FP_GetCleanedSamples(struct V32Wrap *obj, int *outCount)
{
  FakeEng *e = eng_of(obj);
  int i;
  *outCount = g_clean_n;
  for (i = 0; i < 160; i++) e->clean[i] = (short)(i * 3 + (int)e->idx);
  emit(0, "ENG cleaned e%d -> %s n=%d", eng_id(e), g_clean_mode ? "buf" : "NULL", g_clean_n);
  return g_clean_mode ? (char *)(e->clean) : (char *)(NULL);
}
int modem_debug_log_data(struct modem *m, unsigned id, const void *data, int count)
{
  /* the data is the engine's clean buffer: described as a block-less engine field and hashed */
  emit(0, "LOGDATA m%d id=%u n=%d h=%016llx", modem_no(m), id, count, fnv_bytes(data, count > 0 && count < 320 ? (size_t)count : 0));
  return 0;
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
  V32Glue *g = (V32Glue *)dp;
  if (!dp) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s id=%d modem=m%d status=%u op=%s dp_data=%s bits=%d rate=%d want=%d eng=%d wrapper=%s tx=%016llx rx=%016llx", tag, (int)dp->id, modem_no(dp->modem), dp->status,
       dp->op == g_ops ? "ops" : (dp->op == NULL ? "NULL" : "other"), dp->dp_data == (void *)g->wrapper ? "wrapper" : (dp->dp_data == NULL ? "NULL" : "other"), g->bits, g->rate,
       g->want, g->eng ? eng_id(g->eng) : -1, g->wrapper ? "set" : "NULL", fnv_ints(g->txbits, 100), fnv_ints(g->rxbits, 100));
}
static void init_ops(void)
{
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  dp_v32_init();
  if (g_nreg_op != 2 || g_reg_op[0] != g_reg_op[1]) die3("dp_v32_init registered %d records", g_nreg_op);
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
static void set_param32(unsigned name, int v)
{
  g_param[name] = (long)v; /* modem_get_param returns long: the blob sees 32 bits, the values stay 32-bit-representable on every host */
  g_param_ptr[name] = 0;
}
static struct dp *do_create(int mi, int id, int caller, int srate)
{
  set_regions();
  return g_ops->create(M_(mi), (enum DP_ID)id, caller, srate, 40, g_ops);
}

/* ================================================================================================ scenario 0: registration */
static void sc_reg(void)
{
  int r, i;
  cov_def(CV_REG_V32, "dp_v32_init: DP_V32 registered");
  cov_def(CV_REG_V32BIS, "dp_v32_init: DP_V32BIS registered");
  cov_def(CV_DEREG_V32, "dp_v32_exit: DP_V32 deregistered");
  cov_def(CV_DEREG_V32BIS, "dp_v32_exit: DP_V32BIS deregistered");
  cov_def(CV_OPS_PROC_IS_RUN, "record: process slot is dp_wrapper_run");
  cov_def(CV_OPS_NAME, "record: name v32, use_count 0, hangup NULL");
  arena_init();
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  r = dp_v32_init();
  emit(1, "dp_v32_init -> %d, %d registrations", r, g_nreg_op);
  for (i = 0; i < g_nreg_op; i++) emit(1, "registered id %d", g_reg_id[i]);
  if (g_nreg_op == 2) {
    if (g_reg_id[0] == DP_V32) cov_hit(CV_REG_V32);
    if (g_reg_id[1] == DP_V32BIS) cov_hit(CV_REG_V32BIS);
    if (g_reg_op[0]->process == dp_wrapper_run) cov_hit(CV_OPS_PROC_IS_RUN);
    if (strcmp(g_reg_op[0]->name, "v32") == 0 && g_reg_op[0]->use_count == 0 && g_reg_op[0]->hangup == NULL && g_reg_op[0]->create && g_reg_op[0]->delete) cov_hit(CV_OPS_NAME);
    emit(1, "same record: %d", g_reg_op[0] == g_reg_op[1]);
  }
  dp_v32_exit();
  cov_hit(CV_DEREG_V32);
  cov_hit(CV_DEREG_V32BIS);
  cov_finish();
}

/* ================================================================================================ scenario 1: create / delete */
static const int k_txrate[] = {0, 4800, 9600, 14400, 14401, 0x3841, 0x10000 + 4800, 0x7fffffff, (int)0x80000000u, -1, 7200, 0x383f, 0x383e};
static const int k_delay[] = {0, 0x30, 0x90, 0xa7, 0xa8, 0xa9, 0xb0, 200, 0x7fff, -0x30, -100, (int)0xfffffff0u, 0x7fffff00};
static void sc_lifecycle(void)
{
  static const int ids[3] = {DP_V32, DP_V32BIS, DP_V22};
  static const int callers[3] = {0, 1, 2};
  static const int rates[4] = {9600, 8000, 7777, 48000};
  int a, b, c, n = 0, f;
  cov_def(CV_CR_V32, "create: id DP_V32");
  cov_def(CV_CR_V32BIS, "create: id DP_V32BIS");
  cov_def(CV_CR_CALLER0, "create: caller 0 (cfg role 1)");
  cov_def(CV_CR_CALLER1, "create: caller non-zero (cfg role 0)");
  cov_def(CV_CR_9600, "create: srate 9600 (converters)");
  cov_def(CV_CR_8000, "create: srate 8000 (no converters)");
  cov_def(CV_CR_NOPAIR, "create: srate without a pair (NULL)");
  cov_def(CV_CR_MALLOC1, "create: glue malloc fails");
  cov_def(CV_CR_MALLOC2, "create: wrapper malloc fails");
  cov_def(CV_CR_RC_FAIL, "create: RcFixed_Create fails");
  cov_def(CV_CR_ENG_FAIL, "create: V32FP_create fails");
  cov_def(CV_CR_BIG, "create: srate 48000 (blk 240, create only)");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  cov_def(CV_DELETE, "delete via op->delete");
  cov_def(CV_TX_LOW, "modem param 4 below 14400");
  cov_def(CV_TX_CLIP, "modem param 4 above 14400 (clipped)");
  cov_def(CV_TX_HUGE, "modem param 4 huge / negative as unsigned");
  cov_def(CV_DLY_LOW, "phys. delay + 0x30 below 0xd8");
  cov_def(CV_DLY_EXACT, "phys. delay + 0x30 == 0xd8");
  cov_def(CV_DLY_CLIP, "phys. delay + 0x30 above 0xd8 (param 0xd written)");
  cov_def(CV_DLY_NEG, "phys. delay negative");
  arena_init();
  init_ops();
  for (a = 0; a < 3; a++)
    for (b = 0; b < 3; b++)
      for (c = 0; c < 4; c++) {
        struct dp *d;
        char tag[96];
        int level = n % 5, tx = k_txrate[(n * 7 + a) % 13], dl = k_delay[n % 13];
        n++;
        dsplibs_debug_level = lvl_value(level);
        cov_hit(level == 0 ? CV_DBG0 : (level == 1 ? CV_DBG1 : CV_DBG2));
        set_param32(4, tx);
        set_param32(5, dl);
        if ((unsigned)tx < 14400u) cov_hit(CV_TX_LOW);
        else if ((unsigned)tx <= 0x3840u) cov_hit(CV_TX_CLIP);
        if ((unsigned)tx > 0x3840u) cov_hit(CV_TX_CLIP);
        if (tx < 0 || tx > 0xffff) cov_hit(CV_TX_HUGE);
        if (dl + 0x30 < 0) cov_hit(CV_DLY_NEG);
        if (dl + 0x30 < 0xd8) cov_hit(CV_DLY_LOW);
        else if (dl + 0x30 == 0xd8) cov_hit(CV_DLY_EXACT);
        else cov_hit(CV_DLY_CLIP);
        rc_reset();
        eng_reset();
        d = do_create(n & 1, ids[a], callers[b], rates[c]);
        snprintf(tag, sizeof tag, "create id=%d caller=%d srate=%d level=%d tx=%d dly=%d", ids[a], callers[b], rates[c], level, tx, dl);
        dump_glue(1, tag, d);
        if (ids[a] == DP_V32) cov_hit(CV_CR_V32);
        else if (ids[a] == DP_V32BIS) cov_hit(CV_CR_V32BIS);
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
  set_param32(4, 14400);
  set_param32(5, 0x50);
  for (f = 0; f < 6; f++) {
    struct dp *d;
    char tag[80];
    rc_reset();
    eng_reset();
    g_fail_malloc = f == 0 ? 0 : (f == 1 ? 1 : -1);
    g_rc_fail_mask = f == 2 ? 1u : (f == 3 ? 2u : 0u);
    g_eng_fail = f == 4;
    d = do_create(0, DP_V32BIS, 0, f == 5 ? 7777 : 9600);
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
static const short k_rates[] = {0x2580, 0x2ee0, 0x3840, 0x12c0, 0x1c20, 0, 0x1234, -1, (short)0x8000, 0x2581, 0x4b00};
static void sc_process(void)
{
  int t, it;
  struct dp alias;
  cov_def(CV_ST_OK, "engine status ok (0 1 2 5-11)");
  cov_def(CV_ST_CONN3, "engine status 3 (connect)");
  cov_def(CV_ST_CONN4, "engine status 4 (connect)");
  cov_def(CV_ST_CONN13, "engine status 13 (connect)");
  cov_def(CV_ST_CONN24, "engine status 24 (connect)");
  cov_def(CV_ST_CONN27, "engine status 27 (connect)");
  cov_def(CV_ST_NOCARR12, "engine status 12 (no carrier)");
  cov_def(CV_ST_NOCARR14, "engine status 14 (no carrier)");
  cov_def(CV_ST_ERR16, "engine status 16 (error)");
  cov_def(CV_ST_ERR23, "engine status 23 (error)");
  cov_def(CV_ST_UNKNOWN, "engine status > 28 (unknown)");
  cov_def(CV_ST_28, "engine status 28");
  cov_def(CV_ST_15, "engine status 15");
  cov_def(CV_RATE_2580, "connect tx rate 9600");
  cov_def(CV_RATE_2EE0, "connect tx rate 12000");
  cov_def(CV_RATE_3840, "connect tx rate 14400");
  cov_def(CV_RATE_12C0, "connect tx rate 4800");
  cov_def(CV_RATE_1C20, "connect tx rate 7200");
  cov_def(CV_RATE_OTHER, "connect tx rate not in the table");
  cov_def(CV_BITS_REPORT, "bits per symbol recomputed from the rate");
  cov_def(CV_ABORT4, "dp status 4 on entry: block zeroed, 4 returned");
  cov_def(CV_WANT0, "want 0");
  cov_def(CV_WANT12, "want 12");
  cov_def(CV_WANT_OTHER, "want other");
  cov_def(CV_RX0, "rx count 0 with want != 0");
  cov_def(CV_RXN, "rx bits put (want != 0, count 1 .. 99)");
  cov_def(CV_RX_BIG, "rx count > 99 (clamped to 100)");
  cov_def(CV_RX_EXACT99, "rx count exactly 99");
  cov_def(CV_RX_DROPPED, "rx bits dropped (want 0)");
  cov_def(CV_GB_0, "get_bits returns 0");
  cov_def(CV_GB_FULL, "get_bits returns all requested");
  cov_def(CV_GB_PART, "get_bits returns fewer");
  cov_def(CV_CLEAN_LOG, "cleaned samples logged");
  cov_def(CV_CLEAN_NULL, "cleaned samples NULL");
  cov_def(CV_CLEAN_ZERO, "cleaned samples count <= 0");
  cov_def(CV_ALIAS_DP, "dp != glue (modem read from dp)");
  cov_def(CV_RATES_SET, "set_param calls (connect)");
  cov_def(CV_DPST_0, "dp status 0 before");
  cov_def(CV_DPST_1, "dp status 1 before");
  cov_def(CV_DPST_4, "dp status 4 before");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  cov_def(CV_BITS3, "bits = 3");
  cov_def(CV_BITS5, "bits = 5");
  arena_init();
  init_ops();
  for (t = 0; t < 6; t++) {
    struct dp *d;
    V32Glue *g;
    DpWrapper *w;
    set_param32(4, 14400);
    set_param32(5, 0x40);
    rc_reset();
    eng_reset();
    dsplibs_debug_level = 0;
    d = do_create(0, t & 1 ? DP_V32BIS : DP_V32, t & 2 ? 1 : 0, t & 4 ? 9600 : 8000);
    if (!d) die3("create");
    g = (V32Glue *)d;
    w = g->wrapper;
    for (it = 0; it < 420; it++) {
      struct dp *dp = &g->dp;
      int ret, lvl = (int)rmod(5);
      unsigned st = rpct(75) ? rmod(31) : rmod(256);
      static const int wants[] = {0, 12, 12, 12, 6, 1, 100, 7};
      static const unsigned dst[] = {0, 1, 4, 1, 0, 0};
      static const int bitsv[] = {6, 4, 2, 5, 3, 6, 4};
      static const int ratev[] = {0x3840, 0x2580, 0x12c0, 0x2ee0, 0x1c20, 0x3840, (int)0xc00006b2u, (int)0xc00006b9u, (int)0x80000000u, -1, 0x2581, 7199};
      dsplibs_debug_level = lvl_value(lvl);
      cov_hit(lvl == 0 ? CV_DBG0 : (lvl == 1 ? CV_DBG1 : CV_DBG2));
      g->want = wants[rmod(8)];
      g->bits = bitsv[rmod(7)];
      g->rate = ratev[rmod(12)];
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
      g_stat_tx = k_rates[rmod((unsigned)(sizeof k_rates / sizeof k_rates[0]))];
      g_stat_rx = (short)(rpct(50) ? g_stat_tx : (int)rnd());
      g_eng_rx_out = rpct(25) ? 0 : (rpct(15) ? (int)rmod(130) + 90 : (int)rmod(100));
      if (rpct(4)) g_eng_rx_out = rpct(50) ? -1 : (int)0x80000000u; /* an engine count with the sign bit set: the glue compares it UNSIGNED */
      if (rpct(10)) g_eng_rx_out = 99;
      if (rpct(8)) g_eng_rx_out = 100;
      g_clean_mode = rpct(70);
      g_clean_n = rpct(15) ? (int)rmod(5) - 2 : (int)rmod(41) + 1;
      g_bits_give = (int)rmod(14);
      fill_in(40);
      set_regions();
      if (st == 3) cov_hit(CV_ST_CONN3);
      else if (st == 4) cov_hit(CV_ST_CONN4);
      else if (st == 13) cov_hit(CV_ST_CONN13);
      else if (st == 24) cov_hit(CV_ST_CONN24);
      else if (st == 27) cov_hit(CV_ST_CONN27);
      else if (st == 12) cov_hit(CV_ST_NOCARR12);
      else if (st == 14) cov_hit(CV_ST_NOCARR14);
      else if (st == 16) cov_hit(CV_ST_ERR16);
      else if (st == 23) cov_hit(CV_ST_ERR23);
      else if (st == 28) cov_hit(CV_ST_28);
      else if (st == 15) cov_hit(CV_ST_15);
      else if (st > 28) cov_hit(CV_ST_UNKNOWN);
      else if (st < 12 || st == 25 || st == 26) cov_hit(CV_ST_OK);
      if (g->want == 0) cov_hit(CV_WANT0);
      else if (g->want == 12) cov_hit(CV_WANT12);
      else cov_hit(CV_WANT_OTHER);
      if (dp->status == 0) cov_hit(CV_DPST_0);
      else if (dp->status == 1) cov_hit(CV_DPST_1);
      else cov_hit(CV_DPST_4);
      if (dp->status == 4) cov_hit(CV_ABORT4);
      else {
        if (g_bits_give == 0) cov_hit(CV_GB_0);
        else if (g_bits_give >= g->want) cov_hit(CV_GB_FULL);
        else cov_hit(CV_GB_PART);
        if (g->want == 0) cov_hit(CV_RX_DROPPED);
        else if (g_eng_rx_out == 0) cov_hit(CV_RX0);
        else if (g_eng_rx_out > 99) cov_hit(CV_RX_BIG);
        else cov_hit(CV_RXN);
        if (g->want != 0 && g_eng_rx_out == 99) cov_hit(CV_RX_EXACT99);
        if (!g_clean_mode) cov_hit(CV_CLEAN_NULL);
        else if (g_clean_n <= 0) cov_hit(CV_CLEAN_ZERO);
        else cov_hit(CV_CLEAN_LOG);
      }
      {
        unsigned long nsp0 = g_nsp;
        int bits0 = g->bits;
        ret = w->cb(dp, g_in, g_out, 40);
        if (g_nsp > nsp0) {
          cov_hit(CV_RATES_SET);
          if (g_stat_tx == 0x2580) cov_hit(CV_RATE_2580);
          else if (g_stat_tx == 0x2ee0) cov_hit(CV_RATE_2EE0);
          else if (g_stat_tx == 0x3840) cov_hit(CV_RATE_3840);
          else if (g_stat_tx == 0x12c0) cov_hit(CV_RATE_12C0);
          else if (g_stat_tx == 0x1c20) cov_hit(CV_RATE_1C20);
          else cov_hit(CV_RATE_OTHER);
          if (g->bits != bits0 || 1) cov_hit(CV_BITS_REPORT);
          if (g->bits == 3) cov_hit(CV_BITS3);
          if (g->bits == 5) cov_hit(CV_BITS5);
        }
      }
      emit(0, "process -> %d out=%016llx", ret, fnv_shorts(g_out, 40));
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
  /* training (0 / 2) for a while, a connect (4) that repeats, a rate change reconnect (13) with another status pair, an error burst, no carrier, and again idle */
  unsigned p = g_script_pos++ % 300u;
  g_stat_tx = 0x3840;
  g_stat_rx = 0x3840;
  if (p < 30) return (p & 1) ? 0u : 2u;
  if (p < 60) return 4;
  if (p < 120) {
    return 0;
  }
  if (p < 125) { g_stat_tx = 0x2580; g_stat_rx = 0x2580; return 13; }
  if (p < 200) { g_stat_tx = 0x2580; g_stat_rx = 0x2580; return 0; }
  if (p < 206) { g_stat_tx = 0x1c20; g_stat_rx = 0x2ee0; return 27; }
  if (p < 240) return 9;
  if (p < 250) return 17;
  if (p < 260) return 14;
  if (p < 270) return 0x41;
  return 1;
}
static void sc_e2e(void)
{
  static const int rates[2] = {9600, 8000};
  int t, it;
  cov_def(CV_RUN_E2E, "end to end: op->process = dp_wrapper_run");
  cov_def(CV_E2E_CONNECT, "end to end: connect (rates set)");
  cov_def(CV_E2E_RATECHG, "end to end: second connect with other rates");
  cov_def(CV_E2E_ERR, "end to end: error / no carrier status (4)");
  cov_def(CV_E2E_STATE4, "end to end: zeroed block after status 4");
  cov_def(CV_RXN, "end to end: rx bits put");
  cov_def(CV_GB_FULL, "end to end: tx bits fetched");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  dsplibs_debug_level = 2;
  cov_hit(CV_DBG2);
  set_param32(4, 14400);
  set_param32(5, 0x20);
  for (t = 0; t < 4; t++) {
    struct dp *d;
    V32Glue *g;
    int nconn = 0;
    rc_reset();
    eng_reset();
    g_eng_calls = 0;
    d = do_create(t & 1, t < 2 ? DP_V32BIS : DP_V32, t & 1, rates[t & 1]);
    if (!d) die3("create e2e");
    g = (V32Glue *)d;
    g_script_pos = (unsigned)t * 17u;
    g_eng_status_fn = script_status;
    g_eng_rx_out = 12;
    g_bits_give = 12;
    g_clean_mode = 1;
    g_clean_n = 40;
    for (it = 0; it < 160; it++) {
      int cnt = rpct(25) ? (int)rmod(60) + 1 : (int)rmod(450) + 100, r;
      unsigned long nsp0 = g_nsp, npb0 = g_npb, ngb0 = g_ngb;
      unsigned st0 = g->dp.status;
      if (rpct(5)) cnt = 0;
      if (rpct(20)) g->dp.status = rpct(50) ? 0 : g->dp.status; /* a restarted session: the glue is told nothing, the status is what slmodemd would clear */
      fill_in(cnt);
      set_regions();
      r = g_ops->process(d, g_in, g_out, cnt);
      emit(0, "e2e cnt=%d -> %d out=%016llx", cnt, r, fnv_shorts(g_out, cnt > 0 ? (size_t)cnt : 0));
      cov_hit(CV_RUN_E2E);
      if (g_nsp > nsp0) {
        cov_hit(CV_E2E_CONNECT);
        if (++nconn > 1) cov_hit(CV_E2E_RATECHG);
      }
      if (r == 4) cov_hit(CV_E2E_ERR);
      if (st0 == 4) cov_hit(CV_E2E_STATE4);
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
