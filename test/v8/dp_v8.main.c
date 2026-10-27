/*
 * R02 differential harness, id r02_v8: the V.8 dp glue (v8_create / v8_process / v8_delete = blob LOCAL functions, dp_v8_init / dp_v8_exit, the global table v8StatusName),
 * the real dsplibs.o (32-bit) vs src/v8/dp_v8.c.  Method as test/r02_b103_diff_main.c (read its header): the LOCAL functions are reached through the record dp_v8_init
 * registers (here the process slot is v8_process itself: V.8 does not use dp_wrapper), every callee is a recorder: sysdep_*, modem_get_param / set_param, dp_param_get (the
 * DpRuntime block: its bytes are randomised before every create and dumped as a hash after every call), the engine V8Create / V8Process / V8Delete /
 * V8UpdateModemParameters (this file: records the config words and every argument, returns scripted statuses, and V8UpdateModemParameters rewrites the runtime
 * block from a script), dsplibs_debug_printf.  struct dsp_info is slmodemd's real structure (modem_get_param(m, 0xb) returns the harness' instance; dumped field by field).
 *
 * Scenarios (argv[1]; own process each; exit 2 = a coverage counter is zero, 3 = harness violation):
 *   0 registration: dp_v8_init / dp_v8_exit, the record, the 19 names of v8StatusName
 *   1 create / delete: id x caller x srate (only 9600 creates), automode values, debug levels 0 / 1 / 2, malloc failing, V8Create returning NULL; the runtime block bits
 *     the glue sets (caller / id dependent) are checked through the dumped hash of the whole block and field by field in the cov counters
 *   2 v8_process called directly (fabricated dp objects: modem from dp): every V8Process status 0 .. 18, the OK status with the idle timer set / clear and the QC /
 *     capability combinations of the runtime block, status 16 with ids 0x5a / 0x5c / others, idle timer expiry / countdown, status repeats / changes, debug levels
 *   3 end to end: whole V.8 sessions through op->create / process / delete at 9600 Hz with status scripts (originate and answer, V.90 / V.92 / V.34 / V.32bis outcomes)
 */
#include "../r02_common.h"

enum {
  CV_REG_V8 = 0, CV_DEREG_V8, CV_OPS_NAME, CV_OPS_PROC, CV_NAMES,
  CV_CR_OK = 10, CV_CR_SRATE_REJECT, CV_CR_MALLOC, CV_CR_ENG_FAIL, CV_CR_CALLER0, CV_CR_CALLER1, CV_CR_ID5A_ANS, CV_CR_ID5C_ANS, CV_CR_ID5A_ORG, CV_CR_ID_OTHER, CV_DBG0, CV_DBG1, CV_DBG2, CV_DELETE,
  CV_AUTOMODE,
  CV_ST_TIMEOUT = 30, CV_ST_13_BUSY, CV_ST_13_QC, CV_ST_13_90, CV_ST_13_34, CV_ST_13_32, CV_ST_13_NONE, CV_ST_16_OK, CV_ST_16_BAD, CV_ST_16_BUSY, CV_ST_DEFAULT, CV_ST_HI, CV_ST_SAME, CV_ST_CHG,
  CV_IDLE_COUNT, CV_IDLE_EXPIRE, CV_IDLE_NONE, CV_ALIAS_DP, CV_REQ, CV_ST_3, CV_ST_2,
  CV_E2E_SESSION = 60, CV_E2E_REQ, CV_E2E_IDLE_EXPIRE, CV_E2E_TIMEOUT
};

/* ------------------------------------------------------------------------------------------------ the engine recorders */
typedef struct {
  int idx;
  int live;
} FakeEng;
static FakeEng g_eng[8];
static int g_neng, g_eng_fail;
static unsigned g_eng_calls;
static unsigned g_status;                    /* scripted V8Process result */
static unsigned (*g_status_fn)(void);
static unsigned char g_upd_b0, g_upd_b2;     /* V8UpdateModemParameters rewrites the runtime block with these */
static int g_upd_w10;
static int eng_id(const void *p) { return (int)(((const FakeEng *)p) - g_eng); }

void *V8Create(const V8Cfg *cfg)
{
  int i;
  emit(0, "ENG create side=%u opMode=%u sig=%u msg=%u pad=%u caps=%s", cfg->side, cfg->opMode, cfg->sigTimeout, cfg->msgTimeout, cfg->pad,
       cfg->caps == (const unsigned char *)&g_rt_block ? "rt" : (cfg->caps ? "other" : "NULL"));
  if (g_eng_fail) {
    emit(0, "ENG create -> NULL");
    return NULL;
  }
  if (g_neng >= 8) die3("engine pool");
  i = g_neng++;
  g_eng[i].idx = i;
  g_eng[i].live = 1;
  emit(0, "ENG create -> e%d", i);
  return &g_eng[i];
}
void V8Delete(void *eng)
{
  FakeEng *e = (FakeEng *)eng;
  if (!e->live) die3("engine deleted twice");
  e->live = 0;
  emit(0, "ENG delete e%d", eng_id(e));
}
unsigned int V8Process(V8Obj *eng, const short *in, short *out, int n)
{
  FakeEng *e = (FakeEng *)eng;
  char di[96], dout[96];
  unsigned st;
  int i;
  if (!e->live) die3("engine used after delete");
  describe(in, di, sizeof di);
  describe(out, dout, sizeof dout);
  st = g_status_fn ? g_status_fn() : g_status;
  for (i = 0; i < n && i < 1024; i++) out[i] = (short)(in[i] + (short)g_eng_calls);
  emit(0, "ENG process e%d in=%s out=%s n=%d h=%016llx -> %u", eng_id(e), di, dout, n, fnv_shorts(in, n > 0 && n < 1024 ? (size_t)n : 0), st);
  g_eng_calls++;
  return st;
}
unsigned int V8UpdateModemParameters(V8Obj *eng, unsigned char *params)
{
  emit(0, "ENG update e%d params=%s rt=%016llx", eng_id(eng), params == (unsigned char *)&g_rt_block ? "rt" : "other", fnv_bytes(&g_rt_block, sizeof g_rt_block));
  g_rt_block.flags0 = g_upd_b0;
  g_rt_block.flags2 = g_upd_b2;
  g_rt_block.qcIndex = g_upd_w10;
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
static const char *slot_name(const void *fn) { return fn ? "set" : "NULL"; }
/* dp_wrapper_run is not linked here: the process slot is the glue's own v8_process; compare against NULL / non-NULL only */
static void randomize_state(void)
{
  unsigned char *p = (unsigned char *)&g_rt_block;
  size_t i;
  for (i = 0; i < sizeof g_rt_block; i++) p[i] = (unsigned char)rnd();
  g_dsp_info.connection_type = rnd();
  g_dsp_info.clock_deviation = (long)(int)rnd();
  g_dsp_info.qc_lapm = rnd();
  g_dsp_info.qc_index = rnd();
}
static void dump_glue(int show, const char *tag, struct dp *dp)
{
  V8Glue *g = (V8Glue *)dp;
  if (!dp) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s id=%d modem=m%d status=%u op=%s dp_data=%s orig=%d gid=%d w1c=%d idle=%d di=%s rt=%s st=%d eng=%d", tag, (int)dp->id, modem_no(dp->modem), dp->status,
       dp->op == g_ops ? "ops" : (dp->op == NULL ? "NULL" : "other"), dp->dp_data == (void *)g ? "self" : (dp->dp_data == NULL ? "NULL" : "other"), g->originate, g->id, g->w1c, g->idle,
       g->di == &g_dsp_info ? "info" : "other", g->rt == &g_rt_block ? "rt" : "other", g->status, g->eng ? eng_id(g->eng) : -1);
  emit(show, "  rt=%016llx dsp_info type=%u dev=%d lapm=%u qc=%u", fnv_bytes(&g_rt_block, sizeof g_rt_block), g_dsp_info.connection_type, (int)g_dsp_info.clock_deviation, g_dsp_info.qc_lapm,
       g_dsp_info.qc_index);
}
static void init_ops(void)
{
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  dp_v8_init();
  if (g_nreg_op != 1) die3("dp_v8_init registered %d records", g_nreg_op);
  g_ops = g_reg_op[0];
  g_param[8] = 1;
  g_param[5] = 640;
  g_param[0xb] = (long)(intptr_t)&g_dsp_info;
  g_param_ptr[0xb] = 1;
}
static short g_in[2048], g_out[2048];
static void set_regions(void)
{
  reg_clear();
  reg_add(g_in, sizeof g_in, 2, "IN");
  reg_add(g_out, sizeof g_out, 2, "OUT");
}
static void fill_in(int n)
{
  int i;
  for (i = 0; i < n && i < 2048; i++) g_in[i] = (short)rnd();
  for (i = 0; i < 2048; i++) g_out[i] = 0x5a5a;
}

/* ================================================================================================ scenario 0: registration */
static void sc_reg(void)
{
  int r, i, ok = 1;
  static const char *const want[19] = {"V8_INIT", "V8_ANS_SEND_ANSAM", "V8_ANS_CM_DETECTED", "V8_ANS_SEND_JM", "V8_ANS_TIME_OUT_WAITING_FOR_CM", "V8_ANS_TIME_OUT_WAITING_FOR_CJ",
                                       "V8_ORG_WAITING_FOR_ANSAM", "V8_ORG_ANSAM_DETECTED_WAITING_TE", "V8_ORG_SEND_CM", "V8_ORG_JM_DETECTED", "V8_ORG_SEND_CJ",
                                       "V8_ORG_TIME_OUT_WAITING_FOR_ANSAM", "V8_ORG_TIME_OUT_WAITING_FOR_JM", "V8_OK", "V8_ORG_SEND_QC", "V8_ORG_WAITING_FOR_QCA1d",
                                       "V8_ORG_BAD_QCA1d_MESSAGE", "V8_ORG_TIME_OUT_WAITING_FOR_QCA1d", "V8_LAST_ENUM"};
  cov_def(CV_REG_V8, "dp_v8_init: DP_V8 registered");
  cov_def(CV_DEREG_V8, "dp_v8_exit: DP_V8 deregistered");
  cov_def(CV_OPS_NAME, "record: name V8, use_count 0, hangup NULL");
  cov_def(CV_OPS_PROC, "record: create / delete / process set");
  cov_def(CV_NAMES, "v8StatusName: 19 names equal");
  arena_init();
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  r = dp_v8_init();
  emit(1, "dp_v8_init -> %d, %d registrations", r, g_nreg_op);
  for (i = 0; i < g_nreg_op; i++) emit(1, "registered id %d", g_reg_id[i]);
  if (g_nreg_op == 1) {
    if (g_reg_id[0] == DP_V8) cov_hit(CV_REG_V8);
    if (strcmp(g_reg_op[0]->name, "V8") == 0 && g_reg_op[0]->use_count == 0 && g_reg_op[0]->hangup == NULL) cov_hit(CV_OPS_NAME);
    if (g_reg_op[0]->create && g_reg_op[0]->delete && g_reg_op[0]->process) cov_hit(CV_OPS_PROC);
  }
  for (i = 0; i < 19; i++) {
    emit(1, "name %d %s", i, v8StatusName[i]);
    if (strcmp(v8StatusName[i], want[i]) != 0) ok = 0;
  }
  if (ok) cov_hit(CV_NAMES);
  dp_v8_exit();
  cov_hit(CV_DEREG_V8);
  cov_finish();
}

/* ================================================================================================ scenario 1: create / delete */
static void sc_lifecycle(void)
{
  static const int ids[7] = {DP_V8, 0x5a, 0x5c, 0x22, 0x20, 0x84, 0};
  static const int callers[3] = {0, 1, 2};
  static const int rates[4] = {9600, 8000, 0, 48000};
  int a, b, c, n = 0, f;
  cov_def(CV_CR_OK, "create ok (srate 9600)");
  cov_def(CV_CR_SRATE_REJECT, "create: srate != 9600 -> NULL");
  cov_def(CV_CR_MALLOC, "create: malloc fails");
  cov_def(CV_CR_ENG_FAIL, "create: V8Create fails");
  cov_def(CV_CR_CALLER0, "create: caller 0");
  cov_def(CV_CR_CALLER1, "create: caller != 0");
  cov_def(CV_CR_ID5A_ANS, "create: caller != 0, id 0x5a");
  cov_def(CV_CR_ID5C_ANS, "create: caller != 0, id 0x5c");
  cov_def(CV_CR_ID5A_ORG, "create: caller == 0, id 0x5a");
  cov_def(CV_CR_ID_OTHER, "create: other id");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  cov_def(CV_DELETE, "delete via op->delete");
  cov_def(CV_AUTOMODE, "automode parameter (printed)");
  arena_init();
  init_ops();
  for (a = 0; a < 7; a++)
    for (b = 0; b < 3; b++)
      for (c = 0; c < 4; c++) {
        struct dp *d;
        char tag[80];
        int level = n % 5;
        n++;
        dsplibs_debug_level = lvl_value(level);
        cov_hit(level == 0 ? CV_DBG0 : (level == 1 ? CV_DBG1 : CV_DBG2));
        g_param[8] = (long)(n % 4);
        cov_hit(CV_AUTOMODE);
        rc_reset();
        eng_reset();
        randomize_state();
        set_regions();
        d = g_ops->create(M_(n & 1), (enum DP_ID)ids[a], callers[b], rates[c], 160, g_ops);
        snprintf(tag, sizeof tag, "create id=%d caller=%d srate=%d level=%d", ids[a], callers[b], rates[c], level);
        dump_glue(1, tag, d);
        if (callers[b] == 0) cov_hit(CV_CR_CALLER0);
        else cov_hit(CV_CR_CALLER1);
        if (!d) {
          cov_hit(CV_CR_SRATE_REJECT);
        } else {
          int r;
          cov_hit(CV_CR_OK);
          if (callers[b] != 0 && ids[a] == 0x5a) cov_hit(CV_CR_ID5A_ANS);
          else if (callers[b] != 0 && ids[a] == 0x5c) cov_hit(CV_CR_ID5C_ANS);
          else if (callers[b] == 0 && ids[a] == 0x5a) cov_hit(CV_CR_ID5A_ORG);
          else cov_hit(CV_CR_ID_OTHER);
          r = g_ops->delete(d);
          emit(1, "delete -> %d", r);
          cov_hit(CV_DELETE);
        }
        trial_end(tag);
      }
  dsplibs_debug_level = 2;
  for (f = 0; f < 2; f++) {
    struct dp *d;
    char tag[80];
    rc_reset();
    eng_reset();
    randomize_state();
    g_fail_malloc = f == 0 ? 0 : -1;
    g_eng_fail = f == 1;
    d = g_ops->create(M_(0), DP_V8, 0, 9600, 160, g_ops);
    snprintf(tag, sizeof tag, "failure %d", f);
    dump_glue(1, tag, d);
    if (d) die3("a failing callee still created the glue (%d)", f);
    cov_hit(f == 0 ? CV_CR_MALLOC : CV_CR_ENG_FAIL);
    trial_end(tag);
  }
  cov_finish();
}

/* ================================================================================================ scenario 2: process called directly */
static void sc_process(void)
{
  static const int gids[] = {0x5a, 0x5c, 0x22, 0x20, 0, 7, 0x5c, 0x5a};
  static const int idles[] = {0, 0, 0, 5, 100, -1, 700, 0x2a0, 160, 161, 159, 1, 2};
  static const int cnts[] = {0, 1, 160, 640, 100, 159, 161, 1000};
  struct dp alias;
  int t, it;
  cov_def(CV_ST_TIMEOUT, "status 4 / 5 / 11 / 12 / 17 (timeout, ret 4)");
  cov_def(CV_ST_13_BUSY, "status 13 with idle != 0");
  cov_def(CV_ST_13_QC, "status 13, QC bit set (id requested)");
  cov_def(CV_ST_13_90, "status 13, capability bit 8 (V.90)");
  cov_def(CV_ST_13_34, "status 13, capability bit 0x20 (V.34)");
  cov_def(CV_ST_13_32, "status 13, capability bit 0x80 (V.32bis)");
  cov_def(CV_ST_13_NONE, "status 13, no capability (ret 4)");
  cov_def(CV_ST_16_OK, "status 16 with id 0x5a / 0x5c, idle clear (request 0x5c)");
  cov_def(CV_ST_16_BAD, "status 16 with another id (ret 4)");
  cov_def(CV_ST_16_BUSY, "status 16 with idle != 0");
  cov_def(CV_ST_DEFAULT, "status without an arm (0 1 3 6 8 10 14 15, ret 0)");
  cov_def(CV_ST_HI, "status 18 (beyond the jump table)");
  cov_def(CV_ST_SAME, "status equals the last status");
  cov_def(CV_ST_CHG, "status changes (print at level 2)");
  cov_def(CV_IDLE_COUNT, "idle timer counts down");
  cov_def(CV_IDLE_EXPIRE, "idle timer expires (request 0, ret 10)");
  cov_def(CV_IDLE_NONE, "idle timer clear or expired (no countdown)");
  cov_def(CV_ALIAS_DP, "dp != glue (modem read from dp)");
  cov_def(CV_REQ, "DP change requested (ret 10)");
  cov_def(CV_ST_3, "status 3");
  cov_def(CV_ST_2, "status 2");
  cov_def(CV_DBG0, "debug level 0");
  cov_def(CV_DBG1, "debug level 1");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  for (t = 0; t < 6; t++) {
    struct dp *d;
    V8Glue *g;
    rc_reset();
    eng_reset();
    randomize_state();
    set_regions();
    d = g_ops->create(M_(0), DP_V8, t & 1, 9600, 160, g_ops);
    if (!d) die3("create");
    g = (V8Glue *)d;
    for (it = 0; it < 400; it++) {
      struct dp *dp = d;
      int ret, lvl = (int)rmod(5), cnt = cnts[rmod(8)];
      unsigned st = rmod(19);
      unsigned long nsp0 = g_nsp;
      if (rpct(10)) st = 13;
      if (rpct(6)) st = 16;
      dsplibs_debug_level = lvl_value(lvl);
      cov_hit(lvl == 0 ? CV_DBG0 : (lvl == 1 ? CV_DBG1 : CV_DBG2));
      g->id = gids[rmod(8)];
      g->idle = idles[rmod(13)];
      g->status = rpct(30) ? (int)st : (int)rmod(19);
      g->w1c = (int)rnd();
      g_param[5] = (long)(int)rmod(1000);
      g_param[0xb] = (long)(intptr_t)&g_dsp_info;
      g_dsp_info.qc_lapm = rnd();
      g_dsp_info.qc_index = rnd();
      {
        unsigned char *p = (unsigned char *)&g_rt_block;
        size_t i;
        for (i = 0; i < sizeof g_rt_block; i++) p[i] = (unsigned char)rnd();
      }
      g_upd_b0 = (unsigned char)rnd();
      g_upd_b2 = (unsigned char)rnd();
      g_upd_w10 = (int)rnd();
      if (rpct(25)) g_upd_b2 &= (unsigned char)~0x10;
      if (rpct(20)) g_upd_b0 &= (unsigned char)~0xa8;
      if (rpct(20)) g_upd_b0 &= (unsigned char)~0x08;
      if (rpct(20)) g_upd_b0 &= (unsigned char)~0x28;
      if (rpct(50)) {
        alias = *d;
        alias.modem = M_(1 + (int)rmod(2));
        alias.dp_data = g;
        dp = &alias;
        cov_hit(CV_ALIAS_DP);
      }
      g_status = st;
      fill_in(cnt);
      set_regions();
      if (st == 4 || st == 5 || st == 11 || st == 12 || st == 17) cov_hit(CV_ST_TIMEOUT);
      else if (st == 13) {
        if (g->idle != 0) cov_hit(CV_ST_13_BUSY);
        else if (g_upd_b2 & 0x10) cov_hit(CV_ST_13_QC);
        else if (g_upd_b0 & 8) cov_hit(CV_ST_13_90);
        else if (g_upd_b0 & 0x20) cov_hit(CV_ST_13_34);
        else if (g_upd_b0 & 0x80) cov_hit(CV_ST_13_32);
        else cov_hit(CV_ST_13_NONE);
      } else if (st == 16) {
        if (g->id != 0x5c && g->id != 0x5a) cov_hit(CV_ST_16_BAD);
        else if (g->idle != 0) cov_hit(CV_ST_16_BUSY);
        else cov_hit(CV_ST_16_OK);
      } else if (st == 18) cov_hit(CV_ST_HI);
      else cov_hit(CV_ST_DEFAULT);
      if (st == 3) cov_hit(CV_ST_3);
      if (st == 2) cov_hit(CV_ST_2);
      if ((int)st == g->status) cov_hit(CV_ST_SAME);
      else cov_hit(CV_ST_CHG);
      {
        int idle0 = g->idle;
        ret = g_ops->process(dp, g_in, g_out, cnt);
        if (idle0 > 0 && g->idle > 0 && g->idle < idle0) cov_hit(CV_IDLE_COUNT);
        if (idle0 > 0 && g->idle == -1) cov_hit(CV_IDLE_EXPIRE);
        if (idle0 <= 0) cov_hit(CV_IDLE_NONE);
        if (ret == 10) cov_hit(CV_REQ);
      }
      (void)nsp0;
      emit(0, "process st=%u -> %d out=%016llx", st, ret, fnv_shorts(g_out, cnt > 0 ? (size_t)cnt : 0));
      dump_glue(0, "after", d);
    }
    dump_glue(1, "process end", d);
    emit(1, "delete -> %d", g_ops->delete(d));
    trial_end("process");
  }
  cov_finish();
}

/* ================================================================================================ scenario 3: end to end */
static unsigned g_script_pos, g_script_kind;
static unsigned script_status(void)
{
  static const unsigned org[] = {0, 6, 6, 6, 7, 8, 8, 8, 9, 10, 10, 13};
  static const unsigned ans[] = {0, 1, 1, 2, 3, 3, 3, 13};
  static const unsigned qc[] = {0, 6, 7, 8, 9, 14, 15, 15, 13};
  static const unsigned badqc[] = {0, 6, 7, 8, 9, 14, 15, 16};
  static const unsigned tmo[] = {0, 6, 6, 11, 11};
  const unsigned *s;
  unsigned n;
  switch (g_script_kind) {
    case 0: s = org; n = sizeof org / sizeof org[0]; break;
    case 1: s = ans; n = sizeof ans / sizeof ans[0]; break;
    case 2: s = qc; n = sizeof qc / sizeof qc[0]; break;
    case 3: s = badqc; n = sizeof badqc / sizeof badqc[0]; break;
    default: s = tmo; n = sizeof tmo / sizeof tmo[0]; break;
  }
  {
    unsigned p = g_script_pos++ / 3u; /* each status for a few blocks */
    return p < n ? s[p] : s[n - 1];
  }
}
static void sc_e2e(void)
{
  static const int ids[6] = {DP_V8, 0x5a, 0x5c, 0x22, 0x20, 0x5c};
  int t, it;
  cov_def(CV_E2E_SESSION, "end to end: blocks processed");
  cov_def(CV_E2E_REQ, "end to end: DP change requested");
  cov_def(CV_E2E_IDLE_EXPIRE, "end to end: idle timer expired");
  cov_def(CV_E2E_TIMEOUT, "end to end: timeout status (4)");
  cov_def(CV_DBG2, "debug level 2");
  arena_init();
  init_ops();
  dsplibs_debug_level = 2;
  cov_hit(CV_DBG2);
  for (t = 0; t < 10; t++) {
    struct dp *d;
    V8Glue *g;
    rc_reset();
    eng_reset();
    randomize_state();
    g_eng_calls = 0;
    g_script_pos = 0;
    g_script_kind = (unsigned)t % 5u;
    g_status_fn = script_status;
    g_param[5] = 160 + 40 * t;
    g_upd_b0 = (unsigned char)(t & 1 ? 0x20 : 0x08);
    g_upd_b2 = (unsigned char)(t & 2 ? 0x10 : 0);
    g_upd_w10 = 3 + t;
    set_regions();
    d = g_ops->create(M_(t & 1), (enum DP_ID)ids[t % 6], t % 3 == 0 ? 0 : 1, 9600, 160, g_ops);
    if (!d) die3("create e2e");
    g = (V8Glue *)d;
    for (it = 0; it < 80; it++) {
      int cnt = 160, r;
      int idle0 = g->idle;
      fill_in(cnt);
      set_regions();
      r = g_ops->process(d, g_in, g_out, cnt);
      emit(0, "e2e it=%d -> %d idle=%d out=%016llx", it, r, g->idle, fnv_shorts(g_out, (size_t)cnt));
      cov_hit(CV_E2E_SESSION);
      if (r == 10) cov_hit(CV_E2E_REQ);
      if (r == 10 && idle0 > 0 && g->idle == -1) cov_hit(CV_E2E_IDLE_EXPIRE);
      if (r == 4) cov_hit(CV_E2E_TIMEOUT);
      if (it % 10 == 0) dump_glue(0, "e2e", d);
    }
    dump_glue(1, "e2e end", d);
    emit(1, "delete -> %d", g_ops->delete(d));
    trial_end("e2e");
    g_status_fn = NULL;
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
