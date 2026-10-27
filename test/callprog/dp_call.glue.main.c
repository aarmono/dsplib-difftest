/*
 * R11 (id r11_glue): differential test of the call dp glue: call_create / call_run / call_delete / call_GetSRegister (blob LOCAL functions reached, as slmodemd does, through the
 * dp_operations record dp_call_init registers) and dp_call_init / dp_call_exit.  CALLPROG_Create / _Dial / _Progress / _Delete and RcFixed_Create / _Delete / _Resample are scripted
 * recorders (WEAKENED in a copy of dsplibs.o), modem_get_param / modem_get_sreg / modem_dp_register / sysdep_* / dsplibs_debug_printf recorders too: the real dsplibs.o vs
 * src/callprog/dp_call.c.  The glue block is described layout independently (members by name, pointers by pool handle or region + offset).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 registration: dp_call_init / dp_call_exit (the record's fields, the return value of modem_dp_register passed through), the sreg callback (call_GetSRegister) called through
 *     the CALLPROG_Create recorder with registers 0 .. 255 and with garbage in the upper 16 bits
 *   1 create / delete: 400 trials: srate 8000 / 9600 / 48000 / 16000 / 0 / -1, ids, malloc failure, S38 / S16 values, dial strings (digit first, long, empty, not digit first), the
 *     pulse parameters, debug levels; every glue member afterwards; delete (resamplers, glue freed, dp_data cleared)
 *   2 call_run on fabricated objects: 20,000 calls over poked ring states, handles, ended flag, status, scripted CALLPROG_Progress results 0 .. 0x13, 0x11 / 0x12 / big / negative,
 *     cnt = -3 .. 400, chunks around the 0x30 / 0x60 boundaries, every case of the status switch
 *   3 sessions at 8000, 9600 and 48000 Hz: create + 300 call_run calls of 160 / 96 / 47 / 48 / 49 / 1 / odd chunks with scripted CALLPROG results, delete
 * Coverage counters exit 2 on a zero, 3 = harness violation.  Env R11_SEED, R11_VERBOSE=1.
 */
#include "r11_common.h"

enum { CV_REG = 0, CV_DEREG, CV_SREG, CV_SREGHI, CV_CRT, CV_CRT_FAIL, CV_CRT_8K, CV_CRT_9K, CV_CRT_48K, CV_CRT_OTHER, CV_DIG, CV_DIGLONG, CV_NODIG, CV_S38_0, CV_S38_1, CV_S38_3, CV_S38_2, CV_S10_0,
       CV_S10_1, CV_DEL, CV_RUN, CV_RUN_NEG, CV_RUN_ENDED, CV_RUN_RCIN, CV_RUN_NORC, CV_RUN_RCOUT, CV_STAT_CHG, CV_BLOCK, CV_ROOMB, CV_ROOMA, CV_ST0 = 40 /* 40 .. 60: status r */ };

/* ------------------------------------------------------------------------------------------------ pools / identity */
static int g_pool[64];
static int g_npool;
static int g_rcidx[64];
static CallGlue *g_glue;
static const char *dh(const void *p, char *buf)
{
  int i;
  if (!p) return "NULL";
  for (i = 0; i < g_npool; i++)
    if (p == (const void *)&g_pool[i]) {
      sprintf(buf, "rc%d", i);
      return buf;
    }
  return "?";
}
/* a pointer into the glue block described by member + byte offset */
static const char *region(const void *q, char *buf)
{
  const unsigned char *p = (const unsigned char *)q;
  const CallGlue *g = g_glue;
  if (!g) return "?";
#define R(m, name) do { const unsigned char *b = (const unsigned char *)&g->m; if (p >= b && p < b + sizeof g->m) { sprintf(buf, "%s+%u", name, (unsigned)(p - b)); return buf; } } while (0)
  R(stageIn, "stageIn"); R(stageOut, "stageOut"); R(outRing.data, "outRing.data"); R(inRing.data, "inRing.data");
#undef R
  return "ext";
}
static unsigned char *g_ext;  /* external buffers (caller in / out) */
static int g_status_script[24];
static unsigned g_stw;        /* weights: results 0 .. 0x13, 0x40, -1 */
static int g_stwv[24];
static int pick_status(void)
{
  int tot = 0, i, r;
  for (i = 0; i < 24; i++) tot += g_stwv[i];
  if (tot <= 0) return 0;
  r = (int)rmod((unsigned)tot);
  for (i = 0; i < 24; i++) {
    if (r < g_stwv[i]) return i < 20 ? i : i == 20 ? 0x40 : i == 21 ? -1 : i == 22 ? 0x7fffffff : 0x12;
    r -= g_stwv[i];
  }
  return 0;
}

/* ------------------------------------------------------------------------------------------------ recorders */
static int cp_off(const void *cp)
{
  Blk *b = find_blk(cp);
  if (!b || !g_glue) return -1;
  return (const unsigned char *)cp == (const unsigned char *)&((CallGlue *)b->p)->cp ? 0x444 : -2;
}
static int g_create_calls;
static int g_sregfn_probe;
static int (*g_saved_sreg)(struct modem *, int);
void CALLPROG_Create(CallProg *cp, CallProgCfg *cfg)
{
  emit(0, "CPC cp=%d blind=%d m=%d w0c=%d fn=%s", cp_off(cp), cfg->blind, MID(cfg->modem), cfg->w0c, cfg->getSReg ? "set" : "NULL");
  g_saved_sreg = cfg->getSReg;
  g_create_calls++;
  cp->state = 77;
  if (g_sregfn_probe) {
    int r;
    unsigned k;
    for (k = 0; k < 8; k++) {
      int reg = (int)(k < 4 ? k * 60 + 3 : (rnd() | 0x10000u));
      r = cfg->getSReg(cfg->modem, reg);
      emit(0, "CPC probe reg=%d -> %d", reg, r);
    }
  }
}
void CALLPROG_Dial(CallProg *cp, const char *s) { emit(0, "CPD cp=%d s=\"%s\"", cp_off(cp), s ? s : "(null)"); }
int CALLPROG_Progress(CallProg *cp, const short *in, short *out, int n)
{
  char b1[32], b2[32];
  int r = pick_status(), i;
  cov_hit(CV_ST0 + (r >= 0 && r <= 19 ? r : 20));
  emit(0, "CPP cp=%d in=%s out=%s n=%d inh=%016llx -> %d", cp_off(cp), region(in, b1), region(out, b2), n, n > 0 && n <= 0xa0 ? fnv_bytes(in, (size_t)n * 2) : 0ULL, r);
  for (i = 0; i < n && i < 0xa0 && n > 0; i++) out[i] = (short)(in[i] / 3 + (short)((unsigned)r + (unsigned)i));
  return r;
}
void CALLPROG_Delete(CallProg *cp) { emit(0, "CPX cp=%d", cp_off(cp)); }
static int g_rc_fail;
RcFixed *RcFixed_Create(unsigned int idx)
{
  char b[16];
  RcFixed *h;
  if (g_npool >= 64) die3("pool");
  g_rcidx[g_npool] = (int)idx;
  h = (RcFixed *)&g_pool[g_npool++];
  emit(0, "RCC %u -> %s", idx, dh(h, b));
  return h;
}
void RcFixed_Delete(RcFixed *h)
{
  char b[16];
  emit(0, "RCD %s", dh(h, b));
}
void RcFixed_Resample(RcFixed *h, const short *in, unsigned int nIn, short *out, unsigned int *pnOut)
{
  char b[16], b1[32], b2[32];
  unsigned i, nOut, cap = *pnOut;
  int idx = g_rcidx[(int *)h - g_pool];
  switch (idx) {
  case 3: nOut = nIn * 5 / 6; break;
  case 2: nOut = nIn * 6 / 5; break;
  case 5: nOut = nIn / 6; break;
  case 4: nOut = nIn * 6; break;
  default: nOut = nIn; break;
  }
  if (nOut > cap) nOut = cap;
  emit(0, "RCR %s idx=%d in=%s n=%u out=%s cap=%u -> %u inh=%016llx", dh(h, b), idx, region(in, b1), nIn, region(out, b2), cap, nOut, fnv_bytes(in, (size_t)(nIn <= 0xa0 ? nIn : 0) * 2));
  for (i = 0; i < nOut; i++) out[i] = nIn ? in[(i * nIn) / (nOut ? nOut : 1) % nIn] : (short)0;
  *pnOut = nOut;
}
static int g_reg_ret = 0;
static struct dp_operations *g_op;
int modem_dp_register(enum DP_ID id, struct dp_operations *op)
{
  emit(0, "REG %d name=%s use=%d create=%d delete=%d process=%d hangup=%d", (int)id, op->name, op->use_count, op->create != 0, op->delete != 0, op->process != 0, op->hangup != 0);
  g_op = op;
  cov_hit(CV_REG);
  return g_reg_ret;
}
void modem_dp_deregister(enum DP_ID id, struct dp_operations *op)
{
  emit(0, "DEREG %d name=%s same=%d", (int)id, op->name, op == g_op);
  cov_hit(CV_DEREG);
}

/* ------------------------------------------------------------------------------------------------ description of the glue */
static u64 hash_glue(const CallGlue *g)
{
  char b[16];
  u64 h = 1469598103934665603ULL;
  const CallProg *cp = &g->cp;
  h = fnv_mix(h, (unsigned)g->dp.id);
  h = fnv_mix(h, g->dp.modem == FAKEM ? 1u : g->dp.modem == NULL ? 0u : 2u);
  h = fnv_mix(h, g->dp.status);
  h = fnv_mix(h, g->dp.op == g_op ? 1u : g->dp.op == NULL ? 0u : 2u);
  h = fnv_mix(h, g->dp.dp_data == (void *)g ? 1u : g->dp.dp_data == NULL ? 0u : 2u);
  h = fnv_bytes(dh(g->rcIn, b), strlen(dh(g->rcIn, b))) ^ (h * 31u);
  h = fnv_bytes(dh(g->rcOut, b), strlen(dh(g->rcOut, b))) ^ (h * 31u);
  h ^= fnv_bytes(g->stageIn, sizeof g->stageIn) * 3u;
  h ^= fnv_bytes(g->stageOut, sizeof g->stageOut) * 5u;
  h = fnv_mix(h, (unsigned)g->outRing.cnt); h = fnv_mix(h, (unsigned)g->outRing.i1); h = fnv_mix(h, (unsigned)g->outRing.i2); h = fnv_mix(h, (unsigned)g->outRing.i3);
  h = fnv_mix(h, (unsigned)g->inRing.cnt); h = fnv_mix(h, (unsigned)g->inRing.i1); h = fnv_mix(h, (unsigned)g->inRing.i2); h = fnv_mix(h, (unsigned)g->inRing.i3);
  h ^= fnv_bytes(g->outRing.data, sizeof g->outRing.data) * 7u;
  h ^= fnv_bytes(g->inRing.data, sizeof g->inRing.data) * 11u;
  h = fnv_mix(h, (unsigned)g->ended); h = fnv_mix(h, (unsigned)g->status);
  h = fnv_mix(h, (unsigned)cp->state);
  h = fnv_mix(h, g->pulse.makeTime); h = fnv_mix(h, g->pulse.breakTime); h = fnv_mix(h, g->pulse.w5bc); h = fnv_mix(h, g->pulse.count); h = fnv_mix(h, g->pulse.counter);
  h = fnv_mix(h, (unsigned)g->pulse.hookOn);
  return h;
}
static u64 snap(void) { return g_glue ? hash_glue(g_glue) : 0ULL; }

static void dbg_hook(const char *msg)
{
  if (strncmp(msg, "call: process: msg ", 19) == 0) cov_hit(CV_STAT_CHG);
  else if (strncmp(msg, "call: create RC", 15) == 0) cov_hit(CV_CRT_OTHER);
}

/* ------------------------------------------------------------------------------------------------ drivers */
static struct dp *do_create(int srate, int id)
{
  struct dp *dp;
  g_glue = NULL;
  dp = g_op->create(FAKEM, (enum DP_ID)id, (int)rnd(), srate, (int)rnd(), g_op);
  cov_hit(CV_CRT);
  if (dp) g_glue = (CallGlue *)dp;
  emit(0, "CREATE srate=%d id=%d -> %s glue=%016llx", srate, id, dp ? "dp" : "NULL", dp ? hash_glue((CallGlue *)dp) : 0ULL);
  if (dp && dp->dp_data != (void *)dp) die3("dp_data is not the glue block");
  return dp;
}
static void init_prm(void)
{
  static const char *const strs[] = {"5551212", "0", "", "T5551212", "p5", "123456789012345678901234567890123456789012345678901234567890123", "1234567890123456789012345678901234567890123456789012345678901234", "12345678901234567890123456789012345678901234567890123456789012",
                                     "123456789012345678901234567890123456789012345678901234567890123456789", "9,,5;", "a1", "#5", "/", ":"};
  memset(g_prm, 0, sizeof g_prm);
  memset(g_sreg, 0, sizeof g_sreg);
  g_prm[0x11] = rrange(8, 40);
  g_prm[0x12] = rpct(80) ? rrange(8, 40) : (long)(int)rnd();
  g_sreg[0x38] = rpct(70) ? rrange(0, 4) : (rpct(50) ? -1 : (int)rnd());
  g_sreg[0x10] = rpct(60) ? rrange(0, 2) : (int)rnd();
  g_dialstr = strs[rmod(sizeof strs / sizeof strs[0])];
  { int i; for (i = 0; i < 256; i++) if (i != 0x38 && i != 0x10) g_sreg[i] = (int)rnd() >> (int)(rnd() & 15); }
}
static void init_status_weights(int kind)
{
  int i;
  for (i = 0; i < 24; i++) g_stwv[i] = 0;
  if (kind == 0) { for (i = 0; i < 24; i++) g_stwv[i] = 1; g_stwv[0] = 60; }
  else if (kind == 1) { g_stwv[0] = 1000; g_stwv[3] = 3; g_stwv[5] = 3; g_stwv[1] = 2; g_stwv[2] = 2; g_stwv[12] = 2; g_stwv[10] = 2; g_stwv[4] = 2; g_stwv[7] = 1; }
  else { g_stwv[rrange(0, 23)] = 10; g_stwv[0] = 3; }
}

static void scen0(void)
{
  int t;
  struct dp *dp;
  for (t = 0; t < 40; t++) {
    g_reg_ret = rpct(50) ? 0 : (int)rnd();
    emit(0, "INIT -> %d", dp_call_init());
    if (strcmp(g_op->name, "call") != 0) die3("name");
    dp_call_exit();
  }
  g_reg_ret = 0;
  dp_call_init();
  g_sregfn_probe = 1;
  for (t = 0; t < 60; t++) {
    init_prm();
    init_status_weights(1);
    dp = do_create(8000, DP_CALLPROG);
    if (dp) {
      g_op->delete(dp);
      g_glue = NULL;
    }
    trial_end("probe");
  }
  cov_hit(CV_SREG); cov_hit(CV_SREGHI);
  emit(1, "scenario 0 done");
}
static void scen1(void)
{
  static const int rates[] = {8000, 9600, 48000, 16000, 0, -1, 44100, 8000, 9600, 48000};
  int t;
  dp_call_init();
  for (t = 0; t < 400; t++) {
    struct dp *dp;
    int sr = rates[rmod(sizeof rates / sizeof rates[0])];
    g_npool = 0;
    init_prm();
    init_status_weights(1);
    dsplibs_debug_level = lvl_value((int)rmod(8));
    if (rpct(8)) g_fail_malloc = 0;
    dp = do_create(sr, rpct(80) ? DP_CALLPROG : (int)rnd());
    if (!dp) {
      cov_hit(CV_CRT_FAIL);
    } else {
      if (sr == 8000) cov_hit(CV_CRT_8K); else if (sr == 9600) cov_hit(CV_CRT_9K); else if (sr == 48000) cov_hit(CV_CRT_48K);
      if (g_dialstr && g_dialstr[0] >= '0' && g_dialstr[0] <= '9') cov_hit(strlen(g_dialstr) > 0x3e ? CV_DIGLONG : CV_DIG); else cov_hit(CV_NODIG);
      { unsigned s = (unsigned)g_sreg[0x38]; cov_hit(s == 0 ? CV_S38_0 : s == 1 ? CV_S38_1 : s == 3 ? CV_S38_3 : CV_S38_2); }
      cov_hit(g_sreg[0x10] == 0 ? CV_S10_0 : CV_S10_1);
      if (rpct(10)) {
        dsplibs_debug_level = 0;
        emit(0, "DELRET %d", g_op->delete(dp));
        cov_hit(CV_DEL);
        emit(0, "DELETED");
      } else {
        dsplibs_debug_level = lvl_value((int)rmod(8));
        emit(0, "DELRET %d", g_op->delete(dp));
        cov_hit(CV_DEL);
      }
      g_glue = NULL;
    }
    trial_end("create");
  }
  emit(1, "scenario 1 done");
}
static void scen2(void)
{
  int t;
  struct dp *dp = NULL;
  unsigned char in[1000], out[1000], inref[1000];
  dp_call_init();
  g_glue = NULL;
  for (t = 0; t < 20000; t++) {
    CallGlue *g;
    int cnt, k;
    if (t % 400 == 0) {
      if (dp) {
        dsplibs_debug_level = 0;
        g_op->delete(dp);
        trial_end("run");
      }
      g_npool = 0;
      init_prm();
      dsplibs_debug_level = 0;
      dp = do_create(rpct(30) ? 8000 : rpct(50) ? 9600 : 48000, DP_CALLPROG);
      if (!dp) die3("create failed");
      init_status_weights(rrange(0, 2));
    }
    g = g_glue;
    dsplibs_debug_level = lvl_value((int)rmod(8));
    /* poke the ring state: indices anywhere (halves are 0 / 0x60 normally) */
    if (rpct(70)) {
      g->inRing.cnt = rpct(50) ? (rrange(0, 0x5f) & ~1) : (rrange(0, 0xc0) & ~1);
      if (rpct(25)) g->inRing.cnt = rrange(0x5c, 0x62); /* around the 0x5f test (odd values too) */
      if (rpct(5)) g->inRing.cnt = -rrange(1, 200);      /* a negative count: the test is signed */
      g->inRing.i1 = rpct(70) ? (rpct(50) ? 0 : 0x60) : (int)(rmod(0xc0) & ~1u);
      g->inRing.i3 = rpct(50) ? 0 : 0x60;
      g->outRing.cnt = rrange(0, 0xc0) & ~1;
      g->outRing.i2 = rpct(70) ? (rpct(50) ? 0 : 0x60) : (int)(rmod(0xc0) & ~1u);
      g->outRing.i3 = rpct(50) ? 0 : 0x60;
      for (k = 0; k < 0xc0; k++) { g->inRing.data[k] = (unsigned char)rnd(); g->outRing.data[k] = (unsigned char)rnd(); }
    }
    if (rpct(5)) g->ended = !g->ended;
    if (rpct(10)) g->status = rpct(50) ? 0x12 : (int)(rnd() & 0x1f);
    if (rpct(3)) init_status_weights(rrange(0, 2));
    switch (rmod(8)) {
    case 0: cnt = rrange(-3, 5); break;
    case 1: cnt = 0x30; break;
    case 2: cnt = 0x2f + rrange(0, 3); break;
    case 3: cnt = 160; break;
    case 4: cnt = rrange(1, 400); break;
    default: cnt = rrange(1, 100); break;
    }
    if (cnt > 400) die3("cnt");
    for (k = 0; k < 1000; k++) { in[k] = (unsigned char)rnd(); out[k] = (unsigned char)(0x77 + k); }
    memcpy(inref, in, sizeof in);
    {
      u64 before = hash_glue(g);
      int r = g_op->process(dp, in, out, cnt);
      int used = cnt > 0 ? cnt * 2 : 0;
      if (cnt > 0 && (cnt * 2 > 1000)) die3("buffer");
      if (memcmp(in, inref, sizeof in) != 0) die3("call_run modified its input");
      cov_hit(CV_RUN);
      if (cnt <= 0) cov_hit(CV_RUN_NEG);
      if (g->ended) cov_hit(CV_RUN_ENDED);
      if (g->rcIn) cov_hit(CV_RUN_RCIN); else cov_hit(CV_RUN_NORC);
      if (g->rcOut) cov_hit(CV_RUN_RCOUT);
      emit(0, "RUN t=%d cnt=%d -> %d out=%016llx before=%016llx glue=%016llx", t, cnt, r, fnv_bytes(out, (size_t)(used > 0 ? used + 0 : 0)), before, hash_glue(g));
      for (k = used; k < 1000; k++)
        if (out[k] != (unsigned char)(0x77 + k)) { if (cnt > 0 && (k < used + 2)) continue; die3("call_run wrote past its output (cnt=%d at byte %d)", cnt, k); }
    }
  }
  if (dp) {
    dsplibs_debug_level = 0;
    g_op->delete(dp);
  }
  trial_end("run");
  emit(1, "scenario 2 done");
}
static void scen3(void)
{
  static const int rates[3] = {8000, 9600, 48000};
  int t;
  dp_call_init();
  for (t = 0; t < 60; t++) {
    struct dp *dp;
    int k;
    g_npool = 0;
    init_prm();
    dsplibs_debug_level = rpct(80) ? 0 : 2;
    dp = do_create(rates[t % 3], DP_CALLPROG);
    if (!dp) die3("create");
    init_status_weights(1);
    for (k = 0; k < 300; k++) {
      unsigned char in[960], out[960];
      static const int chunks[] = {160, 96, 47, 48, 49, 1, 160, 33, 192, 3, 160, 120};
      int cnt = rpct(60) ? 160 : chunks[rmod(sizeof chunks / sizeof chunks[0])], j;
      for (j = 0; j < 960; j++) in[j] = (unsigned char)rnd();
      memset(out, 0xa5, sizeof out);
      {
        int r = g_op->process(dp, in, out, cnt);
        emit(0, "SESS t=%d k=%d cnt=%d -> %d out=%016llx glue=%016llx", t, k, cnt, r, fnv_bytes(out, (size_t)cnt * 2), hash_glue(g_glue));
        cov_hit(CV_RUN);
      }
    }
    dsplibs_debug_level = 0;
    g_op->delete(dp);
    g_glue = NULL;
    trial_end("sess");
  }
  emit(1, "scenario 3 done");
}

static void cov_setup(int sc)
{
  int i;
  static char nm[40][40];
  if (sc == 0) { cov_def(CV_REG, "dp_call_init registers"); cov_def(CV_DEREG, "dp_call_exit deregisters"); cov_def(CV_SREG, "sreg callback probed"); cov_def(CV_SREGHI, "sreg callback with high garbage"); cov_def(CV_CRT, "call_create"); }
  if (sc == 1) {
    cov_def(CV_CRT, "call_create"); cov_def(CV_CRT_FAIL, "create: malloc fails"); cov_def(CV_CRT_8K, "create 8000"); cov_def(CV_CRT_9K, "create 9600"); cov_def(CV_CRT_48K, "create 48000");
    cov_def(CV_CRT_OTHER, "create RC debug text"); cov_def(CV_DIG, "dial string with a digit first"); cov_def(CV_DIGLONG, "digit-first dial string longer than 62"); cov_def(CV_NODIG, "dial string not digit first");
    cov_def(CV_S38_0, "S38 = 0"); cov_def(CV_S38_1, "S38 = 1"); cov_def(CV_S38_3, "S38 = 3"); cov_def(CV_S38_2, "S38 other"); cov_def(CV_S10_0, "S16 = 0"); cov_def(CV_S10_1, "S16 != 0"); cov_def(CV_DEL, "call_delete");
  }
  if (sc == 2) {
    cov_def(CV_RUN, "call_run"); cov_def(CV_RUN_NEG, "call_run cnt <= 0"); cov_def(CV_RUN_ENDED, "call_run ended"); cov_def(CV_RUN_RCIN, "call_run with rcIn"); cov_def(CV_RUN_NORC, "call_run without rcIn");
    cov_def(CV_RUN_RCOUT, "call_run with rcOut"); cov_def(CV_STAT_CHG, "status change debug text");
    for (i = 0; i <= 20; i++) { snprintf(nm[i], sizeof nm[i], i == 20 ? "Progress status out of range" : "Progress status %d", i); cov_def(CV_ST0 + i, nm[i]); }
  }
  if (sc == 3) { cov_def(CV_RUN, "call_run"); }
}
int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R11_SEED");
  g_verbose = getenv("R11_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)atoi(sd) * 2654435761u;
  g_lcg += (unsigned)sc * 0x9e3779b9u;
  g_snap = snap;
  g_dbg_hook = dbg_hook;
  cov_setup(sc);
  switch (sc) {
  case 0: scen0(); break;
  case 1: scen1(); break;
  case 2: scen2(); break;
  default: scen3(); break;
  }
  cov_finish();
  return 0;
}
