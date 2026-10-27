/*
 * R02 differential harness, id r02_rt: dp_runtime_create / dp_runtime_delete, dcr_create / dcr_process / dcr_reset / dcr_delete and RD_create / RD_process / RD_delete /
 * RD_ring_details, the real dsplibs.o (32-bit) vs src/core/dp_runtime.c.  Method as test/fsk/dp_v21_b103.main.c (read its header): every callee is a recorder (sysdep_*,
 * modem_get_param, dsplibs_debug_printf, RingDetector_Create / Process / GetLastRing / Delete here); the objects are dumped field by field through the typed structs of
 * src/core/dp_glue.h (DpRuntime / DcrObj keep the i386 layout on every host; RdObj does not and is dumped by member).  `struct dsp_info` is slmodemd's real structure.
 *
 * Scenarios (argv[1]; own process each; exit 2 = a coverage counter is zero, 3 = harness / independent-model violation):
 *   0 dp_runtime: dsp_info combinations (qc_lapm bits, qc_index 0 / non-zero, negative clock deviation, connection types), codec types, the double memset, malloc failing,
 *     dp_runtime_delete; every field of the 0x88-byte block is dumped
 *   1 dcr lifecycle: create (field values), malloc failing, delete (NULL ok), reset, the objects after pokes
 *   2 dcr_process, poked states: flags 0 .. 7 and high bits, states 0 .. 5, counts / sums / thresholds / windows poked (small windows reach every transition fast), dc and
 *     thr extremes, sample patterns (zero, DC + noise, full scale, one non-zero sample), n = 0 / negative / large, debug levels
 *   3 RD: create (rates 8000 / 9600 / others, every codec type 0 .. 20: threshold 1000 / 650 / 850 / -3000), RingDetector_Create failing, malloc failing, process (result 0 /
 *     non-zero: GetLastRing + debug print), RD_ring_details (long out values widened), delete
 *   4 dcr from a fresh object through the whole life with constant-DC input: independent checks (the estimate equals the input DC, the output is zero afterwards)
 */
#include "../r02_common.h"

enum {
  CV_RT_LAPM0 = 0, CV_RT_LAPM1, CV_RT_LAPM_HI, CV_RT_QC0, CV_RT_QCN, CV_RT_DEVNEG, CV_RT_MALLOC, CV_RT_DELETE, CV_RT_CODECS,
  CV_DC_CREATE = 10, CV_DC_MALLOC, CV_DC_DELETE, CV_DC_DELETE_NULL, CV_DC_RESET,
  CV_P_NULL = 20, CV_P_DISABLED, CV_P_S0_COUNT, CV_P_S0_TO1, CV_P_S1_COUNT, CV_P_S1_DONE_EN, CV_P_S1_DONE_DIS, CV_P_S2_COUNT, CV_P_S2_DONE, CV_P_S3, CV_P_S_OTHER, CV_P_ZERO_BLOCK,
  CV_P_N0, CV_P_NNEG, CV_P_RET1, CV_P_RET0, CV_P_SUB, CV_P_NOSUB, CV_P_DBG, CV_P_DC_NEG, CV_P_ABS_EQ_THR, CV_P_FULLSCALE, CV_P_ONE_NONZERO, CV_P_ZERO_THEN_NONZERO, CV_P_S23_NONZERO,
  CV_RD_RATE8000 = 50, CV_RD_RATE9600, CV_RD_RATE_BAD, CV_RD_MALLOC, CV_RD_ENG_FAIL, CV_RD_THR1000, CV_RD_THR650, CV_RD_THR850, CV_RD_THRDEF, CV_RD_PROC0, CV_RD_PROC1, CV_RD_DBG_PRINT,
  CV_RD_DETAILS, CV_RD_DELETE, CV_RD_DBG_CREATE, CV_RD_DBG_DELETE,
  CV_M_CONST = 70, CV_M_DC_EST, CV_M_ZERO_OUT, CV_M_STATE2
};

/* ------------------------------------------------------------------------------------------------ the ring detector recorders */
typedef struct {
  int idx;
  int live;
} FakeRd;
static FakeRd g_rd[8];
static int g_nrd, g_rd_fail;
static int g_rd_ret, g_rd_freq, g_rd_dur;
static int rd_id(const void *p) { return (int)(((const FakeRd *)p) - g_rd); }
RingDetector *RingDetector_Create(int *cfg)
{
  int i;
  emit(0, "RD create cfg=%d,%d,%d,%d,%d,%d", cfg[0], cfg[1], cfg[2], cfg[3], cfg[4], cfg[5]);
  if (g_rd_fail) {
    emit(0, "RD create -> NULL");
    return NULL;
  }
  if (g_nrd >= 8) die3("rd pool");
  i = g_nrd++;
  g_rd[i].idx = i;
  g_rd[i].live = 1;
  emit(0, "RD create -> d%d", i);
  return (RingDetector *)(void *)&g_rd[i];
}
void RingDetector_Delete(RingDetector *rd)
{
  FakeRd *r = (FakeRd *)(void *)rd;
  if (!r->live) die3("rd deleted twice");
  r->live = 0;
  emit(0, "RD delete d%d", rd_id(r));
}
int RingDetector_Process(RingDetector *rd, short *in, unsigned int count)
{
  char d[96];
  describe(in, d, sizeof d);
  emit(0, "RD process d%d in=%s count=%u h=%016llx -> %d", rd_id(rd), d, count, fnv_shorts(in, count < 1024 ? count : 0), g_rd_ret);
  return g_rd_ret;
}
void RingDetector_GetLastRing(RingDetector *rd, int *freq, int *dur)
{
  emit(0, "RD lastring d%d -> %d,%d", rd_id(rd), g_rd_freq, g_rd_dur);
  *freq = g_rd_freq;
  *dur = g_rd_dur;
}
static void rd_reset(void)
{
  g_nrd = 0;
  g_rd_fail = 0;
  memset(g_rd, 0, sizeof g_rd);
}

/* ------------------------------------------------------------------------------------------------ dumps */
static void dump_rt(int show, const char *tag, const DpRuntime *r)
{
  if (!r) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s b0=%02x b1=%02x b2=%02x b3=%02x w4=%d w8=%d wc=%d w10=%d w14=%d w40=%d w44=%d w48=%d w4c=%d b50=%02x w54=%d w58=%d w5c=%d w60=%d w64=%d w68=%d w6c=%d all=%016llx", tag, r->flags0, r->flags1, r->flags2, r->flags3, r->w4,
       r->w8, r->ansLevel, r->qcIndex, r->txMdTime, r->powerReduction, r->minTxPowerReduction, r->connectionType, r->clockDeviation, r->flags50, r->codecType, r->w58, r->w5c, r->minLevel, r->delay, r->extDelay, r->p2Silence, fnv_bytes(r, sizeof *r));
}
static void dump_dcr(int show, const char *tag, const DcrObj *d)
{
  if (!d) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s flags=%02x state=%d dc=%d thr=%d sum=%d count=%d n1=%d n2=%d n3=%d all=%016llx", tag, d->flags, d->state, d->dc, d->thr, d->sum, d->count, d->n1, d->n2, d->n3, fnv_bytes(d, sizeof *d));
}

/* ================================================================================================ scenario 0: dp_runtime */
static void sc_runtime(void)
{
  static const unsigned lapms[] = {0, 1, 2, 3, 0x100, 0x80000001u, 0xfffffffeu};
  static const unsigned qcs[] = {0, 1, 7, 0x80000000u, 0xffffffffu, 3};
  static const long devs[] = {0, 100, -100, 0x7fffffffL, -0x7fffffffL - 1};
  int a, b, c, d = 0;
  cov_def(CV_RT_LAPM0, "qc_lapm bit 0 clear");
  cov_def(CV_RT_LAPM1, "qc_lapm bit 0 set");
  cov_def(CV_RT_LAPM_HI, "qc_lapm with higher bits");
  cov_def(CV_RT_QC0, "qc_index 0 (-> 9)");
  cov_def(CV_RT_QCN, "qc_index non-zero");
  cov_def(CV_RT_DEVNEG, "clock deviation negative");
  cov_def(CV_RT_MALLOC, "malloc fails");
  cov_def(CV_RT_DELETE, "dp_runtime_delete");
  cov_def(CV_RT_CODECS, "codec type values (modem_get_param(m, 6))");
  arena_init();
  g_param[0xb] = (long)(intptr_t)&g_dsp_info;
  g_param_ptr[0xb] = 1;
  for (a = 0; a < 7; a++)
    for (b = 0; b < 6; b++)
      for (c = 0; c < 5; c++) {
        DpRuntime *r;
        char tag[80];
        g_dsp_info.qc_lapm = lapms[a];
        g_dsp_info.qc_index = qcs[b];
        g_dsp_info.clock_deviation = devs[c];
        g_dsp_info.connection_type = rnd();
        g_param[6] = (long)(d % 21) - (d % 7 == 0 ? 5 : 0);
        d++;
        cov_hit(CV_RT_CODECS);
        r = (DpRuntime *)dp_runtime_create(M_(d & 1));
        snprintf(tag, sizeof tag, "runtime lapm=%x qc=%x dev=%ld conn=%u codec=%ld", lapms[a], qcs[b], devs[c], g_dsp_info.connection_type, g_param[6]);
        dump_rt(1, tag, r);
        if (lapms[a] & 1) cov_hit(CV_RT_LAPM1);
        else cov_hit(CV_RT_LAPM0);
        if (lapms[a] > 1) cov_hit(CV_RT_LAPM_HI);
        if (qcs[b] == 0) cov_hit(CV_RT_QC0);
        else cov_hit(CV_RT_QCN);
        if (devs[c] < 0) cov_hit(CV_RT_DEVNEG);
        if (r) {
          dp_runtime_delete(r);
          cov_hit(CV_RT_DELETE);
        }
        trial_end(tag);
      }
  g_fail_malloc = 0;
  {
    void *r = dp_runtime_create(M_(0));
    emit(1, "malloc fails -> %s", r ? "OBJECT" : "NULL");
    if (r) die3("created with failing malloc");
    cov_hit(CV_RT_MALLOC);
  }
  trial_end("malloc");
  cov_finish();
}

/* ================================================================================================ scenario 1: dcr lifecycle */
static void sc_dcr_life(void)
{
  int i;
  cov_def(CV_DC_CREATE, "dcr_create");
  cov_def(CV_DC_MALLOC, "dcr_create: malloc fails");
  cov_def(CV_DC_DELETE, "dcr_delete");
  cov_def(CV_DC_DELETE_NULL, "dcr_delete(NULL)");
  cov_def(CV_DC_RESET, "dcr_reset");
  arena_init();
  for (i = 0; i < 3; i++) {
    DcrObj *d = (DcrObj *)dcr_create();
    if (!d) die3("create");
    cov_hit(CV_DC_CREATE);
    dump_dcr(1, "created", d);
    d->dc = (short)(-1234 + i);
    d->sum = 0x12345 * (i + 1);
    d->count = 777 * (i + 1);
    d->state = i + 1;
    d->flags = (unsigned char)(0xf0 | i);
    dump_dcr(1, "poked", d);
    dcr_reset(d);
    cov_hit(CV_DC_RESET);
    dump_dcr(1, "reset", d);
    dcr_delete(d);
    cov_hit(CV_DC_DELETE);
    trial_end("dcr");
  }
  dcr_delete(NULL);
  cov_hit(CV_DC_DELETE_NULL);
  emit(1, "delete NULL done");
  g_fail_malloc = 0;
  {
    void *d = dcr_create();
    emit(1, "malloc fails -> %s", d ? "OBJECT" : "NULL");
    if (d) die3("created with failing malloc");
    cov_hit(CV_DC_MALLOC);
  }
  trial_end("malloc");
  cov_finish();
}

/* ================================================================================================ scenario 2: dcr_process on poked objects */
#define NB 2200
#define GUARD 16
static short g_raw[GUARD + NB + 64];
#define g_buf (g_raw + GUARD) /* GUARD shorts of 0x7777 in front of the block (an index running below 0 reads / writes them), the padding behind it is 0 or 0x5a5a */
static void gen_block(int n, int kind, int dc)
{
  int i, pad = rpct(50) ? 0 : 0x5a5a;
  for (i = 0; i < GUARD; i++) g_raw[i] = 0x7777;
  for (i = 0; i < NB + 64; i++) g_buf[i] = (short)pad;
  for (i = 0; i < n && i < NB; i++) {
    int v;
    switch (kind) {
      case 0: v = 0; break;
      case 1: v = dc + (int)rmod(41) - 20; break;
      case 2: v = (rnd() & 1) ? 32767 : -32768; break;
      case 3: v = i == n / 2 ? 1 : 0; break;
      case 4: v = dc; break;
      case 5: v = (short)rnd(); break;
      default: v = i < 7 ? 0 : dc + (int)rmod(5); break;
    }
    g_buf[i] = (short)v;
  }
}
static void sc_dcr_process(void)
{
  static const int ns[] = {0, 1, 2, 7, 40, 160, 161, 1000, 2000, -1, -100, 300};
  static const int small[] = {1, 2, 3, 5, 50, 160, 400, 1000};
  int t, it;
  cov_def(CV_P_NULL, "dcr_process(NULL)");
  cov_def(CV_P_DISABLED, "flag bit 0 clear (disabled)");
  cov_def(CV_P_S0_COUNT, "state 0: counting");
  cov_def(CV_P_S0_TO1, "state 0 -> 1");
  cov_def(CV_P_S1_COUNT, "state 1: accumulating");
  cov_def(CV_P_S1_DONE_EN, "state 1 -> 2 (flag bit 2 set, 'enabled')");
  cov_def(CV_P_S1_DONE_DIS, "state 1 -> 3 (flag bit 2 clear, 'disabled')");
  cov_def(CV_P_S2_COUNT, "state 2: accumulating");
  cov_def(CV_P_S2_DONE, "state 2: window done (dc smoothed)");
  cov_def(CV_P_S3, "state 3: tracking off");
  cov_def(CV_P_S_OTHER, "state > 3");
  cov_def(CV_P_ZERO_BLOCK, "state 2 / 3: all-zero block (returns 0)");
  cov_def(CV_P_N0, "n == 0");
  cov_def(CV_P_NNEG, "n < 0");
  cov_def(CV_P_RET1, "returns 1 (|dc| >= thr)");
  cov_def(CV_P_RET0, "returns 0");
  cov_def(CV_P_SUB, "flag bit 1: dc subtracted");
  cov_def(CV_P_NOSUB, "flag bit 1 clear: block untouched");
  cov_def(CV_P_DBG, "debug message printed");
  cov_def(CV_P_DC_NEG, "dc negative");
  cov_def(CV_P_ABS_EQ_THR, "|dc| == thr");
  cov_def(CV_P_FULLSCALE, "full-scale block");
  cov_def(CV_P_ONE_NONZERO, "single non-zero sample in a state 2 / 3 block");
  cov_def(CV_P_S23_NONZERO, "state 2 / 3 block with data");
  arena_init();
  {
    int r = dcr_process(NULL, g_buf, 10);
    emit(1, "process NULL -> %d", r);
    cov_hit(CV_P_NULL);
  }
  for (t = 0; t < 40; t++) {
    DcrObj *d = (DcrObj *)dcr_create();
    if (!d) die3("create");
    d->n1 = small[rmod(8)];
    d->n2 = small[rmod(8)];
    d->n3 = small[rmod(8)];
    if (t < 4) { /* the real constants for a few objects */
      d->n1 = 0x1680;
      d->n2 = 0x2580;
      d->n3 = 0x4b00;
    }
    for (it = 0; it < 120; it++) {
      int n = ns[rmod(12)], kind = (int)rmod(7), dc = (int)rmod(8000) - 4000, ret;
      static const int states[] = {0, 1, 2, 3, 4, 5, 0, 1, 2, 2, 3};
      static const int thrs[] = {3000, 0, 1, 100, 32767, -5, 3000};
      int lvl = (int)rmod(5);
      if (rpct(25)) d->flags = (unsigned char)rnd();
      else d->flags = (unsigned char)(rpct(30) ? 7 : (rnd() & 7));
      if (rpct(50)) d->state = states[rmod(11)];
      if (rpct(40)) d->dc = (short)(rpct(10) ? (rpct(50) ? -32768 : 32767) : (int)rmod(8000) - 4000);
      if (rpct(40)) d->thr = (short)thrs[rmod(7)];
      if (rpct(30)) d->sum = (int)rmod(60000) - 30000;
      if (rpct(30)) d->count = (int)rmod((unsigned)(d->n3 + 5));
      if (rpct(10)) d->count = -3;
      if (rpct(8)) d->thr = d->dc < 0 ? (short)(-(int)d->dc) : d->dc;
      if (rpct(25)) { /* dc around +-thr, +-1, 0 */
        static const int edge[] = {-1, 0, 1, -2, 2};
        int e = rpct(50) ? edge[rmod(5)] : (int)d->thr + (int)rmod(5) - 2;
        if (rpct(50)) e = -e;
        d->dc = (short)e;
      }
      dsplibs_debug_level = lvl_value(lvl);
      gen_block(n, kind, dc);
      if (kind == 2) cov_hit(CV_P_FULLSCALE);
      {
        DcrObj before = *d;
        short save[GUARD + NB + 64];
        unsigned long ndbg0 = g_lines;
        memcpy(save, g_raw, sizeof save);
        ret = dcr_process(d, g_buf, n);
        (void)ndbg0;
        emit(0, "process n=%d kind=%d -> %d buf=%016llx", n, kind, ret, fnv_shorts(g_raw, GUARD + NB + 64));
        dump_dcr(0, "after", d);
        if (!(before.flags & 1)) cov_hit(CV_P_DISABLED);
        else {
          int st = before.state;
          if (st == 0) { cov_hit(CV_P_S0_COUNT); if (d->state == 1) cov_hit(CV_P_S0_TO1); }
          else if (st == 1) {
            cov_hit(CV_P_S1_COUNT);
            if (d->state == 2) cov_hit(CV_P_S1_DONE_EN);
            if (d->state == 3) cov_hit(CV_P_S1_DONE_DIS);
          } else if (st == 2) {
            cov_hit(CV_P_S2_COUNT);
            if (d->count == 0 && d->sum == 0 && before.count + n >= before.n3) cov_hit(CV_P_S2_DONE);
          } else if (st == 3) cov_hit(CV_P_S3);
          else cov_hit(CV_P_S_OTHER);
          if ((st == 2 || st == 3) && n > 0) {
            int z = 1, i2;
            for (i2 = 0; i2 < n; i2++) if (save[GUARD + i2] != 0) z = 0;
            if (z) cov_hit(CV_P_ZERO_BLOCK);
            else {
              cov_hit(CV_P_S23_NONZERO);
              if (kind == 3) cov_hit(CV_P_ONE_NONZERO);
            }
          }
          if (n == 0) cov_hit(CV_P_N0);
          if (n < 0) cov_hit(CV_P_NNEG);
          if (ret == 1) cov_hit(CV_P_RET1);
          if (before.flags & 2) cov_hit(CV_P_SUB);
          else cov_hit(CV_P_NOSUB);
          if (d->dc < 0) cov_hit(CV_P_DC_NEG);
          {
            int a = d->dc < 0 ? -(int)d->dc : d->dc;
            if (a == d->thr) cov_hit(CV_P_ABS_EQ_THR);
          }
          if (ret == 0) cov_hit(CV_P_RET0);
          if (lvl > 1 && before.state == 1 && (d->state == 2 || d->state == 3)) cov_hit(CV_P_DBG);
        }
        /* nothing beyond buf[n) may change */
        {
          int i2;
          for (i2 = 0; i2 < GUARD + NB + 64; i2++) {
            int idx = i2 - GUARD;
            if ((idx < 0 || idx >= n) && g_raw[i2] != save[i2]) die3("dcr_process wrote outside [0, n=%d) at %d", n, idx);
          }
        }
      }
    }
    dcr_delete(d);
    trial_end("dcr_process");
  }
  /* state 2 smoothing at the rounding boundary: dc' = (dc * 0x7333 + q * 0xccd + 0x4000) >> 15 with the low 15 bits of dc * 0x7333 + q * 0xccd at 0x3ffd .. 0x4003 */
  {
    static const unsigned tg[] = {0x3ffd, 0x3ffe, 0x3fff, 0x4000, 0x4001, 0x4002, 0x4003, 0x7fff, 0x0000};
    unsigned k2;
    for (k2 = 0; k2 < sizeof tg / sizeof tg[0]; k2++) {
      int found = 0, tries = 0, dcv = 0, q = 0;
      /* dc * 0x7333 + q * 0xccd = 0x8000 * dc + 0xccd * (q - dc): the residue is 0xccd * (q - dc) mod 0x8000, so choose the difference */
      unsigned inv = 0, dif;
      for (inv = 1; inv < 0x8000u; inv += 2) if ((inv * 0xccdu) % 0x8000u == 1u) break;
      dif = (tg[k2] * inv) % 0x8000u;
      while (!found && tries < 100000) {
        int diff = rpct(50) ? (int)dif : (int)dif - 0x8000;
        dcv = (int)rmod(65536) - 32768;
        q = dcv + diff;
        if (q >= -32768 && q <= 32767 && (unsigned)(dcv * 0x7333 + q * 0xccd) % 0x8000u == tg[k2]) found = 1;
        tries++;
      }
      if (!found) die3("no rounding case for %x", tg[k2]);
      {
        DcrObj *d = (DcrObj *)dcr_create();
        int i2, ret;
        d->n3 = 4;
        d->state = 2;
        d->dc = (short)dcv;
        d->thr = 32767;
        gen_block(4, 4, q);
        ret = dcr_process(d, g_buf, 4);
        emit(0, "rounding target %x dc=%d q=%d -> dc=%d ret=%d", tg[k2], dcv, q, d->dc, ret);
        for (i2 = 0; i2 < 4; i2++) emit(0, "  out[%d]=%d", i2, g_buf[i2]);
        dcr_delete(d);
        trial_end("rounding");
      }
    }
  }
  cov_finish();
}

/* ================================================================================================ scenario 3: RD */
static void sc_rd(void)
{
  static const unsigned rates[] = {8000, 9600, 7777, 0, 48000};
  int a, c, i;
  cov_def(CV_RD_RATE8000, "RD_create rate 8000");
  cov_def(CV_RD_RATE9600, "RD_create rate 9600");
  cov_def(CV_RD_RATE_BAD, "RD_create other rate (NULL)");
  cov_def(CV_RD_MALLOC, "RD_create: malloc fails");
  cov_def(CV_RD_ENG_FAIL, "RD_create: RingDetector_Create fails");
  cov_def(CV_RD_THR1000, "threshold 1000 (codec 4, 12)");
  cov_def(CV_RD_THR650, "threshold 650 (codec 13, 15)");
  cov_def(CV_RD_THR850, "threshold 850 (codec 14)");
  cov_def(CV_RD_THRDEF, "threshold -3000 (other codecs)");
  cov_def(CV_RD_PROC0, "RD_process result 0");
  cov_def(CV_RD_PROC1, "RD_process result non-zero (GetLastRing)");
  cov_def(CV_RD_DBG_PRINT, "RD_process debug print");
  cov_def(CV_RD_DETAILS, "RD_ring_details");
  cov_def(CV_RD_DELETE, "RD_delete");
  cov_def(CV_RD_DBG_CREATE, "RD_create debug level 2");
  cov_def(CV_RD_DBG_DELETE, "RD_delete debug level 2");
  arena_init();
  for (a = 0; a < 5; a++)
    for (c = -2; c < 21; c++) {
      void *o;
      char tag[80];
      int level = (a + c + 2) % 5;
      rd_reset();
      dsplibs_debug_level = lvl_value(level);
      g_param[6] = c;
      o = RD_create(M_((a + c) & 1), rates[a]);
      snprintf(tag, sizeof tag, "RD create rate=%u codec=%d level=%d", rates[a], c, level);
      emit(1, "%s -> %s", tag, o ? "OBJECT" : "NULL");
      if (level >= 2) cov_hit(CV_RD_DBG_CREATE);
      if (a == 0) cov_hit(CV_RD_RATE8000);
      else if (a == 1) cov_hit(CV_RD_RATE9600);
      else cov_hit(CV_RD_RATE_BAD);
      if (o) {
        RdObj *r = (RdObj *)o;
        emit(1, "  modem=m%d rd=%d", modem_no(r->modem), r->rd ? rd_id(r->rd) : -1);
        if (c == 4 || c == 12) cov_hit(CV_RD_THR1000);
        else if (c == 13 || c == 15) cov_hit(CV_RD_THR650);
        else if (c == 14) cov_hit(CV_RD_THR850);
        else cov_hit(CV_RD_THRDEF);
        for (i = 0; i < 6; i++) {
          int res = (i & 1) ? (int)rmod(5) : 0;
          static short in[256];
          int k;
          long f = 0x5a5a5a5aL, d = -0x5a5a5a5aL;
          for (k = 0; k < 256; k++) in[k] = (short)rnd();
          reg_clear();
          reg_add(in, sizeof in, 2, "IN");
          g_rd_ret = res;
          g_rd_freq = (int)rnd() % 70;
          g_rd_dur = (int)rnd() % 5000 - 100;
          {
            int pr = RD_process(o, in, 100 + i);
            emit(1, "  process -> %d", pr);
            if (pr == 0) cov_hit(CV_RD_PROC0);
            else {
              cov_hit(CV_RD_PROC1);
              if (level >= 2) cov_hit(CV_RD_DBG_PRINT);
            }
          }
          RD_ring_details(o, &f, &d);
          emit(1, "  details freq=%ld dur=%ld", f, d);
          cov_hit(CV_RD_DETAILS);
        }
        if (level >= 2) cov_hit(CV_RD_DBG_DELETE);
        RD_delete(o);
        cov_hit(CV_RD_DELETE);
      }
      trial_end(tag);
    }
  dsplibs_debug_level = 2;
  g_param[6] = 4;
  rd_reset();
  g_fail_malloc = 0;
  {
    void *o = RD_create(M_(0), 8000);
    emit(1, "malloc fails -> %s", o ? "OBJECT" : "NULL");
    if (o) die3("created with failing malloc");
    cov_hit(CV_RD_MALLOC);
  }
  trial_end("malloc");
  rd_reset();
  g_rd_fail = 1;
  {
    void *o = RD_create(M_(0), 9600);
    emit(1, "RingDetector_Create fails -> %s", o ? "OBJECT" : "NULL");
    if (o) die3("created with failing engine");
    cov_hit(CV_RD_ENG_FAIL);
  }
  trial_end("engfail");
  cov_finish();
}

/* ================================================================================================ scenario 4: dcr model */
static void sc_dcr_model(void)
{
  static const int dcs[] = {0, 5, -5, 100, -300, 2999, -2999, 3000, -3000, 4000, 12000, -20000, 32767, -32768};
  int k;
  cov_def(CV_M_CONST, "constant-DC streams run to state 2");
  cov_def(CV_M_DC_EST, "estimate equals the input DC");
  cov_def(CV_M_ZERO_OUT, "output zero after the estimate");
  cov_def(CV_M_STATE2, "state 2 smoothing keeps the DC");
  cov_def(CV_P_RET1, "returns 1 (|dc| >= thr)");
  cov_def(CV_P_RET0, "returns 0");
  arena_init();
  for (k = 0; k < (int)(sizeof dcs / sizeof dcs[0]); k++) {
    DcrObj *d = (DcrObj *)dcr_create();
    int dc = dcs[k], blk, total = 0, nblk;
    static const int blks[] = {160, 100, 333, 7, 1};
    int bl = blks[k % 5];
    int reached2 = 0;
    for (nblk = 0; total < 5760 + 9600 + 19200 + 2000; nblk++) {
      int i, ret;
      blk = bl;
      for (i = 0; i < blk; i++) g_buf[i] = (short)dc;
      ret = dcr_process(d, g_buf, blk);
      if (ret) cov_hit(CV_P_RET1);
      else cov_hit(CV_P_RET0);
      total += blk;
      if (d->state == 2) reached2 = 1;
      if (nblk % 97 == 0) {
        emit(0, "model dc=%d blk=%d total=%d state=%d est=%d out0=%d ret=%d", dc, blk, total, d->state, d->dc, g_buf[0], ret);
      }
      if (d->state >= 2 && total > 5760 + 9600 + blk) {
        /* from the first estimate on: dc is exact for a constant input, the subtracted output is zero (flag bit 1 set by dcr_create) */
        if (d->dc != dc) die3("model: estimate %d != input %d (dc=%d total=%d)", d->dc, dc, dc, total);
        cov_hit(CV_M_DC_EST);
        for (i = 0; i < blk; i++)
          if (g_buf[i] != 0) die3("model: output %d not zero (dc=%d)", g_buf[i], dc);
        cov_hit(CV_M_ZERO_OUT);
        if (d->state == 2 && total > 5760 + 9600 + 19200 + blk) cov_hit(CV_M_STATE2);
      }
    }
    if (reached2) cov_hit(CV_M_CONST);
    dump_dcr(1, "model end", d);
    dcr_delete(d);
    trial_end("model");
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
    case 0: sc_runtime(); break;
    case 1: sc_dcr_life(); break;
    case 2: sc_dcr_process(); break;
    case 3: sc_rd(); break;
    case 4: sc_dcr_model(); break;
    default: die3("no such scenario %d", sc);
  }
  return 0;
}
