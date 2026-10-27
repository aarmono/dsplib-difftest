/* R30 harness part 3 (R30_GLUE build): scenarios 4 .. 5 (FAX_create / FAX_delete / FAX_class1_command / FAX_process over scripted fax_class1_* and RcFixed_* recorders).  Included by fax_api.main.c. */
#ifdef R30_GLUE

#define FRING 384

static FaxGlue *g_glue; /* the glue block under test (the recorders tell the buffers apart by it) */
static short g_in_s[1 << 14], g_out_s[1 << 14];

/* ---------------------------------------------------------------------------------------------- scripts */
typedef struct {
  int c1_fail;             /* fax_class1_create returns NULL */
  int rc_fail_at;          /* RcFixed_Create: the n-th call returns NULL (-1: never) */
  int rc_calls;
  int rc_n_in;             /* RcFixed_Resample (input converter): samples produced (< 0: min(nIn, cap)) */
  int ps_n;                /* entries of the progress script */
  int ps_i;                /* next entry */
  int ps_status[16], ps_p7[16], ps_p8[16], ps_nout[16];
  int ps_poke;             /* bit 0: the progress recorder pokes the glue (online / tty_cnt) */
  int cmd_ret;
} GScript;
static GScript g_gs;
static unsigned long g_nprog, g_ncmd1;
static void gscript_reset(void)
{
  memset(&g_gs, 0, sizeof g_gs);
  g_gs.rc_fail_at = -1;
  g_gs.rc_n_in = -1;
  g_gs.ps_n = 1;
  dsplibs_debug_level = g_lvl_base;
  g_tty_in_n = g_tty_in_pos = g_tty_out_n = 0;
  g_recv_cap = -1;
  g_recv_neg = 0;
  g_recv_hook = NULL;
  g_dbg_hook = NULL;
  g_snap = NULL;
}

static u64 snap_cur(void) { return g_glue ? snap_glue(g_glue) : 0ULL; }
static void show_glue(const char *what, const FaxGlue *g) { emit(0, "%s glue=%016llx", what, snap_glue(g)); }

/* ---------------------------------------------------------------------------------------------- the recorders */
/* where a pointer lies in the glue block, as an index (host independent) */
static const char *where(const void *p, char *b, size_t cap)
{
  const FaxGlue *g = g_glue;
  if (!g) snprintf(b, cap, "?");
  else if (p == (const void *)g->rxbuf) snprintf(b, cap, "rxbuf");
  else if (p == (const void *)g->txbuf) snprintf(b, cap, "txbuf");
  else if (p == (const void *)g->tty_out) snprintf(b, cap, "tty_out");
  else if (p == (const void *)g->tty_in) snprintf(b, cap, "tty_in");
  else if ((const char *)p >= (const char *)g->in.data && (const char *)p < (const char *)(g->in.data + FRING)) snprintf(b, cap, "in+%d", (int)((const short *)p - g->in.data));
  else if ((const char *)p >= (const char *)g->out.data && (const char *)p < (const char *)(g->out.data + FRING)) snprintf(b, cap, "out+%d", (int)((const short *)p - g->out.data));
  else if ((const char *)p >= (const char *)g_in_s && (const char *)p < (const char *)g_in_s + sizeof g_in_s) snprintf(b, cap, "ext+%d", (int)((const short *)p - g_in_s));
  else if ((const char *)p >= (const char *)g_out_s && (const char *)p < (const char *)g_out_s + sizeof g_out_s) snprintf(b, cap, "extout+%d", (int)((const short *)p - g_out_s));
  else snprintf(b, cap, "?");
  return b;
}
FaxT30 *fax_class1_create(FaxT30 *o, const FaxClass1Cfg *cfg)
{
  emit(0, "c1_create o=%s mode=%d s7=%d w8=%d wc=%d w10=%d w14=%d", o ? "supplied" : "NULL", cfg->mode, cfg->s7, cfg->cdFlag, cfg->hpFilter, cfg->answerMs, cfg->cngOff);
  if (g_gs.c1_fail) return NULL;
  return (FaxT30 *)sysdep_malloc(72);
}
int fax_class1_delete(FaxT30 *o)
{
  emit(0, "c1_delete %u", hid(o));
  sysdep_free(o);
  return 1;
}
int fax_class1_command(FaxT30 *o, int cmd, int param, int extra)
{
  g_ncmd1++;
  emit(0, "c1_command %u cmd=%d param=%d extra=%d", hid(o), cmd, param, extra);
  return g_gs.cmd_ret;
}
int fax_class1_progress(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8)
{
  char w1[24], w2[24], w3[24], w4[24];
  int i = g_gs.ps_i++ % (g_gs.ps_n > 0 ? g_gs.ps_n : 1), k;
  g_nprog++;
  emit(0, "c1_progress %u rx=%s tx=%s out=%s in=%s n_in=%d n_out=%d p7=%d p8=%d rxh=%016llx inh=%016llx", hid(o), where(rx, w1, sizeof w1), where(tx, w2, sizeof w2),
       where(tty_out, w3, sizeof w3), where(tty_in, w4, sizeof w4), *n_in, *n_out, *p7, *p8, fnv_shorts(H0, rx, 160), fnv_bytes(tty_in, 32));
  for (k = 0; k < 160; k++) tx[k] = (short)(0x300 + k * 3 + g_nprog);
  for (k = 0; k < 64; k++) tty_out[k] = (unsigned char)(0x40 + k + g_nprog);
  if (g_gs.ps_status[i] >= 0 && g_gs.ps_status[i] <= 10) cov_hit(CV_G5_S0 + g_gs.ps_status[i]);
  else cov_hit(CV_G5_SUNK);
  if (*n_in == 160 && *n_out == 160) cov_hit(CV_G5_NIN_SENT);
  *n_out = g_gs.ps_nout[i];
  *p7 = g_gs.ps_p7[i];
  *p8 = g_gs.ps_p8[i];
  if (g_gs.ps_poke & 1 && g_glue) {
    g_glue->online = (int)rmod(2);
    g_glue->tty_cnt = (int)rmod(600);
  }
  return g_gs.ps_status[i];
}
RcFixed *RcFixed_Create(unsigned int idx)
{
  int n = g_gs.rc_calls++;
  if (g_gs.rc_fail_at == n) {
    emit(0, "RcFixed_Create %u -> NULL", idx);
    return NULL;
  }
  emit(0, "RcFixed_Create %u", idx);
  return (RcFixed *)sysdep_malloc((unsigned)sizeof(RcFixed));
}
void RcFixed_Delete(RcFixed *o)
{
  emit(0, "RcFixed_Delete %u", hid(o));
  sysdep_free(o);
}
void RcFixed_Resample(RcFixed *o, const short *in, unsigned int nIn, short *out, unsigned int *pnOut)
{
  unsigned cap = *pnOut, m, i;
  int to_engine = g_glue && (out == g_glue->rxbuf);
  char w1[24], w2[24];
  emit(0, "RcFixed_Resample o=%u dir=%s nIn=%u cap=%u in=%s out=%s inh=%016llx", hid(o), to_engine ? "in" : "out", nIn, cap, where(in, w1, sizeof w1), where(out, w2, sizeof w2),
       fnv_shorts(H0, in, nIn > 0x400 ? 0x400 : nIn));
  if (to_engine) {
    m = g_gs.rc_n_in < 0 ? (nIn < cap ? nIn : cap) : (unsigned)g_gs.rc_n_in;
    if (m > 0xa0) m = 0xa0;
    for (i = 0; i < m; i++) out[i] = (short)(i < nIn ? in[i] : 0x1234);
  } else {
    m = cap;
    for (i = 0; i < m; i++) out[i] = (short)(nIn ? in[i % nIn] + (short)i : (short)(i * 3));
  }
  *pnOut = m;
}

/* ---------------------------------------------------------------------------------------------- helpers */
static FaxGlue *mk_glue(unsigned srate, unsigned caller)
{
  FaxGlue *g = (FaxGlue *)FAX_create(FAKEM, caller, srate);
  if (!g) die3("FAX_create(%u) failed", srate);
  g_glue = g;
  return g;
}
static void kill_glue(FaxGlue *g)
{
  g_glue = NULL;
  FAX_delete(g);
}
static void child_delete_null(void *arg)
{
  (void)arg;
  FAX_delete(NULL);
}
static void guarded_crash(const char *what, int r)
{
  if (g_verbose) fprintf(stderr, "guarded %s: child status %d\n", what, r);
  if (r != 1) die3("%s: expected a crash in both (child status %d)", what, r);
  emit(1, "GUARDED %s crashed (as the blob)", what);
}
static void child_create_fail(void *arg)
{
  FaxGlue *g;
  gscript_reset();
  g_fail_malloc = (int)(intptr_t)arg;
  g = (FaxGlue *)FAX_create(FAKEM, 1, 9600);
  (void)g;
}

/* ================================================================================================ scenario 4: FAX_create / FAX_delete / FAX_class1_command */
static void scen_fax_create(void)
{
  static const unsigned rates[] = {8000, 9600, 48000, 0, 1, 7999, 8001, 9601, 44100, 0xffffffffu, 4800000u, 26843546u, 48001, 16000, 12000, 24000};
  static const unsigned callers[] = {0, 1, 2, 0xffffffffu};
  static const unsigned cmds[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 100, 0x80000000u, 0xffffffffu};
  static const unsigned params[] = {0, 1, 2, 3, 4, 0x17, 0x18, 0x19, 0x30, 0x31, 0x48, 0x49, 0x4a, 0x4b, 0x60, 0x61, 0x62, 0x63, 0x79, 0x7a, 0x7b, 0x91, 0x92, 0x93, 0x100, 0x118,
                                    0x7fffffffu, 0x80000000u, 0xffffffffu, 24, 48, 72, 96, 145};
  unsigned ri, ci, pi;
  int t, rf, f, c1f;
  cov_hit(CV_G4_CREATE);
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (ri = 0; ri < sizeof rates / sizeof rates[0]; ri++)
      for (rf = -1; rf < 3; rf++)
        for (c1f = 0; c1f < 2; c1f++)
          for (f = -1; f < 4; f++) {
            FaxGlue *g;
            unsigned caller = callers[(ri + (unsigned)f + (unsigned)t) % 4];
            gscript_reset();
            g_sreg[7] = rpct(30) ? 7 : (int)rmod(100);
            g_gs.rc_fail_at = rf;
            g_gs.c1_fail = c1f;
            g_fail_malloc = f;
            g = (FaxGlue *)FAX_create(FAKEM, caller, rates[ri]);
            emit(0, "FAX_create rate=%u caller=%u rf=%d c1f=%d mf=%d -> %u", rates[ri], caller, rf, c1f, f, hid(g));
            if (caller == 0) cov_hit(CV_G4_CALLER0);
            else cov_hit(CV_G4_CALLER1);
            if (g) {
              g_glue = g;
              g_snap = snap_cur;
              show_glue("created", g);
              emit(0, "created blk=%u online=%d cnt=%d outc=%u outh=%u", g->blk, g->online, g->tty_cnt, g->out.count, g->out.half);
              g_snap = NULL;
              if (rates[ri] == 8000) cov_hit(CV_G4_R8000);
              else if (rates[ri] == 9600) cov_hit(CV_G4_R9600);
              else if (rates[ri] == 48000) cov_hit(CV_G4_R48000);
              kill_glue(g);
              cov_hit(CV_G4_DELETE);
            } else {
              if (f >= 0) cov_hit(CV_G4_MALLOC_FAIL);
              else if (c1f && (rates[ri] == 8000 || ((rates[ri] == 9600 || rates[ri] == 48000) && rf < 0))) cov_hit(CV_G4_C1_FAIL);
              else if (rf == 0 && (rates[ri] == 9600 || rates[ri] == 48000)) cov_hit(CV_G4_RC1_FAIL);
              else if (rf == 1 && (rates[ri] == 9600 || rates[ri] == 48000)) cov_hit(CV_G4_RC2_FAIL);
              else if (rates[ri] != 8000 && rates[ri] != 9600 && rates[ri] != 48000) cov_hit(CV_G4_RBAD);
            }
            trial_end("create");
          }
  }
  SETLVL(0);
  /* FAX_delete without a class 1 object, with a converter missing */
  for (t = 0; t < 7; t += 2) {
    FaxGlue *g;
    SETLVL(lvl_value(t));
    gscript_reset();
    g = mk_glue(rpct(50) ? 9600 : 8000, 1);
    if (rpct(50)) {
      fax_class1_delete(g->c1);
      g->c1 = NULL;
      cov_hit(CV_G4_DELETE_NOC1);
    }
    if (rpct(30) && g->rcOut) {
      RcFixed_Delete(g->rcOut);
      g->rcOut = NULL;
    }
    kill_glue(g);
    cov_hit(CV_G4_DELETE);
    trial_end("delete");
  }
  SETLVL(0);
  /* FAX_class1_command: every command x parameter x debug level, with / without class 1 object */
  for (t = 0; t < 4; t++) {
    int lv = t == 0 ? 0 : (t == 1 ? 2 : (t == 2 ? 3 : 5));
    SETLVL(lvl_value(lv));
    for (ci = 0; ci < sizeof cmds / sizeof cmds[0]; ci++)
      for (pi = 0; pi < sizeof params / sizeof params[0]; pi++) {
        FaxGlue *g;
        int r;
        gscript_reset();
        g = mk_glue(9600, 1);
        g_gs.cmd_ret = (int)rnd();
        r = FAX_class1_command(g, cmds[ci], params[pi]);
        emit(0, "FAX_class1_command cmd=%u param=%u -> %d", cmds[ci], params[pi], r);
        cov_hit(CV_G4_CMD);
        switch (cmds[ci]) {
          case 0: cov_hit(CV_G4_CMD_FTS); break;
          case 1: cov_hit(CV_G4_CMD_FRS); break;
          case 2: cov_hit(CV_G4_CMD_FTM); break;
          case 3: cov_hit(CV_G4_CMD_FRM); break;
          case 4: cov_hit(CV_G4_CMD_FTH); break;
          case 5: cov_hit(CV_G4_CMD_FRH); break;
          default: cov_hit(CV_G4_CMD_BAD); break;
        }
        if (r == 1) cov_hit(CV_G4_CMD_OK);
        else cov_hit(CV_G4_CMD_REJECT);
        if (pi % 3 == 0) {
          /* no class 1 object */
          fax_class1_delete(g->c1);
          g->c1 = NULL;
          r = FAX_class1_command(g, cmds[ci], params[pi]);
          emit(0, "FAX_class1_command (no c1) cmd=%u param=%u -> %d", cmds[ci], params[pi], r);
          cov_hit(CV_G4_CMD_NOC1);
        }
        kill_glue(g);
        trial_end("cmd");
      }
  }
  SETLVL(0);
  gscript_reset();
  emit(0, "FAX_class1_command(NULL) -> %d", FAX_class1_command(NULL, 0, 0));
  cov_hit(CV_G4_CMD_NULL);
  /* a NULL object crashes FAX_delete in both */
  guarded_crash("FAX_delete(NULL)", guarded(child_delete_null, NULL));
  cov_hit(CV_G4_CRASH);
  /* malloc failure probes of FAX_create: the blob (and the port) fail cleanly */
  {
    int k;
    for (k = 0; k < 5; k++) guarded_report("FAX_create malloc failure", guarded(child_create_fail, (void *)(intptr_t)k));
  }
}

/* ================================================================================================ scenario 5: FAX_process */
static void recv_mut_hook(void)
{
  if (!g_glue) return;
  switch (rmod(4)) {
    case 0: g_glue->online = (int)rmod(2); break;
    case 1: g_glue->tty_cnt = (int)(rpct(50) ? rmod(300) : rnd()); break;
    default: break;
  }
}
static unsigned long g_hook_n;
static void mut_hook(const char *msg)
{
  (void)msg;
  g_hook_n++;
  if (!g_glue) return;
  switch (rmod(5)) {
    case 0: g_glue->online = (int)rmod(2); break;
    case 1: g_glue->tty_cnt = (int)(rpct(50) ? rmod(300) : rnd()); break;
    case 2: dsplibs_debug_level = rpct(50) ? 0 : 1; break;
    default: break;
  }
}
static void fill_in(unsigned n)
{
  unsigned i;
  for (i = 0; i < n; i++) g_in_s[i] = (short)(rpct(10) ? 0 : (short)(rnd() >> 3));
}
static void ps_random(int n_states)
{
  int i;
  g_gs.ps_n = n_states;
  for (i = 0; i < 16; i++) {
    int r = (int)rmod(14);
    g_gs.ps_status[i] = r < 11 ? r : (r == 11 ? -1 : (r == 12 ? 11 + (int)rmod(30) : (int)(rpct(50) ? 0x7fffffff : (int)0x80000000u)));
    g_gs.ps_p7[i] = rpct(30) ? 0 : (rpct(15) ? -(int)rmod(5) : (int)rmod(70));
    g_gs.ps_p8[i] = rpct(30) ? 0 : (rpct(70) ? (int)rmod(300) : (int)rnd());
    g_gs.ps_nout[i] = rpct(70) ? 0xa0 : (int)rmod(300);
  }
}
static const int k_chunks[] = {1, 2, 3, 7, 50, 100, 159, 160, 161, 191, 192, 193, 200, 319, 320, 321, 383, 384, 385, 512, 577, 0, -1, -5};
static void load_tty(unsigned n)
{
  unsigned i;
  for (i = 0; i < n && i < sizeof g_tty_in; i++) g_tty_in[i] = (unsigned char)(rnd() >> 5);
  g_tty_in_n = n;
  g_tty_in_pos = 0;
  g_tty_out_n = 0;
}
static void scen_fax_process(void)
{
  static const unsigned rates[] = {8000, 9600};
  static const int blks[] = {0, 0, 100, 1, 2, 161, 190};
  int t, i, k;
  cov_hit(CV_G5_PROC);
  for (t = 0; t < 7; t++) {
    int lv = t;
    for (i = 0; i < 700; i++) {
      unsigned srate = rates[rmod(2)];
      FaxGlue *g;
      int chunk, ncall, m, r;
      unsigned blk0, wr_before;
      gscript_reset();
      SETLVL(lvl_value(lv));
      g_gs.rc_n_in = rpct(85) ? -1 : (int)rmod(300);
      ps_random(1 + (int)rmod(8));
      if (rpct(50)) g_gs.ps_poke = 1;
      load_tty(rpct(80) ? 2000 + rmod(1000) : rmod(50));
      g_recv_cap = rpct(70) ? -1 : (int)rmod(40);
      if (rpct(6)) g_recv_neg = rpct(50) ? -1 : -(int)rmod(1000);
      g = mk_glue(srate, 1);
      blk0 = g->blk;
      /* a block length other than the creation value: the sample count handed to the class 1 layer is not 160 (the rings keep their 384 samples) */
      if (rpct(15)) {
        unsigned nb = (unsigned)blks[rmod((unsigned)(sizeof blks / sizeof blks[0]))];
        if (nb != 0 && nb <= 192) {
          g->blk = nb;
          g->out.count = nb;
          g->out.half = nb;
        }
      }
      g->online = rpct(60) ? 1 : (rpct(50) ? 0 : (int)rnd());
      g->tty_cnt = rpct(25) ? 0 : (rpct(15) ? (int)rmod(0x1000) : (rpct(20) ? 0x1000 + (int)rmod(5) : (rpct(10) ? (int)(0x80000000u + rmod(5)) : (int)rmod(300))));
      /* the ring words: poked independently (the limits of the chunk arithmetic) */
      if (rpct(40)) {
        unsigned lim = 2 * g->blk;
        g->in.wr = rmod(lim);
        g->in.count = rpct(70) ? g->in.wr % (g->blk + 1) : rmod(lim + 1);
        g->in.half = rpct(50) ? 0 : g->blk;
        g->out.rd = rmod(lim);
        g->out.count = rmod(lim + 1);
        g->out.half = rpct(50) ? 0 : g->blk;
        cov_hit(CV_G5_RING_POKE);
      }
      {
        unsigned q;
        for (q = 0; q < FRING; q++) {
          g->in.data[q] = (short)(0x2000 + q);
          g->out.data[q] = (short)(0x4000 + q);
        }
      }
      (void)blk0;
      if (rpct(40)) {
        g_dbg_hook = mut_hook;
        cov_hit(CV_G5_MUT);
      }
      if (rpct(25)) g_recv_hook = recv_mut_hook;
      g_snap = snap_cur;
      ncall = 1 + (int)rmod(4);
      for (m = 0; m < ncall; m++) {
        chunk = k_chunks[rmod((unsigned)(sizeof k_chunks / sizeof k_chunks[0]))];
        fill_in(1 << 12);
        for (k = 0; k < (1 << 12); k++) g_out_s[k] = (short)0x7777;
        wr_before = g->in.wr;
        r = FAX_process(g, g_in_s, g_out_s, chunk);
        if (chunk > 0 && g->in.wr < wr_before) cov_hit(CV_G5_RING_WRAP);
        emit(0, "FAX_process rate=%u chunk=%d -> %d out=%016llx glue=%016llx", srate, chunk, r, fnv_shorts(H0, g_out_s, 1 << 11), snap_glue(g));
        if (chunk <= 0) cov_hit(CV_G5_ZERO_COUNT);
        if (chunk > (int)g->blk) cov_hit(CV_G5_MULTI);
        if (chunk > 0 && chunk < (int)g->blk) cov_hit(CV_G5_PARTIAL);
        if (r != 0) cov_hit(CV_G5_RET_LAST);
      }
      if (srate == 9600) cov_hit(CV_G5_RC);
      else cov_hit(CV_G5_NORC);
      g_snap = NULL;
      g_recv_hook = NULL;
      g_dbg_hook = NULL;
      SETLVL(0);
      kill_glue(g);
      trial_end("process");
    }
  }
  /* every status code once per glue, in sequence, at level 2 (the messages) and 0 */
  for (t = 0; t < 2; t++) {
    int s;
    FaxGlue *g;
    gscript_reset();
    SETLVL(t == 0 ? 2 : 0);
    g = mk_glue(8000, 1);
    g_glue = g;
    g_gs.ps_n = 14;
    for (s = 0; s < 14; s++) {
      g_gs.ps_status[s] = s < 12 ? s : (s == 12 ? -1 : 77);
      g_gs.ps_p7[s] = s % 3 == 0 ? 0 : 5;
      g_gs.ps_p8[s] = s * 3;
      g_gs.ps_nout[s] = 0xa0;
    }
    load_tty(500);
    g_snap = snap_cur;
    for (s = 0; s < 28; s++) {
      int r;
      fill_in(256);
      g->online = (s & 1);
      g->tty_cnt = 40 + s;
      r = FAX_process(g, g_in_s, g_out_s, 160);
      emit(0, "status %d -> %d online=%d cnt=%d", s % 14, r, g->online, g->tty_cnt);
    }
    g_snap = NULL;
    kill_glue(g);
    SETLVL(0);
    trial_end("status");
  }
  /* the tty paths: online x tty_cnt x byte count of the class 1 layer, with the counts of the recorder calls */
  {
    static const int cnts[] = {0, 1, 40, 0xffe, 0xfff, 0x1000, 0x1001, 0x2000, (int)0x80000000u, -1};
    static const int p7s[] = {0, 5, -1};
    int on, ci, pi;
    for (on = 0; on < 2; on++)
      for (ci = 0; ci < 10; ci++)
        for (pi = 0; pi < 3; pi++) {
          FaxGlue *g;
          unsigned nr0, ns0;
          gscript_reset();
          SETLVL(2);
          g = mk_glue(8000, 1);
          g_gs.ps_n = 1;
          g_gs.ps_p7[0] = p7s[pi];
          g_gs.ps_p8[0] = 7;
          g_gs.ps_nout[0] = 0xa0;
          load_tty(3000);
          g->online = on;
          g->tty_cnt = cnts[ci];
          nr0 = (unsigned)g_nrecv;
          ns0 = (unsigned)g_nsend;
          g_snap = snap_cur;
          fill_in(256);
          FAX_process(g, g_in_s, g_out_s, 160);
          g_snap = NULL;
          emit(0, "tty on=%d cnt=%d p7=%d -> recv %u send %u last=%d", on, cnts[ci], p7s[pi], (unsigned)g_nrecv - nr0, (unsigned)g_nsend - ns0, g_last_recv_n);
          if (on && cnts[ci] == 0 && (unsigned)g_nrecv == nr0) cov_hit(CV_G5_RECV_ZERO);
          if (!on && cnts[ci] != 0 && (unsigned)g_nrecv == nr0) cov_hit(CV_G5_OFFLINE);
          if (on && (unsigned)cnts[ci] > 0x1000u && (unsigned)g_nrecv > nr0 && g_last_recv_n == 0x1000) cov_hit(CV_G5_RECV_CLAMP);
          if ((unsigned)g_nsend == ns0) cov_hit(CV_G5_NOSEND);
          kill_glue(g);
          SETLVL(0);
          trial_end("tty");
        }
  }
}

#endif /* R30_GLUE */
