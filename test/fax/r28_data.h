/* R28 harness part 3: scenario 3 (the data transmit states) and scenario 4 (the data transmitter life cycle).  Included by fax_t30_tx.main.c. */


/* an object with a data channel: recorder channels, a real FIFO, a config block as _init_transmitter leaves it */
static void data_object(FaxT30 *o, int data_id, unsigned fifo_size, unsigned fill)
{
  t30_garbage(o);
  t30_channels(o, data_id);
  o->fifo = new_fifo(safe_fifo_size(fifo_size), fill);
  o->frameWords = rpct(10) ? 0 : (rpct(30) ? 36 : (int)rmod(70));
  o->sec = (int)rmod(100);
  o->ms = (int)rmod(100);
}
static void script_data(void)
{
  unsigned a;
  script_random();
  a = rmod(100);
  g_rs.proc_ret = rnd() & 0x00feffffu; /* clear the flags then set what the script wants */
  if (a < 40) g_rs.proc_ret |= 0x1000000u;
  a = rmod(100);
  if (a < 40) g_rs.proc_ret |= 0x2000000u;
  a = rmod(100);
  if (a < 50) g_rs.proc_ret |= 0x100u;
  else g_rs.proc_ret &= ~0x100u;
  g_rs.hdi_nset = rpct(85);
  if (g_rs.hdi_k > 60) g_rs.hdi_k = 60;
}

static void data_single(void)
{
  int t;
  static const int e_nul[] = {0xfa};
  for (t = 0; t < 5000; t++) {
    FaxT30 *o = &g_t30;
    int which, p8;
    unsigned dbg, fsize, fill;
    fsize = rpct(30) ? 64 : (unsigned)(40 + rmod(300));
    fill = rpct(25) ? 0 : (rpct(25) ? fsize : (unsigned)rmod(fsize + 1));
    data_object(o, 11, fsize, fill);
    script_data();
    dbg = rmod(4);
    dsplibs_debug_level = dbg;
    which = (int)rmod(4);
    o->enableDelay = rpct(40) ? 0 : (rpct(40) ? 1 : (rpct(30) ? 2 : (int)rmod(5) - 2));
    o->txEnable = rpct(50) ? 0 : (int)rnd();
    o->txConnected = rpct(50) ? 0 : (int)rnd();
    o->fifoReady = rpct(50) ? 0 : (int)rnd();
    o->lastByte = rpct(50) ? 0 : (int)rmod(256);
    o->silenceTime = rpct(20) ? 0 : (int)rmod(500);
    o->state = (int)rmod(20);
    o->status = (int)rnd();
    o->tmr = rpct(40) ? pick_tmr(e_nul, 1) : (int)rmod(30);
    COUNTER = rpct(30) ? 0 : (rpct(30) ? 1 : (rpct(10) ? -1 : (int)rmod(8)));
    p8 = rpct(35) ? 0 : (rpct(8) ? -(int)(1 + rmod(5)) : (rpct(10) ? 0x100 : (int)(1 + rmod(40))));
    io_prep(&g_io, 160, rpct(70) ? 0xa0 : (int)rnd(), (int)rnd(), p8);
    emit(0, "DS t=%d dbg=%u which=%d", t, dbg, which);
    switch (which) {
      case 0: {
        int w1270 = o->enableDelay;
        call_state(_tx_scrambled_ones_state, o, &g_io, "ones");
        cov_hit(CV_D_ONES);
        if (COUNTER == 1) cov_hit(CV_D_ONES_FIRST);
        if (w1270 > 0) cov_hit(CV_D_ONES_DEC);
        if (w1270 == 1) cov_hit(CV_D_ONES_ENABLE);
        if (p8 > 0) cov_hit(CV_D_ONES_DATA);
        else cov_hit(CV_D_ONES_NODATA);
        if (o->state == 10) cov_hit(CV_D_ONES_GO);
        if (o->txConnected == 1 && o->enableDelay == 2) cov_hit(CV_D_ONES_CONNECT);
        if (dbg > 2) cov_hit(CV_D_ONES_DBG3);
        break;
      }
      case 1:
        call_state(_tx_data_state, o, &g_io, "data");
        cov_hit(CV_D_DATA);
        if (p8 > 0) cov_hit(CV_D_DATA_IN);
        if (dbg > 2) cov_hit(CV_D_DATA_DBG3);
        if (g_rs.proc_ret & 0x1000000u) {
          if (o->state == 8) cov_hit(CV_D_DATA_QEMPTY_STOP);
          if (o->state == 11) cov_hit(CV_D_DATA_QEMPTY_NULLS);
        }
        if (g_rs.proc_ret & 0x2000000u) cov_hit(CV_D_DATA_QFULL);
        break;
      case 2:
        call_state(_tx_nulls_state, o, &g_io, "nulls");
        cov_hit(CV_D_NULLS);
        if (o->state == 8 && o->status == 5) cov_hit(CV_D_NULLS_TO);
        if (p8 > 0) cov_hit(CV_D_NULLS_IN);
        else cov_hit(CV_D_NULLS_NOIN);
        if (p8 < 0) cov_hit(CV_D_NULLS_NEG);
        break;
      default:
        o->tmr = rpct(50) ? (int)rmod(40) : (int)rnd();
        call_state(_tx_silence_before_scrm_ones, o, &g_io, "silence_ones");
        cov_hit(CV_D_SIL);
        if (o->state == 9) cov_hit(CV_D_SIL_END);
        break;
    }
    t30_free_channels(o);
    trial_end("data_single");
  }
  dsplibs_debug_level = 0;
}

static void run_data_sequence(int tag, int maxcalls, int mod, int sil)
{
  FaxT30 *o = &g_t30;
  int c, ncalls = 0, idle_run = 0;
  unsigned dbg;
  t30_garbage(o);
  t30_channels(o, -1);
  o->sec = 0;
  o->ms = 0;
  o->silenceTime = sil;
  o->dataRate = 0;
  o->lastByte = rpct(50) ? 0 : 1;
  script_data();
  dbg = rmod(4);
  dsplibs_debug_level = dbg;
  poke_consts(rpct(30) ? 1 : 0);
  emit(0, "DSEQ %d mod=%02x sil=%d dbg=%u", tag, mod, sil, dbg);
  _tx_scrambled_ones_init(o, mod);
  dump_t30("dseq_init", o);
  for (c = 0; c < maxcalls; c++) {
    StateFn f = state_fn(o->state);
    int prev = o->state, p8, burst;
    if (!f) {
      emit(0, "DSEQ %d stop at state %d after %d calls", tag, o->state, ncalls);
      break;
    }
    script_data();
    burst = (c / 7) % 3;
    g_rs.proc_ret = (g_rs.proc_ret & ~0x1000000u);
    if (rpct(burst == 2 ? 30 : 6)) g_rs.proc_ret |= 0x1000000u;
    if (rpct(burst == 1 ? 40 : 8)) g_rs.proc_ret |= 0x2000000u;
    g_rs.hdi_k = rpct(10) ? -1 : (int)rmod(burst == 0 ? 60 : 12);
    g_rs.hdi_ret = 1;
    p8 = burst == 0 && rpct(70) ? (int)(1 + rmod(40)) : (burst == 1 ? 0 : (rpct(30) ? (int)rmod(10) : 0));
    io_prep(&g_io, 160, 0xa0, 0, p8);
    clock_tick(o);
    call_state(f, o, &g_io, "dseq");
    ncalls++;
    if (o->state != prev) emit(0, "DSEQ %d call %d state %d -> %d", tag, c, prev, o->state);
    cov_hit(CV_D_SEQ);
    if (prev == 9) cov_hit(CV_D_SEQ_STATE9);
    if (prev == 10) cov_hit(CV_D_SEQ_STATE10);
    if (prev == 11) cov_hit(CV_D_SEQ_STATE11);
    if (prev == 18) cov_hit(CV_D_SEQ_STATE18);
    if (prev == 8) cov_hit(CV_D_SEQ_STATE8);
    if (o->txConnected == 1) cov_hit(CV_D_SEQ_CONNECT);
    if (o->state == 8) idle_run++;
    else idle_run = 0;
    if (idle_run > 4) {
      emit(0, "DSEQ %d idle after %d calls", tag, ncalls);
      break;
    }
  }
  _delete_data_tx_modem(o);
  cov_hit(CV_L_DEL);
  dump_t30("dseq_del", o);
  t30_free_channels(o);
  trial_end("dseq");
  dsplibs_debug_level = 0;
  poke_consts(0);
}
static void data_sequences(void)
{
  int t;
  for (t = 0; t < 300; t++) run_data_sequence(t, t % 12 == 0 ? 900 : 260, k_mods[t % NMODS], t % 3 == 0 ? 0 : (t % 3 == 1 ? 60 : (int)(1 + rmod(400))));
}
static void scen_data(void)
{
  poke_consts(0);
  dump_consts();
  data_single();
  data_sequences();
}

/* ---------------------------------------------------------------------------------------------- scenario 4: life cycle */
static void cov_mod(int mod)
{
  switch (mod) {
    case 0x18: cov_hit(CV_L_M18); break;
    case 0x30: cov_hit(CV_L_M30); break;
    case 0x48: cov_hit(CV_L_M48); break;
    case 0x60: cov_hit(CV_L_M60); break;
    case 0x49: cov_hit(CV_L_M49); break;
    case 0x4a: cov_hit(CV_L_M4A); break;
    case 0x61: cov_hit(CV_L_M61); break;
    case 0x62: cov_hit(CV_L_M62); break;
    case 0x79: cov_hit(CV_L_M79); break;
    case 0x7a: cov_hit(CV_L_M7A); break;
    case 0x91: cov_hit(CV_L_M91); break;
    default: cov_hit(CV_L_M92); break;
  }
}
/* the previous data channel of an object (a config block with its modem block, a recorder channel, a FIFO) */
static void give_channel(FaxT30 *o, int cfg, int vmi, int fifo)
{
  if (cfg) {
    FaxVmiM *m = (FaxVmiM *)sysdep_malloc((unsigned)sizeof(FaxVmiM));
    memset(m, 0, sizeof *m);
    m->s0e = 11;
    m->u10.cfg = sysdep_malloc(0x1c);
    o->data_cfg = m;
  }
  if (vmi) o->data_vmi = new_vmi(11);
  if (fifo) o->fifo = new_fifo(safe_fifo_size(0x800), rmod(30)); /* a reused FIFO is re-created with size 0x800 over its own buffer */
}
static void life_variant(int v)
{
  switch (v) {
    case 0: INITTAB[0] = rec_init_v27; INITTAB[1] = rec_init_v29; INITTAB[2] = rec_init_v17; break;
    case 1: INITTAB[2] = rec_init_v29; break; /* class 2 -> id 9 */
    case 2: INITTAB[2] = rec_init_v27; break; /* class 2 -> id 7 */
    default: INITTAB[2] = rec_init_other; break; /* class 2 -> id 3: the V.27ter record branch */
  }
}
static void life_trial(int tag, int mod, int pre, int variant, int sil, int dbg, int use_ones)
{
  FaxT30 *o = &g_t30;
  t30_garbage(o);
  t30_channels(o, -1);
  switch (pre) {
    case 0: break;
    case 1: give_channel(o, 1, 1, rpct(60)); break;
    case 2: give_channel(o, 1, 0, rpct(60)); break;
    default: give_channel(o, 0, 1, rpct(60)); break;
  }
  life_variant(variant);
  script_random();
  poke_consts(rpct(70) ? 1 : 0);
  o->silenceTime = sil;
  o->sec = (int)rmod(100);
  o->ms = (int)rmod(100);
  o->modemClass = (int)rnd();
  dsplibs_debug_level = (unsigned)dbg;
  emit(0, "LIFE %d mod=%02x pre=%d variant=%d sil=%d dbg=%d ones=%d", tag, mod, pre, variant, sil, dbg, use_ones);
  if (use_ones) {
    int r = _tx_scrambled_ones_init(o, mod);
    emit(0, "LIFE ones r=%d", r);
    cov_hit(CV_L_ONESINIT);
    if (pre == 1 || pre == 2) cov_hit(CV_L_FIFO_REUSE);
  } else {
    _init_transmitter(o, mod);
  }
  cov_hit(CV_L_INIT);
  cov_mod(mod);
  if (pre == 0) cov_hit(CV_L_FRESH);
  if (pre == 1) cov_hit(CV_L_REPLACE);
  if (pre == 2) cov_hit(CV_L_REUSE);
  if (pre == 3) cov_hit(CV_L_SKIP);
  if (mod == 0x4a || mod == 0x62 || mod == 0x7a || mod == 0x92) {
    cov_hit(CV_L_SHORT);
    if (pre != 3) {
      if (variant == 0) cov_hit(CV_L_CTL_V17);
      else if (variant == 1) cov_hit(CV_L_CTL_V29);
      else cov_hit(CV_L_CTL_V27);
    }
  } else {
    cov_hit(CV_L_NOCTL);
  }
  if (sil == 0) cov_hit(CV_L_SIL0);
  else cov_hit(CV_L_SIL1);
  if (dbg > 1) cov_hit(CV_L_DBG);
  dump_t30("life", o);
  /* delete when there is a config block (the blob dereferences it) */
  if (o->data_cfg && rpct(60)) {
    _delete_data_tx_modem(o);
    cov_hit(CV_L_DEL);
    if (!o->fifo) cov_hit(CV_L_DEL_NOFIFO);
    dump_t30("life_del", o);
  }
  t30_free_channels(o);
  trial_end("life");
  dsplibs_debug_level = 0;
  poke_consts(0);
  life_variant(0);
}
/* forked probes: the outcome class of a child that calls the function with the n-th malloc failing: 0 returns, 1 SIGSEGV, 2 other */
static int probe_child(int failk, int fn)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    FaxT30 *o = &g_t30;
    if (freopen("/dev/null", "w", stderr) == NULL) _exit(9);
    g_dbg_on = 0;
    t30_garbage(o);
    t30_channels(o, -1);
    o->silenceTime = 0;
    life_variant(0);
    g_nmsz = 0;
    g_malloc_ctr = 0;
    g_fail_malloc = failk;
    if (fn == 0) _init_transmitter(o, 0x18);
    else _tx_scrambled_ones_init(o, 0x30);
    _exit(0);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) return WTERMSIG(st) == SIGSEGV ? 1 : 2;
  return WEXITSTATUS(st) == 0 ? 0 : 2;
}
static void life_probes(void)
{
  /* fn 0 = _init_transmitter: malloc #0 the 0x18 config block, #1 the modem block (recorder initialiser), #2 the recorder channel;
   * fn 1 = _tx_scrambled_ones_init: those, then #3 the FIFO object, #4 its buffer.  Expected (blob / port): #0 SIGSEGV / returns (the port does not dereference the NULL block), #1 #2 return,
   * #3 #4 SIGSEGV (FIFO_create stores through the NULL of the failed allocation; the port equals the blob) */
  static const struct { int fn, k, blob, port; } pr[] = {
      {0, 0, 1, 0}, {0, 1, 0, 0}, {0, 2, 0, 0}, {1, 0, 1, 0}, {1, 1, 0, 0}, {1, 2, 0, 0}, {1, 3, 1, 1}, {1, 4, 1, 1},
  };
  unsigned i;
  for (i = 0; i < sizeof pr / sizeof pr[0]; i++) {
    int r = probe_child(pr[i].k, pr[i].fn);
    int want = SIDE_BLOB ? pr[i].blob : pr[i].port;
    emit(1, "PROBE fn %d malloc #%d fails: %s", pr[i].fn, pr[i].k, r == want ? "as expected (blob: crash or return, port: safe where the blob crashes)" : "UNEXPECTED");
    if (r != want) die3("probe fn %d #%d: outcome %d, wanted %d", pr[i].fn, pr[i].k, r, want);
    cov_hit(CV_L_PROBE);
  }
}
static void scen_life(void)
{
  int t, pre, variant, m;
  poke_consts(0);
  dump_consts();
  /* every modem x every initial state x init class variants (the short train records come from three different classes) */
  t = 0;
  for (m = 0; m < NMODS; m++)
    for (pre = 0; pre < 4; pre++)
      for (variant = 0; variant < 4; variant++) {
        int sil, dbg, ones;
        for (ones = 0; ones < 2; ones++) {
          int mod = k_mods[m];
          sil = (t & 1) ? 0 : (int)(1 + rmod(50));
          dbg = (t >> 1) & 3;
          if (pre == 3 && (mod == 0x4a || mod == 0x62 || mod == 0x7a || mod == 0x92)) {
            /* a channel without a config block: the blob dereferences the missing block for the record id */
            continue;
          }
          life_trial(t, mod, pre, variant, sil, dbg, ones);
          t++;
        }
      }
  for (m = 0; m < 600; m++) {
    int mod, p, v, ones;
    mod = k_mods[rmod((unsigned)NMODS)];
    p = (int)rmod(4);
    v = (int)rmod(4);
    ones = (int)rmod(2);
    if (p == 3 && (mod == 0x4a || mod == 0x62 || mod == 0x7a || mod == 0x92)) p = 1;
    life_trial(t++, mod, p, v, rpct(50) ? 0 : (int)rmod(300), (int)rmod(4), ones);
  }
  life_probes();
}
