/* R29 harness part 3: scenario 3 (the data receive states _rx_look_carrier_state / _rx_data_state: single calls, dispatched sequences, _rx_look_carrier_init) and scenario 4 (the data
 * receiver life cycle: _init_receiver, _delete_data_rx_modem, forked failure probes).  Included by fax_t30_rx.main.c (REC build). */

/* the data states fill *n_in samples of silence: the sample count must fit the I/O block (the HDLC states' u16 truncation case 0x10000 + 160 does not) */
static int pick_n_in_d(void)
{
  int n = pick_n_in();
  return n > 1000 ? 160 : n;
}
static const int k_ids[3] = {FAXVMI_ID_V27RX, FAXVMI_ID_V29RX, FAXVMI_ID_V17RX};

static void dr_lookcarrier(void)
{
  int t;
  for (t = 0; t < 4000; t++) {
    FaxT30 *o = &g_t30;
    int n_in, to, k, p8, did = k_ids[rmod(3)];
    unsigned dbg = rmod(4);
    t30_garbage(o);
    t30_channels(o, did);
    script_random();
    dsplibs_debug_level = dbg;
    o->s7 = rpct(15) ? 0 : (int)rmod(30);
    n_in = pick_n_in_d();
    to = s7_timeout(o, n_in);
    {
      int edges[1];
      edges[0] = to;
      o->tmr = pick_tmr(edges, 1);
    }
    o->state_prev = (int)rmod(10);
    p8 = pick_p8();
    k = (int)rmod(10);
    if (k < 3) g_rs.ret_d = (g_rs.ret_d & ~0x300u) | 0x100u;
    io_prep(&g_io, n_in, (int)rnd(), (int)rnd(), p8);
    emit(0, "DLC t=%d dbg=%u s7=%d to=%d", t, dbg, o->s7, to);
    {
      unsigned prev_tmr = (unsigned)o->tmr;
      call_state(_rx_look_carrier_state, o, &g_io, "dlc");
      cov_hit(CV_DR_LC);
      if (n_in == 0) cov_hit(CV_DR_LC_N0);
      if (to == 0) cov_hit(CV_DR_LC_TIMEOUT0);
      if (g_rs.ret_d & 0x2000u) cov_hit(CV_DR_LC_DATACARR);
      else if ((g_rs.ret_h & 0x2000u) != 0) cov_hit(CV_DR_LC_HDLC);
      if (((g_rs.ret_d >> 8) & 3u) == 1u) cov_hit(CV_DR_LC_CONNECT);
      if (to != 0 && prev_tmr + 1u > (unsigned)to && !(g_rs.ret_d & 0x2000u)) cov_hit(CV_DR_LC_TIMEOUT);
      else cov_hit(CV_DR_LC_WAIT);
      if (p8 != 0) cov_hit(CV_DR_LC_ABORT);
    }
    t30_free_channels(o);
    trial_end2("dlc");
  }
  dsplibs_debug_level = 0;
}

static void dr_data(void)
{
  int t;
  for (t = 0; t < 4000; t++) {
    FaxT30 *o = &g_t30;
    int n_in, p8, p7, k, did = k_ids[rmod(3)];
    unsigned dbg = rmod(4);
    t30_garbage(o);
    t30_channels(o, did);
    script_random();
    dsplibs_debug_level = dbg;
    p8 = pick_p8();
    n_in = pick_n_in_d();
    k = (int)rmod(10);
    p7 = k < 5 ? 0 : (k < 8 ? (int)rmod(100) : (int)rmod(200)); /* (the word count is bounded by the I/O block: tty_out holds 1024 bytes) */
    k = (int)rmod(4);
    if (k == 0) {
      o->alignDone = 0;
      o->alignHist = 0xffffffffu;
    }
    io_prep(&g_io, n_in, (int)rnd(), p7, p8);
    emit(0, "DDT t=%d dbg=%u", t, dbg);
    call_state(_rx_data_state, o, &g_io, "ddt");
    cov_hit(CV_DR_DT);
    if (g_rs.ret_d & 0x2000u) cov_hit(CV_DR_DT_CARR);
    else cov_hit(CV_DR_DT_NOCARR);
    if (p8 != 0) cov_hit(CV_DR_DT_ABORT);
    if (g_io.p7 == 0) cov_hit(CV_DR_DT_N0);
    else cov_hit(CV_DR_DT_WORDS);
    t30_free_channels(o);
    trial_end2("ddt");
  }
  dsplibs_debug_level = 0;
}

static void dr_init(void)
{
  int t;
  for (t = 0; t < 60; t++) {
    FaxT30 *o = &g_t30;
    int mod = k_mods[rmod((unsigned)NMODS)];
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    dsplibs_debug_level = rmod(4);
    emit(0, "DINIT t=%d mod=%02x", t, mod);
    {
      int r = _rx_look_carrier_init(o, mod);
      emit(0, "DINIT r=%d", r);
    }
    dump_t30("dinit", o);
    cov_hit(CV_DR_INIT);
    t30_free_channels(o);
    trial_end2("dinit");
  }
  dsplibs_debug_level = 0;
}

static void dr_sequences(void)
{
  int t;
  g_seq_scen = 3;
  for (t = 0; t < 200; t++) {
    int mode = (t % 3 == 0) ? 4 : ((t % 3 == 1) ? 5 : (t % 6 == 2 ? 6 : 0));
    run_sequence_rx(t, t % 10 == 0 ? 1200 : 450, mode, mode == 5 ? k_ids[rmod(3)] : -1);
  }
}

static void scen_data(void)
{
  poke_consts(0);
  dump_consts();
  dr_lookcarrier();
  dr_data();
  dr_init();
  dr_sequences();
}

/* ---------------------------------------------------------------------------------------------- scenario 4: the life cycle */
/* an object in one of the initial states: 0 fresh (no config block, no channel), 1 a receiver of the same class exists (created by _init_receiver itself), 2 a receiver of another class exists,
 * 3 a config block without a channel (the blob dereferences the NULL channel), 4 a channel without a config block (overwritten by the new one) */
static int life_variant(FaxT30 *o, int mod, int pre)
{
  int cls_mod[3] = {0x18, 0x48, 0x49}, c = mod_class_id(mod) == FAXVMI_ID_V27RX ? 0 : (mod_class_id(mod) == FAXVMI_ID_V29RX ? 1 : 2);
  int other = cls_mod[(c + 1 + (int)rmod(2)) % 3];
  switch (pre) {
    case 0:
      o->modemClass = (int)rnd();
      return 0;
    case 1:
      g_dbg_on = 0;
      _init_receiver(o, cls_mod[c]);
      g_dbg_on = 1;
      return 0;
    case 2:
      g_dbg_on = 0;
      _init_receiver(o, other);
      g_dbg_on = 1;
      return 0;
    case 3:
      g_dbg_on = 0;
      _init_receiver(o, cls_mod[c]);
      g_dbg_on = 1;
      FAXVMI_delete(o->data_vmi);
      o->data_vmi = NULL;
      return 1; /* the blob crashes */
    default:
      g_dbg_on = 0;
      _init_receiver(o, cls_mod[c]);
      g_dbg_on = 1;
      {
        FaxVmiM *cf = o->data_cfg;
        if ((unsigned short)cf->s0e == FAXVMI_ID_V17RX) {
          V17RxCfg *vc = (V17RxCfg *)cf->u10.cfg;
          sysdep_free(vc->freq);
          sysdep_free(vc->coefQ);
          sysdep_free(vc->coefI);
        }
        sysdep_free(cf->u10.cfg);
        sysdep_free(cf);
        o->data_cfg = NULL;
      }
      return 0;
  }
}
static void life_trial(int tag, int mod, int pre, int dbg)
{
  FaxT30 *o = &g_t30;
  int crash;
  t30_garbage(o);
  t30_channels(o, -1);
  script_random();
  o->cd_flag = rpct(30) ? 0 : (int)rnd();
  o->agc_rcp = (int)rnd();
  o->agc_shift = (int)rnd();
  poke_consts(rpct(40) ? 1 : 0);
  crash = life_variant(o, mod, pre);
  if (crash) return; /* only the forked probes run this */
  dsplibs_debug_level = (unsigned)dbg;
  emit(0, "LIFE %d mod=%02x pre=%d dbg=%d", tag, mod, pre, dbg);
  o->sec = (int)rmod(100);
  o->ms = (int)rmod(100);
  o->cd_flag = rpct(30) ? 0 : (int)rnd();
  o->agc_rcp = (int)rnd();
  o->agc_shift = (int)rnd();
  _init_receiver(o, mod);
  cov_hit(CV_LF_INIT);
  dump_t30("l", o);
  if (o->data_cfg) {
    char b[24];
    emit(0, "LIFE cfg: w0=%d s08=%d s0a=%d s0c=%d s0e=%d one=%d eng=%s cfg=%s", o->data_cfg->w0, (int)o->data_cfg->s08, (int)o->data_cfg->s0a, (int)o->data_cfg->s0c, (int)o->data_cfg->s0e,
         o->data_cfg->u4.init1 == 1, dptr(o->data_cfg->eng, b, sizeof b), dptr(o->data_cfg->u10.cfg, b, sizeof b));
  }
  switch (mod) {
    case 0x18: cov_hit(CV_LF_M18); break;
    case 0x30: cov_hit(CV_LF_M30); break;
    case 0x48: cov_hit(CV_LF_M48); break;
    case 0x60: cov_hit(CV_LF_M60); break;
    case 0x49: cov_hit(CV_LF_M49); break;
    case 0x4a: cov_hit(CV_LF_M4A); cov_hit(CV_LF_SHORT); break;
    case 0x61: cov_hit(CV_LF_M61); break;
    case 0x62: cov_hit(CV_LF_M62); cov_hit(CV_LF_SHORT); break;
    case 0x79: cov_hit(CV_LF_M79); break;
    case 0x7a: cov_hit(CV_LF_M7A); cov_hit(CV_LF_SHORT); break;
    case 0x91: cov_hit(CV_LF_M91); break;
    default: cov_hit(CV_LF_M92); cov_hit(CV_LF_SHORT); break;
  }
  if (pre == 0) cov_hit(CV_LF_FRESH);
  if (pre == 2) cov_hit(CV_LF_REPLACE);
  if (pre == 4) cov_hit(CV_LF_LEAK);
  if (pre == 1) {
    int id = mod_class_id(mod);
    if (id == FAXVMI_ID_V17RX) cov_hit(CV_LF_RESTART17);
    else if (id == FAXVMI_ID_V29RX) cov_hit(CV_LF_RESTART29);
    else cov_hit(CV_LF_RESTART27);
  }
  {
    int id = mod_class_id(mod);
    if (id == FAXVMI_ID_V17RX) cov_hit(CV_LF_AGC17);
    else if (id == FAXVMI_ID_V29RX) cov_hit(CV_LF_AGC29);
    else cov_hit(CV_LF_AGC27);
  }
  /* the delete function over what was built */
  if (rpct(60) && o->data_cfg && o->data_vmi) {
    int id17 = (unsigned short)o->data_cfg->s0e == FAXVMI_ID_V17RX;
    _delete_data_rx_modem(o);
    cov_hit(CV_LF_DEL);
    if (id17) cov_hit(CV_LF_DEL17);
    else cov_hit(CV_LF_DEL_OTHER);
    dump_t30("l3", o);
  }
  t30_free_channels(o);
  trial_end2("life");
  dsplibs_debug_level = 0;
}

/* forked probes: the outcome class of a child that calls the function in a failing situation: 0 returns, 1 SIGSEGV, 2 other */
static int probe_child(int kind)
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
    script_random();
    g_nmsz = 0;
    g_malloc_ctr = 0;
    if (kind <= 2) {
      /* malloc #k fails inside a fresh _init_receiver: #0 the config block, #1 the recorder's config block of the modem, #2 the recorder's FaxVmi object */
      g_fail_malloc = kind;
      o->modemClass = 5;
      _init_receiver(o, 0x18);
    } else if (kind == 6) {
      /* a config block without a channel (restart path dereferences the NULL channel) */
      life_variant(o, 0x18, 1);
      FAXVMI_delete(o->data_vmi);
      o->data_vmi = NULL;
      g_nmsz = 0;
      _init_receiver(o, 0x18);
    } else if (kind == 7) {
      /* the same with a class change */
      life_variant(o, 0x18, 1);
      FAXVMI_delete(o->data_vmi);
      o->data_vmi = NULL;
      _init_receiver(o, 0x48);
    } else if (kind == 8) {
      /* FAXVMI_create fails */
      g_rs.create_null = 1;
      o->modemClass = 5;
      _init_receiver(o, 0x30);
    } else {
      /* class V.17 create with the channel failing */
      g_rs.create_null = 1;
      o->modemClass = 7;
      _init_receiver(o, 0x91);
    }
    _exit(0);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) return WTERMSIG(st) == SIGSEGV ? 1 : 2;
  return WEXITSTATUS(st) == 0 ? 0 : 2;
}
static void life_probes(void)
{
  /* expected (blob / port): kind 0 (the config block of the object fails): SIGSEGV / returns; kind 1 (the recorder's modem config block): the recorder stores a NULL block pointer and goes on: returns /
   * returns; kind 2 (the recorder's FaxVmi object: FAXVMI_create returns NULL): SIGSEGV / returns; kinds 6 / 7 (config block without a channel): SIGSEGV / returns; kinds 8 / 9 (FAXVMI_create
   * returns NULL): SIGSEGV / returns */
  static const struct {
    int kind, blob, port;
  } pr[] = {
      {0, 1, 0}, {1, 0, 0}, {2, 1, 0}, {6, 1, 0}, {7, 1, 0}, {8, 1, 0}, {9, 1, 0},
  };
  unsigned i;
  for (i = 0; i < sizeof pr / sizeof pr[0]; i++) {
    int r = probe_child(pr[i].kind);
    int want = SIDE_BLOB ? pr[i].blob : pr[i].port;
    emit(1, "PROBE kind %d: %s", pr[i].kind, r == want ? "as expected (blob: crash or return, port: safe where the blob crashes)" : "UNEXPECTED");
    if (r != want) die3("probe kind %d: outcome %d, wanted %d", pr[i].kind, r, want);
    cov_hit(CV_LF_PROBE);
  }
}

static void scen_life(void)
{
  int t, m, pre, k;
  poke_consts(0);
  dump_consts();
  t = 0;
  for (m = 0; m < NMODS; m++)
    for (pre = 0; pre < 5; pre++)
      for (k = 0; k < 4; k++) {
        if (pre == 3) continue; /* crash case: probes only */
        life_trial(t, k_mods[m], pre, (t + k) & 3);
        t++;
        cov_hit(CV_LF_RATES);
      }
  for (m = 0; m < 700; m++) {
    int mod, p;
    mod = k_mods[rmod((unsigned)NMODS)];
    p = (int)rmod(5);
    if (p == 3) p = 1;
    life_trial(t++, mod, p, (int)rmod(4));
  }
  life_probes();
}
