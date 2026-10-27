/* R30 harness part 2 (R30_CLASS1 build): scenarios 0 .. 3 (fax_class1_create / _command / _progress / _delete over recorders).  Included by fax_api.main.c. */
#ifdef R30_CLASS1

static const int k_lvls[] = {0, 1, 2, 3, 4, 5, 6};
static void set_level(int k) { SETLVL(lvl_value(k)); }

/* ================================================================================================ scenario 0: fax_class1_create */
static void cfg_pick(FaxClass1Cfg *c, int mode)
{
  static const int w10s[] = {0, 0, 1, 9, 10, 99, 100, 149, 150, 1499, 12345, -1, -9, -10, -15, 0x7fffffff, (int)0x80000000u};
  int r;
  c->mode = mode;
  r = (int)rmod(10);
  c->s7 = r < 3 ? 7 : (r < 6 ? (int)rmod(60) : (r < 8 ? -(int)rmod(5) : (int)rnd()));
  r = (int)rmod(4);
  c->cdFlag = r == 0 ? 0 : (r == 1 ? 1 : (r == 2 ? -1 : (int)rnd()));
  r = (int)rmod(4);
  c->hpFilter = r == 0 ? 0 : (r == 1 ? 1 : (r == 2 ? (int)0x80000000u : (int)rnd()));
  c->answerMs = w10s[rmod((unsigned)(sizeof w10s / sizeof w10s[0]))];
  r = (int)rmod(4);
  c->cngOff = r == 0 ? 0 : (r == 1 ? 1 : (r == 2 ? -1 : (int)rnd()));
}
static void create_check(const char *tag, FaxT30 *o, const FaxClass1Cfg *c)
{
  cov_hit(CV_C0_CREATE);
  if (c->mode == 2) cov_hit(CV_C0_ANSWER);
  else {
    cov_hit(CV_C0_ORIG);
    if (c->cngOff == 0) cov_hit(CV_C0_CNG_ON);
    else if (dsplibs_debug_level > 1u) cov_hit(CV_C0_CNG_OFF);
    else cov_hit(CV_C0_CNG_OFF_QUIET);
  }
  if (c->mode != 1 && c->mode != 2) cov_hit(CV_C0_MODE_OTHER);
  if (c->answerMs != 0) {
    cov_hit(CV_C0_ANSLEN);
    if (c->answerMs < 0) cov_hit(CV_C0_ANSLEN_NEG);
    if (dsplibs_debug_level > 1u) cov_hit(CV_C0_ANSLEN_DBG);
  } else cov_hit(CV_C0_ANSLEN_DEF);
  if (c->hpFilter != 0) cov_hit(CV_C0_HP_ON);
  else cov_hit(CV_C0_HP_OFF);
  if (dsplibs_debug_level > 1u) cov_hit(CV_C0_BANNER);
  g_cur = o;
  dump_t30(tag, o);
  dump_cfg_block(tag, o->hdlc_tx_cfg);
  dump_cfg_block(tag, o->hdlc_rx_cfg);
  emit(0, "%s ptrs tx=%u rx=%u tone=%u", tag, hid(o->hdlc_tx), hid(o->hdlc_rx), hid(o->tone));
  dump_table(tag);
}
static void child_create(void *arg)
{
  FaxClass1Cfg c;
  FaxT30 *o;
  script_reset();
  poke_consts(0);
  cfg_pick(&c, 1);
  c.cngOff = 0;
  g_fail_malloc = (int)(intptr_t)arg;
  o = fax_class1_create(NULL, &c);
#ifndef TEST_BLOB
  if (o == NULL && g_live != 0) _exit(7); /* the port frees what it allocated */
#endif
  (void)o;
}
static void scen_create(void)
{
  static const int modes[] = {1, 2, 0, 3, -1, 0x10002, 0x30001, 1, 2};
  int t, i, k;
  unsigned vn, tn;
  FaxClass1Cfg c;
  FaxT30 *o;
  /* exhaustive over the cfg words that matter, recorders default */
  for (t = 0; t < 7; t++) {
    int m, w14, wc, w10;
    static const int w10v[] = {0, 150, -1};
    set_level(t);
    for (m = 1; m <= 2; m++)
      for (w14 = 0; w14 < 2; w14++)
        for (wc = 0; wc < 2; wc++)
          for (w10 = 0; w10 < 3; w10++) {
            script_reset();
            poke_consts(1);
            cfg_pick(&c, m);
            c.cngOff = w14;
            c.hpFilter = wc;
            c.answerMs = w10v[w10];
            o = fax_class1_create(NULL, &c);
            emit(0, "create lvl=%d m=%d w14=%d wc=%d w10=%d -> %u", t, m, w14, wc, w10, hid(o));
            if (o) {
              create_check("ex", o, &c);
              post_create();
              fax_class1_delete(o);
              cov_hit(CV_C0_DELETE);
            }
            g_cur = NULL;
            trial_end("create");
          }
  }
  /* random cfg words, poked constants, creation failures */
  for (i = 0; i < 700; i++) {
    int mode = modes[rmod((unsigned)(sizeof modes / sizeof modes[0]))];
    int lv = (int)rmod(7);
    set_level(lv);
    script_reset();
    poke_consts(rpct(70));
    cov_hit(CV_C0_POKED);
    cfg_pick(&c, mode);
    vn = rpct(25) ? rmod(4) : 0;
    tn = rpct(20) ? 1u : 0u;
    g_sc.vmi_null = vn;
    g_sc.tone_null = tn;
    g_sc.mk = rpct(30) ? (int)rmod(256) : 0;
    g_sc.mk_val = (int)rmod(50);
    o = fax_class1_create(NULL, &c);
    emit(0, "create i=%d lvl=%d mode=%d vn=%u tn=%u -> %u", i, lv, mode, vn, tn, hid(o));
    if (vn & 1) cov_hit(CV_C0_VMI_NULL_TX);
    if (vn & 2) cov_hit(CV_C0_VMI_NULL_RX);
    if (tn) cov_hit(CV_C0_TONE_NULL);
    if (o) {
      create_check("rnd", o, &c);
      post_create();
      fax_class1_delete(o);
      cov_hit(CV_C0_DELETE);
    }
    g_cur = NULL;
    trial_end("create");
  }
  /* a supplied object: only the common part runs (no channels, nothing cleared) */
  for (i = 0; i < 120; i++) {
    int mode = modes[rmod((unsigned)(sizeof modes / sizeof modes[0]))];
    set_level((int)rmod(7));
    script_reset();
    poke_consts(rpct(50));
    cfg_pick(&c, mode);
    t30_garbage(&g_t30);
    g_t30.hdlc_tx = (FaxVmi *)(void *)&g_dummy_a;
    g_cur = &g_t30;
    dump_t30("pre", &g_t30);
    o = fax_class1_create(&g_t30, &c);
    emit(0, "create supplied i=%d -> %s", i, o == &g_t30 ? "same" : "OTHER");
    cov_hit(CV_C0_SUPPLIED);
    create_check("sup", &g_t30, &c);
    post_create();
    g_cur = NULL;
    trial_end("supplied");
  }
  /* malloc failure probes: the blob stores through NULL for the object / config block allocations (SIGSEGV), the port frees what it allocated and returns NULL */
  set_level(0);
  for (k = 0; k < 9; k++) {
    int r = guarded(child_create, (void *)(intptr_t)k);
    guarded_report("create malloc failure", r);
    cov_hit(CV_C0_PROBE);
    if (k < 3 || k == 4 || k == 5) cov_hit(CV_C0_PROBE_FAIL);
  }
}

/* ================================================================================================ scenario 1: fax_class1_command */
static void cmd_object(FaxT30 *o, int cmd, int param)
{
  int r;
  t30_garbage(o);
  r = (int)rmod(10);
  o->state = r < 4 ? 6 : (r < 8 ? (int)rmod(19) : (int)rnd());
  r = (int)rmod(3);
  o->modem_sel = r == 0 ? param : (r == 1 ? k_mods[rmod(NMODS)] : (int)rnd());
  r = (int)rmod(10);
  o->sf_fill = r < 5 ? 0 : (r < 8 ? (int)rmod(300) : (int)rnd());
  r = (int)rmod(5);
  o->chan = r == 0 ? 0 : (r == 1 ? 1 : (r == 2 ? 2 : (r == 3 ? (int)rmod(5) : (int)rnd())));
  if (rpct(55)) o->data_cfg = (FaxVmiM *)(void *)&g_dummy_a;
  if (rpct(70)) o->data_vmi = (FaxVmi *)(void *)&g_dummy_b;
  o->sec = (int)rmod(1000);
  o->ms = (int)rmod(100);
  (void)cmd;
}
static void do_command(const char *tag, FaxT30 *o, int cmd, int param, int extra)
{
  int r;
  int pre_chan = o->chan, pre_dc = o->data_cfg != NULL, pre_dv = o->data_vmi != NULL, pre_state = o->state, pre_sel = o->modem_sel, pre_sf = o->sf_fill;
  g_cur = o;
  dump_t30(tag, o);
  r = fax_class1_command(o, cmd, param, extra);
  emit(0, "%s command cmd=%d param=%d extra=%d -> %d", tag, cmd, param, extra, r);
  dump_t30(tag, o);
  cov_hit(CV_C1_CMD);
  if (dsplibs_debug_level > 1u) {
    cov_hit(CV_C1_DBG1);
    if (cmd >= 0 && cmd <= 5) cov_hit(CV_C1_DBG2);
  }
  if (cmd < 0) cov_hit(CV_C1_NEG);
  if (cmd != 3) cov_hit(CV_C1_SFCLEAR);
  else cov_hit(CV_C1_SFKEEP);
  switch (cmd) {
    case 0: cov_hit(CV_C1_TH); break;
    case 1:
      cov_hit(CV_C1_TM);
      if (pre_dc && pre_dv && pre_chan == 1) cov_hit(CV_C1_TM_DEL);
      else cov_hit(CV_C1_TM_NODEL);
      break;
    case 2:
      cov_hit(CV_C1_RM);
      if (pre_dc && pre_dv && pre_chan == 2) cov_hit(CV_C1_RM_DEL);
      else cov_hit(CV_C1_RM_NODEL);
      break;
    case 3:
      cov_hit(CV_C1_RH);
      if (pre_state == 6 && pre_sel == param) cov_hit(CV_C1_RH_SAME);
      else {
        if (pre_state == 6) cov_hit(CV_C1_RH_DIFF);
        if (pre_sf == 0) cov_hit(CV_C1_RH_IDLE);
        else cov_hit(CV_C1_RH_EMU);
      }
      break;
    case 4:
    case 5:
      cov_hit(cmd == 4 ? CV_C1_TS : CV_C1_RS);
      if (param / 2 == 0) cov_hit(CV_C1_TIMER_ONE);
      if (param < 0) cov_hit(CV_C1_TIMER_NEG);
      break;
    default: cov_hit(CV_C1_BAD); break;
  }
  if (g_sc.mk) cov_hit(CV_C1_ORDER);
}
static void scen_command(void)
{
  static const int cmds[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 100, -1, -2, -3, (int)0x80000000u, 0x7fffffff, (int)0x80000001u};
  static const int params[] = {0, 1, 2, 3, 4, 5, 0x17, 0x18, 0x19, 0x30, 0x31, 0x48, 0x49, 0x4a, 0x4b, 0x60, 0x61, 0x62, 0x79, 0x7a, 0x91, 0x92, 0x93, 100, 160, 161, 4000, -1, -2, -3, 0x7fffffff, (int)0x80000000u};
  static const int extras[] = {0, 0x50, -1};
  int t, ci, pi, n = 0;
  FaxT30 *o = &g_t30;
  for (t = 0; t < 3; t++) {
    int lv = t == 0 ? 0 : (t == 1 ? 2 : 3);
    for (ci = 0; ci < (int)(sizeof cmds / sizeof cmds[0]); ci++)
      for (pi = 0; pi < (int)(sizeof params / sizeof params[0]); pi++) {
        int cmd = cmds[ci], param = params[pi], extra;
        int l2 = lv;
        script_reset();
        if (cmd < 0 && l2 > 1) l2 = 1; /* the blob prints the caller's EDI as the command name of an unmatched command <= 5 */
        set_level(l2);
        cmd_object(o, cmd, param);
        extra = extras[rmod(3)];
        if (rpct(45)) {
          g_sc.mk = (int)rmod(256);
          g_sc.mk_val = (int)rmod(40);
        }
        do_command("cmd", o, cmd, param, extra);
        n++;
        trial_end("cmd");
      }
  }
  /* random sequences of commands on one object (every callee effect feeds the next command) */
  for (t = 0; t < 250; t++) {
    int k, len = 2 + (int)rmod(6);
    script_reset();
    set_level(rpct(50) ? 0 : 2);
    cmd_object(o, 0, 3);
    for (k = 0; k < len; k++) {
      int cmd = (int)rmod(7);
      int param = rpct(60) ? k_mods[rmod(NMODS)] : (rpct(50) ? 3 : (int)rmod(400) - 100);
      g_sc.mk = rpct(30) ? (int)rmod(256) : 0;
      g_sc.mk_val = (int)rmod(40);
      do_command("seq", o, cmd, param, rpct(50) ? 0x50 : 0);
    }
    trial_end("cmdseq");
  }
}

/* ================================================================================================ scenario 2: fax_class1_progress */
typedef struct {
  short rx[1024], tx[1024];
  unsigned char tty_out[1024], tty_in[1024];
  int n_in, n_out, p7, p8;
} Io;
static Io g_io;
static void io_prep(Io *io, int n_in, int n_out, int zero_pct)
{
  int i;
  for (i = 0; i < 1024; i++) {
    io->rx[i] = rpct(zero_pct) ? 0 : (short)rnd();
    io->tx[i] = (short)0x5a5a;
    io->tty_out[i] = 0xee;
    io->tty_in[i] = (unsigned char)rnd();
  }
  io->n_in = n_in;
  io->n_out = n_out;
  io->p7 = (int)rnd();
  io->p8 = (int)rnd();
}
static void do_progress(const char *tag, FaxT30 *o, Io *io)
{
  int r;
  int pre_state = o->state, pre_prev = o->state_prev, pre_act = o->rx_act, pre_kind = o->rx_kind, n0 = io->n_in;
  int has_chan = o->chan == 1 && o->data_vmi != NULL;
  int sel_in = o->modem_sel == 0x91 || o->modem_sel == 0x79 || o->modem_sel == 0x61 || o->modem_sel == 0x49 || o->modem_sel == 0x60 || o->modem_sel == 0x48 || o->modem_sel == 0x30 ||
               o->modem_sel == 0x18;
  int pre_q = has_chan ? q4e_of(o, o->modem_sel) : -1;
  g_kind_seen = 0;
  g_cur = o;
  dump_t30(tag, o);
  r = fax_class1_progress(o, io->rx, io->tx, io->tty_out, io->tty_in, &io->n_in, &io->n_out, &io->p7, &io->p8);
  emit(0, "%s PROG r=%d n_in=%d n_out=%d p7=%d p8=%d tx=%016llx rx=%016llx out=%016llx", tag, r, io->n_in, io->n_out, io->p7, io->p8, fnv_shorts(H0, io->tx, 1024),
       fnv_shorts(H0, io->rx, 1024), fnv_bytes(io->tty_out, sizeof io->tty_out));
  dump_t30(tag, o);
  cov_hit(CV_C2_PROG);
  if (io->n_in == n0) cov_hit(CV_C2_NIN_COPY);
  if (pre_act != 0) {
    cov_hit(CV_C2_WATCH);
    if (dsplibs_debug_level > 1u) cov_hit(CV_C2_WATCH_DBG);
    if (n0 < 0) cov_hit(CV_C2_WATCH_NEG);
    if (o->rx_act == 0) cov_hit(CV_C2_WATCH_RESET);
  } else cov_hit(CV_C2_WATCH_OFF);
  if (o->ms == 0 && o->sec != 0) cov_hit(CV_C2_CLK_SEC);
  else cov_hit(CV_C2_CLK_MS);
  if (o->hp_on != 0) cov_hit(CV_C2_HP);
  else cov_hit(CV_C2_NOHP);
  (void)pre_kind;
  if (pre_state != pre_prev || o->state != pre_state) cov_hit(CV_C2_STATE_CHG);
  else cov_hit(CV_C2_STATE_SAME);
  if (o->state_prev == o->state && o->state >= 19) cov_hit(CV_C2_STATE_UNKNOWN);
  if (io->n_out != 0xa0) cov_hit(CV_C2_TXCNT);
  if (g_kind_seen != 0) cov_hit(CV_C2_DELAY);
  if (g_kind_seen == 1) cov_hit(CV_C2_DELAY_FIRE);
  if (has_chan && sel_in && r == 0 && pre_q == 0) cov_hit(CV_C2_ACCEPT_NO);
  if (r != 0) {
    cov_hit(CV_C2_STATUS);
    if (r >= 0 && r <= 10) cov_hit(CV_C2_STATUS_NAMED);
    else cov_hit(CV_C2_STATUS_UNNAMED);
  } else cov_hit(CV_C2_RET0);
  if (has_chan && r == 10) {
    if (o->modem_sel == 0x18 || o->modem_sel == 0x30) cov_hit(CV_C2_ACCEPT27);
    else if (o->modem_sel == 0x48 || o->modem_sel == 0x60) cov_hit(CV_C2_ACCEPT29);
    else cov_hit(CV_C2_ACCEPT17);
  }
}
static void child_progress_nochan(void *arg)
{
  FaxT30 *o = &g_t30;
  Io *io = &g_io;
  script_reset();
  set_level(0);
  t30_garbage(o);
  o->state = 8;
  o->state_prev = 8;
  o->rx_act = 0;
  o->hp_on = 0;
  o->rx_kind = 0;
  o->chan = 1;
  o->modem_sel = (int)(intptr_t)arg;
  o->data_vmi = NULL;
  g_sc.st_set = 2;
  g_sc.st_w122c = 0;
  io_prep(io, 160, 0xa0, 50);
  fax_class1_progress(o, io->rx, io->tx, io->tty_out, io->tty_in, &io->n_in, &io->n_out, &io->p7, &io->p8);
}
static void child_progress_state(void *arg)
{
  FaxT30 *o = &g_t30;
  Io *io = &g_io;
  script_reset();
  set_level(0);
  t30_garbage(o);
  o->state = (int)(intptr_t)arg;
  o->state_prev = o->state;
  o->rx_act = 0;
  o->hp_on = 0;
  o->rx_kind = 0;
  o->chan = 0;
  g_sc.st_set = 2;
  g_sc.st_w122c = 0;
  io_prep(io, 160, 0xa0, 50);
  fax_class1_progress(o, io->rx, io->tx, io->tty_out, io->tty_in, &io->n_in, &io->n_out, &io->p7, &io->p8);
}
static void scen_progress(void)
{
  static const int nins[] = {160, 160, 160, 0, 1, 7, 8, 9, 15, 16, 17, 31, 32, 33, 159, 161, 320, 1000, 8191, 8192, -1, -7, -8, -9, -15, -16, -17, -160, -8192};
  static const int bignins[] = {0x10000 + 160, 0x7fffffff, (int)0x80000000u, 70000, -70000, 0x12345678};
  static const int sels[] = {0x91, 0x79, 0x61, 0x49, 0x60, 0x48, 0x30, 0x18, 0x92, 0x7a, 0x62, 0x4a, 3, 0, 0x17, 0x93};
  static const int sts[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, -1, 100, 0x7fffffff, (int)0x80000000u};
  FaxT30 *o = &g_t30;
  Io *io = &g_io;
  int i, t;
  cov_hit(CV_C2_SEQ);
  for (i = 0; i < 6000; i++) {
    int lv = (int)rmod(7), r, n_in, zp;
    script_reset();
    set_level(lv);
    t30_garbage(o);
    r = (int)rmod(10);
    o->state = r < 9 ? (int)rmod(19) : (int)rmod(19);
    r = (int)rmod(10);
    o->state_prev = r < 5 ? o->state : (r < 8 ? (int)rmod(21) : (int)rnd());
    r = (int)rmod(10);
    o->rx_act = r < 3 ? 0 : (r < 6 ? 1 : (r < 8 ? (int)rmod(40) : (int)rnd()));
    r = (int)rmod(10);
    o->ms = r < 3 ? 98 : (r < 5 ? 97 : (r < 6 ? 99 : (r < 7 ? 96 : (int)rmod(110) - 5)));
    o->sec = (int)rmod(100);
    r = (int)rmod(4);
    o->hp_on = r == 0 ? 0 : (r == 1 ? 1 : (int)rnd());
    o->hpCoef = g_hp_dummy;
    {
      int k;
      for (k = 0; k < 10; k++) g_hp_dummy[k] = (short)rnd();
    }
    o->rx_kind = rpct(70) ? 0 : (rpct(60) ? 1 : (rpct(60) ? 2 : (int)rmod(5) - 2));
    o->rx_event = (int)rnd();
    o->status = (int)rnd();
    r = (int)rmod(10);
    if (r < 5) {
      o->chan = 1;
      o->modem_sel = sels[rmod((unsigned)(sizeof sels / sizeof sels[0]))];
      fake_chan(o, o->modem_sel, rpct(50) ? 0 : (rpct(50) ? 1 : (int)(short)rnd()));
      if (rpct(10)) o->chan = 2;
    } else {
      o->chan = (int)rmod(4);
      o->modem_sel = sels[rmod((unsigned)(sizeof sels / sizeof sels[0]))];
      if (o->chan == 1) o->chan = 3; /* (a receiver without a channel: the blob dereferences NULL: guarded probes below) */
    }
    /* the script of the state function */
    g_sc.st_set = (int)rmod(1024) & ~512;
    if (rpct(15)) g_sc.st_set |= 512;
    r = (int)rmod(4);
    g_sc.st_state = r == 0 ? (int)rmod(19) : (r == 1 ? o->state : (r == 2 ? 19 + (int)rmod(5) : (int)rnd() % 40));
    g_sc.st_w122c = sts[rmod((unsigned)(sizeof sts / sizeof sts[0]))];
    g_sc.st_kind = rpct(60) ? 0 : (int)rmod(6) - 1;
    g_sc.st_event = (int)rnd();
    g_sc.st_nout = rpct(60) ? 0xa0 : (int)rmod(300);
    g_sc.st_p7 = (int)rmod(50);
    g_sc.st_p8 = (int)rnd();
    g_sc.st_tmr = (int)rnd();
    g_sc.st_nin = (int)rnd();
    g_sc.st_ret = (int)rnd();
    if (rpct(70)) g_sc.st_set &= ~1; /* the state mostly stays: the state change message needs a change in 30 % */
    if ((g_sc.st_set & 1) && g_sc.st_state > 18) g_sc.st_state = 18 - (g_sc.st_state & 3);
    r = (int)rmod(10);
    if (o->rx_act != 0) n_in = nins[rmod((unsigned)(sizeof nins / sizeof nins[0]))];
    else n_in = r < 8 ? nins[rmod((unsigned)(sizeof nins / sizeof nins[0]))] : bignins[rmod((unsigned)(sizeof bignins / sizeof bignins[0]))];
    zp = rpct(30) ? 0 : (rpct(30) ? 100 : (int)rmod(100));
    io_prep(io, n_in, rpct(80) ? 0xa0 : (int)rmod(400), zp);
    if (o->state >= 0 && o->state <= 18) {
      char tag[8];
      snprintf(tag, sizeof tag, "p%d", lv);
      do_progress(tag, o, io);
    }
    trial_end("progress");
  }
  /* the state change message: every pair of state numbers, known and unknown names */
  {
    int a, b;
    for (a = -1; a <= 21; a += (a == -1 ? 1 : 1))
      for (b = 0; b <= 21; b++) {
        script_reset();
        set_level(2);
        t30_garbage(o);
        o->state = b > 18 ? 0 : b;
        o->state_prev = a;
        o->rx_act = 0;
        o->hp_on = 0;
        o->ms = 10;
        o->rx_kind = 0;
        o->status = 0;
        o->chan = 0;
        g_sc.st_set = 1 | 2;
        g_sc.st_state = b;
        g_sc.st_w122c = 0;
        io_prep(io, 160, 0xa0, 50);
        if (b <= 18 || 1) {
          /* the state function sets the state to b (possibly out of the table: only the message is looked up) */
          do_progress("pairs", o, io);
        }
        if (b > 18 && b != 19) cov_hit(CV_C2_STATE_PREVUNK);
        trial_end("pairs");
      }
  }
  /* statuses: every code through the name table at level 2 */
  for (t = 0; t < 40; t++) {
    int st = t < 15 ? t - 2 : (int)rmod(40) - 10;
    script_reset();
    set_level(rpct(70) ? 2 : 1);
    t30_garbage(o);
    o->state = (int)rmod(19);
    o->state_prev = o->state;
    o->rx_act = 0;
    o->hp_on = 0;
    o->chan = 0;
    o->rx_kind = 0;
    g_sc.st_set = 2;
    g_sc.st_w122c = st;
    io_prep(io, 160, 0xa0, 50);
    do_progress("stat", o, io);
    trial_end("stat");
  }
  /* a receiver without a channel object: the blob dereferences NULL (SIGSEGV), the port returns 0 */
  {
    static const int sl[] = {0x91, 0x60, 0x30};
    for (t = 0; t < 3; t++) {
      guarded_report("accept check without a channel", guarded(child_progress_nochan, (void *)(intptr_t)sl[t]));
      cov_hit(CV_C2_ACCEPT_NOCHAN);
    }
  }
  /* out of range state numbers: the blob calls through the table past its end / before its start (a crash or a wild jump); the port skips the call (a port that indexed the table would crash) */
  {
    static const int bad[] = {-1, 19, 20, 21, 100, 0x7fffffff, (int)0x80000000u, -2};
    for (t = 0; t < (int)(sizeof bad / sizeof bad[0]); t++) guarded_report("progress state out of range", guarded(child_progress_state, (void *)(intptr_t)bad[t]));
    cov_hit(CV_C2_BADSTATE);
  }
  /* the input watch: every n_in around the 8 / 16 sample boundaries against rx_act values around the limit, silent and loud input */
  {
    int n, ra, loud;
    for (n = -80; n <= 80; n++)
      for (ra = -4; ra <= 5; ra++)
        for (loud = 0; loud < 2; loud++) {
          if (ra == 0) continue;
          script_reset();
          set_level(0);
          t30_garbage(o);
          o->state = 8;
          o->state_prev = 8;
          o->rx_act = ra;
          o->hp_on = 0;
          o->rx_kind = 0;
          o->chan = 0;
          g_sc.st_set = 2;
          g_sc.st_w122c = 0;
          io_prep(io, n, 0xa0, loud ? 0 : 100);
          do_progress("watch", o, io);
          trial_end("watch");
        }
  }
}

/* ================================================================================================ scenario 3: life cycles and fax_class1_delete */
static void scen_life(void)
{
  int i, k;
  FaxT30 *o;
  FaxClass1Cfg c;
  Io *io = &g_io;
  /* create -> commands -> progress -> delete with scripted callees */
  for (i = 0; i < 200; i++) {
    int mode = rpct(50) ? 2 : 1, n, lv = (int)rmod(7);
    script_reset();
    set_level(lv);
    poke_consts(rpct(40));
    cfg_pick(&c, mode);
    c.answerMs = rpct(60) ? 0 : c.answerMs;
    o = fax_class1_create(NULL, &c);
    if (!o) die3("create failed");
    post_create();
    g_cur = o;
    emit(0, "life %d create lvl=%d mode=%d", i, lv, mode);
    cov_hit(CV_C3_LIFE);
    if (mode == 2) cov_hit(CV_C3_ANS);
    dump_t30("life", o);
    for (n = 0; n < 12; n++) {
      int act = (int)rmod(10);
      if (act < 4) {
        int cmd = (int)rmod(6);
        int param = rpct(70) ? k_mods[rmod(NMODS)] : 3;
        if (cmd == 0 || cmd == 3) param = 3; /* (FAX_class1_command only forwards FTH / FRH with parameter 3: a class value would leave the progress accept check on an uninitialised `chan`) */
        g_sc.mk = rpct(25) ? ((int)rmod(256) & ~8) : 0; /* (no chan marker: chan == 1 without a typed receiver would be read by the accept check) */
        g_sc.mk_val = (int)rmod(40);
        emit(0, "life cmd %d %d", cmd, param);
        fax_class1_command(o, cmd, param, rpct(50) ? 0x50 : 0);
        g_sc.mk = 0;
        cov_hit(CV_C3_CMDS);
        dump_t30("life", o);
      } else {
        int nn = rpct(70) ? 160 : (int)rmod(300);
        g_sc.st_set = (int)rmod(256) & ~(1 | 512);
        if (rpct(40)) g_sc.st_set |= 1;
        if (g_sc.st_set & 4) g_sc.st_set |= 8; /* (the delayed status words are set together: rx_event is heap garbage in a fresh object) */
        g_sc.st_state = (int)rmod(19);
        g_sc.st_w122c = rpct(70) ? 0 : (int)rmod(12);
        g_sc.st_kind = rpct(70) ? 0 : (int)rmod(3);
        g_sc.st_event = (int)rnd() & 0xf;
        g_sc.st_nout = rpct(70) ? 0xa0 : (int)rmod(300);
        g_sc.st_p7 = (int)rmod(50);
        g_sc.st_p8 = (int)rmod(300);
        g_sc.st_tmr = (int)rmod(100);
        io_prep(io, nn, 0xa0, 40);
        {
          int r = fax_class1_progress(o, io->rx, io->tx, io->tty_out, io->tty_in, &io->n_in, &io->n_out, &io->p7, &io->p8);
          emit(0, "life prog -> %d n_out=%d p7=%d", r, io->n_out, io->p7);
        }
        cov_hit(CV_C3_PROGS);
        dump_t30("life", o);
      }
    }
    fax_class1_delete(o);
    cov_hit(CV_C3_DEL);
    g_cur = NULL;
    trial_end("life");
  }
  /* fax_class1_delete of random object compositions (every block comes from the recorders / the heap: the frees are logged in the blob's order) */
  for (i = 0; i < 400; i++) {
    int bare = (i % 25) == 0;
    int hasdata = bare ? 0 : (int)rmod(4), has_dvmi = bare ? 0 : rpct(70), chan = (int)rmod(4);
    FaxVmiCfg *cfg;
    int *mcfg;
    set_level(rpct(70) ? 0 : 2);
    script_reset();
    o = (FaxT30 *)sysdep_malloc((unsigned)sizeof(FaxT30));
    memset(o, 0, sizeof *o);
    if (!bare && rpct(70)) {
      cfg = (FaxVmiCfg *)sysdep_malloc((unsigned)sizeof(FaxVmiCfg));
      mcfg = (int *)sysdep_malloc(28);
      memset(cfg, 0, sizeof *cfg);
      cfg->id = 5;
      cfg->mcfg = mcfg;
      o->hdlc_tx_cfg = cfg;
      cov_hit(CV_C3_DEL_TXCFG);
    }
    if (!bare && rpct(70)) {
      FaxVmiCfg cc;
      memset(&cc, 0, sizeof cc);
      cc.id = 5;
      o->hdlc_tx = FAXVMI_create(NULL, &cc);
      cov_hit(CV_C3_DEL_CH);
    }
    if (!bare && rpct(70)) {
      cfg = (FaxVmiCfg *)sysdep_malloc((unsigned)sizeof(FaxVmiCfg));
      mcfg = (int *)sysdep_malloc(24);
      memset(cfg, 0, sizeof *cfg);
      cfg->id = 6;
      cfg->mcfg = mcfg;
      o->hdlc_rx_cfg = cfg;
      cov_hit(CV_C3_DEL_RXCFG);
    }
    if (!bare && rpct(70)) {
      FaxVmiCfg cc;
      memset(&cc, 0, sizeof cc);
      cc.id = 6;
      o->hdlc_rx = FAXVMI_create(NULL, &cc);
    }
    if (hasdata) o->data_cfg = (FaxVmiM *)sysdep_malloc(48);
    if (has_dvmi) {
      FaxVmiCfg cc;
      memset(&cc, 0, sizeof cc);
      cc.id = 8;
      o->data_vmi = FAXVMI_create(NULL, &cc);
    }
    o->chan = chan;
    if (!bare && rpct(60)) {
      o->fifo = (FaxFifo *)sysdep_malloc(48);
      cov_hit(CV_C3_DEL_FIFO);
    }
    if (!bare && rpct(60)) {
      o->tone = (FpmTone *)sysdep_malloc(40);
      cov_hit(CV_C3_DEL_TONE);
    }
    g_cur = NULL;
    emit(0, "delete i=%d data_cfg=%u data_vmi=%u chan=%d", i, hid(o->data_cfg), hid(o->data_vmi), chan);
    {
      FaxVmiM *dc = o->data_cfg;
      FaxVmi *dv = o->data_vmi;
      int did_data = dc != NULL && dv != NULL;
      if (did_data) {
        if (chan == 1) cov_hit(CV_C3_DEL_DRX);
        else cov_hit(CV_C3_DEL_DTX);
      } else if (dc != NULL) cov_hit(CV_C3_DEL_NODATA);
      if (!o->hdlc_tx_cfg && !o->hdlc_tx && !o->hdlc_rx_cfg && !o->hdlc_rx && !dc && !dv && !o->fifo && !o->tone) cov_hit(CV_C3_DEL_BARE);
      k = fax_class1_delete(o);
      emit(0, "delete -> %d", k);
      cov_hit(CV_C3_DEL);
      /* the recorders of _delete_data_* free nothing: the harness frees the data blocks the object pointed to */
      if (dv) sysdep_free(dv);
      if (dc) sysdep_free(dc);
    }
    trial_end("delete");
  }
}

#endif /* R30_CLASS1 */
