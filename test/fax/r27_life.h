/* R27 harness part 3 (REC build): wiring (scenario 0), lifecycle (scenario 4), process (scenario 5).  Included by fax_vmi.main.c. */
#ifndef HARNESS_ENG

/* ---------------------------------------------------------------------------------------------- scenario 0: wiring */
static void scen_wiring(void)
{
  int id, k;
  FaxVmiM m;
  char b[40];
  emit(1, "FAXVMI_CFG mode=%d flag=%d s08=%u s0a=%u s0c=%u id=%u mcfg=%s w14=%d", FAXVMI_CFG.mode, FAXVMI_CFG.flag, (unsigned)FAXVMI_CFG.ringMin, (unsigned)FAXVMI_CFG.maxItems, (unsigned)FAXVMI_CFG.frmWords,
       (unsigned)FAXVMI_CFG.id, FAXVMI_CFG.mcfg ? "set" : "NULL", FAXVMI_CFG.w14);
  emit(1, "FAXVMI_CTL w0=%d w4=%d s8=%u wc=%d s10=%u mctl=%s", FAXVMI_CTL.flushRing, FAXVMI_CTL.breakReq, (unsigned)FAXVMI_CTL.breakCount, FAXVMI_CTL.rxReset, (unsigned)FAXVMI_CTL.newMode, FAXVMI_CTL.mctl ? "set" : "NULL");
  emit(1, "FAXVMI_STS s0=%d w4=%d w8=%d wc=%d w10=%d w14=%d msts=%s", (int)FAXVMI_STS.ringFree, FAXVMI_STS.ovf, FAXVMI_STS.txEmpty, FAXVMI_STS.rxOverflow, FAXVMI_STS.rxBreak, FAXVMI_STS.hskipDone, FAXVMI_STS.msts ? "set" : "NULL");
  cov_hit(CV_W_TAB);
  for (id = 0; id < FAXVMI_NIDS; id++) {
    unsigned short a, bb;
    int r;
    unsigned i;
    memset(&m, 0xA5, sizeof m);
    for (i = 0; i < 64; i++) g_bits_store[i] = (short)rnd();
    for (i = 0; i < 256; i++) g_buf0_store[i] = (short)rnd();
    m.u4.bits = g_bits_store;
    m.buf0 = g_buf0_store;
    rc_set(0, 11 + id, 3 + id % 5, 5);
    vxx_create[id](&m, id >= 5 ? &g_rc[0] : NULL);
    cov_hit(CV_W_CREATE);
    emit(1, "id %d create: s0c=%d s0e=%d s10=%d eng=%s", id, (int)m.s0c, (int)m.s0e, (int)m.u10.s10, dptr(m.eng, b, sizeof b));
    for (k = 0; k < 3; k++) {
      for (i = 0; i < 1024; i++) g_samples[i] = (short)rnd();
      a = (unsigned short)(1 + rmod(40));
      bb = (unsigned short)(1 + rmod(120));
      g_rec_k = rmod(30);
      g_rec_ret = rnd();
      r = vxx_process[id](&m, g_samples, &a, &bb);
      cov_hit(id < 5 ? CV_W_PROCESS_NULL : (g_istx[id] ? CV_W_PROCESS_TX : CV_W_PROCESS_RX));
      emit(1, "id %d process: r=%d a=%u b=%u samples=%016llx bits=%016llx buf0=%016llx", id, r, (unsigned)a, (unsigned)bb, fnv_shorts(g_samples, 400), fnv_shorts(m.u4.bits, 50),
           fnv_shorts(m.buf0, 200));
    }
    memset(g_sbuf, 0, sizeof g_sbuf);
    r = vxx_status[id](&m, g_sbuf);
    r += vxx_status[id](&m, NULL);
    cov_hit(CV_W_STATUS);
    emit(1, "id %d status: r=%d st=%02x%02x", id, r, g_sbuf[0], g_sbuf[1]);
    for (k = 0; k < 16; k++) g_cbuf[k] = (unsigned char)rnd();
    emit(1, "id %d control: r=%d", id, vxx_control[id](&m, g_cbuf));
    cov_hit(CV_W_CONTROL);
    for (k = -1; k <= 4; k++) {
      const char *out = (const char *)(uintptr_t)1;
      vxx_message[id](&m, k, &out);
      pr_str(out, b, sizeof b);
      emit(1, "id %d message %d: %s", id, k, b);
      cov_hit(CV_W_MESSAGE);
      if (!out) cov_hit(CV_W_MSGNULL);
    }
    vxx_delete[id](&m);
    cov_hit(CV_W_DELETE);
  }
  /* FAXVMI_message through objects */
  for (id = 0; id < FAXVMI_NIDS; id++) {
    FaxVmi *o;
    int idx;
    rc_set(1, 10, 4, 4);
    o = vmi_new(mkcfg(0, 0, 8, 4, 4, id, id >= 5 ? &g_rc[1] : NULL), 0);
    for (idx = -3; idx <= 300; idx += (idx < 8 ? 1 : 37)) {
      const char *msg = FAXVMI_message(o, idx);
      pr_str(msg, b, sizeof b);
      emit(1, "FAXVMI_message id %d idx %d: %s", id, idx, b);
      cov_hit(CV_W_FMSG);
    }
    FAXVMI_delete(o);
    trial_end("wiring");
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 4: lifecycle */
static FaxVmi *make_supplied(unsigned rs, unsigned fs, int garbage)
{
  FaxVmi *o = (FaxVmi *)sysdep_malloc((unsigned)sizeof(FaxVmi));
  FaxVmiSt *s = (FaxVmiSt *)sysdep_malloc((unsigned)sizeof(FaxVmiSt));
  FaxVmiM *m = (FaxVmiM *)sysdep_malloc((unsigned)sizeof(FaxVmiM));
  unsigned i;
  o->st = s;
  o->m = m;
  s->ring = (unsigned short *)sysdep_malloc(safe_bytes(2 * rs));
  s->size = (unsigned short)rs;
  s->frm = (unsigned short *)sysdep_malloc(safe_bytes(2 * fs));
  s->frm_size = (unsigned short)fs;
  m->u4.bits = (short *)sysdep_malloc(0x64);
  m->buf0 = (short *)sysdep_malloc(0x190);
  if (garbage) {
    for (i = 0; i < rs; i++) s->ring[i] = (unsigned short)rnd();
    for (i = 0; i < fs; i++) s->frm[i] = (unsigned short)rnd();
    for (i = 0; i < 50; i++) m->u4.bits[i] = (short)rnd();
    for (i = 0; i < 200; i++) m->buf0[i] = (short)rnd();
    s->rd = (unsigned short)rnd();
    s->wr = (unsigned short)rnd();
    s->cnt = (unsigned short)rnd();
    s->hskip = (unsigned short)rnd();
    s->flen = (short)rnd();
    s->ones = (short)rnd();
    s->rx_state = (int)rnd();
    s->in_frame = (int)rnd();
    s->tx_flag = (int)rnd();
    s->tx_mask = (int)rnd();
    s->breakLeft = (unsigned short)rnd();
  }
  return o;
}
static void probe_create(int failk, int id, int s0c)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    FaxVmiCfg c = mkcfg(1, 0, 12, 5, s0c, id, &g_rc[2]);
    if (freopen("/dev/null", "w", stderr) == NULL) _exit(9);
    g_fail_malloc = failk;
    FAXVMI_create(NULL, &c);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE create malloc #%d fails (id %d s0c %d): signal %d", failk, id, s0c, WTERMSIG(st));
  else emit(1, "PROBE create malloc #%d fails (id %d s0c %d): exit %d", failk, id, s0c, WEXITSTATUS(st));
  cov_hit(CV_L_PROBE);
}

static void scen_life(void)
{
  int t;
  FaxVmi *o;
  /* NULL object, NULL cfg */
  o = FAXVMI_create(NULL, NULL);
  cov_hit(CV_L_NEWNULL);
  cov_hit(CV_L_NEW);
  dump_vmi("defaults", o);
  FAXVMI_delete(o);
  cov_hit(CV_L_DEL);
  trial_end("defaults");
  /* sizes: the ring is max(s08, s0a + 3) shorts */
  {
    static const int sz[][2] = {{0, 0}, {1, 0}, {3, 0}, {4, 0}, {5, 2}, {6, 3}, {7, 4}, {8, 5}, {20, 20}, {30, 1}, {2, 40}, {10, 7}, {12, 9}};
    unsigned i;
    for (i = 0; i < sizeof sz / sizeof sz[0]; i++) {
      FaxVmiCfg c = mkcfg(0, 0, sz[i][0], sz[i][1], (int)i, 0, NULL);
      FaxVmi *p;
      fix_cfg(&c);
      p = FAXVMI_create(NULL, &c);
      emit(1, "SIZES s08=%d s0a=%d -> ring %u frm %u", sz[i][0], sz[i][1], (unsigned)p->st->size, (unsigned)p->st->frm_size);
      cov_hit(CV_L_SIZES);
      if (c.frmWords == 0) cov_hit(CV_L_SIZE0);
      FAXVMI_delete(p);
      trial_end("sizes");
    }
  }
  /* random creates */
  for (t = 0; t < 400; t++) {
    int id = t < 13 ? t : (int)rmod(13);
    int k = (int)rmod(8);
    FaxVmiCfg c;
    FaxVmi *p;
    unsigned i, calls;
    rc_rand(k, 50);
    {
      int dm, df, d8, da, dc, usec;
      dm = (int)rmod(3);
      df = (int)rmod(2);
      d8 = (int)rmod(60);
      da = (int)rmod(40);
      dc = rpct(10) ? (int)rmod(3) : (int)rmod(60);
      usec = id >= 5 && rpct(80);
      c = mkcfg(dm, df, d8, da, dc, id, usec ? &g_rc[k] : NULL);
    }
    c.w14 = (int)rnd();
    if (rpct(10)) c.mode |= (int)(rnd() & 0xffff0000u); /* the high half of the mode word is carried along */
    fix_cfg(&c);
    emit(0, "LIFE trial %d id %d", t, id);
    cov_hit(CV_L_CFG);
    cov_hit(id < 5 ? CV_L_ID_NULL : CV_L_ID_REC);
    if (t & 1) {
      p = FAXVMI_create(NULL, &c);
      cov_hit(CV_L_NEW);
    } else {
      unsigned rs = rmod(40), fs = rmod(50);
      p = make_supplied(rs, fs, rpct(70));
      p->st->pad1e = 0x1e1e;
      p = FAXVMI_create(p, &c);
      emit(0, "supplied pad2e=%04x", (unsigned)p->st->pad2e);
      cov_hit(CV_L_SUP);
      if (rs == 0 || fs == 0) cov_hit(CV_L_SIZE0);
    }
    dump_vmi("new", p);
    calls = rmod(3);
    for (i = 0; i < calls; i++) {
      /* control / status with random records */
      FaxVmiCtl ctl;
      FaxVmiSts sts;
      int r;
      memset(&ctl, 0, sizeof ctl);
      memset(&sts, 0, sizeof sts);
      ctl.flushRing = rpct(40) ? (int)rnd() : 0;
      ctl.breakReq = (int)rnd();
      ctl.breakCount = (unsigned short)rmod(6);
      ctl.rxReset = rpct(50) ? (int)(1 + rmod(3)) : 0;
      ctl.newMode = (unsigned short)(rpct(70) ? rmod(3) : rmod(6));
      ctl.mctl = rpct(50) ? (const void *)g_cbuf : NULL;
      if (rpct(50)) {
        unsigned q;
        for (q = 0; q < 64; q++) g_cbuf[q] = (unsigned char)rnd();
      }
      {
        unsigned rd0, cn0;
        rd0 = rmod(p->st->size + 1u);
        cn0 = rmod((unsigned)p->st->size + 1u);
        ring_set(p->st, rd0, cn0);
      }
      poke_rx(p->st);
      p->st->tx_flag = (int)rmod(3);
      if (rpct(20)) poke_tx(p->st);
      {
        unsigned q;
        for (q = 0; q < p->st->frm_size; q++) p->st->frm[q] = (unsigned short)rnd();
      }
      p->st->pad1e = 0x1e1e;
      p->st->pad2e = 0x2e2e;
      if (rpct(50)) p->st->ovf = (int)rnd();
      cov_hit(CV_L_CTL);
      if (ctl.mctl) cov_hit(CV_L_CTL_M);
      if (ctl.rxReset) cov_hit(ctl.newMode <= 2 ? CV_L_CTL_RESET : CV_L_CTL_RESET_BIG);
      if (ctl.flushRing) cov_hit(CV_L_CTL_FLUSH);
      r = FAXVMI_control(p, &ctl);
      emit(0, "control r=%d pad2e=%04x", r, (unsigned)p->st->pad2e);
      dump_vmi("ctl", p);
      if (rpct(10)) {
        r = FAXVMI_control(p, NULL);
        cov_hit(CV_L_CTL_NULL);
        emit(0, "control NULL r=%d", r);
      }
      sts.ringFree = 0x5a5a;
      sts.ovf = sts.txEmpty = sts.rxOverflow = sts.rxBreak = sts.hskipDone = 0x5a5a5a5a;
      sts.msts = rpct(50) ? (void *)g_sbuf : NULL;
      if (rpct(50)) p->st->ovf = (int)rnd();
      memset(g_sbuf, 0, sizeof g_sbuf);
      p->txEmpty = (int)rmod(3);
      p->rxOverflow = (int)rmod(3);
      cov_hit(CV_L_ST);
      if (sts.msts) cov_hit(CV_L_ST_M);
      r = FAXVMI_status(p, &sts);
      emit(0, "status r=%d s0=%d w4=%d w8=%d wc=%d w10=%d w14=%d sb=%02x%02x", r, (int)sts.ringFree, sts.ovf, sts.txEmpty, sts.rxOverflow, sts.rxBreak, sts.hskipDone, g_sbuf[0], g_sbuf[1]);
      if (rpct(10)) {
        r = FAXVMI_status(p, NULL);
        cov_hit(CV_L_ST_NULL);
        emit(0, "status NULL r=%d", r);
      }
    }
    {
      int idx;
      for (idx = 0; idx < 4; idx++) {
        char sb[40];
        pr_str(FAXVMI_message(p, idx), sb, sizeof sb);
        emit(0, "message %d %s", idx, sb);
        cov_hit(CV_L_MSG);
      }
    }
    FAXVMI_delete(p);
    cov_hit(CV_L_DEL);
    trial_end("life");
  }
  /* null_* directly */
  {
    FaxVmiM m;
    int cnts[] = {0, 1, 5, 200, -1, -7, 100};
    unsigned i;
    static short nbuf[1024];
    memset(&m, 0xA5, sizeof m);
    m.buf0 = nbuf;
    m.u4.bits = g_bits_store;
    null_create(&m, NULL);
    m.u4.bits = g_bits_store;
    emit(1, "null_create s0c=%d s0e=%d s10=%d eng=%s", (int)m.s0c, (int)m.s0e, (int)m.u10.s10, m.eng ? "set" : "NULL");
    for (i = 0; i < sizeof cnts / sizeof cnts[0]; i++) {
      unsigned short cnt = (unsigned short)cnts[i], a = 3;
      unsigned q;
      for (q = 0; q < 1024; q++) nbuf[q] = q < 256 ? 0x1234 : 0x5a5a; /* canary behind the 200 / 256 shorts buf0 holds */
      for (q = 0; q < 300; q++) g_samples[q] = (short)rnd();
      {
        int rr;
        rr = null_process(&m, g_samples, &a, &cnt);
        emit(1, "null_process cnt=%d r=%d buf0=%016llx", cnts[i], rr, fnv_shorts(nbuf, 1024));
      }
    }
    emit(1, "null_status %d null_control %d", null_status(&m, g_sbuf), null_control(&m, g_cbuf));
    {
      const char *out = (const char *)(uintptr_t)1;
      null_message(&m, 3, &out);
      emit(1, "null_message %s", out ? "set" : "NULL");
    }
    null_delete(&m);
    cov_hit(CV_L_NULLFN);
  }
  /* malloc-failure probes of the NULL-object create (obj, st, m, ring, frm, bits, buf0) */
  {
    int k;
    for (k = 0; k < 8; k++) probe_create(k, 5 + (k & 3), 10);
    for (k = 0; k < 8; k++) probe_create(k, 0, 0);
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 5: FAXVMI_process */
static unsigned short g_pbuf[2048];
static void make_rx_source(int mode, unsigned width)
{
  unsigned i;
  g_nbits = 0;
  if (rpct(30)) {
    for (i = 0; i < 1024; i++) g_rec_src[i] = (unsigned short)rnd();
    g_rec_len = 1024;
    return;
  }
  if (mode == 1) {
    for (i = 0; i < 40; i++) {
      bput(0);
      bput_byte(rmod(256));
      bput(1);
      if (rpct(20)) bput(1);
    }
  } else if (mode == 2) {
    unsigned f;
    for (f = 0; f < 6; f++) {
      unsigned char d[40];
      unsigned len = 3 + rmod(10);
      for (i = 0; i < len; i++) d[i] = (unsigned char)(i == 0 ? 0xff : rnd());
      hdlc_model(d, len, f == 0 ? 4 : 1, 0, 1, 1);
    }
  } else {
    for (i = 0; i < 40; i++) bput_byte(rmod(256));
  }
  {
    unsigned w = 0, from = 0;
    FaxVmiM tmp;
    short store[64];
    tmp.u4.bits = store;
    while (from < g_nbits && w < 1000) {
      unsigned k;
      from = bits_to_words(&tmp, from, 1, width);
      (void)k;
      g_rec_src[w++] = (unsigned short)store[0];
    }
    if (w == 0) g_rec_src[w++] = 0xffff;
    g_rec_len = w;
  }
}
static void scen_process(void)
{
  int t;
  for (t = 0; t < 1000; t++) {
    static const int ids[] = {0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    int id = ids[rmod(11)];
    int mode, flag, k;
    mode = (int)rmod(3);
    flag = (int)rmod(2);
    k = (int)rmod(8);
    FaxVmiCfg c;
    FaxVmi *o;
    unsigned calls = 1 + rmod(8), i, width;
    rc_rand(k, 40);
    width = (unsigned)g_rc[k].s10;
    {
      int d8, da, dc;
      d8 = 4 + (int)rmod(40);
      da = (int)rmod(30);
      dc = rpct(10) ? 2 + (int)rmod(5) : 30 + (int)rmod(60);
      c = mkcfg(mode, flag, d8, da, dc, id, id >= 5 ? &g_rc[k] : NULL);
    }
    o = vmi_new(c, 0);
    emit(0, "PROC trial %d id %d mode %d flag %d", t, id, mode, flag);
    cov_hit(CV_X_PROC);
    cov_hit(mode == 0 ? CV_X_M0 : (mode == 1 ? CV_X_M1 : CV_X_M2));
    cov_hit(flag ? CV_X_FLAG : CV_X_NOFLAG);
    cov_hit(id < 5 ? CV_X_NULL : (g_istx[id] ? CV_X_TX : CV_X_RX));
    make_rx_source(mode, width);
    g_rec_pos = 0;
    dbg_level(rpct(30) ? rmod(4) : 0);
    for (i = 0; i < calls; i++) {
      unsigned short n, cnt2;
      unsigned state, q;
      unsigned pos = 0;
      for (q = 0; q < 2048; q++) g_pbuf[q] = 0;
      for (q = 0; q < 1024; q++) g_samples[q] = (short)rnd();
      g_rec_ret = rpct(50) ? (rnd() & 0xe0ffffffu) | (rpct(30) ? 0x10000000u : 0) : rnd();
      g_rec_k = rpct(10) ? 0 : rmod(40);
      if (mode == 2) {
        unsigned nf = rmod(4), f;
        for (f = 0; f < nf; f++) {
          unsigned len = 1 + rmod(8);
          g_pbuf[pos++] = (unsigned short)len;
          for (q = 0; q < len; q++) g_pbuf[pos++] = (unsigned short)(q == 0 ? 0xff : rmod(256));
        }
        n = (unsigned short)nf;
      } else {
        n = (unsigned short)rmod(10);
        for (q = 0; q < 700; q++) g_pbuf[q] = (unsigned short)(rpct(80) ? rmod(256) : rnd());
        if (flag == 1 && 0) n = 0;
      }
      if (id < 5) cnt2 = (unsigned short)rmod(200);
      else if (g_istx[id]) cnt2 = (unsigned short)rnd();
      else cnt2 = (unsigned short)rmod(300);
      if (rpct(20)) o->st->ovf = (int)rmod(3);
      if (rpct(20)) o->rxOverflow = (int)rmod(3);
      if (rpct(20)) o->st->rxBreak = (int)rmod(3);
      if (rpct(5)) o->st->cnt = (unsigned short)(o->st->size + 1u + rmod(3));
      if (rpct(15)) {
        unsigned rd0, cn0;
        rd0 = rmod(o->st->size);
        cn0 = o->st->size > 2 ? (unsigned)o->st->size - rmod(3) : 0u;
        ring_set(o->st, rd0, cn0);
      }
      if (rpct(15)) poke_rx(o->st);
      if (rpct(10)) poke_tx(o->st);
      state = FAXVMI_process(o, g_pbuf, g_samples, &n, &cnt2);
      if (state & 0x01000000u) cov_hit(CV_X_S_W18);
      if (state & 0x02000000u) cov_hit(CV_X_S_ROOM);
      if (state & 0x04000000u) cov_hit(CV_X_S_OVF);
      if (state & 0x08000000u) cov_hit(CV_X_S_W1C);
      if (o->st->rxBreak && !(state & 0x10000000u) && (g_rec_ret & 0x10000000u)) cov_hit(CV_X_S_W3C);
      if (g_rec_ret & 0xe0000000u) cov_hit(CV_X_RETBITS);
      emit(0, "process state=%08x n=%u cnt2=%u buf=%016llx samples=%016llx", state, (unsigned)n, (unsigned)cnt2, fnv_shorts((short *)g_pbuf, 2048), fnv_shorts(g_samples, 512));
      dump_vmi("p", o);
      if (i > 3) cov_hit(CV_X_MANY);
    }
    dbg_level(0);
    FAXVMI_delete(o);
    trial_end("process");
  }
}
#endif
