/* R29 harness part 1: scenario 0 (the DTE input converters, _recieve_silence_state_init) and scenario 1 (the small functions: _hdlc_receive_state_init, _cHDLCrx_init_from_idle,
 * _recieve_silence_state, the gain threshold table).  Included by fax_t30_rx.main.c (REC build). */

static unsigned char g_in[4200];
static unsigned short g_obuf[4300];
#define GOUT (g_obuf + 32)

/* a DTE byte stream: mode 0 uniform bytes, 1 many DLE (0x10) and ETX (0x03), 2 an ETX at a random place */
static void fill_stream(unsigned char *in, int n, int mode)
{
  int i;
  for (i = 0; i < n; i++) {
    unsigned r = rmod(100);
    if (mode == 0) in[i] = (unsigned char)rnd();
    else if (mode == 1) in[i] = r < 30 ? 0x10 : (r < 55 ? 0x03 : (unsigned char)rnd());
    else in[i] = r < 12 ? 0x10 : (unsigned char)rnd();
  }
  if (mode == 2 && n > 1 && rpct(75)) {
    int at = (int)rmod((unsigned)(n - 1));
    in[at] = 0x10;
    in[at + 1] = 0x03;
  }
}
static void obuf_prep(void)
{
  int i;
  for (i = 0; i < 4300; i++) g_obuf[i] = (unsigned short)(0x4242 + i);
}

static void scen_hdi(void)
{
  int t;
  for (t = 0; t < 3500; t++) {
    FaxT30 *o = &g_t30;
    int n, k, big = (t % 40) == 39, pend, busy;
    unsigned short *out;
    int r, prev_cnt;
    unsigned dbg = rmod(4);
    t30_garbage(o);
    obuf_prep();
    o->dataEnd = rpct(12) ? 1 + (int)rmod(3) : 0;
    o->dlePending = rpct(30) ? (rpct(80) ? 1 : (int)rnd()) : 0;
    o->lastByte = (int)rnd();
    pend = o->dlePending != 0;
    busy = o->dataEnd != 0;
    k = (int)rmod(20);
    if (big) n = 2060 + (int)rmod(150);
    else if (k == 0) n = 0;
    else if (k == 1) n = -(1 + (int)rmod(5));
    else n = 1 + (int)rmod(120);
    fill_stream(g_in, n > 0 ? n : 0, big ? 0 : (int)rmod(3));
    if (big) {
      g_in[n - 2] = 0x10;
      g_in[n - 1] = 0x03;
      o->dlePending = 0;
      o->dataEnd = 0;
      pend = 0;
      busy = 0;
    }
    out = (big || rpct(40)) ? GOUT : o->buf;
    dsplibs_debug_level = dbg;
    prev_cnt = HDI_COUNTER;
    {
      int nn = n;
      emit(0, "HDI t=%d dbg=%u n=%d w12b0=%d w124c=%d out=%s", t, dbg, n, o->dataEnd, o->dlePending, out == o->buf ? "obj" : "sep");
      r = _handle_data_input(o, g_in, out, &nn);
      emit(0, "HDI R r=%d n=%d outsep=%016llx outobj=%016llx last=%d cnt+%d", r, nn, fnv_shorts((const short *)(g_obuf), 4300), fnv_shorts((const short *)o->buf, 0x900), o->lastByte,
           HDI_COUNTER - prev_cnt);
      dump_t30("hdi", o);
      cov_hit(CV_IN_HDI);
      if (busy) cov_hit(CV_IN_HDI_BUSY);
      else {
        if (n <= 0) cov_hit(CV_IN_HDI_N0);
        if (pend && n > 0) cov_hit(CV_IN_HDI_PEND);
        if (HDI_COUNTER - prev_cnt != 0) cov_hit(CV_IN_CNT);
        if (o->dataEnd != 0) {
          cov_hit(CV_IN_HDI_ETX);
          if (nn >= 0x800) cov_hit(CV_IN_HDI_ETX_FULL);
        }
        {
          int i, p = pend;
          for (i = 0; i < n; i++) {
            if (p) {
              if (g_in[i] == 0x10) cov_hit(CV_IN_HDI_DLE);
              else if (g_in[i] != 3) cov_hit(CV_IN_HDI_OTHER);
              p = 0;
              if (g_in[i] == 3) break;
            } else if (g_in[i] == 0x10) {
              p = 1;
            }
          }
        }
      }
    }
    trial_end2("hdi");
  }
  dsplibs_debug_level = 0;
}

static void scen_hhi(void)
{
  int t;
  for (t = 0; t < 3500; t++) {
    FaxT30 *o = &g_t30;
    int n, k, r, nn, prev_idx;
    unsigned short *out;
    unsigned dbg = rmod(4);
    t30_garbage(o);
    obuf_prep();
    k = (int)rmod(20);
    o->hdlcWr = rpct(25) ? 1 : (rpct(10) ? 0 : (int)rmod(0x300));
    o->dlePending = rpct(25) ? (rpct(80) ? 1 : (int)rnd()) : 0;
    o->lastFrame = rpct(40) ? 0 : (rpct(50) ? 1 : (int)rnd());
    if (rpct(50)) o->buf[2] |= 0x10;
    else o->buf[2] &= (unsigned short)~0x10u;
    if (k == 0) n = 0;
    else if (k == 1) n = -(1 + (int)rmod(5));
    else n = 1 + (int)rmod(150);
    fill_stream(g_in, n > 0 ? n : 0, rpct(30) ? 1 : 2);
    out = rpct(75) ? o->buf : GOUT;
    dsplibs_debug_level = dbg;
    prev_idx = o->hdlcWr;
    nn = n;
    emit(0, "HHI t=%d n=%d w1250=%d w124c=%d w1224=%d out=%s", t, n, o->hdlcWr, o->dlePending, o->lastFrame, out == o->buf ? "obj" : "sep");
    r = _handle_hdlc_input(o, g_in, out, &nn);
    emit(0, "HHI R r=%d n=%d outsep=%016llx", r, nn, fnv_shorts((const short *)g_obuf, 4300));
    dump_t30("hhi", o);
    cov_hit(CV_IN_HHI);
    if (r == 1) {
      cov_hit(CV_IN_HHI_FRAME);
      if (o->buf[2] & 0x10) cov_hit(CV_IN_HHI_LAST);
      else cov_hit(CV_IN_HHI_NOLAST);
    } else {
      cov_hit(CV_IN_HHI_PART);
    }
    if (n <= 0) cov_hit(CV_IN_HHI_N0);
    if (out != o->buf) cov_hit(CV_IN_HHI_SEP);
    {
      int i, p = o->dlePending != 0 && 0;
      (void)p;
      (void)prev_idx;
      for (i = 0; i + 1 < n; i++)
        if (g_in[i] == 0x10 && g_in[i + 1] == 0x10) {
          cov_hit(CV_IN_HHI_DLE);
          break;
        }
      for (i = 0; i + 1 < n; i++)
        if (g_in[i] == 0x10 && g_in[i + 1] != 0x10 && g_in[i + 1] != 3) {
          cov_hit(CV_IN_HHI_OTHER);
          break;
        }
    }
    trial_end2("hhi");
  }
  dsplibs_debug_level = 0;
}

static void scen_open_close(void)
{
  int t;
  for (t = 0; t < 300; t++) {
    FaxT30 *o = &g_t30;
    int r;
    t30_garbage(o);
    o->hdlcWr = (int)rnd();
    if (t & 1) {
      r = _handle_hdlc_input_open(o);
      emit(0, "OPEN r=%d", r);
      cov_hit(CV_IN_OPEN);
    } else {
      if (rpct(50)) o->buf[2] |= 0x10;
      else o->buf[2] &= (unsigned short)~0x10u;
      o->lastFrame = rpct(50) ? 0 : (int)rnd();
      r = _handle_hdlc_input_close(o);
      emit(0, "CLOSE r=%d", r);
      cov_hit(CV_IN_CLOSE);
      if (o->buf[2] & 0x10) cov_hit(CV_IN_CLOSE_LAST);
    }
    dump_t30("oc", o);
    trial_end2("oc");
  }
}

static void scen_rsi(void)
{
  int t;
  static const int edge[] = {0, 1, 2, 3, -1, -2, -3, 160, 161, 0x7fffffff, (int)0x80000000u, 1000};
  for (t = 0; t < 200; t++) {
    FaxT30 *o = &g_t30;
    int n, r;
    t30_garbage(o);
    n = t < 12 ? edge[t] : (rpct(50) ? (int)rmod(2000) - 1000 : (int)rnd());
    r = _recieve_silence_state_init(o, n);
    emit(0, "RSI n=%d r=%d", n, r);
    dump_t30("rsi", o);
    cov_hit(CV_IN_RSI);
    if (n / 2 == 0) cov_hit(CV_IN_RSI_ZERO);
    if (n < 0) cov_hit(CV_IN_RSI_NEG);
    trial_end2("rsi");
  }
}

/* the three dead caller-ID helpers (src/cid/cid_unused.c): _look_for / _look_for_other_than on random messages ([type][total length][type len data ...]), pack_next_bit on a CidFsk with random state */
extern int _look_for(const signed char *buf, unsigned char c);
extern int _look_for_other_than(const signed char *buf, short start, int skip);
extern void pack_next_bit(short bit, CidFsk *o);
static CidFsk g_fsk;
static void scen_dead(void)
{
  int t;
  static signed char msg[300];
  for (t = 0; t < 4000; t++) {
    int i, pos, n, r, c, st, sk;
    n = rpct(8) ? (int)rmod(4) : 2 + (int)rmod(60);
    msg[0] = (signed char)rnd();
    msg[1] = (signed char)(rpct(8) ? (int)(signed char)rnd() : n);
    pos = 2;
    for (i = 0; pos < 200 && i < 20; i++) {
      int len = (int)rmod(6); /* (a non positive step would never end, in the blob too) */
      msg[pos] = (signed char)(rpct(60) ? 1 + (int)rmod(8) : (int)rnd());
      msg[pos + 1] = (signed char)len;
      pos += 2 + (len > 0 ? len : 0);
    }
    for (; pos < 300; pos++) msg[pos] = (signed char)(1 + (int)rmod(5));
    if (rpct(50)) {
      /* a well formed message: the total length covers the parameters */
      msg[1] = (signed char)(pos < 120 ? pos : 100);
    }
    c = rpct(70) ? 1 + (int)rmod(8) : (int)(unsigned char)rnd();
    st = rpct(60) ? 2 : (int)(short)rnd() % 70;
    sk = rpct(40) ? -1 : (int)rmod(60);
    r = _look_for(msg, (unsigned char)c);
    emit(0, "LF c=%d len=%d r=%d", c, (int)msg[1], r);
    r = _look_for_other_than(msg, (short)st, sk);
    emit(0, "LFO st=%d sk=%d r=%d", st, sk, r);
  }
  for (t = 0; t < 6000; t++) {
    CidFsk *o = &g_fsk;
    short bit;
    int i;
    o->cfgThr = (unsigned short)(rpct(70) ? 2 : (int)rmod(9));
    o->ones = (short)(rpct(60) ? (int)rmod(20) - 2 : (int)rmod(40));
    o->state = (short)(rpct(70) ? (int)rmod(4) : (int)rmod(7) - 1);
    o->acc = (short)rnd();
    o->bitIdx = (short)(rpct(70) ? (int)rmod(9) : (int)rmod(40) - 4);
    o->count = (short)rmod(100);
    for (i = 0; i < 0x78; i++) o->msg[i] = (unsigned char)(0x30 + i);
    bit = (short)(rpct(40) ? 1 : (rpct(50) ? 0 : (int)(short)rmod(5) - 1));
    pack_next_bit(bit, o);
    emit(0, "PNB bit=%d thr=%u ones=%d st=%d acc=%d bi=%d cnt=%d msg=%016llx", (int)bit, (unsigned)o->cfgThr, (int)o->ones, (int)o->state, (int)o->acc, (int)o->bitIdx, (int)o->count,
         fnv_bytes(o->msg, 0x78));
    cov_hit(CV_IN_DEAD);
  }
}

static void scen_input(void)
{
  scen_dead();
  poke_consts(0);
  dump_consts();
  scen_hdi();
  scen_hhi();
  scen_open_close();
  scen_rsi();
}

/* ---------------------------------------------------------------------------------------------- scenario 1: the small functions */
static void scen_small(void)
{
  int t, i;
  poke_consts(0);
  dump_consts();
  for (i = 0; i < 4; i++) emit(1, "GAIN_TAB[%d] = %d", i, (int)GAIN_TAB[i]);
  cov_hit(CV_SM_TAB);
  /* _hdlc_receive_state_init */
  for (t = 0; t < 120; t++) {
    FaxT30 *o = &g_t30;
    int r;
    unsigned dbg = (unsigned)(t & 3);
    t30_garbage(o);
    dsplibs_debug_level = dbg;
    r = _hdlc_receive_state_init(o);
    emit(0, "HRSI t=%d dbg=%u r=%d", t, dbg, r);
    dump_t30("hrsi", o);
    cov_hit(CV_SM_HRSI);
    trial_end2("hrsi");
  }
  /* _cHDLCrx_init_from_idle */
  for (t = 0; t < 320; t++) {
    FaxT30 *o = &g_t30;
    int poke = rpct(40) ? 1 : 0, mode;
    unsigned dbg = (unsigned)(t & 3);
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    if (poke) poke_consts(1 + (int)rmod(2));
    else poke_consts(0);
    mode = rpct(50) ? 3 : (int)rmod(8);
    dsplibs_debug_level = dbg;
    emit(0, "IDLE t=%d dbg=%u mode=%d poke=%d", t, dbg, mode, poke);
    _cHDLCrx_init_from_idle(o, mode);
    dump_t30("idle", o);
    cov_hit(CV_SM_IDLE);
    if (mode == 3) cov_hit(CV_SM_IDLE_3);
    else cov_hit(CV_SM_IDLE_OTHER);
    if (poke) cov_hit(CV_SM_IDLE_POKE);
    t30_free_channels(o);
    trial_end2("idle");
  }
  poke_consts(0);
  /* _recieve_silence_state */
  for (t = 0; t < 1200; t++) {
    FaxT30 *o = &g_t30;
    int p8, n_in, k;
    unsigned dbg = rmod(4);
    t30_garbage(o);
    script_random();
    dsplibs_debug_level = dbg;
    k = (int)rmod(10);
    o->tmr = k < 6 ? 1 + (int)rmod(40) : (k < 8 ? (int)rnd() : -(int)rmod(5));
    o->silenceTime = k < 6 ? o->tmr - 2 + (int)rmod(4) : (int)rnd();
    p8 = rpct(30) ? (rpct(50) ? 1 + (int)rmod(5) : -1) : 0;
    k = (int)rmod(10);
    n_in = k < 5 ? 160 : (k == 5 ? 0 : (k == 6 ? 0x10000 + 160 : (k == 7 ? -160 : (int)rnd())));
    io_prep(&g_io, n_in, (int)rnd(), (int)rnd(), p8);
    emit(0, "RSIL t=%d dbg=%u rms=%d", t, dbg, g_rs.rms);
    call_state(T_RSIL, o, &g_io, "rsil");
    cov_hit(CV_SM_SIL);
    if (p8 != 0) cov_hit(CV_SM_SIL_ABORT);
    else if ((short)g_rs.rms > 100) cov_hit(CV_SM_SIL_LOUD);
    else {
      cov_hit(CV_SM_SIL_QUIET);
      if (o->state == 8) cov_hit(CV_SM_SIL_END);
    }
    if (g_rs.rms < 0) cov_hit(CV_SM_SIL_NEG);
    trial_end2("rsil");
  }
  dsplibs_debug_level = 0;
}
