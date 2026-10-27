/* R29 harness part 2: scenario 2 (the HDLC receive states: single calls and dispatched sequences).  Included by fax_t30_rx.main.c (REC build). */

static int pick_n_in(void)
{
  int k = (int)rmod(10);
  if (k < 6) return 160;
  if (k == 6) return 0;
  if (k == 7) return (int)rmod(4) == 0 ? 1 : 80 + (int)rmod(3) * 80;
  if (k == 8) return rpct(50) ? -160 : 0x10000 + 160;
  return (int)rmod(400) - 100;
}
static int pick_p8(void)
{
  int k = (int)rmod(10);
  if (k < 6) return 0;
  if (k == 6) return 5;
  if (k == 7) return -1;
  return 1 + (int)rmod(40);
}
static int s7_timeout(const FaxT30 *o, int n_in) { return n_in != 0 ? (int)((unsigned)o->s7 * 0x1f40u) / n_in : (int)((unsigned)o->s7 * 50u); }

/* a superframe of nfr frames in buf[0x800 ..] (frame bytes are bytes: the debug print of the output function indexes a 256 byte table); returns the words used */
static int build_superframe(FaxT30 *o, int nfr)
{
  int f, pos = 0, i;
  for (f = 0; f < nfr; f++) {
    int len = (int)rmod(9);
    o->buf[0x800 + pos] = (unsigned short)len;
    for (i = 0; i < len; i++) o->buf[0x800 + pos + 1 + i] = (unsigned short)(rnd() & 0xff);
    pos += len + 1;
  }
  o->buf[0x800 + pos] = 0x7fff; /* whatever follows is a huge record: the blob's stack array of 12 entries is never overrun */
  return pos;
}

static void hr_lookcarrier(void)
{
  int t;
  static const int thr[] = {514, 727, 1026, 1450};
  for (t = 0; t < 4000; t++) {
    FaxT30 *o = &g_t30;
    int n_in, to, k, p8, kidx;
    unsigned dbg = rmod(4);
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    dsplibs_debug_level = dbg;
    o->s7 = rpct(15) ? 0 : (int)rmod(30);
    n_in = pick_n_in();
    to = s7_timeout(o, n_in);
    {
      int edges[1];
      edges[0] = to;
      o->tmr = pick_tmr(edges, 1);
    }
    k = (int)rmod(10);
    o->cng_on = k < 4 ? 1 : (k == 4 ? (int)rnd() : 0);
    k = (int)rmod(10);
    o->cng_tone = k < 4 ? 1 : (k == 4 ? (int)rnd() : 0);
    k = (int)rmod(10);
    o->cng_ctr = k < 2 ? 0x5dc0 - 0xa0 + (int)rmod(3) - 1 : (k < 4 ? 0xfa0 - 0xa0 + (int)rmod(3) - 1 : (k < 8 ? (int)rmod(0x6200) : (int)rnd()));
    o->tone = (FpmTone *)(void *)g_tone_mark;
    o->state_prev = (int)rmod(10);
    p8 = pick_p8();
    io_prep(&g_io, n_in, (int)rnd(), (int)rnd(), p8);
    emit(0, "HLC t=%d dbg=%u s7=%d to=%d", t, dbg, o->s7, to);
    {
      int prev_on = o->cng_on, prev_tone = o->cng_tone, prev_ctr = o->cng_ctr;
      unsigned prev_tmr = (unsigned)o->tmr;
      call_state(_hdlc_receive_look_carrier_state, o, &g_io, "hlc");
      cov_hit(CV_HR_LC);
      if (n_in == 0) cov_hit(CV_HR_LC_N0);
      if (g_rs.ret_h & 0x2000u) {
        cov_hit(CV_HR_LC_CARRIER);
        for (kidx = 0; kidx < 4; kidx++)
          if ((short)g_rs.agc_hist < thr[kidx]) break;
        if (kidx == 0) cov_hit(CV_HR_LC_GAIN0);
        if (kidx == 1) cov_hit(CV_HR_LC_GAIN1);
        if (kidx == 2) cov_hit(CV_HR_LC_GAIN2);
        if (kidx == 3) cov_hit(CV_HR_LC_GAIN3);
        if (kidx == 4) cov_hit(CV_HR_LC_GAINNONE);
      } else if (to == 0) {
        cov_hit(CV_HR_LC_TIMEOUT0);
      } else if (prev_tmr + 1u > (unsigned)to && prev_on == 0) {
        cov_hit(CV_HR_LC_TIMEOUT);
      } else {
        cov_hit(CV_HR_LC_NOTIME);
      }
      if (!(g_rs.ret_h & 0x2000u) || 1) {
        if (prev_on == 0 || (g_rs.ret_h & 0x2000u)) cov_hit(CV_HR_LC_CNG_OFF);
        else if (prev_tone == 0) {
          cov_hit(CV_HR_LC_CNG_SIL);
          if (prev_ctr + 0xa0 > 0x5dc0) cov_hit(CV_HR_LC_CNG_SIL_END);
        } else {
          cov_hit(CV_HR_LC_CNG_TONE);
          if (prev_ctr + 0xa0 > 0xfa0) cov_hit(CV_HR_LC_CNG_TONE_END);
        }
      }
      if (p8 != 0) cov_hit(CV_HR_LC_ABORT);
    }
    t30_free_channels(o);
    trial_end2("hlc");
  }
  dsplibs_debug_level = 0;
}

static void hr_state(void)
{
  int t;
  for (t = 0; t < 4000; t++) {
    FaxT30 *o = &g_t30;
    int k, p8, n_in;
    unsigned dbg = rmod(4);
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    dsplibs_debug_level = dbg;
    k = (int)rmod(10);
    o->tmr = k < 3 ? -1 : (k < 6 ? (int)rmod(50) : (int)rnd());
    o->state_prev = rpct(30) ? 6 : (int)rmod(10);
    p8 = pick_p8();
    n_in = pick_n_in();
    io_prep(&g_io, n_in, (int)rnd(), (int)rnd(), p8);
    emit(0, "HRS t=%d dbg=%u", t, dbg);
    {
      int wrap = o->tmr == -1, prev6 = o->state_prev == 6;
      call_state(_hdlc_receive_state, o, &g_io, "hrs");
      cov_hit(CV_HR_ST);
      if (wrap) cov_hit(CV_HR_ST_WRAP);
      if (prev6) cov_hit(CV_HR_ST_PREV6);
      if (!wrap) {
        int nf = g_rs.fr_len >= 0 && g_rs.fr_a != 0;
        if (!(g_rs.ret_h & 0x2000u)) {
          cov_hit(CV_HR_ST_NOCARR);
          if (nf) cov_hit(CV_HR_ST_NOCARR_FRAME);
        }
        if (g_rs.fr_a != 0 && g_rs.fr_a != -1) {
          if (g_rs.fr_len == 0) cov_hit(CV_HR_ST_ERR);
          else if (g_rs.fr_len > 0) cov_hit(CV_HR_ST_OK);
        } else {
          cov_hit(CV_HR_ST_NOFRAME);
        }
      }
      if (p8 != 0) cov_hit(CV_HR_ST_ABORT);
    }
    t30_free_channels(o);
    trial_end2("hrs");
  }
  dsplibs_debug_level = 0;
}

static void hr_between(void)
{
  int t;
  for (t = 0; t < 3500; t++) {
    FaxT30 *o = &g_t30;
    int k, p8, n_in;
    unsigned dbg = rmod(4);
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    dsplibs_debug_level = dbg;
    k = (int)rmod(10);
    o->sf_fill = k < 3 ? 0 : (k < 8 ? (int)rmod(0x100) : 0xff - (int)rmod(8) + (int)rmod(4));
    if (rpct(20)) g_rs.fr_len = (int)rmod(8);
    p8 = pick_p8();
    n_in = pick_n_in();
    io_prep(&g_io, n_in, (int)rnd(), (int)rnd(), p8);
    emit(0, "HRB t=%d dbg=%u fill=%d", t, dbg, o->sf_fill);
    {
      int fill = o->sf_fill;
      call_state(_hdlc_receive_between_buffers_state, o, &g_io, "hrb");
      cov_hit(CV_HR_BB);
      if (g_rs.fr_len >= 0 && g_rs.fr_a != 0 && g_rs.fr_a != -1) {
        /* the frame length the recorder wrote (g_rs.fr_len, bytes) and the fill the object had */
        if (fill + g_rs.fr_len + 1 > 0xff) cov_hit(CV_HR_BB_FULL);
        else cov_hit(CV_HR_BB_STORE);
        if (g_rs.fr_len == 0) cov_hit(CV_HR_BB_ZERO);
      } else {
        cov_hit(CV_HR_BB_NOFRAME);
      }
      if (g_rs.ret_h & 0x2000u) cov_hit(CV_HR_BB_CARR);
      else cov_hit(CV_HR_BB_NOCARR);
      if (p8 != 0) cov_hit(CV_HR_BB_ABORT);
    }
    t30_free_channels(o);
    trial_end2("hrb");
  }
  dsplibs_debug_level = 0;
}

static void hr_emulate(void)
{
  int t;
  for (t = 0; t < 4000; t++) {
    FaxT30 *o = &g_t30;
    int k, nfr, total, p8, n_in;
    unsigned dbg = rmod(4);
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    dsplibs_debug_level = dbg;
    k = (int)rmod(10);
    nfr = k == 0 ? 0 : (k < 7 ? 1 + (int)rmod(4) : 1 + (int)rmod(11));
    total = build_superframe(o, nfr);
    o->sf_fill = rpct(85) ? total : (total > 1 ? total - 1 - (int)rmod(2) : total); /* (a fill past the records would hand out the 0x7fff terminator: a read beyond the object) */
    k = (int)rmod(10);
    o->emu_delay = k < 4 ? (int)rmod(3) : (k < 7 ? 2 : (k == 7 ? -1 : (int)rmod(5) - 2));
    k = (int)rmod(10);
    o->emu_idx = k < 6 ? (int)rmod((unsigned)(nfr + 2)) : (k < 8 ? nfr : (int)rmod(5) - 2);
    if (nfr == 0 && o->emu_idx < 0) o->emu_idx = 0; /* (a negative index with no frame would hand out the terminator record) */
    o->state_prev = rpct(40) ? 7 : (int)rmod(10);
    p8 = pick_p8();
    n_in = pick_n_in();
    io_prep(&g_io, n_in, (int)rnd(), (int)rnd(), p8);
    emit(0, "HRE t=%d dbg=%u nfr=%d fill=%d", t, dbg, nfr, o->sf_fill);
    {
      int prev7 = o->state_prev == 7, delay = o->emu_delay, idx = o->emu_idx;
      call_state(_hdlc_emulate_receive_state, o, &g_io, "hre");
      cov_hit(CV_HR_EM);
      if (prev7) cov_hit(CV_HR_EM_PREV7);
      else cov_hit(CV_HR_EM_OTHER);
      if (nfr == 0) cov_hit(CV_HR_EM_NONE);
      if (nfr >= 8) cov_hit(CV_HR_EM_MANY);
      if (delay > 0) cov_hit(CV_HR_EM_WAIT);
      else if (idx < nfr) cov_hit(CV_HR_EM_FRAME);
      else cov_hit(CV_HR_EM_DONE);
      if (o->sf_fill == 0 && o->emu_delay == 2 && o->emu_idx == 0) cov_hit(CV_HR_EM_RESET);
      if (!prev7 && idx < nfr) cov_hit(CV_HR_EM_W6);
    }
    t30_free_channels(o);
    trial_end2("hre");
  }
  dsplibs_debug_level = 0;
}

/* ---------------------------------------------------------------------------------------------- dispatched sequences */
static void seq_start_rx(FaxT30 *o, int mode)
{
  switch (mode) {
    case 0:
      o->s7 = 1 + (int)rmod(3);
      o->cng_on = rpct(50) ? 1 : 0;
      o->cng_tone = (int)rmod(2);
      o->cng_ctr = 0;
      _cHDLCrx_init_from_idle(o, 3);
      break;
    case 1:
      _hdlc_receive_state_init(o);
      o->state = 5;
      break;
    case 2:
      _hdlc_receive_state_init(o);
      o->state = 6;
      o->sf_fill = 0;
      break;
    case 3: {
      int nfr = 1 + (int)rmod(6);
      o->sf_fill = build_superframe(o, nfr);
      o->emu_delay = 2;
      o->emu_idx = 0;
      o->state = 7;
      break;
    }
    case 4:
      o->s7 = 1 + (int)rmod(3);
      _rx_look_carrier_init(o, k_mods[rmod((unsigned)NMODS)]);
      break;
    case 5:
      o->state = 13;
      _hdlc_receive_state_init(o);
      break;
    default:
      _recieve_silence_state_init(o, 40 + (int)rmod(200));
      break;
  }
}
static int g_seq_scen; /* 2: HDLC mix, 3: data mix (coverage ids differ) */
static void run_sequence_rx(int tag, int maxcalls, int mode, int data_id)
{
  FaxT30 *o = &g_t30;
  int c, ncalls = 0, idle_run = 0, phases = 0;
  t30_garbage(o);
  t30_channels(o, data_id);
  script_random();
  o->sec = 0;
  o->ms = 0;
  o->status = 0;
  o->s7 = 1 + (int)rmod(3);
  o->sf_fill = 0; /* the superframe words are small non-negative (a garbage fill would index outside the buffer) */
  o->emu_delay = 2;
  o->emu_idx = 0;
  dsplibs_debug_level = rmod(4);
  seq_start_rx(o, mode);
  emit(0, "SEQ %d mode=%d start state=%d", tag, mode, o->state);
  for (c = 0; c < maxcalls; c++) {
    StateFn f = state_fn(o->state);
    int p8, prev = o->state, k;
    if (!f) {
      emit(0, "SEQ %d stop at state %d after %d calls", tag, o->state, ncalls);
      break;
    }
    script_random();
    /* make progress likely: carrier appears and goes, frames complete, the data channel connects */
    if (rpct(70)) g_rs.ret_h |= 0x2000u;
    else g_rs.ret_h &= ~0x2000u;
    k = (int)rmod(10);
    if (prev == 12 && k < 3) g_rs.ret_d = (g_rs.ret_d | 0x2000u | 0x100u) & ~0x200u;
    if (prev == 13 && c < maxcalls / 2 && k < 8) g_rs.ret_d |= 0x2000u;
    if (prev == 4 && k < 3) g_rs.ret_h |= 0x2000u;
    if (prev == 5) g_rs.fr_a = rpct(30) ? 1 : 0;
    p8 = rpct(85) ? 0 : (int)rmod(30);
    io_prep(&g_io, 160, 0xa0, 0, p8);
    clock_tick(o);
    o->state_prev = prev; /* fax_class1_progress keeps the previous state; the receive states read it */
    call_state(f, o, &g_io, "seq");
    ncalls++;
    if (o->state != prev) emit(0, "SEQ %d call %d state %d -> %d", tag, c, prev, o->state);
    if (g_seq_scen == 2) {
      cov_hit(CV_HR_SEQ);
      if (prev == 4) cov_hit(CV_HR_SEQ_S4);
      if (prev == 5) cov_hit(CV_HR_SEQ_S5);
      if (prev == 6) cov_hit(CV_HR_SEQ_S6);
      if (prev == 7) cov_hit(CV_HR_SEQ_S7);
      if (prev == 8) cov_hit(CV_HR_SEQ_S8);
    } else {
      cov_hit(CV_DR_SEQ);
      if (prev == 12) cov_hit(CV_DR_SEQ_S12);
      if (prev == 13) cov_hit(CV_DR_SEQ_S13);
      if (prev == 8) cov_hit(CV_DR_SEQ_S8);
      if (prev == 16) cov_hit(CV_DR_SEQ_S16);
      if (o->state == 5) cov_hit(CV_DR_SEQ_S5);
    }
    if (o->state == 8) idle_run++;
    else idle_run = 0;
    if (idle_run > 3) {
      if (phases < (maxcalls > 500 ? 80 : 5)) {
        int m = g_seq_scen == 2 ? (int)rmod(4) : 4 + (int)rmod(3);
        if (m == 5 && !o->data_vmi) m = 4;
        phases++;
        emit(0, "SEQ %d phase %d restart mode %d after %d calls", tag, phases, m, ncalls);
        seq_start_rx(o, m);
        idle_run = 0;
      } else {
        emit(0, "SEQ %d idle after %d calls", tag, ncalls);
        break;
      }
    }
  }
  if (ncalls > 300) cov_hit(g_seq_scen == 2 ? CV_HR_SEQ_LONG : CV_DR_SEQ_LONG);
  t30_free_channels(o);
  trial_end2("seq");
  dsplibs_debug_level = 0;
}
static void hr_sequences(void)
{
  int t;
  g_seq_scen = 2;
  for (t = 0; t < 220; t++) run_sequence_rx(t, t % 10 == 0 ? 1200 : 450, t % 4, -1);
}

static void scen_hdlc(void)
{
  poke_consts(0);
  dump_consts();
  hr_lookcarrier();
  hr_state();
  hr_between();
  hr_emulate();
  hr_sequences();
}
