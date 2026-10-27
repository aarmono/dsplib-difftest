/* R28 harness part 2: scenario 2 (the HDLC transmit states: single calls and dispatched sequences).  Included by fax_t30_tx.main.c. */

static void hdlc_single(void)
{
  int t;
  static const int e_sil[] = {0x190}, e_pre[] = {0x1f40, 0x9c40}, e_btw[] = {1, 2, 0xfa}, e_off[] = {2, 0xf};
  for (t = 0; t < 4000; t++) {
    FaxT30 *o = &g_t30;
    int which, p8, n_in;
    unsigned dbg;
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    if (rpct(50)) g_rs.proc_ret |= 0x2000u;
    else g_rs.proc_ret &= ~0x2000u;
    poke_consts(rpct(40) ? 1 + (int)rmod(2) : 0);
    dbg = rmod(4);
    dsplibs_debug_level = dbg;
    which = (int)rmod(5);
    o->sec = (int)rmod(100);
    o->ms = (int)rmod(100);
    o->lastFrame = rpct(40) ? 1 : (rpct(50) ? 0 : (int)rnd());
    o->frameDone = rpct(50) ? 0 : (int)rnd();
    o->frameCount = rpct(30) ? 0 : (int)rmod(300);
    o->state = (int)rmod(20);
    o->status = (int)rnd();
    p8 = rpct(40) ? 0 : (rpct(10) ? -(int)(1 + rmod(5)) : (rpct(30) ? 0x200 : (int)rmod(40)));
    n_in = rpct(60) ? 160 : (int)rnd();
    io_prep(&g_io, n_in, (int)rnd(), (int)rnd(), p8);
    switch (which) {
      case 0:
        o->tmr = pick_tmr(e_sil, 1);
        emit(0, "SB t=%d dbg=%u", t, dbg);
        call_state(_t30_silence_before_tx_state, o, &g_io, "silence_tx");
        cov_hit(CV_H_PRE);
        if (o->state == 1) cov_hit(CV_H_PRE_50);
        if (dbg > 1 && o->state == 1) cov_hit(CV_H_PRE_DBG);
        break;
      case 1:
        o->tmr = pick_tmr(e_pre, 2);
        emit(0, "PP t=%d dbg=%u", t, dbg);
        call_state(_t30_preabmle_state, o, &g_io, "preamble");
        cov_hit(CV_H_PP);
        if (p8 > 0) cov_hit(CV_H_PP_IN);
        else cov_hit(CV_H_PP_NOIN);
        if (p8 > 0 && g_rs.hhi_ret != 0) cov_hit(CV_H_PP_FRAME);
        if (p8 > 0 && g_rs.hhi_ret == 0) cov_hit(CV_H_PP_ZERO);
        if (p8 > 0 && dbg > 2) cov_hit(CV_H_PP_DBG3);
        if ((unsigned)o->tmr > 0x1f40u + 0xa0u && o->state == 2) cov_hit(CV_H_PP_1S);
        if ((unsigned)o->tmr > 0x1f40u + 0xa0u && o->state != 2 && o->frameDone == 0) cov_hit(CV_H_PP_1S_NOFRAME);
        if (o->status == 4) cov_hit(CV_H_PP_5S);
        break;
      case 2:
        emit(0, "SH t=%d dbg=%u", t, dbg);
        call_state(_send_hdlc_buffer_state, o, &g_io, "send_buf");
        cov_hit(CV_H_SB);
        if (g_rs.sts[2] == 1) cov_hit(CV_H_SB_EMPTY);
        else cov_hit(CV_H_SB_BUSY);
        if (dbg > 1 && g_rs.sts[2] == 1) cov_hit(CV_H_SB_DBG);
        break;
      case 3:
        o->tmr = pick_tmr(e_btw, 3);
        emit(0, "HB t=%d dbg=%u", t, dbg);
        call_state(_send_hdlc_between_buffer_state, o, &g_io, "between");
        cov_hit(CV_H_HB);
        if (p8 > 0 && (unsigned)o->tmr > 1u) cov_hit(CV_H_HB_IN);
        if (p8 > 0 && (unsigned)o->tmr <= 1u) cov_hit(CV_H_HB_T1);
        if (o->status == 5 && o->state == 8) cov_hit(CV_H_HB_TO);
        if (dbg > 1 && o->status == 5) cov_hit(CV_H_HB_DBG);
        break;
      default:
        o->tmr = pick_tmr(e_off, 2);
        emit(0, "OFF t=%d dbg=%u", t, dbg);
        call_state(cHDLCtx_off, o, &g_io, "off");
        cov_hit(CV_H_OFF);
        if (g_rs.proc_ret & 0x2000u) cov_hit(CV_H_OFF_BIT);
        else cov_hit(CV_H_OFF_NOBIT);
        if (o->state == 8 && o->status == 3) {
          if (g_rs.proc_ret & 0x2000u) cov_hit(CV_H_OFF_END15);
          else cov_hit(CV_H_OFF_END2);
          if (dbg > 1) cov_hit(CV_H_OFF_DBG);
        } else {
          cov_hit(CV_H_OFF_STAY);
        }
        break;
    }
    t30_free_channels(o);
    trial_end("hdlc_single");
  }
  dsplibs_debug_level = 0;
}

/* between-state specifics the random single calls reach rarely: the timer 2 branch */
static void hdlc_between(void)
{
  int t;
  for (t = 0; t < 400; t++) {
    FaxT30 *o = &g_t30;
    unsigned dbg;
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    dbg = rmod(4);
    dsplibs_debug_level = dbg;
    o->tmr = 2;
    o->lastFrame = (t & 1) ? 1 : (int)(rpct(50) ? 0 : rmod(5)) + (t & 2 ? 0 : 2);
    o->sec = (int)rmod(100);
    o->ms = (int)rmod(100);
    io_prep(&g_io, 160, 0, 0, (int)rmod(3) - 1);
    emit(0, "HB2 t=%d dbg=%u", t, dbg);
    call_state(_send_hdlc_between_buffer_state, o, &g_io, "between2");
    cov_hit(CV_H_HB_T2);
    if (o->lastFrame == 1) cov_hit(CV_H_HB_T2_IDLE);
    else cov_hit(CV_H_HB_T2_OTHER);
    if (g_rs.hhi_ret != 0 && g_io.p8 != 0) cov_hit(CV_H_HB_FRAME);
    else cov_hit(CV_H_HB_NOFRAME);
    t30_free_channels(o);
    trial_end("hdlc_between");
  }
  dsplibs_debug_level = 0;
}

static void seq_start(FaxT30 *o, int mode)
{
  /* how the class 1 command layer starts a transmission / reception: the init functions of the state */
  switch (mode) {
    case 0:
      cHDLCtx_preamble_state_init(o);
      o->state = 0;
      break;
    case 1:
      cHDLCtx_preamble_state_init(o);
      o->state = 1;
      break;
    case 2:
      _send_hdlc_between_buffer_state_init(o);
      o->state = 3;
      break;
    case 3:
      cHDLCtx_off_init(o);
      o->state = 17;
      break;
    default:
      o->ans_len = (int)rmod(8);
      o->tmr = 0;
      o->state = 14;
      break;
  }
  o->lastFrame = 0;
}
static void run_sequence(int tag, int maxcalls, int mode)
{
  FaxT30 *o = &g_t30;
  int c, ncalls = 0, idle_run = 0, phases = 0;
  t30_garbage(o);
  t30_channels(o, -1);
  script_random();
  o->sec = 0;
  o->ms = 0;
  o->status = 0;
  dsplibs_debug_level = rmod(4);
  seq_start(o, mode);
  emit(0, "SEQ %d mode=%d start state=%d", tag, mode, o->state);
  for (c = 0; c < maxcalls; c++) {
    StateFn f = state_fn(o->state);
    int p8, prev = o->state;
    if (!f) {
      emit(0, "SEQ %d stop at state %d after %d calls", tag, o->state, ncalls);
      break;
    }
    script_random();
    /* make progress likely: frames complete often, the queue empties, the receive channel reports the bit */
    g_rs.sts[2] = rpct(35) ? 1u : 0u;
    if (rpct(50)) g_rs.proc_ret |= 0x2000u;
    else g_rs.proc_ret &= ~0x2000u;
    g_rs.hhi_ret = rpct(30) ? 1 : 0;
    if (rpct(8)) o->lastFrame = 1;
    p8 = rpct(70) ? 0 : (int)rmod(30);
    io_prep(&g_io, 160, 0, 0, p8);
    clock_tick(o);
    call_state(f, o, &g_io, "seq");
    ncalls++;
    if (prev >= 0 && prev < 32) g_seq_seen[prev]++;
    if (o->state != prev) emit(0, "SEQ %d call %d state %d -> %d", tag, c, prev, o->state);
    cov_hit(CV_H_SEQ);
    if (prev == 0) cov_hit(CV_H_SEQ_STATE0);
    if (prev == 1) cov_hit(CV_H_SEQ_STATE1);
    if (prev == 2) cov_hit(CV_H_SEQ_STATE2);
    if (prev == 3) cov_hit(CV_H_SEQ_STATE3);
    if (prev == 8) cov_hit(CV_H_SEQ_STATE8);
    if (prev == 17) cov_hit(CV_H_SEQ_OFF);
    if (o->state == 8) idle_run++;
    else idle_run = 0;
    if (idle_run > 3) {
      if (phases < 5) {
        int m = (int)rmod(5);
        phases++;
        emit(0, "SEQ %d phase %d restart mode %d after %d calls", tag, phases, m, ncalls);
        seq_start(o, m);
        idle_run = 0;
      } else {
        emit(0, "SEQ %d idle after %d calls", tag, ncalls);
        break;
      }
    }
  }
  if (ncalls > 300) cov_hit(CV_H_SEQ_LONG);
  t30_free_channels(o);
  trial_end("seq");
  dsplibs_debug_level = 0;
}
static void hdlc_sequences(void)
{
  int t;
  for (t = 0; t < 220; t++) run_sequence(t, t % 10 == 0 ? 1200 : 450, t % 5);
}
static void scen_hdlc(void)
{
  poke_consts(0);
  dump_consts();
  hdlc_single();
  hdlc_between();
  hdlc_sequences();
}
