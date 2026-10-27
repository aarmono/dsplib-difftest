/* R29 harness part 4 (ENG build): the REAL FAXVMI layer and the REAL fax modems (V.21 HDLC channels, V.27ter / V.29 / V.17 data transmitters and receivers) behind the T.30 states, on both
 * sides (the blob's own FAXVMI / engines / R28 transmit states / R29 input functions vs the ported ones).  Two objects: the TRANSMITTING class 1 object (R28 states) writes samples that the
 * RECEIVING one (the R29 states) reads.  Scenarios: 5 HDLC sessions (frames through _handle_hdlc_input and the V.21 channels into the receive states 4 / 5 / 6 / 7), 6 data sessions (every
 * modem value: _tx_scrambled_ones_init ... into _rx_look_carrier_init / _rx_look_carrier_state / _rx_data_state; the DTE bytes come out of tty_out), 7 _init_receiver life cycle over the real
 * init functions (every modem value, restart, class change, deletion) with the engine words it pokes dumped through the typed views.  Included by fax_t30_rx.main.c. */
#ifdef HARNESS_ENG

#include "fax/fax_modem_views.h"
#include "fax/fax_v21_tables.h"

#ifndef TEST_BLOB
/* the port runs link src/v32/v32_datapath.c (the ported VTB_decoder behind FAXVTB_decoder); the symbols that file references but this test never reaches are abort stubs */
static void unreachable_stub(const char *n)
{
  fprintf(stderr, "r29: unexpected call of %s\n", n);
  abort();
}
void FPM_ECC_cancel(void) { unreachable_stub("FPM_ECC_cancel"); }
void FPM_ECC_init(void) { unreachable_stub("FPM_ECC_init"); }
void FPM_FSE_receive(void) { unreachable_stub("FPM_FSE_receive"); }
void FPM_PPS_filter(void) { unreachable_stub("FPM_PPS_filter"); }
void FPM_SRE_recover(void) { unreachable_stub("FPM_SRE_recover"); }
void FSE_decision_128pt(void) { unreachable_stub("FSE_decision_128pt"); }
void FSE_decision_16pt(void) { unreachable_stub("FSE_decision_16pt"); }
void FSE_decision_16Tpt(void) { unreachable_stub("FSE_decision_16Tpt"); }
void FSE_decision_32pt(void) { unreachable_stub("FSE_decision_32pt"); }
void FSE_decision_4pt(void) { unreachable_stub("FSE_decision_4pt"); }
void FSE_decision_64pt(void) { unreachable_stub("FSE_decision_64pt"); }
void FSE_decision_AB(void) { unreachable_stub("FSE_decision_AB"); }
void SDMv32_descrambler(void) { unreachable_stub("SDMv32_descrambler"); }
void SDMv32_scrambler(void) { unreachable_stub("SDMv32_scrambler"); }
#endif

static struct {
  unsigned short pre[16];
  FaxT30 o;
} g_wrap2;
#define g_t30b (g_wrap2.o)

/* ---------------------------------------------------------------------------------------------- real channels */
static void *g_hblk[4];
static void eng_hdlc_channels(FaxT30 *o, int slot)
{
  FaxVmiCfg c;
  int *mc = (int *)sysdep_malloc(0x1c);
  int *mc2 = (int *)sysdep_malloc(0x18);
  memcpy(mc, V21TX_CFG, 0x1c);
  memcpy(mc2, V21RX_CFG, 0x18);
  memset(&c, 0, sizeof c);
  c.mode = 2;
  c.flag = 1;
  c.ringMin = 0x60;
  c.maxItems = 0x34;
  c.frmWords = 0;
  c.id = FAXVMI_ID_V21TX;
  c.mcfg = mc;
  o->hdlc_tx = FAXVMI_create(NULL, &c);
  memset(&c, 0, sizeof c);
  c.mode = 2;
  c.flag = 1;
  c.ringMin = 0;
  c.maxItems = 0x60;
  c.frmWords = 0x60;
  c.id = FAXVMI_ID_V21RX;
  c.mcfg = mc2;
  o->hdlc_rx = FAXVMI_create(NULL, &c);
  g_hblk[slot * 2] = mc;
  g_hblk[slot * 2 + 1] = mc2;
}
static void eng_free_channels(FaxT30 *o, int slot)
{
  if (o->hdlc_tx) FAXVMI_delete(o->hdlc_tx);
  if (o->hdlc_rx) FAXVMI_delete(o->hdlc_rx);
  o->hdlc_tx = o->hdlc_rx = NULL;
  if (g_hblk[slot * 2]) sysdep_free(g_hblk[slot * 2]);
  if (g_hblk[slot * 2 + 1]) sysdep_free(g_hblk[slot * 2 + 1]);
  g_hblk[slot * 2] = g_hblk[slot * 2 + 1] = NULL;
}
static void dump_vmi_eng(const char *tag, const FaxVmi *o)
{
  char b1[24];
  const FaxVmiSt *s;
  if (!o) {
    emit(0, "%s VMI NULL", tag);
    return;
  }
  s = o->st;
  emit(0, "%s VMI mode=%d flag=%d s08=%u s0a=%u s0c=%u id=%u mcfg=%s w14=%d w18=%d w1c=%d w20=%08x size=%u rd=%u wr=%u cnt=%u ovf=%d rxs=%d rxm=%x rxc=%x rxn=%u flen=%d hskip=%u ones=%d inf=%d", tag,
       o->cfg.mode, o->cfg.flag, (unsigned)o->cfg.ringMin, (unsigned)o->cfg.maxItems, (unsigned)o->cfg.frmWords, (unsigned)o->cfg.id, dptr(o->cfg.mcfg, b1, sizeof b1), o->cfg.w14, o->txEmpty, o->rxOverflow, o->stateWord,
       (unsigned)s->size, (unsigned)s->rd, (unsigned)s->wr, (unsigned)s->cnt, s->ovf, s->rx_state, (unsigned)s->rx_mask, (unsigned)s->rx_cur, (unsigned)s->rx_nbits, (int)s->flen,
       (unsigned)s->hskip, (int)s->ones, s->in_frame);
}
/* the engine words _init_receiver pokes, through the typed views (the blob's layout at -m32 is the typed one) */
static void dump_rx_engine(const char *tag, const FaxT30 *o)
{
  const FaxVmiM *m;
  if (!o->data_vmi || !o->data_vmi->m) {
    emit(0, "%s RXENG none", tag);
    return;
  }
  m = o->data_vmi->m;
  switch (o->data_vmi->cfg.id) {
    case FAXVMI_ID_V27RX: {
      const V27Rx *e = (const V27Rx *)m->eng;
      emit(0, "%s RXENG v27 cfg0=%d cfg2=%d cfg5=%d rcp=%d shift=%d flag=%d rate=%d s10=%d hold=%d state=%d", tag, e->cfg.w[0], (int)e->cfg.s[2], e->cfg.w[5], (int)e->bundle->agc.rcp,
           (int)e->bundle->agc.shift, (int)e->hdx->s14, (int)e->hdx->rate, (int)m->u10.s10, e->hdx->hold, (int)e->hdx->state);
      break;
    }
    case FAXVMI_ID_V29RX: {
      const V29Rx *e = (const V29Rx *)m->eng;
      emit(0, "%s RXENG v29 cfg0=%d cfg2=%d cfg5=%d rcp=%d shift=%d flag=%d rate=%d s10=%d hold=%d state=%d", tag, e->cfg.w[0], (int)e->cfg.s[2], e->cfg.w[5], (int)e->bundle->agc.rcp,
           (int)e->bundle->agc.shift, (int)e->hdx->s1c, (int)e->hdx->rate, (int)m->u10.s10, e->hdx->hold, (int)e->hdx->state);
      break;
    }
    case FAXVMI_ID_V17RX: {
      const V17Rx *e = (const V17Rx *)m->eng;
      emit(0, "%s RXENG v17 cfg0=%d cfg2=%d cfg5=%d rcp=%d shift=%d flag=%d rate=%d s10=%d hold=%d state=%d", tag, e->cfg.w[0], (int)e->cfg.s[2], e->cfg.w[5], (int)e->bundle->agc.rcp,
           (int)e->bundle->agc.shift, (int)e->hdx->s20, (int)e->hdx->rate, (int)m->u10.s10, e->hdx->hold, (int)e->hdx->state);
      break;
    }
    default: emit(0, "%s RXENG id=%u", tag, (unsigned)o->data_vmi->cfg.id); break;
  }
}
static void eng_dump(const char *tag, const FaxT30 *o)
{
  dump_vmi_eng(tag, o->hdlc_tx);
  dump_vmi_eng(tag, o->hdlc_rx);
  dump_vmi_eng(tag, o->data_vmi);
}

/* the state table of the transmitting object (R28's states) */
static StateFn tx_state_fn(int st)
{
  switch (st) {
    case 0: return _t30_silence_before_tx_state;
    case 1: return _t30_preabmle_state;
    case 2: return _send_hdlc_buffer_state;
    case 3: return _send_hdlc_between_buffer_state;
    case 8: return T_IDLE;
    case 9: return _tx_scrambled_ones_state;
    case 10: return _tx_data_state;
    case 11: return _tx_nulls_state;
    case 17: return cHDLCtx_off;
    case 18: return _tx_silence_before_scrm_ones;
    default: return NULL;
  }
}

/* DLE stuffed frame stream: frames [ff 03 / 13 fcf ... ] each followed by DLE ETX */
static unsigned char g_stream[4096];
static unsigned g_stream_len, g_stream_pos;
static void make_stream(int nframes, int final_last)
{
  int f;
  g_stream_len = 0;
  g_stream_pos = 0;
  for (f = 0; f < nframes; f++) {
    unsigned len, k;
    unsigned char ctl, fcf;
    len = 1 + rmod(14);
    ctl = (f == nframes - 1 && final_last) ? 0x13 : 0x03;
    fcf = (unsigned char)(0x80 + rmod(8));
    g_stream[g_stream_len++] = 0xff;
    g_stream[g_stream_len++] = ctl;
    g_stream[g_stream_len++] = fcf;
    for (k = 0; k < len; k++) {
      unsigned v;
      v = rnd();
      g_stream[g_stream_len++] = (unsigned char)v;
      if ((unsigned char)v == 0x10) g_stream[g_stream_len++] = 0x10;
    }
    g_stream[g_stream_len++] = 0x10;
    g_stream[g_stream_len++] = 0x03;
  }
}

static Io g_iot; /* the I/O block of the transmitting object */
static void obj_reset(FaxT30 *o)
{
  t30_garbage(o);
  o->tone = NULL;
  o->sec = 0;
  o->ms = 0;
  o->lastFrame = 0;
  o->sf_fill = 0; /* the superframe words are small non-negative (a garbage fill would index outside the buffer) */
  o->emu_delay = 2;
  o->emu_idx = 0;
  o->gain_att = 0;
  o->agc_rcp = 0;
  o->agc_shift = 0;
}

/* ---------------------------------------------------------------------------------------------- HDLC sessions */
static void eng_hdlc_session(int tag)
{
  FaxT30 *tx = &g_t30b, *rx = &g_t30;
  int c, ncalls = 0, idle_run = 0, mode = tag % 4, phases = 0, frames = 0;
  unsigned st_seen = 0;
  short prev_tx[512];
  memset(prev_tx, 0, sizeof prev_tx);
  obj_reset(tx);
  obj_reset(rx);
  rx->s7 = 6 + (int)rmod(10);
  rx->cng_on = 0;
  eng_hdlc_channels(tx, 0);
  eng_hdlc_channels(rx, 1);
  dsplibs_debug_level = rmod(4);
  make_stream(1 + (int)rmod(4), rpct(60));
  cov_hit(CV_E_HSESS);
  emit(0, "RSEQ %d mode=%d stream=%u", tag, mode, g_stream_len);
  cHDLCtx_preamble_state_init(tx);
  tx->state = (mode == 1) ? 1 : 0;
  if (mode == 0 || mode == 3) {
    _cHDLCrx_init_from_idle(rx, 3);
  } else if (mode == 1) {
    _hdlc_receive_state_init(rx);
    rx->state = 5;
  } else {
    _hdlc_receive_state_init(rx);
    rx->state = 6;
    rx->sf_fill = 0;
  }
  eng_dump("h0", tx);
  eng_dump("h0", rx);
  for (c = 0; c < 800; c++) {
    StateFn ft = tx_state_fn(tx->state), fr = state_fn(rx->state);
    int p8 = 0, ptx = tx->state, prx = rx->state;
    unsigned avail;
    short samples[160];
    int i;
    /* the transmitter (n_out = the number of samples wanted, as fax_class1_progress passes it) */
    io_prep(&g_iot, 160, 0xa0, 0, 0);
    avail = g_stream_len - g_stream_pos;
    if (avail > 0 && ((ptx == 1 && tx->frameDone == 0) || ptx == 3) && c > 2) {
      unsigned k = 1 + rmod(10);
      if (k > avail) k = avail;
      memcpy(g_iot.tty_in, g_stream + g_stream_pos, k);
      g_stream_pos += k;
      p8 = (int)k;
    }
    g_iot.p8 = p8;
    clock_tick(tx);
    if (ft) {
      call_state(ft, tx, &g_iot, "rt");
      if (ptx == 3 || ptx == 2) {
        /* a frame was transmitted: count it */
      }
    } else {
      for (i = 0; i < 160; i++) g_iot.tx[i] = 0;
      g_iot.n_out = 0xa0;
    }
    for (i = 0; i < 160; i++) samples[i] = (g_iot.n_out == 0xa0) ? g_iot.tx[i] : 0;
    if (mode == 3 && ptx == 2 && (tx->tmr & 1) == 0 && rpct(30)) { /* line noise in the middle of a frame: the receiver sees a bad CRC */
      for (i = 0; i < 160; i++) samples[i] = (short)(samples[i] ^ (short)(rnd() & 0x3fff));
    }
    /* the receiver */
    if (!fr) {
      emit(0, "RSEQ %d rx stop at state %d after %d calls", tag, rx->state, ncalls);
      break;
    }
    io_prep(&g_io, 160, 0xa0, 0, 0);
    memcpy(g_io.rx, samples, sizeof samples);
    clock_tick(rx);
    rx->state_prev = prx;
    call_state(fr, rx, &g_io, "rr");
    eng_dump("rr", rx);
    ncalls++;
    st_seen |= 1u << (prx & 31);
    if (prx == 4) cov_hit(CV_E_H_LC);
    if (prx == 5) cov_hit(CV_E_H_RX);
    if (prx == 6) cov_hit(CV_E_H_BB);
    if (prx == 7) cov_hit(CV_E_H_EM);
    if (g_io.p7 > 2 && rx->rx_event == 1) {
      cov_hit(CV_E_H_FRAME_OK);
      cov_hit(CV_E_H_BYTES);
      frames++;
      if (frames > 1) cov_hit(CV_E_H_MULTI);
    }
    if (rx->rx_event == 2 && rx->rx_kind == 2) cov_hit(CV_E_H_ERR);
    if (rx->state == 5 && prx == 4) cov_hit(CV_E_H_GAIN); /* the carrier was detected (the gain loop ran; the real AGC history is above the thresholds) */
    if (rx->agc_rcp != 0 || rx->agc_shift != 0) cov_hit(CV_E_H_AGC);
    if (rx->state != prx) emit(0, "RSEQ %d call %d rx state %d -> %d", tag, c, prx, rx->state);
    if (tx->state != ptx) emit(0, "RSEQ %d call %d tx state %d -> %d", tag, c, ptx, tx->state);
    if (rx->state == 8 && (mode != 2 || 1)) idle_run++;
    else idle_run = 0;
    if (mode == 2 && rx->state == 8 && phases < 1 && idle_run > 1) {
      /* the command mode replay of the collected superframe */
      phases++;
      rx->emu_delay = 2;
      rx->emu_idx = 0;
      rx->state = 7;
      idle_run = 0;
      emit(0, "RSEQ %d replay superframe fill=%d", tag, rx->sf_fill);
    }
    if (idle_run > 3) {
      if (phases < 2 && mode != 2) {
        phases++;
        make_stream(1 + (int)rmod(3), rpct(60));
        tx->dataEnd = 0;
        cHDLCtx_preamble_state_init(tx);
        tx->state = 1;
        _cHDLCrx_init_from_idle(rx, 3);
        idle_run = 0;
        emit(0, "RSEQ %d restart %d", tag, phases);
      } else {
        break;
      }
    }
  }
  emit(0, "RSEQ %d done calls=%d seen=%08x frames=%d", tag, ncalls, st_seen, frames);
  eng_free_channels(tx, 0);
  eng_free_channels(rx, 1);
  trial_end("rhdlc");
  dsplibs_debug_level = 0;
}
static void scen_eng_hdlc(void)
{
  int t;
  for (t = 0; t < 40; t++) eng_hdlc_session(t);
}

/* ---------------------------------------------------------------------------------------------- data sessions */
static void eng_data_session(int tag, int mod, int sil)
{
  FaxT30 *tx = &g_t30b, *rx = &g_t30;
  int c, ncalls = 0, idle_run = 0, connected = 0, sent_etx = 0, got = 0, was13 = 0;
  obj_reset(tx);
  obj_reset(rx);
  tx->silenceTime = sil;
  tx->lastByte = 0;
  tx->dataEnd = 0;
  tx->dlePending = 0;
  rx->s7 = 8 + (int)rmod(8);
  rx->modemClass = (int)rnd();
  rx->cd_flag = rpct(70) ? 0 : (int)rmod(3);
  rx->agc_rcp = rpct(40) ? 0 : (int)(short)rnd();
  rx->agc_shift = rpct(40) ? 0 : (int)(short)rnd();
  eng_hdlc_channels(tx, 0);
  eng_hdlc_channels(rx, 1);
  dsplibs_debug_level = rmod(3);
  emit(0, "RDS %d mod=%02x sil=%d", tag, mod, sil);
  _tx_scrambled_ones_init(tx, mod);
  _rx_look_carrier_init(rx, mod);
  eng_dump("d0", tx);
  eng_dump("d0", rx);
  dump_rx_engine("d0", rx);
  dump_t30("d0t", tx);
  dump_t30("d0r", rx);
  for (c = 0; c < 480; c++) {
    StateFn ft = tx_state_fn(tx->state), fr = state_fn(rx->state);
    int p8 = 0, ptx = tx->state, prx = rx->state, i;
    short samples[160];
    io_prep(&g_iot, 160, 0xa0, 0, 0);
    if (c > 4 && c < 330 && (c % 3) != 0) {
      unsigned k = rmod(40), q;
      for (q = 0; q < k; q++) {
        unsigned v;
        v = rnd();
        g_iot.tty_in[q] = (unsigned char)(rpct(6) ? 0x10 : v);
      }
      p8 = (int)k;
    }
    if (c == 340 && !sent_etx) {
      g_iot.tty_in[0] = 0x10;
      g_iot.tty_in[1] = 0x03;
      p8 = 2;
      sent_etx = 1;
    }
    g_iot.p8 = p8;
    clock_tick(tx);
    if (ft) {
      call_state(ft, tx, &g_iot, "dt");
    } else {
      for (i = 0; i < 160; i++) g_iot.tx[i] = 0;
      g_iot.n_out = 0xa0;
    }
    for (i = 0; i < 160; i++) samples[i] = (g_iot.n_out == 0xa0) ? g_iot.tx[i] : 0;
    if (!fr) {
      emit(0, "RDS %d rx stop at state %d after %d calls", tag, rx->state, ncalls);
      break;
    }
    io_prep(&g_io, 160, 0xa0, 0, 0);
    memcpy(g_io.rx, samples, sizeof samples);
    clock_tick(rx);
    rx->state_prev = prx;
    call_state(fr, rx, &g_io, "dr");
    ncalls++;
    if (rx->state == 13 && !connected) {
      connected = 1;
      cov_hit(CV_E_D_CONNECT);
    }
    if (prx == 13) {
      was13 = 1;
      if (g_io.p7 > 0) {
        got += g_io.p7;
        cov_hit(CV_E_D_BYTES);
      }
    }
    if (was13 && rx->state == 8 && prx == 13) cov_hit(CV_E_D_CARRIER_LOST);
    if (rx->state != prx) emit(0, "RDS %d call %d rx state %d -> %d", tag, c, prx, rx->state);
    if (tx->state != ptx) emit(0, "RDS %d call %d tx state %d -> %d", tag, c, ptx, tx->state);
    if (c % 25 == 0) {
      eng_dump("dr", rx);
      dump_rx_engine("dr", rx);
    }
    if (rx->state == 8 && tx->state == 8) idle_run++;
    else idle_run = 0;
    if (idle_run > 3) break;
  }
  emit(0, "RDS %d done calls=%d bytes=%d", tag, ncalls, got);
  eng_dump("d9", rx);
  dump_rx_engine("d9", rx);
  if (mod == 0x18 || mod == 0x30) cov_hit(CV_E_D_V27);
  else if (mod == 0x48 || mod == 0x60) cov_hit(CV_E_D_V29);
  else cov_hit(CV_E_D_V17);
  if (mod == 0x4a || mod == 0x62 || mod == 0x7a || mod == 0x92) cov_hit(CV_E_D_SHORT);
  cov_hit(CV_E_DSESS);
  _delete_data_tx_modem(tx);
  _delete_data_rx_modem(rx);
  eng_free_channels(tx, 0);
  eng_free_channels(rx, 1);
  trial_end("rdata");
  dsplibs_debug_level = 0;
}
static void scen_eng_data(void)
{
  int t, m;
  t = 0;
  for (m = 0; m < NMODS; m++) {
    eng_data_session(t++, k_mods[m], 0);
    eng_data_session(t++, k_mods[m], 60 + (int)rmod(200));
  }
  for (m = 0; m < 8; m++) eng_data_session(t++, k_mods[rmod((unsigned)NMODS)], rpct(50) ? 0 : (int)rmod(300));
}

/* ---------------------------------------------------------------------------------------------- life cycle over the real init functions */
static void eng_life_trial(int tag, int mod, int pre)
{
  FaxT30 *o = &g_t30;
  char b[24];
  t30_garbage(o);
  o->tone = NULL;
  eng_hdlc_channels(o, 1);
  o->sec = (int)rmod(100);
  o->ms = (int)rmod(100);
  o->cd_flag = rpct(40) ? 0 : (int)rnd();
  o->agc_rcp = (int)rnd();
  o->agc_shift = (int)rnd();
  dsplibs_debug_level = (unsigned)(tag & 3);
  emit(0, "RLIFE %d mod=%02x pre=%d", tag, mod, pre);
  if (pre == 1) {
    /* an existing receiver of another class, then a restart of the same class, then a class change */
    _init_receiver(o, k_mods[rmod((unsigned)NMODS)]);
    cov_hit(CV_E_L_REPLACE);
    dump_rx_engine("l0", o);
  }
  _init_receiver(o, mod);
  cov_hit(CV_E_LIFE);
  emit(0, "RLIFE cfg: w0=%d s08=%d s0a=%d s0c=%d s0e=%d one=%d eng=%s cfg=%s", o->data_cfg->w0, (int)o->data_cfg->s08, (int)o->data_cfg->s0a, (int)o->data_cfg->s0c, (int)o->data_cfg->s0e,
       o->data_cfg->u4.init1 == 1, dptr(o->data_cfg->eng, b, sizeof b), dptr(o->data_cfg->u10.cfg, b, sizeof b));
  eng_dump("l", o);
  dump_rx_engine("l", o);
  dump_t30("l", o);
  if (pre == 2) {
    /* restart with the same modem and with a changed flag word */
    o->cd_flag = (int)rnd();
    o->agc_rcp = (int)rnd();
    _init_receiver(o, mod);
    cov_hit(CV_E_L_REPLACE);
    eng_dump("l2", o);
    dump_rx_engine("l2", o);
    dump_t30("l2", o);
  }
  _delete_data_rx_modem(o);
  cov_hit(CV_E_L_DEL);
  dump_t30("l3", o);
  eng_free_channels(o, 1);
  trial_end("rlife");
  dsplibs_debug_level = 0;
}
static void scen_eng_life(void)
{
  int t, m;
  t = 0;
  for (m = 0; m < NMODS; m++) {
    int p;
    for (p = 0; p < 3; p++) eng_life_trial(t++, k_mods[m], p);
    cov_hit(CV_E_L_MODS);
  }
}
#endif
