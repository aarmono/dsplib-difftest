/* R28 harness part 4 (ENG build): the REAL FAXVMI layer and the REAL fax modems (V.21 HDLC channels, V.27ter / V.29 / V.17 data transmitters) behind the T.30 transmit states, on both
 * sides (the blob's own FAXVMI / engines vs src/fax/fax_vmi.c + the ported engines).  R29's input functions (_handle_data_input / _handle_hdlc_input / _handle_hdlc_input_open) are
 * HARNESS SHIMS here, transcribed from the blob's machine code (so that the sessions get real frames / data through the state machines); the same shims run on both sides.
 * Scenarios: 5 HDLC sessions (silence, preamble with DLE stuffed frames, send buffer, between buffers, the receive channel off state fed with the transmit samples),
 * 6 data sessions (every modem value: scrambled ones, data, nulls with the real FIFO and bytes from the DTE), 7 _init_transmitter / _delete_data_tx_modem life cycle with the real
 * init_vmi_data_tx_modem functions.  Included by fax_t30_tx.main.c. */
#ifdef HARNESS_ENG

#include "fax/fax_modem_views.h"
#include "fax/fax_v21_tables.h"

#ifndef TEST_BLOB
/* the port runs link src/v32/v32_datapath.c (the ported VTB_decoder behind FAXVTB_decoder); the symbols that file references but this test never reaches are abort stubs */
static void unreachable_stub(const char *n)
{
  fprintf(stderr, "r28: unexpected call of %s\n", n);
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

/* ---------------------------------------------------------------------------------------------- R29 shims (transcribed from the blob) */
static unsigned g_shim_ctr;
int _handle_data_input(FaxT30 *o, const unsigned char *in, unsigned short *out, int *n)
{
  int esi = 0, ebx;
  if (o->dataEnd != 0) {
    *n = 0;
    return 0;
  }
  if (*n > 0) o->lastByte = in[*n - 1];
  for (ebx = 0; *n > ebx; ebx++) {
    unsigned char al;
    g_shim_ctr++;
    al = in[ebx];
    if (o->dlePending != 0) {
      if (dsplibs_debug_level > 2) dsplibs_debug_printf("%02X", (unsigned)al);
      o->dlePending = 0;
      al = in[ebx];
      if (al == 0x10) {
        out[esi++] = 0x10;
      } else if (al == 3) {
        int k;
        o->dataEnd = 1;
        for (k = 0x13; k >= 0; k--)
          if (esi <= 0x7ff) out[esi++] = 0;
        *n = esi;
        return 0;
      }
    } else if (al == 0x10) {
      o->dlePending = 1;
    } else {
      out[esi++] = al;
    }
  }
  *n = esi;
  return 0;
}
int _handle_hdlc_input_open(FaxT30 *o)
{
  o->hdlcWr = 1;
  return 0;
}
int _handle_hdlc_input(FaxT30 *o, const unsigned char *in, unsigned short *out, int *n)
{
  int ebx = o->hdlcWr, ecx = 0, flag = 0;
  int edx = *n;
  while (edx > 0) {
    unsigned char al;
    if (o->dlePending != 0) {
      o->dlePending = 0;
      al = in[ecx];
      if (al == 0x10) {
        out[ebx++] = 0x10;
      } else if (al == 3) {
        o->hdlcWr = ebx;
        o->buf[0] = (unsigned short)(ebx - 1);
        if (o->buf[2] & 0x10) o->lastFrame = 1;
        *n = ebx;
        flag = 1;
        break;
      }
    } else {
      al = in[ecx];
      if (al == 0x10) o->dlePending = 1;
      else out[ebx++] = al;
    }
    ecx++;
    edx = *n;
    if (!(edx > ecx)) break;
  }
  o->hdlcWr = ebx;
  if (flag == 0) {
    *n = 0;
    return 0;
  }
  return 1;
}

/* ---------------------------------------------------------------------------------------------- real channels */
static void *g_hblk[2];
static void eng_hdlc_channels(FaxT30 *o)
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
  g_hblk[0] = mc;
  g_hblk[1] = mc2;
}
static void eng_free_channels(FaxT30 *o)
{
  if (o->hdlc_tx) FAXVMI_delete(o->hdlc_tx);
  if (o->hdlc_rx) FAXVMI_delete(o->hdlc_rx);
  o->hdlc_tx = o->hdlc_rx = NULL;
  if (g_hblk[0]) sysdep_free(g_hblk[0]);
  if (g_hblk[1]) sysdep_free(g_hblk[1]);
  g_hblk[0] = g_hblk[1] = NULL;
  if (o->data_vmi) {
    /* the data channel is deleted by _delete_data_tx_modem */
  }
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
  emit(0, "%s VMI mode=%d flag=%d s08=%u s0a=%u s0c=%u id=%u mcfg=%s w14=%d w18=%d w1c=%d w20=%08x size=%u rd=%u wr=%u cnt=%u ovf=%d txm=%x txc=%x txn=%u rxs=%d tl=%d tf=%d", tag,
       o->cfg.mode, o->cfg.flag, (unsigned)o->cfg.ringMin, (unsigned)o->cfg.maxItems, (unsigned)o->cfg.frmWords, (unsigned)o->cfg.id, dptr(o->cfg.mcfg, b1, sizeof b1), o->cfg.w14, o->txEmpty, o->rxOverflow, o->stateWord,
       (unsigned)s->size, (unsigned)s->rd, (unsigned)s->wr, (unsigned)s->cnt, s->ovf, (unsigned)s->tx_mask, (unsigned)s->tx_cur, (unsigned)s->tx_nbits, s->rx_state, (int)s->tx_left, s->tx_flag);
}
static void eng_dump(const char *tag, const FaxT30 *o)
{
  dump_vmi_eng(tag, o->hdlc_tx);
  dump_vmi_eng(tag, o->hdlc_rx);
  dump_vmi_eng(tag, o->data_vmi);
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

static void eng_hdlc_session(int tag)
{
  FaxT30 *o = &g_t30;
  int c, ncalls = 0, idle_run = 0, mode, phases = 0;
  short prev_tx[512];
  unsigned st_seen = 0;
  memset(prev_tx, 0, sizeof prev_tx);
  t30_garbage(o);
  o->tone = NULL;
  o->sec = 0;
  o->ms = 0;
  o->lastFrame = 0;
  eng_hdlc_channels(o);
  dsplibs_debug_level = rmod(4);
  mode = tag % 4;
  make_stream(1 + (int)rmod(4), rpct(60));
  cov_hit(CV_E_HSESS);
  emit(0, "ESEQ %d mode=%d stream=%u", tag, mode, g_stream_len);
  if (mode == 0) {
    cHDLCtx_preamble_state_init(o);
    o->state = 0;
  } else if (mode == 1) {
    cHDLCtx_preamble_state_init(o);
    o->state = 1;
  } else if (mode == 2) {
    _send_hdlc_between_buffer_state_init(o);
    o->state = 3;
    o->tmr = 2;
  } else {
    cHDLCtx_off_init(o);
    o->state = 17;
  }
  eng_dump("h0", o);
  for (c = 0; c < 700; c++) {
    StateFn f = state_fn(o->state);
    int p8 = 0, prev = o->state;
    unsigned avail;
    if (!f) {
      emit(0, "ESEQ %d stop at state %d after %d calls", tag, o->state, ncalls);
      break;
    }
    io_prep(&g_io, 160, 0, 0, 0);
    if (prev == 17) memcpy(g_io.rx, prev_tx, sizeof(short) * 160);
    avail = g_stream_len - g_stream_pos;
    /* one frame per buffer: the preamble state appends a second frame behind the first (the blob writes the length word only once), so it gets none after the first */
    if (avail > 0 && ((prev == 1 && o->frameDone == 0) || prev == 3) && c > 2) {
      unsigned k = 1 + rmod(10);
      if (k > avail) k = avail;
      memcpy(g_io.tty_in, g_stream + g_stream_pos, k);
      g_stream_pos += k;
      p8 = (int)k;
      cov_hit(CV_E_H_FRAME);
    }
    g_io.p8 = p8;
    clock_tick(o);
    call_state(f, o, &g_io, "eh");
    eng_dump("eh", o);
    memcpy(prev_tx, g_io.tx, sizeof(short) * 160);
    if (g_io.n_out == 0xa0) {
      unsigned k;
      for (k = 0; k < 160; k++)
        if (g_io.tx[k] != 0) {
          cov_hit(CV_E_H_SAMPLES);
          break;
        }
    }
    ncalls++;
    st_seen |= 1u << (prev & 31);
    if (prev == 0) cov_hit(CV_E_H_SIL);
    if (prev == 1) cov_hit(CV_E_H_PRE);
    if (prev == 2) cov_hit(CV_E_H_BUF);
    if (prev == 3) cov_hit(CV_E_H_BTW);
    if (prev == 17) cov_hit(CV_E_H_OFF);
    if (o->state != prev) emit(0, "ESEQ %d call %d state %d -> %d", tag, c, prev, o->state);
    if (o->state == 8) idle_run++;
    else idle_run = 0;
    if (idle_run > 3) {
      if (phases < 2) {
        phases++;
        make_stream(1 + (int)rmod(3), rpct(60));
        o->dataEnd = 0;
        cHDLCtx_preamble_state_init(o);
        o->state = 1;
        idle_run = 0;
        emit(0, "ESEQ %d restart %d", tag, phases);
      } else {
        break;
      }
    }
  }
  emit(0, "ESEQ %d done calls=%d seen=%08x", tag, ncalls, st_seen);
  eng_free_channels(o);
  trial_end("ehdlc");
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
  FaxT30 *o = &g_t30;
  int c, ncalls = 0, idle_run = 0, connected = 0, sent_etx = 0;
  t30_garbage(o);
  o->tone = NULL;
  o->sec = 0;
  o->ms = 0;
  o->silenceTime = sil;
  o->lastByte = 0;
  o->dataEnd = 0;
  o->dlePending = 0;
  eng_hdlc_channels(o);
  dsplibs_debug_level = rmod(3);
  emit(0, "EDS %d mod=%02x sil=%d", tag, mod, sil);
  _tx_scrambled_ones_init(o, mod);
  eng_dump("d0", o);
  dump_t30("d0", o);
  for (c = 0; c < 420; c++) {
    StateFn f = state_fn(o->state);
    int p8 = 0, prev = o->state;
    if (!f) {
      emit(0, "EDS %d stop at state %d after %d calls", tag, o->state, ncalls);
      break;
    }
    io_prep(&g_io, 160, 0xa0, 0, 0);
    if (c > 4 && c < 320 && (c % 3) != 0) {
      unsigned k = rmod(40), q;
      for (q = 0; q < k; q++) {
        unsigned v;
        v = rnd();
        g_io.tty_in[q] = (unsigned char)(rpct(6) ? 0x10 : v);
      }
      p8 = (int)k;
    }
    if (c == 330 && !sent_etx) {
      g_io.tty_in[0] = 0x10;
      g_io.tty_in[1] = 0x03;
      p8 = 2;
      sent_etx = 1;
    }
    g_io.p8 = p8;
    clock_tick(o);
    call_state(f, o, &g_io, "ed");
    ncalls++;
    if (prev == 10) cov_hit(CV_E_D_DATA);
    if (prev == 11) cov_hit(CV_E_D_NULLS);
    if (o->txConnected == 1 && !connected) {
      connected = 1;
      cov_hit(CV_E_D_CONNECT);
    }
    if (g_io.n_out == 0xa0) {
      unsigned k;
      for (k = 0; k < 160; k++)
        if (g_io.tx[k] != 0) {
          cov_hit(CV_E_D_SAMPLES);
          break;
        }
    }
    if (o->state != prev) emit(0, "EDS %d call %d state %d -> %d", tag, c, prev, o->state);
    if (c % 25 == 0) eng_dump("ed", o);
    if (o->state == 8) idle_run++;
    else idle_run = 0;
    if (idle_run > 3) break;
  }
  emit(0, "EDS %d done calls=%d", tag, ncalls);
  eng_dump("d9", o);
  if (mod == 0x18 || mod == 0x30) cov_hit(CV_E_D_V27);
  else if (mod == 0x48 || mod == 0x60) cov_hit(CV_E_D_V29);
  else cov_hit(CV_E_D_V17);
  if (mod == 0x4a || mod == 0x62 || mod == 0x7a || mod == 0x92) cov_hit(CV_E_D_SHORT);
  cov_hit(CV_E_DSESS);
  _delete_data_tx_modem(o);
  eng_free_channels(o);
  trial_end("edata");
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
  for (m = 0; m < 12; m++) eng_data_session(t++, k_mods[rmod((unsigned)NMODS)], rpct(50) ? 0 : (int)rmod(300));
}

/* ---------------------------------------------------------------------------------------------- life cycle over the real init functions */
static void eng_life_trial(int tag, int mod, int pre)
{
  FaxT30 *o = &g_t30;
  char b[24];
  t30_garbage(o);
  o->tone = NULL;
  eng_hdlc_channels(o);
  o->silenceTime = (tag & 1) ? 0 : 20;
  o->sec = (int)rmod(100);
  o->ms = (int)rmod(100);
  dsplibs_debug_level = (unsigned)(tag & 3);
  emit(0, "ELIFE %d mod=%02x pre=%d", tag, mod, pre);
  _init_transmitter(o, mod);
  cov_hit(CV_E_LIFE);
  emit(0, "ELIFE cfg: w0=%d s08=%d s0a=%d s0c=%d s0e=%d one=%d eng=%s cfg=%s", o->data_cfg->w0, (int)o->data_cfg->s08, (int)o->data_cfg->s0a, (int)o->data_cfg->s0c, (int)o->data_cfg->s0e,
       o->data_cfg->u4.init1 == 1, dptr(o->data_cfg->eng, b, sizeof b), dptr(o->data_cfg->u10.cfg, b, sizeof b));
  eng_dump("l", o);
  dump_t30("l", o);
  if (pre) {
    /* replace the channel (second call: the debug line, the deletion, a new channel) */
    _init_transmitter(o, k_mods[rmod((unsigned)NMODS)]);
    cov_hit(CV_E_L_REPLACE);
    eng_dump("l2", o);
    dump_t30("l2", o);
  }
  _delete_data_tx_modem(o);
  cov_hit(CV_E_L_DEL);
  dump_t30("l3", o);
  eng_free_channels(o);
  trial_end("elife");
  dsplibs_debug_level = 0;
}
static void scen_eng_life(void)
{
  int t, m;
  t = 0;
  for (m = 0; m < NMODS; m++) {
    eng_life_trial(t++, k_mods[m], 0);
    eng_life_trial(t++, k_mods[m], 1);
    cov_hit(CV_E_L_MODS);
  }
}
#endif
