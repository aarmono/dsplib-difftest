/*
 * R06 differential harness, id r06_eng: the V.22 / V.22bis / Bell 212A engine - V22FP_create / V22FP_delete / V22FP_modem / V22FP_control / V22FP_GetDiagnostics / V22_status and the
 * helpers (SetTxRate, SetRxRate, ScrambleDataV22, DescrambleDataV22, ModDataV22, DemodDataV22, ResetRx, SetAdaptEqV22, TxClockSync, MakeTxData, RxTrained1200 / 2400, TxNOP, RxClampV22,
 * ReadGTimer, CarrierDetect, SignalDetect, GetSignalQuality, ScramblerOn, DescramblerOn): the real dsplibs.o (32-bit) vs src/v22/v22_engine.c.  ONE source.  Library: test/v22/r06_common.h (hashed
 * canonical output, deterministic heap with red zones and id-patterned garbage, callee recorders, coverage counters exit 2, 3 = harness violation) + test/v22/r06_eng_common.h (the layout
 * independent description of the engine, the scripted handler recorders, engine creation).  Everything printed is architecture independent: the 32-bit BLOB run is the golden of the
 * x86-64 build (test/golden/v22/v22_engine.blob32.out.gz).
 *
 * Callees.  REAL on each side: the FPM_TONE / FPM_MTD / FPM_AGC / FPM_SDM / FPM_SMC / V22_PPS / V22_MRF / V22_SRE / V22_FSE / V22IIRFilterInit / V22_iir_filt_demod / FPM_rms functions (the
 * blob's own / the port's: R04, R05 and the FPM_* ports), sysdep_malloc / sysdep_free / sysdep_memcpy are the harness' recorders (ids, memcpy lengths and hashes).  CALLEE RECORDERS on
 * BOTH sides: the seven state handlers V22FP_modem dispatches to through V22_PROTOCOL (v22_data, v22_originate, v22_answer, v22_local_loop, v22_org_rmloop2, v22_ans_rmloop2, v22_retrain =
 * R07 / R08, still blob functions: the blob's copies are made weak with objcopy and these definitions win): each logs the engine fields, its counts and hashes of its buffers and
 * applies a scripted effect (engine state byte, flag bytes, tones.state, rx bits, rx count).
 *
 * Scenarios (own process each):
 *   0 create / delete lifecycle: a sweep of the 7-int configuration (mode 0 / 1 / 2 / other, protocol 0 / 1 / 2 / other, the three bit words, the 700 word, the 60000 word), debug levels,
 *     the full description of the engine (every member, block ids and sizes, tables by identity, coefficient buffers, the detector / AGC / tone objects), the re-create path (V22FP_create
 *     on an existing engine: no new blocks), delete (free order) and the leak check
 *   1 V22FP_modem dispatch: every tones.state 0 .. 6, every state byte class (3 / 4 and 6 .. 8 drop to the data state), tx counts 0 .. 100, rx counts 0 .. 160, receive shifts (also >= 32),
 *     gains, flag bytes, handler effects; the returned dword, the counts, the bits, the scaled output and the engine after every call
 *   2 small functions over poked state: SetTxRate / SetRxRate (all rates), SetAdaptEqV22, TxClockSync, V22FP_control, V22_status (every state, pre-filled output), the getters, ReadGTimer,
 *     GetSignalQuality, ResetRx, MakeTxData, RxTrained1200 / 2400, TxNOP, RxClampV22, V22FP_GetDiagnostics, ScrambleDataV22 / DescrambleDataV22 / ModDataV22 on random data
 *   3 the signal chain: a transmitting engine (scramble + modulate at 1200 and 2400 bit/s) feeds a receiving engine (DemodDataV22 + descramble) block by block through gains, noise,
 *     silence, tones and saturation, over long streams, both roles (the IIR / tone demodulator path of mode 0), pokes of the receive switches
 */
#include "r06_eng_common.h"

enum {
  CV_CR = 0, CV_CR_M0, CV_CR_M1, CV_CR_M2, CV_CR_MOTHER, CV_CR_P0, CV_CR_P1, CV_CR_P2, CV_CR_POTHER, CV_CR_DATASTART, CV_CR_B3, CV_CR_B5, CV_CR_B6, CV_CR_DBG, CV_RECREATE, CV_DELETE,
  CV_STATE0 = 20, CV_STATE1, CV_STATE2, CV_STATE3, CV_STATE4, CV_STATE5, CV_STATE6, CV_H0, CV_H1, CV_H2, CV_H3, CV_H4, CV_H5, CV_H6, CV_ST3, CV_ST4, CV_ST6, CV_ST7, CV_ST8, CV_ST_OTHER, CV_ST0,
  CV_RXZERO, CV_RXN, CV_TXN, CV_SHIFT0, CV_SHIFT_N, CV_SHIFT_BIG, CV_GAIN_DEF, CV_GAIN_NEG, CV_GAIN_BIG, CV_RX_NZST, CV_DROP_STATE, CV_NODROP,
  CV_TXR0 = 50, CV_TXR1, CV_TXR_OTHER, CV_RXR0, CV_RXR1, CV_RXR_OTHER, CV_ADEQ1, CV_ADEQ2, CV_ADEQ3, CV_ADEQ_OTHER, CV_CLKSYNC, CV_CTL, CV_CTL_D4, CV_CTL_D2, CV_STATUS, CV_GETTERS, CV_TIMER,
  CV_SQ, CV_RESETRX, CV_MAKETX0, CV_MAKETX1, CV_MAKETX2, CV_MAKETX3, CV_MAKETX4, CV_MAKETX_OTHER, CV_RT12_TRUE, CV_RT12_FALSE, CV_RT24_TRUE, CV_RT24_FALSE, CV_TXNOP, CV_CLAMP, CV_DIAG,
  CV_SCR, CV_DESCR, CV_MOD, CV_MAKETX_WRAP,
  CV_DEM = 80, CV_DEM_M0, CV_DEM_M1, CV_DEM_EARLY, CV_DEM_THRU, CV_DEM_SYMS, CV_DEM_STATE0, CV_DEM_STATE_NZ, CV_DEM_ADAPT, CV_DEM_NOADAPT, CV_DEM_PLLC0, CV_DEM_PLLC1, CV_DEM_SREMODE0, CV_DEM_SREMODE_NZ,
  CV_DEM_TIE, CV_DEM_1200, CV_DEM_2400, CV_DEM_SILENCE, CV_DEM_NOISE, CV_DEM_SAT, CV_DEM_CLEAN, CV_MOD_OUT, CV_DEM_CARRIER
};

extern short FPM_rms(const short *data, short count); /* the real one of each side: measures the block energy the receiver compares with its threshold */
static short g_a[70000 + 64];
static void fill_samples(short *p, int n, int kind, int amp)
{
  int i;
  static unsigned ph;
  for (i = 0; i < n; i++) {
    switch (kind) {
      case 0: p[i] = (short)rnd(); break;
      case 1: p[i] = (short)(rpct(50) ? 32767 : -32768); break;
      case 2: p[i] = (short)((int)(rnd() & 0xff) - 128); break;
      case 3: ph += 0x1a2b3c4dU; p[i] = (short)((((ph >> 16) & 0x7fff) ^ ((ph >> 15) & 1 ? 0x7fff : 0)) - 0x4000); break; /* a ramp / triangle */
      default: p[i] = (short)((int)(rnd() % (unsigned)(2 * amp + 1)) - amp); break;
    }
  }
}

/* ================================================================================================ scenario 0: create / delete */
static void sc_lifecycle(void)
{
  int k, n = 0;
  cov_def(CV_CR, "create: fresh engine");
  cov_def(CV_CR_M0, "create: mode 0");
  cov_def(CV_CR_M1, "create: mode 1");
  cov_def(CV_CR_M2, "create: mode 2");
  cov_def(CV_CR_MOTHER, "create: mode other (keeps the default 1)");
  cov_def(CV_CR_P0, "create: protocol 0 (2400)");
  cov_def(CV_CR_P1, "create: protocol 1 (1200)");
  cov_def(CV_CR_P2, "create: protocol 2 (1200)");
  cov_def(CV_CR_POTHER, "create: protocol other (keeps 2400)");
  cov_def(CV_CR_DATASTART, "create: cfg[5] bit set (start in the data state)");
  cov_def(CV_CR_B3, "create: cfg[3] bit set");
  cov_def(CV_CR_B5, "create: cfg[5] low bit clear");
  cov_def(CV_CR_B6, "create: cfg[6] low bit clear");
  cov_def(CV_CR_DBG, "create: debug level > 1 (message)");
  cov_def(CV_RECREATE, "re-create on an existing engine");
  cov_def(CV_DELETE, "delete");
  arena_init();
  for (k = 0; k < 330; k++) {
    int cfg[7], cfg2[7];
    char tag[64];
    V22Engine *e, *e2;
    V22Core *core;
    V22Tones *tones;
    unsigned lvl = lvl_value(k % 5);
    if (k < 180) make_cfg(cfg, k);
    else {
      cfg[0] = (int)(rpct(70) ? rmod(3) : rnd()); cfg[1] = (int)(rpct(70) ? rmod(3) : rnd()); cfg[2] = (int)rnd(); cfg[3] = (int)rnd(); cfg[4] = (int)rnd(); cfg[5] = (int)rnd(); cfg[6] = (int)rnd();
    }
    make_cfg(cfg2, k * 7 + 3);
    dsplibs_debug_level = lvl;
    snprintf(tag, sizeof tag, "K%d", k);
    emit(k < 3, "%s cfg %d,%d,%d,%d,%d,%d,%d level=%u", tag, cfg[0], cfg[1], cfg[2], cfg[3], cfg[4], cfg[5], cfg[6], lvl);
    e = V22FP_create(NULL, cfg);
    n++;
    cov_hit(CV_CR);
    if (lvl > 1) cov_hit(CV_CR_DBG);
    cov_hit(cfg[0] == 0 ? CV_CR_M0 : cfg[0] == 1 ? CV_CR_M1 : cfg[0] == 2 ? CV_CR_M2 : CV_CR_MOTHER);
    cov_hit(cfg[1] == 0 ? CV_CR_P0 : cfg[1] == 1 ? CV_CR_P1 : cfg[1] == 2 ? CV_CR_P2 : CV_CR_POTHER);
    if (cfg[5] & 1) cov_hit(CV_CR_DATASTART);
    if (cfg[3] & 1) cov_hit(CV_CR_B3);
    if (!(cfg[5] & 1)) cov_hit(CV_CR_B5);
    if (!(cfg[6] & 1)) cov_hit(CV_CR_B6);
    if (!e) die3("V22FP_create returned NULL");
    core = e->core;
    tones = e->tones;
    d_blocks(k < 3, e);
    d_engine(k < 2, tag, e);
    /* the re-create path: the same blocks, re-initialised with another configuration */
    dsplibs_debug_level = 0;
    {
      unsigned long nm = g_nmalloc, nf = g_nfree;
      e2 = V22FP_create(e, cfg2);
      cov_hit(CV_RECREATE);
      emit(0, "%s recreate same=%d core=%d tones=%d mallocs=%lu frees=%lu", tag, e2 == e, e->core == core, e->tones == tones, g_nmalloc - nm, g_nfree - nf);
      d_engine(k < 2, "R", e);
    }
    V22FP_delete(e);
    cov_hit(CV_DELETE);
    trial_end(tag);
  }
  emit(1, "created %d engines", n);
  cov_finish();
}

/* ================================================================================================ scenario 1: V22FP_modem */
static void sc_modem(void)
{
  static const int stv[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 200, 255};
  static const int gains[] = {13014, 13014, 13014, 0, 1, 32767, 32768, 65536, -1, -13014, 0x7fffffff, (int)0x80000000u, 1 << 20};
  int t, it;
  cov_def(CV_STATE0, "tones.state 0 (v22_data)"); cov_def(CV_STATE1, "tones.state 1"); cov_def(CV_STATE2, "tones.state 2"); cov_def(CV_STATE3, "tones.state 3");
  cov_def(CV_STATE4, "tones.state 4"); cov_def(CV_STATE5, "tones.state 5"); cov_def(CV_STATE6, "tones.state 6");
  cov_def(CV_H0, "handler 0 called"); cov_def(CV_H1, "handler 1 called"); cov_def(CV_H2, "handler 2 called"); cov_def(CV_H3, "handler 3 called"); cov_def(CV_H4, "handler 4 called");
  cov_def(CV_H5, "handler 5 called"); cov_def(CV_H6, "handler 6 called");
  cov_def(CV_ST3, "engine state 3 after the handler (drop to data)"); cov_def(CV_ST4, "engine state 4"); cov_def(CV_ST6, "engine state 6"); cov_def(CV_ST7, "engine state 7");
  cov_def(CV_ST8, "engine state 8"); cov_def(CV_ST_OTHER, "engine state 1 / 2 / 5 / 9 .. (no drop)"); cov_def(CV_ST0, "engine state 0 (rx bits delivered)");
  cov_def(CV_RXZERO, "rx count 0"); cov_def(CV_RXN, "rx count > 0"); cov_def(CV_TXN, "tx count > 0");
  cov_def(CV_SHIFT0, "rx shift 0"); cov_def(CV_SHIFT_N, "rx shift 1 .. 31"); cov_def(CV_SHIFT_BIG, "rx shift >= 32 (modulo 32)");
  cov_def(CV_GAIN_DEF, "gain 13014"); cov_def(CV_GAIN_NEG, "gain negative"); cov_def(CV_GAIN_BIG, "gain > 32768");
  cov_def(CV_RX_NZST, "rx count cleared by a non-zero state"); cov_def(CV_DROP_STATE, "tones.state / sc reset by the state"); cov_def(CV_NODROP, "no reset");
  arena_init();
  for (t = 0; t < 6; t++) {
    V22Engine *e;
    int cfg[7];
    make_cfg(cfg, t * 11 + 1);
    cfg[4] = 700;
    dsplibs_debug_level = 0;
    e = V22FP_create(NULL, cfg);
    for (it = 0; it < 700; it++) {
      int txBits[104], rxs[8 + 200 + 8], *rxBits = rxs + 8;
      short ins[8 + 170 + 8], outs[8 + 170 + 8], *in = ins + 8, *out = outs + 8;
      unsigned txc = rmod(101), rxc = rmod(161), rxc0, txc0;
      unsigned ret;
      int i, ts = (int)rmod(7);
      int stv0 = stv[rmod(13)];
      int shift = rpct(40) ? 0 : (rpct(70) ? (int)rmod(17) : (rpct(50) ? (int)rmod(70) : (int)(short)rnd()));
      if (rpct(5)) rxc = 160;
      if (rpct(5)) txc = 0;
      if (rpct(4)) txc = rpct(50) ? 0xffffffffu : 0x80000000u + rmod(3); /* negative counts (the loops compare signed: nothing is copied) */
      if (rpct(4)) rxc = rpct(50) ? 0xffffffffu : 0x80000000u + rmod(3);
      e->tones->state = (short)ts;
      e->tones->shift = (short)shift;
      e->state = (unsigned char)stv0;
      e->flags1d = (unsigned char)rnd();
      e->flags1e = (unsigned char)rnd();
      e->tones->sc = (short)rnd();
      e->cfg.gain = gains[rmod(13)];
      g_hs.fill_out = rpct(85);
      g_hs.rx_n = rpct(20) ? 0 : (int)rmod(101);
      g_hs.tx_write = (int)(short)rnd();
      g_hs.new_state = rpct(35) ? -1 : stv[rmod(13)];
      g_hs.new_f1d = rpct(50) ? -1 : (int)(rnd() & 0xff);
      g_hs.new_f1e = rpct(50) ? -1 : (int)(rnd() & 0xff);
      g_hs.new_tstate = rpct(50) ? -1 : (int)rmod(7);
      g_hs.new_sc = rpct(50) ? -1 : (int)(short)rnd();
      g_hs.mode = 0;
      for (i = 0; i < 104; i++) txBits[i] = (int)rnd();
      for (i = 0; i < 8 + 200 + 8; i++) rxs[i] = i >= 8 && i < 208 ? 0x7eadbeef : 0x1badf00d;
      fill_samples(in, 170, it % 7, 2000);
      for (i = 0; i < 8; i++) ins[i] = ins[8 + 170 + i] = outs[i] = outs[8 + 170 + i] = 0x5a5a;
      for (i = 0; i < 170; i++) out[i] = (short)rnd();
      e->b1f = (unsigned char)(rpct(30) ? rnd() : 0);
      rxc0 = rxc;
      txc0 = txc;
      cov_hit(CV_STATE0 + ts);
      if (shift == 0) cov_hit(CV_SHIFT0);
      else if (shift > 0 && shift < 32) cov_hit(CV_SHIFT_N);
      else cov_hit(CV_SHIFT_BIG);
      if (e->cfg.gain == 13014) cov_hit(CV_GAIN_DEF);
      else if (e->cfg.gain < 0) cov_hit(CV_GAIN_NEG);
      else if (e->cfg.gain > 32768) cov_hit(CV_GAIN_BIG);
      if ((int)rxc > 0) cov_hit(CV_RXN); else cov_hit(CV_RXZERO);
      if ((int)txc > 0) cov_hit(CV_TXN);
      reg_clear();
      reg_add(in, sizeof in, 2, "IN");
      reg_add(out, sizeof out, 2, "OUT");
      {
        unsigned long before[7];
        for (i = 0; i < 7; i++) before[i] = g_hcalls[i];
        ret = V22FP_modem(e, txBits, out, in, rxBits, &txc, &rxc);
        for (i = 0; i < 7; i++)
          if (g_hcalls[i] != before[i]) cov_hit(CV_H0 + i);
      }
      if (txc != txc0) die3("V22FP_modem changed *txCount");
      for (i = 0; i < 8; i++)
        if (ins[i] != 0x5a5a || ins[8 + 170 + i] != 0x5a5a || outs[i] != 0x5a5a || outs[8 + 170 + i] != 0x5a5a || rxs[i] != 0x1badf00d || rxs[208 + i] != 0x1badf00d) die3("V22FP_modem wrote outside a buffer");
      {
        unsigned st = e->state;
        if (st == 3) cov_hit(CV_ST3);
        else if (st == 4) cov_hit(CV_ST4);
        else if (st == 6) cov_hit(CV_ST6);
        else if (st == 7) cov_hit(CV_ST7);
        else if (st == 8) cov_hit(CV_ST8);
        else if (st == 0) cov_hit(CV_ST0);
        else cov_hit(CV_ST_OTHER);
        if (st != 0 && rxc0 != 0) cov_hit(CV_RX_NZST);
        if (e->tones->state == 0 && e->tones->sc == 0) cov_hit(CV_DROP_STATE);
        else cov_hit(CV_NODROP);
      }
      emit(0, "modem ret=%08x txc=%u rxc=%u->%u rxbits=%016llx out=%016llx", ret, txc, rxc0, rxc, fnv_ints(rxBits, 200), fnv_shorts(out, 170));
      d_engine(0, "M", e);
    }
    d_engine(1, "Mend", e);
    V22FP_delete(e);
    trial_end("modem");
  }
  emit(1, "handler calls %lu %lu %lu %lu %lu %lu %lu", g_hcalls[0], g_hcalls[1], g_hcalls[2], g_hcalls[3], g_hcalls[4], g_hcalls[5], g_hcalls[6]);
  cov_finish();
}

/* ================================================================================================ scenario 2: the small functions */
static void poke_core(V22Engine *e)
{
  V22Core *c = e->core;
  c->sdm.tap1 = (short)rnd(); c->sdm.tap2 = (short)rnd(); c->sdm.state = rnd(); c->sdm.n = (short)rmod(20); c->sdm.mask = rnd(); c->sdm.nmask = rnd(); c->sdm.sh1 = (short)rnd(); c->sdm.sh2 = (short)rnd();
  c->sdmRx.tap1 = (short)rnd(); c->sdmRx.tap2 = (short)rnd(); c->sdmRx.state = rnd(); c->sdmRx.n = (short)rmod(20); c->sdmRx.mask = rnd(); c->sdmRx.nmask = rnd(); c->sdmRx.sh1 = (short)rnd();
  c->sdmRx.sh2 = (short)rnd();
  c->smc.cfg.mask2 = (unsigned short)(rpct(50) ? 0 : 3); c->smc.cfg.shift = (unsigned short)(rpct(50) ? 0 : 2);
  c->pps.imap = rpct(50) ? SMCv22_IMAP_2400BPS : SMCv22_IMAP_1200BPS; c->pps.qmap = rpct(50) ? SMCv22_QMAP_2400BPS : SMCv22_QMAP_1200BPS;
  c->txBis = (short)(rpct(40) ? 0 : (rpct(50) ? 1 : (int)rnd())); c->rxBis = (short)(rpct(40) ? 0 : (rpct(50) ? 1 : (int)rnd())); c->fse.decide = rpct(50) ? FSEv22_decision24 : FSEv22_decision12;
  c->w0 = (int)rmod(3); c->fseS18Gate = (int)rmod(3); c->adaptGate = (int)rmod(3); c->w20 = (int)rnd(); c->scrOn = (int)rnd(); c->descrOn = (int)rnd(); c->sreGate = (int)rnd(); c->fsePllGate = (int)rnd();
  c->fse.errPow = (short)rnd(); c->fse.mu = (short)rnd(); c->fse.adaptEnable = (int)rnd(); c->sre.integ = (short)rmod(4); /* a negative step would walk the PPS phase below the coefficient table (blob and port crash alike) */ c->sre.wantTrack = (int)rnd(); c->agcIn.detect = (int)rnd();
  c->agcIn.active = (int)rnd();
  e->cfg.b11 = (unsigned char)rnd();
  e->tones->state = (short)rmod(7);
}
static void sc_small(void)
{
  int t, it, i;
  static short rates[] = {0, 1, 2, -1, 0x4000, (short)0x8000, 3, 100};
  static unsigned short aeq[] = {0, 1, 2, 3, 4, 5, 0xffff, 0x8003};
  cov_def(CV_TXR0, "SetTxRate 0"); cov_def(CV_TXR1, "SetTxRate 1"); cov_def(CV_TXR_OTHER, "SetTxRate other");
  cov_def(CV_RXR0, "SetRxRate 0"); cov_def(CV_RXR1, "SetRxRate 1"); cov_def(CV_RXR_OTHER, "SetRxRate other");
  cov_def(CV_ADEQ1, "SetAdaptEqV22 1"); cov_def(CV_ADEQ2, "SetAdaptEqV22 2"); cov_def(CV_ADEQ3, "SetAdaptEqV22 3"); cov_def(CV_ADEQ_OTHER, "SetAdaptEqV22 other");
  cov_def(CV_CLKSYNC, "TxClockSync"); cov_def(CV_CTL, "V22FP_control"); cov_def(CV_CTL_D4, "V22FP_control: control byte 0xd bit 2"); cov_def(CV_CTL_D2, "V22FP_control: control byte 0xd >> 6 == 2");
  cov_def(CV_STATUS, "V22_status"); cov_def(CV_GETTERS, "getters"); cov_def(CV_TIMER, "ReadGTimer"); cov_def(CV_SQ, "GetSignalQuality"); cov_def(CV_RESETRX, "ResetRx");
  cov_def(CV_MAKETX0, "MakeTxData 0"); cov_def(CV_MAKETX1, "MakeTxData 1"); cov_def(CV_MAKETX2, "MakeTxData 2"); cov_def(CV_MAKETX3, "MakeTxData 3"); cov_def(CV_MAKETX4, "MakeTxData 4");
  cov_def(CV_MAKETX_OTHER, "MakeTxData other pattern"); cov_def(CV_MAKETX_WRAP, "MakeTxData with a wrapping 16-bit count");
  cov_def(CV_RT12_TRUE, "RxTrained1200 true"); cov_def(CV_RT12_FALSE, "RxTrained1200 false"); cov_def(CV_RT24_TRUE, "RxTrained2400 true"); cov_def(CV_RT24_FALSE, "RxTrained2400 false");
  cov_def(CV_TXNOP, "TxNOP"); cov_def(CV_CLAMP, "RxClampV22"); cov_def(CV_DIAG, "V22FP_GetDiagnostics"); cov_def(CV_SCR, "ScrambleDataV22"); cov_def(CV_DESCR, "DescrambleDataV22");
  cov_def(CV_MOD, "ModDataV22");
  arena_init();
  for (t = 0; t < 4; t++) {
    V22Engine *e;
    int cfg[7];
    make_cfg(cfg, t * 5 + 2);
    cfg[0] = t & 1;
    dsplibs_debug_level = 0;
    e = V22FP_create(NULL, cfg);
    if (t == 0) { /* MakeTxData swept over every pattern and the counts around 0 .. 12 and 160 (even counts only for pattern 0: an odd count never reaches 0) */
      static short mb[70000 + 16];
      static const short mc[] = {0, 1, 2, 3, 4, 5, 6, 7, 11, 12, 40, 160};
      int pat, ci;
      for (pat = -1; pat <= 5; pat++)
        for (ci = 0; ci < 12; ci++) {
          short cnt = mc[ci];
          if (pat == 0 && (cnt & 1)) continue;
          for (i = 0; i < 70000 + 16; i++) mb[i] = (short)0x1234;
          MakeTxData(mb, &cnt, (short)pat);
          cov_hit(pat >= 0 && pat <= 4 ? CV_MAKETX0 + pat : CV_MAKETX_OTHER);
          emit(0, "MakeTxData sweep pat=%d cnt=%d -> %016llx", pat, (int)cnt, fnv_shorts(mb, 400));
        }
      for (pat = 0; pat <= 4; pat++) { /* a 16-bit count that wraps (-2 for pattern 0, -1 for the others): 65,535 / 65,536 words are written */
        short cnt = (short)(pat == 0 ? -2 : -1);
        for (i = 0; i < 70000 + 16; i++) mb[i] = (short)0x1234;
        MakeTxData(mb, &cnt, (short)pat);
        cov_hit(CV_MAKETX_WRAP);
        emit(0, "MakeTxData wrap pat=%d -> %016llx", pat, fnv_shorts(mb, 70000 + 16));
      }
    }
    if (t == 0) { /* RxTrained1200 / 2400 swept over every small count, seven data kinds and both values of the words around the data */
      static short dd[8 + 40 + 8];
      short *d = dd + 8;
      int c, kind, sent;
      for (c = 0; c <= 14; c++)
        for (kind = 0; kind < 7; kind++)
          for (sent = 0; sent < 2; sent++) {
            unsigned short cnt = (unsigned short)c;
            int r1, r2;
            for (i = 0; i < 8; i++) dd[i] = dd[8 + 40 + i] = (short)(sent ? 0xf : 3);
            for (i = 0; i < 40; i++) d[i] = (short)(kind == 0 ? 3 : kind == 1 ? 0xf : 5);
            if (kind >= 4) for (i = 0; i < kind - 3; i++) d[i] = 3; /* a run of 1 .. 3 words of 3 at the start */
            if (kind == 2 && c >= 7) for (i = 0; i < 7; i++) d[c - 1 - i] = 0xf;
            if (kind == 3 && c >= 8) for (i = 0; i < 8; i++) d[c - 1 - i] = 0xf;
            if (kind == 3 && c >= 9) d[c - 9] = 3;
            r1 = (int)RxTrained1200(d, &cnt);
            r2 = (int)RxTrained2400(d, &cnt);
            cov_hit(r1 ? CV_RT12_TRUE : CV_RT12_FALSE);
            cov_hit(r2 ? CV_RT24_TRUE : CV_RT24_FALSE);
            emit(0, "RxTrained sweep c=%d kind=%d sent=%d -> %d %d", c, kind, sent, r1, r2);
          }
    }
    for (it = 0; it < 260; it++) {
      char tag[32];
      snprintf(tag, sizeof tag, "S%d.%d", t, it);
      poke_core(e);
      d_engine(0, tag, e);
      switch (it % 13) {
        case 0: {
          int rep;
          for (rep = 0; rep < 2; rep++) { /* twice, over differently poked state: every field the rate sets is seen to change */
            short r = rates[(unsigned)(it / 13 + rep) % 8u];
            if (rep) { poke_core(e); d_engine(0, "pre", e); }
            SetTxRate(e, r);
            cov_hit(r == 0 ? CV_TXR0 : r == 1 ? CV_TXR1 : CV_TXR_OTHER);
            emit(0, "SetTxRate %d", r);
            d_engine(0, "mid", e);
          }
          break;
        }
        case 1: {
          int rep;
          for (rep = 0; rep < 2; rep++) {
            short r = rates[(unsigned)(it / 13 + rep) % 8u];
            if (rep) { poke_core(e); d_engine(0, "pre", e); }
            SetRxRate(e, r);
            cov_hit(r == 0 ? CV_RXR0 : r == 1 ? CV_RXR1 : CV_RXR_OTHER);
            emit(0, "SetRxRate %d", r);
            d_engine(0, "mid", e);
          }
          break;
        }
        case 2: { unsigned short m = aeq[rmod(8)]; SetAdaptEqV22(e, m); cov_hit(m == 1 ? CV_ADEQ1 : m == 2 ? CV_ADEQ2 : m == 3 ? CV_ADEQ3 : CV_ADEQ_OTHER); emit(0, "SetAdaptEq %u", m); break; }
        case 3: TxClockSync(e); cov_hit(CV_CLKSYNC); break;
        case 4: {
          unsigned char ctl[16];
          int r;
          for (i = 0; i < 16; i++) ctl[i] = (unsigned char)rnd();
          if (rpct(30)) ctl[13] = (unsigned char)((ctl[13] & 0x3f) | 0x80);
          if (rpct(30)) ctl[13] |= 4;
          r = V22FP_control(e, ctl);
          cov_hit(CV_CTL);
          if (ctl[13] & 4) cov_hit(CV_CTL_D4);
          if ((ctl[13] >> 6) == 2) cov_hit(CV_CTL_D2);
          emit(0, "control -> %d", r);
          break;
        }
        case 5: {
          short st[24];
          int r;
          for (i = 0; i < 24; i++) st[i] = (short)rnd();
          r = V22_status(e, st);
          cov_hit(CV_STATUS);
          emit(0, "status -> %d %016llx w=%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x", r, fnv_shorts(st, 24), (unsigned short)st[0], (unsigned short)st[1], (unsigned short)st[2],
               (unsigned short)st[3], (unsigned short)st[4], (unsigned short)st[5], (unsigned short)st[6], (unsigned short)st[7], (unsigned short)st[8], (unsigned short)st[9],
               (unsigned short)st[10], (unsigned short)st[11]);
          break;
        }
        case 6: {
          int tm = rpct(20) ? (int)(0x7fffffff - (int)rmod(40)) : (int)rnd();
          long long g1, g2;
          e->tones->timer = tm;
          g1 = ReadGTimer(e);
          g2 = ReadGTimer(e);
          cov_hit(CV_TIMER);
          emit(0, "timer %lld %lld", g1, g2);
          break;
        }
        case 7:
          emit(0, "get %d %d %u %d %d %u", CarrierDetect(e), SignalDetect(e), GetSignalQuality(e), ScramblerOn(e), DescramblerOn(e), GetSignalQuality(e));
          cov_hit(CV_GETTERS);
          cov_hit(CV_SQ);
          break;
        case 8: ResetRx(e); cov_hit(CV_RESETRX); break;
        case 9: {
          short pat = (short)(rpct(80) ? (int)rmod(5) : (rpct(50) ? 5 : -1)), cnt;
          int i2;
          static short buf[70000 + 16];
          static const short cnts[] = {0, 2, 4, 6, 40, 160, 12, 1, 3, 5, 7, 9, 0, 0};
          int wrap = rpct(4);
          cnt = wrap ? (pat == 0 ? (short)-2 : (short)-1) : (pat == 0 ? cnts[rmod(6)] : cnts[rmod(12)]);
          for (i2 = 0; i2 < 70000 + 16; i2++) buf[i2] = (short)0x1234;
          MakeTxData(buf, &cnt, pat);
          cov_hit(pat >= 0 && pat <= 4 ? CV_MAKETX0 + pat : CV_MAKETX_OTHER);
          if (wrap) cov_hit(CV_MAKETX_WRAP);
          emit(0, "MakeTxData pat=%d cnt=%d -> %016llx", pat, cnt, fnv_shorts(buf, 70000 + 16));
          break;
        }
        case 10: {
          static short dd[8 + 400 + 8];
          short *d = dd + 8;
          unsigned short cnt = (unsigned short)(rpct(25) ? rmod(4) : rpct(30) ? 6 + rmod(5) : rmod(300));
          int kind = (int)rmod(5), r1, r2;
          for (i = 0; i < 8; i++) dd[i] = dd[8 + 400 + i] = (short)(rpct(50) ? 0xf : 3); /* the words around the data: the loops must not look at them */
          for (i = 0; i < 400; i++) d[i] = (short)(kind == 0 ? 3 : kind == 1 ? 0xf : (int)rmod(4) == 0 ? 3 : (kind == 2 ? 0xf : (int)rnd() & 0xf));
          if (cnt > 0 && cnt < 12 && kind == 1) for (i = 0; i < (int)cnt; i++) d[i] = 0xf;
          if (kind == 3 && cnt > 12) for (i = 0; i < 12; i++) d[(int)cnt - 1 - i] = 0xf;
          if (kind == 4 && cnt > 7) for (i = 0; i < 7; i++) d[(int)cnt - 1 - i] = 0xf;
          if (kind == 4 && cnt > 9) d[(int)cnt - 8 - rmod(2)] = 0xf;
          if (rpct(30) && cnt > 3) d[rmod(cnt)] = 5;
          r1 = (int)RxTrained1200(d, &cnt);
          r2 = (int)RxTrained2400(d, &cnt);
          cov_hit(r1 ? CV_RT12_TRUE : CV_RT12_FALSE);
          cov_hit(r2 ? CV_RT24_TRUE : CV_RT24_FALSE);
          emit(0, "RxTrained kind=%d cnt=%u -> %d %d", kind, (unsigned)cnt, r1, r2);
          break;
        }
        case 11: {
          short bufs[8 + 200 + 8], *buf = bufs + 8;
          short cnt = 0x5555;
          for (i = 0; i < 8 + 200 + 8; i++) bufs[i] = (short)0x4321;
          if (rpct(50)) { TxNOP(e, (void *)buf, buf, &cnt); cov_hit(CV_TXNOP); }
          else { RxClampV22(e, (void *)buf, buf, &cnt); cov_hit(CV_CLAMP); }
          emit(0, "nop/clamp cnt=%d %016llx", cnt, fnv_shorts(bufs, 216));
          break;
        }
        default: {
          unsigned short b[400], b2[400];
          int n = (int)rmod(300), r;
          for (i = 0; i < 400; i++) b[i] = (unsigned short)rnd();
          memcpy(b2, b, sizeof b);
          if (it % 13 == 12) {
            int dd[4];
            r = V22FP_GetDiagnostics(e, (int)rnd(), dd, 4);
            cov_hit(CV_DIAG);
            emit(0, "diag -> %d", r);
          }
          ScrambleDataV22(e, b, (unsigned short)n);
          cov_hit(CV_SCR);
          DescrambleDataV22(e, b2, (unsigned short)n);
          cov_hit(CV_DESCR);
          emit(0, "scr n=%d %016llx descr %016llx", n, fnv_shorts((short *)b, 400), fnv_shorts((short *)b2, 400));
          {
            unsigned short mb[64];
            short mo[900];
            unsigned r2;
            int nn = (int)rmod(rpct(10) ? 25 : 13);
            for (i = 0; i < 64; i++) mb[i] = (unsigned short)(rnd() & (rpct(50) ? 0xf : 0xffff));
            for (i = 0; i < 900; i++) mo[i] = 0x2468;
            r2 = ModDataV22(e, mb, mo, (unsigned short)nn);
            cov_hit(CV_MOD);
            emit(0, "mod n=%d -> %u %016llx", nn, r2, fnv_shorts(mo, 900));
          }
        }
      }
      d_engine(0, "after", e);
      if (it % 100 == 0) { /* a fresh state: the next functions work on a real engine, not on poked words */
        V22FP_delete(e);
        trial_end(tag);
        make_cfg(cfg, t * 31 + it);
        e = V22FP_create(NULL, cfg);
      }
    }
    V22FP_delete(e);
    trial_end("small");
  }
  cov_finish();
}

/* ================================================================================================ scenario 3: the signal chain */
static void sc_chain(void)
{
  int t, blk;
  cov_def(CV_DEM, "DemodDataV22 called");
  cov_def(CV_DEM_M0, "receiver mode 0 (IIR + tone demodulator)"); cov_def(CV_DEM_M1, "receiver mode 1 / 2");
  cov_def(CV_DEM_EARLY, "DemodDataV22: below the energy threshold (early return)"); cov_def(CV_DEM_THRU, "DemodDataV22: through AGC / SRE / FSE");
  cov_def(CV_DEM_SYMS, "DemodDataV22: symbols produced"); cov_def(CV_DEM_STATE0, "tones.state 0 during the call"); cov_def(CV_DEM_STATE_NZ, "tones.state != 0 during the call");
  cov_def(CV_DEM_ADAPT, "equaliser adaptation enabled (w10, AGC active)"); cov_def(CV_DEM_NOADAPT, "equaliser adaptation disabled");
  cov_def(CV_DEM_PLLC0, "SRE pllFlagC == 0"); cov_def(CV_DEM_PLLC1, "SRE pllFlagC != 0"); cov_def(CV_DEM_SREMODE0, "SRE mode 0"); cov_def(CV_DEM_SREMODE_NZ, "SRE mode != 0");
  cov_def(CV_DEM_TIE, "receiver threshold at / next to the measured energy"); cov_def(CV_DEM_1200, "chain at 1200 bit/s"); cov_def(CV_DEM_2400, "chain at 2400 bit/s"); cov_def(CV_DEM_SILENCE, "silence"); cov_def(CV_DEM_NOISE, "noise"); cov_def(CV_DEM_SAT, "saturated input");
  cov_def(CV_DEM_CLEAN, "loop-back signal"); cov_def(CV_MOD_OUT, "ModDataV22 produced samples"); cov_def(CV_DEM_CARRIER, "carrier detected by the SRE");
  arena_init();
  for (t = 0; t < 8; t++) {
    V22Engine *tx, *rx, *pz;
    int rate = t & 1, mode = (t >> 1) & 1, cfg[7], ntx = 0, nrx = 0, avail = 0;
    static short q[4000 + 400];
    int bad = 0;
    dsplibs_debug_level = 0;
    make_cfg(cfg, t * 3);
    cfg[0] = 1; cfg[1] = rate ? 0 : 1; cfg[5] = 0; cfg[4] = 700;
    tx = V22FP_create(NULL, cfg);
    cfg[0] = mode; cfg[1] = rate ? 0 : 1;
    rx = V22FP_create(NULL, cfg);
    pz = V22FP_create(NULL, cfg); /* the probe: a second receiver of the same kind that sees the same blocks and always returns early (threshold 32767): it measures the energy the receiver tests */
    SetTxRate(tx, (short)rate);
    SetRxRate(rx, (short)rate);
    SetRxRate(pz, (short)rate);
    SetAdaptEqV22(rx, 3);
    cov_hit(rate ? CV_DEM_2400 : CV_DEM_1200);
    cov_hit(mode ? CV_DEM_M1 : CV_DEM_M0);
    for (blk = 0; blk < 420; blk++) {
      unsigned short bits[40];
      short mo[700];
      short blkbuf[200], dout[64];
      int i, n, kind = (blk / 30) % 8, amp = 0;
      unsigned nsym;
      char tag[48];
      /* transmit 12 symbols (a block of audio is about 160 samples) */
      for (i = 0; i < 40; i++) bits[i] = (unsigned short)(rnd() & (rate ? 0xf : 0x3));
      if (blk < 12) for (i = 0; i < 12; i++) bits[i] = (unsigned short)(rate ? 0xf : 3); /* the all-ones training */
      ScrambleDataV22(tx, bits, 12);
      n = (int)ModDataV22(tx, bits, mo, 12);
      cov_hit(CV_MOD_OUT);
      if (avail + n > 4000) { avail = 0; }
      for (i = 0; i < n; i++) q[avail + i] = mo[i];
      avail += n;
      ntx++;
      if (avail < 160) continue;
      memcpy(blkbuf, q, 160 * sizeof(short));
      memmove(q, q + 160, (size_t)(avail - 160) * sizeof(short));
      avail -= 160;
      switch (kind) {
        case 0: break;
        case 1: for (i = 0; i < 160; i++) blkbuf[i] = (short)(blkbuf[i] / 4); break;
        case 2: for (i = 0; i < 160; i++) blkbuf[i] = (short)(blkbuf[i] / 64 + (int)(rnd() % 7) - 3); break;
        case 3: for (i = 0; i < 160; i++) blkbuf[i] = (short)(blkbuf[i] + (int)(rnd() % 801) - 400); cov_hit(CV_DEM_NOISE); break;
        case 4: for (i = 0; i < 160; i++) blkbuf[i] = 0; cov_hit(CV_DEM_SILENCE); break;
        case 5: for (i = 0; i < 160; i++) { int v = blkbuf[i] * 8; blkbuf[i] = (short)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); } cov_hit(CV_DEM_SAT); break;
        case 6: fill_samples(blkbuf, 160, 0, 0); cov_hit(CV_DEM_NOISE); break;
        default: for (i = 0; i < 160; i++) blkbuf[i] = (short)(blkbuf[i] / 2 + (int)(rnd() % 61) - 30); break;
      }
      if (kind != 3 && kind != 4 && kind != 5 && kind != 6) cov_hit(CV_DEM_CLEAN);
      amp = blkbuf[0];
      (void)amp;
      /* the receive switches */
      if (blk % 41 == 7) rx->core->sre.mode = (short)rmod(3);
      if (blk % 53 == 9) rx->core->sre.pllFlagC = (int)rmod(2);
      if (blk % 29 == 3) rx->tones->state = (short)rmod(7);
      if (blk % 37 == 5) rx->core->sreGate = rpct(30) ? (int)rnd() : (int)rmod(4);
      if (blk % 43 == 11) { rx->core->adaptGate = rpct(30) ? (int)rnd() : (int)rmod(4); rx->core->fseS18Gate = rpct(30) ? (int)rnd() : (int)rmod(4); rx->core->fsePllGate = rpct(30) ? (int)rnd() : (int)rmod(4); }
      if (blk % 47 == 13) rx->core->agcIn.active = rpct(30) ? (int)rnd() : (int)rmod(4);
      if (blk % 19 == 4) rx->core->sre.wantTrack = 1;
      {
        short zbuf[200], zout[64];
        short rz;
        static const int delta[3] = {0, 1, -1};
        memcpy(zbuf, blkbuf, 160 * sizeof(short));
        pz->tones->state = 0;
        pz->cfg.thr = 32767;
        DemodDataV22(pz, zbuf, zout, 160);
        rz = FPM_rms(pz->core->rxBuf, pz->core->rxCount);
        if (blk % 7 == 3) { /* a threshold exactly at / next to the measured energy (the comparison is `threshold > energy`) */
          rx->tones->state = 0;
          rx->cfg.thr = (short)(rz + delta[(blk / 7) % 3]);
          cov_hit(CV_DEM_TIE);
        } else rx->cfg.thr = 150;
      }
      if (blk % 59 == 17) V22FP_control(rx, (unsigned char[16]){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (unsigned char)(rnd() & 0xff), (unsigned char)(rnd() & 0xff), 0, 0});
      if (blk % 61 == 19) ResetRx(rx);
      if (rx->tones->state == 0) cov_hit(CV_DEM_STATE0); else cov_hit(CV_DEM_STATE_NZ);
      if (rx->core->adaptGate && rx->core->agcIn.active) cov_hit(CV_DEM_ADAPT); else cov_hit(CV_DEM_NOADAPT);
      if (rx->core->sre.pllFlagC) cov_hit(CV_DEM_PLLC1); else cov_hit(CV_DEM_PLLC0);
      if (rx->core->sre.mode) cov_hit(CV_DEM_SREMODE_NZ); else cov_hit(CV_DEM_SREMODE0);
      for (i = 0; i < 64; i++) dout[i] = (short)0x2222;
      reg_clear();
      reg_add(blkbuf, sizeof blkbuf, 2, "IN");
      snprintf(tag, sizeof tag, "C%d.%d", t, blk);
      nsym = DemodDataV22(rx, blkbuf, dout, 160);
      cov_hit(CV_DEM);
      if (nsym) cov_hit(CV_DEM_SYMS);
      if (rx->core->sre.wantTrack) cov_hit(CV_DEM_CARRIER);
      {
        unsigned short rb[64];
        for (i = 0; i < 64; i++) rb[i] = (unsigned short)dout[i];
        DescrambleDataV22(rx, rb, (unsigned short)(nsym < 64 ? nsym : 0));
        emit(0, "%s kind=%d n=%d nsym=%u dout=%016llx descr=%016llx buf=%016llx", tag, kind, n, nsym, fnv_shorts(dout, 64), fnv_shorts((short *)rb, 64), fnv_shorts(blkbuf, 160));
      }
      if (nsym) cov_hit(CV_DEM_THRU);
      else cov_hit(CV_DEM_EARLY);
      nrx++;
      d_engine(0, tag, rx);
      if (blk % 100 == 0) d_engine(0, "Ctx", tx);
      if (bad) break;
    }
    emit(1, "chain t=%d rate=%d mode=%d tx blocks %d rx blocks %d", t, rate, mode, ntx, nrx);
    d_engine(1, "Cend rx", rx);
    d_engine(1, "Cend tx", tx);
    V22FP_delete(tx);
    V22FP_delete(rx);
    V22FP_delete(pz);
    trial_end("chain");
  }
  cov_finish();
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  g_verbose = getenv("R02_VERBOSE") != NULL;
  g_lcg = 0x2545f491u + 1000003u * (unsigned)sc;
  if (getenv("R02_SEED")) g_lcg ^= (unsigned)atoi(getenv("R02_SEED"));
  (void)g_a;
  switch (sc) {
    case 0: sc_lifecycle(); break;
    case 1: sc_modem(); break;
    case 2: sc_small(); break;
    case 3: sc_chain(); break;
    default: die3("no such scenario %d", sc);
  }
  return 0;
}
