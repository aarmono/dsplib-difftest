/* R27 harness part 4 (ENG build): the real engines behind the ops tables.  Included by fax_vmi.main.c. */
#ifdef HARNESS_ENG

#include "fax/fax_modem_views.h"
#ifdef TEST_BLOB
extern FaxVmiInitFn init_vmi_data_tx_modem[3];
extern FaxVmiInitFn init_vmi_data_rx_modem[3];
#define fax_init_vmi_data_tx_modem init_vmi_data_tx_modem
#define fax_init_vmi_data_rx_modem init_vmi_data_rx_modem
#endif

#if UINTPTR_MAX != 0xffffffffu
/* x86-64: the harness links every fax engine, so every faxw_* accessor is defined by an engine file */
#endif
#ifndef TEST_BLOB
/* the port runs link src/v32/v32_datapath.c (the ported VTB_decoder behind FAXVTB_decoder); the symbols that file references but this test never reaches are abort stubs */
static void unreachable_stub(const char *n)
{
  fprintf(stderr, "r27: unexpected call of %s\n", n);
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

typedef struct {
  int kind, rate, txid, rxid, tk;
} Pair;
static const Pair g_pairs[] = {
    {0, 0, FAXVMI_ID_V21TX, FAXVMI_ID_V21RX, -1},
    {1, 2400, FAXVMI_ID_V27TX, FAXVMI_ID_V27RX, 0}, {1, 4800, FAXVMI_ID_V27TX, FAXVMI_ID_V27RX, 0},
    {2, 7200, FAXVMI_ID_V29TX, FAXVMI_ID_V29RX, 1}, {2, 9600, FAXVMI_ID_V29TX, FAXVMI_ID_V29RX, 1},
    {3, 7200, FAXVMI_ID_V17TX, FAXVMI_ID_V17RX, 2}, {3, 9600, FAXVMI_ID_V17TX, FAXVMI_ID_V17RX, 2},
    {3, 12000, FAXVMI_ID_V17TX, FAXVMI_ID_V17RX, 2}, {3, 14400, FAXVMI_ID_V17TX, FAXVMI_ID_V17RX, 2},
};
#define NPAIRS ((int)(sizeof g_pairs / sizeof g_pairs[0]))
static unsigned short g_ebuf[4096], g_erbuf[4096];
static short g_esmp[8192];

/* the modem config blocks the init functions make (and the three caller buffers of the V.17 receiver) */
static const void *init_cfg(const Pair *p, int rx, FaxVmiM *tmp, int *ret)
{
  int r;
  memset(tmp, 0xA5, sizeof *tmp);
  if (p->tk < 0) {
    *ret = 0;
    return NULL;
  }
  r = (rx ? fax_init_vmi_data_rx_modem : fax_init_vmi_data_tx_modem)[p->tk](tmp, (unsigned short)p->rate, 0, NULL);
  *ret = r;
  return tmp->u10.cfg;
}
static void free_cfg(const Pair *p, int rx, const void *cfg)
{
  if (!cfg) return;
  if (rx && p->kind == 3) {
    const V17RxCfg *c = (const V17RxCfg *)cfg;
    sysdep_free(c->coefI);
    sysdep_free(c->coefQ);
    sysdep_free(c->freq);
  }
  sysdep_free((void *)cfg);
}
static void dump_cfg_words(const char *tag, const void *cfg)
{
  if (!cfg) {
    emit(0, "%s cfg NULL", tag);
    return;
  }
  emit(0, "%s cfg words %016llx %08x %08x", tag, fnv_ints((const int *)cfg, 6), ((const unsigned *)cfg)[1], ((const unsigned *)cfg)[2]);
}

/* the V.17 receiver's hdx.s20 flag is heap garbage in the blob (R26): defined here on both sides right after every create */
static void fix_s20(FaxVmi *o)
{
  if (o->cfg.id == FAXVMI_ID_V17RX && o->m->eng) ((V17Rx *)o->m->eng)->hdx->s20 = 0;
}

static void loop_trial(const Pair *p, int mode, int flag, int ncalls)
{
  FaxVmiM tm1, tm2;
  int r1, r2, call;
  const void *txcfg = init_cfg(p, 0, &tm1, &r1);
  const void *rxcfg = init_cfg(p, 1, &tm2, &r2);
  FaxVmi *tx, *rx;
  unsigned total_items = 0, total_smp = 0;
  emit(0, "LOOP kind %d rate %d mode %d flag %d init %d %d", p->kind, p->rate, mode, flag, r1, r2);
  dump_cfg_words("tx", txcfg);
  dump_cfg_words("rx", rxcfg);
  tx = vmi_new(mkcfg(mode, flag, 60, 30, 100, p->txid, txcfg), 0);
  rx = vmi_new(mkcfg(mode, flag, 60, 30, 100, p->rxid, rxcfg), 0);
  fix_s20(rx);
  dump_vmi("tx0", tx);
  dump_vmi("rx0", rx);
  for (call = 0; call < ncalls; call++) {
    unsigned short n, cnt2, rn, rcnt;
    unsigned st1, st2, q, pos = 0;
    for (q = 0; q < 4096; q++) g_ebuf[q] = 0;
    if (mode == 2) {
      unsigned nf = (call % 7 == 1 || call % 11 == 3) ? 1 : 0;
      if (nf) {
        unsigned len = 4 + rmod(10);
        g_ebuf[pos++] = (unsigned short)len;
        for (q = 0; q < len; q++) g_ebuf[pos++] = (unsigned short)(q == 0 ? 0xff : (q == 1 ? 0x03 : rmod(256)));
      }
      n = (unsigned short)nf;
    } else {
      n = (unsigned short)(rpct(40) ? rmod(8) : 0);
      for (q = 0; q < n; q++) g_ebuf[q] = (unsigned short)rmod(256);
    }
    cnt2 = 0;
    st1 = FAXVMI_process(tx, g_ebuf, g_esmp, &n, &cnt2);
    if (cnt2 > 8192) die3("transmit produced %u samples", (unsigned)cnt2);
    total_smp += cnt2;
    emit(0, "TX call %d state=%08x n=%u cnt2=%u smp=%016llx", call, st1, (unsigned)n, (unsigned)cnt2, fnv_shorts(g_esmp, cnt2));
    dump_vmi("tx", tx);
    for (q = 0; q < 4096; q++) g_erbuf[q] = 0;
    rn = 0;
    rcnt = cnt2;
    st2 = FAXVMI_process(rx, g_erbuf, g_esmp, &rn, &rcnt);
    emit(0, "RX call %d state=%08x n=%u cnt=%u buf=%016llx", call, st2, (unsigned)rn, (unsigned)rcnt, fnv_shorts((short *)g_erbuf, 512));
    dump_vmi("rx", rx);
    total_items += rn;
  }
  emit(0, "LOOP done items %u samples %u", total_items, total_smp);
  if (total_smp) cov_hit(CV_E_TXSAMPLES);
  if (total_items) {
    cov_hit(p->kind == 0 ? CV_E_V21 : (p->kind == 1 ? CV_E_V27 : (p->kind == 2 ? CV_E_V29 : CV_E_V17)));
    cov_hit(mode == 2 ? CV_E_RXFRAMES : CV_E_RXBYTES);
  }
  FAXVMI_delete(tx);
  FAXVMI_delete(rx);
  free_cfg(p, 0, txcfg);
  free_cfg(p, 1, rxcfg);
  trial_end("loop");
}

static void scen_loop(void)
{
  int t;
  for (t = 0; t < NPAIRS * 6; t++) {
    const Pair *p = &g_pairs[t % NPAIRS];
    int mode = (t / NPAIRS) % 3, flag = (t / (NPAIRS * 3)) % 2;
    int ncalls = p->kind == 3 ? 160 : (p->kind == 0 ? 90 : 120);
    cov_hit(CV_E_LOOP);
    cov_hit(mode == 0 ? CV_E_M0 : (mode == 1 ? CV_E_M1 : CV_E_M2));
    if (flag) cov_hit(CV_E_FLAG);
    loop_trial(p, mode, flag, ncalls);
  }
}

static void scen_eng_life(void)
{
  int t, k;
  for (t = 0; t < NPAIRS * 2; t++) {
    const Pair *p = &g_pairs[t % NPAIRS];
    int rx = (t / NPAIRS) & 1;
    FaxVmiM tmp;
    int r;
    const void *cfg = init_cfg(p, rx, &tmp, &r);
    FaxVmi *o;
    FaxVmiCtl ctl;
    FaxVmiSts sts;
    int mode = t % 3;
    unsigned q;
    emit(0, "ELIFE trial %d kind %d rate %d rx %d init %d", t, p->kind, p->rate, rx, r);
    if (p->tk >= 0) {
      cov_hit(rx ? CV_E_INITRX : CV_E_INIT);
      emit(0, "init m w0=%d s08=%d s0a=%d s0c=%d s0e=%d one=%d eng=%s", tmp.w0, (int)tmp.s08, (int)tmp.s0a, (int)tmp.s0c, (int)tmp.s0e, tmp.u4.init1 == 1, tmp.eng ? "set" : "NULL");
      dump_cfg_words("e", cfg);
    }
    o = vmi_new(mkcfg(mode, 0, 40, 20, 50, rx ? p->rxid : p->txid, cfg), 0);
    cov_hit(CV_E_LIFE);
    fix_s20(o);
    dump_vmi("new", o);
    for (k = 0; k < 6; k++) {
      const char *msg = FAXVMI_message(o, k);
      char b[48];
      pr_str(msg, b, sizeof b);
      emit(0, "message %d %s", k, b);
      cov_hit(CV_E_MSG);
    }
    for (k = 0; k < 4; k++) {
      unsigned short n = 0, cnt2 = 0;
      unsigned st;
      for (q = 0; q < 64; q++) g_cbuf[q] = (unsigned char)rnd();
      memset(&ctl, 0, sizeof ctl);
      ctl.flushRing = k & 1;
      ctl.breakReq = (int)rmod(5);
      ctl.breakCount = (unsigned short)rmod(3);
      ctl.rxReset = k == 3;
      ctl.newMode = (unsigned short)(k % 3);
      ctl.mctl = g_cbuf;
      g_cbuf[0x0c] = (unsigned char)(rpct(50) ? 0x08 : 0x04);
      g_cbuf[0x0d] = (unsigned char)(k == 2 ? 0x12 : (rpct(50) ? 0x10 : 0x00));
      g_cbuf[4] = g_cbuf[5] = g_cbuf[6] = g_cbuf[7] = 0;
      g_cbuf[8] = g_cbuf[9] = g_cbuf[10] = 0;
      g_cbuf[11] = (unsigned char)rmod(2);
      g_cbuf[0x10] = g_cbuf[0x11] = g_cbuf[0x12] = g_cbuf[0x13] = 0;
      r = FAXVMI_control(o, &ctl);
      cov_hit(CV_E_CTL);
      emit(0, "control r=%d", r);
      memset(&sts, 0, sizeof sts);
      memset(g_sbuf, 0, sizeof g_sbuf);
      sts.msts = g_sbuf;
      r = FAXVMI_status(o, &sts);
      cov_hit(CV_E_ST);
      emit(0, "status r=%d s0=%d w4=%d w8=%d wc=%d w10=%d w14=%d st=%016llx", r, (int)sts.ringFree, sts.ovf, sts.txEmpty, sts.rxOverflow, sts.rxBreak, sts.hskipDone, fnv_bytes(g_sbuf, 0x16));
      for (q = 0; q < 4096; q++) g_ebuf[q] = (unsigned short)(q < 8 ? 0x41 + q : 0);
      if (mode == 2) {
        g_ebuf[0] = 4;
        g_ebuf[1] = 0xff;
        g_ebuf[2] = 3;
        g_ebuf[3] = 0x55;
        g_ebuf[4] = 0xaa;
      }
      n = (unsigned short)(mode == 2 ? 1 : 4);
      cnt2 = rx ? 160 : 0;
      for (q = 0; q < 8192; q++) g_esmp[q] = (short)(rnd() >> 3);
      st = FAXVMI_process(o, g_ebuf, g_esmp, &n, &cnt2);
      emit(0, "process st=%08x n=%u cnt2=%u smp=%016llx", st, (unsigned)n, (unsigned)cnt2, fnv_shorts(g_esmp, cnt2 > 8192 ? 0 : cnt2));
      dump_vmi("e", o);
    }
    FAXVMI_delete(o);
    free_cfg(p, rx, cfg);
    trial_end("elife");
  }
}
#endif
