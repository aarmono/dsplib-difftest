/*
 * R19 differential harness: the fax V.21 channel of dsplibs.o (V21RX_* / V21TX_*, RxHdx*V21 / TxHdx*V21, Rx / TxNextStateV21, DemodDataV21, ModDataV21, TxNoCarrierV21, CarrierDetectV21,
 * GetSNRV21, the v21rx_* / v21tx_* VMI wrappers) vs src/fax/fax_v21.c + tables.  The blob run (-DTEST_BLOB, linked with ../dsplibs.o) and the port run (-m32 -O0 / -O2, x86-64 against
 * the recorded blob output) execute the same code; library test/fax/r19_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation, deterministic heap with block ids /
 * red zones / poisoned frees, callee recorders sysdep_malloc / sysdep_free and dsplibs_debug_printf).  Integer signals only: a quarter-wave sine table and an LCG, no libm.  Nothing
 * printed depends on the host layout: objects are dumped member by member (pointers as table+index / block id, the content of every malloc()ed leaf block as a hash).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 tables: every R19 table dumped, the message tables through v21rx_message / v21tx_message (all indexes incl. the out-of-range 7 / 6: blob returns the CTL word, port NULL);
 *   1 RX lifecycle: V21RX_create (NULL object / caller-supplied with and without hdx / bundle, NULL / custom cfg, channels 0 / 1 / other, debug level 0 and 2), re-create over a live object,
 *     V21RX_delete, forked malloc-failure probes; 2 TX lifecycle (the same for V21TX_*);
 *   3 RX streams: V.21 FSK with HDLC flags and data on both channels, noise, wrong band, carrier loss, bit slip (clock drift), pure noise, silence, TX -> RX loopback, random block sizes,
 *     control / status calls in between;
 *   4 TX streams: bit chunks of 0 .. 9 bits (under-run, over-run), pend / hold control, scale control, status, re-create via control, all states reached through the machine;
 *   5 RX state machine: every handler (Start / Wait / Data / Idle / Error) x every hdx state x carrier / no carrier x hold x flag counters, RxNextStateV21 for state -2 .. 7, DemodDataV21
 *     with fn == Data or not, CarrierDetectV21 / GetSNRV21;
 *   6 TX state machine: every handler (Start / Data / Idle) x every hdx state x FIFO empty / filled x pend x n = 0 .. 9, TxNextStateV21, ModDataV21 / TxNoCarrierV21;
 *   7 VMI wrappers on FaxVmiV21 objects: create / delete / process / status / control / message for both halves, process through the loopback.
 * R19_VERBOSE=1 prints every line (debugging), R19_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r19_common.h"
#include "fax/fax_v21_tables.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
#else
#define SIDE_BLOB 0
#endif

extern unsigned int dsplibs_debug_level;
/* callee recorder: the debug text of the code under test (the build renames the stub's own) */
int dsplibs_debug_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int dsplibs_debug_printf(const char *fmt, ...)
{
  char b[256];
  char *q;
  int n;
  va_list ap;
  va_start(ap, fmt);
  n = vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  for (q = b; *q; q++)
    if (*q == '\n') *q = '|';
  emit(0, "DBG %s", b);
  return n;
}

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_TAB, CV_MSG_RX, CV_MSG_TX, CV_MSG_OOB_RX, CV_MSG_OOB_TX, CV_MSG_NEG,
  CV_L_RX_NEW, CV_L_RX_SUP_EMPTY, CV_L_RX_SUP_LIVE, CV_L_RX_NULLCFG, CV_L_RX_CH0, CV_L_RX_CH1, CV_L_RX_CHX, CV_L_RX_DBG, CV_L_RX_DEL, CV_L_RX_PROBE, CV_L_RX_RECREATE,
  CV_L_TX_NEW, CV_L_TX_SUP_EMPTY, CV_L_TX_SUP_LIVE, CV_L_TX_NULLCFG, CV_L_TX_CH0, CV_L_TX_CH1, CV_L_TX_CHX, CV_L_TX_DBG, CV_L_TX_DEL, CV_L_TX_PROBE, CV_L_TX_RECREATE,
  CV_R_START, CV_R_WAIT, CV_R_DATA, CV_R_IDLE, CV_R_ERROR, CV_R_TO_WAIT, CV_R_TO_DATA, CV_R_TO_IDLE, CV_R_TO_ERROR, CV_R_CARRIER, CV_R_NOCARRIER, CV_R_BITS, CV_R_NOISE, CV_R_SLIP,
  CV_R_LOSS, CV_R_WRONG, CV_R_LOOP, CV_R_CTL_HOLD, CV_R_STATUS, CV_R_CH0, CV_R_CH1, CV_R_MTDZERO, CV_R_BIGN,
  CV_T_START, CV_T_DATA, CV_T_IDLE, CV_T_UNDER, CV_T_OVER, CV_T_PEND, CV_T_NOCAR, CV_T_SCALE, CV_T_STATUS, CV_T_RECREATE, CV_T_FILL, CV_T_N0, CV_T_FULL6, CV_T_CH0, CV_T_CH1, CV_T_CHX,
  CV_SR_H0, CV_SR_H1, CV_SR_H2, CV_SR_H3, CV_SR_H4, CV_SR_DEFAULT, CV_SR_HOLD, CV_SR_FLAGS5, CV_SR_NEXT0, CV_SR_NEXT1, CV_SR_NEXT2, CV_SR_NEXTD, CV_SR_DEMOD_DATA, CV_SR_DEMOD_OTHER,
  CV_SR_SNR, CV_SR_DBG,
  CV_ST_H0, CV_ST_H1, CV_ST_H2, CV_ST_DEFAULT, CV_ST_NEXT0, CV_ST_NEXT1, CV_ST_NEXT2, CV_ST_NEXTD, CV_ST_FIFO0, CV_ST_FIFOF, CV_ST_PEND, CV_ST_MOD, CV_ST_NOCAR, CV_ST_DBG,
  CV_V_RX_CREATE, CV_V_TX_CREATE, CV_V_RX_PROC, CV_V_TX_PROC, CV_V_STATUS, CV_V_CTL, CV_V_DEL, CV_V_CFGNULL, CV_V_CFGSET
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_TAB, "tables dumped");
      cov_def(CV_MSG_RX, "v21rx_message idx 0..6");
      cov_def(CV_MSG_TX, "v21tx_message idx 0..5");
      cov_def(CV_MSG_OOB_RX, "v21rx_message idx 7 (out of table)");
      cov_def(CV_MSG_OOB_TX, "v21tx_message idx 6 (out of table)");
      cov_def(CV_MSG_NEG, "v21*_message idx > limit / negative");
      break;
    case 1:
      cov_def(CV_L_RX_NEW, "V21RX_create NULL object");
      cov_def(CV_L_RX_SUP_EMPTY, "V21RX_create supplied object, hdx NULL / live");
      cov_def(CV_L_RX_SUP_LIVE, "V21RX_create supplied live object (re-create)");
      cov_def(CV_L_RX_NULLCFG, "V21RX_create NULL cfg");
      cov_def(CV_L_RX_CH0, "V21RX_create channel 0");
      cov_def(CV_L_RX_CH1, "V21RX_create channel 1");
      cov_def(CV_L_RX_CHX, "V21RX_create channel other");
      cov_def(CV_L_RX_DBG, "V21RX_create debug level 2");
      cov_def(CV_L_RX_DEL, "V21RX_delete");
      cov_def(CV_L_RX_PROBE, "RX create malloc-failure probe");
      cov_def(CV_L_RX_RECREATE, "V21RX_create(o, o)");
      break;
    case 2:
      cov_def(CV_L_TX_NEW, "V21TX_create NULL object");
      cov_def(CV_L_TX_SUP_EMPTY, "V21TX_create supplied object, hdx NULL / live");
      cov_def(CV_L_TX_SUP_LIVE, "V21TX_create supplied live object (re-create)");
      cov_def(CV_L_TX_NULLCFG, "V21TX_create NULL cfg");
      cov_def(CV_L_TX_CH0, "V21TX_create channel 0");
      cov_def(CV_L_TX_CH1, "V21TX_create channel 1");
      cov_def(CV_L_TX_CHX, "V21TX_create channel other (error flags)");
      cov_def(CV_L_TX_DBG, "V21TX_create debug level 2");
      cov_def(CV_L_TX_DEL, "V21TX_delete");
      cov_def(CV_L_TX_PROBE, "TX create malloc-failure probe");
      cov_def(CV_L_TX_RECREATE, "V21TX_create(o, o)");
      break;
    case 3:
      cov_def(CV_R_START, "RX state Start observed");
      cov_def(CV_R_WAIT, "RX state Wait observed");
      cov_def(CV_R_DATA, "RX state Data observed");
      cov_def(CV_R_IDLE, "RX state Idle observed");
      cov_def(CV_R_ERROR, "RX state Error observed");
      cov_def(CV_R_TO_WAIT, "RX Start -> Wait");
      cov_def(CV_R_TO_DATA, "RX Wait -> Data");
      cov_def(CV_R_TO_IDLE, "RX Data -> Idle");
      cov_def(CV_R_TO_ERROR, "RX Wait -> Error");
      cov_def(CV_R_CARRIER, "RX carrier flag seen");
      cov_def(CV_R_NOCARRIER, "RX no carrier in a block");
      cov_def(CV_R_BITS, "RX demodulated bits returned");
      cov_def(CV_R_NOISE, "RX noisy stream");
      cov_def(CV_R_SLIP, "RX clock drift stream");
      cov_def(CV_R_LOSS, "RX carrier loss stream");
      cov_def(CV_R_WRONG, "RX wrong band stream");
      cov_def(CV_R_LOOP, "TX -> RX loopback stream");
      cov_def(CV_R_CTL_HOLD, "RX control hold");
      cov_def(CV_R_STATUS, "RX status");
      cov_def(CV_R_CH0, "RX channel 0 streams");
      cov_def(CV_R_CH1, "RX channel 1 streams");
      cov_def(CV_R_MTDZERO, "RX MTD fired");
      cov_def(CV_R_BIGN, "RX block of 128 samples");
      break;
    case 4:
      cov_def(CV_T_START, "TX state Start observed");
      cov_def(CV_T_DATA, "TX state Data observed");
      cov_def(CV_T_IDLE, "TX state Idle observed");
      cov_def(CV_T_UNDER, "TX under-run");
      cov_def(CV_T_OVER, "TX over-run (FIFO full)");
      cov_def(CV_T_PEND, "TX pend control");
      cov_def(CV_T_NOCAR, "TX idle / no carrier");
      cov_def(CV_T_SCALE, "TX scale control");
      cov_def(CV_T_STATUS, "TX status");
      cov_def(CV_T_RECREATE, "TX control re-create");
      cov_def(CV_T_FILL, "TX 6 bits queued");
      cov_def(CV_T_N0, "TX call with n = 0");
      cov_def(CV_T_FULL6, "TX call with n = 6");
      cov_def(CV_T_CH0, "TX channel 0");
      cov_def(CV_T_CH1, "TX channel 1");
      cov_def(CV_T_CHX, "TX channel other");
      break;
    case 5:
      cov_def(CV_SR_H0, "RxHdxStartV21 called");
      cov_def(CV_SR_H1, "RxHdxWaitV21 called");
      cov_def(CV_SR_H2, "RxHdxDataV21 called");
      cov_def(CV_SR_H3, "RxHdxIdleV21 called");
      cov_def(CV_SR_H4, "RxHdxErrorV21 called");
      cov_def(CV_SR_DEFAULT, "state number outside 0..2");
      cov_def(CV_SR_HOLD, "Data with hold");
      cov_def(CV_SR_FLAGS5, "Start with > 4 flags");
      cov_def(CV_SR_NEXT0, "RxNextStateV21 state 0");
      cov_def(CV_SR_NEXT1, "RxNextStateV21 state 1");
      cov_def(CV_SR_NEXT2, "RxNextStateV21 state 2");
      cov_def(CV_SR_NEXTD, "RxNextStateV21 default");
      cov_def(CV_SR_DEMOD_DATA, "DemodDataV21 with fn == Data");
      cov_def(CV_SR_DEMOD_OTHER, "DemodDataV21 with fn != Data");
      cov_def(CV_SR_SNR, "GetSNRV21");
      cov_def(CV_SR_DBG, "debug strings");
      break;
    case 6:
      cov_def(CV_ST_H0, "TxHdxStartV21 called");
      cov_def(CV_ST_H1, "TxHdxDataV21 called");
      cov_def(CV_ST_H2, "TxHdxIdleV21 called");
      cov_def(CV_ST_DEFAULT, "state number outside 0..2");
      cov_def(CV_ST_NEXT0, "TxNextStateV21 state 0");
      cov_def(CV_ST_NEXT1, "TxNextStateV21 state 1");
      cov_def(CV_ST_NEXT2, "TxNextStateV21 state 2");
      cov_def(CV_ST_NEXTD, "TxNextStateV21 default");
      cov_def(CV_ST_FIFO0, "FIFO empty");
      cov_def(CV_ST_FIFOF, "FIFO filled");
      cov_def(CV_ST_PEND, "pend set");
      cov_def(CV_ST_MOD, "ModDataV21");
      cov_def(CV_ST_NOCAR, "TxNoCarrierV21");
      cov_def(CV_ST_DBG, "debug strings");
      break;
    case 7:
      cov_def(CV_V_RX_CREATE, "v21rx_create");
      cov_def(CV_V_TX_CREATE, "v21tx_create");
      cov_def(CV_V_RX_PROC, "v21rx_process");
      cov_def(CV_V_TX_PROC, "v21tx_process");
      cov_def(CV_V_STATUS, "v21*_status");
      cov_def(CV_V_CTL, "v21*_control");
      cov_def(CV_V_DEL, "v21*_delete");
      cov_def(CV_V_CFGNULL, "v21*_create NULL cfg");
      cov_def(CV_V_CFGSET, "v21*_create cfg");
      break;
  }
}

/* ---------------------------------------------------------------------------------------------- dumps */
static void hblk(char *buf, size_t cap, const void *p)
{
  Blk *b;
  if (!p) {
    snprintf(buf, cap, "NULL");
    return;
  }
  b = find_blk(p);
  if (!b) {
    snprintf(buf, cap, "?");
    return;
  }
  snprintf(buf, cap, "blk%u/%u/%016llx%s", b->id, b->size, fnv_bytes(b->p, b->size), p == (const void *)b->p ? "" : "+");
}
static unsigned u(int v) { return (unsigned)v; }
static void d_agc(const char *t, const FpmAgc *a)
{
  char p1[40], p2[40];
  emit(0, "%s AGC a=%08x %08x %08x alpha=%s beta=%s w5=%08x act=%d det=%d hist=%d s22=%d rcp=%d sh=%d frz=%d", t, u(a->a.w[0]), u(a->a.w[1]), u(a->a.w[2]), dptr(a->alpha, p1, 40),
       dptr(a->beta, p2, 40), u(a->w5), a->active, a->detect, a->hist, a->s22, a->rcp, a->shift, a->freeze);
}
static void d_mrf(const char *t, const FpmMrf *m)
{
  char p1[40], p2[80];
  emit(0, "%s MRF a=%08x coefs=%s b=%08x %08x phase=%d frac=%d wpos=%d buflen=%d buf=%s", t, u(m->a.w), dptr(m->coefs, p1, 40), u(m->b.w[0]), u(m->b.w[1]), m->phase, m->frac, m->wpos,
       m->buflen, (hblk(p2, 80, m->buf), p2));
}
static void d_fsd(const char *t, const FpmFsd *f)
{
  char p1[40], p2[40], p3[80], p4[80], p5[80];
  emit(0, "%s FSD intrp=%s c1=%08x coefs=%s c3=%08x c4=%08x c5=%08x w6=%08x out=%s s20=%d s22=%d hist=%s s28=%d state=%s s30=%d s32=%d s34=%d", t, dptr(f->intrp, p1, 40),
       u(f->c1.w), dptr(f->coefs, p2, 40), u(f->c3.w), u(f->c4.w), u(f->c5.w), u(f->w6), (hblk(p3, 80, f->out), p3), f->outCount, f->s22, (hblk(p4, 80, f->hist), p4), f->histPos,
       (hblk(p5, 80, f->state), p5), f->prevBit, f->bitClock, f->mismatchRun);
}
static void d_mtd(const char *t, const FpmMtd *m)
{
  char p1[40], p2[80];
  if (!m) {
    emit(0, "%s MTD NULL", t);
    return;
  }
  emit(0, "%s MTD coef=%s c1=%08x c2=%08x buf=%s dc=%d,%d e14=%d e16=%d", t, dptr(m->coef, p1, 40), u(m->c1.w), u(m->c2.w), (hblk(p2, 80, m->buf), p2), m->dc[0], m->dc[1], m->diffEnergy, m->inEnergy);
}
/* r1.w[5]: its high half (the blob's bytes 0x2a / 0x2b of the tone object) is padding the blob never writes; its garbage depends on the object layout, not printed */
static void d_tone(const char *t, const FpmTone *o)
{
  char p1[80], p2[80], p3[80], p4[80], p5[40];
  if (!o) {
    emit(0, "%s TONE NULL", t);
    return;
  }
  emit(0, "%s TONE r0=%08x %08x %08x %08x lpf=%s r1=%08x %08x %08x %08x %08x %08x lpfOut=%s hist=%s r2=%016llx kfilt=%s kstate=%s kill=%d r3=%d,%d,%d,%d", t, u(o->r0.w[0]), u(o->r0.w[1]), u(o->r0.w[2]),
       u(o->r0.w[3]), dptr(o->lpf, p5, 40), u(o->r1.w[0]), u(o->r1.w[1]), u(o->r1.w[2]), u(o->r1.w[3]), u(o->r1.w[4]), u(o->r1.w[5] & 0xffff), (hblk(p1, 80, o->lpfOut), p1),
       (hblk(p2, 80, o->hist), p2), fnv_shorts(o->r2, 0x60), (hblk(p3, 80, o->kfilt), p3), (hblk(p4, 80, o->kstate), p4), o->killCoef ? (int)(o->killCoef - o->r2) : -999, o->r3[0], o->r3[1],
       o->r3[2], o->r3[3]);
}
static void d_fsm(const char *t, const FpmFsm *f)
{
  emit(0, "%s FSM c=%08x %08x f=%d,%d", t, u(f->c.w[0]), u(f->c.w[1]), f->f[0], f->f[1]);
  d_tone(t, f->tone);
}
static void d_fifo(const char *t, const FaxFifo *f)
{
  char p[80];
  if (!f) {
    emit(0, "%s FIFO NULL", t);
    return;
  }
  emit(0, "%s FIFO w0=%d size=%d fill=%d count=%d rd=%d wr=%d buf=%s", t, f->w0, f->size, f->fill, f->count, f->rd, f->wr, (hblk(p, 80, f->buf), p));
}
static const char *rxfn(V21RxHdxFn f)
{
  if (f == RxHdxStartV21) return "Start";
  if (f == RxHdxWaitV21) return "Wait";
  if (f == RxHdxDataV21) return "Data";
  if (f == RxHdxIdleV21) return "Idle";
  if (f == RxHdxErrorV21) return "Error";
  return f ? "other" : "NULL";
}
static const char *txfn(V21TxHdxFn f)
{
  if (f == TxHdxStartV21) return "Start";
  if (f == TxHdxDataV21) return "Data";
  if (f == TxHdxIdleV21) return "Idle";
  return f ? "other" : "NULL";
}
static void d_rx(const char *t, const V21Rx *o)
{
  char p1[80], p2[80], p3[80];
  emit(0, "%s RX cfg=%08x %08x %08x %08x %08x %08x st=%08x fsdOut=%s s20ok=%d w20=%d w28=%d w2c=%d s30=%d w34=%d w38=%d s3c=%d w40=%d w44=%d s48=%d", t, u(o->cfg.w[0]), u(o->cfg.w[1]),
       u(o->cfg.w[2]), u(o->cfg.w[3]), u(o->cfg.w[4]), u(o->cfg.w[5]), u(o->st.w), (hblk(p1, 80, o->fsdOut), p1), o->bundle ? o->fsdS20 == &o->bundle->fsd.outCount : -1, o->w20, o->w28, o->w2c,
       o->s30, o->w34, o->w38, o->s3c, o->w40, o->w44, o->s48);
  if (o->hdx) emit(0, "%s RXH hold=%d fn=%s state=%d wait=%d ones=%d flags=%d", t, o->hdx->hold, rxfn(o->hdx->fn), o->hdx->state, o->hdx->wait, o->hdx->ones, o->hdx->flags);
  if (o->bundle) {
    const V21RxBundle *b = o->bundle;
    emit(0, "%s RXB w0=%d w4=%d w8=%d buf=%s", t, b->w0, b->agcDetect, b->noTone, (hblk(p2, 80, b->buf), p2));
    d_agc(t, &b->agc);
    d_mrf(t, &b->mrf);
    d_fsd(t, &b->fsd);
    d_mtd(t, b->mtd);
  }
  (void)p3;
}
static void d_tx(const char *t, const V21Tx *o)
{
  char p2[80];
  emit(0, "%s TX cfg=%08x %08x %08x %08x %08x %08x w18=%08x st=%08x", t, u(o->cfg.w[0]), u(o->cfg.w[1]), u(o->cfg.w[2]), u(o->cfg.w[3]), u(o->cfg.w[4]), u(o->cfg.w[5]), u(o->w18),
       u(o->st.w));
  if (o->hdx) {
    emit(0, "%s TXH pend=%d fn=%s state=%d s0e=%d", t, o->hdx->pend, txfn(o->hdx->fn), o->hdx->state, o->hdx->s0e);
    d_fifo(t, o->hdx->fifo);
  }
  if (o->bundle) {
    d_fsm(t, &o->bundle->fsm);
    d_mrf(t, &o->bundle->mrf);
    emit(0, "%s TXB buf=%s", t, (hblk(p2, 80, o->bundle->buf), p2));
  }
}
static void register_tables(void)
{
  htab_add("V21_MRF_FILT", V21_MRF_FILT, sizeof(short) * 360);
  htab_add("V21RX_IIR_LPF", V21RX_IIR_LPF, sizeof(short) * 15);
  htab_add("V21RX_CHAN1_INTRP", V21RX_CHAN1_INTRP, sizeof(short) * 15);
  htab_add("V21RX_CHAN2_INTRP", V21RX_CHAN2_INTRP, sizeof(short) * 15);
  htab_add("AGC_DEF_ALPHA_v21", AGC_DEF_ALPHA_v21, sizeof(short) * 2);
  htab_add("AGC_DEF_BETA_v21", AGC_DEF_BETA_v21, sizeof(short) * 2);
  htab_add("V21_CHAN1_MTD_COEFF", V21_CHAN1_MTD_COEFF, sizeof(short) * 10);
  htab_add("V21_CHAN2_MTD_COEFF", V21_CHAN2_MTD_COEFF, sizeof(short) * 10);
}
static void dbg_level(unsigned v) { dsplibs_debug_level = v; }
/* fill the stack below the caller with a pattern, so that a local the code under test forgets to initialise does not read the zeros of a fresh stack */
static void __attribute__((noinline)) stack_poison(void)
{
  volatile unsigned char b[2048];
  unsigned i;
  for (i = 0; i < sizeof b; i++) b[i] = (unsigned char)(0xa5u ^ (i * 31u));
}

/* ---------------------------------------------------------------------------------------------- integer FSK source */
static const short g_sin[257] = {
  0, 201, 402, 603, 804, 1005, 1206, 1407, 1608, 1809, 2009, 2210, 2410, 2611, 2811, 3012,
  3212, 3412, 3612, 3811, 4011, 4210, 4410, 4609, 4808, 5007, 5205, 5404, 5602, 5800, 5998, 6195,
  6393, 6590, 6786, 6983, 7179, 7375, 7571, 7767, 7962, 8157, 8351, 8545, 8739, 8932, 9126, 9319,
  9511, 9703, 9895, 10087, 10278, 10469, 10659, 10849, 11038, 11227, 11416, 11604, 11792, 11980, 12166, 12353,
  12539, 12724, 12909, 13094, 13278, 13462, 13645, 13827, 14009, 14191, 14372, 14552, 14732, 14911, 15090, 15268,
  15446, 15623, 15799, 15975, 16150, 16325, 16499, 16672, 16845, 17017, 17189, 17360, 17530, 17699, 17868, 18036,
  18204, 18371, 18537, 18702, 18867, 19031, 19194, 19357, 19519, 19680, 19840, 20000, 20159, 20317, 20474, 20631,
  20787, 20942, 21096, 21249, 21402, 21554, 21705, 21855, 22004, 22153, 22301, 22448, 22594, 22739, 22883, 23027,
  23170, 23311, 23452, 23592, 23731, 23869, 24006, 24143, 24278, 24413, 24546, 24679, 24811, 24942, 25072, 25201,
  25329, 25456, 25582, 25707, 25831, 25954, 26077, 26198, 26318, 26437, 26556, 26673, 26789, 26905, 27019, 27132,
  27244, 27355, 27466, 27575, 27683, 27790, 27896, 28001, 28105, 28208, 28309, 28410, 28510, 28608, 28706, 28802,
  28897, 28992, 29085, 29177, 29268, 29358, 29446, 29534, 29621, 29706, 29790, 29873, 29955, 30036, 30116, 30195,
  30272, 30349, 30424, 30498, 30571, 30643, 30713, 30783, 30851, 30918, 30984, 31049, 31113, 31175, 31236, 31297,
  31356, 31413, 31470, 31525, 31580, 31633, 31684, 31735, 31785, 31833, 31880, 31926, 31970, 32014, 32056, 32097,
  32137, 32176, 32213, 32249, 32284, 32318, 32350, 32382, 32412, 32441, 32468, 32495, 32520, 32544, 32567, 32588,
  32609, 32628, 32646, 32662, 32678, 32692, 32705, 32717, 32727, 32736, 32744, 32751, 32757, 32761, 32764, 32766,
  32767
};
static int isin(unsigned ph)
{
  unsigned idx = ph >> 22, off = idx & 255u;
  switch (idx >> 8) {
    case 0: return g_sin[off];
    case 1: return g_sin[256 - off];
    case 2: return -g_sin[off];
    default: return -g_sin[256 - off];
  }
}
static unsigned hz_inc(int hz) { return (unsigned)((unsigned long long)(unsigned)hz * 536871ull); } /* 2^32 / 8000 */

typedef struct {
  unsigned ph;
  int acc;       /* bit clock accumulator */
  int pos;       /* current bit index */
  int mark, space; /* Hz */
  int amp;       /* peak */
  int noise;     /* uniform +- */
  int drift;     /* added to 300 per sample (clock error) */
} Fsk;
static void fsk_init(Fsk *g, int ch, int amp, int noise, int drift)
{
  memset(g, 0, sizeof *g);
  /* the receiver of channel 0 decodes the 1180 Hz tone as the one bit (inverted against the standard's 980 Hz mark) */
  g->mark = ch == 0 ? 1180 : 1650;
  g->space = ch == 0 ? 980 : 1850;
  g->amp = amp;
  g->noise = noise;
  g->drift = drift;
}
/* idle: 0 silence after the bits, 1 mark tone */
static void fsk_gen(Fsk *g, const unsigned char *bits, int nbits, short *out, int cnt, int idle)
{
  int i;
  for (i = 0; i < cnt; i++) {
    int v = 0;
    g->acc += 300 + g->drift;
    while (g->acc >= 8000) {
      g->acc -= 8000;
      g->pos++;
    }
    if (g->pos < nbits || idle) {
      int bit = g->pos < nbits ? bits[g->pos] : 1;
      g->ph += hz_inc(bit ? g->mark : g->space);
      v = (int)(((long long)isin(g->ph) * g->amp) >> 15);
    }
    if (g->noise) v += (int)rmod((unsigned)(2 * g->noise + 1)) - g->noise;
    out[i] = (short)v;
  }
}
/* HDLC-like bits: nflags flags, `nbytes` random bytes (bit stuffed, LSB first), nflags flags */
static int build_bits(unsigned char *b, int cap, int nflags, int nbytes)
{
  int n = 0, i, k, ones = 0;
  static const unsigned char flag[8] = {0, 1, 1, 1, 1, 1, 1, 0};
  for (i = 0; i < (int)(rnd() & 15u); i++) b[n++] = 1; /* a short mark lead-in */
  for (i = 0; i < nflags; i++)
    for (k = 0; k < 8 && n < cap; k++) b[n++] = flag[k];
  for (i = 0; i < nbytes; i++) {
    unsigned by = rnd() & 0xffu;
    for (k = 0; k < 8 && n + 2 < cap; k++) {
      int bit = (int)((by >> k) & 1u);
      b[n++] = (unsigned char)bit;
      ones = bit ? ones + 1 : 0;
      if (ones == 5) {
        b[n++] = 0;
        ones = 0;
      }
    }
  }
  for (i = 0; i < nflags; i++)
    for (k = 0; k < 8 && n < cap; k++) b[n++] = flag[k];
  return n;
}

#define MAXS 40000
static short g_src[MAXS + 512];
static short g_inb[1024];
static short g_out[4096];
static unsigned char g_bits[16384];

static unsigned long long fnv_n(const short *p, int n) { return fnv_shorts(p, (unsigned)(n < 0 ? 0 : n)); }

/* the state observation of the RX machine for the coverage counters: (state code, fn) */
static void rx_observe(const V21Rx *o, int *prevState)
{
  const V21RxHdx *h = o->hdx;
  int s = h->state;
  if (h->fn == RxHdxStartV21) cov_hit(CV_R_START);
  if (h->fn == RxHdxWaitV21) cov_hit(CV_R_WAIT);
  if (h->fn == RxHdxDataV21) cov_hit(CV_R_DATA);
  if (h->fn == RxHdxIdleV21) cov_hit(CV_R_IDLE);
  if (h->fn == RxHdxErrorV21) cov_hit(CV_R_ERROR);
  if (*prevState == 0 && s == 1) cov_hit(CV_R_TO_WAIT);
  if (*prevState == 1 && s == 2) cov_hit(CV_R_TO_DATA);
  if (*prevState == 2 && s == 3) cov_hit(CV_R_TO_IDLE);
  if (*prevState == 1 && s == 4) cov_hit(CV_R_TO_ERROR);
  *prevState = s;
}

static void make_cfg_rx(int *w, int ch)
{
  int i;
  for (i = 0; i < 6; i++) w[i] = V21RX_CFG[i];
  w[0] = (w[0] & ~0xffff) | (ch & 0xffff);
}
static void make_cfg_tx(int *w, int ch)
{
  int i;
  for (i = 0; i < 7; i++) w[i] = V21TX_CFG[i];
  w[0] = (w[0] & ~0xffff) | (ch & 0xffff);
}
/* the words of the config the engine copies but never decides on: other values (the blob forwards word 5 / 6 to the MRF / FSD configs, the rest is kept in the instance) */
static void fuzz_cfg(int *w, int n)
{
  int i;
  for (i = 1; i < n; i++)
    if (i != 2) w[i] = (int)(rnd() & 0x7fu) + 1; /* word 2 (60000) is overwritten by the control */
}

/* ============================================================================================ scenario 0: tables */
static void rx_dispatch_tests(void);
static void rx_misc_tests(void);
static void tx_dispatch_tests(void);
static void tx_misc_tests(void);
static void scen0(void)
{
  int i;
  register_tables();
  emit(0, "TAB MRF %016llx", fnv_shorts(V21_MRF_FILT, 360));
  emit(0, "TAB RXIIR %016llx CH1 %016llx CH2 %016llx", fnv_shorts(V21RX_IIR_LPF, 15), fnv_shorts(V21RX_CHAN1_INTRP, 15), fnv_shorts(V21RX_CHAN2_INTRP, 15));
  emit(0, "TAB AGC alpha %016llx beta %016llx", fnv_shorts(AGC_DEF_ALPHA_v21, 2), fnv_shorts(AGC_DEF_BETA_v21, 2));
  emit(0, "TAB MTD1 %016llx MTD2 %016llx", fnv_shorts(V21_CHAN1_MTD_COEFF, 10), fnv_shorts(V21_CHAN2_MTD_COEFF, 10));
  emit(0, "TAB AGCv21_CFG %08x %08x %08x w3=%d w4=%d %08x", u(AGCv21_CFG[0]), u(AGCv21_CFG[1]), u(AGCv21_CFG[2]), AGCv21_CFG[3] != 0, AGCv21_CFG[4] != 0, u(AGCv21_CFG[5]));
  emit(0, "TAB RXCFG %08x %08x %08x %08x %08x %08x", u(V21RX_CFG[0]), u(V21RX_CFG[1]), u(V21RX_CFG[2]), u(V21RX_CFG[3]), u(V21RX_CFG[4]), u(V21RX_CFG[5]));
  emit(0, "TAB RXCTL %08x %08x %08x %08x", u(V21RX_CTL[0]), u(V21RX_CTL[1]), u(V21RX_CTL[2]), u(V21RX_CTL[3]));
  emit(0, "TAB TXCFG %08x %08x %08x %08x %08x %08x %08x", u(V21TX_CFG[0]), u(V21TX_CFG[1]), u(V21TX_CFG[2]), u(V21TX_CFG[3]), u(V21TX_CFG[4]), u(V21TX_CFG[5]), u(V21TX_CFG[6]));
  emit(0, "TAB TXCTL %08x %08x %08x %08x %08x", u(V21TX_CTL[0]), u(V21TX_CTL[1]), u(V21TX_CTL[2]), u(V21TX_CTL[3]), u(V21TX_CTL[4]));
  cov_hit(CV_TAB);
  {
    FaxVmiV21 m;
    memset(&m, 0, sizeof m);
    for (i = -3; i < 12; i++) {
      const char *s = "unset";
      v21rx_message(&m, i, &s);
      if (i == 7) {
        /* the blob reads the first word of V21RX_CTL (0x12c0000), the port stores NULL: one common line, the blob's value is checked */
        if (SIDE_BLOB && (unsigned)(uintptr_t)s != u(V21RX_CTL[0])) die3("blob v21rx_message(7) != V21RX_CTL[0]");
        if (!SIDE_BLOB && s != NULL) die3("port v21rx_message(7) != NULL");
        emit(1, "MSG rx %d = (CTL word 0 in the blob, NULL in the port)", i);
        cov_hit(CV_MSG_OOB_RX);
      } else {
        emit(0, "MSG rx %d = %s", i, s ? s : "NULL");
        if (i >= 0 && i <= 6) cov_hit(CV_MSG_RX); else cov_hit(CV_MSG_NEG);
      }
      s = "unset";
      v21tx_message(&m, i, &s);
      if (i == 6) {
        if (SIDE_BLOB && (unsigned)(uintptr_t)s != u(V21TX_CTL[0])) die3("blob v21tx_message(6) != V21TX_CTL[0]");
        if (!SIDE_BLOB && s != NULL) die3("port v21tx_message(6) != NULL");
        emit(1, "MSG tx %d = (CTL word 0 in the blob, NULL in the port)", i);
        cov_hit(CV_MSG_OOB_TX);
      } else {
        emit(0, "MSG tx %d = %s", i, s ? s : "NULL");
        if (i >= 0 && i <= 5) cov_hit(CV_MSG_TX); else cov_hit(CV_MSG_NEG);
      }
    }
    for (i = 0; i < 8; i++) {
      const char *s = 0;
      v21rx_message(&m, -256 + i, &s); /* a negative index whose low byte is a valid index: NULL (unsigned compare) */
      emit(0, "MSG rx %d = %s", -256 + i, s ? s : "NULL");
      s = 0;
      v21tx_message(&m, -256 + i, &s);
      emit(0, "MSG tx %d = %s", -256 + i, s ? s : "NULL");
    }
    for (i = 0; i < 6; i++) {
      const char *s = 0;
      v21rx_message(&m, 0x100 + i, &s);
      emit(0, "MSG rx %d = %s", 0x100 + i, s ? s : "NULL");
      v21tx_message(&m, 0x100 + i, &s);
      emit(0, "MSG tx %d = %s", 0x100 + i, s ? s : "NULL");
    }
  }
}

/* ============================================================================================ scenario 1 / 2: lifecycle */
static void probe_create(int isrx, int failk, int cfgvar)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    int w[7];
    g_fail_malloc = failk;
    if (isrx) {
      make_cfg_rx(w, cfgvar);
      V21RX_create(NULL, w);
    } else {
      make_cfg_tx(w, cfgvar);
      V21TX_create(NULL, w);
    }
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  /* a failed allocation: the blob and the port crash or survive alike; the outcome is one line */
  if (WIFSIGNALED(st)) emit(1, "PROBE %s create malloc #%d fails (cfg %d): signal %d", isrx ? "RX" : "TX", failk, cfgvar, WTERMSIG(st));
  else emit(1, "PROBE %s create malloc #%d fails (cfg %d): exit %d", isrx ? "RX" : "TX", failk, cfgvar, WEXITSTATUS(st));
}
/* a caller-supplied object without a bundle: the bundle is allocated but the FPM init functions are told `fresh = 0`, they use their still unallocated buffers (the blob crashes) */
static void probe_supplied(int isrx)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    if (isrx) {
      V21Rx *o = (V21Rx *)sysdep_malloc(sizeof(V21Rx));
      o->hdx = NULL;
      o->bundle = NULL;
      V21RX_create(o, NULL);
    } else {
      V21Tx *o = (V21Tx *)sysdep_malloc(sizeof(V21Tx));
      o->hdx = NULL;
      o->bundle = NULL;
      V21TX_create(o, NULL);
    }
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE %s create of a supplied object without bundle: signal %d", isrx ? "RX" : "TX", WTERMSIG(st));
  else emit(1, "PROBE %s create of a supplied object without bundle: exit %d", isrx ? "RX" : "TX", WEXITSTATUS(st));
}
static void scen1(void)
{
  int trial, k;
  register_tables();
  for (trial = 0; trial < 120; trial++) {
    int ch = trial % 5 == 0 ? 0 : trial % 5 == 1 ? 1 : trial % 5 == 2 ? -1 : trial % 5 == 3 ? 7 : 0x7fff;
    int mode = (trial / 5) % 6; /* 0 new + cfg, 1 new NULL cfg, 2 supplied empty, 3 supplied live, 4 create(o,o), 5 supplied hdx only */
    int dbg = (trial / 30) & 1;
    int w[7];
    V21Rx *o = NULL, *r;
    make_cfg_rx(w, ch);
    if (trial % 3 == 2) fuzz_cfg(w, 6);
    dbg_level(dbg ? 2 : 0);
    if (ch == 0) cov_hit(CV_L_RX_CH0);
    else if (ch == 1) cov_hit(CV_L_RX_CH1);
    else cov_hit(CV_L_RX_CHX);
    if (dbg) cov_hit(CV_L_RX_DBG);
    switch (mode) {
      case 0:
        stack_poison();
        o = V21RX_create(NULL, w);
        cov_hit(CV_L_RX_NEW);
        break;
      case 1:
        stack_poison();
        o = V21RX_create(NULL, NULL);
        cov_hit(CV_L_RX_NEW);
        cov_hit(CV_L_RX_NULLCFG);
        break;
      case 2:
      case 5: {
        V21Rx *s = V21RX_create(NULL, NULL);
        d_rx("first", s);
        if (mode == 2) {
          sysdep_free(s->hdx);
          s->hdx = NULL;
        }
        stack_poison();
        o = V21RX_create(s, w);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_RX_SUP_EMPTY);
        break;
      }
      case 3: {
        V21Rx *s = V21RX_create(NULL, NULL);
        d_rx("first", s);
        o = V21RX_create(s, w);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_RX_SUP_LIVE);
        break;
      }
      default: {
        V21Rx *s = V21RX_create(NULL, w);
        d_rx("first", s);
        s->cfg.w[2] = 12345; /* the cfg is the object itself */
        o = V21RX_create(s, (const int *)s);
        cov_hit(CV_L_RX_RECREATE);
        break;
      }
    }
    dbg_level(0);
    d_rx("created", o);
    r = o;
    /* poke some state, then delete */
    if (trial & 1) {
      short bb[160];
      unsigned short n = 20;
      memset(bb, 0, sizeof bb);
      (void)V21RX_modem(r, bb, g_out, &n);
      d_rx("ran", r);
    }
    V21RX_delete(o);
    cov_hit(CV_L_RX_DEL);
    trial_end("rx life");
  }
  probe_supplied(1);
  for (k = 0; k < 14; k++) {
    probe_create(1, k, k & 1);
    cov_hit(CV_L_RX_PROBE);
  }
  /* malloc failure injection after a live object (create over it never allocates) */
  {
    V21Rx *s = V21RX_create(NULL, NULL);
    g_fail_malloc = 0;
    V21RX_create(s, NULL);
    d_rx("recreate no malloc", s);
    V21RX_delete(s);
    trial_end("rx fail");
  }
}
static void scen2(void)
{
  int trial, k;
  register_tables();
  for (trial = 0; trial < 120; trial++) {
    int ch = trial % 5 == 0 ? 0 : trial % 5 == 1 ? 1 : trial % 5 == 2 ? -1 : trial % 5 == 3 ? 7 : 0x7fff;
    int mode = (trial / 5) % 6;
    int dbg = (trial / 30) & 1;
    int w[7];
    V21Tx *o = NULL;
    make_cfg_tx(w, ch);
    if (trial % 3 == 2) fuzz_cfg(w, 7);
    dbg_level(dbg ? 2 : 0);
    if (ch == 0) cov_hit(CV_L_TX_CH0);
    else if (ch == 1) cov_hit(CV_L_TX_CH1);
    else cov_hit(CV_L_TX_CHX);
    if (dbg) cov_hit(CV_L_TX_DBG);
    switch (mode) {
      case 0:
        stack_poison();
        o = V21TX_create(NULL, w);
        cov_hit(CV_L_TX_NEW);
        break;
      case 1:
        stack_poison();
        o = V21TX_create(NULL, NULL);
        cov_hit(CV_L_TX_NEW);
        cov_hit(CV_L_TX_NULLCFG);
        break;
      case 2:
      case 5: {
        V21Tx *s = V21TX_create(NULL, NULL);
        d_tx("first", s);
        if (mode == 2) {
          FIFO_delete(s->hdx->fifo);
          sysdep_free(s->hdx);
          s->hdx = NULL;
        }
        stack_poison();
        o = V21TX_create(s, w);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_TX_SUP_EMPTY);
        break;
      }
      case 3: {
        V21Tx *s = V21TX_create(NULL, NULL);
        d_tx("first", s);
        o = V21TX_create(s, w);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_TX_SUP_LIVE);
        break;
      }
      default: {
        V21Tx *s = V21TX_create(NULL, w);
        d_tx("first", s);
        s->cfg.w[2] = 777;
        o = V21TX_create(s, (const int *)s);
        cov_hit(CV_L_TX_RECREATE);
        break;
      }
    }
    dbg_level(0);
    d_tx("created", o);
    if (trial & 1) {
      short bits[8] = {1, 0, 1, 1, 0, 0, 1, 0};
      unsigned short n = 5;
      (void)V21TX_modem(o, bits, g_out, &n);
      d_tx("ran", o);
    }
    V21TX_delete(o);
    cov_hit(CV_L_TX_DEL);
    trial_end("tx life");
  }
  probe_supplied(0);
  for (k = 0; k < 14; k++) {
    probe_create(0, k, k & 1);
    cov_hit(CV_L_TX_PROBE);
  }
  {
    V21Tx *s = V21TX_create(NULL, NULL);
    g_fail_malloc = 0;
    V21TX_create(s, NULL);
    d_tx("recreate no malloc", s);
    V21TX_delete(s);
    trial_end("tx fail");
  }
}

/* ============================================================================================ scenario 3: RX streams */
static void rx_call(V21Rx *o, int nsamp, const short *src, int *prev, unsigned long long *outHash, int *nbitsTot)
{
  unsigned short n = (unsigned short)nsamp;
  int ret, i;
  memcpy(g_inb, src, (size_t)nsamp * 2);
  for (i = 0; i < 8; i++) g_inb[nsamp + i] = 0x1234; /* a write one past the block is seen */
  for (i = 0; i < 4096; i++) g_out[i] = 0x7777;
  ret = V21RX_modem(o, g_inb, g_out, &n);
  emit(0, "rx call n=%d ret=%08x outn=%d in=%016llx out=%016llx", nsamp, u(ret), n, fnv_n(g_inb, nsamp + 4), fnv_n(g_out, (int)n + 12));
  *outHash ^= fnv_n(g_out, (int)n);
  *nbitsTot += n;
  if (n) cov_hit(CV_R_BITS);
  d_rx("rx", o);
  rx_observe(o, prev);
  if (o->bundle->agcDetect & o->bundle->noTone) cov_hit(CV_R_CARRIER); else cov_hit(CV_R_NOCARRIER);
  if (o->bundle->noTone == 0) cov_hit(CV_R_MTDZERO);
  if (nsamp == 128) cov_hit(CV_R_BIGN);
}
static void scen3(void)
{
  int trial;
  register_tables();
  dbg_level(0);
  for (trial = 0; trial < 96; trial++) {
    int ch = trial & 1;
    int kind = (trial >> 1) & 7;
    int w[7], total, i, nbits, pos = 0, prev = 0, nbt = 0;
    unsigned long long oh = 0;
    V21Rx *o;
    V21Tx *tx = NULL;
    Fsk g;
    int amp = 4000 + (int)rmod(12000), noise = 0, drift = 0, silent_from = -1;
    int cfgch;
    if ((trial >> 1 & 7) == 6) ch = 1; /* the loopback receiver is channel 1 (band 1650 / 1850) */
    cfgch = (trial % 11 == 5 && (trial >> 1 & 7) != 6) ? 3 : ch;
    make_cfg_rx(w, cfgch);
    dbg_level(trial % 6 == 1 ? 2 : 0);
    stack_poison();
    o = V21RX_create(NULL, trial % 7 == 3 ? NULL : w);
    d_rx("created", o);
    if (trial % 7 == 3) ch = 1; /* NULL cfg = V21RX_CFG: channel 1 */
    if (kind == 6 && trial % 7 == 3) cfgch = 1;
    if (ch == 0) cov_hit(CV_R_CH0); else cov_hit(CV_R_CH1);
    total = 4000 + (int)rmod(24000);
    if (total > MAXS) total = MAXS;
    nbits = build_bits(g_bits, (int)sizeof g_bits, 8 + (int)rmod(10), 20 + (int)rmod(300));
    fsk_init(&g, ch, amp, 0, 0);
    switch (kind) {
      case 0: break;
      case 1: noise = 300 + (int)rmod(2500); cov_hit(CV_R_NOISE); break;
      case 2: fsk_init(&g, 1 - ch, amp, 0, 0); cov_hit(CV_R_WRONG); break;
      case 3: silent_from = 3000 + (int)rmod(8000); cov_hit(CV_R_LOSS); break;
      case 4: drift = (int)rmod(61) - 30; cov_hit(CV_R_SLIP); break;
      case 5: amp = 0; noise = 200 + (int)rmod(4000); break;
      case 6: break; /* loopback below */
      default: amp = 100 + (int)rmod(900); noise = 50 + (int)rmod(300); break;
    }
    g.amp = amp;
    g.noise = noise;
    g.drift = drift;
    if (kind == 6) {
      /* TX engine (channel 1: the blob transmits the default pair on every channel) -> RX engine channel 1 */
      int tw[7], off = 0, c;
      unsigned short nn;
      short bb[16];
      make_cfg_tx(tw, 1);
      tx = V21TX_create(NULL, tw);
      cov_hit(CV_R_LOOP);
      pos = 0;
      for (total = 0, c = 0; c < nbits && total < MAXS - 400; c += 4) {
        int j, m = nbits - c < 4 ? nbits - c : 4;
        for (j = 0; j < m; j++) bb[j] = g_bits[c + j];
        nn = (unsigned short)m;
        (void)V21TX_modem(tx, bb, &g_src[total], &nn);
        total += nn;
        (void)off;
      }
      emit(0, "loopback tx produced %d samples", total);
      for (i = 0; i < total; i++) {
        g_src[i] = (short)((g_src[i] * 3) / (1 + (trial & 3)));
      }
    } else {
      fsk_gen(&g, g_bits, nbits, g_src, total, kind == 0 ? 0 : 0);
      if (silent_from >= 0)
        for (i = silent_from; i < total; i++) g_src[i] = 0;
    }
    while (pos < total) {
      int nn = trial % 5 == 4 ? 128 : (int)(1 + rmod(128));
      if (pos + nn > total) nn = total - pos;
      rx_call(o, nn, &g_src[pos], &prev, &oh, &nbt);
      pos += nn;
      if ((pos / 977) != ((pos - nn) / 977)) {
        FaxV21Status st;
        FaxV21Ctl ctl;
        memset(&st, 0x55, sizeof st);
        emit(0, "rx status ret=%d", V21RX_status(o, &st));
        emit(0, "rx status %d %d %d %d %d %d %d %d %d %d %02x %02x", st.channel, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15);
        cov_hit(CV_R_STATUS);
        if (trial % 3 != 0 && rpct(10)) {
          memset(&ctl, 0, sizeof ctl);
          ctl.w4 = rpct(50) ? 60000 : (int)rnd();
          ctl.flags = (unsigned char)((rpct(25) ? 0x10 : 0) | (rpct(35) ? 2 : 0));
          {
            int r1 = V21RX_control(o, &ctl);
            int r2 = V21RX_control(o, NULL);
            emit(0, "rx control ret=%d null=%d", r1, r2);
          }
          cov_hit(CV_R_CTL_HOLD);
          d_rx("ctl", o);
        }
      }
    }
    emit(0, "rx trial %d kind %d ch %d bits=%d outhash=%016llx", trial, kind, ch, nbt, oh);
    if (trial % 8 == 0) emit(1, "RX trial %d kind %d ch %d: %d bits out", trial, kind, ch, nbt);
    if (tx) V21TX_delete(tx);
    V21RX_delete(o);
    trial_end("rx run");
  }
}

/* ============================================================================================ scenario 4: TX streams */
static void tx_call(V21Tx *o, int n, int *prev, unsigned long long *oh, int *tot, int pat)
{
  short bits[64];
  unsigned short nn = (unsigned short)n;
  int i, ret;
  for (i = 0; i < 64; i++) bits[i] = (short)(pat == 0 ? (rnd() & 1) : pat == 1 ? 1 : pat == 2 ? 0 : (i & 1));
  for (i = 0; i < 4096; i++) g_out[i] = 0x7777;
  if (o->hdx->fn == TxHdxStartV21) cov_hit(CV_T_START);
  if (o->hdx->fn == TxHdxDataV21) cov_hit(CV_T_DATA);
  if (o->hdx->fn == TxHdxIdleV21) cov_hit(CV_T_IDLE);
  if (n == 0) cov_hit(CV_T_N0);
  if (n == 6) cov_hit(CV_T_FULL6);
  if (n >= 6 && o->hdx->pend == 0 && o->hdx->fifo->count == 0) cov_hit(CV_T_FILL);
  ret = V21TX_modem(o, bits, g_out, &nn);
  emit(0, "tx call n=%d ret=%08x outn=%d bits=%016llx out=%016llx", n, u(ret), nn, fnv_n(bits, 16), fnv_n(g_out, (int)nn + 12));
  *oh ^= fnv_n(g_out, nn);
  *tot += nn;
  d_tx("tx", o);
  if (o->st.b[0] == 4) cov_hit(CV_T_OVER);
  if (n == 0 && (o->st.b[1] & 2) && o->hdx->pend == 0) cov_hit(CV_T_UNDER);
  if (o->hdx->state == 2) cov_hit(CV_T_NOCAR);
  (void)prev;
}
static void scen4(void)
{
  int trial;
  register_tables();
  dbg_level(0);
  for (trial = 0; trial < 60; trial++) {
    int ch = trial % 4 == 0 ? 0 : trial % 4 == 1 ? 1 : trial % 4 == 2 ? 5 : 1;
    int w[7], i, calls, tot = 0, prev = 0;
    unsigned long long oh = 0;
    V21Tx *o;
    make_cfg_tx(w, ch);
    dbg_level(trial % 6 == 1 ? 2 : 0);
    stack_poison();
    o = V21TX_create(NULL, trial % 9 == 4 ? NULL : w);
    if (ch == 0) cov_hit(CV_T_CH0); else if (ch == 1) cov_hit(CV_T_CH1); else cov_hit(CV_T_CHX);
    d_tx("created", o);
    calls = 40 + (int)rmod(160);
    for (i = 0; i < calls; i++) {
      int n = trial % 3 == 0 ? (int)rmod(10) : trial % 3 == 1 ? 6 : (int)(rpct(70) ? 1 + rmod(6) : rmod(10));
      if (trial % 13 == 7 && i > 20 && i < 40) n = 0; /* idle */
      tx_call(o, n, &prev, &oh, &tot, (int)(rnd() & 3));
      if (rpct(8)) {
        FaxV21Status st;
        FaxV21Ctl ctl;
        memset(&st, 0x55, sizeof st);
        V21TX_status(o, &st);
        emit(0, "tx status %d %d %d %d %d %d %d %d %d %d %02x %02x", st.channel, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15);
        cov_hit(CV_T_STATUS);
        memset(&ctl, 0, sizeof ctl);
        ctl.w4 = (int)rnd();
        ctl.txScale = rpct(50) ? 0x4000 : (int)(rnd() & 0xffff);
        ctl.b0c = (unsigned char)(rpct(50) ? 4 : 0);
        ctl.flags = (unsigned char)((rpct(30) ? 0x10 : 0) | (rpct(8) ? 2 : 0));
        if (ctl.flags & 0x10) cov_hit(CV_T_PEND);
        if (ctl.txScale == 0x4000 || 1) cov_hit(CV_T_SCALE);
        if (ctl.flags & 2) cov_hit(CV_T_RECREATE);
        emit(0, "tx control ret=%d", V21TX_control(o, &ctl));
        d_tx("ctl", o);
        emit(0, "tx control null ret=%d", V21TX_control(o, NULL));
        emit(0, "tx status null ret=%d", V21TX_status(o, NULL));
      }
    }
    emit(0, "tx trial %d ch %d samples=%d outhash=%016llx", trial, ch, tot, oh);
    if (trial % 8 == 0) emit(1, "TX trial %d ch %d: %d samples", trial, ch, tot);
    V21TX_delete(o);
    trial_end("tx run");
  }
}

/* ============================================================================================ scenario 5: RX state machine */
static void warm_rx(V21Rx *o, int tone, int ch, int blocks)
{
  Fsk g;
  int i;
  static const unsigned char ones[1] = {1};
  fsk_init(&g, ch, 9000, 0, 0);
  for (i = 0; i < blocks; i++) {
    short b[160];
    if (tone) fsk_gen(&g, ones, 1, b, 80, 1);
    else memset(b, 0, sizeof b);
    DemodDataV21(o, b, g_out, 80);
  }
}
static void scen5(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 2400; trial++) {
    int fnsel = trial % 5, stsel = (trial / 5) % 9, ch = (trial / 45) & 1, tone = (trial / 90) & 1, hold = (trial / 180) & 1, ctr = (trial / 360) % 4;
    int dbg = (trial / 1440) & 1;
    static const int sts[9] = {0, 1, 2, 3, 4, 5, 7, -1, 0x7fff};
    int w[7], nn;
    V21Rx *o;
    unsigned short n;
    int ret;
    Fsk g;
    static const unsigned char pat[3] = {1, 0, 1};
    make_cfg_rx(w, ch);
    o = V21RX_create(NULL, w);
    warm_rx(o, tone, ch, 3 + (int)rmod(5));
    o->hdx->state = (short)sts[stsel];
    o->hdx->hold = hold;
    o->hdx->wait = (short)(ctr == 0 ? 0 : ctr == 1 ? 1 : ctr == 2 ? 2 : -3);
    o->hdx->ones = (short)(ctr == 3 ? 6 : (int)rmod(8));
    o->hdx->flags = (short)(ctr == 0 ? 4 : ctr == 1 ? 5 : ctr == 2 ? 0 : 9);
    o->st.w = (int)rnd();
    nn = 1 + (int)rmod(100);
    stack_poison();
    if (stsel > 2) cov_hit(CV_SR_DEFAULT);
    if (hold && fnsel == 2) cov_hit(CV_SR_HOLD);
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_SR_DBG);
    fsk_init(&g, ch, 9000, 0, 0);
    if (tone) fsk_gen(&g, pat, 3, g_inb, nn, 1);
    else memset(g_inb, 0, (size_t)nn * 2);
    n = (unsigned short)nn;
    {
      int i;
      for (i = 0; i < 4096; i++) g_out[i] = 0x7777;
    }
    d_rx("pre", o);
    stack_poison();
    switch (fnsel) {
      case 0: o->hdx->fn = RxHdxStartV21; ret = RxHdxStartV21(o, g_inb, g_out, &n); cov_hit(CV_SR_H0); break;
      case 1: o->hdx->fn = RxHdxWaitV21; ret = RxHdxWaitV21(o, g_inb, g_out, &n); cov_hit(CV_SR_H1); break;
      case 2: o->hdx->fn = RxHdxDataV21; ret = RxHdxDataV21(o, g_inb, g_out, &n); cov_hit(CV_SR_H2); break;
      case 3: o->hdx->fn = RxHdxIdleV21; ret = RxHdxIdleV21(o, g_inb, g_out, &n); cov_hit(CV_SR_H3); break;
      default: o->hdx->fn = RxHdxErrorV21; ret = RxHdxErrorV21(o, g_inb, g_out, &n); cov_hit(CV_SR_H4); break;
    }
    dbg_level(0);
    if (fnsel == 0 && o->hdx->flags > 4) cov_hit(CV_SR_FLAGS5);
    emit(0, "rxsm trial %d fn %d st %d ch %d tone %d hold %d ctr %d: ret=%d n=%d in=%016llx out=%016llx cd=%d", trial, fnsel, sts[stsel], ch, tone, hold, ctr, ret, n, fnv_n(g_inb, nn),
         fnv_n(g_out, nn + 8), CarrierDetectV21(o));
    d_rx("post", o);
    if (fnsel == 0) {
      /* RxNextStateV21 directly, same object */
      o->hdx->state = (short)sts[stsel];
      dbg_level(dbg ? 2 : 0);
      RxNextStateV21(o);
      dbg_level(0);
      d_rx("next", o);
      if (sts[stsel] == 0) cov_hit(CV_SR_NEXT0);
      else if (sts[stsel] == 1) cov_hit(CV_SR_NEXT1);
      else if (sts[stsel] == 2) cov_hit(CV_SR_NEXT2);
      else cov_hit(CV_SR_NEXTD);
    }
    if (fnsel == 1) {
      /* DemodDataV21 with the handler Data / other and SNR */
      int fni = (int)(rnd() & 1);
      o->hdx->fn = fni ? RxHdxDataV21 : RxHdxStartV21;
      if (tone) fsk_gen(&g, pat, 3, g_inb, nn, 1);
      else memset(g_inb, 0, (size_t)nn * 2);
      ret = DemodDataV21(o, g_inb, g_out, (unsigned short)nn);
      emit(0, "demod fn %d: ret=%d in=%016llx out=%016llx w4=%d w8=%d", fni, ret, fnv_n(g_inb, nn), fnv_n(g_out, nn + 8), o->bundle->agcDetect, o->bundle->noTone);
      cov_hit(fni ? CV_SR_DEMOD_DATA : CV_SR_DEMOD_OTHER);
      emit(0, "snr ret=%d buf=%016llx", GetSNRV21(o), fnv_shorts(o->bundle->buf, 160));
      cov_hit(CV_SR_SNR);
      d_rx("demod", o);
    }
    V21RX_delete(o);
    trial_end("rx sm");
  }
  rx_dispatch_tests();
  rx_misc_tests();
}

static int g_fake_calls, g_fake_cons[16], g_fake_ret[16];
static short *g_fake_in, *g_fake_out;
static int fake_rx(V21Rx *o, short *in, short *out, unsigned short *n)
{
  int k = g_fake_calls++;
  int c = g_fake_cons[k & 15], r = g_fake_ret[k & 15];
  (void)o;
  emit(0, "fake rx call %d: in+%d out+%d n=%d -> consume %d ret %d", k, (int)(in - g_fake_in), (int)(out - g_fake_out), *n, c, r);
  *n = (unsigned short)(*n - c);
  return r;
}
/* the dispatcher V21RX_modem with a handler that consumes part of the block and returns any bit count; flag byte pokes */
static void rx_dispatch_tests(void)
{
  int t, i;
  for (t = 0; t < 300; t++) {
    int w[6];
    V21Rx *o;
    unsigned short n;
    int nn = 1 + (int)rmod(60), left, ret;
    make_cfg_rx(w, t & 1);
    o = V21RX_create(NULL, w);
    left = nn;
    for (i = 0; i < 16; i++) {
      int c = left > 0 ? 1 + (int)rmod((unsigned)(left > 3 ? left / 2 + 1 : left)) : 0;
      if (i == 15 || (t % 7 == 0 && i == 3)) c = left;
      if (c > left) c = left;
      g_fake_cons[i] = c;
      left -= c;
      g_fake_ret[i] = (int)rmod(41) - (t % 5 == 0 ? 5 : 0) + (int)(rpct(10) ? 40000 : 0); /* up to a negative short */
    }
    g_fake_calls = 0;
    g_fake_in = &g_inb[0];
    g_fake_out = &g_out[0];
    o->hdx->fn = fake_rx;
    o->st.w = (int)rnd();
    for (i = 0; i < 64; i++) g_inb[i] = (short)i;
    n = (unsigned short)nn;
    ret = V21RX_modem(o, g_inb, &g_out[64], &n);
    emit(0, "rxdisp t=%d nn=%d: ret=%08x n=%d calls=%d st=%08x", t, nn, u(ret), n, g_fake_calls, u(o->st.w));
    V21RX_delete(o);
    trial_end("rx disp");
  }
}
/* V21RX_control (every flag byte, NULL) and V21RX_status (field pokes, NULL), GetSNRV21 with every fsd count, Start with extreme flag counters */
static void rx_misc_tests(void)
{
  int t, f;
  for (f = 0; f < 256; f++) {
    int w[6];
    V21Rx *o;
    FaxV21Ctl ctl;
    make_cfg_rx(w, f & 1);
    o = V21RX_create(NULL, w);
    o->hdx->hold = (int)(rnd() & 3u);
    o->hdx->state = (short)(f & 7);
    o->cfg.w[2] = 0x1111;
    memset(&ctl, 0, sizeof ctl);
    ctl.w4 = 100 + f;
    ctl.flags = (unsigned char)f;
    emit(0, "rxctl f=%02x: ret=%d", f, V21RX_control(o, &ctl));
    d_rx("ctl", o);
    emit(0, "rxctl null: ret=%d", V21RX_control(o, NULL));
    V21RX_delete(o);
    trial_end("rx ctl");
  }
  for (t = 0; t < 200; t++) {
    int w[6];
    V21Rx *o;
    FaxV21Status st;
    make_cfg_rx(w, t & 1);
    o = V21RX_create(NULL, w);
    o->bundle->fsd.s22 = (short)((int)rmod(400) - 200);
    o->bundle->fsd.c4.s[1] = (short)(rpct(50) ? 24 : (int)(1 + rmod(60)) * (rpct(30) ? -1 : 1));
    o->st.b[1] = (unsigned char)rnd();
    memset(&st, t % 3 == 0 ? 0xff : t % 3 == 1 ? 0x55 : 0x00, sizeof st);
    emit(0, "rxstat t=%d ret=%d", t, V21RX_status(o, &st));
    emit(0, "rxstat %d %d %d %d %d %d %d %d %d %d %02x %02x", st.channel, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15);
    emit(0, "rxstat null ret=%d", V21RX_status(o, NULL));
    V21RX_delete(o);
    trial_end("rx stat");
  }
  for (t = 0; t < 120; t++) {
    int w[6], i;
    V21Rx *o;
    static const short cnts[12] = {0, 1, -1, -5, 3, 10, 9, 2, 8, 7, -32768, 32767};
    make_cfg_rx(w, t & 1);
    o = V21RX_create(NULL, w);
    for (i = 0; i < 160; i++) o->bundle->buf[i] = 0x2222;
    for (i = 0; i < 10; i++) o->bundle->fsd.out[i] = (short)((int)rmod(200) - 100); /* the FSD output buffer holds 10 shorts */
    o->bundle->fsd.outCount = cnts[t % 12] > 10 ? (short)(t % 5) : cnts[t % 12]; /* a count above 10 would read past the FSD output buffer (the blob too) */
    {
      int r = GetSNRV21(o); /* (the arguments of emit are evaluated in an unspecified order) */
      emit(0, "snr t=%d s20=%d ret=%d buf=%016llx", t, o->bundle->fsd.outCount, r, fnv_shorts(o->bundle->buf, 160));
    }
    V21RX_delete(o);
    trial_end("rx snr");
  }
  for (t = 0; t < 120; t++) {
    int w[6], i;
    V21Rx *o;
    unsigned short n = 40;
    static const short fl[12] = {-32768, -1, 0, 4, 5, 6, 32767, 1, 3, 9, -5, 255};
    static const short on[6] = {0, 6, 5, 7, 1, -1};
    make_cfg_rx(w, t & 1);
    o = V21RX_create(NULL, w);
    warm_rx(o, 1, t & 1, 4);
    o->hdx->fn = RxHdxStartV21;
    o->hdx->state = 0;
    o->hdx->flags = fl[t % 12];
    o->hdx->ones = on[(t / 12) % 6];
    for (i = 0; i < 4096; i++) g_out[i] = 0x7777;
    {
      Fsk g;
      static const unsigned char pat[3] = {1, 0, 1};
      fsk_init(&g, t & 1, 9000, 0, 0);
      fsk_gen(&g, pat, 3, g_inb, 40, 1);
    }
    stack_poison();
    {
      int r = RxHdxStartV21(o, g_inb, g_out, &n);
      emit(0, "rxstart t=%d flags=%d ones=%d: ret=%d n=%d", t, fl[t % 12], on[(t / 12) % 6], r, n);
    }
    d_rx("start", o);
    V21RX_delete(o);
    trial_end("rx start");
  }
}

/* ============================================================================================ scenario 6: TX state machine */
static void scen6(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 2400; trial++) {
    int fnsel = trial % 3, stsel = (trial / 3) % 6, fill = (trial / 18) % 4, pend = (trial / 72) & 1, nsel = (trial / 144) % 5, dbg = (trial / 720) & 1;
    static const int sts[6] = {0, 1, 2, 3, 9, -1};
    static const int ns[5] = {0, 1, 3, 6, 5};
    int w[7], i, ret;
    V21Tx *o;
    unsigned short n;
    short bits[16], ob[1];
    make_cfg_tx(w, trial & 1);
    o = V21TX_create(NULL, w);
    for (i = 0; i < 16; i++) bits[i] = (short)(rnd() & 1);
    {
      unsigned short nf = (unsigned short)(fill == 0 ? 0 : fill == 1 ? 2 : fill == 2 ? 6 : 4);
      if (nf) FIFO_write(o->hdx->fifo, (const unsigned short *)bits, nf);
      cov_hit(nf ? CV_ST_FIFOF : CV_ST_FIFO0);
    }
    o->hdx->state = (short)sts[stsel];
    o->hdx->pend = pend;
    if (pend) cov_hit(CV_ST_PEND);
    o->hdx->s0e = (short)(rnd() & 3);
    o->st.w = (int)rnd();
    if (stsel > 2) cov_hit(CV_ST_DEFAULT);
    n = (unsigned short)ns[nsel];
    for (i = 0; i < 4096; i++) g_out[i] = 0x7777;
    for (i = 0; i < 16; i++) bits[i] = (short)(rnd() & 1);
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_ST_DBG);
    d_tx("pre", o);
    stack_poison();
    switch (fnsel) {
      case 0: o->hdx->fn = TxHdxStartV21; ret = TxHdxStartV21(o, bits, g_out, &n); cov_hit(CV_ST_H0); break;
      case 1: o->hdx->fn = TxHdxDataV21; ret = TxHdxDataV21(o, bits, g_out, &n); cov_hit(CV_ST_H1); break;
      default: o->hdx->fn = TxHdxIdleV21; ret = TxHdxIdleV21(o, bits, g_out, &n); cov_hit(CV_ST_H2); break;
    }
    dbg_level(0);
    emit(0, "txsm trial %d fn %d st %d fill %d pend %d n %d: ret=%d n=%d in=%016llx out=%016llx", trial, fnsel, sts[stsel], fill, pend, ns[nsel], ret, n, fnv_n(bits, 16), fnv_n(g_out, ret + 8));
    d_tx("post", o);
    if (fnsel == 0) {
      o->hdx->state = (short)sts[stsel];
      dbg_level(dbg ? 2 : 0);
      TxNextStateV21(o);
      dbg_level(0);
      d_tx("next", o);
      if (sts[stsel] == 0) cov_hit(CV_ST_NEXT0);
      else if (sts[stsel] == 1) cov_hit(CV_ST_NEXT1);
      else if (sts[stsel] == 2) cov_hit(CV_ST_NEXT2);
      else cov_hit(CV_ST_NEXTD);
    }
    if (fnsel == 1) {
      int c = (int)(rnd() % 7);
      ret = ModDataV21(o, bits, g_out, (unsigned short)c);
      emit(0, "moddata n=%d ret=%d out=%016llx", c, ret, fnv_n(g_out, ret + 8));
      cov_hit(CV_ST_MOD);
      d_tx("mod", o);
      ret = TxNoCarrierV21(o, bits, g_out, (unsigned short)c);
      emit(0, "nocarrier n=%d ret=%d out=%016llx", c, ret, fnv_n(g_out, ret + 8));
      cov_hit(CV_ST_NOCAR);
      d_tx("nocar", o);
    }
    (void)ob;
    V21TX_delete(o);
    trial_end("tx sm");
  }
  tx_dispatch_tests();
  tx_misc_tests();
}

static int fake_tx(V21Tx *o, short *in, short *out, unsigned short *n)
{
  int k = g_fake_calls++;
  int c = g_fake_cons[k & 15], r = g_fake_ret[k & 15];
  (void)o;
  emit(0, "fake tx call %d: in+%d out+%d n=%d -> consume %d ret %d", k, (int)(in - g_fake_in), (int)(out - g_fake_out), (short)*n, c, r);
  *n = (unsigned short)(*n - c);
  return r;
}
/* the dispatcher V21TX_modem with a handler that takes part of the count (also more than is left) and returns any sample count; flag byte and FIFO pokes */
static void tx_dispatch_tests(void)
{
  int t, i;
  for (t = 0; t < 400; t++) {
    int w[7], ret;
    V21Tx *o;
    unsigned short n;
    int nn = (int)rmod(10);
    make_cfg_tx(w, 1);
    o = V21TX_create(NULL, w);
    for (i = 0; i < 16; i++) {
      g_fake_cons[i] = i >= 14 ? 100 : (int)rmod(5) - (t % 3 == 0 ? 0 : 1);
      g_fake_ret[i] = (int)rmod(41) - (t % 4 == 0 ? 5 : 0);
    }
    g_fake_calls = 0;
    g_fake_in = &g_inb[0];
    g_fake_out = &g_out[0];
    for (i = 0; i < 16; i++) g_inb[i] = (short)(rnd() & 1);
    o->hdx->fn = fake_tx;
    o->hdx->pend = t & 1;
    if (t % 5 < 2) FIFO_write(o->hdx->fifo, (const unsigned short *)g_inb, (unsigned short)(1 + t % 6));
    o->st.w = (int)rnd();
    n = (unsigned short)nn;
    ret = V21TX_modem(o, g_inb, &g_out[64], &n);
    emit(0, "txdisp t=%d nn=%d: ret=%08x n=%d calls=%d st=%08x", t, nn, u(ret), n, g_fake_calls, u(o->st.w));
    d_fifo("disp", o->hdx->fifo);
    V21TX_delete(o);
    trial_end("tx disp");
  }
}
/* V21TX_control (every flag byte of both bytes, NULL) and V21TX_status (field pokes, NULL) */
static void tx_misc_tests(void)
{
  int t, f;
  for (f = 0; f < 512; f++) {
    int w[7];
    V21Tx *o;
    FaxV21Ctl ctl;
    make_cfg_tx(w, f & 1);
    o = V21TX_create(NULL, w);
    o->hdx->pend = (int)(rnd() & 3u);
    o->hdx->state = (short)(f & 3);
    memset(&ctl, 0, sizeof ctl);
    ctl.w4 = 100 + f;
    ctl.txScale = (int)rnd();
    if (f < 256) {
      ctl.flags = (unsigned char)f;
      ctl.b0c = (unsigned char)rnd();
    } else {
      ctl.b0c = (unsigned char)(f - 256);
      ctl.flags = (unsigned char)(rnd() & 0xfdu);
    }
    emit(0, "txctl f=%d: ret=%d", f, V21TX_control(o, &ctl));
    d_tx("ctl", o);
    emit(0, "txctl null: ret=%d", V21TX_control(o, NULL));
    V21TX_delete(o);
    trial_end("tx ctl");
  }
  for (t = 0; t < 200; t++) {
    int w[7];
    V21Tx *o;
    FaxV21Status st;
    make_cfg_tx(w, t & 3);
    o = V21TX_create(NULL, w);
    o->cfg.b[0x10] = (unsigned char)rnd();
    o->cfg.s[0] = (short)(rpct(50) ? (int)(t & 3) : (int)rnd());
    memset(&st, t % 3 == 0 ? 0xff : t % 3 == 1 ? 0x55 : 0x00, sizeof st);
    emit(0, "txstat t=%d ret=%d", t, V21TX_status(o, &st));
    emit(0, "txstat %d %d %d %d %d %d %d %d %d %d %02x %02x", st.channel, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15);
    emit(0, "txstat null ret=%d", V21TX_status(o, NULL));
    V21TX_delete(o);
    trial_end("tx stat");
  }
}

/* ============================================================================================ scenario 7: VMI wrappers */
/* RS9: FaxVmiM's word at +8 is the two shorts s08 / s0a (the int alias `w8` of an anonymous union is gone); the dump still prints it as one int, read from
   the two shorts.  (EOS: the first version of this view, a union with struct { int a, b, w8; }, assumed the 4-byte u4 of the i386 object and dumped the
   wrong word at x86-64, where u4 is a pointer: r19_life GOLDEN64 FAIL.) */
static void d_vmi(const char *t, const FaxVmiV21 *m)
{
  char p[80];
  int w8;
  memcpy(&w8, &m->s08, sizeof w8);
  emit(0, "%s VMI w0=%d w8=%d s0c=%d s0e=%d s10=%d bits=%s eng=%s", t, m->w0, w8, m->s0c, m->s0e, m->u10.s10, m->u4.bits ? "set" : "NULL", dptr(m->eng, p, 80));
}
static void scen7(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 24; trial++) {
    FaxVmiV21 tm, rm;
    int tw[7], rw[7], i, total = 0, nbits, c, pos = 0, prev = 0, ret;
    unsigned short n, on, n2;
    short *bitbuf = (short *)sysdep_malloc(64 * 2);
    short *rxbits = (short *)sysdep_malloc(2048 * 2);
    unsigned long long oh = 0;
    int nbt = 0;
    memset(&tm, 0xa5, sizeof tm); /* the create writes the three shorts: garbage before */
    memset(&rm, 0x5a, sizeof rm);
    tm.u4.bits = bitbuf;
    rm.u4.bits = rxbits;
    make_cfg_tx(tw, 1);
    make_cfg_rx(rw, 1);
    if (trial % 5 >= 2) {
      fuzz_cfg(tw, 7);
      fuzz_cfg(rw, 6);
    }
    tm.eng = NULL;
    rm.eng = NULL;
    stack_poison();
    v21tx_create(&tm, trial % 4 == 0 ? NULL : tw);
    stack_poison();
    v21rx_create(&rm, trial % 3 == 0 ? NULL : rw);
    cov_hit(CV_V_TX_CREATE);
    cov_hit(CV_V_RX_CREATE);
    cov_hit((trial % 4 == 0) ? CV_V_CFGNULL : CV_V_CFGSET);
    d_vmi("tx", &tm);
    d_vmi("rx", &rm);
    nbits = build_bits(g_bits, (int)sizeof g_bits, 6, 40 + (int)rmod(60));
    for (c = 0; c < nbits && total < MAXS - 400; c += 5) {
      int m = nbits - c < 5 ? nbits - c : 5;
      for (i = 0; i < m; i++) bitbuf[i] = g_bits[c + i];
      n = (unsigned short)m;
      on = 0xabcd;
      ret = v21tx_process(&tm, &g_src[total], &n, &on);
      emit(0, "v21tx_process m=%d ret=%08x n=%d outn=%d out=%016llx", m, u(ret), n, on, fnv_n(&g_src[total], on));
      total += on;
      cov_hit(CV_V_TX_PROC);
    }
    while (pos < total) {
      int nn = (int)(1 + rmod(100));
      if (pos + nn > total) nn = total - pos;
      memcpy(g_inb, &g_src[pos], (size_t)nn * 2);
      n2 = (unsigned short)nn;
      on = 0xabcd;
      ret = v21rx_process(&rm, g_inb, &on, &n2);
      emit(0, "v21rx_process n=%d ret=%08x n=%d outn=%d bits=%016llx", nn, u(ret), n2, on, fnv_n(rxbits, on + 4));
      cov_hit(CV_V_RX_PROC);
      nbt += on;
      oh ^= fnv_n(rxbits, on);
      pos += nn;
      (void)prev;
    }
    emit(0, "vmi trial %d bits=%d hash=%016llx", trial, nbt, oh);
    {
      FaxV21Status st;
      FaxV21Ctl ctl;
      memset(&st, 0x55, sizeof st);
      emit(0, "v21rx_status ret=%d", v21rx_status(&rm, &st));
      emit(0, "st %d %d %d %d %d %d %d %d %d %d %02x %02x", st.channel, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15);
      memset(&st, 0x55, sizeof st);
      emit(0, "v21tx_status ret=%d", v21tx_status(&tm, &st));
      emit(0, "st %d %d %d %d %d %d %d %d %d %d %02x %02x", st.channel, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15);
      cov_hit(CV_V_STATUS);
      memset(&ctl, 0, sizeof ctl);
      ctl.w4 = 1234 + trial;
      ctl.txScale = 0x2000;
      ctl.b0c = 4;
      ctl.flags = (unsigned char)(trial & 1 ? 0x12 : 0x10);
      emit(0, "v21rx_control ret=%d", v21rx_control(&rm, &ctl));
      emit(0, "v21tx_control ret=%d", v21tx_control(&tm, &ctl));
      emit(0, "v21rx_control null ret=%d", v21rx_control(&rm, NULL));
      emit(0, "v21tx_control null ret=%d", v21tx_control(&tm, NULL));
      cov_hit(CV_V_CTL);
      d_vmi("tx2", &tm);
      d_vmi("rx2", &rm);
      d_rx("rxeng", (V21Rx *)rm.eng);
      d_tx("txeng", (V21Tx *)tm.eng);
    }
    v21tx_delete(&tm);
    v21rx_delete(&rm);
    sysdep_free(bitbuf);
    sysdep_free(rxbits);
    cov_hit(CV_V_DEL);
    trial_end("vmi");
  }
}

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : -1;
  const char *e;
  if (scen < 0 || scen > 7) {
    fprintf(stderr, "usage: %s scenario(0..7)\n", argv[0]);
    return 2;
  }
  g_verbose = (e = getenv("R19_VERBOSE")) != NULL && *e == '1';
  if ((e = getenv("R19_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(e, NULL, 0);
  g_lcg += (unsigned)scen * 0x9e3779b9u;
  alarm(240);
  cov_setup(scen);
  switch (scen) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    case 4: scen4(); break;
    case 5: scen5(); break;
    case 6: scen6(); break;
    default: scen7(); break;
  }
  cov_finish();
  return 0;
}
