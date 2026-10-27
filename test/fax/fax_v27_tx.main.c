/*
 * R20 differential harness: the fax V.27ter TRANSMITTER of dsplibs.o (V27TX_create / _delete / _modem / _control / _status, TxHdx{Start,Quiet,EQCond,SCR1,Alt,Data,Idle}V27, TxNextStateV27,
 * ModDataV27, TxNoCarrierV27, the v27tx_* VMI wrappers and the blob-LOCAL init_vmi_v27tx) vs src/fax/fax_v27_tx.c + src/fax/fax_pulse_shaping.c + tables.  The blob run (-DTEST_BLOB, linked with ../dsplibs.o,
 * init_vmi_v27tx made global by objcopy) and the port run (-m32 -O0 / -O2, x86-64 against the recorded blob output) execute the same code; library test/fax/r20_common.h (hashed canonical output,
 * coverage counters exit 2, 3 = harness violation, deterministic heap with block ids / red zones / poisoned frees, callee recorders sysdep_malloc / sysdep_free / dsplibs_debug_printf).  The
 * R18 callees (FIFO_*, SGD_*, SDMv27_*, SMC_*, ScrambleDataV27, SetScramblerV27) are the real ones on both sides (the blob's own in the blob run, src/fax/fax_*.c in the port run).  Integer
 * signals only: the modulated samples are compared sample for sample through hashes of every call's output block.  Nothing printed depends on the host layout: objects are dumped member by
 * member (pointers as table+index / block id, the content of every malloc()ed block as a hash).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 tables: every R20 table dumped (incl. the filter tables the pulse shaper reads), the message table through v27tx_message for -3 .. 12 and 0x100 ..;
 *   1 lifecycle: V27TX_create (NULL object / caller-supplied with and without hdx / bundle / live, NULL / custom / fuzzed cfg words, rates 2400 / 4800 / other, debug level 0 and 2),
 *     create(o, o), delete, run-after-create, forked probes: a failing malloc #0 .. #11, a supplied object without bundle, the rate-changing re-create (the blob clears more circular buffer than
 *     it allocated: the red zone verdict is compared);
 *   2 transmit streams: both rates through the whole machine (start, quiet, carrier, quiet, alternations, EQ conditioning, SCR1, data, SCR1, quiet, idle, start ...), steady / short / long input
 *     blocks (under-run, over-run), pend / gain / re-create control, status, random symbol data, 60 engines, the whole engine dumped after every call;
 *   3 handlers: every TxHdx*V27 x every hdx state {0 .. 10, -1, 11, 0x7fff} x rate x count x FIFO fill x pend x n, plus TxHdxDataV27 with odd rate words;
 *   4 TxNextStateV27 for every state x rate x s0e x debug level (the jump table: every case and the default), ModDataV27 / TxNoCarrierV27 directly with n up to the circular buffer wrap;
 *   5 V27TX_modem with a harness handler (any count / return value), V27TX_control for 512 flag-byte combinations (then a modulated block: the gain), V27TX_status with pokes;
 *   6 VMI: fax_init_vmi_v27tx (blob: init_vmi_v27tx) with debug levels 0 .. 3, v27tx_create / process / status / control / delete / message on garbage-initialised FaxVmiV27 objects.
 * R20_VERBOSE=1 prints every line (debugging), R20_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r20_common.h"
#include "fax/fax_v27_tx_tables.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
extern int init_vmi_v27tx(FaxVmiV27 *m, unsigned short rate, int unused, void *user);
#define fax_init_vmi_v27tx init_vmi_v27tx
#else
#define SIDE_BLOB 0
#endif

extern unsigned int dsplibs_debug_level;

#include "fax/fax_modem_views.h"
#if UINTPTR_MAX != 0xffffffffu
/* x86-64: src/fax/fax_modem_wrappers.c references ALL eleven accessors of src/fax/fax_modem_views.h; this test uses the two of the V.27ter TRANSMIT half, which src/fax/fax_v27_tx.c defines.  The other nine belong to the
 * V.17 / V.29 / V.27 receive engines (R21 - R26): stand-ins that abort (no scenario of this test reaches the wrappers that use them). */
static void *no_engine(void) { fprintf(stderr, "HARNESS/MODEL VIOLATION: accessor of another engine called\n"); exit(3); }
FaxSdm *faxw_v17_txsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSmcV17 *faxw_v17_txsmc(void *o) { (void)o; return (FaxSmcV17 *)no_engine(); }
unsigned short *faxw_v17_encmode(void *o) { (void)o; return (unsigned short *)no_engine(); }
FaxSdm *faxw_v17_rxsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSdm *faxw_v29_txsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
int *faxw_v29_encsel(void *o) { (void)o; return (int *)no_engine(); }
FaxSdm *faxw_v29_rxsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSdmV27 *faxw_v27_rxsdm(void *o) { (void)o; return (FaxSdmV27 *)no_engine(); }
#endif
/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_TAB, CV_MSG, CV_MSG_OOB, CV_MSG_NEG,
  CV_L_NEW, CV_L_SUP_EMPTY, CV_L_SUP_NOHDX, CV_L_SUP_LIVE, CV_L_NULLCFG, CV_L_R2400, CV_L_R4800, CV_L_RBAD, CV_L_DBG, CV_L_DEL, CV_L_PROBE, CV_L_RECREATE, CV_L_RUN, CV_L_REALLOC,
  CV_L_NOREALLOC, CV_L_OVERRUN, CV_L_S0E0, CV_L_S0E1, CV_L_SUPNOBUNDLE,
  CV_S_START, CV_S_QUIET, CV_S_CARR, CV_S_NOCARR, CV_S_ALT, CV_S_EQ, CV_S_SCR1, CV_S_DATA, CV_S_TURNOFF, CV_S_IDLE, CV_S_TO_DATA, CV_S_TO_IDLE, CV_S_TO_START, CV_S_UNDER, CV_S_OVER,
  CV_S_PEND, CV_S_GAIN, CV_S_STATUS, CV_S_RECREATE, CV_S_R2400, CV_S_R4800, CV_S_N0, CV_S_NFULL, CV_S_NLONG, CV_S_NSHORT, CV_S_DBG, CV_S_S0E0, CV_S_S0E1, CV_S_BADRATE, CV_S_BIGFIFO,
  CV_H_START, CV_H_QUIET, CV_H_EQ, CV_H_SCR1, CV_H_ALT, CV_H_DATA, CV_H_IDLE, CV_H_COUNT0, CV_H_COUNTNEG, CV_H_COUNTBIG, CV_H_FIFO0, CV_H_FIFOF, CV_H_PEND, CV_H_UNDER, CV_H_UNDERPEND,
  CV_H_STATEDEF, CV_H_RATEODD, CV_H_N0, CV_H_NSMALL,
  CV_N_C0, CV_N_C1, CV_N_C2, CV_N_C3, CV_N_C4, CV_N_C5, CV_N_C6, CV_N_C7, CV_N_C8, CV_N_C9, CV_N_C10, CV_N_DEFAULT, CV_N_DBG, CV_N_MOD, CV_N_NOCAR, CV_N_WRAP, CV_N_MODE0,
  CV_D_FAKE, CV_D_PEND, CV_D_NOPEND, CV_D_OVER, CV_D_NEGRET, CV_D_CTL, CV_D_CTL_PEND, CV_D_CTL_RECREATE, CV_D_CTL_GAIN, CV_D_CTL_B0C, CV_D_CTL_NULL, CV_D_STAT, CV_D_STAT_NULL,
  CV_V_INIT, CV_V_INIT_DBG, CV_V_CREATE, CV_V_CREATE_NULLCFG, CV_V_CREATE_SUP, CV_V_PROC, CV_V_STATUS, CV_V_CTL, CV_V_DEL, CV_V_MSG, CV_V_R2400, CV_V_R4800
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_TAB, "tables dumped");
      cov_def(CV_MSG, "v27tx_message idx 0..7");
      cov_def(CV_MSG_OOB, "v27tx_message idx 8 (out of table)");
      cov_def(CV_MSG_NEG, "v27tx_message idx > limit / negative");
      break;
    case 1:
      cov_def(CV_L_NEW, "V27TX_create NULL object");
      cov_def(CV_L_SUP_EMPTY, "V27TX_create supplied object with live hdx / bundle");
      cov_def(CV_L_SUP_NOHDX, "V27TX_create supplied object, hdx NULL");
      cov_def(CV_L_SUP_LIVE, "V27TX_create supplied live object (re-create)");
      cov_def(CV_L_NULLCFG, "V27TX_create NULL cfg");
      cov_def(CV_L_R2400, "V27TX_create rate 2400");
      cov_def(CV_L_R4800, "V27TX_create rate 4800");
      cov_def(CV_L_RBAD, "V27TX_create other rate (error flags)");
      cov_def(CV_L_DBG, "V27TX_create debug level 2");
      cov_def(CV_L_DEL, "V27TX_delete");
      cov_def(CV_L_PROBE, "create malloc-failure probe");
      cov_def(CV_L_RECREATE, "V27TX_create(o, o)");
      cov_def(CV_L_RUN, "run after create");
      cov_def(CV_L_REALLOC, "PPS buffers re-allocated (taps grow)");
      cov_def(CV_L_NOREALLOC, "PPS buffers kept (taps shrink / equal)");
      cov_def(CV_L_OVERRUN, "rate-changing re-create probe");
      cov_def(CV_L_S0E0, "cfg word 6 != 0");
      cov_def(CV_L_S0E1, "cfg word 6 == 0");
      cov_def(CV_L_SUPNOBUNDLE, "supplied object without bundle probe");
      break;
    case 2:
      cov_def(CV_S_START, "state Start observed");
      cov_def(CV_S_QUIET, "state Quiet observed");
      cov_def(CV_S_CARR, "state Carrier observed");
      cov_def(CV_S_NOCARR, "state 3 (quiet) observed");
      cov_def(CV_S_ALT, "state Alt observed");
      cov_def(CV_S_EQ, "state EQCond observed");
      cov_def(CV_S_SCR1, "state SCR1 observed");
      cov_def(CV_S_DATA, "state Data observed");
      cov_def(CV_S_TURNOFF, "state 8 / 9 observed");
      cov_def(CV_S_IDLE, "state Idle observed");
      cov_def(CV_S_TO_DATA, "SCR1 -> Data");
      cov_def(CV_S_TO_IDLE, "-> Idle");
      cov_def(CV_S_TO_START, "Idle -> Start");
      cov_def(CV_S_UNDER, "TX under-run");
      cov_def(CV_S_OVER, "TX over-run");
      cov_def(CV_S_PEND, "pend control");
      cov_def(CV_S_GAIN, "gain control");
      cov_def(CV_S_STATUS, "status");
      cov_def(CV_S_RECREATE, "control re-create");
      cov_def(CV_S_R2400, "rate 2400 engine");
      cov_def(CV_S_R4800, "rate 4800 engine");
      cov_def(CV_S_N0, "call with n = 0");
      cov_def(CV_S_NFULL, "call with a full frame");
      cov_def(CV_S_NLONG, "call with more than a frame");
      cov_def(CV_S_NSHORT, "call with less than a frame");
      cov_def(CV_S_DBG, "debug level 2 stream");
      cov_def(CV_S_S0E0, "stream with cfg word 6 != 0");
      cov_def(CV_S_S0E1, "stream with cfg word 6 == 0");
      cov_def(CV_S_BADRATE, "stream with an invalid rate");
      cov_def(CV_S_BIGFIFO, "stream with a 2 frame FIFO");
      break;
    case 3:
      cov_def(CV_H_START, "TxHdxStartV27");
      cov_def(CV_H_QUIET, "TxHdxQuietV27");
      cov_def(CV_H_EQ, "TxHdxEQCondV27");
      cov_def(CV_H_SCR1, "TxHdxSCR1V27");
      cov_def(CV_H_ALT, "TxHdxAltV27");
      cov_def(CV_H_DATA, "TxHdxDataV27");
      cov_def(CV_H_IDLE, "TxHdxIdleV27");
      cov_def(CV_H_COUNT0, "count 0");
      cov_def(CV_H_COUNTNEG, "count negative");
      cov_def(CV_H_COUNTBIG, "count > n");
      cov_def(CV_H_FIFO0, "FIFO empty");
      cov_def(CV_H_FIFOF, "FIFO filled");
      cov_def(CV_H_PEND, "pend set");
      cov_def(CV_H_UNDER, "Data under-run without pend");
      cov_def(CV_H_UNDERPEND, "Data under-run with pend");
      cov_def(CV_H_STATEDEF, "state number outside 0..10");
      cov_def(CV_H_RATEODD, "Data with a rate word outside 0 / 1");
      cov_def(CV_H_N0, "n = 0");
      cov_def(CV_H_NSMALL, "n < count");
      break;
    case 4:
      cov_def(CV_N_C0, "TxNextStateV27 state 0");
      cov_def(CV_N_C1, "TxNextStateV27 state 1");
      cov_def(CV_N_C2, "TxNextStateV27 state 2");
      cov_def(CV_N_C3, "TxNextStateV27 state 3");
      cov_def(CV_N_C4, "TxNextStateV27 state 4");
      cov_def(CV_N_C5, "TxNextStateV27 state 5");
      cov_def(CV_N_C6, "TxNextStateV27 state 6");
      cov_def(CV_N_C7, "TxNextStateV27 state 7");
      cov_def(CV_N_C8, "TxNextStateV27 state 8");
      cov_def(CV_N_C9, "TxNextStateV27 state 9");
      cov_def(CV_N_C10, "TxNextStateV27 state 10");
      cov_def(CV_N_DEFAULT, "TxNextStateV27 default");
      cov_def(CV_N_DBG, "debug strings");
      cov_def(CV_N_MOD, "ModDataV27");
      cov_def(CV_N_NOCAR, "TxNoCarrierV27");
      cov_def(CV_N_WRAP, "circular buffer wrap");
      cov_def(CV_N_MODE0, "pulse shaper mode 0 (I / Q buffers) and phase step");
      break;
    case 5:
      cov_def(CV_D_FAKE, "V27TX_modem with a harness handler");
      cov_def(CV_D_PEND, "dispatcher with pend");
      cov_def(CV_D_NOPEND, "dispatcher without pend");
      cov_def(CV_D_OVER, "dispatcher over-run");
      cov_def(CV_D_NEGRET, "handler returns a negative count");
      cov_def(CV_D_CTL, "V27TX_control");
      cov_def(CV_D_CTL_PEND, "control pend");
      cov_def(CV_D_CTL_RECREATE, "control re-create");
      cov_def(CV_D_CTL_GAIN, "control gain");
      cov_def(CV_D_CTL_B0C, "control b0c bit 2");
      cov_def(CV_D_CTL_NULL, "control NULL");
      cov_def(CV_D_STAT, "V27TX_status");
      cov_def(CV_D_STAT_NULL, "status NULL");
      break;
    case 6:
      cov_def(CV_V_INIT, "fax_init_vmi_v27tx");
      cov_def(CV_V_INIT_DBG, "init debug level 3");
      cov_def(CV_V_CREATE, "v27tx_create");
      cov_def(CV_V_CREATE_NULLCFG, "v27tx_create NULL cfg");
      cov_def(CV_V_CREATE_SUP, "v27tx_create with a supplied engine");
      cov_def(CV_V_PROC, "v27tx_process");
      cov_def(CV_V_STATUS, "v27tx_status");
      cov_def(CV_V_CTL, "v27tx_control");
      cov_def(CV_V_DEL, "v27tx_delete");
      cov_def(CV_V_MSG, "v27tx_message");
      cov_def(CV_V_R2400, "VMI rate 2400");
      cov_def(CV_V_R4800, "VMI rate 4800");
      break;
  }
}

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
  /* the machine's own transitions inside one V27TX_modem call are only visible through the debug strings (level 2 runs) */
  if (!strncmp(b, "V27TX_STATE_START", 17)) cov_hit(CV_S_START);
  if (!strncmp(b, "V27TX_STATE_IDLE", 16)) cov_hit(CV_S_TO_START);
  return n;
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
static void d_fifo(const char *t, const FaxFifo *f)
{
  char p[80];
  if (!f) {
    emit(0, "%s FIFO NULL", t);
    return;
  }
  emit(0, "%s FIFO w0=%d size=%d fill=%d count=%d rd=%d wr=%d buf=%s", t, f->w0, f->size, f->fill, f->count, f->rd, f->wr, (hblk(p, 80, f->buf), p));
}
/* the SGD: only the words the V.27 code can have set deterministically (the sequence-part words tab / len / mode / w14 are stack garbage in the blob) */
static void d_sgd(const char *t, const FaxSgd *s)
{
  char p[80];
  if (!s) {
    emit(0, "%s SGD NULL", t);
    return;
  }
  emit(0, "%s SGD bits=%u nbuf=%d w4=%d w6=%u w18=%08x w1c=%u pass=%u pos=%u symbit=%u bufn=%d scale=%u buf=%s", t, s->cfg.bits, s->cfg.nbuf, s->cfg.w4, s->cfg.w6, u(s->cfg.pattern),
       (unsigned)(unsigned short)s->cfg.patSyms, s->pass, s->pos, s->symbit, s->bufn, s->scale, (hblk(p, 80, s->buf), p));
}
static const char *txfn(V27TxHdxFn f)
{
  if (f == TxHdxStartV27) return "Start";
  if (f == TxHdxQuietV27) return "Quiet";
  if (f == TxHdxEQCondV27) return "EQCond";
  if (f == TxHdxSCR1V27) return "SCR1";
  if (f == TxHdxAltV27) return "Alt";
  if (f == TxHdxDataV27) return "Data";
  if (f == TxHdxIdleV27) return "Idle";
  return f ? "other" : "NULL";
}
static short g_ibuf[64], g_qbuf[64];
static void register_tables(void)
{
  htab_add("PPS_IMAP_2400", V27TX_PPS_IMAP_2400, sizeof V27TX_PPS_IMAP_2400);
  htab_add("PPS_IMAP_4800", V27TX_PPS_IMAP_4800, sizeof V27TX_PPS_IMAP_4800);
  htab_add("PPS_QMAP_2400", V27TX_PPS_QMAP_2400, sizeof V27TX_PPS_QMAP_2400);
  htab_add("PPS_QMAP_4800", V27TX_PPS_QMAP_4800, sizeof V27TX_PPS_QMAP_4800);
  htab_add("PPS_IFILT_2400", V27TX_PPS_IFILT_2400, sizeof V27TX_PPS_IFILT_2400);
  htab_add("PPS_IFILT_4800", V27TX_PPS_IFILT_4800, sizeof V27TX_PPS_IFILT_4800);
  htab_add("PPS_QFILT_2400", V27TX_PPS_QFILT_2400, sizeof V27TX_PPS_QFILT_2400);
  htab_add("PPS_QFILT_4800", V27TX_PPS_QFILT_4800, sizeof V27TX_PPS_QFILT_4800);
  htab_add("SMC_PMAP_24", V27TX_SMC_PMAP_24, sizeof V27TX_SMC_PMAP_24);
  htab_add("SMC_PMAP_48", V27TX_SMC_PMAP_48, sizeof V27TX_SMC_PMAP_48);
  htab_add("IBUF", g_ibuf, sizeof g_ibuf);
  htab_add("QBUF", g_qbuf, sizeof g_qbuf);
  htab_add("SMC_IMAP", SMC_CFG.imap, 64);
  htab_add("SMC_QMAP", SMC_CFG.qmap, 64);
  htab_add("SMC_COS", SMC_CFG.cosv, 1024);
  htab_add("SMC_SIN", SMC_CFG.sinv, 1024);
}
static void d_tx(const char *t, const V27Tx *o)
{
  char p1[80], p2[80], p3[80], p4[80], p5[80], p6[80], p7[80], p8[80], p9[80];
  emit(0, "%s TX cfg=%08x %08x %08x %08x %08x %08x %08x %08x st=%08x", t, u(o->cfg.w[0]), u(o->cfg.w[1]), u(o->cfg.w[2]), u(o->cfg.w[3]), u(o->cfg.w[4]), u(o->cfg.w[5]), u(o->cfg.w[6]),
       u(o->cfg.w[7]), u(o->st.w));
  if (o->hdx) {
    const V27TxHdx *h = o->hdx;
    emit(0, "%s TXH pend=%d rate=%d s0e=%d fn=%s state=%d count=%d", t, h->pend, h->rate, h->longTrain, txfn(h->fn), h->state, h->count);
    d_fifo(t, h->fifo);
    d_sgd(t, h->sgd);
  }
  if (o->bundle) {
    const V27TxBundle *b = o->bundle;
    const FaxSmcCfg *c = &b->smc.cfg;
    const FaxPps *p = &b->pps;
    emit(0, "%s TXB circ i=%s q=%s buf=%s wr=%d rd=%d lim=%d", t, dptr(b->circ.i, p1, 80), dptr(b->circ.q, p2, 80), (hblk(p3, 80, b->circ.buf), p3), b->circ.wr, b->circ.rd, b->circ.lim);
    emit(0, "%s TXB sdm n=%u mask=%04x nmask=%04x reg=%04x cnt=%u f1=%u f2=%u", t, b->sdm.n, b->sdm.mask, b->sdm.nmask, b->sdm.reg, b->sdm.cnt, b->sdm.f1, b->sdm.f2);
    emit(0, "%s TXB smc w0=%d flag=%d step=%d mod=%d shift=%u mask=%u mask2=%u mask3=%u pmap=%s imap=%s qmap=%s cos=%s sin=%s w28=%08x a=%d b=%d", t, c->phaseOut, c->flag, c->step, c->mod, c->shift,
         c->mask, c->mask2, c->mask3, dptr(c->pmap, p4, 80), dptr(c->imap, p5, 80), dptr(c->qmap, p6, 80), dptr(c->cosv, p7, 80), dptr(c->sinv, p8, 80), u(c->w28), b->smc.a, b->smc.b);
    emit(0, "%s TXB pps up=%d down=%d mode=%d scale=%d step=%d s0e=%d imap=%s qmap=%s ifilt=%s qfilt=%s filtlen=%d s22=%d w24=%08x need=%d acc=%d pos=%d taps=%d", t, p->cfg.up, p->cfg.down,
         p->cfg.mode, p->cfg.scale, p->cfg.step, p->cfg.s0e, dptr(p->cfg.imap, p1, 80), dptr(p->cfg.qmap, p2, 80), dptr(p->cfg.ifilt, p3, 80), dptr(p->cfg.qfilt, p4, 80), p->cfg.filtlen,
         p->cfg.s22, u(p->cfg.w24), p->need, p->acc, p->pos, p->taps);
    emit(0, "%s TXB hist I=%s Q=%s", t, (hblk(p5, 80, p->histI), p5), (hblk(p9, 80, p->histQ), p9));
    (void)p6; (void)p7; (void)p8;
  }
}
static void dbg_level(unsigned v) { dsplibs_debug_level = v; }
static void __attribute__((noinline)) stack_poison(void)
{
  volatile unsigned char b[2048];
  unsigned i;
  for (i = 0; i < sizeof b; i++) b[i] = (unsigned char)(0xa5u ^ (i * 31u));
}

static short g_inb[4096];
static short g_out[8192];
static unsigned long long fnv_n(const short *p, int n) { return fnv_shorts(p, (unsigned)(n < 0 ? 0 : n)); }
static void sym_fill(short *b, int n, int rate)
{
  int i;
  for (i = 0; i < n; i++) b[i] = (short)(rpct(3) ? (int)(rnd() & 0xffffu) : (int)(rnd() & (rate ? 7u : 3u)));
}

static void make_cfg(int *w, int rate, int variant)
{
  int i;
  for (i = 0; i < 8; i++) w[i] = V27TX_CFG[i];
  w[0] = (w[0] & 0xffff) | ((rate == 0 ? 0x960 : rate == 1 ? 0x12c0 : rate) << 16); /* s[1] = the baud rate */
  if (variant & 1) w[6] = 1;                                                      /* s0e = 0 */
  if (variant & 2) w[3] = 1 + (int)rmod(3);                                         /* gain factor */
  if (variant & 4) w[5] = 1 + (int)rmod(2);                                         /* FIFO frames */
  if (variant & 8) w[7] = (int)rnd();
  if (variant & 16) {
    w[2] = (int)rnd();
    w[4] = (int)rnd();
  }
}

/* ============================================================================================ scenario 0: tables */
static void scen0(void)
{
  int i;
  register_tables();
  emit(0, "TAB counts alt %u %u eq %u %u frm %u %u nocarr %u %u", V27TX_ALT_COUNT[0], V27TX_ALT_COUNT[1], V27TX_EQCOND_COUNT[0], V27TX_EQCOND_COUNT[1], V27TX_FRMSIZE[0], V27TX_FRMSIZE[1],
       V27TX_NOCARR_SYMBOL[0], V27TX_NOCARR_SYMBOL[1]);
  emit(0, "TAB patterns alt %u %u carr %u %u scr1 %u %u", V27TX_PATTERN_ALT[0], V27TX_PATTERN_ALT[1], V27TX_PATTERN_CARR[0], V27TX_PATTERN_CARR[1], V27TX_PATTERN_SCR1[0], V27TX_PATTERN_SCR1[1]);
  emit(0, "TAB pps up %u %u down %u %u len %u %u scale %d %d", V27TX_PPS_UP_FACT[0], V27TX_PPS_UP_FACT[1], V27TX_PPS_DOWN_FACT[0], V27TX_PPS_DOWN_FACT[1], V27TX_PPS_FILT_LEN[0],
       V27TX_PPS_FILT_LEN[1], V27TX_PPS_SCALE[0], V27TX_PPS_SCALE[1]);
  emit(0, "TAB smc adj %u %u len %u %u mask %u %u", V27TX_SMC_CRR_ADJ[0], V27TX_SMC_CRR_ADJ[1], V27TX_SMC_CRR_LEN[0], V27TX_SMC_CRR_LEN[1], V27TX_SMC_PHS_MASK[0], V27TX_SMC_PHS_MASK[1]);
  emit(0, "TAB sdm bits %u %u", V27TX_SDM_NUM_BITS[0], V27TX_SDM_NUM_BITS[1]);
  emit(0, "TAB pmap24 %016llx pmap48 %016llx", fnv_shorts((const short *)V27TX_SMC_PMAP_24, 4), fnv_shorts((const short *)V27TX_SMC_PMAP_48, 8));
  emit(0, "TAB imap %016llx %016llx qmap %016llx %016llx", fnv_shorts(V27TX_PPS_IMAP_2400, 5), fnv_shorts(V27TX_PPS_IMAP_4800, 9), fnv_shorts(V27TX_PPS_QMAP_2400, 5), fnv_shorts(V27TX_PPS_QMAP_4800, 9));
  emit(0, "TAB ifilt %016llx %016llx qfilt %016llx %016llx", fnv_shorts(V27TX_PPS_IFILT_2400, 120), fnv_shorts(V27TX_PPS_IFILT_4800, 60), fnv_shorts(V27TX_PPS_QFILT_2400, 120),
       fnv_shorts(V27TX_PPS_QFILT_4800, 60));
  for (i = 0; i < 2; i++) {
    char a[80], b[80], c[80], d[80], e[80];
    emit(0, "TAB ptr %d imap=%s qmap=%s ifilt=%s qfilt=%s pmap=%s", i, dptr(V27TX_PPS_IMAP[i], a, 80), dptr(V27TX_PPS_QMAP[i], b, 80), dptr(V27TX_PPS_IFILT[i], c, 80),
         dptr(V27TX_PPS_QFILT[i], d, 80), dptr(V27TX_SMC_PMAP[i], e, 80));
  }
  emit(0, "TAB CFG %08x %08x %08x %08x %08x %08x %08x %08x", u(V27TX_CFG[0]), u(V27TX_CFG[1]), u(V27TX_CFG[2]), u(V27TX_CFG[3]), u(V27TX_CFG[4]), u(V27TX_CFG[5]), u(V27TX_CFG[6]), u(V27TX_CFG[7]));
  emit(0, "TAB CTL %08x %08x %08x %08x %08x", u(V27TX_CTL[0]), u(V27TX_CTL[1]), u(V27TX_CTL[2]), u(V27TX_CTL[3]), u(V27TX_CTL[4]));
  cov_hit(CV_TAB);
  {
    FaxVmiV27 m;
    memset(&m, 0, sizeof m);
    for (i = -3; i < 13; i++) {
      const char *s = "unset";
      v27tx_message(&m, i, &s);
      if (i == 8) {
        /* the blob reads the first word of V27TX_CTL (0x2580), the port stores NULL: one common line, the blob's value is checked */
        if (SIDE_BLOB && (unsigned)(uintptr_t)s != u(V27TX_CTL[0])) die3("blob v27tx_message(8) != V27TX_CTL[0]");
        if (!SIDE_BLOB && s != NULL) die3("port v27tx_message(8) != NULL");
        emit(1, "MSG %d = (CTL word 0 in the blob, NULL in the port)", i);
        cov_hit(CV_MSG_OOB);
      } else {
        emit(0, "MSG %d = %s", i, s ? s : "NULL");
        if (i >= 0 && i <= 7) cov_hit(CV_MSG); else cov_hit(CV_MSG_NEG);
      }
    }
    for (i = 0; i < 9; i++) {
      const char *s = 0;
      v27tx_message(&m, -256 + i, &s); /* a negative index whose low byte is a valid index: NULL (unsigned compare) */
      emit(0, "MSG %d = %s", -256 + i, s ? s : "NULL");
      s = 0;
      v27tx_message(&m, 0x100 + i, &s);
      emit(0, "MSG %d = %s", 0x100 + i, s ? s : "NULL");
    }
  }
}

/* ============================================================================================ scenario 1: lifecycle */
static void probe_create(int failk, int rate)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    int w[8];
    g_fail_malloc = failk;
    make_cfg(w, rate, 0);
    V27TX_create(NULL, w);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  /* a failed allocation: the blob and the port crash or survive alike; the outcome is one line.  Exception: malloc #2 is the SGD object inside SGD_create (R18): the blob stores through the NULL
   * pointer (SIGSEGV), the port's SGD_create returns NULL (a deliberate guard of src/fax/fax_fifo_sgd.c), so V27TX_create survives; the line is common, the sides are checked */
  if (failk == 2) {
    if (SIDE_BLOB != (WIFSIGNALED(st) != 0)) die3("probe #2: the blob must crash and the port survive");
    emit(1, "PROBE create malloc #%d fails (rate %d): (SGD_create guard: blob SIGSEGV, port survives)", failk, rate);
    return;
  }
  if (WIFSIGNALED(st)) emit(1, "PROBE create malloc #%d fails (rate %d): signal %d", failk, rate, WTERMSIG(st));
  else emit(1, "PROBE create malloc #%d fails (rate %d): exit %d", failk, rate, WEXITSTATUS(st));
}
/* a caller-supplied object without a bundle: the bundle is allocated but the PPS init is told `fresh = 0` and reads the (garbage) taps of the new bundle */
static void probe_supplied(void)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    V27Tx *o;
    if (freopen("/dev/null", "w", stderr) == NULL) _exit(9); /* the garbage free() is a harness violation (exit 3) in the child: no noise */
    o = (V27Tx *)sysdep_malloc(sizeof(V27Tx));
    o->hdx = NULL;
    o->bundle = NULL;
    V27TX_create(o, NULL);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE create of a supplied object without bundle: signal %d", WTERMSIG(st));
  else emit(1, "PROBE create of a supplied object without bundle: exit %d", WEXITSTATUS(st));
  cov_hit(CV_L_SUPNOBUNDLE);
}
/* the re-create over a live engine with another rate: the circular buffer (allocated for the old frame size) is cleared for the NEW size: an overrun when the rate goes up */
static void probe_rate_change(int from, int to)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    int w[8];
    V27Tx *o;
    make_cfg(w, from, 0);
    o = V27TX_create(NULL, w);
    make_cfg(w, to, 0);
    V27TX_create(o, w);
    _exit(redzones_dirty() ? 11 : 12);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE rate change %d -> %d: signal %d", from, to, WTERMSIG(st));
  else emit(1, "PROBE rate change %d -> %d: %s", from, to, WEXITSTATUS(st) == 11 ? "wrote outside a block" : "clean");
  cov_hit(CV_L_OVERRUN);
}
static void run_some(V27Tx *o)
{
  int i, calls = 3 + (int)rmod(6);
  for (i = 0; i < calls; i++) {
    unsigned short n = (unsigned short)(o->hdx->rate ? 32 : 24);
    int ret;
    sym_fill(g_inb, 64, o->hdx->rate);
    ret = V27TX_modem(o, g_inb, g_out, &n);
    emit(0, "life run %d ret=%08x n=%d out=%016llx", i, u(ret), n, fnv_n(g_out, n + 4));
  }
  d_tx("ran", o);
  cov_hit(CV_L_RUN);
}
static void scen1(void)
{
  int trial, k;
  register_tables();
  for (trial = 0; trial < 240; trial++) {
    static const int rates[6] = {0, 1, 0x1111, 0, 1, 0x2580};
    int rate = rates[trial % 6];
    int mode = (trial / 6) % 7;
    int dbg = (trial / 42) & 1;
    int variant = (int)(rnd() & 31u);
    int w[8];
    V27Tx *o = NULL;
    make_cfg(w, rate, trial % 3 == 2 ? variant : trial % 3);
    dbg_level(dbg ? 2 : 0);
    if (rate == 0) cov_hit(CV_L_R2400); else if (rate == 1) cov_hit(CV_L_R4800); else cov_hit(CV_L_RBAD);
    if (dbg) cov_hit(CV_L_DBG);
    if (w[6]) cov_hit(CV_L_S0E0); else cov_hit(CV_L_S0E1);
    switch (mode) {
      case 0:
        stack_poison();
        o = V27TX_create(NULL, w);
        cov_hit(CV_L_NEW);
        break;
      case 1:
        stack_poison();
        o = V27TX_create(NULL, NULL);
        cov_hit(CV_L_NEW);
        cov_hit(CV_L_NULLCFG);
        break;
      case 2:
      case 5: {
        int w0[8];
        V27Tx *s;
        make_cfg(w0, rate == 1 ? 1 : 0, 0);
        s = V27TX_create(NULL, w0);
        d_tx("first", s);
        if (mode == 2) { /* hdx dropped: the create allocates a new one (the old pieces are freed here) */
          FIFO_delete(s->hdx->fifo);
          SGD_delete(s->hdx->sgd);
          sysdep_free(s->hdx);
          s->hdx = NULL;
        }
        stack_poison();
        o = V27TX_create(s, w0);
        if (o != s) die3("supplied object not returned");
        cov_hit(mode == 2 ? CV_L_SUP_NOHDX : CV_L_SUP_EMPTY);
        if (mode == 5) {
          /* the second create goes with the test rate when it does not change the frame size (a changing size overruns the circular buffer: probed below) */
          int same = (rate == 0 || rate == 1) ? 0 : (rate == 1);
          (void)same;
        }
        break;
      }
      case 3: {
        int w0[8];
        V27Tx *s;
        make_cfg(w0, rate == 1 ? 1 : 0, 0);
        s = V27TX_create(NULL, w0);
        d_tx("first", s);
        if (rate == 1) w0[3] = 2;
        w0[6] = w[6];
        o = V27TX_create(s, w0);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_SUP_LIVE);
        break;
      }
      case 4: {
        V27Tx *s;
        make_cfg(w, rate == 1 ? 1 : 0, variant & ~1);
        s = V27TX_create(NULL, w);
        d_tx("first", s);
        s->cfg.w[2] = 777;
        s->cfg.w[6] = (int)(rnd() & 1u);
        o = V27TX_create(s, s->cfg.w);
        cov_hit(CV_L_RECREATE);
        break;
      }
      default: { /* a re-create that GROWS the pulse shaper (2400 -> 4800 taps 6 -> 12 would also grow the frame: done in a probe) and one that shrinks it */
        int w0[8], w1[8];
        V27Tx *s;
        make_cfg(w0, 1, 0);
        s = V27TX_create(NULL, w0);
        d_tx("first", s);
        make_cfg(w1, 1, 2);
        o = V27TX_create(s, w1);
        cov_hit(CV_L_SUP_LIVE);
        break;
      }
    }
    dbg_level(0);
    d_tx("created", o);
    if (trial & 1) run_some(o);
    V27TX_delete(o);
    cov_hit(CV_L_DEL);
    trial_end("life");
  }
  /* PPS re-allocation: fresh = 0 with a growing / shrinking tap count.  The frame size changes with the rate, so only the PPS object is re-inited on its own (the typed bundle is public) */
  for (k = 0; k < 4; k++) {
    int w0[8], w1[8];
    V27Tx *o;
    dbg_level(k & 1 ? 2 : 0);
    make_cfg(w0, k & 1, 0);
    make_cfg(w1, (k >> 1) & 1, 0);
    o = V27TX_create(NULL, w0);
    {
      /* re-initialise the pulse shaper alone with the other rate's config through a second engine's cfg: V27TX_create on a supplied engine would also resize the circular buffer */
      V27Tx *p = V27TX_create(NULL, w1);
      V27TxBundle *ob = o->bundle;
      FaxPps *pp = &p->bundle->pps;
      (void)ob;
      (void)pp;
      d_tx("second", p);
      V27TX_delete(p);
    }
    d_tx("pps", o);
    V27TX_delete(o);
    trial_end("life pps");
  }
  {
    /* taps growth / shrink through create(o, cfg) on a LIVE engine where the frame size does not change is impossible (rate decides both), so the tap logic is covered by the rate-change
     * engines below, whose circular buffer overrun is confined to a forked child; here the PPS re-init runs on engines of the SAME rate (taps equal: buffers kept, zeroed) */
    int w0[8];
    V27Tx *o;
    dbg_level(2);
    make_cfg(w0, 0, 0);
    o = V27TX_create(NULL, w0);
    run_some(o);
    V27TX_create(o, w0);
    d_tx("same rate re-create", o);
    cov_hit(CV_L_NOREALLOC);
    V27TX_delete(o);
    dbg_level(0);
    trial_end("life same");
  }
  probe_supplied();
  for (k = 0; k < 12; k++) {
    probe_create(k, k & 1);
    cov_hit(CV_L_PROBE);
  }
  probe_rate_change(0, 1);
  probe_rate_change(1, 0);
  probe_rate_change(0, 0);
  probe_rate_change(1, 1);
  for (k = 1; k <= 2; k++) {
    /* the pulse-shaper re-allocation: a forked child re-creates a 2400 engine as 4800 (taps 6 -> 12: "Reallocate FPM_PPS buffer") and dumps the PPS words and the freed / new block ids */
    int st = 0;
    pid_t pid;
    fflush(stdout);
    pid = fork();
    if (pid == 0) {
      int w0[8];
      V27Tx *o;
      make_cfg(w0, 0, 0);
      o = V27TX_create(NULL, w0);
      make_cfg(w0, 1, 0);
      dbg_level((unsigned)k); /* level 1: no text, level 2: the "Reallocate FPM_PPS buffer" line */
      g_verbose = 1; /* the child's own lines (allocator calls, the "Reallocate" text, the engine dump) are part of the output */
      V27TX_create(o, w0);
      dbg_level(0);
      d_tx("grown", o);
      g_verbose = 0;
      fflush(stdout);
      _exit(redzones_dirty() ? 11 : 12);
    }
    if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
    cov_hit(CV_L_REALLOC);
    emit(1, "PROBE pps growth child (debug level %d): %s %d", k, WIFSIGNALED(st) ? "signal" : "exit", WIFSIGNALED(st) ? WTERMSIG(st) : WEXITSTATUS(st));
  }
  {
    V27Tx *s;
    int w0[8];
    make_cfg(w0, 1, 0);
    s = V27TX_create(NULL, w0);
    g_fail_malloc = 0;
    V27TX_create(s, NULL); /* a supplied live engine allocates nothing (default cfg: rate 9600, error flags) */
    d_tx("recreate no malloc", s);
    V27TX_delete(s);
    trial_end("life fail");
  }
}

/* ============================================================================================ scenario 2: transmit streams */
static void observe(const V27Tx *o, int *prev, int *bits_seen)
{
  int s = o->hdx->state;
  if (s == 1) cov_hit(CV_S_QUIET);
  if (s == 2) cov_hit(CV_S_CARR);
  if (s == 3) cov_hit(CV_S_NOCARR);
  if (s == 4) cov_hit(CV_S_ALT);
  if (s == 5) cov_hit(CV_S_EQ);
  if (s == 6) cov_hit(CV_S_SCR1);
  if (s == 7) cov_hit(CV_S_DATA);
  if (s == 8 || s == 9) cov_hit(CV_S_TURNOFF);
  if (s == 10) cov_hit(CV_S_IDLE);
  if (*prev == 6 && s == 7) cov_hit(CV_S_TO_DATA);
  if (*prev != 10 && s == 10) cov_hit(CV_S_TO_IDLE);
  *prev = s;
  (void)bits_seen;
}
static void scen2(void)
{
  int trial;
  register_tables();
  dbg_level(0);
  for (trial = 0; trial < 64; trial++) {
    int rate = trial & 1;
    int variant = (trial / 2) % 6;
    int pol = (trial / 12) % 3;
    int w[8], i, calls, tot = 0, prev = -1, frm = rate ? 32 : 24;
    unsigned long long oh = 0;
    V27Tx *o;
    int dbg = trial % 9 == 4;
    if (trial % 17 == 5) rate = 0x1111;
    make_cfg(w, rate, variant == 0 ? 1 : variant == 1 ? 1 | 2 : variant == 2 ? 1 | 4 : variant == 3 ? 0 : variant == 4 ? 1 | 2 | 4 | 8 | 16 : 1);
    if (rate == 0) cov_hit(CV_S_R2400); else if (rate == 1) cov_hit(CV_S_R4800); else cov_hit(CV_S_BADRATE);
    if (w[6]) cov_hit(CV_S_S0E0); else cov_hit(CV_S_S0E1);
    if (w[5] == 2) cov_hit(CV_S_BIGFIFO);
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_S_DBG);
    stack_poison();
    o = V27TX_create(NULL, trial % 11 == 7 ? NULL : w);
    d_tx("created", o);
    if (trial % 11 == 7) {
      rate = 1;
      frm = 32;
    }
    calls = 380 + (int)rmod(260);
    for (i = 0; i < calls; i++) {
      int n = pol == 0 ? frm : pol == 1 ? (rpct(70) ? frm : (int)rmod((unsigned)(frm + 9))) : (rpct(50) ? frm : rpct(50) ? 2 * frm : (int)rmod((unsigned)frm));
      unsigned short nn;
      int ret, k;
      if (rpct(3)) n = 0;
      if (n == 0) cov_hit(CV_S_N0);
      else if (n == frm) cov_hit(CV_S_NFULL);
      else if (n > frm) cov_hit(CV_S_NLONG);
      else cov_hit(CV_S_NSHORT);
      sym_fill(g_inb, 4096, rate == 1 ? 1 : 0);
      for (k = 0; k < 8192; k++) g_out[k] = 0x7777;
      nn = (unsigned short)n;
      ret = V27TX_modem(o, g_inb, g_out, &nn);
      emit(0, "tx call %d n=%d ret=%08x outn=%d in=%016llx out=%016llx", i, n, u(ret), nn, fnv_n(g_inb, 80), fnv_n(g_out, 8192));
      oh ^= fnv_n(g_out, nn);
      tot += nn;
      d_tx("tx", o);
      observe(o, &prev, &k);
      if (o->st.b[0] == 7) cov_hit(CV_S_OVER);
      if (o->st.b[0] == 6) cov_hit(CV_S_UNDER);
      if (rpct(5)) {
        FaxV27Status st;
        FaxV27Ctl ctl;
        memset(&st, 0x55, sizeof st);
        emit(0, "tx status ret=%d", V27TX_status(o, &st));
        emit(0, "tx status %d %d %d %d %d %d %d %d %d %d %02x %02x %08x", st.s0, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15, u(st.w18));
        cov_hit(CV_S_STATUS);
        memset(&ctl, 0, sizeof ctl);
        ctl.w4 = (int)rnd();
        ctl.w8 = rpct(50) ? 0x4000 : (int)(rnd() & 0xffff);
        ctl.w10 = rpct(70) ? o->cfg.w[6] : (int)(rnd() & 1u);
        ctl.b0c = (unsigned char)(rpct(50) ? 4 : 0);
        ctl.b0d = (unsigned char)((rpct(i > 150 ? 35 : 10) ? 0x10 : 0) | (rpct(3) ? 2 : 0));
        if (ctl.b0d & 0x10) cov_hit(CV_S_PEND);
        cov_hit(CV_S_GAIN);
        if (ctl.b0d & 2) cov_hit(CV_S_RECREATE);
        emit(0, "tx control ret=%d", V27TX_control(o, &ctl));
        d_tx("ctl", o);
      }
      if (o->hdx->pend && rpct(25)) {
        FaxV27Ctl ctl;
        memset(&ctl, 0, sizeof ctl);
        ctl.w8 = 0x4000;
        ctl.w10 = o->cfg.w[6];
        ctl.w4 = o->cfg.w[2];
        V27TX_control(o, &ctl); /* pend off again */
      }
    }
    emit(0, "tx trial %d rate %d variant %d: samples=%d outhash=%016llx", trial, rate, variant, tot, oh);
    if (trial % 8 == 0) emit(1, "TX trial %d rate %d: %d samples", trial, rate, tot);
    dbg_level(0);
    V27TX_delete(o);
    trial_end("tx run");
  }
}

/* ============================================================================================ scenario 3: handlers */
static V27TxHdxFn pick_fn(int sel)
{
  switch (sel) {
    case 0: return TxHdxStartV27;
    case 1: return TxHdxQuietV27;
    case 2: return TxHdxEQCondV27;
    case 3: return TxHdxSCR1V27;
    case 4: return TxHdxAltV27;
    case 5: return TxHdxDataV27;
    default: return TxHdxIdleV27;
  }
}
static void scen3(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 6000; trial++) {
    static const int sts[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, -1, 11, 0x7fff};
    static const int cnts[9] = {0, 1, 5, 24, 32, -1, -32768, 400, 0x7fff};
    static const int ns[8] = {0, 1, 3, 8, 24, 32, 45, 6};
    int fnsel = trial % 7, rate = (trial / 7) & 1, stsel = (trial / 14) % 14, cn = (trial / 196) % 9, fill = (trial / 1764) % 3, pend = (int)(rnd() & 1u);
    int ns_i = (int)(rnd() % 8u), dbg = trial % 29 == 3;
    int w[8], i, ret;
    V27Tx *o;
    unsigned short n;
    int nf;
    make_cfg(w, rate, trial % 4 == 3 ? 1 : 0);
    o = V27TX_create(NULL, w);
    nf = fill == 0 ? 0 : fill == 1 ? 3 : (rate ? 32 : 24);
    sym_fill(g_inb, 4096, rate);
    for (i = 0; i < 16; i++) g_inb[i] = (short)(0x6b00 + i); /* a sentinel in front of the block the handler gets (g_inb + 16) */
    if (nf) FIFO_write(o->hdx->fifo, (const unsigned short *)(g_inb + 16), (unsigned short)nf);
    cov_hit(nf ? CV_H_FIFOF : CV_H_FIFO0);
    o->hdx->state = (short)sts[stsel];
    o->hdx->count = (short)cnts[cn];
    o->hdx->pend = pend;
    o->hdx->longTrain = (short)(rnd() & 1u);
    if (pend) cov_hit(CV_H_PEND);
    o->st.w = (int)rnd();
    if (stsel > 10) cov_hit(CV_H_STATEDEF);
    if (cnts[cn] == 0) cov_hit(CV_H_COUNT0);
    if (cnts[cn] < 0) cov_hit(CV_H_COUNTNEG);
    n = (unsigned short)ns[ns_i];
    if (n == 0) cov_hit(CV_H_N0);
    if (cnts[cn] > ns[ns_i]) cov_hit(CV_H_COUNTBIG);
    if (cnts[cn] > 0 && cnts[cn] < ns[ns_i]) cov_hit(CV_H_NSMALL);
    sym_fill(g_inb + 16, 4000, rate);
    for (i = 0; i < 8192; i++) g_out[i] = 0x7777;
    if (fnsel == 5 && trial % 5 == 0) { /* Data with odd rate words (state byte 2 / 3 / 5) */
      static const int odd[5] = {0, 1, 2, -1, 0x7fff};
      o->hdx->rate = (short)odd[(trial / 5) % 5];
      if (o->hdx->rate != 0 && o->hdx->rate != 1) o->hdx->pend = 0; /* pend would run TxNextStateV27, which indexes the pattern tables with the rate word (out of the table: layout dependent) */
      if (o->hdx->rate != 0 && o->hdx->rate != 1) cov_hit(CV_H_RATEODD);
      if (cnts[cn] == 0) o->hdx->count = 1;
    }
    dbg_level(dbg ? 2 : 0);
    d_tx("pre", o);
    stack_poison();
    o->hdx->fn = pick_fn(fnsel);
    switch (fnsel) {
      case 0: ret = TxHdxStartV27(o, g_inb + 16, g_out, &n); cov_hit(CV_H_START); break;
      case 1: ret = TxHdxQuietV27(o, g_inb + 16, g_out, &n); cov_hit(CV_H_QUIET); break;
      case 2: ret = TxHdxEQCondV27(o, g_inb + 16, g_out, &n); cov_hit(CV_H_EQ); break;
      case 3: ret = TxHdxSCR1V27(o, g_inb + 16, g_out, &n); cov_hit(CV_H_SCR1); break;
      case 4: ret = TxHdxAltV27(o, g_inb + 16, g_out, &n); cov_hit(CV_H_ALT); break;
      case 5:
        ret = TxHdxDataV27(o, g_inb + 16, g_out, &n);
        cov_hit(CV_H_DATA);
        if ((o->st.b[0] == 6) && !pend) cov_hit(CV_H_UNDER);
        if (pend && nf < ns[ns_i]) cov_hit(CV_H_UNDERPEND);
        break;
      default: ret = TxHdxIdleV27(o, g_inb + 16, g_out, &n); cov_hit(CV_H_IDLE); break;
    }
    dbg_level(0);
    emit(0, "hdx trial %d fn %d rate %d st %d cnt %d fill %d pend %d n %d: ret=%d n=%d in=%016llx out=%016llx", trial, fnsel, rate, sts[stsel], cnts[cn], fill, pend, ns[ns_i], ret, n,
         fnv_n(g_inb, 116), fnv_n(g_out, 1024));
    d_tx("post", o);
    V27TX_delete(o);
    trial_end("hdx");
  }
}

/* ============================================================================================ scenario 4: next state, modulator, no carrier */
static void scen4(void)
{
  int trial, st, rate, s0e, dbg;
  register_tables();
  for (trial = 0; trial < 14 * 2 * 2 * 2 * 3; trial++) {
    static const int sts[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, -1, 11, 0x7fff};
    int w[8];
    V27Tx *o;
    st = sts[trial % 14];
    rate = (trial / 14) & 1;
    s0e = (trial / 28) & 1;
    dbg = (trial / 56) & 1;
    make_cfg(w, rate, 0);
    o = V27TX_create(NULL, w);
    o->hdx->state = (short)st;
    o->hdx->longTrain = (short)s0e;
    o->hdx->count = (short)(int)(rnd() & 0x7fu);
    o->st.w = (int)rnd();
    sym_fill(g_inb, 64, rate);
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_N_DBG);
    d_tx("pre", o);
    stack_poison();
    TxNextStateV27(o);
    dbg_level(0);
    d_tx("next", o);
    switch (st) {
      case 0: cov_hit(CV_N_C0); break;
      case 1: cov_hit(CV_N_C1); break;
      case 2: cov_hit(CV_N_C2); break;
      case 3: cov_hit(CV_N_C3); break;
      case 4: cov_hit(CV_N_C4); break;
      case 5: cov_hit(CV_N_C5); break;
      case 6: cov_hit(CV_N_C6); break;
      case 7: cov_hit(CV_N_C7); break;
      case 8: cov_hit(CV_N_C8); break;
      case 9: cov_hit(CV_N_C9); break;
      case 10: cov_hit(CV_N_C10); break;
      default: cov_hit(CV_N_DEFAULT); break;
    }
    /* the machine after the transition: one handler call moves the state on again */
    if (o->hdx->fn && st >= 0 && st <= 10) {
      unsigned short n = (unsigned short)(1 + rmod(40));
      int ret;
      int k;
      for (k = 0; k < 4096; k++) g_out[k] = 0x7777;
      ret = o->hdx->fn(o, g_inb, g_out, &n);
      emit(0, "next+handler ret=%d n=%d out=%016llx", ret, n, fnv_n(g_out, 1024));
      d_tx("after", o);
    }
    V27TX_delete(o);
    trial_end("next");
  }
  for (trial = 0; trial < 500; trial++) {
    int w[8], n, k, ret;
    V27Tx *o;
    rate = trial & 1;
    make_cfg(w, rate, trial % 5 == 4 ? 2 : 0);
    o = V27TX_create(NULL, w);
    n = (int)(trial % 7 == 0 ? rmod(6) : rmod(80));
    if (trial % 4 == 0) {
      o->bundle->circ.wr = (short)rmod((unsigned)o->bundle->circ.lim);
      o->bundle->circ.rd = (short)rmod((unsigned)o->bundle->circ.lim);
    }
    if (n + 2 > o->bundle->circ.lim) cov_hit(CV_N_WRAP);
    if (trial % 5 == 1) { /* the pulse shaper's direct mode: I / Q taken from two sample buffers (never used by V.27ter itself: the mode word is 1) and a phase step */
      int q;
      for (q = 0; q < 64; q++) {
        g_ibuf[q] = (short)((int)(rnd() & 0xffffu) - 32768);
        g_qbuf[q] = (short)((int)(rnd() & 0xffffu) - 32768);
      }
      o->bundle->pps.cfg.mode = 0;
      o->bundle->pps.cfg.step = (short)(trial % 3);
      o->bundle->circ.i = g_ibuf;
      o->bundle->circ.q = g_qbuf;
      cov_hit(CV_N_MODE0);
    }
    sym_fill(g_inb, 100, rate);
    for (k = 0; k < 8192; k++) g_out[k] = 0x7777;
    d_tx("mpre", o);
    if (trial & 2) {
      ret = ModDataV27(o, g_inb, g_out, (unsigned short)n);
      emit(0, "moddata trial %d n=%d ret=%d out=%016llx", trial, n, ret, fnv_n(g_out, 1024));
      cov_hit(CV_N_MOD);
    } else {
      ret = TxNoCarrierV27(o, g_inb, g_out, (unsigned short)n);
      emit(0, "nocarrier trial %d n=%d ret=%d out=%016llx", trial, n, ret, fnv_n(g_out, 1024));
      cov_hit(CV_N_NOCAR);
    }
    d_tx("mpost", o);
    if (trial % 3 == 0) { /* both in a row: the PPS history / accumulator carry over */
      ret = ModDataV27(o, g_inb, g_out, (unsigned short)(n / 2));
      emit(0, "moddata2 ret=%d out=%016llx", ret, fnv_n(g_out, 1024));
      ret = TxNoCarrierV27(o, g_inb, g_out, (unsigned short)(n / 3));
      emit(0, "nocarrier2 ret=%d out=%016llx", ret, fnv_n(g_out, 1024));
      d_tx("mpost2", o);
    }
    V27TX_delete(o);
    trial_end("mod");
  }
  {
    /* a long direct-mode run of the pulse shaper (random 16 bit I / Q, phase step 0 .. 2): the filter sums land on rounding boundaries often enough to see an offset of one in a sum */
    int e;
    for (e = 0; e < 4; e++) {
      int w[8], blk;
      V27Tx *o;
      unsigned long long oh = 0;
      make_cfg(w, e & 1, 0);
      o = V27TX_create(NULL, w);
      o->bundle->pps.cfg.mode = 0;
      o->bundle->pps.cfg.step = (short)(e >> 1);
      o->bundle->circ.i = g_ibuf;
      o->bundle->circ.q = g_qbuf;
      for (blk = 0; blk < 2200; blk++) {
        int q, n = 1 + (int)rmod(40);
        for (q = 0; q < 64; q++) {
          g_ibuf[q] = (short)((int)(rnd() & 0xffffu) - 32768);
          g_qbuf[q] = (short)((int)(rnd() & 0xffffu) - 32768);
        }
        o->bundle->circ.rd = (short)rmod((unsigned)o->bundle->circ.lim);
        oh = oh * 1099511628211ULL ^ (unsigned long long)TxNoCarrierV27(o, g_inb, g_out, (unsigned short)n) ^ fnv_n(g_out, 400);
      }
      emit(0, "mode0 run %d: %016llx", e, oh);
      d_tx("mode0", o);
      V27TX_delete(o);
      trial_end("mode0");
    }
  }
  (void)s0e;
}

/* ============================================================================================ scenario 5: dispatcher, control, status */
static int g_fake_calls, g_fake_cons[16], g_fake_ret[16];
static short *g_fake_in, *g_fake_out;
static int fake_tx(V27Tx *o, short *in, short *out, unsigned short *n)
{
  int k = g_fake_calls++;
  int c = g_fake_cons[k & 15], r = g_fake_ret[k & 15];
  (void)o;
  emit(0, "fake tx call %d: in+%d out+%d n=%d -> consume %d ret %d", k, (int)(in - g_fake_in), (int)(out - g_fake_out), (short)*n, c, r);
  *n = (unsigned short)(*n - c);
  return r;
}
static void scen5(void)
{
  int t, i, f;
  register_tables();
  for (t = 0; t < 600; t++) {
    int w[8], ret, rate = t & 1;
    V27Tx *o;
    unsigned short n;
    int nn = (int)rmod(40);
    make_cfg(w, rate, 0);
    o = V27TX_create(NULL, w);
    for (i = 0; i < 16; i++) {
      g_fake_cons[i] = i >= 14 ? 1000 : (int)rmod(rate ? 17 : 13) - (t % 3 == 0 ? 0 : 1);
      g_fake_ret[i] = (int)rmod(41) - (t % 4 == 0 ? 20 : 0);
      if (g_fake_ret[i] < 0) cov_hit(CV_D_NEGRET);
    }
    g_fake_calls = 0;
    g_fake_in = &g_inb[0];
    g_fake_out = &g_out[0];
    sym_fill(g_inb, 64, rate);
    o->hdx->fn = fake_tx;
    o->hdx->pend = t & 2 ? 1 : 0;
    cov_hit(o->hdx->pend ? CV_D_PEND : CV_D_NOPEND);
    if (t % 5 < 2) FIFO_write(o->hdx->fifo, (const unsigned short *)g_inb, (unsigned short)(1 + t % 6));
    o->st.w = (int)rnd();
    n = (unsigned short)nn;
    ret = V27TX_modem(o, g_inb, &g_out[200], &n);
    emit(0, "txdisp t=%d nn=%d: ret=%08x n=%d calls=%d st=%08x", t, nn, u(ret), n, g_fake_calls, u(o->st.w));
    d_fifo("disp", o->hdx->fifo);
    if (o->st.b[0] == 7) cov_hit(CV_D_OVER);
    cov_hit(CV_D_FAKE);
    V27TX_delete(o);
    trial_end("tx disp");
  }
  for (f = 0; f < 512; f++) {
    int w[8], k;
    V27Tx *o;
    FaxV27Ctl ctl;
    unsigned short n = 12;
    make_cfg(w, f & 1, 0);
    o = V27TX_create(NULL, w);
    o->hdx->pend = (int)(rnd() & 3u);
    o->hdx->state = (short)(f & 3);
    memset(&ctl, 0, sizeof ctl);
    ctl.w4 = 100 + f;
    ctl.w8 = f % 5 == 0 ? 0x4000 : f % 5 == 1 ? 0x7fffffff : (int)rnd();
    ctl.w10 = (int)rnd();
    if (f < 256) {
      ctl.b0d = (unsigned char)f;
      ctl.b0c = (unsigned char)rnd();
    } else {
      ctl.b0c = (unsigned char)(f - 256);
      ctl.b0d = (unsigned char)(rnd() & 0xfdu);
    }
    emit(0, "txctl f=%d: ret=%d", f, V27TX_control(o, &ctl));
    cov_hit(CV_D_CTL);
    if (ctl.b0d & 0x10) cov_hit(CV_D_CTL_PEND);
    if (ctl.b0d & 2) cov_hit(CV_D_CTL_RECREATE);
    if (ctl.b0c & 4) cov_hit(CV_D_CTL_B0C);
    cov_hit(CV_D_CTL_GAIN);
    d_tx("ctl", o);
    /* the gain shows in the modulated samples */
    sym_fill(g_inb, 64, f & 1);
    for (k = 0; k < 8192; k++) g_out[k] = 0x7777;
    o->hdx->state = 0;
    o->hdx->fn = TxHdxAltV27;
    o->hdx->count = 40;
    emit(0, "txctl run ret=%d n=%d out=%016llx", TxHdxAltV27(o, g_inb, g_out, &n), n, fnv_n(g_out, 1024));
    emit(0, "txctl null: ret=%d", V27TX_control(o, NULL));
    cov_hit(CV_D_CTL_NULL);
    V27TX_delete(o);
    trial_end("tx ctl");
  }
  for (t = 0; t < 200; t++) {
    int w[8];
    V27Tx *o;
    FaxV27Status st;
    make_cfg(w, t & 1, t % 4 == 2 ? 8 : 0);
    o = V27TX_create(NULL, w);
    o->cfg.b[0x10] = (unsigned char)rnd();
    o->cfg.s[0] = (short)(rpct(50) ? (int)(t & 3) : (int)rnd());
    o->cfg.w[6] = (int)rnd();
    memset(&st, t % 3 == 0 ? 0xff : t % 3 == 1 ? 0x55 : 0x00, sizeof st);
    emit(0, "txstat t=%d ret=%d", t, V27TX_status(o, &st));
    emit(0, "txstat %d %d %d %d %d %d %d %d %d %d %02x %02x %08x", st.s0, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15, u(st.w18));
    cov_hit(CV_D_STAT);
    emit(0, "txstat null ret=%d", V27TX_status(o, NULL));
    cov_hit(CV_D_STAT_NULL);
    V27TX_delete(o);
    trial_end("tx stat");
  }
}

/* ============================================================================================ scenario 6: VMI wrappers */
static void d_vmi(const char *t, const FaxVmiV27 *m, int created)
{
  /* u4: the integer 1 after the init (blob and port), the bit buffer pointer afterwards; u10: the config block pointer after the init, its low half is zeroed by v27tx_create (print the half) */
  if (!created) emit(0, "%s VMI w0=%d u4=%lu s08=%d s0a=%d s0c=%d s0e=%d cfg=blk%u eng=%s", t, m->w0, (unsigned long)m->u4.init1, m->s08, m->s0a, m->s0c, m->s0e, blk_id_of(m->u10.cfg), m->eng ? "set" : "NULL");
  else emit(0, "%s VMI w0=%d s08=%d s0a=%d s0c=%d s0e=%d s10=%d eng=%s", t, m->w0, m->s08, m->s0a, m->s0c, m->s0e, m->u10.s10, m->eng ? "set" : "NULL");
}
static void scen6(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 40; trial++) {
    FaxVmiV27 m;
    int supplied = trial % 5 == 3;
    int w[8], rate = supplied ? 1 : trial & 1, c, ret, i, calls, dbglev = trial % 4; /* a supplied engine is re-created with the SAME frame size (rate index 1: 4800 or the default 9600) */
    unsigned short n, on;
    short *bitbuf = (short *)sysdep_malloc(4096 * 2);
    short *out = (short *)sysdep_malloc(8192 * 2);
    V27Tx *user = NULL;
    unsigned long long oh = 0;
    memset(&m, trial & 2 ? 0xa5 : 0x5a, sizeof m);
    if (supplied) {
      int w0[8];
      make_cfg(w0, rate, 0);
      user = V27TX_create(NULL, w0);
    }
    make_cfg(w, rate == 0 ? 0x960 : 0x12c0, supplied ? 0 : trial % 6 >= 3 ? 1 | 4 : 0);
    w[0] = (w[0] & 0xffff) | ((rate == 0 ? 0x960 : 0x12c0) << 16);
    dbg_level((unsigned)dbglev);
    if (dbglev > 2) cov_hit(CV_V_INIT_DBG);
    stack_poison();
    ret = fax_init_vmi_v27tx(&m, (unsigned short)(rate == 0 ? 0x960 : 0x12c0), trial, user);
    dbg_level(0);
    cov_hit(CV_V_INIT);
    emit(0, "init ret=%d", ret);
    d_vmi("init", &m, 0);
    {
      /* the config block copy: the user word (a pointer's low half) is layout dependent, hashed with the word zeroed; its equality with the user word is printed */
      V27Cfg *cb = (V27Cfg *)m.u10.cfg;
      V27Cfg cc;
      memcpy(&cc, cb, sizeof cc);
      emit(0, "init cfg block %08x %08x %08x %08x %08x %08x %08x user=%d", u(cc.w[0]), u(cc.w[1]), u(cc.w[2]), u(cc.w[3]), u(cc.w[4]), u(cc.w[5]), u(cc.w[6]),
           cc.w[7] == (int)(uintptr_t)user);
    }
    m.u4.bits = bitbuf;
    stack_poison();
    ret = v27tx_create(&m, trial % 3 == 0 ? NULL : w); /* NULL cfg: the default rate 9600 (error flags, rate index 1) */
    cov_hit(CV_V_CREATE);
    if (trial % 3 == 0) cov_hit(CV_V_CREATE_NULLCFG);
    if (supplied) cov_hit(CV_V_CREATE_SUP);
    emit(0, "v27tx_create ret=%d", ret);
    d_vmi("created", &m, 1);
    d_tx("eng", (V27Tx *)m.eng);
    cov_hit(((V27Tx *)m.eng)->hdx->rate ? CV_V_R4800 : CV_V_R2400);
    calls = 60 + (int)rmod(40);
    for (i = 0; i < calls; i++) {
      int k;
      n = (unsigned short)(((V27Tx *)m.eng)->hdx->rate ? 32 : 24);
      sym_fill(bitbuf, 64, ((V27Tx *)m.eng)->hdx->rate);
      for (k = 0; k < 8192; k++) out[k] = 0x7777;
      on = 0xabcd;
      ret = v27tx_process(&m, out, &n, &on);
      emit(0, "v27tx_process %d ret=%08x n=%d outn=%d out=%016llx", i, u(ret), n, on, fnv_n(out, 8192));
      oh ^= fnv_n(out, on);
      cov_hit(CV_V_PROC);
    }
    emit(0, "vmi trial %d hash=%016llx", trial, oh);
    {
      FaxV27Status st;
      FaxV27Ctl ctl;
      memset(&st, 0x55, sizeof st);
      emit(0, "v27tx_status ret=%d", v27tx_status(&m, &st));
      emit(0, "st %d %d %d %d %d %d %d %d %d %d %02x %02x %08x", st.s0, st.s2, st.s4, st.s6, st.s8, st.sa, st.sc, st.se, st.s10, st.s12, st.b14, st.b15, u(st.w18));
      cov_hit(CV_V_STATUS);
      memset(&ctl, 0, sizeof ctl);
      ctl.w4 = 1234 + trial;
      ctl.w8 = 0x2000;
      ctl.b0c = 4;
      ctl.b0d = (unsigned char)(trial & 1 ? 0x12 : 0x10);
      ctl.w10 = trial & 4;
      emit(0, "v27tx_control ret=%d", v27tx_control(&m, &ctl));
      emit(0, "v27tx_control null ret=%d", v27tx_control(&m, NULL));
      cov_hit(CV_V_CTL);
      d_tx("eng2", (V27Tx *)m.eng);
    }
    for (c = -2; c < 11; c++) {
      const char *s = "unset";
      v27tx_message(&m, c, &s);
      if (c != 8) emit(0, "vmi msg %d %s", c, s ? s : "NULL");
    }
    cov_hit(CV_V_MSG);
    v27tx_delete(&m);
    cov_hit(CV_V_DEL);
    sysdep_free(bitbuf);
    sysdep_free(out);
    trial_end("vmi");
  }
}

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : -1;
  const char *e;
  if (scen < 0 || scen > 6) {
    fprintf(stderr, "usage: %s scenario(0..6)\n", argv[0]);
    return 2;
  }
  g_verbose = (e = getenv("R20_VERBOSE")) != NULL && *e == '1';
  if ((e = getenv("R20_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(e, NULL, 0);
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
    default: scen6(); break;
  }
  cov_finish();
  return 0;
}
