/*
 * R23 differential harness: the fax V.29 RECEIVER of dsplibs.o (V29RX_create / _delete / _modem / _control / _status, RxHdx{Start,EpochDet,Prtcol,Data,Idle,Error}V29, RxNextStateV29, DemodDataV29,
 * DataCarrierDetectV29 / CarrierDetectV29 / EpochDetectV29 / QualityDetectV29 / GetSNRV29, the FSE callbacks V29RX_epoch_det / _decision / _eq_train, the v29rx_* VMI wrappers and the blob-LOCAL
 * init_vmi_v29rx) vs src/fax/fax_v29_rx.c + tables (the typed FAXFSE_* / FAXSRE_* of R21 are the port's sub-objects; their stand-alone verification is R21's r21_dsp).  The blob run (-DTEST_BLOB, linked with
 * ../dsplibs.o, init_vmi_v29rx made global by objcopy) and the port run (-m32 -O0 / -O2, x86-64 against the recorded blob output) execute the same code; library test/fax/r23_common.h (hashed canonical output,
 * coverage counters exit 2, 3 = harness violation, deterministic heap with block ids / red zones / poisoned frees, callee recorders sysdep_malloc / sysdep_free / dsplibs_debug_printf).  The R18 / R22 / FPM
 * callees are the real ones on both sides (the blob's own in the blob run, the files of src/ in the port run): the transmit signal of the end-to-end streams is made by the blob's V29TX_* in the blob
 * run and by src/fax/fax_v29_tx.c in the port runs (the two are proven identical by R22), so the receiver sees the same samples.  Integer signals only (no libm).  Nothing printed depends on the host layout:
 * objects are dumped member by member (pointers as table+index / block id, the content of every malloc()ed block as a hash).  The hdx.s1c flag (never written by the blob's create: heap garbage) is set
 * explicitly to 0 / 1 right after every create, on both sides.
 *
 * Scenarios (each its own process; argv[1]):
 *   0 tables: every R23 table dumped, the default config words, v29rx_message -3 .. 12 and 0x100 ..;
 *   1 lifecycle: V29RX_create (NULL object / supplied with live hdx + bundle / hdx NULL / bundle NULL / live re-create / create(o, o), NULL / custom / fuzzed cfg words, rates 7200 / 9600 / other, debug level
 *     0 and 2), delete, run-after-create, forked probes: a failing malloc #0 .. #44, a supplied object without bundle, the rate-changing re-creates (red zone verdicts);
 *   2 VMI: fax_init_vmi_v29rx (blob: init_vmi_v29rx) with debug levels 0 .. 3, v29rx_create / process / status / control / delete / message on garbage-initialised FaxVmiV29 objects;
 *   3 receive streams: the transmitter of the same side (V29TX, R22) -> impairments (gain, noise, DC, clock slip, carrier loss, noise bursts, re-trains) -> V29RX_modem in random chunks, both rates, control
 *     (hold, re-create, gates), status, the whole engine dumped after EVERY call;
 *   4 handlers: every RxHdx*V29 / DemodDataV29 / DataCarrierDetectV29 / QualityDetectV29 / RxNextStateV29 x hdx state x rate x poked decision state, on engines warmed up by real signals, sweeps of the
 *     carrier / idle / quality detectors, big-block probes;
 *   5 the FSE callbacks directly (exhaustive sweeps), RxNextStateV29 for every state / rate / debug level, V29RX_modem with a harness handler, V29RX_control for 512 flag-byte combinations, V29RX_status.
 * R23_VERBOSE=1 prints every line (debugging), R23_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r23_common.h"
#include "fax/fax_v29_rx_tables.h"
#include "fax/fax_v29_tx_tables.h"
#include "fax/fax_v21_tables.h"
#include "fpm/fpm_tables.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
extern int init_vmi_v29rx(FaxVmiV29 *m, unsigned short rate, int unused, void *user);
#define fax_init_vmi_v29rx init_vmi_v29rx
extern const int FPM_SRE_CFG[14];
extern const int FPM_FSE_CFG[14];
#else
#define SIDE_BLOB 0
#endif

extern unsigned int dsplibs_debug_level;
extern int FPM_MRF_init(FpmMrf *mrf, const int *cfg, int fresh);
extern unsigned int FPM_MRF_filter(FpmMrf *mrf, short *in, short *out, short n);
extern void FPM_MRF_free(FpmMrf *mrf);
extern void FPM_MTD_delete(FpmMtd *mtd);
extern void FPM_TONE_delete(FpmTone *tone);

#include "fax/fax_modem_views.h"
#if UINTPTR_MAX != 0xffffffffu
/* x86-64: src/fax/fax_modem_wrappers.c references ALL eleven accessors of src/fax/fax_modem_views.h; this test uses the V.29 transmit ones (src/fax/fax_v29_tx.c) and the receive one (src/fax/fax_v29_rx.c).  The other
 * eight belong to the V.17 / V.27ter engines (R24 - R26, R20 / R21): stand-ins that abort (no scenario of this test reaches the wrappers that use them). */
static void *no_engine(void) { fprintf(stderr, "HARNESS/MODEL VIOLATION: accessor of another engine called\n"); exit(3); }
FaxSdm *faxw_v17_txsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSmcV17 *faxw_v17_txsmc(void *o) { (void)o; return (FaxSmcV17 *)no_engine(); }
unsigned short *faxw_v17_encmode(void *o) { (void)o; return (unsigned short *)no_engine(); }
FaxSdm *faxw_v17_rxsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSdmV27 *faxw_v27_txsdm(void *o) { (void)o; return (FaxSdmV27 *)no_engine(); }
short *faxw_v27_rate(void *o) { (void)o; return (short *)no_engine(); }
FaxSdmV27 *faxw_v27_rxsdm(void *o) { (void)o; return (FaxSdmV27 *)no_engine(); }
#endif

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_TAB, CV_MSG, CV_MSG_OOB, CV_MSG_NEG, CV_CFGDEF,
  CV_L_NEW, CV_L_SUP_LIVE, CV_L_SUP_NOHDX, CV_L_SUP_NOBUNDLE_PROBE, CV_L_RECREATE, CV_L_NULLCFG, CV_L_R7200, CV_L_R9600, CV_L_RBAD, CV_L_DBG, CV_L_DEL, CV_L_PROBE, CV_L_RUN, CV_L_USER,
  CV_L_RATECHG, CV_L_SAMERATE,
  CV_V_INIT, CV_V_INIT_DBG, CV_V_CREATE, CV_V_CREATE_NULLCFG, CV_V_CREATE_SUP, CV_V_PROC, CV_V_STATUS, CV_V_CTL, CV_V_DEL, CV_V_MSG, CV_V_R7200, CV_V_R9600,
  CV_S_ST0, CV_S_ST1, CV_S_ST2, CV_S_ST3, CV_S_ST4, CV_S_ST5, CV_S_TO1, CV_S_TO2, CV_S_TO3, CV_S_TO4, CV_S_IDLE_BACK, CV_S_ERR, CV_S_LOSS, CV_S_Q4E, CV_S_Q4A_FULL, CV_S_HOLD, CV_S_CTL,
  CV_S_RECREATE, CV_S_STATUS, CV_S_R7200, CV_S_R9600, CV_S_NOISE, CV_S_SLIP, CV_S_LOSSSEG, CV_S_BURST, CV_S_RETRAIN, CV_S_GAINLOW, CV_S_CLIP, CV_S_DC, CV_S_PATHB, CV_S_S2A,
  CV_S_N0, CV_S_N1, CV_S_NFULL, CV_S_NRAND, CV_S_BITS, CV_S_DBG, CV_S_CALLBACK_EQ, CV_S_CALLBACK_DEC, CV_S_C40, CV_S_BADRATE, CV_S_USER, CV_S_PROTO, CV_S_DATAOK, CV_S_DROP2, CV_S_ENERGY, CV_S_LOWSNR, CV_S_TONESEG, CV_S_V21CD,
  CV_H_START, CV_H_EPOCH, CV_H_PRTCOL, CV_H_DATA, CV_H_IDLE, CV_H_ERROR, CV_H_STATEDEF, CV_H_HOLD, CV_H_PATHA, CV_H_PATHB, CV_H_CNT, CV_H_E52HI, CV_H_Q50, CV_H_ENERGY, CV_H_Q54, CV_H_S28, CV_H_MTD2,
  CV_H_QUAL0, CV_H_QUAL1, CV_H_QUAL2, CV_H_QUAL3, CV_H_DEMOD, CV_H_DEMOD_MTD, CV_H_SRE_BIG, CV_H_CARR, CV_H_NOCARR, CV_H_CDET_BRANCH, CV_H_IDLE_SMALL, CV_H_N0, CV_H_NBIG, CV_H_WARM, CV_H_LOWSNR,
  CV_H_HIGHSNR, CV_H_PRTCOL_END, CV_H_V21CD, CV_H_STATUS,
  CV_C_EPOCH, CV_C_EPOCH_H0, CV_C_EPOCH_H2, CV_C_EPOCH_NEG, CV_C_EPOCH_CNT, CV_C_EPOCH_SWITCH, CV_C_EPOCH_NOSW, CV_C_TRAIN, CV_C_TRAIN_ONE, CV_C_TRAIN_ZERO, CV_C_TRAIN_R0, CV_C_TRAIN_R1, CV_C_TRAIN_END,
  CV_C_TRAIN_WRAP, CV_C_DEC, CV_C_DEC_R7200, CV_C_DEC_R9600, CV_C_DEC_WRAP, CV_C_DEC_BIT3, CV_C_NEXT_DBG, CV_C_NEXT_C0, CV_C_NEXT_C1, CV_C_NEXT_C2, CV_C_NEXT_C3, CV_C_NEXT_C4, CV_C_NEXT_DEF,
  CV_C_FAKE, CV_C_FAKE_NEG, CV_C_FAKE_PART, CV_C_CTL, CV_C_CTL_HOLD, CV_C_CTL_RECREATE, CV_C_CTL_W0, CV_C_CTL_W20, CV_C_CTL_NULL, CV_C_STATUS, CV_C_STATUS_NULL, CV_C_GETSNR, CV_C_EPOCHDET, CV_C_CARRDET
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_TAB, "tables dumped");
      cov_def(CV_CFGDEF, "default SRE / FSE config words");
      cov_def(CV_MSG, "v29rx_message idx 0..7");
      cov_def(CV_MSG_OOB, "v29rx_message idx 8 (out of table)");
      cov_def(CV_MSG_NEG, "v29rx_message idx > limit / negative");
      break;
    case 1:
      cov_def(CV_L_NEW, "V29RX_create NULL object");
      cov_def(CV_L_SUP_LIVE, "V29RX_create supplied live object");
      cov_def(CV_L_SUP_NOHDX, "V29RX_create supplied object, hdx NULL (bundle live)");
      cov_def(CV_L_SUP_NOBUNDLE_PROBE, "supplied object without bundle probe");
      cov_def(CV_L_RECREATE, "V29RX_create(o, o)");
      cov_def(CV_L_NULLCFG, "V29RX_create NULL cfg");
      cov_def(CV_L_R7200, "V29RX_create rate 7200");
      cov_def(CV_L_R9600, "V29RX_create rate 9600");
      cov_def(CV_L_RBAD, "V29RX_create other rate (error flags)");
      cov_def(CV_L_DBG, "V29RX_create debug level 2");
      cov_def(CV_L_DEL, "V29RX_delete");
      cov_def(CV_L_PROBE, "create malloc-failure probe");
      cov_def(CV_L_RUN, "run after create");
      cov_def(CV_L_USER, "cfg word 5 (user word) != 0");
      cov_def(CV_L_RATECHG, "rate-changing re-create");
      cov_def(CV_L_SAMERATE, "same-rate re-create");
      break;
    case 2:
      cov_def(CV_V_INIT, "fax_init_vmi_v29rx");
      cov_def(CV_V_INIT_DBG, "init debug level 3");
      cov_def(CV_V_CREATE, "v29rx_create");
      cov_def(CV_V_CREATE_NULLCFG, "v29rx_create NULL cfg");
      cov_def(CV_V_CREATE_SUP, "v29rx_create with a supplied engine");
      cov_def(CV_V_PROC, "v29rx_process");
      cov_def(CV_V_STATUS, "v29rx_status");
      cov_def(CV_V_CTL, "v29rx_control");
      cov_def(CV_V_DEL, "v29rx_delete");
      cov_def(CV_V_MSG, "v29rx_message");
      cov_def(CV_V_R7200, "VMI rate 7200");
      cov_def(CV_V_R9600, "VMI rate 9600");
      break;
    case 3:
      cov_def(CV_S_ST0, "RX state 0 (start) observed");
      cov_def(CV_S_ST1, "RX state 1 (epoch detect) observed");
      cov_def(CV_S_ST2, "RX state 2 (protocol) observed");
      cov_def(CV_S_ST3, "RX state 3 (data) observed");
      cov_def(CV_S_ST5, "RX state 5 (error) observed");
      cov_def(CV_S_TO1, "0 -> 1");
      cov_def(CV_S_TO2, "1 -> 2");
      cov_def(CV_S_TO3, "2 -> 3");
      cov_def(CV_S_ERR, "-> 5");
      cov_def(CV_S_LOSS, "state byte 4 (loss of carrier)");
      cov_def(CV_S_Q4E, "q4e set");
      cov_def(CV_S_Q4A_FULL, "q4a reached 0x33");
      cov_def(CV_S_HOLD, "hold control");
      cov_def(CV_S_CTL, "control");
      cov_def(CV_S_RECREATE, "control re-create");
      cov_def(CV_S_STATUS, "status");
      cov_def(CV_S_R7200, "rate 7200 stream");
      cov_def(CV_S_R9600, "rate 9600 stream");
      cov_def(CV_S_NOISE, "noisy stream");
      cov_def(CV_S_SLIP, "clock slip stream");
      cov_def(CV_S_LOSSSEG, "carrier loss segment");
      cov_def(CV_S_BURST, "noise burst");
      cov_def(CV_S_RETRAIN, "transmitter re-train");
      cov_def(CV_S_GAINLOW, "low level stream");
      cov_def(CV_S_CLIP, "clipping stream");
      cov_def(CV_S_DC, "DC offset stream");
      cov_def(CV_S_PATHB, "hdx.s1c != 0 stream (carrier detector path B)");
      cov_def(CV_S_S2A, "path B with s2a set");
      cov_def(CV_S_N0, "call with n = 0");
      cov_def(CV_S_N1, "call with n = 1");
      cov_def(CV_S_NFULL, "call with a 120-sample block");
      cov_def(CV_S_NRAND, "call with a random block");
      cov_def(CV_S_BITS, "symbols returned");
      cov_def(CV_S_DBG, "debug level 2 stream");
      cov_def(CV_S_CALLBACK_EQ, "FSE callback = eq_train observed");
      cov_def(CV_S_CALLBACK_DEC, "FSE callback = decision observed");
      cov_def(CV_S_C40, "FSE c40 set");
      cov_def(CV_S_BADRATE, "stream with an invalid rate word");
      cov_def(CV_S_USER, "stream with a user word");
      cov_def(CV_S_PROTO, "protocol state with a block returned");
      cov_def(CV_S_DATAOK, "data state with the quality flag");
      cov_def(CV_S_ENERGY, "energy drop");
      cov_def(CV_S_TONESEG, "V.21 tone segment");
      cov_def(CV_S_V21CD, "V29: V21 Carrier detected");
      break;
    case 4:
      cov_def(CV_H_START, "RxHdxStartV29");
      cov_def(CV_H_EPOCH, "RxHdxEpochDetV29");
      cov_def(CV_H_PRTCOL, "RxHdxPrtcolV29");
      cov_def(CV_H_DATA, "RxHdxDataV29");
      cov_def(CV_H_IDLE, "RxHdxIdleV29");
      cov_def(CV_H_ERROR, "RxHdxErrorV29");
      cov_def(CV_H_STATEDEF, "state number outside 0..4");
      cov_def(CV_H_HOLD, "Data with hold");
      cov_def(CV_H_PATHA, "DataCarrierDetectV29 path A");
      cov_def(CV_H_PATHB, "DataCarrierDetectV29 path B");
      cov_def(CV_H_CNT, "loop.cnt > 0x3e7");
      cov_def(CV_H_E52HI, "fse.e52 > 0x3fff");
      cov_def(CV_H_Q50, "energy test enabled");
      cov_def(CV_H_ENERGY, "energy drop detected");
      cov_def(CV_H_Q54, "q54 reached 2");
      cov_def(CV_H_S28, "hdx.s28 > 0x4ff");
      cov_def(CV_H_MTD2, "hdx MTD2 fired");
      cov_def(CV_H_QUAL0, "QualityDetectV29 q4a == 0");
      cov_def(CV_H_QUAL1, "QualityDetectV29 q4a < 0x32");
      cov_def(CV_H_QUAL2, "QualityDetectV29 q4a == 0x32");
      cov_def(CV_H_QUAL3, "QualityDetectV29 q4a > 0x32");
      cov_def(CV_H_DEMOD, "DemodDataV29");
      cov_def(CV_H_DEMOD_MTD, "DemodDataV29 aborted by the tone detector");
      cov_def(CV_H_SRE_BIG, "SRE output > 164");
      cov_def(CV_H_CARR, "carrier detected");
      cov_def(CV_H_NOCARR, "no carrier");
      cov_def(CV_H_CDET_BRANCH, "carrier detect with the gate off");
      cov_def(CV_H_IDLE_SMALL, "Idle: decision error small");
      cov_def(CV_H_N0, "n = 0");
      cov_def(CV_H_NBIG, "n >= 150");
      cov_def(CV_H_WARM, "warmed up engine");
      cov_def(CV_H_LOWSNR, "SNR flag set (GetSNRV29 <= 8)");
      cov_def(CV_H_HIGHSNR, "SNR flag clear (GetSNRV29 > 8)");
      cov_def(CV_H_PRTCOL_END, "protocol state finished (count <= 0)");
      cov_def(CV_H_V21CD, "V29: V21 Carrier detected (debug text)");
      cov_def(CV_H_STATUS, "V29RX_status on a driven engine");
      cov_def(CV_C_GETSNR, "GetSNRV29");
      cov_def(CV_C_EPOCHDET, "EpochDetectV29");
      cov_def(CV_C_CARRDET, "CarrierDetectV29");
      break;
    case 5:
      cov_def(CV_C_EPOCH, "V29RX_epoch_det");
      cov_def(CV_C_EPOCH_H0, "epoch: angle step > 0x4000 (h0 path)");
      cov_def(CV_C_EPOCH_H2, "epoch: angle step <= 0x4000 (h2 path)");
      cov_def(CV_C_EPOCH_NEG, "epoch: negative angle difference");
      cov_def(CV_C_EPOCH_CNT, "epoch: cnt18 > 0x80 test");
      cov_def(CV_C_EPOCH_SWITCH, "epoch: switch to eq_train");
      cov_def(CV_C_EPOCH_NOSW, "epoch: cnt18 > 0x80 but no switch");
      cov_def(CV_C_TRAIN, "V29RX_eq_train");
      cov_def(CV_C_TRAIN_ONE, "train: LFSR bit 0 set");
      cov_def(CV_C_TRAIN_ZERO, "train: LFSR bit 0 clear");
      cov_def(CV_C_TRAIN_R0, "train: rate 7200");
      cov_def(CV_C_TRAIN_R1, "train: rate 9600");
      cov_def(CV_C_TRAIN_END, "train: switch to decision");
      cov_def(CV_C_TRAIN_WRAP, "train / decision: cnt wrap 0x8000");
      cov_def(CV_C_DEC, "V29RX_decision");
      cov_def(CV_C_DEC_R7200, "decision 7200 (8 points)");
      cov_def(CV_C_DEC_R9600, "decision 9600 (16 points)");
      cov_def(CV_C_DEC_WRAP, "decision cnt wrap");
      cov_def(CV_C_DEC_BIT3, "decision 9600: point >= 8");
      cov_def(CV_C_NEXT_DBG, "NextState debug strings");
      cov_def(CV_C_NEXT_C0, "RxNextStateV29 state 0");
      cov_def(CV_C_NEXT_C1, "RxNextStateV29 state 1");
      cov_def(CV_C_NEXT_C2, "RxNextStateV29 state 2");
      cov_def(CV_C_NEXT_C3, "RxNextStateV29 state 3");
      cov_def(CV_C_NEXT_C4, "RxNextStateV29 state 4");
      cov_def(CV_C_NEXT_DEF, "RxNextStateV29 default");
      cov_def(CV_C_FAKE, "V29RX_modem with a harness handler");
      cov_def(CV_C_FAKE_NEG, "handler returns a negative count");
      cov_def(CV_C_FAKE_PART, "handler consumes part of the block");
      cov_def(CV_C_CTL, "V29RX_control");
      cov_def(CV_C_CTL_HOLD, "control hold");
      cov_def(CV_C_CTL_RECREATE, "control re-create");
      cov_def(CV_C_CTL_W0, "control b0c bit 3");
      cov_def(CV_C_CTL_W20, "control b0c bit 5");
      cov_def(CV_C_CTL_NULL, "control NULL");
      cov_def(CV_C_STATUS, "V29RX_status");
      cov_def(CV_C_STATUS_NULL, "status NULL");
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
  if (!strncmp(b, "V29RX_STATE_START", 17)) cov_hit(CV_C_NEXT_C0);
  if (!strncmp(b, "V29RX_STATE_EPOCH_DET", 21)) cov_hit(CV_C_NEXT_C1);
  if (!strncmp(b, "V29RX_STATE_PROTOCOL", 20)) cov_hit(CV_C_NEXT_C2);
  if (!strncmp(b, "V29RX_STATE_DATA", 16)) cov_hit(CV_C_NEXT_C3);
  if (!strncmp(b, "V29RX_STATE_IDLE", 16)) cov_hit(CV_C_NEXT_C4);
  if (!strncmp(b, "V29RX_DEFAULT", 13)) cov_hit(CV_C_NEXT_DEF);
  if (!strncmp(b, "ERROR: SRE buffer", 17)) cov_hit(CV_H_SRE_BIG);
  if (!strncmp(b, "Decision error is small", 23)) cov_hit(CV_H_IDLE_SMALL);
  if (!strncmp(b, "sudden energy", 13)) cov_hit(CV_H_ENERGY);
  if (!strncmp(b, "V29: V21 Carrier", 16)) { cov_hit(CV_S_V21CD); cov_hit(CV_H_V21CD); }
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
static void prt_status(const char *t, const FaxV29Status *st)
{
  emit(0, "%s %d %d %d %d %d %d %d %d %d %02x %02x", t, st->s0, st->s2, st->s4, st->s6, st->s8, st->sa, st->se, st->s10, st->s12, st->b14, st->b15); /* sc is never written */
}
static const void *cp(int w) { return cfgp_ptr(w); } /* a CFGP word as the address of its table */
static const short *g_agcbase; /* the AGCv29_CFG alpha table (a blob LOCAL: named by position) */

static void register_tables(void)
{
  htab_add("FSE_IFILT", V29RX_FSE_IFILT, sizeof V29RX_FSE_IFILT);
  htab_add("FSE_QFILT", V29RX_FSE_QFILT, sizeof V29RX_FSE_QFILT);
  htab_add("CRR", V29RX_CRR_TABLE, sizeof V29RX_CRR_TABLE);
  htab_add("FSE_PLLK1", V29RX_FSE_PLLK1, sizeof V29RX_FSE_PLLK1);
  htab_add("FSE_PLLK2", V29RX_FSE_PLLK2, sizeof V29RX_FSE_PLLK2);
  htab_add("MRF_FILT", V29RX_MRF_FILT, sizeof V29RX_MRF_FILT);
  htab_add("SRE_FILT", V29RX_SRE_FILT, sizeof V29RX_SRE_FILT);
  htab_add("XB", V29RX_XB_COFFS, sizeof V29RX_XB_COFFS);
  htab_add("XCLOCK", V29RX_XCLOCK, sizeof V29RX_XCLOCK);
  htab_add("YCLOCK", V29RX_YCLOCK, sizeof V29RX_YCLOCK);
  htab_add("SRE_PLLK1", V29RX_SRE_PLLK1, sizeof V29RX_SRE_PLLK1);
  htab_add("SRE_PLLK2", V29RX_SRE_PLLK2, sizeof V29RX_SRE_PLLK2);
  htab_add("MTD_V29", V29_MTD_COEFF, sizeof V29_MTD_COEFF);
  htab_add("MTD_V21CH2", V21_CHAN2_MTD_COEFF, sizeof V21_CHAN2_MTD_COEFF);
  g_agcbase = (const short *)cp(AGCv29_CFG[3]);
}
static const char *rxfn(V29RxHdxFn f)
{
  if (f == RxHdxStartV29) return "Start";
  if (f == RxHdxEpochDetV29) return "EpochDet";
  if (f == RxHdxPrtcolV29) return "Prtcol";
  if (f == RxHdxDataV29) return "Data";
  if (f == RxHdxIdleV29) return "Idle";
  if (f == RxHdxErrorV29) return "Error";
  return f ? "other" : "NULL";
}
static const char *cbname(FaxFseCb f)
{
  if (f == V29RX_epoch_det) return "epoch_det";
  if (f == V29RX_eq_train) return "eq_train";
  if (f == V29RX_decision) return "decision";
  return f ? "other" : "NULL";
}
static void d_agc(const char *t, const FpmAgc *a)
{
  emit(0, "%s AGC a=%08x %08x %08x alpha=%d beta=%d w5=%08x act=%d det=%d hist=%d rcp=%d sh=%d frz=%d", t, u(a->a.w[0]), u(a->a.w[1]), u(a->a.w[2]), (int)(a->alpha - g_agcbase),
       (int)(a->beta - (const short *)cp(AGCv29_CFG[4])), u(a->w5), a->active, a->detect, a->hist, a->rcp, a->shift, a->freeze); /* AGC.s22 is never written by FPM_AGC_init */
}
static void d_mrf(const char *t, const FpmMrf *m)
{
  char p1[40], p2[80];
  emit(0, "%s MRF a=%08x coefs=%s b=%08x %08x phase=%d frac=%d wpos=%d buflen=%d buf=%s", t, u(m->a.w), dptr(m->coefs, p1, 40), u(m->b.w[0]), u(m->b.w[1]), m->phase, m->frac, m->wpos,
       m->buflen, (hblk(p2, 80, m->buf), p2));
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
static void d_sre(const char *t, const FaxSre *s)
{
  char p1[40], p2[40], p3[40], p4[40], p5[40], p6[40], q1[80], q2[80], q3[80], q4[80];
  const FaxSreCfg *c = &s->cfg;
  emit(0, "%s SRE cfg sps=%d s02=%d s04=%d s06=%d s08=%d s0a=%d nt=%d s0e=%d w10=%s w14=%s w18=%s w1c=%s w20=%s w24=%s s28=%d s2a=%d s2c=%d s2e=%d s30=%d s32=%d w34=%08x", t, c->sps, c->periodsTrack,
       c->periodsLock, c->startCount, c->unlockGain, c->lockGain, c->nt, c->s0e, dptr(cp(c->coefTab), p1, 40), dptr(cp(c->tedCoef), p2, 40), dptr(cp(c->corrCosTab), p3, 40), dptr(cp(c->corrSinTab), p4, 40), dptr(cp(c->propGainTab), p5, 40),
       dptr(cp(c->intGainTab), p6, 40), c->errOn, c->errOff, c->unlockThresh, c->lockThresh, c->levelThresh, c->levelLen, u(c->w34));
  emit(0, "%s SRE st s38=%d s3a=%d s3c=%d s3e=%d w40=%d w44=%d w48=%d histLen=%d histPos=%d coefs=%s hist=%s st=%s rms=%s w5c=%08x w60=%08x s64=%d s66=%d s68=%d s6a=%d s6c=%d s6e=%d s70=%d s78=%d "
       "s7a=%d s7c=%d s7e=%d s80=%d s82=%d s84=%d s86=%d s88=%d s8a=%d s8c=%d s8e=%d",
       t, s->loopState, s->loopInteg, s->phaseAvg, s->errMagAvg, s->pllEnable, s->w44, s->loopEnable, s->histLen, s->histPos, (hblk(q1, 80, s->coefs), q1), (hblk(q2, 80, s->hist), q2), (hblk(q3, 80, s->st), q3),
       (hblk(q4, 80, s->rms), q4), u(s->corrA), u(s->corrB), s->phaseFrac, s->phaseRow, s->periodsCur, s->startCtr, s->periodCount, s->corrIdx, s->untilSym, s->levelMeasure, s->levelIdx, s->ppmTick, s->ppmElapsed, s->ppmSum, s->ppmOffset, s->ppmN, s->ppmNet, s->ppmStep, s->ppmPeriod,
       s->ppmNMax, s->ppmFirst);
}
static void d_fse(const char *t, const FaxFse *f)
{
  char p1[40], p2[40], p3[40], p4[40], p5[40], q1[80], q2[80], q3[80], q4[80], q5[80];
  const FaxFseCfg *c = &f->cfg;
  int n0 = f->nd0 < 0 ? 0 : f->nd0 > FAXFSE_DIAG0 ? FAXFSE_DIAG0 : f->nd0;
  emit(0, "%s FSE cfg s00=%d s02=%d w04=%s w08=%s nt=%d mu=%d,%d,%d w14=%s crrLen=%d crrAdj=%d thr=%d,%d,%d s22=%d w24=%s w28=%s cb=%s w34=%08x", t, c->blockLen, c->sps, dptr(cp(c->iCoefTab), p1, 40),
       dptr(cp(c->qCoefTab), p2, 40), c->nt, c->mu[0], c->mu[1], c->mu[2], dptr(cp(c->carrierTab), p3, 40), c->crrLen, c->crrAdj, c->settleCount, c->errThreshHi, c->errThreshLo, c->s22, dptr(cp(c->propTab), p4, 40),
       dptr(cp(c->intTab), p5, 40), cbname(c->cb), u(c->w34));
  emit(0, "%s FSE st muIdx=%d s3a=%d w3c=%08x c40=%d c44=%d c48=%d c4c=%d e50=%d e52=%d outI=%s outQ=%s count=%u idx=%d tapsI=%s tapsQ=%s hbuf=%s pos=%d w70=%08x s74=%d s76=%d pc=%d,%d,%d,%d "
       "ph=%d,%d,%d,%d s88=%d s8a=%d nd=%d,%d,%d diag=%016llx",
       t, f->muIdx, f->loopState, u(f->freqAcc), f->eqForce, f->pllEnable, f->predEnable, f->eqEnable, f->phaseErrAvg, f->decErrAvg, (hblk(q1, 80, f->outI), q1), (hblk(q2, 80, f->outQ), q2), f->count, f->idx, (hblk(q3, 80, f->tapsI), q3),
       (hblk(q4, 80, f->tapsQ), q4), (hblk(q5, 80, f->hbuf), q5), f->pos, u(f->carrierAcc), f->phaseAcc, f->predOffset, f->pc[0], f->pc[1], f->pc[2], f->pc[3], f->ph[0], f->ph[1], f->ph[2], f->ph[3], f->settleCtr, f->untilSym,
       f->nd0, f->nd1, f->nd2, fnv_ints(f->diag0, (unsigned)n0 * 2));
}
static void d_rx(const char *t, const V29Rx *o)
{
  char p1[80], p2[80], p3[80], p4[80], p5[80], p6[80];
  emit(0, "%s RX cfg=%08x %08x %08x %08x %08x %08x st=%08x fseI=%s fseQ=%s idx=%s tapsI=%s tapsQ=%s nt=%d w34=%d w38=%d s3c=%d w40=%d w44=%d s48=%d", t, u(o->cfg.w[0]), u(o->cfg.w[1]),
       u(o->cfg.w[2]), u(o->cfg.w[3]), u(o->cfg.w[4]), u(o->cfg.w[5]), u(o->st.w), (hblk(p1, 80, o->fseI), p1), (hblk(p2, 80, o->fseQ), p2),
       o->bundle ? (o->fseIdx == &o->bundle->fse.idx ? "&fse.idx" : "other") : "-", (hblk(p3, 80, o->fseTapsI), p3), (hblk(p4, 80, o->fseTapsQ), p4), o->fseNt, o->w34, o->w38, o->s3c, o->w40, o->w44,
       o->s48);
  if (o->hdx) {
    const V29RxHdx *h = o->hdx;
    emit(0, "%s RXH hold=%d rate=%d fn=%s state=%d count=%d buf=%s s1c=%d buf2=%s s28=%d s2a=%d", t, h->hold, h->rate, rxfn(h->fn), h->state, h->count, (hblk(p5, 80, h->buf), p5), h->s1c,
         (hblk(p6, 80, h->buf2), p6), h->sinceTone, h->toneSearch);
    d_mtd(t, h->mtd);
    d_tone(t, h->tone);
    d_mtd(t, h->mtd2);
    d_agc(t, &h->agc);
  }
  if (o->bundle) {
    const V29RxBundle *b = o->bundle;
    const V29RxLoop *l = &b->loop;
    emit(0, "%s RXB w=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d buf1=%s buf2=%s q=%d,%d,%d,%d,%d,%d,%d", t, b->w0, b->sreGate, b->pllGate, b->wc, b->w10, b->w14, b->w18, b->w1c, b->eqGate, b->w24,
         (hblk(p1, 80, b->buf1), p1), (hblk(p2, 80, b->buf2), p2), b->decErrSmooth, b->qualBlocks, b->errLimit, b->qualityOk, b->energyTest, b->refLevel, b->refBlocks);
    emit(0, "%s RXB loop h=%d,%d,%d,%d,%d,%d,%d,%d rate=%d idx=%d reg=%d cnt18=%d s1a=%d prev=%d cnt=%d", t, l->avgA, l->avgB, l->symI0, l->symQ0, l->symI1, l->symQ1, l->symI2, l->symQ2, l->rate, l->idx, l->reg, l->phaseCnt,
         l->s1a, l->prev, l->cnt);
    emit(0, "%s RXB sdm n=%d tap=%d,%d mask=%08x nmask=%08x state=%08x sh=%d,%d", t, b->sdm.n, b->sdm.tap1, b->sdm.tap2, b->sdm.mask, b->sdm.nmask, b->sdm.state, b->sdm.sh1, b->sdm.sh2);
    d_mrf(t, &b->mrf);
    d_agc(t, &b->agc);
    d_sre(t, &b->sre);
    d_fse(t, &b->fse);
  }
  (void)p3; (void)p4;
}
/* the default SRE / FSE config blocks of the blob (FPM_SRE_CFG / FPM_FSE_CFG) / of the port (fax_*_cfg_default): the same words (scenario 0 compares them) */
static void defaults_init(void) {}
static void dbg_level(unsigned v) { dsplibs_debug_level = v; }
static void __attribute__((noinline)) stack_poison(void)
{
  volatile unsigned char b[4096];
  unsigned i;
  for (i = 0; i < sizeof b; i++) b[i] = (unsigned char)(0xa5u ^ (i * 31u));
}

/* ---------------------------------------------------------------------------------------------- signal helpers */
static short g_in[4096];    /* a copy of the block the receiver gets (the AGC rewrites it in place) */
static short g_out[2048];   /* the symbol output buffer */
static short g_tx[4096];    /* transmitter symbol input */
static short g_txo[8192];   /* transmitter samples */
static unsigned long long fnv_n(const short *p, int n) { return fnv_shorts(p, (unsigned)(n < 0 ? 0 : n)); }

/* an integer tone source (the table of test/fax/fax_v21.main.c) at the 9600 Hz sampling of the V.29 receiver: the carrier detector's V.21 channel-2 tone detector (V21_CHAN2_MTD_COEFF) returns 0 for a pure
 * tone of 1900 - 2300 Hz */
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
static unsigned g_tph;
static void tone_fill(short *dst, int n, int hz, int amp)
{
  int i;
  unsigned inc = (unsigned)((unsigned long long)(unsigned)hz * 447392ull); /* 2^32 / 9600 */
  for (i = 0; i < n; i++) {
    g_tph += inc;
    dst[i] = (short)(((long long)isin(g_tph) * amp) >> 15);
  }
}

/* the transmitter (src/fax/fax_v29_tx.c in the port run, the blob's own in the blob run): config like R22's make_cfg */
static int rate_word(int rate) { return rate == 0 ? 0x1c20 : rate == 1 ? 0x2580 : rate; }
static void make_tcfg(int *w, int rate)
{
  int i;
  for (i = 0; i < 7; i++) w[i] = V29TX_CFG[i];
  w[0] = (w[0] & 0xffff) | (rate_word(rate) << 16); /* s[1] = the baud rate */
}
typedef struct {
  V29Tx *o;
  int rate, frm;
} Gen;
static void gen_new(Gen *g, int rate)
{
  int w[7];
  make_tcfg(w, rate);
  g->o = V29TX_create(NULL, w);
  g->rate = rate;
  g->frm = 48;
}
/* one frame of transmitter samples (160); data symbols random (full constellation) or constant */
static int gen_frame(Gen *g, short *dst, int sym_mode)
{
  unsigned short n = (unsigned short)g->frm;
  int i;
  for (i = 0; i < g->frm; i++) g_tx[i] = (short)(sym_mode == 0 ? (int)(rnd() & (g->rate ? 15u : 7u)) : sym_mode == 1 ? 0 : (i & (g->rate ? 15 : 7)));
  V29TX_modem(g->o, g_tx, g_txo, &n);
  memcpy(dst, g_txo, (size_t)n * 2);
  return n;
}
static void gen_free(Gen *g)
{
  V29TX_delete(g->o);
  g->o = NULL;
}

typedef struct {
  int gain;    /* Q8 gain (256 = unity) */
  int noise;   /* amplitude mask of the additive noise (0 = none) */
  int dc;      /* DC offset */
  int slip;    /* every `slip` samples one is dropped (> 0) or repeated (< 0); 0 = none */
} Imp;
static int imp_phase;
/* impair a block in place; returns the new length (slip may change it by one) */
static int impair(short *s, int n, const Imp *p, int *clipped)
{
  int i, m = 0;
  static short tmp[4096];
  for (i = 0; i < n; i++) {
    int v = ((int)s[i] * p->gain) >> 8;
    if (p->noise) v += (int)(rnd() & (unsigned)p->noise) - (p->noise >> 1);
    v += p->dc;
    if (v > 32767) {
      v = 32767;
      *clipped = 1;
    }
    if (v < -32768) {
      v = -32768;
      *clipped = 1;
    }
    if (p->slip != 0) {
      imp_phase++;
      if (p->slip > 0 && imp_phase % p->slip == 0) continue;
      tmp[m++] = (short)v;
      if (p->slip < 0 && imp_phase % (-p->slip) == 0) tmp[m++] = (short)v;
    } else {
      tmp[m++] = (short)v;
    }
  }
  memcpy(s, tmp, (size_t)m * 2);
  return m;
}

static void make_rcfg(int *w, int rate, int variant)
{
  int i;
  for (i = 0; i < 6; i++) w[i] = V29RX_CFG[i];
  w[1] = (w[1] & (int)0xffff0000u) | rate_word(rate);
  if (variant & 1) w[5] = (int)rnd();                    /* user word */
  if (variant & 2) w[2] = (int)rnd();                    /* overwritten by the control */
  if (variant & 4) w[0] = (int)rnd(), w[3] = (int)rnd(), w[4] = (int)rnd();
}
/* a created engine: the hdx.s1c flag is garbage in the blob, defined here on both sides */
static V29Rx *rx_make(V29Rx *sup, const int *cfg, int s1c)
{
  V29Rx *o = V29RX_create(sup, cfg);
  if (o && o->hdx) o->hdx->s1c = (short)s1c;
  return o;
}
static unsigned long long g_calls;
/* one receiver call on a copy of src[0..n): everything the call can change is hashed.  Returns the engine state dword; *outn = symbols returned */
static int rx_call(V29Rx *o, const short *src, int n, int *outn, int dump)
{
  unsigned short nn = (unsigned short)n;
  int ret, k;
  memcpy(g_in, src, (size_t)n * 2);
  for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
  g_in[n] = 0x5151; /* a sentinel behind the block */
  ret = V29RX_modem(o, g_in, g_out, &nn);
  emit(0, "rx call n=%d ret=%08x outn=%u in=%016llx out=%016llx sent=%d", n, u(ret), nn, fnv_n(g_in, n), fnv_n(g_out, 1024), g_in[n]);
  if (dump) d_rx("rx", o);
  *outn = nn;
  g_calls++;
  return ret;
}
static void observe(const V29Rx *o, int *prev)
{
  int s = o->hdx->state;
  if (s == 0) cov_hit(CV_S_ST0);
  if (s == 1) cov_hit(CV_S_ST1);
  if (s == 2) cov_hit(CV_S_ST2);
  if (s == 3) cov_hit(CV_S_ST3);
  if (s == 4) cov_hit(CV_S_ST4);
  if (s == 5) cov_hit(CV_S_ST5);
  if (*prev == 0 && s == 1) cov_hit(CV_S_TO1);
  if (*prev == 1 && s == 2) cov_hit(CV_S_TO2);
  if (*prev == 2 && s == 3) cov_hit(CV_S_TO3);
  if (*prev == 3 && s == 4) cov_hit(CV_S_TO4);
  if (*prev == 4 && s == 3) cov_hit(CV_S_IDLE_BACK);
  if (s == 5 && *prev != 5) cov_hit(CV_S_ERR);
  if (o->st.b[0] == 4) cov_hit(CV_S_LOSS);
  if (o->st.b[0] == 5) cov_hit(CV_S_DROP2);
  if (o->bundle->qualityOk) cov_hit(CV_S_Q4E);
  if (o->bundle->qualBlocks == 0x33) cov_hit(CV_S_Q4A_FULL);
  if (o->bundle->fse.eqForce) cov_hit(CV_S_C40);
  if (o->bundle->fse.cfg.cb == V29RX_eq_train) cov_hit(CV_S_CALLBACK_EQ);
  if (o->bundle->fse.cfg.cb == V29RX_decision) cov_hit(CV_S_CALLBACK_DEC);
  if (o->hdx->toneSearch) cov_hit(CV_S_S2A);
  if (o->st.b[1] & 0x80) cov_hit(CV_S_LOWSNR);
  *prev = s;
}

/* ============================================================================================ captured transmitter streams */
#define CAPFR 220
static short g_cap[2][CAPFR * 200]; /* transmitter output, both rates (the transmitter of this side: blob or port) */
static int g_capn[2];
static void capture(int unused)
{
  int r;
  (void)unused;
  for (r = 0; r < 2; r++) {
    Gen g;
    int i, n = 0;
    gen_new(&g, r);
    for (i = 0; i < CAPFR; i++) {
      short fr[512];
      int k = gen_frame(&g, fr, i > 150 && i < 170 ? 1 : 0);
      memcpy(g_cap[r] + n, fr, (size_t)k * 2);
      n += k;
    }
    gen_free(&g);
    g_capn[r] = n;
    emit(0, "capture rate %d samples %d hash %016llx", r, n, fnv_n(g_cap[r], n));
  }
  trial_end("capture");
}

/* ============================================================================================ scenario 0: tables */
static void __attribute__((unused)) words_of_sre(const FaxSreCfg *c, int *w)
{
  w[0] = (int)((unsigned short)c->sps | ((unsigned)(unsigned short)c->periodsTrack << 16));
  w[1] = (int)((unsigned short)c->periodsLock | ((unsigned)(unsigned short)c->startCount << 16));
  w[2] = (int)((unsigned short)c->unlockGain | ((unsigned)(unsigned short)c->lockGain << 16));
  w[3] = (int)((unsigned short)c->nt | ((unsigned)(unsigned short)c->s0e << 16));
  w[4] = c->coefTab; w[5] = c->tedCoef; w[6] = c->corrCosTab; w[7] = c->corrSinTab; w[8] = c->propGainTab; w[9] = c->intGainTab;
  w[10] = (int)((unsigned short)c->errOn | ((unsigned)(unsigned short)c->errOff << 16));
  w[11] = (int)((unsigned short)c->unlockThresh | ((unsigned)(unsigned short)c->lockThresh << 16));
  w[12] = (int)((unsigned short)c->levelThresh | ((unsigned)(unsigned short)c->levelLen << 16));
  w[13] = c->w34;
}
static void __attribute__((unused)) words_of_fse(const FaxFseCfg *c, int *w)
{
  w[0] = (int)((unsigned short)c->blockLen | ((unsigned)(unsigned short)c->sps << 16));
  w[1] = c->iCoefTab; w[2] = c->qCoefTab;
  w[3] = (int)((unsigned short)c->nt | ((unsigned)(unsigned short)c->mu[0] << 16));
  w[4] = (int)((unsigned short)c->mu[1] | ((unsigned)(unsigned short)c->mu[2] << 16));
  w[5] = c->carrierTab;
  w[6] = (int)((unsigned short)c->crrLen | ((unsigned)(unsigned short)c->crrAdj << 16));
  w[7] = (int)((unsigned short)c->settleCount | ((unsigned)(unsigned short)c->errThreshHi << 16));
  w[8] = (int)((unsigned short)c->errThreshLo | ((unsigned)(unsigned short)c->s22 << 16));
  w[9] = c->propTab; w[10] = c->intTab;
  w[11] = c->hist ? 1 : 0; w[12] = c->cb ? 1 : 0; w[13] = c->w34;
}
static void scen0(void)
{
  int i;
  register_tables();
  emit(0, "TAB mrf filt %016llx sre filt %016llx", fnv_shorts(V29RX_MRF_FILT, 270), fnv_shorts(V29RX_SRE_FILT, 181));
  emit(0, "TAB xb %016llx clock x %016llx y %016llx", fnv_shorts(V29RX_XB_COFFS, 11), fnv_shorts(V29RX_XCLOCK, 3), fnv_shorts(V29RX_YCLOCK, 3));
  emit(0, "TAB sre pll1 %016llx pll2 %016llx", fnv_shorts(V29RX_SRE_PLLK1, 3), fnv_shorts(V29RX_SRE_PLLK2, 3));
  emit(0, "TAB fse ifilt %016llx qfilt %016llx pll1 %016llx pll2 %016llx", fnv_shorts(V29RX_FSE_IFILT, 49), fnv_shorts(V29RX_FSE_QFILT, 49), fnv_shorts(V29RX_FSE_PLLK1, 3), fnv_shorts(V29RX_FSE_PLLK2, 3));
  emit(0, "TAB crr %016llx", fnv_shorts(V29RX_CRR_TABLE, 72));
  emit(0, "TAB dec angle %016llx imap %016llx qmap %016llx mag %016llx pmap %016llx", fnv_shorts(V29RX_DEC_ANGLE, 16), fnv_shorts(V29RX_DEC_IMAP, 16), fnv_shorts(V29RX_DEC_QMAP, 16),
       fnv_shorts(V29RX_DEC_MAG, 16), fnv_shorts((const short *)V29RX_DEC_PMAP, 8));
  emit(0, "TAB mtd %016llx", fnv_shorts(V29_MTD_COEFF, 10));
  emit(0, "TAB agc cfg %08x %08x %08x alpha %d,%d beta %d,%d w5 %08x", u(AGCv29_CFG[0]), u(AGCv29_CFG[1]), u(AGCv29_CFG[2]), ((const short *)cp(AGCv29_CFG[3]))[0],
       ((const short *)cp(AGCv29_CFG[3]))[1], ((const short *)cp(AGCv29_CFG[4]))[0], ((const short *)cp(AGCv29_CFG[4]))[1], u(AGCv29_CFG[5]));
  {
    char a[80], b[80], c[80], d[80], e[80], f[80], g[80], h[80];
    emit(0, "TAB ptr crr=%s ifilt=%s qfilt=%s pll1=%s pll2=%s mrf=%s sre=%s xb=%s", dptr(cp(CFGP(V29RX_CRR_TABLE)), a, 80), dptr(cp(CFGP(V29RX_FSE_IFILT)), b, 80),
         dptr(cp(CFGP(V29RX_FSE_QFILT)), c, 80), dptr(cp(CFGP(V29RX_FSE_PLLK1)), d, 80), dptr(cp(CFGP(V29RX_FSE_PLLK2)), e, 80), dptr(cp(CFGP(V29RX_MRF_FILT)), f, 80),
         dptr(cp(CFGP(V29RX_SRE_FILT)), g, 80), dptr(cp(CFGP(V29RX_XB_COFFS)), h, 80));
    emit(0, "TAB ptr xc=%s yc=%s p1=%s p2=%s mtd=%s", dptr(cp(CFGP(V29RX_XCLOCK)), a, 80), dptr(cp(CFGP(V29RX_YCLOCK)), b, 80), dptr(cp(CFGP(V29RX_SRE_PLLK1)), c, 80),
         dptr(cp(CFGP(V29RX_SRE_PLLK2)), d, 80), dptr(cp(CFGP(V29_MTD_COEFF)), e, 80));
  }
  emit(0, "TAB CFG %08x %08x %08x %08x %08x %08x", u(V29RX_CFG[0]), u(V29RX_CFG[1]), u(V29RX_CFG[2]), u(V29RX_CFG[3]), u(V29RX_CFG[4]), u(V29RX_CFG[5]));
  emit(0, "TAB CTL %08x %08x %08x %08x", u(V29RX_CTL[0]), u(V29RX_CTL[1]), u(V29RX_CTL[2]), u(V29RX_CTL[3]));
  {
    int w[14], k;
#ifdef TEST_BLOB
    for (k = 0; k < 14; k++) w[k] = FPM_SRE_CFG[k];
#else
    words_of_sre(&fax_sre_cfg_default, w);
#endif
    emit(0, "TAB SRE default %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x", u(w[0]), u(w[1]), u(w[2]), u(w[3]), u(w[4]), u(w[5]), u(w[6]), u(w[7]), u(w[8]), u(w[9]), u(w[10]),
         u(w[11]), u(w[12]), u(w[13]));
#ifdef TEST_BLOB
    for (k = 0; k < 14; k++) w[k] = FPM_FSE_CFG[k];
#else
    words_of_fse(&fax_fse_cfg_default, w);
#endif
    emit(0, "TAB FSE default %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x", u(w[0]), u(w[1]), u(w[2]), u(w[3]), u(w[4]), u(w[5]), u(w[6]), u(w[7]), u(w[8]), u(w[9]), u(w[10]),
         u(w[11]), u(w[12]), u(w[13]));
    (void)k;
    cov_hit(CV_CFGDEF);
  }
  cov_hit(CV_TAB);
  {
    FaxVmiV29 m;
    memset(&m, 0, sizeof m);
    for (i = -3; i < 13; i++) {
      const char *s = "unset";
      v29rx_message(&m, i, &s);
      if (i == 8) {
        /* the blob reads the first word of V29RX_CTL (0x25800000), the port stores NULL: one common line, the blob's value is checked */
        if (SIDE_BLOB && (unsigned)(uintptr_t)s != u(V29RX_CTL[0])) die3("blob v29rx_message(8) != V29RX_CTL[0]");
        if (!SIDE_BLOB && s != NULL) die3("port v29rx_message(8) != NULL");
        emit(1, "MSG %d = (CTL word 0 in the blob, NULL in the port)", i);
        cov_hit(CV_MSG_OOB);
      } else {
        emit(0, "MSG %d = %s", i, s ? s : "NULL");
        if (i >= 0 && i <= 7) cov_hit(CV_MSG); else cov_hit(CV_MSG_NEG);
      }
    }
    for (i = 0; i < 9; i++) {
      const char *s = 0;
      v29rx_message(&m, -256 + i, &s); /* a negative index whose low byte is a valid index: NULL (unsigned compare) */
      emit(0, "MSG %d = %s", -256 + i, s ? s : "NULL");
      s = 0;
      v29rx_message(&m, 0x100 + i, &s);
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
    int w[6];
    g_fail_malloc = failk;
    make_rcfg(w, rate, 0);
    V29RX_create(NULL, w);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE create malloc #%d fails (rate %d): signal %d", failk, rate, WTERMSIG(st));
  else emit(1, "PROBE create malloc #%d fails (rate %d): exit %d", failk, rate, WEXITSTATUS(st));
}
/* a caller-supplied object with the given hdx / bundle pieces missing: the sub-objects are created with `fresh = 0` over the garbage of the new block */
static void probe_supplied(int mode)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    V29Rx *o;
    if (freopen("/dev/null", "w", stderr) == NULL) _exit(9); /* a garbage free() is a harness violation (exit 3) in the child: no noise */
    o = (V29Rx *)sysdep_malloc(sizeof(V29Rx));
    o->hdx = NULL;
    o->bundle = NULL;
    if (mode == 1) { /* hdx present (a real one), bundle missing */
      V29Rx *t = V29RX_create(NULL, NULL);
      o->hdx = t->hdx;
    }
    V29RX_create(o, NULL);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE create of a supplied object (mode %d) without bundle: signal %d", mode, WTERMSIG(st));
  else emit(1, "PROBE create of a supplied object (mode %d) without bundle: exit %d", mode, WEXITSTATUS(st));
  cov_hit(CV_L_SUP_NOBUNDLE_PROBE);
}
/* the re-create over a live engine with another rate: buffers sized for the old rate are re-used / re-allocated (the red zone verdict is compared) */
static void probe_rate_change(int from, int to, int dbg)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    int w[6];
    V29Rx *o;
    make_rcfg(w, from, 0);
    o = rx_make(NULL, w, 0);
    make_rcfg(w, to, 0);
    dbg_level((unsigned)dbg);
    g_verbose = 1; /* the child's own lines (allocator calls, debug text, the engine dump) are part of the output */
    V29RX_create(o, w);
    dbg_level(0);
    d_rx("changed", o);
    g_verbose = 0;
    fflush(stdout);
    V29RX_delete(o);
    _exit(redzones_dirty() ? 11 : 12);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE rate change %d -> %d (debug %d): signal %d", from, to, dbg, WTERMSIG(st));
  else emit(1, "PROBE rate change %d -> %d (debug %d): %s", from, to, dbg, WEXITSTATUS(st) == 11 ? "wrote outside a block" : WEXITSTATUS(st) == 12 ? "clean" : "other exit");
  cov_hit(CV_L_RATECHG);
}
/* garbage in the AGC state words that only a fresh init resets (rcp / shift / active): a live re-create must keep them (fresh = 0) */
static void poke_agcs(V29Rx *o)
{
  o->hdx->agc.rcp = 0x123;
  o->hdx->agc.shift = 7;
  o->hdx->agc.active = 9;
  o->bundle->agc.rcp = 0x321;
  o->bundle->agc.shift = -3;
  o->bundle->agc.active = 5;
}
static void run_some(V29Rx *o)
{
  int i, calls = 3 + (int)rmod(6);
  for (i = 0; i < calls; i++) {
    int n = 1 + (int)rmod(160), k, outn;
    for (k = 0; k < n; k++) g_tx[k] = (short)((int)(rnd() & 0x1fffu) - 0x1000);
    rx_call(o, g_tx, n, &outn, 0);
  }
  d_rx("ran", o);
  cov_hit(CV_L_RUN);
}
static void scen1(void)
{
  int trial, k;
  register_tables();
  for (trial = 0; trial < 240; trial++) {
    static const int rates[6] = {0, 1, 0x1111, 0, 1, 0x2580};
    int rate = rates[trial % 6];
    int mode = (trial / 6) % 6;
    int dbg = (trial / 36) & 1;
    int variant = (int)(rnd() & 15u);
    int w[6];
    V29Rx *o = NULL;
    make_rcfg(w, rate, trial % 3 == 2 ? variant : trial % 3);
    dbg_level(dbg ? (trial % 37 == 11 ? 0x80000002u : 2u) : 0u); /* (a level above 2 as an unsigned number, below 0 as a signed one) */
    if (rate == 0) cov_hit(CV_L_R7200); else if (rate == 1) cov_hit(CV_L_R9600); else cov_hit(CV_L_RBAD);
    if (dbg) cov_hit(CV_L_DBG);
    if (w[5]) cov_hit(CV_L_USER);
    switch (mode) {
      case 0:
        stack_poison();
        o = rx_make(NULL, w, trial & 1);
        cov_hit(CV_L_NEW);
        break;
      case 1:
        stack_poison();
        o = rx_make(NULL, NULL, 0);
        cov_hit(CV_L_NEW);
        cov_hit(CV_L_NULLCFG);
        break;
      case 2: { /* a live engine re-created with a config of the SAME rate */
        int w0[6];
        V29Rx *s;
        make_rcfg(w0, rate == 1 ? 1 : 0, 0);
        s = rx_make(NULL, w0, 0);
        d_rx("first", s);
        run_some(s);
        poke_agcs(s);
        stack_poison();
        make_rcfg(w0, rate == 1 ? 1 : 0, (int)(rnd() & 15u));
        o = V29RX_create(s, w0);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_SUP_LIVE);
        cov_hit(CV_L_SAMERATE);
        break;
      }
      case 3: { /* hdx dropped: the create allocates a new one (the old pieces are freed here) */
        int w0[6];
        V29Rx *s;
        make_rcfg(w0, rate == 1 ? 1 : 0, 0);
        s = rx_make(NULL, w0, 0);
        d_rx("first", s);
        FPM_MTD_delete(s->hdx->mtd);
        FPM_TONE_delete(s->hdx->tone);
        FPM_MTD_delete(s->hdx->mtd2);
        sysdep_free(s->hdx->buf);
        sysdep_free(s->hdx->buf2);
        sysdep_free(s->hdx);
        s->hdx = NULL;
        stack_poison();
        o = rx_make(s, w0, 0);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_SUP_NOHDX);
        break;
      }
      case 4: { /* create(o, o): the object is its own config (control path) */
        V29Rx *s;
        make_rcfg(w, rate == 1 ? 1 : 0, variant & ~1);
        s = rx_make(NULL, w, 0);
        d_rx("first", s);
        s->cfg.w[2] = 777;
        o = V29RX_create(s, s->cfg.w);
        cov_hit(CV_L_RECREATE);
        break;
      }
      default: { /* a re-create that keeps the rate but changes the user word / s0a */
        int w0[6], w1[6];
        V29Rx *s;
        make_rcfg(w0, 1, 0);
        s = rx_make(NULL, w0, 0);
        d_rx("first", s);
        poke_agcs(s);
        make_rcfg(w1, 1, 3);
        o = V29RX_create(s, w1);
        cov_hit(CV_L_SUP_LIVE);
        break;
      }
    }
    dbg_level(0);
    d_rx("created", o);
    if (trial & 1) run_some(o);
    V29RX_delete(o);
    cov_hit(CV_L_DEL);
    trial_end("life");
  }
  probe_supplied(0); /* (a supplied object with a live hdx but no bundle dies on garbage pointers in a layout dependent way: not probed) */
  for (k = 0; k < 27; k++) {
    probe_create(k, k & 1);
    cov_hit(CV_L_PROBE);
  }
  probe_rate_change(0, 1, 0);
  probe_rate_change(1, 0, 0);
  probe_rate_change(0, 0, 0);
  probe_rate_change(1, 1, 0);
  probe_rate_change(0, 1, 2);
  probe_rate_change(1, 0, 2);
}

/* ============================================================================================ scenario 2: VMI wrappers */
/* kind 0: right after the initialiser (u4 = the integer 1, u10 = the config block); kind 1: after v29rx_create (u4 = the bit buffer, u10 = the small flag short: the blob stores a short over the low half of the
 * block pointer, the block is leaked) */
static void d_vmi(const char *t, const FaxVmiV29 *m, int kind)
{
  char p1[80], p2[80];
  emit(0, "%s VMI w0=%08x u4=%s s08=%d s0a=%d s0c=%d s0e=%d u10=%s eng=%s", t, u(m->w0), kind == 0 ? (m->u4.init1 == 1 ? "1" : "other") : (hblk(p1, 80, m->u4.bits), p1), m->s08, m->s0a, m->s0c, m->s0e,
       kind == 0 ? (hblk(p2, 80, m->u10.cfg), p2) : (m->u10.s10 == 3 ? "3" : m->u10.s10 == 4 ? "4" : "other"), m->eng ? "eng" : "NULL");
}
static void scen2(void)
{
  int trial, k;
  register_tables();
  capture(0);
  for (trial = 0; trial < 60; trial++) {
    FaxVmiV29 m;
    int rate = (trial & 1) ? 0x2580 : 0x1c20;
    unsigned dbg = trial % 29 == 17 ? 0x80000003u : (unsigned)(trial % 4);
    int ret;
    void *cfgblk;
    short *bits = (short *)sysdep_malloc(1024 * 2);
    unsigned char *raw = (unsigned char *)&m;
    for (k = 0; k < (int)sizeof m; k++) raw[k] = (unsigned char)(0xa0u + (unsigned)k * 7u + (unsigned)trial);
    dbg_level((unsigned)dbg);
    if (dbg >= 3) cov_hit(CV_V_INIT_DBG);
    stack_poison();
    ret = fax_init_vmi_v29rx(&m, (unsigned short)rate, 0x55, (void *)0);
    dbg_level(0);
    emit(0, "vmi init ret=%d", ret);
    d_vmi("init", &m, 0);
    emit(0, "vmi init cfg %08x %08x %08x %08x %08x %08x", u(((V29RxCfg *)m.u10.cfg)->w[0]), u(((V29RxCfg *)m.u10.cfg)->w[1]), u(((V29RxCfg *)m.u10.cfg)->w[2]), u(((V29RxCfg *)m.u10.cfg)->w[3]),
         u(((V29RxCfg *)m.u10.cfg)->w[4]), u(((V29RxCfg *)m.u10.cfg)->w[5]));
    cfgblk = m.u10.cfg;
    cov_hit(CV_V_INIT);
    if (rate == 0x1c20) cov_hit(CV_V_R7200); else cov_hit(CV_V_R9600);
    { /* the VMI object keeps the user word as the engine handle: re-init with a live engine below */
      const int *cfg = trial % 3 == 1 ? NULL : (const int *)m.u10.cfg;
      int e;
      V29Rx *sup = NULL;
      int w[6];
      if (trial % 3 == 2) {
        make_rcfg(w, trial & 1, 0);
        sup = rx_make(NULL, w, 0);
        m.eng = (struct V29Tx *)sup;
        cov_hit(CV_V_CREATE_SUP);
      } else {
        m.eng = NULL;
      }
      m.u4.bits = bits;
      m.s0c = (short)rnd(); /* v29rx_create sets these: garbage in front */
      m.s0e = (short)rnd();
      stack_poison();
      e = v29rx_create(&m, cfg);
      emit(0, "vmi create ret=%d", e);
      if (cfg == NULL) cov_hit(CV_V_CREATE_NULLCFG);
      cov_hit(CV_V_CREATE);
      d_vmi("create", &m, 1);
      ((V29Rx *)m.eng)->hdx->s1c = 0;
      d_rx("vmi", (V29Rx *)m.eng);
    }
    {
      int calls = trial % 3 == 0 ? 75 : 6, rr = (trial & 1) ? 1 : 0;
      for (k = 0; k < calls; k++) {
        unsigned short n = (unsigned short)(trial % 3 == 0 ? 160 : 1 + rmod(160)), outn = 0x5555;
        int i, r;
        static short ibuf[4096];
        if (trial % 3 == 0) {
          for (i = 0; i < 160; i++) ibuf[i] = g_cap[rr][(k * 160 + i) % g_capn[rr]];
        } else {
          for (i = 0; i < 4096; i++) ibuf[i] = (short)((int)(rnd() & 0x3fffu) - 0x2000);
        }
        for (i = 0; i < 1024; i++) bits[i] = 0x4242;
        r = v29rx_process(&m, ibuf, &outn, &n);
        emit(0, "vmi process n=%u outn=%u ret=%08x in=%016llx bits=%016llx", n, outn, u(r), fnv_n(ibuf, 160), fnv_n(bits, 1024));
        cov_hit(CV_V_PROC);
      }
    }
    {
      FaxV29Ctl ctl;
      FaxV29Status sx;
      memset(&ctl, 0, sizeof ctl);
      ctl.w4 = (int)rnd();
      ctl.b0c = (unsigned char)(trial & 0x28);
      ctl.b0d = (unsigned char)((trial & 4 ? 0x10 : 0) | (trial & 8 ? 2 : 0));
      memset(&sx, 0x55, sizeof sx);
      emit(0, "vmi status %d", v29rx_status(&m, &sx));
      prt_status("vmi status", &sx);
      emit(0, "vmi status null %d", v29rx_status(&m, NULL));
      emit(0, "vmi control %d", v29rx_control(&m, &ctl));
      emit(0, "vmi control null %d", v29rx_control(&m, NULL));
      cov_hit(CV_V_STATUS);
      cov_hit(CV_V_CTL);
      d_vmi("ctl", &m, 1);
      ((V29Rx *)m.eng)->hdx->s1c = 0;
      d_rx("vmi ctl", (V29Rx *)m.eng);
    }
    for (k = -2; k < 11; k++) {
      const char *s = "unset";
      v29rx_message(&m, k, &s);
      if (k == 8) emit(1, "vmi msg %d = (CTL word in the blob, NULL in the port)", k);
      else emit(0, "vmi msg %d = %s", k, s ? s : "NULL");
      cov_hit(CV_V_MSG);
    }
    v29rx_delete(&m);
    cov_hit(CV_V_DEL);
    sysdep_free(cfgblk);
    sysdep_free(bits);
    trial_end("vmi");
  }
}

/* ============================================================================================ scenario 3: receive streams */
static short g_sbuf[16384];
static void rx_control_random(V29Rx *o, int i, int pathb)
{
  FaxV29Ctl ctl;
  memset(&ctl, 0, sizeof ctl);
  ctl.w4 = (int)rnd();
  ctl.b0c = (unsigned char)((rpct(30) ? 8 : 0) | (rpct(30) ? 0x20 : 0));
  ctl.b0d = (unsigned char)((rpct(i > 120 ? 40 : 10) ? 0x10 : 0) | (rpct(5) ? 2 : 0));
  cov_hit(CV_S_CTL);
  if (ctl.b0d & 0x10) cov_hit(CV_S_HOLD);
  if (ctl.b0d & 2) cov_hit(CV_S_RECREATE);
  emit(0, "rx control ret=%d b0c=%02x b0d=%02x", V29RX_control(o, &ctl), ctl.b0c, ctl.b0d);
  if (o->hdx) o->hdx->s1c = (short)pathb; /* the flag survives a re-create of a live hdx; re-applied for clarity */
  d_rx("ctl", o);
}
static void scen3(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 72; trial++) {
    int rxr = (trial % 13 == 6) ? 0x1111 : (trial & 1); /* receiver rate word: 0 / 1 / invalid */
    int txr = rxr == 0 ? 0 : 1;
    int variant = (trial / 4) & 3;
    int prof = (trial / 8) % 9;
    int pol = (trial / 3) % 6;
    int pathb = (trial % 5 == 3) ? 1 : 0;
    int dbg = trial % 9 == 4;
    int w[6], i, calls, prev = -1, frm;
    Imp imp = {256, 0, 0, 0};
    Gen g;
    V29Rx *o;
    int lossleft = 0, burstleft = 0, gainleft = 0, toneleft = 0, sn = 0;
    unsigned long long oh = 0;
    int tot = 0;
    switch (prof) {
      case 1: imp.gain = 12; cov_hit(CV_S_GAINLOW); break;
      case 2: imp.gain = 1500; cov_hit(CV_S_CLIP); break;
      case 3: imp.noise = 0x3ff; cov_hit(CV_S_NOISE); break;
      case 4: imp.noise = 0x1fff; cov_hit(CV_S_NOISE); break;
      case 5: imp.dc = 2500; cov_hit(CV_S_DC); break;
      case 6: imp.slip = -61; cov_hit(CV_S_SLIP); break;
      case 7: imp.slip = 79; cov_hit(CV_S_SLIP); break;
      case 8: imp.noise = 0x7ff; imp.gain = 150; cov_hit(CV_S_NOISE); break;
      default: break;
    }
    make_rcfg(w, rxr, variant);
    if (rxr == 0) cov_hit(CV_S_R7200); else if (rxr == 1) cov_hit(CV_S_R9600); else cov_hit(CV_S_BADRATE);
    if (variant & 1) cov_hit(CV_S_USER);
    if (pathb) cov_hit(CV_S_PATHB);
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_S_DBG);
    stack_poison();
    gen_new(&g, txr);
    frm = g.frm;
    o = rx_make(NULL, w, pathb);
    d_rx("created", o);
    imp_phase = 0;
    calls = 160 + (int)rmod(120);
    for (i = 0; i < calls; i++) {
      short fr[512];
      int n, k, take, outn, clip = 0, pst;
      int sym_mode = (i > 100 && rpct(2)) ? 1 : 0;
      (void)frm;
      n = gen_frame(&g, fr, sym_mode);
      if (lossleft > 0) {
        lossleft--;
        for (k = 0; k < n; k++) fr[k] = 0;
        cov_hit(CV_S_LOSSSEG);
      } else if (rpct(2) && i > 20 && trial >= 4) { /* the first trials are clean: lock, quality counters */
        lossleft = 1 + (int)rmod(24);
      }
      if (burstleft > 0) {
        burstleft--;
        for (k = 0; k < n; k++) fr[k] = (short)((int)(rnd() & 0xffffu) - 0x8000);
        cov_hit(CV_S_BURST);
      } else if (rpct(1) && i > 30 && trial >= 4) {
        burstleft = 1 + (int)rmod(5);
      }
      if (gainleft > 0) {
        gainleft--;
        for (k = 0; k < n; k++) fr[k] = (short)(fr[k] / 5);
        cov_hit(CV_S_ENERGY);
      } else if (rpct(2) && i > 30 && trial >= 4) {
        gainleft = 3 + (int)rmod(20);
      }
      if (toneleft > 0) {
        toneleft--;
        tone_fill(fr, n, 2100, 4000 + (int)rmod(8000));
        cov_hit(CV_S_TONESEG);
      } else if (rpct(2) && i > 40 && trial >= 4) {
        toneleft = 3 + (int)rmod(30);
      }
      if (rpct(1) && i > 20 && trial >= 4) { /* the transmitter re-trains */
        gen_free(&g);
        gen_new(&g, txr);
        cov_hit(CV_S_RETRAIN);
      }
      n = impair(fr, n, &imp, &clip);
      (void)clip;
      if (sn + n > 16000) sn = 0;
      memcpy(g_sbuf + sn, fr, (size_t)n * 2);
      sn += n;
      while (sn > 0) {
        take = pol == 0 ? 120 : pol == 4 ? 40 : pol == 5 ? 60 : pol == 1 ? 1 + (int)rmod(120) : pol == 2 ? (rpct(50) ? 80 : 1 + (int)rmod(40)) : (rpct(5) ? 0 : rpct(8) ? 1 : 1 + (int)rmod(120)); /* (blocks above ~150 can overrun the SRE output buffer of the blob when the loop slips) */
        if (take > sn) {
          if (rpct(70)) break; /* wait for more samples */
          take = sn;
        }
        if (take == 0) cov_hit(CV_S_N0);
        else if (take == 1) cov_hit(CV_S_N1);
        else if (take == 120) cov_hit(CV_S_NFULL);
        else cov_hit(CV_S_NRAND);
        pst = o->hdx->state;
        rx_call(o, g_sbuf, take, &outn, 1);
        tot += outn;
        if (outn > 0) cov_hit(CV_S_BITS);
        if (pst == 2 && outn > 0) cov_hit(CV_S_PROTO);
        if (o->hdx->state == 3 && (o->st.b[1] & 0x80) == 0 && outn > 0) cov_hit(CV_S_DATAOK);
        oh ^= fnv_n(g_out, outn);
        observe(o, &prev);
        if (take > 0) {
          memmove(g_sbuf, g_sbuf + take, (size_t)(sn - take) * 2);
          sn -= take;
        } else if (rpct(60)) {
          break;
        }
      }
      if (rpct(3)) {
        FaxV29Status stx[2];
        memset(stx, 0x55, sizeof stx);
        emit(0, "rx status ret=%d null=%d", V29RX_status(o, &stx[0]), V29RX_status(o, NULL));
        prt_status("rx status", &stx[0]);
        cov_hit(CV_S_STATUS);
      }
      if (rpct(3)) rx_control_random(o, i, pathb);
    }
    emit(0, "rx trial %d rxrate %d variant %d prof %d pol %d: symbols=%d outhash=%016llx", trial, rxr, variant, prof, pol, tot, oh);
    if (trial % 8 == 0) emit(1, "RX trial %d rate %d: %d symbols", trial, rxr, tot);
    dbg_level(0);
    V29RX_delete(o);
    gen_free(&g);
    trial_end("rx run");
  }
}

/* ============================================================================================ scenario 4: handlers / detectors */
static void loop_init(V29RxLoop *l, int r)
{
  memset(l, 0, sizeof *l);
  l->rate = r == 1;
  l->reg = 0x6a;
}
typedef struct {
  Gen g;
  V29Rx *o;
  int rate, frames;
} Sess;
static void sess_new(Sess *s, int rate, int variant, int warm, int pathb, int longtrain)
{
  int w[6], i, outn, prev = -1;
  make_rcfg(w, rate, variant);
  (void)longtrain;
  gen_new(&s->g, rate == 0 ? 0 : 1);
  s->o = rx_make(NULL, w, pathb);
  s->rate = rate;
  s->frames = 0;
  for (i = 0; i < warm; i++) {
    short fr[512];
    int n = gen_frame(&s->g, fr, 0);
    rx_call(s->o, fr, n / 2, &outn, 0); /* half frames: a full 160-sample block can overrun the SRE output buffer when the loop slips (see probe_big) */
    rx_call(s->o, fr + n / 2, n - n / 2, &outn, 0);
    observe(s->o, &prev);
    s->frames++;
  }
  d_rx("warm", s->o);
}
static void sess_free(Sess *s)
{
  V29RX_delete(s->o);
  gen_free(&s->g);
}
/* a block of input: the next frame of the session's transmitter, noise, silence or a captured stream */
static int next_block(Sess *s, short *dst, int want)
{
  int k, n, kind = (int)rmod(10);
  if (kind < 5) {
    short fr[512];
    n = gen_frame(&s->g, fr, 0);
    for (k = 0; k < want; k++) dst[k] = fr[k % n];
    if (kind == 0) for (k = 0; k < want; k++) dst[k] = (short)(dst[k] / 8);
  } else if (kind < 7) {
    for (k = 0; k < want; k++) dst[k] = (short)((int)(rnd() & 0x7fffu) - 0x4000);
  } else if (kind == 7) {
    for (k = 0; k < want; k++) dst[k] = 0;
  } else if (kind == 9) {
    static const int hz[7] = {1900, 2000, 2100, 2200, 2300, 1500, 2600};
    tone_fill(dst, want, hz[rnd() % 7u], 3000 + (int)rmod(12000));
  } else {
    int r = s->rate == 0 ? 0 : 1, off = (int)rmod((unsigned)(g_capn[r] - 200));
    for (k = 0; k < want; k++) dst[k] = g_cap[r][off + k];
  }
  return want;
}
static void agc_reset(V29Rx *o)
{
  /* RxNextStateV29 state 1 advances the AGC table pointers by one entry (two entries per table): a second advance would read past the table (layout dependent), so direct calls start at entry 0 */
  o->bundle->agc.alpha = (const short *)cp(AGCv29_CFG[3]);
  o->bundle->agc.beta = (const short *)cp(AGCv29_CFG[4]);
}
static void poke_rx(V29Rx *o)
{
  static const int cnts[8] = {0x3e5, 0x3e6, 0x3e7, 0x3e8, 0x7ffe, 0x7fff, -1, 100};
  static const int e52s[12] = {0, 100, 0x1fff, 0x2000, 0x3fff, 0x4000, -5, 0x7fff, 4, 5, 6, 7};
  static const int q4as[8] = {0, 1, 0x30, 0x31, 0x32, 0x33, -1, 7};
  V29RxBundle *b = o->bundle;
  V29RxHdx *h = o->hdx;
  if (rpct(30)) h->hold = (int)(rnd() & 1u);
  if (rpct(25)) h->s1c = (short)(rnd() & 1u);
  if (rpct(25)) h->toneSearch = (short)(rnd() & 1u);
  if (rpct(30)) h->sinceTone = (short)(rpct(50) ? (int)rmod(0x500) : 0x4f0 + (int)rmod(24));
  if (rpct(30)) b->loop.cnt = (short)cnts[rnd() & 7u];
  if (rpct(20)) b->loop.phaseCnt = (short)((int)rmod(0x90) - 4);
  if (rpct(20)) b->loop.reg = (short)(rnd() & 0xffu);
  if (rpct(20)) b->loop.idx = (short)(rnd() & 7u);
  if (rpct(20)) b->loop.prev = (short)rnd();
  if (rpct(30)) b->fse.decErrAvg = (short)e52s[rnd() % 12u];
  if (rpct(15)) b->fse.eqForce = (int)(rnd() & 1u);
  if (rpct(30)) b->sre.pllEnable = rpct(50) ? (int)(rnd() & 1u) : (int)rnd();
  if (rpct(30)) b->agc.detect = rpct(70) ? (int)(rnd() & 1u) : (int)rnd();
  if (rpct(25)) b->energyTest = (short)(rnd() & 1u);
  if (rpct(30)) b->refLevel = (short)(rpct(40) ? (int)rmod(30000) : (int)rmod(300));
  if (rpct(30)) b->refBlocks = (short)((int)rmod(4) - 1);
  if (rpct(30)) b->qualBlocks = (short)q4as[rnd() & 7u];
  if (rpct(20)) b->decErrSmooth = (short)((int)rmod(0x3000) - 0x800);
  if (rpct(20)) b->errLimit = (short)rmod(0x2000);
  if (rpct(15)) b->decErrSmooth = (short)(b->errLimit + (int)rmod(3) - 1);
  if (rpct(15)) b->qualityOk = (short)(rnd() & 1u);
  if (rpct(20)) b->w0 = (int)(rnd() & 1u);
  if (rpct(20)) b->sreGate = (int)(rnd() & 1u);
  if (rpct(20)) b->pllGate = (int)(rnd() & 1u);
  if (rpct(20)) b->eqGate = (int)(rnd() & 1u);
  if (rpct(20)) b->w18 = (int)rnd();
  if (rpct(20)) b->w0 = (int)rnd();
  if (rpct(15)) b->eqGate = (int)rnd();
  if (rpct(50)) o->st.w = (int)rnd();
}
static __attribute__((unused)) V29RxHdxFn pick_fn(int sel)
{
  switch (sel) {
    case 0: return RxHdxStartV29;
    case 1: return RxHdxEpochDetV29;
    case 2: return RxHdxPrtcolV29;
    case 3: return RxHdxDataV29;
    case 4: return RxHdxIdleV29;
    default: return RxHdxErrorV29;
  }
}

/* ---- exhaustive sweeps of the three FSE callbacks on stack fakes (they use nothing but the FSE's callback / flag members and the loop object) ---- */
static FaxFse g_ffake;
static short g_fi[64], g_fq[64];
static unsigned long long g_sh;
static void smix(unsigned v)
{
  g_sh ^= v;
  g_sh *= 1099511628211ULL;
}
static void fake_setup(V29RxLoop *l, int r)
{
  memset(&g_ffake, 0, sizeof g_ffake);
  loop_init(l, r);
  l->rate = r; /* the sweeps also try rate words 2 / -1 (never produced by the engine: they pick the 16 point tables like 1) */
  g_ffake.cfg.hist = l;
  g_ffake.outI = g_fi;
  g_ffake.outQ = g_fq;
}
static void sweep_callbacks(void)
{
  static const int RT[5] = {0, 1, 2, -1, -2};
  int r, ri, idx, i, q, k, cnt, reg, c18;
  V29RxLoop l;
  /* the decision: a grid of (I, Q) points (both rates, every previous phase index, the cnt wrap), then points within a few units of every constellation point (the distance ties and the shifts) */
  for (ri = 0; ri < 5; ri++) {
    r = RT[ri];
    fake_setup(&l, r);
    g_sh = 1469598103934665603ULL;
    for (cnt = 0; cnt < 2; cnt++)
      for (idx = 0; idx < 8; idx++)
        for (i = -16384; i < 16384; i += 331)
          for (q = -16384; q < 16384; q += 331) {
            short angle = 77, mag[3] = {1, 2, 3};
            int ret;
            l.idx = (short)idx;
            l.cnt = (short)(cnt ? 0x7fff : 5);
            g_ffake.idx = 3;
            g_fi[3] = (short)i;
            g_fq[3] = (short)q;
            ret = V29RX_decision(&g_ffake, &angle, mag);
            smix((unsigned)ret);
            smix((unsigned)(unsigned short)angle);
            smix((unsigned)(unsigned short)mag[0]);
            smix((unsigned)(unsigned short)l.idx);
            smix((unsigned)(unsigned short)l.cnt);
            if (i == -16384 && q == -16384 && idx == 0) cov_hit(CV_C_DEC);
          }
    for (k = 0; k < 16; k++) {
      static const short IM[16] = {6144, 2048, 0, -2048, -6144, -2048, 0, 2048, 10240, 6144, 0, -6144, -10240, -6144, 0, 6144};
      static const short QM[16] = {0, 2048, 6144, 2048, 0, -2048, -6144, -2048, 0, 6144, 10240, 6144, 0, -6144, -10240, -6144};
      int di, dq;
      for (di = -24; di <= 24; di++)
        for (dq = -24; dq <= 24; dq++) {
          short angle = 0, mag[3] = {1, 2, 3};
          int ret;
          l.idx = (short)(k & 7);
          l.cnt = 9;
          g_ffake.idx = 5;
          g_fi[5] = (short)(IM[k] + di * 7);
          g_fq[5] = (short)(QM[k] + dq * 7);
          ret = V29RX_decision(&g_ffake, &angle, mag);
          smix((unsigned)ret);
          smix((unsigned)(unsigned short)angle);
          smix((unsigned)(unsigned short)mag[0]);
          smix((unsigned)(unsigned short)l.idx);
        }
    }
    emit(0, "sweep decision rate %d hash %016llx", r, g_sh);
  }
  /* the training callback: every register value (also with high / sign bits), both rates, the counter around its end (0x17e) and its wrap (0x8000) */
  for (ri = 0; ri < 5; ri++) {
    r = RT[ri];
    fake_setup(&l, r);
    g_sh = 1469598103934665603ULL;
    for (reg = -300; reg < 600; reg++)
      for (c18 = 0x17c; c18 <= 0x180; c18++)
        for (cnt = 0; cnt < 3; cnt++) {
          short angle = 5, mag[3] = {1, 2, 3};
          int ret;
          l.reg = (short)reg;
          l.phaseCnt = (short)(c18 - 1);
          l.cnt = (short)(cnt == 0 ? 17 : cnt == 1 ? 0x7fff : -1);
          l.idx = (short)(reg & 7);
          g_ffake.muIdx = 7;
          g_ffake.eqForce = 1;
          g_ffake.eqEnable = 9;
          g_ffake.decErrAvg = 123;
          g_ffake.cfg.mu[1] = 55;
          g_ffake.cfg.cb = NULL;
          ret = V29RX_eq_train(&g_ffake, &angle, mag);
          smix((unsigned)ret);
          smix((unsigned)(unsigned short)angle);
          smix((unsigned)(unsigned short)mag[0]);
          smix((unsigned)(unsigned short)l.idx);
          smix((unsigned)(unsigned short)l.reg);
          smix((unsigned)(unsigned short)l.phaseCnt);
          smix((unsigned)(unsigned short)l.cnt);
          smix((unsigned)g_ffake.muIdx);
          smix((unsigned)g_ffake.eqForce);
          smix((unsigned)g_ffake.eqEnable);
          smix((unsigned)(unsigned short)g_ffake.decErrAvg);
          smix((unsigned)(unsigned short)g_ffake.cfg.mu[1]);
          smix(g_ffake.cfg.cb == V29RX_decision);
        }
    emit(0, "sweep eq_train rate %d hash %016llx", r, g_sh);
  }
  /* the epoch detector: random histories / energies at several scales (so that c = a + b and 2 * e are comparable), the angle step around 0x4000 / negative, cnt18 around 0x80 */
  for (r = 0; r < 2; r++) {
    static const int scales[7] = {40, 200, 600, 1500, 4000, 12000, 30000};
    fake_setup(&l, r);
    g_sh = 1469598103934665603ULL;
    for (k = 0; k < 160000; k++) {
      short angle, mag[3];
      int ret, sc = scales[k % 7];
      int sym = (int)rmod((unsigned)(2 * sc)) - sc;
      static const int steps[12] = {0x4000, 0x4001, 0x3fff, -0x4000, -0x3fff, -0x4001, 0x7fff, -0x8000, 0, 1, -1, 0x2000};
      g_fi[3] = (short)(rpct(50) ? sym : (int)rmod((unsigned)(2 * sc)) - sc);
      g_fq[3] = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      g_ffake.idx = 3;
      l.avgA = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      l.avgB = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      l.symI0 = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      l.symQ0 = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      l.symI1 = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      l.symQ1 = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      l.symI2 = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      l.symQ2 = (short)((int)rmod((unsigned)(2 * sc)) - sc);
      l.prev = (short)rnd();
      angle = (short)(rpct(60) ? l.prev + steps[rnd() % 12u] : (int)rnd());
      mag[0] = (short)(rpct(50) ? 0x3299 : (int)rnd());
      mag[1] = 0x1111;
      mag[2] = 0x2222;
      l.phaseCnt = (short)(rpct(60) ? 0x7f + (int)rmod(4) : rpct(10) ? -1 : (int)rmod(0x200));
      l.cnt = (short)rnd();
      l.reg = 0x11;
      l.idx = 3;
      g_ffake.eqForce = (int)(rnd() & 1u);
      g_ffake.eqEnable = 5;
      g_ffake.cfg.mu[0] = 9;
      g_ffake.cfg.cb = NULL;
      ret = V29RX_epoch_det(&g_ffake, &angle, mag);
      smix((unsigned)ret);
      smix((unsigned)(unsigned short)angle);
      smix((unsigned)(unsigned short)l.avgA);
      smix((unsigned)(unsigned short)l.avgB);
      smix((unsigned)(unsigned short)l.symI0);
      smix((unsigned)(unsigned short)l.symQ0);
      smix((unsigned)(unsigned short)l.symI1);
      smix((unsigned)(unsigned short)l.symQ1);
      smix((unsigned)(unsigned short)l.symI2);
      smix((unsigned)(unsigned short)l.symQ2);
      smix((unsigned)(unsigned short)l.prev);
      smix((unsigned)(unsigned short)l.phaseCnt);
      smix((unsigned)(unsigned short)l.cnt);
      smix((unsigned)(unsigned short)l.reg);
      smix((unsigned)(unsigned short)l.idx);
      smix((unsigned)g_ffake.eqForce);
      smix((unsigned)g_ffake.eqEnable);
      smix((unsigned)(unsigned short)g_ffake.cfg.mu[0]);
      smix(g_ffake.cfg.cb == V29RX_eq_train);
    }
    emit(0, "sweep epoch_det rate %d hash %016llx", r, g_sh);
  }
}

extern short FPM_rms(const short *data, short count);
/* DataCarrierDetectV29: the combinations of every input it reads (both paths, the thresholds 0x5db / 0x3fff / 0x4ff, the energy-drop boundary, the q54 counter) */
static void dcd_sweep(void)
{
  Sess s;
  static short blk[512];
  static const int cnts[6] = {0x3e6, 0x3e7, 0x3e8, 0, -1, 0x7fff};
  static const int e52s[5] = {-1, 0x3ffe, 0x3fff, 0x4000, 0x4001};
  static const int dets[4] = {0, 1, 2, 3};
  static const int w40s[4] = {0, 1, 2, -1};
  int path, ci, ei, di, wi, s22, q50, k, n = 40, kind, dd, ds, qq, q54;
  sess_new(&s, 1, 0, 70, 0, 1);
  agc_reset(s.o);
  for (kind = 0; kind < 5; kind++) {
    if (kind >= 3) tone_fill(blk, n, kind == 3 ? 2100 : 1500, 5000);
    else for (k = 0; k < n; k++) blk[k] = kind == 0 ? g_cap[1][500 + k] : kind == 1 ? (short)((int)(rnd() & 0x3fffu) - 0x2000) : (short)(g_cap[1][700 + k] / 7);
    for (path = 0; path < 2; path++)
      for (ci = 0; ci < 6; ci++)
        for (ei = 0; ei < 5; ei++)
          for (di = 0; di < 4; di++)
            for (wi = 0; wi < 4; wi++)
              for (s22 = 0; s22 < 2; s22++)
                for (q50 = 0; q50 < 2; q50++) {
                  V29Rx *o = s.o;
                  int ret;
                  o->hdx->s1c = (short)path;
                  o->hdx->toneSearch = (short)s22;
                  o->hdx->sinceTone = 0;
                  o->bundle->loop.cnt = (short)cnts[ci];
                  o->bundle->fse.decErrAvg = (short)e52s[ei];
                  o->bundle->agc.detect = dets[di];
                  o->bundle->sre.pllEnable = w40s[wi];
                  o->bundle->energyTest = (short)q50;
                  o->bundle->refLevel = 0;
                  o->bundle->refBlocks = 0;
                  dbg_level((ci & 1) ? 2 : 0);
                  ret = DataCarrierDetectV29(o, blk, (unsigned short)n);
                  dbg_level(0);
                  smix(0);
                  emit(0, "dcd k%d p%d c%d e%d d%d w%d s%d q%d: ret=%d s2a=%d s28=%d q54=%d q52=%d", kind, path, ci, ei, di, wi, s22, q50, ret, o->hdx->toneSearch, o->hdx->sinceTone, o->bundle->refBlocks, o->bundle->refLevel);
                  cov_hit(path ? CV_H_PATHB : CV_H_PATHA);
                }
    /* the energy-drop boundary: q52 so that the threshold is rms - 1 .. rms + 1; q54 counting to 2 */
    {
      int r = FPM_rms(blk, (short)n);
      for (path = 0; path < 2; path++)
        for (ds = -2; ds <= 2; ds++) {
          int base = (int)((((long long)(r + ds)) << 15) / 0x32fe);
          for (qq = base - 3; qq <= base + 4; qq++) {
            int thr;
            if (qq < 0 || qq > 32767) continue;
            thr = (qq * 0x32fe) >> 15;
            if (thr < r - 1 || thr > r + 1) continue;
            for (q54 = 0; q54 < 3; q54++) {
              V29Rx *o = s.o;
              int ret;
              o->hdx->s1c = (short)path;
              o->hdx->toneSearch = 0;
              o->bundle->loop.cnt = 0;
              o->bundle->fse.decErrAvg = 0;
              o->bundle->agc.detect = 1;
              o->bundle->sre.pllEnable = 1;
              o->bundle->energyTest = 1;
              o->bundle->refLevel = (short)qq;
              o->bundle->refBlocks = (short)q54;
              ret = DataCarrierDetectV29(o, blk, (unsigned short)n);
              emit(0, "dcd energy k%d p%d r=%d q52=%d thr=%d q54=%d: ret=%d q52'=%d q54'=%d", kind, path, r, qq, thr, q54, ret, o->bundle->refLevel, o->bundle->refBlocks);
            }
          }
        }
    }
    /* a negative / huge reference level: the threshold (q52 * 0x32fe) >> 15 is signed */
    for (qq = 0; qq < 5; qq++) {
      static const int q52s[5] = {-1, -100, -32768, 0x7fff, 0x4000};
      V29Rx *o = s.o;
      int ret;
      o->hdx->s1c = 0;
      o->hdx->toneSearch = 0;
      o->bundle->loop.cnt = 0;
      o->bundle->fse.decErrAvg = 0;
      o->bundle->agc.detect = 1;
      o->bundle->sre.pllEnable = 1;
      o->bundle->energyTest = 1;
      o->bundle->refLevel = (short)q52s[qq];
      o->bundle->refBlocks = 0;
      ret = DataCarrierDetectV29(o, blk, (unsigned short)n);
      emit(0, "dcd q52 k%d q52=%d: ret=%d q52'=%d q54'=%d", kind, q52s[qq], ret, o->bundle->refLevel, o->bundle->refBlocks);
    }
    /* the carrier-timeout counter s28 around 0x4ff (path B, s2a set) */
    for (dd = -6; dd <= 6; dd++)
      for (q50 = 0; q50 < 2; q50++) {
        V29Rx *o = s.o;
        int ret;
        o->hdx->s1c = 1;
        o->hdx->toneSearch = 1;
        o->hdx->sinceTone = (short)(dd <= -4 ? -n - 100 + dd : 0x4ff - n + dd);
        o->bundle->loop.cnt = 0;
        o->bundle->fse.decErrAvg = 0;
        o->bundle->agc.detect = 1;
        o->bundle->sre.pllEnable = 1;
        o->bundle->energyTest = (short)(q50 ? 1 : 0);
        o->bundle->refLevel = 0;
        dbg_level(q50 ? 2 : 0);
        ret = DataCarrierDetectV29(o, blk, (unsigned short)n);
        dbg_level(0);
        emit(0, "dcd s28 k%d dd=%d q50=%d: ret=%d s20=%d", kind, dd, q50, ret, o->hdx->sinceTone);
      }
  }
  sess_free(&s);
  trial_end("dcd sweep");
}
/* RxHdxIdleV29: the carrier flag and the decision error at the boundary 0x1fff / 0x2000 (a block too short to complete a symbol keeps the error unchanged) */
static void idle_sweep(void)
{
  Sess s;
  static short blk[512];
  int e, st, kind, k, n = 40;
  sess_new(&s, 0, 0, 70, 0, 1);
  for (kind = 0; kind < 2; kind++)
    for (e = 0x1ffd; e <= 0x2002; e++)
      for (st = 3; st <= 4; st++) {
        V29Rx *o = s.o;
        unsigned short nn = (unsigned short)n;
        int ret;
        for (k = 0; k < n; k++) blk[k] = kind == 0 ? g_cap[0][500 + k] : (short)(g_cap[0][500 + k] * 3);
        agc_reset(o);
        o->hdx->state = (short)st;
        o->hdx->fn = RxHdxIdleV29;
        o->bundle->fse.decErrAvg = (short)e;
        o->bundle->fse.untilSym = 30000;
        o->bundle->sre.pllEnable = 1;
        o->bundle->sreGate = 1;
        o->st.w = 0x7000 + st;
        dbg_level(e & 1 ? 2 : 0);
        ret = RxHdxIdleV29(o, blk, g_out, &nn);
        dbg_level(0);
        emit(0, "idle sweep e52=%x st=%d kind=%d: ret=%d nn=%u state=%d b=%08x e52'=%d", e, st, kind, ret, nn, o->hdx->state, u(o->st.w), o->bundle->fse.decErrAvg);
        d_rx("idle", o);
      }
  sess_free(&s);
  trial_end("idle sweep");
}


/* RxHdxPrtcolV29: the end of the protocol state (count <= 0) sets the connect code and the SNR flag from GetSNRV29 = 14 - e52 at its boundary (e52 = 5 .. 7); a block too short to complete a symbol keeps e52 */
static void prtcol_sweep(void)
{
  Sess s;
  static short blk[512];
  int e, c, kind, k, n = 40;
  for (kind = 0; kind < 2; kind++) {
    sess_new(&s, kind, 0, 70, 0, 1);
    for (e = 3; e <= 11; e++)
      for (c = 0; c <= 3; c++) {
        V29Rx *o = s.o;
        unsigned short nn = (unsigned short)n;
        int ret;
        for (k = 0; k < n; k++) blk[k] = g_cap[kind][500 + k];
        agc_reset(o);
        o->hdx->state = 2;
        o->hdx->fn = RxHdxPrtcolV29;
        o->hdx->count = (short)c;
        o->bundle->fse.decErrAvg = (short)e;
        o->bundle->fse.untilSym = 30000;
        o->bundle->sre.pllEnable = 1;
        o->bundle->sreGate = 1;
        o->st.w = 0x3000 + c;
        ret = RxHdxPrtcolV29(o, blk, g_out, &nn);
        emit(0, "prtcol sweep rate %d e52=%d count=%d: ret=%d nn=%u state=%d b=%08x e52'=%d", kind, e, c, ret, nn, o->hdx->state, u(o->st.w), o->bundle->fse.decErrAvg);
        if (c == 1 && o->hdx->state == 3) cov_hit(CV_H_PRTCOL_END);
        d_rx("prtcol", o);
      }
    sess_free(&s);
  }
  trial_end("prtcol sweep");
}
/* QualityDetectV29: every q4a stage x the decoder error at / around the threshold x the carrier flag */
static void quality_sweep(void)
{
  static const int q4as[8] = {0, 1, 0x30, 0x31, 0x32, 0x33, 0x34, -1};
  static const int q4cs[4] = {0x320, -1, 0x7fff, -32768};
  int qa, qi, d, det, w40, r, e;
  V29Rx *o;
  int w[6];
  make_rcfg(w, 0, 0);
  o = rx_make(NULL, w, 0);
  {
    /* rounding constants of the smoothing: e52 * 0xccd and q48 * 0x7333 at the exact carry of the +0x4000 round */
    static int g_e52r[8], g_q48r[8];
    int ne = 0, nq = 0, v;
    for (v = -32768; v < 32768 && ne < 6; v++) {
      int m = (v * 0xccd) & 0x7fff;
      if (m == 0x4000 || m == 0x3fff || m == 0x4001) g_e52r[ne++] = v;
    }
    for (v = -32768; v < 32768 && nq < 6; v++) {
      int m = (v * 0x7333) & 0x7fff;
      if (m == 0x4000 || m == 0x3fff || m == 0x4001) g_q48r[nq++] = v;
    }
    for (e = 0; e < ne; e++)
      for (d = 0; d < nq; d++) {
        o->bundle->qualBlocks = 5;
        o->bundle->errLimit = 0x1b9e;
        o->bundle->decErrSmooth = (short)g_q48r[d];
        o->bundle->agc.detect = 1;
        o->bundle->sre.pllEnable = 1;
        o->bundle->fse.decErrAvg = (short)g_e52r[e];
        r = QualityDetectV29(o);
        emit(0, "quality round e52=%d q48=%d: r=%d q48'=%d q4a=%d", g_e52r[e], g_q48r[d], r, o->bundle->decErrSmooth, o->bundle->qualBlocks);
      }
  }
  for (qi = 0; qi < 4; qi++)
  for (qa = 0; qa < 8; qa++)
    for (d = -2; d <= 2; d++)
      for (det = 0; det < 3; det++)
        for (w40 = 0; w40 < 2; w40++)
          for (e = 0; e < 3; e++) {
            o->bundle->qualBlocks = (short)q4as[qa];
            o->bundle->errLimit = (short)q4cs[qi];
            o->bundle->decErrSmooth = (short)(q4cs[qi] + d);
            o->bundle->qualityOk = 0;
            o->bundle->agc.detect = det;
            o->bundle->sre.pllEnable = w40;
            o->bundle->fse.decErrAvg = (short)(e == 0 ? q4cs[qi] + d : e == 1 ? -77 : 5000);
            dbg_level(d == 0 ? 2 : 0);
            r = QualityDetectV29(o);
            dbg_level(0);
            emit(0, "quality qi=%d qa=%d d=%d det=%d w40=%d e=%d: r=%d q48=%d q4a=%d q4e=%d", qi, q4as[qa], d, det, w40, e, r, o->bundle->decErrSmooth, o->bundle->qualBlocks, o->bundle->qualityOk);
          }
  V29RX_delete(o);
  trial_end("quality sweep");
}
/* a block longer than the receiver's buffers (160 shorts: n * 9 / 10 samples leave the matched filter, more than 164 reach the SRE output): the blob overruns its two sample buffers and says so at debug
 * level 2.  Forked child; the verdict (red zones written / debug text seen) and the child's own lines are compared */
static void probe_big(int n, int rate)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    Sess s;
    static short blk[1024];
    int k;
    unsigned short nn;
    sess_new(&s, rate, 0, 60, 0, 1);
    for (k = 0; k < n + 1; k++) blk[k] = g_cap[rate][(300 + k) % g_capn[rate]];
    dbg_level(2);
    g_verbose = 1;
    nn = (unsigned short)n;
    k = RxHdxDataV29(s.o, blk, g_out, &nn);
    emit(1, "big block n=%d ret=%d nn=%u out=%016llx", n, k, nn, fnv_n(g_out, 1024));
    g_verbose = 0;
    fflush(stdout);
    if (g_cov[CV_H_SRE_BIG] == 0) _exit(13);
    _exit(redzones_dirty() ? 11 : 12);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE big block n=%d rate %d: signal %d", n, rate, WTERMSIG(st));
  else emit(1, "PROBE big block n=%d rate %d: %s", n, rate, WEXITSTATUS(st) == 11 ? "wrote outside a block" : WEXITSTATUS(st) == 12 ? "clean" : WEXITSTATUS(st) == 13 ? "no SRE violation text" : "other exit");
  if (WEXITSTATUS(st) == 11 || WEXITSTATUS(st) == 12) cov_hit(CV_H_SRE_BIG);
}
static void scen4(void)
{
  int trial;
  register_tables();
  capture(0);
  for (trial = 0; trial < 90; trial++) {
    Sess s;
    int rate = trial % 7 == 3 ? 0x1111 : (trial & 1);
    int variant = (trial / 2) & 3;
    int warm = (int)(trial % 5 == 0 ? 0 : trial % 5 == 1 ? 12 : trial % 5 == 2 ? 30 : trial % 5 == 3 ? 70 : 110) + (int)rmod(6);
    int sub, dbgs = trial % 6 == 5;
    sess_new(&s, rate, variant, warm, (trial % 4 == 1), (trial / 3) & 1);
    cov_hit(CV_H_WARM);
    dbg_level(dbgs ? 2 : 0);
    for (sub = 0; sub < 55; sub++) {
      static const int sts[10] = {0, 1, 2, 3, 4, 5, -1, 7, 0x7fff, 3};
      static const int cnts[7] = {0, 1, 2, 5, -1, 100, 3};
      static const int ns[9] = {0, 1, 5, 40, 100, 150, 152, 17, 152}; /* (blocks of 159 / 160 can make the SRE emit more than the 164 samples of its output buffer when it slips: a blob overrun, probed separately) */
      int fnsel = (int)(rnd() % 12u), n = ns[rnd() % 9u], i, ret = 0, outn;
      V29Rx *o = s.o;
      unsigned short nn;
      static short blk[512];
      int adv = (int)rmod(4);
      short q54pre, s20pre, mwpre, mppre, mfpre;
      int statepre;
      agc_reset(o);
      for (i = 0; i < adv; i++) {
        short fr[512];
        int k = gen_frame(&s.g, fr, 0);
        rx_call(o, fr, k / 2, &outn, 0);
        rx_call(o, fr + k / 2, k - k / 2, &outn, 0);
      }
      if (rpct(45)) poke_rx(o);
      if (rpct(40)) {
        o->hdx->state = (short)sts[rnd() % 10u];
        o->hdx->count = (short)cnts[rnd() % 7u];
      }
      agc_reset(o);
      next_block(&s, blk, n);
      blk[n] = 0x5151;
      q54pre = o->bundle->refBlocks;
      s20pre = o->hdx->sinceTone;
      mwpre = o->bundle->mrf.wpos;
      mppre = o->bundle->mrf.phase;
      mfpre = o->bundle->mrf.frac;
      statepre = o->hdx->state;
      for (i = 0; i < 2048; i++) g_out[i] = 0x7777;
      nn = (unsigned short)n;
      if (n == 0) cov_hit(CV_H_N0);
      if (n >= 150) cov_hit(CV_H_NBIG);
      if (o->hdx->state < 0 || o->hdx->state > 4) cov_hit(CV_H_STATEDEF);
      if (o->hdx->hold) cov_hit(CV_H_HOLD);
      if (o->hdx->s1c == 0) cov_hit(CV_H_PATHA); else cov_hit(CV_H_PATHB);
      if (o->bundle->loop.cnt > 0x3e7) cov_hit(CV_H_CNT);
      if (o->bundle->fse.decErrAvg > 0x3fff) cov_hit(CV_H_E52HI);
      if (o->bundle->energyTest) cov_hit(CV_H_Q50);
      if (o->hdx->sinceTone > 0x4ff) cov_hit(CV_H_S28);
      d_rx("pre", o);
      stack_poison();
      switch (fnsel) {
        case 0: o->hdx->fn = RxHdxStartV29; ret = RxHdxStartV29(o, blk, g_out, &nn); cov_hit(CV_H_START); break;
        case 1: o->hdx->fn = RxHdxEpochDetV29; ret = RxHdxEpochDetV29(o, blk, g_out, &nn); cov_hit(CV_H_EPOCH); break;
        case 2: o->hdx->fn = RxHdxPrtcolV29; ret = RxHdxPrtcolV29(o, blk, g_out, &nn); cov_hit(CV_H_PRTCOL); break;
        case 3: o->hdx->fn = RxHdxDataV29; ret = RxHdxDataV29(o, blk, g_out, &nn); cov_hit(CV_H_DATA); break;
        case 4: o->hdx->fn = RxHdxIdleV29; ret = RxHdxIdleV29(o, blk, g_out, &nn); cov_hit(CV_H_IDLE); break;
        case 5: o->hdx->fn = RxHdxErrorV29; ret = RxHdxErrorV29(o, blk, g_out, &nn); cov_hit(CV_H_ERROR); break;
        case 6: ret = DemodDataV29(o, blk, g_out, nn); cov_hit(CV_H_DEMOD); break;
        case 7:
          ret = DataCarrierDetectV29(o, blk, nn);
          if (o->hdx->s1c == 0) cov_hit(CV_H_PATHA);
          if (ret) cov_hit(CV_H_CARR); else cov_hit(CV_H_NOCARR);
          break;
        case 8:
          ret = QualityDetectV29(o);
          if (o->bundle->qualBlocks == 0) cov_hit(CV_H_QUAL0);
          else if (o->bundle->qualBlocks < 0x32) cov_hit(CV_H_QUAL1);
          else if (o->bundle->qualBlocks == 0x32) cov_hit(CV_H_QUAL2);
          else cov_hit(CV_H_QUAL3);
          emit(0, "quality again %d", QualityDetectV29(o));
          break;
        case 9:
          RxNextStateV29(o);
          ret = o->hdx->state;
          break;
        case 10:
          ret = CarrierDetectV29(o);
          emit(0, "epoch %d snr %d", EpochDetectV29(o), GetSNRV29(o));
          {
            FaxV29Status st;
            memset(&st, (int)(rnd() & 0xffu), sizeof st);
            emit(0, "hdx status %d", V29RX_status(o, &st));
            prt_status("hdx status", &st);
            cov_hit(CV_H_STATUS);
          }
          cov_hit(CV_C_GETSNR);
          cov_hit(CV_C_EPOCHDET);
          cov_hit(CV_C_CARRDET);
          break;
        default: { /* the dispatcher on this state */
          unsigned short n2 = nn;
          ret = V29RX_modem(o, blk, g_out, &n2);
          nn = n2;
          break;
        }
      }
      if (o->bundle->sre.pllEnable == 0 && o->bundle->sreGate) cov_hit(CV_H_CDET_BRANCH);
      if (fnsel == 2 && statepre == 2 && o->hdx->state == 3) cov_hit(CV_H_PRTCOL_END);
      if (fnsel == 2 || fnsel == 3) {
        if (o->st.b[1] & 0x80) cov_hit(CV_H_LOWSNR); else cov_hit(CV_H_HIGHSNR);
      }
      if ((fnsel == 3 || fnsel == 7) && q54pre == 1 && o->bundle->energyTest && o->bundle->refBlocks == 0) cov_hit(CV_H_Q54);
      if ((fnsel == 3 || fnsel == 7) && o->hdx->s1c != 0 && s20pre != 0 && o->hdx->sinceTone == 0) cov_hit(CV_H_MTD2);
      if (fnsel == 6 && statepre == 0 && n >= 16 && o->bundle->mrf.wpos == mwpre && o->bundle->mrf.phase == mppre && o->bundle->mrf.frac == mfpre) cov_hit(CV_H_DEMOD_MTD);
      emit(0, "hdx trial %d.%d fn %d n %d: ret=%d n=%u in=%016llx out=%016llx sent=%d", trial, sub, fnsel, n, ret, nn, fnv_n(blk, n), fnv_n(g_out, 1024), blk[n]);
      d_rx("post", o);
    }
    dbg_level(0);
    sess_free(&s);
    trial_end("hdx session");
  }
  dcd_sweep();
  idle_sweep();
  prtcol_sweep();
  quality_sweep();
  sweep_callbacks();
  {
    int pb;
    for (pb = 181; pb <= 192; pb++) probe_big(pb, pb & 1);
    probe_big(200, 1);
    probe_big(183, 0);
    probe_big(165, 1);
    probe_big(166, 1);
    probe_big(164, 1);
    probe_big(163, 0);
  }
}

/* ============================================================================================ scenario 5: callbacks, next state, dispatcher, control, status */
static int g_fake_calls;
static const short *g_fk_in, *g_fk_out;
static int fake_handler(V29Rx *o, short *in, short *out, unsigned short *n)
{
  int k;
  g_fake_calls++;
  emit(0, "fake call %d in+%ld out+%ld n=%u in0=%d", g_fake_calls, (long)(in - g_fk_in), (long)(out - g_fk_out), *n, in[0]);
  /* consumes part of the block, produces any count (also negative, large) */
  switch (g_fake_calls % 6) {
    case 0: *n = (unsigned short)(*n / 2); out[0] = 0x1111; o->st.b[1] ^= 4; return 1;
    case 1: *n = 0; out[0] = 0x2222; out[1] = 0x2223; return 2;
    case 2: *n = (unsigned short)(*n > 3 ? *n - 3 : 0); return -1;
    case 3: *n = 0; return 0x12345;
    case 4: k = *n; *n = (unsigned short)(k > 40 ? k - 40 : 0); return 5;
    default: *n = 0; return -70000;
  }
}
static void scen5(void)
{
  int trial, a, b0c, b0d;
  register_tables();
  capture(0);
  /* (a) the FSE callbacks directly, on engines warmed up by real signals */
  for (trial = 0; trial < 36; trial++) {
    Sess s;
    int rate = trial & 1, sub;
    sess_new(&s, rate, (trial / 2) & 3, trial % 3 == 0 ? 0 : 20 + 20 * (trial % 4), 0, (trial / 4) & 1);
    for (sub = 0; sub < 90; sub++) {
      V29Rx *o = s.o;
      FaxFse *f = &o->bundle->fse;
      V29RxLoop *l = &o->bundle->loop;
      short angle, mag, mag2[3];
      int which = (int)(rnd() % 3u), ret, k;
      static const int cnts[6] = {0, 0x7ffe, 0x7fff, 0x8000 - 1, 5, 0x1234};
      static const int c18s[8] = {-1, 0, 0x7f, 0x80, 0x81, 0x17d, 0x17e, 0x17f};
      f->idx = (short)rmod(20);
      for (k = 0; k < 20; k++) {
        f->outI[k] = (short)(rpct(50) ? (int)rmod(8000) - 4000 : (int)rmod(60000) - 30000);
        f->outQ[k] = (short)(rpct(50) ? (int)rmod(8000) - 4000 : (int)rmod(60000) - 30000);
      }
      l->avgA = (short)rnd(); l->avgB = (short)rnd(); l->symI0 = (short)rnd(); l->symQ0 = (short)rnd(); l->symI1 = (short)rnd(); l->symQ1 = (short)rnd(); l->symI2 = (short)rnd(); l->symQ2 = (short)rnd();
      l->cnt = (short)cnts[rnd() % 6u];
      l->phaseCnt = (short)c18s[rnd() % 8u];
      l->reg = (short)rnd();
      l->idx = (short)(rnd() & 7u);
      l->prev = (short)rnd();
      angle = (short)rnd();
      mag = (short)rnd();
      mag2[0] = mag;
      mag2[1] = 0x1111;
      mag2[2] = 0x2222;
      f->muIdx = (short)(rnd() & 1u);
      f->eqForce = (int)(rnd() & 1u);
      d_fse("cbpre", f);
      emit(0, "cbpre loop h=%d,%d,%d,%d,%d,%d,%d,%d rate=%d idx=%d reg=%d cnt18=%d prev=%d cnt=%d angle=%d", l->avgA, l->avgB, l->symI0, l->symQ0, l->symI1, l->symQ1, l->symI2, l->symQ2, l->rate, l->idx, l->reg, l->phaseCnt, l->prev,
           l->cnt, angle);
      stack_poison();
      switch (which) {
        case 0: {
          int pre18 = (short)l->phaseCnt, dd = (int)(short)((unsigned short)angle - (unsigned short)l->prev);
          ret = V29RX_epoch_det(f, &angle, mag2);
          cov_hit(CV_C_EPOCH);
          if (dd < 0) cov_hit(CV_C_EPOCH_NEG);
          if (angle == V29RX_DEC_ANGLE[4]) cov_hit(CV_C_EPOCH_H0);
          if (angle == V29RX_DEC_ANGLE[7]) cov_hit(CV_C_EPOCH_H2);
          if (pre18 > 0x80) {
            cov_hit(CV_C_EPOCH_CNT);
            if (f->cfg.cb == V29RX_eq_train) cov_hit(CV_C_EPOCH_SWITCH); else cov_hit(CV_C_EPOCH_NOSW);
          }
          break;
        }
        case 1:
          if (l->reg & 1) cov_hit(CV_C_TRAIN_ONE); else cov_hit(CV_C_TRAIN_ZERO);
          ret = V29RX_eq_train(f, &angle, mag2);
          cov_hit(CV_C_TRAIN);
          if (l->rate == 0) cov_hit(CV_C_TRAIN_R0); else cov_hit(CV_C_TRAIN_R1);
          if (f->cfg.cb == V29RX_decision) cov_hit(CV_C_TRAIN_END);
          if (l->cnt == 0x4000) cov_hit(CV_C_TRAIN_WRAP);
          break;
        default:
          ret = V29RX_decision(f, &angle, mag2);
          cov_hit(CV_C_DEC);
          if (l->rate) cov_hit(CV_C_DEC_R9600); else cov_hit(CV_C_DEC_R7200);
          if (l->cnt == 0x4000) cov_hit(CV_C_DEC_WRAP);
          if (l->rate && (ret & 8)) cov_hit(CV_C_DEC_BIT3);
          break;
      }
      emit(0, "callback %d ret=%d angle=%d mag=%d %d %d", which, ret, angle, mag2[0], mag2[1], mag2[2]);
      d_fse("cbpost", f);
      emit(0, "cbpost loop h=%d,%d,%d,%d,%d,%d,%d,%d idx=%d reg=%d cnt18=%d prev=%d cnt=%d", l->avgA, l->avgB, l->symI0, l->symQ0, l->symI1, l->symQ1, l->symI2, l->symQ2, l->idx, l->reg, l->phaseCnt, l->prev, l->cnt);
    }
    sess_free(&s);
    trial_end("callbacks");
  }
  /* (b) RxNextStateV29 for every state x rate x debug level, then one handler call */
  for (a = 0; a < 12 * 2 * 2; a++) {
    static const int sts[12] = {0, 1, 2, 3, 4, 5, 6, -1, 7, 0x7fff, 2, 4};
    int st = sts[a % 12], rate = (a / 12) & 1, dbg = (a / 24) & 1;
    int w[6];
    V29Rx *o;
    make_rcfg(w, rate, a & 1);
    o = rx_make(NULL, w, 0);
    o->hdx->state = (short)st;
    o->hdx->count = (short)(int)(rnd() & 0x7fu);
    o->st.w = (int)rnd();
    dbg_level(dbg ? 2 : 0);
    d_rx("pre", o);
    stack_poison();
    RxNextStateV29(o);
    dbg_level(0);
    d_rx("next", o);
    if (dbg) cov_hit(CV_C_NEXT_DBG);
    if (st >= 0 && st <= 4) {
      unsigned short n = (unsigned short)(1 + rmod(160));
      int ret, k;
      static short blk[512];
      for (k = 0; k < 512; k++) blk[k] = (short)((int)(rnd() & 0xfffu) - 0x800);
      for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
      ret = o->hdx->fn(o, blk, g_out, &n);
      emit(0, "next+handler ret=%d n=%u out=%016llx", ret, n, fnv_n(g_out, 1024));
      d_rx("after", o);
    }
    V29RX_delete(o);
    trial_end("next");
  }
  /* (c) the dispatcher with a harness handler */
  for (trial = 0; trial < 60; trial++) {
    int w[6], n, k;
    V29Rx *o;
    unsigned short nn;
    static short blk[512];
    make_rcfg(w, trial & 1, 0);
    o = rx_make(NULL, w, 0);
    o->hdx->fn = fake_handler;
    g_fake_calls = trial % 6;
    n = trial % 9 == 0 ? 0 : 1 + (int)rmod(150);
    for (k = 0; k < 512; k++) blk[k] = (short)(0x6000 + k);
    for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
    nn = (unsigned short)n;
    o->st.w = (int)rnd();
    g_fk_in = blk;
    g_fk_out = g_out;
    k = V29RX_modem(o, blk, g_out, &nn);
    emit(0, "fake modem n=%d ret=%08x nn=%u calls=%d out=%016llx", n, u(k), nn, g_fake_calls, fnv_n(g_out, 1024));
    cov_hit(CV_C_FAKE);
    if (trial % 6 == 2 || trial % 6 == 5) cov_hit(CV_C_FAKE_NEG);
    if (trial % 6 == 0 || trial % 6 == 4) cov_hit(CV_C_FAKE_PART);
    d_rx("fake", o);
    V29RX_delete(o);
    trial_end("fake");
  }
  /* (d) V29RX_control for every b0c x four b0d values, then a block */
  for (b0c = 0; b0c < 256; b0c++) {
    for (b0d = 0; b0d < 8; b0d++) {
      static const int d4[8] = {0, 0x10, 2, 0x12, 0x20, 0x30, 1, 0xff};
      FaxV29Ctl ctl;
      int w[6], k, ret;
      V29Rx *o;
      unsigned short nn = 60;
      static short blk[512];
      make_rcfg(w, (b0c + b0d) & 1, 0);
      o = rx_make(NULL, w, 0);
      memset(&ctl, 0, sizeof ctl);
      ctl.w4 = (int)rnd();
      ctl.b0c = (unsigned char)b0c;
      ctl.b0d = (unsigned char)d4[b0d];
      ret = V29RX_control(o, &ctl);
      emit(0, "control c=%02x d=%02x ret=%d", b0c, d4[b0d], ret);
      cov_hit(CV_C_CTL);
      if (d4[b0d] & 0x10) cov_hit(CV_C_CTL_HOLD);
      if (d4[b0d] & 2) cov_hit(CV_C_CTL_RECREATE);
      if (b0c & 8) cov_hit(CV_C_CTL_W0);
      if (b0c & 0x20) cov_hit(CV_C_CTL_W20);
      for (k = 0; k < 512; k++) blk[k] = (short)((int)(rnd() & 0x1fffu) - 0x1000);
      for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
      V29RX_modem(o, blk, g_out, &nn);
      emit(0, "control run nn=%u out=%016llx", nn, fnv_n(g_out, 1024));
      if (b0c % 17 == 0) d_rx("ctl", o);
      emit(0, "control hold=%d w0=%d w20=%d w2=%08x", o->hdx->hold, o->bundle->w0, o->bundle->eqGate, u(o->cfg.w[2]));
      V29RX_delete(o);
      trial_end("ctl");
    }
  }
  emit(0, "control null %d", V29RX_control((V29Rx *)NULL, NULL));
  cov_hit(CV_C_CTL_NULL);
  {
    FaxV29Status dummy;
    V29Rx *o = rx_make(NULL, NULL, 0);
    memset(&dummy, 0x55, sizeof dummy);
    emit(0, "status %d %d", V29RX_status(o, &dummy), V29RX_status(o, NULL));
    prt_status("status", &dummy);
    cov_hit(CV_C_STATUS);
    cov_hit(CV_C_STATUS_NULL);
    V29RX_delete(o);
    trial_end("status");
  }
}

/* ============================================================================================ main */
int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R23_SEED");
  g_verbose = getenv("R23_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)strtoul(sd, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)scen * 7919u;
  defaults_init();
  cov_setup(scen);
  switch (scen) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    case 4: scen4(); break;
    case 5: scen5(); break;
    default: fprintf(stderr, "scenario 0..5\n"); return 2;
  }
  cov_finish();
  return 0;
}
