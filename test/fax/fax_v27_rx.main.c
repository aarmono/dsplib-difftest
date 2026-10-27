/*
 * R21 differential harness: the fax V.27ter RECEIVER of dsplibs.o (V27RX_create / _delete / _modem / _control / _status, RxHdx{Start,EpochDet,Prtcol,Data,Idle,Error}V27, RxNextStateV27, DemodDataV27,
 * DataCarrierDetectV27 / CarrierDetectV27 / EpochDetectV27 / QualityDetectV27 / GetSNRV27, the FSE callbacks V27RX_epoch_det / _decision / _eq_train, the v27rx_* VMI wrappers and the blob-LOCAL
 * init_vmi_v27rx) and, as stand-alone units, the blob's FPM_FSE_* / FPM_SRE_* against the typed FAXFSE_* / FAXSRE_* (src/fax/fax_equalizer.c, src/fax/fax_timing.c), vs src/fax/fax_v27_rx.c + tables.  The blob run
 * (-DTEST_BLOB, linked with ../dsplibs.o, init_vmi_v27rx made global by objcopy) and the port run (-m32 -O0 / -O2, x86-64 against the recorded blob output) execute the same code; library
 * test/fax/r21_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation, deterministic heap with block ids / red zones / poisoned frees, callee recorders sysdep_malloc /
 * sysdep_free / dsplibs_debug_printf).  The R18 / R20 / FPM callees are the real ones on both sides (the blob's own in the blob run, the files of src/ in the port run): the transmit signal of the
 * end-to-end streams is made by the blob's V27TX_* in the blob run and by src/fax/fax_v27_tx.c in the port run (the two are proven identical by R20), so the receiver sees the same samples.
 * Integer signals only (no libm).  Nothing printed depends on the host layout: objects are dumped member by member (pointers as table+index / block id, the content of every malloc()ed block
 * as a hash).  The hdx.s14 flag (never written by the blob's create: heap garbage) is set explicitly to 0 / 1 right after every create, on both sides.
 *
 * Scenarios (each its own process; argv[1]):
 *   0 tables: every R21 table dumped, the pointer tables as table + index, the default SRE / FSE config words (blob FPM_SRE_CFG / FPM_FSE_CFG vs the typed defaults), v27rx_message -3 .. 12;
 *   1 lifecycle: V27RX_create (NULL object / supplied with live hdx + bundle / hdx NULL / bundle NULL / live re-create / create(o, o), NULL / custom / fuzzed cfg words, rates 2400 / 4800 /
 *     other, debug level 0 and 2), delete, run-after-create, forked probes: a failing malloc #0 .. #27, a supplied object without bundle, the rate-changing re-creates (red zone verdicts);
 *   2 VMI: fax_init_vmi_v27rx (blob: init_vmi_v27rx) with debug levels 0 .. 3, v27rx_create / process / status / control / delete / message on garbage-initialised FaxVmiV27 objects;
 *   3 receive streams: the transmitter of the same side (V27TX, R20) -> impairments (gain, noise, DC, clock slip, carrier loss, noise bursts, re-trains) -> V27RX_modem in random chunks, both
 *     rates, short and long training, control (hold, re-create, gates), status, the whole engine dumped after EVERY call;
 *   4 handlers: every RxHdx*V27 / DemodDataV27 / DataCarrierDetectV27 / QualityDetectV27 / RxNextStateV27 x hdx state x rate x poked decision state, on engines warmed up by real signals;
 *   5 the FSE callbacks directly, RxNextStateV27 for every state / rate / s0a / debug level, V27RX_modem with a harness handler, V27RX_control for 512 flag-byte combinations, V27RX_status;
 *   6 FAXFSE_* vs FPM_FSE_*: random configs, a real signal (transmitter -> matched filter -> SRE) through the equalizer with harness callbacks, re-init (fresh = 0), pokes of every flag;
 *   7 FAXSRE_* vs FPM_SRE_*: random configs, real signals, re-init with a growing / shrinking filter, pokes.
 * R21_VERBOSE=1 prints every line (debugging), R21_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r21_common.h"
#include "fax/fax_v27_rx_tables.h"
#include "fax/fax_v27_tx_tables.h"
#include "fax/fax_v21_tables.h"
#include "fpm/fpm_tables.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
extern int init_vmi_v27rx(FaxVmiV27 *m, unsigned short rate, int unused, void *user);
#define fax_init_vmi_v27rx init_vmi_v27rx
/* the blob's own FSE / SRE: a typed FaxFse / FaxSre IS the blob's raw block at -m32 (layout pinned by the _Static_asserts) */
extern void FPM_FSE_init(void *fse, const int *cfg, int fresh);
extern void FPM_FSE_free(void *fse);
extern short FPM_FSE_receive(void *fse, short *in, short *out, unsigned short n);
extern void FPM_SRE_init(void *sre, const int *cfg, int fresh);
extern void FPM_SRE_free(void *sre);
extern unsigned int FPM_SRE_recover(void *sre, unsigned short *in, short *out, short n);
extern const int FPM_SRE_CFG[14];
extern const int FPM_FSE_CFG[14];
#define FAXFSE_init(f, c, fr) FPM_FSE_init((void *)(f), (const int *)(c), (fr))
#define FAXFSE_free(f) FPM_FSE_free((void *)(f))
#define FAXFSE_receive(f, i, o, n) FPM_FSE_receive((void *)(f), (i), (o), (n))
#define FAXSRE_init(s, c, fr) FPM_SRE_init((void *)(s), (const int *)(c), (fr))
#define FAXSRE_free(s) FPM_SRE_free((void *)(s))
#define FAXSRE_recover(s, i, o, n) FPM_SRE_recover((void *)(s), (unsigned short *)(i), (o), (n))
#else
#define SIDE_BLOB 0
#endif

extern unsigned int dsplibs_debug_level;
extern int FPM_MRF_init(FpmMrf *mrf, const int *cfg, int fresh);
extern unsigned int FPM_MRF_filter(FpmMrf *mrf, short *in, short *out, short n);
extern void FPM_MRF_free(FpmMrf *mrf);
extern void FPM_MTD_delete(FpmMtd *mtd);
extern void FPM_atan(short i, short q, unsigned short *angle_out);

#include "fax/fax_modem_views.h"
#if UINTPTR_MAX != 0xffffffffu
/* x86-64: src/fax/fax_modem_wrappers.c references ALL eleven accessors of src/fax/fax_modem_views.h; this test uses the V.27ter transmit ones (src/fax/fax_v27_tx.c) and the receive one (src/fax/fax_v27_rx.c).  The other
 * seven belong to the V.17 / V.29 engines (R22 - R26): stand-ins that abort (no scenario of this test reaches the wrappers that use them). */
static void *no_engine(void) { fprintf(stderr, "HARNESS/MODEL VIOLATION: accessor of another engine called\n"); exit(3); }
FaxSdm *faxw_v17_txsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSmcV17 *faxw_v17_txsmc(void *o) { (void)o; return (FaxSmcV17 *)no_engine(); }
unsigned short *faxw_v17_encmode(void *o) { (void)o; return (unsigned short *)no_engine(); }
FaxSdm *faxw_v17_rxsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSdm *faxw_v29_txsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
int *faxw_v29_encsel(void *o) { (void)o; return (int *)no_engine(); }
FaxSdm *faxw_v29_rxsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
#endif

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_TAB, CV_MSG, CV_MSG_OOB, CV_MSG_NEG, CV_CFGDEF,
  CV_L_NEW, CV_L_SUP_LIVE, CV_L_SUP_NOHDX, CV_L_SUP_NOBUNDLE_PROBE, CV_L_RECREATE, CV_L_NULLCFG, CV_L_R2400, CV_L_R4800, CV_L_RBAD, CV_L_DBG, CV_L_DEL, CV_L_PROBE, CV_L_RUN, CV_L_S0A0, CV_L_S0A1,
  CV_L_RATECHG, CV_L_SAMERATE, CV_L_GROW, CV_L_SHRINK,
  CV_V_INIT, CV_V_INIT_DBG, CV_V_CREATE, CV_V_CREATE_NULLCFG, CV_V_CREATE_SUP, CV_V_PROC, CV_V_STATUS, CV_V_CTL, CV_V_DEL, CV_V_MSG, CV_V_R2400, CV_V_R4800,
  CV_S_ST0, CV_S_ST1, CV_S_ST2, CV_S_ST3, CV_S_ST4, CV_S_ST5, CV_S_TO1, CV_S_TO2, CV_S_TO3, CV_S_TO4, CV_S_TO3B, CV_S_IDLE_BACK, CV_S_ERR, CV_S_LOSS, CV_S_Q4E, CV_S_Q4A_FULL, CV_S_HOLD, CV_S_CTL,
  CV_S_RECREATE, CV_S_STATUS, CV_S_R2400, CV_S_R4800, CV_S_SHORT, CV_S_LONG, CV_S_NOISE, CV_S_SLIP, CV_S_LOSSSEG, CV_S_BURST, CV_S_RETRAIN, CV_S_GAINLOW, CV_S_CLIP, CV_S_DC, CV_S_PATHB, CV_S_S22,
  CV_S_N0, CV_S_N1, CV_S_NFULL, CV_S_NRAND, CV_S_BITS, CV_S_DBG, CV_S_CALLBACK_EQ, CV_S_CALLBACK_DEC, CV_S_C40, CV_S_BADRATE, CV_S_USER, CV_S_PROTO, CV_S_DATAOK, CV_S_DROP2, CV_S_ENERGY,
  CV_H_START, CV_H_EPOCH, CV_H_PRTCOL, CV_H_DATA, CV_H_IDLE, CV_H_ERROR, CV_H_STATEDEF, CV_H_HOLD, CV_H_PATHA, CV_H_PATHB, CV_H_CNT, CV_H_E52HI, CV_H_Q50, CV_H_ENERGY, CV_H_Q54, CV_H_S20, CV_H_MTD2,
  CV_H_QUAL0, CV_H_QUAL1, CV_H_QUAL2, CV_H_QUAL3, CV_H_DEMOD, CV_H_DEMOD_MTD, CV_H_SRE_BIG, CV_H_CARR, CV_H_NOCARR, CV_H_CDET_BRANCH, CV_H_IDLE_SMALL, CV_H_NEXT0, CV_H_NEXT1, CV_H_NEXT2, CV_H_NEXT3,
  CV_H_NEXT4, CV_H_NEXTD, CV_H_N0, CV_H_NBIG, CV_H_WARM,
  CV_C_EPOCH, CV_C_EPOCH_HI, CV_C_EPOCH_LO, CV_C_EPOCH_SWITCH, CV_C_EPOCH_THR0, CV_C_EPOCH_THR1, CV_C_TRAIN, CV_C_TRAIN_STEP, CV_C_TRAIN_NOSTEP, CV_C_TRAIN_END, CV_C_TRAIN_FLAG0, CV_C_TRAIN_FLAG1, CV_C_TRAIN_WRAP,
  CV_C_TRAIN_ADJ1, CV_C_TRAIN_ADJ2, CV_C_DEC, CV_C_DEC_R2400, CV_C_DEC_R4800, CV_C_DEC_NEG, CV_C_DEC_BIG, CV_C_DEC_WRAP, CV_C_DEC_D0, CV_C_NEXT_DBG, CV_C_NEXT_C0, CV_C_NEXT_C1, CV_C_NEXT_C2,
  CV_C_NEXT_C3, CV_C_NEXT_C4, CV_C_NEXT_DEF, CV_C_FAKE, CV_C_FAKE_NEG, CV_C_FAKE_PART, CV_C_CTL, CV_C_CTL_HOLD, CV_C_CTL_RECREATE, CV_C_CTL_W0, CV_C_CTL_W10, CV_C_CTL_NULL, CV_C_STATUS, CV_C_STATUS_NULL,
  CV_C_GETSNR, CV_C_EPOCHDET, CV_C_CARRDET,
  CV_F_INIT, CV_F_FRESH0, CV_F_FRESH1, CV_F_RECV, CV_F_C40, CV_F_C44, CV_F_C48, CV_F_C4C, CV_F_MU1, CV_F_PLL_T0, CV_F_PLL_T1, CV_F_PLL_T2, CV_F_PRED, CV_F_PARTIAL, CV_F_MULTI, CV_F_DIAGWRAP, CV_F_DEBUG,
  CV_F_CB_SWITCH, CV_F_NT_SMALL, CV_F_NT_BIG, CV_F_CRRWRAP, CV_F_E52SAT, CV_F_S88, CV_F_FREE, CV_F_R2400, CV_F_R4800, CV_F_N0, CV_F_N1,
  CV_R_INIT, CV_R_FRESH0, CV_R_FRESH1, CV_R_REALLOC, CV_R_RECOVER, CV_R_STATE0, CV_R_STATE1, CV_R_STATE2, CV_R_LOCKED, CV_R_UNLOCK, CV_R_W48_0, CV_R_W48_1, CV_R_W40_0, CV_R_W40_1, CV_R_PPM, CV_R_PPM_RESET,
  CV_R_PPM_FULL, CV_R_PARTIAL, CV_R_RMS_GATE, CV_R_HIST_WRAP, CV_R_R2400, CV_R_R4800, CV_R_FREE, CV_R_DEBUG, CV_R_N0, CV_R_BIG
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_TAB, "tables dumped");
      cov_def(CV_CFGDEF, "default SRE / FSE config words");
      cov_def(CV_MSG, "v27rx_message idx 0..7");
      cov_def(CV_MSG_OOB, "v27rx_message idx 8 (out of table)");
      cov_def(CV_MSG_NEG, "v27rx_message idx > limit / negative");
      break;
    case 1:
      cov_def(CV_L_NEW, "V27RX_create NULL object");
      cov_def(CV_L_SUP_LIVE, "V27RX_create supplied live object");
      cov_def(CV_L_SUP_NOHDX, "V27RX_create supplied object, hdx NULL (bundle live)");
      cov_def(CV_L_SUP_NOBUNDLE_PROBE, "supplied object without bundle probe");
      cov_def(CV_L_RECREATE, "V27RX_create(o, o)");
      cov_def(CV_L_NULLCFG, "V27RX_create NULL cfg");
      cov_def(CV_L_R2400, "V27RX_create rate 2400");
      cov_def(CV_L_R4800, "V27RX_create rate 4800");
      cov_def(CV_L_RBAD, "V27RX_create other rate (error flags)");
      cov_def(CV_L_DBG, "V27RX_create debug level 2");
      cov_def(CV_L_DEL, "V27RX_delete");
      cov_def(CV_L_PROBE, "create malloc-failure probe");
      cov_def(CV_L_RUN, "run after create");
      cov_def(CV_L_S0A0, "cfg word 5 != 0");
      cov_def(CV_L_S0A1, "cfg word 5 == 0");
      cov_def(CV_L_RATECHG, "rate-changing re-create");
      cov_def(CV_L_SAMERATE, "same-rate re-create");
      cov_def(CV_L_GROW, "re-create 4800 -> 2400 (SRE filter grows)");
      cov_def(CV_L_SHRINK, "re-create 2400 -> 4800 (SRE filter shrinks)");
      break;
    case 2:
      cov_def(CV_V_INIT, "fax_init_vmi_v27rx");
      cov_def(CV_V_INIT_DBG, "init debug level 3");
      cov_def(CV_V_CREATE, "v27rx_create");
      cov_def(CV_V_CREATE_NULLCFG, "v27rx_create NULL cfg");
      cov_def(CV_V_CREATE_SUP, "v27rx_create with a supplied engine");
      cov_def(CV_V_PROC, "v27rx_process");
      cov_def(CV_V_STATUS, "v27rx_status");
      cov_def(CV_V_CTL, "v27rx_control");
      cov_def(CV_V_DEL, "v27rx_delete");
      cov_def(CV_V_MSG, "v27rx_message");
      cov_def(CV_V_R2400, "VMI rate 2400");
      cov_def(CV_V_R4800, "VMI rate 4800");
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
      cov_def(CV_S_R2400, "rate 2400 stream");
      cov_def(CV_S_R4800, "rate 4800 stream");
      cov_def(CV_S_SHORT, "short training stream");
      cov_def(CV_S_LONG, "long training stream");
      cov_def(CV_S_NOISE, "noisy stream");
      cov_def(CV_S_SLIP, "clock slip stream");
      cov_def(CV_S_LOSSSEG, "carrier loss segment");
      cov_def(CV_S_BURST, "noise burst");
      cov_def(CV_S_RETRAIN, "transmitter re-train");
      cov_def(CV_S_GAINLOW, "low level stream");
      cov_def(CV_S_CLIP, "clipping stream");
      cov_def(CV_S_DC, "DC offset stream");
      cov_def(CV_S_PATHB, "hdx.s14 != 0 stream (carrier detector path B)");
      cov_def(CV_S_S22, "path B with s22 set");
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
      break;
    case 4:
      cov_def(CV_H_START, "RxHdxStartV27");
      cov_def(CV_H_EPOCH, "RxHdxEpochDetV27");
      cov_def(CV_H_PRTCOL, "RxHdxPrtcolV27");
      cov_def(CV_H_DATA, "RxHdxDataV27");
      cov_def(CV_H_IDLE, "RxHdxIdleV27");
      cov_def(CV_H_ERROR, "RxHdxErrorV27");
      cov_def(CV_H_STATEDEF, "state number outside 0..4");
      cov_def(CV_H_HOLD, "Data with hold");
      cov_def(CV_H_PATHA, "DataCarrierDetectV27 path A");
      cov_def(CV_H_PATHB, "DataCarrierDetectV27 path B");
      cov_def(CV_H_CNT, "loop.cnt > 0x5db");
      cov_def(CV_H_E52HI, "fse.e52 > 0x3fff");
      cov_def(CV_H_Q50, "energy test enabled");
      cov_def(CV_H_ENERGY, "energy drop detected");
      cov_def(CV_H_Q54, "q54 reached 2");
      cov_def(CV_H_S20, "hdx.s20 > 0x4ff");
      cov_def(CV_H_MTD2, "hdx MTD2 fired");
      cov_def(CV_H_QUAL0, "QualityDetectV27 q4a == 0");
      cov_def(CV_H_QUAL1, "QualityDetectV27 q4a < 0x32");
      cov_def(CV_H_QUAL2, "QualityDetectV27 q4a == 0x32");
      cov_def(CV_H_QUAL3, "QualityDetectV27 q4a > 0x32");
      cov_def(CV_H_DEMOD, "DemodDataV27");
      cov_def(CV_H_DEMOD_MTD, "DemodDataV27 aborted by the tone detector");
      cov_def(CV_H_SRE_BIG, "SRE output > 164");
      cov_def(CV_H_CARR, "carrier detected");
      cov_def(CV_H_NOCARR, "no carrier");
      cov_def(CV_H_CDET_BRANCH, "carrier detect with the gate off");
      cov_def(CV_H_IDLE_SMALL, "Idle: decision error small");
      cov_def(CV_H_N0, "n = 0");
      cov_def(CV_H_NBIG, "n >= 150");
      cov_def(CV_H_WARM, "warmed up engine");
      cov_def(CV_C_GETSNR, "GetSNRV27");
      cov_def(CV_C_EPOCHDET, "EpochDetectV27");
      cov_def(CV_C_CARRDET, "CarrierDetectV27");
      break;
    case 5:
      cov_def(CV_C_EPOCH, "V27RX_epoch_det");
      cov_def(CV_C_EPOCH_HI, "epoch: s1c above the threshold, no switch");
      cov_def(CV_C_EPOCH_LO, "epoch: s1c below the threshold");
      cov_def(CV_C_EPOCH_SWITCH, "epoch: switch to eq_train");
      cov_def(CV_C_EPOCH_THR0, "epoch: c10 == 0");
      cov_def(CV_C_EPOCH_THR1, "epoch: c10 != 0");
      cov_def(CV_C_TRAIN, "V27RX_eq_train");
      cov_def(CV_C_TRAIN_STEP, "train: phase step");
      cov_def(CV_C_TRAIN_NOSTEP, "train: no step");
      cov_def(CV_C_TRAIN_END, "train: switch to decision");
      cov_def(CV_C_TRAIN_FLAG0, "train: c10 == 0");
      cov_def(CV_C_TRAIN_FLAG1, "train: c10 != 0");
      cov_def(CV_C_TRAIN_WRAP, "train / decision: cnt wrap 0x8000");
      cov_def(CV_C_TRAIN_ADJ1, "train: angle difference > 0x4000");
      cov_def(CV_C_TRAIN_ADJ2, "train: angle difference < -0x4000");
      cov_def(CV_C_DEC, "V27RX_decision");
      cov_def(CV_C_DEC_R2400, "decision 2400");
      cov_def(CV_C_DEC_R4800, "decision 4800");
      cov_def(CV_C_DEC_NEG, "decision d < 0");
      cov_def(CV_C_DEC_WRAP, "decision cnt wrap");
      cov_def(CV_C_DEC_D0, "decision d == 0");
      cov_def(CV_C_NEXT_DBG, "NextState debug strings");
      cov_def(CV_C_NEXT_C0, "RxNextStateV27 state 0");
      cov_def(CV_C_NEXT_C1, "RxNextStateV27 state 1");
      cov_def(CV_C_NEXT_C2, "RxNextStateV27 state 2");
      cov_def(CV_C_NEXT_C3, "RxNextStateV27 state 3");
      cov_def(CV_C_NEXT_C4, "RxNextStateV27 state 4");
      cov_def(CV_C_NEXT_DEF, "RxNextStateV27 default");
      cov_def(CV_C_FAKE, "V27RX_modem with a harness handler");
      cov_def(CV_C_FAKE_NEG, "handler returns a negative count");
      cov_def(CV_C_FAKE_PART, "handler consumes part of the block");
      cov_def(CV_C_CTL, "V27RX_control");
      cov_def(CV_C_CTL_HOLD, "control hold");
      cov_def(CV_C_CTL_RECREATE, "control re-create");
      cov_def(CV_C_CTL_W0, "control b0c bit 3");
      cov_def(CV_C_CTL_W10, "control b0c bit 5");
      cov_def(CV_C_CTL_NULL, "control NULL");
      cov_def(CV_C_STATUS, "V27RX_status");
      cov_def(CV_C_STATUS_NULL, "status NULL");
      break;
    case 6:
      cov_def(CV_F_INIT, "FSE init");
      cov_def(CV_F_FRESH0, "FSE init fresh = 0");
      cov_def(CV_F_FRESH1, "FSE init fresh = 1");
      cov_def(CV_F_RECV, "FSE receive");
      cov_def(CV_F_C40, "c40 set");
      cov_def(CV_F_C44, "c44 set");
      cov_def(CV_F_C48, "c48 set (phase predictor)");
      cov_def(CV_F_C4C, "c4c set");
      cov_def(CV_F_MU1, "muIdx 1");
      cov_def(CV_F_PLL_T0, "PLL table 0");
      cov_def(CV_F_PLL_T1, "PLL table 1");
      cov_def(CV_F_PLL_T2, "PLL table 2");
      cov_def(CV_F_PRED, "predictor output non-zero");
      cov_def(CV_F_PARTIAL, "block shorter than the samples to the next symbol");
      cov_def(CV_F_MULTI, "block with several symbols");
      cov_def(CV_F_DIAGWRAP, "diag ring wrapped");
      cov_def(CV_F_DEBUG, "Decoder Error debug line");
      cov_def(CV_F_CB_SWITCH, "callback switched the callback");
      cov_def(CV_F_NT_SMALL, "nt <= 8");
      cov_def(CV_F_NT_BIG, "nt >= 60");
      cov_def(CV_F_CRRWRAP, "carrier table index wrap");
      cov_def(CV_F_E52SAT, "e52 saturated");
      cov_def(CV_F_S88, "s88 counting");
      cov_def(CV_F_FREE, "FSE free");
      cov_def(CV_F_R2400, "2400 tables");
      cov_def(CV_F_R4800, "4800 tables");
      cov_def(CV_F_N0, "receive n = 0");
      cov_def(CV_F_N1, "receive n = 1");
      break;
    case 7:
      cov_def(CV_R_INIT, "SRE init");
      cov_def(CV_R_FRESH0, "SRE init fresh = 0");
      cov_def(CV_R_FRESH1, "SRE init fresh = 1");
      cov_def(CV_R_REALLOC, "SRE buffers re-allocated (filter grows)");
      cov_def(CV_R_RECOVER, "SRE recover");
      cov_def(CV_R_STATE0, "SRE state 0");
      cov_def(CV_R_STATE1, "SRE state 1");
      cov_def(CV_R_STATE2, "SRE state 2 (locked)");
      cov_def(CV_R_LOCKED, "SRE state 1 -> 2");
      cov_def(CV_R_UNLOCK, "SRE state 2 -> 1");
      cov_def(CV_R_W48_0, "w48 = 0");
      cov_def(CV_R_W48_1, "w48 = 1");
      cov_def(CV_R_W40_0, "w40 = 0");
      cov_def(CV_R_W40_1, "w40 = 1");
      cov_def(CV_R_PPM, "clock offset estimate");
      cov_def(CV_R_PPM_RESET, "clock offset estimator reset (s8e)");
      cov_def(CV_R_PPM_FULL, "clock offset estimator full (s84 >= s8c)");
      cov_def(CV_R_PARTIAL, "block shorter than the samples to the next output");
      cov_def(CV_R_RMS_GATE, "energy gate closes the loop");
      cov_def(CV_R_HIST_WRAP, "history wrap");
      cov_def(CV_R_R2400, "2400 tables");
      cov_def(CV_R_R4800, "4800 tables");
      cov_def(CV_R_FREE, "SRE free");
      cov_def(CV_R_DEBUG, "TimingVxx debug line");
      cov_def(CV_R_N0, "recover n = 0");
      cov_def(CV_R_BIG, "recover n > 100");
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
  if (!strncmp(b, "V27RX_STATE_START", 17)) cov_hit(CV_C_NEXT_C0);
  if (!strncmp(b, "V27RX_STATE_EPOCH_DET", 21)) cov_hit(CV_C_NEXT_C1);
  if (!strncmp(b, "V27RX_STATE_PROTOCOL", 20)) cov_hit(CV_C_NEXT_C2);
  if (!strncmp(b, "V27RX_STATE_DATA", 16)) cov_hit(CV_C_NEXT_C3);
  if (!strncmp(b, "V27RX_STATE_IDLE", 16)) cov_hit(CV_C_NEXT_C4);
  if (!strncmp(b, "V27RX_DEFAULT", 13)) cov_hit(CV_C_NEXT_DEF);
  if (!strncmp(b, "Decoder Error", 13)) cov_hit(CV_F_DEBUG);
  if (!strncmp(b, "TimingVxx", 9)) cov_hit(CV_R_DEBUG);
  if (!strncmp(b, "ERROR: SRE buffer", 17)) cov_hit(CV_H_SRE_BIG);
  if (!strncmp(b, "Decision error is small", 23)) cov_hit(CV_H_IDLE_SMALL);
  if (!strncmp(b, "sudden energy", 13)) cov_hit(CV_H_ENERGY);
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
static const void *cp(int w) { return cfgp_ptr(w); } /* a CFGP word as the address of its table */
static const short *g_agcbase; /* the AGCv27_CFG alpha table (a blob LOCAL: named by position) */

static void register_tables(void)
{
  htab_add("FSE_IFILT_2400", V27RX_FSE_IFILT_2400, sizeof V27RX_FSE_IFILT_2400);
  htab_add("FSE_IFILT_4800", V27RX_FSE_IFILT_4800, sizeof V27RX_FSE_IFILT_4800);
  htab_add("FSE_QFILT_2400", V27RX_FSE_QFILT_2400, sizeof V27RX_FSE_QFILT_2400);
  htab_add("FSE_QFILT_4800", V27RX_FSE_QFILT_4800, sizeof V27RX_FSE_QFILT_4800);
  htab_add("CRR_2400", V27RX_CRR_TABLE_2400, sizeof V27RX_CRR_TABLE_2400);
  htab_add("CRR_4800", V27RX_CRR_TABLE_4800, sizeof V27RX_CRR_TABLE_4800);
  htab_add("FSE_PLLK1_2400", V27RX_FSE_PLLK1_2400, sizeof V27RX_FSE_PLLK1_2400);
  htab_add("FSE_PLLK1_4800", V27RX_FSE_PLLK1_4800, sizeof V27RX_FSE_PLLK1_4800);
  htab_add("FSE_PLLK2_2400", V27RX_FSE_PLLK2_2400, sizeof V27RX_FSE_PLLK2_2400);
  htab_add("FSE_PLLK2_4800", V27RX_FSE_PLLK2_4800, sizeof V27RX_FSE_PLLK2_4800);
  htab_add("MRF_FILT_2400", V27RX_MRF_FILT_2400, sizeof V27RX_MRF_FILT_2400);
  htab_add("MRF_FILT_4800", V27RX_MRF_FILT_4800, sizeof V27RX_MRF_FILT_4800);
  htab_add("SRE_FILT_2400", V27RX_SRE_FILT_2400, sizeof V27RX_SRE_FILT_2400);
  htab_add("SRE_FILT_4800", V27RX_SRE_FILT_4800, sizeof V27RX_SRE_FILT_4800);
  htab_add("XB_2400", V27RX_XB_COFFS_2400, sizeof V27RX_XB_COFFS_2400);
  htab_add("XB_4800", V27RX_XB_COFFS_4800, sizeof V27RX_XB_COFFS_4800);
  htab_add("XCLOCK_2400", V27RX_XCLOCK_2400, sizeof V27RX_XCLOCK_2400);
  htab_add("XCLOCK_4800", V27RX_XCLOCK_4800, sizeof V27RX_XCLOCK_4800);
  htab_add("YCLOCK_2400", V27RX_YCLOCK_2400, sizeof V27RX_YCLOCK_2400);
  htab_add("YCLOCK_4800", V27RX_YCLOCK_4800, sizeof V27RX_YCLOCK_4800);
  htab_add("SRE_PLLK1_2400", V27RX_SRE_PLLK1_2400, sizeof V27RX_SRE_PLLK1_2400);
  htab_add("SRE_PLLK1_4800", V27RX_SRE_PLLK1_4800, sizeof V27RX_SRE_PLLK1_4800);
  htab_add("SRE_PLLK2_2400", V27RX_SRE_PLLK2_2400, sizeof V27RX_SRE_PLLK2_2400);
  htab_add("SRE_PLLK2_4800", V27RX_SRE_PLLK2_4800, sizeof V27RX_SRE_PLLK2_4800);
  htab_add("MTD_2400", V27_MTD_COEFF_2400, sizeof V27_MTD_COEFF_2400);
  htab_add("MTD_4800", V27_MTD_COEFF_4800, sizeof V27_MTD_COEFF_4800);
  htab_add("MTD_V21CH2", V21_CHAN2_MTD_COEFF, sizeof V21_CHAN2_MTD_COEFF);
  htab_add("PMAP_2400", V27RX_DEC_PMAP_2400, sizeof V27RX_DEC_PMAP_2400);
  htab_add("PMAP_4800", V27RX_DEC_PMAP_4800, sizeof V27RX_DEC_PMAP_4800);
  htab_add("LAST_2400", V27RX_DEC_LAST_PHASE_2400, sizeof V27RX_DEC_LAST_PHASE_2400);
  htab_add("LAST_4800", V27RX_DEC_LAST_PHASE_4800, sizeof V27RX_DEC_LAST_PHASE_4800);
  g_agcbase = (const short *)cp(AGCv27_CFG[3]);
}
static const char *rxfn(V27RxHdxFn f)
{
  if (f == RxHdxStartV27) return "Start";
  if (f == RxHdxEpochDetV27) return "EpochDet";
  if (f == RxHdxPrtcolV27) return "Prtcol";
  if (f == RxHdxDataV27) return "Data";
  if (f == RxHdxIdleV27) return "Idle";
  if (f == RxHdxErrorV27) return "Error";
  return f ? "other" : "NULL";
}
static int hcb_a(FaxFse *f, short *angle, short *mag);
static int hcb_b(FaxFse *f, short *angle, short *mag);
static int hcb_c(FaxFse *f, short *angle, short *mag);
static int hcb_d(FaxFse *f, short *angle, short *mag);
static const char *cbname(FaxFseCb f)
{
  if (f == V27RX_epoch_det) return "epoch_det";
  if (f == V27RX_eq_train) return "eq_train";
  if (f == V27RX_decision) return "decision";
  if (f == hcb_a) return "hcb_a";
  if (f == hcb_b) return "hcb_b";
  if (f == hcb_c) return "hcb_c";
  if (f == hcb_d) return "hcb_d";
  return f ? "other" : "NULL";
}
static void d_agc(const char *t, const FpmAgc *a)
{
  emit(0, "%s AGC a=%08x %08x %08x alpha=%d beta=%d w5=%08x act=%d det=%d hist=%d rcp=%d sh=%d frz=%d", t, u(a->a.w[0]), u(a->a.w[1]), u(a->a.w[2]), (int)(a->alpha - g_agcbase),
       (int)(a->beta - (const short *)cp(AGCv27_CFG[4])), u(a->w5), a->active, a->detect, a->hist, a->rcp, a->shift, a->freeze); /* AGC.s22 is never written by FPM_AGC_init */
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
static void d_rx(const char *t, const V27Rx *o)
{
  char p1[80], p2[80], p3[80], p4[80], p5[80];
  emit(0, "%s RX cfg=%08x %08x %08x %08x %08x %08x %08x st=%08x fseI=%s fseQ=%s idx=%s tapsI=%s tapsQ=%s nt=%d w38=%d w3c=%d s40=%d w44=%d w48=%d s4c=%d", t, u(o->cfg.w[0]), u(o->cfg.w[1]),
       u(o->cfg.w[2]), u(o->cfg.w[3]), u(o->cfg.w[4]), u(o->cfg.w[5]), u(o->cfg.w[6]), u(o->st.w), (hblk(p1, 80, o->fseI), p1), (hblk(p2, 80, o->fseQ), p2),
       o->bundle ? (o->fseIdx == &o->bundle->fse.idx ? "&fse.idx" : "other") : "-", (hblk(p3, 80, o->fseTapsI), p3), (hblk(p4, 80, o->fseTapsQ), p4), o->fseNt, o->w38, o->w3c, o->s40, o->w44, o->w48,
       o->s4c);
  if (o->hdx) {
    const V27RxHdx *h = o->hdx;
    emit(0, "%s RXH hold=%d rate=%d s0a=%d fn=%s state=%d count=%d s14=%d buf=%s s20=%d s22=%d", t, h->hold, h->rate, h->longTrain, rxfn(h->fn), h->state, h->count, h->s14, (hblk(p5, 80, h->buf), p5), h->sinceTone,
         h->toneSearch);
    d_mtd(t, h->mtd);
    d_mtd(t, h->mtd2);
    d_agc(t, &h->agc);
  }
  if (o->bundle) {
    const V27RxBundle *b = o->bundle;
    const V27RxLoop *l = &b->loop;
    char pm[40], la[40];
    emit(0, "%s RXB w0=%d w4=%d w8=%d wc=%d w10=%d buf1=%s buf2=%s q=%d,%d,%d,%d,%d,%d,%d", t, b->w0, b->sreGate, b->pllGate, b->wc, b->eqGate, (hblk(p1, 80, b->buf1), p1), (hblk(p2, 80, b->buf2), p2), b->decErrSmooth, b->qualBlocks,
         b->errLimit, b->qualityOk, b->energyTest, b->refLevel, b->refBlocks);
    emit(0, "%s RXB loop h=%d,%d,%d,%d,%d,%d c0c=%d c10=%d mask=%u idx=%d pmap=%s s1c=%d avg=%d last=%s prev=%d cnt=%d", t, l->symI0, l->symQ0, l->symI1, l->symQ1, l->symI2, l->symQ2, l->rate4800, l->shortTrain, l->mask, l->idx,
         dptr(l->pmap, pm, 40), l->phaseCnt, l->avg, dptr(l->last, la, 40), l->prev, l->cnt);
    emit(0, "%s RXB sdm n=%u mask=%04x nmask=%04x reg=%04x cnt=%u f1=%u f2=%u", t, b->sdm.n, b->sdm.mask, b->sdm.nmask, b->sdm.reg, b->sdm.cnt, b->sdm.f1, b->sdm.f2);
    d_mrf(t, &b->mrf);
    d_agc(t, &b->agc);
    d_sre(t, &b->sre);
    d_fse(t, &b->fse);
  }
  (void)p4;
}
/* the default SRE / FSE config blocks of the blob (FPM_SRE_CFG / FPM_FSE_CFG) / of the port (fax_*_cfg_default): the same words (scenario 0 compares them) */
static FaxSreCfg h_sre_def;
static FaxFseCfg h_fse_def;
static void defaults_init(void)
{
#ifdef TEST_BLOB
  memcpy(&h_sre_def, FPM_SRE_CFG, sizeof h_sre_def);
  memcpy(&h_fse_def, FPM_FSE_CFG, sizeof h_fse_def);
#else
  h_sre_def = fax_sre_cfg_default;
  h_fse_def = fax_fse_cfg_default;
#endif
}
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

/* the transmitter (src/fax/fax_v27_tx.c in the port run, the blob's own in the blob run): config like R20's make_cfg */
static void make_tcfg(int *w, int rate, int longtrain)
{
  int i;
  for (i = 0; i < 8; i++) w[i] = V27TX_CFG[i];
  w[0] = (w[0] & 0xffff) | ((rate == 0 ? 0x960 : 0x12c0) << 16);
  w[6] = longtrain ? 0 : 1; /* s0e = 1: the long alternation / EQ conditioning */
}
typedef struct {
  V27Tx *o;
  int rate, frm;
} Gen;
static void gen_new(Gen *g, int rate, int longtrain)
{
  int w[8];
  make_tcfg(w, rate, longtrain);
  g->o = V27TX_create(NULL, w);
  g->rate = rate;
  g->frm = rate ? 32 : 24;
}
/* one frame of transmitter samples (160); data symbols random (full constellation) or constant */
static int gen_frame(Gen *g, short *dst, int sym_mode)
{
  unsigned short n = (unsigned short)g->frm;
  int i;
  for (i = 0; i < g->frm; i++) g_tx[i] = (short)(sym_mode == 0 ? (int)(rnd() & (g->rate ? 7u : 3u)) : sym_mode == 1 ? 0 : (i & (g->rate ? 7 : 3)));
  V27TX_modem(g->o, g_tx, g_txo, &n);
  memcpy(dst, g_txo, (size_t)n * 2);
  return n;
}
static void gen_free(Gen *g)
{
  V27TX_delete(g->o);
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
  for (i = 0; i < 7; i++) w[i] = V27RX_CFG[i];
  w[1] = (w[1] & (int)0xffff0000u) | (rate == 0 ? 0x960 : rate == 1 ? 0x12c0 : rate);
  if (variant & 1) w[5] = 1 + (int)rmod(5);              /* hdx.s0a = 0 */
  if (variant & 2) w[6] = (int)rnd();                    /* user word */
  if (variant & 4) w[2] = (int)rnd();                    /* overwritten by the control */
  if (variant & 8) w[0] = (int)rnd(), w[3] = (int)rnd(), w[4] = (int)rnd();
}
/* a created engine: the hdx.s14 flag is garbage in the blob, defined here on both sides */
static V27Rx *rx_make(V27Rx *sup, const int *cfg, int s14)
{
  V27Rx *o = V27RX_create(sup, cfg);
  if (o && o->hdx) o->hdx->s14 = (short)s14;
  return o;
}
static unsigned long long g_calls;
/* one receiver call on a copy of src[0..n): everything the call can change is hashed.  Returns the engine state dword; *outn = symbols returned */
static int rx_call(V27Rx *o, const short *src, int n, int *outn, int dump)
{
  unsigned short nn = (unsigned short)n;
  int ret, k;
  memcpy(g_in, src, (size_t)n * 2);
  for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
  g_in[n] = 0x5151; /* a sentinel behind the block */
  ret = V27RX_modem(o, g_in, g_out, &nn);
  emit(0, "rx call n=%d ret=%08x outn=%u in=%016llx out=%016llx sent=%d", n, u(ret), nn, fnv_n(g_in, n), fnv_n(g_out, 1024), g_in[n]);
  if (dump) d_rx("rx", o);
  *outn = nn;
  g_calls++;
  return ret;
}
static void observe(const V27Rx *o, int *prev)
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
  if (o->bundle->fse.cfg.cb == V27RX_eq_train) cov_hit(CV_S_CALLBACK_EQ);
  if (o->bundle->fse.cfg.cb == V27RX_decision) cov_hit(CV_S_CALLBACK_DEC);
  if (o->hdx->toneSearch) cov_hit(CV_S_S22);
  *prev = s;
}

/* ============================================================================================ captured transmitter streams */
#define CAPFR 220
static short g_cap[2][CAPFR * 160]; /* transmitter output, both rates (the transmitter of this side: blob or port) */
static int g_capn[2];
static short g_mrf[2][CAPFR * 160]; /* the same, through the receive matched filter (what the SRE gets) */
static int g_mrfn[2];
static void capture(int need_mrf)
{
  int r;
  for (r = 0; r < 2; r++) {
    Gen g;
    int i, n = 0, m = 0;
    FpmMrf mrf;
    int cfg[4];
    gen_new(&g, r, 1);
    if (need_mrf) {
      memset(&mrf, 0, sizeof mrf);
      cfg[0] = (int)((unsigned)V27RX_MRF_UP[r] | ((unsigned)V27RX_MRF_DOWN[r] << 16));
      cfg[1] = cfgp_word(V27RX_MRF_FILT[r]);
      cfg[2] = (int)(V27RX_MRF_FILT_LEN[r]); /* the blob's FPM_MRF_CFG[2] = 0x10e: its high half is 0 */
      cfg[3] = 0;
      FPM_MRF_init(&mrf, cfg, 1);
    }
    for (i = 0; i < CAPFR; i++) {
      short fr[512];
      int k = gen_frame(&g, fr, i > 150 && i < 170 ? 1 : 0);
      memcpy(g_cap[r] + n, fr, (size_t)k * 2);
      n += k;
      if (need_mrf) {
        static short tmp[512];
        unsigned int got = FPM_MRF_filter(&mrf, fr, tmp, (short)k) & 0xffffu;
        memcpy(g_mrf[r] + m, tmp, (size_t)got * 2);
        m += (int)got;
      }
    }
    if (need_mrf) FPM_MRF_free(&mrf);
    gen_free(&g);
    g_capn[r] = n;
    g_mrfn[r] = m;
    emit(0, "capture rate %d samples %d mrf %d hash %016llx %016llx", r, n, m, fnv_n(g_cap[r], n), fnv_n(g_mrf[r], m));
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
  emit(0, "TAB mrf up %u %u down %u %u len %u %u", V27RX_MRF_UP[0], V27RX_MRF_UP[1], V27RX_MRF_DOWN[0], V27RX_MRF_DOWN[1], V27RX_MRF_FILT_LEN[0], V27RX_MRF_FILT_LEN[1]);
  emit(0, "TAB sps %u %u srelen %u %u fselen %u %u mu train %u %u track %u %u", V27RX_SAMP_PER_BAUD[0], V27RX_SAMP_PER_BAUD[1], V27RX_SRE_FILT_LEN[0], V27RX_SRE_FILT_LEN[1], V27RX_FSE_FILT_LEN[0],
       V27RX_FSE_FILT_LEN[1], V27RX_FSE_MU_TRAIN[0], V27RX_FSE_MU_TRAIN[1], V27RX_FSE_MU_TRACK[0], V27RX_FSE_MU_TRACK[1]);
  emit(0, "TAB crr len %u %u adj %u %u mask %u %u", V27RX_CRR_TABLE_LEN[0], V27RX_CRR_TABLE_LEN[1], V27RX_CRR_ADJUST[0], V27RX_CRR_ADJUST[1], V27RX_DEC_PHS_MASK[0], V27RX_DEC_PHS_MASK[1]);
  emit(0, "TAB mrf filt %016llx %016llx", fnv_shorts(V27RX_MRF_FILT_2400, 270), fnv_shorts(V27RX_MRF_FILT_4800, 36));
  emit(0, "TAB sre filt %016llx %016llx", fnv_shorts(V27RX_SRE_FILT_2400, 271), fnv_shorts(V27RX_SRE_FILT_4800, 201));
  emit(0, "TAB xb %016llx %016llx clock x %016llx %016llx y %016llx %016llx", fnv_shorts(V27RX_XB_COFFS_2400, 11), fnv_shorts(V27RX_XB_COFFS_4800, 11), fnv_shorts(V27RX_XCLOCK_2400, 6),
       fnv_shorts(V27RX_XCLOCK_4800, 5), fnv_shorts(V27RX_YCLOCK_2400, 6), fnv_shorts(V27RX_YCLOCK_4800, 5));
  emit(0, "TAB sre pll1 %016llx %016llx pll2 %016llx %016llx", fnv_shorts(V27RX_SRE_PLLK1_2400, 3), fnv_shorts(V27RX_SRE_PLLK1_4800, 3), fnv_shorts(V27RX_SRE_PLLK2_2400, 3),
       fnv_shorts(V27RX_SRE_PLLK2_4800, 3));
  emit(0, "TAB fse ifilt %016llx %016llx qfilt %016llx %016llx", fnv_shorts(V27RX_FSE_IFILT_2400, 97), fnv_shorts(V27RX_FSE_IFILT_4800, 81), fnv_shorts(V27RX_FSE_QFILT_2400, 97),
       fnv_shorts(V27RX_FSE_QFILT_4800, 81));
  emit(0, "TAB fse pll1 %016llx %016llx pll2 %016llx %016llx", fnv_shorts(V27RX_FSE_PLLK1_2400, 3), fnv_shorts(V27RX_FSE_PLLK1_4800, 3), fnv_shorts(V27RX_FSE_PLLK2_2400, 3),
       fnv_shorts(V27RX_FSE_PLLK2_4800, 3));
  emit(0, "TAB crr %016llx %016llx", fnv_shorts(V27RX_CRR_TABLE_2400, 4), fnv_shorts(V27RX_CRR_TABLE_4800, 40));
  emit(0, "TAB dec pmap %016llx %016llx last %016llx %016llx", fnv_shorts((const short *)V27RX_DEC_PMAP_2400, 4), fnv_shorts((const short *)V27RX_DEC_PMAP_4800, 8),
       fnv_shorts(V27RX_DEC_LAST_PHASE_2400, 4), fnv_shorts(V27RX_DEC_LAST_PHASE_4800, 8));
  emit(0, "TAB mtd %016llx %016llx", fnv_shorts(V27_MTD_COEFF_2400, 10), fnv_shorts(V27_MTD_COEFF_4800, 10));
  emit(0, "TAB agc cfg %08x %08x %08x alpha %d,%d beta %d,%d w5 %08x", u(AGCv27_CFG[0]), u(AGCv27_CFG[1]), u(AGCv27_CFG[2]), ((const short *)cp(AGCv27_CFG[3]))[0],
       ((const short *)cp(AGCv27_CFG[3]))[1], ((const short *)cp(AGCv27_CFG[4]))[0], ((const short *)cp(AGCv27_CFG[4]))[1], u(AGCv27_CFG[5]));
  for (i = 0; i < 2; i++) {
    char a[80], b[80], c[80], d[80], e[80], f[80], g[80], h[80];
    emit(0, "TAB ptr %d crr=%s last=%s pmap=%s ifilt=%s qfilt=%s pll1=%s pll2=%s mrf=%s", i, dptr(V27RX_CRR_TABLE[i], a, 80), dptr(V27RX_DEC_LAST_PHASE[i], b, 80), dptr(V27RX_DEC_PMAP[i], c, 80),
         dptr(V27RX_FSE_IFILT[i], d, 80), dptr(V27RX_FSE_QFILT[i], e, 80), dptr(V27RX_FSE_PLLK1[i], f, 80), dptr(V27RX_FSE_PLLK2[i], g, 80), dptr(V27RX_MRF_FILT[i], h, 80));
    emit(0, "TAB ptr %d sre=%s xb=%s xc=%s yc=%s p1=%s p2=%s", i, dptr(V27RX_SRE_FILT[i], a, 80), dptr(V27RX_XB_COFFS[i], b, 80), dptr(V27RX_XCLOCK[i], c, 80), dptr(V27RX_YCLOCK[i], d, 80),
         dptr(V27RX_SRE_PLLK1[i], e, 80), dptr(V27RX_SRE_PLLK2[i], f, 80));
  }
  emit(0, "TAB CFG %08x %08x %08x %08x %08x %08x %08x", u(V27RX_CFG[0]), u(V27RX_CFG[1]), u(V27RX_CFG[2]), u(V27RX_CFG[3]), u(V27RX_CFG[4]), u(V27RX_CFG[5]), u(V27RX_CFG[6]));
  emit(0, "TAB CTL %08x %08x %08x %08x %08x", u(V27RX_CTL[0]), u(V27RX_CTL[1]), u(V27RX_CTL[2]), u(V27RX_CTL[3]), u(V27RX_CTL[4]));
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
    FaxVmiV27 m;
    memset(&m, 0, sizeof m);
    for (i = -3; i < 13; i++) {
      const char *s = "unset";
      v27rx_message(&m, i, &s);
      if (i == 8) {
        /* the blob reads the first word of V27RX_CTL (0x12c00000), the port stores NULL: one common line, the blob's value is checked */
        if (SIDE_BLOB && (unsigned)(uintptr_t)s != u(V27RX_CTL[0])) die3("blob v27rx_message(8) != V27RX_CTL[0]");
        if (!SIDE_BLOB && s != NULL) die3("port v27rx_message(8) != NULL");
        emit(1, "MSG %d = (CTL word 0 in the blob, NULL in the port)", i);
        cov_hit(CV_MSG_OOB);
      } else {
        emit(0, "MSG %d = %s", i, s ? s : "NULL");
        if (i >= 0 && i <= 7) cov_hit(CV_MSG); else cov_hit(CV_MSG_NEG);
      }
    }
    for (i = 0; i < 9; i++) {
      const char *s = 0;
      v27rx_message(&m, -256 + i, &s); /* a negative index whose low byte is a valid index: NULL (unsigned compare) */
      emit(0, "MSG %d = %s", -256 + i, s ? s : "NULL");
      s = 0;
      v27rx_message(&m, 0x100 + i, &s);
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
    int w[7];
    g_fail_malloc = failk;
    make_rcfg(w, rate, 0);
    V27RX_create(NULL, w);
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
    V27Rx *o;
    if (freopen("/dev/null", "w", stderr) == NULL) _exit(9); /* a garbage free() is a harness violation (exit 3) in the child: no noise */
    o = (V27Rx *)sysdep_malloc(sizeof(V27Rx));
    o->hdx = NULL;
    o->bundle = NULL;
    if (mode == 1) { /* hdx present (a real one), bundle missing */
      V27Rx *t = V27RX_create(NULL, NULL);
      o->hdx = t->hdx;
    }
    V27RX_create(o, NULL);
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
    int w[7];
    V27Rx *o;
    make_rcfg(w, from, 0);
    o = rx_make(NULL, w, 0);
    make_rcfg(w, to, 0);
    dbg_level((unsigned)dbg);
    g_verbose = 1; /* the child's own lines (allocator calls, debug text, the engine dump) are part of the output */
    V27RX_create(o, w);
    dbg_level(0);
    d_rx("changed", o);
    g_verbose = 0;
    fflush(stdout);
    V27RX_delete(o);
    _exit(redzones_dirty() ? 11 : 12);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE rate change %d -> %d (debug %d): signal %d", from, to, dbg, WTERMSIG(st));
  else emit(1, "PROBE rate change %d -> %d (debug %d): %s", from, to, dbg, WEXITSTATUS(st) == 11 ? "wrote outside a block" : WEXITSTATUS(st) == 12 ? "clean" : "other exit");
  cov_hit(CV_L_RATECHG);
}
static void run_some(V27Rx *o)
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
    int w[7];
    V27Rx *o = NULL;
    make_rcfg(w, rate, trial % 3 == 2 ? variant : trial % 3);
    dbg_level(dbg ? 2 : 0);
    if (rate == 0) cov_hit(CV_L_R2400); else if (rate == 1) cov_hit(CV_L_R4800); else cov_hit(CV_L_RBAD);
    if (dbg) cov_hit(CV_L_DBG);
    if (w[5]) cov_hit(CV_L_S0A0); else cov_hit(CV_L_S0A1);
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
        int w0[7];
        V27Rx *s;
        make_rcfg(w0, rate == 1 ? 1 : 0, 0);
        s = rx_make(NULL, w0, 0);
        d_rx("first", s);
        run_some(s);
        stack_poison();
        make_rcfg(w0, rate == 1 ? 1 : 0, (int)(rnd() & 15u));
        o = V27RX_create(s, w0);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_SUP_LIVE);
        cov_hit(CV_L_SAMERATE);
        break;
      }
      case 3: { /* hdx dropped: the create allocates a new one (the old pieces are freed here) */
        int w0[7];
        V27Rx *s;
        make_rcfg(w0, rate == 1 ? 1 : 0, 0);
        s = rx_make(NULL, w0, 0);
        d_rx("first", s);
        FPM_MTD_delete(s->hdx->mtd);
        FPM_MTD_delete(s->hdx->mtd2);
        sysdep_free(s->hdx->buf);
        sysdep_free(s->hdx);
        s->hdx = NULL;
        stack_poison();
        o = rx_make(s, w0, 0);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_SUP_NOHDX);
        break;
      }
      case 4: { /* create(o, o): the object is its own config (control path) */
        V27Rx *s;
        make_rcfg(w, rate == 1 ? 1 : 0, variant & ~1);
        s = rx_make(NULL, w, 0);
        d_rx("first", s);
        s->cfg.w[2] = 777;
        o = V27RX_create(s, s->cfg.w);
        cov_hit(CV_L_RECREATE);
        break;
      }
      default: { /* a re-create that keeps the rate but changes the user word / s0a */
        int w0[7], w1[7];
        V27Rx *s;
        make_rcfg(w0, 1, 0);
        s = rx_make(NULL, w0, 0);
        d_rx("first", s);
        make_rcfg(w1, 1, 3);
        o = V27RX_create(s, w1);
        cov_hit(CV_L_SUP_LIVE);
        break;
      }
    }
    dbg_level(0);
    d_rx("created", o);
    if (trial & 1) run_some(o);
    V27RX_delete(o);
    cov_hit(CV_L_DEL);
    trial_end("life");
  }
  probe_supplied(0); /* (a supplied object with a live hdx but no bundle dies on garbage pointers in a layout dependent way: not probed) */
  for (k = 0; k < 28; k++) {
    probe_create(k, k & 1);
    cov_hit(CV_L_PROBE);
  }
  probe_rate_change(0, 1, 0);
  probe_rate_change(1, 0, 0);
  probe_rate_change(0, 0, 0);
  probe_rate_change(1, 1, 0);
  probe_rate_change(0, 1, 2);
  probe_rate_change(1, 0, 2);
  cov_hit(CV_L_GROW);
  cov_hit(CV_L_SHRINK);
}

/* ============================================================================================ scenario 2: VMI wrappers */
/* kind 0: right after the initialiser (u4 = the integer 1, u10 = the config block); kind 1: after v27rx_create (u4 = the bit buffer, u10 = the small flag short: the blob stores a short over the low half of the
 * block pointer, the block is leaked) */
static void d_vmi(const char *t, const FaxVmiV27 *m, int kind)
{
  char p1[80], p2[80];
  emit(0, "%s VMI w0=%08x u4=%s s08=%d s0a=%d s0c=%d s0e=%d u10=%s eng=%s", t, u(m->w0), kind == 0 ? (m->u4.init1 == 1 ? "1" : "other") : (hblk(p1, 80, m->u4.bits), p1), m->s08, m->s0a, m->s0c, m->s0e,
       kind == 0 ? (hblk(p2, 80, m->u10.cfg), p2) : (m->u10.s10 == 2 ? "2" : m->u10.s10 == 3 ? "3" : "other"), m->eng ? "eng" : "NULL");
}
static void scen2(void)
{
  int trial, k;
  register_tables();
  capture(0);
  for (trial = 0; trial < 60; trial++) {
    FaxVmiV27 m;
    int rate = (trial & 1) ? 0x12c0 : 0x960;
    int dbg = trial % 4;
    int ret;
    void *cfgblk;
    short *bits = (short *)sysdep_malloc(1024 * 2);
    unsigned char *raw = (unsigned char *)&m;
    for (k = 0; k < (int)sizeof m; k++) raw[k] = (unsigned char)(0xa0u + (unsigned)k * 7u + (unsigned)trial);
    dbg_level((unsigned)dbg);
    if (dbg >= 3) cov_hit(CV_V_INIT_DBG);
    stack_poison();
    ret = fax_init_vmi_v27rx(&m, (unsigned short)rate, 0x55, (void *)0);
    dbg_level(0);
    emit(0, "vmi init ret=%d", ret);
    d_vmi("init", &m, 0);
    emit(0, "vmi init cfg %08x %08x %08x %08x %08x %08x %08x", u(((V27RxCfg *)m.u10.cfg)->w[0]), u(((V27RxCfg *)m.u10.cfg)->w[1]), u(((V27RxCfg *)m.u10.cfg)->w[2]), u(((V27RxCfg *)m.u10.cfg)->w[3]),
         u(((V27RxCfg *)m.u10.cfg)->w[4]), u(((V27RxCfg *)m.u10.cfg)->w[5]), u(((V27RxCfg *)m.u10.cfg)->w[6]));
    cfgblk = m.u10.cfg;
    cov_hit(CV_V_INIT);
    if (rate == 0x960) cov_hit(CV_V_R2400); else cov_hit(CV_V_R4800);
    { /* the VMI object keeps the user word as the engine handle: re-init with a live engine below */
      const int *cfg = trial % 3 == 1 ? NULL : (const int *)m.u10.cfg;
      void *e;
      V27Rx *sup = NULL;
      int w[7];
      if (trial % 3 == 2) {
        make_rcfg(w, trial & 1, 0);
        sup = rx_make(NULL, w, 0);
        m.eng = sup;
        cov_hit(CV_V_CREATE_SUP);
      } else {
        m.eng = NULL;
      }
      m.u4.bits = bits;
      m.s0c = (short)rnd(); /* v27rx_create sets these: garbage in front */
      m.s0e = (short)rnd();
      stack_poison();
      e = v27rx_create(&m, cfg);
      emit(0, "vmi create ret=%s", e == m.eng ? "eng" : "other");
      if (cfg == NULL) cov_hit(CV_V_CREATE_NULLCFG);
      cov_hit(CV_V_CREATE);
      d_vmi("create", &m, 1);
      ((V27Rx *)m.eng)->hdx->s14 = 0;
      d_rx("vmi", (V27Rx *)m.eng);
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
        r = v27rx_process(&m, ibuf, &outn, &n);
        emit(0, "vmi process n=%u outn=%u ret=%08x in=%016llx bits=%016llx", n, outn, u(r), fnv_n(ibuf, 160), fnv_n(bits, 1024));
        cov_hit(CV_V_PROC);
      }
    }
    {
      FaxV27Ctl ctl;
      int sx[16];
      memset(&ctl, 0, sizeof ctl);
      ctl.w4 = (int)rnd();
      ctl.b0c = (unsigned char)(trial & 0x28);
      ctl.b0d = (unsigned char)((trial & 4 ? 0x10 : 0) | (trial & 8 ? 2 : 0));
      emit(0, "vmi status %d", v27rx_status(&m, sx));
      emit(0, "vmi status null %d", v27rx_status(&m, NULL));
      emit(0, "vmi control %d", v27rx_control(&m, &ctl));
      emit(0, "vmi control null %d", v27rx_control(&m, NULL));
      cov_hit(CV_V_STATUS);
      cov_hit(CV_V_CTL);
      d_vmi("ctl", &m, 1);
      ((V27Rx *)m.eng)->hdx->s14 = 0;
      d_rx("vmi ctl", (V27Rx *)m.eng);
    }
    for (k = -2; k < 11; k++) {
      const char *s = "unset";
      v27rx_message(&m, k, &s);
      if (k == 8) emit(1, "vmi msg %d = (CTL word in the blob, NULL in the port)", k);
      else emit(0, "vmi msg %d = %s", k, s ? s : "NULL");
      cov_hit(CV_V_MSG);
    }
    v27rx_delete(&m);
    cov_hit(CV_V_DEL);
    sysdep_free(cfgblk);
    sysdep_free(bits);
    trial_end("vmi");
  }
}

/* ============================================================================================ scenario 3: receive streams */
static short g_sbuf[16384];
static void rx_control_random(V27Rx *o, int i, int pathb)
{
  FaxV27Ctl ctl;
  memset(&ctl, 0, sizeof ctl);
  ctl.w4 = (int)rnd();
  ctl.b0c = (unsigned char)((rpct(30) ? 8 : 0) | (rpct(30) ? 0x20 : 0));
  ctl.b0d = (unsigned char)((rpct(i > 120 ? 40 : 10) ? 0x10 : 0) | (rpct(5) ? 2 : 0));
  cov_hit(CV_S_CTL);
  if (ctl.b0d & 0x10) cov_hit(CV_S_HOLD);
  if (ctl.b0d & 2) cov_hit(CV_S_RECREATE);
  emit(0, "rx control ret=%d b0c=%02x b0d=%02x", V27RX_control(o, &ctl), ctl.b0c, ctl.b0d);
  if (o->hdx) o->hdx->s14 = (short)pathb; /* the flag survives a re-create of a live hdx; re-applied for clarity */
  d_rx("ctl", o);
}
static void scen3(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 72; trial++) {
    int rxr = (trial % 13 == 6) ? 0x1111 : (trial & 1); /* receiver rate word: 0 / 1 / invalid */
    int txr = rxr == 0 ? 0 : 1;
    int train = (trial / 2) & 1;
    int variant = (trial / 4) & 3;
    int prof = (trial / 8) % 9;
    int pol = (trial / 3) & 3;
    int pathb = (trial % 5 == 3) ? 1 : 0;
    int dbg = trial % 9 == 4;
    int w[7], i, calls, prev = -1, frm;
    Imp imp = {256, 0, 0, 0};
    Gen g;
    V27Rx *o;
    int lossleft = 0, burstleft = 0, gainleft = 0, sn = 0;
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
    if (rxr == 0) cov_hit(CV_S_R2400); else if (rxr == 1) cov_hit(CV_S_R4800); else cov_hit(CV_S_BADRATE);
    if (train) cov_hit(CV_S_LONG); else cov_hit(CV_S_SHORT);
    if (variant & 2) cov_hit(CV_S_USER);
    if (pathb) cov_hit(CV_S_PATHB);
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_S_DBG);
    stack_poison();
    gen_new(&g, txr, train);
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
      if (rpct(1) && i > 20 && trial >= 4) { /* the transmitter re-trains */
        gen_free(&g);
        gen_new(&g, txr, train);
        cov_hit(CV_S_RETRAIN);
      }
      n = impair(fr, n, &imp, &clip);
      (void)clip;
      if (sn + n > 16000) sn = 0;
      memcpy(g_sbuf + sn, fr, (size_t)n * 2);
      sn += n;
      while (sn > 0) {
        take = pol == 0 ? 120 : pol == 1 ? 1 + (int)rmod(120) : pol == 2 ? (rpct(50) ? 80 : 1 + (int)rmod(40)) : (rpct(5) ? 0 : rpct(8) ? 1 : 1 + (int)rmod(120)); /* (blocks above ~150 can overrun the SRE output buffer of the blob when the loop slips) */
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
        FaxV27Status stx[2];
        memset(stx, 0x55, sizeof stx);
        emit(0, "rx status ret=%d null=%d", V27RX_status(o, &stx[0]), V27RX_status(o, NULL));
        cov_hit(CV_S_STATUS);
      }
      if (rpct(3)) rx_control_random(o, i, pathb);
    }
    emit(0, "rx trial %d rxrate %d train %d variant %d prof %d pol %d: symbols=%d outhash=%016llx", trial, rxr, train, variant, prof, pol, tot, oh);
    if (trial % 8 == 0) emit(1, "RX trial %d rate %d: %d symbols", trial, rxr, tot);
    dbg_level(0);
    V27RX_delete(o);
    gen_free(&g);
    trial_end("rx run");
  }
}

/* ============================================================================================ scenario 4: handlers / detectors */
static void loop_init(V27RxLoop *l, int r)
{
  memset(l, 0, sizeof *l);
  l->rate4800 = r == 1;
  l->shortTrain = (int)(rnd() & 1u);
  l->mask = V27RX_DEC_PHS_MASK[r];
  l->pmap = V27RX_DEC_PMAP[r];
  l->last = V27RX_DEC_LAST_PHASE[r];
  l->avg = 0x3299;
}
typedef struct {
  Gen g;
  V27Rx *o;
  int rate, frames;
} Sess;
static void sess_new(Sess *s, int rate, int variant, int warm, int pathb, int longtrain)
{
  int w[7], i, outn, prev = -1;
  make_rcfg(w, rate, variant);
  gen_new(&s->g, rate == 0 ? 0 : 1, longtrain);
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
  V27RX_delete(s->o);
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
  } else {
    int r = s->rate == 0 ? 0 : 1, off = (int)rmod((unsigned)(g_capn[r] - 200));
    for (k = 0; k < want; k++) dst[k] = g_cap[r][off + k];
  }
  return want;
}
static void agc_reset(V27Rx *o)
{
  /* RxNextStateV27 state 1 advances the AGC table pointers by one entry (two entries per table): a second advance would read past the table (layout dependent), so direct calls start at entry 0 */
  o->bundle->agc.alpha = (const short *)cp(AGCv27_CFG[3]);
  o->bundle->agc.beta = (const short *)cp(AGCv27_CFG[4]);
}
static void poke_rx(V27Rx *o)
{
  static const int cnts[8] = {0x5da, 0x5db, 0x5dc, 0x5dd, 0x7ffe, 0x7fff, -1, 100};
  static const int e52s[8] = {0, 100, 0x1fff, 0x2000, 0x3fff, 0x4000, -5, 0x7fff};
  static const int q4as[8] = {0, 1, 0x30, 0x31, 0x32, 0x33, -1, 7};
  V27RxBundle *b = o->bundle;
  V27RxHdx *h = o->hdx;
  if (rpct(30)) h->hold = (int)(rnd() & 1u);
  if (rpct(25)) h->s14 = (short)(rnd() & 1u);
  if (rpct(25)) h->toneSearch = (short)(rnd() & 1u);
  if (rpct(30)) h->sinceTone = (short)(rpct(50) ? (int)rmod(0x500) : 0x4f0 + (int)rmod(24));
  if (rpct(30)) b->loop.cnt = (short)cnts[rnd() & 7u];
  if (rpct(20)) b->loop.phaseCnt = (short)((int)rmod(80) - 4);
  if (rpct(20)) b->loop.avg = (short)rmod(0x4000);
  if (rpct(20)) b->loop.idx = (short)(rnd() & b->loop.mask);
  if (rpct(30)) b->fse.decErrAvg = (short)e52s[rnd() & 7u];
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
  if (rpct(50)) o->st.w = (int)rnd();
}
static __attribute__((unused)) V27RxHdxFn pick_fn(int sel)
{
  switch (sel) {
    case 0: return RxHdxStartV27;
    case 1: return RxHdxEpochDetV27;
    case 2: return RxHdxPrtcolV27;
    case 3: return RxHdxDataV27;
    case 4: return RxHdxIdleV27;
    default: return RxHdxErrorV27;
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
static void fake_setup(V27RxLoop *l, int r, int c10)
{
  memset(&g_ffake, 0, sizeof g_ffake);
  loop_init(l, r);
  l->shortTrain = c10;
  g_ffake.cfg.hist = l;
  g_ffake.outI = g_fi;
  g_ffake.outQ = g_fq;
}
static void sweep_callbacks(void)
{
  static const int prevs[16] = {0, 1, 0x2000, 0x2001, 0x3fff, 0x4000, 0x4001, 0x5fff, 0x6000, 0x7fff, -1, -0x2000, -0x4000, -0x4001, -0x8000, 0x1234};
  static const int deltas[40] = {0x1ffe, 0x1fff, 0x2000, 0x2001, 0x2002, 0x3ffe, 0x3fff, 0x4000, 0x4001, 0x4002, 0x5ffe, 0x5fff, 0x6000, 0x6001, 0x6002, 0x7ffe, 0x7fff, 0x8000, 0x8001, 0x8002,
                                -0x1ffe, -0x1fff, -0x2000, -0x2001, -0x2002, -0x3ffe, -0x3fff, -0x4000, -0x4001, -0x4002, -0x5ffe, -0x5fff, -0x6000, -0x6001, -0x6002, -0x7ffe, -0x7fff, -0x8000, 0, 1};
  int r, c10, idx, a, pi, di, k;
  V27RxLoop l;
  /* the decision: every angle, every phase index, both rates; and the cnt wrap */
  for (r = 0; r < 2; r++) {
    int cnt;
    fake_setup(&l, r, 0);
    g_sh = 1469598103934665603ULL;
    for (cnt = 0; cnt < 2; cnt++)
      for (idx = 0; idx <= (r ? 7 : 3); idx++)
        for (a = -32768; a < 32768; a++) {
          short angle = (short)a, mag[3] = {1, 2, 3};
          int ret;
          l.idx = (short)idx;
          l.cnt = (short)(cnt ? 0x7fff : 5);
          ret = V27RX_decision(&g_ffake, &angle, mag);
          smix((unsigned)ret);
          smix((unsigned)(unsigned short)angle);
          smix((unsigned)(unsigned short)mag[0]);
          smix((unsigned)(unsigned short)l.idx);
          smix((unsigned)(unsigned short)l.cnt);
          if (a == 0 && idx == 0) cov_hit(CV_C_DEC);
        }
    emit(0, "sweep decision rate %d hash %016llx", r, g_sh);
  }
  /* the training callback: previous angle x angle (stride + the exact boundary distances) x flags */
  for (r = 0; r < 2; r++)
    for (c10 = 0; c10 < 2; c10++) {
      fake_setup(&l, r, c10);
      g_sh = 1469598103934665603ULL;
      for (pi = 0; pi < 16; pi++) {
        int n = 0;
        for (a = -32768; a < 32768; a += (a < -32760 || a > 32760) ? 1 : 7) {
          short angle = (short)a, mag[3] = {1, 2, 3};
          int ret;
          l.prev = (short)prevs[pi];
          l.idx = (short)(n++ & l.mask);
          l.cnt = (short)(n % 100 == 0 ? 0x7fff : n);
          l.phaseCnt = (short)((c10 ? 0x30 : 0x3e6) + (n & 3));
          g_ffake.muIdx = 7;
          g_ffake.eqForce = 1;
          g_ffake.cfg.cb = NULL;
          ret = V27RX_eq_train(&g_ffake, &angle, mag);
          smix((unsigned)ret);
          smix((unsigned)(unsigned short)angle);
          smix((unsigned)(unsigned short)mag[0]);
          smix((unsigned)(unsigned short)l.idx);
          smix((unsigned)(unsigned short)l.prev);
          smix((unsigned)(unsigned short)l.phaseCnt);
          smix((unsigned)(unsigned short)l.cnt);
          smix((unsigned)g_ffake.muIdx);
          smix((unsigned)g_ffake.eqForce);
          smix(g_ffake.cfg.cb == V27RX_decision);
        }
        for (di = 0; di < 40; di++) {
          short angle = (short)(prevs[pi] + deltas[di]), mag[3] = {1, 2, 3};
          int ret;
          l.prev = (short)prevs[pi];
          l.idx = (short)(n++ & l.mask);
          l.cnt = (short)n;
          l.phaseCnt = 0;
          g_ffake.muIdx = 7;
          g_ffake.eqForce = 1;
          g_ffake.cfg.cb = NULL;
          ret = V27RX_eq_train(&g_ffake, &angle, mag);
          smix((unsigned)ret);
          smix((unsigned)(unsigned short)angle);
          smix((unsigned)(unsigned short)l.idx);
          smix((unsigned)(unsigned short)l.prev);
        }
      }
      emit(0, "sweep eq_train rate %d c10 %d hash %016llx", r, c10, g_sh);
    }
  /* the epoch detector: random histories / energies around the thresholds */
  for (r = 0; r < 2; r++)
    for (c10 = 0; c10 < 2; c10++) {
      fake_setup(&l, r, c10);
      g_sh = 1469598103934665603ULL;
      for (k = 0; k < 120000; k++) {
        short angle = 0, mag[3] = {0, 0, 0};
        int ret, thr = c10 ? 0xa : 0x28;
        int big = rpct(30);
        g_fi[3] = (short)(big ? (int)rmod(40000) - 20000 : (int)rmod(600) - 300);
        g_fq[3] = (short)(big ? (int)rmod(40000) - 20000 : (int)rmod(600) - 300);
        g_ffake.idx = 3;
        l.symI0 = (short)(rpct(50) ? g_fi[3] + (int)rmod(50) - 25 : (int)rnd());
        l.symQ0 = (short)(rpct(50) ? g_fq[3] + (int)rmod(50) - 25 : (int)rnd());
        l.symI1 = (short)(rpct(50) ? l.symI0 + (int)rmod(50) - 25 : (int)rnd());
        l.symQ1 = (short)(rpct(50) ? l.symQ0 + (int)rmod(50) - 25 : (int)rnd());
        l.symI2 = (short)(rpct(50) ? l.symI1 + (int)rmod(5000) - 2500 : (int)rnd());
        l.symQ2 = (short)(rpct(50) ? l.symQ1 + (int)rmod(5000) - 2500 : (int)rnd());
        l.avg = (short)(rpct(60) ? (int)rmod(300) : (int)rmod(0x7000));
        l.phaseCnt = (short)(thr - 1 + (int)rmod(3) + (rpct(10) ? 1000 : 0) - (rpct(5) ? 30 : 0));
        l.cnt = (short)rnd();
        g_ffake.eqForce = (int)(rnd() & 1u);
        g_ffake.cfg.cb = NULL;
        ret = V27RX_epoch_det(&g_ffake, &angle, mag);
        smix((unsigned)ret);
        smix((unsigned)(unsigned short)l.symI0);
        smix((unsigned)(unsigned short)l.symQ0);
        smix((unsigned)(unsigned short)l.symI1);
        smix((unsigned)(unsigned short)l.symQ1);
        smix((unsigned)(unsigned short)l.symI2);
        smix((unsigned)(unsigned short)l.symQ2);
        smix((unsigned)(unsigned short)l.avg);
        smix((unsigned)(unsigned short)l.phaseCnt);
        smix((unsigned)(unsigned short)l.cnt);
        smix((unsigned)g_ffake.eqForce);
        smix(g_ffake.cfg.cb == V27RX_eq_train);
      }
      emit(0, "sweep epoch_det rate %d c10 %d hash %016llx", r, c10, g_sh);
    }
}

extern short FPM_rms(const short *data, short count);
/* DataCarrierDetectV27: the combinations of every input it reads (both paths, the thresholds 0x5db / 0x3fff / 0x4ff, the energy-drop boundary, the q54 counter) */
static void dcd_sweep(void)
{
  Sess s;
  static short blk[512];
  static const int cnts[4] = {0x5da, 0x5db, 0x5dc, 0};
  static const int e52s[5] = {-1, 0x3ffe, 0x3fff, 0x4000, 0x4001};
  static const int dets[4] = {0, 1, 2, 3};
  static const int w40s[4] = {0, 1, 2, -1};
  int path, ci, ei, di, wi, s22, q50, k, n = 40, kind, dd, ds, qq, q54;
  sess_new(&s, 1, 0, 70, 0, 1);
  agc_reset(s.o);
  for (kind = 0; kind < 3; kind++) {
    for (k = 0; k < n; k++) blk[k] = kind == 0 ? g_cap[1][500 + k] : kind == 1 ? (short)((int)(rnd() & 0x3fffu) - 0x2000) : (short)(g_cap[1][700 + k] / 7);
    for (path = 0; path < 2; path++)
      for (ci = 0; ci < 4; ci++)
        for (ei = 0; ei < 5; ei++)
          for (di = 0; di < 4; di++)
            for (wi = 0; wi < 4; wi++)
              for (s22 = 0; s22 < 2; s22++)
                for (q50 = 0; q50 < 2; q50++) {
                  V27Rx *o = s.o;
                  int ret;
                  o->hdx->s14 = (short)path;
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
                  ret = DataCarrierDetectV27(o, blk, (unsigned short)n);
                  dbg_level(0);
                  smix(0);
                  emit(0, "dcd k%d p%d c%d e%d d%d w%d s%d q%d: ret=%d s22=%d s20=%d q54=%d q52=%d", kind, path, ci, ei, di, wi, s22, q50, ret, o->hdx->toneSearch, o->hdx->sinceTone, o->bundle->refBlocks, o->bundle->refLevel);
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
              V27Rx *o = s.o;
              int ret;
              o->hdx->s14 = (short)path;
              o->hdx->toneSearch = 0;
              o->bundle->loop.cnt = 0;
              o->bundle->fse.decErrAvg = 0;
              o->bundle->agc.detect = 1;
              o->bundle->sre.pllEnable = 1;
              o->bundle->energyTest = 1;
              o->bundle->refLevel = (short)qq;
              o->bundle->refBlocks = (short)q54;
              ret = DataCarrierDetectV27(o, blk, (unsigned short)n);
              emit(0, "dcd energy k%d p%d r=%d q52=%d thr=%d q54=%d: ret=%d q52'=%d q54'=%d", kind, path, r, qq, thr, q54, ret, o->bundle->refLevel, o->bundle->refBlocks);
            }
          }
        }
    }
    /* the carrier-timeout counter s20 around 0x4ff (path B, s22 set) */
    for (dd = -6; dd <= 6; dd++)
      for (q50 = 0; q50 < 2; q50++) {
        V27Rx *o = s.o;
        int ret;
        o->hdx->s14 = 1;
        o->hdx->toneSearch = 1;
        o->hdx->sinceTone = (short)(dd <= -4 ? -n - 100 + dd : 0x4ff - n + dd);
        o->bundle->loop.cnt = 0;
        o->bundle->fse.decErrAvg = 0;
        o->bundle->agc.detect = 1;
        o->bundle->sre.pllEnable = 1;
        o->bundle->energyTest = (short)(q50 ? 1 : 0);
        o->bundle->refLevel = 0;
        dbg_level(q50 ? 2 : 0);
        ret = DataCarrierDetectV27(o, blk, (unsigned short)n);
        dbg_level(0);
        emit(0, "dcd s20 k%d dd=%d q50=%d: ret=%d s20=%d", kind, dd, q50, ret, o->hdx->sinceTone);
      }
  }
  sess_free(&s);
  trial_end("dcd sweep");
}
/* RxHdxIdleV27: the carrier flag and the decision error at the boundary 0x1fff / 0x2000 (a block too short to complete a symbol keeps the error unchanged) */
static void idle_sweep(void)
{
  Sess s;
  static short blk[512];
  int e, st, kind, k, n = 40;
  sess_new(&s, 0, 0, 70, 0, 1);
  for (kind = 0; kind < 2; kind++)
    for (e = 0x1ffd; e <= 0x2002; e++)
      for (st = 3; st <= 4; st++) {
        V27Rx *o = s.o;
        unsigned short nn = (unsigned short)n;
        int ret;
        for (k = 0; k < n; k++) blk[k] = kind == 0 ? g_cap[0][500 + k] : (short)(g_cap[0][500 + k] * 3);
        agc_reset(o);
        o->hdx->state = (short)st;
        o->hdx->fn = RxHdxIdleV27;
        o->bundle->fse.decErrAvg = (short)e;
        o->bundle->fse.untilSym = 30000;
        o->bundle->sre.pllEnable = 1;
        o->bundle->sreGate = 1;
        o->st.w = 0x7000 + st;
        dbg_level(e & 1 ? 2 : 0);
        ret = RxHdxIdleV27(o, blk, g_out, &nn);
        dbg_level(0);
        emit(0, "idle sweep e52=%x st=%d kind=%d: ret=%d nn=%u state=%d b=%08x e52'=%d", e, st, kind, ret, nn, o->hdx->state, u(o->st.w), o->bundle->fse.decErrAvg);
        d_rx("idle", o);
      }
  sess_free(&s);
  trial_end("idle sweep");
}


/* QualityDetectV27: every q4a stage x the decoder error at / around the threshold x the carrier flag */
static void quality_sweep(void)
{
  static const int q4as[8] = {0, 1, 0x30, 0x31, 0x32, 0x33, 0x34, -1};
  int qa, d, det, w40, r, e;
  V27Rx *o;
  int w[7];
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
        r = QualityDetectV27(o);
        emit(0, "quality round e52=%d q48=%d: r=%d q48'=%d q4a=%d", g_e52r[e], g_q48r[d], r, o->bundle->decErrSmooth, o->bundle->qualBlocks);
      }
  }
  for (qa = 0; qa < 8; qa++)
    for (d = -2; d <= 2; d++)
      for (det = 0; det < 3; det++)
        for (w40 = 0; w40 < 2; w40++)
          for (e = 0; e < 3; e++) {
            o->bundle->qualBlocks = (short)q4as[qa];
            o->bundle->errLimit = 0x1b9e;
            o->bundle->decErrSmooth = (short)(0x1b9e + d);
            o->bundle->qualityOk = 0;
            o->bundle->agc.detect = det;
            o->bundle->sre.pllEnable = w40;
            o->bundle->fse.decErrAvg = (short)(e == 0 ? 0x1b9e + d : e == 1 ? -77 : 5000);
            dbg_level(d == 0 ? 2 : 0);
            r = QualityDetectV27(o);
            dbg_level(0);
            emit(0, "quality qa=%d d=%d det=%d w40=%d e=%d: r=%d q48=%d q4a=%d q4e=%d", q4as[qa], d, det, w40, e, r, o->bundle->decErrSmooth, o->bundle->qualBlocks, o->bundle->qualityOk);
          }
  V27RX_delete(o);
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
    k = RxHdxDataV27(s.o, blk, g_out, &nn);
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
      V27Rx *o = s.o;
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
      if (o->hdx->s14 == 0) cov_hit(CV_H_PATHA); else cov_hit(CV_H_PATHB);
      if (o->bundle->loop.cnt > 0x5db) cov_hit(CV_H_CNT);
      if (o->bundle->fse.decErrAvg > 0x3fff) cov_hit(CV_H_E52HI);
      if (o->bundle->energyTest) cov_hit(CV_H_Q50);
      if (o->hdx->sinceTone > 0x4ff) cov_hit(CV_H_S20);
      d_rx("pre", o);
      stack_poison();
      switch (fnsel) {
        case 0: o->hdx->fn = RxHdxStartV27; ret = RxHdxStartV27(o, blk, g_out, &nn); cov_hit(CV_H_START); break;
        case 1: o->hdx->fn = RxHdxEpochDetV27; ret = RxHdxEpochDetV27(o, blk, g_out, &nn); cov_hit(CV_H_EPOCH); break;
        case 2: o->hdx->fn = RxHdxPrtcolV27; ret = RxHdxPrtcolV27(o, blk, g_out, &nn); cov_hit(CV_H_PRTCOL); break;
        case 3: o->hdx->fn = RxHdxDataV27; ret = RxHdxDataV27(o, blk, g_out, &nn); cov_hit(CV_H_DATA); break;
        case 4: o->hdx->fn = RxHdxIdleV27; ret = RxHdxIdleV27(o, blk, g_out, &nn); cov_hit(CV_H_IDLE); break;
        case 5: o->hdx->fn = RxHdxErrorV27; ret = RxHdxErrorV27(o, blk, g_out, &nn); cov_hit(CV_H_ERROR); break;
        case 6: ret = DemodDataV27(o, blk, g_out, nn); cov_hit(CV_H_DEMOD); break;
        case 7:
          ret = DataCarrierDetectV27(o, blk, nn);
          if (o->hdx->s14 == 0) cov_hit(CV_H_PATHA);
          if (ret) cov_hit(CV_H_CARR); else cov_hit(CV_H_NOCARR);
          break;
        case 8:
          ret = QualityDetectV27(o);
          if (o->bundle->qualBlocks == 0) cov_hit(CV_H_QUAL0);
          else if (o->bundle->qualBlocks < 0x32) cov_hit(CV_H_QUAL1);
          else if (o->bundle->qualBlocks == 0x32) cov_hit(CV_H_QUAL2);
          else cov_hit(CV_H_QUAL3);
          emit(0, "quality again %d", QualityDetectV27(o));
          break;
        case 9:
          RxNextStateV27(o);
          ret = o->hdx->state;
          break;
        case 10:
          ret = CarrierDetectV27(o);
          emit(0, "epoch %d snr %d", EpochDetectV27(o), GetSNRV27(o));
          cov_hit(CV_C_GETSNR);
          cov_hit(CV_C_EPOCHDET);
          cov_hit(CV_C_CARRDET);
          break;
        default: { /* the dispatcher on this state */
          unsigned short n2 = nn;
          ret = V27RX_modem(o, blk, g_out, &n2);
          nn = n2;
          break;
        }
      }
      if (o->bundle->sre.pllEnable == 0 && o->bundle->sreGate) cov_hit(CV_H_CDET_BRANCH);
      if ((fnsel == 3 || fnsel == 7) && q54pre == 1 && o->bundle->energyTest && o->bundle->refBlocks == 0) cov_hit(CV_H_Q54);
      if ((fnsel == 3 || fnsel == 7) && o->hdx->s14 != 0 && s20pre != 0 && o->hdx->sinceTone == 0) cov_hit(CV_H_MTD2);
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

static void cov_dec_pre(short angle, const V27RxLoop *l)
{
  int d = (int)angle - l->last[l->idx];
  if (d == 0) cov_hit(CV_C_DEC_D0);
  if (d < 0) {
    cov_hit(CV_C_DEC_NEG);
    d += 0x8000;
  }
  if (d > 0x8000) cov_hit(CV_C_DEC_BIG);
}
/* ============================================================================================ scenario 5: callbacks, next state, dispatcher, control, status */
static int g_fake_calls;
static const short *g_fk_in, *g_fk_out;
static int fake_handler(V27Rx *o, short *in, short *out, unsigned short *n)
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
      V27Rx *o = s.o;
      FaxFse *f = &o->bundle->fse;
      V27RxLoop *l = &o->bundle->loop;
      short angle, mag, mag2[3];
      int which = (int)(rnd() % 3u), ret, k;
      static const int cnts[6] = {0, 0x7ffe, 0x7fff, 0x8000 - 1, 5, 0x1234};
      static const int s1cs[8] = {-1, 0, 9, 10, 11, 0x27, 0x28, 0x3e7};
      f->idx = (short)rmod(20);
      for (k = 0; k < 20; k++) {
        f->outI[k] = (short)(rpct(50) ? (int)rmod(8000) - 4000 : (int)rmod(60000) - 30000);
        f->outQ[k] = (short)(rpct(50) ? (int)rmod(8000) - 4000 : (int)rmod(60000) - 30000);
      }
      l->symI0 = (short)rnd(); l->symQ0 = (short)rnd(); l->symI1 = (short)rnd(); l->symQ1 = (short)rnd(); l->symI2 = (short)rnd(); l->symQ2 = (short)rnd();
      l->cnt = (short)cnts[rnd() % 6u];
      l->phaseCnt = (short)s1cs[rnd() % 8u];
      l->avg = (short)(rpct(50) ? (int)rmod(300) : (int)rmod(0x7000));
      l->shortTrain = (int)(rnd() & 1u);
      l->idx = (short)(rnd() & l->mask);
      l->prev = (short)rnd();
      angle = (short)(rpct(30) ? (int)(rnd() & 0xffffu) : (int)(l->last[rnd() & l->mask] + (int)rmod(3000) - 1500));
      if (rpct(5)) angle = l->last[l->idx];
      mag = (short)rnd();
      mag2[0] = mag;
      mag2[1] = 0x1111;
      mag2[2] = 0x2222;
      f->muIdx = (short)(rnd() & 1u);
      f->eqForce = (int)(rnd() & 1u);
      d_fse("cbpre", f);
      emit(0, "cbpre loop h=%d,%d,%d,%d,%d,%d c10=%d idx=%d s1c=%d avg=%d prev=%d cnt=%d angle=%d", l->symI0, l->symQ0, l->symI1, l->symQ1, l->symI2, l->symQ2, l->shortTrain, l->idx, l->phaseCnt, l->avg, l->prev, l->cnt, angle);
      stack_poison();
      switch (which) {
        case 0: {
          int thr = l->shortTrain == 0 ? 0x28 : 0xa, hi = l->phaseCnt > thr;
          ret = V27RX_epoch_det(f, &angle, mag2);
          cov_hit(CV_C_EPOCH);
          if (l->shortTrain == 0) cov_hit(CV_C_EPOCH_THR0); else cov_hit(CV_C_EPOCH_THR1);
          if (f->cfg.cb == V27RX_eq_train) cov_hit(CV_C_EPOCH_SWITCH);
          else if (hi) cov_hit(CV_C_EPOCH_HI);
          else cov_hit(CV_C_EPOCH_LO);
          break;
        }
        case 1: {
          short dd = (short)((unsigned short)angle - (unsigned short)l->prev);
          int adj = dd;
          if (dd > 0x4000) { cov_hit(CV_C_TRAIN_ADJ1); adj = (short)(dd + 0x8000); }
          if (dd < -0x4000) { cov_hit(CV_C_TRAIN_ADJ2); adj = (short)(dd - 0x8000); }
          if ((adj < 0 ? -adj : adj) > 0x2000) cov_hit(CV_C_TRAIN_STEP); else cov_hit(CV_C_TRAIN_NOSTEP);
          ret = V27RX_eq_train(f, &angle, mag2);
          cov_hit(CV_C_TRAIN);
          if (l->shortTrain == 0) cov_hit(CV_C_TRAIN_FLAG0); else cov_hit(CV_C_TRAIN_FLAG1);
          if (f->cfg.cb == V27RX_decision) cov_hit(CV_C_TRAIN_END);
          if (l->cnt == 0x4000) cov_hit(CV_C_TRAIN_WRAP);
          break;
        }
        default:
          cov_dec_pre(angle, l);
          ret = V27RX_decision(f, &angle, mag2);
          cov_hit(CV_C_DEC);
          if (l->rate4800) cov_hit(CV_C_DEC_R4800); else cov_hit(CV_C_DEC_R2400);
          if (l->cnt == 0x4000) cov_hit(CV_C_DEC_WRAP);
          break;
      }
      emit(0, "callback %d ret=%d angle=%d mag=%d %d %d", which, ret, angle, mag2[0], mag2[1], mag2[2]);
      d_fse("cbpost", f);
      emit(0, "cbpost loop h=%d,%d,%d,%d,%d,%d idx=%d s1c=%d avg=%d prev=%d cnt=%d", l->symI0, l->symQ0, l->symI1, l->symQ1, l->symI2, l->symQ2, l->idx, l->phaseCnt, l->avg, l->prev, l->cnt);
    }
    sess_free(&s);
    trial_end("callbacks");
  }
  /* the decision on a dense sweep of angles (every quadrant, both rates) */
  for (trial = 0; trial < 2; trial++) {
    Sess s;
    int ang;
    sess_new(&s, trial, 0, 0, 0, 1);
    for (ang = -32768; ang < 32768; ang += 241) {
      FaxFse *f = &s.o->bundle->fse;
      V27RxLoop *l = &s.o->bundle->loop;
      short angle = (short)ang, mag[3] = {0, 0, 0};
      int ret;
      l->idx = (short)((ang >> 8) & l->mask);
      l->cnt = (short)(ang == 0 ? 0x7fff : 5);
      cov_dec_pre(angle, l);
      ret = V27RX_decision(f, &angle, mag);
      emit(0, "sweep %d rate %d ret=%d angle=%d mag=%d idx=%d cnt=%d", ang, trial, ret, angle, mag[0], l->idx, l->cnt);
    }
    for (ang = -3; ang <= 3; ang++) { /* small distances around 0 (the quirk of the initial minimum) */
      FaxFse *f = &s.o->bundle->fse;
      V27RxLoop *l = &s.o->bundle->loop;
      short angle = (short)(l->last[l->idx] + ang), mag[3] = {0, 0, 0};
      int ret;
      cov_dec_pre(angle, l);
      ret = V27RX_decision(f, &angle, mag);
      emit(0, "near %d rate %d ret=%d angle=%d idx=%d", ang, trial, ret, angle, l->idx);
    }
    sess_free(&s);
    trial_end("sweep");
  }
  /* (b) RxNextStateV27 for every state x rate x s0a x debug level, then one handler call */
  for (a = 0; a < 12 * 2 * 2 * 2; a++) {
    static const int sts[12] = {0, 1, 2, 3, 4, 5, 6, -1, 7, 0x7fff, 2, 4};
    int st = sts[a % 12], rate = (a / 12) & 1, s0a = (a / 24) & 1, dbg = (a / 48) & 1;
    int w[7];
    V27Rx *o;
    make_rcfg(w, rate, s0a ? 0 : 1);
    o = rx_make(NULL, w, 0);
    o->hdx->state = (short)st;
    o->hdx->count = (short)(int)(rnd() & 0x7fu);
    o->st.w = (int)rnd();
    dbg_level(dbg ? 2 : 0);
    d_rx("pre", o);
    stack_poison();
    RxNextStateV27(o);
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
    V27RX_delete(o);
    trial_end("next");
  }
  /* (c) the dispatcher with a harness handler */
  for (trial = 0; trial < 60; trial++) {
    int w[7], n, k;
    V27Rx *o;
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
    k = V27RX_modem(o, blk, g_out, &nn);
    emit(0, "fake modem n=%d ret=%08x nn=%u calls=%d out=%016llx", n, u(k), nn, g_fake_calls, fnv_n(g_out, 1024));
    cov_hit(CV_C_FAKE);
    if (trial % 6 == 2 || trial % 6 == 5) cov_hit(CV_C_FAKE_NEG);
    if (trial % 6 == 0 || trial % 6 == 4) cov_hit(CV_C_FAKE_PART);
    d_rx("fake", o);
    V27RX_delete(o);
    trial_end("fake");
  }
  /* (d) V27RX_control for every b0c x four b0d values, then a block */
  for (b0c = 0; b0c < 256; b0c++) {
    for (b0d = 0; b0d < 8; b0d++) {
      static const int d4[8] = {0, 0x10, 2, 0x12, 0x20, 0x30, 1, 0xff};
      FaxV27Ctl ctl;
      int w[7], k, ret;
      V27Rx *o;
      unsigned short nn = 60;
      static short blk[512];
      make_rcfg(w, (b0c + b0d) & 1, 0);
      o = rx_make(NULL, w, 0);
      memset(&ctl, 0, sizeof ctl);
      ctl.w4 = (int)rnd();
      ctl.b0c = (unsigned char)b0c;
      ctl.b0d = (unsigned char)d4[b0d];
      ret = V27RX_control(o, &ctl);
      emit(0, "control c=%02x d=%02x ret=%d", b0c, d4[b0d], ret);
      cov_hit(CV_C_CTL);
      if (d4[b0d] & 0x10) cov_hit(CV_C_CTL_HOLD);
      if (d4[b0d] & 2) cov_hit(CV_C_CTL_RECREATE);
      if (b0c & 8) cov_hit(CV_C_CTL_W0);
      if (b0c & 0x20) cov_hit(CV_C_CTL_W10);
      for (k = 0; k < 512; k++) blk[k] = (short)((int)(rnd() & 0x1fffu) - 0x1000);
      for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
      V27RX_modem(o, blk, g_out, &nn);
      emit(0, "control run nn=%u out=%016llx", nn, fnv_n(g_out, 1024));
      if (b0c % 17 == 0) d_rx("ctl", o);
      emit(0, "control hold=%d w0=%d w10=%d w2=%08x", o->hdx->hold, o->bundle->w0, o->bundle->eqGate, u(o->cfg.w[2]));
      V27RX_delete(o);
      trial_end("ctl");
    }
  }
  emit(0, "control null %d", V27RX_control((V27Rx *)NULL, NULL));
  cov_hit(CV_C_CTL_NULL);
  {
    int dummy[8];
    V27Rx *o = rx_make(NULL, NULL, 0);
    emit(0, "status %d %d", V27RX_status(o, dummy), V27RX_status(o, NULL));
    cov_hit(CV_C_STATUS);
    cov_hit(CV_C_STATUS_NULL);
    V27RX_delete(o);
    trial_end("status");
  }
}

/* ============================================================================================ scenarios 6 / 7: the FSE and SRE blocks stand-alone */
static int g_hcb_n;
static int hcb_a(FaxFse *f, short *angle, short *mag)
{
  int code = (int)((((unsigned)(unsigned short)*angle + 0x1000u) >> 13) & 7u);
  g_hcb_n++;
  *angle = (short)(code << 13);
  *mag = (short)(0x2000 + f->idx);
  return code | (f->idx << 4);
}
static int hcb_b(FaxFse *f, short *angle, short *mag)
{
  g_hcb_n++;
  if ((g_hcb_n & 7) == 0) {
    f->eqForce = !f->eqForce;
    f->muIdx = (short)((g_hcb_n >> 3) & 1);
  }
  if ((g_hcb_n & 31) == 5) {
    f->cfg.cb = hcb_a;
    cov_hit(CV_F_CB_SWITCH);
  }
  *angle = (short)(*angle - (*angle >> 3));
  *mag = (short)(*mag + 3);
  return (*angle >> 5) & 0xff;
}
#define WADD_H(a, b) ((int)((unsigned int)(a) + (unsigned int)(b)))
static int hcb_c(FaxFse *f, short *angle, short *mag)
{
  static const int deltas[16] = {0x3ffe, 0x3fff, 0x4000, 0x4001, 0x4002, -0x3ffe, -0x3fff, -0x4000, -0x4001, -0x4002, 0x7fff, -0x8000, 1, -1, 0x1234, -0x1234};
  g_hcb_n++;
  *angle = (short)(*angle - deltas[g_hcb_n & 15]);
  *mag = (short)(0x1000 + (g_hcb_n & 255));
  if ((g_hcb_n & 63) == 17) f->cfg.cb = hcb_a;
  return (g_hcb_n * 37) & 0xff;
}
static void fse_cfg_random(FaxFseCfg *c, int r, int variant, V27RxLoop *loop)
{
  int ntmax = r == 0 ? 97 : 81;
  *c = h_fse_def;
  c->sps = (short)(r == 0 ? 6 : 5);
  c->blockLen = (short)(r == 0 ? 0x90 : 0xa0);
  c->iCoefTab = cfgp_word(r == 0 ? V27RX_FSE_IFILT_2400 : V27RX_FSE_IFILT_4800);
  c->qCoefTab = cfgp_word(r == 0 ? V27RX_FSE_QFILT_2400 : V27RX_FSE_QFILT_4800);
  c->nt = (short)(variant & 1 ? (rpct(25) ? 1 + (int)rmod(3) : 4 + (int)rmod(44)) : ntmax);
  c->mu[0] = (short)V27RX_FSE_MU_TRAIN[r];
  c->mu[1] = (short)V27RX_FSE_MU_TRACK[r];
  c->carrierTab = cfgp_word(r == 0 ? V27RX_CRR_TABLE_2400 : V27RX_CRR_TABLE_4800);
  c->crrLen = (short)V27RX_CRR_TABLE_LEN[r];
  c->crrAdj = (short)(variant & 2 ? 1 + (int)rmod(3) : V27RX_CRR_ADJUST[r]);
  c->settleCount = (short)(variant & 4 ? 1 + (int)rmod(40) : 1000);
  c->errThreshHi = (short)(variant & 8 ? (int)rmod(0x3000) : 0x2666);
  c->errThreshLo = (short)(variant & 8 ? (int)rmod(0x1000) : 0x8f6);
  c->propTab = cfgp_word(r == 0 ? V27RX_FSE_PLLK1_2400 : V27RX_FSE_PLLK1_4800);
  c->intTab = cfgp_word(r == 0 ? V27RX_FSE_PLLK2_2400 : V27RX_FSE_PLLK2_4800);
  c->hist = loop;
  c->cb = (variant & 16) ? V27RX_epoch_det : (variant & 32) ? hcb_b : (variant & 256) ? hcb_c : hcb_a;
  c->w34 = (int)rnd();
  if (variant & 64) c->mu[0] = (short)rmod(6000), c->mu[1] = (short)rmod(6000);
  if (variant & 128) c->blockLen = (short)(c->sps * (4 + (int)rmod(20)));
}


/* the blob's static Decoder-Error counter (per sample, prints at > 0x1c1f when debug level > 1, then resets): exact totals around the threshold, at debug levels 2 and 1 */
static void fse_feed(FaxFse *f, int total, int *pos, int r)
{
  static short in[512], out[512];
  while (total > 0) {
    int n = total > 144 ? 144 : total, k;
    for (k = 0; k < n; k++) in[k] = g_mrf[r][(*pos + k) % g_mrfn[r]];
    *pos = (*pos + n) % g_mrfn[r];
    for (k = 0; k < 512; k++) out[k] = 0x7777;
    FAXFSE_receive(f, in, out, (unsigned short)n);
    total -= n;
  }
}
static void fse_counter_probe(void)
{
  FaxFse *f;
  V27RxLoop loop;
  FaxFseCfg cfg;
  int lv, pos = 100;
  for (lv = 2; lv >= 0; lv--) {
    loop_init(&loop, 0);
    fse_cfg_random(&cfg, 0, 0, &loop);
    cfg.cb = hcb_a;
    g_hcb_n = 0;
    f = (FaxFse *)sysdep_malloc(sizeof(FaxFse));
    FAXFSE_init(f, &cfg, 1);
    memset(f->diag0, 0, sizeof f->diag0);
    dbg_level((unsigned)lv);
    fse_feed(f, 0x1c1f, &pos, 0);
    emit(0, "counter probe level %d at 0x1c1f", lv);
    fse_feed(f, 1, &pos, 0);
    emit(0, "counter probe level %d at 0x1c20", lv);
    fse_feed(f, 0x1c1f, &pos, 0);
    emit(0, "counter probe level %d again at 0x1c1f", lv);
    fse_feed(f, 1, &pos, 0);
    emit(0, "counter probe level %d again at 0x1c20", lv);
    fse_feed(f, 3 * 7200 + 5, &pos, 0);
    dbg_level(0);
    d_fse("counter", f);
    FAXFSE_free(f);
    sysdep_free(f);
    trial_end("fse counter");
  }
}

/* ---- corner mode: zero equalizer taps and an all-zero history make the equalizer output exactly (0, 0), so the angle FPM_atan(0, 0), the carrier table value, the predicted phase offset s76, the callback's
 * angle, the error and the PLL sums are known BEFORE the call: the harness steers every value to the exact boundaries of the angle normalisations, the abs shifts and the PLL wrap ---- */
static short g_corner_angle;
static int hcb_d(FaxFse *f, short *angle, short *mag)
{
  (void)f;
  g_hcb_n++;
  *angle = g_corner_angle;
  mag[0] = (short)(g_hcb_n & 0x3f);
  return 5 + (g_hcb_n & 3);
}
static void fse_corner(void)
{
  static const int L1[16] = {-0x4001, -0x4000, -0x3fff, -3, -2, -1, 0, 1, 2, 3, 0x3ffd, 0x3ffe, 0x3fff, 0x4000, 0x4001, 0x7ffe};
  static const int LD[14] = {0x3ffe, 0x3fff, 0x4000, 0x4001, 0x4002, -0x3ffe, -0x3fff, -0x4000, -0x4001, -0x4002, 0x7fff, -0x7fff, 1, 0};
  static const int L3[16] = {8190, 8191, 8192, 8193, 16382, 16383, 16384, 16385, -8190, -8191, -8192, -8193, -16383, -16384, -16385, 0x7ffe};
  static const int LW[10] = {0x20000000, 0x1fffffff, 0x20000001, -0x20000000, -0x1fffffff, -0x20000001, 0x1ffffffe, 0x20000002, -0x20000002, -0x1ffffffe};
  int r, k, j;
  unsigned short a0 = 0;
  FPM_atan(0, 0, &a0);
  for (r = 0; r < 2; r++) {
    FaxFse *f;
    V27RxLoop loop;
    FaxFseCfg cfg;
    loop_init(&loop, r);
    fse_cfg_random(&cfg, r, 0, &loop);
    cfg.cb = hcb_d;
    cfg.nt = (short)(r == 0 ? 40 : 30);
    g_hcb_n = 0;
    f = (FaxFse *)sysdep_malloc(sizeof(FaxFse));
    FAXFSE_init(f, &cfg, 1);
    memset(f->diag0, 0, sizeof f->diag0);
    memset(f->tapsI, 0, (size_t)cfg.nt * 2);
    memset(f->tapsQ, 0, (size_t)cfg.nt * 2);
    memset(f->hbuf, 0, (size_t)cfg.nt * 2);
    for (k = 0; k < 7000; k++) {
      static short in[16], out[16];
      const short *crr = (const short *)cp(f->cfg.carrierTab);
      const short *pll1 = (const short *)cp(f->cfg.propTab), *pll2 = (const short *)cp(f->cfg.intTab);
      int n = 1 + (int)rmod(6), l48, iVar18, s10, s10d, mode = (int)rmod(4), e = 0, newe, target, s3a, g1, wtarget, D = LD[rmod(14)];
      short T1 = (short)L1[rmod(16)], CA;
      f->untilSym = (short)n;
      for (j = 0; j < 16; j++) in[j] = 0;
      s10 = T1;
      if (s10 < 0) s10 += 0x4000;
      if (s10 > 0x3fff) s10 -= 0x4000;
      s10d = (short)(s10 * 2);
      if (mode == 3) { /* the PLL branch s88 <= thr1c: s3a = 0 / 1; w70 so that w70 + gain * e + w3c lands on a wrap boundary (e = D: independent of the carrier table value) */
        int s88 = (int)rmod(2) ? 5 : 0;
        e = (short)D;
        if (e < -0x4000) e = (short)(e - 0x8000);
        if (e > 0x4000) e = (short)(e - 0x8000);
        f->cfg.settleCount = 5;
        f->settleCtr = (short)s88;
        s3a = (s88 < 5) ? 0 : 1;
        g1 = pll1[s3a];
        (void)pll2;
        f->freqAcc = (int)(rnd() & 0xfffu) - 0x800;
        wtarget = LW[rmod(10)];
        f->carrierAcc = WADD_H(WADD_H(wtarget, -(g1 * e)), -f->freqAcc);
      }
      /* the carrier table index after the n samples */
      l48 = (short)(f->phaseAcc + (short)(n * f->cfg.crrAdj));
      while ((short)l48 >= f->cfg.crrLen) l48 = (short)(l48 - f->cfg.crrLen);
      iVar18 = (short)(crr[l48] + (short)((f->carrierAcc + 0x4000) >> 15));
      f->predOffset = (short)((short)(((int)a0 - iVar18) >> 1) - T1);
      if (mode == 2) CA = (short)(2 * L1[rmod(16)] - iVar18);
      else CA = (short)(s10d - D);
      g_corner_angle = CA;
      e = (short)(s10d - CA);
      if (e < -0x4000) e = (short)(e - 0x8000);
      if (e > 0x4000) e = (short)(e - 0x8000);
      /* e50: so that the new smoothed error hits a boundary (the abs shifts, the thresholds) */
      target = L3[rmod(16)];
      {
        int second = (short)((e * 0x666) >> 15), est = (int)(((long long)(target - second) << 15) / 0x799a), c;
        f->phaseErrAvg = (short)est;
        for (c = est - 4; c <= est + 4; c++)
          if ((short)((short)((c * 0x799a) >> 15) + second) == (short)target) {
            f->phaseErrAvg = (short)c;
            break;
          }
        newe = (short)((short)((f->phaseErrAvg * 0x799a) >> 15) + second);
      }
      f->pllEnable = 1;
      f->predEnable = rpct(25);
      f->eqForce = (int)(rnd() & 1u);
      f->eqEnable = (int)(rnd() & 1u);
      if (mode != 3) { /* s88 > thr1c: the abs compare chooses s3a: thresholds on the new error */
        int an = newe < 0 ? -newe : newe;
        f->cfg.settleCount = 0;
        f->settleCtr = 5;
        f->cfg.errThreshHi = (short)(an + (int)rmod(3) - 1);
        f->cfg.errThreshLo = (short)(an + (int)rmod(3) - 1);
      }
      cov_hit(CV_F_RECV);
      stack_poison();
      j = FAXFSE_receive(f, in, out, (unsigned short)n);
      emit(0, "corner r%d k%d mode %d n=%d ret=%d e=%d newe=%d i18=%d", r, k, mode, n, j, e, newe, iVar18);
      if (k % 4 == 0) d_fse("corner", f);
    }
    d_fse("corner end", f);
    FAXFSE_free(f);
    sysdep_free(f);
    trial_end("fse corner");
  }
}
static void scen6(void)
{
  int trial;
  register_tables();
  capture(1);
  fse_counter_probe();
  fse_corner();
  for (trial = 0; trial < 160; trial++) {
    int r = trial & 1, variant = (int)(rnd() & 511u), dbg = trial % 7 == 3 ? 2 : trial % 7 == 5 ? 1 : 0;
    FaxFse *f;
    V27RxLoop loop;
    FaxFseCfg cfg;
    int calls, i, pos = (int)rmod(2000), tot = 0;
    loop_init(&loop, r);
    fse_cfg_random(&cfg, r, variant, &loop);
    if (trial % 5 == 0) cfg.cb = V27RX_epoch_det; /* the real callbacks (epoch -> train -> decision) */
    g_hcb_n = 0;
    f = (FaxFse *)sysdep_malloc(sizeof(FaxFse));
    dbg_level((unsigned)dbg);
    stack_poison();
    FAXFSE_init(f, &cfg, 1);
    memset(f->diag0, 0, sizeof f->diag0); /* the diagnostic ring is not initialised by the init: the pokes of nd0 below make its garbage visible */
    cov_hit(CV_F_INIT);
    cov_hit(CV_F_FRESH1);
    cov_hit(r == 0 ? CV_F_R2400 : CV_F_R4800);
    if (cfg.nt <= 8) cov_hit(CV_F_NT_SMALL);
    if (cfg.nt >= 60) cov_hit(CV_F_NT_BIG);
    d_fse("init", f);
    calls = 90 + (int)rmod(140);
    for (i = 0; i < calls; i++) {
      static short in[512], out[512];
      unsigned short n;
      int cap = cfg.blockLen, k, ret;
      n = (unsigned short)(rpct(5) ? 0 : rpct(5) ? 1 : rpct(30) ? (int)rmod(8) : 1 + (int)rmod((unsigned)(cap > 200 ? 200 : cap)));
      if (n > cap) n = (unsigned short)cap;
      for (k = 0; k < n; k++) in[k] = g_mrf[r][(pos + k) % g_mrfn[r]];
      if (rpct(10)) for (k = 0; k < n; k++) in[k] = (short)((int)(rnd() & 0xffffu) - 0x8000);
      pos = (pos + n) % g_mrfn[r];
      for (k = 0; k < 512; k++) out[k] = 0x7777;
      if (n == 0) cov_hit(CV_F_N0);
      if (n == 1) cov_hit(CV_F_N1);
      if (rpct(6)) f->pllEnable = (int)(rnd() & 1u);
      if (rpct(6)) f->predEnable = (int)(rnd() & 1u);
      if (rpct(5)) { /* the predictor coefficients are zero after the init and nobody writes them: poked */
        int q;
        for (q = 0; q < 4; q++) f->pc[q] = (short)((int)rmod(2000) - 1000);
      }
      if (rpct(6)) f->eqEnable = (int)(rnd() & 1u);
      if (rpct(5)) f->eqForce = (int)(rnd() & 1u);
      if (rpct(3)) f->decErrAvg = (short)((int)rmod(0x8000) - 0x100);
      if (rpct(4)) f->predOffset = (short)((int)rmod(65536) - 32768);
      if (rpct(4)) f->phaseErrAvg = (short)(rpct(50) ? (int)rmod(65536) - 32768 : (int)rmod(900) - 450);
      if (rpct(4)) f->settleCtr = (short)(rpct(30) ? -(int)rmod(5) : (int)rmod(60));
      if (rpct(8)) f->cfg.settleCount = (short)(rpct(70) ? (int)rmod(70) : (int)rmod(3));
      if (rpct(8)) f->cfg.errThreshHi = (short)(rpct(70) ? (int)rmod(450) : -(int)rmod(3));
      if (rpct(8)) f->cfg.errThreshLo = (short)(rpct(70) ? (int)rmod(450) : -(int)rmod(3));
      if (rpct(3)) f->carrierAcc = (int)((rnd() & 0x1ffu) << 15) + (rpct(50) ? 0x3fff : 0x4000 + (int)rmod(3) - 1) - 0x100000;
      if (rpct(3)) f->muIdx = (short)(rnd() & 1u);
      if (rpct(3)) f->nd0 = 0x1dd + (int)rmod(5);
      if (rpct(2) && n < 20) f->untilSym = 0; /* never 0 in operation: a symbol is produced from the stale history and the count is untouched */
      if (rpct(3)) f->settleCtr = (short)rmod(1100);
      if (rpct(2)) f->carrierAcc = (int)(rnd() & 0x3fffffffu) - 0x1fffffff;
      if (rpct(2)) f->phaseAcc = (short)rmod((unsigned)f->cfg.crrLen);
      if (f->eqForce) cov_hit(CV_F_C40);
      if (f->pllEnable) cov_hit(CV_F_C44);
      if (f->predEnable) cov_hit(CV_F_C48);
      if (f->eqEnable) cov_hit(CV_F_C4C);
      if (f->muIdx == 1) cov_hit(CV_F_MU1);
      cov_hit(CV_F_RECV);
      stack_poison();
      ret = FAXFSE_receive(f, in, out, n);
      tot += n;
      if (ret > 1) cov_hit(CV_F_MULTI);
      if (n > 0 && ret == 0) cov_hit(CV_F_PARTIAL);
      if (f->loopState == 0) cov_hit(CV_F_PLL_T0);
      if (f->loopState == 1) cov_hit(CV_F_PLL_T1);
      if (f->loopState == 2) cov_hit(CV_F_PLL_T2);
      if (f->predOffset != 0) cov_hit(CV_F_PRED);
      if (f->decErrAvg == 0x7fff) cov_hit(CV_F_E52SAT);
      if (f->settleCtr > 0) cov_hit(CV_F_S88);
      if (f->nd0 < 5 && tot > 200) cov_hit(CV_F_DIAGWRAP);
      if (f->phaseAcc + f->cfg.crrAdj >= f->cfg.crrLen) cov_hit(CV_F_CRRWRAP);
      emit(0, "fse call %d n=%u ret=%d in=%016llx out=%016llx", i, n, ret, fnv_n(in, n), fnv_n(out, 512));
      if (i % 3 == 0 || n == 0) d_fse("fse", f);
      if (rpct(2)) { /* re-init a live block: the buffers are freed and re-allocated (fresh = 0) */
        FaxFseCfg c2 = cfg;
        c2.nt = (short)(variant & 1 ? 4 + (int)rmod(44) : (r == 0 ? 97 : 81));
        c2.cb = hcb_a;
        FAXFSE_init(f, &c2, 0);
        cov_hit(CV_F_FRESH0);
        d_fse("reinit", f);
      }
    }
    d_fse("end", f);
    FAXFSE_free(f);
    cov_hit(CV_F_FREE);
    dbg_level(0);
    sysdep_free(f);
    trial_end("fse");
  }
}

static void sre_cfg_random(FaxSreCfg *c, int r, int variant)
{
  int ntmax = r == 0 ? 270 : 200;
  *c = h_sre_def;
  c->sps = (short)V27RX_SAMP_PER_BAUD[r];
  c->periodsTrack = (short)(variant & 1 ? 1 + (int)rmod(4) : 1);
  c->periodsLock = (short)(variant & 2 ? 2 + (int)rmod(10) : 8);
  c->startCount = (short)(variant & 4 ? 4 + (int)rmod(60) : 0x40);
  c->unlockGain = (short)(variant & 8 ? (int)rmod(0x7000) : 0x555);
  c->lockGain = (short)(variant & 8 ? (int)rmod(0x7000) : 0x4000);
  c->nt = (short)(variant & 16 ? 20 + 10 * (int)rmod((unsigned)((ntmax - 20) / 10)) : ntmax);
  c->coefTab = cfgp_word(r == 0 ? V27RX_SRE_FILT_2400 : V27RX_SRE_FILT_4800);
  c->tedCoef = cfgp_word(r == 0 ? V27RX_XB_COFFS_2400 : V27RX_XB_COFFS_4800);
  c->corrCosTab = cfgp_word(r == 0 ? V27RX_XCLOCK_2400 : V27RX_XCLOCK_4800);
  c->corrSinTab = cfgp_word(r == 0 ? V27RX_YCLOCK_2400 : V27RX_YCLOCK_4800);
  c->propGainTab = cfgp_word(r == 0 ? V27RX_SRE_PLLK1_2400 : V27RX_SRE_PLLK1_4800);
  c->intGainTab = cfgp_word(r == 0 ? V27RX_SRE_PLLK2_2400 : V27RX_SRE_PLLK2_4800);
  c->errOn = (short)(variant & 32 ? 1 + (int)rmod(6) : 2);
  c->errOff = (short)(variant & 32 ? 1 + (int)rmod(4) : 1);
  c->unlockThresh = (short)(variant & 64 ? (int)rmod(0x6000) : 0x3333);
  c->lockThresh = (short)(variant & 64 ? (int)rmod(0x6000) : 0x199a);
  c->levelThresh = (short)(variant & 128 ? (int)rmod(4000) : 2158);
  c->levelLen = (short)(variant & 256 ? 8 + (int)rmod(40) : c->sps * 3);
  c->w34 = (int)rnd();
}

/* ---- SRE corner mode: zero input and a zero loop-filter state make the timing error exactly 0, so the loop's angle (FPM_atan of the poked accumulators), the phase-error average and the integrator
 * target are known before the call: s3a is steered to the exact clamp window of the integrator output ---- */
static void sre_corner(void)
{
  int r, k;
  for (r = 0; r < 2; r++) {
    FaxSre *s;
    FaxSreCfg cfg;
    sre_cfg_random(&cfg, r, 0);
    s = (FaxSre *)sysdep_malloc(sizeof(FaxSre));
    FAXSRE_init(s, &cfg, 1);
    s->ppmTick = (short)(r == 0 ? 24 : 32);
    s->ppmPeriod = (short)(s->ppmTick * 200);
    s->ppmStep = (short)(1000000 / (cfg.sps * s->ppmPeriod));
    s->ppmNMax = (short)(1000000 / s->ppmPeriod);
    for (k = 0; k < 6000; k++) {
      static unsigned short in[4];
      static short out[8];
      const short *pll1 = (const short *)cp(s->cfg.propGainTab), *pll2 = (const short *)cp(s->cfg.intGainTab);
      int st = (int)rmod(3), ret, i;
      unsigned short ang;
      short a14, inner, base, target;
      static const int TI[12] = {29486, 29487, 29488, 29489, 29490, 29491, 29492, 29493, 29494, 29495, 29496, 29300};
      for (i = 0; i < 6; i++) s->st[i] = 0;
      in[0] = 0;
      s->pllEnable = 1;
      s->loopEnable = 1;
      s->errMagAvg = 0x7fff;
      s->levelMeasure = 0;
      s->untilSym = 1;
      s->corrIdx = (short)(s->cfg.sps - 1);
      s->periodCount = (short)(s->periodsCur - 1);
      s->loopState = (short)st;
      s->startCtr = (short)(s->cfg.startCount + 1 + (int)rmod(5));
      s->phaseAvg = (short)(st == 2 ? 0 : rpct(50) ? 0x4000 : -0x4000); /* state 2 stays locked: a small phase-error average */
      {
        int want = rpct(25) ? 0x7fe : -1, tries = 0, wang = rpct(10) ? 0x3fff + (int)rmod(5) : -1;
        unsigned short ang0v = 0; /* (0x7ff is not reachable: FPM_atan yields ~2100 distinct angles; none gives the residue) */
        for (;;) { /* the pair (w60, w5c) that gives the angle: for `want` the rounding residue of the second integrator term sits exactly at the carry */
          s->corrA = (int)rnd();
          s->corrB = (int)rnd();
          if (((s->corrA >> 16) > -0x40 && (s->corrA >> 16) < 0x40) || (s->corrB >> 16) == 0) continue; /* state 2 needs |high half of w5c| > 0x31 for the update to run */
          FPM_atan((short)(s->corrB >> 16), (short)(s->corrA >> 16), &ang);
          ang0v = ang;
          if (ang > 0x4000) ang = (unsigned short)(ang - 0x8000);
          a14 = (short)(((int)s->cfg.sps * (int)(short)ang) >> 3);
          ang0v = ang; /* (the angle before the quadrant fix) */
          if (wang >= 0 ? (ang0v == (unsigned short)wang || ++tries >= 60000) : (want < 0 || (((int)pll2[st] * (int)a14) & 0xfff) == want || ++tries >= 20000)) break;
        }
      }
      base = (short)((pll1[st] * (int)a14) >> 12);
      target = (short)(rpct(70) ? TI[rmod(12)] : (int)rmod(65536) - 32768);
      inner = target;
      s->loopInteg = (short)(inner - base);
      s->phaseFrac = (short)rmod(0x800);
      s->phaseRow = (short)rmod(10);
      (void)pll2;
      stack_poison();
      ret = (int)FAXSRE_recover(s, in, out, 1);
      emit(0, "sre corner r%d k%d st%d: ret=%d a14=%d inner=%d s3a=%d s64=%d s66=%d s38=%d", r, k, st, ret, a14, inner, s->loopInteg, s->phaseFrac, s->phaseRow, s->loopState);
      if (k % 5 == 0) d_sre("corner", s);
    }
    FAXSRE_free(s);
    sysdep_free(s);
    trial_end("sre corner");
  }
  /* tiny filters / windows: init and free only (a recover would write into a zero-sized block) */
  for (k = 0; k < 24; k++) {
    FaxSre *s = (FaxSre *)sysdep_malloc(sizeof(FaxSre));
    FaxSreCfg cfg;
    static const int nts[8] = {0, 1, 2, 5, 9, 10, 11, 19};
    sre_cfg_random(&cfg, k & 1, 0);
    cfg.nt = (short)nts[k & 7];
    cfg.levelLen = (short)(k % 3 == 0 ? 0 : k % 3 == 1 ? 1 : 2);
    FAXSRE_init(s, &cfg, 1);
    s->ppmTick = 24; /* written by the receiver's create, not by the init */
    s->ppmPeriod = 4800;
    s->ppmStep = 34;
    s->ppmNMax = 208;
    d_sre("tiny", s);
    FAXSRE_free(s);
    sysdep_free(s);
    trial_end("sre tiny");
  }
}

static void scen7(void)
{
  int trial;
  register_tables();
  capture(1);
  sre_corner();
  for (trial = 0; trial < 200; trial++) {
    int r = trial & 1, variant = (int)(rnd() & 511u), dbg = trial % 6 == 1 ? 2 : trial % 6 == 4 ? 1 : 0, sweep = trial % 3 == 2;
    FaxSre *s;
    FaxSreCfg cfg;
    int calls, i, pos = (int)rmod(2000);
    sre_cfg_random(&cfg, r, variant);
    if (rpct(10)) cfg.nt = (short)(10 + 10 * (int)rmod(3)); /* tiny filters: histLen 1 .. 3 */
    s = (FaxSre *)sysdep_malloc(sizeof(FaxSre));
    dbg_level((unsigned)dbg);
    stack_poison();
    FAXSRE_init(s, &cfg, 1);
    s->ppmTick = (short)(r == 0 ? 24 : 32); /* written by the receiver's create, not by the init */
    s->ppmPeriod = (short)(s->ppmTick * 200);
    s->ppmStep = (short)(1000000 / (cfg.sps * s->ppmPeriod));
    s->ppmNMax = (short)(1000000 / s->ppmPeriod);
    cov_hit(CV_R_INIT);
    cov_hit(CV_R_FRESH1);
    cov_hit(r == 0 ? CV_R_R2400 : CV_R_R4800);
    d_sre("init", s);
    calls = 120 + (int)rmod(160);
    for (i = 0; i < calls; i++) {
      static short in[512], out[512];
      unsigned short n = (unsigned short)(rpct(4) ? 0 : rpct(30) ? 1 + (int)rmod(10) : 1 + (int)rmod(150));
      int k, ret, gain = rpct(10) ? 1 + (int)rmod(7) : 8;
      for (k = 0; k < n; k++) in[k] = (short)((g_mrf[r][(pos + k) % g_mrfn[r]] * gain) >> 3);
      pos = (pos + n) % g_mrfn[r];
      for (k = 0; k < 512; k++) out[k] = 0x7777;
      if (n == 0) cov_hit(CV_R_N0);
      if (n > 100) cov_hit(CV_R_BIG);
      if (rpct(5)) s->loopEnable = (int)(rnd() & 1u);
      if (rpct(4)) s->pllEnable = (int)(rnd() & 1u);
      if (rpct(3)) s->levelMeasure = (short)(rnd() & 1u);
      if (rpct(2)) s->loopState = (short)rmod(3);
      if (rpct(2)) s->ppmFirst = (short)(rnd() & 1u);
      if (rpct(2)) s->ppmElapsed = (short)(s->ppmPeriod - s->ppmTick);
      if (rpct(3)) s->phaseAvg = (short)rmod(0x4000);
      if (rpct(5)) s->untilSym = (short)(2 + (int)rmod(5));
      if (rpct(2)) s->ppmN = (short)(s->ppmNMax - (int)rmod(3));
      if (rpct(2)) s->startCtr = (short)rmod(100);
      if (rpct(6)) { /* the loop accumulators / filter state: large values (clamps, the energy average) */
        int q;
        s->corrA = (int)rnd() >> (int)rmod(12);
        s->corrB = (int)rnd() >> (int)rmod(12);
        for (q = 0; q < 6; q++) s->st[q] = (short)((int)rmod(65536) - 32768);
        s->errMagAvg = (short)(rpct(50) ? 0x7fff - (int)rmod(40) : (int)rmod(0x8000));
      }
      if (rpct(5)) s->loopInteg = (short)((int)rmod(65536) - 32768);
      if (sweep) { /* the loop update runs at the first output sample; s3a / s3c / the state sweep the clamp windows */
        static int sw;
        sw += 7;
        s->corrIdx = (short)(s->cfg.sps - 1);
        s->periodCount = (short)(s->periodsCur - 1);
        s->loopEnable = 1;
        s->pllEnable = 1;
        s->loopInteg = (short)((sw & 1 ? 1 : -1) * (29200 + (sw * 13) % 700 - 350) + (rpct(30) ? (int)rmod(5) : 0));
        s->phaseAvg = (short)((int)rmod(2 * 0x2000) - 0x2000 + (rpct(50) ? 0x7ff : 0));
        s->startCtr = (short)(rpct(50) ? s->cfg.startCount + 1 : (int)rmod(100));
        if (rpct(60)) s->loopState = (short)(1 + (sw & 1));
        s->pllEnable = 1;
        s->phaseFrac = (short)((int)rmod(0x800));
        s->phaseRow = (short)rmod(10);
      }
      if (s->loopEnable) cov_hit(CV_R_W48_1); else cov_hit(CV_R_W48_0);
      if (s->pllEnable) cov_hit(CV_R_W40_1); else cov_hit(CV_R_W40_0);
      cov_hit(CV_R_RECOVER);
      {
        short before = s->loopState;
        stack_poison();
        ret = (int)FAXSRE_recover(s, (const unsigned short *)in, out, (short)n);
        if (s->loopState == 0) cov_hit(CV_R_STATE0);
        if (s->loopState == 1) cov_hit(CV_R_STATE1);
        if (s->loopState == 2) cov_hit(CV_R_STATE2);
        if (before == 1 && s->loopState == 2) cov_hit(CV_R_LOCKED);
        if (before == 2 && s->loopState == 1) cov_hit(CV_R_UNLOCK);
      }
      if (n > 0 && ret == 0) cov_hit(CV_R_PARTIAL);
      if (s->levelMeasure == 0) cov_hit(CV_R_RMS_GATE);
      if (s->histPos < 2 && n > 4) cov_hit(CV_R_HIST_WRAP);
      if (s->ppmOffset != 0) cov_hit(CV_R_PPM);
      if (s->ppmN >= s->ppmNMax && s->ppmNMax > 0) cov_hit(CV_R_PPM_FULL);
      if (s->ppmFirst == 0) cov_hit(CV_R_PPM_RESET);
      emit(0, "sre call %d n=%u ret=%d in=%016llx out=%016llx", i, n, ret, fnv_n(in, n), fnv_n(out, 512));
      if (i % 3 == 0 || n == 0 || ret == 0 || sweep) d_sre("sre", s);
      if (rpct(2)) { /* re-init a live block: grow (buffers re-allocated) / shrink / same (kept) */
        FaxSreCfg c2 = cfg;
        int ntmax = r == 0 ? 270 : 200;
        c2.nt = (short)(rpct(40) ? ntmax : rpct(50) ? 30 + 10 * (int)rmod((unsigned)((ntmax - 30) / 10)) : cfg.nt);
        if (c2.nt > s->cfg.nt) cov_hit(CV_R_REALLOC);
        c2.levelLen = cfg.levelLen;
        FAXSRE_init(s, &c2, 0);
        cov_hit(CV_R_FRESH0);
        d_sre("reinit", s);
        cfg.nt = c2.nt;
      }
    }
    d_sre("end", s);
    FAXSRE_free(s);
    cov_hit(CV_R_FREE);
    dbg_level(0);
    sysdep_free(s);
    trial_end("sre");
  }
}

/* ============================================================================================ main */
int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R21_SEED");
  g_verbose = getenv("R21_VERBOSE") != NULL;
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
    case 6: scen6(); break;
    case 7: scen7(); break;
    default: fprintf(stderr, "scenario 0..7\n"); return 2;
  }
  cov_finish();
  return 0;
}
