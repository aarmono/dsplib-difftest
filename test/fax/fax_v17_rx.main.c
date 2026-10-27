/*
 * R26 differential harness: the fax V.17 RECEIVER of dsplibs.o (V17RX_create / _delete / _modem / _control / _status, RxHdx{Start,EpochDet,Prtcol,Bridge,Scram,Data,Idle,Error}V17, RxNextStateV17,
 * DemodDataV17, DataCarrierDetectV17 / CarrierDetectV17 / EpochDetectV17 / QualityDetectV17 / GetSNRV17, StoreCoefV17 / Restore_rateV17, the v17rx_* VMI wrappers and the blob-LOCAL init_vmi_v17rx) vs
 * src/fax/fax_v17_rx.c + tables (the typed FAXFSE_* / FAXSRE_* of R21, the FSE decision callbacks / FaxDec17 / FAXVTB_decoder of R24 and the transmitter of R25 are the port's callees; their stand-alone
 * verification is r21_dsp / r24_dec / r25_*).  The blob run (-DTEST_BLOB, linked with ../dsplibs.o, init_vmi_v17rx made global by objcopy) and the port run (-m32 -O0 / -O2, x86-64 against the
 * recorded blob output) execute the same code; library test/fax/r26_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation, deterministic heap with block ids / red zones / poisoned
 * frees, callee recorders sysdep_malloc / sysdep_free / dsplibs_debug_printf).  The R18 / R24 / R25 / FPM callees are the real ones on both sides (the blob's own in the blob run, the files of src/ in the
 * port run): the transmit signal of the end-to-end streams is made by the blob's V17TX_* in the blob run and by src/fax/fax_v17_tx.c in the port runs (the two are proven identical by R25), so the receiver
 * sees the same samples.  Integer signals only (no libm).  Nothing printed depends on the host layout: objects are dumped member by member (pointers as table+index / block id, the content of every
 * malloc()ed block as a hash).  The hdx.s20 flag (never written by the blob's create: heap garbage) is set explicitly to 0 / 1 right after every create, on both sides.  The three caller-supplied
 * buffers of the short training (cfg coefI / coefQ / freq) belong to the harness (sysdep_malloc: ids logged); the FSE config's filter words w04 / w08 are printed as "a static table or not" (the blob
 * keeps the caller's ADDRESS there, the port a handle, 0 for a heap block: never read again after the FSE init).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 tables: every R26 table dumped, the default config words, v17rx_message -3 .. 13 and 0x100 ..;
 *   1 lifecycle: V17RX_create (NULL object / supplied with live hdx + bundle / hdx NULL / bundle NULL / live re-create / create(o, &o->cfg), NULL / custom / fuzzed cfg, the four rates + other, the short
 *     training flag with caller-supplied tap buffers, debug level 0 and 2), delete, run-after-create, forked probes: a failing malloc #0 .. #50, the rate-changing re-creates (red zone verdicts);
 *   2 VMI: fax_init_vmi_v17rx (blob: init_vmi_v17rx) with debug levels 0 .. 3, v17rx_create / process / status / control / delete / message on garbage-initialised FaxVmiV17 objects;
 *   3 receive streams: the transmitter of the same side (V17TX, R25) -> impairments (gain, noise, DC, clock slip, carrier loss, noise bursts, re-trains) -> V17RX_modem in random chunks, the four rates,
 *     long and short training, control (hold, re-create, gates), status, the whole engine dumped after EVERY call;
 *   4 handlers: every RxHdx*V17 / DemodDataV17 / DataCarrierDetectV17 / QualityDetectV17 / RxNextStateV17 / StoreCoef / Restore x hdx state x rate x poked decision state, on engines warmed up by real
 *     signals, sweeps of the carrier / idle / quality / protocol detectors, big-block probes;
 *   5 StoreCoefV17 / Restore_rateV17 directly, RxNextStateV17 for every state / rate / flag / debug level, V17RX_modem with a harness handler, V17RX_control for 512 flag-byte combinations, V17RX_status.
 * R26_VERBOSE=1 prints every line (debugging), R26_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r26_common.h"
#include "fax/fax_v17_rx_tables.h"
#include "fax/fax_v17_tx_tables.h"
#include "fax/fax_v17_decision_tables.h"
#include "fax/fax_v21_tables.h"
#include "fpm/fpm_tables.h"
#include "v32/v32_viterbi_tables.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
extern int init_vmi_v17rx(FaxVmiV17 *m, unsigned short rate, int unused, void *user);
#define fax_init_vmi_v17rx init_vmi_v17rx
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
/* x86-64: src/fax/fax_modem_wrappers.c references ALL eleven accessors of src/fax/fax_modem_views.h; this test uses the V.17 transmit ones (src/fax/fax_v17_tx.c) and the receive one (src/fax/fax_v17_rx.c).  The other
 * seven belong to the V.27ter / V.29 engines: stand-ins that abort (no scenario of this test reaches the wrappers that use them). */
static void *no_engine(void) { fprintf(stderr, "HARNESS/MODEL VIOLATION: accessor of another engine called\n"); exit(3); }
FaxSdm *faxw_v29_txsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
int *faxw_v29_encsel(void *o) { (void)o; return (int *)no_engine(); }
FaxSdm *faxw_v29_rxsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSdmV27 *faxw_v27_txsdm(void *o) { (void)o; return (FaxSdmV27 *)no_engine(); }
short *faxw_v27_rate(void *o) { (void)o; return (short *)no_engine(); }
FaxSdmV27 *faxw_v27_rxsdm(void *o) { (void)o; return (FaxSdmV27 *)no_engine(); }
#endif

#ifndef TEST_BLOB
/* the port runs link src/v32/v32_datapath.c (the ported VTB_decoder behind FAXVTB_decoder); the symbols that file references but this test never reaches are abort stubs (the blob build has the real ones) */
static void unreachable_stub(const char *n)
{
  fprintf(stderr, "r26: unexpected call of %s\n", n);
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

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_TAB, CV_MSG, CV_MSG_OOB, CV_MSG_NEG, CV_CFGDEF,
  CV_L_NEW, CV_L_SUP_LIVE, CV_L_SUP_NOHDX, CV_L_SUP_NOBUNDLE_PROBE, CV_L_RECREATE, CV_L_NULLCFG, CV_L_R7200, CV_L_R9600, CV_L_R12000, CV_L_R14400, CV_L_RBAD, CV_L_DBG, CV_L_DEL, CV_L_PROBE,
  CV_L_RUN, CV_L_USER, CV_L_RATECHG, CV_L_SAMERATE, CV_L_FLAG, CV_L_FLAGCHG, CV_L_FLAGLIVE,
  CV_V_INIT, CV_V_INIT_DBG, CV_V_CREATE, CV_V_CREATE_NULLCFG, CV_V_CREATE_SUP, CV_V_PROC, CV_V_STATUS, CV_V_CTL, CV_V_DEL, CV_V_MSG, CV_V_R7200, CV_V_R9600, CV_V_R12000, CV_V_R14400,
  CV_V_RBAD, CV_V_RET_CFG1,
  CV_S_ST0, CV_S_ST1, CV_S_ST2, CV_S_ST3, CV_S_ST4, CV_S_ST5, CV_S_ST6, CV_S_ST7, CV_S_TO1, CV_S_TO2, CV_S_TO3, CV_S_TO4, CV_S_TO5, CV_S_TO6, CV_S_IDLE_BACK, CV_S_ERR, CV_S_LOSS, CV_S_Q4E,
  CV_S_Q4A_FULL, CV_S_HOLD, CV_S_CTL, CV_S_RECREATE, CV_S_STATUS, CV_S_R7200, CV_S_R9600, CV_S_R12000, CV_S_R14400, CV_S_NOISE, CV_S_SLIP, CV_S_LOSSSEG, CV_S_BURST, CV_S_RETRAIN, CV_S_GAINLOW,
  CV_S_CLIP, CV_S_DC, CV_S_PATHB, CV_S_S2E, CV_S_N0, CV_S_N1, CV_S_NFULL, CV_S_NRAND, CV_S_BITS, CV_S_DBG, CV_S_CB_AB, CV_S_CB_EQ, CV_S_CB_BRIDGE, CV_S_CB_DEC, CV_S_C40, CV_S_BADRATE, CV_S_USER,
  CV_S_PROTO, CV_S_DATAOK, CV_S_DROP2, CV_S_ENERGY, CV_S_LOWSNR, CV_S_TONESEG, CV_S_V21CD, CV_S_SHORT, CV_S_LONG, CV_S_STORE, CV_S_RESTORE, CV_S_CODE9, CV_S_CODE8, CV_S_CODE7, CV_S_CODE6, CV_S_FLAGCTL,
  CV_H_START, CV_H_EPOCH, CV_H_PRTCOL, CV_H_BRIDGE, CV_H_SCRAM, CV_H_DATA, CV_H_IDLE, CV_H_ERROR, CV_H_STATEDEF, CV_H_HOLD, CV_H_PATHA, CV_H_PATHB, CV_H_CNT, CV_H_E52HI, CV_H_Q50, CV_H_ENERGY, CV_H_Q54,
  CV_H_S2C, CV_H_MTD2, CV_H_QUAL0, CV_H_QUAL1, CV_H_QUAL2, CV_H_QUAL3, CV_H_DEMOD, CV_H_DEMOD_MTD, CV_H_SRE_BIG, CV_H_CARR, CV_H_NOCARR, CV_H_CDET_BRANCH, CV_H_IDLE_SMALL, CV_H_N0, CV_H_NBIG,
  CV_H_WARM, CV_H_LOWSNR, CV_H_HIGHSNR, CV_H_PRTCOL_END, CV_H_V21CD, CV_H_STATUS, CV_H_FLAG, CV_H_C40, CV_H_NEXT, CV_H_STORE, CV_H_RESTORE, CV_H_DECERR_NC, CV_H_DECERR_PLAIN, CV_H_DECERR_TEXT,
  CV_C_NEXT_DBG, CV_C_NEXT_C0, CV_C_NEXT_C1, CV_C_NEXT_C2, CV_C_NEXT_C3, CV_C_NEXT_C4, CV_C_NEXT_C5, CV_C_NEXT_C6, CV_C_NEXT_DEF, CV_C_NEXT_FLAG1, CV_C_NEXT_FLAG0, CV_C_FAKE, CV_C_FAKE_NEG,
  CV_C_FAKE_PART, CV_C_CTL, CV_C_CTL_HOLD, CV_C_CTL_RECREATE, CV_C_CTL_W0, CV_C_CTL_W10, CV_C_CTL_NULL, CV_C_STATUS, CV_C_STATUS_NULL, CV_C_GETSNR, CV_C_EPOCHDET, CV_C_CARRDET, CV_C_STORE,
  CV_C_RESTORE, CV_C_STORE_ROUND, CV_C_CTL_FLAG
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_TAB, "tables dumped");
      cov_def(CV_CFGDEF, "default SRE / FSE config words");
      cov_def(CV_MSG, "v17rx_message idx 0..9");
      cov_def(CV_MSG_OOB, "v17rx_message idx 10 (out of table)");
      cov_def(CV_MSG_NEG, "v17rx_message idx > limit / negative");
      break;
    case 1:
      cov_def(CV_L_NEW, "V17RX_create NULL object");
      cov_def(CV_L_SUP_LIVE, "V17RX_create supplied live object");
      cov_def(CV_L_SUP_NOHDX, "V17RX_create supplied object, hdx NULL (bundle live)");
      cov_def(CV_L_SUP_NOBUNDLE_PROBE, "supplied object without bundle probe");
      cov_def(CV_L_RECREATE, "V17RX_create(o, &o->cfg)");
      cov_def(CV_L_NULLCFG, "V17RX_create NULL cfg");
      cov_def(CV_L_R7200, "V17RX_create rate 7200");
      cov_def(CV_L_R9600, "V17RX_create rate 9600");
      cov_def(CV_L_R12000, "V17RX_create rate 12000");
      cov_def(CV_L_R14400, "V17RX_create rate 14400");
      cov_def(CV_L_RBAD, "V17RX_create other rate (error flags)");
      cov_def(CV_L_DBG, "V17RX_create debug level 2");
      cov_def(CV_L_DEL, "V17RX_delete");
      cov_def(CV_L_PROBE, "create malloc-failure probe");
      cov_def(CV_L_RUN, "run after create");
      cov_def(CV_L_USER, "cfg user word != 0");
      cov_def(CV_L_RATECHG, "rate-changing re-create");
      cov_def(CV_L_SAMERATE, "same-rate re-create");
      cov_def(CV_L_FLAG, "short-training flag with caller buffers");
      cov_def(CV_L_FLAGCHG, "flag-changing re-create");
      cov_def(CV_L_FLAGLIVE, "short-training re-create of a live engine");
      break;
    case 2:
      cov_def(CV_V_INIT, "fax_init_vmi_v17rx");
      cov_def(CV_V_INIT_DBG, "init debug level 3");
      cov_def(CV_V_CREATE, "v17rx_create");
      cov_def(CV_V_CREATE_NULLCFG, "v17rx_create NULL cfg");
      cov_def(CV_V_CREATE_SUP, "v17rx_create with a supplied engine");
      cov_def(CV_V_PROC, "v17rx_process");
      cov_def(CV_V_STATUS, "v17rx_status");
      cov_def(CV_V_CTL, "v17rx_control");
      cov_def(CV_V_DEL, "v17rx_delete");
      cov_def(CV_V_MSG, "v17rx_message");
      cov_def(CV_V_R7200, "VMI rate 7200");
      cov_def(CV_V_R9600, "VMI rate 9600");
      cov_def(CV_V_R12000, "VMI rate 12000");
      cov_def(CV_V_R14400, "VMI rate 14400");
      cov_def(CV_V_RBAD, "VMI other rate");
      cov_def(CV_V_RET_CFG1, "v17rx_create returned the config word 1 (12000 / 14400)");
      break;
    case 3:
      cov_def(CV_S_ST0, "RX state 0 (start) observed");
      cov_def(CV_S_ST1, "RX state 1 (epoch detect) observed");
      cov_def(CV_S_ST2, "RX state 2 (protocol) observed");
      cov_def(CV_S_ST3, "RX state 3 (bridge) observed");
      cov_def(CV_S_ST4, "RX state 4 (scrambler) observed");
      cov_def(CV_S_ST5, "RX state 5 (data) observed");
      cov_def(CV_S_ST6, "RX state 6 (idle) observed");
      cov_def(CV_S_ST7, "RX state 7 (error) observed");
      cov_def(CV_S_TO1, "0 -> 1");
      cov_def(CV_S_TO2, "1 -> 2");
      cov_def(CV_S_TO3, "2 -> 3");
      cov_def(CV_S_TO4, "3 -> 4 or 2 -> 4");
      cov_def(CV_S_TO5, "4 -> 5");
      cov_def(CV_S_TO6, "5 -> 6");
      cov_def(CV_S_IDLE_BACK, "6 -> 5");
      cov_def(CV_S_ERR, "-> 7");
      cov_def(CV_S_LOSS, "state byte 4 (loss of carrier)");
      cov_def(CV_S_Q4E, "q4e set");
      cov_def(CV_S_Q4A_FULL, "q4a reached 0x33");
      cov_def(CV_S_HOLD, "hold control");
      cov_def(CV_S_CTL, "control");
      cov_def(CV_S_RECREATE, "control re-create");
      cov_def(CV_S_STATUS, "status");
      cov_def(CV_S_R7200, "rate 7200 stream");
      cov_def(CV_S_R9600, "rate 9600 stream");
      cov_def(CV_S_R12000, "rate 12000 stream");
      cov_def(CV_S_R14400, "rate 14400 stream");
      cov_def(CV_S_NOISE, "noisy stream");
      cov_def(CV_S_SLIP, "clock slip stream");
      cov_def(CV_S_LOSSSEG, "carrier loss segment");
      cov_def(CV_S_BURST, "noise burst");
      cov_def(CV_S_RETRAIN, "transmitter re-train");
      cov_def(CV_S_GAINLOW, "low level stream");
      cov_def(CV_S_CLIP, "clipping stream");
      cov_def(CV_S_DC, "DC offset stream");
      cov_def(CV_S_PATHB, "hdx.s20 != 0 stream (carrier detector path B)");
      cov_def(CV_S_S2E, "path B with s2e set");
      cov_def(CV_S_N0, "call with n = 0");
      cov_def(CV_S_N1, "call with n = 1");
      cov_def(CV_S_NFULL, "call with a 120-sample block");
      cov_def(CV_S_NRAND, "call with a random block");
      cov_def(CV_S_BITS, "symbols returned");
      cov_def(CV_S_DBG, "debug level 2 stream");
      cov_def(CV_S_CB_AB, "FSE callback = AB observed");
      cov_def(CV_S_CB_EQ, "FSE callback = eqtrn observed");
      cov_def(CV_S_CB_BRIDGE, "FSE callback = bridge observed");
      cov_def(CV_S_CB_DEC, "FSE callback = trellis decision observed");
      cov_def(CV_S_C40, "FSE c40 set");
      cov_def(CV_S_BADRATE, "stream with an invalid rate word");
      cov_def(CV_S_USER, "stream with a user word");
      cov_def(CV_S_PROTO, "protocol state with a block returned");
      cov_def(CV_S_DATAOK, "data state with the quality flag");
      cov_def(CV_S_ENERGY, "energy drop");
      cov_def(CV_S_TONESEG, "V.21 tone segment");
      cov_def(CV_S_V21CD, "V17: V21 Carrier detected");
      cov_def(CV_S_SHORT, "short training stream (flag set)");
      cov_def(CV_S_LONG, "long training stream");
      cov_def(CV_S_STORE, "StoreCoefV17 reached in a stream");
      cov_def(CV_S_RESTORE, "Restore_rateV17 reached in a stream");
      cov_def(CV_S_CODE9, "connect code 9 (7200)");
      cov_def(CV_S_CODE8, "connect code 8 (9600)");
      cov_def(CV_S_CODE7, "connect code 7 (12000)");
      cov_def(CV_S_CODE6, "connect code 6 (14400)");
      cov_def(CV_S_FLAGCTL, "control changed the training flag");
      break;
    case 4:
      cov_def(CV_H_START, "RxHdxStartV17");
      cov_def(CV_H_EPOCH, "RxHdxEpochDetV17");
      cov_def(CV_H_PRTCOL, "RxHdxPrtcolV17");
      cov_def(CV_H_BRIDGE, "RxHdxBridgeV17");
      cov_def(CV_H_SCRAM, "RxHdxScramV17");
      cov_def(CV_H_DATA, "RxHdxDataV17");
      cov_def(CV_H_IDLE, "RxHdxIdleV17");
      cov_def(CV_H_ERROR, "RxHdxErrorV17");
      cov_def(CV_H_STATEDEF, "state number outside 0..6");
      cov_def(CV_H_HOLD, "Data with hold");
      cov_def(CV_H_PATHA, "DataCarrierDetectV17 path A");
      cov_def(CV_H_PATHB, "DataCarrierDetectV17 path B");
      cov_def(CV_H_CNT, "dec.ctr > 0x3e7");
      cov_def(CV_H_E52HI, "fse.e52 > 0x3fff");
      cov_def(CV_H_Q50, "energy test enabled");
      cov_def(CV_H_ENERGY, "energy drop detected");
      cov_def(CV_H_Q54, "q54 reached 2");
      cov_def(CV_H_S2C, "hdx.s2c > 0x4ff");
      cov_def(CV_H_MTD2, "hdx MTD2 fired");
      cov_def(CV_H_QUAL0, "QualityDetectV17 q4a == 0");
      cov_def(CV_H_QUAL1, "QualityDetectV17 q4a < 0x32");
      cov_def(CV_H_QUAL2, "QualityDetectV17 q4a == 0x32");
      cov_def(CV_H_QUAL3, "QualityDetectV17 q4a > 0x32");
      cov_def(CV_H_DEMOD, "DemodDataV17");
      cov_def(CV_H_DEMOD_MTD, "DemodDataV17 aborted by the tone detector");
      cov_def(CV_H_SRE_BIG, "SRE output > 164");
      cov_def(CV_H_CARR, "carrier detected");
      cov_def(CV_H_NOCARR, "no carrier");
      cov_def(CV_H_CDET_BRANCH, "carrier detect with the gate off");
      cov_def(CV_H_IDLE_SMALL, "Idle: decision error small");
      cov_def(CV_H_N0, "n = 0");
      cov_def(CV_H_NBIG, "n >= 150");
      cov_def(CV_H_WARM, "warmed up engine");
      cov_def(CV_H_LOWSNR, "SNR flag set (GetSNRV17 <= 8)");
      cov_def(CV_H_HIGHSNR, "SNR flag clear (GetSNRV17 > 8)");
      cov_def(CV_H_PRTCOL_END, "protocol / bridge / scram state finished (count <= 0)");
      cov_def(CV_H_V21CD, "V17: V21 Carrier detected (debug text)");
      cov_def(CV_H_STATUS, "V17RX_status on a driven engine");
      cov_def(CV_H_FLAG, "hdx.flag set");
      cov_def(CV_H_C40, "fse.c40 set");
      cov_def(CV_H_NEXT, "RxNextStateV17 on a driven engine");
      cov_def(CV_H_STORE, "StoreCoefV17 on a driven engine");
      cov_def(CV_H_RESTORE, "Restore_rateV17 on a driven engine");
      cov_def(CV_H_DECERR_NC, "Decoder error too big: no carrier (flag, c40, ctr > 0x3e7, e52 > 0x3fff)");
      cov_def(CV_H_DECERR_PLAIN, "flag, c40, ctr > 0x3e7 with a small error (mask & 1)");
      cov_def(CV_H_DECERR_TEXT, "Decoder error too big debug text");
      cov_def(CV_C_GETSNR, "GetSNRV17");
      cov_def(CV_C_EPOCHDET, "EpochDetectV17");
      cov_def(CV_C_CARRDET, "CarrierDetectV17");
      break;
    case 5:
      cov_def(CV_C_NEXT_DBG, "NextState debug strings");
      cov_def(CV_C_NEXT_C0, "RxNextStateV17 state 0");
      cov_def(CV_C_NEXT_C1, "RxNextStateV17 state 1");
      cov_def(CV_C_NEXT_C2, "RxNextStateV17 state 2");
      cov_def(CV_C_NEXT_C3, "RxNextStateV17 state 3");
      cov_def(CV_C_NEXT_C4, "RxNextStateV17 state 4");
      cov_def(CV_C_NEXT_C5, "RxNextStateV17 state 5");
      cov_def(CV_C_NEXT_C6, "RxNextStateV17 state 6");
      cov_def(CV_C_NEXT_DEF, "RxNextStateV17 default");
      cov_def(CV_C_NEXT_FLAG1, "RxNextStateV17 with the short-training flag");
      cov_def(CV_C_NEXT_FLAG0, "RxNextStateV17 with the long-training flag");
      cov_def(CV_C_FAKE, "V17RX_modem with a harness handler");
      cov_def(CV_C_FAKE_NEG, "handler returns a negative count");
      cov_def(CV_C_FAKE_PART, "handler consumes part of the block");
      cov_def(CV_C_CTL, "V17RX_control");
      cov_def(CV_C_CTL_HOLD, "control hold");
      cov_def(CV_C_CTL_RECREATE, "control re-create");
      cov_def(CV_C_CTL_W0, "control b0c bit 3");
      cov_def(CV_C_CTL_W10, "control b0c bit 5");
      cov_def(CV_C_CTL_NULL, "control NULL");
      cov_def(CV_C_CTL_FLAG, "control with a flag change");
      cov_def(CV_C_STATUS, "V17RX_status");
      cov_def(CV_C_STATUS_NULL, "status NULL");
      cov_def(CV_C_STORE, "StoreCoefV17");
      cov_def(CV_C_RESTORE, "Restore_rateV17");
      cov_def(CV_C_STORE_ROUND, "StoreCoefV17 / Restore round trip");
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
  if (!strncmp(b, "V17RX_STATE_START", 17)) cov_hit(CV_C_NEXT_C0);
  if (!strncmp(b, "V17RX_STATE_EPOCH_DET", 21)) cov_hit(CV_C_NEXT_C1);
  if (!strncmp(b, "V17RX_STATE_PROTOCOL", 20)) cov_hit(CV_C_NEXT_C2);
  if (!strncmp(b, "V17RX_STATE_BRIDGE", 18)) cov_hit(CV_C_NEXT_C3);
  if (!strncmp(b, "V17RX_STATE_SCRAM", 17)) cov_hit(CV_C_NEXT_C4);
  if (!strncmp(b, "V17RX_STATE_DATA", 16)) cov_hit(CV_C_NEXT_C5);
  if (!strncmp(b, "V17RX_STATE_IDLE", 16)) cov_hit(CV_C_NEXT_C6);
  if (!strncmp(b, "V17RX_DEFAULT", 13)) cov_hit(CV_C_NEXT_DEF);
  if (!strncmp(b, "ERROR: SRE buffer", 17)) cov_hit(CV_H_SRE_BIG);
  if (!strncmp(b, "Decision error is small", 23)) cov_hit(CV_H_IDLE_SMALL);
  if (!strncmp(b, "sudden energy", 13)) cov_hit(CV_H_ENERGY);
  if (!strncmp(b, "V17 Decoder error", 17)) cov_hit(CV_H_DECERR_TEXT);
  if (!strncmp(b, "V17: V21 Carrier", 16)) { cov_hit(CV_S_V21CD); cov_hit(CV_H_V21CD); }
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
static void prt_status(const char *t, const FaxV17Status *st)
{
  emit(0, "%s %d %d %d %d %d %d %d %d %d %02x %02x", t, st->s0, st->s2, st->s4, st->s6, st->s8, st->sa, st->se, st->s10, st->s12, st->b14, st->b15); /* sc and w18 are never written */
}
static const void *cp(int w) { return cfgp_ptr(w); } /* a CFGP word as the address of its table */
static const short *g_agcbase; /* the AGCv17_CFG alpha table (a blob LOCAL: named by position) */

static void register_tables(void)
{
  htab_add("MTD_V17", V17_MTD_COEFF, sizeof V17_MTD_COEFF);
  htab_add("MRF_COFFS", MRFv17_COFFS, sizeof MRFv17_COFFS);
  htab_add("SRE_COFFS", SREv17_COFFS, sizeof SREv17_COFFS);
  htab_add("XB", SREv17_XB_COFFS, sizeof SREv17_XB_COFFS);
  htab_add("XCLOCK", SREv17_xCLOCK, sizeof SREv17_xCLOCK);
  htab_add("YCLOCK", SREv17_yCLOCK, sizeof SREv17_yCLOCK);
  htab_add("SRE_PLLK1", SREv17_PLL_K1, sizeof SREv17_PLL_K1);
  htab_add("SRE_PLLK1_S", SREv17_PLL_K1_S, sizeof SREv17_PLL_K1_S);
  htab_add("SRE_PLLK2", SREv17_PLL_K2, sizeof SREv17_PLL_K2);
  htab_add("FSE_ICOFF", FSEv17_ICOFF, sizeof FSEv17_ICOFF);
  htab_add("FSE_QCOFF", FSEv17_QCOFF, sizeof FSEv17_QCOFF);
  htab_add("CRR_CLK", CRRv17_CLK, sizeof CRRv17_CLK);
  htab_add("CRR_PLLK1", CRRv17_PLL_K1, sizeof CRRv17_PLL_K1);
  htab_add("CRR_PLLK1_S", CRRv17_PLL_K1_S, sizeof CRRv17_PLL_K1_S);
  htab_add("CRR_PLLK2", CRRv17_PLL_K2, sizeof CRRv17_PLL_K2);
  htab_add("MTD_V21CH2", V21_CHAN2_MTD_COEFF, sizeof V21_CHAN2_MTD_COEFF);
  htab_add("VTB_I16", VTBv17_IMAP16T, sizeof VTBv17_IMAP16T);
  htab_add("VTB_Q16", VTBv17_QMAP16T, sizeof VTBv17_QMAP16T);
  htab_add("VTB_I32", VTBv17_IMAP32, sizeof VTBv17_IMAP32);
  htab_add("VTB_Q32", VTBv17_QMAP32, sizeof VTBv17_QMAP32);
  htab_add("VTB_I64", VTBv17_IMAP64, sizeof VTBv17_IMAP64);
  htab_add("VTB_Q64", VTBv17_QMAP64, sizeof VTBv17_QMAP64);
  htab_add("VTB_I128", VTBv17_IMAP128, sizeof VTBv17_IMAP128);
  htab_add("VTB_Q128", VTBv17_QMAP128, sizeof VTBv17_QMAP128);
  htab_add("VTB_BOUND_7200", VTB_BOUND_7200, sizeof VTB_BOUND_7200);
  htab_add("VTB_BOUND_9600", VTB_BOUND_9600, sizeof VTB_BOUND_9600);
  htab_add("VTB_BOUND_12000", VTB_BOUND_12000, sizeof VTB_BOUND_12000);
  htab_add("VTB_BOUND_14400", VTB_BOUND_14400, sizeof VTB_BOUND_14400);
  htab_add("VTB_REGION_7200", VTB_REGION_7200, sizeof VTB_REGION_7200);
  htab_add("VTB_REGION_9600", VTB_REGION_9600, sizeof VTB_REGION_9600);
  htab_add("VTB_REGION_12000", VTB_REGION_12000, sizeof VTB_REGION_12000);
  htab_add("VTB_REGION_14400", VTB_REGION_14400, sizeof VTB_REGION_14400);
  g_agcbase = (const short *)cp(AGCv17_CFG[3]);
}
static const char *rxfn(V17RxHdxFn f)
{
  if (f == RxHdxStartV17) return "Start";
  if (f == RxHdxEpochDetV17) return "EpochDet";
  if (f == RxHdxPrtcolV17) return "Prtcol";
  if (f == RxHdxBridgeV17) return "Bridge";
  if (f == RxHdxScramV17) return "Scram";
  if (f == RxHdxDataV17) return "Data";
  if (f == RxHdxIdleV17) return "Idle";
  if (f == RxHdxErrorV17) return "Error";
  return f ? "other" : "NULL";
}
static const char *cbname(FaxFseCb f)
{
  if (f == FAX_FSE_decision_AB) return "AB";
  if (f == FSE_decision_eqtrn) return "eqtrn";
  if (f == FSE_Bridge_det) return "bridge";
  if (f == FAX_FSE_decision_16pt) return "dec16";
  if (f == FAX_FSE_decision_32pt) return "dec32";
  if (f == FAX_FSE_decision_64pt) return "dec64";
  if (f == FAX_FSE_decision_128pt) return "dec128";
  return f ? "other" : "NULL";
}
static void d_agc(const char *t, const FpmAgc *a)
{
  emit(0, "%s AGC a=%08x %08x %08x alpha=%d beta=%d w5=%08x act=%d det=%d hist=%d rcp=%d sh=%d frz=%d", t, u(a->a.w[0]), u(a->a.w[1]), u(a->a.w[2]), (int)(a->alpha - g_agcbase),
       (int)(a->beta - (const short *)cp(AGCv17_CFG[4])), u(a->w5), a->active, a->detect, a->hist, a->rcp, a->shift, a->freeze); /* AGC.s22 is never written by FPM_AGC_init */
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
/* the FSE: the filter words w04 / w08 are "a static table" or "not" (see the file header) */
static const char *tabname_or_dash(int w, char *buf, size_t cap)
{
  const void *p = cp(w);
  int i;
  for (i = 0; i < g_nhtab; i++) {
    const char *b = (const char *)g_htab[i].base;
    if ((const char *)p >= b && (const char *)p < b + g_htab[i].bytes) return dptr(p, buf, cap);
  }
  return "-";
}
static void d_fse(const char *t, const FaxFse *f)
{
  char p1[40], p2[40], p3[40], p4[40], p5[40], q1[80], q2[80], q3[80], q4[80], q5[80];
  const FaxFseCfg *c = &f->cfg;
  int n0 = f->nd0 < 0 ? 0 : f->nd0 > FAXFSE_DIAG0 ? FAXFSE_DIAG0 : f->nd0;
  emit(0, "%s FSE cfg s00=%d s02=%d w04=%s w08=%s nt=%d mu=%d,%d,%d w14=%s crrLen=%d crrAdj=%d thr=%d,%d,%d s22=%d w24=%s w28=%s cb=%s w34=%08x", t, c->blockLen, c->sps,
       tabname_or_dash(c->iCoefTab, p1, 40), tabname_or_dash(c->qCoefTab, p2, 40), c->nt, c->mu[0], c->mu[1], c->mu[2], dptr(cp(c->carrierTab), p3, 40), c->crrLen, c->crrAdj, c->settleCount, c->errThreshHi, c->errThreshLo, c->s22,
       dptr(cp(c->propTab), p4, 40), dptr(cp(c->intTab), p5, 40), cbname(c->cb), u(c->w34));
  emit(0, "%s FSE st muIdx=%d s3a=%d w3c=%08x c40=%d c44=%d c48=%d c4c=%d e50=%d e52=%d outI=%s outQ=%s count=%u idx=%d tapsI=%s tapsQ=%s hbuf=%s pos=%d w70=%08x s74=%d s76=%d pc=%d,%d,%d,%d "
       "ph=%d,%d,%d,%d s88=%d s8a=%d nd=%d,%d,%d diag=%016llx",
       t, f->muIdx, f->loopState, u(f->freqAcc), f->eqForce, f->pllEnable, f->predEnable, f->eqEnable, f->phaseErrAvg, f->decErrAvg, (hblk(q1, 80, f->outI), q1), (hblk(q2, 80, f->outQ), q2), f->count, f->idx, (hblk(q3, 80, f->tapsI), q3),
       (hblk(q4, 80, f->tapsQ), q4), (hblk(q5, 80, f->hbuf), q5), f->pos, u(f->carrierAcc), f->phaseAcc, f->predOffset, f->pc[0], f->pc[1], f->pc[2], f->pc[3], f->ph[0], f->ph[1], f->ph[2], f->ph[3], f->settleCtr, f->untilSym,
       f->nd0, f->nd1, f->nd2, fnv_ints(f->diag0, (unsigned)n0 * 2));
}
static void d_sgd(const char *t, const FaxSgd *s)
{
  char p[80], q[40], r[40];
  if (!s) {
    emit(0, "%s SGD NULL", t);
    return;
  }
  emit(0, "%s SGD bits=%u nbuf=%d w4=%d w6=%u tab=%s len=%08x mode=%08x w14=%08x w18=%08x w1c=%08x tab2=%s w24=%08x w28=%08x w2c=%08x w30=%08x pass=%u pos=%u symbit=%u bufn=%d scale=%u buf=%s", t,
       s->cfg.bits, s->cfg.nbuf, s->cfg.w4, s->cfg.w6, dptr(s->cfg.tab, q, 40), u(s->cfg.len), u(s->cfg.mode), u(s->cfg.tailValue), u(s->cfg.pattern), u(s->cfg.patSyms), dptr(s->cfg.tab2, r, 40), u(s->cfg.w24),
       u(s->cfg.w28), u(s->cfg.w2c), u(s->cfg.w30), s->pass, s->pos, s->symbit, s->bufn, s->scale, (hblk(p, 80, s->buf), p));
}
static void d_dec(const char *t, const FaxDec17 *d)
{
  char p1[80], p2[40], p3[40], p4[40], p5[40];
  const FaxVtb *v = &d->vtb;
  emit(0, "%s DEC a3c=%u h=%d,%d,%d,%d,%d,%d acc=%d,%d w50=%d s54=%d s56=%d cnt=%d reg=%08x flag=%d sel=%d s66=%d ctr=%u", t, d->prevAngle, d->abI0, d->abQ0, d->abI1, d->abQ1, d->abI2, d->abQ2, d->accA, d->accB,
       d->w50, d->s54, d->s56, d->cnt, u(d->reg), d->flag, d->sel, d->s66, d->ctr);
  emit(0, "%s VTB hist=%s metric=%d,%d,%d,%d,%d,%d,%d,%d wr=%d imap=%s qmap=%s bound=%s sel=%d region=%s size=%d depth=%d prev=%d lowMask=%u shift=%d", t, (hblk(p1, 80, v->hist), p1), v->metric[0],
       v->metric[1], v->metric[2], v->metric[3], v->metric[4], v->metric[5], v->metric[6], v->metric[7], v->wr, dptr(cp(v->imap), p2, 40), dptr(cp(v->qmap), p3, 40), dptr(cp(v->bound), p4, 40), v->sel,
       dptr(cp(v->region), p5, 40), v->size, v->depth, v->prevQuad, (unsigned)v->lowMask, v->shift);
  d_sgd(t, d->sgd);
}
static void d_rx(const char *t, const V17Rx *o)
{
  char p1[80], p2[80], p3[80], p4[80], p5[80], p6[80], p7[80], p8[80], p9[80];
  emit(0, "%s RX cfg=%08x %08x %08x %08x %08x %08x coefI=%s coefQ=%s freq=%s user=%08x st=%08x fseI=%s fseQ=%s idx=%s tapsI=%s tapsQ=%s nt=%d w44=%d w48=%d s4c=%d w50=%d w54=%d s58=%d", t,
       u(o->cfg.w[0]), u(o->cfg.w[1]), u(o->cfg.w[2]), u(o->cfg.w[3]), u(o->cfg.w[4]), u(o->cfg.w[5]), (hblk(p7, 80, o->cfg.coefI), p7), (hblk(p8, 80, o->cfg.coefQ), p8),
       (hblk(p9, 80, o->cfg.freq), p9), u(o->cfg.user), u(o->st.w), (hblk(p1, 80, o->fseI), p1), (hblk(p2, 80, o->fseQ), p2),
       o->bundle ? (o->fseIdx == &o->bundle->fse.idx ? "&fse.idx" : "other") : "-", (hblk(p3, 80, o->fseTapsI), p3), (hblk(p4, 80, o->fseTapsQ), p4), o->fseNt, o->w44, o->w48, o->s4c, o->w50,
       o->w54, o->s58);
  if (o->hdx) {
    const V17RxHdx *h = o->hdx;
    emit(0, "%s RXH hold=%d rate=%d flag=%d fn=%s state=%d count=%d buf=%s s20=%d buf2=%s s2c=%d s2e=%d", t, h->hold, h->rate, h->flag, rxfn(h->fn), h->state, h->count, (hblk(p5, 80, h->buf), p5), h->s20,
         (hblk(p6, 80, h->buf2), p6), h->sinceTone, h->toneSearch);
    d_mtd(t, h->mtd);
    d_tone(t, h->tone);
    d_mtd(t, h->mtd2);
    d_agc(t, &h->agc);
  }
  if (o->bundle) {
    const V17RxBundle *b = o->bundle;
    emit(0, "%s RXB w=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d rate24=%d buf1=%s buf2=%s q=%d,%d,%d,%d,%d,%d,%d", t, b->w0, b->sreGate, b->pllGate, b->wc, b->eqGate, b->w14, b->w18, b->w1c, b->w20, b->w28, b->rate24,
         (hblk(p1, 80, b->buf1), p1), (hblk(p2, 80, b->buf2), p2), b->decErrSmooth, b->qualBlocks, b->errLimit, b->qualityOk, b->energyTest, b->refLevel, b->refBlocks);
    d_dec(t, &b->dec);
    emit(0, "%s RXB sdm n=%d tap=%d,%d mask=%08x nmask=%08x state=%08x sh=%d,%d", t, b->sdm.n, b->sdm.tap1, b->sdm.tap2, b->sdm.mask, b->sdm.nmask, b->sdm.state, b->sdm.sh1, b->sdm.sh2);
    d_mrf(t, &b->mrf);
    d_agc(t, &b->agc);
    d_sre(t, &b->sre);
    d_fse(t, &b->fse);
  }
  (void)p3; (void)p4;
}
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

/* an integer tone source (the table of test/fax/fax_v21.main.c) at the 9600 Hz sampling of the V.17 receiver: the carrier detector's V.21 channel-2 tone detector (V21_CHAN2_MTD_COEFF) returns 0 for a pure
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

/* rate codes: 0 = 7200, 1 = 9600, 2 = 12000, 3 = 14400, anything else is the raw baud word */
static int rate_word(int rate) { return rate == 0 ? 0x1c20 : rate == 1 ? 0x2580 : rate == 2 ? 0x2ee0 : rate == 3 ? 0x3840 : rate; }
static int rate_bits(int rate) { return rate >= 0 && rate <= 3 ? 3 + rate : 6; }
/* the transmitter (src/fax/fax_v17_tx.c in the port run, the blob's own in the blob run): config like R25's make_cfg; shortt: the short training (cfg word 6) */
static void make_tcfg(int *w, int rate, int shortt)
{
  int i;
  for (i = 0; i < 8; i++) w[i] = V17TX_CFG[i];
  w[0] = (w[0] & 0xffff) | (rate_word(rate) << 16); /* s[1] = the baud rate */
  if (shortt) w[6] = 1;
}
typedef struct {
  V17Tx *o;
  int rate, frm;
} Gen;
static void gen_new(Gen *g, int rate, int shortt)
{
  int w[8];
  make_tcfg(w, rate, shortt);
  g->o = V17TX_create(NULL, w);
  g->rate = rate;
  g->frm = 48;
}
/* one frame of transmitter samples; data symbols random (full constellation) or constant */
static int gen_frame(Gen *g, short *dst, int sym_mode)
{
  unsigned short n = (unsigned short)g->frm;
  int i, bits = rate_bits(g->rate);
  for (i = 0; i < 4096; i++) g_tx[i] = (short)(sym_mode == 0 ? (int)(rnd() & ((1u << bits) - 1u)) : sym_mode == 1 ? 0 : (i & ((1 << bits) - 1)));
  V17TX_modem(g->o, g_tx, g_txo, &n);
  if (n > 400) die3("transmitter frame too long");
  memcpy(dst, g_txo, (size_t)n * 2);
  return n;
}
static void gen_free(Gen *g)
{
  V17TX_delete(g->o);
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

/* the receiver config: V17RX_CFG with the rate word; `variant` bits: 1 user word, 2 word 2 (overwritten by the control), 4 words 0 / 3 / 4, 8 the high half of word 1; `flag` != 0: the short training.
 * The three caller buffers of the short training (cfg coefI / coefQ: 0x31 shorts, freq: one short) are ALWAYS supplied (the control can switch the flag on later): harness-owned (ubuf_free) */
static void ubuf_fill(V17RxCfg *c, int variant)
{
  int k;
  for (k = 0; k < 0x31; k++) {
    c->coefI[k] = (short)(FSEv17_ICOFF[k] + (variant & 16 ? (int)(rnd() & 0x3fu) - 32 : 0));
    c->coefQ[k] = (short)(FSEv17_QCOFF[k] + (variant & 16 ? (int)(rnd() & 0x3fu) - 32 : 0));
  }
  c->freq[0] = (short)(variant & 32 ? (int)(rnd() & 0x7fffu) - 0x4000 : 0);
}
/* the bytes around a caller buffer (the 4 bytes in front of it, the padding behind its 0x62 bytes) hold known values while the engine lives: a copy loop that starts early or runs on copies them into the
 * taps (whose red zone and padding the heap checks watch: with the heap's own 0xCC fill the copy would be invisible) */
static void ubuf_poison(short *b, int on)
{
  unsigned char *p = (unsigned char *)b;
  int k;
  for (k = 1; k <= 4; k++) p[-k] = (unsigned char)(on ? 0x41 + k * 3 : 0xcc);
  for (k = 0x62; k < 0x70; k++) p[k] = (unsigned char)(on ? 0x60 + k : 0xcc);
}
static void make_rcfg(V17RxCfg *c, int rate, int variant, int flag)
{
  int i;
  for (i = 0; i < 6; i++) c->w[i] = V17RX_CFG[i];
  c->user = V17RX_CFG[9];
  c->s[2] = (short)rate_word(rate);
  if (variant & 1) c->user = (int)rnd();                    /* user word */
  if (variant & 2) c->w[2] = (int)rnd();                    /* overwritten by the control */
  if (variant & 4) c->w[0] = (int)rnd(), c->w[3] = (int)rnd(), c->w[4] = (int)rnd();
  if (variant & 8) c->s[3] = (short)rnd();
  c->w[5] = flag;
  c->coefI = (short *)sysdep_malloc(0x62);
  c->coefQ = (short *)sysdep_malloc(0x62);
  c->freq = (short *)sysdep_malloc(2);
  ubuf_fill(c, variant);
  ubuf_poison(c->coefI, 1);
  ubuf_poison(c->coefQ, 1);
}
static void ubuf_free(V17RxCfg *c)
{
  ubuf_poison(c->coefI, 0);
  ubuf_poison(c->coefQ, 0);
  sysdep_free(c->coefI);
  sysdep_free(c->coefQ);
  sysdep_free(c->freq);
  c->coefI = c->coefQ = c->freq = NULL;
}
/* a created engine: the hdx.s20 flag is garbage in the blob, defined here on both sides */
static V17Rx *rx_make(V17Rx *sup, const V17RxCfg *cfg, int s20)
{
  V17Rx *o = V17RX_create(sup, cfg);
  if (o && o->hdx) o->hdx->s20 = (short)s20;
  return o;
}
static unsigned long long g_calls;
/* one receiver call on a copy of src[0..n): everything the call can change is hashed.  Returns the engine state dword; *outn = symbols returned */
static int rx_call(V17Rx *o, const short *src, int n, int *outn, int dump)
{
  unsigned short nn = (unsigned short)n;
  int ret, k;
  memcpy(g_in, src, (size_t)n * 2);
  for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
  g_in[n] = 0x5151; /* a sentinel behind the block */
  ret = V17RX_modem(o, g_in, g_out, &nn);
  emit(0, "rx call n=%d ret=%08x outn=%u in=%016llx out=%016llx sent=%d", n, u(ret), nn, fnv_n(g_in, n), fnv_n(g_out, 1024), g_in[n]);
  if (dump) d_rx("rx", o);
  *outn = nn;
  g_calls++;
  return ret;
}
static void observe(const V17Rx *o, int *prev)
{
  int s = o->hdx->state;
  if (s == 0) cov_hit(CV_S_ST0);
  if (s == 1) cov_hit(CV_S_ST1);
  if (s == 2) cov_hit(CV_S_ST2);
  if (s == 3) cov_hit(CV_S_ST3);
  if (s == 4) cov_hit(CV_S_ST4);
  if (s == 5) cov_hit(CV_S_ST5);
  if (s == 6) cov_hit(CV_S_ST6);
  if (s == 7) cov_hit(CV_S_ST7);
  if (*prev == 0 && s == 1) cov_hit(CV_S_TO1);
  if (*prev == 1 && s == 2) cov_hit(CV_S_TO2);
  if (*prev == 2 && s == 3) cov_hit(CV_S_TO3);
  if ((*prev == 3 || *prev == 2) && s == 4) cov_hit(CV_S_TO4);
  if (*prev == 4 && s == 5) cov_hit(CV_S_TO5);
  if (*prev == 5 && s == 6) cov_hit(CV_S_TO6);
  if (*prev == 6 && s == 5) cov_hit(CV_S_IDLE_BACK);
  if (s == 7 && *prev != 7) cov_hit(CV_S_ERR);
  if (o->st.b[0] == 4) cov_hit(CV_S_LOSS);
  if (o->st.b[0] == 5) cov_hit(CV_S_DROP2);
  if (o->st.b[0] == 9) cov_hit(CV_S_CODE9);
  if (o->st.b[0] == 8) cov_hit(CV_S_CODE8);
  if (o->st.b[0] == 7) cov_hit(CV_S_CODE7);
  if (o->st.b[0] == 6) cov_hit(CV_S_CODE6);
  if (o->bundle->qualityOk) cov_hit(CV_S_Q4E);
  if (o->bundle->qualBlocks == 0x33) cov_hit(CV_S_Q4A_FULL);
  if (o->bundle->fse.eqForce) cov_hit(CV_S_C40);
  if (o->bundle->fse.cfg.cb == FAX_FSE_decision_AB) cov_hit(CV_S_CB_AB);
  if (o->bundle->fse.cfg.cb == FSE_decision_eqtrn) cov_hit(CV_S_CB_EQ);
  if (o->bundle->fse.cfg.cb == FSE_Bridge_det) cov_hit(CV_S_CB_BRIDGE);
  if (o->bundle->fse.cfg.cb == FAX_FSE_decision_16pt || o->bundle->fse.cfg.cb == FAX_FSE_decision_32pt || o->bundle->fse.cfg.cb == FAX_FSE_decision_64pt ||
      o->bundle->fse.cfg.cb == FAX_FSE_decision_128pt)
    cov_hit(CV_S_CB_DEC);
  if (o->hdx->toneSearch) cov_hit(CV_S_S2E);
  if (o->st.b[1] & 0x80) cov_hit(CV_S_LOWSNR);
  if (*prev == 1 && s == 2 && o->hdx->flag) cov_hit(CV_S_RESTORE);
  if (*prev == 2 && s == 3) cov_hit(CV_S_STORE);
  *prev = s;
}

/* ============================================================================================ captured transmitter streams */
#define CAPFR 520
#define CAPMAX (CAPFR * 170)
static short g_cap[8][CAPMAX]; /* transmitter output: rate r, training t -> g_cap[r * 2 + t] (the transmitter of this side: blob or port) */
static int g_capn[8];
static void capture(int unused)
{
  int r, t;
  (void)unused;
  for (r = 0; r < 4; r++)
    for (t = 0; t < 2; t++) {
      Gen g;
      int i, n = 0, idx = r * 2 + t;
      gen_new(&g, r, t);
      for (i = 0; i < CAPFR; i++) {
        short fr[512];
        int k = gen_frame(&g, fr, 0);
        if (n + k > CAPMAX) die3("capture overflow");
        memcpy(g_cap[idx] + n, fr, (size_t)k * 2);
        n += k;
      }
      gen_free(&g);
      g_capn[idx] = n;
      emit(0, "capture rate %d short %d samples %d hash %016llx", r, t, n, fnv_n(g_cap[idx], n));
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
  emit(0, "TAB mrf %016llx sre coffs %016llx xb %016llx", fnv_shorts(MRFv17_COFFS, 360), fnv_shorts(SREv17_COFFS, 181), fnv_shorts(SREv17_XB_COFFS, 11));
  emit(0, "TAB clock x %016llx y %016llx", fnv_shorts(SREv17_xCLOCK, 3), fnv_shorts(SREv17_yCLOCK, 3));
  emit(0, "TAB sre pll1 %016llx pll1s %016llx pll2 %016llx", fnv_shorts(SREv17_PLL_K1, 3), fnv_shorts(SREv17_PLL_K1_S, 3), fnv_shorts(SREv17_PLL_K2, 3));
  emit(0, "TAB fse icoff %016llx qcoff %016llx", fnv_shorts(FSEv17_ICOFF, 49), fnv_shorts(FSEv17_QCOFF, 49));
  emit(0, "TAB crr clk %016llx pll1 %016llx pll1s %016llx pll2 %016llx", fnv_shorts(CRRv17_CLK, 4), fnv_shorts(CRRv17_PLL_K1, 3), fnv_shorts(CRRv17_PLL_K1_S, 3), fnv_shorts(CRRv17_PLL_K2, 3));
  emit(0, "TAB mtd %016llx", fnv_shorts(V17_MTD_COEFF, 10));
  emit(0, "TAB agc cfg %08x %08x %08x alpha %d,%d beta %d,%d w5 %08x", u(AGCv17_CFG[0]), u(AGCv17_CFG[1]), u(AGCv17_CFG[2]), ((const short *)cp(AGCv17_CFG[3]))[0],
       ((const short *)cp(AGCv17_CFG[3]))[1], ((const short *)cp(AGCv17_CFG[4]))[0], ((const short *)cp(AGCv17_CFG[4]))[1], u(AGCv17_CFG[5]));
  {
    char a[80], b[80], c[80], d[80], e[80], f[80], g[80], h[80];
    emit(0, "TAB ptr mtd=%s mrf=%s sre=%s xb=%s xc=%s yc=%s p1=%s p1s=%s", dptr(cp(CFGP(V17_MTD_COEFF)), a, 80), dptr(cp(CFGP(MRFv17_COFFS)), b, 80), dptr(cp(CFGP(SREv17_COFFS)), c, 80),
         dptr(cp(CFGP(SREv17_XB_COFFS)), d, 80), dptr(cp(CFGP(SREv17_xCLOCK)), e, 80), dptr(cp(CFGP(SREv17_yCLOCK)), f, 80), dptr(cp(CFGP(SREv17_PLL_K1)), g, 80),
         dptr(cp(CFGP(SREv17_PLL_K1_S)), h, 80));
    emit(0, "TAB ptr p2=%s ic=%s qc=%s clk=%s c1=%s c1s=%s c2=%s", dptr(cp(CFGP(SREv17_PLL_K2)), a, 80), dptr(cp(CFGP(FSEv17_ICOFF)), b, 80), dptr(cp(CFGP(FSEv17_QCOFF)), c, 80),
         dptr(cp(CFGP(CRRv17_CLK)), d, 80), dptr(cp(CFGP(CRRv17_PLL_K1)), e, 80), dptr(cp(CFGP(CRRv17_PLL_K1_S)), f, 80), dptr(cp(CFGP(CRRv17_PLL_K2)), g, 80));
    emit(0, "TAB ptr vtb i16=%s q16=%s i32=%s q32=%s i64=%s q64=%s i128=%s q128=%s", dptr(cp(CFGP(VTBv17_IMAP16T)), a, 80), dptr(cp(CFGP(VTBv17_QMAP16T)), b, 80),
         dptr(cp(CFGP(VTBv17_IMAP32)), c, 80), dptr(cp(CFGP(VTBv17_QMAP32)), d, 80), dptr(cp(CFGP(VTBv17_IMAP64)), e, 80), dptr(cp(CFGP(VTBv17_QMAP64)), f, 80),
         dptr(cp(CFGP(VTBv17_IMAP128)), g, 80), dptr(cp(CFGP(VTBv17_QMAP128)), h, 80));
  }
  emit(0, "TAB CFG %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x", u(V17RX_CFG[0]), u(V17RX_CFG[1]), u(V17RX_CFG[2]), u(V17RX_CFG[3]), u(V17RX_CFG[4]), u(V17RX_CFG[5]), u(V17RX_CFG[6]),
       u(V17RX_CFG[7]), u(V17RX_CFG[8]), u(V17RX_CFG[9]));
  emit(0, "TAB CTL %08x %08x %08x %08x %08x", u(V17RX_CTL[0]), u(V17RX_CTL[1]), u(V17RX_CTL[2]), u(V17RX_CTL[3]), u(V17RX_CTL[4]));
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
    FaxVmiV17 m;
    memset(&m, 0, sizeof m);
    for (i = -3; i < 14; i++) {
      const char *s = "unset";
      v17rx_message(&m, i, &s);
      if (i == 10) {
        /* the blob reads the first word of V17RX_CTL (0x38400000), the port stores NULL: one common line, the blob's value is checked */
        if (SIDE_BLOB && (unsigned)(uintptr_t)s != u(V17RX_CTL[0])) die3("blob v17rx_message(10) != V17RX_CTL[0]");
        if (!SIDE_BLOB && s != NULL) die3("port v17rx_message(10) != NULL");
        emit(1, "MSG %d = (CTL word 0 in the blob, NULL in the port)", i);
        cov_hit(CV_MSG_OOB);
      } else {
        emit(0, "MSG %d = %s", i, s ? s : "NULL");
        if (i >= 0 && i <= 9) cov_hit(CV_MSG); else cov_hit(CV_MSG_NEG);
      }
    }
    for (i = 0; i < 11; i++) {
      const char *s = 0;
      v17rx_message(&m, -256 + i, &s); /* a negative index whose low byte is a valid index: NULL (unsigned compare) */
      emit(0, "MSG %d = %s", -256 + i, s ? s : "NULL");
      s = 0;
      v17rx_message(&m, 0x100 + i, &s);
      emit(0, "MSG %d = %s", 0x100 + i, s ? s : "NULL");
    }
  }
}

/* ============================================================================================ scenario 1: lifecycle */
static int sgd_index(void)
{
  /* the allocation index of the SGD OBJECT inside the (fresh) create: the malloc after the Viterbi history (0x200 bytes) */
  V17RxCfg c;
  V17Rx *o;
  unsigned k;
  int idx = -1;
  make_rcfg(&c, 0, 0, 0);
  g_nmsz = 0;
  o = V17RX_create(NULL, &c);
  for (k = 0; k + 1 < g_nmsz && k < 256; k++)
    if (g_msz[k] == 0x200) {
      idx = (int)k + 1;
      break;
    }
  V17RX_delete(o);
  ubuf_free(&c);
  trial_end("sgd index");
  if (idx < 0) die3("no Viterbi history allocation");
  return idx;
}
static void probe_create(int failk, int rate, int flag, int sgdk)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    V17RxCfg c;
    make_rcfg(&c, rate, 0, flag);
    g_fail_malloc = failk;
    V17RX_create(NULL, &c);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  /* a failed allocation: the blob and the port crash or survive alike; the outcome is one line.  Exception: the SGD OBJECT inside SGD_create (R18): the blob stores through the NULL pointer (SIGSEGV),
   * the port's SGD_create returns NULL (a deliberate guard of src/fax/fax_fifo_sgd.c), so V17RX_create survives; the line is common, the sides are checked */
  if (failk == sgdk) {
    if (SIDE_BLOB != (WIFSIGNALED(st) != 0)) die3("probe: the blob must crash and the port survive at the SGD object");
    emit(1, "PROBE create malloc #%d fails (rate %d flag %d): (SGD_create guard: blob SIGSEGV, port survives)", failk, rate, flag);
    return;
  }
  if (WIFSIGNALED(st)) emit(1, "PROBE create malloc #%d fails (rate %d flag %d): signal %d", failk, rate, flag, WTERMSIG(st));
  else emit(1, "PROBE create malloc #%d fails (rate %d flag %d): exit %d", failk, rate, flag, WEXITSTATUS(st));
}
/* a caller-supplied object without hdx / bundle: the sub-objects are created with `fresh = 0` over the garbage of the new blocks; WHICH garbage word the init functions see depends on the host layout (the
 * blob: the recorder's sysdep_free refuses the pointer, exit 3; x86-64: a fault), so the common line only says that the create ran into the garbage */
static void probe_supplied(int mode)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    V17Rx *o;
    if (freopen("/dev/null", "w", stderr) == NULL) _exit(9); /* a garbage free() is a harness violation (exit 3) in the child: no noise */
    o = (V17Rx *)sysdep_malloc(sizeof(V17Rx));
    o->hdx = NULL;
    o->bundle = NULL;
    if (mode == 1) { /* hdx present (a real one), bundle missing */
      V17Rx *t = V17RX_create(NULL, NULL);
      o->hdx = t->hdx;
    }
    V17RX_create(o, NULL);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st) || WEXITSTATUS(st) == 3) emit(1, "PROBE create of a supplied object (mode %d) without bundle: ran into the garbage (signal or refused free)", mode);
  else emit(1, "PROBE create of a supplied object (mode %d) without bundle: exit %d", mode, WEXITSTATUS(st));
  cov_hit(CV_L_SUP_NOBUNDLE_PROBE);
}
/* the re-create over a live engine with another rate / training flag: buffers sized for the old rate are re-used / re-allocated (the red zone verdict is compared) */
static void probe_rate_change(int from, int to, int fflag, int tflag, int dbg)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    V17RxCfg c, c2;
    V17Rx *o;
    make_rcfg(&c, from, 0, fflag);
    o = rx_make(NULL, &c, 0);
    make_rcfg(&c2, to, 0, tflag);
    dbg_level((unsigned)dbg);
    g_verbose = 1; /* the child's own lines (allocator calls, debug text, the engine dump) are part of the output */
    V17RX_create(o, &c2);
    dbg_level(0);
    d_rx("changed", o);
    g_verbose = 0;
    fflush(stdout);
    V17RX_delete(o);
    _exit(redzones_dirty() ? 11 : 12);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE rate change %d -> %d flag %d -> %d (debug %d): signal %d", from, to, fflag, tflag, dbg, WTERMSIG(st));
  else emit(1, "PROBE rate change %d -> %d flag %d -> %d (debug %d): %s", from, to, fflag, tflag, dbg, WEXITSTATUS(st) == 11 ? "wrote outside a block" : WEXITSTATUS(st) == 12 ? "clean" : "other exit");
  cov_hit(CV_L_RATECHG);
}
/* garbage in the AGC state words that only a fresh init resets (rcp / shift / active): a live re-create must keep them (fresh = 0) */
static void poke_agcs(V17Rx *o)
{
  o->hdx->agc.rcp = 0x123;
  o->hdx->agc.shift = 7;
  o->hdx->agc.active = 9;
  o->bundle->agc.rcp = 0x321;
  o->bundle->agc.shift = -3;
  o->bundle->agc.active = 5;
}
static void run_some(V17Rx *o)
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
  int trial, k, sgdk;
  register_tables();
  sgdk = sgd_index();
  emit(1, "SGD object allocation index %d", sgdk);
  for (trial = 0; trial < 320; trial++) {
    static const int rates[10] = {0, 1, 2, 3, 0x1111, 0x3840, 3, 0, 2, 0x2580};
    int rate = rates[trial % 10];
    int mode = (trial / 10) % 6;
    int dbg = (trial / 60) & 1;
    int flagv = trial % 7 == 3 ? 1 : trial % 13 == 6 ? 5 : 0;
    int variant = (int)(rnd() & 63u);
    V17RxCfg cs[3];
    int ncs = 0;
    V17Rx *o = NULL;
    make_rcfg(&cs[ncs++], rate, trial % 3 == 2 ? variant : trial % 3, flagv);
    dbg_level(dbg ? (trial % 37 == 11 ? 0x80000002u : 2u) : 0u); /* (a level above 2 as an unsigned number, below 0 as a signed one) */
    if (rate == 0) cov_hit(CV_L_R7200); else if (rate == 1) cov_hit(CV_L_R9600); else if (rate == 2) cov_hit(CV_L_R12000); else if (rate == 3 || rate == 0x3840) cov_hit(CV_L_R14400);
    else cov_hit(CV_L_RBAD);
    if (dbg) cov_hit(CV_L_DBG);
    if (cs[0].user) cov_hit(CV_L_USER);
    if (flagv) cov_hit(CV_L_FLAG);
    switch (mode) {
      case 0:
        stack_poison();
        o = rx_make(NULL, &cs[0], trial & 1);
        cov_hit(CV_L_NEW);
        break;
      case 1:
        stack_poison();
        o = rx_make(NULL, NULL, 0);
        cov_hit(CV_L_NEW);
        cov_hit(CV_L_NULLCFG);
        break;
      case 2: { /* a live engine re-created with a config of the SAME rate */
        V17Rx *s;
        make_rcfg(&cs[ncs++], rate == 1 ? 1 : 0, 0, flagv);
        s = rx_make(NULL, &cs[1], 0);
        d_rx("first", s);
        run_some(s);
        poke_agcs(s);
        stack_poison();
        make_rcfg(&cs[ncs++], rate == 1 ? 1 : 0, (int)(rnd() & 63u), flagv);
        o = V17RX_create(s, &cs[2]);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_SUP_LIVE);
        cov_hit(CV_L_SAMERATE);
        if (flagv) cov_hit(CV_L_FLAGLIVE);
        break;
      }
      case 3: { /* hdx dropped: the create allocates a new one (the old pieces are freed here) */
        V17Rx *s;
        make_rcfg(&cs[ncs++], rate == 1 ? 1 : 0, 0, flagv);
        s = rx_make(NULL, &cs[1], 0);
        d_rx("first", s);
        FPM_MTD_delete(s->hdx->mtd);
        FPM_TONE_delete(s->hdx->tone);
        FPM_MTD_delete(s->hdx->mtd2);
        sysdep_free(s->hdx->buf);
        sysdep_free(s->hdx->buf2);
        sysdep_free(s->hdx);
        s->hdx = NULL;
        stack_poison();
        o = rx_make(s, &cs[1], 0);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_SUP_NOHDX);
        break;
      }
      case 4: { /* create(o, &o->cfg): the object is its own config (control path) */
        V17Rx *s;
        make_rcfg(&cs[ncs++], rate == 1 ? 1 : 0, variant & ~1, flagv);
        s = rx_make(NULL, &cs[1], 0);
        d_rx("first", s);
        s->cfg.w[2] = 777;
        o = V17RX_create(s, &s->cfg);
        cov_hit(CV_L_RECREATE);
        break;
      }
      default: { /* a re-create that changes the rate, the training flag and the user word */
        V17Rx *s;
        make_rcfg(&cs[ncs++], 1, 0, flagv ? 0 : 1);
        s = rx_make(NULL, &cs[1], 0);
        d_rx("first", s);
        poke_agcs(s);
        make_rcfg(&cs[ncs++], rate, 3, flagv);
        o = V17RX_create(s, &cs[2]);
        cov_hit(CV_L_SUP_LIVE);
        cov_hit(CV_L_FLAGCHG);
        break;
      }
    }
    dbg_level(0);
    d_rx("created", o);
    if (trial & 1) run_some(o);
    V17RX_delete(o);
    for (k = 0; k < ncs; k++) ubuf_free(&cs[k]);
    cov_hit(CV_L_DEL);
    trial_end("life");
  }
  probe_supplied(0);
  for (k = 0; k < 52; k++) {
    probe_create(k, k % 4, (k / 4) & 1, sgdk);
    cov_hit(CV_L_PROBE);
  }
  probe_rate_change(0, 1, 0, 0, 0);
  probe_rate_change(1, 0, 0, 0, 0);
  probe_rate_change(3, 2, 0, 0, 0);
  probe_rate_change(2, 3, 0, 0, 0);
  probe_rate_change(0, 0, 0, 0, 0);
  probe_rate_change(3, 3, 0, 0, 0);
  probe_rate_change(1, 1, 1, 0, 0);
  probe_rate_change(1, 1, 0, 1, 0);
  probe_rate_change(2, 3, 1, 1, 0);
  probe_rate_change(0, 1, 0, 0, 2);
  probe_rate_change(3, 0, 1, 0, 2);
  probe_rate_change(0, 3, 0, 1, 2);
}

/* ============================================================================================ scenario 2: VMI wrappers */
/* kind 0: right after the initialiser (u4 = the integer 1, u10 = the config block); kind 1: after v17rx_create (u4 = the bit buffer, u10 = the small flag short: the blob stores a short over the low half of the
 * block pointer, the block is leaked) */
static void hblk_id(char *buf, size_t cap, const void *p) /* the config block holds heap addresses and is host sized: only its id */
{
  Blk *b = p ? find_blk(p) : NULL;
  if (b) snprintf(buf, cap, "blk%u", b->id);
  else snprintf(buf, cap, p ? "?" : "NULL");
}
static void d_vmi(const char *t, const FaxVmiV17 *m, int kind)
{
  char p1[80], p2[80];
  emit(0, "%s VMI w0=%08x u4=%s s08=%d s0a=%d s0c=%d s0e=%d u10=%s eng=%s", t, u(m->w0), kind == 0 ? (m->u4.init1 == 1 ? "1" : "other") : (hblk(p1, 80, m->u4.bits), p1), m->s08, m->s0a, m->s0c, m->s0e,
       kind == 0 ? (hblk_id(p2, 80, m->u10.cfg), p2) : (m->u10.s10 == 3 ? "3" : m->u10.s10 == 4 ? "4" : m->u10.s10 == 5 ? "5" : m->u10.s10 == 6 ? "6" : "other"), m->eng ? "eng" : "NULL");
}
static void scen2(void)
{
  int trial, k;
  register_tables();
  capture(0);
  for (trial = 0; trial < 80; trial++) {
    static const int rate_words[5] = {0x1c20, 0x2580, 0x2ee0, 0x3840, 0x1111};
    FaxVmiV17 m;
    int rate = rate_words[trial % 5];
    unsigned dbg = trial % 29 == 17 ? 0x80000003u : (unsigned)(trial % 4);
    int ret;
    V17RxCfg *cfgblk;
    short *bits = (short *)sysdep_malloc(1024 * 2);
    unsigned char *raw = (unsigned char *)&m;
    for (k = 0; k < (int)sizeof m; k++) raw[k] = (unsigned char)(0xa0u + (unsigned)k * 7u + (unsigned)trial);
    dbg_level((unsigned)dbg);
    if (dbg >= 3) cov_hit(CV_V_INIT_DBG);
    stack_poison();
    ret = fax_init_vmi_v17rx(&m, (unsigned short)rate, 0x55, (void *)0);
    dbg_level(0);
    emit(0, "vmi init ret=%d", ret);
    d_vmi("init", &m, 0);
    cfgblk = (V17RxCfg *)m.u10.cfg;
    {
      char p1[80], p2[80], p3[80];
      emit(0, "vmi init cfg %08x %08x %08x %08x %08x %08x coefI=%s coefQ=%s freq=%s user=%08x", u(cfgblk->w[0]), u(cfgblk->w[1]), u(cfgblk->w[2]), u(cfgblk->w[3]), u(cfgblk->w[4]), u(cfgblk->w[5]),
           (hblk(p1, 80, cfgblk->coefI), p1), (hblk(p2, 80, cfgblk->coefQ), p2), (hblk(p3, 80, cfgblk->freq), p3), u(cfgblk->user));
    }
    cov_hit(CV_V_INIT);
    if (rate == 0x1c20) cov_hit(CV_V_R7200); else if (rate == 0x2580) cov_hit(CV_V_R9600); else if (rate == 0x2ee0) cov_hit(CV_V_R12000); else if (rate == 0x3840) cov_hit(CV_V_R14400);
    else cov_hit(CV_V_RBAD);
    { /* the VMI object keeps the user word as the engine handle: re-init with a live engine below */
      const V17RxCfg *cfg = trial % 3 == 1 ? NULL : cfgblk;
      int e;
      V17Rx *sup = NULL;
      V17RxCfg sc;
      int have_sc = 0;
      if (trial % 4 == 3 && cfg) {
        int j;
        cfgblk->w[5] = 1; /* the short training with the garbage taps of the initialiser's heap blocks (the filter taps start from them) */
        for (j = 0; j < 0x31; j++) cfgblk->coefI[j] = (short)(FSEv17_ICOFF[j] / 2), cfgblk->coefQ[j] = (short)(FSEv17_QCOFF[j] / 2);
        cfgblk->freq[0] = 11;
      }
      if (trial % 3 == 2) {
        make_rcfg(&sc, trial & 3, 0, 0);
        have_sc = 1;
        sup = rx_make(NULL, &sc, 0);
        m.eng = (struct V17Tx *)sup;
        cov_hit(CV_V_CREATE_SUP);
      } else {
        m.eng = NULL;
      }
      m.u4.bits = bits;
      m.s0c = (short)rnd(); /* v17rx_create sets these: garbage in front */
      m.s0e = (short)rnd();
      stack_poison();
      e = v17rx_create(&m, cfg);
      emit(0, "vmi create ret=%d", e);
      if (cfg == NULL) cov_hit(CV_V_CREATE_NULLCFG);
      else if (rate == 0x2ee0 || rate == 0x3840) {
        if (e == cfgblk->w[1]) cov_hit(CV_V_RET_CFG1);
      }
      cov_hit(CV_V_CREATE);
      d_vmi("create", &m, 1);
      ((V17Rx *)m.eng)->hdx->s20 = 0;
      d_rx("vmi", (V17Rx *)m.eng);
      {
        int rr = rate == 0x1c20 ? 0 : rate == 0x2580 ? 1 : rate == 0x2ee0 ? 2 : 3;
        int calls = trial % 3 == 0 ? 75 : 6;
        for (k = 0; k < calls; k++) {
          unsigned short n = (unsigned short)(trial % 3 == 0 ? 160 : 1 + rmod(160)), outn = 0x5555;
          int i, r;
          static short ibuf[4096];
          if (trial % 3 == 0) {
            for (i = 0; i < 160; i++) ibuf[i] = g_cap[rr * 2][(k * 160 + i) % g_capn[rr * 2]];
          } else {
            for (i = 0; i < 4096; i++) ibuf[i] = (short)((int)(rnd() & 0x3fffu) - 0x2000);
          }
          for (i = 0; i < 1024; i++) bits[i] = 0x4242;
          r = v17rx_process(&m, ibuf, &outn, &n);
          emit(0, "vmi process n=%u outn=%u ret=%08x in=%016llx bits=%016llx", n, outn, u(r), fnv_n(ibuf, 160), fnv_n(bits, 1024));
          cov_hit(CV_V_PROC);
        }
      }
      {
        FaxV17Ctl ctl;
        FaxV17Status sx;
        memset(&ctl, 0, sizeof ctl);
        ctl.w4 = (int)rnd();
        ctl.b0c = (unsigned char)(trial & 0x28);
        ctl.b0d = (unsigned char)((trial & 4 ? 0x10 : 0) | (trial & 8 ? 2 : 0));
        ctl.w10 = (trial & 16 && ((V17Rx *)m.eng)->cfg.coefI) ? 1 : 0; /* the flag needs the caller buffers (the blob dereferences them unchecked) */
        memset(&sx, 0x55, sizeof sx);
        emit(0, "vmi status %d", v17rx_status(&m, &sx));
        prt_status("vmi status", &sx);
        emit(0, "vmi status null %d", v17rx_status(&m, NULL));
        emit(0, "vmi control %d", v17rx_control(&m, &ctl));
        emit(0, "vmi control null %d", v17rx_control(&m, NULL));
        cov_hit(CV_V_STATUS);
        cov_hit(CV_V_CTL);
        d_vmi("ctl", &m, 1);
        ((V17Rx *)m.eng)->hdx->s20 = 0;
        d_rx("vmi ctl", (V17Rx *)m.eng);
      }
      for (k = -2; k < 13; k++) {
        const char *s = "unset";
        v17rx_message(&m, k, &s);
        if (k == 10) emit(1, "vmi msg %d = (CTL word in the blob, NULL in the port)", k);
        else emit(0, "vmi msg %d = %s", k, s ? s : "NULL");
        cov_hit(CV_V_MSG);
      }
      v17rx_delete(&m);
      if (have_sc) ubuf_free(&sc);
    }
    cov_hit(CV_V_DEL);
    sysdep_free(cfgblk->coefI);
    sysdep_free(cfgblk->coefQ);
    sysdep_free(cfgblk->freq);
    sysdep_free(cfgblk);
    sysdep_free(bits);
    trial_end("vmi");
  }
}

/* ============================================================================================ scenario 3: receive streams */
static short g_sbuf[16384];
static void rx_control_random(V17Rx *o, int i, int pathb)
{
  FaxV17Ctl ctl;
  memset(&ctl, 0, sizeof ctl);
  ctl.w4 = (int)rnd();
  ctl.b0c = (unsigned char)((rpct(30) ? 8 : 0) | (rpct(30) ? 0x20 : 0));
  ctl.b0d = (unsigned char)((rpct(i > 300 ? 40 : 10) ? 0x10 : 0) | (rpct(5) ? 2 : 0));
  ctl.w10 = rpct(75) ? o->cfg.w[5] : (int)(rnd() & 1u);
  cov_hit(CV_S_CTL);
  if (ctl.b0d & 0x10) cov_hit(CV_S_HOLD);
  if (ctl.b0d & 2) {
    cov_hit(CV_S_RECREATE);
    if ((ctl.w10 != 0) != (o->cfg.w[5] != 0)) cov_hit(CV_S_FLAGCTL);
  }
  emit(0, "rx control ret=%d b0c=%02x b0d=%02x w10=%d", V17RX_control(o, &ctl), ctl.b0c, ctl.b0d, ctl.w10);
  if (o->hdx) o->hdx->s20 = (short)pathb; /* the flag survives a re-create of a live hdx; re-applied for clarity */
  d_rx("ctl", o);
}
static void scen3(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 64; trial++) {
    int rxr = (trial % 13 == 6) ? 0x1111 : (trial & 3); /* receiver rate word: 0 .. 3 / invalid */
    int txr = rxr > 3 ? 3 : rxr;
    int tshort = trial % 5 == 1 || trial % 11 == 5;     /* the transmitter's training: short (cfg word 6) */
    int rflag = trial % 5 == 1 ? 1 : trial % 11 == 5 ? 1 : trial % 17 == 3 ? -1 : trial % 19 == 4 ? 2 : 0; /* the receiver's: short (the caller's taps); any nonzero word */
    int variant = ((trial / 4) & 3) | (trial % 7 == 3 ? 16 : 0) | (trial % 9 == 4 ? 32 : 0);
    int prof = (trial / 8) % 9;
    int pol = (trial / 3) % 6;
    int pathb = (trial % 5 == 3) ? 1 : 0;
    int dbg = trial % 9 == 4;
    int i, calls, prev = -1;
    Imp imp = {256, 0, 0, 0};
    Gen g;
    V17RxCfg c;
    V17Rx *o;
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
    make_rcfg(&c, rxr, variant, rflag);
    if (rxr == 0) cov_hit(CV_S_R7200); else if (rxr == 1) cov_hit(CV_S_R9600); else if (rxr == 2) cov_hit(CV_S_R12000); else if (rxr == 3) cov_hit(CV_S_R14400); else cov_hit(CV_S_BADRATE);
    if (variant & 1) cov_hit(CV_S_USER);
    if (rflag) cov_hit(CV_S_SHORT); else cov_hit(CV_S_LONG);
    if (pathb) cov_hit(CV_S_PATHB);
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_S_DBG);
    stack_poison();
    gen_new(&g, txr, tshort);
    o = rx_make(NULL, &c, pathb);
    d_rx("created", o);
    imp_phase = 0;
    calls = (rflag ? 160 : 400) + (int)rmod(120);
    for (i = 0; i < calls; i++) {
      short fr[512];
      int n, k, take, outn, clip = 0, pst;
      int sym_mode = (i > 330 && rpct(2)) ? 1 : 0;
      n = gen_frame(&g, fr, sym_mode);
      if (lossleft > 0) {
        lossleft--;
        for (k = 0; k < n; k++) fr[k] = 0;
        cov_hit(CV_S_LOSSSEG);
      } else if (rpct(2) && i > 20 && trial >= 8) { /* the first trials are clean: lock, quality counters */
        lossleft = 1 + (int)rmod(24);
      }
      if (burstleft > 0) {
        burstleft--;
        for (k = 0; k < n; k++) fr[k] = (short)((int)(rnd() & 0xffffu) - 0x8000);
        cov_hit(CV_S_BURST);
      } else if (rpct(1) && i > 30 && trial >= 8) {
        burstleft = 1 + (int)rmod(5);
      }
      if (gainleft > 0) {
        gainleft--;
        for (k = 0; k < n; k++) fr[k] = (short)(fr[k] / 5);
        cov_hit(CV_S_ENERGY);
      } else if (rpct(2) && i > 30 && trial >= 8) {
        gainleft = 3 + (int)rmod(20);
      }
      if (toneleft > 0) {
        toneleft--;
        tone_fill(fr, n, 2100, 4000 + (int)rmod(8000));
        cov_hit(CV_S_TONESEG);
      } else if (rpct(2) && i > 40 && trial >= 8) {
        toneleft = 3 + (int)rmod(30);
      } else if (pathb && o->hdx->state == 5 && rpct(6)) {
        toneleft = 12 + (int)rmod(14); /* long enough for the V.21 carrier timeout of the path B detector (0x4ff samples) */
      }
      if (rpct(1) && i > 20 && trial >= 8) { /* the transmitter re-trains */
        gen_free(&g);
        gen_new(&g, txr, tshort);
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
        if (pst >= 2 && pst <= 4 && outn > 0) cov_hit(CV_S_PROTO);
        if (o->hdx->state == 5 && (o->st.b[1] & 0x80) == 0 && outn > 0) cov_hit(CV_S_DATAOK);
        oh ^= fnv_n(g_out, outn);
        observe(o, &prev);
        if (take > 0) {
          memmove(g_sbuf, g_sbuf + take, (size_t)(sn - take) * 2);
          sn -= take;
        } else if (rpct(60)) {
          break;
        }
      }
      if (o->hdx->state == 5 && rpct(3)) { /* the caller (the T.30 layer) ends the data state: RxNextStateV17 is a GLOBAL entry of the blob */
        RxNextStateV17(o);
        emit(0, "caller next state %d", o->hdx->state);
        d_rx("next", o);
        observe(o, &prev);
      } else if (o->hdx->state == 6 && rpct(8)) {
        RxNextStateV17(o);
        emit(0, "caller next state %d", o->hdx->state);
        d_rx("next", o);
        observe(o, &prev);
      }
      if (o->hdx->state == 7 && rpct(10)) { /* the error state is final: only a control re-create restarts the receiver */
        FaxV17Ctl ctl;
        memset(&ctl, 0, sizeof ctl);
        ctl.w4 = o->cfg.w[2];
        ctl.b0d = 2;
        ctl.w10 = o->cfg.w[5];
        emit(0, "rx restart ret=%d", V17RX_control(o, &ctl));
        o->hdx->s20 = (short)pathb;
        d_rx("restart", o);
        prev = -1;
        cov_hit(CV_S_RECREATE);
      }
      if (rpct(3)) {
        FaxV17Status stx[2];
        memset(stx, 0x55, sizeof stx);
        emit(0, "rx status ret=%d null=%d", V17RX_status(o, &stx[0]), V17RX_status(o, NULL));
        prt_status("rx status", &stx[0]);
        cov_hit(CV_S_STATUS);
      }
      if (rpct(3)) rx_control_random(o, i, pathb);
    }
    emit(0, "rx trial %d rxrate %d tshort %d rflag %d variant %d prof %d pol %d: symbols=%d outhash=%016llx", trial, rxr, tshort, rflag, variant, prof, pol, tot, oh);
    if (trial % 8 == 0) emit(1, "RX trial %d rate %d: %d symbols", trial, rxr, tot);
    dbg_level(0);
    V17RX_delete(o);
    gen_free(&g);
    ubuf_free(&c);
    trial_end("rx run");
  }
}

/* ============================================================================================ scenario 4: handlers / detectors */
typedef struct {
  Gen g;
  V17Rx *o;
  V17RxCfg c;
  int rate, frames;
} Sess;
static void sess_new(Sess *s, int rate, int variant, int warm, int pathb, int flag)
{
  int i, outn, prev = -1;
  make_rcfg(&s->c, rate, variant, flag);
  gen_new(&s->g, rate > 3 ? 3 : rate, flag);
  s->o = rx_make(NULL, &s->c, pathb);
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
  V17RX_delete(s->o);
  gen_free(&s->g);
  ubuf_free(&s->c);
}
/* a block of input: the next frame of the session's transmitter, noise, silence, a tone or a captured stream */
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
    int r = (s->rate > 3 ? 3 : s->rate) * 2, off = (int)rmod((unsigned)(g_capn[r] - 200));
    for (k = 0; k < want; k++) dst[k] = g_cap[r][off + k];
  }
  return want;
}
static void agc_reset(V17Rx *o)
{
  /* RxNextStateV17 state 1 advances the AGC table pointers by one entry (two entries per table): a second advance would read past the table (layout dependent), so direct calls start at entry 0 */
  o->bundle->agc.alpha = (const short *)cp(AGCv17_CFG[3]);
  o->bundle->agc.beta = (const short *)cp(AGCv17_CFG[4]);
}
static void poke_rx(V17Rx *o)
{
  static const int cnts[8] = {0x3e5, 0x3e6, 0x3e7, 0x3e8, 0x7ffe, 0x7fff, -1, 100};
  static const int e52s[12] = {0, 100, 0x1fff, 0x2000, 0x3fff, 0x4000, -5, 0x7fff, 4, 5, 6, 7};
  static const int q4as[8] = {0, 1, 0x30, 0x31, 0x32, 0x33, -1, 7};
  V17RxBundle *b = o->bundle;
  V17RxHdx *h = o->hdx;
  if (rpct(30)) h->hold = (int)(rnd() & 1u);
  if (rpct(25)) h->s20 = (short)(rnd() & 1u);
  if (rpct(25)) h->toneSearch = (short)(rnd() & 1u);
  if (rpct(30)) h->sinceTone = (short)(rpct(50) ? (int)rmod(0x500) : 0x4f0 + (int)rmod(24));
  if (rpct(25)) { static const int fl[8] = {0, 1, -1, 2, -2, 0x10000, 5, -0x8000}; h->flag = fl[rnd() % 8u]; }
  if (rpct(30)) b->dec.ctr = (unsigned short)cnts[rnd() & 7u];
  if (rpct(30)) b->fse.decErrAvg = (short)e52s[rnd() % 12u];
  if (rpct(25)) b->fse.eqForce = (int)(rnd() & 1u);
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
  if (rpct(20)) b->w1c = (int)rnd();
  if (rpct(20)) b->w0 = (int)rnd();
  if (rpct(15)) b->eqGate = (int)rnd();
  if (rpct(50)) o->st.w = (int)rnd();
  if (rpct(15)) { /* the caller's buffers hold other taps (Restore / StoreCoef / the FSE start taps read them) */
    int k;
    for (k = 0; k < 0x31; k++) o->cfg.coefI[k] = (short)rnd(), o->cfg.coefQ[k] = (short)rnd();
    o->cfg.freq[0] = (short)rnd();
  }
}
static __attribute__((unused)) V17RxHdxFn pick_fn(int sel)
{
  switch (sel) {
    case 0: return RxHdxStartV17;
    case 1: return RxHdxEpochDetV17;
    case 2: return RxHdxPrtcolV17;
    case 3: return RxHdxBridgeV17;
    case 4: return RxHdxScramV17;
    case 5: return RxHdxDataV17;
    case 6: return RxHdxIdleV17;
    default: return RxHdxErrorV17;
  }
}

extern short FPM_rms(const short *data, short count);
/* DataCarrierDetectV17 / CarrierDetectV17: the combinations of every input they read (both paths, the training flag, c40, the call counter around 0x3e7, the thresholds 0x3fff / 0x4ff, the energy-drop
 * boundary, the q54 counter) */
static void dcd_sweep(void)
{
  Sess s;
  static short blk[512];
  static const int cnts[6] = {0x3e6, 0x3e7, 0x3e8, 0, -1, 0x7fff};
  static const int e52s[5] = {-1, 0x3ffe, 0x3fff, 0x4000, 0x4001};
  static const int dets[4] = {0, 1, 2, 3};
  static const int w40s[4] = {0, 1, 2, -1};
  int path, fl, c40, ci, ei, di, wi, s2e, q50, k, n = 40, kind, dd, ds, qq, q54;
  sess_new(&s, 1, 0, 70, 0, 0);
  agc_reset(s.o);
  /* CarrierDetectV17 alone: the gate words, the flag, c40, the counter and the error */
  for (fl = 0; fl < 2; fl++)
    for (c40 = 0; c40 < 2; c40++)
      for (ci = 0; ci < 6; ci++)
        for (ei = 0; ei < 5; ei++)
          for (di = 0; di < 4; di++)
            for (wi = 0; wi < 4; wi++) {
              V17Rx *o = s.o;
              int ret;
              o->hdx->flag = fl;
              o->bundle->fse.eqForce = c40;
              o->bundle->dec.ctr = (unsigned short)cnts[ci];
              o->bundle->fse.decErrAvg = (short)e52s[ei];
              o->bundle->agc.detect = dets[di];
              o->bundle->sre.pllEnable = w40s[wi];
              dbg_level(((ci + ei) & 1) ? 2 : 0);
              ret = CarrierDetectV17(o);
              dbg_level(0);
              emit(0, "cd fl%d c%d ci%d e%d d%d w%d: ret=%d", fl, c40, ci, ei, di, wi, ret);
              if (fl && c40 && (short)cnts[ci] > 0x3e7 && e52s[ei] > 0x3fff) cov_hit(CV_H_DECERR_NC);
              if (fl && c40 && (short)cnts[ci] > 0x3e7 && e52s[ei] <= 0x3fff) cov_hit(CV_H_DECERR_PLAIN);
            }
  for (kind = 0; kind < 5; kind++) {
    if (kind >= 3) tone_fill(blk, n, kind == 3 ? 2100 : 1500, 5000);
    else for (k = 0; k < n; k++) blk[k] = kind == 0 ? g_cap[3][500 + k] : kind == 1 ? (short)((int)(rnd() & 0x3fffu) - 0x2000) : (short)(g_cap[3][700 + k] / 7);
    for (path = 0; path < 2; path++)
      for (fl = 0; fl < 2; fl++)
        for (c40 = 0; c40 < 2; c40++)
          for (ci = 0; ci < 6; ci += (path ? 5 : 1))
            for (ei = 0; ei < 5; ei++)
              for (di = 0; di < 4; di++)
                for (wi = 0; wi < 4; wi++)
                  for (s2e = 0; s2e < 2; s2e++)
                    for (q50 = 0; q50 < 2; q50++) {
                      V17Rx *o = s.o;
                      int ret;
                      if (path == 0 && s2e) continue;
                      o->hdx->s20 = (short)path;
                      o->hdx->toneSearch = (short)s2e;
                      o->hdx->sinceTone = 0;
                      o->hdx->flag = fl;
                      o->bundle->fse.eqForce = c40;
                      o->bundle->dec.ctr = (unsigned short)cnts[ci];
                      o->bundle->fse.decErrAvg = (short)e52s[ei];
                      o->bundle->agc.detect = dets[di];
                      o->bundle->sre.pllEnable = w40s[wi];
                      o->bundle->energyTest = (short)q50;
                      o->bundle->refLevel = 0;
                      o->bundle->refBlocks = 0;
                      dbg_level((ci & 1) ? 2 : 0);
                      ret = DataCarrierDetectV17(o, blk, (unsigned short)n);
                      dbg_level(0);
                      emit(0, "dcd k%d p%d f%d c40_%d c%d e%d d%d w%d s%d q%d: ret=%d s2e=%d s2c=%d q54=%d q52=%d", kind, path, fl, c40, ci, ei, di, wi, s2e, q50, ret, o->hdx->toneSearch, o->hdx->sinceTone,
                           o->bundle->refBlocks, o->bundle->refLevel);
                      cov_hit(path ? CV_H_PATHB : CV_H_PATHA);
                      if (!path && fl && c40 && (short)cnts[ci] > 0x3e7 && e52s[ei] > 0x3fff) cov_hit(CV_H_DECERR_NC);
                      if (!path && fl && c40 && (short)cnts[ci] > 0x3e7 && e52s[ei] <= 0x3fff) cov_hit(CV_H_DECERR_PLAIN);
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
              V17Rx *o = s.o;
              int ret;
              o->hdx->s20 = (short)path;
              o->hdx->toneSearch = 0;
              o->hdx->flag = 0;
              o->bundle->fse.decErrAvg = 0;
              o->bundle->agc.detect = 1;
              o->bundle->sre.pllEnable = 1;
              o->bundle->energyTest = 1;
              o->bundle->refLevel = (short)qq;
              o->bundle->refBlocks = (short)q54;
              ret = DataCarrierDetectV17(o, blk, (unsigned short)n);
              emit(0, "dcd energy k%d p%d r=%d q52=%d thr=%d q54=%d: ret=%d q52'=%d q54'=%d", kind, path, r, qq, thr, q54, ret, o->bundle->refLevel, o->bundle->refBlocks);
            }
          }
        }
    }
    /* a negative / huge reference level: the threshold (q52 * 0x32fe) >> 15 is signed */
    for (qq = 0; qq < 5; qq++) {
      static const int q52s[5] = {-1, -100, -32768, 0x7fff, 0x4000};
      V17Rx *o = s.o;
      int ret;
      o->hdx->s20 = 0;
      o->hdx->toneSearch = 0;
      o->hdx->flag = 0;
      o->bundle->fse.decErrAvg = 0;
      o->bundle->agc.detect = 1;
      o->bundle->sre.pllEnable = 1;
      o->bundle->energyTest = 1;
      o->bundle->refLevel = (short)q52s[qq];
      o->bundle->refBlocks = 0;
      ret = DataCarrierDetectV17(o, blk, (unsigned short)n);
      emit(0, "dcd q52 k%d q52=%d: ret=%d q52'=%d q54'=%d", kind, q52s[qq], ret, o->bundle->refLevel, o->bundle->refBlocks);
    }
    /* the carrier-timeout counter s2c around 0x4ff (path B, s2e set) */
    for (dd = -6; dd <= 6; dd++)
      for (q50 = 0; q50 < 2; q50++) {
        V17Rx *o = s.o;
        int ret;
        o->hdx->s20 = 1;
        o->hdx->toneSearch = 1;
        o->hdx->sinceTone = (short)(dd <= -4 ? -n - 100 + dd : 0x4ff - n + dd);
        o->hdx->flag = 0;
        o->bundle->fse.decErrAvg = 0;
        o->bundle->agc.detect = 1;
        o->bundle->sre.pllEnable = 1;
        o->bundle->energyTest = (short)(q50 ? 1 : 0);
        o->bundle->refLevel = 0;
        dbg_level(q50 ? 2 : 0);
        ret = DataCarrierDetectV17(o, blk, (unsigned short)n);
        dbg_level(0);
        emit(0, "dcd s2c k%d dd=%d q50=%d: ret=%d s2c=%d", kind, dd, q50, ret, o->hdx->sinceTone);
      }
  }
  sess_free(&s);
  trial_end("dcd sweep");
}
/* RxHdxIdleV17: the carrier flag and the decision error at the boundary 0x1fff / 0x2000 (a block too short to complete a symbol keeps the error unchanged) */
static void idle_sweep(void)
{
  Sess s;
  static short blk[512];
  int e, st, kind, k, n = 40;
  sess_new(&s, 0, 0, 70, 0, 0);
  for (kind = 0; kind < 2; kind++)
    for (e = 0; e < 12; e++)
      for (st = 5; st <= 6; st++) {
        static const int ees[12] = {0x1ffd, 0x1ffe, 0x1fff, 0x2000, 0x2001, 0x2002, -1, -5, -32768, 0x7fff, 0, 0x1000};
        int ee = ees[e];
        V17Rx *o = s.o;
        unsigned short nn = (unsigned short)n;
        int ret;
        for (k = 0; k < n; k++) blk[k] = kind == 0 ? g_cap[0][500 + k] : (short)(g_cap[0][500 + k] * 3);
        agc_reset(o);
        o->hdx->state = (short)st;
        o->hdx->fn = RxHdxIdleV17;
        o->bundle->fse.decErrAvg = (short)ee;
        o->bundle->fse.untilSym = 30000;
        o->bundle->sre.pllEnable = 1;
        o->bundle->sreGate = 1;
        o->hdx->flag = 0;
        o->st.w = 0x7000 + st;
        dbg_level(ee & 1 ? 2 : 0);
        ret = RxHdxIdleV17(o, blk, g_out, &nn);
        dbg_level(0);
        emit(0, "idle sweep e52=%x st=%d kind=%d: ret=%d nn=%u state=%d b=%08x e52'=%d", ee, st, kind, ret, nn, o->hdx->state, u(o->st.w), o->bundle->fse.decErrAvg);
        d_rx("idle", o);
      }
  sess_free(&s);
  trial_end("idle sweep");
}
/* RxHdxPrtcolV17 / Bridge / Scram: the end of the state (count <= 0) sets the SNR flag from GetSNRV17 = 13 - e52 at its boundary (e52 = 4 .. 6) and (Scram) the connect code of the rate; a block too short to
 * complete a symbol keeps e52 */
static void prtcol_sweep(void)
{
  Sess s;
  static short blk[512];
  int e, c, kind, k, n = 40, which;
  for (kind = 0; kind < 4; kind++) {
    sess_new(&s, kind, 0, 70, 0, 0);
    for (which = 0; which < 3; which++)
      for (e = 2; e <= 10; e++)
        for (c = 0; c <= 3; c++) {
          V17Rx *o = s.o;
          unsigned short nn = (unsigned short)n;
          int ret;
          for (k = 0; k < n; k++) blk[k] = g_cap[kind * 2][500 + k];
          agc_reset(o);
          o->hdx->state = (short)(2 + which);
          o->hdx->fn = pick_fn(2 + which);
          o->hdx->count = (short)c;
          o->bundle->fse.decErrAvg = (short)e;
          o->bundle->fse.untilSym = 30000;
          o->bundle->sre.pllEnable = 1;
          o->bundle->sreGate = 1;
          o->hdx->flag = which == 0 ? (e & 1) : 0;
          o->st.w = 0x3000 + c;
          ret = pick_fn(2 + which)(o, blk, g_out, &nn);
          emit(0, "prtcol sweep rate %d which %d e52=%d count=%d: ret=%d nn=%u state=%d b=%08x e52'=%d", kind, which, e, c, ret, nn, o->hdx->state, u(o->st.w), o->bundle->fse.decErrAvg);
          if (c == 1) cov_hit(CV_H_PRTCOL_END);
          d_rx("prtcol", o);
        }
    sess_free(&s);
  }
  trial_end("prtcol sweep");
}
/* QualityDetectV17: every q4a stage x the decoder error at / around the threshold x the carrier flag */
static void quality_sweep(void)
{
  static const int q4as[8] = {0, 1, 0x30, 0x31, 0x32, 0x33, 0x34, -1};
  static const int q4cs[4] = {0xa28, -1, 0x7fff, -32768};
  int qa, qi, d, det, w40, r, e;
  V17Rx *o;
  V17RxCfg c;
  make_rcfg(&c, 0, 0, 0);
  o = rx_make(NULL, &c, 0);
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
        r = QualityDetectV17(o);
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
            r = QualityDetectV17(o);
            dbg_level(0);
            emit(0, "quality qi=%d qa=%d d=%d det=%d w40=%d e=%d: r=%d q48=%d q4a=%d q4e=%d", qi, q4as[qa], d, det, w40, e, r, o->bundle->decErrSmooth, o->bundle->qualBlocks, o->bundle->qualityOk);
          }
  V17RX_delete(o);
  ubuf_free(&c);
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
    sess_new(&s, rate, 0, 60, 0, 0);
    for (k = 0; k < n + 1; k++) blk[k] = g_cap[(rate & 3) * 2][(300 + k) % g_capn[(rate & 3) * 2]];
    dbg_level(2);
    g_verbose = 1;
    nn = (unsigned short)n;
    k = DemodDataV17(s.o, blk, g_out, nn);
    emit(1, "big block n=%d ret=%d out=%016llx", n, k, fnv_n(g_out, 1024));
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
  for (trial = 0; trial < 120; trial++) {
    Sess s;
    int rate = trial % 7 == 3 ? 0x1111 : (trial & 3);
    int variant = ((trial / 4) & 3) | (trial % 9 == 4 ? 16 : 0);
    int flag = (trial / 3) % 3 == 1;
    int warm = (int)(trial % 6 == 0 ? 0 : trial % 6 == 1 ? 14 : trial % 6 == 2 ? 40 : trial % 6 == 3 ? 75 : trial % 6 == 4 ? 110 : 160) + (int)rmod(6);
    int sub, dbgs = trial % 6 == 5;
    sess_new(&s, rate, variant, flag ? warm / 2 : warm, (trial % 4 == 1), flag);
    cov_hit(CV_H_WARM);
    dbg_level(dbgs ? 2 : 0);
    for (sub = 0; sub < 60; sub++) {
      static const int sts[12] = {0, 1, 2, 3, 4, 5, 6, -1, 7, 8, 0x7fff, 5};
      static const int cnts[7] = {0, 1, 2, 5, -1, 100, 3};
      static const int ns[9] = {0, 1, 5, 40, 100, 150, 152, 17, 152}; /* (blocks of 159 / 160 can make the SRE emit more than the 164 samples of its output buffer when it slips: a blob overrun, probed separately) */
      int fnsel = (int)(rnd() % 16u), n = ns[rnd() % 9u], i, ret = 0, outn;
      V17Rx *o = s.o;
      unsigned short nn;
      static short blk[512];
      int adv = (int)rmod(4);
      short q54pre, s2cpre, mwpre, mppre, mfpre;
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
        o->hdx->state = (short)sts[rnd() % 12u];
        o->hdx->count = (short)cnts[rnd() % 7u];
      }
      agc_reset(o);
      next_block(&s, blk, n);
      blk[n] = 0x5151;
      q54pre = o->bundle->refBlocks;
      s2cpre = o->hdx->sinceTone;
      mwpre = o->bundle->mrf.wpos;
      mppre = o->bundle->mrf.phase;
      mfpre = o->bundle->mrf.frac;
      statepre = o->hdx->state;
      for (i = 0; i < 2048; i++) g_out[i] = 0x7777;
      nn = (unsigned short)n;
      if (n == 0) cov_hit(CV_H_N0);
      if (n >= 150) cov_hit(CV_H_NBIG);
      if (o->hdx->state < 0 || o->hdx->state > 6) cov_hit(CV_H_STATEDEF);
      if (o->hdx->hold) cov_hit(CV_H_HOLD);
      if (o->hdx->s20 == 0) cov_hit(CV_H_PATHA); else cov_hit(CV_H_PATHB);
      if (o->bundle->dec.ctr > 0x3e7 && o->bundle->dec.ctr < 0x8000) cov_hit(CV_H_CNT);
      if (o->bundle->fse.decErrAvg > 0x3fff) cov_hit(CV_H_E52HI);
      if (o->bundle->energyTest) cov_hit(CV_H_Q50);
      if (o->hdx->sinceTone > 0x4ff) cov_hit(CV_H_S2C);
      if (o->hdx->flag) cov_hit(CV_H_FLAG);
      if (o->bundle->fse.eqForce) cov_hit(CV_H_C40);
      d_rx("pre", o);
      stack_poison();
      switch (fnsel) {
        case 0: o->hdx->fn = RxHdxStartV17; ret = RxHdxStartV17(o, blk, g_out, &nn); cov_hit(CV_H_START); break;
        case 1: o->hdx->fn = RxHdxEpochDetV17; ret = RxHdxEpochDetV17(o, blk, g_out, &nn); cov_hit(CV_H_EPOCH); break;
        case 2: o->hdx->fn = RxHdxPrtcolV17; ret = RxHdxPrtcolV17(o, blk, g_out, &nn); cov_hit(CV_H_PRTCOL); break;
        case 3: o->hdx->fn = RxHdxBridgeV17; ret = RxHdxBridgeV17(o, blk, g_out, &nn); cov_hit(CV_H_BRIDGE); break;
        case 4: o->hdx->fn = RxHdxScramV17; ret = RxHdxScramV17(o, blk, g_out, &nn); cov_hit(CV_H_SCRAM); break;
        case 5: o->hdx->fn = RxHdxDataV17; ret = RxHdxDataV17(o, blk, g_out, &nn); cov_hit(CV_H_DATA); break;
        case 6: o->hdx->fn = RxHdxIdleV17; ret = RxHdxIdleV17(o, blk, g_out, &nn); cov_hit(CV_H_IDLE); break;
        case 7: o->hdx->fn = RxHdxErrorV17; ret = RxHdxErrorV17(o, blk, g_out, &nn); cov_hit(CV_H_ERROR); break;
        case 8: ret = DemodDataV17(o, blk, g_out, nn); cov_hit(CV_H_DEMOD); break;
        case 9:
          ret = DataCarrierDetectV17(o, blk, nn);
          if (o->hdx->s20 == 0) cov_hit(CV_H_PATHA);
          if (ret) cov_hit(CV_H_CARR); else cov_hit(CV_H_NOCARR);
          break;
        case 10:
          ret = QualityDetectV17(o);
          if (o->bundle->qualBlocks == 0) cov_hit(CV_H_QUAL0);
          else if (o->bundle->qualBlocks < 0x32) cov_hit(CV_H_QUAL1);
          else if (o->bundle->qualBlocks == 0x32) cov_hit(CV_H_QUAL2);
          else cov_hit(CV_H_QUAL3);
          emit(0, "quality again %d", QualityDetectV17(o));
          break;
        case 11:
          RxNextStateV17(o);
          ret = o->hdx->state;
          cov_hit(CV_H_NEXT);
          break;
        case 12:
          ret = CarrierDetectV17(o);
          emit(0, "epoch %d snr %d", EpochDetectV17(o), GetSNRV17(o));
          {
            FaxV17Status st;
            memset(&st, (int)(rnd() & 0xffu), sizeof st);
            emit(0, "hdx status %d", V17RX_status(o, &st));
            prt_status("hdx status", &st);
            cov_hit(CV_H_STATUS);
          }
          cov_hit(CV_C_GETSNR);
          cov_hit(CV_C_EPOCHDET);
          cov_hit(CV_C_CARRDET);
          break;
        case 13:
          StoreCoefV17(o);
          ret = o->cfg.freq[0];
          emit(0, "store coefI=%016llx coefQ=%016llx", fnv_n(o->cfg.coefI, 0x31), fnv_n(o->cfg.coefQ, 0x31));
          cov_hit(CV_H_STORE);
          break;
        case 14:
          Restore_rateV17(o);
          ret = o->bundle->fse.settleCtr;
          cov_hit(CV_H_RESTORE);
          break;
        default: { /* the dispatcher on this state */
          unsigned short n2 = nn;
          ret = V17RX_modem(o, blk, g_out, &n2);
          nn = n2;
          break;
        }
      }
      if (o->bundle->sre.pllEnable == 0 && o->bundle->sreGate) cov_hit(CV_H_CDET_BRANCH);
      if (fnsel >= 2 && fnsel <= 4 && statepre >= 2 && statepre <= 4 && o->hdx->state != statepre) cov_hit(CV_H_PRTCOL_END);
      if (fnsel >= 2 && fnsel <= 5) {
        if (o->st.b[1] & 0x80) cov_hit(CV_H_LOWSNR); else cov_hit(CV_H_HIGHSNR);
      }
      if ((fnsel == 5 || fnsel == 9) && q54pre == 1 && o->bundle->energyTest && o->bundle->refBlocks == 0) cov_hit(CV_H_Q54);
      if ((fnsel == 5 || fnsel == 9) && o->hdx->s20 != 0 && s2cpre != 0 && o->hdx->sinceTone == 0) cov_hit(CV_H_MTD2);
      if (fnsel == 8 && statepre == 0 && n >= 16 && o->bundle->mrf.wpos == mwpre && o->bundle->mrf.phase == mppre && o->bundle->mrf.frac == mfpre) cov_hit(CV_H_DEMOD_MTD);
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
  {
    int pb;
    for (pb = 181; pb <= 192; pb++) probe_big(pb, pb & 3);
    probe_big(200, 1);
    probe_big(183, 0);
    probe_big(165, 1);
    probe_big(166, 3);
    probe_big(164, 2);
    probe_big(163, 0);
  }
}

/* ============================================================================================ scenario 5: StoreCoef / Restore, next state, dispatcher, control, status */
static int g_fake_calls;
static const short *g_fk_in, *g_fk_out;
static int fake_handler(V17Rx *o, short *in, short *out, unsigned short *n)
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
  /* (a) StoreCoefV17 / Restore_rateV17 directly, on engines warmed up by real signals and on poked ones */
  for (trial = 0; trial < 40; trial++) {
    Sess s;
    int rate = trial & 3, sub;
    sess_new(&s, rate, (trial / 4) & 3, trial % 3 == 0 ? 0 : 20 + 20 * (trial % 4), 0, (trial / 4) & 1);
    for (sub = 0; sub < 24; sub++) {
      V17Rx *o = s.o;
      V17RxBundle *b = o->bundle;
      int k;
      short ref[0x31 * 2 + 1];
      static const int thr[6] = {0x5dc, 0x100, 0, -1, 0x7ffb, -32768};
      for (k = 0; k < 0x31; k++) {
        b->fse.tapsI[k] = (short)(rpct(50) ? (int)rnd() : k * 331);
        b->fse.tapsQ[k] = (short)(rpct(50) ? (int)rnd() : -k * 77);
        o->cfg.coefI[k] = (short)rnd();
        o->cfg.coefQ[k] = (short)rnd();
      }
      b->fse.freqAcc = (int)rnd();
      b->fse.cfg.settleCount = (short)thr[rnd() % 6u];
      b->fse.settleCtr = (short)rnd();
      o->cfg.freq[0] = (short)(rpct(30) ? (int)(rnd() & 0xff) - 128 : (int)rnd());
      stack_poison();
      {
        unsigned char *pi = (unsigned char *)b->fse.tapsI, *pq = (unsigned char *)b->fse.tapsQ;
        for (k = 0x62; k < 0x70; k++) pi[k] = (unsigned char)(0x5e + k), pq[k] = (unsigned char)(0x7e + k); /* the padding behind the source taps holds known bytes: a store loop that runs past 0x31 taps copies them into the caller's padding (the heap checks refuse that) */
      }
      if (sub & 1) {
        StoreCoefV17(o);
        {
          unsigned char *pi = (unsigned char *)b->fse.tapsI, *pq = (unsigned char *)b->fse.tapsQ;
          for (k = 0x62; k < 0x70; k++) pi[k] = 0xcc, pq[k] = 0xcc;
        }
        cov_hit(CV_C_STORE);
        emit(0, "store coefI=%016llx coefQ=%016llx freq=%d w3c=%08x", fnv_n(o->cfg.coefI, 0x31), fnv_n(o->cfg.coefQ, 0x31), o->cfg.freq[0], u(b->fse.freqAcc));
        for (k = 0; k < 0x31; k++) ref[k] = o->cfg.coefI[k], ref[0x31 + k] = o->cfg.coefQ[k];
        ref[0x62] = o->cfg.freq[0];
        b->fse.freqAcc = 12345;
        Restore_rateV17(o);
        cov_hit(CV_C_RESTORE);
        if (b->fse.freqAcc == (int)ref[0x62]) cov_hit(CV_C_STORE_ROUND);
      } else {
        Restore_rateV17(o);
        cov_hit(CV_C_RESTORE);
        {
          unsigned char *pi = (unsigned char *)b->fse.tapsI, *pq = (unsigned char *)b->fse.tapsQ;
          for (k = 0x62; k < 0x70; k++) pi[k] = 0xcc, pq[k] = 0xcc;
        }
      }
      emit(0, "restore w3c=%08x s88=%d thr1c=%d", u(b->fse.freqAcc), b->fse.settleCtr, b->fse.cfg.settleCount);
      d_rx("sr", o);
    }
    sess_free(&s);
    trial_end("storecoef");
  }
  /* (a2) StoreCoefV17 with a large canary block behind the caller's buffers: a store loop that starts at a wrong index writes far beyond them */
  for (trial = 0; trial < 3; trial++) {
    V17RxCfg c;
    V17Rx *o;
    unsigned char *canary;
    int k;
    make_rcfg(&c, trial, 0, trial & 1);
    canary = (unsigned char *)sysdep_malloc(300000);
    for (k = 0; k < 300000; k++) canary[k] = (unsigned char)(0x11 + k * 7);
    o = rx_make(NULL, &c, 0);
    {
      unsigned char *pi = (unsigned char *)o->bundle->fse.tapsI, *pq = (unsigned char *)o->bundle->fse.tapsQ;
      for (k = 0x62; k < 0x70; k++) pi[k] = (unsigned char)(0x5e + k), pq[k] = (unsigned char)(0x7e + k);
      for (k = 0; k < 0x31; k++) o->bundle->fse.tapsI[k] = (short)(k * 13 + 1), o->bundle->fse.tapsQ[k] = (short)(-k * 17 - 1);
      o->bundle->fse.freqAcc = 77 + trial;
      StoreCoefV17(o);
      for (k = 0x62; k < 0x70; k++) pi[k] = 0xcc, pq[k] = 0xcc;
    }
    emit(0, "store canary %016llx coefI=%016llx coefQ=%016llx freq=%d", fnv_bytes(canary, 300000), fnv_n(o->cfg.coefI, 0x31), fnv_n(o->cfg.coefQ, 0x31), o->cfg.freq[0]);
    V17RX_delete(o);
    sysdep_free(canary);
    ubuf_free(&c);
    trial_end("store canary");
  }
  /* (b) RxNextStateV17 for every state x rate x training flag x debug level, then one handler call */
  for (a = 0; a < 14 * 4 * 8 * 2; a++) {
    static const int sts[14] = {0, 1, 2, 3, 4, 5, 6, -1, 7, 8, 0x7fff, 2, 4, 6};
    static const int flags[8] = {0, 1, -1, 2, -2, 0x10000, 5, -0x8000};
    int st = sts[a % 14], rate = (a / 14) & 3, flag = flags[(a / 56) & 7], dbg = (a / 448) & 1;
    V17RxCfg c;
    V17Rx *o;
    make_rcfg(&c, rate, a & 1, flag);
    o = rx_make(NULL, &c, 0);
    o->hdx->state = (short)st;
    o->hdx->count = (short)(int)(rnd() & 0x7fu);
    o->hdx->hold = 1;
    o->st.w = (int)rnd();
    dbg_level(dbg ? 2 : 0);
    d_rx("pre", o);
    stack_poison();
    RxNextStateV17(o);
    dbg_level(0);
    d_rx("next", o);
    if (dbg) cov_hit(CV_C_NEXT_DBG);
    if (st == 1 || st == 2) cov_hit(flag ? CV_C_NEXT_FLAG1 : CV_C_NEXT_FLAG0);
    if (st >= 0 && st <= 6) {
      unsigned short n = (unsigned short)(1 + rmod(160));
      int ret, k;
      static short blk[512];
      for (k = 0; k < 512; k++) blk[k] = (short)((int)(rnd() & 0xfffu) - 0x800);
      for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
      ret = o->hdx->fn(o, blk, g_out, &n);
      emit(0, "next+handler ret=%d n=%u out=%016llx", ret, n, fnv_n(g_out, 1024));
      d_rx("after", o);
    }
    V17RX_delete(o);
    ubuf_free(&c);
    trial_end("next");
  }
  /* (c) the dispatcher with a harness handler */
  for (trial = 0; trial < 60; trial++) {
    V17RxCfg c;
    int n, k;
    V17Rx *o;
    unsigned short nn;
    static short blk[512];
    make_rcfg(&c, trial & 3, 0, 0);
    o = rx_make(NULL, &c, 0);
    o->hdx->fn = fake_handler;
    g_fake_calls = trial % 6;
    n = trial % 9 == 0 ? 0 : 1 + (int)rmod(150);
    for (k = 0; k < 512; k++) blk[k] = (short)(0x6000 + k);
    for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
    nn = (unsigned short)n;
    o->st.w = (int)rnd();
    g_fk_in = blk;
    g_fk_out = g_out;
    k = V17RX_modem(o, blk, g_out, &nn);
    emit(0, "fake modem n=%d ret=%08x nn=%u calls=%d out=%016llx", n, u(k), nn, g_fake_calls, fnv_n(g_out, 1024));
    cov_hit(CV_C_FAKE);
    if (trial % 6 == 2 || trial % 6 == 5) cov_hit(CV_C_FAKE_NEG);
    if (trial % 6 == 0 || trial % 6 == 4) cov_hit(CV_C_FAKE_PART);
    d_rx("fake", o);
    V17RX_delete(o);
    ubuf_free(&c);
    trial_end("fake");
  }
  /* (d) V17RX_control for every b0c x eight b0d values (and the training flag), then a block */
  for (b0c = 0; b0c < 256; b0c++) {
    for (b0d = 0; b0d < 8; b0d++) {
      static const int d4[8] = {0, 0x10, 2, 0x12, 0x20, 0x30, 1, 0xff};
      FaxV17Ctl ctl;
      V17RxCfg c;
      int k, ret;
      V17Rx *o;
      unsigned short nn = 60;
      static short blk[512];
      make_rcfg(&c, (b0c + b0d) & 3, 0, (b0c >> 3) & 1);
      o = rx_make(NULL, &c, 0);
      memset(&ctl, 0, sizeof ctl);
      ctl.w4 = (int)rnd();
      ctl.b0c = (unsigned char)b0c;
      ctl.b0d = (unsigned char)d4[b0d];
      ctl.w10 = (b0c >> 4) & 1;
      ret = V17RX_control(o, &ctl);
      emit(0, "control c=%02x d=%02x ret=%d", b0c, d4[b0d], ret);
      cov_hit(CV_C_CTL);
      if (d4[b0d] & 0x10) cov_hit(CV_C_CTL_HOLD);
      if (d4[b0d] & 2) {
        cov_hit(CV_C_CTL_RECREATE);
        if (ctl.w10 != c.w[5]) cov_hit(CV_C_CTL_FLAG);
      }
      if (b0c & 8) cov_hit(CV_C_CTL_W0);
      if (b0c & 0x20) cov_hit(CV_C_CTL_W10);
      for (k = 0; k < 512; k++) blk[k] = (short)((int)(rnd() & 0x1fffu) - 0x1000);
      for (k = 0; k < 2048; k++) g_out[k] = 0x7777;
      V17RX_modem(o, blk, g_out, &nn);
      emit(0, "control run nn=%u out=%016llx", nn, fnv_n(g_out, 1024));
      if (b0c % 17 == 0) d_rx("ctl", o);
      emit(0, "control hold=%d w0=%d w10=%d w2=%08x flag=%d", o->hdx->hold, o->bundle->w0, o->bundle->eqGate, u(o->cfg.w[2]), o->hdx->flag);
      V17RX_delete(o);
      ubuf_free(&c);
      trial_end("ctl");
    }
  }
  emit(0, "control null %d", V17RX_control((V17Rx *)NULL, NULL));
  cov_hit(CV_C_CTL_NULL);
  {
    FaxV17Status dummy;
    V17Rx *o = rx_make(NULL, NULL, 0);
    memset(&dummy, 0x55, sizeof dummy);
    emit(0, "status %d %d", V17RX_status(o, &dummy), V17RX_status(o, NULL));
    prt_status("status", &dummy);
    cov_hit(CV_C_STATUS);
    cov_hit(CV_C_STATUS_NULL);
    V17RX_delete(o);
    trial_end("status");
  }
}

/* ============================================================================================ main */
int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R26_SEED");
  g_verbose = getenv("R26_VERBOSE") != NULL;
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
