/*
 * R25 differential harness: the fax V.17 TRANSMITTER of dsplibs.o (V17TX_create / _delete / _modem / _control / _status, TxHdx{Start,Silence,TEP_,AB,Bridge,EQCond,SCR1,Data,Idle}V17,
 * TxNextStateV17, SetTxModeV17, ModDataV17, TxNoCarrierV17, the v17tx_* VMI wrappers and the blob-LOCAL init_vmi_v17tx) vs src/fax/fax_v17_tx.c + src/fax/fax_pulse_shaping.c + tables.  The blob run
 * (-DTEST_BLOB, linked with ../dsplibs.o, init_vmi_v17tx made global by objcopy) and the port run (-m32 -O0 / -O2, x86-64 against the recorded blob output) execute the same code; library
 * test/fax/r25_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation, deterministic heap with block ids / red zones / poisoned frees, callee recorders sysdep_malloc /
 * sysdep_free / dsplibs_debug_printf).  The R18 callees (FIFO_*, SGD_*, SDM_*, SMCv17_*, ScrambleDataV17, SeedScramblerV17, SetEncoderV17) are the real ones on both sides (the blob's own in the
 * blob run, src/fax/fax_*.c in the port runs).  Integer signals only: the modulated samples are compared sample for sample through hashes of every call's output block.  Nothing printed depends on
 * the host layout: objects are dumped member by member (pointers as table+index / block id, the content of every malloc()ed block as a hash).
 *
 * Fields the blob never writes (the bundle's two leading words, smc.w4 / w0a / k, the encoder select until SetTxModeV17 / SetEncoderV17 run, hdx.s12 / s1c) are uninitialised heap in the
 * blob: the harness POKES known values right after every create (settle()), so the dumps can print every member and a stray port write to one of them shows.  Two consistent
 * configurations are used for direct handler calls (the pulse shaper indexes its constellation map with the symbol codes, a mismatch would read past a table, layout dependent): mode A =
 * as created (map SMCv17_IMAP4, no-carrier code 4, encoder select dif / abs), mode B = after SetTxModeV17(rate) (VTBv17_* map, trellis encoder possible).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 tables: every R25 table dumped, the message table through v17tx_message for -3 .. 13 and 0x100 ..;
 *   1 lifecycle: V17TX_create (NULL object / caller-supplied with and without hdx / live, NULL / custom / fuzzed cfg, the four rates + other, debug level 0 and 2), create(o, o), delete,
 *     run-after-create, rate-changing re-creates, forked probes: a failing malloc #0 .. #11 and a supplied object without bundle, FIFO frame growth;
 *   2 transmit streams: the four rates through the whole machine (start, silence, TEP, silence, AB, EQ conditioning, bridge, SCR1, data, [pend: under-run -> SCR1 end -> silence -> idle -> start]),
 *     long and short training, steady / short / long input blocks (under-run, over-run), pend / gain / flag / re-create control, status, 64 engines, the whole engine dumped after every call;
 *   3 handlers: every TxHdx*V17 x every hdx state {0 .. 11, -1, 12, 0x7fff} x rate x count x FIFO fill x pend x n x flag x mode A / B, plus TxHdxDataV17 with odd rate words;
 *   4 TxNextStateV17 for every state x rate x flag x debug level x mode (the jump table: every case and the default), SetTxModeV17 for the rates (and the invalid ones: flags only),
 *     ModDataV17 / TxNoCarrierV17 directly with n up to the circular buffer wrap and all three encoders, long runs of the pulse shaper;
 *   5 V17TX_modem with a harness handler (any count / return value), V17TX_control for 512 flag-byte combinations (then a modulated block: the gain), V17TX_status with pokes;
 *   6 VMI: fax_init_vmi_v17tx (blob: init_vmi_v17tx) with debug levels 0 .. 3, v17tx_create / process / status / control / delete / message on garbage-initialised FaxVmiV17 objects.
 * R25_VERBOSE=1 prints every line (debugging), R25_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r25_common.h"
#include "fax/fax_v17_tx_tables.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
extern int init_vmi_v17tx(FaxVmiV17 *m, unsigned short rate, int unused, void *user);
#define fax_init_vmi_v17tx init_vmi_v17tx
#else
#define SIDE_BLOB 0
#endif

extern unsigned int dsplibs_debug_level;

#include "fax/fax_modem_views.h"
#if UINTPTR_MAX != 0xffffffffu
/* x86-64: src/fax/fax_modem_wrappers.c references ALL eleven accessors of src/fax/fax_modem_views.h; this test uses the three of the V.17 TRANSMIT half, which src/fax/fax_v17_tx.c defines.  The other eight belong to the
 * V.17 / V.29 receive / V.27ter engines: stand-ins that abort (no scenario of this test reaches the wrappers that use them). */
static void *no_engine(void) { fprintf(stderr, "HARNESS/MODEL VIOLATION: accessor of another engine called\n"); exit(3); }
FaxSdm *faxw_v17_rxsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSdm *faxw_v29_txsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
int *faxw_v29_encsel(void *o) { (void)o; return (int *)no_engine(); }
FaxSdm *faxw_v29_rxsdm(void *o) { (void)o; return (FaxSdm *)no_engine(); }
FaxSdmV27 *faxw_v27_txsdm(void *o) { (void)o; return (FaxSdmV27 *)no_engine(); }
short *faxw_v27_rate(void *o) { (void)o; return (short *)no_engine(); }
FaxSdmV27 *faxw_v27_rxsdm(void *o) { (void)o; return (FaxSdmV27 *)no_engine(); }
#endif
/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_TAB, CV_MSG, CV_MSG_OOB, CV_MSG_NEG,
  CV_L_NEW, CV_L_SUP_EMPTY, CV_L_SUP_NOHDX, CV_L_SUP_LIVE, CV_L_NULLCFG, CV_L_R7200, CV_L_R9600, CV_L_R12000, CV_L_R14400, CV_L_RBAD, CV_L_DBG, CV_L_DEL, CV_L_PROBE, CV_L_RECREATE, CV_L_RUN,
  CV_L_NOREALLOC, CV_L_RATECHG, CV_L_SUPNOBUNDLE, CV_L_FIFO2, CV_L_FIFOGROW, CV_L_FLAG,
  CV_S_START, CV_S_C0, CV_S_C1, CV_S_C2, CV_S_C3, CV_S_C4, CV_S_C5, CV_S_C6, CV_S_C7, CV_S_C8, CV_S_C9, CV_S_C10, CV_S_C11, CV_S_BRIDGE, CV_S_SHORT,
  CV_S_TO_DATA, CV_S_TO_IDLE, CV_S_TO_START, CV_S_UNDER, CV_S_OVER, CV_S_PEND, CV_S_GAIN, CV_S_STATUS, CV_S_RECREATE, CV_S_R7200, CV_S_R9600, CV_S_R12000, CV_S_R14400, CV_S_N0, CV_S_NFULL,
  CV_S_NLONG, CV_S_NSHORT, CV_S_DBG, CV_S_BADRATE, CV_S_BIGFIFO, CV_S_DATASTART2, CV_S_DATASTART3, CV_S_DATASTART4, CV_S_DATASTART5, CV_S_FLAGCTL,
  CV_H_START, CV_H_SILENCE, CV_H_TEP, CV_H_AB, CV_H_BRIDGE, CV_H_EQ, CV_H_SCR1, CV_H_DATA, CV_H_IDLE, CV_H_COUNT0, CV_H_COUNTNEG, CV_H_COUNTBIG, CV_H_FIFO0, CV_H_FIFOF, CV_H_PEND,
  CV_H_UNDER, CV_H_UNDERPEND, CV_H_STATEDEF, CV_H_RATEODD, CV_H_N0, CV_H_NSMALL, CV_H_IDLEFIFO, CV_H_MODEA, CV_H_MODEB, CV_H_ENC0, CV_H_ENC1, CV_H_ENC2, CV_H_FLAG,
  CV_N_C0, CV_N_C1, CV_N_C2, CV_N_C3, CV_N_C4, CV_N_C5, CV_N_C5S, CV_N_C6, CV_N_C7, CV_N_C8, CV_N_C9, CV_N_C10, CV_N_C11, CV_N_DEFAULT, CV_N_DBG, CV_N_MOD0, CV_N_MOD1, CV_N_MOD2, CV_N_NOCAR,
  CV_N_WRAP, CV_N_SETMODE, CV_N_SETMODE_BAD, CV_N_LONG, CV_N_STEP,
  CV_D_FAKE, CV_D_PEND, CV_D_NOPEND, CV_D_OVER, CV_D_NEGRET, CV_D_CTL, CV_D_CTL_PEND, CV_D_CTL_RECREATE, CV_D_CTL_GAIN, CV_D_CTL_B0C, CV_D_CTL_NULL, CV_D_CTL_W10, CV_D_STAT, CV_D_STAT_NULL,
  CV_V_INIT, CV_V_INIT_DBG, CV_V_CREATE, CV_V_CREATE_NULLCFG, CV_V_CREATE_SUP, CV_V_PROC, CV_V_STATUS, CV_V_CTL, CV_V_DEL, CV_V_MSG, CV_V_R7200, CV_V_R9600, CV_V_R12000, CV_V_R14400, CV_V_RBAD
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_TAB, "tables dumped");
      cov_def(CV_MSG, "v17tx_message idx 0..9");
      cov_def(CV_MSG_OOB, "v17tx_message idx 10 (out of table)");
      cov_def(CV_MSG_NEG, "v17tx_message idx > limit / negative");
      break;
    case 1:
      cov_def(CV_L_NEW, "V17TX_create NULL object");
      cov_def(CV_L_SUP_EMPTY, "V17TX_create supplied object with live hdx / bundle");
      cov_def(CV_L_SUP_NOHDX, "V17TX_create supplied object, hdx NULL");
      cov_def(CV_L_SUP_LIVE, "V17TX_create supplied live object (re-create)");
      cov_def(CV_L_NULLCFG, "V17TX_create NULL cfg");
      cov_def(CV_L_R7200, "V17TX_create rate 7200");
      cov_def(CV_L_R9600, "V17TX_create rate 9600");
      cov_def(CV_L_R12000, "V17TX_create rate 12000");
      cov_def(CV_L_R14400, "V17TX_create rate 14400");
      cov_def(CV_L_RBAD, "V17TX_create other rate (error flags)");
      cov_def(CV_L_DBG, "V17TX_create debug level 2");
      cov_def(CV_L_DEL, "V17TX_delete");
      cov_def(CV_L_PROBE, "create malloc-failure probe");
      cov_def(CV_L_RECREATE, "V17TX_create(o, o)");
      cov_def(CV_L_RUN, "run after create");
      cov_def(CV_L_NOREALLOC, "PPS buffers kept (taps equal)");
      cov_def(CV_L_RATECHG, "rate-changing re-create");
      cov_def(CV_L_SUPNOBUNDLE, "supplied object without bundle probe");
      cov_def(CV_L_FIFO2, "cfg word 5 = 2 FIFO frames");
      cov_def(CV_L_FIFOGROW, "FIFO frame count change probe");
      cov_def(CV_L_FLAG, "cfg word 6 (training flag) set");
      break;
    case 2:
      cov_def(CV_S_START, "state Start observed");
      cov_def(CV_S_C0, "transition out of state 0");
      cov_def(CV_S_C1, "transition out of state 1");
      cov_def(CV_S_C2, "transition out of state 2");
      cov_def(CV_S_C3, "transition out of state 3");
      cov_def(CV_S_C4, "transition out of state 4");
      cov_def(CV_S_C5, "transition out of state 5");
      cov_def(CV_S_C6, "transition out of state 6");
      cov_def(CV_S_C7, "transition out of state 7");
      cov_def(CV_S_C8, "transition out of state 8");
      cov_def(CV_S_C9, "transition out of state 9");
      cov_def(CV_S_C10, "transition out of state 10");
      cov_def(CV_S_C11, "transition out of state 11");
      cov_def(CV_S_BRIDGE, "bridge state observed (long training)");
      cov_def(CV_S_SHORT, "short training stream (no bridge)");
      cov_def(CV_S_TO_DATA, "SCR1 -> Data");
      cov_def(CV_S_TO_IDLE, "-> Idle");
      cov_def(CV_S_TO_START, "Idle -> Start");
      cov_def(CV_S_UNDER, "TX under-run");
      cov_def(CV_S_OVER, "TX over-run");
      cov_def(CV_S_PEND, "pend control");
      cov_def(CV_S_GAIN, "gain control");
      cov_def(CV_S_STATUS, "status");
      cov_def(CV_S_RECREATE, "control re-create");
      cov_def(CV_S_R7200, "rate 7200 engine");
      cov_def(CV_S_R9600, "rate 9600 engine");
      cov_def(CV_S_R12000, "rate 12000 engine");
      cov_def(CV_S_R14400, "rate 14400 engine");
      cov_def(CV_S_N0, "call with n = 0");
      cov_def(CV_S_NFULL, "call with a full frame");
      cov_def(CV_S_NLONG, "call with more than a frame");
      cov_def(CV_S_NSHORT, "call with less than a frame");
      cov_def(CV_S_DBG, "debug level 2 stream");
      cov_def(CV_S_BADRATE, "stream with an invalid rate");
      cov_def(CV_S_BIGFIFO, "stream with a 2 frame FIFO");
      cov_def(CV_S_DATASTART2, "data start state byte 2 (14400)");
      cov_def(CV_S_DATASTART3, "data start state byte 3 (12000)");
      cov_def(CV_S_DATASTART4, "data start state byte 4 (9600)");
      cov_def(CV_S_DATASTART5, "data start state byte 5 (7200)");
      cov_def(CV_S_FLAGCTL, "control sets the training flag");
      break;
    case 3:
      cov_def(CV_H_START, "TxHdxStartV17");
      cov_def(CV_H_SILENCE, "TxHdxSilenceV17");
      cov_def(CV_H_TEP, "TxHdxTEP_V17");
      cov_def(CV_H_AB, "TxHdxABV17");
      cov_def(CV_H_BRIDGE, "TxHdxBridgeV17");
      cov_def(CV_H_EQ, "TxHdxEQCondV17");
      cov_def(CV_H_SCR1, "TxHdxSCR1V17");
      cov_def(CV_H_DATA, "TxHdxDataV17");
      cov_def(CV_H_IDLE, "TxHdxIdleV17");
      cov_def(CV_H_COUNT0, "count 0");
      cov_def(CV_H_COUNTNEG, "count negative");
      cov_def(CV_H_COUNTBIG, "count > n");
      cov_def(CV_H_FIFO0, "FIFO empty");
      cov_def(CV_H_FIFOF, "FIFO filled");
      cov_def(CV_H_PEND, "pend set");
      cov_def(CV_H_UNDER, "Data under-run without pend");
      cov_def(CV_H_UNDERPEND, "Data under-run with pend");
      cov_def(CV_H_STATEDEF, "state number outside 0..11");
      cov_def(CV_H_RATEODD, "Data with a rate word outside 0 .. 3");
      cov_def(CV_H_N0, "n = 0");
      cov_def(CV_H_NSMALL, "n < count");
      cov_def(CV_H_IDLEFIFO, "Idle with a non-empty FIFO");
      cov_def(CV_H_MODEA, "mode A (as created)");
      cov_def(CV_H_MODEB, "mode B (after SetTxModeV17)");
      cov_def(CV_H_ENC0, "encoder select 0 (differential)");
      cov_def(CV_H_ENC1, "encoder select 1 (absolute)");
      cov_def(CV_H_ENC2, "encoder select 2 (trellis coded)");
      cov_def(CV_H_FLAG, "short training flag");
      break;
    case 4:
      cov_def(CV_N_C0, "TxNextStateV17 state 0");
      cov_def(CV_N_C1, "TxNextStateV17 state 1");
      cov_def(CV_N_C2, "TxNextStateV17 state 2");
      cov_def(CV_N_C3, "TxNextStateV17 state 3");
      cov_def(CV_N_C4, "TxNextStateV17 state 4");
      cov_def(CV_N_C5, "TxNextStateV17 state 5 (long training)");
      cov_def(CV_N_C5S, "TxNextStateV17 state 5 (short training)");
      cov_def(CV_N_C6, "TxNextStateV17 state 6");
      cov_def(CV_N_C7, "TxNextStateV17 state 7");
      cov_def(CV_N_C8, "TxNextStateV17 state 8");
      cov_def(CV_N_C9, "TxNextStateV17 state 9");
      cov_def(CV_N_C10, "TxNextStateV17 state 10");
      cov_def(CV_N_C11, "TxNextStateV17 state 11");
      cov_def(CV_N_DEFAULT, "TxNextStateV17 default");
      cov_def(CV_N_DBG, "debug strings");
      cov_def(CV_N_MOD0, "ModDataV17 differential");
      cov_def(CV_N_MOD1, "ModDataV17 absolute");
      cov_def(CV_N_MOD2, "ModDataV17 trellis");
      cov_def(CV_N_NOCAR, "TxNoCarrierV17");
      cov_def(CV_N_WRAP, "circular buffer wrap");
      cov_def(CV_N_SETMODE, "SetTxModeV17");
      cov_def(CV_N_SETMODE_BAD, "SetTxModeV17 invalid rate");
      cov_def(CV_N_LONG, "long pulse shaper runs");
      cov_def(CV_N_STEP, "pulse shaper phase step");
      break;
    case 5:
      cov_def(CV_D_FAKE, "V17TX_modem with a harness handler");
      cov_def(CV_D_PEND, "dispatcher with pend");
      cov_def(CV_D_NOPEND, "dispatcher without pend");
      cov_def(CV_D_OVER, "dispatcher over-run");
      cov_def(CV_D_NEGRET, "handler returns a negative count");
      cov_def(CV_D_CTL, "V17TX_control");
      cov_def(CV_D_CTL_PEND, "control pend");
      cov_def(CV_D_CTL_RECREATE, "control re-create");
      cov_def(CV_D_CTL_GAIN, "control gain");
      cov_def(CV_D_CTL_B0C, "control b0c bit 2");
      cov_def(CV_D_CTL_NULL, "control NULL");
      cov_def(CV_D_CTL_W10, "control w10 (training flag)");
      cov_def(CV_D_STAT, "V17TX_status");
      cov_def(CV_D_STAT_NULL, "status NULL");
      break;
    case 6:
      cov_def(CV_V_INIT, "fax_init_vmi_v17tx");
      cov_def(CV_V_INIT_DBG, "init debug level 3");
      cov_def(CV_V_CREATE, "v17tx_create");
      cov_def(CV_V_CREATE_NULLCFG, "v17tx_create NULL cfg");
      cov_def(CV_V_CREATE_SUP, "v17tx_create with a supplied engine");
      cov_def(CV_V_PROC, "v17tx_process");
      cov_def(CV_V_STATUS, "v17tx_status");
      cov_def(CV_V_CTL, "v17tx_control");
      cov_def(CV_V_DEL, "v17tx_delete");
      cov_def(CV_V_MSG, "v17tx_message");
      cov_def(CV_V_R7200, "VMI rate 7200");
      cov_def(CV_V_R9600, "VMI rate 9600");
      cov_def(CV_V_R12000, "VMI rate 12000");
      cov_def(CV_V_R14400, "VMI rate 14400");
      cov_def(CV_V_RBAD, "VMI other rate");
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
  /* the machine's own transitions inside one V17TX_modem call are only visible through the debug strings (level 2 runs) */
  if (!strncmp(b, "V17TX_STATE_START", 17)) { cov_hit(CV_S_START); cov_hit(CV_S_C0); }
  if (!strncmp(b, "V17TX_STATE_SILENCE", 19)) cov_hit(CV_S_C1);
  if (!strncmp(b, "V17TX_STATE_TEP", 15)) cov_hit(CV_S_C2);
  if (!strncmp(b, "V17TX_STATE_QUIET|", 18)) cov_hit(CV_S_C3);
  if (!strncmp(b, "V17TX_STATE_ALT", 15)) cov_hit(CV_S_C4);
  if (!strncmp(b, "V17TX_STATE_EQCOND", 18)) cov_hit(CV_S_C5);
  if (!strncmp(b, "V17TX_STATE_BRIDGE", 18)) { cov_hit(CV_S_C6); cov_hit(CV_S_BRIDGE); }
  if (!strncmp(b, "V17TX_STATE_SCR1|", 17)) cov_hit(CV_S_C7);
  if (!strncmp(b, "V17TX_STATE_DATA", 16)) cov_hit(CV_S_C8);
  if (!strncmp(b, "V17TX_STATE_SCR1_END", 20)) cov_hit(CV_S_C9);
  if (!strncmp(b, "V17TX_STATE_QUIET_END", 21)) cov_hit(CV_S_C10);
  if (!strncmp(b, "V17TX_STATE_IDLE", 16)) { cov_hit(CV_S_C11); cov_hit(CV_S_TO_START); }
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
/* the SGD: only the words the V.17 code can have set deterministically (the sequence-part words tab / len / mode / w14 are stack garbage in the blob) */
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
static const char *txfn(V17TxHdxFn f)
{
  if (f == TxHdxStartV17) return "Start";
  if (f == TxHdxSilenceV17) return "Silence";
  if (f == TxHdxTEP_V17) return "TEP";
  if (f == TxHdxABV17) return "AB";
  if (f == TxHdxBridgeV17) return "Bridge";
  if (f == TxHdxEQCondV17) return "EQCond";
  if (f == TxHdxSCR1V17) return "SCR1";
  if (f == TxHdxDataV17) return "Data";
  if (f == TxHdxIdleV17) return "Idle";
  return f ? "other" : "NULL";
}
/* the encoder slots: the blob stores SMCv17_encoder_dif / _abs / _tcm, the port stores two thin wrappers of dif / abs (same signature) and SMCv17_encoder_tcm: the label says "set" for any of the
 * three non-NULL entries and the blob run asserts the exact addresses (the behaviour of the wrappers is what the modulated samples compare) */
static const char *encfn(V17EncFn f, int k)
{
#ifdef TEST_BLOB
  if (f != (V17EncFn)(k == 0 ? (void *)SMCv17_encoder_dif : k == 1 ? (void *)SMCv17_encoder_abs : (void *)SMCv17_encoder_tcm)) return "other";
  return k == 0 ? "dif" : k == 1 ? "abs" : "tcm";
#else
  if (k == 2 && f != (V17EncFn)SMCv17_encoder_tcm) return "other";
  return f ? (k == 0 ? "dif" : k == 1 ? "abs" : "tcm") : "NULL";
#endif
}
static short g_ibuf[64], g_qbuf[64];
static void register_tables(void)
{
  htab_add("PPS_ICOFFS", PPSv17_ICOFFS, sizeof PPSv17_ICOFFS);
  htab_add("PPS_QCOFFS", PPSv17_QCOFFS, sizeof PPSv17_QCOFFS);
  htab_add("VTB_IMAP16T", VTBv17_IMAP16T, sizeof VTBv17_IMAP16T);
  htab_add("VTB_QMAP16T", VTBv17_QMAP16T, sizeof VTBv17_QMAP16T);
  htab_add("VTB_IMAP32", VTBv17_IMAP32, sizeof VTBv17_IMAP32);
  htab_add("VTB_QMAP32", VTBv17_QMAP32, sizeof VTBv17_QMAP32);
  htab_add("VTB_IMAP64", VTBv17_IMAP64, sizeof VTBv17_IMAP64);
  htab_add("VTB_QMAP64", VTBv17_QMAP64, sizeof VTBv17_QMAP64);
  htab_add("VTB_IMAP128", VTBv17_IMAP128, sizeof VTBv17_IMAP128);
  htab_add("VTB_QMAP128", VTBv17_QMAP128, sizeof VTBv17_QMAP128);
  htab_add("SMC_IMAP4", SMCv17_IMAP4, sizeof SMCv17_IMAP4);
  htab_add("SMC_QMAP4", SMCv17_QMAP4, sizeof SMCv17_QMAP4);
  htab_add("IBUF", g_ibuf, sizeof g_ibuf);
  htab_add("QBUF", g_qbuf, sizeof g_qbuf);
}
static void d_tx(const char *t, const V17Tx *o)
{
  char p1[80], p2[80], p3[80], p4[80], p5[80], p6[80], p7[80], p8[80], p9[80];
  emit(0, "%s TX cfg=%08x %08x %08x %08x %08x %08x %08x %08x st=%08x", t, u(o->cfg.w[0]), u(o->cfg.w[1]), u(o->cfg.w[2]), u(o->cfg.w[3]), u(o->cfg.w[4]), u(o->cfg.w[5]), u(o->cfg.w[6]),
       u(o->cfg.w[7]), u(o->st.w));
  if (o->hdx) {
    const V17TxHdx *h = o->hdx;
    emit(0, "%s TXH pend=%d flag=%d rate=%d s12=%04x fn=%s state=%d count=%d s1c=%04x nocarr=%d", t, h->pend, h->flag, h->rate, (unsigned)(unsigned short)h->s12, txfn(h->fn), h->state, h->count,
         (unsigned)(unsigned short)h->s1c, h->nocarr);
    d_fifo(t, h->fifo);
    d_sgd(t, h->sgd);
  }
  if (o->bundle) {
    const V17TxBundle *b = o->bundle;
    const FaxSmcV17 *c = &b->smc;
    const FaxPps *p = &b->pps;
    emit(0, "%s TXB w0=%08x w4=%08x circ i=%s q=%s buf=%s wr=%d rd=%d lim=%d", t, u(b->w0), u(b->w4), (hblk(p1, 80, b->circ.i), p1), (hblk(p2, 80, b->circ.q), p2),
         (hblk(p3, 80, b->circ.buf), p3), b->circ.wr, b->circ.rd, b->circ.lim);
    emit(0, "%s TXB sdm n=%u mask=%04x nmask=%04x reg=%08x tap1=%d tap2=%d sh1=%d sh2=%d", t, (unsigned)b->sdm.n, b->sdm.mask, b->sdm.nmask, b->sdm.state, b->sdm.tap1, b->sdm.tap2, b->sdm.sh1,
         b->sdm.sh2);
    emit(0, "%s TXB smc w0=%04x w2=%04x w4=%04x x=%d y=%d w0a=%04x y2=%d st=%d w10=%04x k=%d  enc=%s %s %s mode=%u", t, c->tcmMask, c->w2, c->w4, c->x, c->y, c->w0a, c->y2, c->st, c->w10, c->k,
         encfn(b->enc[0], 0), encfn(b->enc[1], 1), encfn(b->enc[2], 2), (unsigned)b->encmode);
    emit(0, "%s TXB pps up=%d down=%d mode=%d scale=%d step=%d s0e=%d imap=%s qmap=%s ifilt=%s qfilt=%s filtlen=%d s22=%d w24=%08x need=%d acc=%d pos=%d taps=%d", t, p->cfg.up, p->cfg.down,
         p->cfg.mode, p->cfg.scale, p->cfg.step, p->cfg.s0e, dptr(p->cfg.imap, p1, 80), dptr(p->cfg.qmap, p2, 80), dptr(p->cfg.ifilt, p3, 80), dptr(p->cfg.qfilt, p4, 80), p->cfg.filtlen,
         p->cfg.s22, u(p->cfg.w24), p->need, p->acc, p->pos, p->taps);
    emit(0, "%s TXB hist I=%s Q=%s", t, (hblk(p5, 80, p->histI), p5), (hblk(p9, 80, p->histQ), p9));
    (void)p6; (void)p7; (void)p8; (void)p4;
  }
}
/* poke the members the blob never writes (uninitialised heap) with known values; call right after every create of a NEW object */
static void settle(V17Tx *o)
{
  o->bundle->w0 = 0x1357;
  o->bundle->w4 = 0x2468;
  o->bundle->smc.w4 = 0x1234;
  o->bundle->smc.w0a = 0x5678;
  o->bundle->smc.k = 2;
  o->bundle->encmode = 1;
  o->hdx->s12 = 0x3c3c;
  o->hdx->s1c = 0x5a5a;
}
/* mode B: the constellation of a rate (SetTxModeV17) and an encoder select */
static void mode_b(V17Tx *o, int rate)
{
  SetTxModeV17(o, (short)rate);
  SetEncoderV17(o, (short)(rnd() % 3u), (short)(rnd() & 3u));
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
static void sym_fill(short *b, int n, int bits)
{
  int i;
  for (i = 0; i < n; i++) b[i] = (short)(rpct(3) ? (int)(rnd() & 0xffffu) : (int)(rnd() & ((1u << bits) - 1u)));
}
static void prt_status(const char *t, const FaxV17Status *st)
{
  emit(0, "%s %d %d %d %d %d %d %d %d %d %d %02x %02x %08x", t, st->s0, st->s2, st->s4, st->s6, st->s8, st->sa, st->sc, st->se, st->s10, st->s12, st->b14, st->b15, u(st->w18));
}

/* rate codes: 0 = 7200, 1 = 9600, 2 = 12000, 3 = 14400, anything else is the raw baud word */
static int rate_word(int rate) { return rate == 0 ? 0x1c20 : rate == 1 ? 0x2580 : rate == 2 ? 0x2ee0 : rate == 3 ? 0x3840 : rate; }
static int rate_bits(int rate) { return rate >= 0 && rate <= 3 ? 3 + rate : 6; }
static void make_cfg(int *w, int rate, int variant)
{
  int i;
  for (i = 0; i < 8; i++) w[i] = V17TX_CFG[i];
  w[0] = (w[0] & 0xffff) | (rate_word(rate) << 16); /* s[1] = the baud rate */
  if (variant & 1) w[7] = (int)rnd();                /* PPS w24 */
  if (variant & 2) w[3] = 1 + (int)rmod(3);          /* a word nobody reads: carried */
  if (variant & 4) w[5] = 1 + (int)rmod(2);          /* FIFO frames */
  if (variant & 8) w[7] = 0;
  if (variant & 16) {
    w[2] = (int)rnd();
    w[4] = (int)rnd();
  }
  if (variant & 32) w[6] = 1 + (int)rmod(3);         /* short training */
}

/* ============================================================================================ scenario 0: tables */
static void scen0(void)
{
  int i;
  register_tables();
  emit(0, "TAB patterns scr1 %d %d %d %d sym %d %d %d %d scale %d %d %d %d", V17TX_PATTERN_SCR1[0], V17TX_PATTERN_SCR1[1], V17TX_PATTERN_SCR1[2], V17TX_PATTERN_SCR1[3], V17TX_SYM_SIZE[0],
       V17TX_SYM_SIZE[1], V17TX_SYM_SIZE[2], V17TX_SYM_SIZE[3], V17TX_PPS_SCALE[0], V17TX_PPS_SCALE[1], V17TX_PPS_SCALE[2], V17TX_PPS_SCALE[3]);
  emit(0, "TAB ifilt %016llx qfilt %016llx", fnv_shorts(PPSv17_ICOFFS, 120), fnv_shorts(PPSv17_QCOFFS, 120));
  emit(0, "TAB vtb16 %016llx %016llx vtb32 %016llx %016llx", fnv_shorts(VTBv17_IMAP16T, 17), fnv_shorts(VTBv17_QMAP16T, 17), fnv_shorts(VTBv17_IMAP32, 33), fnv_shorts(VTBv17_QMAP32, 33));
  emit(0, "TAB vtb64 %016llx %016llx vtb128 %016llx %016llx", fnv_shorts(VTBv17_IMAP64, 65), fnv_shorts(VTBv17_QMAP64, 65), fnv_shorts(VTBv17_IMAP128, 129), fnv_shorts(VTBv17_QMAP128, 129));
  for (i = 0; i < 17; i++) emit(0, "TAB vtb16 %d %d %d", i, VTBv17_IMAP16T[i], VTBv17_QMAP16T[i]);
  for (i = 0; i < 33; i += 4) emit(0, "TAB vtb32 %d %d %d", i, VTBv17_IMAP32[i], VTBv17_QMAP32[i]);
  for (i = 0; i < 65; i += 8) emit(0, "TAB vtb64 %d %d %d", i, VTBv17_IMAP64[i], VTBv17_QMAP64[i]);
  for (i = 0; i < 129; i += 16) emit(0, "TAB vtb128 %d %d %d", i, VTBv17_IMAP128[i], VTBv17_QMAP128[i]);
  for (i = 0; i < 120; i += 7) emit(0, "TAB filt %d %d %d", i, PPSv17_ICOFFS[i], PPSv17_QCOFFS[i]);
  emit(0, "TAB CFG %08x %08x %08x %08x %08x %08x %08x %08x", u(V17TX_CFG[0]), u(V17TX_CFG[1]), u(V17TX_CFG[2]), u(V17TX_CFG[3]), u(V17TX_CFG[4]), u(V17TX_CFG[5]), u(V17TX_CFG[6]),
       u(V17TX_CFG[7]));
  emit(0, "TAB CTL %08x %08x %08x %08x %08x", u(V17TX_CTL[0]), u(V17TX_CTL[1]), u(V17TX_CTL[2]), u(V17TX_CTL[3]), u(V17TX_CTL[4]));
  cov_hit(CV_TAB);
  {
    FaxVmiV17 m;
    memset(&m, 0, sizeof m);
    for (i = -3; i < 14; i++) {
      const char *s = "unset";
      v17tx_message(&m, i, &s);
      if (i == 10) {
        /* the blob reads the first word of V17TX_CTL (0x3840), the port stores NULL: one common line, the blob's value is checked */
        if (SIDE_BLOB && (unsigned)(uintptr_t)s != u(V17TX_CTL[0])) die3("blob v17tx_message(10) != V17TX_CTL[0]");
        if (!SIDE_BLOB && s != NULL) die3("port v17tx_message(10) != NULL");
        emit(1, "MSG %d = (CTL word 0 in the blob, NULL in the port)", i);
        cov_hit(CV_MSG_OOB);
      } else {
        emit(0, "MSG %d = %s", i, s ? s : "NULL");
        if (i >= 0 && i <= 9) cov_hit(CV_MSG); else cov_hit(CV_MSG_NEG);
      }
    }
    for (i = 0; i < 11; i++) {
      const char *s = 0;
      v17tx_message(&m, -256 + i, &s); /* a negative index whose low byte is a valid index: NULL (unsigned compare) */
      emit(0, "MSG %d = %s", -256 + i, s ? s : "NULL");
      s = 0;
      v17tx_message(&m, 0x100 + i, &s);
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
    V17TX_create(NULL, w);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  /* a failed allocation: the blob and the port crash or survive alike; the outcome is one line.  Exception: malloc #4 is the SGD object inside SGD_create (R18): the blob stores through the NULL
   * pointer (SIGSEGV), the port's SGD_create returns NULL (a deliberate guard of src/fax/fax_fifo_sgd.c), so V17TX_create survives; the line is common, the sides are checked */
  if (failk == 4) {
    if (SIDE_BLOB != (WIFSIGNALED(st) != 0)) die3("probe #4: the blob must crash and the port survive");
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
    V17Tx *o;
    if (freopen("/dev/null", "w", stderr) == NULL) _exit(9); /* the garbage free() is a harness violation (exit 3) in the child: no noise */
    o = (V17Tx *)sysdep_malloc(sizeof(V17Tx));
    o->hdx = NULL;
    o->bundle = NULL;
    V17TX_create(o, NULL);
    _exit(7);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  /* the new bundle is uninitialised heap: FPM_PPS_init reads its garbage `taps` and then frees / clears garbage pointers.  WHICH garbage word it sees depends on the host layout (the blob: the
   * recorder's sysdep_free refuses the pointer, exit 3; x86-64: the memset of the "kept" buffer faults), so the common line only says that the create ran into the garbage */
  if (WIFSIGNALED(st) || WEXITSTATUS(st) == 3) emit(1, "PROBE create of a supplied object without bundle: ran into the garbage (signal or refused free)");
  else emit(1, "PROBE create of a supplied object without bundle: exit %d", WEXITSTATUS(st));
  cov_hit(CV_L_SUPNOBUNDLE);
}
/* the re-create of a live engine with MORE FIFO frames (cfg word 5): FIFO_create keeps the buffer of the old FIFO, the new size is larger: the FIFO writes beyond its buffer (blob bug) */
static void probe_fifo_grow(int from, int to)
{
  int st = 0;
  pid_t pid;
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    int w[8];
    V17Tx *o;
    unsigned short blk[200];
    int k;
    make_cfg(w, 1, 0);
    w[5] = from;
    o = V17TX_create(NULL, w);
    w[5] = to;
    V17TX_create(o, w);
    for (k = 0; k < 200; k++) blk[k] = (unsigned short)k;
    FIFO_write(o->hdx->fifo, blk, (unsigned short)(48 * to));
    _exit(redzones_dirty() ? 11 : 12);
  }
  if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
  if (WIFSIGNALED(st)) emit(1, "PROBE fifo frames %d -> %d: signal %d", from, to, WTERMSIG(st));
  else emit(1, "PROBE fifo frames %d -> %d: %s", from, to, WEXITSTATUS(st) == 11 ? "wrote outside a block" : "clean");
  cov_hit(CV_L_FIFOGROW);
}
static void run_some(V17Tx *o)
{
  int i, calls = 3 + (int)rmod(6);
  for (i = 0; i < calls; i++) {
    unsigned short n = 48;
    int ret;
    sym_fill(g_inb, 64, rate_bits(o->hdx->rate));
    ret = V17TX_modem(o, g_inb, g_out, &n);
    emit(0, "life run %d ret=%08x n=%d out=%016llx", i, u(ret), n, fnv_n(g_out, n + 4));
  }
  d_tx("ran", o);
  cov_hit(CV_L_RUN);
}
static void scen1(void)
{
  int trial, k;
  register_tables();
  for (trial = 0; trial < 300; trial++) {
    static const int rates[8] = {0, 1, 2, 3, 0x1111, 0x2580, 2, 3};
    int rate = rates[trial % 8];
    int mode = (trial / 8) % 7;
    int dbg = (trial / 56) & 1;
    int variant = (int)(rnd() & 63u);
    int w[8];
    V17Tx *o = NULL;
    make_cfg(w, rate, trial % 3 == 2 ? variant : trial % 3);
    dbg_level(dbg ? (trial % 37 == 11 ? 0x80000002u : 2u) : 0u);
    if (rate == 0) cov_hit(CV_L_R7200); else if (rate == 1 || rate == 0x2580) cov_hit(CV_L_R9600); else if (rate == 2) cov_hit(CV_L_R12000); else if (rate == 3) cov_hit(CV_L_R14400);
    else cov_hit(CV_L_RBAD);
    if (dbg) cov_hit(CV_L_DBG);
    if ((w[5] & 0xffff) == 2) cov_hit(CV_L_FIFO2);
    if (w[6]) cov_hit(CV_L_FLAG);
    switch (mode) {
      case 0:
        stack_poison();
        o = V17TX_create(NULL, w);
        settle(o);
        cov_hit(CV_L_NEW);
        break;
      case 1:
        stack_poison();
        o = V17TX_create(NULL, NULL);
        settle(o);
        cov_hit(CV_L_NEW);
        cov_hit(CV_L_NULLCFG);
        break;
      case 2:
      case 5: {
        int w0[8];
        V17Tx *s;
        make_cfg(w0, rate == 1 ? 1 : 0, 0);
        s = V17TX_create(NULL, w0);
        settle(s);
        d_tx("first", s);
        if (mode == 2) { /* hdx dropped: the create allocates a new one (the old pieces are freed here) */
          FIFO_delete(s->hdx->fifo);
          SGD_delete(s->hdx->sgd);
          sysdep_free(s->hdx);
          s->hdx = NULL;
        }
        stack_poison();
        if (mode == 5) w[5] = 1; /* a larger FIFO would overrun the buffer of the live engine (blob bug, probed below) */
        o = V17TX_create(s, mode == 5 ? w : w0);
        if (o != s) die3("supplied object not returned");
        if (mode == 2) { o->hdx->s12 = 0x3c3c; o->hdx->s1c = 0x5a5a; }
        cov_hit(mode == 2 ? CV_L_SUP_NOHDX : CV_L_SUP_EMPTY);
        if (mode == 5) cov_hit(CV_L_RATECHG);
        break;
      }
      case 3: {
        int w0[8];
        V17Tx *s;
        make_cfg(w0, rate == 1 ? 1 : 0, 0);
        s = V17TX_create(NULL, w0);
        settle(s);
        d_tx("first", s);
        w0[7] = w[7];
        o = V17TX_create(s, w0);
        if (o != s) die3("supplied object not returned");
        cov_hit(CV_L_SUP_LIVE);
        cov_hit(CV_L_NOREALLOC);
        break;
      }
      case 4: {
        V17Tx *s;
        make_cfg(w, rate == 1 ? 1 : 0, variant & ~4);
        s = V17TX_create(NULL, w);
        settle(s);
        d_tx("first", s);
        s->cfg.w[2] = 777;
        s->cfg.w[7] = (int)rnd();
        o = V17TX_create(s, s->cfg.w);
        cov_hit(CV_L_RECREATE);
        break;
      }
      default: { /* a re-create with another config word set on a live engine */
        int w0[8], w1[8];
        V17Tx *s;
        make_cfg(w0, 1, 0);
        s = V17TX_create(NULL, w0);
        settle(s);
        d_tx("first", s);
        make_cfg(w1, rate == 0 ? 0 : rate == 1 ? 1 : 3, 2);
        o = V17TX_create(s, w1);
        cov_hit(CV_L_SUP_LIVE);
        break;
      }
    }
    dbg_level(0);
    d_tx("created", o);
    if (trial & 1) run_some(o);
    V17TX_delete(o);
    cov_hit(CV_L_DEL);
    trial_end("life");
  }
  probe_supplied();
  probe_fifo_grow(1, 2);
  probe_fifo_grow(2, 1);
  probe_fifo_grow(2, 2);
  for (k = 0; k < 12; k++) {
    probe_create(k, k & 3);
    cov_hit(CV_L_PROBE);
  }
  {
    V17Tx *s;
    int w0[8];
    make_cfg(w0, 1, 0);
    s = V17TX_create(NULL, w0);
    settle(s);
    g_fail_malloc = 0; /* a supplied live engine allocates nothing (the circular buffer is allocated with a new bundle only) and the PPS buffers are kept: no malloc is reached */
    V17TX_create(s, NULL); /* default cfg: rate 14400 */
    d_tx("recreate one malloc", s);
    V17TX_delete(s);
    trial_end("life fail");
  }
}

/* ============================================================================================ scenario 2: transmit streams */
static void observe(const V17Tx *o, int *prev)
{
  int s = o->hdx->state;
  if (s == 6) cov_hit(CV_S_BRIDGE);
  if (*prev == 7 && s == 8) cov_hit(CV_S_TO_DATA);
  if (*prev != 11 && s == 11) cov_hit(CV_S_TO_IDLE);
  *prev = s;
}
static void scen2(void)
{
  int trial;
  register_tables();
  dbg_level(0);
  for (trial = 0; trial < 64; trial++) {
    int rate = trial & 3;
    int variant = (trial / 4) % 6;
    int pol = (trial / 24) % 3;
    int w[8], i, calls, tot = 0, prev = -1, frm = 48;
    unsigned long long oh = 0;
    V17Tx *o;
    int dbg = trial % 4 == 2 || trial % 9 == 4;
    int shortt = trial % 5 == 1;
    if (trial % 17 == 5) rate = 0x1111;
    make_cfg(w, rate, (variant == 0 ? 1 : variant == 1 ? 1 | 2 : variant == 2 ? 1 | 4 : variant == 3 ? 0 : variant == 4 ? 1 | 2 | 4 | 8 | 16 : 8) | (shortt ? 32 : 0));
    if (rate == 0) cov_hit(CV_S_R7200); else if (rate == 1) cov_hit(CV_S_R9600); else if (rate == 2) cov_hit(CV_S_R12000); else if (rate == 3) cov_hit(CV_S_R14400); else cov_hit(CV_S_BADRATE);
    if (w[5] == 2) cov_hit(CV_S_BIGFIFO);
    if (shortt) cov_hit(CV_S_SHORT);
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_S_DBG);
    stack_poison();
    o = V17TX_create(NULL, trial % 11 == 7 ? NULL : w);
    settle(o);
    d_tx("created", o);
    if (trial % 11 == 7) rate = 3;
    calls = 380 + (int)rmod(300);
    for (i = 0; i < calls; i++) {
      int n = pol == 0 ? frm : pol == 1 ? (rpct(70) ? frm : (int)rmod((unsigned)(frm + 9))) : (rpct(50) ? frm : rpct(50) ? 2 * frm : (int)rmod((unsigned)frm));
      unsigned short nn;
      int ret, k;
      if (rpct(3)) n = 0;
      if (n == 0) cov_hit(CV_S_N0);
      else if (n == frm) cov_hit(CV_S_NFULL);
      else if (n > frm) cov_hit(CV_S_NLONG);
      else cov_hit(CV_S_NSHORT);
      sym_fill(g_inb, 4096, rate_bits(rate));
      for (k = 0; k < 8192; k++) g_out[k] = 0x7777;
      nn = (unsigned short)n;
      ret = V17TX_modem(o, g_inb, g_out, &nn);
      emit(0, "tx call %d n=%d ret=%08x outn=%d in=%016llx out=%016llx", i, n, u(ret), nn, fnv_n(g_inb, 80), fnv_n(g_out, 8192));
      oh ^= fnv_n(g_out, nn);
      tot += nn;
      d_tx("tx", o);
      observe(o, &prev);
      if (o->st.b[0] == 9) cov_hit(CV_S_OVER);
      if (o->st.b[0] == 8) cov_hit(CV_S_UNDER);
      if (o->st.b[0] == 2) cov_hit(CV_S_DATASTART2);
      if (o->st.b[0] == 3) cov_hit(CV_S_DATASTART3);
      if (o->st.b[0] == 4) cov_hit(CV_S_DATASTART4);
      if (o->st.b[0] == 5) cov_hit(CV_S_DATASTART5);
      if (rpct(5)) {
        FaxV17Status st;
        FaxV17Ctl ctl;
        int modeb;
        memset(&st, 0x55, sizeof st);
        emit(0, "tx status ret=%d", V17TX_status(o, &st));
        prt_status("tx status", &st);
        cov_hit(CV_S_STATUS);
        memset(&ctl, 0, sizeof ctl);
        ctl.w4 = (int)rnd();
        ctl.w8 = rpct(50) ? 0x4000 : (int)(rnd() & 0xffff);
        ctl.w10 = rpct(60) ? o->cfg.w[6] : (int)(rnd() & 3u);
        ctl.b0c = (unsigned char)(rpct(50) ? 4 : 0);
        ctl.b0d = (unsigned char)((rpct(i > 150 ? 35 : 10) ? 0x10 : 0) | (rpct(3) ? 2 : 0));
        if (ctl.b0d & 0x10) cov_hit(CV_S_PEND);
        cov_hit(CV_S_GAIN);
        if (ctl.w10 != o->cfg.w[6]) cov_hit(CV_S_FLAGCTL);
        if (ctl.b0d & 2) cov_hit(CV_S_RECREATE);
        modeb = o->hdx->nocarr != 4; /* SetTxModeV17 has run: a re-create would leave the pulse shaper on the 5 entry map with trellis codes: repaired below */
        emit(0, "tx control ret=%d", V17TX_control(o, &ctl));
        if ((ctl.b0d & 2) && modeb) SetTxModeV17(o, o->hdx->rate);
        d_tx("ctl", o);
      }
      if (o->hdx->pend && rpct(25)) {
        FaxV17Ctl ctl;
        memset(&ctl, 0, sizeof ctl);
        ctl.w8 = 0x4000;
        ctl.w4 = o->cfg.w[2];
        ctl.w10 = o->cfg.w[6];
        V17TX_control(o, &ctl); /* pend off again */
      }
    }
    emit(0, "tx trial %d rate %d variant %d: samples=%d outhash=%016llx", trial, rate, variant, tot, oh);
    if (trial % 8 == 0) emit(1, "TX trial %d rate %d: %d samples", trial, rate, tot);
    dbg_level(0);
    V17TX_delete(o);
    trial_end("tx run");
  }
}

/* ============================================================================================ scenario 3: handlers */
static V17TxHdxFn pick_fn(int sel)
{
  switch (sel) {
    case 0: return TxHdxStartV17;
    case 1: return TxHdxSilenceV17;
    case 2: return TxHdxTEP_V17;
    case 3: return TxHdxABV17;
    case 4: return TxHdxBridgeV17;
    case 5: return TxHdxEQCondV17;
    case 6: return TxHdxSCR1V17;
    case 7: return TxHdxDataV17;
    default: return TxHdxIdleV17;
  }
}
static short g_big[0x10100];
static void scen3(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 9000; trial++) {
    static const int sts[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, -1, 12, 0x7fff};
    static const int cnts[9] = {0, 1, 5, 48, 64, -1, -32768, 400, 0x7fff};
    static const int ns[8] = {0, 1, 3, 8, 48, 64, 70, 6};
    int fnsel = trial % 9, rate = (trial / 9) & 3, stsel = (trial / 36) % 15, cn = (trial / 540) % 9, fill = (trial / 4860) % 2 + (trial % 3 == 2 ? 1 : 0), pend = (int)(rnd() & 1u);
    int ns_i = (int)(rnd() % 8u), dbg = trial % 29 == 3, modeb = (int)(rnd() & 1u), flag = (int)(rnd() % 3u == 0);
    int w[8], i, ret;
    V17Tx *o;
    unsigned short n;
    int nf, big = 0;
    short *base = g_inb;
    make_cfg(w, rate, (trial % 4 == 3 ? 1 : 0) | (flag ? 32 : 0));
    o = V17TX_create(NULL, w);
    settle(o);
    cov_hit(modeb ? CV_H_MODEB : CV_H_MODEA);
    if (flag) cov_hit(CV_H_FLAG);
    if (modeb) {
      mode_b(o, rate);
      if (o->bundle->encmode == 0) cov_hit(CV_H_ENC0);
      if (o->bundle->encmode == 1) cov_hit(CV_H_ENC1);
      if (o->bundle->encmode == 2) cov_hit(CV_H_ENC2);
    } else {
      o->bundle->encmode = (unsigned short)(rnd() & 1u); /* mode A: dif / abs only (the 5 entry map of the pulse shaper) */
      cov_hit(o->bundle->encmode ? CV_H_ENC1 : CV_H_ENC0);
    }
    nf = fill == 0 ? 0 : fill == 1 ? 3 : 48;
    big = fnsel == 7 && pend && nf > 0 && trial % 7 == 0;
    if (big) base = g_big; /* the count is above 0x7fff: FIFO_read fills the whole count (a block of up to 64 K shorts) */
    sym_fill(base, big ? 0x10100 : 4096, rate_bits(rate));
    for (i = 0; i < 16; i++) base[i] = (short)(0x6b00 + i); /* a sentinel in front of the block the handler gets (base + 16) */
    if (nf) FIFO_write(o->hdx->fifo, (const unsigned short *)(base + 16), (unsigned short)nf);
    cov_hit(nf ? CV_H_FIFOF : CV_H_FIFO0);
    o->hdx->state = (short)sts[stsel];
    o->hdx->count = (short)cnts[cn];
    o->hdx->pend = pend;
    if (pend) cov_hit(CV_H_PEND);
    o->st.w = (int)rnd();
    if (stsel > 11) cov_hit(CV_H_STATEDEF);
    if (cnts[cn] == 0) cov_hit(CV_H_COUNT0);
    if (cnts[cn] < 0) cov_hit(CV_H_COUNTNEG);
    n = (unsigned short)ns[ns_i];
    if (fnsel == 7 && pend && nf > 0 && trial % 7 == 0) n = (unsigned short)(trial % 14 == 0 ? 0x8000 + rmod(0x7fff) : 0xffff); /* a count above 0x7fff: the under-run test is an UNSIGNED 16 bit compare; with pend only the FIFO's symbols are processed */
    if (n == 0) cov_hit(CV_H_N0);
    if (cnts[cn] > ns[ns_i]) cov_hit(CV_H_COUNTBIG);
    if (cnts[cn] > 0 && cnts[cn] < ns[ns_i]) cov_hit(CV_H_NSMALL);
    sym_fill(base + 16, 4000, rate_bits(rate));
    for (i = 0; i < 8192; i++) g_out[i] = 0x7777;
    if (fnsel == 7 && trial % 5 == 0) { /* Data with odd rate words (state byte 7) */
      static const int odd[5] = {4, -1, 0x7fff, 5, 3};
      int st = o->hdx->state;
      o->hdx->rate = (short)odd[(trial / 5) % 5];
      if (o->hdx->rate < 0 || o->hdx->rate > 3) cov_hit(CV_H_RATEODD);
      if (cnts[cn] == 0) o->hdx->count = 1;
      /* pend would run TxNextStateV17 cases 5 / 6 / 8, which index V17TX_PATTERN_SCR1 and V17TX_SYM_SIZE with the rate word (out of the table: layout dependent) */
      if ((o->hdx->rate < 0 || o->hdx->rate > 3) && (st == 5 || st == 6 || st == 8)) o->hdx->state = 7;
    }
    dbg_level(dbg ? 2 : 0);
    d_tx("pre", o);
    stack_poison();
    o->hdx->fn = pick_fn(fnsel);
    switch (fnsel) {
      case 0: ret = TxHdxStartV17(o, base + 16, g_out, &n); cov_hit(CV_H_START); break;
      case 1: ret = TxHdxSilenceV17(o, base + 16, g_out, &n); cov_hit(CV_H_SILENCE); break;
      case 2: ret = TxHdxTEP_V17(o, base + 16, g_out, &n); cov_hit(CV_H_TEP); break;
      case 3: ret = TxHdxABV17(o, base + 16, g_out, &n); cov_hit(CV_H_AB); break;
      case 4: ret = TxHdxBridgeV17(o, base + 16, g_out, &n); cov_hit(CV_H_BRIDGE); break;
      case 5: ret = TxHdxEQCondV17(o, base + 16, g_out, &n); cov_hit(CV_H_EQ); break;
      case 6: ret = TxHdxSCR1V17(o, base + 16, g_out, &n); cov_hit(CV_H_SCR1); break;
      case 7:
        ret = TxHdxDataV17(o, base + 16, g_out, &n);
        cov_hit(CV_H_DATA);
        if ((o->st.b[0] == 8) && !pend) cov_hit(CV_H_UNDER);
        if (pend && nf < ns[ns_i]) cov_hit(CV_H_UNDERPEND);
        break;
      default:
        if (nf) cov_hit(CV_H_IDLEFIFO);
        ret = TxHdxIdleV17(o, base + 16, g_out, &n);
        cov_hit(CV_H_IDLE);
        break;
    }
    dbg_level(0);
    emit(0, "hdx trial %d fn %d rate %d st %d cnt %d fill %d pend %d n %d: ret=%d n=%d in=%016llx out=%016llx", trial, fnsel, rate, sts[stsel], cnts[cn], fill, pend, ns[ns_i], ret, n,
         fnv_n(base, 116), fnv_n(g_out, 1024));
    d_tx("post", o);
    V17TX_delete(o);
    trial_end("hdx");
  }
}

/* ============================================================================================ scenario 4: next state, mode, modulator, no carrier */
static void scen4(void)
{
  int trial, st, rate, dbg;
  register_tables();
  for (trial = 0; trial < 15 * 4 * 2 * 2 * 3; trial++) {
    static const int sts[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, -1, 12, 0x7fff};
    int w[8];
    V17Tx *o;
    int flag = (trial / 60) & 1, modeb = (trial / 120) % 3 == 2;
    st = sts[trial % 15];
    rate = (trial / 15) & 3;
    dbg = (trial / 120) & 1;
    make_cfg(w, rate, flag ? 32 : 0);
    o = V17TX_create(NULL, w);
    settle(o);
    if (modeb) mode_b(o, rate);
    else o->bundle->encmode = (unsigned short)((trial / 30) & 1);
    o->hdx->state = (short)st;
    o->hdx->count = (short)(int)(rnd() & 0x7fu);
    o->st.w = (int)rnd();
    o->bundle->smc.x = (short)(rnd() & 3u);
    sym_fill(g_inb, 64, rate_bits(rate));
    dbg_level(dbg ? 2 : 0);
    if (dbg) cov_hit(CV_N_DBG);
    d_tx("pre", o);
    stack_poison();
    TxNextStateV17(o);
    dbg_level(0);
    d_tx("next", o);
    switch (st) {
      case 0: cov_hit(CV_N_C0); break;
      case 1: cov_hit(CV_N_C1); break;
      case 2: cov_hit(CV_N_C2); break;
      case 3: cov_hit(CV_N_C3); break;
      case 4: cov_hit(CV_N_C4); break;
      case 5: cov_hit(flag ? CV_N_C5S : CV_N_C5); break;
      case 6: cov_hit(CV_N_C6); break;
      case 7: cov_hit(CV_N_C7); break;
      case 8: cov_hit(CV_N_C8); break;
      case 9: cov_hit(CV_N_C9); break;
      case 10: cov_hit(CV_N_C10); break;
      case 11: cov_hit(CV_N_C11); break;
      default: cov_hit(CV_N_DEFAULT); break;
    }
    /* the machine after the transition: one handler call moves the state on again */
    if (o->hdx->fn && st >= 0 && st <= 11) {
      unsigned short n = (unsigned short)(1 + rmod(60));
      int ret;
      int k;
      for (k = 0; k < 4096; k++) g_out[k] = 0x7777;
      ret = o->hdx->fn(o, g_inb, g_out, &n);
      emit(0, "next+handler ret=%d n=%d out=%016llx", ret, n, fnv_n(g_out, 1024));
      d_tx("after", o);
    }
    V17TX_delete(o);
    trial_end("next");
  }
  /* SetTxModeV17 */
  for (trial = 0; trial < 96; trial++) {
    int w[8];
    V17Tx *o;
    int r = trial & 3;
    make_cfg(w, trial % 5 == 0 ? 3 : r, trial % 7 == 0 ? 32 : 0);
    o = V17TX_create(NULL, w);
    settle(o);
    o->bundle->sdm.state = rnd(); /* the scrambler register survives */
    o->bundle->smc.x = (short)rnd();
    o->bundle->smc.y = (short)rnd();
    o->bundle->smc.y2 = (short)rnd();
    o->bundle->smc.st = (short)rnd();
    o->bundle->smc.w10 = (unsigned short)rnd();
    o->bundle->smc.tcmMask = (unsigned short)rnd();
    o->bundle->encmode = (unsigned short)(rnd() & 1u);
    o->st.w = (int)rnd();
    d_tx("mpre", o);
    stack_poison();
    SetTxModeV17(o, (short)r);
    cov_hit(CV_N_SETMODE);
    d_tx("mode", o);
    if (trial % 3 == 0) { /* twice: the maps and the scrambler register carry over */
      SetTxModeV17(o, (short)((r + 1) & 3));
      d_tx("mode2", o);
    }
    V17TX_delete(o);
    trial_end("setmode");
  }
  {
    static const int bad[6] = {4, 5, -1, -2, 7, 3};
    int e;
    for (e = 0; e < 6; e++) {
      int w[8];
      V17Tx *o;
      make_cfg(w, 0, 0);
      o = V17TX_create(NULL, w);
      settle(o);
      o->st.w = e & 1 ? 0x11223344 : 0;
      o->bundle->sdm.state = rnd();
      o->bundle->smc.tcmMask = (unsigned short)rnd();
      o->bundle->smc.x = (short)rnd();
      o->bundle->smc.y = (short)rnd();
      o->bundle->smc.y2 = (short)rnd();
      o->bundle->smc.st = (short)rnd();
      o->bundle->smc.w10 = (unsigned short)rnd();
      o->bundle->encmode = (unsigned short)(rnd() & 1u);
      SetTxModeV17(o, (short)bad[e]);
      if (bad[e] >= 0 && bad[e] <= 3) {
        emit(0, "setmode %d st=%08x", bad[e], u(o->st.w));
      } else {
        /* the table index is out of the table: the bits per symbol the blob reads are the bytes after V17TX_SYM_SIZE (the scrambler width and the SGD bits are host dependent here): the flags, the
         * encoder state, the encoder select, the scrambler register and the no-carry code are common */
        const FaxSmcV17 *c = &o->bundle->smc;
        emit(0, "setmode %d st=%08x smc w0=%04x w2=%04x x=%d y=%d y2=%d st=%d w10=%04x mode=%u reg=%08x nocarr=%d", bad[e], u(o->st.w), c->tcmMask, c->w2, c->x, c->y, c->y2, c->st, c->w10,
             (unsigned)o->bundle->encmode, o->bundle->sdm.state, o->hdx->nocarr);
        cov_hit(CV_N_SETMODE_BAD);
      }
      V17TX_delete(o);
      trial_end("setmode bad");
    }
  }
  for (trial = 0; trial < 600; trial++) {
    int w[8], n, k, ret;
    V17Tx *o;
    int tmode = trial % 3; /* 0: nocarrier, 1: moddata, 2: both */
    rate = (trial >> 1) & 3;
    make_cfg(w, rate, trial % 5 == 4 ? 2 : 0);
    o = V17TX_create(NULL, w);
    settle(o);
    if (trial % 2 == 1) {
      SetTxModeV17(o, (short)rate);
      o->bundle->encmode = (unsigned short)(trial / 2 % 3); /* dif / abs / trellis */
      o->bundle->smc.x = (short)rnd();
      o->bundle->smc.y = (short)(rnd() & 3u);
      o->bundle->smc.y2 = (short)(rnd() & 3u);
      o->bundle->smc.st = (short)(rnd() & 7u);
    } else {
      o->bundle->encmode = (unsigned short)(trial / 2 & 1);
    }
    n = (int)(trial % 7 == 0 ? rmod(6) : rmod(120));
    if (trial % 4 == 0) {
      o->bundle->circ.wr = (short)rmod((unsigned)o->bundle->circ.lim);
      o->bundle->circ.rd = (short)rmod((unsigned)o->bundle->circ.lim);
    }
    if (trial % 9 == 3) { /* TxNoCarrierV17 with small / non-positive limits (the wrap test is a signed 16 bit compare) */
      static const short lims[4] = {0, 1, 2, -3};
      o->bundle->circ.lim = lims[(trial / 9) & 3];
      o->bundle->circ.wr = 0;
      o->bundle->circ.rd = 0;
      tmode = 0;
    }
    if (n + 2 > o->bundle->circ.lim) cov_hit(CV_N_WRAP);
    sym_fill(g_inb, 140, rate_bits(rate));
    for (k = 0; k < 8192; k++) g_out[k] = 0x7777;
    d_tx("mpre", o);
    if (tmode >= 1) {
      ret = ModDataV17(o, g_inb, g_out, (unsigned short)n);
      emit(0, "moddata trial %d n=%d ret=%d out=%016llx", trial, n, ret, fnv_n(g_out, 1024));
      cov_hit(o->bundle->encmode == 0 ? CV_N_MOD0 : o->bundle->encmode == 1 ? CV_N_MOD1 : CV_N_MOD2);
    }
    if (tmode != 1) {
      ret = TxNoCarrierV17(o, g_inb, g_out, (unsigned short)n);
      emit(0, "nocarrier trial %d n=%d ret=%d out=%016llx", trial, n, ret, fnv_n(g_out, 1024));
      cov_hit(CV_N_NOCAR);
    }
    d_tx("mpost", o);
    if (trial % 3 == 0) { /* both in a row: the PPS history / accumulator carry over */
      ret = ModDataV17(o, g_inb, g_out, (unsigned short)(n / 2));
      emit(0, "moddata2 ret=%d out=%016llx", ret, fnv_n(g_out, 1024));
      ret = TxNoCarrierV17(o, g_inb, g_out, (unsigned short)(n / 3));
      emit(0, "nocarrier2 ret=%d out=%016llx", ret, fnv_n(g_out, 1024));
      d_tx("mpost2", o);
    }
    V17TX_delete(o);
    trial_end("mod");
  }
  {
    /* a long run of the pulse shaper on random symbols (the encoder fills circ.buf; a phase step 0 .. 2 and a random read index): the filter sums land on rounding boundaries often enough to see
     * an offset of one in a sum.  Every rate (its constellation map), every encoder. */
    int e;
    for (e = 0; e < 8; e++) {
      int w[8], blk;
      V17Tx *o;
      unsigned long long oh = 0;
      make_cfg(w, e & 3, 0);
      o = V17TX_create(NULL, w);
      settle(o);
      SetTxModeV17(o, (short)(e & 3));
      o->bundle->encmode = (unsigned short)(e % 3);
      o->bundle->pps.cfg.step = (short)(e >> 2);
      if (e >> 2) cov_hit(CV_N_STEP);
      cov_hit(CV_N_LONG);
      for (blk = 0; blk < 2200; blk++) {
        int q, nn = 1 + (int)rmod(40);
        for (q = 0; q < 64; q++) g_ibuf[q] = (short)(rnd() & 0xffffu);
        if (blk % 5 == 0) o->bundle->encmode = (unsigned short)((blk / 5 + e) % 3);
        o->bundle->smc.y2 = (short)(o->bundle->smc.y2 & 3);
        o->bundle->smc.st = (short)(o->bundle->smc.st & 7);
        if (blk % 11 == 0) {
          oh = oh * 1099511628211ULL ^ (unsigned long long)TxNoCarrierV17(o, g_ibuf, g_out, (unsigned short)nn) ^ fnv_n(g_out, 400);
        } else {
          oh = oh * 1099511628211ULL ^ (unsigned long long)ModDataV17(o, g_ibuf, g_out, (unsigned short)nn) ^ fnv_n(g_out, 400);
        }
      }
      emit(0, "long run %d: %016llx", e, oh);
      d_tx("long", o);
      V17TX_delete(o);
      trial_end("long");
    }
  }
}

/* ============================================================================================ scenario 5: dispatcher, control, status */
static int g_fake_calls, g_fake_cons[16], g_fake_ret[16];
static short *g_fake_in, *g_fake_out;
static V17TxHdx g_hdx_b;   /* a copy of the hdx block the dispatcher re-reads every iteration (V17TX_modem takes the handler from o->hdx each time round) */
static V17TxHdx *g_hdx_orig;
static int fake_tx_b(V17Tx *o, short *in, short *out, unsigned short *n);
static int fake_tx(V17Tx *o, short *in, short *out, unsigned short *n)
{
  int k = g_fake_calls++;
  if (g_hdx_orig && k == 1) {
    g_hdx_b = *o->hdx;
    g_hdx_b.fn = fake_tx_b;
    o->hdx = &g_hdx_b;
  }
  int c = g_fake_cons[k & 15], r = g_fake_ret[k & 15];
  (void)o;
  emit(0, "fake tx call %d: in+%d out+%d n=%d -> consume %d ret %d", k, (int)(in - g_fake_in), (int)(out - g_fake_out), (short)*n, c, r);
  *n = (unsigned short)(*n - c);
  return r;
}
static int fake_tx_b(V17Tx *o, short *in, short *out, unsigned short *n)
{
  int r = fake_tx(o, in, out, n);
  emit(0, "fake tx B (swapped hdx) -> %d", r);
  return r + 1;
}
static void scen5(void)
{
  int t, i, f;
  register_tables();
  for (t = 0; t < 600; t++) {
    int w[8], ret, rate = t & 3;
    V17Tx *o;
    unsigned short n;
    int nn = (int)rmod(60);
    make_cfg(w, rate, 0);
    o = V17TX_create(NULL, w);
    settle(o);
    for (i = 0; i < 16; i++) {
      g_fake_cons[i] = i >= 14 ? 1000 : (int)rmod(23 + 2 * (unsigned)rate) - (t % 3 == 0 ? 0 : 1);
      g_fake_ret[i] = (int)rmod(41) - (t % 4 == 0 ? 20 : 0);
      if (g_fake_ret[i] < 0) cov_hit(CV_D_NEGRET);
    }
    g_fake_calls = 0;
    g_fake_in = &g_inb[0];
    g_fake_out = &g_out[0];
    sym_fill(g_inb, 64, rate_bits(rate));
    o->hdx->fn = fake_tx;
    o->hdx->pend = t & 4 ? 1 : 0;
    cov_hit(o->hdx->pend ? CV_D_PEND : CV_D_NOPEND);
    if (t % 5 < 2) FIFO_write(o->hdx->fifo, (const unsigned short *)g_inb, (unsigned short)(1 + t % 6));
    o->st.w = (int)rnd();
    n = (unsigned short)nn;
    g_hdx_orig = t % 3 == 1 ? o->hdx : NULL;
    ret = V17TX_modem(o, g_inb, &g_out[200], &n);
    if (g_hdx_orig) o->hdx = g_hdx_orig;
    g_hdx_orig = NULL;
    emit(0, "txdisp t=%d nn=%d: ret=%08x n=%d calls=%d st=%08x", t, nn, u(ret), n, g_fake_calls, u(o->st.w));
    d_fifo("disp", o->hdx->fifo);
    if (o->st.b[0] == 9) cov_hit(CV_D_OVER);
    cov_hit(CV_D_FAKE);
    V17TX_delete(o);
    trial_end("tx disp");
  }
  for (f = 0; f < 512; f++) {
    int w[8], k;
    V17Tx *o;
    FaxV17Ctl ctl;
    unsigned short n = 12;
    make_cfg(w, f & 3, 0);
    o = V17TX_create(NULL, w);
    settle(o);
    o->hdx->pend = (int)(rnd() & 3u);
    o->hdx->state = (short)(f & 3);
    memset(&ctl, 0, sizeof ctl);
    ctl.w4 = 100 + f;
    ctl.w8 = f % 5 == 0 ? 0x4000 : f % 5 == 1 ? 0x7fffffff : (int)rnd();
    ctl.w10 = f % 3 == 0 ? 0 : f % 3 == 1 ? 1 : (int)rnd();
    if (f < 256) {
      ctl.b0d = (unsigned char)f;
      ctl.b0c = (unsigned char)rnd();
    } else {
      ctl.b0c = (unsigned char)(f - 256);
      ctl.b0d = (unsigned char)(rnd() & 0xfdu);
    }
    emit(0, "txctl f=%d: ret=%d", f, V17TX_control(o, &ctl));
    cov_hit(CV_D_CTL);
    if (ctl.b0d & 0x10) cov_hit(CV_D_CTL_PEND);
    if (ctl.b0d & 2) cov_hit(CV_D_CTL_RECREATE);
    if (ctl.b0c & 4) cov_hit(CV_D_CTL_B0C);
    if (ctl.w10 != 0) cov_hit(CV_D_CTL_W10);
    cov_hit(CV_D_CTL_GAIN);
    d_tx("ctl", o);
    /* the gain shows in the modulated samples */
    sym_fill(g_inb, 64, rate_bits(f & 3));
    for (k = 0; k < 8192; k++) g_out[k] = 0x7777;
    o->hdx->state = 4;
    o->hdx->fn = TxHdxABV17;
    o->hdx->count = 40;
    emit(0, "txctl run ret=%d n=%d out=%016llx", TxHdxABV17(o, g_inb, g_out, &n), n, fnv_n(g_out, 1024));
    emit(0, "txctl null: ret=%d", V17TX_control(o, NULL));
    cov_hit(CV_D_CTL_NULL);
    V17TX_delete(o);
    trial_end("tx ctl");
  }
  for (t = 0; t < 200; t++) {
    int w[8];
    V17Tx *o;
    FaxV17Status st;
    make_cfg(w, t & 3, t % 4 == 2 ? 8 : 0);
    o = V17TX_create(NULL, w);
    settle(o);
    o->cfg.b[0x10] = (unsigned char)rnd();
    o->cfg.s[0] = (short)(rpct(50) ? (int)(t & 3) : (int)rnd());
    o->cfg.w[6] = (int)rnd();
    memset(&st, t % 3 == 0 ? 0xff : t % 3 == 1 ? 0x55 : 0x00, sizeof st);
    emit(0, "txstat t=%d ret=%d", t, V17TX_status(o, &st));
    prt_status("txstat", &st);
    cov_hit(CV_D_STAT);
    emit(0, "txstat null ret=%d", V17TX_status(o, NULL));
    cov_hit(CV_D_STAT_NULL);
    V17TX_delete(o);
    trial_end("tx stat");
  }
}

/* ============================================================================================ scenario 6: VMI wrappers */
static void d_vmi(const char *t, const FaxVmiV17 *m, int created)
{
  /* u4: the integer 1 after the init (blob and port), the bit buffer pointer afterwards; u10: the config block pointer after the init, its low half is zeroed by v17tx_create (print the half) */
  if (!created) emit(0, "%s VMI w0=%d u4=%lu s08=%d s0a=%d s0c=%d s0e=%d cfg=blk%u eng=%s", t, m->w0, (unsigned long)m->u4.init1, m->s08, m->s0a, m->s0c, m->s0e, blk_id_of(m->u10.cfg), m->eng ? "set" : "NULL");
  else emit(0, "%s VMI w0=%d s08=%d s0a=%d s0c=%d s0e=%d s10=%d eng=%s", t, m->w0, m->s08, m->s0a, m->s0c, m->s0e, m->u10.s10, m->eng ? "set" : "NULL");
}
static void scen6(void)
{
  int trial;
  register_tables();
  for (trial = 0; trial < 50; trial++) {
    FaxVmiV17 m;
    int supplied = trial % 5 == 3;
    int w[8], rate = trial % 6 == 5 ? 0x1111 : trial & 3, c, ret, i, calls, dbglev = trial % 4;
    unsigned short n, on;
    short *bitbuf, *out;
    V17Tx *user = NULL, *eng;
    unsigned long long oh = 0;
    unsigned int cret;
    if (trial == 9 || trial == 14) dbglev = (int)0x80000001u; /* unsigned level compare */
    bitbuf = (short *)sysdep_malloc(4096 * 2);
    out = (short *)sysdep_malloc(8192 * 2);
    memset(&m, trial & 2 ? 0xa5 : 0x5a, sizeof m);
    if (supplied) {
      int w0[8];
      make_cfg(w0, rate, 0);
      user = V17TX_create(NULL, w0);
      settle(user);
    }
    make_cfg(w, rate, supplied ? 0 : trial % 6 >= 3 ? 1 | 4 : 0);
    if (rate == 0) cov_hit(CV_V_R7200); else if (rate == 1) cov_hit(CV_V_R9600); else if (rate == 2) cov_hit(CV_V_R12000); else if (rate == 3) cov_hit(CV_V_R14400); else cov_hit(CV_V_RBAD);
    dbg_level((unsigned)dbglev);
    if (dbglev > 2) cov_hit(CV_V_INIT_DBG);
    stack_poison();
    ret = fax_init_vmi_v17tx(&m, (unsigned short)rate_word(rate), trial, user);
    dbg_level(0);
    cov_hit(CV_V_INIT);
    emit(0, "init ret=%d", ret);
    d_vmi("init", &m, 0);
    {
      /* the config block copy: the user word (a pointer's low half) is layout dependent; its equality with the user word is printed */
      V17TxCfg *cb = m.u10.cfg;
      V17TxCfg cc;
      memcpy(&cc, cb, sizeof cc);
      emit(0, "init cfg block %08x %08x %08x %08x %08x %08x %08x user=%d", u(cc.w[0]), u(cc.w[1]), u(cc.w[2]), u(cc.w[3]), u(cc.w[4]), u(cc.w[5]), u(cc.w[6]), cc.w[7] == (int)(uintptr_t)user);
    }
    m.u4.bits = bitbuf;
    stack_poison();
    cret = v17tx_create(&m, trial % 3 == 0 ? NULL : w); /* NULL cfg: the default rate 14400 */
    cov_hit(CV_V_CREATE);
    if (trial % 3 == 0) cov_hit(CV_V_CREATE_NULLCFG);
    if (supplied) cov_hit(CV_V_CREATE_SUP);
    eng = m.eng;
    settle(eng);
    emit(0, "v17tx_create ret=%u eng=%s", cret, eng == user && supplied ? "the supplied engine" : "the engine");
    d_vmi("created", &m, 1);
    d_tx("eng", m.eng);
    calls = 60 + (int)rmod(40);
    for (i = 0; i < calls; i++) {
      int k;
      n = 48;
      sym_fill(bitbuf, 64, rate_bits(((V17Tx *)m.eng)->hdx->rate));
      for (k = 0; k < 8192; k++) out[k] = 0x7777;
      on = 0xabcd;
      ret = v17tx_process(&m, out, &n, &on);
      emit(0, "v17tx_process %d ret=%08x n=%d outn=%d out=%016llx", i, u(ret), n, on, fnv_n(out, 8192));
      oh ^= fnv_n(out, on);
      cov_hit(CV_V_PROC);
    }
    emit(0, "vmi trial %d hash=%016llx", trial, oh);
    {
      FaxV17Status st;
      FaxV17Ctl ctl;
      int modeb;
      memset(&st, 0x55, sizeof st);
      emit(0, "v17tx_status ret=%d", v17tx_status(&m, &st));
      prt_status("st", &st);
      cov_hit(CV_V_STATUS);
      memset(&ctl, 0, sizeof ctl);
      ctl.w4 = 1234 + trial;
      ctl.w8 = 0x2000;
      ctl.w10 = trial & 1;
      ctl.b0c = 4;
      ctl.b0d = (unsigned char)(trial & 1 ? 0x12 : 0x10);
      modeb = ((V17Tx *)m.eng)->hdx->nocarr != 4;
      emit(0, "v17tx_control ret=%d", v17tx_control(&m, &ctl));
      if ((ctl.b0d & 2) && modeb) SetTxModeV17(m.eng, ((V17Tx *)m.eng)->hdx->rate); /* a re-create falls back to the 5 entry map: repair */
      emit(0, "v17tx_control null ret=%d", v17tx_control(&m, NULL));
      cov_hit(CV_V_CTL);
      d_tx("eng2", m.eng);
    }
    for (c = -2; c < 13; c++) {
      const char *s = "unset";
      v17tx_message(&m, c, &s);
      if (c != 10) emit(0, "vmi msg %d %s", c, s ? s : "NULL");
    }
    cov_hit(CV_V_MSG);
    v17tx_delete(&m);
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
  g_verbose = (e = getenv("R25_VERBOSE")) != NULL && *e == '1';
  if ((e = getenv("R25_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(e, NULL, 0);
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
