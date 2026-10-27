/*
 * R30 differential harness: the fax class 1 layer (fax_class1_create / _progress / _command / _delete, the state table class1_state_functions) and the entry points slmodemd links
 * (FAX_create / FAX_process / FAX_delete / FAX_class1_command) of the real dsplibs.o vs src/fax/fax_class1.c + src/fax/fax_api.c.  The blob run (-DTEST_BLOB, linked with ../dsplibs.o) and the
 * port run (-m32 -O0 / -O2; x86-64 against the recorded blob output) execute the same code; library test/fax/r30_common.h (hashed canonical output, coverage counters exit 2 on a zero,
 * 3 = harness violation, deterministic heap with block ids / red zones / poisoned frees, scripted modem).  Everything printed is host independent: objects are dumped member by member,
 * pointers as block / recorder serial numbers, buffers as hashes.
 *
 * Three builds (one main file each: fax_class1.main.c / fax_api.glue.main.c / fax_api.eng.main.c):
 *   R30_CLASS1 (id r30_class1, scenarios 0 .. 3): every callee of the class 1 layer is a RECORDER (the blob's copies are weakened with objcopy): FAXVMI_create / _delete, FPM_TONE_create /
 *      _delete, FPM_iir_filt_II, FIFO_delete, the 15 global state functions, _cHDLCrx_init_from_idle, _hdlc_receive_state_init, _rx_look_carrier_init, _delete_data_rx_modem /
 *      _delete_data_tx_modem, _tx_scrambled_ones_init, cHDLCtx_preamble_state_init.  The four blob-LOCAL state functions (_idle_state, _send_silence_state, _answer_tone_state,
 *      _recieve_silence_state) cannot be replaced in the blob (their addresses are .text relocations): the harness compares the table slots with the real functions after fax_class1_create and
 *      then overwrites them (blob run only) with recorders of its own, so every state of fax_class1_progress is scripted.  The constant records the code copies (FAXVMI_CFG, V21TX_CFG,
 *      V21RX_CFG, FPM_TONE_CFG) are POKED with random patterns (the blob's .rodata through mprotect) so that every copied word is observable.
 *        0 fax_class1_create: modes, cfg words, answer tone length, calling tone, supplied object, channel / tone creation failures, malloc failure probes (the blob crashes)
 *        1 fax_class1_command: every command x modem / parameter x object state x debug level, the order of the callee calls and of the object stores
 *        2 fax_class1_progress: single calls (scripted state functions, clock, input watch, high pass, state change message, delayed status, accept-rate flags of the three data receivers)
 *        3 life cycles: create -> commands -> progress -> delete with scripted callees, fax_class1_delete of random object compositions
 *   R30_GLUE (id r30_glue, scenarios 4 .. 5): fax_class1_create / progress / command / delete, RcFixed_*, the modem are recorders; FAX_create / FAX_delete / FAX_class1_command / FAX_process.
 *   HARNESS_ENG (id r30_eng, scenarios 6 .. 8): NOTHING replaced: two FAX objects over the real class 1 layer, FAXVMI and modems exchange audio blocks.
 * R30_VERBOSE=1 prints every line (debugging), R30_SEED selects another input stream.
 */
#include <sys/mman.h>

#include "r30_common.h"
#include "fax/fax_v21.h"
#include "fax/fax_v27_rx.h"
#include "fax/fax_v29_rx.h"
#include "fax/fax_v17_rx.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
#else
#define SIDE_BLOB 0
#endif
extern unsigned int dsplibs_debug_level;

/* ---------------------------------------------------------------------------------------------- blob / port symbol names */
typedef FaxT30StateFn StateFn;
#ifdef TEST_BLOB
extern FaxT30StateFn class1_state_functions[19];
extern int _idle_state(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
extern int _send_silence_state(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
extern int _answer_tone_state(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
extern int _recieve_silence_state(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
#define STATETAB class1_state_functions
#define REAL_S8 _idle_state
#define REAL_S14 _answer_tone_state
#define REAL_S15 _send_silence_state
#define REAL_S16 _recieve_silence_state
#define COUNTER_NONE 0
#else
#define STATETAB fax_class1_state_functions
#endif

/* write into a read-only (blob .rodata) or writable object */
static void poke_bytes(void *dst, const void *src, size_t n)
{
#ifdef TEST_BLOB
  uintptr_t a = (uintptr_t)dst & ~(uintptr_t)4095, e = ((uintptr_t)dst + n + 4095) & ~(uintptr_t)4095;
  if (mprotect((void *)a, e - a, PROT_READ | PROT_WRITE | PROT_EXEC) != 0) die3("mprotect");
#endif
  memcpy(dst, src, n);
}

#if defined(R30_CLASS1) || defined(R30_GLUE)
#define HARNESS_REC 1
#endif

#ifdef R30_CLASS1
/* the constant records fax_class1_create copies (the port's are defined here: the id links no table file) */
#ifdef TEST_BLOB
extern FaxVmiCfg h_FAXVMI_CFG __asm__("FAXVMI_CFG");
extern int h_V21TX_CFG[7] __asm__("V21TX_CFG");
extern int h_V21RX_CFG[6] __asm__("V21RX_CFG");
extern int h_FPM_TONE_CFG[9] __asm__("FPM_TONE_CFG");
#else
FaxVmiCfg h_FAXVMI_CFG __asm__("FAXVMI_CFG");
int h_V21TX_CFG[7] __asm__("V21TX_CFG");
int h_V21RX_CFG[6] __asm__("V21RX_CFG");
int h_FPM_TONE_CFG[9] __asm__("FPM_TONE_CFG_raw");
#endif
#endif

/* ---------------------------------------------------------------------------------------------- coverage */
#define COVLIST(X)                                                                                                                                                                  \
  X(C0_CREATE, 0, "fax_class1_create")                                                                                                                                              \
  X(C0_ORIG, 0, "create: originate side (state 4)")                                                                                                                                 \
  X(C0_ANSWER, 0, "create: answer side (state 14)")                                                                                                                                 \
  X(C0_MODE_OTHER, 0, "create: mode neither 1 nor 2")                                                                                                                               \
  X(C0_ANSLEN, 0, "create: answer tone length from cfg.w10")                                                                                                                        \
  X(C0_ANSLEN_NEG, 0, "create: negative answer tone length")                                                                                                                        \
  X(C0_ANSLEN_DEF, 0, "create: answer tone length 150")                                                                                                                             \
  X(C0_ANSLEN_DBG, 0, "create: answer tone length debug line")                                                                                                                      \
  X(C0_CNG_ON, 0, "create: calling tone on")                                                                                                                                        \
  X(C0_CNG_OFF, 0, "create: calling tone off (debug line)")                                                                                                                         \
  X(C0_CNG_OFF_QUIET, 0, "create: calling tone off, no debug")                                                                                                                      \
  X(C0_HP_ON, 0, "create: high pass on")                                                                                                                                            \
  X(C0_HP_OFF, 0, "create: high pass off")                                                                                                                                          \
  X(C0_BANNER, 0, "create: version banner")                                                                                                                                         \
  X(C0_SUPPLIED, 0, "create: supplied object")                                                                                                                                      \
  X(C0_VMI_NULL_TX, 0, "create: transmit channel creation fails")                                                                                                                   \
  X(C0_VMI_NULL_RX, 0, "create: receive channel creation fails")                                                                                                                    \
  X(C0_TONE_NULL, 0, "create: tone creation fails")                                                                                                                                 \
  X(C0_TABLE, 0, "create: state table dumped")                                                                                                                                      \
  X(C0_POKED, 0, "create: poked constant records")                                                                                                                                  \
  X(C0_PROBE, 0, "create: malloc failure probes")                                                                                                                                   \
  X(C0_PROBE_FAIL, 0, "create: probe returned NULL")                                                                                                                                \
  X(C0_DELETE, 0, "create + delete round trip")                                                                                                                                     \
  X(C1_CMD, 1, "fax_class1_command")                                                                                                                                                \
  X(C1_TH, 1, "command TH")                                                                                                                                                         \
  X(C1_TM, 1, "command TM")                                                                                                                                                         \
  X(C1_TM_DEL, 1, "TM: receiver deleted first")                                                                                                                                     \
  X(C1_TM_NODEL, 1, "TM: no receiver to delete")                                                                                                                                    \
  X(C1_RM, 1, "command RM")                                                                                                                                                         \
  X(C1_RM_DEL, 1, "RM: transmitter deleted first")                                                                                                                                  \
  X(C1_RM_NODEL, 1, "RM: no transmitter to delete")                                                                                                                                 \
  X(C1_RH, 1, "command RH")                                                                                                                                                         \
  X(C1_RH_SAME, 1, "RH: state 6 and the same modem (state 5)")                                                                                                                      \
  X(C1_RH_DIFF, 1, "RH: state 6, other modem")                                                                                                                                      \
  X(C1_RH_IDLE, 1, "RH: superframe empty (init from idle)")                                                                                                                         \
  X(C1_RH_EMU, 1, "RH: superframe present (state 7)")                                                                                                                              \
  X(C1_TS, 1, "command TS")                                                                                                                                                         \
  X(C1_RS, 1, "command RS")                                                                                                                                                         \
  X(C1_TIMER_ONE, 1, "TS / RS: timer 0 / 1 -> 1")                                                                                                                                   \
  X(C1_TIMER_NEG, 1, "TS / RS: negative parameter")                                                                                                                                 \
  X(C1_BAD, 1, "unknown command (ignored)")                                                                                                                                         \
  X(C1_NEG, 1, "negative command")                                                                                                                                                  \
  X(C1_DBG1, 1, "command debug line 1")                                                                                                                                             \
  X(C1_DBG2, 1, "command debug line 2 (names)")                                                                                                                                     \
  X(C1_SFCLEAR, 1, "sf_fill cleared (command != RH)")                                                                                                                               \
  X(C1_SFKEEP, 1, "sf_fill kept (RH)")                                                                                                                                              \
  X(C1_ORDER, 1, "recorders poke markers (order of the stores)")                                                                                                                    \
  X(C2_PROG, 2, "fax_class1_progress")                                                                                                                                              \
  X(C2_WATCH, 2, "progress: input watch on (rx_act != 0)")                                                                                                                          \
  X(C2_WATCH_OFF, 2, "progress: input watch off")                                                                                                                                   \
  X(C2_WATCH_RESET, 2, "progress: watch reset (too many non zero)")                                                                                                                 \
  X(C2_WATCH_DBG, 2, "progress: input dump (debug)")                                                                                                                                \
  X(C2_WATCH_NEG, 2, "progress: n_in < 0 in the watch")                                                                                                                             \
  X(C2_CLK_SEC, 2, "progress: clock second step")                                                                                                                                   \
  X(C2_CLK_MS, 2, "progress: clock ms step")                                                                                                                                        \
  X(C2_HP, 2, "progress: high pass called")                                                                                                                                         \
  X(C2_NOHP, 2, "progress: no high pass")                                                                                                                                           \
  X(C2_STATE_CHG, 2, "progress: state change message")                                                                                                                              \
  X(C2_STATE_SAME, 2, "progress: state unchanged")                                                                                                                                  \
  X(C2_STATE_UNKNOWN, 2, "progress: state name unknown")                                                                                                                            \
  X(C2_STATE_PREVUNK, 2, "progress: previous state name unknown")                                                                                                                   \
  X(C2_TXCNT, 2, "progress: TxSmpCnt != 160")                                                                                                                                       \
  X(C2_DELAY, 2, "progress: delayed status counting")                                                                                                                               \
  X(C2_DELAY_FIRE, 2, "progress: delayed status delivered")                                                                                                                         \
  X(C2_STATUS, 2, "progress: status reported")                                                                                                                                      \
  X(C2_STATUS_NAMED, 2, "progress: status name known")                                                                                                                              \
  X(C2_STATUS_UNNAMED, 2, "progress: status name unknown")                                                                                                                          \
  X(C2_ACCEPT27, 2, "progress: accept rate V.27ter")                                                                                                                                \
  X(C2_ACCEPT29, 2, "progress: accept rate V.29")                                                                                                                                   \
  X(C2_ACCEPT17, 2, "progress: accept rate V.17")                                                                                                                                   \
  X(C2_ACCEPT_NO, 2, "progress: accept flag clear")                                                                                                                                 \
  X(C2_ACCEPT_NOCHAN, 2, "progress: accept check but no receiver")                                                                                                                  \
  X(C2_RET0, 2, "progress: returns 0")                                                                                                                                              \
  X(C2_BADSTATE, 2, "progress: state out of range probe")                                                                                                                           \
  X(C2_NIN_COPY, 2, "progress: state function gets a copy of n_in")                                                                                                                 \
  X(C2_SEQ, 2, "progress: sequences")                                                                                                                                               \
  X(C3_LIFE, 3, "life cycle")                                                                                                                                                       \
  X(C3_DEL, 3, "fax_class1_delete")                                                                                                                                                 \
  X(C3_DEL_TXCFG, 3, "delete: transmit config block")                                                                                                                               \
  X(C3_DEL_RXCFG, 3, "delete: receive config block")                                                                                                                                \
  X(C3_DEL_CH, 3, "delete: channels")                                                                                                                                               \
  X(C3_DEL_DRX, 3, "delete: data receiver")                                                                                                                                         \
  X(C3_DEL_DTX, 3, "delete: data transmitter")                                                                                                                                      \
  X(C3_DEL_NODATA, 3, "delete: data_cfg without data_vmi")                                                                                                                          \
  X(C3_DEL_FIFO, 3, "delete: fifo")                                                                                                                                                 \
  X(C3_DEL_TONE, 3, "delete: tone")                                                                                                                                                 \
  X(C3_DEL_BARE, 3, "delete: nothing to free but the object")                                                                                                                       \
  X(C3_CMDS, 3, "life: commands")                                                                                                                                                   \
  X(C3_PROGS, 3, "life: progress calls")                                                                                                                                            \
  X(C3_ANS, 3, "life: answering side")                                                                                                                                              \
  X(G4_CREATE, 4, "FAX_create")                                                                                                                                                     \
  X(G4_R8000, 4, "FAX_create 8000 (no converters)")                                                                                                                                 \
  X(G4_R9600, 4, "FAX_create 9600")                                                                                                                                                 \
  X(G4_R48000, 4, "FAX_create 48000")                                                                                                                                               \
  X(G4_RBAD, 4, "FAX_create unsupported rate")                                                                                                                                      \
  X(G4_RC1_FAIL, 4, "FAX_create: first converter fails")                                                                                                                            \
  X(G4_RC2_FAIL, 4, "FAX_create: second converter fails")                                                                                                                           \
  X(G4_C1_FAIL, 4, "FAX_create: class 1 creation fails")                                                                                                                            \
  X(G4_MALLOC_FAIL, 4, "FAX_create: malloc failure")                                                                                                                                \
  X(G4_CALLER0, 4, "FAX_create caller 0 (mode 2)")                                                                                                                                  \
  X(G4_CALLER1, 4, "FAX_create caller != 0 (mode 1)")                                                                                                                               \
  X(G4_DELETE, 4, "FAX_delete")                                                                                                                                                     \
  X(G4_DELETE_NOC1, 4, "FAX_delete without class 1 object")                                                                                                                         \
  X(G4_CMD, 4, "FAX_class1_command")                                                                                                                                                \
  X(G4_CMD_FTS, 4, "command FTS")                                                                                                                                                   \
  X(G4_CMD_FRS, 4, "command FRS")                                                                                                                                                   \
  X(G4_CMD_FTM, 4, "command FTM")                                                                                                                                                   \
  X(G4_CMD_FRM, 4, "command FRM")                                                                                                                                                   \
  X(G4_CMD_FTH, 4, "command FTH")                                                                                                                                                   \
  X(G4_CMD_FRH, 4, "command FRH")                                                                                                                                                   \
  X(G4_CMD_BAD, 4, "unknown command")                                                                                                                                               \
  X(G4_CMD_REJECT, 4, "modem / parameter rejected")                                                                                                                                 \
  X(G4_CMD_OK, 4, "command forwarded")                                                                                                                                              \
  X(G4_CMD_NULL, 4, "NULL object")                                                                                                                                                  \
  X(G4_CMD_NOC1, 4, "no class 1 object")                                                                                                                                            \
  X(G4_CRASH, 4, "NULL delete crashes (both)")                                                                                                                                      \
  X(G5_PROC, 5, "FAX_process")                                                                                                                                                      \
  X(G5_S0, 5, "status 0")                                                                                                                                                           \
  X(G5_S1, 5, "status 1 OK")                                                                                                                                                        \
  X(G5_S2, 5, "status 2 ERROR")                                                                                                                                                     \
  X(G5_S3, 5, "status 3 OK_NO_CARRIER")                                                                                                                                             \
  X(G5_S4, 5, "status 4 ERROR_NO_CARRIER")                                                                                                                                          \
  X(G5_S5, 5, "status 5 ERROR_ON_HOOK")                                                                                                                                             \
  X(G5_S6, 5, "status 6 CONNECT")                                                                                                                                                   \
  X(G5_S7, 5, "status 7 NO_CARRIER")                                                                                                                                                \
  X(G5_S8, 5, "status 8 NO_CARRIER_NO_MESSAGE")                                                                                                                                     \
  X(G5_S9, 5, "status 9 OTHER_CARRIER")                                                                                                                                             \
  X(G5_S10, 5, "status 10 ACCEPT_RATE")                                                                                                                                             \
  X(G5_SUNK, 5, "unknown status")                                                                                                                                                   \
  X(G5_RECV, 5, "tty bytes received (online)")                                                                                                                                      \
  X(G5_RECV_CLAMP, 5, "tty count clamped to 0x1000")                                                                                                                                \
  X(G5_RECV_ZERO, 5, "tty count zero")                                                                                                                                              \
  X(G5_OFFLINE, 5, "offline: no tty read")                                                                                                                                          \
  X(G5_SEND, 5, "bytes sent to the tty")                                                                                                                                            \
  X(G5_NOSEND, 5, "no bytes / offline: no send")                                                                                                                                    \
  X(G5_BADCNT, 5, "sample count != 160 (error path)")                                                                                                                               \
  X(G5_RC, 5, "converters in the path (9600)")                                                                                                                                      \
  X(G5_NORC, 5, "no converters (8000)")                                                                                                                                             \
  X(G5_RING_WRAP, 5, "ring positions wrap")                                                                                                                                         \
  X(G5_MULTI, 5, "more than one block per call")                                                                                                                                    \
  X(G5_PARTIAL, 5, "partial blocks (no processing)")                                                                                                                                \
  X(G5_RET_LAST, 5, "return value = last non-zero status")                                                                                                                          \
  X(G5_ZERO_COUNT, 5, "count <= 0")                                                                                                                                                 \
  X(G5_MUT, 5, "mutating debug hooks")                                                                                                                                              \
  X(G5_RING_POKE, 5, "ring words poked independently")                                                                                                                              \
  X(G5_DBG, 5, "debug lines")                                                                                                                                                       \
  X(G5_NIN_SENT, 5, "progress sees n_in = n_out = 160")                                                                                                                             \
  X(E6_SESS, 6, "HDLC sessions")                                                                                                                                                    \
  X(E6_CONNECT, 6, "sender CONNECT (status 6)")                                                                                                                                     \
  X(E6_OK, 6, "status OK")                                                                                                                                                          \
  X(E6_BYTES, 6, "bytes delivered to the DTE")                                                                                                                                      \
  X(E6_STATE4, 6, "state 4 ran")                                                                                                                                                    \
  X(E6_STATE5, 6, "state 5 ran")                                                                                                                                                    \
  X(E6_ANSWER, 6, "answer tone side ran (state 14)")                                                                                                                                \
  X(E6_R8000, 6, "8000 Hz session")                                                                                                                                                 \
  X(E6_R9600, 6, "9600 Hz session")                                                                                                                                                 \
  X(E6_NOISE, 6, "line noise session")                                                                                                                                              \
  X(E6_NOCARRIER, 6, "no carrier / timeout status")                                                                                                                                 \
  X(E7_SESS, 7, "data sessions")                                                                                                                                                    \
  X(E7_V27, 7, "V.27ter session")                                                                                                                                                   \
  X(E7_V29, 7, "V.29 session")                                                                                                                                                      \
  X(E7_V17, 7, "V.17 session")                                                                                                                                                      \
  X(E7_CONNECT, 7, "data CONNECT (status 6)")                                                                                                                                       \
  X(E7_BYTES, 7, "data bytes delivered to the DTE")                                                                                                                                 \
  X(E7_SHORT, 7, "short train session")                                                                                                                                             \
  X(E7_ACCEPT, 7, "accept-rate status 10")                                                                                                                                          \
  X(E7_TS, 7, "silence commands (FTS / FRS)")                                                                                                                                       \
  X(E8_LIFE, 8, "create / delete mid session")                                                                                                                                      \
  X(E8_ORDER, 8, "command sequence")                                                                                                                                                \
  X(E8_TIMEOUT, 8, "receivers waiting for a carrier (S7 timeout)")                                                                                                                 \
  X(E8_ANS, 8, "answering side data")                                                                                                                                               \
  X(E9_PROBE, 9, "malloc failure probe children")
#define X(id, scen, name) CV_##id,
enum { COVLIST(X) CV_END };
#undef X
static void cov_setup(int scen)
{
#define X(id, sc, name) \
  if (sc == scen) cov_def(CV_##id, name);
  COVLIST(X)
#undef X
}

/* the coverage observer of the debug text (every build) */
static void dbg_cov(const char *b)
{
  if (strstr(b, "samples count")) cov_hit(CV_G5_BADCNT);
  if (strncmp(b, "fax: ", 5) == 0) cov_hit(CV_G5_DBG);
  if (strstr(b, "fax: snd:")) cov_hit(CV_G5_RECV);
  if (strstr(b, "fax: rcv:")) cov_hit(CV_G5_SEND);
  if (strstr(b, "FAX_CLASS1_ACCEPT_RATE")) cov_hit(CV_E7_ACCEPT);
}

#include "r30_obj.h"
#ifdef R30_CLASS1
#include "r30_rec.h"
#include "r30_class1.h"
#endif
#ifdef R30_GLUE
#include "r30_glue.h"
#endif
#ifdef HARNESS_ENG
#include "r30_eng.h"
#endif

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R30_SEED");
  g_verbose = getenv("R30_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)strtoul(sd, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)scen * 7919u;
  cov_setup(scen);
  g_sreg[7] = 7;
#ifdef R30_CLASS1
  rec_names();
#endif
#ifdef R30_CLASS1
  wrapfn_init();
  if (scen != 0) table_boot();
  switch (scen) {
    case 0: scen_create(); break;
    case 1: scen_command(); break;
    case 2: scen_progress(); break;
    case 3: scen_life(); break;
    default: fprintf(stderr, "scenario 0..3\n"); return 2;
  }
#endif
#ifdef R30_GLUE
  switch (scen) {
    case 4: scen_fax_create(); break;
    case 5: scen_fax_process(); break;
    default: fprintf(stderr, "scenario 4..5\n"); return 2;
  }
#endif
#ifdef HARNESS_ENG
  switch (scen) {
    case 6: scen_eng_hdlc(); break;
    case 7: scen_eng_data(); break;
    case 8: scen_eng_life(); break;
    case 9: scen_eng_probes(); break;
    default: fprintf(stderr, "scenario 6..9\n"); return 2;
  }
#endif
  cov_finish();
  return 0;
}
