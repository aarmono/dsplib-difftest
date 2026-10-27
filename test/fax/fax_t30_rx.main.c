/*
 * R29 differential harness: the T.30 / HDLC RECEIVE side of the fax class 1 object of dsplibs.o (the receive state machines, their initialisers, _init_receiver / _delete_data_rx_modem,
 * the DTE input converters _handle_data_input / _handle_hdlc_input(_open / _close), the receive silence state) vs src/fax/fax_t30_rx.c.  The blob run (-DTEST_BLOB, linked with ../dsplibs.o) and the
 * port run (-m32 -O0 / -O2; x86-64 against the recorded blob output) execute the same code; library test/fax/r29_common.h (hashed canonical output, coverage counters exit 2 on a zero,
 * 3 = harness violation, deterministic heap with block ids / red zones / poisoned frees).  Everything printed is host independent: objects are dumped member by member, pointers as
 * block / recorder serial numbers, buffers as hashes.
 *
 * REC build (default): the callees of the code under test are RECORDERS with scripted results (the blob's copies are weakened with objcopy, see fax_t30_rx.*.sh):
 *   FAXVMI_create / _delete / _process / _control (arguments logged, the modem records behind FaxVmiCtl.mctl dumped; _process writes a scripted HDLC frame ([len][bytes] words, 0 = bad CRC)
 *   or scripted data words into the buffer and returns a scripted state word), FPM_rms, FPM_TONE_generate, the init_vmi_data_rx_modem table entries (recorder initialisers for every modem
 *   id, V.17 with its three caller buffers), sysdep_malloc / _free (block ids, red zones), sysdep_sprintf, dsplibs_debug_printf.  The recorder channels carry FAKE modem engines (typed
 *   V21Rx / V27Rx / V29Rx / V17Rx objects whose hdx / bundle pointers are real: the receive code pokes the AGC / carrier flag / baud rate words of the engines; the layouts are the
 *   typed ones of src/fax/fax_*_rx.h, identical to the blob's at -m32).  The real R28 functions (cTOOLS_*, _put_silence, _idle_state_init, _handle_data_output) run on both sides.
 *   The constant records FAXVMI_CTL / V21RX_CTL / V27RX_CTL ... are POKED with random patterns (the blob's in .rodata through mprotect) so that every copied word is observable.
 *   Scenarios:
 *     0 DTE input: _handle_data_input, _handle_hdlc_input, _handle_hdlc_input_open / _close, _recieve_silence_state_init (random streams with DLE / ETX, random object state);
 *     1 small functions: _hdlc_receive_state_init, _cHDLCrx_init_from_idle, _recieve_silence_state, the gain threshold table;
 *     2 HDLC receive states: _hdlc_receive_look_carrier_state, _hdlc_receive_state, _hdlc_receive_between_buffers_state, _hdlc_emulate_receive_state (single calls, dispatched sequences);
 *     3 data receive states: _rx_look_carrier_state, _rx_data_state (single calls, dispatched sequences, _rx_look_carrier_init);
 *     4 life cycle: _init_receiver (every modem value, every object state, recorder initialisers of every class), _delete_data_rx_modem, forked failure probes.
 * ENG build (HARNESS_ENG): the REAL FAXVMI layer and the real V.21 / V.27ter / V.29 / V.17 engines on both sides (scenarios 5 .. 7, test/fax/r29_eng.h): the R28 transmit states of one object
 * feed the receive states of another.
 * R29_VERBOSE=1 prints every line (debugging), R29_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r29_common.h"
#include "fax/fax_v21.h"
#include "fax/fax_v21_tables.h"
#include "fax/fax_v27_rx.h"
#include "fax/fax_v29_rx.h"
#include "fax/fax_v17_rx.h"
#include "cid/cid.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
#else
#define SIDE_BLOB 0
#endif
extern unsigned int dsplibs_debug_level;

/* ---------------------------------------------------------------------------------------------- blob / port symbol names */
typedef int (*StateFn)(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
#ifdef TEST_BLOB
extern int _idle_state(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
extern int _send_silence_state(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
extern int _answer_tone_state(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
extern int _recieve_silence_state(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8);
extern int cDATAtx_counter;
extern FaxVmiInitFn init_vmi_data_tx_modem[3];
extern FaxVmiInitFn init_vmi_data_rx_modem[3];
extern const short GAIN_THRESHOLD_TABLE[4];
#define T_IDLE _idle_state
#define T_SILENCE _send_silence_state
#define T_ANSWER _answer_tone_state
#define T_RSIL _recieve_silence_state
#define COUNTER cDATAtx_counter
#define HDI_COUNTER (*(&cDATAtx_counter + 1)) /* the blob-LOCAL `temp.0` (.bss + 0x8c4) follows cDATAtx_counter (.bss + 0x8c0) */
#define INITTAB_TX init_vmi_data_tx_modem
#define INITTAB_RX init_vmi_data_rx_modem
#define GAIN_TAB GAIN_THRESHOLD_TABLE
#ifndef HARNESS_ENG
extern unsigned char h_FAXVMI_CTL[] __asm__("FAXVMI_CTL");
extern unsigned char h_FAXVMI_STS[] __asm__("FAXVMI_STS");
extern int h_V21TX_CTL[5] __asm__("V21TX_CTL");
extern int h_V21RX_CTL[4] __asm__("V21RX_CTL");
extern int h_V27TX_CTL[5] __asm__("V27TX_CTL");
extern int h_V29TX_CTL[4] __asm__("V29TX_CTL");
extern int h_V17TX_CTL[5] __asm__("V17TX_CTL");
extern int h_V27RX_CTL[5] __asm__("V27RX_CTL");
extern int h_V29RX_CTL[4] __asm__("V29RX_CTL");
extern int h_V17RX_CTL[5] __asm__("V17RX_CTL");
#endif
#else
#define T_IDLE fax_idle_state
#define T_SILENCE fax_send_silence_state
#define T_ANSWER fax_answer_tone_state
#define T_RSIL fax_recieve_silence_state
#define COUNTER fax_cDATAtx_counter
#define HDI_COUNTER fax_handle_data_input_count
#define INITTAB_TX fax_init_vmi_data_tx_modem
#define INITTAB_RX fax_init_vmi_data_rx_modem
#define GAIN_TAB fax_gain_threshold_table
#ifndef HARNESS_ENG
FaxVmiInitFn fax_init_vmi_data_tx_modem[3];
FaxVmiInitFn fax_init_vmi_data_rx_modem[3];
/* the constant records the port references, defined here (writable, bound to the blob's symbol names; sized for the x86-64 layouts of FaxVmiCtl / FaxVmiSts) */
unsigned char h_FAXVMI_CTL[64] __asm__("FAXVMI_CTL");
unsigned char h_FAXVMI_STS[64] __asm__("FAXVMI_STS");
int h_V21TX_CTL[5] __asm__("V21TX_CTL") = {0x12c, 0xea60, 0xc80, 0, 0};
int h_V21RX_CTL[4] __asm__("V21RX_CTL") = {0x12c0000, 0xea60, 0, 0};
int h_V27TX_CTL[5] __asm__("V27TX_CTL") = {0x2580, 0xea60, 1, 0, 0};
int h_V29TX_CTL[4] __asm__("V29TX_CTL") = {0x2580, 0xea60, 1, 0};
int h_V17TX_CTL[5] __asm__("V17TX_CTL") = {0x3840, 0xea60, 1, 0, 0};
int h_V27RX_CTL[5] __asm__("V27RX_CTL") = {0x2580, 0xea60, 1, 0, 0};
int h_V29RX_CTL[4] __asm__("V29RX_CTL") = {0x2580, 0xea60, 1, 0};
int h_V17RX_CTL[5] __asm__("V17RX_CTL") = {0x3840, 0xea60, 1, 0, 0};
#endif
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

/* ---------------------------------------------------------------------------------------------- coverage */
#define COVLIST(X)                                                                                                                                                                  \
  X(IN_HDI, 0, "_handle_data_input")                                                                                                                                                \
  X(IN_HDI_BUSY, 0, "hdi: w12b0 set (nothing done)")                                                                                                                                \
  X(IN_HDI_DLE, 0, "hdi: DLE doubled")                                                                                                                                              \
  X(IN_HDI_ETX, 0, "hdi: DLE ETX (zero fill, w12b0)")                                                                                                                               \
  X(IN_HDI_ETX_FULL, 0, "hdi: ETX with the buffer full (esi > 0x7ff)")                                                                                                              \
  X(IN_HDI_OTHER, 0, "hdi: DLE + other byte dropped")                                                                                                                               \
  X(IN_HDI_DBG3, 0, "hdi: debug line (level 3)")                                                                                                                                    \
  X(IN_HDI_N0, 0, "hdi: n <= 0")                                                                                                                                                    \
  X(IN_HDI_PEND, 0, "hdi: DLE pending from the previous call")                                                                                                                      \
  X(IN_HHI, 0, "_handle_hdlc_input")                                                                                                                                                \
  X(IN_HHI_FRAME, 0, "hhi: frame complete (returns 1)")                                                                                                                             \
  X(IN_HHI_PART, 0, "hhi: frame not complete (returns 0)")                                                                                                                          \
  X(IN_HHI_LAST, 0, "hhi: last frame bit (w1224)")                                                                                                                                  \
  X(IN_HHI_NOLAST, 0, "hhi: not the last frame")                                                                                                                                    \
  X(IN_HHI_DLE, 0, "hhi: DLE doubled")                                                                                                                                              \
  X(IN_HHI_OTHER, 0, "hhi: DLE + other byte dropped")                                                                                                                               \
  X(IN_HHI_N0, 0, "hhi: n <= 0")                                                                                                                                                    \
  X(IN_HHI_SEP, 0, "hhi: output buffer is not the object")                                                                                                                          \
  X(IN_OPEN, 0, "_handle_hdlc_input_open")                                                                                                                                          \
  X(IN_CLOSE, 0, "_handle_hdlc_input_close")                                                                                                                                        \
  X(IN_CLOSE_LAST, 0, "close: last frame bit")                                                                                                                                      \
  X(IN_RSI, 0, "_recieve_silence_state_init")                                                                                                                                       \
  X(IN_RSI_ZERO, 0, "silence init: half = 0")                                                                                                                                       \
  X(IN_RSI_NEG, 0, "silence init: n < 0")                                                                                                                                           \
  X(IN_CNT, 0, "hdi byte counter")                                                                                                                                                 \
  X(IN_DEAD, 0, "dead caller-ID helpers (_look_for*, pack_next_bit)")                                                                                                                                                  \
  X(SM_HRSI, 1, "_hdlc_receive_state_init")                                                                                                                                         \
  X(SM_HRSI_DBG, 1, "hdlc_receive_state_init debug line")                                                                                                                           \
  X(SM_IDLE, 1, "_cHDLCrx_init_from_idle")                                                                                                                                          \
  X(SM_IDLE_3, 1, "init_from_idle mode 3")                                                                                                                                          \
  X(SM_IDLE_OTHER, 1, "init_from_idle other mode")                                                                                                                                  \
  X(SM_IDLE_DBG, 1, "init_from_idle debug line")                                                                                                                                    \
  X(SM_IDLE_POKE, 1, "init_from_idle with poked records")                                                                                                                           \
  X(SM_SIL, 1, "_recieve_silence_state")                                                                                                                                            \
  X(SM_SIL_ABORT, 1, "silence: pending byte aborts")                                                                                                                                \
  X(SM_SIL_LOUD, 1, "silence: energy > 100")                                                                                                                                        \
  X(SM_SIL_QUIET, 1, "silence: energy <= 100, waiting")                                                                                                                             \
  X(SM_SIL_END, 1, "silence: time reached")                                                                                                                                         \
  X(SM_SIL_EDGE, 1, "silence: energy 100 / 101")                                                                                                                                    \
  X(SM_SIL_NEG, 1, "silence: negative energy")                                                                                                                                      \
  X(SM_TAB, 1, "gain threshold table")                                                                                                                                              \
  X(HR_LC, 2, "_hdlc_receive_look_carrier_state")                                                                                                                                   \
  X(HR_LC_CARRIER, 2, "look carrier: carrier detected")                                                                                                                             \
  X(HR_LC_GAIN0, 2, "look carrier: gain loop k = 0")                                                                                                                                \
  X(HR_LC_GAIN1, 2, "look carrier: gain loop k = 1")                                                                                                                                \
  X(HR_LC_GAIN2, 2, "look carrier: gain loop k = 2")                                                                                                                                \
  X(HR_LC_GAIN3, 2, "look carrier: gain loop k = 3")                                                                                                                                \
  X(HR_LC_GAINNONE, 2, "look carrier: gain loop finds nothing")                                                                                                                     \
  X(HR_LC_TIMEOUT, 2, "look carrier: no carrier, time elapsed")                                                                                                                     \
  X(HR_LC_NOTIME, 2, "look carrier: no carrier, waiting")                                                                                                                           \
  X(HR_LC_TIMEOUT0, 2, "look carrier: s7 timeout 0")                                                                                                                                \
  X(HR_LC_CNG_OFF, 2, "look carrier: calling tone off")                                                                                                                             \
  X(HR_LC_CNG_SIL, 2, "look carrier: tone silence phase")                                                                                                                           \
  X(HR_LC_CNG_SIL_END, 2, "look carrier: silence phase ends")                                                                                                                       \
  X(HR_LC_CNG_TONE, 2, "look carrier: tone phase")                                                                                                                                  \
  X(HR_LC_CNG_TONE_END, 2, "look carrier: tone phase ends")                                                                                                                         \
  X(HR_LC_ABORT, 2, "look carrier: TxDatCnt abort")                                                                                                                                 \
  X(HR_LC_N0, 2, "look carrier: n_in = 0")                                                                                                                                          \
  X(HR_ST, 2, "_hdlc_receive_state")                                                                                                                                                \
  X(HR_ST_WRAP, 2, "receive state: timer wraps (no process)")                                                                                                                       \
  X(HR_ST_NOCARR, 2, "receive state: no carrier")                                                                                                                                   \
  X(HR_ST_NOCARR_FRAME, 2, "receive state: no carrier but a frame")                                                                                                                 \
  X(HR_ST_OK, 2, "receive state: frame OK")                                                                                                                                         \
  X(HR_ST_ERR, 2, "receive state: frame with error")                                                                                                                                \
  X(HR_ST_NOFRAME, 2, "receive state: no frame")                                                                                                                                    \
  X(HR_ST_PREV6, 2, "receive state: previous state 6")                                                                                                                              \
  X(HR_ST_ABORT, 2, "receive state: TxDatCnt abort")                                                                                                                                \
  X(HR_BB, 2, "_hdlc_receive_between_buffers_state")                                                                                                                                \
  X(HR_BB_STORE, 2, "between buffers: frame stored")                                                                                                                                \
  X(HR_BB_FULL, 2, "between buffers: superframe full")                                                                                                                              \
  X(HR_BB_NOFRAME, 2, "between buffers: no frame")                                                                                                                                  \
  X(HR_BB_NOCARR, 2, "between buffers: no carrier")                                                                                                                                 \
  X(HR_BB_CARR, 2, "between buffers: carrier")                                                                                                                                      \
  X(HR_BB_ABORT, 2, "between buffers: TxDatCnt abort")                                                                                                                              \
  X(HR_BB_ZERO, 2, "between buffers: empty frame")                                                                                                                                  \
  X(HR_EM, 2, "_hdlc_emulate_receive_state")                                                                                                                                        \
  X(HR_EM_PREV7, 2, "emulate: previous state 7")                                                                                                                                    \
  X(HR_EM_OTHER, 2, "emulate: other previous state")                                                                                                                                \
  X(HR_EM_WAIT, 2, "emulate: delay running")                                                                                                                                        \
  X(HR_EM_FRAME, 2, "emulate: frame handed out")                                                                                                                                    \
  X(HR_EM_DONE, 2, "emulate: all frames out (state 8)")                                                                                                                             \
  X(HR_EM_RESET, 2, "emulate: superframe cleared")                                                                                                                                  \
  X(HR_EM_NONE, 2, "emulate: no frames")                                                                                                                                            \
  X(HR_EM_W6, 2, "emulate: w122c = 6")                                                                                                                                              \
  X(HR_EM_MANY, 2, "emulate: 8 or more frames")                                                                                                                                     \
  X(HR_SEQ, 2, "dispatched sequences")                                                                                                                                              \
  X(HR_SEQ_S4, 2, "sequence ran state 4")                                                                                                                                           \
  X(HR_SEQ_S5, 2, "sequence ran state 5")                                                                                                                                           \
  X(HR_SEQ_S6, 2, "sequence ran state 6")                                                                                                                                           \
  X(HR_SEQ_S7, 2, "sequence ran state 7")                                                                                                                                           \
  X(HR_SEQ_S8, 2, "sequence ran state 8")                                                                                                                                           \
  X(HR_SEQ_LONG, 2, "long sequence")                                                                                                                                                \
  X(DR_LC, 3, "_rx_look_carrier_state")                                                                                                                                             \
  X(DR_LC_HDLC, 3, "rx look carrier: HDLC carrier during data search")                                                                                                              \
  X(DR_LC_CONNECT, 3, "rx look carrier: data connect")                                                                                                                              \
  X(DR_LC_TIMEOUT, 3, "rx look carrier: S7 elapsed")                                                                                                                                \
  X(DR_LC_WAIT, 3, "rx look carrier: waiting")                                                                                                                                      \
  X(DR_LC_ABORT, 3, "rx look carrier: TxDatCnt abort")                                                                                                                              \
  X(DR_LC_DATACARR, 3, "rx look carrier: data carrier bit (no hdlc process)")                                                                                                       \
  X(DR_LC_N0, 3, "rx look carrier: n_in = 0")                                                                                                                                       \
  X(DR_LC_TIMEOUT0, 3, "rx look carrier: s7 timeout 0")                                                                                                                             \
  X(DR_DT, 3, "_rx_data_state")                                                                                                                                                     \
  X(DR_DT_CARR, 3, "rx data: carrier")                                                                                                                                              \
  X(DR_DT_NOCARR, 3, "rx data: no carrier")                                                                                                                                         \
  X(DR_DT_ABORT, 3, "rx data: TxDatCnt abort")                                                                                                                                      \
  X(DR_DT_N0, 3, "rx data: no words")                                                                                                                                               \
  X(DR_DT_WORDS, 3, "rx data: words handed out")                                                                                                                                    \
  X(DR_INIT, 3, "_rx_look_carrier_init")                                                                                                                                            \
  X(DR_SEQ, 3, "dispatched sequences")                                                                                                                                              \
  X(DR_SEQ_S12, 3, "sequence ran state 12")                                                                                                                                         \
  X(DR_SEQ_S13, 3, "sequence ran state 13")                                                                                                                                         \
  X(DR_SEQ_S5, 3, "sequence entered state 5")                                                                                                                                       \
  X(DR_SEQ_S8, 3, "sequence ran state 8")                                                                                                                                           \
  X(DR_SEQ_S16, 3, "sequence ran state 16")                                                                                                                                         \
  X(DR_SEQ_LONG, 3, "long sequence")                                                                                                                                                \
  X(LF_INIT, 4, "_init_receiver")                                                                                                                                                   \
  X(LF_M18, 4, "modem 0x18")                                                                                                                                                        \
  X(LF_M30, 4, "modem 0x30")                                                                                                                                                        \
  X(LF_M48, 4, "modem 0x48")                                                                                                                                                        \
  X(LF_M60, 4, "modem 0x60")                                                                                                                                                        \
  X(LF_M49, 4, "modem 0x49")                                                                                                                                                        \
  X(LF_M4A, 4, "modem 0x4a")                                                                                                                                                        \
  X(LF_M61, 4, "modem 0x61")                                                                                                                                                        \
  X(LF_M62, 4, "modem 0x62")                                                                                                                                                        \
  X(LF_M79, 4, "modem 0x79")                                                                                                                                                        \
  X(LF_M7A, 4, "modem 0x7a")                                                                                                                                                        \
  X(LF_M91, 4, "modem 0x91")                                                                                                                                                        \
  X(LF_M92, 4, "modem 0x92")                                                                                                                                                        \
  X(LF_FRESH, 4, "fresh data channel")                                                                                                                                              \
  X(LF_REPLACE, 4, "previous channel of another class replaced")                                                                                                                    \
  X(LF_RESTART17, 4, "restart V.17 receiver (control record)")                                                                                                                      \
  X(LF_RESTART29, 4, "restart V.29 receiver (control record)")                                                                                                                      \
  X(LF_RESTART27, 4, "restart V.27ter receiver (control record)")                                                                                                                   \
  X(LF_LEAK, 4, "channel without a config block (overwritten)")                                                                                                                     \
  X(LF_DEL, 4, "_delete_data_rx_modem")                                                                                                                                             \
  X(LF_DEL17, 4, "delete V.17 receiver (three buffers)")                                                                                                                            \
  X(LF_DEL_OTHER, 4, "delete V.27ter / V.29 receiver")                                                                                                                              \
  X(LF_PROBE, 4, "failure probes")                                                                                                                                                  \
  X(LF_SHORT, 4, "short train")                                                                                                                                                     \
  X(LF_DBG, 4, "debug lines")                                                                                                                                                       \
  X(LF_AGC17, 4, "AGC / flag pokes V.17")                                                                                                                                           \
  X(LF_AGC29, 4, "AGC / flag pokes V.29")                                                                                                                                           \
  X(LF_AGC27, 4, "AGC / flag pokes V.27ter")                                                                                                                                        \
  X(LF_RATES, 4, "every rate code")                                                                                                                                                 \
  X(E_HSESS, 5, "HDLC sessions")                                                                                                                                                    \
  X(E_H_LC, 5, "state 4 ran")                                                                                                                                                       \
  X(E_H_RX, 5, "state 5 ran")                                                                                                                                                       \
  X(E_H_BB, 5, "state 6 ran")                                                                                                                                                       \
  X(E_H_EM, 5, "state 7 ran")                                                                                                                                                       \
  X(E_H_FRAME_OK, 5, "frame received (w123c = 1)")                                                                                                                                  \
  X(E_H_BYTES, 5, "bytes delivered to the DTE")                                                                                                                                     \
  X(E_H_ERR, 5, "bad frame (w123c = 2)")                                                                                                                                            \
  X(E_H_GAIN, 5, "carrier detected in state 4 (gain loop)")                                                                                                                                        \
  X(E_H_AGC, 5, "AGC words copied")                                                                                                                                                 \
  X(E_H_MULTI, 5, "more than one frame in a session")                                                                                                                               \
  X(E_DSESS, 6, "data sessions")                                                                                                                                                    \
  X(E_D_V27, 6, "V.27ter sessions")                                                                                                                                                 \
  X(E_D_V29, 6, "V.29 sessions")                                                                                                                                                    \
  X(E_D_V17, 6, "V.17 sessions")                                                                                                                                                    \
  X(E_D_CONNECT, 6, "receiver connected (state 13)")                                                                                                                                \
  X(E_D_BYTES, 6, "data bytes delivered")                                                                                                                                           \
  X(E_D_SHORT, 6, "short train sessions")                                                                                                                                           \
  X(E_D_CARRIER_LOST, 6, "carrier lost after the data")                                                                                                                             \
  X(E_LIFE, 7, "_init_receiver over real engines")                                                                                                                                  \
  X(E_L_MODS, 7, "every modem value")                                                                                                                                               \
  X(E_L_REPLACE, 7, "replace / restart")                                                                                                                                            \
  X(E_L_DEL, 7, "_delete_data_rx_modem")
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

/* ---------------------------------------------------------------------------------------------- debug line recorders */
static int g_dbg_on = 1;
static void dbg_cov(const char *b)
{
  static const struct {
    const char *s;
    int id;
  } t[] = {
      {"CLASS1: DLE", CV_IN_HDI_DBG3}, {"[sec] hdlc_receive_state_init", CV_SM_HRSI_DBG}, {"HDLCrx_init_from_idle", CV_SM_IDLE_DBG}, {"Abort waiting for silence", CV_SM_SIL_ABORT},
      {"Energy", CV_SM_SIL_EDGE},
  };
  unsigned i;
  for (i = 0; i < sizeof t / sizeof t[0]; i++)
    if (strstr(b, t[i].s)) cov_hit(t[i].id);
  if (strstr(b, "Initializing RX modem") || strstr(b, "Short train") || strstr(b, "New RX Modem") || strstr(b, "Restarting existing")) cov_hit(CV_LF_DBG);
}
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
  if (g_dbg_on) emit(0, "DBG %s", b);
  dbg_cov(b);
  return n;
}
int sysdep_sprintf(char *buf, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
int sysdep_sprintf(char *buf, const char *fmt, ...)
{
  int n;
  va_list ap;
  va_start(ap, fmt);
  n = vsprintf(buf, fmt, ap);
  va_end(ap);
  emit(0, "SPRINTF %s", buf);
  return n;
}

/* ---------------------------------------------------------------------------------------------- the script the recorders follow */
typedef struct {
  unsigned ret_h, ret_d; /* FAXVMI_process result of the V.21 receive channel (id 6) / of the data channel */
  int fr_len, fr_a;      /* V.21 channel: frame words written ([len][bytes]; < 0: buffer untouched), value stored to *n (< 0: untouched) */
  int d_k, d_n;          /* data channel: words written to the buffer (< 0: none), value stored to *n (< 0: untouched) */
  int ctl_ret;           /* FAXVMI_control result */
  int rms;               /* FPM_rms result */
  int agc_hist, agc_rcp, agc_shift; /* the V.21 engine's AGC words at the time of the process call */
  int create_null;       /* FAXVMI_create returns NULL */
  unsigned gen_ctr;
} Script;
static Script g_rs;
static unsigned g_serial = 100;
static unsigned g_tone_mark[4];
static unsigned rec_serial(const FaxVmi *o) { return o ? (unsigned)o->rxOverflow : 0u; }

#ifndef HARNESS_ENG
/* the FAKE engines of the recorder channels: typed objects at the blob's i386 layout (-m32) with real hdx / bundle pointers.  One pool per type; a create hands out the next slot. */
typedef struct { FaxVmiM m; V21Rx e; V21RxHdx h; V21RxBundle b; } F21;
typedef struct { FaxVmiM m; V27Rx e; V27RxHdx h; V27RxBundle b; } F27;
typedef struct { FaxVmiM m; V29Rx e; V29RxHdx h; V29RxBundle b; } F29;
typedef struct { FaxVmiM m; V17Rx e; V17RxHdx h; V17RxBundle b; } F17;
#define NFAKE 24
static F21 g_f21[NFAKE];
static F27 g_f27[NFAKE];
static F29 g_f29[NFAKE];
static F17 g_f17[NFAKE];
static unsigned g_n21, g_n27, g_n29, g_n17;
static void fakes_reset(void) { g_n21 = g_n27 = g_n29 = g_n17 = 0; }
static FaxVmiM *fake_make(int id)
{
  switch (id) {
    case FAXVMI_ID_V21RX: {
      F21 *f = &g_f21[g_n21++ % NFAKE];
      memset(f, 0, sizeof *f);
      f->e.hdx = &f->h;
      f->e.bundle = &f->b;
      f->m.eng = &f->e;
      return &f->m;
    }
    case FAXVMI_ID_V27RX: {
      F27 *f = &g_f27[g_n27++ % NFAKE];
      memset(f, 0, sizeof *f);
      f->e.hdx = &f->h;
      f->e.bundle = &f->b;
      f->m.eng = &f->e;
      return &f->m;
    }
    case FAXVMI_ID_V29RX: {
      F29 *f = &g_f29[g_n29++ % NFAKE];
      memset(f, 0, sizeof *f);
      f->e.hdx = &f->h;
      f->e.bundle = &f->b;
      f->m.eng = &f->e;
      return &f->m;
    }
    case FAXVMI_ID_V17RX: {
      F17 *f = &g_f17[g_n17++ % NFAKE];
      memset(f, 0, sizeof *f);
      f->e.hdx = &f->h;
      f->e.bundle = &f->b;
      f->m.eng = &f->e;
      return &f->m;
    }
    default: {
      static FaxVmiM none[NFAKE];
      static unsigned nn;
      FaxVmiM *m = &none[nn++ % NFAKE];
      memset(m, 0, sizeof *m);
      return m;
    }
  }
}
static void dump_fake(const char *tag, const FaxVmi *o)
{
  const FaxVmiM *m;
  if (!o || !o->m) return;
  m = o->m;
  switch (o->cfg.id) {
    case FAXVMI_ID_V21RX: {
      const V21Rx *e = (const V21Rx *)m->eng;
      emit(0, "%s FAKE%u v21 hist=%d rcp=%d shift=%d", tag, rec_serial(o), (int)e->bundle->agc.hist, (int)e->bundle->agc.rcp, (int)e->bundle->agc.shift);
      break;
    }
    case FAXVMI_ID_V27RX: {
      const V27Rx *e = (const V27Rx *)m->eng;
      emit(0, "%s FAKE%u v27 cfg2=%d rcp=%d shift=%d flag=%d s10=%d", tag, rec_serial(o), (int)e->cfg.s[2], (int)e->bundle->agc.rcp, (int)e->bundle->agc.shift, (int)e->hdx->s14, (int)m->u10.s10);
      break;
    }
    case FAXVMI_ID_V29RX: {
      const V29Rx *e = (const V29Rx *)m->eng;
      emit(0, "%s FAKE%u v29 cfg2=%d rcp=%d shift=%d flag=%d s10=%d", tag, rec_serial(o), (int)e->cfg.s[2], (int)e->bundle->agc.rcp, (int)e->bundle->agc.shift, (int)e->hdx->s1c, (int)m->u10.s10);
      break;
    }
    case FAXVMI_ID_V17RX: {
      const V17Rx *e = (const V17Rx *)m->eng;
      emit(0, "%s FAKE%u v17 cfg2=%d rcp=%d shift=%d flag=%d s10=%d", tag, rec_serial(o), (int)e->cfg.s[2], (int)e->bundle->agc.rcp, (int)e->bundle->agc.shift, (int)e->hdx->s20, (int)m->u10.s10);
      break;
    }
    default: break;
  }
}

/* ---------------------------------------------------------------------------------------------- callee recorders */
FaxVmi *FAXVMI_create(FaxVmi *o, const FaxVmiCfg *cfg)
{
  char b[24];
  emit(0, "VC o=%s cfg mode=%d flag=%d s08=%u s0a=%u s0c=%u id=%u mcfg=%s w14=%d", o ? "supplied" : "NULL", cfg->mode, cfg->flag, (unsigned)cfg->ringMin, (unsigned)cfg->maxItems, (unsigned)cfg->frmWords,
       (unsigned)cfg->id, dptr(cfg->mcfg, b, sizeof b), cfg->w14);
  if (g_rs.create_null) return NULL;
  if (!o) {
    o = (FaxVmi *)sysdep_malloc((unsigned)sizeof(FaxVmi));
    if (!o) return NULL;
  }
  memset(o, 0, sizeof *o);
  o->cfg = *cfg;
  o->rxOverflow = (int)g_serial++;
  o->m = fake_make(cfg->id);
  emit(0, "VC -> serial %u", rec_serial(o));
  return o;
}
void FAXVMI_delete(FaxVmi *o)
{
  emit(0, "VD serial %u id=%u", rec_serial(o), (unsigned)o->cfg.id);
  sysdep_free(o);
}
unsigned int FAXVMI_process(FaxVmi *o, unsigned short *buf, short *samples, unsigned short *n, unsigned short *cnt2)
{
  unsigned c = *cnt2, hd = o->cfg.id == FAXVMI_ID_V21RX;
  emit(0, "VP serial %u id=%u n=%u cnt2=%u buf=%016llx in=%016llx", rec_serial(o), (unsigned)o->cfg.id, (unsigned)*n, c, fnv_shorts((const short *)buf, 0x100), fnv_shorts(samples, c < 400 ? c : 400));
  g_rs.gen_ctr++;
  if (hd) {
    if (o->m && o->m->eng) {
      V21Rx *e = (V21Rx *)o->m->eng;
      e->bundle->agc.hist = (short)g_rs.agc_hist;
      e->bundle->agc.rcp = (short)g_rs.agc_rcp;
      e->bundle->agc.shift = (short)g_rs.agc_shift;
    }
    if (g_rs.fr_len >= 0) {
      int i;
      buf[0] = (unsigned short)g_rs.fr_len;
      for (i = 0; i < g_rs.fr_len && i < 0x1f0; i++) buf[1 + i] = (unsigned short)((g_rs.gen_ctr * 31u + (unsigned)i * 17u) & 0xff);
    }
    if (g_rs.fr_a >= 0) *n = (unsigned short)g_rs.fr_a;
    return g_rs.ret_h;
  }
  if (g_rs.d_k >= 0) {
    int i;
    for (i = 0; i < g_rs.d_k && i < 0x7f0; i++) buf[i] = (unsigned short)(g_rs.gen_ctr * 97u + (unsigned)i * 13u + 5u);
  }
  if (g_rs.d_n >= 0) *n = (unsigned short)g_rs.d_n;
  return g_rs.ret_d;
}
int FAXVMI_control(FaxVmi *o, const FaxVmiCtl *ctl)
{
  const int *m = (const int *)ctl->mctl;
  int nw = (o->cfg.id == FAXVMI_ID_V27RX || o->cfg.id == FAXVMI_ID_V17RX) ? 5 : 4;
  char b[96];
  int i, k = 0;
  emit(0, "VK serial %u id=%u ctl w0=%d w4=%d s8=%u pad0a=%u wc=%d s10=%u pad12=%u", rec_serial(o), (unsigned)o->cfg.id, ctl->flushRing, ctl->breakReq, (unsigned)ctl->breakCount, (unsigned)ctl->pad0a, ctl->rxReset,
       (unsigned)ctl->newMode, (unsigned)ctl->pad12);
  b[0] = 0;
  for (i = 0; i < nw; i++) k += snprintf(b + k, sizeof b - (size_t)k, " %08x", (unsigned)m[i]);
  emit(0, "VK mctl%s", b);
  return g_rs.ctl_ret;
}
int FAXVMI_status(FaxVmi *o, FaxVmiSts *st)
{
  (void)o; (void)st;
  die3("FAXVMI_status called");
}
const char *FAXVMI_message(FaxVmi *o, int idx)
{
  (void)o; (void)idx;
  return NULL;
}
int FPM_TONE_generate(FpmTone *tone, short *buf, short n)
{
  int i;
  emit(0, "TONE tone=%s n=%d", (const void *)tone == (const void *)g_tone_mark ? "mark" : "other", (int)n);
  for (i = 0; i < n && i < 400; i++) buf[i] = (short)(0x300 + i * 3);
  return 1;
}
short FPM_rms(const short *data, short count)
{
  emit(0, "RMS n=%d in=%016llx", (int)count, fnv_shorts(data, count > 0 && count < 400 ? (unsigned)count : 0u));
  return (short)g_rs.rms;
}
/* the recorder initialisers of the data receive classes: what the init_vmi_data_rx_modem functions do to the 0x18 byte block (a config block of the modem is allocated, its pointer is stored;
 * the V.17 one carries its three caller buffers) */
static int rec_init(int id, FaxVmiM *m, unsigned short rate, int unused, void *user)
{
  void *cfgblk;
  char b[24];
  emit(0, "INIT id=%d rate=%u unused=%d user=%s", id, (unsigned)rate, unused, dptr(user, b, sizeof b));
  if (id == FAXVMI_ID_V17RX) {
    V17RxCfg *c = (V17RxCfg *)sysdep_malloc((unsigned)sizeof(V17RxCfg));
    if (c) {
      memset(c, 0x17, sizeof *c);
      c->coefI = (short *)sysdep_malloc(0x62);
      c->coefQ = (short *)sysdep_malloc(0x62);
      c->freq = (short *)sysdep_malloc(2);
    }
    cfgblk = c;
  } else {
    cfgblk = sysdep_malloc(0x1c);
    if (cfgblk) memset(cfgblk, 0x11 * (id & 15), 0x1c);
  }
  m->w0 = 100 + id; /* the real initialisers store 0: any value is copied into the FAXVMI config */
  m->u4.init1 = 1;
  m->s08 = (short)(0x40 + id);
  m->s0a = (short)(0x25 + id);
  m->s0c = (short)(7 + id);
  m->s0e = (short)id;
  m->u10.cfg = cfgblk;
  m->eng = (void *)(uintptr_t)(0x1000 + id); /* (the real ones store `user`) */
  return id * 3 + (int)rate;
}
static int rec_init_v27(FaxVmiM *m, unsigned short r, int u, void *x) { return rec_init(FAXVMI_ID_V27RX, m, r, u, x); }
static int rec_init_v29(FaxVmiM *m, unsigned short r, int u, void *x) { return rec_init(FAXVMI_ID_V29RX, m, r, u, x); }
static int rec_init_v17(FaxVmiM *m, unsigned short r, int u, void *x) { return rec_init(FAXVMI_ID_V17RX, m, r, u, x); }
static void init_tab_default(void)
{
  INITTAB_RX[0] = rec_init_v27;
  INITTAB_RX[1] = rec_init_v29;
  INITTAB_RX[2] = rec_init_v17;
  INITTAB_TX[0] = rec_init_v27; /* never called here */
  INITTAB_TX[1] = rec_init_v29;
  INITTAB_TX[2] = rec_init_v17;
}
#endif /* !HARNESS_ENG */

/* ---------------------------------------------------------------------------------------------- objects, dumps */
/* the object sits behind a guard block: a state function writing below the object (buf[-1]) is seen */
static struct {
  unsigned short pre[16];
  FaxT30 o;
} g_wrap;
#define g_t30 (g_wrap.o)
static unsigned short g_obuf_dummy;

static const char *vmi_name(const FaxVmi *p, char *b, size_t cap)
{
#ifdef HARNESS_ENG
  return dptr(p, b, cap);
#else
  if (!p) snprintf(b, cap, "NULL");
  else snprintf(b, cap, "vmi%u", rec_serial(p));
  return b;
#endif
}
static void dump_t30(const char *tag, const FaxT30 *o)
{
  char b1[24], b2[24], b3[24], b4[24], b5[24], b6[24], b7[24], b8[24];
  emit(0, "%s T hdlc_tx=%s hdlc_rx=%s data_vmi=%s txcfg=%s rxcfg=%s data_cfg=%s tone=%s fifo=%s", tag, vmi_name(o->hdlc_tx, b1, sizeof b1), vmi_name(o->hdlc_rx, b2, sizeof b2),
       vmi_name(o->data_vmi, b3, sizeof b3), dptr(o->hdlc_tx_cfg, b4, sizeof b4), dptr(o->hdlc_rx_cfg, b5, sizeof b5), dptr(o->data_cfg, b6, sizeof b6),
       (const void *)o->tone == (const void *)g_tone_mark ? "mark" : (o->tone ? "other" : "NULL"), dptr(o->fifo, b7, sizeof b7));
  emit(0, "%s T w1218=%d state=%d prev=%d w1224=%d tmr=%d w122c=%d w1230=%d w1234=%d w1238=%d w123c=%d w1240=%d w1244=%d w1248=%d w124c=%d w1250=%d w1254=%d", tag, o->ans_org, o->state,
       o->state_prev, o->lastFrame, o->tmr, o->status, o->w1230, o->w1234, o->lastByte, o->rx_event, o->rx_kind, o->chan, o->modemClass, o->dlePending, o->hdlcWr, o->modem_sel);
  emit(0, "%s T w125c=%d w1260=%d w1264=%d w1268=%d w126c=%d w1270=%d sec=%d ms=%d w127c=%d w1280=%d w1284=%d w128c=%d w1290=%d w1294=%d w1298=%d", tag, o->cng_tone, o->cng_ctr,
       o->cng_on, o->ans_len, o->dataRate, o->enableDelay, o->sec, o->ms, o->rx_act, o->frameDone, o->frameCount, o->txEnable, o->frameWords, o->txConnected, o->fifoReady);
  emit(0, "%s T w129c=%d w12a0=%08x w12a4=%d w12a8=%08x w12ac=%d w12b0=%d w12b4=%d w12b8=%d w12bc=%d w12c0=%d w12c4=%d w12c8=%d w12cc=%d w12d0=%d w12d4=%d w12d8=%d p12dc=%s b12e0=%016llx w12f0=%d",
       tag, o->alignDone, o->alignHist, o->alignShift, o->alignMask, o->w12ac, o->dataEnd, o->s7, o->silenceTime, o->silence_rms, o->agc_rcp, o->agc_shift, o->emu_delay, o->emu_idx, o->sf_fill, o->cd_flag, o->gain_att,
       dptr(o->hpCoef, b8, sizeof b8), fnv_bytes(o->hpState, sizeof o->hpState), o->hp_on);
  emit(0, "%s T buf=%016llx cnt=%d hdi=%d guard=%016llx", tag, fnv_shorts((const short *)o->buf, 0x900), COUNTER, HDI_COUNTER, o == &g_t30 ? fnv_shorts((const short *)g_wrap.pre, 16) : 0ull);
#ifndef HARNESS_ENG
  dump_fake(tag, o->hdlc_rx);
  dump_fake(tag, o->data_vmi);
#endif
}
/* a fresh object: every non-pointer member deterministic garbage (member by member, so the number of PRNG draws is the same on both hosts), the pointers cleared */
#define G32(f) o->f = (int)rnd()
static void t30_garbage(FaxT30 *o)
{
  unsigned i, r = 0;
  for (i = 0; i < 16; i++) g_wrap.pre[i] = (unsigned short)(0x7a00 + i);
  for (i = 0; i < 0x900; i++) {
    if ((i & 1) == 0) r = rnd();
    o->buf[i] = (unsigned short)(r >> (16 * (i & 1)));
  }
  o->hdlc_tx = o->hdlc_rx = o->data_vmi = NULL;
  o->hdlc_tx_cfg = o->hdlc_rx_cfg = NULL;
  o->data_cfg = NULL;
  o->tone = NULL;
  o->fifo = NULL;
  o->hpCoef = NULL;
  G32(ans_org); G32(state); G32(state_prev); G32(lastFrame); G32(tmr); G32(status); G32(w1230); G32(w1234); G32(lastByte); G32(rx_event); G32(rx_kind); G32(chan); G32(modemClass); G32(dlePending);
  G32(hdlcWr); G32(modem_sel); G32(cng_tone); G32(cng_ctr); G32(cng_on); G32(ans_len); G32(dataRate); G32(enableDelay); G32(sec); G32(ms); G32(rx_act); G32(frameDone); G32(frameCount); G32(txEnable); G32(frameWords);
  G32(txConnected); G32(fifoReady); G32(alignDone); o->alignHist = rnd(); G32(alignShift); o->alignMask = rnd(); G32(w12ac); G32(dataEnd); G32(s7); G32(silenceTime); G32(silence_rms); G32(agc_rcp); G32(agc_shift); G32(emu_delay);
  G32(emu_idx); G32(sf_fill); G32(cd_flag); G32(gain_att); G32(hp_on);
  for (i = 0; i < sizeof o->hpState; i++) {
    if ((i & 3) == 0) r = rnd();
    o->hpState[i] = (unsigned char)(r >> (8 * (i & 3)));
  }
  /* small garbage in the words some functions use as sizes / indices (the garbage must not make a function write outside the object) */
  o->hdlcWr = (int)rmod(0x100);
  o->s7 = (int)rmod(40);
  o->sec = (int)rmod(100);
  o->ms = (int)rmod(100);
}
#ifndef HARNESS_ENG
static FaxVmi *new_vmi(int id)
{
  FaxVmiCfg c;
  memset(&c, 0, sizeof c);
  c.mode = 2;
  c.id = (unsigned short)id;
  c.ringMin = 0x60;
  c.maxItems = 0x34;
  return FAXVMI_create(NULL, &c);
}
/* an object with recorder channels, as fax_class1_create leaves it (the HDLC channels) plus an optional data receive channel of the given VMI id (8 / 10 / 12) */
static void t30_channels(FaxT30 *o, int data_id)
{
  o->hdlc_tx = new_vmi(5);
  o->hdlc_rx = new_vmi(6);
  o->data_vmi = data_id >= 0 ? new_vmi(data_id) : NULL;
  o->tone = (FpmTone *)(void *)g_tone_mark;
}
static void t30_free_channels(FaxT30 *o)
{
  if (o->hdlc_tx) FAXVMI_delete(o->hdlc_tx);
  if (o->hdlc_rx) FAXVMI_delete(o->hdlc_rx);
  if (o->data_vmi) FAXVMI_delete(o->data_vmi);
  o->hdlc_tx = o->hdlc_rx = o->data_vmi = NULL;
  if (o->data_cfg) {
    if ((unsigned short)o->data_cfg->s0e == FAXVMI_ID_V17RX) {
      V17RxCfg *c = (V17RxCfg *)o->data_cfg->u10.cfg;
      sysdep_free(c->freq);
      sysdep_free(c->coefQ);
      sysdep_free(c->coefI);
    }
    sysdep_free(o->data_cfg->u10.cfg);
    sysdep_free(o->data_cfg);
    o->data_cfg = NULL;
  }
}
static void trial_end2(const char *what)
{
  trial_end(what);
  fakes_reset();
}
#endif

/* the I/O block of a state function call */
typedef struct {
  short rx[1024], tx[1024];
  unsigned char tty_out[1024], tty_in[1024];
  int n_in, n_out, p7, p8;
} Io;
static Io g_io;
static void io_prep(Io *io, int n_in, int n_out, int p7, int p8)
{
  int i;
  for (i = 0; i < 1024; i++) {
    io->rx[i] = (short)rnd();
    io->tx[i] = (short)0x5a5a;
    io->tty_out[i] = 0xee;
    io->tty_in[i] = (unsigned char)rnd();
  }
  io->n_in = n_in;
  io->n_out = n_out;
  io->p7 = p7;
  io->p8 = p8;
}
static int call_state(StateFn fn, FaxT30 *o, Io *io, const char *tag)
{
  int r;
  r = fn(o, io->rx, io->tx, io->tty_out, io->tty_in, &io->n_in, &io->n_out, &io->p7, &io->p8);
  emit(0, "%s R r=%d n_in=%d n_out=%d p7=%d p8=%d tx=%016llx rx=%016llx out=%016llx", tag, r, io->n_in, io->n_out, io->p7, io->p8, fnv_shorts(io->tx, 1024), fnv_shorts(io->rx, 1024),
       fnv_bytes(io->tty_out, sizeof io->tty_out));
  dump_t30(tag, o);
  return r;
}

#ifndef HARNESS_ENG
static void poke_consts(int randomize)
{
  FaxVmiCtl c;
  FaxVmiSts s;
  int a[5];
  int i;
  memset(&c, 0, sizeof c);
  memset(&s, 0, sizeof s);
  if (randomize) {
    c.flushRing = (int)rnd();
    c.breakReq = (int)rnd();
    c.breakCount = (unsigned short)rnd();
    c.pad0a = (unsigned short)rnd();
    c.rxReset = (int)rnd();
    c.newMode = (unsigned short)rnd();
    c.pad12 = (unsigned short)rnd();
    s.ringFree = (short)rnd();
    s.pad2 = (short)rnd();
    s.ovf = (int)rnd();
    s.txEmpty = (int)rnd();
    s.rxOverflow = (int)rnd();
    s.rxBreak = (int)rnd();
    s.hskipDone = (int)rnd();
    s.msts = NULL;
  }
  poke_bytes(h_FAXVMI_CTL, &c, sizeof c);
  poke_bytes(h_FAXVMI_STS, &s, sizeof s);
  if (randomize) {
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V21RX_CTL, a, sizeof h_V21RX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V27RX_CTL, a, sizeof h_V27RX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V29RX_CTL, a, sizeof h_V29RX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V17RX_CTL, a, sizeof h_V17RX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V21TX_CTL, a, sizeof h_V21TX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V27TX_CTL, a, sizeof h_V27TX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V29TX_CTL, a, sizeof h_V29TX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V17TX_CTL, a, sizeof h_V17TX_CTL);
  } else {
    static const int v21rx[4] = {0x12c0000, 0xea60, 0, 0}, v27[5] = {0x2580, 0xea60, 1, 0, 0}, v29[4] = {0x2580, 0xea60, 1, 0}, v17[5] = {0x3840, 0xea60, 1, 0, 0}, v21tx[5] = {0x12c, 0xea60, 0xc80, 0, 0};
    poke_bytes(h_V21RX_CTL, v21rx, sizeof v21rx);
    poke_bytes(h_V27RX_CTL, v27, sizeof v27);
    poke_bytes(h_V29RX_CTL, v29, sizeof v29);
    poke_bytes(h_V17RX_CTL, v17, sizeof v17);
    poke_bytes(h_V21TX_CTL, v21tx, sizeof v21tx);
    poke_bytes(h_V27TX_CTL, v27, sizeof v27);
    poke_bytes(h_V29TX_CTL, v29, sizeof v29);
    poke_bytes(h_V17TX_CTL, v17, sizeof v17);
  }
}
static void dump_consts(void)
{
  const FaxVmiCtl *c = (const FaxVmiCtl *)(const void *)h_FAXVMI_CTL;
  emit(1, "FAXVMI_CTL w0=%d w4=%d s8=%u pad0a=%u wc=%d s10=%u pad12=%u", c->flushRing, c->breakReq, (unsigned)c->breakCount, (unsigned)c->pad0a, c->rxReset, (unsigned)c->newMode, (unsigned)c->pad12);
  emit(1, "V21RX_CTL %08x %08x %08x %08x", h_V21RX_CTL[0], h_V21RX_CTL[1], h_V21RX_CTL[2], h_V21RX_CTL[3]);
  emit(1, "V27RX_CTL %08x %08x %08x %08x %08x", h_V27RX_CTL[0], h_V27RX_CTL[1], h_V27RX_CTL[2], h_V27RX_CTL[3], h_V27RX_CTL[4]);
  emit(1, "V29RX_CTL %08x %08x %08x %08x", h_V29RX_CTL[0], h_V29RX_CTL[1], h_V29RX_CTL[2], h_V29RX_CTL[3]);
  emit(1, "V17RX_CTL %08x %08x %08x %08x %08x", h_V17RX_CTL[0], h_V17RX_CTL[1], h_V17RX_CTL[2], h_V17RX_CTL[3], h_V17RX_CTL[4]);
}

#endif

/* the state table of fax_class1_progress as far as R29 / R28 own it (index = o->state) */
static StateFn state_fn(int st)
{
  switch (st) {
    case 4: return _hdlc_receive_look_carrier_state;
    case 5: return _hdlc_receive_state;
    case 6: return _hdlc_receive_between_buffers_state;
    case 7: return _hdlc_emulate_receive_state;
    case 8: return T_IDLE;
    case 12: return _rx_look_carrier_state;
    case 13: return _rx_data_state;
    case 16: return T_RSIL;
    default: return NULL;
  }
}
/* the clock of fax_class1_progress: 2 per call in the hundredths, seconds when it reaches 100 */
static void clock_tick(FaxT30 *o)
{
  if (o->ms + 2 > 99) {
    o->sec++;
    o->ms = 0;
  } else {
    o->ms += 2;
  }
}
static const int k_mods[] = {0x18, 0x30, 0x48, 0x60, 0x49, 0x4a, 0x61, 0x62, 0x79, 0x7a, 0x91, 0x92};
#define NMODS ((int)(sizeof k_mods / sizeof k_mods[0]))
static int mod_class_id(int mod) /* the VMI id of the receiver the modem value creates */
{
  if (mod == 0x18 || mod == 0x30) return FAXVMI_ID_V27RX;
  if (mod == 0x48 || mod == 0x60) return FAXVMI_ID_V29RX;
  return FAXVMI_ID_V17RX;
}

#ifndef HARNESS_ENG
/* script helpers (each draw its own statement) */
static void script_random(void)
{
  unsigned a, b, c, d;
  int k;
  memset(&g_rs, 0, sizeof g_rs);
  a = rnd();
  g_rs.ret_h = a;
  b = rnd();
  g_rs.ret_d = b;
  k = (int)rmod(4);
  if (k == 0) g_rs.ret_d = (g_rs.ret_d & ~0x300u) | 0x100u;
  else if (k == 1) g_rs.ret_d &= ~0x300u;
  if (rpct(50)) g_rs.ret_h |= 0x2000u;
  else g_rs.ret_h &= ~0x2000u;
  if (rpct(50)) g_rs.ret_d |= 0x2000u;
  else g_rs.ret_d &= ~0x2000u;
  c = rmod(10);
  g_rs.fr_len = c == 0 ? -1 : (c < 3 ? 0 : (c < 9 ? 1 + (int)rmod(14) : (int)rmod(0x120)));
  d = rmod(10);
  g_rs.fr_a = d < 5 ? 0 : (d < 9 ? 1 + (int)rmod(3) : -1);
  g_rs.d_k = rpct(20) ? -1 : (rpct(25) ? 0 : (int)rmod(60));
  g_rs.d_n = rpct(15) ? -1 : (rpct(60) ? (g_rs.d_k > 0 ? g_rs.d_k : 0) : (int)rmod(200));
  g_rs.ctl_ret = (int)(rnd() & 0xffff);
  {
    unsigned r = rmod(8);
    g_rs.rms = r < 2 ? (int)rmod(60) : (r == 2 ? 100 : (r == 3 ? 101 : (r == 4 ? -(int)rmod(300) : (int)(short)rnd())));
  }
  {
    static const int thr[] = {514, 727, 1026, 1450};
    unsigned r = rmod(10);
    g_rs.agc_hist = r < 6 ? thr[rmod(4)] + (int)rmod(3) - 1 : (r < 8 ? (int)rmod(2000) : (int)(short)rnd());
  }
  g_rs.agc_rcp = (int)(short)rnd();
  g_rs.agc_shift = (int)(short)rnd();
}
/* a random 32 bit timer value with emphasis on the boundaries of the states */
static int pick_tmr(const int *edges, int ne)
{
  int k = (int)rmod(10);
  if (k < 5) {
    int e = edges[rmod((unsigned)ne)];
    int d = (int)rmod(5) - 2;
    return e + d;
  }
  if (k == 5) return (int)rnd();
  if (k == 6) return -(int)rmod(5);
  return (int)rmod(300);
}
#endif

#ifndef HARNESS_ENG
#include "r29_in.h"
#include "r29_hdlc.h"
#include "r29_data.h"
#else
#include "r29_eng.h"
#endif

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R29_SEED");
  g_verbose = getenv("R29_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)strtoul(sd, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)scen * 7919u;
  cov_setup(scen);
#ifndef HARNESS_ENG
  init_tab_default();
#endif
  htab_add("tone", g_tone_mark, sizeof g_tone_mark);
  htab_add("obuf", &g_obuf_dummy, sizeof g_obuf_dummy);
#ifndef HARNESS_ENG
  switch (scen) {
    case 0: scen_input(); break;
    case 1: scen_small(); break;
    case 2: scen_hdlc(); break;
    case 3: scen_data(); break;
    case 4: scen_life(); break;
    default: fprintf(stderr, "scenario 0..4\n"); return 2;
  }
#else
  switch (scen) {
    case 5: scen_eng_hdlc(); break;
    case 6: scen_eng_data(); break;
    case 7: scen_eng_life(); break;
    default: fprintf(stderr, "scenario 5..7\n"); return 2;
  }
#endif
  cov_finish();
  return 0;
}
