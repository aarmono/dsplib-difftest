/*
 * R28 differential harness: the T.30 / HDLC TRANSMIT side of the fax class 1 object of dsplibs.o (the transmit state machines, their initialisers, _init_transmitter /
 * _delete_data_tx_modem, the ctools output helpers, GetT30FrameIDFromBuffer / GetT30FrameNameByID, the shared idle / silence / answer tone states) vs src/fax/fax_t30_tx.c +
 * src/fax/fax_t30_tables.c.  The blob run (-DTEST_BLOB, linked with ../dsplibs.o) and the port run (-m32 -O0 / -O2; x86-64 against the recorded blob output) execute the same code; library
 * test/fax/r28_common.h (hashed canonical output, coverage counters exit 2 on a zero, 3 = harness violation, deterministic heap with block ids / red zones / poisoned frees).
 * Everything printed is host independent: objects are dumped member by member, pointers as block / recorder serial numbers, buffers as hashes.
 *
 * REC build (default): the callees of the code under test are RECORDERS with scripted results (the blob's copies are weakened with objcopy, see fax_t30_tx.*.sh):
 *   FAXVMI_create / _delete / _process / _control / _status (arguments logged, the modem records behind FaxVmiCtl.mctl dumped, the state word / status record scripted),
 *   _handle_data_input / _handle_hdlc_input / _handle_hdlc_input_open (R29's: scripted word counts, results, flag side effects),  FPM_TONE_generate, the init_vmi_data_tx_modem
 *   table entries (recorder initialisers, every modem id), sysdep_malloc / _free (block ids, red zones), sysdep_sprintf, dsplibs_debug_printf.  The real FIFO_* run on both sides (R18).
 *   The constant records FAXVMI_CTL / FAXVMI_STS / V21TX_CTL ... are POKED with random patterns (the blob's in .rodata through mprotect) so that every copied word is observable.
 *   Scenarios:
 *     0 T30frames + ctools: GetT30FrameIDFromBuffer (all 0xff,b1,b2), GetT30FrameNameByID, _put_silence, _handle_data_output, cTOOLS_handle_hdlc_output, ..._output_reset;
 *     1 shared states: _idle_state(_init), _send_silence_state(_init), _answer_tone_state, _init_tx_nulls_state, cHDLCtx_preamble_state_init / _off_init, _send_hdlc_between_buffer_state_init;
 *     2 HDLC states: _t30_silence_before_tx_state, _t30_preabmle_state, _send_hdlc_buffer_state, _send_hdlc_between_buffer_state, cHDLCtx_off (single calls and dispatched sequences);
 *     3 data transmit states: _tx_scrambled_ones_state, _tx_data_state, _tx_nulls_state, _tx_silence_before_scrm_ones (single calls and dispatched sequences, real FIFO);
 *     4 data transmitter life cycle: _init_transmitter (every modem value, every object state, recorder initialisers of every modem id), _tx_scrambled_ones_init,
 *       _delete_data_tx_modem, forked malloc-failure probes.
 * R28_VERBOSE=1 prints every line (debugging), R28_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r28_common.h"

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
extern int cDATAtx_counter;
extern FaxVmiInitFn init_vmi_data_tx_modem[3];
#define T_IDLE _idle_state
#define T_SILENCE _send_silence_state
#define T_ANSWER _answer_tone_state
#define COUNTER cDATAtx_counter
#define INITTAB init_vmi_data_tx_modem
#ifndef HARNESS_ENG
extern unsigned char h_FAXVMI_CTL[] __asm__("FAXVMI_CTL");
extern unsigned char h_FAXVMI_STS[] __asm__("FAXVMI_STS");
extern int h_V21TX_CTL[5] __asm__("V21TX_CTL");
extern int h_V21RX_CTL[4] __asm__("V21RX_CTL");
extern int h_V27TX_CTL[5] __asm__("V27TX_CTL");
extern int h_V29TX_CTL[4] __asm__("V29TX_CTL");
extern int h_V17TX_CTL[5] __asm__("V17TX_CTL");
#endif
#else
#define T_IDLE fax_idle_state
#define T_SILENCE fax_send_silence_state
#define T_ANSWER fax_answer_tone_state
#define COUNTER fax_cDATAtx_counter
#define INITTAB fax_init_vmi_data_tx_modem
#ifndef HARNESS_ENG
FaxVmiInitFn fax_init_vmi_data_tx_modem[3];
/* the constant records the port references, defined here (writable, bound to the blob's symbol names; sized for the x86-64 layouts of FaxVmiCtl / FaxVmiSts) */
unsigned char h_FAXVMI_CTL[64] __asm__("FAXVMI_CTL");
unsigned char h_FAXVMI_STS[64] __asm__("FAXVMI_STS");
int h_V21TX_CTL[5] __asm__("V21TX_CTL") = {0x12c, 0xea60, 0xc80, 0, 0};
int h_V21RX_CTL[4] __asm__("V21RX_CTL") = {0x12c0000, 0xea60, 0, 0};
int h_V27TX_CTL[5] __asm__("V27TX_CTL") = {0x2580, 0xea60, 1, 0, 0};
int h_V29TX_CTL[4] __asm__("V29TX_CTL") = {0x2580, 0xea60, 1, 0};
int h_V17TX_CTL[5] __asm__("V17TX_CTL") = {0x3840, 0xea60, 1, 0, 0};
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

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  /* 0 misc */
  CV_M_ID, CV_M_ID_NOT_FF, CV_M_ID_B1_3, CV_M_ID_B1_13, CV_M_ID_OTHER, CV_M_ID_BIG, CV_M_ID_SMALL, CV_M_NAME, CV_M_NAME_KNOWN, CV_M_NAME_UNK, CV_M_NAME_BUF, CV_M_SIL, CV_M_SIL_NEG, CV_M_SIL_0,
  CV_M_RESET, CV_M_DO, CV_M_DO_ALIGN, CV_M_DO_SKIP, CV_M_DO_RUN, CV_M_DO_DLE, CV_M_DO_FLAG, CV_M_DO_N0, CV_M_DO_NEG, CV_M_DO_MID, CV_M_HO, CV_M_HO_NAME, CV_M_HO_DLE, CV_M_HO_FLAG, CV_M_HO_N0,
  CV_M_HO_N1, CV_M_HO_N3, CV_M_HO_DBG, CV_M_HO_HIGH, CV_M_HO_NEG,
  /* 1 shared states */
  CV_S_IDLE_N, CV_S_IDLE_0, CV_S_IDLE_NEG, CV_S_SIL, CV_S_SIL_END, CV_S_SILI, CV_S_SILI_ZERO, CV_S_ANS, CV_S_ANS_TONE, CV_S_ANS_END, CV_S_ANS_NEG, CV_S_PRE, CV_S_PRE_POKE, CV_S_OFFI,
  CV_S_HBI, CV_S_NULLI, CV_S_IDLEI, CV_S_CTL_V21TX, CV_S_CTL_V21RX,
  /* 2 hdlc */
  CV_H_SB, CV_H_SB_EMPTY, CV_H_SB_BUSY, CV_H_SB_DBG, CV_H_PRE, CV_H_PRE_50, CV_H_PRE_DBG, CV_H_PP, CV_H_PP_IN, CV_H_PP_FRAME, CV_H_PP_1S, CV_H_PP_1S_NOFRAME, CV_H_PP_5S, CV_H_PP_DBG3,
  CV_H_PP_NOIN, CV_H_PP_ZERO, CV_H_HB, CV_H_HB_T2, CV_H_HB_T2_IDLE, CV_H_HB_T2_OTHER, CV_H_HB_IN, CV_H_HB_FRAME, CV_H_HB_NOFRAME, CV_H_HB_T1, CV_H_HB_TO, CV_H_HB_DBG, CV_H_OFF, CV_H_OFF_BIT,
  CV_H_OFF_NOBIT, CV_H_OFF_END2, CV_H_OFF_END15, CV_H_OFF_DBG, CV_H_OFF_STAY, CV_H_SEQ, CV_H_SEQ_LONG, CV_H_SEQ_STATE0, CV_H_SEQ_STATE1, CV_H_SEQ_STATE2, CV_H_SEQ_STATE3, CV_H_SEQ_STATE8,
  CV_H_SEQ_OFF,
  /* 3 data tx */
  CV_D_ONES, CV_D_ONES_FIRST, CV_D_ONES_DEC, CV_D_ONES_ENABLE, CV_D_ONES_DATA, CV_D_ONES_FULL, CV_D_ONES_NODATA, CV_D_ONES_GO, CV_D_ONES_CONNECT, CV_D_ONES_UNDER, CV_D_ONES_DBG3,
  CV_D_DATA, CV_D_DATA_IN, CV_D_DATA_FULL, CV_D_DATA_UNDER, CV_D_DATA_QEMPTY_STOP, CV_D_DATA_QEMPTY_NULLS, CV_D_DATA_QFULL, CV_D_DATA_DBG3, CV_D_NULLS, CV_D_NULLS_TO, CV_D_NULLS_IN,
  CV_D_NULLS_NOIN, CV_D_NULLS_FULL, CV_D_NULLS_UNDER, CV_D_NULLS_NEG, CV_D_SIL, CV_D_SIL_END, CV_D_SEQ, CV_D_SEQ_STATE9, CV_D_SEQ_STATE10, CV_D_SEQ_STATE11, CV_D_SEQ_STATE18, CV_D_SEQ_STATE8,
  CV_D_SEQ_CONNECT,
  /* 4 life */
  CV_L_INIT, CV_L_M18, CV_L_M30, CV_L_M48, CV_L_M60, CV_L_M49, CV_L_M4A, CV_L_M61, CV_L_M62, CV_L_M79, CV_L_M7A, CV_L_M91, CV_L_M92, CV_L_FRESH, CV_L_REPLACE, CV_L_REUSE, CV_L_SKIP,
  CV_L_CTL_V17, CV_L_CTL_V29, CV_L_CTL_V27, CV_L_NOCTL, CV_L_SIL0, CV_L_SIL1, CV_L_DBG, CV_L_ONESINIT, CV_L_DEL, CV_L_DEL_NOFIFO, CV_L_PROBE, CV_L_SHORT, CV_L_FIFO_REUSE,
  /* 5 .. 7 ENG */
  CV_E_HSESS, CV_E_H_SIL, CV_E_H_PRE, CV_E_H_BUF, CV_E_H_BTW, CV_E_H_FRAME, CV_E_H_SAMPLES, CV_E_H_OFF, CV_E_H_RXBIT, CV_E_H_LONG, CV_E_DSESS, CV_E_D_V27, CV_E_D_V29, CV_E_D_V17, CV_E_D_CONNECT,
  CV_E_D_DATA, CV_E_D_NULLS, CV_E_D_SAMPLES, CV_E_D_SHORT, CV_E_LIFE, CV_E_L_MODS, CV_E_L_REPLACE, CV_E_L_DEL
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_M_ID, "GetT30FrameIDFromBuffer");
      cov_def(CV_M_ID_NOT_FF, "id: first byte != 0xff");
      cov_def(CV_M_ID_B1_3, "id: b1 = 3");
      cov_def(CV_M_ID_B1_13, "id: b1 = 0x13");
      cov_def(CV_M_ID_OTHER, "id: other b1");
      cov_def(CV_M_ID_BIG, "id: value > 0x84 masked");
      cov_def(CV_M_ID_SMALL, "id: value <= 0x84");
      cov_def(CV_M_NAME, "GetT30FrameNameByID");
      cov_def(CV_M_NAME_KNOWN, "name: known id");
      cov_def(CV_M_NAME_UNK, "name: unknown id (sprintf)");
      cov_def(CV_M_NAME_BUF, "name: static buffer reused");
      cov_def(CV_M_SIL, "_put_silence");
      cov_def(CV_M_SIL_NEG, "_put_silence n < 0");
      cov_def(CV_M_SIL_0, "_put_silence n = 0");
      cov_def(CV_M_RESET, "cTOOLS_handle_data_output_reset");
      cov_def(CV_M_DO, "_handle_data_output");
      cov_def(CV_M_DO_ALIGN, "do: alignment found (w129c set)");
      cov_def(CV_M_DO_SKIP, "do: no zero bit (eight shifts)");
      cov_def(CV_M_DO_RUN, "do: aligned extraction");
      cov_def(CV_M_DO_DLE, "do: DLE doubled");
      cov_def(CV_M_DO_FLAG, "do: DLE ETX appended");
      cov_def(CV_M_DO_N0, "do: n = 0");
      cov_def(CV_M_DO_NEG, "do: n < 0");
      cov_def(CV_M_DO_MID, "do: alignment found after skips");
      cov_def(CV_M_HO, "cTOOLS_handle_hdlc_output");
      cov_def(CV_M_HO_NAME, "ho: frame name printed");
      cov_def(CV_M_HO_DLE, "ho: DLE doubled");
      cov_def(CV_M_HO_FLAG, "ho: DLE ETX appended");
      cov_def(CV_M_HO_N0, "ho: n = 0");
      cov_def(CV_M_HO_N1, "ho: n = 1");
      cov_def(CV_M_HO_N3, "ho: n > 2");
      cov_def(CV_M_HO_DBG, "ho: debug lines");
      cov_def(CV_M_HO_HIGH, "ho: word above 0xff");
      cov_def(CV_M_HO_NEG, "ho: n < 0");
      break;
    case 1:
      cov_def(CV_S_IDLE_N, "idle state n > 0");
      cov_def(CV_S_IDLE_0, "idle state n = 0");
      cov_def(CV_S_IDLE_NEG, "idle state n < 0");
      cov_def(CV_S_SIL, "send silence state");
      cov_def(CV_S_SIL_END, "send silence state ends (timer 0)");
      cov_def(CV_S_SILI, "send silence init");
      cov_def(CV_S_SILI_ZERO, "send silence init half = 0");
      cov_def(CV_S_ANS, "answer tone state");
      cov_def(CV_S_ANS_TONE, "answer tone generated");
      cov_def(CV_S_ANS_END, "answer tone ends (preamble init)");
      cov_def(CV_S_ANS_NEG, "answer tone negative timer");
      cov_def(CV_S_PRE, "cHDLCtx_preamble_state_init");
      cov_def(CV_S_PRE_POKE, "preamble init with poked records");
      cov_def(CV_S_OFFI, "cHDLCtx_off_init");
      cov_def(CV_S_HBI, "_send_hdlc_between_buffer_state_init");
      cov_def(CV_S_NULLI, "_init_tx_nulls_state");
      cov_def(CV_S_IDLEI, "_idle_state_init");
      cov_def(CV_S_CTL_V21TX, "control record V.21 tx");
      cov_def(CV_S_CTL_V21RX, "control record V.21 rx");
      break;
    case 2:
      cov_def(CV_H_SB, "_send_hdlc_buffer_state");
      cov_def(CV_H_SB_EMPTY, "send buffer: queue empty (w8 = 1)");
      cov_def(CV_H_SB_BUSY, "send buffer: busy");
      cov_def(CV_H_SB_DBG, "send buffer: debug");
      cov_def(CV_H_PRE, "_t30_silence_before_tx_state");
      cov_def(CV_H_PRE_50, "silence: 50 ms elapsed");
      cov_def(CV_H_PRE_DBG, "silence: debug");
      cov_def(CV_H_PP, "_t30_preabmle_state");
      cov_def(CV_H_PP_IN, "preamble: data bytes");
      cov_def(CV_H_PP_FRAME, "preamble: frame complete");
      cov_def(CV_H_PP_1S, "preamble: 1 s with frames");
      cov_def(CV_H_PP_1S_NOFRAME, "preamble: 1 s without frames");
      cov_def(CV_H_PP_5S, "preamble: 5 s");
      cov_def(CV_H_PP_DBG3, "preamble: collect debug");
      cov_def(CV_H_PP_NOIN, "preamble: no data");
      cov_def(CV_H_PP_ZERO, "preamble: frame not complete");
      cov_def(CV_H_HB, "_send_hdlc_between_buffer_state");
      cov_def(CV_H_HB_T2, "between: timer = 2");
      cov_def(CV_H_HB_T2_IDLE, "between: timer = 2, w1224 = 1");
      cov_def(CV_H_HB_T2_OTHER, "between: timer = 2, w1224 != 1");
      cov_def(CV_H_HB_IN, "between: data bytes");
      cov_def(CV_H_HB_FRAME, "between: frame complete");
      cov_def(CV_H_HB_NOFRAME, "between: frame not complete");
      cov_def(CV_H_HB_T1, "between: timer <= 1 with data");
      cov_def(CV_H_HB_TO, "between: timeout (5 s)");
      cov_def(CV_H_HB_DBG, "between: debug");
      cov_def(CV_H_OFF, "cHDLCtx_off");
      cov_def(CV_H_OFF_BIT, "off: state bit 0x2000");
      cov_def(CV_H_OFF_NOBIT, "off: no state bit");
      cov_def(CV_H_OFF_END2, "off: ends at timer > 2");
      cov_def(CV_H_OFF_END15, "off: ends at timer > 15");
      cov_def(CV_H_OFF_DBG, "off: debug");
      cov_def(CV_H_OFF_STAY, "off: stays");
      cov_def(CV_H_SEQ, "dispatched sequences");
      cov_def(CV_H_SEQ_LONG, "long sequence");
      cov_def(CV_H_SEQ_STATE0, "sequence reached state 0");
      cov_def(CV_H_SEQ_STATE1, "sequence reached state 1");
      cov_def(CV_H_SEQ_STATE2, "sequence reached state 2");
      cov_def(CV_H_SEQ_STATE3, "sequence reached state 3");
      cov_def(CV_H_SEQ_STATE8, "sequence reached state 8");
      cov_def(CV_H_SEQ_OFF, "sequence ran cHDLCtx_off");
      break;
    case 3:
      cov_def(CV_D_ONES, "_tx_scrambled_ones_state");
      cov_def(CV_D_ONES_FIRST, "ones: first call (counter 0)");
      cov_def(CV_D_ONES_DEC, "ones: w1270 decremented");
      cov_def(CV_D_ONES_ENABLE, "ones: ENABLE_TRANSMIT");
      cov_def(CV_D_ONES_DATA, "ones: data bytes");
      cov_def(CV_D_ONES_FULL, "ones: fifo full");
      cov_def(CV_D_ONES_NODATA, "ones: no data");
      cov_def(CV_D_ONES_GO, "ones: fifo read (state 10)");
      cov_def(CV_D_ONES_CONNECT, "ones: Tx connect");
      cov_def(CV_D_ONES_UNDER, "ones: fifo under run");
      cov_def(CV_D_ONES_DBG3, "ones: counter debug");
      cov_def(CV_D_DATA, "_tx_data_state");
      cov_def(CV_D_DATA_IN, "data: data bytes");
      cov_def(CV_D_DATA_FULL, "data: fifo full");
      cov_def(CV_D_DATA_UNDER, "data: fifo underrun");
      cov_def(CV_D_DATA_QEMPTY_STOP, "data: queue underrun, stop");
      cov_def(CV_D_DATA_QEMPTY_NULLS, "data: queue underrun, nulls");
      cov_def(CV_D_DATA_QFULL, "data: queue full");
      cov_def(CV_D_DATA_DBG3, "data: counter debug");
      cov_def(CV_D_NULLS, "_tx_nulls_state");
      cov_def(CV_D_NULLS_TO, "nulls: 5 s timeout");
      cov_def(CV_D_NULLS_IN, "nulls: data bytes");
      cov_def(CV_D_NULLS_NOIN, "nulls: no data");
      cov_def(CV_D_NULLS_FULL, "nulls: fifo full");
      cov_def(CV_D_NULLS_UNDER, "nulls: fifo under run");
      cov_def(CV_D_NULLS_NEG, "nulls: negative count");
      cov_def(CV_D_SIL, "_tx_silence_before_scrm_ones");
      cov_def(CV_D_SIL_END, "silence before ones: ends");
      cov_def(CV_D_SEQ, "dispatched sequences");
      cov_def(CV_D_SEQ_STATE9, "sequence reached state 9");
      cov_def(CV_D_SEQ_STATE10, "sequence reached state 10");
      cov_def(CV_D_SEQ_STATE11, "sequence reached state 11");
      cov_def(CV_D_SEQ_STATE18, "sequence reached state 18");
      cov_def(CV_D_SEQ_STATE8, "sequence reached state 8");
      cov_def(CV_D_SEQ_CONNECT, "sequence connected");
      break;
#ifdef HARNESS_ENG
    case 5:
      cov_def(CV_E_HSESS, "HDLC sessions");
      cov_def(CV_E_H_SIL, "silence before TX state");
      cov_def(CV_E_H_PRE, "preamble state");
      cov_def(CV_E_H_BUF, "send buffer state");
      cov_def(CV_E_H_BTW, "between buffers state");
      cov_def(CV_E_H_FRAME, "a frame was queued");
      cov_def(CV_E_H_SAMPLES, "V.21 samples produced");
      cov_def(CV_E_H_OFF, "cHDLCtx_off");
      break;
    case 6:
      cov_def(CV_E_DSESS, "data sessions");
      cov_def(CV_E_D_V27, "V.27ter sessions");
      cov_def(CV_E_D_V29, "V.29 sessions");
      cov_def(CV_E_D_V17, "V.17 sessions");
      cov_def(CV_E_D_CONNECT, "Tx connect reached");
      cov_def(CV_E_D_DATA, "data state reached");
      cov_def(CV_E_D_SAMPLES, "modem samples produced");
      cov_def(CV_E_D_SHORT, "short train sessions");
      break;
    case 7:
      cov_def(CV_E_LIFE, "_init_transmitter over real engines");
      cov_def(CV_E_L_MODS, "every modem value");
      cov_def(CV_E_L_REPLACE, "replacing a previous channel");
      cov_def(CV_E_L_DEL, "_delete_data_tx_modem");
      break;
#endif
    case 4:
      cov_def(CV_L_INIT, "_init_transmitter");
      cov_def(CV_L_M18, "modem 0x18");
      cov_def(CV_L_M30, "modem 0x30");
      cov_def(CV_L_M48, "modem 0x48");
      cov_def(CV_L_M60, "modem 0x60");
      cov_def(CV_L_M49, "modem 0x49");
      cov_def(CV_L_M4A, "modem 0x4a");
      cov_def(CV_L_M61, "modem 0x61");
      cov_def(CV_L_M62, "modem 0x62");
      cov_def(CV_L_M79, "modem 0x79");
      cov_def(CV_L_M7A, "modem 0x7a");
      cov_def(CV_L_M91, "modem 0x91");
      cov_def(CV_L_M92, "modem 0x92");
      cov_def(CV_L_FRESH, "fresh data channel");
      cov_def(CV_L_REPLACE, "previous channel replaced");
      cov_def(CV_L_REUSE, "config block reused (no channel)");
      cov_def(CV_L_SKIP, "channel kept (no config block)");
      cov_def(CV_L_CTL_V17, "control record V.17");
      cov_def(CV_L_CTL_V29, "control record V.29");
      cov_def(CV_L_CTL_V27, "control record V.27ter");
      cov_def(CV_L_NOCTL, "no control call");
      cov_def(CV_L_SIL0, "silence 0 -> state 9");
      cov_def(CV_L_SIL1, "silence > 0 -> state 0x12");
      cov_def(CV_L_DBG, "debug lines");
      cov_def(CV_L_ONESINIT, "_tx_scrambled_ones_init");
      cov_def(CV_L_DEL, "_delete_data_tx_modem");
      cov_def(CV_L_DEL_NOFIFO, "delete without fifo");
      cov_def(CV_L_PROBE, "malloc failure probes");
      cov_def(CV_L_SHORT, "short train");
      cov_def(CV_L_FIFO_REUSE, "ones init reuses a fifo");
      break;
  }
}

/* ---------------------------------------------------------------------------------------------- callee recorders */
static int g_dbg_on = 1;
/* coverage of the debug lines (they are the only observable of some branches) */
static void dbg_cov(const char *b)
{
  static const struct { const char *s; int id; } t[] = {
      {"End of HDLC buffer transmission", CV_H_SB_DBG}, {"Elapsed 50MS", CV_H_PRE_DBG}, {"Collect data with", CV_H_PP_DBG3}, {"Elapsed 1 second", CV_H_PP_1S}, {"Elapsed 5 second", CV_H_PP_5S},
      {"Idle state", CV_H_HB_T2_IDLE}, {"CURRENT_STATE_TIMER > 5[sec]", CV_H_HB_DBG}, {"End of off state", CV_H_OFF_DBG}, {"ENABLE_TRANSMIT", CV_D_ONES_ENABLE},
      {"Fifo is full in _tx_scrambled_ones_state", CV_D_ONES_FULL}, {"fifo under run in _tx_scrambled_ones_state", CV_D_ONES_UNDER}, {"Tx connect", CV_D_ONES_CONNECT},
      {"Fifo is full in _tx_data_state", CV_D_DATA_FULL}, {"fifo underrun in _tx_data_state", CV_D_DATA_UNDER}, {"Queue underrun, Stop TX", CV_D_DATA_QEMPTY_STOP},
      {"Queue underrun, Continue NULLS", CV_D_DATA_QEMPTY_NULLS}, {"Queue is full in _tx_data_state", CV_D_DATA_QFULL}, {"Fifo is full in _tx_nulls_state", CV_D_NULLS_FULL},
      {"fifo under run in _tx_nulls_state", CV_D_NULLS_UNDER}, {"FIVE_SECONDS in _tx_nulls_state", CV_D_NULLS_TO}, {"cDATAtx_counter", CV_D_ONES_DBG3},
  };
  unsigned i;
  for (i = 0; i < sizeof t / sizeof t[0]; i++)
    if (strstr(b, t[i].s)) cov_hit(t[i].id);
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

/* the script the recorders follow (set by the scenario before each call) */
typedef struct {
  unsigned proc_ret;    /* FAXVMI_process result */
  int nsamp;            /* samples the transmit recorder writes (0 .. 0x140) */
  int ctl_ret, st_ret;
  unsigned sts[6];      /* FAXVMI_status: s0 / w4 / w8 / wc / w10 / w14 of the record handed back */
  int hdi_k, hdi_ret, hdi_1238, hdi_nset; /* _handle_data_input: words written (< 0: none), result, new w1238 (< 0: keep), set *n */
  int hhi_k, hhi_ret, hhi_1224;           /* _handle_hdlc_input: words written (< 0: none), result, new w1224 (< 0: keep) */
  unsigned gen_ctr;
} Script;
static Script g_rs;
static unsigned g_serial = 100;
static unsigned g_tone_mark[4];

static unsigned rec_serial(const FaxVmi *o) { return o ? (unsigned)o->rxOverflow : 0u; }
#ifndef HARNESS_ENG
FaxVmi *FAXVMI_create(FaxVmi *o, const FaxVmiCfg *cfg)
{
  char b[24];
  emit(0, "VC o=%s cfg mode=%d flag=%d s08=%u s0a=%u s0c=%u id=%u mcfg=%s w14=%d", o ? "supplied" : "NULL", cfg->mode, cfg->flag, (unsigned)cfg->ringMin, (unsigned)cfg->maxItems, (unsigned)cfg->frmWords,
       (unsigned)cfg->id, dptr(cfg->mcfg, b, sizeof b), cfg->w14);
  if (!o) {
    o = (FaxVmi *)sysdep_malloc((unsigned)sizeof(FaxVmi));
    if (!o) return NULL;
  }
  memset(o, 0, sizeof *o);
  o->cfg = *cfg;
  o->rxOverflow = (int)g_serial++;
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
  int i, k = g_rs.nsamp;
  unsigned c = *cnt2;
  emit(0, "VP serial %u id=%u n=%u cnt2=%u buf=%016llx in=%016llx", rec_serial(o), (unsigned)o->cfg.id, (unsigned)*n, c, fnv_shorts((const short *)buf, 320),
       fnv_shorts(samples, c < 400 ? c : 400));
  g_rs.gen_ctr++;
  for (i = 0; i < k; i++) samples[i] = (short)(g_rs.gen_ctr * 77u + (unsigned)i * 13u + rec_serial(o));
  return g_rs.proc_ret;
}
static unsigned g_ctl_words[8];
int FAXVMI_control(FaxVmi *o, const FaxVmiCtl *ctl)
{
  const int *m = (const int *)ctl->mctl;
  int nw = (o->cfg.id == FAXVMI_ID_V27TX || o->cfg.id == FAXVMI_ID_V17TX) ? 5 : 4;
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
  emit(0, "VS serial %u in s0=%d pad2=%d w4=%d w8=%d wc=%d w10=%d w14=%d msts=%u", rec_serial(o), (int)st->ringFree, (int)st->pad2, st->ovf, st->txEmpty, st->rxOverflow, st->rxBreak, st->hskipDone,
       st->msts ? (unsigned)(uintptr_t)st->msts & 0xffffffffu : 0u);
  st->ringFree = (short)g_rs.sts[0];
  st->ovf = (int)g_rs.sts[1];
  st->txEmpty = (int)g_rs.sts[2];
  st->rxOverflow = (int)g_rs.sts[3];
  st->rxBreak = (int)g_rs.sts[4];
  st->hskipDone = (int)g_rs.sts[5];
  return g_rs.st_ret;
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
/* R29's input side (recorders: the blob's own functions are weakened) */
static unsigned g_in_ctr;
static void rec_fill_words(unsigned short *out, int k)
{
  int i;
  for (i = 0; i < k; i++) {
    g_in_ctr++;
    out[i] = (unsigned short)(g_in_ctr * 97u + 5u);
  }
}
int _handle_data_input(FaxT30 *o, const unsigned char *in, unsigned short *out, int *n)
{
  int cnt = *n, k = g_rs.hdi_k;
  emit(0, "HDI n=%d in=%016llx out_is_buf=%d w1238=%d", cnt, fnv_bytes(in, cnt > 0 && cnt < 64 ? (size_t)cnt : 0u), out == o->buf, o->lastByte);
  if (k >= 0) {
    if (k > 0x7ff) die3("hdi_k");
    rec_fill_words(out, k);
  }
  if (g_rs.hdi_nset) *n = k >= 0 ? k : 0;
  if (g_rs.hdi_1238 >= 0) o->lastByte = g_rs.hdi_1238;
  return g_rs.hdi_ret;
}
int _handle_hdlc_input(FaxT30 *o, const unsigned char *in, unsigned short *out, int *n)
{
  int cnt = *n, k = g_rs.hhi_k;
  emit(0, "HHI n=%d in=%016llx out_is_buf=%d", cnt, fnv_bytes(in, cnt > 0 && cnt < 64 ? (size_t)cnt : 0u), out == o->buf);
  if (k >= 0) {
    if (k > 0x7ff) die3("hhi_k");
    rec_fill_words(out, k);
    *n = k;
  }
  if (g_rs.hhi_1224 >= 0) o->lastFrame = g_rs.hhi_1224;
  return g_rs.hhi_ret;
}
static unsigned g_hho_count;
int _handle_hdlc_input_open(FaxT30 *o)
{
  emit(0, "HHO");
  g_hho_count++;
  o->hdlcWr = 1;
  return 0;
}
/* the recorder initialisers of the data modem classes: what the init_vmi_data_* functions do to the 0x18 byte block (a config block of the modem is allocated, its pointer is stored) */
static int rec_init(int id, FaxVmiM *m, unsigned short rate, int unused, void *user)
{
  unsigned char *cfgblk;
  char b[24];
  emit(0, "INIT id=%d rate=%u unused=%d user=%s", id, (unsigned)rate, unused, dptr(user, b, sizeof b));
  cfgblk = (unsigned char *)sysdep_malloc(0x1c);
  if (cfgblk) memset(cfgblk, 0x11 * (id & 15), 0x1c);
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
static int rec_init_v27(FaxVmiM *m, unsigned short r, int u, void *x) { return rec_init(FAXVMI_ID_V27TX, m, r, u, x); }
static int rec_init_v29(FaxVmiM *m, unsigned short r, int u, void *x) { return rec_init(FAXVMI_ID_V29TX, m, r, u, x); }
static int rec_init_v17(FaxVmiM *m, unsigned short r, int u, void *x) { return rec_init(FAXVMI_ID_V17TX, m, r, u, x); }
static int rec_init_other(FaxVmiM *m, unsigned short r, int u, void *x) { return rec_init(3, m, r, u, x); }
static void init_tab_default(void)
{
  INITTAB[0] = rec_init_v27;
  INITTAB[1] = rec_init_v29;
  INITTAB[2] = rec_init_v17;
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
static void pr_ptr(const void *p, char *b, size_t cap) { dptr(p, b, cap); }

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
static void dump_fifo(const char *tag, const FaxFifo *f)
{
  char b[24];
  if (!f) {
    emit(0, "%s F NULL", tag);
    return;
  }
  emit(0, "%s F w0=%u size=%u fill=%u cnt=%u rd=%u wr=%u buf=%s data=%016llx", tag, (unsigned)f->w0, (unsigned)f->size, (unsigned)f->fill, (unsigned)f->count, (unsigned)f->rd, (unsigned)f->wr,
       dptr(f->buf, b, sizeof b), fnv_shorts((const short *)f->buf, f->size));
}
static void dump_t30(const char *tag, const FaxT30 *o)
{
  char b1[24], b2[24], b3[24], b4[24], b5[24], b6[24], b7[24], b8[24];
  emit(0, "%s T hdlc_tx=%s hdlc_rx=%s data_vmi=%s txcfg=%s rxcfg=%s data_cfg=%s tone=%s fifo=%s", tag, vmi_name(o->hdlc_tx, b1, sizeof b1), vmi_name(o->hdlc_rx, b2, sizeof b2),
       vmi_name(o->data_vmi, b3, sizeof b3), dptr(o->hdlc_tx_cfg, b4, sizeof b4), dptr(o->hdlc_rx_cfg, b5, sizeof b5), dptr(o->data_cfg, b6, sizeof b6),
       (const void *)o->tone == (const void *)g_tone_mark ? "mark" : (o->tone ? "other" : "NULL"), dptr(o->fifo, b7, sizeof b7));
  (void)b8;
  emit(0, "%s T w1218=%d state=%d prev=%d w1224=%d tmr=%d w122c=%d w1230=%d w1234=%d w1238=%d w123c=%d w1240=%d w1244=%d w1248=%d w124c=%d w1250=%d w1254=%d", tag, o->ans_org, o->state,
       o->state_prev, o->lastFrame, o->tmr, o->status, o->w1230, o->w1234, o->lastByte, o->rx_event, o->rx_kind, o->chan, o->modemClass, o->dlePending, o->hdlcWr, o->modem_sel);
  emit(0, "%s T w125c=%d w1260=%d w1264=%d w1268=%d w126c=%d w1270=%d sec=%d ms=%d w127c=%d w1280=%d w1284=%d w128c=%d w1290=%d w1294=%d w1298=%d", tag, o->cng_tone, o->cng_ctr,
       o->cng_on, o->ans_len, o->dataRate, o->enableDelay, o->sec, o->ms, o->rx_act, o->frameDone, o->frameCount, o->txEnable, o->frameWords, o->txConnected, o->fifoReady);
  emit(0, "%s T w129c=%d w12a0=%08x w12a4=%d w12a8=%08x w12ac=%d w12b0=%d w12b4=%d w12b8=%d w12bc=%d w12c0=%d w12c4=%d w12c8=%d w12cc=%d w12d0=%d w12d4=%d w12d8=%d p12dc=%s b12e0=%016llx w12f0=%d",
       tag, o->alignDone, o->alignHist, o->alignShift, o->alignMask, o->w12ac, o->dataEnd, o->s7, o->silenceTime, o->silence_rms, o->agc_rcp, o->agc_shift, o->emu_delay, o->emu_idx, o->sf_fill, o->cd_flag, o->gain_att,
       dptr(o->hpCoef, b8, sizeof b8), fnv_bytes(o->hpState, sizeof o->hpState), o->hp_on);
  emit(0, "%s T buf=%016llx cnt=%d guard=%016llx", tag, fnv_shorts((const short *)o->buf, 0x900), COUNTER, o == &g_t30 ? fnv_shorts((const short *)g_wrap.pre, 16) : 0ull);
  dump_fifo(tag, o->fifo);
  (void)pr_ptr;
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
  /* small non-negative garbage in the word the data states use as a size (the garbage must not make a state write outside the object) */
  o->frameWords = (int)rmod(40);
}
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
/* an object with recorder channels, as fax_class1_create leaves it (the HDLC channels) */
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
    sysdep_free(o->data_cfg->u10.cfg);
    sysdep_free(o->data_cfg);
    o->data_cfg = NULL;
  }
  if (o->fifo) {
    FIFO_delete(o->fifo);
    o->fifo = NULL;
  }
}
static FaxFifo *new_fifo(unsigned size, unsigned fill_count)
{
  FaxFifoCfg c;
  FaxFifo *f;
  unsigned short buf[0x800];
  unsigned i;
  c.w0 = 0;
  c.size = (unsigned short)size;
  c.fill = 0;
  f = FIFO_create(NULL, &c);
  for (i = 0; i < fill_count && i < 0x800; i++) buf[i] = (unsigned short)rnd();
  if (fill_count) FIFO_write(f, buf, (unsigned short)(fill_count > size ? size : fill_count));
  return f;
}
/* a size in shorts whose byte size (2 * size) is not the size of a typed object (the heap log masks those) */
static unsigned safe_fifo_size(unsigned size)
{
  while (safe_bytes(2u * size) != 2u * size) size++;
  return size;
}

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

/* script helpers (each draw its own statement) */
static void script_random(void)
{
  unsigned a, b, c, d;
  memset(&g_rs, 0, sizeof g_rs);
  a = rnd();
  g_rs.proc_ret = a;
  b = rmod(5);
  g_rs.nsamp = b == 0 ? 0 : (b == 1 ? 0xa0 : (int)rmod(0x141));
  c = rnd();
  g_rs.ctl_ret = (int)(c & 0xffff);
  d = rnd();
  g_rs.st_ret = (int)(d & 0xffff);
  g_rs.sts[0] = rnd() & 0xffff;
  g_rs.sts[1] = rnd();
  g_rs.sts[2] = rpct(40) ? 1u : (rpct(30) ? 0u : rnd());
  g_rs.sts[3] = rnd();
  g_rs.sts[4] = rnd();
  g_rs.sts[5] = rnd();
  g_rs.hdi_k = rpct(25) ? -1 : (rpct(30) ? 0 : (int)rmod(60));
  g_rs.hdi_ret = (int)rmod(3);
  g_rs.hdi_1238 = rpct(50) ? -1 : (int)rmod(300);
  g_rs.hdi_nset = rpct(80);
  g_rs.hhi_k = rpct(25) ? -1 : (rpct(30) ? 0 : (int)rmod(300));
  g_rs.hhi_ret = rpct(50) ? 1 : (int)rmod(3);
  g_rs.hhi_1224 = rpct(60) ? -1 : (int)rmod(3);
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

/* the state table of fax_class1_progress as far as R28 owns it (index = o->state); R29 / R30 slots are NULL: a sequence ends there */
static StateFn state_fn(int st)
{
  switch (st) {
    case 0: return _t30_silence_before_tx_state;
    case 1: return _t30_preabmle_state;
    case 2: return _send_hdlc_buffer_state;
    case 3: return _send_hdlc_between_buffer_state;
    case 8: return T_IDLE;
    case 9: return _tx_scrambled_ones_state;
    case 10: return _tx_data_state;
    case 11: return _tx_nulls_state;
    case 14: return T_ANSWER;
    case 15: return T_SILENCE;
    case 17: return cHDLCtx_off;
    case 18: return _tx_silence_before_scrm_ones;
    default: return NULL;
  }
}
static unsigned g_seq_seen[32];
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

#ifndef HARNESS_ENG
#include "r28_misc.h"
#include "r28_hdlc.h"
#include "r28_data.h"
#else
#include "r28_eng.h"
#endif

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R28_SEED");
  g_verbose = getenv("R28_VERBOSE") != NULL;
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
    case 0: scen_misc(); break;
    case 1: scen_shared(); break;
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
