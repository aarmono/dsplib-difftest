/*
 * R10 differential harness: the dialer (DialerCreate / DialerProgress / DialerAbort / GetDialerConfig / IsDialStringInvalid / Dialer_IsDialStringInvalid, the pulse-dial
 * helpers, the calling-tone generator) - real dsplibs.o vs src/callprog/dialer.c.  Library test/callprog/r10_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation,
 * callee recorders: modem_get_param / modem_set_param / dsplibs_debug_printf (+ a hash of the objects at the very moment of the print) / sysdep_strlen / sysdep_strcpy and the R11
 * functions TONE_read / FP_Pow).  The typed objects of src/callprog/dialer.h are used on both sides; the 32-bit BLOB run is the x86-64 golden.  AnalyseDialString and
 * GetNextDigitAndReturnNextState are blob LOCALs: they are tested through DialerCreate / IsDialStringInvalid and through DialerProgress state 0.
 * Integer only, no libm.
 *
 * Scenarios (each its own process; argv[1]):
 *   0 pulse helpers on random call blocks: PulseDialDigit / IsPulseDialerReady / LastPulseDigitDialed / SetPulseBreakTime / SetPulseMakeTime in random sequences, the
 *     dp NULL case, dp.modem != the argument, every counter / break / make / hook class around the thresholds, debug levels;
 *   1 pulse sessions: a digit dialled to the end (counter +5 per ready call), several digits in a row, hook on / off transitions;
 *   2 GetDialerConfig: random S-register values for all 16 parameters (gain table classes: level -3 .. 20, difference -2 .. 9), debug levels, the level poked in the middle;
 *   3 dial string analysis through DialerCreate / IsDialStringInvalid / Dialer_IsDialStringInvalid: every character after every prefix (0 .. 255) under all configs, random
 *     strings, 99 .. 102 character strings, NULL / empty, ';' positions; checked against an independent model of the analysis (exit 3);
 *   4 DialerCreate state / DialerAbort over every state x pulse flags x levels / ResetCallingTone + GenerateCallingTone chunk sequences over random tone objects;
 *   5 DialerProgress state matrix: every state 0 .. 10, > 10, every configuration class, every digit / pattern, buffer windows of 0 .. 400 samples;
 *   6 GetNextDigitAndReturnNextState through DialerProgress state 0: every character 0 .. 255 (cur == 0 and > 0, tone / pulse, A..D, ^, mixing allowed or not);
 *   7 full dial sessions (DialerCreate + DialerProgress until the end event, random chunk sizes, tone / pulse / mixed strings, flash, commas, waits, calling tone, pulse
 *     patterns 0 .. 4) at debug levels 0 and 2, with DialerAbort at random points.
 * R10_VERBOSE=1 prints every line (debugging), R10_SEED selects another input stream.
 */
#include "r10_common.h"

static Dialer g_dl;
static union {
  Dialer d;
  char raw[CALLPROG_DIALER_OFF + sizeof(Dialer) + 16];
  unsigned long long align;
} g_cp;
static Dialer *g_d; /* the object the debug hook describes */
#define DPFX 0xd4 /* the bytes of a Dialer up to its modem pointer: the same count of random bytes on every host (the pointer moves on x86-64) */

static u64 snap_cur(void) { return (g_d ? hash_dialer(g_d) : 0ULL) ^ (hash_pulse() * 31u); }

static void dirty(void *p, size_t n)
{
  unsigned char *b = (unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) b[i] = (unsigned char)rnd();
}

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_PD_NULL, CV_PD_ZERO, CV_PD_DIGIT, CV_IR_NODP, CV_IR_COUNT0, CV_IR_HOOKON, CV_IR_HOOKOFF, CV_IR_NEXTPULSE, CV_IR_SAMEPULSE, CV_IR_HOOK_LT, CV_IR_MODEM2, CV_SB, CV_SM, CV_LAST, CV_DBG_PD, CV_DBG_IR,
  CV_DBG_HOOKON, CV_DBG_HOOKOFF, CV_DBG_LAST, CV_DBG_SB, CV_DBG_SM, CV_SESS_PULSE_END,
  CV_G_LEVEL_IN, CV_G_LEVEL_OUT, CV_G_DIFF_IN, CV_G_DIFF_OUT, CV_G_FLAG0, CV_G_FLAG1, CV_G_DBG, CV_G_POKE, CV_G_NODBG,
  CV_A_NULL, CV_A_EMPTY, CV_A_NOPREFIX, CV_A_TOOLONG, CV_A_TLOW, CV_A_PLOW, CV_A_VALID, CV_A_TOLERABLE, CV_A_INVALID, CV_A_SEMI_LAST, CV_A_SEMI_MID, CV_A_ABCD_OK, CV_A_ABCD_BAD, CV_A_CALL_OK, CV_A_CALL_BAD,
  CV_A_T_IN_P_OK, CV_A_T_IN_P_BAD, CV_A_P_IN_T_OK, CV_A_P_IN_T_BAD, CV_A_BADCHAR_STRICT, CV_A_BADCHAR_TOL, CV_A_HIGHCHAR, CV_A_LAST_SYMBOL, CV_A_NOLAST, CV_A_IDI, CV_A_DIDI, CV_A_RET7, CV_A_COPY, CV_A_LEN100, CV_A_ILLEGAL,
  CV_C_NULLSTR, CV_ABORT_ST_OK, CV_ABORT_ST_BAD, CV_ABORT_PULSE, CV_ABORT_NOPULSE_ACTIVE0, CV_ABORT_DONE, CV_CT_RESET, CV_CT_TONE, CV_CT_SILENCE, CV_CT_SWITCH_ON, CV_CT_SWITCH_OFF, CV_CT_PARTIAL, CV_CT_CHUNKEDGE,
  CV_CT_ZEROCNT, CV_CT_NEGN, CV_CT_BIGPHASE, CV_CT_ONOTHER,
  CV_P_ST0, CV_P_ST1_TONE, CV_P_ST1_PULSE, CV_P_ST2, CV_P_ST3, CV_P_ST4, CV_P_ST5, CV_P_ST6, CV_P_ST7, CV_P_ST8, CV_P_ST9, CV_P_ST10, CV_P_DEFAULT,
  CV_P_DIGIT_TONE_DONE, CV_P_DIGIT_PULSE_DONE, CV_P_LOOKAHEAD, CV_P_NOLOOK, CV_P_SIL_DONE, CV_P_COMMA_DONE, CV_P_FLASH_BEGIN, CV_P_FLASH_END, CV_P_FLASH_DONE, CV_P_WAITING, CV_P_CALLING, CV_P_ENDDIGIT,
  CV_P_PAT1, CV_P_PAT2, CV_P_PAT3, CV_P_PAT0, CV_P_PAT3_11, CV_P_PAT3_LE9, CV_P_PAT3_GT9, CV_P_COMMAS, CV_P_COMMAS_LIM, CV_P_SAMPLES, CV_P_LASTPULSE_TONE, CV_P_LASTPULSE_END,
  CV_P_RC0, CV_P_RC1, CV_P_RC2, CV_P_RC3, CV_P_RC4, CV_P_RC5, CV_P_RC6, CV_P_RC7, CV_P_POSGT, CV_P_PARTIAL_SIL, CV_P_PARTIAL_TONE, CV_P_TONE_LEN0, CV_P_FLASH_NOTREADY1, CV_P_FLASH_NOTREADY2,
  CV_P_PULSE_NOTREADY2, CV_P_BETWEEN_TONE, CV_P_BETWEEN_PULSE, CV_P_FREQ_HI,
  CV_N_DIG, CV_N_BANG, CV_N_HASH_T, CV_N_HASH_P, CV_N_STAR_T, CV_N_STAR_P, CV_N_DOLLAR, CV_N_COMMA, CV_N_COMMA_RUN, CV_N_SEMI_LAST, CV_N_SEMI_MID, CV_N_AT, CV_N_CARET1, CV_N_CARET0, CV_N_W, CV_N_ABCD_T,
  CV_N_ABCD_P, CV_N_ABCD_BAD, CV_N_P_FIRST, CV_N_P_SWITCH, CV_N_P_NOPERM, CV_N_P_ALREADY, CV_N_T_FIRST, CV_N_T_SWITCH, CV_N_T_NOPERM, CV_N_T_ALREADY, CV_N_OTHER, CV_N_END, CV_N_HIGH
};
#define CD(id, name) cov_def(id, name)
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      CD(CV_PD_NULL, "PulseDialDigit dp NULL"); CD(CV_PD_ZERO, "PulseDialDigit digit 0 (count 10)"); CD(CV_PD_DIGIT, "PulseDialDigit digit != 0");
      CD(CV_IR_NODP, "IsPulseDialerReady dp NULL"); CD(CV_IR_COUNT0, "IsPulseDialerReady count 0"); CD(CV_IR_HOOKON, "IsPulseDialerReady counter < break, hook on starts");
      CD(CV_IR_HOOKOFF, "IsPulseDialerReady counter >= break, hook off"); CD(CV_IR_NEXTPULSE, "IsPulseDialerReady next pulse (counter >= break + make)");
      CD(CV_IR_SAMEPULSE, "IsPulseDialerReady inside the pulse"); CD(CV_IR_HOOK_LT, "IsPulseDialerReady counter < break, hook already on"); CD(CV_IR_MODEM2, "dp.modem != argument modem");
      CD(CV_SB, "SetPulseBreakTime"); CD(CV_SM, "SetPulseMakeTime"); CD(CV_LAST, "LastPulseDigitDialed"); CD(CV_DBG_PD, "debug PulseDialDigit"); CD(CV_DBG_IR, "debug IsPulseDialerReady");
      CD(CV_DBG_HOOKON, "debug hook on"); CD(CV_DBG_HOOKOFF, "debug hook off"); CD(CV_DBG_LAST, "debug LastPulseDigitDialed"); CD(CV_DBG_SB, "debug SetPulseBreakTime"); CD(CV_DBG_SM, "debug SetPulseMakeTime");
      break;
    case 1:
      CD(CV_SESS_PULSE_END, "pulse digit dialled to the end"); CD(CV_IR_HOOKON, "hook on seen"); CD(CV_IR_HOOKOFF, "hook off seen"); CD(CV_IR_NEXTPULSE, "next pulse"); CD(CV_IR_COUNT0, "count 0");
      break;
    case 2:
      CD(CV_G_LEVEL_IN, "gain: level 6 .. 12"); CD(CV_G_LEVEL_OUT, "gain: level outside"); CD(CV_G_DIFF_IN, "gain: diff 1 .. 5"); CD(CV_G_DIFF_OUT, "gain: diff outside"); CD(CV_G_FLAG0, "pulse flag 0");
      CD(CV_G_FLAG1, "pulse flag != 0"); CD(CV_G_DBG, "debug level > 1"); CD(CV_G_POKE, "level poked inside a callback"); CD(CV_G_NODBG, "debug level <= 1");
      break;
    case 3:
      CD(CV_A_NULL, "analysis: NULL string"); CD(CV_A_EMPTY, "analysis: empty string"); CD(CV_A_NOPREFIX, "analysis: first character not T/P"); CD(CV_A_TOOLONG, "analysis: > 100 characters");
      CD(CV_A_TLOW, "analysis: prefix t"); CD(CV_A_PLOW, "analysis: prefix p"); CD(CV_A_VALID, "verdict VALID"); CD(CV_A_TOLERABLE, "verdict TOLERABLE"); CD(CV_A_INVALID, "verdict INVALID");
      CD(CV_A_SEMI_LAST, "';' last"); CD(CV_A_SEMI_MID, "';' inside"); CD(CV_A_ABCD_OK, "A..D allowed"); CD(CV_A_ABCD_BAD, "A..D not allowed"); CD(CV_A_CALL_OK, "'^' allowed"); CD(CV_A_CALL_BAD, "'^' with flag 2");
      CD(CV_A_T_IN_P_OK, "T inside a pulse string, mixing allowed"); CD(CV_A_T_IN_P_BAD, "T inside a pulse string, not allowed"); CD(CV_A_P_IN_T_OK, "P inside a tone string, mixing allowed");
      CD(CV_A_P_IN_T_BAD, "P inside a tone string, not allowed"); CD(CV_A_BADCHAR_STRICT, "illegal char, strict"); CD(CV_A_BADCHAR_TOL, "illegal char, tolerant"); CD(CV_A_HIGHCHAR, "char >= 0x80");
      CD(CV_A_LAST_SYMBOL, "last dialable symbol recorded"); CD(CV_A_NOLAST, "no dialable symbol"); CD(CV_A_IDI, "IsDialStringInvalid"); CD(CV_A_DIDI, "Dialer_IsDialStringInvalid"); CD(CV_A_RET7, "DialerCreate returns 7");
      CD(CV_A_COPY, "DialerCreate copies the string"); CD(CV_A_LEN100, "100 character string"); CD(CV_A_ILLEGAL, "FATAL verdict (> 100)");
      break;
    case 4:
      CD(CV_C_NULLSTR, "DialerCreate NULL string"); CD(CV_ABORT_ST_OK, "abort state <= 10"); CD(CV_ABORT_ST_BAD, "abort state > 10"); CD(CV_ABORT_PULSE, "abort: LastPulseDigitDialed called");
      CD(CV_ABORT_NOPULSE_ACTIVE0, "abort: pulse not active"); CD(CV_ABORT_DONE, "abort: pulse already done"); CD(CV_CT_RESET, "ResetCallingTone"); CD(CV_CT_TONE, "calling tone samples"); CD(CV_CT_SILENCE, "calling tone silence");
      CD(CV_CT_SWITCH_ON, "calling tone off -> on"); CD(CV_CT_SWITCH_OFF, "calling tone on -> off"); CD(CV_CT_PARTIAL, "segment longer than the buffer"); CD(CV_CT_CHUNKEDGE, "segment ends exactly at the buffer end");
      CD(CV_CT_ZEROCNT, "cnt 0 at entry"); CD(CV_CT_NEGN, "n <= 0"); CD(CV_CT_BIGPHASE, "phase > 0x3fff at entry"); CD(CV_CT_ONOTHER, "on not 0/1");
      break;
    case 5:
      CD(CV_P_ST0, "state 0"); CD(CV_P_ST1_TONE, "state 1 tone digit"); CD(CV_P_ST1_PULSE, "state 1 pulse digit"); CD(CV_P_ST2, "state 2"); CD(CV_P_ST3, "state 3"); CD(CV_P_ST4, "state 4");
      CD(CV_P_ST5, "state 5"); CD(CV_P_ST6, "state 6"); CD(CV_P_ST7, "state 7"); CD(CV_P_ST8, "state 8"); CD(CV_P_ST9, "state 9"); CD(CV_P_ST10, "state 10"); CD(CV_P_DEFAULT, "state > 10");
      CD(CV_P_DIGIT_TONE_DONE, "tone digit complete"); CD(CV_P_DIGIT_PULSE_DONE, "pulse digit complete"); CD(CV_P_LOOKAHEAD, "digit complete, cur == lastIdx"); CD(CV_P_NOLOOK, "digit complete, cur != lastIdx");
      CD(CV_P_SIL_DONE, "silence done"); CD(CV_P_COMMA_DONE, "comma pause done"); CD(CV_P_FLASH_BEGIN, "flash begin"); CD(CV_P_FLASH_END, "flash end"); CD(CV_P_FLASH_DONE, "flash done");
      CD(CV_P_WAITING, "waiting for the pulse dialer"); CD(CV_P_CALLING, "PulseDialDigit called"); CD(CV_P_ENDDIGIT, "end of pulse digit");
      CD(CV_P_PAT1, "digit pattern 1"); CD(CV_P_PAT2, "digit pattern 2"); CD(CV_P_PAT3, "digit pattern 3"); CD(CV_P_PAT0, "digit pattern other"); CD(CV_P_PAT3_11, "pattern 3 index 11"); CD(CV_P_PAT3_LE9, "pattern 3 index <= 9");
      CD(CV_P_PAT3_GT9, "pattern 3 index 10 / 12"); CD(CV_P_COMMAS, "comma samples"); CD(CV_P_COMMAS_LIM, "comma samples limited"); CD(CV_P_SAMPLES, "tone digit samples"); CD(CV_P_LASTPULSE_TONE, "tone digit after a pulse digit");
      CD(CV_P_LASTPULSE_END, "end state after a pulse digit"); CD(CV_P_RC0, "rc 0"); CD(CV_P_RC1, "rc 1"); CD(CV_P_RC2, "rc 2"); CD(CV_P_RC3, "rc 3"); CD(CV_P_RC4, "rc 4"); CD(CV_P_RC5, "rc 5"); CD(CV_P_RC6, "rc 6");
      CD(CV_P_RC7, "rc 7"); CD(CV_P_POSGT, "pos > last at entry"); CD(CV_P_PARTIAL_SIL, "partial silence / pause"); CD(CV_P_PARTIAL_TONE, "partial tone digit"); CD(CV_P_TONE_LEN0, "tone digit length 0");
      CD(CV_P_FLASH_NOTREADY1, "flash: not ready before the hook"); CD(CV_P_FLASH_NOTREADY2, "flash: not ready after the hook"); CD(CV_P_PULSE_NOTREADY2, "pulse digit: not ready after the digit");
      CD(CV_P_BETWEEN_TONE, "between digits, tone"); CD(CV_P_BETWEEN_PULSE, "between digits, pulse"); CD(CV_P_FREQ_HI, "tone digit A..D (column 3)");
      break;
    case 6:
      CD(CV_N_DIG, "digit 0-9"); CD(CV_N_BANG, "'!'"); CD(CV_N_HASH_T, "'#' tone"); CD(CV_N_HASH_P, "'#' pulse"); CD(CV_N_STAR_T, "'*' tone"); CD(CV_N_STAR_P, "'*' pulse"); CD(CV_N_DOLLAR, "'$'");
      CD(CV_N_COMMA, "',' single"); CD(CV_N_COMMA_RUN, "',' run"); CD(CV_N_SEMI_LAST, "';' last"); CD(CV_N_SEMI_MID, "';' inside"); CD(CV_N_AT, "'@'"); CD(CV_N_CARET1, "'^' flag 1 / 3");
      CD(CV_N_CARET0, "'^' other flag"); CD(CV_N_W, "'W' / 'w'"); CD(CV_N_ABCD_T, "A..D tone"); CD(CV_N_ABCD_P, "A..D pulse"); CD(CV_N_ABCD_BAD, "A..D not allowed"); CD(CV_N_P_FIRST, "'P' first");
      CD(CV_N_P_SWITCH, "'P' switch to pulse"); CD(CV_N_P_NOPERM, "'P' not permitted"); CD(CV_N_P_ALREADY, "'P' already pulse"); CD(CV_N_T_FIRST, "'T' first"); CD(CV_N_T_SWITCH, "'T' switch to tone");
      CD(CV_N_T_NOPERM, "'T' not permitted"); CD(CV_N_T_ALREADY, "'T' already tone"); CD(CV_N_OTHER, "other characters"); CD(CV_N_END, "end of string"); CD(CV_N_HIGH, "char >= 0x80 / out of the table");
      break;
    default:
      break;
  }
}

/* debug text -> coverage (the debug level is 2 or more for the scenarios that count these) */
static int g_cov_dbg_on;
static void dbg_cov(const char *m)
{
  const Dialer *d = g_d;
  if (!g_cov_dbg_on) return;
  if (!strncmp(m, "call: PulseDialDigit", 20)) cov_hit(CV_DBG_PD);
  else if (!strncmp(m, "call: IsPulseDialerReady", 24)) cov_hit(CV_DBG_IR);
  else if (strstr(m, "hook on...")) cov_hit(CV_DBG_HOOKON);
  else if (strstr(m, "hook off...")) cov_hit(CV_DBG_HOOKOFF);
  else if (!strncmp(m, "call: LastPulseDigitDialed", 26)) cov_hit(CV_DBG_LAST);
  else if (!strncmp(m, "call: SetPulseBreakTime", 23)) cov_hit(CV_DBG_SB);
  else if (!strncmp(m, "call: SetPulseMakeTime", 22)) cov_hit(CV_DBG_SM);
  else if (!strncmp(m, "Digit is ", 9) && d) {
    int c = (unsigned char)m[9];
    int t = d->cfg.toneOrPulse == 1;
    if (c >= '0' && c <= '9') cov_hit(CV_N_DIG);
    else if (c == '!') cov_hit(CV_N_BANG);
    else if (c == '#') cov_hit(t ? CV_N_HASH_T : CV_N_HASH_P);
    else if (c == '*') cov_hit(t ? CV_N_STAR_T : CV_N_STAR_P);
    else if (c == '$') cov_hit(CV_N_DOLLAR);
    else if (c == ',') cov_hit(d->str[d->cur + 1] == ',' ? CV_N_COMMA_RUN : CV_N_COMMA);
    else if (c == ';') cov_hit(d->str[d->cur + 1] == 0 ? CV_N_SEMI_LAST : CV_N_SEMI_MID);
    else if (c == '@') cov_hit(CV_N_AT);
    else if (c == '^') cov_hit((d->cfg.callingTone == 1 || d->cfg.callingTone == 3) ? CV_N_CARET1 : CV_N_CARET0);
    else if (c == 'W' || c == 'w') cov_hit(CV_N_W);
    else if ((c >= 'A' && c <= 'D') || (c >= 'a' && c <= 'd')) cov_hit(d->cfg.abcd ? CV_N_ABCD_BAD : (t ? CV_N_ABCD_T : CV_N_ABCD_P));
    else if (c == 'P' || c == 'p') {
      if (d->cur == 0) cov_hit(CV_N_P_FIRST);
      else if (d->cfg.toneOrPulse != 1) cov_hit(CV_N_P_ALREADY);
      else cov_hit(d->cfg.pulseAndTone ? CV_N_P_NOPERM : CV_N_P_SWITCH);
    } else if (c == 'T' || c == 't') {
      if (d->cur == 0) cov_hit(CV_N_T_FIRST);
      else if (d->cfg.toneOrPulse != 0) cov_hit(CV_N_T_ALREADY);
      else cov_hit(d->cfg.pulseAndTone ? CV_N_T_NOPERM : CV_N_T_SWITCH);
    } else if (c >= 0x80 || c < 0x20 || c > 0x77) cov_hit(CV_N_HIGH);
    else cov_hit(CV_N_OTHER);
  } else if (!strcmp(m, "DIALER_INITIAL_STATE")) cov_hit(CV_P_ST0);
  else if (!strcmp(m, "DIALER_WAIT_FOR_DIALTONE_STATE")) cov_hit(CV_P_ST5);
  else if (!strcmp(m, "DIALER_WAIT_FOR_SILENCE_STATE")) cov_hit(CV_P_ST6);
  else if (!strcmp(m, "DIALER_WAIT_FOR_BONGTONE_STATE")) cov_hit(CV_P_ST7);
  else if (!strcmp(m, "DIALER_CALLING_TONE_STATE")) cov_hit(CV_P_ST8);
  else if (!strcmp(m, "DIALER_END_PARTIALLY_STATE")) cov_hit(CV_P_ST9);
  else if (!strcmp(m, "DIALER_END_STATE")) cov_hit(CV_P_ST10);
  else if (!strcmp(m, "Dialer switch default case")) cov_hit(CV_P_DEFAULT);
  else if (!strcmp(m, "Done Generating digit") && d) {
    cov_hit(d->cfg.toneOrPulse == 1 ? CV_P_DIGIT_TONE_DONE : CV_P_DIGIT_PULSE_DONE);
    if (d->cfg.toneOrPulse == 1) cov_hit(CV_P_ST1_TONE);
    cov_hit(d->cur == d->lastIdx ? CV_P_LOOKAHEAD : CV_P_NOLOOK);
  } else if (!strcmp(m, "Done Generating silence between digits")) {
    cov_hit(CV_P_SIL_DONE);
    cov_hit(CV_P_ST2);
    if (d) cov_hit(d->cfg.toneOrPulse == 1 ? CV_P_BETWEEN_TONE : CV_P_BETWEEN_PULSE);
  }
  else if (!strcmp(m, "Done Generating Dial-Pause")) { cov_hit(CV_P_COMMA_DONE); cov_hit(CV_P_ST4); }
  else if (strstr(m, "Begin Dialing FLASH")) { cov_hit(CV_P_FLASH_BEGIN); cov_hit(CV_P_ST3); }
  else if (strstr(m, "End dialing Flash")) cov_hit(CV_P_FLASH_END);
  else if (!strcmp(m, "Done Generating flash")) cov_hit(CV_P_FLASH_DONE);
  else if (strstr(m, "Waiting to PULSE_IS_DIALER_READY_PROC")) cov_hit(CV_P_WAITING);
  else if (strstr(m, "Calling PULSE_DIAL_DIGIT_PROC")) {
    cov_hit(CV_P_CALLING);
    cov_hit(CV_P_ST1_PULSE);
    if (d) {
      switch (d->cfg.digitPattern) {
        case 1: cov_hit(CV_P_PAT1); break;
        case 2: cov_hit(CV_P_PAT2); break;
        case 3: break;
        default: cov_hit(CV_P_PAT0); break;
      }
    }
  } else if (strstr(m, "END OF DIAL DIGIT DETECTED")) cov_hit(CV_P_ENDDIGIT);
  else if (!strncmp(m, "Original digitToPulseDial ", 26)) {
    int v = atoi(m + 26);
    cov_hit(CV_P_PAT3);
    cov_hit(v == 11 ? CV_P_PAT3_11 : (v <= 9 ? CV_P_PAT3_LE9 : CV_P_PAT3_GT9));
  } else if (!strncmp(m, "Consequitive-Commas", 19)) {
    cov_hit(CV_P_COMMAS);
    if (d && (int)((unsigned)d->cfg.dialPause * (unsigned)d->commaCount) > d->cfg.commaLimit) cov_hit(CV_P_COMMAS_LIM);
  } else if (!strncmp(m, "Samples left = ", 15)) {
    cov_hit(CV_P_SAMPLES);
    if (d && d->col == 3) cov_hit(CV_P_FREQ_HI);
  }
  else if (strstr(m, " **** Dialer.C: LastPulseDigitDialed")) {
    if (d && d->state >= 9u) cov_hit(CV_P_LASTPULSE_END);
    else cov_hit(CV_P_LASTPULSE_TONE);
  } else if (!strncmp(m, "Dialer.c: GetNextDigit... ", 26)) {
    /* the P / T messages are counted at the "Digit is" print */
  }
}

/* ---------------------------------------------------------------------------------------------- configuration + pulse state generators */
static int ipick(const int *a, int n) { return a[rmod((unsigned)n)]; }
static void rand_prm(int style)
{
  static const int pat[] = {0, 1, 2, 3, 4, -1, 5, 100};
  static const int flags[] = {0, 1, 2, 3, 0, 1};
  memset(g_prm, 0, sizeof g_prm);
  if (style == 0) { /* realistic */
    g_prm[0x11] = rrange(8, 40);
    g_prm[0x12] = rrange(8, 40);
    g_prm[0x13] = ipick(pat, 4);
    g_prm[0x14] = rrange(5, 13);
    g_prm[0x15] = rrange(1, 12);
    g_prm[0x1b] = ipick(flags, 6);
    g_prm[0x1c] = rrange(1, 12);
    g_prm[0x1f] = rrange(0, 4);
    g_prm[0x21] = rpct(50);
    g_prm[0x24] = rrange(0, 6);
    g_prm[0x25] = rpct(50);
    g_prm[0x28] = rpct(25);
    g_prm[0x29] = rrange(0, 5);
    g_prm[0x2a] = rpct(40);
    g_prm[0x2c] = rrange(1, 6);
  } else { /* wild (within the int range) */
    int k;
    static const int ids[] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x1b, 0x1c, 0x1f, 0x21, 0x24, 0x25, 0x28, 0x29, 0x2a, 0x2c};
    for (k = 0; k < 15; k++) {
      unsigned c = rmod(8);
      long v = c == 0 ? 0 : c == 1 ? 1 : c == 2 ? -1 : c == 3 ? (long)rmod(50) : c == 4 ? (long)(int)rnd() : c == 5 ? (long)((int)rmod(65536) - 32768) : c == 6 ? 0x7fffffffL : (long)rmod(3);
      g_prm[ids[k]] = v;
    }
    g_prm[0x14] = rpct(70) ? rrange(4, 14) : (long)(int)rnd();
    g_prm[0x24] = rpct(70) ? rrange(0, 7) : (long)(int)rnd();
    g_prm[0x13] = rpct(70) ? ipick(pat, 8) : (long)(int)rnd();
  }
}
static unsigned rand_u(void)
{
  switch (rmod(8)) {
    case 0: return 0;
    case 1: return 1;
    case 2: return rmod(12);
    case 3: return rmod(100);
    case 4: return rnd();
    case 5: return 0xffffffffu - rmod(8);
    case 6: return 0x80000000u + rmod(4) - 2u;
    default: return rmod(40);
  }
}
static void rand_pulse(void)
{
  CallPulse *p = &g_glue.pulse;
  unsigned brk, mk;
  p->w5bc = rnd();
  brk = rpct(15) ? rand_u() : rmod(60) * 5u;
  mk = rpct(15) ? rand_u() : rmod(60) * 5u;
  p->breakTime = brk;
  p->makeTime = mk;
  p->count = rpct(20) ? 0u : (rpct(60) ? rmod(11) : rand_u());
  switch (rmod(6)) {
    case 0: p->counter = 0; break;
    case 1: p->counter = brk + rmod(11) - 5u; break;
    case 2: p->counter = brk + mk + rmod(11) - 5u; break;
    case 3: p->counter = rmod(2 * (brk + mk) + 3); break;
    case 4: p->counter = rand_u(); break;
    default: p->counter = 0xfffffffbu + rmod(5); break;
  }
  p->hookOn = rpct(45) ? 0 : (rpct(70) ? 1 : (int)rnd());
}
static void glue_init(void)
{
  memset(&g_glue, 0, sizeof g_glue);
  g_glue.dp.modem = FAKEM;
  g_glue.dp.dp_data = &g_glue;
  g_nodp = 0;
}
static void set_level(int k) { dsplibs_debug_level = lvl_value(k); }
static void pulse_line(const char *tag)
{
  const CallPulse *p = &g_glue.pulse;
  emit(0, "%s pulse %u %u %u %u %u %d", tag, p->makeTime, p->breakTime, p->w5bc, p->count, p->counter, p->hookOn);
}

/* ---------------------------------------------------------------------------------------------- scenario 0: the pulse helpers */
static void scen_pulse(void)
{
  int t;
  g_snap = snap_cur;
  g_d = NULL;
  g_dbg_hook = dbg_cov;
  g_cov_dbg_on = 1;
  for (t = 0; t < 40000; t++) {
    int nops = 1 + (int)rmod(7), k;
    glue_init();
    rand_pulse();
    if (rpct(8)) g_nodp = 1;
    if (rpct(25)) { g_glue.dp.modem = FAKEM2; cov_hit(CV_IR_MODEM2); }
    set_level(rpct(60) ? 0 : (int)rmod(8));
    for (k = 0; k < nops; k++) {
      unsigned op = rmod(10);
      if (op < 4) {
        CallPulse before = g_glue.pulse;
        int r = IsPulseDialerReady(FAKEM);
        if (g_nodp) cov_hit(CV_IR_NODP);
        else if (before.count == 0) cov_hit(CV_IR_COUNT0);
        else if (before.counter >= before.breakTime) {
          if (before.hookOn) cov_hit(CV_IR_HOOKOFF);
          else if (before.counter < before.breakTime + before.makeTime) cov_hit(CV_IR_SAMEPULSE);
          else cov_hit(CV_IR_NEXTPULSE);
        } else if (before.hookOn) {
          cov_hit(CV_IR_HOOK_LT);
          if (before.counter < before.breakTime + before.makeTime) cov_hit(CV_IR_SAMEPULSE);
          else cov_hit(CV_IR_NEXTPULSE);
        } else cov_hit(CV_IR_HOOKON);
        emit(0, "IR -> %d", r);
      } else if (op < 6) {
        unsigned dg = rpct(30) ? 0u : (rpct(60) ? rmod(11) : rand_u());
        PulseDialDigit(FAKEM, dg);
        cov_hit(g_nodp ? CV_PD_NULL : (dg ? CV_PD_DIGIT : CV_PD_ZERO));
      } else if (op == 6) {
        LastPulseDigitDialed(FAKEM);
        cov_hit(CV_LAST);
      } else if (op == 7 || op == 8) {
        unsigned v = rand_u();
        if (op == 7) { SetPulseBreakTime(FAKEM, v); cov_hit(CV_SB); }
        else { SetPulseMakeTime(FAKEM, v); cov_hit(CV_SM); }
      } else {
        emit(0, "IR -> %d", IsPulseDialerReady(FAKEM));
      }
      pulse_line("op");
    }
  }
  /* coverage of the paths the random sequences rarely hit in one pass: make sure the three counter classes exist */
  if (!g_cov[CV_IR_HOOKOFF] || !g_cov[CV_IR_NEXTPULSE]) { /* reported by cov_finish */ }
}

/* ---------------------------------------------------------------------------------------------- scenario 1: pulse sessions */
static void scen_sessions(void)
{
  int t;
  g_snap = snap_cur;
  g_d = NULL;
  g_dbg_hook = dbg_cov;
  g_cov_dbg_on = 0;
  for (t = 0; t < 3000; t++) {
    int digits = 1 + (int)rmod(4), dg, guard;
    glue_init();
    g_glue.pulse.w5bc = rpct(10) ? rnd() : 0;
    SetPulseMakeTime(FAKEM, rmod(6) * 5u + (rpct(5) ? rand_u() % 40u : 0u));
    SetPulseBreakTime(FAKEM, rmod(6) * 5u + (rpct(5) ? rand_u() % 40u : 0u));
    if (rpct(30)) g_glue.dp.modem = FAKEM2;
    set_level(rpct(70) ? 0 : 2);
    for (dg = 0; dg < digits; dg++) {
      int rc = 0;
      PulseDialDigit(FAKEM, rmod(11));
      for (guard = 0; guard < 400; guard++) {
        int hook0 = g_glue.pulse.hookOn;
        unsigned count0 = g_glue.pulse.count;
        rc = IsPulseDialerReady(FAKEM);
        emit(0, "r %d", rc);
        if (!hook0 && g_glue.pulse.hookOn) cov_hit(CV_IR_HOOKON);
        if (hook0 && !g_glue.pulse.hookOn) cov_hit(CV_IR_HOOKOFF);
        if (g_glue.pulse.count < count0) cov_hit(CV_IR_NEXTPULSE);
        if (rc && g_glue.pulse.count == 0) cov_hit(CV_IR_COUNT0);
        if (rc) break;
      }
      pulse_line("end");
      if (rc) cov_hit(CV_SESS_PULSE_END);
      if (rpct(30)) { LastPulseDigitDialed(FAKEM); pulse_line("last"); }
    }
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 2: GetDialerConfig */
static int g_poke_at, g_poke_to, g_poke_ctr;
static void poke_hook(void)
{
  if (++g_poke_ctr == g_poke_at) { dsplibs_debug_level = (unsigned)g_poke_to; cov_hit(CV_G_POKE); }
}
static void dump_cfg(const char *tag, const DialerCfg *c)
{
  emit(0, "%s cfg %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", tag, c->toneDigitLen, c->toneBetween, (int)c->gain1, (int)c->gain2, c->pulseOnHookTime, c->pulseOffHookTime, c->pulseBetween, c->dialPause,
       c->flashTime, c->toneOrPulse, c->modValid, c->abcd, c->pulseAndTone, c->callingTone, c->commaLimit, c->digitPattern);
}
static void scen_config(void)
{
  int t;
  static DialerCfg cfg;
  g_snap = NULL;
  g_d = NULL;
  g_dbg_hook = NULL;
  g_cov_dbg_on = 0;
  for (t = 0; t < 30000; t++) {
    long v14, v24;
    rand_prm(rpct(40) ? 0 : 1);
    if (rpct(40)) {
      static const int lv[] = {-3, -1, 0, 5, 6, 7, 9, 11, 12, 13, 14, 20};
      static const int df[] = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9};
      g_prm[0x14] = lv[rmod(12)];
      g_prm[0x24] = df[rmod(11)];
    }
    v14 = g_prm[0x14];
    v24 = g_prm[0x24];
    cov_hit((unsigned)((int)v14 - 6) <= 6u ? CV_G_LEVEL_IN : CV_G_LEVEL_OUT);
    cov_hit((unsigned)((int)v24 - 1) <= 4u ? CV_G_DIFF_IN : CV_G_DIFF_OUT);
    cov_hit((int)g_prm[0x25] != 0 ? CV_G_FLAG1 : CV_G_FLAG0);
    dirty(&cfg, sizeof cfg);
    set_level(rpct(50) ? 2 : (int)rmod(8));
    cov_hit(dsplibs_debug_level > 1 ? CV_G_DBG : CV_G_NODBG);
    g_event_hook = NULL;
    g_poke_ctr = 0;
    if (rpct(25)) {
      g_poke_at = 1 + (int)rmod(40);
      g_poke_to = (int)lvl_value((int)rmod(8));
      g_event_hook = poke_hook;
    }
    GetDialerConfig(&cfg, FAKEM);
    g_event_hook = NULL;
    dump_cfg("c", &cfg);
  }
}

/* ---------------------------------------------------------------------------------------------- strings */
static const char A_DIAL[] = "0123456789*#ABCDabcd!";
static const char A_MOD[] = ",;@!^WwPpTt $()-";
static void mkstr(char *dst, int len, int style)
{
  int i;
  for (i = 0; i < len; i++) {
    char c;
    switch (style) {
      case 0: c = A_DIAL[rmod(sizeof A_DIAL - 1)]; break;
      case 1: c = rpct(60) ? A_DIAL[rmod(sizeof A_DIAL - 1)] : A_MOD[rmod(sizeof A_MOD - 1)]; break;
      case 2: c = (char)(1 + rmod(255)); break;
      default: c = rpct(50) ? A_DIAL[rmod(sizeof A_DIAL - 1)] : (rpct(50) ? A_MOD[rmod(sizeof A_MOD - 1)] : (char)(1 + rmod(255))); break;
    }
    dst[i] = c;
  }
  dst[len] = 0;
}
static char mkprefix(void)
{
  static const char pf[] = "TTPPtp";
  return rpct(92) ? pf[rmod(6)] : (char)rmod(256);
}

/* independent model of AnalyseDialString (table style), returns the verdict; *last = the last dialable symbol */
static int model_analyse(const DialerCfg *c, const char *s, int *last, int *cls_semi_last, int *cls_semi_mid)
{
  size_t n;
  int mode, verdict = 3, i, lastd = -2;
  *last = -2;
  if (!s) return 3;
  n = strlen(s);
  if (n > 100) return 0;
  if (s[0] == 'T' || s[0] == 't') mode = 1;
  else if (s[0] == 'P' || s[0] == 'p') mode = 0;
  else return s[0] ? 1 : 3;
  for (i = 1; s[i]; i++) {
    unsigned char ch = (unsigned char)s[i];
    int ok = 1;
    if (strchr("0123456789*#!", ch)) lastd = i;
    else if (strchr(" $(),-@Ww", ch)) {
    } else if (ch == ';') {
      if (s[i + 1]) { ok = 0; *cls_semi_mid = 1; } else *cls_semi_last = 1;
    } else if (strchr("ABCDabcd", ch)) {
      if (c->abcd) ok = 0; else lastd = i;
    } else if (ch == '^') {
      if (c->callingTone == 2) ok = 0;
    } else if (strchr("Tt", ch)) {
      if (mode == 0 && c->pulseAndTone) ok = 0;
    } else if (strchr("Pp", ch)) {
      if (mode == 1 && c->pulseAndTone) ok = 0;
    } else ok = 0;
    if (!ok) {
      int cap = c->modValid ? 2 : 1;
      if (verdict > cap) verdict = cap;
    }
  }
  *last = lastd;
  return verdict;
}

static void cov_analysis(const DialerCfg *c, const char *s, int rc, int verdict_expected)
{
  size_t i, n;
  if (!s) { cov_hit(CV_A_NULL); return; }
  n = strlen(s);
  if (n == 0) { cov_hit(CV_A_EMPTY); return; }
  if (n == 100) cov_hit(CV_A_LEN100);
  if (n > 100) { cov_hit(CV_A_TOOLONG); cov_hit(CV_A_ILLEGAL); return; }
  if (s[0] == 't') cov_hit(CV_A_TLOW);
  if (s[0] == 'p') cov_hit(CV_A_PLOW);
  if (!strchr("TtPp", s[0])) { cov_hit(CV_A_NOPREFIX); return; }
  cov_hit(verdict_expected == 3 ? CV_A_VALID : verdict_expected == 2 ? CV_A_TOLERABLE : CV_A_INVALID);
  (void)rc;
  for (i = 1; i < n; i++) {
    unsigned char ch = (unsigned char)s[i];
    int mode = (s[0] == 'T' || s[0] == 't');
    if (ch >= 0x80) cov_hit(CV_A_HIGHCHAR);
    if (strchr("ABCDabcd", ch)) cov_hit(c->abcd ? CV_A_ABCD_BAD : CV_A_ABCD_OK);
    if (ch == '^') cov_hit(c->callingTone == 2 ? CV_A_CALL_BAD : CV_A_CALL_OK);
    if (strchr("Tt", ch) && !mode) cov_hit(c->pulseAndTone ? CV_A_T_IN_P_BAD : CV_A_T_IN_P_OK);
    if (strchr("Pp", ch) && mode) cov_hit(c->pulseAndTone ? CV_A_P_IN_T_BAD : CV_A_P_IN_T_OK);
    if (!strchr("0123456789*#!ABCDabcd ;$(),-@WwTtPp^", ch)) cov_hit(c->modValid ? CV_A_BADCHAR_TOL : CV_A_BADCHAR_STRICT);
  }
}

static void dump_dialer(const char *tag, const Dialer *d)
{
  emit(0, "%s D %016llx P %016llx", tag, hash_dialer(d), hash_pulse());
  if (g_verbose)
    emit(0, "%s fields str %016llx cfg %d %d %d %d %d %d %d %d %d %d %d %d | %d %d %d %d %d %d %u %d %u %u %u %u %d %d %d", tag, fnv_bytes(d->str, sizeof d->str), d->cfg.toneDigitLen, d->cfg.toneBetween,
         d->cfg.gain1, d->cfg.gain2, d->cfg.pulseOnHookTime, d->cfg.pulseOffHookTime, d->cfg.pulseBetween, d->cfg.dialPause, d->cfg.flashTime, d->cfg.toneOrPulse, d->cfg.modValid, d->cfg.abcd, d->lastIdx,
         d->syntax, d->commaCount, d->cur, d->row, d->col, d->state, d->left, d->ph1, d->ph2, d->inc1, d->inc2, d->digitSent, d->pulseActive, d->pulseDone);
}

static void one_create(const char *s, int use_idi)
{
  Dialer *d = &g_dl;
  int rc, last, sl = 0, sm = 0, expect;
  dirty(d, DPFX);
  d->modem = FAKEM2; /* DialerCreate must store its argument (the pulse helpers it calls use d->modem) */
  g_d = d;
  rc = DialerCreate(d, s, FAKEM);
  dump_dialer("create", d);
  emit(0, "create rc %d syntax %d last %d str %016llx", rc, d->syntax, d->lastIdx, fnv_bytes(d->str, sizeof d->str));
  if (s) {
    expect = model_analyse(&d->cfg, s, &last, &sl, &sm);
    if (d->syntax != expect) die3("syntax verdict %d, model %d for string %02x%02x%02x", d->syntax, expect, (unsigned char)s[0], s[0] ? (unsigned char)s[1] : 0, s[0] && s[1] ? (unsigned char)s[2] : 0);
    if (d->lastIdx != last && !(expect <= 0)) die3("lastIdx %d, model %d", d->lastIdx, last);
    if (expect <= 1 ? rc != 7 : rc != 0) die3("DialerCreate rc %d for verdict %d", rc, expect);
    if (expect <= 1) cov_hit(CV_A_RET7); else cov_hit(CV_A_COPY);
    if (last >= 0) cov_hit(CV_A_LAST_SYMBOL); else if (expect >= 2 && strchr("TtPp", s[0])) cov_hit(CV_A_NOLAST);
    if (sl) cov_hit(CV_A_SEMI_LAST);
    if (sm) cov_hit(CV_A_SEMI_MID);
    cov_analysis(&d->cfg, s, rc, expect);
  } else {
    cov_analysis(&d->cfg, s, rc, 3);
    cov_hit(CV_C_NULLSTR);
  }
  if (use_idi) {
    int r;
    /* IsDialStringInvalid on the same object, then on the embedded object of a container */
    dirty(&g_cp, 0x98); /* (the i386 offset of the embedded Dialer: the draw count must not depend on the host layout) */
    dirty((char *)&g_cp + CALLPROG_DIALER_OFF, DPFX);
    memcpy(&((Dialer *)((char *)&g_cp + CALLPROG_DIALER_OFF))->cfg, &d->cfg, sizeof d->cfg);
    r = IsDialStringInvalid(d, s ? s : "");
    cov_hit(CV_A_IDI);
    dump_dialer("idi", d);
    emit(0, "idi -> %d", r);
    {
      Dialer *e = (Dialer *)((char *)&g_cp + CALLPROG_DIALER_OFF);
      e->modem = FAKEM;
      g_d = e;
      r = Dialer_IsDialStringInvalid(&g_cp, s ? s : "");
      cov_hit(CV_A_DIDI);
      dump_dialer("didi", e);
      emit(0, "didi -> %d", r);
    }
  }
}
static void scen_analysis(void)
{
  int t, c, k;
  char buf[160];
  g_snap = snap_cur;
  g_dbg_hook = NULL;
  g_cov_dbg_on = 0;
  /* every character after every prefix, under every configuration that matters */
  for (k = 0; k < 64; k++) {
    int flags = k;
    glue_init();
    rand_prm(0);
    g_prm[0x21] = (flags & 1) != 0;
    g_prm[0x28] = (flags & 2) != 0;
    g_prm[0x2a] = (flags & 4) != 0;
    g_prm[0x1b] = (flags >> 3) & 3;
    set_level(k & 1 ? 2 : 0);
    for (c = 0; c < 256; c++) {
      static const char pf[] = "TtPp";
      int p;
      for (p = 0; p < 4; p++) {
        buf[0] = pf[p];
        buf[1] = (char)c;
        buf[2] = 0;
        if (c) one_create(buf, (c & 7) == 0);
        /* a second character after: ';' handling and the mix */
        buf[1] = '5';
        buf[2] = (char)c;
        buf[3] = c ? '9' : 0;
        buf[4] = 0;
        one_create(buf, 0);
        buf[0] = pf[p];
        buf[1] = (char)(c ? c : 'x');
        buf[2] = ';';
        buf[3] = 0;
        one_create(buf, 0);
      }
      /* the first character itself */
      buf[0] = (char)c;
      buf[1] = '1';
      buf[2] = 0;
      one_create(buf, (c & 15) == 0);
    }
  }
  /* random strings */
  for (t = 0; t < 12000; t++) {
    int len = rpct(10) ? 99 + (int)rmod(5) : (int)rmod(40), st = (int)rmod(4);
    glue_init();
    rand_prm(rpct(70) ? 0 : 1);
    g_prm[0x1b] = rmod(5);
    set_level(rpct(70) ? 0 : (int)rmod(8));
    buf[0] = mkprefix();
    if (len > 0) mkstr(buf + 1, len, st); else buf[1] = 0;
    if (rpct(3)) one_create(NULL, 1); else one_create(buf, rpct(15));
  }
  /* IsDialStringInvalid / Dialer_IsDialStringInvalid with a NULL string (AnalyseDialString: verdict VALID, nothing recorded) */
  for (k = 0; k < 4; k++) {
    Dialer *d = &g_dl;
    glue_init();
    rand_prm(0);
    set_level(k & 1 ? 2 : 0);
    dirty(d, DPFX);
    d->modem = FAKEM;
    g_d = d;
    emit(0, "idi NULL -> %d", IsDialStringInvalid(d, NULL));
    dump_dialer("idiN", d);
    cov_hit(CV_A_NULL);
  }
  one_create("", 1);
  one_create("T", 1);
  one_create("P;", 1);
  one_create("T;;", 1);
  one_create("P1;2", 1);
}

/* ---------------------------------------------------------------------------------------------- scenario 4: create state, abort, calling tone */
static void scen_abort_tone(void)
{
  int t, k;
  static short out[40000 + 64];
  g_snap = snap_cur;
  g_dbg_hook = NULL;
  g_cov_dbg_on = 0;
  /* DialerAbort over the states */
  for (t = 0; t < 8000; t++) {
    Dialer *d = &g_dl;
    static const unsigned sts[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0x80000000u, 0xffffffffu, 100};
    glue_init();
    rand_pulse();
    dirty(d, DPFX);
    d->modem = FAKEM;
    d->state = sts[rmod(16)];
    d->pulseDone = rpct(50) ? 0 : (rpct(60) ? 1 : (int)rnd());
    d->pulseActive = rpct(50) ? 0 : (rpct(60) ? 1 : (int)rnd());
    g_d = d;
    set_level(rpct(40) ? 0 : (int)rmod(8));
    DialerAbort(d);
    dump_dialer("abort", d);
    if (d->state > 10u) cov_hit(CV_ABORT_ST_BAD);
    else {
      cov_hit(CV_ABORT_ST_OK);
      if (d->pulseDone && d->pulseDone != 1) cov_hit(CV_ABORT_DONE);
    }
  }
  /* abort with pulse flags known: the rare combinations deterministically */
  for (k = 0; k < 8; k++) {
    Dialer *d = &g_dl;
    glue_init();
    dirty(d, DPFX);
    d->modem = FAKEM;
    d->state = (unsigned)(k % 3) * 5u;
    d->pulseDone = k & 1;
    d->pulseActive = (k >> 1) & 1;
    g_d = d;
    set_level(k & 4 ? 2 : 0);
    DialerAbort(d);
    dump_dialer("abort2", d);
    if (!d->pulseDone || (k & 1)) { }
    if (!(k & 1) && ((k >> 1) & 1) && d->state <= 10u) cov_hit(CV_ABORT_PULSE);
    if (!(k & 1) && !((k >> 1) & 1) && d->state <= 10u) cov_hit(CV_ABORT_NOPULSE_ACTIVE0);
    if ((k & 1) && d->state <= 10u) cov_hit(CV_ABORT_DONE);
  }
  /* DialerCreate over dirty objects with a few strings: the create-state stores and the Set*Time calls */
  for (t = 0; t < 300; t++) {
    glue_init();
    rand_prm(rpct(70) ? 0 : 1);
    set_level(rpct(60) ? 0 : 2);
    if (rpct(30)) one_create(NULL, 0); else { char b[24]; b[0] = mkprefix(); mkstr(b + 1, (int)rmod(12), (int)rmod(2)); one_create(b, 0); }
  }
  /* calling tone */
  for (t = 0; t < 3000; t++) {
    CallingTone ct;
    int total = 0, chunks = 1 + (int)rmod(4), c;
    static const int lv[] = {0, 1, -1, 10, -10, 20, 127, -128, 63, -63};
    int level = rpct(80) ? ipick(lv, 10) : (int)rnd();
    glue_init();
    set_level(rpct(70) ? 0 : 2);
    memset(&ct, 0xa5, sizeof ct);
    ResetCallingTone(&ct, level);
    cov_hit(CV_CT_RESET);
    emit(0, "ct reset %u %d %d %d", ct.phase, ct.on, ct.cnt, (int)ct.gain);
    if (rpct(50)) { /* random tone object */
      ct.phase = (unsigned short)(rpct(70) ? rmod(0x4000) : rnd());
      if (ct.phase > 0x3fff) cov_hit(CV_CT_BIGPHASE);
      ct.on = rpct(45) ? 0 : (rpct(80) ? 1 : (int)rmod(5) + 2);
      if (ct.on > 1) cov_hit(CV_CT_ONOTHER);
      ct.cnt = rpct(10) ? 0 : (rpct(30) ? (int)rmod(20) : (rpct(10) ? -(int)rmod(10) : (int)rmod(20000)));
      if (ct.cnt == 0) cov_hit(CV_CT_ZEROCNT);
    }
    for (c = 0; c < chunks; c++) {
      int n = rpct(10) ? (int)rmod(3) : (rpct(5) ? -(int)rmod(5) : (rpct(40) ? (int)rmod(300) : (int)rmod(5700)));
      int i;
      if (rpct(20) && ct.cnt > 0 && ct.cnt <= 5700) n = ct.cnt + (int)rmod(3) - 1; /* around the segment end */
      /* NOTE: n > 5760 can hang the blob (and the port): min(cnt, n) is compared with the ABSOLUTE index, so a 5760 sample segment ends before i */
      if (n <= 0) cov_hit(CV_CT_NEGN);
      if (n > 5760) n = 5760;
      for (i = 0; i < (n > 0 ? n : 0) + 8; i++) out[i] = (short)0x5a5a;
      {
        CallingTone b = ct;
        GenerateCallingTone(&ct, out, n);
        if (n > 0) {
          if (b.on && b.cnt > 0) cov_hit(CV_CT_TONE);
          if (!b.on && b.cnt > 0) cov_hit(CV_CT_SILENCE);
          if (b.cnt > n) cov_hit(CV_CT_PARTIAL);
          if (b.cnt == n) cov_hit(CV_CT_CHUNKEDGE);
          if (b.on && ct.on == 0) cov_hit(CV_CT_SWITCH_OFF);
          if (!b.on && ct.on == 1) cov_hit(CV_CT_SWITCH_ON);
        }
      }
      for (i = (n > 0 ? n : 0); i < (n > 0 ? n : 0) + 8; i++)
        if (out[i] != (short)0x5a5a) die3("calling tone wrote past n");
      emit(0, "ct gen n %d out %016llx -> %u %d %d %d", n, fnv_shorts(out, (unsigned)(n > 0 ? n : 0)), ct.phase, ct.on, ct.cnt, (int)ct.gain);
      total += n;
    }
    (void)total;
  }
  /* long runs through every segment */
  for (t = 0; t < 12; t++) {
    CallingTone ct;
    int i, n;
    glue_init();
    set_level(0);
    ResetCallingTone(&ct, t * 9 - 50);
    for (i = 0; i < 12; i++) {
      n = 1000 + (int)rmod(4700);
      GenerateCallingTone(&ct, out, n);
      emit(0, "ct long n %d out %016llx -> %u %d %d", n, fnv_shorts(out, (unsigned)n), ct.phase, ct.on, ct.cnt);
    }
  }
}

/* ---------------------------------------------------------------------------------------------- DialerProgress harness pieces */
#define BUFN 1400
static short g_buf[BUFN + 64];
static int g_buf_init;
static int progress_call(Dialer *d, int pos0, int last, const char *tag)
{
  static unsigned ncalls;
  int pos = pos0, rc, i, lo, hi;
  if (!g_buf_init) {
    for (i = 0; i < BUFN + 64; i++) g_buf[i] = (short)(0x7000 + (i & 0xff));
    g_buf_init = 1;
  }
  g_d = d;
  rc = DialerProgress(d, g_buf, &pos, last);
  /* the samples outside [pos0, last] must be untouched: the neighbourhood every call, the whole buffer every 64th call */
  ncalls++;
  lo = (ncalls & 63u) == 0 ? 0 : (pos0 - 16 > 0 ? pos0 - 16 : 0);
  hi = (ncalls & 63u) == 0 ? BUFN + 64 : (last + 64 < BUFN + 64 ? last + 64 : BUFN + 64);
  for (i = lo; i < hi; i++) {
    int inside = i >= pos0 && i <= last;
    if (!inside && g_buf[i] != (short)(0x7000 + (i & 0xff))) die3("%s: sample %d outside [%d, %d] written", tag, i, pos0, last);
  }
  emit(0, "%s rc %d pos %d buf %016llx D %016llx P %016llx", tag, rc, pos, last >= pos0 ? fnv_shorts(g_buf + pos0, (unsigned)(last - pos0 + 1)) : 0ULL, hash_dialer(d), hash_pulse());
  for (i = pos0 < 0 ? 0 : pos0; i <= last && i < BUFN + 64; i++) g_buf[i] = (short)(0x7000 + (i & 0xff)); /* restore the pattern */
  cov_hit(rc >= 0 && rc <= 7 ? CV_P_RC0 + rc : CV_P_RC0);
  if (pos0 > last) cov_hit(CV_P_POSGT);
  return rc;
}

static void rand_dialer_for_state(Dialer *d, unsigned state)
{
  int len = rpct(8) ? 0 : 1 + (int)rmod(18), i;
  dirty(d, DPFX);
  d->modem = rpct(15) ? FAKEM2 : FAKEM;
  d->str[0] = mkprefix();
  mkstr(d->str + 1, len, (int)rmod(4));
  if (rpct(60)) { /* a simpler string: digits and a few modifiers */
    static const char sc[] = "T1234567890*#,W@$^;!A";
    for (i = 0; i < len; i++) d->str[1 + i] = sc[rmod(sizeof sc - 1)];
  }
  d->cfg.toneOrPulse = rpct(50);
  d->cfg.modValid = rpct(50);
  d->cfg.abcd = rpct(30);
  d->cfg.pulseAndTone = rpct(50);
  d->cfg.callingTone = (int)rmod(5);
  d->cfg.digitPattern = rpct(80) ? (int)rmod(4) : (rpct(50) ? -1 : (int)rnd());
  d->cfg.toneDigitLen = rpct(5) ? 0 : (rpct(80) ? 1 + (int)rmod(8) : (rpct(50) ? (int)rmod(100) : (int)rnd()));
  d->cfg.toneBetween = rpct(80) ? (int)rmod(8) : (int)rnd();
  d->cfg.pulseBetween = rpct(80) ? (int)rmod(40) : (int)rnd();
  d->cfg.dialPause = rpct(80) ? (int)rmod(4) : (int)rnd();
  d->cfg.commaLimit = rpct(80) ? (int)rmod(8) : (int)rnd();
  d->cfg.flashTime = rpct(80) ? (int)rmod(20) : (int)rnd();
  d->cfg.pulseOnHookTime = (int)(rpct(80) ? rmod(40) : rnd());
  d->cfg.pulseOffHookTime = (int)(rpct(80) ? rmod(40) : rnd());
  d->cfg.gain1 = (short)rnd();
  d->cfg.gain2 = (short)rnd();
  d->syntax = (int)rmod(4);
  d->commaCount = 1 + (int)rmod(5);
  if (rpct(10)) d->commaCount = (int)rnd();
  d->cur = len > 0 ? -1 + (int)rmod((unsigned)len + 1) : -1;
  if (d->cur >= len + 1) d->cur = len;
  d->lastIdx = rpct(50) ? d->cur : -2 + (int)rmod((unsigned)len + 4);
  d->row = (int)rmod(4);
  d->col = (int)rmod(4);
  if (rpct(5)) { d->row = (int)rmod(5); d->col = (int)rmod(10); }
  d->state = state;
  switch (rmod(7)) {
    case 0: d->left = 0; break;
    case 1: d->left = 1; break;
    case 2: d->left = (int)rmod(30); break;
    case 3: d->left = (int)rmod(500); break;
    case 4: d->left = -(int)rmod(20); break;
    case 5: d->left = (int)(0x7fffffff - rmod(5)); break;
    default: d->left = (int)rnd(); break;
  }
  d->digitSent = rpct(50) ? 0 : (rpct(70) ? 1 : (int)rnd());
  d->pulseActive = rpct(50) ? 0 : (rpct(70) ? 1 : (int)rnd());
  d->pulseDone = rpct(50) ? 0 : (rpct(70) ? 1 : (int)rnd());
  glue_init();
  rand_pulse();
  if (rpct(20)) { g_glue.pulse.count = 0; }
  if (rpct(25)) g_glue.dp.modem = FAKEM2;
}

static void scen_progress(void)
{
  int t;
  g_snap = snap_cur;
  g_dbg_hook = dbg_cov;
  g_cov_dbg_on = 1;
  for (t = 0; t < 70000; t++) {
    Dialer *d = &g_dl;
    static const unsigned sts[] = {0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0x80000000u, 0xffffffffu, 1, 0, 2, 4};
    unsigned st = sts[rmod(24)];
    int pos0, last, rc, calls, c;
    rand_dialer_for_state(d, st);
    if (st == 1) { /* never toneOrPulse not in {0, 1} (blob: endless loop) */ d->cfg.toneOrPulse = rpct(50); }
    set_level(rpct(45) ? 0 : (rpct(80) ? 2 : (int)rmod(8)));
    if (d->cfg.toneDigitLen == 0 && st == 1 && d->cfg.toneOrPulse) cov_hit(CV_P_TONE_LEN0);
    calls = 1 + (int)rmod(3);
    pos0 = rpct(70) ? (int)rmod(40) : (int)rmod(600);
    last = pos0 + (rpct(10) ? -1 - (int)rmod(3) : (int)(rpct(50) ? rmod(40) : rmod(400)));
    if (last >= BUFN) last = BUFN - 1;
    if (last < -1) last = -1;
    for (c = 0; c < calls; c++) {
      int before_left = d->left;
      unsigned stb = d->state;
      int ds_before = d->digitSent;
      rc = progress_call(d, pos0, last, "pr");
      if ((stb == 2 || stb == 4) && before_left > 0 && before_left > last - pos0 + 1 && pos0 <= last) cov_hit(CV_P_PARTIAL_SIL);
      if (stb == 1 && d->cfg.toneOrPulse == 1 && before_left > last - pos0 + 1 && pos0 <= last && d->cfg.toneDigitLen != 0) cov_hit(CV_P_PARTIAL_TONE);
      if (stb == 3 && ds_before == 0 && d->digitSent == 0 && d->state == 3 && rc == 0) cov_hit(CV_P_FLASH_NOTREADY1);
      if (stb == 3 && d->digitSent != 0 && d->state == 3 && rc == 0) cov_hit(CV_P_FLASH_NOTREADY2);
      if (stb == 1 && !d->cfg.toneOrPulse && d->digitSent && d->state == 1) cov_hit(CV_P_PULSE_NOTREADY2);
      if (rc >= 1 && rc <= 4) { pos0 = rpct(50) ? 0 : (int)rmod(50); last = pos0 + (int)rmod(100); }
      else if (d->state == 10u || d->state == 9u) break;
      else { pos0 = 0; last = (int)rmod(160); }
    }
  }
  /* the flash states with the dialer ready / not ready combinations */
  for (t = 0; t < 4000; t++) {
    Dialer *d = &g_dl;
    int c;
    rand_dialer_for_state(d, 3);
    set_level(rpct(50) ? 2 : 0);
    d->cfg.flashTime = (int)rmod(6);
    d->cfg.pulseBetween = (int)rmod(10);
    for (c = 0; c < 80; c++) {
      int rc = progress_call(d, 0, (int)rmod(60), "fl");
      if (d->state != 3) break;
      (void)rc;
    }
    if (d->state != 3) { }
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 6: GetNext through state 0 */
static void scen_getnext(void)
{
  int t, c, k;
  g_snap = snap_cur;
  g_dbg_hook = dbg_cov;
  g_cov_dbg_on = 1;
  for (k = 0; k < 64; k++) {
    for (c = 0; c < 256; c++) {
      int variant;
      for (variant = 0; variant < 6; variant++) {
        Dialer *d = &g_dl;
        char *s;
        int i, rc, pos0 = 0;
        glue_init();
        dirty(d, DPFX);
        d->modem = FAKEM;
        d->cfg.toneOrPulse = k & 1;
        d->cfg.abcd = (k >> 1) & 1;
        d->cfg.pulseAndTone = (k >> 2) & 1;
        d->cfg.callingTone = (k >> 3) & 3;
        d->cfg.digitPattern = (k >> 5) & 1 ? 3 : 1;
        d->cfg.toneDigitLen = 1 + (c & 3);
        d->cfg.toneBetween = c & 3;
        d->cfg.pulseBetween = c & 7;
        d->cfg.dialPause = 1 + (c & 3);
        d->cfg.commaLimit = 1 + (k & 3);
        d->cfg.modValid = 0;
        d->commaCount = 7;
        s = d->str;
        memset(s, 0, sizeof d->str);
        for (i = 0; i < 4; i++) s[i] = "0123"[i];
        switch (variant) {
          case 0: s[0] = (char)c; s[1] = 0; d->cur = -1; break;                          /* cur becomes 0: the first character */
          case 1: s[0] = 'T'; s[1] = (char)c; s[2] = '5'; s[3] = 0; d->cur = 0; break;  /* cur becomes 1 */
          case 2: s[0] = 'P'; s[1] = (char)c; s[2] = ';'; s[3] = 0; d->cur = 0; break;
          case 3: s[0] = 'T'; s[1] = (char)c; s[2] = (char)c; s[3] = 'W'; s[4] = 0; d->cur = 0; break; /* ',' runs, ';' not last */
          case 4: s[0] = 'P'; s[1] = (char)c; s[2] = ','; s[3] = (char)c; s[4] = ',', s[5] = ',', s[6] = 0; d->cur = 0; break;
          default: s[0] = 'T'; s[1] = (char)c; s[2] = 0; d->cur = 0; break;
        }
        d->state = 0;
        d->left = 0;
        d->digitSent = 0;
        d->row = d->col = 9;
        set_level(k & 1 ? 2 : (k & 2 ? 0 : 2));
        g_d = d;
        (void)pos0;
        rc = progress_call(d, 0, 20 + (variant == 5 ? 300 : 0), "gn");
        (void)rc;
        /* the same position again, now continuing with the state the first call left */
        if (d->state == 10u) cov_hit(CV_N_END);
        if (d->state <= 10u) rc = progress_call(d, 0, 30, "gn2");
      }
    }
  }
  for (t = 0; t < 20000; t++) {
    Dialer *d = &g_dl;
    int len = 1 + (int)rmod(30), i;
    glue_init();
    rand_dialer_for_state(d, 0);
    memset(d->str, 0, sizeof d->str);
    d->str[0] = mkprefix();
    mkstr(d->str + 1, len, (int)rmod(4));
    d->cur = -1;
    d->cfg.toneOrPulse = rpct(50);
    set_level(rpct(60) ? 2 : 0);
    for (i = 0; i < 8 && d->state <= 10u; i++) {
      int rc = progress_call(d, 0, 10 + (int)rmod(200), "gr");
      if (rc >= 5) break;
    }
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 7: full dial sessions */
static void scen_session(void)
{
  int t;
  char s[128];
  g_snap = snap_cur;
  g_dbg_hook = dbg_cov;
  g_cov_dbg_on = 1;
  for (t = 0; t < 2500; t++) {
    Dialer *d = &g_dl;
    int len, it, rc, abort_at = -1, ended = 0, lvl;
    static const char tail[] = "0123456789";
    glue_init();
    rand_prm(0);
    g_prm[0x1b] = rmod(5);
    g_prm[0x13] = 1 + (long)rmod(3); /* patterns 0 / 4 / -1 dial -1 = 4 billion pulses: the matrix scenario covers them */
    /* shorter pulse times keep the sessions short, the tone digit lengths are in 10 ms units */
    g_prm[0x11] = rrange(5, 12);
    g_prm[0x12] = rrange(5, 12);
    len = rpct(4) ? 100 : 1 + (int)rmod(rpct(15) ? 60 : 14);
    s[0] = mkprefix();
    if (!strchr("TtPp", s[0]) && rpct(95)) s[0] = "TP"[rmod(2)];
    {
      int i;
      for (i = 0; i < len; i++) {
        unsigned r = rmod(100);
        s[1 + i] = r < 55 ? tail[rmod(10)] : (r < 62 ? ',' : (r < 66 ? '!' : (r < 70 ? 'W' : (r < 73 ? '@' : (r < 76 ? '$' : (r < 80 ? '^' : (r < 84 ? 'P' : (r < 88 ? 'T' : (r < 91 ? '*' : (r < 94 ? '#' : (r < 97 ? "ABCD"[rmod(4)] : ';')))))))))));
      }
      s[1 + len] = 0;
    }
    lvl = rpct(55) ? 0 : 2;
    set_level(lvl);
    g_d = d;
    dirty(d, DPFX);
    d->modem = FAKEM;
    rc = DialerCreate(d, s, FAKEM);
    emit(0, "ses create rc %d D %016llx", rc, hash_dialer(d));
    if (rc != 0) continue;
    if (rpct(10)) abort_at = (int)rmod(60);
    for (it = 0; it < 30000 && !ended; it++) {
      int n = rpct(30) ? 160 : (rpct(30) ? 1 + (int)rmod(16) : 1 + (int)rmod(500));
      rc = progress_call(d, 0, n - 1, "ses");
      if (rc >= 5) ended = 1;
      if (it == abort_at) {
        DialerAbort(d);
        emit(0, "ses abort D %016llx P %016llx", hash_dialer(d), hash_pulse());
        ended = 1;
      }
    }
    if (ended) DialerAbort(d);
    emit(0, "ses end it %d D %016llx", it, hash_dialer(d));
  }
}

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : -1;
  const char *e;
  if ((e = getenv("R10_VERBOSE")) && *e == '1') g_verbose = 1;
  if ((e = getenv("R10_SEED")) && *e) g_lcg ^= (unsigned)strtoul(e, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)scen * 0x9e3779b9u;
  glue_init();
  cov_setup(scen);
  switch (scen) {
    case 0: scen_pulse(); break;
    case 1: scen_sessions(); break;
    case 2: scen_config(); break;
    case 3: scen_analysis(); break;
    case 4: scen_abort_tone(); break;
    case 5: scen_progress(); break;
    case 6: scen_getnext(); break;
    case 7: scen_session(); break;
    default: fprintf(stderr, "usage: %s scenario(0..7)\n", argv[0]); return 1;
  }
  cov_finish();
  return 0;
}
