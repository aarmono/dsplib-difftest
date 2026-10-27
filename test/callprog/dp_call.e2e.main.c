/*
 * R11 (id r11_e2e): END-TO-END differential test of the whole call-progress stack with NOTHING between the pieces replaced: CALLPROG (src/callprog/callprog.c) + the real cadence / toneiir
 * (R09), dialer + calling tone + pulse helpers (R10), Dual_TONE / IIR pre-filter / TONE_read / FP_Pow (R11), RcFixed (R01) and the call dp glue (src/callprog/dp_call.c) vs the real dsplibs.o
 * (every blob function real).  Only the slmodemd side is a recorder (modem_get_param / modem_get_sreg / modem_set_param scripted, modem_dp_register, sysdep_*, dsplibs_debug_printf).
 * The line is a test-signal generator: integer resonators (no libm) playing call-progress schedules (silence, dial tone 350 + 440, busy 480 + 620 at 0.5 / 0.5 s, congestion at
 * 0.25 / 0.25 s, ringback 440 + 480 at 2 / 4 s, 2100 Hz and 2250 Hz answer tones, voice-like noise bursts, noise).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 CALLPROG alone at 8000 Hz (CALLPROG_Create / _Dial / _Progress / _Delete): 48 line schedules x filter / threshold / cadence parameter variants, tone / pulse / blind / calling-tone
 *     dialling, 160-sample (and odd) buffers, every return code and the whole object hashed
 *   1 the call glue through the dp_operations record (call_create / call_run / call_delete) at 8000, 9600 and 48000 Hz with the real RcFixed converters, the same schedules, chunks
 *     160 / 96 / 47 / 48 / 49 / 1, the dp status returned to the caller, the output audio hashed
 *   2 whole-dial sessions (ATD-like): dial strings of every kind through the glue at 8000 / 9600 with the schedule of an answering modem (dial tone, DTMF absorbed, ringback, 2100 Hz
 *     answer tone), the debug level 2 text and the pulse helper callbacks hashed
 * Coverage counters (debug text and return codes seen) exit 2 on a zero, 3 = harness violation.  Env R11_SEED, R11_VERBOSE=1.
 */
#include "r11_common.h"

extern int dp_call_init(void);
extern void dp_call_exit(void);

enum { CV_DIALTONE = 0, CV_BUSY, CV_NOANS, CV_2100, CV_2250, CV_TIMEOUT, CV_SILENCE5, CV_LC, CV_WAITDIAL, CV_AT, CV_DOLLAR, CV_CARET, CV_PARTIAL, CV_DSTRERR, CV_STATE, CV_ENDED, CV_PULSE, CV_BLIND, CV_CALLTONE,
       CV_R0 = 30 /* 30 .. 60 return codes */, CV_ST0 = 70 /* glue statuses */ };

/* ------------------------------------------------------------------------------------------------ the scripted slmodemd side */
static struct dp *g_dp;
static struct dp_operations *g_op;
static unsigned long g_nset;
long modem_set_param(struct modem *m, unsigned name, int val)
{
  if (m != FAKEM && m != FAKEM2) die3("modem_set_param with a foreign modem");
  g_nset++;
  emit(0, "SPAR m%d %u %d", MID(m), name, val);
  return 0;
}
/* modem_get_param has its own id 0xe here (the dp that owns the pulse state): wrap the common one through a per-id table */
int modem_dp_register(enum DP_ID id, struct dp_operations *op)
{
  emit(0, "REG %d name=%s", (int)id, op->name);
  g_op = op;
  return 0;
}
void modem_dp_deregister(enum DP_ID id, struct dp_operations *op)
{
  (void)op;
  emit(0, "DEREG %d", (int)id);
}

/* ------------------------------------------------------------------------------------------------ line schedules */
typedef struct {
  int len, type;
} Seg;
enum { T_SIL, T_NOISE, T_DIAL, T_BUSY_ON, T_RING_ON, T_CONG_ON, T_A2100, T_A2250, T_VOICE, T_QUIET, T_TONE1000, T_BUSY_OFF = T_SIL };
#define MAXSEG 40
typedef struct {
  Seg seg[MAXSEG];
  int n;
} Sched;
static void seg_add(Sched *s, int len, int type)
{
  if (s->n < MAXSEG) {
    s->seg[s->n].len = len;
    s->seg[s->n].type = type;
    s->n++;
  }
}
static void sched_make(Sched *s, int kind, int fs)
{
  int k, u = fs / 100; /* 10 ms */
  memset(s, 0, sizeof *s);
  switch (kind) {
  case 0: seg_add(s, 30 * u, T_SIL); seg_add(s, 300 * u, T_DIAL); seg_add(s, 150 * u, T_SIL); for (k = 0; k < 2; k++) { seg_add(s, 200 * u, T_RING_ON); seg_add(s, 400 * u, T_SIL); } seg_add(s, 300 * u, T_A2100); seg_add(s, 300 * u, T_SIL); break;
  case 1: seg_add(s, 30 * u, T_SIL); seg_add(s, 300 * u, T_DIAL); seg_add(s, 150 * u, T_SIL); for (k = 0; k < 12; k++) { seg_add(s, 50 * u, T_BUSY_ON); seg_add(s, 50 * u, T_SIL); } break;
  case 2: seg_add(s, 30 * u, T_SIL); seg_add(s, 300 * u, T_DIAL); seg_add(s, 150 * u, T_SIL); for (k = 0; k < 16; k++) { seg_add(s, 25 * u, T_CONG_ON); seg_add(s, 25 * u, T_SIL); } break;
  case 3: seg_add(s, 600 * u, T_SIL); break;
  case 4: seg_add(s, 30 * u, T_SIL); seg_add(s, 300 * u, T_DIAL); seg_add(s, 150 * u, T_SIL); for (k = 0; k < 2; k++) { seg_add(s, 200 * u, T_RING_ON); seg_add(s, 400 * u, T_SIL); } seg_add(s, 400 * u, T_A2250); break;
  case 5: seg_add(s, 30 * u, T_SIL); seg_add(s, 300 * u, T_DIAL); seg_add(s, 150 * u, T_SIL); seg_add(s, 200 * u, T_RING_ON); seg_add(s, 400 * u, T_SIL); for (k = 0; k < 6; k++) { seg_add(s, 80 * u, T_VOICE); seg_add(s, 30 * u, T_SIL); } break;
  case 6: seg_add(s, 60 * u, T_NOISE); seg_add(s, 900 * u, T_NOISE); break;
  case 10: seg_add(s, 1400 * u, T_SIL); break;
  case 7: seg_add(s, 100 * u, T_DIAL); seg_add(s, 100 * u, T_SIL); seg_add(s, 100 * u, T_DIAL); seg_add(s, 600 * u, T_SIL); break;
  case 8: seg_add(s, 30 * u, T_SIL); seg_add(s, 400 * u, T_DIAL); seg_add(s, 300 * u, T_TONE1000); seg_add(s, 400 * u, T_QUIET); for (k = 0; k < 4; k++) { seg_add(s, 100 * u, T_RING_ON); seg_add(s, 200 * u, T_SIL); } break;
  default: /* random schedule */
    seg_add(s, rrange(0, 100) * u, T_SIL);
    for (k = 0; k < 12; k++) seg_add(s, rrange(5, 300) * u, rrange(0, 10));
    break;
  }
}
static int sched_total(const Sched *s)
{
  int i, t = 0;
  for (i = 0; i < s->n; i++) t += s->seg[i].len;
  return t;
}
/* the whole line signal at fs (8000 / 9600 / 48000): integer resonators; the frequencies are the real ones at 8000 / 9600, at 48000 the 9600 constants (still tones) */
static void render(short *buf, const Sched *s, int fs, int amp)
{
  int i, k = 0;
  int tab = fs == 48000 ? 9600 : fs;
  for (i = 0; i < s->n; i++) {
    Osc a, b;
    int type = s->seg[i].type, f1 = 0, f2 = 0, j;
    switch (type) {
    case T_DIAL: f1 = 350; f2 = 440; break;
    case T_BUSY_ON: f1 = 480; f2 = 620; break;
    case T_CONG_ON: f1 = 480; f2 = 620; break;
    case T_RING_ON: f1 = 440; f2 = 480; break;
    case T_A2100: f1 = 2100; f2 = 2100; break;
    case T_A2250: f1 = 2250; f2 = 2250; break;
    case T_TONE1000: f1 = 1000; f2 = 1000; break;
    default: break;
    }
    osc_init(&a, osc_c(f1, tab), amp);
    osc_init(&b, osc_c(f2, tab), amp);
    for (j = 0; j < s->seg[i].len; j++, k++) {
      int x;
      switch (type) {
      case T_SIL: x = 0; break;
      case T_NOISE: x = (int)rmod((unsigned)(2 * amp / 4 + 1)) - amp / 4; break;
      case T_VOICE: x = (j / 40) & 1 ? (int)rmod(4001) - 2000 : (int)rmod(801) - 400; break;
      case T_QUIET: x = (osc_next(&a) >> 6); break;
      case T_A2100: case T_A2250: case T_TONE1000: x = osc_next(&a); break;
      default: x = (osc_next(&a) + osc_next(&b)) / 2; break;
      }
      buf[k] = (short)x;
    }
  }
}

/* ------------------------------------------------------------------------------------------------ the object under test (identity-free hashes) */
static CallProg g_cp;
static CallGlue *g_glue;
static u64 g_runh;
static u64 snap(void) { return g_runh; }
static void dbg_hook(const char *m)
{
  if (strstr(m, "tone detected by cadence dial")) cov_hit(CV_DIALTONE);
  else if (strstr(m, "busy detected by cadence")) cov_hit(CV_BUSY);
  else if (strstr(m, "no answer detected")) cov_hit(CV_NOANS);
  else if (strstr(m, "Found 2100")) cov_hit(CV_2100);
  else if (strstr(m, "Found 2250")) cov_hit(CV_2250);
  else if (strstr(m, "Time out")) cov_hit(CV_TIMEOUT);
  else if (strstr(m, "5 sec. silence")) cov_hit(CV_SILENCE5);
  else if (strstr(m, "LINE CLEAR")) cov_hit(CV_LC);
  else if (strstr(m, "Wait dial tone")) cov_hit(CV_WAITDIAL);
  else if (strstr(m, "@ encountered")) cov_hit(CV_AT);
  else if (strstr(m, "$ encountered")) cov_hit(CV_DOLLAR);
  else if (strstr(m, "^ encountered")) cov_hit(CV_CARET);
  else if (strstr(m, "partially ended")) cov_hit(CV_PARTIAL);
  else if (strstr(m, "Dialing string error")) cov_hit(CV_DSTRERR);
  else if (strstr(m, "STATE:")) cov_hit(CV_STATE);
  else if (strstr(m, "BlindCall")) cov_hit(CV_BLIND);
}

static int g_longto;
/* realistic parameters: windows of 160 samples (20 ms), cadence limits in units of 10 ms */
static void set_params(int variant)
{
  memset(g_prm, 0, sizeof g_prm);
  memset(g_sreg, 0, sizeof g_sreg);
  g_prm[0x11] = 20; g_prm[0x12] = 20; g_prm[0x13] = 1; g_prm[0x14] = 7; g_prm[0x15] = rrange(4, 9);
  g_prm[0x16] = 40; g_prm[0x17] = 60; g_prm[0x18] = 3; g_prm[0x19] = 40; g_prm[0x1a] = 60; /* units of 10 ms: a 0.5 s / 0.5 s busy tone */
  g_prm[0x1b] = (long)rmod(4); g_prm[0x1c] = 5; g_prm[0x1d] = 2; g_prm[0x1e] = g_longto ? 40 : rrange(4, 12); g_prm[0x1f] = 2;
  g_prm[0x21] = 1; g_prm[0x22] = rrange(2, 30); g_prm[0x23] = 4; g_prm[0x24] = 6; g_prm[0x25] = 0; g_prm[0x26] = (long)rrange(0, 5); g_prm[0x27] = rrange(30, 45);
  g_prm[0x28] = 0; g_prm[0x29] = 5; g_prm[0x2a] = 1; g_prm[0x2b] = (long)rrange(0, 5); g_prm[0x2c] = 3; g_prm[0x2d] = rrange(1, 5);
  g_prm[0x2e] = 150; g_prm[0x2f] = 250; g_prm[0x30] = 2; g_prm[0x31] = 300; g_prm[0x32] = 500; g_prm[0x33] = (long)rmod(8);
  g_prm[0x34] = 20; g_prm[0x35] = 30; g_prm[0x36] = 4; g_prm[0x37] = 20; g_prm[0x38] = 30; g_prm[0x39] = (long)rmod(8);
  g_prm[0x3a] = 160; g_prm[0x3b] = (long)(variant & 1); g_prm[0x3d] = (long)rmod(9); g_prm[0x3e] = (long)((variant >> 1) & 1);
  g_sreg[0x38] = (variant & 4) ? (int)rmod(2) : 2; /* 0 / 1 / 3: blind dialling */
  g_sreg[0x10] = (variant & 8) ? 0 : 1;            /* 0: the glue prefixes 'p' (pulse dialling), else 't' */
  g_sreg[0xdd] = rrange(-30, -5);
}
static const char *const k_dial[] = {"5551212", "T5551212", "P5551212", "0123456789", "9,5551212", "*#ABCD", "55512@12", "5551212;", "W5", "12345678901234567890123456789012345678901234567890123", "5551212^", "", "T,,,,,,5"};

static void scen0(void)
{
  int t;
  static short out[200];
  short *line = NULL;
  int linecap = 0;
  g_snap = snap;
  for (t = 0; t < 96; t++) {
    Sched sc;
    CallProgCfg cfg;
    int variant = t & 15, total, off = 0, calls = 0;
    int kind = t < 9 ? t : (t % 9 == 6 ? 9 : t % 9);
    if (t >= 80) { kind = 10; variant = 4 | (t & 1) | ((t & 3) << 3); }
    sched_make(&sc, kind, 8000);
    total = sched_total(&sc);
    if (total > linecap) { free(line); line = (short *)malloc((size_t)total * 2 + 64); linecap = total; }
    render(line, &sc, 8000, rpct(30) ? 6000 : 12000);
    g_longto = (t / 3) % 2 == 0;
    set_params(variant);
    g_dialstr = t >= 80 ? "T5@5" : k_dial[rmod(sizeof k_dial / sizeof k_dial[0])];
    cfg.blind = t >= 80 ? 1 : (variant & 4) ? (int)rmod(2) : 0;
    cfg.getSReg = NULL;
    cfg.modem = FAKEM;
    cfg.w0c = 0;
    {
      /* the call glue passes call_GetSRegister; the harness' own callback does the same through modem_get_sreg */
      extern int e2e_sreg_cb(struct modem *, int);
      cfg.getSReg = e2e_sreg_cb;
    }
    dsplibs_debug_level = (t % 2 == 0 || kind == 1) ? 2 : 0;
    g_runh = 1;
    g_cp.state = 0;
    CALLPROG_Create(&g_cp, &cfg);
    CALLPROG_Dial(&g_cp, g_dialstr);
    emit(0, "E2E0 t=%d kind=%d dial=\"%s\" total=%d", t, kind, g_dialstr, total);
    while (off < total) {
      int n = rpct(85) ? 160 : rrange(1, 160), r, i;
      if (off + n > total) n = total - off;
      for (i = 0; i < 200; i++) out[i] = (short)0x4d4d;
      r = CALLPROG_Progress(&g_cp, line + off, out, n);
      if (r >= 0 && r < 31) cov_hit(CV_R0 + r);
      g_runh = fnv_mix(g_runh, (unsigned)r);
      g_runh = fnv_mix(g_runh, (unsigned)(fnv_bytes(out, (size_t)n * 2) & 0xffffffffu));
      emit(0, "P t=%d off=%d n=%d -> %d out=%016llx st=%d", t, off, n, r, fnv_bytes(out, (size_t)n * 2), g_cp.state);
      off += n;
      calls++;
      if (r == 12 && rpct(50)) break;
    }
    CALLPROG_Delete(&g_cp);
    trial_end("e2e0");
  }
  free(line);
  emit(1, "scenario 0 done");
}
int e2e_sreg_cb(struct modem *m, int reg) { return modem_get_sreg(m, (unsigned)(unsigned short)reg); }

static void glue_session(int t, int fs, int kind, int variant, int dbg)
{
  static short *line;
  static int cap;
  Sched sc;
  int total, off = 0, st = 0;
  struct dp *dp;
  static const int chunks[] = {160, 96, 47, 48, 49, 1, 160, 120, 33};
  sched_make(&sc, kind, fs);
  total = sched_total(&sc);
  if (total > cap) {
    free(line);
    line = (short *)malloc((size_t)total * 2 + 64);
    cap = total;
  }
  render(line, &sc, fs, 10000);
  g_longto = (t / 3) % 2 == 0;
  set_params(variant);
  g_dialstr = k_dial[rmod(sizeof k_dial / sizeof k_dial[0])];
  dsplibs_debug_level = dbg ? 2 : 0;
  g_runh = 1;
  dp = g_op->create(FAKEM, DP_CALLPROG, 1, fs, 160, g_op);
  if (!dp) die3("create");
  g_dp = dp;
  g_glue = (CallGlue *)dp;
  g_dpaddr = (long)(intptr_t)dp;
  emit(0, "E2E1 t=%d fs=%d kind=%d variant=%d dial=\"%s\" total=%d", t, fs, kind, variant, g_dialstr, total);
  while (off < total) {
    short out[400];
    int n = rpct(70) ? 160 : chunks[rmod(sizeof chunks / sizeof chunks[0])], r, i;
    if (off + n > total) n = total - off;
    for (i = 0; i < 400; i++) out[i] = (short)0x4d4d;
    r = g_op->process(dp, line + off, out, n);
    g_runh = fnv_mix(g_runh, (unsigned)r);
    g_runh = fnv_mix(g_runh, (unsigned)(fnv_bytes(out, (size_t)n * 2) & 0xffffffffu));
    emit(0, "G t=%d off=%d n=%d -> %d out=%016llx cpst=%d status=%d", t, off, n, r, fnv_bytes(out, (size_t)n * 2), g_glue->cp.state, g_glue->status);
    if (r >= 0 && r < 31) cov_hit(CV_R0 + r);
    if (g_glue->status >= 0 && g_glue->status < 20) cov_hit(CV_ST0 + g_glue->status);
    if (g_glue->ended) cov_hit(CV_ENDED);
    off += n;
    st++;
  }
  emit(0, "E2E1 end t=%d glue.cp.state=%d ended=%d", t, g_glue->cp.state, g_glue->ended);
  g_op->delete(dp);
  g_dp = NULL;
  g_glue = NULL;
  g_dpaddr = 0;
  trial_end("e2e1");
}
static void scen1(void)
{
  static const int fss[3] = {8000, 9600, 48000};
  int t;
  g_snap = snap;
  dp_call_init();
  for (t = 0; t < 60; t++) glue_session(t, fss[t % 3], t < 9 ? t : (t % 9 == 6 ? 9 : t % 9), t & 15, t % 11 == 0);
  emit(1, "scenario 1 done");
}
static void scen2(void)
{
  int t;
  g_snap = snap;
  dp_call_init();
  for (t = 0; t < 40; t++) {
    int variant = t & 15;
    glue_session(t, t & 1 ? 9600 : 8000, (t % 3) == 0 ? 0 : (t % 3) == 1 ? 4 : 5, variant, 1);
  }
  emit(1, "scenario 2 done");
}

/* modem_get_param is shared with r11_common.h, which has no id 0xe: the dp pointer of the pulse helpers is set through g_prm[0xe] before each call (the pulse helpers read it) */
static void cov_setup(int sc)
{
  static char nm[80][48];
  int i;
  if (sc == 0 || sc == 1) {
    cov_def(CV_DIALTONE, "tone detected by cadence dial");
    if (sc == 0) cov_def(CV_BUSY, "busy detected by cadence");
    cov_def(CV_STATE, "STATE: change");
    cov_def(CV_TIMEOUT, "CALLPROG: Time out");
  }
  if (sc == 0) {
    cov_def(CV_2100, "Found 2100");
    cov_def(CV_2250, "Found 2250");
    cov_def(CV_SILENCE5, "5 sec. silence");
    cov_def(CV_BLIND, "blind dial pause");
    for (i = 0; i <= 14; i++) { snprintf(nm[i], sizeof nm[i], "CALLPROG return %d", i); if (i == 0 || i == 4 || i == 10 || i == 12 || i == 0x10 || i == 0x11) cov_def(CV_R0 + i, nm[i]); }
  }
  if (sc == 1) { cov_def(CV_ENDED, "glue ended"); }
}
int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R11_SEED");
  g_verbose = getenv("R11_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)atoi(sd) * 2654435761u;
  g_lcg += (unsigned)sc * 0x9e3779b9u;
  g_dbg_hook = dbg_hook;
  cov_setup(sc);
  switch (sc) {
  case 0: scen0(); break;
  case 1: scen1(); break;
  default: scen2(); break;
  }
  cov_finish();
  return 0;
}
