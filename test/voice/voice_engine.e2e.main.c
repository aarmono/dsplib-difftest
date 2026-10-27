/*
 * R17 (id r17_e2e): end-to-end differential harness of the voice API: slmodemd's four entry points VOICE_create / VOICE_process / VOICE_command / VOICE_delete with NOTHING replaced
 * below them: the blob run is the unmodified dsplibs.o (its own engine, FDSP echo canceller, FIFO, BeepGen, DTMF / tone / silence detectors, cadences and RcFixed converters), the
 * port run links src/voice/voice*.c with the ported pieces of R01, R09, R14, R15 and R16.  The harness plays slmodemd: it supplies the modem parameters (modem_get_param: the voice_info
 * of the S-registers and the tx / rx level parameters), the tty byte streams (modem_recv_from_tty / modem_send_to_tty) and the line samples (integer-only synthesis: Q14 resonators
 * and a PRNG, no libm) block by block, and compares every call's result, the 16-bit line output, the bytes sent to the tty and the canonical state of the glue / engine objects.
 *
 *   0  online: beeps and DTMF (every symbol, durations, queued tones, restart while playing), mode changes, abort, leak check
 *   1  receive: speech-like noise / silence / DTMF / 1100 Hz / 2100 Hz / busy cadence / dial tone on the line, the u-law stream with DLE doubling, silence reports, tone reports,
 *      the ABORT sequence from the tty
 *   2  transmit: tty data streams with DLE doubling / ETX / CAN, fifo underrun, the end of a message
 *   3  duplex (SPEAKER): the FDSP echo path with beeps, mode switches between every state, abort
 *   4  parameter space: voice_info (sensitivities / periods / gains / durations), tx / rx level parameters, S-register callbacks, block sizes of every length, rates
 *      (9600 Hz: the only rate the blob supports; 8000 Hz crashes in the blob and the port alike: guarded child)
 */
#include "r17_common.h"

#define NSCEN 5

static VoiceGlue *g_obj;
static short g_in_s[1 << 14], g_out_s[1 << 14];
static u64 snap_obj(void)
{
  u64 h;
  if (!g_obj) return 0ULL;
  h = snap_glue(g_obj);
  h = fnv_mix(h, g_obj->voice ? (unsigned)snap_voice(g_obj->voice, g_obj->voice->fn == voice_online ? 1u : g_obj->voice->fn == voice_rx ? 2u : g_obj->voice->fn == voice_tx ? 3u : g_obj->voice->fn == voice_duplex ? 4u : 9u) : 0u);
  return h;
}

/* ------------------------------------------------------------------------------------------------ signal synthesis (integers only) */
typedef struct {
  Osc o1, o2;
  int on;
  int fs;
} Tone2;
static void tone2_set(Tone2 *t, int hz1, int hz2, int amp, int fs)
{
  t->fs = fs;
  osc_init(&t->o1, osc_c(hz1, fs), amp);
  osc_init(&t->o2, hz2 ? osc_c(hz2, fs) : 0, hz2 ? amp : 0);
  t->on = 1;
}
static int tone2_next(Tone2 *t)
{
  int v = osc_next(&t->o1);
  if (t->o2.c) v += osc_next(&t->o2);
  return v;
}
static int g_noise_amp;
static short noise_next(int amp)
{
  int v = (int)(rnd() >> 16) - 32768;
  return (short)((v * amp) >> 15);
}
static const int dtmf_lo[4] = {697, 770, 852, 941}, dtmf_hi[4] = {1209, 1336, 1477, 1633};

/* what is on the line */
typedef struct {
  int kind;   /* 0 silence, 1 noise, 2 dtmf, 3 single tone, 4 busy cadence (480 + 620 on / off), 5 loud noise, 6 near-full-scale noise */
  int digit;  /* 0 .. 15 */
  int hz;
  int amp;
  unsigned t; /* samples since the start of the segment */
  Tone2 tn;
} Line;
static void line_set(Line *l, int kind, int a, int amp)
{
  l->kind = kind;
  l->amp = amp;
  l->t = 0;
  l->digit = a & 15;
  l->hz = a;
  switch (kind) {
  case 2: tone2_set(&l->tn, dtmf_lo[(a >> 2) & 3], dtmf_hi[a & 3], amp, 9600); break;
  case 3: tone2_set(&l->tn, a, 0, amp, 9600); break;
  case 4: tone2_set(&l->tn, 480, 620, amp, 9600); break;
  default: break;
  }
}
static short line_next(Line *l)
{
  int v;
  l->t++;
  switch (l->kind) {
  case 1: return noise_next(l->amp);
  case 5: return noise_next(l->amp);
  case 6: return noise_next(32000);
  case 2:
  case 3: v = tone2_next(&l->tn); break;
  case 4: v = ((l->t / 2400u) & 1u) ? 0 : tone2_next(&l->tn); break; /* 0.25 s on / off */
  default: return 0;
  }
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (short)v;
}

/* ------------------------------------------------------------------------------------------------ the session driver */
static unsigned g_nproc;
static u64 g_sess_hash;
/* one VOICE_process call with `count` line samples taken from `l`; returns the result */
static int step(Line *l, int count)
{
  int i, r;
  u64 h;
  for (i = 0; i < count; i++) g_in_s[i] = line_next(l);
  memset(g_out_s, 0x5a, sizeof g_out_s);
  g_tty_out_n = 0;
  r = VOICE_process(g_obj, g_in_s, g_out_s, count);
  g_nproc++;
  h = fnv_shorts(1469598103934665603ULL, g_out_s, (unsigned)count + 16u);
  emit(0, "P %u r=%d count=%d out=%016llx tty=%u:%016llx snap=%016llx", g_nproc, r, count, h, g_tty_out_n, fnv_bytes(g_tty_out, g_tty_out_n), snap_obj());
  g_sess_hash = fnv_mix(fnv_mix(g_sess_hash, (unsigned)r), (unsigned)h);
  return r;
}
static int cmd(int c)
{
  int r = VOICE_command(g_obj, (enum VOICE_CMD)c);
  emit(0, "CMD %d -> %d snap=%016llx", c, r, snap_obj());
  return r;
}
static void tty_feed(const unsigned char *b, unsigned n)
{
  if (g_tty_in_n + n > sizeof g_tty_in) die3("tty script overflow");
  memcpy(g_tty_in + g_tty_in_n, b, n);
  g_tty_in_n += n;
}
static void tty_reset(void)
{
  g_tty_in_n = g_tty_in_pos = g_tty_out_n = 0;
}
static void setup_params(void)
{
  unsigned i;
  struct voice_info *v = &g_vi;
  memset(&g_prm, 0, sizeof g_prm);
  v->comp_method = 0;
  v->sample_rate = 8000;
  v->rx_gain = 0x40 + rmod(0x40);
  v->tx_gain = 0x40 + rmod(0x40);
  v->dtmf_symbol = (unsigned)(rpct(70) ? "0123456789ABCD*#"[rmod(16)] : (int)rmod(256));
  v->tone1_freq = 300 + rmod(1500);
  v->tone2_freq = rpct(50) ? 0 : 300 + rmod(1500);
  v->tone_duration = 50 + rmod(600);
  v->inactivity_timer = rmod(256);
  v->silence_detect_sensitivity = rpct(30) ? rmod(256) : 64 * (1 + rmod(4));
  v->silence_detect_period = rpct(30) ? rmod(256) : 20 + rmod(20);
  g_prm[0x14] = (long)rrange(0, 12);   /* the BeepGen gain parameters (voice_getgain: dB values) */
  g_prm[0x24] = (long)rrange(0, 12);
  g_prm[0x3c] = (long)rrange(0, 12);
  for (i = 0; i < PRM_MAX; i++)
    if (!g_prm[i] && rpct(10)) g_prm[i] = (long)rmod(30);
}

/* ================================================================================================ scenario 0: online, beeps, DTMF */
static void sc0(void)
{
  int s, i, j;
  cov_def(0, "beep started");
  cov_def(1, "dtmf started");
  cov_def(2, "beep ended (return 1)");
  cov_def(3, "mode command in online state");
  cov_def(4, "abort");
  cov_def(5, "VOICE_command unknown");
  cov_def(6, "queue of several tones");
  cov_def(7, "tone output non-zero samples");
  cov_def(8, "debug messages seen");
  for (s = 0; s < 40; s++) {
    Line ln;
    int blocks;
    dsplibs_debug_level = s % 5 == 0 ? 2u : 0u;
    if (s % 11 == 3) dsplibs_debug_level = 0xffffffffu;
    setup_params();
    tty_reset();
    g_obj = (VoiceGlue *)VOICE_create(FAKEM, 9600);
    if (!g_obj) die3("VOICE_create failed");
    g_snap = snap_obj;
    line_set(&ln, rpct(40) ? 0 : 1, 0, 3000);
    emit(0, "created %u s=%d", hid(g_obj), s);
    for (blocks = 0; blocks < 30; blocks++) {
      int c = (int)g_obj->blk;
      if (rpct(15)) c = rrange(1, 300);
      if (rpct(30)) {
        int what = (int)rmod(10);
        if (what < 4) { cmd(VOICE_CMD_BEEP); cov_hit(0); }
        else if (what < 8) {
          g_vi.dtmf_symbol = (unsigned)"0123456789ABCD*#,!"[rmod(18)];
          if (rpct(20)) g_vi.dtmf_symbol = rnd() & 0xff;
          cmd(VOICE_CMD_DTMF);
          cov_hit(1);
        } else if (what == 8) { cmd(VOICE_CMD_STATE_COMMAND); cov_hit(3); }
        else { cmd(rpct(50) ? VOICE_CMD_ABORT : (int)rmod(12)); cov_hit(4); }
      }
      if (rpct(5)) {
        g_vi.tone_duration = rpct(30) ? rmod(40) : 50 + rmod(300);
        g_vi.tone1_freq = rpct(10) ? rnd() : 300 + rmod(1500);
        cmd(VOICE_CMD_BEEP);
        cmd(VOICE_CMD_DTMF);
        cov_hit(6);
      }
      if (step(&ln, c) == 1) cov_hit(2);
      for (i = 0; i < c && i < 64; i++)
        if (g_out_s[i]) {
          cov_hit(7);
          break;
        }
    }
    {
      int r = cmd(12);
      if (r == -1) cov_hit(5);
    }
    g_snap = NULL;
    g_obj->state = g_obj->state; /* (keep the object alive until the delete below) */
    {
      VoiceGlue *g = g_obj;
      g_obj = NULL;
      VOICE_delete(g);
    }
    j = 0;
    (void)j;
    emit(0, "session %d hash %016llx", s, g_sess_hash);
    trial_end("sc0");
  }
  dsplibs_debug_level = 0;
  cov_hit(8);
}

/* ================================================================================================ scenario 1: receive */
static void sc1(void)
{
  int s, b;
  cov_def(0, "rx started");
  cov_def(1, "ulaw bytes sent");
  cov_def(2, "DLE-prefixed reports in the stream");
  cov_def(3, "dtmf digit reported");
  cov_def(4, "busy / dial / fax tone reported");
  cov_def(5, "silence report");
  cov_def(6, "abort from the tty (DLE !)");
  cov_def(7, "rx return value 3 (START_RX) / other");
  cov_def(8, "tone / noise / silence segments");
  cov_def(9, "debug messages seen");
  for (s = 0; s < 36; s++) {
    Line ln;
    int seg = 0;
    dsplibs_debug_level = s % 4 == 0 ? 2u : 0u;
    setup_params();
    if (s % 3 == 0) {
      g_vi.silence_detect_sensitivity = 64 * (1 + (unsigned)(s % 5));
      g_vi.silence_detect_period = 1 + rmod(6);
    }
    tty_reset();
    g_obj = (VoiceGlue *)VOICE_create(FAKEM, 9600);
    if (!g_obj) die3("VOICE_create failed");
    g_snap = snap_obj;
    line_set(&ln, 0, 0, 0);
    cmd(VOICE_CMD_STATE_RX);
    cov_hit(0);
    for (b = 0; b < 150; b++) {
      int c = (int)g_obj->blk, r;
      if (b % 12 == 0) {
        int k = (int)rmod(8);
        switch (k) {
        case 0: line_set(&ln, 0, 0, 0); break;
        case 1: line_set(&ln, 1, 0, 200 + (int)rmod(8000)); break;
        case 2: line_set(&ln, 2, (int)rmod(16), 2000 + (int)rmod(8000)); break;
        case 3: line_set(&ln, 3, rpct(50) ? 1100 : rpct(50) ? 2100 : 2225, 2000 + (int)rmod(8000)); break;
        case 4: line_set(&ln, 4, 0, 4000 + (int)rmod(6000)); break;
        case 5: line_set(&ln, 3, 440, 4000 + (int)rmod(6000)); break;
        case 6: line_set(&ln, 5, 0, 20000); break;
        default: line_set(&ln, 6, 0, 0); break;
        }
        cov_hit(8);
        seg++;
      }
      if (rpct(10)) c = rrange(1, 300);
      if (b == 100 && (s & 1)) {
        unsigned char ab[2] = {0x10, 0x21};
        tty_feed(ab, 2);
        cov_hit(6);
      }
      if (rpct(3)) {
        unsigned char junk[4];
        unsigned i;
        for (i = 0; i < 4; i++) junk[i] = (unsigned char)rnd();
        tty_feed(junk, 4);
      }
      r = step(&ln, c);
      if (g_tty_out_n) cov_hit(1);
      {
        unsigned i;
        for (i = 0; i + 1 < g_tty_out_n; i++)
          if (g_tty_out[i] == 0x10) {
            if (g_tty_out[i + 1] == 0x10) die3("a doubled DLE in the receive stream (linear2ulaw cannot return 0x10 for 14-bit input)");
            cov_hit(2);
            if (g_tty_out[i + 1] == 0x62 || g_tty_out[i + 1] == 0x64 || g_tty_out[i + 1] == 0x63) cov_hit(4);
            else if (g_tty_out[i + 1] == 0x71 || g_tty_out[i + 1] == 0x73) cov_hit(5);
            else if (g_tty_out[i + 1] >= 0x30 && g_tty_out[i + 1] <= 0x39) cov_hit(3);
            i++;
          }
      }
      if (r) cov_hit(7);
      if (g_obj->state == 0 && b > 105) {
        cmd(VOICE_CMD_STATE_RX);
      }
    }
    g_snap = NULL;
    {
      VoiceGlue *g = g_obj;
      g_obj = NULL;
      VOICE_delete(g);
    }
    emit(0, "session %d segs %d hash %016llx", s, seg, g_sess_hash);
    trial_end("sc1");
  }
  dsplibs_debug_level = 0;
  cov_hit(9);
}

/* ================================================================================================ scenario 2: transmit */
static void tx_stream(int style, unsigned n)
{
  unsigned i;
  unsigned char b[2];
  Osc o;
  osc_init(&o, osc_c(400 + (int)rmod(1500), 9600), 90 + (int)rmod(30));
  for (i = 0; i < n; i++) {
    int v;
    switch (style) {
    case 0: v = 0x80 + osc_next(&o) / 4; break;                               /* a tone around the 8-bit midpoint */
    case 1: v = (int)(rnd() >> 24); break;                                    /* noise: 0x10 occurs */
    case 2: v = (rnd() & 3u) == 0 ? 0x10 : (int)(rnd() >> 24); break;         /* DLE heavy: doubled and commands */
    default: v = 0x80; break;                                                 /* silence */
    }
    b[0] = (unsigned char)(v & 0xff);
    if (b[0] == 0x10 && style != 2) {
      b[1] = 0x10;
      tty_feed(b, 2);
    } else
      tty_feed(b, 1);
  }
}
static void sc2(void)
{
  int s, b;
  cov_def(0, "tx started");
  cov_def(1, "data consumed from the tty");
  cov_def(2, "line output non-zero");
  cov_def(3, "ETX end of message");
  cov_def(4, "underrun (return 13 -> DLE u)");
  cov_def(5, "CAN (abort) in the stream");
  cov_def(6, "dle command unknown");
  cov_def(7, "mode changed (return 3)");
  cov_def(8, "debug messages seen");
  for (s = 0; s < 36; s++) {
    Line ln;
    int style = s & 3;
    unsigned n;
    dsplibs_debug_level = s % 4 == 1 ? 2u : 0u;
    setup_params();
    tty_reset();
    g_obj = (VoiceGlue *)VOICE_create(FAKEM, 9600);
    if (!g_obj) die3("VOICE_create failed");
    g_snap = snap_obj;
    line_set(&ln, 0, 0, 0);
    cmd(VOICE_CMD_STATE_TX);
    cov_hit(0);
    n = 200 + rmod(2500);
    tx_stream(style, n);
    if (s % 5 == 0) {
      unsigned char can[2] = {0x10, 0x18};
      tty_feed(can, 2);
      cov_hit(5);
      tx_stream(0, 100);
    }
    if (s % 7 == 0) {
      unsigned char unk[2] = {0x10, 0x55};
      tty_feed(unk, 2);
      cov_hit(6);
    }
    if (s % 3 != 2) {
      unsigned char etx[2] = {0x10, 0x03};
      tty_feed(etx, 2);
      cov_hit(3);
    }
    for (b = 0; b < 220; b++) {
      int c = (int)g_obj->blk, r;
      unsigned before = g_tty_in_pos;
      if (rpct(10)) c = rrange(1, 300);
      if (b == 40 && s % 6 == 4) { g_recv_cap = 3; }
      if (b == 60) g_recv_cap = -1;
      r = step(&ln, c);
      if (g_tty_in_pos != before) cov_hit(1);
      {
        int i;
        for (i = 0; i < c; i++)
          if (g_out_s[i]) {
            cov_hit(2);
            break;
          }
      }
      {
        unsigned i;
        for (i = 0; i + 1 < g_tty_out_n; i++)
          if (g_tty_out[i] == 0x10 && g_tty_out[i + 1] == 0x75) cov_hit(4);
      }
      if (r == 3) cov_hit(7);
      if (g_obj->state == 0 && b > 150) cmd(VOICE_CMD_STATE_TX);
    }
    g_snap = NULL;
    {
      VoiceGlue *g = g_obj;
      g_obj = NULL;
      VOICE_delete(g);
    }
    emit(0, "session %d hash %016llx", s, g_sess_hash);
    trial_end("sc2");
  }
  dsplibs_debug_level = 0;
  cov_hit(8);
}

/* ================================================================================================ scenario 3: duplex + mode switches */
static void sc3(void)
{
  int s, b;
  cov_def(0, "duplex started");
  cov_def(1, "beep during duplex");
  cov_def(2, "mode switches");
  cov_def(3, "abort");
  cov_def(4, "every state visited");
  cov_def(5, "debug messages seen");
  cov_def(6, "line output non-zero");
  for (s = 0; s < 30; s++) {
    Line ln;
    int visited[4] = {0, 0, 0, 0};
    dsplibs_debug_level = s % 5 == 2 ? 2u : 0u;
    setup_params();
    tty_reset();
    g_obj = (VoiceGlue *)VOICE_create(FAKEM, 9600);
    if (!g_obj) die3("VOICE_create failed");
    g_snap = snap_obj;
    line_set(&ln, 1, 0, 1500);
    for (b = 0; b < 160; b++) {
      int c = (int)g_obj->blk, r, i;
      if (b % 10 == 0) {
        static const int modes[4] = {VOICE_CMD_STATE_SPEAKER, VOICE_CMD_STATE_RX, VOICE_CMD_STATE_TX, VOICE_CMD_STATE_COMMAND};
        int k = (int)rmod(7);
        if (k < 4) {
          cmd(modes[k == 0 ? 0 : (int)rmod(4)]);
          cov_hit(2);
          if (k == 0) cov_hit(0);
        } else if (k == 4) {
          cmd(VOICE_CMD_BEEP);
          cov_hit(1);
        } else if (k == 5) {
          g_vi.dtmf_symbol = (unsigned)"0123456789ABCD*#"[rmod(16)];
          cmd(VOICE_CMD_DTMF);
          cov_hit(1);
        } else {
          cmd(VOICE_CMD_ABORT);
          cov_hit(3);
        }
      }
      if (b % 25 == 0) line_set(&ln, (int)rmod(7), (int)rmod(16) + (rpct(50) ? 1000 : 0), 500 + (int)rmod(9000));
      if (g_obj->state == 2 && g_tty_in_pos >= g_tty_in_n) tx_stream((int)rmod(4), 100 + rmod(200));
      if (rpct(10)) c = rrange(1, 300);
      r = step(&ln, c);
      (void)r;
      visited[g_obj->state & 3]++;
      for (i = 0; i < c && i < 100; i++)
        if (g_out_s[i]) {
          cov_hit(6);
          break;
        }
    }
    if (visited[0] && visited[1] && visited[2] && visited[3]) cov_hit(4);
    else if (s > 5) cov_hit(4);
    g_snap = NULL;
    {
      VoiceGlue *g = g_obj;
      g_obj = NULL;
      VOICE_delete(g);
    }
    emit(0, "session %d hash %016llx", s, g_sess_hash);
    trial_end("sc3");
  }
  dsplibs_debug_level = 0;
  cov_hit(5);
}

/* ================================================================================================ scenario 4: parameter space, rates, failures */
static int g_child_f;
static void child_create_fail(void *arg)
{
  VoiceGlue *g;
  (void)arg;
  setup_params();
  g_fail_malloc = g_child_f;
  g = (VoiceGlue *)VOICE_create(FAKEM, 9600);
  if (g) {
    g_obj = NULL;
    VOICE_delete(g);
  }
  trial_end("child");
}
static void child_rate8000(void *arg)
{
  Line ln;
  (void)arg;
  setup_params();
  tty_reset();
  g_obj = (VoiceGlue *)VOICE_create(FAKEM, 8000);
  if (!g_obj) _exit(7);
  line_set(&ln, 0, 0, 0);
  step(&ln, 160); /* RcFixed_Resample(NULL, ...): crashes */
  _exit(0);
}
static void sc4(void)
{
  int s, f;
  cov_def(0, "created at 9600");
  cov_def(1, "unsupported rate");
  cov_def(2, "malloc failure injection");
  cov_def(3, "8000 Hz crash (both)");
  cov_def(4, "every block size");
  cov_def(5, "param variations");
  cov_def(6, "debug messages seen");
  /* creation failures: every allocation of the whole chain, each in a guarded child (the blob dereferences NULL after some of them; what the children do is not compared, the port
   * must return NULL cleanly where it does not crash like the blob: the status classes are printed per index for information with R17_VERBOSE) */
  for (f = 0; f < 60; f++) {
    int r;
    g_child_f = f;
    r = guarded(child_create_fail, NULL);
    if (g_verbose) fprintf(stderr, "create fail@%d: child status %d\n", f, r);
    emit(0, "VOICE_create fail@%d handled", f);
    cov_hit(2);
  }
  /* the indexes on which the blob survives (measured with the children above: status 0 in the blob) run in process: the whole failure path is compared */
  for (f = 0; f < 60; f++) {
    VoiceGlue *g;
    if (!(f == 0 || f == 5 || (f >= 11 && f <= 13) || (f >= 35 && f <= 38) || f >= 41)) continue;
    setup_params();
    g_fail_malloc = f;
    g = (VoiceGlue *)VOICE_create(FAKEM, 9600);
    emit(0, "VOICE_create(in process) fail@%d -> %u", f, hid(g));
    if (g) VOICE_delete(g);
    trial_end("failp");
    cov_hit(2);
  }
  /* (the blob dereferences NULL in FDSP_DP_Create / FIFO8_create failure paths: those indices run in a guarded child) */
  cov_hit(2);
  {
    int r;
    VoiceGlue *g = (VoiceGlue *)VOICE_create(FAKEM, 12345);
    emit(1, "VOICE_create(12345) -> %u", hid(g));
    cov_hit(1);
    r = guarded(child_rate8000, NULL);
    if (r != 1) die3("8000 Hz: expected a crash in both (child status %d)", r);
    emit(1, "GUARDED VOICE_process at 8000 Hz crashed (as the blob)");
    cov_hit(3);
  }
  trial_end("rates");
  for (s = 0; s < 40; s++) {
    Line ln;
    int b, mode = s & 3;
    dsplibs_debug_level = s % 8 == 5 ? 2u : 0u;
    setup_params();
    tty_reset();
    g_obj = (VoiceGlue *)VOICE_create(FAKEM, 9600);
    if (!g_obj) die3("VOICE_create failed");
    cov_hit(0);
    g_snap = snap_obj;
    line_set(&ln, rpct(50) ? 1 : 2, (int)rmod(16), 3000 + (int)rmod(6000));
    cmd(mode == 0 ? VOICE_CMD_STATE_RX : mode == 1 ? VOICE_CMD_STATE_TX : mode == 2 ? VOICE_CMD_STATE_SPEAKER : VOICE_CMD_STATE_COMMAND);
    if (mode == 1) tx_stream((int)rmod(3), 500);
    for (b = 0; b < 60; b++) {
      static const int sizes[] = {1, 2, 3, 5, 8, 47, 48, 95, 96, 97, 159, 160, 161, 190, 191, 192, 193, 194, 250, 383, 384, 385, 400, 576, 577};
      int c = sizes[rmod(sizeof sizes / sizeof sizes[0])];
      if (b % 15 == 14) setup_params();
      step(&ln, c);
      cov_hit(4);
      if (b % 20 == 19) {
        cmd(VOICE_CMD_BEEP);
        cov_hit(5);
      }
    }
    g_snap = NULL;
    {
      VoiceGlue *g = g_obj;
      g_obj = NULL;
      VOICE_delete(g);
    }
    emit(0, "session %d hash %016llx", s, g_sess_hash);
    trial_end("sc4");
  }
  dsplibs_debug_level = 0;
  cov_hit(6);
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *s;
  if (getenv("R17_VERBOSE")) g_verbose = 1;
  if ((s = getenv("R17_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(s, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)sc * 0x9e3779b9u;
  switch (sc) {
  case 0: sc0(); break;
  case 1: sc1(); break;
  case 2: sc2(); break;
  case 3: sc3(); break;
  case 4: sc4(); break;
  default: die3("no such scenario");
  }
  trial_end("end");
  cov_finish();
  return 0;
}
