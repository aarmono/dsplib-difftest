/*
 * R17 (id r17_glue): differential harness of the voice glue slmodemd links: VOICE_create / VOICE_delete / VOICE_command / VOICE_process (and the LOCAL callbacks vce_get_sreg /
 * vce_hook_on / vce_hook_off, reached through the VoiceCfg the glue hands to voice_create) of the real dsplibs.o against src/voice/voice_engine.c.  The engine (voice_create / delete / command /
 * modem), the resamplers (RcFixed_Create / Delete / Resample) and everything below are SCRIPTED RECORDERS defined here (WEAKENED in a copy of dsplibs.o so they win in the blob run);
 * modem_get_param (the harness' voice_info), modem_recv_from_tty / modem_send_to_tty (scripted byte queues), sysdep_* and dsplibs_debug_printf are the harness' too.
 *
 *   0  VOICE_create / VOICE_delete: rates, converter / engine / malloc failures, debug levels, the three callbacks the glue builds (every id of vce_get_sreg against random voice_info
 *      values), NULL object
 *   1  VOICE_command: every command x voice_info x engine result, NULL object, unknown commands
 *   2  VOICE_process: every engine message, the state machine (command / rx / tx / duplex), tty reads and writes, ring arithmetic with every chunk size, converter results, mutating
 *      debug hooks (the blob re-reads some glue words after a message, caches others)
 */
#include "r17_common.h"

#define NSCEN 3

/* ------------------------------------------------------------------------------------------------ scripted components */
static u64 hash_floats(const float *p, unsigned n) { return fnv_floats(1469598103934665603ULL, p, n); }
static VoiceGlue *g_glue;          /* the glue block under test (the recorders need its buffers to tell the converters apart) */
static int g_vc_ret;               /* voice_command's scripted result */
static int g_vcreate_fail;         /* voice_create returns NULL */
static int g_probe_cb;             /* voice_create probes the callbacks of the cfg */
static int g_rc_fail_at = -1;      /* RcFixed_Create: the n-th call returns NULL */
static int g_rc_calls;
static int g_rc_n_in = -1;         /* RcFixed_Resample (input converter): samples produced (< 0: min(nIn, cap)) */
static unsigned long g_ncmd, g_nvm;
static unsigned g_probe_ids[] = {0x18, 0x48, 0x49, 0x52, 0x53, 0x8a, 0x8b, 0x00, 0x17, 0x19, 0x47, 0x4a, 0x51, 0x54, 0x89, 0x8c, 0xff, 0x100, 0x152, 0x18c};

Voice *voice_create(const VoiceCfg *cfg)
{
  Voice *v;
  unsigned i;
  emit(0, "voice_create ctx=m%d", MID(cfg->ctx));
  if (g_probe_cb) {
    cfg->hook_on(cfg->ctx);
    cfg->hook_off(cfg->ctx);
    for (i = 0; i < sizeof g_probe_ids / sizeof g_probe_ids[0]; i++) {
      unsigned r = cfg->get_sreg(cfg->ctx, (int)g_probe_ids[i]);
      emit(0, "get_sreg %u -> %u", g_probe_ids[i], r);
    }
  }
  if (g_vcreate_fail) return NULL;
  v = (Voice *)sysdep_malloc((unsigned)sizeof(Voice));
  return v;
}
void voice_delete(Voice *v)
{
  emit(0, "voice_delete %u", hid(v));
  sysdep_free(v);
}
int voice_command(Voice *v, int cmd, const int *args)
{
  g_ncmd++;
  switch (cmd) {
  case 0: emit(0, "voice_command %u %d %d %d %d", hid(v), cmd, args[0], args[1], args[2]); break;
  case 1: emit(0, "voice_command %u %d %d %d", hid(v), cmd, args[0], args[1]); break;
  case 9: emit(0, "voice_command %u %d", hid(v), cmd); break;
  default: emit(0, "voice_command %u %d %d", hid(v), cmd, args[0]); break;
  }
  return g_vc_ret;
}

/* voice_modem: scripted per call from the queue g_vm_*; writes the output buffers with deterministic patterns */
static int g_vm_ret;           /* the message */
static unsigned g_vm_a5, g_vm_a6; /* the counts it reports */
static int g_vm_mutate;        /* 1: the next call also pokes the glue (state / cnt / last) */
static int g_vm_fout_mode;     /* 0 normal samples, 1 overflow, 2 specials */
int voice_modem(Voice *v, void *a1, float *a2, float *a3, void *a4, unsigned short *a5, unsigned short *a6)
{
  unsigned i;
  g_nvm++;
  emit(0, "voice_modem %u a1=%u a2=%u a3=%u a4=%u a5=%u a6=%u tty=%016llx fin=%016llx", hid(v), (unsigned)(a1 == (void *)g_glue->tty), (unsigned)(a2 == g_glue->fout),
       (unsigned)(a3 == g_glue->fin), (unsigned)(a4 == (void *)g_glue->outb), (unsigned)*a5, (unsigned)*a6, fnv_bytes(a1, 0x400), fnv_floats(1469598103934665603ULL, a3, 160));
  *a5 = (unsigned short)g_vm_a5;
  *a6 = (unsigned short)g_vm_a6;
  for (i = 0; i < 160; i++) {
    uint32_t u;
    switch (g_vm_fout_mode) {
    case 0: u = 0x38800000u + ((i * 40503u + (unsigned)g_nvm * 977u) & 0x7fffffu) % 0x07000000u + ((i & 1u) << 31); break;
    case 1: u = 0x3f800000u + (i * 12345u) % 0x01000000u + ((i & 2u) << 30); break;
    default: u = (i % 5 == 0) ? 0x7fc00000u : (i % 5 == 1) ? 0xff800000u : (i % 5 == 2) ? 0x00000001u : (i % 5 == 3) ? 0x80000000u : 0x3f000000u; break;
    }
    memcpy(&a2[i], &u, 4);
  }
  for (i = 0; i < (unsigned)(unsigned short)g_vm_a6 && i < 0x400; i++) ((unsigned char *)a4)[i] = (unsigned char)(0x20u + i * 7u + (unsigned)g_nvm);
  if (g_vm_mutate) {
    g_glue->state = (int)rmod(5);
    g_glue->cnt = (int)(rpct(50) ? rmod(300) : rnd());
    g_glue->last = (int)(rpct(30) ? -1 : (int)rmod(15));
  }
  return g_vm_ret;
}

RcFixed *RcFixed_Create(unsigned int idx)
{
  int n = g_rc_calls++;
  if (g_rc_fail_at == n) {
    emit(0, "RcFixed_Create %u -> NULL", idx);
    return NULL;
  }
  emit(0, "RcFixed_Create %u", idx);
  return (RcFixed *)sysdep_malloc((unsigned)sizeof(RcFixed));
}
void RcFixed_Delete(RcFixed *o)
{
  emit(0, "RcFixed_Delete %u", hid(o));
  sysdep_free(o);
}
void RcFixed_Resample(RcFixed *o, const short *in, unsigned int nIn, short *out, unsigned int *pnOut)
{
  unsigned cap = *pnOut, m, i;
  int to_engine = (out == g_glue->sbuf);
  emit(0, "RcFixed_Resample o=%u dir=%s nIn=%u cap=%u in=%016llx", hid(o), to_engine ? "in" : "out", nIn, cap, fnv_shorts(1469598103934665603ULL, in, nIn > 0x400 ? 0x400 : nIn));
  if (to_engine) {
    m = g_rc_n_in < 0 ? (nIn < cap ? nIn : cap) : (unsigned)g_rc_n_in;
    if (m > 0xa0) m = 0xa0;
    for (i = 0; i < m; i++) out[i] = (short)(i < nIn ? in[i] : 0x1234);
  } else {
    m = cap;
    for (i = 0; i < m; i++) out[i] = (short)(nIn ? in[i % nIn] + (short)i : (short)(i * 3));
  }
  *pnOut = m;
}


#ifndef TEST_BLOB
/* port link: the engine functions of src/voice/voice_engine.c (not reached by this id: voice_create / delete / command / modem are the recorders above) call the components */
#define NOTREACHED(n) die3("%s reached", n)
BeepGen *beepgen_create(BeepGen *o, const BeepCfg *c) { (void)o; (void)c; NOTREACHED("beepgen_create"); }
void beepgen_delete(BeepGen *b) { (void)b; NOTREACHED("beepgen_delete"); }
void beepgen_start_beep(BeepGen *b, int f1, int f2, int d) { (void)b; (void)f1; (void)f2; (void)d; NOTREACHED("beepgen_start_beep"); }
void beepgen_start_dtmf(BeepGen *b, int c, int d) { (void)b; (void)c; (void)d; NOTREACHED("beepgen_start_dtmf"); }
int beepgen_sample(BeepGen *b, float *o) { (void)b; (void)o; NOTREACHED("beepgen_sample"); }
Detector *detector_create(Detector *o, struct modem *c, VoiceParamFn f) { (void)o; (void)c; (void)f; NOTREACHED("detector_create"); }
void detector_delete(Detector *d) { (void)d; NOTREACHED("detector_delete"); }
int detector_progress(Detector *d, const float *x, short n, unsigned char *b, unsigned short *l) { (void)d; (void)x; (void)n; (void)b; (void)l; NOTREACHED("detector_progress"); }
void detector_set_enable(Detector *d, unsigned short v) { (void)d; (void)v; NOTREACHED("detector_set_enable"); }
void detector_set_output_status(Detector *d) { (void)d; NOTREACHED("detector_set_output_status"); }
void detector_set_output_in_stream(Detector *d) { (void)d; NOTREACHED("detector_set_output_in_stream"); }
Silence *silence_create(Silence *o, struct modem *c, VoiceParamFn f) { (void)o; (void)c; (void)f; NOTREACHED("silence_create"); }
void silence_delete(Silence *s) { (void)s; NOTREACHED("silence_delete"); }
void silence_progress(Silence *s, const float *x, short n, unsigned char *b, unsigned short *l) { (void)s; (void)x; (void)n; (void)b; (void)l; NOTREACHED("silence_progress"); }
int silence_is_more_then(Silence *s, float v) { (void)s; (void)v; NOTREACHED("silence_is_more_then"); }
void _status(unsigned char *b, unsigned short *l, int c) { (void)b; (void)l; (void)c; NOTREACHED("_status"); }
FdspObj *FDSP_DP_Create(FdspObj *o, short r, short t) { (void)o; (void)r; (void)t; NOTREACHED("FDSP_DP_Create"); }
void FDSP_DP_Delete(FdspObj *f) { (void)f; NOTREACHED("FDSP_DP_Delete"); }
int FDSP_DP_Run(FdspObj *f, const short *a, float *b, const float *c, short *d, void *e, const unsigned short *n) { (void)f; (void)a; (void)b; (void)c; (void)d; (void)e; (void)n; NOTREACHED("FDSP_DP_Run"); }
void FDSP_Kernel_SetInternalBeepInProgress(int on) { (void)on; NOTREACHED("SetInternalBeepInProgress"); }
void STRM_VCE_GetFDSPEnvironmentalParams(short *r, short *t) { (void)r; (void)t; NOTREACHED("STRM_VCE_GetFDSPEnvironmentalParams"); }
FifoObj *FIFO8_create(FifoObj *f, const FifoCfg *c) { (void)f; (void)c; NOTREACHED("FIFO8_create"); }
void FIFO8_delete(FifoObj *f) { (void)f; NOTREACHED("FIFO8_delete"); }
int FIFO8_write(FifoObj *f, const unsigned char *s, unsigned short n) { (void)f; (void)s; (void)n; NOTREACHED("FIFO8_write"); }
int FIFO8_read(FifoObj *f, unsigned char *d, unsigned short n) { (void)f; (void)d; (void)n; NOTREACHED("FIFO8_read"); }
#endif

/* ------------------------------------------------------------------------------------------------ helpers */
static unsigned g_gone;
static u64 snap_cur(void) { return g_glue ? snap_glue(g_glue) : 0ULL; }
static void script_reset(void)
{
  dsplibs_debug_level = g_lvl_base;
  g_vc_ret = 0;
  g_vcreate_fail = 0;
  g_probe_cb = 0;
  g_rc_fail_at = -1;
  g_rc_calls = 0;
  g_rc_n_in = -1;
  g_vm_ret = 0;
  g_vm_a5 = 0;
  g_vm_a6 = 0;
  g_vm_mutate = 0;
  g_vm_fout_mode = 0;
  g_tty_in_n = g_tty_in_pos = g_tty_out_n = 0;
  g_recv_cap = -1;
  g_recv_hook = NULL;
  g_dbg_hook = NULL;
}
static void rnd_vi(void)
{
  struct voice_info *p = &g_vi;
  unsigned *w = (unsigned *)p;
  unsigned i;
  for (i = 0; i < sizeof *p / sizeof(unsigned); i++) {
    unsigned r = rnd();
    w[i] = rpct(30) ? (rnd() & 0xff) : rpct(20) ? 0xffffffffu : rpct(30) ? (rnd() & 0x3ff) : r;
  }
}
static void show_glue(const char *what, const VoiceGlue *g) { emit(0, "%s glue=%016llx", what, snap_glue(g)); }
static VoiceGlue *mk_glue(unsigned srate)
{
  VoiceGlue *g = (VoiceGlue *)VOICE_create(FAKEM, srate);
  if (!g) die3("VOICE_create(%u) failed", srate);
  g_glue = g;
  return g;
}
static void kill_glue(VoiceGlue *g)
{
  g_glue = NULL;
  VOICE_delete(g);
  if (g_nblk > 2500 && g_live == 0) trial_end("auto");
}

static short g_in_s[1 << 14], g_out_s[1 << 14];
static void child_delete_null(void *arg)
{
  (void)arg;
  VOICE_delete(NULL);
}
static void guarded_crash(const char *what, int r)
{
  if (g_verbose) fprintf(stderr, "guarded %s: child status %d\n", what, r);
  if (r != 1) die3("%s: expected a crash in both (child status %d)", what, r);
  emit(1, "GUARDED %s crashed (as the blob)", what);
}
static void recv_mut_hook(void)
{
  if (!g_glue) return;
  switch (rmod(4)) {
  case 0: g_glue->state = (int)rmod(5); break;
  case 1: g_glue->cnt = (int)(rpct(50) ? rmod(300) : rnd()); break;
  case 2: g_glue->last = (int)rmod(15); break;
  default: break;
  }
}
static unsigned long g_hook_n;
static void mut_hook(const char *msg)
{
  (void)msg;
  g_hook_n++;
  if (!g_glue) return;
  switch (rmod(5)) {
  case 0: g_glue->state = (int)rmod(5); break;
  case 1: g_glue->cnt = (int)(rpct(50) ? rmod(300) : rnd()); break;
  case 2: g_glue->last = (int)rmod(15); break;
  case 3: dsplibs_debug_level = rpct(50) ? 0 : 1; break;
  default: break;
  }
}

/* ================================================================================================ scenario 0: VOICE_create / VOICE_delete */
static void sc0(void)
{
  static const unsigned rates[] = {8000, 9600, 48000, 0, 1, 7999, 8001, 9601, 44100, 0xffffffffu, 4800000u, 26843546u, 48001, 16000, 12000};
  unsigned ri;
  int t, f, rf, vf;
  cov_def(0, "VOICE_create 8000 (no converters)");
  cov_def(1, "VOICE_create 9600");
  cov_def(2, "VOICE_create 48000");
  cov_def(3, "VOICE_create unsupported rate (NULL)");
  cov_def(4, "VOICE_create converter failure");
  cov_def(5, "VOICE_create engine failure");
  cov_def(6, "VOICE_create malloc failure");
  cov_def(7, "callbacks probed");
  cov_def(8, "VOICE_delete");
  cov_def(9, "VOICE_command / VOICE_process on a fresh object");
  cov_def(10, "debug messages seen");
  cov_def(11, "NULL object crashes (both)");
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (ri = 0; ri < sizeof rates / sizeof rates[0]; ri++)
      for (rf = -1; rf < 3; rf++)
        for (vf = 0; vf < 2; vf++)
          for (f = -1; f < 2; f++) {
            VoiceGlue *g;
            script_reset();
            rnd_vi();
            g_probe_cb = (rf == -1 && vf == 0 && f == -1);
            g_rc_fail_at = rf;
            g_vcreate_fail = vf;
            g_fail_malloc = f;
            g = (VoiceGlue *)VOICE_create(FAKEM, rates[ri]);
            emit(0, "VOICE_create rate=%u rf=%d vf=%d mf=%d -> %u", rates[ri], rf, vf, f, hid(g));
            if (g) {
              g_glue = g;
              g_snap = snap_cur;
              show_glue("created", g);
              emit(0, "created blk=%u vi=%d", g->blk, g->vi == &g_vi);
              g_snap = NULL;
              if (rates[ri] == 8000) cov_hit(0);
              else if (rates[ri] == 9600) cov_hit(1);
              else if (rates[ri] == 48000) cov_hit(2);
              g_glue = NULL;
              VOICE_delete(g);
              cov_hit(8);
            } else {
              if (f >= 0) cov_hit(6);
              else if (vf) cov_hit(5);
              else if (rf >= 0 && (rates[ri] == 9600 || rates[ri] == 48000)) cov_hit(4);
              else cov_hit(3);
            }
            if (g_probe_cb) cov_hit(7);
            trial_end("create");
          }
  }
  SETLVL(0);
  /* the callbacks against many voice_info values */
  {
    int k;
    for (t = 0; t < 7; t++) {
      SETLVL(lvl_value(t));
      for (k = 0; k < 150; k++) {
        VoiceGlue *g;
        script_reset();
        rnd_vi();
        if (k < 20) {
          g_vi.silence_detect_sensitivity = (unsigned)k * 13u + (k > 15 ? 0xffffff00u : 0u);
        }
        g_probe_cb = 1;
        g = (VoiceGlue *)VOICE_create(FAKEM, 9600);
        kill_glue(g);
      }
    }
    SETLVL(0);
    trial_end("cb");
  }
  /* VOICE_delete(NULL) dereferences NULL in both */
  guarded_crash("VOICE_delete(NULL)", guarded(child_delete_null, NULL));
  cov_hit(11);
  /* VOICE_command(NULL) is safe */
  {
    int r = VOICE_command(NULL, VOICE_CMD_BEEP);
    emit(1, "VOICE_command(NULL) -> %d", r);
    cov_hit(9);
  }
  cov_hit(10);
}

/* ================================================================================================ scenario 1: VOICE_command */
static void sc1(void)
{
  int t, k;
  cov_def(0, "COMMAND");
  cov_def(1, "RX");
  cov_def(2, "TX");
  cov_def(3, "DUPLEX (unknown)");
  cov_def(4, "SPEAKER");
  cov_def(5, "BEEP");
  cov_def(6, "DTMF (duration 0 -> 1)");
  cov_def(7, "DTMF");
  cov_def(8, "ABORT");
  cov_def(9, "unknown command");
  cov_def(10, "debug messages seen");
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (k = 0; k < 3000; k++) {
      VoiceGlue *g;
      unsigned cmd;
      int r;
      script_reset();
      rnd_vi();
      if (rpct(40)) g_vi.tone_duration = (unsigned)rrange(0, 25);
      g = mk_glue(9600);
      g_snap = snap_cur;
      g->cnt = (int)rnd();
      g->state = (int)rmod(4);
      g_vc_ret = (int)(rpct(50) ? rmod(12) : rnd());
      cmd = rpct(85) ? rmod(9) : rnd();
      if (rpct(5)) cmd = (unsigned)-1;
      r = VOICE_command(g, (enum VOICE_CMD)cmd);
      emit(0, "VOICE_command %u -> %d", cmd, r);
      show_glue("cmd", g);
      switch (cmd) {
      case 0: cov_hit(0); break;
      case 1: cov_hit(1); break;
      case 2: cov_hit(2); break;
      case 3: cov_hit(3); break;
      case 4: cov_hit(4); break;
      case 5: cov_hit(5); break;
      case 6: if (g_vi.tone_duration / 10u == 0) cov_hit(6); else cov_hit(7); break;
      case 7: cov_hit(8); break;
      default: cov_hit(9); break;
      }
      g_snap = NULL;
      kill_glue(g);
    }
  }
  SETLVL(0);
  trial_end("sc1");
  cov_hit(10);
}

/* ================================================================================================ scenario 2: VOICE_process */
static void sc2(void)
{
  int t, k;
  cov_def(0, "msg 0 NO_MESSAGE");
  cov_def(1, "msg 1 OK");
  cov_def(2, "msg 2 START_ONLINE");
  cov_def(3, "msg 3 START_TX");
  cov_def(4, "msg 4 START_RX");
  cov_def(5, "msg 5 START_DUPLEX");
  cov_def(6, "msg 6 PURGE");
  cov_def(7, "msg 7 ERROR");
  cov_def(8, "msg 8 AFTER_ABORT");
  cov_def(9, "msg 9 CANCEL");
  cov_def(10, "msg 10 BUSY");
  cov_def(11, "msg 11 DIALTONE");
  cov_def(12, "msg 12 FAX tone");
  cov_def(13, "msg 13 Underrun");
  cov_def(14, "msg unknown");
  cov_def(15, "same message as the last (ignored)");
  cov_def(16, "rx state: tty read, ABORT sequence");
  cov_def(17, "rx state: tty read, other data");
  cov_def(18, "tx state: tty read");
  cov_def(19, "state command / duplex: no read");
  cov_def(20, "rx state: output sent to the tty");
  cov_def(21, "block run (engine called)");
  cov_def(22, "no block (data buffered)");
  cov_def(23, "chunk larger than one block (the byte-advance quirk)");
  cov_def(24, "count <= 0");
  cov_def(25, "converter produced fewer samples");
  cov_def(26, "mutating hook ran");
  cov_def(27, "fistp overflow / special output samples");
  cov_def(28, "8000 Hz (NULL converters)");
  cov_def(29, "debug messages seen");
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (k = 0; k < 900; k++) {
      VoiceGlue *g;
      unsigned srate = rpct(15) ? 8000u : 9600u;
      int calls = 1 + (int)rmod(6), c;
      script_reset();
      rnd_vi();
      g = mk_glue(srate);
      if (srate == 8000u) cov_hit(28);
      g_snap = snap_cur;
      g->state = (int)(rpct(85) ? rmod(4) : rmod(6));
      g->last = (int)(rpct(50) ? -1 : (int)rmod(15));
      g->cnt = (int)(rpct(60) ? 0xa0 : rpct(50) ? (int)rmod(300) : (int)rnd());
      g_vm_fout_mode = (int)(rpct(80) ? 0 : 1 + rmod(2));
      if (g_vm_fout_mode) cov_hit(27);
      if (rpct(30)) g_dbg_hook = mut_hook;
      if (rpct(25)) g_recv_hook = recv_mut_hook;
      for (c = 0; c < calls; c++) {
        int count, r, i;
        unsigned nin;
        static const int cnts[] = {1, 2, 3, 7, 47, 48, 49, 100, 159, 160, 161, 191, 192, 193, 200, 319, 320, 321, 383, 384, 385, 500, 577};
        if (rpct(30)) { /* poke the ring indexes independently (they are equal after create: in.wr == out.rd always): every limit of the copy arithmetic matters */
          unsigned two = 2 * g->blk;
          g->in.wr = rmod(two);
          g->out.rd = rmod(two);
          g->in.count = rmod(g->blk);
          g->out.count = rmod(2 * g->blk);
          g->in.half = rpct(50) ? 0u : g->blk;
          g->out.half = rpct(50) ? 0u : g->blk;
        }
        count = rpct(60) ? (int)(g->blk) : rpct(60) ? cnts[rmod(sizeof cnts / sizeof cnts[0])] : rpct(95) ? (int)rmod(400) : -(int)rmod(5);
        if (count > 580) count = 580;
        g_vm_ret = (int)(rpct(8) ? (int)rrange(-2, 20) : (int)rmod(14));
        if (rpct(20)) g_vm_ret = g->last; /* the same message again */
        g_vm_a5 = rpct(50) ? rmod(200) : rnd();
        g_vm_a6 = rpct(40) ? 0 : rpct(70) ? rmod(200) : rnd() & 0x3ff;
        g_vm_mutate = rpct(10);
        g_rc_n_in = rpct(70) ? -1 : rrange(0, 0xb0);
        if (rpct(10)) g_rc_n_in = 0xa0;
        /* tty input: a script of its own limit; the ABORT sequence DLE '!' (0x10 0x21) at the front in some */
        nin = rmod(300);
        g_tty_in_n = nin;
        g_tty_in_pos = 0;
        for (i = 0; i < (int)nin; i++) g_tty_in[i] = (unsigned char)rnd();
        if (nin >= 2 && rpct(40)) {
          g_tty_in[0] = 0x10;
          g_tty_in[1] = 0x21;
        }
        g_recv_cap = rpct(30) ? (int)rmod(4) : -1;
        g_tty_out_n = 0;
        for (i = 0; i < (count > 0 ? count : 0) + 4; i++) g_in_s[i] = (short)rnd();
        memset(g_out_s, 0x5a, sizeof g_out_s);
        r = VOICE_process(g, g_in_s, g_out_s, count);
        emit(0, "VOICE_process #%d count=%d state=%d msg=%d -> %d out=%016llx tty_out=%u", c, count, g->state, g_vm_ret, r,
             fnv_shorts(1469598103934665603ULL, g_out_s, (count > 0 ? (unsigned)count : 0u) + 32u), g_tty_out_n);
        show_glue("proc", g);
        if (count <= 0) cov_hit(24);
        if (count > (int)g->blk) cov_hit(23);
        if (g_vm_ret >= 0 && g_vm_ret <= 13) cov_hit(g_vm_ret); else cov_hit(14);
        if (g_tty_out_n) cov_hit(20);
        if (g->in.count < g->blk && count > 0 && count < (int)g->blk) cov_hit(22);
        if (g->blk && count >= (int)g->blk) cov_hit(21);
        if (g_rc_n_in >= 0 && g_rc_n_in < 0xa0) cov_hit(25);
        if (g_vm_mutate || g_hook_n) cov_hit(26);
        g_vm_mutate = 0;
      }
      g_dbg_hook = NULL;
      g_snap = NULL;
      kill_glue(g);
    }
  }
  SETLVL(0);
  trial_end("sc2");
  /* the exact message / state paths: every message from every state, with a block per call at level 2 */
  {
    int st, msg, last;
    for (st = 0; st < 4; st++)
      for (msg = -1; msg < 16; msg++)
        for (last = 0; last < 2; last++) {
          VoiceGlue *g;
          int r, i;
          script_reset();
          dsplibs_debug_level = 2;
          g = mk_glue(9600);
          g_snap = snap_cur;
          g->state = st;
          g->last = last ? msg : (msg == 0 ? 1 : 0);
          g->cnt = 0xa0;
          g_vm_ret = msg;
          g_vm_a5 = 0x77;
          g_vm_a6 = 5;
          g_tty_in_n = 40;
          g_tty_in_pos = 0;
          for (i = 0; i < 40; i++) g_tty_in[i] = (unsigned char)(i * 3);
          if (st == 1) {
            g_tty_in[0] = 0x10;
            g_tty_in[1] = 0x21;
          }
          for (i = 0; i < 200; i++) g_in_s[i] = (short)(i * 9);
          r = VOICE_process(g, g_in_s, g_out_s, (int)g->blk);
          emit(0, "VOICE_process direct st=%d msg=%d last=%d -> %d", st, msg, last, r);
          show_glue("direct", g);
          if (last == 1) cov_hit(15);
          if (st == 1) cov_hit(16);
          if (st == 1) cov_hit(17);
          if (st == 2) cov_hit(18);
          if (st == 0 || st == 3) cov_hit(19);
          g_snap = NULL;
          kill_glue(g);
        }
    SETLVL(0);
    trial_end("sc2d");
  }
  cov_hit(29);
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *s;
  if (getenv("R17_VERBOSE")) g_verbose = 1;
  if ((s = getenv("R17_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(s, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)sc * 0x9e3779b9u;
  (void)g_gone;
  switch (sc) {
  case 0: sc0(); break;
  case 1: sc1(); break;
  case 2: sc2(); break;
  default: die3("no such scenario");
  }
  trial_end("end");
  cov_finish();
  return 0;
}
