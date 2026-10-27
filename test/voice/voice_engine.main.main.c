/*
 * R17 (id r17_voice): differential harness of the voice engine of dsplibs.o: voice_create / voice_delete / voice_command / voice_modem / voice_online / voice_tx / voice_rx /
 * voice_duplex / voice_dle_command / voice_set_rx / voice_set_tx / voice_set_online / voice_set_duplex and linear2ulaw (blob: the real dsplibs.o; port: src/voice/voice_engine.c).  Every
 * component the engine calls is a SCRIPTED RECORDER defined here (beepgen_*, detector_*, silence_*, _status, FDSP_DP_*, FDSP_Kernel_SetInternalBeepInProgress,
 * STRM_VCE_GetFDSPEnvironmentalParams: WEAKENED in a copy of dsplibs.o so these win in the blob run); FIFO8_* is the REAL code on both sides (R14), as are sysdep_*, modem_get_param
 * (harness table), dsplibs_debug_printf.  Each scenario runs in its own process: voice/voice_engine.main.sh builds the blob / port -O0 / port -O2 binaries and diffs the output.
 *
 * What is printed: every call's return value, the written buffers (hashed), the full canonical state of the Voice object (snap_voice: layout independent), the recorders' own log
 * lines (every call and argument) and every debug message together with the state at the moment of the message.  Every INPUT float is built from integers (bit patterns); no libm.
 *
 *   0  linear2ulaw (every int -70000 .. 70000, the extremes, random), voice_dle_command, voice_set_rx / tx / online / duplex, voice_create / voice_delete (failure injection at every
 *      allocation, levels), voice_command (every command x every mode)
 *   1  voice_online and voice_duplex (beep running / done, vls 0 .. 5, sample counts 0 .. 300, the beep generator ending at every position, modes, buffers)
 *   2  voice_tx (DLE stream parsing, fifo fill around every threshold, rates / bit depths, vls, underrun / etx flags, the DC estimate and the pcm conversion)
 *   3  voice_rx (the DC estimate / smoothing, gains, vls 1 / 3 pcm path, the u-law stream with DLE doubling, time marks, abort, special float values)
 *   4  voice_modem (detector status, mode 3, etx / abort resets, scripted and real handlers, the count arithmetic) and debug-level hooks
 */
#include "r17_common.h"

#define NSCEN 5
/* the blob's own linear2ulaw (GLOBAL, not replaced in this build) vs the port's private copy */
#ifdef TEST_BLOB
#define L2U linear2ulaw
extern int linear2ulaw(int);
#else
#define L2U voice_linear2ulaw
#endif

/* ------------------------------------------------------------------------------------------------ scripted components */
static int g_beep_end_at = -1, g_beep_calls, g_beep_mode;
static unsigned g_beep_seed = 1;
static unsigned short *g_a6_hook; /* when set, every beepgen_sample call lowers the count word the engine re-reads inside its loops */
static int g_a6_dec;
static int g_det_ret;
static unsigned g_det_nb;
static unsigned char g_det_bytes[8];
static int g_sil_nb;
static unsigned char g_sil_bytes[8];
static int g_sil_more;
static short g_fd_rx = 0x33, g_fd_tx = 0x171;
static int g_fd_fail;
static unsigned long g_ncreate_beep;

/* functions the harness hands the engine as callbacks (VoiceCfg): names are printed instead of addresses */
static void hk_on(struct modem *m) { emit(0, "HOOK on m%d", MID(m)); }
static void hk_off(struct modem *m) { emit(0, "HOOK off m%d", MID(m)); }
static unsigned g_cb_val[256];
static unsigned g_cb_ids[64];
static unsigned g_cb_n;
static unsigned cb_get(struct modem *m, int id)
{
  unsigned v = g_cb_val[(unsigned)id & 255u];
  if (g_cb_n < 64) g_cb_ids[g_cb_n++] = (unsigned)id;
  emit(0, "CB m%d %d -> %u", MID(m), id, v);
  return v;
}
static const char *cbn(const void *p)
{
  if (!p) return "NULL";
  if (p == (const void *)hk_on) return "hk_on";
  if (p == (const void *)hk_off) return "hk_off";
  if (p == (const void *)cb_get) return "cb_get";
  return "other";
}

BeepGen *beepgen_create(BeepGen *obj, const BeepCfg *cfg)
{
  BeepGen *b;
  g_ncreate_beep++;
  emit(0, "beepgen_create obj=%u ctx=m%d on=%s off=%s sreg=%s", hid(obj), MID(cfg->ctx), cbn((const void *)cfg->hookOn), cbn((const void *)cfg->hookOff), cbn((const void *)cfg->getParam));
  if (obj) return obj;
  b = (BeepGen *)sysdep_malloc((unsigned)sizeof(BeepGen));
  return b;
}
void beepgen_delete(BeepGen *b)
{
  emit(0, "beepgen_delete %u", hid(b));
  sysdep_free(b);
}
void beepgen_start_beep(BeepGen *b, int f1, int f2, int dur)
{
  emit(0, "beepgen_start_beep %u %d %d %d", hid(b), f1, f2, dur);
}
void beepgen_start_dtmf(BeepGen *b, int ch, int dur)
{
  emit(0, "beepgen_start_dtmf %u %d %d", hid(b), ch, dur);
}
static unsigned beep_bits(void)
{
  unsigned k = g_beep_seed * 2654435761u + (unsigned)g_beep_calls * 40503u;
  k ^= k >> 13;
  k *= 0x5bd1e995u;
  k ^= k >> 15;
  switch (g_beep_mode) {
  case 0: return 0x3b800000u + (k & 0x00ffffffu) % 0x03800000u + ((k >> 31) << 31); /* 2^-8 .. 1 */
  case 1: return 0x3f800000u + (k & 0x00ffffffu) % 0x07000000u + ((k >> 31) << 31); /* 1 .. big */
  case 2: return (k & 1u) ? 0x7fc00000u | (k & 0x3fffu) : 0xffc00000u | (k & 0x3fffu); /* quiet NaNs */
  case 3: return (k & 1u) ? 0x7f800001u | (k & 0x3fffu) : 0xff800001u | (k & 0x3fffu); /* signalling NaNs */
  case 4: return (k & 1u) ? 0x7f800000u : 0xff800000u;                                  /* +-inf */
  case 5: return (k & 0x7fffffu) | ((k >> 31) << 31);                                    /* denormals */
  case 6: return (k >> 31) << 31;                                                        /* +-0 */
  default: return k;                                                                      /* anything */
  }
}
int beepgen_sample(BeepGen *b, float *out)
{
  unsigned bits;
  int r;
  g_beep_calls++;
  if (g_a6_hook && *g_a6_hook >= (unsigned)g_a6_dec) *g_a6_hook = (unsigned short)(*g_a6_hook - (unsigned)g_a6_dec);
  bits = beep_bits();
  memcpy(out, &bits, 4);
  r = (g_beep_end_at == g_beep_calls);
  emit(0, "beepgen_sample %u #%d -> %d %08x", hid(b), g_beep_calls, r, bits);
  return r;
}

Detector *detector_create(Detector *obj, struct modem *ctx, VoiceParamFn cb)
{
  emit(0, "detector_create obj=%u ctx=m%d cb=%s", hid(obj), MID(ctx), cbn((const void *)cb));
  if (obj) return obj;
  return (Detector *)sysdep_malloc((unsigned)sizeof(Detector));
}
void detector_delete(Detector *d)
{
  emit(0, "detector_delete %u", hid(d));
  sysdep_free(d);
}
void detector_set_enable(Detector *d, unsigned short v)
{
  emit(0, "detector_set_enable %u %u", hid(d), (unsigned)v);
}
void detector_set_output_status(Detector *d) { emit(0, "detector_set_output_status %u", hid(d)); }
void detector_set_output_in_stream(Detector *d) { emit(0, "detector_set_output_in_stream %u", hid(d)); }
int detector_progress(Detector *d, const float *x, short count, unsigned char *buf, unsigned short *len)
{
  unsigned k;
  emit(0, "detector_progress %u count=%d x=%016llx len=%u -> ret %d nb %u", hid(d), (int)count, fnv_floats(1469598103934665603ULL, x, count > 0 ? (unsigned)count : 0u), (unsigned)*len, g_det_ret, g_det_nb);
  for (k = 0; k < g_det_nb; k++) buf[k] = g_det_bytes[k];
  *len = (unsigned short)(*len + g_det_nb);
  return g_det_ret;
}

Silence *silence_create(Silence *obj, struct modem *ctx, VoiceParamFn cb)
{
  emit(0, "silence_create obj=%u ctx=m%d cb=%s", hid(obj), MID(ctx), cbn((const void *)cb));
  if (obj) return obj;
  return (Silence *)sysdep_malloc((unsigned)sizeof(Silence));
}
void silence_delete(Silence *s)
{
  emit(0, "silence_delete %u", hid(s));
  sysdep_free(s);
}
void silence_progress(Silence *s, const float *x, short count, unsigned char *buf, unsigned short *len)
{
  int k;
  emit(0, "silence_progress %u count=%d x=%016llx len=%u nb %d", hid(s), (int)count, fnv_floats(1469598103934665603ULL, x, count > 0 ? (unsigned)count : 0u), (unsigned)*len, g_sil_nb);
  for (k = 0; k < g_sil_nb; k++) buf[k] = g_sil_bytes[k];
  *len = (unsigned short)(*len + g_sil_nb);
}
int silence_is_more_then(Silence *s, float v)
{
  uint32_t u;
  memcpy(&u, &v, 4);
  emit(0, "silence_is_more_then %u %08x -> %d", hid(s), u, g_sil_more);
  return g_sil_more;
}
void _status(unsigned char *buf, unsigned short *len, int code)
{
  emit(0, "_status len=%u code=%d", (unsigned)*len, code);
  buf[0] = 0x10;
  buf[1] = (unsigned char)code;
  *len = (unsigned short)(*len + 2);
}

FdspObj *FDSP_DP_Create(FdspObj *obj, short rx, short tx)
{
  emit(0, "FDSP_DP_Create obj=%u rx=%d tx=%d", hid(obj), (int)rx, (int)tx);
  if (g_fd_fail) return NULL;
  if (obj) return obj;
  return (FdspObj *)sysdep_malloc((unsigned)sizeof(FdspObj));
}
void FDSP_DP_Delete(FdspObj *f)
{
  emit(0, "FDSP_DP_Delete %u", hid(f));
  sysdep_free(f);
}
int FDSP_DP_Run(FdspObj *f, const short *in1, float *out1, const float *in2, short *out2, void *unused, const unsigned short *n)
{
  unsigned i, nn = *n;
  emit(0, "FDSP_DP_Run %u n=%u in1=%016llx in2=%016llx", hid(f), nn, fnv_shorts(1469598103934665603ULL, in1, nn), fnv_floats(1469598103934665603ULL, in2, nn));
  (void)unused;
  for (i = 0; i < nn; i++) {
    uint32_t u = 0x3c000000u + (unsigned)(unsigned short)in1[i] * 17u + i;
    memcpy(&out1[i], &u, 4);
    {
      short w = (short)(in1[i] + (short)i);
      memcpy((char *)out2 + 2 * (size_t)i, &w, 2); /* out2 = buf + an arbitrary byte count */
    }
  }
  return 1;
}
void FDSP_Kernel_SetInternalBeepInProgress(int on) { emit(0, "SetInternalBeepInProgress %d", on); }
void STRM_VCE_GetFDSPEnvironmentalParams(short *rx, short *tx)
{
  *rx = g_fd_rx;
  *tx = g_fd_tx;
  emit(0, "GetFDSPEnvironmentalParams -> %d %d", (int)g_fd_rx, (int)g_fd_tx);
}


/* ------------------------------------------------------------------------------------------------ helpers */
static Voice *g_curv;
static unsigned fnid_of(const Voice *v)
{
  if (!v->fn) return 0;
  if (v->fn == voice_online) return 1;
  if (v->fn == voice_rx) return 2;
  if (v->fn == voice_tx) return 3;
  if (v->fn == voice_duplex) return 4;
  return 9;
}
static u64 snap_cur(void) { return g_curv ? snap_voice(g_curv, fnid_of(g_curv)) : 0ULL; }

static void script_reset(void)
{
  dsplibs_debug_level = g_lvl_base;
  g_beep_end_at = -1;
  g_beep_calls = 0;
  g_beep_mode = 0;
  g_beep_seed = 1;
  g_det_ret = 0;
  g_det_nb = 0;
  g_sil_nb = 0;
  g_sil_more = 0;
  g_fd_fail = 0;
  g_fd_rx = 0x33;
  g_fd_tx = 0x171;
  g_cb_n = 0;
  g_a6_hook = NULL;
  g_a6_dec = 0;
  memset(g_cb_val, 0, sizeof g_cb_val);
}
static VoiceCfg mkcfg(void)
{
  VoiceCfg c;
  c.ctx = FAKEM;
  c.get_sreg = cb_get;
  c.hook_on = hk_on;
  c.hook_off = hk_off;
  return c;
}
static Voice *mk_voice(void)
{
  VoiceCfg c = mkcfg();
  dsplibs_debug_level = g_lvl_base;
  Voice *v = voice_create(&c);
  if (!v) die3("voice_create failed in mk_voice");
  g_curv = v;
  return v;
}
static void kill_voice(Voice *v)
{
  g_curv = NULL;
  voice_delete(v);
  if (g_nblk > 2500 && g_live == 0) trial_end("auto"); /* the block table is bounded: a quiet point */
}
static void show_voice(const char *what, const Voice *v)
{
  emit(0, "%s state=%016llx", what, snap_voice(v, fnid_of(v)));
}
/* float bit patterns of several classes */
static unsigned fpat(int cls)
{
  unsigned k = rnd();
  switch (cls) {
  case 0: return 0x38800000u + (k & 0x007fffffu) % 0x07800000u + ((k >> 31) << 31); /* small .. 1 */
  case 1: return 0x3f800000u + (k & 0x007fffffu) % 0x0f000000u + ((k >> 31) << 31); /* 1 .. 2^30 */
  case 2: return 0x7fc00000u | (k & 0x3fffffu) | ((k >> 31) << 31);                   /* quiet NaN */
  case 3: return 0x7f800000u | (k & 0x3fffffu) | 1u | ((k >> 31) << 31);              /* signalling NaN */
  case 4: return 0x7f800000u | ((k >> 31) << 31);                                     /* inf */
  case 5: return (k & 0x7fffffu) | ((k >> 31) << 31);                                 /* denormal */
  case 6: return (k >> 31) << 31;                                                     /* +-0 */
  case 7: return 0x46000000u + (k & 0x00ffffffu) % 0x00800000u + ((k >> 31) << 31);   /* 8192 .. 40000 */
  default: return k;
  }
}
static void fill_floats(float *p, unsigned n, int cls_mask_style)
{
  unsigned i;
  for (i = 0; i < n; i++) {
    int cls;
    uint32_t u;
    switch (cls_mask_style) {
    case 0: cls = 0; break;                                   /* normal */
    case 1: cls = (int)rmod(2); break;                        /* normal and large */
    case 2: cls = rpct(5) ? (int)rmod(9) : 0; break;          /* a few specials */
    case 3: cls = 7; break;                                   /* 8192 .. 40000 */
    default: cls = (int)rmod(9); break;                       /* everything */
    }
    u = fpat(cls);
    memcpy(&p[i], &u, 4);
  }
}
static u64 hash_floats(const float *p, unsigned n) { return fnv_floats(1469598103934665603ULL, p, n); }

/* big work buffers: [GUARD] [data ...] [GUARD] so a stray write is seen */
#define BIGF 70000
static float g_in_f[BIGF + 64], g_out_f[BIGF + 64];
static unsigned char g_in_b[2 * BIGF + 64], g_out_b[2 * BIGF + 64];
static short g_in_s[BIGF + 64], g_out_s[BIGF + 64];
static void guards_set(void)
{
  memset(g_out_b, 0xA5, sizeof g_out_b);
  memset(g_out_s, 0x5A, sizeof g_out_s);
  {
    unsigned i;
    for (i = 0; i < BIGF + 64; i++) {
      uint32_t u = 0x7fa5a5a5u;
      memcpy(&g_out_f[i], &u, 4);
    }
  }
}
static void vsel(Voice *v, int mode, int vls)
{
  v->mode = mode;
  v->vls = vls;
}

static void mutate_hook(const char *msg);
static VoiceFn rand_fn(void)
{
  switch (rmod(6)) {
  case 0: return NULL;
  case 1: return voice_rx;
  case 2: return voice_tx;
  case 3: return voice_duplex;
  default: return voice_online;
  }
}
static VoiceCfg g_cfg_for_child;
static void child_create_fifo_fail(void *arg)
{
  int a = (int)(intptr_t)arg;
  Voice *v;
  script_reset();
  g_fd_fail = a & 1;
  g_fail_malloc = a >> 3;
  v = voice_create(&g_cfg_for_child);
  if (v) die3("voice_create succeeded with a failing FIFO8_create");
  g_fail_malloc = -1;
}
/* ================================================================================================ scenario 0 */
static void sc0(void)
{
  unsigned acc = 0, i;
  int k, t, c;
  cov_def(0, "linear2ulaw positive");
  cov_def(1, "linear2ulaw negative");
  cov_def(2, "linear2ulaw overflow (>= 0x7f7c)");
  cov_def(3, "voice_dle_command ETX");
  cov_def(4, "voice_dle_command CAN");
  cov_def(5, "voice_dle_command other");
  cov_def(6, "voice_create ok");
  cov_def(7, "voice_create malloc failure");
  cov_def(8, "voice_create component failure (cleanup)");
  cov_def(9, "voice_create NULL cfg");
  cov_def(10, "voice_command beep ok");
  cov_def(11, "voice_command beep not online");
  cov_def(12, "voice_command dtmf ok");
  cov_def(13, "voice_command set mode rx");
  cov_def(14, "voice_command set mode tx");
  cov_def(15, "voice_command set mode online");
  cov_def(16, "voice_command set mode duplex");
  cov_def(17, "voice_command pcm config");
  cov_def(18, "voice_command abort / vls / time mark / volume / level");
  cov_def(19, "voice_command reset duplex");
  cov_def(20, "voice_command unknown");
  cov_def(21, "voice_set_rx / tx / online / duplex");
  cov_def(22, "voice_delete");
  cov_def(23, "debug messages seen");

  /* ---- linear2ulaw ---- */
  {
    u64 h = 1469598103934665603ULL;
    for (k = -70000; k <= 70000; k++) {
      int r = L2U(k);
      h = fnv_mix(h, (unsigned)r);
      cov_hit(k < 0 ? 1 : 0);
      if (k >= 0x7f7c) cov_hit(2);
    }
    emit(1, "L2U sweep %016llx", h);
    /* the byte 0x10 (the DLE escape voice_rx doubles) is never produced for the 14-bit values voice_rx passes (|x| <= 8192): the doubling branch is unreachable */
    for (k = -8192; k <= 8191; k++)
      if (L2U(k) == 0x10) die3("linear2ulaw(%d) == 0x10", k);
    emit(1, "L2U: no 0x10 for -8192 .. 8191");
    h = 1469598103934665603ULL;
    {
      static const int ex[] = {0x7fffffff, 0x7ffffffe, 0x7fffff7b, 0x7fffff7c, 0x7fffff7d, (int)0x80000000, (int)0x80000001, (int)0x8000007c, (int)0x8000007d, -0x7fff, -0x8000, -0x8001, 0x7f7b, 0x7f7c, 0x7f7d, 0xff, 0x100};
      for (i = 0; i < sizeof ex / sizeof ex[0]; i++) {
        int r = L2U(ex[i]);
        h = fnv_mix(h, (unsigned)r);
        emit(0, "L2U %d -> %d", ex[i], r);
      }
    }
    for (i = 0; i < 400000; i++) {
      int x = (int)rnd();
      if (i & 1) x >>= (int)rmod(31);
      h = fnv_mix(h, (unsigned)L2U(x));
    }
    emit(1, "L2U random %016llx", h);
  }

  /* ---- voice_dle_command ---- */
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (k = -300; k < 600; k++) {
      Voice *v = mk_voice();
      int r;
      g_snap = snap_cur;
      g_dbg_hook = rpct(20) ? mutate_hook : NULL;
      v->etx = rpct(50) ? 0 : 1;
      v->abort = rpct(50) ? 0 : 1;
      r = voice_dle_command(v, k);
      emit(0, "dle %d -> %d", k, r);
      show_voice("dle", v);
      if ((signed char)k == 3) cov_hit(3);
      else if ((signed char)k == 0x18) cov_hit(4);
      else cov_hit(5);
      g_snap = NULL;
      g_dbg_hook = NULL;
      kill_voice(v);
    }
  }
  SETLVL(0);
  trial_end("dle");

  /* ---- voice_create / voice_delete: failure injection ---- */
  for (t = 0; t < 7; t++) {
    int f;
    SETLVL(lvl_value(t));
    for (f = -1; f < 14; f++) {
      VoiceCfg c = mkcfg();
      Voice *v;
      int fdf;
      for (fdf = 0; fdf < 2; fdf++) {
        script_reset();
        g_fd_fail = fdf;
        g_fail_malloc = f;
        if (f == 4 - fdf || f == 5 - fdf) { /* FIFO8_create's two allocations: the blob dereferences NULL (R14, docs/bugs.md) */
          g_cfg_for_child = c;
          guarded_report("voice_create with a failing FIFO8_create", guarded(child_create_fifo_fail, (void *)(intptr_t)(f * 8 + fdf)));
          g_fail_malloc = -1;
          continue;
        }
        v = voice_create(&c);
        emit(0, "voice_create f=%d fdf=%d -> %u", f, fdf, hid(v));
        if (v) {
          cov_hit(6);
          show_voice("created", v);
          g_curv = v;
          g_snap = snap_cur;
          voice_delete(v);
          g_snap = NULL;
          g_curv = NULL;
          cov_hit(22);
        } else {
          cov_hit(f >= 0 ? 7 : 8);
          if (f >= 0) cov_hit(8);
        }
        trial_end("create");
      }
    }
    /* NULL cfg */
    {
      Voice *v = voice_create(NULL);
      emit(0, "voice_create(NULL) -> %u", hid(v));
      cov_hit(9);
    }
    /* different configurations: every callback NULL / set */
    {
      int m;
      for (m = 0; m < 16; m++) {
        VoiceCfg c;
        Voice *v;
        c.ctx = (m & 1) ? FAKEM : FAKEM2;
        c.get_sreg = (m & 2) ? cb_get : NULL;
        c.hook_on = (m & 4) ? hk_on : NULL;
        c.hook_off = (m & 8) ? hk_off : NULL;
        v = voice_create(&c);
        emit(0, "voice_create cfg %d -> %u", m, hid(v));
        if (v) {
          show_voice("cfg", v);
          voice_delete(v);
        }
      }
    }
    trial_end("create2");
  }
  SETLVL(0);

  /* ---- voice_set_* ---- */
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (k = 0; k < 600; k++) {
      Voice *v;
      script_reset();
      v = mk_voice();
      g_snap = snap_cur;
      g_dbg_hook = rpct(20) ? mutate_hook : NULL;
      v->fn = rand_fn();
      v->mode = (int)rnd();
      v->mode_prev = (int)rnd();
      v->en_tx = (unsigned short)rnd();
      v->en_rx = (unsigned short)rnd();
      v->en_on = (unsigned short)rnd();
      v->tmark = (unsigned short)(rpct(30) ? rmod(4) : rnd());
      v->tcount = (unsigned short)rnd();
      v->rate = (unsigned short)rnd();
      v->bits = (unsigned short)rnd();
      v->first = (int)rnd();
      v->dc = 1.0f;
      v->rx_on = (unsigned short)rnd();
      v->underrun = (unsigned short)rnd();
      {
        int j;
        for (j = 0; j < 256; j++) {
          unsigned w = rnd();
          g_cb_val[j] = rpct(30) ? (rpct(50) ? 0xffffffffu : 0x80000000u) : (rpct(40) ? w >> 20 : w);
        }
      }
      switch (k & 3) {
      case 0: voice_set_rx(v); break;
      case 1: voice_set_tx(v); break;
      case 2: voice_set_online(v); break;
      default: voice_set_duplex(v); break;
      }
      cov_hit(21);
      show_voice("set", v);
      g_snap = NULL;
      g_dbg_hook = NULL;
      kill_voice(v);
    }
  }
  SETLVL(0);
  trial_end("set");

  /* ---- voice_command ---- */
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (k = 0; k < 4000; k++) {
      Voice *v;
      int cmd, args[3], r;
      script_reset();
      v = mk_voice();
      g_snap = snap_cur;
      g_dbg_hook = rpct(20) ? mutate_hook : NULL;
      v->fn = rand_fn();
      v->mode = rpct(60) ? (int)rmod(5) : (int)rnd();
      v->mode_prev = (int)rnd();
      v->en_tx = (unsigned short)rnd();
      v->en_rx = (unsigned short)rnd();
      v->en_on = (unsigned short)rnd();
      v->beep_idle = (int)rmod(3);
      v->tmark = (unsigned short)rnd();
      v->vls = (int)rnd();
      v->tcount = (unsigned short)rnd();
      v->playvol = (unsigned short)rnd();
      {
        int j;
        for (j = 0; j < 256; j++) g_cb_val[j] = rpct(20) ? 0xffffffffu : rnd();
      }
      cmd = rpct(85) ? rrange(-2, 13) : (int)rnd();
      for (c = 0; c < 3; c++) args[c] = rpct(40) ? rrange(-3, 5) : (rpct(50) ? (int)rnd() : (int)(rnd() & 0xff00ffu));
      if (cmd == 2 && rpct(80)) args[0] = rrange(-1, 4);
      g_fd_rx = (short)rrange(-5, 400);
      g_fd_tx = (short)rrange(-5, 400);
      r = voice_command(v, cmd, args);
      emit(0, "voice_command %d %d %d %d -> %d", cmd, args[0], args[1], args[2], r);
      show_voice("cmd", v);
      g_dbg_hook = NULL;
      switch ((unsigned)cmd) {
      case 0: cov_hit(r == 7 ? 11 : 10); break;
      case 1: cov_hit(r == 7 ? 11 : 12); break;
      case 2: if (args[0] == 0) cov_hit(13); else if (args[0] == 1) cov_hit(14); else if (args[0] == 2) cov_hit(15); else if (args[0] == 3) cov_hit(16); break;
      case 5: cov_hit(17); break;
      case 3: case 6: case 7: case 8: case 9: cov_hit(18); break;
      case 10: cov_hit(19); break;
      default: cov_hit(20); break;
      }
      g_snap = NULL;
      kill_voice(v);
    }
  }
  SETLVL(0);
  trial_end("cmd");
  cov_hit(23);
  (void)acc;
}

/* ================================================================================================ scenarios 1 .. 4 */
#include "r17_voice_sc.inc"

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *s;
  if (getenv("R17_VERBOSE")) g_verbose = 1;
  if ((s = getenv("R17_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(s, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)sc * 0x9e3779b9u;
  g_vi.rx_gain = 50;
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
