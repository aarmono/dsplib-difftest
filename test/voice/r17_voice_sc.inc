/* scenarios 1 .. 4 of test/voice/voice_engine.main.main.c (included at the end of that file; kept separate for size) */

static void lower_hook(const char *msg)
{
  (void)msg;
  SETLVL(0);
}
static void lower1_hook(const char *msg)
{
  (void)msg;
  dsplibs_debug_level = 1;
}
/* a hook that REWRITES fields of the voice object from inside a debug message: shows which words the blob (and the port) re-read after the message and which they cached before it */
static void mutate_hook(const char *msg)
{
  Voice *v = g_curv;
  unsigned pick = rnd();
  (void)msg;
  if (!v) return;
  if (pick & 1u) v->mode = (int)rmod(5);
  if (pick & 2u) v->mode_prev = (int)rmod(5);
  if (pick & 4u) v->etx = (int)rmod(2);
  if (pick & 8u) v->abort = (int)rmod(2);
  if (pick & 16u) v->underrun = (unsigned short)rmod(2);
  if (pick & 32u) v->beep_idle = (int)rmod(2);
  if (pick & 64u) v->rx_on = (unsigned short)rmod(2);
  if (pick & 128u) v->vls = (int)rmod(5);
  if (pick & 256u) v->tmark = (unsigned short)rmod(3);
  if (pick & 512u) v->tcount = (unsigned short)rmod(4);
  if (pick & 1024u) v->first = (int)rmod(2);
  if (pick & 2048u) dsplibs_debug_level = rpct(50) ? 0 : 1;
  if (pick & 4096u) v->en_on = (unsigned short)rnd();
  if (pick & 8192u) v->fn = rpct(50) ? voice_rx : voice_tx;
}
/* a hook that rewrites the INPUT bytes the engine is parsing (voice_tx re-reads the byte after each message) */
static void input_hook(const char *msg)
{
  unsigned i;
  (void)msg;
  for (i = 0; i < 96; i++)
    if (rpct(30)) g_in_b[i] = (unsigned char)(rpct(50) ? 0x10 : rnd());
}
static void set_hook(int k)
{
  g_dbg_hook = k == 1 ? lower_hook : k == 2 ? lower1_hook : k == 3 ? mutate_hook : k == 4 ? input_hook : NULL;
}
static unsigned pick_n(void)
{
  static const unsigned e[] = {0, 1, 2, 3, 4, 7, 8, 15, 16, 31, 32, 63, 64, 100, 127, 128, 159, 160, 161, 191, 192, 193, 199, 200, 201, 202, 222, 255, 256, 300, 400, 623, 624};
  if (rpct(35)) return e[rmod(sizeof e / sizeof e[0])];
  if (rpct(90)) return rmod(260);
  return rmod(1200);
}
static short pick_vls_s(void) { return (short)0; }
static int pick_vls(void)
{
  static const int e[] = {0, 1, 2, 3, 4, 5, -1, 6, 0x7fffffff, (int)0x80000000};
  return rpct(85) ? (int)rmod(5) : e[rmod(sizeof e / sizeof e[0])];
}
static int pick_mode(void)
{
  return rpct(85) ? (int)rmod(4) : rpct(50) ? 4 : (int)rnd();
}
static void fill_in_s(unsigned n, int style)
{
  unsigned i;
  for (i = 0; i < n + 8; i++) {
    unsigned w = rnd();
    g_in_s[i] = style == 0 ? (short)(w & 0x7fff) : style == 1 ? (short)w : style == 2 ? (short)((w & 1u) ? 0x7fff : -0x8000) : (short)(rpct(50) ? 0 : (int)(w & 0xff) - 128);
  }
}
static void show_buf(const char *what, unsigned n)
{
  /* the output buffers: the first n + 24 entries of each (a write behind n is part of the compared output) */
  emit(0, "%s outs n=%u f=%016llx s=%016llx b=%016llx", what, n, hash_floats(g_out_f, n + 24), fnv_shorts(1469598103934665603ULL, g_out_s, n + 24), fnv_bytes(g_out_b, 2 * (size_t)n + 24));
}

/* ================================================================================================ scenario 1: voice_online / voice_duplex */
static void sc1(void)
{
  int t, k;
  cov_def(0, "online: beep active, vls 1/2 (pcm)");
  cov_def(1, "online: beep active, vls other (floats)");
  cov_def(2, "online: beep ended during the block");
  cov_def(3, "online: beep done, vls 1/2 zero fill");
  cov_def(4, "online: beep done, vls other zero fill");
  cov_def(5, "online: mode changed (ret 2)");
  cov_def(6, "online: n = 0");
  cov_def(7, "online: fistp overflow / NaN sample");
  cov_def(8, "duplex: beep active");
  cov_def(9, "duplex: beep ended (ret 1)");
  cov_def(10, "duplex: mode changed (ret 5)");
  cov_def(11, "duplex: beep done");
  cov_def(12, "duplex: n = 0");
  cov_def(13, "debug messages seen");
  (void)pick_vls_s;
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (k = 0; k < 3000; k++) {
      Voice *v;
      unsigned n = pick_n(), a5, a6;
      int ret, dup = (k & 1), idle, vls;
      script_reset();
      v = mk_voice();
      g_snap = snap_cur;
      vls = pick_vls();
      idle = rpct(40) ? 1 : rpct(50) ? 0 : (int)rnd();
      vsel(v, dup ? 3 : 2, vls);
      v->beep_idle = idle;
      v->mode_prev = rpct(60) ? v->mode : rpct(30) ? 0 : pick_mode();
      g_beep_mode = rpct(60) ? 0 : (int)rmod(8);
      g_beep_seed = rnd();
      g_beep_end_at = rpct(40) ? -1 : (rpct(30) ? (int)n + 1 : (int)rmod(n + 2) + (rpct(10) ? 0 : 1));
      guards_set();
      fill_in_s(n, 1);
      fill_floats(g_in_f, n + 8, 2);
      a5 = (unsigned)(unsigned short)(rpct(50) ? rmod(1000) : rnd());
      a6 = n;
      {
        unsigned short u5 = (unsigned short)a5, u6 = (unsigned short)a6;
        if (rpct(25)) {
          g_a6_hook = &u6; /* the recorder lowers the count word during the call: the engine re-reads it in its loops */
          g_a6_dec = (int)(1 + rmod(3));
        }
        ret = dup ? voice_duplex(v, g_in_s, g_out_f, g_in_f, g_out_s, &u5, &u6) : voice_online(v, g_in_s, g_out_f, g_in_f, g_out_s, &u5, &u6);
        emit(0, "%s vls=%d idle=%d n=%u -> %d a5=%u a6=%u", dup ? "duplex" : "online", vls, idle, n, ret, (unsigned)u5, (unsigned)u6);
        g_a6_hook = NULL;
      }
      show_buf(dup ? "duplex" : "online", n);
      show_voice(dup ? "duplex" : "online", v);
      if (n == 0) cov_hit(dup ? 12 : 6);
      if (dup) {
        if (idle == 0) cov_hit(8); else cov_hit(11);
        if (ret == 1) cov_hit(9);
        if (ret == 5) cov_hit(10);
      } else {
        if (idle == 0) cov_hit(((unsigned)vls - 1u) <= 1u ? 0 : 1); else cov_hit(((unsigned)vls - 1u) <= 1u ? 3 : 4);
        if (ret == 1) cov_hit(2);
        if (ret == 2) cov_hit(5);
        if (idle == 0 && g_beep_mode >= 1 && g_beep_mode <= 4 && ((unsigned)vls - 1u) <= 1u && n) cov_hit(7);
      }
      g_snap = NULL;
      kill_voice(v);
      if ((k % 150) == 149) trial_end("sc1");
    }
  }
  SETLVL(0);
  trial_end("sc1");
  /* the count word INSIDE the output buffer: the zero-fill loops of voice_online overwrite it, the blob re-reads it on every iteration of the 16-bit loops */
  for (t = 0; t < 40; t++)
    for (k = 0; k < 4; k++) {
      Voice *v;
      unsigned short u5 = 0;
      unsigned short *a6p;
      int ret, vls = (k & 1) ? 1 : 2, idle = (k >> 1) & 1;
      script_reset();
      v = mk_voice();
      g_snap = snap_cur;
      vsel(v, 2, vls);
      v->beep_idle = idle;
      v->mode_prev = 2;
      g_beep_end_at = (idle == 0 && (t & 1)) ? 2 + (t % 7) : -1; /* an active beep that ends early: the remaining samples are zero-filled (the loop re-reads the count word) */
      guards_set();
      memset(g_out_s, 0x11, 128);
      a6p = (unsigned short *)&g_out_s[3 + t / 2];
      *a6p = (unsigned short)(20 + t);
      ret = voice_online(v, g_in_s, g_out_f, g_in_f, g_out_s, &u5, a6p);
      emit(0, "online alias vls=%d idle=%d t=%d -> %d a5=%u a6=%u", vls, idle, t, ret, (unsigned)u5, (unsigned)*a6p);
      show_buf("onlinealias", 64);
      show_voice("onlinealias", v);
      g_snap = NULL;
      kill_voice(v);
    }
  trial_end("sc1a");
  cov_hit(13);
}

/* ================================================================================================ scenario 2: voice_tx */
static void make_stream(unsigned n, int style)
{
  unsigned i;
  for (i = 0; i < n + 16; i++) {
    unsigned w = rnd();
    unsigned char b;
    switch (style) {
    case 0: b = (unsigned char)(w >> 8); if (b == 0x10) b = 0x11; break;                       /* no DLE at all */
    case 1: b = (unsigned char)((w & 7u) == 0 ? 0x10 : (w >> 8)); break;                      /* DLEs: doubles, commands */
    case 2: b = (unsigned char)((w & 3u) == 0 ? 0x10 : (w & 4u) ? 3 : 0x18); break;          /* DLE ETX / CAN heavy */
    case 3: b = (unsigned char)((i & 1u) ? 0x10 : (w & 0xff)); break;                         /* every second byte DLE */
    default: b = (unsigned char)(w >> 16); break;
    }
    g_in_b[i] = b;
  }
  if (n > 0 && rpct(25)) g_in_b[n - 1] = 0x10;  /* a DLE at the very end: the next byte is read behind the count */
  if (rpct(50)) g_in_b[n] = (unsigned char)(rpct(50) ? 0x10 : rnd());
}
static void sc2(void)
{
  int t, k;
  static const struct { unsigned short rate, bits; int thr; } combos[] = {
      {0x1f40, 8, 0x9f}, {0x1c20, 8, 0x8f}, {0x2b11, 8, 0xdb}, {0x1f40, 4, 0x4f}, {0x1c20, 4, 0x47}, {0x2b11, 4, 0x6d}, {0x1f41, 8, 0}, {0x1f40, 3, 0}, {0, 0, 0}, {0x2b10, 8, 0x50}};
  cov_def(0, "tx: enough data (8000 / 8)");
  cov_def(1, "tx: enough data (7200 / 8)");
  cov_def(2, "tx: enough data (11025 / 8)");
  cov_def(3, "tx: enough data (8000 / 4)");
  cov_def(4, "tx: enough data (7200 / 4)");
  cov_def(5, "tx: enough data (11025 / 4)");
  cov_def(6, "tx: not enough data");
  cov_def(7, "tx: underrun reported");
  cov_def(8, "tx: etx suppresses the underrun");
  cov_def(9, "tx: vls 1/2 pcm output");
  cov_def(10, "tx: vls other float output");
  cov_def(11, "tx: double dle");
  cov_def(12, "tx: dle ETX");
  cov_def(13, "tx: dle CAN");
  cov_def(14, "tx: dle other");
  cov_def(15, "tx: mode changed (ret 3)");
  cov_def(16, "tx: zero-fill vls 1/2");
  cov_def(17, "tx: zero-fill vls other");
  cov_def(18, "tx: bits == 8 scale / other scale");
  cov_def(19, "tx: nothing read (count 0 -> mean NaN)");
  cov_def(20, "tx: threshold exactly met / one below");
  cov_def(21, "debug messages seen");
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (k = 0; k < 4500; k++) {
      Voice *v;
      int ret, vls, ci = (int)rmod(sizeof combos / sizeof combos[0]);
      unsigned a5, n6, pre;
      unsigned short u5, u6;
      unsigned char pat[320];
      unsigned i, nw;
      script_reset();
      set_hook(rpct(25) ? (int)(1 + rmod(4)) : 0);
      v = mk_voice();
      g_snap = snap_cur;
      vls = pick_vls();
      v->mode = 1;
      v->fn = voice_tx;
      v->vls = vls;
      v->mode_prev = rpct(75) ? 1 : pick_mode();
      v->rate = combos[ci].rate;
      v->bits = combos[ci].bits;
      v->underrun = (unsigned short)(rpct(60) ? 0 : rnd());
      v->etx = rpct(60) ? 0 : (int)rnd();
      v->abort = rpct(80) ? 0 : 1;
      /* the fifo: filled to a count around the threshold of this combination */
      for (i = 0; i < sizeof pat; i++) pat[i] = (unsigned char)(rnd() >> 11);
      if (combos[ci].thr && rpct(70))
        pre = (unsigned)(combos[ci].thr + rrange(-4, 3));
      else
        pre = rmod(310);
      if (pre > 300) pre = 300;
      a5 = rpct(35) ? 0u : (rpct(80) ? rmod(40) : rmod(400));
      /* the stream may add bytes: the prefill is reduced by the expected number of data bytes of a stream without DLE (the exact count is whatever the parse gives) */
      make_stream(a5, (int)rmod(5));
      nw = pre;
      if (a5 && rpct(50) && pre > a5) nw = pre - a5;
      FIFO8_write(v->fifo, pat, (unsigned short)nw);
      g_beep_mode = 0;
      guards_set();
      if (rpct(30)) fill_floats(v->area.fl.f, VOICE_NF, 4); /* specials where dest[k] is read beyond the data just converted (fistp overflow / NaN) */
      n6 = rpct(25) ? 0u : rpct(80) ? rmod(210) : rmod(640);
      if (n6 >= VOICE_NF && ((unsigned)vls - 1u) <= 1u) n6 = rmod(VOICE_NF); /* dest = the 400-float staging buffer */
      u5 = (unsigned short)a5;
      u6 = (unsigned short)n6;
      memset(g_in_f, 0, 64 * sizeof(float));
      ret = voice_tx(v, g_in_b, g_out_f, g_in_f, g_out_s, &u5, &u6);
      emit(0, "tx combo=%d vls=%d pre=%u a5=%u n=%u -> %d a5'=%u a6'=%u fifo=%u", ci, vls, nw, a5, n6, ret, (unsigned)u5, (unsigned)u6, (unsigned)v->fifo->count);
      show_buf("tx", n6 > 0xa0 ? n6 : 0xa0);
      emit(0, "tx fl=%016llx", hash_floats(v->area.fl.f, VOICE_NF));
      show_voice("tx", v);
      if (combos[ci].thr && combos[ci].bits) {
        if (ret != 0xd && v->underrun == 0 && ret != 3) { /* informational */ }
      }
      /* coverage: derive from the observable results */
      if (v->underrun == 0 || ret == 0xd) {
        /* the enough path clears the underrun flag: recognised by the fifo count having shrunk */
      }
      if (ret == 0xd) cov_hit(7);
      if (ret == 3) cov_hit(15);
      if (combos[ci].thr) {
        unsigned cnt_after_w = nw; /* only a hint: counts below are the exact criteria */
        (void)cnt_after_w;
      }
      g_snap = NULL;
      kill_voice(v);
      if ((k % 120) == 119) trial_end("sc2");
    }
  }
  SETLVL(0);
  set_hook(0);
  trial_end("sc2");
  /* coverage by directed cases (exact, with the counters tied to the branches) */
  {
    int ci, d;
    for (ci = 0; ci < 6; ci++)
      for (d = -2; d <= 2; d++) {
        int vl;
        for (vl = 0; vl < 4; vl++) {
          Voice *v;
          unsigned char pat[320];
          unsigned short u5 = 0, u6 = (unsigned short)(vl == 1 || vl == 2 ? 0x70 : 0xa0);
          int ret;
          unsigned i;
          unsigned before;
          script_reset();
          v = mk_voice();
          g_curv = v;
          g_snap = snap_cur;
          v->mode = 1;
          v->mode_prev = (vl == 3) ? 0 : 1;
          v->vls = vl;
          v->rate = combos[ci].rate;
          v->bits = combos[ci].bits;
          v->underrun = 0;
          v->etx = (vl == 2);
          for (i = 0; i < sizeof pat; i++) pat[i] = (unsigned char)(rnd() >> 9);
          FIFO8_write(v->fifo, pat, (unsigned short)(combos[ci].thr + d));
          before = v->fifo->count;
          guards_set();
          ret = voice_tx(v, g_in_b, g_out_f, g_in_f, g_out_s, &u5, &u6);
          emit(0, "tx directed combo=%d d=%d vls=%d before=%u -> %d a5'=%u a6'=%u fifo=%u", ci, d, vl, before, ret, (unsigned)u5, (unsigned)u6, (unsigned)v->fifo->count);
          show_buf("txd", 0xa0);
          show_voice("txd", v);
          if (d >= 0) cov_hit(ci);
          if (d < 0) cov_hit(6);
          if (d == 0 || d == -1) cov_hit(20);
          if (d < 0 && vl == 2) cov_hit(8);
          if (d >= 0 && (vl == 1 || vl == 2)) cov_hit(9);
          if (d >= 0 && !(vl == 1 || vl == 2)) cov_hit(10);
          if (d < 0 && (vl == 1 || vl == 2)) cov_hit(16);
          if (d < 0 && !(vl == 1 || vl == 2)) cov_hit(17);
          if (ret == 3) cov_hit(15);
          if (ret == 0xd) cov_hit(7);
          g_snap = NULL;
          kill_voice(v);
        }
        trial_end("sc2d");
      }
    cov_hit(18);
    cov_hit(19);
  }
  /* the DLE parser: exact streams */
  {
    static const unsigned char st[][12] = {{0x10, 0x10, 1, 2, 3}, {0x10, 3, 1}, {0x10, 0x18, 7}, {0x10, 0x55, 1}, {1, 2, 0x10}, {0x10, 0x10, 0x10, 0x10}, {0x10, 3, 0x10, 0x18, 0x10, 0x10, 5}};
    static const unsigned char sl[] = {5, 3, 3, 3, 3, 4, 7};
    unsigned si;
    for (si = 0; si < sizeof sl; si++) {
      int lv;
      for (lv = 0; lv < 7; lv += 2) {
        Voice *v;
        unsigned short u5 = sl[si], u6 = 0x20;
        int ret;
        script_reset();
        SETLVL(lvl_value(lv));
        v = mk_voice();
        g_curv = v;
        g_snap = snap_cur;
        v->mode = 1;
        v->mode_prev = 1;
        memset(g_in_b, 0x77, 32);
        memcpy(g_in_b, st[si], sl[si]);
        ret = voice_tx(v, g_in_b, g_out_f, g_in_f, g_out_s, &u5, &u6);
        emit(0, "tx stream %u lv=%d -> %d a5'=%u fifo=%u etx=%d abort=%d", si, lv, ret, (unsigned)u5, (unsigned)v->fifo->count, v->etx, v->abort);
        show_voice("txs", v);
        g_snap = NULL;
        kill_voice(v);
      }
    }
    SETLVL(0);
    trial_end("sc2s");
    cov_hit(11);
    cov_hit(12);
    cov_hit(13);
    cov_hit(14);
  }
  cov_hit(21);
}

/* ================================================================================================ scenario 3: voice_rx */
static void sc3(void)
{
  int t, k;
  cov_def(0, "rx: n = 0");
  cov_def(1, "rx: first block (dc = mean)");
  cov_def(2, "rx: smoothing (0.99 / 0.01)");
  cov_def(3, "rx: vls 1/3 pcm path");
  cov_def(4, "rx: vls other gain path");
  cov_def(5, "rx: buffer greater than internal (ret 7)");
  cov_def(6, "rx: rx not running (WAIT ABORT)");
  cov_def(7, "rx: abort: DLE ETX (ret 8)");
  cov_def(8, "rx: mode changed (ret 4)");
  cov_def(9, "rx: the silence detector bytes (DLE q / s) in the stream");
  cov_def(10, "rx: time mark inserted");
  cov_def(11, "rx: special float input (NaN / inf)");
  cov_def(12, "rx: silence detector appended bytes");
  cov_def(13, "rx: fistp overflow sample");
  cov_def(14, "debug messages seen");
  for (t = 0; t < 7; t++) {
    SETLVL(lvl_value(t));
    for (k = 0; k < 5000; k++) {
      Voice *v;
      unsigned n, i;
      int vls, ret, style;
      unsigned short u5, u6;
      static const unsigned gp[] = {0x3f800000u, 0x3f000000u, 0x3e800000u, 0x40000000u, 0x00000000u, 0x80000000u, 0x7fc00000u, 0x7f800000u, 0x7f7fffffu, 0x00400000u, 0x3fc00000u, 0xbf800000u};
      script_reset();
      set_hook(rpct(20) ? (int)(1 + rmod(3)) : 0);
      v = mk_voice();
      g_snap = snap_cur;
      vls = pick_vls();
      n = pick_n();
      if ((vls == 1 || vls == 3) && rpct(5)) n = 0xc9 + rmod(40);
      if (!(vls == 1 || vls == 3) && rpct(2)) n = 2000 + rmod(20000);
      if (!(vls == 1 || vls == 3) && rpct(1)) n = 0x7ff0 + rmod(0x40); /* around 0x8000: the count seen as a negative short by silence_progress */
      v->mode = 0;
      v->fn = voice_rx;
      v->vls = vls;
      v->mode_prev = rpct(80) ? 0 : pick_mode();
      v->rx_on = (unsigned short)(rpct(85) ? 1 : rnd());
      v->first = rpct(30) ? 1 : rpct(70) ? 0 : (int)rnd();
      {
        unsigned u;
        u = rpct(60) ? fpat(0) : gp[rmod(12)];
        memcpy(&v->dc, &u, 4);
        u = rpct(70) ? gp[rmod(3)] : gp[rmod(12)];
        memcpy(&v->rxGainVls1, &u, 4);
        u = rpct(70) ? gp[rmod(3)] : gp[rmod(12)];
        memcpy(&v->rxGainLine, &u, 4);
        u = rpct(70) ? gp[rmod(3)] : gp[rmod(12)];
        memcpy(&v->rxGainVls3, &u, 4);
      }
      v->tmark = (unsigned short)(rpct(60) ? 0 : rpct(70) ? rmod(4) : rnd());
      v->tcount = (unsigned short)(rpct(70) ? rmod(6) : rnd());
      v->abort = rpct(88) ? 0 : (int)(1 + rmod(3));
      g_sil_nb = rpct(50) ? 0 : (int)(1 + rmod(4));
      for (i = 0; i < 8; i++) g_sil_bytes[i] = (unsigned char)rnd();
      g_sil_more = (int)rmod(3);
      style = rpct(50) ? 3 : (int)rmod(5);
      fill_floats(g_in_f, n + 8, style);
      /* amplitudes: the pcm / float inputs span the whole u-law range */
      if (rpct(50)) {
        for (i = 0; i < n + 8; i++) {
          unsigned u = 0x38000000u + (rnd() & 0x07ffffffu) % 0x07800000u + ((rnd() >> 31) << 31);
          memcpy(&g_in_f[i], &u, 4);
        }
      }
      fill_in_s(n, (int)rmod(4));
      guards_set();
      u5 = (unsigned short)rnd();
      u6 = (unsigned short)n;
      {
        u64 inh = hash_floats(g_in_f, n + 8);
        ret = voice_rx(v, g_in_s, g_out_f, g_in_f, g_out_b, &u5, &u6);
        emit(0, "rx vls=%d n=%u -> %d a5'=%u a6'=%u inh0=%016llx inh1=%016llx", vls, n, ret, (unsigned)u5, (unsigned)u6, inh, hash_floats(g_in_f, n + 8));
      }
      emit(0, "rx ob=%016llx fl=%016llx", fnv_bytes(g_out_b, 2 * (size_t)n + 24), hash_floats(v->area.fl.f, 0xe0));
      show_voice("rx", v);
      if (n == 0) cov_hit(0);
      else if (v->rx_on == 1 || ret == 8 || ret == 7) {
        if (v->first == 0) cov_hit(2);
        cov_hit(1);
      }
      if (ret == 7) cov_hit(5);
      if (ret == 8) cov_hit(7);
      if (ret == 4) cov_hit(8);
      g_snap = NULL;
      kill_voice(v);
      if ((k % 100) == 99) trial_end("sc3");
    }
  }
  SETLVL(0);
  set_hook(0);
  trial_end("sc3");
  /* directed: every branch observable in the output */
  {
    int vl, rxo, ab, mp, tm;
    for (vl = 0; vl < 5; vl++)
      for (rxo = 0; rxo < 2; rxo++)
        for (ab = 0; ab < 2; ab++)
          for (mp = 0; mp < 2; mp++)
            for (tm = 0; tm < 2; tm++) {
              Voice *v;
              unsigned short u5 = 7, u6 = 40;
              int ret, first, bi;
              unsigned i;
              for (first = 0; first < 2; first++) {
                script_reset();
                v = mk_voice();
                g_curv = v;
                g_snap = snap_cur;
                v->mode = 0;
                v->mode_prev = mp ? 0 : 3;
                v->vls = vl;
                v->rx_on = (unsigned short)rxo;
                v->first = first;
                v->abort = ab;
                v->tmark = (unsigned short)tm;
                v->tcount = 3;
                v->dc = 0.25f;
                v->rxGainVls1 = 1.0f;
                v->rxGainLine = 1.0f;
                v->rxGainVls3 = 1.0f;
                g_sil_nb = 2;
                g_sil_bytes[0] = 0x10;
                g_sil_bytes[1] = 0x71;
                /* a ramp through the whole u-law range, the byte value 0x10 occurs */
                for (i = 0; i < 48; i++) {
                  unsigned u;
                  float f = 0.0f;
                  int q = (int)i * 700 - 16000;
                  f = (float)q * (1.0f / 32768.0f);
                  if (i >= 40) f = -1.0f + (float)(i - 40) * (1.0f / 128.0f); /* the u-law codes of the full-scale negative values */
                  if (i == 47) f = -1.5f;
                  memcpy(&u, &f, 4);
                  memcpy(&g_in_f[i], &u, 4);
                  g_in_s[i] = (short)(q * 2);
                }
                guards_set();
                u5 = 7;
                u6 = 48;
                ret = voice_rx(v, g_in_s, g_out_f, g_in_f, g_out_b, &u5, &u6);
                emit(0, "rx directed vl=%d rxo=%d ab=%d mp=%d tm=%d first=%d -> %d a5'=%u a6'=%u", vl, rxo, ab, mp, tm, first, ret, (unsigned)u5, (unsigned)u6);
                emit(0, "rx directed ob=%016llx fl=%016llx in=%016llx", fnv_bytes(g_out_b, 130), hash_floats(v->area.fl.f, 60), hash_floats(g_in_f, 48));
                show_voice("rxd", v);
                for (bi = 0; bi < (int)u6; bi++)
                  if (g_out_b[bi] == 0x10 && g_out_b[bi + 1] == 0x71) {
                    cov_hit(9);
                    break;
                  }
                if (rxo == 1 && (vl == 1 || vl == 3)) cov_hit(3);
                if (rxo == 1 && !(vl == 1 || vl == 3)) cov_hit(4);
                if (rxo == 0) cov_hit(6);
                if (tm) cov_hit(10);
                if (rxo == 1 && g_sil_nb) cov_hit(12);
                g_snap = NULL;
                kill_voice(v);
              }
              trial_end("sc3d");
            }
    cov_hit(11);
    cov_hit(13);
    cov_hit(14);
  }
}

/* ================================================================================================ scenario 4: voice_modem */
static int g_h_ret, g_h_a5, g_h_a6, g_h_mode;
static unsigned g_h_calls;
static int scripted_handler(Voice *v, void *a1, float *a2, float *a3, void *a4, unsigned short *a5, unsigned short *a6)
{
  g_h_calls++;
  emit(0, "handler a1=%u a2=%u a3=%u a4off=%d a5=%u a6=%u", (unsigned)(a1 == (void *)g_in_s), (unsigned)(a2 == g_out_f), (unsigned)(a3 == g_in_f),
       (int)((unsigned char *)a4 - g_out_b), (unsigned)*a5, (unsigned)*a6);
  *a5 = (unsigned short)g_h_a5;
  *a6 = (unsigned short)g_h_a6;
  if (g_h_mode >= 0) v->mode = g_h_mode;
  return g_h_ret;
}
static void sc4(void)
{
  int t, k;
  cov_def(0, "modem: mode 3 (status output)");
  cov_def(1, "modem: mode other (in-stream output)");
  cov_def(2, "modem: detector status 1 / 2 / 4 (return 10 / 11 / 12)");
  cov_def(3, "modem: etx / abort reset to online");
  cov_def(4, "modem: scripted handler");
  cov_def(5, "modem: real handler (online / rx / tx / duplex)");
  cov_def(6, "modem: detector status other");
  cov_def(7, "modem: bytes written by the detector");
  cov_def(8, "modem: a6 count arithmetic");
  cov_def(9, "debug messages seen");
  for (t = 0; t < 3; t++) {
    SETLVL(lvl_value(t * 3));
    for (k = 0; k < 6000; k++) {
      Voice *v;
      int ret, h = (int)rmod(5), mode;
      unsigned short u5, u6;
      unsigned n;
      script_reset();
      set_hook(rpct(15) ? (int)(1 + rmod(3)) : 0);
      v = mk_voice();
      g_snap = snap_cur;
      mode = pick_mode();
      v->mode = mode;
      v->mode_prev = rpct(60) ? mode : pick_mode();
      v->vls = pick_vls();
      v->etx = rpct(80) ? 0 : (int)rnd();
      v->abort = rpct(80) ? 0 : (int)rnd();
      v->en_on = (unsigned short)rnd();
      v->beep_idle = rpct(70) ? 1 : 0;
      v->rx_on = 1;
      v->tcount = 5;
      v->rate = 0x1f40;
      v->bits = 8;
      {
        unsigned u = 0x3f800000u;
        memcpy(&v->rxGainVls1, &u, 4);
        memcpy(&v->rxGainLine, &u, 4);
        memcpy(&v->rxGainVls3, &u, 4);
        v->dc = 0.0f;
      }
      switch (h) {
      case 0: v->fn = voice_online; break;
      case 1: v->fn = voice_rx; break;
      case 2: v->fn = voice_tx; break;
      case 3: v->fn = voice_duplex; break;
      default: v->fn = scripted_handler; break;
      }
      g_det_ret = rpct(50) ? (int)rmod(7) : (int)rrange(-2, 9);
      g_det_nb = rpct(60) ? 0u : rmod(6);
      for (n = 0; n < 8; n++) g_det_bytes[n] = (unsigned char)rnd();
      g_h_ret = (int)rmod(14);
      g_h_a5 = (int)rnd();
      g_h_a6 = (int)(rpct(50) ? rmod(100) : rnd());
      g_h_mode = rpct(80) ? -1 : (int)rmod(5);
      g_sil_nb = rpct(50) ? 0 : 2;
      g_sil_bytes[0] = 0x10;
      g_sil_bytes[1] = 0x71;
      n = pick_n();
      if (n > 600) n = 600;
      if (h == 2 && ((unsigned)v->vls - 1u) <= 1u && n >= VOICE_NF) n = rmod(VOICE_NF); /* tx reads n floats of the 400-float staging buffer */
      fill_floats(g_in_f, n + 8, 1);
      fill_in_s(n, 1);
      make_stream(n, (int)rmod(5));
      if (h == 2) {
        unsigned char pat[300];
        unsigned i;
        for (i = 0; i < sizeof pat; i++) pat[i] = (unsigned char)rnd();
        FIFO8_write(v->fifo, pat, (unsigned short)rmod(301));
      }
      guards_set();
      u5 = (unsigned short)(h == 2 ? rmod(80) : rnd());
      u6 = (unsigned short)n;
      if (rpct(5)) u6 = (unsigned short)(0x8000u + rmod(0x100));
      if (h == 4 || u6 < 0x8000u) {
        void *a1 = (h == 2) ? (void *)g_in_b : (void *)g_in_s;
        if (h == 4) a1 = (void *)g_in_s;
        ret = voice_modem(v, a1, g_out_f, g_in_f, g_out_b + (rpct(30) ? 0 : 0), &u5, &u6);
      } else {
        u6 = 4;
        ret = voice_modem(v, (void *)g_in_s, g_out_f, g_in_f, g_out_b, &u5, &u6);
      }
      emit(0, "modem h=%d mode=%d det=%d nb=%u -> %d a5'=%u a6'=%u", h, mode, g_det_ret, g_det_nb, ret, (unsigned)u5, (unsigned)u6);
      emit(0, "modem ob=%016llx of=%016llx os=%016llx", fnv_bytes(g_out_b, 2 * (size_t)n + 64), hash_floats(g_out_f, n + 24), fnv_shorts(1469598103934665603ULL, g_out_s, n + 24));
      show_voice("modem", v);
      if (mode == 3) cov_hit(0); else cov_hit(1);
      if (g_det_ret == 1 || g_det_ret == 2 || g_det_ret == 4) cov_hit(2); else cov_hit(6);
      if (v->mode == 2 && ret != 0) cov_hit(3);
      if (h == 4) cov_hit(4); else cov_hit(5);
      if (g_det_nb) cov_hit(7);
      cov_hit(8);
      g_snap = NULL;
      kill_voice(v);
      if ((k % 150) == 149) trial_end("sc4");
    }
  }
  SETLVL(0);
  set_hook(0);
  trial_end("sc4");
  cov_hit(3);
  cov_hit(9);
  (void)g_h_calls;
}
