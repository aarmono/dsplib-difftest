/*
 * R14 differential harness: FDSP_DP_Create / _Delete / _Run, FDSP_Kernel_InitObj / _SetInternalBeepInProgress, STRM_VCE_GetFDSPEnvironmentalParams, FIFO8_create / write / read / delete:
 * the real dsplibs.o vs src/voice/voice_fdsp.c + src/voice/voice_fifo8.c.  Library test/voice/r14_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation, deterministic heap, recorders
 * sysdep_malloc / free / memset, dsplibs_debug_printf); the typed objects of src/voice/voice_fdsp.h on both sides; the 32-bit BLOB run is the x86-64 golden.
 *
 * Scenarios (each its own process; argv[1]):
 *   0 STRM_VCE_GetFDSPEnvironmentalParams and FDSP_Kernel_SetInternalBeepInProgress over every debug level (a hook can lower the level inside the first message: the second level
 *     read), arbitrary incoming values;
 *   1 FIFO8: create (default cfg / explicit cfg / existing object with its own buffer, sizes 0 .. 32767, fill bytes), write / read with every relation of n to the free space and
 *     the content (wrap, truncation, underrun fill, n = 0, n = 65535), a model of the queue checked on every call, whole object + buffer dumps, delete, malloc failures (guarded);
 *   2 FDSP lifecycle: Kernel_InitObj on dirty objects, Create (NULL = allocate, existing = reinitialise) for the delay grid x every debug level, Delete (all pointers, NULL tails,
 *     NULL hist, NULL obj), Create with each malloc failure point (the blob crashes on points 0 .. 3 and frees an uninitialised pointer on point 4: guarded), Run after Create;
 *   3 FDSP_DP_Run (the x87 code): every int16 input, special floats (+-0, denormals, +-1, the 32768 / 32000 truncation boundaries, huge, inf, qNaN / sNaN payloads), random bit
 *     patterns, every output compared bit for bit, run under the default x87 control word and under precision control = double;
 * R14_VERBOSE=1 prints every line, R14_SEED selects another input stream.
 */
#include "r14_common.h"

enum {
  CV_S_LOW, CV_S_HIGH, CV_S_HOOKDROP, CV_S_HOOKKEEP, CV_B_ON, CV_B_OFF, CV_B_HIGHLVL,
  CV_F_DEFCFG, CV_F_CFG, CV_F_EXIST, CV_F_SIZE0, CV_F_FILL, CV_F_W0, CV_F_WRAP_W, CV_F_WRAP_R, CV_F_TRUNC_W, CV_F_UNDER_R, CV_F_N0_W, CV_F_N0_R, CV_F_NMAX, CV_F_FULL, CV_F_DRAIN,
  CV_F_DELETE, CV_F_FAIL_GUARDED, CV_F_BIGSIZE, CV_F_PARTIAL_R, CV_F_BYTES,
  CV_D_INIT, CV_D_NEW, CV_D_REINIT, CV_D_NEG, CV_D_POS, CV_D_ZERO, CV_D_DBG_VER, CV_D_DBG_REINIT, CV_D_DEL, CV_D_DEL_NULL, CV_D_DEL_NOTAIL, CV_D_DEL_NOHIST, CV_D_FAIL5, CV_D_FAIL_GUARDED,
  CV_D_FAIL_NOFAIL, CV_D_RUN, CV_D_EXT,
  CV_R_ALLI16, CV_R_SPECIAL, CV_R_NAN, CV_R_INF, CV_R_DENORM, CV_R_SAT_POS, CV_R_SAT_NEG, CV_R_INRANGE, CV_R_NEG, CV_R_ZERON, CV_R_BIGN, CV_R_PCDOUBLE, CV_R_MODE, CV_R_NEGZERO, CV_R_RANDBITS,
  CV_R_EDGE_LO, CV_R_EDGE_HI
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_S_LOW, "StrmVCE debug level <= 1 (no text)");
      cov_def(CV_S_HIGH, "StrmVCE debug level > 1 (old text)");
      cov_def(CV_S_HOOKDROP, "StrmVCE level lowered inside the old text (no new text)");
      cov_def(CV_S_HOOKKEEP, "StrmVCE new text");
      cov_def(CV_B_ON, "SetInternalBeepInProgress on");
      cov_def(CV_B_OFF, "SetInternalBeepInProgress off");
      cov_def(CV_B_HIGHLVL, "SetInternalBeepInProgress with text");
      break;
    case 1:
      cov_def(CV_F_DEFCFG, "FIFO8_create cfg NULL");
      cov_def(CV_F_CFG, "FIFO8_create explicit cfg");
      cov_def(CV_F_EXIST, "FIFO8_create existing object");
      cov_def(CV_F_SIZE0, "size 0");
      cov_def(CV_F_FILL, "nonzero fill");
      cov_def(CV_F_W0, "nonzero w0");
      cov_def(CV_F_WRAP_W, "write wraps");
      cov_def(CV_F_WRAP_R, "read wraps");
      cov_def(CV_F_TRUNC_W, "write truncated (n > free)");
      cov_def(CV_F_UNDER_R, "read underrun (n > count)");
      cov_def(CV_F_N0_W, "write n = 0");
      cov_def(CV_F_N0_R, "read n = 0");
      cov_def(CV_F_NMAX, "n = 65535");
      cov_def(CV_F_FULL, "write fills the ring exactly");
      cov_def(CV_F_DRAIN, "read drains the ring exactly");
      cov_def(CV_F_DELETE, "FIFO8_delete");
      cov_def(CV_F_FAIL_GUARDED, "create malloc failure (guarded)");
      cov_def(CV_F_BIGSIZE, "size 32767");
      cov_def(CV_F_PARTIAL_R, "partial read (0 < n < count)");
      cov_def(CV_F_BYTES, "bytes moved");
      break;
    case 2:
      cov_def(CV_D_INIT, "FDSP_Kernel_InitObj on a dirty object");
      cov_def(CV_D_NEW, "Create obj NULL");
      cov_def(CV_D_REINIT, "Create existing object");
      cov_def(CV_D_NEG, "Create rx delay < 0 (mode 0)");
      cov_def(CV_D_POS, "Create rx delay > 0 (mode 2)");
      cov_def(CV_D_ZERO, "Create rx delay == 0");
      cov_def(CV_D_DBG_VER, "version text");
      cov_def(CV_D_DBG_REINIT, "reinitialisation text");
      cov_def(CV_D_DEL, "FDSP_DP_Delete");
      cov_def(CV_D_DEL_NULL, "Delete NULL");
      cov_def(CV_D_DEL_NOTAIL, "Delete with NULL tails");
      cov_def(CV_D_DEL_NOHIST, "Delete with NULL hist");
      cov_def(CV_D_FAIL5, "Create: 6th malloc fails (compared)");
      cov_def(CV_D_FAIL_GUARDED, "Create: malloc 0 .. 4 fails (guarded)");
      cov_def(CV_D_FAIL_NOFAIL, "Create: failure point never reached");
      cov_def(CV_D_RUN, "Run on a created object");
      cov_def(CV_D_EXT, "extreme delays");
      break;
    case 3:
      cov_def(CV_R_ALLI16, "every int16 input");
      cov_def(CV_R_SPECIAL, "special floats");
      cov_def(CV_R_NAN, "NaN input");
      cov_def(CV_R_INF, "infinite input");
      cov_def(CV_R_DENORM, "denormal input");
      cov_def(CV_R_SAT_POS, "positive overflow -> -32768");
      cov_def(CV_R_SAT_NEG, "negative overflow -> -32768");
      cov_def(CV_R_INRANGE, "in-range truncation");
      cov_def(CV_R_NEG, "negative in-range");
      cov_def(CV_R_ZERON, "n == 0");
      cov_def(CV_R_BIGN, "n == 65535");
      cov_def(CV_R_PCDOUBLE, "precision control = double pass");
      cov_def(CV_R_MODE, "obj->mode forced to 2");
      cov_def(CV_R_NEGZERO, "-0.0 input");
      cov_def(CV_R_RANDBITS, "random bit patterns");
      cov_def(CV_R_EDGE_LO, "value just inside -32768");
      cov_def(CV_R_EDGE_HI, "value just inside 32768");
      break;
    default: break;
  }
}

/* ---------------------------------------------------------------------------------------------- helpers */
static const char *pid(const void *p, char *buf, size_t cap)
{
  if (!p) snprintf(buf, cap, "NULL");
  else {
    unsigned id = blk_id_of(p);
    if (id) snprintf(buf, cap, "A%u", id);
    else snprintf(buf, cap, "WILD");
  }
  return buf;
}
static void fill_rand(void *p, size_t n)
{
  unsigned char *b = (unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) b[i] = (unsigned char)rnd();
}
static u64 fnv_i32(const int32_t *p, unsigned n) { return fnv_ints((const int *)p, n); }
static unsigned fbits(float f)
{
  unsigned u;
  memcpy(&u, &f, 4);
  return u;
}

/* run fn in a child: 0 = returned normally, 1 = killed by a signal, 3 = harness violation (die3), 4 = leak */
static int guarded(void (*fn)(void *), void *arg)
{
  int st = 0;
  pid_t c;
  fflush(stdout);
  c = fork();
  if (c < 0) die3("fork");
  if (c == 0) {
    fn(arg);
    if (g_live != 0) _exit(4);
    _exit(0);
  }
  if (waitpid(c, &st, 0) != c) die3("waitpid");
  if (WIFSIGNALED(st)) return 1;
  return WEXITSTATUS(st);
}
/* the blob crashes where the port returns NULL: both are "handled" (a port crash or leak is a violation) */
static void guarded_report(const char *what, int r)
{
  if (g_verbose) fprintf(stderr, "guarded %s: child status %d\n", what, r);
#ifdef TEST_BLOB
  if (r != 0 && r != 1 && r != 3) die3("%s: blob child status %d", what, r);
#else
  if (r != 0) die3("%s: the port child failed (status %d)", what, r);
#endif
  emit(1, "GUARDED %s handled", what);
}

/* ---------------------------------------------------------------------------------------------- scenario 0 */
static unsigned g_hook_to;
static int g_hook_on;
static void hook0(const char *msg)
{
  if (g_hook_on && strstr(msg, "StrmVCE old")) dsplibs_debug_level = g_hook_to;
}
static void scen0(void)
{
  int k, j, hk;
  short a, b;
  static const short vals[] = {0, 1, -1, 0x33, 0x171, 32767, -32768, 12345, -12345};
  g_dbg_hook = hook0;
  for (k = 0; k < 7; k++)
    for (hk = 0; hk < 4; hk++)
      for (j = 0; j < 14; j++) {
        a = j < 9 ? vals[j] : (short)rnd();
        b = j < 9 ? vals[8 - j] : (short)rnd();
        dsplibs_debug_level = lvl_value(k);
        g_hook_on = hk != 0;
        g_hook_to = hk == 1 ? 0u : (hk == 3 ? 1u : 7u); /* 3: lowered to exactly 1 (the boundary of the second level test) */
        STRM_VCE_GetFDSPEnvironmentalParams(&a, &b);
        if (lvl_value(k) > 1) {
          cov_hit(CV_S_HIGH);
          if (hk == 1 || hk == 3) cov_hit(CV_S_HOOKDROP);
          else cov_hit(CV_S_HOOKKEEP);
        } else cov_hit(CV_S_LOW);
        emit(1, "STRM lvl=%u hk=%d -> a=%d b=%d", lvl_value(k), hk, a, b);
        g_hook_on = 0;
        dsplibs_debug_level = lvl_value(k);
      }
  for (k = 0; k < 7; k++)
    for (j = 0; j < 8; j++) {
      static const int ons[] = {0, 1, -1, 2, 0x7fffffff, (int)0x80000000, 0x100, 0};
      dsplibs_debug_level = lvl_value(k);
      FDSP_Kernel_SetInternalBeepInProgress(ons[j]);
      cov_hit(ons[j] ? CV_B_ON : CV_B_OFF);
      if (lvl_value(k) > 1) cov_hit(CV_B_HIGHLVL);
      emit(1, "BEEP lvl=%u on=%d", lvl_value(k), ons[j]);
    }
  dsplibs_debug_level = 0;
  trial_end("s0");
}

/* ---------------------------------------------------------------------------------------------- scenario 1: FIFO8 */
typedef struct {
  unsigned char q[70000];
  unsigned head, n, size;
} Model;
static unsigned char g_src[70000], g_dst[70000], g_dst2[70000];

static void fifo_dump(const char *tag, FifoObj *f)
{
  char b1[24], b2[24];
  unsigned size = f->size;
  emit(1, "%s F=%s buf=%s w0=%u size=%u fill=%u count=%u rd=%u wr=%u bufhash=%016llx", tag, pid(f, b1, sizeof b1), pid(FIFO_BUF(f), b2, sizeof b2), f->w0, f->size, f->fill, f->count, f->rd, f->wr,
       fnv_bytes(FIFO_BUF(f), size));
}
static FifoObj *fifo_make_existing(unsigned bufsize)
{
  FifoObj *f = (FifoObj *)sysdep_malloc((unsigned)sizeof(FifoObj));
  uint8_t *b = (uint8_t *)sysdep_malloc(bufsize);
  fill_rand(b, bufsize);
  f->w0 = (uint16_t)rnd();
  f->size = (uint16_t)rnd();
  f->fill = (uint16_t)rnd();
  f->count = (uint16_t)rnd();
  f->rd = (uint16_t)rnd();
  f->wr = (uint16_t)rnd();
  FIFO_BUF(f) = b;
  return f;
}
typedef struct {
  FifoObj *f;
  const FifoCfg *cfg;
} CreateArg;
static void create_child(void *p)
{
  CreateArg *a = (CreateArg *)p;
  FifoObj *f = FIFO8_create(NULL, a->cfg);
  if (f != NULL) _exit(5);
}
static void fifo_trial(const FifoCfg *cfg, int exist, int nops)
{
  FifoObj *f;
  Model m;
  unsigned size = cfg ? cfg->size : 300, ops;
  unsigned fillb = cfg ? (cfg->fill & 0xff) : 0;
  char b1[24];
  if (exist) f = fifo_make_existing(size + 1000u + (unsigned)rmod(50)); /* away from the sizeof() values the recorder maps */
  else f = NULL;
  f = FIFO8_create(f, cfg);
  if (!f) die3("FIFO8_create returned NULL");
  cov_hit(cfg ? CV_F_CFG : CV_F_DEFCFG);
  if (exist) cov_hit(CV_F_EXIST);
  if (size == 0) cov_hit(CV_F_SIZE0);
  if (size == 32767) cov_hit(CV_F_BIGSIZE);
  if (cfg && cfg->fill) cov_hit(CV_F_FILL);
  if (cfg && cfg->w0) cov_hit(CV_F_W0);
  fifo_dump("create", f);
  if (f->size != size || f->count != 0 || f->rd != 0 || f->wr != 0) die3("create header");
  {
    unsigned i;
    for (i = 0; i < size; i++)
      if (FIFO_BUF(f)[i] != 0) die3("create did not zero the buffer");
  }
  m.head = 0;
  m.n = 0;
  m.size = size;
  for (ops = 0; ops < (unsigned)nops; ops++) {
    int wr = rpct(50);
    unsigned free_sp = size - m.n, cnt = m.n, n, k;
    unsigned sel = rmod(12);
    if (sel == 0) n = 0;
    else if (sel == 1) n = 1;
    else if (sel == 2) n = free_sp;
    else if (sel == 3) n = cnt;
    else if (sel == 4) n = (wr ? free_sp : cnt) + 1 + rmod(5);
    else if (sel == 5) n = rmod(size ? size : 4) + 1;
    else if (sel == 6) n = rmod(70) ;
    else if (sel == 7) n = (wr ? free_sp : cnt) ? (unsigned)rmod(wr ? free_sp : cnt) : 0;
    else if (sel == 8 && rpct(20)) n = 65535;
    else n = rmod(size / 2 + 3);
    if (n > 65535) n = 65535;
    if (wr) {
      int got;
      unsigned want = n > free_sp ? free_sp : n;
      fill_rand(g_src, n);
      got = FIFO8_write(f, g_src, (unsigned short)n);
      if ((unsigned)got != want) die3("write returned %d, model %u", got, want);
      for (k = 0; k < want; k++) m.q[(m.head + m.n + k) % (size ? size : 1)] = g_src[k];
      m.n += want;
      if (n == 0) cov_hit(CV_F_N0_W);
      if (n > free_sp) cov_hit(CV_F_TRUNC_W);
      if (n == 65535) cov_hit(CV_F_NMAX);
      if (want && m.n == size) cov_hit(CV_F_FULL);
      if (want && f->wr < want && size) cov_hit(CV_F_WRAP_W);
      cov_hit(CV_F_BYTES);
      emit(0, "W n=%u -> %d", n, got);
    } else {
      int got;
      unsigned want = n > cnt ? cnt : n;
      memset(g_dst, 0x77, 70000);
      got = FIFO8_read(f, g_dst, (unsigned short)n);
      if ((unsigned)got != want) die3("read returned %d, model %u", got, want);
      for (k = 0; k < want; k++)
        if (g_dst[k] != m.q[(m.head + k) % size]) die3("read byte %u differs from the model", k);
      for (k = want; k < n; k++)
        if (g_dst[k] != fillb) die3("read fill byte %u: %u, model %u", k, g_dst[k], fillb);
      if (g_dst[n] != 0x77) die3("read wrote past n");
      if (want && f->rd < want && size) cov_hit(CV_F_WRAP_R);
      m.head = size ? (m.head + want) % size : 0;
      m.n -= want;
      if (n == 0) cov_hit(CV_F_N0_R);
      if (n > cnt) cov_hit(CV_F_UNDER_R);
      if (n == 65535) cov_hit(CV_F_NMAX);
      if (want && m.n == 0) cov_hit(CV_F_DRAIN);
      if (want && n < cnt) cov_hit(CV_F_PARTIAL_R);
      cov_hit(CV_F_BYTES);
      emit(0, "R n=%u -> %d out=%016llx", n, got, fnv_bytes(g_dst, n));
    }
    if (ops % 7 == 0 || ops + 1 == (unsigned)nops) fifo_dump(wr ? "write" : "read ", f);
    else emit(0, "  state F=%s count=%u rd=%u wr=%u", pid(f, b1, sizeof b1), f->count, f->rd, f->wr);
  }
  FIFO8_delete(f);
  cov_hit(CV_F_DELETE);
  trial_end("fifo");
}
static void scen1(void)
{
  static const unsigned sizes[] = {0, 1, 2, 3, 5, 7, 8, 9, 15, 17, 23, 25, 31, 33, 100, 255, 256, 257, 300, 1000, 4096, 32767};
  unsigned i, t;
  FifoCfg cfg;
  /* default config */
  for (t = 0; t < 12; t++) fifo_trial(NULL, t & 1, 200);
  for (i = 0; i < sizeof sizes / sizeof sizes[0]; i++)
    for (t = 0; t < 6; t++) {
      cfg.w0 = t == 1 ? (uint16_t)rnd() : 0;
      cfg.size = (uint16_t)sizes[i];
      cfg.fill = t == 0 ? 0 : (t == 2 ? 0xffff : (uint16_t)rnd());
      fifo_trial(&cfg, t & 1, sizes[i] > 4096 ? 60 : (sizes[i] > 300 ? 150 : 300));
    }
  for (t = 0; t < 40; t++) {
    cfg.w0 = (uint16_t)rnd();
    do cfg.size = (uint16_t)(1 + rmod(70)); while (cfg.size == 24 || cfg.size == 28 || cfg.size == 32 || cfg.size == 40);
    cfg.fill = (uint16_t)rnd();
    fifo_trial(&cfg, rpct(40), 120);
  }
  /* malloc failures: the blob dereferences NULL */
  for (t = 0; t < 2; t++) {
    CreateArg a;
    a.f = NULL;
    a.cfg = NULL;
    g_fail_malloc = (int)t;
    g_malloc_ctr = 0;
    guarded_report(t == 0 ? "FIFO8_create malloc#0" : "FIFO8_create malloc#1", guarded(create_child, &a));
    g_fail_malloc = -1;
    g_malloc_ctr = 0;
    cov_hit(CV_F_FAIL_GUARDED);
  }
  trial_end("s1");
}

/* ---------------------------------------------------------------------------------------------- scenario 2: FDSP lifecycle */
static void chan_dump(const char *tag, FdspChan *c)
{
  char b1[24], b2[24];
  int32_t *t = CH_TAIL(c);
  unsigned char gaps[8];
  memcpy(gaps, &c->unk_1684, 4);
  memcpy(gaps + 4, &c->unk_168a, 2);
  memcpy(gaps + 6, &c->unk_1692, 2);
  emit(1, "  %s %s w=%016llx gap=%016llx gaps=%016llx tail=%s tailh=%016llx delay=%d gain=%08x s1690=%d v1694=%d,%d,%d,%d v16a4=%d", tag, pid(c, b1, sizeof b1), fnv_i32(c->w, 0x500), fnv_bytes(c->unk_1400, sizeof c->unk_1400), fnv_bytes(gaps, 8), pid(t, b2, sizeof b2), t ? fnv_i32(t, 0xf0) : 0ull, c->delay,
       fbits(c->gain), c->s1690, c->v1694[0], c->v1694[1], c->v1694[2], c->v1694[3], c->v16a4);
}
static void fdsp_dump(const char *tag, FdspObj *o)
{
  char b1[24], b2[24], b3[24], b4[24];
  emit(1, "%s obj=%s mode=%d w4=%d w8=%d wc=%d hist=%s ch14=%s ch18=%s", tag, pid(o, b1, sizeof b1), o->mode, o->w4, o->w8, o->wc, pid(o->hist, b2, sizeof b2), pid(o->ch14, b3, sizeof b3), pid(o->ch18, b4, sizeof b4));
  if (o->hist) emit(1, "  hist x=%016llx y=%016llx gap=%016llx z=%d,%d,%d", fnv_shorts((const short *)o->hist->x, 2000), fnv_shorts((const short *)o->hist->y, 2000), fnv_bytes(o->hist->unk_1f40, sizeof o->hist->unk_1f40), o->hist->z[0], o->hist->z[1], o->hist->z[2]);
  if (o->ch14) chan_dump("ch14", o->ch14);
  if (o->ch18) chan_dump("ch18", o->ch18);
}
static FdspObj *fdsp_make(int dirty, int tails, int hist)
{
  FdspObj *o = (FdspObj *)sysdep_malloc((unsigned)sizeof(FdspObj));
  FdspChan *c, *b;
  FdspHist *h = NULL;
  o->mode = (int)rnd();
  o->w4 = (int)rnd();
  o->w8 = (int)rnd();
  o->wc = (int)rnd();
  if (hist) {
    h = (FdspHist *)sysdep_malloc((unsigned)sizeof(FdspHist));
    if (dirty) {
      fill_rand(h->x, sizeof h->x);
      fill_rand(h->y, sizeof h->y);
      fill_rand(h->z, sizeof h->z);
    }
  }
  o->hist = h;
  c = (FdspChan *)sysdep_malloc((unsigned)sizeof(FdspChan));
  b = (FdspChan *)sysdep_malloc((unsigned)sizeof(FdspChan));
  CH_TAIL(c) = tails ? (int32_t *)sysdep_malloc(FDSP_TAIL_BLOB_SIZE) : NULL;
  CH_TAIL(b) = tails ? (int32_t *)sysdep_malloc(FDSP_TAIL_BLOB_SIZE) : NULL;
  if (dirty) {
    unsigned k;
    fill_rand(c->w, sizeof c->w);
    fill_rand(b->w, sizeof b->w);
    if (tails) {
      fill_rand(CH_TAIL(c), FDSP_TAIL_BLOB_SIZE);
      fill_rand(CH_TAIL(b), FDSP_TAIL_BLOB_SIZE);
    }
    c->delay = (short)rnd();
    b->delay = (short)rnd();
    c->gain = (float)(int)rnd();
    b->gain = (float)(int)rnd();
    c->s1690 = (short)rnd();
    b->s1690 = (short)rnd();
    for (k = 0; k < 4; k++) {
      c->v1694[k] = (int)rnd();
      b->v1694[k] = (int)rnd();
    }
    c->v16a4 = (int)rnd();
    b->v16a4 = (int)rnd();
  }
  o->ch14 = c;
  o->ch18 = b;
  return o;
}
typedef struct {
  short a, b;
} CArg;
static void create_fdsp_child(void *p)
{
  CArg *a = (CArg *)p;
  FdspObj *o = FDSP_DP_Create(NULL, a->a, a->b);
  if (o != NULL) _exit(5);
}
static const short DEL[] = {0, 1, -1, 0x33, 0x171, 100, 32767, -32768, 12345, -300, 2, 80};
#define NDEL ((int)(sizeof DEL / sizeof DEL[0]))
static void scen2(void)
{
  int i, j, k, lv;
  FdspObj *o;
  /* Kernel_InitObj on dirty objects */
  for (i = 0; i < 6; i++) {
    o = fdsp_make(1, 1, 1);
    FDSP_Kernel_InitObj(o);
    cov_hit(CV_D_INIT);
    fdsp_dump("init", o);
    FDSP_DP_Delete(o);
    trial_end("init");
  }
  /* Create (allocate) over the grid x debug levels; Run; Delete */
  for (i = 0; i < NDEL; i++)
    for (j = 0; j < NDEL; j += (i % 3) + 1) {
      lv = (i * 5 + j) % 7;
      dsplibs_debug_level = lvl_value(lv);
      o = FDSP_DP_Create(NULL, DEL[i], DEL[j]);
      dsplibs_debug_level = 0;
      if (!o) die3("Create returned NULL");
      cov_hit(CV_D_NEW);
      cov_hit(DEL[i] < 0 ? CV_D_NEG : (DEL[i] > 0 ? CV_D_POS : CV_D_ZERO));
      if (DEL[i] == 32767 || DEL[i] == -32768 || DEL[j] == 32767 || DEL[j] == -32768) cov_hit(CV_D_EXT);
      if (lvl_value(lv) > 1) cov_hit(CV_D_DBG_VER);
      fdsp_dump("create", o);
      if (i % 4 == 0) {
        short in1[8] = {1, -2, 300, -32768, 32767, 0, 5, -5};
        float o1[8], in2[8] = {0.5f, -0.5f, 1.0f, -1.0f, 1.2f, 0.0f, 1e9f, -1e9f};
        short o2[8];
        unsigned short n = 8;
        o->mode = (int)rnd();
        if (FDSP_DP_Run(o, in1, o1, in2, o2, NULL, &n) != 1) die3("Run");
        cov_hit(CV_D_RUN);
        emit(1, "  run mode=%d o1=%016llx o2=%016llx", o->mode, fnv_bytes(o1, sizeof o1), fnv_shorts(o2, 8));
      }
      FDSP_DP_Delete(o);
      cov_hit(CV_D_DEL);
      trial_end("create");
    }
  /* Create / reinitialise an existing dirty object */
  for (i = 0; i < NDEL; i++) {
    o = fdsp_make(1, 1, 1);
    lv = i % 7;
    dsplibs_debug_level = lvl_value(lv);
    {
      FdspObj *r = FDSP_DP_Create(o, DEL[i], (short)rnd());
      dsplibs_debug_level = 0;
      if (r != o) die3("reinit returned another object");
    }
    cov_hit(CV_D_REINIT);
    cov_hit(DEL[i] < 0 ? CV_D_NEG : (DEL[i] > 0 ? CV_D_POS : CV_D_ZERO));
    if (lvl_value(lv) > 1) {
      cov_hit(CV_D_DBG_VER);
      cov_hit(CV_D_DBG_REINIT);
    }
    fdsp_dump("reinit", o);
    FDSP_DP_Delete(o);
    trial_end("reinit");
  }
  /* Delete variants */
  FDSP_DP_Delete(NULL);
  cov_hit(CV_D_DEL_NULL);
  for (k = 0; k < 4; k++) {
    o = fdsp_make(0, k & 1 ? 0 : 1, k & 2 ? 0 : 1);
    if (k & 1) cov_hit(CV_D_DEL_NOTAIL);
    if (k & 2) cov_hit(CV_D_DEL_NOHIST);
    if (k == 1) { /* only the rx tail missing */
      CH_TAIL(o->ch14) = NULL;
      CH_TAIL(o->ch18) = (int32_t *)sysdep_malloc(FDSP_TAIL_BLOB_SIZE);
    }
    FDSP_DP_Delete(o);
    cov_hit(CV_D_DEL);
    trial_end("delete");
  }
  /* malloc failure points */
  for (k = 0; k <= 7; k++) {
    CArg a;
    a.a = (short)(k * 7 + 3);
    a.b = (short)(-k);
    g_fail_malloc = k;
    g_malloc_ctr = 0;
    if (k <= 4) {
      char tag[40];
      snprintf(tag, sizeof tag, "FDSP_DP_Create malloc#%d", k);
      guarded_report(tag, guarded(create_fdsp_child, &a));
      cov_hit(CV_D_FAIL_GUARDED);
      g_fail_malloc = -1;
      g_malloc_ctr = 0;
    } else {
      o = FDSP_DP_Create(NULL, a.a, a.b);
      if (k == 5) {
        cov_hit(CV_D_FAIL5);
        if (o) die3("malloc#5 failure: Create returned an object");
        emit(1, "create malloc#5 failed -> NULL");
      } else {
        cov_hit(CV_D_FAIL_NOFAIL);
        if (!o) die3("failure point %d never reached but Create returned NULL", k);
        fdsp_dump("create-nofail", o);
        FDSP_DP_Delete(o);
      }
      g_fail_malloc = -1;
      g_malloc_ctr = 0;
      trial_end("fail");
    }
  }
  trial_end("s2");
}

/* ---------------------------------------------------------------------------------------------- scenario 3: FDSP_DP_Run */
#define MAXN 65535
static short g_in1[MAXN];
static float g_in2[MAXN], g_o1[MAXN + 16];
static short g_o2[MAXN + 16];
static float f_from_bits(unsigned u)
{
  float f;
  memcpy(&f, &u, 4);
  return f;
}
static void set_pc(int dbl)
{
#if defined(__i386__) || defined(__x86_64__)
  unsigned short cw;
  __asm__ volatile("fnstcw %0" : "=m"(cw));
  cw = (unsigned short)((cw & ~0x300) | (dbl ? 0x200 : 0x300));
  __asm__ volatile("fldcw %0" : : "m"(cw));
#else
  (void)dbl;
#endif
}
static void run_block(FdspObj *o, unsigned n, const char *tag)
{
  unsigned short nn = (unsigned short)n;
  u64 h1, h2;
  unsigned i;
  for (i = 0; i < 16; i++) {
    g_o1[n + i] = f_from_bits(0xabcdef01u);
    g_o2[n + i] = (short)0x5a5a;
  }
  o->mode = (int)rnd() | 1;
  if (FDSP_DP_Run(o, g_in1, g_o1, g_in2, g_o2, NULL, &nn) != 1) die3("Run result");
  if (o->mode != 2) die3("Run mode %d", o->mode);
  cov_hit(CV_R_MODE);
  for (i = 0; i < 16; i++)
    if (fbits(g_o1[n + i]) != 0xabcdef01u || g_o2[n + i] != (short)0x5a5a) die3("Run wrote past n");
  h1 = fnv_bytes(g_o1, (size_t)n * 4);
  h2 = fnv_shorts(g_o2, n);
  emit(1, "%s n=%u o1=%016llx o2=%016llx", tag, n, h1, h2);
}
static void cov_floats(unsigned n)
{
  unsigned i;
  for (i = 0; i < n; i++) {
    float x = g_in2[i];
    unsigned u = fbits(x);
    double p = (double)x * 32000.0;
    if (x != x) cov_hit(CV_R_NAN);
    else if ((u & 0x7fffffffu) == 0x7f800000u) cov_hit(CV_R_INF);
    else {
      if ((u & 0x7f800000u) == 0 && (u & 0x7fffffu)) cov_hit(CV_R_DENORM);
      if (u == 0x80000000u) cov_hit(CV_R_NEGZERO);
      if (p >= 32768.0) cov_hit(CV_R_SAT_POS);
      else if (p <= -32768.0) cov_hit(CV_R_SAT_NEG);
      else {
        cov_hit(CV_R_INRANGE);
        if (p < 0) cov_hit(CV_R_NEG);
        if (p > 32767.0) cov_hit(CV_R_EDGE_HI);
        if (p < -32767.0) cov_hit(CV_R_EDGE_LO);
      }
    }
  }
}
static void scen3(void)
{
  FdspObj *o = FDSP_DP_Create(NULL, 3, 4);
  unsigned i, pass, n, k;
  static const unsigned special[] = {
      0x00000000, 0x80000000, 0x00000001, 0x80000001, 0x007fffff, 0x807fffff, 0x00800000, 0x80800000, 0x3f800000, 0xbf800000, 0x3f7fffff, 0xbf7fffff, 0x3f800001, 0xbf800001,
      0x3c23d70a /* 0.01 */, 0x7f7fffff, 0xff7fffff, 0x7f800000, 0xff800000, 0x7fc00000, 0xffc00000, 0x7f800001, 0xff800001, 0x7fffffff, 0xffffffff, 0x7fc12345, 0x7f812345,
      0x38d1b717, 0x46fa0000, 0xc6fa0000, 0x47000000, 0xc7000000, 0x00400000, 0x80400000, 0x3a83126f, 0x4b000000, 0x4b7fffff, 0x4efffffc, 0x5f000000, 0xdf000000};
  /* 32768 / 32000 = 1.024 and its float neighbours; the integer boundaries of the truncation */
  static const double edge[] = {1.024, -1.024, 1.0239999, -1.0239999, 1.0240001, -1.0240001, 1.0, 0.99999, 1.00003125, 1.00003126, 3.1249999e-5, 3.125e-5, 3.1250002e-5, 2e-4, 5e-5};
  for (pass = 0; pass < 2; pass++) {
    set_pc(pass);
    if (pass) cov_hit(CV_R_PCDOUBLE);
    emit(1, "== pass %u (precision control %s)", pass, pass ? "double" : "default");
    /* every int16 input (in two chunks to check n boundaries) */
    for (i = 0; i < MAXN; i++) g_in1[i] = (short)(i - 32768); /* -32768 .. 32766 */
    run_block(o, MAXN, "alli16-a");
    cov_hit(CV_R_BIGN);
    g_in1[0] = 32767;
    run_block(o, 1, "alli16-b");
    cov_hit(CV_R_ALLI16);
    /* the float conversion with the same data */
    for (i = 0; i < MAXN; i++) g_in2[i] = (float)(short)(i - 32768) / 32000.0f * (1.0f + 1e-7f * (float)(i & 7));
    cov_floats(MAXN);
    run_block(o, MAXN, "ramp");
    /* special values at every position (each alone and in mixed blocks) */
    for (k = 0; k < sizeof special / sizeof special[0]; k++) {
      for (i = 0; i < 8; i++) g_in2[i] = f_from_bits(special[k]);
      n = 1 + (k % 8);
      cov_floats(n);
      cov_hit(CV_R_SPECIAL);
      run_block(o, n, "special");
    }
    for (i = 0; i < sizeof special / sizeof special[0]; i++) g_in2[i] = f_from_bits(special[i]);
    cov_floats(sizeof special / sizeof special[0]);
    run_block(o, (unsigned)(sizeof special / sizeof special[0]), "special-mix");
    /* the boundary values +- their float neighbours */
    n = 0;
    for (i = 0; i < sizeof edge / sizeof edge[0]; i++) {
      float f = (float)edge[i];
      int d;
      for (d = -3; d <= 3; d++) {
        unsigned u = fbits(f) + (unsigned)d;
        g_in2[n++] = f_from_bits(u);
      }
    }
    for (i = 0; i < 64; i++) {
      float f = (float)(32767 + (int)i / 4) / 32000.0f; /* the truncation boundary at 32767 / 32768 */
      g_in2[n++] = f_from_bits(fbits(f) + (unsigned)((int)(i % 4) - 2));
      g_in2[n++] = -f_from_bits(fbits(f) + (unsigned)((int)(i % 4) - 2));
    }
    cov_floats(n);
    run_block(o, n, "edges");
    /* integer boundaries: x * 32000 just below / above every integer near 0, +-1, +-2 .. and the 0.5 ties */
    n = 0;
    for (i = 0; i < 400; i++) {
      float f = (float)((double)((int)i - 200) / 2.0 / 32000.0);
      int d;
      for (d = -1; d <= 1; d++) g_in2[n++] = f_from_bits(fbits(f) + (unsigned)d);
    }
    cov_floats(n);
    run_block(o, n, "ties");
    /* random bit patterns, random n */
    for (k = 0; k < 40; k++) {
      n = k == 0 ? 0 : (k == 1 ? 1 : (k < 6 ? (unsigned)rmod(10) : (unsigned)rmod(3000)));
      if (k == 0) cov_hit(CV_R_ZERON);
      for (i = 0; i < n; i++) {
        unsigned u = rnd();
        switch (k % 4) {
          case 1: u = (u & 0x807fffffu) | ((0x76u + rmod(20)) << 23); break; /* around 1e-4 .. 1e2 magnitudes */
          case 2: u = (u & 0x807fffffu) | ((0x7eu + rmod(3)) << 23); break;   /* 0.5 .. 2 */
          case 3: u = (u & 0x80ffffffu); break;                                /* denormal / tiny exponents */
          default: break;
        }
        g_in2[i] = f_from_bits(u);
        g_in1[i] = (short)rnd();
      }
      cov_floats(n);
      cov_hit(CV_R_RANDBITS);
      run_block(o, n, "random");
    }
  }
  set_pc(0);
  FDSP_DP_Delete(o);
  trial_end("s3");
}

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : -1;
  const char *e;
  if (scen < 0 || scen > 3) {
    fprintf(stderr, "usage: %s scenario(0..3)\n", argv[0]);
    return 2;
  }
  g_verbose = (e = getenv("R14_VERBOSE")) != NULL && *e == '1';
  if ((e = getenv("R14_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(e, NULL, 0);
  g_lcg += (unsigned)scen * 0x9e3779b9u;
  cov_setup(scen);
  switch (scen) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    default: break;
  }
  cov_finish();
  return 0;
}
