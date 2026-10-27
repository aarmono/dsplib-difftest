/*
 * R06 engine harness pieces shared by test/v22/v22_engine.main.c and test/v22/dp_v22.e2e.main.c (header only; includes test/v22/r06_common.h): the layout independent description of a V22Engine
 * (every member of the engine, tone and core blocks and of the embedded PPS / MRF / SRE / FSE / SMC / SDM / AGC objects, printed by NAME: pointers become block ids / table codes /
 * contents, never addresses or offsets), the scripted recorders of the seven R07 / R08 handlers V22FP_modem dispatches to (v22_data ... v22_retrain: still blob functions, here the
 * harness' own on BOTH sides: the blob's copies are made weak with objcopy), and engine creation helpers.
 */
#ifndef R06_ENG_COMMON_H
#define R06_ENG_COMMON_H
#include "r06_common.h"
#include "fpm/fpm.h"
#include "fpm/fpm_tables.h"

/* ------------------------------------------------------------------------------------------------ pointer identity */
static unsigned blk_code(const void *p)
{
  const Blk *b;
  if (!p) return 0;
  b = find_blk(p);
  if (b) return 0x10000u + b->id * 0x400u + (unsigned)(((const unsigned char *)p - b->p) / 2u);
  return 0xffffu;
}
/* the table a pointer member points to, by identity of the tables the engine installs (the blob's tables are GLOBAL symbols of the same name) */
static unsigned tbl_code(const void *p)
{
  if (!p) return 0;
  if (p == (const void *)SMCv22_IMAP_1200BPS) return 1;
  if (p == (const void *)SMCv22_QMAP_1200BPS) return 2;
  if (p == (const void *)SMCv22_IMAP_2400BPS) return 3;
  if (p == (const void *)SMCv22_QMAP_2400BPS) return 4;
  if (p == (const void *)SMCv22_PMAP) return 5;
  if (p == (const void *)FSEv22_decision12) return 6;
  if (p == (const void *)FSEv22_decision24) return 7;
  return blk_code(p) ? blk_code(p) : 0xffffu;
}

/* ------------------------------------------------------------------------------------------------ the description of an engine */
static void d_agc(int show, const char *tag, const FpmAgc *a)
{
  emit(show, "%s a=%d,%d,%d w5=%d active=%d detect=%d hist=%d s22=%d rcp=%d shift=%d freeze=%d alpha=%d,%d beta=%d,%d", tag, a->a.w[0], a->a.w[1], a->a.w[2], a->w5, a->active, a->detect, a->hist,
       a->s22, a->rcp, a->shift, a->freeze, a->alpha ? a->alpha[0] : -1, a->alpha ? a->alpha[1] : -1, a->beta ? a->beta[0] : -1, a->beta ? a->beta[1] : -1);
}
static void d_mtd(int show, const char *tag, const FpmMtd *m, int ncoef)
{
  emit(show, "%s c1=%d c2=%d coefh=%016llx buf=%x dc=%d,%d e14=%d e16=%d", tag, m->c1.w, m->c2.w, m->coef ? fnv_shorts(m->coef, (size_t)ncoef) : 0ULL, blk_code(m->buf), m->dc[0], m->dc[1], m->diffEnergy,
       m->inEnergy);
}
static void d_sdm(int show, const char *tag, const V22Sdm *s)
{
  emit(show, "%s n=%d t1=%d t2=%d p6=%d mask=%08x nmask=%08x state=%08x sh1=%d sh2=%d", tag, s->n, s->tap1, s->tap2, s->pad6, s->mask, s->nmask, s->state, s->sh1, s->sh2);
}
static void d_smc(int show, const char *tag, const V22Smc *s)
{
  const V22SmcCfg *c = &s->cfg;
  emit(show, "%s w0=%d flag=%d step=%d mod=%d shift=%d mask=%d mask2=%d mask3=%d pmap=%u imap=%u qmap=%u w8=%d w9=%d w10=%d a=%d b=%d", tag, c->w0, c->flag, c->step, c->mod, c->shift, c->mask, c->mask2,
       c->mask3, tbl_code(c->pmap), tbl_code(c->imap), tbl_code(c->qmap), c->w8, c->w9, c->w10, s->a, s->b);
}
static void d_pps(int show, const char *tag, const V22Pps *p)
{
  emit(show, "%s c0=%08x ci=%x cq=%x w3=%d flag=%d ph=%d fill=%d taps=%d hI=%x hQ=%x imap=%u qmap=%u", tag, (unsigned)p->cfg.c0.w, blk_code(p->cfg.coefI), blk_code(p->cfg.coefQ), p->cfg.w3, p->flag,
       p->phase, p->fill, p->taps, blk_code(p->histI), blk_code(p->histQ), tbl_code(p->imap), tbl_code(p->qmap));
  if (p->histI && p->histQ)
    emit(show, "%s H hI=%016llx hQ=%016llx ci=%016llx cq=%016llx", tag, fnv_shorts(p->histI, 6), fnv_shorts(p->histQ, 6), p->cfg.coefI ? fnv_shorts(p->cfg.coefI, 0x78) : 0ULL,
         p->cfg.coefQ ? fnv_shorts(p->cfg.coefQ, 0x78) : 0ULL);
}
static void d_mrf(int show, const char *tag, const V22Mrf *m)
{
  emit(show, "%s coefs=%x w1=%d step=%d ph=%d wpos=%d len=%d buf=%x", tag, blk_code(m->cfg.coefs), m->cfg.w1, m->step, m->phase, m->wpos, m->len, blk_code(m->buf));
  if (m->buf) emit(show, "%s H buf=%016llx coefs=%016llx", tag, fnv_shorts(m->buf, 60), m->cfg.coefs ? fnv_shorts(m->cfg.coefs, 0x10e) : 0ULL);
}
static void d_sre(int show, const char *tag, const V22Sre *s)
{
  emit(show, "%s mode=%d integ=%d avgA=%d avgM=%d want=%d pllC=%d pllE=%d len=%d wpos=%d coefs=%x hist=%x state=%x accX=%d accY=%d frac=%d idx=%d blk=%d cnt=%d sym=%d clk=%d rem=%d", tag, s->mode,
       s->integ, s->avgAngle, s->avgMag, s->wantTrack, s->pllFlagC, s->pllEnable, s->len, s->wpos, blk_code(s->coefs), blk_code(s->hist), blk_code(s->state), s->accX, s->accY, s->frac, s->idx, s->blk,
       s->cnt, s->sym, s->clk, s->rem);
  if (s->hist && s->state && s->coefs)
    emit(show, "%s H hist=%016llx state=%016llx coefs=%016llx", tag, fnv_shorts(s->hist, (size_t)(s->len > 0 && s->len < 200 ? s->len * 2 : 0)), fnv_shorts(s->state, 6), fnv_shorts(s->coefs, 270));
}
static void d_fse(int show, const char *tag, const V22Fse *f)
{
  emit(show, "%s src=%x,%x mu=%d gs=%d integ=%d p0e=%d s10=%d pllE=%d s18=%d adE=%d eAvg=%d ePow=%d outI=%x outQ=%x count=%d nOut=%d taps=%x,%x hist=%x wpos=%d carr=%d ph=%d s42=%d b44=%x b48=%x pllCnt=%d rem=%d p5a=%d "
       "smc=%d decide=%u",
       tag, blk_code(f->srcI), blk_code(f->srcQ), f->mu, f->gainSel, f->integ, f->pad0e, f->s10, f->pllEnable, f->s18, f->adaptEnable, f->errAvg, f->errPow, blk_code(f->outI), blk_code(f->outQ), f->count,
       f->nOut, blk_code(f->tapsI), blk_code(f->tapsQ), blk_code(f->hist), f->wpos, f->carrier, f->phase, f->s42, blk_code(f->buf44), blk_code(f->buf48), f->pllCnt, f->rem, f->pad5a,
       f->smc != NULL, tbl_code((const void *)f->decide));
  if (f->hist && f->tapsI && f->tapsQ && f->srcI && f->srcQ)
    emit(show, "%s H hist=%016llx tI=%016llx tQ=%016llx sI=%016llx sQ=%016llx oI=%016llx oQ=%016llx", tag, fnv_shorts(f->hist, 0x62), fnv_shorts(f->tapsI, 0x31), fnv_shorts(f->tapsQ, 0x31),
         fnv_shorts(f->srcI, 0x31), fnv_shorts(f->srcQ, 0x31), f->outI ? fnv_shorts(f->outI, 14) : 0ULL, f->outQ ? fnv_shorts(f->outQ, 14) : 0ULL);
}
static void d_tone(int show, const char *tag, const FpmTone *t)
{
  int i;
  char b[160];
  size_t n = 0;
  for (i = 0; i < 8; i++) n += (size_t)snprintf(b + n, sizeof b - n, "%d,", t->r0.s[i]);
  emit(show, "%s r0=%s", tag, b);
  n = 0;
  for (i = 0; i < 12; i++) n += (size_t)snprintf(b + n, sizeof b - n, "%d,", t->r1.s[i]);
  emit(show, "%s r1=%s lpf=%d lpfOut=%x hist=%x kfilt=%x kstate=%x kill=%d r2h=%016llx r3=%d,%d,%d,%d", tag, b, t->lpf != NULL, blk_code(t->lpfOut), blk_code(t->hist), blk_code(t->kfilt), blk_code(t->kstate),
       t->killCoef != NULL, fnv_shorts(t->r2, 0x60), t->r3[0], t->r3[1], t->r3[2], t->r3[3]);
}
/* the block size (and class) of every pointer member of a fresh engine: layout independent */
static void d_blocks(int show, const V22Engine *e)
{
  const V22Core *c = e->core;
  const V22Tones *t = e->tones;
  const Blk *b;
  const void *ptrs[12];
  const char *names[12] = {"iir", "rxBuf", "circ", "ppsI", "ppsQ", "mrf", "fseI", "fseQ", "tone", "mtdS1", "mtd1", "mtd2"};
  int i;
  ptrs[0] = t->iir; ptrs[1] = c->rxBuf; ptrs[2] = c->circ.buf; ptrs[3] = c->ppsCoefI; ptrs[4] = c->ppsCoefQ; ptrs[5] = c->mrfCoefs; ptrs[6] = c->fseCoefI; ptrs[7] = c->fseCoefQ;
  ptrs[8] = t->tone; ptrs[9] = t->mtdS1; ptrs[10] = t->mtd1; ptrs[11] = t->mtd2;
  for (i = 0; i < 12; i++) {
    b = find_blk(ptrs[i]);
    if (!b) {
      emit(show, "BLK %s none", names[i]);
      continue;
    }
    if (i >= 8) emit(show, "BLK %s id=%u obj base=%d", names[i], b->id, b->p == (const unsigned char *)ptrs[i]);
    else emit(show, "BLK %s id=%u size=%u base=%d", names[i], b->id, b->size, b->p == (const unsigned char *)ptrs[i]);
  }
  {
    const Blk *be = find_blk(e), *bc = find_blk(c), *bt = find_blk(t);
    emit(show, "BLK eng id=%u core id=%u tones id=%u", be ? be->id : 0, bc ? bc->id : 0, bt ? bt->id : 0);
  }
  emit(show, "BLK fseOutI=%d fseOutQ=%d fseNOut=%d fseTapsI=%d fseTapsQ=%d smcAt=%d", e->fseOutI == c->fse.outI, e->fseOutQ == c->fse.outQ, e->fseNOut == &c->fse.nOut, e->fseTapsI == c->fse.tapsI,
       e->fseTapsQ == c->fse.tapsQ, c->fse.smc == &c->fseSmc);
}
static void d_engine(int show, const char *tag, const V22Engine *e)
{
  const V22Core *c = e->core;
  const V22Tones *t = e->tones;
  char sub[64];
  emit(show, "%s cfg mode=%d rate=%d rate2=%d s6=%d w8=%d gain=%d b10=%02x b11=%02x s12=%d s14=%d thr=%d w18=%d", tag, e->cfg.mode, e->cfg.rate, e->cfg.rate2, e->cfg.s6, e->cfg.handshakeTimeoutMs, e->cfg.gain, e->cfg.b10,
       e->cfg.b11, e->cfg.s12, e->cfg.loopChannel, e->cfg.thr, e->cfg.lossMs);
  emit(show, "%s eng state=%02x f1d=%02x f1e=%02x b1f=%02x s34=%d w38=%d w3c=%d s40=%d w44=%d w48=%d s4c=%d w58=%d", tag, e->state, e->flags1d, e->flags1e, e->b1f, e->s34, e->w38, e->w3c, e->s40, e->w44, e->w48, e->s4c,
       e->w58);
  emit(show, "%s tones timer=%d w4=%d s8=%d sa=%d sc=%d state=%d w10=%d s28=%d w2c=%d s30=%d s32=%d shift=%d w38=%d w3c=%d iir=%016llx,%016llx,%016llx,%016llx", tag, t->timer, t->timeout, t->sigAcc, t->onesAcc, t->sc, t->state,
       t->trained, t->rms, t->rmsSum, t->rmsThr, t->rmsN, t->shift, (int)((unsigned)(unsigned short)t->retrainLocal | ((unsigned)(unsigned short)t->pad3a << 16)), t->lossCnt, t->iir ? fnv_shorts(t->iir->num, 4) : 0ULL, t->iir ? fnv_shorts(t->iir->den, 4) : 0ULL, t->iir ? fnv_shorts(t->iir->x, 4) : 0ULL,
       t->iir ? fnv_shorts(t->iir->y, 4) : 0ULL);
  emit(show, "%s core w0=%d w4=%d w8=%d wc=%d w10=%d w14=%d scr=%d descr=%d w20=%d w24=%d txBis=%d rxBis=%d m2c=%d m2e=%d smcw=%d rxCount=%d", tag, c->w0, c->sreGate, c->fsePllGate, c->fseS18Gate, c->adaptGate, c->w14, c->scrOn,
       c->descrOn, c->w20, c->w24, c->txBis, c->rxBis, c->txChannel, c->rxChannel, c->fseSmc, c->rxCount);
  snprintf(sub, sizeof sub, "%s sdm", tag); d_sdm(show, sub, &c->sdm);
  snprintf(sub, sizeof sub, "%s sdmRx", tag); d_sdm(show, sub, &c->sdmRx);
  snprintf(sub, sizeof sub, "%s smc", tag); d_smc(show, sub, &c->smc);
  snprintf(sub, sizeof sub, "%s pps", tag); d_pps(show, sub, &c->pps);
  emit(show, "%s circ w0=%d w1=%d buf=%x wr=%d rd=%d lim=%d bufh=%016llx", tag, c->circ.w0, c->circ.w1, blk_code(c->circ.buf), c->circ.wr, c->circ.rd, c->circ.lim, c->circ.buf ? fnv_shorts((const short *)c->circ.buf, 12) : 0ULL);
  snprintf(sub, sizeof sub, "%s mrf", tag); d_mrf(show, sub, &c->mrf);
  snprintf(sub, sizeof sub, "%s agcIn", tag); d_agc(show, sub, &c->agcIn);
  snprintf(sub, sizeof sub, "%s agc", tag); d_agc(show, sub, &c->agc);
  snprintf(sub, sizeof sub, "%s sre", tag); d_sre(show, sub, &c->sre);
  snprintf(sub, sizeof sub, "%s fse", tag); d_fse(show, sub, &c->fse);
  if (c->ppsCoefI && c->ppsCoefQ && c->mrfCoefs && c->fseCoefI && c->fseCoefQ && c->rxBuf)
    emit(show, "%s bufs ppsI=%016llx ppsQ=%016llx mrf=%016llx fseI=%016llx fseQ=%016llx rx=%016llx", tag, fnv_shorts(c->ppsCoefI, 0x78), fnv_shorts(c->ppsCoefQ, 0x78), fnv_shorts(c->mrfCoefs, 0x10e),
         fnv_shorts(c->fseCoefI, 0x31), fnv_shorts(c->fseCoefQ, 0x31), fnv_shorts(c->rxBuf, 0xaa));
  if (t->tone) {
    snprintf(sub, sizeof sub, "%s tone", tag); d_tone(show, sub, t->tone);
  }
  if (t->mtdS1) { snprintf(sub, sizeof sub, "%s mtdS1", tag); d_mtd(show, sub, t->mtdS1, 20); }
  if (t->mtd1) { snprintf(sub, sizeof sub, "%s mtd1", tag); d_mtd(show, sub, t->mtd1, 15); }
  if (t->mtd2) { snprintf(sub, sizeof sub, "%s mtd2", tag); d_mtd(show, sub, t->mtd2, 15); }
}

/* ------------------------------------------------------------------------------------------------ the scripted handlers */
typedef struct HScript {
  int fill_out;            /* write 160 output samples */
  int rx_n;                /* *rxCount16 result (0 .. 100) */
  int tx_write;            /* *txCount16 result */
  int new_state;           /* engine state byte (-1: unchanged) */
  int new_f1d, new_f1e;    /* engine flag bytes (-1: unchanged) */
  int new_tstate;          /* tones.state (-1: unchanged) */
  int new_sc;              /* tones.sc (-1: unchanged) */
  int mode;                /* 0: scripted behaviour above; 1: act as the data state (R08's v22_data) with the real R06 chain (scramble + modulate / demodulate + descramble) */
} HScript;
static HScript g_hs;
static short *g_hbuf[3]; /* tx, in, rxbits buffers of the first call: stable and distinct */
static unsigned long g_hcalls[8];
static int g_hlog = 1;
static int g_hid; /* the handler (0 .. 6) being run: for the hook below */
static int (*g_hmode1)(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount);

static int handler(int id, V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount)
{
  int i, ntx = (int)*txCount;
  g_hcalls[id]++;
  g_hid = id;
  if (!g_hbuf[0]) {
    g_hbuf[0] = tx;
    g_hbuf[1] = in;
    g_hbuf[2] = rxBits;
    if (tx == in || tx == rxBits || in == rxBits || tx == out || in == out || rxBits == out) die3("handler buffers overlap");
  } else if (g_hbuf[0] != tx || g_hbuf[1] != in || g_hbuf[2] != rxBits) die3("handler buffers moved");
  if (g_hlog)
    emit(0, "H%d state=%02x f1d=%02x f1e=%02x ts=%d sc=%d txc=%d rxc=%d txh=%016llx inh=%016llx rxh=%016llx outh=%016llx gain=%d", id, e->state, e->flags1d, e->flags1e, e->tones->state, e->tones->sc, ntx,
         (int)*rxCount, fnv_shorts(tx, 100), fnv_shorts(in, 160), fnv_shorts(rxBits, 100), fnv_shorts(out, 160), e->cfg.gain);
  if (g_hs.mode == 1 && g_hmode1) return g_hmode1(e, tx, out, in, rxBits, txCount, rxCount);
  if (g_hs.fill_out)
    for (i = 0; i < 160; i++) out[i] = (short)(rpct(10) ? (rpct(50) ? 32767 : -32768) : (int)rnd());
  for (i = 0; i < g_hs.rx_n && i < 100; i++) rxBits[i] = (short)rnd();
  *rxCount = (short)g_hs.rx_n;
  *txCount = (short)g_hs.tx_write;
  if (g_hs.new_state >= 0) e->state = (unsigned char)g_hs.new_state;
  if (g_hs.new_f1d >= 0) e->flags1d = (unsigned char)g_hs.new_f1d;
  if (g_hs.new_f1e >= 0) e->flags1e = (unsigned char)g_hs.new_f1e;
  if (g_hs.new_tstate >= 0) e->tones->state = (short)g_hs.new_tstate;
  if (g_hs.new_sc >= 0) e->tones->sc = (short)g_hs.new_sc;
  return 0;
}
#ifndef R08_REAL_HANDLERS /* R08: the five handlers v22_data / v22_local_loop / v22_org_rmloop2 / v22_ans_rmloop2 / v22_retrain are the code under test (the blob's / src/v22/v22_data.c, v22_loopback.c, v22_retrain.c), not recorders */
int v22_data(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount) { return handler(0, e, tx, out, in, rxBits, txCount, rxCount); }
#endif
#ifndef R07_REAL_CALLSETUP /* R07: v22_originate / v22_answer are the code under test (the blob's / src/v22/v22_call_setup.c), not recorders */
int v22_originate(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount) { return handler(1, e, tx, out, in, rxBits, txCount, rxCount); }
int v22_answer(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount) { return handler(2, e, tx, out, in, rxBits, txCount, rxCount); }
#endif
#ifndef R08_REAL_HANDLERS
int v22_local_loop(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount) { return handler(3, e, tx, out, in, rxBits, txCount, rxCount); }
int v22_org_rmloop2(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount) { return handler(4, e, tx, out, in, rxBits, txCount, rxCount); }
int v22_ans_rmloop2(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount) { return handler(5, e, tx, out, in, rxBits, txCount, rxCount); }
int v22_retrain(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount) { return handler(6, e, tx, out, in, rxBits, txCount, rxCount); }
#endif

/* ------------------------------------------------------------------------------------------------ engine creation */
static void make_cfg(int *cfg, int k)
{
  static const int c0[] = {0, 1, 2, 3, -1, 0x80000000, 256};
  static const int c1[] = {0, 1, 2, 3, -1, 0x7fffffff};
  static const int c2[] = {60000, 0, -5, 0x7fffffff, 1, 120000};
  static const int bit[] = {0, 1, 2, 3, -1, 0x100, 5};
  static const int c4[] = {700, 0x12345, -1, 0, 0x7fff, 0x8000};
  cfg[0] = c0[k % 7];
  cfg[1] = c1[(k / 3) % 6];
  cfg[2] = c2[(k / 2) % 6];
  cfg[3] = bit[(k / 5) % 7];
  cfg[4] = c4[(k / 7) % 6];
  cfg[5] = bit[(k / 11) % 7];
  cfg[6] = bit[(k / 13) % 7];
}
static V22Engine *new_engine(int mode, int proto, int dataStart)
{
  int cfg[7];
  cfg[0] = mode; cfg[1] = proto; cfg[2] = 60000; cfg[3] = 0; cfg[4] = 700; cfg[5] = dataStart; cfg[6] = 1;
  return V22FP_create(NULL, cfg);
}
#endif
