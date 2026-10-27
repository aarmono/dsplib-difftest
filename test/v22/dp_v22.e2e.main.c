/*
 * R06 differential harness, id r06_e2e: the V.22 / V.22bis / Bell 212A dp glue (src/v22/dp_v22.c) over the REAL dp_wrapper, the REAL RcFixed converters and the REAL engine (src/v22/v22_engine.c + the
 * V.22 / FPM_* ports) vs the real dsplibs.o (32-bit): create / process through op->process = dp_wrapper_run / delete, with two glue instances (caller and answerer) that hear each
 * other one block late.  ONE source (library: test/v22/r06_common.h + test/v22/r06_eng_common.h); the 32-bit BLOB run is the x86-64 golden (test/golden/v22/dp_v22.e2e.blob32.out.gz).
 *
 * Callee recorders on BOTH sides: sysdep_*, dsplibs_debug_printf, modem_get_bits / put_bits / get_param / set_param / dp_register (scripted bits, logged), and the seven R07 / R08 handlers
 * (v22_data, v22_originate, v22_answer, v22_local_loop, v22_org_rmloop2, v22_ans_rmloop2, v22_retrain: still blob functions, here a scripted stand-in: the start states (1 / 2) send
 * silence or a tone for a few blocks and then declare the CONNECT (engine state 3 / 4), the data state (0) runs the REAL R06 chain - ScrambleDataV22 + ModDataV22 for the bits the glue
 * fetched, DemodDataV22 + DescrambleDataV22 for the audio it hears -, other states send silence; scripted restarts (state 1) and errors (state 6 / 9) exercise the glue's status
 * mapping).  The handlers are the same code on both sides: only R06 code and the glue differ between blob and port.
 *
 * Scenarios (own process each): 0 = V.22bis (id 122, 2400 bit/s) at 8000 Hz, 1 = V.22 (id 22, 1200) at 8000 Hz, 2 = Bell 212A (id 212, 1200) at 9600 Hz (converters), 3 = V.22bis at 9600 Hz
 * with restarts and errors.
 */
#define R02_REAL_RC 1
#include "r06_eng_common.h"

enum { CV_CREATE = 0, CV_CONNECT24, CV_CONNECT12, CV_DATA, CV_RXPUT, CV_TXGET, CV_RESTART, CV_ERR, CV_ZEROED, CV_START_STATE, CV_H_DATA, CV_H_START, CV_H_OTHER, CV_SYMS, CV_DELETE, CV_BLOCKS,
       CV_CALLER, CV_ANSWERER, CV_CONV };

typedef struct Inst {
  struct dp *d;
  V22Glue *g;
  V22Engine *e;
  int rate24, blocks_in_state, connectAt, nblocks, errAt, restartAt, datablocks;
  short q[1200];
  int avail;
  short last[256];
  unsigned long syms;
} Inst;
static Inst g_inst[2];
static Inst *inst_of(const V22Engine *e)
{
  int i;
  for (i = 0; i < 2; i++)
    if (g_inst[i].e == e) return &g_inst[i];
  die3("handler for an unknown engine");
  return NULL;
}
static int e2e_handler(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount)
{
  Inst *I = inst_of(e);
  int id = g_hid, i;
  short c16 = 0;
  I->nblocks++;
  *rxCount = 0;
  if (id == 1 || id == 2) {
    cov_hit(CV_H_START);
    if (I->blocks_in_state & 1) TxNOP(e, NULL, out, &c16);
    else for (i = 0; i < 160; i++) out[i] = (short)(((i * 37 + I->nblocks * 11) & 0xff) * 40 - 5000);
    *txCount = 0;
    if (I->blocks_in_state++ >= I->connectAt) {
      e->state = (unsigned char)(I->rate24 ? 3 : 4);
      I->blocks_in_state = 0;
      cov_hit(I->rate24 ? CV_CONNECT24 : CV_CONNECT12);
    } else {
      e->state = 1;
      cov_hit(CV_START_STATE);
    }
    return 0;
  }
  if (id == 0) {
    unsigned short sym[16], dout[16];
    short mo[600], din[200];
    int n = (int)*txCount, mask = I->rate24 ? 0xf : 3, nsym, k;
    unsigned nm;
    cov_hit(CV_H_DATA);
    if (n > 12) n = 12;
    for (i = 0; i < 12; i++) sym[i] = (unsigned short)(i < n && n > 0 ? (tx[i] & mask) : 0);
    ScrambleDataV22(e, sym, 12);
    nm = ModDataV22(e, sym, mo, 12);
    if (I->avail + (int)nm > 1100) I->avail = 0;
    for (k = 0; k < (int)nm; k++) I->q[I->avail + k] = mo[k];
    I->avail += (int)nm;
    for (k = 0; k < 160; k++) out[k] = k < I->avail ? I->q[k] : 0;
    if (I->avail >= 160) {
      memmove(I->q, I->q + 160, (size_t)(I->avail - 160) * sizeof(short));
      I->avail -= 160;
    } else I->avail = 0;
    memcpy(din, in, 160 * sizeof(short));
    nsym = (int)DemodDataV22(e, din, (short *)dout, 160);
    DescrambleDataV22(e, dout, (unsigned short)(nsym > 0 && nsym < 16 ? nsym : 0));
    for (k = 0; k < nsym && k < 14; k++) rxBits[k] = (short)(dout[k] & mask);
    *rxCount = (short)(nsym > 0 && nsym < 15 ? nsym : 0);
    I->syms += (unsigned long)nsym;
    if (nsym) cov_hit(CV_SYMS);
    *txCount = 12;
    I->datablocks++;
    e->state = 0;
    if (I->errAt && I->datablocks == I->errAt) e->state = (unsigned char)((I->nblocks & 1) ? 6 : 9);
    if (I->restartAt && I->datablocks == I->restartAt) { e->state = 1; e->tones->state = (short)(I->rate24 ? 2 : 1); I->blocks_in_state = 0; cov_hit(CV_RESTART); }
    return 0;
  }
  RxClampV22(e, NULL, out, &c16);
  for (i = 0; i < 160; i++) out[i] = 0;
  e->state = 0;
  return 0;
}

static struct dp_operations *g_ops;
static const char *slot_name(const void *fn) { return fn == (const void *)dp_wrapper_run ? "dp_wrapper_run" : (fn ? "other" : "NULL"); }
static void dump_glue(int show, const char *tag, struct dp *dp)
{
  V22Glue *g = (V22Glue *)dp;
  if (!dp) {
    emit(show, "%s NULL", tag);
    return;
  }
  emit(show, "%s id=%d modem=m%d status=%u bits=%d want=%d tx=%016llx rx=%016llx", tag, (int)dp->id, modem_no(dp->modem), dp->status, g->bits, g->want, fnv_ints(g->txbits, 100), fnv_ints(g->rxbits, 100));
}
static short g_in[2][2048], g_out[2][2048];

int main(int argc, char **argv)
{
  static const int ids[4] = {DP_V22BIS, DP_V22, DP_B212, DP_V22BIS};
  static const int srates[4] = {8000, 8000, 9600, 9600};
  int sc = argc > 1 ? atoi(argv[1]) : 0, it, i, blk;
  g_verbose = getenv("R02_VERBOSE") != NULL;
  g_lcg = 0x2545f491u + 1000003u * (unsigned)sc;
  if (getenv("R02_SEED")) g_lcg ^= (unsigned)atoi(getenv("R02_SEED"));
  if (sc < 0 || sc > 3) die3("no such scenario %d", sc);
  cov_def(CV_CREATE, "create (glue + wrapper + engine)"); cov_def(CV_DATA, "data blocks"); cov_def(CV_RXPUT, "rx bits put"); cov_def(CV_TXGET, "tx bits fetched");
  cov_def(CV_START_STATE, "start state blocks"); cov_def(CV_H_DATA, "data handler"); cov_def(CV_H_START, "originate / answer handler"); cov_def(CV_SYMS, "symbols demodulated"); cov_def(CV_DELETE, "delete");
  cov_def(CV_BLOCKS, "blocks processed"); cov_def(CV_CALLER, "caller instance"); cov_def(CV_ANSWERER, "answerer instance");
  /* scenario specific counters: the connect rate, the converters (9600 Hz), the scripted restart and error */
  if (ids[sc] == DP_V22BIS) cov_def(CV_CONNECT24, "connect 2400 (engine state 3)"); else cov_def(CV_CONNECT12, "connect 1200 (engine state 4)");
  if (srates[sc] == 9600) cov_def(CV_CONV, "converters (9600 Hz)");
  if (sc == 3) { cov_def(CV_RESTART, "restart (engine state 1) from the data state"); cov_def(CV_ERR, "error status (glue result 4)"); }
  arena_init();
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  dp_v22_init();
  g_ops = g_reg_op[0];
  g_hmode1 = e2e_handler;
  g_hs.mode = 1;
  g_hlog = 1;
  dsplibs_debug_level = sc == 3 ? 2 : 1;
  g_bits_give = 12;
  g_bits_raw = 0;
  for (i = 0; i < 2; i++) {
    struct dp *d;
    memset(&g_inst[i], 0, sizeof g_inst[i]);
    d = g_ops->create(M_(i), (enum DP_ID)ids[sc], i == 0 ? 1 : 0, srates[sc], 160, g_ops); /* instance 0 is the caller, 1 the answerer */
    if (!d) die3("create");
    g_inst[i].d = d;
    g_inst[i].g = (V22Glue *)d;
    g_inst[i].e = g_inst[i].g->eng;
    g_inst[i].rate24 = ids[sc] == DP_V22BIS;
    g_inst[i].connectAt = 12 + 3 * i;
    cov_hit(CV_CREATE);
    cov_hit(i == 0 ? CV_CALLER : CV_ANSWERER);
    if (srates[sc] == 9600) cov_hit(CV_CONV);
    dump_glue(1, i == 0 ? "caller" : "answerer", d);
  }
  if (sc == 3) {
    g_inst[0].errAt = 70;
    g_inst[1].restartAt = 90;
  } else if (sc == 2) {
    g_inst[0].restartAt = 0;
  }
  blk = srates[sc] == 9600 ? 192 : 160;
  for (it = 0; it < 260; it++) {
    for (i = 0; i < 2; i++) {
      int r, cnt = blk, k;
      unsigned long npb0 = g_npb, ngb0 = g_ngb;
      unsigned st0 = g_inst[i].d->status;
      /* each side hears the other one's previous block (plus a little noise) */
      for (k = 0; k < cnt; k++) g_in[i][k] = (short)(g_out[1 - i][k] + (rpct(3) ? (int)(rnd() % 9) - 4 : 0));
      for (k = 0; k < 2048; k++) g_out[i][k] = 0x5a5a;
      reg_clear();
      reg_add(g_in[i], sizeof g_in[i], 2, "IN");
      reg_add(g_out[i], sizeof g_out[i], 2, "OUT");
      r = g_ops->process(g_inst[i].d, g_in[i], g_out[i], cnt);
      emit(0, "e2e inst=%d it=%d -> %d out=%016llx", i, it, r, fnv_shorts(g_out[i], (unsigned)cnt));
      cov_hit(CV_BLOCKS);
      if (r == 4) cov_hit(CV_ERR);
      (void)st0;
      if (g_npb > npb0) cov_hit(CV_RXPUT);
      if (g_ngb > ngb0) cov_hit(CV_TXGET);
      if (g_inst[i].datablocks) cov_hit(CV_DATA);
      if (it % 20 == 0) {
        dump_glue(0, "e2e", g_inst[i].d);
        d_engine(0, "E", g_inst[i].g->eng);
      }
    }
  }
  for (i = 0; i < 2; i++) {
    dump_glue(1, "end", g_inst[i].d);
    d_engine(1, i ? "end ans" : "end cal", g_inst[i].g->eng);
    emit(1, "inst %d blocks %d data blocks %d symbols %lu", i, g_inst[i].nblocks, g_inst[i].datablocks, g_inst[i].syms);
    emit(1, "delete -> %d", g_ops->delete(g_inst[i].d));
    cov_hit(CV_DELETE);
  }
  trial_end("e2e");
  cov_finish();
  return 0;
}
