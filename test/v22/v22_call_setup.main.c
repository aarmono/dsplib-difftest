/*
 * R07 differential harness, id r07_call: the V.22 / V.22bis / Bell 212A CALL SETUP state machines - v22_originate, v22_answer, connect_2400, connect_1200 - the real dsplibs.o (32-bit)
 * vs src/v22/v22_call_setup.c.  ONE source.  Library: test/v22/r06_common.h (hashed canonical output, deterministic heap, callee recorders, coverage counters exit 2, 3 = harness violation) +
 * test/v22/r06_eng_common.h (the layout independent description of the engine, engine creation; R07_REAL_CALLSETUP leaves v22_originate / v22_answer to the code under test).
 *
 * The handlers are state machines driven by V22FP_modem (the real dispatch through V22_PROTOCOL, the real R06 chain, REAL FPM_TONE / MTD / AGC detectors, the REAL Detect_v22 /
 * Detect_1s / DemodDataV22 ... of each side) with 160-sample blocks.  The input signals are made by the engines themselves (two engines hear each other one block late, so the
 * real handshake tones, S1, scrambled / unscrambled ones and data are what the detectors see; perturbations - silence gaps, noise, attenuation, a muted peer, shortened timeouts,
 * state pokes - are scripted); no libm, integer only.  CALLEE RECORDERS on BOTH sides: the five R08 handlers (v22_data = a stand-in running the real R06 chain
 * ScrambleDataV22 + ModDataV22 / DemodDataV22 + DescrambleDataV22, v22_retrain = silence for a while, then the handshake restarts; v22_local_loop / v22_org_rmloop2 /
 * v22_ans_rmloop2 = silence; the blob's copies are made weak with objcopy), sysdep_*, dsplibs_debug_printf (every message is part of the diffed text and counted).
 *
 * Scenarios (own process each):
 *   0 happy paths: caller + answerer pairs V.22bis 2400, V.22 1200, Bell212 (protocol 2) through the whole handshake into the data state (debug level 2 for the first)
 *   1 V.22bis pairs with perturbations: timeouts of every state, silence gaps / noise / attenuation in every state of the handshake (connect_2400 carrier supervision, retrain, errors)
 *   2 V.22 / Bell212 pairs with the same perturbations (connect_1200, V.22 detection by unscrambled ones)
 *   3 state fuzz: random tones.sc / timer / accumulator pokes on random engines of a running pair, random input sources (peer, silence, noise, tone, clipped, loopback), all counts
 *   4 direct calls of connect_2400 / connect_1200 (and the handlers) on poked engines with real and random input
 */
#define R07_REAL_CALLSETUP 1
#include "r06_eng_common.h"

extern short FPM_rms(const short *data, short count);

/* ---- coverage ids ---- */
enum {
  CV_O_SC = 0,        /* 0 .. 14: originate called with sc = k (14 = beyond the table) */
  CV_A_SC = 16,       /* 16 .. 31: answer called with sc = k (15 = beyond the table) */
  CV_C24_SC = 32,     /* 32 .. 36: connect_2400 with sc 8 .. 11, other */
  CV_C12_SC = 37,     /* 37 .. 39: connect_1200 with sc 12, 13, other */
  CV_EV = 40          /* events, see ev_names */
};
enum {
  EV_O_0_1, EV_O_1_3, EV_O_3_4, EV_O_4_5, EV_O_5_6, EV_O_6_8, EV_O_6_C, EV_O_ERR1, EV_O_ERR3, EV_O_ERR4, EV_O_SHIFT1, EV_O_F1E8, EV_O_F1D10,
  EV_A_0_1, EV_A_0_3, EV_A_1_E, EV_A_E_3, EV_A_3_4, EV_A_3_C, EV_A_4_8, EV_A_ERR5, EV_A_TONE3,
  EV_C_8_9, EV_C_9_A, EV_C_A_B, EV_C_A_DONE, EV_C_B_DONE, EV_C_ERR7, EV_C_C_D, EV_C_C_DONE, EV_C_D_DONE, EV_C_ERR18, EV_C_NOCAR24, EV_C_NOCAR12, EV_C_RETRAIN, EV_C_LOSS24, EV_C_LOSS12,
  EV_DATA, EV_DATA24, EV_DATA12, EV_RETRAIN_H, EV_NODET, EV_NEV
};
static const char *ev_names[EV_NEV] = {
  "originate 0 -> 1", "originate 1 -> 3 (answer tone detected)", "originate 3 -> 4", "originate 4 -> 5", "originate 5 -> 6", "originate 6 -> 8 (S1)", "originate 6 -> 0xc (ones)", "originate error 0x11",
  "originate error 0x13", "originate error 0x14", "originate level shift set", "originate flags1e |= 8", "originate flags1d |= 0x10",
  "answer 0 -> 1", "answer 0 -> 3", "answer 1 -> 0xe", "answer 0xe -> 3", "answer 3 -> 4 (V.22bis)", "answer 3 -> 0xc (V.22)", "answer 4 -> 8", "answer error 0x15", "answer guard tone (timer > 0x5be)",
  "connect_2400 8 -> 9", "connect_2400 9 -> 10", "connect_2400 10 -> 11", "connect_2400 10 -> connected (3)", "connect_2400 11 -> connected (3)", "connect_2400 error 0x17", "connect_1200 0xc -> 0xd",
  "connect_1200 0xc -> connected (4)", "connect_1200 0xd -> connected (4)", "connect_1200 error 0x18", "connect_2400 no carrier 0x10", "connect_1200 no carrier 0x10", "connect_2400 retrain started (0xb)",
  "connect_2400 carrier loss counted (not reported)", "connect_1200 carrier loss counted (not reported)",
  "data handler stand-in", "data at 2400", "data at 1200", "retrain handler stand-in", "signal detected (agcIn.detect) with no symbol out (S1 / ones states)"};
/* debug message substrings (every printf of the code under test) */
static const char *msg_sub[] = {
  "V22_originate,NODE_0", "V22_originate, NODE_1", "V22_originate, NODE_3", "V22_originate, NODE_4", "V22_originate, NODE_5", "V22_originate, NODE_6", "V22_originate, NODE 8", "V22_originate, NODE 9",
  "V22_originate, NODE 10", "V22_originate, NODE 11", "V22_originate, NODE 12", "V22_originate, NODE 13", "V22_MSG_ERROR1", "V22_MSG_ERROR3", "V22_MSG_ERROR4", "V.22 640 modem det true",
  "V22_answer,NODE_0", "V22_answer,NODE_1", "V22_answer,NODE_3", "V22_answer,NODE_4", "V22_answer,NODE_SILENCE_AFTER_2100", "V22_answer, NODE 8", "V22_answer, NODE 12", "V22_MSG_ERROR5",
  "Detected V22 Carrier", "Detected V22bis Carrier", "connect_2400, NODE_2400B", "connect_2400, NODE_2400C", "V22_MSG_CONNECT_2400 In NODE_2400C", "V22_MSG_CONNECT_2400 In NODE_2400D", "V22_MSG_ERROR7",
  "V22_MSG_NO_CARRIER3", "V22_MSG_NO_CARRIER4", "won't be reported", "Carrier back during carrier_loss_time"};
#define NMSG ((int)(sizeof msg_sub / sizeof msg_sub[0]))
#define CV_MSG (CV_EV + EV_NEV)
static void dbg_hook(const char *m)
{
  int i;
  for (i = 0; i < NMSG; i++)
    if (strstr(m, msg_sub[i])) cov_hit(CV_MSG + i);
}
static void cov_setup(int msgs)
{
  int i;
  static char nb[NMSG + 1][80];
  for (i = 0; i < 15; i++) cov_defn(CV_O_SC + i, "originate sc", i);
  for (i = 0; i < 16; i++) cov_defn(CV_A_SC + i, "answer sc", i);
  cov_def(CV_C24_SC + 0, "connect_2400 sc 8"); cov_def(CV_C24_SC + 1, "connect_2400 sc 9"); cov_def(CV_C24_SC + 2, "connect_2400 sc 10"); cov_def(CV_C24_SC + 3, "connect_2400 sc 11"); cov_def(CV_C24_SC + 4, "connect_2400 other sc");
  cov_def(CV_C12_SC + 0, "connect_1200 sc 12"); cov_def(CV_C12_SC + 1, "connect_1200 sc 13"); cov_def(CV_C12_SC + 2, "connect_1200 other sc");
  for (i = 0; i < EV_NEV; i++) cov_def(CV_EV + i, ev_names[i]);
  if (msgs)
    for (i = 0; i < NMSG; i++) {
      snprintf(nb[i], sizeof nb[i], "message \"%s\"", msg_sub[i]);
      cov_def(CV_MSG + i, nb[i]);
    }
}
/* ---- nodes ---- */
/* The counters each scenario must reach (listed from a run: R07_LISTCOV=1 prints the reached ids on stderr); every other counter is dropped for the scenario.  The union of the five
 * scenarios covers every counter (checked when the lists were made). */
static const unsigned char req0[] = {0,1,3,4,5,6,8,9,10,12,16,17,19,20,24,25,26,27,28,30,32,33,34,35,37,40,41,42,43,44,45,46,51,52,53,54,55,56,57,58,59,62,63,64,65,66,69,77,78,79,82,83,84,85,86,87,88,89,90,92,97,98,99,100,101,102,103,104,106,107,108,109,110,111};
static const unsigned char req1[] = {0,1,3,4,5,6,8,9,10,11,12,13,16,17,19,20,24,25,26,27,28,29,30,32,33,34,35,37,38,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,71,72,73,74,75,76,77,78,79,80,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116};
static const unsigned char req2[] = {0,1,3,4,5,6,8,9,10,11,12,13,16,17,19,28,29,30,32,33,34,35,37,38,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,58,60,61,62,63,64,68,69,70,71,73,74,75,76,77,79,80,82,83,84,85,86,87,92,93,94,95,96,97,98,99,100,102,104,105,106,114,115};
static const unsigned char req3[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,37,38,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,67,68,69,70,71,72,73,74,75,76,77,78,79,80,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,112,113,114,115,116};
static const unsigned char req4[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,50,51,52,53,54,57,58,59,60,61,62,63,64,65,67,68,69,70,71,72,73,74,75,76,77,78,79,80,82,83,84,85,86,87,88,89,90,91,92,93,94,95,97,98,99,100,101,102,103,104,105,106,107,108,109,110,112,113,114,115,116};
static const unsigned char req5[] = {1,3,4,5,6,8,9,10,11,12,13,17,19,20,24,25,26,27,28,29,30,32,33,34,35,37,38,42,43,44,45,46,47,48,49,51,55,57,58,59,60,61,62,63,64,65,67,68,69,71,75,76,77,78,79,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,99,100,101,102,103,104,105,106,107,108,109,110,112,115};
static void cov_require(int sc)
{
  static const unsigned char *const lists[6] = {req0, req1, req2, req3, req4, req5};
  static const unsigned sizes[6] = {sizeof req0, sizeof req1, sizeof req2, sizeof req3, sizeof req4, sizeof req5};
  unsigned k;
  int i, keep;
  for (i = 0; i < COV_MAX; i++) {
    keep = 0;
    for (k = 0; k < sizes[sc]; k++) if (lists[sc][k] == i) keep = 1;
    if (!keep) g_covname[i] = NULL;
  }
}

static int g_rxin = 160; /* the number of input samples handed to V22FP_modem (the in-count of the rx count: Demod produces 0 / 1 / 2 ... symbols for small blocks) */
static int g_dense; /* debugging aid: describe the engine after every block (R07_DENSE=1; changes the hash) */

/* ---- nodes ---- */
typedef struct Node {
  V22Engine *e;
  int role;                 /* 0 caller (originate), 1 answerer */
  int nblk;
  short peer[160];          /* what the other side sent in its previous block */
  short out[256];           /* ModDataV22 writes about 13.3 samples per symbol: counts above 12 need room beyond the 160 samples of the block */
  unsigned txc, rxc;
  int pad_before[64];       /* V22FP_modem copies as many bits as the (unchanged) rx count of the call, 160, into rxBits when a handler leaves it alone (answer sc 14): keep room and the counts apart */
  int txbits[128], rxbits[512];
  int pad_after[64];
  /* the data stand-in */
  short q[1200];
  int avail;
  int dataBlocks, retrainBlocks;
  /* perturbation state */
  int gapLeft, gapKind, gapDone;
  int prevSc, prevState;
  int mute;                 /* this node's OUTPUT is replaced by silence for the peer */
} Node;
static Node g_node[4];
static Node *node_of(const V22Engine *e)
{
  int i;
  for (i = 0; i < 4; i++)
    if (g_node[i].e == e) return &g_node[i];
  die3("handler for an unknown engine");
  return NULL;
}
static int stand_in(V22Engine *e, short *tx, short *out, short *in, short *rxBits, short *txCount, short *rxCount)
{
  Node *N = node_of(e);
  int id = g_hid, i;
  short c16 = 0;
  *rxCount = 0;
  if (id == 0) {
    unsigned short sym[16], dout[16];
    short mo[600], din[200];
    int n = (int)*txCount, mask = e->core->txBis ? 0xf : 3, nsym, k;
    unsigned nm;
    cov_hit(CV_EV + EV_DATA);
    cov_hit(CV_EV + (e->core->txBis ? EV_DATA24 : EV_DATA12));
    if (n > 12) n = 12;
    for (i = 0; i < 12; i++) sym[i] = (unsigned short)(i < n && n > 0 ? (tx[i] & mask) : 0);
    ScrambleDataV22(e, sym, 12);
    nm = ModDataV22(e, sym, mo, 12);
    if (N->avail + (int)nm > 1100) N->avail = 0;
    for (k = 0; k < (int)nm; k++) N->q[N->avail + k] = mo[k];
    N->avail += (int)nm;
    for (k = 0; k < 160; k++) out[k] = k < N->avail ? N->q[k] : 0;
    if (N->avail >= 160) {
      memmove(N->q, N->q + 160, (size_t)(N->avail - 160) * sizeof(short));
      N->avail -= 160;
    } else N->avail = 0;
    memcpy(din, in, 160 * sizeof(short));
    nsym = (int)DemodDataV22(e, din, (short *)dout, 160);
    DescrambleDataV22(e, dout, (unsigned short)(nsym > 0 && nsym < 16 ? nsym : 0));
    for (k = 0; k < nsym && k < 14; k++) rxBits[k] = (short)(dout[k] & mask);
    *rxCount = (short)(nsym > 0 && nsym < 15 ? nsym : 0);
    *txCount = 12;
    N->dataBlocks++;
    e->state = 0;
    return 0;
  }
  if (id == 6) { /* retrain: silence, then the handshake restarts */
    cov_hit(CV_EV + EV_RETRAIN_H);
    TxNOP(e, NULL, out, &c16);
    N->retrainBlocks++;
    e->state = 0;
    if (N->retrainBlocks >= 6) {
      N->retrainBlocks = 0;
      e->state = 1;
      e->tones->state = (short)(N->role ? 2 : 1);
      e->tones->sc = 0;
      e->tones->timer = 0;
    }
    return 0;
  }
  RxClampV22(e, NULL, out, &c16);
  for (i = 0; i < 160; i++) out[i] = 0;
  e->state = 0;
  return 0;
}

static Node *node_new(int slot, int role, int proto, int cfg3)
{
  Node *N = &g_node[slot];
  int cfg[7];
  memset(N, 0, sizeof *N);
  N->role = role;
  cfg[0] = role ? 1 : 0; cfg[1] = proto; cfg[2] = 60000; cfg[3] = cfg3; cfg[4] = 700; cfg[5] = 0; cfg[6] = 1;
  N->e = V22FP_create(NULL, cfg);
  return N;
}

/* one block of one node: the input is `in` (160 samples), the tx bits random symbols of the right width */
static unsigned node_step(Node *N, const short *in, int txc)
{
  V22Engine *e = N->e;
  int i, mask = e->core->txBis ? 0xf : 3, sc0 = e->tones->sc, ts0 = e->tones->state, m2e_saved = 0;
  unsigned ret;
  for (i = 0; i < 128; i++) N->txbits[i] = (int)(rnd() & (unsigned)mask);
  for (i = 0; i < 512; i++) N->rxbits[i] = 0x5a5a;
  for (i = 0; i < 256; i++) N->out[i] = 0;
  N->txc = (unsigned)txc;
  /* DemodDataV22 of an engine with the IIR front end (core.m2e == 2: originate engines) filters 160 samples against a demodulator tone that is generated for n samples only: the rest is
   * uninitialised stack in the blob (R06 observation, not reachable: the wrapper always passes 160).  Small blocks therefore run with the front end switched off (m2e = 1, the answer
   * engines' value) for that call. */
  if (g_rxin != 160 && e->core->rxChannel == 2) { e->core->rxChannel = 1; m2e_saved = 2; }
  N->rxc = (unsigned)g_rxin;
  /* the coverage of the entry state */
  if (ts0 == 1) cov_hit(CV_O_SC + (sc0 >= 0 && sc0 <= 13 ? sc0 : 14));
  if (ts0 == 2) cov_hit(CV_A_SC + (sc0 >= 0 && sc0 <= 14 ? sc0 : 15));
  if (ts0 == 1 || ts0 == 2) {
    if (sc0 >= 8 && sc0 <= 11) cov_hit(CV_C24_SC + (sc0 - 8));
    if (sc0 == 12 || sc0 == 13) cov_hit(CV_C12_SC + (sc0 - 12));
  }
  ret = V22FP_modem(e, N->txbits, N->out, in, N->rxbits, &N->txc, &N->rxc);
  if (m2e_saved) e->core->rxChannel = (short)m2e_saved;
  N->nblk++;
  {
    int sc1 = e->tones->sc, st = (int)(ret & 0xff), f1d = (int)((ret >> 8) & 0xff), f1e = (int)((ret >> 16) & 0xff);
    int tstate_before = ts0;
    if (((ts0 == 1 && (sc0 == 5 || sc0 == 6)) || (ts0 == 2 && sc0 == 3)) && e->core->agcIn.detect == 1 && e->core->fse.nOut == 0) cov_hit(CV_EV + EV_NODET);
    if (tstate_before == 1) {
      if (sc0 == 0 && sc1 == 1) cov_hit(CV_EV + EV_O_0_1);
      if (sc0 == 1 && sc1 == 3) cov_hit(CV_EV + EV_O_1_3);
      if (sc0 == 3 && sc1 == 4) cov_hit(CV_EV + EV_O_3_4);
      if (sc0 == 4 && sc1 == 5) cov_hit(CV_EV + EV_O_4_5);
      if (sc0 == 5 && sc1 == 6) cov_hit(CV_EV + EV_O_5_6);
      if (sc0 == 6 && sc1 == 8) cov_hit(CV_EV + EV_O_6_8);
      if (sc0 == 6 && sc1 == 0xc) cov_hit(CV_EV + EV_O_6_C);
      if (st == 0x11) cov_hit(CV_EV + EV_O_ERR1);
      if (st == 0x13) cov_hit(CV_EV + EV_O_ERR3);
      if (st == 0x14) cov_hit(CV_EV + EV_O_ERR4);
      if (sc0 == 4 && e->tones->shift == 1) cov_hit(CV_EV + EV_O_SHIFT1);
      if (f1e & 8) cov_hit(CV_EV + EV_O_F1E8);
      if (f1d & 0x10) cov_hit(CV_EV + EV_O_F1D10);
    }
    if (tstate_before == 2) {
      if (sc0 == 0 && sc1 == 1) cov_hit(CV_EV + EV_A_0_1);
      if (sc0 == 0 && sc1 == 3) cov_hit(CV_EV + EV_A_0_3);
      if (sc0 == 1 && sc1 == 0xe) cov_hit(CV_EV + EV_A_1_E);
      if (sc0 == 0xe && sc1 == 3) cov_hit(CV_EV + EV_A_E_3);
      if (sc0 == 3 && sc1 == 4) cov_hit(CV_EV + EV_A_3_4);
      if (sc0 == 3 && sc1 == 0xc) cov_hit(CV_EV + EV_A_3_C);
      if (sc0 == 4 && sc1 == 8) cov_hit(CV_EV + EV_A_4_8);
      if (st == 0x15) cov_hit(CV_EV + EV_A_ERR5);
      if (sc0 == 3 && e->tones->timer > 0x5be) cov_hit(CV_EV + EV_A_TONE3);
    }
    if (tstate_before == 1 || tstate_before == 2) {
      if (sc0 == 8 && sc1 == 9) cov_hit(CV_EV + EV_C_8_9);
      if (sc0 == 9 && sc1 == 10) cov_hit(CV_EV + EV_C_9_A);
      if (sc0 == 10 && sc1 == 11) cov_hit(CV_EV + EV_C_A_B);
      if (sc0 == 10 && st == 3) cov_hit(CV_EV + EV_C_A_DONE);
      if (sc0 == 11 && st == 3) cov_hit(CV_EV + EV_C_B_DONE);
      if (st == 0x17) cov_hit(CV_EV + EV_C_ERR7);
      if (sc0 == 12 && sc1 == 13) cov_hit(CV_EV + EV_C_C_D);
      if (sc0 == 12 && st == 4) cov_hit(CV_EV + EV_C_C_DONE);
      if (sc0 == 13 && st == 4) cov_hit(CV_EV + EV_C_D_DONE);
      if (st == 0x18) cov_hit(CV_EV + EV_C_ERR18);
      if (st == 0x10 && sc0 >= 8 && sc0 <= 11) cov_hit(CV_EV + EV_C_NOCAR24);
      if (st == 0x10 && (sc0 == 12 || sc0 == 13)) cov_hit(CV_EV + EV_C_NOCAR12);
      if (st == 0xb) cov_hit(CV_EV + EV_C_RETRAIN);
      if (sc0 >= 8 && sc0 <= 11 && e->tones->lossCnt > 0) cov_hit(CV_EV + EV_C_LOSS24);
      if ((sc0 == 12 || sc0 == 13) && e->tones->lossCnt > 0) cov_hit(CV_EV + EV_C_LOSS12);
    }
    N->prevSc = sc1;
    N->prevState = st;
  }
  return ret;
}

/* ---- the pair runner ---- */
typedef struct Trial {
  int proto;            /* 0 = V.22bis 2400, 1 = V.22 1200, 2 = Bell212 1200 */
  int dbg;              /* debug level */
  unsigned toA, toB;    /* the handshake timeouts (0 = leave the default 60000) */
  int lossA, lossB;     /* carrier loss ms (0 = default 700) */
  int nblocks;
  int gapNode, gapSc, gapLen, gapKind; /* while node `gapNode` is in sc `gapSc` its input is cut for gapLen blocks (kind 0 silence, 1 noise, 2 attenuation /8, 3 clipped x16, 4 own echo) */
  int gapAfter;         /* the gap starts after this many blocks in the state */
  int noisePct;         /* per sample noise on every input */
  int bothGaps;
  int txc;
  int tone;             /* cfg[3] of the answerer: 1 = it sends the 2100 Hz answer tone first */
  int muteA, muteB;     /* the node's output is silence from this block on (0 = never) */
  int stopData;         /* stop when both nodes have run this many data blocks */
  int accPoke;          /* poke the signal / ones accumulators to their threshold values every block (percent) */
} Trial;
static int g_tcount;
static void poke_node(Node *N, int how);
static void run_pair(const Trial *T, int tag)
{
  Node *A, *B;
  int b, i;
  char t[96];
  int sinceA = 0, sinceB = 0, lastScA = -1, lastScB = -1;
  short inA[160], inB[160];
  dsplibs_debug_level = (unsigned)T->dbg;
  g_tcount++;
  A = node_new(0, 0, T->proto, 0);
  B = node_new(1, 1, T->proto, T->tone);
  if (T->toA) A->e->tones->timeout = (int)T->toA;
  if (T->toB) B->e->tones->timeout = (int)T->toB;
  if (T->lossA) A->e->cfg.lossMs = T->lossA;
  if (T->lossB) B->e->cfg.lossMs = T->lossB;
  snprintf(t, sizeof t, "T%d", tag);
  d_engine(0, t, A->e);
  d_engine(0, t, B->e);
  for (b = 0; b < T->nblocks; b++) {
    Node *N[2];
    short *inp[2];
    if (T->muteA && b == T->muteA) A->mute = 1;
    if (T->muteB && b == T->muteB) B->mute = 1;
    if (T->stopData && A->dataBlocks >= T->stopData && B->dataBlocks >= T->stopData) break;
    N[0] = A; N[1] = B; inp[0] = inA; inp[1] = inB;
    memcpy(inA, B->mute ? (const short[160]){0} : B->peer, sizeof inA);
    memcpy(inB, A->mute ? (const short[160]){0} : A->peer, sizeof inB);
    for (i = 0; i < 2; i++) {
      Node *X = N[i];
      int sc = X->e->tones->sc, k;
      /* the gap trigger: counts the blocks spent in the trigger state */
      if (i == 0) { if (sc != lastScA) { sinceA = 0; lastScA = sc; } else sinceA++; }
      else { if (sc != lastScB) { sinceB = 0; lastScB = sc; } else sinceB++; }
      if (T->gapLen && T->gapNode == i && !X->gapDone && sc == T->gapSc && X->e->tones->state == (i ? 2 : 1) && (i == 0 ? sinceA : sinceB) >= T->gapAfter) {
        X->gapLeft = T->gapLen;
        X->gapKind = T->gapKind;
        X->gapDone = 1;
      }
      if (X->gapLeft > 0) {
        X->gapLeft--;
        switch (X->gapKind) {
          case 0: case 1: for (k = 0; k < 160; k++) inp[i][k] = 0; break;
          case 2: case 5: for (k = 0; k < 160; k++) inp[i][k] = (short)((int)(rnd() % 2001) - 1000); break;
          case 6: case 7: for (k = 0; k < 160; k++) inp[i][k] = (short)(inp[i][k] / 8); break;
          case 3: for (k = 0; k < 160; k++) { int v = inp[i][k] * 16; inp[i][k] = (short)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); } break;
          default: memcpy(inp[i], X->out, 160 * sizeof(short)); break;
        }
      }
      if (T->noisePct)
        for (k = 0; k < 160; k++) if (rpct(T->noisePct)) inp[i][k] = (short)(inp[i][k] + (int)(rnd() % 201) - 100);
    }
    if (T->accPoke)
      for (i = 0; i < 2; i++)
        if (rpct(T->accPoke)) poke_node(N[i], 16);
    /* the stand-in handlers need to find the node: both nodes are registered in g_node[0 .. 1] */
    for (i = 0; i < 2; i++) {
      unsigned ret = node_step(N[i], inp[i], T->txc ? T->txc : 12);
      emit(0, "%s b=%d n%d ret=%08x sc=%d ts=%d tx=%u rx=%u out=%016llx bits=%016llx", t, b, i, ret, N[i]->e->tones->sc, N[i]->e->tones->state, N[i]->txc, N[i]->rxc,
           fnv_shorts(N[i]->out, 160), fnv_ints(N[i]->rxbits, 100));
      if (b < 30 || (b & 7) == 0 || N[i]->prevSc != (int)N[i]->e->tones->sc || (ret & 0xff) != 1) d_engine(0, t, N[i]->e);
    }
    memcpy(A->peer, A->out, sizeof A->peer);
    memcpy(B->peer, B->out, sizeof B->peer);
  }
  emit(1, "%s end blocks=%d dataA=%d dataB=%d", t, b, A->dataBlocks, B->dataBlocks);
  d_engine(1, "end A", A->e);
  d_engine(1, "end B", B->e);
  V22FP_delete(A->e);
  V22FP_delete(B->e);
  A->e = B->e = NULL;
  trial_end("pair");
}

/* ================================================================================================ scenario 0: happy paths */
static void sc_happy(void)
{
  static const int protos[3] = {0, 1, 2};
  int p;
  Trial T;
  memset(&T, 0, sizeof T);
  for (p = 0; p < 3; p++) {
    T.proto = protos[p];
    T.dbg = p == 0 ? 2 : (p == 1 ? 1 : 2);
    T.nblocks = 200;
    T.stopData = 20;
    run_pair(&T, p);
  }
  for (p = 0; p < 3; p++) { /* the answerer sends the 2100 Hz answer tone first */
    memset(&T, 0, sizeof T);
    T.proto = protos[p];
    T.dbg = p == 1 ? 2 : 0;
    T.tone = 1;
    T.nblocks = 420;
    T.stopData = 20;
    run_pair(&T, 10 + p);
  }
}


/* ================================================================================================ scenarios 1 / 2: perturbation matrices */
static const int kind_len[9] = {3, 45, 10, 8, 6, 45, 45, 20, 130}; /* the gap kinds (gap `kind` of a Trial): 0 silence 3 blocks (carrier back soon), 1 silence 45 (carrier lost for good), 2 noise 10, 3 clipping 8, 4 own echo 6, 5 noise 45, 6 attenuation 45, 7 attenuation 20, 8 noise 130 (spans the following state) */
static void sc_matrix(int family)
{
  /* family 0: V.22bis (protocol 0); family 1: V.22 / Bell212 (protocols 1, 2) */
  static const int a24[] = {1, 3, 4, 5, 6, 8, 9, 10, 11}, b24[] = {1, 0xe, 3, 4, 8, 9, 10, 11};
  static const int a12[] = {1, 3, 4, 5, 6, 12, 13}, b12[] = {1, 0xe, 3, 12, 13};
  const int *as = family ? a12 : a24, *bs = family ? b12 : b24;
  int na = family ? 7 : 9, nb = family ? 5 : 8, node, i, j, n = 0;
  static const unsigned tos[8] = {400, 900, 1400, 2000, 3000, 4500, 6000, 9000};
  Trial T;
  for (node = 0; node < 2; node++) {
    int cnt = node ? nb : na;
    for (i = 0; i < cnt; i++) {
      int sc = node ? bs[i] : as[i];
      for (j = 0; j < 3; j++) {
        int k = (i * 3 + j + node) % 8;
        memset(&T, 0, sizeof T);
        T.proto = family ? 1 + (n & 1) : 0;
        T.dbg = (n % 3) ? 2 : 0;
        T.gapNode = node; T.gapSc = sc; T.gapKind = k; T.gapLen = kind_len[k]; T.gapAfter = j * 2;
        T.tone = (sc == 1 || sc == 0xe || (n % 4) == 0) && node == 1;
        if (!node && sc == 1) T.tone = 1;
        T.nblocks = T.tone ? 420 : 300;
        T.stopData = 6;
        run_pair(&T, 100 + n);
        n++;
      }
    }
  }

  /* targeted: the timeout errors of the individual states and the late connect states */
  {
    static const struct { int node, sc, kind, to, tone; } tg24[] = {
      {0, 3, 1, 500, 0}, {0, 3, 5, 900, 0}, {0, 3, 1, 1200, 1}, {0, 11, 5, 1900, 0}, {0, 11, 5, 2200, 0}, {1, 11, 5, 1900, 0}, {1, 11, 5, 2300, 0}, {0, 10, 5, 1900, 0}, {1, 10, 6, 2100, 0}, {0, 10, 8, 1000, 0}, {1, 10, 8, 1000, 0}, {0, 9, 8, 800, 0}, {0, 10, 0, 500, 0}, {0, 10, 0, 1000, 0}, {0, 10, 3, 500, 0}, {1, 11, 3, 500, 0}, {1, 10, 1, 500, 0}, {1, 11, 1, 500, 0},
      {1, 3, 1, 800, 0}, {1, 4, 1, 400, 0}, {0, 6, 1, 2500, 0}, {0, 4, 1, 300, 0}, {0, 5, 5, 600, 0}, {0, 1, 1, 700, 1}, {1, 1, 1, 1000, 1}},
    tg12[] = {{0, 3, 1, 500, 0}, {0, 3, 5, 900, 0}, {0, 13, 5, 1500, 0}, {1, 13, 5, 1500, 0}, {0, 13, 5, 2000, 0}, {0, 12, 5, 1100, 0}, {1, 12, 5, 1300, 0}, {1, 12, 5, 2000, 0}, {1, 3, 1, 900, 0},
      {0, 12, 6, 1700, 0}, {1, 12, 6, 1700, 0}, {0, 12, 8, 1000, 0}, {1, 12, 8, 1000, 0}, {0, 12, 8, 700, 0}, {1, 12, 1, 500, 0}, {0, 12, 1, 500, 0}, {0, 12, 8, 500, 0}, {1, 12, 8, 500, 0}, {1, 12, 5, 500, 0}, {0, 13, 1, 1000, 0}, {1, 13, 1, 1000, 0}, {0, 5, 5, 600, 0}, {1, 1, 1, 1000, 1}};
    const int cnt = family ? (int)(sizeof tg12 / sizeof tg12[0]) : (int)(sizeof tg24 / sizeof tg24[0]);
    for (i = 0; i < cnt; i++) {
      int nd = family ? tg12[i].node : tg24[i].node, sc = family ? tg12[i].sc : tg24[i].sc, kd = family ? tg12[i].kind : tg24[i].kind, to = family ? tg12[i].to : tg24[i].to,
          tn = family ? tg12[i].tone : tg24[i].tone;
      memset(&T, 0, sizeof T);
      T.proto = family ? 1 + (i & 1) : 0;
      T.dbg = (i & 1) ? 2 : 1;
      T.gapNode = nd; T.gapSc = sc; T.gapKind = kd; T.gapLen = kind_len[kd]; T.gapAfter = i % 3;
      if (nd == 0) T.toA = (unsigned)to; else T.toB = (unsigned)to;
      T.tone = tn;
      T.nblocks = tn ? 420 : 300;
      T.stopData = 6;
      run_pair(&T, 400 + i);
    }
  }

  /* threshold values of the accumulators (S1 / ones detection) poked into the running handshake */
  for (i = 0; i < 6; i++) {
    memset(&T, 0, sizeof T);
    T.proto = family ? 1 + (i & 1) : 0;
    T.dbg = i & 1;
    T.accPoke = 25 + 10 * (i % 3);
    T.tone = i & 1;
    T.nblocks = 260;
    T.stopData = 6;
    run_pair(&T, 600 + i);
  }
  /* shortened timeouts: every handshake state of each side runs out at some point */
  for (i = 0; i < 8; i++) {
    for (j = 0; j < 2; j++) {
      memset(&T, 0, sizeof T);
      T.proto = family ? 1 + (i & 1) : 0;
      T.dbg = j ? 2 : 1;
      T.toA = tos[i];
      T.toB = tos[(i + 3 + j) & 7];
      T.tone = (i + j) & 1;
      T.nblocks = T.tone ? 420 : 300;
      T.stopData = 6;
      if (j) { T.muteB = 20 + 15 * i; }
      run_pair(&T, 200 + n);
      n++;
    }
  }
}

/* ================================================================================================ scenario 3: state fuzz */
static void poke_node(Node *N, int how)
{
  static const int scs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, -1, 100, 0x7fff, -32768, 0x10003};
  V22Tones *t = N->e->tones;
  unsigned r = rnd();
  switch (how) {
    case 0: t->sc = (short)scs[r % (sizeof scs / sizeof scs[0])]; if (t->sc == 3 || t->sc == 4) t->rmsN = (short)(1 + (rnd() & 0xff)); break;
    case 1: t->timer = rpct(80) ? (int)(r % 4000u) : (int)(rpct(50) ? 0x7fffffe0u + (r & 63u) : 0xffffffe0u + (r & 31u)); break; /* also the sign bit: the compares are unsigned */
    case 2: { static const unsigned v[] = {0x1c1, 0x1c0, 0x258, 0x316, 0x2f3, 0x1a0, 0x218, 0x5be, 0x5bd, 0x61, 0x786, 0xce4, 0xce3, 0x1c2, 0x259, 0x317, 0x2f4, 0x1a1, 0x219, 0x5bf, 0x62, 0x787, 0xce5};
              t->timer = (int)(v[r % (sizeof v / sizeof v[0])] - 20u + (rnd() % 6u) - 2u + (rpct(25) ? 20u : 0u)); break; } /* the value after ReadGTimer's +20 is T-2 .. T+3 (or T+18 ..) */
    case 3: t->sigAcc = (short)(rpct(50) ? (int)(rnd() % 0x90u) : (int)(0x38u + rnd() % 12u)); break;
    case 4: t->onesAcc = (short)(rpct(50) ? (int)(rnd() % 0x100u) : (int)(0xe0u + rnd() % 12u)); break;
    case 5: t->trained = (int)(rnd() % 3u); break;
    case 6: t->lossCnt = (int)(rnd() % 50u); break;
    case 7: t->timeout = rpct(50) ? (int)(rnd() % 3000u) : (int)(0xffff0000u | (rnd() & 0xffffu)); break;
    case 8: N->e->cfg.rate2 = (short)(rpct(40) ? 0x960 : (rpct(60) ? 0x4b0 : (int)(rnd() & 0xfff))); break;
    case 9: N->e->cfg.lossMs = (int)(rpct(60) ? (int)(rnd() % 1500u) : (int)(rnd() | 0xffff0000u) + (int)(rnd() & 0xffffu)); break;
    case 10: N->e->core->agcIn.detect = (int)(rnd() % 3u); break;
    case 11: N->e->core->sre.wantTrack = (int)(rnd() % 2u); break;
    case 12: t->rmsN = (short)(1 + (rnd() & 0x3f)); t->rmsSum = (int)(rnd() % 0x20000u); t->rmsThr = (short)(rpct(50) ? 0x2454 : (int)(rnd() % 0x400u)); break;
    case 13: t->state = (short)(rpct(70) ? 1 + (int)(rnd() % 2u) : (int)(rnd() % 7u)); break;
    case 14: t->retrainLocal = (short)rnd(); t->shift = (short)(rnd() % 3u); break;
    case 15: N->e->tones->tone->r0.s[1] = (short)(rpct(70) ? (int)(rnd() % 32768u) : 0x2d43); N->e->tones->tone->r0.s[2] = (short)(rpct(50) ? 0 : (int)(rnd() % 4096u)); break; /* the tone generator's level / second word: retune_tone scales / clears them */
    case 16: { static const unsigned a[] = {0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x85, 0x86, 0x87, 0x88, 0x89, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xffff, 0x8000};
               t->sigAcc = (short)a[r % (sizeof a / sizeof a[0])]; t->onesAcc = (short)a[rnd() % (sizeof a / sizeof a[0])]; break; }
    case 17: { int thr = rpct(50) ? 0x2454 : (int)(rnd() % 0x1000u), n = 1 + (int)(rnd() % 5u), e = (int)(rnd() % 6u) - 2; /* the average of the levels at / next to the threshold (q > thr) */
               t->rmsN = (short)n; t->rmsThr = (short)thr; t->rmsSum = n * thr + e; break; }
    case 19: N->e->core->sreGate = (int)(rnd() % 2u); break;
    case 18: t->timeout = t->timer + 20 + (int)(rnd() % 5u) - 2; break; /* the handshake timeout at / next to the timer after the next ReadGTimer */
    default: N->e->state = (unsigned char)rnd(); break;
  }
}
/* poke the timer (and the timeout) so that the NEXT ReadGTimer (+20) lands at / next to the threshold of the current sub-state: T - 2 .. T + 3 */
static void poke_aligned(Node *N)
{
  V22Tones *t = N->e->tones;
  int sc = t->sc, role = t->state, d = (int)(rnd() % 6u) - 2;
  unsigned T = 0;
  if (role == 1) {
    switch (sc) { case 4: T = 0x1a0; break; case 5: T = 0x218; break; case 6: T = 0x786; break; case 1: case 3: case 11: case 13: break; default: break; }
  } else if (role == 2) {
    switch (sc) { case 1: T = 0xce4; break; case 4: T = 0x61; break; default: break; }
  }
  switch (sc) { case 8: T = 0x1c1; break; case 9: T = 0x258; break; case 10: T = 0x316; break; case 12: T = 0x2f3; break; default: break; }
  if (role != 1 && role != 2) return;
  if (role == 2 && sc == 3 && rpct(50)) { t->timer = (int)(0x5beu + (unsigned)d); return; } /* the guard tone threshold is read before ReadGTimer */
  if (T) t->timer = (int)(T - 20u + (unsigned)d);
  else if (sc == 1 || sc == 3 || sc == 11 || sc == 13) {
    t->timeout = 1000 + (int)(rnd() % 1000u);
    t->timer = t->timeout - 20 + d;
  }
}
static void sc_fuzz(void)
{
  int tr, b, i;
  char t[48];
  static short ext[160];
  for (tr = 0; tr < 28; tr++) {
    Node *N[2];
    int proto = tr % 3, tone = (tr / 3) & 1, k;
    short in[2][160];
    dsplibs_debug_level = (unsigned)(tr % 4 == 0 ? 0 : 2);
    N[0] = node_new(0, 0, proto, 0);
    N[1] = node_new(1, 1, proto, tone);
    snprintf(t, sizeof t, "Z%d", tr);
    for (b = 0; b < 260; b++) {
      for (i = 0; i < 2; i++) {
        Node *X = N[i];
        int src = (int)rmod(10);
        /* the state pokes: a few per block on random nodes */
        if (rpct(14)) poke_node(rpct(50) ? X : N[1 - i], (int)rmod(20));
        if (rpct(4)) { poke_node(X, 0); poke_node(X, 2); }
        if (rpct(10)) poke_aligned(X);
        if (rpct(3)) poke_node(X, 16);
        if (rpct(3)) poke_node(X, 18);
        memcpy(in[i], N[1 - i]->peer, sizeof in[i]);
        switch (src) {
          case 0: for (k = 0; k < 160; k++) in[i][k] = 0; break;
          case 1: for (k = 0; k < 160; k++) in[i][k] = (short)((int)(rnd() % 4001) - 2000); break;
          case 2: for (k = 0; k < 160; k++) in[i][k] = (short)rnd(); break;
          case 3: memcpy(in[i], X->out, sizeof in[i]); break; /* loopback */
          case 4: for (k = 0; k < 160; k++) in[i][k] = (short)(in[i][k] / 8 + (int)(rnd() % 7) - 3); break;
          case 5: for (k = 0; k < 160; k++) { int v = in[i][k] * 8; in[i][k] = (short)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); } break;
          case 6: memcpy(in[i], ext, sizeof in[i]); break; /* the stale block of the last noise burst */
          default: break; /* the peer's signal */
        }
        if (src == 1 || src == 2) memcpy(ext, in[i], sizeof ext);
        {
          int txc = rpct(70) ? 12 : (int)(2 * rmod(9)); /* even counts: MakeTxData pattern 0 does not terminate on an odd count */
          unsigned ret;
          static const int rxns[] = {0, 1, 2, 3, 5, 12, 13, 40, 80, 159};
          g_rxin = rpct(80) ? 160 : rxns[rmod(sizeof rxns / sizeof rxns[0])];
          ret = node_step(X, in[i], txc);
          g_rxin = 160;
          emit(0, "%s b=%d n%d ret=%08x sc=%d ts=%d tx=%u rx=%u out=%016llx bits=%016llx", t, b, i, ret, X->e->tones->sc, X->e->tones->state, X->txc, X->rxc, fnv_shorts(X->out, 160),
               fnv_ints(X->rxbits, 100));
          if (g_dense || b < 6 || (b & 3) == 0 || (ret & 0xff) != 1) d_engine(0, t, X->e);
        }
      }
      memcpy(N[0]->peer, N[0]->out, sizeof N[0]->peer);
      memcpy(N[1]->peer, N[1]->out, sizeof N[1]->peer);
    }
    emit(1, "%s end", t);
    d_engine(1, "fuzz A", N[0]->e);
    d_engine(1, "fuzz B", N[1]->e);
    V22FP_delete(N[0]->e);
    V22FP_delete(N[1]->e);
    N[0]->e = N[1]->e = NULL;
    trial_end("fuzz");
  }
}

/* ================================================================================================ scenario 4: direct calls */
static void sc_direct(void)
{
  int tr, b, i;
  char t[48];
  short dtx[128], dout[256], din[160], drx[512];
  for (tr = 0; tr < 18; tr++) {
    Node *N[2];
    int proto = tr % 3, k;
    dsplibs_debug_level = (unsigned)(tr % 3 == 0 ? 0 : 2);
    N[0] = node_new(0, 0, proto, 0);
    N[1] = node_new(1, 1, proto, (tr / 3) & 1);
    snprintf(t, sizeof t, "D%d", tr);
    for (b = 0; b < 200; b++) {
      for (i = 0; i < 2; i++) {
        Node *X = N[i];
        V22Engine *e = X->e;
        short in[160];
        memcpy(in, N[1 - i]->peer, sizeof in);
        if (b > 3 && rpct(35)) { /* a direct call of one of the handlers with a poked sub-state */
          static const int scs[] = {8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 14, 15, -1, 100, 0x7fff, -32768};
          int which = (int)rmod(4), nsc = scs[rmod(sizeof scs / sizeof scs[0])];
          short txc, rxc;
          int m2e_saved = 0;
          if (which < 2 && rpct(70)) nsc = 8 + (int)rmod(6);
          e->tones->sc = (short)nsc;
          if (which == 0 && (nsc < 8 || nsc > 11)) cov_hit(CV_C24_SC + 4);
          if (which == 1 && nsc != 12 && nsc != 13) cov_hit(CV_C12_SC + 2);
          if (nsc == 3 || nsc == 4) e->tones->rmsN = (short)(1 + (rnd() & 0xff));
          if (rpct(30)) poke_node(X, (int)(3 + rmod(10)));
          for (k = 0; k < 160; k++) { din[k] = rpct(20) ? (short)rnd() : in[k]; }
          for (k = 0; k < 256; k++) dout[k] = 0;
          for (k = 0; k < 128; k++) dtx[k] = (short)(rnd() & (e->core->txBis ? 0xf : 3));
          for (k = 0; k < 512; k++) drx[k] = 0x5a5a;
          txc = (short)(rpct(70) ? 12 : (int)(2 * rmod(9)));
          rxc = (short)(rpct(80) ? 160 : (int)(rmod(41)));
          if (rxc != 160 && e->core->rxChannel == 2) { m2e_saved = 2; e->core->rxChannel = 1; }
          switch (which) {
            case 0: connect_2400(e, dtx, dout, din, drx, &txc, &rxc); break;
            case 1: connect_1200(e, dtx, dout, din, drx, &txc, &rxc); break;
            case 2: v22_originate(e, dtx, dout, din, drx, &txc, &rxc); break;
            default: v22_answer(e, dtx, dout, din, drx, &txc, &rxc); break;
          }
          if (m2e_saved) e->core->rxChannel = (short)m2e_saved;
          emit(0, "%s b=%d n%d direct%d sc=%d state=%02x f1d=%02x f1e=%02x tx=%d rx=%d out=%016llx bits=%016llx", t, b, i, which, nsc, e->state, e->flags1d, e->flags1e, txc, rxc,
               fnv_shorts(dout, 160), fnv_shorts(drx, 100));
          memcpy(X->out, dout, 160 * sizeof(short));
          d_engine(0, t, e);
          if (e->tones->state != 1 && e->tones->state != 2) e->tones->state = (short)(i ? 2 : 1);
        } else {
          unsigned ret = node_step(X, in, 12);
          emit(0, "%s b=%d n%d ret=%08x sc=%d ts=%d out=%016llx", t, b, i, ret, e->tones->sc, e->tones->state, fnv_shorts(X->out, 160));
          if ((ret & 0xff) != 1) d_engine(0, t, e);
        }
      }
      memcpy(N[0]->peer, N[0]->out, sizeof N[0]->peer);
      memcpy(N[1]->peer, N[1]->out, sizeof N[1]->peer);
    }
    emit(1, "%s end", t);
    d_engine(1, "direct A", N[0]->e);
    d_engine(1, "direct B", N[1]->e);
    V22FP_delete(N[0]->e);
    V22FP_delete(N[1]->e);
    N[0]->e = N[1]->e = NULL;
    trial_end("direct");
  }
}

/* ================================================================================================ scenario 5: threshold sweeps */
/* One fresh engine per case, one or two blocks through V22FP_modem with poked sub-state, timer, timeout, accumulators: every comparison of the call setup states is probed at, next to
 * and across its constant (the handshake timers move in 20 ms steps, so a real handshake never lands next to a threshold). */
static void sweep_case(int role, int sc, unsigned timer, int timeout, int sigAcc, int onesAcc, int trained, int rate2, int kind, int detect, int tag)
{
  Node *N;
  short in[160];
  int k;
  char t[48];
  unsigned ret;
  N = node_new(0, role, rate2 == 0x4b0 ? 1 : 0, 0);
  snprintf(t, sizeof t, "W%d", tag);
  N->e->tones->state = (short)(role ? 2 : 1);
  N->e->tones->sc = (short)sc;
  N->e->tones->timer = (int)timer;
  N->e->tones->timeout = timeout;
  N->e->tones->sigAcc = (short)sigAcc;
  N->e->tones->onesAcc = (short)onesAcc;
  N->e->tones->trained = trained;
  N->e->tones->rmsN = 4;
  N->e->tones->rmsSum = 4 * 0x2454 + (int)(timer & 3u) - 1;
  N->e->cfg.rate2 = (short)rate2;
  N->e->core->agcIn.detect = detect;
  for (k = 0; k < 160; k++) {
    switch (kind) {
      case 0: in[k] = 0; break;
      case 1: in[k] = (short)((int)(rnd() % 401) - 200); break;
      case 3: in[k] = (short)((int)(rnd() % 40001) - 20000); break; /* loud: the AGC reports a signal; with the receive path switched off no symbol comes out */
      default: in[k] = (short)(((k * 97) & 0x7ff) - 1000); break;
    }
  }
  if (kind == 3) { /* warm-up: three loud blocks through a state that demodulates (the AGC needs a few blocks to report a signal), then a SHORT block: a signal and no symbol */
    short keep_sc = N->e->tones->sc;
    int keep_w4 = N->e->core->sreGate;
    N->e->core->sreGate = 1;
    N->e->tones->sc = 4;
    N->e->tones->timer = 0;
    for (k = 0; k < 3; k++) node_step(N, in, 12);
    N->e->tones->sc = keep_sc;
    N->e->tones->timer = (int)timer;
    N->e->core->sreGate = keep_w4;
    N->e->tones->sigAcc = (short)sigAcc;
    N->e->tones->onesAcc = (short)onesAcc;
    { static const int sh[] = {1, 2, 4, 6, 9, 12}; g_rxin = sh[(unsigned)tag % 6u]; }
  }
  ret = node_step(N, in, 12);
  g_rxin = 160;
  emit(0, "%s role=%d sc=%d t=%u ret=%08x out=%016llx bits=%016llx", t, role, sc, timer, ret, fnv_shorts(N->out, 160), fnv_ints(N->rxbits, 100));
  d_engine(0, t, N->e);
  if (rpct(50)) { /* a second block: the timer keeps counting */
    ret = node_step(N, in, 12);
    emit(0, "%s 2nd ret=%08x out=%016llx", t, ret, fnv_shorts(N->out, 160));
    d_engine(0, t, N->e);
  }
  V22FP_delete(N->e);
  N->e = NULL;
  trial_end("sweep");
}
static void sc_sweep(void)
{
  /* role, sc, threshold (0 = the timeout) */
  static const struct { int role, sc; unsigned thr; } tm[] = {
    {0, 4, 0x1a0}, {0, 5, 0x218}, {0, 6, 0x786}, {0, 1, 0}, {0, 3, 0}, {0, 8, 0x1c1}, {0, 9, 0x258}, {0, 10, 0x316}, {0, 11, 0}, {0, 12, 0x2f3}, {0, 13, 0},
    {1, 1, 0xce4}, {1, 4, 0x61}, {1, 3, 0}, {1, 8, 0x1c1}, {1, 9, 0x258}, {1, 10, 0x316}, {1, 11, 0}, {1, 12, 0x2f3}, {1, 13, 0}};
  int i, d, v, n = 0;
  dsplibs_debug_level = 2;
  for (i = 0; i < (int)(sizeof tm / sizeof tm[0]); i++)
    for (d = -3; d <= 4; d++)
      for (v = 0; v < 3; v++) {
        unsigned T = tm[i].thr, timer;
        int to = 60000, trained = (int)(rnd() % 3u), kind = v;
        if (T) timer = T - 20u + (unsigned)d;
        else { to = 1000 + (int)(rnd() % 1000u); timer = (unsigned)(to - 20 + d); }
        sweep_case(tm[i].role, tm[i].sc, timer, to, 0, 0, tm[i].sc == 10 || tm[i].sc == 12 ? (v & 1) : trained, 0x960 - (int)(rnd() % 2u) * 0x4b0, kind, (int)(rnd() % 2u), n++);
      }
  /* the answer's guard tone threshold (read before ReadGTimer), the sign bit of the timer / timeout */
  for (d = -3; d <= 4; d++) for (v = 0; v < 2; v++) sweep_case(1, 3, (unsigned)(0x5be + d), 60000, 0, 0, 0, 0x960, v, 1, n++);
  for (i = 0; i < (int)(sizeof tm / sizeof tm[0]); i++) { /* timers with the sign bit: the compares are unsigned */
    static const unsigned bt[] = {0x7fffffecu, 0x7fffffe0u, 0xffffffecu, 0xfffffff0u, 0x80000000u};
    for (v = 0; v < 5; v++) sweep_case(tm[i].role, tm[i].sc, bt[v], (int)(v & 1 ? 0x7ffffffe : 0x80000010u), 0, 0, 0, 0x960, v & 1, 1, n++);
  }
  /* the accumulators: S1 / ones detection thresholds in every state that tests them, with and without a detected signal (agcIn.detect), both rates */
  {
    static const int sig[] = {0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0xffff, 0x8000}, ones[] = {0x3a, 0x3b, 0x3c, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xffff};
    static const int sg3[] = {0x85, 0x86, 0x87, 0x88, 0x89, 0x8000};
    int a, b2;
    for (a = 0; a < 9; a++)
      for (b2 = 0; b2 < 9; b2++) {
        int v;
        for (v = 0; v < 4; v++) { /* silence, noise, silence with the detect flag poked, loud noise with the receive path off */
          int kd = v == 1 ? 1 : (v == 3 ? 3 : 0), dt = v == 2 ? 1 : (v == 0 ? 0 : (a + b2) & 1);
          sweep_case(0, 6, 20, 60000, sig[a], ones[b2], 0, 0x960, kd, dt, n++);
          sweep_case(1, 3, 20, 60000, sig[a], ones[b2], 0, (b2 & 1) ? 0x960 : 0x4b0, kd, dt, n++);
          sweep_case(0, 5, 20, 60000, sig[a], ones[b2], 0, (b2 & 1) ? 0x960 : 0x4b0, kd, dt, n++);
        }
      }
    for (a = 0; a < 6; a++)
      for (b2 = 0; b2 < 6; b2++) sweep_case(0, 3, 20, 60000, sg3[a], 0, 0, 0x960, b2 & 1, 0, n++);
  }
  /* the level average against the shift threshold: q = sum / n at, below and above rmsThr */
  {
    int e, thrs, qv;
    for (thrs = 0; thrs < 3; thrs++)
      for (e = -2; e <= 2; e++)
       for (qv = 0; qv < 2; qv++) {
        Node *N = node_new(0, 0, 0, 0);
        short in[160];
        int k, nn = 1 + thrs * 3, thr = thrs == 0 ? 0x2454 : (thrs == 1 ? 0x100 : 0x7000);
        char t[32];
        snprintf(t, sizeof t, "Q%d.%d.%d", thrs, e, qv);
        N->e->tones->state = 1; N->e->tones->sc = 3; N->e->tones->timer = 0; N->e->tones->sigAcc = 0x90; N->e->tones->rmsThr = (short)thr; N->e->tones->rmsN = (short)(nn - 1);
        /* the block adds its own level when the demodulator reports symbols; with silence it does not: keep rmsN so that the division is exact */
        N->e->tones->rmsN = (short)(nn - qv); N->e->tones->rmsSum = nn * thr + e; /* qv = 1: the block's own level (0 for silence) is added and the count is incremented first */
        for (k = 0; k < 160; k++) in[k] = 0;
        node_step(N, in, 12);
        d_engine(0, t, N->e);
        V22FP_delete(N->e);
        N->e = NULL;
        trial_end("sweepq");
      }
  }
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  g_verbose = getenv("R02_VERBOSE") != NULL;
  g_lcg = 0x2545f491u + 1000003u * (unsigned)sc;
  if (getenv("R02_SEED")) g_lcg ^= (unsigned)atoi(getenv("R02_SEED"));
  arena_init();
  g_dbg_hook = dbg_hook;
  g_hmode1 = stand_in;
  g_hs.mode = 1;
  g_hlog = 0;
  g_dense = getenv("R07_DENSE") != NULL;
  cov_setup(1);
  switch (sc) {
    case 0: sc_happy(); break;
    case 1: sc_matrix(0); break;
    case 2: sc_matrix(1); break;
    case 3: sc_fuzz(); break;
    case 4: sc_direct(); break;
    case 5: sc_sweep(); break;
    default: die3("no such scenario %d", sc);
  }
  if (getenv("R07_LISTCOV")) {
    int i;
    for (i = 0; i < COV_MAX; i++) if (g_covname[i] && g_cov[i]) fprintf(stderr, "REQ %d\n", i);
  }
  cov_require(sc);
  cov_finish();
  return 0;
}
