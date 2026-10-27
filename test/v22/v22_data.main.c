/*
 * R08 differential harness (ids r08_data, r08_loops, r08_retrain): the V.22 / V.22bis / Bell 212A DATA, LOOP-BACK and RETRAIN state handlers - v22_data, v22_local_loop,
 * v22_org_rmloop2, v22_ans_rmloop2, v22_retrain - the real dsplibs.o (32-bit) vs src/v22/v22_data.c, src/v22/v22_loopback.c, src/v22/v22_retrain.c.  ONE source for the three ids (they differ in the
 * scenario list of the script).  Library: test/v22/r06_common.h (hashed canonical output, deterministic heap, callee recorders, coverage counters exit 2, 3 = harness violation) +
 * test/v22/r06_eng_common.h (the layout independent description of the engine; R08_REAL_HANDLERS leaves ALL SEVEN handlers to the code under test, R07_REAL_CALLSETUP likewise).
 *
 * Two engines hear each other one block late through the REAL V22FP_modem (the real dispatch through V22_PROTOCOL, the real R06 chain, REAL FPM_TONE / MTD / AGC detectors, Detect_Retrain /
 * Detect_Rmloop2_ACK / Detect_1s, the call setup of R07 ...): the signals are made by the engines themselves (integer only), so the handshake, the data, the retrain pattern and the
 * remote-loopback patterns the detectors see are the real ones; perturbations (gaps of silence / noise / attenuation / echo, a muted peer, shortened timeouts, state pokes, V22FP_control
 * requests: retrain, remote digital loopback) are scripted.  Callee recorders (BOTH sides): sysdep_*, dsplibs_debug_printf (every message is part of the diffed text and counted).
 *
 * Scenarios (own process each; ids: r08_data = 0 1 5, r08_loops = 2 3 6, r08_retrain = 4 7 8 9):
 *   0 data: caller + answerer pairs V.22bis 2400, V.22 1200, Bell212 through the handshake into the data state, 100+ blocks of random data both ways, V22FP_control scrambler / descrambler /
 *     status bits, data-mode engines (the "no receive path" start, cfg[5] = 1) with supervision timeouts
 *   1 data perturbations: gaps at / after the connect (carrier loss shorter / longer than the loss time, 1200 and 2400), noise levels (signal-quality retrain), silence, attenuation, echo
 *   2 remote digital loopback: V22FP_control requests on one side, the other poked into the answering state; gaps, timeouts, protocols, roles
 *   3 local loop: loop-mode engines (cfg[0] = 2) hearing themselves / each other, the tone / detection states, timeouts, connect states reached from the loop
 *   4 retrain handshakes: requests (V22FP_control bit 2, Detect_Retrain by the peer, quality, carrier back) with gaps / timeouts in every retrain state, both roles, three protocols
 *   5 data state fuzz: random pokes on connected pairs, random input sources, all counts
 *   6 loop state fuzz: pokes on loop / RDL engines
 *   7 retrain state fuzz + direct calls
 *   8 threshold sweeps of the five handlers (fresh engine, poked state, one or two blocks)
 *   9 direct calls of the five handlers with poked sub-states
 */
#define R07_REAL_CALLSETUP 1
#define R08_REAL_HANDLERS 1
#include "r06_eng_common.h"

/* ---- coverage ids ---- */
enum {
  CV_D = 0,           /* v22_data called */
  CV_L_SC = 1,        /* 1 .. 15: local loop sc 0 .. 13, other */
  CV_O_SC = 16,       /* 16 .. 19: org_rmloop2 sc 0, 1, 2, other */
  CV_A_SC = 20,       /* 20 .. 24: ans_rmloop2 sc 0 .. 3, other */
  CV_R_SC = 25,       /* 25 .. 33: retrain sc 0 .. 7, other */
  CV_EV = 40
};
enum {
  EV_D_PLAIN, EV_D_B11, EV_D_B11_LOST, EV_D_RX24, EV_D_RX12, EV_D_REQ, EV_D_NOCAR, EV_D_SIGUP, EV_D_SIGCLR, EV_D_LOSSRST, EV_D_SCR, EV_D_DESCR, EV_D_SYNC, EV_D_QUALRT, EV_D_CARBACK,
  EV_L_0_1, EV_L_0_3, EV_L_1_2, EV_L_2_3, EV_L_3_8, EV_L_3_C, EV_L_ERR, EV_L_C24, EV_L_C12, EV_L_TONE,
  EV_O_0_1, EV_O_1_2, EV_O_1_2_TR, EV_O_1_BACK, EV_O_2_6, EV_O_2_7, EV_O_2_TR,
  EV_A_0_1, EV_A_1_2, EV_A_1_2_TR, EV_A_1_7, EV_A_2_3, EV_A_2_7, EV_A_3_8, EV_A_3_SYNC, EV_A_3_CAR,
  EV_R_0_1, EV_R_0_BACK, EV_R_1_2, EV_R_2_3, EV_R_2_4, EV_R_3_4, EV_R_3_DONE, EV_R_4_5, EV_R_5_6, EV_R_6_7, EV_R_7_TRAINED, EV_R_7_TIMEOUT, EV_R_DATA_AFTER, EV_R_SIGUP,
  EV_NEV
};
static const char *ev_names[EV_NEV] = {
  "data: normal block", "data: data-mode engine (no receive path)", "data: data-mode engine carrier lost (0x10)", "data: symbols received at 2400", "data: symbols received at 1200",
  "data: remote retrain request (state 9)", "data: no carrier (0x10)", "data: signal quality accumulator raised", "data: signal quality accumulator cleared", "data: loss counter reset",
  "data: scrambler on", "data: descrambler on", "data: TxClockSync (status bit 2)", "data: quality retrain started", "data: carrier back retrain started",
  "local loop 0 -> 1", "local loop 0 -> 3", "local loop 1 -> 2", "local loop 2 -> 3", "local loop 3 -> 8", "local loop 3 -> 0xc", "local loop error 0x16", "local loop -> connect_2400", "local loop -> connect_1200", "local loop tone detected",
  "org rmloop 0 -> 1", "org rmloop 1 -> 2", "org rmloop 1 -> 2 trained", "org rmloop 1 -> data (timeout)", "org rmloop 2 -> state 6", "org rmloop 2 -> state 7", "org rmloop 2 trained",
  "ans rmloop 0 -> 1", "ans rmloop 1 -> 2", "ans rmloop 1 -> 2 trained", "ans rmloop 1 -> state 7", "ans rmloop 2 -> 3 (state 5)", "ans rmloop 2 -> state 7", "ans rmloop 3 -> state 8 (carrier lost)", "ans rmloop 3 TxClockSync", "ans rmloop 3 carrier present",
  "retrain 0 -> 1", "retrain 0 -> data (timeout)", "retrain 1 -> 2", "retrain 2 -> 3", "retrain 2 -> 4", "retrain 3 -> 4", "retrain 3 -> done (timeout)", "retrain 4 -> 5", "retrain 5 -> 6", "retrain 6 -> 7", "retrain 7 -> done (trained)", "retrain 7 -> done (timeout)",
  "retrain -> data state", "retrain signal accumulator raised"};
/* debug message substrings (every printf of the five handlers + the call setup messages they reach) */
static const char *msg_sub[] = {
  "V22_MSG_NO_CARRIER2", "Signal quality < Retrain level", "won't be reported", "V.22: Retrain request detected", "Carrier back during carrier_loss_time (v22_data)", "V22_MSG_NO_CARRIER3", "V22_MSG_NO_CARRIER4"};
#define NMSG ((int)(sizeof msg_sub / sizeof msg_sub[0]))
#define CV_MSG (CV_EV + EV_NEV)
static void dbg_hook(const char *m)
{
  int i;
  for (i = 0; i < NMSG; i++)
    if (strstr(m, msg_sub[i])) cov_hit(CV_MSG + i);
}
static void cov_setup(void)
{
  int i;
  static char nb[NMSG + 1][80];
  cov_def(CV_D, "data handler called");
  for (i = 0; i < 14; i++) cov_defn(CV_L_SC + i, "local loop sc", i);
  cov_def(CV_L_SC + 14, "local loop sc other");
  for (i = 0; i < 3; i++) cov_defn(CV_O_SC + i, "org rmloop2 sc", i);
  cov_def(CV_O_SC + 3, "org rmloop2 sc other");
  for (i = 0; i < 4; i++) cov_defn(CV_A_SC + i, "ans rmloop2 sc", i);
  cov_def(CV_A_SC + 4, "ans rmloop2 sc other");
  for (i = 0; i < 8; i++) cov_defn(CV_R_SC + i, "retrain sc", i);
  cov_def(CV_R_SC + 8, "retrain sc other");
  for (i = 0; i < EV_NEV; i++) cov_def(CV_EV + i, ev_names[i]);
  for (i = 0; i < NMSG; i++) {
    snprintf(nb[i], sizeof nb[i], "message \"%s\"", msg_sub[i]);
    cov_def(CV_MSG + i, nb[i]);
  }
}
/* The counters each scenario must reach (listed from a run: R08_LISTCOV=1 prints the reached ids on stderr); every other counter is dropped for the scenario. */
#include "r08_req.h"
static void cov_require(int sc)
{
  unsigned k;
  int i, keep;
  if (sc < 0 || sc >= R08_NSC) return;
  for (i = 0; i < COV_MAX; i++) {
    keep = 0;
    for (k = 0; k < r08_req_n[sc]; k++) if (r08_req[sc][k] == i) keep = 1;
    if (!keep) g_covname[i] = NULL;
  }
}

static int g_rxin = 160; /* the number of input samples handed to V22FP_modem */
static int g_dense;

/* ---- nodes ---- */
typedef struct Node {
  V22Engine *e;
  int role;                 /* 0 caller, 1 answerer, 2 loop mode */
  int nblk;
  short peer[160];          /* what the other side sent in its previous block */
  short out[256];
  unsigned txc, rxc;
  int pad_before[64];
  int txbits[128], rxbits[512];
  int pad_after[64];
  int dataBlocks, rxsyms;
  int gapLeft[2], gapKind[2], gapDone[2];
  int prevSc, prevState;
  int mute;
  int sinceEnter, lastTs, lastSc;
  int tmrDone;
} Node;
static Node g_node[4];

static Node *node_new(int slot, int role, int proto, int cfg3, int dataStart)
{
  Node *N = &g_node[slot];
  int cfg[7];
  memset(N, 0, sizeof *N);
  N->role = role;
  N->lastTs = N->lastSc = -99;
  cfg[0] = role; cfg[1] = proto; cfg[2] = 60000; cfg[3] = cfg3; cfg[4] = 700; cfg[5] = dataStart; cfg[6] = 1;
  N->e = V22FP_create(NULL, cfg);
  return N;
}

static void ctl_send(V22Engine *e, unsigned c, unsigned d)
{
  unsigned char ctl[16];
  memset(ctl, 0, sizeof ctl);
  ctl[0xc] = (unsigned char)c;
  ctl[0xd] = (unsigned char)d;
  (void)V22FP_control(e, ctl);
}

/* BLOB BUG (found by this harness, present in dsplibs.o and kept by the port): TxClockSync (status bit 2 = core.w20, V22FP_control bit 2 / the creator's cfg bit) copies 3 * sre.integ into the PPS phase step; V22_PPS_filter
 * advances its phase by (short)step + 3 per output sample and only consumes a symbol when the phase passes 0x27, so a step <= -3 (a negative PLL integrator) makes the loop run until it overwrites the heap / crashes (a step above 0x25 lets the phase
 * grow past the 0x28-entry coefficient table: reads beyond the buffer).  The harness keeps the step in the range -2 .. 0x25 (the same on both sides). */
static void pps_guard(V22Engine *e)
{
  short st = (short)e->core->pps.cfg.c0.s[0];
  if (st < -2 || st > 0x25) e->core->pps.cfg.c0.s[0] = 0;
}
/* one block of one node */
static unsigned node_step(Node *N, const short *in, int txc)
{
  V22Engine *e = N->e;
  int i, mask = e->core->txBis ? 0xf : 3, sc0 = e->tones->sc, ts0 = e->tones->state, m2e_saved = 0;
  int lossCnt0 = e->tones->lossCnt, sig0 = (unsigned short)e->tones->sigAcc, b11 = e->cfg.b11, rxBis0 = e->core->rxBis, w20 = e->core->w20, scr = e->core->scrOn, descr = e->core->descrOn;
  int trained0 = e->tones->trained;
  unsigned ret;
  for (i = 0; i < 128; i++) N->txbits[i] = (int)(rnd() & (unsigned)mask);
  for (i = 0; i < 512; i++) N->rxbits[i] = 0x5a5a;
  for (i = 0; i < 256; i++) N->out[i] = 0;
  N->txc = (unsigned)txc;
  pps_guard(e);
  if (g_rxin != 160 && e->core->rxChannel == 2) { e->core->rxChannel = 1; m2e_saved = 2; }
  N->rxc = (unsigned)g_rxin;
  if (ts0 == 0) cov_hit(CV_D);
  if (ts0 == 3) cov_hit(CV_L_SC + (sc0 >= 0 && sc0 <= 13 ? sc0 : 14));
  if (ts0 == 4) cov_hit(CV_O_SC + (sc0 >= 0 && sc0 <= 2 ? sc0 : 3));
  if (ts0 == 5) cov_hit(CV_A_SC + (sc0 >= 0 && sc0 <= 3 ? sc0 : 4));
  if (ts0 == 6) cov_hit(CV_R_SC + (sc0 >= 0 && sc0 <= 7 ? sc0 : 8));
  ret = V22FP_modem(e, N->txbits, N->out, in, N->rxbits, &N->txc, &N->rxc);
  if (m2e_saved) e->core->rxChannel = (short)m2e_saved;
  N->nblk++;
  {
    int sc1 = e->tones->sc, ts1 = e->tones->state, st = (int)(ret & 0xff), f1d = (int)((ret >> 8) & 0xff);
    int sig1 = (unsigned short)e->tones->sigAcc, loss1 = e->tones->lossCnt, trained1 = e->tones->trained;
    if (ts0 == 0) {
      if (b11 & 8) {
        cov_hit(CV_EV + EV_D_B11);
        if (st == 0x10) cov_hit(CV_EV + EV_D_B11_LOST);
      } else {
        if (st == 0 && ts1 == 0) cov_hit(CV_EV + EV_D_PLAIN);
        if (N->rxc > 0 || e->core->fse.nOut > 0) cov_hit(CV_EV + (e->core->rxBis ? EV_D_RX24 : EV_D_RX12));
        if (N->rxsyms >= 0 && e->core->fse.nOut > 0) N->rxsyms++;
        if (st == 9 && ts1 == 6 && sc1 == 0) cov_hit(CV_EV + EV_D_REQ);
        if (st == 0x10) cov_hit(CV_EV + EV_D_NOCAR);
        if (rxBis0 == 1 && sig1 > sig0) cov_hit(CV_EV + EV_D_SIGUP);
        if (rxBis0 == 1 && sig0 != 0 && sig1 == 0) cov_hit(CV_EV + EV_D_SIGCLR);
        if (lossCnt0 > 0 && loss1 == 0 && st != 0xb) cov_hit(CV_EV + EV_D_LOSSRST);
        if (scr) cov_hit(CV_EV + EV_D_SCR);
        if (descr) cov_hit(CV_EV + EV_D_DESCR);
        if (w20) cov_hit(CV_EV + EV_D_SYNC);
        if (st == 0xb && sc1 == 1 && ts1 == 6 && loss1 == 0 && lossCnt0 > 0) cov_hit(CV_EV + EV_D_CARBACK);
        if (st == 0xb && sc1 == 1 && ts1 == 6 && sig1 > 0xbb8) cov_hit(CV_EV + EV_D_QUALRT);
      }
    }
    if (ts0 == 3) {
      if (sc0 == 0 && sc1 == 1) cov_hit(CV_EV + EV_L_0_1);
      if (sc0 == 0 && sc1 == 3) cov_hit(CV_EV + EV_L_0_3);
      if (sc0 == 1 && sc1 == 2) cov_hit(CV_EV + EV_L_1_2);
      if (sc0 == 2 && sc1 == 3) cov_hit(CV_EV + EV_L_2_3);
      if (sc0 == 3 && sc1 == 8) cov_hit(CV_EV + EV_L_3_8);
      if (sc0 == 3 && sc1 == 0xc) cov_hit(CV_EV + EV_L_3_C);
      if (st == 0x16) cov_hit(CV_EV + EV_L_ERR);
      if (sc0 >= 8 && sc0 <= 11) cov_hit(CV_EV + EV_L_C24);
      if (sc0 == 12 || sc0 == 13) cov_hit(CV_EV + EV_L_C12);
      if (sc0 == 2 && sc1 == 3) cov_hit(CV_EV + EV_L_TONE);
    }
    if (ts0 == 4) {
      if (sc0 == 0 && sc1 == 1) cov_hit(CV_EV + EV_O_0_1);
      if (sc0 == 1 && sc1 == 2) { cov_hit(CV_EV + EV_O_1_2); if (trained1 == 1 && trained0 == 0) cov_hit(CV_EV + EV_O_1_2_TR); }
      if (sc0 == 1 && ts1 == 0 && sc1 == 0) cov_hit(CV_EV + EV_O_1_BACK);
      if (sc0 == 2 && st == 6) cov_hit(CV_EV + EV_O_2_6);
      if (sc0 == 2 && st == 7) cov_hit(CV_EV + EV_O_2_7);
      if (sc0 == 2 && trained0 == 0 && trained1 == 1) cov_hit(CV_EV + EV_O_2_TR);
    }
    if (ts0 == 5) {
      if (sc0 == 0 && sc1 == 1) cov_hit(CV_EV + EV_A_0_1);
      if (sc0 == 1 && sc1 == 2) { cov_hit(CV_EV + EV_A_1_2); if (trained1 == 1 && trained0 == 0) cov_hit(CV_EV + EV_A_1_2_TR); }
      if (sc0 == 1 && st == 7) cov_hit(CV_EV + EV_A_1_7);
      if (sc0 == 2 && sc1 == 3) cov_hit(CV_EV + EV_A_2_3);
      if (sc0 == 2 && st == 7) cov_hit(CV_EV + EV_A_2_7);
      if (sc0 == 3 && st == 8) cov_hit(CV_EV + EV_A_3_8);
      if (sc0 == 3 && w20) cov_hit(CV_EV + EV_A_3_SYNC);
      if (sc0 == 3 && sig1 == 0) cov_hit(CV_EV + EV_A_3_CAR);
    }
    if (ts0 == 6) {
      if (sc0 == 0 && sc1 == 1) cov_hit(CV_EV + EV_R_0_1);
      if (sc0 == 0 && ts1 == 0) cov_hit(CV_EV + EV_R_0_BACK);
      if (sc0 == 1 && sc1 == 2) cov_hit(CV_EV + EV_R_1_2);
      if (sc0 == 2 && sc1 == 3) cov_hit(CV_EV + EV_R_2_3);
      if (sc0 == 2 && sc1 == 4) cov_hit(CV_EV + EV_R_2_4);
      if (sc0 == 3 && sc1 == 4) cov_hit(CV_EV + EV_R_3_4);
      if (sc0 == 3 && st == 3) cov_hit(CV_EV + EV_R_3_DONE);
      if (sc0 == 4 && sc1 == 5) cov_hit(CV_EV + EV_R_4_5);
      if (sc0 == 5 && sc1 == 6) cov_hit(CV_EV + EV_R_5_6);
      if (sc0 == 6 && sc1 == 7) cov_hit(CV_EV + EV_R_6_7);
      if (sc0 == 7 && st == 3 && (f1d & 0x28) == 0x28) cov_hit(CV_EV + EV_R_7_TRAINED);
      if (sc0 == 7 && st == 3 && (f1d & 0x28) != 0x28) cov_hit(CV_EV + EV_R_7_TIMEOUT);
      if (ts1 == 0 && st != 0) cov_hit(CV_EV + EV_R_DATA_AFTER);
      if ((sc0 == 0 || sc0 == 2 || sc0 == 3) && sig1 > sig0) cov_hit(CV_EV + EV_R_SIGUP);
    }
    N->prevSc = sc1;
    N->prevState = st;
    if (ts1 != N->lastTs || sc1 != N->lastSc) { N->lastTs = ts1; N->lastSc = sc1; N->sinceEnter = 0; } else N->sinceEnter++;
  }
  return ret;
}

/* ---- the pair runner ---- */
typedef struct Trial {
  int proto;            /* 0 = V.22bis 2400, 1 = V.22 1200, 2 = Bell212 1200 */
  int dbg;
  unsigned toA, toB;    /* handshake timeouts (0 = default 60000) */
  int lossA, lossB;     /* carrier loss ms (0 = default 700) */
  int w8A, w8B;         /* cfg.handshakeTimeoutMs (data-mode supervision limit; 0 = leave) */
  int nblocks;
  int gapNode, gapTs, gapSc, gapLen, gapKind, gapAfter, gapSkip; /* while node `gapNode` is in (state gapTs, sc gapSc) its input is cut for gapLen blocks (kind 0/1 silence, 2/5 noise, 3 clipped x16, 4 own echo, 6/7 attenuation) after gapAfter blocks in the state; gapSkip = ignore gapTs/gapSc, use the block number gapAfter */
  int gap2Node, gap2Ts, gap2Sc, gap2Len, gap2Kind, gap2After, gap2Skip; /* a second gap, same meaning */
  int noisePct, noiseAmp;
  int selfLoop;         /* node A hears its own previous output (local loop), node B hears A */
  int txc;
  int tone;
  int muteA, muteB;
  int stopData;         /* stop when both nodes have run this many data blocks */
  int dataStart;        /* the engines start in the data state (cfg[5] = 1) */
  int ctlNode, ctlBlock, ctlC, ctlD; /* V22FP_control(node, ctl[0xc] = ctlC, ctl[0xd] = ctlD) at this block (ctlD 0 = none) */
  int ctl2Node, ctl2Block, ctl2C, ctl2D;
  int pokeNode, pokeBlock, pokeTs, pokeSc; /* poke tones.state / sc of a node at a block (pokeTs < 0 = none) */
  int tmrNode, tmrTs, tmrSc, tmrVal, tmrTo;  /* when the node enters (state, sc): timer = tmrVal (tmrTs < 0: none); tmrTo != 0: also tones.timeout */
  int errNode, errBlock, errLen, errVal; /* core.fse.errPow = errVal before every block of the node for errLen blocks from errBlock (errLen 0 = none) */
  int lossNode, lossBlock, lossVal;      /* tones.lossCnt = lossVal at this block (lossBlock 0 = none) */
  int dataMode;         /* every engine runs after connecting with the status flags of V22FP_control c = this (0 = none) */
  int runBlocks;        
} Trial;
static void run_pair(const Trial *T, int tag)
{
  Node *A, *B;
  int b, i, k, g;
  char t[96];
  short inA[160], inB[160];
  int done[4] = {0, 0, 0, 0};
  dsplibs_debug_level = (unsigned)T->dbg;
  A = node_new(0, T->proto < 0 ? 2 : 0, T->proto < 0 ? -T->proto - 1 : T->proto, (T->tone >> 1) & 1, T->dataStart);
  B = node_new(1, T->proto < 0 ? 2 : 1, T->proto < 0 ? -T->proto - 1 : T->proto, T->tone & 1, T->dataStart);
  if (T->toA) A->e->tones->timeout = (int)T->toA;
  if (T->toB) B->e->tones->timeout = (int)T->toB;
  if (T->lossA) A->e->cfg.lossMs = T->lossA;
  if (T->lossB) B->e->cfg.lossMs = T->lossB;
  if (T->w8A) A->e->cfg.handshakeTimeoutMs = T->w8A;
  if (T->w8B) B->e->cfg.handshakeTimeoutMs = T->w8B;
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
    if (T->selfLoop) memcpy(inA, A->peer, sizeof inA);
    /* the scripted events fire when the node has run that many blocks in the DATA state (dataBlocks), once */
    if (T->ctlD && !done[0] && N[T->ctlNode]->dataBlocks >= T->ctlBlock) { done[0] = 1; ctl_send(N[T->ctlNode]->e, (unsigned)T->ctlC, (unsigned)T->ctlD); }
    if (T->ctl2D && !done[1] && N[T->ctl2Node]->dataBlocks >= T->ctl2Block) { done[1] = 1; ctl_send(N[T->ctl2Node]->e, (unsigned)T->ctl2C, (unsigned)T->ctl2D); }
    if (T->pokeTs >= 0 && !done[2] && N[T->pokeNode]->dataBlocks >= T->pokeBlock) {
      done[2] = 1;
      N[T->pokeNode]->e->tones->state = (short)T->pokeTs; N[T->pokeNode]->e->tones->sc = (short)T->pokeSc; N[T->pokeNode]->e->tones->timer = 0;
    }
    if (T->lossBlock && !done[3] && N[T->lossNode]->dataBlocks >= T->lossBlock) { done[3] = 1; N[T->lossNode]->e->tones->lossCnt = T->lossVal; }
    for (i = 0; i < 2; i++) {
      Node *X = N[i];
      int ts = X->e->tones->state, sc = X->e->tones->sc;
      if (T->tmrTs >= 0 && T->tmrNode == i && !X->tmrDone && ts == T->tmrTs && sc == T->tmrSc) {
        X->e->tones->timer = T->tmrVal;
        if (T->tmrTo) X->e->tones->timeout = T->tmrTo;
        X->tmrDone = 1;
      }
      for (g = 0; g < 2; g++) {
        int gn = g ? T->gap2Node : T->gapNode, gts = g ? T->gap2Ts : T->gapTs, gsc = g ? T->gap2Sc : T->gapSc, glen = g ? T->gap2Len : T->gapLen, gkind = g ? T->gap2Kind : T->gapKind,
            gafter = g ? T->gap2After : T->gapAfter, gskip = g ? T->gap2Skip : T->gapSkip;
        if (glen && gn == i && !X->gapDone[g] && (gskip ? X->dataBlocks >= gafter : (ts == gts && sc == gsc && X->sinceEnter >= gafter))) {
          X->gapLeft[g] = glen;
          X->gapKind[g] = gkind;
          X->gapDone[g] = 1;
        }
        if (X->gapLeft[g] > 0) {
          X->gapLeft[g]--;
          switch (X->gapKind[g]) {
            case 0: case 1: for (k = 0; k < 160; k++) inp[i][k] = 0; break;
            case 2: case 5: for (k = 0; k < 160; k++) inp[i][k] = (short)((int)(rnd() % 2001) - 1000); break;
            case 6: case 7: for (k = 0; k < 160; k++) inp[i][k] = (short)(inp[i][k] / 8); break;
            case 3: for (k = 0; k < 160; k++) { int v = inp[i][k] * 16; inp[i][k] = (short)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); } break;
            default: memcpy(inp[i], X->out, 160 * sizeof(short)); break;
          }
        }
      }
      if (T->noisePct) {
        int amp = T->noiseAmp ? T->noiseAmp : 100;
        for (k = 0; k < 160; k++) if (rpct(T->noisePct)) inp[i][k] = (short)(inp[i][k] + (int)(rnd() % (unsigned)(2 * amp + 1)) - amp);
      }
    }
    for (i = 0; i < 2; i++) {
      unsigned ret;
      int ts0 = N[i]->e->tones->state;
      if (T->errLen && T->errNode == i && N[i]->dataBlocks >= T->errBlock && N[i]->dataBlocks < T->errBlock + T->errLen) N[i]->e->core->fse.errPow = (short)T->errVal;
      ret = node_step(N[i], inp[i], T->txc ? T->txc : 12);
      if (ts0 == 0) N[i]->dataBlocks++;
      emit(0, "%s b=%d n%d ret=%08x sc=%d ts=%d tx=%u rx=%u out=%016llx bits=%016llx", t, b, i, ret, N[i]->e->tones->sc, N[i]->e->tones->state, N[i]->txc, N[i]->rxc,
           fnv_shorts(N[i]->out, 160), fnv_ints(N[i]->rxbits, 100));
      if (getenv("R08_DUMP") && b == atoi(getenv("R08_DUMP")) && tag == 27) { int q; for (q = 0; q < 160; q++) emit(1, "OUT%d %d", q, N[i]->out[q]); }
      if (g_dense || b < 30 || (b & 7) == 0 || N[i]->prevSc != (int)N[i]->e->tones->sc || (ret & 0xff) != 1) d_engine(0, t, N[i]->e);
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
static void trial_init(Trial *T)
{
  memset(T, 0, sizeof *T);
  T->pokeTs = -1;
  T->tmrTs = -1;
}

/* ================================================================================================ scenario 0: data */
static void sc_data(void)
{
  static const int cs[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x0f, 0x80, 0x87, 0xff, 0x81, 0x84, 0x86};
  int p, i, n = 0;
  Trial T;
  for (p = 0; p < 3; p++) {
    trial_init(&T);
    T.proto = p; T.dbg = p == 1 ? 1 : 2; T.nblocks = 500; T.stopData = 60;
    run_pair(&T, p);
  }
  for (p = 0; p < 3; p++) { /* the answerer sends the 2100 Hz answer tone first; fewer data blocks, odd counts */
    trial_init(&T);
    T.proto = p; T.dbg = p == 1 ? 2 : 0; T.tone = 1; T.nblocks = 700; T.stopData = 30; T.txc = p == 2 ? 6 : 12;
    run_pair(&T, 10 + p);
  }
  for (i = 0; i < 16; i++) { /* V22FP_control: scrambler / descrambler / sync / agc / quality bits on both sides at different times */
    trial_init(&T);
    T.proto = i % 3; T.dbg = (i & 1) ? 2 : 0; T.nblocks = 500; T.stopData = 40;
    T.ctlNode = i & 1; T.ctlBlock = 5 + i; T.ctlC = cs[i]; T.ctlD = 1;
    T.ctl2Node = !(i & 1); T.ctl2Block = 12 + 2 * i; T.ctl2C = cs[(i * 5 + 3) & 15]; T.ctl2D = 1;
    run_pair(&T, 20 + i);
  }
  /* data-mode engines (cfg[5] = 1): no receive path; the supervision of the blocks against cfg.handshakeTimeoutMs (60000 = never within the run) */
  for (p = 0; p < 3; p++) {
    static const int w8s[] = {0, 100, 41, 2000};
    for (i = 0; i < 4; i++) {
      trial_init(&T);
      T.proto = p; T.dbg = (p + i) & 1 ? 2 : 0; T.dataStart = 1; T.nblocks = 120; T.w8A = w8s[i]; T.w8B = w8s[(i + 1) & 3];
      T.ctlNode = 0; T.ctlBlock = 3; T.ctlC = cs[(i + p) & 15]; T.ctlD = 1;
      run_pair(&T, 100 + n++);
    }
  }
  /* data mode with the loss counter poked (the blocks counted so far) */
  for (i = 0; i < 6; i++) {
    trial_init(&T);
    T.proto = i % 3; T.dbg = 2; T.dataStart = 1; T.nblocks = 40; T.w8A = 20 * (i + 1) * 10; T.lossNode = 0; T.lossBlock = 4; T.lossVal = 9 + i * 3;
    run_pair(&T, 120 + i);
  }
}

/* ================================================================================================ scenario 1: data perturbations */
static void sc_dataperturb(void)
{
  static const int lens[] = {1, 5, 14, 15, 16, 20, 34, 35, 36, 50};
  static const int losses[] = {0, 100, 300, 1000};
  int p, node, i, n = 0, j;
  Trial T;
  for (p = 0; p < 3; p++)
    for (node = 0; node < 2; node++)
      for (i = 0; i < 10; i++) { /* carrier gaps of every length around the 14-block carrier-back limit and the 35-block loss time */
        trial_init(&T);
        T.proto = p; T.dbg = (n % 3) ? 2 : 0;
        T.gapNode = node; T.gapSkip = 1; T.gapAfter = 8 + (i % 4); T.gapLen = lens[i]; T.gapKind = 0;
        T.lossA = losses[(i + n) & 3]; T.lossB = losses[(i + n / 2) & 3];
        T.nblocks = 700; T.stopData = 8 + lens[i] + 70;
        run_pair(&T, 200 + n++);
      }
  /* other disturbances in the data state: noise bursts of every size, attenuation, clipping, echo, a muted peer, long gaps (quality retrain needs 150 poor blocks) */
  for (p = 0; p < 3; p++)
    for (node = 0; node < 2; node++)
      for (j = 0; j < 8; j++) {
        static const int kinds[] = {2, 5, 6, 7, 3, 4, 2, 5};
        static const int lns[] = {10, 45, 45, 20, 8, 6, 170, 200};
        trial_init(&T);
        T.proto = p; T.dbg = (n & 1) ? 2 : 1;
        T.gapNode = node; T.gapSkip = 1; T.gapAfter = 10 + j; T.gapLen = lns[j]; T.gapKind = kinds[j];
        T.nblocks = 800; T.stopData = 10 + lns[j] + 80;
        if (j == 7) T.lossA = T.lossB = 20000;
        run_pair(&T, 300 + n++);
      }
  /* sustained moderate noise (the amplitude sets the signal quality) from the start: handshake + data under noise */
  for (p = 0; p < 3; p++)
    for (i = 0; i < 6; i++) {
      static const int amps[] = {30, 120, 300, 600, 1000, 2000};
      trial_init(&T);
      T.proto = p; T.dbg = (i & 1) ? 2 : 0; T.noisePct = 60 + 8 * i; T.noiseAmp = amps[i];
      T.nblocks = 600; T.stopData = 120;
      run_pair(&T, 500 + n++);
    }
  /* the quality error power poked high for a while (the quality threshold 0x7f37) */
  for (p = 0; p < 3; p++)
    for (i = 0; i < 6; i++) {
      static const int evs[] = {0x30, 0xc8, 0xc9, 0xca, 0x400, -1};
      trial_init(&T);
      T.proto = p; T.dbg = (i & 1) ? 2 : 0; T.errNode = i & 1; T.errBlock = 6; T.errLen = 190 + i * 3; T.errVal = evs[i];
      T.nblocks = 700; T.stopData = 8 + T.errLen + 60;
      run_pair(&T, 700 + n++);
    }
  /* muted peers (no carrier for good) and shortened carrier-loss times */
  for (p = 0; p < 3; p++)
    for (i = 0; i < 4; i++) {
      trial_init(&T);
      T.proto = p; T.dbg = (i & 1) ? 2 : 0; T.nblocks = 400; T.stopData = 70;
      if (i & 1) T.muteA = 170 + 30 * i; else T.muteB = 170 + 30 * i;
      T.lossA = T.lossB = 100 * (i + 1);
      run_pair(&T, 800 + n++);
    }
}

/* ================================================================================================ scenario 2: remote digital loopback */
static void sc_rdl(void)
{
  int p, i, n = 0;
  Trial T;
  for (p = 0; p < 3; p++)
    for (i = 0; i < 12; i++) {
      /* node A is asked for the remote digital loopback (V22FP_control top bits = 2 -> originating state 4); node B enters the answering state 5 at the same time / earlier / later (or not at all) */
      trial_init(&T);
      T.proto = p; T.dbg = (i % 3) ? 2 : 0;
      T.ctlNode = (i & 8) ? 1 : 0; T.ctlBlock = 10; T.ctlC = 0x07; T.ctlD = 0x80;
      T.pokeNode = (i & 8) ? 0 : 1; T.pokeBlock = (i % 8 < 5) ? 10 + (i % 5) - 2 : 10; T.pokeTs = 5; T.pokeSc = 0;
      if (i % 8 == 5) T.pokeTs = -1;       /* nobody answers */
      if (i % 8 == 6) T.pokeBlock = 60;     /* the answerer comes late */
      if (i % 8 == 7) { T.ctl2Node = (i & 8) ? 0 : 1; T.ctl2Block = 10; T.ctl2C = 0x07; T.ctl2D = 0x80; T.pokeTs = -1; } /* both ask */
      T.nblocks = 800; T.stopData = 400;
      run_pair(&T, n++);
    }
  /* perturbations of the loopback states: gaps in every sub-state of both roles, timers aligned to the thresholds */
  for (p = 0; p < 3; p++) {
    int ts, sc;
    for (ts = 4; ts <= 5; ts++)
      for (sc = 0; sc <= (ts == 4 ? 2 : 3); sc++)
        for (i = 0; i < 3; i++) {
          static const int kinds[] = {1, 5, 6};
          static const int lns[] = {3, 30, 12};
          trial_init(&T);
          T.proto = p; T.dbg = (i == 1) ? 2 : 0;
          T.ctlNode = 0; T.ctlBlock = 10; T.ctlC = 0x07; T.ctlD = 0x80;
          T.pokeNode = 1; T.pokeBlock = 10; T.pokeTs = 5; T.pokeSc = 0;
          T.gapNode = ts == 4 ? 0 : 1; T.gapTs = ts; T.gapSc = sc; T.gapAfter = i; T.gapKind = kinds[i]; T.gapLen = lns[i];
          T.nblocks = 700; T.stopData = 300;
          run_pair(&T, 100 + n++);
        }
  }
  {
    /* timers at / next to the thresholds of the loopback states (0x514 org sc 1 / 2, ans sc 1; 0x1388 ans sc 2): the timer is poked when the node enters the sub-state */
    static const struct { int node, ts, sc; unsigned thr; } tm[] = {{0, 4, 1, 0x514}, {0, 4, 2, 0x514}, {1, 5, 1, 0x514}, {1, 5, 2, 0x1388}};
    int d, k;
    for (p = 0; p < 3; p++)
      for (k = 0; k < 4; k++)
        for (d = -3; d <= 4; d++) {
          trial_init(&T);
          T.proto = p; T.dbg = (d & 1) ? 2 : 0;
          T.ctlNode = 0; T.ctlBlock = 10; T.ctlC = 0x07; T.ctlD = 0x80;
          T.pokeNode = 1; T.pokeBlock = 10; T.pokeTs = 5; T.pokeSc = 0;
          T.tmrNode = tm[k].node; T.tmrTs = tm[k].ts; T.tmrSc = tm[k].sc; T.tmrVal = (int)(tm[k].thr - 20u + (unsigned)d);
          T.nblocks = 500; T.stopData = 120;
          run_pair(&T, 300 + n++);
        }
  }
}

/* ================================================================================================ scenario 3: local loop */
static void sc_local(void)
{
  int p, i, n = 0, k, d;
  Trial T;
  for (p = 0; p < 3; p++)
    for (i = 0; i < 6; i++) { /* loop-mode engines: pairs hearing each other / one hearing itself */
      trial_init(&T);
      T.proto = -(p + 1); T.dbg = (i & 1) ? 2 : 0; T.nblocks = 520; T.selfLoop = i >= 3; T.tone = i % 3;
      if (i == 1 || i == 4) { T.toA = 900 + 400 * (unsigned)p; T.toB = 700; }
      if (i == 2) T.muteB = 30;
      run_pair(&T, n++);
    }
  /* the tone detection: node A is moved to sc 2 while B is still sending its tone */
  for (p = 0; p < 3; p++)
    for (d = -3; d <= 4; d++) {
      trial_init(&T);
      T.proto = -(p + 1); T.dbg = (d & 1) ? 2 : 0; T.nblocks = 420; T.tmrNode = 0; T.tmrTs = 3; T.tmrSc = 1; T.tmrVal = (int)(0xce4u - 20u + (unsigned)d);
      run_pair(&T, 100 + n++);
    }
  /* timers at the thresholds of each loop state, gaps in each */
  {
    static const struct { int sc; unsigned thr; } tm[] = {{1, 0xce4}, {3, 0}, {8, 0x1c1}, {9, 0x258}, {10, 0x316}, {12, 0x2f3}};
    for (p = 0; p < 3; p++)
      for (k = 0; k < 6; k++)
        for (d = -2; d <= 3; d++) {
          trial_init(&T);
          T.proto = -(p + 1); T.dbg = (d & 1) ? 2 : 0; T.nblocks = 450; T.tone = d & 1;
          T.tmrNode = d & 1; T.tmrTs = 3; T.tmrSc = tm[k].sc;
          if (tm[k].thr) T.tmrVal = (int)(tm[k].thr - 20u + (unsigned)d); else { T.tmrTo = 1000 + 20 * p; T.tmrVal = T.tmrTo - 20 + d; }
          if (k >= 2 && (d & 2)) { T.gapNode = d & 1; T.gapTs = 3; T.gapSc = tm[k].sc; T.gapAfter = 1; T.gapKind = 5; T.gapLen = 25; }
          run_pair(&T, 200 + n++);
        }
    for (p = 0; p < 3; p++)
      for (k = 0; k < 8; k++) {
        static const int sc[] = {0, 1, 2, 3, 8, 9, 10, 12};
        trial_init(&T);
        T.proto = -(p + 1); T.dbg = 2; T.nblocks = 420; T.tone = k & 1; T.selfLoop = (k >> 1) & 1;
        T.gapNode = k & 1; T.gapTs = 3; T.gapSc = sc[k]; T.gapAfter = 2; T.gapKind = k % 3 ? 1 : 5; T.gapLen = 40;
        run_pair(&T, 400 + n++);
      }
  }
}

/* ================================================================================================ scenario 4: retrain handshakes */
static void sc_retrain(void)
{
  static const int kinds[] = {1, 5, 6, 2, 3, 4};
  static const int lns[] = {3, 40, 12, 10, 8, 6};
  int p, k, i, n = 0, d;
  Trial T;
  /* requested retrains: V22FP_control bit 2 on one node, the peer follows through Detect_Retrain; with and without gaps in each retrain state of either node */
  for (p = 0; p < 3; p++)
    for (k = 0; k < 8; k++)
      for (i = 0; i < 4; i++) {
        trial_init(&T);
        T.proto = p; T.dbg = ((k + i) & 1) ? 2 : 0;
        T.ctlNode = i & 1; T.ctlBlock = 12; T.ctlC = 0x87; T.ctlD = 0x04;
        if (i == 3) { T.ctl2Node = 1 - T.ctlNode; T.ctl2Block = 12 + k; T.ctl2C = 0x87; T.ctl2D = 0x04; } /* both ask */
        T.gapNode = (i >> 1) ^ (i & 1); T.gapTs = 6; T.gapSc = k; T.gapAfter = i; T.gapKind = kinds[(k + i) % 6]; T.gapLen = lns[(k + i) % 6];
        T.nblocks = 700; T.stopData = 140;
        run_pair(&T, n++);
      }
  /* locally started retrains: the carrier comes back after 15 .. 30 silent blocks (tones.s38 = 1 path: sc 2 -> 3), then gaps in the retrain states of the peer / the node itself */
  for (p = 0; p < 3; p++)
    for (k = 0; k < 8; k++)
      for (i = 0; i < 3; i++) {
        trial_init(&T);
        T.proto = p; T.dbg = ((k + i) & 1) ? 2 : 0;
        T.gapNode = i & 1; T.gapSkip = 1; T.gapAfter = 10; T.gapLen = 16 + 4 * i; T.gapKind = 0;
        T.gap2Node = (i + k) & 1; T.gap2Ts = 6; T.gap2Sc = k; T.gap2After = i; T.gap2Kind = kinds[(k + 2 * i) % 6]; T.gap2Len = lns[(k + 2 * i) % 6];
        T.nblocks = 800; T.stopData = 150;
        run_pair(&T, 200 + n++);
      }
  /* timers at / next to every threshold of the retrain states (poked when the node enters the sub-state): both roles, both origins (requested / local) */
  {
    static const struct { int sc; unsigned thr; } tm[] = {{0, 0x514}, {2, 0x63}, {3, 0x514}, {4, 0x1c1}, {5, 0x257}, {6, 0x31f}, {7, 0x7cf}};
    for (p = 0; p < 3; p++)
      for (k = 0; k < 7; k++)
        for (d = -3; d <= 4; d++) {
          trial_init(&T);
          T.proto = p; T.dbg = (d & 1) ? 2 : 0;
          T.ctlNode = 0; T.ctlBlock = 12; T.ctlC = 0x87; T.ctlD = 0x04;
          T.tmrNode = (d >> 1) & 1; T.tmrTs = 6; T.tmrSc = tm[k].sc; T.tmrVal = (int)(tm[k].thr - 20u + (unsigned)d);
          if (d >= 2) { T.gapNode = T.ctlNode; T.gapSkip = 1; T.gapAfter = 10; T.gapLen = 18; T.gapKind = 0; T.ctlD = 0; } /* local retrain instead of a request */
          T.nblocks = 600; T.stopData = 100;
          run_pair(&T, 400 + n++);
        }
  }
  /* retrain requests while the peer is in a loopback state / not connected: mixed states */
  for (p = 0; p < 3; p++)
    for (i = 0; i < 4; i++) {
      trial_init(&T);
      T.proto = p; T.dbg = 2;
      T.ctlNode = 0; T.ctlBlock = 12; T.ctlC = 0x87; T.ctlD = 0x04;
      T.pokeNode = 1; T.pokeBlock = 12 + i; T.pokeTs = 4 + (i & 1); T.pokeSc = i >> 1;
      T.nblocks = 500; T.stopData = 100;
      run_pair(&T, 800 + n++);
    }
}

/* ================================================================================================ fuzz */
static const unsigned fz_thr[] = {0xce4, 0x514, 0x63, 0x1c1, 0x257, 0x31f, 0x7cf, 0x1388, 0x258, 0x316, 0x2f3, 0x1a0, 0x218, 0x5be, 0x61, 0x786};
static void poke_node(Node *N, int how)
{
  static const int scs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, -1, 100, 0x7fff, -32768, 0x10003};
  V22Tones *t = N->e->tones;
  V22Core *c = N->e->core;
  unsigned r = rnd();
  switch (how) {
    case 0: t->sc = (short)scs[r % (sizeof scs / sizeof scs[0])]; break;
    case 1: t->state = (short)(rpct(60) ? (int)(r % 7u) : (int)(rpct(50) ? 0 : (int)(r % 7u))); t->sc = (short)(rpct(70) ? (int)(rnd() % 8u) : scs[rnd() % (sizeof scs / sizeof scs[0])]); break;
    case 2: t->timer = (int)(fz_thr[r % (sizeof fz_thr / sizeof fz_thr[0])] - 20u + (rnd() % 6u) - 2u); break;
    case 3: t->timer = rpct(80) ? (int)(r % 4000u) : (int)(rpct(50) ? 0x7fffffe0u + (r & 63u) : 0xffffffe0u + (r & 31u)); break;
    case 4: { static const unsigned a[] = {0x26, 0x27, 0x28, 0x39, 0x3a, 0x3b, 0x3c, 0x60, 0x61, 0x62, 0x99, 0x9a, 0x9b, 0xe5, 0xe6, 0xe7, 0xe8, 0xbb7, 0xbb8, 0xbb9, 0xba4, 0xba5, 0xffff, 0x8000, 0, 0x14, 0x28};
              t->sigAcc = (short)a[r % (sizeof a / sizeof a[0])]; t->onesAcc = (short)a[rnd() % (sizeof a / sizeof a[0])]; break; }
    case 5: t->trained = (int)(rnd() % 3u); break;
    case 6: t->lossCnt = (int)(rpct(70) ? 12 + (int)(rnd() % 6u) : (int)(rnd() % 60u)); break;
    case 7: N->e->cfg.lossMs = (int)(rpct(60) ? (int)(rnd() % 1500u) : (int)(rnd() | 0xffff0000u) + (int)(rnd() & 0xffffu)); break;
    case 8: N->e->cfg.handshakeTimeoutMs = (int)(rpct(70) ? (int)(rnd() % 800u) : (int)(rnd())); break;
    case 9: { static const int ev[] = {0, 0xc7, 0xc8, 0xc9, 0xca, 0x100, 0x7fff, -1, -0x4000}; c->fse.errPow = (short)ev[r % (sizeof ev / sizeof ev[0])]; break; }
    case 10: N->e->cfg.rate2 = (short)(rpct(40) ? 0x960 : (rpct(60) ? 0x4b0 : (int)(rnd() & 0xfff))); break;
    case 11: c->rxBis = (short)(rpct(70) ? (int)(rnd() % 2u) : (int)(rnd() % 4u)); c->txBis = (short)(rnd() % 2u); break;
    case 12: N->e->cfg.b11 = (unsigned char)(rnd()); break;
    case 13: c->w20 = (int)(rnd() % 2u); c->scrOn = (int)(rnd() % 2u); c->descrOn = (int)(rnd() % 2u); break;
    case 14: c->agcIn.detect = (int)(rnd() % 3u); c->agcIn.active = (int)(rnd() % 2u); break;
    case 15: c->sre.wantTrack = (int)(rnd() % 2u); break;
    case 16: t->retrainLocal = (short)(rpct(70) ? (int)(rnd() % 2u) : (int)rnd()); break;
    case 17: N->e->flags1d = (unsigned char)rnd(); N->e->flags1e = (unsigned char)rnd(); N->e->state = (unsigned char)rnd(); break;
    case 18: t->timeout = rpct(50) ? (int)(rnd() % 3000u) : (int)(0xffff0000u | (rnd() & 0xffffu)); break;
    case 19: ctl_send(N->e, rnd() & 0xff, (rpct(30) ? 0x04u : (rpct(30) ? 0x80u : (rnd() & 0xffu)))); break;
    default: t->sigAcc = (short)rnd(); t->onesAcc = (short)rnd(); break;
  }
}
/* the focus selects the pokes: 0 data, 1 loops (states 3 .. 5), 2 retrain (state 6), 3 everything */
static void sc_fuzz(int focus, int ntrials, int nblk)
{
  int tr, b, i;
  char t[48];
  static short ext[160];
  for (tr = 0; tr < ntrials; tr++) {
    Node *N[2];
    int proto = tr % 3, k, mode0 = (focus == 1 && (tr & 1)) ? 2 : 0;
    short in[2][160];
    dsplibs_debug_level = (unsigned)(tr % 4 == 0 ? 0 : 2);
    N[0] = node_new(0, mode0, proto, (tr / 3) & 1, (focus == 0 && (tr % 5) == 4));
    N[1] = node_new(1, mode0 ? 2 : 1, proto, (tr / 3) & 1, 0);
    snprintf(t, sizeof t, "Z%d.%d", focus, tr);
    for (b = 0; b < nblk; b++) {
      for (i = 0; i < 2; i++) {
        Node *X = N[i];
        int src = (int)rmod(10);
        if (rpct(14)) poke_node(rpct(50) ? X : N[1 - i], (int)rmod(21));
        if (b > 8 && rpct(focus == 3 ? 3 : 2)) {
          V22Tones *tn = X->e->tones;
          switch (focus) {
            case 0: tn->state = 0; tn->sc = 0; break;
            case 1: tn->state = (short)(3 + (int)rmod(3)); tn->sc = (short)(tn->state == 3 ? (int)rmod(14) : (int)rmod(4)); break;
            case 2: tn->state = 6; tn->sc = (short)rmod(8); break;
            default: tn->state = (short)rmod(7); tn->sc = (short)rmod(14); break;
          }
          if (rpct(60)) tn->timer = 0;
        }
        if (rpct(10)) X->e->tones->timer = (int)(fz_thr[rmod(sizeof fz_thr / sizeof fz_thr[0])] - 20u + (unsigned)rmod(6) - 2u);
        if (rpct(6)) poke_node(X, 4);
        if (rpct(5)) poke_node(X, 6);
        if (rpct(4)) poke_node(X, 9);
        memcpy(in[i], N[1 - i]->peer, sizeof in[i]);
        switch (src) {
          case 0: for (k = 0; k < 160; k++) in[i][k] = 0; break;
          case 1: for (k = 0; k < 160; k++) in[i][k] = (short)((int)(rnd() % 4001) - 2000); break;
          case 2: for (k = 0; k < 160; k++) in[i][k] = (short)rnd(); break;
          case 3: memcpy(in[i], X->out, sizeof in[i]); break;
          case 4: for (k = 0; k < 160; k++) in[i][k] = (short)(in[i][k] / 8 + (int)(rnd() % 7) - 3); break;
          case 5: for (k = 0; k < 160; k++) { int v = in[i][k] * 8; in[i][k] = (short)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); } break;
          case 6: memcpy(in[i], ext, sizeof in[i]); break;
          default: break;
        }
        if (src == 1 || src == 2) memcpy(ext, in[i], sizeof ext);
        {
          int txc = rpct(70) ? 12 : (int)(2 * rmod(9));
          unsigned ret;
          static const int rxns[] = {0, 1, 2, 3, 5, 12, 13, 40, 80, 159};
          g_rxin = rpct(85) ? 160 : rxns[rmod(sizeof rxns / sizeof rxns[0])];
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

/* ================================================================================================ direct calls of the five handlers */
static void sc_direct(int ntrials, int nblk)
{
  int tr, b, i;
  char t[48];
  short dtx[128], dout[256], din[160], drx[512];
  for (tr = 0; tr < ntrials; tr++) {
    Node *N[2];
    int proto = tr % 3, k;
    dsplibs_debug_level = (unsigned)(tr % 3 == 0 ? 0 : 2);
    N[0] = node_new(0, 0, proto, 0, 0);
    N[1] = node_new(1, 1, proto, (tr / 3) & 1, 0);
    snprintf(t, sizeof t, "D%d", tr);
    for (b = 0; b < nblk; b++) {
      for (i = 0; i < 2; i++) {
        Node *X = N[i];
        V22Engine *e = X->e;
        short in[160];
        memcpy(in, N[1 - i]->peer, sizeof in);
        if (b > 3 && rpct(40)) {
          static const int scs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1, 100, 0x7fff, -32768};
          int which = (int)rmod(5), nsc = scs[rmod(sizeof scs / sizeof scs[0])];
          short txc, rxc;
          int m2e_saved = 0;
          if (rpct(60)) nsc = (int)rmod(which == 3 ? 4 : (which == 2 ? 3 : (which == 1 ? 14 : 8)));
          e->tones->sc = (short)nsc;
          if (rpct(35)) poke_node(X, (int)(2 + rmod(18)));
          if (rpct(30)) poke_node(X, 4);
          for (k = 0; k < 160; k++) din[k] = rpct(20) ? (short)rnd() : in[k];
          for (k = 0; k < 256; k++) dout[k] = 0;
          for (k = 0; k < 128; k++) dtx[k] = (short)(rnd() & (e->core->txBis ? 0xf : 3));
          for (k = 0; k < 512; k++) drx[k] = 0x5a5a;
          txc = (short)(rpct(70) ? 12 : (int)(2 * rmod(9)));
          rxc = (short)(rpct(80) ? 160 : (int)(rmod(41)));
          if (rxc != 160 && e->core->rxChannel == 2) { m2e_saved = 2; e->core->rxChannel = 1; }
          pps_guard(e);
          switch (which) {
            case 0: v22_data(e, dtx, dout, din, drx, &txc, &rxc); break;
            case 1: v22_local_loop(e, dtx, dout, din, drx, &txc, &rxc); break;
            case 2: v22_org_rmloop2(e, dtx, dout, din, drx, &txc, &rxc); break;
            case 3: v22_ans_rmloop2(e, dtx, dout, din, drx, &txc, &rxc); break;
            default: v22_retrain(e, dtx, dout, din, drx, &txc, &rxc); break;
          }
          if (m2e_saved) e->core->rxChannel = (short)m2e_saved;
          emit(0, "%s b=%d n%d direct%d sc=%d state=%02x f1d=%02x f1e=%02x tx=%d rx=%d out=%016llx bits=%016llx", t, b, i, which, nsc, e->state, e->flags1d, e->flags1e, txc, rxc,
               fnv_shorts(dout, 160), fnv_shorts(drx, 100));
          memcpy(X->out, dout, 160 * sizeof(short));
          d_engine(0, t, e);
          if (e->tones->state > 2 || rpct(50)) e->tones->state = (short)(i ? 2 : 1);
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

/* ================================================================================================ threshold sweeps */
/* One fresh engine per case, one or two blocks through V22FP_modem with poked sub-state, timer, accumulators, counters, error power, rates: every comparison of the five handlers is probed at, next to
 * and across its constant (the handshake timers move in 20 ms steps and the counters in steps of 1, so a real run never lands next to a threshold). */
typedef struct Sw {
  int mode, ts, sc, rate2, kind, detect, trained, lossCnt, lossMs, w8, errPow, b11, rxBis, s38, w20, wantTrack;
  unsigned timer;
  int timeout, sigAcc, onesAcc;
} Sw;
static void sweep_case(const Sw *W, int tag)
{
  Node *N;
  short in[160];
  int k;
  char t[48];
  unsigned ret;
  N = node_new(0, W->mode, W->rate2 == 0x4b0 ? 1 : 0, 0, 0);
  snprintf(t, sizeof t, "W%d", tag);
  N->e->tones->state = (short)W->ts;
  N->e->tones->sc = (short)W->sc;
  N->e->tones->timer = (int)W->timer;
  N->e->tones->timeout = W->timeout;
  N->e->tones->sigAcc = (short)W->sigAcc;
  N->e->tones->onesAcc = (short)W->onesAcc;
  N->e->tones->trained = W->trained;
  N->e->tones->lossCnt = W->lossCnt;
  N->e->tones->retrainLocal = (short)W->s38;
  N->e->cfg.rate2 = (short)W->rate2;
  N->e->cfg.lossMs = W->lossMs;
  N->e->cfg.handshakeTimeoutMs = W->w8;
  N->e->cfg.b11 = (unsigned char)W->b11;
  N->e->core->rxBis = (short)W->rxBis;
  N->e->core->w20 = W->w20;
  N->e->core->agcIn.detect = W->detect;
  N->e->core->sre.wantTrack = W->wantTrack;
  N->e->core->fse.errPow = (short)W->errPow;
  for (k = 0; k < 160; k++) {
    switch (W->kind) {
      case 0: in[k] = 0; break;
      case 1: in[k] = (short)((int)(rnd() % 401) - 200); break;
      case 3: in[k] = (short)((int)(rnd() % 40001) - 20000); break;
      default: in[k] = (short)(((k * 97) & 0x7ff) - 1000); break;
    }
  }
  ret = node_step(N, in, 12);
  emit(0, "%s ts=%d sc=%d t=%u ret=%08x out=%016llx bits=%016llx", t, W->ts, W->sc, W->timer, ret, fnv_shorts(N->out, 160), fnv_ints(N->rxbits, 100));
  d_engine(0, t, N->e);
  if (rpct(50)) {
    ret = node_step(N, in, 12);
    emit(0, "%s 2nd ret=%08x out=%016llx", t, ret, fnv_shorts(N->out, 160));
    d_engine(0, t, N->e);
  }
  V22FP_delete(N->e);
  N->e = NULL;
  trial_end("sweep");
}
static void sw_default(Sw *W)
{
  memset(W, 0, sizeof *W);
  W->rate2 = 0x960; W->timeout = 60000; W->lossMs = 700; W->w8 = 60000; W->b11 = 0x02; W->rxBis = 1; W->wantTrack = 1;
}
static void sc_sweep(void)
{
  /* ts, sc, thr (0 = the timeout / none), mode */
  static const struct { int ts, sc; unsigned thr; } tm[] = {
    {3, 1, 0xce4}, {3, 3, 0}, {3, 8, 0x1c1}, {3, 9, 0x258}, {3, 10, 0x316}, {3, 11, 0}, {3, 12, 0x2f3}, {3, 13, 0},
    {4, 1, 0x514}, {4, 2, 0x514}, {5, 1, 0x514}, {5, 2, 0x1388},
    {6, 0, 0x514}, {6, 2, 0x63}, {6, 3, 0x514}, {6, 4, 0x1c1}, {6, 5, 0x257}, {6, 6, 0x31f}, {6, 7, 0x7cf}};
  int i, d, v, n = 0;
  Sw W;
  dsplibs_debug_level = 2;
  for (i = 0; i < (int)(sizeof tm / sizeof tm[0]); i++)
    for (d = -3; d <= 4; d++)
      for (v = 0; v < 3; v++) {
        unsigned T = tm[i].thr;
        sw_default(&W);
        W.mode = tm[i].ts == 3 ? 2 : (tm[i].ts == 5 ? 1 : 0);
        W.ts = tm[i].ts; W.sc = tm[i].sc; W.kind = v; W.rate2 = (rnd() & 1) ? 0x960 : 0x4b0; W.trained = (int)(rnd() % 2u); W.detect = (int)(rnd() % 2u);
        if (T) W.timer = T - 20u + (unsigned)d;
        else { W.timeout = 1000 + (int)(rnd() % 1000u); W.timer = (unsigned)(W.timeout - 20 + d); }
        sweep_case(&W, n++);
      }
  /* the timer / timeout sign bit (unsigned compares) */
  for (i = 0; i < (int)(sizeof tm / sizeof tm[0]); i++) {
    static const unsigned bt[] = {0x7fffffecu, 0x7fffffe0u, 0xffffffecu, 0xfffffff0u, 0x80000000u};
    for (v = 0; v < 5; v++) {
      sw_default(&W);
      W.mode = tm[i].ts == 3 ? 2 : (tm[i].ts == 5 ? 1 : 0);
      W.ts = tm[i].ts; W.sc = tm[i].sc; W.timer = bt[v]; W.timeout = (int)(v & 1 ? 0x7ffffffe : 0x80000010u); W.kind = v & 1;
      sweep_case(&W, n++);
    }
  }
  /* the data state: carrier counters, loss time, error power, signal accumulator, request detection */
  {
    static const int lc[] = {0, 1, 12, 13, 14, 15, 16, 34, 35, 36}, lm[] = {0, 19, 20, 21, 300, 700, 0x7fff, 0xffff, -1}, w8s[] = {0, 19, 20, 21, 400, 60000},
                     ep[] = {0, 0xc7, 0xc8, 0xc9, 0xca, 0x1000, -1}, sg[] = {0, 0x14, 0xba4, 0xba5, 0xbb7, 0xbb8, 0xbb9, 0xffff, 0x8000};
    int a, b2, c;
    for (a = 0; a < 10; a++)
      for (b2 = 0; b2 < 9; b2++)
        for (c = 0; c < 4; c++) {
          sw_default(&W);
          W.ts = 0; W.sc = 0; W.lossCnt = lc[a]; W.lossMs = lm[b2]; W.kind = c & 1; W.wantTrack = (c >> 1) & 1; W.rxBis = (a + b2) & 1; W.rate2 = W.rxBis ? 0x960 : 0x4b0;
          W.detect = 1; W.b11 = (c == 3 && a < 3) ? 0x0a : 0x02;
          sweep_case(&W, n++);
        }
    for (a = 0; a < 6; a++)
      for (b2 = 0; b2 < 10; b2++) {
        sw_default(&W);
        W.ts = 0; W.sc = 0; W.b11 = 0x0a; W.w8 = w8s[a]; W.lossCnt = lc[b2];
        sweep_case(&W, n++);
      }
    for (a = 0; a < 7; a++)
      for (b2 = 0; b2 < 9; b2++)
        for (c = 0; c < 2; c++) {
          sw_default(&W);
          W.ts = 0; W.sc = 0; W.errPow = ep[a]; W.sigAcc = sg[b2]; W.kind = c; W.wantTrack = 1; W.b11 = (b2 & 1) ? 0x02 : 0x00; W.lossCnt = c ? 0 : 20; W.detect = 1;
          sweep_case(&W, n++);
        }
  }
  /* the accumulators of the loop states against their thresholds (with silence, noise, loud noise; trained 0 / 1) */
  {
    static const int sig[] = {0x26, 0x27, 0x28, 0x3a, 0x3b, 0x3c, 0x60, 0x61, 0x62, 0x99, 0x9a, 0x9b, 0xe6, 0xe7, 0xe8, 0xffff, 0x8000};
    static const int ones[] = {0x3b, 0xe5, 0xe6, 0xe7, 0xe8, 0xffff};
    static const int cases[][2] = {{3, 3}, {4, 1}, {4, 2}, {5, 1}, {5, 2}, {5, 3}, {6, 0}, {6, 3}};
    int a, b2, c, k;
    for (k = 0; k < 8; k++)
      for (a = 0; a < 17; a++)
        for (b2 = 0; b2 < 6; b2++)
          for (c = 0; c < 3; c++) {
            sw_default(&W);
            W.ts = cases[k][0]; W.sc = cases[k][1]; W.mode = W.ts == 3 ? 2 : (W.ts == 5 ? 1 : 0);
            W.sigAcc = sig[a]; W.onesAcc = ones[b2]; W.kind = c; W.trained = (a + b2) & 1; W.detect = (c != 0); W.rate2 = ((a + b2 + k) & 1) ? 0x960 : 0x4b0; W.s38 = (a >> 2) & 1;
            sweep_case(&W, n++);
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
  g_dense = getenv("R08_DENSE") != NULL;
  cov_setup();
  switch (sc) {
    case 0: sc_data(); break;
    case 1: sc_dataperturb(); break;
    case 2: sc_rdl(); break;
    case 3: sc_local(); break;
    case 4: sc_retrain(); break;
    case 5: sc_fuzz(0, 24, 300); break;
    case 6: sc_fuzz(1, 24, 300); break;
    case 7: sc_fuzz(2, 24, 300); sc_fuzz(3, 12, 300); break;
    case 8: sc_sweep(); break;
    case 9: sc_direct(24, 220); break;
    default: die3("no such scenario %d", sc);
  }
  if (getenv("R08_LISTCOV")) {
    int i;
    for (i = 0; i < COV_MAX; i++) if (g_covname[i] && g_cov[i]) fprintf(stderr, "REQ %d\n", i);
  }
  cov_require(sc);
  cov_finish();
  return 0;
}
