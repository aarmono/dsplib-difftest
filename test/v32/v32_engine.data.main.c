/*
 * R03 differential harness, id r03_data: the V.32 protocol-dispatch targets v32_data / v32_handshake / v32_null_protocol (blob LOCAL functions, in src/v32/v32_engine.c) through the
 * blob's GLOBAL V32FP_modem (the dispatch is on hdx+0x76 through V32_PROTOCOL).  Real engine on both sides (dsplibs.o vs src/v32/v32_engine.c and its engine files); one source; the printed
 * lines are layout independent (state fields of the engine, hashes of the audio / bit buffers, never an address), so the 32-bit BLOB run is the golden of the x86-64 build.
 *
 * The engine is created with V32FP_create (rate class 0) and driven by V32FP_modem with LCG input; the harness pokes the fields the dispatch targets read (the same precedent as
 * test/v32/v32_engine.loclop.main.c): the role word (hdx+0x76), the status option bits (bigbuf+0x1c / +0x20 / +0x24, obj+0x11 bit 1 = bit 7: scramble / descramble / clock sync / retrain
 * detect enable), the FSE history object's flag word (engine slot +0x230, +0x62: bit 0 = retrain request, bit 1 = renegotiate request), the object's timer fields (obj+0x2a,
 * obj+0x31 bit 5) and the SNR-drop inputs (obj+0x2c, bigbuf+0x50d6).  The retrain branch of v32_data builds a control block that the blob hands to V32FP_control on the NEXT call
 * through a stale stack slot: the block loop therefore has NO function call between two V32FP_modem calls (inputs are prepared before, outputs are copied and hashed after).
 *
 * Scenarios (argv[1]; own process each; exit 2 = a coverage counter is zero, 3 = harness violation):
 *   0 role 6 with every combination of the four status option bits (scramble, descramble, clock sync, retrain-detect enable) for 5 blocks each
 *   1 retrain request (originate object): flag + bit 7 -> status 9, Control_Flag set, the next block runs V32FP_control (full reconnect), then the handshake of the reconnected object
 *   2 retrain request (answer object), 3 renegotiate request (hist bit 1: control block with hdx role 5 and status 0x1c, RngResp reconnect)
 *   4 both requests in one block, with other option-bit combinations, 5 the elapsed-time counter (obj+0x31 bit 5 clear: counter, status 0xc once obj+0x2a < 5 * count; set: counter cleared)
 *   6 .. 13 roles 0 1 2 3 4 5 7 8 (handshake roles through v32_handshake, null roles through v32_null_protocol)
 *   14 SNR-drop retrain from V32FP_status inside v32_data (obj+0x2c >= 200, decoder error high): nine drops -> full reconnect request
 *   18 .. 21 a REAL connect (V32LocLoopNextState driven through states 7 .. 10 by poke-and-call; V32FP_modem sets role 6 / state 0x22), then the connected pump: 18 plain, 19 all-ones / zero data bits,
 *   20 the option-bit matrix on the connected pump, 21 SNR drops (nine consecutive: full retrain request)
 *   22 the option-bit matrix at 9600 bit/s (the bits per block change), 23 / 24 a real connect and the connected pump at 14400 / 12000 bit/s
 *   15 / 16 / 17 the option-bit matrix of scenario 0 with silent / +-300 / +-3000 input audio
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);
extern unsigned char V32FP_modem(char *obj, void *txBits, void *txOut, void *rxIn, void *rxBits, unsigned int *txCount, unsigned int *rxCount);
extern int Control_Flag;
extern void V32LocLoopNextState(char *obj);

#define NB 120
#define TXL 12
#define RXL 40
typedef unsigned long long u64;

static unsigned g_lcg = 0x2545f491u;
static unsigned rnd(void)
{
  unsigned x;
  g_lcg = g_lcg * 1664525u + 1013904223u;
  x = g_lcg;
  x ^= x >> 15;
  x *= 0x2c1b3c6du;
  x ^= x >> 12;
  x *= 0x297a2d39u;
  x ^= x >> 15;
  return x;
}
static u64 fnv(const void *p, size_t n)
{
  u64 h = 1469598103934665603ULL;
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    h ^= b[i];
    h *= 1099511628211ULL;
  }
  return h;
}

typedef struct {
  unsigned char st;
  unsigned tc, rc;
  short s30, s31, s2a, s2c, s20, s11;
  short h76, h74, hae, h9e;
  int h78, h7c;
  short b28, b2a, b2c, b2e, b50d8, b50d6;
  short f62, f6e, f74;
  int cf;
  unsigned cc;
  short o11b;
} Rec;
static Rec g_rec[NB];
static int g_in_ints[NB][64];
static int g_bw[NB][10];
static unsigned char g_poke31[NB];
static short g_in_audio[NB][RXL];
static short g_txout[NB][400];
static int g_rxout[NB][100];
static short g_txsave[NB][400];
static int g_rxsave[NB][100];
static short g_clsave[NB][160];
static unsigned long g_cov[32];
static const char *g_covname[32] = {
    "role 6: data pump blocks", "flag 1 (scramble) set", "flag 2 (descramble) set", "flag 4 (clock sync) set", "flag 0x80 (retrain detect) set", "retrain detected (status 9)",
    "control block run on the next call (Control_Flag 1 -> 0)", "renegotiate detected (status 0x1c)", "elapsed counter: status 0xc", "elapsed counter cleared (bit 0x20)",
    "handshake role 0/1 blocks", "handshake role 2 blocks", "handshake role 4 blocks", "handshake role 5 blocks", "null role blocks (3 / 7 / 8)",
    "SNR-drop: obj+0x20 counts", "SNR-drop: full retrain request (obj+0x20 back to 0)", "role changed during the run", "tx / rx counts non-zero", "reconnect: handshake after data", "connect: V32FP_modem sets role 6 / state 0x22 (handshake completed)", "connected pump: role 6 blocks after the connect",
};

static short rs16(char *a) { return *(short *)(intptr_t)a; }

static void run(int sc, int role, int bring)
{
  int cfg[6];
  void *p;
  char *obj, *hdx, *big, *hist;
  unsigned txc, rxc;
  int k, k0;
  unsigned char st;
  int prevrole = -1;
  int txl;

  memset(cfg, 0, sizeof cfg);
  cfg[0] = (sc == 2 || sc == 7 || sc == 9) ? 1 : 0; /* (scenario 7 = role 1; 9 = role 3 on an answer object) */
  cfg[1] = 100;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1;
  {
    unsigned rate = sc == 22 ? 0x2580u : (sc == 23 ? 0x3840u : (sc == 24 ? 0x2ee0u : 0x12c0u));
    *(unsigned int *)&cfg[5] = rate | (rate << 16);
  }
  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    exit(3);
  }
  obj = (char *)(p);
  hdx = V32W_PG(char *, obj, 0x64);
  big = V32W_PG(char *, obj, 0x68);
  hist = V32E_PG(char *, big, 0x230);
  for (k = 0; k < NB; k++) {
    int i;
    for (i = 0; i < 64; i++) g_in_ints[k][i] = getenv("R03_TXZERO") ? 0 : (int)(rnd() & 0x3f);
    g_poke31[k] = (unsigned char)rnd();
    for (i = 0; i < RXL; i++) g_in_audio[k][i] = (sc == 15 || (sc >= 18 && sc <= 21) || sc >= 23) ? 0 : (sc == 16 ? (short)(rnd() % 601u) - 300 : ((sc <= 5 || sc == 14 || sc == 22) ? (short)(rnd() % 25u) - 12 : (short)(rnd() % 6000u) - 3000));
  }
  (void)bring;
  txl = *(short *)(intptr_t)(hdx + 0x9e); /* transmit bits per block of this rate class (12 at 4800) */
  if (txl <= 0 || txl > 64) txl = 12;
  *(short *)(intptr_t)(hdx + 0x76) = (short)role;
  if (sc == 18 || sc == 19 || sc == 20 || sc == 21 || sc == 23 || sc == 24) {
    /* a real connect: drive the loop-back machine (role 2: the shortest path to the connect state; the poke-and-call technique of test/v32/v32_engine.loclop.main.c) through its last
     * states: 7 -> 8 -> 9 -> 10 installs TxHdxScrSequence / RxHdxData; a one-sample transmit budget makes the next block's Tx handler call the NextState of state 10, which sets the
     * connect bits (obj+0x31 |= 0x19, status V32_CONNECT[rate]) that V32FP_modem turns into role 6 / state 0x22 */
    int st0;
    *(short *)(intptr_t)(hdx + 0x76) = 2;
    for (st0 = 7; st0 <= 9; st0++) {
      *(short *)(intptr_t)(hdx + 0x74) = (short)st0;
      V32LocLoopNextState(obj);
    }
    *(int *)(intptr_t)(hdx + 0x78) = 1;
  }
  for (k = 0; k < NB; k++) {
    /* ---- per-block pokes (inline: no function call between two V32FP_modem calls) ---- */
    if (sc == 0 || (sc >= 15 && sc <= 17) || sc == 22) {
      int combo = (k / 5) & 15; /* 16 combos x 5 blocks = blocks 0 .. 79 */
      *(unsigned char *)(intptr_t)(big + 0x1c) = (unsigned char)(combo & 1);
      *(unsigned char *)(intptr_t)(big + 0x20) = (unsigned char)((combo >> 1) & 1);
      *(unsigned char *)(intptr_t)(big + 0x24) = (unsigned char)((combo >> 2) & 1);
      *(unsigned char *)(intptr_t)(obj + 0x11) = (unsigned char)((*(unsigned char *)(intptr_t)(obj + 0x11) & ~2) | (((combo >> 3) & 1) << 1));
      *(short *)(intptr_t)(obj + 0x2c) = 0; /* stay in the SNR warm-up */
    } else if (sc >= 1 && sc <= 4) {
      int fl = sc == 4 ? (k == 4 ? 2 : (k % 8)) : (sc == 3 ? 5 : 7); /* (scenario 4, block 4: the descramble option bit set while retrain detect is disabled and a retrain request is pending) */
      *(unsigned char *)(intptr_t)(big + 0x1c) = (unsigned char)(fl & 1);
      *(unsigned char *)(intptr_t)(big + 0x20) = (unsigned char)((fl >> 1) & 1);
      *(unsigned char *)(intptr_t)(big + 0x24) = (unsigned char)((fl >> 2) & 1);
      if (sc == 1 || sc == 2) *(unsigned char *)(intptr_t)(obj + 0x11) |= 2;
      else if (sc == 3) *(unsigned char *)(intptr_t)(obj + 0x11) &= (unsigned char)~2;
      *(short *)(intptr_t)(obj + 0x2c) = 0;
      if (sc == 1 || sc == 2 || sc == 3) {
        if (k == 4 || k == 20 || k == 40) {
          unsigned short *f = (unsigned short *)(intptr_t)(hist + 0x62);
          *f |= (unsigned short)(sc == 3 ? 2 : 1);
        }
        if (k == 20 || k == 40) *(short *)(intptr_t)(hdx + 0x76) = (short)role; /* back to the data pump after the reconnect */
      } else {
        /* scenario 4: the two requests together.  A retrain request that is detected in the SAME call as a renegotiate request (or whose follow-up block is not a
         * v32_data call) leaves Control_Flag set until a later data block, where the blob hands V32FP_control a STALE STACK block (see NOTES R03): not reproducible,
         * so the requests are interleaved such that a retrain detection is always followed by a data block */
        unsigned short *f = (unsigned short *)(intptr_t)(hist + 0x62);
        if (k == 4) { *f |= 3; *(unsigned char *)(intptr_t)(obj + 0x11) &= (unsigned char)~2; }                         /* retrain detect disabled: only the renegotiate request acts */
        if (k == 8) { *(short *)(intptr_t)(hdx + 0x76) = 6; *(unsigned char *)(intptr_t)(obj + 0x11) |= 2; }          /* enabled: the pending retrain request fires */
        if (k == 14) { *(short *)(intptr_t)(hdx + 0x76) = 6; *f |= 2; *(unsigned char *)(intptr_t)(obj + 0x11) |= 2; }
        if (k == 19) { *(short *)(intptr_t)(hdx + 0x76) = 6; *f |= 1; }
        if (k == 29) { *(short *)(intptr_t)(hdx + 0x76) = 6; *f |= 3; *(unsigned char *)(intptr_t)(obj + 0x11) &= (unsigned char)~2; }
        if (k == 33) { *(short *)(intptr_t)(hdx + 0x76) = 6; *(unsigned char *)(intptr_t)(obj + 0x11) |= 2; }
      }
    } else if (sc == 5) {
      *(short *)(intptr_t)(obj + 0x2c) = 0;
      if (k == 0) *(short *)(intptr_t)(obj + 0x2a) = 10; /* elapsed * 5 == 10 at the second count: the boundary of the compare */
      if (k == 26) *(short *)(intptr_t)(obj + 0x2a) = -1; /* negative: a signed compare says 'expired' at once */
      /* blocks 10 .. 12: an (apparent) rx carrier through engine word +0x108 makes DemodDataV32 skip its carrier block, which would otherwise rewrite bit 0x20 of obj+0x31 */
      if (k >= 10 && k <= 12) {
        *(int *)(intptr_t)(big + 0x108) = 1;
        *(unsigned char *)(intptr_t)(obj + 0x31) |= 0x20;
      }
      if (k == 13) *(int *)(intptr_t)(big + 0x108) = 0;
      if (k == 20) *(short *)(intptr_t)(obj + 0x2a) = 3;
    } else if (sc >= 18 && sc != 22) {
      *(short *)(intptr_t)(obj + 0x2c) = sc == 21 && k >= 30 ? 200 : 0;
      if (sc == 21 && k >= 30) *(short *)(intptr_t)(big + 0x50d6) = 0x7000; /* SNR drops while connected */
      /* (a connected object that hears its own audio, or any signal, raises renegotiate requests at once and then misbehaves in the blob itself: the connected scenarios listen to silence) */
      if (sc == 19) for (k0 = 0; k0 < 64; k0++) g_in_ints[k][k0] = k < 60 ? 0x3f : 0; /* all-ones then all-zero data bits */
      if (sc == 20) { /* the option-bit matrix on the connected pump (a renegotiate request in this loop-back-started object makes the blob itself crash: TxHdxScrSequence with a negative count) */
        int combo = (k / 5) & 15;
        *(unsigned char *)(intptr_t)(big + 0x1c) = (unsigned char)(combo & 1);
        *(unsigned char *)(intptr_t)(big + 0x20) = (unsigned char)((combo >> 1) & 1);
        *(unsigned char *)(intptr_t)(big + 0x24) = (unsigned char)((combo >> 2) & 1);
        *(unsigned char *)(intptr_t)(obj + 0x11) = (unsigned char)((*(unsigned char *)(intptr_t)(obj + 0x11) & ~2) | (((combo >> 3) & 1) << 1));
      }
    } else if (sc == 14) {
      if (k == 0) {
        *(short *)(intptr_t)(obj + 0x2c) = 200;
        *(unsigned char *)(intptr_t)(big + 0x1c) = 1;
        *(unsigned char *)(intptr_t)(big + 0x20) = 1;
      }
      *(short *)(intptr_t)(big + 0x50d6) = 0x7000; /* decoder error: SNR far below the retrain threshold */
    }
    if (sc >= 6 && sc <= 13 && (k & 1)) *(unsigned char *)(intptr_t)(obj + 0x31) |= (unsigned char)(0x0c | (g_poke31[k] & 2) | ((sc == 6 || sc == 7 || sc == 8 || sc == 10 || sc == 11) ? (g_poke31[k] & 1) : 0)); /* the dispatch targets' own bit changes of obj+0x31 become visible */
    txc = (unsigned)txl;
    rxc = RXL;
    st = V32FP_modem(obj, g_in_ints[k], g_txout[k], g_in_audio[k], g_rxout[k], &txc, &rxc);
    /* ---- record (inline) ---- */
    g_rec[k].st = st;
    g_rec[k].tc = txc;
    g_rec[k].rc = rxc;
    g_rec[k].s30 = (signed char)*(unsigned char *)(intptr_t)(obj + 0x30);
    g_rec[k].s31 = *(unsigned char *)(intptr_t)(obj + 0x31);
    g_rec[k].s2a = rs16(obj + 0x2a);
    g_rec[k].s2c = rs16(obj + 0x2c);
    g_rec[k].s20 = rs16(obj + 0x20);
    g_rec[k].s11 = *(unsigned char *)(intptr_t)(obj + 0x11);
    hdx = V32W_PG(char *, obj, 0x64);
    big = V32W_PG(char *, obj, 0x68);
    hist = V32E_PG(char *, big, 0x230);
    g_rec[k].h76 = rs16(hdx + 0x76);
    g_rec[k].h74 = rs16(hdx + 0x74);
    g_rec[k].hae = rs16(hdx + 0xae);
    g_rec[k].h9e = rs16(hdx + 0x9e);
    g_rec[k].h78 = *(int *)(intptr_t)(hdx + 0x78);
    g_rec[k].h7c = *(int *)(intptr_t)(hdx + 0x7c);
    g_rec[k].b28 = rs16(big + 0x28);
    g_rec[k].b2a = rs16(big + 0x2a);
    g_rec[k].b2c = rs16(big + 0x2c);
    g_rec[k].b2e = rs16(big + 0x2e);
    g_rec[k].b50d8 = rs16(big + 0x50d8);
    g_rec[k].b50d6 = rs16(big + 0x50d6);
    g_rec[k].f62 = rs16(hist + 0x62);
    g_rec[k].f6e = rs16(hist + 0x6e);
    g_rec[k].f74 = rs16(hist + 0x74);
    g_rec[k].cf = Control_Flag;
    for (k0 = 0; k0 < 10; k0++) g_bw[k][k0] = *(int *)(intptr_t)(big + 4 * k0); /* the engine's option words 0 .. 9 (what V32FP_control writes from its control block) */
    /* the engine's cleaned-sample buffer (engine +0x50d0, count +0x50d4: what V32FP_GetCleanedSamples returns): read inline, no call */
    g_rec[k].cc = *(unsigned short *)(intptr_t)(big + 0x50d4);
    for (k0 = 0; k0 < 160; k0++) g_clsave[k][k0] = (unsigned)k0 < g_rec[k].cc ? V32E_PG(short *, big, 0x50d0)[k0] : 0;
    for (k0 = 0; k0 < 400; k0++) g_txsave[k][k0] = g_txout[k][k0];
    for (k0 = 0; k0 < 100; k0++) g_rxsave[k][k0] = g_rxout[k][k0];
  }
  /* ---- print + coverage ---- */
  for (k = 0; k < NB; k++) {
    Rec *r = &g_rec[k];
    printf("B%d st=%02x tc=%u rc=%u txh=%016llx rxh=%016llx o30=%d o31=%02x o2a=%d o2c=%d o20=%d o11=%02x h76=%d h74=%d hae=%d h9e=%d h78=%d h7c=%d b28=%d b2a=%d b2c=%d b2e=%d b50d8=%d b50d6=%d f62=%d f6e=%d f74=%d cf=%d cc=%u clh=%016llx bw=%016llx\n", k,
           r->st, r->tc, r->rc, fnv(g_txsave[k], 2u * (r->tc < 400 ? r->tc : 400)), fnv(g_rxsave[k], 4u * (r->rc < 100 ? r->rc : 100)), r->s30, r->s31, r->s2a, r->s2c, r->s20, r->s11, r->h76,
           r->h74, r->hae, r->h9e, r->h78, r->h7c, r->b28, r->b2a, r->b2c, r->b2e, r->b50d8, r->b50d6, r->f62, r->f6e, r->f74, r->cf, r->cc, fnv(g_clsave[k], 320), fnv(g_bw[k], 40));
    if (r->h76 == 6 || (k > 0 && g_rec[k - 1].h76 == 6)) g_cov[0]++;
    if (r->tc || r->rc) g_cov[18]++;
    if (r->st == 9) g_cov[5]++;
    if (r->st == 0x1c) g_cov[7]++;
    if (r->st == 0xc) g_cov[8]++;
    if (k > 0 && g_rec[k - 1].cf == 1 && r->cf == 0) g_cov[6]++;
    if (k > 0 && (g_rec[k - 1].h76 != r->h76)) g_cov[17]++;
    if (k > 0 && g_rec[k - 1].h76 == 6 && r->h76 != 6 && r->h76 != 5) g_cov[19]++;
    if (k > 0 && g_rec[k - 1].hae > 0 && r->hae == 0 && sc == 5) g_cov[9]++;
    if ((k == 0 ? sc >= 18 : g_rec[k - 1].h76 != 6) && r->h76 == 6 && r->h74 == 0x22) g_cov[20]++;
    if (sc >= 18 && k > 0 && g_rec[k - 1].h76 == 6 && r->h76 == 6) g_cov[21]++;
    if (r->h76 == 0 || r->h76 == 1) g_cov[10]++;
    if (r->h76 == 2) g_cov[11]++;
    if (r->h76 == 4) g_cov[12]++;
    if (r->h76 == 5) g_cov[13]++;
    if (r->h76 == 3 || r->h76 == 7 || r->h76 == 8) g_cov[14]++;
    if (k > 0 && r->s20 > g_rec[k - 1].s20) g_cov[15]++;
    if (k > 0 && g_rec[k - 1].s20 > 0 && r->s20 == 0) g_cov[16]++;
    (void)prevrole;
  }
  if (sc == 0 || (sc >= 15 && sc <= 17) || sc == 22) {
    int combo;
    for (combo = 0; combo < 16; combo++) {
      if (combo & 1) g_cov[1]++;
      if (combo & 2) g_cov[2]++;
      if (combo & 4) g_cov[3]++;
      if (combo & 8) g_cov[4]++;
    }
  }
  V32FP_delete(obj);
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0, i, bad = 0;
  static const int roles[] = {6, 6, 6, 6, 6, 6, 0, 1, 2, 3, 4, 5, 7, 8, 6, 6, 6, 6, 2, 2, 2, 2, 6, 2, 2};
  /* the counters each scenario must reach (a mask over g_covname) */
  static const unsigned long need[] = {
      (1ul << 0) | (1ul << 1) | (1ul << 2) | (1ul << 3) | (1ul << 4) | (1ul << 18),
      (1ul << 0) | (1ul << 5) | (1ul << 6) | (1ul << 17) | (1ul << 19),
      (1ul << 0) | (1ul << 5) | (1ul << 6) | (1ul << 19),
      (1ul << 0) | (1ul << 7) | (1ul << 13) | (1ul << 17),
      (1ul << 0) | (1ul << 5) | (1ul << 6) | (1ul << 7),
      (1ul << 0) | (1ul << 8) | (1ul << 9),
      (1ul << 10), (1ul << 10), (1ul << 11), (1ul << 14), (1ul << 12), (1ul << 13), (1ul << 14), (1ul << 14),
      (1ul << 0) | (1ul << 15) | (1ul << 16),
      (1ul << 0) | (1ul << 18), (1ul << 0) | (1ul << 18), (1ul << 0) | (1ul << 18),
      (1ul << 20) | (1ul << 21), (1ul << 20) | (1ul << 21), (1ul << 20) | (1ul << 21), (1ul << 20) | (1ul << 21),
      (1ul << 0) | (1ul << 18), (1ul << 20) | (1ul << 21), (1ul << 20) | (1ul << 21),
  };
  if (sc < 0 || sc > 24) {
    fprintf(stderr, "scenario 0..24\n");
    return 3;
  }
  g_lcg += 7919u * (unsigned)sc;
  if (getenv("R03_SEED")) g_lcg ^= (unsigned)atoi(getenv("R03_SEED"));
  run(sc, roles[sc], 0);
  for (i = 0; i < 22; i++) {
    if (!(need[sc] & (1ul << i))) continue;
    printf("COV %-60s %lu\n", g_covname[i], g_cov[i]);
    if (g_cov[i] == 0) {
      fprintf(stderr, "COVERAGE FAIL: %s was never reached\n", g_covname[i]);
      bad = 1;
    }
  }
  fflush(stdout);
  return bad ? 2 : 0;
}
