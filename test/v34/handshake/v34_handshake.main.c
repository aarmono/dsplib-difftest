/*
 * Differential test harness for port chunk V04a (src/v34/handshake/v34_hs_tx_small.cpp): the SMALL transmit-state cases of v34handshak's per-sample loop (SSEG, TONE_AB, XMIT0, DATAXMIT, TXLEVEL, JaTXMIT, K56JaTXMIT and
 * the four MOH_* states).  v34handshak is ONE 61,541 B function; the pieces are ported separately (porting/decompiled/dp/v04_v34handshak.txt), so the unit under test is "the blob's loop run ONCE":
 *   blob  = the REAL v34handshak with a run-time patch: the loop's back edge (`jl 62950` at v34handshak+0xf7) is overwritten by `jmp <epilogue>` so the function returns after the FIRST dispatch of the
 *           transmit-state jump table (the object is set up so that the loop condition (short)txIdx_ < (short)blkLen_ holds and the post-loop code is never reached);
 *   port  = the dispatch below (iso_port: the same signed 16-bit state arithmetic as the blob, `(unsigned)(state - 5) <= 0x51`) calling the ported case helper.
 * ONE C source, no #ifdef around the code under test (V04_BLOB only selects blob_iso / the patch); built by test/build_and_diff_v04.sh against the blob (dsplibs.o with the callees WEAK), the port (-O0 / -O2)
 * and x86-64 (diffed against the 32-bit BLOB's output = test/golden/v04_blob32.out).
 *
 * Every callee is a RECORDER (txmit, modulatevector, txwritequeue, V34EchoReportCoeff, v90Phase34, k56FlexPhase34, dsplibs_debug_printf): the record carries the arguments as tokens (pointers into the
 * object as MEMBER + delta, never as a raw offset: the x86-64 offsets differ) and a SNAPSHOT (hash) of the object; a recorder then applies its scripted POKE (debug level, the state words, the flags, the
 * counters ...: the instant at which the blob reloads a member is observable).  There is NO separate model: the 32-bit blob is the reference (a per-call whole-object dump + record log is compared).
 *
 * Scenarios (own process each): 0 SSEG TONE_AB XMIT0 TXLEVEL, 1 DATAXMIT + MOH_*, 2 JaTXMIT + K56JaTXMIT, 3 every ported state with every poke kind, 4 sequences (walks through SSEG's counter, MOH's 192-symbol
 * wait, JaTXMIT's countdown) and the states the blob ignores (jump table default / not ported yet: only no-op states are driven).
 */

#include <sys/mman.h>

#include "../v01_common.h"
#include "../v01_layout.h"

extern const int vect4[4];

void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

enum { G_A = 0, G_B = 1, G_C = 2, G_POKE = 3, G_SEQ = 4, G_D = 5, G_E = 6, G_F = 7, G_H = 8, G_I = 9, G_J = 10, G_K = 11, G_L = 12, G_M = 13, G_N = 14, G_O = 15, G_P = 16, G_Q = 17, G_R = 18, G_S = 19, G_T = 20, G_U = 21, G_V = 22 };

typedef struct { uint32_t kind[NSC], val[NSC]; } Script;
static Script g_sc;
static unsigned g_pk;
static Log g_il;
static int g_nostate;   /* the head's recorders (V34agc, fskdemodulate) must not move the state words the table dispatch reads next */
static int g_show, g_showall;
static int g_whole;   /* V04_WHOLE=1: the real getbit / ApplyBulkDelay run (blob: its own local helpers, port: V04y): the queue records must not make the real getbit recurse forever */
static unsigned long g_symctr, g_ncalls;
static unsigned g_pokemask;
static V34Obj *g_o;
static int g_sc_run;
static uint8_t *g_can;
/* the scenarios that run the WHOLE function from the post-loop sequence on (the loop skipped by construction): receive case + TXPOST + status tail */
static int postloop_sc(int sc) { return sc == G_E || sc == G_F || sc == G_K || sc == G_M || sc == G_N || sc == G_O || sc == G_P || sc == G_R || sc == G_S || sc == G_T; }

static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) {
    g_sc.kind[i] = h_rpct(60) ? 0u : 1u + h_rmod(8);
    g_sc.val[i] = h_rnd();
  }
}
static uint16_t pick_h(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffu; case 3: return 0xc0; case 4: return 0xbf; default: return (uint16_t)(v >> 3); } }
/* the TX_DPSK pokes (scenario G_I): the members the case re-reads after a print or a callee */
static void poke_i(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->mohF0_ = (v >> 3) % 6u; break;
    case 3: o->mohF8_ = (uint8_t)((v & 8) ? 0 : (v >> 4) | 1u); break;
    case 4: o->mohF9_ = (uint8_t)((v & 8) ? 0 : (v >> 4) | 1u); break;
    case 5: o->mohF4_ = (v >> 3) % 7u; break;
    case 6: if (v & 8) o->k56Idx_ = (uint16_t)(v >> 4); else o->waa7e_ = (uint16_t)(v >> 4); break;
    case 7: { static const uint16_t st[] = { 0x54, 0x51, 0x23, 0x3c, 0x18, 0x4a }; if (v & 8) o->w3596_ = st[(v >> 4) % 6]; else o->rxState_ = st[(v >> 4) % 6]; break; }
    case 8: if (v & 8) o->mohE8_ = (uint8_t)(v >> 4); else o->mohE2_ = (uint16_t)(1 + ((v >> 4) & 3u)); break;
    default: break;
  }
}
/* the pokes of scenario G_M (the call / answer microstates): the microstate and receive-state words the cases keep register copies of, the INFO0 words, the counters */
static void poke_m(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  if (g_nostate && (kind == 3 || kind == 4 || kind == 5)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->rxFlags_ = (uint16_t)(v >> 4); break;
    case 3: { static const uint16_t ms[] = { 0x29, 0x2e, 0x2f, 0x31, 0x32, 0x33, 0x37, 0x3a, 0x3b, 0x3f, 0x50 }; o->mst_ = (v & 8) ? ms[(v >> 4) % 11] : (uint16_t)((v >> 4) % 87u); break; }
    case 4: { static const uint16_t rs[] = { 0x2b, 0x35, 0x48, 0x23, 0x04 }; o->rxState_ = (v & 8) ? rs[(v >> 4) % 5] : (uint16_t)((v >> 4) % 87u); break; }
    case 5: o->w3596_ = (v & 8) ? ((v & 4) ? 0x18 : 0x3c) : (uint16_t)((v >> 4) % 87u); break;
    case 6: if (v & 8) o->w3588_ = (uint16_t)((v >> 4) & 3u); else o->shortPhase2_ = (uint16_t)((v >> 4) & 1u); break;
    case 7: if (v & 8) o->wAae2_ = (uint16_t)(v >> 4); else o->wAae0_ = (uint16_t)(v >> 4); break;
    case 8: if (v & 8) o->waa78_ = (uint16_t)(v >> 4); else o->filtDelay_ = (uint16_t)(v >> 4); break;
    default: break;
  }
}
/* the TRNSEG4 pokes (scenario G_L) */
static void poke_l(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->txFlags_ = (uint16_t)(v >> 4); break;
    case 3: if (v & 8) o->rxState_ = (uint16_t)((v >> 4) % 87u); else o->mst_ = (uint16_t)((v >> 4) % 87u); break;
    case 4: o->waa7e_ = (uint16_t)((v & 8) ? (v >> 4) % 6u : (v >> 4)); break;
    case 5: if (v & 8) o->pcmMode_ = (int32_t)((v >> 4) % 4u); else o->k56Mode_ = (int32_t)((v >> 4) % 4u); break;
    case 6: if (v & 8) o->txCnt_ = (uint16_t)(v >> 4); else o->waa78_ = (uint16_t)(v >> 4); break;
    case 7: o->w3596_ = (uint16_t)((v & 8) ? 0x40 : (v >> 4) % 87u); break;
    case 8: if (v & 8) o->waa96_ = (uint16_t)(0x960 + (v >> 4) % 0x400u); else o->waaa8_ = (uint16_t)(0x640 + (v >> 4) % 0x200u); break;
    default: break;
  }
}
/* the XMITMP pokes (scenario G_J) */
static void poke_j(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->rxFlags_ = (uint16_t)((v >> 4) ^ ((v & 8) ? 0x20u : 0u) ^ ((v & 4) ? 0x90u : 0u)); break;
    case 3: o->rxSig_ = (v & 8) ? 0x89b0 : (uint16_t)((v >> 4) | 1u); break;
    case 4: if (v & 8) o->k56Idx_ = (uint16_t)(v >> 4); else o->w3590_ = (uint16_t)(v >> 4); break;
    case 5: if (v & 8) o->w359e_ = (uint16_t)((v >> 4) % 6u); else o->w3598_ = (uint16_t)(v >> 4); break;
    case 6: o->mpUnk00_[0] = (uint8_t)((o->mpUnk00_[0] & ~1u) | ((v >> 3) & 1u)); break;
    case 7: o->txFlags_ = (uint16_t)(v >> 4); break;
    case 8: o->w3596_ = (v & 8) ? 0x45 : (uint16_t)(0x43 + (v >> 4) % 3u); break;
    default: break;
  }
}
/* the DET_SYNC pokes (scenario G_O): every member the case re-reads after a print or a callee */
static void poke_o(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  if (g_nostate && kind == 3) return;   /* the head's recorders must not move the state words the dispatch reads next */
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: if (v & 8) o->rxFlags_ = (uint16_t)(v >> 4); else o->mohE8_ = (uint8_t)((v & 4) ? 0 : (v >> 4) | 1u); break;
    case 3: if (v & 8) o->mst_ = (uint16_t)((v & 4) ? 0x29 : (v >> 4) % 87u); else o->rxState_ = (uint16_t)((v & 4) ? 0x2b : (v >> 4) % 87u); break;
    case 4: o->w3596_ = (uint16_t)((v & 8) ? ((v & 4) ? 0x18 : 0x3c) : (v >> 4) % 87u); break;
    case 5: if (v & 8) o->w3588_ = (uint16_t)((v >> 4) & 3u); else o->w35a0_ = (uint16_t)((v & 4) ? 0x30 + ((v >> 4) & 3u) : (v >> 4)); break;
    case 6: if (v & 8) o->wAae2_ = (uint16_t)((v & 4) ? ((v & 2) ? 0xffffu : 0) : (v >> 4)); else o->wAae0_ = (uint16_t)(v >> 4); break;
    case 7: if (v & 8) o->w358a_ = (uint16_t)((v >> 4) % 4u); else o->waa78_ = (uint16_t)(v >> 4); break;
    case 8: if (v & 8) o->k56Idx_ = (uint16_t)(v >> 4); else if (v & 4) o->mohF0_ = (v >> 4) % 3u; else o->mohF8_ = (uint8_t)((v >> 4) & 1u); break;
    default: break;
  }
}
/* the DET_INFO pokes (scenario G_P): the members the case re-reads after a print or a callee, and the record pointers */
static void poke_p(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  if (g_nostate && kind == 3) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: if (v & 8) o->rxFlags_ = (uint16_t)(v >> 4); else o->shortPhase2_ = (uint16_t)((v & 4) ? 0 : (v >> 4) | 1u); break;
    case 3: if (v & 8) o->mst_ = (uint16_t)((v & 4) ? 0x29 : (v >> 4) % 87u); else o->rxState_ = (uint16_t)((v & 4) ? 0x2b : (v >> 4) % 87u); break;
    case 4: o->w3596_ = (uint16_t)((v & 8) ? ((v & 4) ? 0x18 : 0x3c) : (v >> 4) % 87u); break;
    case 5: if (v & 8) o->modeChar_ = (uint16_t)((v & 4) ? 0x65 : 0x66); else o->mohF0_ = (v >> 4) % 3u; break;
    case 6: if (v & 8) o->wAae2_ = (uint16_t)(v >> 4); else o->waa78_ = (uint16_t)(v >> 4); break;
    case 7: if (v & 8) o->w358a_ = (uint16_t)((v >> 4) % 3u); else o->bitQ2_ = (v & 4) ? (void *)o->i0dUnk00_ : (v & 2) ? (void *)o->i1cUnk00_ : (void *)o->mpUnk00_; break;
    case 8: if (v & 8) o->bitQ_ = (v & 4) ? (void *)o->i0aUnk00_ : (void *)o->i1aUnk00_; else o->pcmMode_ = (int32_t)((v >> 4) % 3u); break;
    default: break;
  }
}
/* the TRNSEG4A pokes (scenario G_Q): the members the case re-reads after txmit / bitreverse / the prints */
static void poke_q(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: if (v & 8) o->rxFlags_ = (uint16_t)(v >> 4); else o->w359a_ = (uint16_t)((v >> 4) & 1u); break;
    case 3: if (v & 8) o->txCnt_ = (uint16_t)(v >> 4); else o->period_ = (uint16_t)((v >> 4) % 300u); break;
    case 4: if (v & 8) o->w2218_ = (int32_t)((v & 4) ? (v >> 4) % 7u : (v >> 4)); else o->w47e_ = (int16_t)(v >> 4); break;
    case 5: { uint16_t w = (uint16_t)((v & 8) ? (v >> 4) % 64u : (v >> 4)); memcpy(o->unk04b0_ + 4, &w, 2); break; }
    case 6: if (v & 8) o->modeChar_ = (uint16_t)((v & 4) ? 0x65 : 0x66); else o->w3596_ = (uint16_t)((v & 4) ? 0x43 : (v >> 4) % 87u); break;
    case 7: { static const uint16_t bd[] = { 0x960, 0xaf0, 0xbb8, 0xc80, 0xd65 }; if (v & 8) o->waa96_ = bd[(v >> 4) % 5u]; else o->rxSig_ = (uint16_t)((v & 4) ? 0x89b0 : 0x8990); break; }
    case 8: if (v & 8) o->rxState_ = (uint16_t)((v >> 4) % 87u); else o->mst_ = (uint16_t)((v >> 4) % 87u); break;
    default: break;
  }
}
/* the RX_L1 pokes (scenario G_R) */
static void poke_r(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  if (g_nostate && kind == 3) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: if (v & 8) o->rxFlags_ = (uint16_t)(v >> 4); else o->rxGain_ = (int16_t)((v & 4) ? 0x34ff + (int)((v >> 4) & 3u) - 1 : (int)(v >> 4)); break;
    case 3: if (v & 8) o->mst_ = (uint16_t)((v & 4) ? 0x33 + ((v >> 4) & 1u) : (v >> 4) % 87u); else o->rxState_ = (uint16_t)((v & 4) ? 0x2b : (v >> 4) % 87u); break;
    case 4: o->w3596_ = (uint16_t)((v & 8) ? ((v & 4) ? 0x3c : 0x18) : (v >> 4) % 87u); break;
    case 5: if (v & 8) o->wAae2_ = (uint16_t)((v & 4) ? 0 : (v >> 4)); else o->wAae0_ = (uint16_t)((v & 4) ? 0x14 + ((v >> 4) & 3u) : (v >> 4)); break;
    case 6: o->waa78_ = (uint16_t)((v & 8) ? (v >> 4) : 0x40u + ((v >> 4) & 0x3ffu)); break;
    case 7: if (v & 8) o->waa7e_ = (uint16_t)(v >> 4); else o->w358c_ = (uint16_t)(v >> 4); break;
    case 8: o->modeChar_ = (uint16_t)((v & 8) ? ((v & 4) ? 0x65 : 0x66) : (v >> 4)); break;
    default: break;
  }
}
/* the RECEIVE pokes (scenario G_S) */
static void poke_s(unsigned kind, unsigned v) {
  V34Obj *o = g_o;
  if (g_nostate && kind != 1) return;   /* the receiver recorder: only the debug level may move */
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->rxFlags_ = (uint16_t)(o->rxFlags_ ^ ((v & 8) ? (v >> 4) & 0x7f0u : 0x10u << ((v >> 4) % 4u))); break;
    case 3: if (v & 8) o->mst_ = (uint16_t)((v & 4) ? 0x29 + ((v >> 4) % 2u) : (v >> 4) % 87u); else o->rxState_ = (uint16_t)((v & 4) ? 4 : (v >> 4) % 87u); break;
    case 4: o->w3596_ = (uint16_t)((v & 8) ? ((v & 4) ? 0x12 : 0x3c) : (v >> 4) % 87u); break;
    case 5: if (v & 8) o->w35a2_ = (uint16_t)(v >> 4); else o->waa80_ = (uint16_t)((v >> 4) & 1u); break;
    case 6: { uint16_t w = (uint16_t)((v & 8) ? (v >> 4) % 64u : (v >> 4)); memcpy((uint8_t *)&o->w388_, &w, 2); break; }
    case 7: if (v & 8) o->w3598_ = (uint16_t)((v >> 4) & 1u); else o->waa78_ = (uint16_t)(v >> 4); break;
    case 8: if (v & 8) o->modeChar_ = (uint16_t)((v & 4) ? 0x65 : 0x66); else { uint16_t w = (uint16_t)((v & 4) ? 3 : (v >> 4)); memcpy(o->unk356c_ + 4, &w, 2); } break;
    default: break;
  }
}
static void poke(void) {
  unsigned k = g_pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  V34Obj *o = g_o;
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  if (g_sc_run == G_I) { poke_i(kind, v); return; }
  if (g_sc_run == G_J) { poke_j(kind, v); return; }
  if (g_sc_run == G_L) { poke_l(kind, v); return; }
  if (g_sc_run == G_M || g_sc_run == G_T) { poke_m(kind, v); return; }
  if (g_sc_run == G_O) { poke_o(kind, v); return; }
  if (g_sc_run == G_P) { poke_p(kind, v); return; }
  if (g_sc_run == G_Q) { poke_q(kind, v); return; }
  if (g_sc_run == G_R) { poke_r(kind, v); return; }
  if (g_sc_run == G_S) { poke_s(kind, v); return; }
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->rxFlags_ = (uint16_t)((v & 8 ? 8u : 0u) | (v >> 4)); break;
    case 3: o->txFlags_ = (uint16_t)((v & 8 ? 0x10u : 0u) | (v >> 4 & 0xffe0u)); break;
    case 4: if (v & 8) o->txCnt_ = pick_h(v >> 4); else o->k56Idx_ = pick_h(v >> 4); break;
    case 5: o->waa78_ = pick_h(v >> 4); break;
    case 6: if (v & 8) o->mohEC_ = (v >> 4) & 1u ? 1u : v >> 5; else o->mohF0_ = (v >> 4) & 1u ? 1u : v >> 5; break;
    case 7: { static const uint16_t st[] = { 0x51, 0x52, 0x53, 0x13, 0x12, 0x4e, 0x55, 0x41, 0x3c, 0x4a, 0x18, 0x36 }; o->w3596_ = st[(v >> 4) % 12]; break; }
    case 8: if (v & 8) o->w2218_ = (int32_t)(v >> 4 & 3u); else o->waa98_ = (uint16_t)(v >> 4); break;
    default: break;
  }
}

/* ------------------------------------------------------------------------------------------------ the snapshot of the object at the moment of a record */
static const char *memb(const void *p);
extern const short c1200_[8], c2400_[8], hsine1600[], hsine1680[], hsine1800[], hsine1829[], hsine1867[], hsine1920[], hsine1959[], hsine2000[];
extern const short scale2400[], scale2800[], scale3000[], scale3200[], scale3429[];
static const char *tabname(const void *p) {   /* the static tables the blob stores / passes: the address differs between the blob and the port binary, the name does not */
  if (p == (const void *)c1200_) return "c1200_";
  if (p == (const void *)c2400_) return "c2400_";
  if (p == (const void *)hsine1600) return "hsine1600";
  if (p == (const void *)hsine1680) return "hsine1680";
  if (p == (const void *)hsine1800) return "hsine1800";
  if (p == (const void *)hsine1829) return "hsine1829";
  if (p == (const void *)hsine1867) return "hsine1867";
  if (p == (const void *)hsine1920) return "hsine1920";
  if (p == (const void *)hsine1959) return "hsine1959";
  if (p == (const void *)hsine2000) return "hsine2000";
  if (p == (const void *)scale2400) return "scale2400";
  if (p == (const void *)scale2800) return "scale2800";
  if (p == (const void *)scale3000) return "scale3000";
  if (p == (const void *)scale3200) return "scale3200";
  if (p == (const void *)scale3429) return "scale3429";
  return NULL;
}
static uint32_t hash_tbl(uint32_t h, const uint8_t *t, const HFld *tbl, int nf) {
  int i;
  for (i = 0; i < nf; i++) {
    const HFld *fl = &tbl[i];
    unsigned n = fl->count ? fl->count : 1, es, j;
    const uint8_t *q = t + fl->off;
    switch (fl->kind) {
      case 'p': case 'o':
        for (j = 0; j < n; j++) {
          void *v; int id;
          memcpy(&v, q + j * sizeof v, sizeof v);
          if (!strcmp(fl->name, "rxEnd_") && (uintptr_t)v >= (uintptr_t)g_o->rxFskBuf_ - 0x10000u && (uintptr_t)v < (uintptr_t)g_o->rxFskBuf_ + 0x10000u) { char tb[48]; snprintf(tb, sizeof tb, "bufdelta%ld", (long)((const uint8_t *)v - (const uint8_t *)g_o->rxFskBuf_)); h = fnv_upd(h, tb, strlen(tb)); continue; }   /* the end pointer is relative to the buffer: stable across the architectures */
          if ((const uint8_t *)v >= (const uint8_t *)g_o && (const uint8_t *)v < (const uint8_t *)g_o + sizeof(V34Obj)) { const char *m = memb(v); h = fnv_upd(h, m, strlen(m)); }   /* a pointer into the object: member + delta (the offsets differ per architecture) */
          else if (tabname(v)) { const char *tn = tabname(v); h = fnv_upd(h, tn, strlen(tn)); }
          else { id = tok_of(v); h = fnv_upd(h, &id, sizeof id); }
        }
        continue;
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      default: es = 4; break;
    }
    h = fnv_upd(h, q, (size_t)n * es);
  }
  return h;
}
static void obj_snapshot(char *out, size_t cap, int model) {
  (void)model;
  snprintf(out, cap, "|h=%08x lvl=%u", hash_tbl(2166136261u, (const uint8_t *)g_o, f_V34Obj, NF_V34Obj), dsplibs_debug_level);
}

/* a pointer into the object as MEMBER + delta (the member offsets differ between i386 and x86-64, the member names do not) */
static const char *memb(const void *p) {
  static char b[16][64];
  static int n;
  char *o = b[n++ & 15];
  const uint8_t *q = (const uint8_t *)p, *base = (const uint8_t *)g_o;
  int i, best = -1;
  if (q < base || q >= base + sizeof(V34Obj)) { snprintf(o, 64, "ext"); return o; }
  for (i = 0; i < NF_V34Obj; i++) if (f_V34Obj[i].off <= (size_t)(q - base) && (best < 0 || f_V34Obj[i].off >= f_V34Obj[best].off)) best = i;
  snprintf(o, 64, "obj.%s+%zu", f_V34Obj[best].name, (size_t)(q - base) - f_V34Obj[best].off);
  return o;
}

/* ------------------------------------------------------------------------------------------------ the recorders */
#define ENVREC(...) do { rec_add(&g_il, 0, 'c', __VA_ARGS__); poke(); } while (0)
extern void rc_txmit(void *) L("txmit");
extern void rc_modv(void *) L("modulatevector");
extern void rc_twq(void *, const short *) L("txwritequeue");
extern void rc_echo(void *) L("V34EchoReportCoeff");
extern int rc_v90(void *) L("v90Phase34");
extern int rc_k56(void *) L("k56FlexPhase34");
extern int *rc_setmod(void *, int, int, int, int, int) L("V34SetupModulator");
extern void rc_rxq(void *) L("rxreadqueue");
extern int rc_tone(void *, void *, const short *, const short *) L("tone_detect");
extern void rc_stl(void *, const unsigned short *) L("settxlevel");
extern void rc_txi(void *) L("txinit");
extern int rc_hsi(void *, unsigned) L("v34handshakinit");
extern void rc_i0a(void *, void *) L("V34SetINFO0aBits");
extern void rc_agc(void *) L("V34agc");
extern void rc_fsk(void *, void *, void *) L("fskdemodulate");
#ifdef __i386__
#define RP3 __attribute__((regparm(3)))
#else
#define RP3
#endif
extern void rc_smb(void *, void *) L("VPcmV34SetMohMessageBits");
extern void rc_initd(void *) L("initdigital");
extern void rc_rxinit(void *) L("rxinit");
extern void rc_rti(void *) L("rxtiminginit");
extern int rc_i1a(void *, void *) L("V34SetINFO1aBits");
extern void rc_i0d(void *, void *) L("V34SetINFO0dBits");
extern void rc_pbs(void *) L("probeselect");
extern void rc_abd(void *, int) L("ApplyBulkDelay") RP3;
extern void rc_deti(void *, const void *, int, int, int, int, int) L("detectorinit");
extern void rc_ija(void *) L("indicateJaTransmission");
extern void rc_rsea(void *) L("VPcmV34ReportStartOfEchoAdapt");
extern void rc_rmea(void *) L("VPcmV34ReportMiddleOfEchoAdapt");
extern int16_t rc_getbit(void *) L("getbit") RP3;
extern void rc_g0d(void *, void *) L("V34GiveINFO0dBits");
extern void rc_rcv(void *) L("receiver");
extern void rc_lto(void *, int) L("VPcmV34LogTimingOffset");
extern void rc_rtg(void *) L("rxtiming");
extern void rc_den(void *, int, int) L("dftenergy");
extern void rc_dup(void *, int, const void *, int) L("dftupdate");
extern int rc_gmu(void *) L("VPcmV34GetMaxUpstreamRateIndex");
extern int rc_g1a(void *, void *) L("V34GiveINFO1aBits");
extern int rc_g1d(void *, void *) L("V34GiveINFO1dBits");
extern void rc_gpr(void *, void *) L("V34GiveProbeResults");
extern void rc_imb(void *, void *) L("VPcmV34InterpretMohMessageBits");
extern void rc_sfr(void *) L("setfinalrate");
extern unsigned rc_brev(unsigned, int) L("bitreverse");
void rc_txmit(void *o) { ENVREC("txmit(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_modv(void *o) { ENVREC("modulatevector(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_twq(void *q, const short *s) { ENVREC("txwritequeue(q=%s src=%04x %04x %04x %04x)", memb(q), (unsigned)(uint16_t)s[0], (unsigned)(uint16_t)s[1], (unsigned)(uint16_t)s[2], (unsigned)(uint16_t)s[3]); }
void rc_echo(void *p) { ENVREC("V34EchoReportCoeff(%s)", memb(p)); }
int *rc_setmod(void *m, int baud, int carrier, int preemp, int v90, int full) { ENVREC("V34SetupModulator(m=%s baud=%d carrier=%d preemp=%d v90=%d full=%d)", memb(m), baud, carrier, preemp, v90, full); return 0; }
void rc_i0a(void *o, void *q) { ENVREC("V34SetINFO0aBits(obj=%s q=%s)", o == (void *)g_o ? "obj" : "wild", memb(q)); }
static unsigned g_rv;
static const char *ptok(const void *p) { const char *t = tabname(p); return t ? t : (((const uint8_t *)p >= (const uint8_t *)g_o && (const uint8_t *)p < (const uint8_t *)g_o + sizeof(V34Obj)) ? memb(p) : idn(tok_of(p))); }
static const char *ptok(const void *p);
static const char *endtok(const void *p) { return ((const uint8_t *)p >= (const uint8_t *)g_o && (const uint8_t *)p < (const uint8_t *)g_o + sizeof(V34Obj)) ? memb(p) : idn(tok_of(p)); }
int rc_tone(void *r, void *det, const short *p, const short *end) {
  unsigned v, hit, ret;
  g_rv = g_rv * 1664525u + 1013904223u;
  v = g_rv;
  hit = (v >> 8) & 1u;
  ret = (v & 0xffff0000u) | (hit ? (((v >> 16) & 0xffffu) | 1u) : 0u);   /* a 16-bit result with garbage in the upper half (the blob tests ax) */
  ENVREC("tone_detect(R=%s det=%s p=%s end=%s) -> %04x", memb(r), memb(det), memb(p), endtok(end), ret & 0xffffu);
  return (int)ret;
}
void rc_stl(void *o, const unsigned short *q) { ENVREC("settxlevel(obj=%s q=%s)", o == (void *)g_o ? "obj" : "wild", memb(q)); }
void rc_txi(void *o) { ENVREC("txinit(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
int rc_hsi(void *o, unsigned m) { ENVREC("v34handshakinit(obj=%s mode=%u)", o == (void *)g_o ? "obj" : "wild", m); return 0; }
void rc_rti(void *o) { ENVREC("rxtiminginit(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_pbs(void *o) { ENVREC("probeselect(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_i0d(void *o, void *q) { ENVREC("V34SetINFO0dBits(obj=%s q=%s)", o == (void *)g_o ? "obj" : "wild", memb(q)); }
int rc_i1a(void *o, void *q) { ENVREC("V34SetINFO1aBits(obj=%s q=%s)", o == (void *)g_o ? "obj" : "wild", memb(q)); return 0; }
RP3 void rc_abd(void *o, int d) { ENVREC("ApplyBulkDelay(obj=%s delay=%d)", o == (void *)g_o ? "obj" : "wild", d); }
void rc_rxinit(void *o) { ENVREC("rxinit(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_ija(void *o) { ENVREC("indicateJaTransmission(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_rsea(void *o) { ENVREC("VPcmV34ReportStartOfEchoAdapt(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_rmea(void *o) { ENVREC("VPcmV34ReportMiddleOfEchoAdapt(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_deti(void *b, const void *tab, int p3, int p4, int p5, int p6, int p7) { ENVREC("detectorinit(blk=%s tab=%s %d %d %d %d %d)", memb(b), ptok(tab), p3, p4, p5, p6, p7); }
void rc_initd(void *o) { ENVREC("initdigital(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_smb(void *o, void *q) { ENVREC("VPcmV34SetMohMessageBits(obj=%s q=%s)", o == (void *)g_o ? "obj" : "wild", memb(q)); }
RP3 int16_t rc_getbit(void *q) {   /* the blob's LOCAL getbit (regparm(3), q in eax): the result is scripted -1 / 0 / 1 / other (the callers sign-extend ax) */
  static const int16_t res[] = { -1, 0, 1, 1, 0x100, (int16_t)0x8000, 0, -1 };
  int16_t r;
  g_rv = g_rv * 1664525u + 1013904223u;
  r = res[(g_rv >> 16) & 7u];
  ENVREC("getbit(q=%s) -> %d", memb(q), (int)r);
  return r;
}
void rc_g0d(void *o, void *q) { ENVREC("V34GiveINFO0dBits(obj=%s q=%s)", o == (void *)g_o ? "obj" : "wild", memb(q)); }
int rc_g1a(void *o, void *q) {
  unsigned r;
  g_rv = g_rv * 1664525u + 1013904223u;
  r = (g_rv & 0xffff0000u) | ((g_rv >> 8) & 1u ? ((g_rv >> 3) & 0xffffu) | 1u : 0u);   /* a 16-bit result with garbage in the upper half */
  ENVREC("V34GiveINFO1aBits(obj=%s q=%s) -> %04x", o == (void *)g_o ? "obj" : "wild", memb(q), r & 0xffffu);
  return (int)r;
}
int rc_g1d(void *o, void *q) {
  unsigned r;
  g_rv = g_rv * 1664525u + 1013904223u;
  r = (g_rv >> 9) & 1u ? (g_rv >> 4) | 1u : 0u;   /* the blob tests the full eax */
  ENVREC("V34GiveINFO1dBits(obj=%s q=%s) -> %08x", o == (void *)g_o ? "obj" : "wild", memb(q), r);
  return (int)r;
}
void rc_gpr(void *o, void *p) { ENVREC("V34GiveProbeResults(obj=%s p=%s)", o == (void *)g_o ? "obj" : "wild", memb(p)); }
void rc_imb(void *o, void *q) { ENVREC("VPcmV34InterpretMohMessageBits(obj=%s q=%s)", o == (void *)g_o ? "obj" : "wild", memb(q)); }
int rc_gmu(void *o) {
  static const int base[] = { 0, 1, 0x960, 0x961, 0x95f, 0x12c0, 0x2d00, 0x4b00, 0x7080, 0x8000, 0x10000, 0x7fffffff, -1, -0x960 };
  int r;
  g_rv = g_rv * 1664525u + 1013904223u;
  r = base[(g_rv >> 16) % 14u];
  if ((g_rv >> 12) & 1u) r += 0x960 * (int)((g_rv >> 6) % 16u) + (int)((g_rv >> 3) % 3u) - 1;
  ENVREC("VPcmV34GetMaxUpstreamRateIndex(obj=%s) -> %d", o == (void *)g_o ? "obj" : "wild", r);
  return r;
}
void rc_rtg(void *o) { ENVREC("rxtiming(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
void rc_den(void *p, int n, int m) { int ns = g_nostate; if (g_sc_run == G_T) g_nostate = 1; ENVREC("dftenergy(p=%s n=%d m=%d)", memb(p), n, m); g_nostate = ns; }   /* in the head window the state words must not move before the dispatch */
void rc_dup(void *p, int n, const void *b, int cnt) { int ns = g_nostate; if (g_sc_run == G_T) g_nostate = 1; ENVREC("dftupdate(p=%s n=%d buf=%s cnt=%d)", memb(p), n, memb(b), cnt); g_nostate = ns; }
void rc_rcv(void *o) { g_nostate = 1; ENVREC("receiver(obj=%s)", o == (void *)g_o ? "obj" : "wild"); g_nostate = 0; }
void rc_lto(void *o, int v) { ENVREC("VPcmV34LogTimingOffset(obj=%s %d)", o == (void *)g_o ? "obj" : "wild", v); }
void rc_sfr(void *o) { ENVREC("setfinalrate(obj=%s)", o == (void *)g_o ? "obj" : "wild"); }
unsigned rc_brev(unsigned x, int n) {
  unsigned r;
  g_rv = g_rv * 1664525u + 1013904223u;
  r = (g_rv >> 11) & 0xffffu;
  ENVREC("bitreverse(%u,%d) -> %04x", (unsigned)(uint16_t)x, (int)(int16_t)n, r);
  return (g_rv & 0xffff0000u) | r;   /* ax is the result: the upper half is garbage */
}
void rc_rxq(void *r) { ENVREC("rxreadqueue(R=%s)", memb(r)); }
void rc_agc(void *r) { g_nostate = 1; ENVREC("V34agc(R=%s)", memb(r)); g_nostate = 0; }
void rc_fsk(void *o, void *a, void *b) {
  g_nostate = 1;
  ENVREC("fskdemodulate(obj=%s a=%s b=%s)", o == (void *)g_o ? "obj" : "wild", memb(a), memb(b));
  g_nostate = 0;
  if ((g_sc_run == G_M || g_sc_run == G_T) && h_rpct(50)) g_o->wAae0_ = (uint16_t)h_rnd();   /* the real fskdemodulate updates the INFO0 word: the head's register copy (bx) goes stale */
}
int rc_v90(void *o) { ENVREC("v90Phase34(obj=%s)", o == (void *)g_o ? "obj" : "wild"); return 0; }
int rc_k56(void *o) { ENVREC("k56FlexPhase34(obj=%s)", o == (void *)g_o ? "obj" : "wild"); return 0; }
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  ENVREC("printf %s", buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the unit under test */
#ifdef V04_BLOB
extern void blob_hs(void *) L("v34handshak");
/* v34handshak+0xf7 is `jl v34handshak+0x60` (the loop's back edge): replace it by `jmp v34handshak+0x1e8` (the epilogue: add esp,0xac ; pop ; ret), so the function returns after ONE case */
/* the blob's getbit is a LOCAL regparm(3) function (ELF .text 0x5eaf0; v34handshak is at 0x628f0): overwrite its entry with `mov ecx, rc_getbit ; jmp ecx` (eax = the queue goes straight through) */
static void patch_getbit(void) {
  static const unsigned char sig[17] = { 0x83, 0xec, 0x2c, 0x89, 0x74, 0x24, 0x20, 0x89, 0xc6, 0x89, 0x5c, 0x24, 0x1c, 0x89, 0x7c, 0x24, 0x24 };
  unsigned char *p = (unsigned char *)((uintptr_t)&blob_hs - 0x628f0u + 0x5eaf0u);
  uintptr_t pg = (uintptr_t)p & ~(uintptr_t)4095;
  uint32_t a = (uint32_t)(uintptr_t)&rc_getbit;
  if (memcmp(p, sig, sizeof sig)) { fprintf(stderr, "getbit address arithmetic failed (prologue mismatch)\n"); exit(3); }
  if (mprotect((void *)pg, 8192, PROT_READ | PROT_WRITE | PROT_EXEC)) { perror("mprotect"); exit(3); }
  p[0] = 0xb9; memcpy(p + 1, &a, 4); p[5] = 0xff; p[6] = 0xe1;
}
static void patch_abd(void) {
  static const unsigned char sig[17] = { 0x83, 0xec, 0x1c, 0x89, 0x74, 0x24, 0x14, 0x0f, 0xbf, 0xf2, 0x66, 0x85, 0xf6, 0x89, 0x5c, 0x24, 0x10 };
  unsigned char *p = (unsigned char *)((uintptr_t)&blob_hs - 0x628f0u + 0x5dd10u);
  uintptr_t pg = (uintptr_t)p & ~(uintptr_t)4095;
  uint32_t a = (uint32_t)(uintptr_t)&rc_abd;
  if (memcmp(p, sig, sizeof sig)) { fprintf(stderr, "ApplyBulkDelay address arithmetic failed (prologue mismatch)\n"); exit(3); }
  if (mprotect((void *)pg, 8192, PROT_READ | PROT_WRITE | PROT_EXEC)) { perror("mprotect"); exit(3); }
  p[0] = 0xb9; memcpy(p + 1, &a, 4); p[5] = 0xff; p[6] = 0xe1;
}
/* the blob's LOCAL regparm(3) getbit (.text 0x5eaf0) and ApplyBulkDelay (0x5dd10), called through their addresses by the helper scenarios (their entries are NOT patched there) */
typedef short __attribute__((regparm(3))) (*gb_fn_t)(void *);
typedef void __attribute__((regparm(3))) (*abd_fn_t)(void *, int);
static short gb_call(void *q) { return ((gb_fn_t)((uintptr_t)&blob_hs - 0x628f0u + 0x5eaf0u))(q); }
static void abd_call(void *o, int d) { ((abd_fn_t)((uintptr_t)&blob_hs - 0x628f0u + 0x5dd10u))(o, d); }
static void patch_iso(int sc) {
  if (sc == G_U || sc == G_V) return;
  if (!getenv("V04_WHOLE")) { patch_getbit(); patch_abd(); }   /* V04_WHOLE=1: the blob's own getbit / ApplyBulkDelay run (the port side links the real ports, see build_and_diff_v04.sh) */
  if (postloop_sc(sc)) return;   /* the post-loop scenarios run the WHOLE blob function: no loop patch */
  unsigned char *f = (unsigned char *)(uintptr_t)&blob_hs, *p = f + 0xf7;
  uintptr_t pg = (uintptr_t)p & ~(uintptr_t)4095;
  int32_t rel;
  static const unsigned char epi[] = { 0x81, 0xc4, 0xac, 0x00, 0x00, 0x00, 0x5b, 0x5e, 0x5f, 0x5d, 0xc3 };   /* add esp,0xac ; pop ebx ; pop esi ; pop edi ; pop ebp ; ret */
  /* v34handshak+0xf7 is `jl v34handshak+0x60` (the loop's back edge): replace it by `jmp v34handshak+0x1e8` (the epilogue), so the function returns after ONE case */
  memcpy(&rel, p + 2, 4);
  if (p[0] != 0x0f || p[1] != 0x8c || (int32_t)(p + 6 + rel - f) != 0x60 || memcmp(f + 0x1e8, epi, sizeof epi)) { fprintf(stderr, "v34handshak loop patch: prologue / branch mismatch\n"); exit(3); }
  if (mprotect((void *)pg, 8192, PROT_READ | PROT_WRITE | PROT_EXEC)) { perror("mprotect"); exit(3); }
  rel = (int32_t)((f + 0x1e8) - (p + 5));
  p[0] = 0xe9; memcpy(p + 1, &rel, 4); p[5] = 0x90;
}
static void run_helpers(V34Obj *o);
static void run_iso(V34Obj *o) { if (g_sc_run == G_U || g_sc_run == G_V) { run_helpers(o); return; } blob_hs(o); }
#else
extern void v34handshak(void *);
extern void v04z_tx_dispatch(void *);
extern short v04_getbit_impl(void *);
extern void v04_apply_bulk_delay_impl(void *, int);
static short gb_call(void *q) { return v04_getbit_impl(q); }
static void abd_call(void *o, int d) { v04_apply_bulk_delay_impl(o, d); }
static void run_helpers(V34Obj *o);
static void patch_iso(int sc) { (void)sc; }
static void run_iso(V34Obj *o) {
  if (g_sc_run == G_U || g_sc_run == G_V) { run_helpers(o); return; }
  if (postloop_sc(g_sc_run)) { v34handshak(o); return; }   /* the assembled function: receive chain + TXPOST + status tail */
  v04z_tx_dispatch(o);   /* ONE loop iteration of the assembled function (the blob side: the loop back edge is patched to leave after one case) */
}
#endif

static int g_arg;   /* the delay argument of the ApplyBulkDelay scenario */
/* the helper scenarios: getbit on the INFO0a record (a run of calls), ApplyBulkDelay(obj, delay) */
static void run_helpers(V34Obj *o) {
  int i, n;
  if (g_sc_run == G_U) {
    uint8_t *q = o->i0aUnk00_;
    n = 1 + (int)((g_rv >> 8) % 40u);
    for (i = 0; i < n; i++) {
      uint16_t w28, w18, w1a, w1c, w16, w20;
      int rem;
      short r;
      memcpy(&w28, q + 0x28, 2); memcpy(&w18, q + 0x18, 2); memcpy(&w1a, q + 0x1a, 2); memcpy(&w1c, q + 0x1c, 2); memcpy(&w16, q + 0x16, 2); memcpy(&w20, q + 0x20, 2);
      rem = (int16_t)(uint16_t)(w18 - w1a);
      if (w28) CV(G_U, "gb: bits left in the register");
      else if (rem > 0) {
        if ((int16_t)w1c > rem) CV(G_U, "gb: load the remainder (chunk longer)");
        else if ((int16_t)w1c == rem) CV(G_U, "gb: load a whole chunk == remainder");
        else CV(G_U, "gb: load a whole chunk");
        if (w16) { if ((int16_t)w1c == 0 && (int16_t)w1c <= rem) CV(G_U, "gb: crc skipped (zero count)"); else CV(G_U, "gb: crc fed"); } else CV(G_U, "gb: crc off");
      } else if (rem == 0) { if (w16) CV(G_U, "gb: the 16 crc bits"); else if (w20) CV(G_U, "gb: armed next message at rem 0"); else CV(G_U, "gb: end of message at rem 0"); }
      else if (rem == -16) CV(G_U, "gb: constant 0xf");
      else { if (w20) CV(G_U, "gb: armed next message, other negative remainder"); else CV(G_U, "gb: end of message, negative remainder"); }
      r = gb_call(q);
      rec_add(&g_il, 0, 'c', "getbit -> %d", (int)r);
    }
    return;
  }
  {
    int d = g_arg;
    uint32_t lim;
    lim = o->w35b4_;
    if ((int16_t)d <= 0) CV(G_V, "abd: delay <= 0: 0x90"); else CV(G_V, "abd: delay > 0");
    {
      int e = (int16_t)d <= 0 ? 0x90 : (int16_t)d;
      if ((uint32_t)e >= lim) CV(G_V, "abd: estimate at or above the limit: 0"); else CV(G_V, "abd: estimate below the limit");
      if ((uint32_t)e >= lim) e = 0;
      if (o->pcmMode_ != 0 || o->k56Mode_ != 0) CV(G_V, "abd: pcm / k56 mode: echo flag cleared");
      else if (o->echoOn_ == 0) CV(G_V, "abd: echo already off");
      else if ((int16_t)e > 0x1d) CV(G_V, "abd: echo on, estimate > 0x1d: stays on");
      else { CV(G_V, "abd: masking far EC, dma delay reduced"); if (e + 0xf > 0x1e) CV(G_V, "abd: reduction clamped at 0x1e"); else CV(G_V, "abd: reduction e + 0xf"); }
    }
    if (dsplibs_debug_level > 1u) CV(G_V, "abd: prints"); else CV(G_V, "abd: quiet");
    abd_call(o, d);
    rec_add(&g_il, 0, 'c', "abd done");
  }
}

/* the txstates this chunk ports and the ones the blob's table 1 sends to its loop head without doing anything */
static const uint16_t st_a[] = { 0x12, 0x3c, 0x41, 0x47 };
static const uint16_t st_b[] = { 0x46, 0x51, 0x52, 0x53, 0x54 };
static const uint16_t st_c[] = { 0x4e, 0x55 };
static const uint16_t st_h[] = { 5, 0x36, 0x4a };
static const uint16_t st_d[] = { 0x33, 0x13, 0x14, 0x56, 0x45, 0x40, 0x44 };
static const uint16_t st_noop[] = { 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 22, 23, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 52, 53, 55, 56, 57, 58, 59, 61, 62, 63, 72, 73, 75, 76, 77, 79, 80 };
#define NEL(a) ((unsigned)(sizeof a / sizeof a[0]))
static int is_ported(uint16_t s) { unsigned i; for (i = 0; i < NEL(st_a); i++) if (st_a[i] == s) return 1; for (i = 0; i < NEL(st_b); i++) if (st_b[i] == s) return 1; for (i = 0; i < NEL(st_c); i++) if (st_c[i] == s) return 1; for (i = 0; i < NEL(st_d); i++) if (st_d[i] == s) return 1; for (i = 0; i < NEL(st_h); i++) if (st_h[i] == s) return 1; return 0; }
static int is_noop(uint16_t s) { unsigned i; for (i = 0; i < NEL(st_noop); i++) if (st_noop[i] == s) return 1; return (unsigned)((int)(int16_t)s - 5) > 0x51u; }

static uint16_t pick_state(int sc) {
  switch (sc) {
    case G_A: return st_a[h_rmod(NEL(st_a))];
    case G_B: return st_b[h_rmod(NEL(st_b))];
    case G_C: return st_c[h_rmod(NEL(st_c))];
    case G_D: return st_d[h_rmod(NEL(st_d))];
    case G_H: return st_h[h_rmod(NEL(st_h))];
    case G_I: return 0x18;
    case G_J: return 0x43;
    case G_L: return 0x15;
    case G_POKE: { unsigned k = h_rmod(NEL(st_a) + NEL(st_b) + NEL(st_c) + NEL(st_d)); return k < NEL(st_a) ? st_a[k] : k < NEL(st_a) + NEL(st_b) ? st_b[k - NEL(st_a)] : k < NEL(st_a) + NEL(st_b) + NEL(st_c) ? st_c[k - NEL(st_a) - NEL(st_b)] : st_d[k - NEL(st_a) - NEL(st_b) - NEL(st_c)]; }
    default:
      if (h_rpct(35)) { static const uint16_t sq[] = { 0x12, 0x51, 0x4e, 0x55, 0x46, 0x14, 0x13, 0x33, 0x56 }; return sq[h_rmod(NEL(sq))]; }
      if (h_rpct(30)) { static const uint16_t bad[] = { 0, 1, 4, 0x57, 0x58, 0x7fff, 0x8000, 0xffff, 0xfffb, 0xfffc }; return bad[h_rmod(NEL(bad))]; }
      return st_noop[h_rmod(NEL(st_noop))];
  }
}

static const uint16_t mst_p[] = { 0x3e, 0x30, 0x2f, 0x38, 0x3f, 0x50 };
static const uint16_t mst_noop[] = { 0x2a, 0x2b, 0x2d, 0x34, 0x35, 0x36, 0x39, 0x3c, 0x3d, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e };
static const uint16_t mst_bad[] = { 0, 1, 0x28, 0x51, 0x56, 0x7fff, 0x8000, 0xffff, 0xfff0 };
static uint16_t pick_mst(void) {
  unsigned r = h_rmod(100);
  if (r < 75) return mst_p[h_rmod(NEL(mst_p))];
  if (r < 90) return mst_noop[h_rmod(NEL(mst_noop))];
  return mst_bad[h_rmod(NEL(mst_bad))];
}
static unsigned pick_lv(void) { return h_rpct(65) ? 2u + h_rmod(2) : pick_level(); }
static uint16_t pick_cnt(void) {
  static const uint16_t v[] = { 0, 1, 0x3e, 0x3f, 0x40, 0x41, 0xbe, 0xbf, 0xc0, 0xc1, 0x7fff, 0x8000, 0xfffe, 0xffff };
  return h_rpct(80) ? v[h_rmod(NEL(v))] : (uint16_t)h_rnd();
}
static uint16_t pick_h16(void) { return (uint16_t)(h_rpct(25) ? 0 : h_rpct(30) ? 1 : h_rpct(25) ? 0xffffu : h_rnd()); }

static void fill_tbl(uint8_t *t, const HFld *tbl, int nf) {
  int i;
  for (i = 0; i < nf; i++) {
    const HFld *f = &tbl[i];
    unsigned n = f->count ? f->count : 1, es, tot, j;
    uint8_t *q = t + f->off;
    switch (f->kind) {
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      case 'u': case 'i': case 'f': es = 4; break;
      case 'p': case 'o': {
        unsigned e;
        for (e = 0; e < n; e++) { void *v; uintptr_t r = h_rnd(); r = (r << 4) ^ (uintptr_t)h_rnd(); v = (void *)((r & 0x3ffffff0u) | 1u); memcpy(q + e * sizeof v, &v, sizeof v); }
        continue;
      }
      default: continue;
    }
    tot = n * es;
    for (j = 0; j < tot; j += 4) { uint32_t r = h_rnd(); memcpy(q + j, &r, tot - j < 4 ? tot - j : 4); }
  }
}

static void put16(uint8_t *r, unsigned off, unsigned v) { uint16_t x = (uint16_t)v; memcpy(r + off, &x, 2); }
static void put32(uint8_t *r, unsigned off, uint32_t v) { memcpy(r + off, &v, 4); }
/* a bit-queue record (tag_bitQ, 0x30 B: data words at +0, w14 crc, w16 crc enable, w18 length, w1a position, w1c chunk, w1e word index, w20 more, w22 count, d24 register, w28 bits, w2a / d2c restart) */
static void fill_queue(uint8_t *rec) {
  unsigned pos, rem, chunk;
  put16(rec, 0x14, h_rnd());
  put16(rec, 0x16, h_rpct(60) ? 1 : h_rpct(50) ? 0 : h_rnd());
  pos = h_rpct(30) ? 0 : h_rpct(70) ? h_rmod(40) : (h_rnd() & 0xffffu);
  { unsigned r = h_rmod(100);
    if (r < 28) rem = 1 + h_rmod(30);
    else if (r < 43) rem = 0;
    else if (r < 58) rem = 0xfff0u;
    else if (r < 68) rem = (unsigned)(0x10000 - 1 - (int)h_rmod(40));
    else if (r < 78) rem = 1 + h_rmod(32);
    else rem = h_rnd() & 0xffffu; }
  put16(rec, 0x1a, pos);
  put16(rec, 0x18, pos + rem);
  { unsigned r = h_rmod(100);
    if (r < 45 && (int16_t)rem > 0) chunk = rem + h_rmod(3) - 1;
    else if (r < 70) chunk = 1 + h_rmod(16);
    else if (r < 75) chunk = 0;
    else if (r < 80) chunk = 0x10000u - 1u - h_rmod(8);
    else chunk = h_rnd() & 0xffffu; }
  put16(rec, 0x1c, chunk);
  put16(rec, 0x1e, h_rmod(10));
  put16(rec, 0x20, h_rpct(50) ? 0 : h_rpct(60) ? 1 : h_rnd());
  put16(rec, 0x22, h_rnd());
  put32(rec, 0x24, h_rnd());
  put32(rec, 0x2c, h_rnd());
  put16(rec, 0x28, h_rpct(45) ? 0 : h_rpct(65) ? 1 + h_rmod(32) : (h_rnd() & 0xffffu));
  put16(rec, 0x2a, h_rpct(30) ? 0 : h_rpct(60) ? 1 + h_rmod(30) : (h_rnd() & 0xffffu));
  if (h_rpct(12)) { put16(rec, 0x18, 0xfff0u); if (!h_rpct(50)) put16(rec, 0x1a, 1 + h_rmod(40)); }
}

static uint16_t pick_mm(void) { static const uint16_t mm[] = { 0x31, 0x32, 0x37, 0x33, 0x3a, 0x3b, 0x2e, 0x2e }; return mm[h_rmod(8)]; }
static void new_world(int sc, uint16_t state) {
  uint8_t *raw;
  V34Obj *o;
  size_t osz = sizeof(V34Obj);
  blk_reset();
  g_nobj = 0;
  raw = (uint8_t *)h_obj_new(osz + CAN);
  memset(raw + osz, 0x5a, CAN);
  g_can = raw + osz;
  fill_tbl(raw, f_V34Obj, NF_V34Obj);
  {   /* a random fake pointer must never alias the object (its address differs between the blob and the port binary): move it out of the object's range */
    int k;
    for (k = 0; k < NF_V34Obj; k++) {
      const HFld *fl = &f_V34Obj[k];
      if (fl->kind == 'p' || fl->kind == 'o') {
        uintptr_t v;
        memcpy(&v, raw + fl->off, sizeof v);
        while (v + 0x10000u >= (uintptr_t)raw && v < (uintptr_t)raw + osz + 0x10000u) v += 0x100000u;   /* also out of the +-64 KB window rxEnd_ is printed relative to the buffer in */
        memcpy(raw + fl->off, &v, sizeof v);
      }
    }
  }
  o = (V34Obj *)raw;
  g_o = o;
  /* the loop condition holds (the blob's post-loop code must not run), the states are valid StateName indexes when the transition print can run */
  o->txIdx_ = (uint16_t)(int)((int16_t)h_rnd() / 4);
  o->blkLen_ = (uint16_t)((int16_t)o->txIdx_ + 1 + (int)h_rmod(40));
  if ((int16_t)o->blkLen_ <= (int16_t)o->txIdx_) { o->txIdx_ = 0; o->blkLen_ = 8; }
  o->w3596_ = state;
  o->rxState_ = (uint16_t)h_rmod(87);
  o->mst_ = (uint16_t)h_rmod(87);
  o->txCnt_ = pick_cnt(); o->k56Idx_ = pick_cnt();
  o->txFlags_ = (uint16_t)(h_rpct(50) ? (h_rnd() | 0x10u) : (h_rnd() & ~0x10u));
  o->rxFlags_ = (uint16_t)(h_rpct(50) ? (h_rnd() | 8u) : (h_rnd() & ~8u));
  o->w358c_ = (uint16_t)h_rnd();
  o->modeChar_ = h_rpct(40) ? 0x65 : h_rpct(50) ? 0x66 : (uint16_t)h_rnd();
  o->waa78_ = h_rpct(60) ? (uint16_t)h_rmod(4) : pick_h16();
  o->w2218_ = h_rpct(60) ? (int32_t)h_rmod(3) : (int32_t)h_rnd();
  o->mohEC_ = h_rpct(60) ? (h_rpct(50) ? 1u : 0u) : h_rnd();
  o->mohF0_ = h_rpct(60) ? (h_rpct(50) ? 1u : 0u) : h_rnd();
  o->scrState_ = h_rnd();
  {
    static const uint16_t bauds[] = { 0x960, 0xaf0, 0xbb8, 0xc80, 0xd65 };
    o->baud_ = h_rpct(75) ? bauds[h_rmod(5)] : (uint16_t)h_rnd();
    o->carrier_ = pick_h16(); o->preemp_ = (uint16_t)h_rmod(16); o->period_ = pick_h16(); o->waa7e_ = pick_h16(); o->filtDelay_ = pick_h16();
    o->txGain_ = pick_h16(); o->w358e_ = (uint16_t)(h_rpct(70) ? h_rmod(8) : h_rnd());
    o->w354c_ = h_rnd(); o->w3550_ = (uint16_t)h_rnd(); o->w3552_ = (uint16_t)h_rnd();
    o->w35a4_ = (int16_t)(h_rpct(25) ? 0 : h_rpct(60) ? (int)h_rmod(300) : (int)(int16_t)h_rnd());
    o->w35a6_ = pick_h16();
    o->dmaDelay_ = (uint16_t)(h_rpct(60) ? 0x400 + h_rmod(0x400) : h_rnd());
    o->pcmMode_ = h_rpct(40) ? 0 : (int32_t)h_rnd(); o->k56Mode_ = h_rpct(40) ? 0 : (int32_t)h_rnd();
  }
  if (state == 0x33) { o->mst_ = (uint16_t)(h_rpct(60) ? 0x33 : h_rpct(50) ? 0x34 : h_rmod(87)); if (h_rpct(50)) o->k56Idx_ = (uint16_t)(0x5fc + h_rmod(5)); }
  if (state == 0x13) { if (h_rpct(60)) o->txCnt_ = 7; if (sc == G_D && h_rpct(25)) { o->txFlags_ = 0; o->w35a4_ = (int16_t)(1 + h_rmod(300)); } }   /* gap: SBARSEG with txFlags_ == 0 at the 8th call */
  if (state == 5 || state == 0x36 || state == 0x4a) {
    if (state == 0x36) o->k56Idx_ = (uint16_t)(h_rpct(60) ? 9 : h_rmod(12));
    if (state == 0x4a) { o->k56Idx_ = (uint16_t)(h_rpct(65) ? 0xb3 : h_rmod(0xc0)); o->mohE8_ = (uint8_t)(h_rpct(50) ? 0 : h_rnd()); o->rxState_ = (uint16_t)(h_rpct(30) ? 0x2b : h_rmod(87));
                         o->modeChar_ = h_rpct(45) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd(); o->mst_ = (uint16_t)(h_rpct(30) ? (h_rpct(50) ? 0x3a : 0x2e) : h_rmod(87)); if (h_rpct(15)) o->mst_ = 0x4f; }
    o->pcmMode_ = h_rpct(50) ? 0 : (int32_t)h_rnd();
  }
  if (state == 0x45) { o->rxSig_ = h_rpct(50) ? 0x89b0 : (uint16_t)h_rnd(); o->k56Idx_ = (uint16_t)(h_rpct(50) ? (o->rxSig_ == 0x89b0 ? 0x10 : 0x12) : h_rmod(0x20)); }
  if (state == 0x40 || state == 0x44) {
    o->k56Idx_ = (uint16_t)(h_rpct(40) ? 7 : h_rpct(70) ? h_rmod(8) : h_rnd());
    o->k56Word_ = h_rpct(50) ? 0x899f : (uint16_t)h_rnd();
    o->waa78_ = (uint16_t)(h_rpct(30) ? 1 : h_rpct(20) ? 0 : 2 + h_rmod(3));
    o->w25da_ = (uint16_t)(h_rpct(50) ? 2 : h_rpct(50) ? 0 : h_rnd());
    o->modeChar_ = h_rpct(55) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    o->symCnt_ = (uint16_t)(h_rpct(50) ? 0x81 + h_rmod(8) : h_rpct(50) ? 0x80 : h_rnd());
    o->w35a2_ = pick_h16();
    o->rxFlags_ = (uint16_t)(h_rpct(50) ? (o->rxFlags_ | 0x400u) : (o->rxFlags_ & ~0x400u));
  }
  if ((state == 0x40 || state == 0x44) && sc == G_D && h_rpct(30)) { o->symCnt_ = (uint16_t)(0x80 + h_rmod(2)); o->waa78_ = (uint16_t)(2 + h_rmod(3)); o->w25da_ = 2; o->modeChar_ = 0x65; o->k56Idx_ = 7; }   /* gap: JTXMIT at the symCnt_ boundary */
  if (state == 0x14) { o->k56Idx_ = (uint16_t)(h_rpct(50) ? 0x2f : h_rmod(0x30)); if (h_rpct(60)) o->w358e_ = 5; }
  if (state == 0x56) { if (h_rpct(40)) o->waa78_ = (uint16_t)(o->k56Idx_ + 1u); if (h_rpct(40)) o->w35a6_ = (uint16_t)(o->k56Idx_ + 1u); else if (h_rpct(30)) o->w35a6_ = (uint16_t)(o->k56Idx_ + 2u); }
  if (sc == G_E || sc == G_K || sc == G_M || sc == G_N || sc == G_O || sc == G_P || sc == G_R || sc == G_S || sc == G_T) {   /* the post-loop receive branch: the loop is skipped, RX_DPSK with the dft branch off, `state` = the microstate */
    o->w3596_ = (uint16_t)h_rmod(87);
    o->mst_ = state;
    o->blkLen_ = (uint16_t)(int)((int16_t)o->txIdx_ - (int)h_rmod(5));
    o->rxW0_ = (int16_t)(6 + (int)h_rmod(100));
    o->rxState_ = 0x2b;
    o->dftActive_ = 0;
    o->wAae2_ = (uint16_t)(h_rpct(55) ? (h_rnd() | 0xfu) : h_rnd());
    o->wAae0_ = pick_h16(); o->rxGain_ = (int16_t)h_rnd(); o->filtDelay_ = pick_h16();
    o->w3588_ = (uint16_t)(h_rpct(60) ? 0 : h_rnd()); o->w358a_ = pick_h16(); o->wAbc2_ = pick_h16(); o->shortPhase2_ = pick_h16(); o->wabca_ = pick_h16();
    { static const uint16_t wt[] = { 0x77, 0xa1, 0x5e, 0x5f, 0x60, 0x78, 0xa2, 0x7fff, 0xffff }; o->waa78_ = h_rpct(70) ? wt[h_rmod(NEL(wt))] : pick_h16(); }
    { unsigned k; for (k = 0; k < 10; k++) o->wAbae_[k] = (uint16_t)h_rnd(); }
    if (state == 0x30 && h_rpct(30)) { o->w3596_ = 5; o->waa78_ = 0xa1; }   /* TX_PHASE3_ANS: the 0xa2 step with txstate already SILENCE */
    if (state == 0x3f) {
      o->w3596_ = h_rpct(35) ? 0x3c : h_rpct(65) ? 5 : (uint16_t)h_rmod(87);
      o->waa78_ = (uint16_t)(h_rpct(60) ? 0xe + h_rmod(3) : h_rpct(50) ? 0 : h_rnd());
      o->modeChar_ = h_rpct(50) ? 0x65 : h_rpct(60) ? 0x66 : (uint16_t)h_rnd();
      o->w8_ = h_rpct(50) ? 0x30 : (int32_t)h_rnd();
      { static const uint8_t wb[] = { 0, 1, 0x7f, 0x80, 0xff, 2 }; o->wAbff_ = (uint8_t)(h_rpct(45) ? 1 + h_rmod(100) : h_rpct(50) ? wb[h_rmod(NEL(wb))] : h_rnd()); }
      o->w240_ = h_rpct(60) ? 0x241 + (int32_t)h_rmod(5) : h_rpct(50) ? 0x240 : (int32_t)h_rnd();
    }
    if (state == 0x50) {
      o->waa7e_ = (uint16_t)(h_rpct(50) ? h_rmod(100) : h_rnd());
      o->waa78_ = (uint16_t)(h_rpct(55) ? (unsigned)(0x12c0 + ((int)(int16_t)o->waa7e_ >> 2) - (int)h_rmod(3)) : h_rnd());
      o->mohF8_ = (uint8_t)(h_rpct(50) ? 0 : h_rnd()); o->mohF9_ = (uint8_t)(h_rpct(50) ? 0 : h_rnd());
      o->w3596_ = h_rpct(30) ? 0x54 : (uint16_t)h_rmod(87);
      o->rxState_ = 0x2b;
    }
  }
  if (sc == G_F) {   /* the post-loop skeleton: loop skipped, any receive / transmit state word (never the four unported receive cases) */
    static const uint16_t txi[] = { 5, 0x12, 0x13, 0x14, 0x15, 0x18, 0x33, 0x36, 0x3c, 0x40, 0x42, 0x43, 0x44, 0x45, 0x46, 0x4a, 0x4e, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x41, 0x47, 0x4b, 0x04, 0x4f, 0x50 };
    static const uint16_t rxi[] = { 0x23, 0x23, 0x04, 0x04, 0x2b, 0x35, 0x48, 0x00, 0x01, 0x05, 0x22, 0x24, 0x2c, 0x34, 0x36, 0x47, 0x49, 0x56, 0xffff, 0x7fff, 0x8000 };
    o->blkLen_ = (uint16_t)(int)((int16_t)o->txIdx_ - (int)h_rmod(5));
    o->w3596_ = h_rpct(75) ? txi[h_rmod(NEL(txi))] : (uint16_t)h_rnd();
    o->rxState_ = h_rpct(85) ? rxi[h_rmod(NEL(rxi))] : (uint16_t)h_rnd();
    o->rxW0_ = (int16_t)(h_rpct(70) ? (int)h_rmod(40) : (int)(int16_t)h_rnd());
    { unsigned r = o->rxState_; if ((int16_t)o->rxW0_ > 5 && ((int16_t)r == 4 || (int16_t)r == 0x2b || (int16_t)r == 0x35 || (int16_t)r == 0x48)) o->rxState_ = 0x23; }
    o->w2218_ = h_rpct(75) ? (int32_t)h_rmod(7) : (int32_t)h_rnd();
    o->w238_ = h_rpct(30) ? (int32_t)o->w23c_ : h_rpct(50) ? (int32_t)(o->w23c_ + 1u) : o->w238_;
    o->minSigLevel_ = h_rpct(60) ? (int32_t)(int16_t)o->rxLevel_ + (int32_t)h_rmod(3) - 1 : (int32_t)h_rnd();
    o->w234_ = h_rpct(60) ? (int32_t)(0x257e + (int)h_rmod(3)) : (int32_t)h_rnd();
    o->wE4c_ = (uint16_t)(h_rpct(50) ? 0 : h_rnd());
    o->mst_ = (uint16_t)(h_rpct(40) ? (h_rpct(50) ? 0x3f : 0x2c) : h_rnd());
    o->k56Idx_ = (uint16_t)(h_rpct(50) ? 0x3b + h_rmod(3) : h_rnd());
    o->rxFlags_ = (uint16_t)h_rnd();
    if (h_rpct(25)) {   /* steer the SILENCE status stub (txstate 5): receive state RECEIVE (gate closed) or WAIT, microstate 0x3f / 0x2c, role 0x65 / 0x66 */
      o->w3596_ = 5;
      o->rxState_ = h_rpct(50) ? 4 : 0x23;
      if (o->rxState_ == 4) o->rxW0_ = (int16_t)h_rmod(6);
      o->mst_ = h_rpct(50) ? 0x3f : 0x2c;
      o->modeChar_ = h_rpct(50) ? 0x65 : 0x66;
    }
  }
  if (sc == G_I) {   /* TX_DPSK: the bit queue is the INFO0a record, the MOH state machine decides at the end of a message */
    unsigned pos, rem, chunk;
    o->bitQ_ = o->i0aUnk00_;
    o->mohE8_ = h_rpct(20) ? 0 : (uint8_t)(1 + h_rmod(255));
    o->mohF8_ = h_rpct(35) ? 0 : (uint8_t)(1 + h_rmod(255));
    o->mohF9_ = h_rpct(45) ? 0 : (uint8_t)(1 + h_rmod(255));
    o->mohF0_ = h_rpct(88) ? h_rmod(6) : h_rnd();
    o->mohF4_ = h_rpct(88) ? (h_rpct(35) ? 5u : h_rmod(7)) : h_rnd();
    o->mohE2_ = (uint16_t)(h_rpct(40) ? 3 : h_rmod(5));
    o->w3596_ = 0x18;
    o->rxState_ = (uint16_t)(h_rpct(40) ? 0x23 : h_rmod(87));
    o->w358c_ = (uint16_t)h_rnd();
    o->waa7e_ = (uint16_t)(h_rpct(60) ? h_rmod(300) : h_rnd());
    if (h_rpct(55)) o->k56Idx_ = (uint16_t)(((int)(int16_t)o->waa7e_ >> 4) + 0x4b0 + (int)h_rmod(5) - 3);
    o->i0a14_ = (uint16_t)h_rnd();
    o->i0a16_ = (uint16_t)(h_rpct(60) ? 1 : h_rpct(50) ? 0 : h_rnd());
    pos = h_rpct(30) ? 0 : h_rpct(70) ? h_rmod(40) : (h_rnd() & 0xffffu);
    { unsigned r = h_rmod(100);
      if (r < 28) rem = 1 + h_rmod(30);
      else if (r < 43) rem = 0;
      else if (r < 58) rem = 0xfff0u;
      else if (r < 68) rem = (unsigned)(0x10000 - 1 - (int)h_rmod(40));
      else if (r < 78) rem = 1 + h_rmod(32);
      else rem = h_rnd() & 0xffffu; }
    o->i0a1a_ = (uint16_t)pos;
    o->i0a18_ = (uint16_t)(pos + rem);
    { unsigned r = h_rmod(100);
      if (r < 45 && (int16_t)rem > 0) chunk = rem + h_rmod(3) - 1;
      else if (r < 70) chunk = 1 + h_rmod(16);
      else if (r < 75) chunk = 0;
      else if (r < 80) chunk = 0x10000u - 1u - h_rmod(8);
      else chunk = h_rnd() & 0xffffu; }
    o->i0a1c_ = (uint16_t)chunk;
    o->i0a1e_ = (uint16_t)h_rmod(10);
    o->i0a20_ = (uint16_t)(h_rpct(50) ? 0 : h_rpct(60) ? 1 : h_rnd());
    o->i0a22_ = (uint16_t)h_rnd();
    o->i0a24_ = h_rnd();
    o->i0a2c_ = h_rnd();
    o->i0a28_ = (uint16_t)(h_rpct(45) ? 0 : h_rpct(65) ? 1 + h_rmod(32) : (h_rnd() & 0xffffu));
    o->i0a2a_ = (uint16_t)(h_rpct(30) ? 0 : h_rpct(60) ? 1 + h_rmod(30) : (h_rnd() & 0xffffu));
    if (h_rpct(12)) { o->i0a18_ = 0xfff0u; if (o->i0a1a_ == 0) o->i0a1a_ = (uint16_t)(1 + h_rmod(40)); }
    if (h_rpct(35)) {   /* steer to the end of the message: empty register, negative remainder, no next message (or an armed one: getbit decides) */
      o->i0a28_ = 0;
      o->i0a1a_ = (uint16_t)(h_rpct(50) ? 1 + h_rmod(40) : 0);
      o->i0a18_ = (uint16_t)(o->i0a1a_ - 1 - (int)h_rmod(20) - (h_rpct(50) ? 1 : 0));
      if (o->i0a18_ == (uint16_t)(o->i0a1a_ - 16)) o->i0a18_ = (uint16_t)(o->i0a18_ - 1);
      o->i0a20_ = (uint16_t)(h_rpct(60) ? 0 : 1);
      if (o->mohE8_ == 0 && h_rpct(75)) o->mohE8_ = 1;
    }
  }
  if (sc == G_N) {   /* DET_AB: the receive state is the case, the microstate / counter gates are steered */
    o->bitQ_ = o->i0aUnk00_;
    fill_queue(o->i0aUnk00_);
    o->rxState_ = 0x35;
    o->mst_ = (uint16_t)(h_rpct(70) ? 0x34 : h_rmod(87));
    o->k56Idx_ = (uint16_t)(h_rpct(60) ? 0x5da + h_rmod(4) : h_rpct(50) ? (unsigned)h_rmod(0x40) : (unsigned)h_rnd());
    o->waa7e_ = (uint16_t)(h_rpct(60) ? h_rmod(0x20) : h_rnd());
    if (h_rpct(25)) o->k56Idx_ = (uint16_t)((int)(int16_t)o->waa7e_ + 0x2418 + (int)h_rmod(3) - 1);
    o->modeChar_ = h_rpct(45) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    o->w3596_ = (uint16_t)(h_rpct(25) ? 0x18 : h_rpct(25) ? 0x3c : h_rmod(87));
    o->w4c8_ = (uint16_t)h_rnd();
    o->bitQ2_ = h_rpct(50) ? (void *)o->i1cUnk00_ : (void *)o->i0dUnk00_;
  }
  if (sc == G_M || sc == G_T) {   /* the answer / call microstates: counters around their thresholds, INFO0 words, recovery gates */
    static const uint16_t wa2[] = { 0x372, 0x372 | 0x400, 0x201, 0x200, 0x3ff, 1, 0, 7, 0xf, 0xffff, 0x373, 0x8000 };
    unsigned r;
    o->bitQ_ = h_rpct(60) ? (void *)o->i0aUnk00_ : h_rpct(40) ? (void *)o->i0dUnk00_ : h_rpct(50) ? (void *)o->i1aUnk00_ : (void *)o->mpUnk00_;   /* any valid record: the recoveries re-point it at info0a_ */
    fill_queue(o->i0dUnk00_); fill_queue(o->i1aUnk00_); fill_queue(o->mpUnk00_);
    o->mst_ = state;
    o->rxState_ = 0x2b;
    o->filtDelay_ = (uint16_t)(h_rpct(75) ? h_rmod(120) : h_rnd());
    o->wAae2_ = (uint16_t)(h_rpct(75) ? wa2[h_rmod(12)] : h_rnd());
    o->w3588_ = (uint16_t)(h_rpct(45) ? 0 : h_rpct(35) ? 2 : h_rpct(50) ? 3 : h_rnd());
    o->w358a_ = (uint16_t)(h_rpct(50) ? 2 : h_rpct(50) ? 1 : h_rnd());
    o->shortPhase2_ = (uint16_t)(h_rpct(45) ? 0 : h_rpct(60) ? 1 : h_rnd());
    o->modeChar_ = h_rpct(45) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    o->pcmMode_ = h_rpct(40) ? 0 : (int32_t)h_rnd();
    o->w3596_ = (uint16_t)(h_rpct(40) ? 0x18 : h_rpct(30) ? 0x33 : h_rmod(87));
    o->waa7e_ = (uint16_t)(h_rpct(50) ? h_rmod(8) : h_rnd());
    o->wac02_ = (uint16_t)(h_rpct(50) ? h_rmod(100) : h_rnd());
    o->rxGain_ = (uint16_t)(h_rpct(60) ? (h_rpct(50) ? 0x2000 + (int)h_rmod(3) - 1 : 0x1000 + (int)h_rmod(3) - 1) : (int)h_rnd());
    o->baud_ = (uint16_t)(h_rpct(80) ? (h_rpct(25) ? 0xab7 : (h_rpct(50) ? 0x960 : h_rpct(50) ? 0xaf0 : h_rpct(50) ? 0xbb8 : h_rpct(50) ? 0xc80 : 0xd65)) : h_rnd());
    if (h_rpct(25)) o->baud_ = (uint16_t)(o->baud_ + (int)h_rmod(5) - 2);   /* neighbours of the ladder constants */
    o->carrier_ = (uint16_t)(h_rpct(50) ? 0x725 : h_rpct(60) ? 0x7d0 : h_rnd());
    if (h_rpct(20)) o->carrier_ = (uint16_t)(o->carrier_ + (int)h_rmod(5) - 2);
    fill_queue(o->i0aUnk00_);
    if (state == 0x31) {
      r = h_rmod(100);
      o->waa78_ = (uint16_t)(r < 20 ? 0x27 + h_rmod(3) : r < 50 ? (unsigned)(o->filtDelay_ + 0x4a + h_rmod(5)) : r < 80 ? (unsigned)(o->filtDelay_ + 0x4e + h_rmod(6)) : h_rpct(50) ? (unsigned)h_rmod(0x2a) : (unsigned)h_rnd());
    }
    if (state == 0x32) {
      r = h_rmod(100);
      o->waa78_ = (uint16_t)(r < 20 ? 0x31 + h_rmod(3) : r < 50 ? (unsigned)(o->filtDelay_ + 0x4a + h_rmod(5)) : r < 80 ? (unsigned)(o->filtDelay_ + 0x4e + h_rmod(6)) : h_rpct(50) ? (unsigned)h_rmod(0x33) : (unsigned)h_rnd());
    }
    if (state == 0x33) o->waa78_ = (uint16_t)(h_rpct(60) ? 0x29 : h_rpct(50) ? h_rmod(0x30) : h_rnd());
    if (state == 0x37) {
      r = h_rmod(100);
      o->waa78_ = (uint16_t)(r < 30 ? 0x5e + h_rmod(3) : r < 50 ? h_rmod(4) : r < 70 ? (unsigned)h_rnd() : (unsigned)(0x5e + h_rmod(3)));
      o->wAae0_ = (uint16_t)h_rnd();
    }
    if (state == 0x2e) {
      static const uint16_t w2[] = { 0, 0, 0x372, 0xf72, 0x4f72, 0x372 | 0x400, 1, 0xffff, 0xf71 };
      unsigned r2 = h_rmod(100);
      o->w3596_ = (uint16_t)(h_rpct(35) ? 0x18 : h_rpct(65) ? 0x3c : h_rmod(87));
      o->waa78_ = (uint16_t)(r2 < 25 ? 0x18d + h_rmod(4) : r2 < 45 ? 0xc6 + h_rmod(4) : r2 < 65 ? 0x4ae + h_rmod(4) : h_rpct(50) ? (unsigned)h_rmod(0x30) : (unsigned)h_rnd());
      o->wAae2_ = (uint16_t)(h_rpct(80) ? w2[h_rmod(9)] : h_rnd());
      o->w3588_ = (uint16_t)(h_rpct(40) ? 0 : h_rpct(55) ? 2 : h_rpct(50) ? 4 : h_rnd());
      o->b_ac00_ = (uint8_t)(h_rpct(50) ? 0 : 1 + h_rmod(255));
      o->wAa7a_ = (uint16_t)(h_rpct(70) ? 0xa + h_rmod(5) : h_rnd());
      o->modeChar_ = h_rpct(40) ? 0x66 : h_rpct(60) ? 0x65 : (uint16_t)h_rnd();
      o->bitQ2_ = h_rpct(50) ? (void *)o->i0dUnk00_ : h_rpct(50) ? (void *)o->i1cUnk00_ : (void *)o->mpUnk00_;
    }
    if (state == 0x3b) {
      static const uint16_t w2[] = { 7, 8, 0x18, 0x372, 0xf, 0x8, 0x18, 0x17, 1, 0x28, 0xffff };
      r = h_rmod(100);
      o->waa78_ = (uint16_t)(r < 15 ? 0x28 + h_rmod(3) : r < 35 ? 0x125d + h_rmod(4) : r < 65 ? (unsigned)(o->filtDelay_ + 0x5a + h_rmod(5)) : h_rpct(50) ? (unsigned)h_rmod(0x30) : (unsigned)h_rnd());
      o->wAae2_ = (uint16_t)(h_rpct(80) ? w2[h_rmod(11)] : h_rnd());
      o->w402_ = (uint16_t)(h_rpct(50) ? 0 : h_rpct(70) ? 1 : h_rnd());
      o->w3596_ = (uint16_t)(h_rpct(50) ? 5 : h_rmod(87));
      o->waa7e_ = (uint16_t)(h_rpct(50) ? h_rmod(8) : h_rnd());
    }
    if (state == 0x3a) {
      r = h_rmod(100);
      o->waa78_ = (uint16_t)(r < 20 ? 0x5e + h_rmod(3) : r < 45 ? 0x3be + h_rmod(3) : r < 70 ? 0x4af + h_rmod(3) : h_rpct(50) ? (unsigned)h_rmod(4) : (unsigned)h_rnd());
      if (h_rpct(40)) o->w3596_ = 0x18;
    }
  }
  if (sc == G_O) {   /* DET_SYNC: the sub-state w358a_, the INFO0 receive flags, the window counters around their thresholds */
    static const uint16_t wa2[] = { 0x72, 0x172, 0xff72, 0x0, 0xffff, 0xf, 0x372, 0x1, 0x71, 0x73 };
    unsigned r = h_rmod(100);
    o->mst_ = 0x29;
    o->rxState_ = 0x2b;
    o->wAae2_ = (uint16_t)(h_rpct(85) ? wa2[h_rmod(10)] : h_rnd());
    o->mohE8_ = (uint8_t)(h_rpct(45) ? 0 : 1 + h_rmod(255));
    o->w358a_ = (uint16_t)(h_rpct(30) ? 0 : h_rpct(38) ? 1 : h_rpct(60) ? 2 : h_rpct(70) ? h_rmod(4) : h_rnd());
    o->waa7e_ = (uint16_t)(h_rpct(50) ? h_rmod(300) : h_rpct(50) ? h_rnd() : (unsigned)(h_rmod(40) << 4));
    o->wAae0_ = (uint16_t)(r < 20 ? 0x62 + h_rmod(5) : r < 35 ? 0x382 + h_rmod(5) : r < 70 ? (unsigned)(((int)(int16_t)o->waa7e_ >> 4) + 0x62 + (int)h_rmod(5)) : r < 80 ? (unsigned)(((int)(int16_t)o->waa7e_ >> 4) + 0x18e + (int)h_rmod(5)) : r < 90 ? (unsigned)(((int)(int16_t)o->waa7e_ >> 4) + 0x1e8 + (int)h_rmod(5)) : h_rnd());
    o->w35a0_ = (uint16_t)(h_rpct(40) ? 0x2f + h_rmod(5) : h_rpct(50) ? h_rmod(4) : h_rnd());
    o->w3596_ = (uint16_t)(h_rpct(25) ? 0x3c : h_rpct(25) ? 0x18 : h_rpct(25) ? 0x4a : h_rmod(87));
    o->modeChar_ = h_rpct(50) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    o->mohF0_ = h_rpct(60) ? 1u : h_rpct(50) ? 0u : h_rnd();
    o->mohF8_ = (uint8_t)(h_rpct(50) ? 0 : 1 + h_rmod(255));
    o->w3588_ = (uint16_t)(h_rpct(50) ? 0 : h_rnd());
    o->pcmMode_ = h_rpct(40) ? 0 : (int32_t)h_rnd();
    o->wAbc2_ = (uint16_t)(h_rpct(15) ? 0xffffu : h_rmod(10));
    { unsigned k; for (k = 0; k < 10; k++) o->wAbae_[k] = (uint16_t)h_rnd(); }
    o->wAae0_ = (o->wAae0_ == 0 && h_rpct(50)) ? 0x65 : o->wAae0_;
    fill_queue(o->i0aUnk00_); fill_queue(o->i0dUnk00_); fill_queue(o->i1aUnk00_); fill_queue(o->i1cUnk00_); fill_queue(o->mpUnk00_);
    o->bitQ_ = h_rpct(50) ? (void *)o->i0aUnk00_ : h_rpct(50) ? (void *)o->i1aUnk00_ : (void *)o->mpUnk00_;
    o->bitQ2_ = h_rpct(60) ? (void *)o->i0dUnk00_ : h_rpct(50) ? (void *)o->i1cUnk00_ : (void *)o->mpUnk00_;
    { uint8_t *q = (uint8_t *)o->bitQ_; q[4] = (uint8_t)(h_rpct(55) ? (q[4] | 0x80u) : (q[4] & 0x7fu)); }
    { uint16_t w = (uint16_t)(h_rpct(55) ? 1 : h_rnd()); memcpy(o->unka24a_, &w, 2); }
  }
  if (sc == G_P) {   /* DET_INFO: one INFO bit per call; the record length w18_ and the bit counter waa78_ around the CRC end */
    static const uint16_t lens[] = { 0x11, 0x1e, 0x4d, 0x26, 8, 0x11, 0x1e, 0x4d, 0x26, 8, 0x20, 0x38, 0x3f, 0, 1, 7, 0x10, 0x40 };
    static const uint16_t bd[] = { 0x960, 0xaf0, 0xab7, 0xbb8, 0xc80, 0xd65 };
    static const uint16_t cr[] = { 0x640, 0x690, 0x708, 0x725, 0x74b, 0x780, 0x7a7, 0x7d0 };
    unsigned r = h_rmod(100);
    uint8_t *q;
#define Q18 (*(uint16_t *)(q + 0x18))
#define Q14 (*(uint16_t *)(q + 0x14))
    o->mst_ = 0x2c;
    o->rxState_ = 0x2b;
    fill_queue(o->i0aUnk00_); fill_queue(o->i0dUnk00_); fill_queue(o->i1aUnk00_); fill_queue(o->i1cUnk00_); fill_queue(o->mpUnk00_);
    o->bitQ2_ = h_rpct(55) ? (void *)o->i0dUnk00_ : h_rpct(50) ? (void *)o->i1cUnk00_ : h_rpct(50) ? (void *)o->mpUnk00_ : (void *)o->i1aUnk00_;
    o->bitQ_ = h_rpct(60) ? (void *)o->i0aUnk00_ : h_rpct(50) ? (void *)o->i1aUnk00_ : (void *)o->mpUnk00_;
    q = (uint8_t *)o->bitQ2_;
    Q18 = lens[h_rmod(NEL(lens))];
    if (h_rpct(8)) Q18 = (uint16_t)h_rnd();
    Q14 = (uint16_t)(h_rpct(60) ? (h_rnd() & 0x7fffu) : h_rnd());
    if (h_rpct(25)) { static const uint16_t sp[] = { 0xffff, 0xfffe, 0xfffd, 0x8000, 0x7fff, 0, 1, 0x1021, 0x8001 }; Q14 = sp[h_rmod(9)]; }   /* the CRC register's special values: the initial 0xffff, the sign boundary, zero */
    o->wAae0_ = (uint16_t)(h_rpct(10) ? 0 : h_rpct(30) ? 1 : h_rpct(60) ? 2 + h_rmod(50) : h_rnd());
    if (r < 55) o->waa78_ = (uint16_t)((int)(int16_t)Q18 + 0xf);                        /* the CRC end: waa78_ + 1 == w18_ + 0x10 */
    else if (r < 62) o->waa78_ = (uint16_t)((int)(int16_t)Q18 + 0xe + (int)h_rmod(3));  /* neighbours */
    else if (r < 80) o->waa78_ = (uint16_t)(8 * (1 + (int)h_rmod(10)) - 1);                 /* the byte stores: waa78_ + 1 a multiple of 8 */
    else if (r < 90) o->waa78_ = (uint16_t)h_rmod(0x60);
    else o->waa78_ = pick_h16();
    o->wAae2_ = (uint16_t)h_rnd();
    if (h_rpct(70)) o->wAae2_ = (uint16_t)(Q14 ^ (h_rpct(25) ? (h_rnd() & 0xffffu) : 0u));    /* the received CRC equals the register (mostly) */
    o->w358a_ = (uint16_t)(h_rpct(70) ? 1 : h_rpct(50) ? 0 : h_rmod(4));
    {
      uint8_t *bq = (uint8_t *)o->bitQ_;
      bq[4] = (uint8_t)(h_rpct(50) ? (bq[4] | 0x80u) : (bq[4] & 0x7fu));
      bq = (uint8_t *)o->bitQ2_;
      bq[4] = (uint8_t)(h_rpct(50) ? (bq[4] | 0x80u) : (bq[4] & 0x7fu));
    }
    o->modeChar_ = h_rpct(40) ? 0x65 : h_rpct(50) ? 0x66 : (uint16_t)h_rnd();
    o->shortPhase2_ = (uint16_t)(h_rpct(50) ? 0 : h_rpct(60) ? 1 : h_rnd());
    o->mohF0_ = h_rpct(55) ? 1u : h_rpct(50) ? 0u : h_rnd();
    o->w3596_ = (uint16_t)(h_rpct(25) ? 0x18 : h_rmod(87));
    o->pcmMode_ = h_rpct(40) ? 0 : (int32_t)h_rnd();
    o->filtDelay_ = pick_h16();
    o->baud_ = (uint16_t)(h_rpct(75) ? bd[h_rmod(6)] : h_rnd());
    o->waa96_ = (uint16_t)(h_rpct(88) ? bd[h_rmod(6)] : h_rnd());
    o->waaa8_ = (uint16_t)(h_rpct(88) ? cr[h_rmod(8)] : h_rnd());
    if (h_rpct(20)) o->waa96_ = (uint16_t)(o->waa96_ + (int)h_rmod(5) - 2);
    if (h_rpct(20)) o->waaa8_ = (uint16_t)(o->waaa8_ + (int)h_rmod(5) - 2);
    o->rxGainEst_ = (int16_t)h_rnd();
    { unsigned k; for (k = 0; k < 10; k++) o->wAbae_[k] = (uint16_t)h_rnd(); }
    o->mst_ = 0x2c;
    if ((int16_t)Q18 > 0x40 && !(Q18 == 0x4d || Q18 == 0x26 || Q18 == 8) && (int)(int16_t)(o->waa78_ + 1u) == (int)(int16_t)Q18 + 0x10) Q18 = 0x11;   /* the INFO0 word copy stays inside the ten-word array */
  #undef Q18
#undef Q14
  }
  if (sc == G_Q) {   /* TRNSEG4A: the MP data-rate selection */
    static const uint16_t bd[] = { 0x960, 0xaf0, 0xbb8, 0xc80, 0xd65 };
    static const short *const tabs[] = { scale2400, scale2800, scale3000, scale3200, scale3429 };
    unsigned t = h_rmod(100);
    int tgt, r0 = 9;
    uint16_t w;
    o->w3596_ = 0x42;
    o->modeChar_ = h_rpct(50) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    o->rxSig_ = (uint16_t)(h_rpct(50) ? 0x89b0 : h_rpct(70) ? 0x8990 : h_rnd());
    o->baud_ = (uint16_t)(h_rpct(85) ? bd[h_rmod(5)] : (h_rpct(50) ? h_rmod(400) : h_rnd()));
    o->period_ = (uint16_t)(h_rpct(75) ? h_rmod(200) : h_rpct(50) ? h_rnd() & 0x7fffu : h_rnd());
    tgt = (int)(int16_t)o->baud_ + ((int)(int16_t)o->baud_ >> 1) + (int)(int16_t)o->period_;
    if (t < 45) o->txCnt_ = (uint16_t)(tgt - 1);
    else if (t < 55) o->txCnt_ = (uint16_t)(tgt - 2 - (int)h_rmod(3));
    else if (t < 68) o->txCnt_ = (uint16_t)(tgt + (int)h_rmod(3));
    else if (t < 80) o->txCnt_ = (uint16_t)((int)(int16_t)o->baud_ + (int)(int16_t)o->period_ - 2 + (int)h_rmod(5));
    else if (t < 90) o->txCnt_ = (uint16_t)(tgt + 2 + (int)h_rmod(200));
    else o->txCnt_ = pick_cnt();
    o->w2218_ = h_rpct(40) ? (int32_t)h_rmod(4) : h_rpct(50) ? (int32_t)(4 + h_rmod(3)) : h_rpct(50) ? (int32_t)h_rnd() : (int32_t)(h_rnd() | 0x80000000u);
    o->waa96_ = bd[h_rmod(5)];
    o->scaleTab_ = (void *)tabs[h_rmod(5)];
    o->scalePtr_ = (void *)tabs[h_rmod(5)];
    o->w359a_ = (uint16_t)(h_rpct(80) ? 0 : h_rnd());
    o->rxFlags_ = (uint16_t)(h_rpct(55) ? (o->rxFlags_ & ~0x20u) : (o->rxFlags_ | 0x20u));
    if (h_rpct(40)) o->rxFlags_ = (uint16_t)(o->rxFlags_ | 0x1000u); else if (h_rpct(30)) o->rxFlags_ = (uint16_t)(o->rxFlags_ & ~0x1000u);
    o->w47e_ = (int16_t)(h_rpct(55) ? (int)h_rmod(0x1ff + 8) : h_rpct(30) ? (int)h_rnd() : (int)h_rmod(0xd0));
    { uint16_t dv = (uint16_t)(h_rpct(65) ? (unsigned)(o->w47e_ - 1 - (int)h_rmod(60)) : h_rpct(50) ? h_rmod(300) : h_rnd()); memcpy(o->unk0488_, &dv, 2); }
    { unsigned k; for (k = 0; k < 6; k++) { int16_t c = (int16_t)(h_rpct(60) ? (int)h_rmod(0x2000) - 0x1000 : h_rpct(60) ? (int)h_rmod(0x6000) - 0x3000 : (int)(int16_t)h_rnd()); memcpy(o->unk04ca_ + 0x22 + 2 * k, &c, 2); } }
    if (h_rpct(60)) { int16_t c = (int16_t)(h_rpct(50) ? 0x7ff0 : (int)h_rmod(0x7fff)); memcpy(o->unk04ca_ + 0x22 + 2 * h_rmod(6), &c, 2); }
    switch (o->waa96_) { case 0x960: r0 = 9; break; case 0xaf0: r0 = 0xb; break; case 0xbb8: r0 = 0xc; break; case 0xc80: r0 = 0xd; break; default: r0 = 0xe; break; }
    {   /* the SNR estimate around the thresholds of the table row (an independent evaluation of the formula) */
      unsigned k = h_rmod(100);
      int r = r0 - (int)h_rmod(5);
      int v = (int)((const short *)o->scaleTab_)[r + 14 - 1] >> 5, sq = (int16_t)(v * v), thr = (int16_t)((sq * (r >= 0xd ? 0x4268 : 0x3a98)) >> 14);
      if (k < 60) w = (uint16_t)(thr - 1 + (int)h_rmod(3)); else if (k < 80) w = (uint16_t)(h_rmod(64)); else w = (uint16_t)h_rnd();
      memcpy(o->unk04b0_ + 4, &w, 2);
    }
    w = (uint16_t)(h_rpct(30) ? h_rmod(2) : h_rpct(30) ? 2 : h_rpct(50) ? 3 + h_rmod(5) : h_rnd()); memcpy((uint8_t *)&o->w4c2_, &w, 2);
    w = (uint16_t)(h_rpct(85) ? (unsigned)(r0 - 3 + (int)h_rmod(7)) : h_rmod(13)); memcpy((uint8_t *)&o->w4c4_, &w, 2);
    if (h_rpct(10)) { static const uint16_t sp[] = { 0xffff, 0xfffe, 0xfffd }; w = sp[h_rmod(3)]; memcpy((uint8_t *)&o->w4c4_, &w, 2); }   /* a negative reference rate word (the table row stays in range: r = -4 .. 0) */
    { uint16_t n1 = 1; memcpy(o->unkaa9a_ + 0xc, &n1, 2); }
    fill_queue(o->mpUnk00_);
    o->i0dUnk00_[2] = (uint8_t)(h_rpct(50) ? (o->i0dUnk00_[2] | 4u) : (o->i0dUnk00_[2] & ~4u));
    o->w248_ = (int32_t)h_rnd();
    o->w238_ = (int32_t)(h_rpct(40) ? (unsigned)o->w248_ + 0x17700u + h_rmod(3) - 1u : h_rnd());
    o->minRateIdx_ = (int32_t)(h_rpct(80) ? h_rmod(15) : h_rnd());
    o->maxRateIdx_ = (int32_t)(h_rpct(80) ? (unsigned)o->minRateIdx_ + h_rmod(15) : h_rnd());
    o->w228_ = (int32_t)(h_rpct(60) ? h_rmod(16) : h_rnd());
    o->w22c_ = (int32_t)(h_rpct(30) ? o->w228_ : h_rpct(30) ? -1 - (int)h_rmod(3) : h_rpct(70) ? (int)h_rmod(16) : (int)h_rnd());
    o->waa88_ = (int16_t)h_rnd();
    o->w358c_ = (uint16_t)h_rnd();
  }
  if (sc == G_R) {   /* RX_L1: the symbol counter around the dispatch thresholds, the INFO0 flags, the timers */
    static const unsigned cnts[] = { 0x3f, 0x40, 0x41, 0x17f, 0x180, 0x181, 0x1c0, 0x1c1, 0x2ff, 0x300, 0x301, 0x43f, 0x440, 0x441, 0x442, 0x20, 0x7fff, 0x8000, 0xffff };
    unsigned r = h_rmod(100), k;
    o->rxState_ = 0x48;
    o->mst_ = (uint16_t)(h_rpct(40) ? 0x3e : h_rpct(30) ? 0x33 : h_rmod(87));
    o->w3596_ = (uint16_t)(h_rpct(30) ? 0x3c : h_rmod(87));
    o->modeChar_ = h_rpct(50) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    if (r < 55) o->waa78_ = (uint16_t)(cnts[h_rmod(NEL(cnts))] - 1u);
    else if (r < 75) o->waa78_ = (uint16_t)(0x442 + h_rmod(0x2000));
    else if (r < 90) o->waa78_ = (uint16_t)h_rmod(0x450);
    else o->waa78_ = pick_h16();
    if ((int16_t)o->waa78_ >= 0x441 && h_rpct(50)) {   /* the timers: waa78_ + 1 == (waa7e_ >> 2) + 0xf14 / 0xf2c (neighbours) */
      static const int toff[] = { 0xf10, 0xf11, 0xf14, 0xf15, 0xf13, 0xf2c, 0xf2b, 0xf2d };
      int target = (int)(int16_t)(o->waa78_ + 1u), v = (target - toff[h_rmod(8)]) * 4 + (int)h_rmod(4);
      if (v >= -32768 && v <= 32767) o->waa7e_ = (uint16_t)v;
    }
    o->wAae0_ = (uint16_t)(h_rpct(50) ? 0x14 + h_rmod(4) : h_rpct(50) ? h_rmod(0x20) : h_rnd());
    o->wAae2_ = (uint16_t)(h_rpct(45) ? (h_rnd() & 0xf000u) : h_rpct(40) ? 0xffffu : h_rnd());
    o->rxGain_ = (int16_t)(h_rpct(50) ? 0x34ff + (int)h_rmod(3) - 1 : (int)(int16_t)h_rnd());
    o->w4c8_ = (uint16_t)h_rnd();
    o->w358c_ = (uint16_t)h_rnd();
    { int cnt = h_rpct(80) ? (int)h_rmod(700) : (int)h_rmod(40000) - 20000; o->rxEnd_ = (void *)((uint8_t *)o->rxFskBuf_ + 2 * cnt); }
    for (k = 0; k < 4; k++) { uint16_t z = 0; if (h_rpct(25)) memcpy(((uint8_t *)&o->wa248_) + 0x524 + 0xc + 0x2c * k, &z, 2); }
    if (h_rpct(25)) for (k = 0; k < 4; k++) { uint16_t z = 0; memcpy(((uint8_t *)&o->wa248_) + 0x524 + 0xc + 0x2c * k, &z, 2); }   /* the energy sum of B is zero: no division */
    o->dftActive_ = 0;
  }
  if (sc == G_S) {   /* RECEIVE: flag classes, counter thresholds, the signature steering, the MP bit-serial receiver */
    static const uint16_t bd[] = { 0x960, 0xaf0, 0xbb8, 0xc80, 0xd65, 0xab7 };
    static const uint16_t cr[] = { 0x640, 0x690, 0x708, 0x725, 0x74b, 0x780, 0x7a7, 0x7d0 };
    unsigned t = h_rmod(100), k;
    uint16_t w;
    uint8_t *q2b = o->q2Unk00_;
    o->rxState_ = 4;
    o->mst_ = (uint16_t)(h_rpct(45) ? 0x29 : h_rpct(25) ? 0x2c : h_rmod(87));
    o->w3596_ = (uint16_t)(h_rpct(25) ? 0x12 : h_rmod(87));
    o->modeChar_ = h_rpct(55) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    o->waa96_ = bd[h_rmod(6)];
    o->waaa8_ = (uint16_t)(h_rpct(90) ? cr[h_rmod(8)] : h_rnd());
    o->baud_ = bd[h_rmod(6)]; o->carrier_ = cr[h_rmod(8)]; o->preemp_ = (uint16_t)h_rmod(16);
    o->pcmMode_ = h_rpct(40) ? 0 : (int32_t)h_rnd(); o->k56Mode_ = h_rpct(60) ? 0 : (int32_t)h_rnd();
    o->rxFlags_ = (uint16_t)h_rnd();
    o->rxFlags_ = (uint16_t)(o->rxFlags_ & ~0x40u);
    if (t < 10) o->rxFlags_ = (uint16_t)(o->rxFlags_ | 0x40u);
    else if (t < 18) o->rxFlags_ = (uint16_t)(o->rxFlags_ & ~0x80u);
    else if (t < 26) o->rxFlags_ = (uint16_t)(o->rxFlags_ | 0x98u);
    else if (t < 50) o->rxFlags_ = (uint16_t)((o->rxFlags_ & ~0x88u) | 0x410u | (h_rpct(50) ? 0x80u : 0u));   /* the MP bit-serial receiver (flags 0x10 / 0x90) */
    else if (t < 58) o->rxFlags_ = (uint16_t)((o->rxFlags_ & ~0x498u) | 0x10u | (h_rpct(50) ? 0x80u : 0u));   /* 0x10 without 0x400 */
    else if (t < 72) o->rxFlags_ = (uint16_t)(h_rpct(50) ? ((o->rxFlags_ & ~0x98u) | 8u) : (o->rxFlags_ & ~0x98u));   /* the signature words / the phase 3 end */
    else if (t < 82) o->rxFlags_ = (uint16_t)((o->rxFlags_ & ~0x498u) | 8u);
    else o->rxFlags_ = (uint16_t)(o->rxFlags_ & ~0x498u);
    if (h_rpct(25)) o->rxFlags_ = (uint16_t)(o->rxFlags_ & ~0x400u);
    { unsigned c = h_rmod(100), w96 = (unsigned)(int)(int16_t)o->waa96_;
      w = (uint16_t)(c < 20 ? 7u * w96 + h_rmod(3) - 1u : c < 40 ? 0xbb8u + h_rmod(3) - 1u : c < 50 ? 0x61a7u + h_rmod(3) - 1u : c < 75 ? 0xbb9u + h_rmod(2000) : c < 90 ? h_rmod(0x80) : h_rnd());
      memcpy((uint8_t *)&o->w388_, &w, 2); }
    { unsigned c = h_rmod(100);   /* the signature words */
      uint16_t sig = (uint16_t)(c < 30 ? 0x2640u + h_rmod(4) : c < 50 ? 0x26c0u + h_rmod(4) : h_rnd());
      uint16_t r11c = (uint16_t)(c < 70 ? ((sig & ~3u) | (h_rpct(70) ? 2u : h_rmod(4))) : h_rpct(50) ? 0x267cu + h_rmod(4) : h_rnd());
      uint16_t r126 = (uint16_t)(h_rpct(60) ? (r11c & 3u) : h_rnd());
      o->rxSig_ = sig; memcpy(o->unk0378_ + 8, &r11c, 2); memcpy(o->unk038a_, &r126, 2); }
    fill_queue(o->q2Unk00_); fill_queue(o->mpUnk00_);
    memcpy(&w, q2b + 0x28, 2);
    w = (uint16_t)(h_rpct(75) ? h_rmod(0x24) : h_rpct(50) ? h_rnd() : 0xfffeu + h_rmod(2));
    if (h_rpct(97)) memcpy(q2b + 0x28, &w, 2); else { uint16_t big = (uint16_t)(0x20 + h_rmod(60)); memcpy(q2b + 0x28, &big, 2); }
    { uint16_t a = (uint16_t)(h_rpct(60) ? h_rmod(0x14) : h_rpct(50) ? 0x10 + h_rmod(5) : h_rnd() & 0x7fu); memcpy(q2b + 0x1a, &a, 2); }
    { uint16_t i = (uint16_t)(h_rpct(80) ? h_rmod(12) : h_rmod(3)); memcpy(q2b + 0x1e, &i, 2); }
    if (h_rpct(25)) { uint32_t d = (uint32_t)h_rnd() | 0x10000u; memcpy(q2b + 0x24, &d, 4); }
    for (k = 0; k < 5; k++) { uint16_t ww = (uint16_t)(h_rpct(30) ? (0x8000u | h_rnd()) : h_rnd()); if (h_rpct(30)) memcpy(q2b + 2 * k, &ww, 2); }
    q2b[0] = (uint8_t)(h_rpct(50) ? (q2b[0] | 1u) : (q2b[0] & ~1u));
    if (h_rpct(25)) {   /* the word at the index w1e_ equals the incoming word in its low 15 bits (the repeat counter waa78_ steps): v = (d24 & ~(-1 << n)) | (R126 << n) */
      uint16_t w28w, w1ew, r126w; uint32_t d24w, vv; unsigned n;
      memcpy(&w28w, q2b + 0x28, 2); memcpy(&w1ew, q2b + 0x1e, 2); memcpy(&d24w, q2b + 0x24, 4); memcpy(&r126w, o->unk038a_, 2);
      n = (unsigned)(int)(int16_t)w28w & 31u;
      vv = (d24w & ~(0xffffffffu << n)) | ((unsigned)(int)(int16_t)r126w << n);
      if (w1ew < 12) { uint16_t mw = (uint16_t)(vv ^ (h_rpct(50) ? 0x8000u : 0u)); memcpy(q2b + 2 * w1ew, &mw, 2); }
    }
    o->mpUnk00_[0] = (uint8_t)(h_rpct(50) ? (o->mpUnk00_[0] | 1u) : (o->mpUnk00_[0] & ~1u));
    w = (uint16_t)(h_rpct(35) ? 3 : h_rpct(50) ? 1 : h_rmod(6)); memcpy(o->unk356c_ + 4, &w, 2);
    o->w35a2_ = (uint16_t)(h_rpct(35) ? 0 : h_rpct(50) ? h_rmod(0x80) : h_rpct(50) ? (unsigned)(o->w388_ + h_rmod(3) - 1u) : h_rnd());
    o->waa80_ = (uint16_t)(h_rpct(50) ? 0 : h_rnd());
    o->w3598_ = (uint16_t)(h_rpct(50) ? 0 : 1);
    o->rxGain_ = (int16_t)h_rnd(); o->rxGainEst_ = (int16_t)h_rnd();
    o->resOffs_ = (int16_t)h_rnd();
    o->waa78_ = pick_h16();
    o->dftActive_ = 0;
  }
  if (sc == G_T) {   /* the RX_DPSK head's retrain-detector window: dftActive_ != 0, the DFT records and the detector state words */
    uint8_t *u = (uint8_t *)&o->wa248_, *C = u + 0x5d4;
    unsigned k;
    uint16_t w;
    o->dftActive_ = h_rpct(70) ? 1u : (h_rnd() | 1u);
    o->mst_ = (uint16_t)(h_rpct(45) ? 0x2eu : h_rpct(50) ? 0x2au : h_rpct(50) ? 0x2du : 0x34u);
    o->rxState_ = 0x2b;
    { uint32_t c = h_rpct(60) ? 0x7cu : h_rpct(50) ? (4u * h_rmod(32)) : h_rnd(); memcpy(u + 4, &c, 4); }
    w = (uint16_t)(h_rpct(40) ? 1 : h_rpct(70) ? 2 : h_rpct(50) ? 0 : h_rnd()); memcpy(u + 2, &w, 2);
    w = (uint16_t)(h_rpct(50) ? h_rmod(4) : h_rpct(50) ? 8 + h_rmod(3) : h_rnd()); memcpy(u + 8, &w, 2);
    w = (uint16_t)(h_rpct(50) ? h_rmod(5) : h_rnd()); memcpy(u + 0xa, &w, 2);
    w = (uint16_t)(h_rpct(50) ? 9 : h_rpct(50) ? h_rmod(12) : h_rnd()); memcpy(u + 0xc, &w, 2);
    if (h_rpct(35)) { uint16_t a8; memcpy(&a8, u + 0xc, 2); memcpy(u + 8, &a8, 2); if (h_rpct(60)) { uint16_t m1 = (uint16_t)(a8 - 1u); memcpy(u + 8, &m1, 2); } }
    for (k = 0; k < 3; k++) {
      uint16_t e = (uint16_t)(h_rpct(55) ? h_rmod(0x100) : h_rnd()), l = (uint16_t)(h_rpct(55) ? 0x80 + h_rmod(0x100) : h_rnd());
      if (h_rpct(30)) l = (uint16_t)(e + (h_rpct(50) ? 0 : 1) - (h_rpct(30) ? 1 : 0));
      memcpy(C + 0x2c * k + 0xc, &e, 2); memcpy(C + 0x2c * k + 0x28, &l, 2);
    }
    { uint16_t l = (uint16_t)(h_rpct(50) ? 0x80 + h_rmod(0x100) : h_rnd()), e1; memcpy(&e1, C + 0x2c + 0xc, 2); if (h_rpct(40)) l = (uint16_t)(e1 + (int)h_rmod(3) - 1); memcpy(C + 0x56, &l, 2); }
    if (h_rpct(35)) { uint16_t e0 = 0x10, l0 = 0x20; for (k = 0; k < 3; k++) { memcpy(C + 0x2c * k + 0xc, &e0, 2); memcpy(C + 0x2c * k + 0x28, &l0, 2); } }
    o->w358c_ = (uint16_t)(h_rpct(40) ? h_rmod(3) : h_rnd());
    o->w3588_ = (uint16_t)(h_rpct(60) ? 0 : h_rnd());
  }
  if (sc == G_U) {   /* getbit: a random record, then a run of reads */
    fill_queue(o->i0aUnk00_);
    if (h_rpct(30)) put16(o->i0aUnk00_, 0x28, 0);
    if (h_rpct(40)) put16(o->i0aUnk00_, 0x20, 1 + h_rmod(2));
    if (h_rpct(25)) put16(o->i0aUnk00_, 0x2a, 1 + h_rmod(20));
    { uint16_t w20; memcpy(&w20, o->i0aUnk00_ + 0x20, 2); if (w20) put16(o->i0aUnk00_, 0x18, 1 + h_rmod(0x40)); }   /* an armed record restarts with its length: a non-positive length would recurse forever (in the blob too) */
  }
  if (sc == G_V) {   /* ApplyBulkDelay: the delay around its thresholds, the limit word, the echo flag */
    static const int dl[] = { 0, -5, 1, 0x1d, 0x1e, 0x1f, 0x8f, 0x90, 0x91, 100, 0x7fff, -32768, 12, 0xf, 0x10, 0x11, 0x1c, 2 };
    uint32_t lim = (uint32_t)(h_rpct(30) ? 0x90u + h_rmod(3) - 1u : h_rpct(50) ? h_rmod(300) : h_rmod(9600));
    g_arg = h_rpct(75) ? dl[h_rmod(18)] : (int)(int16_t)h_rnd();
    if (h_rpct(30) && (int16_t)g_arg > 0) lim = (uint32_t)(int16_t)g_arg + h_rmod(3) - 1u;
    if (lim > 9600u) lim = 9600u;
    o->w35b4_ = lim;
    o->pcmMode_ = h_rpct(60) ? 0 : (int32_t)h_rnd(); o->k56Mode_ = h_rpct(60) ? 0 : (int32_t)h_rnd();
    o->echoOn_ = (uint16_t)(h_rpct(50) ? 0 : h_rpct(70) ? 1 : h_rnd());
    o->dmaDelay_ = (uint16_t)h_rnd();
  }
  if (sc == G_K) {   /* MOH_TONE: the symbol limit wabfc_, the request word mohF0_, the role */
    o->wabfc_ = (uint16_t)(h_rpct(60) ? h_rmod(200) : h_rnd());
    o->waa78_ = (uint16_t)(h_rpct(60) ? o->wabfc_ - 3u + h_rmod(5) : h_rpct(50) ? (unsigned)h_rmod(4) : (unsigned)h_rnd());
    o->mohF0_ = h_rpct(45) ? 1u : h_rpct(50) ? 0u : h_rnd();
    o->modeChar_ = h_rpct(45) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    o->w3596_ = (uint16_t)(h_rpct(35) ? 0x18 : h_rmod(87));
    o->mst_ = 0x4f;
    o->rxState_ = 0x2b;
  }
  if (sc == G_L) {   /* TRNSEG4: the symbol counter near the report / segment-end positions, the baud and carrier ladders */
    static const uint16_t bd[] = { 0x960, 0xaf0, 0xab7, 0xbb8, 0xc80, 0xd65 };
    static const uint16_t cr[] = { 0x640, 0x690, 0x708, 0x725, 0x74b, 0x780, 0x7a7, 0x7d0 };
    int mid = h_rpct(70) ? 100 + (int)h_rmod(400) : (int)h_rnd() / 4;
    unsigned t = h_rmod(100);
    o->w3596_ = 0x15;
    o->echoMid_ = mid;
    o->baud_ = (uint16_t)(h_rpct(85) ? bd[h_rmod(6)] : h_rnd());
    o->period_ = (uint16_t)(h_rpct(60) ? 50 + h_rmod(200) : h_rnd());
    if (t < 25) o->txCnt_ = (uint16_t)(mid - 1);
    else if (t < 40) o->txCnt_ = (uint16_t)((mid >> 1) - 1);
    else if (t < 55) { int b = (int16_t)o->baud_; o->txCnt_ = (uint16_t)(b + (b >> 1) - 1); }
    else if (t < 70) o->txCnt_ = (uint16_t)((int)(int16_t)o->period_ - 1 - (int)h_rmod(2));
    else if (t < 80) o->txCnt_ = pick_cnt();
    else o->txCnt_ = (uint16_t)h_rmod(60);
    if (h_rpct(30)) o->waa78_ = (uint16_t)(o->txCnt_ + 1u);
    o->waa96_ = (uint16_t)(h_rpct(88) ? bd[h_rmod(6)] : h_rnd());
    o->waaa8_ = (uint16_t)(h_rpct(88) ? cr[h_rmod(8)] : h_rnd());
    if (h_rpct(20)) o->waa96_ = (uint16_t)(o->waa96_ + (int)h_rmod(5) - 2);   /* neighbours of the ladder constants */
    if (h_rpct(20)) o->waaa8_ = (uint16_t)(o->waaa8_ + (int)h_rmod(5) - 2);
    o->waa7e_ = (uint16_t)(h_rpct(40) ? h_rmod(6) : h_rpct(50) ? 3 + h_rmod(300) : h_rnd());
    o->pcmMode_ = h_rpct(30) ? 0 : h_rpct(30) ? 1 : h_rpct(60) ? (int32_t)(2 + h_rmod(3)) : (int32_t)h_rnd();
    o->k56Mode_ = h_rpct(35) ? 0 : h_rpct(30) ? 1 : h_rpct(60) ? (int32_t)(2 + h_rmod(3)) : (int32_t)h_rnd();
    o->modeChar_ = h_rpct(50) ? 0x65 : h_rpct(70) ? 0x66 : (uint16_t)h_rnd();
    o->rxState_ = (uint16_t)(h_rpct(35) ? 4 : h_rpct(30) ? 0x23 : h_rmod(87));
    o->mst_ = (uint16_t)(h_rpct(30) ? 0x29 : h_rmod(87));
    o->txFlags_ = (uint16_t)(h_rpct(40) ? (h_rnd() | 0x100u) : (h_rnd() & ~0x100u));
  }
  if (sc == G_J) {   /* XMITMP: the MP bit queue (mpQ_, or sometimes the INFO0a record), the receiver flags and the MP pass counters */
    static const uint16_t tg[] = { 0x55, 0x58, 0xbb, 0xbc };
    static const uint16_t c359[] = { 0, 1, 2, 3, 4, 5, 0xffff, 0x7fff };
    unsigned r;
    o->w3596_ = 0x43;
    fill_queue(o->mpUnk00_);
    fill_queue(o->i0aUnk00_);
    o->bitQ_ = h_rpct(78) ? (void *)o->mpUnk00_ : (void *)o->i0aUnk00_;
    o->mpUnk00_[0] = (uint8_t)(h_rpct(50) ? (o->mpUnk00_[0] | 1u) : (o->mpUnk00_[0] & ~1u));
    o->rxSig_ = (uint16_t)(h_rpct(50) ? 0x89b0 : h_rpct(60) ? 0x8990 : h_rnd());
    if (h_rpct(15)) o->rxSig_ = (uint16_t)(0x89b0 + (int)h_rmod(5) - 2);   /* neighbours of the 16-point signature */
    r = h_rmod(100);
    if (r < 55) o->k56Idx_ = (uint16_t)(tg[h_rmod(4)] - 1u - h_rmod(4));
    else if (r < 70) { o->w3590_ = (uint16_t)h_rmod(0x60); o->k56Idx_ = (uint16_t)(o->w3590_ - 1u - h_rmod(4)); }
    else o->k56Idx_ = pick_cnt();
    if (r < 55 || r >= 70) o->w3590_ = (uint16_t)(h_rpct(35) ? (o->k56Idx_ + 1u + h_rmod(4)) : pick_h16());
    o->rxFlags_ = (uint16_t)h_rnd();
    o->rxFlags_ = (uint16_t)(h_rpct(50) ? (o->rxFlags_ | 0x20u) : (o->rxFlags_ & ~0x20u));
    o->rxFlags_ = (uint16_t)(h_rpct(50) ? (o->rxFlags_ | 0x90u) : (h_rpct(50) ? (o->rxFlags_ & ~0x10u) : (o->rxFlags_ & ~0x80u)));
    if (h_rpct(40)) o->rxFlags_ = (uint16_t)((o->rxFlags_ & ~0x18u) | 0x10u);
    o->w359e_ = h_rpct(85) ? c359[h_rmod(8)] : (uint16_t)h_rnd();
    o->w3598_ = (uint16_t)(h_rpct(50) ? 0 : h_rpct(70) ? 1 : h_rnd());
    o->w3596_ = h_rpct(55) ? 0x43 : 0x43;
  }
  if (g_whole) {   /* the real ApplyBulkDelay clears 2 * delay bytes behind obj + 0x35b8 and the limit word at obj + 0x35b4 bounds the delay: keep it inside the 19,200-byte far-echo area */
    uint32_t lim;
    lim = o->w35b4_;
    if (lim > 9600u) { lim = h_rnd() % 9601u; o->w35b4_ = lim; }
  }
  if (g_whole) {   /* an armed record (w20_ != 0) restarts with its length w18_: keep it positive, as the transmitters always do */
    uint8_t *recs[6];
    int k;
    recs[0] = o->i0aUnk00_; recs[1] = o->i0dUnk00_; recs[2] = o->i1aUnk00_; recs[3] = o->i1cUnk00_; recs[4] = o->q2Unk00_; recs[5] = o->mpUnk00_;
    for (k = 0; k < 6; k++) {
      uint16_t w18, w20;
      memcpy(&w18, recs[k] + 0x18, 2); memcpy(&w20, recs[k] + 0x20, 2);
      if (w20 && (int16_t)w18 <= 0) { w18 = (uint16_t)(1 + (h_rnd() % 0x40u)); memcpy(recs[k] + 0x18, &w18, 2); }
    }
  }
  if (state == 0x51 && h_rpct(45)) o->k56Idx_ = 0xbf;
  if (state == 0x12 && h_rpct(30)) o->txCnt_ = 0x3f;
  (void)sc;
}

/* the pointer slots (ctx_, k56_, params_, bitQ_ ...): a pointer into the object is printed as member + delta, else as its token */
static void dump_slots(int show) {
  char buf[1024];
  int i;
  size_t n = 0;
  buf[0] = 0;
  for (i = 0; i < NF_V34Obj; i++) {
    const HFld *fl = &f_V34Obj[i];
    void *v;
    if (fl->kind != 'p' && fl->kind != 'o') continue;
    memcpy(&v, (const uint8_t *)g_o + fl->off, sizeof v);
    if (!strcmp(fl->name, "rxEnd_") && (uintptr_t)v >= (uintptr_t)g_o->rxFskBuf_ - 0x10000u && (uintptr_t)v < (uintptr_t)g_o->rxFskBuf_ + 0x10000u) { n += (size_t)snprintf(buf + n, sizeof buf - n, " %s=bufdelta%ld", fl->name, (long)((const uint8_t *)v - (const uint8_t *)g_o->rxFskBuf_)); continue; }
    if ((const uint8_t *)v >= (const uint8_t *)g_o && (const uint8_t *)v < (const uint8_t *)g_o + sizeof(V34Obj)) n += (size_t)snprintf(buf + n, sizeof buf - n, " %s=%s", fl->name, memb(v));
    else if (tabname(v)) n += (size_t)snprintf(buf + n, sizeof buf - n, " %s=%s", fl->name, tabname(v));
    else n += (size_t)snprintf(buf + n, sizeof buf - n, " %s=%s", fl->name, idn(tok_of(v)));
  }
  h_emit(show, "slots%s", buf);
}
static void dump_world(int show) {
  char snap[TXTMAX * 2];
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(show, "world %s", snap);
  dump_sliced(show, "D", g_o, f_V34Obj, NF_V34Obj);
  dump_slots(show);
}

/* coverage: the outcome of the call as seen from the object before / after */
/* did the tone_detect recorder of the call just made return a non-zero 16-bit value? (the record text ends with `-> <hex>`) */
static int tone_hit(void) {
  unsigned i;
  for (i = 0; i < g_il.n && i < LOGMAX; i++) {
    const char *t = strstr(g_il.r[i].t, "tone_detect(");
    const char *a = t ? strstr(t, " -> ") : NULL;
    if (a) return strtoul(a + 4, NULL, 16) != 0;
  }
  return 0;
}
/* the results of the getbit recorder in the log of the call just made, in order */
static int gb_results(int *res, int max) {
  unsigned i;
  int n = 0;
  for (i = 0; i < g_il.n && i < LOGMAX && n < max; i++) {
    const char *t = strstr(g_il.r[i].t, "getbit(");
    const char *a = t ? strstr(t, " -> ") : NULL;
    if (a) res[n++] = (int)strtol(a + 4, NULL, 10);
  }
  return n;
}
static void cover_dpsk(const V34Obj *pre, unsigned lv) {
  int gb[4] = { 0, 0, 0, 0 }, ngb = gb_results(gb, 4), gi = 0;   /* (zero-initialised: with the real getbit (V04_WHOLE) there are no recorder results) */
  unsigned bits = pre->i0a28_;
  int rem = (int16_t)(uint16_t)(pre->i0a18_ - pre->i0a1a_);
  int eom = 0, next = 0;
  int k56 = (int)(int16_t)(uint16_t)(pre->k56Idx_ + (pre->mohE8_ ? 1u : 0u));
  if (pre->mohE8_) CV(G_I, "dpsk: mohE8 counts"); else CV(G_I, "dpsk: mohE8 clear");
  if (bits) CV(G_I, "bitq: bits in the register");
  else if (rem > 0) {
    int ch = (int16_t)pre->i0a1c_;
    if (ch <= rem) { CV(G_I, "bitq: whole chunk"); if (ch == rem) CV(G_I, "bitq: chunk == remainder"); if (ch < 0) CV(G_I, "bitq: whole chunk, negative count"); if (ch == 0) CV(G_I, "bitq: whole chunk, zero count"); }
    else CV(G_I, "bitq: part of the chunk");
    if (!pre->i0a16_) CV(G_I, "bitq: crc off"); else if (ch <= rem && ch == 0) CV(G_I, "bitq: crc skipped (zero count)"); else CV(G_I, "bitq: crc fed");
  } else if (rem == 0) { if (pre->i0a16_) CV(G_I, "bitq: the 16 crc bits"); else { CV(G_I, "bitq: remainder 0, no crc"); next = 1; } }
  else if ((int16_t)rem == (int16_t)0xfff0) CV(G_I, "bitq: constant 0xf");
  else { CV(G_I, "bitq: other negative remainder"); next = 1; }
  if (next) {
    if (pre->i0a20_ == 0) { CV(G_I, "bitq: end of the message"); eom = 1; }
    else { CV(G_I, "bitq: next message via getbit"); if ((int16_t)gb[gi++] < 0) eom = 1; }
  }
  if (!eom) { CV(G_I, "dpsk: bit sent"); return; }
  if (!pre->mohE8_) { CV(G_I, "eom: mohE8 clear"); if (lv > 1u) CV(G_I, "eom: TONE_AB printed"); else CV(G_I, "eom: TONE_AB quiet"); return; }
  if (lv > 1u) CV(G_I, "eom: msg print"); else CV(G_I, "eom: msg quiet");
  if (!pre->mohF8_) {
    int bit2 = 0, remr = (int16_t)pre->i0a18_;
    CV(G_I, "eom: restart");
    if (pre->i0a2a_ != 0) CV(G_I, "restart: stored count"); 
    else if (remr > 0) { int ch = (int16_t)pre->i0a1c_; if (ch > remr) CV(G_I, "restart: part chunk"); else CV(G_I, "restart: whole chunk"); }
    else if (remr == 0) { if (pre->i0a16_) CV(G_I, "restart: 16 crc bits"); else bit2 = 1; }
    else if ((int16_t)remr == (int16_t)0xfff0) CV(G_I, "restart: constant 0xf");
    else bit2 = 1;
    if (bit2) { if (pre->i0a20_ == 0) CV(G_I, "restart: end of the message, bit -1"); else { CV(G_I, "restart: next message via getbit"); (void)ngb; } }
    goto timeouts;
  }
  CV(G_I, "eom: message ended");
  if (pre->mohF0_ == 1) CV(G_I, "end: F0 == 1 -> MOH_SILENCE");
  else if (pre->mohF0_ == 0) {
    if (pre->mohF4_ == 4 || pre->mohF4_ == 0) CV(G_I, "end: F0 == 0, F4 4 or 0 -> MOH_SILENCE");
    else if (pre->mohF4_ == 5) {
      if (pre->mohE2_ != 3) CV(G_I, "end: reply, mohE2 set"); else CV(G_I, "end: reply, mohE2 already 3");
      if (pre->mohF9_) { if (lv > 1u) CV(G_I, "end: MHcda printed"); else CV(G_I, "end: MHcda quiet"); } else { if (lv > 1u) CV(G_I, "end: MHfrr printed"); else CV(G_I, "end: MHfrr quiet"); }
    } else { if (lv > 1u) CV(G_I, "end: illegal MH printed"); else CV(G_I, "end: illegal MH quiet"); }
  }
  else if (pre->mohF0_ > 3) CV(G_I, "end: F0 > 3");
  else { CV(G_I, "end: F0 2 or 3 -> disconnect"); CV(G_I, "end: disconnect, tx changed"); if (pre->rxState_ == 0x23) CV(G_I, "end: disconnect, rx already WAIT"); else CV(G_I, "end: disconnect, rx changed"); }
  if (pre->mohF0_ == 1 || (pre->mohF0_ == 0 && (pre->mohF4_ == 4 || pre->mohF4_ == 0))) { k56 = 0; if (pre->rxState_ == 0x23) CV(G_I, "end: MOH_SILENCE, rx already WAIT"); }
timeouts:
  {
    int thr = ((int)(int16_t)pre->waa7e_ >> 4) + 0x4b0;
    if (k56 < thr) { CV(G_I, "timeouts: below the limit"); return; }
    if (pre->mohF9_) CV(G_I, "timeouts: MHreq disconnect");
    else if ((pre->mohF0_ - 2u) > 1u) CV(G_I, "timeouts: MHreq retrain");
    else CV(G_I, "timeouts: cleardown");
  }
}
static void cover_xmitmp(const V34Obj *pre, unsigned lv) {
  int n = 0;
  unsigned k56 = pre->k56Idx_, w3590 = pre->w3590_, w359e = pre->w359e_, w3598 = pre->w3598_;
  unsigned rf = pre->rxFlags_;
  int wide = pre->rxSig_ == 0x89b0;
  int mpbit = pre->mpUnk00_[0] & 1;
  int qismp = pre->bitQ_ == (void *)g_o->mpUnk00_;
  int qbit0 = qismp ? mpbit : (pre->i0aUnk00_[0] & 1);
  for (;;) {
    int k;
    if (wide ? !(n <= 3) : (n > 1)) break;
    k56 = (uint16_t)(k56 + 1u);
    k = (int16_t)k56;
    if ((rf & 0x20u) ? k == 0x55 : k == 0xbb) { if (rf & 0x20u) CV(G_J, "xm: stretch by 3"); else CV(G_J, "xm: stretch by 1"); }
    else if ((rf & 0x20u) ? k == 0x58 : k == 0xbc) {
      unsigned old = w359e, rfl = rf;
      int rearm = 0, detected = 0;
      w359e = (uint16_t)(old + 1u);
      if (!mpbit) { if ((rfl & 0x18u) != 0x10u) { CV(G_J, "xm: pass end, no MP, re-arm"); rearm = 1; } else { CV(G_J, "xm: pass end, no MP, 0x18 == 0x10"); detected = 1; } }
      else if ((rfl & 0x90u) == 0x90u && (int16_t)(old + 1u) > 3) {
        if (w3598 == 0) { CV(G_J, "xm: EXMIT, initdigital"); w3598 = 1; } else CV(G_J, "xm: EXMIT, initdigital already done");
        if (lv > 1u) CV(G_J, "xm: EXMIT, tx change printed"); else CV(G_J, "xm: EXMIT, tx change quiet");
        if (pre->rxSig_ == 0x89b0) CV(G_J, "xm: EXMIT, 16-point symbol"); else CV(G_J, "xm: EXMIT, 4-point symbol");
        return;
      }
      else if ((rfl & 0x18u) != 0x10u) { CV(G_J, "xm: pass end, MP seen, re-arm"); rearm = 1; }
      else { CV(G_J, "xm: pass end, MP seen, 0x18 == 0x10"); detected = 1; }
      if (detected) {
        if ((int16_t)w359e <= 1) { CV(G_J, "xm: detected, count <= 1: restart"); rearm = 1; }
        else {
          if (!qbit0) { CV(G_J, "xm: detected, queue bit 0 clear: count cleared"); w359e = 0; } else CV(G_J, "xm: detected, queue bit 0 set");
          qbit0 = 1; if (qismp) mpbit = 1;
          if (lv > 1u) CV(G_J, "xm: MP detected print"); else CV(G_J, "xm: MP detected quiet");
          rearm = 1;
        }
      }
      if (rearm) { if (lv > 1u) CV(G_J, "xm: restart printed"); else CV(G_J, "xm: restart quiet"); k56 = 0; w3590 = 0x22; }
    }
    else if ((uint16_t)k56 == w3590) { CV(G_J, "xm: w3590 stretch"); w3590 = (uint16_t)(w3590 + 0x11u); }
    else CV(G_J, "xm: plain bit");
    n = (int16_t)(n + 1);
  }
  if (wide) CV(G_J, "xm: 16-point symbol sent"); else CV(G_J, "xm: 4-point symbol sent");
}
static void cover_moh_tone(const V34Obj *pre, unsigned lv) {
  if (!tone_hit()) { CV(G_K, "mt: no tone hit"); return; }
  if ((int16_t)(uint16_t)(pre->waa78_ + 1u) < (int16_t)pre->wabfc_) { CV(G_K, "mt: below the symbol limit"); return; }
  if (pre->w3596_ == 0x18) CV(G_K, "mt: tx already TX_DPSK"); else if (lv > 1u) CV(G_K, "mt: tx change printed"); else CV(G_K, "mt: tx change quiet");
  if (pre->mohF0_ == 1) {
    if (pre->modeChar_ == 0x65) CV(G_K, "mt: retrain request, role 0x65 (c2400_)"); else CV(G_K, "mt: retrain request, other role (c1200_)");
    if (lv > 1u) CV(G_K, "mt: mst -> 0x50 printed"); else CV(G_K, "mt: mst -> 0x50 quiet");   /* the microstate is MOH_TONE here: the "already" guards are context constants */
  } else {
    if (lv > 1u) CV(G_K, "mt: mst -> 0x29 printed"); else CV(G_K, "mt: mst -> 0x29 quiet");
  }
}
static void cover_trnseg4(const V34Obj *pre, unsigned lv) {
  unsigned cnt = (uint16_t)(pre->txCnt_ + 1u);
  int c = (int16_t)cnt, mid = pre->echoMid_, b = (int16_t)pre->baud_;
  if (pre->waa78_ == (uint16_t)cnt) CV(G_L, "t4: echo adaptation start");
  if (pre->txFlags_ & 0x100u) CV(G_L, "t4: flag 0x100 already set");
  else if (c >= (int16_t)pre->period_) CV(G_L, "t4: txCnt reaches period, flag set");
  else CV(G_L, "t4: txCnt below period");
  if (c == (mid >> 1)) { CV(G_L, "t4: middle of the echo adaptation"); return; }
  if (c == b + (b >> 1)) { CV(G_L, "t4: 1.5 * baud, flag 0x400"); return; }
  if (c != mid) { CV(G_L, "t4: plain symbol"); return; }
  if (lv > 1u) CV(G_L, "t4: end, tx -> JTXMIT printed"); else CV(G_L, "t4: end, tx -> JTXMIT quiet");
  if ((int16_t)pre->waa7e_ > 2) {
    if (pre->pcmMode_ != 0) CV(G_L, "t4: delay = waa7e_ (pcm)"); else if (pre->k56Mode_ != 0) CV(G_L, "t4: delay = waa7e_ (k56)"); else CV(G_L, "t4: delay = waa7e_ / 2");
    if (lv > 1u) CV(G_L, "t4: bulk delay print"); else CV(G_L, "t4: bulk delay quiet");
  } else {
    if (lv > 1u) CV(G_L, "t4: freeze EC printed"); else CV(G_L, "t4: freeze EC quiet");
  }
  if (pre->rxState_ == 4) CV(G_L, "t4: rx already RECEIVE"); else if (lv > 1u) CV(G_L, "t4: rx -> RECEIVE printed"); else CV(G_L, "t4: rx -> RECEIVE quiet");
  switch (pre->waa96_) {
    case 0xaf0: CV(G_L, "t4: baud 2800"); break;
    case 0x960: CV(G_L, "t4: baud 2400"); break;
    case 0xab7: CV(G_L, "t4: baud 2743"); break;
    case 0xc80: CV(G_L, "t4: baud 3200"); break;
    case 0xbb8: CV(G_L, "t4: baud 3000"); break;
    case 0xd65: CV(G_L, "t4: baud 3429"); break;
    default: CV(G_L, "t4: other baud"); break;
  }
  switch (pre->waaa8_) {
    case 0x725: CV(G_L, "t4: carrier 1829"); break;
    case 0x780: CV(G_L, "t4: carrier 1920"); break;
    case 0x74b: CV(G_L, "t4: carrier 1867"); break;
    case 0x7a7: CV(G_L, "t4: carrier 1959"); break;
    case 0x7d0: CV(G_L, "t4: carrier 2000"); break;
    case 0x690: CV(G_L, "t4: carrier 1680"); break;
    case 0x708: CV(G_L, "t4: carrier 1800"); break;
    case 0x640: CV(G_L, "t4: carrier 1600"); break;
    default: CV(G_L, "t4: other carrier"); break;
  }
  if (lv > 1u) CV(G_L, "t4: demodulator / gain prints"); else CV(G_L, "t4: receiver quiet");
  if (pre->mst_ == 0x29) CV(G_L, "t4: mst already 0x29"); else if (lv > 1u) CV(G_L, "t4: mst -> 0x29 printed"); else CV(G_L, "t4: mst -> 0x29 quiet");
  if ((uint32_t)pre->pcmMode_ > 1u) CV(G_L, "t4: pcm -> JaTXMIT");
  else if ((uint32_t)pre->k56Mode_ > 1u) CV(G_L, "t4: k56 -> K56JaTXMIT");
  else CV(G_L, "t4: plain V.34");
}
static void cover_mm(const V34Obj *pre, unsigned lv) {
  int fd = (int16_t)pre->filtDelay_;
  unsigned a = (uint16_t)(pre->waa78_ + 1u);
  int c = (int16_t)a;
  switch (pre->mst_) {
    case 0x31: {
      int rec = 0;
      if (c > 0x28 && c < fd + 0x4c && (pre->wAae2_ & 0x3ffu) == 0x372u) { if (pre->w3588_ == 0) { CV(G_M, "ra1: recovery"); rec = 1; if (pre->w3596_ == 0x18) CV(G_M, "ra1: recovery, tx already 0x18"); else if (lv > 1u) CV(G_M, "ra1: recovery, tx change printed"); else CV(G_M, "ra1: recovery, tx change quiet"); } else CV(G_M, "ra1: recovery gate w3588"); }
      (void)rec;
      if (c <= 0x28) CV(G_M, "ra1: count <= 0x28");
      if ((int16_t)(rec ? pre->waa78_ : (uint16_t)a) <= fd + 0x50 && !rec) { CV(G_M, "ra1: below the limit"); return; }
      if (!(pre->wAae2_ & 1u) && !rec) { CV(G_M, "ra1: INFO0 bit clear"); return; }
      if (rec) return;
      if (pre->shortPhase2_ != 0) { if (lv > 1u) CV(G_M, "ra1: short phase 2, prev session delay printed"); else CV(G_M, "ra1: short phase 2, quiet"); } else CV(G_M, "ra1: delay from the count");
      if (lv > 1u) CV(G_M, "ra1: bulk print"); else CV(G_M, "ra1: bulk quiet");
      CV(G_M, "ra1: bulk delay applied");
      if (lv > 1u) CV(G_M, "ra1: mst printed"); else CV(G_M, "ra1: mst quiet");   /* the microstate is the dispatched one: the "already" guards are context constants */
      break;
    }
    case 0x32: {
      int rec = 0;
      if (c > 0x32 && c < fd + 0x4c && (pre->wAae2_ & 0x3ffu) > 0x200u) { if (pre->w3588_ == 0) { CV(G_M, "ra2: recovery"); rec = 1; } else CV(G_M, "ra2: recovery gate w3588"); }
      if (rec) return;
      if (c <= fd + 0x50) { CV(G_M, "ra2: below the limit"); return; }
      if (!(pre->wAae2_ & 1u)) { CV(G_M, "ra2: INFO0 bit clear"); return; }
      if (lv > 1u) CV(G_M, "ra2: set-up printed"); else CV(G_M, "ra2: set-up quiet");
      if (lv > 1u) CV(G_M, "ra2: rx change printed"); else CV(G_M, "ra2: rx change quiet");
      break;
    }
    case 0x37: {
      /* the first recovery depends on the head's stale wAae0_ copy (the recorder pokes the word): only its gates are driven */
      if ((pre->wAae2_ & 7u) == 7u) CV(G_M, "tc1: INFO0 flags 7");
      if (c <= 0x5f) CV(G_M, "tc1: counting"); else CV(G_M, "tc1: mst change");
      if (pre->w3588_ == 2 && pre->w358a_ == 2) { CV(G_M, "tc1: double marked"); if ((pre->wAae2_ & 0x3ffu) == 0x372u) CV(G_M, "tc1: second recovery"); }
      break;
    }
    case 0x33:
      if ((uint16_t)a != 0x2a) { CV(G_M, "txl1m: count not at 0x2a"); return; }
      if (pre->shortPhase2_ == 0) {
        CV(G_M, "txl1m: normal case");
        if (pre->w3596_ == 0x33) CV(G_M, "txl1m: tx already TX_L1"); else if (lv > 1u) CV(G_M, "txl1m: tx change printed"); else CV(G_M, "txl1m: tx change quiet");
        if (pre->modeChar_ == 0x65) CV(G_M, "txl1m: role 0x65 (c2400_)"); else CV(G_M, "txl1m: other role (c1200_)");
        if (lv > 1u) CV(G_M, "txl1m: rx change printed"); else CV(G_M, "txl1m: rx change quiet");
      } else {
        CV(G_M, "txl1m: short phase 2");
        switch (pre->baud_) {
          case 0xaf0: CV(G_M, "txl1m: baud 2800"); break;
          case 0x960: CV(G_M, "txl1m: baud 2400"); break;
          case 0xab7: if (lv > 1u) CV(G_M, "txl1m: baud 2743 printed"); else CV(G_M, "txl1m: baud 2743 quiet"); if (pre->carrier_ == 0x725) CV(G_M, "txl1m: 2743, carrier 1829"); else CV(G_M, "txl1m: 2743, other carrier"); break;
          case 0xc80: CV(G_M, "txl1m: baud 3200"); break;
          case 0xbb8: CV(G_M, "txl1m: baud 3000"); break;
          case 0xd65: CV(G_M, "txl1m: baud 3429"); break;
          default: CV(G_M, "txl1m: other baud"); break;
        }
        if (lv > 1u) CV(G_M, "txl1m: probe print"); else CV(G_M, "txl1m: probe quiet");
      }
      { int g = (int16_t)pre->rxGain_; if (g > 0x2000) CV(G_M, "txl1m: gain / 4"); else if (g > 0x1000) CV(G_M, "txl1m: gain / 2"); else CV(G_M, "txl1m: gain kept"); }
      if (lv > 1u) CV(G_M, "txl1m: retrain print"); else CV(G_M, "txl1m: retrain quiet");
      break;
    case 0x2e: {
      int tx = pre->w3596_;
      if (tx == 0x18 && pre->i0a20_ != 0 && pre->i0a22_ != 0) { CV(G_M, "pa1: queue closed"); tx = (int)pre->w3596_; }
      if (tx == 0x3c) {
        if (c <= 0x18f) { CV(G_M, "pa1: TONE_AB counting"); }
        else if (pre->wAae2_ != 0) CV(G_M, "pa1: TONE_AB, INFO0 present");
        else if (pre->b_ac00_ != 0) { CV(G_M, "pa1: retrain INFO0"); if (pre->modeChar_ == 0x66) CV(G_M, "pa1: retrain, role 0x66 INFO0d"); if (lv > 1u) CV(G_M, "pa1: retrain print"); else CV(G_M, "pa1: retrain quiet"); return; }
        else { CV(G_M, "pa1: TONE_AB end"); if (lv > 1u) CV(G_M, "pa1: TONE_AB end, mst printed"); else CV(G_M, "pa1: TONE_AB end, mst quiet"); return; }
      }
      if (pre->w3588_ == 0) {
        if (c > 0xc7 && (pre->wAae2_ & 0x3ffu) == 0x372u) { CV(G_M, "pa1: R1"); if (pre->w3596_ == 0x18) CV(G_M, "pa1: R1, tx already 0x18"); else if (lv > 1u) CV(G_M, "pa1: R1, tx printed"); else CV(G_M, "pa1: R1, tx quiet"); return; }
        if (c <= 0x4b0) CV(G_M, "pa1: below the 0x4b0 gate"); else if (pre->wAae2_ == 0) CV(G_M, "pa1: no INFO0 yet"); else { CV(G_M, "pa1: R2"); if (lv > 1u) CV(G_M, "pa1: R2 printed"); else CV(G_M, "pa1: R2 quiet"); }
      } else if (pre->w3588_ == 2) {
        if (c <= 0xc7) CV(G_M, "pa1: R3 gate, count low");
        else if ((pre->wAae2_ & 0xfffu) != 0xf72u) CV(G_M, "pa1: R3 gate, word mismatch");
        else if ((int16_t)(uint16_t)(pre->wAa7a_ + 1u) <= 0xc) CV(G_M, "pa1: R3 counting");
        else { CV(G_M, "pa1: R3"); if (lv > 1u) CV(G_M, "pa1: R3 printed"); else CV(G_M, "pa1: R3 quiet"); }
      } else CV(G_M, "pa1: w3588 other");
      break;
    }
    case 0x3b: {
      int cx = c;
      if ((pre->wAae2_ & 7u) == 7u) { CV(G_M, "pc2: first recovery"); cx = 1; if (pre->w3596_ == 0x18) CV(G_M, "pc2: recovery, tx already 0x18"); }
      if ((uint16_t)cx == 0x2a) { if (pre->w3596_ == 5) CV(G_M, "pc2: 0x2a, tx already SILENCE"); else if (lv > 1u) CV(G_M, "pc2: 0x2a, tx change printed"); else CV(G_M, "pc2: 0x2a, tx change quiet"); }
      if (!((uint16_t)cx == 0x2a && pre->w3596_ == 5) && cx > 0x125f) {
        if (pre->w402_ != 0) { if (tone_hit()) { CV(G_M, "pc2: retrain initiated"); return; } CV(G_M, "pc2: no tone, flag set"); } else CV(G_M, "pc2: flag first set");
      }
      if (cx > fd + 0x5c) {
        if ((uint16_t)pre->wAae2_ == 8 || (uint16_t)pre->wAae2_ == 0x18) {
          int e = (int16_t)(cx - fd - 0x63) * 4;
          if ((int16_t)e <= 0) CV(G_M, "pc2: bulk delay clamped to 1"); else CV(G_M, "pc2: bulk delay from the count");
          if (lv > 1u) CV(G_M, "pc2: set-up printed"); else CV(G_M, "pc2: set-up quiet");
          if (lv > 1u) CV(G_M, "pc2: rx change printed"); else CV(G_M, "pc2: rx change quiet");
          if (lv > 1u) CV(G_M, "pc2: gain print"); else CV(G_M, "pc2: gain quiet");
        } else CV(G_M, "pc2: INFO0 word not 8 / 0x18");
      }
      if (pre->w3588_ == 2 && pre->w358a_ == 2 && (pre->wAae2_ & 0x3ffu) == 0x372u) CV(G_M, "pc2: second recovery");
      break;
    }
    case 0x3a:
      if (pre->w3596_ == 0x18) { if (pre->i0a20_ != 0 && pre->i0a22_ != 0) CV(G_M, "rc1: queue closed"); else CV(G_M, "rc1: queue left"); }
      if (c > 0x5f && pre->wAae2_ == 1) { CV(G_M, "rc1: phase complete"); if (lv > 1u) CV(G_M, "rc1: mst change printed"); else CV(G_M, "rc1: mst change quiet"); if (lv > 1u) CV(G_M, "rc1: received print"); return; }
      if ((int16_t)(uint16_t)a > 0x3bf && (pre->w3588_ & 2u) && pre->w358a_ == 2 && (pre->wAae2_ & 0x3ffu) == 0x372u) { CV(G_M, "rc1: recovery"); if ((uint16_t)pre->w3588_ == 2) CV(G_M, "rc1: recovery with v34handshakinit"); return; }
      if (c > 0x4b0 && pre->w3588_ == 0 && pre->wAae2_ != 0) CV(G_M, "rc1: silent recovery");
      else CV(G_M, "rc1: nothing");
      break;
    default: break;
  }
}
static void cover_detab(const V34Obj *pre, unsigned lv) {
  if (pre->mst_ != 0x34) { CV(G_N, "ab: mst not 0x34"); return; }
  if ((int16_t)pre->k56Idx_ <= 0x5db) { CV(G_N, "ab: count below 0x5dc"); return; }
  if (!tone_hit()) {
    if ((int)(int16_t)pre->k56Idx_ > (int)(int16_t)pre->waa7e_ + 0x2418) { if (lv > 1u) CV(G_N, "ab: no tone, retrain printed"); else CV(G_N, "ab: no tone, retrain quiet"); }
    else CV(G_N, "ab: no tone, waiting");
    return;
  }
  if (lv > 1u) CV(G_N, "ab: end of L2 printed"); else CV(G_N, "ab: end of L2 quiet");
  if (pre->modeChar_ == 0x65) {
    CV(G_N, "ab: role 0x65");
    if (pre->w3596_ == 0x18) CV(G_N, "ab: 0x65, tx already TX_DPSK"); else if (lv > 1u) CV(G_N, "ab: 0x65, tx printed"); else CV(G_N, "ab: 0x65, tx quiet");
    if (lv > 1u) CV(G_N, "ab: 0x65, prints"); else CV(G_N, "ab: 0x65, quiet");
  } else {
    CV(G_N, "ab: other role");
    if (pre->w3596_ == 0x3c) CV(G_N, "ab: tx already TONE_AB"); else if (lv > 1u) CV(G_N, "ab: tx -> TONE_AB printed"); else CV(G_N, "ab: tx -> TONE_AB quiet");
    if (lv > 1u) CV(G_N, "ab: mst -> 0x30 printed"); else CV(G_N, "ab: mst -> 0x30 quiet");
    if (lv > 1u) CV(G_N, "ab: rx -> RX_DPSK printed"); else CV(G_N, "ab: rx -> RX_DPSK quiet");
  }
}
static void cover_dsync(const V34Obj *pre, unsigned lv) {
  unsigned bp = pre->wAae2_;
  int w7e = (int)(int16_t)pre->waa7e_ >> 4, di = (int16_t)pre->wAae0_;
  uint16_t w3a0 = pre->w35a0_;
  if ((uint8_t)bp == 0x72) {
    CV(G_O, "ds: wAae2 low byte 0x72");
    if (pre->w358a_ == 0) CV(G_O, "ds: 0x72, w358a clear: rxFlags bit cleared"); else CV(G_O, "ds: 0x72, w358a set");
    if (lv > 1u) CV(G_O, "ds: 0x72 printed"); else CV(G_O, "ds: 0x72 quiet");
    return;
  }
  if (pre->mohE8_ == 0 && pre->w358a_ == 0) {
    if (di <= 0x64) { CV(G_O, "ds: idle (wAae0 low)"); return; }
    if (!tone_hit() && di <= 0x384) { CV(G_O, "ds: start, no tone, below 0x385"); return; }
    if (tone_hit()) CV(G_O, "ds: start via tone"); else CV(G_O, "ds: start via time-out");
    if (pre->modeChar_ == 0x65 && pre->pcmMode_ != 0) CV(G_O, "ds: start, role 0x65 pcm (info0a 0x1e)"); else CV(G_O, "ds: start, info0a 0x11");
    if (pre->w3596_ == 0x18) CV(G_O, "ds: start, tx already TX_DPSK"); else if (lv > 1u) CV(G_O, "ds: start, tx printed"); else CV(G_O, "ds: start, tx quiet");
    if (lv > 1u) CV(G_O, "ds: start, errorrecovery print"); else CV(G_O, "ds: start, quiet");
    return;
  }
  if (pre->w358a_ == 1) {
    if (!tone_hit()) { CV(G_O, "ds: s1 no tone"); goto moh; }
    if (!(pre->bitQ_ && ((const uint8_t *)pre->bitQ_)[4] & 0x80u)) { CV(G_O, "ds: s1 tone, queue bit 7 clear"); goto moh; }
    CV(G_O, "ds: s1 tone and queue bit 7");
    if ((int16_t)pre->wAbc2_ >= 0) CV(G_O, "ds: s1 copy loop"); else CV(G_O, "ds: s1 no copy");
    if (pre->modeChar_ == 0x65) {
      CV(G_O, "ds: s1 role 0x65");
      if (lv > 1u) CV(G_O, "ds: s1 0x65 mst printed + RX_PHASE1_CALL print"); else CV(G_O, "ds: s1 0x65 quiet");
    } else {
      CV(G_O, "ds: s1 other role");
      if (lv > 1u) CV(G_O, "ds: s1 other mst printed + TX_PHASE1_ANS print"); else CV(G_O, "ds: s1 other quiet");
    }
    if (lv > 1u) CV(G_O, "ds: s1 info0 prints"); else CV(G_O, "ds: s1 info0 quiet");
    return;
  }
  if (pre->w358a_ == 2) {
    unsigned step;
    uint16_t neww;
    int cleared = !(di > w7e + 0x190);
    if (di <= w7e + 0x64) { CV(G_O, "ds: s2 below the first window"); goto moh; }
    if (cleared) CV(G_O, "ds: s2 inside, w35a0 cleared"); else CV(G_O, "ds: s2 past the second window");
    step = (bp == 0 || bp == 0xffffu) ? (unsigned)(cleared ? 0u : w3a0) + 1u : 0u;
    if (bp == 0 || bp == 0xffffu) CV(G_O, "ds: s2 counting window"); else CV(G_O, "ds: s2 other wAae2");
    neww = pre->w3596_ == 0x3c ? 0 : (uint16_t)step;
    if (pre->w3596_ == 0x3c) CV(G_O, "ds: s2 tx TONE_AB clears the count");
    if (di < w7e + 0x190) { CV(G_O, "ds: s2 before w7e + 0x190"); return; }
    if ((int16_t)neww > 0x31 && (bp == 0xffffu || bp == 0)) {
      if (bp == 0xffffu) {
        CV(G_O, "ds: s2 info1a restart");
        if (pre->w3596_ == 0x18) CV(G_O, "ds: s2 restart, tx already TX_DPSK"); else if (lv > 1u) CV(G_O, "ds: s2 restart, tx printed"); else CV(G_O, "ds: s2 restart, tx quiet");
        if (lv > 1u) CV(G_O, "ds: s2 restart, mst + INFOMARKS print"); else CV(G_O, "ds: s2 restart, quiet");
      } else {
        CV(G_O, "ds: s2 info0 repeat");
        if (pre->w3596_ == 0x4a) CV(G_O, "ds: s2 repeat, tx already SILENCERETRAIN"); else if (lv > 1u) CV(G_O, "ds: s2 repeat, tx printed"); else CV(G_O, "ds: s2 repeat, tx quiet");
        if (lv > 1u) CV(G_O, "ds: s2 repeat, rx printed"); else CV(G_O, "ds: s2 repeat, rx quiet");   /* rx is RX_DPSK (0x2b) at dispatch: never WAIT */
        if (lv > 1u) CV(G_O, "ds: s2 repeat, info1 print"); else CV(G_O, "ds: s2 repeat, quiet");
      }
      return;
    }
    if (di < w7e + 0x1ea) { CV(G_O, "ds: s2 below 0x1ea"); return; }
    if (pre->modeChar_ != 0x65) {
      uint16_t w;
      memcpy(&w, pre->unka24a_, 2);
      if (w != 1) CV(G_O, "ds: s2 other role, no retrain");
      else { CV(G_O, "ds: s2 other role, retrain"); if (lv > 1u) CV(G_O, "ds: s2 retrain printed"); else CV(G_O, "ds: s2 retrain quiet"); }
      return;
    }
    CV(G_O, "ds: s2 role 0x65 recovery");
    if (lv > 1u) CV(G_O, "ds: s2 0x65 prints"); else CV(G_O, "ds: s2 0x65 quiet");
    if (pre->w3596_ == 0x3c) CV(G_O, "ds: s2 0x65, tx already TONE_AB");
    if (pre->rxState_ == 0x2b) CV(G_O, "ds: s2 0x65, rx already RX_DPSK");
    return;
  }
  CV(G_O, "ds: other w358a");
moh:
  if (pre->mohE8_ == 0) { CV(G_O, "ds: moh tail, mohE8 clear"); return; }
  if (pre->mohF0_ != 1) { CV(G_O, "ds: moh tail, mohF0 != 1"); return; }
  if ((int16_t)pre->w35a0_ <= 0x31) { CV(G_O, "ds: moh tail, w35a0 low"); return; }
  if (pre->wAae2_ != 0) { CV(G_O, "ds: moh tail, wAae2 nonzero"); return; }
  if (pre->mohF8_ != 0) { CV(G_O, "ds: moh tail, mohF8 set"); return; }
  if (lv > 1u) CV(G_O, "ds: moh tail armed printed"); else CV(G_O, "ds: moh tail armed quiet");
}
static void cover_dinfo(const V34Obj *pre, unsigned lv) {
  const uint8_t *q = (const uint8_t *)pre->bitQ2_;
  uint16_t w14, w18;
  unsigned bit = pre->wAae2_ & 1u;
  int n, w18s;
  int endbit;
  if (pre->wAae0_ == 0) { CV(G_P, "di: wAae0 zero, nothing"); return; }
  memcpy(&w14, q + 0x14, 2); memcpy(&w18, q + 0x18, 2);
  w18s = (int16_t)w18;
  if ((int16_t)pre->waa78_ < w18s) {
    unsigned msb = (w14 >> 15) & 1u;
    CV(G_P, "di: data bit, crc update");
    if ((msb ^ bit) != 0) CV(G_P, "di: crc feedback"); else CV(G_P, "di: crc no feedback");
    if (msb) CV(G_P, "di: crc msb set");
    if (bit) CV(G_P, "di: crc bit set");
  } else CV(G_P, "di: past the data bits");
  n = (int16_t)(uint16_t)(pre->waa78_ + 1u);
  endbit = (n == w18s + 0x10);
  if (!endbit) {
    CV(G_P, "di: not the end");
    if ((n & 7) != 0) { CV(G_P, "di: tail, not a byte boundary"); return; }
    if (n <= 0) { CV(G_P, "di: tail, count not positive"); return; }
    if (((n >> 3) - 1) > 9) { CV(G_P, "di: tail, word index above 9"); return; }
    CV(G_P, "di: tail, byte stored"); return;
  }
  {
    uint16_t crc = w14;
    if ((int16_t)pre->waa78_ < w18s) { unsigned msb = (w14 >> 15) & 1u; crc = (uint16_t)(w14 * 2u); if ((msb ^ bit) != 0) crc ^= 0x1021u; }
    if (pre->wAae2_ != crc) {
      CV(G_P, "di: end, crc error");
      if (pre->modeChar_ == 0x66) CV(G_P, "di: crc error, role 0x66"); else CV(G_P, "di: crc error, other role");
      if (lv > 1u) CV(G_P, "di: crc error printed"); else CV(G_P, "di: crc error quiet");
      if (pre->modeChar_ == 0x65) { if (pre->pcmMode_ != 0) CV(G_P, "di: rearm, role 0x65 pcm (0x1e)"); else CV(G_P, "di: rearm, role 0x65 no pcm"); } else CV(G_P, "di: rearm, other role (0x11)");
      return;
    }
  }
  CV(G_P, "di: end, crc good");
  if (w18 == 0x4d) {
    CV(G_P, "di: INFO1a");
    if (pre->w3596_ == 0x18) CV(G_P, "di: INFO1a, tx already TX_DPSK"); else if (lv > 1u) CV(G_P, "di: INFO1a, tx printed"); else CV(G_P, "di: INFO1a, tx quiet");
    if (lv > 1u) CV(G_P, "di: INFO1a prints"); else CV(G_P, "di: INFO1a quiet");
    return;
  }
  if (w18 == 0x26) {
    CV(G_P, "di: INFO1c");
    if (lv > 1u) CV(G_P, "di: INFO1c prints"); else CV(G_P, "di: INFO1c quiet");
    return;
  }
  if (w18 == 8) {
    CV(G_P, "di: MOH message");
    if (lv > 1u) CV(G_P, "di: MOH print"); else CV(G_P, "di: MOH quiet");
    if (pre->mohF0_ == 1) { if (pre->modeChar_ == 0x65) CV(G_P, "di: MOH tone request, role 0x65"); else CV(G_P, "di: MOH tone request, other role"); }
    else CV(G_P, "di: MOH other request");
    return;
  }
  CV(G_P, "di: INFO0 message");
  if (pre->w358a_ == 1) {
    if (q[4] & 0x80u) CV(G_P, "di: INFO0, queue bit 7 set"); else CV(G_P, "di: INFO0, queue bit 7 clear (back to DET_SYNC)");
  } else CV(G_P, "di: INFO0, w358a not 1");
  if (pre->modeChar_ == 0x65) CV(G_P, "di: INFO0 done, role 0x65");
  else if (pre->shortPhase2_ == 0) CV(G_P, "di: INFO0 done, other role, long phase 2");
  else CV(G_P, "di: INFO0 done, other role, short phase 2");
}
static int gmu_results(int *res, int max) {
  unsigned i;
  int n = 0;
  for (i = 0; i < g_il.n && i < LOGMAX && n < max; i++) {
    const char *t = strstr(g_il.r[i].t, "VPcmV34GetMaxUpstreamRateIndex(");
    const char *a = t ? strstr(t, " -> ") : NULL;
    if (a) res[n++] = (int)strtol(a + 4, NULL, 10);
  }
  return n;
}
static void cover_t4a(const V34Obj *pre, unsigned lv) {
  int baud = (int16_t)pre->baud_, per = (int16_t)pre->period_;
  int cc = (int16_t)(uint16_t)(pre->txCnt_ + 1u), tgt = baud + (baud >> 1) + per;
  uint16_t snr, w0, w1;
  int lo = 0, r = 0, thr = 0, n = 1, r_start = 0;
  int bis = 0;
  memcpy(&snr, pre->unk04b0_ + 4, 2); memcpy(&w0, (uint8_t *)&pre->w4c2_, 2); memcpy(&w1, (uint8_t *)&pre->w4c4_, 2);
  if (pre->rxSig_ == 0x89b0) CV(G_Q, "a: 16-point symbol"); else CV(G_Q, "a: 4-point symbol");
  if (pre->modeChar_ == 0x65) CV(G_Q, "a: scrambler tap 13"); else CV(G_Q, "a: scrambler tap 26");
  if (cc != tgt) {
    if ((uint32_t)pre->w2218_ <= 3u) { CV(G_Q, "a: off target, w2218 <= 3: nothing"); return; }
    if (cc < tgt) {
      if (cc < baud + per) { CV(G_Q, "a: below baud + period: nothing"); return; }
      if ((int)(int16_t)pre->w47e_ > (int)(int16_t)snr + 10) { CV(G_Q, "a: SNR gate: nothing"); return; }
      CV(G_Q, "a: between baud + period and the target: go");
    } else CV(G_Q, "a: above the target: go");
  } else CV(G_Q, "a: at the target: go");
  if (pre->rxFlags_ & 0x20u) CV(G_Q, "a: rxflags bit 5 set: no precoefs");
  else {
    int sum = 0, k, e, d;
    unsigned cxv = (uint16_t)pre->w47e_, dv;
    uint16_t dvw;
    memcpy(&dvw, pre->unk0488_, 2); dv = dvw;
    for (k = 0; k < 6; k++) { int16_t c; int v; memcpy(&c, pre->unk04ca_ + 0x22 + 2 * k, 2); v = c >> 2; sum += v < 0 ? -v : v; }
    d = (int16_t)(cxv - dv);
    e = d <= 0 ? (int16_t)dv : d;
    e = (int16_t)(e - ((int)(int16_t)cxv >> 2));
    if (sum > 0x1f40 && (int16_t)cxv > 0xc8 && e > 0) { sum = 0x5000; CV(G_Q, "a: sum forced to 0x5000"); }
    if (pre->w359a_ != 0) CV(G_Q, "a: precoefs off: w359a");
    else if ((int16_t)cxv > 0x1ff) CV(G_Q, "a: precoefs off: w47e > 0x1ff");
    else if (!((int16_t)dv < (int16_t)cxv)) CV(G_Q, "a: precoefs off: unk0488 >= w47e");
    else if (!(sum <= 0x3fff)) CV(G_Q, "a: precoefs off: sum too large");
    else { CV(G_Q, "a: precoefs used"); if (lv > 1u) CV(G_Q, "a: precoefs print"); else CV(G_Q, "a: precoefs quiet"); }
    if (lv > 1u && !(pre->w359a_ == 0 && !((int16_t)cxv > 0x1ff) && (int16_t)dv < (int16_t)cxv && sum <= 0x3fff)) CV(G_Q, "a: no-precoefs print");
    lo = e;
  }
  switch (pre->waa96_) {
    case 0x960: lo = 1; r = 9; CV(G_Q, "a: baud 0x960"); break;
    case 0xaf0: lo = 2; r = 0xb; CV(G_Q, "a: baud 0xaf0"); break;
    case 0xbb8: lo = 2; r = 0xc; CV(G_Q, "a: baud 0xbb8"); break;
    case 0xc80: lo = 2; r = 0xd; CV(G_Q, "a: baud 0xc80"); break;
    default: lo = 2; r = 0xe; CV(G_Q, "a: baud 0xd65"); break;
  }
  if (pre->i0dUnk00_[2] & 4u) CV(G_Q, "a: bis allowed (info0d bit)");
  else {
    int cx = (int16_t)pre->waa96_;
    bis = 1;
    if (cx == 0xc80 || cx == 0xd65) { r = 0xc; CV(G_Q, "a: bis, baud 0xc80 / 0xd65: r = 12"); }
    else if (cx == 0xbb8 || cx == 0xaf0) { r = r - 1; CV(G_Q, "a: bis, baud 0xbb8 / 0xaf0: r - 1"); }
    else CV(G_Q, "a: bis, baud 0x960: r unchanged");
    if (lv > 1u) CV(G_Q, "a: bis print"); else CV(G_Q, "a: bis quiet");
  }
  (void)bis;
  r_start = r;
  {
    int steps = 0;
    if (r > lo) {
      do {
        const short *tab = (const short *)pre->scaleTab_;
        int vv = (int)tab[r + 14 * n - 1] >> 5, sq = (int16_t)(vv * vv);
        thr = (int16_t)((sq * (r >= 0xd ? 0x4268 : 0x3a98)) >> 14);
        if ((int16_t)snr < (int16_t)thr) break;
        r--; steps++;
      } while (r > lo);
    }
    if (steps == 0) CV(G_Q, "a: loop stops at the first threshold"); else if (r <= lo) CV(G_Q, "a: loop runs down to lo"); else CV(G_Q, "a: loop lowers r and stops");
  }
  if ((int16_t)w0 > 1) {
    int down = 0;
    if ((uint16_t)w0 == 2) { if (r > (int)(int16_t)w1 - 1) { r = (int16_t)(w1 - 1); down = 1; CV(G_Q, "a: rrn == 2: forced down"); if (lv > 1u) CV(G_Q, "a: forced down print"); else CV(G_Q, "a: forced down quiet"); } else CV(G_Q, "a: rrn == 2: not forced down"); }
    else CV(G_Q, "a: rrn > 2");
    if (!down) { if (!(r >= (int)(int16_t)w1 + 1)) { r = (int16_t)(w1 + 1u); CV(G_Q, "a: forced up"); if (lv > 1u) CV(G_Q, "a: forced up print"); else CV(G_Q, "a: forced up quiet"); } else CV(G_Q, "a: not forced up"); }
    { const short *tab = (const short *)pre->scaleTab_; int vv = (int)tab[r + 14 * n - 1] >> 5, sq = (int16_t)(vv * vv); thr = (int16_t)((sq * (r >= 0xd ? 0x4268 : 0x3a98)) >> 14); }
  } else CV(G_Q, "a: rrn <= 1");
  (void)thr;
  if (lv > 1u) CV(G_Q, "a: ethresh prints"); else CV(G_Q, "a: ethresh quiet");
  if (r <= lo) CV(G_Q, "a: reneg-down from the SNR (r <= lo)"); else CV(G_Q, "a: reneg-down from the table");
  if (r_start <= r) CV(G_Q, "a: reneg-up zero (waa88 <= r)"); else CV(G_Q, "a: reneg-up from the table");
  if ((uint32_t)pre->w238_ >= (uint32_t)pre->w248_ + 0x17700u) CV(G_Q, "a: timer expired: no extra drop");
  else if ((uint32_t)pre->w2218_ > 3u) CV(G_Q, "a: timer running, w2218 > 3: no extra drop");
  else { if ((int16_t)(r - 2) < (int16_t)lo) CV(G_Q, "a: extra drop clamps at lo"); else CV(G_Q, "a: extra drop by two"); r = (int16_t)(r - 2) < (int16_t)lo ? lo : (int16_t)(r - 2); }
  if (lv > 1u) CV(G_Q, "a: automatic print"); else CV(G_Q, "a: automatic quiet");
  {
    int w98 = (int16_t)r;
    if (w98 < pre->minRateIdx_) { CV(G_Q, "a: clamp to the minimum"); w98 = pre->minRateIdx_; }
    if ((int)(int16_t)w98 > pre->maxRateIdx_) { CV(G_Q, "a: clamp to the maximum"); w98 = pre->maxRateIdx_; }
    if (pre->w22c_ >= 0 && pre->w22c_ != pre->w228_) CV(G_Q, "a: DATAXMIT rate override"); else CV(G_Q, "a: no override");
  }
  if (pre->modeChar_ == 0x65) CV(G_Q, "a: MP fields, role 0x65"); else CV(G_Q, "a: MP fields, other role");
  { int gm[2], ng = gmu_results(gm, 2); if (ng) { /* the limit applied? */ CV(G_Q, "a: max upstream queried"); } }
  if (lv > 1u) CV(G_Q, "a: tx change printed"); else CV(G_Q, "a: tx change quiet");   /* txstate is TRNSEG4A at entry: the `already XMITMP` arm is a context constant */
  if (lv > 1u) CV(G_Q, "a: txmp bits print"); else CV(G_Q, "a: txmp bits quiet");
  if (pre->rxFlags_ & 0x20u) CV(G_Q, "a: MP length 0x30"); else CV(G_Q, "a: MP length 0x90");
}
static void cover_rl(const V34Obj *pre, unsigned lv) {
  unsigned w = (uint16_t)(pre->waa78_ + 1u);
  int ptrdiff = (int16_t)((int32_t)((uint32_t)(uintptr_t)pre->rxEnd_ - (uint32_t)(uintptr_t)pre->rxFskBuf_) >> 1);
  (void)ptrdiff;
  if ((int16_t)w <= 0x440) {
    if ((int16_t)w > 0x300) { CV(G_R, "rl: > 0x300: A updated"); return; }
    if ((int16_t)w == 0x300) {
      int z = 1, i;
      for (i = 0; i < 4; i++) { uint16_t b; memcpy(&b, ((const uint8_t *)&pre->wa248_) + 0x524 + 0xc + 0x2c * i, 2); if (b) z = 0; }
      CV(G_R, "rl: == 0x300"); if (z) CV(G_R, "rl: 0x300, energy sum zero"); else CV(G_R, "rl: 0x300, rounded division");
      return;
    }
    if ((uint16_t)(w - 0x1c1u) <= 0x13eu) { CV(G_R, "rl: 0x1c1 .. 0x2ff: A and B updated"); return; }
    if ((uint16_t)w == 0x180) {
      int z = 1, i;
      for (i = 0; i < 4; i++) { uint16_t b; memcpy(&b, ((const uint8_t *)&pre->wa248_) + 0x524 + 0xc + 0x2c * i, 2); if (b) z = 0; }
      CV(G_R, "rl: == 0x180"); if (z) CV(G_R, "rl: 0x180, energy sum zero"); else CV(G_R, "rl: 0x180, rounded division");
      return;
    }
    if ((uint16_t)(w - 0x41u) <= 0x13eu) { CV(G_R, "rl: 0x41 .. 0x17f: A and B updated, counter bumped"); return; }
    if ((uint16_t)w == 0x40) {
      if ((int16_t)pre->rxGain_ > 0x34ff) { if (lv > 1u) CV(G_R, "rl: 0x40, gain error printed"); else CV(G_R, "rl: 0x40, gain error quiet"); }
      else CV(G_R, "rl: 0x40, gain doubled");
      if (lv > 1u) CV(G_R, "rl: 0x40, gain print"); else CV(G_R, "rl: 0x40, gain quiet");
      return;
    }
    CV(G_R, "rl: no action count");
    return;
  }
  if (pre->modeChar_ != 0x65) {
    CV(G_R, "rl: other role: 2400 baud set-up");
    if (lv > 1u) CV(G_R, "rl: other role prints"); else CV(G_R, "rl: other role quiet");
    if (pre->w3596_ == 0x3c) CV(G_R, "rl: other role, tx already TONE_AB");
    if (pre->mst_ == 0x29) CV(G_R, "rl: other role, mst already DET_SYNC");
    return;
  }
  if (w == 0x441) {
    CV(G_R, "rl: role 0x65, first call");
    if (pre->w3596_ == 0x3c) CV(G_R, "rl: first call, tx already TONE_AB"); else if (lv > 1u) CV(G_R, "rl: first call, tx printed"); else CV(G_R, "rl: first call, tx quiet");
    return;
  }
  if ((int16_t)pre->wAae0_ > 0x14 && (pre->wAae2_ & 0xfffu) == 0) {
    CV(G_R, "rl: role 0x65, INFO0 complete");
    if (lv > 1u) CV(G_R, "rl: complete, rx printed"); else CV(G_R, "rl: complete, rx quiet");   /* rx is RX_L1 at entry: the `already RX_DPSK` arms are context constants */
    if (pre->mst_ == 0x3e) CV(G_R, "rl: complete, mst already RX_PHASE3_CALL"); else if (lv > 1u) CV(G_R, "rl: complete, mst printed"); else CV(G_R, "rl: complete, mst quiet");
    return;
  }
  {
    int edx = (int)(int16_t)pre->waa7e_ >> 2, ecx = (int16_t)w;
    if (ecx <= edx + 0xf10) { CV(G_R, "rl: timers: below"); return; }
    if (ecx == edx + 0xf14) { CV(G_R, "rl: timers: tone A wait"); if (lv > 1u) CV(G_R, "rl: tone A wait printed"); else CV(G_R, "rl: tone A wait quiet"); return; }
    if (ecx == edx + 0xf2c) {
      CV(G_R, "rl: timers: restart L1");
      if (lv > 1u) CV(G_R, "rl: restart, rx printed"); else CV(G_R, "rl: restart, rx quiet");
      if (pre->mst_ == 0x33) CV(G_R, "rl: restart, mst already TX_L1"); else if (lv > 1u) CV(G_R, "rl: restart, mst printed"); else CV(G_R, "rl: restart, mst quiet");
      return;
    }
    CV(G_R, "rl: timers: between");
  }
}
static int has(const char *sub) {
  unsigned i;
  for (i = 0; i < g_il.n && i < LOGMAX; i++) if (strstr(g_il.r[i].t, sub)) return 1;
  return 0;
}
static void cover_rcv(const V34Obj *pre, unsigned lv) {
  unsigned fl = pre->rxFlags_;
  uint16_t cnt, r11c, r126;
  unsigned dx, bx;
  memcpy(&cnt, (uint8_t *)&pre->w388_, 2); memcpy(&r11c, pre->unk0378_ + 8, 2); memcpy(&r126, pre->unk038a_, 2);
  if (!has("receiver(")) h_die3("RECEIVE without the receiver call");
  if (fl & 0x40u) { CV(G_S, "rc: flag 0x40 retrain"); if (lv > 1u) CV(G_S, "rc: retrain print"); else CV(G_S, "rc: retrain quiet"); return; }
  if ((int16_t)cnt > 7 * (int16_t)pre->waa96_ && !(fl & 0x80u)) { CV(G_S, "rc: counter retrain"); return; }
  if ((fl & 0x98u) == 0x98u) CV(G_S, "rc: flags 0x98 set");
  else if (fl & 0x10u) { if (fl & 0x400u) CV(G_S, "rc: MP bit-serial receiver"); else CV(G_S, "rc: 0x10 without 0x400"); }
  else if ((int16_t)cnt > 0xbb8) {
    if (fl & 8u) {
      unsigned b = (((unsigned)(int)(int16_t)r126 & 3u) << 14) | ((unsigned)r11c >> 2);
      if ((uint16_t)b == 0x899f) CV(G_S, "rc: flag 8 signature 0x899f"); else CV(G_S, "rc: flag 8 signature other");
    } else {
      dx = (((unsigned)r11c & 3u) << 14) | ((unsigned)pre->rxSig_ >> 2);
      bx = (((unsigned)(int)(int16_t)r126 & 3u) << 14) | ((unsigned)r11c >> 2);
      if ((int16_t)bx != (int16_t)dx) CV(G_S, "rc: signature bx != dx");
      else if ((uint16_t)bx == 0x8990 || (uint16_t)bx == 0x89b0) CV(G_S, "rc: signature 0x8990 / 0x89b0: timing offset logged");
      else CV(G_S, "rc: signature bx == dx other");
    }
  } else CV(G_S, "rc: counter at most 0xbb8");
  if ((fl & 0x98u) == 8u) { if (pre->modeChar_ == 0x65) CV(G_S, "rc: phase 3 end role 0x65"); else CV(G_S, "rc: flags 8, other role"); }
  if (has("E sequence detected")) CV(G_S, "rc: MP E sequence detected (print)");
  if (has("E received before MP")) CV(G_S, "rc: E received before MP (print)");
  if (has("-MP sequence")) CV(G_S, "rc: MP sequence (print)");
  if (has("-MP1 sequence")) CV(G_S, "rc: MP1 sequence (print)");
  if (has("=>DET_INFO(")) CV(G_S, "rc: microstate -> DET_INFO");
  if (has("=>DET_SYNC(")) CV(G_S, "rc: microstate -> DET_SYNC");
  if (has("initdigital(")) CV(G_S, "rc: initdigital");
  if (has("VPcmV34LogTimingOffset(")) CV(G_S, "rc: LogTimingOffset");
  if (has("settxlevel(")) CV(G_S, "rc: phase 3 settxlevel");
  if (has("V34SetupModulator(")) CV(G_S, "rc: phase 3 V34SetupModulator");
  if (has("txinit(")) CV(G_S, "rc: txinit");
  if (has("rxinit(")) CV(G_S, "rc: rxinit (MD length over)");
  if (has("detectorinit(")) CV(G_S, "rc: detectorinit");
  if (has("tone_detect(")) CV(G_S, "rc: tone_detect");
  if (has("Agc gain estimate")) CV(G_S, "rc: Agc print");
  if (has("V34TIMING")) CV(G_S, "rc: V34TIMING print");
  if (has("S detected")) CV(G_S, "rc: S detected print");
  if (has("RX MDLENGTH, getting")) CV(G_S, "rc: MDLENGTH waiting print");
  if (has("RX MDLENGTH over")) CV(G_S, "rc: MDLENGTH over print");
  if (has("V34AGC, setup")) CV(G_S, "rc: V34AGC setup print");
  if (has("V34SetupDemodulator")) CV(G_S, "rc: V34SetupDemodulator print");
}
static void cover_dft(const V34Obj *pre, unsigned lv) {
  const uint8_t *u = (const uint8_t *)&pre->wa248_, *C = u + 0x5d4;
  uint32_t cnt;
  uint16_t st, a8, aa, ac;
  memcpy(&cnt, u + 4, 4); memcpy(&st, u + 2, 2); memcpy(&a8, u + 8, 2); memcpy(&aa, u + 0xa, 2); memcpy(&ac, u + 0xc, 2);
  CV(G_T, "dw: window fed");
  if (cnt + 4u != 0x80u) { CV(G_T, "dw: counter below 0x80"); return; }
  CV(G_T, "dw: evaluation");
  if ((int16_t)st == 1) {
    uint16_t e0, e1, e2; int16_t l0, l1, l2;
    memcpy(&e0, C + 0xc, 2); memcpy(&l0, C + 0x28, 2); memcpy(&e1, C + 0x38, 2); memcpy(&l1, C + 0x54, 2); memcpy(&e2, C + 0x64, 2); memcpy(&l2, C + 0x80, 2);
    if ((int)e0 < l0 && (int)e1 < l1 && (int)e2 < l2) CV(G_T, "dw: state 1, all energies below the limits");
    else { if ((int16_t)a8 >= (int16_t)aa) CV(G_T, "dw: state 1 -> 2"); else CV(G_T, "dw: state 1, quiet counter below its limit"); }
    return;
  }
  if ((int16_t)st != 2) { CV(G_T, "dw: other state word"); return; }
  {
    uint16_t e1; int16_t l;
    int cur = a8;
    memcpy(&e1, C + 0x38, 2); memcpy(&l, C + 0x56, 2);
    if ((int)e1 <= l) { CV(G_T, "dw: state 2, energy not above: back to state 1"); cur = 0; } else { CV(G_T, "dw: state 2, energy above: count"); cur = (uint16_t)(a8 + 1u); }
    if ((uint16_t)cur != ac) { CV(G_T, "dw: count below the request limit"); return; }
    CV(G_T, "dw: retrain request declared");
    if (lv > 1u) CV(G_T, "dw: declared printed"); else CV(G_T, "dw: declared quiet");
    if (pre->mst_ == 0x2e) CV(G_T, "dw: mst already TX_PHASE1_ANS");
    if (pre->w358c_ > 1u) CV(G_T, "dw: w358c cleared"); else CV(G_T, "dw: w358c kept");
  }
}
static void cover_rx(const V34Obj *pre, unsigned lv) {
  switch (pre->mst_) {
    case 0x3e:
      if (!(pre->wAae2_ & 1u)) CV(G_E, "r3c: signal bit clear");
      else if (lv > 1u) CV(G_E, "r3c: -> 0x38 printed");
      else CV(G_E, "r3c: -> 0x38 quiet");
      break;
    case 0x30: {
      uint16_t a = (uint16_t)(pre->waa78_ + 1u);
      if (a == 0x78) CV(G_E, "t3a: 0x78 flip");
      else if (a != 0xa2) CV(G_E, "t3a: counting");
      else {
        if (pre->w3596_ == 5) CV(G_E, "t3a: 0xa2, tx already SILENCE");
        else if (lv > 1u) CV(G_E, "t3a: 0xa2, tx change printed");
        else CV(G_E, "t3a: 0xa2, tx change quiet");
        if (lv > 1u) CV(G_E, "t3a: 0xa2, micro printed"); else CV(G_E, "t3a: 0xa2, micro quiet");
      }
      break;
    }
    case 0x2f: case 0x38:
      if ((pre->wAae2_ & 0xfu) == 0xfu && pre->w3588_ == 0) {
        if (pre->w3596_ == 0x18) CV(G_E, "t2: init, tx already TX_DPSK");
        else if (lv > 1u) CV(G_E, "t2: init, tx change printed");
        else CV(G_E, "t2: init, tx change quiet");
        if (pre->modeChar_ == 0x65 && pre->pcmMode_ != 0) CV(G_E, "t2: init, role 0x65 with pcm");
        else CV(G_E, "t2: init, other role / no pcm");
        if (lv > 1u) CV(G_E, "t2: init, repeated-info0 print"); else CV(G_E, "t2: init, quiet");
      } else CV(G_E, "t2: no init");
      if ((int16_t)(uint16_t)(pre->waa78_ + 1u) <= 0x5f) CV(G_E, "t2: counter below 0x60");
      else if (lv > 1u) CV(G_E, "t2: flip, micro printed");
      else CV(G_E, "t2: flip, micro quiet");
      break;
    case 0x3f:
      if (pre->w3596_ == 0x3c) {
        if ((int16_t)(uint16_t)(pre->waa78_ + 1u) <= 0xf) CV(G_E, "idone: TONE_AB counting");
        else if (pre->modeChar_ == 0x65) { if (lv > 1u) CV(G_E, "idone: TONE_AB role 0x65 -> SILENCE printed"); else CV(G_E, "idone: TONE_AB role 0x65 -> SILENCE quiet"); }
        else { if (lv > 1u) CV(G_E, "idone: TONE_AB other role printed"); else CV(G_E, "idone: TONE_AB other role quiet"); if (pre->w8_ == 0x30) CV(G_E, "idone: TONE_AB w8 0x30"); else CV(G_E, "idone: TONE_AB w8 other"); }
      } else if (pre->w3596_ != 5) CV(G_E, "idone: other txstate");
      else if ((int8_t)pre->wAbff_ <= 0) CV(G_E, "idone: SILENCE, wAbff low");
      else if (pre->w240_ <= 0x240) CV(G_E, "idone: SILENCE, w240 low");
      else {
        if (lv > 1u) CV(G_E, "idone: phase 3 set-up printed"); else CV(G_E, "idone: phase 3 set-up quiet");
        if (pre->pcmMode_ != 0 || pre->k56Mode_ != 0) CV(G_E, "idone: v90 flag"); else CV(G_E, "idone: no v90 flag");
        if (lv > 1u) CV(G_E, "idone: rx -> WAIT and tx -> SSEG printed"); else CV(G_E, "idone: rx -> WAIT and tx -> SSEG quiet");   /* the `already WAIT / SSEG` guards cannot be true here: rx is RX_DPSK, tx is SILENCE */
      }
      break;
    case 0x50:
      if (tone_hit()) { if (pre->mohF8_ == 0) { if (lv > 1u) CV(G_E, "mtd: tone hit arms mohF8_, printed"); else CV(G_E, "mtd: tone hit arms mohF8_, quiet"); } else CV(G_E, "mtd: tone hit, mohF8_ already set"); } else CV(G_E, "mtd: no tone hit");
      if ((int)(int16_t)(uint16_t)(pre->waa78_ + 1u) < ((int)(int16_t)pre->waa7e_ >> 2) + 0x12c0) CV(G_E, "mtd: below the limit");
      else if (pre->mohF9_ != 0) { if (pre->w3596_ == 0x54) CV(G_E, "mtd: disconnect, tx already 0x54"); else CV(G_E, "mtd: disconnect, tx change"); }
      else CV(G_E, "mtd: retrain");
      break;
    default: CV(G_E, "no-op / out-of-range microstate"); break;
  }
}
static void cover_post(const V34Obj *pre) {
  unsigned tx = pre->w3596_;
  int gate = (int16_t)pre->rxW0_ > 5;
  unsigned idx = (unsigned)((int)(int16_t)tx - 5);
  if (!gate) CV(G_F, "post: gate closed"); else if ((int16_t)pre->rxState_ == 0x23) CV(G_F, "post: WAIT"); else CV(G_F, "post: other receive state");
  if (idx > 0x45u) CV(G_F, "txpost: out of range");
  else switch (tx) {
    case 5: {
      unsigned mst = pre->mst_, rx = pre->rxState_;
      if (mst == 0x3f && pre->modeChar_ == 0x66) CV(G_F, "silence: 0x3f role 0x66 -> 1");
      else if ((uint16_t)rx == 4) CV(G_F, "silence: rx RECEIVE");
      else if ((uint16_t)rx == 0x23) { if (mst == 0x3f && pre->modeChar_ == 0x65) CV(G_F, "silence: rx WAIT 0x3f role 0x65 -> 1"); else CV(G_F, "silence: rx WAIT other -> 0"); }
      else CV(G_F, "silence: other -> 0");
      break;
    }
    case 0x18: case 0x33: case 0x36: case 0x3c: case 0x4a: CV(G_F, "txpost: status 0 group"); break;
    case 0x46: if (pre->wE4c_ == 0) CV(G_F, "txpost: DATAXMIT 3"); else CV(G_F, "txpost: DATAXMIT 4"); break;
    case 0x42: case 0x43: case 0x45: CV(G_F, "txpost: status 3 group"); break;
    case 0x14: case 0x15: case 0x40: case 0x44: CV(G_F, "txpost: status 2 group"); break;
    case 0x12: case 0x13: if (pre->modeChar_ == 0x65) CV(G_F, "txpost: SSEG role 0x65"); else if ((pre->rxFlags_ >> 3) & 1u) CV(G_F, "txpost: SSEG rxFlags bit 3 -> 3"); else CV(G_F, "txpost: SSEG -> 2"); break;
    default: CV(G_F, "txpost: no stub"); break;
  }
  if (pre->w2218_ == 1) CV(G_F, "tail: w2218 == 1");
  if ((uint32_t)(pre->w2218_ - 4) <= 1u) CV(G_F, "tail: w2218 4 or 5");
  if ((uint32_t)(pre->w2218_ - 2) <= 1u && (uint16_t)tx == 0x4a) { if ((int16_t)pre->k56Idx_ > 0x3c) CV(G_F, "tail: SILENCERETRAIN k56Idx high"); else CV(G_F, "tail: SILENCERETRAIN k56Idx low"); }
  if ((uint32_t)pre->w238_ > pre->w23c_) CV(G_F, "tail: w238 > w23c");
  if ((int32_t)(int16_t)pre->rxLevel_ >= pre->minSigLevel_) CV(G_F, "tail: level ok, counter cleared"); else CV(G_F, "tail: level low, counter incremented");
  if (!((int32_t)(int16_t)pre->rxLevel_ >= pre->minSigLevel_) && (int32_t)(pre->w234_ + 1) > 0x257f) CV(G_F, "tail: counter above 0x257f");
  if ((int16_t)tx == 0x53) CV(G_F, "tail: MOH_FRR -> 0xf"); else if ((int16_t)tx == 0x52) CV(G_F, "tail: MOH_ON_HOLD -> 0xd"); else if ((int16_t)tx == 0x54) CV(G_F, "tail: MOH_CLEARDOWN -> 0x10");
}
static void cover(int sc, uint16_t state, const V34Obj *pre, unsigned lv, int printed) {
  if (sc == G_F) { cover_post(pre); return; }
  if (sc == G_I) { cover_dpsk(pre, lv); return; }
  if (sc == G_J) { cover_xmitmp(pre, lv); return; }
  if (sc == G_K) { cover_moh_tone(pre, lv); return; }
  if (sc == G_M) { cover_mm(pre, lv); return; }
  if (sc == G_N) { cover_detab(pre, lv); return; }
  if (sc == G_O) { cover_dsync(pre, lv); return; }
  if (sc == G_P) { cover_dinfo(pre, lv); return; }
  if (sc == G_Q) { cover_t4a(pre, lv); return; }
  if (sc == G_R) { cover_rl(pre, lv); return; }
  if (sc == G_S) { cover_rcv(pre, lv); return; }
  if (sc == G_T) { cover_dft(pre, lv); return; }
  if (sc == G_U || sc == G_V) return;   /* the sites are inside run_helpers */
  if (sc == G_L) { cover_trnseg4(pre, lv); return; }
  if (sc == G_E) { cover_rx(pre, lv); return; }
  if (sc == G_POKE || sc == G_SEQ) {   /* the poke / sequence scenarios require only the generic sites */
    if (sc == G_POKE && is_ported(state)) CV(G_POKE, "ported state dispatched");
    if (sc == G_SEQ && is_ported(state)) CV(G_SEQ, "ported state dispatched");
    if (sc == G_SEQ && is_noop(state)) CV(G_SEQ, "no-op state dispatched");
    return;
  }
  switch (state) {
    case 0x12:
      if ((uint16_t)(pre->txCnt_ + 1u) != 0x40) CV(G_A, "sseg: counter not at 0x40");
      else if (lv > 1u) CV(G_A, "sseg: wrap, state change printed");
      else CV(G_A, "sseg: wrap, state change quiet");
      break;
    case 0x3c: if ((int16_t)pre->w358c_ & 1) CV(G_A, "toneab: odd point"); else CV(G_A, "toneab: even point"); break;
    case 0x41:
      if (!(pre->rxFlags_ & 8u)) CV(G_A, "xmit0: rxFlags bit 3 clear");
      else if (lv > 1u) CV(G_A, "xmit0: bit 3 set, SSEG transition printed");
      else CV(G_A, "xmit0: bit 3 set, SSEG transition quiet");
      break;
    case 0x47: if (pre->modeChar_ == 0x65) CV(G_A, "txlevel: role 0x65"); else CV(G_A, "txlevel: other role"); if (g_o->txBits_ != 0) CV(G_A, "txlevel: top bits nonzero"); else CV(G_A, "txlevel: top bits zero"); break;
    case 0x46:
      if (!(pre->txFlags_ & 0x10u)) CV(G_B, "dataxmit: flush bit clear");
      else if (pre->w2218_ != 0) CV(G_B, "dataxmit: flushed, w2218 nonzero");
      else if (lv > 1u) CV(G_B, "dataxmit: flushed, w2218 zero, printed");
      else CV(G_B, "dataxmit: flushed, w2218 zero, quiet");
      break;
    case 0x51: case 0x52: case 0x53: case 0x54:
      if ((uint16_t)(pre->k56Idx_ + 1u) != 0xc0) CV(G_B, "moh: counter not at 0xc0");
      else if (state != 0x51) CV(G_B, "moh: 0xc0 in a state other than MOH_SILENCE");
      else if (pre->mohEC_ == 1u) CV(G_B, "moh: 0xc0 -> FRR via mohEC");
      else if (pre->mohF0_ == 1u) CV(G_B, "moh: 0xc0 -> FRR via mohF0");
      else CV(G_B, "moh: 0xc0 -> ON_HOLD");
      if ((uint16_t)(pre->k56Idx_ + 1u) == 0xc0 && state == 0x51) { if (lv > 1u) CV(G_B, "moh: 0xc0 printed"); else CV(G_B, "moh: 0xc0 quiet"); }
      break;
    case 0x4e: case 0x55: {
      const char *nm = state == 0x4e ? "ja" : "k56ja";
      (void)nm;
      if (pre->waa78_ == 0) { if (state == 0x4e) CV(G_C, "ja: countdown idle"); else CV(G_C, "k56ja: countdown idle"); }
      else if ((uint16_t)(pre->waa78_ - 1u) != 0) { if (state == 0x4e) CV(G_C, "ja: counting down"); else CV(G_C, "k56ja: counting down"); }
      else if (lv > 1u) { if (state == 0x4e) CV(G_C, "ja: freeze printed"); else CV(G_C, "k56ja: freeze printed"); }
      else { if (state == 0x4e) CV(G_C, "ja: freeze quiet"); else CV(G_C, "k56ja: freeze quiet"); }
      break;
    }
    case 0x33:
      if (pre->mst_ == 0x33) CV(G_D, "txl1: doubled samples (mst 0x33)"); else CV(G_D, "txl1: plain samples");
      if (pre->mst_ == 0x33 && (uint16_t)(pre->k56Idx_ + 4u) == 0x600) { if (lv > 1u) CV(G_D, "txl1: 0x600 -> TX_L2 printed"); else CV(G_D, "txl1: 0x600 -> TX_L2 quiet"); }
      break;
    case 0x13:
      if ((uint16_t)(pre->txCnt_ + 1u) != 8) CV(G_D, "sbarseg: counter not at 8");
      else if (pre->txFlags_ & 0x2000u) { if (lv > 1u) CV(G_D, "sbarseg: bit 13 -> TRNSEG4A printed"); else CV(G_D, "sbarseg: bit 13 -> TRNSEG4A quiet"); }
      else if ((uint16_t)pre->w35a4_ == 0) CV(G_D, "sbarseg: w35a4 zero -> PPSEG");
      else if ((int16_t)pre->txFlags_ < 0) CV(G_D, "sbarseg: txFlags negative -> PPSEG");
      else { if (lv > 1u) CV(G_D, "sbarseg: TXMD set-up printed"); else CV(G_D, "sbarseg: TXMD set-up quiet"); }
      break;
    case 0x14:
      if ((uint16_t)(pre->k56Idx_ + 1u) != 0x30) CV(G_D, "ppseg: counter not at 0x30");
      else if ((uint16_t)(pre->w358e_ + 1u) != 6) CV(G_D, "ppseg: 0x30 but pass count not 6");
      else {
        switch (pre->baud_) {
          case 0x960: CV(G_D, "ppseg: baud 0x960"); break;
          case 0xaf0: CV(G_D, "ppseg: baud 0xaf0"); break;
          case 0xbb8: CV(G_D, "ppseg: baud 0xbb8"); break;
          case 0xc80: CV(G_D, "ppseg: baud 0xc80"); break;
          case 0xd65: CV(G_D, "ppseg: baud 0xd65"); break;
          default: CV(G_D, "ppseg: other baud"); break;
        }
        if (pre->modeChar_ == 0x65) CV(G_D, "ppseg: role 0x65"); else CV(G_D, "ppseg: other role");
        if (lv > 1u) CV(G_D, "ppseg: transition + wait print"); else CV(G_D, "ppseg: quiet");
      }
      break;
    case 5: CV(G_H, "silence: SILENCE, queue write only"); break;
    case 0x36:
      if ((uint16_t)(pre->k56Idx_ + 1u) != 0xa) CV(G_H, "silenceinfo: counter below 10");
      else {
        if (lv > 1u) CV(G_H, "silenceinfo: 10 -> TX_DPSK printed"); else CV(G_H, "silenceinfo: 10 -> TX_DPSK quiet");
        if (pre->modeChar_ == 0x65 && pre->pcmMode_ != 0) CV(G_H, "silenceinfo: role 0x65 with pcm"); else CV(G_H, "silenceinfo: other role / no pcm");
      }
      break;
    case 0x4a:
      if ((uint16_t)(pre->k56Idx_ + 1u) != 0xb4) CV(G_H, "silenceretrain: counter below 0xb4");
      else {
        unsigned want = pre->mohE8_ == 0 ? (pre->modeChar_ == 0x65 ? 0x3au : 0x2eu) : 0x4fu;
        if (pre->mst_ == want) CV(G_H, "silenceretrain: microstate already set"); else if (lv > 1u) CV(G_H, "silenceretrain: microstate change printed"); else CV(G_H, "silenceretrain: microstate change quiet");
        if (pre->mohE8_ == 0) CV(G_H, "silenceretrain: mohE8 clear"); else CV(G_H, "silenceretrain: mohE8 set");
        if (pre->rxState_ == 0x2b) CV(G_H, "silenceretrain: rx already RX_DPSK"); else if (lv > 1u) CV(G_H, "silenceretrain: rx change printed"); else CV(G_H, "silenceretrain: rx change quiet");
        if (lv > 1u) CV(G_H, "silenceretrain: tx change printed + finished print"); else CV(G_H, "silenceretrain: quiet");
      }
      break;
    case 0x45:
      if (pre->rxSig_ == 0x89b0) CV(G_D, "exmit: 16-point"); else CV(G_D, "exmit: 4-point");
      if ((uint16_t)(pre->k56Idx_ + (pre->rxSig_ == 0x89b0 ? 4u : 2u)) == 0x14) { if (lv > 1u) CV(G_D, "exmit: done printed"); else CV(G_D, "exmit: done quiet"); }
      else CV(G_D, "exmit: not done");
      if (pre->txFlags_ & 1u) CV(G_D, "exmit: tap 26"); else CV(G_D, "exmit: tap 13");
      break;
    case 0x40: case 0x44:
      if (pre->waa78_ == 0) CV(G_D, "j: countdown idle");
      else if ((uint16_t)(pre->waa78_ - 1u) != 0) CV(G_D, "j: counting down");
      else if (lv > 1u) CV(G_D, "j: freeze printed");
      else CV(G_D, "j: freeze quiet");
      if (pre->modeChar_ == 0x66) {
        if (pre->rxFlags_ & 0x400u) { if (lv > 1u) CV(G_D, "j: role 0x66, rx signal -> XMIT0 printed"); else CV(G_D, "j: role 0x66, rx signal -> XMIT0 quiet"); }
        else if (pre->w35a2_ == 0) CV(G_D, "j: role 0x66, w35a2 zero");
        else if (pre->w25da_ == 0) CV(G_D, "j: role 0x66, w25da zero");
        else CV(G_D, "j: role 0x66, flags -> XMIT0");
      } else if (((pre->k56Idx_ + 1u) & 7u) != 0) CV(G_D, "j: symbol not at the end");
      else if (state == 0x44) { if (lv > 1u) CV(G_D, "j: J1TXMIT end -> TRNSEG4A printed"); else CV(G_D, "j: J1TXMIT end -> TRNSEG4A quiet"); }
      else if (pre->w25da_ != 2) CV(G_D, "j: JTXMIT end, w25da not 2");
      else if ((int16_t)pre->symCnt_ <= 0x80) CV(G_D, "j: JTXMIT end, symCnt small");
      else if (pre->waa78_ <= 1) CV(G_D, "j: JTXMIT -> J1TXMIT, no forced freeze");
      else { if (lv > 1u) CV(G_D, "j: JTXMIT -> J1TXMIT, forced freeze printed"); else CV(G_D, "j: JTXMIT -> J1TXMIT, forced freeze quiet"); }
      break;
    case 0x56:
      if (pre->waa78_ == (uint16_t)(pre->k56Idx_ + 1u)) { if (lv > 1u) CV(G_D, "txmd: echo adaptation printed"); else CV(G_D, "txmd: echo adaptation quiet"); }
      if ((uint16_t)(pre->k56Idx_ + 1u) == pre->w35a6_) {
        if (pre->pcmMode_ != 0) CV(G_D, "txmd: done, pcmMode");
        else if (pre->k56Mode_ != 0) CV(G_D, "txmd: done, k56Mode");
        else CV(G_D, "txmd: done, neither mode");
        if (lv > 1u) CV(G_D, "txmd: done printed"); else CV(G_D, "txmd: done quiet");
      } else CV(G_D, "txmd: not done");
      break;
    default: break;
  }
  (void)printed;
}

static void do_call(int sc) {
  unsigned i, lv0 = dsplibs_debug_level;
  V34Obj pre;
  uint16_t state = (sc == G_E || sc == G_K || sc == G_M || sc == G_O || sc == G_P || sc == G_T) ? g_o->mst_ : (sc == G_N || sc == G_R || sc == G_S) ? g_o->rxState_ : g_o->w3596_;
  memcpy(&pre, g_o, sizeof pre);
  g_il.n = 0; g_pk = 0; g_rv = h_rnd();   /* the tone_detect recorder's result sequence differs per call */
  cov_clear();
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 1500) || g_showall;
  h_emit(g_show, "call state=%04x lvl=%u", (unsigned)state, dsplibs_debug_level);
  g_sc_run = sc;
  run_iso(g_o);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("canary byte %u behind the object was overwritten", i);
  dump_world(g_show);
  cover(sc, state, &pre, lv0, 0);
  dsplibs_debug_level = lv0;
  cov_flush();
  g_ncalls++;
}

static void set_state(uint16_t st) {   /* the harness never gives PPSEG a counter outside its 48-dword table (vectpp has no bounds check in the blob either) */
  g_o->w3596_ = st;
  if (st == 0x14) g_o->k56Idx_ = (uint16_t)(g_o->k56Idx_ % 0x30u);
}
static void trial(int sc) {
  int k, nm;
  uint16_t st = sc == G_E ? pick_mst() : sc == G_K ? (uint16_t)0x4f : sc == G_M ? pick_mm() : sc == G_N ? (uint16_t)0x35 : sc == G_O ? (uint16_t)0x29 : sc == G_P ? (uint16_t)0x2c : sc == G_Q ? (uint16_t)0x42 : sc == G_R ? (uint16_t)0x48 : sc == G_S ? (uint16_t)4 : sc == G_T ? (uint16_t)0x2e : (sc == G_U || sc == G_V) ? (uint16_t)0 : pick_state(sc);
  new_world(sc, st);
  gen_script();
  dsplibs_debug_level = (sc == G_E || sc == G_K || sc == G_M || sc == G_N || sc == G_O || sc == G_P || sc == G_R || sc == G_S || sc == G_T || sc == G_U || sc == G_V) && h_rpct(30) ? h_rmod(2) : pick_lv();
  if (sc == G_D && (st == 0x40 || st == 0x44) && h_rpct(20)) dsplibs_debug_level = 1;   /* gap: JTXMIT at level exactly 1 */
  g_pokemask = (sc == G_POKE || sc == G_H || sc == G_E || sc == G_D || sc == G_I || sc == G_J || sc == G_K || sc == G_L || sc == G_M || sc == G_N || sc == G_O || sc == G_P || sc == G_Q || sc == G_R || sc == G_S || sc == G_T) ? 0xfffffffeu : 0;   /* the SILENCE trio keeps register copies across prints: observable only with pokes */
  if (sc == G_SEQ && h_rpct(60)) {   /* a walk: the state one call leaves is the next one's input, as long as it stays in the ported / no-op set */
    int len = 70 + (int)h_rmod(200);
    uint16_t walk[] = { 0x12, 0x51, 0x4e, 0x55 };
    st = walk[h_rmod(4)];
    set_state(st);
    if (st == 0x12) g_o->txCnt_ = (uint16_t)(h_rpct(50) ? 0 : 0x20 + h_rmod(0x20));
    if (st == 0x51) g_o->k56Idx_ = (uint16_t)(h_rpct(50) ? 0 : 0x80 + h_rmod(0x40));
    if (st == 0x4e || st == 0x55) g_o->waa78_ = (uint16_t)(1 + h_rmod(4));
    for (k = 0; k < len; k++) {
      if (!(k & 15)) gen_script();
      if (k && !is_ported(g_o->w3596_)) break;
      do_call(sc);
    }
  } else {
    nm = sc == G_POKE || sc == G_SEQ ? (int)(1 + h_rmod(3)) : 1;
    for (k = 0; k < nm; k++) {
      if (k) {
        gen_script();
        if (sc == G_POKE || sc == G_SEQ) { set_state(pick_state(sc)); if (!is_ported(g_o->w3596_) && !is_noop(g_o->w3596_)) break; }
        if (h_rpct(15)) dsplibs_debug_level = pick_lv();
      }
      do_call(sc);
    }
  }
  h_trial_end("v04");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("V04_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("V04_ALL")) g_showall = 1;
  if (getenv("V04_WHOLE")) g_whole = 1;
  if (sc < 0 || sc > 22) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  v01_layout_check();
  h_x87_check();
  patch_iso(sc);
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("V04_TRIALS") ? atoi(getenv("V04_TRIALS")) : (sc == G_SEQ ? 3000 : sc == G_POKE ? 12000 : sc == G_D ? 20000 : sc == G_I ? 30000 : sc == G_J ? 30000 : sc == G_K ? 12000 : sc == G_L ? 30000 : sc == G_M ? 40000 : sc == G_N ? 30000 : sc == G_O ? 40000 : sc == G_P ? 40000 : sc == G_Q ? 40000 : sc == G_R ? 40000 : sc == G_S ? 40000 : sc == G_T ? 40000 : (sc == G_U || sc == G_V) ? 30000 : 8000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("V04_NOCOV")) ? 2 : 0;
}
