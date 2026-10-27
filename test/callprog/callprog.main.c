/*
 * R11 (id r11_callprog): differential test of CALLPROG_Create / _Dial / _Progress / _Delete / _Status_string, the call-progress state machine, with ALL its callees replaced by
 * scripted recorders (cadence_create / _progress / _reset / _delete, DialerCreate / DialerProgress / DialerAbort, ResetCallingTone / GenerateCallingTone, Dual_TONE_*, _iir_filter_*,
 * modem_get_param, the sreg callback, sysdep_*, dsplibs_debug_printf): the real dsplibs.o (those callees WEAKENED with objcopy, so the recorders win) vs src/callprog/callprog.c.  ORDER and
 * ARGUMENTS of every callback and the whole CallProg object (layout independent description: pointers as pool handles, the sreg callback by identity) are compared after every call.
 * The state tables are the blob's .bss LOCALs (rebuilt by Create / Dial): they are observed through the behaviour of Progress (every state x event cell is driven).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 Create: 160 trials (cfg blind / getSReg NULL or not / modem / w0c, modem_get_param 0x3b filter flag, cadence_create NULL x2, dual NULL, dirty objects, debug levels incl.
 *     0x80000000 / 0xffffffff)
 *   1 Dial: 400 trials over every CALLPROG state, blind / not, getSReg NULL or returning random values (high bits set: only the low SIGNED byte is used), the modem parameters
 *     0x1b / 0x2d / 0x22 / 0x1d / 0x1e (negative, huge, around the (x + 9) / 10 > 2 test), DialerCreate results 0 / 7 / other, NULL / empty / long dial strings
 *   2 Progress on fabricated objects: 30,000 calls over states 0 .. 9, counters at their boundaries, every event / table cell, n = 0 .. 0xa0 and 0xa1 .., scripted cadence /
 *     dialer / dual-tone results, the silence detector near its threshold, line-clear / timeout boundaries
 *   3 sessions: Create + Dial + 300 Progress calls with low probabilities of the scripted events (blind and normal dialling, short timeouts), then Delete
 *   4 Delete lifecycles (filter flag, cadence / dual-tone NULL, the sequence of debug text) and Status_string for every id -5 .. 40
 * Coverage counters exit 2 on a zero, 3 = harness violation.  Env R11_SEED (other input stream), R11_VERBOSE=1 (print every line).
 */
#include "r11_common.h"

enum { CV_STATE0 = 0, CV_RET0 = 10, /* 10 .. 28: Progress result r */ CV_DLR0 = 30, /* 30 .. 40: dialer result */ CV_DBG0 = 50 /* debug strings */ };
/* ------------------------------------------------------------------------------------------------ handle pools */
static int g_pool[3][256];
static int g_npool[3];
static int g_failc[8], g_nfailc;      /* cadence_create: bit k set = the k-th call returns NULL */
static unsigned g_cad_fail_mask, g_dual_fail_mask;
static unsigned g_ncad_create, g_ndual_create;
static void *newh(int kind)
{
  if (g_npool[kind] >= 256) die3("pool exhausted");
  return &g_pool[kind][g_npool[kind]++];
}
static const char *dh(const void *p, char *buf)
{
  int k, i;
  if (!p) return "NULL";
  for (k = 0; k < 3; k++)
    for (i = 0; i < g_npool[k]; i++)
      if (p == (const void *)&g_pool[k][i]) {
        sprintf(buf, "%c%d", "cid"[k], i);
        return buf;
      }
  return "?";
}
static CallProg *g_cp;       /* the object under test (the fakes check the interior pointers they get) */

/* ------------------------------------------------------------------------------------------------ the scripted callees */
static int g_pc1 = 2, g_pc7 = 1;   /* per mille: cadence_progress returns 1 / 7 */
static int g_dialw[10];            /* DialerProgress result weights (0 .. 7, 8 = out of range, 9 = negative) */
static int g_dualw[6];             /* Dual_TONE_detect result weights 0 .. 5 */
static int g_dres_create = 0;      /* DialerCreate result (-1 random) */
static int g_prog4run;
static int g_dialer_calls;

static int pick_w(const int *w, int n)
{
  int tot = 0, i, r;
  for (i = 0; i < n; i++) tot += w[i];
  if (tot <= 0) return 0;
  r = (int)rmod((unsigned)tot);
  for (i = 0; i < n; i++) {
    if (r < w[i]) return i;
    r -= w[i];
  }
  return 0;
}
static unsigned long g_ncadp;
Cadence *cadence_create(Cadence *obj, CadenceParams *p, int extra, struct modem *m)
{
  char b[16];
  unsigned k = g_ncad_create++;
  emit(0, "CADC obj=%s p=%d,%d,%d,%d,%u,%d,%d x=%d m=%d", obj ? "set" : "NULL", p->w0, p->w4, p->w8, p->callerParamA, p->type, p->w14, p->callerParamB, extra, MID(m));
  if (g_cad_fail_mask & (1u << k)) return NULL;
  (void)b;
  return (Cadence *)newh(0);
}
int cadence_progress(Cadence *c, short sample)
{
  char b[16];
  int r = 0, x = (int)rmod(1000);
  g_ncadp++;
  if (x < g_pc1) r = 1;
  else if (x < g_pc1 + g_pc7) r = 7;
  else if (x < g_pc1 + g_pc7 + 3) r = (int)rmod(10) - 1;
  { /* per-sample callback: folded into the hash */
    u64 hh = fnv_bytes(dh(c, b), strlen(dh(c, b)));
    hh = fnv_mix(hh, (unsigned)(int)sample);
    hh = fnv_mix(hh, (unsigned)r);
    g_hash ^= hh;
    g_hash *= 1099511628211ULL;
    g_lines++;
  }
  return r;
}
void cadence_reset(Cadence *c)
{
  char b[16];
  emit(0, "CADR %s", dh(c, b));
}
void cadence_delete(Cadence *c)
{
  char b[16];
  emit(0, "CADD %s", dh(c, b));
}
DualTone *Dual_TONE_create(void)
{
  unsigned k = g_ndual_create++;
  emit(0, "DTC");
  if (g_dual_fail_mask & (1u << k)) return NULL;
  return (DualTone *)newh(2);
}
int Dual_TONE_detect(DualTone *d, const short *in, int n)
{
  char b[16];
  int r = pick_w(g_dualw, 6);
  emit(0, "DTD %s n=%d in=%016llx -> %d", dh(d, b), n, n > 0 && n <= 0xa0 ? fnv_bytes(in, (size_t)n * 2) : 0ULL, r);
  return r;
}
void Dual_TONE_delete(DualTone *d)
{
  char b[16];
  emit(0, "DTX %s", dh(d, b));
}
IirFilter *_iir_filter_create(IirFilter *obj, int nA, int nB, const short *a, const short *b, const short *scales)
{
  emit(0, "IFC obj=%s %d %d a=%016llx b=%016llx s=%016llx", obj ? "set" : "NULL", nA, nB, fnv_bytes(a, 24), fnv_bytes(b, 24), fnv_bytes(scales, 10));
  return (IirFilter *)newh(1);
}
void _iir_filter_progress(IirFilter *f, int n, short *buf)
{
  char b[16];
  int i;
  emit(0, "IFP %s n=%d in=%016llx", dh(f, b), n, n > 0 && n <= 0xa0 ? fnv_bytes(buf, (size_t)n * 2) : 0ULL);
  for (i = 0; i < n && i < 0xa0; i++) buf[i] = (short)(buf[i] / 2 + 3);
}
void _iir_filter_delete(IirFilter *f)
{
  char b[16];
  emit(0, "IFD %s", dh(f, b));
}
void ResetCallingTone(CallingTone *ct, int level)
{
  emit(0, "RCT %s level=%d", (char *)ct - (char *)g_cp == (long)offsetof(CallProg, ct) ? "ct" : "?", level);
  ct->phase = (unsigned short)(level + 7);
  ct->on = level;
  ct->cnt = level ^ 0x55;
  ct->gain = (short)(level * 3);
}
void GenerateCallingTone(CallingTone *ct, short *out, int n)
{
  int i;
  emit(0, "GCT %s n=%d phase=%u on=%d cnt=%d", (char *)ct - (char *)g_cp == (long)offsetof(CallProg, ct) ? "ct" : "?", n, ct->phase, ct->on, ct->cnt);
  for (i = 0; i < n; i++) out[i] = (short)(ct->phase + 11 * i);
  ct->phase = (unsigned short)(ct->phase + n);
}
int DialerCreate(Dialer *d, const char *s, struct modem *m)
{
  int r = g_dres_create >= 0 ? g_dres_create : (rpct(60) ? 0 : rpct(50) ? 7 : rrange(-2, 9));
  emit(0, "DLC %s s=%s%s%s m=%d -> %d", (char *)d - (char *)g_cp == (long)offsetof(CallProg, dialer) ? "dialer" : "?", s ? "\"" : "", s ? s : "NULL", s ? "\"" : "", MID(m), r);
  d->state = 0x100u + (unsigned)r;
  d->cur = -1;
  return r;
}
void DialerAbort(Dialer *d)
{
  emit(0, "DLA %s", (char *)d - (char *)g_cp == (long)offsetof(CallProg, dialer) ? "dialer" : "?");
  d->state = 0xabu;
}
int DialerProgress(Dialer *d, short *buf, int *pos, int last)
{
  int r = pick_w(g_dialw, 10), i, upto;
  g_dialer_calls++;
  if (r == 4) {
    if (++g_prog4run > 5) r = 0;
  } else {
    g_prog4run = 0;
  }
  cov_hit(CV_DLR0 + (r > 9 ? 9 : r));
  if (r == 9) r = -1 - (int)rmod(3);
  else if (r == 8) r = 8 + (int)rmod(5);
  upto = *pos;
  if (last >= *pos && rpct(80)) upto = *pos + (int)rmod((unsigned)(last - *pos + 2)); /* produced samples: [pos .. upto - 1] */
  if (upto > last + 1) upto = last + 1;
  for (i = *pos; i < upto; i++) buf[i] = (short)(0x100 + i * 5 + d->cur);
  emit(0, "DLP %s buf=%s pos=%d last=%d -> pos=%d r=%d", (char *)d - (char *)g_cp == (long)offsetof(CallProg, dialer) ? "dialer" : "?", "out", *pos, last, upto, r);
  *pos = upto;
  d->cur++;
  return r;
}
static int g_sregv[256];
static int my_sreg(struct modem *m, int reg)
{
  int v = g_sregv[reg & 255];
  emit(0, "GSR m%d %d -> %d", MID(m), reg, v);
  return v;
}

/* ------------------------------------------------------------------------------------------------ object description */
static u64 hash_dialer(const Dialer *d)
{
  u64 h = fnv_bytes(d->str, sizeof d->str);
  const DialerCfg *c = &d->cfg;
  h = fnv_mix(h, (unsigned)c->toneDigitLen); h = fnv_mix(h, (unsigned)c->toneBetween);
  h = fnv_mix(h, (unsigned)(unsigned short)c->gain1); h = fnv_mix(h, (unsigned)(unsigned short)c->gain2);
  h = fnv_mix(h, (unsigned)c->pulseOnHookTime); h = fnv_mix(h, (unsigned)c->pulseOffHookTime); h = fnv_mix(h, (unsigned)c->pulseBetween);
  h = fnv_mix(h, (unsigned)c->dialPause); h = fnv_mix(h, (unsigned)c->flashTime); h = fnv_mix(h, (unsigned)c->toneOrPulse);
  h = fnv_mix(h, (unsigned)c->modValid); h = fnv_mix(h, (unsigned)c->abcd); h = fnv_mix(h, (unsigned)c->pulseAndTone);
  h = fnv_mix(h, (unsigned)c->callingTone); h = fnv_mix(h, (unsigned)c->commaLimit); h = fnv_mix(h, (unsigned)c->digitPattern);
  h = fnv_mix(h, (unsigned)d->lastIdx); h = fnv_mix(h, (unsigned)d->syntax); h = fnv_mix(h, (unsigned)d->commaCount); h = fnv_mix(h, (unsigned)d->cur);
  h = fnv_mix(h, (unsigned)d->row); h = fnv_mix(h, (unsigned)d->col); h = fnv_mix(h, d->state); h = fnv_mix(h, (unsigned)d->left);
  h = fnv_mix(h, d->ph1); h = fnv_mix(h, d->ph2); h = fnv_mix(h, d->inc1); h = fnv_mix(h, d->inc2);
  h = fnv_mix(h, (unsigned)d->digitSent); h = fnv_mix(h, (unsigned)d->pulseActive); h = fnv_mix(h, (unsigned)d->pulseDone);
  return h;
}
static u64 hash_cp(void)
{
  const CallProg *cp = g_cp;
  char b[16];
  u64 h = 1469598103934665603ULL;
  int i;
  if (!cp) return 0;
  for (i = 0; i < 7; i++) h = fnv_mix(h, (unsigned)cp->cfg[i]);
  h = fnv_mix(h, (unsigned)cp->blind);
  h = fnv_mix(h, cp->getSReg == my_sreg ? 1u : cp->getSReg == NULL ? 0u : 2u);
  h = fnv_mix(h, cp->modem == FAKEM ? 1u : cp->modem == NULL ? 0u : cp->modem == FAKEM2 ? 3u : 2u);
  h = fnv_mix(h, (unsigned)cp->w28); h = fnv_mix(h, (unsigned)cp->state); h = fnv_mix(h, (unsigned)cp->prevState); h = fnv_mix(h, (unsigned)cp->timer);
  h = fnv_mix(h, (unsigned)cp->nextState); h = fnv_mix(h, (unsigned)cp->change); h = fnv_mix(h, (unsigned)cp->lastRet); h = fnv_mix(h, (unsigned)cp->lcCount);
  h = fnv_mix(h, (unsigned)cp->lcLimit); h = fnv_mix(h, (unsigned)cp->lcEnable); h = fnv_mix(h, (unsigned)cp->event); h = fnv_mix(h, (unsigned)cp->silence);
  h = fnv_mix(h, (unsigned)cp->dialRes); h = fnv_mix(h, (unsigned)cp->callTone); h = fnv_mix(h, (unsigned)cp->callToneOn);
  h = fnv_mix(h, (unsigned)cp->w68); h = fnv_mix(h, (unsigned)cp->w70); h = fnv_mix(h, (unsigned)cp->w74);
  h = fnv_mix(h, (unsigned)cp->applyFilter); h = fnv_mix(h, cp->dialSeen);
  h = fnv_bytes(dh(cp->dialCad, b), strlen(dh(cp->dialCad, b))) ^ (h * 31u);
  h = fnv_bytes(dh(cp->busyCad, b), strlen(dh(cp->busyCad, b))) ^ (h * 31u);
  h = fnv_bytes(dh(cp->iir, b), strlen(dh(cp->iir, b))) ^ (h * 31u);
  h = fnv_bytes(dh(cp->dual, b), strlen(dh(cp->dual, b))) ^ (h * 31u);
  h = fnv_mix(h, cp->ct.phase); h = fnv_mix(h, (unsigned)cp->ct.on); h = fnv_mix(h, (unsigned)cp->ct.cnt); h = fnv_mix(h, (unsigned)(unsigned short)cp->ct.gain);
  h ^= hash_dialer(&cp->dialer) * 1099511628211ULL;
  return h;
}
static void dirty(void *p, size_t n)
{
  unsigned char *b = (unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) b[i] = (unsigned char)rnd();
}

/* every member individually, in declaration order, a FIXED number of draws: the object is bigger on x86-64 (pointers) but the input stream must not depend on that */
static void dirty_dialer(Dialer *d)
{
  int i;
  for (i = 0; i < 0x64; i++) d->str[i] = (char)rnd();
  d->cfg.toneDigitLen = (int)rnd(); d->cfg.toneBetween = (int)rnd(); d->cfg.gain1 = (short)rnd(); d->cfg.gain2 = (short)rnd();
  d->cfg.pulseOnHookTime = (int)rnd(); d->cfg.pulseOffHookTime = (int)rnd(); d->cfg.pulseBetween = (int)rnd(); d->cfg.dialPause = (int)rnd();
  d->cfg.flashTime = (int)rnd(); d->cfg.toneOrPulse = (int)rnd(); d->cfg.modValid = (int)rnd(); d->cfg.abcd = (int)rnd();
  d->cfg.pulseAndTone = (int)rnd(); d->cfg.callingTone = (int)rnd(); d->cfg.commaLimit = (int)rnd(); d->cfg.digitPattern = (int)rnd();
  d->lastIdx = (int)rnd(); d->syntax = (int)rnd(); d->commaCount = (int)rnd(); d->cur = (int)rnd(); d->row = (int)rnd(); d->col = (int)rnd();
  d->state = rnd(); d->left = (int)rnd(); d->ph1 = (unsigned short)rnd(); d->ph2 = (unsigned short)rnd(); d->inc1 = (unsigned short)rnd(); d->inc2 = (unsigned short)rnd();
  d->digitSent = (int)rnd(); d->pulseActive = (int)rnd(); d->pulseDone = (int)rnd();
  d->modem = (rnd() & 1) ? FAKEM : NULL;
}
static void dirty_cp(CallProg *cp)
{
  int i;
  for (i = 0; i < 7; i++) cp->cfg[i] = (int)rnd();
  cp->blind = (int)rnd();
  cp->getSReg = (rnd() & 1) ? my_sreg : NULL;
  cp->modem = (rnd() & 1) ? FAKEM : NULL;
  cp->w28 = (int)rnd(); cp->state = (int)rnd(); cp->prevState = (int)rnd(); cp->timer = (int)rnd(); cp->nextState = (int)rnd(); cp->change = (int)rnd();
  cp->lastRet = (int)rnd(); cp->lcCount = (int)rnd(); cp->lcLimit = (int)rnd(); cp->lcEnable = (int)rnd(); cp->event = (int)rnd(); cp->silence = (int)rnd();
  cp->dialRes = (int)rnd(); cp->callTone = (int)rnd(); cp->callToneOn = (int)rnd();
  cp->dialCad = (rnd() & 1) ? (Cadence *)&g_pool[0][0] : NULL;
  cp->w68 = (int)rnd();
  cp->busyCad = (rnd() & 1) ? (Cadence *)&g_pool[0][0] : NULL;
  cp->w70 = (int)rnd(); cp->w74 = (int)rnd();
  cp->iir = (rnd() & 1) ? (IirFilter *)&g_pool[1][0] : NULL;
  cp->applyFilter = (int)rnd(); cp->dialSeen = (unsigned char)rnd();
  cp->dual = (rnd() & 1) ? (DualTone *)&g_pool[2][0] : NULL;
  cp->ct.phase = (unsigned short)rnd(); cp->ct.on = (int)rnd(); cp->ct.cnt = (int)rnd(); cp->ct.gain = (short)rnd();
  dirty_dialer(&cp->dialer);
}

/* ------------------------------------------------------------------------------------------------ coverage */
static const char *const k_dbg[] = {
    "CALLPROG Create <<", "CallProgFP_Create >>", "APPLY_FILTER = ", "CALLPROG_Delete is entered", "CALLPROG_Delete is exited", "cadence_delete with CADENCE_DIAL_OBJ is invoked",
    "cadence_delete with CADENCE_OBJ is invoked", "CALLPROG Dialing ", "sreg function is not defined!", "GetNoAnswerTimeOut. ", "WAIT DIAL TIMEOUT = ", "BlindCall: GetBlindDialPause = ",
    "Set state to CALLPROG_DIALING_STATE", "CALLPROG_Dial was exited.", "Invalid buffer length", "DIALER_ERROR_MSG encountered in Callprog_progress.", "tone detected by cadence dial",
    "busy detected by cadence", "no answer detected by cadence", "CALLPROG: Time out", "CALLPROG: LINE CLEAR TIMEOUT", "Found 2100", "Found 2250", "CALLPROG: 5 sec. silence was detected.",
    "CALLPROG: Wait dial tone, reset the cadence", "CALLPROG: @ encountered.", "CALLPROG: $ encountered.", "CALLPROG: ^ encountered.", "CALLPROG: ^ encountered. Disabling Calling-Tone.",
    "CALLPROG: ^ encountered. Enabling Calling-Tone.", "CALLPROG: Dialing string partially ended.", "CALLPROG: Dialing string error.", "STATE:  CALLPROG_WAIT_DIAL --> CALLPROG_DIALING",
    "STATE:  CALLPROG_DIALING --> CALLPROG_END", "STATE:  CALLPROG_WAIT_TO_ANSWER --> CALLPROG_ANSWER_STATE",
    "STATE:  CALLPROG_WFS_STATE --> CALLPROG_DIALING", "STATE:  CALLPROG_DIALING --> CALLPROG_END_PARTIALLY_STATE", "STATE:  CALLPROG_DIALING --> CALLPROG_WFS_STATE",
    "STATE:  CALLPROG_DIALING --> CALLPROG_BONGTONE_STATE", "STATE:  CALLPROG_DIALING --> CALLPROG_WAIT_RING"};
#define NDBG ((int)(sizeof k_dbg / sizeof k_dbg[0]))
static int g_covscen;
static void dbg_hook(const char *msg)
{
  int i;
  for (i = 0; i < NDBG; i++)
    if (strcmp(msg, k_dbg[i]) == 0 || (k_dbg[i][strlen(k_dbg[i]) - 1] == ' ' && strncmp(msg, k_dbg[i], strlen(k_dbg[i])) == 0)) {
      cov_hit(CV_DBG0 + i);
      return;
    }
}
static const int k_rets[] = {0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 16, 17};
static int dbg_scen(int i) { return i <= 2 ? 0 : i <= 6 ? 4 : i <= 13 ? 1 : 2; } /* the scenario that must produce debug text number i */
static void cov_setup(int sc)
{
  int i;
  g_covscen = sc;
  for (i = 0; i < NDBG; i++) {
    static char names[64][120];
    snprintf(names[i], sizeof names[i], "debug text: %s", k_dbg[i]);
    if (dbg_scen(i) == sc) cov_def(CV_DBG0 + i, names[i]);
  }
  if (sc == 2) {
    static char sn[10][40], rn[20][40], dn[12][40];
    for (i = 1; i <= 9; i++) {
      snprintf(sn[i], sizeof sn[i], "Progress entered in state %d", i);
      cov_def(CV_STATE0 + i, sn[i]);
    }
    for (i = 0; i < (int)(sizeof k_rets / sizeof k_rets[0]); i++) {
      snprintf(rn[i], sizeof rn[i], "Progress result %d", k_rets[i]);
      cov_def(CV_RET0 + i, rn[i]);
    }
    for (i = 0; i <= 9; i++) {
      snprintf(dn[i], sizeof dn[i], "DialerProgress result %s%d", i == 8 ? ">= " : "", i);
      cov_def(CV_DLR0 + i, dn[i]);
    }
  }
}
static void note_ret(int r)
{
  int i;
  for (i = 0; i < (int)(sizeof k_rets / sizeof k_rets[0]); i++)
    if (k_rets[i] == r) cov_hit(CV_RET0 + i);
}

/* ------------------------------------------------------------------------------------------------ drivers */
static CallProg g_obj;
static CallProgCfg mkcfg(void)
{
  CallProgCfg c;
  c.blind = rpct(30) ? (int)rnd() : 0;
  c.getSReg = rpct(85) ? my_sreg : NULL;
  c.modem = rpct(85) ? FAKEM : FAKEM2;
  c.w0c = (int)rnd();
  return c;
}
static void defaults(void)
{
  int i;
  memset(g_prm, 0, sizeof g_prm);
  for (i = 0; i < 256; i++) g_sregv[i] = (int)(rnd() | (rpct(50) ? 0x7f00u : 0u));
  g_prm[0x3b] = rpct(50) ? 1 : 0;
  g_prm[0x1b] = rmod(5);
  g_prm[0x2d] = rrange(0, 20);
  g_prm[0x22] = rpct(60) ? rrange(0, 60) : rrange(-50, 400);
  g_prm[0x1d] = rrange(0, 6);
  g_prm[0x1e] = rrange(1, 3);
}
static void reset_pools(void)
{
  memset(g_npool, 0, sizeof g_npool);
  g_ncad_create = g_ndual_create = 0;
  g_cad_fail_mask = g_dual_fail_mask = 0;
}
static void setw(void)
{
  int i;
  for (i = 0; i < 10; i++) g_dialw[i] = 0;
  for (i = 0; i < 6; i++) g_dualw[i] = 0;
  g_prog4run = 0;
}
static void std_weights(int heavy)
{
  g_dialw[0] = 50; g_dialw[1] = 3; g_dialw[2] = 2; g_dialw[3] = 2; g_dialw[4] = heavy ? 6 : 2; g_dialw[5] = 2; g_dialw[6] = 2; g_dialw[7] = 2; g_dialw[8] = 1; g_dialw[9] = 1;
  g_dualw[0] = 80; g_dualw[1] = 4; g_dualw[2] = 4; g_dualw[3] = 5; g_dualw[4] = 4; g_dualw[5] = 5;
}
static unsigned g_lvl;
static int g_burst, g_alt;
static int g_forcec = -1; /* kinds 5 / 6: the exact level (silence detector scan) */
static void sample_buf(short *b, int n, int kind)
{
  int i;
  for (i = 0; i < n; i++) {
    switch (kind) {
    case 0: b[i] = 0; break;
    case 1: b[i] = (short)((int)rmod(7) - 3); break;
    case 2: b[i] = (short)rnd(); break;
    case 3: b[i] = (short)(((i * 977) & 0x3fff) - 0x2000); break;
    case 7: b[i] = (short)(i < g_burst ? ((i & g_alt) ? -g_forcec : g_forcec) : 0); break; /* a burst of g_burst loud samples, then silence: the decay lands on the threshold */
    case 5: b[i] = (short)(g_forcec >= 0 ? g_forcec : 60 + (int)(g_lvl % 160u)); break; /* a constant level around the silence detector's threshold */
    case 6: { int c = g_forcec >= 0 ? g_forcec : 60 + (int)(g_lvl % 160u); b[i] = (short)((i & 1) ? c : -c); } break;
    default: b[i] = (short)((int)rmod(401) - 200); break;
    }
  }
}

static void do_progress(int n, int kind, const char *tag)
{
  short in[0xa8 + 8], ref[0xa8 + 8], out[0xa8 + 8], outref[0xa8 + 8];
  int r, i;
  g_lvl = rnd();
  sample_buf(in, 0xa0 + 8, kind);
  memcpy(ref, in, sizeof in);
  for (i = 0; i < 0xa8 + 8; i++) out[i] = (short)(0x5a5a + i);
  memcpy(outref, out, sizeof out);
  cov_hit(CV_STATE0 + (g_cp->state >= 0 && g_cp->state <= 9 ? g_cp->state : 0));
  r = CALLPROG_Progress(g_cp, in, out, n);
  if (memcmp(in, ref, sizeof in) != 0) die3("Progress modified its input");
  if (n >= 0 && n <= 0xa0 && memcmp(&out[n], &outref[n], (size_t)(0xa8 + 8 - n) * sizeof(short)) != 0) die3("Progress wrote past n=%d", n);
  note_ret(r);
  emit(0, "%s n=%d kind=%d -> %d out=%016llx cp=%016llx", tag, n, kind, r, fnv_bytes(out, (size_t)(n > 0 && n <= 0xa0 ? n : 0) * 2), hash_cp());
}
static int pick_n(void)
{
  static const int sp[] = {0, 1, 2, 3, 7, 16, 47, 48, 96, 159, 160, 0xa0, 0xa0, 40, 24, 8, 80, 13, 17, 160, 160, 13, 17};
  if (rpct(4)) return 0xa1 + (int)rmod(5);
  if (rpct(6)) return (int)rmod(0xa1);
  return sp[rmod(sizeof sp / sizeof sp[0])];
}
static void rand_obj(void)
{
  CallProg *cp = g_cp;
  static const int tm[] = {0, 0, 1, 2, 3, -10, -1, 5, 1000000, 80000, 100, 159, 160, 161, 7};
  dirty_cp(cp);
  cp->state = rpct(3) ? (int)rmod(10) : rrange(1, 9);
  cp->prevState = rpct(40) ? cp->state : (int)rmod(10);
  cp->timer = tm[rmod(sizeof tm / sizeof tm[0])];
  cp->nextState = rrange(0, 9);
  cp->change = rpct(15) ? 1 : (rpct(5) ? (int)rnd() : 0);
  cp->lastRet = rpct(70) ? rrange(0, 17) : (int)rnd();
  cp->lcEnable = rpct(40) ? 1 : (rpct(10) ? 2 : 0);
  cp->lcLimit = rpct(50) ? rrange(0, 400) : (int)rnd() % 1000;
  cp->lcCount = rpct(60) ? cp->lcLimit - rrange(0, 3) : rrange(0, 500);
  cp->event = rrange(0, 8);
  cp->silence = rpct(50) ? 40000 / (rpct(50) ? 160 : (int)(1 + rmod(160))) - rrange(0, 2) : rrange(0, 300);
  cp->dialRes = rpct(10) ? 7 : rpct(60) ? 0 : rrange(0, 9);
  cp->callTone = rrange(0, 5);
  cp->callToneOn = rpct(50);
  cp->applyFilter = rpct(40) ? 1 : (rpct(10) ? (int)rnd() : 0);
  cp->dialSeen = (unsigned char)(rpct(25) ? 1 : rpct(5) ? rnd() : 0);
  cp->dialCad = rpct(95) ? (Cadence *)newh(0) : NULL;
  cp->busyCad = rpct(95) ? (Cadence *)newh(0) : NULL;
  cp->iir = rpct(95) ? (IirFilter *)newh(1) : NULL;
  cp->dual = rpct(95) ? (DualTone *)newh(2) : NULL;
  cp->getSReg = my_sreg;
  cp->modem = FAKEM;
}

static void scen0(void)
{
  int t;
  for (t = 0; t < 160; t++) {
    CallProgCfg cfg;
    reset_pools();
    setw();
    defaults();
    g_cp = &g_obj;
    dirty_cp(&g_obj);
    dsplibs_debug_level = lvl_value((int)rmod(8));
    cfg = mkcfg();
    g_prm[0x3b] = rpct(60) ? (rpct(50) ? 0 : 1) : (long)(int)rnd();
    g_cad_fail_mask = (unsigned)(rpct(25) ? rrange(1, 3) : 0);
    g_dual_fail_mask = rpct(15) ? 1u : 0u;
    CALLPROG_Create(g_cp, &cfg);
    emit(0, "CREATE t=%d lvl=%u cp=%016llx", t, dsplibs_debug_level, hash_cp());
    if (rpct(50)) { /* probe the tables: a few Progress calls on the fresh object */
      int k;
      std_weights(0);
      dsplibs_debug_level = 0;
      for (k = 0; k < 20; k++) do_progress(pick_n(), rrange(0, 4), "PRC");
    }
  }
  emit(1, "scenario 0 done");
}
static void scen1(void)
{
  int t;
  static const char *const strs[] = {"", "5551212", "T5551212", "p,,,5551212;", "W@^$", "123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890", NULL};
  for (t = 0; t < 400; t++) {
    CallProgCfg cfg;
    const char *s;
    reset_pools();
    setw();
    defaults();
    g_cp = &g_obj;
    dsplibs_debug_level = 0;
    cfg = mkcfg();
    CALLPROG_Create(g_cp, &cfg);
    dsplibs_debug_level = lvl_value((int)rmod(8));
    g_cp->state = rpct(20) ? 7 : (int)rmod(10);
    g_cp->blind = rpct(40);
    if (rpct(12)) g_cp->getSReg = NULL;
    g_cp->cfg[0] = rrange(0, 12);
    g_cp->cfg[6] = rrange(0, 70);
    g_prm[0x1b] = rpct(70) ? (long)rmod(5) : (long)(int)rnd();
    g_prm[0x2d] = rpct(70) ? rrange(0, 20) : (long)(int)rnd();
    g_prm[0x22] = rpct(30) ? rrange(0, 40) : rpct(50) ? rrange(0, 400) : (long)(int)rnd();
    if (rpct(10)) g_prm[0x22] = 20 + (long)rmod(4);   /* (x + 9) / 10 around 2 / 3 */
    g_prm[0x1d] = rpct(70) ? rrange(0, 30) : (long)(int)rnd();
    g_prm[0x1e] = rpct(70) ? rrange(0, 30) : (long)(int)rnd();
    g_dres_create = rpct(50) ? 0 : rpct(50) ? 7 : -1;
    s = strs[rmod(sizeof strs / sizeof strs[0])];
    g_ncad_create = 0;
    if (rpct(50)) { /* a first Dial of the other kind: the tables the second one must put right again */
      int keep = g_cp->blind;
      g_cp->blind = !keep;
      g_cp->state = rpct(50) ? 7 : 1;
      CALLPROG_Dial(g_cp, "1");
      g_cp->blind = keep;
      g_cp->state = rpct(20) ? 7 : (int)rmod(10);
    }
    g_cp->lcEnable = (int)rnd();
    g_cp->silence = (int)rnd();
    g_flip22 = rpct(25);
    CALLPROG_Dial(g_cp, s);
    g_flip22 = 0;
    emit(0, "DIAL t=%d lvl=%u cp=%016llx", t, dsplibs_debug_level, hash_cp());
    {
      int k;
      std_weights(0);
      dsplibs_debug_level = 0;
      for (k = 0; k < 24; k++) do_progress(pick_n(), rrange(0, 4), "PRD");
    }
  }
  emit(1, "scenario 1 done");
}
static void scen2(void)
{
  int t;
  CallProgCfg cfg;
  for (t = 0; t < 30000; t++) {
    if (t % 500 == 0) {
      /* reconfigure the shared tables: a fresh Create, optionally a Dial (blind or not) */
      reset_pools();
      g_cp = &g_obj;
      defaults();
      dsplibs_debug_level = 0;
      cfg = mkcfg();
      cfg.getSReg = my_sreg;
      CALLPROG_Create(g_cp, &cfg);
      if (rpct(60)) {
        g_cp->blind = rpct(50);
        g_dres_create = 0;
        CALLPROG_Dial(g_cp, "5551212");
      }
      /* each block of 500 calls has its own script flavour */
      setw();
      std_weights(rpct(30));
      g_pc1 = rpct(50) ? 40 : 2;
      g_pc7 = rpct(50) ? 40 : 1;
      if (rpct(25)) { g_dialw[0] = 0; g_dialw[rrange(1, 7)] = 40; }
    }
    if (g_npool[0] + 4 > 250 || g_npool[1] + 4 > 250 || g_npool[2] + 4 > 250) reset_pools(), g_ncad_create = 0;
    g_cp = &g_obj;
    rand_obj();
    dsplibs_debug_level = lvl_value((int)rmod(8));
    if (rpct(5)) dsplibs_debug_level = 2;
    {
      int n = pick_n();
      if (g_cp->state == 8 && n > 0 && rpct(60)) g_cp->silence = 40000 / n - rrange(0, 2);
      do_progress(n, rpct(35) ? rrange(5, 6) : rrange(0, 4), "PRG");
    }
  }
  /* the silence detector (state 8) at constant levels 0 .. 12000 (every level near the thresholds where acc reaches 99 / 100 after 1, 2, 3 .. samples; alternating sign too): the threshold (acc > 99) and the filter constants.  The smoothed value is acc = |x| * 0.01 + 0.99 acc (floors: a level below 100 stays at 0) */
  {
    static const int ns[] = {160, 1, 2, 3, 4, 5, 6, 8, 13, 17, 40, 100};
    int x, k, kd;
    reset_pools();
    g_cp = &g_obj;
    dsplibs_debug_level = 0;
    for (x = 0; x <= 12000; x++)
      for (k = 0; k < (int)(sizeof ns / sizeof ns[0]); k++)
        for (kd = 5; kd <= 6; kd++) {
          if (g_npool[0] + 4 > 250) reset_pools();
          rand_obj();
          g_cp->state = 8;
          g_cp->dialRes = 0;
          g_cp->silence = 40000 / ns[k] - 1;
          g_cp->change = 0;
          g_forcec = x;
          do_progress(ns[k], kd, "PRZ");
          g_forcec = -1;
        }
  }
  {
    int t2;
    reset_pools();
    for (t2 = 0; t2 < 60000; t2++) {
      int n = rpct(70) ? 160 : (int)(rmod(160) + 1);
      if (g_npool[0] + 4 > 250) reset_pools();
      rand_obj();
      g_cp->state = 8;
      g_cp->dialRes = 0;
      g_cp->silence = 40000 / n - 1;
      g_cp->change = 0;
      g_burst = (int)rmod((unsigned)n + 1u);
      g_alt = rpct(50) ? 1 : 0;
      g_forcec = rpct(30) ? (int)rmod(32768) : rpct(60) ? (int)rmod(600) : (int)rmod(12000);
      do_progress(n, 7, "PRB");
      g_forcec = -1;
    }
  }
  emit(1, "scenario 2 done");
}
static void scen3(void)
{
  int t;
  for (t = 0; t < 200; t++) {
    CallProgCfg cfg;
    int k, calls = 300;
    reset_pools();
    setw();
    defaults();
    g_cp = &g_obj;
    dsplibs_debug_level = lvl_value((int)rmod(8));
    if (rpct(70)) dsplibs_debug_level = 0;
    cfg = mkcfg();
    cfg.getSReg = my_sreg;
    cfg.modem = FAKEM;
    CALLPROG_Create(g_cp, &cfg);
    g_prm[0x1e] = rrange(1, 2);
    g_dres_create = rpct(85) ? 0 : 7;
    g_cp->blind = rpct(40);
    CALLPROG_Dial(g_cp, "5551212");
    std_weights(0);
    g_dialw[0] = 400;
    g_dualw[0] = 600;
    g_pc1 = rpct(50) ? 1 : 0;
    g_pc7 = rpct(50) ? 1 : 0;
    if (rpct(30)) g_cp->cfg[0] = 0, g_cp->timer = rrange(0, 5000);
    if (rpct(30)) {
      g_cp->lcLimit = rrange(100, 3000);
    }
    for (k = 0; k < calls; k++) {
      int n = rpct(70) ? 160 : pick_n();
      if (k % 40 == 39) g_cp->timer = rrange(0, 600);
      do_progress(n, rrange(0, 4), "PRS");
    }
    emit(0, "SESSION t=%d cp=%016llx", t, hash_cp());
    CALLPROG_Delete(g_cp);
    emit(0, "SESSION-DEL cp=%016llx", hash_cp());
    if (rpct(10)) CALLPROG_Dial(g_cp, "");
  }
  emit(1, "scenario 3 done");
}
static void scen4(void)
{
  int t, id;
  for (t = 0; t < 120; t++) {
    CallProgCfg cfg;
    reset_pools();
    setw();
    defaults();
    g_cp = &g_obj;
    dsplibs_debug_level = 0;
    cfg = mkcfg();
    dirty_cp(g_cp);
    g_prm[0x3b] = (t & 1) ? 1 : 0;
    g_cad_fail_mask = (unsigned)((t >> 1) & 3);
    g_dual_fail_mask = (unsigned)((t >> 3) & 1);
    CALLPROG_Create(g_cp, &cfg);
    dsplibs_debug_level = lvl_value(t >> 4);
    if (t & 64) g_cp->applyFilter = (int)rnd();
    CALLPROG_Delete(g_cp);
    emit(0, "DELETE t=%d cp=%016llx", t, hash_cp());
  }
  for (id = -5; id <= 40; id++) emit(0, "STATUS %d \"%s\"", id, CALLPROG_Status_string(id));
  emit(1, "scenario 4 done");
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R11_SEED");
  g_verbose = getenv("R11_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)atoi(sd) * 2654435761u;
  g_lcg += (unsigned)sc * 0x9e3779b9u;
  g_snap = hash_cp;
  g_dbg_hook = dbg_hook;
  cov_setup(sc);
  switch (sc) {
  case 0: scen0(); break;
  case 1: scen1(); break;
  case 2: scen2(); break;
  case 3: scen3(); break;
  default: scen4(); break;
  }
  trial_end("end");
  cov_finish();
  return 0;
}
