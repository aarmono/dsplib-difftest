/* R30 harness part 4 (HARNESS_ENG build): scenarios 6 .. 8.  NOTHING is replaced: two FAX objects (FAX_create) over the real class 1 layer, the real R28 / R29 state machines, the real FAXVMI
 * layer and the real V.21 / V.27ter / V.29 / V.17 modems and resamplers of both sides (the blob's own vs the ported ones) exchange audio blocks back to back through FAX_process, driven by
 * a scripted DTE (AT+F* commands through FAX_class1_command, DLE stuffed frame / data streams through modem_recv_from_tty, the bytes the objects deliver through modem_send_to_tty).
 * Scenarios: 6 HDLC sessions (frames both ways, every rate / role / impairment), 7 data sessions (every modem value, short train, silence commands), 8 life cycles (create / delete in
 * the middle of a session, random command sequences, rejected commands).  Included by fax_api.main.c. */
#ifdef HARNESS_ENG

#ifndef TEST_BLOB
/* the port runs link src/v32/v32_datapath.c (the ported VTB_decoder behind FAXVTB_decoder); the symbols that file references but this test never reaches are abort stubs */
static void unreachable_stub(const char *n)
{
  fprintf(stderr, "r30: unexpected call of %s\n", n);
  abort();
}
void FPM_ECC_cancel(void) { unreachable_stub("FPM_ECC_cancel"); }
void FPM_ECC_init(void) { unreachable_stub("FPM_ECC_init"); }
void FPM_FSE_receive(void) { unreachable_stub("FPM_FSE_receive"); }
void FPM_PPS_filter(void) { unreachable_stub("FPM_PPS_filter"); }
void FPM_SRE_recover(void) { unreachable_stub("FPM_SRE_recover"); }
void FSE_decision_128pt(void) { unreachable_stub("FSE_decision_128pt"); }
void FSE_decision_16pt(void) { unreachable_stub("FSE_decision_16pt"); }
void FSE_decision_16Tpt(void) { unreachable_stub("FSE_decision_16Tpt"); }
void FSE_decision_32pt(void) { unreachable_stub("FSE_decision_32pt"); }
void FSE_decision_4pt(void) { unreachable_stub("FSE_decision_4pt"); }
void FSE_decision_64pt(void) { unreachable_stub("FSE_decision_64pt"); }
void FSE_decision_AB(void) { unreachable_stub("FSE_decision_AB"); }
void SDMv32_descrambler(void) { unreachable_stub("SDMv32_descrambler"); }
void SDMv32_scrambler(void) { unreachable_stub("SDMv32_scrambler"); }
#endif

typedef struct {
  struct modem *m;
  FaxGlue *g;
  int mid;      /* 1 / 2: the index of the tty queue */
  int caller;   /* FAX_create argument */
  unsigned blk; /* samples per block */
  /* the DTE program */
  int nstep, pc;
  struct {
    int cmd, param;      /* FAX_class1_command arguments */
    int tx;              /* 1: a transmit step: the stream is queued at CONNECT */
    unsigned len;        /* stream length in frames (HDLC) or bytes (data) */
    int hdlc;            /* stream kind */
    int wait;            /* blocks to wait before issuing the command */
    int wait_status;     /* wait for the first non zero status before issuing the command */
    int maxblk;          /* blocks the step may last */
  } step[8];
  int active, since, loaded, waited;
  int first_status_seen;
  unsigned long nstatus[8];
  int last_r;
  short out[1024], in[1024], prev_out[1024];
} Side;
static Side g_A, g_B;
static int g_state_seen[19];
static unsigned long g_nblocks;

/* DLE stuffed streams: HDLC frames [ff 03 / 13 fcf ...] each followed by DLE ETX, or `len` data bytes followed by DLE ETX */
static unsigned stream_hdlc(unsigned char *p, unsigned nframes, int final_last)
{
  unsigned n = 0, f;
  for (f = 0; f < nframes; f++) {
    unsigned len, k;
    unsigned char ctl, fcf;
    len = 1 + rmod(14);
    ctl = (f == nframes - 1 && final_last) ? 0x13 : 0x03;
    fcf = (unsigned char)(0x80 + rmod(8));
    p[n++] = 0xff;
    p[n++] = ctl;
    p[n++] = fcf;
    for (k = 0; k < len; k++) {
      unsigned v;
      v = rnd();
      p[n++] = (unsigned char)v;
      if ((unsigned char)v == 0x10) p[n++] = 0x10;
    }
    p[n++] = 0x10;
    p[n++] = 0x03;
  }
  return n;
}
static unsigned stream_data(unsigned char *p, unsigned len)
{
  unsigned n = 0, k;
  for (k = 0; k < len; k++) {
    unsigned v;
    v = rnd();
    p[n++] = (unsigned char)v;
    if ((unsigned char)v == 0x10) p[n++] = 0x10;
  }
  p[n++] = 0x10;
  p[n++] = 0x03;
  return n;
}
static void queue_stream(Side *s, int hdlc, unsigned len)
{
  TtyQ *q = &g_q[s->mid];
  unsigned n = hdlc ? stream_hdlc(q->in, len, 1) : stream_data(q->in, len);
  q->in_n = n;
  q->in_pos = 0;
  s->loaded = 1;
  emit(0, "STREAM m%d hdlc=%d len=%u -> %u bytes %016llx", s->mid, hdlc, len, n, fnv_bytes(q->in, n));
}

static void side_init(Side *s, struct modem *m, int mid, int caller, unsigned rate)
{
  memset(s, 0, sizeof *s);
  s->m = m;
  s->mid = mid;
  s->caller = caller;
  s->g = (FaxGlue *)FAX_create(m, (unsigned)caller, rate);
  if (!s->g) die3("FAX_create failed");
  s->blk = s->g->blk;
  s->g->c1->modemClass = 0; /* the data modem class is never initialised by fax_class1_create and the debug lines print it: a fresh process heap holds zeros there (the pattern of this harness' heap depends on the host) */
  g_q[mid].in_n = g_q[mid].in_pos = 0;
  g_q[mid].out_n = 0;
  g_q[mid].out_total = 0;
}
static void add_step(Side *s, int cmd, int param, int tx, int hdlc, unsigned len, int wait, int wait_status, int maxblk)
{
  int i = s->nstep++;
  s->step[i].cmd = cmd;
  s->step[i].param = param;
  s->step[i].tx = tx;
  s->step[i].hdlc = hdlc;
  s->step[i].len = len;
  s->step[i].wait = wait;
  s->step[i].wait_status = wait_status;
  s->step[i].maxblk = maxblk;
}

/* one block of one side: issue the next command when due, run FAX_process, react to the status */
static int side_block(Side *s, const short *in, int blkno)
{
  int r;
  if (!s->active && s->pc < s->nstep) {
    int i = s->pc;
    if (s->waited < s->step[i].wait) s->waited++;
    else if (s->step[i].wait_status && !s->first_status_seen) {
    } else {
      int c = FAX_class1_command(s->g, (unsigned)s->step[i].cmd, (unsigned)s->step[i].param);
      emit(0, "CMD m%d blk=%d cmd=%d param=%d -> %d", s->mid, blkno, s->step[i].cmd, s->step[i].param, c);
      if (c == 1) {
        s->active = 1;
        s->since = 0;
        s->loaded = 0;
      } else {
        s->pc++;
        s->waited = 0;
      }
    }
  }
  memcpy(s->in, in, s->blk * sizeof(short));
  r = FAX_process(s->g, s->in, s->out, (int)s->blk);
  s->last_r = r;
  if (r >= 0 && r < 8) s->nstatus[r]++;
  if (r != 0) {
    s->first_status_seen = 1;
    emit(0, "STATUS m%d blk=%d r=%d", s->mid, blkno, r);
  }
  if (s->active) {
    int i = s->pc;
    s->since++;
    if (r == 3 && s->step[i].tx && !s->loaded) queue_stream(s, s->step[i].hdlc, s->step[i].len);
    if (r == 1 || r == 2 || r == 4 || (s->step[i].maxblk > 0 && s->since > s->step[i].maxblk)) {
      emit(0, "STEP m%d blk=%d done pc=%d r=%d since=%d", s->mid, blkno, s->pc, r, s->since);
      s->active = 0;
      s->pc++;
      s->waited = 0;
    }
  }
  return r;
}

/* one session: both sides run `nblk` blocks, the output of one is the input of the other (one block of delay), with an optional impairment */
static void run_session(const char *tag, int nblk, int noise, int gain_shift)
{
  int b;
  Side *A = &g_A, *B = &g_B;
  short ia[1024], ib[1024];
  unsigned n = A->blk;
  memset(A->prev_out, 0, sizeof A->prev_out);
  memset(B->prev_out, 0, sizeof B->prev_out);
  for (b = 0; b < nblk; b++) {
    unsigned k;
    int ra, rb;
    for (k = 0; k < n; k++) {
      int va = B->prev_out[k] >> gain_shift, vb = A->prev_out[k] >> gain_shift;
      if (noise) {
        unsigned r1, r2;
        r1 = rnd();
        r2 = rnd();
        va += (int)(r1 & (unsigned)noise) - (noise >> 1);
        vb += (int)(r2 & (unsigned)noise) - (noise >> 1);
      }
      ia[k] = (short)va;
      ib[k] = (short)vb;
    }
    ra = side_block(A, ia, b);
    rb = side_block(B, ib, b);
    memcpy(A->prev_out, A->out, n * sizeof(short));
    memcpy(B->prev_out, B->out, n * sizeof(short));
    g_nblocks++;
    emit(0, "BLK %s %d ra=%d rb=%d oa=%016llx ob=%016llx sa=%d sb=%d", tag, b, ra, rb, fnv_shorts(H0, A->out, n), fnv_shorts(H0, B->out, n), A->g->c1->state, B->g->c1->state);
    if (A->g->c1->state >= 0 && A->g->c1->state < 19) g_state_seen[A->g->c1->state]++;
    if (B->g->c1->state >= 0 && B->g->c1->state < 19) g_state_seen[B->g->c1->state]++;
    if ((b & 7) == 0 || ra != 0 || rb != 0) {
      emit(0, "SNAP %s %d ga=%016llx gb=%016llx ca=%016llx cb=%016llx", tag, b, snap_glue(A->g), snap_glue(B->g), h30(A->g->c1), h30(B->g->c1));
    }
    if (A->pc >= A->nstep && B->pc >= B->nstep && !A->active && !B->active) {
      emit(0, "SESSION %s done at block %d", tag, b);
      break;
    }
  }
  dump_t30("sa", A->g->c1);
  dump_t30("sb", B->g->c1);
  emit(0, "TTY a total=%lu out=%016llx / b total=%lu out=%016llx", g_q[A->mid].out_total, fnv_bytes(g_q[A->mid].out, g_q[A->mid].out_n), g_q[B->mid].out_total,
       fnv_bytes(g_q[B->mid].out, g_q[B->mid].out_n));
  if (g_q[A->mid].out_total || g_q[B->mid].out_total) cov_hit(CV_E6_BYTES);
}
static void sessions_cov(void)
{
  if (g_state_seen[4]) cov_hit(CV_E6_STATE4);
  if (g_state_seen[5]) cov_hit(CV_E6_STATE5);
  if (g_state_seen[14]) cov_hit(CV_E6_ANSWER);
  if (g_A.nstatus[3] + g_B.nstatus[3]) cov_hit(CV_E6_CONNECT);
  if (g_A.nstatus[1] + g_B.nstatus[1]) cov_hit(CV_E6_OK);
  if (g_A.nstatus[4] + g_B.nstatus[4] + g_A.nstatus[2] + g_B.nstatus[2]) cov_hit(CV_E6_NOCARRIER);
}
static void session_end(void)
{
  FAX_delete(g_A.g);
  FAX_delete(g_B.g);
  g_A.g = g_B.g = NULL;
  g_q[1].in_n = g_q[1].in_pos = g_q[2].in_n = g_q[2].in_pos = 0;
  trial_end("session");
  SETLVL(0);
}

/* ================================================================================================ scenario 6: HDLC sessions */
static void scen_eng_hdlc(void)
{
  int t;
  for (t = 0; t < 40; t++) {
    unsigned rate = (t % 3 == 2) ? 9600 : 8000;
    int v = t % 5, lv = (t % 7 == 6) ? 2 : 0, swap = (t / 5) & 1;
    int noise = (v == 3) ? 2047 : ((t % 11 == 10) ? 255 : 0), gain = (t % 6 == 5) ? 2 : 0;
    char tag[16];
    unsigned nf = 1 + rmod(3);
    SETLVL(lvl_value(lv));
    g_sreg[7] = 6 + (int)rmod(8);
    memset(g_state_seen, 0, sizeof g_state_seen);
    side_init(&g_A, FAKEM, 1, swap ? 0 : 1, rate);
    side_init(&g_B, FAKEM2, 2, swap ? 1 : 0, rate);
    cov_hit(CV_E6_SESS);
    if (rate == 8000) cov_hit(CV_E6_R8000);
    else cov_hit(CV_E6_R9600);
    if (noise) cov_hit(CV_E6_NOISE);
    snprintf(tag, sizeof tag, "h%d", t);
    emit(0, "SESSION %s rate=%u v=%d swap=%d noise=%d gain=%d nf=%u", tag, rate, v, swap, noise, gain, nf);
    switch (v) {
      case 0:
      case 3:
        add_step(&g_A, 4, 3, 1, 1, nf, 0, 0, 500);
        add_step(&g_A, 5, 3, 0, 1, 0, 0, 0, 500);
        add_step(&g_B, 5, 3, 0, 1, 0, 0, 0, 500);
        add_step(&g_B, 4, 3, 1, 1, 1 + nf / 2, 0, 0, 500);
        break;
      case 1:
        add_step(&g_A, 4, 3, 1, 1, nf, 0, 1, 500);
        add_step(&g_B, 5, 3, 0, 1, 0, 0, 1, 500);
        break;
      case 2:
        add_step(&g_A, 0, 100, 0, 0, 0, 0, 0, 200);
        add_step(&g_A, 4, 3, 1, 1, nf, 0, 0, 500);
        add_step(&g_B, 1, 40, 0, 0, 0, 0, 0, 200);
        add_step(&g_B, 5, 3, 0, 1, 0, 0, 0, 500);
        break;
      default:
        add_step(&g_A, 4, 3, 1, 1, nf, 3, 0, 500);
        add_step(&g_B, 5, 3, 0, 1, 0, 200, 0, 300);
        break;
    }
    run_session(tag, 700, noise, gain);
    sessions_cov();
    session_end();
  }
}

/* ================================================================================================ scenario 7: data sessions */
static void scen_eng_data(void)
{
  static const int mods[] = {0x18, 0x30, 0x48, 0x60, 0x49, 0x4a, 0x61, 0x62, 0x79, 0x7a, 0x91, 0x92};
  int t;
  for (t = 0; t < 36; t++) {
    int mod = mods[t % 12];
    unsigned rate = (t % 4 == 3) ? 9600 : 8000;
    int swap = (t / 12) & 1, lv = (t % 3 == 2) ? 2 : 0;
    static const unsigned bps[12] = {300, 600, 900, 1200, 900, 900, 1200, 1200, 1500, 1500, 1800, 1800};
    unsigned len = 40 + rmod(300);
    if (t % 3 == 2) len = bps[t % 12] * 3 / 2; /* longer than the 50 blocks the receiver's quality detector needs (status 10, ACCEPT_RATE) */
    char tag[16];
    SETLVL(lvl_value(lv));
    g_sreg[7] = 6 + (int)rmod(6);
    memset(g_state_seen, 0, sizeof g_state_seen);
    side_init(&g_A, FAKEM, 1, swap ? 0 : 1, rate);
    side_init(&g_B, FAKEM2, 2, swap ? 1 : 0, rate);
    cov_hit(CV_E7_SESS);
    if (mod == 0x18 || mod == 0x30) cov_hit(CV_E7_V27);
    else if (mod == 0x48 || mod == 0x60) cov_hit(CV_E7_V29);
    else cov_hit(CV_E7_V17);
    if (mod == 0x4a || mod == 0x62 || mod == 0x7a || mod == 0x92) cov_hit(CV_E7_SHORT);
    snprintf(tag, sizeof tag, "d%d", t);
    emit(0, "SESSION %s mod=%x rate=%u swap=%d len=%u", tag, mod, rate, swap, len);
    if (t % 6 == 5) {
      add_step(&g_A, 0, 60, 0, 0, 0, 0, 0, 200);   /* FTS: silence */
      add_step(&g_B, 1, 30, 0, 0, 0, 0, 0, 200);   /* FRS */
      cov_hit(CV_E7_TS);
    }
    add_step(&g_A, 2, mod, 1, 0, len, 0, 0, 900);
    add_step(&g_B, 3, mod, 0, 0, 0, 0, 0, 900);
    run_session(tag, len > 1000 ? 1500 : 1100, 0, 0);
    sessions_cov();
    if (g_A.nstatus[3] + g_B.nstatus[3]) cov_hit(CV_E7_CONNECT);
    if (g_q[1].out_total + g_q[2].out_total) cov_hit(CV_E7_BYTES);
    if (g_A.nstatus[1] + g_B.nstatus[1] + g_A.nstatus[2] + g_B.nstatus[2] + g_A.nstatus[4] + g_B.nstatus[4] + g_A.nstatus[5] + g_B.nstatus[5]) {
    }
    session_end();
  }
}

/* ================================================================================================ scenario 8: life cycles */
static void scen_eng_life(void)
{
  static const int mods[] = {0x18, 0x30, 0x48, 0x60, 0x49, 0x4a, 0x61, 0x62, 0x79, 0x7a, 0x91, 0x92};
  int t;
  for (t = 0; t < 40; t++) {
    unsigned rate = (t % 4 == 3) ? 9600 : 8000;
    int mod = mods[rmod(12)], nblk = 20 + (int)rmod(300);
    char tag[16];
    SETLVL(lvl_value(rpct(20) ? 2 : 0));
    g_sreg[7] = 5 + (int)rmod(6);
    if (t % 5 == 4) g_sreg[7] = 1 + (int)rmod(2); /* the S7 timeout of the receivers fires within the session */
    memset(g_state_seen, 0, sizeof g_state_seen);
    side_init(&g_A, FAKEM, 1, 1, rate);
    side_init(&g_B, FAKEM2, 2, 0, rate);
    cov_hit(CV_E8_LIFE);
    snprintf(tag, sizeof tag, "l%d", t);
    emit(0, "SESSION %s mod=%x rate=%u nblk=%d", tag, mod, rate, nblk);
    switch (t % 5) {
      case 0: /* a data session cut short: the objects are deleted while the modems run (data transmitter / receiver deleted by fax_class1_delete) */
        add_step(&g_A, 2, mod, 1, 0, 200, 0, 0, 900);
        add_step(&g_B, 3, mod, 0, 0, 0, 0, 0, 900);
        break;
      case 1: /* random commands, including rejected ones, issued back to back */
      {
        int k;
        for (k = 0; k < 6; k++) {
          int cmd = (int)rmod(7);
          int param = rpct(60) ? mods[rmod(12)] : (rpct(50) ? 3 : (int)rmod(300));
          unsigned nfr = 1 + rmod(3);
          int wt = (int)rmod(8);
          add_step(&g_A, cmd, param, cmd == 2 || cmd == 4, cmd == 4, nfr, wt, 0, 60);
          cmd = (int)rmod(7);
          param = rpct(60) ? mods[rmod(12)] : (rpct(50) ? 3 : (int)rmod(300));
          nfr = 1 + rmod(3);
          wt = (int)rmod(8);
          add_step(&g_B, cmd, param, cmd == 2 || cmd == 4, cmd == 4, nfr, wt, 0, 60);
        }
        cov_hit(CV_E8_ORDER);
        break;
      }
      case 2: /* the answering side stays in the answer tone while the caller sends HDLC */
        add_step(&g_A, 4, 3, 1, 1, 2, 0, 0, 400);
        break;
      case 4: /* receivers waiting for a carrier that never comes: the S7 timeout */
        add_step(&g_A, 5, 3, 0, 1, 0, 0, 0, 400);
        add_step(&g_B, 3, mod, 0, 0, 0, 0, 0, 400);
        add_step(&g_A, 3, mod, 0, 0, 0, 0, 0, 400);
        cov_hit(CV_E8_TIMEOUT);
        break;
      default: /* FRM against a silent line, then FTM */
        add_step(&g_B, 3, mod, 0, 0, 0, 0, 0, 80);
        add_step(&g_B, 2, mod, 1, 0, 50, 0, 0, 300);
        add_step(&g_A, 3, mod, 0, 0, 0, 5, 0, 300);
        cov_hit(CV_E8_ANS);
        break;
    }
    run_session(tag, nblk, 0, 0);
    session_end();
  }
}


/* ================================================================================================ malloc failure probes of the whole chain */
/* the status of a child that runs FAX_create (+ a command + a few blocks + FAX_delete) with the k-th allocation failing: 0 returned normally, 1 killed by a signal */
static int g_probe_rate, g_probe_cmd;
static void child_probe(void *arg)
{
  int k = (int)(intptr_t)arg, b;
  FaxGlue *g;
  short in[1024], out[1024];
  struct modem *m = FAKEM;
  memset(in, 0, sizeof in);
  g_sreg[7] = 7;
  g_fail_malloc = -1;
  if (g_probe_cmd >= 0) {
    /* the allocation index is counted from the command: create first without failure */
    g = (FaxGlue *)FAX_create(m, 1, (unsigned)g_probe_rate);
    if (!g) _exit(0);
    g_malloc_ctr = 0;
    g_fail_malloc = k;
    FAX_class1_command(g, (unsigned)g_probe_cmd, g_probe_cmd == 4 || g_probe_cmd == 5 ? 3u : 0x30u);
    for (b = 0; b < 12; b++) FAX_process(g, in, out, (int)g->blk);
    g_fail_malloc = -1;
    FAX_delete(g);
    return;
  }
  g_fail_malloc = k;
  g = (FaxGlue *)FAX_create(m, 1, (unsigned)g_probe_rate);
  g_fail_malloc = -1;
  if (!g) return;
  for (b = 0; b < 12; b++) FAX_process(g, in, out, (int)g->blk);
  FAX_delete(g);
}
static void scen_eng_probes(void)
{
  static const int cmds[] = {-1, 2, 3, 4, 5};
  static const int rates[] = {8000, 9600};
  const char *pf = getenv("R30_PROBE_FILE");
  FILE *fp = pf ? fopen(pf, "a") : NULL;
  int ci, ri, k;
  for (ri = 0; ri < 2; ri++)
    for (ci = 0; ci < 5; ci++) {
      unsigned nalloc;
      g_probe_rate = rates[ri];
      g_probe_cmd = cmds[ci];
      /* count the allocations of the unit under test */
      g_malloc_ctr = 0;
      g_nmsz = 0;
      {
        FaxGlue *g;
        short in[1024], out[1024];
        int b;
        memset(in, 0, sizeof in);
        g_sreg[7] = 7;
        g = (FaxGlue *)FAX_create(FAKEM, 1, (unsigned)g_probe_rate);
        if (cmds[ci] >= 0) {
          g_nmsz = 0;
          FAX_class1_command(g, (unsigned)cmds[ci], cmds[ci] == 4 || cmds[ci] == 5 ? 3u : 0x30u);
        }
        for (b = 0; b < 12; b++) FAX_process(g, in, out, (int)g->blk);
        nalloc = g_nmsz;
        FAX_delete(g);
        trial_end("probecount");
      }
      emit(0, "PROBE rate=%d cmd=%d allocations=%u", rates[ri], cmds[ci], nalloc);
      for (k = 0; k < (int)nalloc && k < 300; k++) {
        int r = guarded(child_probe, (void *)(intptr_t)k);
        cov_hit(CV_E9_PROBE);
        /* the outcomes of the lower layers differ by design (the ports of some engines survive an allocation failure the blob crashes on): written to a side file the build script compares in
         * ONE direction only (a port that crashes where the blob survives is a failure) */
        if (fp) fprintf(fp, "%d %d %d %d\n", rates[ri], cmds[ci], k, r != 0);
        if (r != 0 && r != 1) die3("probe child status %d", r);
      }
    }
  if (fp) fclose(fp);
}

#endif /* HARNESS_ENG */
