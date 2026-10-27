/* R28 harness part 1: scenario 0 (T30frames + ctools) and scenario 1 (the shared states and the small initialisers).  Included by fax_t30_tx.main.c. */

/* ---------------------------------------------------------------------------------------------- poking the constant records */
static void poke_consts(int randomize)
{
  FaxVmiCtl c;
  FaxVmiSts s;
  int a[5];
  int i;
  memset(&c, 0, sizeof c);
  memset(&s, 0, sizeof s);
  if (randomize) {
    c.flushRing = (int)rnd();
    c.breakReq = (int)rnd();
    c.breakCount = (unsigned short)rnd();
    c.pad0a = (unsigned short)rnd();
    c.rxReset = (int)rnd();
    c.newMode = (unsigned short)rnd();
    c.pad12 = (unsigned short)rnd();
    s.ringFree = (short)rnd();
    s.pad2 = (short)rnd();
    s.ovf = (int)rnd();
    s.txEmpty = (int)rnd();
    s.rxOverflow = (int)rnd();
    s.rxBreak = (int)rnd();
    s.hskipDone = (int)rnd();
    s.msts = randomize > 1 ? (void *)(uintptr_t)(rnd() & 0xfffffff0u) : NULL;
  }
  poke_bytes(h_FAXVMI_CTL, &c, sizeof c);
  poke_bytes(h_FAXVMI_STS, &s, sizeof s);
  if (randomize) {
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V21TX_CTL, a, sizeof h_V21TX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V21RX_CTL, a, sizeof h_V21RX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V27TX_CTL, a, sizeof h_V27TX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V29TX_CTL, a, sizeof h_V29TX_CTL);
    for (i = 0; i < 5; i++) a[i] = (int)rnd();
    poke_bytes(h_V17TX_CTL, a, sizeof h_V17TX_CTL);
  } else {
    static const int v21tx[5] = {0x12c, 0xea60, 0xc80, 0, 0}, v21rx[4] = {0x12c0000, 0xea60, 0, 0}, v27[5] = {0x2580, 0xea60, 1, 0, 0}, v29[4] = {0x2580, 0xea60, 1, 0},
                     v17[5] = {0x3840, 0xea60, 1, 0, 0};
    poke_bytes(h_V21TX_CTL, v21tx, sizeof v21tx);
    poke_bytes(h_V21RX_CTL, v21rx, sizeof v21rx);
    poke_bytes(h_V27TX_CTL, v27, sizeof v27);
    poke_bytes(h_V29TX_CTL, v29, sizeof v29);
    poke_bytes(h_V17TX_CTL, v17, sizeof v17);
  }
}
static void dump_consts(void)
{
  const FaxVmiCtl *c = (const FaxVmiCtl *)(const void *)h_FAXVMI_CTL;
  const FaxVmiSts *s = (const FaxVmiSts *)(const void *)h_FAXVMI_STS;
  emit(1, "FAXVMI_CTL w0=%d w4=%d s8=%u pad0a=%u wc=%d s10=%u pad12=%u", c->flushRing, c->breakReq, (unsigned)c->breakCount, (unsigned)c->pad0a, c->rxReset, (unsigned)c->newMode, (unsigned)c->pad12);
  emit(1, "FAXVMI_STS s0=%d pad2=%d w4=%d w8=%d wc=%d w10=%d w14=%d msts=%u", (int)s->ringFree, (int)s->pad2, s->ovf, s->txEmpty, s->rxOverflow, s->rxBreak, s->hskipDone, s->msts ? (unsigned)(uintptr_t)s->msts : 0u);
  emit(1, "V21TX_CTL %08x %08x %08x %08x %08x", h_V21TX_CTL[0], h_V21TX_CTL[1], h_V21TX_CTL[2], h_V21TX_CTL[3], h_V21TX_CTL[4]);
  emit(1, "V21RX_CTL %08x %08x %08x %08x", h_V21RX_CTL[0], h_V21RX_CTL[1], h_V21RX_CTL[2], h_V21RX_CTL[3]);
  emit(1, "V27TX_CTL %08x %08x %08x %08x %08x", h_V27TX_CTL[0], h_V27TX_CTL[1], h_V27TX_CTL[2], h_V27TX_CTL[3], h_V27TX_CTL[4]);
  emit(1, "V29TX_CTL %08x %08x %08x %08x", h_V29TX_CTL[0], h_V29TX_CTL[1], h_V29TX_CTL[2], h_V29TX_CTL[3]);
  emit(1, "V17TX_CTL %08x %08x %08x %08x %08x", h_V17TX_CTL[0], h_V17TX_CTL[1], h_V17TX_CTL[2], h_V17TX_CTL[3], h_V17TX_CTL[4]);
}

/* ---------------------------------------------------------------------------------------------- scenario 0 */
static void misc_ids(void)
{
  unsigned b0, b1, b2, k;
  static const unsigned other0[] = {0x00, 0x01, 0x7f, 0x80, 0xfe, 0x10, 0x03};
  u64 h = 1469598103934665603ULL;
  for (b1 = 0; b1 < 256; b1++) {
    u64 hh = 1469598103934665603ULL;
    for (b2 = 0; b2 < 256; b2++) {
      unsigned id = GetT30FrameIDFromBuffer(0xff, (unsigned char)b1, (unsigned char)b2);
      hh ^= id;
      hh *= 1099511628211ULL;
      cov_hit(CV_M_ID);
      if (b1 == 3) cov_hit(CV_M_ID_B1_3);
      else if (b1 == 0x13) cov_hit(CV_M_ID_B1_13);
      else cov_hit(CV_M_ID_OTHER);
      if (((b1 == 3 || b1 == 0x13) ? (unsigned)aReversedCharsArray[b2] : b2) > 0x84) cov_hit(CV_M_ID_BIG);
      else cov_hit(CV_M_ID_SMALL);
      if ((id & 0x7fffu) == 0x7fffu) die3("id");
    }
    emit(b1 < 4 || b1 == 0x13 || b1 == 0xff, "ID ff %02x * -> %016llx", b1, hh);
    h ^= hh;
    h *= 1099511628211ULL;
  }
  emit(1, "ID all %016llx", h);
  for (k = 0; k < sizeof other0 / sizeof other0[0]; k++)
    for (b1 = 0; b1 < 256; b1 += 17)
      for (b2 = 0; b2 < 256; b2 += 5) {
        unsigned id = GetT30FrameIDFromBuffer((unsigned char)other0[k], (unsigned char)b1, (unsigned char)b2);
        emit(0, "ID %02x %02x %02x -> %x", other0[k], b1, b2, id);
        cov_hit(CV_M_ID_NOT_FF);
        if (id != 0xff) die3("id of a non-0xff first byte");
      }
  for (k = 0; k < 20000; k++) {
    unsigned x, y, z;
    unsigned id;
    x = rnd() & 0xff;
    y = rnd() & 0xff;
    z = rnd() & 0xff;
    id = GetT30FrameIDFromBuffer((unsigned char)x, (unsigned char)y, (unsigned char)z);
    emit(0, "IDR %02x %02x %02x -> %x", x, y, z, id);
    if (x != 0xff) cov_hit(CV_M_ID_NOT_FF);
  }
  (void)b0;
}
static void misc_names(void)
{
  unsigned id, k, i;
  const char *p1, *p2, *last_unknown = NULL;
  /* every id of the table, then a sweep, then random */
  for (id = 0; id < 0x600; id++) {
    const char *nm = GetT30FrameNameByID(id);
    cov_hit(CV_M_NAME);
    emit(id < 0x100 || id == 0x2ff, "NAME %x -> \"%s\"", id, nm);
    if (strncmp(nm, "Unknown frame", 13) == 0) {
      cov_hit(CV_M_NAME_UNK);
      if (last_unknown && nm == last_unknown) cov_hit(CV_M_NAME_BUF);
      last_unknown = nm;
    } else {
      cov_hit(CV_M_NAME_KNOWN);
    }
  }
  for (k = 0; k < 400; k++) {
    unsigned x;
    x = rnd();
    if (rpct(30)) x &= 0xff;
    p1 = GetT30FrameNameByID(x);
    emit(0, "NAMER %08x -> \"%s\"", x, p1);
    if (strncmp(p1, "Unknown frame", 13) == 0) {
      unsigned y;
      y = rnd();
      p2 = GetT30FrameNameByID(y);
      emit(0, "NAMER2 %08x -> \"%s\" same_buffer=%d first_now=\"%s\"", y, p2, p1 == p2 || (strncmp(p2, "Unknown frame", 13) != 0), p1);
    }
  }
  (void)i;
  emit(1, "NAME 0xffffffff \"%s\"", GetT30FrameNameByID(0xffffffffu));
  emit(1, "NAME 0x80000000 \"%s\"", GetT30FrameNameByID(0x80000000u));
}
static void misc_silence(void)
{
  int k;
  for (k = -6; k <= 0x1a0; k += (k < 0x20 ? 1 : 29)) {
    short store[0x210], *buf = store + 8; /* guard words before and after: a write below / above the range is seen */
    int i;
    for (i = 0; i < 0x210; i++) store[i] = (short)(0x7700 + i);
    _put_silence(buf, k);
    emit(0, "SIL n=%d -> %016llx", k, fnv_shorts(store, 0x210));
    cov_hit(CV_M_SIL);
    if (k < 0) cov_hit(CV_M_SIL_NEG);
    if (k == 0) cov_hit(CV_M_SIL_0);
  }
}
static void misc_data_output(void)
{
  int t;
  for (t = 0; t < 3000; t++) {
    FaxT30 *o = &g_t30;
    int calls, c;
    t30_garbage(o);
    dsplibs_debug_level = 0;
    if (rpct(50)) {
      cTOOLS_handle_data_output_reset(o);
      cov_hit(CV_M_RESET);
      emit(0, "DOR reset w129c=%d w12a0=%08x", o->alignDone, o->alignHist);
    } else {
      o->alignDone = rpct(50) ? 0 : (int)rmod(3);
      if (rpct(40)) {
        o->alignHist = 0xffffffffu;
      } else {
        /* zero bits at random positions of bits 16 .. 23 */
        o->alignHist = rnd();
        if (rpct(50)) o->alignHist &= ~(1u << (16 + rmod(8)));
      }
      if (o->alignDone) {
        int sh = (int)(0x10 + rmod(9));
        o->alignShift = rpct(85) ? sh : (int)rmod(40);
        o->alignMask = rpct(85) ? (0xffu << (unsigned)sh) : rnd();
      }
    }
    calls = 1 + (int)rmod(4);
    for (c = 0; c < calls; c++) {
      unsigned short in[80];
      unsigned char out[240];
      int n, flag, i, r;
      n = rpct(8) ? 0 : (rpct(5) ? -(int)(1 + rmod(3)) : (int)rmod(60));
      flag = (int)rmod(3);
      for (i = 0; i < 80; i++) {
        unsigned v;
        v = rnd();
        in[i] = (unsigned short)(rpct(15) ? 0x10u : (rpct(10) ? (v & 0xffffu) : (v & 0xffu)));
        if (rpct(10)) in[i] = (unsigned short)(in[i] | 0xff00u); /* only the low byte is the data byte */
      }
      memset(out, 0xee, sizeof out);
      r = _handle_data_output(o, in, out, n, flag);
      cov_hit(CV_M_DO);
      if (n == 0) cov_hit(CV_M_DO_N0);
      if (n < 0) cov_hit(CV_M_DO_NEG);
      if (flag) cov_hit(CV_M_DO_FLAG);
      for (i = 0; i < r; i++)
        if (out[i] == 0x10 && i + 1 < r) {
          cov_hit(CV_M_DO_DLE);
          break;
        }
      if (n > 0) {
        if (o->alignDone) cov_hit(CV_M_DO_ALIGN);
        else cov_hit(CV_M_DO_SKIP);
        if (c > 0 && o->alignDone) cov_hit(CV_M_DO_RUN);
      }
      emit(0, "DO t=%d c=%d n=%d flag=%d r=%d out=%016llx w129c=%d w12a0=%08x w12a4=%d w12a8=%08x", t, c, n, flag, r, fnv_bytes(out, sizeof out), o->alignDone, o->alignHist, o->alignShift,
           o->alignMask);
      if (r > 2 * (n > 0 ? n : 0) + 2) die3("_handle_data_output wrote %d bytes for %d", r, n);
      if (c > 0 && o->alignDone == 1) cov_hit(CV_M_DO_MID);
    }
  }
}
static void misc_hdlc_output(void)
{
  int t;
  static const unsigned char names[][3] = {{0xff, 0x03, 0x80}, {0xff, 0x13, 0x80}, {0xff, 0x03, 0x01}, {0xff, 0x13, 0x02}, {0xff, 0x03, 0xfa}, {0xff, 0x13, 0x00}, {0xff, 0xc8, 0x80}};
  for (t = 0; t < 2500; t++) {
    FaxT30 *o = &g_t30;
    unsigned short in[80];
    unsigned char out[240];
    int n, flag, i, r, dbg, high;
    t30_garbage(o);
    dbg = (int)rmod(4);
    dsplibs_debug_level = (unsigned)dbg;
    flag = (int)rmod(3);
    n = rpct(6) ? 0 : (rpct(4) ? 1 : (rpct(3) ? 2 : (rpct(4) ? -(int)(1 + rmod(3)) : (int)rmod(50))));
    high = dbg <= 1 && rpct(20);
    for (i = 0; i < 80; i++) {
      unsigned v;
      v = rnd();
      in[i] = (unsigned short)(rpct(12) ? 0x10u : (v & 0xffu));
      if (high && rpct(15)) in[i] = (unsigned short)(in[i] | (v & 0xff00u));
    }
    if (n > 2 && rpct(60)) {
      const unsigned char *nm = names[rmod(sizeof names / sizeof names[0])];
      in[0] = nm[0];
      in[1] = nm[1];
      in[2] = nm[2];
      if (rpct(30)) in[2] = (unsigned short)(rnd() & 0xff);
    }
    memset(out, 0xee, sizeof out);
    r = cTOOLS_handle_hdlc_output(o, in, out, n, flag);
    emit(0, "HO t=%d dbg=%d n=%d flag=%d r=%d out=%016llx", t, dbg, n, flag, r, fnv_bytes(out, sizeof out));
    cov_hit(CV_M_HO);
    if (n == 0) cov_hit(CV_M_HO_N0);
    if (n == 1) cov_hit(CV_M_HO_N1);
    if (n > 2) cov_hit(CV_M_HO_N3);
    if (n < 0) cov_hit(CV_M_HO_NEG);
    if (flag) cov_hit(CV_M_HO_FLAG);
    if (n > 2 && dbg > 1) cov_hit(CV_M_HO_NAME);
    if (dbg > 1) cov_hit(CV_M_HO_DBG);
    if (high) cov_hit(CV_M_HO_HIGH);
    for (i = 0; i < r - 1; i++)
      if (out[i] == 0x10 && out[i + 1] == 0x10) {
        cov_hit(CV_M_HO_DLE);
        break;
      }
    if (r > 2 * (n > 0 ? n : 0) + 2) die3("hdlc output wrote %d bytes", r);
  }
  dsplibs_debug_level = 0;
}
static void scen_misc(void)
{
  poke_consts(0);
  dump_consts();
  misc_ids();
  misc_names();
  misc_silence();
  misc_data_output();
  misc_hdlc_output();
}

/* ---------------------------------------------------------------------------------------------- scenario 1 */
static const StateFn k_idle = T_IDLE;
static void shared_idle(void)
{
  int t;
  for (t = 0; t < 600; t++) {
    FaxT30 *o = &g_t30;
    int n_in;
    t30_garbage(o);
    t30_channels(o, -1);
    n_in = rpct(10) ? 0 : (rpct(10) ? -(int)(1 + rmod(1000)) : (rpct(10) ? 160 : (int)rmod(400)));
    io_prep(&g_io, n_in, (int)rnd(), (int)rnd(), (int)rnd());
    emit(0, "IDLE t=%d init=%d", t, _idle_state_init(o));
    call_state(k_idle, o, &g_io, "idle");
    if (n_in > 0) cov_hit(CV_S_IDLE_N);
    else if (n_in == 0) cov_hit(CV_S_IDLE_0);
    else cov_hit(CV_S_IDLE_NEG);
    cov_hit(CV_S_IDLEI);
    t30_free_channels(o);
    trial_end("idle");
  }
}
static void shared_silence(void)
{
  int t;
  static const int ns[] = {-5, -2, -1, 0, 1, 2, 3, 4, 100, 0x7fffffff, (int)0x80000000, 1000, 77};
  for (t = 0; t < 400; t++) {
    FaxT30 *o = &g_t30;
    int r, n;
    t30_garbage(o);
    t30_channels(o, -1);
    if (t < 40) {
      n = ns[t % (int)(sizeof ns / sizeof ns[0])];
      r = _send_silence_state_init(o, n);
      emit(0, "SILI t=%d n=%d r=%d state=%d tmr=%d", t, n, r, o->state, o->tmr);
      cov_hit(CV_S_SILI);
      if (n / 2 == 0) cov_hit(CV_S_SILI_ZERO);
    }
    {
      int pick = (int)rmod(5);
      o->tmr = pick == 0 ? 1 : (pick == 1 ? 0 : (pick == 2 ? 2 : (pick == 3 ? (int)rnd() : (int)rmod(10))));
    }
    o->status = (int)rnd();
    io_prep(&g_io, (int)rnd(), (int)rnd(), (int)rnd(), (int)rnd());
    call_state(T_SILENCE, o, &g_io, "silence");
    cov_hit(CV_S_SIL);
    if (o->tmr == 0 && o->status == 3) cov_hit(CV_S_SIL_END);
    t30_free_channels(o);
    trial_end("silence");
  }
}
static void shared_answer(void)
{
  int t;
  unsigned hho0 = 0;
  for (t = 0; t < 800; t++) {
    FaxT30 *o = &g_t30;
    int pick;
    t30_garbage(o);
    t30_channels(o, -1);
    if (rpct(10)) o->tone = NULL;
    o->ans_len = rpct(15) ? -(int)rmod(4) : (int)rmod(20);
    pick = (int)rmod(6);
    o->tmr = pick == 0 ? o->ans_len : (pick == 1 ? o->ans_len - 1 : (pick == 2 ? o->ans_len + 1 : (pick == 3 ? (int)rnd() : (pick == 4 ? -(int)rmod(5) : (int)rmod(30)))));
    if (rpct(50)) o->tmr = (int)((unsigned)o->tmr & 0xffu);
    script_random();
    poke_consts(rpct(60) ? 1 + (int)rmod(2) : 0);
    dsplibs_debug_level = rmod(4);
    io_prep(&g_io, (int)rnd(), (int)rnd(), (int)rnd(), (int)rnd());
    emit(0, "ANS t=%d dbg=%u", t, dsplibs_debug_level);
    hho0 = g_hho_count;
    call_state(T_ANSWER, o, &g_io, "answer");
    cov_hit(CV_S_ANS);
    if (g_hho_count != hho0) cov_hit(CV_S_ANS_END);
    if (g_io.tx[0] == 0x300) cov_hit(CV_S_ANS_TONE);
    if (o->tmr < 0) cov_hit(CV_S_ANS_NEG);
    t30_free_channels(o);
    trial_end("answer");
  }
  dsplibs_debug_level = 0;
}
static void shared_inits(void)
{
  int t;
  for (t = 0; t < 300; t++) {
    FaxT30 *o = &g_t30;
    int r;
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    poke_consts(rpct(70) ? 1 + (int)rmod(2) : 0);
    r = cHDLCtx_preamble_state_init(o);
    emit(0, "PREINIT t=%d r=%d", t, r);
    dump_t30("preinit", o);
    cov_hit(CV_S_PRE);
    cov_hit(CV_S_CTL_V21TX);
    if (h_V21TX_CTL[0] != 0x12c) cov_hit(CV_S_PRE_POKE);
    t30_free_channels(o);
    t30_garbage(o);
    t30_channels(o, -1);
    script_random();
    r = cHDLCtx_off_init(o);
    emit(0, "OFFINIT t=%d r=%d", t, r);
    dump_t30("offinit", o);
    cov_hit(CV_S_OFFI);
    cov_hit(CV_S_CTL_V21RX);
    t30_free_channels(o);
    t30_garbage(o);
    t30_channels(o, -1);
    r = _send_hdlc_between_buffer_state_init(o);
    emit(0, "HBINIT t=%d r=%d", t, r);
    dump_t30("hbinit", o);
    cov_hit(CV_S_HBI);
    t30_free_channels(o);
    t30_garbage(o);
    r = _init_tx_nulls_state(o);
    emit(0, "NULLINIT t=%d r=%d", t, r);
    dump_t30("nullinit", o);
    cov_hit(CV_S_NULLI);
    trial_end("inits");
  }
  poke_consts(0);
}
static void scen_shared(void)
{
  poke_consts(0);
  dump_consts();
  shared_idle();
  shared_silence();
  shared_answer();
  shared_inits();
}
