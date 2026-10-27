/* R30 harness part 0: the canonical description of the objects of the fax stack (the class 1 object FaxT30 and the glue block FaxGlue), shared by the builds.  Included by fax_api.main.c. */
#define H0 1469598103934665603ULL

/* named address ranges (pointers into static harness objects are described by name, heap blocks by their block id: host independent) */
typedef struct {
  const char *name;
  const void *base;
  unsigned bytes;
} HTab;
static HTab g_htab[24];
static int g_nhtab;
static void htab_add(const char *name, const void *base, unsigned bytes)
{
  if (g_nhtab >= 24) die3("htab");
  g_htab[g_nhtab].name = name;
  g_htab[g_nhtab].base = base;
  g_htab[g_nhtab].bytes = bytes;
  g_nhtab++;
}
static const char *pname(const void *p, char *b, size_t cap)
{
  int i;
  if (!p) {
    snprintf(b, cap, "NULL");
    return b;
  }
  for (i = 0; i < g_nhtab; i++)
    if ((const char *)p >= (const char *)g_htab[i].base && (const char *)p < (const char *)g_htab[i].base + g_htab[i].bytes) {
      snprintf(b, cap, "%s", g_htab[i].name);
      return b;
    }
  if (blk_id_of(p)) snprintf(b, cap, "blk%u", blk_id_of(p));
  else snprintf(b, cap, "?");
  return b;
}
static u64 blk_hash(const void *p)
{
  const Blk *b = find_blk(p);
  return b ? fnv_bytes(b->p, b->size) : 0ULL;
}
static FaxT30 *g_cur; /* the object whose hash the debug recorder appends */
/* is the member still holding the byte pattern of a fresh heap block?  (the creation functions leave most members uninitialised: their garbage depends on the member's offset in the block,
 * which differs between i386 and x86-64, so an untouched member is dumped as "u") */
static int untouched(const FaxT30 *o, const void *m, size_t n)
{
  const Blk *b = find_blk(o);
  const unsigned char *p = (const unsigned char *)m;
  size_t k;
  if (!b || b->p != (const unsigned char *)o) return 0;
  for (k = 0; k < n; k++) {
    size_t off = (size_t)(p + k - b->p);
    if (p[k] != (unsigned char)(0x35u + b->id * 13u + (unsigned)off * 7u + ((unsigned)off >> 6))) return 0;
  }
  return 1;
}
static void t30_text(char *b, size_t cap, const FaxT30 *o)
{
  char p1[24], p2[24], p3[24], p4[24], p5[24], p6[24], p7[24], p8[24];
  FaxT30 c = *o;
#define UN(m) \
  if (untouched(o, &o->m, sizeof o->m)) c.m = 0x7e57e57e
  UN(ans_org); UN(state); UN(state_prev); UN(lastFrame); UN(tmr); UN(status); UN(w1230); UN(w1234); UN(lastByte); UN(rx_event); UN(rx_kind); UN(chan); UN(modemClass); UN(dlePending); UN(hdlcWr);
  UN(modem_sel); UN(cng_tone); UN(cng_ctr); UN(cng_on); UN(ans_len); UN(dataRate); UN(enableDelay); UN(sec); UN(ms); UN(rx_act); UN(frameDone); UN(frameCount); UN(txEnable); UN(frameWords); UN(txConnected); UN(fifoReady);
  UN(alignDone); UN(alignHist); UN(alignShift); UN(alignMask); UN(w12ac); UN(dataEnd); UN(s7); UN(silenceTime); UN(silence_rms); UN(agc_rcp); UN(agc_shift); UN(emu_delay); UN(emu_idx); UN(sf_fill); UN(cd_flag); UN(gain_att);
  UN(hp_on);
#undef UN
  if (untouched(o, o->hpState, sizeof o->hpState)) memset(c.hpState, 0x7e, sizeof c.hpState);
  if (untouched(o, (const char *)&o->silence_rms + 2, 2)) c.silence_rms = (c.silence_rms & 0xffff) | 0x7e570000; /* RS clears only the low half */
  snprintf(b, cap,
           "hdlc_tx=%s hdlc_rx=%s data_vmi=%s txcfg=%s rxcfg=%s data_cfg=%s tone=%s fifo=%s ans_org=%d state=%d prev=%d w1224=%d tmr=%d w122c=%d w1230=%d w1234=%d w1238=%d ev=%d kind=%d "
           "chan=%d w1248=%d w124c=%d w1250=%d sel=%d cng_tone=%d cng_ctr=%d cng_on=%d ans_len=%d w126c=%d w1270=%d sec=%d ms=%d act=%d w1280=%d w1284=%d w128c=%d w1290=%d w1294=%d "
           "w1298=%d w129c=%d w12a0=%08x w12a4=%d w12a8=%08x w12ac=%d w12b0=%d s7=%d w12b8=%d rms=%d agc_rcp=%d agc_shift=%d emu_delay=%d emu_idx=%d sf_fill=%d cd=%d gain=%d "
           "p12dc=%016llx b12e0=%016llx hp=%d buf=%016llx",
           pname(o->hdlc_tx, p1, sizeof p1), pname(o->hdlc_rx, p2, sizeof p2), pname(o->data_vmi, p3, sizeof p3), pname(o->hdlc_tx_cfg, p4, sizeof p4),
           pname(o->hdlc_rx_cfg, p5, sizeof p5), pname(o->data_cfg, p6, sizeof p6), pname(o->tone, p7, sizeof p7), pname(o->fifo, p8, sizeof p8), c.ans_org, c.state, c.state_prev,
           c.lastFrame, c.tmr, c.status, c.w1230, c.w1234, c.lastByte, c.rx_event, c.rx_kind, c.chan, c.modemClass, c.dlePending, c.hdlcWr, c.modem_sel, c.cng_tone, c.cng_ctr, c.cng_on,
           c.ans_len, c.dataRate, c.enableDelay, c.sec, c.ms, c.rx_act, c.frameDone, c.frameCount, c.txEnable, c.frameWords, c.txConnected, c.fifoReady, c.alignDone, c.alignHist, c.alignShift, c.alignMask, c.w12ac,
           c.dataEnd, c.s7, c.silenceTime, c.silence_rms, c.agc_rcp, c.agc_shift, c.emu_delay, c.emu_idx, c.sf_fill, c.cd_flag, c.gain_att,
           o->hpCoef ? fnv_bytes(o->hpCoef, 20) : 0ULL, fnv_bytes(c.hpState, sizeof c.hpState), c.hp_on, fnv_shorts(H0, (const short *)o->buf, 0x900));
}
static u64 h30(const FaxT30 *o)
{
  char b[2048];
  t30_text(b, sizeof b, o);
  return fnv_bytes(b, strlen(b));
}

static struct {
  unsigned short pre[16];
  FaxT30 o;
} g_wrap;
#define g_t30 (g_wrap.o)
static void dump_t30(const char *tag, const FaxT30 *o)
{
  char b[2048];
  t30_text(b, sizeof b, o);
  emit(0, "%s T %s guard=%016llx", tag, b, o == &g_t30 ? fnv_shorts(H0, (const short *)g_wrap.pre, 16) : 0ULL);
}
/* the glue block */
static u64 snap_glue(const FaxGlue *g)
{
  u64 h = H0;
  h = fnv_mix(h, (unsigned)MID(g->modem));
  h = fnv_mix(h, hid(g->c1));
  h = fnv_mix(h, (unsigned)g->online);
  h = fnv_mix(h, (unsigned)g->tty_cnt);
  h ^= fnv_bytes(g->tty_out, sizeof g->tty_out);
  h ^= fnv_bytes(g->tty_in, sizeof g->tty_in) * 3;
  h = fnv_mix(h, hid(g->rcIn));
  h = fnv_mix(h, hid(g->rcOut));
  h = fnv_shorts(h, g->rxbuf, 0xa0);
  h = fnv_shorts(h, g->txbuf, 0xa0);
  h = fnv_mix(h, g->blk);
  h = fnv_mix(h, g->out.count);
  h = fnv_mix(h, g->out.wr);
  h = fnv_mix(h, g->out.rd);
  h = fnv_mix(h, g->out.half);
  h = fnv_shorts(h, g->out.data, 384);
  h = fnv_mix(h, g->in.count);
  h = fnv_mix(h, g->in.wr);
  h = fnv_mix(h, g->in.rd);
  h = fnv_mix(h, g->in.half);
  h = fnv_shorts(h, g->in.data, 384);
  return h;
}
