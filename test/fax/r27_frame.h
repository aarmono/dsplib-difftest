/* R27 harness part 2: pure functions (scenario 1), ring writers (scenario 2), framing (scenario 3).  Included by fax_vmi.main.c. */

static void dbg_level(unsigned v) { dsplibs_debug_level = v; }

/* ---------------------------------------------------------------------------------------------- model: CRC-16 CCITT (MSB first, init 0xffff, complemented) */
static unsigned short crc_model(const unsigned char *d, unsigned n)
{
  unsigned crc = 0xffff, i, k;
  for (i = 0; i < n; i++) {
    crc ^= (unsigned)d[i] << 8;
    for (k = 0; k < 8; k++) crc = (crc & 0x8000) ? (((crc << 1) ^ 0x1021) & 0xffff) : ((crc << 1) & 0xffff);
  }
  return (unsigned short)~crc;
}

/* ---------------------------------------------------------------------------------------------- scenario 1: pure functions */
static unsigned short g_big[70000], g_big2[70000];
static void scen_pure(void)
{
  int t;
  for (t = 0; t < 500; t++) {
    unsigned n = t < 60 ? (unsigned)t : (unsigned)rmod(90), i;
    unsigned short buf[200];
    unsigned char bytes[200];
    unsigned short r;
    int wide = rpct(25);
    for (i = 0; i < 200; i++) {
      bytes[i] = (unsigned char)rnd();
      buf[i] = wide ? (unsigned short)rnd() : bytes[i];
    }
    r = faxvmi_gen_fcs16(buf, (short)n);
    cov_hit(CV_P_FCS);
    if (n == 0) cov_hit(CV_P_FCS0);
    emit(0, "FCS n=%u wide=%d -> %04x", n, wide, r);
    if (!wide) {
      if (r != crc_model(bytes, n)) die3("faxvmi_gen_fcs16 differs from the CRC-16 model (n=%u)", n);
      /* data + crc (high byte first) must give the residue */
      if (n + 2 <= 200) {
        buf[n] = (unsigned short)(r >> 8);
        buf[n + 1] = (unsigned short)(r & 0xff);
        if (faxvmi_gen_fcs16(buf, (short)(n + 2)) != 0xe2f0) die3("frame + crc has no residue 0xe2f0");
        cov_hit(CV_P_FCSOK);
      }
    }
  }
  for (t = 0; t < 3; t++) {
    unsigned i;
    for (i = 0; i < 70000; i++) g_big[i] = (unsigned short)(rpct(80) ? rmod(256) : rnd());
    emit(0, "FCS big -> %04x", faxvmi_gen_fcs16(g_big, (short)-1));
    cov_hit(CV_P_FCSBIG);
    for (i = 0; i < 70000; i++) g_big2[i] = g_big[i];
    faxvmi_byte_reverse(g_big2, (short)-1);
    emit(0, "BREV big %016llx %04x %04x", fnv_shorts((short *)g_big2, 70000), g_big2[65535], g_big2[65536]);
    cov_hit(CV_P_BREVBIG);
  }
  for (t = 0; t < 400; t++) {
    unsigned short buf[64];
    unsigned i, n = (unsigned)rmod(40);
    for (i = 0; i < 64; i++) buf[i] = (unsigned short)(rpct(60) ? rmod(256) : rnd());
    faxvmi_byte_reverse(buf, (short)n);
    cov_hit(CV_P_BREV);
    if (n == 0) cov_hit(CV_P_BREV0);
    emit(0, "BREV n=%u %016llx", n, fnv_shorts((short *)buf, 64));
  }
  for (t = 0; t < 400; t++) {
    unsigned short buf[200];
    unsigned i, k, nf = (unsigned)rmod(7), pos = 0;
    for (i = 0; i < 200; i++) buf[i] = (unsigned short)(rpct(60) ? rmod(256) : rnd());
    for (k = 0; k < nf + 1 && pos < 150; k++) {
      unsigned len = rpct(15) ? 0 : rmod(12);
      buf[pos] = (unsigned short)len;
      if (len == 0) cov_hit(CV_P_FREVLEN0);
      pos += 1 + len;
    }
    faxvmi_frame_reverse(buf, (short)nf);
    cov_hit(CV_P_FREV);
    if (nf == 0) cov_hit(CV_P_FREV0);
    emit(0, "FREV nf=%u %016llx", nf, fnv_shorts((short *)buf, 200));
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 2: ring writers */
static void ring_negative(void);
static void scen_ring(void)
{
  int t;
  ring_negative();
  for (t = 0; t < 1500; t++) {
    FaxVmiCfg c;
    FaxVmi *o;
    int d8, da, dc;
    d8 = 1 + (int)rmod(30);
    da = (int)rmod(12);
    dc = (int)rmod(4);
    c = mkcfg(0, 0, d8, da, dc, 0, NULL);
    o = vmi_new(c, 0);
    FaxVmiSt *s = o->st;
    unsigned short in[200], *p;
    unsigned i;
    int r, which = t & 1;
    short n;
    unsigned room0;
    emit(0, "RING trial %d which %d", t, which);
    {
      unsigned rd0, cn0;
      rd0 = rmod(s->size);
      cn0 = rpct(30) ? 0 : (rpct(15) ? s->size : rmod((unsigned)s->size + 1));
      ring_set(s, rd0, cn0);
    }
    if (rpct(15)) s->wr = (unsigned short)rmod(s->size);
    o->txEmpty = rpct(50) ? 1 : (rpct(50) ? 0 : (int)rnd());
    for (i = 0; i < 200; i++) in[i] = (unsigned short)(rpct(70) ? rmod(256) : rnd());
    room0 = (unsigned)s->size - s->cnt;
    dump_vmi("pre", o);
    p = in;
    if (which == 0) {
      int w18 = o->txEmpty;
      unsigned cnt0 = s->cnt;
      unsigned wr0 = s->wr;
      n = (short)(rpct(8) ? -(int)(1 + rmod(3)) : (int)rmod((unsigned)s->size + 4));
      r = faxvmi_write_fifo(o, &p, n);
      cov_hit(CV_R_FIFO);
      if (n == 0) cov_hit(CV_R_FIFO_N0);
      if (n < 0) cov_hit(CV_R_FIFO_NEG);
      if (room0 == 0) cov_hit(CV_R_FIFO_FULL);
      if (r > 0 && (unsigned)r < (unsigned)n) cov_hit(CV_R_FIFO_PART);
      if (r > 0 && r == n) cov_hit(CV_R_FIFO_ALL);
      if (r > 0 && wr0 + (unsigned)r > s->size) cov_hit(CV_R_FIFO_WRAP);
      if (r > 0 && w18 != 0 && o->txEmpty == 0) cov_hit(CV_R_FIFO_W18);
      (void)cnt0;
    } else {
      unsigned pos = 0, k, nfr = 1 + rmod(7);
      int w18 = o->txEmpty;
      unsigned wr0 = s->wr;
      for (i = 0; i < 200; i++) in[i] = (unsigned short)rmod(256);
      for (i = 0; i < 200; i++) if (rpct(50)) in[i] = 0;
      for (k = 0; k < 8 && pos < 150; k++) {
        unsigned len = rpct(10) ? 0 : (rpct(15) ? rmod((unsigned)s->size + 3) : rmod(8));
        in[pos] = (unsigned short)len;
        pos += 1 + len;
      }
      /* the frames of the input array: `n` frames; the later ones may run into random data (their lengths are read as written) */
      n = (short)(rpct(10) ? 0 : (int)nfr);
      r = faxvmi_write_frame(o, &p, n);
      cov_hit(CV_R_FRAME);
      if (n == 0) cov_hit(CV_R_FRAME_N0);
      if (r == 0 && n != 0) cov_hit(CV_R_FRAME_REJECT);
      if (r > 1) cov_hit(CV_R_FRAME_MULTI);
      if (r > 0 && r < n) cov_hit(CV_R_FRAME_STOP);
      if (r > 0 && wr0 + 3u > s->size) cov_hit(CV_R_FRAME_WRAP);
      if (r > 0 && w18 != 0 && o->txEmpty == 0) cov_hit(CV_R_FRAME_W18);
      if (r > 0 && in[0] == 0) cov_hit(CV_R_FRAME_LEN0);
    }
    emit(0, "RING r=%d adv=%d", r, (int)(p - in));
    dump_vmi("post", o);
    FAXVMI_delete(o);
    trial_end("ring");
  }
}

/* write_frame with NEGATIVE lengths (16 bit words >= 0x8000): the room test passes, the CRC loop and the data copy run ~65535 words (the copy stops when the ring is full: the overflow path) */
static void ring_negative(void)
{
  int t;
  for (t = 0; t < 40; t++) {
    FaxVmiCfg c;
    FaxVmi *o;
    FaxVmiSt *s;
    unsigned short *p;
    unsigned i;
    int r, d8, da;
    d8 = 8 + (int)rmod(24);
    da = (int)rmod(6);
    c = mkcfg(0, 0, d8, da, 1, 0, NULL);
    o = vmi_new(c, 0);
    s = o->st;
    {
      unsigned rd0, cn0;
      rd0 = rmod(s->size);
      cn0 = rpct(50) ? 0 : rmod(4);
      ring_set(s, rd0, cn0);
    }
    o->txEmpty = (int)rmod(2);
    for (i = 0; i < 70000; i++) g_big[i] = (unsigned short)rmod(256);
    g_big[0] = (unsigned short)(0xffff - rmod(3)); /* -1 .. -3 */
    g_big[70000 - 1] = 0;
    p = g_big;
    r = faxvmi_write_frame(o, &p, (short)(1 + rmod(2)));
    emit(0, "NEG r=%d adv=%d", r, (int)(p - g_big));
    dump_vmi("neg", o);
    FAXVMI_delete(o);
    trial_end("ring negative");
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 3: framing */
/* bit stream builders (the models) */
static unsigned char g_bits[1 << 17];
static unsigned g_nbits;
static int g_had_stuff;
static void bput(unsigned b) { g_bits[g_nbits++] = (unsigned char)(b & 1); }
static void bput_byte(unsigned v)
{
  int k;
  for (k = 7; k >= 0; k--) bput((v >> k) & 1);
}
static void bput_flag(void) { bput_byte(0x7e); }
/* data bytes with bit stuffing */
static void bput_stuffed(const unsigned char *d, unsigned n)
{
  unsigned i, ones = 0;
  int k;
  for (i = 0; i < n; i++)
    for (k = 7; k >= 0; k--) {
      unsigned b = (d[i] >> k) & 1;
      bput(b);
      if (b) {
        if (++ones == 5) {
          bput(0);
          g_had_stuff = 1;
          ones = 0;
        }
      } else
        ones = 0;
    }
}
/* an HDLC frame: preflags, junk zeros, data + crc (stuffed), postflags */
static void hdlc_model(const unsigned char *data, unsigned len, int pre, int junk, int post, int crc_ok)
{
  unsigned char tmp[128];
  unsigned short f = crc_model(data, len);
  int i;
  memcpy(tmp, data, len);
  if (!crc_ok) f ^= (unsigned short)(1u << rmod(16));
  tmp[len] = (unsigned char)(f >> 8);
  tmp[len + 1] = (unsigned char)f;
  for (i = 0; i < pre; i++) bput_flag();
  for (i = 0; i < junk; i++) bput(0);
  bput_stuffed(tmp, len + 2);
  for (i = 0; i < post; i++) bput_flag();
}
/* feed the stream g_bits[from..] into m->bits as words of `width` bits (MSB first), `nw` words; returns the next bit index */
static unsigned bits_to_words(FaxVmiM *m, unsigned from, unsigned nw, unsigned width)
{
  unsigned w, k;
  for (w = 0; w < nw; w++) {
    unsigned v = 0;
    for (k = 0; k < width; k++) {
      unsigned b = from < g_nbits ? g_bits[from] : 1u; /* idle: ones */
      from++;
      v = (v << 1) | b;
    }
    m->u4.bits[w] = (short)v;
  }
  return from;
}
static unsigned short g_out[4096];

static void set_m(FaxVmi *o, int s0c, int s0e, int s10)
{
  o->m->s0c = (short)s0c;
  o->m->s0e = (short)s0e;
  o->m->u10.s10 = (short)s10;
}
static void out_reset(void)
{
  unsigned i;
  for (i = 0; i < 4096; i++) g_out[i] = 0xa5a5;
}

/* simple / asynchronous / hdlc packing on random states */
static void framing_pack(int mode, int trials)
{
  int t;
  for (t = 0; t < trials; t++) {
    FaxVmiCfg c;
    FaxVmi *o;
    int d8, da, dc;
    d8 = 3 + (int)rmod(38);
    da = (int)rmod(10);
    dc = (int)rmod(24);
    c = mkcfg(mode, 0, d8, da, dc, 0, NULL);
    o = vmi_new(c, 0);
    FaxVmiSt *s = o->st;
    unsigned short in[400];
    unsigned i, calls = 1 + rmod(5), k;
    int dbg = 0;
    if (mode == 2 && rpct(50)) dbg = (int)rmod(4);
    emit(0, "PACK mode %d trial %d dbg %d", mode, t, dbg);
    {
      int a, b;
      a = 1 + (int)rmod(40);
      b = 1 + (int)rmod(16);
      set_m(o, a, b, 4);
    }
    if (rpct(60)) poke_tx(s);
    if (mode == 2 && rpct(70)) s->tx_flag = rpct(50);
    if (rpct(50)) {
      unsigned rd0, cn0;
      rd0 = rmod(s->size);
      cn0 = rmod((unsigned)s->size + 1);
      ring_set(s, rd0, cn0);
    }
    if (rpct(10)) o->txEmpty = (int)rnd();
    dbg_level((unsigned)dbg);
    for (k = 0; k < calls; k++) {
      int r;
      short n;
      unsigned pos = 0, f;
      if (rpct(20) && k > 0) {
        dump_vmi("c", o);
        n = 0;
      } else if (mode == 2) {
        unsigned nf = rmod(4);
        for (i = 0; i < 400; i++) in[i] = 0;
        for (f = 0; f < nf && pos < 300; f++) {
          unsigned len = 1 + rmod(8);
          unsigned char d[16];
          unsigned short fc;
          in[pos++] = (unsigned short)len;
          for (i = 0; i < len; i++) {
            d[i] = (unsigned char)(rpct(15) ? 0xff : rnd());
            in[pos++] = d[i];
          }
          (void)fc;
        }
        n = (short)(rpct(10) ? nf + 1 : nf);
      } else {
        unsigned nb = rmod(10);
        for (i = 0; i < 400; i++) in[i] = (unsigned short)(rpct(70) ? rmod(256) : rnd());
        if (rpct(30)) in[0] = 0xff;
        n = (short)nb;
      }
      if (dbg && s->size && s->rd == s->size - 1 && n != 0) cov_hit(CV_F_HDLC_WRAPDBG);
      if (rpct(10) && mode == 1) {
        s->breakReq = 1;
        s->breakLeft = (unsigned short)rmod(3);
      }
      if (mode == 0 && s->cnt == 0) cov_hit(CV_F_SIMP_UNDER);
      if (mode == 1 && s->breakReq && s->breakLeft) cov_hit(CV_F_ASYC_BREAK);
      if (mode == 1 && s->breakReq && !s->breakLeft) cov_hit(CV_F_ASYC_BREAKEND);
      if (mode == 1 && !s->breakReq && s->cnt == 0) cov_hit(CV_F_ASYC_EMPTY);
      if (mode == 1 && !s->breakReq && s->cnt != 0) cov_hit(CV_F_ASYC_DATA);
      if (mode == 0 && s->cnt != 0) cov_hit(CV_F_SIMP_DATA);
      if (mode == 0 && s->tx_mask != 0) cov_hit(CV_F_SIMP_MID);
      if (mode == 0) r = faxvmi_simp_pack(o, in, n), cov_hit(CV_F_SIMP_PACK);
      else if (mode == 1) r = faxvmi_asyc_pack(o, in, n), cov_hit(CV_F_ASYC_PACK), cov_hit(CV_F_ASYC_CHAR);
      else r = faxvmi_hdlc_frame(o, in, n), cov_hit(CV_F_HDLC_PACK);
      if (mode == 2) {
        if (s->tx_flag && s->tx_mask) cov_hit(CV_F_HDLC_IDLE);
        if (!s->tx_flag) cov_hit(CV_F_HDLC_DATA);
        if (s->tx_left) cov_hit(CV_F_HDLC_START);
        if (s->tx_acc && (s->tx_acc & 0x1f) == 0x1f) cov_hit(CV_F_HDLC_STUFF);
      }
      emit(0, "pack r=%d n=%d", r, (int)n);
      dump_vmi("post", o);
    }
    dbg_level(0);
    FAXVMI_delete(o);
    trial_end("pack");
  }
}

/* a hdlc_frame run over real frames (so that the HDLC bit stream contains flags, stuffing and CRC): the bits of up to 40 calls are appended to g_bits */
static void hdlc_tx_stream(FaxVmi *o, unsigned nframes)
{
  unsigned f, i, call;
  unsigned short in[64];
  unsigned pos = 0;
  for (i = 0; i < 64; i++) in[i] = 0;
  for (f = 0; f < nframes; f++) {
    unsigned len = 3 + rmod(9);
    in[pos++] = (unsigned short)len;
    for (i = 0; i < len; i++) in[pos++] = (unsigned short)(i == 0 ? 0xff : (i == 1 ? 0xc8 : (rpct(25) ? 0xff : rmod(256))));
  }
  for (call = 0; call < 40; call++) {
    unsigned w, k;
    faxvmi_hdlc_frame(o, in, (short)(call == 0 ? nframes : 0));
    for (w = 0; w < (unsigned)o->m->s0c; w++) {
      unsigned v = (unsigned short)o->m->u4.bits[w];
      for (k = 0; k < (unsigned)o->m->s0e; k++) g_bits[g_nbits++] = (unsigned char)((v >> (o->m->s0e - 1 - k)) & 1);
    }
    if (call > 4 && o->st->cnt == 0 && o->st->tx_left == 0) break;
  }
}

static void framing_unpack_simple(int mode, int trials)
{
  int t;
  for (t = 0; t < trials; t++) {
    FaxVmiCfg c;
    FaxVmi *o;
    int da, dc;
    da = (int)rmod(14);
    dc = (int)rmod(4);
    c = mkcfg(mode, 0, 3, da, dc, 0, NULL);
    o = vmi_new(c, 0);
    FaxVmiSt *s = o->st;
    unsigned calls = 1 + rmod(6), k, width = 1 + rmod(16);
    int stream = rpct(60);
    unsigned pos = 0;
    emit(0, "UNPACK mode %d trial %d width %u stream %d", mode, t, width, stream);
    set_m(o, 4, 4, (int)width);
    if (rpct(50)) poke_rx(s);
    if (stream) {
      /* model characters: start 0, 8 data bits MSB first, stop 1, idle ones, sometimes a break (zeros) */
      unsigned nch = 3 + rmod(20), i;
      g_nbits = 0;
      if (mode == 1) {
        if (rpct(30)) {
          unsigned z = 14 + rmod(18);
          while (z--) bput(0);
          bput(1);
        }
        for (i = 0; i < nch; i++) {
          unsigned v = rmod(256);
          if (rpct(15)) {
            unsigned z = 14 + rmod(18);
            while (z--) bput(0);
            cov_hit(CV_F_ASYC_BREAKFLAG);
            bput(1);
          }
          bput(0);
          bput_byte(v);
          bput(1);
          if (rpct(30)) {
            unsigned z = rmod(5);
            while (z--) bput(1);
          }
        }
        cov_hit(CV_F_ASYC_FRAMING);
      } else {
        for (i = 0; i < nch; i++) bput_byte(rmod(256));
      }
    }
    for (k = 0; k < calls; k++) {
      int r;
      unsigned nw = rmod(51);
      unsigned i;
      out_reset();
      if (stream)
        pos = bits_to_words(o->m, pos, nw, width);
      else
        for (i = 0; i < 50; i++) o->m->u4.bits[i] = (short)(rpct(40) ? (rpct(50) ? 0 : 0xffff) : (int)rnd());
      if (mode == 1) {
        cov_hit(CV_F_ASYC_UNPACK);
        r = faxvmi_asyc_unpack(o, g_out, (short)nw);
        if (s->rx_state == 0) cov_hit(CV_F_ASYC_STATE0);
        if (s->rx_state != 0) cov_hit(CV_F_ASYC_HUNT);
        if (o->rxOverflow) cov_hit(CV_F_ASYC_OVER);
        if (s->rxBreak) cov_hit(CV_F_ASYC_ZEROS);
      } else {
        cov_hit(CV_F_SIMP_UNPACK);
        r = faxvmi_simp_unpack(o, g_out, (short)nw);
        if (o->rxOverflow) cov_hit(CV_F_SIMP_OVER);
      }
      emit(0, "unpack nw=%u r=%d out=%016llx", nw, r, fnv_shorts((short *)g_out, 64));
      dump_vmi("post", o);
    }
    FAXVMI_delete(o);
    trial_end("unpack");
  }
}

/* asynchronous receive: a sweep of the start states (data / hunting, bits already collected, shift register clear or not) against runs of 0 .. 40 zero bits (the 23 zero break rule) followed by a one and a character */
static void asyc_sweep(void)
{
  int st0, accz, nb0, z;
  for (st0 = 0; st0 < 2; st0++)
    for (accz = 0; accz < 2; accz++)
      for (nb0 = 0; nb0 < 9; nb0++)
        for (z = 0; z <= 40; z++) {
          FaxVmiCfg c;
          FaxVmi *o;
          unsigned width, nw, i;
          int r;
          c = mkcfg(1, 0, 3, 8, 1, 0, NULL);
          o = vmi_new(c, 0);
          width = 2 + rmod(15);
          set_m(o, 4, 4, (int)width);
          o->st->rx_state = st0;
          o->st->rx_nbits = (unsigned short)nb0;
          o->st->rx_acc = accz ? 0 : (int)rnd();
          o->st->rx_mask = 0;
          g_nbits = 0;
          for (i = 0; (int)i < z; i++) bput(0);
          bput(1);
          bput(0);
          bput_byte(rmod(256));
          bput(1);
          bput(1);
          nw = (g_nbits + width - 1) / width;
          if (nw > 50) die3("sweep stream too long");
          bits_to_words(o->m, 0, nw, width);
          out_reset();
          r = faxvmi_asyc_unpack(o, g_out, (short)nw);
          cov_hit(CV_F_ASYC_UNPACK);
          emit(0, "SWEEP st %d accz %d nb %d z %d width %u r=%d out=%016llx", st0, accz, nb0, z, width, r, fnv_shorts((short *)g_out, 64));
          dump_vmi("sw", o);
          FAXVMI_delete(o);
          trial_end("asyc sweep");
        }
}

static void framing_hdlc_unframe(int trials)
{
  int t;
  for (t = 0; t < trials; t++) {
    int kind = t % 15;
    int s0a = (kind == 11 || rpct(8)) ? (int)rmod(8) : 20 + (int)rmod(30);
    int s0c = (kind == 10 || rpct(8)) ? 2 + (int)rmod(6) : 40 + (int)rmod(60);
    FaxVmiCfg c = mkcfg(2, 0, 3, s0a, s0c, 0, NULL);
    FaxVmi *o = vmi_new(c, 0);
    FaxVmiSt *s = o->st;
    unsigned width = 1 + rmod(16), nfr = 1 + rmod(4), f, k, pos = 0;
    int dbg = 0;
    if (rpct(60)) dbg = (int)rmod(4);
    unsigned calls_total;
    emit(0, "HU trial %d kind %d width %u s0a %d s0c %d dbg %d", t, kind, width, s0a, s0c, dbg);
    set_m(o, 4, 4, (int)width);
    dbg_level((unsigned)dbg);
    if (dbg > 1) cov_hit(CV_F_HU_DBG);
    g_nbits = 0;
    if (kind == 9) { /* a state poke: a few flags still to skip, an old run of ones */
      s->hskip = (unsigned short)(1 + rmod(3));
      s->ones = (short)rmod(7);
      s->in_frame = rpct(50);
      cov_hit(CV_F_HU_HSKIP);
    }
    if (kind == 8) { /* loopback through the transmitter of this layer */
      FaxVmi *o2 = vmi_new(mkcfg(2, 0, 70, 10, 0, 0, NULL), 0);
      set_m(o2, 20 + (int)rmod(20), 1, 1);
      hdlc_tx_stream(o2, nfr);
      FAXVMI_delete(o2);
      cov_hit(CV_F_LOOP);
    } else if (kind == 7) { /* only flags / noise */
      unsigned i, nf = 3 + rmod(6);
      for (i = 0; i < nf; i++) bput_flag();
      if (rpct(50))
        for (i = 0; i < 80; i++) bput(rpct(50));
      cov_hit(CV_F_HU_FLAGSONLY);
    } else if (kind == 6) { /* pure noise, ones runs, seven ones */
      unsigned i, nb = 100 + rmod(300);
      for (i = 0; i < 6; i++) bput_flag();
      for (i = 0; i < nb; i++) bput(rpct(75));
      for (i = 0; i < 7; i++) bput(1);
      bput(0);
      bput_flag();
      cov_hit(CV_F_HU_SEVEN);
    } else {
      for (f = 0; f < nfr; f++) {
        unsigned char d[100];
        unsigned len = 2 + rmod(kind == 10 ? 12 : 30), i;
        int crc_ok = 1, junk = 0, corrupt0 = 0, corrupt1 = 0;
        for (i = 0; i < len; i++) d[i] = (unsigned char)(rpct(15) ? 0xff : rnd());
        d[0] = 0xff;
        if (rpct(60)) d[1] = 0xc8;
        switch (kind) {
          case 1: crc_ok = 0; break;                       /* wrong CRC: no repair possible */
          case 2: corrupt0 = 1; break;                     /* first byte changed on the line (the original is 0xff) */
          case 3: corrupt1 = 1; d[1] = 0xc8; break;        /* second byte changed (the original is 0xc8) */
          case 4: junk = 1 + (int)rmod(7); break;          /* shifted by `junk` bits */
          case 5: junk = 1 + (int)rmod(7); corrupt0 = 1; break;
          case 12: junk = 1 + (int)rmod(7); corrupt1 = 1; d[1] = 0xc8; break;
          case 13: corrupt0 = 1; corrupt1 = 1; d[1] = 0xc8; break;
          case 14: junk = 1 + (int)rmod(7); corrupt0 = 1; corrupt1 = 1; d[1] = 0xc8; break;
          case 0: if (rpct(30)) junk = 1 + (int)rmod(7); break;
          default: break;
        }
        {
          unsigned char tmp[128];
          unsigned short fc = crc_model(d, len);
          memcpy(tmp, d, len);
          tmp[len] = (unsigned char)(fc >> 8);
          tmp[len + 1] = (unsigned char)fc;
          if (!crc_ok) tmp[len + 1] ^= (unsigned char)(1u << rmod(8));
          if (corrupt0) tmp[0] = (unsigned char)(rnd() | 1);
          if (corrupt1) tmp[1] = (unsigned char)(rnd() ^ 0x3c);
          {
            int pre = f == 0 ? 3 + (int)rmod(3) : (int)(1 + rmod(2));
            for (i = 0; i < (unsigned)pre; i++) bput_flag();
            for (i = 0; i < (unsigned)junk; i++) bput(0);
            bput_stuffed(tmp, len + 2);
            if (f + 1 == nfr) {
              bput_flag();
              if (rpct(50)) bput_flag();
            }
          }
        }
      }
    }
    calls_total = (g_nbits + width - 1) / width + 2;
    {
      unsigned frames_total = 0;
      pos = 0;
      for (k = 0; calls_total > 0;) {
        unsigned nw = 1 + rmod(rpct(30) ? 6 : 50);
        int r;
        if (nw > calls_total) nw = calls_total;
        pos = bits_to_words(o->m, pos, nw, width);
        out_reset();
        r = faxvmi_hdlc_unframe(o, g_out, (short)nw);
        cov_hit(CV_F_HDLC_UNPACK);
        calls_total -= nw;
        frames_total += (unsigned)r;
        if (s->ones == 7) cov_hit(CV_F_HU_SEVEN);
        if (r > 0) cov_hit(CV_F_HU_FRAME_OK);
        {
          int q;
          unsigned pw = 0;
          for (q = 0; q < r && pw < 2000; q++) {
            if (g_out[pw] == 0) cov_hit(CV_F_HU_GIVEUP);
            pw += 1u + g_out[pw];
          }
        }
        if (r > 1) cov_hit(CV_F_HU_MULTI);
        if (o->rxOverflow) cov_hit(CV_F_HU_OVER);
        if (s->flen >= (int)s->frm_size && s->frm_size) cov_hit(CV_F_HU_CAP);
        if (s->hskip == 0 && s->ones == 0) cov_hit(CV_F_HU_FLAG_SKIP);
        emit(0, "hu nw=%u r=%d out=%016llx", nw, r, fnv_shorts((short *)g_out, 1024));
        dump_vmi("post", o);
        k++;
        if (k > 400) break;
      }
      emit(0, "hu frames %u bits %u", frames_total, g_nbits);
      if (g_had_stuff) cov_hit(CV_F_HU_STUFFED);
      g_had_stuff = 0;
    }
    dbg_level(0);
    FAXVMI_delete(o);
    trial_end("hdlc unframe");
  }
}

static void scen_framing(void)
{
  framing_pack(0, 500);
  framing_pack(1, 500);
  framing_pack(2, 700);
  framing_unpack_simple(0, 300);
  framing_unpack_simple(1, 500);
  asyc_sweep();
  framing_hdlc_unframe(900);
}
