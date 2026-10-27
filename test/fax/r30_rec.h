/* R30 harness part 1 (R30_CLASS1 build): the recorders of the class 1 layer's callees, the object generators and the canonical dumps.  Included by fax_api.main.c. */
#ifdef R30_CLASS1


/* ---------------------------------------------------------------------------------------------- the script the recorders follow */
typedef struct {
  /* state function recorders */
  int st_set; /* 1 state, 2 w122c, 4 rx_kind, 8 rx_event, 16 n_out, 32 p7, 64 p8, 128 tmr, 512 *n_in (the copy) */
  int st_state, st_w122c, st_kind, st_event, st_nout, st_p7, st_p8, st_tmr, st_nin;
  int st_ret;
  /* the initialiser / delete recorders poke markers into the object (the order of the stores of the code under test becomes visible) */
  int mk;     /* 1 state, 2 modem_sel, 4 tmr, 8 chan, 16 w12b8, 32 silence_rms, 64 sf_fill, 128 emu_delay */
  int mk_val;
  /* creation recorders */
  unsigned vmi_null;  /* bit k: the k-th FAXVMI_create since script_reset returns NULL */
  unsigned tone_null; /* bit k: the k-th FPM_TONE_create returns NULL */
  unsigned vmi_calls, tone_calls;
} Script;
static Script g_sc;
static unsigned long g_st_calls[19], g_init_calls[8], g_nvmi, g_ntone;
static int g_kind_seen; /* rx_kind as the state function left it */
static void script_reset(void)
{
  memset(&g_sc, 0, sizeof g_sc);
  dsplibs_debug_level = g_lvl_base;
  g_dbg_hook = NULL;
  g_snap = NULL;
}

/* ---------------------------------------------------------------------------------------------- canonical description of an object */
/* the fake data receivers of the progress tests (typed objects at the blob's i386 layout at -m32) */
typedef struct { FaxVmi v; FaxVmiM m; V27Rx e; V27RxBundle b; } Fk27;
typedef struct { FaxVmi v; FaxVmiM m; V29Rx e; V29RxBundle b; } Fk29;
typedef struct { FaxVmi v; FaxVmiM m; V17Rx e; V17RxBundle b; } Fk17;
static Fk27 g_k27;
static Fk29 g_k29;
static Fk17 g_k17;
static short g_hp_dummy[10];
static int g_dummy_a, g_dummy_b;

static void rec_names(void)
{
  htab_add("fk27", &g_k27.v, sizeof g_k27.v);
  htab_add("fk29", &g_k29.v, sizeof g_k29.v);
  htab_add("fk17", &g_k17.v, sizeof g_k17.v);
  htab_add("hpdummy", g_hp_dummy, sizeof g_hp_dummy);
  htab_add("dA", &g_dummy_a, sizeof g_dummy_a);
  htab_add("dB", &g_dummy_b, sizeof g_dummy_b);
}
static u64 snap_cur(void) { return g_cur ? h30(g_cur) : 0ULL; }
static void dump_cfg_block(const char *tag, const FaxVmiCfg *c)
{
  char p1[24];
  if (!c) {
    emit(0, "%s CFG NULL", tag);
    return;
  }
  emit(0, "%s CFG mode=%d flag=%d s08=%u s0a=%u s0c=%u id=%u mcfg=%s mcfgh=%016llx w14=%d", tag, c->mode, c->flag, (unsigned)c->ringMin, (unsigned)c->maxItems, (unsigned)c->frmWords, (unsigned)c->id,
       pname(c->mcfg, p1, sizeof p1), blk_hash(c->mcfg), c->w14);
}
/* the state table: slot k holds recorder k (the real function of a blob-LOCAL slot is accepted as such and reported as "real") */
static StateFn g_wrapfn[19];
static void dump_table(const char *tag)
{
  int i;
  char b[200];
  int k = 0;
  b[0] = 0;
  for (i = 0; i < 19; i++) {
    int id = -1;
    int j;
    for (j = 0; j < 19; j++)
      if (STATETAB[i] == g_wrapfn[j]) id = j;
#ifdef TEST_BLOB
    if (i == 8 && STATETAB[i] == _idle_state) id = 8;
    if (i == 14 && STATETAB[i] == _answer_tone_state) id = 14;
    if (i == 15 && STATETAB[i] == _send_silence_state) id = 15;
    if (i == 16 && STATETAB[i] == _recieve_silence_state) id = 16;
#endif
    k += snprintf(b + k, sizeof b - (size_t)k, " %d", id);
  }
  emit(0, "%s TABLE%s", tag, b);
  cov_hit(CV_C0_TABLE);
}
/* the blob's four LOCAL state functions are replaced by recorders after the table identity was checked (blob run only) */
#ifdef TEST_BLOB
int h_st8(FaxT30 *, short *, short *, unsigned char *, const unsigned char *, int *, int *, int *, int *);
int h_st14(FaxT30 *, short *, short *, unsigned char *, const unsigned char *, int *, int *, int *, int *);
int h_st15(FaxT30 *, short *, short *, unsigned char *, const unsigned char *, int *, int *, int *, int *);
int h_st16(FaxT30 *, short *, short *, unsigned char *, const unsigned char *, int *, int *, int *, int *);
#endif
static void post_create(void)
{
#ifdef TEST_BLOB
  STATETAB[8] = h_st8;
  STATETAB[14] = h_st14;
  STATETAB[15] = h_st15;
  STATETAB[16] = h_st16;
#endif
}

/* ---------------------------------------------------------------------------------------------- the callee recorders */
static void marks(FaxT30 *o, int tag)
{
  int v = g_sc.mk_val * 31 + tag;
  if (g_sc.mk & 1) o->state = v & 15;
  if (g_sc.mk & 2) o->modem_sel = 0x7000 + v;
  if (g_sc.mk & 4) o->tmr = 0x5000 + v;
  if (g_sc.mk & 8) o->chan = (v & 1) + 1;
  if (g_sc.mk & 16) o->silenceTime = 0x6000 + v;
  if (g_sc.mk & 32) o->silence_rms = 0x4000 + v;
  if (g_sc.mk & 64) o->sf_fill = 0x3000 + v;
  if (g_sc.mk & 128) o->emu_delay = 0x2000 + v;
}
static int rec_state(int idx, FaxT30 *o, short *rx, short *tx, unsigned char *tout, const unsigned char *tin, int *n_in, int *n_out, int *p7, int *p8)
{
  g_st_calls[idx]++;
  emit(0, "ST %d state=%d prev=%d n_in=%d n_out=%d p7=%d p8=%d rx=%016llx tx=%016llx tin=%016llx obj=%016llx", idx, o->state, o->state_prev, *n_in, *n_out, *p7, *p8,
       fnv_shorts(H0, rx, 64), fnv_shorts(H0, tx, 16), fnv_bytes(tin, 32), h30(o));
  if (g_sc.st_set & 512) *n_in = g_sc.st_nin;
  if (g_sc.st_set & 1) o->state = g_sc.st_state;
  if (g_sc.st_set & 2) o->status = g_sc.st_w122c;
  if (g_sc.st_set & 4) o->rx_kind = g_sc.st_kind;
  if (g_sc.st_set & 8) o->rx_event = g_sc.st_event;
  if (g_sc.st_set & 16) *n_out = g_sc.st_nout;
  if (g_sc.st_set & 32) *p7 = g_sc.st_p7;
  if (g_sc.st_set & 64) *p8 = g_sc.st_p8;
  if (g_sc.st_set & 128) o->tmr = g_sc.st_tmr;
  tx[0] = (short)(0x1100 + idx);
  tout[0] = (unsigned char)idx;
  g_kind_seen = o->rx_kind;
  return g_sc.st_ret;
}
#define STFN(name, idx) \
  int name(FaxT30 *o, short *rx, short *tx, unsigned char *tty_out, const unsigned char *tty_in, int *n_in, int *n_out, int *p7, int *p8) \
  { \
    return rec_state(idx, o, rx, tx, tty_out, tty_in, n_in, n_out, p7, p8); \
  }
STFN(_t30_silence_before_tx_state, 0)
STFN(_t30_preabmle_state, 1)
STFN(_send_hdlc_buffer_state, 2)
STFN(_send_hdlc_between_buffer_state, 3)
STFN(_hdlc_receive_look_carrier_state, 4)
STFN(_hdlc_receive_state, 5)
STFN(_hdlc_receive_between_buffers_state, 6)
STFN(_hdlc_emulate_receive_state, 7)
STFN(_tx_scrambled_ones_state, 9)
STFN(_tx_data_state, 10)
STFN(_tx_nulls_state, 11)
STFN(_rx_look_carrier_state, 12)
STFN(_rx_data_state, 13)
STFN(cHDLCtx_off, 17)
STFN(_tx_silence_before_scrm_ones, 18)
#ifdef TEST_BLOB
STFN(h_st8, 8)
STFN(h_st14, 14)
STFN(h_st15, 15)
STFN(h_st16, 16)
#else
STFN(fax_idle_state, 8)
STFN(fax_answer_tone_state, 14)
STFN(fax_send_silence_state, 15)
STFN(fax_recieve_silence_state, 16)
#endif
static void wrapfn_init(void)
{
  g_wrapfn[0] = _t30_silence_before_tx_state;
  g_wrapfn[1] = _t30_preabmle_state;
  g_wrapfn[2] = _send_hdlc_buffer_state;
  g_wrapfn[3] = _send_hdlc_between_buffer_state;
  g_wrapfn[4] = _hdlc_receive_look_carrier_state;
  g_wrapfn[5] = _hdlc_receive_state;
  g_wrapfn[6] = _hdlc_receive_between_buffers_state;
  g_wrapfn[7] = _hdlc_emulate_receive_state;
#ifdef TEST_BLOB
  g_wrapfn[8] = h_st8;
  g_wrapfn[14] = h_st14;
  g_wrapfn[15] = h_st15;
  g_wrapfn[16] = h_st16;
#else
  g_wrapfn[8] = fax_idle_state;
  g_wrapfn[14] = fax_answer_tone_state;
  g_wrapfn[15] = fax_send_silence_state;
  g_wrapfn[16] = fax_recieve_silence_state;
#endif
  g_wrapfn[9] = _tx_scrambled_ones_state;
  g_wrapfn[10] = _tx_data_state;
  g_wrapfn[11] = _tx_nulls_state;
  g_wrapfn[12] = _rx_look_carrier_state;
  g_wrapfn[13] = _rx_data_state;
  g_wrapfn[17] = cHDLCtx_off;
  g_wrapfn[18] = _tx_silence_before_scrm_ones;
}

/* the initialisers / deleters (the receive initialiser creates a typed fake receiver and the delete functions drop the data channel, as the real ones do: the accept-rate check of
 * fax_class1_progress reads the receiver's flag) */
static void fake_chan(FaxT30 *o, int mod, int q4e);
#define INIT_REC(name, id, fmt, arg) \
  { \
    g_init_calls[id]++; \
    emit(0, "INIT " #name " obj=%016llx " fmt, h30(o), arg); \
    marks(o, id); \
  }
void _cHDLCrx_init_from_idle(FaxT30 *o, int mode) INIT_REC(_cHDLCrx_init_from_idle, 0, "mode=%d", mode)
int _hdlc_receive_state_init(FaxT30 *o) { INIT_REC(_hdlc_receive_state_init, 1, "%s", "-") return 0; }
int _rx_look_carrier_init(FaxT30 *o, int modem)
{
  INIT_REC(_rx_look_carrier_init, 2, "modem=%d", modem)
  o->data_cfg = (FaxVmiM *)(void *)&g_dummy_a;
  fake_chan(o, modem, (int)(g_sc.mk_val & 3));
  return 0;
}
void _delete_data_rx_modem(FaxT30 *o)
{
  INIT_REC(_delete_data_rx_modem, 3, "%s", "-")
  o->data_cfg = NULL;
  o->data_vmi = NULL;
}
void _delete_data_tx_modem(FaxT30 *o)
{
  INIT_REC(_delete_data_tx_modem, 4, "%s", "-")
  o->data_cfg = NULL;
  o->data_vmi = NULL;
}
int _tx_scrambled_ones_init(FaxT30 *o, int modem)
{
  INIT_REC(_tx_scrambled_ones_init, 5, "modem=%d", modem)
  o->data_cfg = (FaxVmiM *)(void *)&g_dummy_a;
  o->data_vmi = (FaxVmi *)(void *)&g_dummy_b;
  return 0;
}
int cHDLCtx_preamble_state_init(FaxT30 *o) { INIT_REC(cHDLCtx_preamble_state_init, 6, "%s", "-") return 0; }

FaxVmi *FAXVMI_create(FaxVmi *o, const FaxVmiCfg *cfg)
{
  char b1[24], b2[24];
  unsigned k = g_sc.vmi_calls++;
  g_nvmi++;
  emit(0, "VC o=%s cfg=%s mode=%d flag=%d s08=%u s0a=%u s0c=%u id=%u mcfg=%s mcfgh=%016llx w14=%d", o ? "supplied" : "NULL", pname(cfg, b1, sizeof b1), cfg->mode, cfg->flag,
       (unsigned)cfg->ringMin, (unsigned)cfg->maxItems, (unsigned)cfg->frmWords, (unsigned)cfg->id, pname(cfg->mcfg, b2, sizeof b2), blk_hash(cfg->mcfg), cfg->w14);
  if (g_sc.vmi_null & (1u << k)) return NULL;
  if (!o) {
    o = (FaxVmi *)sysdep_malloc((unsigned)sizeof(FaxVmi));
    if (!o) return NULL;
  }
  memset(o, 0, sizeof *o);
  o->cfg = *cfg;
  o->rxOverflow = (int)(k + 1);
  return o;
}
void FAXVMI_delete(FaxVmi *o)
{
  emit(0, "VD %u id=%u", hid(o), (unsigned)o->cfg.id);
  sysdep_free(o);
}
FpmTone *FPM_TONE_create(FpmTone *tone, const int *cfg)
{
  unsigned k = g_sc.tone_calls++;
  g_ntone++;
  emit(0, "TC tone=%s cfg %08x %08x %08x %08x %08x %08x %08x %08x %08x", tone ? "supplied" : "NULL", (unsigned)cfg[0], (unsigned)cfg[1], (unsigned)cfg[2], (unsigned)cfg[3], (unsigned)cfg[4],
       (unsigned)cfg[5], (unsigned)cfg[6], (unsigned)cfg[7], (unsigned)cfg[8]);
  if (g_sc.tone_null & (1u << k)) return NULL;
  return (FpmTone *)sysdep_malloc(40);
}
void FPM_TONE_delete(FpmTone *tone)
{
  emit(0, "TD %u", hid(tone));
  sysdep_free(tone);
}
void FIFO_delete(FaxFifo *f)
{
  emit(0, "FD %u", hid(f));
  sysdep_free(f);
}
void FPM_iir_filt_II(short *data, short *coef, short *state, short n, short count)
{
  emit(0, "IIR data=%016llx coef=%016llx state=%016llx n=%d count=%d", fnv_shorts(H0, data, 64), fnv_shorts(H0, coef, 10), fnv_bytes(state, 16), (int)n, (int)count);
  data[0] = (short)(data[0] ^ 0x55aa);
  ((unsigned char *)state)[0] = (unsigned char)(((unsigned char *)state)[0] + 1);
}

/* the constant records (blob: .rodata / .data through mprotect, port: the harness' own definitions) */
static void poke_consts(int randomize)
{
  FaxVmiCfg c;
  int tx[7], rx[6], tone[9];
  int i;
  memset(&c, 0, sizeof c);
  c.ringMin = 0x80;
  c.maxItems = 0x32;
  c.frmWords = 0x80;
  for (i = 0; i < 7; i++) tx[i] = i * 0x01010101 + 0x12c0001 * (i == 0);
  for (i = 0; i < 6; i++) rx[i] = i * 0x02020202 + 0x12c0000 * (i == 0);
  for (i = 0; i < 9; i++) tone[i] = 0x100000 * (i + 1) + 0x321 * i;
  if (randomize) {
    c.mode = (int)rnd();
    c.flag = (int)rnd();
    c.ringMin = (unsigned short)rnd();
    c.maxItems = (unsigned short)rnd();
    c.frmWords = (unsigned short)rnd();
    c.id = (unsigned short)rnd();
    c.w14 = (int)rnd();
    for (i = 0; i < 7; i++) tx[i] = (int)rnd();
    for (i = 0; i < 6; i++) rx[i] = (int)rnd();
    for (i = 0; i < 9; i++) tone[i] = (int)rnd();
  }
  c.mcfg = NULL;
  poke_bytes(&h_FAXVMI_CFG, &c, sizeof c);
  poke_bytes(h_V21TX_CFG, tx, sizeof tx);
  poke_bytes(h_V21RX_CFG, rx, sizeof rx);
  poke_bytes(h_FPM_TONE_CFG, tone, sizeof tone);
  emit(0, "CONSTS cfg=%d/%d/%u/%u/%u/%u/%d tx=%08x rx=%08x tone=%08x", c.mode, c.flag, (unsigned)c.ringMin, (unsigned)c.maxItems, (unsigned)c.frmWords, (unsigned)c.id, c.w14, (unsigned)tx[0],
       (unsigned)rx[0], (unsigned)tone[0]);
}

/* ---------------------------------------------------------------------------------------------- objects */
#define G32(f) o->f = (int)rnd()
/* a fresh object: every non-pointer member deterministic garbage (member by member: the same PRNG draws on both hosts), the pointers cleared */
static void t30_garbage(FaxT30 *o)
{
  unsigned i, r = 0;
  for (i = 0; i < 16; i++) g_wrap.pre[i] = (unsigned short)(0x7a00 + i);
  for (i = 0; i < 0x900; i++) {
    if ((i & 1) == 0) r = rnd();
    o->buf[i] = (unsigned short)(r >> (16 * (i & 1)));
  }
  o->hdlc_tx = o->hdlc_rx = o->data_vmi = NULL;
  o->hdlc_tx_cfg = o->hdlc_rx_cfg = NULL;
  o->data_cfg = NULL;
  o->tone = NULL;
  o->fifo = NULL;
  o->hpCoef = NULL;
  G32(ans_org); G32(state); G32(state_prev); G32(lastFrame); G32(tmr); G32(status); G32(w1230); G32(w1234); G32(lastByte); G32(rx_event); G32(rx_kind); G32(chan); G32(modemClass); G32(dlePending);
  G32(hdlcWr); G32(modem_sel); G32(cng_tone); G32(cng_ctr); G32(cng_on); G32(ans_len); G32(dataRate); G32(enableDelay); G32(sec); G32(ms); G32(rx_act); G32(frameDone); G32(frameCount); G32(txEnable); G32(frameWords);
  G32(txConnected); G32(fifoReady); G32(alignDone); o->alignHist = rnd(); G32(alignShift); o->alignMask = rnd(); G32(w12ac); G32(dataEnd); G32(s7); G32(silenceTime); G32(silence_rms); G32(agc_rcp); G32(agc_shift); G32(emu_delay);
  G32(emu_idx); G32(sf_fill); G32(cd_flag); G32(gain_att); G32(hp_on);
  for (i = 0; i < sizeof o->hpState; i++) {
    if ((i & 3) == 0) r = rnd();
    o->hpState[i] = (unsigned char)(r >> (8 * (i & 3)));
  }
}
static const int k_mods[] = {0x18, 0x30, 0x48, 0x60, 0x49, 0x4a, 0x61, 0x62, 0x79, 0x7a, 0x91, 0x92};
#define NMODS ((int)(sizeof k_mods / sizeof k_mods[0]))
/* a data channel of the typed fake receiver of the class of the modem value */
static void fake_chan(FaxT30 *o, int mod, int q4e)
{
  if (mod == 0x18 || mod == 0x30) {
    memset(&g_k27, 0, sizeof g_k27);
    g_k27.v.m = &g_k27.m;
    g_k27.m.eng = &g_k27.e;
    g_k27.e.bundle = &g_k27.b;
    g_k27.b.qualityOk = (short)q4e;
    o->data_vmi = &g_k27.v;
  } else if (mod == 0x48 || mod == 0x60) {
    memset(&g_k29, 0, sizeof g_k29);
    g_k29.v.m = &g_k29.m;
    g_k29.m.eng = &g_k29.e;
    g_k29.e.bundle = &g_k29.b;
    g_k29.b.qualityOk = (short)q4e;
    o->data_vmi = &g_k29.v;
  } else {
    memset(&g_k17, 0, sizeof g_k17);
    g_k17.v.m = &g_k17.m;
    g_k17.m.eng = &g_k17.e;
    g_k17.e.bundle = &g_k17.b;
    g_k17.b.qualityOk = (short)q4e;
    o->data_vmi = &g_k17.v;
  }
}
static int q4e_of(const FaxT30 *o, int mod)
{
  if (!o->data_vmi) return -1;
  if (mod == 0x18 || mod == 0x30) return g_k27.b.qualityOk;
  if (mod == 0x48 || mod == 0x60) return g_k29.b.qualityOk;
  return g_k17.b.qualityOk;
}
static void trial_end2(const char *what) { trial_end(what); }
/* fills the state table (the blob's COMMON object is empty until the first fax_class1_create) and replaces the blob's LOCAL state functions by recorders */
static void table_boot(void)
{
  FaxClass1Cfg c;
  FaxT30 *o;
  script_reset();
  poke_consts(0);
  memset(&c, 0, sizeof c);
  c.mode = 1;
  c.s7 = 7;
  c.hpFilter = 1;
  o = fax_class1_create(NULL, &c);
  if (!o) die3("boot create failed");
  post_create();
  fax_class1_delete(o);
  trial_end("boot");
}

#endif /* R30_CLASS1 */
