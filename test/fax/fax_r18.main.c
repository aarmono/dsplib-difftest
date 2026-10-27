/*
 * R18 differential harness: the fax infrastructure of dsplibs.o (FIFO_*, SGD_*, SDM_*, SDMv27_*, SMC_*, SMCv17_*, the Scramble / Descramble / Seed / SetEncoder / SetScrambler
 * wrappers of V.17 / V.27ter / V.29) vs src/fax/fax_fifo_sgd.c, fax_scrambler.c, fax_modem_wrappers.c, fax_common_tables.c.  The blob run (-DTEST_BLOB, linked with ../dsplibs.o) and the port run (-m32 -O0 /
 * -O2, and x86-64 against the recorded blob output) execute the same code; library test/fax/r18_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation,
 * deterministic heap with block ids / red zones / poisoned frees, callee recorders sysdep_malloc / sysdep_free).  Integer signals only (an LCG).  Nothing printed depends on
 * the host layout: objects are dumped member by member, pointers as table+index / block id.
 *
 * Scenarios (each its own process; argv[1]):
 *   0 tables: every R18 table dumped (values, the two SGD_CFG pointers), sizeof checks;
 *   1 FIFO: create (NULL object + NULL / custom cfg, caller-supplied object over garbage, sizes 0 .. 4096, malloc failure with size 0), write / read stress (empty, full, partial,
 *     wrap, underflow fill, n = 0 / 65535, poked count / rd / wr), checked against an independent queue model, delete;
 *   2 SGD: create (NULL object + NULL / custom cfg, caller-supplied object, bufn > 0 / 0 / < 0, hi16 variants), control (seq only, pat only, both, neither), sequence_gen (mode 0 / 1,
 *     len 0 .. 40, pos / pass pokes, n = 0 / negative), symbol_gen (bits 0 .. 33, pattern, w1c 0 .. 40, symbit pokes), delete, forked obj-malloc-failure probe;
 *   3 SDM (V.17 / V.29 scrambler): init over garbage, random configs, scrambler / descrambler streams with all kinds of seeds (an exhaustive 16-bit seed sweep included), round
 *     trips (model), the real configs;
 *   4 SDMv27: init (n = 2 / other, NULL cfg probe), scrambler / descrambler: random / constant / periodic streams, poked reg / cnt / f1 / f2 (the bit-serial guard path, the 0x21 trigger);
 *   5 SMC: init, SMC_encoder both outputs (w0 == 0 complex rotation, w0 != 0 phase index), flag 0 / 1, random cfgs with 64K-entry tables, wr / lim variants;
 *   6 SMCv17: init (NULL / cfg), abs / dif / tcm streams (k 1 .. 12, st / y2 / x any in their legal range, in-place masking checked);
 *   7 wrappers on fake modem objects (raw i386 layout at -m32, typed structs on x86-64): Scramble / Descramble V17 / V27 / V29, Seed, SetEncoder, SetScramblerV27.
 * R18_VERBOSE=1 prints every line (debugging), R18_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r18_common.h"
#include "fax/fax_modem_views.h"

#ifdef TEST_BLOB
extern const FaxFifoCfg FIFO_CFG;
#define FIFO_CFG_SYM FIFO_CFG
#define SIDE_BLOB 1
#else
#define FIFO_CFG_SYM fax_fifo_cfg
#define SIDE_BLOB 0
#endif

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  CV_TAB, CV_F_NEWOBJ, CV_F_SUPPLIED, CV_F_NULLCFG, CV_F_CUSTCFG, CV_F_SIZE0, CV_F_FAIL0, CV_F_W_FULL, CV_F_W_PART, CV_F_W_ZERO, CV_F_W_WRAP, CV_F_R_FILL, CV_F_R_PART, CV_F_R_ZERO,
  CV_F_R_WRAP, CV_F_BIGN, CV_F_POKE_CNT, CV_F_DELETE,
  CV_S_NEWOBJ, CV_S_SUPPLIED, CV_S_NULLCFG, CV_S_BUFN_POS, CV_S_BUFN_ZERO, CV_S_BUFN_NEG, CV_S_HI_NEG, CV_S_CTL_SEQ, CV_S_CTL_PAT, CV_S_CTL_NONE, CV_S_SEQ_MODE1, CV_S_SEQ_MODE0,
  CV_S_SEQ_CONST, CV_S_SEQ_WRAP, CV_S_SEQ_PASS, CV_S_SEQ_NEGN, CV_S_SYM_WRAP, CV_S_SYM_BITS0, CV_S_SYM_BIG, CV_S_SYM_W1C0, CV_S_DELETE, CV_S_PROBE,
  CV_D_INIT, CV_D_SCR, CV_D_DESCR, CV_D_ROUNDTRIP, CV_D_SEEDSWEEP, CV_D_N0, CV_D_NNEG_SHIFT, CV_D_REALCFG, CV_D_LONG,
  CV_V_INIT2, CV_V_INIT_OTHER, CV_V_NORMAL_S, CV_V_SPECIAL_CNT_S, CV_V_SPECIAL_FLAG_S, CV_V_TRIGGER_S, CV_V_E0_S, CV_V_MATCH_S, CV_V_NORMAL_D, CV_V_SPECIAL_CNT_D, CV_V_SPECIAL_FLAG_D,
  CV_V_TRIGGER_D, CV_V_E0_D, CV_V_MATCH_D, CV_V_F1SET, CV_V_F2SET, CV_V_CNTPOS, CV_V_LIMWRAP, CV_V_PROBE,
  CV_M_INIT, CV_M_W0, CV_M_W0Z, CV_M_FLAG, CV_M_NOFLAG, CV_M_MODWRAP, CV_M_BWRAP, CV_M_LIMWRAP, CV_M_NEG_A, CV_M_NEG_B, CV_M_EMPTY,
  CV_T_INITNULL, CV_T_INITCFG, CV_T_ABS, CV_T_DIF, CV_T_TCM, CV_T_ST_HI, CV_T_ST_LO, CV_T_LIMWRAP, CV_T_MASKED, CV_T_NEGB0, CV_T_K,
  CV_W_SCR17, CV_W_SCR27, CV_W_SCR29, CV_W_DES17, CV_W_DES27, CV_W_DES29, CV_W_SEED17, CV_W_SEED29, CV_W_ENC17_0, CV_W_ENC17_1, CV_W_ENC17_2, CV_W_ENC17_X, CV_W_ENC29_0, CV_W_ENC29_1,
  CV_W_ENC29_X, CV_W_SET27_0, CV_W_SET27_1
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0: cov_def(CV_TAB, "tables dumped"); break;
    case 1:
      cov_def(CV_F_NEWOBJ, "FIFO_create NULL object (alloc)");
      cov_def(CV_F_SUPPLIED, "FIFO_create caller-supplied object");
      cov_def(CV_F_NULLCFG, "FIFO_create NULL cfg");
      cov_def(CV_F_CUSTCFG, "FIFO_create custom cfg");
      cov_def(CV_F_SIZE0, "FIFO_create size 0");
      cov_def(CV_F_FAIL0, "FIFO_create buffer malloc failure (size 0)");
      cov_def(CV_F_W_FULL, "FIFO_write into a full FIFO (0 written)");
      cov_def(CV_F_W_PART, "FIFO_write truncated to the free space");
      cov_def(CV_F_W_ZERO, "FIFO_write n = 0");
      cov_def(CV_F_W_WRAP, "FIFO_write wraps the write index");
      cov_def(CV_F_R_FILL, "FIFO_read pads with fill");
      cov_def(CV_F_R_PART, "FIFO_read less than available");
      cov_def(CV_F_R_ZERO, "FIFO_read n = 0");
      cov_def(CV_F_R_WRAP, "FIFO_read wraps the read index");
      cov_def(CV_F_BIGN, "FIFO n >= 32768");
      cov_def(CV_F_POKE_CNT, "poked count > size");
      cov_def(CV_F_DELETE, "FIFO_delete");
      break;
    case 2:
      cov_def(CV_S_NEWOBJ, "SGD_create NULL object (2 mallocs)");
      cov_def(CV_S_SUPPLIED, "SGD_create caller-supplied object");
      cov_def(CV_S_NULLCFG, "SGD_create NULL cfg (SGD_CFG)");
      cov_def(CV_S_BUFN_POS, "bufn > 0 (buffer cleared)");
      cov_def(CV_S_BUFN_ZERO, "bufn == 0");
      cov_def(CV_S_BUFN_NEG, "bufn < 0");
      cov_def(CV_S_HI_NEG, "hi16(w24) > 0x4000 (negative factor)");
      cov_def(CV_S_CTL_SEQ, "SGD_control seq only");
      cov_def(CV_S_CTL_PAT, "SGD_control pat only");
      cov_def(CV_S_CTL_NONE, "SGD_control neither");
      cov_def(CV_S_SEQ_MODE1, "sequence_gen mode != 0");
      cov_def(CV_S_SEQ_MODE0, "sequence_gen mode == 0 (table then constant)");
      cov_def(CV_S_SEQ_CONST, "sequence_gen constant phase");
      cov_def(CV_S_SEQ_WRAP, "sequence_gen mode 1 table wrap");
      cov_def(CV_S_SEQ_PASS, "sequence_gen mode 0 pass flag set");
      cov_def(CV_S_SEQ_NEGN, "sequence_gen negative n");
      cov_def(CV_S_SYM_WRAP, "symbol_gen index wrap");
      cov_def(CV_S_SYM_BITS0, "symbol_gen bits 0");
      cov_def(CV_S_SYM_BIG, "symbol_gen bits >= 17");
      cov_def(CV_S_SYM_W1C0, "symbol_gen w1c 0");
      cov_def(CV_S_DELETE, "SGD_delete");
      cov_def(CV_S_PROBE, "forked obj-malloc-failure probe");
      break;
    case 3:
      cov_def(CV_D_INIT, "SDM_init over garbage");
      cov_def(CV_D_SCR, "SDM_scrambler");
      cov_def(CV_D_DESCR, "SDM_descrambler");
      cov_def(CV_D_ROUNDTRIP, "round trip scramble -> descramble (model)");
      cov_def(CV_D_SEEDSWEEP, "16-bit seed sweep");
      cov_def(CV_D_N0, "n == 0");
      cov_def(CV_D_NNEG_SHIFT, "negative shift counts (tap < n)");
      cov_def(CV_D_REALCFG, "SDM_CFG / V.17 / V.29 configs");
      cov_def(CV_D_LONG, "long stream");
      break;
    case 4:
      cov_def(CV_V_INIT2, "SDMv27_init n == 2");
      cov_def(CV_V_INIT_OTHER, "SDMv27_init n != 2");
      cov_def(CV_V_NORMAL_S, "scrambler normal path");
      cov_def(CV_V_SPECIAL_CNT_S, "scrambler bit path because cnt >= 0x21 - n");
      cov_def(CV_V_SPECIAL_FLAG_S, "scrambler bit path because f1/f2 set");
      cov_def(CV_V_TRIGGER_S, "scrambler cnt reaches 0x21 (f1 := 1)");
      cov_def(CV_V_E0_S, "scrambler bit step with e == 0");
      cov_def(CV_V_MATCH_S, "scrambler match true");
      cov_def(CV_V_NORMAL_D, "descrambler normal path");
      cov_def(CV_V_SPECIAL_CNT_D, "descrambler bit path because cnt >= 0x21 - n");
      cov_def(CV_V_SPECIAL_FLAG_D, "descrambler bit path because f1/f2 set");
      cov_def(CV_V_TRIGGER_D, "descrambler cnt reaches 0x21 (f1 := 1)");
      cov_def(CV_V_E0_D, "descrambler bit step with e == 0");
      cov_def(CV_V_MATCH_D, "descrambler match true");
      cov_def(CV_V_F1SET, "f1 stays set across symbols");
      cov_def(CV_V_F2SET, "f2 set at a symbol start");
      cov_def(CV_V_CNTPOS, "normal-path cnt > 0 after a symbol");
      cov_def(CV_V_LIMWRAP, "0x21 - n wraps (n > 0x21)");
      cov_def(CV_V_PROBE, "forked NULL-cfg probe");
      break;
    case 5:
      cov_def(CV_M_INIT, "SMC_init over garbage");
      cov_def(CV_M_W0, "SMC_encoder w0 != 0 (phase index)");
      cov_def(CV_M_W0Z, "SMC_encoder w0 == 0 (I / Q)");
      cov_def(CV_M_FLAG, "flag != 0");
      cov_def(CV_M_NOFLAG, "flag == 0 (pmap)");
      cov_def(CV_M_MODWRAP, "t >= mod (phase index wrap)");
      cov_def(CV_M_BWRAP, "b >= mod after the step");
      cov_def(CV_M_LIMWRAP, "wr wraps at lim");
      cov_def(CV_M_NEG_A, "a negative");
      cov_def(CV_M_NEG_B, "b negative (I / Q table index)");
      cov_def(CV_M_EMPTY, "n == 0");
      break;
    case 6:
      cov_def(CV_T_INITNULL, "SMCv17_init NULL cfg");
      cov_def(CV_T_INITCFG, "SMCv17_init cfg");
      cov_def(CV_T_ABS, "encoder_abs");
      cov_def(CV_T_DIF, "encoder_dif");
      cov_def(CV_T_TCM, "encoder_tcm");
      cov_def(CV_T_ST_HI, "tcm st > 3");
      cov_def(CV_T_ST_LO, "tcm st <= 3");
      cov_def(CV_T_LIMWRAP, "wr wraps at lim");
      cov_def(CV_T_MASKED, "tcm input changed in place");
      cov_def(CV_T_NEGB0, "tcm w0 low byte negative");
      cov_def(CV_T_K, "tcm k sweep");
      break;
    case 7:
      cov_def(CV_W_SCR17, "ScrambleDataV17");
      cov_def(CV_W_SCR27, "ScrambleDataV27");
      cov_def(CV_W_SCR29, "ScrambleDataV29");
      cov_def(CV_W_DES17, "DescrambleDataV17");
      cov_def(CV_W_DES27, "DescrambleDataV27");
      cov_def(CV_W_DES29, "DescrambleDataV29");
      cov_def(CV_W_SEED17, "SeedScramblerV17");
      cov_def(CV_W_SEED29, "SeedScramblerV29");
      cov_def(CV_W_ENC17_0, "SetEncoderV17 0");
      cov_def(CV_W_ENC17_1, "SetEncoderV17 1");
      cov_def(CV_W_ENC17_2, "SetEncoderV17 2");
      cov_def(CV_W_ENC17_X, "SetEncoderV17 other (no-op)");
      cov_def(CV_W_ENC29_0, "SetEncoderV29 0");
      cov_def(CV_W_ENC29_1, "SetEncoderV29 1");
      cov_def(CV_W_ENC29_X, "SetEncoderV29 other (no-op)");
      cov_def(CV_W_SET27_0, "SetScramblerV27 rate 0");
      cov_def(CV_W_SET27_1, "SetScramblerV27 rate 1");
      break;
    default: break;
  }
}

static unsigned rnd16(void) { return rnd() & 0xffffu; }
static unsigned long long fnv_us(const unsigned short *p, unsigned n) { return fnv_shorts((const short *)p, n); }
static void fill_us(unsigned short *p, unsigned n, int kind)
{
  unsigned i;
  for (i = 0; i < n; i++) {
    switch (kind) {
      case 0: p[i] = (unsigned short)rnd16(); break;
      case 1: p[i] = 0; break;
      case 2: p[i] = 0xffff; break;
      case 3: p[i] = (unsigned short)(i & 1 ? 0xaaaa : 0x5555); break;
      case 4: p[i] = (unsigned short)(i * 7u); break;
      default: p[i] = (unsigned short)(rnd() & 7u); break;
    }
  }
}
static void scribble(void *p, size_t n)
{
  /* always draws max(n, 256) random numbers: the object sizes differ between i386 and x86-64, the random stream must not */
  unsigned char *b = (unsigned char *)p;
  size_t i, m = n < 256 ? 256 : n;
  for (i = 0; i < m; i++) {
    unsigned char v = (unsigned char)rnd();
    if (i < n) b[i] = v;
  }
}

/* ============================================================================================ scenario 0: tables */
static void dump_us(const char *name, const unsigned short *t, unsigned n)
{
  unsigned i;
  char line[4096];
  int o = snprintf(line, sizeof line, "TAB %s", name);
  for (i = 0; i < n && o < (int)sizeof line - 16; i++) o += snprintf(line + o, sizeof line - (size_t)o, " %u", t[i]);
  emit(1, "%s", line);
}
static void dump_s(const char *name, const short *t, unsigned n)
{
  unsigned i;
  char line[512];
  int o = snprintf(line, sizeof line, "TAB %s", name);
  for (i = 0; i < n && o < (int)sizeof line - 16; i++) o += snprintf(line + o, sizeof line - (size_t)o, " %d", t[i]);
  emit(1, "%s", line);
}
static void scen0(void)
{
  char b1[64], b2[64];
  const FaxSgdCfg *g = &SGD_CFG;
  const FaxSmcCfg *m = &SMC_CFG;
  htab_add("xor", FPM_xor_table, sizeof FPM_xor_table);
  emit(1, "FIFO_CFG %u %u %u", FIFO_CFG_SYM.w0, FIFO_CFG_SYM.size, FIFO_CFG_SYM.fill);
  emit(1, "SDM_CFG %d %d %d", SDM_CFG.s[0], SDM_CFG.s[1], SDM_CFG.s[2]);
  emit(1, "SDMv27_CFG %u", SDMv27_CFG.n);
  emit(1, "SMCv17_CFG %u %u", SMCv17_CFG.tcmMask, SMCv17_CFG.w2);
  emit(1, "SGD_CFG bits=%u nbuf=%d w4=%d w6=%u tab=%s len=%d mode=%d w14=%d w18=%d w1c=%d tab2=%s w24=%08x w28=%d w2c=%d w30=%d", g->bits, g->nbuf, g->w4, g->w6,
       dptr(g->tab, b1, sizeof b1), g->len, g->mode, g->tailValue, g->pattern, g->patSyms, dptr(g->tab2, b2, sizeof b2), (unsigned)g->w24, g->w28, g->w2c, g->w30);
  emit(1, "SMC_CFG w0=%d flag=%d step=%d mod=%d shift=%u mask=%u mask2=%u mask3=%u pmap=%s imap=%s qmap=%s cos=%s sin=%s w28=%d", m->phaseOut, m->flag, m->step, m->mod, m->shift, m->mask,
       m->mask2, m->mask3, m->pmap ? "T" : "NULL", m->imap ? "T" : "NULL", m->qmap ? "T" : "NULL", m->cosv ? "T" : "NULL", m->sinv ? "T" : "NULL", m->w28);
  dump_us("SMCv17_MOD", SMCv17_MOD, 8);
  dump_us("SMCv17_ABS4", SMCv17_ABS4, 4);
  dump_us("SMCv17_PMAP4", SMCv17_PMAP4, 4);
  dump_s("SMCv17_IMAP4", SMCv17_IMAP4, 5);
  dump_s("SMCv17_QMAP4", SMCv17_QMAP4, 5);
  dump_us("V27TX_SDM_NUM_BITS", V27TX_SDM_NUM_BITS, 2);
  emit(1, "FPM_xor_table hash %016llx", fnv_us(FPM_xor_table, 256));
  dump_us("FPM_xor_table[0..15]", FPM_xor_table, 16);
  emit(1, "SIZES fifo-cfg=%u sdm=%u sdmv27=%u smcv17=%u sgdcfg-lo=%u", (unsigned)sizeof(FaxFifoCfg), (unsigned)sizeof(FaxSdm), (unsigned)sizeof(FaxSdmV27), (unsigned)sizeof(FaxSmcV17),
       (unsigned)(sizeof(FaxSgd) >= 0x5c));
  cov_hit(CV_TAB);
}

/* ============================================================================================ scenario 1: FIFO */
static void dump_fifo(const char *tag, const FaxFifo *f)
{
  char b[64];
  emit(0, "%s FIFO w0=%u size=%u fill=%u p6=%u cnt=%u rd=%u wr=%u buf=%s bh=%016llx", tag, f->w0, f->size, f->fill, f->pad6, f->count, f->rd, f->wr, dptr(f->buf, b, sizeof b),
       f->buf ? fnv_us(f->buf, f->size) : 0ull);
}
#define BIGN 70000u
static unsigned short g_big[BIGN + 16], g_big2[BIGN + 16], g_orig[BIGN + 16];

static unsigned short g_mq[40000];
static void scen1(void)
{
  static const unsigned sizes[] = {1, 2, 3, 5, 7, 8, 16, 31, 64, 100, 255, 300, 1000, 4096, 20000, 32767};
  int trial;
  htab_add("big", g_big, sizeof g_big);
  for (trial = 0; trial < 500; trial++) {
    unsigned size = sizes[rmod(sizeof sizes / sizeof sizes[0])];
    int mode = (int)rmod(5); /* 0 new + cfg, 1 new + NULL cfg, 2 supplied + cfg, 3 supplied + NULL cfg, 4 size 0 */
    int pokes = trial >= 300;     /* the second part pokes count / rd / wr and checks no data model */
    FaxFifoCfg cfg;
    FaxFifo *f;
    unsigned csize, nops, i, mhead = 0, mcount = 0;
    int fail = 0;
    unsigned short fill;
    if (2 * size == 0x18 || 2 * size == 0x14) size = 13; /* keep the buffer byte size away from sizeof(FaxFifo) on x86-64 (the recorder maps by size) */
    cfg.w0 = (unsigned short)rnd16();
    cfg.size = (unsigned short)size;
    cfg.fill = (unsigned short)(rpct(30) ? 0 : rnd16());
    if (mode == 1 || mode == 3) {
      csize = FIFO_CFG_SYM.size;
      fill = FIFO_CFG_SYM.fill;
    } else if (mode == 4) {
      cfg.size = 0;
      csize = 0;
      fill = cfg.fill;
      fail = rpct(50);
    } else {
      csize = cfg.size;
      fill = cfg.fill;
    }
    emit(0, "TRIAL %d mode %d size %u fill %u fail %d pokes %d", trial, mode, csize, fill, fail, pokes);
    if (mode <= 1 || mode == 4) {
      if (fail) {
        g_fail_malloc = 1; /* the buffer malloc (size 0 -> no zero loop) */
        cov_hit(CV_F_FAIL0);
      }
      f = FIFO_create(NULL, mode == 1 ? NULL : &cfg);
      cov_hit(CV_F_NEWOBJ);
    } else {
      FaxFifo *sup = (FaxFifo *)sysdep_malloc((unsigned)sizeof(FaxFifo));
      scribble(sup, sizeof *sup);
      sup->buf = (unsigned short *)sysdep_malloc(2u * (csize ? csize : 1u));
      f = FIFO_create(sup, mode == 3 ? NULL : &cfg);
      if (f != sup) die3("FIFO_create did not return the supplied object");
      cov_hit(CV_F_SUPPLIED);
    }
    if (mode == 1 || mode == 3) cov_hit(CV_F_NULLCFG); else cov_hit(CV_F_CUSTCFG);
    if (csize == 0) cov_hit(CV_F_SIZE0);
    if (!f) die3("FIFO_create returned NULL");
    dump_fifo("created", f);
    if (f->count != 0 || f->rd != 0 || f->wr != 0 || f->size != csize || f->fill != fill) die3("FIFO_create state");
    if (csize == 0) {
      emit(0, "W0 %d R0 %d", FIFO_write(f, g_big, 5), FIFO_read(f, g_big2, 5));
      dump_fifo("size0", f);
    } else {
      nops = 30 + rmod(60);
      for (i = 0; i < nops; i++) {
        unsigned n, k, exp, j;
        int op = (int)rmod(10);
        n = (unsigned)rmod(csize * 2 + 4);
        if (rpct(8)) n = 0;
        if (rpct(3)) n = 65535u - rmod(3);
        if (rpct(3)) n = 32768u + rmod(4);
        if (rpct(5)) n = csize;
        if (rpct(5) && csize > 1) n = csize - 1;
        if (n >= 32768u) cov_hit(CV_F_BIGN);
        if (pokes && rpct(12)) { /* poke count (rd / wr stay inside the buffer) */
          f->count = (unsigned short)(rpct(50) ? csize + 1 + rmod(10) : rmod(csize + 1));
          if (f->count > csize) cov_hit(CV_F_POKE_CNT);
          emit(0, "POKE cnt %u", f->count);
        }
        if (pokes && rpct(8)) { /* rd / wr anywhere inside the buffer */
          f->rd = (unsigned short)rmod(csize);
          f->wr = (unsigned short)rmod(csize);
          emit(0, "POKE rd %u wr %u", f->rd, f->wr);
        }
        if (op < 5) { /* write */
          unsigned space = (unsigned short)(csize - f->count), wr0 = f->wr;
          fill_us(g_big, n, 0);
          k = (unsigned)FIFO_write(f, g_big, (unsigned short)n);
          exp = n > space ? space : n;
          if (k != exp) die3("write returned %u, expected %u", k, exp);
          if (n == 0) cov_hit(CV_F_W_ZERO);
          else if (space == 0) cov_hit(CV_F_W_FULL);
          else if (n > space) cov_hit(CV_F_W_PART);
          if (k && wr0 + k >= csize) cov_hit(CV_F_W_WRAP);
          if (!pokes)
            for (j = 0; j < k; j++) g_mq[(mhead + mcount + j) % csize] = g_big[j];
          mcount += k;
          emit(0, "WR n=%u k=%u", n, k);
        } else { /* read */
          unsigned avail = f->count, rd0 = f->rd;
          fill_us(g_big2, n < 100 ? n : 100, 3);
          k = (unsigned)FIFO_read(f, g_big2, (unsigned short)n);
          exp = n < avail ? n : avail;
          if (k != exp) die3("read returned %u, expected %u", k, exp);
          if (n == 0) cov_hit(CV_F_R_ZERO);
          else if (n > avail) cov_hit(CV_F_R_FILL);
          else if (n < avail) cov_hit(CV_F_R_PART);
          if (k && rd0 + k >= csize) cov_hit(CV_F_R_WRAP);
          for (j = k; j < n; j++)
            if (g_big2[j] != fill) die3("read fill differs at %u", j);
          if (!pokes) { /* clean sequences: the queue model */
            for (j = 0; j < k; j++)
              if (g_big2[j] != g_mq[(mhead + j) % csize]) die3("read data differs from the model at %u", j);
            mhead = (mhead + k) % csize;
            mcount -= k;
          }
          emit(0, "RD n=%u k=%u h=%016llx", n, k, fnv_us(g_big2, n));
        }
        dump_fifo("op", f);
      }
    }
    FIFO_delete(f);
    cov_hit(CV_F_DELETE);
    trial_end("fifo");
  }
}

/* ============================================================================================ scenario 2: SGD */
#define TABN 4096
static unsigned short g_tab1[TABN], g_tab2[TABN], g_tab3[TABN];
static void dump_sgd(const char *tag, const FaxSgd *o)
{
  char b1[64], b2[64], b3[64];
  const FaxSgdCfg *c = &o->cfg;
  emit(0, "%s SGD bits=%u nbuf=%d w4=%d w6=%u tab=%s len=%d mode=%d w14=%d w18=%d w1c=%d tab2=%s w24=%08x w28=%d w2c=%d w30=%d", tag, c->bits, c->nbuf, c->w4, c->w6,
       dptr(c->tab, b1, sizeof b1), c->len, c->mode, c->tailValue, c->pattern, c->patSyms, dptr(c->tab2, b2, sizeof b2), (unsigned)c->w24, c->w28, c->w2c, c->w30);
  emit(0, "%s SGD w34=%d w38=%d w3c=%u w40=%d w44=%d pass=%u pos=%u symbit=%u buf=%s bufn=%d scale=%u w58=%d bh=%016llx", tag, o->w34, o->w38, o->w3c,
       o->w40, o->w44, o->pass, o->pos, o->symbit, dptr(o->buf, b3, sizeof b3), o->bufn, o->scale, o->w58, o->buf ? fnv_us(o->buf, 64) : 0ull);
}
static void rand_sgd_cfg(FaxSgdCfg *c)
{
  int kind = (int)rmod(4);
  c->bits = (unsigned short)(rpct(70) ? 1 + rmod(16) : rmod(34));
  c->nbuf = (short)(rpct(70) ? 1 + (int)rmod(40) : (int)rmod(5) - 2);
  c->w4 = (short)(rpct(60) ? (int)rmod(4) : (int)rmod(40) - 5);
  c->w6 = (unsigned short)rnd16();
  c->tab = g_tab1 + rmod(64);
  c->len = rpct(60) ? (int)(1 + rmod(40)) : rpct(50) ? (int)(rnd() & 0xffff0000u) | (int)rmod(8) : (int)(rnd() & 0xffff0000u) | (int)(0xfff0u + rmod(16));
  c->mode = rpct(50) ? 0 : (int)(1 + rmod(3));
  c->tailValue = (int)rnd();
  c->pattern = (int)rnd();
  c->patSyms = rpct(80) ? (int)(1 + rmod(40)) : (int)rmod(3);
  c->tab2 = g_tab2 + rmod(64);
  {
    unsigned lo = rpct(60) ? rmod(40) : rmod(3);
    int hi = rpct(70) ? (int)rmod(0x4001) : (int)(0x4000 + rmod(0x2000));
    if (rpct(5)) hi = -(int)rmod(0x2000);
    c->w24 = (int)(lo | ((unsigned)hi << 16));
  }
  c->w28 = (int)rnd();
  c->w2c = (int)rnd();
  c->w30 = (int)rnd();
  (void)kind;
}
static int bytes_ok(unsigned bytes)
{
  /* the recorder maps a block of exactly sizeof(FaxFifo) / sizeof(FaxSgd) bytes to its i386 size: keep buffers away from those sizes */
  return bytes != 0x14 && bytes != 0x18 && bytes != 0x5c && bytes != 0x70; /* 0x14 / 0x5c: the i386 sizes, 0x18 / 0x70: sizeof(FaxFifo) / sizeof(FaxSgd) on x86-64 */
}
static void scen2(void)
{
  int trial;
  htab_add("tab1", g_tab1, sizeof g_tab1);
  htab_add("tab2", g_tab2, sizeof g_tab2);
  htab_add("tab3", g_tab3, sizeof g_tab3);
  htab_add("xor", FPM_xor_table, sizeof FPM_xor_table);
  htab_add("big", g_big, sizeof g_big);
  fill_us(g_tab1, TABN, 0);
  fill_us(g_tab2, TABN, 0);
  fill_us(g_tab3, TABN, 0);
  for (trial = 0; trial < 500; trial++) {
    int mode = (int)rmod(4); /* 0 new + cfg, 1 new + NULL, 2 supplied + cfg, 3 supplied + NULL */
    FaxSgdCfg cfg;
    FaxSgd *o;
    unsigned nops, i;
    int cap, bn; /* cap = shorts the object's buffer holds */
    rand_sgd_cfg(&cfg);
    if (mode == 1 || mode == 3) cfg = SGD_CFG;
    if (rpct(10) && mode != 3 && mode != 1) { /* bufn <= 0 */
      cfg.nbuf = (short)(rpct(50) ? 1 : -(int)rmod(5));
      cfg.w24 = (int)((unsigned)cfg.w24 & 0xffff0000u);
    }
    if (mode <= 1) {
      /* the blob mallocs (u16)(2 * (nbuf + w4) - 2) bytes and clears bufn = nbuf + lo16(w24) - 1 shorts: make the buffer large enough and its size unambiguous */
      if (mode == 0) {
        int lo = (unsigned short)cfg.w24;
        if ((int)cfg.nbuf + (int)cfg.w4 < (int)cfg.nbuf + lo) cfg.w4 = (short)lo;
        while (!bytes_ok((unsigned short)(2 * ((int)cfg.nbuf + (int)cfg.w4) - 2)) || 2 * ((int)cfg.nbuf + (int)cfg.w4) - 2 < 0) cfg.w4 = (short)(cfg.w4 + 1);
      }
      cap = ((int)cfg.nbuf + (int)cfg.w4) - 1;
      o = SGD_create(NULL, mode == 1 ? NULL : &cfg);
      cov_hit(CV_S_NEWOBJ);
    } else {
      FaxSgd *sup = (FaxSgd *)sysdep_malloc((unsigned)sizeof(FaxSgd));
      unsigned capb = 2u * 120u + 2u * (unsigned)(rnd() & 7);
      if (sizeof(FaxSgd) != 0x5c && sizeof(FaxSgd) != 0x70) die3("sizeof(FaxSgd) %u: update bytes_ok", (unsigned)sizeof(FaxSgd));
      while (!bytes_ok(capb)) capb += 2;
      cap = (int)(capb / 2);
      scribble(sup, sizeof *sup);
      sup->buf = (unsigned short *)sysdep_malloc(capb);
      {
        int lo = (unsigned short)cfg.w24;
        if ((int)cfg.nbuf + lo - 1 > cap) cfg.w24 = (int)((unsigned)cfg.w24 & 0xffff0000u);
        if ((int)cfg.nbuf + (int)(unsigned short)cfg.w24 - 1 > cap) cfg.nbuf = 1;
      }
      o = SGD_create(sup, mode == 3 ? NULL : &cfg);
      if (o != sup) die3("SGD_create did not return the supplied object");
      cov_hit(CV_S_SUPPLIED);
    }
    if (mode == 1 || mode == 3) cov_hit(CV_S_NULLCFG);
    if (!o) die3("SGD_create NULL");
    bn = (short)(unsigned short)((unsigned)(unsigned short)o->cfg.nbuf + (unsigned short)o->cfg.w24 - 1u);
    if (bn > cap) die3("harness: bufn %d beyond the buffer %d", bn, cap);
    if (bn > 0) cov_hit(CV_S_BUFN_POS); else if (bn == 0) cov_hit(CV_S_BUFN_ZERO); else cov_hit(CV_S_BUFN_NEG);
    if ((short)((unsigned)o->cfg.w24 >> 16) > 0x4000) cov_hit(CV_S_HI_NEG);
    if (o->buf && bn > 0) {
      int j; /* fresh blocks are patterned: after create the first `bufn` shorts are zero */
      for (j = 0; j < bn; j++)
        if (o->buf[j] != 0) die3("buffer not cleared at %d", j);
    }
    dump_sgd("created", o);
    nops = 12 + rmod(14);
    for (i = 0; i < nops; i++) {
      int op = (int)rmod(8);
      if (op == 0) { /* control */
        FaxSgdCtlSeq sq;
        FaxSgdCtlPat pt;
        FaxSgdCtl ctl;
        int kind = (int)rmod(4), maxlo = cap - (int)(unsigned short)o->cfg.nbuf + 1;
        sq.tab = g_tab3 + rmod(100);
        sq.len = rpct(70) ? (int)(1 + rmod(40)) : (int)(rnd() & 0xffff0000u) | (int)rmod(8);
        sq.mode = rpct(50) ? 0 : (int)(1 + rmod(2));
        sq.tailValue = (int)rnd();
        sq.pattern = (int)rnd();
        sq.patSyms = (int)(rmod(41));
        pt.tab2 = g_tab2 + rmod(100);
        {
          unsigned lo = maxlo > 0 ? rmod((unsigned)(maxlo < 3 ? maxlo : 3)) : 0; /* bufn = nbuf + lo - 1 stays inside the buffer */
          int hi = rpct(70) ? (int)rmod(0x4001) : (int)(0x4000 + rmod(0x2000));
          pt.w24 = (int)(lo | ((unsigned)hi << 16));
          if (maxlo <= 0) pt.w24 = (int)((unsigned)hi << 16);
        }
        pt.w28 = (int)rnd();
        pt.w2c = (int)rnd();
        pt.w30 = (int)rnd();
        ctl.seq = (kind == 0 || kind == 2) ? &sq : NULL;
        ctl.pat = (kind == 1 || kind == 2) ? &pt : NULL;
        if (kind == 0) cov_hit(CV_S_CTL_SEQ);
        if (kind == 1) cov_hit(CV_S_CTL_PAT);
        if (kind == 3) cov_hit(CV_S_CTL_NONE);
        if (kind == 2) { cov_hit(CV_S_CTL_SEQ); cov_hit(CV_S_CTL_PAT); }
        if (ctl.pat) {
          int nb = (short)(unsigned short)((unsigned)(unsigned short)o->cfg.nbuf + (unsigned short)pt.w24 - 1u);
          if (nb > cap) die3("harness: control bufn %d beyond the buffer %d", nb, cap);
          bn = nb;
          if (bn <= 0) cov_hit(CV_S_BUFN_NEG);
        }
        SGD_control(o, &ctl);
        dump_sgd("control", o);
      } else if (op < 4) { /* sequence_gen */
        short n = (short)(rpct(10) ? 0 : rpct(5) ? -(int)rmod(3) - 1 : (int)rmod(90));
        short r;
        unsigned short p0;
        if ((unsigned short)o->cfg.len > 3000 && (unsigned short)o->cfg.len < 0xff00) o->cfg.len = (int)rmod(40); /* lo16 >= 0xff00 = a small NEGATIVE short is allowed */
        if (rpct(25)) o->pos = (unsigned short)rmod((unsigned)(unsigned short)o->cfg.len + 1u);
        if (rpct(10)) o->pass = (unsigned short)rmod(4);
        if (o->pos > 3000) o->pos = 0;
        if (n < 0) cov_hit(CV_S_SEQ_NEGN);
        fill_us(g_big, 200, 3);
        if (o->cfg.mode != 0) cov_hit(CV_S_SEQ_MODE1); else cov_hit(CV_S_SEQ_MODE0);
        if (o->cfg.mode == 0 && o->pass != 0) cov_hit(CV_S_SEQ_CONST);
        p0 = o->pos;
        r = SGD_sequence_gen(o, g_big, n);
        if (o->cfg.mode != 0 && o->pos < p0 + (unsigned)(n > 0 ? n : 0) && n != 0 && (short)o->cfg.len <= n) cov_hit(CV_S_SEQ_WRAP);
        if (o->cfg.mode == 0 && o->pass != 0) cov_hit(CV_S_SEQ_PASS);
        emit(0, "SEQ n=%d r=%d h=%016llx", n, r, fnv_us(g_big, (unsigned short)n));
        dump_sgd("seq", o);
      } else { /* symbol_gen */
        short n = (short)(rpct(10) ? 0 : rpct(5) ? -(int)rmod(3) - 1 : (int)rmod(90));
        short r;
        unsigned short s0;
        if (rpct(30)) o->symbit = (unsigned short)(rpct(20) ? rnd16() : rmod(45));
        if (rpct(10)) o->cfg.bits = (unsigned short)(rpct(40) ? 0 : 1 + rmod(33));
        if (o->cfg.bits == 0) cov_hit(CV_S_SYM_BITS0);
        if (o->cfg.bits >= 17) cov_hit(CV_S_SYM_BIG);
        if ((unsigned short)o->cfg.patSyms == 0) cov_hit(CV_S_SYM_W1C0);
        s0 = o->symbit;
        r = SGD_symbol_gen(o, g_big, n);
        if (n != 0 && (short)o->symbit > (short)s0) cov_hit(CV_S_SYM_WRAP);
        emit(0, "SYM n=%d r=%d h=%016llx", n, r, fnv_us(g_big, (unsigned short)n));
        dump_sgd("sym", o);
      }
    }
    SGD_delete(o);
    cov_hit(CV_S_DELETE);
    trial_end("sgd");
  }
  {
    /* SGD_create(NULL, NULL) with the object malloc failing: the blob stores through the NULL pointer (SIGSEGV), the port returns NULL.  Both are reported as the same line. */
    int st = 0;
    pid_t pid;
    fflush(stdout);
    pid = fork();
    if (pid == 0) {
      FaxSgd *o;
      g_fail_malloc = 0;
      o = SGD_create(NULL, NULL);
      _exit(o == NULL ? 7 : 8);
    }
    if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
    if (SIDE_BLOB) {
      if (!(WIFSIGNALED(st) && WTERMSIG(st) == SIGSEGV)) die3("blob SGD_create with a failing object malloc did not SIGSEGV (status %x)", st);
    } else {
      if (!(WIFEXITED(st) && WEXITSTATUS(st) == 7)) die3("port SGD_create with a failing object malloc did not return NULL (status %x)", st);
    }
    emit(1, "PROBE SGD_create object-malloc failure: NULL-or-crash (blob crashes, port returns NULL)");
    cov_hit(CV_S_PROBE);
  }
}

/* ============================================================================================ scenario 3: SDM */
static void dump_sdm(const char *tag, const FaxSdm *s)
{
  emit(0, "%s SDM n=%d t1=%d t2=%d p6=%d mask=%08x nmask=%08x st=%08x sh1=%d sh2=%d", tag, s->n, s->tap1, s->tap2, s->pad6, s->mask, s->nmask, s->state, s->sh1, s->sh2);
}
static void model_sdm_scr(unsigned *st, int n, int t1, int t2, unsigned short *buf, unsigned cnt)
{
  unsigned mask = (1u << (n & 31)) - 1u, nmask = ~mask, i;
  for (i = 0; i < cnt; i++) {
    unsigned o = ((*st >> ((t1 - n) & 31)) ^ (*st >> ((t2 - n) & 31)) ^ buf[i]) & mask;
    buf[i] = (unsigned short)o;
    *st = ((*st << (n & 31)) & nmask) | (o & 0xffffu);
  }
}
static void scen3(void)
{
  int trial;
  unsigned sw;
  for (trial = 0; trial < 700; trial++) {
    FaxSdm s, t;
    FaxSdmCfg cfg;
    unsigned cnt, kind, st0;
    unsigned short *a = g_big, *b = g_big2, *c;
    int real = rpct(15), masked;
    scribble(&s, sizeof s);
    scribble(&t, sizeof t);
    if (real) {
      static const FaxSdmCfg reals[] = {{{4, 5, 23}}, {{2, 18, 23}}, {{1, 18, 23}}, {{2, 6, 7}}, {{3, 6, 7}}, {{4, 1, 3}}};
      cfg = reals[rmod(sizeof reals / sizeof reals[0])];
      cov_hit(CV_D_REALCFG);
    } else {
      cfg.s[0] = (short)(rpct(85) ? 1 + (int)rmod(8) : (int)rmod(40) - 4);
      cfg.s[1] = (short)(rpct(85) ? 1 + (int)rmod(31) : (int)rmod(70) - 20);
      cfg.s[2] = (short)(rpct(85) ? 1 + (int)rmod(31) : (int)rmod(70) - 20);
    }
    if (cfg.s[1] < cfg.s[0] || cfg.s[2] < cfg.s[0]) cov_hit(CV_D_NNEG_SHIFT);
    SDM_init(&s, &cfg);
    SDM_init(&t, &cfg);
    cov_hit(CV_D_INIT);
    dump_sdm("init", &s);
    st0 = rpct(20) ? 0 : rpct(20) ? 1 : rpct(20) ? 0xffffffffu : rnd();
    s.state = st0;
    t.state = st0;
    kind = rmod(6);
    cnt = rpct(5) ? 0 : rpct(10) ? 3000 + rmod(5000) : 1 + rmod(120);
    if (cnt == 0) cov_hit(CV_D_N0);
    if (cnt > 2000) cov_hit(CV_D_LONG);
    masked = rpct(60);
    fill_us(a, cnt, (int)kind);
    if (masked) {
      unsigned i;
      for (i = 0; i < cnt; i++) a[i] = (unsigned short)(a[i] & ((1u << (cfg.s[0] & 31)) - 1u));
    }
    memcpy(b, a, cnt * 2);
    emit(0, "TRIAL %d cfg %d %d %d st0 %08x cnt %u kind %u masked %d", trial, cfg.s[0], cfg.s[1], cfg.s[2], st0, cnt, kind, masked);
    SDM_scrambler(&s, a, (unsigned short)cnt);
    cov_hit(CV_D_SCR);
    dump_sdm("scr", &s);
    emit(0, "scrambled h=%016llx", fnv_us(a, cnt));
    {
      /* model */
      unsigned ms = st0;
      static unsigned short mb[BIGN];
      memcpy(mb, b, cnt * 2);
      model_sdm_scr(&ms, cfg.s[0], cfg.s[1], cfg.s[2], mb, cnt);
      if (ms != s.state || memcmp(mb, a, cnt * 2) != 0) die3("scrambler differs from the model (trial %d)", trial);
    }
    c = a;
    SDM_descrambler(&t, c, (unsigned short)cnt);
    cov_hit(CV_D_DESCR);
    dump_sdm("des", &t);
    emit(0, "descrambled-of-scrambled h=%016llx", fnv_us(c, cnt));
    if (masked && cfg.s[0] >= 1 && cfg.s[0] <= 8 && memcmp(c, b, cnt * 2) == 0 && cnt) cov_hit(CV_D_ROUNDTRIP);
    /* plain descramble of the original data from another seed */
    t.state = rnd();
    SDM_descrambler(&t, b, (unsigned short)cnt);
    dump_sdm("des2", &t);
    emit(0, "descrambled h=%016llx", fnv_us(b, cnt));
  }
  /* exhaustive 16-bit seed sweep: V.17 style config, 6 words */
  {
    FaxSdm s;
    FaxSdmCfg cfg = {{2, 18, 23}};
    unsigned long long acc = 1469598103934665603ULL;
    unsigned short w[6];
    SDM_init(&s, &cfg);
    for (sw = 0; sw < 65536u; sw++) {
      unsigned k;
      s.state = sw | (sw << 16 & 0xff800000u);
      for (k = 0; k < 6; k++) w[k] = (unsigned short)((sw >> k) & 3u);
      SDM_scrambler(&s, w, 6);
      acc ^= fnv_us(w, 6) ^ s.state;
      acc *= 1099511628211ULL;
      s.state = sw;
      SDM_descrambler(&s, w, 6);
      acc ^= fnv_us(w, 6) ^ s.state;
      acc *= 1099511628211ULL;
    }
    emit(1, "SEEDSWEEP %016llx", acc);
    cov_hit(CV_D_SEEDSWEEP);
  }
}

/* ============================================================================================ scenario 4: SDMv27 */
static void dump_v27(const char *tag, const FaxSdmV27 *s)
{
  emit(0, "%s V27 n=%u mask=%u nmask=%u reg=%u cnt=%u f1=%u f2=%u", tag, s->n, s->mask, s->nmask, s->reg, s->cnt, s->f1, s->f2);
}
/* A second, independently written model of the V.27ter machine (the rules as read from the machine code, bit-serial in both paths); it counts the events the coverage ids need and
 * its result is compared with the function under test (a blob/port output difference is the real test; this checks the harness' own understanding and makes the counters honest). */
typedef struct {
  unsigned trig, e0, match, special_cnt, special_flag, normal;
} V27Ev;
static void v27_model(int scr, FaxSdmV27 *s, unsigned short *buf, unsigned short count, V27Ev *ev)
{
  unsigned n = s->n;
  while (count--) {
    unsigned in = *buf, out = 0;
    int bit;
    if ((unsigned short)s->cnt >= (unsigned short)(0x21u - n)) ev->special_cnt++;
    else if ((s->f1 + s->f2) != 0) ev->special_flag++;
    else ev->normal++;
    if ((unsigned short)s->cnt < (unsigned short)(0x21u - n) && (s->f1 + s->f2) == 0) {
      /* normal: the whole symbol at once */
      unsigned reg = s->reg, d, t, x;
      int j;
      if (scr) {
        t = (reg << (n & 31)) & s->nmask;
        d = (in ^ ((t >> 6) ^ (t >> 7))) & s->mask;
        reg = t | d;
        out = d;
        x = ~(((reg >> 8) ^ d) & ((reg >> 9) ^ d) & ((reg >> 12) ^ d)) & 0xffffu;
      } else {
        unsigned r16 = (reg << (n & 31)) & 0xffffu;
        d = in & s->mask;
        t = r16 & s->nmask;
        reg = t | d;
        out = (((r16 >> 6) ^ (r16 >> 7)) ^ d) & s->mask;
        x = ~(((reg >> 8) ^ d) & ((reg >> 9) ^ d) & ((reg >> 12) ^ d)) & 0xffffu;
      }
      s->reg = (unsigned short)reg;
      for (j = (short)(n - 1u); j >= 0; j--) {
        unsigned b = (x >> (j & 31)) & 1u;
        s->cnt = (unsigned short)((unsigned short)(s->cnt + b) * b);
      }
      *buf++ = (unsigned short)out;
      continue;
    }
    /* bit-serial path, MSB of the symbol first */
    {
      unsigned work = in;
      unsigned reg = s->reg, f1 = s->f1, f2 = s->f2, cnt = s->cnt;
      for (bit = (short)n; bit > 0; bit--) {
        unsigned sreg, fb, d, m, mt, e, inb;
        if (scr) {
          f2 = f1;
          sreg = (reg * 2u) & 0xffffu;
          fb = (sreg >> 6) ^ (sreg >> 7) ^ f2;
          d = ((work >> ((n - 1u) & 31u)) ^ fb) & 1u;
          work = (unsigned short)((work << 1) | d);
        } else {
          unsigned in2 = work * 2u;
          sreg = (reg * 2u) & 0xffffu;
          inb = ((in2 & 0xfffeu) >> (n & 31u)) & 1u;
          d = inb;
          fb = ((((sreg >> 6) ^ (sreg >> 7)) ^ f2) & 1u) ^ inb;
          work = (unsigned short)(in2 | fb);
        }
        reg = sreg | d;
        m = ((d << 16) - d) ^ reg;
        mt = (m & 0x1300u) == 0x1300u;
        if (mt) ev->match++;
        e = ~(mt | f2) & 1u;
        if (!e) ev->e0++;
        cnt = (unsigned short)((unsigned short)(cnt + e) * e);
        f1 = (unsigned short)(e * (scr ? f2 : f1));
        if ((unsigned short)cnt == 0x21) {
          cnt = 0;
          f1 = 1;
          ev->trig++;
        }
        if (!scr) f2 = f1;
      }
      s->reg = (unsigned short)reg;
      s->f1 = (unsigned short)f1;
      s->f2 = (unsigned short)f2;
      s->cnt = (unsigned short)cnt;
      *buf++ = (unsigned short)(work & s->mask);
    }
  }
}
static void v27_check(int scr, const FaxSdmV27 *before, const unsigned short *in, const FaxSdmV27 *after, const unsigned short *out, unsigned count)
{
  static unsigned short tmp[BIGN];
  FaxSdmV27 m = *before;
  V27Ev ev;
  memset(&ev, 0, sizeof ev);
  memcpy(tmp, in, count * 2);
  v27_model(scr, &m, tmp, (unsigned short)count, &ev);
  if (memcmp(&m, after, sizeof m) != 0 || memcmp(tmp, out, count * 2) != 0) die3("SDMv27 %s differs from the harness model", scr ? "scrambler" : "descrambler");
  if (ev.special_cnt) cov_hit(scr ? CV_V_SPECIAL_CNT_S : CV_V_SPECIAL_CNT_D);
  if (ev.special_flag) cov_hit(scr ? CV_V_SPECIAL_FLAG_S : CV_V_SPECIAL_FLAG_D);
  if (ev.normal) cov_hit(scr ? CV_V_NORMAL_S : CV_V_NORMAL_D);
  if (ev.trig) cov_hit(scr ? CV_V_TRIGGER_S : CV_V_TRIGGER_D);
  if (ev.e0) cov_hit(scr ? CV_V_E0_S : CV_V_E0_D);
  if (ev.match) cov_hit(scr ? CV_V_MATCH_S : CV_V_MATCH_D);
  if (before->f2 != 0) cov_hit(CV_V_F2SET);
  if ((unsigned short)(0x21u - before->n) != 0x21u - before->n) cov_hit(CV_V_LIMWRAP);
}
static void scen4(void)
{
  int trial;
  for (trial = 0; trial < 900; trial++) {
    FaxSdmV27 s, t, o1;
    FaxSdmCfg27 cfg;
    unsigned cnt, kind, i;
    unsigned short *a = g_big, *b = g_big2;
    int nsel = (int)rmod(10);
    scribble(&s, sizeof s);
    scribble(&t, sizeof t);
    cfg.n = (unsigned short)(nsel < 4 ? 2 : nsel < 8 ? 3 : (int)(1 + rmod(40)));
    if (rpct(2)) cfg.n = (unsigned short)(0x21 + rmod(4));
    if (rpct(2)) cfg.n = (unsigned short)(0x8000u + rmod(3) + (rpct(50) ? 0x7ff0u : 0u)); /* n as a negative short: no bit-serial iterations, huge shift counts */
    SDMv27_init(&s, &cfg);
    SDMv27_init(&t, &cfg);
    cov_hit(cfg.n == 2 ? CV_V_INIT2 : CV_V_INIT_OTHER);
    dump_v27("init", &s);
    /* poke the state */
    if (rpct(70)) {
      s.reg = (unsigned short)rnd16();
      t.reg = s.reg;
    }
    if (rpct(50)) {
      unsigned c0 = rpct(60) ? 0x1c + rmod(8) : rpct(85) ? rmod(0x30) : 0x8000u + rmod(0x8000u); /* cnt >= 0x8000: negative as a short */
      s.cnt = (unsigned short)c0;
      t.cnt = (unsigned short)c0;
    }
    if (rpct(25)) {
      static const unsigned short fv[] = {0, 1, 2, 0xffff};
      s.f1 = fv[rmod(4)];
      s.f2 = fv[rmod(4)];
      t.f1 = s.f1;
      t.f2 = s.f2;
    }
    kind = rmod(7);
    cnt = rpct(5) ? 0 : rpct(6) ? 2000 + rmod(3000) : 1 + rmod(60);
    if (kind == 6) kind = 0;
    if (kind < 6) {
      fill_us(a, cnt, (int)kind);
      if (kind == 0 && rpct(60))
        for (i = 0; i < cnt; i++) a[i] = (unsigned short)(a[i] & s.mask);
    }
    if (rpct(40)) /* periodic data pulling the guard: constant, or a 3-bit all-ones pattern */
      for (i = 0; i < cnt; i++) a[i] = (unsigned short)(cfg.n == 2 ? 3u : 7u);
    memcpy(b, a, cnt * 2);
    memcpy(g_orig, a, cnt * 2);
    emit(0, "TRIAL %d n=%u kind %u cnt %u state %u %u %u %u", trial, cfg.n, kind, cnt, s.reg, s.cnt, s.f1, s.f2);
    o1 = s;
    {
      FaxSdmV27 before_s = s, before_t = t;
      SDMv27_scrambler(&s, a, (short)cnt);
      v27_check(1, &before_s, g_orig, &s, a, cnt);
      dump_v27("scr", &s);
      emit(0, "scrambled h=%016llx", fnv_us(a, cnt));
      if (s.f1 != 0) cov_hit(CV_V_F1SET);
      if (s.cnt > 0) cov_hit(CV_V_CNTPOS);
      SDMv27_descrambler(&t, b, (short)cnt);
      v27_check(0, &before_t, g_orig, &t, b, cnt);
      dump_v27("des", &t);
      emit(0, "descrambled h=%016llx", fnv_us(b, cnt));
    }
    /* second pass: scrambled data back (same start state) */
    {
      FaxSdmV27 u = o1, before_u = o1;
      unsigned short *c = a;
      static unsigned short in2[BIGN];
      memcpy(in2, c, cnt * 2);
      SDMv27_descrambler(&u, c, (short)cnt);
      v27_check(0, &before_u, in2, &u, c, cnt);
      dump_v27("rt", &u);
      emit(0, "roundtrip h=%016llx eq=%d", fnv_us(c, cnt), memcmp(c, g_orig, cnt * 2) == 0);
    }
  }
  /* long streams from the initial state, constant ones / zeros (reaching the 0x21 trigger by the stream itself) */
  {
    int k;
    for (k = 0; k < 40; k++) {
      FaxSdmV27 s, t;
      FaxSdmCfg27 cfg;
      unsigned i, cnt = 4000;
      unsigned short *a = g_big, *b = g_big2;
      cfg.n = (unsigned short)(k & 1 ? 3 : 2);
      SDMv27_init(&s, &cfg);
      SDMv27_init(&t, &cfg);
      for (i = 0; i < cnt; i++) a[i] = (unsigned short)(k < 10 ? 0 : k < 20 ? s.mask : k < 30 ? (i & s.mask) : (rnd() & s.mask));
      memcpy(b, a, cnt * 2);
      SDMv27_scrambler(&s, a, (short)cnt);
      SDMv27_descrambler(&t, b, (short)cnt);
      dump_v27("L-scr", &s);
      dump_v27("L-des", &t);
      emit(0, "L scr h=%016llx des h=%016llx", fnv_us(a, cnt), fnv_us(b, cnt));
    }
  }
  /* NULL cfg: the blob dereferences NULL (SIGSEGV), the port uses SDMv27_CFG */
  {
    int st = 0;
    pid_t pid;
    fflush(stdout);
    pid = fork();
    if (pid == 0) {
      FaxSdmV27 s;
      SDMv27_init(&s, NULL);
      _exit(s.n == 3 ? 7 : 8);
    }
    if (pid < 0 || waitpid(pid, &st, 0) != pid) die3("fork");
    if (SIDE_BLOB) {
      if (!(WIFSIGNALED(st) && WTERMSIG(st) == SIGSEGV)) die3("blob SDMv27_init(NULL) did not SIGSEGV (status %x)", st);
    } else {
      if (!(WIFEXITED(st) && WEXITSTATUS(st) == 7)) die3("port SDMv27_init(NULL) did not use SDMv27_CFG (status %x)", st);
    }
    emit(1, "PROBE SDMv27_init(NULL cfg): crash-or-default (blob crashes, port uses SDMv27_CFG)");
    cov_hit(CV_V_PROBE);
  }
}

/* ============================================================================================ scenario 5: SMC */
#define TBL 65536
static short g_imap[TBL], g_qmap[TBL], g_cos[TBL], g_sin[TBL];
static unsigned short g_pmap[TBL];
static void dump_smc(const char *tag, const FaxSmc *s, const FaxCirc *c)
{
  const FaxSmcCfg *g = &s->cfg;
  emit(0, "%s SMC w0=%d flag=%d step=%d mod=%d sh=%u mask=%u m2=%u m3=%u w28=%d a=%d b=%d wr=%d", tag, g->phaseOut, g->flag, g->step, g->mod, g->shift, g->mask, g->mask2, g->mask3, g->w28,
       s->a, s->b, c->wr);
}
static void scen5(void)
{
  int trial;
  static short oi[256], oq[256];
  static unsigned short ob[256];
  unsigned i;
  for (i = 0; i < TBL; i++) {
    g_imap[i] = (short)rnd16();
    g_qmap[i] = (short)rnd16();
    g_cos[i] = (short)rnd16();
    g_sin[i] = (short)rnd16();
    g_pmap[i] = (unsigned short)rnd16();
  }
  for (trial = 0; trial < 1200; trial++) {
    FaxSmc s;
    FaxSmcCfg cfg;
    FaxCirc c;
    unsigned n, calls, k;
    cfg.phaseOut = rpct(50) ? 0 : (rpct(80) ? 1 : (int)rnd());
    cfg.flag = rpct(45) ? 0 : (rpct(80) ? 1 : (int)rnd());
    cfg.mod = (short)(rpct(85) ? 1 + (int)rmod(64) : (int)rmod(300) - 20);
    cfg.step = (short)(rpct(80) ? (int)rmod(8) : (int)rmod(200) - 60);
    cfg.shift = (unsigned short)(rpct(80) ? rmod(8) : rmod(40));
    cfg.mask = (unsigned short)(rpct(80) ? rmod(16) : rnd16());
    cfg.mask2 = (unsigned short)(rpct(80) ? rmod(64) : rnd16());
    cfg.mask3 = (unsigned short)(rpct(80) ? rmod(64) : rnd16());
    cfg.cosv = g_cos + 32768;
    cfg.sinv = g_sin + 32768;
    cfg.w28 = (int)rnd();
    /* the 64K tables cover every index (mask, mask2 | mask3 <= 0xffff; the carrier tables are indexed by a short) */
    cfg.pmap = g_pmap;
    cfg.imap = g_imap;
    cfg.qmap = g_qmap;
    scribble(&s, sizeof s);
    SMC_init(&s, &cfg);
    cov_hit(CV_M_INIT);
    if (s.a != 0 || s.b != 0) die3("SMC_init did not clear a / b");
    if (rpct(50)) s.a = (short)rnd16();
    if (rpct(50)) s.b = (short)(rpct(60) ? (int)rmod((unsigned)(cfg.mod > 0 ? cfg.mod : 1)) : (int)(short)rnd16());
    if (s.a < 0) cov_hit(CV_M_NEG_A);
    if (s.b < 0) cov_hit(CV_M_NEG_B);
    c.i = oi;
    c.q = oq;
    c.buf = ob;
    c.lim = (short)(rpct(80) ? 1 + (int)rmod(100) : (int)rmod(130) - 5);
    c.wr = (short)(c.lim > 0 && rpct(80) ? (int)rmod((unsigned)c.lim) : (int)rmod(100));
    c.rd = (short)rnd16();
    dump_smc("init", &s, &c);
    calls = 1 + rmod(3);
    for (k = 0; k < calls; k++) {
      static unsigned short in[256];
      short wr0 = c.wr;
      n = rpct(8) ? 0 : 1 + rmod(150);
      if (n == 0) cov_hit(CV_M_EMPTY);
      fill_us(in, 256, 0);
      scribble(oi, sizeof oi);
      scribble(oq, sizeof oq);
      scribble(ob, sizeof ob);
      SMC_encoder(&s, &c, in, (unsigned short)n);
      if (cfg.phaseOut != 0) cov_hit(CV_M_W0); else cov_hit(CV_M_W0Z);
      if (cfg.flag != 0) cov_hit(CV_M_FLAG); else cov_hit(CV_M_NOFLAG);
      if (n > 1 && c.wr < wr0 + (int)n && c.lim > 0 && c.lim < (int)n) cov_hit(CV_M_LIMWRAP);
      if (cfg.phaseOut != 0 && cfg.mod > 0 && cfg.mod < 20) cov_hit(CV_M_MODWRAP);
      if (cfg.step >= cfg.mod && n > 1) cov_hit(CV_M_BWRAP);
      dump_smc("enc", &s, &c);
      emit(0, "out n=%u wr0=%d i=%016llx q=%016llx b=%016llx", n, wr0, fnv_shorts(oi, 256), fnv_shorts(oq, 256), fnv_us(ob, 256));
    }
  }
}

/* ============================================================================================ scenario 6: SMCv17 */
static void dump_v17(const char *tag, const FaxSmcV17 *s, const FaxCirc *c)
{
  emit(0, "%s V17 w0=%u w2=%u w4=%u x=%d y=%d w0a=%u y2=%d st=%d w10=%u k=%u wr=%d", tag, s->tcmMask, s->w2, s->w4, s->x, s->y, s->w0a, s->y2, s->st, s->w10, s->k, c->wr);
}
static void scen6(void)
{
  int trial;
  static unsigned short ob[256], in[256], in0[256];
  htab_add("big", g_big, sizeof g_big);
  for (trial = 0; trial < 1500; trial++) {
    FaxSmcV17 s;
    FaxSmcV17Cfg cfg;
    FaxCirc c;
    int kind = (int)rmod(3), calls, j;
    scribble(&s, sizeof s);
    if (rpct(20)) {
      SMCv17_init(&s, NULL);
      cov_hit(CV_T_INITNULL);
    } else {
      cfg.tcmMask = (unsigned short)rnd16();
      cfg.w2 = (unsigned short)rnd16();
      SMCv17_init(&s, &cfg);
      cov_hit(CV_T_INITCFG);
    }
    {
      FaxCirc c0;
      memset(&c0, 0, sizeof c0);
      dump_v17("fresh", &s, &c0); /* straight after the init: every field the init writes (and the garbage it leaves) */
    }
    s.k = (unsigned short)(1 + rmod(12));
    cov_hit(CV_T_K);
    if (rpct(60)) s.x = (short)rnd16();
    if (rpct(50)) s.y = (short)rnd16();
    s.y2 = (short)rmod(4);
    s.st = (short)rmod(8);
    if (rpct(15)) s.st = 0;
    if ((signed char)(s.tcmMask & 0xff) < 0) cov_hit(CV_T_NEGB0);
    c.i = NULL;
    c.q = NULL;
    c.buf = ob;
    c.lim = (short)(rpct(80) ? 1 + (int)rmod(100) : (int)rmod(130) - 5);
    c.wr = (short)(c.lim > 0 ? (int)rmod((unsigned)c.lim) : (int)rmod(100));
    c.rd = (short)rnd16();
    dump_v17("init", &s, &c);
    calls = 1 + (int)rmod(3);
    for (j = 0; j < calls; j++) {
      unsigned n = rpct(8) ? 0 : 1 + rmod(150), i;
      short wr0 = c.wr;
      fill_us(in, 256, 0);
      memcpy(in0, in, sizeof in);
      scribble(ob, sizeof ob);
      if (kind == 0) {
        SMCv17_encoder_abs(&s, &c, in, (unsigned short)n);
        cov_hit(CV_T_ABS);
      } else if (kind == 1) {
        SMCv17_encoder_dif(&s, &c, in, (unsigned short)n);
        cov_hit(CV_T_DIF);
      } else {
        short st0 = s.st;
        SMCv17_encoder_tcm(&s, &c, in, (unsigned short)n);
        cov_hit(CV_T_TCM);
        if (st0 > 3) cov_hit(CV_T_ST_HI); else cov_hit(CV_T_ST_LO);
        for (i = 0; i < n; i++)
          if (in[i] != in0[i]) cov_hit(CV_T_MASKED);
        if (s.st < 0 || s.st > 7 || s.y2 < 0 || s.y2 > 3) die3("tcm state left its range");
      }
      if (n > 1 && c.wr < wr0 && c.lim > 0) cov_hit(CV_T_LIMWRAP);
      dump_v17("enc", &s, &c);
      emit(0, "out n=%u wr0=%d b=%016llx in=%016llx", n, wr0, fnv_us(ob, 256), fnv_us(in, 256));
    }
  }
}

/* ============================================================================================ scenario 7: wrappers */
/* fake V.17 / V.27ter / V.29 modem objects.  -m32: the raw blob layout (offsets written down here independently of src/fax/fax_modem_views.h); x86-64: typed structs with unrelated members in
 * front of the sub-objects, whose accessors (faxw_*, declared by fax_modem_views.h) are defined below. */
#if UINTPTR_MAX == 0xffffffffu
#define PUT(base, off, v) (*(void **)((char *)(base) + (off)) = (v))
#define AT(base, off) ((char *)(*(void **)((char *)(base) + (off))))
typedef struct { void *obj; } Fake;
static Fake fake_new(void)
{
  Fake f;
  char *o = (char *)calloc(1, 0x100);
  PUT(o, 0x24, calloc(1, 0x200));  /* V.29 TX state / V.27 rate block (V.29: +0x1c sdm, +0x38 sel; V.27 rate: short at +0xc) */
  PUT(o, 0x28, calloc(1, 0x200));  /* V.17 / V.27 TX state */
  PUT(o, 0x50, calloc(1, 0x5000)); /* V.29 RX state */
  PUT(o, 0x54, calloc(1, 0x200));  /* V.27 RX state */
  PUT(o, 0x60, calloc(1, 0x5000)); /* V.17 RX state */
  f.obj = o;
  return f;
}
static FaxSdm *h_v17_tx(Fake f) { return (FaxSdm *)(AT(f.obj, 0x28) + 0x1c); }
static FaxSmcV17 *h_v17_smc(Fake f) { return (FaxSmcV17 *)(AT(f.obj, 0x28) + 0x34); }
static unsigned short *h_v17_mode(Fake f) { return (unsigned short *)(AT(f.obj, 0x28) + 0x8c); }
static FaxSdm *h_v17_rx(Fake f) { return (FaxSdm *)(AT(f.obj, 0x60) + 0x4f8c); }
static FaxSdm *h_v29_tx(Fake f) { return (FaxSdm *)(AT(f.obj, 0x24) + 0x1c); }
static int *h_v29_sel(Fake f) { return (int *)(AT(f.obj, 0x24) + 0x38); }
static FaxSdm *h_v29_rx(Fake f) { return (FaxSdm *)(AT(f.obj, 0x50) + 0x4f3c); }
static FaxSdmV27 *h_v27_tx(Fake f) { return (FaxSdmV27 *)(AT(f.obj, 0x28) + 0x1c); }
static short *h_v27_rate(Fake f) { return (short *)(AT(f.obj, 0x24) + 0xc); }
static FaxSdmV27 *h_v27_rx(Fake f) { return (FaxSdmV27 *)(AT(f.obj, 0x54) + 0x3c); }
static unsigned h_v17_tx_extra(Fake f) { return (unsigned)((unsigned char)AT(f.obj, 0x28)[0x3a] | ((unsigned char)AT(f.obj, 0x28)[0x3b] << 8)); } /* smc.x seen through the raw offset +0x3a */
#else
typedef struct { void *pad0; int pad1; struct HTx17 *tx17; void *padA[2]; struct HTx29 *tx29; struct HRx29 *rx29; struct HTx27 *tx27; struct HRate *rate; struct HRx27 *rx27; struct HRx17 *rx17; } HObj;
struct HTx17 { int a; void *p; long q[3]; FaxSdm sdm; char gap[5]; FaxSmcV17 smc; unsigned short mode; };
struct HRx17 { char gap[777]; void *p; FaxSdm sdm; };
struct HTx29 { void *p; FaxSdm sdm; int sel; };
struct HRx29 { char gap[123]; void *p; FaxSdm sdm; };
struct HTx27 { void *p; int a; FaxSdmV27 sdm; };
struct HRate { void *p; short rate; };
struct HRx27 { char gap[11]; void *p; FaxSdmV27 sdm; };
typedef struct { HObj *obj; } Fake;
static Fake fake_new(void)
{
  Fake f;
  HObj *o = (HObj *)calloc(1, sizeof *o);
  o->tx17 = (struct HTx17 *)calloc(1, sizeof *o->tx17);
  o->tx29 = (struct HTx29 *)calloc(1, sizeof *o->tx29);
  o->rx29 = (struct HRx29 *)calloc(1, sizeof *o->rx29);
  o->tx27 = (struct HTx27 *)calloc(1, sizeof *o->tx27);
  o->rate = (struct HRate *)calloc(1, sizeof *o->rate);
  o->rx27 = (struct HRx27 *)calloc(1, sizeof *o->rx27);
  o->rx17 = (struct HRx17 *)calloc(1, sizeof *o->rx17);
  f.obj = o;
  return f;
}
FaxSdm *faxw_v17_txsdm(void *o) { return &((HObj *)o)->tx17->sdm; }
FaxSmcV17 *faxw_v17_txsmc(void *o) { return &((HObj *)o)->tx17->smc; }
unsigned short *faxw_v17_encmode(void *o) { return &((HObj *)o)->tx17->mode; }
FaxSdm *faxw_v17_rxsdm(void *o) { return &((HObj *)o)->rx17->sdm; }
FaxSdm *faxw_v29_txsdm(void *o) { return &((HObj *)o)->tx29->sdm; }
int *faxw_v29_encsel(void *o) { return &((HObj *)o)->tx29->sel; }
FaxSdm *faxw_v29_rxsdm(void *o) { return &((HObj *)o)->rx29->sdm; }
FaxSdmV27 *faxw_v27_txsdm(void *o) { return &((HObj *)o)->tx27->sdm; }
short *faxw_v27_rate(void *o) { return &((HObj *)o)->rate->rate; }
FaxSdmV27 *faxw_v27_rxsdm(void *o) { return &((HObj *)o)->rx27->sdm; }
static FaxSdm *h_v17_tx(Fake f) { return faxw_v17_txsdm(f.obj); }
static FaxSmcV17 *h_v17_smc(Fake f) { return faxw_v17_txsmc(f.obj); }
static unsigned short *h_v17_mode(Fake f) { return faxw_v17_encmode(f.obj); }
static FaxSdm *h_v17_rx(Fake f) { return faxw_v17_rxsdm(f.obj); }
static FaxSdm *h_v29_tx(Fake f) { return faxw_v29_txsdm(f.obj); }
static int *h_v29_sel(Fake f) { return faxw_v29_encsel(f.obj); }
static FaxSdm *h_v29_rx(Fake f) { return faxw_v29_rxsdm(f.obj); }
static FaxSdmV27 *h_v27_tx(Fake f) { return faxw_v27_txsdm(f.obj); }
static short *h_v27_rate(Fake f) { return faxw_v27_rate(f.obj); }
static FaxSdmV27 *h_v27_rx(Fake f) { return faxw_v27_rxsdm(f.obj); }
static unsigned h_v17_tx_extra(Fake f) { return (unsigned)(unsigned short)faxw_v17_txsmc(f.obj)->x; }
#endif

/* one fake object per modem: V.17 and V.27ter share the TX state slot (obj+0x28, sdm at +0x1c) in the blob, so they cannot live in one object */
static void dump_o17(const char *tag, Fake f)
{
  char t[64];
  snprintf(t, sizeof t, "%s v17tx", tag);
  dump_sdm(t, h_v17_tx(f));
  snprintf(t, sizeof t, "%s v17rx", tag);
  dump_sdm(t, h_v17_rx(f));
  emit(0, "%s mode17=%u smcx=%d smc-raw=%u", tag, *h_v17_mode(f), h_v17_smc(f)->x, h_v17_tx_extra(f));
}
static void dump_o29(const char *tag, Fake f)
{
  char t[64];
  snprintf(t, sizeof t, "%s v29tx", tag);
  dump_sdm(t, h_v29_tx(f));
  snprintf(t, sizeof t, "%s v29rx", tag);
  dump_sdm(t, h_v29_rx(f));
  emit(0, "%s sel29=%d", tag, *h_v29_sel(f));
}
static void dump_o27(const char *tag, Fake f)
{
  char t[64];
  snprintf(t, sizeof t, "%s v27tx", tag);
  dump_v27(t, h_v27_tx(f));
  snprintf(t, sizeof t, "%s v27rx", tag);
  dump_v27(t, h_v27_rx(f));
  emit(0, "%s rate27=%d", tag, *h_v27_rate(f));
}
static void poke_sdm(FaxSdm *s)
{
  FaxSdmCfg cfg;
  cfg.s[0] = (short)(1 + rmod(4));
  cfg.s[1] = (short)(5 + rmod(20));
  cfg.s[2] = (short)(5 + rmod(20));
  SDM_init(s, &cfg);
  s->state = rnd();
}
static void poke_v27(FaxSdmV27 *s)
{
  FaxSdmCfg27 cfg;
  cfg.n = (unsigned short)(rpct(50) ? 2 : 3);
  SDMv27_init(s, &cfg);
  s->reg = (unsigned short)rnd16();
  s->cnt = (unsigned short)(rpct(30) ? 0x1d + rmod(5) : rmod(8));
}
static void scen7(void)
{
  int trial;
  for (trial = 0; trial < 400; trial++) {
    Fake f17 = fake_new(), f27 = fake_new(), f29 = fake_new();
    unsigned n = rpct(10) ? 0 : 1 + rmod(60);
    unsigned short *a = g_big;
    unsigned short buf[64];
    poke_sdm(h_v17_tx(f17));
    poke_sdm(h_v17_rx(f17));
    poke_sdm(h_v29_tx(f29));
    poke_sdm(h_v29_rx(f29));
    poke_v27(h_v27_tx(f27));
    poke_v27(h_v27_rx(f27));
    *h_v17_mode(f17) = (unsigned short)rnd16();
    h_v17_smc(f17)->x = (short)rnd16();
    *h_v29_sel(f29) = (int)rnd();
    *h_v27_rate(f27) = (short)rmod(2);
    dump_o17("poked", f17);
    dump_o29("poked", f29);
    dump_o27("poked", f27);
    fill_us(a, n, 0);
    memcpy(buf, a, n * 2);
    ScrambleDataV17(f17.obj, buf, (unsigned short)n);
    cov_hit(CV_W_SCR17);
    emit(0, "ScrambleDataV17 n=%u h=%016llx", n, fnv_us(buf, n));
    dump_o17("scr17", f17);
    memcpy(buf, a, n * 2);
    DescrambleDataV17(f17.obj, buf, (unsigned short)n);
    cov_hit(CV_W_DES17);
    emit(0, "DescrambleDataV17 n=%u h=%016llx", n, fnv_us(buf, n));
    dump_o17("des17", f17);
    memcpy(buf, a, n * 2);
    ScrambleDataV29(f29.obj, buf, (unsigned short)n);
    cov_hit(CV_W_SCR29);
    emit(0, "ScrambleDataV29 n=%u h=%016llx", n, fnv_us(buf, n));
    dump_o29("scr29", f29);
    memcpy(buf, a, n * 2);
    DescrambleDataV29(f29.obj, buf, (unsigned short)n);
    cov_hit(CV_W_DES29);
    emit(0, "DescrambleDataV29 n=%u h=%016llx", n, fnv_us(buf, n));
    dump_o29("des29", f29);
    memcpy(buf, a, n * 2);
    ScrambleDataV27(f27.obj, buf, (short)n);
    cov_hit(CV_W_SCR27);
    emit(0, "ScrambleDataV27 n=%u h=%016llx", n, fnv_us(buf, n));
    dump_o27("scr27", f27);
    memcpy(buf, a, n * 2);
    DescrambleDataV27(f27.obj, buf, (short)n);
    cov_hit(CV_W_DES27);
    emit(0, "DescrambleDataV27 n=%u h=%016llx", n, fnv_us(buf, n));
    dump_o27("des27", f27);
    {
      unsigned seed = rnd();
      SeedScramblerV17(f17.obj, seed);
      cov_hit(CV_W_SEED17);
      dump_o17("seed17", f17);
      seed = rnd();
      SeedScramblerV29(f29.obj, seed);
      cov_hit(CV_W_SEED29);
      dump_o29("seed29", f29);
    }
    {
      static const short ws[] = {0, 1, 2, 3, -1, 100, 32767};
      short w = ws[rmod(sizeof ws / sizeof ws[0])], v = (short)rnd16();
      SetEncoderV17(f17.obj, w, v);
      cov_hit(w == 0 ? CV_W_ENC17_0 : w == 1 ? CV_W_ENC17_1 : w == 2 ? CV_W_ENC17_2 : CV_W_ENC17_X);
      emit(0, "enc17 which=%d val=%d", w, v);
      dump_o17("enc17", f17);
      w = ws[rmod(sizeof ws / sizeof ws[0])];
      SetEncoderV29(f29.obj, w);
      cov_hit(w == 0 ? CV_W_ENC29_0 : w == 1 ? CV_W_ENC29_1 : CV_W_ENC29_X);
      emit(0, "enc29 which=%d", w);
      dump_o29("enc29", f29);
    }
    {
      *h_v27_rate(f27) = (short)(trial & 1);
      poke_v27(h_v27_tx(f27));
      h_v27_tx(f27)->f1 = (unsigned short)rmod(3);
      h_v27_tx(f27)->cnt = (unsigned short)rmod(0x30);
      SetScramblerV27(f27.obj);
      cov_hit((trial & 1) ? CV_W_SET27_1 : CV_W_SET27_0);
      dump_o27("set27", f27);
    }
  }
}

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : -1;
  const char *e;
  if (scen < 0 || scen > 7) {
    fprintf(stderr, "usage: %s scenario(0..7)\n", argv[0]);
    return 2;
  }
  g_verbose = (e = getenv("R18_VERBOSE")) != NULL && *e == '1';
  if ((e = getenv("R18_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(e, NULL, 0);
  g_lcg += (unsigned)scen * 0x9e3779b9u;
  cov_setup(scen);
  switch (scen) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    case 4: scen4(); break;
    case 5: scen5(); break;
    case 6: scen6(); break;
    default: scen7(); break;
  }
  cov_finish();
  return 0;
}
