/*
 * R27 differential harness: FAXVMI of dsplibs.o (FAXVMI_create / _delete / _process / _status / _control / _message, the framing functions faxvmi_write_fifo / write_frame / simp_pack /
 * asyc_pack / hdlc_frame / simp_unpack / asyc_unpack / hdlc_unframe / gen_fcs16 / byte_reverse / frame_reverse, the null modem null_*, the ops tables vxx_*, the init_vmi_data_* tables)
 * vs src/fax/fax_vmi.c.  The blob run (-DTEST_BLOB, linked with ../dsplibs.o) and the port run (-m32 -O0 / -O2, x86-64 against the recorded blob output) execute the same code; library
 * test/fax/r27_common.h (hashed canonical output, coverage counters exit 2 on a zero, 3 = harness violation, deterministic heap with block ids / red zones / poisoned frees, callee recorders
 * sysdep_malloc / sysdep_free / dsplibs_debug_printf).  Everything printed is host independent: objects are dumped member by member, pointers as table+index / block id, blocks as hashes.
 *
 * Two binaries from this file:
 *   REC (default): the 48 modem wrappers v21tx_* ... v17rx_* (the blob's weakened with objcopy) and GetT30FrameIDFromBuffer / GetT30FrameNameByID are harness RECORDERS (arguments logged, scripted
 *       results: a transmit recorder consumes m->bits and makes samples, a receive recorder writes scripted bit words), the port links fax_vmi.c only.  Scenarios:
 *     0 wiring: every vxx_* table entry called for the ids 0 .. 12 (which recorder / null function fired, what it returned), the constant tables, FAXVMI_message;
 *     1 pure functions: faxvmi_gen_fcs16 (model of the CRC-16 asserted), byte_reverse, frame_reverse;
 *     2 ring writers: faxvmi_write_fifo / faxvmi_write_frame on random rings (full / partial / wrapped / poked counters);
 *     3 framing: simp / asyc / hdlc pack and unpack on random states and model streams (HDLC frames with stuffing, CRC errors and the repair paths, break patterns, overflow);
 *     4 lifecycle: FAXVMI_create (NULL / supplied objects, cfg variants, ids 0 .. 12), delete, control, status, message, null_*, forked malloc-failure probes;
 *     5 process: FAXVMI_process through the recorder modems, all modes x flags x ids, the state word flags;
 *   ENG (-DHARNESS_ENG): the real engines of both sides behind the ops tables (the blob's own v21 ... v17 wrappers / the ported ones), GetT30* recorders.  Scenarios:
 *     0 loopbacks: a transmit FAXVMI object -> samples -> a receive FAXVMI object for V.21 / V.27ter / V.29 / V.17, all three modes, flags, chunk sizes, the whole FAXVMI state dumped after
 *       every call;
 *     1 the init_vmi_data_* tables (every entry called for the rates, the modem object fields dumped), create / control / status / message / delete of FAXVMI objects over real engines.
 * R27_VERBOSE=1 prints every line (debugging), R27_SEED selects another input stream.
 */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "r27_common.h"
#include "fax/fax_v21.h"
#include "fax/fax_v27_tx.h"
#include "fax/fax_v27_rx.h"
#include "fax/fax_v29_tx.h"
#include "fax/fax_v29_rx.h"
#include "fax/fax_v17_tx.h"
#include "fax/fax_v17_rx.h"

#ifdef TEST_BLOB
#define SIDE_BLOB 1
#else
#define SIDE_BLOB 0
#endif
extern unsigned int dsplibs_debug_level;

/* ---------------------------------------------------------------------------------------------- coverage ids */
enum {
  /* 0 wiring */
  CV_W_TAB, CV_W_CREATE, CV_W_DELETE, CV_W_PROCESS_TX, CV_W_PROCESS_RX, CV_W_PROCESS_NULL, CV_W_STATUS, CV_W_CONTROL, CV_W_MESSAGE, CV_W_MSGNULL, CV_W_FMSG,
  /* 1 pure */
  CV_P_FCS, CV_P_FCS0, CV_P_FCSBIG, CV_P_FCSOK, CV_P_BREV, CV_P_BREV0, CV_P_BREVBIG, CV_P_FREV, CV_P_FREV0, CV_P_FREVLEN0,
  /* 2 ring writers */
  CV_R_FIFO, CV_R_FIFO_FULL, CV_R_FIFO_PART, CV_R_FIFO_N0, CV_R_FIFO_WRAP, CV_R_FIFO_W18, CV_R_FIFO_NEG, CV_R_FIFO_ALL, CV_R_FRAME, CV_R_FRAME_REJECT, CV_R_FRAME_MULTI, CV_R_FRAME_LEN0,
  CV_R_FRAME_WRAP, CV_R_FRAME_W18, CV_R_FRAME_N0, CV_R_FRAME_STOP,
  /* 3 framing */
  CV_F_SIMP_PACK, CV_F_SIMP_UNDER, CV_F_SIMP_DATA, CV_F_SIMP_MID, CV_F_ASYC_PACK, CV_F_ASYC_BREAK, CV_F_ASYC_BREAKEND, CV_F_ASYC_EMPTY, CV_F_ASYC_DATA, CV_F_ASYC_CHAR, CV_F_HDLC_PACK,
  CV_F_HDLC_IDLE, CV_F_HDLC_START, CV_F_HDLC_DATA, CV_F_HDLC_STUFF, CV_F_HDLC_DBG, CV_F_HDLC_NAME, CV_F_HDLC_WRAPDBG, CV_F_SIMP_UNPACK, CV_F_SIMP_OVER, CV_F_ASYC_UNPACK, CV_F_ASYC_OVER,
  CV_F_ASYC_STATE0, CV_F_ASYC_HUNT, CV_F_ASYC_ZEROS, CV_F_ASYC_BREAKFLAG, CV_F_ASYC_FRAMING, CV_F_HDLC_UNPACK, CV_F_HU_STUFFED, CV_F_HU_FLAG_SKIP, CV_F_HU_FRAME_OK, CV_F_HU_BAD, CV_F_HU_FIX1,
  CV_F_HU_FIX2, CV_F_HU_SHIFT, CV_F_HU_SHIFT_OK, CV_F_HU_SHIFT_FIX1, CV_F_HU_SHIFT_FIX2, CV_F_HU_GIVEUP, CV_F_HU_OVER, CV_F_HU_CAP, CV_F_HU_SEVEN, CV_F_HU_FLAGSONLY, CV_F_HU_MULTI, CV_F_HU_J7,
  CV_F_HU_DBG, CV_F_LOOP, CV_F_HU_HSKIP,
  /* 4 lifecycle */
  CV_L_NEW, CV_L_NEWNULL, CV_L_SUP, CV_L_CFG, CV_L_ID_NULL, CV_L_ID_REC, CV_L_DEL, CV_L_CTL, CV_L_CTL_NULL, CV_L_CTL_M, CV_L_CTL_RESET, CV_L_CTL_RESET_BIG, CV_L_CTL_FLUSH, CV_L_ST, CV_L_ST_NULL, CV_L_ST_M,
  CV_L_MSG, CV_L_NULLFN, CV_L_PROBE, CV_L_SIZES, CV_L_SIZE0,
  /* 5 process */
  CV_X_PROC, CV_X_M0, CV_X_M1, CV_X_M2, CV_X_FLAG, CV_X_NOFLAG, CV_X_TX, CV_X_RX, CV_X_NULL, CV_X_S_W18, CV_X_S_ROOM, CV_X_S_OVF, CV_X_S_W1C, CV_X_S_W3C, CV_X_RETBITS, CV_X_MANY,
  /* ENG 0 / 1 */
  CV_E_LOOP, CV_E_V21, CV_E_V27, CV_E_V29, CV_E_V17, CV_E_M0, CV_E_M1, CV_E_M2, CV_E_FLAG, CV_E_BYTES, CV_E_FRAMES, CV_E_TXSAMPLES, CV_E_RXBYTES, CV_E_RXFRAMES, CV_E_INIT, CV_E_INITRX, CV_E_LIFE, CV_E_CTL, CV_E_ST, CV_E_MSG
};
static void cov_setup(int scen)
{
#ifndef HARNESS_ENG
  switch (scen) {
    case 0:
      cov_def(CV_W_TAB, "constant tables dumped");
      cov_def(CV_W_CREATE, "vxx_create entries");
      cov_def(CV_W_DELETE, "vxx_delete entries");
      cov_def(CV_W_PROCESS_TX, "vxx_process transmit entries");
      cov_def(CV_W_PROCESS_RX, "vxx_process receive entries");
      cov_def(CV_W_PROCESS_NULL, "vxx_process null entries");
      cov_def(CV_W_STATUS, "vxx_status entries");
      cov_def(CV_W_CONTROL, "vxx_control entries");
      cov_def(CV_W_MESSAGE, "vxx_message entries");
      cov_def(CV_W_MSGNULL, "message NULL result");
      cov_def(CV_W_FMSG, "FAXVMI_message");
      break;
    case 1:
      cov_def(CV_P_FCS, "gen_fcs16");
      cov_def(CV_P_FCS0, "gen_fcs16 n = 0");
      cov_def(CV_P_FCSBIG, "gen_fcs16 n = -1");
      cov_def(CV_P_FCSOK, "gen_fcs16 over frame + crc = 0xe2f0");
      cov_def(CV_P_BREV, "byte_reverse");
      cov_def(CV_P_BREV0, "byte_reverse n = 0");
      cov_def(CV_P_BREVBIG, "byte_reverse n = -1");
      cov_def(CV_P_FREV, "frame_reverse");
      cov_def(CV_P_FREV0, "frame_reverse n = 0");
      cov_def(CV_P_FREVLEN0, "frame_reverse zero length frame");
      break;
    case 2:
      cov_def(CV_R_FIFO, "write_fifo");
      cov_def(CV_R_FIFO_FULL, "write_fifo ring full at entry");
      cov_def(CV_R_FIFO_PART, "write_fifo partial");
      cov_def(CV_R_FIFO_N0, "write_fifo n = 0");
      cov_def(CV_R_FIFO_WRAP, "write_fifo wrapped");
      cov_def(CV_R_FIFO_W18, "write_fifo clears w18");
      cov_def(CV_R_FIFO_NEG, "write_fifo n < 0");
      cov_def(CV_R_FIFO_ALL, "write_fifo all queued");
      cov_def(CV_R_FRAME, "write_frame");
      cov_def(CV_R_FRAME_REJECT, "write_frame rejects (no room)");
      cov_def(CV_R_FRAME_MULTI, "write_frame several frames");
      cov_def(CV_R_FRAME_LEN0, "write_frame zero length frame");
      cov_def(CV_R_FRAME_WRAP, "write_frame wrapped");
      cov_def(CV_R_FRAME_W18, "write_frame clears w18");
      cov_def(CV_R_FRAME_N0, "write_frame n = 0");
      cov_def(CV_R_FRAME_STOP, "write_frame stops after room ran out");
      break;
    case 3:
      cov_def(CV_F_SIMP_PACK, "simp_pack");
      cov_def(CV_F_SIMP_UNDER, "simp_pack under-run (ring empty)");
      cov_def(CV_F_SIMP_DATA, "simp_pack data from the ring");
      cov_def(CV_F_SIMP_MID, "simp_pack mid-byte state");
      cov_def(CV_F_ASYC_PACK, "asyc_pack");
      cov_def(CV_F_ASYC_BREAK, "asyc_pack break (w38, w34 > 0)");
      cov_def(CV_F_ASYC_BREAKEND, "asyc_pack break end (w38, w34 = 0)");
      cov_def(CV_F_ASYC_EMPTY, "asyc_pack ring empty");
      cov_def(CV_F_ASYC_DATA, "asyc_pack data from the ring");
      cov_def(CV_F_ASYC_CHAR, "asyc_pack word complete");
      cov_def(CV_F_HDLC_PACK, "hdlc_frame");
      cov_def(CV_F_HDLC_IDLE, "hdlc_frame idle flags");
      cov_def(CV_F_HDLC_START, "hdlc_frame frame start (length word)");
      cov_def(CV_F_HDLC_DATA, "hdlc_frame data byte");
      cov_def(CV_F_HDLC_STUFF, "hdlc_frame stuffed bit");
      cov_def(CV_F_HDLC_DBG, "hdlc_frame debug dump");
      cov_def(CV_F_HDLC_NAME, "hdlc_frame frame name");
      cov_def(CV_F_HDLC_WRAPDBG, "hdlc_frame debug dump of a wrapped ring");
      cov_def(CV_F_SIMP_UNPACK, "simp_unpack");
      cov_def(CV_F_SIMP_OVER, "simp_unpack overflow (> s0a bytes)");
      cov_def(CV_F_ASYC_UNPACK, "asyc_unpack");
      cov_def(CV_F_ASYC_OVER, "asyc_unpack overflow");
      cov_def(CV_F_ASYC_STATE0, "asyc_unpack data state");
      cov_def(CV_F_ASYC_HUNT, "asyc_unpack hunting");
      cov_def(CV_F_ASYC_ZEROS, "asyc_unpack 23 zeros");
      cov_def(CV_F_ASYC_BREAKFLAG, "asyc_unpack break end flag (w3c)");
      cov_def(CV_F_ASYC_FRAMING, "asyc_unpack model characters");
      cov_def(CV_F_HDLC_UNPACK, "hdlc_unframe");
      cov_def(CV_F_HU_STUFFED, "hdlc_unframe stuffed zero");
      cov_def(CV_F_HU_FLAG_SKIP, "hdlc_unframe initial flag skipped");
      cov_def(CV_F_HU_FRAME_OK, "hdlc_unframe frame delivered with a good CRC");
      cov_def(CV_F_HU_BAD, "hdlc_unframe bad CRC");
      cov_def(CV_F_HU_FIX1, "hdlc_unframe first byte repair");
      cov_def(CV_F_HU_FIX2, "hdlc_unframe second byte repair");
      cov_def(CV_F_HU_SHIFT, "hdlc_unframe shifting");
      cov_def(CV_F_HU_SHIFT_OK, "hdlc_unframe shift repair succeeded");
      cov_def(CV_F_HU_SHIFT_FIX1, "hdlc_unframe shift + first byte repair");
      cov_def(CV_F_HU_SHIFT_FIX2, "hdlc_unframe shift + second byte repair");
      cov_def(CV_F_HU_GIVEUP, "hdlc_unframe repair failed (empty record)");
      cov_def(CV_F_HU_OVER, "hdlc_unframe output overflow");
      cov_def(CV_F_HU_CAP, "hdlc_unframe frame buffer full");
      cov_def(CV_F_HU_SEVEN, "hdlc_unframe seven ones");
      cov_def(CV_F_HU_FLAGSONLY, "hdlc_unframe flags only");
      cov_def(CV_F_HU_MULTI, "hdlc_unframe several frames per call");
      cov_def(CV_F_HU_J7, "hdlc_unframe shift loop ran 7 rounds");
      cov_def(CV_F_HU_DBG, "hdlc_unframe debug level 2");
      cov_def(CV_F_LOOP, "hdlc_frame -> hdlc_unframe loopback");
      cov_def(CV_F_HU_HSKIP, "hdlc_unframe hskip != 0 state poke");
      break;
    case 4:
      cov_def(CV_L_NEW, "FAXVMI_create NULL object");
      cov_def(CV_L_NEWNULL, "FAXVMI_create NULL object NULL cfg");
      cov_def(CV_L_SUP, "FAXVMI_create supplied object");
      cov_def(CV_L_CFG, "FAXVMI_create custom cfg");
      cov_def(CV_L_ID_NULL, "create id 0 .. 4 (null modem)");
      cov_def(CV_L_ID_REC, "create id 5 .. 12");
      cov_def(CV_L_DEL, "FAXVMI_delete");
      cov_def(CV_L_CTL, "FAXVMI_control");
      cov_def(CV_L_CTL_NULL, "FAXVMI_control NULL");
      cov_def(CV_L_CTL_M, "FAXVMI_control with a modem record");
      cov_def(CV_L_CTL_RESET, "FAXVMI_control reset (wc, s10 <= 2)");
      cov_def(CV_L_CTL_RESET_BIG, "FAXVMI_control wc with s10 > 2");
      cov_def(CV_L_CTL_FLUSH, "FAXVMI_control w0 (flush)");
      cov_def(CV_L_ST, "FAXVMI_status");
      cov_def(CV_L_ST_NULL, "FAXVMI_status NULL");
      cov_def(CV_L_ST_M, "FAXVMI_status with a modem record");
      cov_def(CV_L_MSG, "FAXVMI_message");
      cov_def(CV_L_NULLFN, "null_* direct");
      cov_def(CV_L_PROBE, "create malloc-failure probe");
      cov_def(CV_L_SIZES, "ring / frame sizes (max rule)");
      cov_def(CV_L_SIZE0, "size 0 ring / frame");
      break;
    case 5:
      cov_def(CV_X_PROC, "FAXVMI_process");
      cov_def(CV_X_M0, "mode 0");
      cov_def(CV_X_M1, "mode 1");
      cov_def(CV_X_M2, "mode 2");
      cov_def(CV_X_FLAG, "flag (bit reverse)");
      cov_def(CV_X_NOFLAG, "no flag");
      cov_def(CV_X_TX, "transmit recorder");
      cov_def(CV_X_RX, "receive recorder");
      cov_def(CV_X_NULL, "null modem");
      cov_def(CV_X_S_W18, "state flag 0x01000000");
      cov_def(CV_X_S_ROOM, "state flag 0x02000000");
      cov_def(CV_X_S_OVF, "state flag 0x04000000");
      cov_def(CV_X_S_W1C, "state flag 0x08000000");
      cov_def(CV_X_S_W3C, "state flag 0x10000000 cleared");
      cov_def(CV_X_RETBITS, "recorder state bits through the mask");
      cov_def(CV_X_MANY, "many calls");
      break;
  }
#else
  switch (scen) {
    case 0:
      cov_def(CV_E_LOOP, "loopback trials");
      cov_def(CV_E_V21, "V.21");
      cov_def(CV_E_V27, "V.27ter");
      cov_def(CV_E_V29, "V.29");
      cov_def(CV_E_V17, "V.17");
      cov_def(CV_E_M0, "mode 0");
      cov_def(CV_E_M1, "mode 1");
      cov_def(CV_E_M2, "mode 2");
      cov_def(CV_E_FLAG, "flag");
      cov_def(CV_E_TXSAMPLES, "transmit samples produced");
      cov_def(CV_E_RXBYTES, "receive bytes delivered");
      cov_def(CV_E_RXFRAMES, "receive frames delivered");
      break;
    case 1:
      cov_def(CV_E_INIT, "init_vmi_data_tx_modem entries");
      cov_def(CV_E_INITRX, "init_vmi_data_rx_modem entries");
      cov_def(CV_E_LIFE, "FAXVMI over real engines: create / delete");
      cov_def(CV_E_CTL, "control");
      cov_def(CV_E_ST, "status");
      cov_def(CV_E_MSG, "message");
      break;
  }
#endif
}

/* ---------------------------------------------------------------------------------------------- callee recorders */
int dsplibs_debug_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int dsplibs_debug_printf(const char *fmt, ...)
{
  char b[256];
  char *q;
  int n;
  va_list ap;
  va_start(ap, fmt);
  n = vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  for (q = b; *q; q++)
    if (*q == '\n') *q = '|';
  emit(0, "DBG %s", b);
  if (!strncmp(b, "HDLC Transmitted", 16)) cov_hit(CV_F_HDLC_DBG);
  if (!strncmp(b, "FCL1: FRAME", 11)) cov_hit(CV_F_HDLC_NAME);
  if (!strncmp(b, "Frame with bad CRC", 18)) cov_hit(CV_F_HU_BAD);
  if (!strncmp(b, "Replacing the first byte...", 27)) cov_hit(CV_F_HU_FIX1);
  if (!strncmp(b, "Replacing the second byte...", 28)) cov_hit(CV_F_HU_FIX2);
  if (!strncmp(b, "Shifting data", 13)) cov_hit(CV_F_HU_SHIFT);
  if (!strncmp(b, "Shifting data 7", 15)) cov_hit(CV_F_HU_J7);
  if (!strncmp(b, "Replacing first byte", 20)) cov_hit(CV_F_HU_SHIFT_FIX1);
  if (!strncmp(b, "Replacing the second byte ...", 29)) cov_hit(CV_F_HU_SHIFT_FIX2);
  if (!strncmp(b, "CRC is now OK", 13)) cov_hit(CV_F_HU_SHIFT_OK);
  return n;
}
unsigned int GetT30FrameIDFromBuffer(unsigned char b0, unsigned char b1, unsigned char b2)
{
  unsigned id = ((unsigned)b0 << 16) | ((unsigned)b1 << 8) | b2 | 0x8000u;
  emit(0, "GetT30FrameIDFromBuffer %02x %02x %02x -> %x", b0, b1, b2, id);
  return id;
}
const char *GetT30FrameNameByID(unsigned int id)
{
  static char name[32];
  snprintf(name, sizeof name, "T30FRAME_%x", id);
  emit(0, "GetT30FrameNameByID %x", id);
  return name;
}

/* ---------------------------------------------------------------------------------------------- recorder modems (REC build) */
typedef struct {
  short s0c, s0e, s10;
} RecCfg;
static RecCfg g_rc[8];
static unsigned char g_sbuf[64], g_cbuf[64];
static short g_samples[1024];
static short g_bits_store[64];
static short g_buf0_store[256];
static unsigned g_rec_ret = 0x5001;
static unsigned g_rec_k = 7;
static unsigned short g_rec_src[1024];
static unsigned g_rec_pos, g_rec_len = 1;
static char g_msgs[16][4][12];
static const int g_istx[13] = {1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0};

#ifndef HARNESS_ENG
static uintptr_t rec_create(int id, FaxVmiM *m, const void *cfg)
{
  const RecCfg *c = (const RecCfg *)cfg;
  char b[24];
  char b2[24];
  emit(0, "REC create id=%d cfg=%s eng_in=%s", id, dptr(cfg, b, sizeof b), dptr(m->eng, b2, sizeof b2));
  if (c) {
    m->s0c = c->s0c;
    m->s0e = c->s0e;
    m->u10.s10 = c->s10;
  } else {
    m->s0c = 20;
    m->s0e = 4;
    m->u10.s10 = 4;
  }
  m->eng = NULL;
  return 0x1000u + (unsigned)id * 16u;
}
static void rec_delete(int id, FaxVmiM *m)
{
  (void)m;
  emit(0, "REC delete id=%d", id);
}
static int rec_process(int id, FaxVmiM *m, short *s, unsigned short *a, unsigned short *b)
{
  if (g_istx[id]) {
    unsigned n = *a, i;
    u64 h = n <= 50 ? fnv_shorts(m->u4.bits, n) : 0;
    emit(0, "REC txp id=%d a=%u b=%u bits=%llx", id, n, (unsigned)*b, h);
    if (n > 50) die3("tx recorder: %u words", n);
    for (i = 0; i < n * 3u; i++) s[i] = (short)(h * 31 + i * 7 + (unsigned)id);
    *b = (unsigned short)(n * 3u);
    *a = 0;
  } else {
    unsigned nin = *b, i;
    u64 h = nin <= 1024 ? fnv_shorts(s, nin) : 0;
    unsigned k = g_rec_k;
    emit(0, "REC rxp id=%d a=%u b=%u in=%llx k=%u", id, (unsigned)*a, nin, h, k);
    if (k > 50) die3("rx recorder: %u words", k);
    for (i = 0; i < k; i++) {
      m->u4.bits[i] = (short)g_rec_src[g_rec_pos];
      g_rec_pos = (g_rec_pos + 1) % g_rec_len;
    }
    *a = (unsigned short)k;
    *b = 0;
  }
  return (int)g_rec_ret;
}
static int rec_status(int id, FaxVmiM *m, void *st)
{
  unsigned char *p = (unsigned char *)st;
  (void)m;
  emit(0, "REC status id=%d st=%s", id, st ? "set" : "NULL");
  if (p) {
    p[0] = (unsigned char)(0x40 + id);
    p[1]++;
  }
  return 100 + id;
}
static int rec_control(int id, FaxVmiM *m, const void *ctl)
{
  (void)m;
  emit(0, "REC control id=%d ctl=%llx", id, ctl ? fnv_bytes(ctl, 8) : 0ull);
  return 200 + id;
}
static void rec_message(int id, FaxVmiM *m, int idx, const char **out)
{
  (void)m;
  emit(0, "REC message id=%d idx=%d", id, idx);
  *out = (idx >= 0 && idx < 3) ? g_msgs[id][idx] : NULL;
}
#define REC_MODEM(N, ID, RT, CT, ST, KT) \
  RT N##_create(FaxVmiM *m, CT cfg) { return (RT)(uintptr_t)rec_create(ID, m, cfg); } \
  void N##_delete(FaxVmiM *m) { rec_delete(ID, m); } \
  int N##_process(FaxVmiM *m, short *s, unsigned short *a, unsigned short *b) { return rec_process(ID, m, s, a, b); } \
  int N##_status(FaxVmiM *m, ST st) { return rec_status(ID, m, st); } \
  int N##_control(FaxVmiM *m, KT ctl) { return rec_control(ID, m, ctl); } \
  void N##_message(FaxVmiM *m, int idx, const char **out) { rec_message(ID, m, idx, out); }
REC_MODEM(v21tx, 5, void *, const int *, FaxV21Status *, const FaxV21Ctl *)
REC_MODEM(v21rx, 6, void *, const int *, FaxV21Status *, const FaxV21Ctl *)
REC_MODEM(v27tx, 7, int, const int *, FaxV27Status *, const FaxV27Ctl *)
REC_MODEM(v27rx, 8, void *, const int *, void *, const FaxV27Ctl *)
REC_MODEM(v29tx, 9, V29Tx *, const int *, FaxV29Status *, const FaxV29Ctl *)
REC_MODEM(v29rx, 10, int, const int *, FaxV29Status *, const FaxV29Ctl *)
REC_MODEM(v17tx, 11, unsigned int, const int *, FaxV17Status *, const FaxV17Ctl *)
REC_MODEM(v17rx, 12, int, const V17RxCfg *, FaxV17Status *, const FaxV17Ctl *)
/* the port's fax_vmi.c references the modem initialisers through the init_vmi_data_* tables: never called here */
int fax_init_vmi_v27tx(FaxVmiV27 *m, unsigned short r, int u, void *x) { (void)m; (void)r; (void)u; (void)x; return 0; }
int fax_init_vmi_v29tx(FaxVmiV29 *m, unsigned short r, int u, void *x) { (void)m; (void)r; (void)u; (void)x; return 0; }
int fax_init_vmi_v17tx(FaxVmiV17 *m, unsigned short r, int u, void *x) { (void)m; (void)r; (void)u; (void)x; return 0; }
int fax_init_vmi_v27rx(FaxVmiV27 *m, unsigned short r, int u, void *x) { (void)m; (void)r; (void)u; (void)x; return 0; }
int fax_init_vmi_v29rx(FaxVmiV29 *m, unsigned short r, int u, void *x) { (void)m; (void)r; (void)u; (void)x; return 0; }
int fax_init_vmi_v17rx(FaxVmiV17 *m, unsigned short r, int u, void *x) { (void)m; (void)r; (void)u; (void)x; return 0; }
#endif

/* ---------------------------------------------------------------------------------------------- objects, dumps */
static void defaults_init(void)
{
  int i, k;
  htab_add("rc", g_rc, sizeof g_rc);
  for (i = 0; i < 16; i++)
    for (k = 0; k < 4; k++) snprintf(g_msgs[i][k], sizeof g_msgs[i][k], "m%d.%d", i, k);
  for (i = 0; i < 1024; i++) g_rec_src[i] = (unsigned short)rnd();
  g_rec_len = 1024;
  /* the fixed buffers (0x64 / 0x190 bytes) must not look like a typed object on either host */
  {
    unsigned t[] = {(unsigned)sizeof(FaxVmi), (unsigned)sizeof(FaxVmiSt), (unsigned)sizeof(FaxVmiM)};
    for (i = 0; i < 3; i++)
      if (t[i] == 0x64 || t[i] == 0x190) die3("typed object size equals a fixed buffer size");
  }
}
static void dump_vmi(const char *tag, const FaxVmi *o)
{
  char b1[24], b2[24], b3[24];
  const FaxVmiSt *s = o->st;
  const FaxVmiM *m = o->m;
  emit(0, "%s O mode=%08x flag=%d s08=%u s0a=%u s0c=%u id=%u mcfg=%s w14=%d w18=%d w1c=%d w20=%08x", tag, (unsigned)o->cfg.mode, o->cfg.flag, (unsigned)o->cfg.ringMin, (unsigned)o->cfg.maxItems,
       (unsigned)o->cfg.frmWords, (unsigned)o->cfg.id, dptr(o->cfg.mcfg, b1, sizeof b1), o->cfg.w14, o->txEmpty, o->rxOverflow, o->stateWord);
  emit(0, "%s S size=%u rd=%u wr=%u cnt=%u ovf=%d txm=%x txc=%x txa=%x txn=%u rxm=%x rxc=%x rxa=%x rxn=%u rxs=%d w34=%u w38=%d w3c=%d fsz=%u tl=%d fl=%d hs=%u tf=%d on=%d in=%d", tag,
       (unsigned)s->size, (unsigned)s->rd, (unsigned)s->wr, (unsigned)s->cnt, s->ovf, (unsigned)s->tx_mask, (unsigned)s->tx_cur, (unsigned)s->tx_acc, (unsigned)s->tx_nbits, (unsigned)s->rx_mask,
       (unsigned)s->rx_cur, (unsigned)s->rx_acc, (unsigned)s->rx_nbits, s->rx_state, (unsigned)s->breakLeft, s->breakReq, s->rxBreak, (unsigned)s->frm_size, (int)s->tx_left, (int)s->flen,
       (unsigned)s->hskip, s->tx_flag, (int)s->ones, s->in_frame);
  emit(0, "%s R ring=%016llx frm=%016llx", tag, s->ring ? fnv_shorts((const short *)s->ring, s->size) : 0ull, s->frm ? fnv_shorts((const short *)s->frm, s->frm_size) : 0ull);
  emit(0, "%s M s0c=%d s0e=%d s10=%d bits=%016llx buf0=%016llx eng=%s", tag, (int)m->s0c, (int)m->s0e, (int)m->u10.s10, m->u4.bits ? fnv_shorts(m->u4.bits, 50) : 0ull,
       m->buf0 ? fnv_shorts(m->buf0, 200) : 0ull, dptr(m->eng, b2, sizeof b2));
  (void)b3;
}
static void rc_set(int k, int s0c, int s0e, int s10);
/* random recorder config: every draw in its own statement (the evaluation order of call arguments differs between the hosts) */
static void rc_rand(int k, int s0cmax)
{
  int a, b, c;
  a = 1 + (int)rmod((unsigned)s0cmax);
  b = 1 + (int)rmod(16);
  c = 1 + (int)rmod(16);
  rc_set(k, a, b, c);
}
static FaxVmiCfg mkcfg(int mode, int flag, int s08, int s0a, int s0c, int id, const void *mcfg)
{
  FaxVmiCfg c;
  memset(&c, 0, sizeof c);
  c.mode = mode;
  c.flag = flag;
  c.ringMin = (unsigned short)s08;
  c.maxItems = (unsigned short)s0a;
  c.frmWords = (unsigned short)s0c;
  c.id = (unsigned short)id;
  c.mcfg = mcfg;
  c.w14 = 0;
  return c;
}
/* make the ring / frame buffer byte sizes differ from every typed object size */
static void fix_cfg(FaxVmiCfg *c)
{
  for (;;) {
    unsigned s3 = (unsigned)c->maxItems + 3u;
    unsigned sz = (unsigned)c->ringMin >= s3 ? (unsigned)c->ringMin : s3;
    if (safe_bytes(2 * sz) == 2 * sz) break;
    c->ringMin = (unsigned short)(c->ringMin + 1);
  }
  while (safe_bytes(2u * c->frmWords) != 2u * c->frmWords) c->frmWords = (unsigned short)(c->frmWords + 1);
}
static FaxVmi *vmi_new(FaxVmiCfg c, int id_cov)
{
  FaxVmi *o;
  (void)id_cov;
  fix_cfg(&c);
  o = FAXVMI_create(NULL, &c);
  return o;
}
static void rc_set(int k, int s0c, int s0e, int s10)
{
  g_rc[k].s0c = (short)s0c;
  g_rc[k].s0e = (short)s0e;
  g_rc[k].s10 = (short)s10;
}
/* poke random framing state (transmit and receive halves) */
static void poke_tx(FaxVmiSt *s)
{
  static const unsigned masks[] = {0, 0, 0x80, 0x200, 0x1, 0x40, 0x100, 0x800000, 0x20, 0xffff};
  s->tx_mask = (int)masks[rmod(10)];
  s->tx_cur = (int)rnd();
  s->tx_acc = (int)rnd();
  s->tx_nbits = (unsigned short)rmod(10);
  s->breakLeft = (unsigned short)(rpct(50) ? rmod(4) : 0);
  s->breakReq = rpct(30) ? 1 : 0;
  s->tx_left = (short)(rpct(60) ? 0 : rmod(6));
  s->tx_flag = rpct(50) ? 1 : 0;
}
static void poke_rx(FaxVmiSt *s)
{
  static const unsigned masks[] = {0, 0, 0x80, 0x200, 0x1, 0x40, 0x8000, 0x10, 0x4, 0xffff};
  s->rx_mask = (int)masks[rmod(10)];
  s->rx_cur = (int)rnd();
  s->rx_acc = (int)rnd();
  s->rx_nbits = (unsigned short)rmod(9);
  s->rx_state = rpct(50) ? 1 : (int)rmod(2);
  s->rxBreak = rpct(10);
  s->flen = (short)(rpct(70) ? 0 : rmod(5));
  s->hskip = (unsigned short)(rpct(60) ? 0 : rmod(3));
  s->ones = (short)(rpct(60) ? 0 : rmod(8));
  s->in_frame = rpct(30);
}
/* make the ring hold `cnt` items starting at rd (consistent wr) */
static void ring_set(FaxVmiSt *s, unsigned rd, unsigned cnt)
{
  unsigned i;
  if (s->size == 0) return;
  rd %= s->size;
  if (cnt > s->size) cnt = s->size;
  s->rd = (unsigned short)rd;
  s->cnt = (unsigned short)cnt;
  s->wr = (unsigned short)((rd + cnt) % s->size);
  for (i = 0; i < s->size; i++) s->ring[i] = (unsigned short)rnd();
}

static void pr_str(const char *s, char *b, size_t cap)
{
  if (!s) snprintf(b, cap, "NULL");
  else snprintf(b, cap, "\"%s\"", s);
}

#include "r27_frame.h"
#include "r27_life.h"
#ifdef HARNESS_ENG
#include "r27_eng.h"
#endif

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : 0;
  const char *sd = getenv("R27_SEED");
  g_verbose = getenv("R27_VERBOSE") != NULL;
  if (sd) g_lcg ^= (unsigned)strtoul(sd, NULL, 0) * 2654435761u;
  g_lcg += (unsigned)scen * 7919u;
  defaults_init();
  cov_setup(scen);
#ifndef HARNESS_ENG
  switch (scen) {
    case 0: scen_wiring(); break;
    case 1: scen_pure(); break;
    case 2: scen_ring(); break;
    case 3: scen_framing(); break;
    case 4: scen_life(); break;
    case 5: scen_process(); break;
    default: fprintf(stderr, "scenario 0..5\n"); return 2;
  }
#else
  switch (scen) {
    case 0: scen_loop(); break;
    case 1: scen_eng_life(); break;
    default: fprintf(stderr, "scenario 0..1\n"); return 2;
  }
#endif
  cov_finish();
  return 0;
}
