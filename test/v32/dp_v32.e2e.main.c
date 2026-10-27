/*
 * R03 differential harness, id r03_e2e: the V.32 / V.32bis dp glue with the REAL V.32 engine and the REAL dp_wrapper / RcFixed behind it on both sides: the blob's v32_* glue + V32FP_*
 * + dp_wrapper_* + RcFixed_* (dsplibs.o, 32-bit) versus src/v32/dp_v32.c + src/v32/v32_engine.c (engine) + src/core/dp_wrapper.c + src/core/rcfixed_resampler.c.  Two glue instances (caller and answerer) are created
 * through the record dp_v32_init registers and run through op->process = dp_wrapper_run with the output audio of each fed to the other (one call delayed) at 9600 Hz (through the
 * rate converters) and 8000 Hz (no converters), for DP_V32BIS and DP_V32; slmodemd's modem is the fake of test/r02_common.h (scripted bit supply, put_bits / set_param recorders).
 * Honest scope: two cold-started V.32 engines do NOT converge to CONNECT (the V.8 / V.25 pre-phase is not modelled, test/v32/v32_engine.capstone.main.c): the sessions run the handshake
 * until the engines' own time-outs report an error, the glue turns that into dp status 4 and then zeroes every output block.  So this covers create, the whole pre-connect
 * block loop through the converters, the error exit, the zeroed blocks and delete over real engines; the CONNECT branch of v32_process is in r03_glue (fake engine).
 * Scenarios (argv[1]): 0 = 9600 Hz DP_V32BIS, 1 = 8000 Hz DP_V32BIS, 2 = 9600 Hz DP_V32, 3 = 8000 Hz DP_V32.
 * Printed lines are layout independent (hashes of blocks, glue fields); the heap log shows glue / wrapper / RcFixed blocks only (R02_HEAP_LOG_FILTER: the engine's blocks differ in size on x86-64).
 */
#define R02_REAL_RC 1
#define R02_HEAP_LOG_FILTER 1
#include "../r02_common.h"

enum { CV_RUN = 0, CV_ERR, CV_ST4, CV_PB, CV_GB, CV_DBG, CV_CLEAN, CV_BLOCKS, CV_RATES, CV_BOTH, CV_OUT_NZ };
static unsigned long g_nld;
int modem_debug_log_data(struct modem *m, unsigned id, const void *data, int count)
{
  (void)m;
  g_nld++;
  emit(0, "LOGDATA id=%u n=%d h=%016llx", id, count, fnv_bytes(data, count > 0 && count < 320 ? (size_t)count : 0));
  return 0;
}

static struct dp_operations *g_ops;
static const char *slot_name(const void *fn) { return fn == (const void *)dp_wrapper_run ? "dp_wrapper_run" : (fn ? "other" : "NULL"); }
static short g_inA[1024], g_inB[1024], g_outA[1024], g_outB[1024];

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  static const int rates[4] = {9600, 8000, 9600, 8000};
  static const int ids[4] = {DP_V32BIS, DP_V32BIS, DP_V32, DP_V32};
  struct dp *a, *b;
  int it, i, nz = 0, errA = 0, errB = 0, ncall = 0;
  unsigned long ngb0, npb0;
  g_verbose = getenv("R02_VERBOSE") != NULL;
  g_lcg = 0x2545f491u + 1000003u * (unsigned)(sc + 40);
  if (getenv("R02_SEED")) g_lcg ^= (unsigned)atoi(getenv("R02_SEED"));
  if (sc < 0 || sc > 3) die3("no such scenario %d", sc);
  cov_def(CV_RUN, "e2e: op->process calls");
  cov_def(CV_ERR, "e2e: a session reports status 4");
  cov_def(CV_ST4, "e2e: block zeroed after status 4");
  cov_def(CV_PB, "e2e: modem_put_bits calls");
  cov_def(CV_GB, "e2e: modem_get_bits calls");
  cov_def(CV_DBG, "e2e: debug level 1 (no engine messages)");
  cov_def(CV_CLEAN, "e2e: cleaned samples logged");
  cov_def(CV_BLOCKS, "e2e: engine blocks at 40 samples");
  cov_def(CV_BOTH, "e2e: caller and answerer both ran");
  cov_def(CV_OUT_NZ, "e2e: non-zero transmitted audio");
  arena_init();
  g_op_slot_name = slot_name;
  g_nreg_op = 0;
  dp_v32_init();
  if (g_nreg_op != 2) die3("registrations");
  g_ops = g_reg_op[0];
  dsplibs_debug_level = 1; /* the blob's engine prints debug messages at level 2 (V32FP Config, state changes) that the port's engine does not (debug only); the glue's own messages are covered by r03_glue */
  g_param[4] = 14400;
  g_param[5] = 0x20;
  g_bits_give = 12;
  rc_reset();
  reg_clear();
  reg_add(g_inA, sizeof g_inA, 2, "INA");
  reg_add(g_inB, sizeof g_inB, 2, "INB");
  reg_add(g_outA, sizeof g_outA, 2, "OUTA");
  reg_add(g_outB, sizeof g_outB, 2, "OUTB");
  a = g_ops->create(M_(0), (enum DP_ID)ids[sc], 0, rates[sc], 40, g_ops);
  b = g_ops->create(M_(1), (enum DP_ID)ids[sc], 1, rates[sc], 40, g_ops);
  if (!a || !b) die3("create");
  emit(1, "created caller + answerer: status %u %u", a->status, b->status);
  memset(g_outA, 0, sizeof g_outA);
  memset(g_outB, 0, sizeof g_outB);
  ngb0 = g_ngb;
  npb0 = g_npb;
  for (it = 0; it < 6000 && ncall < 6000; it++) {
    int cnt = (it & 3) == 3 ? 48 + (int)rmod(40) : 40 + (int)rmod(8) * 4, ra, rb;
    if (rates[sc] == 9600) cnt = cnt * 6 / 5;
    /* A hears B's previous block, B hears A's previous block */
    memcpy(g_inA, g_outB, sizeof(short) * (size_t)cnt);
    memcpy(g_inB, g_outA, sizeof(short) * (size_t)cnt);
    ra = g_ops->process(a, g_inA, g_outA, cnt);
    rb = g_ops->process(b, g_inB, g_outB, cnt);
    ncall += 2;
    cov_hit(CV_RUN);
    emit(0, "it %d cnt=%d -> %d %d outA=%016llx outB=%016llx st=%u %u", it, cnt, ra, rb, fnv_shorts(g_outA, (size_t)cnt), fnv_shorts(g_outB, (size_t)cnt), a->status, b->status);
    for (i = 0; i < cnt; i++) if (g_outA[i] || g_outB[i]) nz = 1;
    if (ra == 4) errA++;
    if (rb == 4) errB++;
    if (errA > 3 && errB > 3 && it > 2500) break;
  }
  if (errA) cov_hit(CV_ERR);
  if (errA > 1 && errB > 1) cov_hit(CV_ST4);
  if (g_npb > npb0) cov_hit(CV_PB);
  if (g_ngb > ngb0) cov_hit(CV_GB);
  if (g_nld) cov_hit(CV_CLEAN);
  cov_hit(CV_DBG);
  cov_hit(CV_BLOCKS);
  if (it > 100) cov_hit(CV_BOTH);
  if (nz) cov_hit(CV_OUT_NZ);
  emit(1, "session: iterations %d, errors A %d B %d, status %u %u", it, errA, errB, a->status, b->status);
  emit(1, "delete A -> %d", g_ops->delete(a));
  emit(1, "delete B -> %d", g_ops->delete(b));
  trial_end("e2e");
  (void)CV_RATES;
  cov_finish();
  return 0;
}
