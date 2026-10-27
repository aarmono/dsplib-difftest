/*
 * R31 differential harness (id r31_dp): prop_dp_init / prop_dp_exit of dsplibs.o vs src/core/dp_registry.c.
 * The harness owns EVERY callee: the 14 dp_*_init / dp_*_exit functions are recorders (the blob's copies are weakened by objcopy in
 * test/build_and_diff_r31_dp.sh).  A recorder logs its own id, returns a scripted value (the code under test must ignore it) and clobbers the
 * caller-saved registers.
 * Scenarios (each its own process):
 *   0  prop_dp_init  : 300 calls, recorder return values 0 / special values / random; prints the call order and the result
 *   1  prop_dp_exit  : the same for exit
 *   2  alternating init / exit sequences over a differently initialised stack
 *   3  recorders that call back into prop_dp_init / prop_dp_exit at a chosen position (re-entrancy: no state is kept, so the order is nested)
 * Exit 2: a coverage counter is zero (a recorder never called, a position of the sequence never seen); exit 3: harness violation.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

extern int prop_dp_init(void);
extern int prop_dp_exit(void);

enum { C_CALL, C_B103, C_V22, C_V23, C_V32, C_V8, C_VPCM, NMOD };
static const char *const modn[NMOD] = {"call", "b103", "v22", "v23", "v32", "v8", "vpcm"};

static unsigned long long prng = 0x9e3779b97f4a7c15ull;
static unsigned rnd(void)
{
  prng = prng * 6364136223846793005ull + 1442695040888963407ull;
  return (unsigned)(prng >> 33);
}

static int log_[64], nlog;
static int cnt_init[NMOD], cnt_exit[NMOD];
static int pos_seen[2][NMOD][NMOD];
static int retscript[8];
static int nret;
static int reent_at = -1, reent_depth, reent_kind;
static int cov_reent;

static int next_ret(void)
{
  int v = retscript[nret & 7];
  nret++;
  return v;
}

static void clobber(void)
{
#if defined(__i386__) || defined(__x86_64__)
  __asm__ volatile("" ::: "eax", "ecx", "edx", "memory");
#endif
}

static void maybe_reenter(int pos)
{
  if (reent_at == pos && reent_depth < 1) {
    reent_depth++;
    cov_reent++;
    if (reent_kind) prop_dp_exit(); else prop_dp_init();
    reent_depth--;
  }
}

#define REC_INIT(name, id) int name(void) { log_[nlog++] = id; cnt_init[id]++; clobber(); maybe_reenter(id); return next_ret(); }
#define REC_EXIT(name, id) void name(void) { log_[nlog++] = 100 + id; cnt_exit[id]++; clobber(); maybe_reenter(id); (void)next_ret(); }
REC_INIT(dp_call_init, C_CALL)
REC_INIT(dp_b103_init, C_B103)
REC_INIT(dp_v22_init, C_V22)
REC_INIT(dp_v23_init, C_V23)
REC_INIT(dp_v32_init, C_V32)
REC_INIT(dp_v8_init, C_V8)
REC_INIT(dp_vpcm_init, C_VPCM)
REC_EXIT(dp_call_exit, C_CALL)
REC_EXIT(dp_b103_exit, C_B103)
REC_EXIT(dp_v22_exit, C_V22)
REC_EXIT(dp_v23_exit, C_V23)
REC_EXIT(dp_v32_exit, C_V32)
REC_EXIT(dp_v8_exit, C_V8)
REC_EXIT(dp_vpcm_exit, C_VPCM)

static void fill_script(int kind)
{
  static const int sp[] = {0, 1, -1, INT_MIN, INT_MAX, 0x7f, -0x80};
  int i;
  for (i = 0; i < 8; i++) retscript[i] = kind == 0 ? 0 : kind == 1 ? sp[rnd() % 7] : (int)rnd();
}

static void dump(const char *what, int r)
{
  int i;
  printf("%s ->", what);
  for (i = 0; i < nlog; i++) {
    int e = log_[i] >= 100;
    int id = log_[i] % 100;
    printf(" %s_%s", modn[id], e ? "exit" : "init");
  }
  printf("  ret=%d\n", r);
}

static void run(int ex, const char *tag)
{
  int r, i;
  nlog = 0;
  nret = 0;
  r = ex ? prop_dp_exit() : prop_dp_init();
  dump(tag, r);
  for (i = 0; i < nlog && i < NMOD; i++) pos_seen[ex][i][log_[i] % 100]++;
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0, i, j;
  char tag[64];
  switch (sc) {
  case 0:
  case 1:
    for (i = 0; i < 300; i++) {
      fill_script(i % 3);
      snprintf(tag, sizeof tag, "%s %d", sc ? "exit" : "init", i);
      run(sc, tag);
    }
    break;
  case 2:
    for (i = 0; i < 200; i++) {
      volatile char pad[64];
      memset((void *)pad, (int)rnd(), sizeof pad);
      fill_script(1 + (i & 1));
      snprintf(tag, sizeof tag, "seq %d init", i);
      run(0, tag);
      if (i % 3) {
        snprintf(tag, sizeof tag, "seq %d exit", i);
        run(1, tag);
      }
      if (i % 5 == 0) {
        snprintf(tag, sizeof tag, "seq %d init again", i);
        run(0, tag);
      }
    }
    break;
  case 3:
    for (i = 0; i < NMOD; i++)
      for (j = 0; j < 2; j++) {
        reent_at = i;
        reent_kind = j;
        fill_script(2);
        snprintf(tag, sizeof tag, "reenter@%s kind %d init", modn[i], j);
        run(0, tag);
        snprintf(tag, sizeof tag, "reenter@%s kind %d exit", modn[i], j);
        run(1, tag);
      }
    break;
  default:
    fprintf(stderr, "bad scenario %d\n", sc);
    return 3;
  }
  if (nlog > 62) return 3;
  for (i = 0; i < NMOD; i++) {
    if (sc == 0 && !cnt_init[i]) { fprintf(stderr, "coverage: init %s never called\n", modn[i]); return 2; }
    if (sc == 1 && !cnt_exit[i]) { fprintf(stderr, "coverage: exit %s never called\n", modn[i]); return 2; }
    if (sc >= 2 && (!cnt_init[i] || !cnt_exit[i])) { fprintf(stderr, "coverage: %s\n", modn[i]); return 2; }
  }
  if (sc == 0 || sc == 1)
    for (i = 0; i < NMOD; i++)
      for (j = 0; j < NMOD; j++)
        if ((i == j) != (pos_seen[sc][i][j] > 0)) {
          fprintf(stderr, "coverage: position %d module %d count %d\n", i, j, pos_seen[sc][i][j]);
          return 2;
        }
  if (sc == 3 && cov_reent != 2 * NMOD * 2) { fprintf(stderr, "coverage: reentered %d times\n", cov_reent); return 2; }
  return 0;
}
