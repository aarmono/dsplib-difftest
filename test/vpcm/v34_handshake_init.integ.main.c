/*
 * INTEGRATED scenario for V.34 "tier 9a" (src/vpcm/v34_handshake_init.c): v34handshakinit with its REAL callees.
 *
 * The interposed-callee harness (test/vpcm/v34_handshake_init.main.c) proves every argument, every store, the call order and
 * the debug output of v34handshakinit against recorders.  This file closes the other half: the blob's own v34handshakinit
 * + the blob's own v34modeminit / rxtiminginit / preinitdigital / detectorinit / V34SetINFO0dBits / VPcmV34SetMohMessageBits
 * versus the port's v34handshakinit + the ALREADY VERIFIED ports of those callees, on objects prepared with the tier-4b
 * fixture (seeded fake echo canceller, pointer-slot normalisation, table registry, edprintf buffer learning), which this
 * file pulls in by #including the tier-4b harness as a library (its main() is renamed).
 *
 * One scenario (argv[1] ignored, kept for the build script's convention): 24,000 calls, mode 0..4 and > 4, levels
 * {0, 1, 2, 3, 0x80000000, 0xffffffff}, a level-flip hook after the k-th print on 1/4 of the level > 1 calls; per call the
 * hash of the object DIFF (slots normalised) and every debug line go to stdout.  No independent model here (the callees
 * are real); the state words are kept in 0..86 (the StateName oracle).  Coverage counters exit 2 on a zero.
 */
#define main t4b_main
#include "v34_modem_init.main.c"
#undef main

extern int v34handshakinit(void *obj, unsigned int mode);
extern const short V34TimingPrefilterCoeff[40];
extern const short V34TimingHPFilterCoeff[40];
static const unsigned int k_extra[] = {0xaa70, 0x620, 0x624, 0x35b0};

static void scen_integrated(void) {
  logger_t lg = {"integrated", FNV0, 0, 1500};
  long mode_n[6] = {0}, prints_hist[24] = {0}, hook_fired = 0, lvl_hi = 0;
  int c;
  g_obj = xalloc(OBJSZ);
  g_snap = xalloc(OBJSZ);
  ed_learn();
  for (c = 0; c < 24000; c++) {
    static const unsigned int k_modes[] = {0, 1, 2, 3, 4, 5, 0x100u, 0x104u, 0xffffffffu, 0x80000000u};
    unsigned int mode = (c % 10 < 8) ? (unsigned int)(c % 5) : k_modes[lcg_mod(sizeof k_modes / sizeof k_modes[0])];
    unsigned int lvl = k_levels[lcg_mod(6)];
    unsigned int h;
    long before = g_total_prints;
    static const unsigned short k_so[3] = {0x3596, 0x3594, 0x3592};
    static const int k_t[] = {0x36, 0x2b, 0x29, 0x4a, 0x23, 0x12, 4, 0x4f};
    int i, np;
    seed_echo_obj(g_obj);
    {
      size_t k;
      for (k = 0; k < sizeof k_extra / sizeof k_extra[0]; k++) U32(g_obj, k_extra[k]) = 0x0d000000u | (lcg_next() & 0xffffffu);
    }
    for (i = 0; i < 3; i++) U16(g_obj, k_so[i]) = (uint16_t)(lcg_mod(100) < 50 ? (unsigned int)k_t[lcg_mod(8)] : lcg_mod(87));
    switch (lcg_mod(6)) {
      case 0: U16(g_obj, 0x359c) = 0x65; break;
      case 1: U16(g_obj, 0x359c) = 0x66; break;
      case 2: U16(g_obj, 0x359c) = 0x64; break;
      default: break;
    }
    if (lcg_mod(3) == 0) U32(g_obj, 0x248) = 1u + lcg_mod(0x20000), U32(g_obj, 0x238) = U32(g_obj, 0x248) + lcg_mod(0x18000);
    U32(g_obj, 0xabf0) = lcg_mod(8);
    if (lcg_mod(4) == 0) U8(g_obj, 0xac17) = 0;
    memcpy(g_snap, g_obj, OBJSZ);
    ext_clear();
    if (lvl > 1 && lcg_mod(4) == 0) {
      static const unsigned int to[3] = {0u, 1u, 0x80000000u};
      g_flip_after = lcg_range(0, 6);
      g_flip_to = to[lcg_mod(3)];
    } else {
      g_flip_after = -1;
    }
    g_nprint = 0;
    printf("-- call %d mode=%u level=%u flip=%d\n", c, mode, lvl, g_flip_after);
    dsplibs_debug_level = lvl;
    if (v34handshakinit(g_obj, mode) != 0) die("v34handshakinit returned non-zero");
    dsplibs_debug_level = 0;
    h = diff_obj(g_obj, g_snap);
    {
      unsigned int hs = ed_state_hash();
      h = fnv(h, &hs, 4);
    }
    np = (int)(g_total_prints - before);
    if (np >= 0 && np < 24) prints_hist[np]++;
    if (g_flip_after >= 0 && np > g_flip_after) hook_fired++;
    g_flip_after = -1;
    if (lvl > 1) lvl_hi++;
    mode_n[mode > 4 ? 5 : mode]++;
    log_call2(&lg, h, (long)mode, (long)lvl, (long)np);
  }
  log_done(&lg);
  {
    static const char *nm[6] = {"mode 0", "mode 1", "mode 2", "mode 3", "mode 4", "mode > 4"};
    for (c = 0; c < 6; c++) need(nm[c], mode_n[c]);
  }
  need("calls at level > 1", lvl_hi);
  need("hook fired", hook_fired);
  need("calls with >= 3 prints", prints_hist[3] + prints_hist[4] + prints_hist[5] + prints_hist[6]);
  need("calls with 0 prints", prints_hist[0]);
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  g_verbose = getenv("T9A_VERBOSE") ? atoi(getenv("T9A_VERBOSE")) : 0;
  tables_init();
  add_tbl("V34TimingPrefilterCoeff", V34TimingPrefilterCoeff, 80); /* obj+0x620 / 0x624 (V34TimingFiltersInit via rxtiminginit) */
  add_tbl("V34TimingHPFilterCoeff", V34TimingHPFilterCoeff, 80);
  slots_init();
  { /* slots not in the tier-4b list: 0xaa70 (INFO block pointer, written by v34handshakinit), 0x620 / 0x624 / 0x35b0 (rxtiminginit, tier-5b list) */
    size_t k;
    for (k = 0; k < sizeof k_extra / sizeof k_extra[0]; k++) g_isslot[k_extra[k] / 4] = 1;
  }
  scen_integrated();
  fflush(stdout);
  return g_fail ? 2 : 0;
}
