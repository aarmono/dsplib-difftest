/* R24 (id r24_vtb): the decision callbacks with the REAL Viterbi decoder and the typed front end; scenarios 4 .. 5 (see r24_common.h).
 * Port builds link src/v32/v32_datapath.c (the ported VTB_decoder); the symbols that file references but this test never reaches are abort stubs below (the blob build has the real ones). */
#define R24_REAL 1
#include "r24_common.h"
#ifndef TEST_BLOB
static void unreachable_stub(const char *n)
{
  fprintf(stderr, "r24_vtb: unexpected call of %s\n", n);
  abort();
}
void FPM_AGC_agc(void) { unreachable_stub("FPM_AGC_agc"); }
void FPM_ECC_cancel(void) { unreachable_stub("FPM_ECC_cancel"); }
void FPM_ECC_init(void) { unreachable_stub("FPM_ECC_init"); }
void FPM_FSE_receive(void) { unreachable_stub("FPM_FSE_receive"); }
void FPM_MRF_filter(void) { unreachable_stub("FPM_MRF_filter"); }
void FPM_PPS_filter(void) { unreachable_stub("FPM_PPS_filter"); }
void FPM_rms(void) { unreachable_stub("FPM_rms"); }
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
const int V32_SYMBOL_LEN = 0;
#endif
int main(int argc, char **argv) { return r24_main(argc, argv); }
