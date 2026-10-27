/*
 * R02 differential harness, id r02_wrapper_rc: the same scenarios 2 and 3 of test/core/dp_wrapper.main.main.c (the six rate pairs with odd chunk sizes; poked ring states,
 * asymmetric converters) with the REAL RcFixed behind the wrapper on both sides: the blob's RcFixed_Create / Resample / Delete under the blob's dp_wrapper, src/core/rcfixed_resampler.c
 * under src/core/dp_wrapper.c.  The composition test of the handles (`RcFixed *` in a DpWrapper slot on x86-64) and of the sample data path: the printed lines hash the real
 * resampled samples, the engine blocks and the converter state allocations (sizes mapped to i386).  Create / delete failure injection needs the fake and stays in
 * r02_wrapper (scenarios 0 / 1).
 */
#define R02_REAL_RC 1
#include "dp_wrapper.main.main.c"
