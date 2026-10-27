/*
 * X4: the V.90 / V.92 parameter blocks at the NATIVE layout.  V90Parameters and V92Parameters start with a pointer (4 bytes i386, 8 bytes x86-64), so every
 * word after it is at a different offset on a 64-bit build.  The harnesses of the blob tests build their own i386 image of the blocks and cannot see that;
 * this test fills REAL objects through their named members, with a distinct value in every word that the code under test reads, and checks:
 *   - V90Phase2Info / V92Phase2Info (constructor and setToDefault()): every field comes from its own member (a read one word off picks up the neighbour);
 *   - V90Resampler: the history length (timingHistoryEvaluationBufferLength) is the allocation size, the BLL gain pairs of every state come from the
 *     matching K1 / K2 members, the history period and enable flag steer resample(); with the history disabled and a zero length nothing is written.
 * Built for x86-64 only by test/vpcm/pcm_parameters.native64.sh (a 32-bit build has no libstdc++ to link); the same source is compiled with -m32 -c as a
 * syntax / static-assert check, where the i386 offsets of the members are asserted too.  Exit 0 and "MATCH" iff every check holds.
 */
#include <stdio.h>
#include <stdint.h>

/* no C++ standard library headers: a 32-bit libstdc++ is not available (the -m32 build only compiles this file) */
extern "C" void* calloc(__SIZE_TYPE__, __SIZE_TYPE__);
extern "C" void free(void*);
struct X4Place
{
};
inline void* operator new(__SIZE_TYPE__, void* p, X4Place) { return p; }
#include "vpcm/pcm_parameters.h"
#include "vpcm/pcm_leaf_classes.h"
#include "dsp/dsp_resampler.h"
#include "core/host_interface.h"

#ifdef __i386__
static_assert(offsetof(V90Parameters, phase2InfoAOrMu) == 0x18 && offsetof(V90Parameters, timingHistoryEvaluationBufferLength) == 0x164, "V90Parameters i386");
static_assert(offsetof(V92Parameters, v92Phase2InfoAOrMu) == 0x8 && offsetof(V92Parameters, v92MaxNofCoeffsInEachSection) == 0x64, "V92Parameters i386");
static_assert(offsetof(struct dsp_info, clock_deviation) == 4, "dsp_info i386");
#else
static_assert(offsetof(V90Parameters, phase2InfoAOrMu) == 0x1c && offsetof(V90Parameters, timingHistoryEvaluationBufferLength) == 0x168, "V90Parameters x86-64");
static_assert(offsetof(V92Parameters, v92Phase2InfoAOrMu) == 0xc && offsetof(V92Parameters, v92MaxNofCoeffsInEachSection) == 0x68, "V92Parameters x86-64");
static_assert(offsetof(struct dsp_info, clock_deviation) == 8, "dsp_info x86-64");
#endif

/* the library's debug print: silent here */
extern "C" void edprintf(const char*, ...) {}

static int g_fail;
#define CHECK(c)                                                                                                                                             \
  do                                                                                                                                                         \
  {                                                                                                                                                          \
    if (!(c))                                                                                                                                                \
    {                                                                                                                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                                                                                                    \
      g_fail++;                                                                                                                                              \
    }                                                                                                                                                        \
  } while (0)
#define CHECK_EQ(a, b)                                                                                                                                       \
  do                                                                                                                                                         \
  {                                                                                                                                                          \
    long long va_ = (long long)(a), vb_ = (long long)(b);                                                                                                    \
    if (va_ != vb_)                                                                                                                                          \
    {                                                                                                                                                        \
      printf("FAIL %s:%d: %s == %lld, expected %s == %lld\n", __FILE__, __LINE__, #a, va_, #b, vb_);                                                         \
      g_fail++;                                                                                                                                              \
    }                                                                                                                                                        \
  } while (0)

/* a parameter block: zero, then every word the code under test reads set through its member */
template <class T> static T* newBlock()
{
  return (T*)calloc(1, sizeof(T));
}

static void testPhase2V90()
{
  V90Parameters* p = newBlock<V90Parameters>();
  p->phase2InfoAOrMu = 1;
  p->phase2InfoRtd = 0x1234567;
  p->phase2InfoUinfo = 0x2ab; /* stored in a byte: 0xab */
  p->phase2InfoMaxTxPower = 0x3cd;
  p->phase2InfoTxPowerMeasurePoint = 1;
  V90Phase2Info info(p);
  CHECK_EQ(info.pcmType_, 1);
  CHECK_EQ(info.rtd_, 0x1234567);
  CHECK_EQ(info.uinfo_, 0xab);
  CHECK_EQ(info.maxTx_, 0xcd);
  CHECK_EQ(info.txPoint_, 1);
  /* setToDefault() reloads from the same block */
  p->phase2InfoAOrMu = 0;
  p->phase2InfoRtd = 77;
  p->phase2InfoUinfo = 5;
  p->phase2InfoMaxTxPower = 6;
  p->phase2InfoTxPowerMeasurePoint = 0;
  info.setToDefault();
  CHECK_EQ(info.pcmType_, 0);
  CHECK_EQ(info.rtd_, 77);
  CHECK_EQ(info.uinfo_, 5);
  CHECK_EQ(info.maxTx_, 6);
  CHECK_EQ(info.txPoint_, 0);
  free(p);
}

static void testPhase2V92()
{
  V92Parameters* p = newBlock<V92Parameters>();
  p->v92Phase2InfoAOrMu = 1;
  p->v92Phase2InfoRtd = 0x7654321;
  p->v92Phase2InfoUinfo = 0x1ab;
  p->v92Phase2InfoMaxTxPower = 0x2cd;
  p->v92Phase2InfoTxPowerMeasurePoint = 1;
  p->v92NofFilterSections = 0x1e1;
  p->v92MaxTotalNofCoeffs = 0x2e2;
  p->v92MaxNofCoeffsInEachSection = 0x3e3;
  V92Phase2Info info(p);
  CHECK_EQ(info.pcmType_, 1);
  CHECK_EQ(info.rtd_, 0x7654321);
  CHECK_EQ(info.uinfo_, 0xab);
  CHECK_EQ(info.maxTx_, 0xcd);
  CHECK_EQ(info.txPoint_, 1);
  CHECK_EQ(info.b14_, 0xe1);
  CHECK_EQ(info.b15_, 0xe2);
  CHECK_EQ(info.b16_, 0xe3);
  CHECK_EQ(info.v92CapLocal_, 1);
  p->v92Phase2InfoAOrMu = 0;
  p->v92Phase2InfoRtd = 78;
  p->v92Phase2InfoUinfo = 7;
  p->v92Phase2InfoMaxTxPower = 8;
  p->v92Phase2InfoTxPowerMeasurePoint = 0;
  p->v92NofFilterSections = 9;
  p->v92MaxTotalNofCoeffs = 10;
  p->v92MaxNofCoeffsInEachSection = 11;
  info.setToDefault();
  CHECK_EQ(info.pcmType_, 0);
  CHECK_EQ(info.rtd_, 78);
  CHECK_EQ(info.uinfo_, 7);
  CHECK_EQ(info.maxTx_, 8);
  CHECK_EQ(info.txPoint_, 0);
  CHECK_EQ(info.b14_, 9);
  CHECK_EQ(info.b15_, 10);
  CHECK_EQ(info.b16_, 11);
  free(p);
}

struct GainPair
{
  V90BllState state;
  float V90Parameters::*k1;
  float V90Parameters::*k2;
};
#define PAIR(S, N) {S, &V90Parameters::N##K1, &V90Parameters::N##K2}
static const GainPair kPairs[] = {PAIR(V90BLL_INITIAL, bllInitial),
                                  PAIR(V90BLL_FAST, bllFast),
                                  PAIR(V90BLL_MEDIUM, bllMedium),
                                  PAIR(V90BLL_SLOW, bllSlow),
                                  PAIR(V90BLL_SLOW2, bllSlow2),
                                  PAIR(V90BLL_DIL, bllDil),
                                  PAIR(V90BLL_TRN2_INITIAL, bllTrn2Initial),
                                  PAIR(V90BLL_TRN2, bllTrn2),
                                  PAIR(V90BLL_STEADY_STATE, bllSteadyState),
                                  PAIR(V90BLL_PRE_ANSPCM, bllPreAnspcm),
                                  PAIR(V90BLL_TRN1_QC_INITIAL, bllTrn1QcInitial),
                                  PAIR(V90BLL_TRN1_QC_FAST, bllTrn1QcFast),
                                  PAIR(V90BLL_TRN1_QC_MEDIUM, bllTrn1QcMedium),
                                  PAIR(V90BLL_TRN1_QC_SLOW, bllTrn1QcSlow)};

static void testResampler()
{
  float coef[4] = {0.25f, 0.5f, 0.5f, 0.25f};
  float in[16] = {0};
  float out[64];
  unsigned nOut;
  V90Parameters* p = newBlock<V90Parameters>();
  /* a distinct value in every gain word */
  float v = 1.0f;
  for (unsigned i = 0; i < sizeof kPairs / sizeof kPairs[0]; i++)
  {
    p->*kPairs[i].k1 = v;
    p->*kPairs[i].k2 = v + 0.5f;
    v += 1.0f;
  }
  p->timingHistoryEvaluationEnabled = 1;
  p->timingHistoryEvaluationBufferLength = 7;
  p->timingHistoryEvaluationPeriod = 3;

  alignas(16) static unsigned char rbuf[sizeof(V90Resampler)];
  V90Resampler* r = new (rbuf, X4Place()) V90Resampler(1, 1.0f, 4, coef, p, 0.0f, 0);
  CHECK_EQ(r->tHistLen_, 7); /* the buffer length, not the enable flag or the period */
  CHECK(r->tHist_ != 0);
  for (unsigned i = 0; i < sizeof kPairs / sizeof kPairs[0]; i++)
  {
    r->setBllState(V90BLL_FROZEN, 1); /* so that every state is a change */
    r->setBllState(kPairs[i].state, 1);
    CHECK(r->gainA_ == p->*kPairs[i].k1);
    CHECK(r->gainB_ == p->*kPairs[i].k2);
  }
  /* the history: one entry per `period` input samples, wrapping at the buffer length */
  r->tHistIdx_ = 0;
  r->tCount_ = 0;
  for (int call = 0; call < 3; call++)
  {
    in[8] = 0.0f;
    r->resample(in, 1, out, nOut);
    CHECK_EQ(r->tHistIdx_, call == 2 ? 1 : 0);
  }
  CHECK_EQ(r->tCount_, 0);
  r->~V90Resampler();

  /* history disabled with a zero length: the block has no room and must not be written (the period read as length would be wrong too) */
  p->timingHistoryEvaluationEnabled = 0;
  p->timingHistoryEvaluationBufferLength = 0;
  p->timingHistoryEvaluationPeriod = 1;
  r = new (rbuf, X4Place()) V90Resampler(1, 1.0f, 4, coef, p, 0.0f, 0);
  CHECK_EQ(r->tHistLen_, 0);
  r->resample(in, 1, out, nOut);
  CHECK_EQ(r->tHistIdx_, 0);
  CHECK_EQ(r->tCount_, 0);
  r->~V90Resampler();
  free(p);
}

int main()
{
  testPhase2V90();
  testPhase2V92();
  testResampler();
  if (g_fail)
  {
    printf("X4 FAIL: %d check(s) failed\n", g_fail);
    return 1;
  }
  printf("MATCH: X4 the V90 / V92 parameter blocks are read through their named members (native layout)\n");
  return 0;
}
