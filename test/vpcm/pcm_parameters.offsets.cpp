/*
 * Compile-time pin (i386 only: the asserts vanish on other targets) of the i386 offsets of the members of V90Parameters / V92Parameters that other modules
 * use: the V90Resampler gains and history settings, the V90Phase2Info / V92Phase2Info fields, the V.34 cfg words.  The resampler and the Phase2Info classes
 * read the members by name (the offsets were wrong on a 64-bit build, where the pointer member `params` is 8 bytes); the table keeps the 32-bit layout
 * the blob-facing harnesses rely on.  test/vpcm/pcm_parameters.offsets_check.py reports the byte offsets src/ still uses (none expected).  The native layout is
 * checked by test/vpcm/pcm_parameters.native64.main.cpp.
 * Compiled by test/vpcm/pcm_parameters.main.sh.
 */
#include "vpcm/pcm_parameters.h"

#define A90(member, off, who) DSP_ASSERT_LAYOUT_I386(offsetof(V90Parameters, member) == (off), "V90Parameters::" #member " (" who ")")
#define A92(member, off, who) DSP_ASSERT_LAYOUT_I386(offsetof(V92Parameters, member) == (off), "V92Parameters::" #member " (" who ")")

A90(bllInitialK1, 0x88, "P05 V90Resampler gain pair");
A90(bllInitialK2, 0x8c, "P05 V90Resampler gain pair");
A90(bllFastK1, 0x90, "P05 V90Resampler gain pair");
A90(bllFastK2, 0x94, "P05 V90Resampler gain pair");
A90(bllMediumK1, 0x98, "P05 V90Resampler gain pair");
A90(bllMediumK2, 0x9c, "P05 V90Resampler gain pair");
A90(bllSlowK1, 0xa0, "P05 V90Resampler gain pair");
A90(bllSlowK2, 0xa4, "P05 V90Resampler gain pair");
A90(bllSlow2K1, 0xa8, "P05 V90Resampler gain pair");
A90(bllSlow2K2, 0xac, "P05 V90Resampler gain pair");
A90(bllDilK1, 0xb0, "P05 V90Resampler gain pair");
A90(bllDilK2, 0xb4, "P05 V90Resampler gain pair");
A90(bllTrn2InitialK1, 0xb8, "P05 V90Resampler gain pair");
A90(bllTrn2InitialK2, 0xbc, "P05 V90Resampler gain pair");
A90(bllTrn2K1, 0xc0, "P05 V90Resampler gain pair");
A90(bllTrn2K2, 0xc4, "P05 V90Resampler gain pair");
A90(bllSteadyStateK1, 0xc8, "P05 V90Resampler gain pair");
A90(bllSteadyStateK2, 0xcc, "P05 V90Resampler gain pair");
A90(bllPreAnspcmK1, 0xd0, "P05 V90Resampler gain pair");
A90(bllPreAnspcmK2, 0xd4, "P05 V90Resampler gain pair");
A90(bllTrn1QcInitialK1, 0xd8, "P05 V90Resampler gain pair");
A90(bllTrn1QcInitialK2, 0xdc, "P05 V90Resampler gain pair");
A90(bllTrn1QcFastK1, 0xe0, "P05 V90Resampler gain pair");
A90(bllTrn1QcFastK2, 0xe4, "P05 V90Resampler gain pair");
A90(bllTrn1QcMediumK1, 0xe8, "P05 V90Resampler gain pair");
A90(bllTrn1QcMediumK2, 0xec, "P05 V90Resampler gain pair");
A90(bllTrn1QcSlowK1, 0xf0, "P05 V90Resampler gain pair");
A90(bllTrn1QcSlowK2, 0xf4, "P05 V90Resampler gain pair");
A90(timingHistoryEvaluationEnabled, 0x160, "P05 V90Resampler");
A90(timingHistoryEvaluationBufferLength, 0x164, "P05 V90Resampler");
A90(timingHistoryEvaluationPeriod, 0x168, "P05 V90Resampler");
A90(phase2InfoAOrMu, 0x18, "P06 V90Phase2Info");
A90(phase2InfoRtd, 0x1c, "P06 V90Phase2Info");
A90(phase2InfoUinfo, 0x20, "P06 V90Phase2Info");
A90(phase2InfoMaxTxPower, 0x24, "P06 V90Phase2Info");
A90(phase2InfoTxPowerMeasurePoint, 0x28, "P06 V90Phase2Info");
A90(highLevelTxActive, 0x4f4, "ported V.34 code (chkForceBaudRate: cfg+0x4f4)");
A90(sensitiveIspDetected, 0x4f8, "ported V.34 code (cfg+0x4f8)");
A90(maxTxRateIndexForSensitiveIsp, 0x4fc, "ported V.34 code (cfg+0x4fc)");

A92(v92Phase2InfoAOrMu, 0x8, "P06 V92Phase2Info");
A92(v92Phase2InfoRtd, 0xc, "P06 V92Phase2Info");
A92(v92Phase2InfoUinfo, 0x10, "P06 V92Phase2Info");
A92(v92Phase2InfoMaxTxPower, 0x14, "P06 V92Phase2Info");
A92(v92Phase2InfoTxPowerMeasurePoint, 0x18, "P06 V92Phase2Info");
A92(v92NofFilterSections, 0x5c, "P06 V92Phase2Info");
A92(v92MaxTotalNofCoeffs, 0x60, "P06 V92Phase2Info");
A92(v92MaxNofCoeffsInEachSection, 0x64, "P06 V92Phase2Info");
