/*
 * Compile-time cross-check of the byte offsets by which port chunk P06 (V90ConstellationPower::calcModulusParameters / getConstellationInfo / getPower) still reads a
 * V90MappingParams block: every one of them is the i386 offset of a NAMED member of the struct of v92_cp.h.  (i386 only: the asserts vanish elsewhere.)
 * When the later cleanup replaces `pw(p, 0x620)` / `mpRadix(p, k)` by `p->w620_` / `p->cnt_[k]`, this table is the translation.
 * test/vpcm/v92_cp.offsets_check.py additionally verifies that every V90MappingParams offset used in src/vpcm/pcm_leaf_classes.cpp appears below.
 * Compiled by test/vpcm/v92_cp.sh.
 */
#include "vpcm/v92_cp.h"

#define AMP(member, off, who) DSP_ASSERT_LAYOUT_I386(offsetof(V90MappingParams, member) == (off), "V90MappingParams::" #member " (" who ")")

AMP(kBits_, 0x0, "P06 calcModulusParameters: pw(p, 0); also the TRN2d D of P09 / P10 callers");
AMP(listA_, 0x4, "P06 mpTable: 0x4 + 0x80 k (point != codec output)");
AMP(listB_, 0x304, "P06 mpTable: 0x304 + 0x80 k (point == codec output)");
AMP(cnt_, 0x604, "P06 mpRadix: 0x604 + 4 k");
AMP(sr_, 0x620, "P06 calcModulusParameters: pw(p, 0x620)");
