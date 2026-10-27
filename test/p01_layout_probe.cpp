/*
 * Layout probe for the P01 differential harness (test/p01_diff_main.c): exports sizeof / offsetof of the REAL port classes
 * (from src/dsp/dsp_int_templates.h) so that the harness' C mirror structs can be checked against the port header on every
 * architecture.  Only sizeof / offsetof: no code from the port is instantiated here.  Compiled with -fno-exceptions -fno-rtti,
 * linked with `gcc` (no libstdc++).
 */
#include <stddef.h>
#include "dsp/dsp_int_templates.h"

#define SCR(T) sizeof(T), offsetof(T, buf_), offsetof(T, head0_), offsetof(T, t1_0_), offsetof(T, end_), offsetof(T, wr_), \
               offsetof(T, t1_), offsetof(T, t2_), offsetof(T, tail_)
#define PAR(T) sizeof(T), offsetof(T, state_), offsetof(T, cap_), offsetof(T, n_)

extern "C" {
extern const unsigned p01_layout_probe[];
extern const unsigned p01_layout_probe_count;
const unsigned p01_layout_probe[] = {
  SCR(P01_ScramblerHH), SCR(P01_ScramblerIH), SCR(P01_ScramblerHI), SCR(P01_DescramblerHI), SCR(P01_DescramblerII),
  PAR(P01_PDE), PAR(P01_PDD),
  sizeof(P01_QueueF), offsetof(P01_QueueF, buf_), offsetof(P01_QueueF, last_), offsetof(P01_QueueF, rd_), offsetof(P01_QueueF, wr_),
  offsetof(P01_QueueF, size_),
  sizeof(P01_SDE_H), offsetof(P01_SDE_H, state_), sizeof(P01_SDD_H), offsetof(P01_SDD_H, state_), sizeof(P01_SDD_I), offsetof(P01_SDD_I, state_),
};
const unsigned p01_layout_probe_count = sizeof(p01_layout_probe) / sizeof(p01_layout_probe[0]);
}
