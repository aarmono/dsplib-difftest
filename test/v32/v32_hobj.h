/*
 * test/v32/v32_hobj.h - harness side of src/v32/v32.h (U05).
 *
 * The V.32 harnesses address the wrapper / hdx / engine objects of a V32FP_create()d object by raw i386 offsets (`S16(HDX + 0x76)`): the flat image part of those
 * objects is unchanged on x86-64 (only the address slots, the tone / MTD / AGC objects embedded in them and the engine's FPM objects moved: v32.h), so
 * every plain field access stays; what changes is
 *   - an object is a `char *` (int at -m32, char * on x86-64), `(char *)(p)` converts the pointer V32FP_create returns;
 *   - an address slot is read with V32W_PG / V32H_PG / V32E_PG (wrapper / hdx / engine) instead of `S32(OBJ + 0x68)`;
 *   - the tone / MTD / AGC objects reached through a slot are typed (FpmTone / FpmMtd / FpmAgc, fpm.h) and their 16-bit fields are read with H_TONE_S / H_MTD_S
 *     (-m32: the raw offset, x86-64: the member), table pointers with H_PTR.
 * At -m32 every macro expands to the former raw access (the object code of the harness stays the same).
 */
#ifndef V32_HOBJ_H
#define V32_HOBJ_H
#include <stdint.h>
#include "../src/fpm/fpm.h"
#include "../src/v32/v32.h"

#define V32H_TONE0(h) V32H_PG(FpmTone *, h, 0x2c)
#define V32H_MTD(h) V32H_PG(FpmMtd *, h, 0x38)
#if UINTPTR_MAX == 0xffffffffu
#define H_TONE_S(t, off) (*(short *)(intptr_t)((intptr_t)(t) + (off)))
#define H_TONE_US(t, off) (*(unsigned short *)(intptr_t)((intptr_t)(t) + (off)))
#define H_MTD_S(m, off) (*(short *)(intptr_t)((intptr_t)(m) + (off)))
#define H_MTD_COEF_IS(m, sym) (*(int *)(intptr_t)(m) == (int)(intptr_t)(sym))
/* the first short of the table an AGC alpha / beta pointer points at */
#define H_AGC_FIRST(e, rawoff, member) (*(short *)(intptr_t)(*(int *)(intptr_t)((intptr_t)(e) + (rawoff))))
#else
#define H_TONE_S(t, off) ((t)->r0.s[(off) / 2])
#define H_TONE_US(t, off) ((unsigned short)(t)->r0.s[(off) / 2])
static inline short h_mtd_s16(FpmMtd *m, int off)
{
  switch (off) {
  case 0x10: return m->dc[0];
  case 0x12: return m->dc[1];
  case 0x14: return m->diffEnergy;
  default: return m->inEnergy;
  }
}
#define H_MTD_S(m, off) h_mtd_s16((m), (off))
#define H_MTD_COEF_IS(m, sym) ((m)->coef == (sym))
#define H_AGC_FIRST(e, rawoff, member) (*V32E_AGC(e)->member)
#endif

#endif
