/*
 * test/v8/v8_hobj.h - harness side of src/v8/v8_obj.h (U03).
 *
 * The V.8 harnesses build the engine object as a raw byte buffer in the i386 layout and poke / read it at raw offsets.  That stays so (the flat part of the
 * object has the same offsets on both ABIs).  What changes on x86-64: the 9 POINTER slots (+0x2c +0x114 +0x118 +0x220 +0x224 +0xa58 +0xc48 +0xc4c +0xc50) are real
 * 8-byte pointers in a tail after the 0xee0-byte image (src/v8/v8_obj.h), so
 *   - the buffer must be big enough for the tail and 8-aligned:  `unsigned char obj[V8H_BUFSZ(V8OBJ_SIZE)] V8H_ALIGN;`
 *   - pointer slots are written / read with V8H_PSET(obj, off, ptrvar) / V8H_PGET(ptrvar, obj, off) (a run-time offset is fine);
 *   - the V8_setFilters / detector tables are CFGP handles: V8H_REG(sym, table) makes the harness' private copy of a table the one the port resolves.
 * At -m32 every macro expands to the former harness code (memcpy of the 4-byte slot, no alignment attribute), so the 32-bit harness object code is unchanged.
 */
#ifndef V8_HOBJ_H
#define V8_HOBJ_H
#include <string.h>
#include "../src/v8/v8_obj.h"

#if UINTPTR_MAX == 0xffffffffu

#define V8H_BUFSZ(n) (n)
#define V8H_ALIGN
#define V8H_PSET(obj, off, var) memcpy((obj) + (off), &(var), sizeof(var))
#define V8H_PGET(var, obj, off) memcpy(&(var), (obj) + (off), sizeof(var))
#define V8H_PRAW(raw, obj, off) memcpy(&(raw), (obj) + (off), 4) /* the first 4 bytes of a pointer slot */
#define V8H_REG(sym, table) ((void)0)

#else

#define V8H_BUFSZ(n) ((n) > V8_ALLOC_SIZE ? (n) : V8_ALLOC_SIZE)
#define V8H_ALIGN __attribute__((aligned(16)))
static inline void *v8h_pget(const unsigned char *o, int off)
{
  switch (off) {
  case 0x2c: return V8_PG(void *, o, 0x2c);
  case 0x114: return V8_PG(void *, o, 0x114);
  case 0x118: return V8_PG(void *, o, 0x118);
  case 0x220: return V8_PG(void *, o, 0x220);
  case 0x224: return V8_PG(void *, o, 0x224);
  case 0xa58: return V8_PG(void *, o, 0xa58);
  case 0xc48: return V8_PG(void *, o, 0xc48);
  case 0xc4c: return V8_PG(void *, o, 0xc4c);
  case 0xc50: return V8_PG(void *, o, 0xc50);
  }
  return 0;
}
static inline void v8h_pset(unsigned char *o, int off, const void *v)
{
  switch (off) {
  case 0x2c: V8_PS(void *, o, 0x2c, v); break;
  case 0x114: V8_PS(void *, o, 0x114, v); break;
  case 0x118: V8_PS(void *, o, 0x118, v); break;
  case 0x220: V8_PS(void *, o, 0x220, v); break;
  case 0x224: V8_PS(void *, o, 0x224, v); break;
  case 0xa58: V8_PS(void *, o, 0xa58, v); break;
  case 0xc48: V8_PS(void *, o, 0xc48, v); break;
  case 0xc4c: V8_PS(void *, o, 0xc4c, v); break;
  case 0xc50: V8_PS(void *, o, 0xc50, v); break;
  }
}
#define V8H_PSET(obj, off, var) v8h_pset((unsigned char *)(obj), (off), (var))
#define V8H_PGET(var, obj, off) ((var) = (__typeof__(var))v8h_pget((const unsigned char *)(obj), (off)))
#define V8H_PRAW(raw, obj, off) ((raw) = (unsigned int)(uintptr_t)v8h_pget((const unsigned char *)(obj), (off)))
#define V8H_REG(sym, table) cfgp_set(CFGH_##sym, (table))

#endif
#endif
