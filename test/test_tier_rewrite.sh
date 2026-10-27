#!/bin/sh
# T01/T02: self-test of tools/tier_rewrite.py.  Run by hand (not a blob-vs-port job, so not in tools/run_changed.sh); ~30 s; prints TIER_REWRITE PASS / FAIL.
#   1. the generated files are current: src/v34/v34_object_view.h (header --check),
#      src/v34/v34_subobjects.h (slice --check), the external-class headers (v34_flomodem_view.h v34_k56flex_view.h; RS6: no v34_modem_params_view.h any more, see src/core/dp_params.h)
#   2. all 16 tier files (not the *_tables.c) are FIXED POINTS of `rewrite` (everything resolvable is converted: 0 unresolved literal-obj sites outside the
#      functions listed in test/v34/v34_base_exceptions.txt) and `rewrite --mode all` of each compiles to IDENTICAL objdump -dr at -m32 -O0, -O2, pinned -O2 -fno-strict-aliasing
#   3. the sub-object modes (--struct rx / mod / det / rate / info / ctx ...) rewrite temporary copies of tier7a / 3c / 5c / 8a / 2 and the result is IDENTICAL at -O2 and pinned
#   4. synthetic cases (comments / strings / computed offsets / other bases / decimal + upper-case hex / tier2-style accessors / interior offsets / opaque region offsets /
#      function exceptions / second macro shape)
cd "$(dirname "$0")/.." || exit 1
T=$(mktemp -d "${TMPDIR:-/tmp}/tierrw_test.XXXXXX") || exit 1
trap 'rm -rf "$T"' EXIT
TR="python3 tools/tier_rewrite.py"
fail=0
bad() { echo "FAIL: $*"; fail=1; }

$TR header --cxx test/v34/v34obj_cxx.txt --tag tagV34Object --check >/dev/null || bad "1: src/v34/v34_object_view.h is stale"
$TR slice --check >/dev/null || bad "1: src/v34/v34_subobjects.h is stale"
$TR header --layout test/vpcm/vpcm_flo_modem.layout --class VPcmFloModem --prefix VPFM_ --guard V34CTX_H -o $T/ctx.h >/dev/null; [ "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' $T/ctx.h)" = "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' src/v34/v34_flomodem_view.h)" ] || bad "1: src/v34/v34_flomodem_view.h is stale"
$TR header --layout test/v34/v01.layout --class K56FlexHandle --prefix V34K56_ --guard V34K56_H --tag K56FlexHandle -o $T/k56.h >/dev/null; [ "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' $T/k56.h)" = "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' src/v34/v34_k56flex_view.h)" ] || bad "1: src/v34/v34_k56flex_view.h is stale"
$TR header --layout test/vpcm/pcm_parameters.layout --class V90Parameters --prefix V34CFG_ --guard V34CFG_H -o $T/cfg.h >/dev/null; [ "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' $T/cfg.h)" = "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' src/v34/v34_v90params_view.h)" ] || bad "1: src/v34/v34_v90params_view.h is stale"
$TR header --layout test/vpcm/vpcm_flo_modem.layout --class V90Phase2InfoView --prefix V34V90I_ --guard V34V90I_H -o $T/v90i.h >/dev/null; [ "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' $T/v90i.h)" = "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' src/v34/v34_v90_phase2_view.h)" ] || bad "1: src/v34/v34_v90_phase2_view.h is stale"
$TR header --layout test/vpcm/vpcm_flo_modem.layout --class V92Phase2InfoView --prefix V34V92I_ --guard V34V92I_H -o $T/v92i.h >/dev/null; [ "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' $T/v92i.h)" = "$(sed -e '/^ \* @file /d' -e '/^ \*   python3 tools/,/^ \*$/d' src/v34/v34_v92_phase2_view.h)" ] || bad "1: src/v34/v34_v92_phase2_view.h is stale"
[ "$(grep -c '^#define V34M_0x' src/v34/v34_object_view.h)" -ge 500 ] || bad "1: member macro count"

lib() { if [ -f "src/v34/$1" ]; then echo "src/v34/$1"; else echo "src/vpcm/$1"; fi; }   # RS4: the plain-C V.34 tier files (the former v34_tier*.c) live in src/v34/
ALL="v34_agc_demod v34_init v34_setup v34_tx_level v34_frame_decoder v34_modem_init v34_tx_symbol v34_echo_rx_timing v34_detectors v34_handshake_helpers
     v34_modulator v34_receiver v34_probe_select v34_handshake_init v34_api v34_unused"   # the 16 tiers 2, 3ab, 3c, 3d, 4a, 4b, 5a - 5d, 6a, 7a, 8a, 9a, 10a, 10b
for f in $ALL; do
  $TR rewrite $(lib $f.c) -o $T/$f.c 2>$T/$f.log || { bad "$f: rewrite"; continue; }
  cmp -s $(lib $f.c) $T/$f.c || bad "2: $f is not a fixed point of the rewrite"
  grep -q ' 0 unresolved ' $T/$f.log || bad "2: $f has unresolved literal-obj sites"
  $TR rewrite --mode all $(lib $f.c) -o $T/$f.all.c 2>/dev/null
  $TR check $T/$f.all.c --orig $(lib $f.c) -I src >$T/$f.chk || { bad "2: $f --mode all identity"; cat $T/$f.chk; }
  echo "$f: $(head -1 $T/$f.log | sed 's/ accessors=.*//; s/^[a-z0-9_]*\.c: //') | $(cat $T/$f.chk | tr '\n' ' ' | sed 's/v34_[a-z0-9_]*\.c//g; s/ lines of objdump -dr//g; s/-fno-strict-aliasing/pinned/g')"
done

# ---- 3. sub-object modes on temporary copies (identity at -O2 / pinned; -O0 can differ by the accessor spelling, `apply` picks it)
sub() { f=$1; st=$2; base=$3
  $TR rewrite --struct $st --base $base $(lib $f.c) -o $T/$f.$st.c 2>/dev/null || { bad "3: $f $st rewrite"; return; }
  { $TR check $T/$f.$st.c --orig $(lib $f.c) -I src --opt=-O2 && $TR check $T/$f.$st.c --orig $(lib $f.c) -I src "--opt=-O2 -fno-strict-aliasing"; } >$T/$f.$st.chk || { bad "3: $f $st identity"; cat $T/$f.$st.chk; }
  echo "$f --struct $st --base $base: $(cat $T/$f.$st.chk | tr '\n' ' ' | sed 's/v34_[a-z0-9_]*\.[a-z]*\.c//g; s/ lines of objdump -dr//g')"; }
sub v34_receiver rx b; sub v34_setup mod m; sub v34_detectors det det; sub v34_probe_select rate q; sub v34_probe_select info p; sub v34_agc_demod ctx ctx

# RS4: the synthetic files below stand in for tier files of src/v34/, whose bare includes ("v34_object_view.h") need -I src/v34 as well
# ---- 4. synthetic
mkdir $T/syn
cat > $T/syn/s1.c <<'EOS'
#include <stdint.h>
typedef int32_t s32_t __attribute__((may_alias));
typedef int16_t s16_t __attribute__((may_alias));
#define OBJ_BYTES(o, off) ((unsigned char *)(o) + (off))
#define S32(o, off) (*(s32_t *)OBJ_BYTES(o, off))
#define S16(o, off) (*(s16_t *)OBJ_BYTES(o, off))
#define W16(p, q) (*(uint16_t *)((unsigned char *)(p) + (q)))   /* alternative macro shape, other parameter names */
/* S32(obj, 0x230) in a comment must stay */
int f(void *obj, void *b, int i) {
  const char *s = "S32(obj, 0x230)";
  int x = S32(obj, 0x230);          // trailing S32(obj, 0x230) comment
  x += S32(obj, 560);               /* decimal 560 == 0x230 */
  x += S16(obj, 0X238);             /* upper-case hex */
  x += S16(obj, 0x232);             /* inside minSigLevel_: bound as member + 2 (interior) */
  x += S32(obj, 0x2f58);            /* inside the constPts_ unknown-region: opaque, stays */
  x += S32(obj, 0x2aa8 + 4 * i);    /* computed */
  x += S32(b, 0x230);               /* other base */
  x += W16(obj, 0x230);             /* shape with other param names */
  S32(obj, 0x234) = x + (int)s[0];
  return x + S16(obj,0x04);
}
int g(void *obj) { return S32(obj, 0x230); }   /* excepted by the exceptions file below */
EOS
printf 's1.c g   # reason\ns1_new.c g\n' > $T/syn/deny.txt
$TR rewrite --deny-file $T/syn/deny.txt $T/syn/s1.c -o $T/syn/s1_new.c 2>$T/syn/log || bad "4: synthetic rewrite"
cat $T/syn/log
n_t=$(grep -c "S32T(obj, 0x230)\|S32T(obj, 0x234)\|S16T(obj, 0x238)\|S16T(obj, 0x4)\|W16T(obj, 0x230)\|S16TI(obj, 0x232, minSigLevel_, 2)" $T/syn/s1_new.c)
[ "$n_t" -eq 7 ] || bad "4: expected 7 rewritten lines, got $n_t"
grep -q 'S16(obj, 0x232)' $T/syn/s1_new.c && bad "4: interior offset was not rewritten"
grep -q 'S32(obj, 0x2aa8 + 4 \* i)' $T/syn/s1_new.c || bad "4: computed offset was touched"
grep -q 'S32(b, 0x230)' $T/syn/s1_new.c || bad "4: other base was touched"
grep -q 'S32(obj, 0x2f58)' $T/syn/s1_new.c || bad "4: opaque-region offset was touched"
grep -q 'int g(void \*obj) { return S32(obj, 0x230); }' $T/syn/s1_new.c || bad "4: excepted function was touched"
grep -q '^/\* S32(obj, 0x230) in a comment must stay \*/' $T/syn/s1_new.c || bad "4: comment was touched"
grep -q '"S32(obj, 0x230)"' $T/syn/s1_new.c || bad "4: string was touched"
grep -q 'W16T(obj, 0x230)' $T/syn/s1_new.c || bad "4: W16 (non-OBJ_BYTES shape) handling"
$TR worklist --deny-file $T/syn/deny.txt $T/syn/s1.c > $T/syn/wl; grep -q "0x2f58\*1" $T/syn/wl || bad "4: worklist lacks 0x2f58"
$TR check $T/syn/s1_new.c --orig $T/syn/s1.c -I src -I src/v34 >$T/syn/chk || { bad "4: synthetic identity"; cat $T/syn/chk; }
# negative control: a rewrite that changes the code must be reported as DIFFERENT
sed 's/S16T(obj, 0x238)/S32T(obj, 0x238)/' $T/syn/s1_new.c > $T/syn/s1_bad.c
$TR check $T/syn/s1_bad.c --orig $T/syn/s1.c -I src -I src/v34 >$T/syn/chk2 && bad "4: negative control passed" ; grep -q DIFFERENT $T/syn/chk2 || bad "4: negative control not reported DIFFERENT"
# idempotent
$TR rewrite --deny-file $T/syn/deny.txt $T/syn/s1_new.c -o $T/syn/s1_new2.c 2>/dev/null; cmp -s $T/syn/s1_new.c $T/syn/s1_new2.c || bad "4: not idempotent"
# hazards: an integer accessor on a pointer slot (ctx_ at 0x3548) and a pointer accessor on a non-pointer member are listed
cat > $T/syn/s2.c <<'EOS'
#include <stdint.h>
typedef uint32_t u32_t __attribute__((may_alias));
typedef void *ptr_t __attribute__((may_alias));
#define OBJ_BYTES(o, off) ((unsigned char *)(o) + (off))
#define U32(o, off) (*(u32_t *)OBJ_BYTES(o, off))
#define PTR(o, off) (*(ptr_t *)OBJ_BYTES(o, off))
void h(void *obj) { U32(obj, 0x3548) = 0; PTR(obj, 0x230) = 0; }
EOS
$TR worklist --hazards $T/syn/s2.c > $T/syn/haz; grep -q "integer accessor on pointer slot ctx_" $T/syn/haz || bad "4: hazard: integer accessor on a pointer slot not reported"
grep -q "pointer accessor on" $T/syn/haz || bad "4: hazard: pointer accessor on a non-pointer member not reported"

# T04: the generated files are current (map header for the x86-64 harnesses, header, sub-object structs, layout)
$TR mapheader --check > /dev/null 2>&1 || bad "T04: test/v34/v34objmap.h is not current (tier_rewrite.py mapheader)"
$TR slice --check > /dev/null 2>&1 || bad "T04: src/v34/v34_subobjects.h is not current (tier_rewrite.py slice)"
# T04: a bytes (flat sub-object) rewrite keeps the expression shape: the literal becomes V34REL(base, abs) and nothing else changes
cat > $T/syn/s4.c <<'EOS'
#include <stddef.h>
#include <stdint.h>
typedef int16_t s16_t __attribute__((may_alias));
#define OBJ_BYTES(o, off) ((unsigned char *)(o) + (off))
#define S16(o, off) (*(s16_t *)OBJ_BYTES(o, off))
int f(void *m, int i) { return S16(m, 0xcbc + i * 2); }
EOS
$TR bytes $T/syn/s4.c --accessors --base m --sub mod > $T/syn/s4.out 2>&1; grep -q "1 rewritten" $T/syn/s4.out || bad "T04: bytes --sub mod did not rewrite the synthetic site"

# T05 / V34SUB: V34Obj EMBEDS the sub-object structs (named members for all of them since V34CLEAN: no anonymous unions over flat members any more): the header compiles at
# -m32 and -m64 (the static asserts prove, member by member, that the structs and the flat members agree) and the embedded sub-objects are all there
for m in -m32 -m64; do printf '#include "v34/v34_object_view.h"\nint main(void){return (int)sizeof(V34Obj);}\n' > $T/emb.c; gcc $m -Isrc -fsyntax-only -Wall $T/emb.c 2>$T/emb.err || { bad "T05: v34_object_view.h does not compile with $m"; head -5 $T/emb.err; }; done
[ "$(grep -cE '^ +V34(Rx|Mod|Tx|Echo|ToneDet|Rate|Probe) (rx|mod|tx|echo0|echo1|det|rate|probe);' src/v34/v34_object_view.h)" -eq 8 ] || bad "T05: the sole sub-object members are missing from src/v34/v34_object_view.h"
[ "$(grep -cE '^ +V34(Info0a|Info0d|Info|Info1c|MpQ|Retrain) [a-z0-9]+;' src/v34/v34_object_view.h)" -eq 6 ] || bad "T05: the embedded INFO record members are missing from src/v34/v34_object_view.h"
# T05: embed: V34REL(B, ABS) / V34RELI / V34RELIJ sites -> typed member access / offsetof of the sub-object struct (identity at -m32)
cat > $T/syn/s5.c <<'EOS'
#include <stddef.h>
#include <stdint.h>
typedef int16_t s16_t __attribute__((may_alias));
typedef void *ptr_t __attribute__((may_alias));
#define OBJ_BYTES(o, off) ((unsigned char *)(o) + (off))
#define S16(o, off) (*(s16_t *)OBJ_BYTES(o, off))
#define PTR(o, off) (*(ptr_t *)OBJ_BYTES(o, off))
#include "v34/v34_object_view.h"
#define V34REL(base, off) ((int)(offsetof(V34Obj, V34M_##off) - offsetof(V34Obj, V34M_##base)))
#define V34RELI(base, mem, d) ((int)(offsetof(V34Obj, mem) - offsetof(V34Obj, V34M_##base)) + (d))
#define V34RELIJ(bmem, bd, mem, d) ((int)(offsetof(V34Obj, mem) - offsetof(V34Obj, bmem)) - (bd) + (d))
int f(void *b, void *m, void *R, int i) {
  PTR(b, V34REL(0x264, 0x394)) = 0;                       /* typed: rxEnd_ of V34Rx */
  S16(m, V34REL(0x1450, 0x1450) + 2 * i) = 1;             /* offsetof(V34Mod, ...) + computed */
  S16(b, V34RELI(0x264, rx.unk04d4_, 8) + 2 * i) = 2;       /* inside an unknown-region */
  return S16(R, V34RELIJ(probeEnergy_, 200, probe.bin_, 12) + 0x2c * i);   /* the probe block (the first member of V34Probe) */
}
EOS
cp $T/syn/s5.c $T/syn/s5_new.c; $TR embed $T/syn/s5_new.c --apply -I src -I src/v34 > $T/syn/s5.log 2>&1; grep -q "typed 1, offsetof 3, left 0" $T/syn/s5.log || { bad "T05: embed counts"; cat $T/syn/s5.log; }
grep -q 'PTRT_rx(b, 0x130)' $T/syn/s5_new.c || bad "T05: embed: typed site"
grep -q 'offsetof(V34Probe, bin_) + 12' $T/syn/s5_new.c || bad "T05: embed: probe site"
grep -q 'define V34REL' $T/syn/s5_new.c && bad "T05: embed: unused V34REL helpers were not dropped"

# T06: shims: a parameter named `r` (tier 5d getbit) must not clash with the result variable; a type macro without `*` (V34P6T) is passed through
cat > $T/syn/s6.c <<'EOS'
#include <stdint.h>
#include "v34_img.h"
#if defined(__x86_64__)
#define V34P6T void *
#else
#define V34P6T uint32_t
#endif
extern int getbit(void *r);
extern int initV34(void *blk, unsigned int p2, V34P6T p6);
/* V34SHIMS */
int use(void *p) { return getbit(p) + initV34(p, 1, 0); }
EOS
$TR shims $T/syn/s6.c > $T/syn/s6.log 2>&1; grep -q ': 2 shims' $T/syn/s6.log || { bad "T06: shims count"; cat $T/syn/s6.log; }
grep -q 'v34r_' $T/syn/s6.c || bad "T06: shims: result variable not renamed for a parameter named r"
gcc -m64 -Itest/v34 -Isrc -fsyntax-only -Wall $T/syn/s6.c 2>$T/syn/s6.err || { bad "T06: generated shims do not compile"; head -5 $T/syn/s6.err; }

[ $fail = 0 ] && echo "TIER_REWRITE PASS" || echo "TIER_REWRITE FAIL"
exit $fail
