/* Constructor linked into every test executable while tools/stampkeys.py records a job's compile recipe: exit_group(0) before main(),
   so the binary runs instantly with no output.  Compiled by the recorder with -fPIC for -m32 and -m64 (no libc call: raw system call). */
#if defined(__x86_64__)
__attribute__((constructor(101), used)) static void sk_exit0(void) { __asm__ volatile("mov $231,%eax\n xor %edi,%edi\n syscall"); }
#else
__attribute__((constructor(101), used)) static void sk_exit0(void) { __asm__ volatile("mov $252,%eax\n xor %ebx,%ebx\n int $0x80"); }
#endif
