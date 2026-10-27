/* N01: V32FP_status differential test with a NOISY engine state (decoder error, rate class, warm-up counter set directly), i.e. the live-session
 * paths (decErr != 0: FPM_div / FPM_log10; SNR below SnrToRetrainTable -> V32_CTL ctl block -> V32FP_control) that test/v32/v32_engine.main.main.c never reaches
 * (its objects have decErr == 0 and the warm-up counter below 200).  One process per rate class; prints the status block and a few state words after
 * each of 3 calls.  Found two port bugs (FPM_log10's 2nd argument taken from the wrong FPM_div output; the retrain branch was an abort()). */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(int obj);
extern int V32FP_status(int obj, short *statusOut);

int main(int argc, char **argv)
{
  int rc = argc > 1 ? atoi(argv[1]) : 0;
  static const short dec[] = {0, 1, 5, 13, 64, 500, 5000, 0x1400, 0x2000, -5};
  static const short e256[] = {0, 4, 8, 40, 200, 1000};
  static const short h100[][2] = {{0, 0}, {100, 200}, {300, 100}, {32000, 2}};
  static const short warm[] = {0, 199, 200};
  unsigned i, j, k, w;
  for (i = 0; i < sizeof dec / 2; i++)
    for (j = 0; j < sizeof e256 / 2; j++)
      for (k = 0; k < 4; k++)
        for (w = 0; w < 3; w++) {
          int cfg[6], obj, bigbuf, c;
          short st[24];
          void *p;
          memset(cfg, 0, sizeof cfg);
          cfg[1] = 100; cfg[2] = 60000; cfg[3] = 700; cfg[4] = 1;
          *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16);
          p = V32FP_create(cfg, 0);
          obj = (int)(intptr_t)p;
          bigbuf = *(int *)(intptr_t)(obj + 0x68);
          *(short *)(intptr_t)(bigbuf + 0x2e) = (short)rc;
          *(short *)(intptr_t)(bigbuf + 0x50d6) = dec[i];
          *(short *)(intptr_t)(bigbuf + 0x256) = e256[j];
          *(short *)(intptr_t)(bigbuf + 0x100) = h100[k][0];
          *(short *)(intptr_t)(bigbuf + 0x102) = h100[k][1];
          *(short *)(intptr_t)(obj + 0x2c) = warm[w];
          printf("rc=%d dec=%d e256=%d h=%d,%d warm=%d:", rc, dec[i], e256[j], h100[k][0], h100[k][1], warm[w]);
          for (c = 0; c < 3; c++) {
            int r, q;
            memset(st, 0, sizeof st);
            r = V32FP_status(obj, st);
            printf(" [r=%d", r);
            for (q = 0; q < 22; q++) printf(" %d", (int)st[q]);
            printf(" | %d %d %d %d %x]", (int)*(short *)(intptr_t)(obj + 0x20), (int)*(short *)(intptr_t)(obj + 0x2c),
                   (int)*(short *)(intptr_t)(bigbuf + 0x50d8), (int)*(short *)(intptr_t)(bigbuf + 0x50d6), *(unsigned *)(intptr_t)(bigbuf + 0x1c));
          }
          printf("\n");
          V32FP_delete(obj);
        }
  return 0;
}
