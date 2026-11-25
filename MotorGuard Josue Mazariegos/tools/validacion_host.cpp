// validacion_host.cpp - prueba el DSP + modelo local en el PC (sin ESP32).
//   g++ -O2 -I../wokwi validacion_host.cpp -o validacion_host && ./validacion_host
// Genera vibracion con el MISMO codigo que el firmware (motorguard_sim.h), le suma gravedad
// y un offset para comprobar que la resta de DC funciona, y mide aciertos y falsas alarmas.
#include <stdio.h>
#include <string.h>
#include "motorguard_dsp.h"
#include "motorguard_sim.h"

static float win[MG_WIN][MG_AXES];

int main() {
  MGDsp dsp;
  const int VENTANAS = 300;
  int conf[MODO_N][MG_NCLASS];
  memset(conf, 0, sizeof(conf));
  float anMin[MODO_N], anMax[MODO_N], anSum[MODO_N];
  int anFlag[MODO_N] = {0};

  for (int m = 0; m < MODO_N; m++) {
    MGSim sim(1234567u + 77u * m);
    anMin[m] = 1e9; anMax[m] = 0; anSum[m] = 0;
    for (int w = 0; w < VENTANAS; w++) {
      for (int n = 0; n < MG_WIN; n++) {
        float a[3]; sim.step(m, a);
        win[n][0] = a[0] + 0.02f; win[n][1] = a[1] - 0.01f; win[n][2] = a[2] + 1.0f;  // gravedad en Z
      }
      float f[MG_NFEAT], rms;
      dsp.extract(win, f, &rms);
      MGResult r = dsp.classify(f, rms);
      conf[m][r.clase]++;
      anSum[m] += r.anomalia;
      if (r.anomalia < anMin[m]) anMin[m] = r.anomalia;
      if (r.anomalia > anMax[m]) anMax[m] = r.anomalia;
      if (r.anomalia > 1.0f) anFlag[m]++;
    }
  }
  int ok = 0, tot = 0;
  printf("Matriz de confusion (filas = modo simulado, columnas = clase predicha)\n%14s", "");
  for (int c = 0; c < MG_NCLASS; c++) printf("%14s", MG_CLASS_NAMES[c]);
  printf("\n");
  for (int m = 0; m < MODO_N; m++) {
    printf("%14s", MG_MODO_NOMBRE[m]);
    for (int c = 0; c < MG_NCLASS; c++) { printf("%14d", conf[m][c]); tot += conf[m][c]; if (c == m) ok += conf[m][c]; }
    printf("\n");
  }
  printf("\nExactitud: %.1f %%\n", 100.0 * ok / tot);
  for (int m = 0; m < MODO_N; m++)
    printf("Anomalia %-14s min %.2f  media %.2f  max %.2f  (>1.0 en %.1f %% de ventanas)\n",
           MG_MODO_NOMBRE[m], anMin[m], anSum[m] / VENTANAS, anMax[m], 100.0 * anFlag[m] / VENTANAS);
  printf("RESULTADO: %s\n", (ok == tot && anFlag[0] == 0) ? "OK" : "REVISAR");
  return (ok == tot && anFlag[0] == 0) ? 0 : 1;
}
