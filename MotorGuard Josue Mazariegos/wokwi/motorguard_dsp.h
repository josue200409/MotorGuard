// motorguard_dsp.h
// Extraccion de caracteristicas espectrales + clasificador TinyML local.
// C++ puro (sin Arduino) para poder validarlo en PC con tools/validacion_host.cpp.
// Las caracteristicas son identicas a tools/generar_dataset.py.
#pragma once
#include <math.h>
#include <stdint.h>
#include "modelo_local.h"

#define MG_FS_HZ    100            // frecuencia de muestreo constante
#define MG_WIN      200            // ventana de 2 s
#define MG_AXES     3
#define MG_NBANDS   4
#define MG_NFEAT    (MG_AXES * MG_NBANDS)

// Resolucion espectral = 100 Hz / 200 = 0.5 Hz por bin.
// Bandas: 1-5 Hz, 5-22 Hz (1x = 15 Hz), 22-38 Hz (2x = 30 Hz), 38-50 Hz (3x = 45 Hz)
static const int MG_BAND_LO[MG_NBANDS] = { 2, 10, 44, 76 };
static const int MG_BAND_HI[MG_NBANDS] = { 9, 43, 75, 100 };

struct MGResult {
  int   clase;        // indice en MG_CLASS_NAMES
  float conf;         // confianza 0..1
  float anomalia;     // distancia K-means / umbral  (> 1 = anomalo)
  float rms;          // RMS AC del vector de aceleracion, en g
};

class MGDsp {
 public:
  MGDsp() {
    wsum = 0;
    for (int n = 0; n < MG_WIN; n++) {
      cosT[n] = cosf(2.0f * (float)M_PI * n / MG_WIN);
      sinT[n] = sinf(2.0f * (float)M_PI * n / MG_WIN);
      hann[n] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * n / (MG_WIN - 1));
      wsum += hann[n];
    }
    for (int k = 0; k <= MG_WIN / 2; k++) {
      kBand[k] = -1;
      for (int b = 0; b < MG_NBANDS; b++)
        if (k >= MG_BAND_LO[b] && k <= MG_BAND_HI[b]) kBand[k] = b;
    }
  }

  // win[n][eje] en g. Quita la componente DC (gravedad) y calcula 12 caracteristicas.
  void extract(const float win[][MG_AXES], float feat[MG_NFEAT], float* rms) {
    double acc = 0;
    for (int ax = 0; ax < MG_AXES; ax++) {
      float mean = 0;
      for (int n = 0; n < MG_WIN; n++) mean += win[n][ax];
      mean /= MG_WIN;
      for (int n = 0; n < MG_WIN; n++) {
        float v = win[n][ax] - mean;
        acc += (double)v * v;
        xw[ax][n] = v * hann[n];
      }
    }
    if (rms) *rms = (float)sqrt(acc / MG_WIN);

    for (int i = 0; i < MG_NFEAT; i++) feat[i] = 0;
    for (int ax = 0; ax < MG_AXES; ax++) {
      float E[MG_NBANDS] = {0, 0, 0, 0};
      for (int k = MG_BAND_LO[0]; k <= MG_BAND_HI[MG_NBANDS - 1]; k++) {
        int b = kBand[k];
        if (b < 0) continue;
        float re = 0, im = 0;
        int idx = 0;
        for (int n = 0; n < MG_WIN; n++) {
          re += xw[ax][n] * cosT[idx];
          im += xw[ax][n] * sinT[idx];
          idx += k; if (idx >= MG_WIN) idx -= MG_WIN;
        }
        E[b] += re * re + im * im;
      }
      for (int b = 0; b < MG_NBANDS; b++)
        feat[ax * MG_NBANDS + b] = log10f(2.0f * E[b] / (wsum * wsum) + 1e-9f);
    }
  }

  // Clasificacion (centroides + softmax) y puntaje de anomalia (K-means de "normal").
  MGResult classify(const float feat[MG_NFEAT], float rms) {
    float z[MG_NFEAT];
    for (int i = 0; i < MG_NFEAT; i++) z[i] = (feat[i] - MG_FEAT_MEAN[i]) / MG_FEAT_STD[i];

    float d2[MG_NCLASS], dmin = 1e30f; int best = 0;
    for (int c = 0; c < MG_NCLASS; c++) {
      float s = 0;
      for (int i = 0; i < MG_NFEAT; i++) { float d = z[i] - MG_CLASS_CENT[c][i]; s += d * d; }
      d2[c] = s;
      if (s < dmin) { dmin = s; best = c; }
    }
    float sum = 0, e[MG_NCLASS];
    for (int c = 0; c < MG_NCLASS; c++) { e[c] = expf(-0.5f * (d2[c] - dmin) / (MG_TAU * MG_TAU)); sum += e[c]; }

    float km = 1e30f;
    for (int j = 0; j < MG_KM_K; j++) {
      float s = 0;
      for (int i = 0; i < MG_NFEAT; i++) { float d = z[i] - MG_KM_CENT[j][i]; s += d * d; }
      if (s < km) km = s;
    }
    MGResult r;
    r.clase = best;
    r.conf = e[best] / sum;
    r.anomalia = sqrtf(km) / MG_ANOM_THR;
    r.rms = rms;
    return r;
  }

 private:
  float cosT[MG_WIN], sinT[MG_WIN], hann[MG_WIN], wsum;
  float xw[MG_AXES][MG_WIN];
  int8_t kBand[MG_WIN / 2 + 1];
};
