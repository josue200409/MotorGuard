// motorguard_sim.h
// Wokwi no simula fisica: el MPU6050 solo entrega los valores fijos de sus sliders.
// Este generador SUMA una firma de vibracion sintetica a la lectura real del sensor
// para poder demostrar las fallas. En hardware fisico: SIMULAR_VIBRACION = 0.
// Misma "fisica" que tools/generar_dataset.py (motor a 900 rpm = 15 Hz).
#pragma once
#include <math.h>
#include <stdint.h>

enum MGModo { MODO_NORMAL = 0, MODO_DESBALANCE, MODO_DESALINEACION, MODO_RODAMIENTO, MODO_N };
static const char* const MG_MODO_NOMBRE[MODO_N] = { "normal", "desbalance", "desalineacion", "rodamiento" };

class MGSim {
 public:
  explicit MGSim(uint32_t seed = 2463534242u) : s(seed), ph(0), n(0), motorOn(true) {}

  void setMotor(bool on) { motorOn = on; }

  // Una muestra de aceleracion en g (sin gravedad), llamar a 100 Hz.
  void step(int modo, float out[3]) {
    const float W0 = 2.0f * (float)M_PI * 15.0f;
    float t = n / 100.0f;
    float w = W0 * (1.0f + 0.02f * sinf(2.0f * (float)M_PI * 0.05f * t));   // deriva de velocidad +-2 %
    ph += w / 100.0f;
    if (ph > 1000.0f * (float)M_PI) ph -= 1000.0f * (float)M_PI;
    n++;

    float x = 0, y = 0, z = 0, sigma = 0.02f;
    if (!motorOn) {
      sigma = 0.004f;                                        // motor detenido: solo ruido del sensor
    } else {
      float s1 = ph, s2 = 2 * ph, s3 = 3 * ph;
      const float H = (float)M_PI / 2;
      switch (modo) {
        case MODO_DESBALANCE:                                // 1x radial dominante
          x = 0.40f * sinf(s1)     + 0.02f * sinf(s2 + 1.0f);
          y = 0.40f * sinf(s1 + H);
          z = 0.04f * sinf(s1 + 2.0f);
          break;
        case MODO_DESALINEACION:                             // 2x/3x + axial
          x = 0.10f * sinf(s1) + 0.28f * sinf(s2 + 0.5f) + 0.08f * sinf(s3 + 1.2f);
          y = 0.10f * sinf(s1 + H) + 0.25f * sinf(s2 + 1.5f) + 0.06f * sinf(s3 + 0.3f);
          z = 0.20f * sinf(s1 + 0.7f) + 0.18f * sinf(s2 + 2.2f);
          break;
        case MODO_RODAMIENTO:                                // banda ancha + impactos
          x = 0.04f * sinf(s1);
          y = 0.04f * sinf(s1 + H);
          z = 0.02f * sinf(s1 + 2.0f);
          sigma = 0.11f;
          break;
        default:                                             // normal
          x = 0.030f * sinf(s1) + 0.012f * sinf(s2 + 1.0f);
          y = 0.030f * sinf(s1 + H);
          z = 0.012f * sinf(s1 + 2.0f);
      }
    }
    out[0] = x + sigma * gauss();
    out[1] = y + sigma * gauss();
    out[2] = z + sigma * gauss();
    if (motorOn && modo == MODO_RODAMIENTO) {
      for (int i = 0; i < 3; i++)
        if (uni() < 0.04f) out[i] += (uni() < 0.5f ? -1.0f : 1.0f) * (0.4f + 0.5f * uni());
    }
  }

 private:
  uint32_t s; float ph; uint32_t n; volatile bool motorOn;
  float uni() {                                              // xorshift32 -> [0,1)
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return (s >> 8) * (1.0f / 16777216.0f);
  }
  float gauss() {                                            // suma de 4 uniformes ~ N(0,1)
    return (uni() + uni() + uni() + uni() - 2.0f) * 1.7320508f;
  }
};
