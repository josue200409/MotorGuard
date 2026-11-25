#!/usr/bin/env python3
"""
MotorGuard - generador de datos sinteticos + entrenamiento del modelo local.

Hace DOS cosas con la misma señal sintetica de vibracion:
  1) Escribe CSVs listos para subir a Edge Impulse Studio (Data acquisition > Upload data).
  2) Entrena un clasificador de centroides + detector de anomalias K-means (solo con datos
     "normal") y exporta wokwi/modelo_local.h para el firmware (modelo de respaldo que
     corre en el ESP32 sin Edge Impulse).

Las caracteristicas (log10 de energia por banda, ventana Hann, 200 muestras a 100 Hz) son
IDENTICAS a las de motorguard_dsp.h. Si cambias una, cambia la otra.

Uso:
    python generar_dataset.py                 # CSVs + modelo_local.h
    python generar_dataset.py --sin-csv       # solo reentrena modelo_local.h
"""
import argparse
import os
import numpy as np

FS = 100            # Hz, frecuencia de muestreo
N = 200             # muestras por ventana (2 s)
F_ROT = 15.0        # Hz  (900 rpm). 3x = 45 Hz < Nyquist (50 Hz)
CLASES = ["normal", "desbalance", "desalineacion", "rodamiento"]
# Bandas en bins de 0.5 Hz: 1-5, 5-22, 22-38, 38-50 Hz  (igual que motorguard_dsp.h)
BANDAS = [(2, 9), (10, 43), (44, 75), (76, 100)]


# ------------------------------------------------------------------ señal sintetica
def gen_ventana(cls, n, rng, sev=1.0):
    """Aceleracion (g, sin gravedad) de n muestras: array (n, 3) = [x, y, z]."""
    t = np.arange(n) / FS
    w = 2 * np.pi * F_ROT * (1 + rng.uniform(-0.03, 0.03))     # velocidad variable +-3 %
    ph = lambda: rng.uniform(0, 2 * np.pi)
    s = lambda m, p: np.sin(m * w * t + p)
    kb = rng.uniform(0.8, 1.25)                                 # variacion de montaje/carga
    kf = sev * rng.uniform(0.85, 1.15)                          # severidad de la falla
    p0 = ph()
    x = np.zeros(n); y = np.zeros(n); z = np.zeros(n)
    sigma = 0.02
    if cls == "normal":
        x += kb * (0.030 * s(1, p0) + 0.012 * s(2, ph()))
        y += kb * 0.030 * s(1, p0 + np.pi / 2)
        z += kb * 0.012 * s(1, ph())
    elif cls == "desbalance":                                   # 1x radial dominante
        x += kf * 0.40 * s(1, p0) + 0.02 * s(2, ph())
        y += kf * 0.40 * s(1, p0 + np.pi / 2)
        z += 0.04 * s(1, ph())
    elif cls == "desalineacion":                                # 2x/3x + componente axial
        x += kf * (0.10 * s(1, ph()) + 0.28 * s(2, ph()) + 0.08 * s(3, ph()))
        y += kf * (0.10 * s(1, ph()) + 0.25 * s(2, ph()) + 0.06 * s(3, ph()))
        z += kf * (0.20 * s(1, ph()) + 0.18 * s(2, ph()))
    elif cls == "rodamiento":                                   # ruido de banda ancha + impactos
        x += 0.04 * s(1, p0)
        y += 0.04 * s(1, p0 + np.pi / 2)
        z += 0.02 * s(1, ph())
        sigma = 0.11 * sev
    sig = np.stack([x, y, z], axis=1) + rng.normal(0, sigma, (n, 3))
    if cls == "rodamiento":
        mask = rng.random((n, 3)) < 0.04
        sig += mask * rng.choice([-1.0, 1.0], (n, 3)) * rng.uniform(0.4, 0.9, (n, 3)) * sev
    return sig


# ------------------------------------------------------------------ caracteristicas
def features(win):
    """win: (N, 3) -> vector de 12 = [eje][banda] de log10(energia). Igual que el firmware."""
    x = win - win.mean(axis=0)
    w = np.hanning(N)[:, None]
    X = np.fft.rfft(x * w, axis=0)                              # bins de 0.5 Hz
    P = 2.0 * np.abs(X) ** 2 / (w.sum() ** 2)
    out = []
    for ax in range(3):
        for lo, hi in BANDAS:
            out.append(np.log10(P[lo:hi + 1, ax].sum() + 1e-9))
    return np.array(out)


def dataset(n_por_clase, rng, sev_lo=0.7, sev_hi=1.2):
    X, y = [], []
    for ci, c in enumerate(CLASES):
        for _ in range(n_por_clase):
            sev = rng.uniform(sev_lo, sev_hi)
            X.append(features(gen_ventana(c, N, rng, sev)))
            y.append(ci)
    return np.array(X), np.array(y)


# ------------------------------------------------------------------ modelo
def kmeans(X, k, rng, it=80):
    C = [X[rng.integers(len(X))]]
    for _ in range(k - 1):                                      # init k-means++
        d = np.min([((X - c) ** 2).sum(1) for c in C], axis=0)
        C.append(X[rng.choice(len(X), p=d / d.sum())])
    C = np.array(C)
    for _ in range(it):
        a = ((X[:, None, :] - C[None]) ** 2).sum(-1).argmin(1)
        for j in range(k):
            if (a == j).any():
                C[j] = X[a == j].mean(0)
    return C


def entrenar(rng):
    Xtr, ytr = dataset(400, rng)
    mean, std = Xtr.mean(0), Xtr.std(0) + 1e-6
    Z = (Xtr - mean) / std
    cent = np.array([Z[ytr == i].mean(0) for i in range(len(CLASES))])
    d_own = np.linalg.norm(Z - cent[ytr], axis=1)
    tau = float(np.median(d_own))                               # escala del softmax
    km = kmeans(Z[ytr == 0], 3, rng)                            # solo datos normales
    Xva, yva = dataset(400, rng)
    Zn = (Xva[yva == 0] - mean) / std
    dn = np.sqrt(((Zn[:, None, :] - km[None]) ** 2).sum(-1)).min(1)
    thr = float(np.percentile(dn, 99.5) * 1.2)
    return dict(mean=mean, std=std, cent=cent, tau=tau, km=km, thr=thr)


def evaluar(m, rng):
    X, y = dataset(150, rng)
    Z = (X - m["mean"]) / m["std"]
    d2 = ((Z[:, None, :] - m["cent"][None]) ** 2).sum(-1)
    pred = d2.argmin(1)
    conf = np.zeros((4, 4), int)
    for a, b in zip(y, pred):
        conf[a, b] += 1
    an = np.sqrt(((Z[:, None, :] - m["km"][None]) ** 2).sum(-1)).min(1) / m["thr"]
    print(f"[modelo] exactitud (datos nuevos): {100 * (pred == y).mean():.1f} %")
    print("[modelo] matriz de confusion (filas=real, cols=predicho):")
    print("         " + " ".join(f"{c[:6]:>7}" for c in CLASES))
    for i, c in enumerate(CLASES):
        print(f"{c[:8]:>8} " + " ".join(f"{v:7d}" for v in conf[i]))
    print(f"[modelo] anomalia normal: falsos positivos {100 * (an[y == 0] > 1).mean():.1f} % | "
          f"fallas detectadas {100 * (an[y != 0] > 1).mean():.1f} %")


def arr(a):
    a = np.atleast_1d(a)
    return "{" + ", ".join(f"{v:.7g}f" for v in a) + "}"


def escribir_header(m, ruta):
    L = ["// modelo_local.h - GENERADO por tools/generar_dataset.py. No editar a mano.",
         "// Modelo de respaldo (centroides + K-means) entrenado con datos sinteticos.",
         "#pragma once", "",
         f"#define MG_NCLASS {len(CLASES)}",
         f"#define MG_KM_K   {len(m['km'])}",
         f"#define MG_NFEAT_MODEL {len(m['mean'])}",
         f"static const float MG_TAU = {m['tau']:.7g}f;",
         f"static const float MG_ANOM_THR = {m['thr']:.7g}f;",
         "static const char* const MG_CLASS_NAMES[MG_NCLASS] = {" +
         ", ".join(f'"{c}"' for c in CLASES) + "};",
         f"static const float MG_FEAT_MEAN[{len(m['mean'])}] = {arr(m['mean'])};",
         f"static const float MG_FEAT_STD[{len(m['std'])}] = {arr(m['std'])};",
         f"static const float MG_CLASS_CENT[MG_NCLASS][{len(m['mean'])}] = {{"]
    L += ["  " + arr(c) + "," for c in m["cent"]]
    L += ["};", f"static const float MG_KM_CENT[MG_KM_K][{len(m['mean'])}] = {{"]
    L += ["  " + arr(c) + "," for c in m["km"]]
    L += ["};", ""]
    os.makedirs(os.path.dirname(os.path.abspath(ruta)), exist_ok=True)
    with open(ruta, "w") as f:
        f.write("\n".join(L))
    print(f"[modelo] escrito {os.path.abspath(ruta)}")


def escribir_csv(salida, archivos, segundos, rng):
    os.makedirs(salida, exist_ok=True)
    n = segundos * FS
    ts = (np.arange(n) * (1000 // FS)).astype(int)
    for c in CLASES:
        for i in range(1, archivos + 1):
            sig = gen_ventana(c, n, rng, rng.uniform(0.7, 1.2))
            data = np.column_stack([ts, sig])
            np.savetxt(os.path.join(salida, f"{c}.{i:03d}.csv"), data, delimiter=",",
                       header="timestamp,accX,accY,accZ", comments="",
                       fmt=["%d", "%.5f", "%.5f", "%.5f"])
    print(f"[csv] {archivos * len(CLASES)} archivos de {segundos} s en '{salida}/' "
          f"(etiqueta = texto antes del primer punto del nombre)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--archivos", type=int, default=30, help="CSV por clase")
    ap.add_argument("--segundos", type=int, default=10, help="duracion de cada CSV")
    ap.add_argument("--salida", default="dataset_edgeimpulse")
    ap.add_argument("--header", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                     "..", "wokwi", "modelo_local.h"))
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--sin-csv", action="store_true")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    m = entrenar(rng)
    evaluar(m, np.random.default_rng(a.seed + 1))
    escribir_header(m, a.header)
    if not a.sin_csv:
        escribir_csv(a.salida, a.archivos, a.segundos, np.random.default_rng(a.seed + 2))
