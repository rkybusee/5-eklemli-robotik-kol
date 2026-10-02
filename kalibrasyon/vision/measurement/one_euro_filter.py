"""
1Euro Filter — Titreme Filtreleme

Kamera tabanlı konum tespitinde kaçınılmaz olan kare-kare titremeleri
yumuşatmak için kullanılır. Düşük hızda agresif filtreleme, yüksek hızda
(gerçek hareket) ise minimal gecikme sağlar.

Referans: Casiez et al., "1€ Filter: A Simple Speed-based Low-pass Filter
for Noisy Input in Interactive Systems", CHI 2012.
"""
import math


class LowPassFilter:
    def __init__(self, alpha):
        self.set_alpha(alpha)
        self.y = None

    def set_alpha(self, alpha):
        if not (0 < alpha <= 1):
            raise ValueError("alpha (0, 1] araliginda olmali")
        self.alpha = alpha

    def __call__(self, value, alpha=None):
        if alpha is not None:
            self.set_alpha(alpha)
        s = value if self.y is None else self.alpha * value + (1.0 - self.alpha) * self.y
        self.y = s
        return s


class OneEuroFilter:
    """
    1Euro filtresi.

    Parametreler:
        t0          : İlk zaman damgası (saniye)
        x0          : İlk sinyal değeri
        min_cutoff  : Minimum kesme frekansı (Hz). Düşük → daha fazla yumuşatma.
        beta        : Hız kazancı. Yüksek → hızlı hareketlerde daha az gecikme.
        dcutoff     : Türev sinyali için kesme frekansı (Hz).
    """
    def __init__(self, t0, x0, min_cutoff=1.0, beta=0.01, dcutoff=1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.dcutoff = float(dcutoff)
        self.x_filter = LowPassFilter(self._alpha(0.02, self.min_cutoff))
        self.dx_filter = LowPassFilter(self._alpha(0.02, self.dcutoff))
        self.x_prev = x0
        self.t_prev = t0

    def _alpha(self, t_e, cutoff):
        r = 2 * math.pi * cutoff * t_e
        return r / (r + 1)

    def __call__(self, t, x):
        t_e = max(t - self.t_prev, 1e-5)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.dx_filter(dx, self._alpha(t_e, self.dcutoff))
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        x_hat = self.x_filter(x, self._alpha(t_e, cutoff))
        self.x_prev, self.t_prev = x_hat, t
        return x_hat
