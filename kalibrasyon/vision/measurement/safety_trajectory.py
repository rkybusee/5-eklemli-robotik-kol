"""
Güvenlik Ağı (Outlier Eleme) ve Kübik Spline Yörünge Üretimi

OutlierGuard: Medyan tabanlı kayan pencere ile kameradan gelen
ani sıçramaları (glitch) eler, son güvenli değeri korur.

eklem_yorungesi_uret: Başlangıç ve bitiş hızı 0 olacak şekilde
kübik spline yörünge üretir — motorlara adım fonksiyonu yerine
yumuşak bir hız profili sağlar, sarsıntıyı önler.
"""
import numpy as np
from collections import deque
from scipy.interpolate import CubicSpline
import logging

logger = logging.getLogger(__name__)


class OutlierGuard:
    """Medyan tabanli kayan pencere ile ani sicramalari (glitch) eler."""
    def __init__(self, window_size=5, esik_cm=3.0, ardisik_kabul_esik=3):
        self.gecmis = deque(maxlen=window_size)
        self.esik_cm = esik_cm
        self.ardisik_kabul_esik = ardisik_kabul_esik
        self.ardisik_outlier = 0

    def kontrol_et(self, x_cm, y_cm):
        """
        Yeni (x_cm, y_cm) değerini kontrol eder.

        Dönüş:
            (x, y, gecerli_mi)
            - gecerli_mi=True  → değer kabul edildi, geçmişe eklendi
            - gecerli_mi=False → outlier tespit edildi, son güvenli değer döndürüldü
        """
        yeni = np.array([x_cm, y_cm])
        if len(self.gecmis) < 2:
            self.gecmis.append(yeni)
            self.ardisik_outlier = 0
            return x_cm, y_cm, True

        medyan = np.median(np.array(self.gecmis), axis=0)
        mesafe = np.linalg.norm(yeni - medyan)

        if mesafe > self.esik_cm:
            self.ardisik_outlier += 1
            
            # Art arda ardisik_kabul_esik kadar outlier → obje gerçekten hareket etmiş
            if self.ardisik_outlier >= self.ardisik_kabul_esik:
                logger.debug("Cisim gerçekten hareket etti. OutlierGuard sıfırlanıyor.")
                self.sifirla()
                self.gecmis.append(yeni)
                self.ardisik_outlier = 0
                return x_cm, y_cm, True
            
            son_guvenli = self.gecmis[-1]
            logger.debug(f"Outlier elendi: mesafe={mesafe:.1f}cm > {self.esik_cm:.1f}cm (Sayaç: {self.ardisik_outlier})")
            return son_guvenli[0], son_guvenli[1], False

        self.ardisik_outlier = 0
        self.gecmis.append(yeni)
        return x_cm, y_cm, True

    def sifirla(self):
        """Geçmiş penceresini temizler (yeni hedef için)."""
        self.gecmis.clear()


def eklem_yorungesi_uret(baslangic_acilar, hedef_acilar, sure_sn=1.5, fps=30):
    """
    J1-J4 icin, baslangic ve bitis hizi 0 olacak sekilde kubik spline yorunge uretir.

    Parametreler:
        baslangic_acilar : [j1, j2, j3, j4] (derece) — mevcut pozisyon
        hedef_acilar     : [j1, j2, j3, j4] (derece) — hedef pozisyon
        sure_sn          : Geçiş süresi (saniye)
        fps              : Saniye başına adım sayısı

    Dönüş:
        (adim_sayisi, 4) boyutunda numpy dizisi — her satır bir zaman adımındaki
        [j1, j2, j3, j4] açılarını içerir.
    """
    adim_sayisi = max(2, int(sure_sn * fps))
    t_adimlari = np.linspace(0, sure_sn, adim_sayisi)

    yorunge = []
    for baslangic, hedef in zip(baslangic_acilar, hedef_acilar):
        # bc_type=((1, 0.0), (1, 0.0)) → başlangıç ve bitiş hızı 0
        cs = CubicSpline([0, sure_sn], [baslangic, hedef], bc_type=((1, 0.0), (1, 0.0)))
        yorunge.append(cs(t_adimlari))

    result = np.array(yorunge).T
    logger.debug(f"Spline yorunge uretildi: {adim_sayisi} adim, {sure_sn:.1f}s, "
                 f"baslangic={[f'{a:.0f}' for a in baslangic_acilar]} -> "
                 f"hedef={[f'{a:.0f}' for a in hedef_acilar]}")
    return result
