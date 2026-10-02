"""
Robot Taban Konumu Otomatik Tespiti

TABAN_MARKER_ID (varsayilan: 0), robotun tabanina YAKIN (yanina) yapistirilir.
Marker'in olculen konumuna, elle olculmus sabit bir yerel ofset (marker'in
kendi acisina gore dondurulerek) eklenerek robotun gercek J1 donme merkezinin
masa koordinat sistemindeki cm konumu bulunur. Bu deger fiziksel olarak sabit
oldugu icin bir kez kilitlenir, bir daha guncellenmez.
"""
import numpy as np
import math
from collections import deque


class RobotTabanBulucu:
    def __init__(self, yerel_ofset_x_cm, yerel_ofset_y_cm, kilitleme_kare=15, kayip_max_kare=60):
        self.yerel_ofset = np.array([yerel_ofset_x_cm, yerel_ofset_y_cm])
        self.kilitleme_kare = kilitleme_kare
        self.kayip_max_kare = kayip_max_kare
        self.olcum_gecmisi = deque(maxlen=kilitleme_kare)
        self.kilitli_konum = None
        self.kayip_sayaci = 0

    def _yerel_ofseti_dondur(self, aci_derece):
        aci_rad = math.radians(aci_derece)
        cos_a, sin_a = math.cos(aci_rad), math.sin(aci_rad)
        rot = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        return rot @ self.yerel_ofset

    def guncelle(self, marker_x_cm, marker_y_cm, marker_aci_derece):
        """Donus: (robot_x_cm, robot_y_cm, kilitli_mi: bool)"""
        if self.kilitli_konum is not None:
            self.kayip_sayaci = 0
            return self.kilitli_konum[0], self.kilitli_konum[1], True

        if marker_x_cm is None:
            self.kayip_sayaci += 1
            return None, None, False

        ofset_donmus = self._yerel_ofseti_dondur(marker_aci_derece)
        robot_x = marker_x_cm + ofset_donmus[0]
        robot_y = marker_y_cm + ofset_donmus[1]
        self.olcum_gecmisi.append((robot_x, robot_y))

        if len(self.olcum_gecmisi) >= self.kilitleme_kare:
            medyan = np.median(np.array(self.olcum_gecmisi), axis=0)
            self.kilitli_konum = (float(medyan[0]), float(medyan[1]))
            return self.kilitli_konum[0], self.kilitli_konum[1], True

        return robot_x, robot_y, False

    def kayip_mi(self):
        return self.kilitli_konum is None and self.kayip_sayaci > self.kayip_max_kare

    def sifirla(self):
        """Robot fiziksel olarak yeniden konumlandirildiginda manuel cagrilir."""
        self.olcum_gecmisi.clear()
        self.kilitli_konum = None
        self.kayip_sayaci = 0
