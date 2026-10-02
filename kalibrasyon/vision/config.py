"""
Bu dosya, projedeki tüm temel ayarları (kamera portu, kare/marker boyutları, dosya yolları vb.) tek bir merkezde tutar.
Diğer tüm modüller ihtiyaç duydukları parametreleri bu konfigürasyon sınıfından çeker.
"""
import cv2
from dataclasses import dataclass, field
from typing import Tuple, Dict

@dataclass
class VisionConfig:
    SUTUN_SAYISI: int = 5
    SATIR_SAYISI: int = 7
    KARE_UZUNLUGU_M: float = 0.04
    MARKER_UZUNLUGU_M: float = 0.03
    DICTIONARY: int = cv2.aruco.DICT_4X4_50
    KAMERA_INDEX: int = 1
    MAX_KARE_PER_TUR: int = 50
    MIN_GECERLI_TOPLAM: int = 40
    YAKALAMA_ARALIGI_SN: float = 1.5
    CAPTURE_MIN_KOSE: int = 8
    VALID_MIN_KOSE: int = 18
    HEDEF_MARKER_IDS: list = field(default_factory=lambda: [6, 7, 8, 9])
    ROBOT_BASE_MARKER_ID: int = 0
    BILEK_MARKER_ID: int = 5
    NESNE_BOYUTLARI: dict = field(default_factory=lambda: {
        0: 2.5, 1: 2.5, 2: 2.5, 3: 2.5, 4: 2.5, 5: 2.5,
        6: 2.5, 7: 2.5, 8: 2.5, 9: 2.5
    })
    KOSE_MARKER_IDLERI: list = field(default_factory=lambda: [1, 2, 3, 4])
    KOSE_MARKER_BOYUTU_M: float = 0.025
    KUP_MARKER_BOYUTU_M: float = 0.025
    TABAN_MARKER_BOYUTU_M: float = 0.061
    BILEK_MARKER_BOYUTU_M: float = 0.025
    
    TABAN_MARKER_YEREL_OFSET_X_CM: float = 13.5
    TABAN_MARKER_YEREL_OFSET_Y_CM: float = 0.0
    TABAN_KILITLEME_KARE: int = 15
    TABAN_KAYIP_MAX_KARE: int = 60

    def get_marker_boyutu(self, marker_id: int) -> float:
        if marker_id == self.ROBOT_BASE_MARKER_ID:
            return self.TABAN_MARKER_BOYUTU_M
        if marker_id == self.BILEK_MARKER_ID:
            return self.BILEK_MARKER_BOYUTU_M
        if marker_id in self.KOSE_MARKER_IDLERI:
            return self.KOSE_MARKER_BOYUTU_M
        return self.KUP_MARKER_BOYUTU_M
    KAMERA_YUKSEKLIK_CM: float = 54.5
    KAMERA_ROTASYON: int = 0
    BIRDS_EYE_PX_PER_CM: int = 10
    LINK1_UZUNLUK_CM: float = 21.0
    LINK2_UZUNLUK_CM: float = 24.0
    LINK3_UZUNLUK_CM: float = 11.0
    TABAN_YUKSEKLIK_CM: float = 9.0
    HEDEF_GRIPPER_ACISI_DERECE: float = 0.0
    YAKLASMA_YUKSEKLIK_CM: float = 1.5
    GUVENLI_GECIS_YUKSEKLIK_CM: float = 20.0
    J1_TOLERANS_DERECE: float = 3.0
    GRIPPER_KAPALI_ACI: int = 130
    GRIPPER_ACIK_ACI: int = 30
    GRIPPER_MIN_ACI: int = 30
    GRIPPER_MAX_ACI: int = 130
    J1_OFFSET: float = -42.0
    J2_OFFSET: float = -12.0
    J3_OFFSET: float = -61.0
    J4_OFFSET: float = -23.0
    J5_OFFSET: float = -44.0
    ROBOT_BASE_OFFSET_X_CM: float = -9.10
    ROBOT_BASE_OFFSET_Y_CM: float = 49.77
    KAPALI_CEVRIM_TOLERANS_CM: float = 1.5
    KAPALI_CEVRIM_HIZ: float = 0.4

    SERVER_URL: str = "http://127.0.0.1:5000/set_angles"
    KAVRAMA_BEKLEME_SN: float = 1.5
    KALDIRMA_YUKSEKLIK_CM: float = 15.0
    KALDIRMA_BEKLEME_SN: float = 1.0
    GERI_DONUS_BEKLEME_SN: float = 1.0
    KONUM_SABITLEME_KARE: int = 10

    # --- Katman 1: ID-bazlı köşe sıralama ve kayıp kare yönetimi ---
    KOSE_ID_TO_POSITION: dict = field(default_factory=lambda: {
        2: "sol_ust", 1: "sag_ust", 3: "sag_alt", 4: "sol_alt"
    })
    KOSE_KAYIP_MAX_KARE: int = 30

    # --- Katman 2: 1Euro Filter parametreleri ---
    ONEEURO_MIN_CUTOFF: float = 1.0
    ONEEURO_BETA: float = 0.015

    # --- Bilek Marker (ID 5) Filtrelemesi ---
    BILEK_OUTLIER_PENCERE_BOYUTU: int = 3
    BILEK_OUTLIER_ESIK_CM: float = 2.0
    BILEK_OUTLIER_ARDISIK_KABUL_ESIK: int = 2  # Pencere küçük → daha hızlı kabul
    BILEK_ONEEURO_MIN_CUTOFF: float = 2.0
    BILEK_ONEEURO_BETA: float = 0.05

    # --- Katman 3: Güvenlik Ağı (Outlier) + Spline Yörünge ---
    OUTLIER_PENCERE_BOYUTU: int = 5
    OUTLIER_ESIK_CM: float = 3.0
    OUTLIER_ARDISIK_KABUL_ESIK: int = 3  # Art arda bu kadar outlier → gerçek hareket
    SPLINE_SURE_GUVENLI_SN: float = 1.0
    SPLINE_SURE_ALCAK_SN: float = 1.2
    SPLINE_FPS: int = 30

    PHOTOS_DIR: str = "camPhotos"
    CALIBRATION_FILE: str = "calibration_result.yaml"
    LOG_FILE: str = "kalibrasyon_log.md"
    HOMOGRAPHY_FILE: str = "homography.yaml"
