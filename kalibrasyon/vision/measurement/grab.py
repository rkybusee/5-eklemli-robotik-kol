"""
Cisim Yakalama (Grab) Modülü — Durum Makinesi ile Tam Otonom Kontrol

Kamera tepeden (kuşbakışı) dik bakarak masadaki ArUco marker'lı küpleri tespit eder,
robotik kolu IK ile yönlendirir, cismi kavrar, kaldırır ve başlangıç pozisyonuna döner.

Durum Makinesi:
    BEKLEME   → Hedef marker aranıyor
    TESPIT    → Marker bulundu, konum sabitleniyor (KONUM_SABITLEME_KARE kadar stabil olmalı)
    YAKLASMA  → IK ile cismin üzerine gidiliyor (spline yörünge, gripper açık)
    HIZALAMA  → Kapalı çevrim IBVS ile son mm hassasiyetinde hizalama
    KAVRAMA   → Gripper kapatılıyor
    KALDIRMA  → Kol yukarı kaldırılıyor
    GERI      → Başlangıç pozisyonuna dönüş
    BIRAKMA   → Gripper açılıp nesne bırakılıyor
    TAMAMLANDI → İşlem tamamlandı, yeni hedef için BEKLEME'ye dönüyor

Filtreler:
    - 1Euro Filter: Kare-kare titremeleri yumuşatır (Katman 2)
    - OutlierGuard: Medyan tabanlı ani sıçrama eleme (Katman 3)
    - Kübik Spline: Motorlara yumuşak yörünge profili (Katman 3)
"""
import cv2
import numpy as np
import logging
import math
import requests
import time
import enum
import sys
import os

# 'kalibrasyon' klasörünü (yani iki üst klasörü) sys.path'e ekleyelim ki 'vision' modülünü bulabilsin.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from vision.config import VisionConfig
from vision.core.utils import load_calibration
from vision.kinematics.ik_solver import hesapla_ik, IKSonucu
from vision.measurement.one_euro_filter import OneEuroFilter
from vision.measurement.safety_trajectory import OutlierGuard, eklem_yorungesi_uret
from vision.measurement.base_locator import RobotTabanBulucu

logger = logging.getLogger(__name__)

class RobotDurum(enum.Enum):
    """Robot kolunun mevcut durumu."""
    BEKLEME = "BEKLEME"
    TESPIT = "TESPIT"
    YAKLASMA = "YAKLASMA"
    HIZALAMA = "HIZALAMA"
    KAVRAMA = "KAVRAMA"
    KALDIRMA = "KALDIRMA"
    GERI = "GERI"
    BIRAKMA = "BIRAKMA"
    TAMAMLANDI = "TAMAMLANDI"
DURUM_RENKLERI = {
    RobotDurum.BEKLEME:     (200, 200, 200),
    RobotDurum.TESPIT:      (0, 255, 255),
    RobotDurum.YAKLASMA:    (0, 165, 255),
    RobotDurum.HIZALAMA:    (255, 128, 0),
    RobotDurum.KAVRAMA:     (0, 255, 0),
    RobotDurum.KALDIRMA:    (255, 200, 0),
    RobotDurum.GERI:        (255, 0, 255),
    RobotDurum.BIRAKMA:     (255, 255, 0),
    RobotDurum.TAMAMLANDI:  (0, 255, 0),
}

_son_gonderilen = None

def _servo_gonder(server_url: str, taban: int, omuz: int, dirsek: int, bilek: int, tutucu: int) -> bool:
    """
    Flask sunucusuna servo açılarını gönderir.
    Gereksiz ağı ve Arduino seri portunu boğmamak için sadece açılar değiştiğinde gönderir.
    Başarılı ise True, hata ise False döner.
    """
    global _son_gonderilen
    payload = {
        "taban": int(max(0, min(180, taban))),
        "omuz": int(max(0, min(180, omuz))),
        "dirsek": int(max(0, min(180, dirsek))),
        "bilek": int(max(0, min(180, bilek))),
        "tutucu": int(max(0, min(180, tutucu))),
    }

    if _son_gonderilen == payload:
        return True

    try:
        r = requests.post(server_url, json=payload, timeout=1.0)
        if r.status_code == 200:
            _son_gonderilen = payload
            logger.debug(f"Servo OK: {payload}")
            return True
        else:
            logger.warning(f"Servo HTTP hata: {r.status_code}")
            return False
    except Exception as e:
        logger.error(f"Servo gonderim hatasi: {e}")
        return False

def _sort_corners_clockwise(points: np.ndarray) -> np.ndarray:
    """
    4 noktayı saat yönünde sıralar: sol-üst, sağ-üst, sağ-alt, sol-alt.
    (Eski yöntem — kamera açısına duyarlı, hareketli kamerada kırılgan.)
    """
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    order = np.argsort(angles)
    return points[order]


def _sort_corners_by_id(detected_corner_pts: dict, config: VisionConfig) -> list:
    """
    Köşe marker'larını fiziksel masadaki sabit pozisyonlarına göre sıralar.
    Kameranın açısından bağımsız, ID-bazlı sıralama.

    config.KOSE_ID_TO_POSITION sözlüğü her marker ID'sinin masadaki
    fiziksel köşesini tanımlar (sol_ust, sag_ust, sag_alt, sol_alt).

    Dönüş: [sol_ust_id, sag_ust_id, sag_alt_id, sol_alt_id] sırasında ID listesi
    """
    pozisyon_sirasi = ["sol_ust", "sag_ust", "sag_alt", "sol_alt"]
    id_to_pos = config.KOSE_ID_TO_POSITION
    pos_to_id = {v: k for k, v in id_to_pos.items()}

    sirali_idler = []
    for pos in pozisyon_sirasi:
        mid = pos_to_id.get(pos)
        if mid is not None and mid in detected_corner_pts:
            sirali_idler.append(mid)
        else:
            return None  # Eksik köşe — sıralama yapılamaz
    return sirali_idler

def _kamera_piksel_to_raw_cm(
    piksel_x: float, piksel_y: float,
    cikis_w: int, cikis_h: int,
    config: VisionConfig
) -> tuple:
    """
    Kuşbakışı (warped) görüntüdeki piksel koordinatını, kameranın tam altındaki merkeze 
    (optik eksen) göre cm cinsinden ham konuma çevirir.
    """
    px_per_cm = config.BIRDS_EYE_PX_PER_CM
    dx_px = piksel_x - (cikis_w / 2.0)
    dy_px = piksel_y - (cikis_h / 2.0)
    kamera_x_cm = dx_px / px_per_cm
    kamera_y_cm = -dy_px / px_per_cm
    
    if config.KAMERA_ROTASYON == 1:
        kamera_x_cm, kamera_y_cm = kamera_y_cm, -kamera_x_cm
    elif config.KAMERA_ROTASYON == 2:
        kamera_x_cm, kamera_y_cm = -kamera_x_cm, -kamera_y_cm
    elif config.KAMERA_ROTASYON == 3:
        kamera_x_cm, kamera_y_cm = -kamera_y_cm, kamera_x_cm
        
    return kamera_x_cm, kamera_y_cm

def grab_mode(config: VisionConfig) -> None:
    """
    Cisim Yakalama (Grab) modu.

    Kamerayı açar, masadaki köşe marker'lardan kuşbakışı görünüm oluşturur,
    hedef nesneyi tespit eder ve durum makinesi ile cismi tutup kaldırır.

    Çıkış: 'q' veya ESC tuşu
    """
    calib_data = load_calibration(config.CALIBRATION_FILE)
    if not calib_data:
        raise RuntimeError(
            "Kalibrasyon bulunamadı! Önce 'python -m vision.main --mode calibrate' çalıştırın."
        )

    camera_matrix, dist_coeffs = calib_data
    dictionary = cv2.aruco.getPredefinedDictionary(config.DICTIONARY)
    params = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(dictionary, params)

    # --- Hedefler (Küp vb.) için 5x5 Sözlük Dedektörü (Fallback/Multi-Dict) ---
    dict_5x5 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    aruco_detector_5x5 = cv2.aruco.ArucoDetector(dict_5x5, params)

    logger.info(f"Kalibrasyon yüklendi: {config.CALIBRATION_FILE}")
    cap = cv2.VideoCapture(config.KAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"HATA: Kamera açılamadı (index={config.KAMERA_INDEX}).")
    durum = RobotDurum.BEKLEME
    durum_zamani = time.time()
    cikis_w = None
    cikis_h = None
    matris = None
    kaynak_noktalar = None
    hedef_x_cm = None
    hedef_y_cm = None
    hedef_id = None
    sabit_kare_sayaci = 0
    onceki_hedef_pos = None
    PARK_TABAN = 90
    PARK_OMUZ = 45
    PARK_DIRSEK = 120
    PARK_BILEK = 90
    PARK_TUTUCU = config.GRIPPER_ACIK_ACI
    _servo_gonder(config.SERVER_URL, PARK_TABAN, PARK_OMUZ, PARK_DIRSEK, PARK_BILEK, PARK_TUTUCU)
    son_ik: IKSonucu = None

    # --- Katman 1: Kayıp köşe sayacı ---
    kayip_kare_sayaci = 0

    # --- Katman 2: 1Euro Filter nesneleri ---
    filtre_x = OneEuroFilter(t0=time.time(), x0=0.0,
                             min_cutoff=config.ONEEURO_MIN_CUTOFF,
                             beta=config.ONEEURO_BETA)
    filtre_y = OneEuroFilter(t0=time.time(), x0=0.0,
                             min_cutoff=config.ONEEURO_MIN_CUTOFF,
                             beta=config.ONEEURO_BETA)

    # --- Katman 3: Outlier Guard + Spline yörünge değişkenleri ---
    outlier_guard = OutlierGuard(
        window_size=config.OUTLIER_PENCERE_BOYUTU,
        esik_cm=config.OUTLIER_ESIK_CM,
        ardisik_kabul_esik=config.OUTLIER_ARDISIK_KABUL_ESIK
    )
    
    # --- Yeni: Bilek Filtreleri ---
    bilek_outlier_guard = OutlierGuard(
        window_size=config.BILEK_OUTLIER_PENCERE_BOYUTU,
        esik_cm=config.BILEK_OUTLIER_ESIK_CM,
        ardisik_kabul_esik=config.BILEK_OUTLIER_ARDISIK_KABUL_ESIK
    )
    filtre_bilek_x = OneEuroFilter(
        t0=time.time(), x0=0.0,
        min_cutoff=config.BILEK_ONEEURO_MIN_CUTOFF,
        beta=config.BILEK_ONEEURO_BETA
    )
    filtre_bilek_y = OneEuroFilter(
        t0=time.time(), x0=0.0,
        min_cutoff=config.BILEK_ONEEURO_MIN_CUTOFF,
        beta=config.BILEK_ONEEURO_BETA
    )

    # --- Yeni: Taban Bulucu ---
    robot_taban_bulucu = RobotTabanBulucu(
        config.TABAN_MARKER_YEREL_OFSET_X_CM,
        config.TABAN_MARKER_YEREL_OFSET_Y_CM,
        config.TABAN_KILITLEME_KARE,
        config.TABAN_KAYIP_MAX_KARE,
    )
    yaklasma_yorunge = None
    yaklasma_adim = 0

    logger.info("═" * 50)
    logger.info("  CİSİM YAKALAMA MODU BAŞLADI")
    logger.info("  Çıkmak için 'q' veya 'ESC' tuşuna basın.")
    logger.info("═" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        debug_veri = {}
        warped = None
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        display = frame.copy()
        h_frame, w_frame = display.shape[:2]
        corners1, ids1, rejected = aruco_detector.detectMarkers(frame)
        corners2, ids2, _ = aruco_detector_5x5.detectMarkers(frame)

        # İki farklı sözlükten (4x4 ve 5x5) gelen sonuçları birleştir
        tum_corners = []
        tum_ids = []
        if ids1 is not None:
            tum_corners.extend(corners1)
            tum_ids.extend(ids1.flatten())
        if ids2 is not None:
            tum_corners.extend(corners2)
            tum_ids.extend(ids2.flatten())
        
        if len(tum_ids) > 0:
            corners = tuple(tum_corners)
            ids = np.array(tum_ids).reshape(-1, 1)
        else:
            ids = None

        detected_corner_pts = {}
        detected_corner_raw = {}
        detected_hedef = {}
        detected_base = None  # Dinamik robot taban (ofset) hesaplaması için
        detected_bilek = None # Gripper takibi için (Hizalama)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                c = corners[i][0]
                center = np.mean(c, axis=0)
                
                # Ekranda GÖRÜNEN her markerın ID'sini kırmızı ve kalın olarak yazdır (Debug için)
                cv2.putText(display, f"ID: {marker_id}", (int(center[0]) - 20, int(center[1]) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                if marker_id in config.KOSE_MARKER_IDLERI:
                    detected_corner_pts[marker_id] = center
                    detected_corner_raw[marker_id] = c
                    # Düzeltme 5: Köşe etiketine pozisyon adını da yaz
                    pos_adi = config.KOSE_ID_TO_POSITION.get(marker_id, "?")
                    cv2.circle(display, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)
                    cv2.putText(display, f"K{marker_id} ({pos_adi})", (int(center[0])+5, int(center[1])-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                elif marker_id in config.HEDEF_MARKER_IDS:
                    detected_hedef[marker_id] = {
                        "center": center,
                        "corners": c
                    }
                    cv2.circle(display, (int(center[0]), int(center[1])), 6, (0, 255, 255), -1)
                    cv2.putText(display, f"H{marker_id}", (int(center[0])+5, int(center[1])-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                elif marker_id == config.ROBOT_BASE_MARKER_ID:
                    detected_base = {"center": center, "corners": c}
                    cv2.circle(display, (int(center[0]), int(center[1])), 6, (255, 0, 0), -1)
                    cv2.putText(display, f"Taban (ID {marker_id})", (int(center[0])+5, int(center[1])-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

                elif marker_id == config.BILEK_MARKER_ID:
                    detected_bilek = {"center": center, "corners": c}
                    cv2.circle(display, (int(center[0]), int(center[1])), 6, (0, 255, 0), -1)
                    cv2.putText(display, f"Bilek (ID {marker_id})", (int(center[0])+5, int(center[1])-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            cv2.aruco.drawDetectedMarkers(display, corners, ids)
        if len(detected_corner_pts) == 4:
            # --- Katman 1: ID-bazlı köşe sıralama (kamera açısından bağımsız) ---
            sirali_idler = _sort_corners_by_id(detected_corner_pts, config)
            if sirali_idler is None:
                # Fallback: eski açı-bazlı sıralama
                merkezler = []
                idler = []
                for mid in detected_corner_pts:
                    merkezler.append(detected_corner_pts[mid])
                    idler.append(mid)
                merkezler_arr = np.array(merkezler)
                c_ortalama = np.mean(merkezler_arr, axis=0)
                angles = np.arctan2(merkezler_arr[:, 1] - c_ortalama[1], merkezler_arr[:, 0] - c_ortalama[0])
                order = np.argsort(angles)
                sirali_idler = [idler[idx] for idx in order]
                logger.debug("ID-bazli siralama yapilamadi, aci-bazli fallback kullaniliyor.")

            # Sıralı merkezler dizisini oluştur
            merkezler_arr = np.array([detected_corner_pts[mid] for mid in sirali_idler])

            # Kayıp sayacını sıfırla (4 köşe görünüyor)
            kayip_kare_sayaci = 0

            def get_3d_pos(marker_id):
                corners_2d = detected_corner_raw[marker_id]
                marker_boyut = config.get_marker_boyutu(marker_id)
                s = marker_boyut / 2.0
                obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
                # Düzeltme 1: IPPE_SQUARE — kare planar marker'lar için optimize edilmiş poz çözümü
                success, _, tvec = cv2.solvePnP(
                    obj_pts, corners_2d, camera_matrix, dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                return tvec.flatten() if success else None

            tvec_TL = get_3d_pos(sirali_idler[0])
            tvec_TR = get_3d_pos(sirali_idler[1])
            tvec_BL = get_3d_pos(sirali_idler[3])

            if tvec_TL is not None and tvec_TR is not None and tvec_BL is not None:
                raw_w = np.linalg.norm(tvec_TL - tvec_TR) - config.get_marker_boyutu(sirali_idler[0])
                raw_h = np.linalg.norm(tvec_TL - tvec_BL) - config.get_marker_boyutu(sirali_idler[0])

                genislik_m = max(0.01, float(raw_w))
                yukseklik_m = max(0.01, float(raw_h))

                yeni_w = int(genislik_m * 100.0 * config.BIRDS_EYE_PX_PER_CM)
                yeni_h = int(yukseklik_m * 100.0 * config.BIRDS_EYE_PX_PER_CM)

                if cikis_w is None or abs(yeni_w - cikis_w) > 15 or abs(yeni_h - cikis_h) > 15:
                    cikis_w, cikis_h = yeni_w, yeni_h
                    logger.info(f"Masa Boyutu: {genislik_m*100:.1f} cm x {yukseklik_m*100:.1f} cm")

            if cikis_w is not None:
                hedef_noktalar = np.array([
                    [0, 0], [cikis_w, 0], [cikis_w, cikis_h], [0, cikis_h]
                ], dtype=np.float32)
                tum_orta = np.mean(merkezler_arr, axis=0)
                # İç köşeleri ID sırasına göre topla
                inner_corners = []
                for mid in sirali_idler:
                    marker_4_kose = detected_corner_raw[mid]
                    mesafeler = np.linalg.norm(marker_4_kose - tum_orta, axis=1)
                    en_yakin_idx = np.argmin(mesafeler)
                    inner_corners.append(marker_4_kose[en_yakin_idx])

                kaynak_noktalar = np.array(inner_corners, dtype=np.float32)
                matris = cv2.getPerspectiveTransform(kaynak_noktalar, hedef_noktalar)
        else:
            # --- Katman 1: Kayıp köşe kare sayacı ---
            kayip_kare_sayaci += 1
            if kayip_kare_sayaci > config.KOSE_KAYIP_MAX_KARE:
                if matris is not None:
                    matris = None
                    logger.warning(f"Köşe marker'lar {config.KOSE_KAYIP_MAX_KARE} karedir görünmüyor! "
                                   f"Kamerayı düzeltin. Homografi sıfırlandı.")
                    if durum not in (RobotDurum.BEKLEME, RobotDurum.TAMAMLANDI):
                        durum = RobotDurum.BEKLEME
                        durum_zamani = time.time()
                        logger.warning("Robot BEKLEME durumuna zorlandı (köşe kaybı).")

        mevcut_hedef_x = None
        mevcut_hedef_y = None
        mevcut_hedef_id = None
        mevcut_hedef_aci = 90.0

        if matris is not None and cikis_w is not None and len(detected_hedef) > 0:
            for mid, data_dict in detected_hedef.items():
                center_px = data_dict["center"]
                corners_px = data_dict["corners"]
                if kaynak_noktalar is not None:
                    if cv2.pointPolygonTest(kaynak_noktalar, (float(center_px[0]), float(center_px[1])), False) < 0:
                        continue
                pts = np.array([[[float(center_px[0]), float(center_px[1])],
                                 [float(corners_px[0][0]), float(corners_px[0][1])],
                                 [float(corners_px[1][0]), float(corners_px[1][1])]]], dtype=np.float32)
                warped_pts = cv2.perspectiveTransform(pts, matris)[0]

                center_w = warped_pts[0]
                debug_veri['D1'] = center_px
                debug_veri['D2'] = center_w
                tl_w = warped_pts[1]
                tr_w = warped_pts[2]
                rx, ry = _kamera_piksel_to_raw_cm(
                    center_w[0], center_w[1], cikis_w, cikis_h, config
                )
                debug_veri['D3'] = (rx, ry)
                dx = tr_w[0] - tl_w[0]
                dy = tr_w[1] - tl_w[1]
                aci_rad = math.atan2(dy, dx)
                aci_derece = math.degrees(aci_rad)

                hesaplanan_j4 = 90.0 + aci_derece
                while hesaplanan_j4 < 0: hesaplanan_j4 += 180.0
                while hesaplanan_j4 > 180: hesaplanan_j4 -= 180.0

                mevcut_hedef_x = rx
                mevcut_hedef_y = ry
                mevcut_hedef_id = mid
                mevcut_hedef_aci = hesaplanan_j4
                break

        # --- Düzeltme 2: Outlier Guard ÖNCE çalışmalı (ham veriyi kontrol eder) ---
        if mevcut_hedef_x is not None:
            mevcut_hedef_x, mevcut_hedef_y, gecerli = outlier_guard.kontrol_et(
                mevcut_hedef_x, mevcut_hedef_y
            )
            debug_veri['D4'] = (mevcut_hedef_x, mevcut_hedef_y, gecerli)
            if not gecerli:
                logger.debug(f"Outlier elendi, son güvenli değer kullanılıyor: "
                             f"({mevcut_hedef_x:.1f}, {mevcut_hedef_y:.1f})")

        # --- 1Euro Filter SONRA çalışmalı (temizlenmiş veriyi yumuşatır) ---
        if mevcut_hedef_x is not None:
            t_simdi = time.time()
            mevcut_hedef_x = filtre_x(t_simdi, mevcut_hedef_x)
            mevcut_hedef_y = filtre_y(t_simdi, mevcut_hedef_y)
            debug_veri['D5'] = (mevcut_hedef_x, mevcut_hedef_y)

        # --- Dinamik Taban (Ofset) Hesaplaması ve Koordinat Dönüşümü ---
        taban_kilitli = False
        robot_x_cm = None
        robot_y_cm = None
        
        if matris is not None and cikis_w is not None:
            raw_base_x = None
            raw_base_y = None
            base_aci = 0.0
            if detected_base is not None:
                base_center_px = detected_base["center"]
                base_pts = np.array([[[float(base_center_px[0]), float(base_center_px[1])]]], dtype=np.float32)
                base_warped = cv2.perspectiveTransform(base_pts, matris)[0][0]
                raw_base_x, raw_base_y = _kamera_piksel_to_raw_cm(
                    base_warped[0], base_warped[1], cikis_w, cikis_h, config
                )
                
                # Açı hesabı
                c_px = detected_base["corners"]
                c_pts = np.array([[[float(c_px[0][0]), float(c_px[0][1])],
                                   [float(c_px[1][0]), float(c_px[1][1])]]], dtype=np.float32)
                c_warped = cv2.perspectiveTransform(c_pts, matris)[0]
                dx = c_warped[1][0] - c_warped[0][0]
                dy = c_warped[1][1] - c_warped[0][1]
                base_aci = math.degrees(math.atan2(dy, dx))
                
            robot_x_cm, robot_y_cm, taban_kilitli = robot_taban_bulucu.guncelle(raw_base_x, raw_base_y, base_aci)
            
            if taban_kilitli and not getattr(robot_taban_bulucu, '_loglandi', False):
                logger.info(f"Robot taban tespit edildi ve kilitlendi: ({robot_x_cm:.2f}, {robot_y_cm:.2f}) cm")
                print(f"[D6] Robot taban kilitli konum: x={robot_x_cm:.2f}, y={robot_y_cm:.2f}")
                robot_taban_bulucu._loglandi = True
            
            if robot_taban_bulucu.kayip_mi() and durum not in (RobotDurum.BEKLEME, RobotDurum.TAMAMLANDI):
                logger.warning("Taban kilitli konumu kaybedildi! BEKLEME durumuna geciliyor.")
                durum = RobotDurum.BEKLEME
                durum_zamani = time.time()

        if not taban_kilitli:
            durum = RobotDurum.BEKLEME
            # State machine için askıda kalmış olabilecek tüm değişkenleri sıfırla
            hedef_x_cm = None
            hedef_y_cm = None
            hedef_id = None
            son_ik = None
            yaklasma_yorunge = None
            yaklasma_adim = 0
            sabit_kare_sayaci = 0
            onceki_hedef_pos = None
            outlier_guard.sifirla()
            bilek_outlier_guard.sifirla()

            # Taban bulunana kadar sadece uyarıyı ekrana bas ve döngüyü yeniden başlat
            cv2.putText(display, "Robot taban konumu tespit ediliyor...", (10, h_frame - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow("Cisim Yakalama", display)
            if matris is not None and cikis_w is not None:
                warped = cv2.warpPerspective(frame, matris, (cikis_w, cikis_h))
            else:
                warped = None
                
            if warped is not None:
                cv2.imshow("Kusbakisi Gorunum", warped)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q') or key == 27:
                break
            continue

        # Hedefin, kameraya göre ham konumu hesaplandıysa:
        if mevcut_hedef_x is not None and mevcut_hedef_y is not None:
            if robot_x_cm is not None and robot_y_cm is not None:
                mevcut_hedef_x = mevcut_hedef_x - robot_x_cm
                mevcut_hedef_y = -(mevcut_hedef_y - robot_y_cm)  # Robot ileri yonu +Y
            else:
                # Taban görünmüyorsa config'deki statik (manuel) ofsetleri kullan (fallback)
                mevcut_hedef_x = mevcut_hedef_x + config.ROBOT_BASE_OFFSET_X_CM
                mevcut_hedef_y = -(mevcut_hedef_y + config.ROBOT_BASE_OFFSET_Y_CM)

        # Bilek (gripper) konumunun dinamik ofsetle hesaplanması ve Filtrelenmesi
        bilek_x_cm = None
        bilek_y_cm = None
        if detected_bilek is not None and matris is not None and cikis_w is not None:
            bilek_center_px = detected_bilek["center"]
            bilek_pts = np.array([[[float(bilek_center_px[0]), float(bilek_center_px[1])]]], dtype=np.float32)
            bilek_warped = cv2.perspectiveTransform(bilek_pts, matris)[0][0]
            raw_bilek_x, raw_bilek_y = _kamera_piksel_to_raw_cm(
                bilek_warped[0], bilek_warped[1], cikis_w, cikis_h, config
            )
            
            if raw_bilek_x is not None:
                # Glitch korumasi ONCE
                raw_bilek_x, raw_bilek_y, bilek_gecerli = bilek_outlier_guard.kontrol_et(raw_bilek_x, raw_bilek_y)
                
                # Yumusatma SONRA
                t_simdi = time.time()
                raw_bilek_x = filtre_bilek_x(t_simdi, raw_bilek_x)
                raw_bilek_y = filtre_bilek_y(t_simdi, raw_bilek_y)
                
                if robot_x_cm is not None and robot_y_cm is not None:
                    bilek_x_cm = raw_bilek_x - robot_x_cm
                    bilek_y_cm = -(raw_bilek_y - robot_y_cm) # Robot ileri yonu +Y
                else:
                    bilek_x_cm = raw_bilek_x + config.ROBOT_BASE_OFFSET_X_CM
                    bilek_y_cm = -(raw_bilek_y + config.ROBOT_BASE_OFFSET_Y_CM)
        gecen_sure = time.time() - durum_zamani

        if durum == RobotDurum.BEKLEME:
            if mevcut_hedef_x is not None and taban_kilitli:
                durum = RobotDurum.TESPIT
                durum_zamani = time.time()
                sabit_kare_sayaci = 0
                onceki_hedef_pos = (mevcut_hedef_x, mevcut_hedef_y)
                hedef_id = mevcut_hedef_id
                outlier_guard.sifirla()  # Düzeltme 4: Yeni hedefe geçerken pencereyi temizle
                bilek_outlier_guard.sifirla() # Bilek filtresi sifirlama
                logger.info(f"Hedef bulundu! ID={hedef_id}, Konum=({mevcut_hedef_x:.1f}, {mevcut_hedef_y:.1f})")

        elif durum == RobotDurum.TESPIT:
            if mevcut_hedef_x is not None and onceki_hedef_pos is not None:
                fark = math.sqrt(
                    (mevcut_hedef_x - onceki_hedef_pos[0])**2 + 
                    (mevcut_hedef_y - onceki_hedef_pos[1])**2
                )
                if fark < 2.0:
                    sabit_kare_sayaci += 1
                else:
                    sabit_kare_sayaci = 0

                onceki_hedef_pos = (mevcut_hedef_x, mevcut_hedef_y)

                if sabit_kare_sayaci >= config.KONUM_SABITLEME_KARE:
                    print(f"\n--- HEDEF SABITLENDI, DEBUG CIKTILARI ---")
                    if 'D1' in debug_veri:
                        print(f"[D1] Hedef ham piksel: {debug_veri['D1']}")
                        print(f"[D2] Hedef kusbakisi piksel: {debug_veri['D2']}")
                        print(f"[D3] Hedef HAM cm (filtresiz): x={debug_veri['D3'][0]:.2f}, y={debug_veri['D3'][1]:.2f}")
                        print(f"[D4] OutlierGuard sonrasi: x={debug_veri['D4'][0]:.2f}, y={debug_veri['D4'][1]:.2f}, gecerli={debug_veri['D4'][2]}")
                        print(f"[D5] 1Euro sonrasi (mevcut_hedef): x={debug_veri['D5'][0]:.2f}, y={debug_veri['D5'][1]:.2f}")

                    hedef_x_cm = mevcut_hedef_x
                    hedef_y_cm = mevcut_hedef_y
                    hedef_aci = mevcut_hedef_aci
                    durum = RobotDurum.YAKLASMA
                    durum_zamani = time.time()
                    logger.info(f"Konum sabitlendi! Robot koordinatı: ({hedef_x_cm:.1f}, {hedef_y_cm:.1f}) cm, Açı={hedef_aci:.1f}")
            else:
                sabit_kare_sayaci = 0
                if gecen_sure > 3.0:
                    durum = RobotDurum.BEKLEME
                    durum_zamani = time.time()
                    logger.info("Hedef kayboldu veya stabil olmadı, beklemeye dönülüyor.")

        # --- Katman 3: Spline tabanlı YAKLASMA durumu ---
        elif durum == RobotDurum.YAKLASMA:
            if yaklasma_yorunge is None:
                ik_guvenli = hesapla_ik(hedef_x_cm, hedef_y_cm, config.GUVENLI_GECIS_YUKSEKLIK_CM, config, hedef_aci)
                ik_alcak = hesapla_ik(hedef_x_cm, hedef_y_cm, config.YAKLASMA_YUKSEKLIK_CM, config, hedef_aci)
                son_ik = ik_alcak

                if ik_guvenli.erisilebilir and ik_alcak.erisilebilir:
                    baslangic = [PARK_TABAN, PARK_OMUZ, PARK_DIRSEK, PARK_BILEK]
                    hedef_1 = [ik_guvenli.j1_derece, ik_guvenli.j2_derece, ik_guvenli.j3_derece, ik_guvenli.j4_derece]
                    hedef_2 = [ik_alcak.j1_derece, ik_alcak.j2_derece, ik_alcak.j3_derece, ik_alcak.j4_derece]
                    # İki aşamayı tek spline'da birleştir: park -> güvenli yükseklik -> yaklaşma
                    yorunge_1 = eklem_yorungesi_uret(baslangic, hedef_1,
                                                     sure_sn=config.SPLINE_SURE_GUVENLI_SN,
                                                     fps=config.SPLINE_FPS)
                    yorunge_2 = eklem_yorungesi_uret(hedef_1, hedef_2,
                                                     sure_sn=config.SPLINE_SURE_ALCAK_SN,
                                                     fps=config.SPLINE_FPS)
                    yaklasma_yorunge = np.vstack([yorunge_1, yorunge_2])
                    yaklasma_adim = 0
                    logger.info(f"Spline yörünge üretildi: {len(yaklasma_yorunge)} adım, "
                                f"hedef=({hedef_x_cm:.1f}, {hedef_y_cm:.1f}) cm")
                else:
                    logger.warning(f"Hedef erişim dışı! ({hedef_x_cm:.1f}, {hedef_y_cm:.1f})")
                    durum = RobotDurum.BEKLEME
                    durum_zamani = time.time()
                    yaklasma_yorunge = None

            if yaklasma_yorunge is not None:
                if yaklasma_adim < len(yaklasma_yorunge):
                    j1, j2, j3, j4 = yaklasma_yorunge[yaklasma_adim]
                    if yaklasma_adim == 0:
                        print(f"\n--- YAKLASMA ILK ADIM DEBUG ---")
                        print(f"[D7] IK'ya giden gorece: x={hedef_x_cm:.2f}, y={hedef_y_cm:.2f}")
                        print(f"[D8] IK girdi: x={hedef_x_cm:.2f}, y={hedef_y_cm:.2f}, z={config.YAKLASMA_YUKSEKLIK_CM:.2f} | "
                              f"IK cikti: J1={son_ik.j1_derece:.1f}, J2={son_ik.j2_derece:.1f}, "
                              f"J3={son_ik.j3_derece:.1f}, J4={son_ik.j4_derece:.1f}, erisilebilir={son_ik.erisilebilir}")
                        print(f"[D9] Servoya gonderilen (offsetsiz): J1={int(round(j1))}, J2={int(round(j2))}, "
                              f"J3={int(round(j3))}, J4={int(round(j4))}")

                    _servo_gonder(config.SERVER_URL, int(round(j1)), int(round(j2)),
                                  int(round(j3)), int(round(j4)), config.GRIPPER_ACIK_ACI)
                    yaklasma_adim += 1
                else:
                    durum = RobotDurum.HIZALAMA
                    durum_zamani = time.time()
                    yaklasma_yorunge = None
                    logger.info("Yörünge tamamlandı, hizalama aşamasına geçiliyor...")

        # --- Katman 4: IBVS Hizalama durumu (Düzeltme 3: gerçek gripper takibi) ---
        elif durum == RobotDurum.HIZALAMA:
            if bilek_x_cm is not None:
                # Gerçek gripper konumu (bilek marker ID 5) ile hedef arasındaki fark
                hata_x = hedef_x_cm - bilek_x_cm
                hata_y = hedef_y_cm - bilek_y_cm
                hata_mesafe = math.sqrt(hata_x**2 + hata_y**2)
                logger.debug(f"Hizalama hatası (bilek): {hata_mesafe:.2f}cm (x={hata_x:.2f}, y={hata_y:.2f})")

                if hata_mesafe <= config.KAPALI_CEVRIM_TOLERANS_CM:
                    durum = RobotDurum.KAVRAMA
                    durum_zamani = time.time()
                    logger.info(f"Hizalama tamam (hata={hata_mesafe:.2f}cm), kavramaya geçiliyor.")
                else:
                    # Gripper'ın hedefe olan farkına göre mikro düzeltme uygula
                    duzeltilmis_x = hedef_x_cm + hata_x * config.KAPALI_CEVRIM_HIZ
                    duzeltilmis_y = hedef_y_cm + hata_y * config.KAPALI_CEVRIM_HIZ
                    ik_mikro = hesapla_ik(duzeltilmis_x, duzeltilmis_y, config.YAKLASMA_YUKSEKLIK_CM, config, hedef_aci)
                    son_ik = ik_mikro
                    if ik_mikro.erisilebilir:
                        _servo_gonder(config.SERVER_URL,
                                      int(round(ik_mikro.j1_derece)), int(round(ik_mikro.j2_derece)),
                                      int(round(ik_mikro.j3_derece)), int(round(ik_mikro.j4_derece)),
                                      config.GRIPPER_ACIK_ACI)
            else:
                # Bilek marker görünmüyor → bekleme yerine kavramaya geçiş
                if gecen_sure > 2.0:
                    logger.warning("Bilek marker (ID 5) gorunmuyor, acik cevrim "
                                   "pozisyonuyla kavramaya geciliyor (IBVS atlandi).")
                    durum = RobotDurum.KAVRAMA
                    durum_zamani = time.time()

        elif durum == RobotDurum.KAVRAMA:
            if son_ik is not None:
                _servo_gonder(
                    config.SERVER_URL,
                    taban=int(round(son_ik.j1_derece)),
                    omuz=int(round(son_ik.j2_derece)),
                    dirsek=int(round(son_ik.j3_derece)),
                    bilek=int(round(son_ik.j4_derece)),
                    tutucu=config.GRIPPER_KAPALI_ACI
                )

            if gecen_sure > config.KAVRAMA_BEKLEME_SN:
                durum = RobotDurum.KALDIRMA
                durum_zamani = time.time()
                logger.info("Cisim kavrandı, kaldırılıyor...")

        elif durum == RobotDurum.KALDIRMA:
            kaldirma_z = config.KALDIRMA_YUKSEKLIK_CM
            ik_kaldirma = hesapla_ik(hedef_x_cm, hedef_y_cm, kaldirma_z, config, hedef_aci)

            if ik_kaldirma.erisilebilir:
                _servo_gonder(
                    config.SERVER_URL,
                    taban=int(round(ik_kaldirma.j1_derece)),
                    omuz=int(round(ik_kaldirma.j2_derece)),
                    dirsek=int(round(ik_kaldirma.j3_derece)),
                    bilek=int(round(ik_kaldirma.j4_derece)),
                    tutucu=config.GRIPPER_KAPALI_ACI
                )

            if gecen_sure > config.KALDIRMA_BEKLEME_SN:
                durum = RobotDurum.GERI
                durum_zamani = time.time()
                logger.info("Cisim kaldırıldı, başlangıç pozisyonuna dönülüyor...")

        elif durum == RobotDurum.GERI:
            _servo_gonder(
                config.SERVER_URL,
                PARK_TABAN, PARK_OMUZ, PARK_DIRSEK, PARK_BILEK,
                config.GRIPPER_KAPALI_ACI
            )

            if gecen_sure > config.GERI_DONUS_BEKLEME_SN:
                durum = RobotDurum.BIRAKMA
                durum_zamani = time.time()
                logger.info("Park pozisyonuna ulaşıldı, cisim bırakılıyor...")

        elif durum == RobotDurum.BIRAKMA:
            _servo_gonder(
                config.SERVER_URL,
                PARK_TABAN, PARK_OMUZ, PARK_DIRSEK, PARK_BILEK,
                config.GRIPPER_ACIK_ACI
            )

            if gecen_sure > 1.0:
                durum = RobotDurum.TAMAMLANDI
                durum_zamani = time.time()
                logger.info("═" * 50)
                logger.info(f"  İŞLEM TAMAMLANDI! Hedef ID={hedef_id}")
                logger.info("═" * 50)

        elif durum == RobotDurum.TAMAMLANDI:
            if gecen_sure > 2.0:
                durum = RobotDurum.BEKLEME
                durum_zamani = time.time()
                hedef_x_cm = None
                hedef_y_cm = None
                hedef_id = None
                son_ik = None
                yaklasma_yorunge = None
                yaklasma_adim = 0
                outlier_guard.sifirla()
                logger.info("Yeni hedef aranıyor...")
        durum_text = f"DURUM: {durum.value}"
        durum_renk = DURUM_RENKLERI.get(durum, (255, 255, 255))
        cv2.rectangle(display, (0, 0), (w_frame, 40), (30, 30, 30), -1)
        cv2.putText(display, durum_text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, durum_renk, 2)
        sure_text = f"Sure: {gecen_sure:.1f}s"
        cv2.putText(display, sure_text, (w_frame - 150, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        if hedef_x_cm is not None and hedef_y_cm is not None:
            konum_text = f"Hedef: ({hedef_x_cm:.1f}, {hedef_y_cm:.1f}) cm  ID={hedef_id}"
            cv2.rectangle(display, (0, h_frame - 35), (w_frame, h_frame), (30, 30, 30), -1)
            cv2.putText(display, konum_text, (10, h_frame - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        elif mevcut_hedef_x is not None:
            konum_text = f"Algilanan: ({mevcut_hedef_x:.1f}, {mevcut_hedef_y:.1f}) cm  ID={mevcut_hedef_id}"
            cv2.rectangle(display, (0, h_frame - 35), (w_frame, h_frame), (30, 30, 30), -1)
            cv2.putText(display, konum_text, (10, h_frame - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        if son_ik is not None:
            ik_text = f"IK: J1={son_ik.j1_derece:.0f} J2={son_ik.j2_derece:.0f} J3={son_ik.j3_derece:.0f} J4={son_ik.j4_derece:.0f}"
            cv2.putText(display, ik_text, (10, h_frame - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
        if durum == RobotDurum.TESPIT:
            progress = min(1.0, sabit_kare_sayaci / config.KONUM_SABITLEME_KARE)
            bar_w = int(200 * progress)
            cv2.rectangle(display, (10, 50), (210, 65), (100, 100, 100), 1)
            cv2.rectangle(display, (10, 50), (10 + bar_w, 65), (0, 255, 0), -1)
            cv2.putText(display, f"Sabitleme: {sabit_kare_sayaci}/{config.KONUM_SABITLEME_KARE}",
                        (220, 63), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        if kaynak_noktalar is not None:
            overlay = display.copy()
            alan_noktalari = kaynak_noktalar.astype(np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(overlay, [alan_noktalari], (0, 220, 255))
            cv2.addWeighted(overlay, 0.15, display, 0.85, 0, display)
            cv2.polylines(display, [alan_noktalari], isClosed=True, color=(0, 220, 255), thickness=2)
        if matris is not None and cikis_w is not None:
            warped = cv2.warpPerspective(frame, matris, (cikis_w, cikis_h))
        else:
            warped = None
            
        if warped is not None:
            robot_px_x = int(cikis_w / 2.0 + config.ROBOT_BASE_OFFSET_X_CM * config.BIRDS_EYE_PX_PER_CM)
            robot_px_y = int(cikis_h / 2.0 - config.ROBOT_BASE_OFFSET_Y_CM * config.BIRDS_EYE_PX_PER_CM)
            if 0 <= robot_px_x < cikis_w and 0 <= robot_px_y < cikis_h:
                cv2.drawMarker(warped, (robot_px_x, robot_px_y), (0, 0, 255), 
                               cv2.MARKER_CROSS, 20, 2)
                cv2.putText(warped, "ROBOT", (robot_px_x + 10, robot_px_y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            cv2.imshow("Kusbakisi Gorunum", warped)

        cv2.imshow("Cisim Yakalama", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q') or key == 27:
            break
    _servo_gonder(config.SERVER_URL, PARK_TABAN, PARK_OMUZ, PARK_DIRSEK, PARK_BILEK, PARK_TUTUCU)

    cap.release()
    cv2.destroyAllWindows()
    logger.info("Cisim yakalama modu kapatıldı.")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    cfg = VisionConfig()
    try:
        grab_mode(cfg)
    except KeyboardInterrupt:
        print("\nÇıkış yapıldı.")
