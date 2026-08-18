"""
Cisim Yakalama (Grab) Modülü — Durum Makinesi ile Tam Otonom Kontrol

Kamera tepeden (kuşbakışı) dik bakarak masadaki ArUco marker'lı küpleri tespit eder,
robotik kolu IK ile yönlendirir, cismi kavrar, kaldırır ve başlangıç pozisyonuna döner.

Durum Makinesi:
    BEKLEME   → Hedef marker aranıyor
    TESPIT    → Marker bulundu, konum sabitleniyor (KONUM_SABITLEME_KARE kadar stabil olmalı)
    YAKLASMA  → IK ile cismin üzerine gidiliyor (gripper açık)
    KAVRAMA   → Gripper kapatılıyor
    KALDIRMA  → Kol yukarı kaldırılıyor
    GERI      → Başlangıç pozisyonuna dönüş
    BIRAKMA   → Gripper açılıp nesne bırakılıyor
    TAMAMLANDI → İşlem tamamlandı, yeni hedef için BEKLEME'ye dönüyor
"""
import cv2
import numpy as np
import logging
import math
import requests
import time
import enum
from vision.config import VisionConfig
from vision.core.utils import load_calibration
from vision.kinematics.ik_solver import hesapla_ik, IKSonucu

logger = logging.getLogger(__name__)

class RobotDurum(enum.Enum):
    """Robot kolunun mevcut durumu."""
    BEKLEME = "BEKLEME"
    TESPIT = "TESPIT"
    YAKLASMA = "YAKLASMA"
    KAVRAMA = "KAVRAMA"
    KALDIRMA = "KALDIRMA"
    GERI = "GERI"
    BIRAKMA = "BIRAKMA"
    TAMAMLANDI = "TAMAMLANDI"
DURUM_RENKLERI = {
    RobotDurum.BEKLEME:     (200, 200, 200),
    RobotDurum.TESPIT:      (0, 255, 255),
    RobotDurum.YAKLASMA:    (0, 165, 255),
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
    """
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    order = np.argsort(angles)
    return points[order]

def _kamera_piksel_to_robot_cm(
    piksel_x: float, piksel_y: float,
    cikis_w: int, cikis_h: int,
    config: VisionConfig
) -> tuple:
    """
    Kuşbakışı (warped) görüntüdeki piksel koordinatını robot tabanına göre cm'ye çevirir.

    Kuşbakışı görüntünün merkezi = kameranın optik ekseni = kameranın tam altındaki nokta.
    Robot tabanı bu noktadan config.ROBOT_BASE_OFFSET_X/Y_CM kadar uzakta.

    Warped görüntü koordinat sistemi:
        Piksel (0,0) = sol-üst köşe
        X ekseni = sağa doğru artar  
        Y ekseni = aşağıya doğru artar

    Robot koordinat sistemi:
        +X = sağ (kol arkasından bakınca)
        +Y = ileri (kolun baktığı yön)
        +Z = yukarı

    NOT: Kameranın fiziksel yönelimi (rotasyonu) burada kritik. Tepeden bakan kameranın
    hangi kenarının "ileri" (+Y) olduğu deneme ile belirlenip gerekirse aşağıdaki
    formülde X/Y yer değiştirilmeli veya işaret değiştirilmeli.
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
    robot_x_cm = kamera_x_cm + config.ROBOT_BASE_OFFSET_X_CM
    robot_y_cm = kamera_y_cm + config.ROBOT_BASE_OFFSET_Y_CM

    return robot_x_cm, robot_y_cm

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

    logger.info("═" * 50)
    logger.info("  CİSİM YAKALAMA MODU BAŞLADI")
    logger.info("  Çıkmak için 'q' veya 'ESC' tuşuna basın.")
    logger.info("═" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        display = frame.copy()
        h_frame, w_frame = display.shape[:2]
        corners, ids, rejected = aruco_detector.detectMarkers(frame)

        detected_corner_pts = {}
        detected_corner_raw = {}
        detected_hedef = {}

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                c = corners[i][0]
                center = np.mean(c, axis=0)

                if marker_id in config.KOSE_MARKER_IDLERI:
                    detected_corner_pts[marker_id] = center
                    detected_corner_raw[marker_id] = c
                    cv2.circle(display, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)
                    cv2.putText(display, f"K{marker_id}", (int(center[0])+5, int(center[1])-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                elif marker_id in config.HEDEF_MARKER_IDS:
                    detected_hedef[marker_id] = {
                        "center": center,
                        "corners": c
                    }
                    cv2.circle(display, (int(center[0]), int(center[1])), 6, (0, 255, 255), -1)
                    cv2.putText(display, f"H{marker_id}", (int(center[0])+5, int(center[1])-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            cv2.aruco.drawDetectedMarkers(display, corners, ids)
        if len(detected_corner_pts) == 4:
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
            def get_3d_pos(marker_id):
                corners_2d = detected_corner_raw[marker_id]
                marker_boyut = config.get_marker_boyutu(marker_id)
                s = marker_boyut / 2.0
                obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
                success, _, tvec = cv2.solvePnP(obj_pts, corners_2d, camera_matrix, dist_coeffs)
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
                inner_corners = []
                for mid in detected_corner_pts:
                    marker_4_kose = detected_corner_raw[mid]
                    mesafeler = np.linalg.norm(marker_4_kose - tum_orta, axis=1)
                    en_yakin_idx = np.argmin(mesafeler)
                    inner_corners.append(marker_4_kose[en_yakin_idx])

                kaynak_raw = np.array(inner_corners, dtype=np.float32)
                kaynak_noktalar = _sort_corners_clockwise(kaynak_raw)
                matris = cv2.getPerspectiveTransform(kaynak_noktalar, hedef_noktalar)
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
                tl_w = warped_pts[1]
                tr_w = warped_pts[2]
                rx, ry = _kamera_piksel_to_robot_cm(
                    center_w[0], center_w[1], cikis_w, cikis_h, config
                )
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
        gecen_sure = time.time() - durum_zamani

        if durum == RobotDurum.BEKLEME:
            if mevcut_hedef_x is not None:
                durum = RobotDurum.TESPIT
                durum_zamani = time.time()
                sabit_kare_sayaci = 0
                onceki_hedef_pos = (mevcut_hedef_x, mevcut_hedef_y)
                hedef_id = mevcut_hedef_id
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

        elif durum == RobotDurum.YAKLASMA:
            ik_guvenli = hesapla_ik(hedef_x_cm, hedef_y_cm, config.GUVENLI_GECIS_YUKSEKLIK_CM, config, hedef_aci)
            ik_alçak = hesapla_ik(hedef_x_cm, hedef_y_cm, config.YAKLASMA_YUKSEKLIK_CM, config, hedef_aci)
            son_ik = ik_alçak

            if ik_guvenli.erisilebilir and ik_alçak.erisilebilir:
                hedef_j1 = ik_guvenli.j1_derece
                j1_farki = abs(hedef_j1 - ik_guvenli.j1_derece)
                if gecen_sure < 1.8:
                    _servo_gonder(
                        config.SERVER_URL,
                        taban=int(round(hedef_j1)),
                        omuz=int(round(ik_guvenli.j2_derece)),
                        dirsek=int(round(ik_guvenli.j3_derece)),
                        bilek=int(round(ik_guvenli.j4_derece)),
                        tutucu=config.GRIPPER_ACIK_ACI
                    )
                    if gecen_sure < 0.1:
                        logger.info(f"Aşama 1 - Güvenli dönüş: J1={hedef_j1:.0f}° (güvenli Z={config.GUVENLI_GECIS_YUKSEKLIK_CM}cm)")
                else:
                    _servo_gonder(
                        config.SERVER_URL,
                        taban=int(round(ik_alçak.j1_derece)),
                        omuz=int(round(ik_alçak.j2_derece)),
                        dirsek=int(round(ik_alçak.j3_derece)),
                        bilek=int(round(ik_alçak.j4_derece)),
                        tutucu=config.GRIPPER_ACIK_ACI
                    )
                    if gecen_sure < 1.9:
                        logger.info(f"Aşama 2 - Kol iniyor: J2={ik_alçak.j2_derece:.0f}° J3={ik_alçak.j3_derece:.0f}° (Z={config.YAKLASMA_YUKSEKLIK_CM}cm)")
                    if gecen_sure > 3.5:
                        durum = RobotDurum.KAVRAMA
                        durum_zamani = time.time()
                        logger.info("Hedefe ulaşıldı, kavrama başlıyor...")
            else:
                logger.warning(f"Hedef erişim dışı! ({hedef_x_cm:.1f}, {hedef_y_cm:.1f})")
                durum = RobotDurum.BEKLEME
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
