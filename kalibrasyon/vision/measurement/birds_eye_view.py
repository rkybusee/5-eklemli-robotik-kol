"""
Bu dosya, masanın 4 köşesindeki ArUco marker'ları (ID: 0,1,2,3) her karede tespit ederek
gerçek zamanlı kuşbakışı (bird's-eye / top-down) görünüm penceresi oluşturur.
Her karede perspektif dönüşümü yeniden hesaplanır; böylece marker'lar hareket ettiğinde
görünüm otomatik güncellenir.
"""
import cv2
import numpy as np
import logging
import math
import requests
import time
import threading
from vision.config import VisionConfig
from vision.core.utils import load_calibration
from vision.kinematics.ik_solver import hesapla_ik

logger = logging.getLogger(__name__)

class HareketPlanlayici:
    """
    Robotik kol icin Kapalı Çevrim (Visual Servoing) uyumlu hareket planlayicisi.
    Arka planda calisan bir thread ile mevcut konumu yavas yavas hedefe dogru goturur.
    """

    def __init__(self, server_url: str, update_ms: int = 40):
        self.server_url = server_url
        self.update_sn = update_ms / 1000.0
        self.mevcut = {"taban": 90.0, "omuz": 45.0, "dirsek": 90.0, "bilek": 90.0, "tutucu": 32.0}
        self.hedef = self.mevcut.copy()

        self.aktif = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._kontrol_dongusu, daemon=True)
        self._thread.start()

    def hedefe_git(self, taban: float, omuz: float, dirsek: float, bilek: float, tutucu: float):
        """Kameradan gelen anlik hedefi gunceller."""
        with self._lock:
            self.hedef = {
                "taban": taban,
                "omuz": omuz,
                "dirsek": dirsek,
                "bilek": bilek,
                "tutucu": tutucu
            }

    def durdur(self):
        self.aktif = False

    def _gonder(self, payload: dict):
        try:
            r = requests.post(self.server_url, json=payload, timeout=0.2)
            if r.status_code != 200:
                logger.debug(f"HTTP Err: {r.status_code}")
        except Exception as e:
            logger.debug(f"Server erisim hatasi: {e}")

    def _kontrol_dongusu(self):
        ilk_gonderim_yapildi = False
        while self.aktif:
            try:
                degisim_var = False
                gonderilecek = {}

                with self._lock:
                    for eksen in ["taban", "omuz", "dirsek", "bilek", "tutucu"]:
                        fark = self.hedef[eksen] - self.mevcut[eksen]

                        if not ilk_gonderim_yapildi:
                            degisim_var = True
                            gonderilecek[eksen] = int(round(self.mevcut[eksen]))
                            continue
                        if abs(fark) < 0.5:
                            gonderilecek[eksen] = int(round(self.mevcut[eksen]))
                            continue
                        max_adim = 2.0
                        if eksen == "tutucu":
                            max_adim = 5.0

                        adim = max(-max_adim, min(max_adim, fark))
                        self.mevcut[eksen] += adim
                        gonderilecek[eksen] = int(round(self.mevcut[eksen]))
                        degisim_var = True

                if degisim_var:
                    self._gonder(gonderilecek)
                    ilk_gonderim_yapildi = True

            except Exception as e:
                logger.error(f"Kontrol dongusu hatasi: {e}")

            time.sleep(self.update_sn)

def _sort_corners_clockwise(points: np.ndarray) -> np.ndarray:
    """
    4 noktayı saat yönünde sıralar: sol-üst, sağ-üst, sağ-alt, sol-alt.

    Yöntem:
    1. 4 noktanın ortalama merkezini hesapla.
    2. Her noktanın merkeze göre açısını np.arctan2 ile bul.
    3. Açıya göre sırala — arctan2 sol-üstten başlayıp saat yönünde
       dolaşacak şekilde dönüştürülür.
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
    dx_px = piksel_x - (cikis_w / 2.0)
    dy_px = piksel_y - (cikis_h / 2.0)

    kamera_x_cm = dx_px / config.BIRDS_EYE_PX_PER_CM
    kamera_y_cm = -dy_px / config.BIRDS_EYE_PX_PER_CM 

    if config.KAMERA_ROTASYON == 1:
        kamera_x_cm, kamera_y_cm = kamera_y_cm, -kamera_x_cm
    elif config.KAMERA_ROTASYON == 2:
        kamera_x_cm, kamera_y_cm = -kamera_x_cm, -kamera_y_cm
    elif config.KAMERA_ROTASYON == 3:
        kamera_x_cm, kamera_y_cm = -kamera_y_cm, kamera_x_cm

    robot_x_cm = kamera_x_cm + config.ROBOT_BASE_OFFSET_X_CM
    robot_y_cm = kamera_y_cm + config.ROBOT_BASE_OFFSET_Y_CM
    return robot_x_cm, robot_y_cm

def birds_eye_view(config: VisionConfig) -> None:
    """

    Akış:
    1. calibration_result.yaml'dan camera_matrix ve dist_coeffs yüklenir
    2. Her karede DICT_5X5_100 ile 4 köşe marker aranır
    3. 4'ü de bulunduysa → noktalar saat yönünde sıralanır
    4. getPerspectiveTransform + warpPerspective ile düzleştirilmiş görüntü üretilir
    5. "Kusbakisi Gorunum" penceresinde gösterilir
    6. 4'ü bulunamazsa → orijinal görüntü gösterilir, uyarı yazılır
    7. 'q' veya ESC ile çıkış
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
    cikis_w = None
    cikis_h = None
    hedef_noktalar = None
    kaynak_noktalar = None
    matris = None

    ik_durum_text = ""
    hareket_durumu_text = ""
    hedef_bulundu = False
    bilek_bulundu = False
    yaklasma_basladi = False
    planlayici = HareketPlanlayici(config.SERVER_URL, update_ms=40)
    cap = cv2.VideoCapture(config.KAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"HATA: Kamera açılamadı (index={config.KAMERA_INDEX}).")

    logger.info("Kapalı Çevrim (Visual Servoing) Modu Başladı!")
    logger.info("Çıkmak için 'q' veya 'ESC' tuşuna basın.")

    son_bilinen_hedef_cm = None
    son_bilinen_aci = 90.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        display = frame.copy()
        h_frame, w_frame = display.shape[:2]
        corners, ids, rejected = aruco_detector.detectMarkers(frame)

        detected_corner_raw = {}
        detected_corner_pts = {}
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                c = corners[i][0]
                detected_corner_pts[marker_id] = np.mean(c, axis=0)
                detected_corner_raw[marker_id] = c
        dynamic_corner_ids = set()
        if ids is not None:
            for marker_id in ids.flatten():
                if marker_id in config.KOSE_MARKER_IDLERI:
                    dynamic_corner_ids.add(marker_id)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in dynamic_corner_ids:
                    center = detected_corner_pts[marker_id]
                    cv2.circle(display, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
                    cv2.putText(display, f"Kose {marker_id}", (int(center[0]), int(center[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            cv2.aruco.drawDetectedMarkers(display, corners, ids)

        if len(dynamic_corner_ids) == 4:
            merkezler = []
            idler = []
            for mid in dynamic_corner_ids:
                merkezler.append(detected_corner_pts[mid])
                idler.append(mid)
            merkezler = np.array(merkezler)
            c_ortalama = np.mean(merkezler, axis=0)
            angles = np.arctan2(merkezler[:, 1] - c_ortalama[1], merkezler[:, 0] - c_ortalama[0])
            order = np.argsort(angles)
            sirali_idler = [idler[idx] for idx in order]
            def get_3d_pos(marker_id):
                corners_2d = detected_corner_raw[marker_id]
                marker_boyut = config.get_marker_boyutu(marker_id)
                s = marker_boyut / 2.0
                obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
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
                    logger.info(f"Masa Boyutu Otomatik Güncellendi: {genislik_m*100:.1f} cm x {yukseklik_m*100:.1f} cm")

                    hedef_noktalar = np.array([
                        [0, 0],
                        [cikis_w, 0],
                        [cikis_w, cikis_h],
                        [0, cikis_h]
                    ], dtype=np.float32)
            if cikis_w is None:
                continue
            tum_orta = np.mean(merkezler, axis=0)

            inner_corners = []
            for mid in dynamic_corner_ids:
                marker_4_kose = detected_corner_raw[mid]
                mesafeler = np.linalg.norm(marker_4_kose - tum_orta, axis=1)
                en_yakin_idx = np.argmin(mesafeler)
                inner_corners.append(marker_4_kose[en_yakin_idx])
                pt = marker_4_kose[en_yakin_idx]
                cv2.circle(display, (int(pt[0]), int(pt[1])), 7, (0, 255, 0), 2)

            kaynak_raw = np.array(inner_corners, dtype=np.float32)

            kaynak_noktalar = _sort_corners_clockwise(kaynak_raw)
            matris = cv2.getPerspectiveTransform(kaynak_noktalar, hedef_noktalar)

        if matris is not None:
            bilek_cx = None
            bilek_cy = None
            hedef_cx = None
            hedef_cy = None
            hedef_id = None
            hedef_corners = None

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == config.BILEK_MARKER_ID:
                        marker_corners = corners[i][0]
                        bilek_cx = int(np.mean(marker_corners[:, 0]))
                        bilek_cy = int(np.mean(marker_corners[:, 1]))
                        cv2.circle(display, (bilek_cx, bilek_cy), 8, (0, 255, 0), -1)
                        cv2.putText(display, "Bilek (ID 1)", (bilek_cx+10, bilek_cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                    elif marker_id in config.HEDEF_MARKER_IDS:
                        marker_corners = corners[i][0]
                        cx = int(np.mean(marker_corners[:, 0]))
                        cy = int(np.mean(marker_corners[:, 1]))
                        if cv2.pointPolygonTest(kaynak_noktalar, (float(cx), float(cy)), False) >= 0:
                            hedef_cx = cx
                            hedef_cy = cy
                            hedef_id = marker_id
                            hedef_corners = marker_corners
                            cv2.circle(display, (hedef_cx, hedef_cy), 8, (0, 255, 255), -1)
                            cv2.putText(display, f"Hedef (ID {hedef_id})", (hedef_cx+10, hedef_cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
            hedef_bulundu = (hedef_cx is not None)
            bilek_bulundu = (bilek_cx is not None)

            if hedef_bulundu:
                pts = np.array([[[float(hedef_cx), float(hedef_cy)],
                                 [float(hedef_corners[0][0]), float(hedef_corners[0][1])],
                                 [float(hedef_corners[1][0]), float(hedef_corners[1][1])]]], dtype=np.float32)
                hedef_warped = cv2.perspectiveTransform(pts, matris)[0]
                h_wx, h_wy = hedef_warped[0][0], hedef_warped[0][1]
                tl_wx, tl_wy = hedef_warped[1][0], hedef_warped[1][1]
                tr_wx, tr_wy = hedef_warped[2][0], hedef_warped[2][1]
                hedef_x_cm, hedef_y_cm = _kamera_piksel_to_robot_cm(h_wx, h_wy, cikis_w, cikis_h, config)
                dx = tr_wx - tl_wx
                dy = tr_wy - tl_wy
                aci_rad = math.atan2(dy, dx)
                hesaplanan_j4 = 90.0 + math.degrees(aci_rad)
                while hesaplanan_j4 < 0: hesaplanan_j4 += 180.0
                while hesaplanan_j4 > 180: hesaplanan_j4 -= 180.0
                hedef_aci_derece = hesaplanan_j4
                son_bilinen_hedef_cm = (hedef_x_cm, hedef_y_cm)
                son_bilinen_aci = hedef_aci_derece
            elif son_bilinen_hedef_cm is not None:
                hedef_x_cm, hedef_y_cm = son_bilinen_hedef_cm
                hedef_aci_derece = son_bilinen_aci
                hedef_bulundu = True

            if hedef_bulundu and hedef_cx is not None and hedef_cy is not None:
                mesafe_robot = math.sqrt(hedef_x_cm**2 + hedef_y_cm**2)
                bilgi_yazisi = f"X:{hedef_x_cm:.1f} Y:{hedef_y_cm:.1f} Aci:{hedef_aci_derece:.0f} ({mesafe_robot:.1f}cm uzak)"
                cv2.putText(display, bilgi_yazisi, (hedef_cx - 60, hedef_cy - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if hedef_bulundu:
                robot_z_cm = getattr(config, 'YAKLASMA_YUKSEKLIK_CM', 6.5)
                ik_gosterim = hesapla_ik(hedef_x_cm, hedef_y_cm, robot_z_cm, config, hedef_aci_derece)
                h_frame, w_frame = display.shape[:2]
                if ik_gosterim.erisilebilir:
                    ik_yazi = f"IK -> J1:{ik_gosterim.j1_derece:.0f} J2:{ik_gosterim.j2_derece:.0f} J3:{ik_gosterim.j3_derece:.0f} J4:{ik_gosterim.j4_derece:.0f}"
                    cv2.putText(display, ik_yazi, (10, h_frame - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                else:
                    cv2.putText(display, "IK -> ERISILEMEZ (Cok Uzak)!", (10, h_frame - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                if not bilek_bulundu:
                    ik_res = hesapla_ik(hedef_x_cm, hedef_y_cm, robot_z_cm, config, hedef_aci_derece)
                    if ik_res.erisilebilir:
                        planlayici.hedefe_git(
                            taban=ik_res.j1_derece,
                            omuz=ik_res.j2_derece,
                            dirsek=ik_res.j3_derece,
                            bilek=ik_res.j4_derece,
                            tutucu=config.GRIPPER_ACIK_ACI
                        )
                        ik_durum_text = f"Hedefe Yaklasiliyor (Bilek Gorunmuyor) J1:{ik_res.j1_derece:.0f}"
                else:
                    pts_b = np.array([[[float(bilek_cx), float(bilek_cy)]]], dtype=np.float32)
                    bilek_warped = cv2.perspectiveTransform(pts_b, matris)[0][0]
                    b_wx, b_wy = bilek_warped[0], bilek_warped[1]

                    bilek_x_cm, bilek_y_cm = _kamera_piksel_to_robot_cm(b_wx, b_wy, cikis_w, cikis_h, config)
                    err_x = hedef_x_cm - bilek_x_cm
                    err_y = hedef_y_cm - bilek_y_cm
                    mesafe = math.sqrt(err_x**2 + err_y**2)
                    sanal_hedef_x = hedef_x_cm + (err_x * config.KAPALI_CEVRIM_HIZ)
                    sanal_hedef_y = hedef_y_cm + (err_y * config.KAPALI_CEVRIM_HIZ)

                    ik_res = hesapla_ik(sanal_hedef_x, sanal_hedef_y, robot_z_cm, config, hedef_aci_derece)

                    if ik_res.erisilebilir:
                        if mesafe < config.KAPALI_CEVRIM_TOLERANS_CM:
                            ik_durum_text = f"HEDEFE ULASILDI! Mesafe: {mesafe:.1f}cm (Tutucu Kapatiliyor)"
                            planlayici.hedefe_git(
                                taban=ik_res.j1_derece,
                                omuz=ik_res.j2_derece,
                                dirsek=ik_res.j3_derece,
                                bilek=ik_res.j4_derece,
                                tutucu=config.GRIPPER_KAPALI_ACI
                            )
                        else:
                            if hedef_cx is None:
                                ik_durum_text = f"HAFIZADAN Takip (Hata: {mesafe:.1f}cm) X:{err_x:.1f} Y:{err_y:.1f}"
                            else:
                                ik_durum_text = f"Takip (Hata: {mesafe:.1f}cm) X_err:{err_x:.1f} Y_err:{err_y:.1f}"

                            planlayici.hedefe_git(
                                taban=ik_res.j1_derece,
                                omuz=ik_res.j2_derece,
                                dirsek=ik_res.j3_derece,
                                bilek=ik_res.j4_derece,
                                tutucu=config.GRIPPER_ACIK_ACI
                            )
                    else:
                        ik_durum_text = f"Takip (Hata: {mesafe:.1f}cm) -> HEDEF ERISIM DISI!"
                    if hedef_cx is not None:
                        cv2.line(display, (bilek_cx, bilek_cy), (hedef_cx, hedef_cy), (255, 0, 0), 2)
            else:
                ik_durum_text = "Hedef (Kup) Bulunamadi. Bekleniyor..."
            overlay = display.copy()
            alan_noktalari = kaynak_noktalar.astype(np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(overlay, [alan_noktalari], (0, 220, 255))
            cv2.addWeighted(overlay, 0.25, display, 0.75, 0, display)
            cv2.polylines(display, [alan_noktalari], isClosed=True, color=(0, 220, 255), thickness=2)
            warped = cv2.warpPerspective(frame, matris, (cikis_w, cikis_h))
            if len(dynamic_corner_ids) == 4:
                msg = "Kusbakisi OK - 4/4 kose bulundu"
                color = (0, 255, 0)
            else:
                msg = f"MASA HAFIZADA ({len(dynamic_corner_ids)}/4 kose)"
                color = (0, 165, 255)

            cv2.putText(display, msg,
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.imshow("Kusbakisi Gorunum", warped)

        else:
            uyari_text = f"Kusbakisi gorunum icin masanin 4 kosesi de gorunmeli ({len(detected_corner_pts)}/4)"
            cv2.putText(display, uyari_text,
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            logger.debug(uyari_text)
        cv2.imshow("Kamera (Orijinal)", display)
        if ik_durum_text:
            cv2.putText(display, ik_durum_text,
                        (10, h_frame - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        pass

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q') or key == 27:
            break

    planlayici.durdur()
    cap.release()
    cv2.destroyAllWindows()
    logger.info("Kuşbakışı görünüm modu kapatıldı.")
