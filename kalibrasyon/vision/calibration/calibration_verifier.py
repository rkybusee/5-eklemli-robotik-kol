"""
Bu dosya, yapılan kalibrasyonun gerçek dünya şartlarında ne kadar doğru olduğunu test eder.
ChArUco board üzerindeki komşu kareleri 3D uzayda ölçerek milimetre/yüzde bazında canlı hata oranı sunar.
"""
import cv2
import numpy as np
import logging
from vision.config import VisionConfig
from vision.core.utils import create_charuco_board, get_subpixel_criteria, load_calibration

logger = logging.getLogger(__name__)

def verify_distances(config: VisionConfig) -> None:
    """
    ChArUco board'u canlı kameradan okur.
    1. solvePnP ile board pozunu hesaplar.
    2. Komşu köşeler arası ölçülen 3D mesafeyi, bilinen KARE_UZUNLUGU_M ile karşılaştırır.
    3. Ortalama % hata ve cm cinsinden sapma ekranda canlı gösterilir.
    """
    calib_data = load_calibration(config.CALIBRATION_FILE)
    if not calib_data:
        raise RuntimeError(f"HATA: Kalibrasyon verisi ({config.CALIBRATION_FILE}) bulunamadi. Once calibrate yapin.")

    camera_matrix, dist_coeffs = calib_data
    board, detector = create_charuco_board(config)

    cap = cv2.VideoCapture(config.KAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"HATA: Kamera acilamadi (index={config.KAMERA_INDEX}).")

    logger.info("Dogrulama modu (Verify) basladi. Board'u gosterin. Cikmak icin 'ESC'ye basin.")

    son_hata_cm = None
    son_yuzde = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()
        h_frame, w_frame = display.shape[:2]

        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(frame)

        board_gorunuyor = False

        if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) >= 6:
            if marker_corners is not None and marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
            for corner in charuco_corners:
                pt = corner.flatten()
                cv2.circle(display, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)

            objp, imgp = board.matchImagePoints(charuco_corners, charuco_ids)

            if objp is not None and imgp is not None and len(objp) >= 6:
                success, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs)

                if success:
                    board_gorunuyor = True
                    R, _ = cv2.Rodrigues(rvec)
                    undistorted = cv2.undistortPoints(imgp, camera_matrix, dist_coeffs)
                    normal = R[:, 2]
                    d_plane = np.dot(normal, tvec.flatten())

                    points_3d = []
                    for i in range(len(undistorted)):
                        ray = np.array([undistorted[i][0][0], undistorted[i][0][1], 1.0])
                        t = d_plane / np.dot(normal, ray)
                        p3d = ray * t
                        points_3d.append(p3d)
                    grid_sutun = config.SUTUN_SAYISI - 1
                    ids_flat = charuco_ids.flatten()

                    mesafe_hatalari = []
                    for idx_a in range(len(ids_flat)):
                        id_a = ids_flat[idx_a]
                        row_a = id_a // grid_sutun
                        col_a = id_a % grid_sutun
                        for idx_b in range(idx_a + 1, len(ids_flat)):
                            id_b = ids_flat[idx_b]
                            row_b = id_b // grid_sutun
                            col_b = id_b % grid_sutun
                            dr = abs(row_a - row_b)
                            dc = abs(col_a - col_b)
                            if (dr == 1 and dc == 0) or (dr == 0 and dc == 1):
                                dist_3d = np.linalg.norm(points_3d[idx_a] - points_3d[idx_b])
                                hata = abs(dist_3d - config.KARE_UZUNLUGU_M)
                                mesafe_hatalari.append(hata)

                    if mesafe_hatalari:
                        son_hata_cm = np.mean(mesafe_hatalari) * 100.0
                        son_yuzde = (np.mean(mesafe_hatalari) / config.KARE_UZUNLUGU_M) * 100.0
        cv2.rectangle(display, (0, 0), (w_frame, 90), (30, 30, 30), -1)

        if board_gorunuyor and son_hata_cm is not None:
            renk = (0, 255, 0) if son_yuzde < 5.0 else (0, 165, 255) if son_yuzde < 10.0 else (0, 0, 255)
            cv2.putText(display, f"Board GORUNUYOR - Canli Dogrulama", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display, f"Ortalama Sapma: {son_hata_cm:.3f} cm  |  Yuzde Hata: %{son_yuzde:.2f}", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, renk, 2)
            cv2.putText(display, f"(Referans kare boyutu: {config.KARE_UZUNLUGU_M*100:.1f} cm)", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        elif son_hata_cm is not None:
            cv2.putText(display, "Board GORUNMUYOR - Son olcum:", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            cv2.putText(display, f"Son Sapma: {son_hata_cm:.3f} cm  |  Son Yuzde Hata: %{son_yuzde:.2f}", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            cv2.putText(display, "Board'u tekrar gosterin veya ESC ile cikin", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        else:
            cv2.putText(display, "ChArUco board'u kameraya gosterin...", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            cv2.putText(display, "Board gorunene kadar olcum yapilamaz.", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        cv2.imshow("Kalibrasyon Dogrulama (ESC: Cikis)", display)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    if son_hata_cm is not None:
        logger.info(f"Son dogrulama sonucu: Ortalama sapma {son_hata_cm:.3f} cm, Yuzde hata %{son_yuzde:.2f}")
