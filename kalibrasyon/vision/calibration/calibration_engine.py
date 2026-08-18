"""
Bu dosya, toplanan fotoğrafları analiz ederek kameranın lens bozulmasını (distorsiyon) ve iç parametrelerini hesaplar.
Doğruluğu düşüren kötü fotoğrafları otomatik olarak eleyerek (outlier rejection) en saf sonucu calibration_result.yaml dosyasına kaydeder.
"""
import cv2
import os
import numpy as np
import logging
from datetime import datetime
from typing import Tuple, List, Optional
from vision.config import VisionConfig
from vision.core.utils import create_charuco_board, get_subpixel_criteria, save_calibration

logger = logging.getLogger(__name__)

def find_calibration_images(config: VisionConfig) -> List[str]:
    """Diskteki (PHOTOS_DIR) kayıtlı kalibrasyon görüntülerini bulur."""
    if not os.path.exists(config.PHOTOS_DIR):
        return []

    files = [os.path.join(config.PHOTOS_DIR, f) for f in os.listdir(config.PHOTOS_DIR) 
             if f.startswith("calib-") and f.endswith(".png")]
    return sorted(files)

def _compute_per_image_errors(objpoints: List[np.ndarray], imgpoints: List[np.ndarray], 
                              rvecs: List[np.ndarray], tvecs: List[np.ndarray], 
                              camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> List[float]:
    """Görüntü başına düşen yeniden yansıtma (reprojection) hatalarını hesaplar."""
    per_view_errors = []
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        pts1 = np.array(imgpoints[i], dtype=np.float32).reshape(-1, 2)
        pts2 = np.array(imgpoints2, dtype=np.float32).reshape(-1, 2)
        error = cv2.norm(pts1, pts2, cv2.NORM_L2) / len(pts2)
        per_view_errors.append(error)
    return per_view_errors

def calibrate(config: VisionConfig, force_recalibrate: bool = False) -> None:
    """
    Diskteki resimlerden kalibrasyon yapar ve yaml olarak kaydeder.
    Outlier rejection: Yüksek hatalı kareleri eleyip yeniden hesaplar.
    """
    if not force_recalibrate and os.path.exists(config.CALIBRATION_FILE):
        logger.info(f"Mevcut kalibrasyon bulundu ({config.CALIBRATION_FILE}). Yeniden hesaplanmiyor.")
        logger.info("Yeniden hesaplamak icin: python -m vision.main --mode calibrate --force-recalibrate")
        return

    images = find_calibration_images(config)
    if not images:
        raise RuntimeError(f"HATA: '{config.PHOTOS_DIR}' dizininde goruntu bulunamadi. Once yakalama yapin.")

    logger.info(f"Kalibrasyon hesaplaniyor... Toplam {len(images)} resim incelenecek.")

    board, detector = create_charuco_board(config)
    subpix_criteria = get_subpixel_criteria()

    all_objp = []
    all_imgp = []
    image_size = None
    gecerli_sayisi = 0

    for img_path in images:
        frame = cv2.imread(img_path)
        if frame is None:
            continue
        if image_size is None:
            image_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(frame)
        kose_sayisi = 0 if charuco_corners is None else len(charuco_corners)

        if charuco_corners is not None and kose_sayisi >= config.VALID_MIN_KOSE:
            charuco_corners = cv2.cornerSubPix(gray, charuco_corners, (5, 5), (-1, -1), subpix_criteria)

            objp, imgp = board.matchImagePoints(charuco_corners, charuco_ids)
            if objp is not None and imgp is not None and len(objp) > 0:
                all_objp.append(objp)
                all_imgp.append(imgp)
                gecerli_sayisi += 1

    if gecerli_sayisi < 10:
        raise RuntimeError(f"Yetersiz gecerli resim! Bulunan: {gecerli_sayisi}. Lutfen capture ile daha fazla resim cekin.")

    logger.info(f"Kose esigini gecen gecerli resim sayisi: {gecerli_sayisi}/{len(images)}")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        all_objp, all_imgp, image_size, None, None
    )
    logger.info(f"Ilk kalibrasyon — Reprojection error: {ret:.4f} (kullanilan: {len(all_objp)} resim)")
    MAX_ITERASYON = 3
    for tur in range(MAX_ITERASYON):
        per_view_errors = _compute_per_image_errors(all_objp, all_imgp, rvecs, tvecs, camera_matrix, dist_coeffs)

        mean_err = np.mean(per_view_errors)
        std_err = np.std(per_view_errors)
        esik = mean_err + 1.5 * std_err
        kotu_indexler = [i for i, e in enumerate(per_view_errors) if e > esik]

        if not kotu_indexler:
            logger.info(f"  Tur {tur+1}: Elenecek kare yok, kalibrasyon stabil.")
            break
        if len(all_objp) - len(kotu_indexler) < 10:
            logger.info(f"  Tur {tur+1}: Eleme sonrasi cok az resim kalacak, durduruluyor.")
            break

        logger.info(f"  Tur {tur+1}: {len(kotu_indexler)} kotu kare eleniyor (esik: {esik:.4f}, ortalama: {mean_err:.4f})")
        for idx in sorted(kotu_indexler, reverse=True):
            logger.info(f"    Elenen resim
            del all_objp[idx]
            del all_imgp[idx]
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            all_objp, all_imgp, image_size, None, None
        )
        logger.info(f"  Tur {tur+1} sonrasi — Reprojection error: {ret:.4f} (kalan: {len(all_objp)} resim)")
    print("\n" + "="*60)
    print("       KALIBRASYON SONUCLARI")
    print("="*60)
    print(f"  Reprojection Error : {ret:.4f}")
    print(f"  Kullanilan Resim   : {len(all_objp)} / {len(images)} (toplam)")
    print(f"\n  Kamera Matrisi:")
    print(f"    fx = {camera_matrix[0,0]:.2f}")
    print(f"    fy = {camera_matrix[1,1]:.2f}")
    print(f"    cx = {camera_matrix[0,2]:.2f}")
    print(f"    cy = {camera_matrix[1,2]:.2f}")
    print(f"\n  Distorsiyon Katsayilari:")
    print(f"    {dist_coeffs.flatten().tolist()}")
    print("="*60 + "\n")
    save_calibration(config.CALIBRATION_FILE, camera_matrix, dist_coeffs, ret)
    logger.info(f"Sonuclar '{config.CALIBRATION_FILE}' dosyasina kaydedildi.")
    log_kaydet(config.LOG_FILE, len(images), len(all_objp), ret, camera_matrix, dist_coeffs)

def log_kaydet(log_file: str, toplam_yakalanan: int, gecerli_sayisi: int, reproj_error: float, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> None:
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    zaman = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    yeni_dosya = not os.path.exists(log_file)
    with open(log_file, "a", encoding="utf-8") as f:
        if yeni_dosya:
            f.write("# Kalibrasyon Geçmişi\n\n")
        f.write(f"## Kalibrasyon - {zaman}\n")
        f.write(f"- Toplam incelenen resim: {toplam_yakalanan}\n")
        f.write(f"- Kalibrasyon için kullanılan resim: {gecerli_sayisi}\n")
        f.write(f"- Reprojection error: {reproj_error:.4f}\n")
        f.write(f"- fx: {fx:.2f}, fy: {fy:.2f}, cx: {cx:.2f}, cy: {cy:.2f}\n")
        f.write(f"- Distorsiyon katsayıları: {dist_coeffs.flatten().tolist()}\n\n")

    logger.info(f"Log kaydi '{log_file}' dosyasina eklendi.")
