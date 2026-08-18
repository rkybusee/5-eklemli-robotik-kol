"""
Bu dosya, kamerayı açarak kalibrasyon için gerekli olan ChArUco board fotoğraflarını toplar.
Sadece yeterince net ve kullanılabilir olan kareleri otomatik olarak camPhotos/ klasörüne kaydeder.
"""
import cv2
import os
import time
import logging
from vision.config import VisionConfig
from vision.core.utils import create_charuco_board

logger = logging.getLogger(__name__)

def capture_calibration_images(config: VisionConfig) -> None:
    """
    Kamerayı açarak kalibrasyon için gerekli görüntüleri toplar.
    Görüntüleri belirlenen klasöre kaydeder.
    """
    os.makedirs(config.PHOTOS_DIR, exist_ok=True)
    mevcut_resimler = [f for f in os.listdir(config.PHOTOS_DIR) if f.startswith("calib-") and f.endswith(".png")]
    mevcut_adet = len(mevcut_resimler)

    board, detector = create_charuco_board(config)
    cap = cv2.VideoCapture(config.KAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        raise RuntimeError(f"HATA: Kamera acilamadi (index={config.KAMERA_INDEX}).")

    yakalanan = 0
    son_yakalama_zamani = 0

    logger.info(f"Yakalama modu basladi. Görüntüler '{config.PHOTOS_DIR}' dizinine kaydedilecek.")
    logger.info("Board'u kameraya farkli aci/mesafelerde gosterin.")
    logger.info("  's' veya ESC -> yakalamayi durdur")

    while yakalanan < config.MAX_KARE_PER_TUR:
        ok, frame = cap.read()
        if not ok:
            logger.error("HATA: Kameradan goruntu alinamadi.")
            break

        image_size = (frame.shape[1], frame.shape[0])
        gosterim = frame.copy()

        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(frame)
        kose_sayisi = 0 if charuco_corners is None else len(charuco_corners)

        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(gosterim, marker_corners, marker_ids)
        if charuco_corners is not None and kose_sayisi > 0:
            for corner in charuco_corners:
                flat_corner = corner.flatten()
                x, y = int(flat_corner[0]), int(flat_corner[1])
                renk = (0, 255, 0) if kose_sayisi >= config.VALID_MIN_KOSE else (0, 165, 255)
                cv2.circle(gosterim, (x, y), 5, renk, -1)

        simdi = time.time()
        if kose_sayisi >= config.CAPTURE_MIN_KOSE and (simdi - son_yakalama_zamani) >= config.YAKALAMA_ARALIGI_SN:
            dosya_adi = os.path.join(config.PHOTOS_DIR, f"calib-{mevcut_adet + yakalanan + 1:03d}.png")
            cv2.imwrite(dosya_adi, frame)

            yakalanan += 1
            son_yakalama_zamani = simdi
            logger.info(f"Otomatik yakalandi: {dosya_adi} (kose: {kose_sayisi})")

        durum = f"Bu tur: {yakalanan}/{config.MAX_KARE_PER_TUR}  |  Disk: {mevcut_adet + yakalanan} (hedef min {config.MIN_GECERLI_TOPLAM})"
        cv2.putText(gosterim, durum, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(gosterim, f"Kose sayisi: {kose_sayisi} (gecerli icin >= {config.VALID_MIN_KOSE} gerekli)",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if (simdi - son_yakalama_zamani) < 0.4:
            cv2.putText(gosterim, "CEKILDI!", (int(image_size[0]/2) - 100, int(image_size[1]/2)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)

        cv2.imshow("Kalibrasyon - Capture ('s': bitir)", gosterim)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') or key == 27:
            logger.info("Yakalama elle durduruldu.")
            break

    cap.release()
    cv2.destroyAllWindows()
    logger.info(f"Bu turda {yakalanan} resim yakalandi.")
