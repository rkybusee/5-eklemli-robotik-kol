"""
Bu dosya, tüm görüntü işleme ve kalibrasyon işlemlerinin ana kontrol merkezidir (CLI).
Kullanıcının konsoldan girdiği modlara (--mode capture, calibrate, vb.) göre ilgili alt fonksiyonları çalıştırır.
"""
import argparse
import logging
import sys

from vision.config import VisionConfig
from vision.calibration.capture import capture_calibration_images
from vision.calibration.calibration_engine import calibrate
from vision.calibration.calibration_verifier import verify_distances
from vision.measurement.birds_eye_view import birds_eye_view
from vision.measurement.grab import grab_mode

def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format='[%(levelname)s] %(name)s: %(message)s',
        handlers=[logging.StreamHandler(sys.stdout)]
    )

def main():
    setup_logging()

    parser = argparse.ArgumentParser(description="Robotik Kol Vision (Kalibrasyon ve Olcum) Modulu")
    parser.add_argument("--mode", type=str, required=True, 
                        choices=["capture", "calibrate", "verify", 
                                 "birds-eye-view", "grab"],
                        help="Calistirilacak modu secin.")
    parser.add_argument("--force-recalibrate", action="store_true", 
                        help="Var olan kalibrasyonu gormezden gelip yeniden hesaplar (sadece 'calibrate' modu icin).")

    args = parser.parse_args()
    config = VisionConfig()

    logger = logging.getLogger("vision.main")

    if args.mode == "capture":
        logger.info("Mod: Görüntü Yakalama (Capture)")
        capture_calibration_images(config)

    elif args.mode == "calibrate":
        logger.info(f"Mod: Kalibrasyon Hesaplama (Force: {args.force_recalibrate})")
        calibrate(config, force_recalibrate=args.force_recalibrate)

    elif args.mode == "verify":
        logger.info("Mod: Kalibrasyon Doğrulama (Verify)")
        verify_distances(config)

    elif args.mode == "birds-eye-view":
        logger.info("Mod: Kuşbakışı Görünüm (Bird's-Eye View)")
        birds_eye_view(config)

    elif args.mode == "grab":
        logger.info("Mod: Cisim Yakalama (Grab) — Otonom Tut ve Kaldır")
        grab_mode(config)

if __name__ == "__main__":
    main()
