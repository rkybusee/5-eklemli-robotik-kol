"""
Bu dosya, diğer modüllerin sıkça ihtiyaç duyduğu ortak ve tekrar eden işlemleri barındırır.
Örneğin ChArUco board oluşturma, kalibrasyon dosyasını okuma gibi çekirdek (core) fonksiyonlar buradadır.
"""
import cv2
import numpy as np
import yaml
import os
from typing import Tuple, Dict, Any, Optional

def get_subpixel_criteria() -> Tuple[int, int, float]:
    """
    Subpixel corner refinement için kullanılacak OpenCV kısıtlama kriterlerini döndürür.
    Maksimum 100 iterasyon veya 0.001 epsilon hata hassasiyetine kadar çalışır.
    """
    return (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

def save_calibration(filepath: str, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, reproj_error: float = -1.0) -> None:
    """
    Kamera kalibrasyon verilerini .yaml formatında kaydeder.
    """
    data = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.tolist(),
        "reprojection_error": float(reproj_error)
    }
    with open(filepath, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

def load_calibration(filepath: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    .yaml veya eski .npz dosyasından kamera matrisini ve distorsiyon katsayılarını yükler.
    .yaml tercih edilir, fallback olarak .npz dener.
    """
    if filepath.endswith('.yaml') and os.path.exists(filepath):
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
        dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
        return camera_matrix, dist_coeffs
    fallback_npz = filepath.replace('.yaml', '.npz')
    if os.path.exists(fallback_npz):
        data = np.load(fallback_npz)
        return data["camera_matrix"], data["dist_coeffs"]

    return None

def create_charuco_board(config) -> Tuple[cv2.aruco.CharucoBoard, cv2.aruco.CharucoDetector]:
    """
    Config objesinden gelen ayarlara göre ChArUco board ve detector oluşturur.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(config.DICTIONARY)
    board = cv2.aruco.CharucoBoard(
        (config.SUTUN_SAYISI, config.SATIR_SAYISI), 
        config.KARE_UZUNLUGU_M, 
        config.MARKER_UZUNLUGU_M, 
        dictionary
    )
    detector = cv2.aruco.CharucoDetector(board)
    return board, detector
