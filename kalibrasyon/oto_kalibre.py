import cv2
import cv2.aruco as aruco
import numpy as np
import math
import sys
import os

from vision.config import VisionConfig
from vision.core.utils import load_calibration
from vision.kinematics.ik_solver import hesapla_fk

def main():
    config = VisionConfig()
    print("[1/5] Kamera açılıyor...")
    cap = cv2.VideoCapture(config.KAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("HATA: Kamera açılamadı!")
        return

    calib_data = load_calibration(config.CALIBRATION_FILE)
    if calib_data is None:
        print("HATA: vision/calibration/calibration_result.yaml bulunamadı! Kameranız kalibre edilmemiş.")
        cap.release()
        return
    mtx, dist = calib_data

    print("\n--- ROBOT AÇILARINI GİRİN ---")
    print("Robot, 4 köşe markerını kapatmayacak şekilde konumlandırıldığında o anki açıları giriniz.")
    try:
        j1 = float(input("J1 (Taban) Açısı: "))
        j2 = float(input("J2 (Omuz) Açısı: "))
        j3 = float(input("J3 (Dirsek) Açısı: "))
    except ValueError:
        print("HATA: Geçersiz sayı girdiniz.")
        return
    print(f"\n[2/5] Girilen Açılar: J1={j1}, J2={j2}, J3={j3}")
    fk_x, fk_y, fk_z = hesapla_fk(j1, j2, j3, conf=config)
    print(f"      İleri Kinematik: Robot, küpü X={fk_x:.1f} cm, Y={fk_y:.1f} cm mesafesinde tutuyor.")
    print("\n[3/5] Kameradan görüntü alınıyor... (Ekranda 5 marker görünene kadar bekliyor)")
    print("      Lütfen kamera açısını ve ışığı kontrol edin. İptal için pencere üzerindeyken 'q' tuşuna basın.")

    aruco_dict = aruco.getPredefinedDictionary(config.DICTIONARY)
    parameters = aruco.DetectorParameters()

    frame = None
    corners = None
    ids = None

    while True:
        ret, current_frame = cap.read()
        if not ret:
            print("HATA: Kameradan görüntü okunamadı!")
            break

        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        display = current_frame.copy()
        if ids is not None:
            aruco.drawDetectedMarkers(display, corners, ids)
            cv2.putText(display, f"Bulunan Marker: {len(ids)}/5", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if len(ids)>=5 else (0,0,255), 2)
        else:
            cv2.putText(display, "Marker araniyor...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Otonom Kalibrasyon - Kamera (5 Marker Bekleniyor)", display)
        if ids is not None and len(ids) >= 5:
            id_list = ids.flatten().tolist()
            kose_tam = all(kose in id_list for kose in config.KOSE_MARKER_IDLERI)
            kup_var = any(kup in id_list for kup in [6,7,8,9])

            if kose_tam and kup_var:
                print("Başarılı! Gerekli tüm marker'lar (4 köşe + küp) bulundu.")
                frame = current_frame
                break

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            print("Kullanıcı tarafından iptal edildi.")
            break

    cv2.destroyAllWindows()
    cap.release()

    if frame is None or ids is None or len(ids) < 5:
        print("HATA: Kalibrasyon işlemi tamamlanamadı.")
        return

    ids = ids.flatten()
    print(f"Bulunan Marker ID'leri: {ids}")
    print("\n[4/5] 3D Derinlik analizi yapılıyor (solvePnP)...")

    def get_tvec(marker_id, size_m):
        if marker_id not in ids:
            return None
        idx = np.where(ids == marker_id)[0][0]
        corner_pts = corners[idx][0]
        half = size_m / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(obj_pts, corner_pts, mtx, dist)
        if success:
            return tvec.flatten() * 100.0
        return None
    t0 = get_tvec(0, config.KOSE_MARKER_BOYUTU_M)
    t2 = get_tvec(2, config.KOSE_MARKER_BOYUTU_M)
    t4 = get_tvec(4, config.KOSE_MARKER_BOYUTU_M)
    t5 = get_tvec(5, config.KOSE_MARKER_BOYUTU_M)

    if t0 is None or t2 is None or t4 is None or t5 is None:
        print("HATA: Masanın 4 köşesindeki ana markerlar (0, 2, 4, 5) tam okunamadı.")
        return
    cube_tvec = None
    for cid in [6, 7, 8, 9]:
        res = get_tvec(cid, 0.025)
        if res is not None:
            cube_tvec = res
            print(f"Küp (ID={cid}) tespit edildi.")
            break

    if cube_tvec is None:
        print("HATA: Robotun ucunda tuttuğu küp tespit edilemedi! Küpün kameraya baktığından emin olun.")
        return
    table_center = (t0 + t2 + t4 + t5) / 4.0

    left_mid = (t0 + t4) / 2.0
    right_mid = (t2 + t5) / 2.0
    bottom_mid = (t0 + t2) / 2.0
    top_mid = (t4 + t5) / 2.0

    x_axis = right_mid - left_mid
    y_axis = top_mid - bottom_mid

    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    vec_to_cube = cube_tvec - table_center
    table_x_cm = np.dot(vec_to_cube, x_axis)
    table_y_cm = np.dot(vec_to_cube, y_axis)
    kamera_hedef_x = -table_x_cm
    kamera_hedef_y = table_y_cm

    print(f"\n[5/5] Kamera Analizi: Küp, masa merkezinden X={kamera_hedef_x:.1f} cm, Y={kamera_hedef_y:.1f} cm uzakta.")
    yeni_offset_x = fk_x - kamera_hedef_x
    yeni_offset_y = fk_y - kamera_hedef_y

    print("\n================ OTONOM KALİBRASYON SONUCU ================")
    print(f"YENİ ROBOT_BASE_OFFSET_X_CM : {yeni_offset_x:.2f}")
    print(f"YENİ ROBOT_BASE_OFFSET_Y_CM : {yeni_offset_y:.2f}")
    print("===========================================================\n")
    try:
        config_path = "vision/config.py"
        with open(config_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        with open(config_path, "w", encoding="utf-8") as f:
            for line in lines:
                if "ROBOT_BASE_OFFSET_X_CM: float =" in line:
                    f.write(f"    ROBOT_BASE_OFFSET_X_CM: float = {yeni_offset_x:.2f}
                elif "ROBOT_BASE_OFFSET_Y_CM: float =" in line:
                    f.write(f"    ROBOT_BASE_OFFSET_Y_CM: float = {yeni_offset_y:.2f}
                else:
                    f.write(line)
        print("Harika! Yeni değerler config.py dosyasına otomatik olarak KAYDEDİLDİ.")
        print("Artık kuşbakışı modunu çalıştırabilirsiniz!")
    except Exception as e:
        print(f"Kaydedilemedi: {e}")

if __name__ == "__main__":
    main()
