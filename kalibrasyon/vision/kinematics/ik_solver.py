import math
import logging
from dataclasses import dataclass
from typing import Tuple

from vision.config import VisionConfig

logger = logging.getLogger(__name__)

@dataclass
class IKSonucu:
    j1_derece: float
    j2_derece: float
    j3_derece: float
    j4_derece: float
    erisilebilir: bool

def hesapla_ik(x_cm: float, y_cm: float, z_cm: float, conf: VisionConfig = None, hedef_aci_derece: float = 90.0) -> IKSonucu:
    """
    Verilen (X, Y, Z) noktasina ulasmak icin gerekli SERVO acilarini hesaplar.

    Robot Kol Yapisi (4 DOF):
    - J1: Taban Donusu (yaw)
    - J2: Omuz (pitch)
    - J3: Dirsek (pitch)
    - J4: Bilek (ROLL) - pitch'i degistirmez! L3, L2'nin uzantisidir.

    Servo 0 Noktalari (ONDEN BAKIS perspektifi):
      J1: servo 0 = sol (+X), servo 90 = ileri (+Y), servo 180 = sag (-X)
      J2: servo 0 = one kapali (yatay ileri), servo 90 = dik yukari, servo 180 = arkaya acik
      J3: servo 0 = arkaya acik, servo 90 = duz (L1 hizasi), servo 180 = one kapali
      J4: servo 0 = saga donus, servo 90 = notr, servo 180 = sola donus

    IK, paralel linkage mimarisi kullanir (L3 daima yere paralel).
    Hedef erisim disinda olsa bile en yakin erisim noktasina yonlenir.
    """
    if conf is None:
        conf = VisionConfig()

    l1 = conf.LINK1_UZUNLUK_CM
    l2 = conf.LINK2_UZUNLUK_CM
    l3 = conf.LINK3_UZUNLUK_CM
    h = conf.TABAN_YUKSEKLIK_CM
    j1_rad = math.atan2(y_cm, x_cm)
    s_j1 = math.degrees(j1_rad)
    if y_cm < 0:
        logger.warning(f"Hedef arkada (Y<0), en yakin kenara donuluyor.")
        if x_cm >= 0:
            s_j1 = 0.0
        else:
            s_j1 = 180.0
    r = math.sqrt(x_cm**2 + y_cm**2)
    tam_erisim = True
    l2_eff = l2 + l3
    z_rel = z_cm - h

    d_sq = r**2 + z_rel**2
    d = math.sqrt(d_sq)
    cos_alpha2 = (d_sq - l1**2 - l2_eff**2) / (2 * l1 * l2_eff)

    if cos_alpha2 < -1.0 or cos_alpha2 > 1.0:
        tam_erisim = False
        logger.info(f"Hedef geometri disinda. Maksimum/Minimum erisime ayarlaniyor.")
        cos_alpha2 = max(-1.0, min(1.0, cos_alpha2))

    alpha2 = math.acos(cos_alpha2)
    alpha2 = -alpha2 
    alpha1 = math.atan2(z_rel, r) - math.atan2(l2_eff * math.sin(alpha2), l1 + l2_eff * math.cos(alpha2))
    s_j2 = math.degrees(alpha1)
    q2_abs = alpha1 + alpha2
    s_j3 = 90.0 - math.degrees(q2_abs)
    def clamp(val):
        return max(0.0, min(180.0, val))

    servo_j1 = clamp(s_j1)
    servo_j2 = clamp(s_j2)
    servo_j3 = clamp(s_j3)
    servo_j4 = clamp(hedef_aci_derece)

    logger.info(f"IK Hedef: ({x_cm:.1f}, {y_cm:.1f}, {z_cm:.1f}) cm | Tam erisim: {tam_erisim}")
    logger.info(f"IK Math: q1={math.degrees(alpha1):.1f} q2={math.degrees(alpha2):.1f}")
    logger.info(f"IK Servo: J1={servo_j1:.0f} J2={servo_j2:.0f} J3={servo_j3:.0f} J4={servo_j4:.0f}")

    return IKSonucu(
        j1_derece=round(servo_j1, 2),
        j2_derece=round(servo_j2, 2),
        j3_derece=round(servo_j3, 2),
        j4_derece=round(servo_j4, 2),
        erisilebilir=tam_erisim
    )

def hesapla_fk(j1_servo: float, j2_servo: float, j3_servo: float, j4_servo: float = None, conf: VisionConfig = None) -> Tuple[float, float, float]:
    """
    Ileri Kinematik: Verilen SERVO acilarindan ucun (X, Y, Z) noktasini hesaplar.
    """
    if conf is None:
        conf = VisionConfig()

    l1 = conf.LINK1_UZUNLUK_CM
    h = conf.TABAN_YUKSEKLIK_CM
    j1_rad = math.radians(j1_servo)
    q1 = math.radians(j2_servo)
    q2_abs = math.radians(90.0 - j3_servo)

    l2_eff = conf.LINK2_UZUNLUK_CM + conf.LINK3_UZUNLUK_CM
    r = l1 * math.cos(q1) + l2_eff * math.cos(q2_abs)
    z_cm = h + l1 * math.sin(q1) + l2_eff * math.sin(q2_abs)

    x_cm = r * math.cos(j1_rad)
    y_cm = r * math.sin(j1_rad)

    return x_cm, y_cm, z_cm

def test_ik_fk_tutarliligi():
    """
    IK formulunun dogrulugunu saglamak icin farkli noktalarda test yapar.
    """
    import logging
    logging.disable(logging.CRITICAL)

    conf = VisionConfig()
    z = getattr(conf, 'YAKLASMA_YUKSEKLIK_CM', 8.0)

    test_points = [
        (0.0, 20.0, z),
        (0.0, 30.0, z),
        (0.0, 40.0, z),
        (0.0, 45.0, z),
        (0.0, 50.0, z),
        (0.0, 55.0, z),
        (15.0, 40.0, z),
        (-15.0, 40.0, z),
        (20.0, 35.0, z),
        (0.0, 45.0, 10.0),
    ]

    print("--- IK/FK Tutarlilik Testi ---")
    print(f"L1={conf.LINK1_UZUNLUK_CM}, L2_eff={conf.LINK2_UZUNLUK_CM + conf.LINK3_UZUNLUK_CM}, H={conf.TABAN_YUKSEKLIK_CM}, Z={z}")
    basarili = 0
    toplam = 0

    for pt in test_points:
        x, y, z_val = pt
        toplam += 1
        res = hesapla_ik(x, y, z_val, conf)
        if not res.erisilebilir:
            print(f"  [{x:.0f},{y:.0f},{z_val:.0f}] -> ERISIM DISI")
            continue

        x2, y2, z2 = hesapla_fk(res.j1_derece, res.j2_derece, res.j3_derece, res.j4_derece, conf)
        hata = math.sqrt((x-x2)**2 + (y-y2)**2 + (z_val-z2)**2)

        ok = hata < 0.5
        if ok:
            basarili += 1

        status = "OK" if ok else "HATA"
        print(f"  [{x:.0f},{y:.0f},{z_val:.0f}] -> J1:{res.j1_derece:.0f} J2:{res.j2_derece:.0f} J3:{res.j3_derece:.0f} -> FK[{x2:.1f},{y2:.1f},{z2:.1f}] err={hata:.2f}cm {status}")

    print(f"\nSonuc: {basarili}/{toplam}")

if __name__ == "__main__":
    test_ik_fk_tutarliligi()
