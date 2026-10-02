from flask import Flask, request, jsonify, render_template, session, redirect, url_for
import serial
import time
import secrets
import os
import re
import sys
import threading
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'kalibrasyon')))
from vision.config import VisionConfig
config = VisionConfig()

app = Flask(__name__)
app.secret_key = secrets.token_hex(16)
SERIAL_PORT = "COM6"
BAUD_RATE = 9600
ACCESS_KEY = "A1C67B1290.12"
CONNECTION_MODE = "real"
WOKWI_RFC2217_HOST = "127.0.0.1"
WOKWI_RFC2217_PORT = 4000

arduino = None

# --- Kübik Spline Yörünge Durumu ---
# Mevcut servo açılarını takip eden state (donanım değerleri, offset uygulanmış)
# Başlangıçta 90° (ham) + offset uygulanmış hali
_mevcut_acilar_lock = threading.Lock()
_mevcut_acilar = {
    "taban": 90, "omuz": 90, "dirsek": 90, "bilek": 90, "tutucu": 0
}

# Spline konfigürasyonu
_spline_config = {
    "sure_sn": 0.5,   # Varsayılan geçiş süresi (slider hareketleri için kısa)
    "fps": 30,         # Adım/saniye
    "aktif": True      # Spline aktif mi?
}

# Aktif yörünge thread'ini iptal etmek için event
_yorunge_iptal = threading.Event()
_yorunge_thread = None
_yorunge_lock = threading.Lock()


def cubic_spline_interpolate(baslangic, hedef, sure_sn, fps):
    """
    Tek bir eklem için başlangıç ve bitiş hızı 0 olan kübik spline yörünge üretir.
    scipy gerektirmeden saf Python/numpy ile Hermite interpolasyonu kullanır.

    Hermite bazlı kübik polinom:
        h(t) = (2t³ - 3t² + 1) * p0 + (t³ - 2t² + t) * m0
             + (-2t³ + 3t²) * p1 + (t³ - t²) * m1

    Başlangıç hızı m0=0, bitiş hızı m1=0 olduğunda:
        h(t) = (2t³ - 3t² + 1) * p0 + (-2t³ + 3t²) * p1
             = p0 + (p1 - p0) * (3t² - 2t³)
    """
    adim_sayisi = max(2, int(sure_sn * fps))
    t = np.linspace(0, 1, adim_sayisi)
    # Hermite basis: 3t² - 2t³ (smoothstep) → hız başta 0, sonda 0
    s = 3 * t**2 - 2 * t**3
    return baslangic + (hedef - baslangic) * s


def yorunge_uret(baslangic_acilar, hedef_acilar, sure_sn, fps):
    """
    5 eklem için kübik spline yörünge üretir.

    Parametreler:
        baslangic_acilar : dict {taban, omuz, dirsek, bilek, tutucu}
        hedef_acilar     : dict {taban, omuz, dirsek, bilek, tutucu}
        sure_sn          : Geçiş süresi (saniye)
        fps              : Saniye başına adım sayısı

    Dönüş:
        list of dict — her eleman bir zaman adımındaki açıları içerir
    """
    eklem_isimleri = ["taban", "omuz", "dirsek", "bilek", "tutucu"]
    adim_sayisi = max(2, int(sure_sn * fps))

    # Her eklem için spline yörüngesi üret
    yorungeler = {}
    for isim in eklem_isimleri:
        baslangic = float(baslangic_acilar[isim])
        hedef = float(hedef_acilar[isim])
        yorungeler[isim] = cubic_spline_interpolate(baslangic, hedef, sure_sn, fps)

    # Adım listesi oluştur
    adimlar = []
    for i in range(adim_sayisi):
        adim = {}
        for isim in eklem_isimleri:
            adim[isim] = float(yorungeler[isim][i])
        adimlar.append(adim)

    return adimlar


def _donanim_acilari_hesapla(acilar):
    """Ham açıları offset uygulayarak donanım değerlerine çevirir."""
    taban_donanim = int(round(acilar["taban"] + config.J1_OFFSET))
    omuz_donanim = int(round(acilar["omuz"] + config.J2_OFFSET))
    dirsek_donanim = int(round(acilar["dirsek"] + config.J3_OFFSET))
    bilek_donanim = int(round(acilar["bilek"] + config.J4_OFFSET))
    tutucu_donanim = int(round(acilar["tutucu"] + config.J5_OFFSET))

    return {
        "taban": max(0, min(180, taban_donanim)),
        "omuz": max(0, min(180, omuz_donanim)),
        "dirsek": max(0, min(180, dirsek_donanim)),
        "bilek": max(0, min(180, bilek_donanim)),
        "tutucu": max(config.GRIPPER_MIN_ACI, min(config.GRIPPER_MAX_ACI, tutucu_donanim))
    }


def _servo_gonder(donanim_acilari):
    """Donanım açılarını Arduino'ya seri port üzerinden gönderir."""
    command = (f"J1:{donanim_acilari['taban']},J2:{donanim_acilari['omuz']},"
               f"J3:{donanim_acilari['dirsek']},J4:{donanim_acilari['bilek']},"
               f"J5:{donanim_acilari['tutucu']}\n")

    if CONNECTION_MODE == "mock":
        print(f"[SPLINE] {command.strip()}")
        return "MOCK-OK"

    try:
        arduino.write(command.encode())
        arduino.flush()
        response = arduino.readline().decode().strip()
        return response
    except Exception as e:
        print(f"[SPLINE HATA] Seri port: {e}")
        return f"HATA: {e}"


def _yorunge_calistir(adimlar, fps):
    """
    Arka plan thread'inde yörünge adımlarını sırayla Arduino'ya gönderir.
    _yorunge_iptal event'i set edilirse erken durur ve mevcut pozisyonu günceller.
    """
    global _mevcut_acilar
    adim_suresi = 1.0 / fps

    for i, adim in enumerate(adimlar):
        # İptal kontrolü
        if _yorunge_iptal.is_set():
            print(f"[SPLINE] Yörünge iptal edildi (adım {i}/{len(adimlar)})")
            return

        # Donanım açılarını hesapla ve gönder
        donanim = _donanim_acilari_hesapla(adim)
        _servo_gonder(donanim)

        # Mevcut açıları güncelle
        with _mevcut_acilar_lock:
            _mevcut_acilar = adim.copy()

        # Sonraki adıma kadar bekle (son adım hariç)
        if i < len(adimlar) - 1:
            time.sleep(adim_suresi)

    print(f"[SPLINE] Yörünge tamamlandı ({len(adimlar)} adım)")


def spline_ile_gonder(hedef_acilar):
    """
    Mevcut pozisyondan hedef pozisyona kübik spline yörünge üretip arka planda çalıştırır.
    Devam eden bir yörünge varsa iptal eder ve mevcut pozisyondan yeni yörünge başlatır.
    """
    global _yorunge_thread

    with _yorunge_lock:
        # Devam eden yörüngeyi iptal et
        if _yorunge_thread is not None and _yorunge_thread.is_alive():
            _yorunge_iptal.set()
            _yorunge_thread.join(timeout=1.0)

        _yorunge_iptal.clear()

        # Mevcut açıları al
        with _mevcut_acilar_lock:
            baslangic = _mevcut_acilar.copy()

        sure_sn = _spline_config["sure_sn"]
        fps = _spline_config["fps"]

        # Hareket mesafesine göre süreyi uyarla
        max_fark = max(
            abs(hedef_acilar["taban"] - baslangic["taban"]),
            abs(hedef_acilar["omuz"] - baslangic["omuz"]),
            abs(hedef_acilar["dirsek"] - baslangic["dirsek"]),
            abs(hedef_acilar["bilek"] - baslangic["bilek"]),
            abs(hedef_acilar["tutucu"] - baslangic["tutucu"])
        )

        if max_fark < 2:
            # Çok küçük hareket — spline'a gerek yok, direkt gönder
            donanim = _donanim_acilari_hesapla(hedef_acilar)
            _servo_gonder(donanim)
            with _mevcut_acilar_lock:
                _mevcut_acilar.update(hedef_acilar)
            return 1  # 1 adım gönderildi

        # Büyük hareketlerde süreyi artır (oransal)
        dinamik_sure = max(sure_sn, sure_sn * (max_fark / 45.0))
        dinamik_sure = min(dinamik_sure, 2.5)  # Maksimum 2.5 saniye

        # Yörünge üret
        adimlar = yorunge_uret(baslangic, hedef_acilar, dinamik_sure, fps)

        print(f"[SPLINE] Yörünge başlatıldı: {len(adimlar)} adım, {dinamik_sure:.2f}s, "
              f"max_fark={max_fark:.0f}°")

        # Arka plan thread'inde çalıştır
        _yorunge_thread = threading.Thread(
            target=_yorunge_calistir,
            args=(adimlar, fps),
            daemon=True
        )
        _yorunge_thread.start()

        return len(adimlar)


def connect_serial():
    global arduino

    if CONNECTION_MODE == "mock":
        print("MOCK MODU aktif: hicbir seri baglanti kurulmuyor, komutlar sadece terminale yazilacak.")
        arduino = None
        return

    if CONNECTION_MODE == "wokwi":
        url = f"rfc2217://{WOKWI_RFC2217_HOST}:{WOKWI_RFC2217_PORT}"
        try:
            arduino = serial.serial_for_url(url, baudrate=BAUD_RATE, timeout=1)
            print(f"Wokwi simulasyonuna baglanildi: {url}")
        except Exception as e:
            print(f"UYARI: Wokwi simulasyonuna baglanilamadi ({url}): {e}")
            print("-> VS Code'da simulasyonun calisir ve sekmenin GORUNUR durumda oldugundan emin olun.")
            print("-> wokwi.toml icinde rfc2217ServerPort = 4000 satirinin oldugundan emin olun.")
            arduino = None
        return
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"Arduino'ya bağlanıldı: {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"UYARI: Arduino'ya bağlanılamadı ({SERIAL_PORT}): {e}")
        arduino = None

connect_serial()

@app.before_request
def check_access():
    if request.remote_addr in ("127.0.0.1", "::1"):
        return
    if session.get("authorized"):
        return
    key = request.args.get("key")
    if key == ACCESS_KEY:
        session["authorized"] = True
        session.permanent = True
        if request.path == "/" and request.method == "GET":
            return redirect(url_for("index"))
        return
    if request.path.startswith("/set_angles") or request.path.startswith("/status"):
        return jsonify({"error": "Yetkisiz erisim. Gecerli link ile giris yapin."}), 403
    return "Bu sayfaya erismek icin gecerli bir link gerekiyor.", 403

@app.route("/")
def index():
    return render_template("kol_kontrol.html")

@app.route("/set_angles", methods=["POST"])
def set_angles():
    if arduino is None and CONNECTION_MODE != "mock":
        return jsonify({"error": f"Baglanti yok ({CONNECTION_MODE} modu). Wokwi simulasyonunun VS Code'da acik oldugundan emin olun."}), 503

    data = request.get_json(force=True)

    try:
        hedef = {
            "taban": int(data.get("taban", 90)),
            "omuz": int(data.get("omuz", 90)),
            "dirsek": int(data.get("dirsek", 90)),
            "bilek": int(data.get("bilek", 90)),
            "tutucu": int(data.get("tutucu", 0))
        }
    except (TypeError, ValueError):
        return jsonify({"error": "Gecersiz aci degeri"}), 400

    # Spline aktifse yörünge ile gönder, değilse direkt gönder
    if _spline_config["aktif"]:
        adim_sayisi = spline_ile_gonder(hedef)
        return jsonify({
            "status": "spline_started",
            "adim_sayisi": adim_sayisi,
            "spline_aktif": True
        })
    else:
        # Eski davranış: direkt gönder
        donanim = _donanim_acilari_hesapla(hedef)
        command = (f"J1:{donanim['taban']},J2:{donanim['omuz']},"
                   f"J3:{donanim['dirsek']},J4:{donanim['bilek']},"
                   f"J5:{donanim['tutucu']}\n")

        if CONNECTION_MODE == "mock":
            print(f"[MOCK] Gonderilecek komut: {command.strip()}")
            with _mevcut_acilar_lock:
                _mevcut_acilar.update(hedef)
            return jsonify({"status": "sent", "command": command.strip(), "arduino_response": "MOCK-OK", "spline_aktif": False})

        try:
            print(f"[SERVER] Yaziliyor: {command.strip()}")
            arduino.write(command.encode())
            arduino.flush()
            response = arduino.readline().decode().strip()
            print(f"[SERVER] Cevap: {response}")
            with _mevcut_acilar_lock:
                _mevcut_acilar.update(hedef)
            return jsonify({"status": "sent", "command": command.strip(), "arduino_response": response, "spline_aktif": False})
        except Exception as e:
            return jsonify({"error": f"Seri port hatasi: {e}"}), 500

@app.route("/spline_config", methods=["GET", "POST"])
def spline_config_endpoint():
    """Spline konfigürasyonunu okur veya günceller."""
    if request.method == "GET":
        return jsonify(_spline_config)

    data = request.get_json(force=True)
    if "sure_sn" in data:
        _spline_config["sure_sn"] = max(0.1, min(5.0, float(data["sure_sn"])))
    if "fps" in data:
        _spline_config["fps"] = max(10, min(60, int(data["fps"])))
    if "aktif" in data:
        _spline_config["aktif"] = bool(data["aktif"])

    print(f"[SPLINE] Konfigürasyon güncellendi: {_spline_config}")
    return jsonify({"status": "ok", "config": _spline_config})

@app.route("/mevcut_acilar", methods=["GET"])
def mevcut_acilar_endpoint():
    """Mevcut servo açılarını döndürür."""
    with _mevcut_acilar_lock:
        return jsonify(_mevcut_acilar.copy())

@app.route("/save_gripper", methods=["POST"])
def save_gripper():
    data = request.get_json(force=True)
    try:
        angle = int(data.get("angle", 0))
    except (TypeError, ValueError):
        return jsonify({"error": "Gecersiz aci degeri"}), 400
    config_path = os.path.join(os.path.dirname(__file__), '..', 'kalibrasyon', 'vision', 'config.py')
    config_path = os.path.abspath(config_path)

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            content = f.read()
        new_content = re.sub(r'GRIPPER_KAPALI_ACI:\s*int\s*=\s*\d+', f'GRIPPER_KAPALI_ACI: int = {angle}', content)

        with open(config_path, 'w', encoding='utf-8') as f:
            f.write(new_content)

        return jsonify({"status": "success", "message": f"Kavrama açısı {angle}° olarak kaydedildi."})
    except Exception as e:
        return jsonify({"error": f"Konfigürasyon güncellenemedi: {str(e)}"}), 500

@app.route("/status", methods=["GET"])
def status():
    if CONNECTION_MODE == "mock":
        return jsonify({"connected": True, "port": "mock", "mode": "mock"})
    return jsonify({"connected": arduino is not None, "port": f"{CONNECTION_MODE}", "mode": CONNECTION_MODE})

if __name__ == "__main__":
    print(f"\nErisim linki: http://<bilgisayar-IP-adresiniz>:5000/?key={ACCESS_KEY}")
    print("İnternete acmak icin: ngrok http 5000 komutunu ayri bir terminalde calistirin.\n")
    print(f"Kübik Spline AKTIF: süre={_spline_config['sure_sn']}s, fps={_spline_config['fps']}")
    app.run(host="0.0.0.0", port=5000, debug=False)