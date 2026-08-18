from flask import Flask, request, jsonify, render_template, session, redirect, url_for
import serial
import time
import secrets
import os
import re
import sys
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
        taban = int(data.get("taban", 90))
        omuz = int(data.get("omuz", 90))
        dirsek = int(data.get("dirsek", 90))
        bilek = int(data.get("bilek", 90))
        tutucu = int(data.get("tutucu", 0))
    except (TypeError, ValueError):
        return jsonify({"error": "Gecersiz aci degeri"}), 400
    taban_donanim = int(round(taban + config.J1_OFFSET))
    omuz_donanim = int(round(omuz + config.J2_OFFSET))
    dirsek_donanim = int(round(dirsek + config.J3_OFFSET))
    bilek_donanim = int(round(bilek + config.J4_OFFSET))
    tutucu_donanim = int(round(tutucu + config.J5_OFFSET))

    taban_final = max(0, min(180, taban_donanim))
    omuz_final = max(0, min(180, omuz_donanim))
    dirsek_final = max(0, min(180, dirsek_donanim))
    bilek_final = max(0, min(180, bilek_donanim))
    tutucu_final = max(config.GRIPPER_MIN_ACI, min(config.GRIPPER_MAX_ACI, tutucu_donanim))

    command = f"J1:{taban_final},J2:{omuz_final},J3:{dirsek_final},J4:{bilek_final},J5:{tutucu_final}\n"

    if CONNECTION_MODE == "mock":
        print(f"[MOCK] Gonderilecek komut: {command.strip()}")
        return jsonify({"status": "sent", "command": command.strip(), "arduino_response": "MOCK-OK"})

    try:
        print(f"[SERVER] Wokwi'ye yaziliyor: {command.strip()}")
        arduino.write(command.encode())
        arduino.flush()
        response = arduino.readline().decode().strip()
        print(f"[SERVER] Wokwi'den cevap: {response}")
        return jsonify({"status": "sent", "command": command.strip(), "arduino_response": response})
    except Exception as e:
        return jsonify({"error": f"Seri port hatasi: {e}"}), 500

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
    app.run(host="0.0.0.0", port=5000, debug=False)