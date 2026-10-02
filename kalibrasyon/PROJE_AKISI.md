# Proje Akış Şeması: Grab Modu Çalışma Akışı

Projenin `python -m vision.main --mode grab` komutuyla başlayan otonom çalışma akışı (görüntü işleme, ters kinematik, sunucu ve donanım kontrolü), gerçek kod tabanındaki sırasıyla aşağıda açıklanmıştır.

## 1. Başlangıç (`main.py`)
Süreç `kalibrasyon/vision/main.py` dosyasında başlar.
* **Çalışan Fonksiyon:** `main()` (Satır: 23)
* **İşlem:** Kullanıcı terminalden `--mode grab` argümanını verdiğinde, komut satırı argümanları ayrıştırılır. `args.mode == "grab"` bloğuna girildiğinde, `vision/measurement/grab.py` dosyasındaki `grab_mode(config)` fonksiyonu çağrılır (Satır: 57).
* **Geçiş:** `grab_mode` fonksiyonuna atlanır.

## 2. Grab Modu Başlatma ve Nesne Oluşturma (`grab.py`)
* **Çalışan Fonksiyon:** `grab_mode(config: VisionConfig)` (Satır: 152)
* **Girdi/Çıktı:** `config` nesnesi alır, çıktı döndürmez (sonsuz döngüde kalır).
* **İşlem:** Kamera döngüsü (`while True:`) başlamadan önce aşağıdaki yapı taşları oluşturulur:
    * **Kalibrasyon:** `load_calibration()` ile `camera_matrix` ve `dist_coeffs` yüklenir (Satır: 161).
    * **Aruco Dedektörü:** `cv2.aruco.ArucoDetector` başlatılır (Satır: 170).
    * **Durum Makinesi:** `durum = RobotDurum.BEKLEME` olarak ilklenir (Satır: 176).
    * **Katman 2 (1Euro Filter):** `filtre_x` ve `filtre_y` isimli iki `OneEuroFilter` nesnesi (X ve Y eksenleri için ayrı ayrı) oluşturulur (Satır: 198-204).
    * **Katman 3 (Outlier Guard):** Ani sıçramaları engellemek için `outlier_guard = OutlierGuard(...)` oluşturulur (Satır: 207).

## 3. Ana Kamera Döngüsü (Her Karede Çalışan İşlemler)
`grab.py` dosyasında satır 219'da başlayan `while True:` döngüsü içinde, saniyede ~30 kez (FPS) aşağıdaki akış sırayla çalışır:

* **A) Kamera Okuma ve Düzeltme (Undistort):** `cap.read()` ile kare alınır (L220). Lens bükülmelerini gidermek için `cv2.undistort(frame, camera_matrix, dist_coeffs)` çağrılır (L223).
* **B) Marker Tespiti:** `aruco_detector.detectMarkers(frame)` ile karedeki tüm marker'lar bulunur (L227). Köşeler (masa, ID 1-4), Hedef (nesne, ID 6-9), Taban (ID 0) ve Bilek (gripper, ID 5) marker'ları ayrı ayrı sözlüklere (dict) ayrıştırılır (L240-268).
* **C) Köşe Sıralama:** `_sort_corners_by_id()` çağrılarak, masanın 4 köşesi `config.KOSE_ID_TO_POSITION` sözlüğüne göre (kameranın bakış açısından bağımsız olarak) doğru sıraya sokulur (L273).
* **D) Homografi Hesaplama:**
    * `solvePnP` (IPPE_SQUARE bayrağı ile) kullanılarak köşelerin 3B konumu bulunur (L300).
    * Fiziksel masanın sınırları hesaplanır ve `cv2.getPerspectiveTransform(kaynak_noktalar, hedef_noktalar)` ile homografi matrisi (`matris`) oluşturulur (L338).
* **E) Hedefin Piksel → CM Dönüşümü:** Tespit edilen hedef marker'ın merkezi, `cv2.perspectiveTransform` ile kuşbakışı (warped) düzleme taşınır (L367). Ardından `_kamera_piksel_to_raw_cm()` çağrılarak (L372) bu piksel değeri kameranın optik merkezine göre ham (raw) santimetre koordinatına çevrilir (`mevcut_hedef_x`, `mevcut_hedef_y`).
* **F) Katman 3: Outlier Guard:** Ham koordinatlar, `outlier_guard.kontrol_et(mevcut_hedef_x, mevcut_hedef_y)` fonksiyonundan geçirilir (L392). Eğer değer bir glitch (sıçrama) ise reddedilir, son güvenli değer kullanılır.
* **G) Katman 2: 1Euro Filter:** Temizlenmiş veri `filtre_x(t_simdi, mevcut_hedef_x)` ve Y ekseni için benzer şekilde yumuşatılır (L401-403).
* **H) Taban ve Bilek Dinamik Ofseti:** Taban (ID 0) ve Bilek (ID 5) marker'ları görünüyorsa, hedefin mutlak koordinatlarından robotun taban merkezi çıkartılarak *robota göre (göreli)* CM koordinatları elde edilir (L418-442).
* **I) Durum Makinesi (State Machine) Akışı:**
    1. **BEKLEME:** Hedef `mevcut_hedef_x` bulunduysa `TESPIT` durumuna geçilir. `outlier_guard.sifirla()` çağrılır (L452).
    2. **TESPIT:** Hedef konumun `config.KONUM_SABITLEME_KARE` (örn: 10 kare) boyunca stabil (titremeyen) kalması beklenir. Stabilse `hedef_x_cm` kilitlenir ve `YAKLASMA`'ya geçilir (L472).
    3. **YAKLASMA:** Yörünge oluşturulur. `hesapla_ik()` çağrılarak kinematik çözülür (L485). `eklem_yorungesi_uret()` (L494) ile sarsıntısız bir Kübik Spline (zaman tabanlı eklem açıları dizisi) üretilir. Döngünün her dönüşünde (karede), bu dizideki bir sonraki adım `_servo_gonder` ile motora gönderilir (L513). Yörünge bitince `HIZALAMA`'ya geçilir.
    4. **HIZALAMA (IBVS):** Kamera ile ölçülen gerçek gripper (`bilek_x_cm`) ile `hedef_x_cm` arasındaki fark bulunur (L526). Hata küçükse `KAVRAMA`'ya geçilir. Hata büyükse mikro düzeltmeli (kapalı çevrim) `hesapla_ik()` yapılarak kol hafifçe kaydırılır (L539).
    5. **KAVRAMA:** `_servo_gonder` ile sadece tutucu (`tutucu=config.GRIPPER_KAPALI_ACI`) kapatılır (L561).
    6. **KALDIRMA, GERI, BIRAKMA:** Kol havaya kalkar, başlangıç (Park) pozisyonuna döner, tutucu açılır (L569-612).
    7. **TAMAMLANDI:** İşlem biter, değişkenler sıfırlanıp tekrar `BEKLEME`'ye dönülür (L614-625).

## 4. Ters Kinematik (IK) Hesabı
* **Çalışan Fonksiyon:** `vision/kinematics/ik_solver.py` içindeki `hesapla_ik(x_cm, y_cm, z_cm, config, hedef_aci)` (Örn: L485'te çağrılır)
* **Girdi:** Robot tabanına göre göreli `x_cm`, `y_cm` (hedef uzaklık), o anki durumun istediği `z_cm` yüksekliği, `config` değerleri (link uzunlukları) ve gripper yönelimi `hedef_aci`.
* **Çıktı:** `IKSonucu` nesnesi (j1, j2, j3, j4 derece açıları ve erişilebilirlik boolean değeri).

## 5. Servo Komutunun Server'a İletilmesi
* **Çalışan Fonksiyon:** `_servo_gonder(server_url, taban, omuz, dirsek, bilek, tutucu)` (`grab.py` Satır: 63)
* **İşlem:** Eğer yeni açılar bir öncekiyle aynıysa gereksiz ağ trafiği yaratmamak için işlem atlanır (`_son_gonderilen` kontrolü, L78). Değilse, bir JSON payload'u hazırlanarak `requests.post()` ile Flask sunucusuna (`http://127.0.0.1:5000/set_angles`) HTTP isteği olarak gönderilir (L82).

## 6. Flask Sunucu ve Arduino Komut İletimi (`server.py`)
* **Çalışan Fonksiyon:** `html/server.py` içindeki `set_angles()` (Satır: 73)
* **İşlem:** JSON datasındaki açılar çekilir. Yazılımsal sıfır noktaları donanımın sıfır noktalarıyla eşleşsin diye `config.J1_OFFSET` gibi offset değerleri eklenir (L88). Değerler donanımın fiziksel sınırları (0-180) içine kırpılır (constrain).
* **Formatlama:** Komut `J1:x,J2:y,J3:z,J4:w,J5:v\n` şeklinde bir String (metin) formatına getirilir (L100).
* **İletim:** `arduino.write(command.encode())` ile USB Seri port (veya Wokwi köprüsü) üzerinden donanıma yazılır (L108).

## 7. Arduino Tarafı (Donanım İşlemi)
* **Çalışan Dosya:** `PCA9685_arduino_kodu/PCA9685_arduino_kodu.ino`
* **Çalışan Fonksiyon:** `loop()` (Satır: 71) ve `parseAndApply()` (Satır: 37)
* **İşlem:** Arduino, `loop()` içerisinde USB'den gelen karakterleri `\n` (enter) karakterine kadar tampona (buffer) ekler (L75). Satır bittiğinde `parseAndApply()` fonksiyonuna gönderir.
* **Parsing:** String, virgüllerden parçalanır (`indexOf(',')`). Her bir parça (`J1:45` gibi), iki nokta üst üste ile ayrıştırılarak hangi servonun hangi açıya döneceği `angles` dizisine kaydedilir (L41-58).
* **Motor Kontrolü:** Son olarak `setServoAngle(J1_PIN, angles[0])` gibi çağrılarla (L62) açı, PWM sinyaline (pulse width) dönüştürülüp I2C üzerinden Adafruit PCA9685 servo sürücü kartına iletilir. Sürücü kart motorları fiziksel olarak çevirir.

---

## Tüm Sistem Akış Şeması

```text
[KULLANICI] -> "python -m vision.main --mode grab"
       |
       v
[MAIN.PY] -> args ayrıştırılır, grab_mode(config) çağrılır.
       |
       v
[GRAB.PY : KURULUM] -> Kalibrasyon yüklenir, filtreler (Outlier, 1Euro) oluşturulur.
       |
       +-----> [KAMERA DÖNGÜSÜ] (Saniyede 30 Kez)
                     |
                     +-- 1. cap.read() ve cv2.undistort()
                     |
                     +-- 2. detectMarkers() -> Köşeler, Taban(0), Bilek(5), Hedef(6-9)
                     |
                     +-- 3. _sort_corners_by_id() (Katman 1) -> Fiziksel sıralama
                     |
                     +-- 4. getPerspectiveTransform() -> Homografi Matrisi (Kuşbakışı düzlem)
                     |
                     +-- 5. Hedef marker pikseli -> cm'ye çevrilir (warping)
                     |
                     +-- 6. OutlierGuard (Katman 3) -> Ani sıçramaları filtreler
                     |
                     +-- 7. OneEuroFilter (Katman 2) -> Kalan küçük titremeleri yumuşatır
                     |
                     +-- 8. Koordinat dönüşümü: Hedef CM - Taban CM = Göreli Koordinat
                     |
                     +-- 9. DURUM MAKİNESİ:
                             * BEKLEME  -> Hedef bulundu mu? -> TESPIT'e geç.
                             * TESPIT   -> N kare sabit kaldı mı? -> IK hesapla (YAKLASMA'ya geç).
                             * YAKLASMA -> Spline yörünge adım adım sunucuya gönderilir -> HIZALAMA'ya geç.
                             * HIZALAMA -> (Bilek CM - Hedef CM) küçük mü? -> KAVRAMA'ya geç. (Değilse mikro IK düzeltmesi gönder).
                             * KAVRAMA, KALDIRMA, GERI, BIRAKMA -> Komutlar gönderilir.
                     |
                     v
[GRAB.PY : İLETİŞİM] -> _servo_gonder(taban, omuz, dirsek, bilek, tutucu)
                     |  (Sadece açı değişmişse HTTP POST atar)
                     v
[SERVER.PY] -----> POST /set_angles -> JSON okunur.
                     |
                     +-- 1. config.J1_OFFSET gibi donanım ofsetleri eklenir.
                     |
                     +-- 2. "J1:x,J2:y,J3:z,J4:w,J5:v\n" stringi oluşturulur.
                     |
                     +-- 3. serial.write() ile USB (veya Wokwi) seri porta yazılır.
                     |
                     v
[ARDUINO.INO] ---> loop() -> Serial buffer \n görene kadar dinler.
                     |
                     +-- 1. parseAndApply() -> Virgül ve ":" karakterlerinden stringi ayrıştırır.
                     |
                     +-- 2. setServoAngle() -> Açı değerini PWM sinyaline çevirip donanımı hareket ettirir.
```
