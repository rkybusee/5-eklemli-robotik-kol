# 5-Eklemli Robotik Kol — Tam Teknik Dokümantasyon

> **Bu doküman**, projenin baştan sona nasıl çalıştığını, her dosyanın ne yaptığını, hangi algoritmanın nerede kullanıldığını ve veri akışının nasıl ilerlediğini en ince detayıyla anlatır.

---

## 📁 Proje Klasör Yapısı

```
5-eklemli-robotik-kol-main/
│
├── html/                                    ← Web kontrol paneli + Flask sunucu
│   ├── server.py                            ← Flask API sunucusu (Python ↔ Arduino köprüsü)
│   └── templates/
│       └── kol_kontrol.html                 ← Web arayüzü (slider'larla manuel kontrol)
│
├── PCA9685_arduino_kodu/
│   └── PCA9685_arduino_kodu.ino             ← Arduino sketch (5 servo kontrolü)
│
├── wokwi_proje/                             ← Wokwi simülasyon dosyaları
│   ├── wokwi_proje.ino                      ← Simülasyon Arduino kodu
│   ├── diagram.json                         ← Wokwi devre şeması
│   ├── wokwi.toml                           ← RFC2217 port ayarları
│   └── build/                               ← Derlenmiş .hex/.elf dosyaları
│
├── kalibrasyon/                             ← ★ ANA VİZYON PAKETİ ★
│   ├── requirements.txt                     ← Python bağımlılıkları
│   ├── plan.md                              ← Proje yol haritası
│   ├── sk_plan.md                           ← Vision katman planı (bu dosyadan çalıştık)
│   ├── oto_kalibre.py                       ← Eski kalibrasyon scripti (kullanılmıyor)
│   │
│   └── vision/                              ← Python paketi (tüm görüntü işleme)
│       ├── __init__.py                      ← Paket tanımlayıcı (boş)
│       ├── config.py                        ← ★ Merkezi konfigürasyon (TÜM parametreler)
│       ├── main.py                          ← CLI giriş noktası (--mode ile mod seçimi)
│       │
│       ├── core/                            ← Çekirdek yardımcı fonksiyonlar
│       │   ├── __init__.py
│       │   └── utils.py                     ← Kalibrasyon yükleme/kaydetme, ChArUco oluşturma
│       │
│       ├── calibration/                     ← Kamera kalibrasyonu
│       │   ├── __init__.py
│       │   ├── capture.py                   ← Kalibrasyon fotoğrafı toplama
│       │   ├── calibration_engine.py        ← Kalibrasyon hesaplama + outlier eleme
│       │   └── calibration_verifier.py      ← Kalibrasyon doğrulama (canlı hata ölçümü)
│       │
│       ├── kinematics/                      ← Robot kinematiği
│       │   └── ik_solver.py                 ← Ters Kinematik (IK) + İleri Kinematik (FK)
│       │
│       └── measurement/                     ← Ölçüm + otonom kontrol
│           ├── __init__.py
│           ├── birds_eye_view.py            ← Kuşbakışı görünüm + visual servoing
│           ├── grab.py                      ← ★ ANA OTONOM YAKALAMA MODÜLÜ ★
│           ├── one_euro_filter.py            ← 1Euro titreme filtresi (Katman 2)
│           └── safety_trajectory.py          ← Outlier guard + spline yörünge (Katman 3)
```

---

## 🔄 Sistemin Genel Akışı (Büyük Resim)

```
┌─────────────────────────────────────────────────────────────────────┐
│                        KULLANICI                                     │
│   "python -m vision.main --mode grab" veya web paneli                │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [1] KAMERA + KALİBRASYON                                           │
│  Webcam'den her kare okunur (cv2.VideoCapture)                       │
│  Lens bozulması düzeltilir (cv2.undistort)                           │
│  calibration_result.yaml'daki camera_matrix kullanılır               │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Düzeltilmiş kare (frame)
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [2] ArUco MARKER TESPİTİ                                           │
│  5x5_100 sözlüğünden marker'lar aranır                              │
│  Köşe marker'lar (ID 0,2,4,5) → masa sınırlarını tanımlar           │
│  Hedef marker'lar (ID 6,7,8,9) → tutulacak nesneler                 │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Marker konumları (piksel)
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [3] HOMOGRAFİ + KOORDİNAT DÖNÜŞÜMÜ                                │
│  4 köşe marker → IPPE_SQUARE ile 3D pozisyon → gerçek masa boyutu   │
│  getPerspectiveTransform → kuşbakışı dönüşüm matrisi                │
│  Piksel → cm dönüşümü (_kamera_piksel_to_robot_cm)                  │
│  Robot tabanına göre ofset uygulanır                                  │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Hedef (X, Y) cm cinsinden
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [4] FİLTRELEME KATMANları                                          │
│  OutlierGuard → ani sıçramaları eler (Katman 3)                     │
│  1Euro Filter → temizlenmiş veriyi yumuşatır (Katman 2)             │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Filtrelenmiş (X, Y) cm
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [5] DURUM MAKİNESİ (grab.py)                                        │
│  BEKLEME → TESPIT → YAKLASMA → HIZALAMA → KAVRAMA → KALDIRMA →     │
│  GERI → BIRAKMA → TAMAMLANDI → BEKLEME ...                          │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Eklem açıları (J1, J2, J3, J4, gripper)
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [6] TERS KİNEMATİK (ik_solver.py)                                  │
│  (X, Y, Z) cm → (J1, J2, J3, J4) derece                             │
│  Kosinüs teoremi ile çözüm                                          │
│  Erişilebilirlik kontrolü                                            │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Servo açıları
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [7] FLASK SUNUCU (server.py)                                        │
│  HTTP POST /set_angles → JSON payload                                │
│  Offset uygulaması (J1_OFFSET, J2_OFFSET vb.)                       │
│  Seri port komutu: "J1:x,J2:y,J3:z,J4:w,J5:v\n"                    │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Seri port verisi
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [8] ARDUINO / WOKWI                                                 │
│  Seri porttan gelen komutu parse eder                                │
│  Servo.h ile 5 servoyu fiziksel olarak hareket ettirir               │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 📋 Dosya Dosya Detaylı Açıklama

---

### 1. `vision/config.py` — Merkezi Konfigürasyon

**Amaç:** Projedeki TÜM parametreleri tek bir `VisionConfig` dataclass'ında toplar. Hiçbir modül kendi içinde "magic number" kullanmaz — her şey buradan çekilir.

**Önemli Parametre Grupları:**

| Grup | Parametreler | Açıklama |
|------|-------------|----------|
| **ChArUco Board** | `SUTUN_SAYISI=5`, `SATIR_SAYISI=7`, `KARE_UZUNLUGU_M=0.04`, `MARKER_UZUNLUGU_M=0.03`, `DICTIONARY=DICT_5X5_100` | Kalibrasyon board'unun fiziksel özellikleri. Board basılırken bu değerlerle üretilmiş olmalı. |
| **Kamera** | `KAMERA_INDEX=1`, `KAMERA_YUKSEKLIK_CM=54.5`, `KAMERA_ROTASYON=0` | Hangi kameranın kullanılacağı, masadan yüksekliği ve fiziksel yönelimi. |
| **Marker ID'leri** | `KOSE_MARKER_IDLERI=[0,2,4,5]`, `HEDEF_MARKER_IDS=[6,7,8,9]`, `BILEK_MARKER_ID=1` | Köşe marker'lar masanın sınırlarını tanımlar, hedef marker'lar tutulacak nesnelerdir. |
| **Marker Boyutları** | `KOSE_MARKER_BOYUTU_M=0.061`, `KUP_MARKER_BOYUTU_M=0.025` | ArUco marker'ların kenar uzunluğu (metre). IPPE_SQUARE algoritması ile 3D konum hesabı için kritik. |
| **Robot Geometrisi** | `LINK1=21cm`, `LINK2=24cm`, `LINK3=11cm`, `TABAN_YUKSEKLIK=9cm` | Kolun fiziksel uzunlukları. IK hesabının doğruluğu tamamen bunlara bağlı. |
| **Ofsetler** | `J1_OFFSET=-42`, `J2_OFFSET=-12` vb. | IK'nın hesapladığı matematiksel açı ile servo motorun fiziksel 0 noktası arasındaki fark. |
| **Robot Pozisyonu** | `ROBOT_BASE_OFFSET_X_CM=-9.10`, `ROBOT_BASE_OFFSET_Y_CM=49.77` | Kameranın optik merkezinden robot tabanına olan fiziksel uzaklık (cm). |
| **Gripper** | `GRIPPER_ACIK_ACI=30`, `GRIPPER_KAPALI_ACI=130` | Tutucunun açık/kapalı servo açıları. |
| **Köşe ID→Pozisyon** | `KOSE_ID_TO_POSITION={0:"sol_ust", 2:"sag_ust"...}` | Her köşe marker ID'sinin masadaki fiziksel konumu. Kamera açısından bağımsız sıralama için kullanılır. |
| **1Euro Filter** | `ONEEURO_MIN_CUTOFF=1.0`, `ONEEURO_BETA=0.015` | Titreme filtresinin hassasiyet ayarları. |
| **Outlier Guard** | `OUTLIER_PENCERE_BOYUTU=5`, `OUTLIER_ESIK_CM=3.0` | Ani sıçrama eleme pencere boyutu ve eşik değeri. |
| **Spline Yörünge** | `SPLINE_SURE_GUVENLI_SN=1.0`, `SPLINE_SURE_ALCAK_SN=1.2`, `SPLINE_FPS=30` | Kübik spline yörünge üretim parametreleri. |
| **IBVS** | `KAPALI_CEVRIM_TOLERANS_CM=1.5`, `KAPALI_CEVRIM_HIZ=0.4` | Kapalı çevrim hizalama toleransı ve düzeltme hızı. |

**`get_marker_boyutu(marker_id)` metodu:** Verilen ID'nin köşe mi küp mü olduğuna bakıp doğru boyutu döndürür — IPPE_SQUARE algoritmasının 3D hesabı buna bağlıdır.

---

### 2. `vision/main.py` — CLI Giriş Noktası

**Amaç:** Komut satırından `--mode` argümanıyla hangi modun çalışacağını belirler. Tüm modları tek bir noktadan kontrol eder.

**Kullanım:**
```bash
python -m vision.main --mode capture          # Kalibrasyon fotoğrafı topla
python -m vision.main --mode calibrate        # Kalibrasyonu hesapla
python -m vision.main --mode verify           # Kalibrasyonu doğrula
python -m vision.main --mode birds-eye-view   # Kuşbakışı + visual servoing
python -m vision.main --mode grab             # Otonom cisim yakalama
```

**Çalışma prensibi:**
1. `argparse` ile `--mode` ve `--force-recalibrate` argümanları alınır
2. `VisionConfig()` nesnesi oluşturulur (varsayılan parametrelerle)
3. Seçilen moda göre ilgili fonksiyon çağrılır

---

### 3. `vision/core/utils.py` — Çekirdek Yardımcı Fonksiyonlar

**3 temel fonksiyon:**

#### `create_charuco_board(config)`
- Config'deki parametrelerle (`SUTUN_SAYISI`, `SATIR_SAYISI`, `KARE_UZUNLUGU_M`, `MARKER_UZUNLUGU_M`, `DICTIONARY`) bir ChArUco board ve detector nesnesi oluşturur.
- Kalibrasyon modlarında (capture, calibrate, verify) kullanılır.

#### `save_calibration(filepath, camera_matrix, dist_coeffs, reproj_error)`
- `camera_matrix` (3x3 intrinsik matris) ve `dist_coeffs` (lens bozulma katsayıları) değerlerini YAML formatında diske yazar.
- Kalibrasyon sonucu `calibration_result.yaml` olarak kaydedilir.

#### `load_calibration(filepath)`
- YAML dosyasından kamera matrisini ve distorsiyon katsayılarını okur.
- Geriye dönük uyumluluk: `.yaml` bulamazsa `.npz` dener.
- Tüm ölçüm/yakalama modları bu fonksiyonla kalibrasyonu yükler.

---

### 4. `vision/calibration/capture.py` — Kalibrasyon Fotoğrafı Toplama

**Amaç:** Kamerayı açıp ChArUco board'u otomatik olarak fotoğraflar.

**Nasıl çalışır:**
1. `camPhotos/` klasörü oluşturulur
2. Kamera açılır (`cv2.VideoCapture`)
3. Her karede `detector.detectBoard(frame)` ile ChArUco köşeleri aranır
4. Eğer bulunan köşe sayısı >= `CAPTURE_MIN_KOSE` (8) VE son fotoğraftan bu yana `YAKALAMA_ARALIGI_SN` (1.5s) geçtiyse → otomatik kayıt
5. Fotoğraflar `calib-001.png`, `calib-002.png` şeklinde numaralanır
6. Ekranda köşe sayısı ve ilerleme gösterilir
7. `MAX_KARE_PER_TUR` (50) kareyeveya 's'/ESC tuşuyla durur

**Neden önemli:** Farklı açı ve mesafelerden en az 15-20 kare çekilmeli. Tek açıdan çekilen fotoğraflar kötü kalibrasyon verir.

---

### 5. `vision/calibration/calibration_engine.py` — Kalibrasyon Hesaplama

**Amaç:** Toplanan fotoğraflardan kameranın iç parametrelerini (focal length, optik merkez, distorsiyon) hesaplar.

**Algoritma detayı:**

1. **Fotoğraf yükleme:** `camPhotos/` klasöründeki tüm `calib-*.png` dosyaları okunur
2. **Köşe tespiti:** Her fotoğrafta ChArUco board'un köşeleri tespit edilir
3. **Subpixel refinement:** Bulunan köşeler `cv2.cornerSubPix()` ile alt-piksel hassasiyetine çıkarılır
4. **Filtreleme:** Köşe sayısı < `VALID_MIN_KOSE` (18) olan karelerin kalitesi düşüktür, atlanır
5. **İlk kalibrasyon:** `cv2.calibrateCamera()` çağrılır:
   - Girdi: 3D nesne noktaları (board üzerindeki bilinen pozisyonlar) + 2D görüntü noktaları (tespit edilen piksel konumları)
   - Çıktı: `camera_matrix` (3x3), `dist_coeffs` (5 katsayı), `rvecs`/`tvecs` (her kare için poz)
6. **Outlier rejection (3 tur):**
   - Her kare için reprojection hatası hesaplanır
   - Ortalama + 1.5 x std üzerindeki kareler "kötü" sayılır
   - Kötü kareler elenerek kalibrasyon yeniden yapılır
   - Bu işlem hata stabilize olana kadar (veya 3 tur) tekrarlanır
7. **Sonuçlar:**
   - `calibration_result.yaml` dosyasına kaydedilir
   - `kalibrasyon_log.md` dosyasına tarihli kayıt eklenir

**Çıktı formatı (camera_matrix):**
```
| fx  0  cx |
|  0  fy cy |     fx,fy = odak uzaklığı (piksel)
|  0   0  1 |     cx,cy = optik merkez (piksel)
```

---

### 6. `vision/calibration/calibration_verifier.py` — Kalibrasyon Doğrulama

**Amaç:** Yapılan kalibrasyonun ne kadar doğru olduğunu **canlı olarak** ölçer.

**Nasıl çalışır:**
1. Kalibrasyon yüklenir (`camera_matrix`, `dist_coeffs`)
2. ChArUco board kameraya gösterilir
3. `solvePnP` ile board'un 3D pozu hesaplanır
4. Komşu köşeler arası 3D mesafe ölçülür
5. Bu mesafe, bilinen kare boyutu (`KARE_UZUNLUGU_M = 4cm`) ile karşılaştırılır
6. Sapma cm ve % olarak ekranda canlı gösterilir:
   - **Yeşil:** %5'ten az hata → iyi kalibrasyon
   - **Turuncu:** %5-10 arası → orta
   - **Kırmızı:** %10'dan fazla → yeniden kalibrasyon gerekli

---

### 7. `vision/kinematics/ik_solver.py` — Ters ve İleri Kinematik

**Amaç:** Hedef (X, Y, Z) noktasından servo açılarına (J1-J4) dönüşüm yapar.

#### `hesapla_ik(x_cm, y_cm, z_cm, config, hedef_aci_derece)` — Ters Kinematik

**Robot kol yapısı (4 DOF):**
```
       J2 (omuz)
       /
      / L1 (21cm)
     /
    J3 (dirsek)
     \
      \ L2+L3 (24+11=35cm efektif)
       \
        Gripper ---- J4 (bilek, sadece roll)
         |
    J1 (taban, yaw)
         |
   ══════╧══════ Masa (Z=0)
     Taban (H=9cm)
```

**Hesaplama adımları:**

1. **J1 (Taban):** `atan2(Y, X)` → yatay düzlemdeki hedef yönü belirler
2. **r (yatay mesafe):** `sqrt(X^2 + Y^2)` → hedefin robot tabanından yatay uzaklığı
3. **z_rel (göreli yükseklik):** `Z - taban_yüksekliği` → omuz eklemine göre hedef yüksekliği
4. **J3 (Dirsek) — Kosinüs Teoremi:**
   ```
   cos(a2) = (d^2 - L1^2 - L2_eff^2) / (2 * L1 * L2_eff)
   ```
   - Eğer `cos(a2)` [-1, 1] dışındaysa → hedef erişim dışı (`erisilebilir = False`)
   - a2 negatif alınır (dirsek aşağı konfigürasyonu)
5. **J2 (Omuz):**
   ```
   a1 = atan2(z_rel, r) - atan2(L2_eff * sin(a2), L1 + L2_eff * cos(a2))
   ```
6. **J4 (Bilek):** Doğrudan `hedef_aci_derece` olarak atanır (marker'ın yönelimine göre gripper'ı hizalar)
7. **Clamp:** Tüm açılar [0, 180] aralığına kısıtlanır

**Erişilebilirlik kontrolü:** Hedef çok uzak veya çok yakınsa `erisilebilir=False` döner, program crash yapmaz.

#### `hesapla_fk(j1, j2, j3, j4, config)` — İleri Kinematik

IK'nın tersi: verilen servo açılarından gripper ucun 3D konumunu hesaplar. Test ve doğrulama için kullanılır.

---

### 8. `vision/measurement/one_euro_filter.py` — 1Euro Filter (Katman 2)

**Problem:** Kameradan gelen ham konum verisi kare-kare titrer (+/- 1-3 piksel gürültü). Bu titreşim robota iletildiğinde motorlar sürekli sallanır.

**Çözüm:** 1Euro Filter — adaptif alçak geçiş filtresi.

**Nasıl çalışır:**
- **Yavaş hareket (düşük hız):** Agresif filtreleme → titremeler bastırılır
- **Hızlı hareket (yüksek hız):** Minimal filtreleme → gecikme olmaz

**Kullanım yeri:** `grab.py`'de hedef tespit edildikten ve **OutlierGuard'dan geçtikten sonra**:
```python
mevcut_hedef_x = filtre_x(t_simdi, mevcut_hedef_x)
mevcut_hedef_y = filtre_y(t_simdi, mevcut_hedef_y)
```

---

### 9. `vision/measurement/safety_trajectory.py` — Güvenlik Ağı + Yörünge (Katman 3)

#### `OutlierGuard` — Medyan Tabanlı Outlier Eleme

**Problem:** Kamerada nadiren oluşan "glitch"ler (yanlış marker tespiti, ışık yansıması) hedef konumun anlık olarak 10-20cm sıçramasına neden olabilir. Bu yanlış veri 1Euro filtreye girerse sistemi uzun süre bozar.

**Nasıl çalışır:**
1. Son 5 konum değeri bir kayan pencerede (`deque`) tutulur
2. Yeni değer geldiğinde, pencerenin **medyanı** hesaplanır
3. Yeni değerin medyandan uzaklığı ölçülür
4. Eğer uzaklık > `OUTLIER_ESIK_CM` (3cm) → **outlier!** Son güvenli değer döndürülür
5. Eğer uzaklık <= eşik → kabul edilir, pencereye eklenir

**Önemli Not:** Yeni bir hedefe kilitlenildiğinde (`BEKLEME` → `TESPIT` geçişinde) eski hedefin verilerinin yeni hedefi bozmaması için `outlier_guard.sifirla()` çağrılır.

#### `eklem_yorungesi_uret()` — Kübik Spline Yörünge

**Problem:** Eski kodda robot "park pozisyonu → hedef" arasında ani geçiş yapıyordu (adım fonksiyonu). Bu motorlara darbe gibi gelir ve kolun sarsılmasına neden olur.

**Çözüm:** Kübik spline interpolasyonu ile yumuşak geçiş.

**Nasıl çalışır:**
1. Başlangıç ve hedef açılar verilir (4 eklem)
2. `scipy.interpolate.CubicSpline` kullanılarak ara değerler üretilir
3. **Sınır koşulu:** Başlangıç ve bitiş hızı = 0 (`bc_type=((1, 0.0), (1, 0.0))`)
   - Bu, kolun harekete **yumuşak başlayıp yumuşak durması** anlamına gelir
4. `SPLINE_FPS` (30) x süre kadar adım üretilir
5. Sonuç: (adım_sayısı x 4) boyutunda açı dizisi

**Örnek:** 1.5 saniye, 30 FPS → 45 adım, her adımda [J1, J2, J3, J4] açıları:
```
Adım 0:  [90, 45, 120, 90]  ← Park (hız=0, yumuşak başlangıç)
Adım 10: [82, 55, 108, 88]  ← Hızlanma fazı
Adım 22: [70, 68, 88, 86]   ← Maksimum hız
Adım 35: [63, 77, 74, 84]   ← Yavaşlama fazı
Adım 44: [60, 80, 70, 85]   ← Hedef (hız=0, yumuşak duruş)
```

---

### 10. `vision/measurement/grab.py` — ★ Ana Otonom Yakalama Modülü ★

Bu dosya projenin kalbi. 634 satırlık bir durum makinesi ile robotun tüm otonom davranışını kontrol eder.

#### Durum Makinesi Akışı

```
    ┌──────────┐
    │ BEKLEME  │ ← Başlangıç. Hedef marker aranıyor.
    └────┬─────┘
         │ Hedef marker bulundu
         ▼
    ┌──────────┐
    │ TESPIT   │ ← Konum sabitleniyor (10 kare stabil olmalı)
    └────┬─────┘
         │ 10 kare boyunca < 2cm kayma
         ▼
    ┌──────────┐
    │ YAKLASMA │ ← Spline yörünge ile cismin üzerine gidiliyor
    └────┬─────┘
         │ Yörünge tamamlandı (tüm adımlar gönderildi)
         ▼
    ┌──────────┐
    │ HIZALAMA │ ← Kapalı çevrim IBVS ile son mm hizalama
    └────┬─────┘
         │ Hata < 1.5cm
         ▼
    ┌──────────┐
    │ KAVRAMA  │ ← Gripper kapatılıyor (1.5s bekleme)
    └────┬─────┘
         │ Bekleme süresi doldu
         ▼
    ┌──────────┐
    │ KALDIRMA │ ← Kol yukarı kaldırılıyor (Z=15cm)
    └────┬─────┘
         │ 1s bekleme
         ▼
    ┌──────────┐
    │   GERI   │ ← Park pozisyonuna dönüş
    └────┬─────┘
         │ 1s bekleme
         ▼
    ┌──────────┐
    │ BIRAKMA  │ ← Gripper açılıp nesne bırakılıyor
    └────┬─────┘
         │ 1s bekleme
         ▼
    ┌──────────────┐
    │ TAMAMLANDI   │ ← 2s sonra BEKLEME'ye döner (yeni hedef arar)
    └──────────────┘
```

#### Her Karede (while döngüsünde) Sırayla Ne Olur:

**Adım 1 — Kare oku ve düzelt:**
```python
ret, frame = cap.read()
frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
```

**Adım 2 — ArUco marker'ları tespit et:**
```python
corners, ids, rejected = aruco_detector.detectMarkers(frame)
```
Her marker'ın ID'si ve 4 köşe pikseli belirlenir. Sonra köşe marker'lar (`detected_corner_pts`) ve hedef marker'lar (`detected_hedef`) ayrı sözlüklere kaydedilir.

**Adım 3 — 4 köşe marker bulunduysa → homografi hesapla:**
1. **ID-bazlı sıralama** (`_sort_corners_by_id`): `KOSE_ID_TO_POSITION` sözlüğüne bakılarak marker'lar fiziksel pozisyonlarına göre sıralanır (sol-üst, sağ-üst, sağ-alt, sol-alt). Ekrana bu etiketler çizilir (Örn: `K0 (sol_ust)`).
2. **3D mesafe ile masa boyutu**: Her köşe marker'ın 3D konumu hesaplanır. Kare markerlar için eski/titreyen yöntemler bırakılmış, tamamen **`IPPE_SQUARE`** algoritmasına geçilmiştir.
3. **Homografi matrisi**: `cv2.getPerspectiveTransform()` ile kamera görüntüsünden kuşbakışı görünüme dönüşüm matrisi hesaplanır.

**Adım 3b — 4 köşe bulunamazsa → kayıp sayacı:**
```python
kayip_kare_sayaci += 1
if kayip_kare_sayaci > KOSE_KAYIP_MAX_KARE (30):
    matris = None  # Homografi sıfırlanır
    durum = BEKLEME  # Robot durur
```
Bu, kamera kayarsa veya marker'lar kapanırsa robotun sessizce eski veriyle çalışmaya devam etmesini engeller.

**Adım 4 — Hedef marker'ın robot koordinatlarını hesapla:**
1. Hedef marker'ın piksel merkezi homografi ile kuşbakışına dönüştürülür
2. `_kamera_piksel_to_robot_cm()` ile piksel → cm dönüşümü yapılır:
   - Piksel merkezden uzaklık → cm (BIRDS_EYE_PX_PER_CM ile bölünür)
   - Kamera rotasyonuna göre eksen düzeltmesi
   - Robot tabanı ofsetinin eklenmesi
3. Marker'ın yönelim açısı hesaplanır (gripper'ın nasıl döndürüleceğini belirler)

**Adım 5 — Outlier Guard (Önce Çalışır!):**
```python
mevcut_hedef_x, mevcut_hedef_y, gecerli = outlier_guard.kontrol_et(x, y)
```

**Adım 6 — 1Euro Filter (Temiz Veriyi Yumuşatır):**
```python
mevcut_hedef_x = filtre_x(time.time(), mevcut_hedef_x)
mevcut_hedef_y = filtre_y(time.time(), mevcut_hedef_y)
```

**Adım 7 — Durum makinesinin mevcut durumuna göre eylem:**

##### BEKLEME
- Hedef marker tespit edilirse → `TESPIT`'e geç

##### TESPIT
- Her karede yeni konum ile önceki konum karşılaştırılır
- Fark < 2cm → `sabit_kare_sayaci += 1`
- Fark >= 2cm → sayaç sıfırlanır
- Sayaç `KONUM_SABITLEME_KARE` (10) olunca → pozisyon kilitlenir, `YAKLASMA`'ya geçilir
- 3 saniye içinde stabilize olmazsa → `BEKLEME`'ye geri döner

##### YAKLASMA (Spline Tabanlı)
1. İlk girişte IK ile iki hedef hesaplanır:
   - `ik_guvenli`: Güvenli yükseklikte (Z=20cm) → kol önce yukarı kalkar
   - `ik_alcak`: Yaklaşma yüksekliğinde (Z=1.5cm) → sonra iner
2. Kübik spline ile iki aşamalı yörünge üretilir:
   - Park → Güvenli yükseklik (1.0s)
   - Güvenli yükseklik → Yaklaşma yüksekliği (1.2s)
3. Her karede bir sonraki adım gönderilir (`yaklasma_adim += 1`)
4. Tüm adımlar tamamlanınca → `HIZALAMA`'ya geçilir

##### HIZALAMA (IBVS — Katman 4)
- Gripper üzerindeki **Bilek Marker'ın (ID 1)** güncel konumu ile kilitlenmiş hedef karşılaştırılır.
- `hata_mesafe = sqrt((hedef_x - bilek_x)^2 + (hedef_y - bilek_y)^2)`
- Gerçek tutucu ile hedef arasındaki hata < `KAPALI_CEVRIM_TOLERANS_CM` (1.5cm) → `KAVRAMA`'ya geç
- Hata > tolerans → mikro düzeltme: hedef pozisyon hatanın oranı kadar güncellenip IK tekrar hesaplanır.
- Bu, mekanik kaymalar ve kalibrasyon hataları için gerçek zamanlı telafi sağlar. (Eğer gripper'da marker görünmüyorsa robot bekler ve sonra BEKLEME durumuna döner).

##### KAVRAMA
- Gripper kapatılır (`GRIPPER_KAPALI_ACI = 130`)
- `KAVRAMA_BEKLEME_SN` (1.5s) sonra → `KALDIRMA`

##### KALDIRMA
- IK ile kol `KALDIRMA_YUKSEKLIK_CM` (15cm) yüksekliğe çıkarılır
- 1s sonra → `GERI`

##### GERI
- Park pozisyonuna (J1=90, J2=45, J3=120, J4=90) geri dönülür
- 1s sonra → `BIRAKMA`

##### BIRAKMA
- Gripper açılır → nesne bırakılır
- 1s sonra → `TAMAMLANDI`

##### TAMAMLANDI
- 2s bekleme sonra → `BEKLEME` (yeni hedef aranır)
- Tüm değişkenler sıfırlanır, outlier guard temizlenir

#### Ekran Üzerinde Gösterilenler
- Üst bar: Mevcut durum + geçen süre (renk kodlu)
- Alt bar: Hedef konum (cm) + marker ID
- IK bilgisi: J1-J4 açıları
- Tespit durumunda: İlerleme çubuğu
- Kuşbakışı penceresi: Düzleştirilmiş görünüm + robot konumu

---

### 11. `vision/measurement/birds_eye_view.py` — Kuşbakışı + Visual Servoing

**Amaç:** `grab.py`'nin önceki versiyonu / alternatif modu. Farklı olarak:
- `HareketPlanlayici` sınıfı ile arka planda thread'li servo kontrolü
- Bilek marker (ID 1) tespiti ile gerçek gripper pozisyonu takibi
- Kapalı çevrim: kamera → hata hesabı → düzeltme (sürekli)

**`HareketPlanlayici` sınıfı:**
- Arka planda 40ms'de bir çalışan thread
- Mevcut açılardan hedefe adım adım gider (maks 2 derece/adım)
- Bu, motorlara ani değer göndermek yerine yumuşak geçiş sağlar

---

### 12. `html/server.py` — Flask API Sunucusu

**Amaç:** Python'daki hesaplanan servo açılarını Arduino'ya ileten köprü.

**3 bağlantı modu:**

| Mod | Açıklama |
|-----|----------|
| `mock` | Donanım yok, komutlar sadece terminale yazılır |
| `wokwi` | RFC2217 protokolü ile Wokwi simülasyonuna bağlanır |
| `real` | Gerçek Arduino'ya COM port üzerinden bağlanır |

**Endpoint'ler:**
- `POST /set_angles` — JSON gövdesinde `{taban, omuz, dirsek, bilek, tutucu}` alır
  - Offset uygulanır (`taban_donanim = taban + J1_OFFSET`)
  - [0, 180] aralığına clamp edilir
  - `"J1:x,J2:y,J3:z,J4:w,J5:v\n"` formatında seri porta yazılır
- `GET /status` — Bağlantı durumunu döndürür
- `POST /save_gripper` — Gripper açısını config.py'ye yazar

**Güvenlik:** `?key=A1C67B1290.12` parametresiyle basit erişim kontrolü. İlk girişte session cookie oluşturulur.

---

### 13. Arduino Kodu (`wokwi_proje.ino` / `PCA9685_arduino_kodu.ino`)

**Amaç:** Seri porttan gelen `J1:x,J2:y,J3:z,J4:w,J5:v` komutunu parse edip 5 servoya uygular.

- `Servo.h` kütüphanesi ile PWM kontrolü
- Her komut satırı `\n` ile sonlanır
- Parse edildikten sonra her servo `Servo.write(aci)` ile hareket ettirilir

---

## Kullanılan Temel Algoritmalar Özeti

| Algoritma | Nerede | Ne İçin |
|-----------|--------|---------|
| **ChArUco Board Detection** | `capture.py`, `calibration_engine.py` | Kalibrasyon board'unda köşe noktaları bulmak |
| **cv2.calibrateCamera** | `calibration_engine.py` | Kameranın intrinsik parametrelerini hesaplamak |
| **cv2.undistort** | `grab.py`, `birds_eye_view.py` | Lens bozulmasını düzeltmek |
| **cv2.solvePnP + IPPE_SQUARE** | `grab.py` | Bilinen boyuttaki marker'ın 3D pozisyonunu titreşimsiz bulmak |
| **cv2.getPerspectiveTransform** | `grab.py`, `birds_eye_view.py` | 4 nokta eşleşmesiyle perspektif dönüşüm matrisi |
| **cv2.warpPerspective** | `grab.py`, `birds_eye_view.py` | Görüntüyü kuşbakışına dönüştürmek |
| **atan2 + Kosinüs Teoremi** | `ik_solver.py` | Ters kinematik çözümü |
| **Medyan Tabanlı Outlier Eleme** | `safety_trajectory.py` | Ani sıçramaları filtrelerden önce elemek |
| **1Euro Filter** | `one_euro_filter.py` | Temizlenmiş verideki gürültüyü adaptif yumuşatmak |
| **Kübik Spline (CubicSpline)** | `safety_trajectory.py` | Yumuşak motor yörüngesi üretimi |
| **Outlier Rejection** | `calibration_engine.py` | Kötü kalibrasyon fotoğraflarını eleme |

---

## Çalıştırma Sırası (Sıfırdan)

```
1. pip install opencv-contrib-python numpy PyYAML scipy requests flask pyserial

2. config.py'de KAMERA_INDEX'i ayarlayın (0 veya 1)

3. Kalibrasyon fotoğrafları toplayın:
   cd kalibrasyon
   python -m vision.main --mode capture

4. Kalibrasyonu hesaplayın:
   python -m vision.main --mode calibrate

5. (İsteğe bağlı) Kalibrasyonu doğrulayın:
   python -m vision.main --mode verify

6. Masanın 4 köşesine marker yapıştırın (ID 0, 2, 4, 5)
   Nesnenin üzerine hedef marker yapıştırın (ID 6, 7, 8, 9)

7. config.py'de fiziksel ölçümleri girin:
   - ROBOT_BASE_OFFSET_X_CM, ROBOT_BASE_OFFSET_Y_CM
   - KAMERA_YUKSEKLIK_CM
   - KOSE_ID_TO_POSITION (hangi ID hangi köşede)

8. Flask sunucuyu başlatın:
   cd html
   python server.py

9. Otonom yakalamayı başlatın:
   cd kalibrasyon
   python -m vision.main --mode grab
```

---

## Sorun Giderme

| Sorun | Olası Neden | Çözüm |
|-------|-------------|-------|
| "Kamera açılamadı" | Yanlış `KAMERA_INDEX` | 0 veya 1 deneyin |
| "Kalibrasyon bulunamadı" | `calibration_result.yaml` yok | capture → calibrate sırasıyla çalıştırın |
| Homografi titriyor | Kamera çözünürlüğü düşük veya marker'lar küçük | Marker boyutunu artırın, ışığı iyileştirin |
| Robot yanlış noktaya gidiyor | `ROBOT_BASE_OFFSET` yanlış | Ofseti cetvel ile tekrar ölçün |
| Kol sarsılıyor | `SPLINE_FPS` düşük veya süre kısa | FPS'i artırın veya süreyi uzatın |
| Hedef sürekli atlıyor | `OUTLIER_ESIK_CM` çok yüksek | Eşiği 2.0cm'ye düşürün |

---

**Doküman Sonu.**
