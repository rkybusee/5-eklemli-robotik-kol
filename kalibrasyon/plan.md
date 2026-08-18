# Robotik Kol Projesi — Yol Haritası

## Proje Konusu

4 dönel eklemli (taban/omuz/dirsek/bilek), servo motorlu bir robotik kol geliştiriliyor. Kol, düz kinematik analiz ile tasarlanıp bir web paneli üzerinden manuel kontrol edilebiliyor. Projenin ikinci aşamasında kola bir **kamera** eklenerek, kamera ile masadaki nesneleri tespit edip konumlarını hesaplayan ve kolu otomatik olarak o nesneyi tutmaya yönlendiren bir **görüntü işleme + ters kinematik** sistemi kuruluyor.

---

## Genel Mimari (5 Katman)

```
[1] Kamera + Görüntü İşleme (OpenCV + ArUco)
        ↓ nesnenin/masanın piksel konumu → gerçek dünya (X,Y) koordinatı
[2] Koordinat Dönüşümü (kamera/masa → robot taban çerçevesi)
        ↓ X, Y, Z (robot tabanına göre)
[3] Ters Kinematik / IK (X,Y,Z → J1,J2,J3,J4 açıları + gripper durumu)
        ↓ 5 değer: 4 eklem açısı + gripper aç/kapa
[4] Flask Sunucu (server.py)
        ↓ seri port komutu: J1,J2,J3,J4,J5
[5] Arduino (sketch.ino)
        ↓ servoları fiziksel olarak hareket ettirir
```

---

## Netleşen Tasarım Kararları

| Konu | Karar |
|---|---|
| Kontrol paneli | Web tabanlı, sliderlarla manuel açı kontrolü (`kol_kontrol.html`) |
| Erişim güvenliği | Link üzerinden `?key=...` parametresiyle basit yetkilendirme |
| Python-Arduino köprüsü | Flask sunucu (`server.py`), seri port üzerinden komut gönderiyor |
| Test ortamı | Fiziksel donanım henüz yok → **Wokwi simülasyonu** (Antigravity IDE üzerinden, RFC2217 köprüsüyle) |
| Kamera montajı | Kolun tabanı/J1 seviyesinde, sabit (eye-to-hand konfigürasyonu) |
| Masa/çalışma alanı tespiti | Masanın 4 köşesine farklı ID'li ArUco marker'lar yapıştırılacak → **homografi** ile piksel↔gerçek dünya koordinat dönüşümü |
| Nesne tespiti | Renk bazlı yöntem yerine **ArUco marker bazlı** (daha hassas, ID ile nesne ayırt edilebilir, yönelim bilgisi de verir) |
| Nesne boyutu bilgisi | Her nesnenin ArUco ID'sine karşılık gelen bir **boyut tablosu** (`{id: genislik_cm}`) `server.py` içinde tutulacak |
| Gripper | 5. bir servo ile açılıp kapanan tutucu eklenecek |
| Boyut uygunluğu kontrolü | Nesne genişliği, gripper'ın maksimum açılma mesafesiyle karşılaştırılıp uygun değilse kullanıcı uyarılacak |
| Çalışma modu | **Yarı otomatik** — panelde "Tespit Et" butonuna basılınca kamera nesneyi bulup konumu hesaplayacak, kol otomatik gidecek |
| Kamera kalibrasyonu | ChArUco board (5×7 kare, 4cm kare boyutu, 5X5 dictionary) ile tek seferlik iç parametre (focal length, distorsiyon) kalibrasyonu |

---

## Şimdiye Kadar Yapılanlar

### 1. Temel sistem (manuel kontrol) — TAMAMLANDI
- [x] `kol_kontrol.html` — web kontrol paneli (4 slider: taban, omuz, dirsek, bilek)
- [x] `server.py` — Flask sunucu:
  - Key ile erişim kontrolü
  - `/set_angles` ve `/status` endpoint'leri
  - 3 bağlantı modu: `mock` (donanımsız test), `wokwi` (simülasyon), `real` (gerçek Arduino)
- [x] `sketch.ino` — Arduino kodu, `Servo.h` ile 4 servo kontrolü, seri porttan gelen `J1:x,J2:y,J3:z,J4:w` formatındaki komutu ayrıştırıp uyguluyor

### 2. Wokwi simülasyon entegrasyonu — TAMAMLANDI
- [x] Antigravity IDE'ye Wokwi eklentisi kuruldu ve lisanslandı
- [x] Arduino CLI kurulup `sketch.ino` derlendi (`.hex`/`.elf` üretildi)
- [x] `wokwi.toml` oluşturulup `rfc2217ServerPort = 4000` ile RFC2217 köprüsü aktif edildi
- [x] `server.py` üzerinden Wokwi simülasyonuna başarıyla bağlanıldı, web panelinden gönderilen açılar simülasyondaki servoları hareket ettirdi
- [x] Panel, aynı ağdaki telefon üzerinden de (bilgisayarın yerel IP'si ile) test edildi

### 3. Görüntü işleme katmanının planlanması — TAMAMLANDI (kod aşamasında)
- [x] Mimari 5 katmana çıkarıldı (kamera, koordinat dönüşümü, IK, sunucu, Arduino)
- [x] ChArUco kalibrasyon board'u oluşturuldu (5×7 kare, 4cm, 5X5 dictionary)
- [x] `kalibrasyon.py` yazıldı — webcam ile ChArUco board'u tarayıp `camera_matrix` ve `dist_coeffs` çıkarıp `calibration.npz` dosyasına kaydeden script

### 4. Web kontrol panelinin dışarıya açılması — TAMAMLANDI
- [x] Flask ve pyserial bağımlılıklarının yüklü olduğu doğrulandı (`pip install Flask pyserial`)
- [x] `html/server.py` Flask sunucusu başlatıldı (`python server.py`) — `0.0.0.0:5000` üzerinde dinliyor
- [x] Wokwi simülasyonuna `rfc2217://localhost:4000` üzerinden başarıyla bağlanıldı
- [x] `kol_kontrol.html` sayfasına yerel ağdan `http://localhost:5000/?key=A1C67B1290.12` ile erişim doğrulandı
- [x] **ngrok** ile public tünel açıldı (`ngrok http 5000`) → `https://flyable-lunchtime-astronaut.ngrok-free.dev/?key=A1C67B1290.12` adresi üzerinden tüm cihazlardan (telefon, tablet vb.) erişim sağlandı
- [x] Erişim güvenliği `?key=A1C67B1290.12` parametresiyle sağlanıyor; ilk girişten sonra oturum çerezi ile devam ediyor

---

## Yapılacaklar (Sıralı)

### Adım 1 — Kamera Kalibrasyonu (vision paketi ile yeniden yapılandırıldı) — TAMAMLANDI
- [x] Yeni `vision/` paketi oluşturuldu. `python -m vision.main --mode capture` ile görüntüler toplanacak.
- [x] `python -m vision.main --mode calibrate` ile kalibrasyon hesaplanıp `.yaml` olarak kaydedilecek.
- [x] Board'un gerçek `MARKER_UZUNLUGU_M` değeri ve `DICTIONARY` (5X5_100) ayarlandı.

### Adım 2 — Masa Köşe Tespiti + Homografi (vision paketi ile) — TAMAMLANDI

- [x] Marker'ların piksel konumlarını tespit edip `cv2.findHomography()` ile dönüşüm matrisi çıkaran `python -m vision.main --mode homography` eklendi.
- [x] Eski `olceklendirme.py` (tıkla-ölç) özelliği `python -m vision.main --mode olcum` olarak yeniden yazıldı ve lens distorsiyonu/homografi dönüşümü tam doğru sıraya (undistortPoints -> perspectiveTransform) oturtuldu.

### Adım 3 — Nesne Tespiti (ArUco) — KISMEN TAMAMLANDI
- [x] Nesne üzerine yapıştırılacak ArUco ID'leri (10+) belirlendi ve marker üretim koduna eklendi.
- [x] `NESNE_BOYUTLARI` tablosu `config.py` içinde oluşturuldu.
- [x] `python -m vision.main --mode measure` (distance_pnp) ile nesne konumu hesaplayan fonksiyon yazıldı.
- [ ] Gripper boyut uygunluğu kontrolünü eklemek (`GRIPPER_MAX_ACILMA_CM` ile karşılaştırma)

### Adım 4 — Koordinat Dönüşümü (Kamera/Masa → Robot Tabanı)
- [ ] Kameranın/masanın robot tabanına (J1 eksenine) göre ofsetini elle ölçmek
- [ ] Bu ofseti uygulayan basit bir öteleme fonksiyonu yazmak

### Adım 5 — Ters Kinematik (IK)
- [ ] Kolun link uzunluklarını netleştirmek (taban yüksekliği, omuz-dirsek, dirsek-bilek, bilek-uç mesafeleri)
- [ ] Dönen taban + düzlemsel 3 link için kosinüs teoremi tabanlı IK fonksiyonunu yazmak
- [ ] IK çıktısını Wokwi simülasyonunda görsel olarak doğrulamak (bilinen bir X,Y,Z için doğru açı üretiyor mu)

### Adım 6 — Gripper Donanımı
- [ ] `sketch.ino`'ya 5. servo (gripper) eklemek
- [ ] `server.py`'nin komut formatını `J1,J2,J3,J4,J5` olacak şekilde güncellemek

### Adım 7 — Uçtan Uca Entegrasyon
- [ ] `server.py`'ye `/detect_and_grab` endpoint'i eklemek (kamera → tespit → IK → Arduino komutu tek akışta)
- [ ] `kol_kontrol.html`'e "Tespit Et" butonu ve kamera görüntüsü/sonuç alanı eklemek
- [ ] Tüm sistemi Wokwi simülasyonunda uçtan uca test etmek

### Adım 8 — Gerçek Donanıma Geçiş (fiziksel parçalar elde edilince)
- [ ] `CONNECTION_MODE = "real"` yapıp gerçek Arduino'ya bağlanmak
- [ ] Kamerayı gerçek kol üzerine monte edip kalibrasyonu (Adım 1) o kamerayla tekrar yapmak
- [ ] Masa köşe marker'larını fiziksel olarak yerleştirip homografiyi gerçek ortamda doğrulamak

---

## İlk Başlanacak Adım

**Adım 1 — Kamera Kalibrasyonu.** `kalibrasyon.py` script'i zaten hazır durumda. Yapılacak:
1. `pip install opencv-contrib-python numpy`
2. `python kalibrasyon.py` çalıştırılıp ChArUco board webcam'e gösterilerek en az 15-20 kare toplanacak
3. Üretilen `calibration.npz` dosyası, sonraki tüm görüntü işleme adımlarının (masa homografisi, nesne tespiti) temelini oluşturacak

Bu adım tamamlanmadan Adım 2 ve sonrasına geçilmeyecek, çünkü kalibrasyon değerleri (özellikle `camera_matrix`) sonraki tüm piksel→gerçek-dünya dönüşümlerinde kullanılıyor.

---

### Adım 2.5 — Kuşbakışı Görünüm (Bird's-Eye View) — DEVAM EDİYOR

#### a) Temel Modül Oluşturma — TAMAMLANDI
- [x] `vision/measurement/birds_eye_view.py` modülü oluşturuldu
- [x] `config.py`'ye `MASA_GENISLIK_CM`, `MASA_YUKSEKLIK_CM`, `BIRDS_EYE_PX_PER_CM` eklendi
- [x] `main.py`'ye `--mode birds-eye-view` CLI modu eklendi
- [x] Her karede 4 köşe marker tespiti + saat yönünde sıralama (`np.arctan2`)
- [x] `getPerspectiveTransform` + `warpPerspective` ile gerçek zamanlı kuşbakışı görünüm
- [x] Ayrı pencerede ("Kusbakisi Gorunum") gösterim, orijinal kamera penceresi açık
- [x] Her karede matris yeniden hesaplanıyor (marker hareketi anında güncelleniyor)

#### b) Hassasiyet ve Profesyonelleştirme — YAPILACAK
- [ ] **Undistort ekleme**: Tespit edilen köşe noktalarına `cv2.undistortPoints(corners, camera_matrix, dist_coeffs, P=camera_matrix)` uygulanacak (getPerspectiveTransform'dan ÖNCE)
- [ ] **Outer-corner kullanımı**: Marker merkezi (corners.mean) yerine, her marker'ın 4 köşesinden masa merkezine en uzak olanı seçilecek (~4-5cm sistematik kayma giderilir)
- [ ] **Zamansal yumuşatma (EMA)**: `SMOOTHING_ALPHA` config parametresi ile üstel hareketli ortalama uygulanacak (titreşim filtreleme, gerçek hareketi bastırmadan)
- [ ] **Yeşil alan çizimi**: Orijinal kamera penceresinde 4 köşeyi birleştiren kapalı yeşil dörtgen (`cv2.polylines`, kalınlık 2px)
- [ ] **BirdsEyeTransformer sınıfı**: Fonksiyonel yapı sınıfa taşınacak:
  - `detect_corners(frame) -> Optional[np.ndarray]`
  - `smooth_corners(raw_corners) -> np.ndarray`
  - `compute_warp(smoothed_corners) -> Tuple[np.ndarray, np.ndarray]`
  - `draw_area_overlay(frame, corners) -> np.ndarray`
- [ ] **Config sabitleri**: `SMOOTHING_ALPHA` vb. yeni sabitler `config.py`'ye eklenmeli
- [ ] **DEBUG logları**: "kaç köşe bulundu", "smoothing uygulandı" gibi detay loglar

**Kabul Kriterleri:**
- 4 marker sabitken kuşbakışı görüntü kare-kare TİTREMEMELİ (smoothing doğrulaması)
- 4cm'lik ChArUco karesi kuşbakışıda ~40px görünmeli (outer-corner doğrulaması)
- Marker 10cm kaydırıldığında kuşbakışı 1sn içinde güncellenmeli (smoothing dengelemesi)
- Orijinal pencerede 4 köşeyi birleştiren yeşil dörtgen net görünmeli


GÖREV: vision/kinematics/ik_solver.py adında yeni bir modül oluştur.
Mevcut kod stiline uy (dataclass config, tip ipuçlu, docstring'li, logger).

config.py'ye ekle (SİMÜLASYON İÇİN VARSAYILAN — fiziksel kol/gerçek 
Wokwi ölçüleri belli olunca güncellenecek):
  LINK1_UZUNLUK_CM = 10.0    # omuz -> dirsek
  LINK2_UZUNLUK_CM = 10.0    # dirsek -> bilek
  TABAN_YUKSEKLIK_CM = 5.0   # taban -> omuz ekleni yuksekligi
  HEDEF_GRIPPER_ACISI_DERECE = 90.0   # gripper'in yere gore sabit tutulacagi aci

@dataclass
class IKSonucu:
    j1_derece: float
    j2_derece: float
    j3_derece: float
    j4_derece: float
    erisilebilir: bool   # hedef kolun erisim menzili disindaysa False

def hesapla_ik(x_cm: float, y_cm: float, z_cm: float, config: VisionConfig) -> IKSonucu:
    '''
    Yukaridaki 4 adimli formulu uygula (J1 atan2, J2/J3 kosinus teoremi, 
    J4 telafi). 
    
    ERISILEBILIRLIK KONTROLU (onemli): kosinus teoremi adiminda 
    cos(J3_ic) degeri [-1, 1] araligi disina cikarsa (hedef, 
    LINK1+LINK2 toplam uzunlugundan uzaksa VEYA |LINK1-LINK2|'den 
    yakinsa), bu hedef FIZIKSEL OLARAK ERISILEMEZ demektir. Bu durumda 
    ValueError FIRLATMA - IKSonucu.erisilebilir=False donup, aci 
    degerlerini 0 veya son gecerli degerde birak. Cagiran kod (server.py) 
    bu durumu kontrol edip robotu hareket ettirmemeli, kullaniciya/log'a 
    "hedef erisim disinda" uyarisi vermeli.
    
    Servo aci isaretlerini (yon) ve 0 noktasini, gercek 
    sketch.ino'daki servo kurulumuna gore ayarlaman gerekebilir - 
    bu formul matematiksel olarak dogru ama servonun hangi yonde 
    hangi aciya "0" dedigi donanima ozeldir.
    '''

KABUL KRİTERİ:
- Kolun tam ortasindaki erisilebilir bir nokta (orn. r=15cm, z=5cm) 
  icin J1-J4 degerleri hesaplaniyor ve erisilebilir=True donuyor.
- Kolun erisemeyecegi kadar uzak bir nokta (orn. r=100cm) icin 
  erisilebilir=False donuyor, HATA FIRLATMIYOR (server.py'nin 
  crash olmadan bu durumu yonetebilmesi icin).
- Ayni (x,y,z) icin fonksiyon iki kez cagirildiginda AYNI sonucu 
  veriyor (deterministik, rastgelelik yok). 


  GÖREV: ik_solver.py içine hesapla_fk() fonksiyonu ekle.

def hesapla_fk(j1_derece, j2_derece, j3_derece, config) -> Tuple[float,float,float]:
    '''
    IK'nin tersi: verilen eklem acilarindan uc nokta (X,Y,Z) hesaplar.
    Yukaridaki duz trigonometri formulunu kullan (kosinus teoremi 
    GEREKMEZ, FK icin daha basit).
    '''

TEST FONKSİYONU EKLE (test_ik_fk_tutarliligi):
Rastgele/bilinen 5-10 erisilebilir (x,y,z) noktasi icin:
  1. hesapla_ik(x,y,z) cagir
  2. Sonucu hesapla_fk()'ye ver
  3. Cikan (x',y',z') ile orijinal (x,y,z) arasindaki fark 
     0.01 cm'den (veya makul bir tolerans) kucuk olmali
Herhangi bir nokta bu testi gecemezse, IK formulunde hata var demektir 
- konsola hangi noktanin basarisiz oldugunu ve farkin ne kadar 
oldugunu yazdir.