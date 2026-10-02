# Final Spesifikasyon v6 — 5-Eklemli Robotik Kol Vision Modülü (Konsolide)

**Repo:** `rkybusee/5-eklemli-robotik-kol`
**Kapsam:** `kalibrasyon/vision/` paketi
**Güncelleme:** 2026-10-01 — Mevcut kod tabanıyla tam karşılaştırma yapıldı, tamamlanan / kalan görevler netleştirildi.

> **Bu dosya, önceki tüm dokümanların (v2 spesifikasyonu, düzeltmeler_v3, final v5) YERİNE geçer.** Kodlama ajanına artık sadece bu dosya verilmeli.

---

## 0. Genel Durum Özeti

| # | Görev | Durum | Dosya(lar) | Not |
|---|-------|-------|------------|-----|
| 1 | ID-bazlı köşe sıralama | ✅ Tamamlandı | `grab.py`, `config.py` | `_sort_corners_by_id()` fonksiyonu + fallback mevcut |
| 2 | Kayıp köşe kare sayacı + timeout | ✅ Tamamlandı | `grab.py`, `config.py` | `KOSE_KAYIP_MAX_KARE=30`, homografi sıfırlama + BEKLEME'ye zorlama |
| 3 | 1Euro Filter | ✅ Tamamlandı | `one_euro_filter.py`, `grab.py`, `config.py` | Tam entegrasyon, config parametreleri mevcut |
| 4 | OutlierGuard + `sifirla()` | ✅ Tamamlandı | `safety_trajectory.py`, `grab.py`, `config.py` | `sifirla()` metodu var, BEKLEME→TESPIT ve TAMAMLANDI geçişlerinde çağrılıyor |
| 5 | Filtre sırası (Outlier → 1Euro) | ✅ Tamamlandı | `grab.py` L390-403 | Doğru sıra: önce OutlierGuard, sonra 1Euro |
| 6 | IPPE_SQUARE (`grab.py`) | ✅ Tamamlandı | `grab.py` L299-303 | `get_3d_pos()` içinde `flags=cv2.SOLVEPNP_IPPE_SQUARE` mevcut |
| 7 | IPPE_SQUARE (`birds_eye_view.py`) | ❌ EKSİK | `birds_eye_view.py` L227 | Bayraksız `solvePnP` hâlâ kullanılıyor |
| 8 | Kübik Spline yörünge (YAKLASMA) | ✅ Tamamlandı | `grab.py`, `safety_trajectory.py` | İki aşamalı spline (park→güvenli→yaklaşma) mevcut |
| 9 | HIZALAMA durumu (IBVS, bilek marker) | ✅ Tamamlandı | `grab.py` L522-551 | Bilek marker (ID 5) takibi + kapalı çevrim düzeltme mevcut |
| 10 | Bilek marker tespiti | ✅ Tamamlandı | `grab.py` L264-268, L428-442 | Piksel→cm dönüşümü + dinamik ofset uygulanıyor |
| 11 | Taban marker (basit dinamik ofset) | ⚠️ Kısmen | `grab.py` L258-262, L405-425 | Basit her-kare ofset var ama **kilitleme yok** |
| 12 | `RobotTabanBulucu` sınıfı (`base_locator.py`) | ❌ EKSİK | — | Dosya hiç oluşturulmamış |
| 13 | Config'de taban/bilek marker boyut parametreleri | ❌ EKSİK | `config.py` | `TABAN_MARKER_BOYUTU_M`, `BILEK_MARKER_BOYUTU_M`, `TABAN_MARKER_YEREL_OFSET_X/Y_CM`, `TABAN_KILITLEME_KARE`, `TABAN_KAYIP_MAX_KARE` yok |

---

## 1. Final Marker ID Şeması

| ID | Anlamı | Yerleşim | Davranış | Config Karşılığı |
|---|---|---|---|---|
| **0** | Robot Taban Referansı | Robot tabanının **yanına** (üstüne değil), sabit | Bir kez ölçülüp **kilitlenir** | `ROBOT_BASE_MARKER_ID = 0` ✅ (mevcut) |
| **1, 2, 3, 4** | Masa köşe marker'ları | Masanın 4 köşesi | Her karede yeniden ölçülür | `KOSE_MARKER_IDLERI = [1, 2, 3, 4]` ✅ |
| **5** | Bilek/Gripper Marker'ı (IBVS) | Bilek eklemine, kameraya dönük | Her karede yeniden ölçülür | `BILEK_MARKER_ID = 5` ✅ |
| **6, 7, 8, 9...** | Hedef küp/nesne marker'ları | Tutulacak nesnelerin üzerinde | Her karede yeniden ölçülür | `HEDEF_MARKER_IDS = [6, 7, 8, 9]` ✅ |

---

## 2. Tamamlanmış Katmanlar (Referans)

Aşağıdaki katmanlar kodda zaten uygulanmış durumda. Değişiklik gerekmez, sadece referans olarak tutulur.

### 2.1 Katman 1 — ID-bazlı Köşe Sıralama + Kayıp Kare Yönetimi ✅

**Uygulanan dosyalar:** `grab.py` (`_sort_corners_by_id()`, `kayip_kare_sayaci`), `config.py` (`KOSE_ID_TO_POSITION`, `KOSE_KAYIP_MAX_KARE`)

- `_sort_corners_by_id()` fonksiyonu marker ID→fiziksel köşe eşlemesi yapıyor (kamera açısından bağımsız)
- ID-bazlı sıralama başarısız olursa (config hatası vb.) eski açı-bazlı `_sort_corners_clockwise` fallback devreye giriyor
- `len(detected_corner_pts) < 4` her karede `kayip_kare_sayaci` +1, 4 marker görülünce 0'a sıfırlanıyor
- Sayaç `KOSE_KAYIP_MAX_KARE` (30) aşarsa `matris = None` yapılıp robot BEKLEME'ye zorlanıyor

### 2.2 Katman 2 — 1Euro Filter ✅

**Uygulanan dosyalar:** `one_euro_filter.py`, `grab.py` (L198-204, L399-403), `config.py` (`ONEEURO_MIN_CUTOFF`, `ONEEURO_BETA`)

- `LowPassFilter` + `OneEuroFilter` sınıfları tam implementasyon
- `grab_mode()` başında `filtre_x`/`filtre_y` nesneleri oluşturuluyor
- OutlierGuard'dan sonra çağrılıyor (doğru sıra)

### 2.3 Katman 3 — OutlierGuard + Kübik Spline Yörünge ✅

**Uygulanan dosyalar:** `safety_trajectory.py`, `grab.py`, `config.py`

- `OutlierGuard.kontrol_et()` — medyan tabanlı kayan pencere, `(x, y, gecerli)` üçlüsü döndürüyor
- `OutlierGuard.sifirla()` — yeni hedefe kilitlenirken ve TAMAMLANDI'da çağrılıyor (Düzeltme 4 uygulanmış)
- `eklem_yorungesi_uret()` — CubicSpline, başlangıç/bitiş hızı 0
- `YAKLASMA` durumu iki aşamalı spline kullanıyor: `park → güvenli yükseklik → yaklaşma yüksekliği`

### 2.4 Katman 4 — IBVS Hizalama (Bilek Marker ile) ✅

**Uygulanan dosyalar:** `grab.py` (L522-551, L264-268, L428-442)

- `RobotDurum.HIZALAMA` enum değeri mevcut
- `YAKLASMA` tamamlanınca `HIZALAMA`'ya geçiliyor
- Bilek marker (ID 5) tespiti ve piksel→cm dönüşümü çalışıyor
- `hata = hedef - bilek` hesabı doğru yönde (Düzeltme 3 uygulanmış)
- `KAPALI_CEVRIM_TOLERANS_CM` ve `KAPALI_CEVRIM_HIZ` parametreleri config'de tanımlı

### 2.5 Düzeltme 1 — IPPE_SQUARE (grab.py) ✅

`get_3d_pos()` fonksiyonu `flags=cv2.SOLVEPNP_IPPE_SQUARE` kullanıyor.

### 2.6 Düzeltme 2 — Filtre Sırası ✅

Doğru sıra uygulanmış: önce OutlierGuard (L390-397), sonra 1Euro Filter (L399-403).

### 2.7 Düzeltme 4 — OutlierGuard Sıfırlama ✅

`outlier_guard.sifirla()` hem `BEKLEME→TESPIT` geçişinde (L452) hem `TAMAMLANDI` sonrası (L624) çağrılıyor.

---

## 3. KALAN GÖREVLER (Yapılması Gereken)

### 🔴 Görev A — `birds_eye_view.py` IPPE_SQUARE Düzeltmesi

**Öncelik:** Yüksek
**Durum:** ❌ Eksik

`birds_eye_view.py` satır 227'de hâlâ bayraksız `cv2.solvePnP()` kullanılıyor:

```python
# MEVCUT (L227):
success, _, tvec = cv2.solvePnP(obj_pts, corners_2d, camera_matrix, dist_coeffs)
```

**Yapılacak:**
```python
# DÜZELTİLMİŞ:
success, _, tvec = cv2.solvePnP(
    obj_pts, corners_2d, camera_matrix, dist_coeffs,
    flags=cv2.SOLVEPNP_IPPE_SQUARE
)
```

**`calibration_verifier.py` (L58):** Bu dosyada `solvePnP` ChArUco board pozunu çözüyor — kare marker değil, **dokunulmamalı**.

**Kabul Kriteri:** `birds_eye_view.py`'deki tüm tekil kare marker pozu çözen `solvePnP` çağrılarına `IPPE_SQUARE` eklenmeli.

---

### 🔴 Görev B — `base_locator.py` Oluşturulması ve Entegrasyonu

**Öncelik:** Yüksek
**Durum:** ❌ Dosya yok

Şu an `grab.py` (L405-425) basit bir her-kare dinamik ofset uyguluyor: taban marker (ID 0) her karede tespit edilir, homografi ile cm'ye çevrilir ve hedeften çıkarılır. **Ama kilitleme mekanizması yok** — taban fiziksel olarak sabit olmasına rağmen her karede tekrar ölçülüyor, bu da gürültüye açık.

**Yeni dosya: `vision/measurement/base_locator.py`**

```python
"""
Robot Taban Konumu Otomatik Tespiti

TABAN_MARKER_ID (varsayilan: 0), robotun tabanina YAKIN (yanina) yapistirilir.
Marker'in olculen konumuna, elle olculmus sabit bir yerel ofset (marker'in
kendi acisina gore dondurulerek) eklenerek robotun gercek J1 donme merkezinin
masa koordinat sistemindeki cm konumu bulunur. Bu deger fiziksel olarak sabit
oldugu icin bir kez kilitlenir, bir daha guncellenmez.
"""
import numpy as np
import math
from collections import deque


class RobotTabanBulucu:
    def __init__(self, yerel_ofset_x_cm, yerel_ofset_y_cm, kilitleme_kare=15, kayip_max_kare=60):
        self.yerel_ofset = np.array([yerel_ofset_x_cm, yerel_ofset_y_cm])
        self.kilitleme_kare = kilitleme_kare
        self.kayip_max_kare = kayip_max_kare
        self.olcum_gecmisi = deque(maxlen=kilitleme_kare)
        self.kilitli_konum = None
        self.kayip_sayaci = 0

    def _yerel_ofseti_dondur(self, aci_derece):
        aci_rad = math.radians(aci_derece)
        cos_a, sin_a = math.cos(aci_rad), math.sin(aci_rad)
        rot = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        return rot @ self.yerel_ofset

    def guncelle(self, marker_x_cm, marker_y_cm, marker_aci_derece):
        """Donus: (robot_x_cm, robot_y_cm, kilitli_mi: bool)"""
        if self.kilitli_konum is not None:
            self.kayip_sayaci = 0
            return self.kilitli_konum[0], self.kilitli_konum[1], True

        if marker_x_cm is None:
            self.kayip_sayaci += 1
            return None, None, False

        ofset_donmus = self._yerel_ofseti_dondur(marker_aci_derece)
        robot_x = marker_x_cm + ofset_donmus[0]
        robot_y = marker_y_cm + ofset_donmus[1]
        self.olcum_gecmisi.append((robot_x, robot_y))

        if len(self.olcum_gecmisi) >= self.kilitleme_kare:
            medyan = np.median(np.array(self.olcum_gecmisi), axis=0)
            self.kilitli_konum = (float(medyan[0]), float(medyan[1]))
            return self.kilitli_konum[0], self.kilitli_konum[1], True

        return robot_x, robot_y, False

    def kayip_mi(self):
        return self.kilitli_konum is None and self.kayip_sayaci > self.kayip_max_kare

    def sifirla(self):
        """Robot fiziksel olarak yeniden konumlandirildiginda manuel cagrilir."""
        self.olcum_gecmisi.clear()
        self.kilitli_konum = None
        self.kayip_sayaci = 0
```

**`grab.py`'ye entegrasyon:**

a) Import ekle:
```python
from vision.measurement.base_locator import RobotTabanBulucu
```

b) `grab_mode()` başında nesne oluştur:
```python
robot_taban_bulucu = RobotTabanBulucu(
    config.TABAN_MARKER_YEREL_OFSET_X_CM,
    config.TABAN_MARKER_YEREL_OFSET_Y_CM,
    config.TABAN_KILITLEME_KARE,
    config.TABAN_KAYIP_MAX_KARE,
)
```

c) Mevcut basit dinamik ofset bloğu (L405-425), `robot_taban_bulucu.guncelle()` çağrısıyla değiştirilmeli. Kilitleme tamamlanana kadar `BEKLEME`'de tutulmalı:
```python
if not taban_kilitli:
    durum = RobotDurum.BEKLEME
    # ekrana "Robot taban konumu tespit ediliyor..." yazılır
```

d) Koordinat geçişi: tüm `hesapla_ik()` çağrılarında hedef koordinatlar `robot_x_cm`/`robot_y_cm`'ye göre göreli hesaplanmalı:
```python
ik_x_cm = hedef_x_cm - robot_x_cm
ik_y_cm = hedef_y_cm - robot_y_cm
```

**Kabul Kriteri:** Program başladığında taban marker 15 kare (konfigüre edilebilir) boyunca tutarlı konumda görünmeli, ardından "Robot taban tespit edildi: (X, Y) cm" logu basılmalı ve konum kilitlenmeli. Kilitlendikten sonra taban marker geçici olarak kapansa bile kilitli değer kullanılmaya devam etmeli.

---

### 🔴 Görev C — Config'e Eksik Parametrelerin Eklenmesi

**Öncelik:** Yüksek (Görev B'nin ön koşulu)
**Durum:** ❌ Eksik

`config.py`'ye aşağıdaki parametreler eklenmeli:

```python
# --- Marker Boyutları (yeni) ---
TABAN_MARKER_BOYUTU_M: float = 0.061     # köşelerle aynı varsayım, gerekirse değiştirilir
BILEK_MARKER_BOYUTU_M: float = 0.025     # küp marker'larıyla aynı küçük boyut

# --- Otomatik Taban Tespiti (yeni) ---
TABAN_MARKER_YEREL_OFSET_X_CM: float = -8.0   # KULLANICI KENDİ ÖLÇÜMÜYLE DEĞİŞTİRECEK
TABAN_MARKER_YEREL_OFSET_Y_CM: float = 0.0    # KULLANICI KENDİ ÖLÇÜMÜYLE DEĞİŞTİRECEK
TABAN_KILITLEME_KARE: int = 15
TABAN_KAYIP_MAX_KARE: int = 60
```

Ayrıca `get_marker_boyutu()` fonksiyonu güncellenmeli:

```python
def get_marker_boyutu(self, marker_id: int) -> float:
    if marker_id == self.ROBOT_BASE_MARKER_ID:
        return self.TABAN_MARKER_BOYUTU_M
    if marker_id == self.BILEK_MARKER_ID:
        return self.BILEK_MARKER_BOYUTU_M
    if marker_id in self.KOSE_MARKER_IDLERI:
        return self.KOSE_MARKER_BOYUTU_M
    return self.KUP_MARKER_BOYUTU_M
```

---

### 🟡 Görev D — `KOSE_ID_TO_POSITION` Manuel Doğrulama (Kod Değişikliği Değil)

**Öncelik:** Düşük (fiziksel kurulum)
**Durum:** Kullanıcı tarafından doğrulanacak

Kullanıcı, `--mode grab` çalıştırmadan önce:
1. Köşelere ID 1, 2, 3, 4 marker'larını yapıştırmalı.
2. Ekranda beliren "K1 (sol_ust)", "K2 (sag_ust)" gibi etiketlerin fiziksel masadaki gerçek köşelerle eşleştiğini **gözle** doğrulamalı.
3. Eşleşmiyorsa `config.py`'deki `KOSE_ID_TO_POSITION` sözlüğünü gerçek yerleşime göre düzeltmeli.

Bu adım kamera açısından bağımsızdır (ID-bazlı sıralama kullanıldığı için) — sadece **başlangıç kurulumunun** doğruluğunu garanti eder.

---

## 4. Fiziksel Kurulum Kontrol Listesi (Kod Değil, Donanım)

- [ ] ID 0 marker'ı → robot tabanının **yanına** (üstüne değil) yapıştırıldı
- [ ] ID 0 ile robot J1 ekseni arası mesafe cetvelle ölçüldü → `TABAN_MARKER_YEREL_OFSET_X/Y_CM`'ye girildi
- [ ] ID 1, 2, 3, 4 marker'ları masanın 4 köşesine yapıştırıldı
- [ ] ID 5 marker'ı bilek eklemine (gripper gövdesine, kameraya dönük yüzeye) yapıştırıldı
- [ ] ID 6, 7, 8, 9... hedef nesnelerin üzerine yapıştırıldı
- [ ] Eski ID şeması kullanılan tüm fiziksel marker kağıtları yeni şemaya göre değiştirildi/yeniden basıldı

---

## 5. Değiştirilecek / Eklenecek Dosyalar — Final Özet

| Dosya | İşlem | Durum |
|---|---|---|
| `vision/config.py` | Görev C: `TABAN_MARKER_BOYUTU_M`, `BILEK_MARKER_BOYUTU_M`, `TABAN_MARKER_YEREL_OFSET_X/Y_CM`, `TABAN_KILITLEME_KARE`, `TABAN_KAYIP_MAX_KARE` eklenmeli + `get_marker_boyutu()` güncellenmeli | ❌ |
| `vision/measurement/base_locator.py` | Görev B: **Yeni dosya** — `RobotTabanBulucu` sınıfı | ❌ |
| `vision/measurement/birds_eye_view.py` | Görev A: `solvePnP` → `IPPE_SQUARE` bayrağı eklenmeli (L227) | ❌ |
| `vision/measurement/grab.py` | Görev B: `base_locator` import'u + `RobotTabanBulucu` entegrasyonu (mevcut basit ofset bloğunun yerini alacak) | ❌ |
| `vision/measurement/one_euro_filter.py` | Değişiklik yok | ✅ |
| `vision/measurement/safety_trajectory.py` | Değişiklik yok | ✅ |
| `vision/calibration/calibration_verifier.py` | `solvePnP` ChArUco board pozu — dokunulmaz | ✅ (dokunma) |

---

## 6. Uygulama Sırası

1. **Görev C** (config parametreleri) — bunlar olmadan Görev B çalışmaz
2. **Görev A** (`birds_eye_view.py` IPPE_SQUARE) — bağımsız, hemen yapılabilir
3. **Görev B** (`base_locator.py` + `grab.py` entegrasyonu) — Görev C tamamlandıktan sonra

---

## 7. Kodlama Ajanı İçin Notlar

1. **Geriye dönük uyumluluk:** `hesapla_ik()`, `hesapla_fk()`, `_servo_gonder()`, `_kamera_piksel_to_raw_cm()` fonksiyon imzaları **değiştirilmemeli** — sadece koordinat kaynağı güncelleniyor.
2. **Test sırası:** Önce Görev A uygulanıp `--mode birds-eye-view` ile marker salınımının azaldığı gözlenmeli. Sonra Görev C+B uygulanıp taban kilitleme logları doğrulanmalı.
3. **Fiziksel ön koşul:** Görev B tam çalışmadan önce kullanıcının §4'teki fiziksel kurulum listesini tamamlamış olması gerekir.
4. **Loglama:** Yeni eklenen her adımda mevcut `logger.info`/`logger.debug` düzenine uyulmalı.
5. **`plan.md` güncellemesi:** Tüm görevler tamamlandığında `kalibrasyon/plan.md`'deki marker ID tablosu güncellenmeli.

---# Görev D (Ek) — Bilek Marker (ID 5) Filtrelemesi

**Bu dosya, Final Spesifikasyon v5/v6'ya ektir.** Antigravity'nin tespit ettiği "taban ve bilek ölçümleri ham, filtresiz" bulgusunun bilek (ID 5) kısmını çözer. Taban (ID 0) kısmı zaten Görev B'de (`base_locator.py` + kilitleme) çözülüyor — kilitlendikten sonra taban ölçümü donduğu için ayrıca filtreye ihtiyaç duymuyor. Bilek ise hiçbir zaman kilitlenmiyor (sürekli hareket ediyor), bu yüzden **ayrı bir filtreleme stratejisi** gerekiyor.

---

## Neden Hedef İçin Kullanılan Filtre Doğrudan Kopyalanamaz

Hedef marker için kullanılan `filtre_x`/`filtre_y` (`min_cutoff=1.0`, `beta=0.015`), **"nesne çoğunlukla sabit duruyor"** varsayımına göre ayarlanmış — durgunken sıkı, hareket ederken gevşer.

Bilek marker ise `HIZALAMA` durumunda **aktif bir kapalı çevrim kontrol döngüsünün içinde** — kol sürekli küçük düzeltme hareketleri yapıyor. Aynı sıkı ayarları kullanırsan:
- Filtre, gerçek düzeltme hareketini de "gürültü" sanıp geciktirir.
- Kapalı çevrim gecikmeli geri bildirim alır → **salınım (oscillation)** riski — kol hedefe yakınsamak yerine ileri-geri titreyerek yaklaşabilir.

Bu yüzden bilek filtresi **daha gevşek** (daha az agresif, daha az gecikme) ayarlanmalı — tam tersi mantıkla: "her zaman hafif yumuşat, tepkiselliği neredeyse hiç kaybetme."

---

## 1. Yeni Config Parametreleri (`config.py`)

```python
# --- Bilek Marker (ID 5) Filtrelemesi ---
# Hedef filtresinden FARKLI, daha gevsek ayarlar (IBVS tepkiselligini korumak icin)
BILEK_OUTLIER_PENCERE_BOYUTU: int = 3     # hedefin 5'ine gore daha kucuk pencere
BILEK_OUTLIER_ESIK_CM: float = 2.0        # hedefin 3.0cm'ine gore daha siki (bilek hareketleri kucuk adimli)

BILEK_ONEEURO_MIN_CUTOFF: float = 2.0     # hedefin 1.0'ina gore daha YUKSEK -> durgunken bile az filtreleme
BILEK_ONEEURO_BETA: float = 0.05          # hedefin 0.015'ine gore daha YUKSEK -> harekete hizli tepki
```

**Neden bu değerler:**
- `BILEK_OUTLIER_PENCERE_BOYUTU=3` (hedefteki 5 yerine): Bilek sürekli hareket ettiği için uzun bir geçmiş penceresi, "yeni ama gerçek" bir hareketi de yanlışlıkla outlier sayabilir. Daha kısa pencere, gerçek hareketi daha hızlı "normal" kabul eder.
- `BILEK_OUTLIER_ESIK_CM=2.0` (hedefteki 3.0 yerine): Mikro düzeltmeler zaten küçük adımlarla ilerliyor (`KAPALI_CEVRIM_HIZ=0.4` ile orantılı), bu yüzden daha büyük bir sıçrama gerçek harekete değil, ölçüm hatasına işaret eder — eşik biraz sıkılaştırılabilir.
- `BILEK_ONEEURO_MIN_CUTOFF=2.0` (hedefteki 1.0 yerine): Durgun haldeyken bile daha az yumuşatma — IBVS'in "hareket ettim mi etmedim mi" ayrımını hızlı yapabilmesi için.
- `BILEK_ONEEURO_BETA=0.05` (hedefteki 0.015 yerine): Hıza karşı çok daha duyarlı — gerçek düzeltme hareketleri geldiğinde filtre neredeyse hemen gevşeyip gecikmeyi minimuma indirir.

---

## 2. `grab.py`'ye Entegrasyon

### 2.1 Kurulum (diğer filtre/guard nesneleriyle aynı yerde, `grab_mode()` başında)

```python
# Bilek (ID 5) icin ayri, IBVS'e uygun filtreler
bilek_outlier_guard = OutlierGuard(
    window_size=config.BILEK_OUTLIER_PENCERE_BOYUTU,
    esik_cm=config.BILEK_OUTLIER_ESIK_CM
)
filtre_bilek_x = OneEuroFilter(
    t0=time.time(), x0=0.0,
    min_cutoff=config.BILEK_ONEEURO_MIN_CUTOFF,
    beta=config.BILEK_ONEEURO_BETA
)
filtre_bilek_y = OneEuroFilter(
    t0=time.time(), x0=0.0,
    min_cutoff=config.BILEK_ONEEURO_MIN_CUTOFF,
    beta=config.BILEK_ONEEURO_BETA
)
```

### 2.2 Bilek Ölçümünün Filtrelenmesi (mevcut "Bilek (ID 5) Ölçümü" bloğu, ~L430-442 civarı)

**Mevcut (ham) hali:**
```python
raw_bilek_x, raw_bilek_y = _kamera_piksel_to_raw_cm(...)
bilek_x_cm = raw_bilek_x - base_x_cm   # veya ilgili offset islemi
bilek_y_cm = raw_bilek_y - base_y_cm
```

**Düzeltilmiş hali:**
```python
raw_bilek_x, raw_bilek_y = _kamera_piksel_to_raw_cm(...)

if raw_bilek_x is not None:
    # Adim 1: Glitch koruması (OutlierGuard) — ONCE
    raw_bilek_x, raw_bilek_y, bilek_gecerli = bilek_outlier_guard.kontrol_et(
        raw_bilek_x, raw_bilek_y
    )
    # Adim 2: Hafif yumusatma (1Euro) — SONRA
    t_simdi = time.time()
    raw_bilek_x = filtre_bilek_x(t_simdi, raw_bilek_x)
    raw_bilek_y = filtre_bilek_y(t_simdi, raw_bilek_y)

    bilek_x_cm = raw_bilek_x - base_x_cm
    bilek_y_cm = raw_bilek_y - base_y_cm
else:
    bilek_x_cm, bilek_y_cm = None, None
```

> **Dikkat — Filtre Sırası Burada da Aynı Mantıkla Korunuyor:** Hedef filtrelemesindeki (Düzeltme 2, Final Spesifikasyon §3) prensip burada da geçerli: **önce OutlierGuard, sonra 1Euro**. Sıra tersine çevrilirse, bilek filtresi de kendi iç durumunu bir glitch ile kirletebilir.

### 2.3 Guard Sıfırlama (Tutarlılık İçin)

Hedef `outlier_guard`'ı her yeni hedefe kilitlenirken (`BEKLEME`→`TESPIT`) sıfırlanıyordu (Final Spesifikasyon §6). Aynı mantık `bilek_outlier_guard` için de uygulanmalı — her yeni yakalama döngüsü başladığında eski bilek geçmişi temizlenmeli:

```python
if durum == RobotDurum.BEKLEME:
    if mevcut_hedef_x is not None:
        durum = RobotDurum.TESPIT
        # ... mevcut kod ...
        outlier_guard.sifirla()
        bilek_outlier_guard.sifirla()      # YENİ
```

---

## 3. Kabul Kriterleri

1. **Titreme azalması:** `HIZALAMA` durumunda, kol hedefe yaklaşırken ekrana basılan `hata_mesafe` değerinin tek karede büyük sıçramalar yapmaması (önceki hale göre daha düz bir azalma eğrisi göstermesi).
2. **Gecikme kontrolü:** Mikro düzeltme komutlarının hâlâ makul bir hızda hedefe yakınsadığı gözlenmeli — eğer filtre aşırı gecikme yaratıyorsa (`HIZALAMA` durumunda beklenenden çok daha fazla iterasyon/süre geçiyorsa), `BILEK_ONEEURO_BETA` değeri artırılmalı (daha az gecikme, biraz daha fazla titreme kabul edilir).
3. **Salınım (oscillation) kontrolü:** Kol hedefe yaklaşırken ileri-geri salınım yapmamalı. Eğer hâlâ salınım gözlenirse, `KAPALI_CEVRIM_HIZ` (mikro düzeltme oranı) düşürülmeli — bu filtreleme sorunu değil, kontrol döngüsü kazancı (gain) sorunu olabilir, ayrı bir ayar noktası.

---

## 4. Değiştirilecek Dosyalar — Ek Özet

| Dosya | İşlem |
|---|---|
| `vision/config.py` | §1 — `BILEK_OUTLIER_*` ve `BILEK_ONEEURO_*` parametreleri eklenir |
| `vision/measurement/grab.py` | §2.1 — `bilek_outlier_guard`, `filtre_bilek_x/y` nesneleri oluşturulur; §2.2 — bilek ölçüm bloğu filtrelerden geçirilir; §2.3 — `BEKLEME`→`TESPIT` geçişinde `bilek_outlier_guard.sifirla()` eklenir |

---

## 5. Kodlama Ajanı İçin Not

Bu değişiklik **Görev B (base_locator.py) ile birlikte** test edilmeli — çünkü `bilek_x_cm` hesabı `base_x_cm`'ye bağımlı, ve taban ölçümü kilitlenmeden (Görev B tamamlanmadan) bilek ölçümündeki iyileşme net gözlemlenemeyebilir (taban hâlâ titriyorsa, bilek filtrelense bile toplam sonuç titrek kalır). Test sırası: önce Görev B (taban kilitleme), sonra bu doküman (Görev D, bilek filtreleme) — birlikte uygulanıp birlikte test edilmesi en sağlıklısı.

---

**Doküman Sonu.**

**Doküman Sonu.**