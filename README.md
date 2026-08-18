# 🤖 Otonom Robotik Kol Projesi

Bu proje, görüntü işleme ve ters kinematik (Inverse Kinematics) kullanarak masadaki cisimleri tespit edip otonom olarak kavrayabilen, 4 eksenli (ve 1 tutucu) servo motor tabanlı bir robotik kol sistemidir. Proje aynı zamanda manuel olarak bir web arayüzü üzerinden de kontrol edilebilir.

## 🚀 Proje Mimarisi (5 Katmanlı Yapı)

Proje uçtan uca çalışacak şekilde modüler bir mimariye sahiptir:

1. **Görüntü İşleme Katmanı (OpenCV & ArUco):** Kameradan alınan görüntü üzerinden çalışma alanındaki (masadaki) köşe marker'ları ve nesne marker'ları algılanır. Kuşbakışı görünüm (Bird's-Eye View) ve homografi teknikleriyle nesnelerin piksel bazlı konumları, gerçek dünya (cm) koordinatlarına dönüştürülür.
2. **Koordinat Dönüşümü:** Masadaki nesnelerin merkez koordinatları, robot kolun tabanına (J1 eksenine) göre ofsetlenerek dönüştürülür.
3. **Ters Kinematik (IK - Inverse Kinematics):** Robotun ulaşması istenen 3 boyutlu uzay koordinatları (X, Y, Z), kosinüs teoremi tabanlı çalışan IK motoru ile J1, J2, J3 ve J4 eklem açılarına çevrilir.
4. **Sunucu & Kontrol Paneli (Flask):** 
   - Python Flask tabanlı sunucu (`server.py`), web üzerinden (tarayıcıdan veya telefondan) robota erişim sağlar.
   - Ngrok desteğiyle dış ağlara açılabilir.
   - Görüntü işlemeden ve manuel kontrol arayüzünden gelen açı komutlarını USB Seri port üzerinden Arduino'ya iletir.
5. **Gömülü Sistem (Arduino):** 
   - `sketch.ino` (veya `PCA9685_arduino_kodu`), seri porttan gelen açı komutlarını işler.
   - Fiziksel Servo motorlara uygun sinyalleri (PWM) göndererek donanımı hareket ettirir. Donanım olmadığı durumlarda Wokwi üzerinden tamamen simüle edilebilir.

## 📁 Klasör Yapısı

* **`html/`**: Robot kolun manuel olarak kontrol edilebildiği web arayüzü ve Flask sunucusunu barındırır.
* **`kalibrasyon/`**: Projenin görüntü işleme, otonom kalibrasyon ve ters kinematik (IK) motorunu barındıran temel klasör.
* **`kalibrasyon/vision/`**: Ana modüler Python kütüphanesi. Kamera yapılandırması, ölçüm ve kinematik modüllerini içerir.
* **`PCA9685_arduino_kodu/` / `wokwi_proje/`**: Robot kolu fiziksel olarak hareket ettirecek olan C++ Arduino kodları ve simülasyon ayarları.

## 🛠 Gereksinimler

Projenin tam kapasiteyle çalışabilmesi için aşağıdaki yazılım ve donanımlara ihtiyacınız vardır:

* **Donanım:** 
  * 4 veya 5 Eksenli (J1-J5) Servo Motorlu Robotik Kol
  * Arduino (Örn: Uno/Nano) ve PCA9685 Servo Sürücü Kartı (Opsiyonel)
  * Web Kamerası (Masa yüzeyini tepeden veya karşıdan net görebilecek şekilde)
* **Yazılım:** 
  * Python 3.8 veya üzeri
  * Arduino IDE (veya VS Code Arduino eklentisi)
  * Kalibrasyon için yazdırılmış ChArUco ve ArUco marker kağıtları

## 📝 Adım Adım Başlangıç Rehberi

Sistemi baştan sona ayağa kaldırmak için aşağıdaki adımları **sırasıyla** izlemelisiniz:

### Adım 1: Arduino'nun Hazırlanması
Fiziksel donanımı bağlamadan veya kullanmadan önce Arduino'yu hazırlamamız gerekir:
1. `PCA9685_arduino_kodu/` klasörünün içindeki `.ino` kodunu Arduino IDE ile açın.
2. Arduino'nuzu USB ile bilgisayara bağlayıp bu kodu içine yükleyin.
3. Servo motorlarınızı koda uygun şekilde pinlere takın. *(Donanımınız henüz hazır değilse `wokwi_proje/` içerisindeki simülasyon araçlarıyla sanal olarak da test edebilirsiniz.)*

### Adım 2: Web Kontrol Paneli (Flask Sunucusu)
Robotik kolunuzu internet tarayıcınızdan manuel hareket ettirmek ve diğer servislerle haberleştirmek için Flask sunucusunu başlatmalısınız.

1. Bir terminal (CMD veya PowerShell) açın ve `html` klasörüne girin:
   ```bash
   cd html
   ```
2. Gerekli Python kütüphanelerini indirin (sadece ilk seferde):
   ```bash
   pip install -r Requirements.txt
   ```
3. Sunucuyu başlatın:
   ```bash
   python server.py
   ```
4. Tarayıcınızda `http://127.0.0.1:5000` adresine girerek manuel kontrol arayüzüne ulaşabilirsiniz.

### Adım 3: Görüntü İşleme ve Otonom Yakalama (Vision Modülü)
Kameranın nesneleri algılayıp robotun otonom hareket etmesini sağlayan beyin kısmı buradadır. Yeni bir terminal sekmesi açarak şu adımları izleyin:

1. `kalibrasyon` klasörüne girin:
   ```bash
   cd kalibrasyon
   ```
2. Görüntü işleme kütüphanelerini indirin (sadece ilk seferde):
   ```bash
   pip install -r requirements.txt
   ```

3. **Öncelikle Kameranızı Kalibre Edin:**
   İlk kullanımda kameranızın balık gözü (distorsiyon) etkisini sıfırlamak ve mercek özelliklerini hesaplamak için bir kalibrasyon işlemi yapmalısınız.
   ```bash
   python -m vision.main --mode capture           # ChArUco kağıdını kameraya tutarak fotoğraf çekin
   python -m vision.main --mode calibrate         # Çektiğiniz fotoğraflarla matematiksel matrisi hesaplayın
   ```
   *(Bu işlem başarılı olunca `calibration_result.yaml` adlı bir dosya oluşacaktır.)*

4. **Otonom Yakalama (Visual Servoing / IK):**
   Kalibrasyon bittikten sonra robot kolun kameradaki nesneyi bulup kavraması için ana modu çalıştırın:
   ```bash
   python -m vision.main --mode grab
   ```
   Bu komut kamerayı açar, masadaki ArUco marker'lı nesneyi bulur, X-Y-Z koordinatlarını hesaplayarak robot koldaki Flask sunucusuna açı komutlarını gönderir ve kolun cismi tutmasını sağlar.

## 🤝 Geliştirme Ortamı
Bu proje geliştirilirken Antigravity IDE (Wokwi entegrasyonu dahil) kullanılarak donanım simülasyonlarıyla test edilmiş ve tam otonom süreçler oluşturulmuştur.
