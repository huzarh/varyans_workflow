# 🚁 Yük Bırakma ve RTL Sistemi

Bu klasör drone'un hedefe ulaştığında yük bırakma işlemlerini yöneten ve görev tamamlandığında RTL moduna geçiren scriptleri içerir.

## 📁 Dosyalar

### 🔵 mavi_yuk_servo.py
- **Amaç**: Mavi yük bırakma
- **Servo**: 7 numaralı servo
- **PWM Değerleri**:
  - Normal pozisyon: 1500 PWM
  - Yük bırakma: 1600 PWM

### 🔴 kirmizi_yuk_servo.py
- **Amaç**: Kırmızı yük bırakma
- **Servo**: 7 numaralı servo
- **PWM Değerleri**:
  - Normal pozisyon: 1500 PWM
  - Yük bırakma: 800 PWM

### 🏠 rtl_mode.py
- **Amaç**: RTL (Return to Launch) moduna geçiş
- **Mod ID**: 6
- **Fonksiyon**: Görev tamamlandığında drone'u eve döndürür

## 🎯 Çalışma Mantığı

### 1. Hedef Algılama ve Yönlendirme
1. **Hedef Algılama**: Kamera ile hedef tespit edilir
2. **Yönlendirme**: Hedef merkeze getirilir (40px tolerans)
3. **Yük Bırakma**: Hedef merkeze geldiğinde yük bırakılır

### 2. Yük Bırakma Süreci
1. Hedef türüne göre servo PWM değeri ayarlanır
2. 2 saniye beklenir
3. Servo normal pozisyona döndürülür
4. Yük sayacı artırılır
5. Hedef temizlenir

### 3. Görev Tamamlama ve RTL
1. **Maksimum Yük**: 2 adet
2. **Görev Tamamlama**: 2 yük bırakıldığında
3. **RTL Geçiş**: Otomatik olarak RTL moduna geçer
4. **Eve Dönüş**: Drone otomatik olarak kalkış noktasına döner

## 🚀 Kullanım

### Manuel Test
```bash
# Mavi yük testi
python3 gorev/mavi_yuk_servo.py

# Kırmızı yük testi
python3 gorev/kirmizi_yuk_servo.py

# RTL mod testi
python3 gorev/rtl_mode.py
```

### Otomatik Sistem
Sistem `flight/controller.py` tarafından otomatik olarak çalıştırılır.

## ⚙️ Ayarlar

### Yük Bırakma
- **Merkez Toleransı**: 40 pixel
- **Servo Bekleme**: 2 saniye
- **Normal PWM**: 1500
- **Mavi Yük PWM**: 1600
- **Kırmızı Yük PWM**: 800

### Görev Yönetimi
- **Maksimum Yük**: 2 adet
- **RTL Mod ID**: 6
- **Otomatik Geçiş**: Görev tamamlandığında

## 📊 Durum Takibi

Sistem aşağıdaki durumları takip eder:
- `target_detected`: Hedef algılandı mı?
- `cargo_dropped`: Kaç yük bırakıldı?
- `mission_completed`: Görev tamamlandı mı?
- `max_cargo`: Maksimum yük sayısı

## 🔄 Görev Döngüsü

```
Hedef Algılama → Yönlendirme → Yük Bırakma → Sayac Artırma
       ↓
Görev Tamamlandı mı? → EVET → RTL Moduna Geç → Eve Dönüş
       ↓
      HAYIR → Yeni Hedef Ara
```
