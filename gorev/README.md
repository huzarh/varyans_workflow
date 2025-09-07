# 🚁 Yük Bırakma Sistemi

Bu klasör drone'un hedefe ulaştığında yük bırakma işlemlerini yöneten scriptleri içerir.

## 📁 Dosyalar

### �� mavi_yuk_servo.py
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

## 🎯 Çalışma Mantığı

1. **Hedef Algılama**: Kamera ile hedef tespit edilir
2. **Yönlendirme**: Hedef merkeze getirilir (40px tolerans)
3. **Yük Bırakma**: Hedef merkeze geldiğinde:
   - Hedef türüne göre servo PWM değeri ayarlanır
   - 2 saniye beklenir
   - Servo normal pozisyona döndürülür
   - Hedef temizlenir (yeni hedef aranır)

## 🚀 Kullanım

### Manuel Test
```bash
# Mavi yük testi
python3 gorev/mavi_yuk_servo.py

# Kırmızı yük testi
python3 gorev/kirmizi_yuk_servo.py
```

### Otomatik Sistem
Sistem `flight/controller.py` tarafından otomatik olarak çalıştırılır.

## ⚙️ Ayarlar

- **Merkez Toleransı**: 40 pixel
- **Servo Bekleme**: 2 saniye
- **Normal PWM**: 1500
- **Mavi Yük PWM**: 1600
- **Kırmızı Yük PWM**: 800
