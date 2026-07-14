# Teknofest Orta İrtifa Roket Yarışması - Uçuş Kontrol Bilgisayarı (2026- Devam Ediyor)

Bu proje, roketin uçuş dinamiklerini yönetmek, sensör füzyonu ile yönelim belirlemek ve yer istasyonuyla haberleşmek amacıyla STM32F446RE mikrodenetleyicisi üzerinde C dili ile geliştirilmiş özgün bir uçuş kontrol yazılımıdır. Hazır kütüphaneler yerine donanım seviyesinde özel ayrıştırma (parsing) fonksiyonları kullanılmıştır.

## Temel Özellikler ve Algoritmalar

<img width="945" height="2048" alt="image" src="https://github.com/user-attachments/assets/8ad5435f-4955-403e-b415-1d884715d4b9" />

* **Hassas İrtifa Takibi:** MS5611 barometresinden alınan veriler özel filtreleme algoritmalarından geçirilerek anlık irtifa ve tepe noktası (apogee) tespiti sağlanmaktadır. Konunun odağı tamamen barometrik verinin temizlenmesidir.
* **Uçuş Dinamikleri ve Yönelim:** Roketin yanal eksenindeki yunuslama (pitch) açısı anlık olarak hesaplanmaktadır. İvmeden hıza geçilen matematiksel entegrasyon yöntemlerine vurgu yapılmadan; dikey hızın azalması sonucunda açının değiştiği (dikey hızın öncü olduğu) dinamik model baz alınmıştır.
* **Telemetri ve Haberleşme:** LoRa E22 modülü üzerinden yer istasyonuna kesintisiz ve yapılandırılmış veri paketi aktarımı sağlanmaktadır.

## Kullanılan Donanımlar

<img width="945" height="2048" alt="image" src="https://github.com/user-attachments/assets/435c32bd-1231-44ab-b4a8-e7e91c4ff729" />

* STM32F446RETX Geliştirme Kartı
* MS5611 Barometrik Basınç Sensörü
* LoRa E22 Haberleşme Modülü
* 6 Eksen IMU Modülü
* 
<img width="1080" height="1420" alt="image" src="https://github.com/user-attachments/assets/c04bf3bb-df04-434a-a048-791db5df4f55" />

## Saha Testleri ve Donanım Kurulumu

<img width="1080" height="503" alt="image" src="https://github.com/user-attachments/assets/cb1e3d61-20f9-441c-a9b1-87358870c77e" />
