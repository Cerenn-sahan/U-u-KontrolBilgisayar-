/*
#include "flight_control.h" // Kendi vitrinine bakması lazım
#include <math.h>           // sin, cos hesapları için
#include "main.h"
// 'extern' demek: "Bu paket değişkeni aslında main.c'de duruyor, oradan bak" demek.
extern volatile RoketTelemetri paket;

// main.c'deki static değişkenleri de buraya taşıdık
static float max_irtifa = 0.0f;
static uint8_t dusus_sayaci = 0;
void Ucus_Durum_Kontrol(void) {

if (paket.ucus_durumu == 0) {
  	                                      	if (paket.irtifa_metre > max_irtifa) {
  	                                      	 max_irtifa = paket.irtifa_metre;
  	                                      	  }

  	                                      // Güvenlik Kilidi: İrtifa > 1000 m (Evet)
  	                                      if (paket.irtifa_metre > 1000.0f) {

  	                                      // Açı Kontrolü: 60 < Burun açısı < 120 (Evet)
  	                                      if (paket.pitch_acisi < 60.0f && paket.pitch_acisi > 120.0f) {

  	                                    	// Ardışık N ölçüm düşüş göstermeli

  	                                    	if (paket.irtifa_metre < (max_irtifa - 3.0f)) {
  	                                    	    dusus_sayaci++;
  	                                    	} else {
  	                                    	    dusus_sayaci = 0; // Gürültüyse sıfırla
  	                                    	}
  	                                    	if (dusus_sayaci >= 3) { // 3 ardışık 10ms = 30ms düşüş konfirmasyonu
  	                                    	    paket.ucus_durumu = 1;
  	                                    	}

  	                                      	    // BURAYA 1. PARAŞÜT RÖLE/MOSFET TETİKLEME KODU GELECEK
  	                                      	     // Örn: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  	                                      //irtifa düşüşü
  	                                  }//açı 60 120
  	                                }// 1000 metre koşulu
  	                              } //paket ucus durumu0

  	                                      // Akış Şeması: Kontrol == 1 mi? (Evet: Yani durum 1 ise)
  	                                        else if (paket.ucus_durumu == 1) {

  	                                        // Sadece irtifa kontrolü: İrtifa < 600 m (Evet)
  	                                          if (paket.irtifa_metre < 600.0f) {

  	                                          // Ana Paraşütü Aç ve Bitir
  	                                          paket.ucus_durumu = 2;

  	                                            // BURAYA 2. PARAŞÜT RÖLE/MOSFET TETİKLEME KODU GELECEK
  	                                             // Örn: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  	                                          } //paket uçuş duurmu 2
  	                                        }//paket ucus durumu 1

}
*/

#include "Flight_Control.h"
#include <math.h>
#include "main.h"

extern volatile RoketTelemetri paket;

// Test Hedefleri
#define TEST_APOGEE_IRTIFA_M 2500.128f // 12110 feet
#define TEST_ANA_PARASUT_M   600.0f    // 600 metre altı

static float max_irtifa = 0.0f;
static uint8_t dusus_sayaci = 0;

void Ucus_Durum_Kontrol(void) {



	    // Her durumda maksimum irtifayı kaydet (Apogee takibi)
	    if (paket.irtifa_metre > max_irtifa) {
	        max_irtifa = paket.irtifa_metre;
	    }

	    // ==============================================================
	    // DURUM 0: Rampa ve Tırmanış (Apogee Bekleniyor)
	    // ==============================================================
	    if (paket.ucus_durumu == 0) {

	        // Güvenlik kilidi: Sadece belirli bir irtifanın üzerindeysen tepe noktası ara (Örn: 1000m)
	        // Eğer roket çok düşük irtifalarda değilse ve açı 60-120 derece arasındaysa:
	        if (paket.irtifa_metre > 1000.0f &&
	            paket.pitch_acisi >= 60.0f &&
	            paket.pitch_acisi <= 120.0f) {

	            // DÜŞÜŞ DOĞRULAMASI (Tepe noktasından en az 3 metre aşağı düştü mü?)
	            if (paket.irtifa_metre < (max_irtifa - 3.0f)) {
	                dusus_sayaci++;
	            } else {
	                dusus_sayaci = 0; // Gürültüyse sıfırla
	            }

	            // 3 ardışık 10ms ölçümde düşüş doğrulandıysa (30ms) 1. KADEME AÇ!
	            if (dusus_sayaci >= 3) {
	                paket.ucus_durumu = 1;

	                // 1. Paraşüt (Sürükleme) LED'i
	                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // PA5 YANDI
	            }
	        }
	    }

	    // ==============================================================
	    // DURUM 1: Düşüş (Ana Paraşüt Bekleniyor)
	    // ==============================================================
	    else if (paket.ucus_durumu == 1) {

	        // Şart: İrtifa 600 metrenin altına düştü mü?
	        if (paket.irtifa_metre < 600.0f) {

	            paket.ucus_durumu = 2; // Uçuş tamamlandı

	            // 2. Paraşüt (Ana) LED'i
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // PA6 YANDI
	        }
	    }
	}
