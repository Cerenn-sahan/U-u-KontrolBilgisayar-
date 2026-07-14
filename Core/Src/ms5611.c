#include "ms5611.h"
#include "main.h"
extern volatile RoketTelemetri paket;
// main.c'deki I2C donanımını burada kullanabilmek için:
extern I2C_HandleTypeDef hi2c1;

// ---DEĞİŞKENLER BURAYA GELDİ ---
// Başına 'static' koyuyoruz ki main.c bunları görüp kafası karışmasın. Sadece bu dosya kullanacak.
volatile uint32_t ms5611_tetik_zamani = 0;
static uint16_t C[8];
static int32_t dT, TEMP, P;
static int64_t OFF, SENS;
volatile uint32_t D1 = 0, D2 = 0;
volatile uint8_t ms5611_durum = 0;


void MS5611_Tetikle(uint8_t cmd) {
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 50);
}

uint32_t MS5611_Oku_Hazir() {
    uint8_t parca[3] = {0, 0, 0};
    //uint8_t cmd = CMD_ADC_READ;
   // HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 50);
   // HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, parca, 3, 50);
     HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, parca, 3, 50);
  /*  if (status != HAL_OK) {
            // Eğer okuma başarısızsa, D1/D2'nin 0 kalmaması için bir hata değeri dönelim
            return 0xFFFFFFFF;
        }*/
    return (uint32_t)((parca[0]<<16) | (parca[1]<<8) | parca[2]);
}


void MS5611_Kurulum_Yap (void) {
    // 1. Sensöre Reset At
    uint8_t reset_cmd = CMD_RESET;
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &reset_cmd, 1, 100);
    HAL_Delay(10);

    // 2. PROM'dan Fabrika Katsayılarını Oku (C1 - C6)
        uint8_t prom_dilimler[2];
        for(int i = 0 ; i < 8 ; i++){
            uint8_t cmd = CMD_PROM_READ + (i * 2);
            HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 100);
            HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, prom_dilimler, 2, 100);
            C[i] = (prom_dilimler[0] << 8) | prom_dilimler[1];
        }
}

//STATE MACHINE MANTIĞI
void MS5611_Basinc_Guncelle(void) {

	uint32_t su_an = HAL_GetTick();
	    uint32_t gecen_sure = su_an - ms5611_tetik_zamani;

	    // 0. DURUM: D1 (Basınç) ölçümünü başlat
	    if (ms5611_durum == 0) {
	        MS5611_Tetikle(CMD_CONVERT_D1_OSR4096);
	        ms5611_tetik_zamani = su_an;
	        ms5611_durum = 1;
	    }
	    // 1. DURUM: 20ms bekle, D1'i oku, D2 (Sıcaklık) ölçümünü başlat
	    else if (ms5611_durum == 1 && gecen_sure >= 20) {
	        D1 = MS5611_Oku_Hazir();
	        MS5611_Tetikle(CMD_CONVERT_D2_OSR4096);
	        ms5611_tetik_zamani = su_an;
	        ms5611_durum = 2;
	    }
	    // 2. DURUM: 20ms bekle, D2'yi oku, Hesapla ve BAŞA DÖN
	    else if (ms5611_durum == 2 && gecen_sure >= 20) {
	        D2 = MS5611_Oku_Hazir();

	        // Matematiksel Hesaplamalar
	        int32_t dT = D2 - ((int32_t)C[5] * 256);
	        int64_t OFF = ((int64_t)C[2] * 65536) + (((int64_t)C[4] * dT) / 128);
	        int64_t SENS = ((int64_t)C[1] * 32768) + (((int64_t)C[3] * dT) / 256);
	        int32_t P = ((D1 * SENS / 2097152) - OFF) / 32768;

	        paket.basinc_mbar = (float)P / 100.0f;

	        // İŞTE SİHİRLİ SATIR: Çarkı yeniden başlat!
	        ms5611_durum = 0;
	    }
	}
