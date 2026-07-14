/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ms5611.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "LoRa_E22.h"

#include "flight_control.h"
#define RAD_TO_DEG 57.29577951
//roketin telemetri paketi bu şekilde olacak
#pragma pack(push, 1)

#pragma pack(pop)

// ----> ŞU SATIRI KESİNLİKLE EKLEMELİSİN <----


// --- BNO055 KALMAN YAPISI ---
typedef struct {
    float Q_angle, Q_bias, R_measure, angle, bias;
    float P[2][2];
} Kalman_t;
Kalman_t kalman_pitch;
uint32_t bno_son_zaman = 0;
volatile uint8_t timer_10ms_bayrak = 0;
// --- İRTİFA İÇİN 2 DURUMLU (STATE) KALMAN FİLTRESİ ---
typedef struct {
    float h;        // İrtifa (Metre)
    float v;        // Dikey Hız (Metre/Saniye) - Sadece filtre kararlılığı için hesaplanır
    float P[2][2];  // Hata Kovaryans Matrisi
    float Q_accel;  // İvmeölçer Süreç Gürültüsü
    float R_baro;   // Barometre Ölçüm Gürültüsü
} Kalman_Irtifa_t;

 Kalman_Irtifa_t IrtifaFiltresi;

// Yerçekimi sabiti
#define GRAVITY 9.80665f



// --- BNO055 DEĞİŞKENLERİ ---
#define BNO055_I2C_ADDR (0x28 << 1)
#define BNO055_OPR_MODE_REG 0x3D
#define OPR_MODE_CONFIG 0x00
#define OPR_MODE_ACCGYRO 0x05 // Sadece İvme ve Jiro, füzyon kapalı

// --- MS5611 DEĞİŞKENLERİ ---
//diğer dosyada

extern TIM_HandleTypeDef htim3; // Timer3 kullanacağımızı varsayıyoruz

Lora_t lora;
Lora_Config_t loraConfig;

UartHandler_t uart3;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

volatile uint8_t bulunan_adresler[10] = {0}; // Bulunan sensörlerin adreslerini tutacak
volatile uint8_t cihaz_sayisi = 0;           // Kaç tane sensör bulduğunu sayacak

volatile uint32_t uart1_kesme_sayaci = 0;
volatile float anlik_deniz_seviyesi_irtifa = 0.0f;
extern volatile uint8_t ms5611_durum;
extern volatile uint32_t D1;
extern volatile uint32_t D2;

GPS_t                  rocketGPS;   // Tanım burada
volatile RoketTelemetri paket;      // Tanım burada

uint8_t rx_data;             // UART'tan anlık gelen tek karakter
char rx_buffer[100];         // GPS satırını biriktirdiğimiz dizi
uint8_t rx_index = 0;        // Dizinin neresinde olduğumuzu tutan sayaç
volatile char gps_sentence[100];      // İşlenecek tam satır
volatile uint8_t sentence_ready = 0;  // Yeni tam satır geldiğini belirten bayrak
        // GPS verilerini tutan yapı

volatile float acc_x = 0.0f;
volatile float acc_y = 0.0f;
volatile float acc_z = 0.0f;
volatile float gyro_x = 0.0f;
volatile float gyro_y = 0.0f;
volatile float gyro_z = 0.0f;

volatile float yer_irtifasi = 0.0f;
volatile uint8_t yer_irtifasi_alindi = 0;
volatile float toplam = 0.0f;
volatile uint8_t ortalama_sayaci = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//*************BURASI MAİN İÇERİSİNDE ÇAĞIRILACAK***************

        //irtifa için imu ve basıncı tek kalmana sokup dikey hız ile daha dengeli veri elde ediyoruz.

        void Kalman_Irtifa_Init(Kalman_Irtifa_t *kf, float baslangic_irtifasi) {
            kf->h = baslangic_irtifasi;
            kf->v = 0.0f;

            // Başlangıç güvenilirliği (İlk başta belirsizlik yüksek)
            kf->P[0][0] = 10.0f; kf->P[0][1] = 0.0f;
            kf->P[1][0] = 0.0f;  kf->P[1][1] = 10.0f;

            // Gürültü Parametreleri (Test uçuşlarına göre ayarlanabilir)
            kf->Q_accel = 0.5f;   // İvme gürültüsü varyansı
            kf->R_baro = 2.0f;    // Barometre gürültüsü varyansı (Gürültülü sensör için yüksek)
        }

        // 1. ADIM: Her 10 ms'de bir ivmeölçer ile çalışır
        void Kalman_Irtifa_Predict(Kalman_Irtifa_t *kf, float dikey_ivme, float dt) {
            // Durum tahmini: h = h + v*dt + 0.5*a*dt^2
            kf->h += kf->v * dt + 0.5f * dikey_ivme * dt * dt;
            kf->v += dikey_ivme * dt;

            // Kovaryans güncellemesi: P = F*P*F^T + Q
            kf->P[0][0] += dt * (kf->P[1][0] + kf->P[0][1]) + (dt * dt * kf->P[1][1]) + (kf->Q_accel * dt * dt * dt * dt / 4.0f);
            kf->P[0][1] += dt * kf->P[1][1] + (kf->Q_accel * dt * dt * dt / 2.0f);
            kf->P[1][0] += dt * kf->P[1][1] + (kf->Q_accel * dt * dt * dt / 2.0f);
            kf->P[1][1] += kf->Q_accel * dt * dt;
        }




        //*****************************************************---------------------**********************-----------------*********a bgöefvhdx bnJLWGFVHB*******************
        // 2. ADIM: Barometre verisi (20 ms'de bir) hazır olduğunda çalışır
       /* genel callback 10 ms de bir çalışır, ımu veri verebilir 10 ms de bir o yüzden predict
		çalışır ama baro ms511 state machine ile ımuyu bekletmeden if bloğunun içine girmeden,
		hazır olduğunda if bloğunun içine girirek 20 ms de bir okunur yani o halde update ise 20 ms de bir çalışır
		yalnız update in ihtiyaç duyduğu kf->h ve ve p ler (p en başta sadece ımu ile hazırlanır sonra update alışmasından sonra
				basınçdan gelen veri ile de dengelenmiş olur sonra predict ile ımudan alınann veriye göre üstüne katılır tahminin) her 10 ms de bir güncel verilerin fotoğrafı ile oluşur yeaahhh.
				*/



        void Kalman_Irtifa_Update(Kalman_Irtifa_t *kf, float olculen_irtifa) {
            // y = z - H*X (Fark)
            float y = olculen_irtifa - kf->h;

            // S = H*P*H^T + R
            float S = kf->P[0][0] + kf->R_baro;

            // Kalman Kazancı (K = P*H^T / S)
            float K0 = kf->P[0][0] / S;
            float K1 = kf->P[1][0] / S;

            // Durumu Güncelle (X = X + K*y)
            kf->h += K0 * y;
            kf->v += K1 * y;

            // Kovaryansı Güncelle (P = (I - K*H)*P)
            float P00_temp = kf->P[0][0];
            float P01_temp = kf->P[0][1];
            kf->P[0][0] -= K0 * P00_temp;
            kf->P[0][1] -= K0 * P01_temp;
            kf->P[1][0] -= K1 * P00_temp;
            kf->P[1][1] -= K1 * P01_temp;
        }







        //BNO FONKSİYONLARI VE KALMAN FİLTRESİ

        void i2c_register_yaz(uint8_t reg, uint8_t veri) {
            HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &veri, 1, 100);
        }
/*
        int16_t i2c_register_oku16(uint8_t reg) {
            uint8_t data[2]= {0, 0};
            HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
            return (int16_t)((data[1] << 8) | data[0]);
        }
       */



        int16_t i2c_register_oku16(uint8_t reg) {
            uint8_t data[2]= {0, 0};
            HAL_StatusTypeDef durum = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100);

            // Eğer iletişim hatası veya kilitlenme varsa
            if (durum != HAL_OK) {
                // Ana döngüdeki donanım sıfırlama (hayat öpücüğü) bloğunu zorla tetikle
                hi2c1.ErrorCode = HAL_I2C_ERROR_AF;
                return 0;
            }

            return (int16_t)((data[1] << 8) | data[0]);
        }


        //-----------------------------------------------------------------------------------------------------
        void I2C_Tarayici_Calistir(void) {
            char mesaj[60];
            uint8_t cihaz_sayisi = 0;

            sprintf(mesaj, "\r\n--- I2C TARAMASI BASLIYOR ---\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)mesaj, strlen(mesaj), 100);

            // 1'den 127'ye kadar tüm olası I2C adreslerini tara
            for (uint8_t i = 1; i < 128; i++) {
                // STM32 HAL kütüphanesi adresi 1 bit sola kaydırılmış (<< 1) olarak ister
                HAL_StatusTypeDef sonuc = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 10);

                if (sonuc == HAL_OK) {
                    sprintf(mesaj, "-> Cihaz Bulundu! Adres: 0x%02X\r\n", i);
                    HAL_UART_Transmit(&huart2, (uint8_t*)mesaj, strlen(mesaj), 100);
                    cihaz_sayisi++;
                }
            }

            if (cihaz_sayisi == 0) {
                sprintf(mesaj, "HATA: Hicbir I2C cihazi bulunamadi!\r\n");
            } else {
                sprintf(mesaj, "Tarama Bitti. Toplam Cihaz: %d\r\n", cihaz_sayisi);
            }
            HAL_UART_Transmit(&huart2, (uint8_t*)mesaj, strlen(mesaj), 100);
            sprintf(mesaj, "-----------------------------\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)mesaj, strlen(mesaj), 100);
        }
        //------------------------------------------------------------------------------------------------------

        void bno055_kurulum() {
            i2c_register_yaz(BNO055_OPR_MODE_REG, OPR_MODE_CONFIG);
            HAL_Delay(25);
            i2c_register_yaz(BNO055_OPR_MODE_REG, OPR_MODE_ACCGYRO);
            HAL_Delay(15);

            kalman_pitch.Q_angle = 0.001f; kalman_pitch.Q_bias = 0.003f; kalman_pitch.R_measure = 0.01f;
            bno_son_zaman = HAL_GetTick();
        }

        float kalman_hesapla_pitch(Kalman_t *klm, float yeni_olcum, float gyro_hizi, float dt) {
            klm->angle += dt * (gyro_hizi - klm->bias);
            klm->P[0][0] += dt * (dt * klm->P[1][1] - klm->P[0][1] - klm->P[1][0] + klm->Q_angle);
            klm->P[0][1] -= dt * klm->P[1][1]; klm->P[1][0] -= dt * klm->P[1][1];
            klm->P[1][1] += klm->Q_bias * dt;

            float y = yeni_olcum - klm->angle;

            float S = klm->P[0][0] + klm->R_measure;
            float K0 = klm->P[0][0] / S; float K1 = klm->P[1][0] / S;

            klm->angle += K0 * y; klm->bias += K1 * y;
            float P00_temp = klm->P[0][0]; float P01_temp = klm->P[0][1];
            klm->P[0][0] -= K0 * P00_temp; klm->P[0][1] -= K0 * P01_temp;
            klm->P[1][0] -= K1 * P00_temp; klm->P[1][1] -= K1 * P01_temp;

            return klm->angle;
        }

        void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
        {
            // Hangi timer'ın kesmeye girdiğini kontrol ediyoruz (Sizin projenizde TIM3)
            if(htim->Instance == TIM3)
            {
                // Sadece bayrağı kaldır. Başka HİÇBİR işlem yapma, okuma yapma, bekleme yapma!
                timer_10ms_bayrak = 1;
            }
        }

        //KESME FONKSİYONU ÇALIŞIP BAYRAK 1 YAPIP ÇIKIYOR HEMEN KESMEDE AKSAMA MEYDANA GELMEZ



        void I2C_Hatti_Kurtar(void)
                {
                    GPIO_InitTypeDef GPIO_InitStruct = {0};

                    // 1. STM32 I2C DONANIM BUG'I ÇÖZÜMÜ: Çevre biriminin şalterini indirip kaldır!
                    __HAL_RCC_I2C1_FORCE_RESET();
                    HAL_Delay(2);
                    __HAL_RCC_I2C1_RELEASE_RESET();

                    // 2. I2C çevre birimini tamamen kapat
                    HAL_I2C_DeInit(&hi2c1);

                    // 3. SCL ve SDA'yı manuel GPIO çıkışı yap (Open-Drain, Pull-Up ile)
                    __HAL_RCC_GPIOB_CLK_ENABLE();

                    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9; // PB8=SCL, PB9=SDA (F446RE Pinleri)
                    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
                    GPIO_InitStruct.Pull  = GPIO_PULLUP;
                    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

                    // SDA'yı HIGH bırak, SCL'i de HIGH'dan başlat
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // SDA HIGH
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // SCL HIGH
                    HAL_Delay(1);

                    // 4. SCL'e 9 adet manuel darbe ver (Sensörleri serbest bırak)
                    for (int i = 0; i < 9; i++)
                    {
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL LOW
                        HAL_Delay(1);
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL HIGH
                        HAL_Delay(1);

                        // SDA HIGH'a çıktı mı? (Sensör hattı bıraktı mı kontrol et)
                        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
                        {
                            break;
                        }
                    }

                    // 5. STOP koşulu üret
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
                    HAL_Delay(1);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
                    HAL_Delay(1);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
                    HAL_Delay(1);

                    // 6. Pinleri tekrar I2C moduna al ve çevrebrimini yeniden başlat
                    MX_I2C1_Init();

                    // 7. MS5611 STATE MACHINE'İ BAŞA SAR (Kilitlenmeyi önler)
                    ms5611_durum = 0;

                    // 8. Sensörleri yeniden kur
                    bno055_kurulum();
                    MS5611_Kurulum_Yap();

                    // 9. Kalman hafızasını temizle
                    yer_irtifasi_alindi = 0;
                    ortalama_sayaci     = 0;
                    toplam              = 0.0f;
                    Kalman_Irtifa_Init(&IrtifaFiltresi, 0.0f);

                    // Hata bayrağını tertemiz yap
                    hi2c1.ErrorCode = 0;
                }


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);

  // SESSİZ I2C TARAYICI (Live Expressions İçin)
    cihaz_sayisi = 0;
    for(uint8_t i = 1; i < 128; i++) {
        // Adresi 1 bit sola kaydırarak yolluyoruz
        HAL_StatusTypeDef sonuc = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 10);

        if(sonuc == HAL_OK) {
            if(cihaz_sayisi < 10) {
                bulunan_adresler[cihaz_sayisi] = i; // Cihaz bulursa adresi diziye kaydet
                cihaz_sayisi++;
            }
        }
    }


  // I2C Hattını Tara ve Sonuçları Gönder
    I2C_Tarayici_Calistir();







  bno055_kurulum();  // BNO055'i uçuş moduna al
  MS5611_Kurulum_Yap();

      paket.ucus_durumu = 0; // Roket rampada

      HAL_TIM_Base_Start_IT(&htim3); // ÇOK KRİTİK: 10ms sayacını (kesmeyi) başlat!HEYYYYYYYYYYYYYUNUTMA
      // ---> GPS İÇİN UART DİNLEMESİNİ BAŞLAT (huart1 varsayımıyla) <---
      //-------------------------------------**********************************************************GPS



      // GPS BAŞLAMADAN ÖNCE TAŞMAYI TEMİZLE Kİ KİLİTLENMESİN
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);

        RingBuffer_t rb;
        uart3.ringBuffer = &rb;
        uart3.uart = &huart3;
        Uart_Init(&uart3);

        Lora_Init_t loraInit;
        loraInit.auxPin = E22_AUX_Pin;
        loraInit.auxPort = E22_AUX_GPIO_Port;
        loraInit.m0Pin = E22_M0_Pin;
        loraInit.m0Port = E22_M0_GPIO_Port;
        loraInit.m1Pin = E22_M1_Pin;
        loraInit.m1Port = E22_M1_GPIO_Port;
        loraInit.baudRate = 9600;
        loraInit.bufferSize = 240;
        loraInit.prefix = 10;
        loraInit.suffix = 20;

        Lora_Init(&lora, &loraInit, &uart3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      uint32_t son_telemetri_zamani = 0;
         char mesaj[150];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */






  if (timer_10ms_bayrak == 1)
  	        {

	  if (hi2c1.ErrorCode != 0) {
	      I2C_Hatti_Kurtar();  // Tek satır, her şeyi halleder
	  }



  	            timer_10ms_bayrak = 0;  // AMA ZATEN BU KOD BİTENE KADAR 10 MSÇOKTAN GEÇER BİR MANTIĞI YOK DEĞİL ÇÜNKÜ ALTTAKİ MS5611 İÇİN BEKLEME YAPMIYORUZ SÜRE DOLDU MU DİYE KONTROL EDİYORUZ
  	           // O YÜZDEN 2 MS SÜRER VE GERİYE 8 MS BOŞLUK KALIR. Sabit kalması kalman filtresinin delta t si için mantıklı
  				// yani sabit 10 ms olması. Ayrıca ımu içinde 10 ms süre iyi. 2 ms de bir kodu sensörden veri almaya yollarsan olmaz.
  	            // tekli kalman örneğin sadece barometre bile olsa Q süreç gürültüsü zamandan etkilenir, pitch için ise gyroyu zamandal integralleyip üst üste topluyorduk zten yine sabit zaman ihtiyaç duyar

  	          if (sentence_ready == 1) {
  	        	sentence_ready = 0; // Bayrağı indir
  	          	  	            if (strncmp(gps_sentence, "$GPGGA", 6) == 0 || strncmp(gps_sentence, "$GNGGA", 6) == 0) {
  	          	  	                char *ptr = gps_sentence;
  	          	  	                int comma_count = 0;

  	          	  	              while (*ptr) {
  	          	  	                  if (*ptr == ',') {
  	          	  	                      comma_count++;

  	          	  	                      // Sadece 2. (Enlem) ve 4. (Boylam) virgülde Derece/Dakika hesabı yap!
  	          	  	                      if (comma_count == 2 || comma_count == 4) {
  	          	  	                          float raw = atof(ptr + 1);
  	          	  	                          int derece = (int)(raw / 100);
  	          	  	                          float dakika = raw - (derece * 100);

  	          	  	                          if (comma_count == 2) rocketGPS.latitude = derece + dakika / 60.0f;
  	          	  	                          if (comma_count == 4) rocketGPS.longitude = derece + dakika / 60.0f;
  	          	  	                      }

  	          	  	                      // Diğer verileri normal al
  	          	  	                      if (comma_count == 6) rocketGPS.fix_quality = atoi(ptr + 1);
  	          	  	                      if (comma_count == 9) rocketGPS.altitude = atof(ptr + 1);
  	          	  	                  }
  	          	  	                  ptr++;
  	          	  	                  if (comma_count > 9) break;
  	          	  	              }
  	          	  	            }

  	          	  	        }





  	                    uint32_t su_an = HAL_GetTick();
  	                    paket.zaman_damgasi = su_an;

  	                    // 1. BNO055 OKUMA VE KALMAN (Her 10ms'de bir)
  	                    // dt her zaman sabit 0.01 saniye (10ms) olduğu için hesaplama mükemmel çalışır
  	                    //float dt = 0.01f;


  	                    // 1. BNO055 OKUMA VE KALMAN (Her 10ms'de bir)
  	                    float dt = 0.01f;

  	                  acc_x  = i2c_register_oku16(0x08) / 100.0f;
  	                  acc_y  = i2c_register_oku16(0x0A) / 100.0f;
  	                  acc_z  = i2c_register_oku16(0x0C) / 100.0f;
  	                  gyro_x = i2c_register_oku16(0x14) / 16.0f;
  	                  gyro_y = i2c_register_oku16(0x16) / 16.0f;
  	                  gyro_z = i2c_register_oku16(0x18) / 16.0f;

  	                                  // Roket dik konumda 0, yataya geldiğinde 90 derece kuralı:

  	               // float raw_pitch = atan2(acc_x, sqrt((acc_y * acc_y) + (acc_z * acc_z)));

  	              // float current_pitch = raw_pitch * RAD_TO_DEG;


  	                float yatay_ivme = sqrtf(acc_y*acc_y + acc_z*acc_z);
  	                float raw_pitch = atan2f(yatay_ivme, acc_x);
  	                float current_pitch = raw_pitch * RAD_TO_DEG;


  	                  /*
  	                // 1. Yatay eksenlerin (X ve Y) vektörel büyüklüğünü alıyoruz (Sadece pozitif değer üretir)
  	                float yatay_ivme = sqrt((acc_x * acc_x) + (acc_y * acc_y));

  	                // 2. atan2(Y, X) kuralına göre:
  	                // - Y parametresine yatay bileşkeyi koyuyoruz.
  	                // - X parametresine boylamasına eksen olan Z'yi koyuyoruz.
  	                // Z ekseni dik konumda -9 olduğu için önüne (-) koyarak 0 dereceyi referans noktası yapıyoruz.
  	                float raw_pitch = atan2(yatay_ivme, -acc_z);
  	                float current_pitch = raw_pitch * RAD_TO_DEG;
*/
  	                // 3. Kalman filtresine gönderiyoruz
  	                paket.pitch_acisi = kalman_hesapla_pitch(&kalman_pitch, current_pitch, -gyro_y, dt);


  	                          //  paket.pitch_acisi = current_pitch;

  	                                  // --- İRTİFA İÇİN DİKEY İVME HESABI ---
  	                                          // Roketin X ekseni boylamasına ise, dik dururken acc_x 9.81 gösterir.
  	                                          // Açıya göre bunu dünya eksenindeki dikey ivmeye çevirip yerçekimini çıkarıyoruz.
  	                                          // pitch_acisi dik konumda 0 ise:
  	                             //   float pitch_rad = paket.pitch_acisi / RAD_TO_DEG;
  	                             //   float dikey_ivme = (acc_x * sin(pitch_rad) + acc_z * cos(pitch_rad)) - GRAVITY;
  	                               // YENİ VE KUSURSUZ KISIM:
  	                               float ivme_vektoru = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  	                               float dikey_ivme = ivme_vektoru - GRAVITY;
  	                                          // Kalman 1. Adım: Her 10 ms'de İvme ile Tahmin (Predict)
  	                                          Kalman_Irtifa_Predict(&IrtifaFiltresi, dikey_ivme, dt);





  	                                  // 2. MS5611 STATE MACHINE    BURASI DİĞER DOSYADA
  	                                  // Döngünün kendisi zaten tam 10ms sürdüğü için ekstra zaman kontrolüne gerek yok!
  	                                  //ms5611 süre kontrolüyle non blocking yapalım
  	                                        MS5611_Basinc_Guncelle();
  	                            //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  	                                      anlik_deniz_seviyesi_irtifa = 44330.0f * (1.0f - powf(paket.basinc_mbar / 1013.25f, 0.190295f));

  	                                      // Rampa İrtifasını Kaydetme (Referans Noktası)
  	                                      //****** ROKETİN RAMPADA BULUNDUĞU İLİN İRTİFASI BURAYA GİRİLECEK ŞİMDİLİK 0 KABUL EDİYORUZ***********

                                    // yer irtifası = anlik deniz seviyesi irtifa ama ilk 5 ölçümün ortalamasını al
  	                                    if (paket.basinc_mbar > 100.0f) {
  	                                      if (yer_irtifasi_alindi == 0) {
  	                                    	// İlk N ölçümün ortalamasını al


  	                                    	if (ortalama_sayaci < 20) {
  	                                    	    toplam += anlik_deniz_seviyesi_irtifa;
  	                                    	    ortalama_sayaci++;
  	                                    	} else if (yer_irtifasi_alindi == 0) {
  	                                    	    yer_irtifasi = toplam / 20.0f;
  	                                    	    yer_irtifasi_alindi = 1;
  	                                    	    Kalman_Irtifa_Init(&IrtifaFiltresi, 0.0f);
  	                                    	}


  	                                      }
  	                                    }
  	                                      float baro_irtifa_metre = anlik_deniz_seviyesi_irtifa - yer_irtifasi;
  	                                    if (yer_irtifasi_alindi == 1) {
  	                                                  // Kalman 2. Adım: Barometre verisi hazır olduğunda Güncelle (Update)
  	                                                  Kalman_Irtifa_Update(&IrtifaFiltresi, baro_irtifa_metre);
  	                                    }


  	                                  }
  	                                      // Roketin yerden yüksekliğini ayarlayalım ve güncelleyelim
  	                                     paket.irtifa_metre = IrtifaFiltresi.h;

  	                                 //  paket.irtifa_metre = 3700.0f;

  	                                      //PAKET UCUS DURUMU=0 hiçbirşey açılmadı (rampada veya tırmanışta) =1 sürükleme paraşütü açıldı 2 ana paraşüt açıldı ve bitti
  	                                   Ucus_Durum_Kontrol();



uint32_t su_an_telemetri = HAL_GetTick();

	        // Eğer son gönderimden bu yana 50 ms (veya istediğiniz süre) geçtiyse
	        if (su_an_telemetri - son_telemetri_zamani >= 200)  //paketi eğer  50 ms geçtiyse kargoya ver (uarta ver)
	        {
	            son_telemetri_zamani = su_an_telemetri;

	            sprintf(mesaj, "Zaman: %lu | Durum: %d | Pitch: %d | Basinc: %d | Enlem: %.4f | Boylam: %.4f\r\n",
	                              paket.zaman_damgasi,
	                              paket.ucus_durumu,
	                              (int)paket.pitch_acisi,
	                              (int)paket.basinc_mbar,
	                              rocketGPS.latitude,
	                              rocketGPS.longitude);



	            // DİKKAT: _IT (Interrupt) veya _DMA versiyonunu kullanıyoruz!
	            // İşlemci burada beklemez, sadece veriyi donanıma iletir ve anında alt satıra geçer.
	            paket.accX = acc_x;
	            paket.accY = acc_y;
	            paket.accZ = acc_z;
	            paket.gyroX = gyro_x;
	            paket.gyroY = gyro_y;
	            paket.gyroZ = gyro_z;
	            paket.gps = rocketGPS;
	            Lora_Write(&lora, 101, 18, &paket, sizeof(paket));
	            // 50 ms geçti yeni paket kargo geldi ama hala eskisini teslim ediyor ise bunun uart hızını ayarlaman lazım.
	            // eğer uart hızın 115.200 bit/saniye  ise  sanieyede yani 1000 ms de 11520 (1 karakter 10 bit) karakter gönderirsin ama senin paketin 120 char zaten.
	            //eğer 9600 bps yaparsan 960 karakteri 1000 ms de 120 karakteri de 110 ms de gönderir yani 50 yi aşar ve yetiştiremediği paket uçar 3. paketi ancak götürür, işi o zamana biter
// KESME OLMASAYDI (IT OLMASAYDI SADECE TRANSMİT OLSAYDI) verinin gitmesini beklemek uzun sürerdi ve bayrak o sırada kalkardı ve bizim 0 yapmamaızın önemi olmadan hep 1 olurdu.
	            //yani 10 ms yi geçer süre ve kalman çöker
	            //yer istasyonu 20 Hz yani bu paket saniyede 20 kez gidecek ve 1000 ms / 20 yani her 50 ms de bir bu paket gidecek
	            // kesme ile sen bu verileri gönder, bittiğinde haber veririsin yani uartun meşgulunu rahata çekersin
	            // HAL_Delay(50); // <-- BU KESİNLİKLE SİLİNDİ!
	        }
}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E22_M1_Pin|E22_M0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E22_AUX_Pin */
  GPIO_InitStruct.Pin = E22_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(E22_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E22_M1_Pin E22_M0_Pin */
  GPIO_InitStruct.Pin = E22_M1_Pin|E22_M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    	 uart1_kesme_sayaci++;  // <-- sadece bunu ekle, geri kalan aynı kalıyor
        // Sadece \n (Yeni Satır) karakterini satır sonu kabul et
        if (rx_data == '\n') {
            rx_buffer[rx_index] = '\0';
            if (rx_index > 0) {
                strncpy(gps_sentence, rx_buffer, rx_index + 1);
                sentence_ready = 1;
            }
            rx_index = 0;
        }
        // \r karakterini (Satır Başı) görmezden gel!
        else if (rx_data != '\r') {
            if (rx_index < 99) {
                rx_buffer[rx_index++] = rx_data;
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
    for(uint8_t i = 0; i < uartCount; i++) {
		if(huart == uartHandlers[i]->uart) {
			UartHandler_t* uartHandler = uartHandlers[i];
			RingBuffer_Enqueue(uartHandler->ringBuffer, &uartHandler->rxByte, 1);
			HAL_UART_Receive_IT(uartHandler->uart, &uartHandler->rxByte, 1);
			break;
		}
	}
}
// İşlemci çöp veri duyup paniklediğinde onu hayatta tutan kalp masajı!
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Hata bayraklarını zorla temizle
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        rx_index = 0;  // <-- BU EKSİKTİ, yarım satırı temizle
        // Kulakları tekrar aç ve dinlemeye devam et
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
