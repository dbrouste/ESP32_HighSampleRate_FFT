//FFT.h found here https://github.com/yash-sanghvi/ESP32/tree/master/FFT_on_ESP32_Arduino

#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <soc/syscon_reg.h>

#include "FFT.h"

#include "soc/syscon_struct.h"


#include <math.h>


#define ADC_CHANNEL   ADC1_CHANNEL_6 // GPIO34
#define NUM_SAMPLES   1024                       // number of samples
#define SAMPLERATE 500000


size_t bytes_read;  

float fft_input[NUM_SAMPLES];
float fft_output[NUM_SAMPLES];
fft_config_t *real_fft_plan = fft_init(NUM_SAMPLES, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
float max_magnitude = 0;
float fundamental_freq = 0;
char print_buf[300];
float kfreq = NUM_SAMPLES*1.0/(SAMPLERATE*1.0);

void configure_i2s() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_ADC_BUILT_IN),  // I2S receive mode with ADC
    .sample_rate = SAMPLERATE,                                                        // sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                                 // 16 bit I2S
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,                                   // only the right channel
    // .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),                                  // I2S format
    .intr_alloc_flags = 0,                                                        // none
    .dma_buf_count = 2,                                                           // number of DMA buffers
    .dma_buf_len = NUM_SAMPLES,                                                   // number of samples
    .use_apll = 0,                                                                // no Audio PLL
  };
  // ADC channel 0 with 11db (divide by input 3.6)
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_11db);
  // 12 bit ADC
  adc1_config_width(ADC_WIDTH_12Bit);
  // install I2S 0 driver, using event queue
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

    // adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_1);
    // adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_3);
    // adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_6);
    // adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_7);

  // ADC should use ADC_CHANNEL
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  // The raw ADC data is written in DMA in inverted form. This add aninversion:
  // SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
  // SYSCON.saradc_ctrl2.sar1_inv = 1;	//SAR ADC samples are inverted by default
  // SYSCON.saradc_ctrl.sar1_patt_len = 0; //Use only the first entry of the pattern table

  // enable I2S ADC
  delay(5000);
  i2s_adc_enable(I2S_NUM_0);
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
  //https://github.com/espressif/esp-idf/issues/1911
    // SYSCON.saradc_ctrl.sar1_patt_len = 1; //2 Items in pattern table
    // //SYSCON.saradc_sar1_patt_tab[0] = 0x7f3f6f1f; //Array[0] first 4 items, Array[1] next 4 items...
    //                                              ///ch_sel[3:0] bit_width[1:0] atten[1:0]
    // SYSCON.saradc_ctrl2.sar1_inv = 1; //Inversion
    // SYSCON.saradc_ctrl2.meas_num_limit = 0;
  //Serial.println("I2S started");
  delay(1000); //required for stability of ADC   
}



void setup() {
  // Serial.begin(115200);
  Serial.begin(250000);
  configure_i2s();
}

static const inline void ADC_Sampling(){

  uint16_t i2s_read_buff[NUM_SAMPLES];
  // i2s_zero_dma_buffer(I2S_NUM_0);

  i2s_read(I2S_NUM_0, (int *)i2s_read_buff, NUM_SAMPLES * sizeof(uint16_t), &bytes_read, portMAX_DELAY); // check param

  i2s_adc_disable(I2S_NUM_0);

  int x = 0;
  for (int k = 0; k < NUM_SAMPLES; k++)
  {
    if (x == 0)
    {
      real_fft_plan->input[k] = ((float)(i2s_read_buff[k + 1] & 0xFFF) / 4096); // convert to 0.0 to 1.0
      x++;
    }
    else
    {
      real_fft_plan->input[k] = ((float)(i2s_read_buff[k - 1] & 0xFFF) / 4096); // convert to 0.0 to 1.0
      x--;
    }
    Serial.println((real_fft_plan->input[k])*1000);
  }

  fft_execute(real_fft_plan);
  max_magnitude = 0;
  fundamental_freq = 0;

  for (int k = 1; k < real_fft_plan->size / 2; k++)
  {
    /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
    float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2)) / 1;
    float freq = k * 1.0 / (kfreq);
    //  sprintf(print_buf,"%f Hz : %f", freq, mag);
    //  Serial.println(print_buf);
    if (mag > max_magnitude)
    {
      max_magnitude = mag;
      fundamental_freq = freq;
    }
  }

  // sprintf(print_buf, "Fundamental Freq : %f Hz\t Mag: %f g\n", fundamental_freq, (max_magnitude)*2 / NUM_SAMPLES);
  // Serial.println(print_buf);
  // fft_destroy(real_fft_plan);
  //  int x = 0;

  // for (int i=0;i<NUM_SAMPLES;i++) {
  //   if (x==0)
  //   {Serial.println(i2s_read_buff[i+1]& 0xFFF);
  //   x++;
  //   }
  //   else
  //   {Serial.println(i2s_read_buff[i-1]& 0xFFF);
  //   x--;
  //   }
  //   // Serial.println(i2s_read_buff[i]& 0xFFF);
  //   // Serial.print(" hexa : ");
  //   // Serial.println(i2s_read_buff[i] & 0x1FFF,BIN);
  // }
  delay(1000);

  i2s_adc_enable(I2S_NUM_0);
}

void loop() {
  ADC_Sampling();
}
