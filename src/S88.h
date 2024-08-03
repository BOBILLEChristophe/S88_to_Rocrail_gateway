/*
  S88.h
*/

#ifndef S88_H
#define S88_H


#define S88_CLOCK_PIN GPIO_NUM_12  // Pin de CLOCK (CLK)
#define S88_LOAD_PIN GPIO_NUM_13   // Pin de LOAD (STROBE)
#define S88_DATA_PIN GPIO_NUM_14   // Pin de DATA (DATA_OUT)

#define S88_MODULE_COUNT 2
#define S88_INPUTS_PER_MODULE 16

struct Message {
  uint16_t module;
  uint16_t sensor;
  uint8_t value;
};

class S88 {
private:
  gpio_num_t mClockPin;
  gpio_num_t mPSPin;
  gpio_num_t mDataPin;
  uint8_t mNbModule;
  uint8_t mNbInputPerModule;
  uint16_t mNbSensor = S88_MODULE_COUNT * S88_INPUTS_PER_MODULE;
  bool s88data[S88_MODULE_COUNT * S88_INPUTS_PER_MODULE];
  Message message;
  QueueHandle_t s88Queue;

public:
  S88(QueueHandle_t);
  void setup();
  void readS88();
  void delay_us(uint32_t);
};



#endif