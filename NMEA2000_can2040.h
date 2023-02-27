/*
NMEA2000_can2040.h
*/

#ifndef _NMEA2000_CAN2040_H_
#define _NMEA2000_CAN2040_H_

extern "C"
{
#include "can2040.h"
}
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
//#include "driver/gpio.h"
#include "NMEA2000.h"
#include "N2kMsg.h"

#ifndef CAN2040_TX_PIN
#define CAN2040_TX_PIN 3
#endif
#ifndef CAN2040_RX_PIN
#define CAN2040_RX_PIN 4
#endif
#ifndef CAN2040_PIO
#define CAN2040_PIO 0
#endif

class tNMEA2000_can2040 : public tNMEA2000
{
private:
  struct can2040 cbus;
  bool IsOpen;
  static bool CanInUse;

protected:
  uint32_t speed;
  uint32_t TxPin;
  uint32_t RxPin;
  uint32_t Pio;

public:
   QueueHandle_t RxQueue;
  //  QueueHandle_t TxQueue;

protected:
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent = true);
  bool CANOpen();
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
  virtual void InitCANFrameBuffers();

public:
  /// @brief NMEA2000 constructor
  /// @param _Pio PIO to use, either 0 or 1
  /// @param _TxPin TX pin, default defined CAN2040_TX_PIN 3
  /// @param _RxPin RX pin, default defined CAN2040_RX_PIN 4
  tNMEA2000_can2040(uint32_t _Pio = CAN2040_PIO, uint32_t _TxPin = CAN2040_TX_PIN, uint32_t _RxPin = CAN2040_RX_PIN);

  void InterruptHandler();
};

void PIOx_IRQHandler(void);

static void
can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);
#endif
