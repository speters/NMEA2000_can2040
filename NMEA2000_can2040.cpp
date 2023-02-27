/*
NMEA2000_can2040.cpp
*/

extern "C"
{
#include "can2040.h"
}
#include "NMEA2000_can2040.h"

bool tNMEA2000_can2040::CanInUse = false;
tNMEA2000_can2040 *pNMEA2000_can2040 = 0;

tNMEA2000_can2040::tNMEA2000_can2040(uint32_t _Pio, uint32_t _TxPin, uint32_t _RxPin) : tNMEA2000(), IsOpen(false),
                                                                                        speed(250000), TxPin(_TxPin), RxPin(_RxPin),
                                                                                        RxQueue(NULL), Pio(CAN2040_PIO)
//, TxQueue(NULL)
{
}

bool tNMEA2000_can2040::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool /*wait_sent*/)
{
  can2040_msg msg;
  msg.id = id;
  msg.dlc = len > 8 ? 8 : len;
  memcpy(msg.data, buf, msg.dlc);

  int ret = can2040_transmit(&pNMEA2000_can2040->cbus, &msg);
  if (ret < 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void tNMEA2000_can2040::InitCANFrameBuffers()
{
  if (MaxCANReceiveFrames < 10)
  {
    MaxCANReceiveFrames = 40;
  }

  RxQueue = xQueueCreate(MaxCANReceiveFrames, sizeof(can2040_msg));

  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

bool tNMEA2000_can2040::CANOpen()
{
  if (IsOpen)
  {
    return true;
  }
  if (CanInUse)
  {
    return false; // currently prevent accidental second instance. Maybe possible in future.
  }
  IRQn_Type pioIrq;
  if (Pio == 0)
  {
    pioIrq = PIO0_IRQ_0_IRQn;
  }
  else if (Pio == 1)
  {
    pioIrq = PIO1_IRQ_0_IRQn;
  }
  else
  {
    return false; // only accept PIO 0 and PIO 1
  }

  pNMEA2000_can2040 = this;
  IsOpen = true;

  // Setup canbus

  can2040_setup(&pNMEA2000_can2040->cbus, Pio);
  can2040_callback_config(&pNMEA2000_can2040->cbus, can2040_cb);

  // Enable irqs
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

  // Start canbus
  can2040_start(&pNMEA2000_can2040->cbus, F_CPU, 250000, RxPin, TxPin); // TODO: check if rp2040.f_cpu() is better
  CanInUse = IsOpen;
  return IsOpen;
}
bool tNMEA2000_can2040::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
  can2040_msg msg;

  // receive next CAN frame from queue
  if (xQueueReceive(RxQueue, &msg, 0) == pdTRUE)
  {
    id = msg.id;
    len = msg.dlc;
    memcpy(buf, msg.data, len);

    return true;
  }
  else
  {
    return false;
  }
}

// Main PIO irq handler
void PIOx_IRQHandler(void)
{
  pNMEA2000_can2040->InterruptHandler();
}

//*****************************************************************************
void tNMEA2000_can2040::InterruptHandler()
{
  can2040_pio_irq_handler(&pNMEA2000_can2040->cbus);
}

static void
can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
  if (notify & CAN2040_NOTIFY_TX)
  {
    // canbus_notify_tx();
    return;
  }

  if (notify & CAN2040_NOTIFY_RX)
  {
    xQueueSendToBackFromISR(pNMEA2000_can2040->RxQueue, msg, 0);
  }
}
