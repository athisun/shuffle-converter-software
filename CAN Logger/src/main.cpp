#include "mbed.h"

CAN can1(PA_11, PA_12);  // initialise CAN1
DigitalOut led1(PB_3);   // setup onboard LED
Serial pc(USBTX, USBRX); // initialise uart to pc

char buffer1[64] = {};
char buffer2[256] = {};

void sos();

int main()
{
  wait_ms(1000);
  pc.printf("Hello world!\n");

  if (!can1.frequency(500000))
  {
    pc.printf("error setting can frequency\n");
    sos();
  }
  if (!can1.mode(CAN::Normal))
  {
    pc.printf("error setting can mode\n");
    sos();
  }
  if (!can1.filter(0, 0))
  {
    pc.printf("error setting can filter\n");
    sos();
  }

  CANMessage msg;

  pc.printf("running loop\n");

  uint32_t last_msg = 0;

  while (1)
  {
    wait_ms(1);

    uint32_t millis = (us_ticker_read() / 1000);
    if (millis - last_msg > 1000)
    {
      last_msg = millis;
      can1.write(CANMessage(1));
      continue;
      // pc.printf("Sending bogus can msg\n");
    }

    if (can1.read(msg))
    {
      led1 = !led1;

      // csv format
      // time,id,type,format,length, data1,data2,data3,data4,data5,data6,data7,data8
      memset(&buffer1[0], 0, sizeof(buffer1));
      for (int i = 0; i < msg.len; i++)
      {
        sprintf(buffer1, "%s,%d", buffer1, msg.data[i]);
      }
      sprintf(buffer2, "%lld,%d,%d,%d,%d %s\n", time(NULL), msg.id, msg.type, msg.format, msg.len, buffer1);

      pc.printf("  CAN messsage: %s\n", buffer2);
    }

    if (pc.readable())
    {
      int n = pc.scanf("%i,%i,%i,%i,%i,%i,%i,%i,%i", &msg.id, &msg.data[0], &msg.data[1], &msg.data[2], &msg.data[3], &msg.data[4], &msg.data[5], &msg.data[6], &msg.data[7]);
      if (n <= 0)
      {
        continue;
      }
      msg.len = n - 1;
      msg.type = CANType::CANData;
      msg.format = CANFormat::CANStandard;
      memset(&buffer1[0], 0, sizeof(buffer1));
      for (int i = 0; i < msg.len; i++)
      {
        sprintf(buffer1, "%s,%d", buffer1, msg.data[i]);
      }
      pc.printf("Sending can msg:%x, len:%d, data:%s\n", msg.id, n - 1, buffer1);

      can1.write(msg);
    }
  }
}

void sos()
{
  led1.write(0);
  while (1)
  {
    for (int i = 0; i < 3; i++)
    {
      led1 = !led1;
      wait_ms(100);
      led1 = !led1;
      wait_ms(100);
    }
    for (int i = 0; i < 3; i++)
    {
      led1 = !led1;
      wait_ms(300);
      led1 = !led1;
      wait_ms(300);
    }
    for (int i = 0; i < 3; i++)
    {
      led1 = !led1;
      wait_ms(100);
      led1 = !led1;
      wait_ms(100);
    }
    wait_ms(300);
  }
}