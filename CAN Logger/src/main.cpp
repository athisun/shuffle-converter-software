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

  CANMessage msg;

  pc.printf("running loop\n");

  while (1)
  {
    wait_ms(1);

    if (can1.read(msg))
    {
      led1 = !led1;

      // csv format
      // time,dip,id,type,format,length, data1,data2,data3,data4,data5,data6,data7,data8
      memset(&buffer1[0], 0, sizeof(buffer1));
      for (int i = 0; i < msg.len; i++)
      {
        sprintf(buffer1, "%s,%d", buffer1, msg.data[i]);
      }
      sprintf(buffer2, "%lld,%d,%d,%d,%d,%d %s\n", time(NULL), (msg.id >> 8) & 0xff, msg.id & 0xff, msg.type, msg.format, msg.len, buffer1);
    }

    if (pc.readable())
    {
      int n = pc.scanf("%s", &buffer1);
      if (n <= 0)
      {
        continue;
      }

      char *pch;
      printf("Splitting string \"%s\" into tokens:\n", buffer1);
      pch = strtok(buffer1, " ,.-");
      while (pch != NULL)
      {
        printf("%s\n", pch);
        pch = strtok(NULL, " ,.-");
      }
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