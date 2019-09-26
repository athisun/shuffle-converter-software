#include "mbed.h"
#include "SDFileSystem.h"

CAN can1(PA_11, PA_12);
DigitalOut led1(PB_3);

Serial pc(SERIAL_TX, SERIAL_RX);

// csv format
// time,id,type,format,length,data1,data2,data3,data4,data5,data6,data7,data8

int main()
{
  pc.printf("main enter\n");
  can1.frequency(250000);
  can1.mode(CAN::Normal);

  CANMessage msg;
  while (1)
  {
    pc.printf("loop\n");
    if (can1.read(msg))
    {
      pc.printf("New message\n");
      led1 = !led1;
      pc.printf("ID: %d \nmsg: ", msg.id);
      
      for (int i = 0; i < msg.len; i++)
      {
        pc.printf("%d", msg.data[i]);
      }

      pc.printf("\nFormat: %d", msg.format);
      pc.printf("\nType: %d\n", msg.type);
    }
    else
    {
      pc.printf("No message found\n");
    }


    wait_ms(50);
  }
}