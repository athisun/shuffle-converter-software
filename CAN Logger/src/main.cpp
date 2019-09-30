#include "mbed.h"

#include "SDBlockDevice.h"
#include "FATFileSystem.h"
#include "DS3231.h"
#include "mbed_mktime.h"

SDBlockDevice sd(PA_7, PA_6, PA_5, PB_0); // mosi, miso, sclk, cs
FATFileSystem fs("fs");

CAN can1(PA_11, PA_12);          // initialise CAN1
DigitalOut led1(PB_3);           // setup onboard LED
Serial pc(SERIAL_TX, SERIAL_RX); // initialise uart to pc

int hour;
int minute;
int second;

int dayOfWeek; // Sun = 1, Mon = 2, etc.
int date;
int month;
int year;
// Initialise DS3231 RTC module
DS3231 rtc(D4, D5);
bool EN32kHz()
{
  int reg = rtc.readRegister(DS3231_Control_Status);
  return (reg & DS3231_bit_EN32kHz);
}

// Set RTC date and time
void rtc_setDateTime()
{
  rtc.setDate(2, 30, 9, 2019);
  rtc.setTime(13, 28, 10);
}

// Buffers for CSV
char buffer1[64] = {};
char buffer2[256] = {};

void sos();

void setMcuTimeTest()
{
  rtc.readDateTime(&dayOfWeek, &date, &month, &year, &hour, &minute, &second);

  tm Time = {
      .tm_sec = second,
      .tm_min = minute,
      .tm_hour = hour,
      .tm_mday = date,
      .tm_mon = month - 1,
      .tm_year = year - 1900,
      .tm_wday = dayOfWeek - 1};
  time_t now = mktime(&Time);
  if (now == -1)
  {
    sos();
  }
  else
  {
    set_time(now);
  }
  time_t rawtime = time(NULL);
  pc.printf("%lld\n", rawtime);

}

int main()
{
  wait_ms(1000);
  pc.printf("Hello world!\n");

  // rtc_setDateTime(); // Only need to run if not set
  setMcuTimeTest();
  // sos();
  while (1)
  {
    rtc.readDateTime(&dayOfWeek, &date, &month, &year, &hour, &minute, &second);
    pc.printf("date time : %i / %02i-%02i-%02i %02i:%02i:%02i", dayOfWeek, date, month, year, hour, minute, second);
    pc.printf("\r\n");

    wait_ms(5000);
  }

  if (!can1.frequency(250000))
  {
    pc.printf("error setting can frequency\n");
    sos();
  }
  if (!can1.mode(CAN::Normal))
  {
    pc.printf("error setting can mode\n");
    sos();
  }

  int err;
  if ((err = fs.mount(&sd)) != 0)
  {
    pc.printf("error mounting sd card: %d", err);
    sos();
  }

  int i = 0;
  char buffer1[32] = {};
  sprintf(buffer1, "/fs/log_%d.csv", i);
  struct stat st;
  while (stat(buffer1, &st) == 0)
  {
    i++;
    sprintf(buffer1, "/fs/log_%d.csv", i);
  }

  pc.printf("opening log file: %s", buffer1);
  FILE *fp = fopen(buffer1, "w");
  if (fp == NULL)
  {
    pc.printf("failed to open log file");
    sos();
  }

  CANMessage msg;

  pc.printf("running loop\n");

  while (1)
  {
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

      pc.printf("CAN MSG: %s", buffer2);

      // TODO: check if write fails and maybe restart micro?
      fprintf(fp, buffer2);
      fflush(fp);
      sd.sync();
    }

    wait_ms(1);
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