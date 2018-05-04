#include "mcc_usb1608g.h"

#include <unistd.h>
#include <cstdio>

int main(int argc, char **argv)
{
  auto daq = MCC_USB1608G::getInstance();

  daq->setFrequency(1000);
  daq->setGain(BP_10V);
  daq->setMode(SINGLE_ENDED);
  daq->setNChannel(8);
  daq->setNRead(50);
  daq->setTimeout(500);

  if (daq->config() != MCC1608G_SUCCESS)
    return 1;
  if (daq->start() != MCC1608G_SUCCESS)
    return 1;
  sleep(1);

  auto data = new double[50 * 16];
  uint16_t *raw_data = new uint16_t[16 * 512];
  for (auto i = 0; i < 5; i++)
    for (auto j = 0; j < 20; j++) {
      daq->start();
      printf("%d-%d: %d\n", i, j, daq->read(data));
    }

  daq->stop();

  return 0;
}
