#include "mcc_usb1608g.h"

#include "libusb/pmd.h"
#include "libusb/usb-1608G.h"

#include <cmath>
#include <unistd.h>
#include <cstdio>

MCC_USB1608G *MCC_USB1608G::instance_ = nullptr;

MCC_USB1608G::MCC_USB1608G()
        : is_configured_(false), frequency_(1), gain_(BP_10V), mode_(DIFFERENTIAL),
          nchan_(16), timeout_(1000), nread_(50), sdataIn_(nullptr)
{}

MCC_USB1608G::MCC_USB1608G(MCC_USB1608G &)
{}

MCC_USB1608G::~MCC_USB1608G()
{
  this->stop();
  delete[] sdataIn_;
}

MCC_USB1608G *MCC_USB1608G::getInstance()
{
  if (MCC_USB1608G::instance_ == nullptr)
    MCC_USB1608G::instance_ = new MCC_USB1608G();
  return MCC_USB1608G::instance_;
}

int MCC_USB1608G::config()
{
  if (is_configured_)
    this->stop();
  udev_ = nullptr;
  is_configured_ = false;

  // initialize libusb
  auto ret = libusb_init(nullptr);
  if (ret < 0)
    return MCC1608G_NOTFOUND;

  // find USB1608G
  if ((udev_ = usb_device_find_USB_MCC(USB1608G_PID, nullptr))) // version 1
    usbInit_1608G(udev_, 1);
  else if ((udev_ = usb_device_find_USB_MCC(USB1608G_V2_PID, nullptr))) // version 2
    usbInit_1608G(udev_, 2);
  else
    return MCC1608G_NOTFOUND;

  printf("USB1608G found @0x%lX\n", (long) udev_);

  // select mode
  wMaxPacketSize_ = usb_get_max_packet_size(udev_, 0);

  if (is_continuous_mode_) {
    if ((nchan_ * nread_ * 2 < wMaxPacketSize_) ||
        (nchan_ * nread_ * 2 % wMaxPacketSize_ != 0))
      return MCC1608G_CONFIGFAILED;
  }
  else {
    if (nchan_ * nread_ * 2 > wMaxPacketSize_)
      return MCC1608G_CONFIGFAILED;
  }

  usbBuildGainTable_USB1608G(udev_, table_AIN_);

  // configure hardware
  // write channel settings
  ScanList list[16];
  for (uint8_t i = 0; i < nchan_; i++) {
    list[i].range = gain_;
    list[i].channel = i;
    list[i].mode = mode_;
  }
  list[nchan_ - 1].mode |= LAST_CHANNEL;
  usbAInScanStop_USB1608G(udev_);
  usbAInConfig_USB1608G(udev_, list);
  sleep(1);

  delete[] sdataIn_;
  sdataIn_ = new uint16_t[nchan_ * nread_];

  is_configured_ = true;
  return MCC1608G_SUCCESS;
}

void MCC_USB1608G::setContinuousScanMode(bool isContinuouslyScan)
{
  is_continuous_mode_ = isContinuouslyScan;
}

int MCC_USB1608G::start()
{
  if (!is_configured_)
    return MCC1608G_NOTALLOWED;

  uint8_t nscans = (is_continuous_mode_) ? 0 : nread_;
  usbAInScanStart_USB1608G(udev_, nscans, 0, frequency_, 1);
  return MCC1608G_SUCCESS;
}

int MCC_USB1608G::stop()
{
  if (!is_configured_)
    return MCC1608G_NOTALLOWED;
  usbAInScanStop_USB1608G(udev_);
  usbAInScanClearFIFO_USB1608G(udev_);
  return MCC1608G_SUCCESS;
}

int MCC_USB1608G::read(double *data)
{
  auto len = this->readRawData(sdataIn_);
  for (auto i = 0; i < nread_; i++)
    for (auto j = 0; j < nchan_; j++) {
      auto k = i * nchan_ + j;
      uint16_t temp = rint(sdataIn_[k] * table_AIN_[gain_][0] + table_AIN_[gain_][1]);
      data[k] = volts_USB1608G(gain_, temp);
    }
  return len;
}

int MCC_USB1608G::readRawData(uint16_t *sdataIn)
{
  if (!is_configured_)
    return MCC1608G_CONFIGFAILED;
  if (!is_continuous_mode_)
    this->start();
  return usbAInScanRead_USB1608G(udev_, nread_, nchan_, sdataIn, timeout_, CONTINUOUS) >> 1;
}

void MCC_USB1608G::setFrequency(double frequency)
{ frequency_ = frequency; }

void MCC_USB1608G::setGain(uint8_t gain)
{
  switch (gain) {
    case BP_10V:
    case BP_5V:
    case BP_2V:
    case BP_1V:
      gain_ = gain;
      break;
    default:
      gain_ = BP_10V;
  }
}

void MCC_USB1608G::setMode(uint8_t mode)
{
  if ((mode == SINGLE_ENDED) || (mode == DIFFERENTIAL))
    mode_ = mode;
  else mode_ = DIFFERENTIAL;
}

void MCC_USB1608G::setNChannel(int nchan)
{ nchan_ = nchan; }

void MCC_USB1608G::setNRead(int nread)
{ nread_ = nread; }

void MCC_USB1608G::setTimeout(unsigned int timeout)
{ timeout_ = timeout; }

void MCC_USB1608G::blink(uint8_t count)
{
  if (udev_ != nullptr)
    usbBlink_USB1608G(udev_, count);
}
