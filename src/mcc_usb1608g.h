/**
 * @brief MCC 1608G driver for ROS
 *
 * @file mcc_1608g.h
 * @author 张启宁
 * @date 2018-05-03
 */
#ifndef __MCC_1608G_H__
#define __MCC_1608G_H__

#include "libusb/pmd.h"
#include "libusb/usb-1608G.h"

#define MCC1608G_SUCCESS 0
#define MCC1608G_NOTFOUND -1
#define MCC1608G_NOTALLOWED -2
#define MCC1608G_CONFIGFAILED -3

class MCC_USB1608G
{
private:
  static MCC_USB1608G *instance_;
  libusb_device_handle *udev_;
  float table_AIN_[NGAINS_1608G][2];
  bool is_configured_;
  bool is_continuous_mode_;
  double frequency_;
  uint8_t gain_;
  uint8_t mode_;
  unsigned int timeout_;
  uint32_t nread_;
  int nchan_;
  uint16_t *sdataIn_;
  int wMaxPacketSize_;

  MCC_USB1608G();

  MCC_USB1608G(MCC_USB1608G &);

public:
  /**
   * @brief require an handler of MCC_USB1608G
   * @return MCC_USB1608G*
   */
  static MCC_USB1608G *getInstance();

  /**
   * @brief write config to the hardware
   * @return error code
   *    MCC1608G_SUCCESS    success
   *    MCC1608G_NOTFOUND   cannot find the device
   */
  int config();

  /**
   * @brief start scanning
   *
   * currently deprecated
   *
   * @return error code
   */
  int start();

  int stop();

  /**
   * @brief read and convert data to voltage
   * @param data
   * @return number of data, which should be equal to nchan times nread
   */
  int read(double *data);

  void blink(uint8_t count);

  /**
   * @brief read raw data
   * @param sdataIn where to store returned data
   * @return number of data, which should be equal to nchan times nread
   */
  int readRawData(uint16_t *sdataIn);

  void setFrequency(double frequency);

  /**
   * @brief set gain
   * @param gain  BP_10V || BP_5V || BP_2V || BP_1V
   */
  void setGain(uint8_t gain);

  /**
   * @brief set mode
   * @param mode SINGLE_ENDED || DIFFERENTIAL
   */
  void setMode(uint8_t mode);

  /**
   * @brief set number of channels
   * @param nchan 0~16
   */
  void setNChannel(int nchan);

  void setNRead(int nread);

  /**
   * @brief configure timeout in millisecond
   * @param timeout
   */
  void setTimeout(unsigned int timeout);

  /**
   * @brief set continuous scan
   *
   * NOTE:
   *
   *    if scan continuously, set NChannel*NRead = 256*k (k >= 1)
   *    otherwise, set NChannel*NRead <= 256
   *
   * @param isContinuouslyScan
   */
  void setContinuousScanMode(bool isContinuouslyScan);

  ~MCC_USB1608G();
};

#endif  // __MCC_1608G_H__
