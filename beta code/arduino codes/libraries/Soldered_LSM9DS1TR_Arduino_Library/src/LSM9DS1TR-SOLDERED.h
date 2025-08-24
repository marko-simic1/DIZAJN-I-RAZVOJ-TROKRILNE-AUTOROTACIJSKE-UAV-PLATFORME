/**
 **************************************************
 *
 * @file        LSM9DS1TR-SOLDERED.h
 * @brief       Header file for LSM9DS1TR.
 *
 *
 * @copyright   GNU General Public License v3.0
 * @authors     Karlo Leksic @ soldered.com
 ***************************************************/

#ifndef __LSM9DS1TR_SOLDERED__
#define __LSM9DS1TR_SOLDERED__

#include "Arduino.h"
#include "libs/SparkFun_LSM9DS1_Arduino_Library/src/SparkFunLSM9DS1.h"

class LSM9DS1TR : public LSM9DS1
{
  public:
    LSM9DS1TR() : LSM9DS1()
    {
    }

  protected:
  private:
};

#endif
