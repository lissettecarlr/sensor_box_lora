#ifndef __TYPE_H__
#define __TYPE_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "stm32l1xx.h"

#define PRODUCT_NAME                            "M100D"
#define FIRMWARE_VERSION                        "0.9.1"

#if defined( USE_BAND_470 )
#define BAND_USE                                "CN470-510"
#elif defined( USE_BAND_868 )
#define BAND_USE                                "EU863-870"
#elif defined( USE_BAND_915 )
#define BAND_USE                                "US902-928"
#else
#error "Please define a frequency band in the compiler options."
#endif

#endif
