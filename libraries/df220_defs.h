/**
 * Copyright (C) 2012 - 2019 MiraMEMS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	   df220_defs.h
 * @date       17 Apr 2019
 * @version    0.0.1
 * @brief
 *
 */
/*! \file df220_defs.h */
/*!
 * @defgroup DF220 SENSOR API
 * @brief
 * @{
 */
#ifndef DF220_DEFS_H_
#define DF220_DEFS_H_
/*********************************************************************/
/**\ header files */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif
/*********************************************************************/
/** \name		Common macros				     */
/*********************************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)       S8_C(x)
#define UINT8_C(x)      U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)      S16_C(x)
#define UINT16_C(x)     U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)      S32_C(x)
#define UINT32_C(x)     U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)      S64_C(x)
#define UINT64_C(x)     U64_C(x)
#endif
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL   0
#else
#define NULL   ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE     UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE    UINT8_C(0)
#endif
/********************************************************/
/**\name Macro definitions */
/**\name API success code */
#define DF220_OK       INT8_C(0)
/**\name API error codes */
#define DF220_E_NULL_PTR          INT8_C(-1)
#define DF220_E_COM_FAIL          INT8_C(-2)
#define DF220_E_DEV_NOT_FOUND     INT8_C(-3)
#define DF220_E_INVALID_CONFIG    INT8_C(-4)
/**\name API warning codes */
#define DF220_W_SELF_TEST_FAIL    INT8_C(1)
/**\name CHIP ID VALUE */
#define DF220_CHIP_ID     UINT8_C(0x13)
/********************************************************/
/**\name	USER CONFIGURATION MACROS               */
/**\name Power mode configurations */
#define DF220_NORMAL_MODE       UINT8_C(0)
#define DF220_SLEEP_MODE        UINT8_C(1)
#define DF220_LOW_POWER_MODE    UINT8_C(0)
/**\name Enable / Disable macros */
#define DF220_DISABLE        UINT8_C(0)
#define DF220_ENABLE         UINT8_C(1)
/**\name Data/sensortime selection macros */
#define DF220_DATA_ONLY          UINT8_C(0x00)
#define DF220_DATA_SENSOR_TIME   UINT8_C(0x01)
/**\name ODR configurations  */
#define DF220_ODR_1_HZ       UINT8_C(0x00)
#define DF220_ODR_1_95HZ     UINT8_C(0x01)
#define DF220_ODR_3_9HZ      UINT8_C(0x02)
#define DF220_ODR_7_81HZ     UINT8_C(0x03)
#define DF220_ODR_15_63HZ    UINT8_C(0x04)
#define DF220_ODR_31_25HZ    UINT8_C(0x05)
#define DF220_ODR_62_5HZ     UINT8_C(0x06)
#define DF220_ODR_125HZ      UINT8_C(0x07)
#define DF220_ODR_250HZ      UINT8_C(0x08)
#define DF220_ODR_500HZ      UINT8_C(0x09)
#define DF220_ODR_1000HZ     UINT8_C(0x0A)
/**\name Force sensor Range configuration */
#define DF220_2G_RANGE      UINT8_C(0x00)
#define DF220_4G_RANGE      UINT8_C(0x01)
#define DF220_8G_RANGE      UINT8_C(0x02)
#define DF220_16G_RANGE     UINT8_C(0x03)

/**\name Force sensor bw settings */
/* Force bw = 0.50 * ODR */
#define DF220_FORCE_FILT1_BW_0    UINT8_C(0x00)
/* Force bw = 0.25 * ODR */
#define DF220_FORCE_FILT1_BW_1    UINT8_C(0x01)
/* Force bw = 0.10 * ODR */
#define DF220_FORCE_FILT1_BW_2    UINT8_C(0x10)
/**\name Auto wake-up timeout value of 10.24s */
#define DF220_AUTO_WAKEUP_TIMEOUT_MAX  UINT16_C(0x0FFF)
/**\name Auto low power timeout value of 10.24s */
#define DF220_AUTO_LP_TIMEOUT_MAX      UINT16_C(0x0FFF)

/**********************************************************************/
/**\name DF220 Register Address */
#define DF220_CHIP_ID_ADDR              UINT8_C(0x01)
#define DF220_FORCE_DATA_ADDR           UINT8_C(0x06)
#define DF220_INT_STAT0_ADDR            UINT8_C(0x0E)
#define DF220_FORCE_MODE_BW             UINT8_C(0x11)
#define DF220_FORCE_LSB_RANGE           UINT8_C(0x0F)
#define DF220_INT_MAP1_ADDR             UINT8_C(0x19)
#define DF220_INT_MAP2_ADDR             UINT8_C(0x1A)
#define DF220_INT_MAP3_ADDR             UINT8_C(0x1B)
#define DF220_SPI_CONFIG                UINT8_C(0x00)
/**\name DF220 Command register */
#define DF220_SOFT_RESET_CMD            UINT8_C(0x24)

#define DF220_SOFT_RESET_CMD_MSK       DF220_SOFT_RESET_CMD
/**\name DF220 Delay definitions */
#define DF220_SOFT_RESET_DELAY_MS       UINT8_C(10)
#define DF220_SELF_TEST_DELAY_MS        UINT8_C(7)
#define DF220_SELF_TEST_DATA_READ_MS    UINT8_C(50)
/**\name Interface selection macro */
#define DF220_SPI_WR_MASK    UINT8_C(0x7F)
#define DF220_SPI_RD_MASK    UINT8_C(0x80)

/**\name DF220 bit mask definitions */
#define DF220_POWER_MODE_STATUS_MSK         UINT8_C(0x06)
#define DF220_POWER_MODE_STATUS_POS         UINT8_C(1)

#define DF220_POWER_MODE_MSK      UINT8_C(0x80)
#define DF220_POWER_MODE_POS      UINT8_C(7)

#define DF220_FORCEL_ODR_MSK      UINT8_C(0x0F)

#define DF220_FORCE_RANGE_MSK    UINT8_C(0x03)
#define DF220_FORCE_RANGE_POS    UINT8_C(0)

#define DF220_DATA_FILTER_MSK   UINT8_C(0x0C)
#define DF220_DATA_FILTER_POS   UINT8_C(2)

#define DF220_BW_MSK            UINT8_C(0x06)
#define DF220_BW_POS            UINT8_C(1)

/**\name Macro to SET and GET BITS of a register */
#define DF220_SET_BITS(reg_data, bitname, data) \
	((reg_data & ~(bitname ## _MSK)) |	 \
	 ((data << bitname ## _POS) & bitname ## _MSK))

#define DF220_GET_BITS(reg_data, bitname)  ((reg_data & (bitname ## _MSK)) >> \
					     (bitname ## _POS))

#define DF220_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname ## _MSK)) |	       \
	 (data & bitname ## _MSK))

#define DF220_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname ## _MSK))

/********************************************************/
/*!
 * @brief Interface selection enums
 */
enum df220_intf {
	/*! SPI interface */
	DF220_SPI_INTF,
	/*! I2C interface */
	DF220_I2C_INTF
};
/********************************************************/
/**\name	TYPE DEFINITIONS */
/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read and write functions of the user
 */
typedef int8_t (*df220_com_fptr_t)(void *intf_ptr, uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
				    uint16_t length);
/*!	Delay function pointer */
typedef void (*df220_delay_fptr_t)(uint32_t period);
/********************************************************/
/**\name	STRUCTURE DEFINITIONS*/
/*!
 * @brief Sensor selection enums
 */
enum df220_sensor {
	DF220_FORCE
};

/*!
 * @brief Force sensor basic configuration
 */
struct df220_force_conf {
	/*! Output data rate
	 * Assignable macros :
	 *                      - DF220_ODR_1_HZ   - DF220_ODR_1_95HZ
	 *  - DF220_ODR_3_9HZ   - DF220_ODR_7_81HZ  - DF220_ODR_15_63HZ
	 *  - DF220_ODR_31_25HZ - DF220_ODR_62_5HZ  - DF220_ODR_125HZ   
	 *  - DF220_ODR_250HZ   - DF220_ODR_500HZ   - DF220_ODR_1000HZ
	 */
	uint8_t odr;
	/*! Range of sensor
	 * Assignable macros :
	 *  - DF220_2G_RANGE   - DF220_8G_RANGE
	 *  - DF220_4G_RANGE   - DF220_16G_RANGE
	 */
	uint8_t range;

	/*! Filter 1 Bandwidth
	 * Assignable macros :
	 *  - DF220_FORCE_FILT1_BW_0
	 *  - DF220_FORCE_FILT1_BW_1
	 *  - DF220_FORCE_FILT1_BW_2
	 */
	uint8_t bw;
};
/*!
 * @brief Union of sensor Configurations
 */
union df220_set_param {
	/* Force sensor configurations */
	struct df220_force_conf force;
};
/*!
 * @brief Sensor selection and their configurations
 */
struct df220_sensor_conf {
	/* Sensor selection */
	enum df220_sensor type;
	/* Sensor configuration */
	union df220_set_param param;
};
/*!
 * @brief DF220 sensor data
 */
struct df220_sensor_data {
	/*! Force sensor data */
	int16_t f;
	/*! sensor time */
	uint32_t sensortime;
};

/*!
 * @brief df220 device structure
 */
struct df220_dev {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Device Id */
	uint8_t dev_id;
	/*! SPI/I2C Interface selection */
	enum df220_intf intf;
	/*! Interface handle pointer */
	void *intf_ptr;
	/*! Decide SPI or I2C read mechanism */
	uint8_t dummy_byte;
	/*! Bus read function pointer */
	df220_com_fptr_t read;
	/*! Bus write function pointer */
	df220_com_fptr_t write;
	/*! delay(in ms) function pointer */
	df220_delay_fptr_t delay_ms;
};

#endif /* DF220_DEFS_H_ */
/** @}*/
/** @}*/
