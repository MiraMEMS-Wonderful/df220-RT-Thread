
#include "sensor_mira_df220.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.mira.df220"
#define DBG_COLOR
#include <rtdbg.h>

#define GRAVITY_EARTH (9.80665f)

static void rt_delay_ms(uint32_t period)
{
    rt_thread_mdelay(period);
}

static int8_t rt_i2c_write_reg(void *intf_ptr, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(intf_ptr, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int8_t rt_i2c_read_reg(void *intf_ptr, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(intf_ptr, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static struct df220_dev *_df220_create(struct rt_sensor_intf *intf)
{
    struct df220_dev *_df220_dev = RT_NULL;
    struct rt_i2c_bus_device *i2c_bus_dev = RT_NULL;
	
    int8_t rslt = DF220_OK;
    struct df220_sensor_conf conf;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        LOG_E("can not find device %s", intf->dev_name);
        return RT_NULL;
    }

    _df220_dev = rt_calloc(1, sizeof(struct df220_dev));
    if (_df220_dev == RT_NULL)
    {
        LOG_E("df220 dev memory allocation failed");
        return RT_NULL;
    }

    _df220_dev->dev_id   = (rt_uint32_t)(intf->user_data) & 0xff;
    _df220_dev->intf     = DF220_I2C_INTF;
    _df220_dev->intf_ptr = i2c_bus_dev;
    _df220_dev->read     = rt_i2c_read_reg;
    _df220_dev->write    = rt_i2c_write_reg;
    _df220_dev->delay_ms = rt_delay_ms;

    rslt = df220_init(_df220_dev);
    if (rslt == DF220_OK)
    {
        rslt = df220_soft_reset(_df220_dev);

        /* Select the type of configuration to be modified */
        conf.type = DF220_FORCE;

        /* Get the forceerometer configurations which are set in the sensor */
        rslt = df220_get_sensor_conf(&conf, 1, _df220_dev);

        /* Modify the desired configurations as per macros
         * available in df220_defs.h file */
        conf.param.force.odr = DF220_ODR_125HZ;
        conf.param.force.range = DF220_2G_RANGE;

        df220_set_power_mode(DF220_SLEEP_MODE, _df220_dev);

        return _df220_dev;
    }
    else
    {
        LOG_E("df220 init failed, %d", rslt);
        rt_free(_df220_dev);
        return RT_NULL;
    }
}

static rt_err_t _df220_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    struct df220_dev *_df220_dev = sensor->parent.user_data;
    struct df220_sensor_conf conf;
    uint8_t odr_ctr;

    if (odr == 1)
        odr_ctr = DF220_ODR_1_HZ;
    else if (odr <= 2)
        odr_ctr = DF220_ODR_1_95HZ;
    else if (odr <= 4)
        odr_ctr = DF220_ODR_3_9HZ;
    else if (odr <= 8)
        odr_ctr = DF220_ODR_7_81HZ;
    else if (odr <= 16)
        odr_ctr = DF220_ODR_15_63HZ;
    else if (odr <= 32)
        odr_ctr = DF220_ODR_31_25HZ;
    else if (odr <= 63)
        odr_ctr = DF220_ODR_62_5HZ;
    else if (odr <= 125)
        odr_ctr = DF220_ODR_125HZ;
    else if (odr <= 250)
        odr_ctr = DF220_ODR_250HZ;
    else if (odr <= 500)
        odr_ctr = DF220_ODR_500HZ;
    else
        odr_ctr = DF220_ODR_1000HZ;

    if (sensor->info.type == RT_SENSOR_CLASS_FORCE)
    {
        conf.type = DF220_FORCE;

        /* Get the forceerometer configurations which are set in the sensor */
        df220_get_sensor_conf(&conf, 1, _df220_dev);

        conf.param.force.odr = odr_ctr;

        /* Set the desired configurations to the sensor */
        df220_set_sensor_conf(&conf, 1, _df220_dev);
        return RT_EOK;
    }

    return RT_EOK;
}

static rt_err_t _df220_set_range(rt_sensor_t sensor, rt_uint16_t range)
{
    struct df220_dev *_df220_dev = sensor->parent.user_data;

    if (sensor->info.type == RT_SENSOR_CLASS_FORCE)
    {
        struct df220_sensor_conf conf;
        uint8_t range_ctr;

        if (range <= 2000)
            range_ctr = DF220_2G_RANGE;
        else if (range <= 4000)
            range_ctr = DF220_4G_RANGE;
        else if (range <= 8000)
            range_ctr = DF220_8G_RANGE;
        else
            range_ctr = DF220_16G_RANGE;

        conf.type = DF220_FORCE;

        /* Get the forceerometer configurations which are set in the sensor */
        df220_get_sensor_conf(&conf, 1, _df220_dev);

        conf.param.force.range = range_ctr;

        /* Set the desired configurations to the sensor */
        df220_set_sensor_conf(&conf, 1, _df220_dev);
        return RT_EOK;
    }

    return RT_EOK;
}

static rt_err_t _df220_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    struct df220_dev *_df220_dev = sensor->parent.user_data;
    int8_t rslt = 0;

    if (power == RT_SENSOR_POWER_DOWN)
    {
        rslt = df220_set_power_mode(DF220_SLEEP_MODE, _df220_dev);
    }
    else if ((power == RT_SENSOR_POWER_NORMAL)||(power == RT_SENSOR_POWER_LOW))
    {
        rslt = df220_set_power_mode(DF220_NORMAL_MODE, _df220_dev);
    }
    else
    {
        LOG_W("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }

    return rslt;
}

static rt_size_t df220_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct df220_dev *_df220_dev = sensor->parent.user_data;
    struct rt_sensor_data *data = buf;

    if (sensor->info.type == RT_SENSOR_CLASS_FORCE)
    {
        struct df220_sensor_data comp_data;
        df220_get_force_data(&comp_data, _df220_dev);

        data->type = RT_SENSOR_CLASS_FORCE;
        data->data.force = comp_data.f;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_err_t df220_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    struct df220_dev *_df220_dev = sensor->parent.user_data;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t *)args = _df220_dev->chip_id;
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = _df220_set_odr(sensor, (rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _df220_set_range(sensor, (rt_uint32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _df220_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    default:
        return -RT_EINVAL;
    }

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    df220_fetch_data,
    df220_control
};

int rt_hw_df220_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_acce = RT_NULL;
    struct df220_dev *_df220_dev = RT_NULL;

    /* forceerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -RT_ERROR;

        sensor_acce->info.type       = RT_SENSOR_CLASS_FORCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_MIRAMEMS;
        sensor_acce->info.model      = "df220_force";
        sensor_acce->info.unit       = RT_SENSOR_UNIT_MN;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_acce->info.range_max  = 14000;
        sensor_acce->info.range_min  = 7000;
        sensor_acce->info.period_min = 1;

        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        sensor_acce->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            rt_free(sensor_acce);
            return -RT_ERROR;
        }
    }

    LOG_I("force sensor init success");

    _df220_dev = _df220_create(&cfg->intf);
    if (_df220_dev == RT_NULL)
    {
        LOG_E("sensor create failed");
        return -RT_ERROR;
    }

    sensor_acce->parent.user_data = _df220_dev;

    return RT_EOK;
}
