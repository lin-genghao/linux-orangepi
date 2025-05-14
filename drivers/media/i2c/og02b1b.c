// SPDX-License-Identifier: GPL-2.0
/*
 * og02b1b driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 add enum_frame_interval function.
 * V0.0X01.0X04 add quick stream on/off
 * V0.0X01.0X05 add function g_mbus_config
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>

#define DRIVER_VERSION                  KERNEL_VERSION(0, 0x01, 0x5)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN           V4L2_CID_GAIN
#endif

#define OG02B1B_LINK_FREQ_100MHZ         100000000LL
#define OG02B1B_LINK_FREQ_320MHZ         320000000LL
#define OG02B1B_LINK_FREQ_400MHZ         400000000LL
#define OG02B1B_LINK_FREQ_800MHZ         800000000LL
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define OG02B1B_PIXEL_RATE               (OG02B1B_LINK_FREQ_400MHZ * 2LL * 2LL / 10)
//#define OG02B1B_PIXEL_RATE               (OG02B1B_LINK_FREQ_100MHZ)
#define OG02B1B_XVCLK_FREQ               24000000

#define CHIP_ID1                         0x3F
#define OG02B1B_REG_CHIP_ID1             0x7000
#define CHIP_ID2                         0x02
#define OG02B1B_REG_CHIP_ID2             0x7001
#define CHIP_ID3                         0x0B
#define OG02B1B_REG_CHIP_ID3             0x7002

#define OG02B1B_REG_CTRL_MODE            0x0100
#define OG02B1B_MODE_SW_STANDBY          0x0
#define OG02B1B_MODE_STREAMING           BIT(0)

#define OG02B1B_REG_EXPOSURE             0x3500
#define OG02B1B_EXPOSURE_MIN             4
#define OG02B1B_EXPOSURE_STEP            1
#define OG02B1B_VTS_MAX                  0x7fff

#define OG02B1B_REG_GAIN_H               0x3508
#define OG02B1B_REG_GAIN_L               0x3509
#define OG02B1B_GAIN_H_MASK              0x07
#define OG02B1B_GAIN_H_SHIFT             8
#define OG02B1B_GAIN_L_MASK              0xff
#define OG02B1B_GAIN_MIN                 0x10
#define OG02B1B_GAIN_MAX                 0xf8
#define OG02B1B_GAIN_STEP                1
#define OG02B1B_GAIN_DEFAULT             0x10

#define OG02B1B_REG_TEST_PATTERN         0x5e00
#define OG02B1B_TEST_PATTERN_ENABLE      0x80
#define OG02B1B_TEST_PATTERN_DISABLE     0x0

#define OG02B1B_REG_VTS                  0x380e

#define OG02B1B_AEC_STROBE_REG           0x3927
#define OG02B1B_AEC_STROBE_REG_H         0x3927
#define OG02B1B_AEC_STROBE_REG_L         0x3928

#define OV9282_AEC_GROUP_UPDATE_ADDRESS         0x3208
#define OV9282_AEC_GROUP_UPDATE_START_DATA      0x00
#define OV9282_AEC_GROUP_UPDATE_END_DATA        0x10
#define OV9282_AEC_GROUP_UPDATE_END_LAUNCH      0xA0

#define REG_NULL                        0xFFFF

#define OG02B1B_REG_VALUE_08BIT          1
#define OG02B1B_REG_VALUE_16BIT          2
#define OG02B1B_REG_VALUE_24BIT          3

#define OG02B1B_LANES                    2
#define OG02B1B_BITS_PER_SAMPLE          10

#define OF_CAMERA_PINCTRL_STATE_DEFAULT "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP   "rockchip,camera_sleep"

#define OG02B1B_NAME                     "og02b1b"


//for SL
#define OV9282_FPS              30
#define OV9282_FLIP_ENABLE      1
#define EXP_DEFAULT_TIME_US     3000
#define OV9282_DEFAULT_GAIN     1

#define OV9282_VTS_30_FPS       0xe48
#define OV9282_HTS_30_FPS       0x2d8

#define FPS_HTS_MODE            1
#if FPS_HTS_MODE
#define OV9282_VTS              OV9282_VTS_30_FPS
#define OV9282_HTS              (OV9282_HTS_30_FPS * 30 / OV9282_FPS)
#else
#define OV9282_VTS              (OV9282_HTS_30_FPS * 30 / OV9282_FPS)
#define OV9282_HTS              OV9282_VTS_30_FPS
#endif

#define TIME_MS                 1000

#define OV9282_EXP_TIME_REG     ((uint16_t)(EXP_DEFAULT_TIME_US / 1000 * \
                                OV9282_FPS * OV9282_VTS / TIME_MS) << 4)
#define OV9282_STROBE_TIME_REG  (OV9282_EXP_TIME_REG >> 4)


static const char * const og02b1b_supply_names[] = {
        "avdd",         /* Analog power */
        "dovdd",        /* Digital I/O power */
        "dvdd",         /* Digital core power */
};

#define OG02B1B_NUM_SUPPLIES ARRAY_SIZE(og02b1b_supply_names)

struct regval {
        u16 addr;
        u8 val;
};

struct og02b1b_mode {
        u32 width;
        u32 height;
        struct v4l2_fract max_fps;
        u32 hts_def;
        u32 vts_def;
        u32 exp_def;
        const struct regval *reg_list;
};

struct og02b1b {
        struct i2c_client       *client;
        struct clk              *xvclk;
        struct gpio_desc        *reset_gpio;
        struct gpio_desc        *pwdn_gpio;
        struct regulator_bulk_data supplies[OG02B1B_NUM_SUPPLIES];

        struct pinctrl          *pinctrl;
        struct pinctrl_state    *pins_default;
        struct pinctrl_state    *pins_sleep;

        struct v4l2_subdev      subdev;
        struct media_pad        pad;
        struct v4l2_ctrl_handler ctrl_handler;
        struct v4l2_ctrl        *exposure;
        struct v4l2_ctrl        *anal_gain;
        struct v4l2_ctrl        *digi_gain;
        struct v4l2_ctrl        *hblank;
        struct v4l2_ctrl        *vblank;
        struct v4l2_ctrl        *test_pattern;
        struct v4l2_ctrl        *strobe;
        struct mutex            mutex;
        bool                    streaming;
        bool                    power_on;
        bool                    is_thunderboot;
        bool                    is_thunderboot_ng;
        bool                    is_first_streamoff;
        const struct og02b1b_mode *cur_mode;
        u32                     module_index;
        const char              *module_facing;
        const char              *module_name;
        const char              *len_name;
};

#define to_og02b1b(sd) container_of(sd, struct og02b1b, subdev)

static const struct regval og02b1b_global_regs[] = {
        {0x0103, 0x01},
        {0x0100, 0x00},
        {0x010c, 0x02},
        {0x010b, 0x01},
        {0x0300, 0x01},
        {0x0302, 0x32},
        {0x0303, 0x00},
        {0x0304, 0x03},
        {0x0305, 0x02},
        {0x0306, 0x01},
        {0x030d, 0x5a},
        {0x030e, 0x04},
        {0x3001, 0x02},
        {0x3004, 0x00},
        {0x3005, 0x00},
        {0x3006, 0x0a},
        {0x3011, 0x0d},
        {0x3014, 0x04},
        {0x301c, 0xf0},
        {0x3020, 0x20},
        {0x302c, 0x00},
        {0x302d, 0x00},
        {0x302e, 0x00},
        {0x302f, 0x03},
        {0x3030, 0x10},
        {0x303f, 0x03},
        {0x3103, 0x00},
        {0x3106, 0x08},
        {0x31ff, 0x01},
        {0x3501, 0x05},
        {0x3502, 0x7c},
        {0x3506, 0x00},
        {0x3507, 0x00},
        {0x3620, 0x67},
        {0x3633, 0x78},
        {0x3662, 0x65},
        {0x3664, 0xb0},
        {0x3666, 0x70},
        {0x3670, 0x68},
        {0x3674, 0x10},
        {0x3675, 0x00},
        {0x367e, 0x90},
        {0x3680, 0x84},
        {0x3683, 0x96},
        {0x36a2, 0x04},
        {0x36a3, 0x80},
        {0x36b0, 0x00},
        {0x3700, 0x35},
        {0x3704, 0x39},
        {0x370a, 0x50},
        {0x3712, 0x00},
        {0x3713, 0x02},
        {0x3778, 0x00},
        {0x379b, 0x01},
        {0x379c, 0x10},
        {0x3800, 0x00},
        {0x3801, 0x00},
        {0x3802, 0x00},
        {0x3803, 0x00},
        {0x3804, 0x06},
        {0x3805, 0x4f},
        {0x3806, 0x05},
        {0x3807, 0x23},
        {0x3808, 0x06},
        {0x3809, 0x40},
        {0x380a, 0x05},
        {0x380b, 0x14},
        {0x380c, 0x03},
        {0x380d, 0xa8},
        {0x380e, 0x0b}, //0x05    0b
        {0x380f, 0x10}, //0x88    10
        {0x3810, 0x00},
        {0x3811, 0x08},
        {0x3812, 0x00},
        {0x3813, 0x08},
        {0x3814, 0x11},
        {0x3815, 0x11},
        {0x3816, 0x00},
        {0x3817, 0x01},
        {0x3818, 0x00},
        {0x3819, 0x05},
        {0x3820, 0x00},
        {0x3821, 0x00},
        {0x382b, 0x32},
        {0x382c, 0x0a},
        {0x382d, 0xf8},
        {0x3881, 0x44},
        {0x3882, 0x02},
        {0x3883, 0x8c},
        {0x3885, 0x07},
        {0x389d, 0x03},
        {0x38a6, 0x00},
        {0x38a7, 0x01},
        {0x38b3, 0x07},
        {0x38b1, 0x00},
        {0x38e5, 0x02},
        {0x38e7, 0x00},
        {0x38e8, 0x00},
        {0x3910, 0xff},
        {0x3911, 0xff},
        {0x3912, 0x08},
        {0x3913, 0x00},
        {0x3914, 0x00},
        {0x3915, 0x00},
        {0x391c, 0x00},
        {0x3920, 0xff},
        {0x3921, 0x80},
        {0x3922, 0x00},
        {0x3923, 0x00},
        {0x3924, 0x05},
        {0x3925, 0x00},
        {0x3926, 0x00},
        {0x3927, 0x00},
        {0x3928, 0x1a},
        {0x392d, 0x03},
        {0x392e, 0xa8},
        {0x392f, 0x08},
        {0x4001, 0x00},
        {0x4003, 0x40},
        {0x4008, 0x04},
        {0x4009, 0x1b},
        {0x400c, 0x04},
        {0x400d, 0x1b},
        {0x4010, 0xf4},
        {0x4011, 0x00},
        {0x4016, 0x00},
        {0x4017, 0x04},
        {0x4042, 0x11},
        {0x4043, 0x70},
        {0x4045, 0x00},
        {0x4409, 0x5f},
        {0x4509, 0x00},
        {0x450b, 0x00},
        {0x4600, 0x00},
        {0x4601, 0xa0},
        {0x4708, 0x09},
        {0x470c, 0x81},
        {0x4710, 0x06},
        {0x4711, 0x00},
        {0x4800, 0x00},
        {0x481f, 0x30},
        {0x4837, 0x14},
        {0x4f00, 0x00},
        {0x4f07, 0x00},
        {0x4f08, 0x03},
        {0x4f09, 0x08},
        {0x4f0c, 0x05},
        {0x4f0d, 0xb4},
        {0x4f10, 0x00},
        {0x4f11, 0x00},
        {0x4f12, 0x07},
        {0x4f13, 0xe2},
        {0x5000, 0x1f},
        {0x5001, 0x20},
        {0x5026, 0x00},
        {0x5c00, 0x00},
        {0x5c01, 0x2c},
        {0x5c02, 0x00},
        {0x5c03, 0x7f},
        {0x5e00, 0x00},
        {0x5e01, 0x41},
        {0x38b1, 0x03},
        {REG_NULL, 0x00},
};
static const struct og02b1b_mode supported_modes[] = {
           {
                .width = 1600,
                .height = 1300,
                .max_fps = {
                        .numerator = 10000,
                        .denominator = 300000,
                  },
                  .exp_def = 0x0320,
                  .hts_def = 0x03a8 * 2,
                  .vts_def = 0x0b10,
                  .reg_list = og02b1b_global_regs,
        },
};

static const s64 link_freq_menu_items[] = {
        OG02B1B_LINK_FREQ_400MHZ
};

static const char * const og02b1b_test_pattern_menu[] = {
        "Disabled",
        "Vertical Color Bar Type 1",
        "Vertical Color Bar Type 2",
        "Vertical Color Bar Type 3",
        "Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int og02b1b_write_reg(struct i2c_client *client, u16 reg,
                            u32 len, u32 val)
{
        u32 buf_i, val_i;
        u8 buf[6];
        u8 *val_p;
        __be32 val_be;

        if (len > 4)
                return -EINVAL;

        buf[0] = reg >> 8;
        buf[1] = reg & 0xff;

        val_be = cpu_to_be32(val);
        val_p = (u8 *)&val_be;
        buf_i = 2;
        val_i = 4 - len;

        while (val_i < 4)
                buf[buf_i++] = val_p[val_i++];

        if (i2c_master_send(client, buf, len + 2) != len + 2)
                return -EIO;

        return 0;
}

static int og02b1b_write_array(struct i2c_client *client,
                              const struct regval *regs)
{
        u32 i;
        int ret = 0;

        for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
                ret = og02b1b_write_reg(client, regs[i].addr,
                                       OG02B1B_REG_VALUE_08BIT, regs[i].val);

        return ret;
}

/* Read registers up to 4 at a time */
static int og02b1b_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
                           u32 *val)
{
        struct i2c_msg msgs[2];
        u8 *data_be_p;
        __be32 data_be = 0;
        __be16 reg_addr_be = cpu_to_be16(reg);
        int ret;

        if (len > 4 || !len)
                return -EINVAL;

        data_be_p = (u8 *)&data_be;
        /* Write register address */
        msgs[0].addr = client->addr;
        msgs[0].flags = 0;
        msgs[0].len = 2;
        msgs[0].buf = (u8 *)&reg_addr_be;

        /* Read data from register */
        msgs[1].addr = client->addr;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = len;
        msgs[1].buf = &data_be_p[4 - len];

        ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
        if (ret != ARRAY_SIZE(msgs))
                return -EIO;

        *val = be32_to_cpu(data_be);

        return 0;
}

static int og02b1b_get_reso_dist(const struct og02b1b_mode *mode,
                                struct v4l2_mbus_framefmt *framefmt)
{
        return abs(mode->width - framefmt->width) +
               abs(mode->height - framefmt->height);
}

static const struct og02b1b_mode *
og02b1b_find_best_fit(struct v4l2_subdev_format *fmt)
{
        struct v4l2_mbus_framefmt *framefmt = &fmt->format;
        int dist;
        int cur_best_fit = 0;
        int cur_best_fit_dist = -1;
        unsigned int i;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
                dist = og02b1b_get_reso_dist(&supported_modes[i], framefmt);
                if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
                        cur_best_fit_dist = dist;
                        cur_best_fit = i;
                }
        }

        return &supported_modes[cur_best_fit];
}

static int og02b1b_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_pad_config *cfg,
                          struct v4l2_subdev_format *fmt)
{
        struct og02b1b *og02b1b = to_og02b1b(sd);
        const struct og02b1b_mode *mode;
        s64 h_blank, vblank_def;
        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        mutex_lock(&og02b1b->mutex);

        mode = og02b1b_find_best_fit(fmt);
        fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
        fmt->format.width = mode->width;
        fmt->format.height = mode->height;
        fmt->format.field = V4L2_FIELD_NONE;
        if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
                *v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
                mutex_unlock(&og02b1b->mutex);
                return -ENOTTY;
#endif
        } else {
                og02b1b->cur_mode = mode;
                h_blank = mode->hts_def - mode->width;
                __v4l2_ctrl_modify_range(og02b1b->hblank, h_blank,
                                         h_blank, 1, h_blank);
                vblank_def = mode->vts_def - mode->height;
                __v4l2_ctrl_modify_range(og02b1b->vblank, vblank_def,
                                         OG02B1B_VTS_MAX - mode->height,
                                         1, vblank_def);
        }

        mutex_unlock(&og02b1b->mutex);

        return 0;
}

static int og02b1b_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_pad_config *cfg,
                          struct v4l2_subdev_format *fmt)
{
        struct og02b1b *og02b1b = to_og02b1b(sd);
        const struct og02b1b_mode *mode = og02b1b->cur_mode;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        mutex_lock(&og02b1b->mutex);
        if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
                fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
                mutex_unlock(&og02b1b->mutex);
                return -ENOTTY;
#endif
        } else {
                fmt->format.width = mode->width;
                fmt->format.height = mode->height;
                fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
                fmt->format.field = V4L2_FIELD_NONE;
        }
        mutex_unlock(&og02b1b->mutex);

        return 0;
}

static int og02b1b_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_pad_config *cfg,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
        if (code->index != 0)
                return -EINVAL;
        code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

        return 0;
}

static int og02b1b_enum_frame_sizes(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_pad_config *cfg,
                                   struct v4l2_subdev_frame_size_enum *fse)
{
        if (fse->index >= ARRAY_SIZE(supported_modes))
                return -EINVAL;

        if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
                return -EINVAL;

        fse->min_width  = supported_modes[fse->index].width;
        fse->max_width  = supported_modes[fse->index].width;
        fse->max_height = supported_modes[fse->index].height;
        fse->min_height = supported_modes[fse->index].height;

        return 0;
}

static int og02b1b_enable_test_pattern(struct og02b1b *og02b1b, u32 pattern)
{
        u32 val;

        if (pattern)
                val = (pattern - 1) | OG02B1B_TEST_PATTERN_ENABLE;
        else
                val = OG02B1B_TEST_PATTERN_DISABLE;

        return og02b1b_write_reg(og02b1b->client, OG02B1B_REG_TEST_PATTERN,
                                OG02B1B_REG_VALUE_08BIT, val);
}

static int og02b1b_g_frame_interval(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_frame_interval *fi)
{
        struct og02b1b *og02b1b = to_og02b1b(sd);
        const struct og02b1b_mode *mode = og02b1b->cur_mode;

        mutex_lock(&og02b1b->mutex);
        fi->interval = mode->max_fps;
        mutex_unlock(&og02b1b->mutex);

        return 0;
}

static void og02b1b_get_module_inf(struct og02b1b *og02b1b,
                                  struct rkmodule_inf *inf)
{
        memset(inf, 0, sizeof(*inf));
        strlcpy(inf->base.sensor, OG02B1B_NAME, sizeof(inf->base.sensor));
        strlcpy(inf->base.module, og02b1b->module_name,
                sizeof(inf->base.module));
        strlcpy(inf->base.lens, og02b1b->len_name, sizeof(inf->base.lens));
}

static long og02b1b_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
        struct og02b1b *og02b1b = to_og02b1b(sd);
        long ret = 0;
        u32 stream = 0;

        switch (cmd) {
        case RKMODULE_GET_MODULE_INFO:
                og02b1b_get_module_inf(og02b1b, (struct rkmodule_inf *)arg);
                break;
        case RKMODULE_SET_QUICK_STREAM:

                stream = *((u32 *)arg);

                if (stream)
                        ret = og02b1b_write_reg(og02b1b->client, OG02B1B_REG_CTRL_MODE,
                                OG02B1B_REG_VALUE_08BIT, OG02B1B_MODE_STREAMING);
                else
                        ret = og02b1b_write_reg(og02b1b->client, OG02B1B_REG_CTRL_MODE,
                                OG02B1B_REG_VALUE_08BIT, OG02B1B_MODE_SW_STANDBY);
                break;
        default:
                ret = -ENOIOCTLCMD;
                break;
        }

        return ret;
}

#ifdef CONFIG_COMPAT
static long og02b1b_compat_ioctl32(struct v4l2_subdev *sd,
                                  unsigned int cmd, unsigned long arg)
{
        void __user *up = compat_ptr(arg);
        struct rkmodule_inf *inf;
        struct rkmodule_awb_cfg *cfg;
        long ret;
        u32 stream = 0;

        switch (cmd) {
        case RKMODULE_GET_MODULE_INFO:
                inf = kzalloc(sizeof(*inf), GFP_KERNEL);
                if (!inf) {
                        ret = -ENOMEM;
                        return ret;
                }

                ret = og02b1b_ioctl(sd, cmd, inf);
                if (!ret)
                        ret = copy_to_user(up, inf, sizeof(*inf));
                kfree(inf);
                break;
        case RKMODULE_AWB_CFG:
                cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
                if (!cfg) {
                        ret = -ENOMEM;
                        return ret;
                }

                ret = copy_from_user(cfg, up, sizeof(*cfg));
                if (!ret)
                        ret = og02b1b_ioctl(sd, cmd, cfg);
                kfree(cfg);
                break;
        case RKMODULE_SET_QUICK_STREAM:
                ret = copy_from_user(&stream, up, sizeof(u32));
                if (!ret)
                        ret = og02b1b_ioctl(sd, cmd, &stream);
                break;
        default:
                ret = -ENOIOCTLCMD;
                break;
        }

        return ret;
}
#endif

static int __og02b1b_start_stream(struct og02b1b *og02b1b)
{
        int ret;

        if (!og02b1b->is_thunderboot) {
                ret = og02b1b_write_array(og02b1b->client, og02b1b->cur_mode->reg_list);
                if (ret)
                        return ret;
        }
        /* In case these controls are set before streaming */
        mutex_unlock(&og02b1b->mutex);
        ret = v4l2_ctrl_handler_setup(&og02b1b->ctrl_handler);
        mutex_lock(&og02b1b->mutex);
        if (ret)
                return ret;

        return og02b1b_write_reg(og02b1b->client, OG02B1B_REG_CTRL_MODE,
                                OG02B1B_REG_VALUE_08BIT, OG02B1B_MODE_STREAMING);
}

static int __og02b1b_stop_stream(struct og02b1b *og02b1b)
{
        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        if (og02b1b->is_thunderboot)
                og02b1b->is_first_streamoff = true;
        return og02b1b_write_reg(og02b1b->client, OG02B1B_REG_CTRL_MODE,
                                OG02B1B_REG_VALUE_08BIT, OG02B1B_MODE_SW_STANDBY);
}

static int og02b1b_s_stream(struct v4l2_subdev *sd, int on)
{
        struct og02b1b *og02b1b = to_og02b1b(sd);
        struct i2c_client *client = og02b1b->client;
        int ret = 0;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        mutex_lock(&og02b1b->mutex);
        on = !!on;
        if (on == og02b1b->streaming)
                goto unlock_and_return;

        if (on) {
                ret = pm_runtime_get_sync(&client->dev);
                if (ret < 0) {
                        pm_runtime_put_noidle(&client->dev);
                        goto unlock_and_return;
                }

                ret = __og02b1b_start_stream(og02b1b);
                if (ret) {
                        v4l2_err(sd, "start stream failed while write regs\n");
                        pm_runtime_put(&client->dev);
                        goto unlock_and_return;
                }
        } else {
                __og02b1b_stop_stream(og02b1b);
                pm_runtime_put(&client->dev);
        }

        og02b1b->streaming = on;

unlock_and_return:
        mutex_unlock(&og02b1b->mutex);

        return ret;
}

static int og02b1b_s_power(struct v4l2_subdev *sd, int on)
{
        struct og02b1b *og02b1b = to_og02b1b(sd);
        struct i2c_client *client = og02b1b->client;
        int ret = 0;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        mutex_lock(&og02b1b->mutex);

        /* If the power state is not modified - no work to do. */
        if (og02b1b->power_on == !!on)
                goto unlock_and_return;

        if (on) {
                ret = pm_runtime_get_sync(&client->dev);
                if (ret < 0) {
                        pm_runtime_put_noidle(&client->dev);
                        goto unlock_and_return;
                }
                ret = og02b1b_write_array(og02b1b->client, og02b1b_global_regs);
                if (ret) {
                        v4l2_err(sd, "could not set init registers\n");
                        pm_runtime_put_noidle(&client->dev);
                        goto unlock_and_return;
                }
                og02b1b->power_on = true;
        } else {
                pm_runtime_put(&client->dev);
                og02b1b->power_on = false;
        }

unlock_and_return:
        mutex_unlock(&og02b1b->mutex);

        return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 og02b1b_cal_delay(u32 cycles)
{
        return DIV_ROUND_UP(cycles, OG02B1B_XVCLK_FREQ / 1000 / 1000);
}

static int __og02b1b_power_on(struct og02b1b *og02b1b)
{
        int ret;
        u32 delay_us;
        struct device *dev = &og02b1b->client->dev;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        /* No need when thunderboot. */
        if (og02b1b->is_thunderboot) {
                return 0;
        }

        if (!IS_ERR_OR_NULL(og02b1b->pins_default)) {
                ret = pinctrl_select_state(og02b1b->pinctrl,
                                           og02b1b->pins_default);
                if (ret < 0)
                        dev_err(dev, "could not set pins\n");
        }

        ret = clk_set_rate(og02b1b->xvclk, OG02B1B_XVCLK_FREQ);
        if (ret < 0)
                dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
        if (clk_get_rate(og02b1b->xvclk) != OG02B1B_XVCLK_FREQ)
                dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
        ret = clk_prepare_enable(og02b1b->xvclk);
        if (ret < 0) {
                dev_err(dev, "Failed to enable xvclk\n");
                return ret;
        }

        if (!IS_ERR(og02b1b->reset_gpio))
                gpiod_set_value_cansleep(og02b1b->reset_gpio, 1);

        ret = regulator_bulk_enable(OG02B1B_NUM_SUPPLIES, og02b1b->supplies);
        if (ret < 0) {
                dev_err(dev, "Failed to enable regulators\n");
                goto disable_clk;
        }

        if (!IS_ERR(og02b1b->reset_gpio))
                gpiod_set_value_cansleep(og02b1b->reset_gpio, 0);

        usleep_range(500, 1000);
        if (!IS_ERR(og02b1b->pwdn_gpio))
                gpiod_set_value_cansleep(og02b1b->pwdn_gpio, 1);

        /* 8192 cycles prior to first SCCB transaction */
        delay_us = og02b1b_cal_delay(8192);
        usleep_range(delay_us, delay_us * 2);

        return 0;

disable_clk:
        clk_disable_unprepare(og02b1b->xvclk);

        return ret;
}

static void __og02b1b_power_off(struct og02b1b *og02b1b)
{
        int ret;
        struct device *dev = &og02b1b->client->dev;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        if (og02b1b->is_thunderboot) {
                if (og02b1b->is_first_streamoff) {
                        og02b1b->is_thunderboot = false;
                        og02b1b->is_first_streamoff = false;
                } else {
                        return;
                }
        }

        if (!IS_ERR(og02b1b->pwdn_gpio))
                gpiod_set_value_cansleep(og02b1b->pwdn_gpio, 0);
        clk_disable_unprepare(og02b1b->xvclk);
        if (!IS_ERR(og02b1b->reset_gpio))
                gpiod_set_value_cansleep(og02b1b->reset_gpio, 1);
        if (!IS_ERR_OR_NULL(og02b1b->pins_sleep)) {
                ret = pinctrl_select_state(og02b1b->pinctrl,
                                           og02b1b->pins_sleep);
                if (ret < 0)
                        dev_dbg(dev, "could not set pins\n");
        }
        regulator_bulk_disable(OG02B1B_NUM_SUPPLIES, og02b1b->supplies);
}

static int og02b1b_runtime_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct og02b1b *og02b1b = to_og02b1b(sd);

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        return __og02b1b_power_on(og02b1b);
}

static int og02b1b_runtime_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct og02b1b *og02b1b = to_og02b1b(sd);

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);

        __og02b1b_power_off(og02b1b);

        return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int og02b1b_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
        struct og02b1b *og02b1b = to_og02b1b(sd);
        struct v4l2_mbus_framefmt *try_fmt =
                                v4l2_subdev_get_try_format(sd, fh->pad, 0);
        const struct og02b1b_mode *def_mode = &supported_modes[0];

        mutex_lock(&og02b1b->mutex);
        /* Initialize try_fmt */
        try_fmt->width = def_mode->width;
        try_fmt->height = def_mode->height;
        try_fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
        try_fmt->field = V4L2_FIELD_NONE;

        mutex_unlock(&og02b1b->mutex);
        /* No crop or compose */

        return 0;
}
#endif

static int og02b1b_enum_frame_interval(struct v4l2_subdev *sd,
                                      struct v4l2_subdev_pad_config *cfg,
                                      struct v4l2_subdev_frame_interval_enum *fie)
{
        if (fie->index >= ARRAY_SIZE(supported_modes))
                return -EINVAL;

        if (fie->code != MEDIA_BUS_FMT_SBGGR10_1X10)
                return -EINVAL;

        fie->width = supported_modes[fie->index].width;
        fie->height = supported_modes[fie->index].height;
        fie->interval = supported_modes[fie->index].max_fps;
        return 0;
}

static int og02b1b_g_mbus_config(struct v4l2_subdev *sd,
                                unsigned int pad,
                                struct v4l2_mbus_config *config)
{
        u32 val = 0;

        val = 1 << (OG02B1B_LANES - 1) |
              V4L2_MBUS_CSI2_CHANNEL_0 |
              V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
        config->type = V4L2_MBUS_CSI2_DPHY;
        config->flags = val;

        return 0;
}

static const struct dev_pm_ops og02b1b_pm_ops = {
        SET_RUNTIME_PM_OPS(og02b1b_runtime_suspend,
                           og02b1b_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops og02b1b_internal_ops = {
        .open = og02b1b_open,
};
#endif

static const struct v4l2_subdev_core_ops og02b1b_core_ops = {
        .s_power = og02b1b_s_power,
        .ioctl = og02b1b_ioctl,
#ifdef CONFIG_COMPAT
        .compat_ioctl32 = og02b1b_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops og02b1b_video_ops = {
        .s_stream = og02b1b_s_stream,
        .g_frame_interval = og02b1b_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops og02b1b_pad_ops = {
        .enum_mbus_code = og02b1b_enum_mbus_code,
        .enum_frame_size = og02b1b_enum_frame_sizes,
        .enum_frame_interval = og02b1b_enum_frame_interval,
        .get_fmt = og02b1b_get_fmt,
        .set_fmt = og02b1b_set_fmt,
        .get_mbus_config = og02b1b_g_mbus_config,
};

static const struct v4l2_subdev_ops og02b1b_subdev_ops = {
        .core   = &og02b1b_core_ops,
        .video  = &og02b1b_video_ops,
        .pad    = &og02b1b_pad_ops,
};

static int og02b1b_set_ctrl(struct v4l2_ctrl *ctrl)
{
        struct og02b1b *og02b1b = container_of(ctrl->handler,
                                             struct og02b1b, ctrl_handler);
        struct i2c_client *client = og02b1b->client;
        s64 max;
        int ret = 0;

        /* Propagate change of current control to all related controls */
        switch (ctrl->id) {
        case V4L2_CID_VBLANK:
                /* Update max exposure while meeting expected vblanking */
                max = og02b1b->cur_mode->height + ctrl->val - 4;
                __v4l2_ctrl_modify_range(og02b1b->exposure,
                                         og02b1b->exposure->minimum, max,
                                         og02b1b->exposure->step,
                                         og02b1b->exposure->default_value);
                break;
        }

        if (!pm_runtime_get_if_in_use(&client->dev))
                return 0;

        switch (ctrl->id) {
        case V4L2_CID_EXPOSURE:
                og02b1b_write_reg(og02b1b->client, OV9282_AEC_GROUP_UPDATE_ADDRESS,
                                       OG02B1B_REG_VALUE_08BIT, OV9282_AEC_GROUP_UPDATE_START_DATA);

                /* 4 least significant bits of expsoure are fractional part */
                ret = og02b1b_write_reg(og02b1b->client, OG02B1B_REG_EXPOSURE,
                                       OG02B1B_REG_VALUE_24BIT, ctrl->val << 4);

                og02b1b_write_reg(og02b1b->client, OV9282_AEC_GROUP_UPDATE_ADDRESS,
                                       OG02B1B_REG_VALUE_08BIT, OV9282_AEC_GROUP_UPDATE_END_DATA);
                og02b1b_write_reg(og02b1b->client, OV9282_AEC_GROUP_UPDATE_ADDRESS,
                                       OG02B1B_REG_VALUE_08BIT, OV9282_AEC_GROUP_UPDATE_END_LAUNCH);
                break;
        case V4L2_CID_ANALOGUE_GAIN:
                ret = og02b1b_write_reg(og02b1b->client, OG02B1B_REG_GAIN_H,
                                       OG02B1B_REG_VALUE_08BIT,
                                       (ctrl->val >> OG02B1B_GAIN_H_SHIFT) & OG02B1B_GAIN_H_MASK);
                ret |= og02b1b_write_reg(og02b1b->client, OG02B1B_REG_GAIN_L,
                                       OG02B1B_REG_VALUE_08BIT,
                                       ctrl->val & OG02B1B_GAIN_L_MASK);
                break;
        case V4L2_CID_VBLANK:
                ret = og02b1b_write_reg(og02b1b->client, OG02B1B_REG_VTS,
                                       OG02B1B_REG_VALUE_16BIT,
                                       ctrl->val + og02b1b->cur_mode->height);
                break;
        case V4L2_CID_BRIGHTNESS:
                ret = og02b1b_write_reg(og02b1b->client, OG02B1B_AEC_STROBE_REG_H,
                                           OG02B1B_REG_VALUE_08BIT,
                                           (ctrl->val >> 8) & 0xff);
                ret |= og02b1b_write_reg(og02b1b->client, OG02B1B_AEC_STROBE_REG_L,
                                           OG02B1B_REG_VALUE_08BIT,
                                           ctrl->val & 0xff);
                break;
        case V4L2_CID_TEST_PATTERN:
                ret = og02b1b_enable_test_pattern(og02b1b, ctrl->val);
                break;
        default:
                dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
                         __func__, ctrl->id, ctrl->val);
                break;
        }

        pm_runtime_put(&client->dev);

        return ret;
}

static const struct v4l2_ctrl_ops og02b1b_ctrl_ops = {
        .s_ctrl = og02b1b_set_ctrl,
};

static int og02b1b_initialize_controls(struct og02b1b *og02b1b)
{
        const struct og02b1b_mode *mode;
        struct v4l2_ctrl_handler *handler;
        struct v4l2_ctrl *ctrl;
        s64 exposure_max, vblank_def;
        u32 h_blank;
        int ret;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        handler = &og02b1b->ctrl_handler;
        mode = og02b1b->cur_mode;
        ret = v4l2_ctrl_handler_init(handler, 8);
        if (ret)
                return ret;
        handler->lock = &og02b1b->mutex;

        ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
                                      0, 0, link_freq_menu_items);
        if (ctrl)
                ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

        v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
                          0, OG02B1B_PIXEL_RATE, 1, OG02B1B_PIXEL_RATE);

        h_blank = mode->hts_def - mode->width;
        og02b1b->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
                                h_blank, h_blank, 1, h_blank);
        printk("OG02B1B function:%s line:%d   hblank:%x\n",__FUNCTION__,__LINE__,(int)og02b1b->hblank);
        if (og02b1b->hblank)
                og02b1b->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

        vblank_def = mode->vts_def - mode->height;
        og02b1b->vblank = v4l2_ctrl_new_std(handler, &og02b1b_ctrl_ops,
                                V4L2_CID_VBLANK, vblank_def,
                                OG02B1B_VTS_MAX - mode->height,
                                1, vblank_def);

        printk("OG02B1B function:%s line:%d   vblank:%x\n",__FUNCTION__,__LINE__,(int)og02b1b->vblank);
        exposure_max = mode->vts_def - 4;
        og02b1b->exposure = v4l2_ctrl_new_std(handler, &og02b1b_ctrl_ops,
                                V4L2_CID_EXPOSURE, OG02B1B_EXPOSURE_MIN,
                                exposure_max, OG02B1B_EXPOSURE_STEP,
                                mode->exp_def);

        printk("OG02B1B function:%s line:%d   exposure:%x  exposure_max:%d\n",__FUNCTION__,__LINE__,(int)og02b1b->exposure,(int)exposure_max);
        og02b1b->anal_gain = v4l2_ctrl_new_std(handler, &og02b1b_ctrl_ops,
                                V4L2_CID_ANALOGUE_GAIN, OG02B1B_GAIN_MIN,
                                OG02B1B_GAIN_MAX, OG02B1B_GAIN_STEP,
                                OG02B1B_GAIN_DEFAULT);

        printk("OG02B1B function:%s line:%d   anal_gain:%x\n",__FUNCTION__,__LINE__,(int)og02b1b->anal_gain);

        og02b1b->strobe = v4l2_ctrl_new_std(handler, &og02b1b_ctrl_ops,
                                V4L2_CID_BRIGHTNESS, 1,
                                exposure_max/16, 1,
                                0x88);

        printk("OG02B1B function:%s line:%d   strobe:%x\n",__FUNCTION__,__LINE__,(int)og02b1b->strobe);

        og02b1b->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
                                &og02b1b_ctrl_ops, V4L2_CID_TEST_PATTERN,
                                ARRAY_SIZE(og02b1b_test_pattern_menu) - 1,
                                0, 0, og02b1b_test_pattern_menu);

        printk("OG02B1B function:%s line:%d   test_pattern:%x\n",__FUNCTION__,__LINE__,(int)og02b1b->test_pattern);
        if (handler->error) {
                ret = handler->error;
                dev_err(&og02b1b->client->dev,
                        "Failed to init controls(%d)\n", ret);
                goto err_free_handler;
        }

        og02b1b->subdev.ctrl_handler = handler;

        return 0;

err_free_handler:
        v4l2_ctrl_handler_free(handler);

        return ret;
}

static int og02b1b_check_sensor_id(struct og02b1b *og02b1b,
                                  struct i2c_client *client)
{
        struct device *dev = &og02b1b->client->dev;
        u32 id1 = 0, id2 = 0, id3 = 0;
        int ret;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        printk("------------- OG02B1B ----------\n");
        if (og02b1b->is_thunderboot) {
                dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
                return 0;
        }

        ret  = og02b1b_read_reg(client, OG02B1B_REG_CHIP_ID1,OG02B1B_REG_VALUE_08BIT, &id1);
        ret |= og02b1b_read_reg(client, OG02B1B_REG_CHIP_ID2,OG02B1B_REG_VALUE_08BIT, &id2);
        ret |= og02b1b_read_reg(client, OG02B1B_REG_CHIP_ID3,OG02B1B_REG_VALUE_08BIT, &id3);
        if ((id1 != CHIP_ID1) && (id2 != CHIP_ID2) && (id3 != CHIP_ID3))
        {
                dev_err(dev, "Unexpected sensor id1(%02x) id2(%02x) id3(%02x), ret(%d)\n", id1,id2,id3, ret);
                return -ENODEV;
        }

        dev_info(dev, "Detected OG02B1B %02x %02x %02x sensor\n", id1,id2,id3);
        return 0;
}

static int og02b1b_configure_regulators(struct og02b1b *og02b1b)
{
        unsigned int i;

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        for (i = 0; i < OG02B1B_NUM_SUPPLIES; i++)
                og02b1b->supplies[i].supply = og02b1b_supply_names[i];

        return devm_regulator_bulk_get(&og02b1b->client->dev,
                                       OG02B1B_NUM_SUPPLIES,
                                       og02b1b->supplies);
}

static int og02b1b_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
        struct device *dev = &client->dev;
        struct device_node *node = dev->of_node;
        struct og02b1b *og02b1b;
        struct v4l2_subdev *sd;
        char facing[2];
        int ret;

        dev_info(dev, "driver version: %02x.%02x.%02x",
                DRIVER_VERSION >> 16,
                (DRIVER_VERSION & 0xff00) >> 8,
                DRIVER_VERSION & 0x00ff);

        og02b1b = devm_kzalloc(dev, sizeof(*og02b1b), GFP_KERNEL);
        if (!og02b1b)
                return -ENOMEM;

        ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
                                   &og02b1b->module_index);
        ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
                                       &og02b1b->module_facing);
        ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
                                       &og02b1b->module_name);
        ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
                                       &og02b1b->len_name);
        if (ret) {
                dev_err(dev, "could not get module information!\n");
                return -EINVAL;
        }

        og02b1b->client = client;
        og02b1b->cur_mode = &supported_modes[0];
        og02b1b->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

        og02b1b->xvclk = devm_clk_get(dev, "xvclk");
        if (IS_ERR(og02b1b->xvclk)) {
                dev_err(dev, "Failed to get xvclk\n");
                return -EINVAL;
        }

        og02b1b->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
        if (IS_ERR(og02b1b->reset_gpio))
                dev_warn(dev, "Failed to get reset-gpios\n");

        og02b1b->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
        if (IS_ERR(og02b1b->pwdn_gpio))
                dev_warn(dev, "Failed to get pwdn-gpios\n");

        og02b1b->pinctrl = devm_pinctrl_get(dev);
        if (!IS_ERR(og02b1b->pinctrl)) {
                og02b1b->pins_default =
                        pinctrl_lookup_state(og02b1b->pinctrl,
                                             OF_CAMERA_PINCTRL_STATE_DEFAULT);
                if (IS_ERR(og02b1b->pins_default))
                        dev_err(dev, "could not get default pinstate\n");

                og02b1b->pins_sleep =
                        pinctrl_lookup_state(og02b1b->pinctrl,
                                             OF_CAMERA_PINCTRL_STATE_SLEEP);
                if (IS_ERR(og02b1b->pins_sleep))
                        dev_err(dev, "could not get sleep pinstate\n");
        } else {
                dev_err(dev, "no pinctrl\n");
        }

        ret = og02b1b_configure_regulators(og02b1b);
        if (ret) {
                dev_err(dev, "Failed to get power regulators\n");
                return ret;
        }

        mutex_init(&og02b1b->mutex);

        sd = &og02b1b->subdev;
        v4l2_i2c_subdev_init(sd, client, &og02b1b_subdev_ops);
        ret = og02b1b_initialize_controls(og02b1b);
        if (ret)
                goto err_destroy_mutex;

        ret = __og02b1b_power_on(og02b1b);
        if (ret)
                goto err_free_handler;

        ret = og02b1b_check_sensor_id(og02b1b, client);
        if (ret)
                goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
        sd->internal_ops = &og02b1b_internal_ops;
        sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
                     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
        og02b1b->pad.flags = MEDIA_PAD_FL_SOURCE;
        sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
        ret = media_entity_pads_init(&sd->entity, 1, &og02b1b->pad);
        if (ret < 0)
                goto err_power_off;
#endif

        memset(facing, 0, sizeof(facing));
        if (strcmp(og02b1b->module_facing, "back") == 0)
                facing[0] = 'b';
        else
                facing[0] = 'f';

        snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
                 og02b1b->module_index, facing,
                 OG02B1B_NAME, dev_name(sd->dev));
        ret = v4l2_async_register_subdev_sensor_common(sd);
        if (ret) {
                dev_err(dev, "v4l2 async register subdev failed\n");
                goto err_clean_entity;
        }

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        pm_runtime_set_active(dev);
        pm_runtime_enable(dev);
        pm_runtime_idle(dev);

        return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
        media_entity_cleanup(&sd->entity);
#endif
err_power_off:
        __og02b1b_power_off(og02b1b);
err_free_handler:
        v4l2_ctrl_handler_free(&og02b1b->ctrl_handler);
err_destroy_mutex:
        mutex_destroy(&og02b1b->mutex);

        return ret;
}

static int og02b1b_remove(struct i2c_client *client)
{
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct og02b1b *og02b1b = to_og02b1b(sd);

        printk("OG02B1B function:%s line:%d\n",__FUNCTION__,__LINE__);
        v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
        media_entity_cleanup(&sd->entity);
#endif
        v4l2_ctrl_handler_free(&og02b1b->ctrl_handler);
        mutex_destroy(&og02b1b->mutex);

        pm_runtime_disable(&client->dev);
        if (!pm_runtime_status_suspended(&client->dev))
                __og02b1b_power_off(og02b1b);
        pm_runtime_set_suspended(&client->dev);

        return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id og02b1b_of_match[] = {
        { .compatible = "ovti,og02b1b" },
        {},
};
MODULE_DEVICE_TABLE(of, og02b1b_of_match);
#endif

static const struct i2c_device_id og02b1b_match_id[] = {
        { "ovti,og02b1b", 0 },
        { },
};

static struct i2c_driver og02b1b_i2c_driver = {
        .driver = {
                .name = OG02B1B_NAME,
                .pm = &og02b1b_pm_ops,
                .of_match_table = of_match_ptr(og02b1b_of_match),
        },
        .probe          = &og02b1b_probe,
        .remove         = &og02b1b_remove,
        .id_table       = og02b1b_match_id,
};

static int __init sensor_mod_init(void)
{
        return i2c_add_driver(&og02b1b_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
        i2c_del_driver(&og02b1b_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision og02b1b sensor driver");
MODULE_LICENSE("GPL v2");
