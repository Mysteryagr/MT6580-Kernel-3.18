include $(srctree)/drivers/misc/mediatek/Makefile.custom
subdir-ccflags-y += -Wno-error=date-time
#subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT910)	+=  GT910/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336)	+=  ft6336/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336_V2500)	+= ft6336_v2500/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5346I_V3750)    +=  ft5346i_v3750/
obj-$(CONFIG_TOUCHSCREEN_MTK_EKTF2K)	+=  ektf2k/
obj-$(CONFIG_TOUCHSCREEN_MTK_EKTF2K_V2500)	+=  ektf2k_v2500/
obj-$(CONFIG_TOUCHSCREEN_MTK_EKTF2K_V3702)	+=  ektf2k_v3702/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XXTB_HOTKNOT)	+=  GT9XXTB_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_I2C_RMI4)	+=  synaptics_i2c_rmi4/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT928)	+=  GT928/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_SCP)	+=  GT9XX_hotknot_scp/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5X0X)	+=  ft5x0x/
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_S5250)   +=  synaptics_s5250/
obj-$(CONFIG_TOUCHSCREEN_MTK_S2333_V3750)   +=  s2333_v3750/
obj-$(CONFIG_MTK_TGESTURE)	+=  tgesture/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336_P4200)	+=  ft6336_p4200/
obj-$(CONFIG_TOUCHSCREEN_MTK_EKTF2K_P4200)	+=  ektf2k_4200/
