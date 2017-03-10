ifeq ($(CONFIG_CUSTOM_KERNEL_ALSPS_SUB),y)
    ccflags-y += -DCUSTOM_KERNEL_ALSPS_SUB
endif
ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
obj-y += hwmsen/
obj-y += batchsen/
obj-y += sensor_dts/
else
obj-n := hwmsen/
obj-n += batchsen/
endif
