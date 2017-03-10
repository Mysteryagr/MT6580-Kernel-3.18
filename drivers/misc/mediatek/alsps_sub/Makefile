include $(srctree)/drivers/misc/mediatek/Makefile.custom
# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
obj-y   +=  alsps_sub.o
obj-y   +=  alsps_factory_sub.o
obj-y   +=  aal_control_sub.o

obj-$(CONFIG_MTK_LTR553_SUB)	+=  ltr553_sub/
