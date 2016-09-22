#
# Makefile for TI's shared transport line discipline
# and its protocol drivers (BT, FM, GPS)
#

obj-y						+=sensor_power.o
obj-$(CONFIG_ZTE_SENSORS_ACCEL_KXTF9)		+=kxtik.o
obj-$(CONFIG_ZTE_SENSORS_ACCEL_KIONIX)		+=kionix_accel.o
obj-$(CONFIG_ZTE_SENSORS_ACCEL_LIS3DH) 		+= lis3dh_acc.o
obj-$(CONFIG_ZTE_SENSORS_GYRO_L3G4200D) 	+= l3g4200d_gyr.o
obj-$(CONFIG_ZTE_SENSORS_GYRO_MPU3050) 		+= mpu3050.o
obj-$(CONFIG_ZTE_SENSORS_CAP_PROX_SX9500) 	+= sx95xx.o sx9500.o
obj-$(CONFIG_ZTE_SENSORS_ATTINY44A)         += attiny44a.o
obj-$(CONFIG_ZTE_SENSORS_COMPASS_AK8963)	+= akm8963.o
obj-$(CONFIG_ZTE_SENSORS_COMPASS_AK09911)	+= akm09911.o
obj-$(CONFIG_INPUT_LIGHTSENSOR_TMD2771) += tmd2771.o
obj-$(CONFIG_INPUT_LIGHTSENSOR_TMD2772) += tmd2772.o
obj-$(CONFIG_INPUT_LIGHTSENSOR_LTR558) += ltr558als.o
obj-$(CONFIG_INPUT_LIGHTSENSOR_SENSORTEK) += stk3x1x_zte.o
obj-$(CONFIG_INPUT_CWSTM32)			+= CwMcuSensor.o
obj-$(CONFIG_HALL_SWITCH)			+= hallSensor.o
#lijiangshuo add ltr559 driver 20141229
obj-$(CONFIG_INPUT_LIGHTSENSOR_LTR559) += ltr559.o
#lijiangshuo add bma254 driver 20150112
obj-$(CONFIG_ZTE_SENSORS_ACCEL_BMA2X2) += bstclass.o
obj-$(CONFIG_ZTE_SENSORS_ACCEL_BMA2X2) += bma2x2.o


