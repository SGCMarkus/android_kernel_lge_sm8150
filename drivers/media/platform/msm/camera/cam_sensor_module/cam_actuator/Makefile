ccflags-y += -Idrivers/media/platform/msm/camera/cam_cpas/include
ccflags-y += -Idrivers/media/platform/msm/camera/cam_core
ccflags-y += -Idrivers/media/platform/msm/camera/cam_req_mgr
ccflags-y += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_res_mgr
ccflags-y += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_cci
ccflags-y += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_sensor_io
ccflags-y += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_sensor_utils
ccflags-y += -Idrivers/media/platform/msm/camera/cam_smmu/
ccflags-y += -Idrivers/media/platform/msm/camera/cam_utils

obj-$(CONFIG_SPECTRA_CAMERA) += cam_actuator_dev.o cam_actuator_core.o cam_actuator_soc.o
ifeq ($(CONFIG_MACH_LGE),y)
obj-$(CONFIG_SPECTRA_CAMERA) += lgit_onsemi/lgit_onsemi_actuator_utils.o
endif
