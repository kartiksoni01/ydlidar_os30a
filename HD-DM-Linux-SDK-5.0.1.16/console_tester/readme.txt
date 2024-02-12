1. How to build console tool:

sh build.sh (For x86-64, select '1'. For NVIDIA TX2 and NVIDIA NANO, select '2'. For Rockchip, select '3' )

2. How to run console tool:

(1) For x86-64:
sh run_x86.sh

(2) For NVIDIA TX2 and NVIDIA NANO:
sh run_arm64_tx2.sh

(3) For Rockchip PX30:
sh run_arm64_px30.sh

(4) For MTK:

sh run_arm64_mtk.sh

3. There are the macros in main.cpp to enable/disable the demo:


(1) _ENABLE_INTERACTIVE_UI_ : Configure the camera device through step-by-step instructions. By default, it is enabled.
(2) _ENABLE_PROFILE_UI_ : Profile the frame rate of camera device. By default, it is disabled.
(3) _ENABLE_FILESAVING_DEMO_UI_ : Save rgb files of color stream and yuyv files of depth stream. By default, it is disabled.
(4) _ENABLE_FILESAVING_DEMO_POINT_CLOUD_UI_: Run the point cloud demo. By default, it is disabled.

4. For MIPI Camera SDK, please change the permission of the sysfs nodes if running the console_tester (eSPDI APIs) with the non-root:

chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/asic_data
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/asic_id
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/erro_code
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/fs_data
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/fs_id
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/fw_version
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/mipi_out_en
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/read_asic_reg
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/read_ct_pu_value
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/read_fw_reg
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/read_i2_reg
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/stream_on
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/video_mode
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/video_modes
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/write_asic_reg
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/write_ct_pu_value
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/write_fw_reg
chmod 777 /sys/bus/i2c/devices/[i2c_bus]-[i2c_slave_address]/write_i2_reg

Could type the following command to get the value of '[i2c_bus]-[i2c_slave_address]':

cat /proc/esp876_i2c_info ; echo

For ubuntu 18.04 of TX2 platform, the '[i2c_bus]-[i2c_slave_address]' will be 30-0060:

sudo su
chmod 777 /sys/bus/i2c/devices/30-0060/asic_data
chmod 777 /sys/bus/i2c/devices/30-0060/asic_id
chmod 777 /sys/bus/i2c/devices/30-0060/erro_code
chmod 777 /sys/bus/i2c/devices/30-0060/fs_data
chmod 777 /sys/bus/i2c/devices/30-0060/fs_id
chmod 777 /sys/bus/i2c/devices/30-0060/fw_version
chmod 777 /sys/bus/i2c/devices/30-0060/mipi_out_en
chmod 777 /sys/bus/i2c/devices/30-0060/read_asic_reg
chmod 777 /sys/bus/i2c/devices/30-0060/read_ct_pu_value
chmod 777 /sys/bus/i2c/devices/30-0060/read_fw_reg
chmod 777 /sys/bus/i2c/devices/30-0060/read_i2_reg
chmod 777 /sys/bus/i2c/devices/30-0060/stream_on
chmod 777 /sys/bus/i2c/devices/30-0060/video_mode
chmod 777 /sys/bus/i2c/devices/30-0060/video_modes
chmod 777 /sys/bus/i2c/devices/30-0060/write_asic_reg
chmod 777 /sys/bus/i2c/devices/30-0060/write_ct_pu_value
chmod 777 /sys/bus/i2c/devices/30-0060/write_fw_reg
chmod 777 /sys/bus/i2c/devices/30-0060/write_i2_reg

5. Decoding the saving files website after you doing 'Snapshot'

https://rawpixels.net/
