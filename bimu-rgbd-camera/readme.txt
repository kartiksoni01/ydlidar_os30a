1. 当你是新下载此项目时，首先要安装一下ros环境：运行./HD-DM-Linux-SDK-5.0.1.16/DMPreview目录下的脚本
$: sh setup_env.sh 
2. 安装完成后编译ros，在./bimu-rgbd-camera文件夹下编译
$: catkin_make
3.编译完成后，在./bimu-rgbd-camera文件夹下设置环境变量
$: source ./devel/setup.bash  
4. 用launch文件启动
$: roslaunch bimu_rgbd_camera bimu_camera.launch 


注意：
1. 每次新开启一个终端都要先设置一下环境变量（第三步），再启动launch文件
2. 在./bimu-rgbd-camera/src/bimu_rgbd_camera/CMakeLists.txt文件中，第四行为set(x86_flag TRUE)语句，当此变量为TRUE时，为x86环境编译，若要在arm端编译请将此变量改为FALSE
