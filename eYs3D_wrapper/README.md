## Overview **eYs3D® wrapper SDK** is a cross-platform library for eYs3D® eCapture™ depth cameras
### 0. Download the project

Download v1.x.y.tar.gz in the release asset which includes all required submodules. <br>
https://github.com/eYs3D/eys3d_wrapper_prebuilt_linux/releases

If developers tend to use git to download the source code from this repository.

```
git clone git@github.com:eYs3D/eys3d_wrapper_prebuilt_linux.git --recurse-submodules
```

If developer want seperately download the source code and required submodules.
```
git clone git@github.com:eYs3D/eys3d_wrapper_prebuilt_linux.git
git submodule update --init --recursive
```

### 1. Build general wrapper linux (Ubuntu 16.04 or above)

x86_64
```
sh build.sh
```

aarch64
```
sh build_NVIDIA.sh
```
### 2. Run sample codes

Demo callback APIs. When users want to register a function callback for each stream or even an empty function.
```
$ sh run_callback.sh
```

Demo pipeline APIs. When a streaming frame is ready, insert to the working queue for user to get the frame.
```
$ sh run_pipeline.sh
```

Demo FramesetPipeline APIs. When color, depth, and point cloud are totally available insert to working queue
for user to retrieve the frame set. Its performance strongly depends on your platform.
```
$ sh run_frameset_pipeline.sh
```
