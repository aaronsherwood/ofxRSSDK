# RealSense SDK addon for openframeworks 0.10.x
## Under Construction

Currently supported:
* RGB Streaming
* Depth Streaming (Raw Depth and Depth as Color)
* Point Cloud

## Cameras

Tested:
* Intel® RealSense™ Depth Cameras D435

Untested:
* Intel® RealSense™ Depth Cameras D415
* Intel® RealSense™ Depth Modules D400, D410, D420, D430
* Intel® RealSense™ Vision Processor D4m
* Intel® RealSense™ Tracking Module (limited support)

## Supported Platforms

Tested:
* Windows 10 (Build 1803 or later)
* Mac OSX (El Capitan 10.11.6 )

Untested:
* Ubuntu 16.04/18.04 LTS (Linux Kernels 4.4, 4.8 ,4.10, 4.13 and 4.15)
* Windows 8.1 *

****hardware frame synchronization is not available for the D400 series

### Dependecies

* Openframeworks release 0.10.x [download page](http://openframeworks.cc/download).
* Openframeworks addon [ofxGuiExtended](https://github.com/maybites/ofxGuiExtended)

### Instructions

Drop this repository into the \<openframeworksfolder>/addons/ folder

#### Xcode
I've included an Xcode example that should be ready to use. But when building a new example:
* Drag the osx folder containing the three .dylib files in libs/lib into the project. Select Copy Items if Needed & Add to Targets. After added it should look like:  
![](https://raw.githubusercontent.com/aaronsherwood/ofxRSSDK/master/images/lookslike.png)
* In Build Phases drag and drop the three .dylib files over from the left side of xCode into Copy Files:
![](https://raw.githubusercontent.com/aaronsherwood/ofxRSSDK/master/images/copyfiles.png)
* Select Editor > Add Build Phase > Add Run Script Build Phase:
![](https://raw.githubusercontent.com/aaronsherwood/ofxRSSDK/master/images/addrunscriptphase.png)

* Find the empty Run Script you just created:
![](https://raw.githubusercontent.com/aaronsherwood/ofxRSSDK/master/images/addscript.png)
* Copy and paste the following script into the box:
`install_name_tool -change @rpath/librealsense2.2.dylib @executable_path/../Frameworks/librealsense2.2.dylib "$TARGET_BUILD_DIR/$PRODUCT_NAME.app/Contents/MacOS/$PRODUCT_NAME";`

#### Visual Studio
* Copy the realsense2.dll & realsense2.lib files found in libs/lib/windows folder to the bin folder for the project
* Click on the project in Visual Studio
* Click Properties (the wrench)
* Under Linker > General > Additional Library Directories > (Edit...) add a new entry by typing in `.\bin`
* Under Input > Additional Dependencies > (Edit...) add a new entry by typing in `realsense2.lib`

##### Old Stuff (might be useful)
Examples have a relative path to the RealSenseSDK 2.19.1 inside the ofxRSSDK/libs folder.

if otherwise, change the following:

* Menu > Project > Properties > C/C++ > General > Additional Include directories > (Edit...) > (RSSKD_Dir)\include
* Menu > Project > Properties > Linker > General > Additional Library directories > (Edit...) > (RSSKD_Dir)\lib\x64

## credits

Aaron Sherwood

this addon is based on https://github.com/tecartlab/ofxRSSDK & https://github.com/SethGibson/ofxRSSDK, however it has been heavily altered on order to make it compatible with the current RSSDK.

contains the relevant libraries/includes from Intel® RealSense™ SDK 2.0 (build 2.20.1) https://github.com/IntelRealSense/librealsense
