## 应用场景

相机模组离地高度是4.5cm

## 制作apriltag pdf 文件

* 使用 `tagSlam` 的 `make_tag.py` ( `make_tag_modify.py`的区别是不再画出board外面的矩形）

* 或者 `kalibr`的 `kalibr_create_target_pdf`脚本；

### make_tag_modify.py使用说明

```
sudo apt install python-pyx

./make_tag_modify.py --nx 1 --ny 1 --marginx 0.00 --marginy 0.00 --tsize 0.06 --tspace 0.0 --startid 4 --tfam t36h11 --borderbits 1 

nx: 列；
ny: 行；
marginx: x方向、白边的宽度，单位米；
marginy: y方向、白边的宽度，单位米；
tsize: 黑色正方形边长，单位米；
tspace: tag之间的间隔 占tsize的比例；
startid: 起始的tag id；
tfam: tag family
borderbits: 黑色边界的宽度，单位是bit，bit是t36h11里面6*6个grids的每个grid的边长；


This will produce a pdf file target.pdf of a single tag with id 4, size 16cm, and a 1 bit wide border. When you generate tags, you obviously want to print them as big as possible, but you must leave a large white frame around the tag! Make it about 2 bits wide (a bit corresponds to one of the small squares that make up the tag’s code). The white border around the tag is crucial for detection!

The borderbits parameter determines the size of the black border on the outside of the AprilTag. You will generally want to use a 1 bit wide border, although you can see that the calibration board in the image has tags with 2 bit borders (the corner squares between the tags are for more accurate localization of the tag corners, TagSLAM is not using them).
```

![](/media/clh/06F81D2EF81D1D8D/a_cleanRobotSensors/robotSlam/apriltag/apriltag_about.png)

### 选择tag

* 假设tag的所在空间尺寸 10cm x 10cm

  * 36h11: 6x6、8x8、10x10，如下图示例tagSize为8cm;

  * 16h5 : 4x4 6x6 8x8，一个grid 1.25cm，如下图示例tagSize为7.5cm

* 假设tag所在空间尺寸是5cm x 5cm，

  * 一个grid为 5/8=0.625cm，于是tagSize为 6x0.625=3.75cm

### 命令

```
# 生成单个的tag
./make_tag_modify.py --nx 1 --ny 1 --marginx 0.00 --marginy 0.00 --tsize 0.06 --tspace 0.0 --startid 4 --tfam t36h11 --borderbits 1

# 生成 tag grid

```

### 参考链接

https://github.com/AprilRobotics/apriltag

https://github.com/ethz-asl/kalibr/wiki/calibration-targets

https://berndpfrommer.github.io/tagslam_web/making_tags/

https://github.com/berndpfrommer/tagslam_root

## run apriltag_demo

```
依靠apriltag example，在apriltag_demo 中调用 pose estimator

# ./apriltag_demo -h
Usage: ./apriltag_demo [options] <input files>
  -h | --help          [ true ]       Show this help   
  -d | --debug         [ false ]      Enable debugging output (slow)   
  -q | --quiet         [ false ]      Reduce output   
  -f | --family        [ tag36h11 ]   Tag family to use   
  -i | --iters         [ 1 ]          Repeat processing on input set this many times   
  -t | --threads       [ 1 ]          Use this many CPU threads   
  -a | --hamming       [ 1 ]          Detect tags with up to this many bit errors.   
  -x | --decimate      [ 2.0 ]        Decimate input image by this factor   
  -b | --blur          [ 0.0 ]        Apply low-pass blur to input; negative sharpens   
  -0 | --refine-edges  [ true ]       Spend more time trying to align edges of tags 
```

## 时间统计

光鉴rgb模组内参:

```
rgb parameters:
640 480 422.445 422.836 326.453 278.685
rgb_distortion paprs:
-0.10141 0.0324961 0.000480091 -0.000270821 -0.0247792
```

### 36h11

```
# ./apriltag_demo /userdata/1620977221.911542.jpg 
loading /userdata/1620977221.911542.jpg
detection   0: id (36x11)-0   , hamming 0, margin   99.945
 0                             init        0.014000 ms        0.014000 ms
 1                         decimate        1.484000 ms        1.498000 ms
 2                       blur/sharp        0.003000 ms        1.501000 ms
 3                        threshold        2.109000 ms        3.610000 ms
 4                        unionfind        6.742000 ms       10.352000 ms
 5                    make clusters       14.770000 ms       25.122000 ms
 6            fit quads to clusters       24.148000 ms       49.270000 ms
 7                            quads        0.408000 ms       49.678000 ms
 8                decode+refinement        3.151000 ms       52.829000 ms
 9                        reconcile        0.018000 ms       52.847000 ms
10                     debug output        0.001000 ms       52.848000 ms
11                          cleanup        0.013000 ms       52.861000 ms
hamm     1     0     0     0     0     0     0     0     0     0       52.847    17
Summary
hamm     1     0     0     0     0     0     0     0     0     0       52.847    17

============================================================

# ./apriltag_demo /userdata/1620987718.525104.jpg 
loading /userdata/1620987718.525104.jpg
detection   0: id (36x11)-0   , hamming 0, margin   91.774
cal pose start time:148035703
euler angles:2.67411 -2.8082 -1.19584, t:0.0194705 -0.109928 0.315983, Use time(ms):6
current time:148035709
 0                             init        0.023000 ms        0.023000 ms
 1                         decimate        1.258000 ms        1.281000 ms
 2                       blur/sharp        0.002000 ms        1.283000 ms
 3                        threshold        1.947000 ms        3.230000 ms
 4                        unionfind        7.473000 ms       10.703000 ms
 5                    make clusters       15.759000 ms       26.462000 ms
 6            fit quads to clusters       24.410000 ms       50.872000 ms
 7                            quads        0.450000 ms       51.322000 ms
 8                decode+refinement        3.181000 ms       54.503000 ms
 9                        reconcile        0.036000 ms       54.539000 ms
10                     debug output        0.001000 ms       54.540000 ms
11                          cleanup        0.026000 ms       54.566000 ms
hamm     1     0     0     0     0     0     0     0     0     0       54.543    16
Summary
hamm     1     0     0     0     0     0     0     0     0     0       54.543    16
```

### t16h5

```
# ./apriltag_demo -f tag16h5 /userdata/1622174836.167575.jpg 
loading /userdata/1622174836.167575.jpg
detection   0: id (16x 5)-1   , hamming 0, margin   93.777
cal pose start time:6560298
	* tagSize:0.0375
	* fx:422.445
	* fy:422.836
	* cx:326.453
	* cy:278.685
euler angles:0.88663 -2.06487 0.333566, t:-0.138968 0.00715716 0.504124, Use time(ms):11
current time:6560309
 0                             init        0.031000 ms        0.031000 ms
 1                         decimate        1.373000 ms        1.404000 ms
 2                       blur/sharp        0.002000 ms        1.406000 ms
 3                        threshold        2.226000 ms        3.632000 ms
 4                        unionfind        8.310000 ms       11.942000 ms
 5                    make clusters       17.287000 ms       29.229000 ms
 6            fit quads to clusters       18.097000 ms       47.326000 ms
 7                            quads        0.393000 ms       47.719000 ms
 8                decode+refinement        3.355000 ms       51.074000 ms
 9                        reconcile        0.034000 ms       51.108000 ms
10                     debug output        0.002000 ms       51.110000 ms
11                          cleanup        0.083000 ms       51.193000 ms
hamm     1     0     0     0     0     0     0     0     0     0       51.162    19
Summary
hamm     1     0     0     0     0     0     0     0     0     0       51.162    19
```

## TODO

* 采集数据
* 自动化跑完数据集、获得统计数据

## kalibr 提供的脚本和参数说明

#### 安装`ethz-asl/kalibr`

https://github.com/ethz-asl/kalibr/wiki/calibration-targets

#### 依赖项

```
部分适配于16.04
sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-dev libboost-all-dev doxygen libopencv-dev ros-kinetic-vision-opencv ros-kinetic-image-transport-plugins ros-kinetic-cmake-modules python-software-properties software-properties-common libpoco-dev python-matplotlib python-scipy python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev python-catkin-tools libv4l-dev
```

* `python-catkin-tools`：

  E: Unable to locate package python-catkin-tools

* sudo pip install python-igraph --upgrade　命令有错误输出，改用下面这个:

  https://stackoverflow.com/questions/28435418/failing-to-install-python-igraph

  sudo apt-get install python-igraph

#### 编译

```
cd ~
mkdir kalibr_ws
cd kalibr_ws
mkdir src
cd src
git clone https://github.com/ethz-asl/Kalibr.git
cd ..
catkin_make -j4
```

`kalibr_create_target_pdf`需要 `python-pyx`

`sudo apt-get install python-pyx` (可能下载慢)

#### 使用

`./kalibr_create_target_pdf --h`

```
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.088           #size of apriltag, edge to edge [m]
tagSpacing: 0.3          #ratio of space between tags to tagSize
                         #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]
```

