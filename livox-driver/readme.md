# LIVOX 驱动
### 适用  
驱动大疆livox  
在mid40和horizon测试  
horizon默认为双回波，若要修改更多配置可以阅读驱动  
官方驱动汇总 [[驱动]](https://livox-wiki-cn.readthedocs.io/zh_CN/latest/data_summary/Livox_data_summary.html)
lib可能会有问题，为了最好的效果请下载源码编译  
更多信息参考wiki
### use
1. install sdk  
   安装，进入sdk目录执行：
   >sudo chmod +x install.sh && ./install.sh
    
   卸载，进入sdk目录执行：
   >sudo chmod +x uninstall.sh && ./uninstall.sh  
2. connect  
   >将电脑设置为静态 IP。电脑的静态 IP 地址应设置为 192.168.1.X(其中,X 为 2~233 之间的 任意数字,并且电脑的静态 IP 地址不可与需要连接的激光探测测距仪的 IP 地址相同) 
### get cloud
1. include
```cmake
add_subdirectory(livox_lidar)

TARGET_LINK_LIBRARIES(test
        livox_lidar
        )
```
```c++
#include "livox.h"
```
2. init
```c++
LdsLidar &read_lidar = LdsLidar::GetInstance();
int code = read_lidar.InitLdsLidar(cmdline_broadcast_code,50000);
/*@param
 * std::vector<std::string> broadcast_code
 * int max_size:the size of cloud
 * @return
 * int code
*/
```
3. get/clear
```c++
read_lidar.getcloud(getcloud);
//@param pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out
//Get a copy directly (not a reference)
read_lidar.clearcloud();
//resample when radar moves
```
### more info    
1. livox wiki [[wiki]](https://livox-wiki-cn.readthedocs.io/zh_CN/latest/index.html)  
2. livox download [[down]](https://www.livoxtech.com/cn/downloads)
3. ./refs
   
### 目前的问题
1. ~~抓点云会丢包~~
2. ~~结构有点混乱~~ 
   ~~不利于后续使用~~
3. IMU暂时没有利用
