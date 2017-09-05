# drive

/×使用说明

一、serila_comm
  1.这是一个ros下串口通信包
  2.要使用需要依赖serial包可如下安装
    a| sudo apt-get install ros-indigo-serial；
    b| https://github.com/wjwwood/serial   下载源码到当前工作空间一起编译源码；

二、image_sport
  1.这是ros下opencv图像和ros 消息互换例成包括发布和接收；

三、usb_cam 
  1.这是ros下网络摄像头驱动，底层是V4L2 可根据自己摄像头修改底层驱动参数；


git init          //初始化
touch README
git add .
git commit -m ‘v1.0‘
git remote add origin https://github.com/yaotianfa/drive.git     //连接远程github项目
git push -u origin master
