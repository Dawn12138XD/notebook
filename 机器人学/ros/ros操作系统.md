
## <font size="9">创建PKG</font>


```bash
robot@WP:~$ cd catkin_ws/src/
robot@WP:~/catkin_ws/src$ catkin_create_pkg atr_pkg rospy roscpp std_msgs
Created file atr_pkg/package.xml
Created file atr_pkg/CMakeLists.txt
Created folder atr_pkg/include/atr_pkg
Created folder atr_pkg/src
Successfully created files in /home/robot/catkin_ws/src/atr_pkg. Please adjust the values in package.xml.
robot@WP:~/catkin_ws/src$
```

## 使用节点
![[Pasted image 20261010201256.png]]
roscore 启动ros
robot@WP:~$source ~/catkin_ws/devel/setup.bash
		把当前 catkin 工作空间的 ROS 包环境，加载进当前终端
		`~/catkin_ws/devel/setup.bash`：`catkin_make`编译后自动生成的环境脚本
				这两个命令的作用都是加载 ROS 环境配置，但它们加载的层级不同：

			- `source /opt/ros/noetic/setup.bash`：加载系统安装的 ROS Noetic 环境。
    
			 `source ~/catkin_ws/devel/setup.bash`：加载你自己编译的 ROS 工作空间环境。
    

你可以把它们理解成：第一个让 Ubuntu 认识 ROS，第二个让 ROS 认识你自己编译的项目。
rosrun ssr_pkg chao_node  运行节点--节点即pkg里的一个cpp文件 
## <font size="5">编写节点</font>
![[Pasted image 20261010201536.png]]
## 话题
### topic
![[Pasted image 20261010203821.png|487]]
1. ﻿﻿一个ROS节点网络中，可以同时存在多个话题。
2. ﻿﻿﻿一个话题可以有多个发布者，也可以有多个订阅者。
3. ﻿﻿﻿一个节点可以对多个话题进行订阅，也可以发布多个话题。
4. ﻿﻿﻿不同的传感器消息通常会拥有各自独立话题名称，每个话题只有一个发布者。
5. ﻿﻿﻿机器人速度指令话题通常会有多个发布者，但是同一时间只能  
    有一个发言人。
### message
      消息类型的库 std_msgs 
			      插叙呢https://index.ros.org
[[Pasted image 20261010204636.png]]




![[Pasted image 20261010204610.png|203]]
			      
			      这个是由一对基本类型嵌套而成的复合结构体
	消息是一个队列先进先出，保证新消息在队列里
<font size="9">创建话题，发送消息，控制频率</font>



```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "chao_node"); //ros::init 像是库：：函数
    printf("我的枪去而复返，你的生命有去无回!\n");
    ros::NodeHandle nh;//对象
    ros::Publisher pub = nh.advertise<std_msgs::String>("快上车开黑群",10);
	//创建群聊的方式 advertise
	//topic格式：使用msgs。 string 要引用<std_msgs/String.h>
	ros: :Rate loop_rate(10) ;//设定循环时间
    while(ros::ok())
    {
        printf("我要开始刷屏了！\n");
        std_msgs::String msg;
        msg.data = "国服马超，带飞";
        pub.publish(msg);
        loop_rate.sleep()； //使用阻塞实现循环
    }
    return 0;
}

```
<font size="5">查看正在活跃的topic使用rostopic list</font>
```
```
```bash
robot@WP:~$ rostopic list
/kuai_shang_che_kai_hei_qun
/rosout
/rosout_agg
robot@WP:~$ rostopic echo /kuai_shang_che_kai_hei_qun
```
![[Pasted image 20261010212309.png]]使用echo -e 转译输出的数据 


```bash
	robot@WP:~§ rostopic hz /kuai_shang_che_kai_hei_qun
```
<font size="5">使用rostopic hz 查看发送频率</font>

## <font size="5">订阅话题</font>
