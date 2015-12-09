http://blog.chinaunix.net/uid-27164517-id-3364985.html

 用C语言实现的简单Web服务器(Linux) 2012-10-05 21:24:27
分类：
原文地址：用C语言实现的简单Web服务器(Linux) 作者：cnlinuxfan
相信大家对Apache都有所听闻，Apache是目前使用最为广泛我Web服务器。大家可以从http://news.netcraft.com/这个网站得到证实。这是腾讯的http://uptime.netcraft.com/up/graph?site=www.qq.com.Apache强大的功能和高效的性能并且开放源代码的这种模式对我很有吸引力，但无赖自己水平有限，无法从Apache庞大的source code里面理清头绪，于是乎，我就冒出了个自己动手写一个小型的简单的Web服务器的想法，希望对这方面也和我一样感兴趣的朋友有所帮助。
我的实验环境为:
OS:Red Hat Enterprise Linux 5
gcc:4.1.2
libc:2.5
editor:Vim
lang:C
阅读该源代码需要以下预备知识：
C语言基础
Linux编程基础
socket编程基础(Linux)
TCP/IP基本原理
HTTP基本原理
关键字(Key Words):
Linux C, Web HTTP Server, Linux Socket.
-----------------------------------------------------------------------------------
下面是Mutu的第一个版本(0.1 Alpha)，实现了WEB 服务器的最基本功能
包括以下源文件:
webserver.c----程序入口
init_socket.h init_socket.c----完成一些WEB服务器的初始化工作
get_time.h get_time.c----获得服务器的时间
http_session.h http_session.c----处理一次HTTP会话
以下是各文件源码：

xxxxxx

所以源文件已经打包至附件。
I)大家可以单独编译,在shell 下输入如下命令即可：
gcc -Wall init_socket.c http_session.c get_time.c webserver.c -o webserver
II)下载附件后，将Mutu_webserver_0.1_Alpha.tar.gz解压到目录下：
tar zxvf Mutu_webserver_0.1_Alpha.tar.gz
直接用附件里面的makefile文件，在shell下输入如下命令：
make
以上两种方法都可以编译生成WEB 服务器。
生成webserver后，直接在shell下输入：
./webserver即可启动WEB服务器。

那么如何访问该服务器呢？
首先你要知道运行服务器主机的IP，在服务器主机上输入如下命令(需要超级用户权限)：
ifconfig 
如果你的是以太网(ethernet),那么会看到这样一行
inet addr:xxx.xxx.xxx broadcast:xxx.xxx.xxx.xxx mask:255.xxx.xxx.xxx
xxx代表数字（000-255），第一个inet addr后面的数字便是你的网卡地址。
如果你是在本机进行测试，那IP地址可以直接用127.0.0.1(回环地址，localhost)

取得服务器的IP后，用你喜欢的一款浏览器便可以访问WEB SERVER的内容了。
方法为：在浏览器的地址栏内输入:
http://xxx.xxx.xxx.xxx:8080
回车，即可（xxx.xxx.xxx.xxx无刚取得的服务器IP地址，8080为预设的端口）。
附件里面提供了几个测试用的页面，大家可以使用，也可以自己写一些HTML页面然后放到WEB服务器的根目录(即webserver文件所在的目录)
	
文件:	Mutu_webserver_0.1_Alpha.tar.gz
大小:	18KB
下载:	下载

注意事项：
  1.如果你改变了服务器的监听端口，则地址栏中也要做相应改变，0-1023的端口需要超级用户才能开启
  2.本程序仅仅提供大家参考和测试，对运行此程序所造成的一切后果请使用者自负。
  3.你可以自由传播和修改该程序。
  4.如果我有时间的话，在下一个版本(0.2 Beta)中会给程序适当添加注释，并修正一些bug.
  5.转接请注明出处，谢谢！

注：这还只是一个Alpha版本，所以我没有给程序写太多的注释，同时该程序也有不少bug，欢迎大家下载使用，同时为诚恳的请求各位把使用过程中遇到的问题告诉我，谢谢！
本人才疏学浅，这是第一次在CU发表文章，谢谢大家都支持，错误之处难免很多，欢迎各位批评指正。