修改了cmakelist ，使其适配了交叉编译。学习若格网关的记录

## 1.安装配置交叉编译工具

```
1.下载指定的交叉工具链。
https://pan.baidu.com/s/1a8FAd4AqiFsFfqxY7XmZNw

2.我这里是解压到/home/lkt/Soft_packet/目录下，配置环境变量 这个是临时的配置，重启端口就无效了。
export PATH=/home/lkt/Soft_packet/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin:$PATH

3.因为我们的系统是64位的，而交叉工具是32位的，需要安装以下
apt-get install aptitude  //这个先不试
apt-get install lib32z1
apt-get install lib32ncurses5
apt-get install lib32stdc++6

4.然后执行
lkt@lkt-VirtualBox:~/gitlab/HelloWorld_ARM$ arm-linux-gnueabihf-gcc -v
5.可以查看到版本，表示工具链正常了。
Using built-in specs.
COLLECT_GCC=arm-linux-gnueabihf-gcc
COLLECT_LTO_WRAPPER=/home/lkt/Soft_packet/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/../libexec/gcc/arm-linux-gnueabihf/4.9.2/lto-wrapper
Target: arm-linux-gnueabihf
Configured with: /cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/.build/src/gcc-linaro-4.9-2014.09/configure --build=i686-build_pc-linux-gnu --host=i686-build_pc-linux-gnu --target=arm-linux-gnueabihf --prefix=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/install --with-sysroot=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/install/arm-linux-gnueabihf/libc --enable-languages=c,c++,fortran --disable-multilib --enable-multiarch --with-arch=armv7-a --with-tune=cortex-a9 --with-fpu=vfpv3-d16 --with-float=hard --with-pkgversion='crosstool-NG linaro-1.13.1-4.9-2014.09 - Linaro GCC 4.9-2014.09' --with-bugurl=https://bugs.launchpad.net/gcc-linaro --enable-__cxa_atexit --enable-libmudflap --enable-libgomp --enable-libssp --with-gmp=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/.build/arm-linux-gnueabihf/build/static --with-mpfr=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/.build/arm-linux-gnueabihf/build/static --with-mpc=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/.build/arm-linux-gnueabihf/build/static --with-isl=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/.build/arm-linux-gnueabihf/build/static --with-cloog=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/.build/arm-linux-gnueabihf/build/static --with-libelf=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/.build/arm-linux-gnueabihf/build/static --enable-threads=posix --disable-libstdcxx-pch --enable-linker-build-id --enable-plugin --enable-gold --with-local-prefix=/cbuild/slaves/oorts/crosstool-ng/builds/arm-linux-gnueabihf-linux/install/arm-linux-gnueabihf/libc --enable-c99 --enable-long-long --with-mode=thumb --disable-multilib --with-float=hard
Thread model: posix
gcc version 4.9.2 20140904 (prerelease) (crosstool-NG linaro-1.13.1-4.9-2014.09 - Linaro GCC 4.9-2014.09) 
lkt@lkt-VirtualBox:~/gitlab/HelloWorld_ARM$ arm-linux-gnueabihf-gcc main.cpp -0 -n
arm-linux-gnueabihf-gcc: error: unrecognized command line option ‘-0’


6.编译一个cpp
lkt@lkt-VirtualBox:~/gitlab/HelloWorld_ARM$ arm-linux-gnueabihf-gcc 111.cpp -o eeee
lkt@lkt-VirtualBox:~/gitlab/HelloWorld_ARM$ ./eeee 
bash: ./eeee：无法执行二进制文件: 可执行文件格式错误
因为是32位的程序，64位的虚拟机无法运行。到此验证交叉编译工具完成。
```

## 2.cmake+交叉编译

```
#判断是不是交叉编译环境
if($ENV{XPL_OBJECT_TYPE} STREQUAL ARM) 
	# 指定交叉编译的目标系统，此处为arm-linux-gnueabihf，依情形设置
	SET(CMAKE_SYSTEM_NAME arm-linux-gnueabihf)
	SET(TOOL_ADD "/home/lkt/Soft_packet/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux")
	set(triple arm-linux-gnueabihf)
	# 指定C编译器
	set(CMAKE_C_COMPILER "${TOOL_ADD}/bin/arm-linux-gnueabihf-gcc")
	set(CMAKE_C_COMPILER_TARGET ${triple})

	# 指定C++编译器
	set(CMAKE_CXX_COMPILER "${TOOL_ADD}/bin/arm-linux-gnueabihf-g++")
	set(CMAKE_CXX_COMPILER_TARGET ${triple})	
endif( )


区别在于 XPL_OBJECT_TYPE是否被设置为了ARM
export PATH=/home/lkt/Soft_packet/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin:$PATH

执行source arm_env_linux.sh
就可以了
```



arm_env_linux.sh

```
export XPL_OBJECT_TYPE="ARM"
export XPL_PROJECT_DIR=$(pwd)
export XPL_FEATURE_DIR=$XPL_PROJECT_DIR/feature
export XPL_PROTOCOL_DIR=$XPL_PROJECT_DIR/protocol
export XPL_THIRDPARTY_DIR=$XPL_PROJECT_DIR/thirdparty
export CCACHE_PATH=/usr/bin:$CCACHE_PATH
export CC="gcc"
export AR="gcc-ar"
export CXX="g++"
export CPP="gcc"
export AS="as"
export GDB=gdb
export STRIP=strip
export RANLIB=ranlib
export OBJCOPY=objcopy
export OBJDUMP=objdump
export PATH=/home/lkt/Soft_packet/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin:$PATH

# . /opt/fsl-qoriq/2.0/environment-setup-aarch64-fsl-linux



```

## VSCODE远程调试arm

### 参考这个教程再虚拟机安装了NFS的服务器

```
https://blog.csdn.net/yangxueyangxue/article/details/122318745
1.安装 nfs-kernel-server
apt-get install nfs-kernel-server

2.我在/home/lkt 下创建一个名为 nfs 的文件夹
mkdir nfs

3.打开配置文件添加 nfs 共享目录
vi /etc/exports
在配置文件里面添加一下代码：
/home/lkt/nfs/ *(rw,sync,no_root_squash)
共享的目录
rw:是可读写权限
sync:是资料同步写入内存和硬盘
no_root_squash:当登录 NFS 主机使用共享目录的使用者是 root 时， 其权限将被转换成为一名使用者，通常它的 UID 与 GID 都会变成 nobody 身份。
4.重启 nfs 服务
/etc/init.d/nfs-kernel-server restart

然后在板子里面输入
mount -t nfs -o nolock,nfsvers=3 192.168.2.9:/home/lkt/nfs /mnt/
这样板子里的/mnt/目录就和虚拟机里面的/home/lkt/nfs/目录共享了。
```

看这教程需要我编译一个gdbserver (就是各种报错，最后也没解决)

```
https://www.freesion.com/article/3763162732/
gdbserver

本地虚拟机的gdb版本是9.2，所以我也下了一个9.2的源码
参考这个教程编译的：https://blog.csdn.net/weixin_45829566/article/details/108925998
 wget https://ftp.gnu.org/gnu/gdb/gdb-9.2.tar.gz
 解压
 tar -zxvf gdb-9.2.tar.gz 

编译gdb的时候我报错了。
cd gdb-9.2/
mkdir build
./configure --target=arm-linux
/home/lkt/Soft_packet/gdb-9.2/configure --target=arm-none-linux-gnueabi --prefix=/home/lkt/Soft_packet/gdb-7.8.1/build_install
make -j16

出现错误：
checking for library containing waddstr... no
configure: WARNING: no enhanced curses library found; disabling TUI
checking for library containing tgetent... no
configure: error: no termcap library found
Makefile:8370: recipe for target 'configure-gdb' failed
make[1]: *** [configure-gdb] Error 1
make[1]: Leaving directory '/home/sam/Work/gdb/gdb-7.5'
Makefile:844: recipe for target 'all' failed
make: *** [all] Error 2

安装libncureses5-dev：
sudo apt-get install libncurses5-dev
再次
make -j16


gdb-7.5$ sudo apt-get install libncurses5-dev
mkdir tmp
make install prefix=$pwd/tmp
把arm-linux-gdb复制到/bin目录
调试 /bin/arm-linux-gdb ./test

试一下编译，gdbserver
cd gdb/gdbserver/
./configure --host=arm-linux --prefix=/home/lkt/Soft_packet/gdb-9.2/gdb/gdbserver/build
make -j16
cp gdbserver /work/nfs_root/fs_mini
新的教程
https://www.bilibili.com/read/cv8134380/
./configure --target=arm-none-linux-gnueabi --prefix=/home/lkt/Soft_packet/gdb-7.8.1/build_install

gdbserver
./configure --target=arm-linux --host=arm-linux --prefix=/home/lkt/Soft_packet/gdb-7.8.1/build_install
make -j16 
make CC=/home/lkt/Soft_packet/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-gcc -j16
报错
gdb_proc_service.h:25:26: fatal error: proc_service.h: 没有那个文件或目录


export PATH=/LIB32:$PATH
```

### 使用若格提供的gdb和gdbserver实现对盒子程序的debug

```
1.将压缩包中的gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux\arm-linux-gnueabihf\debug-root\usr\bin\gdbserver 放到盒子中，与可执行文件同一个目录。
如下
root@linkorTech:/mnt# ls 
gdbserver      helloworld     myeasylog.log

执行指令，gdb的服务器就启动了
root@linkorTech:/mnt# ./gdbserver 192.168.2.15:2222 ./helloworld 
Process ./helloworld created; pid = 504
Listening on port 2222

将交叉编译gdb需要用到的动态库.tar中的4个共享库放到/lib32下
libncurses.so.5  libncurses.so.5.9  libtinfo.so.5  libtinfo.so.5.9
这4个库是从若格提供的虚拟机镜像/lib32中复制出来的。

vscode中修改lanch.json的配置
"miDebuggerPath": "/home/lkt/Soft_packet/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-gdb",
"miDebuggerServerAddress": "192.168.2.15:2222"
直接按F5 就可以开启编译了。
结合第一点的NFS，就可以实现方便的嵌入式的在线程序测试了。
```

