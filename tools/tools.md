[TOC]

# 工具使用与杂项记录

## `ip route`

使用`ip route`命令给`kvm`虚拟机添加一条默认路由，在`fedora36`上此条命令是临时添加，系统重启后无效。

```bash
ip route add default via 192.168.122.1 dev enp1s0
```

## `nmcli`

使用`nmcli`进行静态网络设置:

```bash
nmcli con mod enp1s0 ipv4.addresses 192.168.122.66/24
# 等效于ip route add default via 192.168.122.1 dev enp1s0 重启系统后也生效。
nmcli con mod enp1s0 ipv4.gateway 192.168.122.1
nmcli con mod enp1s0 ipv4.method manual
nmcli con mod enp1s0 ipv4.dns "192.168.122.1"
nmcli con up enp1s0
```

## 快速编译替换内核

```bash
CONFIG_LOCALVERSION="_caesar_study"
bear make -j32
sudo make modules_install
sudo make install
```

## 单独编译`kvm`模块

```bash
#!/bin/bash
set -e
set -x

build_path=/home/caesar/linux-5.17.5-300.fc36.x86_64/arch/x86/kvm
install_path=/home/caesar/modules/kvm

pushd ${build_path}
make -C /lib/modules/`uname -r`/build M=`pwd` clean
make -C /lib/modules/`uname -r`/build M=`pwd` modules -j32

mkdir ${install_path} -p
rm -rf ${install_path}/*
cp *.ko ${install_path}
popd

sudo modprobe -r kvm_intel
sudo modprobe -r kvm

pushd ${install_path}
sudo modprobe irqbypass
sudo insmod kvm.ko
sudo insmod kvm-intel.ko
popd
```

## 使用`kvmtool`启动最小虚拟机

参考https://blog.csdn.net/caojinhuajy/article/details/119277087:

```bash
wget https://busybox.net/downloads/busybox-1.32.0.tar.bz2
tar -xf busybox-1.32.0.tar.bz2
cd busybox-1.32.0
#配置为以静态链接的方式编译
make menuconfig 
make -j16 && make install
cd _install
mv linuxrc init
mkdir -p etc/init.d
cd etc/init.d
vim rcS

# 在rcS中补充如下内容
#!/bin/sh
mount -t devtmpfs devtmpfs /dev
mkdir -p /dev/pts
mount -vt devpts -o gid=4,mode=620 none /dev/pts

chmod 755 rcS
cd _install
find . | cpio -o --format=newc > root_fs.cpio
./lkvm run -k bzImage -i root_fs.cpio -m 2048
```

##  在`vscode`中调试`kvmtool`配置

`c_cpp_properties.json`文件的内容:

```json
{
  "configurations": [
    {
      "name": "linux-gcc-x64",
      "includePath": [
        "${workspaceFolder}/**"
      ],
      "compilerPath": "gcc",
      "cStandard": "${default}",
      "cppStandard": "${default}",
      "intelliSenseMode": "linux-gcc-x64",
      "compileCommands": "${workspaceFolder}/compile_commands.json",
      "compilerArgs": [
        ""
      ]
    }
  ],
  "version": 4
}
```

`settings.json`文件的内容:

```json
{
	"editor.tabSize": 4,
	"editor.fontSize": 16,
	"search.exclude": {
		"**/.DS_Store":true,
    	"**/.cmd":true,
    	"**/.o":true,
		"**/arm": true,
		"**/mips": true,
		"**/powerpc": true,
		"**/riscv": true
	},
	"files.exclude": {
		"**/.DS_Store":true,
    	"**/.cmd":true,
    	"**/.o":true,
		"**/arm": true,
		"**/mips": true,
		"**/powerpc": true,
		"**/riscv": true
	}
}
```

`tasks.json`文件的内容:

````json
{
	// See https://go.microsoft.com/fwlink/?LinkId=733558
	// for the documentation about the tasks.json format
	"version": "2.0.0",
	"tasks": [
		{
			"label": "clean",
			"type": "shell",
			"command": "make",
			"args": ["clean"]
		},
		{
			"label": "kvmtool-build",
			"command": "bear make -j8",
			"type": "shell"
		}
	]
}
````

`launch.json`文件的内容:

```json
{
	// Use IntelliSense to learn about possible attributes.
	// Hover to view descriptions of existing attributes.
	// For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
	"version": "0.2.0",
	"configurations": [
		{
			"name": "kvmtool-debug",
			"request": "launch",
			"type":	"lldb",
			"program": "${workspaceFolder}/lkvm-static",
			"args": [
				"run",
				"-c",
				"4",
				"-m",
				"6144",
				"-k",
				"${workspaceFolder}/bzImage",
				"-i",
				"${workspaceFolder}/kvmtool-debug.initrd"
			],
			"stopOnEntry": false,
			"env": {},
			"cwd": "${workspaceFolder}",
			"preLaunchTask": "kvmtool-build"
		}
	]
}
```

上述`4`个`json`文件统一放到`.vscode`目录下。

## `Linux kernel`查询时去除无关架构

`settings.json`:

```json
{
  "search.exclude": {
    "**/.DS_Store":true,
    "**/.cmd":true,
    "**/.o":true,
    "**/sound":true,
    "**/tools":true,
    "**/scripts":true,
    "**/arch/arm":true,
    "**/arch/arm64":true,
    "**/arch/riscv":true,
    "**/arch/csky":true,
    "**/arch/mips":true,
    "**/arch/openrisc":true,
    "**/arch/alpha":true,
    "**/arch/arc":true,
    "**/arch/c6x":true,
    "**/arch/h8300":true,
    "**/arch/hexagon":true,
    "**/arch/microblaze":true,
    "**/arch/mn10300":true,
    "**/arch/nds32":true,
    "**/arch/nios2":true,
    "**/arch/parisc":true,
    "**/arch/powerpc":true,
    "**/arch/s390":true,
    "**/arch/sparc":true,
    "**/arch/score":true,
    "**/arch/sh":true,
    "**/arch/um":true,
    "${workspaceFolder}/arch/x86/kvm/svm/**":true,
    "**/arch/unicore32":true,
    "**/arch/xtensa":true
  },
  "files.exclude": {
    "**/.DS_Store":true,
    "**/.cmd":true,
    "**/.o":true,
    "**/sound":true,
    "**/tools":true,
    "**/scripts":true,
    "**/arch/arm":true,
    "**/arch/arm64":true,
    "**/arch/riscv":true,
    "**/arch/csky":true,
    "**/arch/mips":true,
    "**/arch/openrisc":true,
    "**/arch/alpha":true,
    "**/arch/arc":true,
    "**/arch/c6x":true,
    "**/arch/h8300":true,
    "**/arch/hexagon":true,
    "**/arch/microblaze":true,
    "**/arch/mn10300":true,
    "**/arch/nds32":true,
    "**/arch/nios2":true,
    "**/arch/parisc":true,
    "**/arch/powerpc":true,
    "**/arch/s390":true,
    "**/arch/sparc":true,
    "**/arch/score":true,
    "**/arch/sh":true,
    "**/arch/um":true,
    "${workspaceFolder}/arch/x86/kvm/svm/**":true,
    "**/arch/unicore32":true,
    "**/arch/xtensa":true
  }
}
```

## 树莓派`4B`中配置静态`ip`

打开`/etc/dhcpcd.conf`文件，在文件尾部添加如下内容:

```bash
interface wlan0
static ip_address=192.168.0.214/24
static routers=192.168.0.1 192.168.1.1
static domain_name_servers=192.168.0.1 192.168.1.1
```

## `GDB`调试内核以及内核模块

`GDB`设置打印字节数无限制:

```bash
set max-value-size unlimited
```

内核如下配置项要打开:

```bash
Kernel hacking  --->
    [*] Kernel debugging
    Compile-time checks and compiler options  --->
        [*] Compile the kernel with debug info
        [*]   Provide GDB scripts for kernel debugging
```

如下配置项必须关闭，否则会造成打断点失败:

```bash
Processor type and features ---->
    [] Randomize the address of the kernel image (KASLR)
```

编译并替换内核参见[快速编译替换内核](#快速编译替换内核)。

对应虚拟机`xml`文件中增加如下内容:

```xml
<domain type='kvm' xmlns:qemu='http://libvirt.org/schemas/domain/qemu/1.0'>
    ...
	<qemu:commandline>
    	<qemu:arg value='-gdb'/>
    	<qemu:arg value='tcp::4321'/>
    </qemu:commandline>
</domain>
```

- **`host`和`guest`内核源码都必须是同一份源码且路径保持一致，且是经过编译的，在`guest`中的内核源码可以编译后安装到`guest`中。**
- **假设内核源码路径为`/home/caesar/linux-5.17.5-300.fc36.x86_64`**。

### host

编辑`~/.gdbinit`文件添加如下内容:

```bash
add-auto-load-safe-path /home/caesar/linux-5.17.5-300.fc36.x86_64/.gdbinit
```

编辑`/home/caesar/linux-5.17.5-300.fc36.x86_64/.gdbinit`文件添加如下内容:

```bash
add-auto-load-safe-path /home/caesar/linux-5.17.5-300.fc36.x86_64/vmlinux-gdb.py

```

在`/home/caesar/linux-5.17.5-300.fc36.x86_64/`下启动`gdb`:

```bash
gdb vmlinux
target remote localhost:4321
```

对于非内核模块的代码可以直接加断点进行调试。对于内核模块的代码需要首先在`guest`中查询到内核模块加载的地址。比如对于`kvm`模块:

```bash
# caesar @ fedora56 in ~ [18:00:45] C:1
$ sudo cat /sys/module/kvm/sections/.text
0xffffffffa0971000
# caesar @ fedora56 in ~ [18:00:48] 
$ sudo cat /sys/module/kvm/sections/.data
0xffffffffa0a25000
# caesar @ fedora56 in ~ [18:01:04] 
$ sudo cat /sys/module/kvm/sections/.bss 
0xffffffffa0a40dc0
```

`gdb`中加载虚拟机模块的符号表:

```bash
gdb) add-symbol-file /home/caesar/linux-5.17.5-300.fc36.x86_64/arch/x86/kvm/kvm.ko 0xffffffffa0971000 -s .data 0xffffffffa0a25000 -s .bss 0xffffffffa0a40dc0
```

然后在`kvm`对应代码中使用`break`设置断点就可以调试了。

对于`kvm`模块，如果需要用调试的方法来学习，建议将kvm编译进入内核。

关于调试内核以及内核模块的参考资料参见如下:

> https://howardlau.me/programming/debugging-linux-kernel-with-vscode-qemu.html#i-2
>
> https://zhuanlan.zhihu.com/p/105069730
>
> https://howardlau.me/programming/debugging-linux-kernel-modules.html
>
> http://wiki.lustrefs.cn/index.php?title=%E4%BD%BF%E7%94%A8KVM%E8%BE%85%E5%8A%A9%E5%86%85%E6%A0%B8GDB%E8%BF%9B%E8%A1%8C%E5%AE%9E%E6%97%B6%E8%B0%83%E8%AF%95