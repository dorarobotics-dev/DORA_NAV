## DORA-NAV 

DORA-NAV 的目标是提供面向软硬件融合场景提供一体化导航整体方案，操作系统侧支持 Linux、开源鸿蒙、OpenEuler，硬件平台兼容 X86、NVIDIA Jetson、昇腾、此芯全系列算力设备。

三维建图新增可选的 [FAST-LIO2 Rust Dora 模块](modules/mapping/fast_lio2_dora/README.md)，
使用 Dora 1.1.0 接收 Livox 点云与 IMU，启动入口为 `apps/mapping_fast_lio2.yml`。



开发者：(qiuji chen)2276606574@qq.com  、 cenruping@vip.qq.com  、 bob

室内导航案例视频：https://www.bilibili.com/video/BV199KJ6bEwj

### 1、硬件、操作系统支持列表

| 序号 |    硬件平台     |   系统    | 使用文档                                               |
| :--: | :-------------: | :-------: | -------------------------------------------          |
|  1   |     X86平台     |   linux   | 参考doc目录下 [INSTALL_x86.md](doc/INSTALL_x86.md)    |
|  2   | Jetson AGX Orin |   linux   | 参考doc目录下 [INSTALL_Orin.md](doc/INSTALL_Orin.md)  |
|  3   |   瑞莎星睿O6    |   linux   | 已支持                                                |
|  4   |   瑞莎星睿O6    | 开源鸿蒙  | 已支持                                                 |
|  5   |      昇腾       | OpenEuler | 参考 doc目录下 [INSTALL_310P.md](doc/INSTALL_310P.md) |
|  6   |     RK3588      | 开源鸿蒙  | 已支持                                                |

