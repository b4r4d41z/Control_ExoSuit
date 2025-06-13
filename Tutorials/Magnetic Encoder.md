# Using magnetic encoder
1. 打印磁编码器外壳，打印文件位于`3D Model\Magnetic Encoder Shell\Shell_B.stl`
   1. 打印的外壳安装磁编码器与磁铁(φ6*3)，磁铁用`502`胶水固定
   2. `Shell_B.stl`与`飞特舵机st3215`外形一致；
   3. `Shell_A.stl`是为`Lerobot SO100`项目5号舵机专门制作的外壳，与`飞特舵机st3215`外形不一致，不推荐使用。
   4. 螺丝型号：外壳用`m2*12`的pa自攻螺丝固定，舵盘用这个`m3*6`螺丝固定。

2. 设置磁编码器<虚拟舵机>的视频教程
[lerobot so100机械臂<虚拟舵机>设置组装教程](https://www.bilibili.com/video/BV1ccwpebEn8/)

1. **Note**：驱动板电源只能接`5V`供电！接12V会烧！修改舵机ID需要用`FD.EXE`软件，在`Debug Software`中，详细操作详见下图：
![FD_set1.png](../Debug%20Software/FD_set1.png)
![FD_set2.png](../Debug%20Software/FD_set2.png)
