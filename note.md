# 笔记

SYSCFG_DL_INIT

![image-20260506222718490](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506222718490.png)

# GPIO

![image-20260506204050756](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506204050756.png)

![image-20260506204228357](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506204228357.png)

一个组内可以有多个pin

set高电平 clear低电平

![image-20260506223219867](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506223219867.png)

 输出模式 高驱动 高速 5V开漏输出

![image-20260506224418224](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506224418224.png)

![image-20260506230120162](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506230120162.png)

**返回值并不是单纯的0或1  因为读取的是组Pins**

比如读取PB21 返回

![image-20260506230551454](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506230551454.png)

即第二十一位上是1 每一个引脚对应一个pin

![image-20260506230137910](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506230137910.png)



![image-20260506230201304](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506230201304.png)

![image-20260506231000173](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506231000173.png)

![image-20260506231112746](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506231112746.png)

Delay多少个运行周期 实际上是个宏定义套壳



# Timer

![image-20260507142039181](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507142039181.png)

![image-20260507143508069](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507143508069.png)

![image-20260507143516972](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507143516972.png)

TimA 高级定时器

TimG 通用定时器

TimX  A&G

七个定时器 最多支持22个PWMchannel

TIMA 的独特能力：

- **互补 PWM 输出 + 死区插入**：典型用途是三相桥臂驱动（防止上下管直通）
- **Fault Handler**：检测到故障信号时可以立即强制关断 PWM，硬件级保护
- **Phase Load**：可以让两个定时器产生有相位差的 PWM，用于交错拓扑
- **Repeat Counter**：抑制不必要的中断，比如只在每 N 个 PWM 周期才触发一次中断

各实例用途解释：

- **TIMG0**：最普通的定时器，做 Systick、延时、普通 PWM 都行
- **TIMG6/7**：在 PD1 域，用 MCLK 作为 BUSCLK（最高80MHz），支持 Shadow，适合需要在低功耗模式下保持运行的场景
- **TIMG8**：专门用来读**正交编码器**（QEI），接电机编码器 A/B 相信号直接硬件计数
- **TIMG12**：32-bit 计数器，计时范围大，适合精确测量长时间间隔或高精度输入捕获

 ![image-20260507141113925](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507141113925.png)

![image-20260507142159959](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507142159959.png)

时钟来源都用BUSCLK就行

![image-20260507142352493](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507142352493.png)

![image-20260507143057895](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507143057895.png)

加法定时器和减法定时器

![image-20260507143200340](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507143200340.png)

中断

![image-20260507143247428](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507143247428.png)

数到0的时候进中断 （示例为减法定时器）

![image-20260507143659804](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507143659804.png)

一个定时器的所有中断都共同使用一个服务函数

所以需要加上switchcase来判断



**低功耗的处理** 通过模式的切换  运行 睡眠 停止 空闲 中断       

![image-20260507143829245](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507143829245.png)

![image-20260507144542337](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507144542337.png)

从中断返回到主函数的时候 cpu去到sleep模式 不进入run模式

然后进入中断后 再从sleep模式进到run模式去执行相关操作

![image-20260507144649491](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507144649491.png)

__WFI wait for interupt 让程序进入到sleep模式



**但是这个没啥用 也可以不使用 一直在run模式下运行**



# PWM

 用定时器输出PWM波 大部分配置是通用的

配置频率（周期）

![image-20260507162933613](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507162933613.png)

![image-20260507162915515](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507162915515.png)

Edge_aligned（边沿对齐）和centre_aligned（中心对齐）

![image-20260507163124348](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507163124348.png)

两个波形 左边是对齐的

两路 PWM 的**上升沿都对齐在左边**（Period 起始处），只有下降沿位置不同（取决于各自 CCR 值）。所以叫"边沿对齐"。





![image-20260507163638799](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507163638799.png) 

中轴线对齐

**关键特征**：两路 PWM 的**高电平脉冲都以周期中心对称**，占空比越大脉冲越宽，但始终居中。



同一个定时器输出的不同PWM波频率都是一样的 占空比可以单独设置

![image-20260507165355514](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507165355514.png)  

counter compare valve指的是周期内低电平的时间   和占空比相关和periodcount相关

![image-20260507170035601](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507170035601.png)

配置中写的占空比实际上是初始值

修改占空比实际上就说修改countercomparrevalue

![image-20260507170356836](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507170356836.png)

使用的所谓TimG/TimA 相关函数实际上都是宏定义来自于TimX

![](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507172205315.png)

![image-20260507172452986](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507172452986.png)

![image-20260507172600196](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507172600196.png)



# IIC 

![image-20260506204337071](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506204337071.png)

![image-20260506204356540](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506204356540.png)

**注意速率不要太高**



# 串口

![image-20260506204444255](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506204444255.png)

![image-20260506204539889](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506204539889.png)

![image-20260506204559487](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506204559487.png)

![image-20260506204629762](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260506204629762.png)

串口的中断可以在配置文件中找到 

而GPIO的中断是一组GPIO或多组GPIO共享一组中断 无法在配置文件中找到

用任何中断都需要在主函数里面开启

# ADC

![image-20260505181816126](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260505181816126.png)

12bit 精度4096

2¹² = 4096

格子越多精度越高 最终值是输出电压 量程可选

![image-20260505183146260](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260505183146260.png)

可以选择外部参考电压还是内部参考电压

内部参考电压默认1.4V

可以在VREF中修改 有两档1.4V和2.5V

ADC用内部源比较好 外部可能出现电压不稳等情况

![image-20260505183224031](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260505183224031.png)

换ADC引脚时需要换通道 不同通道对应不同引脚

ADC的中断和串口的类似

![image-20260505183430067](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260505183430067.png)

![image-20260507215206358](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507215206358.png)

single单一转换 

sequence序列 若需要测量多个通道的值选这个

![image-20260507215838713](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507215838713.png)

中断

![image-20260507220031401](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507220031401.png)

![image-20260507215933001](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507215933001.png)

Memory0事件触发中断

ADC开启测量 测量后存在Memory0中 触发中断

![image-20260507220107731](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507220107731.png)

 ![image-20260507222023746](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507222023746.png)

![image-20260507222109426](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507222109426.png)

手动模式可以设置采样时间 一般是125us

![image-20260507222529429](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507222529429.png)

这个函数不仅仅会读取最高优先级的中断而且会清楚对应标志位 所以写判断是必要的

![image-20260507222656680](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507222656680.png)

遵守类似这个格式

![image-20260507222733711](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507222733711.png)

一个标志位 一个存数值

![image-20260507222823637](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507222823637.png)

![image-20260507222905845](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507222905845.png)

以上是single的版本

接下来是多路

![image-20260507223134259](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507223134259.png)

![image-20260507223227156](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507223227156.png)

此处为程序断电

![image-20260507223335765](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507223335765.png)

![image-20260507223348212](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507223348212.png)

最大的区别

![image-20260507223407752](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507223407752.png)

![image-20260507223415635](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507223415635.png)

四路 0-3

注意中断只选择了Memory3而不是把0123全部都选上 是因为测量是按顺序的从0开始到3 到3时本次测量结束 

只需要使用末尾的中断即可

![image-20260507223733626](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507223733626.png)

启动只需要写一次 因为这个启动是针对于ADCx 的 即使多路也不需要多次启动

![image-20260507224041949](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507224041949.png)

![image-20260507224051934](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507224051934.png)

### 为什么要重新 `enableConversions`？

#### 原因：ADC 在序列扫描完成后会**自动禁用自身**

MSPM0 的 ADC12 在 Sequence 模式下，完成一轮完整扫描（MEM0→MEM3）后，硬件会**自动清除 enable 位**，防止立刻自动开始下一轮。

所以在while循环的末尾一定要enable

# DAC

DAC（数字模拟转换器，Digital-to-Analog Converter）的作用是**将数字信号转换为模拟信号**。

一句话总结：

|      | **ADC**                     | **DAC**                     |
| ---- | --------------------------- | --------------------------- |
| 全称 | Analog-to-Digital Converter | Digital-to-Analog Converter |
| 方向 | 模拟 → 数字                 | 数字 → 模拟                 |
| 作用 | **读取**外部世界            | **输出**到外部世界          |

### 数据流方向



```
外部传感器 → [ADC] → MCU 处理 → [DAC] → 外部执行器
  (模拟量)           (数字量)            (模拟量)
```

### 典型使用场景对比

**ADC 用于"感知"**

- 读取电位器、电压、电流
- 采集 IMU 的模拟输出
- 你之前 MSPM0 项目里的 ADC 采样显示到 OLED，就是典型用法

**DAC 用于"控制/输出"**

- 输出参考电压控制电机驱动
- 生成扫描波形
- 音频播放

### 本质都是"量化"问题

两者都涉及**分辨率**（几个 bit）和**参考电压**（Vref）：



```
ADC：输入电压 / Vref × 2^N  → 数字值
DAC：数字值  / 2^N × Vref  → 输出电压
```

![image-20260507224607420](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507224607420.png)

参考电压和输出电压

![image-20260507224641702](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507224641702.png)

![image-20260507224807428](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507224807428.png)

![image-20260507224826742](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507224826742.png)

12位 0到40995 8位 0到255

![image-20260507225401602](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507225401602.png)

默认输出要记得两个都要on

![image-20260507224959740](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507224959740.png)

![image-20260507225141210](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507225141210.png)

**要先使能**

用内部源比较好VREF

![image-20260507225429446](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260507225429446.png)