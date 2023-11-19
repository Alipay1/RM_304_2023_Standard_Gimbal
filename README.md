# 2023赛季全向轮步兵云台代码

## 鸣谢
本项目使用了其他开源仓库的源代码，这些代码极大促进了本项目的开发进度：
- [RoboMaster-C-Board-INS-Example](https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example)
- [Development-Board-C-Examples](https://github.com/RoboMaster/Development-Board-C-Examples)
- [FreeRTOS](https://github.com/FreeRTOS/FreeRTOS)

## 开发环境：
|软件|版本|
|---|---|
|STM32CubeMX|V6.8.1|
|MDK|V5.38|

## 使用说明：
1.Clone本仓库<br>
2.运行"C_Board_Standard_Robot.ioc"并点击"generate code"<br>
3.运行"点我替换port为FreeRTOS_Ports内容.bat"<br>
4.运行".\MDK-ARM\C_Board_Standard_Robot.uvprojx"编译并烧录至C板<br>

## TODO：
1.添加电赛期间对云台程序改进增加的安全功能。


[repository](https://github.com/Alipay1/RM_304_2023_Standard_Gimbal)
