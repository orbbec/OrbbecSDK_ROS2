# 深度 NFOV 和 WFOV 模式配置

对于 Femto Mega 和 Femto Bolt 设备，NFOV 和 WFOV 模式通过在启动文件中配置深度和 IR 的分辨率来实现。

在启动文件中，depth_width、depth_height、ir_width、ir_height 分别表示深度的分辨率和 IR 的分辨率。

IR 的帧率和分辨率必须与深度保持一致。不同模式与分辨率的对应关系如下：

- NFOV unbinned：640 x 576
- NFOV binned：320 x 288
- WFOV unbinned：1024 x 1024
- WFOV binned：512 x 512
