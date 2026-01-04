### 压缩图像

您可以使用 `image_transport` 通过 `jpeg` 压缩图像。以下是使用示例：

要访问压缩的彩色图像，可以使用以下命令：

```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

此命令将允许您从指定话题接收压缩的彩色图像。
