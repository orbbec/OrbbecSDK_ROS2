### Compressed Image

You can use `image_transport` to compress the image using `jpeg`. Below is an example of how to use it:

To access the compressed color image, you can use the following command:

```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

This command will allow you to receive the compressed color image from the specified topic.
