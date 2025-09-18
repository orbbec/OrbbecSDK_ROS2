# Configuration of depth NFOV and WFOV modes

For the Femto Mega and Femto Bolt devices, the NFOV and WFOV modes are implemented by configuring the resolution of
Depth and IR in the launch file.
In launch file, depth_width, depth_height, ir_width, ir_height represents the resolution of the depth and the resolution of
the IR.
The frame fps and resolution of IR must be consistent with the depth. The correspondence between different modes and
resolutions is as follows:

- NFOV unbinned: 640 x 576.
- NFOV binned: 320 x 288.
- WFOV unbinned: 1024 x 1024.
- WFOV binned: 512 x 512.
