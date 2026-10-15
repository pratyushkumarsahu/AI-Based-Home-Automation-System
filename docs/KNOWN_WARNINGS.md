# Known Harmless System Warnings & Diagnostics

This document cataloges non-critical, benign warnings emitted by third-party simulation tools (Gazebo Sim Harmonic / Mesa / Ogre) running in virtualized or headless Linux environments. None of these affect system functionality, security logic, or test determinism.

---

## 1. Virtual Machine Software Rendering Warnings

### `VMware: No 3D enabled (0, Success)`
- **Origin:** Mesa 3D OpenGL driver (`vmwgfx` / `swrast`) inside virtual machine environments.
- **Root Cause:** Gazebo Sim Ogre2 render engine queries the host hypervisor for hardware 3D graphics acceleration. In headless or virtualized Linux VMs without GPU passthrough, Mesa engages CPU software rendering.
- **Impact:** None. Headless rendering proceeds successfully; camera images are correctly generated and rendered in software.

### `libEGL warning: egl: failed to create dri2 screen`
- **Origin:** EGL display manager during headless initialization (`-s` mode).
- **Root Cause:** Gazebo attempts to connect to a native X11/Wayland DRI2 display device before falling back to EGL surfaceless / pbuffer rendering.
- **Impact:** None. The fallback to software rasterization succeeds automatically.

---

## 2. SDFormat Parser Frame ID Diagnostic

### `XML Element[gz_frame_id], child of element[sensor], not defined in SDF`
- **Origin:** Gazebo Sim SDFormat parser (`gz-sim` / `sdformat`).
- **Root Cause:** In ROS-Gazebo bridge configurations, `<gz_frame_id>` is specified inside URDF sensor blocks to define optical link frames for camera and LiDAR. SDFormat standard treats this as an extension property and copies it as an element child.
- **Impact:** None. The frame ID is recognized and successfully bridged to ROS 2 TF frames (`camera_link_optical`, `laser_link`).
