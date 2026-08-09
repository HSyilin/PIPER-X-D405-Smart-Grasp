# Manual Diagnostic Scripts

These scripts are intentionally not started by any launch file. They are
manual camera/marker diagnostics and do not command the arm.

## Capture a frame pair

Start the camera and `smart_grasp_detector`, then run:

```bash
source ~/grasp_ws/env.sh
python3 ~/grasp_ws/src/smart_grasp/scripts/grab_frame.py
```

The script waits for one message from each topic and writes:

```text
/tmp/frame_color.png
/tmp/frame_debug.png
```

It subscribes with best-effort QoS to:

```text
/camera/camera/color/image_raw
/smart_grasp/debug_image
```

## Diagnose ArUco markers

After `grab_frame.py` has created `/tmp/frame_color.png`, run:

```bash
python3 ~/grasp_ws/src/smart_grasp/scripts/aruco_diag.py
```

The script reports image brightness, a simple focus metric, and any marker
detected across the OpenCV ArUco dictionaries. It only reads the saved image.
