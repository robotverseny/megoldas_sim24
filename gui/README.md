# Robot Control Panel — GUI

Dash-based web GUI for controlling the Wheeltec robot and tuning solution parameters live via ROS 2 parameters.

**Start:** `bash setup_and_run.sh`  
**Open:** http://localhost:8050

---

## Megoldás 1 — Simple Pursuit parameters

### Speed Scale  (`velocity` ROS param)

**Formula:**
```
cmd_vel.linear.x = lidar_velocity * 0.5 * VELOCITY
```

`lidar_velocity` is computed dynamically from LiDAR data: it is proportional to how far ahead the path is clear (roughly 0–1, normalized over ~5 m). `VELOCITY` (the slider) scales that entire curve.

| Slider value | Max possible speed |
|---|---|
| 1.0 (default) | ~0.5 m/s |
| 2.0 | ~1.0 m/s |
| 3.0 | ~1.5 m/s |

The car **automatically slows down** when an obstacle is close — the slider just sets the ceiling.  
Setting it to `0` stops forward motion entirely regardless of what the LiDAR sees.

---

### Wheelbase Scale  (`wheelbase` ROS param)

**Formula (Pure Pursuit steering):**
```
steering_angle = atan2(2 * WHEELBASE * sin(alpha) / lookahead_distance, 1)
```

`WHEELBASE` is the physical distance between the front and rear axles, measured at **0.3187 m**.  
The slider is a **scale factor** around that measured value:

```
ROS value sent = slider × 0.3187 m
```

| Slider value | Actual wheelbase sent |
|---|---|
| 0.5× | 0.159 m |
| 1.0× (default) | 0.319 m  ← calibrated |
| 1.5× | 0.478 m |
| 2.0× | 0.637 m |

**Effect of changing it:**
- **Higher than 1.0×** → algorithm thinks the car is longer → produces **gentler, wider turns**
- **Lower than 1.0×** → algorithm thinks the car is shorter → produces **sharper, tighter turns**

Use this to compensate if the car consistently over- or under-steers on curves:
- Car cuts corners too sharply → increase scale
- Car takes corners too wide → decrease scale

---

## Megoldás 2 — Follow the Gap parameters

### Max Throttle  (`max_throttle` ROS param)

Direct constant speed sent to `cmd_vel.linear.x`. Unlike Simple Pursuit, Follow the Gap does **not** dynamically adjust speed based on LiDAR — it always drives at this fixed value.

| Value | Effect |
|---|---|
| 0.5 (default) | Moderate safe speed |
| 1.0+ | Faster, less reaction time for steering |

---

### Safety Radius  (`safety_radius` ROS param)

LiDAR readings below this distance are treated as **blocked** (set to 0) before the gap search runs. Only ranges **beyond** this radius are considered as driveable gaps.

- **Larger value** → more conservative, rejects nearby readings → car stays further from walls
- **Smaller value** → more aggressive, allows tighter gaps → may clip obstacles

---

### Steering Sensitivity  (`steering_sensitivity` ROS param)

**Formula:**
```
cmd_vel.angular.z = best_angle * steering_sensitivity
```

`best_angle` is the angle (radians) to the midpoint of the largest detected gap.

- **Higher value** → stronger steering response for the same gap angle → more reactive but can oscillate
- **Lower value** → smoother, more damped steering → may be slow to react to sharp turns

---

## Notes

- **Stop order:** always stop the solution first, then the driver
- **Parameter changes** take effect immediately if the solution is running; if not running, values are saved and applied automatically as soon as the node finishes starting (polled every 0.4 s, up to 10 s timeout)
- **Wheelbase** is the only parameter where the slider shows a scale factor — the actual meter value sent to ROS is shown in the log
