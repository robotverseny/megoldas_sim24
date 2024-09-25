# `megoldas_sim24` package
ROS 2 python package.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```

``` r
git clone https://github.com/robotverseny/megoldas_sim24
```


### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select megoldas_sim24 --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 launch megoldas_sim24 megoldas1.launch.py
```

``` r
ros2 run megoldas_sim24 simple_pursuit.py
```