# MoveIt Main Workspace

这是 MoveIt 2 main 分支的代码，相比 humble 分支，包含了更多的功能和修复。

> 本文档参考了下面的 MoveIt 2 官方文档  
> <https://moveit.ai/install-moveit2/source/>  
> <https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html>

## 构建 MoveIt 工作空间

### 删除所有 apt 安装的 moveit 相关包

```bash
source /opt/ros/humble/setup.bash
sudo apt purge "ros-$ROS_DISTRO-moveit*"
```

### 克隆代码
```bash
# 以在 ~/Documents/ros2/moveit2_main_ws 构建工作区为例，你也可以放到其他地方
mkdir -p ~/Documents/ros2
cd ~/Documents/ros2

git clone --recurse-submodules <本仓库的 url>

cd moveit2_main_ws
```

配置 git pull 时自动更新子模块

```bash
git config submodule.recurse true
```

如果你已经克隆了项目但忘记了 `--recurse-submodules`，那么可以运行：

```bash
git submodule update --init --recursive
```

### 修改代码

忽略不需要的包

```bash
# 这个 test 包会导致 source 时报错
touch src/moveit2/moveit_ros/tests/COLCON_IGNORE
```

### 安装依赖

如果还没有安装 rosdep，请先安装 rosdep：
```bash
sudo apt install python3-rosdep
sudo rosdep init
```

更新系统：
```bash
rosdep update
sudo apt update
sudo apt dist-upgrade
```

```bash
rosdep install -r --from-paths . --ignore-src -y
```

### 安装 colcon mixin

```bash
sudo apt install python3-colcon-common-extensions python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

### 编译

```bash
colcon build --symlink-install --mixin release compile-commands
```

注意：编译之后如果将工作空间移动到了其他位置，需要删除 build install 文件夹，再次编译

## source 工作空间

为了使用编译后的工作空间，需要先 source 系统 ROS 2，再 source 本工作空间。

为简化流程，我们提供了一个脚本 [ros2_rc](ros2_rc)。你可以将该脚本放在喜欢的目录（如 ~/.local），然后在 ~/.bashrc 中添加：

```bash
if [ -f ~/.local/ros2_rc ]; then
    source ~/.local/ros2_rc
fi
```

### 注意事项

- 如果你之前在 ~/.bashrc 或类似的地方有自动 source 系统 ROS 2 的代码，请将其删除。ros2_rc 脚本会处理对系统 ROS 2 的 source。

- 如果你使用 zsh，请对 ~/.zshrc 也做类似的修改。

- 默认情况下， ros2_rc 不会自动 source 系统 ROS 2 和 MoveIt 2 workspace。这可以防止 ROS 2 对 PYTHONPATH 等环境变量的污染，从而避免一些奇怪的问题。如需使用 ROS 2，请在每个新终端手动执行以下命令：
    ```bash
    rr
    ```
    之后 main 分支的 MoveIt 2 就好像已经安装在系统中一样，可以直接使用。

    > 如果你希望 ros2_rc 自动 source 系统 ROS 2 和 MoveIt 2 workspace，可以取消 `ros2_rc` 脚本末尾注释掉的 `rr`。

- ros2_rc 还提供了 `rs` 命令，可以快速完成下面的几件事：
    1. `rr`
    2. 自动检测当前终端类型，source install/setup.bash 或 install/setup.zsh

    因此，如果你想 source 当前工作空间，可以直接运行：

    ```bash
    # 无需执行 rr
    rs
    ```
