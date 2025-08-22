##### 在普通用户条件下，执行

```shell
sudo -E bash -c "source /opt/ros/humble/setup.bash && rqt_graph"
```

## 🧩 命令结构总览

| 部分                        | 含义                                                         |
| --------------------------- | ------------------------------------------------------------ |
| `sudo`                      | 以 **root 权限** 执行命令                                    |
| `-E`                        | **保留当前用户的静态环境变量**（如 `$DISPLAY`, `$XAUTHORITY`） |
| `bash`                      | 调用一个新的 **Bash shell**                                  |
| `-c`                        | 后面跟的是一条 **字符串形式的命令**，供 bash 执行            |
| `"source ... && rqt_graph"` | Bash shell 要执行的具体命令串                                |

## 🧠 深入本质：`setup.bash` 的作用不仅是设变量，还可能动态地根据环境做设置

ROS 的 `setup.bash` 做的事情远远不止于设置几个变量，它会：

1. **动态拼接环境变量**（如 `PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH`）；
2. 设置 `AMENT_PREFIX_PATH`, `COLCON_PREFIX_PATH`, `CMAKE_PREFIX_PATH` 等；
3. 解析多个工作空间（如 overlay workspace）；
4. 注册插件路径；
5. 设置 Python 的包查找路径（比如 `.egg-link`, `.pth` 文件）；
6. 使用了 `eval` 和 `export` 等 Bash 行为，只在执行时生效。