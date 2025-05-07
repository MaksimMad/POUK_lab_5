## Установка ros1_bridge

```sudo apt-get install ros-foxy-ros1-bridge```


## Сборка
из корневой папки:

```source /opt/ros/noetic/setup.bash```

```source /opt/ros/foxy/setup.bash```

```colcon build```

## Запуск

1-ый терминал

```source /opt/ros/noetic/setup.bash```

```source /opt/ros/foxy/setup.bash```

```source install/setup.bash```

```ros2 launch control_selector selector.launch.xml```

2-ой терминал

```source /opt/ros/noetic/setup.bash```

```source /opt/ros/foxy/setup.bash```

```source install/setup.bash```

```ros2 run switcher switcher_node```