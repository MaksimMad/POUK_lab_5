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

```ros2 launch simple_map map.launch.xml```

2-ой терминал
```source /opt/ros/foxy/setup.bash```

```rviz2```

В открывшейся программе rviz необходимо выставить `fixed_frame` в значение `odom`, добавить через меню окна `Displays` `Add`-> `By Topic` топики с данными дальномера `/base_scan` и карты '/simple_map'. Или через меню `File`->`Open config` открыть map.rviz из папки проекта 