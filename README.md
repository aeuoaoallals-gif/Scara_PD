Создание репозитория

'mkdir -p ~/ros2_ws/src'
'cd ~/ros2_ws/src'

'git clone https://github.com/aeuoaoallals-gif/Scara_PD.git'


Сборка и запуск.

1 Терминал

'git clone https://github.com/aeuoaoallals-gif/Scara_PD.git'
'cd ~/ros2_ws'
'rosdep update'
'rosdep install -i --from-path src --rosdistro humble -y'

'colcon build --packages-select scara_sim'

'ros2 launch scara_sim scara_sim.launch.py'



2 Терминал

source install/setup.bash

ros2 launch scara_sim effort_publisher.launch.py





