## Install and Start

1. Download pkg

```bash
mkdir -p some_ws/src
cd some_ws/src
git clone https://github.com/aeuoaoallals-gif/Scara_PD.git
```

2. Build

```bash
cd ..
colcon build --symlink-install && source install/local_setup.bash
```

3. Launch simulation

```bash
ros2 launch scara_sim scara_sim.launch.py
```

4. Run effort commands. In the new terminal:

```bash
cd some_ws
source install/local_setup.bash
ros2 launch scara_sim effort_publisher.launch.py
```



# Scara_PD

**Scara_PD** — это ROS 2 пакет для симуляции и управления двухзвенным роботом типа SCARA (Selective Compliance Assembly Robot Arm). Проект развертывается в среде **Ignition Gazebo (Fortress)** и использует современный стек `ros2_control` для точного моделирования физики и взаимодействия с суставами робота.

Главная особенность проекта — реализация **аналитической обратной кинематики и обратной динамики (IK/ID)**, а также применение **ПД-регулятора (Proportional-Derivative controller)** для управления усилиями (effort control). Это позволяет отрабатывать алгоритмы управления роботом в условиях, максимально приближенных к реальному железу, с учетом инерционных параметров и динамики звеньев.

### 🛠 Ключевые возможности
- **Параметризованная URDF-модель:** Звенья робота описаны через Xacro с заданными массо-инерционными характеристиками и коллизиями.
- **Пользовательские ROS-ноды:** Включает генератор траекторий (`trajectory_generator`), узел ПД-регулятора (`pd_controller`) и издатель управляющих усилий (`effort_publisher`).
- **Визуализация и анализ:** Ноды `trajectory_visualizer` и `real_trajectory_visualizer` отображают запланированные и фактические траектории в RViz, а `data_logger` записывает все телеметрические данные в CSV-файл для последующего анализа.
- **Автоматизированный запуск:** Единый launch-файл поднимает симулятор Gazebo, RViz, контроллеры и все вспомогательные ноды, а также автоматически настраивает камеру для удобного наблюдения.

Этот репозиторий отлично подходит для изучения основ робототехники, тестирования алгоритмов управления манипуляторами и отладки связки ROS 2 + Gazebo + ros2_control.
