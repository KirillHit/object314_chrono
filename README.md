# Object314 Chrono

Симуляционная среда для гусеничного робота «Объект 314», построенная на базе
[Project Chrono](https://projectchrono.org/). Робот разработан командой
[Polytech Voltage Machine](https://github.com/Polytech-VM-Team).

Репозиторий содержит C++17-модель гусеничной платформы, исполняемые сценарии
для ручного управления и ROS2-симулятор для запуска NMPC-контроллера.

## Что реализовано

- Модель гусеничной машины Object314 на базе `chrono::vehicle::ChTrackedVehicle`.
- Шасси с mesh-визуализацией и набором OBJ-ассетов в `src/object314/data`.
- Две однопальцевые гусеничные сборки: ведущие звездочки, ленивцы, опорные и
  поддерживающие катки, подвеска и траки.
- Управление напрямую моментами на левой и правой гусенице через
  `Object314::ApplyTrackTorques`.
- Сцена Chrono/Irrlicht для интерактивной визуализации.
- Вариант симуляции на жестком грунте и SCM-грунте с параметрами почвы,
  просадкой и bulldozing-эффектом.
- ROS2-узел `object314_ros_sim`, который предоставляет plant-интерфейс для
  внешнего контроллера через сервисы `/vehicle/reset` и `/vehicle/step`.
- Интеграция с `third_party/vehicle_nmpc`: Python NMPC-контроллер запускается
  в отдельном контейнере и управляет Chrono-симулятором через ROS2.
- Docker-окружение на базе Chrono 10 и ROS2 Jazzy, собирающее C++-модель,
  ROS2-интерфейсы и пользовательские исполняемые файлы.

## Сценарии запуска

Перед запуском установите Docker, Docker Compose и
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
Для графического окна Irrlicht на Linux также нужен доступ контейнера к X11:

```bash
xhost +local:root
```

### 1. Teleop

Ручной сценарий запускает `object314_teleop`: Chrono-сцену с визуализацией и
клавиатурным управлением моментами на гусеницах.

```bash
docker compose -f docker-compose.teleop.yaml up --build
```

Управление в окне симуляции:

- `Q` / `A` - увеличить / уменьшить момент на левой гусенице.
- `E` / `D` - увеличить / уменьшить момент на правой гусенице.
- `W` / `S` - увеличить / уменьшить момент на обеих гусеницах.
- `Space` - сбросить оба момента в ноль.

По умолчанию шаг изменения момента равен `1 Н м`, диапазон ручного управления:
от `-100 Н м` до `100 Н м` на каждую гусеницу.

### 2. NMPC

NMPC-сценарий запускает два контейнера:

- `object314_ros_sim` - C++ ROS2-симулятор Object314 в Chrono.
- `vehicle_nmpc` - Python NMPC-контроллер из `third_party/vehicle_nmpc`.

```bash
docker compose -f docker-compose.nmpc.yaml up --build
```

