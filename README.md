# Object314 Chrono

Симуляционная среда для гусеничного робота «Объект 314», построенная на базе Project Chrono. Робот разработан командой [Polytech Voltage Machine](https://github.com/Polytech-VM-Team).

## Запуск

1. Установите Docker и [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

2. Запуск:

    ```bash
    git clone https://github.com/KirillHit/tracked_vehicle_sim.git
    cd tracked_vehicle_sim
    xhost +local:root
    docker compose up
    ```

# Примеры работы

Работа подвески:

![obstacles](.images/obstacles.gif)

Деформируемый грунт SMC:

![obstacles](.images/smc_terrian.gif)
