#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChColor.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    // 1. Создаём физический мир
    ChSystemNSC system;
    system.SetGravity(ChVector<>(0, 0, -9.81));

    // 2. Добавляем шасси
    auto chassis = chrono_types::make_shared<ChBodyEasyBox>(2.0, 1.0, 0.5,  // размеры
                                                           500,           // масса
                                                           true,          // визуализация
                                                           true);         // столкновения
    chassis->SetPos(ChVector<>(0, 0, 1.0));
    system.Add(chassis);

    // 3. Добавляем ведущие колёса (по бокам)
    auto wheel_left = chrono_types::make_shared<ChBodyEasyCylinder>(0.3, 0.2, 50, true, true);
    wheel_left->SetPos(ChVector<>(-0.8, 0.6, 0.5));
    wheel_left->SetRot(ChQuaternion<>(Q_from_AngAxis(CH_C_PI_2, VECT_Y)));
    system.Add(wheel_left);

    auto wheel_right = chrono_types::make_shared<ChBodyEasyCylinder>(0.3, 0.2, 50, true, true);
    wheel_right->SetPos(ChVector<>(-0.8, -0.6, 0.5));
    wheel_right->SetRot(ChQuaternion<>(Q_from_AngAxis(CH_C_PI_2, VECT_Y)));
    system.Add(wheel_right);

    // 4. Простейшая плоскость (земля)
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(20.0, 20.0, 0.5,  // размеры
                                                           1000,             // масса
                                                           true,             // визуализация
                                                           true);            // столкновения
    ground->SetBodyFixed(true);
    ground->SetPos(ChVector<>(0,0,-0.25));
    system.Add(ground);

    // 5. Цикл симуляции
    double step = 0.001;
    while(system.GetChTime() < 5.0) {
        system.DoStepDynamics(step);

        // Опционально выводим позицию шасси
        ChVector<> pos = chassis->GetPos();
        std::cout << "Time: " << system.GetChTime()
                  << "  Chassis z: " << pos.z() << std::endl;
    }

    return 0;
}
