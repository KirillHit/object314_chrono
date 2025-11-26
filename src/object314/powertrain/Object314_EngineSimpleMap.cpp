// Simple engine model for the Object314 vehicle based on torque-speed engine maps

#include "Object314_EngineSimpleMap.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

const double rpm2rads = CH_PI / 30;

Object314_EngineSimpleMap::Object314_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double Object314_EngineSimpleMap::GetMaxEngineSpeed() {
    return 2400 * rpm2rads;
}

void Object314_EngineSimpleMap::SetEngineTorqueMaps(ChFunctionInterp& map0, ChFunctionInterp& mapF) {
    map0.AddPoint(-10.472, 0.000);
    map0.AddPoint(83.776, -20.0);
    map0.AddPoint(104.720, -20.0);
    map0.AddPoint(125.664, -30.0);
    map0.AddPoint(146.608, -30.0);
    map0.AddPoint(167.552, -30.0);
    map0.AddPoint(188.496, -40.0);
    map0.AddPoint(209.440, -50.0);
    map0.AddPoint(230.383, -70.0);
    map0.AddPoint(251.327, -100.0);
    map0.AddPoint(282.743, -800.0);

    mapF.AddPoint(-100 * rpm2rads, 600);
    mapF.AddPoint(702 * rpm2rads, 700);
    mapF.AddPoint(1099 * rpm2rads, 2044);
    mapF.AddPoint(1158 * rpm2rads, 2149);
    mapF.AddPoint(1207 * rpm2rads, 2250);
    mapF.AddPoint(1233 * rpm2rads, 2350);
    mapF.AddPoint(1263 * rpm2rads, 2450);
    mapF.AddPoint(1300 * rpm2rads, 2545);
    mapF.AddPoint(1352 * rpm2rads, 2628);
    mapF.AddPoint(1403 * rpm2rads, 2683);
    mapF.AddPoint(1499 * rpm2rads, 2702);
    mapF.AddPoint(1628 * rpm2rads, 2683);
    mapF.AddPoint(1757 * rpm2rads, 2650);
    mapF.AddPoint(1901 * rpm2rads, 2569);
    mapF.AddPoint(2004 * rpm2rads, 2472);
    mapF.AddPoint(2099 * rpm2rads, 2386);
    mapF.AddPoint(2195 * rpm2rads, 2298);
    mapF.AddPoint(2323 * rpm2rads, 2154);
    mapF.AddPoint(2450 * rpm2rads, -1000.0);  // fading out of engine torque
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
