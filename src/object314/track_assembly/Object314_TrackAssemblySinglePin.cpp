// Object314 single-pin track assembly subsystem.

#include "Object314_TrackAssemblySinglePin.hpp"
#include "Object314_BrakeShafts.hpp"
#include "Object314_BrakeSimple.hpp"
#include "Object314_Idler.hpp"
#include "Object314_IdlerWheel.hpp"
#include "Object314_RoadWheel.hpp"
#include "Object314_SprocketSinglePin.hpp"
#include "Object314_SupportRoller.hpp"
#include "Object314_Suspension.hpp"
#include "Object314_TrackShoeSinglePin.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
static const double supp_z_offset = 0.02;

const double Object314_TrackAssemblySinglePin::m_right_x_offset = 0.1;
const ChVector3d Object314_TrackAssemblySinglePin::m_sprocket_loc(0, 0, 0);
const ChVector3d Object314_TrackAssemblySinglePin::m_idler_loc(-5.4, 0, -0.0647);
const ChVector3d Object314_TrackAssemblySinglePin::m_susp_locs_L[6] = {
    ChVector3d(-0.8458, 0, -0.3759), ChVector3d(-1.6258, 0, -0.3759), ChVector3d(-2.4058, 0, -0.3759),
    ChVector3d(-3.1858, 0, -0.3759), ChVector3d(-3.9658, 0, -0.3759), ChVector3d(-4.7458, 0, -0.3759)};
const ChVector3d Object314_TrackAssemblySinglePin::m_susp_locs_R[6] = {
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 0.8458, 0, -0.3759),
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 1.6258, 0, -0.3759),
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 2.4058, 0, -0.3759),
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 3.1858, 0, -0.3759),
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 3.9658, 0, -0.3759),
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 4.7458, 0, -0.3759)};
const ChVector3d Object314_TrackAssemblySinglePin::m_supp_locs_L[3] = {ChVector3d(-1.2358, 0, 0.1561 + supp_z_offset),
                                                                       ChVector3d(-2.7958, 0, 0.1561 + supp_z_offset),
                                                                       ChVector3d(-4.3106, 0, 0.1561 + supp_z_offset)};
const ChVector3d Object314_TrackAssemblySinglePin::m_supp_locs_R[3] = {
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 1.2358, 0, 0.1561 + supp_z_offset),
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 2.7958, 0, 0.1561 + supp_z_offset),
    ChVector3d(Object314_TrackAssemblySinglePin::m_right_x_offset - 4.3106, 0, 0.1561 + supp_z_offset)};

// -----------------------------------------------------------------------------
// Constructor for the M113 track assembly using single-pin track shoes.
// Create the suspensions, idler, brake, sprocket, and track shoes.
// -----------------------------------------------------------------------------
Object314_TrackAssemblySinglePin::Object314_TrackAssemblySinglePin(VehicleSide side, BrakeType brake_type)
    : ChTrackAssemblySinglePin("", side) {
    size_t num_shoes = 0;
    std::string suspName("Object314_Suspension");
    std::string shoeName("Object314_TrackShoe");
    m_rollers.resize(3);
    switch (side) {
        case LEFT:
            SetName("Object314_TrackAssemblyLeft");
            m_idler = chrono_types::make_shared<Object314_Idler>("Object314_Idler_Left", side);
            m_brake = chrono_types::make_shared<Object314_BrakeShafts>("Object314_BrakeLeft");
            m_sprocket = chrono_types::make_shared<Object314_SprocketSinglePinLeft>();
            num_shoes = 77;
            suspName += "Left_";
            shoeName += "Left_";
            m_rollers[0] = chrono_types::make_shared<Object314_SupportRollerLeft>(0);
            m_rollers[1] = chrono_types::make_shared<Object314_SupportRollerLeft>(1);
            m_rollers[2] = chrono_types::make_shared<Object314_SupportRollerLeft>(2);
            break;
        case RIGHT:
            SetName("Object314_TrackAssemblyRight");
            m_idler = chrono_types::make_shared<Object314_Idler>("Object314_Idler_Right", side);
            m_brake = chrono_types::make_shared<Object314_BrakeShafts>("Object314_BrakeRight");
            m_sprocket = chrono_types::make_shared<Object314_SprocketSinglePinRight>();
            num_shoes = 78;
            suspName += "Right_";
            shoeName += "Right_";
            m_rollers[0] = chrono_types::make_shared<Object314_SupportRollerRight>(0);
            m_rollers[1] = chrono_types::make_shared<Object314_SupportRollerRight>(1);
            m_rollers[2] = chrono_types::make_shared<Object314_SupportRollerRight>(2);
            break;
    }

    m_suspensions.resize(6);
    m_suspensions[0] = chrono_types::make_shared<Object314_Suspension>(suspName + "0", side, 0, true);
    m_suspensions[1] = chrono_types::make_shared<Object314_Suspension>(suspName + "1", side, 1, true);
    m_suspensions[2] = chrono_types::make_shared<Object314_Suspension>(suspName + "2", side, 2, false);
    m_suspensions[3] = chrono_types::make_shared<Object314_Suspension>(suspName + "3", side, 3, false);
    m_suspensions[4] = chrono_types::make_shared<Object314_Suspension>(suspName + "4", side, 4, true);
    m_suspensions[5] = chrono_types::make_shared<Object314_Suspension>(suspName + "5", side, 5, true);

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(chrono_types::make_shared<Object314_TrackShoeSinglePin>(shoeName + std::to_string(it)));
    }
}

// -----------------------------------------------------------------------------
const ChVector3d Object314_TrackAssemblySinglePin::GetSprocketLocation() const {
    return m_sprocket_loc;
}

const ChVector3d Object314_TrackAssemblySinglePin::GetIdlerLocation() const {
    return m_idler_loc;
}

const ChVector3d Object314_TrackAssemblySinglePin::GetRoadWhelAssemblyLocation(int which) const {
    return (m_side == LEFT) ? m_susp_locs_L[which] : m_susp_locs_R[which];
}

const ChVector3d Object314_TrackAssemblySinglePin::GetRollerLocation(int which) const {
    return (m_side == LEFT) ? m_supp_locs_L[which] : m_supp_locs_R[which];
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
