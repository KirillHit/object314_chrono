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
static const double supp_z_offset = 0.017357;

const double Object314_TrackAssemblySinglePin::m_right_x_offset = 0.01;

const ChVector3d Object314_TrackAssemblySinglePin::m_sprocket_loc(-0.55215, 0, -0.044643);
const ChVector3d Object314_TrackAssemblySinglePin::m_idler_loc(0.5, 0, -0.041643);

const ChVector3d Object314_TrackAssemblySinglePin::m_susp_locs_L[4] = {
    ChVector3d(0.37971, 0, -0.28608), ChVector3d(0.12971, 0, -0.28608), ChVector3d(-0.12029, 0, -0.28608),
    ChVector3d(-0.37029, 0, -0.28608)};
const ChVector3d Object314_TrackAssemblySinglePin::m_susp_locs_R[4] = {
    ChVector3d(m_right_x_offset - 0.37971, 0, -0.28608), ChVector3d(m_right_x_offset - 0.12971, 0, -0.28608),
    ChVector3d(m_right_x_offset + 0.12029, 0, -0.28608), ChVector3d(m_right_x_offset + 0.37029, 0, -0.28608)};

const ChVector3d Object314_TrackAssemblySinglePin::m_supp_locs_L[2] = {ChVector3d(0.11785, 0, 0.017357),
                                                                       ChVector3d(-0.10215, 0, 0.017357)};
const ChVector3d Object314_TrackAssemblySinglePin::m_supp_locs_R[2] = {
    ChVector3d(m_right_x_offset - 0.11785, 0, 0.017357), ChVector3d(m_right_x_offset + 0.10215, 0, 0.017357)};

// -----------------------------------------------------------------------------
// Constructor for the M113 track assembly using single-pin track shoes.
// Create the suspensions, idler, brake, sprocket, and track shoes.
// -----------------------------------------------------------------------------
Object314_TrackAssemblySinglePin::Object314_TrackAssemblySinglePin(VehicleSide side, BrakeType brake_type)
    : ChTrackAssemblySinglePin("", side) {
    size_t num_shoes = 0;
    std::string suspName("Object314_Suspension");
    std::string shoeName("Object314_TrackShoe");
    m_rollers.resize(2);
    switch (side) {
        case LEFT:
            SetName("Object314_TrackAssemblyLeft");
            m_idler = chrono_types::make_shared<Object314_Idler>("Object314_Idler_Left", side);
            m_brake = chrono_types::make_shared<Object314_BrakeShafts>("Object314_BrakeLeft");
            m_sprocket = chrono_types::make_shared<Object314_SprocketSinglePinLeft>();
            num_shoes = 61;
            suspName += "Left_";
            shoeName += "Left_";
            m_rollers[0] = chrono_types::make_shared<Object314_SupportRollerLeft>(0);
            m_rollers[1] = chrono_types::make_shared<Object314_SupportRollerLeft>(1);
            break;
        case RIGHT:
            SetName("Object314_TrackAssemblyRight");
            m_idler = chrono_types::make_shared<Object314_Idler>("Object314_Idler_Right", side);
            m_brake = chrono_types::make_shared<Object314_BrakeShafts>("Object314_BrakeRight");
            m_sprocket = chrono_types::make_shared<Object314_SprocketSinglePinRight>();
            num_shoes = 61;
            suspName += "Right_";
            shoeName += "Right_";
            m_rollers[0] = chrono_types::make_shared<Object314_SupportRollerRight>(0);
            m_rollers[1] = chrono_types::make_shared<Object314_SupportRollerRight>(1);
            break;
    }

    m_suspensions.resize(4);
    m_suspensions[0] = chrono_types::make_shared<Object314_Suspension>(suspName + "0", side, 0, true);
    m_suspensions[1] = chrono_types::make_shared<Object314_Suspension>(suspName + "1", side, 1, true);
    m_suspensions[2] = chrono_types::make_shared<Object314_Suspension>(suspName + "2", side, 2, false);
    m_suspensions[3] = chrono_types::make_shared<Object314_Suspension>(suspName + "3", side, 3, false);

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
