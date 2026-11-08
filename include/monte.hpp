//        #include "monte.hpp"
//        monte::mcl.attachChassis(&chassis);
//        monte::mcl.init({initialX, initialY, initialTheta});
//        monte::mcl.startTask();
//        lemlib::Pose p = monte::mcl.getFusedPose();
#pragma once

#include "api.h"           // PROS
#include "lemlib/api.hpp"  // LemLib
#include <array>
#include <vector>
#include <random>
#include <atomic>
#include <cstdint>

namespace monte {

//config
namespace config {

constexpr double ROBOT_WIDTH_IN  = 15.0; //update for robot
constexpr double ROBOT_LENGTH_IN = 15.0; //same here

struct SensorMount {
    double x_offset_in;
    double y_offset_in;
    double heading_offset_deg;
};
//x-offset from tracking point, y-offset from tracking, degree pointing
constexpr SensorMount MOUNT_NORTH{  0.0,  ROBOT_LENGTH_IN / 2.0,   0.0 };
constexpr SensorMount MOUNT_EAST { ROBOT_WIDTH_IN / 2.0,   0.0,    90.0 };
constexpr SensorMount MOUNT_SOUTH{  0.0, -ROBOT_LENGTH_IN / 2.0, 180.0 };
constexpr SensorMount MOUNT_WEST { -ROBOT_WIDTH_IN / 2.0,  0.0,   270.0 };

constexpr double SENSOR_MIN_RANGE_IN = 1.0;
constexpr double SENSOR_MAX_RANGE_IN = 78.0;

constexpr double SENSOR_BEAM_HALF_ANGLE_DEG = 3.0;

//12' x 12' field.
constexpr double FIELD_WIDTH_IN  = 144.0;
constexpr double FIELD_LENGTH_IN = 144.0;


//accounts for interferances in the field, such as towers 
//tune with actual values of dimenstions of game objects.
struct FieldPole {
    double x_in;
    double y_in;
    double diameter_in;
    const char* name;
};
//ehehhhhhhhhhh, scuffed ah estimate for where the goals are and everything
//values in inches, follwing standard x,y,diameter coordinate system
inline const std::array<FieldPole, 13> FIELD_POLES = {{
    //Neutral goals
    {  24.0, 120.0, 6.0, "Short Neutral Goal (near red/north corner)"},
    {  48.0,  96.0, 6.0, "Short Neutral Goal (mid-north)"           },
    {  72.0,  72.0, 6.0, "Tall Neutral Goal (field center)"         },
    {  96.0,  48.0, 6.0, "Short Neutral Goal (mid-south)"           },
    { 120.0,  24.0, 6.0, "Short Neutral Goal (near blue/south corner)"},
    //Alliance goals
    {  24.0,  48.0, 4.0, "Red Alliance Goal 1"                      },
    {  48.0,  24.0, 4.0, "Red Alliance Goal 2"                      },
    {  96.0, 120.0, 4.0, "Blue Alliance Goal 1"                     },
    { 120.0,  96.0, 4.0, "Blue Alliance Goal 2"                     },
    //Toggles, center of each perimeter wall
    {  72.0,   0.0, 3.0, "Toggle (south wall)"                      },
    {  72.0, 144.0, 3.0, "Toggle (north wall)"                      },
    {   0.0,  72.0, 3.0, "Toggle (west wall)"                       },
    { 144.0,  72.0, 3.0, "Toggle (east wall)"                       },
}};

constexpr double MIN_POLE_DETECT_PROB = 0.25;

//absolute death and pain. The arctual particle tuning below.....
constexpr std::size_t NUM_PARTICLES = 200;

//Beam sensor model weights. must sum to about 1.0.
//lwk kind of random ah values, just tune as see fit
constexpr double Z_HIT   = 0.75; // correctly matches expected wall/pole return. SCOREEEEE
constexpr double Z_SHORT = 0.10; // unexpected closer object (cup/pin/robot). oH nO-. Thats why we account for it.
constexpr double Z_MAX   = 0.05; // sensor maxed out / saw nothing, cause it don't got that 20/20 vision like me
constexpr double Z_RAND  = 0.10; // unexplained random noise. shhhh. Got to shake it up every now and then

constexpr double SENSOR_SIGMA_HIT_IN   = 2.0;  // stddev of "good" reading
constexpr double SENSOR_LAMBDA_SHORT   = 0.35; // decay rate of z_short term

//yo lwk, idk what this does. Apparently its good just to include
constexpr double OBJECT_SIZE_NORMALIZER = 120.0; //0-400
constexpr double SIZE_UNCERTAINTY_SCALE = 1.5;

//NOISE
//get the base and add the rate multiplied by the actual motion done
constexpr double ODOM_TRANS_NOISE_BASE_IN     = 0.03;
constexpr double ODOM_TRANS_NOISE_PER_IN      = 0.02;
constexpr double ODOM_ROT_NOISE_BASE_DEG      = 0.15;
constexpr double ODOM_ROT_NOISE_PER_DEG       = 0.02;
constexpr double ODOM_ROT_NOISE_PER_IN        = 0.05; 


//resample variables

constexpr double RESAMPLE_NEFF_RATIO   = 0.5;  // resample when Neff < ratio*N
constexpr double RANDOM_INJECTION_RATIO = 0.02; // fraction of particles randomized every resample
constexpr double KIDNAP_INJECTION_RATIO = 0.35; // used when avg likelihood collapses

//work out the timing so it doesnt explode
constexpr std::uint32_t LOOP_PERIOD_MS = 20;

}


struct Particle {
    double x_in     = 0.0;
    double y_in     = 0.0;
    double theta_deg = 0.0;
    double weight   = 0.0;
};

struct SensorReading {
    bool   valid      = false;
    bool   maxedOut   = false;
    double distance_in = 0.0;
    double confidence  = 0.0;
    double objectSizeNorm = 0.0;
};

struct SensorSuite {
    SensorReading north, east, south, west;
};
//define actual class
class MonteCarloLocalizer {
public:
    MonteCarloLocalizer();
    void attachChassis(lemlib::Chassis* chassis);
    //spread particles around point you know where at.
    void init(const lemlib::Pose& startPose,
              double spreadX_in = 2.0, double spreadY_in = 2.0,
              double spreadTheta_deg = 5.0);

    //resets everything and scatters particles uniformally acrross field
    void forceGlobalRelocalization();

    void startTask();
    void stopTask();
    bool isRunning() const { return running; }

    lemlib::Pose getFusedPose();
    double getEffectiveSampleSize();

    void predict(const lemlib::Pose& currentOdomPose);
    SensorSuite readSensors();
    void updateWeights(const SensorSuite& readings);
    void resample();
    void runLoop();

private:
    std::vector<Particle> particles;
    std::mt19937 rng;

    lemlib::Chassis* chassisPtr = nullptr;
    pros::Mutex poseMutex;
    lemlib::Pose fusedPose{0, 0, 0};
    double lastNeff = 0.0;

    bool hasPrevOdom = false;
    lemlib::Pose prevOdomPose{0, 0, 0};

    std::atomic<bool> running{false};
    pros::task_t taskHandle = nullptr;

    //internals
    void normalizeWeights();
    void computeFusedPose();
    SensorReading readOneSensor(pros::Distance& sensor);
    double likelihoodForSensor(const Particle& p, const config::SensorMount& mount,
                                const SensorReading& reading) const;
    double beamModelProb(double measured_in, double expected_in) const;

    struct RaycastResult {
        double wallDist_in;
        bool   hitPole;
        double poleDist_in;
        double poleDiameter_in;
    };
    RaycastResult raycast(double originX_in, double originY_in, double headingDeg) const;
};

extern MonteCarloLocalizer mcl;
void mclTaskFn(void* param);
}