#include "monte.hpp"
#include "globals.h"
#include "util.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>
#include <utility>

extern lemlib::Chassis chassis;
using namespace meth;
namespace monte {

MonteCarloLocalizer mcl;

MonteCarloLocalizer::MonteCarloLocalizer() {
    std::random_device rd;
    rng.seed(rd());
    particles.resize(config::NUM_PARTICLES);
    chassisPtr = &chassis;
}

void MonteCarloLocalizer::attachChassis(lemlib::Chassis* chassis) {
    chassisPtr = chassis;
}

void MonteCarloLocalizer::init(const lemlib::Pose& startPose,
                                double spreadX_in, double spreadY_in,
                                double spreadTheta_deg) {
    std::normal_distribution<double> nx(0.0, spreadX_in);
    std::normal_distribution<double> ny(0.0, spreadY_in);
    std::normal_distribution<double> nt(0.0, spreadTheta_deg);

    for (auto& p : particles) {
        p.x_in = std::clamp(startPose.x + nx(rng), 0.0, config::FIELD_WIDTH_IN);
        p.y_in = std::clamp(startPose.y + ny(rng), 0.0, config::FIELD_LENGTH_IN);
        p.theta_deg = wrap360(startPose.theta + nt(rng));
        p.weight = 1.0 / particles.size();
    }

    hasPrevOdom = false;
    fusedPose = startPose;
}

void MonteCarloLocalizer::forceGlobalRelocalization() {
    std::uniform_real_distribution<double> ux(0.0, config::FIELD_WIDTH_IN);
    std::uniform_real_distribution<double> uy(0.0, config::FIELD_LENGTH_IN);
    std::uniform_real_distribution<double> ut(0.0, 360.0);

    for (auto& p : particles) {
        p.x_in = ux(rng);
        p.y_in = uy(rng);
        p.theta_deg = ut(rng);
        p.weight = 1.0 / particles.size();
    }
    hasPrevOdom = false;
}

void mclTaskFn(void*) {
    mcl.runLoop();
}

void MonteCarloLocalizer::startTask() {
    if (running) return;
    running = true;
    taskHandle = pros::c::task_create(mclTaskFn, nullptr, TASK_PRIORITY_DEFAULT,
                                       TASK_STACK_DEPTH_DEFAULT, "MCL Task");
}

void MonteCarloLocalizer::stopTask() {
    running = false;
    if (taskHandle != nullptr) {
        pros::c::task_delete(taskHandle);
        taskHandle = nullptr;
    }
}

void MonteCarloLocalizer::runLoop() {
    std::uint32_t lastWake = pros::millis();

    while (running) {
        if (chassisPtr == nullptr) {
            pros::c::task_delay_until(&lastWake, config::LOOP_PERIOD_MS);
            continue;
        }

        lemlib::Pose odomPose = chassisPtr->getPose();

        predict(odomPose);
        SensorSuite readings = readSensors();
        updateWeights(readings);
        normalizeWeights();
        resample();
        computeFusedPose();

        pros::c::task_delay_until(&lastWake, config::LOOP_PERIOD_MS);
    }
}

void MonteCarloLocalizer::predict(const lemlib::Pose& currentOdomPose) {
    if (!hasPrevOdom) {
        prevOdomPose = currentOdomPose;
        hasPrevOdom = true;
        return;
    }

    double fdx = currentOdomPose.x - prevOdomPose.x;
    double fdy = currentOdomPose.y - prevOdomPose.y;
    double dtheta = wrap180(currentOdomPose.theta - prevOdomPose.theta);

    double rx, ry;
    fieldToRobotVector(fdx, fdy, prevOdomPose.theta, rx, ry);

    prevOdomPose = currentOdomPose;

    double transMag = std::sqrt(fdx * fdx + fdy * fdy);
    double transSigma = config::ODOM_TRANS_NOISE_BASE_IN +
                         config::ODOM_TRANS_NOISE_PER_IN * transMag;
    double rotSigma = config::ODOM_ROT_NOISE_BASE_DEG +
                       config::ODOM_ROT_NOISE_PER_DEG * std::fabs(dtheta) +
                       config::ODOM_ROT_NOISE_PER_IN * transMag;

    std::normal_distribution<double> transNoise(0.0, std::max(transSigma, 1e-6));
    std::normal_distribution<double> rotNoise(0.0, std::max(rotSigma, 1e-6));

    for (auto& p : particles) {
        double noisyRx = rx + transNoise(rng);
        double noisyRy = ry + transNoise(rng);

        double px_f, py_f;
        robotToFieldVector(noisyRx, noisyRy, p.theta_deg, px_f, py_f);

        p.x_in = std::clamp(p.x_in + px_f, 0.0, config::FIELD_WIDTH_IN);
        p.y_in = std::clamp(p.y_in + py_f, 0.0, config::FIELD_LENGTH_IN);
        p.theta_deg = wrap360(p.theta_deg + dtheta + rotNoise(rng));
    }
}

SensorReading MonteCarloLocalizer::readOneSensor(pros::Distance& sensor) {
    SensorReading r;

    std::int32_t raw_mm = sensor.get();
    if (raw_mm == PROS_ERR) {
        r.valid = false;
        return r;
    }

    constexpr double MM_TO_IN = 1.0 / 25.4;

    if (raw_mm >= 9999) {
        r.valid = true;
        r.maxedOut = true;
        r.distance_in = config::SENSOR_MAX_RANGE_IN;
        r.confidence = 1.0;
        r.objectSizeNorm = 0.0;
        return r;
    }

    double dist_in = raw_mm * MM_TO_IN;

    std::int32_t rawConf = sensor.get_confidence();
    std::int32_t rawSize = sensor.get_object_size();

    double confidence;
    if (raw_mm <= 200 || rawConf == PROS_ERR) {
        confidence = 1.0;
    } else {
        confidence = clamp01(static_cast<double>(rawConf) / 63.0);
    }

    double sizeNorm = (rawSize == PROS_ERR)
                           ? 0.5
                           : clamp01(static_cast<double>(rawSize) / config::OBJECT_SIZE_NORMALIZER);

    r.valid = true;
    r.maxedOut = false;
    r.distance_in = std::clamp(dist_in, config::SENSOR_MIN_RANGE_IN, config::SENSOR_MAX_RANGE_IN);
    r.confidence = confidence;
    r.objectSizeNorm = sizeNorm;
    return r;
}

SensorSuite MonteCarloLocalizer::readSensors() {
    SensorSuite s;
    s.north = readOneSensor(dNorth);
    s.east = readOneSensor(dEast);
    s.south = readOneSensor(dSouth);
    s.west = readOneSensor(dWest);
    return s;
}

MonteCarloLocalizer::RaycastResult
MonteCarloLocalizer::raycast(double ox, double oy, double headingDeg) const {
    double dx, dy;
    headingToUnitVector(headingDeg, dx, dy);

    constexpr double EPS = 1e-9;
    double tWall = std::numeric_limits<double>::infinity();

    if (dx > EPS) {
        tWall = std::min(tWall, (config::FIELD_WIDTH_IN - ox) / dx);
    } else if (dx < -EPS) {
        tWall = std::min(tWall, (0.0 - ox) / dx);
    }
    if (dy > EPS) {
        tWall = std::min(tWall, (config::FIELD_LENGTH_IN - oy) / dy);
    } else if (dy < -EPS) {
        tWall = std::min(tWall, (0.0 - oy) / dy);
    }
    if (!std::isfinite(tWall) || tWall < 0) {
        tWall = std::max(config::FIELD_WIDTH_IN, config::FIELD_LENGTH_IN);
    }

    RaycastResult result{tWall, false, tWall, 0.0};

    for (const auto& pole : config::FIELD_POLES) {
        double fx = ox - pole.x_in;
        double fy = oy - pole.y_in;
        double r = pole.diameter_in * 0.5;

        double a = dx * dx + dy * dy;
        double b = 2.0 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - r * r;
        double disc = b * b - 4.0 * a * c;
        if (disc < 0) continue;

        double sq = std::sqrt(disc);
        double t1 = (-b - sq) / (2.0 * a);
        double t2 = (-b + sq) / (2.0 * a);
        double t = (t1 > 1e-6) ? t1 : t2;
        if (t <= 1e-6) continue;
        if (t >= tWall) continue;

        if (!result.hitPole || t < result.poleDist_in) {
            result.hitPole = true;
            result.poleDist_in = t;
            result.poleDiameter_in = pole.diameter_in;
        }
    }

    return result;
}

double MonteCarloLocalizer::beamModelProb(double measured_in, double expected_in) const {
    double p = 0.0;

    {
        double sigma = config::SENSOR_SIGMA_HIT_IN;
        double diff = measured_in - expected_in;
        double gauss = (1.0 / (sigma * std::sqrt(2.0 * PI))) *
                       std::exp(-0.5 * diff * diff / (sigma * sigma));
        p += config::Z_HIT * gauss;
    }

    if (measured_in < expected_in && expected_in > 1e-6) {
        double lambda = config::SENSOR_LAMBDA_SHORT;
        double denom = 1.0 - std::exp(-lambda * expected_in);
        if (denom > 1e-9) {
            double eta = 1.0 / denom;
            p += config::Z_SHORT * eta * lambda * std::exp(-lambda * measured_in);
        }
    }

    if (measured_in >= config::SENSOR_MAX_RANGE_IN - 0.5) {
        p += config::Z_MAX;
    }

    if (measured_in >= 0.0 && measured_in <= config::SENSOR_MAX_RANGE_IN) {
        p += config::Z_RAND * (1.0 / config::SENSOR_MAX_RANGE_IN);
    }

    return p;
}

double MonteCarloLocalizer::likelihoodForSensor(const Particle& p,
                                                 const config::SensorMount& mount,
                                                 const SensorReading& reading) const {
    if (!reading.valid) return 1.0;

    double sx_r = mount.x_offset_in, sy_r = mount.y_offset_in;
    double sx_f, sy_f;
    robotToFieldVector(sx_r, sy_r, p.theta_deg, sx_f, sy_f);
    double originX = p.x_in + sx_f;
    double originY = p.y_in + sy_f;
    double sensorHeading = wrap360(p.theta_deg + mount.heading_offset_deg);

    RaycastResult cast = raycast(originX, originY, sensorHeading);

    double likelihood;
    if (cast.hitPole) {
        double angularSubtenseDeg =
            toDegrees(2.0 * std::atan((cast.poleDiameter_in * 0.5) /
                                       std::max(cast.poleDist_in, 1e-3)));
        double beamFullAngle = 2.0 * config::SENSOR_BEAM_HALF_ANGLE_DEG;

        double pDetect = (angularSubtenseDeg >= beamFullAngle)
                              ? 1.0
                              : std::clamp(angularSubtenseDeg / beamFullAngle,
                                            config::MIN_POLE_DETECT_PROB, 1.0);

        double poleProb = beamModelProb(reading.distance_in, cast.poleDist_in);
        double wallProb = beamModelProb(reading.distance_in, cast.wallDist_in);
        likelihood = pDetect * poleProb + (1.0 - pDetect) * wallProb;
    } else {
        likelihood = beamModelProb(reading.distance_in, cast.wallDist_in);
    }

    double reliability = clamp01(0.5 * reading.confidence + 0.5 * reading.objectSizeNorm);
    double uniform = 1.0 / config::SENSOR_MAX_RANGE_IN;
    likelihood = reliability * likelihood + (1.0 - reliability) * uniform;

    return std::max(likelihood, 1e-9);
}

void MonteCarloLocalizer::updateWeights(const SensorSuite& readings) {
    std::array<std::pair<const config::SensorMount*, const SensorReading*>, 4> sensors{{
        {&config::MOUNT_NORTH, &readings.north},
        {&config::MOUNT_EAST, &readings.east},
        {&config::MOUNT_SOUTH, &readings.south},
        {&config::MOUNT_WEST, &readings.west},
    }};

    for (auto& p : particles) {
        double logWeight = std::log(std::max(p.weight, 1e-300));
        for (auto& [mount, reading] : sensors) {
            double lik = likelihoodForSensor(p, *mount, *reading);
            logWeight += std::log(lik);
        }
        p.weight = logWeight;
    }
}

void MonteCarloLocalizer::normalizeWeights() {
    double maxLog = -std::numeric_limits<double>::infinity();
    for (auto& p : particles) maxLog = std::max(maxLog, p.weight);

    double sum = 0.0;
    for (auto& p : particles) {
        p.weight = std::exp(p.weight - maxLog);
        sum += p.weight;
    }

    if (sum < 1e-300) {
        for (auto& p : particles) p.weight = 1.0 / particles.size();
        return;
    }

    for (auto& p : particles) p.weight /= sum;
}

void MonteCarloLocalizer::resample() {
    double sumSqWeights = 0.0;
    for (auto& p : particles) sumSqWeights += p.weight * p.weight;
    double neff = (sumSqWeights > 1e-300) ? (1.0 / sumSqWeights) : 0.0;
    lastNeff = neff;

    bool collapsed = neff < 2.0;
    if (neff >= config::RESAMPLE_NEFF_RATIO * particles.size() && !collapsed) {
        return;
    }

    std::size_t n = particles.size();
    std::vector<Particle> resampled;
    resampled.reserve(n);

    std::uniform_real_distribution<double> uni(0.0, 1.0 / n);
    double r = uni(rng);
    double c = particles[0].weight;
    std::size_t i = 0;

    for (std::size_t m = 0; m < n; ++m) {
        double u = r + static_cast<double>(m) / n;
        while (u > c && i < n - 1) {
            ++i;
            c += particles[i].weight;
        }
        resampled.push_back(particles[i]);
    }

    double injectionRatio = collapsed ? config::KIDNAP_INJECTION_RATIO
                                       : config::RANDOM_INJECTION_RATIO;
    std::size_t injectCount = static_cast<std::size_t>(injectionRatio * n);

    std::uniform_real_distribution<double> ux(0.0, config::FIELD_WIDTH_IN);
    std::uniform_real_distribution<double> uy(0.0, config::FIELD_LENGTH_IN);
    std::uniform_real_distribution<double> ut(0.0, 360.0);
    std::uniform_int_distribution<std::size_t> pickIdx(0, n - 1);

    for (std::size_t k = 0; k < injectCount; ++k) {
        std::size_t idx = pickIdx(rng);
        resampled[idx].x_in = ux(rng);
        resampled[idx].y_in = uy(rng);
        resampled[idx].theta_deg = ut(rng);
    }

    double uniformWeight = 1.0 / n;
    for (auto& p : resampled) p.weight = uniformWeight;

    particles = std::move(resampled);
}

void MonteCarloLocalizer::computeFusedPose() {
    double sumX = 0.0, sumY = 0.0, sumSin = 0.0, sumCos = 0.0;
    for (auto& p : particles) {
        sumX += p.weight * p.x_in;
        sumY += p.weight * p.y_in;
        double rad = toRadians(p.theta_deg);
        sumSin += p.weight * std::sin(rad);
        sumCos += p.weight * std::cos(rad);
    }

    double meanTheta = wrap360(toDegrees(std::atan2(sumSin, sumCos)));

    poseMutex.take(TIMEOUT_MAX);
    fusedPose = lemlib::Pose(sumX, sumY, meanTheta);
    poseMutex.give();
}

lemlib::Pose MonteCarloLocalizer::getFusedPose() {
    poseMutex.take(TIMEOUT_MAX);
    lemlib::Pose copy = fusedPose;
    poseMutex.give();
    return copy;
}

double MonteCarloLocalizer::getEffectiveSampleSize() {
    return lastNeff;
}

}