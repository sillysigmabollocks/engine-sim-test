#include "../include/vtec_valvetrain.h"

#include "../include/engine.h"
#include <algorithm>

VtecValvetrain::VtecValvetrain() {
    m_intakeCamshaft = nullptr;
    m_exhaustCamshaft = nullptr;

    m_vtecIntakeCamshaft = nullptr;
    m_vtecExhaustCamshaft = nullptr;

    m_engine = nullptr;

    m_minRpm = 0.0;
    m_minSpeed = 0.0;
    m_minThrottlePosition = 0.0;
    m_manifoldVacuum = 0.0;
}

VtecValvetrain::~VtecValvetrain() {
    /* void */
}

void VtecValvetrain::initialize(const Parameters &parameters) {
    m_intakeCamshaft = parameters.intakeCamshaft;
    m_exhaustCamshaft = parameters.exhaustCamshaft;
    m_vtecIntakeCamshaft = parameters.vtecIntakeCamshaft;
    m_vtecExhaustCamshaft = parameters.vtexExhaustCamshaft;

    m_minRpm = parameters.minRpm;
    m_minSpeed = parameters.minSpeed;
    m_minThrottlePosition = parameters.minThrottlePosition;
    m_manifoldVacuum = parameters.manifoldVacuum;
    m_engine = parameters.engine;
    m_maxIntakeAdvance = parameters.maxIntakeAdvance;
    m_lowBaseAdvance = m_intakeCamshaft->getAdvance();
    m_highBaseAdvance = m_vtecIntakeCamshaft->getAdvance();
    m_intakeHigh.assign(m_intakeCamshaft->getLobeCount(), false);
    m_exhaustHigh.assign(m_exhaustCamshaft->getLobeCount(), false);
    m_liftRequested = false;
}

void VtecValvetrain::updateIntakeAdvance() {
    const double rpm = units::toRpm(m_engine->getSpeed());
    const double speedFactor = std::clamp(1.0 - std::abs(rpm - 3500.0) / 3500.0, 0.0, 1.0);
    const double load = std::clamp(1.0 - m_engine->getThrottle(), 0.0, 1.0);
    const double advance = m_maxIntakeAdvance * speedFactor * load;
    m_intakeCamshaft->setAdvance(m_lowBaseAdvance + advance);
    m_vtecIntakeCamshaft->setAdvance(m_highBaseAdvance + advance);
}

double VtecValvetrain::intakeValveLift(int cylinder) {
    updateIntakeAdvance();
    const bool requested = isVtecEnabled();
    Camshaft *current = m_intakeHigh[cylinder] ? m_vtecIntakeCamshaft : m_intakeCamshaft;
    Camshaft *target = requested ? m_vtecIntakeCamshaft : m_intakeCamshaft;
    if (m_intakeHigh[cylinder] != requested &&
        current->valveLift(cylinder) <= 0.0 && target->valveLift(cylinder) <= 0.0) {
        m_intakeHigh[cylinder] = requested;
        current = target;
    }
    return current->valveLift(cylinder);
}

double VtecValvetrain::exhaustValveLift(int cylinder) {
    const bool requested = isVtecEnabled();
    Camshaft *current = m_exhaustHigh[cylinder] ? m_vtecExhaustCamshaft : m_exhaustCamshaft;
    Camshaft *target = requested ? m_vtecExhaustCamshaft : m_exhaustCamshaft;
    if (m_exhaustHigh[cylinder] != requested &&
        current->valveLift(cylinder) <= 0.0 && target->valveLift(cylinder) <= 0.0) {
        m_exhaustHigh[cylinder] = requested;
        current = target;
    }
    return current->valveLift(cylinder);
}

Camshaft *VtecValvetrain::getActiveIntakeCamshaft() {
    return isVtecEnabled() ? m_vtecIntakeCamshaft : m_intakeCamshaft;
}

Camshaft *VtecValvetrain::getActiveExhaustCamshaft() {
    return isVtecEnabled() ? m_vtecExhaustCamshaft : m_exhaustCamshaft;
}

bool VtecValvetrain::isVtecEnabled() {
    const double threshold = std::max(0.0, m_minRpm - (m_liftRequested ? units::rpm(300) : 0.0));
    m_liftRequested = m_engine->getManifoldPressure() > m_manifoldVacuum
        && m_engine->getSpeed() > threshold
        && (1 - m_engine->getThrottle()) > m_minThrottlePosition;
    return m_liftRequested;
}
