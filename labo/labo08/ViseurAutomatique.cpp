#include "ViseurAutomatique.h"

ViseurAutomatique::ViseurAutomatique(int p1, int p2, int p3, int p4, float& distanceRef)
  : _stepper(AccelStepper::FULL4WIRE, p1, p3, p2, p4), _distance(distanceRef) {
    _stepper.setMaxSpeed(500);
    _stepper.setAcceleration(100);
}

void ViseurAutomatique::update() {
    _currentTime = millis();
    switch (_etat) {
        case INACTIF: _inactifState(_currentTime); break;
        case SUIVI: _suiviState(_currentTime); break;
        case REPOS: _reposState(_currentTime); break;
    }
}

void ViseurAutomatique::setAngleMin(float angle) {_angleMin = angle;}
void ViseurAutomatique::setAngleMax(float angle) {_angleMax = angle;}
void ViseurAutomatique::setPasParTour(int steps) {_stepsPerRev = steps;}
void ViseurAutomatique::setDistanceMinSuivi(float distanceMin) {_distanceMinSuivi = distanceMin;}
void ViseurAutomatique::setDistanceMaxSuivi(float distanceMax) {_distanceMaxSuivi = distanceMax;}

float ViseurAutomatique::getAngle() const {
    long currentStep = _stepper.currentPosition();
    return (float)currentStep * 360.0 / _stepsPerRev;
}

void ViseurAutomatique::activer() {_etat = REPOS;}
void ViseurAutomatique::desactiver() {_etat = INACTIF;}

const char* ViseurAutomatique::getEtatTexte() const {
    switch (_etat) {
        case INACTIF: return "INACTIF";
        case SUIVI: return "SUIVI";
        case REPOS: return "REPOS";
        default: return "INCONNU";
    }
}

void ViseurAutomatique::_inactifState(unsigned long) {
}

void ViseurAutomatique::_suiviState(unsigned long) {
    if (_distance >= _distanceMinSuivi && _distance <= _distanceMaxSuivi) {
        float mappedAngle = map(_distance, _distanceMinSuivi, _distanceMaxSuivi, _angleMin, _angleMax);
        long targetStep = _angleEnSteps(mappedAngle);
        _stepper.moveTo(targetStep);
        _stepper.run();
    } else {
        _etat = REPOS;
    }
}

void ViseurAutomatique::_reposState(unsigned long) {
    if (_distance >= _distanceMinSuivi && _distance <= _distanceMaxSuivi) {
        _etat = SUIVI;
    } else {
        float middleAngle = (_angleMin + _angleMax) / 2.0;
        long middleStep = _angleEnSteps(middleAngle);
        _stepper.moveTo(middleStep);
        _stepper.run();
    }
}

long ViseurAutomatique::_angleEnSteps(float angle) const {
    angle = constrain(angle, _angleMin, _angleMax);
    return (long)(angle * _stepsPerRev / 360.0);
}
