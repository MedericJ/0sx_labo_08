// Alarm.cpp

#include "Alarm.h"

Alarm::Alarm(int rPin, int gPin, int bPin, int buzzerPin, float& distancePtr)
  : _rPin(rPin), _gPin(gPin), _bPin(bPin), _buzzerPin(buzzerPin), _distance(&distancePtr) {
    pinMode(_rPin, OUTPUT);
    pinMode(_gPin, OUTPUT);
    pinMode(_bPin, OUTPUT);
    pinMode(_buzzerPin, OUTPUT);
    _setRGB(0, 0, 0);
}

void Alarm::update() {
    _currentTime = millis();

    switch (_state) {
        case OFF: _offState(); break;
        case WATCHING: _watchState(); break;
        case ON: _onState(); break;
        case TESTING: _testingState(); break;
    }
}

void Alarm::setColourA(int r, int g, int b) {
    _colA[0] = r; _colA[1] = g; _colA[2] = b;
}

void Alarm::setColourB(int r, int g, int b) {
    _colB[0] = r; _colB[1] = g; _colB[2] = b;
}

void Alarm::setVariationTiming(unsigned long ms) {
    _variationRate = ms;
}

void Alarm::setDistance(float d) {
    _distanceTrigger = d;
}

float Alarm::getDistance() const {
    return *_distance;
}

void Alarm::setTimeout(unsigned long ms) {
    _timeoutDelay = ms;
}

void Alarm::turnOff() {
    _turnOffFlag = true;
}

void Alarm::turnOn() {
    _turnOnFlag = true;
}

void Alarm::test() {
    _state = TESTING;
    _testStartTime = millis();
    _setRGB(255, 0, 0);
    tone(_buzzerPin, buzzerTone1, buzzerTone2);
}

AlarmState Alarm::getState() const {
    return _state;
}

// --- Méthodes internes ---

void Alarm::_setRGB(int r, int g, int b) {
    analogWrite(_rPin, r);
    if (_gPin != 0) analogWrite(_gPin, g);
    analogWrite(_bPin, b);
}

void Alarm::_turnOff() {
    _setRGB(0, 0, 0);
    noTone(_buzzerPin);
}

void Alarm::_offState() {
    _turnOff();
    if (_turnOnFlag) {
        _turnOnFlag = false;
        _state = WATCHING;
    }
}

void Alarm::_watchState() {
    _turnOff();
    if (_distance && (*_distance <= _distanceTrigger)) {
        _state = ON;
        _lastDetectedTime = _currentTime;
    }
    if (_turnOffFlag) {
        _turnOffFlag = false;
        _state = OFF;
    }
}

void Alarm::_onState() {
    if (_currentTime - _lastUpdate >= _variationRate) {
        _lastUpdate = _currentTime;
        _currentColor = !_currentColor;
        if (_currentColor) _setRGB(_colA[0], _colA[1], _colA[2]);
        else _setRGB(_colB[0], _colB[1], _colB[2]);
    }
    tone(_buzzerPin, buzzerTone1, buzzerTone2);

    if (_distance && (*_distance > _distanceTrigger)) {
        if (_currentTime - _lastDetectedTime >= _timeoutDelay) {
            _state = WATCHING;
            _turnOff();
        }
    } else {
        _lastDetectedTime = _currentTime;
    }

    if (_turnOffFlag) {
        _turnOffFlag = false;
        _state = OFF;
    }
}

void Alarm::_testingState() {
    if (_currentTime - _testStartTime < secDelay) {
        _setRGB(255, 0, 0);
        tone(_buzzerPin, buzzerTone1, buzzerTone2);
    } else {
        _state = OFF;
        _turnOff();
    }
}
