/**
 * @file PS_Generator.cpp
 * @brief Реализация базового класса генератора энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_Generator.hpp"
#include <sstream>

namespace PowerSystem {

    Generator::Generator()
        : PowerSystemElement(),
        m_busId(0),
        m_nominalPower(0.0, 0.0),
        m_voltage(1.0),
        m_activePower(0.0),
        m_reactivePower(0.0),
        m_maxReactivePower(0.0),
        m_minReactivePower(0.0) {
    }

    Generator::Generator(Types::ElementId id, const std::string& name, Types::ElementId busId)
        : PowerSystemElement(id, name),
        m_busId(busId),
        m_nominalPower(0.0, 0.0),
        m_voltage(1.0),
        m_activePower(0.0),
        m_reactivePower(0.0),
        m_maxReactivePower(0.0),
        m_minReactivePower(0.0) {
    }

    Types::ElementId Generator::getBusId() const {
        return m_busId;
    }

    void Generator::setBusId(Types::ElementId busId) {
        m_busId = busId;
    }

    Types::Complex Generator::getNominalPower() const {
        return m_nominalPower;
    }

    void Generator::setNominalPower(const Types::Complex& power) {
        m_nominalPower = power;
    }

    double Generator::getVoltage() const {
        return m_voltage;
    }

    void Generator::setVoltage(double voltage) {
        m_voltage = voltage;
    }

    double Generator::getActivePower() const {
        return m_activePower;
    }

    void Generator::setActivePower(double power) {
        m_activePower = power;
    }

    double Generator::getReactivePower() const {
        return m_reactivePower;
    }

    void Generator::setReactivePower(double power) {
        m_reactivePower = power;
    }

    double Generator::getMaxReactivePower() const {
        return m_maxReactivePower;
    }

    void Generator::setMaxReactivePower(double power) {
        m_maxReactivePower = power;
    }

    double Generator::getMinReactivePower() const {
        return m_minReactivePower;
    }

    void Generator::setMinReactivePower(double power) {
        m_minReactivePower = power;
    }

    bool Generator::initialize() {
        // Базовая инициализация генератора
        return PowerSystemElement::initialize();
    }

    bool Generator::compute(Types::TimeType time, Types::TimeType timeStep) {
        // В MVP просто заглушка для будущей реализации
        return PowerSystemElement::compute(time, timeStep);
    }

    std::string Generator::serialize() const {
        std::stringstream ss;
        ss << PowerSystemElement::serialize();
        ss << "BUS_ID=" << m_busId << ";";
        ss << "NOMINAL_POWER_RE=" << m_nominalPower.real() << ";";
        ss << "NOMINAL_POWER_IM=" << m_nominalPower.imag() << ";";
        ss << "VOLTAGE=" << m_voltage << ";";
        ss << "ACTIVE_POWER=" << m_activePower << ";";
        ss << "REACTIVE_POWER=" << m_reactivePower << ";";
        ss << "MAX_REACTIVE_POWER=" << m_maxReactivePower << ";";
        ss << "MIN_REACTIVE_POWER=" << m_minReactivePower << ";";

        return ss.str();
    }

    bool Generator::deserialize(const std::string& data) {
        if (!PowerSystemElement::deserialize(data)) {
            return false;
        }

        try {
            size_t pos = 0;
            std::string token;
            std::string str = data;

            while ((pos = str.find(";")) != std::string::npos) {
                token = str.substr(0, pos);

                size_t eqPos = token.find("=");
                if (eqPos != std::string::npos) {
                    std::string key = token.substr(0, eqPos);
                    std::string value = token.substr(eqPos + 1);

                    if (key == "BUS_ID") {
                        m_busId = std::stoull(value);
                    }
                    else if (key == "NOMINAL_POWER_RE") {
                        m_nominalPower.real(std::stod(value));
                    }
                    else if (key == "NOMINAL_POWER_IM") {
                        m_nominalPower.imag(std::stod(value));
                    }
                    else if (key == "VOLTAGE") {
                        m_voltage = std::stod(value);
                    }
                    else if (key == "ACTIVE_POWER") {
                        m_activePower = std::stod(value);
                    }
                    else if (key == "REACTIVE_POWER") {
                        m_reactivePower = std::stod(value);
                    }
                    else if (key == "MAX_REACTIVE_POWER") {
                        m_maxReactivePower = std::stod(value);
                    }
                    else if (key == "MIN_REACTIVE_POWER") {
                        m_minReactivePower = std::stod(value);
                    }
                }

                str.erase(0, pos + 1);
            }

            return true;
        }
        catch (const std::exception& e) {
            return false;
        }
    }

} // namespace PowerSystem