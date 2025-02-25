/**
 * @file PS_Branch.cpp
 * @brief Реализация класса ветви энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_Branch.hpp"
#include <sstream>

namespace PowerSystem {

    Branch::Branch()
        : PowerSystemElement(),
        m_fromBusId(0),
        m_toBusId(0),
        m_impedance(0.0, 0.0),
        m_admittance(0.0, 0.0),
        m_ratedCurrent(0.0),
        m_current(0.0, 0.0),
        m_power(0.0, 0.0) {
    }

    Branch::Branch(Types::ElementId id, const std::string& name,
        Types::ElementId fromBusId, Types::ElementId toBusId)
        : PowerSystemElement(id, name),
        m_fromBusId(fromBusId),
        m_toBusId(toBusId),
        m_impedance(0.0, 0.0),
        m_admittance(0.0, 0.0),
        m_ratedCurrent(0.0),
        m_current(0.0, 0.0),
        m_power(0.0, 0.0) {
    }

    Types::ElementId Branch::getFromBusId() const {
        return m_fromBusId;
    }

    void Branch::setFromBusId(Types::ElementId busId) {
        m_fromBusId = busId;
    }

    Types::ElementId Branch::getToBusId() const {
        return m_toBusId;
    }

    void Branch::setToBusId(Types::ElementId busId) {
        m_toBusId = busId;
    }

    Types::Complex Branch::getImpedance() const {
        return m_impedance;
    }

    void Branch::setImpedance(const Types::Complex& impedance) {
        m_impedance = impedance;
        // Автоматический расчет проводимости как 1/z
        if (m_impedance != Types::Complex(0.0, 0.0)) {
            m_admittance = Types::Complex(1.0, 0.0) / m_impedance;
        }
        else {
            m_admittance = Types::Complex(0.0, 0.0);
        }
    }

    Types::Complex Branch::getAdmittance() const {
        return m_admittance;
    }

    void Branch::setAdmittance(const Types::Complex& admittance) {
        m_admittance = admittance;
        // Автоматический расчет сопротивления как 1/y
        if (m_admittance != Types::Complex(0.0, 0.0)) {
            m_impedance = Types::Complex(1.0, 0.0) / m_admittance;
        }
        else {
            m_impedance = Types::Complex(0.0, 0.0);
        }
    }

    double Branch::getRatedCurrent() const {
        return m_ratedCurrent;
    }

    void Branch::setRatedCurrent(double current) {
        m_ratedCurrent = current;
    }

    Types::Complex Branch::getCurrent() const {
        return m_current;
    }

    void Branch::setCurrent(const Types::Complex& current) {
        m_current = current;
    }

    Types::Complex Branch::getPower() const {
        return m_power;
    }

    void Branch::setPower(const Types::Complex& power) {
        m_power = power;
    }

    bool Branch::initialize() {
        // Базовая инициализация ветви
        return PowerSystemElement::initialize();
    }

    bool Branch::compute(Types::TimeType time, Types::TimeType timeStep) {
        // В MVP просто заглушка для будущей реализации
        return PowerSystemElement::compute(time, timeStep);
    }

    std::string Branch::serialize() const {
        std::stringstream ss;
        ss << PowerSystemElement::serialize();
        ss << "FROM_BUS_ID=" << m_fromBusId << ";";
        ss << "TO_BUS_ID=" << m_toBusId << ";";
        ss << "IMPEDANCE_RE=" << m_impedance.real() << ";";
        ss << "IMPEDANCE_IM=" << m_impedance.imag() << ";";
        ss << "ADMITTANCE_RE=" << m_admittance.real() << ";";
        ss << "ADMITTANCE_IM=" << m_admittance.imag() << ";";
        ss << "RATED_CURRENT=" << m_ratedCurrent << ";";
        ss << "CURRENT_RE=" << m_current.real() << ";";
        ss << "CURRENT_IM=" << m_current.imag() << ";";
        ss << "POWER_RE=" << m_power.real() << ";";
        ss << "POWER_IM=" << m_power.imag() << ";";

        return ss.str();
    }

    bool Branch::deserialize(const std::string& data) {
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

                    if (key == "FROM_BUS_ID") {
                        m_fromBusId = std::stoull(value);
                    }
                    else if (key == "TO_BUS_ID") {
                        m_toBusId = std::stoull(value);
                    }
                    else if (key == "IMPEDANCE_RE") {
                        m_impedance.real(std::stod(value));
                    }
                    else if (key == "IMPEDANCE_IM") {
                        m_impedance.imag(std::stod(value));
                    }
                    else if (key == "ADMITTANCE_RE") {
                        m_admittance.real(std::stod(value));
                    }
                    else if (key == "ADMITTANCE_IM") {
                        m_admittance.imag(std::stod(value));
                    }
                    else if (key == "RATED_CURRENT") {
                        m_ratedCurrent = std::stod(value);
                    }
                    else if (key == "CURRENT_RE") {
                        m_current.real(std::stod(value));
                    }
                    else if (key == "CURRENT_IM") {
                        m_current.imag(std::stod(value));
                    }
                    else if (key == "POWER_RE") {
                        m_power.real(std::stod(value));
                    }
                    else if (key == "POWER_IM") {
                        m_power.imag(std::stod(value));
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