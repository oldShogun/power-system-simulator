/**
 * @file PS_Bus.cpp
 * @brief Реализация класса узла энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_Bus.hpp"
#include <sstream>
#include <algorithm>

namespace PowerSystem {

    Bus::Bus()
        : PowerSystemElement(),
        m_type(BusType::PQ),
        m_voltage(1.0, 0.0),
        m_power(0.0, 0.0) {
    }

    Bus::Bus(Types::ElementId id, const std::string& name, BusType type)
        : PowerSystemElement(id, name),
        m_type(type),
        m_voltage(1.0, 0.0),
        m_power(0.0, 0.0) {
    }

    BusType Bus::getType() const {
        return m_type;
    }

    void Bus::setType(BusType type) {
        m_type = type;
    }

    Types::Complex Bus::getVoltage() const {
        return m_voltage;
    }

    void Bus::setVoltage(const Types::Complex& voltage) {
        m_voltage = voltage;
    }

    Types::Complex Bus::getPower() const {
        return m_power;
    }

    void Bus::setPower(const Types::Complex& power) {
        m_power = power;
    }

    void Bus::addElement(Types::ElementId elementId) {
        if (std::find(m_elements.begin(), m_elements.end(), elementId) == m_elements.end()) {
            m_elements.push_back(elementId);
        }
    }

    bool Bus::removeElement(Types::ElementId elementId) {
        auto it = std::find(m_elements.begin(), m_elements.end(), elementId);
        if (it != m_elements.end()) {
            m_elements.erase(it);
            return true;
        }
        return false;
    }

    const std::vector<Types::ElementId>& Bus::getElements() const {
        return m_elements;
    }

    bool Bus::initialize() {
        // Базовая инициализация узла
        return PowerSystemElement::initialize();
    }

    bool Bus::compute(Types::TimeType time, Types::TimeType timeStep) {
        // В MVP просто заглушка для будущей реализации
        return PowerSystemElement::compute(time, timeStep);
    }

    std::string Bus::serialize() const {
        std::stringstream ss;
        ss << PowerSystemElement::serialize();
        ss << "BUS_TYPE=" << static_cast<int>(m_type) << ";";
        ss << "VOLTAGE_RE=" << m_voltage.real() << ";";
        ss << "VOLTAGE_IM=" << m_voltage.imag() << ";";
        ss << "POWER_RE=" << m_power.real() << ";";
        ss << "POWER_IM=" << m_power.imag() << ";";

        // Сериализация подключенных элементов
        ss << "ELEMENTS=";
        for (size_t i = 0; i < m_elements.size(); ++i) {
            if (i > 0) ss << ",";
            ss << m_elements[i];
        }
        ss << ";";

        return ss.str();
    }

    bool Bus::deserialize(const std::string& data) {
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

                    if (key == "BUS_TYPE") {
                        m_type = static_cast<BusType>(std::stoi(value));
                    }
                    else if (key == "VOLTAGE_RE") {
                        m_voltage.real(std::stod(value));
                    }
                    else if (key == "VOLTAGE_IM") {
                        m_voltage.imag(std::stod(value));
                    }
                    else if (key == "POWER_RE") {
                        m_power.real(std::stod(value));
                    }
                    else if (key == "POWER_IM") {
                        m_power.imag(std::stod(value));
                    }
                    else if (key == "ELEMENTS") {
                        m_elements.clear();

                        if (!value.empty()) {
                            size_t commaPos = 0;
                            std::string elemStr = value;

                            while ((commaPos = elemStr.find(",")) != std::string::npos) {
                                std::string elemId = elemStr.substr(0, commaPos);
                                m_elements.push_back(std::stoull(elemId));
                                elemStr.erase(0, commaPos + 1);
                            }

                            if (!elemStr.empty()) {
                                m_elements.push_back(std::stoull(elemStr));
                            }
                        }
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