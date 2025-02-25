/**
 * @file PS_PowerSystemElement.cpp
 * @brief Реализация базового класса элемента энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_PowerSystemElement.hpp"
#include <sstream>
#include <iostream>

namespace PowerSystem {

    PowerSystemElement::PowerSystemElement()
        : m_id(0), m_name(""), m_state(Types::ElementState::DISABLED) {
    }

    PowerSystemElement::PowerSystemElement(Types::ElementId id, const std::string& name)
        : m_id(id), m_name(name), m_state(Types::ElementState::DISABLED) {
    }

    Types::ElementId PowerSystemElement::getId() const {
        return m_id;
    }

    void PowerSystemElement::setId(Types::ElementId id) {
        m_id = id;
    }

    std::string PowerSystemElement::getName() const {
        return m_name;
    }

    void PowerSystemElement::setName(const std::string& name) {
        m_name = name;
    }

    bool PowerSystemElement::initialize() {
        // Базовая инициализация, переопределяется в производных классах
        return true;
    }

    bool PowerSystemElement::compute(Types::TimeType time, Types::TimeType timeStep) {
        // Базовый расчет, переопределяется в производных классах
        return true;
    }

    std::string PowerSystemElement::serialize() const {
        std::stringstream ss;
        ss << "ID=" << m_id << ";";
        ss << "NAME=" << m_name << ";";
        ss << "STATE=" << static_cast<int>(m_state) << ";";

        // Сериализация параметров
        for (const auto& param : m_parameters) {
            ss << "PARAM:" << param.first << "=" << param.second << ";";
        }

        return ss.str();
    }

    bool PowerSystemElement::deserialize(const std::string& data) {
        // Простая десериализация для MVP, в будущем можно улучшить
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

                    if (key == "ID") {
                        m_id = std::stoull(value);
                    }
                    else if (key == "NAME") {
                        m_name = value;
                    }
                    else if (key == "STATE") {
                        m_state = static_cast<Types::ElementState>(std::stoi(value));
                    }
                    else if (key.find("PARAM:") == 0) {
                        std::string paramName = key.substr(6);
                        m_parameters[paramName] = std::stod(value);
                    }
                }

                str.erase(0, pos + 1);
            }

            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Error in deserialize: " << e.what() << std::endl;
            return false;
        }
    }

    Types::ElementState PowerSystemElement::getState() const {
        return m_state;
    }

    void PowerSystemElement::setState(Types::ElementState state) {
        m_state = state;
    }

    void PowerSystemElement::setParameter(const std::string& name, double value) {
        m_parameters[name] = value;
    }

    double PowerSystemElement::getParameter(const std::string& name) const {
        auto it = m_parameters.find(name);
        if (it == m_parameters.end()) {
            throw DataAccessException("Parameter not found: " + name);
        }
        return it->second;
    }

    bool PowerSystemElement::hasParameter(const std::string& name) const {
        return m_parameters.find(name) != m_parameters.end();
    }

} // namespace PowerSystem