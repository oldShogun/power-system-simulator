/**
 * @file PS_Load.cpp
 * @brief Реализация класса нагрузки энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_Load.hpp"
#include <sstream>

namespace PowerSystem {

    Load::Load()
        : PowerSystemElement(),
        m_busId(0),
        m_model(LoadModel::CONSTANT_POWER),
        m_power(0.0, 0.0),
        m_powerFactor(0.9) {
    }

    Load::Load(Types::ElementId id, const std::string& name, Types::ElementId busId)
        : PowerSystemElement(id, name),
        m_busId(busId),
        m_model(LoadModel::CONSTANT_POWER),
        m_power(0.0, 0.0),
        m_powerFactor(0.9) {
    }

    Types::ElementId Load::getBusId() const {
        return m_busId;
    }

    void Load::setBusId(Types::ElementId busId) {
        m_busId = busId;
    }

    LoadModel Load::getModel() const {
        return m_model;
    }

    void Load::setModel(LoadModel model) {
        m_model = model;
    }

    Types::Complex Load::getPower() const {
        return m_power;
    }

    void Load::setPower(const Types::Complex& power) {
        m_power = power;
        // Обновление коэффициента мощности
        if (std::abs(m_power) > 0) {
            m_powerFactor = m_power.real() / std::abs(m_power);
        }
    }

    double Load::getPowerFactor() const {
        return m_powerFactor;
    }

    void Load::setPowerFactor(double factor) {
        if (factor < -1.0) factor = -1.0;
        if (factor > 1.0) factor = 1.0;

        m_powerFactor = factor;

        // Обновление реактивной мощности при изменении коэффициента мощности
        double p = m_power.real();
        if (m_powerFactor != 0) {
            double q = p * std::sqrt(1.0 - m_powerFactor * m_powerFactor) / m_powerFactor;
            // Учет знака реактивной мощности
            q = (m_power.imag() >= 0) ? std::abs(q) : -std::abs(q);
            m_power.imag(q);
        }
    }

    void Load::setModelParameter(const std::string& name, double value) {
        m_modelParams[name] = value;
    }

    double Load::getModelParameter(const std::string& name) const {
        auto it = m_modelParams.find(name);
        if (it == m_modelParams.end()) {
            throw DataAccessException("Model parameter not found: " + name);
        }
        return it->second;
    }

    bool Load::hasModelParameter(const std::string& name) const {
        return m_modelParams.find(name) != m_modelParams.end();
    }

    bool Load::initialize() {
        // Базовая инициализация нагрузки
        return PowerSystemElement::initialize();
    }

    bool Load::compute(Types::TimeType time, Types::TimeType timeStep) {
        // В MVP просто заглушка для будущей реализации
        return PowerSystemElement::compute(time, timeStep);
    }

    std::string Load::serialize() const {
        std::stringstream ss;
        ss << PowerSystemElement::serialize();
        ss << "BUS_ID=" << m_busId << ";";
        ss << "LOAD_MODEL=" << static_cast<int>(m_model) << ";";
        ss << "POWER_RE=" << m_power.real() << ";";
        ss << "POWER_IM=" << m_power.imag() << ";";
        ss << "POWER_FACTOR=" << m_powerFactor << ";";

        // Сериализация параметров модели
        for (const auto& param : m_modelParams) {
            ss << "MODEL_PARAM:" << param.first << "=" << param.second << ";";
        }

        return ss.str();
    }

    bool Load::deserialize(const std::string& data) {
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
                    else if (key == "LOAD_MODEL") {
                        m_model = static_cast<LoadModel>(std::stoi(value));
                    }
                    else if (key == "POWER_RE") {
                        m_power.real(std::stod(value));
                    }
                    else if (key == "POWER_IM") {
                        m_power.imag(std::stod(value));
                    }
                    else if (key == "POWER_FACTOR") {
                        m_powerFactor = std::stod(value);
                    }
                    else if (key.find("MODEL_PARAM:") == 0) {
                        std::string paramName = key.substr(12);
                        m_modelParams[paramName] = std::stod(value);
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