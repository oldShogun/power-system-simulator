/**
 * @file PS_Regulators.cpp
 * @brief Реализация базовых классов регуляторов
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_Regulators.hpp"
#include "PS_ExcitationRegulators.hpp"
#include "PS_TurbineRegulators.hpp"
#include "../Core/PS_Exception.hpp"

namespace PowerSystem {

    // Реализация BaseRegulator
    BaseRegulator::BaseRegulator() : m_initialized(false) {
    }

    bool BaseRegulator::setParameters(const std::map<std::string, double>& parameters) {
        for (const auto& param : parameters) {
            m_parameters[param.first] = param.second;
        }
        return true;
    }

    std::map<std::string, double> BaseRegulator::getParameters() const {
        return m_parameters;
    }

    bool BaseRegulator::reset() {
        m_initialized = false;
        // Сброс внутреннего состояния
        std::fill(m_state.begin(), m_state.end(), 0.0);
        return true;
    }

    bool BaseRegulator::checkRequiredParameters(const std::vector<std::string>& requiredParams) const {
        for (const auto& param : requiredParams) {
            if (m_parameters.find(param) == m_parameters.end()) {
                return false;
            }
        }
        return true;
    }

    // Реализация BaseExcitationRegulator
    BaseExcitationRegulator::BaseExcitationRegulator() : BaseRegulator() {
    }

    // Реализация BaseTurbineRegulator
    BaseTurbineRegulator::BaseTurbineRegulator() : BaseRegulator() {
    }

    // Реализация ExcitationRegulatorFactory
    std::unique_ptr<IExcitationRegulator> ExcitationRegulatorFactory::createRegulator(ExcitationRegulatorType type) {
        switch (type) {
        case ExcitationRegulatorType::CONSTANT:
            return std::make_unique<ConstantExcitationRegulator>();
        case ExcitationRegulatorType::IEEE_DC1A:
            return std::make_unique<IEEEDC1AExcitationRegulator>();
        case ExcitationRegulatorType::IEEE_AC1A:
            // Заглушка для будущей реализации
            throw PowerSystemException("IEEE AC1A excitation regulator not implemented yet");
        case ExcitationRegulatorType::CUSTOM:
            // Заглушка для пользовательской модели
            throw PowerSystemException("Custom excitation regulator not implemented yet");
        default:
            throw PowerSystemException("Unknown excitation regulator type");
        }
    }

    // Реализация TurbineRegulatorFactory
    std::unique_ptr<ITurbineRegulator> TurbineRegulatorFactory::createRegulator(TurbineRegulatorType type) {
        switch (type) {
        case TurbineRegulatorType::CONSTANT:
            return std::make_unique<ConstantPowerTurbineRegulator>();
        case TurbineRegulatorType::SPEED_GOVERNOR:
            return std::make_unique<SpeedGovernorTurbineRegulator>();
        case TurbineRegulatorType::CUSTOM:
            // Заглушка для пользовательской модели
            throw PowerSystemException("Custom turbine regulator not implemented yet");
        default:
            throw PowerSystemException("Unknown turbine regulator type");
        }
    }

} // namespace PowerSystem