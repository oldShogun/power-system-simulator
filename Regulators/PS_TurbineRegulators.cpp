/**
 * @file PS_TurbineRegulators.cpp
 * @brief Реализация регуляторов турбины
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_TurbineRegulators.hpp"
#include "../Core/PS_Exception.hpp"
#include <cmath>
#include <algorithm>

namespace PowerSystem {

    // Реализация ConstantPowerTurbineRegulator
    ConstantPowerTurbineRegulator::ConstantPowerTurbineRegulator() : BaseTurbineRegulator() {
        // Параметры по умолчанию
        m_parameters["power"] = 1.0;      // Постоянная механическая мощность, о.е.
    }

    bool ConstantPowerTurbineRegulator::initialize() {
        const std::vector<std::string> requiredParams = { "power" };

        if (!checkRequiredParameters(requiredParams)) {
            return false;
        }

        m_initialized = true;
        return true;
    }

    bool ConstantPowerTurbineRegulator::step(
        const std::vector<double>& inputs,
        std::vector<double>& outputs,
        double dt) {

        if (!m_initialized || outputs.size() != getOutputSize()) {
            return false;
        }

        // Просто возвращаем постоянную механическую мощность
        outputs[0] = m_parameters["power"];

        return true;
    }

    int ConstantPowerTurbineRegulator::getInputSize() const {
        return 2;  // Частота, уставка мощности
    }

    int ConstantPowerTurbineRegulator::getOutputSize() const {
        return 1;  // Механическая мощность
    }

    std::string ConstantPowerTurbineRegulator::getType() const {
        return "Constant Power Turbine Regulator";
    }

    TurbineRegulatorType ConstantPowerTurbineRegulator::getTurbineType() const {
        return TurbineRegulatorType::CONSTANT;
    }

    bool ConstantPowerTurbineRegulator::reset() {
        // Вызов базового метода
        if (!BaseRegulator::reset()) {
            return false;
        }

        // Специфичная для класса логика сброса состояния
        m_parameters["power"] = 1.0;

        return true;
    }

    // Реализация SpeedGovernorTurbineRegulator
    SpeedGovernorTurbineRegulator::SpeedGovernorTurbineRegulator()
        : BaseTurbineRegulator(), m_pSetpoint(1.0), m_omega0(1.0), m_pm(0.0), m_pv(0.0) {

        // Параметры по умолчанию
        m_parameters["R"] = 0.05;          // Статизм (5%)
        m_parameters["Tg"] = 0.2;          // Постоянная времени регулятора
        m_parameters["Tch"] = 0.5;         // Постоянная времени турбины
        m_parameters["Pmax"] = 1.1;        // Максимальная мощность
        m_parameters["Pmin"] = 0.0;        // Минимальная мощность

        // Инициализация внутреннего состояния
        m_state.resize(2, 0.0);            // PV (положение клапана), PM (механическая мощность)
    }

    bool SpeedGovernorTurbineRegulator::initialize() {
        const std::vector<std::string> requiredParams = {
            "R", "Tg", "Tch", "Pmax", "Pmin"
        };

        if (!checkRequiredParameters(requiredParams)) {
            return false;
        }

        // Инициализация начальных значений
        m_pv = m_pSetpoint;
        m_pm = m_pSetpoint;

        // Инициализация состояния
        m_state[0] = m_pv;           // Положение клапана
        m_state[1] = m_pm;           // Механическая мощность

        m_initialized = true;
        return true;
    }

    bool SpeedGovernorTurbineRegulator::step(
        const std::vector<double>& inputs,
        std::vector<double>& outputs,
        double dt) {

        if (!m_initialized || inputs.size() != getInputSize() || outputs.size() != getOutputSize()) {
            return false;
        }

        // Извлечение входных переменных
        double omega = inputs[0];        // Частота ротора
        double pref = inputs[1];         // Уставка мощности (может отличаться от m_pSetpoint)

        // Извлечение параметров
        double R = m_parameters["R"];          // Статизм
        double Tg = m_parameters["Tg"];        // Постоянная времени регулятора
        double Tch = m_parameters["Tch"];      // Постоянная времени турбины
        double Pmax = m_parameters["Pmax"];    // Максимальная мощность
        double Pmin = m_parameters["Pmin"];    // Минимальная мощность

        // Текущее состояние
        double pv = m_state[0];          // Положение клапана
        double pm = m_state[1];          // Механическая мощность

        // Расчет ошибки частоты
        double perror = pref - (omega - m_omega0) / R;

        // Динамические уравнения системы

        // 1. Регулятор скорости: dPV/dt = (Perror - PV) / Tg
        double dpv_dt = (perror - pv) / Tg;
        // Ограничение положения клапана
        pv += dpv_dt * dt;
        pv = std::max(Pmin, std::min(pv, Pmax));

        // 2. Модель турбины: dPm/dt = (PV - Pm) / Tch
        double dpm_dt = (pv - pm) / Tch;
        pm += dpm_dt * dt;

        // Обновление состояния
        m_state[0] = pv;
        m_state[1] = pm;

        // Запоминание значений для следующего шага
        m_pv = pv;
        m_pm = pm;

        // Выход регулятора
        outputs[0] = pm;

        return true;
    }

    int SpeedGovernorTurbineRegulator::getInputSize() const {
        return 2;  // Частота, уставка мощности
    }

    int SpeedGovernorTurbineRegulator::getOutputSize() const {
        return 1;  // Механическая мощность
    }

    std::string SpeedGovernorTurbineRegulator::getType() const {
        return "Speed Governor Turbine Regulator";
    }

    TurbineRegulatorType SpeedGovernorTurbineRegulator::getTurbineType() const {
        return TurbineRegulatorType::SPEED_GOVERNOR;
    }

    void SpeedGovernorTurbineRegulator::setPowerSetpoint(double pSetpoint) {
        m_pSetpoint = pSetpoint;
    }

    bool SpeedGovernorTurbineRegulator::reset() {
        // Вызов базового метода
        if (!BaseRegulator::reset()) {
            return false;
        }

        // Специфичная для класса логика сброса состояния
        m_pv = m_pSetpoint;
        m_pm = m_pSetpoint;

        // Сброс внутреннего состояния
        m_state[0] = m_pv;
        m_state[1] = m_pm;

        return true;
    }

} // namespace PowerSystem