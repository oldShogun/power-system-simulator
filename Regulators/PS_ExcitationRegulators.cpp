/**
 * @file PS_ExcitationRegulators.cpp
 * @brief Реализация регуляторов возбуждения
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_ExcitationRegulators.hpp"
#include "../Core/PS_Exception.hpp"
#include <cmath>
#include <algorithm>

namespace PowerSystem {

    // Реализация ConstantExcitationRegulator
    ConstantExcitationRegulator::ConstantExcitationRegulator() : BaseExcitationRegulator() {
        // Параметры по умолчанию
        m_parameters["excitation"] = 1.0;      // Постоянное напряжение возбуждения, о.е.
    }

    bool ConstantExcitationRegulator::initialize() {
        const std::vector<std::string> requiredParams = { "excitation" };

        if (!checkRequiredParameters(requiredParams)) {
            return false;
        }

        m_initialized = true;
        return true;
    }

    bool ConstantExcitationRegulator::step(
        const std::vector<double>& inputs,
        std::vector<double>& outputs,
        double dt) {

        if (!m_initialized || outputs.size() != getOutputSize()) {
            return false;
        }

        // Просто возвращаем постоянное напряжение возбуждения
        outputs[0] = m_parameters["excitation"];

        return true;
    }

    int ConstantExcitationRegulator::getInputSize() const {
        return 2;  // Напряжение генератора, уставка напряжения
    }

    int ConstantExcitationRegulator::getOutputSize() const {
        return 1;  // Напряжение возбуждения
    }

    std::string ConstantExcitationRegulator::getType() const {
        return "Constant Excitation Regulator";
    }

    ExcitationRegulatorType ConstantExcitationRegulator::getExcitationType() const {
        return ExcitationRegulatorType::CONSTANT;
    }

    bool ConstantExcitationRegulator::reset() {
        // Вызов базового метода
        if (!BaseRegulator::reset()) {
            return false;
        }

        // Специфичная для класса логика сброса состояния
        m_parameters["excitation"] = 1.0;

        return true;
    }

    // Реализация IEEEDC1AExcitationRegulator
    IEEEDC1AExcitationRegulator::IEEEDC1AExcitationRegulator()
        : BaseExcitationRegulator(), m_vref(1.0), m_vr(0.0), m_efd(0.0) {

        // Параметры по умолчанию по стандарту IEEE DC1A
        m_parameters["Ka"] = 40.0;       // Коэффициент усиления регулятора напряжения
        m_parameters["Ta"] = 0.02;       // Постоянная времени регулятора напряжения
        m_parameters["Ke"] = 1.0;        // Коэффициент возбудителя
        m_parameters["Te"] = 0.8;        // Постоянная времени возбудителя
        m_parameters["Kf"] = 0.03;       // Коэффициент стабилизации
        m_parameters["Tf"] = 1.0;        // Постоянная времени стабилизации
        m_parameters["Vrmin"] = -8.5;    // Минимальный выход регулятора
        m_parameters["Vrmax"] = 8.5;     // Максимальный выход регулятора
        m_parameters["E1"] = 3.1;        // Точка насыщения 1 по напряжению
        m_parameters["SE1"] = 0.33;      // Значение насыщения в точке 1
        m_parameters["E2"] = 2.3;        // Точка насыщения 2 по напряжению
        m_parameters["SE2"] = 0.1;       // Значение насыщения в точке 2

        // Инициализация внутреннего состояния
        m_state.resize(3, 0.0);          // Vr, Vf, Efd
    }

    bool IEEEDC1AExcitationRegulator::initialize() {
        const std::vector<std::string> requiredParams = {
            "Ka", "Ta", "Ke", "Te", "Kf", "Tf", "Vrmin", "Vrmax"
        };

        if (!checkRequiredParameters(requiredParams)) {
            return false;
        }

        // Инициализация начальных значений
        m_vr = 0.0;
        m_efd = m_vref / m_parameters["Ke"];

        // Инициализация состояния
        m_state[0] = m_vr;           // Выход регулятора напряжения
        m_state[1] = 0.0;            // Сигнал обратной связи
        m_state[2] = m_efd;          // Напряжение возбуждения

        m_initialized = true;
        return true;
    }

    bool IEEEDC1AExcitationRegulator::step(
        const std::vector<double>& inputs,
        std::vector<double>& outputs,
        double dt) {

        if (!m_initialized || inputs.size() != getInputSize() || outputs.size() != getOutputSize()) {
            return false;
        }

        // Извлечение входных переменных
        double vt = inputs[0];        // Напряжение на выводах генератора
        double ifd = inputs[1];       // Ток возбуждения

        // Извлечение параметров
        double Ka = m_parameters["Ka"];
        double Ta = m_parameters["Ta"];
        double Ke = m_parameters["Ke"];
        double Te = m_parameters["Te"];
        double Kf = m_parameters["Kf"];
        double Tf = m_parameters["Tf"];
        double Vrmin = m_parameters["Vrmin"];
        double Vrmax = m_parameters["Vrmax"];

        // Текущее состояние
        double vr = m_state[0];      // Выход регулятора напряжения
        double vf = m_state[1];      // Сигнал обратной связи
        double efd = m_state[2];     // Напряжение возбуждения

        // Расчет ошибки напряжения
        double error = m_vref - vt - vf;

        // Динамические уравнения системы

        // 1. Регулятор напряжения: dVr/dt = (Ka*error - Vr) / Ta
        double dvr_dt = (Ka * error - vr) / Ta;
        // Ограничение выхода регулятора
        vr += dvr_dt * dt;
        vr = std::max(Vrmin, std::min(vr, Vrmax));

        // 2. Контур обратной связи: dVf/dt = (Kf*dEfd/dt - Vf) / Tf
        double dvf_dt = (Kf * (efd - m_efd) / dt - vf) / Tf;
        vf += dvf_dt * dt;

        // 3. Возбудитель: dEfd/dt = (Vr - Ke*Efd) / Te
        double defd_dt = (vr - Ke * efd) / Te;
        efd += defd_dt * dt;

        // Обновление состояния
        m_state[0] = vr;
        m_state[1] = vf;
        m_state[2] = efd;

        // Запоминание текущего значения для следующего шага
        m_efd = efd;

        // Выход регулятора
        outputs[0] = efd;

        return true;
    }

    int IEEEDC1AExcitationRegulator::getInputSize() const {
        return 2;  // Напряжение генератора, ток возбуждения
    }

    int IEEEDC1AExcitationRegulator::getOutputSize() const {
        return 1;  // Напряжение возбуждения
    }

    std::string IEEEDC1AExcitationRegulator::getType() const {
        return "IEEE DC1A Excitation Regulator";
    }

    ExcitationRegulatorType IEEEDC1AExcitationRegulator::getExcitationType() const {
        return ExcitationRegulatorType::IEEE_DC1A;
    }

    void IEEEDC1AExcitationRegulator::setVoltageSetpoint(double vref) {
        m_vref = vref;
    }

    bool IEEEDC1AExcitationRegulator::reset() {
        // Вызов базового метода
        if (!BaseRegulator::reset()) {
            return false;
        }

        // Специфичная для класса логика сброса состояния
        m_vref = 1.0;
        m_vr = 0.0;
        m_efd = 0.0;

        // Сброс внутреннего состояния
        m_state[0] = m_vr;
        m_state[1] = 0.0;
        m_state[2] = m_efd;

        return true;
    }

} // namespace PowerSystem