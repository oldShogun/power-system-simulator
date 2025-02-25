/**
 * @file PS_IntegrationMethods.cpp
 * @brief Реализация методов интегрирования
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_IntegrationMethods.hpp"
#include "../Core/PS_Exception.hpp"
#include <cmath>

namespace PowerSystem {

    // Реализация BaseIntegrationMethod
    BaseIntegrationMethod::BaseIntegrationMethod()
        : m_stateSize(0), m_inputSize(0), m_initialized(false) {
    }

    bool BaseIntegrationMethod::initialize(int stateSize, int inputSize, DerivativeFunction derivFunc) {
        m_stateSize = stateSize;
        m_inputSize = inputSize;
        m_derivFunc = derivFunc;
        m_initialized = true;
        return true;
    }

    bool BaseIntegrationMethod::resizeBuffers(int stateSize, int inputSize) {
        m_stateSize = stateSize;
        m_inputSize = inputSize;
        return true;
    }

    bool BaseIntegrationMethod::reset() {
        m_initialized = false;
        return true;
    }

    // Реализация EulerMethod
    EulerMethod::EulerMethod() : BaseIntegrationMethod() {
        // Будет инициализирован в методе initialize
    }

    bool EulerMethod::step(
        std::vector<double>& state,
        const std::vector<double>& inputs,
        double dt) {

        if (!m_initialized || state.size() != m_stateSize || inputs.size() != m_inputSize) {
            return false;
        }

        // Расчет производных
        if (!m_derivFunc(state, inputs, m_derivatives)) {
            return false;
        }

        // Обновление состояния по формуле: y[n+1] = y[n] + h*f(y[n], t[n])
        for (int i = 0; i < m_stateSize; ++i) {
            state[i] += dt * m_derivatives[i];
        }

        return true;
    }

    double EulerMethod::getError(double dt) const {
        // Погрешность метода O(h^2)
        return dt * dt;
    }

    int EulerMethod::getOrder() const {
        return 1;
    }

    bool EulerMethod::resizeBuffers(int stateSize, int inputSize) {
        if (!BaseIntegrationMethod::resizeBuffers(stateSize, inputSize)) {
            return false;
        }

        try {
            m_derivatives.resize(stateSize);
            return true;
        }
        catch (const std::exception& e) {
            throw PowerSystemException("Failed to resize Euler method buffers: " + std::string(e.what()));
            return false;
        }
    }

    // Реализация RungeKutta4Method
    RungeKutta4Method::RungeKutta4Method() : BaseIntegrationMethod() {
        // Буферы инициализируются в методе initialize
    }

    bool RungeKutta4Method::initialize(int stateSize, int inputSize, DerivativeFunction derivFunc) {
        m_stateSize = stateSize;
        m_inputSize = inputSize;
        m_derivFunc = derivFunc;

        // Инициализация буферов с правильными размерами
        m_k1.resize(stateSize, 0.0);
        m_k2.resize(stateSize, 0.0);
        m_k3.resize(stateSize, 0.0);
        m_k4.resize(stateSize, 0.0);
        m_tempState.resize(stateSize, 0.0);

        m_initialized = true;
        return true;
    }

    bool RungeKutta4Method::step(
        std::vector<double>& state,
        const std::vector<double>& inputs,
        double dt) {

        if (!m_initialized || state.size() != m_stateSize || inputs.size() != m_inputSize) {
            return false;
        }

        // Проверка размеров буферов
        if (m_k1.size() != m_stateSize) m_k1.resize(m_stateSize, 0.0);
        if (m_k2.size() != m_stateSize) m_k2.resize(m_stateSize, 0.0);
        if (m_k3.size() != m_stateSize) m_k3.resize(m_stateSize, 0.0);
        if (m_k4.size() != m_stateSize) m_k4.resize(m_stateSize, 0.0);
        if (m_tempState.size() != m_stateSize) m_tempState.resize(m_stateSize, 0.0);


        // Шаг 1: Вычисляем k1 = f(t, y)
        if (!m_derivFunc(state, inputs, m_k1)) {
            return false;
        }

        // Шаг 2: Вычисляем k2 = f(t + dt/2, y + dt*k1/2)
        for (int i = 0; i < m_stateSize; ++i) {
            m_tempState[i] = state[i] + 0.5 * dt * m_k1[i];
        }
        if (!m_derivFunc(m_tempState, inputs, m_k2)) {
            return false;
        }

        // Шаг 3: Вычисляем k3 = f(t + dt/2, y + dt*k2/2)
        for (int i = 0; i < m_stateSize; ++i) {
            m_tempState[i] = state[i] + 0.5 * dt * m_k2[i];
        }
        if (!m_derivFunc(m_tempState, inputs, m_k3)) {
            return false;
        }

        // Шаг 4: Вычисляем k4 = f(t + dt, y + dt*k3)
        for (int i = 0; i < m_stateSize; ++i) {
            m_tempState[i] = state[i] + dt * m_k3[i];
        }
        if (!m_derivFunc(m_tempState, inputs, m_k4)) {
            return false;
        }

        // Шаг 5: Обновляем состояние y = y + dt*(k1 + 2*k2 + 2*k3 + k4)/6
        for (int i = 0; i < m_stateSize; ++i) {
            state[i] += dt * (m_k1[i] + 2.0 * m_k2[i] + 2.0 * m_k3[i] + m_k4[i]) / 6.0;
        }

        return true;
    }

    double RungeKutta4Method::getError(double dt) const {
        // Погрешность метода O(h^5)
        return std::pow(dt, 5);
    }

    int RungeKutta4Method::getOrder() const {
        return 4;
    }

    bool RungeKutta4Method::resizeBuffers(int stateSize, int inputSize) {
        if (!BaseIntegrationMethod::resizeBuffers(stateSize, inputSize)) {
            return false;
        }

        try {
            m_k1.resize(stateSize);
            m_k2.resize(stateSize);
            m_k3.resize(stateSize);
            m_k4.resize(stateSize);
            m_tempState.resize(stateSize);
            return true;
        }
        catch (const std::exception& e) {
            throw PowerSystemException("Failed to resize RK4 method buffers: " + std::string(e.what()));
            return false;
        }
    }

    // Реализация AdamsBashforthMoultonMethod
    AdamsBashforthMoultonMethod::AdamsBashforthMoultonMethod()
        : BaseIntegrationMethod(), m_historySize(4) {
        // Буферы инициализируются в методе initialize
    }

    bool AdamsBashforthMoultonMethod::initialize(int stateSize, int inputSize, DerivativeFunction derivFunc) {
        if (!BaseIntegrationMethod::initialize(stateSize, inputSize, derivFunc)) {
            return false;
        }

        // Сброс истории
        m_stateHistory.clear();
        m_derivativesHistory.clear();

        // Выделение памяти для буферов
        m_tempState.resize(stateSize);
        m_tempDerivatives.resize(stateSize);

        for (int i = 0; i < m_historySize; ++i) {
            m_stateHistory.push_back(std::vector<double>(stateSize, 0.0));
            m_derivativesHistory.push_back(std::vector<double>(stateSize, 0.0));
        }

        return true;
    }

    bool AdamsBashforthMoultonMethod::step(
        std::vector<double>& state,
        const std::vector<double>& inputs,
        double dt) {

        if (!m_initialized || state.size() != m_stateSize || inputs.size() != m_inputSize) {
            return false;
        }

        // Проверка, инициализирована ли история
        if (m_stateHistory.empty() || m_derivativesHistory.empty()) {
            return initializeHistory(state, inputs, dt);
        }

        // Шаг 1: Предиктор (Адамс-Башфорт)
        for (int i = 0; i < m_stateSize; ++i) {
            // y[n+1] = y[n] + h/24 * (55*f[n] - 59*f[n-1] + 37*f[n-2] - 9*f[n-3])
            m_tempState[i] = state[i] + dt / 24.0 * (
                55.0 * m_derivativesHistory[0][i] -
                59.0 * m_derivativesHistory[1][i] +
                37.0 * m_derivativesHistory[2][i] -
                9.0 * m_derivativesHistory[3][i]
                );
        }

        // Шаг 2: Вычисление производных для предсказанного состояния
        if (!m_derivFunc(m_tempState, inputs, m_tempDerivatives)) {
            return false;
        }

        // Шаг 3: Корректор (Адамс-Мультон)
        for (int i = 0; i < m_stateSize; ++i) {
            // y[n+1] = y[n] + h/24 * (9*f[n+1] + 19*f[n] - 5*f[n-1] + f[n-2])
            state[i] = state[i] + dt / 24.0 * (
                9.0 * m_tempDerivatives[i] +
                19.0 * m_derivativesHistory[0][i] -
                5.0 * m_derivativesHistory[1][i] +
                m_derivativesHistory[2][i]
                );
        }

        // Шаг 4: Обновление истории
        // Рассчитываем производные для скорректированного состояния
        if (!m_derivFunc(state, inputs, m_tempDerivatives)) {
            return false;
        }

        // Обновляем историю
        m_stateHistory.pop_back();
        m_derivativesHistory.pop_back();

        m_stateHistory.push_front(state);
        m_derivativesHistory.push_front(m_tempDerivatives);

        return true;
    }

    bool AdamsBashforthMoultonMethod::initializeHistory(
        std::vector<double>& state,
        const std::vector<double>& inputs,
        double dt) {

        // Используем метод Рунге-Кутта 4-го порядка для инициализации истории
        RungeKutta4Method rk4;
        if (!rk4.resizeBuffers(m_stateSize, m_inputSize) ||
            !rk4.initialize(m_stateSize, m_inputSize, m_derivFunc)) {
            return false;
        }

        // Очистка истории
        m_stateHistory.clear();
        m_derivativesHistory.clear();

        // Изначальное состояние
        std::vector<double> currentState = state;
        std::vector<double> derivatives(m_stateSize);

        // Добавляем текущее состояние и его производные
        if (!m_derivFunc(currentState, inputs, derivatives)) {
            return false;
        }

        m_stateHistory.push_back(currentState);
        m_derivativesHistory.push_back(derivatives);

        // Генерируем историю для трех предыдущих шагов
        for (int i = 1; i < m_historySize; ++i) {
            // Обратный шаг (отрицательный dt)
            if (!rk4.step(currentState, inputs, -dt)) {
                return false;
            }

            // Вычисляем производные для этого состояния
            if (!m_derivFunc(currentState, inputs, derivatives)) {
                return false;
            }

            // Добавляем в историю
            m_stateHistory.push_back(currentState);
            m_derivativesHistory.push_back(derivatives);
        }

        // Возвращаемся к начальному состоянию
        state = m_stateHistory.front();

        return true;
    }

    double AdamsBashforthMoultonMethod::getError(double dt) const {
        // Погрешность метода O(h^5)
        return std::pow(dt, 5);
    }

    int AdamsBashforthMoultonMethod::getOrder() const {
        return 4;
    }

    bool AdamsBashforthMoultonMethod::resizeBuffers(int stateSize, int inputSize) {
        if (!BaseIntegrationMethod::resizeBuffers(stateSize, inputSize)) {
            return false;
        }

        try {
            m_tempState.resize(stateSize);
            m_tempDerivatives.resize(stateSize);

            m_stateHistory.clear();
            m_derivativesHistory.clear();

            for (int i = 0; i < m_historySize; ++i) {
                m_stateHistory.push_back(std::vector<double>(stateSize, 0.0));
                m_derivativesHistory.push_back(std::vector<double>(stateSize, 0.0));
            }

            return true;
        }
        catch (const std::exception& e) {
            throw PowerSystemException("Failed to resize Adams method buffers: " + std::string(e.what()));
            return false;
        }
    }

    bool AdamsBashforthMoultonMethod::reset() {
        if (!BaseIntegrationMethod::reset()) {
            return false;
        }

        m_stateHistory.clear();
        m_derivativesHistory.clear();

        return true;
    }

    // Реализация IntegrationMethodFactory
    std::unique_ptr<IIntegrationMethod> IntegrationMethodFactory::createMethod(IntegrationMethodType type) {
        switch (type) {
        case IntegrationMethodType::EULER:
            return std::make_unique<EulerMethod>();
        case IntegrationMethodType::RK4:
            return std::make_unique<RungeKutta4Method>();
        case IntegrationMethodType::ADAMS_BASHFORTH:
            return std::make_unique<AdamsBashforthMoultonMethod>();
        default:
            throw PowerSystemException("Unknown integration method type");
        }
    }

} // namespace PowerSystem