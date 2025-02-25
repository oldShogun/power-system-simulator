/**
 * @file PS_AdaptiveIntegrator.cpp
 * @brief Реализация класса адаптивного интегрирования для энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_AdaptiveIntegrator.hpp"
#include "../Core/PS_Exception.hpp"
#include "../Core/PS_PowerSystemSimulator.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace PowerSystem {

    AdaptiveIntegrator::AdaptiveIntegrator(
        std::unique_ptr<IIntegrationMethod> method,
        std::unique_ptr<IIntegrationMethod> errorMethod)
        : m_logger(nullptr),
        m_minStep(1e-6),
        m_maxStep(0.1),
        m_tolerance(1e-4),
        m_safetyFactor(0.9),
        m_maxStepReductions(10),
        m_method(std::move(method)),
        m_errorMethod(std::move(errorMethod)),
        m_nextStepScale(1.0),
        m_lowRateThreshold(0.05),
        m_highRateThreshold(0.5) {
    }

    bool AdaptiveIntegrator::step(
        ComputeFunction computeFunc,
        Types::TimeType currentTime,
        Types::TimeType suggestedStep,
        Types::TimeType& newTime,
        Types::TimeType& actualStep) {

        // Проверка входных данных
        if (!m_method || !m_errorMethod) {
            if (m_logger) m_logger->error("Integration methods not set");
            return false;
        }

        if (suggestedStep <= 0) {
            if (m_logger) m_logger->error("Suggested time step must be positive");
            return false;
        }

        // Анализ характера изменения переменных состояния для выбора шага
        Types::TimeType h = analyzeStateDerivatives(suggestedStep);

        // Ограничение шага
        h = std::min(m_maxStep, std::max(m_minStep, h));

        // Учет запланированных событий
        h = adjustStepForScheduledEvents(currentTime, h);

        bool stepAccepted = false;
        int reductions = 0;
        double error = 0.0;

        while (!stepAccepted && reductions < m_maxStepReductions) {
            // Сохранение текущего состояния для возможной отмены шага
            if (!savePreviousState()) {
                if (m_logger) m_logger->error("Failed to save system state");
                return false;
            }

            // Расчет с основным методом интегрирования
            if (!computeFunc(currentTime, h)) {
                if (m_logger) m_logger->error("Computation failed with main method");
                return false;
            }

            // Восстановление предыдущего состояния
            if (!restorePreviousState()) {
                if (m_logger) m_logger->error("Failed to restore system state");
                return false;
            }

            // Расчет с методом для оценки погрешности
            if (!m_errorMethod->step(m_state, m_inputs, h)) {
                if (m_logger) m_logger->error("Error estimation step failed");
                return false;
            }

            // Расчет с основным методом
            if (!m_method->step(m_state, m_inputs, h)) {
                if (m_logger) m_logger->error("Main integration step failed");
                return false;
            }

            // Вычисление локальной погрешности методом Ричардсона
            error = estimateError();

            // Проверка погрешности
            if (error <= m_tolerance) {
                stepAccepted = true;

                // При очень малой погрешности можно увеличить следующий шаг
                if (error < m_tolerance * 0.1) {
                    m_nextStepScale = 1.5;
                }
                else {
                    // Иначе выбираем масштаб на основе погрешности
                    m_nextStepScale = m_safetyFactor * std::pow(m_tolerance / error, 1.0 / (m_method->getOrder() + 1));
                    m_nextStepScale = std::min(m_nextStepScale, 1.5); // Ограничиваем рост шага
                }
            }
            else {
                // Уменьшение шага
                h *= m_safetyFactor * std::pow(m_tolerance / error, 1.0 / (m_method->getOrder() + 1));
                h = std::max(h, m_minStep);
                reductions++;

                if (m_logger) {
                    m_logger->debug("Step size reduced to " + std::to_string(h) +
                        " due to error " + std::to_string(error));
                }
            }
        }

        // Если шаг был принят или достигнуто максимальное число уменьшений
        if (stepAccepted || reductions >= m_maxStepReductions) {
            newTime = currentTime + h;
            actualStep = h;

            if (!stepAccepted && m_logger) {
                m_logger->warning("Maximum step reductions reached, continuing with step " +
                    std::to_string(h));
            }

            return stepAccepted;
        }

        // В случае проблем
        if (m_logger) m_logger->error("Failed to find suitable step size");
        return false;
    }

    void AdaptiveIntegrator::setMinStep(Types::TimeType minStep) {
        if (minStep <= 0) {
            throw InvalidParameterException("Minimum step must be positive");
        }
        m_minStep = minStep;
    }

    void AdaptiveIntegrator::setMaxStep(Types::TimeType maxStep) {
        if (maxStep <= 0) {
            throw InvalidParameterException("Maximum step must be positive");
        }
        m_maxStep = maxStep;
    }

    void AdaptiveIntegrator::setTolerance(double tolerance) {
        if (tolerance <= 0) {
            throw InvalidParameterException("Tolerance must be positive");
        }
        m_tolerance = tolerance;
    }

    void AdaptiveIntegrator::setSafetyFactor(double safetyFactor) {
        if (safetyFactor <= 0 || safetyFactor >= 1) {
            throw InvalidParameterException("Safety factor must be in range (0, 1)");
        }
        m_safetyFactor = safetyFactor;
    }

    void AdaptiveIntegrator::setMaxStepReductions(int maxReductions) {
        if (maxReductions <= 0) {
            throw InvalidParameterException("Maximum step reductions must be positive");
        }
        m_maxStepReductions = maxReductions;
    }

    void AdaptiveIntegrator::setRateThresholds(double lowRate, double highRate) {
        if (lowRate < 0 || highRate < 0 || lowRate >= highRate) {
            throw InvalidParameterException("Rate thresholds must be positive and low < high");
        }
        m_lowRateThreshold = lowRate;
        m_highRateThreshold = highRate;
    }

    void AdaptiveIntegrator::setScheduledEvents(const std::vector<ScheduledEvent>& events) {
        m_scheduledEvents = events;
    }

    void AdaptiveIntegrator::setLogger(std::shared_ptr<Logger> logger) {
        m_logger = logger;
    }

    Types::TimeType AdaptiveIntegrator::analyzeStateDerivatives(Types::TimeType suggestedStep) {
        // Анализ скорости изменения переменных состояния
        if (m_derivatives.empty() || m_prevDerivatives.empty()) {
            return suggestedStep; // Недостаточно данных для анализа
        }

        double maxRate = 0.0;

        // Находим максимальную скорость изменения производных
        for (size_t i = 0; i < m_derivatives.size(); i++) {
            double denom = std::abs(m_prevDerivatives[i]) + 1e-10; // Избегаем деления на ноль
            double rate = std::abs(m_derivatives[i] - m_prevDerivatives[i]) / denom;
            maxRate = std::max(maxRate, rate);
        }

        // Адаптируем шаг в зависимости от скорости изменения
        if (maxRate > m_highRateThreshold) {
            // Быстрые изменения - уменьшаем шаг
            return suggestedStep * 0.5;
        }
        else if (maxRate < m_lowRateThreshold) {
            // Медленные изменения - увеличиваем шаг
            return suggestedStep * 1.5;
        }

        return suggestedStep;
    }

    Types::TimeType AdaptiveIntegrator::adjustStepForScheduledEvents(Types::TimeType currentTime, Types::TimeType step) {
        // Проверка запланированных событий на интервале [currentTime, currentTime + step]
        Types::TimeType nearestEventTime = std::numeric_limits<Types::TimeType>::max();

        for (const auto& event : m_scheduledEvents) {
            if (event.time > currentTime && event.time < currentTime + step) {
                nearestEventTime = std::min(nearestEventTime, event.time);
            }
        }

        // Если есть ближайшее событие, адаптируем шаг, чтобы не пропустить его
        if (nearestEventTime < std::numeric_limits<Types::TimeType>::max()) {
            // Добавляем небольшой запас для точного попадания на событие
            return nearestEventTime - currentTime - 1e-6;
        }

        // Особая обработка для событий, которые требуют предварительного уменьшения шага
        for (const auto& event : m_scheduledEvents) {
            // Если до события осталось меньше 2 шагов и это важное событие, уменьшаем шаг
            if (event.time > currentTime &&
                event.time < currentTime + 2 * step &&
                (event.type == "Fault" || event.type == "BranchOutage" || event.type == "GeneratorOutage")) {

                // Уменьшаем шаг для более точного моделирования перед возмущением
                return std::min(step, (event.time - currentTime) / 4);
            }
        }

        return step;
    }

    double AdaptiveIntegrator::estimateError() {
        double maxError = 0.0;

        // Оценка погрешности методом Ричардсона
        if (!m_errorState.empty() && m_errorState.size() == m_state.size()) {
            for (size_t i = 0; i < m_state.size(); i++) {
                double localError = std::abs(m_state[i] - m_errorState[i]);
                double relativeError = localError / (std::abs(m_state[i]) + 1e-10);
                maxError = std::max(maxError, relativeError);
            }
        }

        return maxError;
    }

    bool AdaptiveIntegrator::savePreviousState() {
        // Сохранение предыдущего состояния
        // В реальной реализации здесь должно быть сохранение состояния системы

        return true;
    }

    bool AdaptiveIntegrator::restorePreviousState() {
        // Восстановление предыдущего состояния
        // В реальной реализации здесь должно быть восстановление состояния системы

        return true;
    }

    std::unique_ptr<IAdaptiveIntegrator> AdaptiveIntegratorFactory::createIntegrator(
        IntegrationMethodType highOrderMethod,
        IntegrationMethodType lowOrderMethod) {

        // Проверка, что метод высокого порядка имеет больший порядок, чем метод низкого порядка
        auto highMethod = IntegrationMethodFactory::createMethod(highOrderMethod);
        auto lowMethod = IntegrationMethodFactory::createMethod(lowOrderMethod);

        if (highMethod->getOrder() <= lowMethod->getOrder()) {
            throw InvalidParameterException("High order method must have higher order than low order method");
        }

        return std::make_unique<AdaptiveIntegrator>(std::move(highMethod), std::move(lowMethod));
    }

} // namespace PowerSystem