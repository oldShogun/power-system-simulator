/**
 * @file PS_AdaptiveIntegrator.cpp
 * @brief Реализация класса адаптивного интегрирования для энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_AdaptiveIntegrator.hpp"
#include "../Core/PS_Exception.hpp"
#include <cmath>
#include <algorithm>

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
        m_errorMethod(std::move(errorMethod)) {
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

        // Ограничение шага
        Types::TimeType h = std::min(m_maxStep, std::max(m_minStep, suggestedStep));
        bool stepAccepted = false;
        int reductions = 0;

        while (!stepAccepted && reductions < m_maxStepReductions) {
            // Расчет с основным методом интегрирования
            if (!computeFunc(currentTime, h)) {
                if (m_logger) m_logger->error("Computation failed with main method");
                return false;
            }

            // Расчет с методом для оценки погрешности
            if (!computeFunc(currentTime, h)) {
                if (m_logger) m_logger->error("Computation failed with error estimation method");
                return false;
            }

            // Вычисление локальной погрешности (должно быть реализовано в классе, использующем этот интегратор)
            // Здесь предполагается, что computeFunc вычисляет и оценивает погрешность
            double error = 0.0;  // Это должно быть вычислено в computeFunc

            // Проверка погрешности
            if (error <= m_tolerance) {
                stepAccepted = true;
            } else {
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

    void AdaptiveIntegrator::setLogger(std::shared_ptr<Logger> logger) {
        m_logger = logger;
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