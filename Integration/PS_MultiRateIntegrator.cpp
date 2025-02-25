/**
 * @file PS_MultiRateIntegrator.cpp
 * @brief Реализация класса многоскоростного интегрирования для компонентов с разными частотными характеристиками
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_MultiRateIntegrator.hpp"
#include "../Core/PS_Exception.hpp"
#include <algorithm>

namespace PowerSystem {

    MultiRateIntegrator::MultiRateIntegrator(Types::TimeType minStep)
        : m_logger(nullptr),
        m_minStep(minStep),
        m_currentTime(0.0) {
    }

    bool MultiRateIntegrator::step(Types::TimeType targetTime) {
        if (targetTime <= m_currentTime) {
            if (m_logger) {
                m_logger->error("Target time must be greater than current time");
            }
            return false;
        }

        if (m_groups.empty()) {
            if (m_logger) {
                m_logger->warning("No component groups added to multi-rate integrator");
            }
            m_currentTime = targetTime;
            return true;
        }

        // Интегрирование до целевого времени
        while (m_currentTime < targetTime) {
            // Определение следующего времени для всех групп
            Types::TimeType nextStepTime = targetTime;

            // Находим ближайшее время шага среди всех групп
            for (auto& group : m_groups) {
                nextStepTime = std::min(nextStepTime, group.nextStepTime);
            }

            // Если ближайшее время - целевое время, то выполняем шаг для всех групп
            if (std::abs(nextStepTime - targetTime) < 1e-10) {
                nextStepTime = targetTime;
            }

            // Выполнение шагов для групп, которым нужно обновиться
            bool success = true;
            for (auto& group : m_groups) {
                if (std::abs(group.nextStepTime - nextStepTime) < 1e-10) {
                    Types::TimeType step = nextStepTime - m_currentTime;
                    if (!stepGroup(group, step)) {
                        success = false;
                        if (m_logger) {
                            m_logger->error("Failed to step component group");
                        }
                    }
                    
                    // Обновление следующего времени для этой группы
                    group.nextStepTime = nextStepTime + group.stepSize;
                }
            }

            if (!success) {
                return false;
            }

            // Обновление текущего времени
            m_currentTime = nextStepTime;
        }

        return true;
    }

    void MultiRateIntegrator::addComponentGroup(
        const std::vector<int>& componentIndices,
        Types::TimeType stepSize,
        std::unique_ptr<IIntegrationMethod> integrator) {
        
        // Проверка параметров
        if (stepSize < m_minStep) {
            if (m_logger) {
                m_logger->warning("Step size is less than minimum step, using minimum step instead");
            }
            stepSize = m_minStep;
        }

        // Создание и добавление новой группы
        ComponentGroup group;
        group.componentIndices = componentIndices;
        group.stepSize = stepSize;
        group.integrator = std::move(integrator);
        group.nextStepTime = m_currentTime + stepSize;

        m_groups.push_back(std::move(group));

        if (m_logger) {
            m_logger->info("Component group added with step size " + std::to_string(stepSize) +
                ", components count: " + std::to_string(componentIndices.size()));
        }
    }

    void MultiRateIntegrator::setLogger(std::shared_ptr<Logger> logger) {
        m_logger = logger;
    }

    bool MultiRateIntegrator::stepGroup(ComponentGroup& group, Types::TimeType step) {
        try {
            // Здесь должна быть реализация шага для конкретной группы компонентов
            // В полной реализации нужно:
            // 1. Собрать состояние компонентов этой группы
            // 2. Сформировать векторы состояния и входов
            // 3. Вызвать метод шага для интегратора группы
            // 4. Обновить состояние компонентов

            // В данной реализации просто предполагаем, что шаг выполнен успешно
            if (m_logger) {
                m_logger->debug("Stepping component group with " + 
                    std::to_string(group.componentIndices.size()) + 
                    " components, step size: " + std::to_string(step));
            }

            // В реальной реализации здесь будет вызов интегратора группы:
            // return group.integrator->step(stateVector, inputVector, step);

            return true;
        }
        catch (const std::exception& e) {
            if (m_logger) {
                m_logger->error("Exception in step group: " + std::string(e.what()));
            }
            return false;
        }
    }

} // namespace PowerSystem