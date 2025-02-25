/**
 * @file PS_MultiRateIntegrator.hpp
 * @brief Класс многоскоростного интегрирования для компонентов с разными частотными характеристиками
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_MULTI_RATE_INTEGRATOR_HPP
#define PS_MULTI_RATE_INTEGRATOR_HPP

#include "../Core/PS_Types.hpp"
#include "../Integration/PS_IntegrationMethods.hpp"
#include "../Utils/PS_Logger.hpp"
#include <memory>
#include <vector>
#include <functional>

namespace PowerSystem {

    /**
     * @brief Интерфейс многоскоростного интегратора
     */
    class IMultiRateIntegrator {
    public:
        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IMultiRateIntegrator() = default;

        /**
         * @brief Выполнить шаг до указанного времени
         * @param targetTime Целевое время
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool step(Types::TimeType targetTime) = 0;

        /**
         * @brief Добавить группу компонентов с собственным шагом и методом интегрирования
         * @param componentIndices Индексы компонентов
         * @param stepSize Размер шага для группы
         * @param integrator Метод интегрирования
         */
        virtual void addComponentGroup(
            const std::vector<int>& componentIndices,
            Types::TimeType stepSize,
            std::unique_ptr<IIntegrationMethod> integrator) = 0;

        /**
         * @brief Установить логгер
         * @param logger Указатель на логгер
         */
        virtual void setLogger(std::shared_ptr<Logger> logger) = 0;
    };

    /**
     * @brief Класс многоскоростного интегрирования
     */
    class MultiRateIntegrator : public IMultiRateIntegrator {
    private:
        /**
         * @brief Структура группы компонентов
         */
        struct ComponentGroup {
            std::vector<int> componentIndices;          ///< Индексы компонентов
            Types::TimeType stepSize;                    ///< Размер шага
            std::unique_ptr<IIntegrationMethod> integrator; ///< Метод интегрирования
            Types::TimeType nextStepTime;                ///< Время следующего шага
        };
        
        std::shared_ptr<Logger> m_logger;               ///< Логгер
        std::vector<ComponentGroup> m_groups;           ///< Группы компонентов
        Types::TimeType m_minStep;                      ///< Минимальный шаг
        Types::TimeType m_currentTime;                  ///< Текущее время
        
    public:
        /**
         * @brief Конструктор
         * @param minStep Минимальный шаг
         */
        MultiRateIntegrator(Types::TimeType minStep);
        
        /**
         * @brief Деструктор
         */
        ~MultiRateIntegrator() override = default;
        
        /**
         * @brief Выполнить шаг до указанного времени
         * @param targetTime Целевое время
         * @return true в случае успеха, false в случае ошибки
         */
        bool step(Types::TimeType targetTime) override;
        
        /**
         * @brief Добавить группу компонентов с собственным шагом и методом интегрирования
         * @param componentIndices Индексы компонентов
         * @param stepSize Размер шага для группы
         * @param integrator Метод интегрирования
         */
        void addComponentGroup(
            const std::vector<int>& componentIndices,
            Types::TimeType stepSize,
            std::unique_ptr<IIntegrationMethod> integrator) override;
        
        /**
         * @brief Установить логгер
         * @param logger Указатель на логгер
         */
        void setLogger(std::shared_ptr<Logger> logger) override;
        
    private:
        /**
         * @brief Выполнить шаг для группы компонентов
         * @param group Группа компонентов
         * @param step Размер шага
         * @return true в случае успеха, false в случае ошибки
         */
        bool stepGroup(ComponentGroup& group, Types::TimeType step);
    };

} // namespace PowerSystem

#endif // PS_MULTI_RATE_INTEGRATOR_HPP