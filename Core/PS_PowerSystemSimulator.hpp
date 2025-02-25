/**
 * @file PS_PowerSystemSimulator.hpp
 * @brief Основной симулятор энергосистемы с поддержкой адаптивного шага и итерационной увязки
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_POWER_SYSTEM_SIMULATOR_HPP
#define PS_POWER_SYSTEM_SIMULATOR_HPP

#include "PS_Types.hpp"
#include "PS_Exception.hpp"
#include "PS_CorePowerSystem.hpp"
#include "../Utils/PS_Logger.hpp"
#include "../PowerFlow/PS_PowerFlowSolver.hpp"
#include "../Integration/PS_AdaptiveIntegrator.hpp"
#include <memory>
#include <string>
#include <functional>

namespace PowerSystem {

    /**
     * @brief Конфигурация симулятора энергосистемы
     */
    struct SimulatorConfig {
        Types::SimulationMode mode;              ///< Режим моделирования
        Types::TimeType startTime;               ///< Время начала моделирования
        Types::TimeType endTime;                 ///< Время окончания моделирования
        Types::TimeType timeStep;                ///< Базовый шаг моделирования
        Types::TimeType minTimeStep;             ///< Минимальный шаг моделирования
        Types::TimeType maxTimeStep;             ///< Максимальный шаг моделирования
        bool useAdaptiveStep;                    ///< Использовать адаптивный шаг
        bool usePredictorCorrector;              ///< Использовать алгоритм предиктор-корректор
        int maxIterations;                       ///< Максимальное число итераций
        double tolerance;                        ///< Точность расчета
        double convergenceTolerance;             ///< Точность сходимости итераций
        bool calculatePowerFlow;                 ///< Расчет потокораспределения на каждом шаге
        IntegrationMethodType integrationMethod; ///< Метод интегрирования
        PowerFlowMethod powerFlowMethod;         ///< Метод расчета потокораспределения

        /**
         * @brief Конструктор с параметрами по умолчанию
         */
        SimulatorConfig()
            : mode(Types::SimulationMode::STEADY_STATE),
            startTime(0.0),
            endTime(1.0),
            timeStep(0.01),
            minTimeStep(0.0001),
            maxTimeStep(0.1),
            useAdaptiveStep(false),
            usePredictorCorrector(false),
            maxIterations(10),
            tolerance(1e-4),
            convergenceTolerance(1e-3),
            calculatePowerFlow(true),
            integrationMethod(IntegrationMethodType::RK4),
            powerFlowMethod(PowerFlowMethod::NEWTON_RAPHSON) {
        }
    };

    /**
     * @brief Основной класс симулятора энергосистемы
     */
    class PowerSystemSimulator {
    private:
        std::shared_ptr<CorePowerSystem> m_system;          ///< Моделируемая энергосистема
        std::shared_ptr<Logger> m_logger;                   ///< Логгер
        SimulatorConfig m_config;                           ///< Конфигурация симулятора
        Types::TimeType m_currentTime;                      ///< Текущее время моделирования
        bool m_initialized;                                 ///< Флаг инициализации симулятора
        bool m_running;                                     ///< Флаг работы симулятора

        // Решатели
        std::unique_ptr<IPowerFlowSolver> m_powerFlowSolver;    ///< Решатель потокораспределения
        std::unique_ptr<IAdaptiveIntegrator> m_adaptiveIntegrator; ///< Адаптивный интегратор

        // Колбэки для различных событий
        std::function<void(Types::TimeType)> m_stepCallback;      ///< Колбэк на каждом шаге моделирования
        std::function<void()> m_initCallback;                     ///< Колбэк после инициализации
        std::function<void()> m_finishCallback;                   ///< Колбэк после завершения моделирования
        std::function<void(PowerFlowResults&)> m_powerFlowCallback; ///< Колбэк после расчета потокораспределения

        // Рабочие данные
        PowerFlowResults m_powerFlowResults;                 ///< Результаты последнего расчета потокораспределения

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        PowerSystemSimulator();

        /**
         * @brief Конструктор с параметрами
         * @param system Указатель на энергосистему
         * @param logger Указатель на логгер
         * @param config Конфигурация симулятора
         */
        PowerSystemSimulator(std::shared_ptr<CorePowerSystem> system,
            std::shared_ptr<Logger> logger,
            const SimulatorConfig& config = SimulatorConfig());

        /**
         * @brief Деструктор
         */
        ~PowerSystemSimulator();

        /**
         * @brief Установить энергосистему для моделирования
         * @param system Указатель на энергосистему
         */
        void setSystem(std::shared_ptr<CorePowerSystem> system);

        /**
         * @brief Получить указатель на моделируемую энергосистему
         * @return Указатель на энергосистему
         */
        std::shared_ptr<CorePowerSystem> getSystem() const;

        /**
         * @brief Установить логгер
         * @param logger Указатель на логгер
         */
        void setLogger(std::shared_ptr<Logger> logger);

        /**
         * @brief Установить конфигурацию симулятора
         * @param config Конфигурация симулятора
         */
        void setConfig(const SimulatorConfig& config);

        /**
         * @brief Получить текущую конфигурацию симулятора
         * @return Конфигурация симулятора
         */
        SimulatorConfig getConfig() const;

        /**
         * @brief Получить текущее время моделирования
         * @return Текущее время
         */
        Types::TimeType getCurrentTime() const;

        /**
         * @brief Получить результаты последнего расчета потокораспределения
         * @return Результаты расчета потокораспределения
         */
        const PowerFlowResults& getPowerFlowResults() const;

        /**
         * @brief Установить колбэк для вызова на каждом шаге моделирования
         * @param callback Функция колбэка
         */
        void setStepCallback(std::function<void(Types::TimeType)> callback);

        /**
         * @brief Установить колбэк для вызова после инициализации
         * @param callback Функция колбэка
         */
        void setInitCallback(std::function<void()> callback);

        /**
         * @brief Установить колбэк для вызова после завершения моделирования
         * @param callback Функция колбэка
         */
        void setFinishCallback(std::function<void()> callback);

        /**
         * @brief Установить колбэк для вызова после расчета потокораспределения
         * @param callback Функция колбэка
         */
        void setPowerFlowCallback(std::function<void(PowerFlowResults&)> callback);

        /**
         * @brief Инициализировать симулятор
         * @return true в случае успеха, false в случае ошибки
         * @throw InitializationException при ошибках инициализации
         */
        bool initialize();

        /**
         * @brief Выполнить моделирование
         * @return true в случае успеха, false в случае ошибки
         * @throw ComputationException при ошибках моделирования
         */
        bool run();

        /**
         * @brief Выполнить один шаг моделирования
         * @return true в случае успеха, false в случае ошибки
         * @throw ComputationException при ошибках моделирования
         */
        bool step();

        /**
         * @brief Выполнить один шаг с итерационной увязкой
         * @param timeStep Шаг по времени
         * @return true в случае успеха, false в случае ошибки
         * @throw ComputationException при ошибках моделирования
         */
        bool iterativeCouplingStep(Types::TimeType timeStep);

        /**
         * @brief Выполнить расчет потокораспределения
         * @return true в случае успеха, false в случае ошибки
         * @throw ComputationException при ошибках расчета
         */
        bool solvePowerFlow();

        /**
         * @brief Остановить моделирование
         */
        void stop();

        /**
         * @brief Перезапустить моделирование
         * @return true в случае успеха, false в случае ошибки
         */
        bool reset();

        /**
         * @brief Проверка, инициализирован ли симулятор
         * @return true если симулятор инициализирован, false иначе
         */
        bool isInitialized() const;

        /**
         * @brief Проверка, запущено ли моделирование
         * @return true если моделирование запущено, false иначе
         */
        bool isRunning() const;

    private:
        /**
         * @brief Создать компоненты симулятора на основе конфигурации
         * @return true в случае успеха, false в случае ошибки
         */
        bool createComponents();

        /**
         * @brief Функция для адаптивного интегрирования
         * @param time Текущее время
         * @param timeStep Шаг по времени
         * @return true в случае успеха, false в случае ошибки
         */
        bool computeWithAdaptiveStep(Types::TimeType time, Types::TimeType timeStep);

        /**
         * @brief Выполнить один шаг с использованием алгоритма предиктор-корректор
         * @param timeStep Шаг по времени
         * @return true в случае успеха, false в случае ошибки
         */
        bool predictorCorrectorStep(Types::TimeType timeStep);
    };

} // namespace PowerSystem

#endif // PS_POWER_SYSTEM_SIMULATOR_HPP