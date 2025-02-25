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
#include "../Integration/PS_MultiRateIntegrator.hpp"
#include <memory>
#include <string>
#include <functional>
#include <vector>
#include <deque>

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

        // Новые параметры
        bool useMultiRateIntegration;            ///< Использовать многоскоростное интегрирование
        bool useParallelComputing;               ///< Использовать параллельные вычисления
        int maxJacobianReuse;                    ///< Максимальное число повторных использований Якоби
        bool usePartialUpdateJacobian;           ///< Использовать частичное обновление Якоби
        double energyBalanceTolerance;           ///< Допустимая погрешность в энергетическом балансе
        double rapidChangeThreshold;             ///< Порог для определения быстрых изменений
        int numThreads;                          ///< Количество потоков для параллельных вычислений

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
            powerFlowMethod(PowerFlowMethod::NEWTON_RAPHSON),
            // Новые параметры
            useMultiRateIntegration(false),
            useParallelComputing(false),
            maxJacobianReuse(3),
            usePartialUpdateJacobian(false),
            energyBalanceTolerance(1e-3),
            rapidChangeThreshold(0.1),
            numThreads(4) {
        }
    };

    /**
     * @brief Структура для сохранения состояния системы
     */
    struct SystemState {
        Types::TimeType time;                                              ///< Время снимка состояния
        std::map<Types::ElementId, std::shared_ptr<Bus>> buses;           ///< Копии узлов
        std::map<Types::ElementId, std::shared_ptr<Generator>> generators; ///< Копии генераторов
        std::map<Types::ElementId, std::shared_ptr<SynchronousGenerator>> syncGenerators; ///< Копии синхронных генераторов
        std::map<Types::ElementId, GeneratorInterface> interfaces;        ///< Копии интерфейсов генераторов
    };

    /**
     * @brief Структура для события в системе
     */
    struct ScheduledEvent {
        Types::TimeType time;                ///< Время события
        std::string type;                    ///< Тип события
        Types::ElementId elementId;          ///< ID элемента
        std::map<std::string, double> params; ///< Параметры события
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
        std::unique_ptr<IMultiRateIntegrator> m_multiRateIntegrator; ///< Многоскоростной интегратор

        // Колбэки для различных событий
        std::function<void(Types::TimeType)> m_stepCallback;      ///< Колбэк на каждом шаге моделирования
        std::function<void()> m_initCallback;                     ///< Колбэк после инициализации
        std::function<void()> m_finishCallback;                   ///< Колбэк после завершения моделирования
        std::function<void(PowerFlowResults&)> m_powerFlowCallback; ///< Колбэк после расчета потокораспределения

        // Рабочие данные
        PowerFlowResults m_powerFlowResults;                  ///< Результаты последнего расчета потокораспределения
        SystemState m_savedState;                             ///< Сохраненное состояние для отката
        std::deque<SystemState> m_stateHistory;               ///< История состояний системы
        std::vector<ScheduledEvent> m_scheduledEvents;        ///< Запланированные события

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
         * @brief Добавить запланированное событие
         * @param event Событие для добавления в расписание
         */
        void addScheduledEvent(const ScheduledEvent& event);

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

        /**
         * @brief Сохранить текущее состояние системы для возможного отката
         */
        void saveSystemState();

        /**
         * @brief Восстановить сохраненное состояние системы
         */
        void restoreSystemState();

        /**
         * @brief Вычислить максимальную невязку по напряжениям и токам
         * @param prevVoltages Предыдущие значения напряжений
         * @param prevCurrents Предыдущие значения токов
         * @return Максимальная невязка
         */
        double calculateMaxMismatch(
            std::map<Types::ElementId, Types::Complex>& prevVoltages,
            std::map<Types::ElementId, Types::Complex>& prevCurrents);

        /**
         * @brief Определить, происходят ли быстрые изменения в системе
         * @return true если происходят быстрые изменения, false иначе
         */
        bool isRapidChange();

        /**
         * @brief Проверить энергетический баланс системы
         * @return true если баланс в пределах допустимой погрешности, false иначе
         */
        bool checkEnergyBalance();

        /**
         * @brief Обработать запланированные события
         * @param time Текущее время
         * @return true в случае успеха, false в случае ошибки
         */
        bool processScheduledEvents(Types::TimeType time);

        /**
         * @brief Выбрать шаг интегрирования на основе запланированных событий
         * @param currentTime Текущее время
         * @param suggestedStep Предлагаемый шаг
         * @return Скорректированный шаг
         */
        Types::TimeType adjustStepForEvents(Types::TimeType currentTime, Types::TimeType suggestedStep);

        /**
         * @brief Обновить историю состояний системы
         */
        void updateStateHistory();
    };

} // namespace PowerSystem

#endif // PS_POWER_SYSTEM_SIMULATOR_HPP