/**
 * @file PS_PowerSystemSimulator.cpp
 * @brief Реализация основного симулятора энергосистемы с поддержкой адаптивного шага и итерационной увязки
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_PowerSystemSimulator.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <limits>

namespace PowerSystem {

    PowerSystemSimulator::PowerSystemSimulator()
        : m_system(nullptr),
        m_logger(nullptr),
        m_config(),
        m_currentTime(0.0),
        m_initialized(false),
        m_running(false),
        m_powerFlowSolver(nullptr),
        m_adaptiveIntegrator(nullptr),
        m_multiRateIntegrator(nullptr) {
    }

    PowerSystemSimulator::PowerSystemSimulator(
        std::shared_ptr<CorePowerSystem> system,
        std::shared_ptr<Logger> logger,
        const SimulatorConfig& config)
        : m_system(system),
        m_logger(logger),
        m_config(config),
        m_currentTime(config.startTime),
        m_initialized(false),
        m_running(false),
        m_powerFlowSolver(nullptr),
        m_adaptiveIntegrator(nullptr),
        m_multiRateIntegrator(nullptr) {

        // Автоматическое определение необходимости использования алгоритма предиктор-корректор
        // на основе наличия синхронных генераторов в системе
        if (system && !system->getSynchronousGenerators().empty() && !config.usePredictorCorrector) {
            m_config.usePredictorCorrector = true;
            if (m_logger) {
                m_logger->info("Predictor-corrector enabled automatically due to synchronous generators");
            }
        }
    }

    PowerSystemSimulator::~PowerSystemSimulator() {
        if (m_running) {
            stop();
        }
    }

    void PowerSystemSimulator::setSystem(std::shared_ptr<CorePowerSystem> system) {
        if (m_running) {
            throw InvalidParameterException("Cannot change system while simulation is running");
        }
        m_system = system;
        m_initialized = false;
    }

    std::shared_ptr<CorePowerSystem> PowerSystemSimulator::getSystem() const {
        return m_system;
    }

    void PowerSystemSimulator::setLogger(std::shared_ptr<Logger> logger) {
        m_logger = logger;

        // Обновляем логгеры в компонентах, если они уже созданы
        if (m_powerFlowSolver) {
            m_powerFlowSolver->setLogger(logger);
        }

        if (m_adaptiveIntegrator) {
            m_adaptiveIntegrator->setLogger(logger);
        }

        if (m_multiRateIntegrator) {
            m_multiRateIntegrator->setLogger(logger);
        }

        if (m_system) {
            m_system->setLogger(logger);
        }
    }

    void PowerSystemSimulator::setConfig(const SimulatorConfig& config) {
        if (m_running) {
            throw InvalidParameterException("Cannot change configuration while simulation is running");
        }

        // Проверка параметров конфигурации
        if (config.endTime <= config.startTime) {
            throw InvalidParameterException("End time must be greater than start time");
        }

        if (config.timeStep <= 0) {
            throw InvalidParameterException("Time step must be positive");
        }

        if (config.minTimeStep <= 0 || config.minTimeStep > config.timeStep) {
            throw InvalidParameterException("Minimum time step must be positive and not greater than time step");
        }

        if (config.maxTimeStep < config.timeStep) {
            throw InvalidParameterException("Maximum time step must not be less than time step");
        }

        m_config = config;
        m_initialized = false;
    }

    SimulatorConfig PowerSystemSimulator::getConfig() const {
        return m_config;
    }

    Types::TimeType PowerSystemSimulator::getCurrentTime() const {
        return m_currentTime;
    }

    const PowerFlowResults& PowerSystemSimulator::getPowerFlowResults() const {
        return m_powerFlowResults;
    }

    void PowerSystemSimulator::setStepCallback(std::function<void(Types::TimeType)> callback) {
        m_stepCallback = callback;
    }

    void PowerSystemSimulator::setInitCallback(std::function<void()> callback) {
        m_initCallback = callback;
    }

    void PowerSystemSimulator::setFinishCallback(std::function<void()> callback) {
        m_finishCallback = callback;
    }

    void PowerSystemSimulator::setPowerFlowCallback(std::function<void(PowerFlowResults&)> callback) {
        m_powerFlowCallback = callback;
    }

    void PowerSystemSimulator::addScheduledEvent(const ScheduledEvent& event) {
        m_scheduledEvents.push_back(event);

        // Сортируем события по времени
        std::sort(m_scheduledEvents.begin(), m_scheduledEvents.end(),
            [](const ScheduledEvent& a, const ScheduledEvent& b) {
                return a.time < b.time;
            });
    }

    bool PowerSystemSimulator::initialize() {
        if (!m_system) {
            throw InitializationException("Power system not set");
        }

        if (m_config.endTime <= m_config.startTime) {
            throw InitializationException("Invalid time parameters");
        }

        try {
            // Создание компонентов симулятора
            if (!createComponents()) {
                if (m_logger) {
                    m_logger->error("Failed to create simulator components");
                }
                return false;
            }

            // Инициализация энергосистемы
            if (!m_system->initialize()) {
                if (m_logger) {
                    m_logger->error("Failed to initialize power system");
                }
                return false;
            }

            // Начальный расчет потокораспределения, если требуется
            if (m_config.calculatePowerFlow) {
                if (!solvePowerFlow()) {
                    if (m_logger) {
                        m_logger->error("Failed to solve initial power flow");
                    }
                    return false;
                }
            }

            // Сброс времени моделирования
            m_currentTime = m_config.startTime;
            m_initialized = true;
            m_running = false;

            // Очистка истории состояний
            m_stateHistory.clear();

            // Инициализация истории состояний начальным состоянием
            saveSystemState();
            updateStateHistory();

            // Вызов колбэка после инициализации
            if (m_initCallback) {
                m_initCallback();
            }

            if (m_logger) {
                m_logger->info("Simulator initialized successfully");
            }

            return true;
        }
        catch (const std::exception& e) {
            if (m_logger) {
                m_logger->error("Initialization error: " + std::string(e.what()));
            }
            throw InitializationException(e.what());
        }
    }

    bool PowerSystemSimulator::createComponents() {
        // Создание решателя потокораспределения
        try {
            m_powerFlowSolver = PowerFlowSolverFactory::createSolver(m_config.powerFlowMethod);
            m_powerFlowSolver->setMaxIterations(m_config.maxIterations);
            m_powerFlowSolver->setTolerance(m_config.tolerance);
            m_powerFlowSolver->setLogger(m_logger);

            // Устанавливаем дополнительные параметры для оптимизации расчета Якоби
            if (auto nrSolver = dynamic_cast<NewtonRaphsonPowerFlowSolver*>(m_powerFlowSolver.get())) {
                nrSolver->setMaxJacobianReuse(m_config.maxJacobianReuse);
                nrSolver->setUsePartialUpdate(m_config.usePartialUpdateJacobian);
            }
        }
        catch (const std::exception& e) {
            if (m_logger) {
                m_logger->error("Failed to create power flow solver: " + std::string(e.what()));
            }
            return false;
        }

        // Создание адаптивного интегратора, если он нужен
        if (m_config.useAdaptiveStep) {
            try {
                // Используем метод высшего порядка и метод на 1 порядок ниже для оценки погрешности
                IntegrationMethodType lowerOrderMethod;

                switch (m_config.integrationMethod) {
                case IntegrationMethodType::RK4:
                    lowerOrderMethod = IntegrationMethodType::EULER;
                    break;
                case IntegrationMethodType::ADAMS_BASHFORTH:
                    lowerOrderMethod = IntegrationMethodType::RK4;
                    break;
                default:
                    lowerOrderMethod = IntegrationMethodType::EULER;
                    break;
                }

                m_adaptiveIntegrator = AdaptiveIntegratorFactory::createIntegrator(
                    m_config.integrationMethod, lowerOrderMethod);

                m_adaptiveIntegrator->setMinStep(m_config.minTimeStep);
                m_adaptiveIntegrator->setMaxStep(m_config.maxTimeStep);
                m_adaptiveIntegrator->setTolerance(m_config.tolerance);
                m_adaptiveIntegrator->setLogger(m_logger);

                // Устанавливаем дополнительные параметры для анализа изменений
                if (auto adaptiveIntegrator = dynamic_cast<AdaptiveIntegrator*>(m_adaptiveIntegrator.get())) {
                    adaptiveIntegrator->setRateThresholds(0.05, 0.5); // lowRate, highRate
                    adaptiveIntegrator->setScheduledEvents(m_scheduledEvents);
                }
            }
            catch (const std::exception& e) {
                if (m_logger) {
                    m_logger->error("Failed to create adaptive integrator: " + std::string(e.what()));
                }
                return false;
            }
        }

        // Создание многоскоростного интегратора, если он нужен
        if (m_config.useMultiRateIntegration) {
            try {
                m_multiRateIntegrator = std::make_unique<MultiRateIntegrator>(m_config.minTimeStep);
                m_multiRateIntegrator->setLogger(m_logger);

                // Создание групп компонентов для разных скоростей
                createComponentGroups();
            }
            catch (const std::exception& e) {
                if (m_logger) {
                    m_logger->error("Failed to create multi-rate integrator: " + std::string(e.what()));
                }
                return false;
            }
        }

        return true;
    }

    void PowerSystemSimulator::createComponentGroups() {
        // Реализация метода для создания групп компонентов с разными шагами
        if (!m_multiRateIntegrator) {
            return;
        }

        // Пример разделения компонентов на быстрые и медленные
        std::vector<int> fastComponents;
        std::vector<int> mediumComponents;
        std::vector<int> slowComponents;

        // Синхронные генераторы - быстрые компоненты
        int index = 0;
        for (const auto& genPair : m_system->getSynchronousGenerators()) {
            // В реальности нужна более сложная логика классификации компонентов
            // Здесь упрощенный пример
            auto& gen = genPair.second;

            // Все синхронные генераторы - быстрые компоненты
            fastComponents.push_back(index);
            index++;
        }

        // Ветви - средние компоненты
        index = 0;
        for (const auto& branchPair : m_system->getBranches()) {
            mediumComponents.push_back(index);
            index++;
        }

        // Нагрузки - медленные компоненты
        index = 0;
        for (const auto& loadPair : m_system->getLoads()) {
            slowComponents.push_back(index);
            index++;
        }

        // Добавляем группы компонентов в многоскоростной интегратор

        // Быстрые компоненты - мелкий шаг
        auto fastIntegrator = IntegrationMethodFactory::createMethod(IntegrationMethodType::RK4);
        m_multiRateIntegrator->addComponentGroup(fastComponents, m_config.minTimeStep, std::move(fastIntegrator));

        // Средние компоненты - средний шаг
        auto mediumIntegrator = IntegrationMethodFactory::createMethod(IntegrationMethodType::RK4);
        m_multiRateIntegrator->addComponentGroup(mediumComponents, m_config.timeStep / 2, std::move(mediumIntegrator));

        // Медленные компоненты - крупный шаг
        auto slowIntegrator = IntegrationMethodFactory::createMethod(IntegrationMethodType::EULER);
        m_multiRateIntegrator->addComponentGroup(slowComponents, m_config.timeStep, std::move(slowIntegrator));
    }

    bool PowerSystemSimulator::run() {
        if (!m_initialized) {
            if (!initialize()) {
                return false;
            }
        }

        if (m_running) {
            return true;
        }

        m_running = true;

        try {
            if (m_logger) {
                m_logger->info("Simulation started");
            }

            // Замеряем время выполнения
            auto startTime = std::chrono::high_resolution_clock::now();

            // Основной цикл моделирования
            while (m_running && m_currentTime < m_config.endTime) {
                // Обработка запланированных событий
                if (!processScheduledEvents(m_currentTime)) {
                    m_running = false;
                    if (m_logger) {
                        m_logger->error("Failed to process scheduled events");
                    }
                    return false;
                }

                if (!step()) {
                    m_running = false;
                    if (m_logger) {
                        m_logger->error("Simulation step failed");
                    }
                    return false;
                }
            }

            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

            m_running = false;

            // Вызов колбэка после завершения моделирования
            if (m_finishCallback) {
                m_finishCallback();
            }

            if (m_logger) {
                m_logger->info("Simulation completed successfully in " +
                    std::to_string(duration.count()) + " ms");
            }

            return true;
        }
        catch (const std::exception& e) {
            m_running = false;
            if (m_logger) {
                m_logger->error("Simulation error: " + std::string(e.what()));
            }
            throw ComputationException(e.what());
        }
    }

    bool PowerSystemSimulator::step() {
        if (!m_initialized) {
            throw ComputationException("Simulator not initialized");
        }

        if (m_currentTime >= m_config.endTime) {
            return false;
        }

        try {
            Types::TimeType timeStep = m_config.timeStep;
            Types::TimeType newTime = m_currentTime;

            // Ограничиваем шаг, чтобы не выйти за конечное время
            if (m_currentTime + timeStep > m_config.endTime) {
                timeStep = m_config.endTime - m_currentTime;
            }

            // Корректировка шага с учетом запланированных событий
            timeStep = adjustStepForEvents(m_currentTime, timeStep);

            // Используем адаптивный шаг, если он включен
            if (m_config.useAdaptiveStep && m_adaptiveIntegrator) {
                Types::TimeType actualStep;

                auto computeFunc = [this](Types::TimeType time, Types::TimeType dt) -> bool {
                    return this->computeWithAdaptiveStep(time, dt);
                    };

                if (!m_adaptiveIntegrator->step(computeFunc, m_currentTime, timeStep, newTime, actualStep)) {
                    if (m_logger) {
                        m_logger->error("Adaptive integration failed at time: " +
                            std::to_string(m_currentTime));
                    }
                    return false;
                }

                // Обновляем текущее время
                m_currentTime = newTime;
            }
            // Используем многоскоростное интегрирование
            else if (m_config.useMultiRateIntegration && m_multiRateIntegrator) {
                // Определение целевого времени
                Types::TimeType targetTime = m_currentTime + timeStep;

                if (!m_multiRateIntegrator->step(targetTime)) {
                    if (m_logger) {
                        m_logger->error("Multi-rate integration failed at time: " +
                            std::to_string(m_currentTime));
                    }
                    return false;
                }

                // Обновляем текущее время
                m_currentTime = targetTime;
            }
            else {
                // Обычный расчет с фиксированным шагом
                bool success = false;

                if (m_config.usePredictorCorrector) {
                    // Используем метод предиктор-корректор с итерационной увязкой
                    success = predictorCorrectorStep(timeStep);
                }
                else {
                    // Обычный расчет с итерационной увязкой
                    success = iterativeCouplingStep(timeStep);
                }

                if (!success) {
                    if (m_logger) {
                        m_logger->error("Computation failed at time: " +
                            std::to_string(m_currentTime));
                    }
                    return false;
                }

                // Обновляем текущее время
                m_currentTime += timeStep;
            }

            // Обновление истории состояний
            updateStateHistory();

            // Вызов колбэка шага
            if (m_stepCallback) {
                m_stepCallback(m_currentTime);
            }

            if (m_logger && m_currentTime >= m_config.endTime) {
                m_logger->info("Simulation reached end time: " +
                    std::to_string(m_config.endTime));
            }

            return true;
        }
        catch (const std::exception& e) {
            if (m_logger) {
                m_logger->error("Step error at time " +
                    std::to_string(m_currentTime) + ": " + std::string(e.what()));
            }
            throw ComputationException(e.what());
        }
    }

    bool PowerSystemSimulator::computeWithAdaptiveStep(Types::TimeType time, Types::TimeType timeStep) {
        // Этот метод вызывается адаптивным интегратором
        // Используем итерационную увязку или предиктор-корректор в зависимости от настроек

        bool success = false;

        if (m_config.usePredictorCorrector) {
            success = predictorCorrectorStep(timeStep);
        }
        else {
            success = iterativeCouplingStep(timeStep);
        }

        return success;
    }

    bool PowerSystemSimulator::iterativeCouplingStep(Types::TimeType timeStep) {
        if (!m_system) {
            throw ComputationException("Power system not set");
        }

        // Сохранение текущего состояния системы для возможного отката
        saveSystemState();

        // Итерационная увязка между генераторами и сетью
        int iteration = 0;
        bool converged = false;
        double maxMismatch = 0.0;
        double prevMaxMismatch = std::numeric_limits<double>::max();
        bool mismatches_diverging = false;

        // Создаем буферы для хранения предыдущих значений
        std::map<Types::ElementId, Types::Complex> prevVoltages;
        std::map<Types::ElementId, Types::Complex> prevCurrents;

        // Инициализация буферов начальными значениями
        for (const auto& busPair : m_system->getBuses()) {
            prevVoltages[busPair.first] = busPair.second->getVoltage();
        }
        for (const auto& genPair : m_system->getSynchronousGenerators()) {
            auto interface = genPair.second->getInterface();
            prevCurrents[genPair.first] = interface.current;
        }

        // Используем адаптивное управление точностью в зависимости от характера процесса
        double dynamicTolerance = m_config.convergenceTolerance;

        while (iteration < m_config.maxIterations && !converged && !mismatches_diverging) {
            // 1. Этап предиктора для генераторов
            if (!m_system->predictorStep(m_currentTime, timeStep)) {
                if (m_logger) {
                    m_logger->error("Generator dynamics computation failed at time: " +
                        std::to_string(m_currentTime));
                }
                // Откат к сохраненному состоянию
                restoreSystemState();
                // Уменьшение шага для следующей попытки
                m_config.timeStep = std::max(m_config.timeStep / 2.0, m_config.minTimeStep);
                return false;
            }

            // 2. Расчет сети с новыми инъекциями
            if (m_config.calculatePowerFlow) {
                if (!solvePowerFlow()) {
                    if (m_logger) {
                        m_logger->error("Power flow computation failed at time: " +
                            std::to_string(m_currentTime));
                    }
                    // Откат к сохраненному состоянию
                    restoreSystemState();
                    // Уменьшение шага для следующей попытки
                    m_config.timeStep = std::max(m_config.timeStep / 2.0, m_config.minTimeStep);
                    return false;
                }
            }

            // 3. Этап корректора для генераторов
            if (!m_system->correctorStep(m_currentTime, timeStep)) {
                if (m_logger) {
                    m_logger->error("Corrector step failed at time: " +
                        std::to_string(m_currentTime));
                }
                // Откат к сохраненному состоянию
                restoreSystemState();
                // Уменьшение шага для следующей попытки
                m_config.timeStep = std::max(m_config.timeStep / 2.0, m_config.minTimeStep);
                return false;
            }

            // 4. Оценка погрешности: проверка сходимости по напряжениям и токам
            maxMismatch = calculateMaxMismatch(prevVoltages, prevCurrents);

            // Проверка на расходимость процесса
            mismatches_diverging = (maxMismatch > prevMaxMismatch * 1.5 && iteration > 2);
            prevMaxMismatch = maxMismatch;

            // Динамическое управление точностью в зависимости от характера процесса
            if (isRapidChange()) {
                // Повышаем требуемую точность в моменты резких изменений режима
                dynamicTolerance = m_config.convergenceTolerance * 0.5;
            }
            else {
                // Снижаем требуемую точность при плавных процессах
                dynamicTolerance = m_config.convergenceTolerance * 1.5;
            }

            if (m_logger) {
                m_logger->debug("Iteration " + std::to_string(iteration) +
                    ", max mismatch = " + std::to_string(maxMismatch) +
                    ", tolerance = " + std::to_string(dynamicTolerance));
            }

            iteration++;

            // Проверка сходимости
            if (maxMismatch < dynamicTolerance) {
                converged = true;
            }
        }

        // Анализ результатов итерационного процесса
        if (converged) {
            // Процесс сошелся - принимаем результаты текущего шага
            if (m_logger) {
                m_logger->info("Iterative coupling converged in " +
                    std::to_string(iteration) + " iterations at time " +
                    std::to_string(m_currentTime));
            }

            // При хорошей сходимости увеличиваем следующий шаг
            if (iteration < m_config.maxIterations / 2 && !m_config.useAdaptiveStep) {
                m_config.timeStep = std::min(1.5 * m_config.timeStep, m_config.maxTimeStep);
            }

            // Проверяем энергетический баланс
            if (!checkEnergyBalance()) {
                if (m_logger) {
                    m_logger->warning("Energy balance check failed at time: " +
                        std::to_string(m_currentTime));
                }
            }

            return true;
        }
        else {
            // Процесс не сошелся или расходится
            if (m_logger) {
                m_logger->warning("Iterative coupling not converged at time: " +
                    std::to_string(m_currentTime) +
                    ", max mismatch = " + std::to_string(maxMismatch));
            }

            // Откат к сохраненному состоянию
            restoreSystemState();

            // Уменьшение шага для следующей попытки
            m_config.timeStep = std::max(m_config.timeStep / 2.0, m_config.minTimeStep);

            // Проверка минимального допустимого шага
            if (m_config.timeStep <= m_config.minTimeStep) {
                if (m_logger) {
                    m_logger->error("Simulation step reached minimum allowed size. Cannot continue.");
                }
                return false;
            }

            // Повторяем шаг с уменьшенным размером
            return iterativeCouplingStep(m_config.timeStep);
        }
    }

    bool PowerSystemSimulator::predictorCorrectorStep(Types::TimeType timeStep) {
        if (!m_system) {
            throw ComputationException("Power system not set");
        }

        // Фактически делегируем выполнение методу iterativeCouplingStep,
        // так как он уже реализует логику предиктор-корректор
        return iterativeCouplingStep(timeStep);
    }

    bool PowerSystemSimulator::solvePowerFlow() {
        if (!m_system || !m_powerFlowSolver) {
            return false;
        }

        try {
            // Выполняем расчет потокораспределения
            bool success = m_powerFlowSolver->solve(
                m_system->getBuses(),
                m_system->getBranches(),
                m_system->getGenerators(),
                m_system->getLoads(),
                m_powerFlowResults);

            // Вызов колбэка после расчета потокораспределения
            if (m_powerFlowCallback) {
                m_powerFlowCallback(m_powerFlowResults);
            }

            if (!success && m_logger) {
                m_logger->warning("Power flow solution did not converge");
            }

            return success;
        }
        catch (const std::exception& e) {
            if (m_logger) {
                m_logger->error("Power flow computation error: " + std::string(e.what()));
            }
            return false;
        }
    }

    void PowerSystemSimulator::stop() {
        if (m_running) {
            m_running = false;
            if (m_logger) {
                m_logger->info("Simulation stopped at time: " + std::to_string(m_currentTime));
            }
        }
    }

    bool PowerSystemSimulator::reset() {
        stop();
        m_currentTime = m_config.startTime;
        m_initialized = false;
        return initialize();
    }

    bool PowerSystemSimulator::isInitialized() const {
        return m_initialized;
    }

    bool PowerSystemSimulator::isRunning() const {
        return m_running;
    }

    void PowerSystemSimulator::saveSystemState() {
        // Сохранение текущего состояния системы для возможного отката
        m_savedState.time = m_currentTime;
        m_savedState.buses.clear();
        m_savedState.generators.clear();
        m_savedState.syncGenerators.clear();
        m_savedState.interfaces.clear();

        // Сохраняем состояние узлов
        for (const auto& busPair : m_system->getBuses()) {
            m_savedState.buses[busPair.first] = std::make_shared<Bus>(*busPair.second);
        }

        // Сохраняем состояние генераторов
        for (const auto& genPair : m_system->getGenerators()) {
            m_savedState.generators[genPair.first] = std::make_shared<Generator>(*genPair.second);
        }

        // Сохраняем состояние синхронных генераторов и их интерфейсы
        for (const auto& syncGenPair : m_system->getSynchronousGenerators()) {
            // В реальной реализации необходимо более сложное копирование
            m_savedState.syncGenerators[syncGenPair.first] = std::make_shared<SynchronousGenerator>(*syncGenPair.second);
            m_savedState.interfaces[syncGenPair.first] = syncGenPair.second->getInterface();
        }
    }

    void PowerSystemSimulator::restoreSystemState() {
        // Восстановление сохраненного состояния системы

        // Восстанавливаем состояние узлов
        for (const auto& busPair : m_savedState.buses) {
            auto bus = m_system->getBus(busPair.first);
            if (bus) {
                *bus = *busPair.second;
            }
        }

        // Восстанавливаем состояние генераторов
        for (const auto& genPair : m_savedState.generators) {
            auto gen = m_system->getGenerator(genPair.first);
            if (gen) {
                *gen = *genPair.second;
            }
        }

        // Восстанавливаем состояние синхронных генераторов
        for (const auto& syncGenPair : m_savedState.syncGenerators) {
            auto syncGen = m_system->getSynchronousGenerator(syncGenPair.first);
            if (syncGen) {
                // Восстановление состояния генератора
                *syncGen = *syncGenPair.second;

                // Восстановление интерфейса
                auto interfaceIt = m_savedState.interfaces.find(syncGenPair.first);
                if (interfaceIt != m_savedState.interfaces.end()) {
                    syncGen->updateInterface(interfaceIt->second);
                }
            }
        }
    }

    double PowerSystemSimulator::calculateMaxMismatch(
        std::map<Types::ElementId, Types::Complex>& prevVoltages,
        std::map<Types::ElementId, Types::Complex>& prevCurrents) {

        double maxMismatch = 0.0;

        // Проверка изменения напряжений
        for (const auto& busPair : m_system->getBuses()) {
            auto busId = busPair.first;
            auto& bus = busPair.second;

            auto prevVoltage = prevVoltages[busId];
            auto currentVoltage = bus->getVoltage();

            Types::Complex diff = currentVoltage - prevVoltage;
            double relativeDiff = std::abs(diff) / std::abs(prevVoltage);

            maxMismatch = std::max(maxMismatch, relativeDiff);

            // Обновляем значение для следующей итерации
            prevVoltages[busId] = currentVoltage;
        }

        // Проверка изменения токов генераторов
        for (const auto& genPair : m_system->getSynchronousGenerators()) {
            auto genId = genPair.first;
            auto& gen = genPair.second;

            auto interface = gen->getInterface();
            auto prevCurrent = prevCurrents[genId];
            auto currentCurrent = interface.current;

            Types::Complex diff = currentCurrent - prevCurrent;
            double relativeDiff = std::abs(diff) / (std::abs(prevCurrent) + 1e-10); // Избегаем деления на ноль

            maxMismatch = std::max(maxMismatch, relativeDiff);

            // Обновляем значение для следующей итерации
            prevCurrents[genId] = currentCurrent;
        }

        return maxMismatch;
    }

    bool PowerSystemSimulator::isRapidChange() {
        // Определение резких изменений режима на основе анализа
        // скорости изменения переменных состояния (углов роторов и т.д.)

        double maxRateOfChange = 0.0;

        for (const auto& genPair : m_system->getSynchronousGenerators()) {
            auto& gen = genPair.second;
            auto interface = gen->getInterface();

            // Считаем изменение угла ротора за последний шаг
            if (!m_stateHistory.empty()) {
                auto stateHistoryIt = std::find_if(m_stateHistory.begin(), m_stateHistory.end(),
                    [&genPair](const SystemState& state) {
                        return state.interfaces.find(genPair.first) != state.interfaces.end();
                    });

                if (stateHistoryIt != m_stateHistory.end()) {
                    auto& prevInterface = stateHistoryIt->interfaces.at(genPair.first);
                    double deltaTime = m_currentTime - stateHistoryIt->time;

                    if (deltaTime > 0) {
                        double rateOfChange = std::abs(interface.rotorAngle - prevInterface.rotorAngle) / deltaTime;
                        maxRateOfChange = std::max(maxRateOfChange, rateOfChange);
                    }
                }
            }
        }

        // Считаем, что изменения быстрые, если скорость выше порогового значения
        return maxRateOfChange > m_config.rapidChangeThreshold;
    }

    bool PowerSystemSimulator::checkEnergyBalance() {
        // Проверка баланса мощностей
        double totalGeneration = 0.0;
        double totalLoad = 0.0;
        double totalLosses = 0.0;

        // Суммируем генерацию
        for (const auto& genPair : m_system->getGenerators()) {
            totalGeneration += genPair.second->getActivePower();
        }

        for (const auto& syncGenPair : m_system->getSynchronousGenerators()) {
            auto interface = syncGenPair.second->getInterface();
            totalGeneration += interface.activePower;
        }

        // Суммируем нагрузку
        for (const auto& loadPair : m_system->getLoads()) {
            totalLoad += loadPair.second->getPower().real();
        }

        // Рассчитываем потери в ветвях
        for (const auto& branchPair : m_system->getBranches()) {
            auto& branch = branchPair.second;
            auto current = branch->getCurrent();
            auto impedance = branch->getImpedance();

            // P_loss = I^2 * R
            totalLosses += std::norm(current) * impedance.real();
        }

        // Проверяем баланс: P_gen = P_load + P_loss
        double imbalance = std::abs(totalGeneration - (totalLoad + totalLosses));
        double relativeMismatch = imbalance / (totalGeneration + 1e-10); // Избегаем деления на ноль

        // Возвращаем true, если дисбаланс меньше допустимого
        return relativeMismatch < m_config.energyBalanceTolerance;
    }

    bool PowerSystemSimulator::processScheduledEvents(Types::TimeType time) {
        bool eventsProcessed = false;

        // Обрабатываем события, запланированные на текущее время
        auto it = m_scheduledEvents.begin();
        while (it != m_scheduledEvents.end()) {
            if (it->time <= time) {
                // Обработка события
                if (m_logger) {
                    m_logger->info("Processing scheduled event: " + it->type +
                        " for element " + std::to_string(it->elementId) +
                        " at time " + std::to_string(time));
                }

                // Реализация обработки различных типов событий
                if (it->type == "BranchOutage") {
                    // Отключение ветви
                    auto branch = m_system->getBranch(it->elementId);
                    if (branch) {
                        branch->setState(Types::ElementState::DISABLED);
                        eventsProcessed = true;
                    }
                }
                else if (it->type == "GeneratorOutage") {
                    // Отключение генератора
                    auto gen = m_system->getGenerator(it->elementId);
                    if (gen) {
                        gen->setState(Types::ElementState::DISABLED);
                        eventsProcessed = true;
                    }

                    auto syncGen = m_system->getSynchronousGenerator(it->elementId);
                    if (syncGen) {
                        syncGen->setState(GeneratorState::OFFLINE);
                        eventsProcessed = true;
                    }
                }
                else if (it->type == "LoadChange") {
                    // Изменение нагрузки
                    auto load = m_system->getLoad(it->elementId);
                    if (load) {
                        // Используем параметр 'scale' для масштабирования нагрузки
                        auto scaleIt = it->params.find("scale");
                        if (scaleIt != it->params.end()) {
                            double scale = scaleIt->second;
                            Types::Complex currentPower = load->getPower();
                            load->setPower(currentPower * scale);
                            eventsProcessed = true;
                        }
                    }
                }
                else if (it->type == "SetVoltage") {
                    // Изменение уставки напряжения генератора
                    auto gen = m_system->getGenerator(it->elementId);
                    if (gen) {
                        auto voltageIt = it->params.find("voltage");
                        if (voltageIt != it->params.end()) {
                            gen->setVoltage(voltageIt->second);
                            eventsProcessed = true;
                        }
                    }
                }
                // Добавьте обработку других типов событий по необходимости

                // Удаляем обработанное событие
                it = m_scheduledEvents.erase(it);
            }
            else {
                ++it;
            }
        }

        return true;
    }

    Types::TimeType PowerSystemSimulator::adjustStepForEvents(Types::TimeType currentTime, Types::TimeType suggestedStep) {
        // Выбор шага интегрирования с учётом запланированных событий
        Types::TimeType adjustedStep = suggestedStep;

        // Ищем ближайшее событие в пределах предлагаемого шага
        Types::TimeType nearestEventTime = currentTime + suggestedStep;
        bool hasNearEvent = false;

        for (const auto& event : m_scheduledEvents) {
            if (event.time > currentTime && event.time < nearestEventTime) {
                nearestEventTime = event.time;
                hasNearEvent = true;
            }
        }

        // Если найдено событие в пределах шага, корректируем шаг
        if (hasNearEvent) {
            // Шаг до события минус небольшой запас
            adjustedStep = nearestEventTime - currentTime - 1e-6;

            if (m_logger) {
                m_logger->info("Adjusting step size from " + std::to_string(suggestedStep) +
                    " to " + std::to_string(adjustedStep) +
                    " due to upcoming event at " + std::to_string(nearestEventTime));
            }
        }

        // Особая обработка для событий, которые требуют предварительного уменьшения шага
        // Например, тяжелые возмущения (КЗ и т.п.)
        for (const auto& event : m_scheduledEvents) {
            // Если до события осталось меньше 2 шагов, уменьшаем шаг для лучшей точности
            if (event.time > currentTime &&
                event.time < currentTime + 2 * suggestedStep &&
                (event.type == "Fault" || event.type == "BranchOutage" || event.type == "GeneratorOutage")) {

                // Уменьшаем шаг для более точного моделирования перед возмущением
                adjustedStep = std::min(adjustedStep, (event.time - currentTime) / 4);

                if (m_logger) {
                    m_logger->info("Reducing step size to " + std::to_string(adjustedStep) +
                        " in preparation for major disturbance");
                }
            }
        }

        // Обеспечиваем, что шаг не меньше минимального и не больше максимального
        return std::min(std::max(adjustedStep, m_config.minTimeStep), m_config.maxTimeStep);
    }

    void PowerSystemSimulator::updateStateHistory() {
        // Сохраняем текущее состояние в историю
        SystemState currentState;
        currentState.time = m_currentTime;

        // Сохраняем только интерфейсы генераторов для экономии памяти
        for (const auto& syncGenPair : m_system->getSynchronousGenerators()) {
            currentState.interfaces[syncGenPair.first] = syncGenPair.second->getInterface();
        }

        // Добавляем состояние в историю
        m_stateHistory.push_back(currentState);

        // Ограничиваем размер истории
        const size_t MAX_HISTORY_SIZE = 100;  // Максимальное количество сохраняемых состояний
        while (m_stateHistory.size() > MAX_HISTORY_SIZE) {
            m_stateHistory.pop_front();
        }
    }

} // namespace PowerSystem