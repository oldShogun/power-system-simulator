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

namespace PowerSystem {

    PowerSystemSimulator::PowerSystemSimulator()
        : m_system(nullptr),
        m_logger(nullptr),
        m_config(),
        m_currentTime(0.0),
        m_initialized(false),
        m_running(false),
        m_powerFlowSolver(nullptr),
        m_adaptiveIntegrator(nullptr) {
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
        m_adaptiveIntegrator(nullptr) {

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
            }
            catch (const std::exception& e) {
                if (m_logger) {
                    m_logger->error("Failed to create adaptive integrator: " + std::string(e.what()));
                }
                return false;
            }
        }

        return true;
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

        // Итерационная увязка между генераторами и сетью
        int iteration = 0;
        bool converged = false;
        double maxMismatch = 0.0;

        // Сохраняем состояние сети перед началом итераций
        std::map<Types::ElementId, Types::Complex> prevVoltages;
        for (const auto& busPair : m_system->getBuses()) {
            prevVoltages[busPair.first] = busPair.second->getVoltage();
        }

        while (iteration < m_config.maxIterations && !converged) {
            // 1. Расчет динамики генераторов
            if (!m_system->compute(m_currentTime, timeStep)) {
                if (m_logger) {
                    m_logger->error("Generator dynamics computation failed at time: " +
                        std::to_string(m_currentTime));
                }
                return false;
            }

            // 2. Расчет потокораспределения
            if (m_config.calculatePowerFlow) {
                if (!solvePowerFlow()) {
                    if (m_logger) {
                        m_logger->error("Power flow computation failed at time: " +
                            std::to_string(m_currentTime));
                    }
                    return false;
                }
            }

            // 3. Проверка сходимости: максимальное изменение напряжений
            maxMismatch = 0.0;
            for (const auto& busPair : m_system->getBuses()) {
                Types::ElementId busId = busPair.first;
                Types::Complex voltage = busPair.second->getVoltage();

                auto it = prevVoltages.find(busId);
                if (it != prevVoltages.end()) {
                    Types::Complex diff = voltage - it->second;
                    double absDiff = std::abs(diff);
                    maxMismatch = std::max(maxMismatch, absDiff);
                    // Обновляем предыдущее значение
                    prevVoltages[busId] = voltage;
                }
            }

            iteration++;

            if (m_logger) {
                m_logger->debug("Iteration " + std::to_string(iteration) +
                    ", max voltage mismatch = " + std::to_string(maxMismatch));
            }

            if (maxMismatch < m_config.convergenceTolerance) {
                converged = true;
            }
        }

        if (!converged && m_logger) {
            m_logger->warning("Iterative coupling not converged at time: " +
                std::to_string(m_currentTime) +
                ", max mismatch = " + std::to_string(maxMismatch));
        }

        return true; // Даже если не сошлось, продолжаем расчет
    }

    bool PowerSystemSimulator::predictorCorrectorStep(Types::TimeType timeStep) {
        if (!m_system) {
            throw ComputationException("Power system not set");
        }

        // 1. Шаг предиктора: расчет промежуточного состояния генераторов
        if (!m_system->predictorStep(m_currentTime, timeStep)) {
            if (m_logger) {
                m_logger->error("Predictor step failed at time: " +
                    std::to_string(m_currentTime));
            }
            return false;
        }

        // 2. Расчет потокораспределения
        if (m_config.calculatePowerFlow) {
            if (!solvePowerFlow()) {
                if (m_logger) {
                    m_logger->error("Power flow computation failed at time: " +
                        std::to_string(m_currentTime));
                }
                return false;
            }
        }

        // 3. Шаг корректора: уточнение состояния генераторов
        if (!m_system->correctorStep(m_currentTime, timeStep)) {
            if (m_logger) {
                m_logger->error("Corrector step failed at time: " +
                    std::to_string(m_currentTime));
            }
            return false;
        }

        return true;
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

} // namespace PowerSystem