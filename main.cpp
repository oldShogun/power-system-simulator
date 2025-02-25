/**
 * @file main.cpp
 * @brief Тестирование небольшой 4-узловой схемы с одним генератором
 * @author Claude
 * @date 25.02.2025
 */


#include "Core/PS_PowerSystemSimulator.hpp"
#include "Core/PS_CorePowerSystem.hpp"
#include "Utils/PS_Logger.hpp"
#include "Elements/PS_SynchronousGenerator.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <iomanip>

using namespace PowerSystem;

// Вспомогательная функция вывода результатов расчета
void printResults(const PowerFlowResults& results, const std::shared_ptr<CorePowerSystem>& system) {
    std::cout << "========= Power Flow Results =========" << std::endl;
    std::cout << "Converged: " << (results.converged ? "Yes" : "No") << std::endl;
    std::cout << "Iterations: " << results.iterations << std::endl;
    std::cout << "Max mismatch: " << results.maxMismatch << std::endl;

    std::cout << "\n--- Bus Voltages ---" << std::endl;
    std::cout << std::setw(8) << "Bus ID" << std::setw(12) << "Name" << std::setw(8) << "Type"
        << std::setw(12) << "Voltage(pu)" << std::setw(12) << "Angle(deg)"
        << std::setw(10) << "P(MW)" << std::setw(10) << "Q(MVAr)" << std::endl;

    for (const auto& busPair : system->getBuses()) {
        auto busId = busPair.first;
        auto& bus = busPair.second;
        std::string busType;

        switch (bus->getType()) {
        case BusType::SLACK:
            busType = "Slack";
            break;
        case BusType::PV:
            busType = "PV";
            break;
        case BusType::PQ:
            busType = "PQ";
            break;
        default:
            busType = "Unknown";
            break;
        }

        auto voltage = results.voltage.find(busId) != results.voltage.end() ?
            results.voltage.at(busId) : bus->getVoltage();
        auto power = results.power.find(busId) != results.power.end() ?
            results.power.at(busId) : bus->getPower();

        double vmag = std::abs(voltage);
        double vang = std::arg(voltage) * 180.0 / M_PI;  // Преобразуем в градусы
        double p = power.real() * 100.0;  // Преобразуем в МВт (базис 100 МВА)
        double q = power.imag() * 100.0;  // Преобразуем в МВАр

        std::cout << std::setw(8) << busId << std::setw(12) << bus->getName() << std::setw(8) << busType
            << std::setw(12) << std::fixed << std::setprecision(4) << vmag
            << std::setw(12) << std::fixed << std::setprecision(4) << vang
            << std::setw(10) << std::fixed << std::setprecision(2) << p
            << std::setw(10) << std::fixed << std::setprecision(2) << q << std::endl;
    }

    std::cout << "\n--- Branch Flows ---" << std::endl;
    std::cout << std::setw(10) << "Branch ID" << std::setw(8) << "From" << std::setw(8) << "To"
        << std::setw(10) << "P(MW)" << std::setw(10) << "Q(MVAr)"
        << std::setw(12) << "Current(%)" << std::endl;

    for (const auto& branchPair : system->getBranches()) {
        auto branchId = branchPair.first;
        auto& branch = branchPair.second;

        auto power = results.branchPower.find(branchId) != results.branchPower.end() ?
            results.branchPower.at(branchId) : branch->getPower();
        auto current = results.current.find(branchId) != results.current.end() ?
            results.current.at(branchId) : branch->getCurrent();

        double p = power.real() * 100.0;  // Преобразуем в МВт
        double q = power.imag() * 100.0;  // Преобразуем в МВАр
        double i = std::abs(current);
        double irated = branch->getRatedCurrent();
        double iPercent = irated > 0 ? (i / irated) * 100.0 : 0.0;

        std::cout << std::setw(10) << branchId << std::setw(8) << branch->getFromBusId()
            << std::setw(8) << branch->getToBusId()
            << std::setw(10) << std::fixed << std::setprecision(2) << p
            << std::setw(10) << std::fixed << std::setprecision(2) << q
            << std::setw(12) << std::fixed << std::setprecision(2) << iPercent << std::endl;
    }

    std::cout << "=====================================" << std::endl;
}

// Функция для создания тестовой схемы из 4 узлов и 1 генератора
std::shared_ptr<CorePowerSystem> createTestSystem(std::shared_ptr<Logger> logger) {
    auto system = std::make_shared<CorePowerSystem>("Test 4-Bus System", logger);

    // Создаем 4 узла
    auto bus1 = system->createBus("Bus 1", BusType::SLACK);  // Балансирующий узел
    auto bus2 = system->createBus("Bus 2", BusType::PQ);     // Нагрузочный узел
    auto bus3 = system->createBus("Bus 3", BusType::PQ);     // Нагрузочный узел
    auto bus4 = system->createBus("Bus 4", BusType::PV);     // Генераторный узел

    if (!bus1 || !bus2 || !bus3 || !bus4) {
        logger->error("Failed to create buses");
        return nullptr;
    }

    // Устанавливаем состояние узлов (ENABLED)
    bus1->setState(Types::ElementState::ENABLED);
    bus2->setState(Types::ElementState::ENABLED);
    bus3->setState(Types::ElementState::ENABLED);
    bus4->setState(Types::ElementState::ENABLED);

    // Устанавливаем напряжения в узлах
    bus1->setVoltage(Types::Complex(1.05, 0.0));  // 1.05 о.е., 0 градусов
    bus2->setVoltage(Types::Complex(1.0, 0.0));   // начальное приближение
    bus3->setVoltage(Types::Complex(1.0, 0.0));   // начальное приближение
    bus4->setVoltage(Types::Complex(1.03, 0.0));  // 1.03 о.е., уставка для PV-узла

    // Создаем ветви
    auto branch1 = system->createBranch("Branch 1-2", bus1->getId(), bus2->getId());
    auto branch2 = system->createBranch("Branch 1-3", bus1->getId(), bus3->getId());
    auto branch3 = system->createBranch("Branch 2-3", bus2->getId(), bus3->getId());
    auto branch4 = system->createBranch("Branch 2-4", bus2->getId(), bus4->getId());
    auto branch5 = system->createBranch("Branch 3-4", bus3->getId(), bus4->getId());

    if (!branch1 || !branch2 || !branch3 || !branch4 || !branch5) {
        logger->error("Failed to create branches");
        return nullptr;
    }

    // Устанавливаем параметры ветвей (сопротивления и проводимости)
    branch1->setImpedance(Types::Complex(0.05, 0.2));    // R=0.05, X=0.2 о.е.
    branch2->setImpedance(Types::Complex(0.08, 0.3));    // R=0.08, X=0.3 о.е.
    branch3->setImpedance(Types::Complex(0.05, 0.25));   // R=0.05, X=0.25 о.е.
    branch4->setImpedance(Types::Complex(0.1, 0.35));    // R=0.1, X=0.35 о.е.
    branch5->setImpedance(Types::Complex(0.12, 0.4));    // R=0.12, X=0.4 о.е.

    // Устанавливаем номинальные токи ветвей
    branch1->setRatedCurrent(1.5);
    branch2->setRatedCurrent(1.5);
    branch3->setRatedCurrent(1.2);
    branch4->setRatedCurrent(1.2);
    branch5->setRatedCurrent(1.2);

    // Устанавливаем состояние ветвей (ENABLED)
    branch1->setState(Types::ElementState::ENABLED);
    branch2->setState(Types::ElementState::ENABLED);
    branch3->setState(Types::ElementState::ENABLED);
    branch4->setState(Types::ElementState::ENABLED);
    branch5->setState(Types::ElementState::ENABLED);

    // Создаем нагрузки
    auto load1 = system->createLoad("Load 2", bus2->getId());
    auto load2 = system->createLoad("Load 3", bus3->getId());

    if (!load1 || !load2) {
        logger->error("Failed to create loads");
        return nullptr;
    }

    // Устанавливаем мощности нагрузок
    load1->setPower(Types::Complex(0.3, 0.1));  // P=30 МВт, Q=10 МВАр (базис 100 МВА)
    load2->setPower(Types::Complex(0.4, 0.15)); // P=40 МВт, Q=15 МВАр

    // Устанавливаем состояние нагрузок (ENABLED)
    load1->setState(Types::ElementState::ENABLED);
    load2->setState(Types::ElementState::ENABLED);

    // Создаем классический генератор в узле 4
    auto gen = system->createGenerator("Gen 4", bus4->getId());

    if (!gen) {
        logger->error("Failed to create generator");
        return nullptr;
    }

    // Устанавливаем параметры генератора
    gen->setActivePower(0.5);     // P=50 МВт (базис 100 МВА)
    gen->setVoltage(1.03);        // Уставка напряжения 1.03 о.е.
    gen->setMaxReactivePower(0.3); // Qmax=30 МВАр
    gen->setMinReactivePower(-0.2); // Qmin=-20 МВАр

    // Устанавливаем состояние генератора (ENABLED)
    gen->setState(Types::ElementState::ENABLED);

    // Создаем синхронный генератор в узле 4
    auto syncGen = system->createSynchronousGenerator(
        "SyncGen 4",
        bus4->getId(),
        GeneratorModelType::SIMPLIFIED   // Используем упрощенную модель для тестирования
    );

    if (syncGen) {
        // Настройка параметров синхронного генератора
        syncGen->setNominalPower(50.0);  // МВА
        syncGen->setNominalVoltage(10.5); // кВ

        // Электрические параметры
        syncGen->setElectricalParameters(
            1.8,    // xd
            1.7,    // xq
            0.3,    // xd1
            0.55,   // xq1
            0.25,   // xd2
            0.25,   // xq2
            0.0025  // ra
        );

        // Постоянные времени
        syncGen->setTimeConstants(
            8.0,    // Td0
            1.0,    // Tq0
            0.8,    // Td1
            0.4,    // Tq1
            0.035,  // Td2
            0.07    // Tq2
        );

        // Механические параметры
        syncGen->setMechanicalParameters(
            6.5,    // H - постоянная инерции
            2.0     // D - коэффициент демпфирования
        );

        // Установка регуляторов
        syncGen->setExcitationRegulator(ExcitationRegulatorType::IEEE_DC1A);
        syncGen->setTurbineRegulator(TurbineRegulatorType::SPEED_GOVERNOR);

        // Установка состояния генератора (включен)
        syncGen->setState(GeneratorState::NORMAL);

        logger->info("Synchronous generator created successfully");
    }
    else {
        logger->error("Failed to create synchronous generator");
    }

    logger->info("Test system created successfully");
    return system;
}

// Колбэк для отображения прогресса моделирования
void stepCallback(Types::TimeType time) {
    static int lastPercent = -1;
    int percent = static_cast<int>(time * 100);

    if (percent != lastPercent) {
        std::cout << "Simulation progress: " << percent << "%" << std::endl;
        lastPercent = percent;
    }
}

// Колбэк после расчета потокораспределения
void powerFlowCallback(PowerFlowResults& results) {
    std::cout << "Power flow calculated, iterations: " << results.iterations
        << ", max mismatch: " << results.maxMismatch << std::endl;
}

int main() {
    try {
        // Создаем логгер
        auto logger = std::make_shared<Logger>("power_system_test.log", Types::LogLevel::INFO, true);
        logger->info("Starting 4-bus test system simulation");

        // Создаем тестовую схему
        auto system = createTestSystem(logger);
        if (!system) {
            logger->error("Failed to create test system");
            std::cerr << "Failed to create test system. See log for details." << std::endl;
            return 1;
        }

        // Создаем и настраиваем симулятор
        SimulatorConfig config;
        config.mode = Types::SimulationMode::DYNAMIC;
        config.startTime = 0.0;
        config.endTime = 5.0;  // 5 секунд моделирования
        config.timeStep = 0.01;
        config.minTimeStep = 0.001;
        config.maxTimeStep = 0.1;
        config.useAdaptiveStep = true;
        config.usePredictorCorrector = true;
        config.maxIterations = 20;
        config.tolerance = 1e-6;
        config.convergenceTolerance = 1e-4;
        config.calculatePowerFlow = true;
        config.integrationMethod = IntegrationMethodType::RK4;
        config.powerFlowMethod = PowerFlowMethod::NEWTON_RAPHSON;

        auto simulator = std::make_shared<PowerSystemSimulator>(system, logger, config);

        // Устанавливаем колбэки
        simulator->setStepCallback(stepCallback);
        simulator->setPowerFlowCallback(powerFlowCallback);

        // Инициализируем симулятор
        logger->info("Initializing simulator");
        if (!simulator->initialize()) {
            logger->error("Failed to initialize simulator");
            std::cerr << "Failed to initialize simulator. See log for details." << std::endl;
            return 2;
        }

        // Получаем и выводим результаты начального расчета потокораспределения
        auto initialResults = simulator->getPowerFlowResults();
        printResults(initialResults, system);

        // Запускаем моделирование
        logger->info("Starting simulation");
        auto simulationStart = std::chrono::high_resolution_clock::now();

        if (!simulator->run()) {
            logger->error("Simulation failed");
            std::cerr << "Simulation failed. See log for details." << std::endl;
            return 3;
        }

        auto simulationEnd = std::chrono::high_resolution_clock::now();
        auto simulationTime = std::chrono::duration_cast<std::chrono::milliseconds>(simulationEnd - simulationStart);

        logger->info("Simulation completed in " + std::to_string(simulationTime.count()) + " ms");
        std::cout << "Simulation completed in " << simulationTime.count() << " ms" << std::endl;

        // Получаем и выводим результаты после моделирования
        auto finalResults = simulator->getPowerFlowResults();
        printResults(finalResults, system);

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
}