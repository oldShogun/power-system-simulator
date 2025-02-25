/**
 * @file main.cpp
 * @brief Тестирование загрузки и расчета тестовой схемы IEEE14
 * @author Claude
 * @date 25.02.2025
 */

#include "Core/PS_PowerSystemSimulator.hpp"
#include "Core/PS_CorePowerSystem.hpp"
#include "Utils/PS_Logger.hpp"
#include "Import/PS_CdfImporter.hpp"
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

    std::cout << "\n--- PV to PQ Bus Conversions ---" << std::endl;
    if (results.pv_to_pq_buses.empty()) {
        std::cout << "No PV buses converted to PQ" << std::endl;
    }
    else {
        for (auto busId : results.pv_to_pq_buses) {
            auto bus = system->getBus(busId);
            if (bus) {
                std::cout << "Bus " << busId << " (" << bus->getName() << ") converted from PV to PQ" << std::endl;
            }
        }
    }

    std::cout << "=====================================" << std::endl;
}

// Вспомогательная функция для добавления синхронных генераторов в систему
void addSynchronousGenerators(std::shared_ptr<CorePowerSystem> system, std::shared_ptr<Logger> logger) {
    if (!system) return;

    logger->info("Adding synchronous generators to the system");

    // Находим генераторы и заменяем их на синхронные генераторы
    for (const auto& genPair : system->getGenerators()) {
        auto busId = genPair.second->getBusId();
        auto genId = genPair.first;
        auto genName = genPair.second->getName();

        logger->info("Converting generator " + genName + " to synchronous generator");

        // Создаем синхронный генератор
        auto syncGen = system->createSynchronousGenerator(
            genName + "_Sync",
            busId,
            GeneratorModelType::PARK_GOREV
        );

        if (syncGen) {
            // Настройка параметров синхронного генератора
            syncGen->setNominalPower(80.0); // МВА
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
    }
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

int main(int argc, char* argv[]) {
    try {
        // Создаем логгер
        auto logger = std::make_shared<Logger>("power_system_test.log", Types::LogLevel::INFO, true);
        logger->info("Starting IEEE 14-bus test");

        // Путь к файлу тестовой схемы
        std::string testFilePath = "data/ieee14.cdf";

        // Если передан аргумент командной строки, используем его как путь к файлу
        if (argc > 1) {
            testFilePath = argv[1];
        }

        // Создаем систему и импортер
        auto system = std::make_shared<CorePowerSystem>("IEEE 14-bus Test System", logger);
        auto importer = std::make_shared<CdfImporter>(logger);

        // Загружаем тестовую схему
        auto start = std::chrono::high_resolution_clock::now();

        bool importSuccess = importer->importFromFile(testFilePath, system);
        if (!importSuccess) {
            logger->error("Failed to import test system from: " + testFilePath);
            std::cerr << "Failed to import test system. See log for details." << std::endl;
            return 1;
        }

        auto importEnd = std::chrono::high_resolution_clock::now();
        auto importTime = std::chrono::duration_cast<std::chrono::milliseconds>(importEnd - start);
        logger->info("System imported in " + std::to_string(importTime.count()) + " ms");

        // Добавляем синхронные генераторы
        addSynchronousGenerators(system, logger);

        // Создаем и настраиваем симулятор
        SimulatorConfig config;
        config.mode = Types::SimulationMode::DYNAMIC;
        config.startTime = 0.0;
        config.endTime = 5.0;  // Уменьшаем для тестирования
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
        if (!simulator->initialize()) {
            logger->error("Failed to initialize simulator");
            std::cerr << "Failed to initialize simulator. See log for details." << std::endl;
            return 2;
        }

        // Получаем и выводим результаты начального расчета потокораспределения
        auto initialResults = simulator->getPowerFlowResults();
        printResults(initialResults, system);

        // Запускаем моделирование
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