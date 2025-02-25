/**
 * @file PS_PowerFlowSolver.cpp
 * @brief Реализация класса для решения задачи потокораспределения
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_PowerFlowSolver.hpp"
#include "../Core/PS_Exception.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace PowerSystem {

    // ------ BasePowerFlowSolver ------

    BasePowerFlowSolver::BasePowerFlowSolver(PowerFlowMethod method)
        : m_logger(nullptr),
        m_maxIterations(100),
        m_tolerance(1e-6),
        m_baseMVA(100.0),
        m_method(method) {
    }

    void BasePowerFlowSolver::setMaxIterations(int maxIterations) {
        if (maxIterations <= 0) {
            throw InvalidParameterException("Max iterations must be positive");
        }
        m_maxIterations = maxIterations;
    }

    void BasePowerFlowSolver::setTolerance(double tolerance) {
        if (tolerance <= 0) {
            throw InvalidParameterException("Tolerance must be positive");
        }
        m_tolerance = tolerance;
    }

    void BasePowerFlowSolver::setBaseMVA(double baseMVA) {
        if (baseMVA <= 0) {
            throw InvalidParameterException("Base MVA must be positive");
        }
        m_baseMVA = baseMVA;
    }

    void BasePowerFlowSolver::setLogger(std::shared_ptr<Logger> logger) {
        m_logger = logger;
    }

    PowerFlowMethod BasePowerFlowSolver::getMethod() const {
        return m_method;
    }

    bool BasePowerFlowSolver::prepareData(
        const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        const std::map<Types::ElementId, std::shared_ptr<Branch>>& branches,
        const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators,
        const std::map<Types::ElementId, std::shared_ptr<Load>>& loads) {

        // Очистка буферов
        m_busIds.clear();
        m_pvBusIds.clear();
        m_pqBusIds.clear();
        m_slackBusIds.clear();
        m_admittanceMatrix.clear();

        // Проверка наличия данных
        if (buses.empty()) {
            if (m_logger) m_logger->error("No buses in the system");
            return false;
        }
        if (branches.empty()) {
            if (m_logger) m_logger->error("No branches in the system");
            return false;
        }

        // Сбор ID всех узлов
        for (const auto& pair : buses) {
            m_busIds.push_back(pair.first);
        }
        std::sort(m_busIds.begin(), m_busIds.end());

        // Классификация узлов
        if (!classifyBuses(buses, generators)) {
            return false;
        }

        // Проверка наличия балансирующего узла
        if (m_slackBusIds.empty()) {
            if (m_logger) m_logger->error("No slack bus in the system");
            return false;
        }

        // Построение матрицы проводимостей
        if (!buildAdmittanceMatrix(buses, branches)) {
            return false;
        }

        return true;
    }

    bool BasePowerFlowSolver::buildAdmittanceMatrix(
        const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        const std::map<Types::ElementId, std::shared_ptr<Branch>>& branches) {

        m_admittanceMatrix.clear();

        try {
            // Заполнение диагональных элементов нулями
            for (const auto& busId : m_busIds) {
                m_admittanceMatrix[std::make_pair(busId, busId)] = Types::Complex(0.0, 0.0);
            }

            // Построение матрицы проводимостей
            for (const auto& branchPair : branches) {
                const auto& branch = branchPair.second;

                // Проверка активности ветви
                if (branch->getState() != Types::ElementState::ENABLED) {
                    continue;
                }

                auto fromBusId = branch->getFromBusId();
                auto toBusId = branch->getToBusId();
                auto admittance = branch->getAdmittance();

                // Проверка узлов
                if (std::find(m_busIds.begin(), m_busIds.end(), fromBusId) == m_busIds.end() ||
                    std::find(m_busIds.begin(), m_busIds.end(), toBusId) == m_busIds.end()) {
                    if (m_logger) m_logger->error("Branch connects to non-existent bus");
                    return false;
                }

                // Диагональные элементы
                auto diagFrom = std::make_pair(fromBusId, fromBusId);
                auto diagTo = std::make_pair(toBusId, toBusId);

                if (m_admittanceMatrix.find(diagFrom) == m_admittanceMatrix.end()) {
                    m_admittanceMatrix[diagFrom] = Types::Complex(0.0, 0.0);
                }

                if (m_admittanceMatrix.find(diagTo) == m_admittanceMatrix.end()) {
                    m_admittanceMatrix[diagTo] = Types::Complex(0.0, 0.0);
                }

                m_admittanceMatrix[diagFrom] += admittance;
                m_admittanceMatrix[diagTo] += admittance;

                // Недиагональные элементы
                auto nonDiagFrom = std::make_pair(fromBusId, toBusId);
                auto nonDiagTo = std::make_pair(toBusId, fromBusId);

                m_admittanceMatrix[nonDiagFrom] = -admittance;
                m_admittanceMatrix[nonDiagTo] = -admittance;
            }

            if (m_logger) {
                m_logger->info("Admittance matrix built successfully");
            }

            return true;
        }
        catch (const std::exception& e) {
            if (m_logger) {
                m_logger->error("Failed to build admittance matrix: " + std::string(e.what()));
            }
            return false;
        }
    }

    bool BasePowerFlowSolver::classifyBuses(
        const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators) {

        m_pvBusIds.clear();
        m_pqBusIds.clear();
        m_slackBusIds.clear();

        // Создаем карту соответствия узлов и генераторов
        std::map<Types::ElementId, std::vector<std::shared_ptr<Generator>>> busGenerators;
        for (const auto& genPair : generators) {
            if (genPair.second->getState() == Types::ElementState::ENABLED) {
                busGenerators[genPair.second->getBusId()].push_back(genPair.second);
            }
        }

        // Классифицируем узлы по типам
        for (const auto& busPair : buses) {
            BusType type = busPair.second->getType();
            Types::ElementId busId = busPair.first;

            if (busPair.second->getState() != Types::ElementState::ENABLED) {
                continue; // Пропускаем отключенные узлы
            }

            switch (type) {
            case BusType::SLACK:
                m_slackBusIds.push_back(busId);
                break;
            case BusType::PV:
                // Проверяем наличие генераторов
                if (busGenerators.find(busId) == busGenerators.end() ||
                    busGenerators[busId].empty()) {
                    // Узел без генераторов, переводим в PQ
                    if (m_logger) {
                        m_logger->warning("PV bus without generators converted to PQ: " +
                            std::to_string(busId));
                    }
                    m_pqBusIds.push_back(busId);
                }
                else {
                    m_pvBusIds.push_back(busId);
                }
                break;
            case BusType::PQ:
                m_pqBusIds.push_back(busId);
                break;
            default:
                if (m_logger) {
                    m_logger->error("Unknown bus type for bus: " + std::to_string(busId));
                }
                return false;
            }
        }

        // Сортировка списков для стабильной индексации
        std::sort(m_slackBusIds.begin(), m_slackBusIds.end());
        std::sort(m_pvBusIds.begin(), m_pvBusIds.end());
        std::sort(m_pqBusIds.begin(), m_pqBusIds.end());

        if (m_logger) {
            m_logger->info("Buses classified: Slack=" + std::to_string(m_slackBusIds.size()) +
                ", PV=" + std::to_string(m_pvBusIds.size()) +
                ", PQ=" + std::to_string(m_pqBusIds.size()));
        }

        return true;
    }

    bool BasePowerFlowSolver::updateNetworkState(
        std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        std::map<Types::ElementId, std::shared_ptr<Branch>>& branches,
        const PowerFlowResults& results) {

        // Обновление напряжений в узлах
        for (auto& busPair : buses) {
            auto busId = busPair.first;
            auto& bus = busPair.second;

            auto it = results.voltage.find(busId);
            if (it != results.voltage.end()) {
                bus->setVoltage(it->second);
            }

            auto powerIt = results.power.find(busId);
            if (powerIt != results.power.end()) {
                bus->setPower(powerIt->second);
            }
        }

        // Обновление токов и мощностей в ветвях
        for (auto& branchPair : branches) {
            auto branchId = branchPair.first;
            auto& branch = branchPair.second;

            auto currentIt = results.current.find(branchId);
            if (currentIt != results.current.end()) {
                branch->setCurrent(currentIt->second);
            }

            auto powerIt = results.branchPower.find(branchId);
            if (powerIt != results.branchPower.end()) {
                branch->setPower(powerIt->second);
            }
        }

        return true;
    }

    bool BasePowerFlowSolver::checkReactivePowerLimits(
        std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators,
        PowerFlowResults& results) {

        bool changes = false;
        results.pv_to_pq_buses.clear();

        // Создаем карту соответствия узлов и генераторов
        std::map<Types::ElementId, std::vector<std::shared_ptr<Generator>>> busGenerators;
        for (const auto& genPair : generators) {
            if (genPair.second->getState() == Types::ElementState::ENABLED) {
                busGenerators[genPair.second->getBusId()].push_back(genPair.second);
            }
        }

        // Проверяем ограничения для PV-узлов
        for (auto busId : m_pvBusIds) {
            auto busIt = buses.find(busId);
            if (busIt == buses.end()) continue;

            auto& bus = busIt->second;
            auto busGeneratorsIt = busGenerators.find(busId);
            if (busGeneratorsIt == busGenerators.end()) continue;

            double totalQ = bus->getPower().imag();
            double totalQmax = 0.0;
            double totalQmin = 0.0;

            // Суммируем ограничения реактивной мощности для всех генераторов в узле
            for (const auto& gen : busGeneratorsIt->second) {
                totalQmax += gen->getMaxReactivePower();
                totalQmin += gen->getMinReactivePower();
            }

            // Проверяем верхний предел
            if (totalQ > totalQmax) {
                if (m_logger) {
                    m_logger->warning("Generator reactive power limit (max) violated in bus " +
                        std::to_string(busId) + ": Q=" + std::to_string(totalQ) +
                        ", Qmax=" + std::to_string(totalQmax));
                }

                // Ограничиваем реактивную мощность и конвертируем в PQ-узел
                bus->setPower(Types::Complex(bus->getPower().real(), totalQmax));
                bus->setType(BusType::PQ);
                results.pv_to_pq_buses.push_back(busId);
                changes = true;
            }
            // Проверяем нижний предел
            else if (totalQ < totalQmin) {
                if (m_logger) {
                    m_logger->warning("Generator reactive power limit (min) violated in bus " +
                        std::to_string(busId) + ": Q=" + std::to_string(totalQ) +
                        ", Qmin=" + std::to_string(totalQmin));
                }

                // Ограничиваем реактивную мощность и конвертируем в PQ-узел
                bus->setPower(Types::Complex(bus->getPower().real(), totalQmin));
                bus->setType(BusType::PQ);
                results.pv_to_pq_buses.push_back(busId);
                changes = true;
            }
        }

        if (changes && m_logger) {
            m_logger->info("Reactive power limits violated, converted " +
                std::to_string(results.pv_to_pq_buses.size()) +
                " PV buses to PQ buses");
        }

        return changes;
    }

    // ------ NewtonRaphsonPowerFlowSolver ------

    NewtonRaphsonPowerFlowSolver::NewtonRaphsonPowerFlowSolver()
        : BasePowerFlowSolver(PowerFlowMethod::NEWTON_RAPHSON) {
    }

    std::string NewtonRaphsonPowerFlowSolver::getName() const {
        return "Newton-Raphson Power Flow Solver";
    }

    bool NewtonRaphsonPowerFlowSolver::solve(
        const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        const std::map<Types::ElementId, std::shared_ptr<Branch>>& branches,
        const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators,
        const std::map<Types::ElementId, std::shared_ptr<Load>>& loads,
        PowerFlowResults& results) {

        // Инициализируем результаты
        results.converged = false;
        results.iterations = 0;
        results.maxMismatch = 0.0;
        results.voltage.clear();
        results.power.clear();
        results.current.clear();
        results.branchPower.clear();
        results.pv_to_pq_buses.clear();

        // Подготовка данных
        if (!prepareData(buses, branches, generators, loads)) {
            if (m_logger) m_logger->error("Failed to prepare data for power flow calculation");
            return false;
        }

        // Создаем изменяемую копию узлов для проверки ограничений
        std::map<Types::ElementId, std::shared_ptr<Bus>> busesCopy;
        for (const auto& busPair : buses) {
            busesCopy[busPair.first] = std::make_shared<Bus>(*busPair.second);
        }

        // Подготавливаем индексы для быстрого доступа
        m_busIndices.clear();
        int index = 0;
        for (auto busId : m_busIds) {
            m_busIndices[busId] = index++;
        }

        int pvBusCount = m_pvBusIds.size();
        int pqBusCount = m_pqBusIds.size();
        int totalSize = pvBusCount + 2 * pqBusCount;  // |P_pv| + |P_pq| + |Q_pq|

        // Инициализируем матрицы и векторы для метода Ньютона-Рафсона
        m_jacobian = Eigen::MatrixXd::Zero(totalSize, totalSize);
        m_mismatch = Eigen::VectorXd::Zero(totalSize);
        m_correction = Eigen::VectorXd::Zero(totalSize);

        // Итерационный процесс
        bool converged = false;
        int iteration = 0;
        double maxMismatch = 0.0;

        while (iteration < m_maxIterations && !converged) {
            // Строим матрицу Якоби
            if (!buildJacobian(busesCopy, pvBusCount, pqBusCount)) {
                if (m_logger) m_logger->error("Failed to build Jacobian matrix");
                return false;
            }

            // Вычисляем невязки мощности
            if (!calculatePowerMismatch(busesCopy, pvBusCount, pqBusCount)) {
                if (m_logger) m_logger->error("Failed to calculate power mismatch");
                return false;
            }

            // Находим поправки
            m_correction = m_jacobian.colPivHouseholderQr().solve(m_mismatch);

            // Обновляем напряжения
            if (!updateVoltages(busesCopy, pvBusCount, pqBusCount)) {
                if (m_logger) m_logger->error("Failed to update voltages");
                return false;
            }

            // Проверяем сходимость
            maxMismatch = m_mismatch.lpNorm<Eigen::Infinity>();
            iteration++;

            if (m_logger) {
                m_logger->info("Iteration " + std::to_string(iteration) +
                    ", max mismatch = " + std::to_string(maxMismatch));
            }

            if (maxMismatch < m_tolerance) {
                converged = true;
            }

            // Проверка ограничений по реактивной мощности после сходимости
            if (converged) {
                bool limitViolations = checkReactivePowerLimits(busesCopy, generators, results);
                if (limitViolations) {
                    // Если есть нарушения, продолжаем итерации
                    converged = false;

                    // Реклассификация узлов
                    if (!classifyBuses(busesCopy, generators)) {
                        if (m_logger) m_logger->error("Failed to reclassify buses after limit violations");
                        return false;
                    }

                    // Обновление индексов
                    pvBusCount = m_pvBusIds.size();
                    pqBusCount = m_pqBusIds.size();
                    totalSize = pvBusCount + 2 * pqBusCount;

                    // Изменение размера матриц
                    m_jacobian.resize(totalSize, totalSize);
                    m_mismatch.resize(totalSize);
                    m_correction.resize(totalSize);

                    if (m_logger) {
                        m_logger->info("Reactive power limits violated, continuing iterations");
                    }
                }
            }
        }

        // Заполняем результаты
        results.converged = converged;
        results.iterations = iteration;
        results.maxMismatch = maxMismatch;

        // Заполняем напряжения и мощности
        for (const auto& busPair : busesCopy) {
            auto busId = busPair.first;
            auto& bus = busPair.second;

            results.voltage[busId] = bus->getVoltage();
            results.power[busId] = bus->getPower();
        }

        // Вычисляем токи и мощности в ветвях
        for (const auto& branchPair : branches) {
            auto branchId = branchPair.first;
            auto& branch = branchPair.second;

            auto fromBusId = branch->getFromBusId();
            auto toBusId = branch->getToBusId();

            auto fromBusIt = busesCopy.find(fromBusId);
            auto toBusIt = busesCopy.find(toBusId);

            if (fromBusIt != busesCopy.end() && toBusIt != busesCopy.end()) {
                // Напряжения в узлах
                Types::Complex v1 = fromBusIt->second->getVoltage();
                Types::Complex v2 = toBusIt->second->getVoltage();

                // Ток в ветви
                Types::Complex y = branch->getAdmittance();
                Types::Complex i = y * (v1 - v2);

                // Мощность в ветви
                Types::Complex s = v1 * std::conj(i);

                results.current[branchId] = i;
                results.branchPower[branchId] = s;
            }
        }

        // Обновляем состояние сети
        if (converged) {
            updateNetworkState(const_cast<std::map<Types::ElementId, std::shared_ptr<Bus>>&>(buses),
                const_cast<std::map<Types::ElementId, std::shared_ptr<Branch>>&>(branches),
                results);
        }

        return converged;
    }

    bool NewtonRaphsonPowerFlowSolver::buildJacobian(
        const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        int pvBusCount, int pqBusCount) {

        int totalSize = pvBusCount + 2 * pqBusCount;
        m_jacobian.setZero(totalSize, totalSize);

        // Заполнение матрицы Якоби
        // J = [ dP/dθ  dP/dV ]
        //     [ dQ/dθ  dQ/dV ]

        int pvIndex = 0;
        int pqIndex = pvBusCount;
        int qIndex = pvBusCount + pqBusCount;

        // Формируем матрицу Якоби по блокам
        for (int i = 0; i < pvBusCount + pqBusCount; i++) {
            Types::ElementId busI;
            if (i < pvBusCount) {
                busI = m_pvBusIds[i];
            }
            else {
                busI = m_pqBusIds[i - pvBusCount];
            }

            auto busIIt = buses.find(busI);
            if (busIIt == buses.end()) continue;

            Types::Complex vi = busIIt->second->getVoltage();
            double vi_abs = std::abs(vi);
            double theta_i = std::arg(vi);

            for (int j = 0; j < pvBusCount + pqBusCount; j++) {
                Types::ElementId busJ;
                if (j < pvBusCount) {
                    busJ = m_pvBusIds[j];
                }
                else {
                    busJ = m_pqBusIds[j - pvBusCount];
                }

                auto busJIt = buses.find(busJ);
                if (busJIt == buses.end()) continue;

                Types::Complex vj = busJIt->second->getVoltage();
                double vj_abs = std::abs(vj);
                double theta_j = std::arg(vj);

                // Находим элемент матрицы проводимостей
                Types::Complex yij = Types::Complex(0.0, 0.0);
                auto pairIJ = std::make_pair(busI, busJ);
                auto it = m_admittanceMatrix.find(pairIJ);
                if (it != m_admittanceMatrix.end()) {
                    yij = it->second;
                }

                double gij = yij.real();
                double bij = yij.imag();
                double theta_ij = theta_i - theta_j;

                // Заполняем блок dP/dθ
                if (i < pvBusCount && j < pvBusCount) {
                    // PV-PV
                    m_jacobian(i, j) = vi_abs * vj_abs * (gij * std::sin(theta_ij) - bij * std::cos(theta_ij));
                }
                else if (i < pvBusCount && j >= pvBusCount) {
                    // PV-PQ
                    m_jacobian(i, j) = vi_abs * vj_abs * (gij * std::sin(theta_ij) - bij * std::cos(theta_ij));
                }
                else if (i >= pvBusCount && j < pvBusCount) {
                    // PQ-PV
                    m_jacobian(i - pvBusCount + pqIndex, j) = vi_abs * vj_abs * (gij * std::sin(theta_ij) - bij * std::cos(theta_ij));
                }
                else {
                    // PQ-PQ
                    m_jacobian(i - pvBusCount + pqIndex, j - pvBusCount + pqIndex) = vi_abs * vj_abs * (gij * std::sin(theta_ij) - bij * std::cos(theta_ij));
                }

                // Заполняем блок dP/dV
                if (i < pvBusCount && j >= pvBusCount) {
                    // PV-PQ
                    m_jacobian(i, j - pvBusCount + qIndex) = vi_abs * (gij * std::cos(theta_ij) + bij * std::sin(theta_ij));
                }
                else if (i >= pvBusCount && j >= pvBusCount) {
                    // PQ-PQ
                    m_jacobian(i - pvBusCount + pqIndex, j - pvBusCount + qIndex) = vi_abs * (gij * std::cos(theta_ij) + bij * std::sin(theta_ij));
                }

                // Заполняем блок dQ/dθ
                if (i >= pvBusCount && j < pvBusCount) {
                    // PQ-PV
                    m_jacobian(i - pvBusCount + qIndex, j) = vi_abs * vj_abs * (-gij * std::cos(theta_ij) - bij * std::sin(theta_ij));
                }
                else if (i >= pvBusCount && j >= pvBusCount) {
                    // PQ-PQ
                    m_jacobian(i - pvBusCount + qIndex, j - pvBusCount + pqIndex) = vi_abs * vj_abs * (-gij * std::cos(theta_ij) - bij * std::sin(theta_ij));
                }

                // Заполняем блок dQ/dV
                if (i >= pvBusCount && j >= pvBusCount) {
                    // PQ-PQ
                    m_jacobian(i - pvBusCount + qIndex, j - pvBusCount + qIndex) = vi_abs * (gij * std::sin(theta_ij) - bij * std::cos(theta_ij));
                }
            }
        }

        return true;
    }

    bool NewtonRaphsonPowerFlowSolver::calculatePowerMismatch(
        const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        int pvBusCount, int pqBusCount) {

        int totalSize = pvBusCount + 2 * pqBusCount;
        m_mismatch.setZero(totalSize);

        // Вычисляем невязки мощности для каждого узла
        int pvIndex = 0;
        int pqIndex = pvBusCount;
        int qIndex = pvBusCount + pqBusCount;

        // Обработка PV-узлов (только активная мощность)
        for (int i = 0; i < pvBusCount; i++) {
            Types::ElementId busId = m_pvBusIds[i];
            auto busIt = buses.find(busId);
            if (busIt == buses.end()) continue;

            auto& bus = busIt->second;
            Types::Complex scheduledPower = bus->getPower();
            Types::Complex calculatedPower = calculatePower(buses, busId);

            // Невязка по активной мощности
            m_mismatch(i) = scheduledPower.real() - calculatedPower.real();
        }

        // Обработка PQ-узлов (активная и реактивная мощность)
        for (int i = 0; i < pqBusCount; i++) {
            Types::ElementId busId = m_pqBusIds[i];
            auto busIt = buses.find(busId);
            if (busIt == buses.end()) continue;

            auto& bus = busIt->second;
            Types::Complex scheduledPower = bus->getPower();
            Types::Complex calculatedPower = calculatePower(buses, busId);

            // Невязка по активной мощности
            m_mismatch(i + pqIndex) = scheduledPower.real() - calculatedPower.real();
            // Невязка по реактивной мощности
            m_mismatch(i + qIndex) = scheduledPower.imag() - calculatedPower.imag();
        }

        return true;
    }

    Types::Complex NewtonRaphsonPowerFlowSolver::calculatePower(
        const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        Types::ElementId busId) {

        auto busIt = buses.find(busId);
        if (busIt == buses.end()) return Types::Complex(0.0, 0.0);

        Types::Complex vi = busIt->second->getVoltage();
        Types::Complex power(0.0, 0.0);

        for (const auto& busPair : buses) {
            Types::ElementId j = busPair.first;
            Types::Complex vj = busPair.second->getVoltage();

            // Находим элемент матрицы проводимостей
            auto pairIJ = std::make_pair(busId, j);
            auto it = m_admittanceMatrix.find(pairIJ);
            if (it != m_admittanceMatrix.end()) {
                Types::Complex yij = it->second;
                power += vi * std::conj(vj * yij);
            }
        }

        return power;
    }

    bool NewtonRaphsonPowerFlowSolver::updateVoltages(
        std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
        int pvBusCount, int pqBusCount) {

        int pqIndex = pvBusCount;
        int qIndex = pvBusCount + pqBusCount;

        // Обновляем углы для PV-узлов
        for (int i = 0; i < pvBusCount; i++) {
            Types::ElementId busId = m_pvBusIds[i];
            auto busIt = buses.find(busId);
            if (busIt == buses.end()) continue;

            auto& bus = busIt->second;
            Types::Complex v = bus->getVoltage();
            double magnitude = std::abs(v);
            double angle = std::arg(v) + m_correction(i);

            bus->setVoltage(Types::Complex(magnitude * std::cos(angle), magnitude * std::sin(angle)));
        }

        // Обновляем углы и модули для PQ-узлов
        for (int i = 0; i < pqBusCount; i++) {
            Types::ElementId busId = m_pqBusIds[i];
            auto busIt = buses.find(busId);
            if (busIt == buses.end()) continue;

            auto& bus = busIt->second;
            Types::Complex v = bus->getVoltage();
            double magnitude = std::abs(v) * (1.0 + m_correction(i + qIndex));
            double angle = std::arg(v) + m_correction(i + pqIndex);

            bus->setVoltage(Types::Complex(magnitude * std::cos(angle), magnitude * std::sin(angle)));
        }

        return true;
    }

    // ------ PowerFlowSolverFactory ------

    std::unique_ptr<IPowerFlowSolver> PowerFlowSolverFactory::createSolver(PowerFlowMethod method) {
        switch (method) {
        case PowerFlowMethod::NEWTON_RAPHSON:
            return std::make_unique<NewtonRaphsonPowerFlowSolver>();
        case PowerFlowMethod::GAUSS_SEIDEL:
            throw PowerSystemException("Gauss-Seidel method not implemented yet");
        case PowerFlowMethod::FAST_DECOUPLED:
            throw PowerSystemException("Fast Decoupled method not implemented yet");
        default:
            throw PowerSystemException("Unknown power flow method");
        }
    }

} // namespace PowerSystem