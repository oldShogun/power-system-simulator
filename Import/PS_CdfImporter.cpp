/**
 * @file PS_CdfImporter.cpp
 * @brief Реализация класса импорта данных из CDF формата (IEEE Common Format)
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_CdfImporter.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace PowerSystem {

    CdfImporter::CdfImporter(std::shared_ptr<Logger> logger)
        : m_logger(logger) {
    }

    bool CdfImporter::importFromFile(const std::string& filename, std::shared_ptr<CorePowerSystem> system) {
        if (!system) {
            if (m_logger) m_logger->error("CorePowerSystem is null");
            return false;
        }

        std::ifstream file(filename);
        if (!file.is_open()) {
            if (m_logger) m_logger->error("Failed to open file: " + filename);
            return false;
        }

        // Очищаем систему перед импортом
        system->clear();

        std::string line;
        std::string section = "";
        bool inBusSection = false;
        bool inBranchSection = false;

        while (std::getline(file, line)) {
            // Пропускаем пустые строки
            if (line.empty()) {
                continue;
            }

            // Проверяем секционные маркеры
            if (line.find("BUS DATA FOLLOWS") != std::string::npos) {
                inBusSection = true;
                if (m_logger) m_logger->info("Processing bus data section");
                continue;
            }

            if (line.find("-999") != std::string::npos) {
                if (inBusSection) {
                    inBusSection = false;
                    inBranchSection = true;
                    if (m_logger) m_logger->info("Processing branch data section");
                }
                else if (inBranchSection) {
                    inBranchSection = false;
                }
                continue;
            }

            // Пропускаем остальные служебные строки
            if (line.find("LOSS ZONES FOLLOWS") != std::string::npos ||
                line.find("INTERCHANGE DATA FOLLOWS") != std::string::npos ||
                line.find("TIE LINES FOLLOWS") != std::string::npos ||
                line.find("END OF DATA") != std::string::npos ||
                line[0] == '-') {
                continue;
            }

            // Обработка данных в зависимости от секции
            if (inBusSection) {
                if (!processBusData(line, system)) {
                    if (m_logger) m_logger->warning("Failed to process bus data: " + line);
                }
            }
            else if (inBranchSection) {
                if (!processBranchData(line, system)) {
                    if (m_logger) m_logger->warning("Failed to process branch data: " + line);
                }
            }
        }

        file.close();

        bool hasSlackBus = false;
        for (const auto& busPair : system->getBuses()) {
            if (busPair.second->getType() == BusType::SLACK) {
                hasSlackBus = true;
                break;
            }
        }

        if (!hasSlackBus) {
            if (m_logger) m_logger->error("No slack bus found in the system");
            // Если нет балансирующего узла, можно выбрать первый генераторный узел
            bool slackAssigned = false;
            for (const auto& busPair : system->getBuses()) {
                auto busId = busPair.first;
                auto& bus = busPair.second;

                // Проверяем наличие генераторов в узле
                bool hasGenerator = false;
                for (const auto& genPair : system->getGenerators()) {
                    if (genPair.second->getBusId() == busId) {
                        hasGenerator = true;
                        break;
                    }
                }

                if (hasGenerator) {
                    bus->setType(BusType::SLACK);
                    if (m_logger) m_logger->warning("Automatically assigned slack bus to: " +
                        bus->getName() + " (ID: " + std::to_string(busId) + ")");
                    slackAssigned = true;
                    break;
                }
            }

            if (!slackAssigned) {
                if (m_logger) m_logger->error("Could not assign a slack bus. System must have at least one generator");
                return false;
            }
        }

        if (m_logger) {
            m_logger->info("Imported from CDF file: " + filename);
            m_logger->info("System name: " + system->getName());
            m_logger->info("Buses: " + std::to_string(system->getBuses().size()));
            m_logger->info("Branches: " + std::to_string(system->getBranches().size()));
            m_logger->info("Generators: " + std::to_string(system->getGenerators().size()));
            m_logger->info("Loads: " + std::to_string(system->getLoads().size()));
        }

        return true;
    }

    bool CdfImporter::processBusData(const std::string& line, std::shared_ptr<CorePowerSystem> system) {
        try {
            std::istringstream iss(line);
            std::string token;
            std::vector<std::string> tokens;

            // Разбиваем строку на токены
            while (iss >> token) {
                tokens.push_back(token);
            }

            if (tokens.size() < 13) {
                if (m_logger) m_logger->error("Invalid bus data format: " + line);
                return false;
            }

            // Считывание данных узла
            int busNumber = std::stoi(tokens[0]);

            // Собираем имя узла (может состоять из нескольких слов)
            std::string busName = tokens[1];
            size_t nameIndex = 2;
            while (nameIndex < tokens.size() &&
                tokens[nameIndex] != "1" &&
                tokens[nameIndex] != "2" &&
                tokens[nameIndex] != "3" &&
                !std::isdigit(tokens[nameIndex][0])) {
                busName += " " + tokens[nameIndex];
                nameIndex++;
            }

            // Пропускаем прочие служебные поля до типа узла
            size_t typeIndex = nameIndex;
            while (typeIndex < tokens.size() && tokens[typeIndex].length() == 1 &&
                (tokens[typeIndex] == "1" || tokens[typeIndex] == "0")) {
                typeIndex++;
            }

            // Тип узла (3=SLACK, 2=PV, 0=PQ)
            int busType = 0;
            if (typeIndex < tokens.size()) {
                busType = std::stoi(tokens[typeIndex]);
            }

            // После типа узла идут напряжение и угол
            size_t voltageIndex = typeIndex + 1;
            double vmag = 1.0;  // Значение по умолчанию
            if (voltageIndex < tokens.size()) {
                vmag = std::stod(tokens[voltageIndex]);
            }

            size_t angleIndex = voltageIndex + 1;
            double vang = 0.0;  // Значение по умолчанию
            if (angleIndex < tokens.size()) {
                vang = std::stod(tokens[angleIndex]);
            }

            // Нагрузка: P и Q
            size_t loadPIndex = angleIndex + 1;
            double pLoad = 0.0;
            if (loadPIndex < tokens.size()) {
                pLoad = std::stod(tokens[loadPIndex]);
            }

            size_t loadQIndex = loadPIndex + 1;
            double qLoad = 0.0;
            if (loadQIndex < tokens.size()) {
                qLoad = std::stod(tokens[loadQIndex]);
            }

            // Генерация: P и Q
            size_t genPIndex = loadQIndex + 1;
            double pGen = 0.0;
            if (genPIndex < tokens.size()) {
                pGen = std::stod(tokens[genPIndex]);
            }

            size_t genQIndex = genPIndex + 1;
            double qGen = 0.0;
            if (genQIndex < tokens.size()) {
                qGen = std::stod(tokens[genQIndex]);
            }

            // Уставка напряжения для генератора
            size_t voltageSetIndex = genQIndex + 2;  // Пропускаем "BASE KV"
            double vSet = vmag;  // По умолчанию равно текущему напряжению
            if (voltageSetIndex < tokens.size()) {
                vSet = std::stod(tokens[voltageSetIndex]);
            }

            // Ограничения по реактивной мощности
            size_t qmaxIndex = voltageSetIndex + 1;
            double qMax = 9999.0;  // Значение по умолчанию
            if (qmaxIndex < tokens.size()) {
                qMax = std::stod(tokens[qmaxIndex]);
            }

            size_t qminIndex = qmaxIndex + 1;
            double qMin = -9999.0;  // Значение по умолчанию
            if (qminIndex < tokens.size()) {
                qMin = std::stod(tokens[qminIndex]);
            }

            // Преобразуем градусы в радианы
            double vangRad = vang * M_PI / 180.0;

            // Определяем тип узла
            BusType psType;
            switch (busType) {
            case 3:
                psType = BusType::SLACK;
                break;
            case 2:
                psType = BusType::PV;
                break;
            case 1:
            case 0:
            default:
                psType = BusType::PQ;
                break;
            }

            // Создаем узел
            auto bus = system->createBus("Bus " + std::to_string(busNumber), psType);
            if (!bus) {
                if (m_logger) m_logger->error("Failed to create bus: " + std::to_string(busNumber));
                return false;
            }

            // Устанавливаем напряжение в узле
            bus->setVoltage(Types::Complex(vmag * cos(vangRad), vmag * sin(vangRad)));

            // Если есть нагрузка, создаем нагрузку
            if (std::abs(pLoad) > 1e-6 || std::abs(qLoad) > 1e-6) {
                auto load = system->createLoad("Load " + std::to_string(busNumber), bus->getId());
                if (!load) {
                    if (m_logger) m_logger->warning("Failed to create load for bus: " + std::to_string(busNumber));
                }
                else {
                    // Преобразуем МВт и МВАр в о.е.
                    load->setPower(Types::Complex(pLoad / 100.0, qLoad / 100.0));  // Базис 100 МВА
                    load->setModel(LoadModel::CONSTANT_POWER);
                }
            }

            // Если есть генерация, создаем генератор
            if (std::abs(pGen) > 1e-6 || std::abs(qGen) > 1e-6 || psType == BusType::PV || psType == BusType::SLACK) {
                auto generator = system->createGenerator("Gen " + std::to_string(busNumber), bus->getId());
                if (!generator) {
                    if (m_logger) m_logger->warning("Failed to create generator for bus: " + std::to_string(busNumber));
                }
                else {
                    // Преобразуем МВт и МВАр в о.е.
                    generator->setActivePower(pGen / 100.0);  // Базис 100 МВА
                    generator->setReactivePower(qGen / 100.0);
                    generator->setVoltage(vSet);
                    generator->setMaxReactivePower(qMax / 100.0);
                    generator->setMinReactivePower(qMin / 100.0);
                    generator->setNominalPower(Types::Complex(pGen / 100.0, qGen / 100.0));
                }
            }

            return true;
        }
        catch (const std::exception& e) {
            if (m_logger) m_logger->error("Exception in processBusData: " + std::string(e.what()) +
                " when processing line: " + line);
            return false;
        }
    }

    bool CdfImporter::processBranchData(const std::string& line, std::shared_ptr<CorePowerSystem> system) {
        try {
            std::istringstream iss(line);
            std::string token;
            std::vector<std::string> tokens;

            // Разбиваем строку на токены
            while (iss >> token) {
                tokens.push_back(token);
            }

            if (tokens.size() < 6) {
                if (m_logger) m_logger->error("Invalid branch data format: " + line);
                return false;
            }

            // Считывание данных ветви
            int fromBus = std::stoi(tokens[0]);
            int toBus = std::stoi(tokens[1]);

            // Номер цепи (не используется в текущей реализации)
            // int circuitId = std::stoi(tokens[2]);

            // Пропускаем статусные и конфигурационные поля
            // Ищем параметры ветви
            size_t rIndex = 5;
            while (rIndex < tokens.size() &&
                (tokens[rIndex] == "0" || tokens[rIndex] == "1")) {
                rIndex++;
            }

            double r = 0.0;
            if (rIndex < tokens.size()) {
                r = std::stod(tokens[rIndex]);
            }

            double x = 0.0;
            if (rIndex + 1 < tokens.size()) {
                x = std::stod(tokens[rIndex + 1]);
            }

            double b = 0.0;
            if (rIndex + 2 < tokens.size()) {
                b = std::stod(tokens[rIndex + 2]);
            }

            // Находим ID узлов в нашей системе
            Types::ElementId fromBusId = 0, toBusId = 0;
            for (const auto& busPair : system->getBuses()) {
                if (busPair.second->getName() == "Bus " + std::to_string(fromBus)) {
                    fromBusId = busPair.first;
                }
                if (busPair.second->getName() == "Bus " + std::to_string(toBus)) {
                    toBusId = busPair.first;
                }
            }

            if (fromBusId == 0 || toBusId == 0) {
                if (m_logger) m_logger->error("Failed to find buses for branch: " +
                    std::to_string(fromBus) + "-" + std::to_string(toBus));
                return false;
            }

            // Создаем ветвь
            auto branch = system->createBranch("Branch " + std::to_string(fromBus) + "-" +
                std::to_string(toBus), fromBusId, toBusId);
            if (!branch) {
                if (m_logger) m_logger->error("Failed to create branch: " +
                    std::to_string(fromBus) + "-" + std::to_string(toBus));
                return false;
            }

            // Устанавливаем параметры ветви
            branch->setImpedance(Types::Complex(r, x));

            // Дополнительно можно установить проводимость (шунт)
            if (b > 0) {
                // В текущей реализации отсутствует прямой метод для установки проводимости в ветви
                // Для полной функциональности может потребоваться расширение класса Branch
            }

            // Устанавливаем ограничение по току (по умолчанию высокое значение)
            branch->setRatedCurrent(999.0);

            return true;
        }
        catch (const std::exception& e) {
            if (m_logger) m_logger->error("Exception in processBranchData: " + std::string(e.what()) +
                " when processing line: " + line);
            return false;
        }
    }

    double CdfImporter::parseValue(const std::string& str, double defaultValue) {
        std::string trimmed = str;
        // Удаляем пробелы
        trimmed.erase(std::remove_if(trimmed.begin(), trimmed.end(), ::isspace), trimmed.end());

        if (trimmed.empty()) {
            return defaultValue;
        }

        try {
            return std::stod(trimmed);
        }
        catch (const std::exception&) {
            return defaultValue;
        }
    }

} // namespace PowerSystem