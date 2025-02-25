/**
 * @file PS_CorePowerSystem.cpp
 * @brief Реализация ядра энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_CorePowerSystem.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

namespace PowerSystem {

	CorePowerSystem::CorePowerSystem()
		: m_name("New Power System"),
		m_logger(nullptr),
		m_initialized(false),
		m_nextElementId(1) {
	}

	CorePowerSystem::CorePowerSystem(const std::string& name, std::shared_ptr<Logger> logger)
		: m_name(name),
		m_logger(logger),
		m_initialized(false),
		m_nextElementId(1) {
	}

	CorePowerSystem::~CorePowerSystem() {
		clear();
	}

	std::string CorePowerSystem::getName() const {
		return m_name;
	}

	void CorePowerSystem::setName(const std::string& name) {
		m_name = name;
	}

	void CorePowerSystem::setLogger(std::shared_ptr<Logger> logger) {
		m_logger = logger;
	}

	bool CorePowerSystem::addBus(std::shared_ptr<Bus> bus) {
		if (!bus) {
			return false;
		}

		auto id = bus->getId();
		if (id == 0) {
			id = generateId();
			bus->setId(id);
		}
		else if (m_buses.find(id) != m_buses.end()) {
			return false;
		}

		m_buses[id] = bus;
		m_initialized = false;

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Bus added: ID=" + std::to_string(id) +
				", Name=" + bus->getName());
		}

		return true;
	}

	bool CorePowerSystem::addBranch(std::shared_ptr<Branch> branch) {
		if (!branch) {
			return false;
		}

		auto id = branch->getId();
		if (id == 0) {
			id = generateId();
			branch->setId(id);
		}
		else if (m_branches.find(id) != m_branches.end()) {
			return false;
		}

		// Проверка существования узлов
		if (m_buses.find(branch->getFromBusId()) == m_buses.end() ||
			m_buses.find(branch->getToBusId()) == m_buses.end()) {
			return false;
		}

		m_branches[id] = branch;
		m_initialized = false;

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Branch added: ID=" + std::to_string(id) +
				", Name=" + branch->getName());
		}

		return true;
	}

	bool CorePowerSystem::addGenerator(std::shared_ptr<Generator> generator) {
		if (!generator) {
			return false;
		}

		auto id = generator->getId();
		if (id == 0) {
			id = generateId();
			generator->setId(id);
		}
		else if (m_generators.find(id) != m_generators.end()) {
			return false;
		}

		// Проверка существования узла подключения
		if (m_buses.find(generator->getBusId()) == m_buses.end()) {
			return false;
		}

		m_generators[id] = generator;
		m_initialized = false;

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Generator added: ID=" + std::to_string(id) +
				", Name=" + generator->getName());
		}

		return true;
	}

	bool CorePowerSystem::addLoad(std::shared_ptr<Load> load) {
		if (!load) {
			return false;
		}

		auto id = load->getId();
		if (id == 0) {
			id = generateId();
			load->setId(id);
		}
		else if (m_loads.find(id) != m_loads.end()) {
			return false;
		}

		// Проверка существования узла подключения
		if (m_buses.find(load->getBusId()) == m_buses.end()) {
			return false;
		}

		m_loads[id] = load;
		m_initialized = false;

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Load added: ID=" + std::to_string(id) +
				", Name=" + load->getName());
		}

		return true;
	}

	bool CorePowerSystem::removeBus(Types::ElementId id) {
		auto it = m_buses.find(id);
		if (it == m_buses.end()) {
			return false;
		}

		// Проверка наличия подключенных элементов
		for (const auto& branch : m_branches) {
			if (branch.second->getFromBusId() == id || branch.second->getToBusId() == id) {
				if (m_logger) {
					m_logger->log(Types::LogLevel::WARNING, "Cannot remove bus ID=" + std::to_string(id) +
						": connected branch exists");
				}
				return false;
			}
		}

		for (const auto& generator : m_generators) {
			if (generator.second->getBusId() == id) {
				if (m_logger) {
					m_logger->log(Types::LogLevel::WARNING, "Cannot remove bus ID=" + std::to_string(id) +
						": connected generator exists");
				}
				return false;
			}
		}

		for (const auto& load : m_loads) {
			if (load.second->getBusId() == id) {
				if (m_logger) {
					m_logger->log(Types::LogLevel::WARNING, "Cannot remove bus ID=" + std::to_string(id) +
						": connected load exists");
				}
				return false;
			}
		}

		// Удаление узла
		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Bus removed: ID=" + std::to_string(id) +
				", Name=" + it->second->getName());
		}

		m_buses.erase(it);
		m_initialized = false;

		return true;
	}

	bool CorePowerSystem::removeBranch(Types::ElementId id) {
		auto it = m_branches.find(id);
		if (it == m_branches.end()) {
			return false;
		}

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Branch removed: ID=" + std::to_string(id) +
				", Name=" + it->second->getName());
		}

		m_branches.erase(it);
		m_initialized = false;

		return true;
	}

	bool CorePowerSystem::removeGenerator(Types::ElementId id) {
		auto it = m_generators.find(id);
		if (it == m_generators.end()) {
			return false;
		}

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Generator removed: ID=" + std::to_string(id) +
				", Name=" + it->second->getName());
		}

		m_generators.erase(it);
		m_initialized = false;

		return true;
	}

	bool CorePowerSystem::removeLoad(Types::ElementId id) {
		auto it = m_loads.find(id);
		if (it == m_loads.end()) {
			return false;
		}

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Load removed: ID=" + std::to_string(id) +
				", Name=" + it->second->getName());
		}

		m_loads.erase(it);
		m_initialized = false;

		return true;
	}

	std::shared_ptr<Bus> CorePowerSystem::getBus(Types::ElementId id) const {
		auto it = m_buses.find(id);
		if (it != m_buses.end()) {
			return it->second;
		}
		return nullptr;
	}

	std::shared_ptr<Branch> CorePowerSystem::getBranch(Types::ElementId id) const {
		auto it = m_branches.find(id);
		if (it != m_branches.end()) {
			return it->second;
		}
		return nullptr;
	}

	std::shared_ptr<Generator> CorePowerSystem::getGenerator(Types::ElementId id) const {
		auto it = m_generators.find(id);
		if (it != m_generators.end()) {
			return it->second;
		}
		return nullptr;
	}

	std::shared_ptr<Load> CorePowerSystem::getLoad(Types::ElementId id) const {
		auto it = m_loads.find(id);
		if (it != m_loads.end()) {
			return it->second;
		}
		return nullptr;
	}

	const std::map<Types::ElementId, std::shared_ptr<Bus>>& CorePowerSystem::getBuses() const {
		return m_buses;
	}

	const std::map<Types::ElementId, std::shared_ptr<Branch>>& CorePowerSystem::getBranches() const {
		return m_branches;
	}

	const std::map<Types::ElementId, std::shared_ptr<Generator>>& CorePowerSystem::getGenerators() const {
		return m_generators;
	}

	const std::map<Types::ElementId, std::shared_ptr<Load>>& CorePowerSystem::getLoads() const {
		return m_loads;
	}

	const Types::AdmittanceMatrix& CorePowerSystem::getAdmittanceMatrix() const {
		return m_admittanceMatrix;
	}

	std::shared_ptr<Bus> CorePowerSystem::createBus(const std::string& name, BusType type) {
		auto id = generateId();
		auto bus = std::make_shared<Bus>(id, name, type);

		if (addBus(bus)) {
			return bus;
		}

		return nullptr;
	}

	std::shared_ptr<Branch> CorePowerSystem::createBranch(const std::string& name,
		Types::ElementId fromBusId,
		Types::ElementId toBusId) {
		// Проверка существования узлов
		if (m_buses.find(fromBusId) == m_buses.end() ||
			m_buses.find(toBusId) == m_buses.end()) {
			return nullptr;
		}

		auto id = generateId();
		auto branch = std::make_shared<Branch>(id, name, fromBusId, toBusId);

		if (addBranch(branch)) {
			return branch;
		}

		return nullptr;
	}

	std::shared_ptr<Generator> CorePowerSystem::createGenerator(const std::string& name,
		Types::ElementId busId) {
		// Проверка существования узла
		if (m_buses.find(busId) == m_buses.end()) {
			return nullptr;
		}

		auto id = generateId();
		auto generator = std::make_shared<Generator>(id, name, busId);

		if (addGenerator(generator)) {
			return generator;
		}

		return nullptr;
	}

	std::shared_ptr<Load> CorePowerSystem::createLoad(const std::string& name,
		Types::ElementId busId) {
		// Проверка существования узла
		if (m_buses.find(busId) == m_buses.end()) {
			return nullptr;
		}

		auto id = generateId();
		auto load = std::make_shared<Load>(id, name, busId);

		if (addLoad(load)) {
			return load;
		}

		return nullptr;
	}

	bool CorePowerSystem::buildAdmittanceMatrix() {
		m_admittanceMatrix.clear();

		try {
			// Построение матрицы проводимостей
			for (const auto& branch : m_branches) {
				auto fromBusId = branch.second->getFromBusId();
				auto toBusId = branch.second->getToBusId();
				auto admittance = branch.second->getAdmittance();

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
				m_logger->log(Types::LogLevel::INFO, "Admittance matrix built successfully");
			}

			return true;
		}
		catch (const std::exception& e) {
			if (m_logger) {
				m_logger->log(Types::LogLevel::ERROR, "Failed to build admittance matrix: " +
					std::string(e.what()));
			}
			return false;
		}
	}

	// Добавление сихронного генератора
		bool CorePowerSystem::addSynchronousGenerator(std::shared_ptr<SynchronousGenerator> generator) {
		if (!generator) {
			return false;
		}

		auto id = generator->getId();
		if (id == 0) {
			id = generateId();
			generator->setId(id);
		}
		else if (m_syncGenerators.find(id) != m_syncGenerators.end()) {
			return false;
		}

		// Проверка существования узла подключения
		if (m_buses.find(generator->getBusId()) == m_buses.end()) {
			return false;
		}

		m_syncGenerators[id] = generator;
		m_initialized = false;

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Synchronous generator added: ID=" + std::to_string(id) +
				", Name=" + generator->getName());
		}

		return true;
	}

	// Удаление синхронного генератора
	bool CorePowerSystem::removeSynchronousGenerator(Types::ElementId id) {
		auto it = m_syncGenerators.find(id);
		if (it == m_syncGenerators.end()) {
			return false;
		}

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Synchronous generator removed: ID=" + std::to_string(id) +
				", Name=" + it->second->getName());
		}

		m_syncGenerators.erase(it);
		m_initialized = false;

		return true;
	}

	// Получение синхронного генератора по ID
	std::shared_ptr<SynchronousGenerator> CorePowerSystem::getSynchronousGenerator(Types::ElementId id) const {
		auto it = m_syncGenerators.find(id);
		if (it != m_syncGenerators.end()) {
			return it->second;
		}
		return nullptr;
	}

	// Получение всех синхронных генераторов
	const std::map<Types::ElementId, std::shared_ptr<SynchronousGenerator>>& CorePowerSystem::getSynchronousGenerators() const {
		return m_syncGenerators;
	}

	// Создание нового синхронного генератора
	std::shared_ptr<SynchronousGenerator> CorePowerSystem::createSynchronousGenerator(
		const std::string& name,
		Types::ElementId busId,
		GeneratorModelType modelType) {

		// Проверка существования узла
		if (m_buses.find(busId) == m_buses.end()) {
			return nullptr;
		}

		auto id = generateId();
		auto generator = SynchronousGeneratorFactory::createGenerator(modelType, id, name, busId);

		if (addSynchronousGenerator(generator)) {
			return generator;
		}

		return nullptr;
	}

	// Обновление интерфейсов синхронных генераторов
	bool CorePowerSystem::updateSynchronousGeneratorInterfaces() {
		for (auto& syncGenPair : m_syncGenerators) {
			auto& syncGen = syncGenPair.second;
			auto busId = syncGen->getBusId();

			// Получение узла и его параметров
			auto bus = getBus(busId);
			if (!bus) {
				return false;
			}

			// Создание и заполнение интерфейса для генератора
			GeneratorInterface interface;
			interface.voltage = bus->getVoltage();
			interface.frequency = 50.0; // Базовая частота

			// В реальной системе здесь будет более сложная логика обновления
			// всех параметров интерфейса на основе текущего состояния системы

			// Обновление интерфейса генератора
			syncGen->updateInterface(interface);
		}

		return true;
	}

	// Методы предиктор-корректор для модели с синхронными генераторами
	bool CorePowerSystem::predictorStep(Types::TimeType time, Types::TimeType timeStep) {
		// Обновление интерфейсов генераторов
		if (!updateSynchronousGeneratorInterfaces()) {
			return false;
		}

		// Шаг предиктора для всех синхронных генераторов
		for (auto& syncGenPair : m_syncGenerators) {
			if (!syncGenPair.second->predictorStep(time, timeStep)) {
				return false;
			}
		}

		// Расчет потоков мощности (UPF/NR/FastDecoupled/DC)
		// Здесь должна быть реализация расчета установившегося режима с учетом
		// инжекций мощности от синхронных генераторов

		return true;
	}

	bool CorePowerSystem::correctorStep(Types::TimeType time, Types::TimeType timeStep) {
		// Обновление интерфейсов генераторов с учетом результатов расчета сети
		if (!updateSynchronousGeneratorInterfaces()) {
			return false;
		}

		// Шаг корректора для всех синхронных генераторов
		for (auto& syncGenPair : m_syncGenerators) {
			if (!syncGenPair.second->correctorStep(time, timeStep)) {
				return false;
			}
		}

		return true;
	}

	bool CorePowerSystem::saveToFile(const std::string& filename) const {
		try {
			std::ofstream file(filename);
			if (!file.is_open()) {
				throw std::runtime_error("Could not open file for writing: " + filename);
			}

			// Запись метаинформации
			file << "POWER_SYSTEM_NAME=" << m_name << ";" << std::endl;
			file << "VERSION=1.0;" << std::endl;

			// Запись узлов
			file << "BUSES_COUNT=" << m_buses.size() << ";" << std::endl;
			for (const auto& bus : m_buses) {
				file << "BUS:" << bus.second->serialize() << std::endl;
			}

			// Запись ветвей
			file << "BRANCHES_COUNT=" << m_branches.size() << ";" << std::endl;
			for (const auto& branch : m_branches) {
				file << "BRANCH:" << branch.second->serialize() << std::endl;
			}

			// Запись генераторов
			file << "GENERATORS_COUNT=" << m_generators.size() << ";" << std::endl;
			for (const auto& generator : m_generators) {
				file << "GENERATOR:" << generator.second->serialize() << std::endl;
			}

			// Запись нагрузок
			file << "LOADS_COUNT=" << m_loads.size() << ";" << std::endl;
			for (const auto& load : m_loads) {
				file << "LOAD:" << load.second->serialize() << std::endl;
			}

			// Запись синхронных генераторов
			file << "SYNCHRONOUS_GENERATORS_COUNT=" << m_syncGenerators.size() << ";" << std::endl;
			for (const auto& syncGen : m_syncGenerators) {
				file << "SYNC_GENERATOR:" << syncGen.second->serialize() << std::endl;
			}

			file.close();

			if (m_logger) {
				m_logger->log(Types::LogLevel::INFO, "Power system saved to file: " + filename);
			}

			return true;
		}
		catch (const std::exception& e) {
			if (m_logger) {
				m_logger->log(Types::LogLevel::ERROR, "Failed to save power system: " +
					std::string(e.what()));
			}
			return false;
		}
	}

	bool CorePowerSystem::loadFromFile(const std::string& filename) {
		try {
			std::ifstream file(filename);
			if (!file.is_open()) {
				throw std::runtime_error("Could not open file for reading: " + filename);
			}

			// Очистка текущей системы
			clear();

			std::string line;
			while (std::getline(file, line)) {
				if (line.empty()) {
					continue;
				}

				if (line.find("POWER_SYSTEM_NAME=") == 0) {
					// Чтение имени энергосистемы
					size_t startPos = line.find("=") + 1;
					size_t endPos = line.find(";");
					m_name = line.substr(startPos, endPos - startPos);
				}
				else if (line.find("BUS:") == 0) {
					// Чтение узла
					std::string busData = line.substr(4);
					auto bus = std::make_shared<Bus>();
					if (bus->deserialize(busData)) {
						m_buses[bus->getId()] = bus;
					}
				}
				else if (line.find("BRANCH:") == 0) {
					// Чтение ветви
					std::string branchData = line.substr(7);
					auto branch = std::make_shared<Branch>();
					if (branch->deserialize(branchData)) {
						m_branches[branch->getId()] = branch;
					}
				}
				else if (line.find("GENERATOR:") == 0) {
					// Чтение генератора
					std::string generatorData = line.substr(10);
					auto generator = std::make_shared<Generator>();
					if (generator->deserialize(generatorData)) {
						m_generators[generator->getId()] = generator;
					}
				}
				else if (line.find("LOAD:") == 0) {
					// Чтение нагрузки
					std::string loadData = line.substr(5);
					auto load = std::make_shared<Load>();
					if (load->deserialize(loadData)) {
						m_loads[load->getId()] = load;
					}
				}
				else if (line.find("SYNC_GENERATOR:") == 0) {
					// Чтение синхронного генератора
					std::string syncGenData = line.substr(14);
					auto syncGen = std::make_shared<SynchronousGenerator>();
					if (syncGen->deserialize(syncGenData)) {
						m_syncGenerators[syncGen->getId()] = syncGen;
					}
				}
			}

			file.close();

			// Обновление nextElementId
			Types::ElementId maxId = 0;

			for (const auto& bus : m_buses) {
				maxId = std::max(maxId, bus.first);
			}

			for (const auto& branch : m_branches) {
				maxId = std::max(maxId, branch.first);
			}

			for (const auto& generator : m_generators) {
				maxId = std::max(maxId, generator.first);
			}

			for (const auto& load : m_loads) {
				maxId = std::max(maxId, load.first);
			}

			for (const auto& syncGen : m_syncGenerators) {
				maxId = std::max(maxId, syncGen.first);
			}

			m_nextElementId = maxId + 1;
			m_initialized = false;

			if (m_logger) {
				m_logger->log(Types::LogLevel::INFO, "Power system loaded from file: " + filename);
			}

			return true;
		}
		catch (const std::exception& e) {
			if (m_logger) {
				m_logger->log(Types::LogLevel::ERROR, "Failed to load power system: " +
					std::string(e.what()));
			}
			return false;
		}
	}

	void CorePowerSystem::clear() {
		m_buses.clear();
		m_branches.clear();
		m_generators.clear();
		m_loads.clear();
		m_syncGenerators.clear();
		m_admittanceMatrix.clear();
		m_initialized = false;
		m_nextElementId = 1;

		if (m_logger) {
			m_logger->log(Types::LogLevel::INFO, "Power system cleared");
		}
	}

	bool CorePowerSystem::initialize() {
		if (m_initialized) {
			return true;
		}

		try {
			// Инициализация всех элементов
			for (auto& bus : m_buses) {
				if (!bus.second->initialize()) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to initialize bus: ID=" +
							std::to_string(bus.first));
					}
					return false;
				}
			}

			for (auto& branch : m_branches) {
				if (!branch.second->initialize()) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to initialize branch: ID=" +
							std::to_string(branch.first));
					}
					return false;
				}
			}

			for (auto& generator : m_generators) {
				if (!generator.second->initialize()) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to initialize generator: ID=" +
							std::to_string(generator.first));
					}
					return false;
				}
			}

			for (auto& load : m_loads) {
				if (!load.second->initialize()) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to initialize load: ID=" +
							std::to_string(load.first));
					}
					return false;
				}
			}

			// Инициализация синхронных генераторов
			for (auto& syncGen : m_syncGenerators) {
				if (!syncGen.second->initialize()) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to initialize synchronous generator: ID=" +
							std::to_string(syncGen.first));
					}
					return false;
				}
			}

			// Построение матрицы проводимостей
			if (!buildAdmittanceMatrix()) {
				return false;
			}

			// Начальное обновление интерфейсов синхронных генераторов
			if (!updateSynchronousGeneratorInterfaces()) {
				return false;
			}

			m_initialized = true;

			if (m_logger) {
				m_logger->log(Types::LogLevel::INFO, "Power system initialized successfully");
			}

			return true;
		}
		catch (const std::exception& e) {
			if (m_logger) {
				m_logger->log(Types::LogLevel::ERROR, "Initialization error: " + std::string(e.what()));
			}
			return false;
		}
	}

	bool CorePowerSystem::compute(Types::TimeType time, Types::TimeType timeStep) {
		if (!m_initialized) {
			if (!initialize()) {
				return false;
			}
		}

		try {
			// Если есть синхронные генераторы, используем подход предиктор-корректор
			if (!m_syncGenerators.empty()) {
				// Шаг предиктора: расчет промежуточного состояния генераторов
				if (!predictorStep(time, timeStep)) {
					return false;
				}

				// Шаг корректора: уточнение состояния с учетом рассчитанных напряжений
				if (!correctorStep(time, timeStep)) {
					return false;
				}

				return true;
			}

			// Стандартный расчет для системы без синхронных генераторов

			// Расчет всех элементов
			for (auto& bus : m_buses) {
				if (!bus.second->compute(time, timeStep)) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to compute bus: ID=" +
							std::to_string(bus.first));
					}
					return false;
				}
			}

			for (auto& branch : m_branches) {
				if (!branch.second->compute(time, timeStep)) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to compute branch: ID=" +
							std::to_string(branch.first));
					}
					return false;
				}
			}

			for (auto& generator : m_generators) {
				if (!generator.second->compute(time, timeStep)) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to compute generator: ID=" +
							std::to_string(generator.first));
					}
					return false;
				}
			}

			for (auto& load : m_loads) {
				if (!load.second->compute(time, timeStep)) {
					if (m_logger) {
						m_logger->log(Types::LogLevel::ERROR, "Failed to compute load: ID=" +
							std::to_string(load.first));
					}
					return false;
				}
			}

			return true;
		}
		catch (const std::exception& e) {
			if (m_logger) {
				m_logger->log(Types::LogLevel::ERROR, "Computation error: " + std::string(e.what()));
			}
			return false;
		}
	}

	Types::ElementId CorePowerSystem::generateId() {
		return m_nextElementId++;
	}
}