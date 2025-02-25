/**
 * @file PS_SynchronousGenerator.cpp
 * @brief Реализация класса синхронного генератора
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_SynchronousGenerator.hpp"
#include <sstream>
#include <cmath>
#include <algorithm>

namespace PowerSystem {

    // Реализация GeneratorInterface
    GeneratorInterface::GeneratorInterface()
        : voltage(1.0, 0.0),
        current(0.0, 0.0),
        frequency(50.0),
        activePower(0.0),
        reactivePower(0.0),
        rotorAngle(0.0),
        rotorSpeed(1.0),
        excitationVoltage(1.0),
        excitationCurrent(0.0),
        state(GeneratorState::OFFLINE),
        timestamp(0.0) {
    }

    void GeneratorInterface::copyFrom(const GeneratorInterface& other) {
        voltage = other.voltage;
        current = other.current;
        frequency = other.frequency;
        activePower = other.activePower;
        reactivePower = other.reactivePower;
        rotorAngle = other.rotorAngle;
        rotorSpeed = other.rotorSpeed;
        excitationVoltage = other.excitationVoltage;
        excitationCurrent = other.excitationCurrent;
        state = other.state;
        timestamp = other.timestamp;
    }

    // Реализация SynchronousGenerator
    SynchronousGenerator::SynchronousGenerator()
        : PowerSystemElement(),
        m_busId(0),
        m_modelType(GeneratorModelType::SIMPLIFIED),
        m_integrationType(IntegrationMethodType::EULER),
        m_state(GeneratorState::OFFLINE),
        m_nominalPower(100.0),
        m_nominalVoltage(10.5),
        m_powerFactor(0.9),
        m_nominalFrequency(50.0),
        m_inertiaConstant(6.5),
        m_dampingFactor(2.0),
        m_ra(0.0025),
        m_xd(1.8),
        m_xq(1.7),
        m_xd1(0.3),
        m_xq1(0.55),
        m_xd2(0.25),
        m_xq2(0.25),
        m_Td0(8.0),
        m_Tq0(1.0),
        m_Td1(0.8),
        m_Tq1(0.4),
        m_Td2(0.035),
        m_Tq2(0.07),
        m_initialized(false),
        m_needRecalculate(true) {

        // Создание интерфейсов
        m_primaryInterface = std::make_unique<GeneratorInterface>();
        m_secondaryInterface = std::make_unique<GeneratorInterface>();

        // Создание компонентов по умолчанию
        createModel(m_modelType);
        createIntegrationMethod(m_integrationType);
        createExcitationRegulator(ExcitationRegulatorType::CONSTANT);
        createTurbineRegulator(TurbineRegulatorType::CONSTANT);
    }

    SynchronousGenerator::SynchronousGenerator(Types::ElementId id, const std::string& name, Types::ElementId busId)
        : PowerSystemElement(id, name),
        m_busId(busId),
        m_modelType(GeneratorModelType::SIMPLIFIED),
        m_integrationType(IntegrationMethodType::EULER),
        m_state(GeneratorState::OFFLINE),
        m_nominalPower(100.0),
        m_nominalVoltage(10.5),
        m_powerFactor(0.9),
        m_nominalFrequency(50.0),
        m_inertiaConstant(6.5),
        m_dampingFactor(2.0),
        m_ra(0.0025),
        m_xd(1.8),
        m_xq(1.7),
        m_xd1(0.3),
        m_xq1(0.55),
        m_xd2(0.25),
        m_xq2(0.25),
        m_Td0(8.0),
        m_Tq0(1.0),
        m_Td1(0.8),
        m_Tq1(0.4),
        m_Td2(0.035),
        m_Tq2(0.07),
        m_initialized(false),
        m_needRecalculate(true) {

        // Создание интерфейсов
        m_primaryInterface = std::make_unique<GeneratorInterface>();
        m_secondaryInterface = std::make_unique<GeneratorInterface>();

        // Создание компонентов по умолчанию
        createModel(m_modelType);
        createIntegrationMethod(m_integrationType);
        createExcitationRegulator(ExcitationRegulatorType::CONSTANT);
        createTurbineRegulator(TurbineRegulatorType::CONSTANT);
    }

    SynchronousGenerator::~SynchronousGenerator() {
        // Освобождение ресурсов при необходимости
    }

    Types::ElementId SynchronousGenerator::getBusId() const {
        return m_busId;
    }

    void SynchronousGenerator::setBusId(Types::ElementId busId) {
        m_busId = busId;
        m_needRecalculate = true;
    }

    GeneratorState SynchronousGenerator::getState() const {
        return m_state.load();
    }

    void SynchronousGenerator::setState(GeneratorState state) {
        m_state.store(state);
        m_needRecalculate = true;
    }

    GeneratorModelType SynchronousGenerator::getModelType() const {
        return m_modelType;
    }

    bool SynchronousGenerator::setModelType(GeneratorModelType type) {
        std::lock_guard<std::mutex> lock(m_calculationMutex);

        if (m_modelType == type) {
            return true; // Тип модели не изменился
        }

        // Создание новой модели
        if (!createModel(type)) {
            return false;
        }

        // Адаптация метода интегрирования к новой модели
        if (m_model && m_integrator) {
            int stateSize = m_model->getStateSize();
            int inputSize = m_model->getInputSize();

            auto derivFunc = [this](const std::vector<double>& state,
                const std::vector<double>& inputs,
                std::vector<double>& derivatives) -> bool {
                    return m_model->calculateDerivatives(state, inputs, derivatives);
                };

            if (!m_integrator->resizeBuffers(stateSize, inputSize) ||
                !m_integrator->initialize(stateSize, inputSize, derivFunc)) {
                return false;
            }

            // Выделение памяти для векторов состояния
            m_state_variables.resize(stateSize, 0.0);
            m_state_derivatives.resize(stateSize, 0.0);

            // Инициализация начальными значениями
            m_state_variables = m_model->getInitialState();
        }

        m_modelType = type;
        m_initialized = false;
        m_needRecalculate = true;

        return true;
    }

    bool SynchronousGenerator::setIntegrationType(IntegrationMethodType type) {
        std::lock_guard<std::mutex> lock(m_calculationMutex);

        if (m_integrationType == type) {
            return true; // Тип метода не изменился
        }

        // Создание нового метода интегрирования
        if (!createIntegrationMethod(type)) {
            return false;
        }

        m_integrationType = type;
        m_initialized = false;
        m_needRecalculate = true;

        return true;
    }

    double SynchronousGenerator::getNominalPower() const {
        return m_nominalPower;
    }

    void SynchronousGenerator::setNominalPower(double power) {
        m_nominalPower = power;
        m_needRecalculate = true;
    }

    double SynchronousGenerator::getNominalVoltage() const {
        return m_nominalVoltage;
    }

    void SynchronousGenerator::setNominalVoltage(double voltage) {
        m_nominalVoltage = voltage;
        m_needRecalculate = true;
    }

    void SynchronousGenerator::setElectricalParameters(
        double xd, double xq, double xd1, double xq1,
        double xd2, double xq2, double ra) {

        std::lock_guard<std::mutex> lock(m_calculationMutex);

        m_xd = xd;
        m_xq = xq;
        m_xd1 = xd1;
        m_xq1 = xq1;
        m_xd2 = xd2;
        m_xq2 = xq2;
        m_ra = ra;

        // Обновление параметров модели
        if (m_model) {
            std::map<std::string, double> params = m_model->getParameters();
            params["xd"] = m_xd;
            params["xq"] = m_xq;
            params["xd1"] = m_xd1;
            params["xq1"] = m_xq1;
            params["xd2"] = m_xd2;
            params["xq2"] = m_xq2;
            params["ra"] = m_ra;
            m_model->setParameters(params);
        }

        m_initialized = false;
        m_needRecalculate = true;
    }

    void SynchronousGenerator::setTimeConstants(
        double Td0, double Tq0, double Td1, double Tq1,
        double Td2, double Tq2) {

        std::lock_guard<std::mutex> lock(m_calculationMutex);

        m_Td0 = Td0;
        m_Tq0 = Tq0;
        m_Td1 = Td1;
        m_Tq1 = Tq1;
        m_Td2 = Td2;
        m_Tq2 = Tq2;

        // Обновление параметров модели
        if (m_model) {
            std::map<std::string, double> params = m_model->getParameters();
            params["Td0"] = m_Td0;
            params["Tq0"] = m_Tq0;
            params["Td1"] = m_Td1;
            params["Tq1"] = m_Tq1;
            params["Td2"] = m_Td2;
            params["Tq2"] = m_Tq2;
            m_model->setParameters(params);
        }

        m_initialized = false;
        m_needRecalculate = true;
    }

    void SynchronousGenerator::setMechanicalParameters(double H, double D) {
        std::lock_guard<std::mutex> lock(m_calculationMutex);

        m_inertiaConstant = H;
        m_dampingFactor = D;

        // Обновление параметров модели
        if (m_model) {
            std::map<std::string, double> params = m_model->getParameters();
            params["H"] = m_inertiaConstant;
            params["D"] = m_dampingFactor;
            m_model->setParameters(params);
        }

        m_initialized = false;
        m_needRecalculate = true;
    }

    GeneratorInterface SynchronousGenerator::getInterface() const {
        std::lock_guard<std::mutex> lock(m_interfaceMutex);
        // Возвращаем копию интерфейса
        return *m_primaryInterface;
    }

    void SynchronousGenerator::updateInterface(const GeneratorInterface& interface) {
        std::lock_guard<std::mutex> lock(m_interfaceMutex);
        // Обновляем вторичный буфер
        m_secondaryInterface->copyFrom(interface);
        // Обмен буферами
        swapInterfaces();
    }

    bool SynchronousGenerator::setExcitationRegulator(ExcitationRegulatorType type) {
        std::lock_guard<std::mutex> lock(m_calculationMutex);

        // Создание нового регулятора возбуждения
        bool result = createExcitationRegulator(type);

        if (result) {
            m_initialized = false;
            m_needRecalculate = true;
        }

        return result;
    }

    bool SynchronousGenerator::setTurbineRegulator(TurbineRegulatorType type) {
        std::lock_guard<std::mutex> lock(m_calculationMutex);

        // Создание нового регулятора турбины
        bool result = createTurbineRegulator(type);

        if (result) {
            m_initialized = false;
            m_needRecalculate = true;
        }

        return result;
    }

    bool SynchronousGenerator::initialize() {
        std::lock_guard<std::mutex> lock(m_calculationMutex);

        if (m_initialized) {
            return true;
        }

        try {
            // Проверка наличия компонентов
            if (!m_model || !m_integrator || !m_excitationRegulator || !m_turbineRegulator) {
                return false;
            }

            // Инициализация модели
            if (!m_model->initialize()) {
                return false;
            }

            // Установка параметров модели
            std::map<std::string, double> params;
            params["xd"] = m_xd;
            params["xq"] = m_xq;
            params["xd1"] = m_xd1;
            params["xq1"] = m_xq1;
            params["xd2"] = m_xd2;
            params["xq2"] = m_xq2;
            params["ra"] = m_ra;
            params["Td0"] = m_Td0;
            params["Tq0"] = m_Tq0;
            params["Td1"] = m_Td1;
            params["Tq1"] = m_Tq1;
            params["Td2"] = m_Td2;
            params["Tq2"] = m_Tq2;
            params["H"] = m_inertiaConstant;
            params["D"] = m_dampingFactor;

            if (!m_model->setParameters(params)) {
                return false;
            }

            // Инициализация интегратора
            int stateSize = m_model->getStateSize();
            int inputSize = m_model->getInputSize();

            auto derivFunc = [this](const std::vector<double>& state,
                const std::vector<double>& inputs,
                std::vector<double>& derivatives) -> bool {
                    // Проверка и изменение размера вектора derivatives
                    if (derivatives.size() != m_model->getStateSize()) {
                        derivatives.resize(m_model->getStateSize(), 0.0);
                    }
                    return m_model->calculateDerivatives(state, inputs, derivatives);
                };

            if (!m_integrator->initialize(stateSize, inputSize, derivFunc)) {
                return false;
            }

            // Инициализация регуляторов
            if (!m_excitationRegulator->initialize() || !m_turbineRegulator->initialize()) {
                return false;
            }

            // Выделение памяти для векторов состояния
            m_state_variables.resize(stateSize, 0.0);
            m_state_derivatives.resize(stateSize, 0.0);

            // Инициализация начальными значениями
            m_state_variables = m_model->getInitialState();

            m_initialized = true;
            m_needRecalculate = true;

            return true;
        }
        catch (const std::exception& e) {
            // Обработка исключений
            return false;
        }
    }

    bool SynchronousGenerator::compute(Types::TimeType time, Types::TimeType timeStep) {
        if (m_state.load() == GeneratorState::OFFLINE) {
            return true; // Генератор отключен, расчет не требуется
        }

        std::lock_guard<std::mutex> lock(m_calculationMutex);

        if (!m_initialized) {
            if (!initialize()) {
                return false;
            }
        }

        // Обновление входных данных от энергосистемы (из интерфейса)
        GeneratorInterface interface = getInterface();

        // Преобразование из интерфейса во входные данные для модели
        std::vector<double> modelInputs(m_model->getInputSize());

        // Получение напряжения в осях d-q
        Types::Complex voltage = interface.voltage;
        double delta = interface.rotorAngle;

        // Преобразование напряжения из фазных координат в оси d-q
        double vd = voltage.real() * std::sin(delta) + voltage.imag() * std::cos(delta);
        double vq = voltage.real() * std::cos(delta) - voltage.imag() * std::sin(delta);

        // Заполнение входных данных для модели
        modelInputs[0] = vd;              // Напряжение по оси d
        modelInputs[1] = vq;              // Напряжение по оси q
        modelInputs[2] = interface.excitationVoltage;  // Напряжение возбуждения
        modelInputs[3] = interface.activePower;        // Механическая мощность (предполагаем равной активной)
        modelInputs[4] = interface.frequency / 50.0;   // Относительная частота системы

        // Расчет одного шага интегрирования
        if (!m_integrator->step(m_state_variables, modelInputs, timeStep)) {
            return false;
        }

        // Расчет выходных переменных модели
        std::vector<double> modelOutputs(m_model->getOutputSize());
        if (!m_model->calculateOutputs(m_state_variables, modelInputs, modelOutputs)) {
            return false;
        }

        // Обновление интерфейса
        interface.rotorAngle = m_state_variables[0];  // Угол ротора
        interface.rotorSpeed = m_state_variables[1];  // Скорость ротора

        // Токи в осях d-q
        double id = modelOutputs[0];
        double iq = modelOutputs[1];

        // Преобразование токов из осей d-q в фазные координаты
        double iReal = id * std::sin(delta) + iq * std::cos(delta);
        double iImag = id * std::cos(delta) - iq * std::sin(delta);
        interface.current = Types::Complex(iReal, iImag);

        interface.activePower = modelOutputs[6];      // Активная мощность
        interface.reactivePower = modelOutputs[7];    // Реактивная мощность
        interface.excitationCurrent = modelOutputs[2]; // Ток возбуждения
        interface.timestamp = time;

        // Обновление интерфейса
        updateInterface(interface);

        return true;
    }

    bool SynchronousGenerator::predictorStep(Types::TimeType time, Types::TimeType timeStep) {
        // Реализация шага предиктора (используется в схеме предиктор-корректор)
        return compute(time, timeStep);
    }

    bool SynchronousGenerator::correctorStep(Types::TimeType time, Types::TimeType timeStep) {
        // Реализация шага корректора (используется в схеме предиктор-корректор)
        if (m_state.load() == GeneratorState::OFFLINE) {
            return true; // Генератор отключен, расчет не требуется
        }

        std::lock_guard<std::mutex> lock(m_calculationMutex);

        if (!m_initialized) {
            if (!initialize()) {
                return false;
            }
        }

        // Обновление входных данных от энергосистемы (из интерфейса)
        GeneratorInterface interface = getInterface();

        // Доступные актуальные данные после шага предиктора и расчета сети
        // Преобразование из интерфейса во входные данные для модели
        std::vector<double> modelInputs(m_model->getInputSize());

        // Получение напряжения в осях d-q (актуальное после предиктора)
        Types::Complex voltage = interface.voltage;
        double delta = interface.rotorAngle;

        // Преобразование напряжения из фазных координат в оси d-q
        double vd = voltage.real() * std::sin(delta) + voltage.imag() * std::cos(delta);
        double vq = voltage.real() * std::cos(delta) - voltage.imag() * std::sin(delta);

        // Заполнение входных данных для модели
        modelInputs[0] = vd;              // Напряжение по оси d
        modelInputs[1] = vq;              // Напряжение по оси q
        modelInputs[2] = interface.excitationVoltage;  // Напряжение возбуждения
        modelInputs[3] = interface.activePower;        // Механическая мощность (предполагаем равной активной)
        modelInputs[4] = interface.frequency / 50.0;   // Относительная частота системы

        // Расчет одного шага интегрирования (корректор)
        if (!m_integrator->step(m_state_variables, modelInputs, timeStep)) {
            return false;
        }

        // Расчет выходных переменных модели
        std::vector<double> modelOutputs(m_model->getOutputSize());
        if (!m_model->calculateOutputs(m_state_variables, modelInputs, modelOutputs)) {
            return false;
        }

        // Обновление интерфейса с уточненными данными
        interface.rotorAngle = m_state_variables[0];  // Угол ротора
        interface.rotorSpeed = m_state_variables[1];  // Скорость ротора

        // Токи в осях d-q
        double id = modelOutputs[0];
        double iq = modelOutputs[1];

        // Преобразование токов из осей d-q в фазные координаты
        double iReal = id * std::sin(delta) + iq * std::cos(delta);
        double iImag = id * std::cos(delta) - iq * std::sin(delta);
        interface.current = Types::Complex(iReal, iImag);

        interface.activePower = modelOutputs[6];      // Активная мощность
        interface.reactivePower = modelOutputs[7];    // Реактивная мощность
        interface.excitationCurrent = modelOutputs[2]; // Ток возбуждения
        interface.timestamp = time;

        // Обновление интерфейса
        updateInterface(interface);

        return true;
    }

    std::string SynchronousGenerator::serialize() const {
        std::stringstream ss;
        ss << PowerSystemElement::serialize();
        ss << "BUS_ID=" << m_busId << ";";
        ss << "MODEL_TYPE=" << static_cast<int>(m_modelType) << ";";
        ss << "INTEGRATION_TYPE=" << static_cast<int>(m_integrationType) << ";";
        ss << "STATE=" << static_cast<int>(m_state.load()) << ";";

        // Сериализация параметров генератора
        ss << "NOMINAL_POWER=" << m_nominalPower << ";";
        ss << "NOMINAL_VOLTAGE=" << m_nominalVoltage << ";";
        ss << "POWER_FACTOR=" << m_powerFactor << ";";
        ss << "NOMINAL_FREQUENCY=" << m_nominalFrequency << ";";
        ss << "INERTIA_CONSTANT=" << m_inertiaConstant << ";";
        ss << "DAMPING_FACTOR=" << m_dampingFactor << ";";

        // Электрические параметры
        ss << "RA=" << m_ra << ";";
        ss << "XD=" << m_xd << ";";
        ss << "XQ=" << m_xq << ";";
        ss << "XD1=" << m_xd1 << ";";
        ss << "XQ1=" << m_xq1 << ";";
        ss << "XD2=" << m_xd2 << ";";
        ss << "XQ2=" << m_xq2 << ";";

        // Постоянные времени
        ss << "TD0=" << m_Td0 << ";";
        ss << "TQ0=" << m_Tq0 << ";";
        ss << "TD1=" << m_Td1 << ";";
        ss << "TQ1=" << m_Tq1 << ";";
        ss << "TD2=" << m_Td2 << ";";
        ss << "TQ2=" << m_Tq2 << ";";

        return ss.str();
    }

    bool SynchronousGenerator::deserialize(const std::string& data) {
        if (!PowerSystemElement::deserialize(data)) {
            return false;
        }

        try {
            size_t pos = 0;
            std::string token;
            std::string str = data;

            while ((pos = str.find(";")) != std::string::npos) {
                token = str.substr(0, pos);

                size_t eqPos = token.find("=");
                if (eqPos != std::string::npos) {
                    std::string key = token.substr(0, eqPos);
                    std::string value = token.substr(eqPos + 1);

                    if (key == "BUS_ID") {
                        m_busId = std::stoull(value);
                    }
                    else if (key == "MODEL_TYPE") {
                        m_modelType = static_cast<GeneratorModelType>(std::stoi(value));
                    }
                    else if (key == "INTEGRATION_TYPE") {
                        m_integrationType = static_cast<IntegrationMethodType>(std::stoi(value));
                    }
                    else if (key == "STATE") {
                        m_state.store(static_cast<GeneratorState>(std::stoi(value)));
                    }
                    else if (key == "NOMINAL_POWER") {
                        m_nominalPower = std::stod(value);
                    }
                    else if (key == "NOMINAL_VOLTAGE") {
                        m_nominalVoltage = std::stod(value);
                    }
                    else if (key == "POWER_FACTOR") {
                        m_powerFactor = std::stod(value);
                    }
                    else if (key == "NOMINAL_FREQUENCY") {
                        m_nominalFrequency = std::stod(value);
                    }
                    else if (key == "INERTIA_CONSTANT") {
                        m_inertiaConstant = std::stod(value);
                    }
                    else if (key == "DAMPING_FACTOR") {
                        m_dampingFactor = std::stod(value);
                    }
                    else if (key == "RA") {
                        m_ra = std::stod(value);
                    }
                    else if (key == "XD") {
                        m_xd = std::stod(value);
                    }
                    else if (key == "XQ") {
                        m_xq = std::stod(value);
                    }
                    else if (key == "XD1") {
                        m_xd1 = std::stod(value);
                    }
                    else if (key == "XQ1") {
                        m_xq1 = std::stod(value);
                    }
                    else if (key == "XD2") {
                        m_xd2 = std::stod(value);
                    }
                    else if (key == "XQ2") {
                        m_xq2 = std::stod(value);
                    }
                    else if (key == "TD0") {
                        m_Td0 = std::stod(value);
                    }
                    else if (key == "TQ0") {
                        m_Tq0 = std::stod(value);
                    }
                    else if (key == "TD1") {
                        m_Td1 = std::stod(value);
                    }
                    else if (key == "TQ1") {
                        m_Tq1 = std::stod(value);
                    }
                    else if (key == "TD2") {
                        m_Td2 = std::stod(value);
                    }
                    else if (key == "TQ2") {
                        m_Tq2 = std::stod(value);
                    }
                }

                str.erase(0, pos + 1);
            }

            // Пересоздание компонентов
            createModel(m_modelType);
            createIntegrationMethod(m_integrationType);
            createExcitationRegulator(ExcitationRegulatorType::CONSTANT);
            createTurbineRegulator(TurbineRegulatorType::CONSTANT);

            m_initialized = false;
            m_needRecalculate = true;

            return true;
        }
        catch (const std::exception& e) {
            return false;
        }
    }

    bool SynchronousGenerator::createModel(GeneratorModelType type) {
        try {
            m_model = GeneratorModelFactory::createModel(type);
            return (m_model != nullptr);
        }
        catch (const std::exception& e) {
            // Обработка исключений
            return false;
        }
    }

    bool SynchronousGenerator::createIntegrationMethod(IntegrationMethodType type) {
        try {
            m_integrator = IntegrationMethodFactory::createMethod(type);

            if (m_integrator && m_model) {
                int stateSize = m_model->getStateSize();
                int inputSize = m_model->getInputSize();

                auto derivFunc = [this](const std::vector<double>& state,
                    const std::vector<double>& inputs,
                    std::vector<double>& derivatives) -> bool {
                        return m_model->calculateDerivatives(state, inputs, derivatives);
                    };

                return m_integrator->initialize(stateSize, inputSize, derivFunc);
            }

            return (m_integrator != nullptr);
        }
        catch (const std::exception& e) {
            // Обработка исключений
            return false;
        }
    }

    bool SynchronousGenerator::createExcitationRegulator(ExcitationRegulatorType type) {
        try {
            m_excitationRegulator = ExcitationRegulatorFactory::createRegulator(type);
            return (m_excitationRegulator != nullptr);
        }
        catch (const std::exception& e) {
            // Обработка исключений
            return false;
        }
    }

    bool SynchronousGenerator::createTurbineRegulator(TurbineRegulatorType type) {
        try {
            m_turbineRegulator = TurbineRegulatorFactory::createRegulator(type);
            return (m_turbineRegulator != nullptr);
        }
        catch (const std::exception& e) {
            // Обработка исключений
            return false;
        }
    }

    void SynchronousGenerator::swapInterfaces() {
        std::swap(m_primaryInterface, m_secondaryInterface);
    }

    // Реализация SynchronousGeneratorFactory
    std::shared_ptr<SynchronousGenerator> SynchronousGeneratorFactory::createGenerator(
        GeneratorModelType modelType,
        Types::ElementId id,
        const std::string& name,
        Types::ElementId busId) {

        auto generator = std::make_shared<SynchronousGenerator>(id, name, busId);

        if (generator) {
            generator->setModelType(modelType);

            // Установка метода интегрирования в зависимости от типа модели
            if (modelType == GeneratorModelType::SIMPLIFIED) {
                generator->setIntegrationType(IntegrationMethodType::EULER);
            }
            else {
                generator->setIntegrationType(IntegrationMethodType::RK4);
            }
        }

        return generator;
    }

} // namespace PowerSystem