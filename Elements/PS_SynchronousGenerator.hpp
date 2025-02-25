/**
 * @file PS_SynchronousGenerator.hpp
 * @brief Класс синхронного генератора
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_SYNCHRONOUS_GENERATOR_HPP
#define PS_SYNCHRONOUS_GENERATOR_HPP

#include "../Core/PS_PowerSystemElement.hpp"
#include "../Core/PS_Types.hpp"
#include "../Core/PS_Exception.hpp"
#include "../Models/PS_GeneratorModels.hpp"
#include "../Integration/PS_IntegrationMethods.hpp"
#include "../Regulators/PS_Regulators.hpp"
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>

namespace PowerSystem {

    /**
     * @brief Структура для обмена данными генератора с энергосистемой
     */
    struct GeneratorInterface {
        // Электрические параметры
        Types::Complex voltage;        ///< Комплексное напряжение на выводах генератора
        Types::Complex current;        ///< Комплексный ток
        double frequency;              ///< Частота на выводах генератора

        // Механические параметры
        double activePower;            ///< Активная мощность
        double reactivePower;          ///< Реактивная мощность
        double rotorAngle;             ///< Угол ротора
        double rotorSpeed;             ///< Скорость вращения ротора

        // Параметры возбуждения
        double excitationVoltage;      ///< Напряжение возбуждения
        double excitationCurrent;      ///< Ток возбуждения

        // Состояние и временная метка
        GeneratorState state;          ///< Состояние генератора
        Types::TimeType timestamp;     ///< Временная метка

        /**
         * @brief Конструктор по умолчанию
         */
        GeneratorInterface();

        /**
         * @brief Копирование данных из другого интерфейса
         * @param other Другой интерфейс
         */
        void copyFrom(const GeneratorInterface& other);
    };

    /**
     * @brief Класс синхронного генератора
     */
    class SynchronousGenerator : public PowerSystemElement {
    private:
        // Параметры и состояние генератора
        Types::ElementId m_busId;                                    ///< ID узла подключения
        GeneratorModelType m_modelType;                              ///< Тип модели
        IntegrationMethodType m_integrationType;                     ///< Тип метода интегрирования
        std::atomic<GeneratorState> m_state;                         ///< Текущее состояние

        // Основные параметры генератора
        double m_nominalPower;                                       ///< Номинальная мощность, МВА
        double m_nominalVoltage;                                     ///< Номинальное напряжение, кВ
        double m_powerFactor;                                        ///< Коэффициент мощности
        double m_nominalFrequency;                                   ///< Номинальная частота, Гц
        double m_inertiaConstant;                                    ///< Постоянная инерции H, с
        double m_dampingFactor;                                      ///< Коэффициент демпфирования D

        // Электрические параметры (о.е.)
        double m_ra;                                                 ///< Активное сопротивление статора
        double m_xd;                                                 ///< Синхронное реактивное сопротивление по оси d
        double m_xq;                                                 ///< Синхронное реактивное сопротивление по оси q
        double m_xd1;                                                ///< Переходное реактивное сопротивление по оси d
        double m_xq1;                                                ///< Переходное реактивное сопротивление по оси q
        double m_xd2;                                                ///< Сверхпереходное реактивное сопротивление по оси d
        double m_xq2;                                                ///< Сверхпереходное реактивное сопротивление по оси q

        // Постоянные времени (с)
        double m_Td0;                                                ///< Постоянная времени обмотки возбуждения при разомкнутой обмотке статора
        double m_Tq0;                                                ///< Постоянная времени обмотки по оси q при разомкнутой обмотке статора
        double m_Td1;                                                ///< Переходная постоянная времени по оси d при короткозамкнутой обмотке статора
        double m_Tq1;                                                ///< Переходная постоянная времени по оси q при короткозамкнутой обмотке статора
        double m_Td2;                                                ///< Сверхпереходная постоянная времени по оси d
        double m_Tq2;                                                ///< Сверхпереходная постоянная времени по оси q

        // Компоненты модели
        std::unique_ptr<IGeneratorModel> m_model;                    ///< Модель генератора
        std::unique_ptr<IIntegrationMethod> m_integrator;            ///< Метод интегрирования
        std::unique_ptr<IExcitationRegulator> m_excitationRegulator; ///< Регулятор возбуждения
        std::unique_ptr<ITurbineRegulator> m_turbineRegulator;       ///< Регулятор турбины

        // Интерфейс с энергосистемой и буферы
        std::unique_ptr<GeneratorInterface> m_primaryInterface;      ///< Основной интерфейс
        std::unique_ptr<GeneratorInterface> m_secondaryInterface;    ///< Вторичный интерфейс (буфер)
        mutable std::mutex m_interfaceMutex;                         ///< Мьютекс для синхронизации доступа к интерфейсу

        // Состояние и переменные
        std::vector<double> m_state_variables;                       ///< Вектор переменных состояния
        std::vector<double> m_state_derivatives;                     ///< Вектор производных

        // Флаги и параметры расчета
        bool m_initialized;                                          ///< Флаг инициализации
        bool m_needRecalculate;                                      ///< Флаг необходимости пересчета

        // Мьютекс для потокобезопасности
        mutable std::mutex m_calculationMutex;                       ///< Мьютекс для расчетов

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        SynchronousGenerator();

        /**
         * @brief Конструктор с параметрами
         * @param id Идентификатор генератора
         * @param name Название генератора
         * @param busId Идентификатор узла подключения
         */
        SynchronousGenerator(Types::ElementId id, const std::string& name, Types::ElementId busId);

        /**
         * @brief Деструктор
         */
        ~SynchronousGenerator() override;

        /**
         * @brief Получить идентификатор узла подключения
         * @return Идентификатор узла
         */
        Types::ElementId getBusId() const;

        /**
         * @brief Установить идентификатор узла подключения
         * @param busId Идентификатор узла
         */
        void setBusId(Types::ElementId busId);

        /**
         * @brief Получить состояние генератора
         * @return Состояние генератора
         */
        GeneratorState getState() const;

        /**
         * @brief Установить состояние генератора
         * @param state Новое состояние
         */
        void setState(GeneratorState state);

        /**
         * @brief Получить тип модели генератора
         * @return Тип модели
         */
        GeneratorModelType getModelType() const;

        /**
         * @brief Установить тип модели генератора
         * @param type Тип модели
         * @return true в случае успеха, false в случае ошибки
         */
        bool setModelType(GeneratorModelType type);

        /**
         * @brief Получить тип метода интегрирования
         * @return Тип метода интегрирования
         */
        IntegrationMethodType getIntegrationType() const;

        /**
         * @brief Установить тип метода интегрирования
         * @param type Тип метода интегрирования
         * @return true в случае успеха, false в случае ошибки
         */
        bool setIntegrationType(IntegrationMethodType type);

        /**
         * @brief Получить номинальную мощность генератора
         * @return Номинальная мощность, МВА
         */
        double getNominalPower() const;

        /**
         * @brief Установить номинальную мощность генератора
         * @param power Номинальная мощность, МВА
         */
        void setNominalPower(double power);

        /**
         * @brief Получить номинальное напряжение генератора
         * @return Номинальное напряжение, кВ
         */
        double getNominalVoltage() const;

        /**
         * @brief Установить номинальное напряжение генератора
         * @param voltage Номинальное напряжение, кВ
         */
        void setNominalVoltage(double voltage);

        /**
         * @brief Установить параметры генератора
         * @param xd Синхронное реактивное сопротивление по оси d
         * @param xq Синхронное реактивное сопротивление по оси q
         * @param xd1 Переходное реактивное сопротивление по оси d
         * @param xq1 Переходное реактивное сопротивление по оси q
         * @param xd2 Сверхпереходное реактивное сопротивление по оси d
         * @param xq2 Сверхпереходное реактивное сопротивление по оси q
         * @param ra Активное сопротивление статора
         */
        void setElectricalParameters(double xd, double xq, double xd1, double xq1,
            double xd2, double xq2, double ra);

        /**
         * @brief Установить постоянные времени генератора
         * @param Td0 Постоянная времени обмотки возбуждения при разомкнутой обмотке статора
         * @param Tq0 Постоянная времени обмотки по оси q при разомкнутой обмотке статора
         * @param Td1 Переходная постоянная времени по оси d при короткозамкнутой обмотке статора
         * @param Tq1 Переходная постоянная времени по оси q при короткозамкнутой обмотке статора
         * @param Td2 Сверхпереходная постоянная времени по оси d
         * @param Tq2 Сверхпереходная постоянная времени по оси q
         */
        void setTimeConstants(double Td0, double Tq0, double Td1, double Tq1,
            double Td2, double Tq2);

        /**
         * @brief Установить механические параметры генератора
         * @param H Постоянная инерции, с
         * @param D Коэффициент демпфирования
         */
        void setMechanicalParameters(double H, double D);

        /**
         * @brief Получить интерфейс генератора (потокобезопасно)
         * @return Копия интерфейса генератора
         */
        GeneratorInterface getInterface() const;

        /**
         * @brief Обновить интерфейс генератора (потокобезопасно)
         * @param interface Новые данные интерфейса
         */
        void updateInterface(const GeneratorInterface& interface);

        /**
         * @brief Установить регулятор возбуждения
         * @param type Тип регулятора возбуждения
         * @return true в случае успеха, false в случае ошибки
         */
        bool setExcitationRegulator(ExcitationRegulatorType type);

        /**
         * @brief Установить регулятор турбины
         * @param type Тип регулятора турбины
         * @return true в случае успеха, false в случае ошибки
         */
        bool setTurbineRegulator(TurbineRegulatorType type);

        /**
         * @brief Пересчитать промежуточное состояние (шаг предиктора)
         * @param time Текущее время моделирования
         * @param timeStep Шаг моделирования
         * @return true в случае успеха, false в случае ошибки
         */
        bool predictorStep(Types::TimeType time, Types::TimeType timeStep);

        /**
         * @brief Уточнить состояние с учетом новых данных (шаг корректора)
         * @param time Текущее время моделирования
         * @param timeStep Шаг моделирования
         * @return true в случае успеха, false в случае ошибки
         */
        bool correctorStep(Types::TimeType time, Types::TimeType timeStep);

        // Переопределение методов базового класса
        bool initialize() override;
        bool compute(Types::TimeType time, Types::TimeType timeStep) override;
        std::string serialize() const override;
        bool deserialize(const std::string& data) override;

    private:
        /**
         * @brief Создать модель генератора
         * @param type Тип модели
         * @return true в случае успеха, false в случае ошибки
         */
        bool createModel(GeneratorModelType type);

        /**
         * @brief Создать метод интегрирования
         * @param type Тип метода интегрирования
         * @return true в случае успеха, false в случае ошибки
         */
        bool createIntegrationMethod(IntegrationMethodType type);

        /**
         * @brief Создать регулятор возбуждения
         * @param type Тип регулятора возбуждения
         * @return true в случае успеха, false в случае ошибки
         */
        bool createExcitationRegulator(ExcitationRegulatorType type);

        /**
         * @brief Создать регулятор турбины
         * @param type Тип регулятора турбины
         * @return true в случае успеха, false в случае ошибки
         */
        bool createTurbineRegulator(TurbineRegulatorType type);

        /**
         * @brief Обмен буферами интерфейса
         */
        void swapInterfaces();
    };

    /**
     * @brief Фабрика для создания синхронных генераторов
     */
    class SynchronousGeneratorFactory {
    public:
        /**
         * @brief Создать синхронный генератор
         * @param modelType Тип модели генератора
         * @param id Идентификатор генератора
         * @param name Название генератора
         * @param busId Идентификатор узла подключения
         * @return Указатель на созданный генератор
         */
        static std::shared_ptr<SynchronousGenerator> createGenerator(
            GeneratorModelType modelType,
            Types::ElementId id,
            const std::string& name,
            Types::ElementId busId);
    };

} // namespace PowerSystem

#endif // PS_SYNCHRONOUS_GENERATOR_HPP