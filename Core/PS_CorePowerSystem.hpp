/**
 * @file PS_CorePowerSystem.hpp
 * @brief Ядро энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_CORE_POWER_SYSTEM_HPP
#define PS_CORE_POWER_SYSTEM_HPP

#include "PS_Types.hpp"
#include "PS_Interfaces.hpp"
#include "PS_PowerSystemElement.hpp"
#include "../Elements/PS_Bus.hpp"
#include "../Elements/PS_Branch.hpp"
#include "../Elements/PS_Generator.hpp"
#include "../Elements/PS_Load.hpp"
#include "../Elements/PS_SynchronousGenerator.hpp"
#include "../Utils/PS_Logger.hpp"
#include <memory>
#include <map>
#include <vector>
#include <unordered_map>
#include <string>

namespace PowerSystem {

    /**
     * @brief Класс ядра энергосистемы
     *
     * Этот класс представляет собой контейнер для всех элементов энергосистемы
     * и отвечает за выполнение расчетов и управление элементами.
     */
    class CorePowerSystem : public IComputable {
    private:
        std::string m_name;                          ///< Название энергосистемы
        std::shared_ptr<Logger> m_logger;            ///< Логгер

        // Хранилища элементов
        std::map<Types::ElementId, std::shared_ptr<Bus>> m_buses;               ///< Узлы
        std::map<Types::ElementId, std::shared_ptr<Branch>> m_branches;         ///< Ветви
        std::map<Types::ElementId, std::shared_ptr<Generator>> m_generators;    ///< Генераторы
        std::map<Types::ElementId, std::shared_ptr<Load>> m_loads;              ///< Нагрузки
        std::map<Types::ElementId, std::shared_ptr<SynchronousGenerator>> m_syncGenerators; ///< Синхронные генераторы

        Types::AdmittanceMatrix m_admittanceMatrix;  ///< Матрица проводимостей

        bool m_initialized;                          ///< Флаг инициализации
        Types::ElementId m_nextElementId;            ///< Следующий доступный идентификатор

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        CorePowerSystem();

        /**
         * @brief Конструктор с параметрами
         * @param name Название энергосистемы
         * @param logger Указатель на логгер
         */
        CorePowerSystem(const std::string& name, std::shared_ptr<Logger> logger);

        /**
         * @brief Деструктор
         */
        ~CorePowerSystem();

        /**
         * @brief Получить название энергосистемы
         * @return Название энергосистемы
         */
        std::string getName() const;

        /**
         * @brief Установить название энергосистемы
         * @param name Новое название
         */
        void setName(const std::string& name);

        /**
         * @brief Установить логгер
         * @param logger Указатель на логгер
         */
        void setLogger(std::shared_ptr<Logger> logger);

        /**
         * @brief Добавить узел в энергосистему
         * @param bus Указатель на узел
         * @return true в случае успеха, false в случае ошибки
         */
        bool addBus(std::shared_ptr<Bus> bus);

        /**
         * @brief Добавить ветвь в энергосистему
         * @param branch Указатель на ветвь
         * @return true в случае успеха, false в случае ошибки
         */
        bool addBranch(std::shared_ptr<Branch> branch);

        /**
         * @brief Добавить генератор в энергосистему
         * @param generator Указатель на генератор
         * @return true в случае успеха, false в случае ошибки
         */
        bool addGenerator(std::shared_ptr<Generator> generator);

        /**
         * @brief Добавить нагрузку в энергосистему
         * @param load Указатель на нагрузку
         * @return true в случае успеха, false в случае ошибки
         */
        bool addLoad(std::shared_ptr<Load> load);

        /**
         * @brief Добавить синхронный генератор в энергосистему
         * @param generator Указатель на синхронный генератор
         * @return true в случае успеха, false в случае ошибки
         */
        bool addSynchronousGenerator(std::shared_ptr<SynchronousGenerator> generator);

        /**
         * @brief Удалить узел из энергосистемы
         * @param id Идентификатор узла
         * @return true в случае успеха, false в случае ошибки
         */
        bool removeBus(Types::ElementId id);

        /**
         * @brief Удалить ветвь из энергосистемы
         * @param id Идентификатор ветви
         * @return true в случае успеха, false в случае ошибки
         */
        bool removeBranch(Types::ElementId id);

        /**
         * @brief Удалить генератор из энергосистемы
         * @param id Идентификатор генератора
         * @return true в случае успеха, false в случае ошибки
         */
        bool removeGenerator(Types::ElementId id);

        /**
         * @brief Удалить нагрузку из энергосистемы
         * @param id Идентификатор нагрузки
         * @return true в случае успеха, false в случае ошибки
         */
        bool removeLoad(Types::ElementId id);

        /**
         * @brief Удалить синхронный генератор из энергосистемы
         * @param id Идентификатор синхронного генератора
         * @return true в случае успеха, false в случае ошибки
         */
        bool removeSynchronousGenerator(Types::ElementId id);

        /**
         * @brief Получить узел по идентификатору
         * @param id Идентификатор узла
         * @return Указатель на узел или nullptr, если узел не найден
         */
        std::shared_ptr<Bus> getBus(Types::ElementId id) const;

        /**
         * @brief Получить ветвь по идентификатору
         * @param id Идентификатор ветви
         * @return Указатель на ветвь или nullptr, если ветвь не найдена
         */
        std::shared_ptr<Branch> getBranch(Types::ElementId id) const;

        /**
         * @brief Получить генератор по идентификатору
         * @param id Идентификатор генератора
         * @return Указатель на генератор или nullptr, если генератор не найден
         */
        std::shared_ptr<Generator> getGenerator(Types::ElementId id) const;

        /**
         * @brief Получить нагрузку по идентификатору
         * @param id Идентификатор нагрузки
         * @return Указатель на нагрузку или nullptr, если нагрузка не найдена
         */
        std::shared_ptr<Load> getLoad(Types::ElementId id) const;

        /**
         * @brief Получить синхронный генератор по идентификатору
         * @param id Идентификатор синхронного генератора
         * @return Указатель на синхронный генератор или nullptr, если не найден
         */
        std::shared_ptr<SynchronousGenerator> getSynchronousGenerator(Types::ElementId id) const;

        /**
         * @brief Получить все узлы
         * @return Карта идентификаторов и указателей на узлы
         */
        const std::map<Types::ElementId, std::shared_ptr<Bus>>& getBuses() const;

        /**
         * @brief Получить все ветви
         * @return Карта идентификаторов и указателей на ветви
         */
        const std::map<Types::ElementId, std::shared_ptr<Branch>>& getBranches() const;

        /**
         * @brief Получить все генераторы
         * @return Карта идентификаторов и указателей на генераторы
         */
        const std::map<Types::ElementId, std::shared_ptr<Generator>>& getGenerators() const;

        /**
         * @brief Получить все нагрузки
         * @return Карта идентификаторов и указателей на нагрузки
         */
        const std::map<Types::ElementId, std::shared_ptr<Load>>& getLoads() const;

        /**
         * @brief Получить все синхронные генераторы
         * @return Карта идентификаторов и указателей на синхронные генераторы
         */
        const std::map<Types::ElementId, std::shared_ptr<SynchronousGenerator>>& getSynchronousGenerators() const;

        /**
         * @brief Получить матрицу проводимостей
         * @return Матрица проводимостей
         */
        const Types::AdmittanceMatrix& getAdmittanceMatrix() const;

        /**
         * @brief Создать новый узел
         * @param name Название узла
         * @param type Тип узла
         * @return Указатель на созданный узел
         */
        std::shared_ptr<Bus> createBus(const std::string& name, BusType type);

        /**
         * @brief Создать новую ветвь
         * @param name Название ветви
         * @param fromBusId Идентификатор узла начала ветви
         * @param toBusId Идентификатор узла конца ветви
         * @return Указатель на созданную ветвь
         */
        std::shared_ptr<Branch> createBranch(const std::string& name,
            Types::ElementId fromBusId,
            Types::ElementId toBusId);

        /**
         * @brief Создать новый генератор
         * @param name Название генератора
         * @param busId Идентификатор узла подключения
         * @return Указатель на созданный генератор
         */
        std::shared_ptr<Generator> createGenerator(const std::string& name,
            Types::ElementId busId);

        /**
         * @brief Создать новую нагрузку
         * @param name Название нагрузки
         * @param busId Идентификатор узла подключения
         * @return Указатель на созданную нагрузку
         */
        std::shared_ptr<Load> createLoad(const std::string& name,
            Types::ElementId busId);

        /**
         * @brief Создать новый синхронный генератор
         * @param name Название синхронного генератора
         * @param busId Идентификатор узла подключения
         * @param modelType Тип модели генератора
         * @return Указатель на созданный синхронный генератор
         */
        std::shared_ptr<SynchronousGenerator> createSynchronousGenerator(
            const std::string& name,
            Types::ElementId busId,
            GeneratorModelType modelType = GeneratorModelType::SIMPLIFIED);

        /**
         * @brief Построить матрицу проводимостей
         * @return true в случае успеха, false в случае ошибки
         */
        bool buildAdmittanceMatrix();

        /**
         * @brief Обновить интерфейсы синхронных генераторов
         * @return true в случае успеха, false в случае ошибки
         */
        bool updateSynchronousGeneratorInterfaces();

        /**
         * @brief Сохранить модель в файл
         * @param filename Имя файла
         * @return true в случае успеха, false в случае ошибки
         */
        bool saveToFile(const std::string& filename) const;

        /**
         * @brief Загрузить модель из файла
         * @param filename Имя файла
         * @return true в случае успеха, false в случае ошибки
         */
        bool loadFromFile(const std::string& filename);

        /**
         * @brief Очистить все элементы энергосистемы
         */
        void clear();

        // Реализация IComputable
        bool initialize() override;
        bool compute(Types::TimeType time, Types::TimeType timeStep) override;

        /**
         * @brief Шаг предиктора для модели с синхронными генераторами
         * @param time Текущее время
         * @param timeStep Шаг по времени
         * @return true в случае успеха, false в случае ошибки
         */
        bool predictorStep(Types::TimeType time, Types::TimeType timeStep);

        /**
         * @brief Шаг корректора для модели с синхронными генераторами
         * @param time Текущее время
         * @param timeStep Шаг по времени
         * @return true в случае успеха, false в случае ошибки
         */
        bool correctorStep(Types::TimeType time, Types::TimeType timeStep);

    private:
        /**
         * @brief Сгенерировать новый уникальный идентификатор элемента
         * @return Новый идентификатор
         */
        Types::ElementId generateId();
    };

} // namespace PowerSystem

#endif // PS_CORE_POWER_SYSTEM_HPP