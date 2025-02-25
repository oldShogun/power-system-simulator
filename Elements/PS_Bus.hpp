/**
 * @file PS_Bus.hpp
 * @brief Класс узла энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_BUS_HPP
#define PS_BUS_HPP

#include "../Core/PS_PowerSystemElement.hpp"
#include <vector>
#include <memory>

namespace PowerSystem {

    /**
     * @brief Типы узлов энергосистемы по классификации для расчета установившегося режима
     */
    enum class BusType {
        PQ,     ///< Узел с заданной активной и реактивной мощностью (нагрузочный)
        PV,     ///< Узел с заданной активной мощностью и напряжением (генераторный)
        SLACK   ///< Балансирующий узел
    };

    /**
     * @brief Класс узла энергосистемы
     */
    class Bus : public PowerSystemElement {
    private:
        BusType m_type;                            ///< Тип узла
        Types::Complex m_voltage;                  ///< Комплексное напряжение узла
        Types::Complex m_power;                    ///< Комплексная мощность узла
        std::vector<Types::ElementId> m_elements;  ///< Подключенные элементы

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        Bus();

        /**
         * @brief Конструктор с параметрами
         * @param id Идентификатор узла
         * @param name Название узла
         * @param type Тип узла
         */
        Bus(Types::ElementId id, const std::string& name, BusType type);

        /**
         * @brief Деструктор
         */
        ~Bus() override = default;

        /**
         * @brief Получить тип узла
         * @return Тип узла
         */
        BusType getType() const;

        /**
         * @brief Установить тип узла
         * @param type Новый тип узла
         */
        void setType(BusType type);

        /**
         * @brief Получить напряжение узла
         * @return Комплексное напряжение
         */
        Types::Complex getVoltage() const;

        /**
         * @brief Установить напряжение узла
         * @param voltage Комплексное напряжение
         */
        void setVoltage(const Types::Complex& voltage);

        /**
         * @brief Получить мощность узла
         * @return Комплексная мощность
         */
        Types::Complex getPower() const;

        /**
         * @brief Установить мощность узла
         * @param power Комплексная мощность
         */
        void setPower(const Types::Complex& power);

        /**
         * @brief Добавить элемент к узлу
         * @param elementId Идентификатор элемента
         */
        void addElement(Types::ElementId elementId);

        /**
         * @brief Удалить элемент из узла
         * @param elementId Идентификатор элемента
         * @return true если элемент был удален, false иначе
         */
        bool removeElement(Types::ElementId elementId);

        /**
         * @brief Получить список подключенных элементов
         * @return Вектор идентификаторов элементов
         */
        const std::vector<Types::ElementId>& getElements() const;

        // Переопределение методов базового класса
        bool initialize() override;
        bool compute(Types::TimeType time, Types::TimeType timeStep) override;
        std::string serialize() const override;
        bool deserialize(const std::string& data) override;
    };

} // namespace PowerSystem

#endif // PS_BUS_HPP