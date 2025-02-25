/**
 * @file PS_PowerSystemElement.hpp
 * @brief Базовый класс элемента энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_POWER_SYSTEM_ELEMENT_HPP
#define PS_POWER_SYSTEM_ELEMENT_HPP

#include "PS_Types.hpp"
#include "PS_Interfaces.hpp"
#include "PS_Exception.hpp"
#include <string>
#include <map>

namespace PowerSystem {

    /**
     * @brief Базовый класс для всех элементов энергосистемы
     */
    class PowerSystemElement :
        public IIdentifiable,
        public INamed,
        public IComputable,
        public ISerializable {
    protected:
        Types::ElementId m_id;                        ///< Идентификатор элемента
        std::string m_name;                           ///< Название элемента
        Types::ElementState m_state;                  ///< Состояние элемента
        std::map<std::string, double> m_parameters;   ///< Параметры элемента

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        PowerSystemElement();

        /**
         * @brief Конструктор с параметрами
         * @param id Идентификатор элемента
         * @param name Название элемента
         */
        PowerSystemElement(Types::ElementId id, const std::string& name);

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~PowerSystemElement() = default;

        // Реализация IIdentifiable
        Types::ElementId getId() const override;
        void setId(Types::ElementId id) override;

        // Реализация INamed
        std::string getName() const override;
        void setName(const std::string& name) override;

        // Реализация IComputable
        bool initialize() override;
        bool compute(Types::TimeType time, Types::TimeType timeStep) override;

        // Реализация ISerializable
        std::string serialize() const override;
        bool deserialize(const std::string& data) override;

        /**
         * @brief Получить состояние элемента
         * @return Текущее состояние элемента
         */
        Types::ElementState getState() const;

        /**
         * @brief Установить состояние элемента
         * @param state Новое состояние
         */
        void setState(Types::ElementState state);

        /**
         * @brief Установить параметр элемента
         * @param name Имя параметра
         * @param value Значение параметра
         */
        void setParameter(const std::string& name, double value);

        /**
         * @brief Получить параметр элемента
         * @param name Имя параметра
         * @return Значение параметра
         * @throw DataAccessException если параметр не найден
         */
        double getParameter(const std::string& name) const;

        /**
         * @brief Проверить, существует ли параметр
         * @param name Имя параметра
         * @return true если параметр существует, false иначе
         */
        bool hasParameter(const std::string& name) const;
    };

} // namespace PowerSystem

#endif // PS_POWER_SYSTEM_ELEMENT_HPP