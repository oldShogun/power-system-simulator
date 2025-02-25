/**
 * @file PS_Interfaces.hpp
 * @brief Базовые интерфейсы системы моделирования энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_INTERFACES_HPP
#define PS_INTERFACES_HPP

#include "PS_Types.hpp"
#include <vector>
#include <string>

namespace PowerSystem {

    /**
     * @brief Интерфейс для идентифицируемого объекта
     */
    class IIdentifiable {
    public:
        /**
         * @brief Получить идентификатор объекта
         * @return Идентификатор объекта
         */
        virtual Types::ElementId getId() const = 0;

        /**
         * @brief Установить идентификатор объекта
         * @param id Новый идентификатор
         */
        virtual void setId(Types::ElementId id) = 0;

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IIdentifiable() = default;
    };

    /**
     * @brief Интерфейс для именуемого объекта
     */
    class INamed {
    public:
        /**
         * @brief Получить имя объекта
         * @return Имя объекта
         */
        virtual std::string getName() const = 0;

        /**
         * @brief Установить имя объекта
         * @param name Новое имя
         */
        virtual void setName(const std::string& name) = 0;

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~INamed() = default;
    };

    /**
     * @brief Интерфейс для расчетного объекта
     */
    class IComputable {
    public:
        /**
         * @brief Выполнить инициализацию перед расчетом
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool initialize() = 0;

        /**
         * @brief Выполнить расчет на текущем шаге моделирования
         * @param time Текущее время моделирования
         * @param timeStep Шаг моделирования
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool compute(Types::TimeType time, Types::TimeType timeStep) = 0;

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IComputable() = default;
    };

    /**
     * @brief Интерфейс для сериализуемого объекта
     */
    class ISerializable {
    public:
        /**
         * @brief Сериализовать объект в строку
         * @return Строковое представление объекта
         */
        virtual std::string serialize() const = 0;

        /**
         * @brief Десериализовать объект из строки
         * @param data Строковое представление объекта
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool deserialize(const std::string& data) = 0;

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~ISerializable() = default;
    };

} // namespace PowerSystem

#endif // PS_INTERFACES_HPP