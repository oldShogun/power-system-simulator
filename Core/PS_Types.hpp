/**
 * @file PS_Types.hpp
 * @brief Основные типы данных и константы для системы моделирования энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_TYPES_HPP
#define PS_TYPES_HPP

#define _USE_MATH_DEFINES

#include <string>
#include <vector>
#include <map>
#include <complex>
#include <memory>
#include <cstdint>

namespace PowerSystem {

    /**
     * @brief Пространство имен для общих типов данных
     */
    namespace Types {

        /**
         * @brief Тип для идентификаторов элементов энергосистемы
         */
        using ElementId = uint64_t;

        /**
         * @brief Тип для времени моделирования (в секундах)
         */
        using TimeType = double;

        /**
         * @brief Тип для комплексных величин
         */
        using Complex = std::complex<double>;

        /**
         * @brief Тип матрицы полных проводимостей
         */
        using AdmittanceMatrix = std::map<std::pair<ElementId, ElementId>, Complex>;

        /**
         * @brief Перечисление состояний элемента энергосистемы
         */
        enum class ElementState {
            ENABLED,  ///< Элемент включен
            DISABLED, ///< Элемент отключен
            FAULT     ///< Элемент в аварийном состоянии
        };

        /**
         * @brief Перечисление режимов моделирования
         */
        enum class SimulationMode {
            STEADY_STATE,    ///< Установившийся режим
            DYNAMIC,         ///< Динамическое моделирование
            TRANSIENT        ///< Переходной режим
        };

        /**
         * @brief Перечисление уровней логирования
         */
        enum class LogLevel {
            DEBUG,   ///< Отладочная информация
            INFO,    ///< Информационные сообщения
            WARNING, ///< Предупреждения
            ERROR,   ///< Ошибки
            FATAL    ///< Критические ошибки
        };

    } // namespace Types

    /**
     * @brief Перечисление типов моделей синхронного генератора
     */
    enum class GeneratorModelType {
        SIMPLIFIED,       ///< Упрощенная модель (модель второго порядка)
        PARK_GOREV,       ///< Полная модель Парка-Горева (модель шестого порядка)
        CUSTOM            ///< Пользовательская модель
    };

    /**
     * @brief Перечисление состояний генератора
     */
    enum class GeneratorState {
        OFFLINE,          ///< Отключен от сети
        NORMAL,           ///< Нормальный режим работы
        ASYNCHRONOUS      ///< Асинхронный режим работы
    };

    /**
     * @brief Перечисление типов регуляторов возбуждения
     */
    enum class ExcitationRegulatorType {
        CONSTANT,         ///< Постоянное возбуждение
        IEEE_DC1A,        ///< Модель IEEE DC1A
        IEEE_AC1A,        ///< Модель IEEE AC1A
        CUSTOM            ///< Пользовательская модель
    };

    /**
     * @brief Перечисление типов регуляторов турбины
     */
    enum class TurbineRegulatorType {
        CONSTANT,         ///< Постоянная мощность
        SPEED_GOVERNOR,   ///< Регулятор скорости
        CUSTOM            ///< Пользовательская модель
    };

} // namespace PowerSystem

#endif // PS_TYPES_HPP