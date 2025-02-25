/**
 * @file PS_Exception.hpp
 * @brief Классы исключений для системы моделирования энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_EXCEPTION_HPP
#define PS_EXCEPTION_HPP

#include <stdexcept>
#include <string>

namespace PowerSystem {

    /**
     * @brief Базовый класс исключений для системы моделирования
     */
    class PowerSystemException : public std::runtime_error {
    public:
        /**
         * @brief Конструктор
         * @param message Сообщение об ошибке
         */
        explicit PowerSystemException(const std::string& message)
            : std::runtime_error(message) {}
    };

    /**
     * @brief Исключение при ошибке инициализации
     */
    class InitializationException : public PowerSystemException {
    public:
        /**
         * @brief Конструктор
         * @param message Сообщение об ошибке
         */
        explicit InitializationException(const std::string& message)
            : PowerSystemException("Initialization error: " + message) {
        }
    };

    /**
     * @brief Исключение при ошибке расчета
     */
    class ComputationException : public PowerSystemException {
    public:
        /**
         * @brief Конструктор
         * @param message Сообщение об ошибке
         */
        explicit ComputationException(const std::string& message)
            : PowerSystemException("Computation error: " + message) {
        }
    };

    /**
     * @brief Исключение при ошибке доступа к данным
     */
    class DataAccessException : public PowerSystemException {
    public:
        /**
         * @brief Конструктор
         * @param message Сообщение об ошибке
         */
        explicit DataAccessException(const std::string& message)
            : PowerSystemException("Data access error: " + message) {
        }
    };

    /**
     * @brief Исключение при ошибке параметров
     */
    class InvalidParameterException : public PowerSystemException {
    public:
        /**
         * @brief Конструктор
         * @param message Сообщение об ошибке
         */
        explicit InvalidParameterException(const std::string& message)
            : PowerSystemException("Invalid parameter: " + message) {
        }
    };

} // namespace PowerSystem

#endif // PS_EXCEPTION_HPP