/**
 * @file PS_Logger.hpp
 * @brief Система логирования для энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_LOGGER_HPP
#define PS_LOGGER_HPP

#include "../Core/PS_Types.hpp"
#include <string>
#include <fstream>
#include <mutex>
#include <functional>
#include <vector>

namespace PowerSystem {

    /**
     * @brief Класс для логирования сообщений системы
     */
    class Logger {
    private:
        std::string m_logFilePath;                ///< Путь к файлу логов
        std::ofstream m_logFile;                  ///< Поток файла логов
        Types::LogLevel m_logLevel;               ///< Минимальный уровень логирования
        bool m_consoleOutput;                     ///< Флаг вывода в консоль
        std::mutex m_logMutex;                    ///< Мьютекс для потокобезопасности
        std::vector<std::function<void(Types::LogLevel, const std::string&)>> m_callbacks;  ///< Колбэки логирования

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        Logger();

        /**
         * @brief Конструктор с параметрами
         * @param logFilePath Путь к файлу логов
         * @param level Минимальный уровень логирования
         * @param consoleOutput Флаг вывода в консоль
         */
        Logger(const std::string& logFilePath,
            Types::LogLevel level = Types::LogLevel::INFO,
            bool consoleOutput = true);

        /**
         * @brief Деструктор
         */
        ~Logger();

        /**
         * @brief Открыть файл логов
         * @param logFilePath Путь к файлу логов
         * @return true в случае успеха, false в случае ошибки
         */
        bool openLogFile(const std::string& logFilePath);

        /**
         * @brief Закрыть файл логов
         */
        void closeLogFile();

        /**
         * @brief Установить уровень логирования
         * @param level Минимальный уровень логирования
         */
        void setLogLevel(Types::LogLevel level);

        /**
         * @brief Получить текущий уровень логирования
         * @return Уровень логирования
         */
        Types::LogLevel getLogLevel() const;

        /**
         * @brief Установить флаг вывода в консоль
         * @param enable Включить/выключить вывод в консоль
         */
        void setConsoleOutput(bool enable);

        /**
         * @brief Проверить флаг вывода в консоль
         * @return true если вывод в консоль включен, false иначе
         */
        bool isConsoleOutput() const;

        /**
         * @brief Добавить колбэк для логирования
         * @param callback Функция колбэка, принимающая уровень логирования и сообщение
         */
        void addCallback(std::function<void(Types::LogLevel, const std::string&)> callback);

        /**
         * @brief Очистить все колбэки
         */
        void clearCallbacks();

        /**
         * @brief Записать сообщение в лог
         * @param level Уровень логирования
         * @param message Сообщение для записи
         */
        void log(Types::LogLevel level, const std::string& message);

        /**
         * @brief Записать отладочное сообщение
         * @param message Сообщение для записи
         */
        void debug(const std::string& message);

        /**
         * @brief Записать информационное сообщение
         * @param message Сообщение для записи
         */
        void info(const std::string& message);

        /**
         * @brief Записать предупреждение
         * @param message Сообщение для записи
         */
        void warning(const std::string& message);

        /**
         * @brief Записать сообщение об ошибке
         * @param message Сообщение для записи
         */
        void error(const std::string& message);

        /**
         * @brief Записать сообщение о критической ошибке
         * @param message Сообщение для записи
         */
        void fatal(const std::string& message);

    private:
        /**
         * @brief Получить строковое представление уровня логирования
         * @param level Уровень логирования
         * @return Строковое представление
         */
        std::string getLevelString(Types::LogLevel level) const;

        /**
         * @brief Получить текущую временную метку
         * @return Строка с текущей датой и временем
         */
        std::string getTimestamp() const;
    };

} // namespace PowerSystem

#endif // PS_LOGGER_HPP