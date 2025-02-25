/**
 * @file PS_Logger.cpp
 * @brief Реализация системы логирования для энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_Logger.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace PowerSystem {

    Logger::Logger()
        : m_logLevel(Types::LogLevel::INFO),
        m_consoleOutput(true) {
    }

    Logger::Logger(const std::string& logFilePath,
        Types::LogLevel level,
        bool consoleOutput)
        : m_logLevel(level),
        m_consoleOutput(consoleOutput) {
        openLogFile(logFilePath);
    }

    Logger::~Logger() {
        closeLogFile();
    }

    bool Logger::openLogFile(const std::string& logFilePath) {
        std::lock_guard<std::mutex> lock(m_logMutex);

        // Сначала закрываем старый файл, если открыт
        if (m_logFile.is_open()) {
            m_logFile.close();
        }

        m_logFilePath = logFilePath;
        m_logFile.open(m_logFilePath, std::ios::app);

        if (!m_logFile.is_open()) {
            std::cerr << "Failed to open log file: " << m_logFilePath << std::endl;
            return false;
        }

        return true;
    }

    void Logger::closeLogFile() {
        std::lock_guard<std::mutex> lock(m_logMutex);

        if (m_logFile.is_open()) {
            m_logFile.close();
        }
    }

    void Logger::setLogLevel(Types::LogLevel level) {
        std::lock_guard<std::mutex> lock(m_logMutex);
        m_logLevel = level;
    }

    Types::LogLevel Logger::getLogLevel() const {
        return m_logLevel;
    }

    void Logger::setConsoleOutput(bool enable) {
        std::lock_guard<std::mutex> lock(m_logMutex);
        m_consoleOutput = enable;
    }

    bool Logger::isConsoleOutput() const {
        return m_consoleOutput;
    }

    void Logger::addCallback(std::function<void(Types::LogLevel, const std::string&)> callback) {
        std::lock_guard<std::mutex> lock(m_logMutex);
        m_callbacks.push_back(callback);
    }

    void Logger::clearCallbacks() {
        std::lock_guard<std::mutex> lock(m_logMutex);
        m_callbacks.clear();
    }

    void Logger::log(Types::LogLevel level, const std::string& message) {
        // Фильтрация по уровню логирования
        if (level < m_logLevel) {
            return;
        }

        std::string timestamp = getTimestamp();
        std::string levelStr = getLevelString(level);
        std::string logMessage = timestamp + " [" + levelStr + "] " + message;

        std::lock_guard<std::mutex> lock(m_logMutex);

        // Запись в файл
        if (m_logFile.is_open()) {
            m_logFile << logMessage << std::endl;
            m_logFile.flush();
        }

        // Вывод в консоль
        if (m_consoleOutput) {
            if (level == Types::LogLevel::ERROR || level == Types::LogLevel::FATAL) {
                std::cerr << logMessage << std::endl;
            }
            else {
                std::cout << logMessage << std::endl;
            }
        }

        // Вызов колбэков
        for (const auto& callback : m_callbacks) {
            callback(level, message);
        }
    }

    void Logger::debug(const std::string& message) {
        log(Types::LogLevel::DEBUG, message);
    }

    void Logger::info(const std::string& message) {
        log(Types::LogLevel::INFO, message);
    }

    void Logger::warning(const std::string& message) {
        log(Types::LogLevel::WARNING, message);
    }

    void Logger::error(const std::string& message) {
        log(Types::LogLevel::ERROR, message);
    }

    void Logger::fatal(const std::string& message) {
        log(Types::LogLevel::FATAL, message);
    }

    std::string Logger::getLevelString(Types::LogLevel level) const {
        switch (level) {
        case Types::LogLevel::DEBUG:
            return "DEBUG";
        case Types::LogLevel::INFO:
            return "INFO";
        case Types::LogLevel::WARNING:
            return "WARNING";
        case Types::LogLevel::ERROR:
            return "ERROR";
        case Types::LogLevel::FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
        }
    }

    std::string Logger::getTimestamp() const {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }

} // namespace PowerSystem