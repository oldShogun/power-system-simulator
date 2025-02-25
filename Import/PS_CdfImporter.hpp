/**
 * @file PS_CdfImporter.hpp
 * @brief Класс импорта данных из CDF формата
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_CDF_IMPORTER_HPP
#define PS_CDF_IMPORTER_HPP

#include "../Core/PS_CorePowerSystem.hpp"
#include "../Utils/PS_Logger.hpp"
#include <string>
#include <memory>

namespace PowerSystem {

/**
 * @brief Класс для импорта данных энергосистемы из формата CDF
 */
class CdfImporter {
private:
    std::shared_ptr<Logger> m_logger;  ///< Логгер

public:
    /**
     * @brief Конструктор
     * @param logger Указатель на логгер
     */
    explicit CdfImporter(std::shared_ptr<Logger> logger = nullptr);

    /**
     * @brief Импорт данных из CDF файла в модель энергосистемы
     * @param filename Имя файла
     * @param system Указатель на модель энергосистемы
     * @return true в случае успеха, false в случае ошибки
     */
    bool importFromFile(const std::string& filename, std::shared_ptr<CorePowerSystem> system);

private:
    /**
     * @brief Обработка секции с информацией о узлах
     * @param line Строка с данными
     * @param system Указатель на модель энергосистемы
     * @return true в случае успеха, false в случае ошибки
     */
    bool processBusData(const std::string& line, std::shared_ptr<CorePowerSystem> system);

    /**
     * @brief Обработка секции с информацией о ветвях
     * @param line Строка с данными
     * @param system Указатель на модель энергосистемы
     * @return true в случае успеха, false в случае ошибки
     */
    bool processBranchData(const std::string& line, std::shared_ptr<CorePowerSystem> system);

    /**
     * @brief Обработка секции с информацией о генераторах
     * @param line Строка с данными
     * @param system Указатель на модель энергосистемы
     * @return true в случае успеха, false в случае ошибки
     */
    bool processGeneratorData(const std::string& line, std::shared_ptr<CorePowerSystem> system);

    /**
     * @brief Преобразование строки CDF в числовое значение
     * @param str Строка для преобразования
     * @param defaultValue Значение по умолчанию
     * @return Преобразованное числовое значение
     */
    double parseValue(const std::string& str, double defaultValue = 0.0);
};

} // namespace PowerSystem

#endif // PS_CDF_IMPORTER_HPP