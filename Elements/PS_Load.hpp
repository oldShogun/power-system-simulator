/**
 * @file PS_Load.hpp
 * @brief Класс нагрузки энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_LOAD_HPP
#define PS_LOAD_HPP

#include "../Core/PS_PowerSystemElement.hpp"

namespace PowerSystem {

    /**
     * @brief Типы моделей нагрузки
     */
    enum class LoadModel {
        CONSTANT_POWER,     ///< Нагрузка постоянной мощности (S = const)
        CONSTANT_CURRENT,   ///< Нагрузка постоянного тока (I = const)
        CONSTANT_IMPEDANCE, ///< Нагрузка постоянного сопротивления (Z = const)
        POLYNOMIAL,         ///< Полиномиальная модель (ZIP)
        FREQUENCY_DEPENDENT ///< Частотно-зависимая модель
    };

    /**
     * @brief Класс нагрузки энергосистемы
     */
    class Load : public PowerSystemElement {
    private:
        Types::ElementId m_busId;            ///< Идентификатор узла подключения
        LoadModel m_model;                   ///< Модель нагрузки
        Types::Complex m_power;              ///< Мощность нагрузки
        double m_powerFactor;                ///< Коэффициент мощности
        std::map<std::string, double> m_modelParams; ///< Параметры модели

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        Load();

        /**
         * @brief Конструктор с параметрами
         * @param id Идентификатор нагрузки
         * @param name Название нагрузки
         * @param busId Идентификатор узла подключения
         */
        Load(Types::ElementId id, const std::string& name, Types::ElementId busId);

        /**
         * @brief Деструктор
         */
        ~Load() override = default;

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
         * @brief Получить модель нагрузки
         * @return Модель нагрузки
         */
        LoadModel getModel() const;

        /**
         * @brief Установить модель нагрузки
         * @param model Модель нагрузки
         */
        void setModel(LoadModel model);

        /**
         * @brief Получить мощность нагрузки
         * @return Комплексная мощность
         */
        Types::Complex getPower() const;

        /**
         * @brief Установить мощность нагрузки
         * @param power Комплексная мощность
         */
        void setPower(const Types::Complex& power);

        /**
         * @brief Получить коэффициент мощности
         * @return Коэффициент мощности
         */
        double getPowerFactor() const;

        /**
         * @brief Установить коэффициент мощности
         * @param factor Коэффициент мощности
         */
        void setPowerFactor(double factor);

        /**
         * @brief Установить параметр модели
         * @param name Имя параметра
         * @param value Значение параметра
         */
        void setModelParameter(const std::string& name, double value);

        /**
         * @brief Получить параметр модели
         * @param name Имя параметра
         * @return Значение параметра
         * @throw DataAccessException если параметр не найден
         */
        double getModelParameter(const std::string& name) const;

        /**
         * @brief Проверить наличие параметра модели
         * @param name Имя параметра
         * @return true если параметр существует, false иначе
         */
        bool hasModelParameter(const std::string& name) const;

        // Переопределение методов базового класса
        bool initialize() override;
        bool compute(Types::TimeType time, Types::TimeType timeStep) override;
        std::string serialize() const override;
        bool deserialize(const std::string& data) override;
    };

} // namespace PowerSystem

#endif // PS_LOAD_HPP