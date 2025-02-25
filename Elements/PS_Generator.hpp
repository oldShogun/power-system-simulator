/**
 * @file PS_Generator.hpp
 * @brief Базовый класс генератора энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_GENERATOR_HPP
#define PS_GENERATOR_HPP

#include "../Core/PS_PowerSystemElement.hpp"
#include <memory>

namespace PowerSystem {

    /**
     * @brief Базовый класс генератора энергосистемы
     */
    class Generator : public PowerSystemElement {
    private:
        Types::ElementId m_busId;            ///< Идентификатор узла подключения
        Types::Complex m_nominalPower;       ///< Номинальная мощность генератора
        double m_voltage;                    ///< Заданное напряжение
        double m_activePower;                ///< Активная мощность
        double m_reactivePower;              ///< Реактивная мощность
        double m_maxReactivePower;           ///< Максимальная реактивная мощность
        double m_minReactivePower;           ///< Минимальная реактивная мощность

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        Generator();

        /**
         * @brief Конструктор с параметрами
         * @param id Идентификатор генератора
         * @param name Название генератора
         * @param busId Идентификатор узла подключения
         */
        Generator(Types::ElementId id, const std::string& name, Types::ElementId busId);

        /**
         * @brief Деструктор
         */
        ~Generator() override = default;

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
         * @brief Получить номинальную мощность генератора
         * @return Комплексная номинальная мощность
         */
        Types::Complex getNominalPower() const;

        /**
         * @brief Установить номинальную мощность генератора
         * @param power Комплексная номинальная мощность
         */
        void setNominalPower(const Types::Complex& power);

        /**
         * @brief Получить заданное напряжение
         * @return Заданное напряжение
         */
        double getVoltage() const;

        /**
         * @brief Установить заданное напряжение
         * @param voltage Заданное напряжение
         */
        void setVoltage(double voltage);

        /**
         * @brief Получить активную мощность
         * @return Активная мощность
         */
        double getActivePower() const;

        /**
         * @brief Установить активную мощность
         * @param power Активная мощность
         */
        void setActivePower(double power);

        /**
         * @brief Получить реактивную мощность
         * @return Реактивная мощность
         */
        double getReactivePower() const;

        /**
         * @brief Установить реактивную мощность
         * @param power Реактивная мощность
         */
        void setReactivePower(double power);

        /**
         * @brief Получить максимальную реактивную мощность
         * @return Максимальная реактивная мощность
         */
        double getMaxReactivePower() const;

        /**
         * @brief Установить максимальную реактивную мощность
         * @param power Максимальная реактивная мощность
         */
        void setMaxReactivePower(double power);

        /**
         * @brief Получить минимальную реактивную мощность
         * @return Минимальная реактивная мощность
         */
        double getMinReactivePower() const;

        /**
         * @brief Установить минимальную реактивную мощность
         * @param power Минимальная реактивная мощность
         */
        void setMinReactivePower(double power);

        // Переопределение методов базового класса
        bool initialize() override;
        bool compute(Types::TimeType time, Types::TimeType timeStep) override;
        std::string serialize() const override;
        bool deserialize(const std::string& data) override;
    };

} // namespace PowerSystem

#endif // PS_GENERATOR_HPP