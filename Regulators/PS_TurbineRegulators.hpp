/**
 * @file PS_TurbineRegulators.hpp
 * @brief Реализация регуляторов турбины
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_TURBINE_REGULATORS_HPP
#define PS_TURBINE_REGULATORS_HPP

#include "PS_Regulators.hpp"
#include <deque>

namespace PowerSystem {

    /**
     * @brief Регулятор с постоянной мощностью
     *
     * Простейший регулятор турбины, поддерживающий постоянную механическую мощность.
     */
    class ConstantPowerTurbineRegulator : public BaseTurbineRegulator {
    public:
        /**
         * @brief Конструктор
         */
        ConstantPowerTurbineRegulator();

        /**
         * @brief Деструктор
         */
        ~ConstantPowerTurbineRegulator() override = default;

        /**
         * @brief Инициализация регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize() override;

        /**
         * @brief Выполнить один шаг расчета регулятора
         * @param inputs Входные переменные (0 - частота, 1 - уставка мощности)
         * @param outputs Выходные переменные (0 - механическая мощность)
         * @param dt Шаг по времени
         * @return true в случае успеха, false в случае ошибки
         */
        bool step(const std::vector<double>& inputs,
            std::vector<double>& outputs,
            double dt) override;

        /**
         * @brief Получить размер вектора входных переменных
         * @return Размер вектора входных переменных
         */
        int getInputSize() const override;

        /**
         * @brief Получить размер вектора выходных переменных
         * @return Размер вектора выходных переменных
         */
        int getOutputSize() const override;

        /**
         * @brief Получить тип регулятора
         * @return Строковое представление типа регулятора
         */
        std::string getType() const override;

        /**
         * @brief Получить тип регулятора турбины
         * @return Тип регулятора турбины
         */
        TurbineRegulatorType getTurbineType() const override;

        /**
         * @brief Сбросить состояние регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool reset() override;
    };

    /**
     * @brief Регулятор скорости турбины
     *
     * Регулятор, управляющий скоростью турбины с учетом статизма и инерции системы.
     */
    class SpeedGovernorTurbineRegulator : public BaseTurbineRegulator {
    protected:
        double m_pSetpoint;             ///< Уставка мощности
        double m_omega0;                ///< Номинальная частота
        double m_pm;                    ///< Механическая мощность
        double m_pv;                    ///< Положение клапана

    public:
        /**
         * @brief Конструктор
         */
        SpeedGovernorTurbineRegulator();

        /**
         * @brief Деструктор
         */
        ~SpeedGovernorTurbineRegulator() override = default;

        /**
         * @brief Инициализация регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize() override;

        /**
         * @brief Выполнить один шаг расчета регулятора
         * @param inputs Входные переменные (0 - частота, 1 - уставка мощности)
         * @param outputs Выходные переменные (0 - механическая мощность)
         * @param dt Шаг по времени
         * @return true в случае успеха, false в случае ошибки
         */
        bool step(const std::vector<double>& inputs,
            std::vector<double>& outputs,
            double dt) override;

        /**
         * @brief Получить размер вектора входных переменных
         * @return Размер вектора входных переменных
         */
        int getInputSize() const override;

        /**
         * @brief Получить размер вектора выходных переменных
         * @return Размер вектора выходных переменных
         */
        int getOutputSize() const override;

        /**
         * @brief Получить тип регулятора
         * @return Строковое представление типа регулятора
         */
        std::string getType() const override;

        /**
         * @brief Получить тип регулятора турбины
         * @return Тип регулятора турбины
         */
        TurbineRegulatorType getTurbineType() const override;

        /**
         * @brief Установить уставку мощности
         * @param pSetpoint Уставка мощности
         */
        void setPowerSetpoint(double pSetpoint);

        /**
         * @brief Сбросить состояние регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool reset() override;
    };

} // namespace PowerSystem

#endif // PS_TURBINE_REGULATORS_HPP