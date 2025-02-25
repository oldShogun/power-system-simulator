/**
 * @file PS_ExcitationRegulators.hpp
 * @brief Реализация регуляторов возбуждения
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_EXCITATION_REGULATORS_HPP
#define PS_EXCITATION_REGULATORS_HPP

#include "PS_Regulators.hpp"
#include <deque>

namespace PowerSystem {

    /**
     * @brief Регулятор с постоянным возбуждением
     *
     * Простейший регулятор возбуждения, поддерживающий постоянное напряжение возбуждения.
     */
    class ConstantExcitationRegulator : public BaseExcitationRegulator {
    public:
        /**
         * @brief Конструктор
         */
        ConstantExcitationRegulator();

        /**
         * @brief Деструктор
         */
        ~ConstantExcitationRegulator() override = default;

        /**
         * @brief Инициализация регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize() override;

        /**
         * @brief Выполнить один шаг расчета регулятора
         * @param inputs Входные переменные (0 - напряжение генератора, 1 - уставка напряжения)
         * @param outputs Выходные переменные (0 - напряжение возбуждения)
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
         * @brief Получить тип регулятора возбуждения
         * @return Тип регулятора возбуждения
         */
        ExcitationRegulatorType getExcitationType() const override;

        /**
         * @brief Сбросить состояние регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool reset() override;
    };

    /**
     * @brief Регулятор возбуждения IEEE DC1A
     *
     * Реализация стандартного регулятора возбуждения типа IEEE DC1A.
     */
    class IEEEDC1AExcitationRegulator : public BaseExcitationRegulator {
    protected:
        double m_vref;                  ///< Уставка напряжения
        double m_vr;                    ///< Выход регулятора напряжения
        double m_efd;                   ///< Выходное напряжение возбуждения

    public:
        /**
         * @brief Конструктор
         */
        IEEEDC1AExcitationRegulator();

        /**
         * @brief Деструктор
         */
        ~IEEEDC1AExcitationRegulator() override = default;

        /**
         * @brief Инициализация регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize() override;

        /**
         * @brief Выполнить один шаг расчета регулятора
         * @param inputs Входные переменные (0 - напряжение генератора, 1 - ток возбуждения)
         * @param outputs Выходные переменные (0 - напряжение возбуждения)
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
         * @brief Получить тип регулятора возбуждения
         * @return Тип регулятора возбуждения
         */
        ExcitationRegulatorType getExcitationType() const override;

        /**
         * @brief Установить уставку напряжения
         * @param vref Уставка напряжения
         */
        void setVoltageSetpoint(double vref);

        /**
         * @brief Сбросить состояние регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool reset() override;
    };

} // namespace PowerSystem

#endif // PS_EXCITATION_REGULATORS_HPP