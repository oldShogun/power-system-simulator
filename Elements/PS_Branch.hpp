/**
 * @file PS_Branch.hpp
 * @brief Класс ветви энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_BRANCH_HPP
#define PS_BRANCH_HPP

#include "../Core/PS_PowerSystemElement.hpp"

namespace PowerSystem {

    /**
     * @brief Класс ветви энергосистемы (линия, трансформатор)
     */
    class Branch : public PowerSystemElement {
    private:
        Types::ElementId m_fromBusId;    ///< Идентификатор узла начала ветви
        Types::ElementId m_toBusId;      ///< Идентификатор узла конца ветви
        Types::Complex m_impedance;      ///< Комплексное сопротивление ветви
        Types::Complex m_admittance;     ///< Комплексная проводимость ветви
        double m_ratedCurrent;           ///< Номинальный ток
        Types::Complex m_current;        ///< Текущий ток ветви
        Types::Complex m_power;          ///< Текущая мощность ветви

    public:
        /**
         * @brief Конструктор по умолчанию
         */
        Branch();

        /**
         * @brief Конструктор с параметрами
         * @param id Идентификатор ветви
         * @param name Название ветви
         * @param fromBusId Идентификатор узла начала ветви
         * @param toBusId Идентификатор узла конца ветви
         */
        Branch(Types::ElementId id, const std::string& name,
            Types::ElementId fromBusId, Types::ElementId toBusId);

        /**
         * @brief Деструктор
         */
        ~Branch() override = default;

        /**
         * @brief Получить идентификатор узла начала ветви
         * @return Идентификатор узла начала
         */
        Types::ElementId getFromBusId() const;

        /**
         * @brief Установить идентификатор узла начала ветви
         * @param busId Идентификатор узла начала
         */
        void setFromBusId(Types::ElementId busId);

        /**
         * @brief Получить идентификатор узла конца ветви
         * @return Идентификатор узла конца
         */
        Types::ElementId getToBusId() const;

        /**
         * @brief Установить идентификатор узла конца ветви
         * @param busId Идентификатор узла конца
         */
        void setToBusId(Types::ElementId busId);

        /**
         * @brief Получить комплексное сопротивление ветви
         * @return Комплексное сопротивление
         */
        Types::Complex getImpedance() const;

        /**
         * @brief Установить комплексное сопротивление ветви
         * @param impedance Комплексное сопротивление
         */
        void setImpedance(const Types::Complex& impedance);

        /**
         * @brief Получить комплексную проводимость ветви
         * @return Комплексная проводимость
         */
        Types::Complex getAdmittance() const;

        /**
         * @brief Установить комплексную проводимость ветви
         * @param admittance Комплексная проводимость
         */
        void setAdmittance(const Types::Complex& admittance);

        /**
         * @brief Получить номинальный ток ветви
         * @return Номинальный ток
         */
        double getRatedCurrent() const;

        /**
         * @brief Установить номинальный ток ветви
         * @param current Номинальный ток
         */
        void setRatedCurrent(double current);

        /**
         * @brief Получить текущий ток ветви
         * @return Комплексный ток
         */
        Types::Complex getCurrent() const;

        /**
         * @brief Установить текущий ток ветви
         * @param current Комплексный ток
         */
        void setCurrent(const Types::Complex& current);

        /**
         * @brief Получить текущую мощность ветви
         * @return Комплексная мощность
         */
        Types::Complex getPower() const;

        /**
         * @brief Установить текущую мощность ветви
         * @param power Комплексная мощность
         */
        void setPower(const Types::Complex& power);

        // Переопределение методов базового класса
        bool initialize() override;
        bool compute(Types::TimeType time, Types::TimeType timeStep) override;
        std::string serialize() const override;
        bool deserialize(const std::string& data) override;
    };

} // namespace PowerSystem

#endif // PS_BRANCH_HPP