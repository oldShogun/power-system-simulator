/**
 * @file PS_Regulators.hpp
 * @brief Интерфейсы и базовые классы регуляторов
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_REGULATORS_HPP
#define PS_REGULATORS_HPP

#include "../Core/PS_Types.hpp"
#include <memory>
#include <map>
#include <string>
#include <vector>

namespace PowerSystem {

    /**
     * @brief Базовый интерфейс регулятора
     *
     * Определяет общий интерфейс для всех типов регуляторов в системе.
     */
    class IRegulator {
    public:
        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IRegulator() = default;

        /**
         * @brief Инициализация регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool initialize() = 0;

        /**
         * @brief Выполнить один шаг расчета регулятора
         * @param inputs Входные переменные
         * @param outputs Выходные переменные (результат)
         * @param dt Шаг по времени
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool step(const std::vector<double>& inputs,
            std::vector<double>& outputs,
            double dt) = 0;

        /**
         * @brief Получить размер вектора входных переменных
         * @return Размер вектора входных переменных
         */
        virtual int getInputSize() const = 0;

        /**
         * @brief Получить размер вектора выходных переменных
         * @return Размер вектора выходных переменных
         */
        virtual int getOutputSize() const = 0;

        /**
         * @brief Установить параметры регулятора
         * @param parameters Карта параметров
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool setParameters(const std::map<std::string, double>& parameters) = 0;

        /**
         * @brief Получить текущие параметры регулятора
         * @return Карта параметров
         */
        virtual std::map<std::string, double> getParameters() const = 0;

        /**
         * @brief Получить тип регулятора
         * @return Строковое представление типа регулятора
         */
        virtual std::string getType() const = 0;

        /**
         * @brief Сбросить состояние регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool reset() = 0;
    };

    /**
     * @brief Базовый класс регулятора
     *
     * Предоставляет общую реализацию для всех регуляторов в системе.
     */
    class BaseRegulator : public virtual IRegulator {
    protected:
        bool m_initialized;                        ///< Флаг инициализации
        std::map<std::string, double> m_parameters; ///< Параметры регулятора
        std::vector<double> m_state;               ///< Внутреннее состояние регулятора

    public:
        /**
         * @brief Конструктор
         */
        BaseRegulator();

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~BaseRegulator() override = default;

        /**
         * @brief Установить параметры регулятора
         * @param parameters Карта параметров
         * @return true в случае успеха, false в случае ошибки
         */
        bool setParameters(const std::map<std::string, double>& parameters) override;

        /**
         * @brief Получить текущие параметры регулятора
         * @return Карта параметров
         */
        std::map<std::string, double> getParameters() const override;

        /**
         * @brief Сбросить состояние регулятора
         * @return true в случае успеха, false в случае ошибки
         */
        bool reset() override;

    protected:
        /**
         * @brief Проверить наличие необходимых параметров
         * @param requiredParams Вектор названий необходимых параметров
         * @return true если все параметры присутствуют, false иначе
         */
        bool checkRequiredParameters(const std::vector<std::string>& requiredParams) const;
    };

    /**
     * @brief Интерфейс регулятора возбуждения
     *
     * Расширяет базовый интерфейс для регуляторов возбуждения генераторов.
     */
    class IExcitationRegulator : public virtual IRegulator {
    public:
        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IExcitationRegulator() = default;

        /**
         * @brief Получить тип регулятора возбуждения
         * @return Тип регулятора возбуждения
         */
        virtual ExcitationRegulatorType getExcitationType() const = 0;
    };

    /**
     * @brief Интерфейс регулятора турбины
     *
     * Расширяет базовый интерфейс для регуляторов турбины.
     */
    class ITurbineRegulator : public virtual IRegulator {
    public:
        /**
         * @brief Виртуальный деструктор
         */
        virtual ~ITurbineRegulator() = default;

        /**
         * @brief Получить тип регулятора турбины
         * @return Тип регулятора турбины
         */
        virtual TurbineRegulatorType getTurbineType() const = 0;
    };

    /**
     * @brief Базовый класс регулятора возбуждения
     *
     * Предоставляет базовую реализацию для регуляторов возбуждения генераторов.
     */
    class BaseExcitationRegulator : public BaseRegulator, public virtual IExcitationRegulator {
    public:
        /**
         * @brief Конструктор
         */
        BaseExcitationRegulator();

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~BaseExcitationRegulator() override = default;
    };

    /**
     * @brief Базовый класс регулятора турбины
     *
     * Предоставляет базовую реализацию для регуляторов турбины.
     */
    class BaseTurbineRegulator : public BaseRegulator, public virtual ITurbineRegulator {
    public:
        /**
         * @brief Конструктор
         */
        BaseTurbineRegulator();

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~BaseTurbineRegulator() override = default;
    };

    /**
     * @brief Фабрика регуляторов возбуждения
     */
    class ExcitationRegulatorFactory {
    public:
        /**
         * @brief Создать регулятор возбуждения
         * @param type Тип регулятора возбуждения
         * @return Указатель на созданный регулятор
         */
        static std::unique_ptr<IExcitationRegulator> createRegulator(ExcitationRegulatorType type);
    };

    /**
     * @brief Фабрика регуляторов турбины
     */
    class TurbineRegulatorFactory {
    public:
        /**
         * @brief Создать регулятор турбины
         * @param type Тип регулятора турбины
         * @return Указатель на созданный регулятор
         */
        static std::unique_ptr<ITurbineRegulator> createRegulator(TurbineRegulatorType type);
    };

} // namespace PowerSystem

#endif // PS_REGULATORS_HPP