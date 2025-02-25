/**
 * @file PS_IntegrationMethods.hpp
 * @brief Интерфейс и классы методов интегрирования
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_INTEGRATION_METHODS_HPP
#define PS_INTEGRATION_METHODS_HPP

#include <vector>
#include <memory>
#include <functional>
#include <deque>

namespace PowerSystem {

    // Перечисление типов методов интегрирования
    enum class IntegrationMethodType {
        EULER,            ///< Метод Эйлера
        RK4,              ///< Метод Рунге-Кутта 4 порядка
        ADAMS_BASHFORTH   ///< Метод Адамса-Башфорта-Мультона 4 порядка
    };

    /**
     * @brief Тип функции расчета производных
     */
    using DerivativeFunction = std::function<bool(
        const std::vector<double>&,  // state
        const std::vector<double>&,  // inputs
        std::vector<double>&         // derivatives
        )>;

    /**
     * @brief Интерфейс метода интегрирования
     */
    class IIntegrationMethod {
    public:
        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IIntegrationMethod() = default;

        /**
         * @brief Инициализация метода
         * @param stateSize Размер вектора состояния
         * @param inputSize Размер вектора входов
         * @param derivFunc Функция расчета производных
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool initialize(int stateSize, int inputSize,
            DerivativeFunction derivFunc) = 0;

        /**
         * @brief Выполнить один шаг интегрирования
         * @param state Текущее состояние (входной/выходной параметр)
         * @param inputs Текущие входы
         * @param dt Шаг интегрирования
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool step(std::vector<double>& state,
            const std::vector<double>& inputs,
            double dt) = 0;

        /**
         * @brief Получить погрешность метода
         * @param dt Шаг интегрирования
         * @return Оценка погрешности
         */
        virtual double getError(double dt) const = 0;

        /**
         * @brief Получить порядок метода
         * @return Порядок метода
         */
        virtual int getOrder() const = 0;

        /**
         * @brief Изменить размерность буферов
         * @param stateSize Новый размер вектора состояния
         * @param inputSize Новый размер вектора входов
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool resizeBuffers(int stateSize, int inputSize) = 0;

        /**
         * @brief Сбросить состояние интегратора
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool reset() = 0;
    };

    /**
     * @brief Базовый класс метода интегрирования
     */
    class BaseIntegrationMethod : public IIntegrationMethod {
    protected:
        int m_stateSize;                     ///< Размер вектора состояния
        int m_inputSize;                     ///< Размер вектора входов
        DerivativeFunction m_derivFunc;      ///< Функция расчета производных
        bool m_initialized;                  ///< Флаг инициализации

    public:
        /**
         * @brief Конструктор
         */
        BaseIntegrationMethod();

        /**
         * @brief Деструктор
         */
        virtual ~BaseIntegrationMethod() override = default;

        /**
         * @brief Инициализация метода
         * @param stateSize Размер вектора состояния
         * @param inputSize Размер вектора входов
         * @param derivFunc Функция расчета производных
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize(int stateSize, int inputSize,
            DerivativeFunction derivFunc) override;

        /**
         * @brief Изменить размерность буферов
         * @param stateSize Новый размер вектора состояния
         * @param inputSize Новый размер вектора входов
         * @return true в случае успеха, false в случае ошибки
         */
        bool resizeBuffers(int stateSize, int inputSize) override;

        /**
         * @brief Сбросить состояние интегратора
         * @return true в случае успеха, false в случае ошибки
         */
        bool reset() override;
    };

    /**
     * @brief Метод интегрирования Эйлера
     */
    class EulerMethod : public BaseIntegrationMethod {
    protected:
        std::vector<double> m_derivatives;   ///< Буфер для производных

    public:
        /**
         * @brief Конструктор
         */
        EulerMethod();

        /**
         * @brief Деструктор
         */
        ~EulerMethod() override = default;

        /**
         * @brief Выполнить один шаг интегрирования
         * @param state Текущее состояние (входной/выходной параметр)
         * @param inputs Текущие входы
         * @param dt Шаг интегрирования
         * @return true в случае успеха, false в случае ошибки
         */
        bool step(std::vector<double>& state,
            const std::vector<double>& inputs,
            double dt) override;

        /**
         * @brief Получить погрешность метода
         * @param dt Шаг интегрирования
         * @return Оценка погрешности
         */
        double getError(double dt) const override;

        /**
         * @brief Получить порядок метода
         * @return Порядок метода
         */
        int getOrder() const override;

        /**
         * @brief Изменить размерность буферов
         * @param stateSize Новый размер вектора состояния
         * @param inputSize Новый размер вектора входов
         * @return true в случае успеха, false в случае ошибки
         */
        bool resizeBuffers(int stateSize, int inputSize) override;
    };

    /**
     * @brief Метод интегрирования Рунге-Кутты 4-го порядка
     */
    class RungeKutta4Method : public BaseIntegrationMethod {
    protected:
        std::vector<double> m_k1;            ///< Буфер для k1
        std::vector<double> m_k2;            ///< Буфер для k2
        std::vector<double> m_k3;            ///< Буфер для k3
        std::vector<double> m_k4;            ///< Буфер для k4
        std::vector<double> m_tempState;     ///< Временный буфер для состояния

    public:
        /**
         * @brief Конструктор
         */
        RungeKutta4Method();

        /**
         * @brief Деструктор
         */
        ~RungeKutta4Method() override = default;

        /**
         * @brief Выполнить один шаг интегрирования
         * @param state Текущее состояние (входной/выходной параметр)
         * @param inputs Текущие входы
         * @param dt Шаг интегрирования
         * @return true в случае успеха, false в случае ошибки
         */
        bool step(std::vector<double>& state,
            const std::vector<double>& inputs,
            double dt) override;

        /**
         * @brief Инициализация метода интегрирования
         * @param stateSize Размер вектора состояния
         * @param inputSize Размер вектора входов
         * @param derivFunc Функция расчета производных
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize(int stateSize, int inputSize, DerivativeFunction derivFunc);

        /**
         * @brief Получить погрешность метода
         * @param dt Шаг интегрирования
         * @return Оценка погрешности
         */
        double getError(double dt) const override;

        /**
         * @brief Получить порядок метода
         * @return Порядок метода
         */
        int getOrder() const override;

        /**
         * @brief Изменить размерность буферов
         * @param stateSize Новый размер вектора состояния
         * @param inputSize Новый размер вектора входов
         * @return true в случае успеха, false в случае ошибки
         */
        bool resizeBuffers(int stateSize, int inputSize) override;
    };

    /**
     * @brief Метод интегрирования Адамса-Башфорта-Мультона 4-го порядка (предиктор-корректор)
     */
    class AdamsBashforthMoultonMethod : public BaseIntegrationMethod {
    protected:
        std::deque<std::vector<double>> m_stateHistory;       ///< История состояний
        std::deque<std::vector<double>> m_derivativesHistory; ///< История производных
        std::vector<double> m_tempState;                     ///< Временный буфер для состояния
        std::vector<double> m_tempDerivatives;               ///< Временный буфер для производных
        int m_historySize;                                   ///< Размер истории

    public:
        /**
         * @brief Конструктор
         */
        AdamsBashforthMoultonMethod();

        /**
         * @brief Деструктор
         */
        ~AdamsBashforthMoultonMethod() override = default;

        /**
         * @brief Инициализация метода
         * @param stateSize Размер вектора состояния
         * @param inputSize Размер вектора входов
         * @param derivFunc Функция расчета производных
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize(int stateSize, int inputSize,
            DerivativeFunction derivFunc) override;

        /**
         * @brief Выполнить один шаг интегрирования
         * @param state Текущее состояние (входной/выходной параметр)
         * @param inputs Текущие входы
         * @param dt Шаг интегрирования
         * @return true в случае успеха, false в случае ошибки
         */
        bool step(std::vector<double>& state,
            const std::vector<double>& inputs,
            double dt) override;

        /**
         * @brief Получить погрешность метода
         * @param dt Шаг интегрирования
         * @return Оценка погрешности
         */
        double getError(double dt) const override;

        /**
         * @brief Получить порядок метода
         * @return Порядок метода
         */
        int getOrder() const override;

        /**
         * @brief Изменить размерность буферов
         * @param stateSize Новый размер вектора состояния
         * @param inputSize Новый размер вектора входов
         * @return true в случае успеха, false в случае ошибки
         */
        bool resizeBuffers(int stateSize, int inputSize) override;

        /**
         * @brief Сбросить состояние интегратора
         * @return true в случае успеха, false в случае ошибки
         */
        bool reset() override;

    protected:
        /**
         * @brief Инициализировать историю при помощи метода RK4
         * @param state Текущее состояние
         * @param inputs Текущие входы
         * @param dt Шаг интегрирования
         * @return true в случае успеха, false в случае ошибки
         */
        bool initializeHistory(std::vector<double>& state,
            const std::vector<double>& inputs,
            double dt);
    };

    /**
     * @brief Фабрика методов интегрирования
     */
    class IntegrationMethodFactory {
    public:
        /**
         * @brief Создать метод интегрирования
         * @param type Тип метода интегрирования
         * @return Указатель на созданный метод
         */
        static std::unique_ptr<IIntegrationMethod> createMethod(IntegrationMethodType type);
    };

} // namespace PowerSystem

#endif // PS_INTEGRATION_METHODS_HPP