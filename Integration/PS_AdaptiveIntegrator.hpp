/**
 * @file PS_AdaptiveIntegrator.hpp
 * @brief Класс адаптивного интегрирования для энергосистемы
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_ADAPTIVE_INTEGRATOR_HPP
#define PS_ADAPTIVE_INTEGRATOR_HPP

#include "../Core/PS_Types.hpp"
#include "../Integration/PS_IntegrationMethods.hpp"
#include "../Utils/PS_Logger.hpp"
#include <memory>
#include <functional>
#include <vector>

namespace PowerSystem {

    /**
     * @brief Интерфейс адаптивного интегратора
     */
    class IAdaptiveIntegrator {
    public:
        /**
         * @brief Тип функции для расчета шага
         */
        using ComputeFunction = std::function<bool(Types::TimeType, Types::TimeType)>;

        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IAdaptiveIntegrator() = default;

        /**
         * @brief Выполнить один шаг интегрирования с адаптивным выбором шага
         * @param computeFunc Функция для расчета на шаге
         * @param currentTime Текущее время
         * @param suggestedStep Предлагаемый шаг
         * @param newTime Новое время после интегрирования (выходной параметр)
         * @param actualStep Фактический использованный шаг (выходной параметр)
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool step(ComputeFunction computeFunc,
                        Types::TimeType currentTime,
                        Types::TimeType suggestedStep,
                        Types::TimeType& newTime,
                        Types::TimeType& actualStep) = 0;

        /**
         * @brief Установить минимальный шаг
         * @param minStep Минимальный шаг
         */
        virtual void setMinStep(Types::TimeType minStep) = 0;

        /**
         * @brief Установить максимальный шаг
         * @param maxStep Максимальный шаг
         */
        virtual void setMaxStep(Types::TimeType maxStep) = 0;

        /**
         * @brief Установить допустимую погрешность
         * @param tolerance Допустимая погрешность
         */
        virtual void setTolerance(double tolerance) = 0;

        /**
         * @brief Установить логгер
         * @param logger Указатель на логгер
         */
        virtual void setLogger(std::shared_ptr<Logger> logger) = 0;
    };

    /**
     * @brief Класс адаптивного интегратора с контролем погрешности
     */
    class AdaptiveIntegrator : public IAdaptiveIntegrator {
    private:
        std::shared_ptr<Logger> m_logger;               ///< Логгер
        Types::TimeType m_minStep;                      ///< Минимальный шаг
        Types::TimeType m_maxStep;                      ///< Максимальный шаг
        double m_tolerance;                             ///< Допустимая погрешность
        double m_safetyFactor;                          ///< Коэффициент безопасности
        int m_maxStepReductions;                        ///< Максимальное число уменьшений шага
        std::unique_ptr<IIntegrationMethod> m_method;   ///< Метод интегрирования
        std::unique_ptr<IIntegrationMethod> m_errorMethod; ///< Метод для оценки погрешности

    public:
        /**
         * @brief Конструктор
         * @param method Метод интегрирования
         * @param errorMethod Метод для оценки погрешности (обычно меньшего порядка)
         */
        AdaptiveIntegrator(std::unique_ptr<IIntegrationMethod> method,
                         std::unique_ptr<IIntegrationMethod> errorMethod);

        /**
         * @brief Деструктор
         */
        ~AdaptiveIntegrator() override = default;

        /**
         * @brief Выполнить один шаг интегрирования с адаптивным выбором шага
         * @param computeFunc Функция для расчета на шаге
         * @param currentTime Текущее время
         * @param suggestedStep Предлагаемый шаг
         * @param newTime Новое время после интегрирования (выходной параметр)
         * @param actualStep Фактический использованный шаг (выходной параметр)
         * @return true в случае успеха, false в случае ошибки
         */
        bool step(ComputeFunction computeFunc,
                Types::TimeType currentTime,
                Types::TimeType suggestedStep,
                Types::TimeType& newTime,
                Types::TimeType& actualStep) override;

        /**
         * @brief Установить минимальный шаг
         * @param minStep Минимальный шаг
         */
        void setMinStep(Types::TimeType minStep) override;

        /**
         * @brief Установить максимальный шаг
         * @param maxStep Максимальный шаг
         */
        void setMaxStep(Types::TimeType maxStep) override;

        /**
         * @brief Установить допустимую погрешность
         * @param tolerance Допустимая погрешность
         */
        void setTolerance(double tolerance) override;

        /**
         * @brief Установить коэффициент безопасности
         * @param safetyFactor Коэффициент безопасности
         */
        void setSafetyFactor(double safetyFactor);

        /**
         * @brief Установить максимальное число уменьшений шага
         * @param maxReductions Максимальное число уменьшений
         */
        void setMaxStepReductions(int maxReductions);

        /**
         * @brief Установить логгер
         * @param logger Указатель на логгер
         */
        void setLogger(std::shared_ptr<Logger> logger) override;
    };

    /**
     * @brief Фабрика для создания адаптивных интеграторов
     */
    class AdaptiveIntegratorFactory {
    public:
        /**
         * @brief Создать адаптивный интегратор с заданными методами
         * @param highOrderMethod Метод высокого порядка
         * @param lowOrderMethod Метод низкого порядка
         * @return Указатель на созданный интегратор
         */
        static std::unique_ptr<IAdaptiveIntegrator> createIntegrator(
            IntegrationMethodType highOrderMethod,
            IntegrationMethodType lowOrderMethod);
    };

} // namespace PowerSystem

#endif // PS_ADAPTIVE_INTEGRATOR_HPP