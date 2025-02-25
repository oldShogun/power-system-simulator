/**
 * @file PS_PowerFlowSolver.hpp
 * @brief Класс для решения задачи потокораспределения
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_POWER_FLOW_SOLVER_HPP
#define PS_POWER_FLOW_SOLVER_HPP

#include "../Core/PS_Types.hpp"
#include "../Core/PS_Interfaces.hpp"
#include "../Elements/PS_Bus.hpp"
#include "../Elements/PS_Branch.hpp"
#include "../Elements/PS_Generator.hpp"
#include "../Elements/PS_Load.hpp"
#include "../Utils/PS_Logger.hpp"
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <Eigen/Dense>

namespace PowerSystem {

    /**
     * @brief Тип метода решения задачи потокораспределения
     */
    enum class PowerFlowMethod {
        GAUSS_SEIDEL,   ///< Метод Гаусса-Зейделя
        NEWTON_RAPHSON, ///< Метод Ньютона-Рафсона
        FAST_DECOUPLED  ///< Метод быстрого разделения
    };

    /**
     * @brief Результаты расчета потокораспределения
     */
    struct PowerFlowResults {
        bool converged;                                    ///< Флаг сходимости
        int iterations;                                    ///< Количество итераций
        double maxMismatch;                                ///< Максимальная невязка
        std::map<Types::ElementId, Types::Complex> voltage; ///< Напряжения в узлах
        std::map<Types::ElementId, Types::Complex> power;   ///< Мощности в узлах
        std::map<Types::ElementId, Types::Complex> current; ///< Токи в ветвях
        std::map<Types::ElementId, Types::Complex> branchPower; ///< Мощности в ветвях
        std::vector<Types::ElementId> pv_to_pq_buses;      ///< Узлы, переключенные с PV на PQ
    };

    /**
     * @brief Интерфейс метода решения задачи потокораспределения
     */
    class IPowerFlowSolver {
    public:
        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IPowerFlowSolver() = default;

        /**
         * @brief Решить задачу потокораспределения
         * @param buses Узлы сети
         * @param branches Ветви сети
         * @param generators Генераторы
         * @param loads Нагрузки
         * @param results Результаты расчета (выходной параметр)
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool solve(
            const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            const std::map<Types::ElementId, std::shared_ptr<Branch>>& branches,
            const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators,
            const std::map<Types::ElementId, std::shared_ptr<Load>>& loads,
            PowerFlowResults& results) = 0;

        /**
         * @brief Получить название метода
         * @return Название метода
         */
        virtual std::string getName() const = 0;

        /**
         * @brief Получить тип метода
         * @return Тип метода
         */
        virtual PowerFlowMethod getMethod() const = 0;

        /**
         * @brief Установить максимальное число итераций
         * @param maxIterations Максимальное число итераций
         */
        virtual void setMaxIterations(int maxIterations) = 0;

        /**
         * @brief Установить точность расчета
         * @param tolerance Точность расчета
         */
        virtual void setTolerance(double tolerance) = 0;

        /**
         * @brief Установить базисную мощность
         * @param baseMVA Базисная мощность в МВА
         */
        virtual void setBaseMVA(double baseMVA) = 0;

        /**
         * @brief Установить логгер
         * @param logger Указатель на логгер
         */
        virtual void setLogger(std::shared_ptr<Logger> logger) = 0;
    };

    /**
     * @brief Базовый класс для методов решения задачи потокораспределения
     */
    class BasePowerFlowSolver : public IPowerFlowSolver {
    protected:
        std::shared_ptr<Logger> m_logger;       ///< Логгер
        int m_maxIterations;                    ///< Максимальное число итераций
        double m_tolerance;                     ///< Точность расчета
        double m_baseMVA;                       ///< Базисная мощность в МВА
        PowerFlowMethod m_method;               ///< Тип метода

        // Рабочие буферы
        std::vector<Types::ElementId> m_busIds;        ///< ID всех узлов
        std::vector<Types::ElementId> m_pvBusIds;      ///< ID PV-узлов
        std::vector<Types::ElementId> m_pqBusIds;      ///< ID PQ-узлов
        std::vector<Types::ElementId> m_slackBusIds;   ///< ID балансирующих узлов
        Types::AdmittanceMatrix m_admittanceMatrix;    ///< Матрица проводимостей

    public:
        /**
         * @brief Конструктор
         * @param method Тип метода
         */
        BasePowerFlowSolver(PowerFlowMethod method);

        /**
         * @brief Деструктор
         */
        virtual ~BasePowerFlowSolver() override = default;

        /**
         * @brief Установить максимальное число итераций
         * @param maxIterations Максимальное число итераций
         */
        void setMaxIterations(int maxIterations) override;

        /**
         * @brief Установить точность расчета
         * @param tolerance Точность расчета
         */
        void setTolerance(double tolerance) override;

        /**
         * @brief Установить базисную мощность
         * @param baseMVA Базисная мощность в МВА
         */
        void setBaseMVA(double baseMVA) override;

        /**
         * @brief Установить логгер
         * @param logger Указатель на логгер
         */
        void setLogger(std::shared_ptr<Logger> logger) override;

        /**
         * @brief Получить тип метода
         * @return Тип метода
         */
        PowerFlowMethod getMethod() const override;

    protected:
        /**
         * @brief Подготовить данные для расчета
         * @param buses Узлы сети
         * @param branches Ветви сети
         * @param generators Генераторы
         * @param loads Нагрузки
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool prepareData(
            const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            const std::map<Types::ElementId, std::shared_ptr<Branch>>& branches,
            const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators,
            const std::map<Types::ElementId, std::shared_ptr<Load>>& loads);

        /**
         * @brief Построить матрицу проводимостей
         * @param buses Узлы сети
         * @param branches Ветви сети
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool buildAdmittanceMatrix(
            const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            const std::map<Types::ElementId, std::shared_ptr<Branch>>& branches);

        /**
         * @brief Распределить узлы по типам (PV, PQ, Slack)
         * @param buses Узлы сети
         * @param generators Генераторы
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool classifyBuses(
            const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators);

        /**
         * @brief Обновить состояние сети после расчета
         * @param buses Узлы сети
         * @param branches Ветви сети
         * @param results Результаты расчета
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool updateNetworkState(
            std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            std::map<Types::ElementId, std::shared_ptr<Branch>>& branches,
            const PowerFlowResults& results);

        /**
         * @brief Проверка и корректировка ограничений по реактивной мощности
         * @param buses Узлы сети
         * @param generators Генераторы
         * @param results Результаты расчета
         * @return true если были изменения, false иначе
         */
        virtual bool checkReactivePowerLimits(
            std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators,
            PowerFlowResults& results);
    };

    /**
     * @brief Класс для решения задачи потокораспределения методом Ньютона-Рафсона
     */
    class NewtonRaphsonPowerFlowSolver : public BasePowerFlowSolver {
    private:
        Eigen::MatrixXd m_jacobian;              ///< Матрица Якоби
        Eigen::VectorXd m_mismatch;              ///< Вектор невязок
        Eigen::VectorXd m_correction;            ///< Вектор поправок
        std::map<Types::ElementId, int> m_busIndices; ///< Соответствие между ID узлов и их индексами

    public:
        /**
         * @brief Конструктор
         */
        NewtonRaphsonPowerFlowSolver();

        /**
         * @brief Деструктор
         */
        ~NewtonRaphsonPowerFlowSolver() override = default;

        /**
         * @brief Решить задачу потокораспределения методом Ньютона-Рафсона
         * @param buses Узлы сети
         * @param branches Ветви сети
         * @param generators Генераторы
         * @param loads Нагрузки
         * @param results Результаты расчета (выходной параметр)
         * @return true в случае успеха, false в случае ошибки
         */
        bool solve(
            const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            const std::map<Types::ElementId, std::shared_ptr<Branch>>& branches,
            const std::map<Types::ElementId, std::shared_ptr<Generator>>& generators,
            const std::map<Types::ElementId, std::shared_ptr<Load>>& loads,
            PowerFlowResults& results) override;

        /**
         * @brief Получить название метода
         * @return Название метода
         */
        std::string getName() const override;

    private:
        /**
         * @brief Построить матрицу Якоби
         * @param buses Узлы сети
         * @param pvBusCount Количество PV-узлов
         * @param pqBusCount Количество PQ-узлов
         * @return true в случае успеха, false в случае ошибки
         */
        bool buildJacobian(const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            int pvBusCount, int pqBusCount);

        /**
         * @brief Вычислить невязки мощности
         * @param buses Узлы сети
         * @param pvBusCount Количество PV-узлов
         * @param pqBusCount Количество PQ-узлов
         * @return true в случае успеха, false в случае ошибки
         */
        bool calculatePowerMismatch(const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            int pvBusCount, int pqBusCount);

        /**
         * @brief Вычислить невязки мощности
         */
        Types::Complex calculatePower(
            const std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            Types::ElementId busId);

        /**
         * @brief Обновить напряжения в узлах
         * @param buses Узлы сети
         * @param pvBusCount Количество PV-узлов
         * @param pqBusCount Количество PQ-узлов
         * @return true в случае успеха, false в случае ошибки
         */
        bool updateVoltages(std::map<Types::ElementId, std::shared_ptr<Bus>>& buses,
            int pvBusCount, int pqBusCount);
    };

    /**
     * @brief Фабрика для создания решателей потокораспределения
     */
    class PowerFlowSolverFactory {
    public:
        /**
         * @brief Создать решатель потокораспределения
         * @param method Тип метода
         * @return Указатель на созданный решатель
         */
        static std::unique_ptr<IPowerFlowSolver> createSolver(PowerFlowMethod method);
    };

} // namespace PowerSystem

#endif // PS_POWER_FLOW_SOLVER_HPP