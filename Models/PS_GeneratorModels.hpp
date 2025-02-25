/**
 * @file PS_GeneratorModels.hpp
 * @brief Модели синхронных генераторов
 * @author Claude
 * @date 25.02.2025
 */

#ifndef PS_GENERATOR_MODELS_HPP
#define PS_GENERATOR_MODELS_HPP

#include "../Core/PS_Types.hpp"
#include <vector>
#include <memory>
#include <string>
#include <map>

namespace PowerSystem {

    // Forward declaration
    enum class GeneratorModelType;

    /**
     * @brief Интерфейс модели генератора
     */
    class IGeneratorModel {
    public:
        /**
         * @brief Виртуальный деструктор
         */
        virtual ~IGeneratorModel() = default;

        /**
         * @brief Инициализация модели
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool initialize() = 0;

        /**
         * @brief Расчет производных переменных состояния
         * @param state Вектор переменных состояния
         * @param inputs Вектор входных переменных
         * @param derivatives Вектор производных (результат)
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool calculateDerivatives(
            const std::vector<double>& state,
            const std::vector<double>& inputs,
            std::vector<double>& derivatives) = 0;

        /**
         * @brief Расчет выходных переменных
         * @param state Вектор переменных состояния
         * @param inputs Вектор входных переменных
         * @param outputs Вектор выходных переменных (результат)
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool calculateOutputs(
            const std::vector<double>& state,
            const std::vector<double>& inputs,
            std::vector<double>& outputs) = 0;

        /**
         * @brief Получить размер вектора состояния
         * @return Размер вектора состояния
         */
        virtual int getStateSize() const = 0;

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
         * @brief Получить начальные значения переменных состояния
         * @return Вектор начальных значений
         */
        virtual std::vector<double> getInitialState() const = 0;

        /**
         * @brief Установить параметры модели
         * @param parameters Карта параметров
         * @return true в случае успеха, false в случае ошибки
         */
        virtual bool setParameters(const std::map<std::string, double>& parameters) = 0;

        /**
         * @brief Получить текущие параметры модели
         * @return Карта параметров
         */
        virtual std::map<std::string, double> getParameters() const = 0;

        /**
         * @brief Получить названия переменных состояния
         * @return Вектор названий
         */
        virtual std::vector<std::string> getStateNames() const = 0;

        /**
         * @brief Получить названия входных переменных
         * @return Вектор названий
         */
        virtual std::vector<std::string> getInputNames() const = 0;

        /**
         * @brief Получить названия выходных переменных
         * @return Вектор названий
         */
        virtual std::vector<std::string> getOutputNames() const = 0;
    };

    /**
     * @brief Базовый класс модели генератора
     */
    class BaseGeneratorModel : public IGeneratorModel {
    protected:
        bool m_initialized;                          ///< Флаг инициализации
        std::map<std::string, double> m_parameters;  ///< Параметры модели
        std::vector<std::string> m_stateNames;       ///< Названия переменных состояния
        std::vector<std::string> m_inputNames;       ///< Названия входных переменных
        std::vector<std::string> m_outputNames;      ///< Названия выходных переменных

    public:
        /**
         * @brief Конструктор
         */
        BaseGeneratorModel();

        /**
         * @brief Деструктор
         */
        virtual ~BaseGeneratorModel() override = default;

        /**
         * @brief Установить параметры модели
         * @param parameters Карта параметров
         * @return true в случае успеха, false в случае ошибки
         */
        bool setParameters(const std::map<std::string, double>& parameters) override;

        /**
         * @brief Получить текущие параметры модели
         * @return Карта параметров
         */
        std::map<std::string, double> getParameters() const override;

        /**
         * @brief Получить названия переменных состояния
         * @return Вектор названий
         */
        std::vector<std::string> getStateNames() const override;

        /**
         * @brief Получить названия входных переменных
         * @return Вектор названий
         */
        std::vector<std::string> getInputNames() const override;

        /**
         * @brief Получить названия выходных переменных
         * @return Вектор названий
         */
        std::vector<std::string> getOutputNames() const override;

    protected:
        /**
         * @brief Проверить наличие необходимых параметров
         * @param requiredParams Вектор названий необходимых параметров
         * @return true если все параметры присутствуют, false иначе
         */
        bool checkRequiredParameters(const std::vector<std::string>& requiredParams) const;
    };

    /**
     * @brief Упрощенная модель синхронного генератора (второго порядка)
     */
    class SimplifiedGeneratorModel : public BaseGeneratorModel {
    public:
        /**
         * @brief Конструктор
         */
        SimplifiedGeneratorModel();

        /**
         * @brief Деструктор
         */
        ~SimplifiedGeneratorModel() override = default;

        /**
         * @brief Инициализация модели
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize() override;

        /**
         * @brief Расчет производных переменных состояния
         * @param state Вектор переменных состояния
         * @param inputs Вектор входных переменных
         * @param derivatives Вектор производных (результат)
         * @return true в случае успеха, false в случае ошибки
         */
        bool calculateDerivatives(
            const std::vector<double>& state,
            const std::vector<double>& inputs,
            std::vector<double>& derivatives) override;

        /**
         * @brief Расчет выходных переменных
         * @param state Вектор переменных состояния
         * @param inputs Вектор входных переменных
         * @param outputs Вектор выходных переменных (результат)
         * @return true в случае успеха, false в случае ошибки
         */
        bool calculateOutputs(
            const std::vector<double>& state,
            const std::vector<double>& inputs,
            std::vector<double>& outputs) override;

        /**
         * @brief Получить размер вектора состояния
         * @return Размер вектора состояния
         */
        int getStateSize() const override;

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
         * @brief Получить начальные значения переменных состояния
         * @return Вектор начальных значений
         */
        std::vector<double> getInitialState() const override;
    };

    /**
     * @brief Полная модель синхронного генератора Парка-Горева (шестого порядка)
     */
    class ParkGorevGeneratorModel : public BaseGeneratorModel {
    public:
        /**
         * @brief Конструктор
         */
        ParkGorevGeneratorModel();

        /**
         * @brief Деструктор
         */
        ~ParkGorevGeneratorModel() override = default;

        /**
         * @brief Инициализация модели
         * @return true в случае успеха, false в случае ошибки
         */
        bool initialize() override;

        /**
         * @brief Расчет производных переменных состояния
         * @param state Вектор переменных состояния
         * @param inputs Вектор входных переменных
         * @param derivatives Вектор производных (результат)
         * @return true в случае успеха, false в случае ошибки
         */
        bool calculateDerivatives(
            const std::vector<double>& state,
            const std::vector<double>& inputs,
            std::vector<double>& derivatives) override;

        /**
         * @brief Расчет выходных переменных
         * @param state Вектор переменных состояния
         * @param inputs Вектор входных переменных
         * @param outputs Вектор выходных переменных (результат)
         * @return true в случае успеха, false в случае ошибки
         */
        bool calculateOutputs(
            const std::vector<double>& state,
            const std::vector<double>& inputs,
            std::vector<double>& outputs) override;

        /**
         * @brief Получить размер вектора состояния
         * @return Размер вектора состояния
         */
        int getStateSize() const override;

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
         * @brief Получить начальные значения переменных состояния
         * @return Вектор начальных значений
         */
        std::vector<double> getInitialState() const override;

    protected:
        /**
         * @brief Расчет токов из потокосцеплений
         * @param state Вектор переменных состояния
         * @param currents Вектор токов (результат)
         * @return true в случае успеха, false в случае ошибки
         */
        bool calculateCurrents(const std::vector<double>& state, std::vector<double>& currents);

        /**
         * @brief Расчет электромагнитного момента
         * @param state Вектор переменных состояния
         * @param currents Вектор токов
         * @return Значение электромагнитного момента
         */
        double calculateElectromagneticTorque(
            const std::vector<double>& state,
            const std::vector<double>& currents);
    };

    /**
     * @brief Фабрика моделей генератора
     */
    class GeneratorModelFactory {
    public:
        /**
         * @brief Создать модель генератора
         * @param type Тип модели
         * @return Указатель на созданную модель
         */
        static std::unique_ptr<IGeneratorModel> createModel(GeneratorModelType type);
    };

} // namespace PowerSystem

#endif // PS_GENERATOR_MODELS_HPP