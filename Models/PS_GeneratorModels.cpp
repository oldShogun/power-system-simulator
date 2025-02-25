/**
 * @file PS_GeneratorModels.cpp
 * @brief Реализация моделей синхронных генераторов
 * @author Claude
 * @date 25.02.2025
 */

#include "PS_GeneratorModels.hpp"
#include "../Core/PS_Exception.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace PowerSystem {

    // Реализация BaseGeneratorModel
    BaseGeneratorModel::BaseGeneratorModel() : m_initialized(false) {
    }

    bool BaseGeneratorModel::setParameters(const std::map<std::string, double>& parameters) {
        for (const auto& param : parameters) {
            m_parameters[param.first] = param.second;
        }
        return true;
    }

    std::map<std::string, double> BaseGeneratorModel::getParameters() const {
        return m_parameters;
    }

    std::vector<std::string> BaseGeneratorModel::getStateNames() const {
        return m_stateNames;
    }

    std::vector<std::string> BaseGeneratorModel::getInputNames() const {
        return m_inputNames;
    }

    std::vector<std::string> BaseGeneratorModel::getOutputNames() const {
        return m_outputNames;
    }

    bool BaseGeneratorModel::checkRequiredParameters(const std::vector<std::string>& requiredParams) const {
        for (const auto& param : requiredParams) {
            if (m_parameters.find(param) == m_parameters.end()) {
                return false;
            }
        }
        return true;
    }

    // Реализация SimplifiedGeneratorModel
    SimplifiedGeneratorModel::SimplifiedGeneratorModel() : BaseGeneratorModel() {
        // Названия переменных состояния
        m_stateNames = {
            "delta",      // 0 - угол ротора, рад
            "omega"       // 1 - скорость ротора, о.е.
        };

        // Названия входных переменных
        m_inputNames = {
            "vd",         // 0 - напряжение по оси d
            "vq",         // 1 - напряжение по оси q
            "vfd",        // 2 - напряжение возбуждения
            "Pm",         // 3 - механическая мощность
            "omega_sys"   // 4 - угловая скорость системы
        };

        // Названия выходных переменных
        m_outputNames = {
            "id",         // 0 - ток по оси d
            "iq",         // 1 - ток по оси q
            "ifd",        // 2 - ток возбуждения
            "Te",         // 3 - электромагнитный момент
            "Pe",         // 4 - активная мощность
            "Qe"          // 5 - реактивная мощность
        };

        // Значения параметров по умолчанию
        m_parameters["xd"] = 1.8;           // Синхронное реактивное сопротивление по оси d, о.е.
        m_parameters["xq"] = 1.7;           // Синхронное реактивное сопротивление по оси q, о.е.
        m_parameters["xd1"] = 0.3;          // Переходное реактивное сопротивление по оси d, о.е.
        m_parameters["ra"] = 0.0025;        // Активное сопротивление статора, о.е.
        m_parameters["H"] = 6.5;            // Постоянная инерции, с
        m_parameters["D"] = 2.0;            // Коэффициент демпфирования, о.е.
        m_parameters["Td0"] = 8.0;          // Постоянная времени поля

        m_initialized = false;
    }

    bool SimplifiedGeneratorModel::initialize() {
        const std::vector<std::string> requiredParams = {
            "xd", "xq", "xd1", "ra", "H", "D", "Td0"
        };

        if (!checkRequiredParameters(requiredParams)) {
            return false;
        }

        m_initialized = true;
        return true;
    }

    int SimplifiedGeneratorModel::getStateSize() const {
        return 2;  // delta, omega
    }

    int SimplifiedGeneratorModel::getInputSize() const {
        return 5;  // vd, vq, vfd, Pm, omega_sys
    }

    int SimplifiedGeneratorModel::getOutputSize() const {
        return 6;  // id, iq, ifd, Te, Pe, Qe
    }

    std::vector<double> SimplifiedGeneratorModel::getInitialState() const {
        // Начальные значения для установившегося режима
        return {
            0.0,   // delta - угол ротора (0 для начального состояния)
            1.0    // omega - скорость ротора (1.0 о.е.)
        };
    }

    bool SimplifiedGeneratorModel::calculateDerivatives(
        const std::vector<double>& state,
        const std::vector<double>& inputs,
        std::vector<double>& derivatives) {

        if (!m_initialized) {
            return false;
        }

        if (state.size() != getStateSize() || inputs.size() != getInputSize() ||
            derivatives.size() != getStateSize()) {
            return false;
        }

        // Получение переменных состояния
        double delta = state[0];   // Угол ротора
        double omega = state[1];   // Скорость ротора

        // Получение входных переменных
        double vd = inputs[0];       // Напряжение по оси d
        double vq = inputs[1];       // Напряжение по оси q
        double vfd = inputs[2];      // Напряжение возбуждения
        double Pm = inputs[3];       // Механическая мощность
        double omega_sys = inputs[4]; // Угловая скорость системы

        // Получение параметров модели
        double xd = m_parameters["xd"];      // Синхронное сопротивление по оси d
        double xq = m_parameters["xq"];      // Синхронное сопротивление по оси q
        double xd1 = m_parameters["xd1"];    // Переходное сопротивление по оси d
        double ra = m_parameters["ra"];      // Активное сопротивление статора
        double H = m_parameters["H"];        // Постоянная инерции
        double D = m_parameters["D"];        // Коэффициент демпфирования
        double Td0 = m_parameters["Td0"];    // Постоянная времени поля

        // Расчет переходной ЭДС по оси q
        double Eq1 = vfd * (xd / xd1);

        // Расчет токов
        double id = (-vq + Eq1) / xd1;
        double iq = vd / xq;

        // Расчет электромагнитного момента
        double Te = Eq1 * iq;

        // Расчет производной угла ротора (dδ/dt = ω - ω₀)
        derivatives[0] = omega - omega_sys;

        // Расчет производной скорости ротора (dω/dt = (Pm - Pe - D(ω-ω₀))/(2H))
        // где Pe = Te * ω
        derivatives[1] = (Pm - Te * omega - D * (omega - omega_sys)) / (2.0 * H);

        return true;
    }

    bool SimplifiedGeneratorModel::calculateOutputs(
        const std::vector<double>& state,
        const std::vector<double>& inputs,
        std::vector<double>& outputs) {

        if (!m_initialized) {
            return false;
        }

        if (state.size() != getStateSize() || inputs.size() != getInputSize() ||
            outputs.size() != getOutputSize()) {
            return false;
        }

        // Получение переменных состояния
        double delta = state[0];   // Угол ротора
        double omega = state[1];   // Скорость ротора

        // Получение входных переменных
        double vd = inputs[0];       // Напряжение по оси d
        double vq = inputs[1];       // Напряжение по оси q
        double vfd = inputs[2];      // Напряжение возбуждения
        double Pm = inputs[3];       // Механическая мощность
        double omega_sys = inputs[4]; // Угловая скорость системы

        // Получение параметров модели
        double xd = m_parameters["xd"];      // Синхронное сопротивление по оси d
        double xq = m_parameters["xq"];      // Синхронное сопротивление по оси q
        double xd1 = m_parameters["xd1"];    // Переходное сопротивление по оси d
        double ra = m_parameters["ra"];      // Активное сопротивление статора

        // Расчет переходной ЭДС по оси q
        double Eq1 = vfd * (xd / xd1);

        // Расчет токов
        double id = (-vq + Eq1) / xd1;
        double iq = vd / xq;

        // Расчет тока возбуждения
        double ifd = (Eq1 - (xd - xd1) * id) / xd1;

        // Расчет электромагнитного момента
        double Te = Eq1 * iq;

        // Расчет активной и реактивной мощности
        double Pe = Te * omega;
        double Qe = vd * id + vq * iq;

        // Заполнение выходного вектора
        outputs[0] = id;       // id - ток по оси d
        outputs[1] = iq;       // iq - ток по оси q
        outputs[2] = ifd;      // ifd - ток возбуждения
        outputs[3] = Te;       // Te - электромагнитный момент
        outputs[4] = Pe;       // Pe - активная мощность
        outputs[5] = Qe;       // Qe - реактивная мощность

        return true;
    }

    // Реализация модели Парка-Горева (полная модель шестого порядка)
    ParkGorevGeneratorModel::ParkGorevGeneratorModel() : BaseGeneratorModel() {
        // Названия переменных состояния
        m_stateNames = {
            "delta",      // 0 - угол ротора, рад
            "omega",      // 1 - скорость ротора, о.е.
            "psi_d",      // 2 - потокосцепление по оси d
            "psi_q",      // 3 - потокосцепление по оси q
            "psi_fd",     // 4 - потокосцепление обмотки возбуждения
            "psi_kd",     // 5 - потокосцепление демпферной обмотки по оси d
            "psi_kq"      // 6 - потокосцепление демпферной обмотки по оси q
        };

        // Названия входных переменных
        m_inputNames = {
            "vd",         // 0 - напряжение по оси d
            "vq",         // 1 - напряжение по оси q
            "vfd",        // 2 - напряжение возбуждения
            "Pm",         // 3 - механическая мощность
            "omega_sys"   // 4 - угловая скорость системы
        };

        // Названия выходных переменных
        m_outputNames = {
            "id",         // 0 - ток по оси d
            "iq",         // 1 - ток по оси q
            "ifd",        // 2 - ток возбуждения
            "ikd",        // 3 - ток демпферной обмотки по оси d
            "ikq",        // 4 - ток демпферной обмотки по оси q
            "te",         // 5 - электромагнитный момент
            "Pe",         // 6 - активная мощность
            "Qe"          // 7 - реактивная мощность
        };

        // Значения параметров по умолчанию
        m_parameters["ra"] = 0.0025;        // Активное сопротивление статора, о.е.
        m_parameters["xd"] = 1.8;           // Синхронное реактивное сопротивление по оси d, о.е.
        m_parameters["xq"] = 1.7;           // Синхронное реактивное сопротивление по оси q, о.е.
        m_parameters["xd1"] = 0.3;          // Переходное реактивное сопротивление по оси d, о.е.
        m_parameters["xq1"] = 0.55;         // Переходное реактивное сопротивление по оси q, о.е.
        m_parameters["xd2"] = 0.25;         // Сверхпереходное реактивное сопротивление по оси d, о.е.
        m_parameters["xq2"] = 0.25;         // Сверхпереходное реактивное сопротивление по оси q, о.е.

        m_parameters["rfd"] = 0.0015;       // Сопротивление обмотки возбуждения, о.е.
        m_parameters["rkd"] = 0.005;        // Сопротивление демпферной обмотки по оси d, о.е.
        m_parameters["rkq"] = 0.0045;       // Сопротивление демпферной обмотки по оси q, о.е.

        m_parameters["Td0"] = 8.0;          // Постоянная времени для оси d при разомкнутом статоре, с
        m_parameters["Tq0"] = 1.0;          // Постоянная времени для оси q при разомкнутом статоре, с
        m_parameters["Td1"] = 0.8;          // Переходная постоянная времени по оси d, с
        m_parameters["Tq1"] = 0.4;          // Переходная постоянная времени по оси q, с
        m_parameters["Td2"] = 0.035;        // Сверхпереходная постоянная времени по оси d, с
        m_parameters["Tq2"] = 0.07;         // Сверхпереходная постоянная времени по оси q, с

        m_parameters["H"] = 6.5;            // Постоянная инерции, с
        m_parameters["D"] = 2.0;            // Коэффициент демпфирования, о.е.

        m_parameters["xad"] = 1.6;          // Взаимная индуктивность по оси d, о.е.
        m_parameters["xaq"] = 1.6;          // Взаимная индуктивность по оси q, о.е.

        m_parameters["xl"] = 0.2;           // Индуктивное сопротивление утечки, о.е.

        m_parameters["xfd"] = 1.9;          // Собственное сопротивление обмотки возбуждения, о.е.
        m_parameters["xkd"] = 1.9;          // Собственное сопротивление демпферной обмотки по оси d, о.е.
        m_parameters["xkq"] = 1.9;          // Собственное сопротивление демпферной обмотки по оси q, о.е.

        m_initialized = false;
    }

    bool ParkGorevGeneratorModel::initialize() {
        const std::vector<std::string> requiredParams = {
            "ra", "xd", "xq", "xd1", "xq1", "xd2", "xq2",
            "rfd", "rkd", "rkq", "Td0", "Tq0", "H", "D"
        };

        if (!checkRequiredParameters(requiredParams)) {
            return false;
        }

        // Преобразование и проверка параметров для расчета
        double xd = m_parameters["xd"];
        double xq = m_parameters["xq"];
        double xd1 = m_parameters["xd1"];
        double xq1 = m_parameters["xq1"];
        double xd2 = m_parameters["xd2"];
        double xq2 = m_parameters["xq2"];
        double Td0 = m_parameters["Td0"];
        double Tq0 = m_parameters["Tq0"];

        // Расчет дополнительных параметров
        double xl = xd2;                              // Индуктивное сопротивление утечки
        double xad = xd - xl;                         // Взаимное сопротивление по оси d
        double xaq = xq - xl;                         // Взаимное сопротивление по оси q

        // Расчет параметров обмотки возбуждения
        double xfd = (xd - xd1) * xad / (xd1 - xl);  // Сопротивление рассеяния обмотки возбуждения

        // Расчет параметров демпферных обмоток
        double xkd = (xd1 - xd2) * xad * xfd / (xd1 * (xfd + xad) - xad * xfd - xd2 * xfd);
        double xkq = (xq - xq2) * xaq / (xq2 - xl);

        // Обновление расчетных параметров
        m_parameters["xl"] = xl;
        m_parameters["xad"] = xad;
        m_parameters["xaq"] = xaq;
        m_parameters["xfd"] = xfd;
        m_parameters["xkd"] = xkd;
        m_parameters["xkq"] = xkq;

        m_initialized = true;
        return true;
    }

    int ParkGorevGeneratorModel::getStateSize() const {
        return 7;  // delta, omega, psi_d, psi_q, psi_fd, psi_kd, psi_kq
    }

    int ParkGorevGeneratorModel::getInputSize() const {
        return 5;  // vd, vq, vfd, Pm, omega_sys
    }

    int ParkGorevGeneratorModel::getOutputSize() const {
        return 8;  // id, iq, ifd, ikd, ikq, te, Pe, Qe
    }

    std::vector<double> ParkGorevGeneratorModel::getInitialState() const {
        // Начальные значения для установившегося режима
        // Предполагаем, что генератор работает с номинальными параметрами
        return {
            0.0,   // delta - угол ротора (0 для начального состояния)
            1.0,   // omega - скорость ротора (1.0 о.е.)
            0.0,   // psi_d - потокосцепление по оси d (расчетное)
            0.0,   // psi_q - потокосцепление по оси q (расчетное)
            0.0,   // psi_fd - потокосцепление обмотки возбуждения (расчетное)
            0.0,   // psi_kd - потокосцепление демпферной обмотки по оси d (расчетное)
            0.0    // psi_kq - потокосцепление демпферной обмотки по оси q (расчетное)
        };
    }

    bool ParkGorevGeneratorModel::calculateDerivatives(
        const std::vector<double>& state,
        const std::vector<double>& inputs,
        std::vector<double>& derivatives) {

        if (!m_initialized) {
            return false;
        }

        // Проверка и изменение размера вектора derivatives при необходимости
        if (derivatives.size() != getStateSize()) {
            derivatives.resize(getStateSize(), 0.0);
        }

        if (state.size() != getStateSize() || inputs.size() != getInputSize()) {
            return false;
        }

        // Получение переменных состояния
        double delta = state[0];   // Угол ротора
        double omega = state[1];   // Скорость ротора
        double psi_d = state[2];   // Потокосцепление по оси d
        double psi_q = state[3];   // Потокосцепление по оси q
        double psi_fd = state[4];  // Потокосцепление обмотки возбуждения
        double psi_kd = state[5];  // Потокосцепление демпферной обмотки по оси d
        double psi_kq = state[6];  // Потокосцепление демпферной обмотки по оси q

        // Получение входных переменных
        double vd = inputs[0];       // Напряжение по оси d
        double vq = inputs[1];       // Напряжение по оси q
        double vfd = inputs[2];      // Напряжение возбуждения
        double Pm = inputs[3];       // Механическая мощность
        double omega_sys = inputs[4]; // Угловая скорость системы

        // Получение параметров модели
        double ra = m_parameters["ra"];      // Активное сопротивление статора
        double rfd = m_parameters["rfd"];    // Сопротивление обмотки возбуждения
        double rkd = m_parameters["rkd"];    // Сопротивление демпферной обмотки по оси d
        double rkq = m_parameters["rkq"];    // Сопротивление демпферной обмотки по оси q
        double H = m_parameters["H"];        // Постоянная инерции
        double D = m_parameters["D"];        // Коэффициент демпфирования
        double omega0 = 2.0 * M_PI * 50.0;   // Базовая угловая скорость (рад/с)

        // Расчет токов из потокосцеплений
        std::vector<double> currents(5);
        if (!calculateCurrents(state, currents)) {
            return false;
        }

        double id = currents[0];    // Ток по оси d
        double iq = currents[1];    // Ток по оси q
        double ifd = currents[2];   // Ток обмотки возбуждения
        double ikd = currents[3];   // Ток демпферной обмотки по оси d
        double ikq = currents[4];   // Ток демпферной обмотки по оси q

        // Расчет электромагнитного момента
        double Te = calculateElectromagneticTorque(state, currents);

        // Расчет производных переменных состояния

        // 1. Производная угла ротора: dδ/dt = ω - ω₀
        derivatives[0] = omega - omega_sys;

        // 2. Производная скорости ротора: dω/dt = (Pm - Pe - D(ω-ω₀))/(2H)
        derivatives[1] = (Pm - Te - D * (omega - omega_sys)) / (2.0 * H);

        // 3. Производная потокосцепления по оси d: dψd/dt = ω₀(vd + raId - ψq(ω/ω₀))
        derivatives[2] = omega0 * (vd + ra * id - psi_q * (omega / omega_sys));

        // 4. Производная потокосцепления по оси q: dψq/dt = ω₀(vq + raIq + ψd(ω/ω₀))
        derivatives[3] = omega0 * (vq + ra * iq + psi_d * (omega / omega_sys));

        // 5. Производная потокосцепления обмотки возбуждения: dψfd/dt = ω₀(vfd - rfdIfd)
        derivatives[4] = omega0 * (vfd - rfd * ifd);

        // 6. Производная потокосцепления демпферной обмотки по оси d: dψkd/dt = -ω₀rkdIkd
        derivatives[5] = -omega0 * rkd * ikd;

        // 7. Производная потокосцепления демпферной обмотки по оси q: dψkq/dt = -ω₀rkqIkq
        derivatives[6] = -omega0 * rkq * ikq;

        return true;
    }

    bool ParkGorevGeneratorModel::calculateOutputs(
        const std::vector<double>& state,
        const std::vector<double>& inputs,
        std::vector<double>& outputs) {

        if (!m_initialized) {
            return false;
        }

        if (state.size() != getStateSize() || inputs.size() != getInputSize() ||
            outputs.size() != getOutputSize()) {
            return false;
        }

        // Расчет токов
        std::vector<double> currents(5);
        if (!calculateCurrents(state, currents)) {
            return false;
        }

        // Получение переменных состояния и входов
        double omega = state[1];                // Скорость ротора
        double vd = inputs[0];                  // Напряжение по оси d
        double vq = inputs[1];                  // Напряжение по оси q
        double omega_sys = inputs[4];           // Угловая скорость системы

        // Расчет электромагнитного момента
        double Te = calculateElectromagneticTorque(state, currents);

        // Расчет активной и реактивной мощности
        double Pe = Te * omega;                 // Активная мощность
        double Qe = vd * currents[1] - vq * currents[0];  // Реактивная мощность

        // Заполнение выходного вектора
        outputs[0] = currents[0];       // id - ток по оси d
        outputs[1] = currents[1];       // iq - ток по оси q
        outputs[2] = currents[2];       // ifd - ток обмотки возбуждения
        outputs[3] = currents[3];       // ikd - ток демпферной обмотки по оси d
        outputs[4] = currents[4];       // ikq - ток демпферной обмотки по оси q
        outputs[5] = Te;                // Te - электромагнитный момент
        outputs[6] = Pe;                // Pe - активная мощность
        outputs[7] = Qe;                // Qe - реактивная мощность

        return true;
    }

    bool ParkGorevGeneratorModel::calculateCurrents(
        const std::vector<double>& state,
        std::vector<double>& currents) {

        if (currents.size() < 5) {
            currents.resize(5);
        }

        // Получение потокосцеплений
        double psi_d = state[2];
        double psi_q = state[3];
        double psi_fd = state[4];
        double psi_kd = state[5];
        double psi_kq = state[6];

        // Получение параметров модели
        double xd = m_parameters["xd"];
        double xq = m_parameters["xq"];
        double xl = m_parameters["xl"];
        double xad = m_parameters["xad"];
        double xaq = m_parameters["xaq"];
        double xfd = m_parameters["xfd"];
        double xkd = m_parameters["xkd"];
        double xkq = m_parameters["xkq"];

        // Расчет токов по осям d и q

        // Вспомогательные переменные
        double Dd = xd * (xfd + xad) * (xkd + xad) - xad * xad * (xfd + xkd + xad);
        double Dq = xq * (xkq + xaq) - xaq * xaq;

        // Расчет тока статора по оси d
        currents[0] = ((xfd + xad) * (xkd + xad) * psi_d -
            xad * (xkd + xad) * psi_fd -
            xad * (xfd + xad) * psi_kd) / Dd;

        // Расчет тока статора по оси q
        currents[1] = ((xkq + xaq) * psi_q - xaq * psi_kq) / Dq;

        // Расчет тока обмотки возбуждения
        currents[2] = (-(xkd + xad) * xad * psi_d +
            (xkd + xad) * psi_fd +
            xad * psi_kd) / Dd;

        // Расчет тока демпферной обмотки по оси d
        currents[3] = (-(xfd + xad) * xad * psi_d +
            xad * psi_fd +
            (xfd + xad) * psi_kd) / Dd;

        // Расчет тока демпферной обмотки по оси q
        currents[4] = (-xaq * psi_q + psi_kq) / Dq;

        return true;
    }

    double ParkGorevGeneratorModel::calculateElectromagneticTorque(
        const std::vector<double>& state,
        const std::vector<double>& currents) {

        // Получение потокосцеплений и токов
        double psi_d = state[2];
        double psi_q = state[3];
        double id = currents[0];
        double iq = currents[1];

        // Расчет электромагнитного момента: Te = psi_d*iq - psi_q*id
        return psi_d * iq - psi_q * id;
    }

    // Реализация GeneratorModelFactory
    std::unique_ptr<IGeneratorModel> GeneratorModelFactory::createModel(GeneratorModelType type) {
        switch (type) {
        case GeneratorModelType::SIMPLIFIED:
            return std::make_unique<SimplifiedGeneratorModel>();
        case GeneratorModelType::PARK_GOREV:
            return std::make_unique<ParkGorevGeneratorModel>();
        case GeneratorModelType::CUSTOM:
            // В этом случае предполагается, что пользователь сам предоставит модель
            throw PowerSystemException("Custom generator model not implemented");
        default:
            throw PowerSystemException("Unknown generator model type");
        }
    }

} // namespace PowerSystem