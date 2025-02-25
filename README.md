/
├── main.cpp                          # Основной файл с тестированием IEEE14
├── Core/                             # Ядро системы моделирования
│   ├── PS_Types.hpp                  # Основные типы данных и константы
│   ├── PS_Interfaces.hpp             # Базовые интерфейсы системы
│   ├── PS_Exception.hpp              # Классы исключений
│   ├── PS_PowerSystemElement.hpp     # Базовый класс элемента энергосистемы
│   ├── PS_PowerSystemElement.cpp     # Реализация базового класса
│   ├── PS_PowerSystemSimulator.hpp   # Основной симулятор (модифицированный)
│   ├── PS_PowerSystemSimulator.cpp   # Реализация симулятора (модифицированная)
│   ├── PS_CorePowerSystem.hpp        # Ядро энергосистемы
│   └── PS_CorePowerSystem.cpp        # Реализация ядра энергосистемы
│
├── Elements/                         # Элементы энергосистемы
│   ├── PS_Bus.hpp                    # Узлы сети
│   ├── PS_Bus.cpp                    # Реализация узлов
│   ├── PS_Branch.hpp                 # Ветви сети
│   ├── PS_Branch.cpp                 # Реализация ветвей
│   ├── PS_Generator.hpp              # Базовый класс генератора
│   ├── PS_Generator.cpp              # Реализация базового генератора
│   ├── PS_Load.hpp                   # Нагрузки
│   ├── PS_Load.cpp                   # Реализация нагрузок
│   ├── PS_SynchronousGenerator.hpp   # Синхронный генератор
│   └── PS_SynchronousGenerator.cpp   # Реализация синхронного генератора
│
├── Models/                           # Модели генераторов
│   ├── PS_GeneratorModels.hpp        # Интерфейс и классы моделей
│   └── PS_GeneratorModels.cpp        # Реализация моделей
│
├── Integration/                      # Методы интегрирования
│   ├── PS_IntegrationMethods.hpp     # Интерфейс и классы методов
│   ├── PS_IntegrationMethods.cpp     # Реализация методов интегрирования
│   ├── PS_AdaptiveIntegrator.hpp     # Класс адаптивного интегрирования (новый)
│   └── PS_AdaptiveIntegrator.cpp     # Реализация адаптивного интегрирования (новый)
│
├── PowerFlow/                        # Расчет потокораспределения (новая директория)
│   ├── PS_PowerFlowSolver.hpp        # Класс решения потокораспределения (новый)
│   └── PS_PowerFlowSolver.cpp        # Реализация решения потокораспределения (новый)
│
├── Import/                           # Импорт данных (новая директория)
│   ├── PS_CdfImporter.hpp            # Класс импорта данных из CDF формата (новый)
│   └── PS_CdfImporter.cpp            # Реализация импорта данных (новый)
│
├── Regulators/                       # Регуляторы
│   ├── PS_Regulators.hpp             # Базовые интерфейсы регуляторов
│   ├── PS_Regulators.cpp             # Реализация базовых классов
│   ├── PS_ExcitationRegulators.hpp   # Регуляторы возбуждения
│   ├── PS_ExcitationRegulators.cpp   # Реализация регуляторов возбуждения
│   ├── PS_TurbineRegulators.hpp      # Регуляторы турбины
│   └── PS_TurbineRegulators.cpp      # Реализация регуляторов турбины
│
├── Analysis/                         # Анализ и обработка результатов
│   ├── PS_DataAnalyzer.hpp           # Анализаторы данных
│   ├── PS_StateEstimator.hpp         # Оценка состояния
│   └── PS_Oscillation.hpp            # Анализ колебаний
│
├── Utils/                            # Вспомогательные компоненты
│   ├── PS_Logger.hpp                 # Система логирования
│   ├── PS_Logger.cpp                 # Реализация логирования
│   ├── PS_Config.hpp                 # Конфигурация системы
│   └── PS_Math.hpp                   # Математические утилиты
│
├── Synchronization/                  # Синхронизация и параллельные вычисления
│   ├── PS_Synchronization.hpp        # Механизмы синхронизации
│   ├── PS_ParallelComputing.hpp      # Параллельные вычисления
│   └── PS_DataExchange.hpp           # Обмен данными между потоками
│
├── data/                             # Директория с тестовыми данными (новая)
│   ├── ieee14.cdf                    # Тестовая схема IEEE-14 в формате CDF (новый)
│   └── ieee30.cdf                    # Тестовая схема IEEE-30 в формате CDF (опционально)
│
└── docs/                             # Документация
    ├── README.md                     # Общее описание системы
    └── generator-requirements.md     # Требования к реализации синхронного генератора
	



PS_Types.hpp                  (не имеет зависимостей внутри проекта)
|
+---> PS_Interfaces.hpp       (зависит от PS_Types.hpp)
|     |
|     +---> PS_PowerSystemElement.hpp (зависит от PS_Types.hpp, PS_Interfaces.hpp, PS_Exception.hpp)
|           |
|           +---> PS_Bus.hpp          (зависит от PS_PowerSystemElement.hpp)
|           |
|           +---> PS_Branch.hpp       (зависит от PS_PowerSystemElement.hpp)
|           |
|           +---> PS_Generator.hpp    (зависит от PS_PowerSystemElement.hpp)
|           |
|           +---> PS_Load.hpp         (зависит от PS_PowerSystemElement.hpp)
|           |
|           +---> PS_SynchronousGenerator.hpp (зависит от PS_PowerSystemElement.hpp, PS_GeneratorModels.hpp, 
|                                             PS_IntegrationMethods.hpp, PS_Regulators.hpp)
|
+---> PS_Exception.hpp        (не имеет зависимостей внутри проекта)
|
+---> PS_IntegrationMethods.hpp (зависит от PS_Exception.hpp)
|     |
|     +---> PS_AdaptiveIntegrator.hpp (зависит от PS_IntegrationMethods.hpp, PS_Types.hpp, PS_Logger.hpp)
|
+---> PS_PowerFlowSolver.hpp  (зависит от PS_Types.hpp, PS_Interfaces.hpp, PS_Bus.hpp, PS_Branch.hpp, 
|                             PS_Generator.hpp, PS_Load.hpp, PS_Logger.hpp)
|
+---> PS_CdfImporter.hpp      (зависит от PS_CorePowerSystem.hpp, PS_Logger.hpp)
|
+---> PS_Regulators.hpp       (зависит от PS_Types.hpp)
|     |
|     +---> PS_ExcitationRegulators.hpp (зависит от PS_Regulators.hpp)
|     |
|     +---> PS_TurbineRegulators.hpp    (зависит от PS_Regulators.hpp)
|
+---> PS_GeneratorModels.hpp  (зависит от PS_Types.hpp)
|
+---> PS_CorePowerSystem.hpp  (зависит от PS_Types.hpp, PS_Interfaces.hpp, PS_PowerSystemElement.hpp, 
|                             PS_Bus.hpp, PS_Branch.hpp, PS_Generator.hpp, PS_Load.hpp, PS_SynchronousGenerator.hpp)
|
+---> PS_PowerSystemSimulator.hpp (зависит от PS_Types.hpp, PS_Exception.hpp, PS_CorePowerSystem.hpp,
                                   PS_PowerFlowSolver.hpp, PS_AdaptiveIntegrator.hpp)