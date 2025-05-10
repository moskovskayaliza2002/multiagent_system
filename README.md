# Мультиагентная система управления антропоморфным роботом

Проект представляет собой конструкторы для создания мультиагентной системы управления антропоморфным роботом на ROS1. Система автоматизирует выполнение сценариев, разрешает конфликты операций и генерирует адаптивные сценарии поведения с использованием GPT.

---

## 🛠 Основные компоненты

### 1. **Базовый агент (`Agent.py`)**
- **Наследование**: Все агенты должны наследоваться от класса `Agent`.
- **Конфликт-менеджмент**:  
  Автоматическое разрешение конфликтов на основе:
  - **Приоритета операций** (чем выше приоритет, тем важнее действие).
  - **Иерархии агентов** (родительские агенты могут переопределять действия дочерних).
- **Поведение по умолчанию**:  
  После завершения операций агент возвращается к заданному действию (например, поза покоя). 
  - **Филлеры**: Случайные действия при долгом бездействии для поддержания интерактивности.

#### Все атрибуты класса:

```mermaid
classDiagram
    class Agent {
        +str name
        +dict agent_operations
        +str current_operation
        +int current_operation_priority
        +Agent parent
        +list[Agent] children
        +float last_operation_time
        +float default_action_delay
        +bool is_default_operation_running
        +str default_operation
        +float default_operation_start
        +bool is_stopped
        +bool force_default_now
        +dict fillers
        +float filler_activation_time
        
        +setStoppedState(state: bool)
        +findChildByName(name: str) Agent
        +updateLastOperationTime()
        +addChild(agent: Agent)
        +addChildren(agent_list: list[Agent])
        #_setParent(parent_agent: Agent)
        +getCurrentPriority() int
        +getParentMaxPriority() int
        +getChildrenMaxPriority() int
        +doOperation(operation_name: str, override_priority: int, from_parent: bool, **kwargs)
        +registerOperation(operation_name: str, function_to_do, priority: int, cancel_function)
        +initOperationDefault(operation_name: str, **operation_kwargs)
        +addFiller(operation_name: str, **operation_kwargs)
        +cancelCurrentOperation()
        +cancelAllChildrenOperations()
        +cancelAllParentOperations()
        +checkForDefaultAction(event)
    }
```

#### Алгоритм разрешения конфликтов:

```mermaid
flowchart TD
    A[New operation received] --> B{Agent stopped?}
    B -- Yes --> C[Ignoring new agent operation]
    B -- No --> D{Operation from an ancestor?}

    D -- Yes --> E{Does the current operation have a higher priority?}
    D -- No --> F{Do ancestors have higher priority?} 

    F -- Yes --> C[Ignoring new agent operation]
    F -- No --> H[Undoing Ancestors' Operations]
    H --> E{Does the current operation have a higher priority?}
    
    E -- Yes --> C[Ignoring new agent operation]
    E -- No --> G{Do descendants have higher priority?}

    G -- Yes --> C[Ignoring new agent operation]
    G -- No --> I[Undoing Descendant Operations]
    I --> J[Starting a new agent operation]

    J --> N[End]
    C --> N[End]
```

### 2. **Диспетчер (`Dispatcher.py`)**
- **Координация**:  
  Управляет воспроизведением текста через синтезатор речи [SOL](!https://github.com/asanmalyshev/speak_out_loud) и парсит метки действий.
- **Метки действий**:  
  Формат `<тип:параметр_1;параметр_n>`. Примеры:
  - Поза: `<pose:left_arm;attention>`
  - Анимация: `<anim:right_arm;slow_splash;2>`
- **Синхронизация**:  
  После завершения сценария переводит агентов в режим по умолчанию.

#### Общий алгоритм обработки цели
```mermaid
flowchart LR
    A[Receive Goal] --> B[Init mentor agents]
    B --> C{Goal is dir?}
    C -- Yes --> D[Read last file]
    C -- No --> E[Speak by SoL]
    D --> E

    E --> F{Is SoL finished with success?}
    F -- No --> G[Server: set preempted]
    F -- Yes --> H{All agents priority < 2?}
    H -- No --> I[Wait for a high priority action to complete]
    H -- Yes --> J[Server: set succeeded]
```
### 3. **Пример агента (`AgentLimb.py`)**
- **Расширенный конфликт-менеджмент**:  
  Вместо отмены операций ожидает завершения анимаций (например, последние 25% времени выполнения).
- **Интеграция с ROS**:  
  Использует ActionLib для управления позами и анимациями конечностей робота.

### 4. **Генератор сценариев (`CreateScenarioServer.py`)**
- **Автоматизация**:  
  Создает тексты для робота-экскурсовода на основе GPT:
  - Адаптация под длительность, стиль (технический/школьный) и наличие юмора.
  - Расстановка меток действий.
- **Файловая структура**:  
```bash
multiagent_system
├── src/
│ ├── Dispatcher.py
│ ├── Agent.py
│ ├── AgentLimb.py
│ └── CreateScenarioServer.py
├── config/
│ ├── full_description/  # Описания экспонатов (название = имя файла)
│ ├── prompt/ # Шаблоны для GPT: сокращение текста и метки
│ └── scenario/ # Результаты генерации (scenario_1.txt, ...), разложенные по папкам (название = имя дирректории)
├── action/
│ ├── CreateScenario.action
│ └── MultiAgentScenarioExecutor.action
└── README.md
```
#### Общий алгоритм обработки цели
```mermaid
flowchart LR
    A[Start creating scenario] --> B[Read Full Description, prompts, params]
    B --> C[Modify Reduce Prompt with params]
    C --> D[Send to GPT for Reduced Description]

    D --> E{Is Reduce Description Valid?}
    E -- Yes --> F[Read Action Labels Prompt]
    F --> G[Send to GPT for Scenario]

    G --> H{Is Scenario Valid?}
    H -- Yes --> I[Save new scenario]
    I --> J[Return True]

    H -- No --> N[Return False]
    E -- No --> O[Return False]
```

## 🌟 Особенности системы
- **Самовосстановление**:  
Автоматически корректирует ошибки, вызванные «галлюцинациями» GPT.
- **ROS-совместимость**:  
Легко адаптируется под любого антропоморфного робота.
- **Гибкость**:  
- Новые операции добавляются через `register_operation()`.
- Иерархия агентов настраивается методом `add_children()`.
- **Практическое применение**:  
Система внедрена в реального робота-экскурсовода и успешно используется.

## 📂 Структура репозитория
- `Agent.py` — базовый класс агента.
- `AgentLimb.py` — пример агента для управления конечностями.
- `Dispatcher.py` — диспетчер для координации команд.
- `CreateScenarioServer.py` — генератор сценариев.
- `action/` — кастомные ROS-action файлы.
- `config/` — конфигурации для генерации сценариев.

## 🚀 Использование
1. **Запуск генератора сценариев**:
```bash
rosrun moc_multiagent_scenario create_scenario_action_server.py
```
2. **Запуск диспетчера**:
```bash
rosrun moc_multiagent_scenario multiagent_dispatcher.py
```
3. **Интеграция с роботом:**:
- Настройте агенты под специфику вашего робота, унаследовав их от Agent
- В Dispatcher: создайте иерархию, определите тип меток действий
- Добавьте правила формирования меток в промпт

## 📜 Лицензия
Проект разработан в рамках выпускной квалификационной работы.
Лицензия: MIT.

**Автор:** Moscovskaya, E.D.
**Репозиторий:** [репо](!https://github.com/moskovskayaliza2002)