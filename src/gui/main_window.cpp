// src/gui/main_window.cpp
#include "main_window.hpp"
#include "logic/PathfindingLogic.hpp" 

// --- Qt Headers ---
#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit> // Added
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QSizePolicy>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTextEdit> // Added
#include <QStyle>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QDebug>
#include <QCheckBox> // Added
#include <QtConcurrent/QtConcurrent> // Added for async run
#include <QFutureWatcher> // Added for async run

// --- Standard Headers ---
#include <thread>
#include <vector>
#include <memory>
#include <algorithm>
#include <exception>
#include <string>
#include <sstream> // For parsing obstacle costs
#include <iomanip> // For formatting output

// --- Backend Includes (Ensure paths are correct) ---
#include "IO/PathSaver.hpp" // For saving the path
#include "map/ParallelProcessorFlags.hpp" // If needed (for omp_set_num_threads)
#include "map/ElevationFetcherPy.hpp"
#ifdef _OPENMP
#include <omp.h> // Required for omp_set_num_threads
#endif

// --- Placeholder for the actual backend processing function ---
// This function should encapsulate the logic from your backend main.cpp
// It takes BackendInputParams and returns BackendResult.
// Ideally, put this in a separate .cpp file linked to the project.
BackendResult runBackendProcessing(BackendInputParams params) {
    // --- !!! IMPLEMENT THIS FUNCTION BASED ON YOUR BACKEND main.cpp !!! ---
    BackendResult result;
    result.usedMapFilePath = params.mapFilePath; // Echo inputs for GUI state
    result.usedGridWidth = params.desiredGridWidth;
    result.usedGridHeight = params.desiredGridHeight;

    // 1. Set Thread Count (if OpenMP enabled)
#ifdef _OPENMP
    omp_set_num_threads(params.numThreads);
    qDebug() << "Backend: Set OpenMP threads to" << params.numThreads;
#endif

    // 2. Instantiate Processor / Call Scan / Process Map (or use existing)
    //    - Check params.reuseGridIfPossible and params.existingGrid
    //    - Handle map loading, scanning, grid generation
    //    - Populate result.processedGrid, result.normalizationInfo, result.mapProcessingDurationMs
    //    - Handle errors and set result.success = false, result.errorMessage

    // 3. Extract Waypoints
    //    - Populate result.waypointsFound
    //    - Handle errors

    // 4. Fetch/Prepare Elevation
    //    - Call fetchElevationDataEmbedded, convertLatLonToProjectedViaPython etc.
    //    - Handle fallback to dummy data
    //    - Populate result.elevationDataUsed, result.usedDummyElevation,
    //      result.finalLogicalResolutionMeters, result.finalOriginOffsetX/Y,
    //      result.elevationFetchDurationMs
    //    - Handle errors

    // 5. Run Pathfinding Loop
    //    - Select algorithm based on params.algorithmName
    //    - Loop through waypoints
    //    - Call the appropriate findPath function (A*, Dijkstra, GPU etc.)
    //      using params.heuristicType, params.gpuDelta etc. where needed.
    //    - Concatenate paths
    //    - Populate result.fullPathIndices, result.pathfindingDurationMs
    //    - Handle errors (e.g., path not found between segments)

    // 6. Finalize Result
    //    - Ensure result.success reflects overall success/failure
    //    - Ensure result.errorMessage is set on any failure

    // Example placeholder:
    qDebug() << "Backend: Starting processing for" << QString::fromStdString(params.mapFilePath);
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate work
    if (params.mapFilePath.empty() || params.controlsFilePath.empty()) {
        result.success = false;
        result.errorMessage = "Map or Controls file path is empty.";
        qDebug() << "Backend: Error - Empty file path.";
    }
    else {
        result.success = true; // Assume success for now
        result.fullPathIndices = { 0, 1, 2, 3, 4 }; // Dummy path
        result.processedGrid = mapgeo::Grid_V3(10, 10); // Dummy grid
        result.normalizationInfo = mapgeo::NormalizationResult{ true, 0, 0, 100, 100 }; // Dummy norm
        result.mapProcessingDurationMs = 550.5;
        result.pathfindingDurationMs = 123.4;
        qDebug() << "Backend: Processing finished successfully (simulated).";
    }

    return result;
}


namespace app {

    // =========================================================================
    // Stylesheets (light & dark versions) - DEFINED HERE
    // =========================================================================
    const QString lightStylesheet = R"(
        /* --- Base --- */
        QMainWindow, QDialog { background-color: #f4f4f4; }
        QWidget { color: #333333; font-size: 10pt; }

        /* --- Buttons --- */
        QPushButton {
            background-color: #0078d7; color: white;
            border: 1px solid #005a9e; padding: 8px 16px; border-radius: 4px;
            min-height: 20px;
        }
        QPushButton:hover { background-color: #005a9e; }
        QPushButton:pressed { background-color: #003c6a; }
        QPushButton:disabled { background-color: #cccccc; color: #666666; border-color: #aaaaaa; }

        /* --- Toolbar Buttons --- */
        QToolBar { background-color: #e8e8e8; border-bottom: 1px solid #cccccc; padding: 2px; }
        QToolButton {
            background-color: transparent; color: #333333; border: 1px solid transparent;
            padding: 5px 8px; margin: 1px; border-radius: 4px; min-height: 20px;
        }
        QToolButton:hover { background-color: #cce5ff; border: 1px solid #a8d1ff; color: #003c6a; }
        QToolButton:pressed { background-color: #a8d1ff; border: 1px solid #84b8ff; }
        QToolButton:checked { background-color: #e0e0e0; border: 1px solid #b0b0b0; }
        QToolButton:disabled { background-color: transparent; color: #aaaaaa; border-color: transparent; }

        /* --- Inputs --- */
        QSpinBox, QComboBox, QLineEdit, QTextEdit, QDoubleSpinBox {
            padding: 4px; border: 1px solid #c5c5c5; border-radius: 3px; min-height: 20px;
            background-color: #ffffff; color: #333333;
        }
        QSpinBox::up-button, QSpinBox::down-button,
        QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { width: 16px; }
        QComboBox::drop-down { border: none; }
        QComboBox::down-arrow { image: url(:/qt-project.org/styles/commonstyle/images/downarraow-16.png); } /* Might need resource file */

        /* --- Containers & Misc --- */
        QDockWidget { background-color: #fafafa; border: none; }
        QDockWidget::title { background-color: #e1e1e1; text-align: left; padding: 5px; border: 1px solid #c5c5c5; }
        QWidget#SettingsDockContent { background-color: #fafafa; }
        QStatusBar { background-color: #e1e1e1; color: #333; font-size: 9pt; }
        QMenuBar { background-color: #e8e8e8; }
        QMenu { background-color: #ffffff; border: 1px solid #cccccc; }
        QMenu::item:selected { background-color: #0078d7; color: white; }
        QGroupBox { border: 1px solid #c5c5c5; border-radius: 4px; margin-top: 10px; font-weight: bold; }
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; left: 10px; background-color: #f4f4f4; }
        QTabWidget::pane { border: 1px solid #c5c5c5; border-top: none; }
        QTabBar::tab { padding: 8px 15px; border: 1px solid #c5c5c5; border-bottom: none; border-top-left-radius: 4px; border-top-right-radius: 4px; background-color: #e1e1e1; margin-right: 2px;}
        QTabBar::tab:selected { background-color: #fafafa; }
        QTabBar::tab:!selected:hover { background-color: #eaeaea; }
        QPushButton#GeneratorSettingsButton, QPushButton#SolverSettingsButton { /* Dock tab buttons */
            border-radius: 0; border:none; border-bottom: 2px solid transparent; background: #e8e8e8; padding: 6px 10px; color: #333;
        }
        QPushButton#GeneratorSettingsButton:hover, QPushButton#SolverSettingsButton:hover { background-color: #f0f0f0; }
        QPushButton#GeneratorSettingsButton:checked, QPushButton#SolverSettingsButton:checked { border-bottom-color: #0078d7; background: #fafafa; font-weight: bold; color: #003c6a; }
    )";

    const QString darkStylesheet = R"(
        /* --- Base --- */
        QMainWindow, QDialog { background-color: #2d2d2d; }
        QWidget { color: #e0e0e0; font-size: 10pt; }

        /* --- Buttons --- */
        QPushButton {
            background-color: #0078d7; color: white;
            border: 1px solid #005a9e; padding: 8px 16px; border-radius: 4px;
            min-height: 20px;
        }
        QPushButton:hover { background-color: #108de0; }
        QPushButton:pressed { background-color: #005a9e; }
        QPushButton:disabled { background-color: #454545; color: #888888; border-color: #555555; }

        /* --- Toolbar Buttons --- */
         QToolBar { background-color: #3a3a3a; border-bottom: 1px solid #4a4a4a; padding: 2px; }
         QToolButton {
            background-color: transparent; color: #e0e0e0; border: 1px solid transparent;
            padding: 5px 8px; margin: 1px; border-radius: 4px; min-height: 20px;
        }
        QToolButton:hover { background-color: #555555; border: 1px solid #6a6a6a; color: #ffffff; }
        QToolButton:pressed { background-color: #606060; border: 1px solid #777777; }
        QToolButton:checked { background-color: #484848; border: 1px solid #5a5a5a; }
        QToolButton:disabled { background-color: transparent; color: #777777; border-color: transparent; }

        /* --- Inputs --- */
        QSpinBox, QComboBox, QLineEdit, QTextEdit, QDoubleSpinBox {
            padding: 4px; border: 1px solid #555555; border-radius: 3px; min-height: 20px;
            background-color: #3c3c3c; color: #e0e0e0;
        }
        QSpinBox::up-button, QSpinBox::down-button,
        QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { background-color: #505050; border-left: 1px solid #555555;}
        QComboBox QAbstractItemView { background-color: #3c3c3c; border: 1px solid #555555; selection-background-color: #0078d7; }
        QComboBox::drop-down { border: none; }
        QComboBox::down-arrow { image: url(:/qt-project.org/styles/commonstyle/images/downarraow-16-dark.png); } /* Might need specific dark arrow */

        /* --- Containers & Misc --- */
        QDockWidget { background-color: #353535; border: none; }
        QDockWidget::title { background-color: #404040; text-align: left; padding: 5px; border: 1px solid #4a4a4a; color: #e0e0e0;}
        QWidget#SettingsDockContent { background-color: #353535; }
        QStatusBar { background-color: #404040; color: #cccccc; font-size: 9pt; }
        QMenuBar { background-color: #3a3a3a; color: #e0e0e0; }
        QMenu { background-color: #3c3c3c; border: 1px solid #555555; color: #e0e0e0;}
        QMenu::item:selected { background-color: #0078d7; color: white; }
        QGroupBox { border: 1px solid #555555; border-radius: 4px; margin-top: 10px; font-weight: bold; color: #e0e0e0;}
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; left: 10px; background-color: #2d2d2d; color: #e0e0e0;}
        QTabWidget::pane { border: 1px solid #555555; border-top: none; }
        QTabBar::tab { padding: 8px 15px; border: 1px solid #555555; border-bottom: none; border-top-left-radius: 4px; border-top-right-radius: 4px; background-color: #404040; margin-right: 2px; color: #bbbbbb;}
        QTabBar::tab:selected { background-color: #353535; color: #e0e0e0; }
        QTabBar::tab:!selected:hover { background-color: #484848; }
        QPushButton#GeneratorSettingsButton, QPushButton#SolverSettingsButton { /* Dock tab buttons */
             border-radius: 0; border:none; border-bottom: 2px solid transparent;
             background: #3a3a3a; padding: 6px 10px; color: #e0e0e0; /* Brighter base text */
        }
         QPushButton#GeneratorSettingsButton:hover, QPushButton#SolverSettingsButton:hover {
             background-color: #484848; color: #ffffff; /* White text on hover */
        }
        QPushButton#GeneratorSettingsButton:checked, QPushButton#SolverSettingsButton:checked {
             border-bottom-color: #0078d7; background: #353535; font-weight: bold;
             color: #ffffff; /* White selected text */
        }
    )";

    // =========================================================================
    // Implementation Struct
    // =========================================================================
    struct MainWindow::Impl {
        // UI Elements
        QWidget* centralWidget{ nullptr };
        QPushButton* calculatePathButton{ nullptr }; // Renamed from testerButton
        QPushButton* exportButton{ nullptr };
        QPushButton* browseMapButton{ nullptr };
        QPushButton* browseControlsButton{ nullptr };
        QLineEdit* mapLineEdit{ nullptr };
        QLineEdit* controlsLineEdit{ nullptr };
        QStatusBar* statusBar{ nullptr };
        QDockWidget* settingsDockWidget{ nullptr };
        QStackedWidget* settingsStack{ nullptr };
        QCheckBox* autoExportCheckBox{ nullptr };
        PathfindingLogic pathfindingLogic;

        // Actions
        QAction* mapSettingsAction{ nullptr };
        QAction* algorithmSettingsAction{ nullptr };
        QAction* darkModeAction{ nullptr };
        QAction* aboutAction{ nullptr };
        QAction* usageAction{ nullptr };

        // Settings Widgets - Map & Processing Panel
        QSpinBox* gridWidthSpinBox{ nullptr };
        QSpinBox* gridHeightSpinBox{ nullptr };
        QSpinBox* numThreadsSpinBox{ nullptr };
        QTextEdit* obstacleCostsTextEdit{ nullptr };
        QDoubleSpinBox* desiredElevResSpinBox{ nullptr };
        QGroupBox* gpuParamsGroup{ nullptr }; // Contains GPU settings
        QDoubleSpinBox* gpuDeltaSpinBox{ nullptr };
        QDoubleSpinBox* gpuThresholdSpinBox{ nullptr };
        QSpinBox* hadsRadiusSpinBox{ nullptr };
        QDoubleSpinBox* hadsPruneSpinBox{ nullptr };
        QDoubleSpinBox* hadsWeightSpinBox{ nullptr };

        // Settings Widgets - Solver Panel
        QComboBox* algorithmComboBox{ nullptr };
        QComboBox* heuristicComboBox{ nullptr };

        // State & Data
        QSettings* settings{ nullptr };
        QFutureWatcher<BackendResult> calculationWatcher; // For async task
        bool isCalculating = false;

        // --- Stored Backend Data ---
        std::string currentMapFilePath;     // Path used for last successful grid generation
        std::string currentControlsFilePath;// Path used for last calculation trigger
        int lastGridWidth = 0;             // Grid dimensions used for last successful grid gen
        int lastGridHeight = 0;
        std::optional<mapgeo::Grid_V3> lastProcessedGrid; // Store the generated grid
        std::optional<mapgeo::NormalizationResult> lastNormalizationInfo;
        std::optional<ElevationFetcher::ElevationData> lastElevationDataUsed;
        float lastLogicalResolutionMeters = 1.0f;
        float lastOriginOffsetX = 0.0f;
        float lastOriginOffsetY = 0.0f;
        std::vector<int> lastCalculatedPathIndices; // Store the resulting path

        // --- Stored Settings for Reuse Check ---
        mapgeo::ObstacleConfigMap lastUsedObstacleCosts; // Store for potential future reuse logic

    };


    // =========================================================================
    // Constructor & Destructor
    // =========================================================================
    MainWindow::MainWindow(QWidget* parent)
        : QMainWindow(parent)
        , m_impl(std::make_unique<Impl>())
    {
        setWindowTitle("OMAP Pathfinding Processor");
        resize(900, 700);
        m_impl->settings = new QSettings("YourCompany", "OmapPathfinder", this);

        m_impl->centralWidget = new QWidget(this);
        auto* centralLayout = new QVBoxLayout(m_impl->centralWidget);

        // --- Input File Selection ---
        QGroupBox* fileGroup = new QGroupBox("Input Files");
        auto* fileLayout = new QFormLayout(fileGroup);
        auto* mapLayout = new QHBoxLayout();
        m_impl->mapLineEdit = new QLineEdit();
        m_impl->browseMapButton = new QPushButton("Browse...");
        mapLayout->addWidget(m_impl->mapLineEdit); mapLayout->addWidget(m_impl->browseMapButton);
        fileLayout->addRow("Map File (.omap):", mapLayout);

        auto* controlsLayout = new QHBoxLayout();
        m_impl->controlsLineEdit = new QLineEdit();
        m_impl->browseControlsButton = new QPushButton("Browse...");
        controlsLayout->addWidget(m_impl->controlsLineEdit); controlsLayout->addWidget(m_impl->browseControlsButton);
        fileLayout->addRow("Controls File (.omap):", controlsLayout);
        centralLayout->addWidget(fileGroup);

        // --- Main Action Buttons ---
        auto* mainButtonsLayout = new QHBoxLayout();
        // REMOVED: importButton
        m_impl->calculatePathButton = new QPushButton(this->style()->standardIcon(QStyle::SP_MediaPlay), " Calculate Path", m_impl->centralWidget);
        m_impl->exportButton = new QPushButton(this->style()->standardIcon(QStyle::SP_DialogSaveButton), " Export Path", m_impl->centralWidget);
        m_impl->autoExportCheckBox = new QCheckBox("Auto Export");
        m_impl->calculatePathButton->setToolTip("Process map, controls, and calculate path using current settings.");
        m_impl->exportButton->setToolTip("Export the last successfully calculated path to an OMAP file.");
        m_impl->autoExportCheckBox->setToolTip("If checked, automatically save the path after successful calculation without prompting.");

        mainButtonsLayout->addWidget(m_impl->calculatePathButton);
        mainButtonsLayout->addWidget(m_impl->exportButton);
        mainButtonsLayout->addWidget(m_impl->autoExportCheckBox);
        mainButtonsLayout->addStretch();
        centralLayout->addLayout(mainButtonsLayout);

        // --- Map Placeholder ---
        QLabel* mapPlaceholder = new QLabel("Map Area (Visualization Placeholder)");
        mapPlaceholder->setAlignment(Qt::AlignCenter);
        mapPlaceholder->setFrameStyle(QFrame::Panel | QFrame::Sunken);
        mapPlaceholder->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        mapPlaceholder->setMinimumSize(400, 300);
        centralLayout->addWidget(mapPlaceholder);

        setCentralWidget(m_impl->centralWidget);

        // --- Status Bar ---
        m_impl->statusBar = new QStatusBar(this);
        setStatusBar(m_impl->statusBar);

        createActions();
        createToolBars();
        createMenus();
        createSettingsDockWidget(); // Creates both panels now

        // --- Connect Signals & Slots ---
        connect(m_impl->browseMapButton, &QPushButton::clicked, this, &MainWindow::onBrowseMapFile);
        connect(m_impl->browseControlsButton, &QPushButton::clicked, this, &MainWindow::onBrowseControlsFile);
        connect(m_impl->calculatePathButton, &QPushButton::clicked, this, &MainWindow::onCalculatePathClicked);
        connect(m_impl->exportButton, &QPushButton::clicked, this, &MainWindow::triggerExport);
        connect(m_impl->autoExportCheckBox, &QCheckBox::toggled, this, &MainWindow::onAutoExportToggled); // Optional: Save state
        connect(m_impl->mapSettingsAction, &QAction::triggered, this, &MainWindow::onMapSettingsClicked);
        connect(m_impl->algorithmSettingsAction, &QAction::triggered, this, &MainWindow::onAlgorithmSettingsClicked);
        connect(m_impl->darkModeAction, &QAction::toggled, this, &MainWindow::onToggleDarkMode);
        connect(m_impl->aboutAction, &QAction::triggered, this, &MainWindow::onAbout);
        connect(m_impl->usageAction, &QAction::triggered, this, &MainWindow::onShowUsage);

        // Connect algorithm change to update heuristic/GPU visibility
        connect(m_impl->algorithmComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onAlgorithmChanged);
        // Connect the watcher for async results
        connect(&m_impl->calculationWatcher, &QFutureWatcher<BackendResult>::finished, this, &MainWindow::onCalculationFinished);


        // Initial state
        m_impl->exportButton->setEnabled(false);
        loadSettings(); // Load settings, which might set initial UI state
        onAlgorithmChanged(m_impl->algorithmComboBox->currentIndex()); // Ensure initial UI state for heuristic/GPU is correct
    }

    MainWindow::~MainWindow() = default;

    // =========================================================================
    // Event Overrides
    // =========================================================================
    void MainWindow::closeEvent(QCloseEvent* event)
    {
        if (m_impl->isCalculating) {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(this, "Calculation in Progress",
                "A path calculation is currently running. Are you sure you want to quit?",
                QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::No) {
                event->ignore();
                return;
            }
            // Note: We don't explicitly cancel the QFuture here, it will continue
            // until finished but the result won't be processed by the closed window.
        }
        saveSettings();
        QMainWindow::closeEvent(event);
    }

    // =========================================================================
    // Private Helper Methods - UI Creation
    // =========================================================================
    void MainWindow::createActions() {
        // --- Icons ---
        // Use appropriate icons, maybe custom ones or standard Qt ones
        QIcon mapIcon = QIcon::fromTheme("document-properties", style()->standardIcon(QStyle::SP_FileDialogDetailedView)); // Example theme icon
        QIcon algoIcon = QIcon::fromTheme("preferences-system", style()->standardIcon(QStyle::SP_ComputerIcon));
        QIcon themeIcon = QIcon::fromTheme("preferences-desktop-theme", style()->standardIcon(QStyle::SP_DesktopIcon));
        QIcon aboutIcon = style()->standardIcon(QStyle::SP_DialogHelpButton);
        QIcon usageIcon = style()->standardIcon(QStyle::SP_DialogApplyButton); // Maybe SP_DialogHelpButton too?

        // --- Actions ---
        m_impl->mapSettingsAction = new QAction(mapIcon, "&Map & Processing Settings", this);
        m_impl->mapSettingsAction->setToolTip("Show/hide map processing, elevation, and obstacle cost settings");

        m_impl->algorithmSettingsAction = new QAction(algoIcon, "&Solver Settings", this);
        m_impl->algorithmSettingsAction->setToolTip("Show/hide pathfinding algorithm and heuristic settings");

        m_impl->darkModeAction = new QAction(themeIcon, "&Dark Mode", this);
        m_impl->darkModeAction->setCheckable(true);
        m_impl->darkModeAction->setToolTip("Toggle between light and dark themes.");

        m_impl->aboutAction = new QAction(aboutIcon, "&About", this);
        m_impl->aboutAction->setToolTip("Show information about this application.");

        m_impl->usageAction = new QAction(usageIcon, "&Usage Guide", this);
        m_impl->usageAction->setToolTip("Show basic usage instructions.");
    }

    void MainWindow::createToolBars() {
        QToolBar* settingsToolBar = addToolBar("Controls");
        settingsToolBar->setObjectName("ControlsToolBar");
        settingsToolBar->setMovable(false);
        if (m_impl->mapSettingsAction) settingsToolBar->addAction(m_impl->mapSettingsAction);
        if (m_impl->algorithmSettingsAction) settingsToolBar->addAction(m_impl->algorithmSettingsAction);
        settingsToolBar->addSeparator();
        if (m_impl->darkModeAction) settingsToolBar->addAction(m_impl->darkModeAction);
    }

    void MainWindow::createMenus() {
        QMenuBar* menu = menuBar();
        QMenu* viewMenu = menu->addMenu("&View");
        if (m_impl->darkModeAction) viewMenu->addAction(m_impl->darkModeAction);
        if (m_impl->settingsDockWidget) {
            viewMenu->addSeparator();
            viewMenu->addAction(m_impl->settingsDockWidget->toggleViewAction());
        }
        QMenu* helpMenu = menu->addMenu("&Help");
        if (m_impl->usageAction) helpMenu->addAction(m_impl->usageAction);
        if (m_impl->aboutAction) helpMenu->addAction(m_impl->aboutAction);
    }

    void MainWindow::createSettingsDockWidget() {
        m_impl->settingsDockWidget = new QDockWidget("Settings", this);
        m_impl->settingsDockWidget->setObjectName("SettingsDockWidget");
        m_impl->settingsDockWidget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

        QWidget* dockContentWidget = new QWidget();
        dockContentWidget->setObjectName("SettingsDockContent");
        auto* dockContentLayout = new QVBoxLayout(dockContentWidget);
        dockContentLayout->setContentsMargins(0, 0, 0, 0); // No margins for the main dock layout

        // --- Tab-like Switcher Buttons ---
        auto* switcherLayout = new QHBoxLayout();
        QPushButton* mapProcButton = new QPushButton(QString::fromUtf8("\xF0\x9F\x97\xBA Map & Processing")); // Use map icon
        mapProcButton->setObjectName("GeneratorSettingsButton"); // Keep old name for style compatibility
        QPushButton* solverButton = new QPushButton("Solver");
        solverButton->setObjectName("SolverSettingsButton");
        mapProcButton->setCheckable(true);
        solverButton->setCheckable(true);
        mapProcButton->setChecked(true);
        mapProcButton->setAutoExclusive(true);
        solverButton->setAutoExclusive(true);

        switcherLayout->addWidget(mapProcButton);
        switcherLayout->addWidget(solverButton);
        switcherLayout->setSpacing(0);

        // --- Stacked Widget for Panels ---
        m_impl->settingsStack = new QStackedWidget();
        m_impl->settingsStack->addWidget(createMapSettingsPanel());      // Index 0: Map/Processing
        m_impl->settingsStack->addWidget(createAlgorithmSettingsPanel()); // Index 1: Solver

        connect(mapProcButton, &QPushButton::clicked, this, [this]() { if (m_impl && m_impl->settingsStack) m_impl->settingsStack->setCurrentIndex(0); });
        connect(solverButton, &QPushButton::clicked, this, [this]() { if (m_impl && m_impl->settingsStack) m_impl->settingsStack->setCurrentIndex(1); });

        dockContentLayout->addLayout(switcherLayout);
        dockContentLayout->addWidget(m_impl->settingsStack);

        m_impl->settingsDockWidget->setWidget(dockContentWidget);
        addDockWidget(Qt::RightDockWidgetArea, m_impl->settingsDockWidget);
    }

    // Renamed and Repurposed Panel
    QWidget* MainWindow::createMapSettingsPanel() {
        QWidget* panel = new QWidget();
        auto* panelLayout = new QVBoxLayout(panel);
        panelLayout->setContentsMargins(15, 15, 15, 15); // Add padding inside the panel

        // --- Grid & Threads Group ---
        QGroupBox* gridGroup = new QGroupBox("Grid & Processing");
        auto* gridFormLayout = new QFormLayout(gridGroup);

        m_impl->gridWidthSpinBox = new QSpinBox();
        m_impl->gridWidthSpinBox->setRange(100, 20000); m_impl->gridWidthSpinBox->setSingleStep(100);
        m_impl->gridWidthSpinBox->setToolTip("Target width of the internal logical grid.");
        gridFormLayout->addRow("Grid Width:", m_impl->gridWidthSpinBox);

        m_impl->gridHeightSpinBox = new QSpinBox();
        m_impl->gridHeightSpinBox->setRange(100, 20000); m_impl->gridHeightSpinBox->setSingleStep(100);
        m_impl->gridHeightSpinBox->setToolTip("Target height of the internal logical grid.");
        gridFormLayout->addRow("Grid Height:", m_impl->gridHeightSpinBox);

        m_impl->numThreadsSpinBox = new QSpinBox();
        unsigned int maxThreads = std::thread::hardware_concurrency();
        m_impl->numThreadsSpinBox->setRange(1, std::max(1u, maxThreads));
        m_impl->numThreadsSpinBox->setToolTip("Number of processor threads for map processing (OpenMP).");
        gridFormLayout->addRow("Processing Threads:", m_impl->numThreadsSpinBox);
        panelLayout->addWidget(gridGroup);

        // --- Obstacle Costs Group ---
        QGroupBox* costGroup = new QGroupBox("Obstacle Costs");
        auto* costVLayout = new QVBoxLayout(costGroup);
        QLabel* costLabel = new QLabel("Enter costs (Code: Value) per line:");
        m_impl->obstacleCostsTextEdit = new QTextEdit();
        m_impl->obstacleCostsTextEdit->setPlaceholderText("e.g.,\n201: -1.0\n302: 5.0\n401: 1.0\n...");
        m_impl->obstacleCostsTextEdit->setAcceptRichText(false);
        m_impl->obstacleCostsTextEdit->setMinimumHeight(100); // Give it some space
        m_impl->obstacleCostsTextEdit->setToolTip("Define traversal cost multipliers. Use -1.0 for impassable barriers.");
        costVLayout->addWidget(costLabel);
        costVLayout->addWidget(m_impl->obstacleCostsTextEdit);
        panelLayout->addWidget(costGroup);

        // --- Elevation Group ---
        QGroupBox* elevGroup = new QGroupBox("Elevation Settings");
        auto* elevFormLayout = new QFormLayout(elevGroup);
        m_impl->desiredElevResSpinBox = new QDoubleSpinBox();
        m_impl->desiredElevResSpinBox->setRange(1.0, 1000.0); m_impl->desiredElevResSpinBox->setDecimals(1); m_impl->desiredElevResSpinBox->setSingleStep(10.0);
        m_impl->desiredElevResSpinBox->setToolTip("Desired resolution (meters) for fetched elevation data (if available).");
        elevFormLayout->addRow("Desired Resolution (m):", m_impl->desiredElevResSpinBox);
        // Add Python module/func names here if needed
        panelLayout->addWidget(elevGroup);

        // --- GPU Parameters Group (Initially Hidden) ---
        m_impl->gpuParamsGroup = new QGroupBox("GPU Parameters");
        m_impl->gpuParamsGroup->setToolTip("Settings specific to GPU-based algorithms (Delta-Stepping, HADS, A*).");
        auto* gpuFormLayout = new QFormLayout(m_impl->gpuParamsGroup);

        m_impl->gpuDeltaSpinBox = new QDoubleSpinBox();
        m_impl->gpuDeltaSpinBox->setRange(0.1, 10000.0); m_impl->gpuDeltaSpinBox->setDecimals(1);
        gpuFormLayout->addRow("Delta (Δ):", m_impl->gpuDeltaSpinBox);

        m_impl->gpuThresholdSpinBox = new QDoubleSpinBox();
        m_impl->gpuThresholdSpinBox->setRange(0.1, 10000.0); m_impl->gpuThresholdSpinBox->setDecimals(1);
        gpuFormLayout->addRow("Light Edge Threshold:", m_impl->gpuThresholdSpinBox);

        m_impl->hadsRadiusSpinBox = new QSpinBox();
        m_impl->hadsRadiusSpinBox->setRange(1, 10000);
        gpuFormLayout->addRow("HADS Heuristic Radius:", m_impl->hadsRadiusSpinBox);

        m_impl->hadsPruneSpinBox = new QDoubleSpinBox();
        m_impl->hadsPruneSpinBox->setRange(1.0, 5.0); m_impl->hadsPruneSpinBox->setDecimals(2); m_impl->hadsPruneSpinBox->setSingleStep(0.05);
        gpuFormLayout->addRow("HADS Pruning Factor:", m_impl->hadsPruneSpinBox);

        m_impl->hadsWeightSpinBox = new QDoubleSpinBox();
        m_impl->hadsWeightSpinBox->setRange(0.1, 5.0); m_impl->hadsWeightSpinBox->setDecimals(2); m_impl->hadsWeightSpinBox->setSingleStep(0.05);
        gpuFormLayout->addRow("HADS Heuristic Weight:", m_impl->hadsWeightSpinBox);

        m_impl->gpuParamsGroup->setVisible(false); // Initially hidden
        panelLayout->addWidget(m_impl->gpuParamsGroup);

        panelLayout->addStretch(); // Push content upwards

        return panel;
    }

    QWidget* MainWindow::createAlgorithmSettingsPanel() {
        QWidget* panel = new QWidget();
        auto* panelLayout = new QVBoxLayout(panel);
        panelLayout->setContentsMargins(15, 15, 15, 15);

        QGroupBox* algoGroup = new QGroupBox("Pathfinding Algorithm");
        auto* formLayout = new QFormLayout(algoGroup);

        m_impl->algorithmComboBox = new QComboBox();
        // Make sure these strings exactly match expected names in backend/parsing logic
        // Store the actual algorithm name string as QString in the QVariant user data
        m_impl->algorithmComboBox->addItem("Optimized A*", QVariant(QString("Optimized A*")));
        m_impl->algorithmComboBox->addItem("Dijkstra", QVariant(QString("Dijkstra")));
        m_impl->algorithmComboBox->addItem("BFS", QVariant(QString("BFS")));
        m_impl->algorithmComboBox->addItem("Theta*", QVariant(QString("Theta*")));
        m_impl->algorithmComboBox->addItem("Lazy Theta*", QVariant(QString("Lazy Theta*"))); // Ensure backend uses this exact name string
        m_impl->algorithmComboBox->addItem("Delta Stepping - GPU", QVariant(QString("Delta Stepping - GPU")));
        m_impl->algorithmComboBox->addItem("HADS - GPU", QVariant(QString("HADS - GPU")));
        m_impl->algorithmComboBox->addItem("A* - GPU", QVariant(QString("A* - GPU")));
        m_impl->algorithmComboBox->setToolTip("Select the algorithm to find the path.");
        formLayout->addRow("Algorithm:", m_impl->algorithmComboBox);

        // Add Heuristic Selection ComboBox
        m_impl->heuristicComboBox = new QComboBox();
        m_impl->heuristicComboBox->addItem("Euclidean", QVariant(PathfindingUtils::HEURISTIC_EUCLIDEAN));
        m_impl->heuristicComboBox->addItem("Diagonal", QVariant(PathfindingUtils::HEURISTIC_DIAGONAL));
        m_impl->heuristicComboBox->addItem("Manhattan", QVariant(PathfindingUtils::HEURISTIC_MANHATTAN));
        m_impl->heuristicComboBox->addItem("Min Cost", QVariant(PathfindingUtils::HEURISTIC_MIN_COST));
        m_impl->heuristicComboBox->setToolTip("Select the heuristic function for A* and related algorithms.");
        m_impl->heuristicComboBox->setEnabled(false); // Disabled by default
        formLayout->addRow("Heuristic (A*/Theta*):", m_impl->heuristicComboBox);


        panelLayout->addWidget(algoGroup);
        panelLayout->addStretch();
        return panel;
    }

    // =========================================================================
    // Public Slots
    // =========================================================================

    void MainWindow::onBrowseMapFile() {
        QString filePath = browseFileDialog("Select Map File", "OMAP Files (*.omap);;XML Files (*.xml);;All Files (*)");
        if (!filePath.isEmpty() && m_impl) {
            m_impl->mapLineEdit->setText(filePath);
        }
    }

    void MainWindow::onBrowseControlsFile() {
        QString filePath = browseFileDialog("Select Controls File", "OMAP Files (*.omap);;XML Files (*.xml);;All Files (*)");
        if (!filePath.isEmpty() && m_impl) {
            m_impl->controlsLineEdit->setText(filePath);
        }
    }


    void MainWindow::onCalculatePathClicked() {
        if (!m_impl || m_impl->isCalculating) return;

        // 1. Gather Inputs
        
        BackendInputParams params;
        params.mapFilePath = m_impl->mapLineEdit->text().toStdString();
        params.controlsFilePath = m_impl->controlsLineEdit->text().toStdString();
        params.desiredGridWidth = m_impl->gridWidthSpinBox->value();
        params.desiredGridHeight = m_impl->gridHeightSpinBox->value();
        params.numThreads = static_cast<unsigned int>(m_impl->numThreadsSpinBox->value());
        params.desiredElevationResolution = m_impl->desiredElevResSpinBox->value();
        params.algorithmName = m_impl->algorithmComboBox->currentData().toString().toStdString(); // Get std::string from QVariant
        
        // Parse Obstacle Costs
        if (!parseObstacleCosts(params.obstacleCosts)) {
            QMessageBox::warning(this, "Input Error", "Invalid format in Obstacle Costs. Please use 'Code: Value' per line (e.g., '201: -1.0').");
            return;
        }

        // Get Heuristic (if applicable)
        if (m_impl->heuristicComboBox->isEnabled()) {
            params.heuristicType = m_impl->heuristicComboBox->currentData().toInt();
        }
        else {
            params.heuristicType = -1; // Indicate not applicable
        }

        // Get GPU Params (if applicable)
        if (m_impl->gpuParamsGroup->isVisible()) {
            params.gpuDelta = static_cast<float>(m_impl->gpuDeltaSpinBox->value());
            params.gpuThreshold = static_cast<float>(m_impl->gpuThresholdSpinBox->value());
            params.hadsRadius = m_impl->hadsRadiusSpinBox->value();
            params.hadsPruneFactor = static_cast<float>(m_impl->hadsPruneSpinBox->value());
            params.hadsHeuristicWeight = static_cast<float>(m_impl->hadsWeightSpinBox->value());
        }

        // 2. Validate Basic Inputs
        if (params.mapFilePath.empty() || params.controlsFilePath.empty()) {
            QMessageBox::warning(this, "Input Error", "Please select both a Map file and a Controls file.");
            return;
        }
        QFileInfo mapInfo(QString::fromStdString(params.mapFilePath));
        QFileInfo controlsInfo(QString::fromStdString(params.controlsFilePath));
        if (!mapInfo.exists() || !mapInfo.isFile()) {
            QMessageBox::warning(this, "Input Error", "Map file not found: " + mapInfo.filePath());
            return;
        }
        if (!controlsInfo.exists() || !controlsInfo.isFile()) {
            QMessageBox::warning(this, "Input Error", "Controls file not found: " + controlsInfo.filePath());
            return;
        }

        // 3. Check for Grid Reuse
        // Basic check: If map path and dimensions match previous run AND grid exists
        if (m_impl->lastProcessedGrid.has_value() &&
            m_impl->currentMapFilePath == params.mapFilePath &&
            m_impl->lastGridWidth == params.desiredGridWidth &&
            m_impl->lastGridHeight == params.desiredGridHeight)
        {
            // More complex: Could also check if obstacle costs affecting grid gen changed
            params.reuseGridIfPossible = true;
            params.existingGrid = m_impl->lastProcessedGrid;
            params.existingNormInfo = m_impl->lastNormalizationInfo;
            qDebug() << "Requesting grid reuse for map:" << mapInfo.fileName();
        }
        else {
            params.reuseGridIfPossible = false;
            qDebug() << "Grid reuse parameters changed or no previous grid. Regenerating.";
        }

        // 4. Start Asynchronous Calculation
        runBackendProcessingAsync(params);
    }


    void MainWindow::onCalculationFinished() {
        if (!m_impl) return;

        BackendResult result = m_impl->calculationWatcher.result();

        setGuiCalculating(false);

        // --- Store results ---
        m_impl->currentMapFilePath = result.usedMapFilePath;
        m_impl->currentControlsFilePath = result.currentControlsFilePath; // *** STORED HERE ***
        m_impl->lastGridWidth = result.usedGridWidth;
        m_impl->lastGridHeight = result.usedGridHeight;
        m_impl->lastProcessedGrid = std::move(result.processedGrid);
        m_impl->lastNormalizationInfo = std::move(result.normalizationInfo);
        m_impl->lastElevationDataUsed = std::move(result.elevationDataUsed);
        m_impl->lastLogicalResolutionMeters = result.finalLogicalResolutionMeters;
        m_impl->lastOriginOffsetX = result.finalOriginOffsetX;
        m_impl->lastOriginOffsetY = result.finalOriginOffsetY;
        m_impl->lastCalculatedPathIndices = std::move(result.fullPathIndices);


        // --- Process Outcome ---
        if (result.success) {
            m_impl->exportButton->setEnabled(!m_impl->lastCalculatedPathIndices.empty());

            // Format result message
            QString statusMsg = QString("Path Found (%1 waypoints). Length: %2 nodes.")
                .arg(result.waypointsFound)
                .arg(m_impl->lastCalculatedPathIndices.size()); // Use size AFTER move
            QString timingMsg = QString("Timing: Map Proc: %L1ms | Elev Fetch: %L2ms | Pathfinding: %L3ms")
                .arg(result.mapProcessingDurationMs, 0, 'f', 1)
                .arg(result.elevationFetchDurationMs, 0, 'f', 1)
                .arg(result.pathfindingDurationMs, 0, 'f', 1);
            if (result.usedDummyElevation) {
                statusMsg += " (Used dummy elevation data)";
            }
            m_impl->statusBar->showMessage(statusMsg + " | " + timingMsg, 15000);

            qDebug() << "MainWindow: Calculation successful. Path length:" << m_impl->lastCalculatedPathIndices.size();

            // Handle Auto-Export or Prompt
            if (m_impl->autoExportCheckBox->isChecked() && !m_impl->lastCalculatedPathIndices.empty()) {
                qDebug() << "MainWindow: Auto-export triggered.";
                triggerExport();
            }
            else if (!m_impl->lastCalculatedPathIndices.empty()) {
                QMessageBox msgBox(this);
                msgBox.setWindowTitle("Calculation Complete");
                msgBox.setText(statusMsg + "\n" + timingMsg + "\n\nSave calculated path to file?");
                msgBox.setIcon(QMessageBox::Question);
                msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
                msgBox.setDefaultButton(QMessageBox::Yes);
                if (msgBox.exec() == QMessageBox::Yes) {
                    qDebug() << "MainWindow: User chose to export.";
                    triggerExport();
                }
            }

        }
        else {
            // Failure
            // m_impl->lastCalculatedPathIndices is already empty due to move or clear on failure
            m_impl->exportButton->setEnabled(false);
            QString error = QString::fromStdString(result.errorMessage);
            if (error.isEmpty()) error = "An unknown error occurred during processing.";
            QMessageBox::critical(this, "Calculation Error", error);
            m_impl->statusBar->showMessage("Calculation failed.", 5000);
            qDebug() << "MainWindow: Calculation failed:" << error;
        }
    }



    void MainWindow::onMapSettingsClicked() {
        if (m_impl && m_impl->settingsDockWidget) {
            m_impl->settingsDockWidget->show();
            m_impl->settingsDockWidget->raise();
            if (m_impl->settingsStack) m_impl->settingsStack->setCurrentIndex(0); // Show Map/Proc Panel
            QPushButton* mapProcButton = m_impl->settingsDockWidget->findChild<QPushButton*>("GeneratorSettingsButton"); // Use the object name
            if (mapProcButton) mapProcButton->setChecked(true);
            if (m_impl->gridWidthSpinBox) m_impl->gridWidthSpinBox->setFocus();
        }
    }

    void MainWindow::onAlgorithmSettingsClicked() {
        if (m_impl && m_impl->settingsDockWidget) {
            m_impl->settingsDockWidget->show();
            m_impl->settingsDockWidget->raise();
            if (m_impl->settingsStack) m_impl->settingsStack->setCurrentIndex(1); // Show Solver Panel
            QPushButton* solverButton = m_impl->settingsDockWidget->findChild<QPushButton*>("SolverSettingsButton");
            if (solverButton) solverButton->setChecked(true);
            if (m_impl->algorithmComboBox) m_impl->algorithmComboBox->setFocus();
        }
    }

    void MainWindow::onToggleDarkMode(bool checked) {
        applyStylesheet(checked);
    }

    void MainWindow::onAlgorithmChanged(int /*index*/) {
        if (!m_impl) return;

        QString algoName = m_impl->algorithmComboBox->currentText(); // More robust than currentData maybe

        // Algorithms that use heuristics
        bool usesHeuristic = algoName.contains("A*", Qt::CaseInsensitive) ||
            algoName.contains("Theta*", Qt::CaseInsensitive);

        // Algorithms that are GPU based (for GPU params)
        bool usesGpuParams = algoName.contains("GPU", Qt::CaseInsensitive);

        m_impl->heuristicComboBox->setEnabled(usesHeuristic);
        m_impl->gpuParamsGroup->setVisible(usesGpuParams);

        qDebug() << "Algorithm changed to:" << algoName << "Uses heuristic:" << usesHeuristic << "Uses GPU params:" << usesGpuParams;
    }

    void MainWindow::triggerExport() {
        if (!m_impl || m_impl->isCalculating) return;

        // --- Validation ---
        if (m_impl->lastCalculatedPathIndices.empty()) {
            QMessageBox::information(this, "Export Path", "No path data available to export. Please calculate a path first.");
            return;
        }
        if (!m_impl->lastProcessedGrid.has_value() || !m_impl->lastNormalizationInfo.has_value()) {
            QMessageBox::critical(this, "Export Error", "Cannot export path: Missing required map grid or normalization data.");
            return;
        }
        // *** Check the stored controls path ***
        if (m_impl->currentControlsFilePath.empty()) {
            QMessageBox::critical(this, "Export Error", "Cannot determine output filename: Controls file path from the last calculation run is missing.");
            return;
        }


        // --- Determine Output Filename ---
        QString outputOmapPath;
        QString controlsPath = QString::fromStdString(m_impl->currentControlsFilePath); // Use stored path
        QFileInfo controlsInfo(controlsPath);
        QString baseName = controlsInfo.completeBaseName();
        QString autoFileName = baseName + "_path.omap"; // Construct the default filename part

        if (m_impl->autoExportCheckBox->isChecked()) {
            // Auto-generate filename based on controls file's directory
            QString dirPath = controlsInfo.absolutePath(); // Get directory from original controls file
            if (dirPath.isEmpty()) { // Handle cases where path might be relative/invalid
                QMessageBox::critical(this, "Export Error", "Cannot determine directory from controls file path for auto-export.");
                return;
            }
            outputOmapPath = dirPath + "/" + autoFileName;
            qDebug() << "MainWindow: Auto-generating export filename:" << outputOmapPath;
        }
        else {
            // Ask user for DIRECTORY
            QString lastDir = QDir::homePath();
            if (m_impl->settings) {
                // Start in directory of controls file or last export dir
                lastDir = m_impl->settings->value("lastExportDir", controlsInfo.absolutePath()).toString();
            }

            QString selectedDir = QFileDialog::getExistingDirectory(this, "Select Folder to Save Path File", lastDir);

            if (selectedDir.isEmpty()) {
                m_impl->statusBar->showMessage("Export cancelled.", 3000);
                qDebug() << "MainWindow: User cancelled directory selection.";
                return; // User cancelled
            }

            // Store last used directory
            if (m_impl->settings) {
                m_impl->settings->setValue("lastExportDir", selectedDir);
            }

            // Construct full path using selected directory and auto-generated name
            outputOmapPath = QDir(selectedDir).filePath(autoFileName); // Use QDir for robustness
            qDebug() << "MainWindow: User selected export directory:" << selectedDir << "-> Full path:" << outputOmapPath;
        }


        // --- Perform Save Operation ---
        m_impl->statusBar->showMessage("Exporting path to " + QFileInfo(outputOmapPath).fileName() + "...", 0);
        setGuiCalculating(true);
        QCoreApplication::processEvents();

        bool success = false;
        std::string errorMsg;
        try {
            qDebug() << "MainWindow: Calling PathSaver::savePathToOmap with output:" << outputOmapPath;
            // Assuming PathSaver handles the copy OR you add copy logic here
            // You might need to pass the original controlsPath to savePathToOmap if it needs it for copying
            // success = pathsaver::savePathToOmap(outputOmapPath.toStdString(), controlsPath.toStdString(), ...); // Example if pathsaver needs controls path

            // Current implementation assumes pathsaver only needs output path and data
            success = pathsaver::savePathToOmap(
                outputOmapPath.toStdString(),
                m_impl->lastCalculatedPathIndices,
                m_impl->lastProcessedGrid.value(),
                m_impl->lastNormalizationInfo.value(),
                "704",
                "course"
            );
            if (!success) {
                errorMsg = "PathSaver function returned false. Check logs.";
            }
        }
        catch (const std::exception& e) {
            success = false;
            errorMsg = std::string("Standard Exception during export: ") + e.what();
        }
        catch (...) {
            success = false;
            errorMsg = "Unknown exception occurred during export.";
        }

        setGuiCalculating(false);

        // --- Report Result ---
        if (success) {
            QMessageBox::information(this, "Export Successful", "Path saved successfully to:\n" + outputOmapPath);
            m_impl->statusBar->showMessage("Path exported successfully.", 5000);
            qDebug() << "MainWindow: Export successful.";
        }
        else {
            QMessageBox::critical(this, "Export Failed", "Could not save path.\nError: " + QString::fromStdString(errorMsg));
            m_impl->statusBar->showMessage("Export failed.", 5000);
            qDebug() << "MainWindow: Export failed:" << QString::fromStdString(errorMsg);
        }
    }
    void MainWindow::onAutoExportToggled(bool /*checked*/)
    {
        // Optional: Save this preference immediately if desired
        // if(m_impl && m_impl->settings) {
        //     m_impl->settings->setValue("export/autoExport", checked);
        // }
    }


    void MainWindow::onAbout() {
        QMessageBox::about(this, "About OMAP Pathfinding Processor",
            "<h2>OMAP Pathfinding Processor v1.1</h2>"
            "<p>Copyright © 2024 Your Name/Company</p>"
            "<p>Processes OMAP maps and finds paths using various algorithms.</p>"
            "<p>Integrates elevation data via Python and supports GPU acceleration.</p>"
            "<p>Built with Qt and C++.</p>");
    }

    void MainWindow::onShowUsage() {
        QMessageBox::information(this, "Usage Guide",
            "1.  <b>Select Files:</b> Choose the main map `.omap` file and the controls `.omap` file (containing Start 701, Finish 706, etc.).\n"
            "2.  <b>Configure Map & Processing:</b> Use 'Map & Processing Settings' (🗺️) to set target grid dimensions, obstacle costs, elevation resolution, and processing threads.\n"
            "3.  <b>Configure Solver:</b> Use 'Solver Settings' (⚙️) to choose the pathfinding algorithm and heuristic (if applicable). GPU parameters appear here if a GPU algorithm is selected.\n"
            "4.  <b>Calculate Path:</b> Click the 'Calculate Path' (▶️) button. This processes the map and finds the path between waypoints defined in the controls file.\n"
            "5.  <b>Export Results:</b> If a path is found, you will be prompted to save it, or it will save automatically if 'Auto Export' is checked. You can also click 'Export Path' (💾) later.\n\n"
            "Use the View menu or toolbar (🔆) to toggle Dark Mode and settings panel visibility.");
    }

    // =========================================================================
    // Private Helper Methods - Settings & Style
    // =========================================================================
    void MainWindow::loadSettings() {
        if (!m_impl || !m_impl->settings) return;
        m_impl->settings->beginGroup("MainWindow");
        restoreGeometry(m_impl->settings->value("geometry").toByteArray());
        restoreState(m_impl->settings->value("windowState").toByteArray());
        bool dockVisible = m_impl->settings->value("settingsDockVisible", true).toBool();
        if (m_impl->settingsDockWidget && !dockVisible) m_impl->settingsDockWidget->hide();
        bool darkMode = m_impl->settings->value("darkModeEnabled", false).toBool();
        m_impl->settings->endGroup();


        // Theme
        if (m_impl->darkModeAction) {
            m_impl->darkModeAction->blockSignals(true);
            m_impl->darkModeAction->setChecked(darkMode);
            m_impl->darkModeAction->blockSignals(false);
        }
        applyStylesheet(darkMode);

        // File Paths
        m_impl->settings->beginGroup("Files");
        m_impl->mapLineEdit->setText(m_impl->settings->value("lastMapFile", "").toString());
        m_impl->controlsLineEdit->setText(m_impl->settings->value("lastControlsFile", "").toString());
        m_impl->settings->endGroup();

        // Map & Processing Settings
        m_impl->settings->beginGroup("Processing");
        if (m_impl->gridWidthSpinBox) m_impl->gridWidthSpinBox->setValue(m_impl->settings->value("gridWidth", 1000).toInt());
        if (m_impl->gridHeightSpinBox) m_impl->gridHeightSpinBox->setValue(m_impl->settings->value("gridHeight", 1000).toInt());
        if (m_impl->numThreadsSpinBox) m_impl->numThreadsSpinBox->setValue(m_impl->settings->value("numThreads", std::max(1u, std::thread::hardware_concurrency())).toUInt());
        if (m_impl->desiredElevResSpinBox) m_impl->desiredElevResSpinBox->setValue(m_impl->settings->value("elevationResolution", 90.0).toDouble());
        // Default obstacle costs (used if loading fails or first time)
        QString defaultCosts = "201: -1.0\n301: -1.0\n307: -1.0\n509: -1.0\n513: -1.0\n514: -1.0\n515: -1.0\n516: -1.0\n520: -1.0\n526: -1.0\n528: -1.0\n529: -1.0\n206: -1.0\n417: -1.0\n518: -1.0\n202: 10.0\n210: 1.25\n211: 1.67\n212: 5.0\n213: 1.25\n302: 5.0\n308: 2.0\n309: 1.67\n310: 1.43\n403: 1.25\n404: 1.25\n406: 1.50\n407: 1.50\n408: 1.67\n409: 1.67\n410: 5.0\n412: 1.11\n413: 1.11\n414: 1.11\n311: 1.01\n401: 1.0\n402: 1.0\n405: 1.0\n501: 0.6\n502: 0.6\n503: 0.6\n504: 0.6\n505: 0.6\n506: 0.65\n507: 0.75\n508: 0.8\n519: 0.9\n527: 1.0";
        if (m_impl->obstacleCostsTextEdit) m_impl->obstacleCostsTextEdit->setText(m_impl->settings->value("obstacleCosts", defaultCosts).toString());
        m_impl->settings->endGroup();


        // Solver Settings
        m_impl->settings->beginGroup("Solver");
        QString defaultAlgoName = "Optimized A*";
        QString savedAlgoName = m_impl->settings->value("algorithm", defaultAlgoName).toString();
        int algoIndex = m_impl->algorithmComboBox->findText(savedAlgoName);
        m_impl->algorithmComboBox->setCurrentIndex((algoIndex != -1) ? algoIndex : 0);

        int defaultHeuristic = PathfindingUtils::HEURISTIC_MIN_COST;
        int savedHeuristic = m_impl->settings->value("heuristic", defaultHeuristic).toInt();
        int heuristicIndex = m_impl->heuristicComboBox->findData(QVariant(savedHeuristic));
        m_impl->heuristicComboBox->setCurrentIndex((heuristicIndex != -1) ? heuristicIndex : 3); // Default to Min Cost index

        // GPU Defaults (from backend main example)
        if (m_impl->gpuDeltaSpinBox) m_impl->gpuDeltaSpinBox->setValue(m_impl->settings->value("gpuDelta", 50.0).toDouble());
        if (m_impl->gpuThresholdSpinBox) m_impl->gpuThresholdSpinBox->setValue(m_impl->settings->value("gpuThreshold", 50.0).toDouble());
        if (m_impl->hadsRadiusSpinBox) m_impl->hadsRadiusSpinBox->setValue(m_impl->settings->value("hadsRadius", 1000).toInt());
        if (m_impl->hadsPruneSpinBox) m_impl->hadsPruneSpinBox->setValue(m_impl->settings->value("hadsPruneFactor", 1.05).toDouble());
        if (m_impl->hadsWeightSpinBox) m_impl->hadsWeightSpinBox->setValue(m_impl->settings->value("hadsWeight", 0.95).toDouble());
        m_impl->settings->endGroup();

        // Export Settings
        m_impl->settings->beginGroup("Export");
        if (m_impl->autoExportCheckBox) m_impl->autoExportCheckBox->setChecked(m_impl->settings->value("autoExport", false).toBool());
        m_impl->settings->endGroup();

    }

    void MainWindow::saveSettings() {
        if (!m_impl || !m_impl->settings) return;

        m_impl->settings->beginGroup("MainWindow");
        m_impl->settings->setValue("geometry", saveGeometry());
        m_impl->settings->setValue("windowState", saveState());
        if (m_impl->settingsDockWidget) m_impl->settings->setValue("settingsDockVisible", m_impl->settingsDockWidget->isVisible());
        m_impl->settings->setValue("darkModeEnabled", m_impl->darkModeAction ? m_impl->darkModeAction->isChecked() : false);
        m_impl->settings->endGroup();

        m_impl->settings->beginGroup("Files");
        m_impl->settings->setValue("lastMapFile", m_impl->mapLineEdit->text());
        m_impl->settings->setValue("lastControlsFile", m_impl->controlsLineEdit->text());
        m_impl->settings->setValue("lastOpenDir", m_impl->settings->value("lastOpenDir", QDir::homePath())); // Preserve last browse dir
        m_impl->settings->endGroup();


        m_impl->settings->beginGroup("Processing");
        if (m_impl->gridWidthSpinBox) m_impl->settings->setValue("gridWidth", m_impl->gridWidthSpinBox->value());
        if (m_impl->gridHeightSpinBox) m_impl->settings->setValue("gridHeight", m_impl->gridHeightSpinBox->value());
        if (m_impl->numThreadsSpinBox) m_impl->settings->setValue("numThreads", m_impl->numThreadsSpinBox->value());
        if (m_impl->desiredElevResSpinBox) m_impl->settings->setValue("elevationResolution", m_impl->desiredElevResSpinBox->value());
        if (m_impl->obstacleCostsTextEdit) m_impl->settings->setValue("obstacleCosts", m_impl->obstacleCostsTextEdit->toPlainText());
        m_impl->settings->endGroup();

        m_impl->settings->beginGroup("Solver");
        if (m_impl->algorithmComboBox) m_impl->settings->setValue("algorithm", m_impl->algorithmComboBox->currentText()); // Save name
        if (m_impl->heuristicComboBox) m_impl->settings->setValue("heuristic", m_impl->heuristicComboBox->currentData().toInt()); // Save int constant

        if (m_impl->gpuDeltaSpinBox) m_impl->settings->setValue("gpuDelta", m_impl->gpuDeltaSpinBox->value());
        if (m_impl->gpuThresholdSpinBox) m_impl->settings->setValue("gpuThreshold", m_impl->gpuThresholdSpinBox->value());
        if (m_impl->hadsRadiusSpinBox) m_impl->settings->setValue("hadsRadius", m_impl->hadsRadiusSpinBox->value());
        if (m_impl->hadsPruneSpinBox) m_impl->settings->setValue("hadsPruneFactor", m_impl->hadsPruneSpinBox->value());
        if (m_impl->hadsWeightSpinBox) m_impl->settings->setValue("hadsWeight", m_impl->hadsWeightSpinBox->value());
        m_impl->settings->endGroup();

        m_impl->settings->beginGroup("Export");
        if (m_impl->autoExportCheckBox) m_impl->settings->setValue("autoExport", m_impl->autoExportCheckBox->isChecked());
        m_impl->settings->endGroup();

        m_impl->settings->sync(); // Ensure settings are written
    }

    void MainWindow::applyStylesheet(bool dark) {
        qApp->setStyleSheet(dark ? darkStylesheet : lightStylesheet);
        // Optionally: Force style refresh if icons don't update
        // QEvent event(QEvent::StyleChange);
        // QApplication::sendEvent(this, &event);
        // QApplication::processEvents();
    }

    // =========================================================================
    // Private Helper Methods - File Dialog & Backend Integration
    // =========================================================================
    QString MainWindow::browseFileDialog(const QString& title, const QString& filter) {
        QString filePath;
        QString lastDir = QDir::homePath();
        if (m_impl && m_impl->settings) {
            lastDir = m_impl->settings->value("lastOpenDir", QDir::homePath()).toString();
        }

        filePath = QFileDialog::getOpenFileName(this, title, lastDir, filter);


        if (!filePath.isEmpty() && m_impl && m_impl->settings) {
            m_impl->settings->setValue("lastOpenDir", QFileInfo(filePath).absolutePath());
        }
        return filePath;
    }

    void MainWindow::setGuiCalculating(bool calculating) {
        if (!m_impl) return;
        m_impl->isCalculating = calculating;

        // Disable/Enable controls during calculation
        m_impl->calculatePathButton->setEnabled(!calculating);
        m_impl->exportButton->setEnabled(!calculating && !m_impl->lastCalculatedPathIndices.empty());
        m_impl->browseMapButton->setEnabled(!calculating);
        m_impl->browseControlsButton->setEnabled(!calculating);
        m_impl->mapLineEdit->setEnabled(!calculating);
        m_impl->controlsLineEdit->setEnabled(!calculating);
        m_impl->autoExportCheckBox->setEnabled(!calculating);

        // Disable settings panels
        if (m_impl->settingsDockWidget) {
            // Find the content widget of the dock and disable it
            QWidget* dockContent = m_impl->settingsDockWidget->widget();
            if (dockContent) dockContent->setEnabled(!calculating);
        }

        if (calculating) {
            m_impl->statusBar->showMessage("Calculating path...", 0); // Persistent message
        }
        else {
            m_impl->statusBar->clearMessage(); // Clear persistent message
        }
        // Ensure UI updates visually
        QCoreApplication::processEvents();
    }


    bool MainWindow::parseObstacleCosts(mapgeo::ObstacleConfigMap& configMap) {
        configMap.clear();
        if (!m_impl || !m_impl->obstacleCostsTextEdit) return false;

        QString text = m_impl->obstacleCostsTextEdit->toPlainText();
        std::stringstream ss(text.toStdString());
        std::string line;
        int lineNumber = 0;

        while (std::getline(ss, line)) {
            lineNumber++;
            // Trim whitespace
            line.erase(0, line.find_first_not_of(" \t\n\r"));
            line.erase(line.find_last_not_of(" \t\n\r") + 1);

            if (line.empty() || line[0] == '#') { // Skip empty lines and comments
                continue;
            }

            size_t colonPos = line.find(':');
            if (colonPos == std::string::npos) {
                qWarning() << "Obstacle cost parsing error: Missing ':' on line" << lineNumber << ":" << QString::fromStdString(line);
                return false; // Invalid format
            }

            std::string codeStr = line.substr(0, colonPos);
            std::string valueStr = line.substr(colonPos + 1);

            // Trim whitespace around code and value
            codeStr.erase(0, codeStr.find_first_not_of(" \t"));
            codeStr.erase(codeStr.find_last_not_of(" \t") + 1);
            valueStr.erase(0, valueStr.find_first_not_of(" \t"));
            valueStr.erase(valueStr.find_last_not_of(" \t") + 1);


            if (codeStr.empty()) {
                qWarning() << "Obstacle cost parsing error: Empty code on line" << lineNumber;
                return false;
            }

            try {
                float value = std::stof(valueStr);
                configMap[codeStr] = value;
            }
            catch (const std::invalid_argument& e) {
                qWarning() << "Obstacle cost parsing error: Invalid float value '" << QString::fromStdString(valueStr) << "' on line" << lineNumber << ". Error:" << e.what();
                return false;
            }
            catch (const std::out_of_range& e) {
                qWarning() << "Obstacle cost parsing error: Float value out of range '" << QString::fromStdString(valueStr) << "' on line" << lineNumber << ". Error:" << e.what();
                return false;
            }
        }
        qDebug() << "Parsed" << configMap.size() << "obstacle cost entries.";
        return true;
    }


    // =========================================================================
    // runBackendProcessingAsync (Modified to call PathfindingLogic member)
    // =========================================================================
    void MainWindow::runBackendProcessingAsync(const BackendInputParams& params) {
        if (!m_impl) return;
        qDebug() << "MainWindow: Starting PathfindingLogic processing asynchronously...";
        setGuiCalculating(true);

        // Use QtConcurrent to run the MEMBER function via a lambda
        QFuture<BackendResult> future = QtConcurrent::run([this, params]() {
            // Capture 'this' to access m_impl safely
            if (!m_impl) {
                // This should be rare, but handle defensively
                BackendResult errorResult;
                errorResult.success = false;
                errorResult.errorMessage = "Internal error: GUI state lost before processing could start.";
                //qCritical() << errorResult.errorMessage;
                return errorResult;
            }
            // Call the member function on the logic object stored in m_impl
            return m_impl->pathfindingLogic.processAndFindPath(params);
            });

        m_impl->calculationWatcher.setFuture(future);
    }


} // namespace app