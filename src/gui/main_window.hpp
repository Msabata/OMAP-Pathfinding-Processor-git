#pragma once
// src/gui/main_window.hpp
#ifndef APP_MAIN_WINDOW_HPP
#define APP_MAIN_WINDOW_HPP

#include <QMainWindow>
#include <memory>
#include <string>
#include <vector>
#include <optional>

// --- Qt Headers ---
#include <QCloseEvent> // Already included
#include <QFutureWatcher> // For async execution

// --- Backend Headers (Adjust paths as necessary) ---
#include "map/MapProcessingCommon.h" // For GridPoint, ObstacleConfigMap, NormalizationResult
#include "map/MapProcessor.hpp"      // For Grid_V3
#include "map/ElevationFetchingCommon.hpp" // For ElevationData
#include "map/PathfindingUtils.hpp"      // For GridPoint (if not in MapProcessingCommon), heuristic constants
#include "map/ElevationFetcherPy.hpp"
#include "logic/BackendInterface.hpp"

// Forward declarations for Qt Widgets used in Impl
class QAction;
class QSettings;
class QSpinBox;
class QDoubleSpinBox;
class QPushButton;
class QGroupBox;
class QLineEdit;
class QTextEdit;
class QComboBox;
class QCheckBox;
class QStatusBar;
class QDockWidget;
class QStackedWidget;


// --- Backend Interface Structs ---


namespace app {

    class MainWindow : public QMainWindow {
        Q_OBJECT

    public:
        explicit MainWindow(QWidget* parent = nullptr);
        ~MainWindow() override;

    protected:
        void closeEvent(QCloseEvent* event) override;

    private slots:
        // File Handling
        void onBrowseMapFile();
        void onBrowseControlsFile();

        // Calculation Trigger & Result Handling
        void onCalculatePathClicked(); // Replaces onGenerateMapClicked/onCalculateClicked
        void onCalculationFinished();  // Slot for QFutureWatcher

        // Settings & UI Interaction
        void onMapSettingsClicked();
        void onAlgorithmSettingsClicked();
        void onToggleDarkMode(bool checked);
        void onAlgorithmChanged(int index); // Slot for algorithmComboBox change
        // No specific slot needed for heuristic, read directly before calculation

        // Export
        void triggerExport(); // Connected to exportButton and potentially auto-export logic
        void onAutoExportToggled(bool checked);

        // Help
        void onAbout();
        void onShowUsage();


    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;

        // Private Helper Methods
        void createActions();
        void createToolBars();
        void createMenus();
        void createSettingsDockWidget();
        QWidget* createMapSettingsPanel();      // Renamed/Repurposed
        QWidget* createAlgorithmSettingsPanel();
        QString browseFileDialog(const QString& title, const QString& filter);
        void loadSettings();
        void saveSettings();
        void applyStylesheet(bool dark);
        void setGuiCalculating(bool calculating); // Helper to enable/disable UI

        // Backend Integration Helpers
        bool parseObstacleCosts(mapgeo::ObstacleConfigMap& configMap); // Parses text edit
        void runBackendProcessingAsync(const BackendInputParams& params); // Starts the async task

    }; // class MainWindow

} // namespace app

#endif // APP_MAIN_WINDOW_HPP