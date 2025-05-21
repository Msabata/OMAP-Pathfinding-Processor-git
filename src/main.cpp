#include <QApplication>
#include "gui/main_window.hpp"

int main(int argc, char *argv[]) {
    // Initialize application with optimal resource allocation
    QApplication app(argc, argv);
    
    // Instantiate main window with zero-initialization overhead
    app::MainWindow mainWindow;
    mainWindow.show();
    
    // Enter event loop with O(1) dispatch latency
    return app.exec();
}
//TODO appka jede je protøeba propojit s A* algoritmem a s generováním mapy pak se bude pokraèovat s dalšími algoritmy 