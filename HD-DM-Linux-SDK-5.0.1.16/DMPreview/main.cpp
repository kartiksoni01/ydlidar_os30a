#include "mainwindow.h"
#include <QApplication>
#include <QMainWindow>
#include "eSPDI_version.h"
#include "CPointCloudViewerWidget.h"

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    MainWindow mainWindow;
    //mainWindow.setWindowIcon(QIcon(":/image/eys3d.png"));
    QString title = "DMPreview ";
    title += APC_VERSION;
    mainWindow.setWindowTitle(title);
    mainWindow.show();

    return app.exec();
}
