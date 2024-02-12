#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>
#include <unordered_map>
#include "CEYSDUIView.h"

namespace Ui {
class MainWindow;
}

class CVideoDeviceDialog;
class MainWindow : public QMainWindow,
                   public CEYSDUIView
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    virtual void paintEvent(QPaintEvent *event);

private:
    void UpdateListState();
    void UpdateLogEnable();

private slots:
    void on_open_dialog_button_clicked();

    void on_enable_sdk_log_checkBox_stateChanged(int state);

private:
    Ui::MainWindow *ui;

private:
    std::vector<CVideoDeviceDialog *> m_VideoDialogs;
    std::unordered_map<CVideoDeviceModel *, CVideoDeviceDialog *> m_dialogMap;
    bool m_bIsLogEnabled;
    bool m_bAutoPreview;
};

#endif // MAINWINDOW_H
