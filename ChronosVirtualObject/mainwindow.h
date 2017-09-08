#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "chronos_utility.h"
#include "chronos.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:
    void handle_osem(chronos_osem data);

private slots:

    void on_updateButton_clicked();
    void updateLabelOSEM(chronos_osem msg);

private:
    Ui::MainWindow *ui;
    Chronos *chronos;
};




#endif // MAINWINDOW_H
