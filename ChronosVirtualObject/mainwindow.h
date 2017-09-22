#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "chronos_utility.h"
#include "chronos.h"
#include "virtualobject.h"

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
    //void handle_osem(chronos_osem data);
    void stop_virtual_object();
private slots:

    //void on_updateButton_clicked();
    //void on_playButton_clicked();
    void on_init_vobj_clicked();
    void on_delete_vobj_clicked();
    //void updateLabelOSEM(chronos_osem msg);
    void removeObject();
    void handleUpdateState(VOBJ_DATA);
    void handleNewTrajectory(QVector<chronos_dopm_pt> traj);


private:
    Ui::MainWindow *ui;
    //Chronos *chronos;
    VirtualObject* vobj;

    void displayTime(qint64 t);
    void updateLabelOSEM(double lat,double lon,double alt);
};






#endif // MAINWINDOW_H
