#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "chronos_utility.h"
#include "chronos.h"
#include "virtualobject.h"
#include <QListWidget>

namespace Ui {
class MainWindow;
}

class ObjectListWidget : public QListWidgetItem
{

public:
    ObjectListWidget(int i = 0){
        id =  i;
        setText("Car " + QString::number(i));
    }
    ~ObjectListWidget(){}

    int getID(){
        return id;
    }
private:
    int id;

};

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
    void removeObject(int ID);
    void handleUpdateState(VOBJ_DATA);
    void handleNewOSEM(chronos_osem);
    void handleNewTrajectory(int ID,QVector<chronos_dopm_pt> traj);


private:
    Ui::MainWindow *ui;
    //Chronos *chronos;
    //VirtualObject* vobj;
    QVector<VirtualObject*> vobjs;

    void startObject(int ID, int udpSocket, int tcpSocket);
    void displayTime(qint64 t);
    void updateLabelOSEM(double lat,double lon,double alt);
};






#endif // MAINWINDOW_H
