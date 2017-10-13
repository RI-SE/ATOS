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
    bool isEnableMONR()
    {
        return enableMONR;
    }
    void setEnableMONR(bool enable)
    {
        enableMONR = enable;
    }

    double getStddev()
    {
        return noise_stddev;
    }
    void setStddev(double var)
    {
        noise_stddev = var;
    }
    void setNoiseEnabled(bool enable)
    {
        noiseEnabled = enable;
    }
    bool isNoiseEnabled()
    {
        return noiseEnabled;
    }
private:
    int id;
    bool enableMONR = true;
    bool noiseEnabled = false;
    double noise_mean = 0.0;
    double noise_stddev = 0.0;

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
    void enableMONRchanged(int,bool);
    void measurement_noise_toggle(bool,double);
private slots:

    void on_init_vobj_clicked();
    void on_delete_vobj_clicked();
    void removeObject(int ID);
    void handleUpdateState(VOBJ_DATA);
    void handleNewOSEM(chronos_osem);
    void handleNewTrajectory(int ID,QVector<chronos_dopm_pt> traj);

    // Slots for simulation time
    void handleSimulationStart(int ID);
    void handleSimulationStop(int ID);
    // update the simulation time label
    void updateTime();

    // QWidgetList specific slots
    void selectedCarChanged();


    // Checkbox slots
    void handleFollowCarToggled(bool);
    void handleMONREnableToggled(bool);
    void handleMeasurementNoiseToggled(bool);
    void handleVarianceChanged();
    void renderWindow();




private:
    Ui::MainWindow *ui;
    //Chronos *chronos;
    //VirtualObject* vobj;
    QVector<VirtualObject*> vobjs;
    QTimer *render_timer;
    QTimer *simulation_timer;

    qint64 simulation_start_time = 0;
    QString currentVariance = "0.0";

    int running_processes = 0;


    VirtualObject* findVirtualObject(int ID);

    void startObject(int ID, int udpSocket, int tcpSocket);
    void displayTime(qint64 t);
    void updateLabelOSEM(double lat,double lon,double alt);
};






#endif // MAINWINDOW_H
