#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
//#include "chronos_utility.h"
#include "chronos.h"
#include "virtualobject.h"
#include <QListWidget>
#include "utility.h"

namespace Ui {
class MainWindow;
}

class ObjectListWidget : public QListWidgetItem
{

public:
    ObjectListWidget(int i = 0, int delayFactor = 0){
        id =  i;
        traj_sim_delay_factor = delayFactor;
        setText("Car " + QString::number(i));
    }
    ~ObjectListWidget(){
        delete obj_data;
        delete obj_time;
    }

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

    bool isTrajSimEnabled()
    {
        return trajSimEnabled;
    }
    void setTrajSimEnabled(bool isEnabled)
    {
        trajSimEnabled = isEnabled;
    }

    double getStddev()
    {
        return noise_stddev;
    }
    void setStddev(double var)
    {
        noise_stddev = var;
    }

    void setTrajSimDelayFactor(int factor)
    {
        traj_sim_delay_factor  = factor;
    }
    int getTrajSimDelayFactor()
    {
       return traj_sim_delay_factor;
    }
    void setNoiseEnabled(bool enable)
    {
        noiseEnabled = enable;
    }
    bool isNoiseEnabled()
    {
        return noiseEnabled;
    }

    void addItem(double time,double data)
    {
        obj_time->append(time);
        obj_data->append(data);
    }
    void clearData()
    {
        obj_data->clear();
        obj_time->clear();
    }

    QVector<double>* getData()
    {
        return obj_data;
    }
    QVector<double>* getTime()
    {
        return obj_time;
    }

    bool getTCMActive()
    {
        return isTCMActive;
    }
    void setTCMActive(bool active)
    {
        isTCMActive = active;
    }
private:
    int id;
    bool enableMONR = true;
    bool noiseEnabled = false;
    bool trajSimEnabled = false;
    bool isTCMActive = false;
    double noise_mean = 0.0;
    double noise_stddev = 0.0;
    int traj_sim_delay_factor = 6;

    QVector<double> *obj_data = new QVector<double>();
    QVector<double> *obj_time = new QVector<double>();

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
    void measurement_noise_toggle(int,bool,double);
    void traj_sim_delay_toggle(int,bool,double);
    void trigger_occured(int);
    void send_trajectory(int,QVector<chronos_dopm_pt>);
private slots:

    void on_init_vobj_clicked();
    void on_delete_vobj_clicked();
    void on_triggerButton_clicked();
    void on_loadtraj_clicked();

    void removeObject(int ID);
    void handleUpdateState(VOBJ_DATA);
    void handleUpdateMTSP(int ID, qint64 sim_time,qint64 mtsp);
    void handleUpdateTCM(int ID, bool active);
    void handleNewOrigin(double,double,double);
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
    void handleStddevChanged();
    void handleTrajSimToggled(bool);
    void handleDelayTimeSliderChanged();
    void renderWindow();




private:
    Ui::MainWindow *ui;
    //Chronos *chronos;
    //VirtualObject* vobj;
    QVector<VirtualObject*> vobjs;
    QTimer *render_timer;
    QTimer *simulation_timer;

    QVector<double> *time_vector;

    qint64 simulation_start_time = 0;
    QString currentStddev = "0.0";
    int defaultTrajSimDelayValue;

    int running_processes = 0;


    VirtualObject* findVirtualObject(int ID);

    void startObject(int ID, int udpSocket, int tcpSocket);
    void displayTime(qint64 t);
    void updateLabelOSEM(double lat,double lon,double alt);
    bool loadTrajectoryFromFile(QString filepath);
};





#endif // MAINWINDOW_H
