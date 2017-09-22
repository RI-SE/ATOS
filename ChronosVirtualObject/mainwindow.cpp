#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    chronos_osem start_state;

    start_state.lat = 57.71495867;
    start_state.lon = 12.89134921;
    start_state.alt = 219.0;
    start_state.heading = 0;

    handleNewOSEM(start_state);

    //Chronos *chronos = new Chronos();


    //PacketInterface mPacketInt;
    //MapWidget *map = ui->widget;
    /* Listen for Chronos OSEM signal */
    //connect(chronos,SIGNAL(handle_osem(chronos_osem)),
    //        this,SLOT(updateLabelOSEM(chronos_osem)));
    //connect(chronos, SIGNAL(handle_osem(chronos_osem)),
    //        map,SLOT(chronosOSEM(chronos_osem)));
    //connect(chronos,SIGNAL(handle_dopm(QVector<chronos_dopm_pt>)),
    //       map,SLOT(chronosDOPM(QVector<chronos_dopm_pt>)));
    /* Start the chronos object */
    //chronos->startServer(53240, 53241);


}

MainWindow::~MainWindow()
{
    //delete chronos;
    delete ui;
}
/*
void MainWindow::on_updateButton_clicked(){
    //ui->label->setText(QString("apa\t Bepa"));
    OSEM_DATA temp;

    temp.latitude = 4.56789f;//(float)(rand() % 100) / 100.0f;
    temp.longitude= 5.1f;
    temp.altitude = 5.3f;
    temp.heading = 1;

    char c_lat[LABEL_TEXT_LENGTH];
    char c_long[LABEL_TEXT_LENGTH];
    char c_alt[LABEL_TEXT_LENGTH];
    char c_head[LABEL_TEXT_LENGTH];

    MapWidget* map = ui->widget;

    temp.latitude   = map->getRefLat();
    temp.longitude  = map->getRefLon();
    temp.altitude   = map->getRefAlt();
    temp.heading    = 0;

    snprintf(c_lat,LABEL_TEXT_LENGTH,"%g",temp.latitude);
    snprintf(c_long,LABEL_TEXT_LENGTH,"%g",temp.longitude);
    snprintf(c_alt,LABEL_TEXT_LENGTH,"%g",temp.altitude);
    snprintf(c_head,LABEL_TEXT_LENGTH,"%d",temp.heading);

    ui->lab_lat->setText(c_lat);
    ui->lab_lon->setText(c_long);
    ui->lab_alt->setText(c_alt);
    ui->lab_head->setText(c_head);
}*/
/*
void MainWindow::on_playButton_clicked(){




    // Disable the starting of a virtual object until it is done
    ui->playButton->setEnabled(false);
    ui->delete_vobj->setEnabled(false);

    // Reset trace
    ui->widget->clearTrace();
    // Add red trace to the carl
    //ui->widget->setTraceCar(0);

    // Start the thread with the highest priority
    vobj->start(QThread::TimeCriticalPriority);

    //delete vobj;
}*/

void MainWindow::on_init_vobj_clicked()
{
    //ui->playButton->setEnabled(true);
    ui->delete_vobj->setEnabled(true);
    ui->init_vobj->setEnabled(false);
    MapWidget *map = ui->widget;


    qint8 ID = 0;
    // Create virtual object as a new Thread
    vobj = new VirtualObject(ID,
                             map->getRefLat(),
                             map->getRefLon(),
                             map->getRefAlt());
    vobj->connectToServer(53240, 53241);








    // Add a car to the map
    // NOTE: the car and the thread must have the same ID
    CarInfo car;
    QList<LocPoint> points;
    car.setInfo("MASTER");
    map->removeCar(ID);
    map->addCar(car);
    map->setSelectedCar(ID);
    map->setFollowCar(ID);
    map->addInfoTrace(points);




    // Set SLOT to delete the object once it is finished running
    //connect(vobj, SIGNAL(finished()), vobj, SLOT(deleteLater()));
    // Connection to handle cleanup after a finished object
    connect(vobj, SIGNAL(finished()),this,SLOT(removeObject()));
    // Connection to do a shutdown of an object
    connect(this,SIGNAL(stop_virtual_object()),
            vobj,SLOT(stopSimulation()));
    // Connection to handle the stream of data passed from the object
    connect(vobj,SIGNAL(updated_state(VOBJ_DATA)),
            this,SLOT(handleUpdateState(VOBJ_DATA)));
    // Handles new OSEM message
    connect(vobj,SIGNAL(new_OSEM(chronos_osem)),
            this,SLOT(handleNewOSEM(chronos_osem)));
    // Connection to show any new trajectory that has been loaded to object
    connect(vobj,SIGNAL(new_trajectory(int,QVector<chronos_dopm_pt>)),
            this,SLOT(handleNewTrajectory(int, QVector<chronos_dopm_pt>)));

    // Reset trace
    ui->widget->clearTrace();
    // Add red trace to the carl
    //ui->widget->setTraceCar(0);

    // Start the thread with the highest priority
    vobj->start(QThread::TimeCriticalPriority);


}
void MainWindow::on_delete_vobj_clicked()
{
    //ui->playButton->setEnabled(false);
    ui->delete_vobj->setEnabled(false);
    //ui->init_vobj->setEnabled(true);



    emit stop_virtual_object();

    //delete vobj;
}

void MainWindow::updateLabelOSEM(double lat, double lon, double alt) {
    //on_updateButton_clicked();

    char c_lat[LABEL_TEXT_LENGTH];
    char c_long[LABEL_TEXT_LENGTH];
    char c_alt[LABEL_TEXT_LENGTH];
    //char c_head[LABEL_TEXT_LENGTH];

    snprintf(c_lat,LABEL_TEXT_LENGTH,"%g",lat);
    snprintf(c_long,LABEL_TEXT_LENGTH,"%g",lon);
    snprintf(c_alt,LABEL_TEXT_LENGTH,"%g",alt);
    //snprintf(c_head,LABEL_TEXT_LENGTH,"%g",heading);

    ui->lab_lat->setText(c_lat);
    ui->lab_lon->setText(c_long);
    ui->lab_alt->setText(c_alt);
    //ui->lab_head->setText(c_head);
}

void MainWindow::removeObject(){
    //ui->playButton->setEnabled(true);
    //ui->delete_vobj->setEnabled(true);

    MapWidget *map = ui->widget;

    map->clearTrace();
    int id = vobj->getID();
    map->removeCar(id);
    map->removeInfoTrace(id);
    map->update();

    delete vobj;

    ui->init_vobj->setEnabled(true);


}

void MainWindow::displayTime(qint64 t){
    // Display the time sent from the object
    char buffer[20];
    double d_t = (double) t/1000.0f;
    snprintf(buffer,20,"%g",d_t);
    ui->lab_runtime->setText(buffer);

}

void MainWindow::handleUpdateState(VOBJ_DATA data){
    MapWidget *map = ui->widget;

    // Display time in the label
    displayTime(data.time);



    // Update the car position
    LocPoint pos;

    pos.setTime(data.time);
    pos.setXY(data.x,data.y);

    double heading = (data.heading-90.0)*M_PI/180;
    pos.setAlpha(heading);

    QString sStatus = "STATUS: ";
    pos.setInfo(sStatus + QString::number(data.status));

    map->updateCarState(data.ID,pos);
    map->update();
}

void MainWindow::handleNewOSEM(chronos_osem msg)
{
    // Show the server reference values
    updateLabelOSEM(msg.lat,msg.lon,msg.alt);
    // Update the map with the reference values
    ui->widget->setEnuRef(msg.lat,msg.lon,msg.alt);
}

void MainWindow::handleNewTrajectory(int ID, QVector<chronos_dopm_pt> traj)
{
    QList<LocPoint> points;
    for(chronos_dopm_pt pt : traj)
    {
        LocPoint point(pt.x,
                       pt.y,
                       0,
                       pt.speed,
                       1,
                       0,
                       Qt::cyan,
                       pt.tRel);
        points.append(point);
    }
    //ui->widget->addInfoTrace(points);
    ui->widget->setInfoTrace(ID,points);
}
