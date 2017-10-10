#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    chronos_osem start_state;

    // Set start reference position
    start_state.lat = 57.71495867;
    start_state.lon = 12.89134921;
    start_state.alt = 219.0;
    start_state.heading = 0;

    handleNewOSEM(start_state);

    QListWidget *lwid = ui->carListWidget;

    render_timer = new QTimer(this);


    connect(lwid,SIGNAL(itemSelectionChanged()),
            this,SLOT(selectedCarChanged()));
    connect(render_timer,SIGNAL(timeout()),
            this, SLOT(renderWindow()));

    // Set the render time to 20ms
    render_timer->start(20);

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
    render_timer->stop();
    delete render_timer;
    delete ui;
}


void MainWindow::on_init_vobj_clicked()
{
    //ui->playButton->setEnabled(true);
    ui->delete_vobj->setEnabled(true);
    ui->init_vobj->setEnabled(false);

    int obj_nr = ui->spinBox->value();

    for(int i = 0; i<obj_nr;i++)
    {
        startObject(i,53240+2*i,53241+2*i);
        //ui->carListWidget->addItem("Car " + QString::number(ID));
        ObjectListWidget *item = new ObjectListWidget(i);
        ui->carListWidget->addItem(item);
        if (i==0) ui->carListWidget->item(i)->setSelected(true);


    }
    //ui->carListWidget->SelectColumns

    //startObject(0,53240,53241);
    //startObject(1,53242,53243);

}
void MainWindow::on_delete_vobj_clicked()
{
    //ui->playButton->setEnabled(false);
    ui->delete_vobj->setEnabled(false);
    //ui->init_vobj->setEnabled(true);

    ui->carListWidget->clear();

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

void MainWindow::startObject(int ID, int udpSocket, int tcpSocket){



    // Do Map-related stuff
    /*-------------------------------------------*/
    MapWidget *map = ui->widget;


    //qint8 ID = 0;
    // Create virtual object as a new Thread
    VirtualObject *vobj = new VirtualObject(ID,
                             map->getRefLat(),
                             map->getRefLon(),
                             map->getRefAlt());
    //vobj->connectToServer(53240, 53241);
    vobj->connectToServer(udpSocket,tcpSocket);






    // Add a car to the map
    // NOTE: the car and the thread must have the same ID
    CarInfo car(ID,Qt::red);
    QList<LocPoint> points;
    car.setInfo("MASTER");
    if (map->removeCar(ID)) qDebug() << "Car removed";
    map->addCar(car);
    //map->setSelectedCar(ID);
    //map->setFollowCar(ID);
    map->addInfoTrace(ID,points);




    // Set SLOT to delete the object once it is finished running
    //connect(vobj, SIGNAL(finished()), vobj, SLOT(deleteLater()));
    // Connection to handle cleanup after a finished object
    //connect(vobj, SIGNAL(finished()),this,SLOT(removeObject()));
    connect(vobj,SIGNAL(thread_done(int)),this,SLOT(removeObject(int)));
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
    //vobj->start();
    vobjs.append(vobj);

}
// Remove a car object based on ID
void MainWindow::removeObject(int ID){


    MapWidget *map = ui->widget;


    if (ID <0)
    {
        qDebug() << "Object does not exist.";
        return;
    }
    map->removeCar(ID);
    if (map->removeInfoTrace(ID)){
        qDebug() << "Failed to remove info Trace.";
    }

    int del_idx = 0;
    bool isFound = false;
    for (del_idx = 0;del_idx<vobjs.size();del_idx++)
    {
        if (vobjs[del_idx]->getID() == ID)
        {
            isFound = true;
            break;
        }
    }
    if (isFound)
    {
        delete vobjs[del_idx];
        vobjs.remove(del_idx);
    }
    map->update();


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

    // Calculate and set the heading in the map
    double heading = (data.heading-90.0)*M_PI/180;
    pos.setAlpha(heading);

    // Set Master or slave status on the car
    QString sStatus = "STATUS: ";
    QString status_text;
    if (data.isMaster){
        status_text = "MASTER";
    }
    else {
        status_text = "SLAVE";
    }
    pos.setInfo(sStatus + QString::number(data.status) + "\n"+ status_text);

    // Update the car on the map
    map->updateCarState(data.ID,pos);
    //map->update();
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
    ui->widget->addInfoTrace(ID,points);
    //ui->widget->setInfoTrace(ID,points);
}

void MainWindow::selectedCarChanged()
{
    QList<QListWidgetItem*> items = ui->carListWidget->selectedItems();
    if (items.size()==0)
    {
        qDebug() << "No item selected";
        return;
    }
    else if (items.size()>1)
    {
        qDebug() << "To many selected items.";
        return;
    }
    ObjectListWidget *item = (ObjectListWidget*) items[0];

    ui->widget->setSelectedCar(item->getID());

}

void MainWindow::renderWindow()
{
    ui->widget->update();
}
