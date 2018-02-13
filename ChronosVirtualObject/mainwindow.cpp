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
    handleNewOrigin(start_state.lat,start_state.lon,start_state.alt);

    QListWidget *lwid = ui->carListWidget;
    QSlider * slider = ui->delayTimeSlider;

    MainWindow::defaultTrajSimDelayValue = slider->minimum() ;//+ (slider->maximum()-slider->minimum()) / 2;
    slider->setValue(MainWindow::defaultTrajSimDelayValue);


    render_timer = new QTimer(this);
    simulation_timer = new QTimer(this);


    connect(lwid,SIGNAL(itemSelectionChanged()),
            this,SLOT(selectedCarChanged()));
    connect(render_timer,SIGNAL(timeout()),
            this, SLOT(renderWindow()));
    connect(simulation_timer,SIGNAL(timeout()),
            this,SLOT(updateTime()));
    connect(ui->followCarBox,SIGNAL(toggled(bool)),
            this,SLOT(handleFollowCarToggled(bool)));
    connect(ui->MONR_enable,SIGNAL(toggled(bool)),
            this,SLOT(handleMONREnableToggled(bool)));
    connect(ui->measurementNoiseEnable,SIGNAL(toggled(bool)),
            this,SLOT(handleMeasurementNoiseToggled(bool)));
    connect(ui->varianceEdit,SIGNAL(editingFinished()),
            this,SLOT(handleStddevChanged()));
    connect(ui->trajSimBox,SIGNAL(toggled(bool)),
            this,SLOT(handleTrajSimToggled(bool)));
    connect(ui->delayTimeSlider,SIGNAL(valueChanged(int)),
            this,SLOT(handleDelayTimeSliderChanged()));

    // Set the render time to 500ms and start it
    render_timer->start(500);
}

MainWindow::~MainWindow()
{
    render_timer->stop();
    simulation_timer->stop();
    delete simulation_timer;
    delete render_timer;
    delete ui;
}


void MainWindow::on_init_vobj_clicked()
{

    QCustomPlot *plot = ui->plot1;
    QPen pen;

    // Read the value from the spinbox and create that many objects
    int obj_nr = ui->spinBox->value();
    for(int i = 0; i<obj_nr;i++)
    {
        // Create and start the virtual object
        startObject(i,53240+2*i,53241+2*i);
        // Create corresponding List Widged item to control the different objects
        ObjectListWidget *item = new ObjectListWidget(i,defaultTrajSimDelayValue);
        // Add the created control to the list
        ui->carListWidget->addItem(item);
        // Set the default selected car
        if (i==0) ui->carListWidget->item(i)->setSelected(true);

        // Add corresponding plot
        plot->addGraph();
        pen.setColor(QColor(qSin(i*0.3)*100+100, qSin(i*1.6+0.7)*100+100, qSin(i*0.4+0.6)*100+100));
        plot->graph(i)->setPen(pen);
        plot->graph(i)->setName("Car " + QString::number(i));
    }

    // Configure the plot
    plot->xAxis->setLabel("t : time");
    plot->yAxis->setLabel("MTSP");
    plot->legend->setVisible(true);


    // Enable the checkboxes
    ui->delete_vobj->setEnabled(true);
    ui->init_vobj->setEnabled(false);
    ui->followCarBox->setEnabled(true);
    ui->MONR_enable->setEnabled(true);
    ui->trajSimBox->setEnabled(true);
    ui->measurementNoiseEnable->setEnabled(true);
    ui->loadtraj->setEnabled(true);

}
void MainWindow::on_delete_vobj_clicked()
{
    // Stop the simulation timer
    simulation_timer->stop();

    // Disable buttons
    ui->delete_vobj->setEnabled(false);
    ui->triggerButton->setEnabled(false);

    // Enable the init button
    ui->init_vobj->setEnabled(true);

    // Reset the checkbox states
    ui->followCarBox->setChecked(true);
    ui->MONR_enable->setChecked(true);
    ui->measurementNoiseEnable->setChecked(false);
    ui->trajSimBox->setChecked(false);

    // Disable the checkboxes
    ui->followCarBox->setEnabled(false);
    ui->MONR_enable->setEnabled(false);
    ui->measurementNoiseEnable->setEnabled(false);
    ui->trajSimBox->setEnabled(false);
    ui->loadtraj->setEnabled(false);



    // Clear the listbox
    ui->carListWidget->clear();

    // Do they do anything?
    ui->plot1->clearGraphs();
    ui->plot1->legend->setVisible(false);

    // Tell all virtual objects to stop running
    emit stop_virtual_object();

}

void MainWindow::on_triggerButton_clicked()
{
    // Find the item that is selected
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
    emit trigger_occured(item->getID());
}

void MainWindow::on_loadtraj_clicked()
{
    //qDebug() << "Load trajectory clicked!";
    QString file = QFileDialog::getOpenFileName(
                this,
                "Select one or more files to open.",
                "./"
                );
    loadTrajectoryFromFile(file);

}

// Update the label containing the OSEM information
void MainWindow::updateLabelOSEM(double lat, double lon, double alt) {

    char c_lat[10];
    char c_long[10];
    char c_alt[10];

    snprintf(c_lat,10,"%g",lat);
    snprintf(c_long,10,"%g",lon);
    snprintf(c_alt,10,"%g",alt);

    ui->lab_lat->setText(c_lat);
    ui->lab_lon->setText(c_long);
    ui->lab_alt->setText(c_alt);
}

// Creates and starts a single object.
void MainWindow::startObject(int ID, int udpSocket, int tcpSocket){


    MapWidget *map = ui->widget;

    // Create virtual object as a new Thread
    VirtualObject *vobj = new VirtualObject(ID,
                             map->getRefLat(),
                             map->getRefLon(),
                             map->getRefAlt());
    vobj->connectToServer(udpSocket,tcpSocket);

    // Add a car to the map
    // NOTE: the car and the thread must have the same ID
    CarInfo car(ID,Qt::red);
    QList<LocPoint> points;
    car.setInfo("MASTER");
    if (map->removeCar(ID)) qDebug() << "Car removed";
    map->addCar(car);
    map->addInfoTrace(ID,points);



    // Make connections according to: Virtual object -> UI

    // Set SLOT to delete the object once it is finished running
    connect(vobj,SIGNAL(thread_done(int)),this,SLOT(removeObject(int)));
    // Connection to do a shutdown of an object
    connect(this,SIGNAL(stop_virtual_object()),
            vobj,SLOT(stopSimulation()));
    // Connection to handle the stream of data passed from the object
    connect(vobj,SIGNAL(updated_state(VOBJ_DATA)),
            this,SLOT(handleUpdateState(VOBJ_DATA)));
    // Handles new OSEM message
    connect(vobj,SIGNAL(new_origin(double,double,double)),
            this,SLOT(handleNewOrigin(double,double,double)));
    // Connection to show any new trajectory that has been loaded to object
    connect(vobj,SIGNAL(new_trajectory(int,QVector<chronos_dopm_pt>)),
            this,SLOT(handleNewTrajectory(int, QVector<chronos_dopm_pt>)));
    connect(vobj,SIGNAL(simulation_start(int)),
            this,SLOT(handleSimulationStart(int)));
    connect(vobj,SIGNAL(simulation_stop(int)),
            this,SLOT(handleSimulationStop(int)));
    connect(vobj,SIGNAL(forward_mtsp(int,qint64,qint64)),
            this,SLOT(handleUpdateMTSP(int,qint64,qint64)));
    connect(vobj,SIGNAL(forward_tcm(int,bool)),
            this,SLOT(handleUpdateTCM(int,bool)));


    // Make connections according to: UI -> Virtual object

    connect(this,SIGNAL(enableMONRchanged(int,bool)),
            vobj,SLOT(MONREnabledChanged(int,bool)));
    connect(this,SIGNAL(measurement_noise_toggle(int,bool,double)),
            vobj,SLOT(handleMeasurementNoiseToggle(int,bool,double)));
    connect(this,SIGNAL(traj_sim_delay_toggle(int,bool,double)),
            vobj,SLOT(handleTrajSimDelayToggle(int,bool,double)));
    connect(this,SIGNAL(trigger_occured(int)),
            vobj,SLOT(triggerOccured(int)));
    connect(this,SIGNAL(send_trajectory(int,QVector<chronos_dopm_pt>)),
            vobj,SLOT(handleLoadedDOPM(int,QVector<chronos_dopm_pt>)));
    // Reset trace
    ui->widget->clearTrace();

    // Start the thread with the highest priority
    vobj->start(QThread::TimeCriticalPriority);
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
    for (del_idx = 0;del_idx<vobjs.size();del_idx++)
    {
        if (vobjs[del_idx]->getID() == ID)
        {
            delete vobjs[del_idx];
            vobjs.remove(del_idx);
            break;
        }
    }
}

VirtualObject* MainWindow::findVirtualObject(int ID)
{
    for(VirtualObject* v : vobjs){
        if (v->getID() == ID) return v;
    }
    return 0;
}

// Update the time label
void MainWindow::updateTime()
{
    qint64 time = QDateTime::currentMSecsSinceEpoch() - simulation_start_time;
    double d_time = (double) time/1000.0;
    ui->lab_runtime->setText(QString::number(d_time));
}

void MainWindow::handleUpdateState(VOBJ_DATA data){

    /* Update the state of the car on the map */
    // Update the car position
    LocPoint pos;
    pos.setTime(data.time);
    pos.setXY(data.x,data.y);

    // Calculate and set the heading in the map
    double heading = (data.heading-90.0)*M_PI/180;
    pos.setAlpha(heading);

    // Set Master or Slave status on the car
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
    ui->widget->updateCarState(data.ID,pos);
}

void MainWindow::handleNewOrigin(double lat, double lon, double alt)
{
    // Show the server reference values
    updateLabelOSEM(lat,lon,alt);
    // Update the map with the reference values
    ui->widget->setEnuRef(lat,lon,alt);
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
}

void MainWindow::handleSimulationStart(int ID)
{
    (void)ID;

    // Clear the data in the plot
    QListWidget *temp = ui->carListWidget;
    ObjectListWidget *obj = (ObjectListWidget*) temp->item(ID);
    obj->clearData();

    // Keep track of the number of processes
    running_processes++;
    qDebug() << "Number of Processes = " << QString::number(running_processes);

    if (!simulation_timer->isActive())
    {
        // Start the simulation
        simulation_timer->start(10);
        // Speed up the rendering when objects are running
        render_timer->setInterval(50);
        simulation_start_time = QDateTime::currentMSecsSinceEpoch();
    }
}

void MainWindow::handleSimulationStop(int ID)
{
    (void)ID;

    --running_processes;
    qDebug() << "Number of Processes = " << QString::number(running_processes);

    // Don't stop simulation until all processes have stoped
    if (running_processes == 0)
    {
        simulation_timer->stop();
        // Slow down the rendering when no objects are running
        render_timer->setInterval(500);
    }

}

void MainWindow::handleUpdateMTSP(int ID, qint64 sim_time,qint64 mtsp)
{
    /* Add points for plotting */
    QListWidget *temp = ui->carListWidget;
    for (int i = 0; i < temp->count();i++){
        ObjectListWidget *obj = (ObjectListWidget*)temp->item(i);
        if (obj->getID() == ID)
        {
            obj->addItem(sim_time / 1000.0,mtsp);
            break;
        }
    }
}

void MainWindow::handleUpdateTCM(int ID, bool active)
{
    /* Add points for plotting */
    QListWidget *temp = ui->carListWidget;
    for (int i = 0; i < temp->count();i++){
        ObjectListWidget *obj = (ObjectListWidget*)temp->item(i);
        if (obj->getID() == ID)
        {
            obj->setTCMActive(active);
            qDebug() << "Updated item " << QString::number(ID);
            break;
        }
    }

    // Find the item that is selected
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
    if (item->getID() == ID)
    {
        ui->triggerButton->setEnabled(active);
    }
}

void MainWindow::selectedCarChanged()
{

    // Find the item that is selected
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


    int iID = item->getID();

    // Change the seleced car in the map
    ui->widget->setSelectedCar(iID);


    if (ui->followCarBox->isChecked())
    {
        // Set the car to follow
        ui->widget->setFollowCar(iID);
    }
    else
    {
        // Do not follow any car
        ui->widget->setFollowCar(-1);
    }
    // Save the new stddev value
    currentStddev = QString::number(item->getStddev(),'g',4);
    // Set the label with the new value
    ui->varianceEdit->setText(currentStddev);
    // Set the status of the checkboxes
    ui->MONR_enable->setChecked(item->isEnableMONR());
    ui->measurementNoiseEnable->setChecked(item->isNoiseEnabled());
    ui->trajSimBox->setChecked(item->isTrajSimEnabled());

    // Enable/Disable the button
    ui->triggerButton->setEnabled(item->getTCMActive());


    // Set the slider value
    ui->delayTimeSlider->setValue(item->getTrajSimDelayFactor());

}

void MainWindow::handleFollowCarToggled(bool checked)
{
    // Find the item that is selected
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
    if (checked)
    {
        ObjectListWidget *item = (ObjectListWidget*) items[0];
        ui->widget->setFollowCar(item->getID());
    }
    else
    {
        ui->widget->setFollowCar(-1);
    }
}

void MainWindow::handleMONREnableToggled(bool checked)
{
    // Find the item that is selected
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

    // Send signal to the virtual object
    emit enableMONRchanged(item->getID(),checked);
}


void MainWindow::handleMeasurementNoiseToggled(bool checked)
{
    // Find the item that is selected
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

    // Se the item variable
    item->setNoiseEnabled(checked);

    // Enable or disable the edit of noise standard devation
    ui->varianceEdit->setEnabled(checked);
    ui->lab_var->setEnabled(checked);

    // Update the virtual object
    emit measurement_noise_toggle(item->getID(),checked,item->getStddev());
}

void MainWindow::handleStddevChanged()
{
    // Find the item that is selected
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

    // Get the Standard devation from the textbox
    QLineEdit *inputLine = ui->varianceEdit;
    QString input = inputLine->text();

    // Makes sure that the textbox entry is an acceptable input
    bool isConversionOK = false;
    double newVariance = input.toDouble(&isConversionOK);
    if (isConversionOK)
    {
        currentStddev = QString::number(newVariance,'g',4);
        item->setStddev(newVariance);
        emit measurement_noise_toggle(item->getID(),item->isNoiseEnabled(),item->getStddev());
    }
    // Set the textbox to either the previous value or the new value
    inputLine->setText(currentStddev);
    // Remove focus from the textbox
    ui->varianceEdit->clearFocus();

}
// To handle exact execution of a trajectory file, i.e. not interpolated
void MainWindow::handleTrajSimToggled(bool checked)
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


    item->setTrajSimEnabled(checked);
    ui->delayTimeSlider->setEnabled(checked);
    emit traj_sim_delay_toggle(item->getID(),item->isTrajSimEnabled(),
                               item->getTrajSimDelayFactor()-1);
}

void MainWindow::handleDelayTimeSliderChanged()
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
    int slide_value = ui->delayTimeSlider->value();
    item->setTrajSimDelayFactor(slide_value);
    qDebug() << "Value Changed";
    emit traj_sim_delay_toggle(item->getID(),item->isTrajSimEnabled(),slide_value-1);

}

void MainWindow::renderWindow()
{
    // Update map
    ui->widget->update();

    // Add data point to plot
    QCustomPlot *plot = ui->plot1;
    QListWidget *temp = ui->carListWidget;
    for (int i = 0; i < temp->count();i++){
        ObjectListWidget *obj = (ObjectListWidget*)temp->item(i);
        QVector<double> *time = obj->getTime();
        QVector<double> *data = obj->getData();
        plot->graph(i)->setData(*time,*data);
    }
    // Replot the graph
    plot->rescaleAxes();
    plot->replot();
}

bool MainWindow::loadTrajectoryFromFile(QString filepath)
{




    QString line;
    QFile file(filepath);
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Could not open file: " << filepath;
    }


    // Find the item/car that is selected
    QList<QListWidgetItem*> items = ui->carListWidget->selectedItems();
    if (items.size()==0)
    {
        qDebug() << "No item selected";
        return false;
    }
    else if (items.size()>1)
    {
        qDebug() << "To many selected items.";
        return false;
    }
    double ref_llh[3];
    double llh[3] = {0,0,0};
    double xyz[3];

    ObjectListWidget *item = (ObjectListWidget*) items[0];
    VirtualObject* obj = findVirtualObject(item->getID());
    obj->getRefLLH(ref_llh[0],ref_llh[1],ref_llh[2]);


    QList<LocPoint> drawtraj;
    QVector<chronos_dopm_pt> traj;

    QTextStream in(&file);
    while(!in.atEnd())
    {
        line = in.readLine();
        QStringList list = line.split(';');
        chronos_dopm_pt point;
        if(list[0].compare("LINE") == 0)
        {

            xyz[0] = list[2].toDouble();
            xyz[1] = list[3].toDouble();
            xyz[2] = list[4].toDouble();

            llh[0] = xyz[1] / ((M_PI/180)*6378137.0) + ref_llh[0];
            llh[1] = xyz[0] / ((M_PI/180)*6378137.0*cos(((llh[0] + ref_llh[0])/2)*(M_PI/180))) + ref_llh[1];
            llh[2] = ref_llh[2] + xyz[2];

            utility::llhToEnu(ref_llh, llh, xyz);

            point.tRel = (uint32_t) (list[1].toDouble()*1000.0f);
            point.x = xyz[0];
            point.y = xyz[1];
            point.z = xyz[2];
            point.heading = (list[5].toDouble())*180/M_PI;
            point.speed = list[6].toDouble();
            point.accel = list[7].toUInt();
            point.curvature = list[8].toUInt();
            point.mode = list[9].toUInt();

            traj.append(point);
            drawtraj.append(LocPoint(xyz[0],xyz[1]));
        }

    }
    file.close();
    if(traj.size() > 0)
    {
        emit send_trajectory(obj->getID(),traj);
        return true;
    }
    return false;

}
