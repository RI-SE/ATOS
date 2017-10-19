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
            this,SLOT(handleVarianceChanged()));
    connect(ui->trajSimBox,SIGNAL(toggled(bool)),
            this,SLOT(handleTrajSimToggled(bool)));
    //connect(ui->delayTimeSlider,SIGNAL(sliderReleased()),
    //        this,SLOT(handleDelayTimeSliderChanged()));
    connect(ui->delayTimeSlider,SIGNAL(valueChanged(int)),
            this,SLOT(handleDelayTimeSliderChanged()));
    // Set the render time to 20ms
    render_timer->start(500);
}

MainWindow::~MainWindow()
{
    //delete chronos;
    render_timer->stop();
    delete simulation_timer;
    delete render_timer;
    delete ui;
}


void MainWindow::on_init_vobj_clicked()
{

    QCustomPlot *plot = ui->plot1;
    QPen pen;

    int obj_nr = ui->spinBox->value();

    for(int i = 0; i<obj_nr;i++)
    {
        startObject(i,53240+2*i,53241+2*i);
        //ui->carListWidget->addItem("Car " + QString::number(ID));
        ObjectListWidget *item = new ObjectListWidget(i,defaultTrajSimDelayValue);
        ui->carListWidget->addItem(item);
        if (i==0) ui->carListWidget->item(i)->setSelected(true);

        plot->addGraph();

        pen.setColor(QColor(qSin(i*0.3)*100+100, qSin(i*1.6+0.7)*100+100, qSin(i*0.4+0.6)*100+100));
        plot->graph(i)->setPen(pen);
        plot->graph(i)->setName("Car " + QString::number(i));
    }

    plot->xAxis->setLabel("t : time");
    plot->yAxis->setLabel("MTSP");
    plot->legend->setVisible(true);


    ui->delete_vobj->setEnabled(true);
    ui->init_vobj->setEnabled(false);
    ui->followCarBox->setEnabled(true);
    ui->MONR_enable->setEnabled(true);
    ui->trajSimBox->setEnabled(true);

    QCheckBox *mcbx = ui->measurementNoiseEnable;
    mcbx->setEnabled(true);

    // Send signal to update the variance enabled state
    emit(mcbx->toggled(mcbx->isChecked()));
    //ui->carListWidget->SelectColumns

    //startObject(0,53240,53241);
    //startObject(1,53242,53243);

}
void MainWindow::on_delete_vobj_clicked()
{

    ui->delete_vobj->setEnabled(false);


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

    //emit(ui->measurementNoiseEnable->toggled(false));

    ui->carListWidget->clear();
    ui->plot1->clearGraphs();
    ui->plot1->legend->setVisible(false);


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
    connect(vobj,SIGNAL(new_OSEM(chronos_osem)),
            this,SLOT(handleNewOSEM(chronos_osem)));
    // Connection to show any new trajectory that has been loaded to object
    connect(vobj,SIGNAL(new_trajectory(int,QVector<chronos_dopm_pt>)),
            this,SLOT(handleNewTrajectory(int, QVector<chronos_dopm_pt>)));
    connect(vobj,SIGNAL(simulation_start(int)),
            this,SLOT(handleSimulationStart(int)));
    connect(vobj,SIGNAL(simulation_stop(int)),
            this,SLOT(handleSimulationStop(int)));


    // Make connections according to: UI -> Virtual object

    connect(this,SIGNAL(enableMONRchanged(int,bool)),
            vobj,SLOT(MONREnabledChanged(int,bool)));
    connect(this,SIGNAL(measurement_noise_toggle(int,bool,double)),
            vobj,SLOT(handleMeasurementNoiseToggle(int,bool,double)));
    connect(this,SIGNAL(traj_sim_delay_toggle(int,bool,double)),
            vobj,SLOT(handleTrajSimDelayToggle(int,bool,double)));
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

VirtualObject* MainWindow::findVirtualObject(int ID)
{
    for(VirtualObject* v : vobjs){
        if (v->getID() == ID) return v;
    }
    return 0;
}
/*
void MainWindow::displayTime(qint64 t){
    // Display the time sent from the object
    char buffer[20];
    double d_t = (double) t/1000.0f;
    snprintf(buffer,20,"%g",d_t);
    ui->lab_runtime->setText(buffer);

}*/

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
    ui->widget->updateCarState(data.ID,pos);

    /* Add points for plotting */

    QCustomPlot *plot = ui->plot1;

    QCPRange range = plot->yAxis->range();
    // find the objective
    QListWidget *temp = ui->carListWidget;
    for (int i = 0; i < temp->count();i++){
        ObjectListWidget *obj = (ObjectListWidget*)temp->item(i);

        if (obj->getID() == data.ID)
        {
            //qDebug() << "Vel = " << QString::number(data.speed);
            obj->addItem(data.time / 1000.0,data.mtsp);
/*
            if (data.speed > range.upper)
            {
                plot->yAxis->setRange(range.lower,data.speed);
            }
            else if (data.speed < range.lower)
            {
                plot->yAxis->setRange(data.speed, range.upper);
            }*/
            break;
        }
    }
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

void MainWindow::handleSimulationStart(int ID)
{
    (void)ID;
    running_processes++;
    // ID implemented for future use
    if (!simulation_timer->isActive())
    {
        simulation_timer->start(10);
        render_timer->setInterval(50);
        simulation_start_time = QDateTime::currentMSecsSinceEpoch();
    }
}

void MainWindow::handleSimulationStop(int ID)
{
    (void)ID;
    if (--running_processes == 0)
    {
        simulation_timer->stop();
        render_timer->setInterval(500);
    }
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
    int iID = item->getID();
    ui->widget->setSelectedCar(iID);

    if (ui->followCarBox->isChecked())
    {
        ui->widget->setFollowCar(iID);
    }
    else
    {
        // Do not follow any car
        ui->widget->setFollowCar(-1);
    }
    // Save the new stddev value
    currentVariance = QString::number(item->getStddev(),'g',4);
    // Set the label with the new value
    ui->varianceEdit->setText(currentVariance);
    // Set the status of the checkboxes
    ui->MONR_enable->setChecked(item->isEnableMONR());
    ui->measurementNoiseEnable->setChecked(item->isNoiseEnabled());
    ui->trajSimBox->setChecked(item->isTrajSimEnabled());

    // Set the slider value
    ui->delayTimeSlider->setValue(item->getTrajSimDelayFactor());

    emit(ui->measurementNoiseEnable->toggled(item->isNoiseEnabled()));

}

void MainWindow::handleFollowCarToggled(bool checked)
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
    item->setEnableMONR(checked);

    VirtualObject *vobj = findVirtualObject(item->getID());

    if (!vobj)
    {
        qDebug() << "Could not find the given virual object.";
        return;
    }

    emit enableMONRchanged(item->getID(),checked);


}


void MainWindow::handleMeasurementNoiseToggled(bool checked)
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

    emit measurement_noise_toggle(item->getID(),checked,item->getStddev());

    item->setNoiseEnabled(checked);

    ui->varianceEdit->setEnabled(checked);
    ui->lab_var->setEnabled(checked);
}

void MainWindow::handleVarianceChanged()
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


    QLineEdit *inputLine = ui->varianceEdit;
    QString input = inputLine->text();

    bool isConversionOK = false;
    double newVariance = input.toDouble(&isConversionOK);

    if (isConversionOK)
    {
        //qDebug() << "OK Conversion";
        currentVariance = QString::number(newVariance,'g',4);
        item->setStddev(newVariance);
        emit measurement_noise_toggle(item->getID(),item->isNoiseEnabled(),item->getStddev());
    }
    inputLine->setText(currentVariance);
    ui->varianceEdit->clearFocus();

}

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
                               //1.0+(double)(slide_value-defaultTrajSimDelayValue)/50.0);

}

void MainWindow::renderWindow()
{
    ui->widget->update();

    //int64_t cols[3] = {Qt::blue, Qt::red, Qt::yellow};

    QCustomPlot *plot = ui->plot1;
    QListWidget *temp = ui->carListWidget;
    for (int i = 0; i < temp->count();i++){
        ObjectListWidget *obj = (ObjectListWidget*)temp->item(i);
        QVector<double> *time = obj->getTime();
        QVector<double> *data = obj->getData();
        plot->graph(i)->setData(*time,*data);
        //plot->graph(i)->setPen(cols[i]);
        //plot->xAxis->setRange(0,(*time).last());
        //plot->yAxis->setRange(0,0.01); // Make dynamic


    }
    plot->rescaleAxes();
    plot->replot();
}
