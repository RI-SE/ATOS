#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    Chronos *chronos = new Chronos();


    //PacketInterface mPacketInt;
    MapWidget *map = ui->widget;
    /* Listen for Chronos OSEM signal */
    connect(chronos,SIGNAL(handle_osem(chronos_osem)),
            this,SLOT(updateLabelOSEM(chronos_osem)));
    connect(chronos, SIGNAL(handle_osem(chronos_osem)),
            map,SLOT(chronosOSEM(chronos_osem)));
    connect(chronos,SIGNAL(handle_dopm(QVector<chronos_dopm_pt>)),
            map,SLOT(chronosDOPM(QVector<chronos_dopm_pt>)));
    /* Start the chronos object */
    chronos->startServer();


}

MainWindow::~MainWindow()
{
    delete chronos;
    delete ui;
}

void MainWindow::on_updateButton_clicked(){
    //ui->label->setText(QString("apa\t Bepa"));
    OSEM_DATA temp;
    /*
    temp.latitude = 4.56789f;//(float)(rand() % 100) / 100.0f;
    temp.longitude= 5.1f;
    temp.altitude = 5.3f;
    temp.heading = 1;
*/
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
}

void MainWindow::on_playButton_clicked(){

    VirtualObject *vobj = new VirtualObject();
    connect(vobj, SIGNAL(finished()), vobj, SLOT(deleteLater()));
    connect(vobj, SIGNAL(finished()),this,SLOT(unlockRun()));
    connect(vobj, SIGNAL(updated_position(double,double,long)),
            this,SLOT(displayTime(double,double,long)));
    // Start the thread with the highest priority
    vobj->start(QThread::TimeCriticalPriority);
    ui->playButton->setEnabled(false);
    //delete vobj;
}

void MainWindow::updateLabelOSEM(chronos_osem msg) {
    //on_updateButton_clicked();

    char c_lat[LABEL_TEXT_LENGTH];
    char c_long[LABEL_TEXT_LENGTH];
    char c_alt[LABEL_TEXT_LENGTH];
    char c_head[LABEL_TEXT_LENGTH];

    snprintf(c_lat,LABEL_TEXT_LENGTH,"%g",msg.lat);
    snprintf(c_long,LABEL_TEXT_LENGTH,"%g",msg.lon);
    snprintf(c_alt,LABEL_TEXT_LENGTH,"%g",msg.alt);
    snprintf(c_head,LABEL_TEXT_LENGTH,"%g",msg.heading);

    ui->lab_lat->setText(c_lat);
    ui->lab_lon->setText(c_long);
    ui->lab_alt->setText(c_alt);
    ui->lab_head->setText(c_head);
}

void MainWindow::unlockRun(){
    ui->playButton->setEnabled(true);
}

void MainWindow::displayTime(double x, double y, long t){
    char buffer[20];

    double d_t = (double) t/1000.0f;

    snprintf(buffer,20,"%g",d_t);

    ui->lab_runtime->setText(buffer);
}
