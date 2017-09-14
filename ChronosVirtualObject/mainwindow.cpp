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
    /* Start the chronos object */
    chronos->startServer();

}

MainWindow::~MainWindow()
{
    delete ui;
    delete chronos;
}

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

    snprintf(c_lat,LABEL_TEXT_LENGTH,"%g",temp.latitude);
    snprintf(c_long,LABEL_TEXT_LENGTH,"%g",temp.longitude);
    snprintf(c_alt,LABEL_TEXT_LENGTH,"%g",temp.altitude);
    snprintf(c_head,LABEL_TEXT_LENGTH,"%d",temp.heading);

    ui->lab_lat->setText(c_lat);
    ui->lab_lon->setText(c_long);
    ui->lab_alt->setText(c_alt);
    ui->lab_head->setText(c_head);
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
