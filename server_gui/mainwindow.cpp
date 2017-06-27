/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <cmath>
#include <QMessageBox>
#include <QFileDialog>
#include <QHostInfo>

#include "utility.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qRegisterMetaType<LocPoint>("LocPoint");


    mKeyUp = false;
    mKeyDown = false;
    mKeyLeft = false;
    mKeyRight = false;

    mSocket = NULL;
    m_boArmed = false;

    qApp->installEventFilter(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::eventFilter(QObject *object, QEvent *e)
{
    Q_UNUSED(object);

    if (e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        bool isPress = e->type() == QEvent::KeyPress;

        switch(keyEvent->key()) {
        case Qt::Key_Up:
        case Qt::Key_Down:
        case Qt::Key_Left:
        case Qt::Key_Right:
            break;

        default:
            return false;
        }

        switch(keyEvent->key()) {
        case Qt::Key_Up: mKeyUp = isPress; break;
        case Qt::Key_Down: mKeyDown = isPress; break;
        case Qt::Key_Left: mKeyLeft = isPress; break;
        case Qt::Key_Right: mKeyRight = isPress; break;

        default:
            break;
        }

        // Return true to not pass the key event on
        return true;
    }

    return false;
}


void MainWindow::on_connectButton_clicked()
{
    if(mSocket == NULL)
    {
        mSocket = new QTcpSocket(this);
        mSocket->connectToHost(ui->ipLineEdit->text(),54241);

        if(mSocket->waitForConnected(3000))
        {
           qDebug() << "Connected to server";
           ui->connectButton->setText("Disconnect");
        }
        else
        {
            qDebug() << "Not able to connect";
            ui->connectButton->setText("Connect");
        }
    }
    else
    {
        mSocket->close();
        mSocket = NULL;
        ui->connectButton->setText("Connect");
    }
}

void MainWindow::on_armButton_clicked()
{
    if(mSocket != NULL)
    {
       qDebug() << "Connected to server";

       if(m_boArmed == false)
       {
            mSocket->write("arm");
            ui->armButton->setText("Disarm");
            //mSocket->waitForBytesWritten(3000);
            qDebug() << "Sent: arm";
           m_boArmed = true;
       }
       else
       {
           m_boArmed = false;
           ui->armButton->setText("Arm");
           mSocket->write("disarm");
           qDebug() << "Sent: disarm";
       }
    }
    else
    {
        qDebug() << "No connection";
    }
}

void MainWindow::on_startButton_clicked()
{

     QString qsCommand = "start " + ui->startTimeLineEdit->text();

     if(mSocket != NULL)
     {
        qDebug() << "Connected to server";

        mSocket->write(qsCommand.toLatin1().data());
        //mSocket->waitForBytesWritten(3000);
        qDebug() << "Sent: " << qsCommand;
     }
     else
     {
         qDebug() << "No connection";
     }
}

void MainWindow::on_exitButton_clicked()
{
    if(mSocket != NULL)
    {
       qDebug() << "Connected to server";

       mSocket->write("exit");
       //mSocket->waitForBytesWritten(3000);
       qDebug() << "Sent: exit";
       m_boArmed = false;
       ui->armButton->setText("Arm");
    }
    else
    {
        qDebug() << "No connection";
    }
}

void MainWindow::on_abortButton_clicked()
{
    if(mSocket != NULL)
    {
       qDebug() << "Connected to server";

       mSocket->write("abort");
       //mSocket->waitForBytesWritten(3000);
       qDebug() << "Sent: abort";
    }
    else
    {
        qDebug() << "No connection";
    }
}
