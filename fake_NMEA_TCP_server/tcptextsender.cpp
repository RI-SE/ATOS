#include "tcptextsender.h"


TCPTextSender::TCPTextSender(QString file, QObject *parent)
{
    /*
    connect(&client, SIGNAL(connected()),this,SLOT(handleConnected()));
    connect(&client, SIGNAL(disconnected()),this,SLOT(handleDisconnected()));

    //connect(this,SIGNAL(finished()),SLOT(quitApplication()));
    connect(&client,SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(handleSocketError(QAbstractSocket::SocketError)));
    */
    connect(this,SIGNAL(sendData(QString)),this,SLOT(handleSendData(QString)));
    connect(&mTcpServer,SIGNAL(connectionChanged(bool)),this,SLOT(handleConnectionChanged(bool)));
    filepath = file;
}

TCPTextSender::~TCPTextSender(){

}

int TCPTextSender::readTextFile(QString filepath, QVector<QString> &output)
{
    QString line;
    QFile file(filepath);
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "File <" << filepath << "> could not be opened.";
        return -1;
    }
    QTextStream in(&file);
    while(!in.atEnd())
    {
        line = in.readLine() + '\0';
        qDebug() << line;
        output.append(line);
    }
    qDebug() <<  QString::number(output.size()) << " lines read.";

    return 0;
}

void TCPTextSender::connectToRec()
{
    //QHostAddress addr("127.0.0.1");
    //client.connectToHost(addr, 52340);
    mTcpServer.startServer(52340);

}

void TCPTextSender::startTransfer()
{
  //client.write("Hello, world", 13);
}
void TCPTextSender::quitApplication()
{
    qDebug() << "Exiting the application.";
    exit(1);
}

void TCPTextSender::run()
{
    QTextStream s(stdin);
    QVector<QString> temp;
    int res = readTextFile(filepath,temp);
    if(res)
    {
        printf("Exiting\n");
        return;
    }

    QByteArray ba;

    printf("Hit ENTER to send the file.\nType EXIT to close the program.\n");
    while(!shutdown)
    {

        QString input = (s.readLine()).toLower();
        if(input.contains("exit"))
        {
            shutdown = true;
        }
        else
        {
            if(isConnected)
            {
                //char data[10] = "apan";
                //QString temp = "apan1apanapanapan";

                for(int i = 0;i<temp.size();i++)
                {

                    ba.clear();
                    ba = temp[i].toLatin1();
                    //printf("Sending: %s with size=%d\n", ba.data(),ba.size());

                    emit sendData(temp[i]);
                    //client.write("apa",10);
                    //client.write(ba.data());
                    msleep(100);
                }
                printf("Files sent!\n");


            }
        }
        //shutdown = true;
    }
    //temp.clear();


}

void TCPTextSender::handleConnected()
{
    isConnected = true;
    qDebug() << "Connected to server.";
}

void TCPTextSender::handleDisconnected()
{
    isConnected = false;
    qDebug() << "Disconnected from server.";
    //exit(1);
}

void TCPTextSender::handleConnectionChanged(bool connected)
{
    isConnected = connected;
    if (connected)
        qDebug() << "Connected to client";
    else
        qDebug() << "Disconnected from client";
}

void TCPTextSender::handleSendData(QString in)
{

    qDebug() << "Sending: " << in.toLatin1().data();
    QByteArray tosend = in.toLatin1();
    //client.write(tosend.data());
    mTcpServer.dataToSend(tosend);

}

void TCPTextSender::handleSocketError(QAbstractSocket::SocketError error)
{
    //qDebug() << "Socket error " << error;

    switch (error) {
    case QAbstractSocket::ConnectionRefusedError:
        msleep(1000);
        //qDebug() << "Connection Refused.";
        connectToRec();
        break;
    default:
        break;
    }
}
