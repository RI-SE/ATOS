#include "tcptextsender.h"


TCPTextSender::TCPTextSender(uint16_t port, QString file, QObject *parent):
mTcpServer(new TcpServerSimple)
{

    //loadedTextFile = new QLinkedList<QString*>();
    this->port = port;
    filepath = file;

    connect(mTcpServer,SIGNAL(connectionChanged(bool)),
            this,SLOT(handleConnectionChanged(bool)));

}

TCPTextSender::~TCPTextSender(){
    delete mTcpServer;
    //delete loadedTextFile;
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
        line = in.readLine();
        qDebug() << line;
        output.append(line);
    }
    qDebug() <<  QString::number(output.size()) << " lines read.";

    return 0;
}

void TCPTextSender::run()
{
    QTextStream s(stdin);
    QVector<QString> temp;


    //QString input(1024,'\0');

    int res = readTextFile(filepath,temp);
    if(res)
    {
        printf("Exiting\n");
        return;
    }
    //qDebug() << "Size of return vector: " << QString::number(temp.size());

    //mTcpServer->startServer(port);
    mTcpServer->connectToRemote("127.0.0.1",port);
    //while(mTcpServer->mTcpSocket->waitForConnected());

    printf("Hit ENTER to send the file.\nType EXIT to close the program.\n");
    while(!shutdown)
    {

        QString input = "";//(s.readLine()).toLower();
        if(input.contains("exit"))
        {
            shutdown = true;
        }
        else
        {
            if(isConnected)
            {
                printf("Files sent!\n");
            }

        }
    }
    temp.clear();
    mTcpServer->stopServer();

}

void TCPTextSender::handleConnectionChanged(bool status)
{
    isConnected = status;
    if(isConnected)
    {
        qDebug() << "Connection established.";
    }
    else
    {
        qDebug() << "Disconnected.";
    }
}


