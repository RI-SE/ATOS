#include <QtCore>
#include <QCoreApplication>
#include <QTimer>

#include "generator.h"
#include "visualizationserver.h"

enum {
    NO_ARG = 0,
    UDP_ADDRESS,
    UDP_PORT,
    GENERATOR_PATH
} ARG_STATE;

int main(int argc, char *argv[])
{
    /* ./websocket_visualization [port] [debug] [generatepath] [generationTime] */

    QCoreApplication a(argc, argv);

    uint32_t port = 53251;
    bool debug = false;
    Generator* gen = NULL;
    QHostAddress addr = QHostAddress::Any;
    uint32_t generationTime = 100;
    quint8 input_type = VisualizationServer::NMEA_TCP_CLIENT;
    quint8 output_type = VisualizationServer::WEBSOCKET;


    quint8 next_arg = NO_ARG;

    QString argument_data = "";
    for(int i = 0; i < argc;i++)
    {

        switch (next_arg) {
        case NO_ARG:
            break;
        case UDP_ADDRESS:
            addr = QHostAddress(QString(argv[i]));
            next_arg = UDP_PORT;
            continue;
        case UDP_PORT:
            port = atoi(argv[i]);
            next_arg = NO_ARG;
            output_type = VisualizationServer::UDPSOCKET;
            continue;
        case GENERATOR_PATH:
        {
            QString str = argv[i];
            gen = new Generator(str);
            next_arg = NO_ARG;
        }
            continue;
        default:
            qDebug() << "Unknown command " << argv[i];
            return -1;
        }

        argument_data = argv[i];
        if (QString::compare(argument_data,"-udp") == 0) next_arg = UDP_ADDRESS;
        if (QString::compare(argument_data,"-gen") == 0) next_arg = GENERATOR_PATH;
        if (QString::compare(argument_data,"-debug") == 0) debug = true;

    }

    if (next_arg != 0) qDebug() << "Incorrect number of arguments.";


    /*
    if(argc > 1)
    {
        port = atoi(argv[1]);
    }

    if(argc > 2)
    {
        debug = argv[2];
    }
    // Change the code bellow to handle multiple scenarios
    if(argc > 3)
    {
        gen = new Generator(argv[3]);
    }

    if(argc > 4)
    {
        generationTime = atoi(argv[4]);
    }
    */


    VisualizationServer* server = new VisualizationServer(gen,
                                                          generationTime,
                                                          addr,
                                                          port,
                                                          debug,
                                                          input_type,
                                                          output_type);


    QObject::connect(server, &VisualizationServer::closed, &a, &QCoreApplication::quit);
    server->start();


    return a.exec();
}
