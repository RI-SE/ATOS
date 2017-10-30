#include <QtCore>
#include <QCoreApplication>
#include <QTimer>

#include "generator.h"
#include "visualizationserver.h"

int main(int argc, char *argv[])
{
    /* ./websocket_visualization [port] [debug] [generatepath] [generationTime] */

    QCoreApplication a(argc, argv);

    uint32_t port = 53251;
    bool debug = false;
    Generator* gen = NULL;
    uint32_t generationTime = 100;

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

    VisualizationServer* server
            = new VisualizationServer(gen,
                                      generationTime,
                                      port,
                                      debug,
                                      VisualizationServer::NMEA_TCP_SERVER);


    QObject::connect(server, &VisualizationServer::closed, &a, &QCoreApplication::quit);
    return a.exec();
}
