#ifndef OSCREADER_H
#define OSCREADER_H

#include <QObject>
#include <QXmlSimpleReader>
#include <QDebug>
#include "oshandler.h"

class OSCReader : public QObject
{
public:
    OSCReader();
    ~OSCReader();

    quint8 parseOSFile(const QString &file_path);
private:

    OSHandler *m_handler;
    QXmlSimpleReader m_xmlReader;




};

#endif // OSCREADER_H
