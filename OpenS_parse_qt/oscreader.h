#ifndef OSCREADER_H
#define OSCREADER_H

#include <QObject>
#include <QXmlSimpleReader>
#include <QDomDocument>
#include <QDebug>
#include "oshandler.h"

class OSCReader : public QObject
{
public:

    typedef enum  {
        FILE_NO_CONTENT = 1,
        FILE_READ_OK = 0,
        FILE_OPEN_ERROR = -1,
        DOM_SET_CONTENT_ERROR = -2
    } XMLReadCode;

    typedef enum {
        NO_ERROR = 0,
        NODE_EMPTY = -1
    } DOMErrorCode;

    OSCReader();
    ~OSCReader();
    qint8 readtoDOMdoc(const QString &file_path);
    qint8 printDOMdoc();
    qint8 printDOMdocTree(const QDomNode &node, int level);

    qint8 loadDomdoc();

    qint8 parseOSFile(const QString &file_path);


private:

    OSHandler *m_handler;
    QXmlSimpleReader m_xmlReader;
    QDomDocument m_dom;

    QVector<OSCActor> m_actors;


    // Read specific parts of the OSC file
    qint8 readActors(const QDomNode &node);

};

#endif // OSCREADER_H
