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
        NODE_EMPTY = -1,
        CHILD_NODE_EMPTY = -2,
        NODE_NOT_EXISTS = -3,
        ATTR_NOT_EXIST = -4,
        NODE_INVALID_ACTOR = -5
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

    bool actorExists(const QString &actorName);

    // Read specific parts of the OSC file
    qint8 readActors(const QDomElement &root);
    qint8 readInitActions(const QDomElement &root);

};

#endif // OSCREADER_H
