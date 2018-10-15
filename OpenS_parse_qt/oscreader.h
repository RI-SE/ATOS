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
        NODE_INVALID_ACTOR = -5,
        DOM_NOT_LOADED = -6
    } DOMErrorCode;

    typedef enum {
        INIT = 0,
        DOC_READ,
        DOC_LOADED
    } OSCReader_INTERNAL_STATE;



    OSCReader();
    ~OSCReader();
    qint8 readtoDOMdoc(const QString &file_path);
    qint8 printDOMdoc();
    qint8 printDOMdocTree(const QDomNode &node, int level);

    qint8 loadDomdoc();
    qint8 printLoadedDomDoc();

    qint8 parseOSFile(const QString &file_path);


private:

    qint8 reader_state = INIT;

    OSHandler *m_handler;
    QXmlSimpleReader m_xmlReader;
    QDomDocument m_dom;

    QVector<OSCObject> m_actors;
    QVector<OSCPrivateAction*> m_initActions;
    QVector<OSCPrivateAction> m_storyActions;
    QVector<OSCParameter> m_parameters;
    QVector<OSCCatalogReference> m_catalogs;


    bool actorExists(const QString &actorName);
    qint32 findActorIndex(const QString &actorName);

    // Read specific parts of the OSC file
    qint8 readActors(const QDomElement &root);
    qint8 readInitActions(const QDomElement &root);
    qint8 readGlobalParameterDeclarations(const QDomElement &root);
    qint8 readCatalogReferences(const QDomElement &root);



};

#endif // OSCREADER_H
