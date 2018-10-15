#include "oscreader.h"

OSCReader::OSCReader()
{
    m_handler = new OSHandler();
    m_xmlReader.setContentHandler(m_handler);
    m_xmlReader.setErrorHandler(m_handler);
}

OSCReader::~OSCReader()
{
    foreach(OSCPrivateAction* actionptr, m_initActions)
    {
        delete actionptr;
    }
    delete m_handler;
}


qint8 OSCReader::readtoDOMdoc(const QString &file_path)
{
    QString domError;
    int errorLine;
    QFile file(file_path);
    if (!file.open(QIODevice::ReadOnly))
        return FILE_OPEN_ERROR;
    if (!m_dom.setContent(&file,&domError,&errorLine)) {
        file.close();
        qDebug() << domError << errorLine;
        return DOM_SET_CONTENT_ERROR;
    }
    file.close();
    reader_state = DOC_READ;
    return FILE_READ_OK;
}

qint8 OSCReader::printDOMdoc()
{
    if (reader_state < DOC_READ) return FILE_NO_CONTENT;
    // print out the element names of all elements that are direct children
    // of the outermost element.
    QDomElement docElem = m_dom.documentElement();

    QDomNode n = docElem.firstChild();
    qDebug() << "<------" + docElem.tagName() + "------>";
    printDOMdocTree(n,0);
    /*
     *
    while(!n.isNull()) {
        QDomElement e = n.toElement(); // try to convert the node to an element.
        if(!e.isNull()) {
            qDebug() << e.tagName(); // the node really is an element.
        }
        n = n.nextSibling();
    } */
    return 0;
}

qint8 OSCReader::printDOMdocTree(const QDomNode &node,int level)
{
    if (node.isNull()) return 0;
    if (!node.isComment()) qDebug() << QString(level,' ') + node.toElement().tagName();
    if (node.hasChildNodes()) printDOMdocTree(node.firstChild(), level + 1);

    return printDOMdocTree(node.nextSibling(), level);
}


qint8 OSCReader::loadDomdoc()
{
    if (reader_state < DOC_READ) return FILE_NO_CONTENT;

    QDomElement docElem = m_dom.documentElement();
    //QDomNode firstnode = docElem.firstChild();

    qDebug() << "ECODE(CatalogRef)" << readCatalogReferences(docElem);
    qDebug() << "ECODE(parameters):" << readGlobalParameterDeclarations(docElem);
    qDebug() << "ECODE(actors):" << readActors(docElem);
    qDebug() << "ECODE(actions):" << readInitActions(docElem);



    reader_state = DOC_LOADED;
    return 0;
}

qint8 OSCReader::printLoadedDomDoc()
{
    if(reader_state < DOC_LOADED) return DOM_NOT_LOADED;

    qDebug() << "# Loaded DOM Doc #";

    foreach(OSCObject actor, m_actors)
    {
        actor.printobject();
    }

    foreach(OSCPrivateAction* action, m_initActions)
    {
        action->printobject();
    }

    foreach(OSCParameter param, m_parameters)
    {
        param.printobject();
    }

    foreach(OSCCatalogReference catalog, m_catalogs)
    {
        catalog.printobject();
    }
    return 0;
}

qint8 OSCReader::parseOSFile(const QString &file_path)
{
    // TODO: handle non-existant file path
    QFile input_file(file_path);

    QXmlInputSource input(&input_file);

    bool ok = m_xmlReader.parse(input);

    if (!ok) qDebug() << "Parsing not ok.";
    else qDebug() << "Parse ok!";

    return 0;
}

bool OSCReader::actorExists(const QString &actorName)
{
    foreach(OSCObject actor, m_actors)
    {
        if(actor.isSame(actorName)) return true;
    }
    return false;
}

qint32 OSCReader::findActorIndex(const QString &actorName)
{
    for (int i = 0; i < m_actors.size();++i)
    {
        if(m_actors[i].isSame(actorName)) return i;
    }
    return -1;
}



qint8 OSCReader::readActors(const QDomElement &root)
{
    if(root.isNull()) return NODE_EMPTY;

    QDomElement e = root.firstChildElement(OSC_KEYWORD_ACTORS);
    if (e.isNull()) return NODE_NOT_EXISTS;

    QDomNodeList dlist = e.elementsByTagName(OSC_KEYWORD_ACTOR);

    for(int i = 0; i<dlist.count();i++)
    {
        QDomAttr a = dlist.item(i).toElement().attributeNode("name");
        if (a.isNull()) return ATTR_NOT_EXIST;
        m_actors.append(OSCObject(a.value()));
    }
    return NO_ERROR;
}

qint8 OSCReader::readInitActions(const QDomElement &root)
{
    if(root.isNull()) return NODE_EMPTY;

    QDomElement e = root.firstChildElement(OSC_KEYWORD_STORYBOARD);
    if (e.isNull()) return NODE_NOT_EXISTS;

    e = e.firstChildElement(OSC_KEYWORD_INIT);
    if (e.isNull()) return NODE_NOT_EXISTS;

    QDomNodeList actors = e.elementsByTagName(OSC_KEYWORD_PRIVATE);
    QDomNodeList actor_actions;
    QDomElement actorElem;
    QString actorName;
    QDomElement actionElem;

    QVector<OSCPrivateAction*> temp_actions;
    OSCPrivateAction* loaded_action;
    qint32 actor_index;
    for(int i = 0; i < actors.size(); i++)
    {
        actorElem = actors.at(i).toElement();
        actorName = actorElem.attribute("object","");
        // Find the actor among all parsed actors

        actor_index = findActorIndex(actorName);
        if (actor_index < 0) return NODE_INVALID_ACTOR;

        // Add all actions which is now bound to the actor
        actor_actions = actorElem.elementsByTagName(OSC_KEYWORD_ACTION);
        for (int j = 0; j < actor_actions.size(); ++j) {

            // Find the type of action
            actionElem = actor_actions.at(j).toElement();
            

            switch (OSCPrivateAction::mapOSCPrivateAction(actionElem.nodeName())) {
            case OSCPrivateAction::MEETING_ACTION:
                loaded_action = new OSCPrivateAction(&m_actors[actor_index],actionElem.firstChildElement().tagName());
                break;
            default:
                qDebug() << "Unknown Action:" << actionElem.nodeName();
                loaded_action = new OSCPrivateAction(&m_actors[actor_index],actionElem.firstChildElement().tagName());
            }

            temp_actions.append(loaded_action);

        }
    }

    m_initActions = temp_actions;
    //qDebug() << "#actions: " << m_initActions.size();

    return NO_ERROR;
}

qint8 OSCReader::readGlobalParameterDeclarations(const QDomElement &root)
{

    if(root.isNull()) return NODE_EMPTY;

    QDomElement e = root.firstChildElement(OSC_KEYWORD_PARAMETERDECLARATION);
    if (e.isNull()) return NODE_NOT_EXISTS;


    QDomNodeList parameters = e.elementsByTagName(OSC_KEYWORD_PARAMETER);
    QDomElement parameterElement;

    QDomAttr paramName;
    QDomAttr paramType;
    QDomAttr paramValue;

    for(int i = 0; i < parameters.size();++i)
    {
        parameterElement = parameters.item(i).toElement();
        if (parameterElement.isNull()) continue;
        paramName = parameterElement.attributeNode("name");
        paramType = parameterElement.attributeNode("type");
        paramValue = parameterElement.attributeNode("default");
        m_parameters.append(OSCParameter(paramName.value(),paramType.value(),paramValue.value()));
    }

    return NO_ERROR;
}

qint8 OSCReader::readCatalogReferences(const QDomElement &root)
{

    if(root.isNull()) return NODE_EMPTY;

    QDomElement e = root.firstChildElement(OSC_KEYWORD_CATALOG);
    if (e.isNull()) return NODE_NOT_EXISTS;


    QDomNodeList catalogs = e.childNodes();
    QDomElement catalogElem;
    QDomElement directoryElem;
    QDomAttr paramPath;

    qDebug() << "Catalog elements: "<< catalogs.size();
    for(int i = 0; i < catalogs.size();++i)
    {
        catalogElem = catalogs.item(i).toElement();
        if (catalogElem.isNull()) {
            qDebug() << catalogs.item(i).nodeName();
            continue;
        }
        directoryElem = catalogElem.firstChildElement(OSC_KEWORD_DIRECTORY);
        if (directoryElem.isNull()) return NODE_EMPTY;
        paramPath = directoryElem.attributeNode("path");
        m_catalogs.append(OSCCatalogReference(catalogElem.nodeName(),paramPath.value()));
    }

    return NO_ERROR;
}
