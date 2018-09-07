#include "oshandler.h"

OSHandler::OSHandler()
{

}

bool OSHandler::startDocument()
{
    qDebug() << "Started Reading Document";
    return true;
}

bool OSHandler::endDocument()
{
    qDebug() << "Stopped Reading Document";
    return true;
}
bool OSHandler::startEntity(const QString &name)
{
    QString outstring = "";
    /*
    for(int i= 0; i < index;i++){
        outstring += " ";
    }*/
    outstring.fill(' ',index);
    outstring += name;
    inc_index();
    qDebug() << outstring;
    return true;
}

bool OSHandler::endEntity(const QString &name)
{
    (void) name;
    dec_index();
    return true;
}

bool OSHandler::startElement(const QString & namespaceURI,
                             const QString &localName,
                             const QString & qName,
                             const QXmlAttributes & atts)
{
    (void) namespaceURI;
    (void) localName;
    (void) atts;
    //startEntity(qName);
    qDebug() << qName;
    return true;
}

bool OSHandler::endElement(const QString & namespaceURI,
                  const QString & localName,
                  const QString & qName)
{
    (void) namespaceURI;
    (void) localName;

    endEntity(qName);

    return true;
}
