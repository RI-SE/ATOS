#ifndef OSHANDLER_H
#define OSHANDLER_H

#include <QObject>
#include <QXmlDefaultHandler>
#include <QDebug>

class OSHandler : public QXmlDefaultHandler
{
public:
    OSHandler();

    // overridden functions
    bool startDocument();
    bool endDocument();
    bool startEntity(const QString &name);
    bool endEntity(const QString &name);
    bool startElement(const QString & namespaceURI,
                      const QString & localName,
                      const QString & qName,
                      const QXmlAttributes & atts);

    bool endElement(const QString & namespaceURI,
                      const QString & localName,
                      const QString & qName);
private:
    int index = 0;

    void inc_index() {++index;}
    void dec_index() {index = index > 0 ? index - 1 : 0;}
};

#endif // OSHANDLER_H
