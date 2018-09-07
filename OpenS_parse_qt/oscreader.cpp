#include "oscreader.h"

OSCReader::OSCReader()
{
    m_handler = new OSHandler();
    m_xmlReader.setContentHandler(m_handler);
    m_xmlReader.setErrorHandler(m_handler);
}

OSCReader::~OSCReader()
{
    delete m_handler;
}

quint8 OSCReader::parseOSFile(const QString &file_path)
{
    // TODO: handle non-existant file path
    QFile input_file(file_path);

    QXmlInputSource input(&input_file);

    bool ok = m_xmlReader.parse(input);

    if (!ok) qDebug() << "Parsing not ok.";
    else qDebug() << "Parse ok!";

    return 0;
}
