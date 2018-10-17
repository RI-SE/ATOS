#include "osccatalogreference.h"

OSCCatalogReference::OSCCatalogReference(const QString &type,const QString &path)
{
    m_type = type;
    m_path = path;
}

OSCCatalogReference::OSCCatalogReference(const OSCCatalogReference &other)
{
    this->m_type = other.m_type;
    this->m_path = other.m_path;
}

OSCCatalogReference::~OSCCatalogReference() {}

OSCParameterType OSCCatalogReference::getOSCCatalogType()
{
    static QHash<QString, qint8> map = {
        {"double", TYPE_ROUTE},
        {"integer", TYPE_TRAJECTORY},
    };
    return  map.value(m_type,TYPE_UNIDENTIFIED);
}


bool OSCCatalogReference::isPathValid(){

    // TODO
    return true;
}

void OSCCatalogReference::printobject()
{
    qDebug() << "OSCCatalogReference( " << m_type << "," << m_path << ")";
}
