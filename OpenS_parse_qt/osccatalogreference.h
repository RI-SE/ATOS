#pragma once
#ifndef OSCCATALOGREFERENCE_H
#define OSCCATALOGREFERENCE_H

#include "osclib.h"
#include <QFile>

typedef qint8 OSCCatalogType;

class OSCCatalogReference : public OSCType
{
public:

    enum ParameterType {
        TYPE_UNIDENTIFIED = -1,
        TYPE_ROUTE,
        TYPE_TRAJECTORY
    };

    OSCCatalogReference(const QString &type = "",const QString &path ="");
    OSCCatalogReference(const OSCCatalogReference &other);

    ~OSCCatalogReference();

    OSCCatalogType getOSCCatalogType();

    bool isPathValid();

    void printobject();

private:
    QString m_type;
    QString m_path;
};

#endif // OSCCATALOGREFERENCE_H
