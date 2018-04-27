#include "backend.h"


BackEnd::BackEnd(QObject *parent) :
    QObject(parent)
{
    IPConnecitonHandler *iph = new IPConnecitonHandler();
    iph->destroy();
}

QString BackEnd::userName()
{
    return m_userName;
}

void BackEnd::setUserName(const QString &userName)
{
    if (userName == m_userName)
        return;

    m_userName = userName;
    emit userNameChanged();
}
