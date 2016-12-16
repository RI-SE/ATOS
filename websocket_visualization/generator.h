#ifndef GENERATOR_H
#define GENERATOR_H

#include <QtCore/QFile>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QTextStream>

class Generator : public QObject
{
    Q_OBJECT
public:
    explicit Generator(QString path, QObject *parent = Q_NULLPTR);
    ~Generator();

    QList<QString> GetNextMessages();
private:
    uint16_t mRow;
    QList<QList<QString>> mFileLines;
};

#endif // GENERATOR_H
