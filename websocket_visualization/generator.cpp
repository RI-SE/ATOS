#include "generator.h"

#include <QtCore/QDebug>
#include <QCoreApplication>
#include <QDir>

Generator::Generator(QString path, QObject *parent) :
    QObject(parent),
    mRow(0)
{
    QDir dir(path);

    if ( dir.exists() )
    {
        QFileInfoList entries = dir.entryInfoList( QDir::NoDotAndDotDot | QDir::Files);
        for(int i = 0; i < entries.size(); ++i)
        {
            QList<QString> stringList;
            QFile file(entries[i].absoluteFilePath());
            file.open(QIODevice::ReadOnly);

            QTextStream in(&file);
            while(!in.atEnd())
            {
                stringList << in.readLine();
            }

            file.close();
            mFileLines.append(stringList);
        }
    }

}

Generator::~Generator()
{
}

void Generator::rewind(){ mRow = 0;}

QList<QString> Generator::GetNextMessages()
{
    QList<QString> list;
    bool bEndOfFile = false;

    for(int k=0;k<mFileLines.size();++k)
    {
        list << mFileLines[k][mRow];

        if(mFileLines[k].size() <= (mRow+1) )
        {
            bEndOfFile = true;
        }
    }

    if(!bEndOfFile)
    {
        ++mRow;
    }

    return list;
}


