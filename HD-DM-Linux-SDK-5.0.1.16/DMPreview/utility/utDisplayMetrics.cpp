#include <QApplication>
#include <QDesktopWidget>
#include "utDisplayMetrics.h"

int utDisplayMetrics::GetDisplayWidth()
{    
    QSize size = QApplication::desktop()->size();
    return size.width();
}

int utDisplayMetrics::GetDisplayHeight()
{
    QSize size = QApplication::desktop()->size();
    return size.height();
}

void utDisplayMetrics::GetDisplayResolution(int &nWidth, int &nHeight)
{
    QSize size = QApplication::desktop()->size();
    nWidth = size.width();
    nHeight = size.height();
    return;
}

QRect utDisplayMetrics::GetAvailableGeometry()
{
    return QApplication::desktop()->availableGeometry();;
}
