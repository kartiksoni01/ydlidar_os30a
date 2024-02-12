#ifndef UTDISPLAYMETRICS_H
#define UTDISPLAYMETRICS_H
#include <QRect>

class utDisplayMetrics
{
public:
    static int GetDisplayWidth();
    static int GetDisplayHeight();
    static void GetDisplayResolution(int &nWidth, int &nHeight);
    static QRect GetAvailableGeometry();
private:
    utDisplayMetrics() = default;
};

#endif // UTDISPLAYMETRICS_H
