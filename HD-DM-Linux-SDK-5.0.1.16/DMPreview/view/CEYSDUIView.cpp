#include "CEYSDUIView.h"
#include <QWidget>

CEYSDUIView::CEYSDUIView()
{

}

void CEYSDUIView::UpdateUI()
{
    RUN_ON_UI_THREAD(
    UpdateChildern();
    UpdateSelf();
    );
}

void CEYSDUIView::UpdateChildern()
{
    UpdateChildernEYSDUIView(dynamic_cast<QWidget *>(this));
}

void CEYSDUIView::UpdateChildernEYSDUIView(QWidget *pWidget)
{
    if (!pWidget) return;

    QObjectList objcetList = pWidget->children();
    for (QObject *pObject : objcetList){
        CEYSDUIView *pUiView = dynamic_cast<CEYSDUIView *>(pObject);
        if (!pUiView) {
            UpdateChildernEYSDUIView(dynamic_cast<QWidget *>(pObject));
            continue;
        }
        pUiView->UpdateUI();
    }
}

void CEYSDUIView::AddUpdateTimer(int mesc)
{
    QWidget *pWidget = dynamic_cast<QWidget *>(this);
    if(!pWidget) return;

    if(m_updateTimer.isActive()){
        m_updateTimer.stop();
    }

    pWidget->connect(&m_updateTimer, SIGNAL(timeout()), SLOT(update()));
    m_updateTimer.start(mesc);
}
