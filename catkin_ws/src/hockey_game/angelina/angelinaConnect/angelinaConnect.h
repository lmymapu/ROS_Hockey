#ifndef ANGELINA_H
#define ANGELINA_H

#include <QListWidget>
#include <QTimer>
#include <QWidget>
#include <QMap>

#include "referee.h"

class Referee;

class Angelina: public QWidget
{
    Q_OBJECT

    public:
       int myPosID;
       int myOriID;
       Angelina(QWidget *parent=0);
       Referee *referee;
       void testconnect();
       void ReportReady();
       void sendPosition(double x, double y);
       void tellAbRatio(double a, double b);
       void tellTeamColor(TeamColor);
       void ReportGoal();
       void ReportDone();
       void SendAlive();
    private Q_SLOTS:
       void GameStart();
       void DetectionStart();
       void GameOver();
       void StopMovement();
       void RealTeamColorFromAngelina(TeamColor);
       void RealAbValuesFromAngelina(double a, double b);
};

#endif /* ANGELINA_H */
