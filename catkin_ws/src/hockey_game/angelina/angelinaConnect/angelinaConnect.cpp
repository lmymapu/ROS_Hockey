#include <QDebug>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "angelinaConnect.h"
#include <iostream>
#include "referee.h"

Angelina::Angelina(QWidget *parent) :
    QWidget(parent), myPosID(0), myOriID(0), referee(0)
{
}
void Angelina::testconnect() {
    if (referee)
        delete referee;
    referee = new Referee(5, this);
    connect(referee, SIGNAL(gameStart()), this, SLOT(GameStart()));
    connect(referee, SIGNAL(detectionStart()), this, SLOT(DetectionStart()));
    connect(referee, SIGNAL(gameOver()), this, SLOT(GameOver()));
    connect(referee, SIGNAL(abValues(double,double)), this, SLOT(RealAbValuesFromAngelina(double,double)));
    connect(referee, SIGNAL(stopMovement()), this, SLOT(StopMovement()));
    //connect(referee, SIGNAL(stopMovement()), this, SLOT(ReportDone()));
    connect(referee, SIGNAL(trueColorOfTeam(TeamColor)), this, SLOT(RealTeamColorFromAngelina(TeamColor)));
    referee->connectToServer("129.187.240.199", 10000); //
}

void Angelina::ReportReady()   //tell Angelina the robot is ready
{
    referee->reportReady();
}

void Angelina::sendPosition(double x, double y)   //tell Angelina the current position of robot
{
    referee->tellEgoPos(x, y);
}

void Angelina::tellAbRatio(double a, double b)   //tell Angelina AB ratio of Field
{
    double ratio = a/b;
    referee->tellAbRatio(ratio);
}

void Angelina::tellTeamColor(TeamColor tColour) //tell Angelina the team color that we have detected
{
    referee->tellTeamColor(tColour);
}

void Angelina::ReportGoal() // tell Angelina the robot has goaled
{
    referee->reportGoal();

}

void Angelina::ReportDone() // tell Angelina the robot has goaled all the Pucks. The connection will be broken if using this funcition
{
    referee->reportDone();
}


void Angelina::SendAlive() /* tell Angelina the robot is still alive.
                               According to the Folie the robot needs to send alive signal at least 45 seconds.
                               Maybe we need a loop or there are some time function in ROS?
                           */
{
    referee->sendAlive();
}
/* The following functions don't need to be called. They will be automatically called by receiving signals from Angelina
   e.g. We click the Stop button on Angelina, one of following function "StopMovement" will be used. We can add the
        responding actions in the function.
*/
void Angelina::GameStart()  // Angelina tells that the game starts
{
    qDebug() << "Game has been started!" << endl;
}

void Angelina::DetectionStart() // Angelina tells that the Robot can detect the field
{
    qDebug() << "Detection Time running!" << endl;
}

void Angelina::GameOver() //Angelina tells that the game is over
{
    qDebug() << "Game Over!" << endl;
    referee->reportDone();
}

void Angelina::StopMovement() // Angelina tells that the Robot must stop immediately
{
    qDebug() << "Stop immediately!" << endl;
}

void Angelina::RealTeamColorFromAngelina(TeamColor tcolor)  // Angelina tells the true team color
{
    if (tcolor == yellow)
        qDebug() << "Real Team color: Yellow" << endl;
    else
        qDebug() << "Real Team color: Blue" << endl;
}

void Angelina::RealAbValuesFromAngelina(double a, double b)  // Angelina tells the true ABratio
{
    qDebug() << "Real AB values are, A =" << a << "B =" << b << endl;
}
