/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA BASESTATION
 *
 * CAMBADA BASESTATION is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA BASESTATION is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __REFBOXDIALOG_H
#define __REFBOXDIALOG_H

#include <QTcpSocket>
#include <QUdpSocket>
#include <QSocketNotifier>

#include "ui_RefBoxDialog.h"

class RefBoxDialog : public QDialog, public Ui::RefBoxDialog
{
Q_OBJECT

public:
	RefBoxDialog(QDialog *parent = 0);
	~RefBoxDialog();
	int connected;

	void processNewRefBoxMsg();
	void processRefBoxMsg();

private:

protected:
	QString destHost;
	quint16 destPort;
	QString interface;

	/* TCP Socket */
	QTcpSocket *socket;
	QUdpSocket *udpSocket;

	char data_received[1500];
	int before_stop_gamePart;

public Q_SLOTS:
	void connectToHost(void);
	void receiveRefMsg(void);

	void SetInterface(void);
	void SetHostAdd(void);
	void SetHostPort(int val);

	void update_manual_config(void);
	void apply_Button_pressed(void);

	void Timer_start_bot_pressed(void);
	void Timer_stop_bot_pressed(void);

	Q_SIGNALS:
	void transmitCoach(void);
	void changeGoalColor(int);
	void updateGameParam(void);

};

enum WSColor
{
	Blue,
	Yellow,
	Magenta,
	Cyan
};

static const int num_refbox_signals = 21;
static const char refbox_signal_names [num_refbox_signals][20] =
{
"SIGnop             ",
"SIGstop            ",
"SIGhalt            ",
"SIGstart           ",
"SIGready           ",
"SIGdropBall        ",
"SIGourKickOff      ",
"SIGtheirKickOff    ",
"SIGourFreeKick     ",
"SIGtheirFreeKick   ",
"SIGourGoalKick     ",
"SIGtheirGoalKick   ",
"SIGourCornerKick   ",
"SIGtheirCornerKick ",
"SIGourThrowIn      ",
"SIGtheirThrowIn    ",
"SIGourPenalty      ",
"SIGtheirPenalty    ",
"SIGourGoalScored   ",
"SIGtheirGoalScored ",
"SIGparking         "
};

#endif
