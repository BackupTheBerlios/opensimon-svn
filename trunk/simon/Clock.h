/**
 * Simon is the legal property of its developers, whose names are too
 * numerous to list here.  Please refer to the COPYRIGHT file
 * distributed with this source distribution.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


// $Id: Clock.h,v 1.12 2004/12/14 18:22:17 alangs Exp $
//------------------------------------------------------------------------------
/**
*  \file Clock.h
*  \class Clock
*  $Author: alangs $
*  $Date: 2004/12/14 18:22:17 $
*  $Revision: 1.12 $
*/
//------------------------------------------------------------------------------

#ifndef CLOCK_H
#define CLOCK_H

#include <simon/config.h>

#include <iostream>

class Clock {

public:

	typedef double TimeType;

	static void init();

	//! Setzt den laufenden Prozess für gegebenen Zeit in den Ruhezustand
	static void sleep(const TimeType time);
	
	//! Setzt den laufenden Prozess bis zur gegebenen Endzeit in den Ruhezustand
	//static void sleepUntil(const TimeType endTime);		

	//! Hällt den Prozess an (sleep), bis mindestens die  übergebene Zeit
	//! seit Aufruf von "start" erreicht ist.
	TimeType fillUp(const TimeType minTime);

	//! startet die Stoppuhr
	void start();

	//! stoppt die Stoppuhr
	void stop();

	//! Gibt die Millisekunden seit Programmstart zurück
	static TimeType getTicks();

	//! Liefert die Zeit, die zwischen start und stop vergangen ist.
	TimeType getDuration() const;

	//! Gibt die Zeit zwischen start() und stop() auf einen ostream aus.
	friend std::ostream& operator <<(std::ostream& os, const Clock& clock);		

private:

	TimeType mStartTime;
	TimeType mStopTime;

	static TimeType mInitTime;

	static TimeType getTime();

	// keep track of calling init()
	static bool mInitWasCalled;
};

#endif
