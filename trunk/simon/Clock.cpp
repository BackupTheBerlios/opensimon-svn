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


// $Id: Clock.cpp,v 1.20 2004/12/15 17:04:49 trappe Exp $
//------------------------------------------------------------------------------
/**
*  \class Clock
*  \author Trappe, Rueger
*
* CVS:
*   - $Author: trappe $
*   - $Date: 2004/12/15 17:04:49 $
*   - $Revision: 1.20 $
*
*  \brief Stellt Timer-Funktionen für die Physik-Simulation zur Verfügung
*
*  
*
*/
//------------------------------------------------------------------------------

#include "Clock.h"
#include <cassert>

#ifdef OS_WINDOWS
# include <windows.h> //Für GetTickCounts()
#endif
#ifdef OS_LINUX
# include <sys/time.h>
#endif

bool Clock::mInitWasCalled = false;

Clock::TimeType Clock::mInitTime;

Clock::TimeType Clock::getTime() {
#ifdef OS_WINDOWS
	return GetTickCount();
#endif

#ifdef OS_LINUX
	timeval timeOfDay;
	gettimeofday(&timeOfDay,0);
	return ((timeOfDay.tv_sec*1000)+(timeOfDay.tv_usec/1000.0f));
#endif	

}

void Clock::init() {
	assert(!mInitWasCalled);
	mInitTime = getTime();
	mInitWasCalled = true;
}

Clock::TimeType Clock::getTicks() {
	
	return getTime() - mInitTime;

}

void Clock::start(void) {

	mStartTime = getTicks();
}

void Clock::stop() {

	mStopTime = getTicks();

}

/**
 * \brief Liefert die Zeit zwischen start() und stop() in Millisekunden
 *
 * \return Millisekunden die vergangen sind zwischen start und Stop
 */
Clock::TimeType Clock::getDuration() const {
	
	return mStopTime - mStartTime;

}

void Clock::sleep(const TimeType time) {

// For Unix (eg. GNU/Linux) we have a usleep(microseconds) method.
// On a Windows Box Sleep(milliseconds) is used.
#ifdef HAVE_UNISTD_H
	usleep((int)time*1000);
#else
	#ifdef OS_WINDOWS
		Sleep((int)time);
	#else
		#error Sorry. Cantnot find a nice sleep()-Function.
	#endif
#endif

}

/**
 * \see Clock :: stop  
 * 
 * \param minTime Millisekunden die seit "start" mindestens vergangen sein sollen
 * \return TimeType Zeit zwischen "start" und "stop", minestens jedoch die
 * übergebene mindest-Zeit
 *
 * Diese Funktion dient dazu, das die Physik nicht zu schnell abläuft und mit
 * der echten, verbrauchten Zeit rechnet. Beispiel:
 * Benötigt die Berechnung der Physik 20ms und "minTime" ist 33ms, wird das
 * Programm 13ms angehalten und es wird 33 zurückgegeben.
 * Benötigt die Berechnung nun aber 50ms, wird das Programm nicht angehalten und
 * 50ms zurückgegeben.
 * So rechnet die Physik mit einem Zeitschrit von min 33ms oder, wenns länger
 * dauert mit der entsprechend verstrichenen Zeit.
*/
Clock::TimeType Clock::fillUp(const TimeType minTime) {

	stop();
	TimeType elapsedTime = getDuration();
	TimeType difference = minTime - elapsedTime;

	if (difference > 0) {
		sleep (difference);
		return minTime;
	}
	else {
		return elapsedTime;
	}
}

std::ostream& operator <<(std::ostream& os,const Clock& clock) {

	std::streamsize oldPrecission=os.precision();
	os.precision(3);
	os << clock.getDuration() << " ms";
	os.precision(oldPrecission);

	return os;
}

