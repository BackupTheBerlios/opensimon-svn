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


//------------------------------------------------------------------------------
/**
 * \file CallbackAdapter.h
 * \class CallbackAdapter
 * \author Markus Geimer
 *
 * CVS:
 *   - $Author: alangs $
 *   - $Date: 2004/12/14 18:22:17 $
 *   - $Revision: 1.1 $
 *
 * \brief Der CallbackAdapter ermöglicht den Zugriff auf Adressen von Memberfunktionen.
 *
 * In der Klasse CallBackAdapter wird generisch die Klasse sowie die
 * aufzurufenende Memberfunktion gespeichert. Bei einem Aufruf des ()-Operators
 * werden die gespeicherte Klasse und Funktion über den DREISTELLIGEN ->* -
 * Operator zusammengefügt und aufgerufen.
 *
 *   Die createCallback-Template Funktion macht die Verwendung dieses Tricks 
 * sehr viel einfacher, da durch die beiden Parameter implizit die Typen des zu
 * erzeugenden CallbackAdapters ermittelt werden kann.
 *
 * Anwendung findet dieses "Pattern" bei Funktionen, die als Parameter einen 
 * templateisierten" Funktionspointer erwarten.
 * 
 * Beispiel für ein Function-Call-Template (wie die boost-Funktionen):
 *
 * template <typename FuncT> void callFunc(FuncT func) {
 *      func();
 * }
 *
 * Richtige Callbackfunktionen wie die GLUT callback's werden leider nicht
 * unterstützt.
 */
//------------------------------------------------------------------------------

#ifndef CALLBACK_ADAPTER
#define CALLBACK_ADAPTER

//! Klasse zum verpacken von Memberfunktionen
template <class ClassTypeT, typename FuncTypeT>
class CallbackAdapter {

public:
	CallbackAdapter(ClassTypeT* instance, FuncTypeT function)
		: mInstance(instance), mFunction(function) {
		// empty constructor
	}

	void operator ()() const {
		(mInstance->*mFunction)();
	}


private:
	ClassTypeT* mInstance;
	FuncTypeT   mFunction;
};


template <class ClassTypeT, typename FuncTypeT> inline
CallbackAdapter<ClassTypeT, FuncTypeT> createAdapter(ClassTypeT* instance,
                                                     FuncTypeT   function) {
	return CallbackAdapter<ClassTypeT, FuncTypeT>(instance, function);
}


#endif // !CALLBACK_ADAPTER
