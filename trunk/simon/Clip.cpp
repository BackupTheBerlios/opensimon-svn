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
 *  \class Clip
 *  \author Kipermann, Haeusler
 *
 * CVS:
 *   - $Author: ritschel $
 *   - $Date: 2004/09/03 09:06:11 $
 *   - $Revision: 1.12 $
 * \brief clipping mit cohen-sutherland
 *
 */
//------------------------------------------------------------------------------
#include "Clip.h"
#include <iostream>
#include <algorithm>

using namespace std;

Clip::Clip(void)
{
}

Clip::~Clip(void)
{
}


/**
* Funktion zum Clippen
*
* \param       window      Ein Array aus Vector3<float> in dem die 4 Punkte des Fensters stehen an dem geclippt wird
* \param	   line		Ein Array aus Vector3<float> in dem die 2 Endpunkte des Liniensegments stehen
*/
unsigned int Clip::clipLine(Vec3* window, Vec3* line, unsigned int ignoreAxis)
{
	int i=0;
	float xmin, xmax, ymin, ymax;		// Clip-Fensterraender
	float x[4];
	float y[4];
	unsigned int a,b;
	
	Vec3* P1 = line;
	Vec3* P2 = line + 1;

	switch(ignoreAxis)
	{
		case X:
			a = Z;
			b = Y;
			break;
		case Y:
			a = X;
			b = Z;
			break;
		case Z:
			a = X;
			b = Y;
			break;
		default:
			assert(false);
			break;
	}
	
	for (i=0;i<4;i++)
	{
		x[i]=(window[i])[X];
	}

	for (i=0;i<4;i++)
	{
		y[i]=(window[i])[Y];
	}


	std::sort(x, x + 4);
	std::sort(y, y + 4);
	
	xmin = x[0];
	xmax = x[3];
	ymin = y[0];
	ymax = y[3];

	//assert(false); // falsch
	
	return cohenSutherland(a, b, P1, P2, xmin, xmax, ymin, ymax);
}


/**
* \brief Liefert den Region Code zu einem Punkt
* \param P Der Punkt
* \param xmin, xmax, ymin,ymax minimale und maximale x bzw. y koordinate des Fensters 
*/
int Clip::region_code(unsigned int a, unsigned int b, Vec3* P, float xmin, float xmax, float ymin, float ymax)
{
	int c = EMPTY;
	if ((*P)[a] < xmin) c = LEFT; 
	else if ((*P)[a] > xmax) c = RIGHT;
	if ((*P)[b] < ymin) c |= BOTTOM; 
	else if ((*P)[b] > ymax) c |= TOP;
	return c;
}
	


/**
* \brief Führt den Cohen Sutherland Algorithmus aus
* \param		P1,P2 Anfangs- und Endpunkt der geclippten Geraden
* \param		Q1, Q2  Speicher für die geclippten Punkte
* \param		xmin, xmax, ymin,ymax minimale und maximale x bzw. y koordinate des Fensters 
*/
unsigned int Clip::cohenSutherland(unsigned int a, unsigned int b, Vec3* P1, Vec3* P2, float xmin, float xmax, float ymin, float ymax)						
{	
	Vec3 Q(0,0,0);
	
	unsigned int edgePos = 10;
	
	// true falls Gerade p1-p2 nicht senkrecht laeuft
	float slope = 0.0;	// Steigung der Geraden p1-p2
	int C, C1, C2;		// Bereichs-Code
	bool finite_slope;

	finite_slope = ((*P1)[a] != (*P2)[a]);

	if (finite_slope)
		slope = ((*P2)[b] - (*P1)[b]) / ((*P2)[a] - (*P1)[a]);

	C1 = region_code(a, b, P1, xmin, xmax, ymin, ymax);
	C2 = region_code(a, b, P2, xmin, xmax, ymin, ymax);

	//cout <<"C1: " <<C1 <<"\n" <<"C2: " <<C2 <<"\n"; 
	
	if (C1 == EMPTY && C2 == EMPTY)
		return 0;
	
	while ((C1 != EMPTY) || (C2 != EMPTY)) 
	{
		if ((C1&C2) != EMPTY) 
		{		
			//cout << "BEIDE AUF DER SELBEN SEITE AUSSERHALB\n"; // beide auf derselben Seite ausserhalb
			edgePos = 1;
			break;
		}
		else
		{
			if (C1 == EMPTY) 
				C = C2; 
			else 
				C = C1; // C ist ausserhalb. Berechne einen Schnittpunkt mit den verlaengerten Fensterkanten
			if ((C & LEFT) != EMPTY) // schneide mit linker Fenster-Kante
			{ 
				Q[a] = xmin;
				Q[b] = ((Q[a] - (*P1)[a]) * slope + (*P1)[b]);
			} 
			else if ((C & RIGHT) != EMPTY) // schneide mit rechter Fenster-Kante
			{
				Q[a] = xmax;
				Q[b] = ((Q[a] - (*P1)[a]) * slope + (*P1)[b]);
			}
			else if ((C & BOTTOM) != EMPTY) // schneide mit unterer Fenster-Kante
			{ 
				Q[b] = ymin;
				if (finite_slope)
					Q[a] = ((Q[b] - (*P1)[b]) / slope + (*P1)[a]); 
				else 
					Q[a] = (*P1)[a];
			}
			else if ((C & TOP) != EMPTY) // schneide mit oberer Fenster-Kante
			{ 
				Q[b] = ymax;
				if (finite_slope)
					Q[a] = ((Q[b] - (*P1)[b]) / slope + (*P1)[a]); 

				else
					Q[a] = (*P1)[a];
			}
			if (C==C1) 
			{	
				(*P1)[a] = Q[a]; 
				(*P1)[b] = Q[b]; 
				C1 = region_code (a, b, P1, xmin, xmax, ymin, ymax);
			}
			else 
			{	
				(*P2)[a] = Q[a]; 
				(*P2)[b] = Q[b]; 
				C2 = region_code (a, b, P2, xmin, xmax, ymin, ymax);
			}
			edgePos = 2;
		}

	}
	return edgePos;
}
