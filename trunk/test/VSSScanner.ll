%{
#include "VSSParser.h"
#include <cstdlib>

using namespace std;


unsigned int lineNo;
%}

%x comment

%option noyywrap


DIGIT			([0-9])
SIGN			([+-]?)
INT				({SIGN}{DIGIT}+)
REAL			({INT}"."{DIGIT}+)

%%

\n				lineNo++;

[ \r\t\v\r\f]+	;   // Ignore whitespaces 

{INT}			{
					yylval.integer = strtol(yytext, (char**)NULL, 10);
					return INTEGER;
				}
{REAL}			{
					yylval.real = strtod(yytext, (char**)NULL);
					return REAL;
				}

global			return GLOBAL;

gravity			return GRAVITY;
contact-threshold return CONTACTTHRESHOLD;
angular-viscosity-slowdown return ANGULARVISCOSITYSLOWDOWN;
linear-viscosity-slowdown return LINEARVISCOSITYSLOWDOWN;
no-collision	return NOCOLLISION;
integration-intervall return INTEGRATIONINTERVALL;
integrations-per-frame return INTEGRATIONSPERFRAME;
framerate 		return FRAMERATE;



sphere			return SPHERE;
plane			return PLANE;
box				return BOX;
capsule			return CAPSULE;

position		return POSITION;
pos				return POSITION;
mass			return MASS;
number			return IDNUM;
num				return IDNUM;
orientation		return ORIENTATION;
ori				return ORIENTATION;
fixed			return FIXED;
force			return FORCE;
steady_force	return STEADYFORCE;

radius			return RADIUS;
rad 			return RADIUS;

height			return HEIGHT;

scale			return SCALE;

bounciness 		return BOUNCINESS;
friction 		return FRICTION;

balljoint		return BALLJOINT;

connect			return CONNECT;
anchor_a		return ANCHOR_A;
anchor_b		return ANCHOR_B;

"//".*$			;	// Ignore comments

"/*"				BEGIN(comment);
<comment>[^*\n]*        /* eat anything that's not a '*' */
<comment>"*"+[^*/\n]*   /* eat up '*'s not followed by '/'s */
<comment>\n             lineNo++;
<comment>"*"+"/"        BEGIN(INITIAL);


.			return yytext[0];

%%
