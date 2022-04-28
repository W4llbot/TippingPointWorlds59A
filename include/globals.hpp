#ifndef _GLOBALS_HPP_
#define _GLOBALS_HPP_

// Motor ports
#define FLPort 3
#define MLPort 6
#define BLPort 9
#define FRPort 5
#define MRPort 7
#define BRPort 2

// Port expander
#define expanderPort 12

// Mech Ports
#define armPort 8
#define intakePort 10

// Pneumatic Ports
#define needleTilterPort 3
#define needlePort 4
#define clampPort 5
#define lTilterPort {{expanderPort, 1}}
#define rTilterPort {{expanderPort, 2}}

// Sensor ports
#define imuPort 11
#define armRotPort 1
#define clampLimitPort 6
#define encdRPort 7
#define encdSPort 1

#endif
