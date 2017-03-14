#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "Addons.h"

#ifndef ___GeoMatk2___
#define ___GeoMatk2___

	// constant data
	const float d1=45;
	const float a1=93;
	const float d2=210;
	const float d4=210;
	const float d5=170;
	const float angle90=90;

	// internal calculations
	
	float convergenceCartesian(Coordinates temp1, Coordinates temp2);
	float convergenceCylindrical(Coordinates temp1, Coordinates temp2);
	
	Coordinates joints2cylin(Coordinates temp);
	Coordinates cylin2cartes(Coordinates temp);
	Coordinates joints2cartes(Coordinates temp);
	Coordinates cartes2cylin(Coordinates temp);
	Coordinates cartes2joints(Coordinates cartes_target, Coordinates joints, float accuracy);
	Coordinates cylin2joints(Coordinates cylin_target, Coordinates joints, float accuracy);

#endif //___GeoMatk2___

