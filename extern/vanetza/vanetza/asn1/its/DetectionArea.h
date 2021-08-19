/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "asn1/TR103562v211.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_DetectionArea_H_
#define	_DetectionArea_H_


#include "asn_application.h"

/* Including external dependencies */
#include "VehicleSensor.h"
#include "AreaRadial.h"
#include "AreaPolygon.h"
#include "AreaCircular.h"
#include "AreaEllipse.h"
#include "AreaRectangle.h"
#include "constr_CHOICE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DetectionArea_PR {
	DetectionArea_PR_NOTHING,	/* No components present */
	DetectionArea_PR_vehicleSensor,
	DetectionArea_PR_stationarySensorRadial,
	DetectionArea_PR_stationarySensorPolygon,
	DetectionArea_PR_stationarySensorCircular,
	DetectionArea_PR_stationarySensorEllipse,
	DetectionArea_PR_stationarySensorRectangle
	/* Extensions may appear below */
	
} DetectionArea_PR;

/* DetectionArea */
typedef struct DetectionArea {
	DetectionArea_PR present;
	union DetectionArea_u {
		VehicleSensor_t	 vehicleSensor;
		AreaRadial_t	 stationarySensorRadial;
		AreaPolygon_t	 stationarySensorPolygon;
		AreaCircular_t	 stationarySensorCircular;
		AreaEllipse_t	 stationarySensorEllipse;
		AreaRectangle_t	 stationarySensorRectangle;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DetectionArea_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DetectionArea;
extern asn_CHOICE_specifics_t asn_SPC_DetectionArea_specs_1;
extern asn_TYPE_member_t asn_MBR_DetectionArea_1[6];
extern asn_per_constraints_t asn_PER_type_DetectionArea_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _DetectionArea_H_ */
#include "asn_internal.h"
