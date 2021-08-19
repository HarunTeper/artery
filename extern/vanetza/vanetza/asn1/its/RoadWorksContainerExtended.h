/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DENM-PDU-Descriptions"
 * 	found in "asn1/EN302637-3v131-DENM.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_RoadWorksContainerExtended_H_
#define	_RoadWorksContainerExtended_H_


#include "asn_application.h"

/* Including external dependencies */
#include "LightBarSirenInUse.h"
#include "SpeedLimit.h"
#include "TrafficRule.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ClosedLanes;
struct RestrictedTypes;
struct CauseCode;
struct ItineraryPath;
struct DeltaReferencePosition;
struct ReferenceDenms;

/* RoadWorksContainerExtended */
typedef struct RoadWorksContainerExtended {
	LightBarSirenInUse_t	*lightBarSirenInUse;	/* OPTIONAL */
	struct ClosedLanes	*closedLanes;	/* OPTIONAL */
	struct RestrictedTypes	*restriction;	/* OPTIONAL */
	SpeedLimit_t	*speedLimit;	/* OPTIONAL */
	struct CauseCode	*incidentIndication;	/* OPTIONAL */
	struct ItineraryPath	*recommendedPath;	/* OPTIONAL */
	struct DeltaReferencePosition	*startingPointSpeedLimit;	/* OPTIONAL */
	TrafficRule_t	*trafficFlowRule;	/* OPTIONAL */
	struct ReferenceDenms	*referenceDenms;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadWorksContainerExtended_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadWorksContainerExtended;
extern asn_SEQUENCE_specifics_t asn_SPC_RoadWorksContainerExtended_specs_1;
extern asn_TYPE_member_t asn_MBR_RoadWorksContainerExtended_1[9];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ClosedLanes.h"
#include "RestrictedTypes.h"
#include "CauseCode.h"
#include "ItineraryPath.h"
#include "DeltaReferencePosition.h"
#include "ReferenceDenms.h"

#endif	/* _RoadWorksContainerExtended_H_ */
#include "asn_internal.h"
