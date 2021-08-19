/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DENM-PDU-Descriptions"
 * 	found in "asn1/EN302637-3v131-DENM.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "ImpactReductionContainer.h"

asn_TYPE_member_t asn_MBR_ImpactReductionContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, heightLonCarrLeft),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HeightLonCarr,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"heightLonCarrLeft"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, heightLonCarrRight),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HeightLonCarr,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"heightLonCarrRight"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, posLonCarrLeft),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PosLonCarr,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"posLonCarrLeft"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, posLonCarrRight),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PosLonCarr,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"posLonCarrRight"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, positionOfPillars),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PositionOfPillars,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"positionOfPillars"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, posCentMass),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PosCentMass,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"posCentMass"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, wheelBaseVehicle),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_WheelBaseVehicle,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"wheelBaseVehicle"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, turningRadius),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TurningRadius,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"turningRadius"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, posFrontAx),
		(ASN_TAG_CLASS_CONTEXT | (8 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PosFrontAx,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"posFrontAx"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, positionOfOccupants),
		(ASN_TAG_CLASS_CONTEXT | (9 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PositionOfOccupants,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"positionOfOccupants"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, vehicleMass),
		(ASN_TAG_CLASS_CONTEXT | (10 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleMass,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"vehicleMass"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, requestResponseIndication),
		(ASN_TAG_CLASS_CONTEXT | (11 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RequestResponseIndication,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"requestResponseIndication"
		},
};
static const ber_tlv_tag_t asn_DEF_ImpactReductionContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ImpactReductionContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* heightLonCarrLeft */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* heightLonCarrRight */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* posLonCarrLeft */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* posLonCarrRight */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* positionOfPillars */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* posCentMass */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* wheelBaseVehicle */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 }, /* turningRadius */
    { (ASN_TAG_CLASS_CONTEXT | (8 << 2)), 8, 0, 0 }, /* posFrontAx */
    { (ASN_TAG_CLASS_CONTEXT | (9 << 2)), 9, 0, 0 }, /* positionOfOccupants */
    { (ASN_TAG_CLASS_CONTEXT | (10 << 2)), 10, 0, 0 }, /* vehicleMass */
    { (ASN_TAG_CLASS_CONTEXT | (11 << 2)), 11, 0, 0 } /* requestResponseIndication */
};
asn_SEQUENCE_specifics_t asn_SPC_ImpactReductionContainer_specs_1 = {
	sizeof(struct ImpactReductionContainer),
	offsetof(struct ImpactReductionContainer, _asn_ctx),
	asn_MAP_ImpactReductionContainer_tag2el_1,
	12,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ImpactReductionContainer = {
	"ImpactReductionContainer",
	"ImpactReductionContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_ImpactReductionContainer_tags_1,
	sizeof(asn_DEF_ImpactReductionContainer_tags_1)
		/sizeof(asn_DEF_ImpactReductionContainer_tags_1[0]), /* 1 */
	asn_DEF_ImpactReductionContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_ImpactReductionContainer_tags_1)
		/sizeof(asn_DEF_ImpactReductionContainer_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_ImpactReductionContainer_1,
	12,	/* Elements count */
	&asn_SPC_ImpactReductionContainer_specs_1	/* Additional specs */
};

