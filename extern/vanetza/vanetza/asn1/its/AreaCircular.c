/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "asn1/TR103562v211.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "AreaCircular.h"

asn_TYPE_member_t asn_MBR_AreaCircular_1[] = {
	{ ATF_POINTER, 1, offsetof(struct AreaCircular, nodeCenterPoint),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OffsetPoint,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"nodeCenterPoint"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct AreaCircular, radius),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Radius,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"radius"
		},
};
static const int asn_MAP_AreaCircular_oms_1[] = { 0 };
static const ber_tlv_tag_t asn_DEF_AreaCircular_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_AreaCircular_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* nodeCenterPoint */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* radius */
};
asn_SEQUENCE_specifics_t asn_SPC_AreaCircular_specs_1 = {
	sizeof(struct AreaCircular),
	offsetof(struct AreaCircular, _asn_ctx),
	asn_MAP_AreaCircular_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_AreaCircular_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_AreaCircular = {
	"AreaCircular",
	"AreaCircular",
	&asn_OP_SEQUENCE,
	asn_DEF_AreaCircular_tags_1,
	sizeof(asn_DEF_AreaCircular_tags_1)
		/sizeof(asn_DEF_AreaCircular_tags_1[0]), /* 1 */
	asn_DEF_AreaCircular_tags_1,	/* Same as above */
	sizeof(asn_DEF_AreaCircular_tags_1)
		/sizeof(asn_DEF_AreaCircular_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_AreaCircular_1,
	2,	/* Elements count */
	&asn_SPC_AreaCircular_specs_1	/* Additional specs */
};

