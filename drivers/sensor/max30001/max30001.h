/*
 * Copyright (c) 2022 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MAX30001_H_
#define ZEPHYR_DRIVERS_SENSOR_MAX30001_H_

#ifndef BIT
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
#endif

#ifndef BITS
#define BITS(start, end)             ((0xFFFFFFFFUL << (start)) & \
					(0xFFFFFFFFUL >> (31U - (uint32_t)(end))))
#endif

#define MAX30001_STATUS			0x1U
#define MAX30001_EN_INT			0x2U
#define MAX30001_EN_INT2		0x3U
#define MAX30001_MNGR_INT		0x4U
#define MAX30001_MNGR_DYN		0x5U
#define MAX30001_SW_RST			0x8U
#define MAX30001_SYNCH			0x9U
#define MAX30001_FIFO_RST		0xAU
#define MAX30001_INFO			0xFU
#define MAX30001_CNFG_GEN		0x10U
#define MAX30001_CNFG_CAL		0x12U
#define MAX30001_CNFG_EMUX		0x14U
#define MAX30001_CNFG_ECG		0x15U
#define MAX30001_CNFG_BMUX		0x17U
#define MAX30001_CNFG_BIOZ		0x18U
#define MAX30001_CNFG_PACE		0x1AU
#define MAX30001_CNFG_RTOR1		0x1DU
#define MAX30001_CNFG_RTOR2		0x1EU
#define MAX30001_ECG_FIFO_BURST		0x20U
#define MAX30001_ECG_FIFO		0x21U
#define MAX30001_BIOZ_FIFO_BURST	0x22U
#define MAX30001_BIOZ_FIFO		0x23U
#define MAX30001_RTOR			0x25U
#define MAX30001_PACE0_BURST		0x30U
#define MAX30001_PACE0__A		0x31U
#define MAX30001_PACE0__B		0x32U
#define MAX30001_PACE0__C		0x33U
#define MAX30001_PACE1_BURST		0x34U
#define MAX30001_PACE1__A		0x35U
#define MAX30001_PACE1__B		0x36U
#define MAX30001_PACE1__C		0x37U
#define MAX30001_PACE2_BURST		0x38U
#define MAX30001_PACE2__A		0x39U
#define MAX30001_PACE2__B		0x3AU
#define MAX30001_PACE2__C		0x3BU
#define MAX30001_PACE3_BURST		0x3CU
#define MAX30001_PACE3__A		0x3DU
#define MAX30001_PACE3__B		0x3EU
#define MAX30001_PACE3__C		0x3FU
#define MAX30001_PACE4_BURST		0x40U
#define MAX30001_PACE4__A		0x41U
#define MAX30001_PACE4__B		0x42U
#define MAX30001_PACE4__C		0x43U
#define MAX30001_PACE5_BURST		0x44U
#define MAX30001_PACE5__A		0x45U
#define MAX30001_PACE5__B		0x46U
#define MAX30001_PACE5__C		0x47U

/* STATUS register bits field */
#define STATUS_EINT		BIT(23)
#define STATUS_EOVF		BIT(22)
#define STATUS_FSTINT		BIT(21)
#define STATUS_DCLOFFINT	BIT(20)
#define STATUS_BINT		BIT(19)
#define STATUS_BOVF		BIT(18)
#define STATUS_BOVER		BIT(17)
#define STATUS_BUNDR		BIT(16)
#define STATUS_BCGMON		BIT(15)
#define STATUS_PINT		BIT(14)
#define STATUS_POVF		BIT(13)
#define STATUS_PEDGE		BIT(12)
#define STATUS_LONINT		BIT(11)
#define STATUS_RRINT		BIT(10)
#define STATUS_SAMP		BIT(9)
#define STATUS_PLLINT		BIT(8)
#define STATUS_BCGMP		BIT(5)
#define STATUS_BCGMN		BIT(4)
#define STATUS_LDOFF_PH		BIT(3)
#define STATUS_LDOFF_PL		BIT(2)
#define STATUS_LDOFF_NH		BIT(1)
#define STATUS_LDOFF_NL		BIT(0)

/* EN_INT register bits field */
#define EN_INT_EN_EINT		BIT(23)
#define EN_INT_EN_EOVF		BIT(22)
#define EN_INT_EN_FSTINT	BIT(21)
#define EN_INT_EN_DCLOFFINT	BIT(20)
#define EN_INT_EN_BINT		BIT(19)
#define EN_INT_EN_BOVF		BIT(18)
#define EN_INT_EN_BOVER		BIT(17)
#define EN_INT_EN_BUNDR		BIT(16)
#define EN_INT_EN_BCGMON	BIT(15)
#define EN_INT_EN_PINT		BIT(14)
#define EN_INT_EN_POVF		BIT(13)
#define EN_INT_EN_PEDGE		BIT(12)
#define EN_INT_EN_LONINT	BIT(11)
#define EN_INT_EN_RRINT		BIT(10)
#define EN_INT_EN_SAMP		BIT(9)
#define EN_INT_EN_PLLINT	BIT(8)
#define EN_INT_INTB_TYPE	BITS(0, 1)

/* EN_INT2 register bits field */
#define EN_INT2_EN_EINT		BIT(23)
#define EN_INT2_EN_EOVF		BIT(22)
#define EN_INT2_EN_FSTINT	BIT(21)
#define EN_INT2_EN_DCLOFFINT	BIT(20)
#define EN_INT2_EN_BINT		BIT(19)
#define EN_INT2_EN_BOVF		BIT(18)
#define EN_INT2_EN_BOVER	BIT(17)
#define EN_INT2_EN_BUNDR	BIT(16)
#define EN_INT2_EN_BCGMON	BIT(15)
#define EN_INT2_EN_PINT		BIT(14)
#define EN_INT2_EN_POVF		BIT(13)
#define EN_INT2_EN_PEDGE	BIT(12)
#define EN_INT2_EN_LONINT	BIT(11)
#define EN_INT2_EN_RRINT	BIT(10)
#define EN_INT2_EN_SAMP		BIT(9)
#define EN_INT2_EN_PLLINT	BIT(8)
#define EN_INT2_INTB_TYPE	BITS(0, 1)

/* MNGR_INT register bits field */
#define MNGR_INT_EFIT		BITS(19, 23)
#define MNGR_INT_BFIT		BITS(16, 18)
#define MNGR_INT_CLR_FAST	BIT(6)
#define MNGR_INT_CLR_RRINT	BITS(4, 5)
#define MNGR_INT_CLR_PEDGE	BIT(3)
#define MNGR_INT_CLR_SAMP	BIT(2)
#define MNGR_INT_SAMP_IT	BITS(0, 1)

/* MNGR_DYN register bits field */
#define MNGR_DYN_FAST		BITS(22, 23)
#define MNGR_DYN_FAST_TH	BITS(16, 21)
#define MNGR_DYN_BLOFF_HI_IT	BITS(8, 15)
#define MNGR_DYN_BLOFF_LO_IT	BITS(0, 7)

/* INFO register bits field */
#define INFO_REV_ID		BIT(16, 19)

/* CNFG_GEN register bits field */
#define CNFG_GEN_EN_ULP_LON	BITS(22, 23)
#define CNFG_GEN_FMSTR		BITS(20, 21)
#define CNFG_GEN_EN_ECG		BIT(19)
#define CNFG_GEN_EN_BIOZ	BIT(18)
#define CNFG_GEN_EN_PACE	BIT(17)
#define CNFG_GEN_EN_BLOFF	BITS(14, 15)
#define CNFG_GEN_EN_DCLOFF	BITS(12, 13)
#define CNFG_GEN_IPOL		BIT(11)
#define CNFG_GEN_IMAG		BITS(8, 10)
#define CNFG_GEN_VTH		BITS(6, 7)
#define CNFG_GEN_EN_RBIAS	BITS(4, 5)
#define CNFG_GEN_RBIASV		BITS(2, 3)
#define CNFG_GEN_RBIASP		BIT(1)
#define CNFG_GEN_RBIASN		BIT(0)

/* CNFG_CAL register bits field */
#define CNFG_CAL_EN_VCAL	BIT(22)
#define CNFG_CAL_VMODE		BIT(21)
#define CNFG_CAL_VMAG		BIT(20)
#define CNFG_CAL_FCAL		BITS(12, 14)
#define CNFG_CAL_FIFTY		BIT(11)
#define CNFG_CAL_THIGH		BITS(0, 10)

/* CNFG_EMUX register bits field */
#define CNFG_EMUX_ECG_POL	BIT(23)
#define CNFG_EMUX_ECG_OPENP	BIT(21)
#define CNFG_EMUX_ECG_OPENN	BIT(20)
#define CNFG_EMUX_ECG_CALP_SEL	BITS(18, 19)
#define CNFG_EMUX_ECG_CALN_SEL	BITS(16, 17)

/* CNFG_ECG register bits field */
#define CNFG_ECG_ECG_RATE	BITS(22, 23)
#define CNFG_ECG_ECG_GAIN	BITS(16, 17)
#define CNFG_ECG_ECG_DHPF	BIT(14)
#define CNFG_ECG_ECG_DLPF	BITS(12, 13)

/* CNFG_BMUX register bits field */
#define CNFG_BMUX_BMUX_OPENP	BIT(21)
#define CNFG_BMUX_BMUX_OPENN	BIT(20)
#define CNFG_BMUX_BMUX_CALP_SEL	BITS(18, 19)
#define CNFG_BMUX_BMUX_CALN_SEL	BITS(16, 17)
#define CNFG_BMUX_BMUX_CG_MODE	BITS(12, 13)
#define CNFG_BMUX_BMUX_EN_BIST	BIT(11)
#define CNFG_BMUX_BMUX_RNOM	BITS(8, 10)
#define CNFG_BMUX_BMUX_RMOD	BITS(4, 6)
#define CNFG_BMUX_BMUX_FBIST	BITS(0, 1)

/* CNFG_BIOZ register bits field */
#define CNFG_BIOZ_BIOZ_RATE	BIT(23)
#define CNFG_BIOZ_BIOZ_AHPF	BITS(20, 22)
#define CNFG_BIOZ_EXT_RBIAS	BIT(19)
#define CNFG_BIOZ_LN_BIOZ	BIT(18)
#define CNFG_BIOZ_BIOZ_GAIN	BITS(16, 17)
#define CNFG_BIOZ_BIOZ_DHPF	BITS(14, 15)
#define CNFG_BIOZ_BIOZ_DLPF	BITS(12, 13)
#define CNFG_BIOZ_BIOZ_FCGEN	BITS(8, 11)
#define CNFG_BIOZ_BIOZ_CGMON	BIT(7)
#define CNFG_BIOZ_BIOZ_CGMAG	BITS(4, 6)
#define CNFG_BIOZ_BIOZ_PHOFF	BITS(0, 3)

/* CNFG_PACE register bits field */
#define CNFG_PACE_PACE_POL	BIT(23)
#define CNFG_PACE_DIFF_OFF	BIT(19)
#define CNFG_PACE_PACE_GAIN	BITS(16, 18)
#define CNFG_PACE_AOUT_LBW	BIT(14)
#define CNFG_PACE_AOUT		BITS(12, 13)
#define CNFG_PACE_PACE_DACP	BITS(4, 7)
#define CNFG_PACE_PACE_DACN	BITS(0, 3)

/* CNFG_RTOR1 register bits field */
#define CNFG_RTOR1_WNDW		BITS(20, 23)
#define CNFG_RTOR1_RGAIN	BITS(16, 19)
#define CNFG_RTOR1_EN_RTOR	BIT(15)
#define CNFG_RTOR1_PAVG		BITS(12, 13)
#define CNFG_RTOR1_PTSF		BITS(8, 11)

/* CNFG_RTOR2 register bits field */
#define CNFG_RTOR2_HOFF		BITS(16, 21)
#define CNFG_RTOR2_RAVG		BITS(12, 13)
#define CNFG_RTOR2_RHSF		BITS(8, 10)

#define ULP_LON_ENABLE		(0x1U << 22)
#define ULP_LON_DISABLE		(0x0U << 22)

#define DCLOFF_DISABLE		(0x0U << 12)
#define DCLOFF_ENABLE		(0x1U << 12)

/* SYNCH register enable value */
#define SYNCH_DIN		0U

/* FIFO_RST register enable value */
#define FIFO_RST_DIN		0U

/* ECG_FIFO register bits field */
#define ECG_FIFO_PTAG		BITS(0, 2)
#define ECG_FIFO_ETAG		BITS(3, 5)
#define ECG_FIFO_SAMPLE		BITS(6, 23)

#define ECG_FIFO_ETAG_VALID	(0U << 3)	/* Valid sample */
#define ECG_FIFO_ETAG_FAST	(1U << 3)	/* Fast mode sample */
#define ECG_FIFO_ETAG_EOF	(2U << 3)	/* Last valid sample (EOF) */
#define ECG_FIFO_ETAG_FAST_EOF	(3U << 3)	/* Last fast mode valid sample (EOF) */
#define ECG_FIFO_ETAG_EMPTY	(6U << 3)	/* FIFO empty (exception */
#define ECG_FIFO_ETAG_OVERFLOW	(7U << 3)	/* FIFO overflow (exception) */

/* CNFG_GEN::IMAG offset. */
#define IMAG_OFFSET		8

/* CNFG_GEN::VTH offset. */
#define VTH_OFFSET		6

/* CNFG_ECG::ECG_RATE offset */
#define ECG_RATE_OFFSET		22

#endif /* ZEPHYR_DRIVERS_SENSOR_MAX30001_H_ */
