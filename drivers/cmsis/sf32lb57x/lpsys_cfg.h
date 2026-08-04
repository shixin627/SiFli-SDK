/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LPSYS_CFG_H
#define __LPSYS_CFG_H

typedef struct
{
    __IO uint32_t SYSCR;
    __IO uint32_t ULPMCR;
    __IO uint32_t DBGR;
    __IO uint32_t MDBGR;
    __IO uint32_t BISTCR;
    __IO uint32_t BISTR;
    __IO uint32_t ROMCR0;
    __IO uint32_t ROMCR1;
    __IO uint32_t ROMCR2;
    __IO uint32_t ROMCR3;
    __IO uint32_t ROMCR4;
    __IO uint32_t ROMCR5;
} LPSYS_CFG_TypeDef;

#define LPSYS_CFG_PINR_SIZE          (0)

/**************** Bit definition for LPSYS_CFG_SYSCR register *****************/
#define LPSYS_CFG_SYSCR_LDO_VSEL_Pos    (0U)
#define LPSYS_CFG_SYSCR_LDO_VSEL_Msk    (0x1UL << LPSYS_CFG_SYSCR_LDO_VSEL_Pos)
#define LPSYS_CFG_SYSCR_LDO_VSEL        LPSYS_CFG_SYSCR_LDO_VSEL_Msk

/**************** Bit definition for LPSYS_CFG_ULPMCR register ****************/
#define LPSYS_CFG_ULPMCR_RAM_RM_Pos     (0U)
#define LPSYS_CFG_ULPMCR_RAM_RM_Msk     (0x3UL << LPSYS_CFG_ULPMCR_RAM_RM_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RM         LPSYS_CFG_ULPMCR_RAM_RM_Msk
#define LPSYS_CFG_ULPMCR_RAM_RME_Pos    (4U)
#define LPSYS_CFG_ULPMCR_RAM_RME_Msk    (0x1UL << LPSYS_CFG_ULPMCR_RAM_RME_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RME        LPSYS_CFG_ULPMCR_RAM_RME_Msk
#define LPSYS_CFG_ULPMCR_RAM_RA_Pos     (5U)
#define LPSYS_CFG_ULPMCR_RAM_RA_Msk     (0x3UL << LPSYS_CFG_ULPMCR_RAM_RA_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RA         LPSYS_CFG_ULPMCR_RAM_RA_Msk
#define LPSYS_CFG_ULPMCR_RAM_WA_Pos     (7U)
#define LPSYS_CFG_ULPMCR_RAM_WA_Msk     (0x7UL << LPSYS_CFG_ULPMCR_RAM_WA_Pos)
#define LPSYS_CFG_ULPMCR_RAM_WA         LPSYS_CFG_ULPMCR_RAM_WA_Msk
#define LPSYS_CFG_ULPMCR_RAM_WPULSE_Pos  (10U)
#define LPSYS_CFG_ULPMCR_RAM_WPULSE_Msk  (0x7UL << LPSYS_CFG_ULPMCR_RAM_WPULSE_Pos)
#define LPSYS_CFG_ULPMCR_RAM_WPULSE     LPSYS_CFG_ULPMCR_RAM_WPULSE_Msk
#define LPSYS_CFG_ULPMCR_RAM_RDY_Pos    (15U)
#define LPSYS_CFG_ULPMCR_RAM_RDY_Msk    (0x1UL << LPSYS_CFG_ULPMCR_RAM_RDY_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RDY        LPSYS_CFG_ULPMCR_RAM_RDY_Msk
#define LPSYS_CFG_ULPMCR_ROM_RM_Pos     (16U)
#define LPSYS_CFG_ULPMCR_ROM_RM_Msk     (0x3UL << LPSYS_CFG_ULPMCR_ROM_RM_Pos)
#define LPSYS_CFG_ULPMCR_ROM_RM         LPSYS_CFG_ULPMCR_ROM_RM_Msk
#define LPSYS_CFG_ULPMCR_ROM_RME_Pos    (20U)
#define LPSYS_CFG_ULPMCR_ROM_RME_Msk    (0x1UL << LPSYS_CFG_ULPMCR_ROM_RME_Pos)
#define LPSYS_CFG_ULPMCR_ROM_RME        LPSYS_CFG_ULPMCR_ROM_RME_Msk

/***************** Bit definition for LPSYS_CFG_DBGR register *****************/
#define LPSYS_CFG_DBGR_LP2HP_NMI_Pos    (28U)
#define LPSYS_CFG_DBGR_LP2HP_NMI_Msk    (0x1UL << LPSYS_CFG_DBGR_LP2HP_NMI_Pos)
#define LPSYS_CFG_DBGR_LP2HP_NMI        LPSYS_CFG_DBGR_LP2HP_NMI_Msk
#define LPSYS_CFG_DBGR_HP2LP_NMIE_Pos   (29U)
#define LPSYS_CFG_DBGR_HP2LP_NMIE_Msk   (0x1UL << LPSYS_CFG_DBGR_HP2LP_NMIE_Pos)
#define LPSYS_CFG_DBGR_HP2LP_NMIE       LPSYS_CFG_DBGR_HP2LP_NMIE_Msk
#define LPSYS_CFG_DBGR_HP2LP_NMIF_Pos   (30U)
#define LPSYS_CFG_DBGR_HP2LP_NMIF_Msk   (0x1UL << LPSYS_CFG_DBGR_HP2LP_NMIF_Pos)
#define LPSYS_CFG_DBGR_HP2LP_NMIF       LPSYS_CFG_DBGR_HP2LP_NMIF_Msk
#define LPSYS_CFG_DBGR_READY_Pos        (31U)
#define LPSYS_CFG_DBGR_READY_Msk        (0x1UL << LPSYS_CFG_DBGR_READY_Pos)
#define LPSYS_CFG_DBGR_READY            LPSYS_CFG_DBGR_READY_Msk

/**************** Bit definition for LPSYS_CFG_MDBGR register *****************/
#define LPSYS_CFG_MDBGR_LS_RAM0_Pos     (0U)
#define LPSYS_CFG_MDBGR_LS_RAM0_Msk     (0x1UL << LPSYS_CFG_MDBGR_LS_RAM0_Pos)
#define LPSYS_CFG_MDBGR_LS_RAM0         LPSYS_CFG_MDBGR_LS_RAM0_Msk
#define LPSYS_CFG_MDBGR_LS_RAM1_Pos     (1U)
#define LPSYS_CFG_MDBGR_LS_RAM1_Msk     (0x1UL << LPSYS_CFG_MDBGR_LS_RAM1_Pos)
#define LPSYS_CFG_MDBGR_LS_RAM1         LPSYS_CFG_MDBGR_LS_RAM1_Msk
#define LPSYS_CFG_MDBGR_LS_ROM_Pos      (2U)
#define LPSYS_CFG_MDBGR_LS_ROM_Msk      (0x1UL << LPSYS_CFG_MDBGR_LS_ROM_Pos)
#define LPSYS_CFG_MDBGR_LS_ROM          LPSYS_CFG_MDBGR_LS_ROM_Msk
#define LPSYS_CFG_MDBGR_PD_ROM_Pos      (3U)
#define LPSYS_CFG_MDBGR_PD_ROM_Msk      (0x1UL << LPSYS_CFG_MDBGR_PD_ROM_Pos)
#define LPSYS_CFG_MDBGR_PD_ROM          LPSYS_CFG_MDBGR_PD_ROM_Msk
#define LPSYS_CFG_MDBGR_DIS_ROM_Pos     (4U)
#define LPSYS_CFG_MDBGR_DIS_ROM_Msk     (0x1UL << LPSYS_CFG_MDBGR_DIS_ROM_Pos)
#define LPSYS_CFG_MDBGR_DIS_ROM         LPSYS_CFG_MDBGR_DIS_ROM_Msk

/**************** Bit definition for LPSYS_CFG_BISTCR register ****************/
#define LPSYS_CFG_BISTCR_BIST_MODE_Pos  (0U)
#define LPSYS_CFG_BISTCR_BIST_MODE_Msk  (0x1UL << LPSYS_CFG_BISTCR_BIST_MODE_Pos)
#define LPSYS_CFG_BISTCR_BIST_MODE      LPSYS_CFG_BISTCR_BIST_MODE_Msk
#define LPSYS_CFG_BISTCR_BIST_DONE_Pos  (1U)
#define LPSYS_CFG_BISTCR_BIST_DONE_Msk  (0x1UL << LPSYS_CFG_BISTCR_BIST_DONE_Pos)
#define LPSYS_CFG_BISTCR_BIST_DONE      LPSYS_CFG_BISTCR_BIST_DONE_Msk
#define LPSYS_CFG_BISTCR_BIST_FAIL_Pos  (2U)
#define LPSYS_CFG_BISTCR_BIST_FAIL_Msk  (0x1UL << LPSYS_CFG_BISTCR_BIST_FAIL_Pos)
#define LPSYS_CFG_BISTCR_BIST_FAIL      LPSYS_CFG_BISTCR_BIST_FAIL_Msk

/**************** Bit definition for LPSYS_CFG_BISTR register *****************/
#define LPSYS_CFG_BISTR_BIST_FAIL_ROM_Pos  (0U)
#define LPSYS_CFG_BISTR_BIST_FAIL_ROM_Msk  (0x3FUL << LPSYS_CFG_BISTR_BIST_FAIL_ROM_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_ROM   LPSYS_CFG_BISTR_BIST_FAIL_ROM_Msk
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM_Pos  (6U)
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM_Msk  (0x3UL << LPSYS_CFG_BISTR_BIST_FAIL_RAM_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM   LPSYS_CFG_BISTR_BIST_FAIL_RAM_Msk
#define LPSYS_CFG_BISTR_BIST_FAIL_RFC_Pos  (8U)
#define LPSYS_CFG_BISTR_BIST_FAIL_RFC_Msk  (0x1UL << LPSYS_CFG_BISTR_BIST_FAIL_RFC_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_RFC   LPSYS_CFG_BISTR_BIST_FAIL_RFC_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR0 register ****************/
#define LPSYS_CFG_ROMCR0_SIG_Pos        (0U)
#define LPSYS_CFG_ROMCR0_SIG_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR0_SIG_Pos)
#define LPSYS_CFG_ROMCR0_SIG            LPSYS_CFG_ROMCR0_SIG_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR1 register ****************/
#define LPSYS_CFG_ROMCR1_SIG_Pos        (0U)
#define LPSYS_CFG_ROMCR1_SIG_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR1_SIG_Pos)
#define LPSYS_CFG_ROMCR1_SIG            LPSYS_CFG_ROMCR1_SIG_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR2 register ****************/
#define LPSYS_CFG_ROMCR2_SIG_Pos        (0U)
#define LPSYS_CFG_ROMCR2_SIG_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR2_SIG_Pos)
#define LPSYS_CFG_ROMCR2_SIG            LPSYS_CFG_ROMCR2_SIG_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR3 register ****************/
#define LPSYS_CFG_ROMCR3_SIG_Pos        (0U)
#define LPSYS_CFG_ROMCR3_SIG_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR3_SIG_Pos)
#define LPSYS_CFG_ROMCR3_SIG            LPSYS_CFG_ROMCR3_SIG_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR4 register ****************/
#define LPSYS_CFG_ROMCR4_SIG_Pos        (0U)
#define LPSYS_CFG_ROMCR4_SIG_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR4_SIG_Pos)
#define LPSYS_CFG_ROMCR4_SIG            LPSYS_CFG_ROMCR4_SIG_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR5 register ****************/
#define LPSYS_CFG_ROMCR5_SIG_Pos        (0U)
#define LPSYS_CFG_ROMCR5_SIG_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR5_SIG_Pos)
#define LPSYS_CFG_ROMCR5_SIG            LPSYS_CFG_ROMCR5_SIG_Msk

#endif
