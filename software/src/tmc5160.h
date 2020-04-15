/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * tmc5160.h: Driver for TMC5160
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef TMC5160_H
#define TMC5160_H

#include "configs/config.h"
#include "bricklib2/hal/spi_fifo/spi_fifo.h"
#include "bricklib2/utility/led_flicker.h"

#define TMC5160_NUM_REGISTERS 116

// ****************** TMC5160 REGISTERS *****************
// R is read-only / W is write-only / R+C is clear upon read

// General Configuration (0x00...0x0F)
#define TMC5160_REG_GCONF         0x00 // RW
#define TMC5160_REG_GSTAT         0x01 // R+C
#define TMC5160_REG_IFCNT         0x02 // R
#define TMC5160_REG_SLAVECONF     0x03 //  W
#define TMC5160_REG_IOEN          0x04 // R
#define TMC5160_REG_X_COMPARE     0x05 //  W
#define TMC5160_REG_OTP_PROG      0x06 //  W
#define TMC5160_REG_OTP_READ      0x07 // R
#define TMC5160_REG_FACTORY_CONF  0x08 // RW
#define TMC5160_REG_SHORT_CONF    0x09 //  W
#define TMC5160_REG_DRV_CONF      0x0A //  W
#define TMC5160_REG_GLOBAL_SCALER 0x0B //  W
#define TMC5160_REG_OFFSET_READ   0x0C // R

// Velocity dependent driver feature control (0x10...0x1F)
#define TMC5160_REG_IHOLD_IRUN    0x10 //  W
#define TMC5160_REG_TPOWERDOWN    0x11 //  W
#define TMC5160_REG_TSTEP         0x12 // R
#define TMC5160_REG_TPWMTHRS      0x13 //  W
#define TMC5160_REG_TCOOLTHRS     0x14 //  W
#define TMC5160_REG_THIGH         0x15 //  W

// Ramp Generator Motion control (0x20..0x2D)
#define TMC5160_REG_RAMPMODE      0x20 // RW
#define TMC5160_REG_XACTUAL       0x21 // RW
#define TMC5160_REG_VACTUAL       0x22 // R
#define TMC5160_REG_VSTART        0x23 //  W
#define TMC5160_REG_A1            0x24 //  W
#define TMC5160_REG_V1            0x25 //  W
#define TMC5160_REG_AMAX          0x26 //  W
#define TMC5160_REG_VMAX          0x27 //  W
#define TMC5160_REG_DMAX          0x28 //  W
#define TMC5160_REG_D1            0x2A //  W
#define TMC5160_REG_VSTOP         0x2B //  W
#define TMC5160_REG_TZEROWAIT     0x2C //  W
#define TMC5160_REG_XTARGET       0x2D // RW

// Ramp Generator Driver Feature Control (0x33..0x36)
#define TMC5160_REG_VDCMIN        0x33 //  W
#define TMC5160_REG_SW_MODE       0x34 // RW
#define TMC5160_REG_RAMP_STAT     0x35 // R+WC
#define TMC5160_REG_XLATCH        0x36 // R

// Encoder control (0x38..0x3D)
#define TMC5160_REG_ENCMODE       0x38 // RW
#define TMC5160_REG_X_ENC         0x39 // RW
#define TMC5160_REG_ENC_CONST     0x3A //  W
#define TMC5160_REG_ENC_STATUS    0x3B // R+WC
#define TMC5160_REG_ENC_LATCH     0x3C // R
#define TMC5160_REG_ENC_DEVIATION 0x3D //  W

// Micro stepping control (0x60...0x6B)
#define TMC5160_REG_MSLUT0        0x60 //  W
#define TMC5160_REG_MSLUT1        0x61 //  W
#define TMC5160_REG_MSLUT2        0x62 //  W
#define TMC5160_REG_MSLUT3        0x63 //  W
#define TMC5160_REG_MSLUT4        0x64 //  W
#define TMC5160_REG_MSLUT5        0x65 //  W
#define TMC5160_REG_MSLUT6        0x66 //  W
#define TMC5160_REG_MSLUT7        0x67 //  W
#define TMC5160_REG_MSLUTSEL      0x68 //  W
#define TMC5160_REG_MSLUTSTART    0x69 //  W
#define TMC5160_REG_MSCNT         0x6A // R
#define TMC5160_REG_MSCURACT      0x6B // R

// Driver (0x6C...0x7F)
#define TMC5160_REG_CHOPCONF      0x6C // RW
#define TMC5160_REG_COOLCONF      0x6D //  W
#define TMC5160_REG_DCCTRL        0x6E //  W
#define TMC5160_REG_DRV_STATUS    0x6F // R
#define TMC5160_REG_PWMCONF       0x70 //  W
#define TMC5160_REG_PWM_SCALE     0x71 // R
#define TMC5160_REG_PWM_AUTO      0x72 // R
#define TMC5160_REG_LOST_STEPS    0x73 // R

#define TMC5160_REG_MSLUT_NUM     8
#define TMC5160_READ              0
#define TMC5160_WRITE             0x80

typedef union {
	struct {
		uint32_t recalibrate:1;
		uint32_t faststandstill:1;
		uint32_t en_pwm_mode:1;
		uint32_t multistep_filt:1;
		uint32_t shaft:1;
		uint32_t diag0_error:1;
		uint32_t diag0_otpw:1;
		uint32_t diag0_stall:1;
		uint32_t diag1_stall:1;
		uint32_t diag1_index:1;
		uint32_t diag1_diag1_onstate:1;
		uint32_t diag1_steps_skipped:1;
		uint32_t diag0_int_pushpull:1;
		uint32_t diag1_poscomp_pushpull:1;
		uint32_t small_hysteresis:1;
		uint32_t stop_enable:1;
		uint32_t direct_mode:1;
		uint32_t test_mode:1;
	} bit;
	uint32_t reg;
} TMC5160RegGCONF;

typedef union {
	struct {
		uint32_t reset:1;
		uint32_t drv_err:1;
		uint32_t uv_cp:1;
	} bit;
	uint32_t reg;
} TMC5160RegGSTAT;

typedef union {
	struct {
		uint32_t count:8;
	} bit;
	uint32_t reg;
} TMC5160RegIFCNT;

typedef union {
	struct {
		uint32_t saveaddr:8;
		uint32_t senddelay:4;
	} bit;
	uint32_t reg;
} TMC5160RegSLAVECONF;

typedef union {
	struct {
		uint32_t refl_step:1;
		uint32_t refr_dir:1;
		uint32_t encb_dcen_cfg4:1;
		uint32_t enca_dcin_cfg5:1;
		uint32_t drv_enn:1;
		uint32_t enc_n_dco_cfg6:1;
		uint32_t sd_mode:1;
		uint32_t swcomp_in:1;
		uint32_t :16;
		uint32_t version:8;
	} bit;
	uint32_t reg;
} TMC5160RegIOEN;

typedef union {
	struct {
		uint32_t compare:32;
	} bit;
	uint32_t reg;
} TMC5160RegX_COMPARE;

typedef union {
	struct {
		uint32_t otpbit:3;
		uint32_t :1;
		uint32_t otpbyte:2;
		uint32_t :2;
		uint32_t otpmagic:8;
	} bit;
	uint32_t reg;
} TMC5160RegOTP_PROG;

typedef union {
	struct {
		uint32_t otp_fclktrim:5;
		uint32_t otp_s2_level:1;
		uint32_t otp_bbm:1;
		uint32_t otp_tbl:1;
	} bit;
	uint32_t reg;
} TMC5160RegOTP_READ;

typedef union {
	struct {
		uint32_t fclktrim:5;
	} bit;
	uint32_t reg;
} TMC5160RegFACTORY_CONF;

typedef union {
	struct {
		uint32_t s2vs_level:4;
		uint32_t :4;
		uint32_t s2g_level:4;
		uint32_t :4;
		uint32_t shortfilter:4;
		uint32_t shortdelay:1;
	} bit;
	uint32_t reg;
} TMC5160RegSHORT_CONF;

typedef union {
	struct {
		uint32_t bbmtime:4;
		uint32_t :4;
		uint32_t bbmclks:4;
		uint32_t :4;
		uint32_t otselect:2;
		uint32_t drvstrength:2;
		uint32_t filt_isense:2;
	} bit;
	uint32_t reg;
} TMC5160RegDRV_CONF;

typedef union {
	struct {
		uint32_t global_scaler:8;
	} bit;
	uint32_t reg;
} TMC5160RegGLOBAL_SCALER;

typedef union {
	struct {
		uint32_t phase_b:8;
		uint32_t phase_a:8;
	} bit;
	uint32_t reg;
} TMC5160RegOFFSET_READ;

typedef union {
	struct {
		uint32_t ihold:5;
		uint32_t :3;
		uint32_t irun:5;
		uint32_t :3;
		uint32_t ihold_delay:4;
	} bit;
	uint32_t reg;
} TMC5160RegIHOLD_IRUN;

typedef union {
	struct {
		uint32_t tpowerdown:8;
	} bit;
	uint32_t reg;
} TMC5160RegTPOWERDOWN;

typedef union {
	struct {
		uint32_t time:20;
	} bit;
	uint32_t reg;
} TMC5160RegTSTEP;

typedef union {
	struct {
		uint32_t tpwmthrs:20;
	} bit;
	uint32_t reg;
} TMC5160RegTPWMTHRS;

typedef union {
	struct {
		uint32_t tcoolthrs:20;
	} bit;
	uint32_t reg;
} TMC5160RegTCOOLTHRS;

typedef union {
	struct {
		uint32_t thigh:20;
	} bit;
	uint32_t reg;
} TMC5160RegTHIGH;

typedef union {
	struct {
		uint32_t rampmode:2;
	} bit;
	uint32_t reg;
} TMC5160RegRAMPMODE;

typedef union {
	struct {
		uint32_t xactual:32;
	} bit;
	uint32_t reg;
} TMC5160RegXACTUAL;

typedef union {
	struct {
		uint32_t vactual:24;
	} bit;
	uint32_t reg;
} TMC5160RegVACTUAL;

typedef union {
	struct {
		uint32_t vstart:18;
	} bit;
	uint32_t reg;
} TMC5160RegVSTART;

typedef union {
	struct {
		uint32_t a1:16;
	} bit;
	uint32_t reg;
} TMC5160RegA1;

typedef union {
	struct {
		uint32_t v1:20;
	} bit;
	uint32_t reg;
} TMC5160RegV1;

typedef union {
	struct {
		uint32_t amax:16;
	} bit;
	uint32_t reg;
} TMC5160RegAMAX;

typedef union {
	struct {
		uint32_t vmax:23;
	} bit;
	uint32_t reg;
} TMC5160RegVMAX;

typedef union {
	struct {
		uint32_t dmax:16;
	} bit;
	uint32_t reg;
} TMC5160RegDMAX;

typedef union {
	struct {
		uint32_t d1:16;
	} bit;
	uint32_t reg;
} TMC5160RegD1;

typedef union {
	struct {
		uint32_t vstop:18;
	} bit;
	uint32_t reg;
} TMC5160RegVSTOP;

typedef union {
	struct {
		uint32_t tzerowait:18;
	} bit;
	uint32_t reg;
} TMC5160RegTZEROWAIT;

typedef union {
	struct {
		uint32_t xtarget:32;
	} bit;
	uint32_t reg;
} TMC5160RegXTARGET;

typedef union {
	struct {
		uint32_t velocity:23;
	} bit;
	uint32_t reg;
} TMC5160RegVDCMIN;

typedef union {
	struct {
		uint32_t stop_l_enable:1;
		uint32_t stop_r_enable:1;
		uint32_t pol_stop_l:1;
		uint32_t pol_stop_r:1;
		uint32_t swap_lr:1;
		uint32_t latch_l_active:1;
		uint32_t latch_l_inactive:1;
		uint32_t latch_r_active:1;
		uint32_t latch_r_inactive:1;
		uint32_t en_latch_encoder:1;
		uint32_t sg_stop:1;
		uint32_t en_softstop:1;
	} bit;
	uint32_t reg;
} TMC5160RegSW_MODE;

typedef union {
	struct {
		uint32_t stop_status_l:1;
		uint32_t stop_status_r:1;
		uint32_t status_latch_l:1;
		uint32_t status_latch_r:1;
		uint32_t event_stop_l:1;
		uint32_t event_stop_r:1;
		uint32_t event_stop_sg:1;
		uint32_t event_pos_reached:1;
		uint32_t velocity_reached:1;
		uint32_t position_reached:1;
		uint32_t vzero:1;
		uint32_t t_zerowait_active:1;
		uint32_t second_move:1;
		uint32_t status_sg:1;
	} bit;
	uint32_t reg;
} TMC5160RegRAMP_STAT;

typedef union {
	struct {
		uint32_t xlatch:32;
	} bit;
	uint32_t reg;
} TMC5160RegXLATCH;

typedef union {
	struct {
		uint32_t pol_a:1;
		uint32_t pol_b:1;
		uint32_t pol_n:1;
		uint32_t ingore_ab:1;
		uint32_t clr_cont:1;
		uint32_t clr_once:1;
		uint32_t pos_edge:1;
		uint32_t neg_edge:1;
		uint32_t clr_enc_x:1;
		uint32_t latch_x_act:1;
		uint32_t enc_sel_decimal:1;
	} bit;
	uint32_t reg;
} TMC5160RegENCMODE;

typedef union {
	struct {
		uint32_t x_enc:32;
	} bit;
	uint32_t reg;
} TMC5160RegX_ENC;

typedef union {
	struct {
		uint32_t enc_const:32;
	} bit;
	uint32_t reg;
} TMC5160RegENC_CONST;

typedef union {
	struct {
		uint32_t enc_status:2;
	} bit;
	uint32_t reg;
} TMC5160RegENC_STATUS;

typedef union {
	struct {
		uint32_t enc_latch:32;
	} bit;
	uint32_t reg;
} TMC5160RegENC_LATCH;

typedef union {
	struct {
		uint32_t enc_deviation:20;
	} bit;
	uint32_t reg;
} TMC5160RegENC_DEVIATION;

typedef union {
	struct {
		uint32_t table_entry:32;
	} bit;
	uint32_t reg;
} TMC5160RegMSLUT;

typedef union {
	struct {
		uint32_t w0:2;
		uint32_t w1:2;
		uint32_t w2:2;
		uint32_t w3:2;
		uint32_t x1:8;
		uint32_t x2:8;
		uint32_t x3:8;
	} bit;
	uint32_t reg;
} TMC5160RegMSLUTSEL;

typedef union {
	struct {
		uint32_t start_sin:8;
		uint32_t :8;
		uint32_t start_sin90:8;
	} bit;
	uint32_t reg;
} TMC5160RegMSLUTSTART;

typedef union {
	struct {
		uint32_t counter:10;
	} bit;
	uint32_t reg;
} TMC5160RegMSCNT;

typedef union {
	struct {
		uint32_t cuar_a:9;
		uint32_t :7;
		uint32_t cuar_b:9;
	} bit;
	uint32_t reg;
} TMC5160RegMSCURACT;

typedef union {
	struct {
		uint32_t toff:4;
		uint32_t hstrt:3;
		uint32_t hend:4;
		uint32_t fd3:1;
		uint32_t disfdcc:1;
		uint32_t :1; // Set to 0
		uint32_t chm:1;
		uint32_t tbl:2;
		uint32_t :1; // Set to 0
		uint32_t vhighfs:1;
		uint32_t vhighchm:1;
		uint32_t tpfd:4;
		uint32_t mres:4;
		uint32_t intpol:1;
		uint32_t dedge:1;
		uint32_t diss2g:1;
		uint32_t diss2vs:1;
	} bit;
	uint32_t reg;
} TMC5160RegCHOPCONF;

typedef union {
	struct {
		uint32_t semin:4;
		uint32_t :1; // Set to 0
		uint32_t seup:2;
		uint32_t :1; // Set to 0
		uint32_t semax:4;
		uint32_t :1; // Set to 0
		uint32_t sedn:2;
		uint32_t seimin:1;
		uint32_t sgt:7;
		uint32_t :1; // Set to 0
		uint32_t sfilt:1;
		uint32_t :1; // Set to 0
	} bit;
	uint32_t reg;
} TMC5160RegCOOLCONF;

typedef union {
	struct {
		uint32_t dc_time:10;
		uint32_t :7;
		uint32_t dc_sg:8;
	} bit;
	uint32_t reg;
} TMC5160RegDCCTRL;

typedef union {
	struct {
		uint32_t sg_result:10;
		uint32_t :2;
		uint32_t s2vsa:1;
		uint32_t s2vsb:1;
		uint32_t stealth:1;
		uint32_t fsactive:1;
		uint32_t cs_actual:5;
		uint32_t :3;
		uint32_t stall_guard:1;
		uint32_t ot:1;
		uint32_t otpw:1;
		uint32_t s2ga:1;
		uint32_t s2gb:1;
		uint32_t ola:1;
		uint32_t olb:1;
		uint32_t stst:1;
	} bit;
	uint32_t reg;
} TMC5160RegDRV_STATUS;

typedef union {
	struct {
		uint32_t pwm_ofs:8;
		uint32_t pwm_grad:8;
		uint32_t pwm_freq:2;
		uint32_t pwm_autoscale:1;
		uint32_t pwm_autograd:1;
		uint32_t freewheel:2;
		uint32_t :2;
		uint32_t pwm_reg:4;
		uint32_t pwm_lim:4;
	} bit;
	uint32_t reg;
} TMC5160RegPWMCONF;

typedef union {
	struct {
		uint32_t amplitude_scalar:8;
	} bit;
	uint32_t reg;
} TMC5160RegPWM_SCALE;

typedef union {
	struct {
		uint32_t steps:20;
	} bit;
	uint32_t reg;
} TMC5160RegLOST_STEPS;

typedef union {
	struct {
		// General
		TMC5160RegGCONF gconf;                   // 0x00
		TMC5160RegGSTAT gstat;                   // 0x01
		TMC5160RegIFCNT ifcnt;                   // 0x02
		TMC5160RegSLAVECONF slaveconf;           // 0x03
		TMC5160RegIOEN ioen;                     // 0x04
		TMC5160RegX_COMPARE x_compare;           // 0x05
		TMC5160RegOTP_PROG otp_prog;             // 0x06
		TMC5160RegOTP_READ otp_read;             // 0x07
		TMC5160RegFACTORY_CONF factory_conf;     // 0x08
		TMC5160RegSHORT_CONF short_conf;         // 0x09
		TMC5160RegDRV_CONF drv_conf;             // 0x0A
		TMC5160RegGLOBAL_SCALER global_scaler;   // 0x0B
		TMC5160RegOFFSET_READ offset_read;       // 0x0C
		uint32_t unused1[3];                     // 0x0D-0x0F

		// Velocity
		TMC5160RegIHOLD_IRUN ihold_irun;         // 0x10
		TMC5160RegTPOWERDOWN tpowerdown;         // 0x11
		TMC5160RegTSTEP tstep;                   // 0x12
		TMC5160RegTPWMTHRS tpwmthrs;             // 0x13
		TMC5160RegTCOOLTHRS tcoolthrs;           // 0x14
		TMC5160RegTHIGH thigh;                   // 0x15
		uint32_t unused2[10];                    // 0x16-0x1F

		// Ramp Generator Motion
		TMC5160RegRAMPMODE rampmode;             // 0x20
		TMC5160RegXACTUAL xactual;               // 0x21
		TMC5160RegVACTUAL vactual;               // 0x22
		TMC5160RegVSTART vstart;                 // 0x23
		TMC5160RegA1 a1;                         // 0x24
		TMC5160RegV1 v1;                         // 0x25
		TMC5160RegAMAX amax;                     // 0x26
		TMC5160RegVMAX vmax;                     // 0x27
		TMC5160RegDMAX dmax;                     // 0x28
		uint32_t unused3;                        // 0x29
		TMC5160RegD1 d1;                         // 0x2A
		TMC5160RegVSTOP vstop;                   // 0x2B
		TMC5160RegTZEROWAIT tzerowait;           // 0x2C
		TMC5160RegXTARGET xtarget;               // 0x2D 
		uint32_t unused4[5];                     // 0x2F-0x32

		// Ramp Generator Driver
		TMC5160RegVDCMIN vdcmin;                 // 0x33
		TMC5160RegSW_MODE sw_mode;               // 0x34
		TMC5160RegRAMP_STAT ramp_stat;           // 0x35
		TMC5160RegXLATCH xlatch;                 // 0x36
		uint32_t unused5;                        // 0x37

		// Encoder
		TMC5160RegENCMODE encmode;               // 0x38
		TMC5160RegX_ENC x_enc;                   // 0x39
		TMC5160RegENC_CONST enc_const;           // 0x3A
		TMC5160RegENC_STATUS enc_status;         // 0x3B
		TMC5160RegENC_LATCH enc_latch;           // 0x3C
		TMC5160RegENC_DEVIATION enc_deviation;   // 0x3D
		uint32_t unused6[34];                    // 0x3F-0x5F

		// Micro Stepping
		TMC5160RegMSLUT mslut[8];                // 0x60-0x67
		TMC5160RegMSLUTSEL mslutsel;             // 0x68
		TMC5160RegMSLUTSTART mslutstart;         // 0x69
		TMC5160RegMSCNT mscnt;                   // 0x6A
		TMC5160RegMSCURACT mscuract;             // 0x6B

		// Driver
		TMC5160RegCHOPCONF chopconf;             // 0x6C
		TMC5160RegCOOLCONF coolconf;             // 0x6D
		TMC5160RegDCCTRL dcctrl;                 // 0x6E
		TMC5160RegDRV_STATUS drv_status;         // 0x6F
		TMC5160RegPWMCONF pwmconf;               // 0x70
		TMC5160RegPWM_SCALE pwm_scale;           // 0x71
		TMC5160RegLOST_STEPS lost_steps;         // 0x72
	} bits;
	uint32_t regs[TMC5160_NUM_REGISTERS];
} __attribute__((__packed__)) TMC5160Registers;



typedef struct {
	SPIFifo spi_fifo;
	TMC5160Registers registers;

	bool registers_write[TMC5160_NUM_REGISTERS];
	bool registers_read[TMC5160_NUM_REGISTERS];

	uint16_t high_level_standstill_current;
	uint16_t high_level_motor_run_current;
	uint16_t high_level_current;
	int32_t  high_level_last_steps;
	uint8_t last_status;

	uint32_t last_read_time;

	LEDFlickerState error_led_flicker_state;
	LEDFlickerState enable_led_flicker_state;
	LEDFlickerState steps_led_flicker_state;
} TMC5160;

uint32_t tmc5160_task_register_read(const uint8_t reg);
void tmc5160_task_register_write(const uint8_t reg, const uint32_t data);

void tmc5160_init(void);
void tmc5160_tick(void);

extern TMC5160 tmc5160;




#endif