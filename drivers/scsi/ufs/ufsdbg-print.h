#ifndef _UFSDBG_PRINT_H
#define _UFSDBG_PRINT_H

/* JESD223C UFSHCI Version 2.1 */
/* ufshci.h */
/*
// Phy Adapter Layer Error Codes
#define PA_ERR_LANE0	BIT(0)
#define PA_ERR_LANE1	BIT(1)
#define PA_ERR_LANE2	BIT(2)
#define PA_ERR_LANE3	BIT(3)
#define PA_ERR_GENERIC_ERROR BIT(4) - LINERESET

// Data Link Layer Error Codes
#define NAC_RECEIVED	BIT(0)
#define TCx_REPLAY_TIMER_EXPIRED BIT(1)
#define AFCx_REQUEST_TIMER_EXPIRED BIT(2)
#define FCx_PROTECTION_TIMER_EXPIRED BIT(3)
#define CRC_ERROR BIT(4)
#define RX_BUFFER_OVERFLOW BIT(5)
#define MAX_FRAME_LENGTH_EXCEEDED BIT(6)
#define WRONG_SEQUENCE_NUMBER BIT(7)
#define AFC_FRAME_SYNTAX_ERROR BIT(8)
#define NAC_FRAME_SYNTAX_ERROR BIT(9)
#define EOF_SYNTAX_ERROR BIT(10)
#define FRAME_SYNTAX_ERROR BIT(11)
#define BAD_CTRL_SYMBOL_TYPE BIT(12)
#define PA_INIT_ERROR BIT(13)
#define PA_ERROR_IND_RECEIVED BIT(14)

// Network Layer Error Codes
#define UNSUPPORTED_HEADER_TYPE BIT(0)
#define BAD_DEVICEID_ENC BIT(1)
#define LHDR_TRAP_PACKET_DROPPING BIT(2)

// Transport Layer Error Codes
#define UNSUPPORTED_HEADER_TYPE BIT(0)
#define UNKNOWN_CPORTID BIT(1)
#define NO_CONNECTION_RX BIT(2)
#define CONTROLLED_SEGMENT_DROPPING BIT(3)
#define BAD_TC BIT(4)
#define E2E_CREDIT_OVERFLOW BIT(5)
#define SAFETY_VALVE_DROPPING BIT(6)

// DME
#define GENERIC_DME_ERR BIT(0)
*/
#define SIZE_OF(a)	(sizeof(a)/sizeof(a[0]))
static const char *e_pa[] = {
	"PA_ERR_LANE0",
	"PA_ERR_LANE1",
	"PA_ERR_LANE2",
	"PA_ERR_LANE3",
	"PA_ERR_GENERIC_ERROR"
};

static const char *e_dl[] = {
	"NAC_RECEIVED",
	"TCx_REPLAY_TIMER_EXPIRED",
	"AFCx_REQUEST_TIMER_EXPIRED",
	"FCx_PROTECTION_TIMER_EXPIRED",
	"CRC_ERROR",
	"RX_BUFFER_OVERFLOW",
	"MAX_FRAME_LENGTH_EXCEEDED",
	"WRONG_SEQUENCE_NUMBER",
	"AFC_FRAME_SYNTAX_ERROR",
	"NAC_FRAME_SYNTAX_ERROR",
	"EOF_SYNTAX_ERROR",
	"FRAME_SYNTAX_ERROR",
	"BAD_CTRL_SYMBOL_TYPE",
	"PA_INIT_ERROR",
	"PA_ERROR_IND_RECEIVED"
};

static const char *e_nl[] = {
	"UNSUPPORTED_HEADER_TYPE",
	"BAD_DEVICEID_ENC",
	"LHDR_TRAP_PACKET_DROPPING"
};

static const char *e_tl[] = {
	"UNSUPPORTED_HEADER_TYPE",
	"UNKNOWN_CPORTID",
	"NO_CONNECTION_RX",
	"CONTROLLED_SEGMENT_DROPPING",
	"BAD_TC",
	"E2E_CREDIT_OVERFLOW",
	"SAFETY_VALVE_DROPPING"
};

static const char *e_dme[] = {
	"GENERIC_DME_ERR"
};

static inline const char** get_tbl(const char *name, int *size)
{
	if (!strcmp(name, "pa_err")) {
		*size = SIZE_OF(e_pa);
		return e_pa;
	} else if(!strcmp(name, "dl_err")) {
		*size = SIZE_OF(e_dl);
		return e_dl;
	} else if(!strcmp(name, "nl_err")) {
		*size = SIZE_OF(e_nl);
		return e_nl;
	} else if(!strcmp(name, "tl_err")) {
		*size = SIZE_OF(e_tl);
		return e_tl;
	} else if(!strcmp(name, "dme_err")) {
		*size = SIZE_OF(e_dme);
		return e_dme;
	} else {
		*size = 0;
		return NULL;
	}
}

static inline void print_ufs_error_spec(struct ufs_hba *hba, char *err_name, u32 err, s64 ts, int hist_index)
{
	char buf[256];
	int curr = 0;
	int i, size;
	const char **tbl = get_tbl(err_name, &size);
	if (!hba || !tbl)
		return;

	for (i=0; i < size; ++i) {
		if (err & BIT(i))
			curr += sprintf(buf+curr, "%s | ", tbl[i]);
	}
	dev_err(hba->dev, "%s[%d] = 0x%x at %lld us | %s", err_name, hist_index,
			err, ts, buf);
}

#endif
