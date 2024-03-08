#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <wchar.h>
#include <stdbool.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/thread.h>
#include <86box/fifo.h>
#include <86box/fifo8.h>
#include <86box/timer.h>
#include <86box/serial.h>
#include <86box/network.h>
#include <86box/plat_unused.h>

/* From RFC 1055. */
#define END             0300    /* indicates end of packet */
#define ESC             0333    /* indicates byte stuffing */
#define ESC_END         0334    /* ESC ESC_END means END data byte */
#define ESC_ESC         0335    /* ESC ESC_ESC means ESC data byte */

typedef enum ResTypes {
	ResNONE,
	ResOK,
	ResERROR,
	ResCONNECT,
	ResRING,
	ResBUSY,
	ResNODIALTONE,
	ResNOCARRIER,
	ResNOANSWER
} ResTypes;

enum modem_types
{
    MODEM_TYPE_SLIP  = 1,
    MODEM_TYPE_PPP   = 2,
    MODEM_TYPE_TCPIP = 3
};

typedef enum modem_mode_t
{
    MODEM_MODE_COMMAND = 0,
    MODEM_MODE_DATA = 1
} modem_mode_t;

typedef struct modem_t
{
    uint8_t   mac[6];
    serial_t *serial;
    double    baudrate;

    modem_mode_t mode;

    uint8_t esc_character_expected;
    pc_timer_t host_to_serial_timer;
    pc_timer_t dtr_timer;

    uint8_t tx_pkt_ser_line[0x10000]; /* SLIP-encoded. */
    uint32_t tx_count;

    Fifo8 rx_data; /* Data received from the network. */
	uint8_t reg[100];

    Fifo8 data_pending; /* Data yet to be sent to the host. */

    char cmdbuf[512];
	uint32_t cmdpos;
    int plusinc, flowcontrol;
    int in_warmup;

    bool connected, ringing;
    bool echo, numericresponse;

    int doresponse;
    
    netcard_t *card;
} modem_t;

static modem_t *instance;

#define MREG_AUTOANSWER_COUNT 0
#define MREG_RING_COUNT 1
#define MREG_ESCAPE_CHAR 2
#define MREG_CR_CHAR 3
#define MREG_LF_CHAR 4
#define MREG_BACKSPACE_CHAR 5
#define MREG_GUARD_TIME 12
#define MREG_DTR_DELAY 25

static uint32_t
modem_scan_number(char **scan)
{
    char c = 0;
	uint32_t ret = 0;
	while (1) {
        c = **scan;
        if (c == 0)
            break;
		if (c >= '0' && c <= '9') {
			ret*=10;
			ret+=c-'0';
			*scan = *scan + 1;
		} else
			break;
	}
	return ret;
}

static uint8_t
modem_fetch_character(char **scan)
{
    uint8_t c = **scan;
	*scan = *scan + 1;
	return c;
}

static void
modem_speed_changed(void *priv)
{
    modem_t *dev = (modem_t *) priv;
    if (!dev)
        return;

    timer_stop(&dev->host_to_serial_timer);
    /* FIXME: do something to dev->baudrate */
    timer_on_auto(&dev->host_to_serial_timer, (1000000.0 / dev->baudrate) * 9);
#if 0
    serial_clear_fifo(dev->serial);
#endif
}

static void
modem_send_line(modem_t* modem, const char* line)
{
    fifo8_push(&modem->data_pending, modem->reg[MREG_CR_CHAR]);
    fifo8_push(&modem->data_pending, modem->reg[MREG_LF_CHAR]);
    fifo8_push_all(&modem->data_pending, (uint8_t*)line, strlen(line));
    fifo8_push(&modem->data_pending, modem->reg[MREG_CR_CHAR]);
    fifo8_push(&modem->data_pending, modem->reg[MREG_LF_CHAR]);
}


static void
modem_send_number(modem_t* modem, uint32_t val)
{
	fifo8_push(&modem->data_pending, modem->reg[MREG_CR_CHAR]);
	fifo8_push(&modem->data_pending, modem->reg[MREG_LF_CHAR]);

	fifo8_push(&modem->data_pending, val / 100 + '0');
	val = val%100;
	fifo8_push(&modem->data_pending, val / 10 + '0');
	val = val%10;
	fifo8_push(&modem->data_pending, val + '0');

	fifo8_push(&modem->data_pending, modem->reg[MREG_CR_CHAR]);
	fifo8_push(&modem->data_pending, modem->reg[MREG_LF_CHAR]);
}

static void
process_tx_packet(modem_t *modem, uint8_t *p, uint32_t len)
{
    int      received            = 0;
    uint32_t pos                 = 0;
    uint8_t *processed_tx_packet = calloc(len, 1);
    uint8_t  c                   = 0;

    while (pos < len) {
        c = p[pos];
        pos++;
        switch (c) {
            case END:
                if (received)
                    goto send_tx_packet;
                else
                    break;

            case ESC:
                {
                    c = p[pos];
                    pos++;

                    switch (c) {
                        case ESC_END:
                            c = END;
                            break;
                        case ESC_ESC:
                            c = ESC;
                            break;
                    }
                }

            default:
                if (received < len)
                    processed_tx_packet[received++] = c;
                break;
        }
    }

send_tx_packet:
    network_tx(modem->card, processed_tx_packet, received);
    return;
}

static void
modem_data_mode_process_byte(modem_t* modem, uint8_t data)
{
    if (modem->reg[MREG_ESCAPE_CHAR] <= 127) {
        if (modem->reg[MREG_ESCAPE_CHAR] == data) {
            return;
        }
    }

    if (data == END) {
        process_tx_packet(modem, modem->tx_pkt_ser_line, (uint32_t)modem->tx_count + 1ul);
    }
    else if (modem->tx_count < 0x10000)
        modem->tx_pkt_ser_line[modem->tx_count++] = data;
}

static void
host_to_modem_cb(void *priv)
{
    modem_t* modem = (modem_t*)priv;

    if ((modem->serial->type >= SERIAL_16550) && modem->serial->fifo_enabled) {
        if (fifo_get_full(modem->serial->rcvr_fifo)) {
            goto no_write_to_machine;
        }
    } else {
        if (modem->serial->lsr & 1) {
            goto no_write_to_machine;
        }
    }

    if (modem->mode == MODEM_MODE_DATA && fifo8_num_used(&modem->rx_data)) {
        serial_write_fifo(modem->serial, fifo8_pop(&modem->rx_data));
    } else if (fifo8_num_used(&modem->data_pending)) {
        serial_write_fifo(modem->serial, fifo8_pop(&modem->data_pending));
    }

no_write_to_machine:
    timer_on_auto(&modem->host_to_serial_timer, (1000000.0 / modem->baudrate) * (double)9);
}

static void
modem_write(UNUSED(serial_t *s), void *priv, uint8_t val)
{
    modem_t* modem = (modem_t*)priv;
}

void modem_send_res(modem_t* modem, const ResTypes response) {
	const char* response_str = NULL;
	uint32_t code = -1;
	switch (response) {
	case ResOK:         code = 0; response_str = "OK"; break;
	case ResCONNECT:    code = 1; response_str = "CONNECT 33600"; break;
	case ResRING:       code = 2; response_str = "RING"; break;
	case ResNOCARRIER:  code = 3; response_str = "NO CARRIER"; break;
	case ResERROR:      code = 4; response_str = "ERROR"; break;
	case ResNODIALTONE: code = 6; response_str = "NO DIALTONE"; break;
	case ResBUSY:       code = 7; response_str = "BUSY"; break;
	case ResNOANSWER:   code = 8; response_str = "NO ANSWER"; break;
	case ResNONE: return;
	}

	if (modem->doresponse != 1) {
		if (modem->doresponse == 2 && (response == ResRING ||
			response == ResCONNECT || response == ResNOCARRIER)) {
			return;
		}
		if (modem->numericresponse && code != ~0) {
			modem_send_number(modem, code);
		} else if (response_str != NULL) {
			modem_send_line(modem, response_str);
		}

		// if(CSerial::CanReceiveByte())	// very fast response
		//	if(rqueue->inuse() && CSerial::getRTS())
		//	{ uint8_t rbyte =rqueue->getb();
		//		CSerial::receiveByte(rbyte);
		//	LOG_MSG("SERIAL: Port %" PRIu8 " modem sending byte %2x back to UART2",
		//	        GetPortNumber(), rbyte);
		//	}
	}
}

void
modem_enter_idle_state(modem_t* modem)
{
    timer_disable(&modem->dtr_timer);
    modem->connected = false;
    modem->ringing = false;
    modem->mode = MODEM_MODE_COMMAND;
    modem->in_warmup = 0;
    serial_set_cts(modem->serial, 1);
    serial_set_dsr(modem->serial, 1);
    serial_set_dcd(modem->serial, 0);
    serial_set_ri(modem->serial, 0);
}

void
modem_enter_connected_state(modem_t* modem)
{
    modem_send_res(modem, ResCONNECT);
    modem->mode = MODEM_MODE_DATA;
    modem->ringing = false;
    modem->connected = true;
    serial_set_dcd(modem->serial, 1);
    serial_set_ri(modem->serial, 0);
}

void
modem_reset(modem_t* modem)
{
    modem_enter_idle_state(modem);
	modem->cmdpos = 0;
	modem->cmdbuf[0] = 0;
	modem->flowcontrol = 0;
	modem->plusinc = 0;

	memset(&modem->reg,0,sizeof(modem->reg));
	modem->reg[MREG_AUTOANSWER_COUNT] = 0;  // no autoanswer
	modem->reg[MREG_RING_COUNT]       = 1;
	modem->reg[MREG_ESCAPE_CHAR]      = '+';
	modem->reg[MREG_CR_CHAR]          = '\r';
	modem->reg[MREG_LF_CHAR]          = '\n';
	modem->reg[MREG_BACKSPACE_CHAR]   = '\b';
	modem->reg[MREG_GUARD_TIME]       = 50;
	modem->reg[MREG_DTR_DELAY]        = 5;

	modem->echo = true;
	modem->doresponse = 0;
	modem->numericresponse = false;
}

void
modem_dial(modem_t* modem, const char* str)
{
    /* TODO: Port TCP/IP support from DOSBox. */
    if (!strncmp(str, "0.0.0.0", sizeof("0.0.0.0") - 1)) {
        modem_enter_connected_state(modem);
    } else {
		modem_send_res(modem, ResNOCARRIER);
		modem_enter_idle_state(modem);
    }
}

static bool
is_next_token(const char* a, size_t N, const char *b)
{
	// Is 'b' at least as long as 'a'?
	size_t N_without_null = N - 1;
	if (strnlen(b, N) < N_without_null)
		return false;
	return (strncmp(a, b, N_without_null) == 0);
}

// https://stackoverflow.com/a/122974
char *trim(char *str)
{
    size_t len = 0;
    char *frontp = str;
    char *endp = NULL;

    if( str == NULL ) { return NULL; }
    if( str[0] == '\0' ) { return str; }

    len = strlen(str);
    endp = str + len;

    /* Move the front and back pointers to address the first non-whitespace
     * characters from each end.
     */
    while( isspace((unsigned char) *frontp) ) { ++frontp; }
    if( endp != frontp )
    {
        while( isspace((unsigned char) *(--endp)) && endp != frontp ) {}
    }

    if( frontp != str && endp == frontp )
            *str = '\0';
    else if( str + len - 1 != endp )
            *(endp + 1) = '\0';

    /* Shift the string so that it starts at str so that if it's dynamically
     * allocated, we can still free it on the returned pointer.  Note the reuse
     * of endp to mean the front of the string buffer now.
     */
    endp = str;
    if( frontp != str )
    {
            while( *frontp ) { *endp++ = *frontp++; }
            *endp = '\0';
    }

    return str;
}

static void
modem_do_command(modem_t* modem)
{
    int i = 0;
    char *scanbuf = NULL;
    modem->cmdbuf[modem->cmdpos] = 0;
    modem->cmdpos = 0;
    for (i = 0; i < sizeof(modem->cmdbuf); i++) {
        modem->cmdbuf[i] = toupper(modem->cmdbuf[i]);
    }

	/* AT command set interpretation */
	if ((modem->cmdbuf[0] != 'A') || (modem->cmdbuf[1] != 'T')) {
		modem_send_res(modem, ResERROR);
		return;
	}

    scanbuf = &modem->cmdbuf[2];

    while (1) {
        char chr = modem_fetch_character(&scanbuf);
        switch (chr) {
            case '+':
                /* None supported yet. */
                modem_send_res(modem, ResERROR);
                return;
            case 'D': { // Dial.
                char buffer[128];
                char obuffer[128];
                char * foundstr = &scanbuf[0];
                size_t i = 0;
                if (*foundstr == 'T' || *foundstr == 'P')
                    foundstr++;
                
                if ((!foundstr[0]) || (strlen(foundstr) > 253)) {
                    modem_send_res(modem, ResERROR);
                    return;
                }

                foundstr = trim(foundstr);
                if (strlen(foundstr) >= 12) {
                    // Check if supplied parameter only consists of digits
                    bool isNum = true;
                    size_t fl = strlen(foundstr);
                    for (i = 0; i < fl; i++)
                        if (foundstr[i] < '0' || foundstr[i] > '9')
                            isNum = false;
                    if (isNum) {
                        // Parameter is a number with at least 12 digits => this cannot
                        // be a valid IP/name
                        // Transform by adding dots
                        size_t j = 0;
                        const size_t foundlen = strlen(foundstr);
                        for (i = 0; i < foundlen; i++) {
                            buffer[j++] = foundstr[i];
                            // Add a dot after the third, sixth and ninth number
                            if (i == 2 || i == 5 || i == 8)
                                buffer[j++] = '.';
                            // If the string is longer than 12 digits,
                            // interpret the rest as port
                            if (i == 11 && foundlen > 12)
                                buffer[j++] = ':';
                        }
                        buffer[j] = 0;
                        foundstr = buffer;

                        // Remove Zeros from beginning of octets
                        size_t k = 0;
                        size_t foundlen2 = strlen(foundstr);
                        for (i = 0; i < foundlen2; i++) {
                            if (i == 0 && foundstr[0] == '0') continue;
                            if (i == 1 && foundstr[0] == '0' && foundstr[1] == '0') continue;
                            if (foundstr[i] == '0' && foundstr[i-1] == '.') continue;
                            if (foundstr[i] == '0' && foundstr[i-1] == '0' && foundstr[i-2] == '.') continue;
                            obuffer[k++] = foundstr[i];
                            }
                        obuffer[k] = 0;
                        foundstr = obuffer;
                    }
                }
                modem_dial(modem, foundstr);
            }
            case 'I': // Some strings about firmware
                switch (modem_scan_number(&scanbuf)) {
                case 3: modem_send_line(modem, "86Box Emulated Modem Firmware V1.00"); break;
                case 4: modem_send_line(modem, "Modem compiled for 86Box"); break;
                }
                break;
            case 'E': // Echo on/off
                switch (modem_scan_number(&scanbuf)) {
                case 0: modem->echo = false; break;
                case 1: modem->echo = true; break;
                }
                break;
            case 'V':
                switch (modem_scan_number(&scanbuf)) {
                case 0: modem->numericresponse = true; break;
                case 1: modem->numericresponse = false; break;
                }
                break;
            case 'H': // Hang up
                switch (modem_scan_number(&scanbuf)) {
                case 0:
                    if (modem->connected) {
                        modem_send_res(modem, ResNOCARRIER);
                        modem_enter_idle_state(modem);
                        return;
                    }
                    // else return ok
                }
                break;
            case 'O': // Return to data mode
                switch (modem_scan_number(&scanbuf)) {
                case 0:
                    if (modem->connected) {
                        modem->mode = MODEM_MODE_DATA;
                        return;
                    } else {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                }
                break;
            case 'T': // Tone Dial
            case 'P': // Pulse Dial
                break;
            case 'M': // Monitor
            case 'L': // Volume
                modem_scan_number(&scanbuf);
                break;
            case 'A': // Answer call
                {
                    modem_send_res(modem, ResERROR);
                    return;
                }
                return;
            case 'Z': { // Reset and load profiles
                // scan the number away, if any
                modem_scan_number(&scanbuf);
                if (modem->connected)
                    modem_send_res(modem, ResNOCARRIER);
                modem_reset(modem);
                break;
            }
            case ' ': // skip space
                break;
            case 'Q': {
                // Response options
                // 0 = all on, 1 = all off,
                // 2 = no ring and no connect/carrier in answermode
                const uint32_t val = modem_scan_number(&scanbuf);
                if (!(val > 2)) {
                    modem->doresponse = val;
                    break;
                } else {
                    modem_send_res(modem, ResERROR);
                    return;
                }
            }
        }
    }
}

void
modem_dtr_callback(serial_t* serial, int status, void *priv)
{
    modem_t *dev = (modem_t *) priv;
    if (status == 1)
        timer_disable(&dev->dtr_timer);
    else if (!timer_is_enabled(&dev->dtr_timer))
        timer_enable(&dev->dtr_timer);
}

static void
fifo8_resize_2x(Fifo8* fifo)
{
    uint32_t pos = 0;
    uint32_t size = fifo->capacity * 2;
    uint32_t used = fifo8_num_used(fifo);
    if (!used)
        return;

    uint8_t* temp_buf = calloc(fifo->capacity * 2, 1);
    if (!temp_buf) {
        fatal("net_modem: Out Of Memory!\n");
    }
    while (!fifo8_is_empty(fifo)) {
        temp_buf[pos] = fifo8_pop(fifo);
        pos++;
    }
    pos = 0;
    fifo8_destroy(fifo);
    fifo8_create(fifo, size);
    fifo8_push_all(fifo, temp_buf, used);
    free(temp_buf);
}


static int
modem_rx(void *priv, uint8_t *buf, int io_len)
{
    modem_t* modem = (modem_t*)priv;
    uint8_t c = 0;
    uint32_t i = 0;

    if ((io_len) < (fifo8_num_free(&modem->rx_data) / 2)) {
        fifo8_resize_2x(&modem->rx_data);
    }

    fifo8_push(&modem->rx_data, END);
    for (i = 0; i < io_len; i++) {
        switch (buf[i]) {
            case END:
                fifo8_push(&modem->rx_data, ESC);
                fifo8_push(&modem->rx_data, ESC_END);
                break;
            case ESC:
                fifo8_push(&modem->rx_data, ESC);
                fifo8_push(&modem->rx_data, ESC_ESC);
                break;
            default:
                fifo8_push(&modem->rx_data, buf[i]);
                break;
        }
    }
    fifo8_push(&modem->rx_data, END);
    return 1;
}

static void
modem_rcr_cb(UNUSED(struct serial_s *serial), void *priv)
{
    modem_t *dev = (modem_t *) priv;

    timer_stop(&dev->host_to_serial_timer);
    /* FIXME: do something to dev->baudrate */
    timer_on_auto(&dev->host_to_serial_timer, (1000000.0 / dev->baudrate) * (double) 9);
#if 0
    serial_clear_fifo(dev->serial);
#endif
}

/* Initialize the device for use by the user. */
static void *
modem_init(const device_t *info)
{
    modem_t* modem = (modem_t*)calloc(1, sizeof(modem_t));
    memset(modem->mac, 0xfc, 6);

    modem->card = network_attach(instance, instance->mac, modem_rx, NULL);
    return modem;
}

void modem_close(void *priv)
{
    free(priv);
}

static const device_config_t modem_config[] = {
    {
        .name = "baudrate",
        .description = "Baud Rate",
        .type = CONFIG_SELECTION,
        .default_string = "",
        .default_int = 115200,
        .file_filter = NULL,
        .spinner = { 0 },
        .selection = {
#if 0
            { .description = "256000", .value = 256000 },
            { .description = "128000", .value = 128000 },
#endif
            { .description = "115200", .value = 115200 },
            { .description =  "57600", .value =  57600 },
            { .description =  "56000", .value =  56000 },
            { .description =  "38400", .value =  38400 },
            { .description =  "19200", .value =  19200 },
            { .description =  "14400", .value =  14400 },
            { .description =   "9600", .value =   9600 },
            { .description =   "7200", .value =   7200 },
            { .description =   "4800", .value =   4800 },
            { .description =   "2400", .value =   2400 },
            { .description =   "1800", .value =   1800 },
            { .description =   "1200", .value =   1200 },
            { .description =    "600", .value =    600 },
            { .description =    "300", .value =    300 },
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

const device_t modem_device = {
    .name          = "Standard Hayes-compliant Modem",
    .flags         = 0,
    .local         = 0,
    .init          = modem_init,
    .close         = modem_close,
    .reset         = NULL,
    { .poll = NULL },
    .speed_changed = modem_speed_changed,
    .force_redraw  = NULL,
    .config        = modem_config
};
