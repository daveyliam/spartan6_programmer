/*
Programs a Spartan 6 FPGA over JTAG using an FTDI FT232H chip.

Takes a ".bin" file as input, which can be output from ISE.
*/

#include <ftdi.h>
#include <usb.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define CLK_DIV_5_DISABLE (0x8a)
#define CLK_DIV_5_ENABLE (0x8b)
#define DATA_CLK_3_PHASE_ENABLE (0x8c)
#define DATA_CLK_3_PHASE_DISABLE (0x8d)
#define DATA_CLK_BITS (0x8e)
#define DATA_CLK_BYTES (0x8f)
#define ADAPTIVE_CLK_ENABLE (0x96)
#define ADAPTIVE_CLK_DISABLE (0x97)

#define JTAG_INSTR_ISC_DNA 		(0x30)	// (110000b)
#define JTAG_INSTR_ISC_DISABLE 	(0x16)	// (010110b)
#define JTAG_INSTR_ISC_NOOP 	(0x14)	// (010100b)
#define JTAG_INSTR_ISC_PROGRAM 	(0x11)	// (010001b)
#define JTAG_INSTR_ISC_ENABLE 	(0x10)	// (010000b)
#define JTAG_INSTR_BYPASS 		(0x3f)	// (111111b)
#define JTAG_INSTR_JSHUTDOWN 	(0x0d)	// (001101b)
#define JTAG_INSTR_JSTART 		(0x0c)	// (001100b)
#define JTAG_INSTR_JPROGRAM 	(0x0b)	// (001011b)
#define JTAG_INSTR_HIGHZ 		(0x0a)	// (001010b)
#define JTAG_INSTR_IDCODE 		(0x09)	// (001001b)
#define JTAG_INSTR_USERCODE 	(0x08)	// (001000b)
#define JTAG_INSTR_INTEST 		(0x07)	// (000111b)
#define JTAG_INSTR_PRELOAD 		(0x01)	// (000001b)
#define JTAG_INSTR_SAMPLE 		(0x01)	// (000001b)
#define JTAG_INSTR_EXTEST 		(0x0f)	// (001111b)
#define JTAG_INSTR_CFG_IN 		(0x05)	// (000101b)

#define FDATA_SIZE (16 * 1024 * 1024)
#define JTAG_BUFFER_SIZE (1024 * 1024)
#define JTAG_CHUNK_SIZE (0x8000)
#define JTAG_RECV_ATTEMPTS (20)
#define JTAG_STARTUP_DELAY (500)
#define JTAG_SHUTDOWN_DELAY (500)
#define JTAG_TCK_DIVISOR_LOW (0)

/*
 FT2232H pin definitions

 pin	| name		| mpsse function
 -------+-----------+---------------
 13		| ADBUS0	| TCK
 14		| ADBUS1	| TDI
 15		| ADBUS2	| TDO
 16		| ADBUS3	| TMS
 17		| ADBUS4	| GPIOL0
 18		| ADBUS5	| GPIOL1
 19		| ADBUS6	| GPIOL2
 20		| ADBUS7	| GPIOL3
		|			|
 21		| ACBUS0	| GPIOH0
 25		| ACBUS1	| GPIOH1
 26		| ACBUS2	| GPIOH2
 27		| ACBUS3	| GPIOH3
 28		| ACBUS4	| GPIOH4
 29		| ACBUS5	| GPIOH5
 30		| ACBUS6	| GPIOH6
 31		| ACBUS7	| GPIOH7

 0 is LSB, 7 is MSB


 *** MPSSE mode commands ***

 * SET_BITS_LOW, data, direction
    sets low port bits and direction
 * SET_BITS_HIGH, data, direction
    sets high port bits and direction
 * TCK_DIVISOR, div_low, div_high
    sets the TCK divisor to {div_high, div_low}
    rate = 60e6 / ((value + 1) * 2)
 * SEND_IMMEDIATE
    immediately send whatever data is in the ftdi device's buffer to
    the host.


 *** MPSSE shifting commands ***

 bit mode format:
  {shifting command,
   length in bits,
   data byte}

 byte mode format:
  {shifting command,
   length in bytes high,
   length in bytes low,
   data byte 0,
   ...,
   data byte n}

 shifting command byte:
  MPSSE_WRITE_NEG 0x01   Write TDI/DO on negative TCK/SK edge
  MPSSE_BITMODE   0x02   Write bits, not bytes
  MPSSE_READ_NEG  0x04   Sample TDO/DI on negative TCK/SK edge
  MPSSE_LSB       0x08   LSB first
  MPSSE_DO_WRITE  0x10   Write TDI/DO
  MPSSE_DO_READ   0x20   Read TDO/DI
  MPSSE_WRITE_TMS 0x40   Write TMS/CS

  * MPSSE_DO_WRITE and MPSSE_WRITE_TMS cannot both be set
  * cannot read and write on the same clock edge

*/


////////////////////////////////////////////////////////////////////////
// low level jtag and ftdi device functions
////////////////////////////////////////////////////////////////////////


unsigned char * jtag_buf = NULL;
int jtag_buf_i = 0;
struct ftdi_context ftdi;

int jtag_send()
{
	//int i;
	
	if(jtag_buf_i < 1)
		return 1;
	
	//printf("jtag_send %d bytes:\n", jtag_buf_i);
	//for(i = 0; (i < jtag_buf_i); i++)
	//	printf("%02x ", jtag_buf[i]);
	//printf("\n");
	
	int l = ftdi_write_data(&ftdi, jtag_buf, jtag_buf_i);
	if(l != jtag_buf_i)
	{
		//printf("error: jtag_send: ftdi_write_data returned %d (expected %d)\n", l, jtag_buf_i);
		return 1;
	}
	jtag_buf_i = 0;
	return 0;
}

int jtag_recv(unsigned char * rbuf, int n)
{
	int timeout = JTAG_RECV_ATTEMPTS, ret;
	//unsigned char * rbuf2 = rbuf;
	unsigned char buf[32];
	while(n > 0)
	{
		if(rbuf != NULL)
		{
			ret = ftdi_read_data(&ftdi, rbuf, n);
			rbuf += ret;
		}
		else
			ret = ftdi_read_data(&ftdi, buf, 32);
		
		n -= ret;
		
		if(timeout-- <= 0)
		{
			//printf("error: jtag_recv: timed out with %d byte remaining\n", n);
			break;
		}
	}
	
	//printf("jtag_recv: ");
	//while(rbuf2 < rbuf)
	//	printf("%02x ", *rbuf2++);
	//printf("\n");
	
	return n;
}

// close and deinitialize ftdi device
void jtag_close()
{
	if(jtag_buf != NULL)
		free(jtag_buf);
	jtag_buf = NULL;
	ftdi_usb_purge_buffers(&ftdi);
	ftdi_usb_reset(&ftdi);
	ftdi_usb_close(&ftdi);
	ftdi_deinit(&ftdi);
}

// initialize ftdi device for jtag
int jtag_init()
{
	int ret;
	
	if((jtag_buf = malloc(JTAG_BUFFER_SIZE)) == NULL)
	{
		printf("error: jtag_init: could not malloc jtag_buf\n");
		return 1;
	}
	
	// initialize ftdi data structure and open the ftdi device with
	// VID:PID = 0403:6014
	ftdi_init(&ftdi);
	if(ftdi_usb_open_desc(&ftdi, 0x0403, 0x6014, 0, 0) < 0)
	{
		printf("error: could not open ftdi device\n");
		ftdi_deinit(&ftdi);
		return 1;
	}
	
	// reset ftdi device
	ret = ftdi_usb_reset(&ftdi);
	
	// use interface A
	//ret += ftdi_set_interface(&ftdi, INTERFACE_A);
	
	// 1ms latency timer
	ret += ftdi_set_latency_timer(&ftdi, 1);
	
	// purge buffers
	ret += ftdi_usb_purge_buffers(&ftdi);
	
	// set bit mode to MPSSE, 0xfb bitmask sets all bits except for
	// TDO to output
	ret += ftdi_set_bitmode(&ftdi, 0x00, 0x00);
	ret += ftdi_set_bitmode(&ftdi, 0x0b, BITMODE_MPSSE);
	
	if(ret < 0)
	{
		printf("error: jtag_init: ftdi device config failed\n");
		return 1;
	}
	
	jtag_buf_i = 0;
	
	// set TMS high, TCK low, TDI low and TDO as input
	jtag_buf[jtag_buf_i++] = SET_BITS_LOW;
	jtag_buf[jtag_buf_i++] = 0x08;
	jtag_buf[jtag_buf_i++] = 0x0b;
	
	// set all pins of the high port to inputs
	jtag_buf[jtag_buf_i++] = SET_BITS_HIGH;
	jtag_buf[jtag_buf_i++] = 0x00;
	jtag_buf[jtag_buf_i++] = 0x00;
	
	// disable the divide by 5 clock prescaler
	jtag_buf[jtag_buf_i++] = CLK_DIV_5_DISABLE;
	
	// set the TCK rate to 30MHz
	jtag_buf[jtag_buf_i++] = TCK_DIVISOR;
	jtag_buf[jtag_buf_i++] = JTAG_TCK_DIVISOR_LOW;
	jtag_buf[jtag_buf_i++] = 0x00;
	
	// disable 3 phase data clocking
	jtag_buf[jtag_buf_i++] = DATA_CLK_3_PHASE_DISABLE;
	
	// disable adaptive clocking
	jtag_buf[jtag_buf_i++] = ADAPTIVE_CLK_DISABLE;
	
	// flush ftdi buffer
	jtag_buf[jtag_buf_i++] = SEND_IMMEDIATE;

	if(jtag_send())
	{
		printf("error: jtag_init: could not send initialization commands\n");
		jtag_close();
		return 1;
	}
	
	return 0;
}

#define jtag_add_send_immediate() (jtag_buf[jtag_buf_i++] = SEND_IMMEDIATE)

// go to test logic reset state
void jtag_to_tlr()
{
	// TMS: 11111
	jtag_buf[jtag_buf_i++] = MPSSE_WRITE_TMS | MPSSE_LSB | MPSSE_BITMODE | MPSSE_WRITE_NEG;
	jtag_buf[jtag_buf_i++] = 4;
	jtag_buf[jtag_buf_i++] = 0x9f;
}

// go to rti state from tlr state
void jtag_tlr_to_rti()
{
	// TMS: 0
	jtag_buf[jtag_buf_i++] = MPSSE_WRITE_TMS | MPSSE_LSB | MPSSE_BITMODE | MPSSE_WRITE_NEG;
	jtag_buf[jtag_buf_i++] = 0;
	jtag_buf[jtag_buf_i++] = 0x80;	
}

// spin in run-test-idle state for 128 * 8 TCK cycles
void jtag_rti_spin()
{
	int n;
	
	// set TMS to 0
	jtag_buf[jtag_buf_i++] = MPSSE_WRITE_TMS | MPSSE_LSB | MPSSE_BITMODE | MPSSE_WRITE_NEG;
	jtag_buf[jtag_buf_i++] = 0;
	jtag_buf[jtag_buf_i++] = 0x80;
	
	// run TCK for 128 * 8 cycles
	for(n = 0; n < 128; n++)
	{
		jtag_buf[jtag_buf_i++] = DATA_CLK_BITS;
		jtag_buf[jtag_buf_i++] = 7;
	}
	
	jtag_send();
}

// go to shift-ir state from rti state
void jtag_rti_to_shift_ir()
{
	// RTI -> SHIFT-IR
	// TMS: 0011
	jtag_buf[jtag_buf_i++] = MPSSE_WRITE_TMS | MPSSE_LSB | MPSSE_BITMODE | MPSSE_WRITE_NEG;
	jtag_buf[jtag_buf_i++] = 3;
	jtag_buf[jtag_buf_i++] = 0x83;
}

// go to shift-dr state from rti state
void jtag_rti_to_shift_dr()
{
	// RTI -> SHIFT-DR
	// TMS: 001
	jtag_buf[jtag_buf_i++] = MPSSE_WRITE_TMS | MPSSE_LSB | MPSSE_BITMODE | MPSSE_WRITE_NEG;
	jtag_buf[jtag_buf_i++] = 2;
	jtag_buf[jtag_buf_i++] = 0x81;
}

void jtag_exit1_ir_to_rti()
{
	// EXIT1-IR -> RTI
	// TMS: 01
	jtag_buf[jtag_buf_i++] = MPSSE_WRITE_TMS | MPSSE_LSB | MPSSE_BITMODE | MPSSE_WRITE_NEG;
	jtag_buf[jtag_buf_i++] = 1;
	jtag_buf[jtag_buf_i++] = 0x81;
}

void jtag_exit1_dr_to_rti()
{
	// EXIT1-DR -> RTI
	// TMS: 01
	jtag_buf[jtag_buf_i++] = MPSSE_WRITE_TMS | MPSSE_LSB | MPSSE_BITMODE | MPSSE_WRITE_NEG;
	jtag_buf[jtag_buf_i++] = 1;
	jtag_buf[jtag_buf_i++] = 0x81;
}

// add commands to jtag_buf to shift out 'n' bytes from 'tdi'.
// if do_read is set then make the command read while shifting out.
// assumes tap already in shift-dr or shift-ir state.
void jtag_shift_bytes(unsigned char * tdi, int n, int do_read)
{
	int i;
	
	// command byte
	jtag_buf[jtag_buf_i] = MPSSE_LSB;
	if(tdi != NULL)
		jtag_buf[jtag_buf_i] |= MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
	if(do_read)
		jtag_buf[jtag_buf_i] |= MPSSE_DO_READ;
	jtag_buf_i++;
	
	// two byte length
	jtag_buf[jtag_buf_i++] = ((n - 1) & 0xff);
	jtag_buf[jtag_buf_i++] = ((n - 1) >> 8) & 0xff;
	
	// data bytes if writing
	if(tdi != NULL)
		for(i = 0; i < n; i++)
			jtag_buf[jtag_buf_i++] = tdi[i];
}

// shift 'n' bits of data onto TDI
// assumes tap already in shift-dr or shift-ir state.

void jtag_shift_bits(unsigned char * tdi, int n, int do_read)
{
	// if more than one bits need to be shifted
	if(n > 1)
	{
		// command byte
		jtag_buf[jtag_buf_i] = MPSSE_BITMODE | MPSSE_LSB;
		if(tdi != NULL)
			jtag_buf[jtag_buf_i] |= MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
		if(do_read)
			jtag_buf[jtag_buf_i] |= MPSSE_DO_READ;
		jtag_buf_i++;
		
		// number of bits
		jtag_buf[jtag_buf_i++] = (n - 2);
		
		// data byte (last byte of buffer)
		if(tdi != NULL)
			jtag_buf[jtag_buf_i++] = *tdi & ((1 << (n - 1)) - 1);
	}

	// shift the final bit
	jtag_buf[jtag_buf_i] = MPSSE_WRITE_TMS | MPSSE_BITMODE | MPSSE_LSB | MPSSE_WRITE_NEG;
	if(do_read)
		jtag_buf[jtag_buf_i] |= MPSSE_DO_READ;
	jtag_buf_i++;
	
	// shift one bit
	jtag_buf[jtag_buf_i++] = 0;
	
	// MSB is value to set TDI to
	// LSB is TMS value (=1)
	if(tdi != NULL)
		jtag_buf[jtag_buf_i++] = (*tdi & (1 << (n - 1))) ? 0x81 : 0x01;
	else
		jtag_buf[jtag_buf_i++] = 0x01;
}

// receive bits from ftdi device
// combines the bits if they were transferred in separate commands
int jtag_recv_bits(unsigned char * tdo, int n)
{
	unsigned char rbuf[2];
	
	if((n < 1) || (n > 8))
		return 1;
	
	if(jtag_recv(rbuf, (n > 1) ? 2 : 1))
	{
		printf("error: jtag_read_bits: could not recv bytes\n");
		return 1;
	}
	
	// if more than one bits were shifted then we need to add the
	// final bit received to the correct position in the prior bits.
	if(n > 1)
	{
		// bits are shifted in from the left (MSB) so if less than 8
		// bits were shifted then need to shift the bits in the
		// received byte right by 8 - n bits.
		*tdo = ((rbuf[1] & 0x80) | (rbuf[0] >> 1)) >> (8 - n);
	} else
		// if only 1 bit received
		*tdo = (rbuf[0] & 0x80) >> 7;
	
	return 0;
}

// read and/or write data register
// n is length of data in bits.
// this attempts to send and receive in one transfer if possible
int jtag_dr_op(unsigned char * tdi, unsigned char * tdo, int n)
{
	int bytes_remaining, bits_remaining, chunk_length;
	int tdi_i, tdo_i;
	
	if((tdi == NULL) && (tdo == NULL))
		return 1;
	
	// go to shift dr state
	jtag_rti_to_shift_dr();
	
	// number of whole bytes that need to be shifted out
	bytes_remaining = (n - 1) / 8;
	// number of bits that need to be shifted out
	// this should only ever be between 1 and 8 inclusive
	bits_remaining  = n - (bytes_remaining * 8);
	
	tdi_i = 0;
	tdo_i = 0;
	chunk_length = 0;
	
	while(bytes_remaining > 0)
	{
		// shift out/in a maximum number bytes at a time
		chunk_length = (bytes_remaining > JTAG_CHUNK_SIZE) ? JTAG_CHUNK_SIZE : bytes_remaining;
		
		//printf("chunk_length %d\n", chunk_length);
		
		// shift the chunk through the data register
		if(tdi != NULL)
		{
			jtag_shift_bytes(&tdi[tdi_i], chunk_length, (tdo != NULL));
			tdi_i += chunk_length;
		} else
			jtag_shift_bytes(NULL, chunk_length, (tdo != NULL));
		
		bytes_remaining -= chunk_length;
		
		// if there is still another chunk to transfer then need to
		// send this chunk and read in any data before sending the next
		if(bytes_remaining > 0)
		{
			if(jtag_send())
			{
				printf("error: jtag_shift_dr: could not send bytes for chunk\n");
				return 1;
			}
			
			if(tdo != NULL)
			{
				if(jtag_recv(&tdo[tdo_i], chunk_length))
				{
					printf("error: jtag_shift_dr: could not receive bytes for chunk\n");
					return 1;
				}
				tdo_i += chunk_length;
			}
		}
	}
	
	// shift the remaining bits
	if(bits_remaining > 0)
	{
		if(tdi != NULL)
			jtag_shift_bits(&tdi[tdi_i], bits_remaining, (tdo != NULL));
		else
			jtag_shift_bits(NULL, bits_remaining, (tdo != NULL));
	}
	
	// back to rti state
	jtag_exit1_dr_to_rti();
	
	// send the last chunk
	if(jtag_send())
	{
		printf("error: jtag_shift_dr: could not send bytes for last chunk\n");
		return 1;
	}
	
	// now receive the bytes from the last chunk sent and any bits
	if(tdo != NULL)
	{
		// if chunk_length is 0 it means that only bits were sent
		if(chunk_length > 0)
		{
			if(jtag_recv(&tdo[tdo_i], chunk_length))
			{
				printf("error: jtag_shift_dr: could not receive bytes for the last chunk\n");
				return 1;
			}
			tdo_i += chunk_length;
		}
		
		if(bits_remaining > 0)
		{
			if(jtag_recv_bits(&tdo[tdo_i], bits_remaining))
			{
				printf("error: jtag_shift_dr: could not receive bits for the last chunk\n");
				return 1;
			}
			tdo_i++;
		}
	}
	
	return 0;
}

////////////////////////////////////////////////////////////////////////
// high level functions
////////////////////////////////////////////////////////////////////////

#define jtag_dr_write(tdi, n) 		(jtag_dr_op(tdi, NULL, n))
#define jtag_dr_read(tdo, n)  		(jtag_dr_op(NULL, tdo, n))
#define jtag_dr_rw(tdi, tdo, n)		(jtag_dr_op(tdi, tdo, n))

// shift in 6 bit instruction
void jtag_ir_write(unsigned char instruction)
{
	jtag_rti_to_shift_ir();
	jtag_shift_bits(&instruction, 6, 0);
	jtag_exit1_ir_to_rti();
}

// sends an invalid command to the ftdi device and checks to see if it
// replies with the correct sequence.
int jtag_mpsse_sync()
{
	int ret;
	unsigned char buf[2];
	// load an invalid instruction
	jtag_buf[jtag_buf_i++] = 0xaa;
	jtag_send();
	ret = jtag_recv(buf, 2);
	return (ret | (buf[0] != 0xfa) | (buf[1] != 0xaa));
}

// read jtag idcode
int jtag_get_idcode(int * idcode)
{
	unsigned char buf[4];
	
	// shift in IDCODE instruction
	jtag_ir_write(JTAG_INSTR_IDCODE);
	
	if(jtag_dr_read(buf, 32))
		return 1;
	
	*idcode = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

	return 0;
}

// swap the bits in the byte 'c' points to
void bit_swap(unsigned char * c)
{
	*c = ((*c & 0xf0) >> 4) | ((*c & 0x0f) << 4);
	*c = ((*c & 0xcc) >> 2) | ((*c & 0x33) << 2);
	*c = ((*c & 0xaa) >> 1) | ((*c & 0x55) << 1);
}

unsigned char * fdata = NULL;
int flength = 0;

// malloc fdata and read from stdin
int load_fdata(char * filename)
{
	int i;
	FILE * fin;
	
	if(fdata == NULL)
		fdata = malloc(FDATA_SIZE);
	if(fdata == NULL)
		return 1;
	
	fin = fopen(filename, "rb");
	
	if(fin == NULL)
		return 1;
	
	flength = fread(fdata, 1, FDATA_SIZE, fin);
	
	fclose(fin);
	
	if((flength < 1) || (flength >= FDATA_SIZE))
		return 1;
	
	for(i = 0; i < flength; i++)
		bit_swap(&fdata[i]);

	return 0;
}

////////////////////////////////////////////////////////////////////////
// main routine and exit function for cleaning up
////////////////////////////////////////////////////////////////////////

int main_exit(int ret, char * s)
{
	if(ret)
		printf("error: main: %s\n", s);
	else
		printf("%s\n", s);
	
	if(fdata != NULL)
	{
		free(fdata);
		fdata = NULL;
	}
	jtag_to_tlr();
	jtag_send();
	jtag_close();
	return ret;
}

int main(int argc, char * argv[])
{
	int idcode, i;
	unsigned char c[2];
	
	if(argc < 2)
	{
		printf("usage: %s <bin file>\n", argv[0]);
		return 1;
	}
	
	// initialize ftdi device for jtag 
	if(jtag_init())
	{
		printf("error: jtag_init failed\n");
		return 1;
	}
	
	if(jtag_mpsse_sync())
		return main_exit(1, "could not sync mpsse controller");
	
	printf("testing 1 byte transfer, send 0xaa\n");
	c[0] = 0xaa;
	if(ftdi_write_data(&ftdi, c, 1) < 0)
		return main_exit(1, "ftdi write 1 byte failed");
	jtag_recv(c, 2);
	printf("receive 0x%02x 0x%02x\n", c[0], c[1]);
	
	
	// put jtag tap into TLR state
	jtag_to_tlr();
	
	// put jtag tap into RTI state
	jtag_tlr_to_rti();

	if(jtag_get_idcode(&idcode))
		return main_exit(1, "could not get idcode");
	
	printf("idcode = 0x%08x\n", idcode);
	
	// check company code and family sections of idcode
	if((idcode & 0x001fffff) != 0x00008093)
		return main_exit(1, "non xilinx fpga device id");
	
	// load file data
	if(load_fdata(argv[1]))
		return main_exit(1, "could not load data from file");
	
	// enable in system configuration
	jtag_ir_write(JTAG_INSTR_JSHUTDOWN);
	
	// spin in RTI waiting for FPGA to shut down
	for(i = 0; i < JTAG_SHUTDOWN_DELAY; i++)
		jtag_rti_spin();
	
	// load CFG_IN instruction
	jtag_ir_write(JTAG_INSTR_CFG_IN);
	
	// write fdata to data register
	if(jtag_dr_write(fdata, flength * 8))
		return main_exit(1, "could not write configuration to data register");
		
	printf("sent %d configuration bytes to fpga\n", flength);
	
	// disable in system configuration
	jtag_ir_write(JTAG_INSTR_JSTART);
	
	// spin in RTI waiting for FPGA to restart
	for(i = 0; i < JTAG_STARTUP_DELAY; i++)
		jtag_rti_spin();
	
	// put jtag into TLR state
	jtag_to_tlr();
	
	if(jtag_send())
		return main_exit(1, "could not disable isc");
	
	return main_exit(0, "configuration complete");
}
