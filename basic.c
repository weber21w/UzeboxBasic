/**
 * Best time for Mandelbrot: 39.2833 seconds
 */

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <uzebox.h>
#include <fatfs/ffconf.h>
#include <fatfs/ff.h>
#include <fatfs/diskio.h>
#include <keyboard.h>
#include <spiram.h>
#include "terminal.h"

#include "data/font6x8-full.inc"
#include "data/fat_pixels.inc"
#include "data/patches.inc"
#include "data/music.inc"

#define Wait200ns() asm volatile("lpm\n\tlpm\n\t");
#define Wait100ns() asm volatile("lpm\n\t");

#define kAutorunFilename "AUTORUN.BAS"

#define PM_INPUT	0
#define PM_OUTPUT	1
#define CONSOLE_BAUD 9600
#define VERSION "0.2"
#define RAM_SIZE 1300
#define STACK_DEPTH	5 //10
#define STACK_SIZE (sizeof(struct stack_for_frame)*STACK_DEPTH)
#define VAR_TYPE float	//type used for number variables
#define VAR_SIZE sizeof(VAR_TYPE)//Size of variables in bytes
#define VAR_TYPE_STR 0
#define VAR_TYPE_NUM 1
#define STRING_BUFFER_SIZE 2 //string buffer
#define HIGHLOW_HIGH	1
#define HIGHLOW_UNKNOWN	4

////////////////////
//ASCII Characters
#define CR	'\r'
#define NL	'\n'
#define LF	0x0a
#define TAB	'\t'
#define BACKSP	'\b'
#define SPACE	 ' '
#define SQUOTE	'\''
#define DQUOTE	'\"'
#define CTRLC	0x03
#define CTRLH	0x08
#define CTRLS	0x13
#define CTRLX	0x18

typedef u16 LINENUM;

//Run flags
#define SD_INITIALIZED	1
#define SPIR_INITIALIZED 2
#define DO_SONG_BUFFER	4

//these will select, at runtime, where IO happens through for load/save
enum{
	kStreamSerial = 0,
	kStreamFile,
	kStreamKeyboard,
	kStreamScreen
};

struct stack_for_frame{
	char frame_type;
	char for_var;
	s16 terminal;
	s16 step;
	u8 *current_line;
	u8 *txt_pos;
	u8 *exit_pos;
};

struct stack_gosub_frame{
	char frame_type;
	u8 *current_line;
	u8 *txtpos;
};

void analogReference(uint8_t mode);
u16 analogRead(u8 pin);
u8 digitalRead(u8 pin);
void analogWrite(u8 pin, u16 val);
void digitalWrite(u8 pin, u8 val);
void tone(u8 f, u8 d);
void noTone();
void pinMode(u8 pin, u8 mode);
static s16 inchar();
static void outchar(char c);
static void line_terminator();
static VAR_TYPE expression();
static bool breakcheck();
void cmd_Files();
char *filenameWord();
void dump_mem(u16 start_addr,u8 rows);

uint8_t SpiRamCursorInit();
uint8_t SpiRamCursorRead(uint32_t addr);
uint8_t SpiRamCursorReadBuffered(uint32_t addr);
void SpiRamCursorWrite(uint32_t addr, uint8_t val);
void SpiRamCursorYield();
void SpiRamCursorUnyield();
u8 SpiRamCursorLoad(char *fname, u32 foff, u32 dlen, u32 roff);

FATFS fs;
FIL f;
u16 bytesWritten;
u16 bytesRead;

u8 patches_loaded = 0;
u8 songs_loaded = 0;
u32 songBase = 0;
u32 songOff = 0;
extern s8 songSpeed;
extern bool playSong;
extern volatile u16 songPos;
extern volatile u16 loopStart;
extern volatile u16 loopEnd;

u8 run_flags = 0;
u8 inhibitOutput = 0;
static u8 runAfterLoad = 0;
static u8 triggerRun = 0;
static u8 inStream = kStreamKeyboard;
static u8 outStream = kStreamScreen;
u8 program[RAM_SIZE];
static u8 *txtpos,*list_line, *tmptxtpos;
static u8 expression_error;
static u8 expression_return_type;
static u8 *tempsp;
static u32 timer_ticks;
static u8 *stack_limit;
static u8 *program_start;
static u8 *program_end;
static u8 *variables_begin;
static u8 *current_line;
static u8 *sp;
static u16 current_line_no;
#define STACK_GOSUB_FLAG 'G'
#define STACK_FOR_FLAG 'F'
static u8 table_index;
static LINENUM linenum;

const u16 uart_bauds[] PROGMEM = { 9600/10, 19200/10, 38400/10, 57600/10, 115200/10 }; 
const u8 uart_divisors[] PROGMEM = { 185, 92, 46, 60, 30};

/***********************************************************/
//Keyword table and constants - the last character has 0x80 added to it
const static u8 keywords[] PROGMEM = {
	'L','I','S','T'+0x80,
	'L','O','A','D'+0x80,
	'N','E','W'+0x80,
	'R','U','N'+0x80,
	'S','A','V','E'+0x80,
	'N','E','X','T'+0x80,
	'L','E','T'+0x80,
	'I','F'+0x80,
	'G','O','T','O'+0x80,
	'G','O','S','U','B'+0x80,
	'R','E','T','U','R','N'+0x80,
	'R','E','M'+0x80,
	'F','O','R'+0x80,
	'I','N','P','U','T'+0x80,
	'P','R','I','N','T'+0x80,
	'P','O','K','E'+0x80,
	'S','T','O','P'+0x80,
	'B','Y','E'+0x80,
	'F','I','L','E','S'+0x80,
	'M','E','M'+0x80,
	'?'+ 0x80,
	'\''+ 0x80,
	'A','W','R','I','T','E'+0x80,
	'D','W','R','I','T','E'+0x80,
	'D','E','L','A','Y'+0x80,
	'E','N','D'+0x80,
	'R','S','E','E','D'+0x80,
	'C','H','A','I','N'+0x80,
	'T','O','N','E','W'+0x80,
	'T','O','N','E'+0x80,
	'N','O','T','O','N','E'+0x80,
	'C','L','S'+0x80,
	'E','X','I','T'+0x80,
	'D','L','O','A','D'+0x80,
	'S','F','X'+0x80,
	'S','F','X','L','D'+0x80,
	'S','O','N','G'+0x80,
	'N','O','S','O','N','G'+0x80,
	'S','O','N','G','L','D'+0x80,
	0
};

enum{//by moving the command list to an enum, we can easily remove sections above and below simultaneously to selectively obliterate functionality.
	KW_LIST = 0,
	KW_LOAD, KW_NEW, KW_RUN, KW_SAVE,
	KW_NEXT, KW_LET, KW_IF,
	KW_GOTO, KW_GOSUB, KW_RETURN,
	KW_REM,
	KW_FOR,
	KW_INPUT, KW_PRINT,
	KW_POKE,
	KW_STOP, KW_BYE,
	KW_FILES,
	KW_MEM,
	KW_QMARK, KW_QUOTE,
	KW_AWRITE, KW_DWRITE,
	KW_DELAY,
	KW_END,
	KW_RSEED,
	KW_CHAIN,
	KW_TONEW, KW_TONE, KW_NOTONE,
	KW_CLS,
	KW_EXIT,
	KW_DLOAD,
	KW_SFX, KW_SFXLD,
	KW_SONG, KW_NOSONG, KW_SONGLD,
	KW_DEFAULT /* always the final one*/
};

#define FUNC_PEEK	0
#define FUNC_ABS	1
#define FUNC_AREAD	2
#define FUNC_DREAD	3
#define FUNC_RND	4
#define FUNC_CHR	5
#define FUNC_TICKS	6
#define FUNC_REDIRI	7
#define FUNC_REDIRO	8
#define FUNC_BAUD	9
#define FUNC_URX	10
#define FUNC_UTX	11
#define FUNC_URXPRT	12
#define FUNC_UTXPRT	13
#define FUNC_JOY	14
#define FUNC_UNKNOWN	15

const static u8 func_tab[] PROGMEM = {
'P','E','E','K'+0x80,
'A','B','S'+0x80,
'A','R','E','A','D'+0x80,
'D','R','E','A','D'+0x80,
'R','N','D'+0x80,
'C','H','R','$'+0x80,
'T','I','C','K','S'+0x80,
'R','E','D','I','R','I'+0x80,
'R','E','D','I','R','O'+0x80,
'B','A','U','D'+0x80,
'U','R','X'+0x80,
'U','T','X'+0x80,
'U','R','X','P','R','T'+0x80,
'U','T','X','P','R','T'+0x80,
0
};

const static u8 to_tab[] PROGMEM = {
	'T','O'+0x80,
	0
};

const static u8 step_tab[] PROGMEM = {
	'S','T','E','P'+0x80,
	0
};

#define RELOP_GE	0
#define RELOP_NE	1
#define RELOP_GT	2
#define RELOP_EQ	3
#define RELOP_LE	4
#define RELOP_LT	5
#define RELOP_NE_BANG	6
#define RELOP_UNKNOWN	7
const static u8 relop_tab[] PROGMEM = {
	'>','='+0x80,
	'<','>'+0x80,
	'>'+0x80,
	'='+0x80,
	'<','='+0x80,
	'<'+0x80,
	'!','='+0x80,
	0
};

const static u8 highlow_tab[] PROGMEM = {
	'H','I','G','H'+0x80,
	'H','I'+0x80,
	'L','O','W'+0x80,
	'L','O'+0x80,
	0
};


static const char okmsg[]			PROGMEM = "Ok";
static const char whatmsg[]			PROGMEM = "Syntax error"; //"What? ";
static const char howmsg[]			PROGMEM = "How?";
static const char sorrymsg[]		PROGMEM = "Sorry!";
static const char initmsg[]			PROGMEM = "UzeBASIC " VERSION;
static const char memorymsg[]		PROGMEM = " bytes free.";
//static const char breakmsg[]		PROGMEM = "\nBreak!";
//static const char unimplimentedmsg[]	PROGMEM = "Unimplemented";
static const char backspacemsg[]	PROGMEM = "\b \b";
static const char indentmsg[]		PROGMEM = "    ";
//static const char sderrormsg[]		PROGMEM = "ERROR: Failed to initialize SD Card, read/write is disabled.";
//static const char sdsuccessmsg[]	PROGMEM = "SUCCESS: SD is initialized";
static const char sdfilemsg[]		PROGMEM = "ERROR: File Operation failed.";
static const char dirextmsg[]		PROGMEM = "(dir)";
static const char slashmsg[]		PROGMEM = "/";
static const char spacemsg[]		PROGMEM = " ";

void vsyncCallback(){
	timer_ticks++;
	terminal_VsyncCallback();	//used to poll the keyboard
}

void dump_mem(u16 start_addr,u8 rows){
	#if SCREEN_TILES_H>40
		u8 width=16;
	#else
		u8 width=8;
	#endif

	printf_P(PSTR("\n"));
	for(u8 i=0;i<rows;i++){
		printf_P(PSTR("%04x: "),(i*width)+start_addr);
		for(u8 j=0;j<width;j++){
			printf_P(PSTR("%02x "),program[((i*width)+j)+(start_addr)]);
		}
		for(u8 j=0;j<width;j++){
			u8 c=program[((i*width)+j)+(start_addr)];
			if(c<32 || c>'~') c='.';
			printf_P(PSTR("%c"),c);
		}
		printf_P(PSTR("\r\n"));
	}
}

void delay(u8 ms){
	s16 time = ms;
	while(time > 0){
		if(GetVsyncFlag()){
			WaitVsync(1);
			time -= 16;
			continue;
		}
		for(u16 i=0;i<1000;i++){
			for(u8 j=0;j<5;j++){
				Wait200ns();
			}
		}
		time--;
	}
}
/***************************************************************************/
static void ignore_blanks(){
	while(*txtpos == SPACE || *txtpos == TAB)
		txtpos++;
}


/***************************************************************************/
static void scantable(const u8 *table){
	s16 i = 0;
	table_index = 0;
	while(1){
		//if(GetVsyncFlag()) WaitVsync(1);
		if(pgm_read_byte(table) == 0)//run out of table entries?
			return;

		if(txtpos[i] == pgm_read_byte(table)){//do we match this character?
			i++;
			table++;
		}else{//do we match the last character of keywork (with 0x80 added)? If so, return
			if(txtpos[i]+0x80 == pgm_read_byte(table)){
				txtpos += i+1;//Advance the pointer to following the keyword
				ignore_blanks();
				return;
			}
			while((pgm_read_byte(table) & 0x80) == 0)//Forward to the end of this keyword
				table++;

			table++;////Now move on to the first character of the next word...
			table_index++;
			ignore_blanks();//...and reset the position index
			i = 0;
		}
	}
}

/***************************************************************************/
static void pushb(u8 b){
	sp--;
	*sp = b;
}

/***************************************************************************/
static u8 popb(){
	u8 b;
	b = *sp;
	sp++;
	return b;
}

/***************************************************************************/
void printnum(VAR_TYPE num){
	printf_P(PSTR("%g"),num);
}

void printUnum(u16 num){
	s16 digits = 0;

	do{
		pushb(num%10+'0');
		num = num/10;
		digits++;
	}
	while(num > 0);

	while(digits > 0){
		outchar(popb());
		digits--;
	}
}

/***************************************************************************/
static u16 test_int_num(){
	u16 num = 0;
	ignore_blanks();

	while(*txtpos>= '0' && *txtpos <= '9'){
		if(num >= 0xFFFF/10){//trap overflows
			num = 0xFFFF;
			break;
		}

		num = num *10 + *txtpos - '0';
		txtpos++;
	}
	return num;
}

/***************************************************************************/
static u8 print_quoted_string(){
	s16 i=0;
	u8 delim = *txtpos;
	if(delim != '"' && delim != '\'')
		return 0;
	txtpos++;

	while(txtpos[i] != delim){//check we have a closing delimiter
		if(txtpos[i] == NL)
			return 0;
		i++;
	}

	while(*txtpos != delim){//print the characters
		outchar(*txtpos);
		txtpos++;
	}
	txtpos++;//skip over the last delimiter
	return 1;
}


/***************************************************************************/
void printmsgNoNL(const char *msg){
	while(pgm_read_byte(msg) != 0){
		outchar(pgm_read_byte(msg++));
	}
}

/***************************************************************************/
void printmsg(const char *msg){
	printmsgNoNL(msg);
	line_terminator();
}

/***************************************************************************/
static void getln(char prompt){
	outchar(prompt);
	txtpos = program_end+sizeof(LINENUM);

	while(1){
		//if(GetVsyncFlag()) WaitVsync(1);
		char c = inchar();
		switch(c){
		case NL:
			//break;
		case CR:
			line_terminator();
			txtpos[0] = NL;//Terminate all strings with a NL
			return;
		case BACKSP:
			if(txtpos == program_end+sizeof(LINENUM))
				break;
			txtpos--;
			printmsgNoNL(backspacemsg);
			break;
		default://We need to leave at least one space to allow us to shuffle the line into order
			if(txtpos == variables_begin-2){
				outchar(BACKSP);
			}else{
				txtpos[0] = c;
				txtpos++;
				outchar(c);
			}
		}
	}
}

/***************************************************************************/
static u8 *findline(){
	u8 *line = program_start;
	while(1){
		if(line == program_end)
			return line;

		if(((LINENUM *)line)[0] >= linenum)
			return line;

		line += line[sizeof(LINENUM)];//Add the line length onto the current address, to get to the next line
	}
}

/***************************************************************************/
static void toUppercaseBuffer(){
	u8 *c = program_end+sizeof(LINENUM);
	u8 quote = 0;

	while(*c != NL){
		//Are we in a quoted string?
		if(*c == quote)
			quote = 0;
		else if(*c == '"' || *c == '\'')
			quote = *c;
		else if(quote == 0 && *c >= 'a' && *c <= 'z')
			*c = *c + 'A' - 'a';
		c++;
	}
}

/***************************************************************************/
void printline(){
	LINENUM line_num;

	line_num = *((LINENUM *)(list_line));
	list_line += sizeof(LINENUM) + sizeof(char);

	//Output the line
	printnum(line_num);
	outchar(' ');
	while(*list_line != NL){
		outchar(*list_line);
		list_line++;
	}
	list_line++;
	line_terminator();
}

/***************************************************************************/
static VAR_TYPE expr4(){
	//fix provided by Jurg Wullschleger wullschleger@gmail.com for whitespace and unary operations
	ignore_blanks();

	//Is it a number?
	if(*txtpos=='-' || (*txtpos >= '0' && *txtpos <= '9')){
		const char *numpos=(char*)txtpos;
		char *endptr;

		do{
			txtpos++;
		}while((*txtpos >= '0' && *txtpos <= '9') || *txtpos=='.' || *txtpos=='E' || *txtpos=='-' || *txtpos=='+' );

		u8 save=*txtpos; //save current char
		*txtpos=0;		//to add zero terminator
		VAR_TYPE num=(VAR_TYPE)strtod(numpos, &endptr);
		*txtpos=save;
		if(endptr==numpos)goto EXPR4_ERROR; //invalid float format
		return num;

	}



	//Is it a function or variable reference?
	if(txtpos[0] >= 'A' && txtpos[0] <= 'Z'){
		VAR_TYPE a;
		//Is it a variable reference (single alpha)
		if(txtpos[1] < 'A' || txtpos[1] > 'Z'){
			a = ((VAR_TYPE *)variables_begin)[*txtpos - 'A'];
			txtpos++;
			return a;
		}

		//Is it a function with a single parameter
		scantable(func_tab);
		if(table_index == FUNC_UNKNOWN)
			goto EXPR4_ERROR;

		u8 f = table_index;
		u8 params;

		if(*txtpos != '(')
			goto EXPR4_ERROR;

		txtpos++;

		if(*txtpos == ')'){
			params=0;
		}else{
			a = expression();
			if(*txtpos != ')') goto EXPR4_ERROR;
			params=1;
		}
		txtpos++;

		switch(f){

			case FUNC_PEEK:
				if(params==0) goto EXPR4_ERROR;
				if(a < RAM_SIZE){
					return program[(u16)a];
				}else{
					return SpiRamCursorRead(a);
				}
			case FUNC_ABS:
				if(params==0) goto EXPR4_ERROR;
				if(a < 0)
					return -a;
				return a;

			case FUNC_AREAD:
				if(params==0) goto EXPR4_ERROR;
				pinMode(a, PM_INPUT);
				return analogRead(a);

			case FUNC_DREAD:
				if(params==0) goto EXPR4_ERROR;
				pinMode(a, PM_INPUT);
				return digitalRead(a);

			case FUNC_RND:
				if(params==0) goto EXPR4_ERROR;
				return(GetPrngNumber(0) % (u16)a);

			case FUNC_CHR:
				if(params==0) goto EXPR4_ERROR;
				expression_return_type=VAR_TYPE_STR;
				return a;

			case FUNC_TICKS:
				return (VAR_TYPE)timer_ticks;

			case FUNC_REDIRI:
				if(params==0)
				return inStream;
				if(params < 0 || params > kStreamScreen) goto EXPR4_ERROR;
				inStream = params;
				return 1;

			case FUNC_REDIRO:
				if(params==0)
				return outStream;
				if(params < 0 || params > kStreamScreen) goto EXPR4_ERROR;
				outStream = params;
				return 1;
			case FUNC_BAUD:
				if(params==0){//get baud
					for(u8 i=0;i<sizeof(uart_divisors);i++){
						if((pgm_read_byte(&uart_divisors[i])*10) == UBRR0L)
							return pgm_read_byte(&uart_bauds[i]);
					}
					goto EXPR4_ERROR;//shouldn't happen..
				}else{//set baud
					for(u8 i=0;i<sizeof(uart_divisors);i++){
						if(pgm_read_word(&uart_bauds[i]) == a/10){
							UBRR0H=0;
							UBRR0L=pgm_read_byte(&uart_divisors[i]);
							UCSR0A=(1<<U2X0); //double speed mode
							UCSR0C=(1<<UCSZ01)+(1<<UCSZ00)+(0<<USBS0); //8-bit frame, no parity, 1 stop bit
							UCSR0B=(1<<RXEN0)+(1<<TXEN0); //Enable UART TX & RX
							return 1;
						}
					}
				}
				return 0;
			case FUNC_URX:
				if(params==0){//check if there is Rx data
					return UartUnreadCount();
				}else{//receive data
					return UartReadChar();
				}
			case FUNC_UTX:
				if(params==0){//check if we could send
					return IsUartTxBufferFull();
				}else{//transmit data
					while(IsUartTxBufferFull());
					UartSendChar(a);
					return 1;
				}

			case FUNC_URXPRT:
				if(params==0){//print everything available in the Rx buffer
					while(UartUnreadCount())
						outchar(UartReadChar());
				}else{
					u8 requested = a;
					while(a--){
						if(!UartUnreadCount())
							break;
						outchar(UartReadChar());
					}
					return requested-a;
				}
			case FUNC_UTXPRT: //TODO
				if(params==0){//Send everything in the string pointed at to UART Tx(wait if needed)

				}else{//Send a specific amount of characters(wait if needed)
					/*while(1){
						while(UartTxBufferIsFull());
						outchar(UartReadChar();
					}*/
					return 1;
				}
			case FUN_JOY:
				if(params==0){//TODO JMAP to allow keyboard to act as joypad
					return 0;
				}else{
					if(a > 1)//TODO multitap..
						return 0;
					return ReadJoypad(a);
				}
		}
	}

	if(*txtpos == '('){
		txtpos++;
		VAR_TYPE a = expression();
		if(*txtpos != ')')
			goto EXPR4_ERROR;

		txtpos++;
		return a;
	}

EXPR4_ERROR:
	expression_error = 1;
	return 0;
}

/***************************************************************************/
static VAR_TYPE expr3(){
	VAR_TYPE a = expr4();
	ignore_blanks();//fix for eg:	100 a = a + 1

	while(1){
		VAR_TYPE b;
		if(*txtpos == '*'){
			txtpos++;
			b = expr4();
			a *= b;
		}else if(*txtpos == '/'){
			txtpos++;
			b = expr4();
			if(b != 0)
				a /= b;
			else
				expression_error = 1;
		}else
			return a;
	}
}

/***************************************************************************/
static VAR_TYPE expr2(){
	VAR_TYPE a;

	if(*txtpos == '-' || *txtpos == '+')
		a = 0;
	else
		a = expr3();

	while(1){
		VAR_TYPE b;
		if(*txtpos == '-'){
			txtpos++;
			b = expr3();
			a -= b;
		}else if(*txtpos == '+'){
			txtpos++;
			b = expr3();
			a += b;
		}else
			return a;
	}
}
/***************************************************************************/
static VAR_TYPE expression(){
	VAR_TYPE a = expr2();
	VAR_TYPE b;

	//Check if we have an error
	if(expression_error)	return a;

	scantable(relop_tab);
	if(table_index == RELOP_UNKNOWN)
		return a;

	switch(table_index){
	case RELOP_GE:
		b = expr2();
		if(a >= b) return 1;
		break;
	case RELOP_NE:
	case RELOP_NE_BANG:
		b = expr2();
		if(a != b) return 1;
		break;
	case RELOP_GT:
		b = expr2();
		if(a > b) return 1;
		break;
	case RELOP_EQ:
		b = expr2();
		if(a == b) return 1;
		break;
	case RELOP_LE:
		b = expr2();
		if(a <= b) return 1;
		break;
	case RELOP_LT:
		b = expr2();
		if(a < b) return 1;
		break;
	}
	return 0;
}

/***************************************************************************/
int main(){
	//bind the terminal receiver to stdout
	stdout = &TERMINAL_STREAM;
	InitMusicPlayer(patches);//temporary, to be replaced with RAM Patches
	patches_loaded = 31;//temporary, to be set by index in file data
	songs_loaded = 2;//temporary

	SetTileTable(font);
	SetFontTilesIndex(0);
	//register interrupt handler to process keystrokes, timers, etc
	SetUserPreVsyncCallback(&vsyncCallback);


	//initialize terminal
	terminal_Init();
	terminal_Clear();
	terminal_SetAutoWrap(true);
	terminal_SetCursorVisible(true);


	GetPrngNumber(GetTrueRandomSeed());
	if(GetPrngNumber(0) == 0)
		GetPrngNumber(0xACE);

	SetMasterVolume(224);
	DDRA |= (1<<PA6);//enable Uzenet module
	PORTD |=(1<<PD3);//reset module
	UBRR0H = 0;
	UBRR0L = 185;//9600
	UCSR0A = (1<<U2X0);//double speed
	UCSR0C = (1<<UCSZ01)+(1<<UCSZ00)+(0<<USBS0);//8N1
	UCSR0B = (1<<RXEN0)+(1<<TXEN0);//enable TX & RX

	/////Serial.println(sentinel);
	printmsg(initmsg);

	run_flags &= ~(SD_INITIALIZED);//try 10 times to initialize else fail
	printmsgNoNL(PSTR("Initializing SD..."));
	for(u8 i=0;i<10;i++){
		if(f_mount(0, &fs) != FR_OK){//if(f_mount(&fs, "", 1) != FR_OK){
			PORTD &= ~(1<<6);//deassert card
			WaitVsync(30);//wait
			continue;
		}else{
			printmsg(PSTR("Success!"));
			run_flags |= SD_INITIALIZED;
			break;
		}
	}
	if(!(run_flags & SD_INITIALIZED)){
		printmsg(PSTR("ERROR"));
	}

	SpiRamCursorInit();
	if(run_flags & SPIR_INITIALIZED){
		printmsg(PSTR("SPI RAM Found!"));
	}

	outStream = kStreamScreen;
	inStream = kStreamKeyboard;
	inhibitOutput = 0;

	printmsgNoNL(PSTR("Searching for "));
	printmsgNoNL(PSTR(kAutorunFilename));
	printmsgNoNL(PSTR("..."));
	if(f_open(&f, kAutorunFilename, FA_OPEN_EXISTING|FA_READ) == FR_OK){//try to load autorun file if present
		printmsg(PSTR("Loaded"));
		program_end = program_start;
		inStream = kStreamFile;
		inhibitOutput = 1;
		runAfterLoad = 1;
	}else
		printmsg(PSTR("Not Found"));
	u8 *start;
	u8 *newEnd;
	u8 linelen;
	u8 isDigital;
	u8 alsoWait = 0;
	VAR_TYPE val,val2,val3;
	u8 var;
	char *filename;

	program_start = program;
	program_end = program_start;
	sp = program+sizeof(program);	//Needed for printnum
	stack_limit = program+sizeof(program)-STACK_SIZE;
	variables_begin = stack_limit - 27*VAR_SIZE;

	//memory free
	printnum(variables_begin-program_end);
	printmsg(memorymsg);

WARMSTART:
	//this signifies that it is running in 'direct' mode.
	current_line = 0;
	sp = program+sizeof(program);
	printmsg(okmsg);

PROMPT:
	if(triggerRun){
		triggerRun = 0;
		current_line = program_start;
		goto EXECLINE;
	}

	getln('>');
	toUppercaseBuffer();
	txtpos = program_end+sizeof(u16);
	while(*txtpos != NL)//find the end of the freshly entered line
		txtpos++;

	u8 *dest;//move it to the end of program_memory
	dest = variables_begin-1;
	while(1){
		*dest = *txtpos;
		if(txtpos == program_end+sizeof(u16))
			break;
		dest--;
		txtpos--;
	}
	txtpos = dest;

	linenum = test_int_num();//now see if we have a line number
	ignore_blanks();
	if(linenum == 0)
		goto DIRECT;

	if(linenum == 0xFFFF)
		goto QHOW;

	linelen = 0;
	while(txtpos[linelen] != NL)//find the length of what's left, including the (not yet populated) line header
		linelen++;
	linelen++;//Include the NL in the line length
	linelen += sizeof(u16)+sizeof(char);//Add space for the line number and line length

	//Now we have the number, add the line header.
	txtpos -= 3;

	*((u16 *)txtpos) = linenum;
	txtpos[sizeof(LINENUM)] = linelen;


	//Merge it into the rest of the program
	start = findline();

	//If a line with that number exists, then remove it
	if(start != program_end && *((LINENUM *)start) == linenum){
		u8 *dest, *from;
		unsigned tomove;

		from = start + start[sizeof(LINENUM)];
		dest = start;

		tomove = program_end - from;
		while(tomove > 0){
			*dest = *from;
			from++;
			dest++;
			tomove--;
		}
		program_end = dest;
	}

	if(txtpos[sizeof(LINENUM)+sizeof(char)] == NL)//If the line has no txt, it was just a delete
		goto PROMPT;



	//Make room for the new line, either all in one hit or lots of little shuffles
	while(linelen > 0){
		u16 tomove;
		u8 *from,*dest;
		u16 space_to_make;

		space_to_make = txtpos - program_end;

		if(space_to_make > linelen)
			space_to_make = linelen;
		newEnd = program_end+space_to_make;
		tomove = program_end - start;


		//Source and destination - as these areas may overlap we need to move bottom up
		from = program_end;
		dest = newEnd;
		while(tomove > 0){
			from--;
			dest--;
			*dest = *from;
			tomove--;
		}

		//Copy over the bytes into the new space
		for(tomove = 0; tomove < space_to_make; tomove++){
			*start = *txtpos;
			txtpos++;
			start++;
			linelen--;
		}
		program_end = newEnd;
	}
	goto PROMPT;

//UNIMPLEMENTED:
//	printmsg(unimplimentedmsg);
//	goto PROMPT;

QHOW:
	printmsg(howmsg);
	goto PROMPT;

QWHAT:
	line_terminator();
	printmsgNoNL(whatmsg);
	if(current_line != NULL){
		printmsgNoNL(PSTR(" in "));
		u8 tmp = *txtpos;
		if(*txtpos != NL)
			*txtpos = '^';
		list_line = current_line;
		printline();
		*txtpos = tmp;
	}else{
		line_terminator();
	}
	printmsg(okmsg);
	goto PROMPT;

QSORRY:
	printmsg(sorrymsg);
	goto WARMSTART;

RUN_NEXT_STATEMENT:
	while(*txtpos == ':')
		txtpos++;
	ignore_blanks();
	if(*txtpos == NL)
		goto EXECNEXTLINE;
	goto INTERPRET_AT_TXT_POS;

DIRECT:
	txtpos = program_end+sizeof(LINENUM);
	if(*txtpos == NL)
		goto PROMPT;

INTERPRET_AT_TXT_POS:
	if(breakcheck()){
		printf_P(PSTR("\nBreak on line %i\n"),(current_line[1]<<8)+current_line[0]);
		goto WARMSTART;
	}

	scantable(keywords);

	switch(table_index){
	case KW_DELAY:
			expression_error = 0;
			val = expression();
			delay(val);
			goto EXECNEXTLINE;
	case KW_FILES:
		goto FILES;
	case KW_LIST:
		goto LIST;
	case KW_CHAIN:
		goto CHAIN;
	case KW_LOAD:
		goto LOAD;
	case KW_MEM:
		goto MEM;
	case KW_NEW:
		if(txtpos[0] != NL)
			goto QWHAT;
		program_end = program_start;
		goto PROMPT;
	case KW_RUN:
		current_line = program_start;
		goto EXECLINE;
	case KW_SAVE:
		goto SAVE;
	case KW_NEXT:
		goto NEXT;
	case KW_LET:
		goto ASSIGNMENT;
	case KW_IF:
		expression_error = 0;
		VAR_TYPE val = expression();
		if(expression_error || *txtpos == NL)
			goto QHOW;
		if(val != 0)
			goto INTERPRET_AT_TXT_POS;
		goto EXECNEXTLINE;

	case KW_GOTO:
		expression_error = 0;
		linenum = expression();
		if(expression_error || *txtpos != NL)
			goto QHOW;
		current_line = findline();
		goto EXECLINE;

	case KW_GOSUB:
		goto GOSUB;
	case KW_RETURN:
		goto GOSUB_RETURN;
	case KW_REM:
	case KW_QUOTE:
		goto EXECNEXTLINE;	//Ignore line completely
	case KW_FOR:
		goto FORLOOP;
	case KW_INPUT:
		goto INPUT;
	case KW_PRINT:
	case KW_QMARK:
		goto PRINT;
	case KW_POKE:
		goto POKE;
	case KW_END:
	case KW_STOP:
		//This is the easy way to end - set the current line to the end of program attempt to run it
		if(txtpos[0] != NL)
			goto QWHAT;
		current_line = program_end;
		goto EXECLINE;
	case KW_BYE://Leave the basic interperater
		goto WARMSTART;

	case KW_AWRITE:	//AWRITE <pin>, HIGH|LOW
		isDigital = 0;
		goto AWRITE;
	case KW_DWRITE:	//DWRITE <pin>, HIGH|LOW
		isDigital = 1;
		goto DWRITE;

	case KW_RSEED:
		goto RSEED;

	case KW_TONEW:
		alsoWait = 1;
	case KW_TONE:
		goto TONEGEN;
	case KW_NOTONE:
		goto TONESTOP;
	case KW_CLS:
		goto CLS;
	case KW_EXIT:
		goto EXIT;
	case KW_DLOAD:
		goto DLOAD;
	case KW_SFX:
		goto SFX;
	case KW_SFXLD:
		goto SFXLD;
	case KW_SONG:
		goto SONG;
	case KW_NOSONG:
		goto NOSONG;
	case KW_SONGLD:
		goto SONGLD;
	case KW_DEFAULT:
		goto ASSIGNMENT;
	default:
		break;
	}

EXECNEXTLINE:
	if(current_line == NULL) goto PROMPT;//Processing direct commands?
	current_line_no = (u8)current_line[-2];
	current_line +=	 current_line[sizeof(LINENUM)];

EXECLINE:
	if(current_line == program_end) goto WARMSTART;//Out of lines to run
	txtpos = current_line+sizeof(LINENUM)+sizeof(char);
	goto INTERPRET_AT_TXT_POS;

INPUT:
		ignore_blanks();
		if(*txtpos < 'A' || *txtpos > 'Z') goto QWHAT;
		var = *txtpos;
		txtpos++;
		ignore_blanks();
		if(*txtpos != NL && *txtpos != ':') goto QWHAT;
INPUTAGAIN:
		tmptxtpos = txtpos;
		getln('?');
		toUppercaseBuffer();
		txtpos = program_end+sizeof(u16);
		ignore_blanks();
		expression_error = 0;
		val = expression();
		if(expression_error)
			goto INPUTAGAIN;
		((VAR_TYPE *)variables_begin)[var-'A'] = val;
		txtpos = tmptxtpos;

		goto RUN_NEXT_STATEMENT;

FORLOOP:
		ignore_blanks();
		if(*txtpos < 'A' || *txtpos > 'Z') goto QWHAT;
		var = *txtpos;
		txtpos++;
		ignore_blanks();
		if(*txtpos != '=') goto QWHAT;
		txtpos++;
		ignore_blanks();

		expression_error = 0;
		VAR_TYPE initial = expression();
		if(expression_error) goto QWHAT;

		scantable(to_tab);
		if(table_index != 0) goto QWHAT;

		VAR_TYPE terminal = expression();
		if(expression_error) goto QWHAT;

		scantable(step_tab);
		VAR_TYPE step;
		if(table_index == 0){
			step = expression();
			if(expression_error) goto QWHAT;
		}else{
			step = 1;
		}
		ignore_blanks();
		if(*txtpos != NL && *txtpos != ':') goto QWHAT;


		if(!expression_error && *txtpos == NL){
			//printf("%c>",var);
			struct stack_for_frame *f;
			if(sp + sizeof(struct stack_for_frame) < stack_limit) goto QSORRY;
			sp -= sizeof(struct stack_for_frame);
			f = (struct stack_for_frame *)sp;
			((VAR_TYPE *)variables_begin)[var-'A'] = initial;
			f->frame_type = STACK_FOR_FLAG;
			f->for_var = var;
			f->terminal = terminal;
			f->step		 = step;
			f->txt_pos	 = txtpos;
			f->current_line = current_line;
			f->exit_pos = 0;
			goto RUN_NEXT_STATEMENT;
		}
	goto QHOW;

GOSUB:


	expression_error = 0;
	linenum = expression();
	if(!expression_error && *txtpos == NL){
		struct stack_gosub_frame *f;
		if(sp + sizeof(struct stack_gosub_frame) < stack_limit)
			goto QSORRY;

		sp -= sizeof(struct stack_gosub_frame);
		f = (struct stack_gosub_frame *)sp;
		f->frame_type = STACK_GOSUB_FLAG;
		f->txtpos = txtpos;
		f->current_line = current_line;
		current_line = findline();
		goto EXECLINE;
	}
	goto QHOW;

//exit the current for-next loop
EXIT:
	//Locate the NEXT statement by scanning through the program memory
	if(*txtpos != NL) goto QWHAT; //EXIT must be the last statement on line

	//did we already have the NEXT statement location?
//	struct stack_for_frame *f2 = (struct stack_for_frame *)sp;
//	if(f2->exit_pos !=0){
//		printf("<ep:%04x>",(u16)f2->exit_pos-(u16)program);
//		txtpos=f2->exit_pos;
//		sp = sp + sizeof(struct stack_for_frame); //pop the stack
//		goto RUN_NEXT_STATEMENT;
//	}

	while(1){
		current_line +=	 current_line[sizeof(LINENUM)];
		if(current_line == program_end) goto QHOW;//Out of lines to run
		txtpos = current_line+sizeof(LINENUM)+sizeof(char);
		char next[]={'N','E','X','T'};
		u8 i=0;

		while(*txtpos != NL){
			if(*txtpos++ != next[i]){
				i=0;
			}else{
				i++;
				if(i==4){
					//NEXT found, find the variable name
					ignore_blanks();
					if(*txtpos < 'A' || *txtpos > 'Z') goto QHOW;
					txtpos++;
					ignore_blanks();
					if(*txtpos != ':' && *txtpos != NL) goto QWHAT;

					//Drop out of the loop, popping the stack
					struct stack_for_frame *f = (struct stack_for_frame *)sp;
					if(f->for_var != txtpos[-1]) goto QHOW;
					sp = sp + sizeof(struct stack_for_frame);
					goto RUN_NEXT_STATEMENT;

				}
			}
		}
	}

NEXT:
	ignore_blanks();//find the variable name
	if(*txtpos < 'A' || *txtpos > 'Z') goto QHOW;
	txtpos++;
	ignore_blanks();
	if(*txtpos != ':' && *txtpos != NL) goto QWHAT;

GOSUB_RETURN:
	tempsp = sp;
	while(tempsp < program+sizeof(program)-1){//walk up the stack frames and find the frame we want(if present)
		switch(tempsp[0]){
		case STACK_GOSUB_FLAG:
			if(table_index == KW_RETURN){
				struct stack_gosub_frame *f = (struct stack_gosub_frame *)tempsp;
				current_line	= f->current_line;
				txtpos			= f->txtpos;
				sp += sizeof(struct stack_gosub_frame);
				goto RUN_NEXT_STATEMENT;
			}
			//This is not the loop you are looking for... so Walk back up the stack
			tempsp += sizeof(struct stack_gosub_frame);
			break;
		case STACK_FOR_FLAG:
			//Flag, Var, Final, Step
			if(table_index == KW_NEXT){
				struct stack_for_frame *f = (struct stack_for_frame *)tempsp;
				//Is the the variable we are looking for?
				if(txtpos[-1] == f->for_var){
					VAR_TYPE *varaddr = ((VAR_TYPE *)variables_begin) + txtpos[-1] - 'A';
					*varaddr = *varaddr + f->step;
					//Use a different test depending on the sign of the step increment
					if((f->step > 0 && *varaddr <= f->terminal) || (f->step < 0 && *varaddr >= f->terminal)){

						//dump_mem(0xa0,20);
						//f->exit_pos=txtpos-7;
						//printf("[ep:%04x]",(u16)f->txt_pos-(u16)program);


						//We have to loop so don't pop the stack
						txtpos = f->txt_pos;
						current_line = f->current_line;
						goto RUN_NEXT_STATEMENT;
					}
					//We've run to the end of the loop. drop out of the loop, popping the stack
					sp = tempsp + sizeof(struct stack_for_frame);
					goto RUN_NEXT_STATEMENT;
				}
			}
			//This is not the loop you are looking for... so Walk back up the stack
			tempsp += sizeof(struct stack_for_frame);
			break;
		default:
			printmsg(PSTR("Stack is stuffed!\n"));
			goto WARMSTART;
		}
	}
	//Didn't find the variable we've been looking for
	goto QHOW;

ASSIGNMENT:
	if(*txtpos < 'A' || *txtpos > 'Z') goto QHOW;
	VAR_TYPE *pvar = (VAR_TYPE *)variables_begin + *txtpos - 'A';
	txtpos++;
	ignore_blanks();

	if (*txtpos != '=') goto QWHAT;
	txtpos++;
	ignore_blanks();
	expression_error = 0;
	val = expression();
	if(expression_error) goto QWHAT;
	if(*txtpos != NL && *txtpos != ':') goto QWHAT;//check that we are at the end of the statement
	*pvar = val;
	goto RUN_NEXT_STATEMENT;

CLS:
	//erase screen
	terminal_Clear();
	goto RUN_NEXT_STATEMENT;

POKE:
	expression_error = 0;
	val = expression();//work out where to put it
	if(expression_error) goto QWHAT;
	//u8 *address = (u8 *)val;

	ignore_blanks();//check for a comma
	if (*txtpos != ',') goto QWHAT;
	txtpos++;
	ignore_blanks();
	expression_error = 0;
	val = expression();//get the value to assign
	if(expression_error) goto QWHAT;
	//printf("Poke %p value %i\n",address, (u8)value);
	//Check that we are at the end of the statement
	if(*txtpos != NL && *txtpos != ':') goto QWHAT;
	goto RUN_NEXT_STATEMENT;

LIST:
	linenum = test_int_num();//Retuns 0 if no line found.

	//Should be EOL
	if(txtpos[0] != NL)
		goto QWHAT;

	//Find the line
	list_line = findline();
	while(list_line != program_end)
		printline();
	goto WARMSTART;

PRINT:
	//If we have an empty list then just put out a NL
	if(*txtpos == ':'){
		line_terminator();
		txtpos++;
		goto RUN_NEXT_STATEMENT;
	}
	if(*txtpos == NL){
		goto EXECNEXTLINE;
	}

	while(1){
		ignore_blanks();
		if(print_quoted_string()){
			;
		}else if(*txtpos == '"' || *txtpos == '\''){
			goto QWHAT;
		}else{
			VAR_TYPE e;
			expression_error = 0;
			expression_return_type=VAR_TYPE_NUM;
			e = expression();
			if(expression_error) goto QWHAT;
			if(expression_return_type==VAR_TYPE_STR){
				putchar((char)e); //todo: support string buffer
			}else{
				printnum(e);
			}
		}

		//At this point we have three options, a comma or a new line
		if(*txtpos == ','){
			txtpos++;	//Skip the comma and move onto the next
		}else if(txtpos[0] == ';' && (txtpos[1] == NL || txtpos[1] == ':')){
			txtpos++;//This has to be the end of the print - no newline
			break;
		}else if(*txtpos == NL || *txtpos == ':'){
			line_terminator();	//The end of the print statement
			break;
		}else
			goto QWHAT;
	}
	goto RUN_NEXT_STATEMENT;

MEM:
	//memory free
	printnum(variables_begin-program_end);
	printmsg(memorymsg);
	goto RUN_NEXT_STATEMENT;


	/*************************************************/
AWRITE://AWRITE <pin>,val
DWRITE:
	expression_error = 0;
	VAR_TYPE pinNo = expression();//get the pin number
	if(expression_error) goto QWHAT;

	ignore_blanks();//check for a comma
	if (*txtpos != ',') goto QWHAT;
	txtpos++;
	ignore_blanks();

	//u8 *txtposBak = txtpos;
	scantable(highlow_tab);
	if(table_index != HIGHLOW_UNKNOWN){
		if(table_index <= HIGHLOW_HIGH){
			val = 1;
		}else{
			val = 0;
		}
	}else{//and the value (numerical)
		expression_error = 0;
		val = expression();
		if(expression_error) goto QWHAT;
	}
	pinMode(pinNo, PM_OUTPUT);
	if(isDigital){
		digitalWrite(pinNo, val);
	}else{
		analogWrite(pinNo, val);
	}
	goto RUN_NEXT_STATEMENT;

FILES:
	cmd_Files();
	goto WARMSTART;

CHAIN:
	runAfterLoad = 1;

LOAD:
	program_end = program_start;//clear the program
	expression_error = 0;
	filename = filenameWord();//work out the filename
	if(expression_error) goto QWHAT;

	if(f_open(&f, (const char*)filename, FA_READ) == FR_OK){
		inStream = kStreamFile;//this will kickstart a series of events to read in from the file.
		inhibitOutput = 1;
	}else{
		printmsg(sdfilemsg);
	}

	goto WARMSTART;

SAVE:
	expression_error = 0;
	filename = filenameWord();//work out the filename
	if(expression_error) goto QWHAT;

	//open the file(overwrite if existing), switch over to file output
	if(f_open(&f, (const char *)filename, FA_WRITE) == FR_OK){//|FA_CREATE_ALWAYS
		outStream = kStreamFile;
	}else{
		printmsg(sdfilemsg);
	}

	list_line = findline();//copied from "List"
	while(list_line != program_end)
		printline();

	outStream = kStreamScreen;//go back to standard output, close the file
	f_close(&f);
	goto WARMSTART;

RSEED:
	expression_error = 0;
	val = expression();//get pin number
	if(expression_error) goto QWHAT;

	GetPrngNumber(val);
	goto RUN_NEXT_STATEMENT;

TONESTOP:
	noTone();
	goto RUN_NEXT_STATEMENT;

TONEGEN://TONE freq, duration
	expression_error = 0;
	val = expression();//get the frequency(if 0, turn off tone)
	if(expression_error) goto QWHAT;
	if(val == 0) goto TONESTOP;
	ignore_blanks();
	if(*txtpos != ',') goto QWHAT;
	txtpos++;
	ignore_blanks();
	expression_error = 0;
	val2 = expression();//get the duration(if 0, turn off tone0
	if(expression_error) goto QWHAT;
	if(val2 == 0) goto TONESTOP;

	tone(val, val2);//frequency, duration
	if(alsoWait){
		delay(val2);
		alsoWait = 0;
	}
	goto RUN_NEXT_STATEMENT;

DLOAD:
	expression_error = 0;
	filename = filenameWord();//work out the filename
		if(expression_error) goto QWHAT;
		//txtpos++;
		//ignore_blanks();
		//if(*txtpos != ',') goto QWHAT;
		txtpos++;
		ignore_blanks();
		expression_error = 0;
	u32 foff = expression();//get starting offset to read from
		if(expression_error) goto QWHAT;
		ignore_blanks();
		if(*txtpos != ',') goto QWHAT;
		txtpos++;
		ignore_blanks();
		expression_error = 0;
	u32 dlen = expression();//get data length to read
		if(expression_error) goto QWHAT;
		ignore_blanks();
		if(*txtpos != ',') goto QWHAT;
		txtpos++;
		ignore_blanks();
		expression_error = 0;
	if(dlen == 0)
		dlen = 999999UL;

	u32 roff = expression();//get starting offset in memory to write
	if(expression_error) goto QWHAT;

	SpiRamCursorLoad(filename, foff, dlen, roff);
	goto RUN_NEXT_STATEMENT;

SFX://SFX patch, volume, retrigger
	expression_error = 0;
	val = expression();//get patch number
	if(expression_error) goto QWHAT;
	if(val >= patches_loaded) goto QSORRY;
	ignore_blanks();
	if(*txtpos != ','){
		val2 = 192;
		val3 = 1;
 	}else{
		txtpos++;
		ignore_blanks();
		expression_error = 0;
		val2 = expression();//get volume
		if(expression_error) goto QWHAT;
		ignore_blanks();
		if(*txtpos != ','){
			val3 = 1;
		}else{
			txtpos++;
			ignore_blanks();
			expression_error = 0;
			val3 = expression();//get the patch number
			if(expression_error) goto QWHAT;
		}
	}
	TriggerFx((u8)val, (u8)val2, (u8)val3);
	goto RUN_NEXT_STATEMENT;

SFXLD:
SONGLD:
	printmsg(PSTR("not implemented"));
	goto RUN_NEXT_STATEMENT;

SONG://SONG songNum
	expression_error = 0;
	val = expression();
	if(expression_error) goto QWHAT;
	if(val >= songs_loaded) goto QSORRY;
	StartSong((const char *)pgm_read_word(&song_table[(u8)val]));//temporary
	//SpiRamCursorYield();
	//songBase = SpiRamReadU32(((uint32_t)((u8)(val)*4))>>16,((u8)(val)*4)&0xFFFF);
	//StartSong();
	//run_flags |= DO_SONG_BUFFER;//start reading compressed MIDI from SPI RAM
	goto RUN_NEXT_STATEMENT;

NOSONG:
	//expression_error = 0;
	StopSong();
	goto RUN_NEXT_STATEMENT;

	return 0;
}

//returns 1 if the character is valid in a filename
static s16 isValidFnChar(char c){
	if(c >= '0' && c <= '9') return 1;//number
	if(c >= 'A' && c <= 'Z') return 1;//LETTER
	if(c >= 'a' && c <= 'z') return 1;//letter (for completeness)
	if(c == '_') return 1;
	if(c == '+') return 1;
	if(c == '.') return 1;
	if(c == '~') return 1;	//Window~1.txt

	return 0;
}

char *filenameWord(){
	//SDL - I wasn't sure if this functionality existed above, so I figured i'd put it here
	u8 * ret = txtpos;
	expression_error = 0;

	//make sure there are no quotes or spaces, search for valid characters
	//while(*txtpos == SPACE || *txtpos == TAB || *txtpos == SQUOTE || *txtpos == DQUOTE) txtpos++;
	while(!isValidFnChar(*txtpos)) txtpos++;
	ret = txtpos;

	if(*ret == '\0'){
		expression_error = 1;
		return (char *)ret;
	}

	//now, find the next nonfnchar
	txtpos++;
	while(isValidFnChar(*txtpos)) txtpos++;
	if(txtpos != ret) *txtpos = '\0';

	//set the error code if we've got no string
	if(*ret == '\0'){
		expression_error = 1;
	}

	return (char *)ret;
}

/***************************************************************************/
static void line_terminator(){
	outchar(NL);
	outchar(CR);
}

/***********************************************************/
static bool breakcheck(){

	if(terminal_HasChar()){
		if(terminal_GetChar()==CTRL_C){
			return true;
		}
	}

	return false;
}
/***********************************************************/
static s16 inchar(){
	s16 v;
	switch(inStream){
	case(kStreamKeyboard):

		//why blocking?
		while(1){
			while(!terminal_HasChar()){}
			v=terminal_GetChar();
			return v;
		}

		break;
	case(kStreamFile):
		if(GetVsyncFlag()) WaitVsync(1);
		f_read(&f, &v, 1, &bytesRead);
		if(bytesRead != 1){
			f_close(&f);
			goto INCHAR_LOADFINISH;
		}
		if(v == NL) v=CR;//file translate

		return v;
		break;
	 case(kStreamSerial):
	default:
		while(1){
			if(GetVsyncFlag()) WaitVsync(1);
			if(UartUnreadCount())
				return UartReadChar();
		}
	}

INCHAR_LOADFINISH:
	inStream = kStreamKeyboard;
	inhibitOutput = 0;

	if(runAfterLoad){
		runAfterLoad = 0;
		triggerRun = 1;
	}
	return NL;//trigger a prompt.
}

/***********************************************************/
static void outchar(char c){
	if(inhibitOutput) return;

	if(outStream == kStreamScreen){
		//ConsolePrintChar(c);
		terminal_SendChar(c);
	}else if(outStream == kStreamFile){
		f_write(&f, &c, 1, &bytesWritten);
	}else{
		while(IsUartTxBufferFull());
		UartSendChar(c);
	}
}

void cmd_Files(){
	DIR d;
	if(f_opendir(&d, "/") != FR_OK)
		return;
	FILINFO entry;

	while(1){
		if(GetVsyncFlag()) WaitVsync(1);
		if(f_readdir(&d, &entry) != FR_OK || entry.fname[0] == 0)
			break;
		//common header
		printmsgNoNL(indentmsg);
		printmsgNoNL((const char *)entry.fname);
		if(entry.fattrib & AM_DIR){
			printmsgNoNL(slashmsg);
			u8 found_end = 0;
			for(u8 i=0; i<13 ; i++){
				if(entry.fname[i] == '\0')
					found_end = 1;
				if(found_end)
					printmsgNoNL(spacemsg);
			}
			printmsgNoNL(dirextmsg);
		}else{//file ending
			u8 found_end = 0;
			for(u8 i=0; i<13 ; i++){
				if(entry.fname[i] == '\0')
					found_end = 1;
				if(found_end)
					printmsgNoNL(spacemsg);
			}
			printnum(entry.fsize);
		}
		line_terminator();
	}
	f_close(&f);
}

void analogReference(uint8_t mode){
}

u16 analogRead(u8 pin){
	return 0;
}
u8 digitalRead(u8 pin){
	return 0;
}

void analogWrite(u8 pin, u16 val){
}

void digitalWrite(u8 pin, u8 val){
}

void tone(u8 f, u8 d){
}

void noTone(){
}

void pinMode(u8 pin, u8 mode){
}

#define SPIR_CURSOR_IDLE	0
#define SPIR_CURSOR_READ	1//always sequential
#define SPIR_CURSOR_WRITE	2//always sequential

u32 spiram_cursor = 0;
u32 spiram_buffer_pos = 0;
u8 spiram_buffer_val = 0;
u8 spiram_state = 0;
u8 spiram_last_state = 0;

uint8_t SpiRamCursorInit(){
	spiram_state = SPIR_CURSOR_IDLE;
	return SpiRamInit();
}

uint8_t SpiRamCursorRead(uint32_t addr){//assumes SPI RAM is not yielded
	if(spiram_state != SPIR_CURSOR_READ){//in a sequential write or yielded?
		//if(spiram_state == SPIR_CURSOR_WRITE)
			SpiRamSeqWriteEnd();
		asm("nop");asm("nop");
		SpiRamSeqReadStart((addr&0xFF0000)>>16, (u16)(addr&0xFFFF));
		asm("nop");asm("nop");
		spiram_state = SPIR_CURSOR_READ;
		spiram_cursor = addr+1;
		return SpiRamSeqReadU8();
	}
	if(spiram_cursor != addr){//current sequential read position doesn't match?
		SpiRamSeqReadEnd();
		asm("nop");asm("nop");
		SpiRamSeqReadStart((addr&0xFF0000)>>16, (u16)(addr&0xFFFF));
		asm("nop");asm("nop");
		spiram_cursor = addr+1;
		return SpiRamSeqReadU8();
	}
	spiram_cursor++;
	return SpiRamSeqReadU8();
}

uint8_t SpiRamCursorReadBuffered(uint32_t addr){//useful if you end up needing the previous byte read...
	if(addr == spiram_buffer_pos)
		return spiram_buffer_val;
	spiram_buffer_pos = addr;
	if(spiram_state != SPIR_CURSOR_READ){//in a sequential write or yielded?
		//if(spiram_state == SPIR_CURSOR_WRITE)
			SpiRamSeqWriteEnd();
		asm("nop");asm("nop");
		SpiRamSeqReadStart((addr&0xFF0000)>>16, (u16)(addr&0xFFFF));
		asm("nop");asm("nop");
		spiram_state = SPIR_CURSOR_READ;
		spiram_cursor = addr+1;
		spiram_buffer_val = SpiRamSeqReadU8();
		return spiram_buffer_val;
	}
	if(spiram_cursor != addr){//current sequential read position doesn't match?
		SpiRamSeqReadEnd();
		asm("nop");asm("nop");
		SpiRamSeqReadStart((addr&0xFF0000)>>16, (u16)(addr&0xFFFF));
		asm("nop");asm("nop");
		spiram_cursor = addr+1;
		spiram_buffer_val = SpiRamSeqReadU8();
		return spiram_buffer_val;
	}
	spiram_cursor++;
	spiram_buffer_val = SpiRamSeqReadU8();
	return spiram_buffer_val;
}

void SpiRamCursorWrite(uint32_t addr, uint8_t val){//assumes SPI RAM is not yielded
	if(spiram_state < SPIR_CURSOR_WRITE){//in a sequential read or yielded?
		//if(spiram_state == SPIR_CURSOR_READ)
			SpiRamSeqReadEnd();
		asm("nop");asm("nop");
		SpiRamSeqWriteStart((addr&0xFF0000)>>16, (u16)(addr&0xFFFF));
		asm("nop");asm("nop");
		spiram_state = SPIR_CURSOR_WRITE;
		spiram_cursor = addr+1;
		SpiRamSeqWriteU8(val);
		return;
	}
	if(spiram_cursor != addr){//current sequential write position doesn't match?
		SpiRamSeqWriteEnd();
		asm("nop");asm("nop");
		spiram_cursor = addr+1;
		SpiRamSeqWriteStart((addr&0xFF0000)>>16, (u16)(addr&0xFFFF));
		asm("nop");asm("nop");
		SpiRamSeqWriteU8(val);
		return;
	}
	spiram_cursor++;
	SpiRamSeqWriteU8(val);
}

void SpiRamCursorYield(){//allow SD access
	spiram_last_state = spiram_state;
	if(spiram_state == SPIR_CURSOR_READ)
		SpiRamSeqReadEnd();
	else if(spiram_state == SPIR_CURSOR_WRITE)
		SpiRamSeqWriteEnd();
	spiram_state = SPIR_CURSOR_IDLE;
}

void SpiRamCursorUnyield(){//this exists so assumptions can be made in read/write
	spiram_state = spiram_last_state;
	if(spiram_state == SPIR_CURSOR_READ)
		SpiRamSeqReadStart((spiram_cursor&0xFF0000)>>16, (u16)(spiram_cursor&0xFFFF));
	else if(spiram_state == SPIR_CURSOR_WRITE)
		SpiRamSeqWriteStart((spiram_cursor&0xFF0000)>>16, (u16)(spiram_cursor&0xFFFF));
}

u8 SpiRamCursorLoad(char *fname, u32 foff, u32 dlen, u32 roff){
	SpiRamCursorYield();
	if(f_open(&f, (const char*)fname, FA_READ) == FR_OK){
		u8 d;

		while(dlen == 999999UL || dlen--){
			f_read(&f, &d, 1, &bytesRead);
			foff++;
			if(bytesRead != 1){
				if(dlen == 999999UL)
					goto SPIR_CURSOR_LOAD_FINISH;
				printmsg(PSTR("ERROR Ran out of file bytes"));
				return 0;
			}
			SpiRamWriteU8((roff>>16), (roff&0xFFFF), d);//pretty slow..TODO add small buffer?
			roff++;
		}
		
	}else{
		printmsg(sdfilemsg);
	}
SPIR_CURSOR_LOAD_FINISH:
	f_close(&f);
	SpiRamCursorUnyield();
	return 1;
}

void CustomWaitVsync(u8 frames){//we do a best effort to keep up to the demand of the song player.
/*
	while(frames){
		if(loopEnd){//we read past the end of the song..luckily it is padded with bytes from the loop start
			SpiRamCursorYield();
			songOff = (songOff-loopEnd)+loopStart;
			loopEnd = 0;//we immediately zero it so we don't keep doing it
			SpiRamSeqReadStart((songBase+songOff)>>16,(songBase+songOff)&0xFFFF);//read from the start of the song, plus the offset we already "read past"
		}

		while(!GetVsyncFlag()){//try to use cycles that we would normally waste

			if((run_flags & DO_SONG_BUFFER) && !SongBufFull()){
				SongBufWrite(SpiRamSeqReadU8());
				songOff++;			
			}		
		}

		ClearVsyncFlag();
		frames--;
	}
*/
}