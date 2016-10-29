#define PRINTK_C

#include "string.h"
#include "../header.h"
#include "boot.h"
#include "spi.h"

int dummy;
// #define mulh(rd,ra,rb) (rd=ra*rb)
#define mulh(rd,ra,rb) __asm{smull dummy,rd,ra,rb} 
#define Xuint32 u32
#define Xint32 int
#define Xuint8 u8
#define Xint8 char

void ad_int_handler();

static int skip_atoi (const char **s)
{
	int i = 0;

	while (is_digit (**s))
		i = i * 10 + *((*s)++) - '0';
	return i;
}

unsigned long _do_div(u32 *n,int base)
{
	int mod;
	
	mod = (*n)%base;
	*n = (*n)/base;
	
	return mod;
}

static char *number (char *str, int num, int base, int size, int precision, int type)
{
	char c, sign, tmp[36];
	const char *digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	int i;

	if (type & SMALL)
		digits = "0123456789abcdefghijklmnopqrstuvwxyz";
	if (type & LEFT)
		type &= ~ZEROPAD;
	if (base < 2 || base > 36)
		return 0;
	c = (type & ZEROPAD) ? '0' : ' ';
	if (type & SIGN && num < 0)
	{
		sign = '-';
		num = -num;
	}
	else
		sign = (type & PLUS) ? '+' : ((type & SPACE) ? ' ' : 0);
	if (sign)
		size--;
	if (type & SPECIAL){
		if (base == 16)
			size -= 2;
		else if (base == 8)
			size--;
	}
	i = 0;
	if (num == 0)
		tmp[i++] = '0';
	else
		while (num != 0){
			tmp[i++] = digits[_do_div((u32*)&num, base)];
		}
	if (i > precision)
		precision = i;
	size -= precision;
	if (!(type & (ZEROPAD + LEFT)))
		while (size-- > 0)
			*str++ = ' ';
	if (sign)
		*str++ = sign;
	if (type & SPECIAL){
		if (base == 8)
			*str++ = '0';
		else if (base == 16)
		{
			*str++ = '0';
			*str++ = digits[33];
		}
	}
	if (!(type & LEFT))
		while (size-- > 0)
			*str++ = c;
	while (i < precision--)
		*str++ = '0';
	while (i-- > 0)
		*str++ = tmp[i];
	while (size-- > 0)
		*str++ = ' ';
	return str;
}

int vsprintk (char *buf, const char *fmt, va_list args)
{
	int len;
	int i;
	char *str;
	char *s;
	int *ip;

	int flags;			/* flags to number() */
	int field_width;		/* width of output field */
	int precision;		/* min. # of digits for integers; max
				   number of chars for from string */
	int qualifier;		/* 'h', 'l', or 'L' for integer fields */
	
	for (str = buf; *fmt; ++fmt)
	{
		if (*fmt != '%')
		{
			*str++ = *fmt;
			continue;
		}

		flags = 0;
repeat:
		++fmt;			/* this also skips first '%' */
		switch (*fmt)
		{
		case '-':
			flags |= LEFT;
			goto repeat;
		case '+':
			flags |= PLUS;
			goto repeat;
		case ' ':
			flags |= SPACE;
			goto repeat;
		case '#':
			flags |= SPECIAL;
			goto repeat;
		case '0':
			flags |= ZEROPAD;
			goto repeat;
		}

/* get field width */
		field_width = -1;
		if (is_digit (*fmt))
			field_width = skip_atoi (&fmt);
		else if (*fmt == '*')
		{
/* it's the next argument */
			field_width = va_arg (args, int);
			if (field_width < 0)
			{
				field_width = -field_width;
				flags |= LEFT;
			}
		}

/* get the precision */
		precision = -1;
		if (*fmt == '.')
		{
			++fmt;
			if (is_digit (*fmt))
				precision = skip_atoi (&fmt);
			else if (*fmt == '*')
			{
/* it's the next argument */
				precision = va_arg (args, int);
			}
			if (precision < 0)
			precision = 0;
		}

/* get the conversion qualifier */
		qualifier = -1;
		if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L')
		{
			qualifier = *fmt;
			++fmt;
		}
		switch (*fmt)
		{
		case 'c':
			if (!(flags & LEFT))
				while (--field_width > 0)
					*str++ = ' ';
			*str++ = (unsigned char) va_arg (args, int);
			while (--field_width > 0)
				*str++ = ' ';
			break;

		case 's':
			s = va_arg (args, char *);
			len = strlen (s);
			if (precision < 0)
				precision = len;
			else if (len > precision)
				len = precision;

			if (!(flags & LEFT))
				while (len < field_width--)
					*str++ = ' ';
			for (i = 0; i < len; ++i)
				*str++ = *s++;
			while (len < field_width--)
				*str++ = ' ';
			break;

		case 'o':
			str = number (str, va_arg (args, unsigned long), 8,
									field_width, precision, flags);
			break;

		case 'p':
			if (field_width == -1)
			{
				field_width = 8;
				flags |= ZEROPAD;
			}
			str = number (str, (unsigned long) va_arg (args, void *), 16,
							field_width, precision, flags);
			break;

		case 'x':
			flags |= SMALL;
		case 'X':
			str = number (str, va_arg (args, unsigned long), 16,
									field_width, precision, flags);
			break;

		case 'd':
		case 'i':
			flags |= SIGN;
		case 'u':
			str = number (str, va_arg (args, unsigned long), 10,
									field_width, precision, flags);
			break;
		case 'n':
			ip = va_arg (args, int *);
			*ip = (str - buf);
			break;
		default:
			if (*fmt != '%')
				*str++ = '%';
			if (*fmt)
				*str++ = *fmt;
			else
				--fmt;
			break;
		}
	}
	*str = '\0';
	return str - buf;
}

int sprintk(char * buf, const char *fmt, ...)
{
	va_list args;
	int i;

	var_start(args, fmt);
	i=vsprintk(buf,fmt,args);
	va_end(args);
	return i;
}

int printk (const char *fmt, ...)
{
	va_list args;
	int i;	
	char print_buf[81];
	#if 1
	var_start(args, fmt);
	i = vsprintk(print_buf, fmt, args);
	va_end(args);
	uart_prints(print_buf);
	#endif
	return i;
}

int uart_rx_byte_1s(u8 *cval)
{
	int i;

	for(i=0;i<1200000;i++){
		if((UART_STATUS & URX_READY))
			break;
	}

	*cval = UART_R_DATA;

	if(i == 1200000)
		return -1;
	else
		return 0;
}

u8 uart_rx_byte(void)
{
	while (!(UART_STATUS & URX_READY));
	return UART_R_DATA;
}


void uart_tx_byte(u8 data)
{
	while (UART_STATUS & UTX_BUSY);
	UART_W_DATA = data;
}

void uart_prints(char *pstr)
{
	while(*pstr)
		uart_tx_byte(*pstr++);
}

void uart_init(u16 baud, u16 ctrl)
{
	UART_BAUD = baud;
	UART_CTRL = ctrl;
}

void init_uart(u32 sysclk,u32 baudrate)
{
	uart_init(sysclk/4/baudrate, UART_CTRL_WL8 | UART_CTRL_TX_EN | UART_CTRL_STOP1 | UART_CTRL_NO_PAR | UART_CTRL_RX_EN);
}

void remap_chip(void)
{
	(*((vu32*)(P_REMAP)))  = 0x01;	//remap	0x7000 0000 to 0x00000000
	(*((void (*)(void))0))();				//longjump
	for(;;);
}

u32 firmware_number(void)
{
	return 0x20111101;
}


void load_memorry()
{	
	u8 *psrc,*pdst;
	u32 sz,i;

	psrc = (unsigned char*)&Load$$ROM_BOOT$$Base;
	pdst = (unsigned char*)&Image$$ROM_BOOT$$Base;
	sz = (unsigned long)&Image$$ROM_BOOT$$Length;
	if(psrc == pdst)
		sz = 0;
	for(i=sz;i>0;i--)
		pdst[i-1] =psrc[i-1];		
	

// Clear ZI must at the end
	pdst = (unsigned char*)&Image$$ROM_BOOT$$ZI$$Base;
	sz = (unsigned long)&Image$$ROM_BOOT$$ZI$$Length;
	for(i=0;i<sz;i++)
		*pdst++ = 0;	
}


void delay_1us(u32 sysclk/*MHZ*/)
{	
	u32 cnter;

	cnter = 1*sysclk/2;
	TIMER0_CURR_CNT = cnter;
	TIMER_CTRL = 2<<2|1<<6;
	while(TIMER0_CURR_CNT!=0xffff);
}


void delay_1ms(u32 sysclk/*MHZ*/)
{	
	u32 cnter;

	cnter = 1000*sysclk/2;
	TIMER0_CURR_CNT = cnter;
	TIMER_CTRL = 2<<2|1<<6;
	while(TIMER0_CURR_CNT!=0xffff);
}

void delay_1s(u32 sysclk/*MHZ*/)
{	
	u32 cnter;

	cnter = 1000*sysclk/128;
	//TIMER1_LOAD_CNT = cnter;
	TIMER1_CURR_CNT = cnter;
	TIMER_CTRL = /*1<<1|*/1<<4|1<<7;
	while(TIMER1_CURR_CNT!=0xffff);
}

void spi_init()
{
	SSI_BAUDR = 2;
	SSI_TXFTLR = 0;
	SSI_RXFTLR = 0;	
	GPIO_A_WDATA = 1<<7;
}

void spi_en()
{
	SSI_SER = 1;
	SSI_SSIENR = 1;
}

void spi_dis()
{
	SSI_SER = 0;
	SSI_SSIENR = 0;
}
u8 w25x_read_status(void)   
{   
	u8 status = 0;   
	
	SSI_CTRLR0 = 3<<8|7;
	SSI_CTRLR1 = 1;
	spi_en();
	GPIO_A_WDATA = 0;
	
	SSI_DR = 0x05;
	while((SSI_SR&(1<<3))==0);
	status = SSI_DR;
	while((SSI_SR&(1<<3))==0);
	status = SSI_DR;

	while(SSI_SR&1);
	
	GPIO_A_WDATA = 1<<7;
	spi_dis();
	
	return status;   
} 

void w25x_write_status(u8 status)   
{	
	SSI_CTRLR0 = 1<<8|7;
	SSI_CTRLR1 = 1;
	spi_en();
	GPIO_A_WDATA = 0;	
	SSI_DR = 0x01;
	SSI_DR = status;
	while(SSI_SR&1);
	GPIO_A_WDATA = 1<<7;
	spi_dis();

} 


void w25x_read(u8 *p_data,u32 addr,int len)
{
	SSI_CTRLR0 = 3<<8|7;
	SSI_CTRLR1 = len-1;
	
	spi_en();
	GPIO_A_WDATA = 0;
	
	SSI_DR = 0x03;//set read en
	SSI_DR = addr>>16;
	SSI_DR = addr>>8;
	SSI_DR = (u8)addr;
		
	while(len--){
		while((SSI_SR&(1<<3))==0);
		*p_data++ = SSI_DR;
	}

	while(SSI_SR&1);
	
	GPIO_A_WDATA = 1<<7;
	spi_dis();
}

u16 w25x_read_id(void)
{
	int cnt;
	u16 id;

	cnt = 2;
	id = 0;
	
	SSI_CTRLR0 = 3<<8|7;
	SSI_CTRLR1 = cnt-1;
	
	spi_en();
	GPIO_A_WDATA = 0;
	
	SSI_DR = 0x90;//set read en
	SSI_DR = 0;
	SSI_DR = 0;
	SSI_DR = 0;

	while(cnt--){
		while((SSI_SR&(1<<3))==0);
		id |= SSI_DR<<(cnt*8);
	}	

	while(SSI_SR&1);
	
	GPIO_A_WDATA = 1<<7;
	spi_dis();
	
	return id;
	
}

u32 w25x_read_jedec(void)
{
	int cnt;
	u32 id;

	cnt = 3;
	id = 0;

	SSI_CTRLR0 = 3<<8|7;
	SSI_CTRLR1 = cnt-1;
	
	spi_en();
	GPIO_A_WDATA = 0;
	
	SSI_DR = 0x9f;//set read en

	while(cnt--){
		while((SSI_SR&(1<<3))==0);
		id |= SSI_DR<<(cnt*8);
	}

	while(SSI_SR&1);

	GPIO_A_WDATA = 1<<7;
	spi_dis();
	
	return id;
	
}
 
void w25x_write_enable(void)
{
	SSI_CTRLR0 = 1<<8|7;
	SSI_CTRLR1 = 0;
	spi_en();
	GPIO_A_WDATA = 0;

	SSI_DR = 0x06;

	while((SSI_SR&(1<<2))==0);
	while(SSI_SR&1);

	GPIO_A_WDATA = 1<<7;
	spi_dis();
}

void w25x_write_disable(void)
{
	SSI_CTRLR0 = 1<<8|7;
	spi_en();
	GPIO_A_WDATA = 0;
	
	SSI_DR = 0x04;

	while((SSI_SR&(1<<2))==0);
	while(SSI_SR&1);
	
	GPIO_A_WDATA = 1<<7;
	spi_dis();
}


void w25x_write_page(u8 *p_data,u32 addr)
{
	int len;
	u8 status;

	w25x_write_enable();

	do{
		status = w25x_read_status();
	}while((status&2)==0);
	
	len = 256;
	
	SSI_CTRLR0 = 1<<8|7;
	SSI_CTRLR1 = 255;
	
	spi_en();
	GPIO_A_WDATA = 0;
	
	SSI_DR = 0x02;
	SSI_DR = addr>>16;
	SSI_DR = addr>>8;
	SSI_DR = (u8)addr;

	while((SSI_SR&(1<<2))==0);
		
	while(len){
		SSI_DR = *p_data++;
		SSI_DR = *p_data++;
		SSI_DR = *p_data++;
		SSI_DR = *p_data++;
		SSI_DR = *p_data++;
		SSI_DR = *p_data++;
		SSI_DR = *p_data++;
		SSI_DR = *p_data++;
		len -= 8;
		while((SSI_SR&(1<<2))==0);
	}

	while(SSI_SR&1);
	
	GPIO_A_WDATA = 1<<7;
	spi_dis();

	w25x_write_disable();

	do{
		status = w25x_read_status();
	}while((status&3)!=0);
	
}

void w25x_chip_erase(void)   
{                          
	u8 status;

	w25x_write_enable();

	do{
		status = w25x_read_status();
	}while((status&2)==0);

	SSI_CTRLR0 = 1<<8|7;
	SSI_CTRLR1 = 0;
	spi_en();
	GPIO_A_WDATA = 0;

	SSI_DR = 0xc7;

	while((SSI_SR&(1<<2))==0);
	while(SSI_SR&1);

	GPIO_A_WDATA = 1<<7;
	spi_dis();
	
    	w25x_write_disable();

	do{
		status = w25x_read_status();
	}while((status&3)!=0);
        
}   

void w25x_block_erase(u32 addr)   
{                          
	u8 status;

	w25x_write_enable();

	do{
		status = w25x_read_status();
	}while((status&2)==0);

	SSI_CTRLR0 = 1<<8|7;
	SSI_CTRLR1 = 0;
	spi_en();
	GPIO_A_WDATA = 0;

	SSI_DR = 0xd8;
	SSI_DR = addr>>16;
	SSI_DR = addr>>8;
	SSI_DR = (u8)addr;

	while((SSI_SR&(1<<2))==0);
	while(SSI_SR&1);

	GPIO_A_WDATA = 1<<7;
	spi_dis();
	
    	w25x_write_disable();

	do{
		status = w25x_read_status();
	}while((status&3)!=0);
        
}   


void w25x_sector_erase(u32 addr)   
{                          
	u8 status;

	w25x_write_enable();

	do{
		status = w25x_read_status();
	}while((status&2)==0);

	SSI_CTRLR0 = 1<<8|7;
	SSI_CTRLR1 = 0;
	spi_en();
	GPIO_A_WDATA = 0;

	SSI_DR = 0x20;
	SSI_DR = addr>>16;
	SSI_DR = addr>>8;
	SSI_DR = (u8)addr;

	while((SSI_SR&(1<<2))==0);
	while(SSI_SR&1);

	GPIO_A_WDATA = 1<<7;
	spi_dis();
	
    	w25x_write_disable();

	do{
		status = w25x_read_status();
	}while((status&3)!=0);
        
}   

void write_spi_boot(u8*p_boot,int szboot)
{
	int i,j;
	u8 *p8,page_cnt;
	
	gpio_set_adir(0,1<<7);

	pin_set_ssi();
	spi_init();
	
	w25x_block_erase(0);
	
	page_cnt = (szboot+255)/256;

	p8 = (u8*)p_boot;
	for(i=0;i<page_cnt;i++){
		w25x_write_page(p8,i*256);
		p8 += 256;
	}
}

u16 crc16_update (u16 crc, u8 data)  
{  
	int i; 
	
	crc = crc ^ ((u16)data << 8);  
	for (i=0; i<8; i++){  
		if (crc & 0x8000)  
			crc = (crc << 1) ^ CRC16_POLY;  
		else  
			crc <<= 1;  
	}  
	return crc;  
}  

   
u16 calcrc16(u8 *ptr, u32 count)   
{   
    u16 crc = 0;   
	
    while (count--){   
        crc =crc16_update(crc,*ptr++);   
    }

    return crc;   
}
   
int xmodem(u8 *paddr)   
{   
	u16 crc;   
	int i;
	u32 bcode_sz;
	u8 *p8,*pval;
	u8 sec_cnt,blk_cnt,STATUS;
	XMODEM xstr;

	STATUS=ST_WAIT_START;
	
	blk_cnt = 0x01;
	pval = (u8*)&xstr;
	
	while(STATUS!=ST_OK){   
		while(STATUS==ST_WAIT_START){
			uart_tx_byte(XMODEM_WAIT_CHAR);
			if(uart_rx_byte_1s(&pval[0])==0)
				break;
		}

		if(STATUS != ST_WAIT_START)
			pval[0] = uart_rx_byte();
		for(i=1;i<133;i++){
			if(uart_rx_byte_1s(&pval[i]))
				break;
		}
		
		switch(xstr.SOH){   
			case XMODEM_SOH:
				STATUS=ST_BLOCK_OK;   
				break;   
			case XMODEM_EOT:
				uart_tx_byte(XMODEM_ACK);
				STATUS=ST_OK;   
				break;   
			case XMODEM_CAN:
				uart_tx_byte(XMODEM_ACK);
				STATUS=ST_OK;   
				break;   
			default:
				uart_tx_byte(XMODEM_NAK);
				STATUS=ST_BLOCK_FAIL;   
				break;   
		}   

		if (STATUS==ST_BLOCK_OK){   
			if (blk_cnt != xstr.blk_no){   
				uart_tx_byte(XMODEM_NAK);
				continue;   
			}
			if (blk_cnt !=(u8)(~xstr.n_blk_no)){   
				uart_tx_byte(XMODEM_NAK);
				continue;   
			}
			
			crc=xstr.crc16_hi<<8;   
			crc+=xstr.crc16_lo;   
			
			if(calcrc16(&xstr.data[0],DATA_BUFFER_SIZE)!=crc){   
				uart_tx_byte(XMODEM_NAK);
				continue;   
			}
			
			for(i=0;i<128;i++){
				*paddr++ = xstr.data[i];
			} 
		   
			uart_tx_byte(XMODEM_ACK);
			blk_cnt++;
		}   
	}      


	return blk_cnt-1;
}   

void pmc_set_clk(u16 set)
{
	PMC_CLK_SEL = 0;
	PMC_PLLEN = 0;
	PMC_CFG = set;
	PMC_PLLEN = 1;
	PMC_CLK_SEL = 1;
}

void pmc_clr_pll(void)
{
	PMC_CLK_SEL = 0;
}


void pmc_set_idle(void)
{
	PMC_IDLE = 1;
	__asm{
		nop;
		nop;
		nop;
		nop;
	}
}

void pmc_set_sleep(u8 wakeup_pin)
{
	u32 cpsr,val;

	cpsr = arm_save_disable_int();

	PMC_WAKEUP = 1<<(wakeup_pin&7);
	val = PMC_CFG;
	
	PMC_STOP = 0x785a69a5;
	__asm{
		nop;
		nop;
		nop;
		nop;
	}

	pmc_set_clk((u16)val);
	
	arm_restore_int(cpsr);
}

void Undefined()
{
	#ifdef DEBUG
	printk("undefined\r\n");
	#endif
	for(;;);
}

void  SWI()
{
	#ifdef DEBUG
	printk("swi\r\n");
	#endif
	for(;;);
}

void Prefetch()
{
	#ifdef DEBUG
	printk("prefetch\r\n");
	#endif
	for(;;);
}

void Abort()
{
	#ifdef DEBUG
	printk("abort\r\n");
	#endif
	for(;;);
}
	
void intc_init(void)
{
	INTC_EN = 1<<30;
	INTC_MASK = ~(1<<30);
	INTC_REQ = 1<<30;
}


void irq_handler(void)
{
	u32 req;

	req = INTC_REQ;
	INTC_REQ = req;

	if(req&(1<<28)){
		// adda_int_handler();
		ad_int_handler();
	}
}

void pin_set_si45_uart0(void)
{
	u32 val;
	
	val = SI_PINMUX;
	val &= ~(0xF<<8);
	val |= 0x5<<8;
	SI_PINMUX = val;

	val = GPIO_A_MODE_H;
	val &= ~(0xF<<0);
	val |= 0x5<<0;
	GPIO_A_MODE_H = val;
}

void pin_set_si23_uart1(void)
{
	u32 val;
	
	val = SI_PINMUX;
	val &= ~(0xF<<4);
	val |= 0x5<<4;
	SI_PINMUX = val;

	val = GPIO_A_MODE_L;
	val &= ~(0xF<<28);
	val |= 0x5<<28;
	GPIO_A_MODE_L = val;
}

void pin_set_si01_uart1(void)
{
	u32 val;
	
	val = SI_PINMUX;
	val &= ~(0xF<<0);
	val |= 0x5<<0;
	SI_PINMUX = val;

	val = GPIO_A_MODE_L;
	val &= ~(0xF<<24);
	val |= 0x5<<24;
	GPIO_A_MODE_L = val;
}

void pin_set_si45_iic0(void)
{
	u32 val;
	
	val = SI_PINMUX;
	val &= ~(0xF<<8);
	val |= 0xA<<8;
	SI_PINMUX = val;

	val = GPIO_A_MODE_H;
	val &= ~(0xF<<0);
	val |= 0x5<<0;
	GPIO_A_MODE_H = val;
}

void pin_set_si23_iic1(void)
{
	u32 val;
	
	val = SI_PINMUX;
	val &= ~(0xF<<4);
	val |= 0xA<<4;
	SI_PINMUX = val;

	val = GPIO_A_MODE_L;
	val &= ~(0xF<<28);
	val |= 0x5<<28;
	GPIO_A_MODE_L = val;
}

void pin_set_si01_iic1(void)
{
	u32 val;
	
	val = SI_PINMUX;
	val &= ~(0xF<<0);
	val |= 0xA<<0;
	SI_PINMUX = val;

	val = GPIO_A_MODE_L;
	val &= ~(0xF<<24);
	val |= 0x5<<24;
	GPIO_A_MODE_L = val;
}

void pin_set_si345_spi0(void)
{
	u32 val;
	
	val = SI_PINMUX;
	val &= ~(0x3F<<6);
	SI_PINMUX = val;

	val = GPIO_A_MODE_L;
	val &= ~(0x3<<30);
	val |= 0x1<<30;
	GPIO_A_MODE_L = val;

	val = GPIO_A_MODE_H;
	val &= ~(0xF<<0);
	val |= 0x5<<0;
	GPIO_A_MODE_H = val;
}

void pin_set_si012_spi1(void)
{
	u32 val;
	
	val = SI_PINMUX;
	val &= ~(0x3F<<0);
	SI_PINMUX = val;

	val = GPIO_A_MODE_L;
	val &= ~(0x3F<<24);
	val |= 0x15<<24;
	GPIO_A_MODE_L = val;
}

void pin_set_adda(void)
{
	u32 val;
	
	val = GPIO_A_MODE_L;
	val &= ~(0xFFFF<<16);
	val |= 0xAA<<24|0x55<<16;
	GPIO_A_MODE_L = val;

	val = GPIO_A_MODE_H;
	val &= ~(0xF);
	val |= 0xA;
	GPIO_A_MODE_H = val;
}

void pin_set_ssi(void)
{
	u32 val;
	
	val = GPIO_A_MODE_L;
	val &= ~(0x3F<<8);
	val |= 0x15<<8;
	GPIO_A_MODE_L = val;
}

void pin_set_uart(void)
{
	u32 val;
	
	val = GPIO_A_MODE_H;
	val &= ~(0xF<<4);
	val |= 0x5<<4;
	GPIO_A_MODE_H = val;
}

void pin_set_al(u32 mask,u32 set)
{
	u32 val;
	
	val = GPIO_A_MODE_L;
	val &= mask;
	val |= set;
	GPIO_A_MODE_L = val;
}

void pin_set_ah(u32 mask,u32 set)
{
	u32 val;
	
	val = GPIO_A_MODE_H;
	val &= mask;
	val |= set;
	GPIO_A_MODE_H = val;
}


void gpio_set_adir(u32 mask,u32 set)
{
	u32 val;
	
	val = GPIO_A_DIR;
	val &= mask;
	val |= set;
	GPIO_A_DIR = val;
}

void gpio_set_aout(u32 mask,u32 set)
{
	GPIO_A_WDATA = mask&set;
}

void gpio_get_ain(u32 mask,u32 *get)
{	
	*get = GPIO_A_RDATA&mask;
}

void gpio_set_bdir(u32 mask,u32 set)
{
	u32 val;
	
	val = GPIO_B_DIR;
	val &= mask;
	val |= set;
	GPIO_B_DIR = val;
}

void gpio_set_bout(u32 mask,u32 set)
{
	GPIO_B_WDATA = mask&set;
}

void gpio_get_bin(u32 mask,u32 *get)
{	
	*get = GPIO_B_RDATA&mask;
}


const u16 pcm16k2chan[162] = 
{
	0x0000, 0x0000, 0x06F2, 0x06F2, 0x0DDA, 0x0DDA, 0x14AC, 0x14AC, 0x1B5D, 0x1B5D, 0x21E3, 0x21E3, 0x2833, 0x2833, 0x2E44, 
	0x2E44, 0x340D, 0x340D, 0x3983, 0x3983, 0x3E9D, 0x3E9D, 0x4355, 0x4355, 0x47A4, 0x47A4, 0x4B81, 0x4B81, 0x4EE7, 0x4EE7, 
	0x51D0, 0x51D0, 0x5438, 0x5438, 0x561B, 0x561B, 0x5775, 0x5775, 0x5848, 0x5848, 0x588D, 0x588D, 0x5848, 0x5848, 0x5776, 
	0x5776, 0x561B, 0x561B, 0x5438, 0x5438, 0x51D0, 0x51D0, 0x4EE7, 0x4EE7, 0x4B80, 0x4B80, 0x47A4, 0x47A4, 0x4356, 0x4356, 
	0x3E9D, 0x3E9D, 0x3983, 0x3983, 0x340C, 0x340C, 0x2E44, 0x2E44, 0x2834, 0x2834, 0x21E3, 0x21E3, 0x1B5C, 0x1B5C, 0x14AC, 
	0x14AC, 0x0DDA, 0x0DDA, 0x06F3, 0x06F3, 0x0000, 0x0000, 0xF90D, 0xF90D, 0xF226, 0xF226, 0xEB54, 0xEB54, 0xE4A3, 0xE4A3, 
	0xDE1C, 0xDE1C, 0xD7CC, 0xD7CC, 0xD1BC, 0xD1BC, 0xCBF3, 0xCBF3, 0xC67D, 0xC67D, 0xC162, 0xC162, 0xBCAA, 0xBCAA, 0xB85C, 
	0xB85C, 0xB47F, 0xB47F, 0xB119, 0xB119, 0xAE31, 0xAE31, 0xABC8, 0xABC8, 0xA9E5, 0xA9E5, 0xA88A, 0xA88A, 0xA7B9, 0xA7B9, 
	0xA773, 0xA773, 0xA7B9, 0xA7B9, 0xA88A, 0xA88A, 0xA9E6, 0xA9E6, 0xABC8, 0xABC8, 0xAE31, 0xAE31, 0xB11A, 0xB11A, 0xB47F, 
	0xB47F, 0xB85D, 0xB85D, 0xBCAA, 0xBCAA, 0xC162, 0xC162, 0xC67E, 0xC67E, 0xCBF3, 0xCBF3, 0xD1BC, 0xD1BC, 0xD7CD, 0xD7CD, 
	0xDE1D, 0xDE1D, 0xE4A3, 0xE4A3, 0xEB54, 0xEB54, 0xF225, 0xF225, 0xF90D, 0xF90D, 0x0000, 0x0000
};


const u16 sin1k[49] = {0x0000, 0xFC63, 0xF8D6, 0xF567, 0xF229, 0xEF26, 0xEC6D, 0xEA09, 0xE806, 0xE66D, 0xE543, 0xE48F, 0xE451, 0xE48E, 0xE543, 
                            0xE66E, 0xE807, 0xEA09, 0xEC6D, 0xEF26, 0xF229, 0xF569, 0xF8D5, 0xFC63, 0xFFFF, 0x039D, 0x072A, 0x0A98, 0x0DD8, 0x10DA, 
                            0x1393, 0x15F6, 0x17F9, 0x1993, 0x1ABD, 0x1B71, 0x1BAE, 0x1B72, 0x1ABD, 0x1992, 0x17FA, 0x15F5, 0x1393, 0x10DA, 0x0DD7, 
                            0x0A98, 0x072A, 0x039D, 0x0000};


static u32 jj = 0;
static int iflag = 0;
static int ad_data0 = 0;
static int ad_data1 = 0;
static int ad_data2 = 0;
static int ad_data3 = 0;
static int ii=0;

void adda_int_handler(void)
{
	ad_data0 = AD_RX_DATA;
	ad_data1 = AD_RX_DATA;
	ad_data2 = AD_RX_DATA;
	ad_data3 = AD_RX_DATA;

	// SINCOS_DIN = i*(0xFFFFFFFF>>6);
	jj += 268435456;
	SINCOS_DIN = jj;
	// DA_TX_DATA = (((u16)((SINCOS_DSINE>>16)+0x8000)>>2)&0xFFFF)|((((u16)((SINCOS_DCOSE>>16)+0x8000)>>2)&0xFFFF)<<16);
	DA_TX_DATA = (((u16)((SINCOS_DSINE>>16)+0x8000)>>2)&0xFFFF)|((((u16)((0>>16)+0x8000)>>2)&0xFFFF)<<16);
	//DA_TX_DATA = ((sin1k[i]+0x8000)&0xFFFF)|(((sin1k[i]+0x8000)&0xFFFF)<<16);

	ii++;
	if(ii>=1000) iflag = 1;
}

/***************** main gyro control program below ***************/
Xuint32 i = 0,j = 0,k = 0;

Xint32 phi_drvout0 = 0, f_adj0 = 0, drv_gain0_fix = 47416438, drvout_hp = 0, drvs_hp = 0, sens_hp = 0, bals_hp = 0;
Xint32 Wdrvout1 = 0, Wdrvout2 = 0, delayph_drv1 = 0, delayamp_drv1 = 0, ref_drvi_hp = 0, ref_drvq_hp = 0;
Xint32 Wdrvs1 = 0, Wdrvs2 = 0, Wsens1 = 0, Wsens2 = 0, delayf_drv1 = 0;
Xint32 adrslt1 = 0, adrslt2 = 0, adrslt3 = 0, adrslt4 = 0, phi_adj = 0;
Xuint8 a1 = 0,b1 = 0,c1 = 0,d1 = 0,a2 = 0,b2 = 0,c2 = 0,d2 = 0,a3 = 0,b3 = 0,c3 = 0,d3 = 0,a4 = 0,b4 = 0,c4 = 0,d4 = 0;
Xint32 drvsi_lpa1 = 0, drvsi_lpa2 = 0,drvsi_lpb1 = 0, drvsi_lpb2 = 0;
Xint32 drvsq_lpa1 = 0, drvsq_lpa2 = 0,drvsq_lpb1 = 0, drvsq_lpb2 = 0;
Xint32 sensi_lpa1 = 0, sensi_lpa2 = 0,sensi_lpb1 = 0, sensi_lpb2 = 0;
Xint32 sensq_lpa1 = 0, sensq_lpa2 = 0,sensq_lpb1 = 0, sensq_lpb2 = 0;

Xint32 drvouti_lpa1 = 0, drvouti_lpa2 = 0,drvouti_lpb1 = 0, drvouti_lpb2 = 0;
Xint32 drvoutq_lpa1 = 0, drvoutq_lpa2 = 0,drvoutq_lpb1 = 0, drvoutq_lpb2 = 0;

Xint32 senouti_lpa1 = 0;
Xint32 senoutq_lpa1 = 0;

Xint32 ref_drvi_fix = 0,ref_drvq_fix = 1,ref_drvi_fix1 = 0, ref_drvq_fix1 = 1,ref_drvi_fix2 = 0, ref_drvq_fix2 = 1,daout = 0,drvout_fix = 0,temp1 = 0,temp2 = 0;
Xint32 ref_drvi_fix3 = 0, ref_drvq_fix3 = 0;
Xint32 y_drvout = 0, err_drvout = 0, miu_drvout_fix = 128849019, drvoutq = 0, drvouti = 0;
Xint32 delayph_drv2 = 0, phi_drvout = 0, kp_drvout_fix = 429496730, ki_drvout_fix = 420906795;

Xint32 drvs_fix = 0, y_drvs = 0, err_drvs = 0, miu_drvs_fix = 128849019, drvsq = 0, drvsi = 0;
Xint32 delayf_drv2 = 0, f_adj = 0, kp_drvs_fix = 42949673*30, ki_drvs_fix = 42793774*30, drv_freq = 0;
Xint32 drvamp_goal_fix = 94832872 * 0.3, delayamp_drv2 = 0, drv_gain_fix =47416438, kpamp_drvs_fix = 42949673*30, kiamp_drvs_fix = 42794925*30;
Xint32 sens_fix = 0, y_sens = 0, err_sens = 0, miu_sens_fix = 128849019 * 6, sensq = 0, sensi = 0;

// define online compensation values
Xuint32  com_phi = 0;
Xint32 com_drvsin = 0, com_drvcos = 0, freq_comp = 300000000;
Xint32 com_refsin1 = 0, com_refcos1 = 0;
Xint32 com_refsin2 = 0, com_refcos2 = 0;
Xint32 drvs_gain_adj = 1431655765;
	// drvout compensation values
Xint32 Wdrvobv1 = 0, Wdrvobv2 = 0, y_drvobv = 0, err_drvobv = 0, miu_drvobv_fix = 128849019;
Xint32 drvobvi = 0, drvobvq = 0;
Xint32 drvobvi_sum = 0;
Xint32 drvobvi_lpa1 = 0;
	// drvs compensation values
Xint32 Wdrvsobv1 = 0, Wdrvsobv2 = 0, y_drvsobv = 0, err_drvsobv = 0, miu_drvsobv_fix = 128849019;
Xint32 drvsobvi = 0, drvsobvq = 0;
Xint32 drvsobvi_sum = 0;
Xint32 drvsobvi_lpa1 = 0;
// end of compensation values
Xint32 sum_freq = 0;

Xint32 darslt;
Xint32 deltaphi;
Xint32 temp3;

Xint32 sensi0, sensq0;
Xint32 delayseni1 = 0, delayseni2 = 0, delayseni3 = 0, sen_gaini = 0, sen_gaini0 = 0;
Xint32 kpseni = 42949673*4, kiseni = 40949673*4, kdseni = 00000*4;// kpseni = 42949673*30, kiseni = 41003774*30, kdseni = 40000*30;
Xint32 delaysenq1 = 0, delaysenq2 = 0, delaysenq3 = 0, sen_gainq = 0, sen_gainq0 = 0;
Xint32 kpsenq = 42949673*4, kisenq = 40949673*4, kdsenq = 00000*4;// kpsenq = 42949673*30, kisenq = 41003774*30, kdsenq = 40000*30;
Xint32 senout = 0, temp3 = 0;
Xint32 bals_fix = 0, y_bals = 0, err_bals = 0, miu_bals_fix = 128849019 * 6, Wbals1 = 0, Wbals2 = 0;
Xint32 phi_refdrv = 0;

Xint32	data_freq[120];			// saving averaging frequency data
Xint32	data_freq_old = 0;	// old value of freq
Xint32	sum_data_freq = 0;	// sum of 120 freq data

Xint32	data_amp[120];			// saving averaging amplitude data of oscillation
Xint32	data_amp_old = 0;		// old value of amp
Xint32	sum_data_amp = 0;		// sum of 120 amp data

Xint32	data_inph[120];			// saving averaging inphase gyro output data
Xint32	data_inph_old = 0;	// old value of inph
Xint32	sum_data_inph = 0;	// sum of 120 inph data

Xint32	data_quad[120];			// saving averaging quadrature gyro output data
Xint32	data_quad_old = 0;	// old value of quad
Xint32	sum_data_quad = 0;	// sum of 120 quad data

Xint32	sp_data = 0;				// point to data
Xint8		bstart_tx = 0;			// start transmit data
Xint8		tx_data = 0;				// transmit data
Xint8		btxstat = 0;				// transmit status
Xint32	chksum = 0;				// checksum of transmition data
Xint8 rxdata;
Xint32 sengaini_lpa1 = 0, sengainq_lpa1 = 0, freq_lpa1 = 0, drvamp_lpa1 = 0;

int bSelfTestOK = 0;			// For power on self test, to check the +12V and -12V is OK


int lp_filter1(int x) // 50Hz filter
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-drvobvi_lpa1;
 mulh(hptemp1,hptemp2,12884902); //1288490
 drvobvi_lpa1 =  hptemp1 + drvobvi_lpa1;
 return drvobvi_lpa1;
}

int lp_filter2(int x)
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-drvsobvi_lpa1;
 mulh(hptemp1,hptemp2,12884902);
 drvsobvi_lpa1 =  hptemp1 + drvsobvi_lpa1;
 return drvsobvi_lpa1;
}

Xint32 lp_filter5(Xint32 x) // 100Hz Low pass filter
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-drvouti_lpa1;
 mulh(hptemp1,hptemp2,27928105);
 drvouti_lpa1 =  hptemp1 + drvouti_lpa1;
 return drvouti_lpa1;
}

Xint32 lp_filter6(Xint32 x) // 100Hz Low pass filter
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-drvoutq_lpa1;
 mulh(hptemp1,hptemp2,27928105);
 drvoutq_lpa1 =  hptemp1 + drvoutq_lpa1;
 return drvoutq_lpa1;
}

// 200Hz lowpass filter
Xint32 lp_filter7(Xint32 x)
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-senouti_lpa1;
 mulh(hptemp1,hptemp2,112441980);
 senouti_lpa1 =  hptemp1 + senouti_lpa1;
 return senouti_lpa1;
}

// 200Hz lowpass filter
Xint32 lp_filter8(Xint32 x)
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-senoutq_lpa1;
 mulh(hptemp1,hptemp2,112441980);
 senoutq_lpa1 =  hptemp1 + senoutq_lpa1;
 return senoutq_lpa1;
}

// 100Hz lowpass filter
Xint32 lp_filter9(Xint32 x)
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-sengaini_lpa1;
 mulh(hptemp1,hptemp2,64424509);
 sengaini_lpa1 =  hptemp1 + sengaini_lpa1;
 return sengaini_lpa1;
}

// 100Hz lowpass filter
Xint32 lp_filter10(Xint32 x)
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-sengainq_lpa1;
 mulh(hptemp1,hptemp2,64424509);
 sengainq_lpa1 =  hptemp1 + sengainq_lpa1;
 return sengainq_lpa1;
}

// 3Hz lowpass filter
Xint32 lp_filter11(Xint32 x)
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-freq_lpa1;
 mulh(hptemp1,hptemp2,1288490);
 freq_lpa1 =  hptemp1 + freq_lpa1;
 return freq_lpa1;
}

// 3Hz lowpass filter
Xint32 lp_filter12(Xint32 x)
{
 Xint32 hptemp1,hptemp2;
 hptemp2 = x-drvamp_lpa1;
 mulh(hptemp1,hptemp2,1288490);
 drvamp_lpa1 =  hptemp1 + drvamp_lpa1;
 return drvamp_lpa1;
}

Xint32 hp_filter1(Xint32 x)
{
 Xint32 y,hptemp1,hptemp2;
 mulh(x,x,2099383714);
 hptemp1 = drvout_hp;
 mulh(hptemp2,hptemp1,2051283780);
 drvout_hp = 2 * (hptemp2 + x);
 y = drvout_hp - hptemp1;
 return y;
}
Xint32 hp_filter2(Xint32 x)
{
 Xint32 y,hptemp1,hptemp2;
 mulh(x,x,2099383714);
 hptemp1 = drvs_hp;
 mulh(hptemp2,hptemp1,2051283780);
 drvs_hp = 2 * (hptemp2 + x);
 y = drvs_hp - hptemp1;
 return y;
}
Xint32 hp_filter3(Xint32 x)
{
 Xint32 y,hptemp1,hptemp2;
 mulh(x,x,2099383714);
 hptemp1 = sens_hp;
 mulh(hptemp2,hptemp1,2051283780);
 sens_hp = 2 * (hptemp2 + x);
 y = sens_hp - hptemp1;
 return y;
}
Xint32 hp_filter4(Xint32 x)
{
 Xint32 y,hptemp1,hptemp2;
 mulh(x,x,2099383714);
 hptemp1 = bals_hp;
 mulh(hptemp2,hptemp1,2051283780);
 bals_hp = 2 * (hptemp2 + x);
 y = bals_hp - hptemp1;
 return y;
}

/*
Xint32 hp_filter2(Xint32 x)
{
 Xint32 y,hptemp1,hptemp2;
 hptemp1 = drvs_hp;
 mulh(hptemp2,hptemp1,2064765799);
 drvs_hp = 2 * hptemp2 + x;
 mulh(y,2106124616,(drvs_hp - hptemp1));
 return y;
}

Xint32 hp_filter3(Xint32 x)
{
 Xint32 y,hptemp1,hptemp2;
 hptemp1 = sens_hp;
 mulh(hptemp2,hptemp1,2064765799);
 sens_hp = 2 * hptemp2 + x;
 mulh(y,2106124616,(sens_hp - hptemp1));
 return y;
}

Xint32 hp_filter4(Xint32 x)
{
 Xint32 y,hptemp1,hptemp2;
 hptemp1 = bals_hp;
 mulh(hptemp2,hptemp1,2064765799);
 bals_hp = 2 * hptemp2 + x;
 mulh(y,2106124616,(bals_hp - hptemp1));
 return y;
}
*/

int process_time = 0;
void ad_int_handler()
{  
	TIMER0_CURR_CNT = 0xffff;
	TIMER_CTRL = 2<<2|1<<6;


// send out DA value
 DA_TX_DATA = (daout<<16) | (senout); 

 ad_data0 = AD_RX_DATA;
 ad_data1 = AD_RX_DATA;
 ad_data2 = AD_RX_DATA;
 ad_data3 = AD_RX_DATA;

//Read drvout
 drvout_fix = ad_data3;

//Read bals
 bals_fix = ad_data2;

//Read drvs
 drvs_fix = ad_data1;

//Read sens
 sens_fix = ad_data0;

 SINCOS_DIN  = phi_adj;
 //
 drvout_fix <<= 9;
 drvs_fix <<= 9;
 sens_fix <<= 7;
 bals_fix <<= 7;
  
 ref_drvi_fix = SINCOS_DSINE;
 ref_drvq_fix = SINCOS_DCOSE;		 

 mulh(temp1,960767921,drv_freq); 		// 960767921 1227647898
 deltaphi = 3 * temp1;	// 11930464 * 50
 SINCOS_DIN = phi_adj - deltaphi + 41756610;		 				// fix phase shift for primary: 
 ref_drvi_fix1 = SINCOS_DSINE;		 
 ref_drvq_fix1 = SINCOS_DCOSE;

 deltaphi = 4 * temp1;// - (drv_freq-315110000) * 12;
 SINCOS_DIN = phi_adj - deltaphi - 103412862;		 // fix phase shift for secondary: 9deg 107374182 131235111
 ref_drvi_fix2 = SINCOS_DSINE;		 
 ref_drvq_fix2 = SINCOS_DCOSE;

 deltaphi = 2 * temp1;// - (drv_freq-315110000) * 12;
 SINCOS_DIN = phi_adj - deltaphi + 41756610;		 // fix phase shift for bal: + 350233991  + 55131173 - 47771643
 ref_drvi_fix3 = SINCOS_DSINE;		 
 ref_drvq_fix3 = SINCOS_DCOSE;

//HP filter
 drvout_fix = hp_filter1(drvout_fix);
 
//Demodulate drvout
 mulh(temp1,Wdrvout1,ref_drvi_fix);
 mulh(temp2,Wdrvout2,ref_drvq_fix);
 y_drvout = 2 * (temp1 + temp2);
 err_drvout = drvout_fix - y_drvout;
 mulh(temp1,miu_drvout_fix,err_drvout);
 mulh(temp2,temp1,ref_drvi_fix);
 Wdrvout1 = 2 * temp2 + Wdrvout1;
 mulh(temp2,temp1,ref_drvq_fix);
 Wdrvout2 = 2 * temp2 + Wdrvout2;
// drvouti = lp_filter5(Wdrvout1);
// drvoutq = lp_filter6(Wdrvout2);
 drvouti = Wdrvout1;
 drvoutq = Wdrvout2;
 if(bSelfTestOK)
 {
	//Demodulate reference comsin
	 mulh(temp1,Wdrvobv1,com_refsin1);
	 mulh(temp2,Wdrvobv2,com_refcos1);
	 y_drvobv = 2 * (temp1 + temp2);
	 err_drvobv = (drvout_fix>>6) - y_drvobv;
	 mulh(temp1,miu_drvobv_fix,err_drvobv);
	 mulh(temp2,temp1,com_refsin1);
	 Wdrvobv1 = 2 * temp2 + Wdrvobv1;
	 mulh(temp2,temp1,com_refcos1);
	 Wdrvobv2 = 2 * temp2 + Wdrvobv2;
	 drvobvi = lp_filter2(Wdrvobv1);
	 drvobvq = Wdrvobv2;
	/////////// self compensation drvout part //////////////
	
	//DLL DA
	// mulh(temp1,drvoutq,1056689149);
	 temp2 = drvoutq << 4;
	 mulh(temp1,temp2,1367130574);
	 delayph_drv2 = -temp1;
	 mulh(temp1,kp_drvout_fix,delayph_drv2);
	 mulh(temp2,ki_drvout_fix,delayph_drv1);
	 phi_drvout = phi_drvout0 + temp1 - temp2;
	 delayph_drv1 = delayph_drv2;
	 phi_drvout0 = phi_drvout;
	
	
	//HP filter
	 drvs_fix = hp_filter2(drvs_fix);
	 
	//Demodulate drvs
	
	 mulh(temp1,Wdrvs1,ref_drvi_fix1);
	 mulh(temp2,Wdrvs2,ref_drvq_fix1);
	 y_drvs = 2 * (temp1 + temp2);
	 err_drvs = drvs_fix - y_drvs;
	 mulh(temp1,miu_drvs_fix,err_drvs);
	 mulh(temp2,temp1,ref_drvi_fix1);
	 Wdrvs1 = 2 * temp2 + Wdrvs1;
	 mulh(temp2,temp1,ref_drvq_fix1);
	 Wdrvs2 = 2 * temp2 + Wdrvs2;
	// drvsi = lp_filter5(Wdrvs1);
	// drvsq = lp_filter6(Wdrvs2);
	 drvsi = Wdrvs1;
	 drvsq = Wdrvs2;
	
	//Demodulate reference comsin
	 mulh(temp1,Wdrvsobv1,com_refsin2);
	 mulh(temp2,Wdrvsobv2,com_refcos2);
	 y_drvsobv = 2 * (temp1 + temp2);
	 err_drvsobv = (drvs_fix) - y_drvsobv;
	 mulh(temp1,miu_drvsobv_fix,err_drvsobv);
	 mulh(temp2,temp1,com_refsin2);
	 Wdrvsobv1 = 2 * temp2 + Wdrvsobv1;
	 mulh(temp2,temp1,com_refcos2);
	 Wdrvsobv2 = 2 * temp2 + Wdrvsobv2;
	 drvsobvi = lp_filter1(Wdrvsobv1);
	 drvsobvq = Wdrvsobv2;
	/////////// self compensation drvs part //////////////
	
	//DLL GYRO
	 mulh(temp1,drvsi,1527887453);
	 delayf_drv2 = - 16 * temp1;
	 mulh(temp1,kp_drvs_fix,delayf_drv2);
	 mulh(temp2,ki_drvs_fix,delayf_drv1);
	 f_adj = f_adj0 + temp1 - temp2;
	 delayf_drv1 = delayf_drv2;
	 f_adj0 = f_adj;
	 drv_freq = 300000000 - f_adj;
	// drv_freq = 545259520 - f_adj;
		// drv_freq = 323500000;
	
	
	 mulh(temp1,drv_freq,1921535841);
	// mulh(temp1,284100000,1921535841);
	 phi_adj = phi_adj + 2 * temp1;
	
	//LOCK AMP OF GYRO
	 // compensation of drvs
	 mulh(drvsq, drvsq, drvs_gain_adj);
	 drvsq *= 3;
	 // end of compensation
	 delayamp_drv2 = drvamp_goal_fix - abs(drvsq);
	 mulh(temp1,kpamp_drvs_fix,delayamp_drv2);
	 mulh(temp2,kiamp_drvs_fix,delayamp_drv1);
	 drv_gain_fix = drv_gain0_fix + temp1 - temp2;
	 delayamp_drv1 = delayamp_drv2;
	 drv_gain0_fix = drv_gain_fix ; 
	
	//HP filter
	 sens_fix = hp_filter3(sens_fix);
	 
	//Demodulate sens
	 mulh(temp1,Wsens1,ref_drvi_fix2);
	 mulh(temp2,Wsens2,ref_drvq_fix2);
	 y_sens = 2 * (temp1 + temp2);
	 err_sens = sens_fix - y_sens;
	 mulh(temp1,miu_sens_fix,err_sens);
	 mulh(temp2,temp1,ref_drvi_fix2);
	 Wsens1 = 2 * temp2 + Wsens1;
	 mulh(temp2,temp1,ref_drvq_fix2);
	 Wsens2 = 2 * temp2 + Wsens2;
	// sensi += lp_filter5(Wsens1);
	// sensq += lp_filter6(Wsens2); 
	// sensi += Wsens1;
	// sensq += Wsens2; 
	
	// HP filter
	 bals_fix = hp_filter4(bals_fix);
	 
	//Demodulate bals
	 mulh(temp1,Wbals1,ref_drvi_fix3);
	 mulh(temp2,Wbals2,ref_drvq_fix3);
	 y_bals = 2 * (temp1 + temp2);
	 err_bals = bals_fix - y_bals;
	 mulh(temp1,miu_bals_fix,err_bals);
	 mulh(temp2,temp1,ref_drvi_fix3);
	 Wbals1 = 2 * temp2 + Wbals1;
	 mulh(temp2,temp1,ref_drvq_fix3);
	 Wbals2 = 2 * temp2 + Wbals2;
	
	 sensi0 = Wbals1;
	 sensq0 = Wbals2;
	
	// sensi0 = Wsens1;
	// sensq0 = Wsens2; 
	
	// force reblance loop
	// inphase close loop for force reblance control
	 delayseni3 = -lp_filter7(Wsens1);
	/* mulh(temp1,kpseni,delayseni3);
	 mulh(temp2,kiseni,delayseni2);
	 mulh(temp3,kdseni,delayseni1);
	 sen_gaini = sen_gaini0 + temp1 - temp2 + temp3;
	 delayseni1 = delayseni2;
	 delayseni2 = delayseni3; */
	 temp1 = delayseni3 * 3;
	 temp1 += (delayseni3>>2);
	 temp1 >>= 1; // for adjust the gain to 1/2, due to adjust the resistor 10k to 5k(increase drive gain 2 times)
	 sen_gaini = temp1 + sen_gaini0;
	 sen_gaini0 = sen_gaini;
	// quad close loop for force reblance control
	 delaysenq3 = -lp_filter8(Wsens2);
	/* mulh(temp1,kpsenq,delaysenq3);
	 mulh(temp2,kisenq,delaysenq2);
	 mulh(temp3,kdsenq,delaysenq1);
	 sen_gainq = sen_gainq0 + temp1 - temp2 + temp3;
	 delaysenq1 = delaysenq2;
	 delaysenq2 = delaysenq3; */
	 temp1 = delaysenq3 * 3;
	 temp1 += (delaysenq3>>2);
	 temp1 >>= 1; // for adjust the gain to 1/2, due to adjust the resistor 10k to 5k(increase drive gain 2 times)
	 sen_gainq = temp1 + sen_gainq0;
	 sen_gainq0 = sen_gainq;
	 
	 //
	 SINCOS_DIN = phi_adj+phi_drvout0;
	 //
	 
	 mulh(temp1,sen_gaini,ref_drvq_fix2);
	 mulh(temp2,sen_gainq,ref_drvi_fix2);
	 temp1 = temp1 - temp2;
	 mulh(temp2,32767,temp1);
	 senout = temp2 + 32768;
	 if(senout<0) senout = 0;
	 if(senout>65535) senout = 65535;
	
	 temp1 = SINCOS_DSINE;		 
	 temp1 = SINCOS_DCOSE;
	 mulh(temp2,drv_gain0_fix,temp1);	// adjust the gain loop
	// temp2 = 0;
	// mulh(temp2,80530635,temp1);
	 // mulh(temp2,80530635,temp1);
	 // compensation part
	 /* mulh(temp3, drv_freq, 1431655765);
	  mulh(temp3, temp3, 1921535841);
	  temp3 <<= 1;
	  com_phi += temp3; */
	 com_phi += 92162839; // (= f*2^32 / 48e3)   1169Hz: 104600349  1069Hz: 92162839 2110Hz: 188889082
	 SINCOS_DIN = com_phi;
	 com_drvsin = SINCOS_DSINE;
	 com_drvcos = SINCOS_DCOSE;
	 // generate com_ref1
	 SINCOS_DIN = com_phi + 41500152;
	 //
	 mulh(temp1, com_drvsin, 28760910);
	 temp2 += temp1;
	
	 mulh(darslt,305835*4,temp2);
	 
	 // senout = darslt;
	
	// mulh(darslt,45875,temp1);
	 daout = darslt + 32768;
	
	 com_refsin1 = SINCOS_DSINE;		 
	 com_refcos1 = SINCOS_DCOSE;
	 // generate com_ref2
	 SINCOS_DIN = com_phi - 12940444;
	 com_refsin2 = SINCOS_DSINE;		 
	 com_refcos2 = SINCOS_DCOSE;
	
	 // global time ticket increment 1
	 // globalticket++ ;
	
	/*********** for self_comp test ***********/
	/*
	drvobvi_sum = drvobvi;
	drvsobvi_sum = drvsobvi;
	{
		// mulh(drvamp_goal_fix, drvsobvi_sum, -150000000);
		drvamp_goal_fix = -drvsobvi_sum * 268;
		if(drvamp_goal_fix < 26671745) drvamp_goal_fix = 26671745;
		if(drvamp_goal_fix > 42674792) drvamp_goal_fix = 42674792;
		drvs_gain_adj = -drvobvi_sum * 7680;
		if(drvs_gain_adj < 1250000000) drvs_gain_adj = 1250000000;
		if(drvs_gain_adj > 1800000000) drvs_gain_adj = 1700000000;
		drvobvi_sum = 0;
		drvsobvi_sum = 0;
	}
	*/
	
	j++;
	drvobvi_sum += drvobvi;
	drvsobvi_sum += drvsobvi;
	if(j > 7679)
	{
		mulh(drvamp_goal_fix, drvsobvi_sum, -200000000);
		// if(drvamp_goal_fix < 26671745) drvamp_goal_fix = 26671745;
		// if(drvamp_goal_fix > 42674792) drvamp_goal_fix = 42674792;
		drvs_gain_adj = -drvobvi_sum;
		// if(drvs_gain_adj < 1250000000) drvs_gain_adj = 1250000000;
		// if(drvs_gain_adj > 1800000000) drvs_gain_adj = 1700000000;
		drvobvi_sum = 0;
		drvsobvi_sum = 0;
		j = 0;
	}
	
	/******************************************/
	
	/************** Data Output(120 taps avaerage) **************/
	sp_data++;
	if(sp_data > 119)	sp_data = 0;
	
	data_freq_old = data_freq[sp_data];
	temp1 = lp_filter11(drv_freq);
	temp1 >>= 6;
	sum_data_freq += (temp1 - data_freq_old);
	data_freq[sp_data] = temp1;
	
	data_amp_old = data_amp[sp_data];
	temp1 = (drv_gain_fix>>6);
	sum_data_amp += (temp1 - data_amp_old);
	data_amp[sp_data] = temp1;
	
	data_inph_old = data_inph[sp_data];
	temp1 = lp_filter9(sensi0);
	sum_data_inph += (temp1 - data_inph_old);
	data_inph[sp_data] = temp1;
	
	data_quad_old = data_quad[sp_data];
	temp1 = (sensq0);
	sum_data_quad += (temp1 - data_quad_old);
	data_quad[sp_data] = temp1;
	/* for automatic sending out data 
	i++;
	if(i==120)
	{
		mulh(adrslt1, sum_data_freq, 229064920);			// final gain of freq is 0.01
		mulh(adrslt2,  sum_data_amp, 42666667);				// final gain of amplitude is 0.0186264514923095703125
		mulh(adrslt3, sum_data_inph, 21333333);				// final gain of inph output is 0.59604643844068050384521484375
		mulh(adrslt4, sum_data_quad, 21333333);				// final gain of inph output is 0.59604643844068050384521484375
		bstart_tx = 1;
		i = 0;
	}
	*/
 }	
 else
 {
	phi_adj += 92162839; // generate 1069Hz reference signal for self-test, determine the +-12V is OK
	daout = 32767 + (ref_drvq_fix >> 17);
	if(abs(drvoutq) > 42660000) bSelfTestOK = 1;
 }
 
process_time = 65536-TIMER0_CURR_CNT;

/* for UART trigger send out data */
 if(UART_STATUS & URX_READY)
 {
	rxdata = UART_R_DATA;
	if(rxdata == 0x67)
	{
		mulh(adrslt1, sum_data_freq, 229064920);			// final gain of freq is 0.01
		// adrslt1 = process_time;
		mulh(adrslt2,  sum_data_amp, 42666667);				// final gain of amplitude is 0.0186264514923095703125
		mulh(adrslt3, sum_data_inph, 21333333);				// final gain of inph output is 0.59604643844068050384521484375
		mulh(adrslt4, sum_data_quad, 21333333);				// final gain of inph output is 0.59604643844068050384521484375
		bstart_tx = 1;
	}
 }
 
}

void adda_init(void)
{

	AD_TIMING0 = 124|16<<16;
	AD_TIMING1 = 120|50<<16;
	DA_TIMING0 = 387|413<<16;
	DA_TIMING1 = 433|1<<12;
	INT_TIMING = 499;

	// ADDA_CTRL = 0<<2|3;	/* For 48MHz CPU */

	ADDA_CTRL = 1<<2|3;	/* For 96MHz CPU */

	INTC_EN |= (1<<28);
	INTC_MASK &= ~(1<<28);
}

void Digit_Volt_SPI_init(void)
{
	u32 val;
	val = GPIO_A_DIR;
	val |= 0x000E;//PORT A 3-2-1 output
	GPIO_A_DIR = val;
	
	val = GPIO_A_MODE_L;
	val &= 0xffffff03;//PORT A 3-2-1 mode = gpio
	GPIO_A_MODE_L = val;
	
	val = GPIO_A_RDATA & 0xFFFF1;
	GPIO_A_WDATA = val; //PORTA output = 0
////////////////////////////////////////////////////////////////////////////////	
	val = GPIO_B_DIR;
	val |= 0x000C;//PORT B 3-2 output;   -0 input
	GPIO_B_DIR = val;
	
	val = GPIO_B_MODE;
	val &= 0xffffff0C;//PORT B 3-2-0 mode = gpio
	GPIO_B_MODE = val;
	
	val = GPIO_B_RDATA | 0x000C;
	GPIO_B_WDATA = val; //PORTB output = 0
}

//write 3 bytes at one time including 1 commond byte and 2 data bytes
void SPI_Write(u32 x)
{
	u8 i;
	GPIO_A_WDATA = 0;//CS set 0
	delay_1ms(1);
	for(i=0;i<24;i++)
	{
		if(x & 0x800000)
			GPIO_A_WDATA = 0x08;//SDI = 1
		else
			GPIO_A_WDATA = 0;//SDI = 0
		delay_1ms(1);
		GPIO_A_WDATA |= 0x04; //CLK = 1 positive edge
		delay_1ms(1);
		x <<= 1;
		GPIO_A_WDATA = 0; //CLK = 0,SDI = 0,CS = 0, negative edge
	}
	GPIO_A_WDATA = 0x02;//CS = 1
	delay_1ms(1);
	GPIO_A_WDATA = 0;//CS = 0
	delay_1ms(1);
	GPIO_A_WDATA = 0x02;//CS = 1
	delay_1ms(1);
}

void Digit_Volt_nWP_nPR(void)
{
	u32 val;
	val = GPIO_B_RDATA & 0xFFF3;
	GPIO_B_WDATA = val; //PORTB output = 0
}

void entry(void)
{

	u8 *p8;
	int kkk;

	load_memorry();

	// pmc_set_clk(1<<11|10<<2);	/* fclk=(10+2) / (1+2) * 12e6 = 48e6*/
	pmc_set_clk(1<<11|22<<2);	/* fclk=(22+2) / (1+2) * 12e6 = 96e6*/

	delay_1ms(48);
	
	init_uart(96e6,230400);

#if 0
	p8 = (u8*)0x8000;

	i = xmodem(p8);
	printk("xmodem recieve %08x bytes\r\n",i*128);
	write_spi_boot(p8,i*128);
	printk("write spi boot ok\r\n");

	sincos_test();
#endif
	// initial array
	for(kkk = 0; kkk < 120; kkk++)
	{
		data_freq[i] = 0;
		data_amp[i] = 0;
		data_inph[i] = 0;
		data_quad[i] = 0;
	}
	
	Digit_Volt_SPI_init();
	SPI_Write(0xB00200);//   0V
	SPI_Write(0x200000);
	Digit_Volt_nWP_nPR();//  write protection

	gpio_set_adir(0,1);//0
	gpio_set_aout(0,0);
	delay_1ms(48);
	gpio_set_aout(1,1);
	pin_set_adda();
	adda_init();
	arm_enable_int();

	while(1)
	{  
		/*********** transmit loop *************/
		switch(btxstat)	// according transmit status dealing transmition data
		{
		case 0:
			if(bstart_tx)
			{
					btxstat = 22;
					bstart_tx = 0;
			}
			break;
		case 22:
			if((UART_STATUS & UTX_BUSY)==0)
			{
				UART_W_DATA = 0x0eb;	// Send Frame head 0xeb
				btxstat = 1;
			}
			break;
		case 1:
			if((UART_STATUS & UTX_BUSY)==0)
			{
				UART_W_DATA = 0x90;	// Send Frame head 0x90
				btxstat = 2;
			}
			break;
		case 2:
			if((UART_STATUS & UTX_BUSY)==0)
			{
				UART_W_DATA = 0x10;	// Send data length 0x10
				btxstat = 3;
			}
			break;
		case 3:
			if((UART_STATUS & UTX_BUSY)==0)
			{
				UART_W_DATA = 0x0f1;	// Send data type ID 0xf1
				btxstat = 4;
				chksum = 0x01;
			}
			break;					
		case 4:	// send gyro data freq 1
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt1 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt1 >>= 8;
				btxstat = 5;
			}
			break;
		case 5:	// send gyro data freq 2
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt1 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt1 >>= 8;
				btxstat = 6;
			}
			break;
		case 6:	// send gyro data freq 3
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt1 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt1 >>= 8;
				btxstat = 7;
			}
			break;
		case 7:	// send gyro data freq 4
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt1 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				btxstat = 8;
			}
			break;
		case 8:	// send gyro data amp 1
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt2 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt2 >>= 8;
				btxstat = 9;
			}
			break;
		case 9:	// send gyro data amp 2
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt2 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt2 >>= 8;
				btxstat = 10;
			}
			break;
		case 10:	// send gyro data amp 3
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt2 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt2 >>= 8;
				btxstat = 11;
			}
			break;
		case 11:	// send gyro data amp 4
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt2 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				btxstat = 12;
			}
			break;
		case 12:	// send gyro data inph 1
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt3 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt3 >>= 8;
				btxstat = 13;
			}
			break;
		case 13:	// send gyro data inph 2
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt3 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt3 >>= 8;
				btxstat = 14;
			}
			break;
		case 14:	// send gyro data inph 3
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt3 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt3 >>= 8;
				btxstat = 15;
			}
			break;
		case 15:	// send gyro data inph 4
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt3 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				btxstat = 16;
			}
			break;
		case 16:	// send gyro data quad 1
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt4 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt4 >>= 8;
				btxstat = 17;
			}
			break;
		case 17:	// send gyro data quad 2
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt4 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt4 >>= 8;
				btxstat = 18;
			}
			break;
		case 18:	// send gyro data quad 3
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt4 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				adrslt4 >>= 8;
				btxstat = 19;
			}
			break;
		case 19:	// send gyro data quad 4
			if((UART_STATUS & UTX_BUSY)==0)
			{
				tx_data = (adrslt4 & 0x000000ff);
				chksum += tx_data;
				UART_W_DATA = tx_data;
				btxstat = 20;
			}
			break;
		case 20:	// send frame count
			if((UART_STATUS & UTX_BUSY)==0)
			{
				UART_W_DATA = 0x00;
				btxstat = 21;
			}
			break;
		case 21:	// send frame count
			if((UART_STATUS & UTX_BUSY)==0)
			{
				UART_W_DATA = (chksum & 0x000000ff);
				btxstat = 0;
			}
			break;
		default:
			btxstat = 0;
			break;
		}
	}
}



