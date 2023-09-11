#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include "fonts.h"
#include <stdio.h>

#define TOGGLE 2
#define ON 1
#define OFF 0

#define CAPTURE_M 0
#define COUNT_M 1

volatile enum adc_states {
	INIT, CH10, CH11, CH14, CH15, STOPPED, RUNNING,
} adc_state;

volatile uint16_t input_old = 0;
volatile uint32_t ms = 0;
volatile uint16_t user_btn_pressed = 0;
volatile uint16_t LED_activated_by_key = 0;
volatile uint16_t LED_timer = 0;
volatile uint16_t backlight_flash_timer = 0;
volatile uint16_t adc_channel_idx =  0;
// LIN global vars/const
enum usart_state_e {IDLE, BRK_SYNC, SYNC_RCV, CMD_RCV, SENDING};
volatile enum usart_state_e usart_state;
volatile uint8_t linDataBuffer[5];
volatile uint8_t linDataBuffer_idx = 0;
volatile uint8_t linDataBuffer_idx_max = 0;
// Frequency global vars
volatile uint16_t tim12_mode, tim12_mode_cnt = 0;
volatile uint32_t last_cnt = 0, delta_cnt;
volatile uint16_t tim12_overflows = 0;
volatile uint32_t f;
// ADC global vars
volatile uint16_t adc_res_percent[4];
volatile uint16_t adc_channel[] = {10, 11, 14, 15}; 

//Forward declaration
void LCD_ClearDisplay (uint16_t color);
void LCD_toggle_backlight(void);
void tim12_config(int mode);

// General functions -----------------------------------------------------------------------------------
void short2bitstring(uint16_t res, char* string_buf){
	for(uint8_t i = 0;i<16;++i){
		string_buf[i] = ((res & 0x8000) == 0) ? '0' : '1';
		res <<= 1;
	}
	string_buf[16] = '\0';
}

void intToString(uint32_t zahl, char* string){
	for(int i = 9; i>=0; --i){
		string[i] = (zahl % 10) + '0';
		zahl /= 10;
	}
	string[10] = '\0';
}
void u_delay(uint32_t usec){
	uint32_t limit = usec * (65/5);
	uint32_t volatile cnt = 0;
	while(limit > cnt){
		cnt++;
	}
}

void toggleGreenLed(int onoff){	
	if (onoff==ON){
		GPIOD->ODR |= (1 << 12); // PD12 = 1
		LED_timer = 1;
	}
	else if (onoff==OFF){
		GPIOD->ODR &= ~(1 << 12); // PD12 = 0
	}
	else if (onoff==TOGGLE){
		if((GPIOD->ODR & (1 << 12)) ==0  ){
			toggleGreenLed(ON);
		}
		else{
			toggleGreenLed(OFF);
		}
	}
}

void calcFrequency(void){
	if (tim12_mode == CAPTURE_M){
		uint32_t cnt = TIM12->CCR1 + (tim12_overflows * 0xFFFF);
		delta_cnt = cnt - last_cnt;
		f = 84000000/delta_cnt;
		last_cnt = cnt - (tim12_overflows * 0xFFFF);
	}
	else{
		uint32_t start_time = ms;
		while(start_time == ms);
		TIM12->CNT = 0;
		while(ms - start_time <= 10);
		f = 1000 * TIM12->CNT/ 10;
	}
	tim12_mode_cnt++;
	tim12_overflows = 0;
}
// Functions with button
void user_btn_LED_action(void){
	if(user_btn_pressed == 1 && LED_timer == 0){
		toggleGreenLed(ON); // Sets LED_timer implicitly to 1
	}
	if(LED_timer == 10001){
		LED_timer = 0;
		toggleGreenLed(OFF);
	}
	if (LED_timer >= 1){
		LED_timer++;
	}
}


void user_btn_LCD_action(void){
	if(user_btn_pressed == 1){
		backlight_flash_timer %= 1000;
		if (backlight_flash_timer == 0){
			LCD_toggle_backlight();
		}
		backlight_flash_timer++;
	}
	else if(backlight_flash_timer != 1000){
		GPIOD->ODR |= (1 << 13);		
		backlight_flash_timer = 1000;
	}
}
// Output (LCD/LED)
void LCD_Output16BitWord(uint16_t data){
    //Bits 0 und 1
    uint16_t shift = (data & 0x0003) << 14;  
    //Bits 2 und 3
    shift += (data & 0x000C) >> 2;
    //Bits 13, 14 und 15
    shift += (data & 0xE000) >> 5;
		GPIOD->ODR |= shift;	
		GPIOD->ODR &= shift +(~0xC703); 
    //Restliche Bits
    shift = (data & 0x1FF0) << 3;
    GPIOE->ODR &= shift & (~0xFF80);
		GPIOE->ODR |= shift;
    return;
}

// LCD --------------------------------------------------------------------
void LCD_WriteData (uint16_t data){
	GPIOE->ODR |= (1 << 3);
	GPIOD->ODR &= ~(1 << 7);
	GPIOD->ODR &= ~(1 << 5);
	LCD_Output16BitWord(data);
	GPIOD->ODR |= (1 << 5);
	GPIOD->ODR |= (1 << 7);
}

void LCD_WriteCommand (uint16_t cmd){
	GPIOE->ODR &= ~(1 << 3);
	GPIOD->ODR &= ~(1 << 7);
	GPIOD->ODR &= ~(1 << 5);
	LCD_Output16BitWord(cmd);
	GPIOD->ODR |= (1 << 5);
	GPIOD->ODR |= (1 << 7);
}

void LCD_WriteReg(uint16_t cmd, uint16_t data){
	LCD_WriteCommand(cmd);
	LCD_WriteData(data);
}

// Inits
void LEDs_InitPorts(void){ // Also used for Display
	RCC->AHB1ENR |= 0x00000008; // Enable Clock for Port D
	RCC->AHB1ENR |= 0x00000010; // Enable Clock for Port E

	GPIOD->MODER &= 0x57557575; 
	GPIOD->MODER |= 0x54554545; // Config Pins at Port D as Outputs (ex. 2, 6, 12)
	GPIOE->MODER &= 0x55557F7F;
	GPIOE->MODER |= 0x55554040; // Config Pins at Port E as Outputs (ex. 0-2, 4-6)
}

void LCD_Init (void){
	LEDs_InitPorts();
	GPIOD->ODR |= (1 << 3);
	GPIOD->ODR &= ~(1 << 3);
	u_delay(15);
	GPIOD->ODR |= (1 << 3);
	GPIOD->ODR |= (1 << 4);				// Set low-active signals to 1 before reset
	GPIOD->ODR |= (1 << 13);			// Activate Backlight
	LCD_WriteReg(0x0010, 0x0001); /* Enter sleep mode */
	LCD_WriteReg(0x001E, 0x00B2); /* Set initial power parameters. */
	LCD_WriteReg(0x0028, 0x0006); /* Set initial power parameters. */
	LCD_WriteReg(0x0000, 0x0001); /* Start the oscillator.*/
	LCD_WriteReg(0x0001, 0x72EF); /* Set pixel format and basic display orientation */
	LCD_WriteReg(0x0002, 0x0600);
	LCD_WriteReg(0x0010, 0x0000); /* Exit sleep mode.*/
	u_delay(30000);								// weniger geht meist auch
	LCD_WriteReg(0x0011, 0x6870); /* Configure pixel color format and MCU interface parameters.*/
	LCD_WriteReg(0x0012, 0x0999); /* Set analog parameters */
	LCD_WriteReg(0x0026, 0x3800);
	LCD_WriteReg(0x0007, 0x0033); /* Enable the display */
	LCD_WriteReg(0x000C, 0x0005); /* Set VCIX2 voltage to 6.1V.*/
	LCD_WriteReg(0x000D, 0x000A); /* Configure Vlcd63 and VCOMl */
	LCD_WriteReg(0x000E, 0x2E00);
	LCD_WriteReg(0x0044, (240-1) << 8); /* Set the display size and ensure that the GRAM window is set to allow access to the full display buffer.*/
	LCD_WriteReg(0x0045, 0x0000);
	LCD_WriteReg(0x0046, 320-1);
	LCD_WriteReg(0x004E, 0x0000); /*Set cursor to 0,0 */
	LCD_WriteReg(0x004F, 0x0000);
	LCD_ClearDisplay(0xF800);
	
}

// LCD functions
void LCD_toggle_backlight(void){
	GPIOD->ODR ^= (1 << 13);			// toggle Backlight
}
void LCD_SetCursor (uint32_t x, uint32_t y){
	LCD_WriteReg(0x004E, x); // Set cursor x
	LCD_WriteReg(0x004F, y); // Set cursot to y
}

void LCD_DrawPixel (uint16_t color){
	LCD_WriteReg(0x0022, color); // Cursor automatically incremented
}

void LCD_ClearDisplay (uint16_t color){
	LCD_SetCursor(0,0);
	for(int i = 0; i < 320*240; i++){
		LCD_DrawPixel(color); // Is Auto-Linebreak enabled?
	}
}

void LCD_WriteLetter(uint8_t c, uint32_t x, uint32_t y, uint16_t color, uint16_t bg_color){
	LCD_SetCursor(x, y);
	uint16_t* cur_row = (uint16_t*)(console_font_12x16 + c * 32);
	for(uint16_t line_nr = 1;line_nr<=16;line_nr++){
		for(uint8_t ct = 0;ct<2;ct++){
			uint16_t bitline = *cur_row;
			short max = 8;
			uint16_t mask = 0x0080;
			if (ct==1){
				max = 4;
				mask = 0x8000;
			}
			for(uint16_t col_nr = 0;col_nr<max;col_nr++){
				if(bitline & mask){
					LCD_DrawPixel(color);
				}
				else{
					LCD_DrawPixel(bg_color); // Slow! Unneccessary register write
				}
				bitline <<= 1;
			}
		}
		LCD_SetCursor(x, y+line_nr);
		cur_row++;
	}
}

void LCD_WriteString(char* string, uint32_t string_length, uint32_t x, uint32_t y, uint16_t color, uint16_t bg_color){
	for(int i = 0; i<string_length && i<26; i++){ // 320/12=26.6667
		LCD_WriteLetter(string[i], x+i*12, y, color, bg_color);
	}
}
// Timer
void tim7_init(void){
	RCC->APB1ENR |= (1 << 5);
	TIM7->DIER |= 1;	// Enable Interrupts
	TIM7->PSC = 83;
	TIM7->ARR = 999;
	TIM7->CR1 |= 1;		// Enable counter
	NVIC_SetPriority(TIM7_IRQn, 5);
	NVIC_EnableIRQ(TIM7_IRQn);
}

void TIM7_IRQHandler(void){
	TIM7->SR = 0; // Clear Status register
	ms++;
	user_btn_LED_action();
	user_btn_LCD_action();
	/*if(LED_activated_by_key){
		if(LED_activated_by_key == 10000){
			LED_activated_by_key = 0;
			toggleGreenLed(OFF);
		}
		else{
			LED_activated_by_key++;
		}
	}*/
}
void tim12_init(void){
	GPIOB->MODER &= ~(1<<28);
	GPIOB->MODER |= (1<<29);
	GPIOB->AFR[1]|= (9<<24);
	
	RCC->APB1ENR |= (1<<6);
	TIM12->ARR = 0xFFFF;
	TIM12->CCMR1 |= 1;	
	NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 7);
	tim12_config(COUNT_M);
	TIM12->CR1 |= 1;
}
void tim12_config(int mode){
	tim12_mode = mode;
	if (tim12_mode == CAPTURE_M){
		TIM12->CR1 &= ~1;
		TIM12->SMCR &= 0xFF00;
		TIM12->DIER |= (1<<1);
		TIM12->CCER |= 1;
		NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
		TIM12->CR1 |= 1;
	}
	else 	if (tim12_mode == COUNT_M){
		TIM12->CR1 &= ~1;
		TIM12->SMCR |= ((5<<4)|7);
		TIM12->DIER &= ~(1<<1);
		TIM12->CCER &= ~1;
		NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
		TIM12->CR1 |= 1;
	}
}
void TIM8_BRK_TIM12_IRQHandler(void){
	if (TIM12->SR & 1){
		++tim12_overflows;
		TIM12->SR &= ~(1);
	}
	TIM12->SR &= ~(1);
	TIM12->SR &= ~(1<<9); // Clear CC1OF
	if (tim12_mode != COUNT_M){
		calcFrequency();
	}

}
// Potentiometer
void pot_init(void){
	RCC->AHB1ENR |= (1<<2); //Takt für GPIO Port C einschalten
	GPIOC->MODER |= (0xF | (0xF << 8)); // PC0:IN10, PC1:IN11, PC4:IN14 und PC5:IN15 auf 11 (Analog) setzen
	RCC->APB2ENR |= (1<<8); //Takt einschalten für ADC1
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_EOCS; // ADON Bit setzen (Wandler einschalten) & EOCS Bit (Jede Wandlung löst interrupt aus)
	ADC1->SQR3 = 10; // Channel number to be converted
	//ADC1->SQR1 &= ~(0xF<<20); // Setze Anzahl zu scannender Kanäle auf 1
	//RES-Bits 25, 24: (00-> 12-bit->15 ADCCLK cycles)(01->10-bit->13 ADCCLK cycles)(10->8-bit->11 ADCCLK cycles)(11:->6-bit->9 ADCCLK cycles)
	//ADC1->CR1 |= (1<<24); 
	//ADC1->SMPR2 = ?? : Sample Time 
	ADC1->CR1 = (1<<5); //Enable EOC interrupts
	NVIC_SetPriority(ADC_IRQn, 10);
	NVIC_EnableIRQ(ADC_IRQn);
}
void ADC_IRQHandler(void){
	uint32_t tempsr = ADC1->SR;
	uint32_t tempdr = ADC1->DR; 
	ADC1->SR &= ~ADC_SR_EOC; // Clear End of Conversion(EOC) Bit
	adc_res_percent[adc_channel_idx] = (tempdr * 100) / 4095;
	adc_channel_idx++;
	if (adc_channel_idx > 3){
		adc_channel_idx = 0;
		adc_state = STOPPED;
		ADC1->SQR3 = adc_channel[adc_channel_idx]; // Select next channel
	}
	else{
		ADC1->SQR3 = adc_channel[adc_channel_idx]; // Select next channel
		ADC1->CR2 |= ADC_CR2_SWSTART; // Start next conversion
	}
}
// Throttle
void throttle_init(void){
	// Config Ports
	GPIOC->MODER |= (1 << 22); // PC11 (Output)
	GPIOB->MODER |= 0xA;  // PB0 + PB1 (AF) (.....1010)
	GPIOB->AFR[0] |= (2 | 2 << 4); // Select AF->2 for PB0 and PB1
	RCC->APB1ENR |= (1 << 1); // Takt für TIM3 einschalten
	// TIM3->PSC = 39; // Prescaler einstellen
	TIM3->ARR = 9999; // Frequenz & Periode einstellen
	TIM3->CCMR2 |= (3<<5)|(3<<13); // CNT<CCR3 -> 1 !!!auch für CCR4 machen
	TIM3->CCR3 = TIM3->ARR/2; // Pulsweite initial auf 50%
	TIM3->CCR4 = TIM3->ARR/2; // Pulsweite initial auf 50%
	TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable Output to Pin
	TIM3->CR1 |= TIM_CR1_CEN; // Enable counter (start)
	
	GPIOC->ODR|= (1<<11); // Enable Throttle actuation
}
void adc_start(void){
	if(adc_state == STOPPED || adc_state == INIT){
			ADC1->CR2 |= (1<<30); // Start conversion in ADC
			adc_state = RUNNING;
	}
}
void adc_write_data(void){
	for(int i = 0; i<4; i++){
		char res_string[10];	
		intToString(adc_res_percent[i], res_string);
		LCD_WriteString(res_string, 10, 190, 20 * i, 0xFFFF, 0xF800);
	}
}
void setPWM(void){
	uint16_t arr_temp = TIM3->ARR;
	TIM3->CCR3 = adc_res_percent[0] * arr_temp / 100;
	TIM3->CCR4 = adc_res_percent[1] * arr_temp / 100;
}
// LIN
void usartResponseInit(uint16_t dr){	
	uint16_t id_full = dr;
	uint16_t id = (id_full >> 4) & 0x3;
	if(id == 1){
		linDataBuffer_idx_max = 2;
		for(int i=0;i<linDataBuffer_idx_max; i++){
			linDataBuffer[i] = (((ms)>>(i*8)) & 0x000000FF);
		}
	}
	else if(id == 2){
		linDataBuffer_idx_max = 4;
		for(int i=0;i<linDataBuffer_idx_max; i++){
			linDataBuffer[i] = ((f>>(i*8)) & 0x000000FF);
		}
	}
	else if(id == 3){
		linDataBuffer_idx_max = 2;
		for(int i=0;i<linDataBuffer_idx_max; i++){
			linDataBuffer[i] = ((input_old>>(i*8)) & 0x00FF);
		} 
	}
	// Calc checksum
	uint16_t ret_value = 0;
	for(int i=0;i<linDataBuffer_idx_max;i++){
		ret_value += linDataBuffer[i];
		if(ret_value>0xFF){
			ret_value -= 255;
		}
	}
	ret_value += id_full;
	if(ret_value>0xFF){
		ret_value -= 255;
	}
	linDataBuffer[linDataBuffer_idx_max] = (uint8_t) ~ret_value;
}

void lin_init(void){
	// LIN
	GPIOC->MODER |= (1<<12);//Config PC6 as an Output
	GPIOC->ODR |= (1<<6); // Set PC6(TXD) to High Voltage
	GPIOB->MODER |= (1<<4);// Config PB2 as an Output
		//GPIOB->ODR &= ~(1<<2); // Set PB2 to LowVoltage (NSLP)
	GPIOB->ODR |= (1<<2); // Set PB2 to HighVoltage
		//GPIOB->ODR &= ~(1<<2); // Reset PB2
	// Ports config
		//PC6, PC7 auf AF
	GPIOC->MODER |= (1<<13)|(1<<15);
	GPIOC->MODER &= ~(1<<12);
	GPIOC->AFR[0] |= 0x88000000; //(1<<27)|(1<<31); 
	// USART
	RCC-> APB2ENR |= (1<<5); // USART Takt enable
	USART6->BRR  = 0x1117; // Set Baudrate
	USART6->CR1 |= (0x200C); // Reciever, Tranciever enable | USART Enable
	USART6->CR2 |= (1<<14) | (1<<5) | (1<<6); // LIN enable | LBDL | LBDIE
	NVIC_SetPriority(USART6_IRQn, 9);
	NVIC_EnableIRQ(USART6_IRQn);
}
void USART6_IRQHandler(void){
	// USART6->SR = 0;
	uint16_t sr = USART6->SR;
	uint16_t dr = USART6->DR;
	switch(usart_state){
		case IDLE:
			if(sr & USART_SR_LBD_Msk){
				USART6->SR &= ~USART_SR_LBD; // Clear LBD Bit
				usart_state = BRK_SYNC;
				USART6->CR1 &= ~USART_CR1_TCIE; // Disable Transmit
				USART6->CR1 |= USART_CR1_RXNEIE; // Enable RXNEI
			}
			break;
		case BRK_SYNC:
			if((sr & USART_SR_RXNE_Msk) && (dr == 0x55)){
				usart_state = SYNC_RCV;
			}
			else{
				usart_state = IDLE;
				USART6->CR1 &= ~USART_CR1_RXNEIE; // Enable RXNEI				
			}
			break;
		case SYNC_RCV:
			if(sr & USART_SR_RXNE_Msk){
				if((dr & 0xF) == 9){
					usartResponseInit(dr);
					USART6->DR = linDataBuffer[linDataBuffer_idx]; // Start sending
					linDataBuffer_idx++;
					usart_state = SENDING;
					USART6->CR1 |= USART_CR1_TCIE; // Enable TCIE
					USART6->CR1 &= ~USART_CR1_RXNEIE; // Disable RXNEI
				}
				else{
					usart_state = IDLE;
					USART6->CR1 &= ~USART_CR1_RXNEIE; // Disable RXNEI
				}
			}
			break;
		case CMD_RCV:
			// Not in use
			break;
		case SENDING:
			if(USART6->SR & USART_SR_TC_Msk){
				if(linDataBuffer_idx<=linDataBuffer_idx_max){
					USART6->DR = linDataBuffer[linDataBuffer_idx]; // continue sending
					linDataBuffer_idx++;
				}
				else{
					linDataBuffer_idx = 0;
					usart_state = IDLE;
					USART6->CR1 &= ~USART_CR1_TCIE; // Disable TCIE
				}
			}
			break;
		default:
			usart_state = IDLE;
			break;
	}
}
// Flash LED
void portInitGreenLed(void){
	RCC->AHB1ENR |= 0x00000001; // Enable Clock for Port A
	RCC->AHB1ENR |= 0x00000008; // Enable Clock for Port D
	GPIOA->MODER &= 0xFFFFFFFC; // Port A Pin 0 -> Input (00) (USER-Button)
	GPIOD->MODER &= 0xFDFFFFFF;
	GPIOD->MODER |= 0x01000000; // Port D Pin 12 -> Output (01) (Green-LED)
}

// Keyboard-Init
void keyboard_port_init(void){
	RCC->AHB1ENR |= 0x00000003; // Enable Clock for Port A & B

	GPIOA->MODER &= 0xFFFFC03F; // Config Pins PA3-PA6 as Inputs
	GPIOA->PUPDR &= 0xFFFFD57F;
	GPIOA->PUPDR |= 0x00001540; // Pull-up at PA3-6
	
	GPIOB->MODER &= 0xFFFF55FF;
	GPIOB->MODER |= 0x00005500; // Pins PB4-7 as Outputs
	GPIOB->OTYPER |= 0x000000F0; // Open-Drain at PB4-7
}

// Flashlight (Green LED) functions ----------------------------------------------------------------------------------------
void greenLEDFlash(void){
	while(GPIOA->IDR & 0x00000001){
		toggleGreenLed(ON);
		u_delay(500000);
		toggleGreenLed(OFF);
		u_delay(500000);
	}
}


// Keyboard functions ------------------------------------------------------------------------------------------------------
void indicate_keyboard_actions(uint16_t press, uint16_t release){
	if (((press & 0xFF) != 0) || ((release & 0xFF00) != 0)){
		toggleGreenLed(TOGGLE);
	}
}

void deactivate_row(void){
	GPIOB->ODR |= (0xF << 4); //	High-voltage to PB4-7
}

void activate_row(int row){
	// deactivate_row(); // make sure that only one row is activated at a time
	int pin_nr = row + 4;
	GPIOB->ODR &= ~(1 << pin_nr); // Low-voltage to PB pin_nr
}

int check_keys_in_row(void){
	int cur_states = (GPIOA->IDR & 0x0078) >> 3;
	int res = -1;
	for (int i = 0; i<4;i++){
		if((cur_states & 1) == 0){
			res = i;
			break;
		}
		else{
			cur_states >>= 1;
		}
	}
	return res;
}
uint16_t read_input(void){
	uint16_t input = 0;
	for(int i = 0;i<4;i++){
		activate_row(i);
		u_delay(20);
		int res = check_keys_in_row();	
		deactivate_row();
		if (res != -1){
			input = (1 << (res+4*i));
			break;
		}
	}
	return input;
}

void check_input_changes(uint16_t in_new, uint16_t in_old, uint16_t* pressed, uint16_t* released){
	if (in_new != in_old){
		if (in_new == 0){ // only works porperly if release and press do not happen in same cycle
			*released = in_old;
			*pressed = 0;
		}
		else{
			*pressed = in_new;
			*released = 0;
		}
	}
	else{
		*released = 0;
		*pressed = 0;
	}
}

/*###############################
  Main function
	############################### */

int main (void) {
	mcpr_SetSystemCoreClock(); // has to be first line in main!
	GPIOD->MODER &=  ~(1<<29);
	GPIOD->MODER |= (1<<28);
	LCD_Init();
	portInitGreenLed();
	keyboard_port_init();
	tim7_init();
	tim12_init();
	LCD_WriteString("Count", 6, 20, 80, 0xFFFF, 0xF800);
	LCD_WriteString("Capture", 8, 20, 120, 0xFFFF, 0xF800);
	pot_init();
	RCC->AHB1ENR |= (1<<2); //Takt für GPIO Port C einschalten
	lin_init();
	throttle_init();
	uint16_t cur_pressed, cur_released;
	char f_string[11];
	while(1){
		adc_write_data();
		adc_start();
		//Following is while loop for Keyboard
		uint16_t input_new = read_input();
		check_input_changes(input_new, input_old, &cur_pressed, &cur_released); // Here also the gloabal variable LED_activated_by_key is set
		indicate_keyboard_actions(cur_pressed, cur_released);
		input_old = input_new;
		char string_buf[17]; 
		short2bitstring(input_new, string_buf);
		LCD_WriteString(string_buf, sizeof(string_buf), 20, 220, 0xFFFF, 0xF800);
		//Following is while loop for Timer
		if (GPIOA->IDR & 1){
			user_btn_pressed = 1;
		}
		else{
			user_btn_pressed = 0;
		}
		if(tim12_mode == COUNT_M){
			calcFrequency();
		}
		intToString(f, f_string);
		if(tim12_mode == COUNT_M){
			LCD_WriteString(f_string, 11, 20, 100, 0xFFFF, 0xF800);
			if(tim12_mode_cnt > 10){
				tim12_config(CAPTURE_M);
				tim12_mode_cnt = 0;
			}
		}
		else{
			LCD_WriteString(f_string, 11, 20, 140, 0xFFFF, 0xF800);
			if(tim12_mode_cnt > 10){
				tim12_config(COUNT_M);
				tim12_mode_cnt = 0;
			}
		}
		intToString(ms, string_buf);
		LCD_WriteString(string_buf, 11, 1, 1, 0xFFFF, 0xF800);
		// 
		setPWM();
		while(ms%50!=0){}
	}
}
/*
 ...............WorkInProgress.........................  

*/
