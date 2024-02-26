#include<lpc17xx.h>
#include "lcd.h"
#include<string.h>

#define ROW_PINS (0x0F << 4)
#define COL_PINS (0x0F << 0)

#define VREF 3.3
#define ADD_CLK_EN (1<<12)
#define SEL_AD0_1 (1<<1)
#define CLKDIV (3<<8)
#define PWRUP (1<<21)
#define START_CNV (0X01<<24)
#define ADC_DONE (1U<<31) //define unsigned bit for the

void pwm_init(void);
void adc_init(void);
void keypad(void);
void battry(void);
void indicator(int volts);
int k=0;

int main(){
lcd_init();
adc_init();
lcd_cmd_write(0x0C);	 //
lcd_str_write("hand break on");

LPC_GPIO2->FIODIR |= ROW_PINS;
LPC_GPIO2->FIODIR &=~COL_PINS;
LPC_GPIO1->FIODIR |= (0xFF<<19);
LPC_GPIO1->FIODIR |= (1<<27);
if(k==0){
LPC_GPIO1->FIOSET = (1<<22);
}


while(1){
   LPC_GPIO1->FIOSET = (1<<23);
   battry();
   keypad();
}
}


void battry(){
  int result=0;
  float volts=0;
  char svolts[20];
  LPC_ADC->ADCR |=START_CNV;
  while((LPC_ADC->ADDR1 & ADC_DONE)==0){}
   
   result = (LPC_ADC->ADDR1>>4) & 0xFFF;
   volts=(result/4096.0)*VREF;
   volts=(volts/VREF)*100;
   sprintf(svolts,"battery level=%.0f ",volts);
   lcd_cmd_write(0xC0);
   lcd_str_write(svolts);
   delay(500);
   indicator(volts);
}

void indicator(int volts){
if(volts>=16){
lcd_cmd_write(0x80);
lcd_str_write("left indicat on");
LPC_GPIO1->FIOSET = (1<<19);
LPC_GPIO1->FIOCLR = (1<<20);
}
if(volts<=9){
lcd_cmd_write(0x80);
lcd_str_write("right indicat on");
LPC_GPIO1->FIOSET = (1<<20);
LPC_GPIO1->FIOCLR = (1<<19);
}
if(10<=volts && volts<= 15){
LPC_GPIO1->FIOCLR = (1<<20);
LPC_GPIO1->FIOCLR = (1<<19);
}
}


void pwm_init(void) 
{
	int i=0;
	char br[20]="Abs break active";
	char brr[20]="break  de-active";
	lcd_cmd_write(0x80);
	lcd_str_write(br);
    LPC_PINCON->PINSEL3 &= ~(1 << 10);// P1.21 as PWM3
    LPC_PINCON->PINSEL3 |= (1 << 11);

	LPC_SC->PCONP |= (1 << 6); //PWM1 power/clk control enabled
	//LPC_SC->PCLKSEL0 |=(0x01 << 4); 

    LPC_PWM1->PR = 0;                  // PR = 3, PCLK / (PR + 1) = 1MHz / (0+1) = 1MHz 
    LPC_PWM1->MR0 = 10000;             // Total period PWM cycle = 10ms
    LPC_PWM1->MCR |= (1 << 1);         // Reset on MR0
    LPC_PWM1->LER |= (1 << 0);         // Enable MR0 Latch

  	LPC_PWM1->PCR |= (1 << 11);        // Enable PWM1.3 output

    LPC_PWM1->TCR = (1 << 0) | (1 << 3);  // Enable PWM Timer and PWM Mode

    while (i<=6) 
   	{
        LPC_PWM1->MR3 = 2000;   // 20% duty cycle
        LPC_PWM1->LER |= (1 << 3);  // Enable MR1 Latch
        delay(100);

        LPC_PWM1->MR3 = 4000;   // 40% duty cycle
        LPC_PWM1->LER |= (1 << 3);  // Enable MR1 Latch
        delay(100);

        LPC_PWM1->MR3 = 6000;   // 60% duty cycle
        LPC_PWM1->LER |= (1 << 3);  // Enable MR1 Latch
        delay(100);
       
	    LPC_PWM1->MR3 = 8000;   // 80% duty cycle
        LPC_PWM1->LER |= (1 << 3);  // Enable MR1 Latch
        delay(100);

		LPC_PWM1->MR3 = 10000;   // 100% duty cycle
        LPC_PWM1->LER |= (1 << 3);  // Enable MR1 Latch
        delay(100);
		i++;
	}
    lcd_cmd_write(0x80);
	lcd_str_write(brr);

}


void adc_init(){
LPC_PINCON->PINSEL1 |= (0x01<<16);//selct A_DC pin channel 1 pin
LPC_SC->PCONP |=ADD_CLK_EN;//enable 12 bit 
LPC_ADC->ADCR= PWRUP|CLKDIV|SEL_AD0_1;
}

void keypad()
{
uint8_t  i,j,val;
uint8_t scan[4]= {0xE,0xD,0xB,0x7};
int key[4][4]={{0,1,2,3},
                 {4,5,6,7},
				 {8,9,10,11},
				 {12,13,14,15}
				};
LPC_GPIO2->FIODIR |= ROW_PINS;

LPC_GPIO2->FIODIR &= ~COL_PINS;

for(i=0;i<4;i++)
{
LPC_GPIO2->FIOCLR |= ROW_PINS; //clear row lines
LPC_GPIO2->FIOSET |= scan[i] << 4; //activate single row at a time
val = LPC_GPIO2->FIOPIN & COL_PINS;//read column lines
for(j=0;j<4;j++)
{
if(val == scan[j]) break; //if any key is pressed stop scanning  
}
if(val != 0x0F) // if key pressed in the scanned row, print key using lookup table
{
if(key[i][j]==0){
if(k<1){
lcd_cmd_write(0x80);
lcd_str_write("hand break off");
LPC_GPIO1->FIOCLR = (1<<22);
k=k+1;
}
else{
LPC_GPIO1->FIOSET =(1<<27);
delay(1000);
LPC_GPIO1->FIOCLR =(1<<27);
}
}
else if(key[i][j]==1){
pwm_init();
LPC_PWM1->MR3 = 0;   // 0% duty cycle
LPC_PWM1->LER |= (1 << 3); 
}
else if(key[i][j]==4){
lcd_cmd_write(0x80);
lcd_str_write(" alcoholic air ");
LPC_GPIO1->FIOCLR = (1<<23);
LPC_GPIO1->FIOSET = (1<<27);
delay(2000);
LPC_GPIO1->FIOCLR = (1<<27);
lcd_cmd_write(0x80);
lcd_str_write("    welcome     "); 
}

}

}

}