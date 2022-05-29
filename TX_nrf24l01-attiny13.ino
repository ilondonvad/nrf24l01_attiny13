// BYTE type definition
//https://forum.arduino.cc/t/solved-attiny13a-and-nrf24l01/431914/3
// 128kHz
//****************************************************
// SPI(nRF24L01) commands
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD_NOAA     0xB0  // Define TX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NOP             0xFF  // Define No Operation, might be used to read status register
//***************************************************
#define RX_DR    0x40
#define TX_DS    0x20
#define MAX_RT   0x10
//***************************************************
// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address

#define SPI_PORT PORTB
#define CE 2
#define CSN 3
#define SPI_SCK 4
#define SPI_MOSI 0
#define cbi(x,y)    x&= ~(1<<y)
#define sbi(x,y)    x|= (1<<y)
#define SPI_DDR (*((&SPI_PORT) -1))
#define SPI_PIN (*((&SPI_PORT) -2))
 
#define TX_PLOAD_WIDTH  1               // 32 byte max, количество байт которые хотите передать

#include <avr/sleep.h>

byte tx_buf[TX_PLOAD_WIDTH] = {0};      //Данные для отправки 
boolean flag=0;                         //флаг кнопки
byte btn_count[1]={0};                  //количесво раз нажатой кнопки,однократно
byte hold_count[1]={0};                 //счетчик увеличиваеться при удержаной кнопке
byte timer_count[1]={0};                //счетчик таймера WDT, регулирует паузу между однократным нажатием и отправкой данных
boolean sent=true;                      //статус отправленых данных, после отправки разрешает сон

ISR(WDT_vect) {                                   //таймер WDT, основной обработчик нажатий
  timer_count[0]++;                               //увеличиваеться каждый раз при срабатывания таймера
  if((PINB&0x02)==0 && flag==true){               //если кнопка удерживается
      hold_count[0]++;                            //увеличивается если кнопка удерживается
      if(hold_count[0]==10 ){                     //если кнопка удерживалась заданое количество инкриментов hold_coun, можно регулировать время срабатывания удержаной кнопки,частотой срабатывания WDT тоже
        tx_data(195);                             //отправка данных
        btn_count[0]=0;                           //обнуление счетчика нажатий так как кнопка была удержана и данные отправились
        sent=true;                                //флаг состояния отправленых данных, разрешает сон после отправки
        WDTCR &=~(1<<WDTIE);                      //запрет прерывания по таймеру, данные отправлены больше тут делать нечего
        GIMSK |= (1<<INT0);                       //разрешить внешнее прерывания по нажатию на кнопку
      }
  }
  else if(timer_count[0]>7 && btn_count[0]==1){   //если кнопка нажата один раз
    timer_count[0]=0;                             //
    btn_count[0]=0;                               //обнуление счетчиков
    tx_data(100);                                 //отправка данных
    sent=true;                                    //флаг состояния отправленых данных, разрешает сон после отправки
    WDTCR &=~(1<<WDTIE);                          //запрет прерывания по таймеру, данные отправлены больше тут делать нечего
    GIMSK |= (1<<INT0);                           //разрешить внешнее прерывания по нажатию на кнопку                  
  }
  else if(timer_count[0]>5 && btn_count[0]>=2){   //если кнопка нажата 2 или более раз
    timer_count[0]=0;                             //
    btn_count[0]=0;                               //обнуление счетчиков
    tx_data(180);                                 //отправка данных
    sent=true;                                    //флаг состояния отправленых данных, разрешает сон после отправки
    WDTCR &=~(1<<WDTIE);                          //запрет прерывания по таймеру, данные отправлены больше тут делать нечего
    GIMSK |= (1<<INT0);                           //разрешить внешнее прерывания по нажатию на кнопку                  
  }
}
ISR(INT0_vect){                                   //прерывания внешнее по нажатие кнопки
  sent=false;                                     //флаг состояния отправленых данных, запрещает сон мы хотим поработать
  WDTCR |= (1<<WDTIE);                            //разрешает прерывания по таймеру WDT, обработка кнопки
  GIMSK &= ~(1<<INT0);                            //запрещает прерывания по кнопке пока обрабатываем нажатия и МК не уходил в луп по прерыванию INT0_vect
}



int main(void){
init();
{
   delay(1000);                                                   // задержка перед включением
   DDRB &= ~_BV(1);                                               // пин с кнопкой на вход
   PORTB |= _BV(1);                                               // подтяжка пина с кнопкой к питанию
   delay(70);                                                     // задержка для инициализации порта, ели ее убрать то будт ложное срабатывание при включение, нужно для зарядки RC цепочки на кнопке если есть
   
   WDTCR |= (1<<WDP0);                                            // настройка WDT таймера на 32ms, время может отличаться
      
   MCUCR |= (0<<ISC01) | (0 << ISC00);                            // настройка внешних прерываний INT0 на FALLING так как только низкий уровень разбудит МК
   GIMSK |= (1<<INT0);                                            // прерывания по INT0

   sei();                                                         // разрешить прерывания
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);                           // настройка сна
   
   SPI_DDR |= ((1<<SPI_SCK) | (1<<SPI_MOSI)| (1<<CE)| (1<<CSN));  // Initialize SPI IO port
}

while(1) { 
  
  if((PINB&0x02)==0 && flag==false){          // кнопка нажата но не отпущена
      timer_count[0]=0;                       // обнуление при каждом нажатие, если нажатий более 1, это обнуление позволяет сначало понажимать нужное количетво раз а только потом обрабатывать нажатия
      delay(10);                              // програмный антидребезг кнопки
      flag=1;                                 // для кнопки
      btn_count[0]++;                         // считает сколько раз кнопка была нажата
    }
    else if((PINB&0x02)==2 && flag==true){    // отаускание кнопки
      delay(10);                              // програмный антидребезг
      hold_count[0]=0;                        // обнуляет hold_count, потомучто кнопка не удержана а была отпущена
      flag=0;                                 // для кнопки
    } 
 }
 
 if(sent){delay(100);sleep_mode();}           // уход в сон если данные отправились
}


void tx_data(byte comand){                           // отправка данных   
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x7E);             //PTX,PWR UP,CRC 2bytes,Enable CRC,no interrurt IRQ
  delayMicroseconds(2000);                          //wait for power upp
  SPI_RW_Reg(FLUSH_TX,0);                           //kill old data
  delayMicroseconds(50); 
  TX_Mode();                                        // настройка модуля
  sbi (SPI_PORT, CE); 
  //delayMicroseconds(80);
  tx_buf[0]= comand;
  SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH); //tx_buf, data to 32 byte buffer in nrf
  SPI_RW_Reg(WRITE_REG+STATUS,0xff);                //NOP
  delayMicroseconds(30);
  cbi (SPI_PORT, CE);
  delayMicroseconds(800);
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x00);             //pwr down 
}
/**************************************************
 * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
void SPI_RW(byte data)
{
    byte i = 8;

    do{ 
        SPI_PORT &= ~(1<<SPI_MOSI);     // clr mosi
        if (data & 0x80) SPI_PIN = (1<<SPI_MOSI);
        SPI_PIN = (1<<SPI_SCK);         // clk hi
        data <<= 1;
        
        SPI_PIN = (1<<SPI_SCK);         // clk lo
    }while(--i); 
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
void SPI_RW_Reg(byte reg, byte value)
{

   cbi (SPI_PORT, CSN);     
   SPI_RW(reg);                   // select register
   SPI_RW(value);                 // ..and write value to it..
   sbi (SPI_PORT, CSN);      
                
}

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
void SPI_Write_Buf(byte reg, byte *pBuf, byte bytes)
{
  byte i;
  cbi (SPI_PORT, CSN);      
 // digitalWrite(CSNq, 0);    // Set CSN low, init SPI tranaction
  SPI_RW(reg);                // Select register to write to and read status unsigned char
  for(i=0;i<32; i++)          // then write all unsigned char in buffer(*pBuf) for(i=0;i<bytes; i++),по длине буфера не работало, пришлось в ручную поставить 32 byte
  {
    SPI_RW(*pBuf++);          //*pBuf++
  }
  sbi (SPI_PORT, CSN);     
  //digitalWrite(CSNq, 1);    // Set CSN high again
  
}


/**************************************************
 * Function: TX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 unsigned chars) is enabled, & PRIM:TX.
 * 
 * ToDo: One high pulse(>10us) on CE will now send this
 * packet and expext an acknowledgment from the RX device.
 **************************************************/
void TX_Mode(void)
{
  //digitalWrite(CEq, 0);
  //SPI_RW_Reg(WRITE_REG + SETUP_AW,0X01);                            //3 byte adress
  //SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // Writes TX_Address to nRF24L01
  //SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // RX_Addr0 same as TX_Adr for Auto.Ack
  SPI_RW_Reg(WRITE_REG + TX_ADDR, 0xE7E7E7E7E7);                      //adress
  //SPI_RW_Reg(WRITE_REG + RX_ADDR_P0, 0xE7E7E7E7E7);
  SPI_RW_Reg(WRITE_REG + SETUP_AW, 0x03);                     //5 bytes adress width
  
  
  //SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);                              // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);                            // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x00);                           // Re-Transmint disabled
  SPI_RW_Reg(WRITE_REG + RF_CH,0x64);                                 // Select RF channel
  SPI_RW_Reg(WRITE_REG + RF_SETUP,5);                                 // -6dBm, Datarate:1Mbps, LNA:HCURR
  //SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);                             // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
  //SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);
 
 
  //SPI_RW_Reg(WRITE_REG + 0x1C,0x01);     //dynamic payload pip0
 // SPI_RW_Reg(WRITE_REG + 0x1D, 0x04);     //dyn payload feature
  //SPI_RW_Reg(WRITE_REG + 0x1D, 0x05);     //dyn payload feature +noacc pay
 // digitalWrite(CEq, 1);
}
/**************************************************/
