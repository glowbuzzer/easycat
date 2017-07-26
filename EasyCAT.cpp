#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>

#include "EasyCAT.h"                         

//---- AB&T EasyCAT library V_1.2 -----------------------------------------------------------------	
//
// AB&T Tecnologie Informatiche - Ivrea Italy
// http://www.bausano.net
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm

//--- V_1.2 ---
//
// The MainTask function now return the state of the 
// Ethercat State machine and of the Wachdog


//--- V_1.1 --- 
// First official release.


#define PIN RPI_GPIO_P1_24

    
//---- constructor --------------------------------------------------------------------------------

EasyCAT::EasyCAT()                              //------- default constructor ---------------------- 
{                                               //                               
}

void EasyCAT::SPI_TransferTx(unsigned char Data)
{
    bcm2835_spi_transfer(Data);
};

void EasyCAT::SPI_TransferTxLast(unsigned char Data)
{
    bcm2835_spi_transfer(Data);
};

unsigned char EasyCAT::SPI_TransferRx(unsigned char Data)
{
    return bcm2835_spi_transfer(Data);
};

//---- EasyCAT board initialization ---------------------------------------------------------------

bool EasyCAT::Init()
{
    // Use for testing
    //bcm2835_set_debug(1);

    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root ?\n");
      return false;
    }

    //Setup SPI pins
    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root ?\n");
      return false;
    }

    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    //Set SPI data mode
    //	BCM2835_SPI_MODE0 = 0,  // CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
    //	BCM2835_SPI_MODE1 = 1,  // CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
    //	BCM2835_SPI_MODE2 = 2,  // CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
    //	BCM2835_SPI_MODE3 = 3,  // CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    //Set SPI clock speed
    //	BCM2835_SPI_CLOCK_DIVIDER_65536 = 0,       ///< 65536 = 262.144us = 3.814697260kHz (total H+L clock period)
    //	BCM2835_SPI_CLOCK_DIVIDER_32768 = 32768,   ///< 32768 = 131.072us = 7.629394531kHz
    //	BCM2835_SPI_CLOCK_DIVIDER_16384 = 16384,   ///< 16384 = 65.536us = 15.25878906kHz
    //	BCM2835_SPI_CLOCK_DIVIDER_8192  = 8192,    ///< 8192 = 32.768us = 30/51757813kHz
    //	BCM2835_SPI_CLOCK_DIVIDER_4096  = 4096,    ///< 4096 = 16.384us = 61.03515625kHz
    //	BCM2835_SPI_CLOCK_DIVIDER_2048  = 2048,    ///< 2048 = 8.192us = 122.0703125kHz
    //	BCM2835_SPI_CLOCK_DIVIDER_1024  = 1024,    ///< 1024 = 4.096us = 244.140625kHz
    //	BCM2835_SPI_CLOCK_DIVIDER_512   = 512,     ///< 512 = 2.048us = 488.28125kHz
    //	BCM2835_SPI_CLOCK_DIVIDER_256   = 256,     ///< 256 = 1.024us = 976.5625MHz
    //	BCM2835_SPI_CLOCK_DIVIDER_128   = 128,     ///< 128 = 512ns = = 1.953125MHz
    //	BCM2835_SPI_CLOCK_DIVIDER_64    = 64,      ///< 64 = 256ns = 3.90625MHz
    //	BCM2835_SPI_CLOCK_DIVIDER_32    = 32,      ///< 32 = 128ns = 7.8125MHz
    //	BCM2835_SPI_CLOCK_DIVIDER_16    = 16,      ///< 16 = 64ns = 15.625MHz
    //	BCM2835_SPI_CLOCK_DIVIDER_8     = 8,       ///< 8 = 32ns = 31.25MHz
    //	BCM2835_SPI_CLOCK_DIVIDER_4     = 4,       ///< 4 = 16ns = 62.5MHz
    //	BCM2835_SPI_CLOCK_DIVIDER_2     = 2,       ///< 2 = 8ns = 125MHz, fastest you can get
    //	BCM2835_SPI_CLOCK_DIVIDER_1     = 1,       ///< 1 = 262.144us = 3.814697260kHz, same as 0/65536
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);
    //Disable management of CS pin we will do it
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
    //Set CS pins polarity to low
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

    // Set the pin to be an output (CS pin)
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);

  ULONG TempLong;

  SPIWriteRegisterDirect (RESET_CTL, (DIGITAL_RST & ETHERCAT_RST)); // LAN9252 reset
  usleep (100);  // wait 100mS
  TempLong.Long = SPIReadRegisterDirect (BYTE_TEST, 4);             // read test register

  if (TempLong.Long == 0x87654321)                                  // if the test register is ok 
  {                                                                 // check also the READY flag
     TempLong.Long = SPIReadRegisterDirect (HW_CFG, 4);              //
     if (TempLong.Long & READY)                                     //
     {                                                              //
		printf("Enabling sync mode\n");
      TempLong.Long = 0x00000004;                        // enable interrupt from SYNC 0
      SPIWriteRegisterIndirect (TempLong.Long, 0x204);   // in AL event mask register and disable 
                                                         // other interrupt sources   

      TempLong.Long = 0x00000111;                        // set LAN9252 interrupt pin driver  
      SPIWriteRegisterDirect (0x54, TempLong.Long);      // as push-pull active high
                                                         // (On the EasyCAT board the IRQ pin
                                                         // is inverted by a mosfet, so Arduino                                                        
                                                         // receives an active low signal)
                                                         
      TempLong.Long = 0x00000001;                        // enable LAN9252 interrupt                 
      SPIWriteRegisterDirect (0x5C, TempLong.Long);      //   

        //Return SPI pins to default inputs state
        bcm2835_spi_end();
        return true;                                                // initalization completed
     }
   }
     
  //Return SPI pins to default inputs state
  bcm2835_spi_end();
  bcm2835_close();

  return false;                                                     // initialization failed
};  


//---- EtherCAT task ------------------------------------------------------------------------------

unsigned char EasyCAT::MainTask()                           // must be called cyclically by the application

{
  bool WatchDog = 0;
  bool Operational = 0; 
  unsigned char i;
  ULONG TempLong;
  unsigned char Status;  

  //Setup SPI pins
  if (!bcm2835_spi_begin())
  {
    printf("bcm2835_spi_begin failed. Are you running as root ?\n");
    return 0;
  }
  // Set the pin to be an output (CS pin)
  bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);

  TempLong.Long = SPIReadRegisterIndirect (WDOG_STATUS, 1); // read watchdog status
  if ((TempLong.Byte[0] & 0x01) == 0x01)                    //
    WatchDog = 0;                                           // set/reset the corrisponding flag
  else                                                      //
    WatchDog = 1;                                           //
    
  TempLong.Long = SPIReadRegisterIndirect (AL_STATUS, 1);   // read the EtherCAT State Machine status
  Status = TempLong.Byte[0] & 0x0F;                         //
  if (Status == ESM_OP)                                     // to see if we are in operational state
    Operational = 1;                                        //
  else                                                      // set/reset the corrisponding flag
    Operational = 0;                                        //    

                                                            //--- process data transfert ----------
                                                            //                                                        
  if (WatchDog | !Operational)                              // if watchdog is active or we are 
  {                                                         // not in operational state, reset 
    for (i=0; i<4; i++)                                     // the output buffer
    BufferOut.Long[i] = 0;                                  //

/*                                                          // debug
    if (!Operational)                                       //
      printf("Not operational\n");							//
    if (WatchDog)                                           //    
      printf("WatchDog\n");									//  
*/                                                          //
   
  }
  
  else                                                      
  {                                                         
    SPIReadProcRamFifo();                                   // otherwise transfer process data from 
  }                                                         // the EtherCAT core to the output buffer  

  SPIWriteProcRamFifo();                                    // we always transfer process data from
                                                            // the input buffer to the EtherCAT core  
                                                            
  //Return SPI pins to default inputs state
  bcm2835_spi_end();

  if (WatchDog)                                             // return the status of the State Machine      
  {                                                         // and of the watchdog
    Status |= 0x80;                                         //
  }                                                         //
  return Status;                                            //  

}

    
//---- read a directly addressable registers  -----------------------------------------------------

unsigned long EasyCAT::SPIReadRegisterDirect (unsigned short Address, unsigned char Len)

                                                   // Address = register to read
                                                   // Len = number of bytes to read (1,2,3,4)
                                                   //
                                                   // a long is returned but only the requested bytes
                                                   // are meaningful, starting from LsByte                                                 
{
  ULONG Result; 
  UWORD Addr;
  Addr.Word = Address; 
  unsigned char i; 
  
  SCS_Low_macro                                             // SPI chip select enable

  SPI_TransferTx(COMM_SPI_READ);                            // SPI read command
  SPI_TransferTx(Addr.Byte[1]);                             // address of the register
  SPI_TransferTxLast(Addr.Byte[0]);                         // to read, MsByte first
 
  for (i=0; i<Len; i++)                                     // read the requested number of bytes
  {                                                         // LsByte first 
    Result.Byte[i] = SPI_TransferRx(DUMMY_BYTE);            //
  }                                                         //    
  
  SCS_High_macro                                            // SPI chip select disable 
 
  return Result.Long;                                       // return the result
}


//---- write a directly addressable registers  ----------------------------------------------------

void EasyCAT::SPIWriteRegisterDirect (unsigned short Address, unsigned long DataOut)

                                                   // Address = register to write
                                                   // DataOut = data to write
{ 
  ULONG Data; 
  UWORD Addr;
  Addr.Word = Address;
  Data.Long = DataOut;    

  SCS_Low_macro                                             // SPI chip select enable  
  
  SPI_TransferTx(COMM_SPI_WRITE);                           // SPI write command
  SPI_TransferTx(Addr.Byte[1]);                             // address of the register
  SPI_TransferTx(Addr.Byte[0]);                             // to write MsByte first

  SPI_TransferTx(Data.Byte[0]);                             // data to write 
  SPI_TransferTx(Data.Byte[1]);                             // LsByte first
  SPI_TransferTx(Data.Byte[2]);                             //
  SPI_TransferTxLast(Data.Byte[3]);                         //
 
  SCS_High_macro                                            // SPI chip select enable   
}


//---- read an undirectly addressable registers  --------------------------------------------------

unsigned long EasyCAT::SPIReadRegisterIndirect (unsigned short Address, unsigned char Len)

                                                   // Address = register to read
                                                   // Len = number of bytes to read (1,2,3,4)
                                                   //
                                                   // a long is returned but only the requested bytes
                                                   // are meaningful, starting from LsByte                                                  
{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;
                                                            // compose the command
                                                            //
  TempLong.Byte[0] = Addr.Byte[0];                          // address of the register
  TempLong.Byte[1] = Addr.Byte[1];                          // to read, LsByte first
  TempLong.Byte[2] = Len;                                   // number of bytes to read
  TempLong.Byte[3] = ESC_READ;                              // ESC read 

  SPIWriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);     // write the command

  do
  {                                                         // wait for command execution
    TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD,4);  //
  }                                                         //
  while(TempLong.Byte[3] & ECAT_CSR_BUSY);                  //
                                                             
                                                              
  TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_DATA,Len); // read the requested register
  return TempLong.Long;                                     //
}


//---- write an undirectly addressable registers  -------------------------------------------------

void  EasyCAT::SPIWriteRegisterIndirect (unsigned long DataOut, unsigned short Address)

                                                   // Address = register to write
                                                   // DataOut = data to write                                                    
{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;


  SPIWriteRegisterDirect (ECAT_CSR_DATA, DataOut);            // write the data

                                                              // compose the command
                                                              //                                
  TempLong.Byte[0] = Addr.Byte[0];                            // address of the register  
  TempLong.Byte[1] = Addr.Byte[1];                            // to write, LsByte first
  TempLong.Byte[2] = 4;                                       // we write always 4 bytes
  TempLong.Byte[3] = ESC_WRITE;                               // ESC write

  SPIWriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);       // write the command

  do                                                          // wait for command execution
  {                                                           //
    TempLong.Long = SPIReadRegisterDirect (ECAT_CSR_CMD, 4);  //  
  }                                                           //  
  while (TempLong.Byte[3] & ECAT_CSR_BUSY);                   //
  
}


//---- read from process ram fifo ----------------------------------------------------------------

void EasyCAT::SPIReadProcRamFifo()    // read 32 bytes from the output process ram, through the fifo
                                      //        
                                      // these are the bytes received from the EtherCAT master and
                                      // that will be use by our application to write the outputs
{
  ULONG TempLong;
  unsigned char i;
  
  SPIWriteRegisterDirect (ECAT_PRAM_RD_ADDR_LEN, 0x00201000);   // we always read 32 bytes   0x0020----
                                                                // output process ram offset 0x----1000
                                                                
  SPIWriteRegisterDirect (ECAT_PRAM_RD_CMD, 0x80000000);        // start command                                                                  
                                                                                                                               
  do                                                            // wait for data to be transferred      
  {                                                             // from the output process ram 
    TempLong.Long = SPIReadRegisterDirect (ECAT_PRAM_RD_CMD,4); // to the read fifo 
  }                                                             //    
  while (!(TempLong.Byte[0] & PRAM_READ_AVAIL) || (TempLong.Byte[1] != 8));     
  
 
  SCS_Low_macro                                                 // SPI chip select enable  
  
  SPI_TransferTx(COMM_SPI_READ);                                // SPI read command
  SPI_TransferTx(0x00);                                         // address of the read  
  SPI_TransferTxLast(0x00);                                     // fifo MsByte first
  

  for (i=0; i<32; i++)                                          // 32 bytes read loop 
  {                                                             // 
    BufferOut.Byte[i] = SPI_TransferRx(DUMMY_BYTE);             //
  }                                                             //
    
  SCS_High_macro                                                // SPI chip select disable    
}


//---- write to the process ram fifo --------------------------------------------------------------

void EasyCAT::SPIWriteProcRamFifo()    // write 32 bytes to the input process ram, through the fifo
                                       //    
                                       // these are the bytes that we have read from the inputs of our                   
                                       // application and that will be sent to the EtherCAT master
{
  ULONG TempLong;
  unsigned char i;

  SPIWriteRegisterDirect (ECAT_PRAM_WR_ADDR_LEN, 0x00201200);   // we always write 32 bytes 0x0020----
                                                                // input process ram offset 0x----1200

  SPIWriteRegisterDirect (ECAT_PRAM_WR_CMD, 0x80000000);        // start command  

  do                                                            // check fifo has available space     
  {                                                             // for data to be written
    TempLong.Long = SPIReadRegisterDirect (ECAT_PRAM_WR_CMD,4); //  
  }                                                             //    
  while (!(TempLong.Byte[0] & PRAM_WRITE_AVAIL) || (TempLong.Byte[1] < 8) );             
  
  
  SCS_Low_macro                                                 // enable SPI chip select
  
  SPI_TransferTx(COMM_SPI_WRITE);                               // SPI write command
  SPI_TransferTx(0x00);                                         // address of the write fifo 
  SPI_TransferTx(0x20);                                         // MsByte first 

  for (i=0; i<31; i++)                                          // 32 bytes write loop
  {                                                             //
    SPI_TransferTx (BufferIn.Byte[i]);                          // 
  }                                                             //
                                                                //  
  SPI_TransferTxLast (BufferIn.Byte[31]);                       //
  
  SCS_High_macro                                                // disable SPI chip select      
} 
