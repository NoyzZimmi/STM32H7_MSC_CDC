# SomSdCardTest

Project for NUCLEO-H753ZI eval kit to test USB composite class for CDC ACM and MSC. 
- CDC is an simplified communication handling of main application. In this case, the rx thread receives a string from Virtual COM and puts it on a queue. The write thread is waiting for new messages on the queue and echos back the string. In non-compositie configuration (removing MSC), the echo works as expected.
- MSC implementation is based on the STM's example code (migrated to this NUCLEO): https://github.com/STMicroelectronics/x-cube-azrtos-h7/tree/main/Projects/STM32H735G-DK/Applications/USBX/Ux_Device_MSC

Hardware overview:
 - USB_OTG_FS used for USB communication:
    - full speed mode (12MB/s)
    - EP0, shared FIFO, 512 words
    - EP1, CDC CMD In, 64 words
    - EP2, CDC DATA, 128 words
    - EP3, MSC Data, 128 words
 - SDMMC1 used to connect to SD card in 1bit or 4bit wide mode. SD card used: SDCIT2/8GB (8GB microSDHC Industrial C10 A1 pSLC Card)
 - Middleware: Azure RTOS with USBX
 - Debug pins on CN9 connector A0 - A4 implemented for USB activation, MSC read and write activity.

NOTE: MPU used to configure memory sections for cache handling. USB region is placed in D1 region at 0x24078000 (32KByte) to accomodate USB buffers and PCD handle.


