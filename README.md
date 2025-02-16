STM32_RTOS_Guess_the_numbers

Overview

STM32_RTOS_Guess_the_numbers เป็นโปรเจกต์ที่ใช้ STM32 ร่วมกับ FreeRTOS เพื่อสร้างระบบที่สามารถรับข้อมูลภาพจาก UART, ประมวลผล, เปรียบเทียบค่ากับตัวเลขที่สุ่มขึ้นมา, และแสดงผลผ่านเสียงและ OLED Display

Features

  รับข้อมูลภาพขนาด 28×28 pixels ผ่าน UART
  
  ประมวลผลภาพเพื่อให้ได้ค่าผลลัพธ์
  
  ใช้โมเดล Digit Recognition เพื่อจำแนกตัวเลขจากภาพ
  
  สุ่มตัวเลขและเปรียบเทียบค่ากับผลลัพธ์จากการประมวลผล
  
  ส่งเสียงผ่าน DAC โดยใช้ TIM4 Interrupt
  
  แสดงผลการเปรียบเทียบผ่าน OLED Display

System Architecture

โปรเจกต์นี้ใช้ FreeRTOS เพื่อจัดการ 5 Tasks หลัก ได้แก่:

- UART Receive Task - รับข้อมูลภาพ 28×28 pixels ผ่าน UART

- Image Processing Task - ประมวลผลข้อมูลภาพ

- Digit Recognition Task - ใช้โมเดล Machine Learning เพื่อจำแนกตัวเลขจากภาพ

- Logic Task - สุ่มตัวเลข และเปรียบเทียบค่าผลลัพธ์ที่ได้

- Sound Output Task - ใช้ Timer 4 (TIM4) Interrupt และ DAC เพื่อส่งเสียงออก

- Display Task - แสดงผลผ่าน OLED Display

Machine Learning Model

  ใช้โมเดล Digit Recognition ที่ถูกฝึกบน MNIST Dataset
  
  โมเดลถูกแปลงเป็น TensorFlow Lite (TFLite) หรือ CMSIS-NN เพื่อให้สามารถรันบน STM32
  
  ใช้ STM32Cube.AI เพื่อ Convert โมเดลให้อยู่ในรูปแบบที่เหมาะสมกับ MCU

Hardware Requirements

  STM32 Development Board
  
  OLED Display (I2C/SPI)
  
  UART Interface
  
  DAC สำหรับส่งเสียงออก

Software Requirements

  STM32CubeIDE
  
  FreeRTOS
  
  CMSIS (สำหรับการใช้งาน FreeRTOS)
  
  TensorFlow Lite for Microcontrollers (TFLM) หรือ CMSIS-NN
  
  Libraries สำหรับ OLED และ DAC

Installation & Usage

  Flash Firmware: ใช้ STM32CubeIDE หรือ ST-Link Utility เพื่ออัปโหลดโค้ดไปยังบอร์ด
  
  เชื่อมต่ออุปกรณ์:
  
  ส่งข้อมูลภาพผ่าน UART
  
  เชื่อมต่อ OLED Display
  
  เชื่อมต่อลำโพงเข้ากับ DAC
  
  Run Project: เปิดบอร์ด STM32 และเริ่มต้นการทำงานของระบบ

Future Improvements

  ปรับปรุงความเร็วของ Image Processing และ Digit Recognition
  
  เพิ่มการรองรับโมเดลที่แม่นยำยิ่งขึ้น
  
  ปรับแต่งการใช้งาน Low-power Mode ให้เหมาะสมกับอุปกรณ์ฝังตัว

License

  โปรเจกต์นี้เป็น Open-source ภายใต้ MIT License
