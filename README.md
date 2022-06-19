# Audio Loopback excample

Audio loopback STM32F769I-Discovery Line in buffer copied to Line out.
This software builds heavily on the great work by https://community.st.com/people/Beaulier.Francois

wm8994.c is modified from the BSP version  
Input Mixer L and R zeroed and muted registers 0x29 and 0x2A  
Read the data sheet. I know its long and complicated. Do it anyway.  
SW4STM32 project using STM32 HAL  
Output on serial port looks like  
Connected to STM32F769I-Discovery USART 1  

Line level input is expected, not speaker level so turn your volume down at least to 50% or lower
