# audio_pass-through<br>
Audio pass-through STM32F769I-Discovery Line in buffer copied to Line out<br>
This software builds heavily on the great work by https://community.st.com/people/Beaulier.Francois<br>
wm8994.c is modified from the BSP version<br>
Input Mixer L and R zeroed and muted registers 0x29 and 0x2A<br>
Read the data sheet. I know its long and complicated. Do it anyway.<br>
SW4STM32 project using STM32 HAL<br>
Output on serial port looks like<br>
Connected to STM32F769I-Discovery USART 1<br>
<br>
Audio I/O initialization OK<br>
SAI receive begin OK<br>
Copying Record buffer to Playback buffer<br>
Audio output OK<br>
<br>
Line level input is expected, not speaker level so turn your volume down at least to 50% or lower<br>
<br>
I have not tested microphone input or speaker output<br>
