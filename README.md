# covenlfo
Eurorack LFO Module based on SAMD21/ Xiao


![image of module](https://github.com/cctvfm/covenlfo/blob/main/IMG_E0173-01.jpeg)

# How to Hack Your Coven LFO  




If you are interested in modifying the code, you will first need to download the latest version of Arduino IDE which can be found here:  

https://www.arduino.cc/en/software 

Now, follow these instructions so that Arduino can recognize our board, the Seeeduino XIAO.  

https://wiki.seeedstudio.com/Seeeduino-XIAO/ 

Next, we need to install an additional library so that your code will compile. This library can be downloaded directly from inside the Arduino IDE by going to Tools> Manage Libraries... 

Once the Library Manager opens you can use the search bar located at the top to find the library we will need to install. Using the search bar, type in “FlashStorage_SAMD” and install the latest version. Note that this code was written with version 1.3.2 of the FlashStorage_SAMD library.

Select the Xiao Board: Tools -> Board -> Seeed SAMD -> Seeeduino Xiao

Once you have the Arduino IDE installed and recognizing your XIAO and have installed the “FlashStorage_SAMD” library, you are now ready to compile the code and modify it until your heart and ears are content.  

Read the comments for more info on what to HACK
