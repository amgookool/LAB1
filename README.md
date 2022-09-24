# Question 4

## (a) Coding Standards
---
The BARR C embedded C coding standard was used to standardized the development code. This standard was developed to minimize bugs in firmware by focusing on practical rules that keep bugs out while also improving the maintability and portablity of embedded software.

## (b) Toolchain Features
---
Both the ESP8266 RTOS SDK Programming Guide and the Mastering FreeRTOS Real Time Kernel Guide were used for interpretting and understanding the various macro definitions, kernel specific functions and the peripheral specific macros and functions of the ESP device. This was very helpful as it provided information on the parameters of the functions and what they return. Additionally, the ESP8266 RTOS SDK provided information on how to use the macros and functions for the configuration of the GPIO and I2C periheral interfaces.Lastly, the ADS1115 datasheet was used to determine the programming method for the device. This refers to how the ESP8266 will write to and read from the ADS1115 sensor. The use of these documentation is crucial for development of complex embedded systems as it is a great source of information on the operating principle of the ESP8266 SDK and the FreeRTOS kernel.  

## (c) Debugging Practices
---
The use of the embedded article on debugging provided insight on how to debug a microcontroller based system. The article describes four phases of the debugging process which are testing, stabilization, localization and correction.

The testing phase refers to testing the capabilities of the program to perform correctly under a wide range of input values. 
The program is tested under normal conditions, special cases and boundary conditions and are engineered to force the execution of all program branches, ensuring that every decision node is executed correctly. Strange performance by the program during testing can be considered as a potential bug and should be investigated.

The stabilization phase refers to an attempt to control conditions to the extent where a specific bug can be generated at will.
Note that certain classes if bugs typical in embedded C programs are difficult to stabilize as any change in the source code or linking process can significantly alter the bugs' behaviour or even make them go away.

The localization phase refers to the programmer narrowing the range of possible code blocks until the bug can be isolated. There are three (3) approaches to achieve this which are:

- Construct a hypothesis to explain how such data might be created, then modify the experiment (test) to test the validity of the hypothesis.

-  Step-through the suspected code block watching carefully for abnormal behaviour. Since the programmer knows what's supposed to happen, therefore the problem can be pinpointed by stepping through the lines of code.

- Examining a trace history of the executed code. Microprocessors emulatores can be used to capture a trace of the program as it executes, and hardware breakpoints can be used to stop execution where desired. The trace history is then used to reconstruct what happened in when the bug occured.

The correction phase refers to the eradication of the bug once it is located. Note that some bugs may reflects a conceptual design flaw in the system. 

Understanding the phases of the debugging process and utilizing the techniques specified in the localization phase will prove very useful for detecting and correcting peculiar behaviour (bug) a complex embeddded system may have. 
