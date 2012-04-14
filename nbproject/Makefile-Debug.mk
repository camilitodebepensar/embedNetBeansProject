#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-MacOSX
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/1801028251/DNETcKAPI.o \
	${OBJECTDIR}/_ext/682039346/TimeAlarms.o \
	${OBJECTDIR}/_ext/1823880294/Server.o \
	${OBJECTDIR}/_ext/285383713/new.o \
	${OBJECTDIR}/_ext/691741071/WDelay.o \
	${OBJECTDIR}/_ext/1937105257/w5100.o \
	${OBJECTDIR}/_ext/1569669406/WMath.o \
	${OBJECTDIR}/_ext/2045159674/Matrix.o \
	${OBJECTDIR}/_ext/616250782/OSC.o \
	${OBJECTDIR}/_ext/691741071/WAnalog.o \
	${OBJECTDIR}/_ext/1801028251/WFConnectionManager.o \
	${OBJECTDIR}/_ext/1318557376/Servo.o \
	${OBJECTDIR}/_ext/1788166970/Wire.o \
	${OBJECTDIR}/_ext/821730051/Encoder.o \
	${OBJECTDIR}/_ext/285383713/Stream.o \
	${OBJECTDIR}/_ext/1195823272/GBEthernetClient.o \
	${OBJECTDIR}/_ext/1385375812/Matrix.o \
	${OBJECTDIR}/_ext/101011447/SPI.o \
	${OBJECTDIR}/_ext/1195823272/GBIMAC.o \
	${OBJECTDIR}/_ext/1597197376/Deeprom.o \
	${OBJECTDIR}/_ext/318936080/I2C_Ambient.o \
	${OBJECTDIR}/_ext/481273448/GUI.o \
	${OBJECTDIR}/_ext/1801028251/WFConnectionProfile.o \
	${OBJECTDIR}/_ext/2037745403/Gallery.o \
	${OBJECTDIR}/_ext/1195823272/GB4DLcdDriver.o \
	${OBJECTDIR}/_ext/360590444/Time.o \
	${OBJECTDIR}/_ext/1801028251/WFInit.o \
	${OBJECTDIR}/_ext/346905599/wiring_analog.o \
	${OBJECTDIR}/_ext/1801028251/WFMac.o \
	${OBJECTDIR}/_ext/920020366/Messenger.o \
	${OBJECTDIR}/_ext/691741071/WDigital.o \
	${OBJECTDIR}/_ext/1801028251/WF_Spi.o \
	${OBJECTDIR}/_ext/1195823272/GBEthernetServer.o \
	${OBJECTDIR}/_ext/547757580/I2C_Magnetometer.o \
	${OBJECTDIR}/_ext/1195823272/GBStatus.o \
	${OBJECTDIR}/_ext/1843772414/Wire.o \
	${OBJECTDIR}/_ext/1803262782/I2C_Thermometer.o \
	${OBJECTDIR}/_ext/1370567834/I2C_Clock.o \
	${OBJECTDIR}/_ext/2008123095/I2C_IMU.o \
	${OBJECTDIR}/_ext/1801028251/DNS.o \
	${OBJECTDIR}/_ext/1801028251/DHCP.o \
	${OBJECTDIR}/_ext/1871859325/SdFile.o \
	${OBJECTDIR}/_ext/913766812/I2C_Serial.o \
	${OBJECTDIR}/_ext/346905599/WMath.o \
	${OBJECTDIR}/_ext/1871859325/Sd2Card.o \
	${OBJECTDIR}/_ext/1801028251/Reboot.o \
	${OBJECTDIR}/_ext/1801028251/TCP.o \
	${OBJECTDIR}/_ext/346905599/wiring_shift.o \
	${OBJECTDIR}/_ext/2006777925/LiquidCrystal.o \
	${OBJECTDIR}/_ext/346905599/cpp-startup.o \
	${OBJECTDIR}/_ext/1569669406/main.o \
	${OBJECTDIR}/_ext/1011032836/Stepper.o \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/_ext/1801028251/SNTP.o \
	${OBJECTDIR}/_ext/1307181659/I2C_Barometer.o \
	${OBJECTDIR}/_ext/169008908/Client.o \
	${OBJECTDIR}/_ext/765940945/I2C_RGB_LED.o \
	${OBJECTDIR}/_ext/1086598594/TcpServer.o \
	${OBJECTDIR}/_ext/1158258697/w5100.o \
	${OBJECTDIR}/_ext/82939850/Matrix.o \
	${OBJECTDIR}/_ext/1801028251/ARP.o \
	${OBJECTDIR}/_ext/1482968431/Sd2Card.o \
	${OBJECTDIR}/_ext/1434022269/I2C_Climate.o \
	${OBJECTDIR}/_ext/346905599/crti.o \
	${OBJECTDIR}/_ext/169008908/Server.o \
	${OBJECTDIR}/_ext/1269018932/main.o \
	${OBJECTDIR}/_ext/1086598594/DNETcK.o \
	${OBJECTDIR}/_ext/1269018932/WPWM.o \
	${OBJECTDIR}/_ext/1801028251/ICMP.o \
	${OBJECTDIR}/_ext/1569669406/Print.o \
	${OBJECTDIR}/_ext/346905599/Tone.o \
	${OBJECTDIR}/_ext/285383713/WMath.o \
	${OBJECTDIR}/_ext/481273448/Serial_LCD.o \
	${OBJECTDIR}/_ext/1195823272/WizFi210.o \
	${OBJECTDIR}/_ext/2122273751/I2C_IMU2.o \
	${OBJECTDIR}/LocalLibrary.o \
	${OBJECTDIR}/_ext/208455599/nmea.o \
	${OBJECTDIR}/_ext/1569669406/wiring_pulse.o \
	${OBJECTDIR}/_ext/117181477/twi.o \
	${OBJECTDIR}/_ext/1935777686/DS1307RTC.o \
	${OBJECTDIR}/_ext/135288884/File.o \
	${OBJECTDIR}/_ext/2129506280/OneWire.o \
	${OBJECTDIR}/_ext/1195823272/SC16SpiTransport.o \
	${OBJECTDIR}/_ext/1760848399/WShift.o \
	${OBJECTDIR}/_ext/346905599/wiring.o \
	${OBJECTDIR}/_ext/1167644829/I2C_Height_IOs.o \
	${OBJECTDIR}/_ext/1801028251/WFConnectionAlgorithm.o \
	${OBJECTDIR}/_ext/691741071/program.o \
	${OBJECTDIR}/_ext/1482968431/SdFile.o \
	${OBJECTDIR}/_ext/285383713/wiring_analog.o \
	${OBJECTDIR}/_ext/1801028251/NBNS.o \
	${OBJECTDIR}/_ext/346905599/WString.o \
	${OBJECTDIR}/_ext/691741071/WTone.o \
	${OBJECTDIR}/_ext/1823880294/Client.o \
	${OBJECTDIR}/_ext/2037749790/Serial_GPS.o \
	${OBJECTDIR}/_ext/1167918412/SoftwareSerial.o \
	${OBJECTDIR}/_ext/462070156/I2C_Clock2.o \
	${OBJECTDIR}/_ext/285383713/WString.o \
	${OBJECTDIR}/_ext/1801028251/WFDataTxRx.o \
	${OBJECTDIR}/_ext/1569669406/WInterrupts.o \
	${OBJECTDIR}/_ext/169308162/SoftwareSerial.o \
	${OBJECTDIR}/_ext/1269018932/WTone.o \
	${OBJECTDIR}/_ext/1937105257/socket.o \
	${OBJECTDIR}/_ext/904039052/MatrixMath.o \
	${OBJECTDIR}/_ext/1801028251/WFPowerSave.o \
	${OBJECTDIR}/_ext/1801028251/WFMgmtMsg.o \
	${OBJECTDIR}/_ext/1697618995/Button.o \
	${OBJECTDIR}/_ext/481273448/proxySerial.o \
	${OBJECTDIR}/_ext/1111878700/MenuBackend.o \
	${OBJECTDIR}/_ext/1801028251/WFDriverCom.o \
	${OBJECTDIR}/_ext/1801028251/UDP.o \
	${OBJECTDIR}/_ext/1888968704/SoftPWMServo.o \
	${OBJECTDIR}/_ext/102619962/Sprite.o \
	${OBJECTDIR}/_ext/346905599/wiring_pulse.o \
	${OBJECTDIR}/_ext/1923869628/nmea.o \
	${OBJECTDIR}/_ext/1086598594/UdpClient.o \
	${OBJECTDIR}/_ext/422011384/pic32_RTC.o \
	${OBJECTDIR}/_ext/1789163801/EEPROM.o \
	${OBJECTDIR}/_ext/346905599/WInterrupts.o \
	${OBJECTDIR}/_ext/169008908/Udp.o \
	${OBJECTDIR}/_ext/1760848399/WMath.o \
	${OBJECTDIR}/_ext/346905599/exceptions.o \
	${OBJECTDIR}/_ext/1569669406/HardwareSerial.o \
	${OBJECTDIR}/_ext/1163168747/DSPI.o \
	${OBJECTDIR}/_ext/355312364/NewSoftSerialTest.o \
	${OBJECTDIR}/_ext/346905599/wiring_digital.o \
	${OBJECTDIR}/_ext/338935723/EEPROM.o \
	${OBJECTDIR}/_ext/1569669406/wiring_analog.o \
	${OBJECTDIR}/_ext/1048325648/Stepper.o \
	${OBJECTDIR}/_ext/346905599/HardwareSerial_usb.o \
	${OBJECTDIR}/_ext/1801028251/Delay.o \
	${OBJECTDIR}/_ext/1569669406/Tone.o \
	${OBJECTDIR}/_ext/1559931568/TwoNSSTest.o \
	${OBJECTDIR}/_ext/691741071/WPulse.o \
	${OBJECTDIR}/_ext/1320345559/I2C_Stepper.o \
	${OBJECTDIR}/_ext/972814340/Servo.o \
	${OBJECTDIR}/_ext/1823880294/Ethernet.o \
	${OBJECTDIR}/_ext/691741071/main.o \
	${OBJECTDIR}/_ext/1269018932/WPulse.o \
	${OBJECTDIR}/_ext/1163757000/Wire.o \
	${OBJECTDIR}/_ext/1269018932/WAnalog.o \
	${OBJECTDIR}/_ext/285383713/wiring_digital.o \
	${OBJECTDIR}/_ext/1442948484/I2C_RGBC_Reader.o \
	${OBJECTDIR}/_ext/1760848399/Print.o \
	${OBJECTDIR}/_ext/698302099/int.o \
	${OBJECTDIR}/_ext/1569669406/wiring_shift.o \
	${OBJECTDIR}/_ext/366363529/LiquidCrystal.o \
	${OBJECTDIR}/_ext/1269018932/WInterrupts.o \
	${OBJECTDIR}/_ext/1801028251/Tick.o \
	${OBJECTDIR}/embedExample.o \
	${OBJECTDIR}/_ext/285383713/main.o \
	${OBJECTDIR}/_ext/1649275740/Password.o \
	${OBJECTDIR}/_ext/346905599/Print.o \
	${OBJECTDIR}/_ext/1523693162/I2C_Compass.o \
	${OBJECTDIR}/_ext/1801028251/WFParamMsg.o \
	${OBJECTDIR}/_ext/1195823272/GBEthernet.o \
	${OBJECTDIR}/_ext/1769378116/TimerSerial.o \
	${OBJECTDIR}/_ext/1801028251/Helpers.o \
	${OBJECTDIR}/_ext/1851770985/twi.o \
	${OBJECTDIR}/_ext/18730292/Potentiometer.o \
	${OBJECTDIR}/_ext/616242143/FiniteStateMachine.o \
	${OBJECTDIR}/_ext/1871859325/SdVolume.o \
	${OBJECTDIR}/_ext/2122985726/I2C_20x4.o \
	${OBJECTDIR}/_ext/285383713/Print.o \
	${OBJECTDIR}/_ext/346905599/HardwareSerial.o \
	${OBJECTDIR}/_ext/285383713/WInterrupts.o \
	${OBJECTDIR}/_ext/1195823272/SpiTransport.o \
	${OBJECTDIR}/_ext/1557079726/Stepper.o \
	${OBJECTDIR}/_ext/1940649685/Keypad.o \
	${OBJECTDIR}/_ext/1215421894/Wire.o \
	${OBJECTDIR}/_ext/2115340346/Sprite.o \
	${OBJECTDIR}/_ext/2037745403/proxySerial.o \
	${OBJECTDIR}/_ext/1158258697/socket.o \
	${OBJECTDIR}/_ext/1801028251/DWIFIcKAPI.o \
	${OBJECTDIR}/_ext/2037745403/Serial_LCD.o \
	${OBJECTDIR}/_ext/1482968431/SdVolume.o \
	${OBJECTDIR}/_ext/1416585193/Supervisor.o \
	${OBJECTDIR}/_ext/1712916022/Servo.o \
	${OBJECTDIR}/_ext/1269018932/program.o \
	${OBJECTDIR}/_ext/346905599/main.o \
	${OBJECTDIR}/_ext/2056730713/I2C_Accelerometer.o \
	${OBJECTDIR}/_ext/731939294/I2C_TemplateLibrary.o \
	${OBJECTDIR}/_ext/1269018932/WHardwareTimer.o \
	${OBJECTDIR}/_ext/1693551763/Firmata.o \
	${OBJECTDIR}/_ext/582721243/NewSoftSerial.o \
	${OBJECTDIR}/_ext/1086598594/UdpServer.o \
	${OBJECTDIR}/_ext/1801028251/WFDriverRaw.o \
	${OBJECTDIR}/_ext/99335511/SPI.o \
	${OBJECTDIR}/_ext/1269018932/WDigital.o \
	${OBJECTDIR}/_ext/383632880/SoftwareSerial.o \
	${OBJECTDIR}/_ext/1269018932/WConstantTypes.o \
	${OBJECTDIR}/_ext/1569669406/pins_arduino.o \
	${OBJECTDIR}/_ext/2037745403/GUI.o \
	${OBJECTDIR}/_ext/1801028251/StackTsk.o \
	${OBJECTDIR}/_ext/285383713/HardwareSerial.o \
	${OBJECTDIR}/_ext/890755859/SPI.o \
	${OBJECTDIR}/_ext/1137634907/I2C_Humidity.o \
	${OBJECTDIR}/_ext/1111583014/File.o \
	${OBJECTDIR}/_ext/1129379939/EEPROM.o \
	${OBJECTDIR}/_ext/686635027/I2C_Pressure.o \
	${OBJECTDIR}/_ext/1823880294/Udp.o \
	${OBJECTDIR}/_ext/135288884/SD.o \
	${OBJECTDIR}/_ext/1801028251/WFEventHandler.o \
	${OBJECTDIR}/_ext/1195823272/Transport.o \
	${OBJECTDIR}/_ext/1760848399/WMemory.o \
	${OBJECTDIR}/_ext/1590063084/TimedAction.o \
	${OBJECTDIR}/_ext/33302971/Firmata.o \
	${OBJECTDIR}/_ext/1801028251/WFScan.o \
	${OBJECTDIR}/_ext/2037745403/Graphics.o \
	${OBJECTDIR}/_ext/1086598594/DWIFIcK.o \
	${OBJECTDIR}/_ext/1579878775/Firmata.o \
	${OBJECTDIR}/_ext/1569669406/wiring.o \
	${OBJECTDIR}/_ext/481273448/Graphics.o \
	${OBJECTDIR}/_ext/285383713/wiring.o \
	${OBJECTDIR}/_ext/346905599/crtn.o \
	${OBJECTDIR}/_ext/1269018932/WDelay.o \
	${OBJECTDIR}/_ext/1760848399/WString.o \
	${OBJECTDIR}/_ext/346905599/pins_arduino.o \
	${OBJECTDIR}/_ext/1086598594/TcpClient.o \
	${OBJECTDIR}/_ext/691741071/WPWM.o \
	${OBJECTDIR}/_ext/1423700061/I2C_Gyroscope.o \
	${OBJECTDIR}/_ext/691741071/WHardwareSerial.o \
	${OBJECTDIR}/_ext/1569669406/WString.o \
	${OBJECTDIR}/_ext/1195823272/Console.o \
	${OBJECTDIR}/_ext/1269018932/WHardwareSerial.o \
	${OBJECTDIR}/_ext/537124095/LiquidCrystal.o \
	${OBJECTDIR}/_ext/169008908/Ethernet.o \
	${OBJECTDIR}/_ext/557593157/I2C_Potentiometer.o \
	${OBJECTDIR}/_ext/55187095/twi.o \
	${OBJECTDIR}/_ext/1570935624/Sprite.o \
	${OBJECTDIR}/_ext/360590444/DateStrings.o \
	${OBJECTDIR}/_ext/581063589/twi.o \
	${OBJECTDIR}/_ext/1569669406/wiring_digital.o \
	${OBJECTDIR}/_ext/1801028251/WF_Eint.o \
	${OBJECTDIR}/_ext/1195823272/SerialTransport.o \
	${OBJECTDIR}/_ext/346905599/HardwareSerial_cdcacm.o \
	${OBJECTDIR}/_ext/1111583014/SD.o \
	${OBJECTDIR}/_ext/1801028251/IP.o \
	${OBJECTDIR}/_ext/1414948095/SoftSPI.o \
	${OBJECTDIR}/_ext/616247466/LED.o \
	${OBJECTDIR}/_ext/691741071/WHardwareTimer.o \
	${OBJECTDIR}/_ext/491004673/ArduinoTestSuite.o \
	${OBJECTDIR}/_ext/691741071/WInterrupts.o \
	${OBJECTDIR}/_ext/15516293/NewSoftSerial.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/embednetbeansproject

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/embednetbeansproject: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/embednetbeansproject ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/_ext/1801028251/DNETcKAPI.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/DNETcKAPI.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/DNETcKAPI.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/DNETcKAPI.c

${OBJECTDIR}/_ext/682039346/TimeAlarms.o: /Users/OlS/Documents/Arduino/Libraries/Time/TimeAlarms/TimeAlarms.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/682039346
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/682039346/TimeAlarms.o /Users/OlS/Documents/Arduino/Libraries/Time/TimeAlarms/TimeAlarms.cpp

${OBJECTDIR}/_ext/1823880294/Server.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/Server.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1823880294
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1823880294/Server.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/Server.cpp

${OBJECTDIR}/_ext/285383713/new.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/new.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/new.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/new.cpp

${OBJECTDIR}/_ext/691741071/WDelay.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WDelay.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WDelay.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WDelay.c

${OBJECTDIR}/_ext/1937105257/w5100.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/utility/w5100.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1937105257
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1937105257/w5100.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/utility/w5100.cpp

${OBJECTDIR}/_ext/1569669406/WMath.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/WMath.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/WMath.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/WMath.cpp

${OBJECTDIR}/_ext/2045159674/Matrix.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Matrix/Matrix.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2045159674
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2045159674/Matrix.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Matrix/Matrix.cpp

${OBJECTDIR}/_ext/616250782/OSC.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/OSC/OSC.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/616250782
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/616250782/OSC.o /Applications/Wiring.app/Contents/Resources/Java/libraries/OSC/OSC.cpp

${OBJECTDIR}/_ext/691741071/WAnalog.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WAnalog.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WAnalog.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WAnalog.c

${OBJECTDIR}/_ext/1801028251/WFConnectionManager.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFConnectionManager.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFConnectionManager.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFConnectionManager.c

${OBJECTDIR}/_ext/1318557376/Servo.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Servo/Servo.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1318557376
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1318557376/Servo.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Servo/Servo.cpp

${OBJECTDIR}/_ext/1788166970/Wire.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/libraries/Wire/Wire.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1788166970
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1788166970/Wire.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/libraries/Wire/Wire.cpp

${OBJECTDIR}/_ext/821730051/Encoder.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Encoder/Encoder.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/821730051
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/821730051/Encoder.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Encoder/Encoder.cpp

${OBJECTDIR}/_ext/285383713/Stream.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/Stream.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/Stream.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/Stream.cpp

${OBJECTDIR}/_ext/1195823272/GBEthernetClient.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBEthernetClient.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/GBEthernetClient.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBEthernetClient.cpp

${OBJECTDIR}/_ext/1385375812/Matrix.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Matrix/Matrix.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1385375812
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1385375812/Matrix.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Matrix/Matrix.cpp

${OBJECTDIR}/_ext/101011447/SPI.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SPI/SPI.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/101011447
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/101011447/SPI.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SPI/SPI.cpp

${OBJECTDIR}/_ext/1195823272/GBIMAC.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBIMAC.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/GBIMAC.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBIMAC.cpp

${OBJECTDIR}/_ext/1597197376/Deeprom.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/EEPROM/utility/Deeprom.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1597197376
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1597197376/Deeprom.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/EEPROM/utility/Deeprom.c

${OBJECTDIR}/_ext/318936080/I2C_Ambient.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Ambient/I2C_Ambient.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/318936080
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/318936080/I2C_Ambient.o /Users/OlS/Documents/Arduino/Libraries/I2C_Ambient/I2C_Ambient.cpp

${OBJECTDIR}/_ext/481273448/GUI.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Archive/GUI.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/481273448
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/481273448/GUI.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Archive/GUI.cpp

${OBJECTDIR}/_ext/1801028251/WFConnectionProfile.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFConnectionProfile.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFConnectionProfile.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFConnectionProfile.c

${OBJECTDIR}/_ext/2037745403/Gallery.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Gallery.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2037745403
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2037745403/Gallery.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Gallery.cpp

${OBJECTDIR}/_ext/1195823272/GB4DLcdDriver.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GB4DLcdDriver.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/GB4DLcdDriver.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GB4DLcdDriver.cpp

${OBJECTDIR}/_ext/360590444/Time.o: /Users/OlS/Documents/Arduino/Libraries/Time/Time/Time.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/360590444
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/360590444/Time.o /Users/OlS/Documents/Arduino/Libraries/Time/Time/Time.cpp

${OBJECTDIR}/_ext/1801028251/WFInit.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFInit.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFInit.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFInit.c

${OBJECTDIR}/_ext/346905599/wiring_analog.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring_analog.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/wiring_analog.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring_analog.c

${OBJECTDIR}/_ext/1801028251/WFMac.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFMac.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFMac.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFMac.c

${OBJECTDIR}/_ext/920020366/Messenger.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/Messenger/Messenger.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/920020366
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/920020366/Messenger.o /Applications/Wiring.app/Contents/Resources/Java/libraries/Messenger/Messenger.cpp

${OBJECTDIR}/_ext/691741071/WDigital.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WDigital.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WDigital.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WDigital.c

${OBJECTDIR}/_ext/1801028251/WF_Spi.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WF_Spi.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WF_Spi.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WF_Spi.c

${OBJECTDIR}/_ext/1195823272/GBEthernetServer.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBEthernetServer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/GBEthernetServer.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBEthernetServer.cpp

${OBJECTDIR}/_ext/547757580/I2C_Magnetometer.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Magnetometer/I2C_Magnetometer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/547757580
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/547757580/I2C_Magnetometer.o /Users/OlS/Documents/Arduino/Libraries/I2C_Magnetometer/I2C_Magnetometer.cpp

${OBJECTDIR}/_ext/1195823272/GBStatus.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBStatus.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/GBStatus.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBStatus.cpp

${OBJECTDIR}/_ext/1843772414/Wire.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Wire/Wire.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1843772414
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1843772414/Wire.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Wire/Wire.cpp

${OBJECTDIR}/_ext/1803262782/I2C_Thermometer.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Thermometer/I2C_Thermometer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1803262782
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1803262782/I2C_Thermometer.o /Users/OlS/Documents/Arduino/Libraries/I2C_Thermometer/I2C_Thermometer.cpp

${OBJECTDIR}/_ext/1370567834/I2C_Clock.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Clock/I2C_Clock.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1370567834
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1370567834/I2C_Clock.o /Users/OlS/Documents/Arduino/Libraries/I2C_Clock/I2C_Clock.cpp

${OBJECTDIR}/_ext/2008123095/I2C_IMU.o: /Users/OlS/Documents/Arduino/Libraries/I2C_IMU/I2C_IMU.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2008123095
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2008123095/I2C_IMU.o /Users/OlS/Documents/Arduino/Libraries/I2C_IMU/I2C_IMU.cpp

${OBJECTDIR}/_ext/1801028251/DNS.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/DNS.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/DNS.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/DNS.c

${OBJECTDIR}/_ext/1801028251/DHCP.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/DHCP.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/DHCP.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/DHCP.c

${OBJECTDIR}/_ext/1871859325/SdFile.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/utility/SdFile.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1871859325
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1871859325/SdFile.o /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/utility/SdFile.cpp

${OBJECTDIR}/_ext/913766812/I2C_Serial.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Serial/I2C_Serial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/913766812
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/913766812/I2C_Serial.o /Users/OlS/Documents/Arduino/Libraries/I2C_Serial/I2C_Serial.cpp

${OBJECTDIR}/_ext/346905599/WMath.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/WMath.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/WMath.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/WMath.cpp

${OBJECTDIR}/_ext/1871859325/Sd2Card.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/utility/Sd2Card.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1871859325
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1871859325/Sd2Card.o /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/utility/Sd2Card.cpp

${OBJECTDIR}/_ext/1801028251/Reboot.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/Reboot.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/Reboot.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/Reboot.c

${OBJECTDIR}/_ext/1801028251/TCP.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/TCP.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/TCP.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/TCP.c

${OBJECTDIR}/_ext/346905599/wiring_shift.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring_shift.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/wiring_shift.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring_shift.c

${OBJECTDIR}/_ext/2006777925/LiquidCrystal.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/LiquidCrystal/LiquidCrystal.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2006777925
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2006777925/LiquidCrystal.o /Applications/Arduino.app/Contents/Resources/Java/libraries/LiquidCrystal/LiquidCrystal.cpp

${OBJECTDIR}/_ext/346905599/cpp-startup.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/cpp-startup.S 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	$(AS) $(ASFLAGS) -g -o ${OBJECTDIR}/_ext/346905599/cpp-startup.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/cpp-startup.S

${OBJECTDIR}/_ext/1569669406/main.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/main.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/main.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/main.cpp

${OBJECTDIR}/_ext/1011032836/Stepper.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Stepper/Stepper.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1011032836
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1011032836/Stepper.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Stepper/Stepper.cpp

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/_ext/1801028251/SNTP.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/SNTP.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/SNTP.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/SNTP.c

${OBJECTDIR}/_ext/1307181659/I2C_Barometer.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Barometer/I2C_Barometer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1307181659
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1307181659/I2C_Barometer.o /Users/OlS/Documents/Arduino/Libraries/I2C_Barometer/I2C_Barometer.cpp

${OBJECTDIR}/_ext/169008908/Client.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/Client.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/169008908
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/169008908/Client.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/Client.cpp

${OBJECTDIR}/_ext/765940945/I2C_RGB_LED.o: /Users/OlS/Documents/Arduino/Libraries/I2C_RGB_LED/I2C_RGB_LED.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/765940945
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/765940945/I2C_RGB_LED.o /Users/OlS/Documents/Arduino/Libraries/I2C_RGB_LED/I2C_RGB_LED.cpp

${OBJECTDIR}/_ext/1086598594/TcpServer.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/TcpServer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1086598594
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1086598594/TcpServer.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/TcpServer.cpp

${OBJECTDIR}/_ext/1158258697/w5100.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/utility/w5100.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1158258697
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1158258697/w5100.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/utility/w5100.cpp

${OBJECTDIR}/_ext/82939850/Matrix.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Matrix/Matrix.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/82939850
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/82939850/Matrix.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Matrix/Matrix.cpp

${OBJECTDIR}/_ext/1801028251/ARP.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/ARP.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/ARP.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/ARP.c

${OBJECTDIR}/_ext/1482968431/Sd2Card.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/utility/Sd2Card.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1482968431
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1482968431/Sd2Card.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/utility/Sd2Card.cpp

${OBJECTDIR}/_ext/1434022269/I2C_Climate.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Climate/I2C_Climate.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1434022269
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1434022269/I2C_Climate.o /Users/OlS/Documents/Arduino/Libraries/I2C_Climate/I2C_Climate.cpp

${OBJECTDIR}/_ext/346905599/crti.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/crti.S 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	$(AS) $(ASFLAGS) -g -o ${OBJECTDIR}/_ext/346905599/crti.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/crti.S

${OBJECTDIR}/_ext/169008908/Server.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/Server.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/169008908
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/169008908/Server.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/Server.cpp

${OBJECTDIR}/_ext/1269018932/main.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/main.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/main.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/main.cpp

${OBJECTDIR}/_ext/1086598594/DNETcK.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/DNETcK.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1086598594
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1086598594/DNETcK.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/DNETcK.cpp

${OBJECTDIR}/_ext/1269018932/WPWM.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WPWM.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WPWM.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WPWM.cpp

${OBJECTDIR}/_ext/1801028251/ICMP.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/ICMP.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/ICMP.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/ICMP.c

${OBJECTDIR}/_ext/1569669406/Print.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/Print.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/Print.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/Print.cpp

${OBJECTDIR}/_ext/346905599/Tone.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/Tone.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/Tone.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/Tone.cpp

${OBJECTDIR}/_ext/285383713/WMath.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/WMath.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/WMath.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/WMath.cpp

${OBJECTDIR}/_ext/481273448/Serial_LCD.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Archive/Serial_LCD.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/481273448
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/481273448/Serial_LCD.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Archive/Serial_LCD.cpp

${OBJECTDIR}/_ext/1195823272/WizFi210.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/WizFi210.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/WizFi210.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/WizFi210.cpp

${OBJECTDIR}/_ext/2122273751/I2C_IMU2.o: /Users/OlS/Documents/Arduino/Libraries/I2C_IMU2/I2C_IMU2.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2122273751
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2122273751/I2C_IMU2.o /Users/OlS/Documents/Arduino/Libraries/I2C_IMU2/I2C_IMU2.cpp

${OBJECTDIR}/LocalLibrary.o: LocalLibrary.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/LocalLibrary.o LocalLibrary.cpp

${OBJECTDIR}/_ext/208455599/nmea.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/libraries/NMEA/nmea.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/208455599
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/208455599/nmea.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/libraries/NMEA/nmea.cpp

${OBJECTDIR}/_ext/1569669406/wiring_pulse.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring_pulse.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/wiring_pulse.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring_pulse.c

${OBJECTDIR}/_ext/117181477/twi.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Wire/utility/twi.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/117181477
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/117181477/twi.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Wire/utility/twi.c

${OBJECTDIR}/_ext/1935777686/DS1307RTC.o: /Users/OlS/Documents/Arduino/Libraries/Time/DS1307RTC/DS1307RTC.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1935777686
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1935777686/DS1307RTC.o /Users/OlS/Documents/Arduino/Libraries/Time/DS1307RTC/DS1307RTC.cpp

${OBJECTDIR}/_ext/135288884/File.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/File.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/135288884
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/135288884/File.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/File.cpp

${OBJECTDIR}/_ext/2129506280/OneWire.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/OneWire/OneWire.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2129506280
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2129506280/OneWire.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/OneWire/OneWire.cpp

${OBJECTDIR}/_ext/1195823272/SC16SpiTransport.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/SC16SpiTransport.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/SC16SpiTransport.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/SC16SpiTransport.cpp

${OBJECTDIR}/_ext/1760848399/WShift.o: /Applications/Wiring.app/Contents/Resources/Java/cores/Common/WShift.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1760848399
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1760848399/WShift.o /Applications/Wiring.app/Contents/Resources/Java/cores/Common/WShift.cpp

${OBJECTDIR}/_ext/346905599/wiring.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/wiring.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring.c

${OBJECTDIR}/_ext/1167644829/I2C_Height_IOs.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Height_IOs/I2C_Height_IOs.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1167644829
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1167644829/I2C_Height_IOs.o /Users/OlS/Documents/Arduino/Libraries/I2C_Height_IOs/I2C_Height_IOs.cpp

${OBJECTDIR}/_ext/1801028251/WFConnectionAlgorithm.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFConnectionAlgorithm.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFConnectionAlgorithm.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFConnectionAlgorithm.c

${OBJECTDIR}/_ext/691741071/program.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/program.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/program.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/program.cpp

${OBJECTDIR}/_ext/1482968431/SdFile.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/utility/SdFile.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1482968431
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1482968431/SdFile.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/utility/SdFile.cpp

${OBJECTDIR}/_ext/285383713/wiring_analog.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/wiring_analog.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/wiring_analog.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/wiring_analog.c

${OBJECTDIR}/_ext/1801028251/NBNS.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/NBNS.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/NBNS.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/NBNS.c

${OBJECTDIR}/_ext/346905599/WString.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/WString.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/WString.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/WString.cpp

${OBJECTDIR}/_ext/691741071/WTone.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WTone.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WTone.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WTone.cpp

${OBJECTDIR}/_ext/1823880294/Client.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/Client.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1823880294
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1823880294/Client.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/Client.cpp

${OBJECTDIR}/_ext/2037749790/Serial_GPS.o: /Users/OlS/Documents/Arduino/Libraries/Serial_GPS/Serial_GPS.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2037749790
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2037749790/Serial_GPS.o /Users/OlS/Documents/Arduino/Libraries/Serial_GPS/Serial_GPS.cpp

${OBJECTDIR}/_ext/1167918412/SoftwareSerial.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/SoftwareSerial/SoftwareSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1167918412
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1167918412/SoftwareSerial.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/SoftwareSerial/SoftwareSerial.cpp

${OBJECTDIR}/_ext/462070156/I2C_Clock2.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Clock2/I2C_Clock2.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/462070156
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/462070156/I2C_Clock2.o /Users/OlS/Documents/Arduino/Libraries/I2C_Clock2/I2C_Clock2.cpp

${OBJECTDIR}/_ext/285383713/WString.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/WString.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/WString.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/WString.cpp

${OBJECTDIR}/_ext/1801028251/WFDataTxRx.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFDataTxRx.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFDataTxRx.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFDataTxRx.c

${OBJECTDIR}/_ext/1569669406/WInterrupts.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/WInterrupts.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/WInterrupts.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/WInterrupts.c

${OBJECTDIR}/_ext/169308162/SoftwareSerial.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SoftwareSerial/SoftwareSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/169308162
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/169308162/SoftwareSerial.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SoftwareSerial/SoftwareSerial.cpp

${OBJECTDIR}/_ext/1269018932/WTone.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WTone.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WTone.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WTone.cpp

${OBJECTDIR}/_ext/1937105257/socket.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/utility/socket.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1937105257
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1937105257/socket.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/utility/socket.cpp

${OBJECTDIR}/_ext/904039052/MatrixMath.o: /Users/OlS/Documents/Arduino/Libraries/MatrixMath/MatrixMath.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/904039052
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/904039052/MatrixMath.o /Users/OlS/Documents/Arduino/Libraries/MatrixMath/MatrixMath.cpp

${OBJECTDIR}/_ext/1801028251/WFPowerSave.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFPowerSave.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFPowerSave.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFPowerSave.c

${OBJECTDIR}/_ext/1801028251/WFMgmtMsg.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFMgmtMsg.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFMgmtMsg.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFMgmtMsg.c

${OBJECTDIR}/_ext/1697618995/Button.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/Button/Button.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1697618995
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1697618995/Button.o /Applications/Wiring.app/Contents/Resources/Java/libraries/Button/Button.cpp

${OBJECTDIR}/_ext/481273448/proxySerial.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Archive/proxySerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/481273448
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/481273448/proxySerial.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Archive/proxySerial.cpp

${OBJECTDIR}/_ext/1111878700/MenuBackend.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/MenuBackend/MenuBackend.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1111878700
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1111878700/MenuBackend.o /Applications/Wiring.app/Contents/Resources/Java/libraries/MenuBackend/MenuBackend.cpp

${OBJECTDIR}/_ext/1801028251/WFDriverCom.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFDriverCom.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFDriverCom.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFDriverCom.c

${OBJECTDIR}/_ext/1801028251/UDP.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/UDP.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/UDP.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/UDP.c

${OBJECTDIR}/_ext/1888968704/SoftPWMServo.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SoftPWMServo/SoftPWMServo.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1888968704
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1888968704/SoftPWMServo.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SoftPWMServo/SoftPWMServo.cpp

${OBJECTDIR}/_ext/102619962/Sprite.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Sprite/Sprite.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/102619962
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/102619962/Sprite.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Sprite/Sprite.cpp

${OBJECTDIR}/_ext/346905599/wiring_pulse.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring_pulse.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/wiring_pulse.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring_pulse.c

${OBJECTDIR}/_ext/1923869628/nmea.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/NMEA/nmea.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1923869628
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1923869628/nmea.o /Applications/Wiring.app/Contents/Resources/Java/libraries/NMEA/nmea.cpp

${OBJECTDIR}/_ext/1086598594/UdpClient.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/UdpClient.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1086598594
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1086598594/UdpClient.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/UdpClient.cpp

${OBJECTDIR}/_ext/422011384/pic32_RTC.o: /Users/OlS/Documents/Arduino/Libraries/pic32_RTC/pic32_RTC.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/422011384
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/422011384/pic32_RTC.o /Users/OlS/Documents/Arduino/Libraries/pic32_RTC/pic32_RTC.cpp

${OBJECTDIR}/_ext/1789163801/EEPROM.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/EEPROM/EEPROM.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1789163801
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1789163801/EEPROM.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/EEPROM/EEPROM.cpp

${OBJECTDIR}/_ext/346905599/WInterrupts.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/WInterrupts.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/WInterrupts.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/WInterrupts.c

${OBJECTDIR}/_ext/169008908/Udp.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/Udp.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/169008908
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/169008908/Udp.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/Udp.cpp

${OBJECTDIR}/_ext/1760848399/WMath.o: /Applications/Wiring.app/Contents/Resources/Java/cores/Common/WMath.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1760848399
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1760848399/WMath.o /Applications/Wiring.app/Contents/Resources/Java/cores/Common/WMath.cpp

${OBJECTDIR}/_ext/346905599/exceptions.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/exceptions.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/exceptions.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/exceptions.c

${OBJECTDIR}/_ext/1569669406/HardwareSerial.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/HardwareSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/HardwareSerial.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/HardwareSerial.cpp

${OBJECTDIR}/_ext/1163168747/DSPI.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/DSPI/DSPI.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1163168747
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1163168747/DSPI.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/DSPI/DSPI.cpp

${OBJECTDIR}/_ext/355312364/NewSoftSerialTest.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/NewSoftSerial/Examples/NewSoftSerialTest/NewSoftSerialTest.pde 
	${MKDIR} -p ${OBJECTDIR}/_ext/355312364
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/355312364/NewSoftSerialTest.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/NewSoftSerial/Examples/NewSoftSerialTest/NewSoftSerialTest.pde

${OBJECTDIR}/_ext/346905599/wiring_digital.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring_digital.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/wiring_digital.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/wiring_digital.c

${OBJECTDIR}/_ext/338935723/EEPROM.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/EEPROM/EEPROM.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/338935723
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/338935723/EEPROM.o /Applications/Arduino.app/Contents/Resources/Java/libraries/EEPROM/EEPROM.cpp

${OBJECTDIR}/_ext/1569669406/wiring_analog.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring_analog.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/wiring_analog.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring_analog.c

${OBJECTDIR}/_ext/1048325648/Stepper.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/Stepper/Stepper.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1048325648
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1048325648/Stepper.o /Applications/Wiring.app/Contents/Resources/Java/libraries/Stepper/Stepper.cpp

${OBJECTDIR}/_ext/346905599/HardwareSerial_usb.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/HardwareSerial_usb.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/HardwareSerial_usb.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/HardwareSerial_usb.c

${OBJECTDIR}/_ext/1801028251/Delay.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/Delay.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/Delay.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/Delay.c

${OBJECTDIR}/_ext/1569669406/Tone.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/Tone.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/Tone.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/Tone.cpp

${OBJECTDIR}/_ext/1559931568/TwoNSSTest.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/NewSoftSerial/Examples/TwoNSSTest/TwoNSSTest.pde 
	${MKDIR} -p ${OBJECTDIR}/_ext/1559931568
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1559931568/TwoNSSTest.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/NewSoftSerial/Examples/TwoNSSTest/TwoNSSTest.pde

${OBJECTDIR}/_ext/691741071/WPulse.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WPulse.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WPulse.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WPulse.cpp

${OBJECTDIR}/_ext/1320345559/I2C_Stepper.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Stepper/I2C_Stepper.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1320345559
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1320345559/I2C_Stepper.o /Users/OlS/Documents/Arduino/Libraries/I2C_Stepper/I2C_Stepper.cpp

${OBJECTDIR}/_ext/972814340/Servo.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Servo/Servo.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/972814340
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/972814340/Servo.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Servo/Servo.cpp

${OBJECTDIR}/_ext/1823880294/Ethernet.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/Ethernet.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1823880294
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1823880294/Ethernet.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/Ethernet.cpp

${OBJECTDIR}/_ext/691741071/main.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/main.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/main.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/main.cpp

${OBJECTDIR}/_ext/1269018932/WPulse.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WPulse.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WPulse.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WPulse.cpp

${OBJECTDIR}/_ext/1163757000/Wire.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Wire/Wire.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1163757000
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1163757000/Wire.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Wire/Wire.cpp

${OBJECTDIR}/_ext/1269018932/WAnalog.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WAnalog.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WAnalog.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WAnalog.c

${OBJECTDIR}/_ext/285383713/wiring_digital.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/wiring_digital.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/wiring_digital.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/wiring_digital.c

${OBJECTDIR}/_ext/1442948484/I2C_RGBC_Reader.o: /Users/OlS/Documents/Arduino/Libraries/I2C_RGBC_Reader/I2C_RGBC_Reader.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1442948484
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1442948484/I2C_RGBC_Reader.o /Users/OlS/Documents/Arduino/Libraries/I2C_RGBC_Reader/I2C_RGBC_Reader.cpp

${OBJECTDIR}/_ext/1760848399/Print.o: /Applications/Wiring.app/Contents/Resources/Java/cores/Common/Print.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1760848399
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1760848399/Print.o /Applications/Wiring.app/Contents/Resources/Java/cores/Common/Print.cpp

${OBJECTDIR}/_ext/698302099/int.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Servo/utility/int.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/698302099
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/698302099/int.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Servo/utility/int.c

${OBJECTDIR}/_ext/1569669406/wiring_shift.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring_shift.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/wiring_shift.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring_shift.c

${OBJECTDIR}/_ext/366363529/LiquidCrystal.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/LiquidCrystal/LiquidCrystal.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/366363529
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/366363529/LiquidCrystal.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/LiquidCrystal/LiquidCrystal.cpp

${OBJECTDIR}/_ext/1269018932/WInterrupts.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WInterrupts.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WInterrupts.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WInterrupts.c

${OBJECTDIR}/_ext/1801028251/Tick.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/Tick.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/Tick.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/Tick.c

${OBJECTDIR}/embedExample.o: embedExample.pde 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/embedExample.o embedExample.pde

${OBJECTDIR}/_ext/285383713/main.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/main.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/main.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/main.cpp

${OBJECTDIR}/_ext/1649275740/Password.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/Password/Password.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1649275740
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1649275740/Password.o /Applications/Wiring.app/Contents/Resources/Java/libraries/Password/Password.cpp

${OBJECTDIR}/_ext/346905599/Print.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/Print.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/Print.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/Print.cpp

${OBJECTDIR}/_ext/1523693162/I2C_Compass.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Compass/I2C_Compass.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1523693162
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1523693162/I2C_Compass.o /Users/OlS/Documents/Arduino/Libraries/I2C_Compass/I2C_Compass.cpp

${OBJECTDIR}/_ext/1801028251/WFParamMsg.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFParamMsg.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFParamMsg.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFParamMsg.c

${OBJECTDIR}/_ext/1195823272/GBEthernet.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBEthernet.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/GBEthernet.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/GBEthernet.cpp

${OBJECTDIR}/_ext/1769378116/TimerSerial.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/libraries/TimerSerial/TimerSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1769378116
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1769378116/TimerSerial.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/libraries/TimerSerial/TimerSerial.cpp

${OBJECTDIR}/_ext/1801028251/Helpers.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/Helpers.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/Helpers.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/Helpers.c

${OBJECTDIR}/_ext/1851770985/twi.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/libraries/Wire/utility/twi.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1851770985
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1851770985/twi.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/libraries/Wire/utility/twi.c

${OBJECTDIR}/_ext/18730292/Potentiometer.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/Potentiometer/Potentiometer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/18730292
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/18730292/Potentiometer.o /Applications/Wiring.app/Contents/Resources/Java/libraries/Potentiometer/Potentiometer.cpp

${OBJECTDIR}/_ext/616242143/FiniteStateMachine.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/FSM/FiniteStateMachine.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/616242143
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/616242143/FiniteStateMachine.o /Applications/Wiring.app/Contents/Resources/Java/libraries/FSM/FiniteStateMachine.cpp

${OBJECTDIR}/_ext/1871859325/SdVolume.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/utility/SdVolume.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1871859325
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1871859325/SdVolume.o /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/utility/SdVolume.cpp

${OBJECTDIR}/_ext/2122985726/I2C_20x4.o: /Users/OlS/Documents/Arduino/Libraries/I2C_20x4/I2C_20x4.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2122985726
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2122985726/I2C_20x4.o /Users/OlS/Documents/Arduino/Libraries/I2C_20x4/I2C_20x4.cpp

${OBJECTDIR}/_ext/285383713/Print.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/Print.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/Print.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/Print.cpp

${OBJECTDIR}/_ext/346905599/HardwareSerial.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/HardwareSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/HardwareSerial.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/HardwareSerial.cpp

${OBJECTDIR}/_ext/285383713/WInterrupts.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/WInterrupts.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/WInterrupts.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/WInterrupts.c

${OBJECTDIR}/_ext/1195823272/SpiTransport.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/SpiTransport.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/SpiTransport.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/SpiTransport.cpp

${OBJECTDIR}/_ext/1557079726/Stepper.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Stepper/Stepper.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1557079726
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1557079726/Stepper.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Stepper/Stepper.cpp

${OBJECTDIR}/_ext/1940649685/Keypad.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/Keypad/Keypad.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1940649685
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1940649685/Keypad.o /Applications/Wiring.app/Contents/Resources/Java/libraries/Keypad/Keypad.cpp

${OBJECTDIR}/_ext/1215421894/Wire.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Wire/Wire.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1215421894
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1215421894/Wire.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Wire/Wire.cpp

${OBJECTDIR}/_ext/2115340346/Sprite.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/Sprite/Sprite.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2115340346
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2115340346/Sprite.o /Applications/Wiring.app/Contents/Resources/Java/libraries/Sprite/Sprite.cpp

${OBJECTDIR}/_ext/2037745403/proxySerial.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/proxySerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2037745403
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2037745403/proxySerial.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/proxySerial.cpp

${OBJECTDIR}/_ext/1158258697/socket.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/utility/socket.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1158258697
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1158258697/socket.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/utility/socket.cpp

${OBJECTDIR}/_ext/1801028251/DWIFIcKAPI.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/DWIFIcKAPI.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/DWIFIcKAPI.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/DWIFIcKAPI.c

${OBJECTDIR}/_ext/2037745403/Serial_LCD.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Serial_LCD.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2037745403
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2037745403/Serial_LCD.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Serial_LCD.cpp

${OBJECTDIR}/_ext/1482968431/SdVolume.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/utility/SdVolume.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1482968431
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1482968431/SdVolume.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/utility/SdVolume.cpp

${OBJECTDIR}/_ext/1416585193/Supervisor.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/Supervisor/Supervisor.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1416585193
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1416585193/Supervisor.o /Applications/Wiring.app/Contents/Resources/Java/libraries/Supervisor/Supervisor.cpp

${OBJECTDIR}/_ext/1712916022/Servo.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Servo/Servo.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1712916022
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1712916022/Servo.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Servo/Servo.cpp

${OBJECTDIR}/_ext/1269018932/program.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/program.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/program.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/program.cpp

${OBJECTDIR}/_ext/346905599/main.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/main.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/main.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/main.cpp

${OBJECTDIR}/_ext/2056730713/I2C_Accelerometer.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Accelerometer/I2C_Accelerometer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2056730713
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2056730713/I2C_Accelerometer.o /Users/OlS/Documents/Arduino/Libraries/I2C_Accelerometer/I2C_Accelerometer.cpp

${OBJECTDIR}/_ext/731939294/I2C_TemplateLibrary.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Template/I2C_TemplateLibrary.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/731939294
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/731939294/I2C_TemplateLibrary.o /Users/OlS/Documents/Arduino/Libraries/I2C_Template/I2C_TemplateLibrary.cpp

${OBJECTDIR}/_ext/1269018932/WHardwareTimer.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WHardwareTimer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WHardwareTimer.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WHardwareTimer.cpp

${OBJECTDIR}/_ext/1693551763/Firmata.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Firmata/Firmata.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1693551763
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1693551763/Firmata.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Firmata/Firmata.cpp

${OBJECTDIR}/_ext/582721243/NewSoftSerial.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/NewSoftSerial/NewSoftSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/582721243
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/582721243/NewSoftSerial.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/NewSoftSerial/NewSoftSerial.cpp

${OBJECTDIR}/_ext/1086598594/UdpServer.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/UdpServer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1086598594
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1086598594/UdpServer.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/UdpServer.cpp

${OBJECTDIR}/_ext/1801028251/WFDriverRaw.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFDriverRaw.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFDriverRaw.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFDriverRaw.c

${OBJECTDIR}/_ext/99335511/SPI.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/SPI/SPI.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/99335511
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/99335511/SPI.o /Applications/Arduino.app/Contents/Resources/Java/libraries/SPI/SPI.cpp

${OBJECTDIR}/_ext/1269018932/WDigital.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WDigital.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WDigital.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WDigital.c

${OBJECTDIR}/_ext/383632880/SoftwareSerial.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/SoftwareSerial/SoftwareSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/383632880
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/383632880/SoftwareSerial.o /Applications/Arduino.app/Contents/Resources/Java/libraries/SoftwareSerial/SoftwareSerial.cpp

${OBJECTDIR}/_ext/1269018932/WConstantTypes.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WConstantTypes.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WConstantTypes.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WConstantTypes.cpp

${OBJECTDIR}/_ext/1569669406/pins_arduino.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/pins_arduino.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/pins_arduino.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/pins_arduino.c

${OBJECTDIR}/_ext/2037745403/GUI.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/GUI.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2037745403
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2037745403/GUI.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/GUI.cpp

${OBJECTDIR}/_ext/1801028251/StackTsk.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/StackTsk.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/StackTsk.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/StackTsk.c

${OBJECTDIR}/_ext/285383713/HardwareSerial.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/HardwareSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/HardwareSerial.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/HardwareSerial.cpp

${OBJECTDIR}/_ext/890755859/SPI.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/SPI/SPI.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/890755859
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/890755859/SPI.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/SPI/SPI.cpp

${OBJECTDIR}/_ext/1137634907/I2C_Humidity.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Humidity/I2C_Humidity.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1137634907
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1137634907/I2C_Humidity.o /Users/OlS/Documents/Arduino/Libraries/I2C_Humidity/I2C_Humidity.cpp

${OBJECTDIR}/_ext/1111583014/File.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/File.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1111583014
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1111583014/File.o /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/File.cpp

${OBJECTDIR}/_ext/1129379939/EEPROM.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/EEPROM/EEPROM.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1129379939
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1129379939/EEPROM.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/EEPROM/EEPROM.cpp

${OBJECTDIR}/_ext/686635027/I2C_Pressure.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Pressure/I2C_Pressure.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/686635027
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/686635027/I2C_Pressure.o /Users/OlS/Documents/Arduino/Libraries/I2C_Pressure/I2C_Pressure.cpp

${OBJECTDIR}/_ext/1823880294/Udp.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/Udp.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1823880294
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1823880294/Udp.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Ethernet/Udp.cpp

${OBJECTDIR}/_ext/135288884/SD.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/SD.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/135288884
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/135288884/SD.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SD/SD.cpp

${OBJECTDIR}/_ext/1801028251/WFEventHandler.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFEventHandler.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFEventHandler.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFEventHandler.c

${OBJECTDIR}/_ext/1195823272/Transport.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/Transport.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/Transport.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/Transport.cpp

${OBJECTDIR}/_ext/1760848399/WMemory.o: /Applications/Wiring.app/Contents/Resources/Java/cores/Common/WMemory.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1760848399
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1760848399/WMemory.o /Applications/Wiring.app/Contents/Resources/Java/cores/Common/WMemory.cpp

${OBJECTDIR}/_ext/1590063084/TimedAction.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/TimedAction/TimedAction.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1590063084
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1590063084/TimedAction.o /Applications/Wiring.app/Contents/Resources/Java/libraries/TimedAction/TimedAction.cpp

${OBJECTDIR}/_ext/33302971/Firmata.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Firmata/Firmata.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/33302971
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/33302971/Firmata.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Firmata/Firmata.cpp

${OBJECTDIR}/_ext/1801028251/WFScan.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFScan.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WFScan.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WFScan.c

${OBJECTDIR}/_ext/2037745403/Graphics.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Graphics.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2037745403
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2037745403/Graphics.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Graphics.cpp

${OBJECTDIR}/_ext/1086598594/DWIFIcK.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/DWIFIcK.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1086598594
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1086598594/DWIFIcK.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/DWIFIcK.cpp

${OBJECTDIR}/_ext/1579878775/Firmata.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Firmata/Firmata.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1579878775
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1579878775/Firmata.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Firmata/Firmata.cpp

${OBJECTDIR}/_ext/1569669406/wiring.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/wiring.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring.c

${OBJECTDIR}/_ext/481273448/Graphics.o: /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Archive/Graphics.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/481273448
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/481273448/Graphics.o /Users/OlS/Documents/Arduino/Libraries/Serial_LCD/Archive/Graphics.cpp

${OBJECTDIR}/_ext/285383713/wiring.o: /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/wiring.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/285383713
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/285383713/wiring.o /Applications/Energia.app/Contents/Resources/Java/hardware/msp430/cores/msp430/wiring.c

${OBJECTDIR}/_ext/346905599/crtn.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/crtn.S 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	$(AS) $(ASFLAGS) -g -o ${OBJECTDIR}/_ext/346905599/crtn.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/crtn.S

${OBJECTDIR}/_ext/1269018932/WDelay.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WDelay.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WDelay.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WDelay.c

${OBJECTDIR}/_ext/1760848399/WString.o: /Applications/Wiring.app/Contents/Resources/Java/cores/Common/WString.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1760848399
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1760848399/WString.o /Applications/Wiring.app/Contents/Resources/Java/cores/Common/WString.cpp

${OBJECTDIR}/_ext/346905599/pins_arduino.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/pins_arduino.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/pins_arduino.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/pins_arduino.c

${OBJECTDIR}/_ext/1086598594/TcpClient.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/TcpClient.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1086598594
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1086598594/TcpClient.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/TcpClient.cpp

${OBJECTDIR}/_ext/691741071/WPWM.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WPWM.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WPWM.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WPWM.cpp

${OBJECTDIR}/_ext/1423700061/I2C_Gyroscope.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Gyroscope/I2C_Gyroscope.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1423700061
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1423700061/I2C_Gyroscope.o /Users/OlS/Documents/Arduino/Libraries/I2C_Gyroscope/I2C_Gyroscope.cpp

${OBJECTDIR}/_ext/691741071/WHardwareSerial.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WHardwareSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WHardwareSerial.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WHardwareSerial.cpp

${OBJECTDIR}/_ext/1569669406/WString.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/WString.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/WString.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/WString.cpp

${OBJECTDIR}/_ext/1195823272/Console.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/Console.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/Console.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/Console.cpp

${OBJECTDIR}/_ext/1269018932/WHardwareSerial.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WHardwareSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1269018932
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1269018932/WHardwareSerial.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/WHardwareSerial.cpp

${OBJECTDIR}/_ext/537124095/LiquidCrystal.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/LiquidCrystal/LiquidCrystal.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/537124095
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/537124095/LiquidCrystal.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/LiquidCrystal/LiquidCrystal.cpp

${OBJECTDIR}/_ext/169008908/Ethernet.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/Ethernet.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/169008908
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/169008908/Ethernet.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Ethernet/Ethernet.cpp

${OBJECTDIR}/_ext/557593157/I2C_Potentiometer.o: /Users/OlS/Documents/Arduino/Libraries/I2C_Potentiometer/I2C_Potentiometer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/557593157
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/557593157/I2C_Potentiometer.o /Users/OlS/Documents/Arduino/Libraries/I2C_Potentiometer/I2C_Potentiometer.cpp

${OBJECTDIR}/_ext/55187095/twi.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/Wire/utility/twi.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/55187095
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/55187095/twi.o /Applications/Arduino.app/Contents/Resources/Java/libraries/Wire/utility/twi.c

${OBJECTDIR}/_ext/1570935624/Sprite.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Sprite/Sprite.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1570935624
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1570935624/Sprite.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/Sprite/Sprite.cpp

${OBJECTDIR}/_ext/360590444/DateStrings.o: /Users/OlS/Documents/Arduino/Libraries/Time/Time/DateStrings.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/360590444
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/360590444/DateStrings.o /Users/OlS/Documents/Arduino/Libraries/Time/Time/DateStrings.cpp

${OBJECTDIR}/_ext/581063589/twi.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Wire/utility/twi.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/581063589
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/581063589/twi.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR8Bit/libraries/Wire/utility/twi.c

${OBJECTDIR}/_ext/1569669406/wiring_digital.o: /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring_digital.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1569669406
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1569669406/wiring_digital.o /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring_digital.c

${OBJECTDIR}/_ext/1801028251/WF_Eint.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WF_Eint.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/WF_Eint.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/WF_Eint.c

${OBJECTDIR}/_ext/1195823272/SerialTransport.o: /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/SerialTransport.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1195823272
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1195823272/SerialTransport.o /Users/OlS/Documents/Arduino/Libraries/GB_Libraries/SerialTransport.cpp

${OBJECTDIR}/_ext/346905599/HardwareSerial_cdcacm.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/HardwareSerial_cdcacm.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/346905599
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/346905599/HardwareSerial_cdcacm.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/cores/pic32/HardwareSerial_cdcacm.c

${OBJECTDIR}/_ext/1111583014/SD.o: /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/SD.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1111583014
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1111583014/SD.o /Applications/Arduino.app/Contents/Resources/Java/libraries/SD/SD.cpp

${OBJECTDIR}/_ext/1801028251/IP.o: /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/IP.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1801028251
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1801028251/IP.o /Users/OlS/Documents/Arduino/Libraries/DWIFIcK/utility/IP.c

${OBJECTDIR}/_ext/1414948095/SoftSPI.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SoftSPI/SoftSPI.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1414948095
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1414948095/SoftSPI.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/SoftSPI/SoftSPI.cpp

${OBJECTDIR}/_ext/616247466/LED.o: /Applications/Wiring.app/Contents/Resources/Java/libraries/LED/LED.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/616247466
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/616247466/LED.o /Applications/Wiring.app/Contents/Resources/Java/libraries/LED/LED.cpp

${OBJECTDIR}/_ext/691741071/WHardwareTimer.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WHardwareTimer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WHardwareTimer.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WHardwareTimer.cpp

${OBJECTDIR}/_ext/491004673/ArduinoTestSuite.o: /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/ArduinoTestSuite/ArduinoTestSuite.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/491004673
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/491004673/ArduinoTestSuite.o /Applications/Mpide.app/Contents/Resources/Java/hardware/pic32/libraries/ArduinoTestSuite/ArduinoTestSuite.cpp

${OBJECTDIR}/_ext/691741071/WInterrupts.o: /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WInterrupts.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/691741071
	${RM} $@.d
	$(COMPILE.c) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/691741071/WInterrupts.o /Applications/Wiring.app/Contents/Resources/Java/cores/AVR16Bit/WInterrupts.c

${OBJECTDIR}/_ext/15516293/NewSoftSerial.o: /Users/OlS/Documents/Arduino/Libraries/NewSoftSerial/NewSoftSerial.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/15516293
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/15516293/NewSoftSerial.o /Users/OlS/Documents/Arduino/Libraries/NewSoftSerial/NewSoftSerial.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/embednetbeansproject

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
