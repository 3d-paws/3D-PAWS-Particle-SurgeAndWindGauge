# SSAWG

StormSurgeAndWindGauge (SSAWG)

### DoAction Functions on Particle Cloud Console

On the view device screen on the Particle Console there is a FUNCTIONS area at the lower right. This is used to send commands to the online device.

>REBOOT - Reboot device. Toggle pin A0. If no relay/watchdog is connected to A0, then a soft boot on the Particle board is performed.

>5MDIST - Configure 5m Sensor. Creates file 5MDIST.TXT. Value read from pin A3 is multiplied by 1.25mm.

>10MDIST - Configure 10m Sensor. Removes file 5MDIST.TXT. Value read from pin A3 is multiplied by 2.5mm. (Default)
