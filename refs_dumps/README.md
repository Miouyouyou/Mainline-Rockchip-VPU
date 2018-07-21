About
-----

Using a modified RKMPP version, I dumped the 120 first frames of the
following H264 sample
[provided by Kodi](https://kodi.wiki/view/Samples#Codecs.2C_Framerates_and_Subtitles) :

https://drive.google.com/file/d/0BwxFVkl63-lERkJRU003ZTd2VEk/view

The archive contains :
* the 120 first frames;
* the registers content related to these frames, sent to the VPU driver;
* the analysis of each of these registers dump;

Now these are registers sent to the VPU driver, not the one sent to the
hardware itself, since the VPU driver performs a few modifications
before sending it to the hardware.

