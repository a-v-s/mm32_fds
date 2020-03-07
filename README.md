mm32_fds_v2.0.16

"MM32 framework development suite" mm32_fds_v2.0.16 from 
http://www.mindmotion.com.cn/en/download.aspx?cid=2542

This is supposed to cover all of the MindMotion MCUs. Even though they have 
some per MCU downloads.


From that I've gathered,
these are the defines that need to be set for each of their MCUs.


| DEFINE    | CORE      | MCU                        |
| __MM3N1   | Cortex-M3 | MM32F103 / MM32L3xx        |
| __MM0N1   | Cortex-M0 | MM32F031x8, xB/MM32L0xx    |
| __MM0P1   | Cortex-M0 | MM32SPIN2x                 |
| __MM0Q1   | Cortex-M0 | MM32F003 / MM32F031x4,x6   |

Please note this library is untested, and will require some rework before it is 
usable with gcc since it is targetted to Keil and IAR. Also it seems some
peripherals, such as USB, are lacking drivers. 
