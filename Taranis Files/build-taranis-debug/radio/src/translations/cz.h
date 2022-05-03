/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

// CZ translations author: Martin Hotar <mhotar@gmail.com>
// CZ translations author: Jan Urbanek @ rcstudio.cz

/*
 * !!!!! DO NOT EDIT cz.h - EDIT cz.h.txt INSTEAD !!!!!!!
 *
 * In order to make translations easier cz.h.txt is parsed and national
 * characters are replaced by bitmap codes. The result is cz.h.
 * See translate.py in the util folder for the list of character codes
 *
 * !!!!! DO NOT EDIT cz.h - EDIT cz.h.txt INSTEAD !!!!!!!
 */

/*
 * Formatting octal codes available in TR_ strings:
 *  \037\x           -sets LCD x-coord (x value in octal)
 *  \036             -newline
 *  \035             -horizontal tab (ARM only)
 *  \001 to \034     -extended spacing (value * FW/2)
 *  \0               -ends current string
 */

// NON ZERO TERMINATED STRINGS
#define LEN_OFFON                      "\003"
#define TR_OFFON                       "VYP""ZAP"

#define LEN_MMMINV                     "\003"
#define TR_MMMINV                      "---""INV"

#define LEN_VBEEPMODE                  TR("\005","\012")
#define TR_VBEEPMODE                   TR("Tich\212""Alarm""BezKl""V\207e\0", "Tich\212\0    ""Jen alarm\0""Bez kl\200ves""V\207e\0      ")

#define LEN_VBLMODE                    TR("\005", "\007")
#define TR_VBLMODE                     TR("Vyp\0 ""Kl\200v.""P\200ky\0""V\207e\0 ""Zap\0 ", "Vypnuto""Kl\200vesy""P\200ky\0  ""V\207e\0   ""Zapnuto")

#define LEN_TRNMODE                    "\003"
#define TR_TRNMODE                     " X "" +="" :="

#define LEN_TRNCHN                     "\003"
#define TR_TRNCHN                      "CH1CH2CH3CH4"

#define LEN_AUX_SERIAL_MODES           "\015"
#if defined(CLI) || defined(DEBUG)
#define TR_AUX_SERIAL_MODES            "Debug\0       ""Telem Mirror\0""Telemetry In\0""SBUS Tren\202r\0 ""LUA\0         "
#else
#define TR_AUX_SERIAL_MODES            "VYP\0         ""Telem Mirror\0""Telemetry In\0""SBUS Tren\202r\0 ""LUA\0         "
#endif

#define LEN_SWTYPES                    "\013"
#define TR_SWTYPES                     "\217\200dn\212\0     ""Bez aretace""2-polohov\212\0""3-polohov\212\0"

#define LEN_POTTYPES                   TR("\013","\016")
#define TR_POTTYPES                    TR("\217\200dn\212\0     ""Pot s aret.""V\204cepol p\206.""Pot\0       ", "\217\200dn\212\0        ""Pot s aretac\204\0""V\204cepol. p\206ep.""Potenciometr\0 ")

#define LEN_SLIDERTYPES                "\006"
#define TR_SLIDERTYPES                 "\217\200dn\212\0""Slider"

#define LEN_VLCD                       "\006"
#define TR_VLCD                        "NormalOptrex"

#define LEN_VPERSISTENT                "\014"
#define TR_VPERSISTENT                 "Ne\0         ""V r\200mci letu""Reset ru\201n\203\0"

#define LEN_COUNTRYCODES               TR("\002", "\010")
#define TR_COUNTRYCODES                TR("US""JP""EU", "Amerika\0""Japonsko""Evropa\0 ")

#if defined(RADIO_FAMILY_TBS)
#define LEN_USBMODES                   TR("\006", "\010")
#define TR_USBMODES                    TR("Ask\0  ""Joyst\0""Agent\0""SDCard""Serial", "Ask\0    ""Joystick""Agent\0 ""Storage\0""Serial\0 ")
#else
#define LEN_USBMODES                   TR("\007", "\011")
#if defined(DEBUG)
#define TR_USBMODES                    TR("Zeptat\0""Joyst\0 ""SDkarta""Serial\0", "Zeptat se""Joystick\0""\210lo\217i\207t\203\0""Serial\0  ")
#else
#define TR_USBMODES                    TR("Zeptat\0""Joyst\0 ""SDkarta""Telem\0 ", "Zeptat se""Joystick\0""\210lo\217i\207t\203\0""Telem\0   ")
#endif
#endif

#define LEN_JACKMODES                  "\007"
#define TR_JACKMODES                   "Zeptat\0""Audio\0 ""Tren\202r"

#define LEN_TELEMETRY_PROTOCOLS        "\017"
#define TR_TELEMETRY_PROTOCOLS         "FrSky S.PORT\0  FrSky D\0       FrSky D (kabel)Spektrum\0      "

#define TR_MULTI_CUSTOM                "Vlastn\204"

#define LEN_VTRIMINC                   "\007"
#define TR_VTRIMINC                    "Expo\0  ""ExJemn\212""Jemn\212\0 ""St\206edn\204""Hrub\212\0 "

#define LEN_VDISPLAYTRIMS              "\006"
#define TR_VDISPLAYTRIMS               "Ne\0   ""Zm\203na\0""Ano\0  "

#define LEN_VBEEPCOUNTDOWN             "\007"
#define TR_VBEEPCOUNTDOWN              "Ne\0    ""Zvuk\0  ""Hlas\0  ""Vibrace"

#define LEN_VVARIOCENTER               "\005"
#define TR_VVARIOCENTER                "T\205n\0 ""Ticho"

#define LEN_CURVE_TYPES                "\010"
#define TR_CURVE_TYPES                 "Rastr-X\0""Voln\200-XY"

#define LEN_RETA123                    "\001"

#if defined(PCBHORUS)
  #define TR_RETA123                   "SVPK13245LR"
#elif defined(PCBX9E)
  #define TR_RETA123                   "SVPK1234LRLR"
#elif defined(PCBTARANIS) || defined(REVX)
  #define TR_RETA123                   "SVPK123LR"
#elif defined(PCBSKY9X)
  #define TR_RETA123                   "SVPK123a"
#else
  #define TR_RETA123                   "SVPK123"
#endif

#if defined(PCBSKY9X) && defined(REVX)
  #define LEN_VOUTPUT_TYPE             "\011"
  #define TR_VOUTPUT_TYPE              "OpenDrain""PushPull\0"
#endif

#define LEN_VCURVEFUNC                 "\003"
#define TR_VCURVEFUNC                  "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX                     "\007"
#define TR_VMLTPX                      "Se\201\204st\0""N\200sobit""Zam\203nit"

#define LEN_VMLTPX2                    "\002"
#define TR_VMLTPX2                     "+=""*="":="

#define LEN_VMIXTRIMS                  "\004"

#if defined(PCBHORUS)
  #define TR_VMIXTRIMS                 "VYP\0""ZAP\0""Sm\203r""V\212\207k""Plyn""K\206id""T5\0""T6\0"
#else
  #define TR_VMIXTRIMS                 "VYP\0""ZAP\0""Sm\203r""V\212\207k""Plyn""K\206id"
#endif

#if LCD_W >= 212
  #define TR_CSWTIMER                  "Timer"
  #define TR_CSWSTICKY                 "Stcky"
  #define TR_CSWRANGE                  "Range"
  #define TR_CSWSTAY                   "Edge\0"
#else
  #define TR_CSWTIMER                  "Tim\0 "
  #define TR_CSWSTICKY                 "Stky\0"
    #define TR_CSWRANGE                "Rnge\0"
    #define TR_CSWSTAY                 "Edge\0"
#endif

  #define TR_CSWEQUAL                  "a=x\0 "

#define LEN_VCSWFUNC                   "\005"
#define TR_VCSWFUNC                    "---\0 " TR_CSWEQUAL "a\173x\0 ""a>x\0 ""a<x\0 " TR_CSWRANGE "|a|>x""|a|<x""AND\0 ""OR\0  ""XOR\0 " TR_CSWSTAY "a=b\0 ""a>b\0 ""a<b\0 ""\306}x\0 ""|\306|}x" TR_CSWTIMER TR_CSWSTICKY

#define LEN_VFSWFUNC                   "\013"

#if defined(VARIO)
  #define TR_VVARIO                    "Vario\0     "
#else
  #define TR_VVARIO                    "[Vario]\0   "
#endif

#if defined(AUDIO)
  #define TR_SOUND                     TR3("\221\222Zvuk\0    ", "\221\222Zvuk\0    ", "Hr\200t zvuk\0 ")
#else
  #define TR_SOUND                     "P\204pnout\0   "
#endif

#if defined(HAPTIC)
  #define TR_HAPTIC                    "Vibrovat\0  "
#else
  #define TR_HAPTIC                    "[Vibrovat]\0"
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK              "\221\222Zvuk\0    "
  #else
    #define TR_PLAY_TRACK              TR3("\221\222Stopa\0   ", "\221\222Stopa\0   ", "P\206ehr\200t wav")
  #endif
  #define TR_PLAY_BOTH                 "\221\222P\200r stop\0"
  #define TR_PLAY_VALUE                TR3("\221\222Hl\200sit\0  ", "\221\222Hl\200sit\0  ", "Hl\200sit stav")
#else
  #define TR_PLAY_TRACK                "[\221\222Stopa]\0 "
  #define TR_PLAY_BOTH                 "[\221\222P\200r]\0   "
  #define TR_PLAY_VALUE                "[\221\222Hl\200sit]\0"
#endif

#define TR_SF_BG_MUSIC                 TR3("\221\222Hudba\0   ""\221\222Hudba ||\0", "\221\222Hudba\0   ""\221\222Hudba ||\0", "Hudba\0     ""Hudba pauza")

#if defined(SDCARD)
  #define TR_SDCLOGS                   "Loguj na SD"
#else
  #define TR_SDCLOGS                   "[Logov\200n\204]\0"
#endif

#ifdef GVARS
  #define TR_ADJUST_GVAR               "Nastav\0    "
#else
  #define TR_ADJUST_GVAR               "[Nastav GP]"
#endif

#if defined(LUA)
  #define TR_SF_PLAY_SCRIPT            "Lua Skript\0"
#else
  #define TR_SF_PLAY_SCRIPT            "[Lua]\0     "
#endif

#if defined(DEBUG)
  #define TR_SF_TEST                   "Test\0      "
#else
  #define TR_SF_TEST
#endif

#if defined(OVERRIDE_CHANNEL_FUNCTION)
  #define TR_SF_SAFETY                 "Z\200mek \0    "
#else
  #define TR_SF_SAFETY                 "---\0       "
#endif

#define TR_SF_SCREENSHOT               "Sn\204mek LCD\0"
#define TR_SF_RACING_MODE              "RacingMode\0"
#define TR_SF_RESERVE                  "[rezerva]\0 "

#define TR_VFSWFUNC                    TR_SF_SAFETY "Tren\202r \0   ""Insta-Trim\0""Reset\0     ""Zm\203na \0    " TR_ADJUST_GVAR "Hlasitost\0 " "SetFailsafe" "RangeCheck\0" "ModuleBind\0" TR_SOUND TR_PLAY_TRACK TR_PLAY_VALUE TR_SF_RESERVE TR_SF_PLAY_SCRIPT TR_SF_RESERVE TR_SF_BG_MUSIC TR_VVARIO TR_HAPTIC TR_SDCLOGS "Podsv\203tlen\204" TR_SF_SCREENSHOT TR_SF_RACING_MODE TR_SF_TEST

#define LEN_VFSWRESET                  TR("\004", "\012")

#define TR_FSW_RESET_TELEM           TR("Telm", "Telemetrie")

#if LCD_W >= 212
  #define TR_FSW_RESET_TIMERS          "Stopky1\0  ""Stopky2\0  ""Stopky3\0  "
#else
  #define TR_FSW_RESET_TIMERS          "Tmr1""Tmr2""Tmr3"
#endif

#define TR_VFSWRESET                   TR(TR_FSW_RESET_TIMERS "V\207e\0" TR_FSW_RESET_TELEM, TR_FSW_RESET_TIMERS "V\207e\0      " TR_FSW_RESET_TELEM)

#define LEN_FUNCSOUNDS                 TR("\004", "\006")
#define TR_FUNCSOUNDS                  TR("Bp1\0""Bp2\0""Bp3\0""Wrn1""Wrn2""Chee""Rata""Tick""Sirn""Ring""SciF""Robt""Chrp""Tada""Crck""Alrm", "Beep1 ""Beep2 ""Beep3 ""Warn1 ""Warn2 ""Cheep ""Ratata""Tick  ""Siren ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""AlmClk")

#define LEN_VTELEMCHNS                 TR("\004", "\005")

#define LENGTH_UNIT_IMP                "ft\0"
#define SPEED_UNIT_IMP                 "mph"
#define LENGTH_UNIT_METR               "m\0 "
#define SPEED_UNIT_METR                "kmh"

#define LEN_VUNITSSYSTEM             TR("\006", "\010")
#define TR_VUNITSSYSTEM              TR("Metr.\0""Imper.", "Metrick\202""Imperial")
#define LEN_VTELEMUNIT               "\003"
#define TR_VTELEMUNIT                "-\0 ""V\0 ""A\0 ""mA\0""kts""m/s""f/s""kmh""mph""m\0 ""ft\0""@C\0""@F\0""%\0 ""mAh""W\0 ""mW\0""dB\0""rpm""g\0 ""@\0 ""rad""ml\0""fOz""mlm""Hz\0""mS\0""uS\0""km\0""dbm"

#define STR_V                          (STR_VTELEMUNIT+1)
#define STR_A                          (STR_VTELEMUNIT+4)

#define LEN_VTELEMSCREENTYPE         "\010"
#define TR_VTELEMSCREENTYPE          "Nic\0    ""Hodnota ""Ukazatel""Skript\0"

#define LEN_GPSFORMAT                  "\004"
#define TR_GPSFORMAT                   "DMS\0""NMEA"

#define LEN2_VTEMPLATES                12
#define LEN_VTEMPLATES                 "\014"
#define TR_TEMPLATE_CLEAR_MIXES        "Smazat mixy "
#define TR_TEMPLATE_SIMPLE_4CH         "Z\200kl. 4kan\200l"
#define TR_TEMPLATE_STICKY_TCUT        "Sticky-T-Cut"
#define TR_TEMPLATE_VTAIL              "V-Tail      "
#define TR_TEMPLATE_DELTA              "Elevon\\Delta"
#define TR_TEMPLATE_ECCPM              "eCCPM       "
#define TR_TEMPLATE_HELI               "Heli Setup  "
#define TR_TEMPLATE_SERVO_TEST         "Servo Test  "

#define LEN_VSWASHTYPE                 "\004"
#define TR_VSWASHTYPE                  "---\0""120\0""120X""140\0""90\0"

#if defined(PCBHORUS)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "PGUP\0""PGDN\0""ENTER""MDL\0 ""RTN\0 ""TELE\0""SYS\0 "
#elif defined(RADIO_FAMILY_JUMPER_T12)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Exit\0""Enter""Dol\211\0""Nhoru""Vprvo""Vlevo"
#elif defined(RADIO_TX12)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Exit\0""Enter""Up\0  ""Down\0""SYS\0 ""MDL\0 ""TELE\0"
#elif defined(RADIO_T8)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "RTN\0 ""ENTER""PGUP\0""PGDN\0""SYS\0 ""MDL\0 ""UP\0  ""DOWN\0"
#elif defined(PCBTARANIS)
#define LEN_VKEYS                      "\005"
#define TR_VKEYS                       "Menu\0""Exit\0""Enter""Page\0""Plus\0""M\204nus"
#else
#define LEN_VKEYS                      "\005"
#define TR_VKEYS                       "Menu\0""Exit\0""Dol\211\0""Nhoru""Vprvo""Vlevo"
#endif

#define LEN_VSWITCHES                  "\003"
#define LEN_VSRCRAW                    "\004"

#define LEN_INPUTNAMES                 "\004"
#define TR_INPUTNAMES                  "Sme\0""Vys\0""Pln\0""Kri\0"

#define TR_STICKS_VSRCRAW              "\307Sm\203""\307V\212\207""\307Pln""\307K\206i"

#if defined(PCBHORUS)
  #define TR_TRIMS_VSRCRAW             "\313Sm\203""\313V\212\207""\313Pln""\313K\206i""\313T5\0""\313T6\0"
#else
  #define TR_TRIMS_VSRCRAW             "\313Sm\203""\313V\212\207""\313Pln""\313K\206i"
#endif

#if defined(PCBHORUS)
  #define TR_TRIMS_SWITCHES            "\313Sl""\313Sp""\313Vd""\313Vn""\313Pd""\313Pn""\313Kl""\313Kp""\3135d""\3135n""\3136d""\3136n"
#else
  #define TR_TRIMS_SWITCHES            TR("tSl""tSp""tVd""tVn""tPd""tPn""tKl""tKp", "\313Sl""\313Sp""\313Vd""\313Vn""\313Pd""\313Pn""\313Kl""\313Kp")
#endif

#if defined(PCBSKY9X)
  #define TR_ROTARY_ENCODERS           "REa\0"
  #define TR_ROTENC_SWITCHES           "REa"
#else
  #define TR_ROTARY_ENCODERS
  #define TR_ROTENC_SWITCHES
#endif

#define TR_ON_ONE_SWITCHES             "ZAP""One"

#if defined(GYRO)
  #define TR_GYR_VSRCRAW               "GyrX""GyrY"
#else
  #define TR_GYR_VSRCRAW
#endif

#if defined(HELI)
  #define TR_CYC_VSRCRAW               "CYC1""CYC2""CYC3"
#else
  #define TR_CYC_VSRCRAW               "[C1]""[C2]""[C3]"
#endif

#define TR_RESERVE_VSRCRAW           "[--]"
#define TR_EXTRA_VSRCRAW             "Bat.""\201as\0""GPS\0" TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW "Tmr1""Tmr2""Tmr3"

#define LEN_VTMRMODES                  "\004"
#define TR_VTMRMODES                   "VYP\0""ZAP\0""Pln>""Pln%""Pln*"

#define LEN_VTRAINERMODES              "\022"
#define TR_VTRAINER_MASTER_JACK        "U\201itel/Jack\0      "
#define TR_VTRAINER_SLAVE_JACK         "\217\200k/Jack\0         "
#define TR_VTRAINER_MASTER_SBUS_MODULE "U\201itel/SBUS Modul\0"
#define TR_VTRAINER_MASTER_CPPM_MODULE "U\201itel/CPPM Modul\0"
#define TR_VTRAINER_MASTER_BATTERY     "U\201itel/Serial\0    "
#define TR_VTRAINER_BLUETOOTH          TR("Master/BT\0        ""Slave/BT\0         ", "Master/Bluetooth\0 ""Slave/Bluetooth\0  ")
#define TR_VTRAINER_MULTI              "Master/Multi\0     "
#define TR_VTRAINER_SPORT_SBUS         "Master/SBUS-Sport\0"

#define LEN_VFAILSAFE                  "\012"
#define TR_VFAILSAFE                   "Nenastaven""Dr\217et\0    ""Vlastn\204\0  ""Bez pulz\211\0""P\206ij\204ma\201\0"


#define LEN_VSENSORTYPES               "\011"
#define TR_VSENSORTYPES                "Vlastn\204\0 ""Vypo\201ten\212"

#define LEN_VFORMULAS                  "\012"
#define TR_VFORMULAS                   "Sou\201et\0   ""Pr\211m\203r\0   ""Min\0      ""Max\0      ""N\200soben\204  ""Totalize  ""\201l\200nek\0   ""Spot\206eba  ""Vzd\200lenost"

#define LEN_VPREC                      "\004"
#define TR_VPREC                       "X   ""X.X ""X.XX"

#define LEN_VCELLINDEX                 "\007"
#define TR_VCELLINDEX                  "N\204zk\212\0 ""1\0     ""2\0     ""3\0     ""4\0     ""5\0     ""6\0     ""Nejv\204ce""Delta\0"

#define LEN_GYROS                      "\004"
#define TR_GYROS                       "GyrX""GyrY"

#define LEN_TEXT_SIZE                  "\010"
#define TR_TEXT_SIZE                   "Standard""Tiny\0   ""Small\0  ""Mid\0    ""Double\0 "

// ZERO TERMINATED STRINGS
#if defined(COLORLCD)
  #define INDENT                       "\007"
  #define LEN_INDENT                   1
  #define INDENT_WIDTH                 12
  #define BREAKSPACE                   "\036"
#else
#define INDENT                         "\001"
#define LEN_INDENT                     1
#define INDENT_WIDTH                   (FW/2)
#define BREAKSPACE                     " "
#endif

#if defined(PCBTARANIS) || defined(PCBHORUS)
  #define TR_ENTER                     "[ENTER]"
#else
  #define TR_ENTER                     "[MENU]"
#endif

#if defined(PCBHORUS)
  #define TR_EXIT                      "[RTN]"
  #define TR_OK                        TR_ENTER
#else
  #define TR_EXIT                      "[EXIT]"
  #define TR_OK                        TR("\010" "\010" "\010" "[OK]", "\010" "\010" "\010" "\010" "\010" "[OK]")
#endif

#if defined(PCBTARANIS)
  #define TR_POPUPS_ENTER_EXIT         TR(TR_EXIT "\010" TR_ENTER, TR_EXIT "\010" "\010" "\010" "\010" TR_ENTER)

#else
  #define TR_POPUPS_ENTER_EXIT         TR_ENTER "\010" TR_EXIT
#endif

#define TR_MENUWHENDONE                CENTER "\011" TR_ENTER " > DAL\207\214"
#define TR_FREE                        TR("voln\202:", "voln\212ch")
#define TR_DELETEMODEL                 "SMAZAT" BREAKSPACE "MODEL"
#define TR_COPYINGMODEL                "Kop\204ruji model.."
#define TR_MOVINGMODEL                 "P\206esouv\200m model."
#define TR_LOADINGMODEL                "Aktivuji model.."
#define TR_NAME                        "N\200zev"
#define TR_MODELNAME                   TR("Model", "N\200zev modelu")
#define TR_PHASENAME                   "N\200zev"
#define TR_MIXNAME                     "N\200zev"
#define TR_INPUTNAME                   "N\200zev"
  #define TR_EXPONAME                  "Popis"
#define TR_BITMAP                      "Obr\200zek"
#define TR_TIMER                       "Stopky"
#define TR_ELIMITS                     TR("Limit +25%", "Kan\200l +/- 125%")
#define TR_ETRIMS                      TR("\207ir\207\204 Trim", "\207irok\212 trim")
#define TR_TRIMINC                     TR("Krok Trimu", "Krok trimu")
#define TR_DISPLAY_TRIMS               TR3("\201\204slo v Trm", "Zobr.hodnotu trimu", "\201\204slo v li\207t\203 trimu")
#define TR_TTRACE                      TR("StopaPlynu", INDENT "Stopa plynu")
#define TR_TTRIM                       TR3("TrimVolnob.", INDENT "Trim jen volnob\203h", "Trim jen pro volnob\203h")
#define TR_TTRIM_SW                    TR("T-Trim-Sw", INDENT "Trim switch")
#define TR_BEEPCTR                     TR3("St\206edy \221\222", "P\204pat st\206edy \221\222", "P\204pat st\206edy")
#define TR_USE_GLOBAL_FUNCS            TR("Glob.Funkce", "Pou\217\204t Glob.Funkce")
#if defined(PCBSKY9X) && defined(REVX)
  #define TR_OUTPUT_TYPE               INDENT "V\212stup"
#endif
#define TR_PROTOCOL                    "Protokol"
#define TR_PPMFRAME                    INDENT "PPM modulace"
#define TR_REFRESHRATE                 TR(INDENT "Obnovit", INDENT "Obn. frekv.")
#define STR_WARN_BATTVOLTAGE           TR(INDENT "V\212stup VBAT: ", INDENT "Varov\200n\204: v\212stupn\204 hodnota VBAT: ")
#define TR_WARN_5VOLTS                 "Warning: output level is 5 volts"
#define TR_MS                          "ms"
#define TR_FREQUENCY                   INDENT "Frequence"
#define TR_SWITCH                      "Sp\204na\201"
#define TR_TRIMS                       "Trimy"
#define TR_FADEIN                      "P\206echod Zap"
#define TR_FADEOUT                     "P\206echod Vyp"
#define TR_DEFAULT                     "(v\212choz\204)"
#define TR_CHECKTRIMS                  "\011Kont.\010Trimy"
#define OFS_CHECKTRIMS                 (9*FW)
#define TR_SWASHTYPE                   "Typ cykliky"
#define TR_COLLECTIVE                  "Kolektiv"
#define TR_AILERON                     "Bo\201n\204 cyklika"
#define TR_ELEVATOR                    TR3("Pod\202ln\200 cykl.", "Pod\202ln\200 cykl.", "Pod\202ln\200 cyklika")
#define TR_SWASHRING                   "Cyklika"
#define TR_ELEDIRECTION                "Sm\203r:V\212\207kovka"
#define TR_AILDIRECTION                "\012K\206id\202lka"
#define TR_COLDIRECTION                "\012Kolektiv"
#define TR_MODE                        "M\205d"
#define TR_SUBTYPE                     INDENT "Subtyp"
#define TR_NOFREEEXPO                  "Nen\204 voln\202 expo!"
#define TR_NOFREEMIXER                 "Nen\204 voln\212 mix!"
#define TR_SOURCE                      "Zdroj"
#define TR_WEIGHT                      "V\200ha"
#define TR_EXPO                        "Expo"
#define TR_SIDE                        "Strana"
#define TR_DIFFERENTIAL                "Dif.v\212chylek"
#define TR_OFFSET                      "Ofset"
#define TR_TRIM                        "Trim"
#define TR_DREX                        "DR/Expo"
#define DREX_CHBOX_OFFSET              45
#define TR_CURVE                       "K\206ivka"
#define TR_FLMODE                      "Re\217im"
#define TR_MIXWARNING                  "Varov\200n\204"
#define TR_OFF                         "VYP"
#define TR_ANTENNA                     "Antenna"
#define TR_NO_INFORMATION              TR("Nen\204 info.", "\217\200dn\200 informace")
#define TR_MULTPX                      TR("Mat.operace", "Operace")
#define TR_DELAYDOWN                   TR3("Zpo\217d\203n\204 Vyp", "Zdr\217et(x)", "Zpo\217d\203n\204 Vyp")
#define TR_DELAYUP                     TR3("Zpo\217d\203n\204 Zap", "Zdr\217et(\43)", "Zpo\217d\203n\204 Zap")
#define TR_SLOWDOWN                    TR3("Zpomalen\204(-)", "Zpomal(\177)", "Zpomalen\204(\177)")
#define TR_SLOWUP                      TR3("Zpomalen\204(+)", "Zpomal(\176)", "Zpomalen\204(\176)")
#define TR_MIXES                       "MIXER"
#define TR_CV                          "K"
#define TR_GV                          TR("G", "GP")
#define TR_ACHANNEL                    "A\004Kan\200l"
#define TR_RANGE                       INDENT"Rozsah"
#define TR_CENTER                      INDENT "St\206ed"
#define TR_BAR                         "Ukazatel"
#define TR_ALARM                       INDENT"Alarm"
#define TR_USRDATA                     TR("U\217ivData", "U\217iv. data")
#define TR_BLADES                      TR(INDENT"ListyVrt", INDENT"Listy vrtule")
#define TR_SCREEN                      "Panel "
#define TR_SOUND_LABEL                 "Zvuk"
#define TR_LENGTH                      INDENT"D\202lka"
#define TR_BEEP_LENGTH                 TR3(INDENT"D\202lka", INDENT"D\202lka", INDENT"D\202lka zvuku")
#define TR_SPKRPITCH                   INDENT"T\205n"
#define TR_HAPTIC_LABEL                "Vibrace"
#define TR_HAPTICSTRENGTH              INDENT"S\204la"
#define TR_GYRO_LABEL                  "Gyro"
#define TR_GYRO_OFFSET                 "Offset"
#define TR_GYRO_MAX                    "Max"
#define TR_CONTRAST                    "Kontrast LCD"
#define TR_ALARMS_LABEL                "Alarmy"
#define TR_BATTERY_RANGE               TR("Ukazatel bat.", "Ukazatel baterie")
#define TR_BATTERYWARNING              INDENT "Vybit\200 bat."
#define TR_INACTIVITYALARM             TR(INDENT "Ne\201innost", INDENT "Ne\201innost r\200dia")
#define TR_MEMORYWARNING               INDENT"Pln\200 pam\203t'"
#define TR_ALARMWARNING                TR3(INDENT "Vypnut\212 zvuk", INDENT "Upozornit na vypnut\212 zvuk", INDENT "Upozornit na vyp. zvuk")
#define TR_RSSISHUTDOWNALARM           TR(INDENT "Rssi p\206i vyp.", INDENT "Hl\204dat Rssi p\206i vypnut\204")
#define TR_MODEL_STILL_POWERED         "Model st\200le spu\207t\203n"
#define TR_MODEL_SHUTDOWN              "Vypnout ?"
#define TR_PRESS_ENTER_TO_CONFIRM      "Zm\200\201kni enter pro potvrzen\204"
#define TR_THROTTLE_LABEL              "Plyn"
#define TR_THROTTLEREVERSE             TR("ReversPlyn", INDENT"Revers plynu")
#define TR_MINUTEBEEP                  TR("Minuta", "Oznamovat minuty")
#define TR_BEEPCOUNTDOWN               INDENT"Odpo\201et"
#define TR_PERSISTENT                  INDENT"Trval\202"
#define TR_BACKLIGHT_LABEL             "Podsv\203tlen\204"
#define TR_GHOST_MENU_LABEL            "GHOST MENU"
#define TR_STATUS                      "Status"
#define TR_RAW_12BITS                  "Raw 12 bits"
#define TR_BLDELAY                     INDENT"Zhasnout po"
#define TR_BLONBRIGHTNESS              TR3(INDENT"Jas Zap.", INDENT"Jas Zap.", INDENT"Jas zapnut\202ho LCD")
#define TR_BLOFFBRIGHTNESS             TR3(INDENT"Jas Vyp.", INDENT"Jas Vyp.", INDENT"Jas vypnut\202ho LCD")
#define TR_KEYS_BACKLIGHT              "Keys backlight"
#define TR_BLCOLOR                     INDENT "Barva"
#define TR_SPLASHSCREEN                TR("\210vodn\204 logo", "Zobrazit \210vodn\204 logo")
#define TR_PWR_ON_DELAY                "Pwr On delay"
#define TR_PWR_OFF_DELAY               "Pwr Off delay"
#define TR_THROTTLEWARNING             TR("* Plyn", INDENT "Kontrola plynu")
#define TR_SWITCHWARNING               TR("* Sp\204na\201e", INDENT "Polohy sp\204na\201\211")
#define TR_POTWARNINGSTATE             TR("* Pot&Slid.", INDENT "Kontrola Pot&Slid.")
#define TR_SLIDERWARNING               TR("* Slidery", INDENT "Pozice slider\211")
#define TR_POTWARNING                  TR("* Potenc.", INDENT "Potenciometry")
#define TR_TIMEZONE                    "\201asov\202 p\200smo"
#define TR_ADJUST_RTC                  TR3("\201as z GPS", INDENT "Pou\217\204t \201as z GPS", INDENT "Pou\217\204t \201as z GPS")
#define TR_GPS                         "GPS"
#define TR_RXCHANNELORD                TR("Po\206ad\204 kan\200l\211", "V\212choz\204 po\206ad\204 kan\200l\211")
#define TR_STICKS                      "P\200ky"
#define TR_POTS                        "Potenciometry"
#define TR_SWITCHES                    "Sp\204na\201e"
#define TR_SWITCHES_DELAY              TR3("Filtr p\206ep\204na\201e", "Filtr p\206ep\204na\201e", "Filtr poloh p\206ep\204na\201e")
#define TR_SLAVE                       "Pod\206\204zen\212"
#define TR_MODESRC                     " M\205d\006% Zdroj"
#define TR_MULTIPLIER                  "N\200sobi\201"
#define TR_CAL                         "Kal."
#define TR_VTRIM                       "Trim- +"
#define TR_BG                          "Poz:"
#if defined(PCBHORUS)
  #define TR_MENUTOSTART               "Stickem [Enter] za\201neme"
  #define TR_SETMIDPOINT               "Vycentruj p\200ky/poty/slidery a stiskni [Enter]"
  #define TR_MOVESTICKSPOTS            "H\212bej p\200kami/poty/slidery pak stiskni [Enter]"
#elif defined(COLORLCD)
  #define TR_MENUTOSTART               TR_ENTER " = START"
  #define TR_SETMIDPOINT               "Nastav p\200ky na st\206ed"
  #define TR_MOVESTICKSPOTS            "H\212bej p\200kami/poty"
#else
  #define TR_MENUTOSTART               CENTER "\011" TR_ENTER " = START"
  #define TR_SETMIDPOINT               CENTER "\004Nastav p\200ky na st\206ed"
  #define TR_MOVESTICKSPOTS            TR(CENTER "\006H\212bej p\200kami/poty", "\014H\212bej p\200kami i potenciometry")
#endif
#define TR_RXBATT                      "Rx Bat.:"
#define TR_TXnRX                       "Tx:\0Rx:"
#define OFS_RX                         4
#define TR_ACCEL                       "Acc:"
#define TR_NODATA                      CENTER "NO DATA"
#define TR_US                          "us"
#define TR_TMIXMAXMS                   "Tmix max"
#define TR_FREE_STACK                  "Free stack"
#define TR_MENUTORESET                 TR_ENTER" >> Reset"
#define TR_PPM_TRAINER                 "TR"
#define TR_CH                          "CH"
#define TR_MODEL                       "MODEL"
#define TR_FM                          "LR"
#define TR_MIX                         "MIX"
#define TR_EEPROMLOWMEM                "Doch\200z\204 EEPROM"
#define TR_ALERT                       "\006   POZOR"
#define TR_PRESSANYKEYTOSKIP           TR("\003Kl\200vesa >>> p\206esko\201it", "Kl\200vesa >>> p\206esko\201it")
#define TR_THROTTLENOTIDLE             TR("\003P\200ka plynu je pohnut\200", "P\200ka plynu nen\204 na nule")
#define TR_ALARMSDISABLED              "Alarmy jsou zak\200z\200ny"
#define TR_PRESSANYKEY                 TR("\006Stiskni kl\200vesu", "Stiskni kl\200vesu")
#define TR_BADEEPROMDATA               TR("\006Chyba dat EEPROM", "Chyba dat EEPROM")
#define TR_BAD_RADIO_DATA              "Chybn\200 data r\200dia"
#define TR_EEPROMFORMATTING            TR("\004Formatov\200n\204 EEPROM", "Formatov\200n\204 EEPROM")
#define TR_STORAGE_FORMAT              "Storage Preparation"
#define TR_EEPROMOVERFLOW              "P\206etekla EEPROM"
#define TR_MENURADIOSETUP              "NASTAVEN\214 R\213DIA"
#define TR_MENUDATEANDTIME             "DATUM A \201AS"
#define TR_MENUTRAINER                 "TREN\220R"
#define TR_MENUSPECIALFUNCS            "GLOB\213LN\214 FUNKCE"
#define TR_MENUVERSION                 "VERZE"
#define TR_MENU_RADIO_SWITCHES         "DIAG"
#define TR_MENU_RADIO_ANALOGS          "ANALOGY"
#define TR_MENUCALIBRATION             "KALIBRACE"
#if defined(COLORLCD)
#define TR_TRIMS2OFFSETS               "Trimy => Subtrimy"
#else
#define TR_TRIMS2OFFSETS               "\006Trimy => Subtrimy"
#endif
#define TR_CHANNELS2FAILSAFE           "Kan\200ly=>Failsafe"
#define TR_CHANNEL2FAILSAFE            "Kan\200l=>Failsafe"
#define TR_MENUMODELSEL                "MODEL"
#define TR_MENUSETUP                   "NASTAVEN\214"
#define TR_MENUFLIGHTMODE              "LETOV\216 RE\217IM"
#define TR_MENUFLIGHTMODES             "LETOV\220 RE\217IMY"
#define TR_MENUHELISETUP               "HELI"

#define TR_MENULIMITS                  "SERVA"

  #define TR_MENUINPUTS                "VSTUPY"

#define TR_MENUCURVES                  "K\215IVKY"
#define TR_MENUCURVE                   "\002K"
#define TR_MENULOGICALSWITCH           "LOG. SP\214NA\201"
#define TR_MENULOGICALSWITCHES         "LOGICK\220 SP\214NA\201E"
#define TR_MENUCUSTOMFUNC              "SPECI\213LN\214 FUNKCE"
#define TR_MENUCUSTOMSCRIPTS           "SKRIPTY LUA"
#define TR_MENUTELEMETRY               "TELEMETRIE"
#define TR_MENUTEMPLATES               "\207ABLONY"
#define TR_MENUSTAT                    "STATISTIKA"
#define TR_MENUDEBUG                   "DIAG"
#define TR_MONITOR_SWITCHES            "MONITOR LOGICK\216CH SP\214NA\201\211"
#define TR_MONITOR_CHANNELS1           "MONITOR KAN\213L\211 1/8"
#define TR_MONITOR_CHANNELS2           "MONITOR KAN\213L\211 9/16"
#define TR_MONITOR_CHANNELS3           "MONITOR KAN\213L\211 17/24"
#define TR_MONITOR_CHANNELS4           "MONITOR KAN\213L\211 25/32"
#define TR_MONITOR_OUTPUT_DESC         "V\212stupy"
#define TR_MONITOR_MIXER_DESC          "Mixy"
#define TR_RECEIVER_NUM                TR("RX \201\204slo", "\201\204slo p\206ij\204ma\201e")
#define TR_RECEIVER                    "P\206ij\204ma\201"
#define TR_MULTI_RFTUNE                TR(INDENT "Lad\203n\204 frek", INDENT "Jemn\202 lad\203n\204 frek. RF")
#define TR_MULTI_RFPOWER               "RF power"
#define TR_MULTI_WBUS                  "Output"
#define TR_MULTI_TELEMETRY             "Telemetrie"
#define TR_MULTI_VIDFREQ               TR(INDENT "Freq. videa", INDENT "Frekvence videa")
#define TR_RFPOWER                     "V\212kon RF"
#define TR_MULTI_FIXEDID               TR("FixedID", "Fixed ID")
#define TR_MULTI_OPTION                TR(INDENT "Mo\217nosti", INDENT "Hodnota")
#define TR_MULTI_AUTOBIND              TR(INDENT "Bind Ch.",INDENT "Bind p\206i zapnut\204")
#define TR_DISABLE_CH_MAP              TR("No Ch. map", "Disable Ch. map")
#define TR_DISABLE_TELEM               TR("No Telem", "Disable Telemetry")
#define TR_MULTI_DSM_AUTODTECT         TR(INDENT "Autodetekce", INDENT "Form\200t autodetekce")
#define TR_MULTI_LOWPOWER              TR(INDENT "N\204zk\212 v\212kon", INDENT "Re\217im n\204zk\202ho v\212konu")
#define TR_MULTI_LNA_DISABLE            INDENT "LNA disable"
#define TR_MODULE_TELEMETRY            TR(INDENT "S.Port", INDENT "S.Port link")
#define TR_MODULE_TELEM_ON             TR("Zap", "Zapnuto")
#define TR_DISABLE_INTERNAL            TR("Vypnout int.", "Vypnout intern\204 RF")
#define TR_MODULE_NO_SERIAL_MODE       TR("Nes\202riov\212", "Nen\204 v s\202riov\202m re\217imu")
#define TR_MODULE_NO_INPUT             TR("\217\200dn\212 vstup", "\217\200dn\212 s\202riov\212 vstup")
#define TR_MODULE_NO_TELEMETRY         TR3("Bez telemetrie", "Bez MULTI_TELEMETIE", "Nedetekov\200na MULTI_TELEMETRIE")
#define TR_MODULE_WAITFORBIND          "P\200rovat p\206i zaveden\204 protokolu"
#define TR_MODULE_BINDING              TR("Bind...","P\200rov\200n\204")
#define TR_MODULE_UPGRADE_ALERT        TR3("Upg. needed", "Module upgrade required", "Module\036Upgrade required")
#define TR_MODULE_UPGRADE              TR("Upg. advised", "Module update recommended")
#define TR_PULSE_RATE                  "Pulse rate"
#define TR_LINK_SPEED                  "Link speed"
#define TR_REBIND                      "Nutno p\206ep\200rovat"
#define TR_REG_OK                      "Registrace ok"
#define TR_BIND_OK                     "\210sp\203\207n\202 p\200rovan\204"
#define TR_BINDING_CH1_8_TELEM_ON      "k1-8 telem zap"
#define TR_BINDING_CH1_8_TELEM_OFF     "k1-8 telem vyp"
#define TR_BINDING_CH9_16_TELEM_ON     "k9-16 telem zap"
#define TR_BINDING_CH9_16_TELEM_OFF    "k9-16 telem vyp"
#define TR_PROTOCOL_INVALID            TR("\207pat. protokol", "\207patn\212 protokol")
#define TR_MODULE_STATUS               TR(INDENT "Stav", INDENT "Stav modulu")
#define TR_MODULE_SYNC                 TR(INDENT "Sync", INDENT "Proto Sync Status")
#define TR_MULTI_SERVOFREQ             TR(INDENT "Frekv. serva", INDENT "Obnovovac\204 frequence serva")
#define TR_MULTI_MAX_THROW             TR("Max. Throw", "Enable max. throw")
#define TR_MULTI_RFCHAN                TR("RF Channel", "Select RF channel")
#define TR_SYNCMENU                    "[Sync]"
#define TR_LIMIT                       INDENT"Limit"
#define TR_MINRSSI                     "Min Rssi"
#define TR_LATITUDE                    "Zem. \207\204\206ka"
#define TR_LONGITUDE                   "Zem. d\202lka"
#define TR_GPSCOORD                    "GPS sou\206adnice"
#define TR_VARIO                       "Vario"
#define TR_PITCH_AT_ZERO               INDENT "T\205n na nule"
#define TR_PITCH_AT_MAX                INDENT "T\205n na maximu"
#define TR_REPEAT_AT_ZERO              TR(INDENT "Opak. na nule", INDENT "Opakov\200n\204 na nule")
#define TR_SHUTDOWN                    "Vyp\204n\200n\204.."
#define TR_SAVEMODEL                   "Ukl\200d\200m nastaven\204 modelu"
#define TR_BATT_CALIB                  "Kalib. bat."
#define TR_CURRENT_CALIB               " +=\006Proud"
#define TR_VOLTAGE                     INDENT"Nap\203t\204"
#define TR_CURRENT                     INDENT"Proud"
#define TR_SELECT_MODEL                "Vyber model"
#define TR_SELECT_MODE                 "Select mode"
#define TR_CREATE_CATEGORY             "Nov\200 kategorie"
#define TR_RENAME_CATEGORY             "P\206ejmenovat kategorii"
#define TR_DELETE_CATEGORY             "Odstranit kategorii"
#define TR_CREATE_MODEL                "Nov\212 model"
#define TR_DUPLICATE_MODEL             "Duplikovat model"
#define TR_COPY_MODEL                  "Kop\204rovat"
#define TR_MOVE_MODEL                  "P\206esunout model"
#define TR_BACKUP_MODEL                "Z\200lohovat na SD"
#define TR_DELETE_MODEL                "Sma\217 model"
#define TR_RESTORE_MODEL               "Obnov model z SD"
#define TR_DELETE_ERROR                "Nelze odstranit"
#define TR_CAT_NOT_EMPTY               "Kategorie nen\204 pr\200zdn\200"
#define TR_SDCARD_ERROR                "Chyba SD karty"
#define TR_NO_SDCARD                   "Nen\204 SD karta"
#define TR_WAITING_FOR_RX              "\201ek\200m na RX..."
#define TR_WAITING_FOR_TX              "\201ek\200m na TX..."
#define TR_WAITING_FOR_MODULE          TR("Waiting module", "Waiting for module...")
#define TR_NO_TOOLS                    TR("Nejsou k dispozici", "\217\200dn\202 n\200stroje k dispozici")
#define TR_NORMAL                      "Norm\200ln\204"
#define TR_NOT_INVERTED                "Neinvert"
#define TR_NOT_CONNECTED               "Nep\206ipojen"
#define TR_CONNECTED                   "P\206ipojen"
#define TR_FLEX_915                    "Flex 915MHz"
#define TR_FLEX_868                    "Flex 868MHz"
#define TR_16CH_WITHOUT_TELEMETRY      TR("16k bez telem.", "16k bez telemetrie")
#define TR_16CH_WITH_TELEMETRY         TR("16k s telem.", "16k s telemetri\204")
#define TR_8CH_WITH_TELEMETRY          TR("8k s telem.", "8k s telemetri\204")
#define TR_EXT_ANTENNA                 "Ext. ant\202na"
#define TR_PIN                         "Pin"
#define TR_UPDATE_RX_OPTIONS           "Ulo\217it nastaven\204?"
#define TR_UPDATE_TX_OPTIONS           "Ulo\217it nastaven\204?"
#define TR_MODULES_RX_VERSION          "Verze modulu a RX"
#define TR_MENU_MODULES_RX_VERSION     "VERZE MODULU A RX"
#define TR_MENU_FIRM_OPTIONS           "FIRMWARE OPTIONS"
#define TR_GYRO                        "Gyro"
#define TR_STICKS_POTS_SLIDERS         "p\200ky/pot./slidery"
#define TR_PWM_STICKS_POTS_SLIDERS     "PWM p\200ky/pot./slidery"
#define TR_RF_PROTOCOL                 "RF Protokol"
#define TR_MODULE_OPTIONS              "Mo\217nosti modulu"
#define TR_POWER                       "V\212kon"
#define TR_NO_TX_OPTIONS               "\217\200dn\202 mo\217nosti"
#define TR_RTC_BATT                    "RTC Bat"
#define TR_POWER_METER_EXT             "M\203\206i\201 v\212konu (EXT)"
#define TR_POWER_METER_INT             "M\203\206i\201 v\212konu (INT)"
#define TR_SPECTRUM_ANALYSER_EXT       "Spekt\200ln\204 an. (EXT)"
#define TR_SPECTRUM_ANALYSER_INT       "Spekt\200ln\204 an. (INT)"
#define TR_SDCARD_FULL                 "Pln\200 SD karta"
#define TR_NEEDS_FILE                  "NEEDS FILE"
#define TR_EXT_MULTI_SPEC              "opentx-inv"
#define TR_INT_MULTI_SPEC              "stm-opentx-noinv"
#define TR_INCOMPATIBLE                "Nekompatibiln\204"
#define TR_WARNING                     "KONTROLA"
#define TR_EEPROMWARN                  "EEPROM"
#define TR_STORAGE_WARNING             "\210LO\217I\207T\203"
#define TR_EEPROM_CONVERTING           "Aktualizuji EEPROM"
#define TR_THROTTLEWARN                "PLYNU"
#define TR_ALARMSWARN                  "ALARMU"
#define TR_SWITCHWARN                  "POZICE"
#define TR_FAILSAFEWARN                "FAILSAFE"
#define TR_TEST_WARNING                 TR("NO\201N\214", "NO\201N\214 SESTAVEN\214")
#define TR_TEST_NOTSAFE                 "Use for tests only"
#define TR_WRONG_SDCARDVERSION         "O\201ek\200van\200 ver.: "
#define TR_WARN_RTC_BATTERY_LOW        "Slab\200 RTC baterie"
#define TR_WARN_MULTI_LOWPOWER         "Low power mode"
#define TR_BATTERY                     "BATERIE"
#define TR_WRONG_PCBREV                "Jin\200 verze PCB/firmware"
#define TR_EMERGENCY_MODE              "NOUZOV\216 RE\217IM"
#define TR_PCBREV_ERROR                "Chyba PCB"
#define TR_NO_FAILSAFE                 "Failsafe nen\204 nastaveno"
#define TR_KEYSTUCK                    "Zasekl\200 kl\200vesa"
#define TR_INVERT_THR                  "Invertovat plyn?"
#define TR_SPEAKER_VOLUME              INDENT "Hlasitost"
#define TR_LCD                         "LCD"
#define TR_BRIGHTNESS                  INDENT "Jas"
#define TR_CPU_TEMP                    "Tepl. CPU\016>"
#define TR_CPU_CURRENT                 "Proud\030>"
#define TR_CPU_MAH                     "Spot\206eba"
#define TR_COPROC                      "CoProc."
#define TR_COPROC_TEMP                 "Tepl. MB \016>"
#define TR_CAPAWARNING                 INDENT "N\204zk\200 kapacita"
#define TR_TEMPWARNING                 INDENT "P\206eh\206\200t\204"
#define TR_FUNC                        "Fce."
#define TR_V1                          "V1"
#define TR_V2                          "V2"
#define TR_DURATION                    "Trv\200n\204"
#define TR_DELAY                       "Zdr\217et"
#define TR_SD_CARD                     "SD"
#define TR_SDHC_CARD                   "SD-HC"
#define TR_NO_SOUNDS_ON_SD             "\217\200dn\212 zvuk" BREAKSPACE "na SD"
#define TR_NO_MODELS_ON_SD             "\217\200dn\212 model" BREAKSPACE "na SD"
#define TR_NO_BITMAPS_ON_SD            "\217\200dn\202 obr\200zky" BREAKSPACE "na SD"
#define TR_NO_SCRIPTS_ON_SD            "\217\200dn\212 skript" BREAKSPACE "na SD"
#define TR_SCRIPT_SYNTAX_ERROR         TR("Syntaktick\200 chyba", "Syntaktick\200 chyba skriptu")
#define TR_SCRIPT_PANIC                "Script zmaten"
#define TR_SCRIPT_KILLED               "Script ukon\201en"
#define TR_SCRIPT_ERROR                "Nezn\200m\200 chyba"
#define TR_PLAY_FILE                   "P\206ehr\200t"
#define TR_DELETE_FILE                 "Odstranit"
#define TR_COPY_FILE                   "Kop\204rovat"
#define TR_RENAME_FILE                 "P\206ejmenovat"
#define TR_ASSIGN_BITMAP               "Zvolit obr\200zek"
#define TR_ASSIGN_SPLASH               "\210vodn\204 obrazovka"
#define TR_EXECUTE_FILE                "Spustit"
#define TR_REMOVED                     " odstran\203n"
#define TR_SD_INFO                     "Informace"
#define TR_SD_FORMAT                   "Form\200t"
#define TR_NA                          "[X]"
#define TR_HARDWARE                    "HARDWARE"
#define TR_FORMATTING                  "Form\200tovan\204.."
#define TR_TEMP_CALIB                  " +=\006Teplota"
#define TR_TIME                        "\201as"
#define TR_MAXBAUDRATE                 "Max baud\211"

#define TR_BLUETOOTH                   "Bluetooth"
#define TR_BLUETOOTH_DISC              "Hledat"
#define TR_BLUETOOTH_INIT              "Init"
#define TR_BLUETOOTH_DIST_ADDR         "Vzd\200l. addr"
#define TR_BLUETOOTH_LOCAL_ADDR        "Lok\200l. addr"
#define TR_BLUETOOTH_PIN_CODE          TR("PIN k\205d", "PIN k\205d")
#define TR_BAUDRATE                    "Baudrate BT"
#define LEN_BLUETOOTH_MODES            "\012"
#if defined(PCBX9E)
#define TR_BLUETOOTH_MODES             "---\0      ""Povoleno\0 "
#else
#define TR_BLUETOOTH_MODES             "---\0      ""Telemetrie""Tren\202r\0   "
#endif
#define TR_SD_INFO_TITLE               "SD INFO"
#define TR_SD_TYPE                     "Typ:"
#define TR_SD_SPEED                    "Rychlost:"
#define TR_SD_SECTORS                  "Sektor\211 :"
#define TR_SD_SIZE                     "Velikost:"
#define TR_TYPE                        INDENT TR_SD_TYPE
#define TR_GLOBAL_VARS                 "Glob\200ln\204 prom\203nn\202"
#define TR_GVARS                       "GLOB.PROM."
#define TR_GLOBAL_VAR                  "Glob\200ln\204 prom\203nn\200"
#define TR_MENUGLOBALVARS              "GLOB\213LN\214 PROM\203NN\220"
#define TR_OWN                         " \043 "
#define TR_DATE                        "Datum"
#define TR_MONTHS                      { "Led", "\210no", "B\206e", "Dub", "Kv\203", "\201vn", "\201vc", "Srp", "Z\200\206", "\215\204j", "Lis", "Pro" }
#define TR_ROTARY_ENCODER              "R.Enko"
#define TR_INVERT_ROTARY             "Invert Rotary"
#define TR_CHANNELS_MONITOR            "MONITOR KAN\213LU"
#define TR_MIXERS_MONITOR              "MONITOR MIXU"
#define TR_PATH_TOO_LONG               "Cesta je moc dlouh\200"
#define TR_VIEW_TEXT                   "Zobrazit text"
#define TR_FLASH_BOOTLOADER            "Flash bootloaderu"
#define TR_FLASH_EXTERNAL_DEVICE       TR("Flash S.Portem", "Flash S.Portem")
#define TR_FLASH_RECEIVER_OTA          "Flash p\206\204j\204ma\201e OTA"
#define TR_FLASH_RECEIVER_BY_EXTERNAL_MODULE_OTA "Flash RX by ext. OTA"
#define TR_FLASH_RECEIVER_BY_INTERNAL_MODULE_OTA "Flash RX by int. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_EXTERNAL_MODULE_OTA "Flash FC by ext. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_INTERNAL_MODULE_OTA "Flash FC by int. OTA"
#define TR_FLASH_BLUETOOTH_MODULE      TR("Flash modulu BT", "Flash modulu Bluetooth")
#define TR_FLASH_POWER_MANAGEMENT_UNIT TR("Flash \206\204zen\204 spot\206.", "Flash jednotky \206\204zen\204 spot\206eby")
#define TR_CURRENT_VERSION             TR("Sou\201asn\200 ver. ", "Sou\201asn\200 verze: ")
#define TR_HW_REV                      "HW Rev"
#define TR_FLASH_INTERNAL_MODULE       TR("Flash vnit\206. modulu", "Flash vnit\206n\204ho modulu")
#define TR_FLASH_INTERNAL_MULTI        TR("Flash Int. Multi", "Flash Internal Multi")
#define TR_FLASH_EXTERNAL_MODULE       TR("Flash ext. modulu", "Flash extern\204ho za\206\204zen\204")
#define TR_FLASH_EXTERNAL_MULTI        TR("Flash Ext. Multi", "Flash External Multi")
#define TR_FLASH_EXTERNAL_ELRS         TR("Flash Ext. ELRS", "Flash External ELRS")
#define TR_FIRMWARE_UPDATE_ERROR       TR("Chyba z\200pisu FW", "Chyba z\200pisu firmware")
#define TR_FIRMWARE_UPDATE_SUCCESS     "\210sp\203\207n\212 z\200pis FW"
#define TR_WRITING                     "Zapisuji.."
#define TR_CONFIRM_FORMAT              "Prov\202st Form\200t?"
#define TR_INTERNALRF                  "Intern\204 RF modul"
#define TR_INTERNAL_MODULE             "Intern\204 modul"
#define TR_EXTERNAL_MODULE             "Extern\204 modul"
#define TR_OPENTX_UPGRADE_REQUIRED     TR("Aktualizujte OpenTX", "Vy\217adov\200na aktualizace OpenTX")
#define TR_TELEMETRY_DISABLED          "Telem. zak\200z\200na"
#define TR_MORE_OPTIONS_AVAILABLE      TR("V\204ce mo\217nost\204", "V\204ce dostupn\212ch mo\217nost\204")
#define TR_NO_MODULE_INFORMATION       "\217\200dn\202 info. o modulu"
#define TR_EXTERNALRF                  "Extern\204 RF modul"
#define TR_FAILSAFE                    TR("Failsafe", "M\205d Failsafe")
#define TR_FAILSAFESET                 "NASTAVEN\214 FAILSAFE"
#define TR_REG_ID                      TR("Reg. ID", "Registra\201n\204 ID")
#define TR_OWNER_ID                    "ID vlastn\204ka"
#define TR_PINMAPSET                   "PINMAP"
#define TR_HOLD                        "Dr\217et"
#define TR_HOLD_UPPERCASE              "DR\217ET"
#define TR_NONE                        "Nic"
#define TR_NONE_UPPERCASE              "NIC"
#define TR_MENUSENSOR                  "SENZOR"
#define TR_POWERMETER_PEAK             "\207pi\201ka"
#define TR_POWERMETER_POWER            "V\212kon"
#define TR_POWERMETER_ATTN             "\210tlum"
#define TR_POWERMETER_FREQ             "Frek."
#define TR_MENUTOOLS                   "N\213STROJE"
#define TR_TURN_OFF_RECEIVER           "Vypn\203te p\206ij\204ma\201"
#define TR_STOPPING                    "Zastavuji..."
#define TR_MENU_SPECTRUM_ANALYSER      "SPEKTR\213LN\214 ANALYZER"
#define TR_MENU_POWER_METER            "M\203\215I\201 V\216KONU"
#define TR_SENSOR                      "SENZOR"
#define TR_COUNTRYCODE                 "K\205d regionu"
#define TR_USBMODE                     "Re\217im USB"
#define TR_JACKMODE                    "Re\217im Jack"
#define TR_VOICELANG                   "Jazyk hlasu"
#define TR_UNITSSYSTEM                 "Jednotky"
#define TR_EDIT                        "Upravit"
#define TR_INSERT_BEFORE               "Vlo\217it p\206ed"
#define TR_INSERT_AFTER                "Vlo\217it za"
#define TR_COPY                        "Kop\204rovat"
#define TR_MOVE                        "P\206esunout"
#define TR_PASTE                       "Vlo\217it"
#define TR_DELETE                      "Odstranit"
#define TR_INSERT                      "P\206idat"
#define TR_RESET_FLIGHT                "Let"
#define TR_RESET_TIMER1                "Stopky1"
#define TR_RESET_TIMER2                "Stopky2"
#define TR_RESET_TIMER3                "Stopky3"
#define TR_RESET_TELEMETRY             "Telemetrii"
#define TR_STATISTICS                  "Statistika"
#define TR_ABOUT_US                    "O n\200s"
#define TR_USB_JOYSTICK                "USB Joystick (HID)"
#define TR_USB_MASS_STORAGE            "USB Disk (SD)"
#define TR_USB_SERIAL                  "USB Serial (Debug)"
#define TR_USB_TELEMETRY               "USB Telem mirror"
#define TR_SETUP_SCREENS               "Obrazovky nastaven\204"
#define TR_MONITOR_SCREENS             "Monitory"
#define TR_AND_SWITCH                  "AND Sp\204na\201"
#define TR_SF                          "SF"
#define TR_GF                          "GF"
#define TR_SPEAKER                     INDENT"Repro"
#define TR_BUZZER                      INDENT"P\204p\200k"
#define TR_BYTES                       "[B]"
#define TR_MODULE_BIND                 BUTTON(TR("Bnd", "Bind"))
#define TR_POWERMETER_ATTN_NEEDED      "\210tlumov\212 \201len nutn\212!"
#define TR_PXX2_SELECT_RX              "Vyber RX..."
#define TR_PXX2_DEFAULT                "<v\212choz\204>"
#define TR_BT_SELECT_DEVICE            "Vyberte za\206\204zen\204"
#define TR_DISCOVER                    "Hledat"
#define TR_BUTTON_INIT                 BUTTON("Init")
#define TR_WAITING                     "\201ek\200m..."
#define TR_RECEIVER_DELETE             "Smazat p\206ij\204ma\201?"
#define TR_RECEIVER_RESET              "Resetovat p\206ij\204ma\201?"
#define TR_SHARE                       "Sd\204let"
#define TR_BIND                        "P\200rovat"
#define TR_REGISTER                    TR("Reg", "Registrovat")
#define TR_MODULE_RANGE                BUTTON(TR("Rng", "Range"))
#define TR_RECEIVER_OPTIONS            TR("MO\217NOSTI RX", "MO\217NOSTI P\215IJ\214MA\201E")
#define TR_DEL_BUTTON                  BUTTON(TR("Sma\217", "Smazat"))
#define TR_RESET_BTN                   BUTTON("Reset")
#define TR_DEBUG                       "Debug"
#define TR_KEYS_BTN                    BUTTON(TR("SW","P\206ep\204na\201e"))
#define TR_ANALOGS_BTN                 BUTTON(TR("Analog","Analogy"))
#define TR_TOUCH_NOTFOUND              "Touch hardware not found"
#define TR_TOUCH_EXIT                  "Touch screen to exit"
#define TR_CALIBRATION                 "Kalibrace"
#define TR_SET                         BUTTON("Nast")
#define TR_TRAINER                     "Tren\202r"
#define TR_CHANS                       "Chans"
#define TR_ANTENNAPROBLEM              CENTER "Probl\202m s TX ant\202nou!"
#define TR_MODELIDUSED                 TR("ID ji\217 pou\217ito","ID modelu je ji\217 pou\217ito")
#define TR_MODULE                      "Modul"
#define TR_RX_NAME                     "Jm\202no RX"
#define TR_TELEMETRY_TYPE              TR("Typ", "Typ telemetrie")
#define TR_TELEMETRY_SENSORS           "Senzory"
#define TR_VALUE                       "Hodnota"
#define TR_TOPLCDTIMER                 "Stopky horn\204ho LCD"
#define TR_UNIT                        "Jednotky"
#define TR_TELEMETRY_NEWSENSOR         INDENT "P\206idat senzor ru\201n\203"
#define TR_CHANNELRANGE                TR(INDENT "Kan\200ly", INDENT "Rozsah kan\200l\211")
#define TR_AFHDS3_RX_FREQ              TR("RX freq.", "RX frequency")
#define TR_AFHDS3_ONE_TO_ONE_TELEMETRY TR("Unicast/Tel.", "Unicast/Telemetry")
#define TR_AFHDS3_ONE_TO_MANY          "Multicast"
#define TR_AFHDS3_ACTUAL_POWER         TR("Act. pow", "Actual power")
#define TR_AFHDS3_POWER_SOURCE         TR("Power src.", TR("Power src.", "Power source"))
#define TR_ANTENNACONFIRM1             "Opravdu p\206epnout?"
#if defined(PCBX12S)
#define LEN_ANTENNA_MODES      "\023"
#define TR_ANTENNA_MODES       "Internal\0          ""Ask\0               ""Per model\0         ""Internal + External"
#else
#define LEN_ANTENNA_MODES      "\011"
#define TR_ANTENNA_MODES       "Internal\0""Ask\0     ""Per model""External"
#endif
#define TR_USE_INTERNAL_ANTENNA        TR("Use int. antenna", "Use internal antenna")
#define TR_USE_EXTERNAL_ANTENNA        TR("Use ext. antenna", "Use external antenna")
#define TR_ANTENNACONFIRM2             TR("Zkont. ant\202nu", "Ujisti se \217e je ant\202na p\206ipojena!")
#define TR_MODULE_PROTOCOL_FLEX_WARN_LINE1   "Vy\217aduje FLEX"
#define TR_MODULE_PROTOCOL_FCC_WARN_LINE1    "Vy\217aduje FCC"
#define TR_MODULE_PROTOCOL_EU_WARN_LINE1     "Vy\217aduje EU"
#define TR_MODULE_PROTOCOL_WARN_LINE2        "firmware."
#define TR_LOWALARM                    INDENT "N\204zk\212 Alarm"
#define TR_CRITICALALARM               INDENT "Kritick\212 Alarm"
#define TR_RSSIALARM_WARN              TR("RSSI","RSSI TELEMETRIE")
#define TR_NO_RSSIALARM                TR(INDENT "Vypnut\202 alarmy", INDENT "Alarmy temetrie vypnuty")
#define TR_DISABLE_ALARM               TR(INDENT "Vypnout alarmy", INDENT "Vypnout telemtrick\202 alarmy")
#define TR_ENABLE_POPUP                "Povolit vyskakovac\204 okno"
#define TR_DISABLE_POPUP               "Zak\200zat vyskakovac\204 okno"
#define TR_POPUP                       "Vysko\201it"
#define TR_MIN                         "Min"
#define TR_MAX                         "Max"
#define TR_CURVE_PRESET                "\207ablona"
#define TR_PRESET                      "\207ablona"
#define TR_MIRROR                      "Zrcadlit"
#define TR_CLEAR                       "Smazat"
#define TR_RESET                       "Reset"
#define TR_RESET_SUBMENU               "Inicializovat ..."
#define TR_COUNT                       "Velikost"
#define TR_PT                          "\201."
#define TR_PTS                         " b."
#define TR_SMOOTH                      "Hladk\200"
#define TR_COPY_STICKS_TO_OFS          TR("P\200ky do subtrimu", "Kopie p\200k do subtrimu")
#define TR_COPY_MIN_MAX_TO_OUTPUTS     TR("Cpy min/max to all",  "Copy min/max/center to all outputs")
#define TR_COPY_TRIMS_TO_OFS           TR("Trimy do subtrimu", "Kop\204e trim\211 do subtrimu")
#define TR_INCDEC                      "Zv\203\207it/Zmen\207it"
#define TR_GLOBALVAR                   "Glob. prom\203nn\200"
#define TR_MIXSOURCE                   "Zdroje mixeru"
#define TR_CONSTANT                    "Konstanta"
#define TR_PERSISTENT_MAH              TR(INDENT "Ulo\217 mAh", INDENT "Ukl\200dat mAh")
#define TR_PREFLIGHT                   "P\206edletov\200 kontrola"
#define TR_CHECKLIST                   TR(INDENT "Pozn\200mky", INDENT "Zobrazit pozn\200mky")
#define TR_FAS_OFFSET                  TR(INDENT "FAS Ofs", INDENT "FAS Ofset")
#define TR_AUX_SERIAL_MODE             "Seriov\212 port"
#define TR_AUX2_SERIAL_MODE            "Seriov\212 port 2"
#define TR_SCRIPT                      "Skript"
#define TR_INPUTS                      "Vstupy"
#define TR_OUTPUTS                     "V\212stupy"
#define STR_EEBACKUP                   "Z\200loha EEPROM"
#define STR_FACTORYRESET               "Tov\200rn\204 reset"
#define TR_CONFIRMRESET                TR("Smazat v\207e ?", "Smazat modely a nastaven\204?")
#define TR_TOO_MANY_LUA_SCRIPTS        "P\206\204li\207 mnoho skript\211!"
#define TR_SPORT_UPDATE_POWER_MODE     "SP Power"
#define LEN_SPORT_UPDATE_POWER_MODES   "\004"
#define TR_SPORT_UPDATE_POWER_MODES    "AUTO""ON\0 "
#define TR_NO_TELEMETRY_SCREENS        "No Telemetry Screens"
#define TR_TOUCH_PANEL                 "Touch panel:"

// Horus and Taranis column headers
#define TR_PHASES_HEADERS_NAME         "N\200zev"
#define TR_PHASES_HEADERS_SW           "Sp\204na\201"
#define TR_PHASES_HEADERS_RUD_TRIM     "Trim sm\203rovky"
#define TR_PHASES_HEADERS_ELE_TRIM     "Trim v\212\207kovky"
#define TR_PHASES_HEADERS_THT_TRIM     "Trim plynu"
#define TR_PHASES_HEADERS_AIL_TRIM     "Trim k\206id\202lek"
#define TR_PHASES_HEADERS_CH5_TRIM     "Trim 5"
#define TR_PHASES_HEADERS_CH6_TRIM     "Trim 6"
#define TR_PHASES_HEADERS_FAD_IN       "P\206echod n\200b\203hu"
#define TR_PHASES_HEADERS_FAD_OUT      "P\206echod konce"

#define TR_LIMITS_HEADERS_NAME         "N\200zev"
#define TR_LIMITS_HEADERS_SUBTRIM      "Subtrim"
#define TR_LIMITS_HEADERS_MIN          "Min"
#define TR_LIMITS_HEADERS_MAX          "Max"
#define TR_LIMITS_HEADERS_DIRECTION    "Invertovat"
#define TR_LIMITS_HEADERS_CURVE        "K\206ivka"
#define TR_LIMITS_HEADERS_PPMCENTER    "St\206ed PPM"
#define TR_LIMITS_HEADERS_SUBTRIMMODE  "Symetrick\202"

#define TR_LSW_HEADERS_FUNCTION        "Funkce"
#define TR_LSW_HEADERS_V1              "Hodnota 1"
#define TR_LSW_HEADERS_V2              "Hodnota 2"
#define TR_LSW_HEADERS_ANDSW           "AND Sp\204na\201"
#define TR_LSW_HEADERS_DURATION        "Trv\200n\204"
#define TR_LSW_HEADERS_DELAY           "Zpo\217d\203n\204"

#define TR_GVAR_HEADERS_NAME           "N\200zev"
#define TR_GVAR_HEADERS_FM0            "Hodnota v re\217imu LR0"
#define TR_GVAR_HEADERS_FM1            "Hodnota v re\217imu LR1"
#define TR_GVAR_HEADERS_FM2            "Hodnota v re\217imu LR2"
#define TR_GVAR_HEADERS_FM3            "Hodnota v re\217imu LR3"
#define TR_GVAR_HEADERS_FM4            "Hodnota v re\217imu LR4"
#define TR_GVAR_HEADERS_FM5            "Hodnota v re\217imu LR5"
#define TR_GVAR_HEADERS_FM6            "Hodnota v re\217imu LR6"
#define TR_GVAR_HEADERS_FM7            "Hodnota v re\217imu LR7"
#define TR_GVAR_HEADERS_FM8            "Hodnota v re\217imu LR8"

// Horus footer descriptions
#define TR_LSW_DESCRIPTIONS            { "Typ porovn\200vac\204 funkce", "Prvn\204 prom\203nn\200", "Druh\200 prom\203nn\200 nebo konstanta", "Druh\200 prom\203nn\200 nebo konstanta", "Dal\207\204 stav povoluj\204c\204 tenko sp\204na\201", "Minim\200ln\204 doba trv\200n\204 sepnut\202ho stavu", "Minim\200ln\204 doba platn\202 podm\204nky pro aktivaci" }

// Horus layouts and widgets
#define TR_FIRST_CHANNEL               "First channel"
#define TR_FILL_BACKGROUND             "Fill background?"
#define TR_BG_COLOR                    "BG Color"
#define TR_SLIDERS_TRIMS               "Sliders+Trims"
#define TR_SLIDERS                     "Sliders"
#define TR_FLIGHT_MODE                 "Flight mode"
#define TR_INVALID_FILE                "Invalid File"
#define TR_TIMER_SOURCE                "Timer source"
#define TR_SIZE                        "Size"
#define TR_SHADOW                      "Shadow"
#define TR_TEXT                        "Text"
#define TR_COLOR                       "Color"
#define TR_MAIN_VIEW_X                 "Main view X"
#define TR_PANEL1_BACKGROUND           "Panel1 background"
#define TR_PANEL2_BACKGROUND           "Panel2 background"

// Taranis About screen
#define TR_ABOUTUS                     TR("  O n\200s   ", "O n\200s")

#define TR_ABOUT_OPENTX_1              TR3("OpenTX je nekomer\201n\204,", "OpenTX\001je\001open-source,", "OpenTX je open-source,")
#define TR_ABOUT_OPENTX_2              TR3("opensource bez z\200ruky.", "nekomer\201n\204 a bez\001z\200ruky.", "nekomer\201n\204 a bez z\200ruky.")
#define TR_ABOUT_OPENTX_3              TR3("Vytvo\206en ve voln\202m \201ase.", "Byl\001vytvo\206en\001nad\207en\212mi\001model\200\206i", "Byl vytvo\206en nad\207en\212mi model\200\206i")
#define TR_ABOUT_OPENTX_4              TR3("Podpora v\212voje pomoc\204", "Podpora\001v\212voje\001pomoc\204", "Podpora v\212voje pomoc\204")
#define TR_ABOUT_OPENTX_5              TR3("finan\201n\204ho daru je v\204t\200na!", "drobn\202ho\001daru\001je\001v\204t\200na!", "drobn\202ho daru je v\204t\200na!")

#define TR_ABOUT_BERTRAND_1            "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2            "Hlavn\204 v\212voj\200\206 OpenTX"
#define TR_ABOUT_BERTRAND_3            "Spoluautor Companion"

#define TR_ABOUT_MIKE_1                "Mike Blandford"
#define TR_ABOUT_MIKE_2                "Specialista na k\205d,"
#define TR_ABOUT_MIKE_3                " a ovlada\201e hardwaru."
#define TR_ABOUT_MIKE_4                ""

#define TR_ABOUT_ROMOLO_1              "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2              "Hlavn\204 v\212voj\200\206"
#define TR_ABOUT_ROMOLO_3              "Companion"

#define TR_ABOUT_ANDRE_1               "Andre Bernet"
#define TR_ABOUT_ANDRE_2               "Funk\201nost, pou\217itelnost,"
#define TR_ABOUT_ANDRE_3               "lad\203n\204 a dokumentace"

#define TR_ABOUT_ROB_1                 "Rob Thomson"
#define TR_ABOUT_ROB_2                 "openrcforums webmaster"

#define TR_ABOUT_KJELL_1               "Kjell Kernen"
#define TR_ABOUT_KJELL_2               "autor www.open-tx.org"
#define TR_ABOUT_KJELL_3               "autor OpenTX Recorder"
#define TR_ABOUT_KJELL_4               "Spoluautor Companion"

#define TR_ABOUT_MARTIN_1              "Martin Hota\206"
#define TR_ABOUT_MARTIN_2              "Design, \201e\207tina"

#if defined(MANUFACTURER_FRSKY)
  #define TR_ABOUT_HARDWARE_1          "FrSky"
  #define TR_ABOUT_HARDWARE_2          "V\212voj a v\212roba Hardware"
  #define TR_ABOUT_HARDWARE_3          "P\206isp\203vatel firmware"
#elif defined(MANUFACTURER_RADIOMASTER)
  #define TR_ABOUT_HARDWARE_1          "Radiomaster"
  #define TR_ABOUT_HARDWARE_2          "V\212voj a v\212roba Hardware"
  #define TR_ABOUT_HARDWARE_3          "P\206isp\203vatel firmware"
#elif defined(MANUFACTURER_JUMPER)
  #define TR_ABOUT_HARDWARE_1          "JumperRC"
  #define TR_ABOUT_HARDWARE_2          "V\212voj a v\212roba Hardware"
  #define TR_ABOUT_HARDWARE_3          "P\206isp\203vatel firmware"
#else
  #define TR_ABOUT_HARDWARE_1          "Brent Nelson"
  #define TR_ABOUT_HARDWARE_2          "Sky9x designer/v\212robce"
  #define TR_ABOUT_HARDWARE_3          ""
#endif

#define TR_ABOUT_PARENTS_1             "Mate\206sk\202 projekty"
#define TR_ABOUT_PARENTS_2             "ersky9x (Mike Blandford)"
#define TR_ABOUT_PARENTS_3             "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4             "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT  's'
#define TR_CHR_LONG   'l'
#define TR_CHR_TOGGLE 't'
#define TR_CHR_HOUR   'h'
#define TR_CHR_INPUT  'I'              // Values between A-I will work

#define TR_BEEP_VOLUME                 "Upozorn\203n\204"
#define TR_WAV_VOLUME                  "Zvuky WAV"
#define TR_BG_VOLUME                   "WAV na pozad\204"

#define TR_TOP_BAR                     "Horn\204 li\207ta"
#define TR_FLASH_ERASE                 "Mazan\204 flash..."
#define TR_FLASH_WRITE                 "Z\200pis flash..."
#define TR_OTA_UPDATE                  "Aktualizace OTA..."
#define TR_MODULE_RESET                "Reset modulu..."
#define TR_UNKNOWN_RX                  "Nezn\200m\212 RX"
#define TR_UNSUPPORTED_RX              "Nepodporovan\212 RX"
#define TR_OTA_UPDATE_ERROR            "Chyba aktualizace"
#define TR_DEVICE_RESET                "Reset za\206\204zen\204..."
#define TR_ALTITUDE                    INDENT "V\212\207ka"
#define TR_SCALE                       "M\203\206\204tko"
#define TR_VIEW_CHANNELS               "Zobrazit kan\200ly"
#define TR_VIEW_NOTES                  "Zobrazit pozn\200mky"
#define TR_MODEL_SELECT                "Zvolit model"
#define TR_MODS_FORBIDDEN              "Zm\203ny nejsou povoleny!"
#define TR_UNLOCKED                    "Odem\201eno"
#define TR_ID                          "ID"
#define TR_PRECISION                   "P\206esnost"
#define TR_RATIO                       "Koeficient"
#define TR_FORMULA                     "Operace"
#define TR_CELLINDEX                   "\201l\200nek"
#define TR_LOGS                        "Logovat"
#define TR_OPTIONS                     "Mo\217nosti"
#define TR_FIRMWARE_OPTIONS            "Firmware options"

#define TR_ALTSENSOR                   "Senzor v\212\207ky"
#define TR_CELLSENSOR                  "Senzor \201l\200nk\211"
#define TR_GPSSENSOR                   "GPS senzor"
#define TR_CURRENTSENSOR               "Senzor"
#define TR_AUTOOFFSET                  "Auto ofset"
#define TR_ONLYPOSITIVE                "Jen kladn\202"
#define TR_FILTER                      "Filtr"
#define TR_TELEMETRYFULL               "V\207echny sloty jsou pln\202!"
#define TR_SERVOS_OK                   "Serva OK"
#define TR_SERVOS_KO                   "Serva KO"
//TODO: translation
#define TR_INVERTED_SERIAL             INDENT "Invert"
#define TR_IGNORE_INSTANCE             TR(INDENT "Chybn\202 ID", INDENT "Ignoruj chyby ID")
#define TR_DISCOVER_SENSORS            "Detekovat nov\202 senzory"
#define TR_STOP_DISCOVER_SENSORS       "Zastavit autodetekci"
#define TR_DELETE_ALL_SENSORS          "Odebrat v\207echny senzory"
#define TR_CONFIRMDELETE               "Opravdu " LCDW_128_480_LINEBREAK "odstranit v\207e ?"
#define TR_SELECT_WIDGET               "Zvolit widget"
#define TR_REMOVE_WIDGET               "Odstranit widget"
#define TR_WIDGET_SETTINGS             "Nastaven\204 widgetu"
#define TR_REMOVE_SCREEN               "Odstranit panel"
#define TR_SETUP_WIDGETS               "Nastavit widgety"
#define TR_USER_INTERFACE              "U\217ivatelk\202 rozhran\204"
#define TR_THEME                       "T\202ma"
#define TR_SETUP                       "Nastaven\204"
#define TR_MAINVIEWX                   "Hlavn\204 panel X"
#define TR_LAYOUT                      "Rozlo\217en\204"
#define TR_ADDMAINVIEW                 "P\206idat hlavn\204 panel"
#define TR_BACKGROUND_COLOR            "Barva pozad\204"
#define TR_MAIN_COLOR                  "Hlavn\204 barva"
#define TR_BAR2_COLOR                  "Secondary bar color"
#define TR_BAR1_COLOR                  "Main bar color"
#define TR_TEXT_COLOR                  "Text color"
#define TR_TEXT_VIEWER                 "Prohl\204\217e\201 textu"

#define TR_MENU_INPUTS                 "\314Vstupy"
#define TR_MENU_LUA                    "\322Lua skripty"
#define TR_MENU_STICKS                 "\307P\200ky"
#define TR_MENU_POTS                   "\310Potenciometry"
#define TR_MENU_MAX                    "\315MAX"
#define TR_MENU_HELI                   "\316Cyklika"
#define TR_MENU_TRIMS                  "\313Trimy"
#define TR_MENU_SWITCHES               "\312Sp\204na\201e"
#define TR_MENU_LOGICAL_SWITCHES       "\312Logick\202 sp\204na\201e"
#define TR_MENU_TRAINER                "\317Tren\202r"
#define TR_MENU_CHANNELS               "\320Kan\200ly"
#define TR_MENU_GVARS                  "\311Glob.prom\203nn\202"
#define TR_MENU_TELEMETRY              "\321Telemetrie"
#define TR_MENU_DISPLAY                "DISPLAY"
#define TR_MENU_OTHER                  "Ostatn\204"
#define TR_MENU_INVERT                 "Invertovat"
#define TR_JITTER_FILTER               "ADC Filtr"
#define TR_RTC_CHECK                   TR("Check RTC", "Check RTC voltage")
#define TR_AUTH_FAILURE                "Auth-failure"
#define TR_RACING_MODE                 "Racing mode"

#define ZSTR_VFR                       "\026\006\022"
#define ZSTR_RSSI                      "\022\023\023\011"
#define ZSTR_R9PW                      "\022\044\020\027"
#define ZSTR_RAS                       "\023\027\022"
#define ZSTR_A1                        "\001\034"
#define ZSTR_A2                        "\001\035"
#define ZSTR_A3                        "\001\036"
#define ZSTR_A4                        "\001\037"
#define ZSTR_BATT                      "\022\350\002\354"
#define ZSTR_ALT                       "\001\364\354"
#define ZSTR_TEMP1                     "\024\363\360\034"
#define ZSTR_TEMP2                     "\024\363\360\035"
#define ZSTR_TEMP3                     "\024\363\360\036"
#define ZSTR_TEMP4                     "\024\363\360\037"
#define ZSTR_RPM2                      "\022\020\015\035"
#define ZSTR_PRES                      "\020\356\373\355"
#define ZSTR_ODO1                      "\017\374\361\034"
#define ZSTR_ODO2                      "\017\374\361\035"
#define ZSTR_TXV                       "\024\030_\026"
#define ZSTR_CURR_SERVO1               "\003\023\352\034"
#define ZSTR_CURR_SERVO2               "\003\023\352\035"
#define ZSTR_CURR_SERVO3               "\003\023\352\036"
#define ZSTR_CURR_SERVO4               "\003\023\352\037"
#define ZSTR_DIST                      "\004\367\355\354"
#define ZSTR_ARM                       "\001\356\363"
#define ZSTR_C50                       "\003\040\033"
#define ZSTR_C200                      "\003\035\033\033"
#define ZSTR_RPM                       "\022\020\015"
#define ZSTR_FUEL                      "\006\353\373\364"
#define ZSTR_VSPD                      "\026\023\360\374"
#define ZSTR_ACCX                      "\001\375\375\030"
#define ZSTR_ACCY                      "\001\375\375\031"
#define ZSTR_ACCZ                      "\001\375\375\032"
#define ZSTR_GYROX                     "\007\031\022\030"
#define ZSTR_GYROY                     "\007\031\022\031"
#define ZSTR_GYROZ                     "\007\031\022\032"
#define ZSTR_CURR                      "\003\353\356\356"
#define ZSTR_CAPACITY                  "\003\377\360\377"
#define ZSTR_VFAS                      "\026\006\001\023"
#define ZSTR_BATT_PERCENT              "\002\377\354%"
#define ZSTR_ASPD                      "\001\023\360\374"
#define ZSTR_GSPD                      "\007\023\360\374"
#define ZSTR_HDG                       "\010\374\371"
#define ZSTR_SATELLITES                "\023\377\354\355"
#define ZSTR_CELLS                     "\003\373\364\355"
#define ZSTR_GPSALT                    "\007\001\364\354"
#define ZSTR_GPSDATETIME               "\004\377\354\373"
#define ZSTR_GPS                       "\007\020\023"
#define ZSTR_BATT1_VOLTAGE             "\022\002\034\026"
#define ZSTR_BATT2_VOLTAGE             "\022\002\035\026"
#define ZSTR_BATT1_CURRENT             "\022\002\034\001"
#define ZSTR_BATT2_CURRENT             "\022\002\035\001"
#define ZSTR_BATT1_CONSUMPTION         "\022\002\034\003"
#define ZSTR_BATT2_CONSUMPTION         "\022\002\035\003"
#define ZSTR_BATT1_TEMP                "\022\002\034\024"
#define ZSTR_BATT2_TEMP                "\022\002\035\024"
#define ZSTR_RB_STATE                  "\022\002\023"
#define ZSTR_CHANS_STATE               "\022\002\003\023"
#define ZSTR_RX_RSSI1                  "\034\022\023\023"
#define ZSTR_RX_RSSI2                  "\035\022\023\023"
#define ZSTR_RX_QUALITY                "\022\021\364\347"
#define ZSTR_RX_SNR                    "\022\023\016\022"
#define ZSTR_RX_NOISE                  "\022\016\355\373"
#define ZSTR_ANTENNA                   "\001\016\024"
#define ZSTR_RF_MODE                   "\022\006\015\004"
#define ZSTR_TX_POWER                  "\024\020\027\022"
#define ZSTR_TX_RSSI                   "\024\022\023\023"
#define ZSTR_TX_QUALITY                "\024\021\364\347"
#define ZSTR_RX_RSSI_PERC              "\022\022\023\020"
#define ZSTR_RX_RF_POWER               "\022\020\027\022"
#define ZSTR_TX_RSSI_PERC              "\024\022\023\020"
#define ZSTR_TX_RF_POWER               "\024\020\027\022"
#define ZSTR_TX_FPS                    "\024\006\020\023"
#define ZSTR_TX_SNR                    "\024\023\016\022"
#define ZSTR_TX_NOISE                  "\024\016\355\373"
#define ZSTR_PITCH                     "\020\354\375\370"
#define ZSTR_ROLL                      "\022\361\364\364"
#define ZSTR_YAW                       "\031\377\351"
#define ZSTR_FLIGHT_MODE               "\006\015"
#define ZSTR_THROTTLE                  "\024\370\356"
#define ZSTR_QOS_A                     "\006\374\373\001"
#define ZSTR_QOS_B                     "\006\374\373\002"
#define ZSTR_QOS_L                     "\006\374\373\014"
#define ZSTR_QOS_R                     "\006\374\373\022"
#define ZSTR_QOS_F                     "\006\014\355\355"
#define ZSTR_QOS_H                     "\010\361\364\374"
#define ZSTR_BIND                      "\002\011\016\004"
#define ZSTR_LAP_NUMBER                "\014\377\360 "
#define ZSTR_GATE_NUMBER               "\007\377\354\373"
#define ZSTR_LAP_TIME                  "\014\377\360\024"
#define ZSTR_GATE_TIME                 "\007\354\373\024"
#define ZSTR_ESC_VOLTAGE               "\005\355\375\026"
#define ZSTR_ESC_CURRENT               "\005\355\375\001"
#define ZSTR_ESC_RPM                   "\005\356\360\363"
#define ZSTR_ESC_CONSUMPTION           "\005\355\375\003"
#define ZSTR_ESC_TEMP                  "\005\355\375\024"
#define ZSTR_SD1_CHANNEL               "\003\370\377\362"
#define ZSTR_GASSUIT_TEMP1             "\007\024\360\034"
#define ZSTR_GASSUIT_TEMP2             "\007\024\360\035"
#define ZSTR_GASSUIT_RPM               "\007\022\020\015"
#define ZSTR_GASSUIT_FLOW              "\007\006\364\361"
#define ZSTR_GASSUIT_CONS              "\007\006\353\373"
#define ZSTR_GASSUIT_RES_VOL           "\007\022\026\364"
#define ZSTR_GASSUIT_RES_PERC          "\007\022\020\375"
#define ZSTR_GASSUIT_MAX_FLOW          "\007\015\006\364"
#define ZSTR_GASSUIT_AVG_FLOW          "\007\001\006\364"
#define ZSTR_SBEC_VOLTAGE              "\002\373\375\026"
#define ZSTR_SBEC_CURRENT              "\002\373\375\001"
#define ZSTR_RB3040_EXTRA_STATE        "\022\002\005\023"
#define ZSTR_RB3040_CHANNEL1           "\003\010\034\001"
#define ZSTR_RB3040_CHANNEL2           "\003\010\035\001"
#define ZSTR_RB3040_CHANNEL3           "\003\010\036\001"
#define ZSTR_RB3040_CHANNEL4           "\003\010\037\001"
#define ZSTR_RB3040_CHANNEL5           "\003\010\040\001"
#define ZSTR_RB3040_CHANNEL6           "\003\010\041\001"
#define ZSTR_RB3040_CHANNEL7           "\003\010\042\001"
#define ZSTR_RB3040_CHANNEL8           "\003\010\043\001"
#define ZSTR_ESC_VIN                   "\005\026\011\016"
#define ZSTR_ESC_TFET                  "\024\006\005\024"
#define ZSTR_ESC_CUR                   "\005\003\025\022"
#define ZSTR_ESC_TBEC                  "\024\002\005\003"
#define ZSTR_ESC_BCUR                  "\003\002\005\003"
#define ZSTR_ESC_VBEC                  "\026\002\005\003"
#define ZSTR_ESC_THR                   "\005\024\010\022"
#define ZSTR_ESC_POUT                  "\005\017\025\024"
#define ZSTR_SMART_BAT_BTMP            "\002\024\363\360"
#define ZSTR_SMART_BAT_BCUR            "\002\003\353\356"
#define ZSTR_SMART_BAT_BCAP            "\002\025\355\373"
#define ZSTR_SMART_BAT_MIN_CEL         "\003\014\015\367"
#define ZSTR_SMART_BAT_MAX_CEL         "\003\014\015\377"
#define ZSTR_SMART_BAT_CYCLES          "\003\347\375\364"
#define ZSTR_SMART_BAT_CAPACITY        "\002\003\360\024"
#define ZSTR_CL01                      "\003\373\364\034"
#define ZSTR_CL02                      "\003\373\364\035"
#define ZSTR_CL03                      "\003\373\364\036"
#define ZSTR_CL04                      "\003\373\364\037"
#define ZSTR_CL05                      "\003\373\364\040"
#define ZSTR_CL06                      "\003\373\364\041"
#define ZSTR_CL07                      "\003\373\364\042"
#define ZSTR_CL08                      "\003\373\364\043"
#define ZSTR_CL09                      "\003\373\364\044"
#define ZSTR_CL10                      "\003\364\034\033"
#define ZSTR_CL11                      "\003\364\034\034"
#define ZSTR_CL12                      "\003\364\034\035"
#define ZSTR_CL13                      "\003\364\034\036"
#define ZSTR_CL14                      "\003\364\034\037"
#define ZSTR_CL15                      "\003\364\034\040"
#define ZSTR_CL16                      "\003\364\034\041"
#define ZSTR_CL17                      "\003\364\034\042"
#define ZSTR_CL18                      "\003\364\034\043"
#define ZSTR_FRAME_RATE                "\006\022\377\354"
#define ZSTR_TOTAL_LATENCY             "\024\014\377\354"
#define ZSTR_VTX_FREQ                  "\026\006\356\357"
#define ZSTR_VTX_PWR                   "\026\020\351\356"
#define ZSTR_VTX_CHAN                  "\026\003\370\362"
#define ZSTR_VTX_BAND                  "\026\002\377\362"
#define ZSTR_SERVO_CURRENT             "\023\356\352\001"
#define ZSTR_SERVO_VOLTAGE             "\023\356\352\026"
#define ZSTR_SERVO_TEMPERATURE         "\023\356\352\024"
#define ZSTR_SERVO_STATUS              "\023\356\352\023"
#define ZSTR_LOSS                      "\014\361\355\355"
#define ZSTR_SPEED                     "\023\360\374 "
#define ZSTR_FLOW                      "\006\364\361\351"

//
// HoTT Telemetry sensor names by Hott device
//
// naming convention: Name of device in capital letters (1 or 2) followed by sensor name in lower case letters
//
// example: GPal = GPS altitude, GAal = GAM altitude, Valt = Vario altitude, GAt2 = GAM temperature sensor 2
//
// T  = transmitter
// R  = receiver
// V  = Vario
// G  = GPS
// ES = ESC
// EA = EAM
//
// ssi = signal strength indicator
// qly = quality
// bt  = battery
// evt = HoTT warnings
// btm = battery lowest voltage
// vpk = VPack
// al or alt = altitude
// vv  = vertical velocity (climb rate)
// hd  = heading
// sp  = speed
// PS  = GPS coordinates
// di  = direction
// ns  = number of satellites
// cp  = capacity
// u   = voltage (may be followed by numner if device offers more voltage sensors
// i   = current (may be followed by numner if device offers more current sensors
// tmp or t = temperature (single t may be followed by numner if device offers more temperature sensors
// rp or r  = temperature (single r may be followed by numner if device offers more temperature sensors
// fl = fuel
//  
// TX
#define ZSTR_HOTT_ID_TX_RSSI_DL     "\024\355\355\367"
#define ZSTR_HOTT_ID_TX_LQI_DL      "\024\357\364\347"
// RX
#define ZSTR_HOTT_ID_RX_RSSI_UL     "\022\355\355\367"
#define ZSTR_HOTT_ID_RX_LQI_UL      "\022\357\364\347"
#define ZSTR_HOTT_ID_RX_VLT         "\022\376\354"
#define ZSTR_HOTT_ID_RX_TMP         "\022\354\363\360" 
#define ZSTR_HOTT_ID_RX_BAT_MIN     "\022\376\354\363"
#define ZSTR_HOTT_ID_RX_VPCK        "\022\352\360\365"
#define ZSTR_HOTT_ID_RX_EVENT       "\022\373\352\354"
// Vario
#define ZSTR_HOTT_ID_VARIO_ALT      "\026\377\364\354"
#define ZSTR_HOTT_ID_VARIO_VV       "\026\352\352"
#define ZSTR_HOTT_ID_VARIO_HDG      "\026\370\374\371"
// GPS
#define ZSTR_HOTT_ID_GPS_HDG        "\007\020\370\374"
#define ZSTR_HOTT_ID_GPS_SPEED      "\007\020\355\360"
#define ZSTR_HOTT_ID_GPS_LL         "\007\020\023"
#define ZSTR_HOTT_ID_GPS_DST        "\007\020\374\367"
#define ZSTR_HOTT_ID_GPS_ALT        "\007\020\377\364" 
#define ZSTR_HOTT_ID_GPS_VV         "\007\020\352\352"
#define ZSTR_HOTT_ID_GPS_NSATS      "\007\020\362\355"
// ESC
#define ZSTR_HOTT_ID_ESC_VLT        "\005\023\353\034"
#define ZSTR_HOTT_ID_ESC_CAP        "\005\023\375\360"
#define ZSTR_HOTT_ID_ESC_TMP        "\005\023\354\034" 
#define ZSTR_HOTT_ID_ESC_CUR        "\005\023\367\034"
#define ZSTR_HOTT_ID_ESC_RPM        "\005\023\356\360"
#define ZSTR_HOTT_ID_ESC_BEC_VLT    "\005\023\353\035"
#define ZSTR_HOTT_ID_ESC_BEC_CUR    "\005\023\367\035" 
#define ZSTR_HOTT_ID_ESC_BEC_TMP    "\005\023\354\035"
#define ZSTR_HOTT_ID_ESC_MOT_TMP    "\005\023\354\036"
// GAM
#define ZSTR_HOTT_ID_GAM_CELS       "\007\001\375\364"
#define ZSTR_HOTT_ID_GAM_VLT1       "\007\001\353\034"
#define ZSTR_HOTT_ID_GAM_VLT2       "\007\001\353\035"
#define ZSTR_HOTT_ID_GAM_TMP1       "\007\001\354\034"
#define ZSTR_HOTT_ID_GAM_TMP2       "\007\001\354\035"
#define ZSTR_HOTT_ID_GAM_FUEL       "\007\001\372\364"
#define ZSTR_HOTT_ID_GAM_RPM1       "\007\001\356\034"
#define ZSTR_HOTT_ID_GAM_ALT        "\007\001\377\364"
#define ZSTR_HOTT_ID_GAM_VV         "\007\001\352\352"
#define ZSTR_HOTT_ID_GAM_CUR        "\007\001\367"
#define ZSTR_HOTT_ID_GAM_VLT3       "\007\001\353\036"
#define ZSTR_HOTT_ID_GAM_CAP        "\007\001\375\360"
#define ZSTR_HOTT_ID_GAM_SPEED      "\007\001\355\360"
#define ZSTR_HOTT_ID_GAM_RPM2       "\007\001\356\035"
// EAM
#define ZSTR_HOTT_ID_EAM_CELS_L     "\005\001\375\034"
#define ZSTR_HOTT_ID_EAM_CELS_H     "\005\001\375\035"
#define ZSTR_HOTT_ID_EAM_VLT1       "\005\001\353\034"
#define ZSTR_HOTT_ID_EAM_VLT2       "\005\001\353\035"
#define ZSTR_HOTT_ID_EAM_TMP1       "\005\001\354\034"
#define ZSTR_HOTT_ID_EAM_TMP2       "\005\001\354\035"
#define ZSTR_HOTT_ID_EAM_ALT        "\005\001\377\364"
#define ZSTR_HOTT_ID_EAM_CUR        "\005\001\367"
#define ZSTR_HOTT_ID_EAM_VLT3       "\005\001\353\036"
#define ZSTR_HOTT_ID_EAM_CAP        "\005\001\375\360"
#define ZSTR_HOTT_ID_EAM_VV         "\005\001\352\352"
#define ZSTR_HOTT_ID_EAM_RPM        "\005\001\356\360"
#define ZSTR_HOTT_ID_EAM_SPEED      "\005\001\355p"