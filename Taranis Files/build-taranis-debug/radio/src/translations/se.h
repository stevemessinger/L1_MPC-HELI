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

// SE translations author: Kjell Kernen <kjell.kernen@gmail.com>

/*
 * !!!!! DO NOT EDIT se.h - EDIT se.h.txt INSTEAD !!!!!!!
 *
 * In order to make translations easier se.h.txt is parsed and national
 * characters are replaced by bitmap codes. The result is se.h.
 *
 * See translate.py in the util folder for the list of character codes
 *
 * !!!!! DO NOT EDIT se.h - EDIT se.h.txt INSTEAD !!!!!!!
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
#define LEN_OFFON              "\003"
#define TR_OFFON               "Av ""P\200\0"

#define LEN_MMMINV             "\003"
#define TR_MMMINV              "---""INV"

#define LEN_VBEEPMODE          "\005"
#define TR_VBEEPMODE           "Tyst\0""Alarm""EjKnp""Alla\0"

#define LEN_VBLMODE            TR("\005", "\010")
#define TR_VBLMODE             TR("Av\0  ""Knapp""Spak\0""Allt\0""P\200\0  ", "Av\0     ""Knappar\0""Spakar\0 ""Allt\0   ""P\200\0     ")

#define LEN_TRNMODE            "\002"
#define TR_TRNMODE             "Av""+="":="

#define LEN_TRNCHN             "\003"
#define TR_TRNCHN              "KN1KN2KN3KN4"

#define LEN_AUX_SERIAL_MODES   "\022"
#if defined(CLI) || defined(DEBUG)
#define TR_AUX_SERIAL_MODES    "Debug\0            ""Spegling av S-Port""Telemetri\0        ""SBUS Trainer\0     ""LUA\0              "
#else
#define TR_AUX_SERIAL_MODES    "Av\0               ""Spegling av S-Port""Telemetri\0        ""SBUS Trainer\0     ""LUA\0              "
#endif

#define LEN_SWTYPES            "\006"
#define TR_SWTYPES             "Ingen\0""Flipp\0""2Pos\0 ""3Pos\0"

#define LEN_POTTYPES           TR("\013","\020")
#define TR_POTTYPES            TR("None\0      ""Pot w. det\0""Multipos\0  ""Pot\0       ", "Ingen\0          ""Mittl\201ges-pot\0  ""Flerl\201gesv\201ljare""Potentiometer\0")

#define LEN_SLIDERTYPES        "\010"
#define TR_SLIDERTYPES         "Ingen\0  ""Skjutpot"

#define LEN_VLCD               "\006"
#define TR_VLCD                "NormalOptrex"

#define LEN_VPERSISTENT        "\014"
#define TR_VPERSISTENT         "Av\0         ""Flygning\0   ""Nolla Sj\201lv\0"

#define LEN_COUNTRYCODES       TR("\002", "\007")
#define TR_COUNTRYCODES        TR("US""JP""EU", "Amerika""Japan\0 ""Europa\0")

#if defined(RADIO_FAMILY_TBS)
#define LEN_USBMODES                   TR("\006", "\010")
#define TR_USBMODES                    TR("Ask\0  ""Joyst\0""Agent\0""SDCard""Serial", "Ask\0    ""Joystick""Agent\0 ""Storage\0""Serial\0 ")
#else
#define LEN_USBMODES           TR("\006", "\010")
#if defined(DEBUG)
#define TR_USBMODES            TR("Ask\0  ""Joyst\0""SDCard""Serial", "Ask\0    ""Joystick""Storage\0""Serial\0 ")
#else
#define TR_USBMODES            TR("Ask\0  ""Joyst\0""SDCard""Telem\0", "Ask\0    ""Joystick""Storage\0""Telem\0  ")
#endif
#endif

#define LEN_JACKMODES          "\007"
#define TR_JACKMODES           "Ask\0   ""Audio\0 ""Trainer"

#define LEN_TELEMETRY_PROTOCOLS "\017"
#define TR_TELEMETRY_PROTOCOLS "FrSky S.PORT\0  ""FrSky D\0       ""FrSky D (cable)""TBS Crossfire\0 ""Spektrum\0      ""AFHDS2A IBUS\0  ""Multi Telemetry"

#define TR_MULTI_CUSTOM        "Custom"

#define LEN_VTRIMINC           TR("\006","\014")
#define TR_VTRIMINC            TR("Expo\0 ""xFin\0 ""Fin\0  ""Medium""Grov\0 ","Exponentiell""Extra Fin\0  ""Fin\0        ""Medium\0     ""Grov\0       ")

#define LEN_VDISPLAYTRIMS      "\005"
#define TR_VDISPLAYTRIMS       "Nej\0 ""\204ndra""Ja\0  "

#define LEN_VBEEPCOUNTDOWN     "\007"
#define TR_VBEEPCOUNTDOWN      "Tyst\0  ""Pip\0   ""R\202st\0  ""Vibrera"

#define LEN_VVARIOCENTER       "\004"
#define TR_VVARIOCENTER        "Pip\0""Tyst"

#define LEN_CURVE_TYPES        "\006"
#define TR_CURVE_TYPES         "Normal""Egen\0 "

#define LEN_RETA123            "\001"

#if defined(PCBX9E)
  #define TR_RETA123           "RHGS1234HVHV"
#elif defined(PCBTARANIS) || defined(REVX)
  #define TR_RETA123           "RHGS123HV"
#elif defined(PCBSKY9X)
  #define TR_RETA123           "RHGS123a"
#else
  #define TR_RETA123           "RHGS123"
#endif

#if defined(PCBSKY9X) && defined(REVX)
  #define LEN_VOUTPUT_TYPE     "\011"
  #define TR_VOUTPUT_TYPE      "OpenDrain""PushPull\0"
#endif

#define LEN_VCURVEFUNC         "\003"
#define TR_VCURVEFUNC          "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX             "\010"
#define TR_VMLTPX              "Addera  ""F\202rst\201rk""Ers\201tt  "

#define LEN_VMLTPX2            "\002"
#define TR_VMLTPX2             "+=""*="":="

#define LEN_VMIXTRIMS          "\003"

#if defined(PCBHORUS)
  #define TR_VMIXTRIMS         "Av\0""P\200\0""Rod""Hjd""Gas""Ske""T5\0""T6\0"
#else
  #define TR_VMIXTRIMS         "Av\0""P\200\0""Rod""Hjd""Gas""Ske"
#endif

#if defined(PCBTARANIS)
  #define TR_CSWTIMER          "Timer"
  #define TR_CSWSTICKY         "Seg\0 "
  #define TR_CSWRANGE          "Vidd\0"
  #define TR_CSWSTAY           "Kant\0"
#else
  #define TR_CSWTIMER          "Tim\0 "
  #define TR_CSWSTICKY         "Seg\0 "
    #define TR_CSWRANGE        "Vidd\0"
    #define TR_CSWSTAY         "Kant\0"
#endif

  #define TR_CSWEQUAL          "a=x\0 "

#define LEN_VCSWFUNC           "\005"
#define TR_VCSWFUNC            "---\0 " TR_CSWEQUAL "a\173x\0 ""a>x\0 ""a<x\0 " TR_CSWRANGE "|a|>x""|a|<x""AND\0 ""OR\0  ""XOR\0 " TR_CSWSTAY "a=b\0 ""a>b\0 ""a<b\0 ""\306}x\0 ""|\306|}x" TR_CSWTIMER TR_CSWSTICKY

#define LEN_VFSWFUNC           "\012"

#if defined(VARIO)
  #define TR_VVARIO            "Vario\0    "
#else
  #define TR_VVARIO            "[Vario]   "
#endif

#if defined(AUDIO)
  #define TR_SOUND             "Spela Ljud"
#else
  #define TR_SOUND             "Pip\0      "
#endif

#if defined(HAPTIC)
  #define TR_HAPTIC            "Vibrator\0 "
#else
  #define TR_HAPTIC            "[Vibrator]"
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK      "Spela Sp\200r"
  #else
    #define TR_PLAY_TRACK      "Spela upp\0"
  #endif
  #define TR_PLAY_BOTH         "Spela B\200da"
  #define TR_PLAY_VALUE        TR("S\201g v\201rdet", "S\201g v\201rdet")
#else
  #define TR_PLAY_TRACK        "[Sp. Sp\200r]"
  #define TR_PLAY_BOTH         "[Sp. B\200da]"
  #define TR_PLAY_VALUE        "[Sp. V\201rd]"
#endif

#define TR_SF_BG_MUSIC        "BgMusik\0  ""BgMusik ||"

#if defined(SDCARD)
  #define TR_SDCLOGS           "SD Loggar\0"
#else
  #define TR_SDCLOGS           "[SD Logg]\0"
#endif

#ifdef GVARS
  #define TR_ADJUST_GVAR       "Justera\0  "
#else
  #define TR_ADJUST_GVAR       "[Just. GV]"
#endif

#if defined(LUA)
  #define TR_SF_PLAY_SCRIPT   "Lua Script"
#else
  #define TR_SF_PLAY_SCRIPT   "[Lua]\0    "
#endif

#if defined(DEBUG)
  #define TR_SF_TEST          "Test\0"
#else
  #define TR_SF_TEST
#endif

#if defined(OVERRIDE_CHANNEL_FUNCTION) && LCD_W >= 212
  #define TR_SF_SAFETY        "S\201kra \0   "
#elif defined(OVERRIDE_CHANNEL_FUNCTION)
  #define TR_SF_SAFETY        "S\201kra \0   "
#else
  #define TR_SF_SAFETY        "---\0      "
#endif

#define TR_SF_SCREENSHOT      "Sk\201rmbild\0"
#define TR_SF_RACING_MODE     "RacingMode"
#define TR_SF_RESERVE         "[reserve]\0"

#define TR_VFSWFUNC            TR_SF_SAFETY "Trainer\0  ""S\201tt Trim\0""Nollst\201ll\0""S\201tt\0     " TR_ADJUST_GVAR "Volym\0    " "SetFailsfe" "RangeCheck" "ModuleBind" TR_SOUND TR_PLAY_TRACK TR_PLAY_VALUE TR_SF_RESERVE TR_SF_PLAY_SCRIPT TR_SF_RESERVE TR_SF_BG_MUSIC TR_VVARIO TR_HAPTIC TR_SDCLOGS "Belysning\0" TR_SF_SCREENSHOT TR_SF_RACING_MODE TR_SF_TEST

#define LEN_VFSWRESET          TR("\004", "\011")

#define TR_FSW_RESET_TELEM     TR("Telm", "Telemetry")

#if defined(PCBTARANIS)
  #define TR_FSW_RESET_TIMERS  "Timer 1\0 ""Timer 2\0 ""Timer 3\0 "
#else
  #define TR_FSW_RESET_TIMERS  "Tmr1""Tmr2""Tmr3"
#endif

#define TR_VFSWRESET           TR(TR_FSW_RESET_TIMERS "All\0" TR_FSW_RESET_TELEM, TR_FSW_RESET_TIMERS "Alla\0    " TR_FSW_RESET_TELEM)

#define LEN_FUNCSOUNDS         TR("\004", "\006")
#define TR_FUNCSOUNDS          TR("Bp1\0""Bp2\0""Bp3\0""Wrn1""Wrn2""Chee""Rata""Tick""Sirn""Ring""SciF""Robt""Chrp""Tada""Crck""Alrm", "Beep1 ""Beep2 ""Beep3 ""Warn1 ""Warn2 ""Cheep ""Ratata""Tick  ""Siren ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""AlmClk")

#define LEN_VTELEMCHNS         "\004"

  #define TR_TELEM_RESERVE     TR("[--]", "[---]")
  #define TR_TELEM_TIME        TR("Tid\0", "Tid\0 ")
  #define TR_RAS               TR("SWR\0", "SWR\0 ")
  #define TR_RX_BATT           TR("[NA]", "[NA]\0")
  #define TR_A3_A4             TR("A3\0 ""A4\0 ", "A3\0  ""A4\0  ")
  #define TR_A3_A4_MIN         TR("A3-\0""A4-\0", "A3-\0 ""A4-\0 ")

#define TR_ASPD_MAX            TR("ASp+", "ASpd+")

#if defined(PCBTARANIS)
  #define TR_TELEM_RSSI_RX     "RSSI\0"
#else
  #define TR_TELEM_RSSI_RX     TR("Rx\0  ", "Rx\0 ")
#endif

  #define TR_TELEM_TIMERS      TR("Tmr1""Tmr2""Tmr3", "Tmr1\0""Tmr2\0""Tmr3\0")

#define LENGTH_UNIT_IMP        "fot"
#define SPEED_UNIT_IMP         "mph"
#define LENGTH_UNIT_METR       "m\0 "
#define SPEED_UNIT_METR        "kmh"

#define LEN_VUNITSSYSTEM     TR("\006", "\010")
#define TR_VUNITSSYSTEM      TR("Metri.""Imper.", "Metriska""Imperial")
#define LEN_VTELEMUNIT       "\003"
#define TR_VTELEMUNIT        "-\0 ""V\0 ""A\0 ""mA\0""kts""m/s""f/s""kmh""mph""m\0 ""ft\0""@C\0""@F\0""%\0 ""mAh""W\0 ""mW\0""dB\0""rpm""g\0 ""@\0 ""rad""ml\0""fOz""mlm""Hz\0""mS\0""uS\0""km\0""dbm"

#define STR_V                  (STR_VTELEMUNIT+1)
#define STR_A                  (STR_VTELEMUNIT+4)

#define LEN_VTELEMSCREENTYPE   "\007"
#define TR_VTELEMSCREENTYPE    "Inget\0 ""Siffror""Staplar""Script\0"

#define LEN_GPSFORMAT          "\004"
#define TR_GPSFORMAT           "HMS NMEA"

#define LEN2_VTEMPLATES        12
#define LEN_VTEMPLATES         "\014"
#define TR_TEMPLATE_CLEAR_MIXES        "Nolla Mixar\0"
#define TR_TEMPLATE_SIMPLE_4CH         "Enkel 4kanal"
#define TR_TEMPLATE_STICKY_TCUT        "Gasklippning"
#define TR_TEMPLATE_VTAIL              "V-Stj\201rt    "
#define TR_TEMPLATE_DELTA              "Deltavinge  "
#define TR_TEMPLATE_ECCPM              "eCCPM       "
#define TR_TEMPLATE_HELI               "Helikopter  "
#define TR_TEMPLATE_SERVO_TEST         "Servotest   "

#define LEN_VSWASHTYPE         "\004"
#define TR_VSWASHTYPE          "--- ""120 ""120X""140 ""90\0 "

#if defined(PCBHORUS)
  #define LEN_VKEYS            "\005"
  #define TR_VKEYS             "PGUP\0""PGDN\0""ENTER""MDL\0 ""RTN\0 ""TELE\0""SYS\0 "
#elif defined(RADIO_FAMILY_JUMPER_T12)
  #define LEN_VKEYS            "\005"
  #define TR_VKEYS             "Exit\0""Enter""Down\0""Up\0  ""Right""Left\0"
#elif defined(RADIO_TX12)
  #define LEN_VKEYS            "\005"
  #define TR_VKEYS             "Exit\0""Enter""Up\0  ""Down\0""SYS\0 ""MDL\0 ""TELE\0"
#elif defined(RADIO_T8)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "RTN\0 ""ENTER""PGUP\0""PGDN\0""SYS\0 ""MDL\0 ""UP\0  ""DOWN\0"
#elif defined(PCBTARANIS)
  #define LEN_VKEYS            "\005"
  #define TR_VKEYS             "Menu\0""Exit\0""Enter""Page\0""Plus\0""Minus"
#else
  #define LEN_VKEYS            "\005"
  #define TR_VKEYS             "Menu\0""Exit\0""Ned\0 ""Upp\0 ""H\202ger""V\201nst"
#endif

#define LEN_VSWITCHES          "\003"
#define LEN_VSRCRAW            "\004"


#define TR_STICKS_VSRCRAW      "\307Rod""\307Hjd""\307Gas""\307Ske"

#if defined(PCBHORUS)
  #define TR_TRIMS_VSRCRAW     "\313Rod""\313Hjd""\313Gas""\313Ske""\313T5\0""\313T6\0"
#else
  #define TR_TRIMS_VSRCRAW     TR("TrmR""TrmH""TrmG""TrmS", "\313Rod""\313Hjd""\313Gas""\313Ske")
#endif

#if defined(PCBHORUS)
  #define TR_TRIMS_SWITCHES    "\313Rv""\313Rh""\313Hn""\313Hu""\313Gn""\313Gu""\313Sv""\313Sh""\3135d""\3135u""\3136d""\3136u"
#else
  #define TR_TRIMS_SWITCHES    TR("tRv""tRh""tHn""tHu""tGn""tGu""tSv""tSh", "\313Rv""\313Rh""\313Hn""\313Hu""\313Gn""\313Gu""\313Sv""\313Sh")
#endif

#if defined(PCBSKY9X)
  #define TR_ROTARY_ENCODERS   "REa\0"
  #define TR_ROTENC_SWITCHES   "REa"
#else
  #define TR_ROTARY_ENCODERS
  #define TR_ROTENC_SWITCHES
#endif

#define TR_ON_ONE_SWITCHES     "P\200\0""Ett"

#if defined(GYRO)
  #define TR_GYR_VSRCRAW       "GyrX""GyrY"
#else
  #define TR_GYR_VSRCRAW
#endif

#if defined(HELI)
  #define TR_CYC_VSRCRAW       "CYK1""CYK2""CYK3"
#else
  #define TR_CYC_VSRCRAW       "[C1]""[C2]""[C3]"
#endif

#define TR_RESERVE_VSRCRAW   "[--]"
#define TR_EXTRA_VSRCRAW     "Batt""Tid\0""GPS\0" TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW "Tmr1""Tmr2""Tmr3"

#define LEN_VTMRMODES          "\003"
#define TR_VTMRMODES           "Av\0""P\200\0""GAs""GA%""GAt"

#define LEN_VTRAINERMODES              "\022"
#define TR_VTRAINER_MASTER_JACK        "L\201rare/Uttag\0     "
#define TR_VTRAINER_SLAVE_JACK         "Elev./Uttag\0      "
#define TR_VTRAINER_MASTER_SBUS_MODULE "L\201rare/SBUS-Modul\0"
#define TR_VTRAINER_MASTER_CPPM_MODULE "L\201rare/CPPM-Modul\0"
#define TR_VTRAINER_MASTER_BATTERY     "L\201rare/Serial\0    "
#define TR_VTRAINER_BLUETOOTH          TR("Master/BT\0        ""Slave/BT\0", "Master/Bluetooth\0 ""Slave/Bluetooth\0  ")
#define TR_VTRAINER_MULTI              "Master/Multi\0     "
#define TR_VTRAINER_SPORT_SBUS         "Master/SBUS-Sport\0"

#define LEN_VFAILSAFE          "\011"
#define TR_VFAILSAFE           "Ej givet\0""L\200s Servo""Anpassat\0""Pulsfritt""Mottagare"


#define LEN_VSENSORTYPES        "\010"
#define TR_VSENSORTYPES        "Egen\0   ""Ber\201knad"

#define LEN_VFORMULAS          "\014"
#define TR_VFORMULAS           "Addera\0     ""Medelv\201rde\0 ""Min\0        ""Max\0        ""Multiplicera""Totalsumma\0 ""Cell\0       ""F\202rbrukning ""Str\201cka    "

#define LEN_VPREC              "\004"
#define TR_VPREC               "0.--""0.0 ""0.00"

#define LEN_VCELLINDEX         "\010"
#define TR_VCELLINDEX          "L\201gsta\0 ""1\0      ""2\0      ""3\0      ""4\0      ""5\0      ""6\0      ""H\202gsta\0 ""Skillnad"

#define LEN_GYROS                      "\004"
#define TR_GYROS                       "GyrX""GyrY"

#define LEN_TEXT_SIZE          "\010"
#define TR_TEXT_SIZE           "Standard""Tiny\0   ""Small\0  ""Mid\0    ""Double\0 "

// ZERO TERMINATED STRINGS
#if defined(COLORLCD)
  #define INDENT               "   "
  #define LEN_INDENT           3
  #define INDENT_WIDTH         12
  #define BREAKSPACE           "\036"
#else
  #define INDENT               "\001"
  #define LEN_INDENT           1
  #define INDENT_WIDTH         (FW/2)
  #define BREAKSPACE           " "
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

#define TR_MENUWHENDONE        CENTER "\011" TR_ENTER " Avslutar "
#define TR_FREE                "kvar"
#define TR_DELETEMODEL         "Radera Modell"
#define TR_COPYINGMODEL        "Kopierar Modell "
#define TR_MOVINGMODEL         "Flyttar Modell "
#define TR_LOADINGMODEL        "Laddar Modell   "
#define TR_NAME                "Namn"
#define TR_MODELNAME           "Modellnamn"
#define TR_PHASENAME           "L\201gesnamn "
#define TR_MIXNAME             "Mixnamn "
#define TR_INPUTNAME           "Inputnamn"
#if defined(PCBTARANIS)
  #define TR_EXPONAME          "Radnamn"
#else
  #define TR_EXPONAME          "Exponamn"
#endif
#define TR_BITMAP              "Modellikon"
#define TR_TIMER               TR("Timer","Timer ")
#define TR_ELIMITS             TR("Gr\201nser++","Ut\202kade Gr\201nser")
#define TR_ETRIMS              TR("Trimmar++","Ut\202kade Trimmar")
#define TR_TRIMINC             TR("Trimning","Trim\202kning")
#define TR_DISPLAY_TRIMS       "Display Trims"
#define TR_TTRACE              TR("F\202lj Gas", INDENT "F\202lj Gas")
#define TR_TTRIM               TR("Gastrim", INDENT "Gastrim")
#define TR_TTRIM_SW            TR("T-Trim-Sw", INDENT "Trim switch")
#define TR_BEEPCTR             TR("Cent.pip", "Centerpip")
#define TR_USE_GLOBAL_FUNCS    TR("Glob.Funkt", "Anv\201nd Global Funk.")
#if defined(PCBSKY9X) && defined(REVX)
  #define TR_OUTPUT_TYPE       INDENT "Output"
#endif
#define TR_PROTOCOL            TR("Proto", "Protokoll")
#define TR_PPMFRAME            INDENT "PPM-paket"
#define TR_REFRESHRATE               TR(INDENT "Refresh", INDENT "Refresh rate")
#define STR_WARN_BATTVOLTAGE           TR(INDENT "Output is VBAT: ", INDENT "Warning: output level is VBAT: ")
#define TR_WARN_5VOLTS                 "Warning: output level is 5 volts"
#define TR_MS                  "ms"
#define TR_FREQUENCY                   INDENT "Frequency"
#define TR_SWITCH              "Brytare"
#define TR_TRIMS               "Trimmar"
#define TR_FADEIN              "Tona In"
#define TR_FADEOUT             "Tona Ut"
#define TR_DEFAULT             "Standard"
#define TR_CHECKTRIMS          CENTER "\006Kolla\012Trimmar"
#define OFS_CHECKTRIMS         CENTER_OFS+(9*FW)
#define TR_SWASHTYPE           "Swashtyp"
#define TR_COLLECTIVE          "Kollektiv"
#define TR_AILERON             "Skevroder-k\201lla"
#define TR_ELEVATOR            "H\202jdroder-k\201lla"
#define TR_SWASHRING           "Swashring"
#define TR_ELEDIRECTION        "H\202JD Riktning"
#define TR_AILDIRECTION        "SKEV Riktning"
#define TR_COLDIRECTION        "PITCH Riktn. "
#define TR_MODE                "L\201ge"
#define TR_SUBTYPE             INDENT "Subtype"
#define TR_NOFREEEXPO          "Expo saknas!"
#define TR_NOFREEMIXER         "Mixer saknas!"
#define TR_SOURCE              "K\201lla"
#define TR_WEIGHT              "Vikt"
#define TR_EXPO                TR("Expo","Exponentiell")
#define TR_SIDE                "Sida"
#define TR_DIFFERENTIAL        "Diff."
#define TR_OFFSET              "Offset"
#define TR_TRIM                "Trim"
#define TR_DREX                "DRex"
#define DREX_CHBOX_OFFSET      30
#define TR_CURVE               "Kurva"
#define TR_FLMODE              TR("L\201ge","Flygl\201gen")
#define TR_MIXWARNING          "Varning"
#define TR_OFF                 "Av "
#define TR_ANTENNA                     "Antenna"
#define TR_NO_INFORMATION              TR("No info", "No information")
#define TR_MULTPX              "Multpx"
#define TR_DELAYDOWN           "Dr\202j Ned"
#define TR_DELAYUP             "Dr\202j Upp"
#define TR_SLOWDOWN            "Tr\202g Ned"
#define TR_SLOWUP              "Tr\202g Upp"
#define TR_MIXES               "MIXAR"
#define TR_CV                  "KU"
#define TR_GV                  TR("G", "GV")
#define TR_ACHANNEL            "A\004kanal  "
#define TR_RANGE               INDENT"MinMax"
#define TR_CENTER              INDENT "Center"
#define TR_BAR                 "Data"
#define TR_ALARM               INDENT"Alarm"
#define TR_USRDATA             "Anv\201ndardata"
#define TR_BLADES              INDENT"Blad"
#define TR_SCREEN              "Sk\201rm "
#define TR_SOUND_LABEL         "Ljud "
#define TR_LENGTH              INDENT"Tid"
#define TR_BEEP_LENGTH         INDENT "Pip-l\201ngd"
#define TR_SPKRPITCH           INDENT "Pip-ton"
#define TR_HAPTIC_LABEL        "Vibrator"
#define TR_HAPTICSTRENGTH      INDENT"Styrka"
#define TR_GYRO_LABEL                  "Gyro"
#define TR_GYRO_OFFSET                 "Offset"
#define TR_GYRO_MAX                    "Max"
#define TR_CONTRAST            "Kontrast"
#define TR_ALARMS_LABEL        "Alarm"
#define TR_BATTERY_RANGE       "Batteri-MinMax"
#define TR_BATTERYWARNING      INDENT"Batteri"
#define TR_INACTIVITYALARM     INDENT"Inaktivitet"
#define TR_MEMORYWARNING       INDENT"Lite Minne"
#define TR_ALARMWARNING        INDENT"Ljud Av"
#define TR_RSSISHUTDOWNALARM   TR(INDENT "Rssi Shutdown", INDENT "Check Rssi on Shutdown")
#define TR_MODEL_STILL_POWERED "Model still powered"
#define TR_MODEL_SHUTDOWN              "Shutdown ?"
#define TR_PRESS_ENTER_TO_CONFIRM      "Press enter to confirm"
#define TR_THROTTLE_LABEL      "Gas"
#define TR_THROTTLEREVERSE     TR("Inv.Gas", INDENT "Inverterad Gas")
#define TR_MINUTEBEEP          "Minutpip"
#define TR_BEEPCOUNTDOWN       INDENT "Nedr\201kning"
#define TR_PERSISTENT          TR("J\201mt p\200 ", INDENT"Alltid P\200")
#define TR_BACKLIGHT_LABEL     "Belysning"
#define TR_GHOST_MENU_LABEL            "GHOST MENU"
#define TR_STATUS                      "Status"
#define TR_RAW_12BITS                  "Raw 12 bits"
#define TR_BLDELAY             INDENT "Av efter"
#define TR_BLONBRIGHTNESS      INDENT "P\200 Ljusstyrka"
#define TR_BLOFFBRIGHTNESS     INDENT "Av Ljusstyrka"
#define TR_KEYS_BACKLIGHT              "Keys backlight"
#define TR_SPLASHSCREEN        "Startbild"
#define TR_PWR_ON_DELAY                "Pwr On delay"
#define TR_PWR_OFF_DELAY               "Pwr Off delay"
#define TR_BLCOLOR             INDENT "Color"
#define TR_THROTTLEWARNING     TR("Gasvarning", INDENT "Gasvarning")
#define TR_SWITCHWARNING       TR("Bryt.varn.", INDENT "Brytarvarning")
#define TR_POTWARNINGSTATE     TR(INDENT "Pot&Slid.", INDENT "Pots & sliders")
#define TR_SLIDERWARNING       TR(INDENT "Slid. pos.", INDENT "Slider positions")
#define TR_POTWARNING          TR("Rattvarn.", INDENT "Rattvarning")
#define TR_TIMEZONE            TR("Tidszon", "GPS Tidszon")
#define TR_ADJUST_RTC          TR("Adjust RTC", INDENT "Adjust RTC")
#define TR_GPS                 "GPS"
#define TR_RXCHANNELORD        "Kanaler i RX"
#define TR_STICKS              "Sticks"
#define TR_POTS                "Pots"
#define TR_SWITCHES            "Switches"
#define TR_SWITCHES_DELAY      "Brytarf\202rdr\202jning"
#define TR_SLAVE               "Elev"
#define TR_MODESRC             " L\201ge\004% K\201lla"
#define TR_MULTIPLIER          "Styrka"
#define TR_CAL                 "Kalib."
#define TR_VTRIM               "Trim- +"
#define TR_BG                  "BG:"
#if defined(PCBHORUS)
  #define TR_MENUTOSTART       "Press [Enter] to start"
  #define TR_SETMIDPOINT       "Center sticks/pots/sliders and press [Enter]"
  #define TR_MOVESTICKSPOTS    "Move sticks, pots and sliders and press [Enter]"
#elif defined(COLORLCD)
  #define TR_MENUTOSTART       TR_ENTER " Startar "
  #define TR_SETMIDPOINT       "Centrera Allt"
  #define TR_MOVESTICKSPOTS    "R\202r Spakar/Rattar"
#else
  #define TR_MENUTOSTART       CENTER "\011" TR_ENTER " Startar "
  #define TR_SETMIDPOINT       TR(CENTER "\012Centrera Allt",CENTER "\013Centrera Allt")
  #define TR_MOVESTICKSPOTS    TR(CENTER "\005R\202r Spakar/Rattar",CENTER "\006R\202r Spakar/Rattar")
#endif
#define TR_RXBATT              "Rx Batt:"
#define TR_TXnRX               "Tx:\0Rx:"
#define OFS_RX                 4
#define TR_ACCEL               "Acc:"
#define TR_NODATA              CENTER "DATA SAKNAS"
#define TR_US                         "us"
#define TR_TMIXMAXMS         "Tmix max"
#define TR_FREE_STACK     "Free stack"
#define TR_MENUTORESET         TR_ENTER " Nollar"
#define TR_PPM_TRAINER         "TR"
#define TR_CH                  "KN"
#define TR_MODEL               "Modell"
#define TR_FM                  "FL"
#define TR_MIX                 "MIX"
#define TR_EEPROMLOWMEM        "Minnesbrist"
#define TR_ALERT               "\016OBS"
#define TR_PRESSANYKEYTOSKIP   "Tryck ned en knapp"
#define TR_THROTTLENOTIDLE     "Gasen \201r p\200slagen!"
#define TR_ALARMSDISABLED      "Alarmen Avst\201ngda!"
#define TR_PRESSANYKEY         "Tryck ned en knapp"
#define TR_BADEEPROMDATA       "Minnet kan inte tolkas"
#define TR_BAD_RADIO_DATA      "Data fr\200n radion kan inte tolkas"
#define TR_EEPROMFORMATTING    "Minnet Nollst\201lls"
#define TR_STORAGE_FORMAT      "SD-Lagring f\202rbereds"
#define TR_EEPROMOVERFLOW      "Minnesfel"
#define TR_MENURADIOSETUP      "INST\204LLNINGAR"
#define TR_MENUDATEANDTIME     "DAG OCH TID"
#define TR_MENUTRAINER         "TRAINER (PPM IN)"
#define TR_MENUSPECIALFUNCS    "GLOBALA FUNKTIONER"
#define TR_MENUVERSION         "VERSION"
#define TR_MENU_RADIO_SWITCHES            TR("BRYTARE","TEST AV BRYTARE")
#define TR_MENU_RADIO_ANALOGS             "ANALOGA V\204RDEN"
#define TR_MENUCALIBRATION     "KALIBRERING"
#if defined(COLORLCD)
  #define TR_TRIMS2OFFSETS     "[Spara Trimv\201rden]"
#else
  #define TR_TRIMS2OFFSETS     "\006[Spara Trimv\201rden]"
#endif
#define TR_CHANNELS2FAILSAFE   "Channels=>Failsafe"
#define TR_CHANNEL2FAILSAFE    "Channel=>Failsafe"
#define TR_MENUMODELSEL        TR("MODELL","V\204LJ MODELL")
#define TR_MENUSETUP           TR("V\204RDEN","MODELLINST\204LLNINGAR")
#define TR_MENUFLIGHTMODE      "FLYGL\204GE"
#define TR_MENUFLIGHTMODES     "FLYGL\204GEN"
#define TR_MENUHELISETUP       "HELIKOPTER"

  #define TR_MENUINPUTS        "INPUT"
  #define TR_MENULIMITS        "SERVON"

#define TR_MENUCURVES          "KURVOR"
#define TR_MENUCURVE           "KURVA"
#define TR_MENULOGICALSWITCH   "LOGISK BRYTARE"
#define TR_MENULOGICALSWITCHES "LOGISKA BRYTARE"
#define TR_MENUCUSTOMFUNC      "BRYTARFUNKTIONER"
#define TR_MENUCUSTOMSCRIPTS   "SPECIALKOD"
#define TR_MENUTELEMETRY       "TELEMETRI"
#define TR_MENUTEMPLATES       "MALLAR"
#define TR_MENUSTAT            "STATISTIK"
#define TR_MENUDEBUG           "DEBUG"
#define TR_MONITOR_CHANNELS1   "CHANNELS MONITOR 1/8"
#define TR_MONITOR_CHANNELS2   "CHANNELS MONITOR 9/16"
#define TR_MONITOR_SWITCHES    "LOGICAL SWITCHES MONITOR"
#define TR_MONITOR_CHANNELS3   "CHANNELS MONITOR 17/24"
#define TR_MONITOR_CHANNELS4   "CHANNELS MONITOR 25/32"
#define TR_MONITOR_OUTPUT_DESC "Outputs"
#define TR_MONITOR_MIXER_DESC  "Mixers"
#define TR_RECEIVER_NUM        TR("RxNum", "Mottagare Nr.")
#define TR_RECEIVER            "Receiver"
#define TR_SYNCMENU            "Synk [MENU]"
#define TR_MULTI_RFTUNE        TR("Freq tune", "RF Freq. fine tune")
#define TR_MULTI_RFPOWER               "RF power"
#define TR_MULTI_WBUS                  "Output"
#define TR_MULTI_TELEMETRY     "Telemetry"
#define TR_MULTI_VIDFREQ       TR("Vid. freq.", "Video frequency")
#define TR_RFPOWER             "RF Power"
#define TR_MULTI_FIXEDID               TR("FixedID", "Fixed ID")
#define TR_MULTI_OPTION        TR("Option", "Option value")
#define TR_MULTI_AUTOBIND      TR(INDENT "Bind Ch.",INDENT "Bind on channel")
#define TR_DISABLE_CH_MAP              TR("No Ch. map", "Disable Ch. map")
#define TR_DISABLE_TELEM               TR("No Telem", "Disable Telemetry")
#define TR_MULTI_DSM_AUTODTECT TR(INDENT "Autodetect", INDENT "Autodetect format")
#define TR_MULTI_LOWPOWER      TR(INDENT "Low power", INDENT "Low power mode")
#define TR_MULTI_LNA_DISABLE            INDENT "LNA disable"
#define TR_MODULE_TELEMETRY            TR(INDENT "S.Port", INDENT "S.Port link")
#define TR_MODULE_TELEM_ON             TR("ON", "Enabled")
#define TR_DISABLE_INTERNAL         TR("Disable int. RF", "Disable internal RF")
#define TR_MODULE_NO_SERIAL_MODE       TR("!serial mode", "Not in serial mode")
#define TR_MODULE_NO_INPUT             TR("No input", "No serial input")
#define TR_MODULE_NO_TELEMETRY         TR3( "No telmetry", "No MULTI_TELEMETRY", "No telemetry (enable MULTI_TELEMETRY)")
#define TR_MODULE_WAITFORBIND          "Bind to load protocol"
#define TR_MODULE_BINDING              "Binding"
#define TR_MODULE_UPGRADE_ALERT        TR3("Upg. needed", "Module upgrade required", "Module\036Upgrade required")
#define TR_MODULE_UPGRADE              TR("Upg. advised", "Module update recommended")
#define TR_PULSE_RATE                  "Pulse rate"
#define TR_LINK_SPEED                  "Link speed"
#define TR_REBIND                      "Rebinding required"
#define TR_REG_OK                      "Registration ok"
#define TR_BIND_OK                     "Bind successful"
#define TR_BINDING_CH1_8_TELEM_ON               "Ch1-8 Telem ON"
#define TR_BINDING_CH1_8_TELEM_OFF               "Ch1-8 Telem OFF"
#define TR_BINDING_CH9_16_TELEM_ON               "Ch9-16 Telem ON"
#define TR_BINDING_CH9_16_TELEM_OFF               "Ch9-16 Telem OFF"
#define TR_PROTOCOL_INVALID            TR("Prot. invalid", "Protocol invalid")
#define TR_MODULE_STATUS                TR(INDENT "Status", INDENT "Module Status")
#define TR_MODULE_SYNC                 TR(INDENT "Sync", INDENT "Proto Sync Status")
#define TR_MULTI_SERVOFREQ     TR("Servo rate", "Servo update rate")
#define TR_MULTI_MAX_THROW             TR("Max. Throw", "Enable max. throw")
#define TR_MULTI_RFCHAN                TR("RF Channel", "Select RF channel")
#define TR_LIMIT               INDENT "Niv\200"
#define TR_MINRSSI             "Min Rssi"
#define TR_LATITUDE            "Breddgrad"
#define TR_LONGITUDE           "L\201ngdgrad"
#define TR_GPSCOORD            TR("GPS-Koordinater", "GPS-koordinatsystem")
#define TR_VARIO               TR("Vario", "Variometer")
#define TR_PITCH_AT_ZERO       INDENT "Ton vid Noll"
#define TR_PITCH_AT_MAX        INDENT "Ton vid Max"
#define TR_REPEAT_AT_ZERO      INDENT "Repetera vid Noll"
#define TR_SHUTDOWN            "ST\204NGER AV"
#define TR_SAVEMODEL           "Spara modellinst\201lln."
#define TR_BATT_CALIB          "Kalib. Batteri"
#define TR_CURRENT_CALIB       "Kalib. Str\202m"
#define TR_VOLTAGE             INDENT"Volt"
#define TR_CURRENT             INDENT"Ampere"
#define TR_SELECT_MODEL        "V\201lj Modell"
#define TR_SELECT_MODE                 "Select mode"
#define TR_CREATE_CATEGORY     "Skapa Kategori"
#define TR_RENAME_CATEGORY     "Byt Kat.namn"
#define TR_DELETE_CATEGORY     "Ta Bort Kategori"
#define TR_CREATE_MODEL        "Skapa Modell"
#define TR_DUPLICATE_MODEL     "Dupl. Modell"
#define TR_COPY_MODEL          "Kopiera Modell"
#define TR_MOVE_MODEL          "Flytta Modell"
#define TR_BACKUP_MODEL        "Modell-backup"
#define TR_DELETE_MODEL        "Ta Bort Modell"
#define TR_RESTORE_MODEL       "\203terst\201ll Modell"
#define TR_DELETE_ERROR        "Delete error"
#define TR_CAT_NOT_EMPTY       "Category is not empty"
#define TR_SDCARD_ERROR        "SDCARD-fel"
#define TR_NO_SDCARD           "SDCARD saknas"
#define TR_WAITING_FOR_RX              "Waiting for RX..."
#define TR_WAITING_FOR_TX              "Waiting for TX..."
#define TR_WAITING_FOR_MODULE          TR("Waiting module", "Waiting for module...")
#define TR_NO_TOOLS                    "No tools available"
#define TR_NORMAL                      "Normal"
#define TR_NOT_INVERTED                "Not inv"
#define TR_NOT_CONNECTED               "!Connected"
#define TR_CONNECTED                   "Connected"
#define TR_FLEX_915                    "Flex 915MHz"
#define TR_FLEX_868                    "Flex 868MHz"
#define TR_16CH_WITHOUT_TELEMETRY      TR("16CH without telem.", "16CH without telemetry")
#define TR_16CH_WITH_TELEMETRY         TR("16CH with telem.", "16CH with telemetry")
#define TR_8CH_WITH_TELEMETRY          TR("8CH with telem.", "8CH with telemetry")
#define TR_EXT_ANTENNA                 "Ext. antenna"
#define TR_PIN                         "Pin"
#define TR_UPDATE_RX_OPTIONS           "Update RX options?"
#define TR_UPDATE_TX_OPTIONS           "Update TX options?"
#define TR_MODULES_RX_VERSION          "Modules / RX version"
#define TR_MENU_MODULES_RX_VERSION     "MODULES / RX VERSION"
#define TR_MENU_FIRM_OPTIONS           "FIRMWARE OPTIONS"
#define TR_GYRO                        "Gyro"
#define TR_STICKS_POTS_SLIDERS         "Sticks/Pots/Sliders"
#define TR_PWM_STICKS_POTS_SLIDERS     "PWM Sticks/Pots/Sliders"
#define TR_RF_PROTOCOL                 "RF Protocol"
#define TR_MODULE_OPTIONS              "Module options"
#define TR_POWER                       "Power"
#define TR_NO_TX_OPTIONS               "No TX options"
#define TR_RTC_BATT                    "RTC Batt"
#define TR_POWER_METER_EXT             "Power Meter (EXT)"
#define TR_POWER_METER_INT             "Power Meter (INT)"
#define TR_SPECTRUM_ANALYSER_EXT       "Spectrum (EXT)"
#define TR_SPECTRUM_ANALYSER_INT       "Spectrum (INT)"
#define TR_SDCARD_FULL                 "SD-kort Fullt"
#define TR_NEEDS_FILE                  "NEEDS FILE"
#define TR_EXT_MULTI_SPEC              "opentx-inv"
#define TR_INT_MULTI_SPEC              "stm-opentx-noinv"
#define TR_INCOMPATIBLE        "Inkompatibel"
#define TR_WARNING             "VARNING"
#define TR_EEPROMWARN          "EEPROM"
#define TR_STORAGE_WARNING     "LAGRING"
#define TR_EEPROM_CONVERTING   "EEPROM Konvertering"
#define TR_THROTTLEWARN        "GAS"
#define TR_ALARMSWARN          "ALARM"
#define TR_SWITCHWARN          "BRYTARE"
#define TR_FAILSAFEWARN        "FAILSAFE"
#define TR_TEST_WARNING        TR("TESTING", "TEST BUILD")
#define TR_TEST_NOTSAFE        "Use for tests only"
#define TR_SDCARDVERSIONWARN   "SD Card Check"
#define TR_WRONG_SDCARDVERSION TR("Expected ver: ","Expected version: ")
#define TR_WARN_RTC_BATTERY_LOW        "RTC Battery low"
#define TR_WARN_MULTI_LOWPOWER         "Low power mode"
#define TR_BATTERY                     "BATTERY"
#define TR_WRONG_PCBREV        "Wrong PCB detected"
#define TR_EMERGENCY_MODE      "EMERGENCY MODE"
#define TR_PCBREV_ERROR        "PCB error"
#define TR_NO_FAILSAFE         "Failsafe ej given"
#define TR_KEYSTUCK            "Knapp-fel"
#define TR_INVERT_THR          TR("Invert.Gas?", "Invertera Gasen?")
#define TR_SPEAKER_VOLUME      INDENT "Volym"
#define TR_LCD                 "LCD"
#define TR_BRIGHTNESS          INDENT "Ljusstyrka"
#define TR_CPU_TEMP            "CPU temp.\016>"
#define TR_CPU_CURRENT         "Str\202m\022>"
#define TR_CPU_MAH             "F\202rbrukn."
#define TR_COPROC              "CoProc."
#define TR_COPROC_TEMP         "MB temp. \016>"
#define TR_CAPAWARNING         INDENT "L\200g Kapacitet"
#define TR_TEMPWARNING         TR(INDENT "H\202g Temp ", INDENT "H\202g Temperatur")
#define TR_FUNC                "Funk"
#define TR_V1                  "V1"
#define TR_V2                  "V2"
#define TR_DURATION            "Tidl\201ngd"
#define TR_DELAY               "F\202rdr\202j"
#define TR_SD_CARD             "SD-kort"
#define TR_SDHC_CARD           "SD/HC-kort"
#define TR_NO_SOUNDS_ON_SD     "Inga ljud i SD"
#define TR_NO_MODELS_ON_SD     "Ingen modell i SD"
#define TR_NO_BITMAPS_ON_SD    "Ikoner saknas p\200 SD"
#define TR_NO_SCRIPTS_ON_SD    "Programkod saknas p\200 SD"
#define TR_SCRIPT_SYNTAX_ERROR  TR("Syntax error", "Script syntax error")
#define TR_SCRIPT_PANIC        "Script panic"
#define TR_SCRIPT_KILLED       "Script killed"
#define TR_SCRIPT_ERROR        "Unknown error"
#define TR_PLAY_FILE           "Spela"
#define TR_DELETE_FILE         "Radera"
#define TR_COPY_FILE           "Kopia"
#define TR_RENAME_FILE         "Byt namn"
#define TR_ASSIGN_BITMAP       "Tilldela ikon"
#define TR_ASSIGN_SPLASH       "Splash screen"
#define TR_EXECUTE_FILE        "Utf\202r"
#define TR_REMOVED             " raderad"
#define TR_SD_INFO             "Information"
#define TR_SD_FORMAT           "Format"
#define TR_NA                  "N/A"
#define TR_HARDWARE            "H\200rdvara"
#define TR_FORMATTING          "Formaterar..."
#define TR_TEMP_CALIB          "Temp. kalib."
#define TR_TIME                "Tid "
#define TR_MAXBAUDRATE         "Max bauds"

#define TR_BLUETOOTH            "Bluetooth"
#define TR_BLUETOOTH_DISC       "Discover"
#define TR_BLUETOOTH_INIT       "Init"
#define TR_BLUETOOTH_DIST_ADDR  "Dist addr"
#define TR_BLUETOOTH_LOCAL_ADDR "Local addr"
#define TR_BLUETOOTH_PIN_CODE   "PIN code"
#define TR_BAUDRATE             "BT Baudrate"
#define LEN_BLUETOOTH_MODES     "\011"
#if defined(PCBX9E)
#define TR_BLUETOOTH_MODES      "---\0     ""Enabled\0 "
#else
#define TR_BLUETOOTH_MODES      "---\0     ""Telemetry""Trainer\0"
#endif
#define TR_SD_INFO_TITLE       "SD INFO"
#define TR_SD_TYPE             "Typ: "
#define TR_SD_SPEED            "Hastighet:"
#define TR_SD_SECTORS          "Sektorer:"
#define TR_SD_SIZE             "Strl:"
#define TR_TYPE                INDENT "Typ "
#define TR_GLOBAL_VARS         "Globala Variabler"
#define TR_GVARS               "GLOBAL V."
#define TR_GLOBAL_VAR          "Global Variabel"
#define TR_MENUGLOBALVARS      "GLOBALA VARIABLER"
#define TR_OWN                 "Egen"
#define TR_DATE                "Datum"
#define TR_MONTHS              { "Jan", "Fev", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" }
#define TR_ROTARY_ENCODER      "R.Enks"
#define TR_INVERT_ROTARY       "Invert Rotary"
#define TR_CHANNELS_MONITOR    "Kanal\202versikt"
#define TR_MIXERS_MONITOR      "MIXER-SK\204RM"
#define TR_PATH_TOO_LONG       "F\202r l\200ng s\202kv\201g"
#define TR_VIEW_TEXT           "Visa Text"
#define TR_FLASH_BOOTLOADER    "Skriv BootLoader"
#define TR_FLASH_EXTERNAL_DEVICE "Flash External Device"
#define TR_FLASH_RECEIVER_OTA          "Flash receiver OTA"
#define TR_FLASH_RECEIVER_BY_EXTERNAL_MODULE_OTA "Flash RX by ext. OTA"
#define TR_FLASH_RECEIVER_BY_INTERNAL_MODULE_OTA "Flash RX by int. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_EXTERNAL_MODULE_OTA "Flash FC by ext. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_INTERNAL_MODULE_OTA "Flash FC by int. OTA"
#define TR_FLASH_BLUETOOTH_MODULE      TR("Flash BT module", "Flash Bluetooth module")
#define TR_FLASH_POWER_MANAGEMENT_UNIT          "Flash pwr mngt unit"
#define TR_CURRENT_VERSION             TR("Current vers. ", "Current version: ")
#define TR_HW_REV                      "HW Rev"
#define TR_FLASH_INTERNAL_MODULE "Flash Internal Module"
#define TR_FLASH_INTERNAL_MULTI        TR("Flash Int. Multi", "Flash Internal Multi")
#define TR_FLASH_EXTERNAL_MODULE       "Flash external module"
#define TR_FLASH_EXTERNAL_MULTI        TR("Flash Ext. Multi", "Flash External Multi")
#define TR_FLASH_EXTERNAL_ELRS         TR("Flash Ext. ELRS", "Flash External ELRS")
#define TR_FIRMWARE_UPDATE_ERROR TR("FW update Error","Firmware update error")
#define TR_FIRMWARE_UPDATE_SUCCESS     "Flash successful"
#define TR_WRITING                     "Skriver..."
#define TR_CONFIRM_FORMAT              "Formatera Minnet?"
#define TR_INTERNALRF                  "Intern Radio"
#define TR_INTERNAL_MODULE             TR("Int. module","Internal module")
#define TR_EXTERNAL_MODULE             TR("Ext. module","External module")
#define TR_OPENTX_UPGRADE_REQUIRED     "OpenTX upgrade required"
#define TR_TELEMETRY_DISABLED          "Telem. disabled"
#define TR_MORE_OPTIONS_AVAILABLE      "More options available"
#define TR_NO_MODULE_INFORMATION       "No module information"
#define TR_EXTERNALRF          "Extern Radiomodul"
#define TR_FAILSAFE            TR(INDENT "Failsafe", INDENT "Failsafel\201ge")
#define TR_FAILSAFESET         "FailsafeInst\201llning"
#define TR_REG_ID                      "Reg. ID"
#define TR_OWNER_ID                    "Owner ID"
#define TR_PINMAPSET                   "PINMAP"
#define TR_HOLD                "Hold"
#define TR_HOLD_UPPERCASE              "HOLD"
#define TR_NONE                "None"
#define TR_NONE_UPPERCASE              "NONE"
#define TR_MENUSENSOR          "SENSOR"
#define TR_POWERMETER_PEAK             "Peak"
#define TR_POWERMETER_POWER            "Power"
#define TR_POWERMETER_ATTN             "Attn"
#define TR_POWERMETER_FREQ             "Freq."
#define TR_MENUTOOLS                   "TOOLS"
#define TR_TURN_OFF_RECEIVER           "Turn off receiver"
#define TR_STOPPING                    "Stopping..."
#define TR_MENU_SPECTRUM_ANALYSER      "SPECTRUM ANALYSER"
#define TR_MENU_POWER_METER            "POWER METER"
#define TR_SENSOR              "SENSOR"
#define TR_COUNTRYCODE         "Landskod"
#define TR_USBMODE             "USB Mode"
#define TR_JACKMODE                    "Jack Mode"
#define TR_VOICELANG           "R\202stspr\200k"
#define TR_UNITSSYSTEM         "Enheter"
#define TR_EDIT                "Redigera"
#define TR_INSERT_BEFORE       "Addera F\202re"
#define TR_INSERT_AFTER        "Addera Efter"
#define TR_COPY                "Kopiera"
#define TR_MOVE                "Flytta"
#define TR_PASTE               "Infoga"
#define TR_DELETE              "Radera"
#define TR_INSERT              "Addera"
#define TR_RESET_FLIGHT        "Nollst\201ll Flygning"
#define TR_RESET_TIMER1        "Nollst\201ll Timer1"
#define TR_RESET_TIMER2        "Nollst\201ll Timer2"
#define TR_RESET_TIMER3        "Nollst\201ll Timer3"
#define TR_RESET_TELEMETRY     "Nollst\201ll Telemetri"
#define TR_STATISTICS          "Statistik"
#define TR_ABOUT_US            "Om Oss"
#define TR_USB_JOYSTICK        "USB Joystick (HID)"
#define TR_USB_MASS_STORAGE    "USB Storage (SD)"
#define TR_USB_SERIAL          "USB Serial (Debug)"
#define TR_USB_TELEMETRY       "USB Telem mirror"
#define TR_SETUP_SCREENS       "Setup screens"
#define TR_MONITOR_SCREENS     "Monitors"
#define TR_AND_SWITCH          "OCH Brytare"
#define TR_SF                  "BF"
#define TR_GF                  "GF"
#define TR_SPEAKER             INDENT"H\202gtalare"
#define TR_BUZZER              INDENT"Summer"
#define TR_BYTES               "byte"
#define TR_MODULE_BIND         TR("[Bnd]", "[Bind]")
#define TR_POWERMETER_ATTN_NEEDED      "Attenuator needed"
#define TR_PXX2_SELECT_RX              "Select RX"
#define TR_PXX2_DEFAULT                "<default>"
#define TR_BT_SELECT_DEVICE            "Select device"
#define TR_DISCOVER             "Discover"
#define TR_BUTTON_INIT                 BUTTON("Init")
#define TR_WAITING                     "Waiting..."
#define TR_RECEIVER_DELETE             "Delete receiver?"
#define TR_RECEIVER_RESET              "Reset receiver?"
#define TR_SHARE                       "Share"
#define TR_BIND                        "Bind"
#define TR_REGISTER                    TR("Reg", "Register")
#define TR_MODULE_RANGE        TR("[Tst]", "[Testa]")
#define TR_RECEIVER_OPTIONS            TR("REC. OPTIONS", "RECEIVER OPTIONS")
#define TR_DEL_BUTTON                  BUTTON(TR("Del", "Delete"))
#define TR_RESET_BTN           "[Nollst\201ll]"
#define TR_DEBUG                       "Debug"
#define TR_KEYS_BTN                BUTTON(TR("SW","Switches"))
#define TR_ANALOGS_BTN                 BUTTON(TR("Analog","Analogs"))
#define TR_TOUCH_NOTFOUND              "Touch hardware not found"
#define TR_TOUCH_EXIT                  "Touch screen to exit"
#define TR_CALIBRATION                   "Calibration"
#define TR_SET                 "[Spara]"
#define TR_TRAINER             "Trainer"
#define TR_CHANS                       "Chans"
#define TR_ANTENNAPROBLEM      CENTER "Fel p\200 TX-antennen"
#define TR_MODELIDUSED         TR("ID finns redan","ModellID anv\201nds redan")
#define TR_MODULE              "Modul"
#define TR_RX_NAME                     "Rx Name"
#define TR_TELEMETRY_TYPE      "Telemetrityp"
#define TR_TELEMETRY_SENSORS   "Sensorer"
#define TR_VALUE               "V\201rde"
#define TR_TOPLCDTIMER         "Top LCD Timer"
#define TR_UNIT                "Enhet"
#define TR_TELEMETRY_NEWSENSOR INDENT "L\201gg till sensor..."
#define TR_CHANNELRANGE        INDENT "Kanalomr\200de"
#define TR_AFHDS3_RX_FREQ              TR("RX freq.", "RX frequency")
#define TR_AFHDS3_ONE_TO_ONE_TELEMETRY TR("Unicast/Tel.", "Unicast/Telemetry")
#define TR_AFHDS3_ONE_TO_MANY          "Multicast"
#define TR_AFHDS3_ACTUAL_POWER         TR("Act. pow", "Actual power")
#define TR_AFHDS3_POWER_SOURCE         TR("Power src.", "Power source")
#define TR_ANTENNACONFIRM1     "EXT. ANTENNA"
#if defined(PCBX12S)
#define LEN_ANTENNA_MODES      "\023"
#define TR_ANTENNA_MODES       "Internal\0          ""Ask\0               ""Per model\0         ""Internal + External"
#else
#define LEN_ANTENNA_MODES      "\011"
#define TR_ANTENNA_MODES       "Internal\0""Ask\0     ""Per model""External"
#endif
#define TR_USE_INTERNAL_ANTENNA        TR("Use int. antenna", "Use internal antenna")
#define TR_USE_EXTERNAL_ANTENNA        TR("Use ext. antenna", "Use external antenna")
#define TR_ANTENNACONFIRM2     TR("Check antenna", "Make sure antenna is installed!")
#define TR_MODULE_PROTOCOL_FLEX_WARN_LINE1                "Requires non"
#define TR_MODULE_PROTOCOL_FCC_WARN_LINE1         "Requires FCC"
#define TR_MODULE_PROTOCOL_EU_WARN_LINE1          "Requires EU"
#define TR_MODULE_PROTOCOL_WARN_LINE2                "certified firmware"
#define TR_LOWALARM            INDENT "L\200g-alarm"
#define TR_CRITICALALARM       INDENT "Kritiskt alarm"
#define TR_RSSIALARM_WARN             TR("RSSI","TELEMETRY RSSI")
#define TR_NO_RSSIALARM                TR(INDENT "Alarms disabled", INDENT "Telemetry alarms disabled")
#define TR_DISABLE_ALARM               TR(INDENT "Disable alarms", INDENT "Disable telemetry alarms")
#define TR_ENABLE_POPUP        "Sl\200 p\200 Dialog"
#define TR_DISABLE_POPUP       "Sl\200 av Dialog"
#define TR_POPUP               "Popup"
#define TR_MIN                 "Min"
#define TR_MAX                 "Max"
#define TR_CURVE_PRESET        "Lutning..."
#define TR_PRESET              "Lutning"
#define TR_MIRROR              "Spegla"
#define TR_CLEAR               "T\202m"
#define TR_RESET               "Nollst\201ll"
#define TR_RESET_SUBMENU       "Nollst\201ll..."
#define TR_COUNT               "Antal"
#define TR_PT                  "pt"
#define TR_PTS                 "pkt"
#define TR_SMOOTH              "Mjuk"
#define TR_COPY_STICKS_TO_OFS  TR("Cpy stick->subtrim", "Spara spakar som subtrim")
#define TR_COPY_MIN_MAX_TO_OUTPUTS     TR("Cpy min/max to all",  "Copy min/max/center to all outputs")
#define TR_COPY_TRIMS_TO_OFS   TR("Cpy trim->subtrim", "Spara trimmar som subtrim")
#define TR_INCDEC              "Inkr/Dekrement"
#define TR_GLOBALVAR           "Globala Var"
#define TR_MIXSOURCE           "Mixer K\201lla"
#define TR_CONSTANT            "Konstant"
#define TR_PERSISTENT_MAH      INDENT "Lagra mAh"
#define TR_PREFLIGHT           "Startkontroller"
#define TR_CHECKLIST           TR(INDENT "Checklista", INDENT "Visa Checklista")
#define TR_FAS_OFFSET          TR(INDENT "FAS Ofs", INDENT "FAS Offset")
#define TR_AUX_SERIAL_MODE     "Serieporten"
#define TR_AUX2_SERIAL_MODE    "Serieporten 2"
#define TR_SCRIPT              "Programkod"
#define TR_INPUTS              "Input"
#define TR_OUTPUTS             "Outputs"
#define STR_EEBACKUP            "S\201kerhetskopiera EEPROM"
#define STR_FACTORYRESET        "Factory reset"
#define TR_CONFIRMRESET        "Erase ALL models and settings?"
#define TR_TOO_MANY_LUA_SCRIPTS "F\202r m\200nga Lua-scripts!"
#define TR_SPORT_UPDATE_POWER_MODE     "SP Power"
#define LEN_SPORT_UPDATE_POWER_MODES   "\004"
#define TR_SPORT_UPDATE_POWER_MODES    "AUTO""ON\0 "
#define TR_NO_TELEMETRY_SCREENS        "No Telemetry Screens"
#define TR_TOUCH_PANEL                 "Touch panel:"

// Horus and Taranis column headers
#define TR_PHASES_HEADERS_NAME         "Namn"
#define TR_PHASES_HEADERS_SW           "Brytare"
#define TR_PHASES_HEADERS_RUD_TRIM     "Rodertrim"
#define TR_PHASES_HEADERS_ELE_TRIM     "H\202jdrodertrim"
#define TR_PHASES_HEADERS_THT_TRIM     "Gastrim"
#define TR_PHASES_HEADERS_AIL_TRIM     "Skevrodertrim"
#define TR_PHASES_HEADERS_CH5_TRIM     "Trim 5"
#define TR_PHASES_HEADERS_CH6_TRIM     "Trim 6"
#define TR_PHASES_HEADERS_FAD_IN       "Tona Upp"
#define TR_PHASES_HEADERS_FAD_OUT      "Tona Ned"

#define TR_LIMITS_HEADERS_NAME         "Namn"
#define TR_LIMITS_HEADERS_SUBTRIM      "Subtrim"
#define TR_LIMITS_HEADERS_MIN          "Min"
#define TR_LIMITS_HEADERS_MAX          "Max"
#define TR_LIMITS_HEADERS_DIRECTION    "Riktning"
#define TR_LIMITS_HEADERS_CURVE        "Kurva"
#define TR_LIMITS_HEADERS_PPMCENTER    "PPM-center"
#define TR_LIMITS_HEADERS_SUBTRIMMODE  "Subtrim-l\201ge"

#define TR_LSW_HEADERS_FUNCTION        "Funktion"
#define TR_LSW_HEADERS_V1              "V1"
#define TR_LSW_HEADERS_V2              "V2"
#define TR_LSW_HEADERS_ANDSW           "OCH Brytare"
#define TR_LSW_HEADERS_DURATION        "Tidsl\201ngd"
#define TR_LSW_HEADERS_DELAY           "F\202rdr\202j"

#define TR_GVAR_HEADERS_NAME          "Name"
#define TR_GVAR_HEADERS_FM0           "Value on FM0"
#define TR_GVAR_HEADERS_FM1           "Value on FM1"
#define TR_GVAR_HEADERS_FM2           "Value on FM2"
#define TR_GVAR_HEADERS_FM3           "Value on FM3"
#define TR_GVAR_HEADERS_FM4           "Value on FM4"
#define TR_GVAR_HEADERS_FM5           "Value on FM5"
#define TR_GVAR_HEADERS_FM6           "Value on FM6"
#define TR_GVAR_HEADERS_FM7           "Value on FM7"
#define TR_GVAR_HEADERS_FM8           "Value on FM8"

// Horus footer descriptions
#define TR_LSW_DESCRIPTIONS    { "Comparison type or function", "First variable", "Second variable or constant", "Second variable or constant", "Additional condition for line to be enabled", "Minimum ON duration of the logical switch", "Minimum TRUE duration for the switch to become ON" }

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

// About screen
#define TR_ABOUTUS             "Om Oss"

#define TR_ABOUT_OPENTX_1      "OpenTX \201r icke-kommersiell,"
#define TR_ABOUT_OPENTX_2      "\202ppen programvara utan"
#define TR_ABOUT_OPENTX_3      "garantier som uvecklas helt"
#define TR_ABOUT_OPENTX_4      "ideellt. St\202d i form av"
#define TR_ABOUT_OPENTX_5      "donationer v\201lkomnas!"

#define TR_ABOUT_BERTRAND_1    "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2    "Chefsutvecklare av OpenTX"
#define TR_ABOUT_BERTRAND_3    "Hj\201lputvecklare av Companion"

#define TR_ABOUT_MIKE_1        "Mike Blandford"
#define TR_ABOUT_MIKE_2        "Kod och drivrutins-guru"
#define TR_ABOUT_MIKE_3        "Stor inspirationsk\201lla"
#define TR_ABOUT_MIKE_4        ""

#define TR_ABOUT_ROMOLO_1      "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2      "Chefsutvecklare av Companion"
#define TR_ABOUT_ROMOLO_3      ""

#define TR_ABOUT_ANDRE_1       "Andre Bernet"
#define TR_ABOUT_ANDRE_2       "Funktionalitet, Anv\201ndbarhet,"
#define TR_ABOUT_ANDRE_3       "Fels\202kning, Dokumentation"

#define TR_ABOUT_ROB_1         "Rob Thomson"
#define TR_ABOUT_ROB_2         "Webmaster f\202r openRCforums"

#define TR_ABOUT_KJELL_1       "Kjell Kernen"
#define TR_ABOUT_KJELL_2       "Utvecklare av www.open-tx.org"
#define TR_ABOUT_KJELL_3       "Utvecklare av OpenTX Recorder"
#define TR_ABOUT_KJELL_4       "Hj\201lputvecklare av Companion"

#define TR_ABOUT_MARTIN_1      "Martin Hotar"
#define TR_ABOUT_MARTIN_2      "Grafikdesigner"

#if defined(MANUFACTURER_FRSKY)
  #define TR_ABOUT_HARDWARE_1  "FrSky"
  #define TR_ABOUT_HARDWARE_2  "H\200rdvarudesign/producent"
  #define TR_ABOUT_HARDWARE_3  ""
#elif defined(MANUFACTURER_RADIOMASTER)
  #define TR_ABOUT_HARDWARE_1  "Radiomaster"
  #define TR_ABOUT_HARDWARE_2  "H\200rdvarudesign/producent"
  #define TR_ABOUT_HARDWARE_3  ""
#elif defined(MANUFACTURER_JUMPER)
  #define TR_ABOUT_HARDWARE_1  "JumperRC"
  #define TR_ABOUT_HARDWARE_2  "H\200rdvarudesign/producent"
  #define TR_ABOUT_HARDWARE_3  ""
#else
  #define TR_ABOUT_HARDWARE_1  "Brent Nelson"
  #define TR_ABOUT_HARDWARE_2  "Sky9x design/producent"
  #define TR_ABOUT_HARDWARE_3  ""
#endif

#define TR_ABOUT_PARENTS_1     "Ursprungsprojekt"
#define TR_ABOUT_PARENTS_2     "ersky9x (Mike Blandford)"
#define TR_ABOUT_PARENTS_3     "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4     "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT           's'
#define TR_CHR_LONG            'l'
#define TR_CHR_TOGGLE          't'
#define TR_CHR_HOUR            'h'
#define TR_CHR_INPUT           'I'   // Values between A-I will work

#define TR_BEEP_VOLUME         "Volym Pip"
#define TR_WAV_VOLUME          "Volym Wav"
#define TR_BG_VOLUME           "Volym Bg"

#define TR_TOP_BAR             "Titelraden"
#define TR_FLASH_ERASE                 "Flash erase..."
#define TR_FLASH_WRITE                 "Flash write..."
#define TR_OTA_UPDATE                  "OTA update..."
#define TR_MODULE_RESET                "Module reset..."
#define TR_UNKNOWN_RX                  "Unknown RX"
#define TR_UNSUPPORTED_RX              "Unsupported RX"
#define TR_OTA_UPDATE_ERROR            "OTA update error"
#define TR_DEVICE_RESET                "Device reset..."
#define TR_ALTITUDE            INDENT "H\202jd"
#define TR_SCALE               "Skala"
#define TR_VIEW_CHANNELS       "Visa Kanaler"
#define TR_VIEW_NOTES          "Visa Noteringars"
#define TR_MODEL_SELECT        "V\201lj Modell"
#define TR_MODS_FORBIDDEN      "Modifikationer f\202rbjudna!"
#define TR_UNLOCKED            "Ol\200st"
#define TR_ID                  "ID"
#define TR_PRECISION           "Precision"
#define TR_RATIO               "Ratio"
#define TR_FORMULA             "Formel"
#define TR_CELLINDEX           "Cell index"
#define TR_LOGS                "Logga"
#define TR_OPTIONS             "Options"
#define TR_FIRMWARE_OPTIONS    "Firmware options"

#define TR_ALTSENSOR           "H\202jdsensor"
#define TR_CELLSENSOR          "Cell sensor"
#define TR_GPSSENSOR           "GPS sensor"
#define TR_CURRENTSENSOR       "Sensor"
#define TR_AUTOOFFSET          "Auto Offset"
#define TR_ONLYPOSITIVE        "Positiv"
#define TR_FILTER              "Filter"
#define TR_TELEMETRYFULL       "Alla telemetriplatser upptagna!"
#define TR_SERVOS_OK           "Servos OK"
#define TR_SERVOS_KO           "Servos KO"
//TODO: translation
#define TR_INVERTED_SERIAL     INDENT "Invert"
#define TR_IGNORE_INSTANCE     TR(INDENT "Inst.fel", INDENT "Hantera Instansfel")
#define TR_DISCOVER_SENSORS    "S\202k nya sensorer"
#define TR_STOP_DISCOVER_SENSORS "Avbryt s\202kning"
#define TR_DELETE_ALL_SENSORS  "Ta bort alla sensorer"
#define TR_CONFIRMDELETE       "Ta bort alla?"
#define TR_SELECT_WIDGET       "Select widget"
#define TR_REMOVE_WIDGET       "Remove widget"
#define TR_WIDGET_SETTINGS     "Widget settings"
#define TR_REMOVE_SCREEN       "Remove screen"
#define TR_SETUP_WIDGETS       "Setup widgets"
#define TR_USER_INTERFACE      "User interface"
#define TR_THEME               "Theme"
#define TR_SETUP               "Setup"
#define TR_MAINVIEWX           "Main view X"
#define TR_LAYOUT              "Layout"
#define TR_ADDMAINVIEW         "Add main view"
#define TR_BACKGROUND_COLOR    "Background color"
#define TR_MAIN_COLOR          "Main color"
#define TR_BAR2_COLOR                  "Secondary bar color"
#define TR_BAR1_COLOR                  "Main bar color"
#define TR_TEXT_COLOR                  "Text color"
#define TR_TEXT_VIEWER                 "Text Viewer"

#define TR_MENU_INPUTS         "\314Inputs"
#define TR_MENU_LUA            "\322Lua scripts"
#define TR_MENU_STICKS         "\307Spakars"
#define TR_MENU_POTS           "\310Rattar"
#define TR_MENU_MAX            "\315MAX"
#define TR_MENU_HELI           "\316Cyclic"
#define TR_MENU_TRIMS          "\313Trimm"
#define TR_MENU_SWITCHES       "\312Brytare"
#define TR_MENU_LOGICAL_SWITCHES "\312Logiska Brytare"
#define TR_MENU_TRAINER        "\317Trainer"
#define TR_MENU_CHANNELS       "\320Servon"
#define TR_MENU_GVARS          "\311GVars"
#define TR_MENU_TELEMETRY      "\321Telemetri"
#define TR_MENU_DISPLAY        "DISPLAY"
#define TR_MENU_OTHER          "Annat"
#define TR_MENU_INVERT         "Invertera"
#define TR_JITTER_FILTER       "ADC Filter"
#define TR_RTC_CHECK           TR("Check RTC", "Check RTC voltage")
#define TR_AUTH_FAILURE                "Auth-failure"
#define TR_RACING_MODE                 "Racing mode"

#define ZSTR_VFR               "\026\006\022"
#define ZSTR_RSSI              "\022\023\023\011"
#define ZSTR_R9PW                      "\022\044\020\027"
#define ZSTR_RAS               "\023\027\022"
#define ZSTR_A1                "\001\034"
#define ZSTR_A2                "\001\035"
#define ZSTR_A3                "\001\036"
#define ZSTR_A4                "\001\037"
#define ZSTR_BATT              "\022\350\002\354"
#define ZSTR_ALT               "\001\364\354"
#define ZSTR_TEMP1             "\024\363\360\034"
#define ZSTR_TEMP2             "\024\363\360\035"
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
#define ZSTR_RPM               "\022\020\015"
#define ZSTR_FUEL              "\006\353\373\364"
#define ZSTR_VSPD              "\026\023\360\374"
#define ZSTR_ACCX              "\001\375\375\030"
#define ZSTR_ACCY              "\001\375\375\031"
#define ZSTR_ACCZ              "\001\375\375\032"
#define ZSTR_GYROX             "\007\031\022\030"
#define ZSTR_GYROY             "\007\031\022\031"
#define ZSTR_GYROZ             "\007\031\022\032"
#define ZSTR_CURR              "\003\353\356\356"
#define ZSTR_CAPACITY          "\003\377\360\377"
#define ZSTR_VFAS              "\026\006\001\023"
#define ZSTR_BATT_PERCENT      "\002\377\354%"
#define ZSTR_ASPD              "\001\023\360\374"
#define ZSTR_GSPD              "\007\023\360\374"
#define ZSTR_HDG               "\010\374\371"
#define ZSTR_SATELLITES        "\023\377\354\355"
#define ZSTR_CELLS             "\003\373\364\355"
#define ZSTR_GPSALT            "\007\001\364\354"
#define ZSTR_GPSDATETIME       "\004\377\354\373"
#define ZSTR_GPS               "\007\020\023"
#define ZSTR_BATT1_VOLTAGE     "\022\002\034\026"
#define ZSTR_BATT2_VOLTAGE     "\022\002\035\026"
#define ZSTR_BATT1_CURRENT     "\022\002\034\001"
#define ZSTR_BATT2_CURRENT     "\022\002\035\001"
#define ZSTR_BATT1_CONSUMPTION "\022\002\034\003"
#define ZSTR_BATT2_CONSUMPTION "\022\002\035\003"
#define ZSTR_BATT1_TEMP        "\022\002\034\024"
#define ZSTR_BATT2_TEMP        "\022\002\035\024"
#define ZSTR_RB_STATE          "\022\002\023"
#define ZSTR_CHANS_STATE       "\022\002\003\023"
#define ZSTR_RX_RSSI1          "\034\022\023\023"
#define ZSTR_RX_RSSI2          "\035\022\023\023"
#define ZSTR_RX_QUALITY        "\022\021\364\347"
#define ZSTR_RX_SNR            "\022\023\016\022"
#define ZSTR_RX_NOISE          "\022\016\355\373"
#define ZSTR_ANTENNA           "\001\016\024"
#define ZSTR_RF_MODE           "\022\006\015\004"
#define ZSTR_TX_POWER          "\024\020\027\022"
#define ZSTR_TX_RSSI           "\024\022\023\023"
#define ZSTR_TX_QUALITY        "\024\021\364\347"
#define ZSTR_RX_RSSI_PERC      "\022\022\023\020"
#define ZSTR_RX_RF_POWER       "\022\020\027\022"
#define ZSTR_TX_RSSI_PERC      "\024\022\023\020"
#define ZSTR_TX_RF_POWER       "\024\020\027\022"
#define ZSTR_TX_FPS            "\024\006\020\023"
#define ZSTR_TX_SNR            "\024\023\016\022"
#define ZSTR_TX_NOISE          "\024\016\355\373"
#define ZSTR_PITCH             "\020\354\375\370"
#define ZSTR_ROLL              "\022\361\364\364"
#define ZSTR_YAW               "\031\377\351"
#define ZSTR_FLIGHT_MODE       "\006\015"
#define ZSTR_THROTTLE          "\024\370\356"
#define ZSTR_QOS_A             "\006\374\373\001"
#define ZSTR_QOS_B             "\006\374\373\002"
#define ZSTR_QOS_L             "\006\374\373\014"
#define ZSTR_QOS_R             "\006\374\373\022"
#define ZSTR_QOS_F             "\006\014\355\355"
#define ZSTR_QOS_H             "\010\361\364\374"
#define ZSTR_BIND              "\002\011\016\004"
#define ZSTR_LAP_NUMBER        "\014\377\360 "
#define ZSTR_GATE_NUMBER       "\007\377\354\373"
#define ZSTR_LAP_TIME          "\014\377\360\024"
#define ZSTR_GATE_TIME         "\007\354\373\024"
#define ZSTR_ESC_VOLTAGE       "\005\355\375\026"
#define ZSTR_ESC_CURRENT       "\005\355\375\001"
#define ZSTR_ESC_RPM           "\005\356\360\363"
#define ZSTR_ESC_CONSUMPTION   "\005\355\375\003"
#define ZSTR_ESC_TEMP          "\005\355\375\024"
#define ZSTR_SD1_CHANNEL       "\003\370\377\362"
#define ZSTR_GASSUIT_TEMP1     "\007\024\360\034"
#define ZSTR_GASSUIT_TEMP2     "\007\024\360\035"
#define ZSTR_GASSUIT_RPM       "\007\022\020\015"
#define ZSTR_GASSUIT_FLOW      "\007\006\364\361"
#define ZSTR_GASSUIT_CONS      "\007\006\353\373"
#define ZSTR_GASSUIT_RES_VOL   "\007\022\026\364"
#define ZSTR_GASSUIT_RES_PERC  "\007\022\020\375"
#define ZSTR_GASSUIT_MAX_FLOW  "\007\015\006\364"
#define ZSTR_GASSUIT_AVG_FLOW  "\007\001\006\364"
#define ZSTR_SBEC_VOLTAGE      "\002\373\375\026"
#define ZSTR_SBEC_CURRENT      "\002\373\375\001"
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
#define ZSTR_HOTT_ID_EAM_SPEED      "\005\001\355\360"
