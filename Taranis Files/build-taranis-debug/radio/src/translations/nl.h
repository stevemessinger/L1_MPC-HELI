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

// NL translations author: Jean-Pierre van Melis (Frater)

/*
 * !!!!! DO NOT EDIT nl.h - EDIT nl.h.txt INSTEAD !!!!!!!
 *
 * In order to make translations easier nl.h.txt is parsed and national
 * characters are replaced by bitmap codes. The result is nl.h.
 * See translate.py in the util folder for the list of character codes
 *
 * !!!!! DO NOT EDIT nl.h - EDIT nl.h.txt INSTEAD !!!!!!!
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
#define TR_OFFON               "UIT""AAN"

#define LEN_MMMINV             "\003"
#define TR_MMMINV              "---""INV"

#define LEN_VBEEPMODE          "\005"
#define TR_VBEEPMODE           "Stil\0""Alarm""NoKey""Alles"

#define LEN_VBLMODE            TR("\005", "\010")
#define TR_VBLMODE             TR("UIT\0 ""Keys\0""Stick""Beide""AAN\0 ", "UIT\0    ""Toetsen\0""Sticks\0 ""Beide\0  ""AAN\0    ")

#define LEN_TRNMODE            "\003"
#define TR_TRNMODE             "UIT"" +="" :="

#define LEN_TRNCHN             "\003"
#define TR_TRNCHN              "CH1CH2CH3CH4"

#define LEN_AUX_SERIAL_MODES   "\015"
#if defined(CLI) || defined(DEBUG)
#define TR_AUX_SERIAL_MODES    "Debug\0       ""Telem Mirror\0""Telemetry In\0""SBUS Leerling""LUA\0         "
#else
#define TR_AUX_SERIAL_MODES    "UIT\0         ""Telem Mirror\0""Telemetry In\0""SBUS Leerling""LUA\0         "
#endif

#define LEN_SWTYPES            "\006"
#define TR_SWTYPES             "Geen\0 ""Wissel""2POS\0 ""3POS\0"


#define LEN_POTTYPES           TR("\013","\021")
#define TR_POTTYPES            TR("None\0      ""Pot w. det\0""Multipos\0  ""Pot\0       ", "Geen\0            ""Pot met Klik\0    ""Standenschakelaar""Pot zonder Klik\0 ")

#define LEN_SLIDERTYPES        "\006"
#define TR_SLIDERTYPES         "Geen\0 ""Schuif"

#define LEN_VLCD               "\006"
#define TR_VLCD                "NormalOptrex"

#define LEN_VPERSISTENT        "\020"
#define TR_VPERSISTENT         "UIT\0            ""Vliegtijd\0      ""Handmatige Reset"

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

#define LEN_JACKMODES                  "\007"
#define TR_JACKMODES                   "Popup\0 ""Audio\0 ""Trainer"

#define LEN_TELEMETRY_PROTOCOLS "\017"
#define TR_TELEMETRY_PROTOCOLS "FrSky S.PORT\0  ""FrSky D\0       ""FrSky D (cable)""TBS Crossfire\0 ""Spektrum\0      ""AFHDS2A IBUS\0  ""Multi Telemetry"

#define TR_MULTI_CUSTOM        "Custom"

#define LEN_VTRIMINC           TR("\006", "\014")
#define TR_VTRIMINC            TR("Expo\0 ""ExFijn""Fijn\0 ""Medium""Grof\0 ", "Exponentieel""Extra Fijn\0 ""Fijn\0       ""Medium\0     ""Grof\0       ")

#define LEN_VDISPLAYTRIMS      "\006"
#define TR_VDISPLAYTRIMS       "Nee\0  ""Kort\0 ""Ja\0   "

#define LEN_VBEEPCOUNTDOWN     "\006"
#define TR_VBEEPCOUNTDOWN      "StilteBeeps\0SpraakTril\0 "

#define LEN_VVARIOCENTER       "\006"
#define TR_VVARIOCENTER        "Tonen\0""Stilte"

#define LEN_CURVE_TYPES        "\011"
#define TR_CURVE_TYPES         "Standaard""Custom\0  "

#define LEN_RETA123            "\001"

#if defined(PCBHORUS)
  #define TR_RETA123           "RETA13245LR"
#elif defined(PCBX9E)
  #define TR_RETA123           "RETA1234LRLR"
#elif defined(PCBTARANIS) || defined(REVX)
  #define TR_RETA123           "RETA123LR"
#elif defined(PCBSKY9X)
  #define TR_RETA123           "RETA123a"
#else
  #define TR_RETA123           "RETA123"
#endif

#if defined(PCBSKY9X) && defined(REVX)
  #define LEN_VOUTPUT_TYPE     "\011"
  #define TR_VOUTPUT_TYPE      "OpenDrain""PushPull\0"
#endif

#define LEN_VCURVEFUNC         "\003"
#define TR_VCURVEFUNC          "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX             "\010"
#define TR_VMLTPX              "Add\0    ""Multiply""Replace\0"

#define LEN_VMLTPX2            "\002"
#define TR_VMLTPX2             "+=""*="":="

#define LEN_VMIXTRIMS          "\003"

#if defined(PCBHORUS)
  #define TR_VMIXTRIMS         "OFF""ON\0""Rud""Ele""Thr""Ail""T5\0""T6\0"
#else
  #define TR_VMIXTRIMS         "OFF""ON\0""Rud""Ele""Thr""Ail"
#endif

#if LCD_W >= 212
  #define TR_CSWTIMER          "Timer"
  #define TR_CSWSTICKY         "Stcky"
  #define TR_CSWRANGE          "Range"
  #define TR_CSWSTAY           "Edge\0"
#else
  #define TR_CSWTIMER          "Tim\0 "
  #define TR_CSWSTICKY         "Glue\0"
    #define TR_CSWRANGE        "Rnge\0"
    #define TR_CSWSTAY         "Edge\0"
#endif

  #define TR_CSWEQUAL          "a=x\0 "

#define LEN_VCSWFUNC           "\005"
#define TR_VCSWFUNC            "---\0 " TR_CSWEQUAL "a\173x\0 ""a>x\0 ""a<x\0 " TR_CSWRANGE "|a|>x""|a|<x""AND\0 ""OR\0  ""XOR\0 " TR_CSWSTAY "a=b\0 ""a>b\0 ""a<b\0 ""\306}x\0 ""|\306|}x" TR_CSWTIMER TR_CSWSTICKY

#define LEN_VFSWFUNC           "\012"

#if defined(VARIO)
  #define TR_VVARIO            "Vario\0    "
#else
  #define TR_VVARIO            "[Vario]\0  "
#endif

#if defined(AUDIO)
  #define TR_SOUND             "Geluid    "
#else
  #define TR_SOUND             "Beep\0    "
#endif

#if defined(HAPTIC)
  #define TR_HAPTIC            "Haptic\0   "
#else
  #define TR_HAPTIC            "[Haptic]\0 "
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK      "Play\0     "
  #else
    #define TR_PLAY_TRACK      "Play Track"
  #endif
  #define TR_PLAY_BOTH         "Play Both\0"
  #define TR_PLAY_VALUE        TR("Play Val\0 ", "Play Value")
#else
  #define TR_PLAY_TRACK        "[Play Trk]"
  #define TR_PLAY_BOTH         "[Play Bth]"
  #define TR_PLAY_VALUE        "[Play Val]"
#endif

#define TR_SF_BG_MUSIC         "BgMusic\0  ""BgMusic ||"

#if defined(SDCARD)
  #define TR_SDCLOGS           "SD Logs\0  "
#else
  #define TR_SDCLOGS           "[SD Logs]\0"
#endif

#if defined(GVARS)
  #define TR_ADJUST_GVAR       "Wijzig\0   "
#else
  #define TR_ADJUST_GVAR       "[WijzigGV]"
#endif

#if defined(LUA)
  #define TR_SF_PLAY_SCRIPT   "Lua Script"
#else
  #define TR_SF_PLAY_SCRIPT   "[Lua]\0    "
#endif

#if defined(DEBUG)
  #define TR_SF_TEST          "Test\0     "
#else
  #define TR_SF_TEST
#endif

#if defined(OVERRIDE_CHANNEL_FUNCTION) && LCD_W >= 212
  #define TR_SF_SAFETY        "Override\0 "
#elif defined(OVERRIDE_CHANNEL_FUNCTION)
  #define TR_SF_SAFETY        "Overr.\0   "
#else
  #define TR_SF_SAFETY        "---\0      "
#endif

#define TR_SF_SCREENSHOT      "Schermafdr"
#define TR_SF_RACING_MODE     "RacingMode"
#define TR_SF_RESERVE         "[reserve]\0"

#define TR_VFSWFUNC            TR_SF_SAFETY "Trainer\0  ""Inst. Trim""Reset\0    ""Set \0     " TR_ADJUST_GVAR "Volume\0   " "SetFailsfe" "RangeCheck" "ModuleBind" TR_SOUND TR_PLAY_TRACK TR_PLAY_VALUE TR_SF_RESERVE TR_SF_PLAY_SCRIPT TR_SF_RESERVE TR_SF_BG_MUSIC TR_VVARIO TR_HAPTIC TR_SDCLOGS "Backlight\0" TR_SF_SCREENSHOT TR_SF_RACING_MODE TR_SF_TEST

#define LEN_VFSWRESET          TR("\004", "\012")

#define TR_FSW_RESET_TELEM   TR("Telm", "Telemetrie")

#if LCD_W >= 212
  #define TR_FSW_RESET_TIMERS  "Timer 1\0  ""Timer 2\0  ""Timer 3\0  "
#else
  #define TR_FSW_RESET_TIMERS  "Tmr1""Tmr2""Tmr3"
#endif

#define TR_VFSWRESET           TR(TR_FSW_RESET_TIMERS "All\0" TR_FSW_RESET_TELEM, TR_FSW_RESET_TIMERS "Vliegdata\0" TR_FSW_RESET_TELEM)

#define LEN_FUNCSOUNDS         TR("\004", "\006")
#define TR_FUNCSOUNDS          TR("Bp1\0""Bp2\0""Bp3\0""Wrn1""Wrn2""Chee""Rata""Tick""Sirn""Ring""SciF""Robt""Chrp""Tada""Crck""Alrm", "Beep1 ""Beep2 ""Beep3 ""Warn1 ""Warn2 ""Cheep ""Ratata""Tick  ""Siren ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""AlmClk")

#define LEN_VTELEMCHNS         TR("\004", "\005")

#define LENGTH_UNIT_IMP        "ft\0"
#define SPEED_UNIT_IMP         "mph"
#define LENGTH_UNIT_METR       "m\0 "
#define SPEED_UNIT_METR        "kmh"

#define LEN_VUNITSSYSTEM     TR("\006", "\010")
#define TR_VUNITSSYSTEM      TR("Mtrsch""Engels", "Metrisch\0 ""Engels\0   ")
#define LEN_VTELEMUNIT       "\003"
#define TR_VTELEMUNIT        "-\0 ""V\0 ""A\0 ""mA\0""kts""m/s""f/s""kmh""mph""m\0 ""ft\0""@C\0""@F\0""%\0 ""mAh""W\0 ""mW\0""dB\0""rpm""g\0 ""@\0 ""rad""ml\0""fOz""mlm""Hz\0""mS\0""uS\0""km\0""dbm"

#define STR_V                  (STR_VTELEMUNIT+1)
#define STR_A                  (STR_VTELEMUNIT+4)

#define LEN_VTELEMSCREENTYPE   "\006"
#define TR_VTELEMSCREENTYPE    "Geen\0 ""Nums\0 ""Balken""Script"

#define LEN_GPSFORMAT          "\004"
#define TR_GPSFORMAT           "DMS\0""NMEA"

#define LEN2_VTEMPLATES        12
#define LEN_VTEMPLATES         "\014"
#define TR_TEMPLATE_CLEAR_MIXES        "Mix wissen  "
#define TR_TEMPLATE_SIMPLE_4CH         "Simple 4-CH "
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
  #define TR_VKEYS                     "Exit\0""Enter""Down\0""Up\0  ""Right""Left\0"
#elif defined(RADIO_TX12)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Exit\0""Enter""Up\0  ""Down\0""SYS\0 ""MDL\0 ""TELE\0"
#elif defined(RADIO_T8)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "RTN\0 ""ENTER""PGUP\0""PGDN\0""SYS\0 ""MDL\0 ""UP\0  ""DOWN\0"
#elif defined(PCBTARANIS)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Menu\0""Exit\0""Enter""Page\0""Plus\0""Minus"
#else
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Menu\0""Exit\0""Down\0""Up\0  ""Right""Left\0"
#endif

#define LEN_VSWITCHES          "\003"
#define LEN_VSRCRAW            "\004"

#define TR_STICKS_VSRCRAW      "\307Rud""\307Ele""\307Thr""\307Ail"

#if defined(PCBHORUS)
  #define TR_TRIMS_VSRCRAW     "\313Rud""\313Ele""\313Thr""\313Ail""\313T5\0""\313T6\0"
#else
  #define TR_TRIMS_VSRCRAW     TR("TrmR""TrmE""TrmT""TrmA", "\313Rud""\313Ele""\313Thr""\313Ail")
#endif

#if defined(PCBHORUS)
  #define TR_TRIMS_SWITCHES    "\313Rl""\313Rr""\313Ed""\313Eu""\313Td""\313Tu""\313Al""\313Ar""\3135d""\3135u""\3136d""\3136u"
#else
  #define TR_TRIMS_SWITCHES    TR("tRl""tRr""tEd""tEu""tTd""tTu""tAl""tAr", "\313Rl""\313Rr""\313Ed""\313Eu""\313Td""\313Tu""\313Al""\313Ar")
#endif

#if defined(PCBSKY9X)
  #define TR_ROTARY_ENCODERS   "REa\0"
  #define TR_ROTENC_SWITCHES   "REa"
#else
  #define TR_ROTARY_ENCODERS
  #define TR_ROTENC_SWITCHES
#endif

#define TR_ON_ONE_SWITCHES     "ON\0""One"

#if defined(GYRO)
  #define TR_GYR_VSRCRAW       "GyrX""GyrY"
#else
  #define TR_GYR_VSRCRAW
#endif
#if defined(HELI)
  #define TR_CYC_VSRCRAW       "CYC1""CYC2""CYC3"
#else
  #define TR_CYC_VSRCRAW       "[C1]""[C2]""[C3]"
#endif

#define TR_RESERVE_VSRCRAW   "[--]"
#define TR_EXTRA_VSRCRAW     "Batt""Time""GPS\0" TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW "Tmr1""Tmr2""Tmr3"

#define LEN_VTMRMODES          "\003"
#define TR_VTMRMODES           "UIT""AAN""THs""TH%""THt"

#define LEN_VTRAINERMODES              "\022"
#define TR_VTRAINER_MASTER_JACK        "Master/Jack\0      "
#define TR_VTRAINER_SLAVE_JACK         "Slave/Jack\0       "
#define TR_VTRAINER_MASTER_SBUS_MODULE "Master/SBUS Module"
#define TR_VTRAINER_MASTER_CPPM_MODULE "Master/CPPM Module"
#define TR_VTRAINER_MASTER_BATTERY     "Master/Serial\0    "
#define TR_VTRAINER_BLUETOOTH          TR("Master/BT\0        ""Slave/BT\0         ", "Master/Bluetooth\0 ""Slave/Bluetooth\0  ")
#define TR_VTRAINER_MULTI              "Master/Multi\0     "
#define TR_VTRAINER_SPORT_SBUS         "Master/SBUS-Sport\0"

#define LEN_VFAILSAFE          TR("\013","\011")
#define TR_VFAILSAFE           TR("Niet Gezet\0""Vasthouden\0""Custom\0    ""Geen Pulsen""Ontvanger\0 ","Not Set\0 ""Hold\0    ""Custom\0  ""No Pulses""Receiver\0")


#define LEN_VSENSORTYPES        "\010"
#define TR_VSENSORTYPES        "Custom\0 ""Berekend"

#define LEN_VFORMULAS          "\014"
#define TR_VFORMULAS           "Optellen\0   ""Gemiddeld\0  ""Min\0        ""Max\0        ""Vermenigvuld""Totaal\0     ""Cellen\0     ""Verbruik\0   ""Afstand\0    "



#define LEN_VPREC              "\004"
#define TR_VPREC               "0.--""0.0 ""0.00"

#define LEN_VCELLINDEX         "\007"
#define TR_VCELLINDEX          "Laagste""1e Cel\0""2e Cel\0""3e Cel\0""4e Cel\0""5e Cel\0""6e Cel\0""Hoogste""Delta\0 "

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

#define TR_FREE                "vrij"
#define TR_DELETEMODEL         "WIS" BREAKSPACE "MODEL"
#define TR_COPYINGMODEL        "Kopieer Model"
#define TR_MOVINGMODEL         "Verplaats Model"
#define TR_LOADINGMODEL        "Laad Model..."
#define TR_NAME                "Naam"
#define TR_MODELNAME           "Modelnaam"
#define TR_PHASENAME           "Modus"
#define TR_MIXNAME             "Mix Naam"
#define TR_INPUTNAME           "Input Naam"
#define TR_EXPONAME            "Lijn Naam"
#define TR_BITMAP              "Modelafbeelding"
#define TR_TIMER               TR("Timer", "Timer ")
#define TR_ELIMITS             TR("E.Limits", "Extended Limits")
#define TR_ETRIMS              TR("E.Trims", "Extended Trims")
#define TR_TRIMINC             "Trim Step"
#define TR_DISPLAY_TRIMS       "Toon Trims"
#define TR_TTRACE              TR("T-Source", INDENT "Source")
#define TR_TTRIM               TR("T-Trim", INDENT "Trim Idle Only")
#define TR_TTRIM_SW            TR("T-Trim-Sw", INDENT "Trim switch")
#define TR_BEEPCTR             TR("Ctr Beep", "Center Beep")
#define TR_USE_GLOBAL_FUNCS    TR("Glob.Funcs", "Globale Functies")
#if defined(PCBSKY9X) && defined(REVX)
  #define TR_OUTPUT_TYPE       INDENT "Output"
#endif
#define TR_PROTOCOL            TR("Proto", "Protocol")
#define TR_PPMFRAME            INDENT "PPM frame"
#define TR_REFRESHRATE               TR(INDENT "Refresh", INDENT "Refresh rate")
#define STR_WARN_BATTVOLTAGE           TR(INDENT "Output is VBAT: ", INDENT "Warning: output level is VBAT: ")
#define TR_WARN_5VOLTS                 "Warning: output level is 5 volts"
#define TR_MS                  "ms"
#define TR_FREQUENCY                   INDENT "Frequency"
#define TR_SWITCH              TR("Schak.", "Schakelaar")
#define TR_TRIMS               "Trims"
#define TR_FADEIN              "Fade in"
#define TR_FADEOUT             "Fade out"
#define TR_DEFAULT             "(default)"
#define TR_CHECKTRIMS          CENTER "\006Check\012Trims"
#define OFS_CHECKTRIMS         CENTER_OFS+(9*FW)
#define TR_SWASHTYPE           "Swash Type"
#define TR_COLLECTIVE          TR("Collective", "Coll. pitch source")
#define TR_AILERON             TR("Lateral cyc.", "Lateral cyc. source")
#define TR_ELEVATOR            TR("Long. cyc.", "Long. cyc. source")
#define TR_SWASHRING           "Swash Ring"
#define TR_ELEDIRECTION        "ELE Direction"
#define TR_AILDIRECTION        "AIL Direction"
#define TR_COLDIRECTION        "PIT Direction"
#define TR_MODE                "Mode"
#define TR_SUBTYPE             INDENT "Subtype"
#define TR_NOFREEEXPO          "Geen vrije expo!"
#define TR_NOFREEMIXER         "Geen vrije mixer!"
#define TR_SOURCE               "Source"
#define TR_WEIGHT              "Gewicht"
#define TR_EXPO                TR("Expo", "Exponentieel")
#define TR_SIDE                "Side"
#define TR_DIFFERENTIAL        "Diff"
#define TR_OFFSET               "Offset"
#define TR_TRIM                "Trim"
#define TR_DREX                "DRex"
#define DREX_CHBOX_OFFSET      30
#define TR_CURVE               "Curve"
#define TR_FLMODE              TR("Mode", "Modes")
#define TR_MIXWARNING          "Melding"
#define TR_OFF                 "UIT"
#define TR_ANTENNA                     "Antenna"
#define TR_NO_INFORMATION              TR("No info", "No information")
#define TR_MULTPX              "Multpx"
#define TR_DELAYDOWN           "Vertr.Dn"
#define TR_DELAYUP             "Vertr.Up"
#define TR_SLOWDOWN            "Langz.Dn"
#define TR_SLOWUP              "Langz.Up"
#define TR_MIXES               "MIXER"
#define TR_CV                  "CV"
#define TR_GV                  TR("G", "GV")
#define TR_ACHANNEL            "A\004kanaal"
#define TR_RANGE               INDENT "Bereik"
#define TR_CENTER              INDENT "Centreer"
#define TR_BAR                 "Balk"
#define TR_ALARM               INDENT "Alarm"
#define TR_USRDATA             "Data berekenen uit"
#define TR_BLADES              "Bladen"

#define TR_SCREEN              "Scherm\001"
#define TR_SOUND_LABEL         "Geluid-"
#define TR_LENGTH              INDENT "Duur"
#define TR_BEEP_LENGTH         INDENT "Piep-Lengte"
#define TR_SPKRPITCH           INDENT "Piep-Freq. +/-"
#define TR_HAPTIC_LABEL        "Haptic"
#define TR_HAPTICSTRENGTH      INDENT "Sterkte"
#define TR_GYRO_LABEL                  "Gyro"
#define TR_GYRO_OFFSET                 "Offset"
#define TR_GYRO_MAX                    "Max"
#define TR_CONTRAST            "LCD-Kontrast"
#define TR_ALARMS_LABEL        "Alarm"
#define TR_BATTERY_RANGE       TR("Accu Bereik", "Accu Spngs-Bereik") // Symbol Akku Ladezustand

#define TR_BATTERYWARNING      INDENT "Accu laag"
#define TR_INACTIVITYALARM     TR(INDENT "Inactiviteit", INDENT "Inactiviteit na") //9XR-Pro
#define TR_MEMORYWARNING       INDENT "Geheugen laag"
#define TR_ALARMWARNING        TR(INDENT "Al Geluid uit?", INDENT "Al het geluid uit?")
#define TR_RSSISHUTDOWNALARM   TR(INDENT "Rssi Shutdown", INDENT "Check Rssi on Shutdown")
#define TR_MODEL_STILL_POWERED "Model still powered"
#define TR_MODEL_SHUTDOWN              "Shutdown ?"
#define TR_PRESS_ENTER_TO_CONFIRM      "Press enter to confirm"
#define TR_THROTTLE_LABEL      "Gas"
#define TR_THROTTLEREVERSE     TR("Reverse", INDENT "Omgekeerd")
#define TR_MINUTEBEEP          TR("Min-Alarm", "Minuten-Alarm")
#define TR_BEEPCOUNTDOWN       INDENT "Countdown"
#define TR_PERSISTENT          TR(INDENT "Vasth.", INDENT "Vasthouden")
#define TR_BACKLIGHT_LABEL     "LCD-Verlichting"
#define TR_GHOST_MENU_LABEL            "GHOST MENU"
#define TR_STATUS                      "Status"
#define TR_RAW_12BITS                  "Raw 12 bits"
#define TR_BLDELAY             INDENT "Duur"
#define TR_BLONBRIGHTNESS      INDENT "Aan-Helderheid"
#define TR_BLOFFBRIGHTNESS     INDENT "Uit-Helderheid"
#define TR_KEYS_BACKLIGHT              "Keys backlight"
#define TR_BLCOLOR             INDENT "Kleur"
#define TR_SPLASHSCREEN        "Startscherm Aan"
#define TR_PWR_ON_DELAY                "Pwr On delay"
#define TR_PWR_OFF_DELAY               "Pwr Off delay"
#define TR_THROTTLEWARNING     TR(INDENT "T-Warning", INDENT "Throttle Status")
#define TR_SWITCHWARNING       TR(INDENT "S-Warning", INDENT "Switch Posities")
#define TR_POTWARNINGSTATE     TR(INDENT "Pot&Slid.", INDENT "Pots & sliders")
#define TR_SLIDERWARNING       TR(INDENT "Slid. pos.", INDENT "Slider positions")
#define TR_POTWARNING          TR(INDENT "Pot Warn.", INDENT "Pot Posities")
#define TR_TIMEZONE            TR("Tijdzone", "GPS-Tijdzone +/-Std")
#define TR_ADJUST_RTC          TR("Klok instellen", INDENT "Klok middels GPS instellen")
#define TR_GPS                 "GPS"
#define TR_RXCHANNELORD        TR("Kan.Volgorde", "Kanaalvolgorde")
#define TR_STICKS              "Sticks"
#define TR_POTS                "Pots"
#define TR_SWITCHES            TR("Switches","Schakelaars")
#define TR_SWITCHES_DELAY      "Vertraging"
#define TR_SLAVE               CENTER "Leerling"
#define TR_MODESRC             "Mode\006% Source"
#define TR_MULTIPLIER          "Multiplier"
#define TR_CAL                 "Cal"
#define TR_VTRIM               "Trim - +"
#define TR_BG                  "BG:"
#if defined(PCBHORUS)
  #define TR_MENUTOSTART       "Geef [Enter] om te starten"
  #define TR_SETMIDPOINT       "Centreer sticks/pots/schuiven en geef [Enter]"
  #define TR_MOVESTICKSPOTS    "Beweeg sticks/pots/schuiven en geef [Enter]"
#elif defined(COLORLCD)
  #define TR_MENUTOSTART       TR_ENTER " VOOR START"
  #define TR_SETMIDPOINT       "CENTREER STICKS/SLIDERS"
  #define TR_MOVESTICKSPOTS    "BEWEEG STICKS/POTS"
  #define TR_MENUWHENDONE      CENTER "\006" TR_ENTER " BIJ GEREED"
#else
  #define TR_MENUTOSTART       CENTER "\010" TR_ENTER " VOOR START"
  #define TR_SETMIDPOINT       TR(CENTER "\004ZET STICKS NAAR HET MIDDEN", CENTER "\004CENTREER STICKS/SLIDERS")
  #define TR_MOVESTICKSPOTS    CENTER "\006BEWEEG STICKS/POTS"
  #define TR_MENUWHENDONE      CENTER "\006" TR_ENTER " BIJ GEREED"
#endif
#define TR_RXBATT              "Rx Accu:"
#define TR_TXnRX               "Tx:\0Rx:"
#define OFS_RX                 4
#define TR_ACCEL               "Acc:"
#define TR_NODATA              CENTER"Geen Data"
#define TR_US                         "us"
#define TR_TMIXMAXMS         "Tmix max"
#define TR_FREE_STACK     "Free stack"
#define TR_MENUTORESET         TR_ENTER" voor Reset"
#define TR_PPM_TRAINER         "TR"
#define TR_CH                  "CH"
#define TR_MODEL               "MODEL"
#define TR_FM                  "FM"
#define TR_MIX                 "MIX"
#define TR_EEPROMLOWMEM        "EEPROM weinig geheugen"
#define TR_ALERT               "ALARM"
#define TR_PRESSANYKEYTOSKIP   "Druk een toets.."
#define TR_THROTTLENOTIDLE     "Gas niet Dicht!"
#define TR_ALARMSDISABLED      "Alarm uitgeschakeld"
#define TR_PRESSANYKEY         TR("\010Druk een Toets", "Druk een Toets")
#define TR_BADEEPROMDATA       "EEPROM Ongeldig"
#define TR_BAD_RADIO_DATA      "Bad Radio Data"
#define TR_EEPROMFORMATTING    "EEPROM Initialiseren"
#define TR_STORAGE_FORMAT      "Storage Preparation"
#define TR_EEPROMOVERFLOW      "EEPROM Overflow"
#define TR_MENURADIOSETUP      TR("ZENDER-INSTELLEN", "ZENDER-BASISINSTELLINGEN")


#define TR_MENUDATEANDTIME     "DATUM EN TIJD"
#define TR_MENUTRAINER         "LERAAR/LEERLING"
#define TR_MENUSPECIALFUNCS    "GLOBALE FUNKTIES"
#define TR_MENUVERSION         "VERSIE"
#define TR_MENU_RADIO_SWITCHES            TR("Schak.", "Schakelaar-Test")
#define TR_MENU_RADIO_ANALOGS             "Analoog-Test"
#define TR_MENUCALIBRATION     TR("CALIB. ANALOOG", "CALIBRERING Sticks+Pots")

#if defined(COLORLCD)
  #define TR_TRIMS2OFFSETS     "Trims => Subtrims"
#else
  #define TR_TRIMS2OFFSETS     "\006Trims => Subtrims"
#endif
#define TR_CHANNELS2FAILSAFE   "Channels=>Failsafe"
#define TR_CHANNEL2FAILSAFE    "Channel=>Failsafe"
#define TR_MENUMODELSEL        TR("MODELLEN", "MODEL KIEZEN")
#define TR_MENUSETUP           TR("MODEL-INSTELLING", "MODEL-INSTELLINGEN")
#define TR_MENUFLIGHTMODES     "VLIEGFASES"
#define TR_MENUFLIGHTMODE      "VLIEGFASE"
#define TR_MENUHELISETUP       "HELI TS-Mixer"

  #define TR_MENUINPUTS        "INPUTS"
  #define TR_MENULIMITS        "OUTPUTS"

#define TR_MENUCURVES          "CURVEN"
#define TR_MENUCURVE           "CURVE"
#define TR_MENULOGICALSWITCH   "LOGISCHE SCHAK."
#define TR_MENULOGICALSWITCHES "LOGISCHE SCHAKELRS"
#define TR_MENUCUSTOMFUNC      TR("SPEC.-FUNKTIES", "SPECIALE-FUNKTIES")
#define TR_MENUCUSTOMSCRIPTS   "LUA-SCRIPTS"
#define TR_MENUTELEMETRY       "TELEMETRIE"
#define TR_MENUTEMPLATES       "SJABLONEN"
#define TR_MENUSTAT            "STAT"
#define TR_MENUDEBUG           "DEBUG"
#define TR_MONITOR_CHANNELS1   "CHANNELS MONITOR 1-8"
#define TR_MONITOR_CHANNELS2   "CHANNELS MONITOR 9-16"
#define TR_MONITOR_CHANNELS3   "CHANNELS MONITOR 17-24"
#define TR_MONITOR_CHANNELS4   "CHANNELS MONITOR 25-32"
#define TR_MONITOR_SWITCHES    "LOGISCHE SCHAKELAARS MONITOR"
#define TR_MONITOR_OUTPUT_DESC "Outputs"
#define TR_MONITOR_MIXER_DESC  "Mixers"
#define TR_RECEIVER_NUM        TR("RxNum", "Receiver Nr.")
#define TR_RECEIVER            "Receiver"
#define TR_MULTI_RFTUNE        TR("Freq tune", "RF Freq. fine tune")
#define TR_MULTI_RFPOWER               "RF power"
#define TR_MULTI_WBUS                  "Output"
#define TR_MULTI_TELEMETRY     "Telemetry"
#define TR_MULTI_VIDFREQ       TR("Vid. freq.", "Video frequency")
#define TR_RFPOWER       "RF Power"
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
#define TR_SYNCMENU            "Sync [MENU]"
#define TR_LIMIT               INDENT "Grenzen"
#define TR_MINRSSI             "Min. RSSI"
#define TR_LATITUDE            "Latitude"
#define TR_LONGITUDE           "Longitude"
#define TR_GPSCOORD            TR("GPS-coord.", "GPS-coordinaten format")
#define TR_VARIO               "Variometer"
#define TR_PITCH_AT_ZERO       INDENT "Laagste Toon"
#define TR_PITCH_AT_MAX        INDENT "Hoogste Toon"
#define TR_REPEAT_AT_ZERO      INDENT "Herhalen bij 0"
#define TR_SHUTDOWN            "Afsluiten"
#define TR_SAVEMODEL           "Bewaar Model-instellingen"
#define TR_BATT_CALIB          "Accu Calib"
#define TR_CURRENT_CALIB       "Stroom Calib"
#define TR_VOLTAGE             TR(INDENT "Spg", INDENT "Spanningsbron")  //9XR-Pro
#define TR_CURRENT             TR(INDENT "Stroom", INDENT "Stroombron")
#define TR_SELECT_MODEL        "Kies Model"
#define TR_SELECT_MODE                 "Select mode"
#define TR_CREATE_CATEGORY     "Create Category"
#define TR_RENAME_CATEGORY     "Rename Category"
#define TR_DELETE_CATEGORY     "Delete Category"
#define TR_CREATE_MODEL        "Nieuw Model"
#define TR_DUPLICATE_MODEL     "Dupliceer Model"
#define TR_COPY_MODEL          "Kopieer Model"
#define TR_MOVE_MODEL          "Verplaats Model"
#define TR_BACKUP_MODEL        "Backup Model"
#define TR_DELETE_MODEL        "Wis Model"
#define TR_RESTORE_MODEL       "Model Terugzetten"
#define TR_DELETE_ERROR        "Fout bij verwijderen"
#define TR_CAT_NOT_EMPTY       "Categorie is niet leeg"
#define TR_SDCARD_ERROR        "SD-Kaart fout"
#define TR_NO_SDCARD           "Geen SD-Kaart"
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
#define TR_SDCARD_FULL                 "SD-Kaart vol"
#define TR_NEEDS_FILE                  "NEEDS FILE"
#define TR_EXT_MULTI_SPEC              "opentx-inv"
#define TR_INT_MULTI_SPEC              "stm-opentx-noinv"
#define TR_INCOMPATIBLE        "Niet compatibel"
#define TR_WARNING             "MELDING"
#define TR_EEPROMWARN          "EEPROM"
#define TR_STORAGE_WARNING     "STORAGE"
#define TR_EEPROM_CONVERTING   "EEPROM Converteren"
#define TR_THROTTLEWARN        "GAS"
#define TR_ALARMSWARN          "ALARM"
#define TR_SWITCHWARN          "SCHAKELAAR"
#define TR_FAILSAFEWARN        "FAILSAFE"
#define TR_TEST_WARNING        TR("TESTING", "TEST BUILD")
#define TR_TEST_NOTSAFE        "Use for tests only"
#define TR_WRONG_SDCARDVERSION TR("Verwachte ver: ","Verwachte versie: ")
#define TR_WARN_RTC_BATTERY_LOW        "RTC Battery low"
#define TR_WARN_MULTI_LOWPOWER         "Low power mode"
#define TR_BATTERY                     "BATTERY"
#define TR_WRONG_PCBREV        "Verkeerde PCB gedetecteerd"
#define TR_EMERGENCY_MODE      "EMERGENCY MODE"
#define TR_PCBREV_ERROR        "PCB fout"
#define TR_NO_FAILSAFE         TR("Failsafe niet ing.", "Failsafe niet ingesteld")
#define TR_KEYSTUCK            "Toets klemt"
#define TR_INVERT_THR          TR("Gas omdraaien?", "Volgas achter?")
#define TR_SPEAKER_VOLUME      INDENT "Volume"
#define TR_LCD                 "LCD"
#define TR_BRIGHTNESS          INDENT "Helderheid"
#define TR_CPU_TEMP            "CPU-Temp.\016>"
#define TR_CPU_CURRENT         "Stroom\022>"
#define TR_CPU_MAH             "Verbruik"
#define TR_COPROC              "CoProc."
#define TR_COPROC_TEMP         "MB Temp. \016>"
#define TR_CAPAWARNING         INDENT "Capaciteit laag" // wg 9XR-Pro
#define TR_TEMPWARNING         INDENT "Overhitting"  //wg 9XR-Pro
#define TR_FUNC                "Funktie"
#define TR_V1                  "V1"
#define TR_V2                  "V2"
#define TR_DURATION            "Duur"
#define TR_DELAY               "Vertrag."
#define TR_SD_CARD             "SD-Card"
#define TR_SDHC_CARD           "SD-HC CARD"
#define TR_NO_SOUNDS_ON_SD     "Geen Geluiden" BREAKSPACE "op SD"
#define TR_NO_MODELS_ON_SD     "Geen Modellen" BREAKSPACE "op SD"
#define TR_NO_BITMAPS_ON_SD    "Geen Bitmaps" BREAKSPACE "op SD"
#define TR_NO_SCRIPTS_ON_SD    "Geen Scripts" BREAKSPACE "op SD"
#define TR_SCRIPT_SYNTAX_ERROR TR("Syntax error", "Script syntax error")
#define TR_SCRIPT_PANIC        "Script panic"
#define TR_SCRIPT_KILLED       "Script killed"
#define TR_SCRIPT_ERROR        "Unknown error"
#define TR_PLAY_FILE           "Speel af"
#define TR_DELETE_FILE         "Verwijderen"
#define TR_COPY_FILE           "Kopieer"
#define TR_RENAME_FILE         "Hernoemen"
#define TR_ASSIGN_BITMAP       "Bitmap toekennen"
#define TR_ASSIGN_SPLASH       "Splash screen"
#define TR_EXECUTE_FILE        "Uitvoeren"
#define TR_REMOVED             " verwijderd"
#define TR_SD_INFO             "Informatie"
#define TR_SD_FORMAT           "Format"
#define TR_NA                  "N/A"
#define TR_HARDWARE            "HARDWARE"
#define TR_FORMATTING          "Formatteren..."
#define TR_TEMP_CALIB          "Temp.  Calib."
#define TR_TIME                "Tijd:"
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
#define TR_SD_INFO_TITLE       "SD-INFO"
#define TR_SD_TYPE             "Type:"
#define TR_SD_SPEED            "Snelheid:"
#define TR_SD_SECTORS          "Sectoren:"
#define TR_SD_SIZE             "Grootte:"
#define TR_TYPE                INDENT "Type"
#define TR_GLOBAL_VARS         "Globale Variabelen"
#define TR_GVARS               "GLOBALE V."
#define TR_GLOBAL_VAR          "Globale Variabele"
#define TR_MENUGLOBALVARS      "GLOBALE VARIABELEN"
#define TR_OWN                 "Eigen"
#define TR_DATE                "Datum:"
#define TR_MONTHS              { "Jan", "Feb", "Mar", "Apr", "Mei", "Jun", "Jul", "Aug", "Sep", "Okt", "Nov", "Dec" }
#define TR_ROTARY_ENCODER      "Draaischakelaar"
#define TR_INVERT_ROTARY       "Invert Rotary"
#define TR_CHANNELS_MONITOR    "Kanaal-Monitor==>"
#define TR_MIXERS_MONITOR      "==>MIXERS MONitor"
#define TR_PATH_TOO_LONG       "Pad te Lang"
#define TR_VIEW_TEXT           "Lees Tekst"
#define TR_FLASH_BOOTLOADER      "Flash bootloader"
#define TR_FLASH_EXTERNAL_DEVICE "Flash extern Apparaat"
#define TR_FLASH_RECEIVER_OTA          "Flash receiver OTA"
#define TR_FLASH_RECEIVER_BY_EXTERNAL_MODULE_OTA "Flash RX by ext. OTA"
#define TR_FLASH_RECEIVER_BY_INTERNAL_MODULE_OTA "Flash RX by int. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_EXTERNAL_MODULE_OTA "Flash FC by ext. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_INTERNAL_MODULE_OTA "Flash FC by int. OTA"
#define TR_FLASH_BLUETOOTH_MODULE      TR("Flash BT module", "Flash Bluetooth module")
#define TR_FLASH_POWER_MANAGEMENT_UNIT          "Flash pwr mngt unit"
#define TR_CURRENT_VERSION             TR("Current vers. ", "Current version: ")
#define TR_HW_REV                      "HW Rev"
#define TR_FLASH_INTERNAL_MODULE "Flash interne XJT-Module"
#define TR_FLASH_INTERNAL_MULTI        TR("Flash Int. Multi", "Flash Internal Multi")
#define TR_FLASH_EXTERNAL_MODULE       "Flash external module"
#define TR_FLASH_EXTERNAL_MULTI        TR("Flash Ext. Multi", "Flash External Multi")
#define TR_FLASH_EXTERNAL_ELRS         TR("Flash Ext. ELRS", "Flash External ELRS")
#define TR_FIRMWARE_UPDATE_ERROR TR("FW update Error","Firmware update error")
#define TR_FIRMWARE_UPDATE_SUCCESS     "Flash successful"
#define TR_WRITING                     "Schrijven..."
#define TR_CONFIRM_FORMAT              "Formatteren bevestigen?"
#define TR_INTERNALRF                  "Interne RF"
#define TR_INTERNAL_MODULE             TR("Int. module","Internal module")
#define TR_EXTERNAL_MODULE             TR("Ext. module","External module")
#define TR_OPENTX_UPGRADE_REQUIRED     "OpenTX upgrade required"
#define TR_TELEMETRY_DISABLED          "Telem. disabled"
#define TR_MORE_OPTIONS_AVAILABLE      "More options available"
#define TR_NO_MODULE_INFORMATION       "No module information"
#define TR_EXTERNALRF          "Externe RF"
#define TR_FAILSAFE            TR(INDENT "Failsafe", INDENT "Failsafe Modus")
#define TR_FAILSAFESET         "Failsafe instellen"
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
#define TR_COUNTRYCODE         "Landcode"
#define TR_USBMODE             "USB Mode"
#define TR_JACKMODE                    "Jack Mode"
#define TR_VOICELANG           "Taal"
#define TR_UNITSSYSTEM         "Eenheden"
#define TR_EDIT                "Wijzigen"
#define TR_INSERT_BEFORE       "Invoegen ervoor"
#define TR_INSERT_AFTER        "Invoegen erna"
#define TR_COPY                "Kopieren"
#define TR_MOVE                "Verplaatsen"
#define TR_PASTE               "Plakken"
#define TR_DELETE              "Verwijderen"
#define TR_INSERT              "Invoegen"
#define TR_RESET_FLIGHT        "Reset Vliegdata"
#define TR_RESET_TIMER1        "Reset Timer1"
#define TR_RESET_TIMER2        "Reset Timer2"
#define TR_RESET_TIMER3        "Reset Timer3"
#define TR_RESET_TELEMETRY     "Reset Telemetrie"
#define TR_STATISTICS          "Statistieken"
#define TR_ABOUT_US            "De Programmeurs"
#define TR_USB_JOYSTICK        "USB Joystick (HID)"
#define TR_USB_MASS_STORAGE    "USB Storage (SD)"
#define TR_USB_SERIAL          "USB Serial (Debug)"
#define TR_USB_TELEMETRY       "USB Telem mirror"
#define TR_SETUP_SCREENS       "Setup screens"
#define TR_MONITOR_SCREENS     "Monitors"
#define TR_AND_SWITCH          "AND Switch"
#define TR_SF                  "SF"
#define TR_GF                  "GF"
#define TR_SPEAKER             INDENT "Speaker"
#define TR_BUZZER              INDENT "Zoemer"
#define TR_BYTES               "bytes"
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
#define TR_REGISTER             TR("Reg", "Register")
#define TR_MODULE_RANGE        TR("[Rng]", "[Range]")
#define TR_RECEIVER_OPTIONS            TR("REC. OPTIONS", "RECEIVER OPTIONS")
#define TR_DEL_BUTTON                  BUTTON(TR("Del", "Delete"))
#define TR_RESET_BTN           "[Reset]"
#define TR_DEBUG                       "Debug"
#define TR_KEYS_BTN                BUTTON(TR("SW","Switches"))
#define TR_ANALOGS_BTN                 BUTTON(TR("Analog","Analogs"))
#define TR_TOUCH_NOTFOUND              "Touch hardware not found"
#define TR_TOUCH_EXIT                  "Touch screen to exit"
#define TR_CALIBRATION                   "Calibration"
#define TR_SET                 "[Set]"
#define TR_TRAINER             "Trainer Poort"
#define TR_CHANS                       "Chans"
#define TR_ANTENNAPROBLEM      CENTER "TX-Antenneprobleem!"
#define TR_MODELIDUSED         TR("ID al gebruikt", "Model-ID al gebruikt")
#define TR_MODULE              "Module-Type"
#define TR_RX_NAME                     "Rx Name"
#define TR_TELEMETRY_TYPE      TR("Type", "Telemetrietype")
#define TR_TELEMETRY_SENSORS   "Sensoren"
#define TR_VALUE               "Waarde"
#define TR_TOPLCDTIMER         "Top LCD Timer"
#define TR_UNIT                "Eenheid"
#define TR_TELEMETRY_NEWSENSOR INDENT "Sensor toevoegen ..."
#define TR_CHANNELRANGE        TR(INDENT "Kanalen", INDENT "Uitgangs Kanalen")  //wg 9XR-Pro
#define TR_AFHDS3_RX_FREQ              TR("RX freq.", "RX frequency")
#define TR_AFHDS3_ONE_TO_ONE_TELEMETRY TR("Unicast/Tel.", "Unicast/Telemetry")
#define TR_AFHDS3_ONE_TO_MANY          "Multicast"
#define TR_AFHDS3_ACTUAL_POWER         TR("Act. pow", "Actual power")
#define TR_AFHDS3_POWER_SOURCE         TR("Power src.", "Power source")
#define TR_ANTENNACONFIRM1     "Antennes wisselen?"
#if defined(PCBX12S)
#define LEN_ANTENNA_MODES      "\023"
#define TR_ANTENNA_MODES       "Internal\0          ""Ask\0               ""Per model\0         ""Internal + External"
#else
#define LEN_ANTENNA_MODES      "\011"
#define TR_ANTENNA_MODES       "Internal\0""Ask\0     ""Per model""External"
#endif
#define TR_USE_INTERNAL_ANTENNA        TR("Use int. antenna", "Use internal antenna")
#define TR_USE_EXTERNAL_ANTENNA        TR("Use ext. antenna", "Use external antenna")
#define TR_ANTENNACONFIRM2     TR("Check antenna", "Is er zeker een antenne geplaatst!")
#define TR_MODULE_PROTOCOL_FLEX_WARN_LINE1                "Requires non"
#define TR_MODULE_PROTOCOL_FCC_WARN_LINE1         "Requires FCC"
#define TR_MODULE_PROTOCOL_EU_WARN_LINE1          "Requires EU"
#define TR_MODULE_PROTOCOL_WARN_LINE2                "certified firmware"
#define TR_LOWALARM            INDENT "Waarschuwing"
#define TR_CRITICALALARM       INDENT "Kritiek Alarm"
#define TR_RSSIALARM_WARN             TR("RSSI","TELEMETRY RSSI")
#define TR_NO_RSSIALARM                TR(INDENT "Alarms disabled", INDENT "Telemetry alarms disabled")
#define TR_DISABLE_ALARM               TR(INDENT "Disable alarms", INDENT "Disable telemetry alarms")
#define TR_ENABLE_POPUP        "Inschakelen Popups"
#define TR_DISABLE_POPUP       "Uitschakelen Popups"
#define TR_POPUP               "Popup"
#define TR_MIN                 "Min"
#define TR_MAX                 "Max"
#define TR_CURVE_PRESET        "Preset..."
#define TR_PRESET              "Preset"
#define TR_MIRROR              "Spiegelen"
#define TR_CLEAR               "Wissen"
#define TR_RESET               "Reset Servowaardes"
#define TR_RESET_SUBMENU       "Reset..."
#define TR_COUNT               "Punten"
#define TR_PT                  "Pt"
#define TR_PTS                 "Ptn"
#define TR_SMOOTH              "Zacht"
#define TR_COPY_STICKS_TO_OFS  TR("Cpy stick->subtrim", "Kopieer Sticks naar Subtrim")
#define TR_COPY_MIN_MAX_TO_OUTPUTS     TR("Cpy min/max to all",  "Copy min/max/center to all outputs")
#define TR_COPY_TRIMS_TO_OFS   TR("Cpy trim->subtrim", "Kopieer Trim naar Subtrim")
#define TR_INCDEC              "Inc/Decrement"
#define TR_GLOBALVAR           "Globale Var"
#define TR_MIXSOURCE           "Mixer Bron"
#define TR_CONSTANT            "Constant"
#define TR_PERSISTENT_MAH      TR(INDENT "Str mAh", INDENT "Vasthouden mAh")
#define TR_PREFLIGHT           "Preflight Checks"
#define TR_CHECKLIST           TR(INDENT "Checklist", INDENT "Toon Checklist")
#define TR_FAS_OFFSET          TR(INDENT "FAS Ofs", INDENT "FAS Offset")
#define TR_AUX_SERIAL_MODE     "Seriele poort"
#define TR_AUX2_SERIAL_MODE    "Seriele poort 2"
#define TR_SCRIPT              "Script"
#define TR_INPUTS              "Inputs"
#define TR_OUTPUTS             "Outputs"
#if defined(COLORLCD)
#define STR_EEBACKUP            "EEPROM backup"
#define STR_FACTORYRESET        "Factory reset"
#else
#define STR_EEBACKUP            TR("Backup", "EEPROM backup => SD")
#define STR_FACTORYRESET        TR("Fact. reset", "Factory reset")

#endif
#define TR_CONFIRMRESET        TR("Wis Alles?", "Wis ALLE modellen en instellingen?")
#define TR_TOO_MANY_LUA_SCRIPTS "Te veel Lua scripts!"
#define TR_SPORT_UPDATE_POWER_MODE     "SP Power"
#define LEN_SPORT_UPDATE_POWER_MODES   "\004"
#define TR_SPORT_UPDATE_POWER_MODES    "AUTO""ON\0 "
#define TR_NO_TELEMETRY_SCREENS        "No Telemetry Screens"
#define TR_TOUCH_PANEL                 "Touch panel:"

// Horus and Taranis column headers
#define TR_PHASES_HEADERS_NAME         "Name"
#define TR_PHASES_HEADERS_SW           "Switch"
#define TR_PHASES_HEADERS_RUD_TRIM     "Rudder Trim"
#define TR_PHASES_HEADERS_ELE_TRIM     "Elevator Trim"
#define TR_PHASES_HEADERS_THT_TRIM     "Throttle Trim"
#define TR_PHASES_HEADERS_AIL_TRIM     "Aileron Trim"
#define TR_PHASES_HEADERS_CH5_TRIM     "Trim 5"
#define TR_PHASES_HEADERS_CH6_TRIM     "Trim 6"
#define TR_PHASES_HEADERS_FAD_IN       "Fade In"
#define TR_PHASES_HEADERS_FAD_OUT      "Fade Out"

#define TR_LIMITS_HEADERS_NAME         "Name"
#define TR_LIMITS_HEADERS_SUBTRIM      "Subtrim"
#define TR_LIMITS_HEADERS_MIN          "Min"
#define TR_LIMITS_HEADERS_MAX          "Max"
#define TR_LIMITS_HEADERS_DIRECTION    "Richting"
#define TR_LIMITS_HEADERS_CURVE        "Curve"
#define TR_LIMITS_HEADERS_PPMCENTER    "PPM-Midden"
#define TR_LIMITS_HEADERS_SUBTRIMMODE  "Subtrim mode"

#define TR_LSW_HEADERS_FUNCTION        "Funktie"
#define TR_LSW_HEADERS_V1              "Var1"
#define TR_LSW_HEADERS_V2              "Var2"
#define TR_LSW_HEADERS_ANDSW           "EN-Schakelaar"
#define TR_LSW_HEADERS_DURATION        "Duur"
#define TR_LSW_HEADERS_DELAY           "Vertraging"

#define TR_GVAR_HEADERS_NAME          "Name"
#define TR_GVAR_HEADERS_FM0           "Waarde bij FM0"
#define TR_GVAR_HEADERS_FM1           "Waarde bij FM1"
#define TR_GVAR_HEADERS_FM2           "Waarde bij FM2"
#define TR_GVAR_HEADERS_FM3           "Waarde bij FM3"
#define TR_GVAR_HEADERS_FM4           "Waarde bij FM4"
#define TR_GVAR_HEADERS_FM5           "Waarde bij FM5"
#define TR_GVAR_HEADERS_FM6           "Waarde bij FM6"
#define TR_GVAR_HEADERS_FM7           "Waarde bij FM7"
#define TR_GVAR_HEADERS_FM8           "Waarde bij FM8"

// Horus footer descriptions
#define TR_LSW_DESCRIPTIONS    { "Vergelijking of Functie", "1e variabele", "2e variabele of constante", "2e variabele of constante", "Additionele conditie", "Minimale AAN duur van de logische schakelaar", "Minimale WAAR duur om AAN te gaan" }

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
#define TR_ABOUTUS             TR(" ABOUT ", "ABOUT")

#define TR_ABOUT_OPENTX_1      TR("OpenTX\001is\001open\001source,\001non", "OpenTX is open source, non-")
#define TR_ABOUT_OPENTX_2      TR("commercial,\001wo\001warranties.", "commercial and comes with no")
#define TR_ABOUT_OPENTX_3      TR("It\001was\001developed\001for\001free.", "warranties. It was developed")
#define TR_ABOUT_OPENTX_4      TR("Support through donations", "for free. Support through")
#define TR_ABOUT_OPENTX_5      TR("is welcome!", "donations is welcome!")

#define TR_ABOUT_BERTRAND_1    "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2    "OpenTX main author"
#define TR_ABOUT_BERTRAND_3    "Companion co-author"

#define TR_ABOUT_MIKE_1        "Mike Blandford"
#define TR_ABOUT_MIKE_2        "Code and drivers guru"
#define TR_ABOUT_MIKE_3        TR("Arguably,\001one\001of\001the\001best", "Arguably, one of the best")
#define TR_ABOUT_MIKE_4        "Inspirational"

#define TR_ABOUT_ROMOLO_1      "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2      "Companion co-author"
#define TR_ABOUT_ROMOLO_3      ""

#define TR_ABOUT_ANDRE_1       "Andre Bernet"
#define TR_ABOUT_ANDRE_2       "Functionality, usability,"
#define TR_ABOUT_ANDRE_3       "debugging, documentation"

#define TR_ABOUT_ROB_1         "Rob Thomson"
#define TR_ABOUT_ROB_2         "openrcforums webmaster"

#define TR_ABOUT_KJELL_1       "Kjell Kernen"
#define TR_ABOUT_KJELL_2       "www.open-tx.org main author"
#define TR_ABOUT_KJELL_3       "OpenTX Recorder author"
#define TR_ABOUT_KJELL_4       "Companion contributor"

#define TR_ABOUT_MARTIN_1      "Martin Hotar"
#define TR_ABOUT_MARTIN_2      "Graphics designer"

#if defined(MANUFACTURER_FRSKY)
  #define TR_ABOUT_HARDWARE_1          "FrSky"
  #define TR_ABOUT_HARDWARE_2          TR("Hardware design/producer", "Hardware designer/producer")
  #define TR_ABOUT_HARDWARE_3          "Firmware contributor"
#elif defined(MANUFACTURER_RADIOMASTER)
  #define TR_ABOUT_HARDWARE_1          "Radiomaster"
  #define TR_ABOUT_HARDWARE_2          TR("Hardware design/producer", "Hardware designer/producer")
  #define TR_ABOUT_HARDWARE_3          "Firmware contributor"
#elif defined(MANUFACTURER_JUMPER)
  #define TR_ABOUT_HARDWARE_1          "JumperRC"
  #define TR_ABOUT_HARDWARE_2          TR("Hardware design/producer", "Hardware designer/producer")
  #define TR_ABOUT_HARDWARE_3          "Firmware contributor"
#else
  #define TR_ABOUT_HARDWARE_1  "Brent Nelson"
  #define TR_ABOUT_HARDWARE_2  "Sky9x designer/producer"
  #define TR_ABOUT_HARDWARE_3  ""
#endif
#define TR_ABOUT_PARENTS_1     "Parent projects"
#define TR_ABOUT_PARENTS_2     TR("Ersky9x (Mike Blandford)", "Ersky9x (Mike Blandford)")
#define TR_ABOUT_PARENTS_3     "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4     "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT  's'
#define TR_CHR_LONG   'l'
#define TR_CHR_TOGGLE 't'
#define TR_CHR_HOUR   'h'
#define TR_CHR_INPUT  'I' // Values between A-I will work

#define TR_BEEP_VOLUME         "Beep-Volume"
#define TR_WAV_VOLUME          "Wav-Volume"
#define TR_BG_VOLUME           "Achtergr-Volume"

#define TR_TOP_BAR             "Info"
#define TR_FLASH_ERASE                 "Flash erase..."
#define TR_FLASH_WRITE                 "Flash write..."
#define TR_OTA_UPDATE                  "OTA update..."
#define TR_MODULE_RESET                "Module reset..."
#define TR_UNKNOWN_RX                  "Unknown RX"
#define TR_UNSUPPORTED_RX              "Unsupported RX"
#define TR_OTA_UPDATE_ERROR            "OTA update error"
#define TR_DEVICE_RESET                "Device reset..."
#define TR_ALTITUDE            INDENT "Hoogte"
#define TR_SCALE               "Schaal"
#define TR_VIEW_CHANNELS       "Toon Kanalen"
#define TR_VIEW_NOTES          "Toon Notities"
#define TR_MODEL_SELECT        "Model Select"
#define TR_MODS_FORBIDDEN      "Wijzigen niet toegestaan!"
#define TR_UNLOCKED            "Vrijgegeven"
#define TR_ID                  "ID"
#define TR_PRECISION           "Precisie"
#define TR_RATIO               "Ratio"
#define TR_FORMULA             "Formule"
#define TR_CELLINDEX           "Cel index"
#define TR_LOGS                "Log Data"
#define TR_OPTIONS             "Opties"
#define TR_FIRMWARE_OPTIONS    "Firmware options"

#define TR_ALTSENSOR           "Hoogte Sensor"
#define TR_CELLSENSOR          "Cellen Sensor"
#define TR_GPSSENSOR           "GPS Sensor"
#define TR_CURRENTSENSOR       "Stroomsensor"
#define TR_AUTOOFFSET          "Auto Offset"
#define TR_ONLYPOSITIVE        "Enkel positief"
#define TR_FILTER              "Filter aktief"
#define TR_TELEMETRYFULL       "Telemetrie slots vol!"
#define TR_SERVOS_OK           "Servos OK"
#define TR_SERVOS_KO           "Servos KO"
//TODO: translation
#define TR_INVERTED_SERIAL     INDENT "Invert"
#define TR_IGNORE_INSTANCE     TR(INDENT "Neg. ID ","Negeer ID's")
#define TR_DISCOVER_SENSORS    "Ontdek nieuwe sensors"
#define TR_STOP_DISCOVER_SENSORS "Stop ontdekking"
#define TR_DELETE_ALL_SENSORS  "Wis alle sensors"
#define TR_CONFIRMDELETE       "Echt alles wissen?"
#define TR_SELECT_WIDGET       "Widget kiezen"
#define TR_REMOVE_WIDGET       "Widget wissen"
#define TR_WIDGET_SETTINGS     "Widget instellen"
#define TR_REMOVE_SCREEN       "Scherm wissen"
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
#define TR_MENU_LUA            "\322Lua Scripts"
#define TR_MENU_STICKS         "\307Sticks"
#define TR_MENU_POTS           "\310Pots"
#define TR_MENU_MAX            "\315MAX"
#define TR_MENU_HELI           "\316HeliCyclic"
#define TR_MENU_TRIMS          "\313Trims"
#define TR_MENU_SWITCHES       "\312Schakelaars"
#define TR_MENU_LOGICAL_SWITCHES "\312Log. Schakelaars"
#define TR_MENU_TRAINER        "\317Trainer"
#define TR_MENU_CHANNELS       "\320Kanalen"
#define TR_MENU_GVARS          "\311GVars"
#define TR_MENU_TELEMETRY      "\321Telemetrie"
#define TR_MENU_DISPLAY        "DISPLAY"
#define TR_MENU_OTHER          "Verdere"
#define TR_MENU_INVERT         "Inverteer"
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
